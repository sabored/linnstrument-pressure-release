// The replay harness: plays a sensor recording through the firmware, with optional scripted events,
// and logs the MIDI the firmware sends and the touches it sees. Compiled at the end of the sketch's
// translation unit like every harness (see sketch_tail.h and harness.h).
//
//   linnstrument-desktop --provision [--restore EXPORT] [--calibration as-exported|stand-in]
//                        [--set NAME=VALUE]... --flash-out FILE
//   linnstrument-desktop --flash-in FILE --recording PREFIX [--events FILE]
//                        [--midi-log FILE] [--touch-log FILE] [--lead-in MS] [--tail MS]
//                        [--timing ADC_NS,PASS_NS]
//
// The first form provisions a flash image (harness.h); the second boots from it and replays.
//
// A recording is what the capture build's recorder (linnstrument_capture.py) decodes: PREFIX_samples.csv,
// one row per scan of a touched pad (plus a few scans after each release), and PREFIX_settings.csv, the
// sensor settings it was made with. Each row's time_us is the instrument's micros() at that read.
//
// The sensor model serves the recording where the firmware reads the sensor (hal::setSensorModel):
// - Time: recording time maps to replay time by a fixed offset, so that the first sample falls
//   --lead-in ms after reset (default 1000). A read at time t returns the pad's latest sample at or
//   before t, so each pad is read when the firmware's own scan reaches it.
// - Z: raw_z is readZ()'s result, after the sensitivity and the column bias, so the model returns the
//   ADC value that readZ() turns into that raw_z, using the recording's sensitivity and the firmware's
//   bias table for the recording's model. Both reads of one selection (readZ()'s settling read) get
//   the same value.
// - Velocity bursts: when the firmware reads the same pad again at once (the 8 reads at a new touch),
//   the next recorded read of the burst is served, one per read, whatever the time. Where the capture
//   dropped reads of a burst, the recorded ones are served in order and the last is held until the
//   firmware's burst ends. A burst is never skipped: the first read that reaches a burst starts it
//   from its first sample, even if that read comes after the burst's recorded time.
// - X and Y: the touch's latest recorded reading, or its first one if the finger hasn't been read yet.
// - Untouched: pads with no sample read as untouched (ADC 4095, raw 0). So do pads after the last
//   recorded scan that follows a release or rejection, and pads whose end the capture dropped: when a
//   touched pad's next sample starts a new touch (flagged, or a velocity burst), or there is none, the
//   touch ends one scan period after its last sample (at most halfway to the next touch). Other gaps
//   from dropped records hold the last value.
// - Control switches (column 0) aren't in recordings; `switch` events press them.
//
// Scripted events (--events): one per line, `TIME EVENT ARGUMENTS`, TIME in ms on the recording's clock
// (time_us / 1000), applied between loop() iterations once the clock reaches it; # starts a comment.
//   pedal  left|right down|up    a footswitch (pins 33 and 34; normally open, released at boot)
//   switch ROW MS                a control switch (column 0, row 0-7) pressed for MS ms; the firmware reads
//                                one control switch per scan, so hold it for at least 8 scans (about 60 ms)
//   midi   HEX...                MIDI bytes the instrument receives, e.g. B0 63 00
//   nrpn   PARAM VALUE [CH]      an NRPN the instrument receives (CC 99, 98, 6, 38), on channel CH (default 1);
//                                microLinn acts only on the 299 query unless `set importing 1` came first
//   set    NAME VALUE            a named setting that can change while running (harness.h, SETTINGS)
//
// Logs, with times on the recording's clock in microseconds (so they line up with the recording's time_us):
// - --midi-log: one line per MIDI message the firmware writes to the UART: the time the firmware
//   wrote its first byte, the bytes, and the message decoded.
// - --touch-log: one line per scan of a pad that is touched or has a recorded sample: the time of the
//   read, the pad, the recorded sample served (its time_us and raw_z), how it was served (scan: the
//   latest sample; burst: the next read of a velocity burst; held: a burst read whose record is
//   missing), the firmware's currentRawZ and pressureZ after the scan, the touch state after the scan,
//   and the pad's note and channel (before the scan, if this scan released the touch).
// The run's summary goes to stdout.

#include "harness.h"

namespace replay {

// ---------------------------------------------------------------- the recording

enum SampleFlags : uint8_t {
  NEW_TOUCH = 0x01, HAS_NOTE = 0x02, RELEASE = 0x04, POST_RELEASE = 0x08, SHORT_CIRCUIT = 0x10,
  BURST_START = 0x20,    // the first read of a velocity burst
  IN_BURST = 0x40,       // a read of a velocity burst, the first and last included
  INFERRED = 0x80,       // not recorded: the model's untouched reading after a touch ends (see above)
};

struct Sample {
  uint64_t ns;           // replay time
  uint64_t us;           // time_us, the recording's clock
  uint16_t rawZ;         // as recorded (after sensitivity and bias)
  uint16_t adcZ;         // the ADC value readZ() turns into rawZ
  uint16_t x, y;         // the ADC values to serve for X and Y
  uint8_t state;         // the firmware's touch state after this scan (TouchState)
  uint8_t flags;
};

struct Pad {
  std::vector<Sample> s;
  std::vector<int32_t> burstAtOrBefore;   // for each sample, the latest burst start at or before it (-1: none)
  int32_t cursor = -1;                    // latest sample at or before the current time
  int32_t served = -1;                    // last sample served
  std::vector<bool> wasServed;
};

static Pad pads[MAXCOLS][MAXROWS];
static uint64_t firstUs = 0, lastUs = 0;
static uint64_t startNs = 0;              // replay time of the first sample
static uint64_t scanPeriodUs = 0;         // the recording's typical scan period
static int recordingModel = 0, recordingSensitivity = 0;
static const uint64_t BURST_GAP_US = 2000; // reads of one burst are ~0.1 ms apart, scans 6.6 ms or more

struct RecordingStats {
  uint64_t samples = 0, newTouches = 0, bursts = 0, noteStarts = 0, inexactZ = 0;
  uint64_t endsAfterRelease = 0, endsLost = 0, touchedAtEnd = 0;
} rec;

static uint64_t toNs(uint64_t us) { return startNs + (us - firstUs) * 1000; }

static uint8_t parseState(const std::string& s) {
  if (s == "touched") return touchedCell;
  if (s == "transfer") return transferCell;
  if (s == "ignored") return ignoredCell;
  if (s == "untouched") return untouchedCell;
  return 0xFF;
}

static std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  size_t a = 0;
  while (true) {
    size_t b = line.find(',', a);
    out.push_back(line.substr(a, b == std::string::npos ? std::string::npos : b - a));
    if (b == std::string::npos) break;
    a = b + 1;
  }
  return out;
}

static bool readCsv(const std::string& path, std::vector<std::string>& header, std::vector<std::vector<std::string>>& rows) {
  FILE* f = fopen(path.c_str(), "r");
  if (!f) return false;
  std::string line;
  char buf[4096];
  bool first = true;
  while (fgets(buf, sizeof(buf), f)) {
    line = buf;
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
    if (line.empty()) continue;
    if (first) { header = splitCsv(line); first = false; }
    else rows.push_back(splitCsv(line));
  }
  fclose(f);
  return !first;
}

static int column(const std::vector<std::string>& header, const char* name, const std::string& path) {
  for (size_t i = 0; i < header.size(); ++i) {
    if (header[i] == name) return (int)i;
  }
  fprintf(stderr, "%s has no column %s\n", path.c_str(), name);
  exit(1);
}

// readZ() (ls_sensor.ino) for the recording's model and sensitivity: raw = 4095 - adc, then the
// sensitivity, then the column bias
static int readZResult(int adc, int col, int row) {
  const short (*bias)[MAXCOLS] = recordingModel == 128 ? Z_BIAS_128_SEPTEMBER2019 : Z_BIAS_200_SEPTEMBER2014;
  short raw = 4095 - adc;
  raw = raw * recordingSensitivity / 100;
  raw = (raw * Z_BIAS_MULTIPLIER) / bias[row][col];
  return (unsigned short)raw;
}

// The ADC value whose readZ() result is rawZ; the closest one if none gives it exactly
static uint16_t adcForRawZ(int rawZ, int col, int row) {
  int lo = 0, hi = 4095;                       // search r = 4095 - adc, over which readZ() only grows
  while (lo < hi) {
    int mid = (lo + hi) / 2;
    if (readZResult(4095 - mid, col, row) >= rawZ) hi = mid;
    else lo = mid + 1;
  }
  int r = lo;
  if (readZResult(4095 - r, col, row) != rawZ) {
    ++rec.inexactZ;
    if (r > 0 && rawZ - readZResult(4095 - (r - 1), col, row) < readZResult(4095 - r, col, row) - rawZ) --r;
  }
  return (uint16_t)(4095 - r);
}

static Sample inferredUntouched(uint64_t us) {
  Sample u;
  u.us = us;
  u.ns = toNs(us);
  u.rawZ = 0;
  u.adcZ = 4095;
  u.x = u.y = 2048;
  u.state = untouchedCell;
  u.flags = INFERRED;
  return u;
}

static void loadRecording(const std::string& prefix, uint64_t leadInNs) {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;

  std::string settingsPath = prefix + "_settings.csv";
  if (!readCsv(settingsPath, header, rows) || rows.empty()) {
    fprintf(stderr, "cannot read %s\n", settingsPath.c_str());
    exit(1);
  }
  int cModel = column(header, "model", settingsPath), cSens = column(header, "sensor_sensitivity_z", settingsPath);
  for (auto& r : rows) {
    int model = atoi(r[cModel].c_str()), sens = atoi(r[cSens].c_str());
    if ((recordingModel && model != recordingModel) || (recordingSensitivity && sens != recordingSensitivity)) {
      fprintf(stderr, "%s: the model or sensitivity changes during the recording\n", settingsPath.c_str());
      exit(1);
    }
    recordingModel = model;
    recordingSensitivity = sens;
  }
  if ((recordingModel != 200 && recordingModel != 128) || recordingSensitivity <= 0) {
    fprintf(stderr, "%s: unexpected model %d or sensitivity %d\n", settingsPath.c_str(), recordingModel, recordingSensitivity);
    exit(1);
  }

  std::string samplesPath = prefix + "_samples.csv";
  header.clear();
  rows.clear();
  if (!readCsv(samplesPath, header, rows) || rows.empty()) {
    fprintf(stderr, "cannot read %s\n", samplesPath.c_str());
    exit(1);
  }
  int cT = column(header, "time_us", samplesPath), cCol = column(header, "col", samplesPath),
      cRow = column(header, "row", samplesPath), cZ = column(header, "raw_z", samplesPath),
      cState = column(header, "state", samplesPath), cNew = column(header, "new_touch", samplesPath),
      cNote = column(header, "has_note", samplesPath), cRel = column(header, "release", samplesPath),
      cPost = column(header, "post_release", samplesPath), cSc = column(header, "short_circuit", samplesPath),
      cXr = column(header, "x_read", samplesPath), cX = column(header, "raw_x", samplesPath),
      cYr = column(header, "y_read", samplesPath), cY = column(header, "raw_y", samplesPath);

  firstUs = strtoull(rows.front()[cT].c_str(), nullptr, 10);
  startNs = leadInNs;
  std::vector<bool> xRead[MAXCOLS][MAXROWS], yRead[MAXCOLS][MAXROWS];
  uint64_t prevUs = 0;
  for (auto& r : rows) {
    uint64_t us = strtoull(r[cT].c_str(), nullptr, 10);
    int col = atoi(r[cCol].c_str()), row = atoi(r[cRow].c_str());
    uint8_t state = parseState(r[cState]);
    if (us < prevUs || col < 1 || col >= MAXCOLS || row < 0 || row >= MAXROWS || state == 0xFF) {
      fprintf(stderr, "%s: unexpected row at time_us %s\n", samplesPath.c_str(), r[cT].c_str());
      exit(1);
    }
    prevUs = us;
    Sample s;
    s.us = us;
    s.ns = toNs(us);
    s.rawZ = (uint16_t)atoi(r[cZ].c_str());
    s.adcZ = adcForRawZ(s.rawZ, col, row);
    s.x = (uint16_t)atoi(r[cX].c_str());
    s.y = (uint16_t)atoi(r[cY].c_str());
    s.state = state;
    s.flags = (r[cNew] == "1" ? NEW_TOUCH : 0) | (r[cNote] == "1" ? HAS_NOTE : 0) | (r[cRel] == "1" ? RELEASE : 0) |
              (r[cPost] == "1" ? POST_RELEASE : 0) | (r[cSc] == "1" ? SHORT_CIRCUIT : 0);
    pads[col][row].s.push_back(s);
    xRead[col][row].push_back(r[cXr] == "1");
    yRead[col][row].push_back(r[cYr] == "1");
    ++rec.samples;
    if (s.flags & NEW_TOUCH) ++rec.newTouches;
  }
  lastUs = prevUs;

  // the typical scan period: the median time between two reads of a pad, velocity bursts left out
  std::vector<uint64_t> gaps;
  for (auto& padColumn : pads) {
    for (Pad& p : padColumn) {
      for (size_t i = 1; i < p.s.size(); ++i) {
        if (p.s[i].us - p.s[i - 1].us > 1000) gaps.push_back(p.s[i].us - p.s[i - 1].us);
      }
    }
  }
  std::sort(gaps.begin(), gaps.end());
  scanPeriodUs = gaps.empty() ? 6560 : gaps[gaps.size() / 2];

  for (int col = 0; col < MAXCOLS; ++col) {
    for (int row = 0; row < MAXROWS; ++row) {
      Pad& p = pads[col][row];
      std::vector<Sample>& s = p.s;
      // velocity bursts, and the notes they started
      for (size_t i = 0; i < s.size(); ++i) {
        bool continues = i > 0 && (s[i - 1].flags & SHORT_CIRCUIT) && s[i].us - s[i - 1].us < BURST_GAP_US;
        if ((s[i].flags & SHORT_CIRCUIT) && !continues) {
          s[i].flags |= BURST_START | IN_BURST;
          ++rec.bursts;
        }
        if (continues) s[i].flags |= IN_BURST;
        if (continues && (s[i].flags & HAS_NOTE) && !(s[i - 1].flags & HAS_NOTE)) ++rec.noteStarts;
      }
      // X and Y: the latest reading in the touch, or its first one if it hasn't been read yet; a touch
      // starts at a flagged new touch or at a velocity burst
      size_t begin = 0;
      while (begin < s.size()) {
        size_t end = begin + 1;
        while (end < s.size() && !(s[end].flags & (NEW_TOUCH | BURST_START))) ++end;
        int firstX = -1, firstY = -1;
        for (size_t i = begin; i < end; ++i) {
          if (firstX < 0 && xRead[col][row][i]) firstX = s[i].x;
          if (firstY < 0 && yRead[col][row][i]) firstY = s[i].y;
        }
        int x = firstX < 0 ? 2048 : firstX, y = firstY < 0 ? 2048 : firstY;
        for (size_t i = begin; i < end; ++i) {
          if (xRead[col][row][i]) x = s[i].x;
          if (yRead[col][row][i]) y = s[i].y;
          s[i].x = (uint16_t)x;
          s[i].y = (uint16_t)y;
        }
        begin = end;
      }
      // where the recording shows no more scans of a pad, it becomes untouched
      std::vector<Sample> out;
      for (size_t i = 0; i < s.size(); ++i) {
        out.push_back(s[i]);
        bool last = i + 1 == s.size();
        uint64_t gap = last ? 0 : s[i + 1].us - s[i].us;
        uint64_t after = last ? scanPeriodUs : std::min(scanPeriodUs, gap / 2);
        if (s[i].state == untouchedCell) {
          // released or rejected: logged again only on the scans after a release, or at a new touch
          if (last || gap > scanPeriodUs * 16 / 10) {
            out.push_back(inferredUntouched(s[i].us + after));
            ++rec.endsAfterRelease;
          }
        }
        else if (last || (s[i + 1].flags & (NEW_TOUCH | BURST_START))) {
          // still touched, but the next sample starts a new touch (a velocity burst only comes with one,
          // even if the capture dropped the read that flagged it) or there is none: the capture dropped
          // the touch's end, or the recording stopped
          out.push_back(inferredUntouched(s[i].us + after));
          if (last) ++rec.touchedAtEnd;
          else ++rec.endsLost;
        }
      }
      s.swap(out);
      p.burstAtOrBefore.resize(s.size());
      int32_t b = -1;
      for (size_t i = 0; i < s.size(); ++i) {
        if (s[i].flags & BURST_START) b = (int32_t)i;
        p.burstAtOrBefore[i] = b;
      }
      p.wasServed.assign(s.size(), false);
    }
  }
}

// ---------------------------------------------------------------- the sensor model

enum ReadKind { READ_UNTOUCHED, READ_SCAN, READ_BURST, READ_HELD };
static const char* READ_KIND[] = {"-", "scan", "burst", "held"};

struct ModelStats {
  uint64_t zReads = 0, zSelections = 0, served[4] = {0, 0, 0, 0}, burstsStarted = 0, burstsStartedLate = 0;
  uint64_t switchReads = 0;
} model;

struct SwitchPress { int row; uint64_t fromNs, toNs; };
static std::vector<SwitchPress> switchPresses;
static const uint16_t SWITCH_PRESS_ADC = 4095 - 400;    // a press: raw 1200 with sensitivity 75 and the switches' bias

static uint64_t lastZSelection = UINT64_MAX;
static int lastZCol = -1, lastZRow = -1;
static uint16_t lastZValue = 4095;

// the scan the harness is watching: its first Z read, for the touch log
static int watchCol = -1, watchRow = -1;
static bool watchHit = false;
static uint64_t watchNs = 0;
static int32_t watchSample = -1;
static ReadKind watchKind = READ_UNTOUCHED;

static int32_t pick(Pad& p, uint64_t tNs, bool again, ReadKind& kind) {
  const std::vector<Sample>& s = p.s;
  while (p.cursor + 1 < (int32_t)s.size() && s[p.cursor + 1].ns <= tNs) ++p.cursor;
  int32_t i = p.served;
  if (again && i >= 0 && (s[i].flags & IN_BURST)) {
    // the firmware reads the pad again at once: the next read of the velocity burst
    if (i + 1 < (int32_t)s.size() && (s[i + 1].flags & IN_BURST) && !(s[i + 1].flags & BURST_START)) {
      kind = READ_BURST;
      return i + 1;
    }
    kind = READ_HELD;                           // the capture dropped reads of this burst
    return i;
  }
  int32_t k = p.cursor;
  if (k < 0) {
    kind = READ_UNTOUCHED;
    return -1;
  }
  if (k <= i) k = i;                            // never go back: a burst may have been served ahead of time
  else {
    int32_t b = p.burstAtOrBefore[k];
    if (b > i) {                                // a burst this pad hasn't served yet: start it
      ++model.burstsStarted;
      if (b != k) ++model.burstsStartedLate;
      k = b;
    }
  }
  kind = (s[k].flags & INFERRED) ? READ_UNTOUCHED : READ_SCAN;
  return k;
}

static uint16_t surface(uint8_t col, uint8_t row, hal::SensorAxis axis, uint64_t tNs, void*) {
  if (col == 0) {
    for (const SwitchPress& sp : switchPresses) {
      if (sp.row == row && tNs >= sp.fromNs && tNs < sp.toNs) {
        if (axis == hal::SENSOR_Z) ++model.switchReads;
        return axis == hal::SENSOR_Z ? SWITCH_PRESS_ADC : 2048;
      }
    }
    return hal::untouchedSurface(col, row, axis, tNs, nullptr);
  }
  if (col >= MAXCOLS || row >= MAXROWS) return hal::untouchedSurface(col, row, axis, tNs, nullptr);
  Pad& p = pads[col][row];
  if (axis != hal::SENSOR_Z) {
    if (p.served < 0 || (p.s[p.served].flags & INFERRED)) return 2048;
    return axis == hal::SENSOR_X ? p.s[p.served].x : p.s[p.served].y;
  }

  ++model.zReads;
  uint64_t selection = hal::sensorSelection();
  if (selection == lastZSelection) return lastZValue;   // readZ()'s settling read
  ++model.zSelections;
  bool again = lastZCol == col && lastZRow == row;
  lastZSelection = selection;
  lastZCol = col;
  lastZRow = row;

  ReadKind kind;
  int32_t i = pick(p, tNs, again, kind);
  ++model.served[kind];
  if (i >= 0) {
    p.served = i;
    p.wasServed[i] = true;
  }
  lastZValue = i < 0 ? 4095 : p.s[i].adcZ;
  if (col == watchCol && row == watchRow && !watchHit) {
    watchHit = true;
    watchNs = tNs;
    watchSample = i;
    watchKind = kind;
  }
  return lastZValue;
}

// ---------------------------------------------------------------- scripted events

enum EventKind { PEDAL, SWITCH, MIDI, SET };

struct Event {
  uint64_t ns;
  double ms;
  int line;
  EventKind kind;
  int pin = 0, level = 0;
  std::vector<uint8_t> bytes;
  const harness::Setting* setting = nullptr;
  int value = 0;
  std::string text;
};

static std::vector<Event> events;

[[noreturn]] static void eventError(const std::string& path, int line, const std::string& what) {
  fprintf(stderr, "%s:%d: %s\n", path.c_str(), line, what.c_str());
  exit(1);
}

static void loadEvents(const std::string& path, uint64_t bootNs) {
  FILE* f = fopen(path.c_str(), "r");
  if (!f) {
    fprintf(stderr, "cannot read %s\n", path.c_str());
    exit(1);
  }
  char buf[4096];
  int line = 0;
  while (fgets(buf, sizeof(buf), f)) {
    ++line;
    std::string text = buf;
    size_t hash = text.find('#');
    if (hash != std::string::npos) text.erase(hash);
    std::vector<std::string> w;
    for (size_t a = 0; a < text.size();) {
      while (a < text.size() && isspace((unsigned char)text[a])) ++a;
      size_t b = a;
      while (b < text.size() && !isspace((unsigned char)text[b])) ++b;
      if (b > a) w.push_back(text.substr(a, b - a));
      a = b;
    }
    if (w.empty()) continue;
    Event e;
    e.line = line;
    char* end;
    e.ms = strtod(w[0].c_str(), &end);
    if (*end || w.size() < 2) eventError(path, line, "expected TIME EVENT ARGUMENTS");
    double us = e.ms * 1000.0;
    if (us < (double)firstUs - (double)(startNs - bootNs) / 1000.0) {
      eventError(path, line, "the replay starts at " + std::to_string(((double)firstUs - (startNs - bootNs) / 1000.0) / 1000.0) +
                             " ms on the recording's clock; this event is earlier");
    }
    e.ns = (uint64_t)((double)startNs + (us - (double)firstUs) * 1000.0 + 0.5);
    std::string kind = w[1];
    for (size_t i = 1; i < w.size(); ++i) e.text += (i > 1 ? " " : "") + w[i];
    if (kind == "pedal") {
      if (w.size() != 4 || (w[2] != "left" && w[2] != "right") || (w[3] != "down" && w[3] != "up")) {
        eventError(path, line, "expected: pedal left|right down|up");
      }
      e.kind = PEDAL;
      e.pin = w[2] == "left" ? FOOT_SW_LEFT : FOOT_SW_RIGHT;
      e.level = w[3] == "down" ? LOW : HIGH;
    }
    else if (kind == "switch") {
      int row = 0, ms = 0;
      if (w.size() != 4 || !harness::parseInt(w[2], row) || !harness::parseInt(w[3], ms) || row < 0 || row >= MAXROWS || ms <= 0) {
        eventError(path, line, "expected: switch ROW(0-7) MS");
      }
      e.kind = SWITCH;
      switchPresses.push_back({row, e.ns, e.ns + (uint64_t)ms * 1000000});
    }
    else if (kind == "midi") {
      std::string hex;
      for (size_t i = 2; i < w.size(); ++i) hex += w[i];
      if (hex.empty() || hex.size() % 2 || hex.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) {
        eventError(path, line, "expected: midi HEXBYTES");
      }
      e.kind = MIDI;
      for (size_t i = 0; i < hex.size(); i += 2) e.bytes.push_back((uint8_t)strtoul(hex.substr(i, 2).c_str(), nullptr, 16));
    }
    else if (kind == "nrpn") {
      int param = 0, value = 0, ch = 1;
      if ((w.size() != 4 && w.size() != 5) || !harness::parseInt(w[2], param) || !harness::parseInt(w[3], value) ||
          (w.size() == 5 && !harness::parseInt(w[4], ch)) || param < 0 || param > 16383 || value < 0 || value > 16383 ||
          ch < 1 || ch > 16) {
        eventError(path, line, "expected: nrpn PARAM(0-16383) VALUE(0-16383) [CHANNEL(1-16)]");
      }
      e.kind = MIDI;
      uint8_t cc = (uint8_t)(0xB0 | (ch - 1));
      e.bytes = {cc, 99, (uint8_t)(param >> 7), cc, 98, (uint8_t)(param & 127),
                 cc, 6, (uint8_t)(value >> 7), cc, 38, (uint8_t)(value & 127)};
    }
    else if (kind == "set") {
      if (w.size() != 4) eventError(path, line, "expected: set NAME VALUE");
      std::string error = harness::checkSetting(w[2], w[3], e.setting, e.value);
      if (!error.empty()) eventError(path, line, error);
      if (!e.setting->running) {
        eventError(path, line, "setting " + w[2] + " can only be set when provisioning; to change it while running, "
                               "send its NRPN");
      }
      e.kind = SET;
    }
    else {
      eventError(path, line, "unknown event '" + kind + "' (pedal, switch, midi, nrpn, set)");
    }
    events.push_back(e);
  }
  fclose(f);
  std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.ns < b.ns; });
}

static void applyEvent(const Event& e) {
  switch (e.kind) {
    case PEDAL: hal::setInputPin(e.pin, e.level); break;
    case SWITCH: break;                                  // the sensor model serves it
    case MIDI: hal::uartInject(e.bytes.data(), e.bytes.size()); break;
    case SET: e.setting->set(e.value); break;
  }
}

// ---------------------------------------------------------------- logs

// the recording's clock, in microseconds with three decimals
static void printTime(FILE* f, uint64_t ns) {
  int64_t t = (int64_t)firstUs * 1000 + ((int64_t)ns - (int64_t)startNs);
  const char* sign = t < 0 ? "-" : "";
  uint64_t a = (uint64_t)(t < 0 ? -t : t);
  fprintf(f, "%s%llu.%03llu", sign, (unsigned long long)(a / 1000), (unsigned long long)(a % 1000));
}

struct MidiStats {
  uint64_t messages = 0, noteOns = 0, noteOffs = 0;
  std::map<std::string, uint64_t> byType;
} midi;

static FILE* midiLog = nullptr;

static void logMessage(const std::vector<uint8_t>& msg, uint64_t writtenNs, uint64_t) {
  uint8_t status = msg[0];
  ++midi.messages;
  ++midi.byType[harness::midiType(status)];
  if ((status & 0xF0) == 0x90 && msg.size() == 3 && msg[2] > 0) ++midi.noteOns;
  if ((status & 0xF0) == 0x80 || ((status & 0xF0) == 0x90 && msg.size() == 3 && msg[2] == 0)) ++midi.noteOffs;
  if (!midiLog) return;
  printTime(midiLog, writtenNs);
  for (uint8_t b : msg) fprintf(midiLog, " %02X", b);
  fprintf(midiLog, "  ");
  int ch = (status & 0x0F) + 1;
  int d1 = msg.size() > 1 ? msg[1] : 0, d2 = msg.size() > 2 ? msg[2] : 0;
  switch (status & 0xF0) {
    case 0x80: fprintf(midiLog, "ch%d note-off %d %d", ch, d1, d2); break;
    case 0x90: fprintf(midiLog, "ch%d note-on %d %d", ch, d1, d2); break;
    case 0xA0: fprintf(midiLog, "ch%d poly-pressure %d %d", ch, d1, d2); break;
    case 0xB0: fprintf(midiLog, "ch%d cc %d %d", ch, d1, d2); break;
    case 0xC0: fprintf(midiLog, "ch%d program %d", ch, d1); break;
    case 0xD0: fprintf(midiLog, "ch%d pressure %d", ch, d1); break;
    case 0xE0: fprintf(midiLog, "ch%d bend %d", ch, (d1 | (d2 << 7)) - 8192); break;
    default: fprintf(midiLog, "%s", harness::midiType(status)); break;
  }
  fprintf(midiLog, "\n");
}

static const char* STATE_NAME[] = {"untouched", "ignored", "transfer", "touched"};

static const char* baseName(const char* path) {
  const char* slash = strrchr(path, '/');
  return slash ? slash + 1 : path;
}

static void usage() {
  fprintf(stderr, "usage: linnstrument-desktop --provision [--restore EXPORT] [--calibration as-exported|stand-in] "
                  "[--set NAME=VALUE]... --flash-out FILE\n"
                  "       linnstrument-desktop --flash-in FILE --recording PREFIX [--events FILE] [--midi-log FILE] "
                  "[--touch-log FILE] [--lead-in MS] [--tail MS] [--timing ADC_NS,PASS_NS]\n");
  exit(2);
}

}  // namespace replay

int main(int argc, char** argv) {
  using namespace replay;
  const char* flashIn = nullptr;
  const char* flashOut = nullptr;
  const char* restore = nullptr;
  const char* recording = nullptr;
  const char* eventsPath = nullptr;
  const char* midiLogPath = nullptr;
  const char* touchLogPath = nullptr;
  double leadInMs = 1000, tailMs = 5000;
  bool provision = false, standIn = false;
  std::vector<harness::Override> overrides;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--provision") { provision = true; continue; }
    if (i + 1 >= argc) usage();
    if (a == "--calibration") {
      std::string v = argv[++i];
      if (v == "stand-in") standIn = true;
      else if (v != "as-exported") usage();
    }
    else if (a == "--flash-in") flashIn = argv[++i];
    else if (a == "--flash-out") flashOut = argv[++i];
    else if (a == "--restore") restore = argv[++i];
    else if (a == "--set") {
      harness::Override o;
      std::string error;
      if (!harness::parseOverride(argv[++i], o, error)) { fprintf(stderr, "%s\n", error.c_str()); return 2; }
      overrides.push_back(o);
    }
    else if (a == "--recording") recording = argv[++i];
    else if (a == "--events") eventsPath = argv[++i];
    else if (a == "--midi-log") midiLogPath = argv[++i];
    else if (a == "--touch-log") touchLogPath = argv[++i];
    else if (a == "--lead-in") leadInMs = atof(argv[++i]);
    else if (a == "--tail") tailMs = atof(argv[++i]);
    else if (a == "--timing") {                    // the clock model's costs, see hal::Timing
      if (sscanf(argv[++i], "%u,%u", &hal::timing.adcReadNs, &hal::timing.busyWaitPassNs) != 2) usage();
    }
    else usage();
  }

  if (provision) {
    if (flashIn || recording || eventsPath || !flashOut) usage();
    setup();
    if (!harness::provision(restore, standIn, overrides)) return 1;
    return harness::saveFlash(flashOut) ? 0 : 1;
  }
  if (!flashIn || !recording || restore || !overrides.empty() || flashOut) usage();
  if (!harness::loadFlash(flashIn)) return 1;

  harness::MidiParser parser;
  parser.onMessage = logMessage;
  hal::setUartTxHook(harness::MidiParser::uartHook, &parser);
  if (midiLogPath) {
    midiLog = fopen(midiLogPath, "w");
    if (!midiLog) { fprintf(stderr, "cannot write %s\n", midiLogPath); return 1; }
    fprintf(midiLog, "# time_us (the recording's clock, when the firmware wrote the message), bytes, decoded\n");
  }
  FILE* touchLog = nullptr;
  if (touchLogPath) {
    touchLog = fopen(touchLogPath, "w");
    if (!touchLog) { fprintf(stderr, "cannot write %s\n", touchLogPath); return 1; }
    fprintf(touchLog, "# time_us col row rec_time_us rec_raw_z read currentRawZ pressureZ state note channel\n");
  }

  setup();
  uint64_t bootNs = hal::nowNs();
  uint64_t leadInNs = (uint64_t)(leadInMs * 1e6);
  if (leadInNs <= bootNs) {
    fprintf(stderr, "--lead-in %.3f ms ends before the boot does (%.3f ms)\n", leadInMs, bootNs / 1e6);
    return 1;
  }
  loadRecording(recording, leadInNs);
  if (eventsPath) loadEvents(eventsPath, bootNs);
  hal::setSensorModel(surface, nullptr);

  uint64_t endNs = toNs(lastUs);
  for (const Event& e : events) endNs = std::max(endNs, e.ns);
  for (const SwitchPress& sp : switchPresses) endNs = std::max(endNs, sp.toNs);
  endNs += (uint64_t)(tailMs * 1e6);

  uint64_t loops = 0, scans = 0, scanStartNs = 0, scanNs = 0, touchLines = 0;
  byte previousCell = cellCount;
  size_t nextEvent = 0;
  while (hal::nowNs() < endNs) {
    while (nextEvent < events.size() && hal::nowNs() >= events[nextEvent].ns) {
      const Event& e = events[nextEvent++];
      applyEvent(e);
      printf("event at %.3f ms (line %d): %s\n", e.ms, e.line, e.text.c_str());
    }

    int col = sensorCol, row = sensorRow;
    TouchInfo& c = cell(col, row);
    TouchState before = c.touched;
    int noteBefore = c.note, channelBefore = c.channel;
    watchCol = col;
    watchRow = row;
    watchHit = false;

    loop();
    ++loops;

    watchCol = -1;
    if (touchLog && watchHit && col > 0) {
      bool recorded = watchSample >= 0 && !(pads[col][row].s[watchSample].flags & INFERRED);
      bool released = before != untouchedCell && c.touched == untouchedCell;
      if (recorded || before != untouchedCell || c.touched != untouchedCell) {
        printTime(touchLog, watchNs);
        fprintf(touchLog, " %d %d ", col, row);
        if (recorded) {
          const Sample& s = pads[col][row].s[watchSample];
          fprintf(touchLog, "%llu %d", (unsigned long long)s.us, s.rawZ);
        }
        else {
          fprintf(touchLog, "- 0");
        }
        if (released) fprintf(touchLog, " %s - -", READ_KIND[watchKind]);
        else fprintf(touchLog, " %s %d %d", READ_KIND[watchKind], (int)c.currentRawZ, (int)c.pressureZ);
        fprintf(touchLog, " %s %d %d\n", STATE_NAME[c.touched & 3], released ? noteBefore : (int)c.note,
                released ? channelBefore : (int)c.channel);
        ++touchLines;
      }
    }

    if (cellCount < previousCell) {                    // cellCount wrapped: a surface scan finished
      if (scans == 0) scanStartNs = hal::nowNs();
      scanNs = hal::nowNs();
      ++scans;
    }
    previousCell = cellCount;
  }
  parser.flush();
  if (midiLog) fclose(midiLog);
  if (touchLog) fclose(touchLog);

  uint64_t servedSamples = 0, recordedSamples = 0;
  for (auto& padColumn : pads) {
    for (Pad& p : padColumn) {
      for (size_t i = 0; i < p.s.size(); ++i) {
        if (p.s[i].flags & INFERRED) continue;
        ++recordedSamples;
        if (p.wasServed[i]) ++servedSamples;
      }
    }
  }
  const hal::Counters& hc = hal::counters();
  printf("firmware: %s%s%s, settings version %d.%d\n", OSVersion, OSVersionBuild, microLinnOSVersion, Device.version,
         Device.microLinn.MLversion);
  printf("recording: %s: model %d, sensitivity %d, %.3f s from time_us %llu, typical scan period %llu us\n", baseName(recording),
         recordingModel, recordingSensitivity, (lastUs - firstUs) / 1e6, (unsigned long long)firstUs,
         (unsigned long long)scanPeriodUs);
  printf("recording: %llu samples, %llu new touches, %llu velocity bursts, %llu notes started by a burst\n",
         (unsigned long long)rec.samples, (unsigned long long)rec.newTouches, (unsigned long long)rec.bursts,
         (unsigned long long)rec.noteStarts);
  printf("recording: touches ended by the model: %llu after their last scan, %llu whose end the capture dropped, "
         "%llu still touched when it stopped\n", (unsigned long long)rec.endsAfterRelease, (unsigned long long)rec.endsLost,
         (unsigned long long)rec.touchedAtEnd);
  printf("recording: raw_z values with no exact ADC value: %llu\n", (unsigned long long)rec.inexactZ);
  printf("replay: %.3f s, %llu loop iterations, %llu surface scans", (endNs - startNs) / 1e9, (unsigned long long)loops,
         (unsigned long long)scans);
  if (scans > 1) printf(", mean scan period %.1f us", (scanNs - scanStartNs) / 1e3 / (scans - 1));
  printf("\n");
  printf("replay: %llu of %llu recorded samples served (%.2f%%)\n", (unsigned long long)servedSamples,
         (unsigned long long)recordedSamples, recordedSamples ? 100.0 * servedSamples / recordedSamples : 0.0);
  printf("replay: Z reads served: %llu scan, %llu burst, %llu burst held (records missing), %llu untouched; "
         "%llu bursts started, %llu of them after their recorded start\n", (unsigned long long)model.served[READ_SCAN],
         (unsigned long long)model.served[READ_BURST], (unsigned long long)model.served[READ_HELD],
         (unsigned long long)model.served[READ_UNTOUCHED], (unsigned long long)model.burstsStarted,
         (unsigned long long)model.burstsStartedLate);
  printf("replay: %zu scripted events, %llu control switch reads pressed\n", events.size(), (unsigned long long)model.switchReads);
  printf("midi: %llu messages, %llu note-ons, %llu note-offs", (unsigned long long)midi.messages,
         (unsigned long long)midi.noteOns, (unsigned long long)midi.noteOffs);
  for (auto& t : midi.byType) printf(", %s %llu", t.first.c_str(), (unsigned long long)t.second);
  printf("\n");
  printf("touch log: %llu lines\n", (unsigned long long)touchLines);
  printf("uart: %llu bytes at %u baud, waited %.3f ms\n", (unsigned long long)hc.uartTxBytes, hal::uartBaud(), hc.uartWaitNs / 1e6);
  printf("clock creep: %.3f ms\n", hc.creepNs / 1e6);
  return 0;
}
