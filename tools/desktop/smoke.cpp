// The desktop build's default harness: boots the firmware and runs it, idle or with --press.
// It's compiled at the end of the sketch's translation unit (see sketch_tail.h), so it can use the
// firmware's globals and functions directly. Other harnesses (--harness) are built the same way.
//
//   linnstrument-desktop [--flash-in FILE]
//                        [--provision [--restore EXPORT] [--calibration as-exported|stand-in] [--set NAME=VALUE]...]
//                        [--flash-out FILE]
//                        [--seconds S] [--midi-log FILE] [--press COL,ROW,FROM,TO,Z]...
//                        [--midi-in AT,HEXBYTES]... [--timing ADC_NS,PASS_NS]
//
// Booting with settings needs two runs, because the instrument does too:
//   1. --provision: boot on erased flash, as after a firmware update; --restore a settings export
//      the way the LinnStrument Updater does, ending included; --flash-out the result. Settings
//      without a calibration are stored as they are (--calibration as-exported, the default), or
//      given a stand-in calibration first (--calibration stand-in). The run says which happened.
//      --set changes a named setting before the settings are stored (see harness.h).
//   2. the run itself: --flash-in that image and boot normally, as after a power cycle.

#include "harness.h"

namespace smoke {

struct MidiLog {
  FILE* file = nullptr;
  uint64_t messages = 0;
  std::map<std::string, uint64_t> byType;
};

// One line per message: when the firmware wrote its first byte and when that byte started on the
// wire (ms), the bytes, and for channel messages the channel and type.
static void logMessage(MidiLog& log, const std::vector<uint8_t>& msg, uint64_t writtenNs, uint64_t wireNs) {
  uint8_t status = msg[0];
  ++log.messages;
  ++log.byType[harness::midiType(status)];
  if (log.file) {
    fprintf(log.file, "%llu.%03llu %llu.%03llu", (unsigned long long)(writtenNs / 1000000),
            (unsigned long long)(writtenNs / 1000 % 1000), (unsigned long long)(wireNs / 1000000),
            (unsigned long long)(wireNs / 1000 % 1000));
    for (uint8_t b : msg) fprintf(log.file, " %02X", b);
    if (status >= 0x80 && status < 0xF0) fprintf(log.file, "  ch%d %s", (status & 0x0F) + 1, harness::midiType(status));
    fprintf(log.file, "\n");
  }
}

// --press: a pad held from FROM to TO seconds (counted from the end of setup) at a constant pressure
// Z, the sensor's reading before the firmware applies sensitivity and bias (0-4095). It's a check
// that touches reach the note code; replaying recorded touches is a harness of its own.
struct Press { int col, row; double from, to; int z; };
static std::vector<Press> presses;
static uint64_t pressOriginNs = 0;

static uint16_t pressedSurface(uint8_t col, uint8_t row, hal::SensorAxis axis, uint64_t tNs, void* user) {
  double t = (double)(tNs - pressOriginNs) / 1e9;
  for (const Press& p : presses) {
    if (p.col == col && p.row == row && t >= p.from && t < p.to) {
      if (axis == hal::SENSOR_Z) return (uint16_t)(4095 - p.z);
      if (axis == hal::SENSOR_Y) return 2048;
      // X: the pad's centre, as the calibration measured it (calibration row = row / 3)
      int x = FXD_TO_INT(Device.calRows[col][std::min(row / 3, 3)].fxdMeasuredX);
      return (uint16_t)std::max(0, std::min(4095, x));
    }
  }
  return hal::untouchedSurface(col, row, axis, tNs, user);
}

// --midi-in: MIDI bytes the instrument receives at AT seconds (counted from the end of setup),
// written as hex, e.g. 2,B06302B0622BB00610B0260F for an NRPN; spaces are ignored.
struct MidiIn { double at; std::vector<uint8_t> bytes; };
static std::vector<MidiIn> midiIn;

static bool parseMidiIn(const char* arg, MidiIn& m) {
  char* end;
  m.at = strtod(arg, &end);
  if (end == arg || *end != ',') return false;
  std::string hex;
  for (const char* p = end + 1; *p; ++p) {
    if (isxdigit((unsigned char)*p)) hex += *p;
    else if (*p != ' ') return false;
  }
  if (hex.empty() || hex.size() % 2) return false;
  for (size_t i = 0; i < hex.size(); i += 2) m.bytes.push_back((uint8_t)strtoul(hex.substr(i, 2).c_str(), nullptr, 16));
  return true;
}

static void usage() {
  fprintf(stderr, "usage: linnstrument-desktop [--flash-in FILE] "
                  "[--provision [--restore EXPORT] [--calibration as-exported|stand-in] [--set NAME=VALUE]...] "
                  "[--flash-out FILE] [--seconds S] [--midi-log FILE] [--press COL,ROW,FROM,TO,Z]... "
                  "[--midi-in AT,HEXBYTES]... [--timing ADC_NS,PASS_NS]\n");
  exit(2);
}

}  // namespace smoke

int main(int argc, char** argv) {
  const char* flashIn = nullptr;
  const char* flashOut = nullptr;
  const char* restore = nullptr;
  const char* midiLogPath = nullptr;
  double seconds = 3.0;
  bool provision = false;
  bool standIn = false;
  std::vector<harness::Override> overrides;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--provision") { provision = true; continue; }
    if (i + 1 >= argc) smoke::usage();
    if (a == "--calibration") {
      std::string v = argv[++i];
      if (v == "stand-in") standIn = true;
      else if (v != "as-exported") smoke::usage();
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
    else if (a == "--midi-log") midiLogPath = argv[++i];
    else if (a == "--seconds") seconds = atof(argv[++i]);
    else if (a == "--press") {
      smoke::Press p;
      if (sscanf(argv[++i], "%d,%d,%lf,%lf,%d", &p.col, &p.row, &p.from, &p.to, &p.z) != 5) smoke::usage();
      smoke::presses.push_back(p);
    }
    else if (a == "--timing") {                    // the clock model's costs, see hal::Timing
      if (sscanf(argv[++i], "%u,%u", &hal::timing.adcReadNs, &hal::timing.busyWaitPassNs) != 2) smoke::usage();
    }
    else if (a == "--midi-in") {
      smoke::MidiIn m;
      if (!smoke::parseMidiIn(argv[++i], m)) smoke::usage();
      smoke::midiIn.push_back(m);
    }
    else smoke::usage();
  }
  if ((restore || !overrides.empty()) && !provision) smoke::usage();

  if (flashIn && !harness::loadFlash(flashIn)) return 1;

  smoke::MidiLog midiLog;
  if (midiLogPath) {
    midiLog.file = fopen(midiLogPath, "w");
    if (!midiLog.file) { fprintf(stderr, "cannot write %s\n", midiLogPath); return 1; }
  }
  harness::MidiParser parser;
  parser.onMessage = [&midiLog](const std::vector<uint8_t>& msg, uint64_t writtenNs, uint64_t wireNs) {
    smoke::logMessage(midiLog, msg, writtenNs, wireNs);
  };
  hal::setUartTxHook(harness::MidiParser::uartHook, &parser);

  setup();
  uint64_t bootNs = hal::nowNs();

  // A first boot on erased flash leaves the instrument in Update OS, on the calibration screen.
  if (provision && !harness::provision(restore, standIn, overrides)) return 1;

  uint64_t startNs = hal::nowNs();
  if (!smoke::presses.empty()) {
    smoke::pressOriginNs = startNs;
    hal::setSensorModel(smoke::pressedSurface, nullptr);
  }
  uint64_t endNs = startNs + (uint64_t)(seconds * 1e9);
  uint64_t loops = 0, scans = 0;
  uint64_t firstScanNs = 0, lastScanNs = 0;
  byte previousCell = cellCount;
  size_t nextMidiIn = 0;
  std::stable_sort(smoke::midiIn.begin(), smoke::midiIn.end(),
                   [](const smoke::MidiIn& a, const smoke::MidiIn& b) { return a.at < b.at; });
  while (hal::nowNs() < endNs) {
    while (nextMidiIn < smoke::midiIn.size() && hal::nowNs() >= startNs + (uint64_t)(smoke::midiIn[nextMidiIn].at * 1e9)) {
      hal::uartInject(smoke::midiIn[nextMidiIn].bytes.data(), smoke::midiIn[nextMidiIn].bytes.size());
      ++nextMidiIn;
    }
    loop();
    ++loops;
    if (cellCount < previousCell) {                   // cellCount wrapped: a surface scan finished
      if (scans == 0) firstScanNs = hal::nowNs();
      lastScanNs = hal::nowNs();
      ++scans;
    }
    previousCell = cellCount;
  }
  parser.flush();
  if (midiLog.file) fclose(midiLog.file);

  if (flashOut && !harness::saveFlash(flashOut)) return 1;

  const hal::Counters& c = hal::counters();
  printf("firmware: %s%s%s\n", OSVersion, OSVersionBuild, microLinnOSVersion);
  printf("settings version: %d.%d\n", Device.version, Device.microLinn.MLversion);
  printf("model: %d\n", LINNMODEL);
  printf("operating mode: %d, display mode: %d, serial mode: %d, calibrated: %d\n",
         (int)operatingMode, (int)displayMode, (int)Device.serialMode, (int)Device.calibrated);
  printf("boot: %.3f ms\n", bootNs / 1e6);
  printf("ran: %.3f s, %llu loop iterations, %llu surface scans\n", (hal::nowNs() - startNs) / 1e9,
         (unsigned long long)loops, (unsigned long long)scans);
  if (scans > 1) printf("scan period: %.1f us\n", (lastScanNs - firstScanNs) / 1e3 / (scans - 1));
  printf("adc reads: %llu, led spi bytes: %llu\n", (unsigned long long)c.adcReads, (unsigned long long)c.ledSpiBytes);
  printf("uart: %llu bytes at %u baud, waited %.3f ms\n", (unsigned long long)c.uartTxBytes, hal::uartBaud(),
         c.uartWaitNs / 1e6);
  printf("midi messages: %llu", (unsigned long long)midiLog.messages);
  for (auto& t : midiLog.byType) printf(", %s %llu", t.first.c_str(), (unsigned long long)t.second);
  printf("\n");
  printf("clock creep: %.3f ms\n", c.creepNs / 1e6);
  return 0;
}
