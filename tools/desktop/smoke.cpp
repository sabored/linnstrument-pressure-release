// The desktop build's default harness: boots the firmware and runs it, idle or with --press.
// It's compiled at the end of the sketch's translation unit (see sketch_tail.h), so it can use the
// firmware's globals and functions directly. Other harnesses (--harness) are built the same way.
//
//   linnstrument-desktop [--flash-in FILE] [--provision [--restore EXPORT] [--calibration as-exported|stand-in]]
//                        [--flash-out FILE]
//                        [--seconds S] [--midi-log FILE] [--press COL,ROW,FROM,TO,Z]...
//                        [--midi-in AT,HEXBYTES]... [--timing ADC_NS,PASS_NS]
//
// Booting with settings needs two runs, because the instrument does too:
//   1. --provision: boot on erased flash, as after a firmware update; --restore a settings export
//      the way the LinnStrument Updater does, ending included; --flash-out the result. Settings
//      without a calibration are stored as they are (--calibration as-exported, the default), or
//      given a stand-in calibration first (--calibration stand-in). The run says which happened.
//   2. the run itself: --flash-in that image and boot normally, as after a power cycle.

namespace smoke {

struct MidiLog {
  FILE* file = nullptr;
  std::vector<uint8_t> msg;
  uint64_t msgWrittenNs = 0, msgWireNs = 0;
  uint64_t messages = 0;
  std::map<std::string, uint64_t> byType;
};

static int midiLength(uint8_t status) {
  if (status < 0xF0) return (status & 0xE0) == 0xC0 ? 2 : 3;   // Cn and Dn have one data byte
  switch (status) {
    case 0xF1: case 0xF3: return 2;
    case 0xF2: return 3;
    case 0xF0: return 0;                                         // sysex: until 0xF7
    default: return 1;
  }
}

static const char* midiType(uint8_t status) {
  switch (status & 0xF0) {
    case 0x80: return "note-off";
    case 0x90: return "note-on";
    case 0xA0: return "poly-pressure";
    case 0xB0: return "control-change";
    case 0xC0: return "program-change";
    case 0xD0: return "channel-pressure";
    case 0xE0: return "pitch-bend";
    default: return status == 0xF0 ? "sysex" : "system";
  }
}

static void midiFlush(MidiLog& log) {
  if (log.msg.empty()) return;
  uint8_t status = log.msg[0];
  ++log.messages;
  ++log.byType[midiType(status)];
  if (log.file) {
    fprintf(log.file, "%llu.%03llu %llu.%03llu", (unsigned long long)(log.msgWrittenNs / 1000000),
            (unsigned long long)(log.msgWrittenNs / 1000 % 1000), (unsigned long long)(log.msgWireNs / 1000000),
            (unsigned long long)(log.msgWireNs / 1000 % 1000));
    for (uint8_t b : log.msg) fprintf(log.file, " %02X", b);
    if (status >= 0x80 && status < 0xF0) fprintf(log.file, "  ch%d %s", (status & 0x0F) + 1, midiType(status));
    fprintf(log.file, "\n");
  }
  log.msg.clear();
}

// Groups the UART bytes into MIDI messages. Times are in milliseconds: when the firmware wrote the
// message's first byte, and when that byte started on the wire.
static void onUartByte(uint64_t writtenNs, uint64_t wireNs, uint8_t b, void* user) {
  MidiLog& log = *(MidiLog*)user;
  if (Device.serialMode) return;                       // serial mode isn't MIDI
  if (b >= 0xF8) {                                     // real-time bytes can appear anywhere
    std::vector<uint8_t> saved;
    saved.swap(log.msg);
    log.msg.push_back(b);
    log.msgWrittenNs = writtenNs;
    log.msgWireNs = wireNs;
    midiFlush(log);
    log.msg.swap(saved);
    return;
  }
  if (b & 0x80) {
    midiFlush(log);
    log.msgWrittenNs = writtenNs;
    log.msgWireNs = wireNs;
  }
  else if (log.msg.empty()) {
    return;                                            // stray data byte
  }
  log.msg.push_back(b);
  int n = midiLength(log.msg[0]);
  if ((n > 0 && (int)log.msg.size() >= n) || (log.msg[0] == 0xF0 && b == 0xF7)) midiFlush(log);
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

static bool readFile(const char* path, std::vector<uint8_t>& out) {
  FILE* f = fopen(path, "rb");
  if (!f) return false;
  uint8_t buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.insert(out.end(), buf, buf + n);
  fclose(f);
  return true;
}

// --calibration stand-in: what completing a calibration records, over the calibration data the
// settings already hold (the firmware's defaults, for an uncalibrated export). For replays that
// should exercise the code that only runs on a calibrated instrument, such as phantom-touch rejection.
static void standInCalibration() {
  Device.calCrc = calculateCalibrationCRC();
  Device.calCrcCalculated = true;
  Device.calibrated = true;
  Device.calibrationHealed = false;
}

// What serialRestoreSettings() (ls_serial.ino) does once it has received the settings from the
// LinnStrument Updater, ending included: Update OS off, then store the settings if they carry a
// calibration, or else open the calibration screen and store nothing. Returns whether the firmware
// stored them; `applied` says whether it accepted them at all.
static bool restoreSettings(const std::vector<uint8_t>& data, bool standIn, bool& applied) {
  dueFlashStorage.write(SETTINGS_OFFSET, (byte*)data.data(), (uint32_t)data.size());
  applied = upgradeConfigurationSettings((int32_t)data.size(), dueFlashStorage.readAddress(SETTINGS_OFFSET));
  if (applied) applyConfiguration();
  if (applied && standIn && !Device.calibrated) standInCalibration();

  delayUsec(1000000);
  switchSerialMode(false);
  bool stored = false;
  if (applied && Device.calibrated) {
    setDisplayMode(displayNormal);
    clearLed(0, GLOBAL_SETTINGS_ROW);
    storeSettings();
    stored = true;
  }
  else {
    setDisplayMode(displayCalibration);
    controlButton = GLOBAL_SETTINGS_ROW;
    lightLed(0, GLOBAL_SETTINGS_ROW);
  }
  updateDisplay();
  waitingForCommands = false;
  return stored;
}

// Switching Update OS off in Global Settings (ls_settings.ino, handleGlobalSettingRelease()), which
// stores the settings; the next power-up then starts in MIDI mode.
static void updateOsOff() {
  Device.microLinn.uninstall = false;
  switchSerialMode(false);
  storeSettings();
}

static void usage() {
  fprintf(stderr, "usage: linnstrument-desktop [--flash-in FILE] "
                  "[--provision [--restore EXPORT] [--calibration as-exported|stand-in]] "
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
  if (restore && !provision) smoke::usage();

  if (flashIn) {
    std::vector<uint8_t> image;
    if (!smoke::readFile(flashIn, image) || image.size() != hal::flashSize()) {
      fprintf(stderr, "cannot read a %zu-byte flash image from %s\n", hal::flashSize(), flashIn);
      return 1;
    }
    memcpy(hal::flash(), image.data(), image.size());
  }

  smoke::MidiLog midiLog;
  if (midiLogPath) {
    midiLog.file = fopen(midiLogPath, "w");
    if (!midiLog.file) { fprintf(stderr, "cannot write %s\n", midiLogPath); return 1; }
  }
  hal::setUartTxHook(smoke::onUartByte, &midiLog);

  setup();
  uint64_t bootNs = hal::nowNs();

  if (provision) {
    // A first boot on erased flash leaves the instrument in Update OS, on the calibration screen.
    bool stored = false;
    if (restore) {
      std::vector<uint8_t> data;
      if (!smoke::readFile(restore, data)) { fprintf(stderr, "cannot read %s\n", restore); return 1; }
      bool applied = false;
      stored = smoke::restoreSettings(data, standIn, applied);
      if (!applied) {
        fprintf(stderr, "the firmware rejected the settings in %s (version %d, %zu bytes; it expects %zu)\n",
                restore, data.empty() ? -1 : data[0], data.size(), sizeof(Configuration));
        return 1;
      }
      printf("provisioning: settings restored the way the LinnStrument Updater does\n");
    }
    else {
      if (standIn) smoke::standInCalibration();
      printf("provisioning: the firmware's default settings\n");
    }
    if (stored) {
      printf("provisioning: calibrated%s, so the firmware stored the settings and switched Update OS off\n",
             standIn ? " (stand-in)" : "");
    }
    else {
      // The instrument would now wait on its calibration screen, with nothing stored. Store the
      // settings as they are, as switching Update OS off would: the flash then holds what an
      // instrument runs with once its calibration has been cleared.
      smoke::updateOsOff();
      printf("provisioning: NOT calibrated; the instrument would open its calibration screen and store "
             "nothing, so the harness switched Update OS off, which stores the settings as they are\n");
    }
  }

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
  smoke::midiFlush(midiLog);
  if (midiLog.file) fclose(midiLog.file);

  if (flashOut) {
    FILE* f = fopen(flashOut, "wb");
    if (!f || fwrite(hal::flash(), 1, hal::flashSize(), f) != hal::flashSize()) {
      fprintf(stderr, "cannot write %s\n", flashOut);
      return 1;
    }
    fclose(f);
  }

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
