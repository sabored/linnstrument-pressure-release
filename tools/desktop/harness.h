// Shared by the desktop build's harnesses (smoke.cpp, replay.cpp). A harness is compiled at the end of
// the sketch's translation unit, after sketch_tail.h, so this code can use the firmware's globals and
// functions directly. It holds what every harness needs: grouping the UART's bytes into MIDI messages,
// reading files, named settings, and provisioning: the flash image a second run boots from, as after a
// power cycle. There are two kinds of provisioning, both starting with a boot on erased flash, as after
// a firmware update:
// - provision(): restores a settings export the way the LinnStrument Updater does, ending included.
//   Settings without a calibration are stored as they are (--calibration as-exported, the default), or
//   given a stand-in calibration first (--calibration stand-in).
// - provisionTestSettings(): keeps the firmware's own defaults and takes only the calibration from a
//   settings export, so a test's settings don't depend on what the export happens to hold.
// Both change named settings (--set NAME=VALUE, see SETTINGS below) before the settings are stored, so
// the boot that follows applies them like any stored setting.
#ifndef LINNSTRUMENT_DESKTOP_HARNESS_H
#define LINNSTRUMENT_DESKTOP_HARNESS_H

#include <functional>

namespace harness {

// ---------------------------------------------------------------- MIDI

inline int midiLength(uint8_t status) {
  if (status < 0xF0) return (status & 0xE0) == 0xC0 ? 2 : 3;   // Cn and Dn have one data byte
  switch (status) {
    case 0xF1: case 0xF3: return 2;
    case 0xF2: return 3;
    case 0xF0: return 0;                                         // sysex: until 0xF7
    default: return 1;
  }
}

inline const char* midiType(uint8_t status) {
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

// Groups the UART's bytes into MIDI messages and hands each one to onMessage, with the times (ns)
// when the firmware wrote its first byte and when that byte started on the wire. Real-time bytes
// (F8-FF) can appear inside another message and are handed over on their own. Bytes sent in serial
// mode (Update OS) aren't MIDI and are ignored.
struct MidiParser {
  std::function<void(const std::vector<uint8_t>& msg, uint64_t writtenNs, uint64_t wireNs)> onMessage;
  std::vector<uint8_t> msg;
  uint64_t msgWrittenNs = 0, msgWireNs = 0;

  void flush() {
    if (msg.empty()) return;
    if (onMessage) onMessage(msg, msgWrittenNs, msgWireNs);
    msg.clear();
  }

  void byte(uint64_t writtenNs, uint64_t wireNs, uint8_t b) {
    if (Device.serialMode) return;
    if (b >= 0xF8) {
      std::vector<uint8_t> saved;
      saved.swap(msg);
      msg.push_back(b);
      msgWrittenNs = writtenNs;
      msgWireNs = wireNs;
      flush();
      msg.swap(saved);
      return;
    }
    if (b & 0x80) {
      flush();
      msgWrittenNs = writtenNs;
      msgWireNs = wireNs;
    }
    else if (msg.empty()) {
      return;                                          // stray data byte
    }
    msg.push_back(b);
    int n = midiLength(msg[0]);
    if ((n > 0 && (int)msg.size() >= n) || (msg[0] == 0xF0 && b == 0xF7)) flush();
  }

  static void uartHook(uint64_t writtenNs, uint64_t wireNs, uint8_t b, void* user) {
    ((MidiParser*)user)->byte(writtenNs, wireNs, b);
  }
};

// ---------------------------------------------------------------- files and arguments

inline bool readFile(const char* path, std::vector<uint8_t>& out) {
  FILE* f = fopen(path, "rb");
  if (!f) return false;
  uint8_t buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.insert(out.end(), buf, buf + n);
  fclose(f);
  return true;
}

inline bool parseInt(const std::string& s, int& out) {
  if (s.empty()) return false;
  char* end;
  long v = strtol(s.c_str(), &end, 10);
  if (*end) return false;
  out = (int)v;
  return true;
}

// ---------------------------------------------------------------- named settings

// Settings a harness can change by name, with NAME=VALUE (--set) or NAME VALUE (a replay's events).
// Those marked `stored` are part of the stored settings: provisioning sets them before the settings are
// stored, so the boot that follows applies them as it applies any stored setting, and `get` reads back
// the value in effect, so a run can check after boot that it got what it asked for. Those marked
// `running` can be changed while the firmware runs. A value is a number in [min, max], or one of the
// setting's names or their numbers. Per-split settings apply to the left split, which covers the whole surface while
// the split is off. To change most settings while running, send the firmware their NRPN instead.
struct NamedValue { const char* name; int value; };

struct Setting {
  const char* name;
  int min, max;
  const NamedValue* names;         // nullptr-terminated, or nullptr
  bool stored, running;
  void (*set)(int value);
  int (*get)();
  const char* what;
};

static const NamedValue OFF_NAMES[] = {{"off", 4}, {nullptr, 0}};
static const NamedValue MIDI_MODE_NAMES[] = {{"oneChannel", 0}, {"channelPerNote", 1}, {"channelPerRow", 2}, {nullptr, 0}};
static const NamedValue PRESSURE_NAMES[] = {{"low", 0}, {"medium", 1}, {"high", 2}, {nullptr, 0}};
static const NamedValue VELOCITY_NAMES[] = {{"low", 0}, {"medium", 1}, {"high", 2}, {"fixed", 3}, {nullptr, 0}};
static const NamedValue HAMMER_ON_NAMES[] = {{"off", 0}, {"R", 1}, {"L", 2}, {"R+L", 3}, {nullptr, 0}};
static const NamedValue ASSIGNMENT_NAMES[] = {
  {"octaveDown", ASSIGNED_OCTAVE_DOWN}, {"octaveUp", ASSIGNED_OCTAVE_UP}, {"sustain", ASSIGNED_SUSTAIN},
  {"cc65", ASSIGNED_CC_65}, {"arpeggiator", ASSIGNED_ARPEGGIATOR}, {"altSplit", ASSIGNED_ALTSPLIT},
  {"autoOctave", ASSIGNED_AUTO_OCTAVE}, {"tapTempo", ASSIGNED_TAP_TEMPO}, {"legato", ASSIGNED_LEGATO},
  {"latch", ASSIGNED_LATCH}, {"presetUp", ASSIGNED_PRESET_UP}, {"presetDown", ASSIGNED_PRESET_DOWN},
  {"reversePitchX", ASSIGNED_REVERSE_PITCH_X}, {"sequencerPlay", ASSIGNED_SEQUENCER_PLAY},
  {"sequencerPrev", ASSIGNED_SEQUENCER_PREV}, {"sequencerNext", ASSIGNED_SEQUENCER_NEXT},
  {"midiClock", ASSIGNED_STANDALONE_MIDI_CLOCK}, {"sequencerMute", ASSIGNED_SEQUENCER_MUTE},
  {"transposeDown", ASSIGNED_TRANSPOSE_DOWN}, {"transposeUp", ASSIGNED_TRANSPOSE_UP},
  {"microLinnOctaveUp", ASSIGNED_MICROLINN_8VE_UP}, {"microLinnOctaveDown", ASSIGNED_MICROLINN_8VE_DOWN},
  {"microLinnPrevPreset", ASSIGNED_MICROLINN_PREV_PRESET}, {"microLinnPrevMemory", ASSIGNED_MICROLINN_PREV_MEMORY},
  {"microLinnPrevScale", ASSIGNED_MICROLINN_PREV_SCALE}, {"edoUp", ASSIGNED_MICROLINN_EDO_UP},
  {"edoDown", ASSIGNED_MICROLINN_EDO_DOWN}, {"disabled", ASSIGNED_DISABLED}, {nullptr, 0}};

// a switch's assignment, as the Global Foot/Switch Assignment NRPNs (228-231) set it
template <int SWITCH> void setAssignment(int v) {
  Global.switchAssignment[SWITCH] = v;
  if (v >= ASSIGNED_TAP_TEMPO && v != ASSIGNED_DISABLED) Global.customSwitchAssignment[SWITCH] = v;
}
template <int SWITCH> int getAssignment() { return Global.switchAssignment[SWITCH]; }
template <int SWITCH> void setSustainCC(int v) { Global.ccForSwitchSustain[SWITCH] = v; }
template <int SWITCH> int getSustainCC() { return Global.ccForSwitchSustain[SWITCH]; }

inline int mpePolyphony() {
  if (!Split[LEFT].mpe) return 0;
  int n = 0;
  for (int c = 0; c < 16; ++c) n += Split[LEFT].midiChanSet[c] ? 1 : 0;
  return n;
}

#define HARNESS_SWITCH(NAME, SWITCH)                                                                        \
  {NAME, 0, MAX_ASSIGNED, ASSIGNMENT_NAMES, true, false, setAssignment<SWITCH>, getAssignment<SWITCH>,        \
   "the switch's assignment (Global Settings), e.g. sustain, transposeUp, transposeDown"},                    \
  {NAME ".sustainCC", 0, 127, nullptr, true, false, setSustainCC<SWITCH>, getSustainCC<SWITCH>,             \
   "the CC the switch sends when assigned to Sustain (holding SUSTAIN in Global Settings)"}

static const Setting SETTINGS[] = {
  {"edo", 4, 55, OFF_NAMES, true, false, [](int v) { Global.microLinn.EDO = v; },
   []() -> int { return Global.microLinn.EDO; }, "microLinn's EDO; off (4) for none"},
  {"rowOffset", 0, 127, nullptr, true, false, [](int v) { Global.rowOffset = v; },
   []() -> int { return Global.rowOffset; }, "the row offset (Global Settings), in semitones"},
  {"colOffset", 1, 10, nullptr, true, false, [](int v) { Split[LEFT].microLinn.colOffset = v; },
   []() -> int { return Split[LEFT].microLinn.colOffset; }, "microLinn's column offset; 1 is OFF"},
  {"mpe", 0, 15, nullptr, true, false,
   [](int v) { if (v) enableMpe(LEFT, 1, v); else disableMpe(LEFT); }, mpePolyphony,
   "MPE with main channel 1 and this many note channels from channel 2, as the firmware sets it up "
   "(enableMpe(): channel per note, bend range 48, Y as CC 74, Z as channel pressure); 0 turns MPE off"},
  {"midiMode", 0, 2, MIDI_MODE_NAMES, true, false,
   [](int v) { Split[LEFT].midiMode = v; if (v != channelPerNote) disableMpe(LEFT); },
   []() -> int { return Split[LEFT].midiMode; }, "the MIDI mode, as the per-split settings set it"},
  {"hammerOnMode", 0, 3, HAMMER_ON_NAMES, true, false, [](int v) { Split[LEFT].microLinn.setHammerOnMode(v); },
   []() -> int { return Split[LEFT].microLinn.hammerOnMode(); }, "microLinn's hammer-ons"},
  {"hammerOnZone", 1, 121, nullptr, true, false, [](int v) { Split[LEFT].microLinn.hammerOnZone = v; },
   []() -> int { return Split[LEFT].microLinn.hammerOnZone; }, "microLinn's hammer-on zone, in tens of cents; 121 is ALL"},
  {"hammerOnWait", 0, 50, nullptr, true, false, [](int v) { Split[LEFT].microLinn.hammerOnWait = v; },
   []() -> int { return Split[LEFT].microLinn.hammerOnWait; }, "microLinn's hammer-on wait, in tens of milliseconds"},
  {"pressureSensitivity", 0, 2, PRESSURE_NAMES, true, false,
   [](int v) { Global.pressureSensitivity = (PressureSensitivity)v; }, []() -> int { return Global.pressureSensitivity; },
   "pressure sensitivity (Global Settings)"},
  {"velocitySensitivity", 0, 3, VELOCITY_NAMES, true, false,
   [](int v) { Global.velocitySensitivity = (VelocitySensitivity)v; }, []() -> int { return Global.velocitySensitivity; },
   "velocity sensitivity (Global Settings)"},
  {"pressureAftertouch", 0, 1, nullptr, true, false, [](int v) { Global.pressureAftertouch = v; },
   []() -> int { return Global.pressureAftertouch; }, "pressure only in the top of the range"},
  {"sensorSensitivityZ", 1, 255, nullptr, true, false, [](int v) { Device.sensorSensitivityZ = v; },
   []() -> int { return Device.sensorSensitivityZ; }, "the sensor's pressure scaling, in percent"},
  {"sensorLoZ", 0, 4095, nullptr, true, false, [](int v) { Device.sensorLoZ = v; },
   []() -> int { return Device.sensorLoZ; }, "raw pressure that starts a touch"},
  {"sensorFeatherZ", 0, 4095, nullptr, true, false, [](int v) { Device.sensorFeatherZ = v; },
   []() -> int { return Device.sensorFeatherZ; }, "raw pressure that continues a touch"},
  {"sensorRangeZ", 0, 4095, nullptr, true, false, [](int v) { Device.sensorRangeZ = v; },
   []() -> int { return Device.sensorRangeZ; }, "the sensor's pressure range"},
  HARNESS_SWITCH("footLeft", SWITCH_FOOT_L),
  HARNESS_SWITCH("footRight", SWITCH_FOOT_R),
  HARNESS_SWITCH("footBoth", SWITCH_FOOT_B),
  HARNESS_SWITCH("switch1", SWITCH_SWITCH_1),
  HARNESS_SWITCH("switch2", SWITCH_SWITCH_2),
  {"importing", 0, 1, nullptr, false, true, [](int v) { microLinnImportingOn = v; }, nullptr,
   "microLinn's NRPN importing (IMP on its settings screen): the firmware acts on NRPNs other than the 299 "
   "query only while it's on; not stored, and v0.1.0 switches it on at every boot (microLinn's debug "
   "preferences, microLinnSetupKitesPersonalPrefs())"},
};

#undef HARNESS_SWITCH

inline const Setting* findSetting(const std::string& name) {
  for (const Setting& s : SETTINGS) {
    if (name == s.name) return &s;
  }
  return nullptr;
}

// Checks a setting's name and value; returns an error message, or "" if both are good.
inline std::string checkSetting(const std::string& name, const std::string& value, const Setting*& setting, int& v) {
  setting = findSetting(name);
  if (!setting) {
    std::string known;
    for (const Setting& s : SETTINGS) known += std::string(known.empty() ? "" : ", ") + s.name;
    return "unknown setting '" + name + "' (known: " + known + ")";
  }
  for (const NamedValue* n = setting->names; n && n->name; ++n) {
    if (value == n->name) {
      v = n->value;
      return "";
    }
  }
  bool named = false;
  if (parseInt(value, v)) {
    for (const NamedValue* n = setting->names; n && n->name; ++n) named |= v == n->value;
  }
  if (!parseInt(value, v) || ((v < setting->min || v > setting->max) && !named)) {
    std::string names;
    for (const NamedValue* n = setting->names; n && n->name; ++n) names += std::string(", ") + n->name;
    return "setting " + name + ": '" + value + "' is not a number from " + std::to_string(setting->min) + " to " +
           std::to_string(setting->max) + names;
  }
  return "";
}

struct Override { const Setting* setting; int value; };

inline bool parseOverride(const char* arg, Override& o, std::string& error) {
  std::string a = arg;
  size_t eq = a.find('=');
  if (eq == std::string::npos) {
    error = "--set takes NAME=VALUE, not '" + a + "'";
    return false;
  }
  error = checkSetting(a.substr(0, eq), a.substr(eq + 1), o.setting, o.value);
  if (!error.empty()) return false;
  if (!o.setting->stored) {
    error = "setting " + std::string(o.setting->name) + " isn't stored, so it can't be set before the firmware boots";
    return false;
  }
  return true;
}

// The values in effect of the settings in `asked`, read back after they have all been set, once per
// setting, in order. A setting can end up with another value than the one asked for when a later one
// changes it, as the firmware's own menus do (midiMode=oneChannel turns MPE off).
inline std::vector<Override> settingsInEffect(const std::vector<Override>& asked) {
  std::vector<Override> out;
  for (const Override& o : asked) {
    bool seen = false;
    for (const Override& e : out) seen |= e.setting == o.setting;
    if (!seen && o.setting->get) out.push_back({o.setting, o.setting->get()});
  }
  return out;
}

// After boot: every setting in `expected` must have that value. Returns the mismatches.
inline std::vector<std::string> checkSettings(const std::vector<Override>& expected) {
  std::vector<std::string> wrong;
  for (const Override& o : expected) {
    if (o.setting->get && o.setting->get() != o.value) {
      wrong.push_back(std::string(o.setting->name) + " is " + std::to_string(o.setting->get()) + ", not " +
                      std::to_string(o.value));
    }
  }
  return wrong;
}

// ---------------------------------------------------------------- provisioning

// --calibration stand-in: what completing a calibration records, over the calibration data the
// settings already hold (the firmware's defaults, for an uncalibrated export). For runs that should
// exercise the code that only runs on a calibrated instrument, such as phantom-touch rejection.
inline void standInCalibration() {
  Device.calCrc = calculateCalibrationCRC();
  Device.calCrcCalculated = true;
  Device.calibrated = true;
  Device.calibrationHealed = false;
}

inline void applyOverrides(const std::vector<Override>& overrides) {
  for (const Override& o : overrides) {
    o.setting->set(o.value);
    printf("provisioning: set %s = %d\n", o.setting->name, o.value);
  }
}

// What serialRestoreSettings() (ls_serial.ino) does once it has received the settings from the
// LinnStrument Updater, ending included: Update OS off, then store the settings if they carry a
// calibration, or else open the calibration screen and store nothing. Returns whether the firmware
// stored them; `applied` says whether it accepted them at all.
inline bool restoreSettings(const std::vector<uint8_t>& data, bool standIn, const std::vector<Override>& overrides,
                            bool& applied) {
  dueFlashStorage.write(SETTINGS_OFFSET, (byte*)data.data(), (uint32_t)data.size());
  applied = upgradeConfigurationSettings((int32_t)data.size(), dueFlashStorage.readAddress(SETTINGS_OFFSET));
  if (applied) applyConfiguration();
  if (applied) applyOverrides(overrides);
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
inline void updateOsOff() {
  Device.microLinn.uninstall = false;
  switchSerialMode(false);
  storeSettings();
}

// Runs right after setup() on erased flash, which leaves the instrument in Update OS, on the
// calibration screen. Restores `restore` (a settings export) if given, else keeps the firmware's
// defaults, and makes sure the settings end up stored. Returns false if the firmware rejected them.
inline bool provision(const char* restore, bool standIn, const std::vector<Override>& overrides) {
  bool stored = false;
  if (restore) {
    std::vector<uint8_t> data;
    if (!readFile(restore, data)) {
      fprintf(stderr, "cannot read %s\n", restore);
      return false;
    }
    bool applied = false;
    stored = restoreSettings(data, standIn, overrides, applied);
    if (!applied) {
      fprintf(stderr, "the firmware rejected the settings in %s (version %d, %zu bytes; it expects %zu)\n",
              restore, data.empty() ? -1 : data[0], data.size(), sizeof(Configuration));
      return false;
    }
    printf("provisioning: settings restored the way the LinnStrument Updater does\n");
  }
  else {
    applyOverrides(overrides);
    if (standIn) standInCalibration();
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
    updateOsOff();
    printf("provisioning: NOT calibrated; the instrument would open its calibration screen and store "
           "nothing, so the harness switched Update OS off, which stores the settings as they are\n");
  }
  return true;
}

// The replay's provisioning, right after setup() on erased flash, which leaves the firmware's own
// defaults in the settings: keeps those defaults and takes only the calibration from `calibrationFrom`
// (a settings export, read through the firmware's own migration of older settings versions). With
// `otherCalibration`, the calibration state is the opposite of the export's: a calibration is cleared
// back to the firmware's defaults, and an uncalibrated export's data is marked valid, as a calibration
// would mark it, which turns on the code only a calibrated instrument runs (phantom-touch rejection).
// Then the named settings, in order, and the settings are stored with Update OS off. `inEffect` gets
// the values the named settings have once all of them are set (settingsInEffect()).
inline bool provisionTestSettings(const char* calibrationFrom, bool otherCalibration, const std::vector<Override>& overrides,
                                  std::vector<Override>& inEffect) {
  static Configuration defaults;
  memcpy(&defaults, &config, sizeof(Configuration));
  const char* source = "the firmware's defaults";
  if (calibrationFrom) {
    std::vector<uint8_t> data;
    if (!readFile(calibrationFrom, data)) {
      fprintf(stderr, "cannot read %s\n", calibrationFrom);
      return false;
    }
    dueFlashStorage.write(SETTINGS_OFFSET, (byte*)data.data(), (uint32_t)data.size());
    if (!upgradeConfigurationSettings((int32_t)data.size(), dueFlashStorage.readAddress(SETTINGS_OFFSET))) {
      fprintf(stderr, "the firmware rejected the settings in %s (version %d, %zu bytes; it expects %zu)\n",
              calibrationFrom, data.empty() ? -1 : data[0], data.size(), sizeof(Configuration));
      return false;
    }
    memcpy(defaults.device.calRows, Device.calRows, sizeof(Device.calRows));
    memcpy(defaults.device.calCols, Device.calCols, sizeof(Device.calCols));
    defaults.device.calCrc = Device.calCrc;
    defaults.device.calCrcCalculated = Device.calCrcCalculated;
    defaults.device.calibrated = Device.calibrated;
    defaults.device.calibrationHealed = Device.calibrationHealed;
    source = "the settings export";
  }
  memcpy(&config, &defaults, sizeof(Configuration));
  applyConfiguration();
  printf("provisioning: the firmware's default settings, with the calibration of %s (%s)\n", source,
         Device.calibrated ? "calibrated" : "not calibrated");
  if (otherCalibration) {
    if (Device.calibrated) {
      initializeCalibrationData();
      printf("provisioning: calibration cleared, as on an instrument whose calibration was reset\n");
    }
    else {
      standInCalibration();
      printf("provisioning: calibration data marked valid, as a calibration would mark it (stand-in)\n");
    }
  }
  applyOverrides(overrides);
  inEffect = settingsInEffect(overrides);
  for (const Override& e : inEffect) {
    for (auto o = overrides.rbegin(); o != overrides.rend(); ++o) {
      if (o->setting != e.setting) continue;
      if (o->value != e.value) {
        printf("provisioning: %s is %d after the settings that follow it (it was set to %d)\n", e.setting->name,
               e.value, o->value);
      }
      break;
    }
  }
  updateOsOff();
  return true;
}

inline bool loadFlash(const char* path) {
  std::vector<uint8_t> image;
  if (!readFile(path, image) || image.size() != hal::flashSize()) {
    fprintf(stderr, "cannot read a %zu-byte flash image from %s\n", hal::flashSize(), path);
    return false;
  }
  memcpy(hal::flash(), image.data(), image.size());
  return true;
}

inline bool saveFlash(const char* path) {
  FILE* f = fopen(path, "wb");
  if (!f || fwrite(hal::flash(), 1, hal::flashSize(), f) != hal::flashSize()) {
    fprintf(stderr, "cannot write %s\n", path);
    if (f) fclose(f);
    return false;
  }
  fclose(f);
  return true;
}

}  // namespace harness

#endif
