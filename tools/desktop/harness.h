// Shared by the desktop build's harnesses (smoke.cpp, replay.cpp). A harness is compiled at the end of
// the sketch's translation unit, after sketch_tail.h, so this code can use the firmware's globals and
// functions directly. It holds what every harness needs: grouping the UART's bytes into MIDI messages,
// reading files, and provisioning, which is what the instrument goes through after a firmware update.
//
// Provisioning (--provision) boots on erased flash, as after a firmware update, restores a settings
// export the way the LinnStrument Updater does, ending included, and leaves the flash image for a
// second run to boot from, as after a power cycle. Settings without a calibration are stored as they
// are (--calibration as-exported, the default), or given a stand-in calibration first (--calibration
// stand-in). Named settings (--set NAME=VALUE, see SETTINGS below) are changed before the settings are
// stored, so the boot that follows applies them like any stored setting.
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

// Settings a harness can change by name. Those marked `stored` are part of the stored settings:
// provisioning sets them before the settings are stored, so the boot that follows applies them as it
// applies any stored setting. Those marked `running` can be changed while the firmware runs (a replay's
// `set` event) and take effect as the firmware's own code would make them. To change most settings
// while running, prefer sending the firmware its NRPN (with `importing` on), which runs the firmware's
// own code. Per-split settings apply to both splits.
struct Setting {
  const char* name;
  int min, max;
  bool stored, running;
  void (*set)(int value);
  const char* what;
};

static const Setting SETTINGS[] = {
  {"edo", 4, 55, true, false, [](int v) { Global.microLinn.EDO = v; },
   "microLinn's EDO; 4 is OFF"},
  {"colOffset", 1, 10, true, false, [](int v) { for (int s = 0; s < NUMSPLITS; ++s) Split[s].microLinn.colOffset = v; },
   "microLinn's column offset; 1 is OFF"},
  {"midiMode", 0, 2, true, false, [](int v) { for (int s = 0; s < NUMSPLITS; ++s) Split[s].midiMode = v; },
   "MIDI mode: 0 one channel, 1 channel per note, 2 channel per row"},
  {"hammerOnMode", 0, 3, true, false, [](int v) { for (int s = 0; s < NUMSPLITS; ++s) Split[s].microLinn.setHammerOnMode(v); },
   "microLinn's hammer-ons: 0 OFF, 1 R, 2 L, 3 R+L"},
  {"hammerOnZone", 1, 121, true, false, [](int v) { for (int s = 0; s < NUMSPLITS; ++s) Split[s].microLinn.hammerOnZone = v; },
   "microLinn's hammer-on zone, in tens of cents; 121 is ALL"},
  {"hammerOnWait", 0, 50, true, false, [](int v) { for (int s = 0; s < NUMSPLITS; ++s) Split[s].microLinn.hammerOnWait = v; },
   "microLinn's hammer-on wait, in tens of milliseconds"},
  {"pressureSensitivity", 0, 2, true, false, [](int v) { Global.pressureSensitivity = (PressureSensitivity)v; },
   "0 low, 1 medium, 2 high"},
  {"velocitySensitivity", 0, 3, true, false, [](int v) { Global.velocitySensitivity = (VelocitySensitivity)v; },
   "0 low, 1 medium, 2 high, 3 fixed"},
  {"pressureAftertouch", 0, 1, true, false, [](int v) { Global.pressureAftertouch = v; },
   "pressure only in the top of the range"},
  {"sensorSensitivityZ", 1, 255, true, false, [](int v) { Device.sensorSensitivityZ = v; },
   "the sensor's pressure scaling, in percent"},
  {"sensorLoZ", 0, 4095, true, false, [](int v) { Device.sensorLoZ = v; },
   "raw pressure that starts a touch"},
  {"sensorFeatherZ", 0, 4095, true, false, [](int v) { Device.sensorFeatherZ = v; },
   "raw pressure that continues a touch"},
  {"sensorRangeZ", 0, 4095, true, false, [](int v) { Device.sensorRangeZ = v; },
   "the sensor's pressure range"},
  {"importing", 0, 1, false, true, [](int v) { microLinnImportingOn = v; },
   "microLinn's NRPN importing (Global Settings > microLinn > IMP), which the firmware needs to act on "
   "any NRPN but the 299 query; not stored"},
};

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
  if (!parseInt(value, v) || v < setting->min || v > setting->max) {
    return "setting " + name + ": '" + value + "' is not a number from " + std::to_string(setting->min) + " to " +
           std::to_string(setting->max);
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
    error = "setting " + std::string(o.setting->name) + " isn't stored, so it can't be set when provisioning";
    return false;
  }
  return true;
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
