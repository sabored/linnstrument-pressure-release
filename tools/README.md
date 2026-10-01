# Firmware tools

Build, measurement and desktop-build tools for the firmware.

Everything runs through one script, `tools/fw.py`, from anywhere in the repository. It needs Python 3
(standard library only), git, and for the desktop build a C++ compiler (clang on macOS). Results go to
`build/fw/`, which git ignores.

## Once per computer: the reference toolchain

```bash
python3 tools/fw.py setup
python3 tools/fw.py verify
```

`setup` downloads Arduino IDE 1.8.19 (for its `arduino-builder`), the Arduino SAM core 1.6.11 and
arm-none-eabi-gcc 4.8.3-2014q1 from downloads.arduino.cc, checks each against its published checksum,
and unpacks them into `../toolchain` next to the repository (1.1 GB, of which the downloaded archives
in `downloads/` are 242 MB and can be deleted). Nothing is installed system-wide; delete the folder to
remove it. `--toolchain DIR` or `LINNSTRUMENT_TOOLCHAIN` chooses another folder. On an
Apple Silicon Mac these x86_64 tools run under Rosetta 2. Linux x86_64 works too.

`verify` rebuilds the tag v0.1.0 from scratch, with nothing cached, and checks that the `.bin` is
byte-identical to the release (SHA-256 `0587f532…bf0`). If it isn't, the toolchain isn't the
reference one: don't release from it.

Cached results and build folders are keyed by a fingerprint of the toolchain's files (path, size and
modification time of each), so a different or updated toolchain never reuses them.

## Every change

```bash
python3 tools/fw.py report --ram-budget N
```

Builds the working tree and the previous build (the merge-base of HEAD with `fork/main`; `--against
REF` picks another). It prints static RAM, the stack figures, `.bin` size and flash left, before and
after, plus any warning that is new. N is the most static RAM the change may add. It exits with an
error if:
- a warning is new;
- the program would overlap the settings;
- the RAM left after the stack without re-entry (the row "RAM left after that stack") is under 256
  bytes;
- static RAM grew by more than N bytes;
- the stack analysis needs attention (an excluded call it couldn't find, or its nesting bound was
  reached).

The stack figures are static and conservative, and there is no static worst case, because nothing
but a few flags bounds how deeply the firmware re-enters its schedulers. The top of `fwlib/stack.py`
describes the model and its assumptions.

```bash
python3 tools/fw.py replay --recordings DIR
```

Plays the sensor recordings in DIR through both builds, in every test configuration, and checks that
the MIDI is identical (see Replays below). A change that must not change what the instrument sends
has to pass it. Add `--repeat --sanitize` to also check that the replay is repeatable and free of
memory errors.

## Commands

| Command | What it does |
|---|---|
| `build [--ref REF]` | The reference build: `.bin` path, SHA-256, size, where the program ends, static RAM |
| `verify` | Rebuilds v0.1.0 from scratch and checks the released SHA-256 |
| `warnings [--ref REF] [--against BASE] [--list]` | The `-Wall -Wextra` build's warnings that are new or gone compared with BASE |
| `stack [--ref REF] [-v]` | Static stack depth without re-entry, a table with re-entry, and the stack below a list of watched functions (the touch, note and MIDI paths); the deepest path with `-v` |
| `report [--ref REF] [--against BASE] [--ram-budget N]` | All of the above, before and after |
| `desktop [--ref REF] [--run] [--settings EXPORT] [--calibration as-exported\|stand-in] [--seconds S] [--midi-log FILE] [--harness FILE]` | Builds the whole sketch for this computer; `--run` boots it and runs it |
| `layout [--ref REF] [-v]` | Checks that the desktop build lays out every struct as the instrument does; fails on any difference not explained by a pointer, and on a struct missing from either build |
| `replay [--ref REF] [--against BASE \| --baseline DIR \| --no-compare] --recordings DIR [--config NAME]... [--recording NAME]... [--repeat] [--sanitize] [--save DIR]` | Plays sensor recordings through the firmware in every test configuration, with and without their scripted events, and compares the MIDI with BASE byte for byte; fails on any difference |

Without `--ref`, a command works on the working tree, uncommitted changes included. With `--ref`, it
works on that commit, and its results are cached in `build/fw/<commit>/`.

## The desktop build

```bash
python3 tools/fw.py desktop --run --settings SETTINGS.bin --midi-log idle.txt
```

`SETTINGS.bin` is a settings export: the instrument's whole `Configuration` struct, as the
LinnStrument Updater backs it up before an update.

This compiles the real firmware with clang, against a model of the instrument's hardware
(`desktop/hal.cpp`), and runs it in virtual time. `--run` does what the instrument does after an
update:
1. a first boot on erased flash;
2. restoring the settings export the way the LinnStrument Updater does, including how the restore
   ends: Update OS off, then the settings stored if they carry a calibration, or else the
   calibration screen with nothing stored;
3. a normal boot from that flash.

For settings that carry no calibration, `--calibration` chooses what happens:
- **`as-exported`** (the default): the settings are stored as they are, by switching Update OS off,
  which reproduces an instrument whose calibration has been cleared;
- **`stand-in`:** the default calibration is first marked as valid, as completing a calibration would,
  which turns on the code only a calibrated instrument runs (phantom-touch rejection).

The run's output says which happened. The executable is
`build/fw/<label>/desktop/linnstrument-desktop`; it can also be run directly:

```bash
EXE=build/fw/worktree/desktop/linnstrument-desktop
$EXE --provision --restore EXPORT --calibration as-exported --seconds 0 --flash-out flash.bin
$EXE --flash-in flash.bin --seconds 2 --press 10,3,0.5,1.0,900 --midi-log out.txt
$EXE --flash-in flash.bin --seconds 8 --midi-in "0.5,B06302B0622BB00610B0260F" --midi-log export.txt
```

- `--set NAME=VALUE` (with `--provision`) changes a setting before the settings are stored; the
  names are listed in `desktop/harness.h` (SETTINGS).
- `--press COL,ROW,FROM,TO,Z` holds a pad from FROM to TO seconds at raw pressure Z (0-4095, before
  the firmware's sensitivity and bias), with the finger at the pad's centre as the calibration sees
  it. It's a quick check that touches reach the note code.
- `--midi-in AT,HEX` has the instrument receive MIDI bytes at AT seconds. The example asks for
  microLinn's bulk export of the six memories (NRPN 299 = 2063), which sends 1,610 poly-pressure
  messages.
- `--timing ADC_NS,PASS_NS` changes the clock model's costs: an ADC conversion (default 32500 ns),
  and one pass of a busy-wait loop (default 3000 ns).

A harness is a C++ file compiled at the end of the sketch's translation unit, so it can use every
firmware global and function. `desktop/smoke.cpp` is the default one; others are chosen with
`--harness`. The HAL, the stand-in headers and the harness always come from the
checkout running `fw.py`, and only the sketch comes from `--ref`, so two versions of the firmware are
always compared on the same hardware model.

## Replays

```bash
python3 tools/fw.py replay --recordings DIR
```

Plays sensor recordings through the whole firmware on the desktop build, in a set of test
configurations, and compares the MIDI it sends with what the previous build (the merge-base of HEAD
with `fork/main`, or `--against REF`) sends from the same input, byte for byte. It exits with an error
if any run differs, and prints the first differing lines of each with their context.
`LINNSTRUMENT_RECORDINGS` can stand in for `--recordings`.

**Recordings** are made with the capture build's recorder (`linnstrument_capture.py`), which isn't
part of this repository. `DIR` holds, for each recording NAME, `NAME_samples.csv` (one row per scan
of a touched pad) and `NAME_settings.csv` (the sensor settings it was made with).

**Settings.** The runs don't use an instrument's settings. Each starts from the firmware's own
defaults (what a first boot on erased flash leaves) and takes only the calibration from a settings
export (`DIR/linnstrument_settings.bin` by default; `--calibration-from` chooses another). On top of
the defaults come the test configuration's settings, then the starting settings of the run's scripted
events; all are stored before the firmware boots, and each run checks after boot that they're in
effect. `replay/configurations.txt` defines and documents the configurations: a base (Wicki-Hayden
in microLinn's 12-EDO, channel per note on channels 2-16, the recordings' own sensor settings) and
the variations replayed on it (7 note channels, column offset 1, EDO off, One Channel, hammer-ons,
the other calibration state, fixed velocity with each pressure sensitivity). `--config` and `--recording` pick
some of them.

**What a run is:** one configuration, one recording, and either no events or the recording's
scripted events (`replay/NAME.events`, if there is one; `--events-dir` chooses another folder).

**How a recording is played** (the top of `desktop/replay.cpp` has the details): each recorded
reading is served where the firmware reads the sensor, at its recorded time, and the firmware's own
scan reads each pad when it reaches it, as on the instrument. The firmware sees exactly the recorded
raw pressure. The 8 reads of a new touch's velocity measurement are served one per read. Where the
capture dropped records, a touched pad holds its last reading; where its records show it lost a
touch's end (the drop flags, and a missed scan of the pad where that's in doubt), the touch ends one
scan period after its last reading, and those reads are marked `cut`. Untouched pads read what their
logged neighbours recorded for them, since the firmware adds a neighbour's pressure to a note's,
kept below the continuation threshold.

**Scripted events**, one per line, add what recordings lack. `setting` lines are starting settings,
stored before the firmware boots; the others happen at a time in ms on the recording's clock (the
CSV's `time_us` / 1000):

```
setting footLeft sustain           # a switch's assignment, or any setting of desktop/harness.h
45200   press switch2              # footLeft, footRight, footBoth, switch1 or switch2
45350   release switch2
132000  nrpn 299 259               # an NRPN the instrument receives: PARAM VALUE [CHANNEL]
132100  midi B0 63 02 B0 62 2B     # MIDI bytes the instrument receives
132200  set importing 0            # a setting that can change while running
```

**Logs**, in `build/fw/<label>/replay/<configuration>/<run>/`, with times in microseconds on the
recording's clock:
- `midi.txt`: each message the firmware sends, when it wrote it, its bytes and what it is. This is
  what the comparison looks at.
- `touches.txt`: each scan of a touched pad: the recorded reading served (its time and raw pressure),
  how it was served (`scan`, `burst`, `held`, `cut`, `stop`, `nbr`), whether the capture lost records
  just before it (`gap`), the firmware's `currentRawZ` and `pressureZ`, the touch state, note and
  channel. With an EDO set, the note is microLinn's edostep, not the MIDI note.
- `run.txt`: the run's summary: how much of the recording was served, the touch ends the model
  added and the notes they ended, the settings checked after boot, and checks on the firmware: the
  tempo and NRPN import state after boot, how full microLinn's hammer-on list got, and the notes left
  sounding at the end of the run (a note-on with no note-off after it on its channel, which the table
  of `fw.py replay` also flags).
- `settings.txt`, `provision.txt` and `flash.bin`: the run's settings once all were set, how they were
  provisioned, and the flash image the run booted from.

**Checks:**
- Results are cached under a key of everything that goes into them (the firmware, these tools, the
  recordings, events, configurations and settings export), so only what changed is run again;
  `--fresh` ignores the cache.
- `--repeat` builds, provisions and replays everything a second time and checks that the flash images
  and all the files are identical.
- `--sanitize` builds the firmware with AddressSanitizer and repeats every run with it: it fails on
  any memory error, and on logs that differ from the normal build's, which would mean the firmware's
  behaviour depends on where its variables are. Memory errors already known in the firmware are
  contained in every desktop build where their code is still there, as `fwlib/memfix.py` lists; the
  replay says when the two builds it compares had different ones contained.
- `--save DIR` copies the logs to DIR, gzipped, with `SHA256SUMS` of the uncompressed files;
  `--baseline DIR` then compares with them instead of building BASE.

## Files

| Path | What it is |
|---|---|
| `fw.py` | The command-line entry point |
| `fwlib/arduino.py` | The reference toolchain: setup, fingerprint, `arduino-builder`, sizes |
| `fwlib/sketch.py` | Staging a commit or the working tree into a folder named for the sketch |
| `fwlib/warnings.py` | Parsing and comparing warnings, by location in the sketch rather than line number |
| `fwlib/stack.py` | Static stack analysis of the linked `.elf`; its assumptions (guarded tasks, excluded calls, annotated indirect calls) are listed at the top |
| `fwlib/longfix.py` | The source rewrites of the desktop build: `long` made 32-bit, and the clock reads of the firmware's busy-waits |
| `fwlib/memfix.py` | The memory errors known in the firmware, contained in the desktop build so that its behaviour doesn't depend on where variables are |
| `fwlib/layout.py` | Struct layout comparison from the DWARF debug info of both builds |
| `fwlib/desktop.py` | The desktop build and its two-run boot |
| `fwlib/replay.py` | Replays: reading the configurations, provisioning, running, caching, comparing and the checks |
| `desktop/hal.h`, `desktop/hal.cpp` | The hardware model: clock, sensor, LEDs, UART, pins, flash |
| `desktop/include/` | Stand-ins for the Arduino core, SPI and DueFlashStorage headers |
| `desktop/sketch_prelude.h`, `desktop/sketch_tail.h` | Wrapped around the sketch in the desktop build |
| `desktop/harness.h` | What the harnesses share: MIDI messages from the UART's bytes, named settings, provisioning (an export restored as the updater does, or the firmware's defaults with an export's calibration) |
| `desktop/smoke.cpp` | The default harness |
| `desktop/replay.cpp` | The replay harness: the recordings' sensor model, scripted events, the logs |
| `replay/configurations.txt` | The replay's test configurations, documented |
| `replay/NAME.events` | The scripted events played with recording NAME, with the starting settings they need |
