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
- the stack without re-entry no longer fits in the RAM left;
- static RAM grew by more than N bytes;
- the stack analysis needs attention (an excluded call it couldn't find, or its nesting bound was
  reached).

The stack figures are static and conservative, and there is no static worst case, because nothing
but a few flags bounds how deeply the firmware re-enters its schedulers. The top of `fwlib/stack.py`
describes the model and its assumptions.

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

## Files

| Path | What it is |
|---|---|
| `fw.py` | The command-line entry point |
| `fwlib/arduino.py` | The reference toolchain: setup, fingerprint, `arduino-builder`, sizes |
| `fwlib/sketch.py` | Staging a commit or the working tree into a folder named for the sketch |
| `fwlib/warnings.py` | Parsing and comparing warnings, by location in the sketch rather than line number |
| `fwlib/stack.py` | Static stack analysis of the linked `.elf`; its assumptions (guarded tasks, excluded calls, annotated indirect calls) are listed at the top |
| `fwlib/longfix.py` | The source rewrites of the desktop build: `long` made 32-bit, and the clock reads of the firmware's busy-waits |
| `fwlib/layout.py` | Struct layout comparison from the DWARF debug info of both builds |
| `fwlib/desktop.py` | The desktop build and its two-run boot |
| `desktop/hal.h`, `desktop/hal.cpp` | The hardware model: clock, sensor, LEDs, UART, pins, flash |
| `desktop/include/` | Stand-ins for the Arduino core, SPI and DueFlashStorage headers |
| `desktop/sketch_prelude.h`, `desktop/sketch_tail.h` | Wrapped around the sketch in the desktop build |
| `desktop/smoke.cpp` | The default harness |
