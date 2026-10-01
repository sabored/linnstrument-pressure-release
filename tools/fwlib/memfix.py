"""Memory errors known in the firmware, contained in the desktop build.

Where the firmware reads or writes past the end of an array, what it touches is whatever variable the
compiler and linker placed next to that array. The desktop build places variables differently from
the instrument build, and any change that adds or moves a variable can move them again. So two
builds compared on the desktop could differ, or fail to differ, for reasons that have nothing to do
with the change being checked. For each access listed here, the desktop build gives the array the
room the firmware reaches into, holding what a missing element should read as, so that the firmware's
behaviour no longer depends on its neighbours in memory. On the instrument these accesses still reach
the variables that follow each array.

The list also holds a copy between overlapping memory: undefined behaviour in C, which the
instrument's forward-copying memcpy() happens to do as memmove() would; the desktop build calls
memmove(), so that the result is defined and the same.

Each piece of text is replaced where it's found once; one found more than once stops the build. A
containment applies only while the code that makes the error is in the sketch: the replaced text,
or the lines its entry lists where a fix would leave the replaced text as it is (an array's
declaration, for reads past its end), so that a version that fixes them runs uncontained and the
sanitizer checks the fix. One that doesn't apply is skipped and reported: that's the version of the
firmware that changed or fixed it, and the build being compared with it may still have it, so the
list stays as it is until no build compared needs it. `fw.py replay` says when the two builds it
compares had different containments. A change that only rewords one of these lines without fixing
the error loses its containment; `fw.py replay --sanitize` runs the replays under AddressSanitizer,
which reports any memory error not contained here.

The containments are for old firmware, such as v0.1.0: the current firmware has fixed these errors.
So `fw.py replay` fails when one applies to the firmware it replays, unless told that firmware is an
old one (--allow-contained); a change that brings back one of these lines would otherwise have its
memory error contained, and hidden from both the replay and the sanitizer.
"""
# (short name, text in the combined sketch, desktop replacement on the same line, why, and the texts of
# the code that makes the error, any of which must be in the sketch for the containment to apply, or
# None if that's the replaced text itself)
CONTAINMENTS = (
    ('touchInfo column 26',
     'TouchInfo touchInfo[MAXCOLS][MAXROWS];',
     'TouchInfo touchInfo[MAXCOLS + 1][MAXROWS]; '
     '__attribute__((constructor)) static void desktopMissingColumn() { for (int r = 0; r < MAXROWS; ++r) { '
     'touchInfo[MAXCOLS][r].note = -1; touchInfo[MAXCOLS][r].channel = -1; } }',
     'at column 25, the slide checks (potentialSlideTransferCandidate(), handleXExpression()) and '
     'handleZExpression() in ls_handleTouches.ino read cell(sensorCol + 1, row), a column that does not '
     'exist; the extra column reads as a pad with no touch and no note',
     ('  if (col < 1) return false;\n',
      '  else if (cell(sensorCol+1, sensorRow).currentRawZ && !cell(sensorCol+1, sensorRow).hasNote()) {\n',
      '  else if (cell(sensorCol+1, sensorRow).touched == transferCell) {\n')),
    ('microLinnHammerOns overflow',
     'MicroLinnHammerOn microLinnHammerOns[9];',
     'MicroLinnHammerOn microLinnHammerOns[256];',
     'microLinn adds a muted note to this list at each hammer-on, but a muted touch that is released '
     'returns early from handleTouchRelease() without removing it, so the list overflows; its index is '
     'a byte, so 256 entries hold every write',
     None),
    ('microLinnDeleteHammeredNote() memcpy',
     'memcpy(&microLinnHammerOns[i], &microLinnHammerOns[i + 1], 4 * (microLinnNumHammerOns - i));',
     'memmove(&microLinnHammerOns[i], &microLinnHammerOns[i + 1], 4 * (microLinnNumHammerOns - i));',
     'microLinnDeleteHammeredNote() shifts the list down by one entry with memcpy(), between overlapping '
     'memory',
     None),
)


def contain(text):
    """Returns (new_text, [names of the containments applied], [names of those whose code isn't there])."""
    applied, absent = [], []
    for name, original, replacement, why, bug in CONTAINMENTS:
        found = text.count(original)
        if found > 1:
            raise RuntimeError('desktop build: `%s` appears %d times in the sketch; update CONTAINMENTS in '
                               'tools/fwlib/memfix.py (%s)' % (original, found, why))
        if found == 0 or (bug and not any(b in text for b in bug)):
            absent.append(name)
            continue
        text = text.replace(original, replacement)
        applied.append(name)
    return text, applied, absent
