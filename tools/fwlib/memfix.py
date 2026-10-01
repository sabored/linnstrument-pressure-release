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
piece that isn't found is skipped and reported: that's the version of the firmware that changed or
fixed it, and the build being compared with it may still have it, so the list stays as it is until no
build compared needs it. `fw.py replay` says when the two builds it compares had different
containments. A change that only rewords one of these lines without fixing the error loses its
containment; `fw.py replay --sanitize` runs the replays under AddressSanitizer, which reports any
memory error not contained here.
"""
# (short name, text in the combined sketch, desktop replacement on the same line, why)
CONTAINMENTS = (
    ('touchInfo column 26',
     'TouchInfo touchInfo[MAXCOLS][MAXROWS];',
     'TouchInfo touchInfo[MAXCOLS + 1][MAXROWS]; '
     '__attribute__((constructor)) static void desktopMissingColumn() { for (int r = 0; r < MAXROWS; ++r) { '
     'touchInfo[MAXCOLS][r].note = -1; touchInfo[MAXCOLS][r].channel = -1; } }',
     'at column 25, the slide checks (potentialSlideTransferCandidate(), handleXExpression()) and '
     'handleZExpression() in ls_handleTouches.ino read cell(sensorCol + 1, row), a column that does not '
     'exist; the extra column reads as a pad with no touch and no note'),
    ('microLinnHammerOns overflow',
     'MicroLinnHammerOn microLinnHammerOns[9];',
     'MicroLinnHammerOn microLinnHammerOns[256];',
     'microLinn adds a muted note to this list at each hammer-on, but a muted touch that is released '
     'returns early from handleTouchRelease() without removing it, so the list overflows; its index is '
     'a byte, so 256 entries hold every write'),
    ('microLinnDeleteHammeredNote() memcpy',
     'memcpy(&microLinnHammerOns[i], &microLinnHammerOns[i + 1], 4 * (microLinnNumHammerOns - i));',
     'memmove(&microLinnHammerOns[i], &microLinnHammerOns[i + 1], 4 * (microLinnNumHammerOns - i));',
     'microLinnDeleteHammeredNote() shifts the list down by one entry with memcpy(), between overlapping '
     'memory'),
)


def contain(text):
    """Returns (new_text, [names of the containments applied], [names of those whose text isn't there])."""
    applied, absent = [], []
    for name, original, replacement, why in CONTAINMENTS:
        found = text.count(original)
        if found > 1:
            raise RuntimeError('desktop build: `%s` appears %d times in the sketch; update CONTAINMENTS in '
                               'tools/fwlib/memfix.py (%s)' % (original, found, why))
        if found == 0:
            absent.append(name)
            continue
        text = text.replace(original, replacement)
        applied.append(name)
    return text, applied, absent
