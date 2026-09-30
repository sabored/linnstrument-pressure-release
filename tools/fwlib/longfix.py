"""Make the sketch's `long` 32-bit for the desktop build, as it is on the instrument.

On the SAM3X8E `long` is 32 bits; on a 64-bit desktop it is 64. The firmware depends on the 32-bit
size in three ways: time arithmetic wraps at 2^32 (calcTimeDelta), struct layouts that are saved to
flash contain `unsigned long` fields, and `sizeof(unsigned long)` is part of the flash layout. Apple
Silicon has no -m32, so the desktop build rewrites the sketch's text instead:

  unsigned long [int]            -> uint32_t
  [signed] long [int]            -> int32_t
  123L / 123UL (any case/order)  -> ((int32_t)123) / ((uint32_t)123U)
  LONG_MAX / LONG_MIN / ULONG_MAX -> INT32_MAX / INT32_MIN / UINT32_MAX

Only the sketch's own files are rewritten (the combined .ino.cpp and the sketch's .h files), never
system headers. Comments, strings and character literals are left alone. `long long` and
`long double` are left alone (they are 64-bit and long double on both). Literals on #if/#elif lines
are left alone, since casts aren't allowed there and the preprocessor computes in intmax_t anyway.

A second rewrite marks the firmware's busy-waits (BUSY_WAITS). The virtual clock doesn't move
during plain computation, so a loop that waits for micros() or millis() to move would never end. In
each listed function, and only there, the clock reads become hal::busyWaitMicros() /
hal::busyWaitMillis(), which first advance the clock by the time one pass of such a loop takes on
the instrument. The firmware's own loop still runs, with whatever it does on each pass. Each function
must be found exactly once and must read the clock, or the build stops.
"""
import re

# The firmware's busy-waits: function name -> the clock functions its loop reads (see above; the pass
# cost is hal::timing.busyWaitPassNs).
BUSY_WAITS = {
    'delayUsec': ('micros',),                         # ls_rtos.ino: runs performContinuousTasks() while it waits
    'serialWaitForMaximumTwoSeconds': ('millis',),    # ls_serial.ino: the updater protocol's 2 s timeout
}
_BUSY_WAIT_READS = {'micros': 'hal::busyWaitMicros', 'millis': 'hal::busyWaitMillis'}

_TOKEN = re.compile(r'''
    (?P<comment>//[^\n]*|/\*.*?\*/)
  | (?P<string>"(?:\\.|[^"\\\n])*")
  | (?P<char>'(?:\\.|[^'\\\n])*')
  | (?P<number>\b(?:0[xX][0-9A-Fa-f]+|[0-9]+)(?:[uU]?[lL]{1,2}[uU]?)?\b)
  | (?P<ident>[A-Za-z_]\w*)
  | (?P<newline>\n)
  | (?P<space>[ \t\r\f\v]+|\\\n)
  | (?P<other>.)
''', re.S | re.X)

_TYPE_WORDS = {'unsigned', 'signed', 'long', 'int'}
_LIMITS = {'LONG_MAX': 'INT32_MAX', 'LONG_MIN': 'INT32_MIN', 'ULONG_MAX': 'UINT32_MAX'}


def _tokens(text):
    pos = 0
    while pos < len(text):
        m = _TOKEN.match(text, pos)
        yield m.lastgroup, m.group(0)
        pos = m.end()


def _fix_number(tok):
    m = re.match(r'(0[xX][0-9A-Fa-f]+|[0-9]+)([uU]?[lL]{1,2}[uU]?)$', tok)
    if not m:
        return tok
    digits, suffix = m.groups()
    if suffix.lower().count('l') != 1:
        return tok                                   # no L suffix, or LL: unchanged
    if 'u' in suffix.lower():
        return '((uint32_t)%sU)' % digits
    return '((int32_t)%s)' % digits


def rewrite(text):
    """Returns (new_text, counts) with `long` made 32-bit as described above."""
    out = []
    counts = {'types': 0, 'literals': 0, 'limits': 0}
    toks = list(_tokens(text))
    line_is_if = False       # current line is an #if/#elif directive
    at_line_start = True
    i = 0
    while i < len(toks):
        kind, tok = toks[i]
        if kind == 'newline':
            out.append(tok)
            line_is_if = False
            at_line_start = True
            i += 1
            continue
        if at_line_start and kind == 'other' and tok == '#':
            # look at the directive name
            j = i + 1
            while j < len(toks) and toks[j][0] == 'space':
                j += 1
            if j < len(toks) and toks[j][1] in ('if', 'elif'):
                line_is_if = True
        if kind not in ('space', 'comment'):
            at_line_start = False

        if kind == 'ident' and tok in _TYPE_WORDS:
            # collect a run of type words, allowing whitespace between them
            run = [i]
            j = i + 1
            while True:
                k = j
                while k < len(toks) and toks[k][0] == 'space':
                    k += 1
                if k < len(toks) and toks[k][0] == 'ident' and toks[k][1] in _TYPE_WORDS:
                    run.append(k)
                    j = k + 1
                else:
                    break
            words = [toks[r][1] for r in run]
            nxt = j
            while nxt < len(toks) and toks[nxt][0] == 'space':
                nxt += 1
            next_word = toks[nxt][1] if nxt < len(toks) else ''
            if words.count('long') == 1 and next_word != 'double' and not line_is_if:
                out.append('uint32_t' if 'unsigned' in words else 'int32_t')
                counts['types'] += 1
                i = run[-1] + 1
                continue
            out.extend(t for _, t in toks[i:run[-1] + 1])
            i = run[-1] + 1
            continue

        if kind == 'number' and not line_is_if:
            new = _fix_number(tok)
            if new != tok:
                counts['literals'] += 1
            out.append(new)
        elif kind == 'ident' and tok in _LIMITS:
            out.append(_LIMITS[tok])
            counts['limits'] += 1
        else:
            out.append(tok)
        i += 1
    return ''.join(out), counts


def _definitions(text, name):
    """(start, open_brace, end) of each definition of function `name` (not declarations), found on
    tokens so that comments and strings don't confuse it; end is just past the closing brace."""
    toks = list(_tokens(text))
    offsets = []
    pos = 0
    for _, tok in toks:
        offsets.append(pos)
        pos += len(tok)
    found = []
    depth = 0
    i = 0
    while i < len(toks):
        kind, tok = toks[i]
        if kind == 'other' and tok == '{':
            depth += 1
        elif kind == 'other' and tok == '}':
            depth -= 1
        elif kind == 'ident' and tok == name and depth == 0:
            # name ( ... ) {   -> a definition; name ( ... ) ;  -> a declaration or a call
            j = i + 1
            while j < len(toks) and toks[j][0] in ('space', 'newline', 'comment'):
                j += 1
            if j < len(toks) and toks[j][1] == '(':
                level = 0
                while j < len(toks):
                    if toks[j][1] == '(':
                        level += 1
                    elif toks[j][1] == ')':
                        level -= 1
                        if level == 0:
                            break
                    j += 1
                k = j + 1
                while k < len(toks) and toks[k][0] in ('space', 'newline', 'comment'):
                    k += 1
                if k < len(toks) and toks[k][1] == '{':
                    level = 0
                    e = k
                    while e < len(toks):
                        if toks[e][0] == 'other' and toks[e][1] == '{':
                            level += 1
                        elif toks[e][0] == 'other' and toks[e][1] == '}':
                            level -= 1
                            if level == 0:
                                break
                        e += 1
                    found.append((offsets[i], offsets[k], offsets[e] + 1))
                    i = e + 1
                    continue
        i += 1
    return found


def mark_busy_waits(text):
    """Rewrites the clock reads inside each function of BUSY_WAITS. Returns (text, {name: reads})."""
    counts = {}
    for name, clocks in BUSY_WAITS.items():
        defs = _definitions(text, name)
        if len(defs) != 1:
            raise RuntimeError(
                'desktop build: expected exactly one definition of %s(), found %d. If the firmware changed '
                'it, update BUSY_WAITS in tools/fwlib/longfix.py.'
                % (name, len(defs)))
        start, brace, end = defs[0]
        body = text[brace:end]
        out, n = [], 0
        toks = list(_tokens(body))
        for idx, (kind, tok) in enumerate(toks):
            if kind == 'ident' and tok in clocks:
                nxt = idx + 1
                while nxt < len(toks) and toks[nxt][0] in ('space', 'newline'):
                    nxt += 1
                if nxt < len(toks) and toks[nxt][1] == '(':
                    out.append(_BUSY_WAIT_READS[tok])
                    n += 1
                    continue
            out.append(tok)
        if n == 0:
            raise RuntimeError(
                'desktop build: %s() no longer reads %s(), so the desktop clock model of its busy-wait '
                'no longer applies. Update BUSY_WAITS in tools/fwlib/longfix.py.' % (name, '/'.join(clocks)))
        text = text[:brace] + ''.join(out) + text[end:]
        counts[name] = n
    return text, counts
