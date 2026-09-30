"""Parsing and comparing the compiler warnings of a -Wall -Wextra build.

A warning is identified by where it is in the sketch, not by its line number, so that edits
elsewhere don't make old warnings look new: the file, the enclosing function, the message and the
text of the source line. Warnings reported inside a core macro (26 of v0.1.0's 40 come from the
core's constrain() and min() macros) are attributed to the sketch line that expanded the macro.
"""
import collections
import re

_CONTEXT = re.compile(r"^(?P<file>[^:]+): (?:In (?:member )?function|In constructor|In destructor|"
                      r"In instantiation of|In static member function|At global scope)(?P<rest>.*)$")
_DIAG = re.compile(r'^(?P<file>[^:\s][^:]*):(?P<line>\d+):(?P<col>\d+): (?P<kind>warning|note|error): (?P<msg>.*)$')
_FLAG = re.compile(r'^(?P<msg>.*?) \[(?P<flag>-W[^\]]+)\]$')

Warning = collections.namedtuple('Warning', 'file line function message flag source')


def _short(path, sketch_markers):
    for m in sketch_markers:
        i = path.rfind(m)
        if i >= 0:
            return path[i + len(m):], True
    return path.rsplit('/', 1)[-1], False


def parse(log, sketch_markers=('/linnstrument-firmware/',)):
    """Returns the list of warnings in a gcc build log."""
    lines = log.splitlines()
    warnings = []
    function = {}
    current = None       # [short_file, line, function, message, flag, source, in_sketch]

    def flush():
        if current:
            warnings.append(Warning(*current[:6]))

    i = 0
    while i < len(lines):
        line = lines[i]
        c = _CONTEXT.match(line)
        if c:
            f, _ = _short(c.group('file'), sketch_markers)
            rest = c.group('rest').strip()
            function[f] = rest[1:-2] if rest.startswith("'") else ''
            i += 1
            continue
        d = _DIAG.match(line)
        if d:
            f, in_sketch = _short(d.group('file'), sketch_markers)
            source = lines[i + 1].strip() if i + 1 < len(lines) else ''
            if d.group('kind') == 'warning':
                flush()
                msg, flag = d.group('msg'), ''
                fm = _FLAG.match(msg)
                if fm:
                    msg, flag = fm.group('msg'), fm.group('flag')
                current = [f, int(d.group('line')), function.get(f, ''), msg, flag, source, in_sketch]
            elif d.group('kind') == 'note' and current and not current[6] and in_sketch \
                    and 'in expansion of macro' in d.group('msg'):
                # the warning is inside a core macro: attribute it to the sketch line that expanded it
                current[0], current[1], current[5], current[6] = f, int(d.group('line')), source, True
                current[2] = function.get(f, current[2])
            i += 1
            continue
        i += 1
    flush()
    return warnings


def key(w):
    return (w.file, w.function, w.message, w.flag, w.source)


def diff(old, new):
    """(added, removed): warnings in new but not old, and in old but not new, counting repeats."""
    co = collections.Counter(key(w) for w in old)
    cn = collections.Counter(key(w) for w in new)
    added_keys = cn - co
    removed_keys = co - cn

    def pick(ws, counts):
        out = []
        left = dict(counts)
        for w in ws:
            k = key(w)
            if left.get(k, 0) > 0:
                out.append(w)
                left[k] -= 1
        return out

    return pick(new, added_keys), pick(old, removed_keys)


def format_warning(w):
    where = '%s:%d' % (w.file, w.line)
    fn = ' (in %s)' % w.function if w.function else ''
    flag = ' [%s]' % w.flag if w.flag else ''
    return '%s: %s%s%s\n      %s' % (where, w.message, flag, fn, w.source)
