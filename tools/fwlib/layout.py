"""Compare the sketch's struct layouts between the instrument build and the desktop build.

Both builds carry DWARF debug info (the instrument's .elf and the desktop sketch object), which
llvm-dwarfdump (`dwarfdump` on macOS) reads for both formats. For every struct, class and union the
sketch defines, the check compares size, and each member's offset, size and bitfield position,
recursively through nested structs and arrays. Type names are ignored, because the desktop build
spells `unsigned long` as uint32_t; sizes and signedness are compared instead.

Differences are expected only in structs that hold pointers (8 bytes on the desktop). Anything else
that differs means the desktop build doesn't see memory as the instrument does, and anything saved
to flash or exchanged with the updater (Configuration, SequencerProject and the old layouts the
settings converters read) must be identical.
"""
import os
import re
import shutil
import subprocess

_DIE = re.compile(r'^(0x[0-9a-f]+):( +)(DW_TAG_\w+|NULL)')
_ATTR = re.compile(r'^\s+(DW_AT_\w+)\s+\((.*)\)\s*$')
_REF = re.compile(r'^(0x[0-9a-f]+)(?: "(.*)")?$')


def dwarfdump_tool():
    for name in ('llvm-dwarfdump', 'dwarfdump'):
        path = shutil.which(name)
        if path:
            return path
    try:
        return subprocess.run(['xcrun', '--find', 'llvm-dwarfdump'], capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        raise RuntimeError('llvm-dwarfdump not found (on macOS it comes with the Xcode command line tools)')


class Die:
    __slots__ = ('offset', 'tag', 'attrs', 'children', 'parent')

    def __init__(self, offset, tag, parent):
        self.offset, self.tag, self.parent = offset, tag, parent
        self.attrs, self.children = {}, []

    def ref(self, name):
        v = self.attrs.get(name)
        if v is None:
            return None
        m = _REF.match(v)
        return int(m.group(1), 16) if m else None

    def num(self, name, default=None):
        v = self.attrs.get(name)
        if v is None:
            return default
        try:
            return int(v, 0)
        except ValueError:
            return default

    def name(self):
        v = self.attrs.get('DW_AT_name')
        return v[1:-1] if v and v.startswith('"') else v


def parse(path, unit_filter=None):
    """Parses `dwarfdump --debug-info PATH` into DIE trees. Returns {offset: Die} for the compile
    units whose DW_AT_name contains unit_filter (all units when None)."""
    out = subprocess.run([dwarfdump_tool(), '--debug-info', path], capture_output=True, text=True, check=True).stdout
    dies = {}
    stack = []            # (depth, die)
    current = None
    keep = True
    for line in out.splitlines():
        m = _DIE.match(line)
        if m:
            offset, indent, tag = int(m.group(1), 16), len(m.group(2)), m.group(3)
            depth = indent // 2
            while stack and stack[-1][0] >= depth:
                stack.pop()
            if tag == 'NULL':
                current = None
                continue
            parent = stack[-1][1] if stack else None
            current = Die(offset, tag, parent)
            if parent:
                parent.children.append(current)
            stack.append((depth, current))
            if tag == 'DW_TAG_compile_unit':
                keep = True
            dies[offset] = current
            continue
        a = _ATTR.match(line)
        if a and current is not None:
            current.attrs[a.group(1)] = a.group(2)
            if current.tag == 'DW_TAG_compile_unit' and a.group(1) == 'DW_AT_name' and unit_filter:
                keep = unit_filter in a.group(2)
                if not keep:
                    current.attrs['_skip'] = '1'
    if unit_filter:
        # drop the DIEs of other compile units
        def unit_of(d):
            while d.parent is not None:
                d = d.parent
            return d
        dies = {o: d for o, d in dies.items() if '_skip' not in unit_of(d).attrs}
    return dies


def _normalize(name):
    # template arguments: GCC writes ByteBuffer<4096u>, clang ByteBuffer<4096U>
    return re.sub(r'\b(\d+)[uUlL]+\b', r'\1', name)


def qualified_name(die):
    parts = [_normalize(die.name() or '<anon>')]
    p = die.parent
    while p is not None and p.tag != 'DW_TAG_compile_unit':
        if p.tag in ('DW_TAG_namespace', 'DW_TAG_structure_type', 'DW_TAG_class_type', 'DW_TAG_union_type'):
            parts.append(p.name() or '<anon>')
        p = p.parent
    return '::'.join(reversed(parts))


_AGGREGATES = ('DW_TAG_structure_type', 'DW_TAG_class_type', 'DW_TAG_union_type')


def aggregates(dies, sketch_dirs):
    """Named, complete structs/classes/unions declared in a file directly inside one of sketch_dirs
    (the folders the build read the sketch's .ino and .h files from), or everywhere when None."""
    dirs = None if sketch_dirs is None else {os.path.realpath(d) for d in sketch_dirs}
    found = {}
    for d in dies.values():
        if d.tag in _AGGREGATES and d.name() and 'DW_AT_declaration' not in d.attrs and 'DW_AT_byte_size' in d.attrs:
            if dirs is not None:
                decl = d.attrs.get('DW_AT_decl_file', '').strip('"')
                if os.path.dirname(os.path.realpath(decl)) not in dirs:
                    continue
            found.setdefault(qualified_name(d), d)
    return found


class Layout:
    """Resolves types to comparable descriptions."""

    def __init__(self, dies):
        self.dies = dies

    def strip(self, off):
        d = self.dies.get(off)
        while d is not None and d.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type'):
            nxt = d.ref('DW_AT_type')
            d = self.dies.get(nxt) if nxt is not None else None
        return d

    def describe(self, off):
        """('base', size, encoding) | ('enum', size) | ('ptr', size) | ('array', dims, elem) | ('agg', die)"""
        d = self.strip(off)
        if d is None:
            return ('void', 0)
        if d.tag == 'DW_TAG_base_type':
            enc = d.attrs.get('DW_AT_encoding', '')
            enc = {'DW_ATE_signed_char': 'DW_ATE_signed', 'DW_ATE_unsigned_char': 'DW_ATE_unsigned'}.get(enc, enc)
            return ('base', d.num('DW_AT_byte_size'), enc)
        if d.tag == 'DW_TAG_enumeration_type':
            return ('enum', d.num('DW_AT_byte_size'))
        if d.tag in ('DW_TAG_pointer_type', 'DW_TAG_reference_type', 'DW_TAG_rvalue_reference_type',
                     'DW_TAG_ptr_to_member_type'):
            return ('ptr',)
        if d.tag == 'DW_TAG_array_type':
            dims = []
            for c in d.children:
                if c.tag == 'DW_TAG_subrange_type':
                    if 'DW_AT_count' in c.attrs:
                        dims.append(c.num('DW_AT_count'))
                    elif 'DW_AT_upper_bound' in c.attrs:
                        dims.append(c.num('DW_AT_upper_bound') + 1)
                    else:
                        dims.append(0)
            return ('array', tuple(dims), self.describe(d.ref('DW_AT_type')))
        if d.tag in _AGGREGATES:
            return ('agg', d)
        return ('other', d.tag)

    def members(self, agg):
        """[(name, bit_offset, bit_size_or_None, type_offset)] in declaration order."""
        result = []
        for c in agg.children:
            if c.tag == 'DW_TAG_inheritance':
                result.append(('<base>', 8 * c.num('DW_AT_data_member_location', 0), None, c.ref('DW_AT_type')))
            elif c.tag == 'DW_TAG_member' and 'DW_AT_external' not in c.attrs and 'DW_AT_declaration' not in c.attrs:
                name = c.name() or '<anon>'
                bit_size = c.num('DW_AT_bit_size')
                if bit_size is not None:
                    if 'DW_AT_data_bit_offset' in c.attrs:                  # DWARF 4+/5 style
                        bit_off = c.num('DW_AT_data_bit_offset')
                    else:                                                  # DWARF 2/3 style, little-endian target
                        storage = 8 * c.num('DW_AT_byte_size', 0)
                        bit_off = 8 * c.num('DW_AT_data_member_location', 0) + storage - c.num('DW_AT_bit_offset', 0) - bit_size
                    result.append((name, bit_off, bit_size, c.ref('DW_AT_type')))
                else:
                    result.append((name, 8 * c.num('DW_AT_data_member_location', 0), None, c.ref('DW_AT_type')))
        return result


def _has_pointer(layout, desc, seen=None):
    seen = seen or set()
    kind = desc[0]
    if kind == 'ptr':
        return True
    if kind == 'array':
        return _has_pointer(layout, desc[2], seen)
    if kind == 'agg':
        if desc[1].offset in seen:
            return False
        seen.add(desc[1].offset)
        return any(_has_pointer(layout, layout.describe(t), seen) for _, _, _, t in layout.members(desc[1]))
    return False


def compare_types(la, da, lb, db, path, diffs, limit=20):
    if len(diffs) >= limit:
        return
    ka, kb = da[0], db[0]
    if ka != kb:
        diffs.append('%s: %s vs %s' % (path, ka, kb))
        return
    if ka in ('base', 'enum', 'ptr'):
        if da[1:] != db[1:]:
            diffs.append('%s: %s %s vs %s' % (path, ka, da[1:], db[1:]))
        return
    if ka == 'array':
        if da[1] != db[1]:
            diffs.append('%s: array %s vs %s' % (path, list(da[1]), list(db[1])))
            return
        compare_types(la, da[2], lb, db[2], path + '[]', diffs, limit)
        return
    if ka == 'agg':
        sa, sb = da[1].num('DW_AT_byte_size'), db[1].num('DW_AT_byte_size')
        if sa != sb:
            diffs.append('%s: size %d vs %d' % (path, sa, sb))
        ma, mb = la.members(da[1]), lb.members(db[1])
        if [m[0] for m in ma] != [m[0] for m in mb]:
            diffs.append('%s: members differ: %s vs %s' % (path, [m[0] for m in ma], [m[0] for m in mb]))
            return
        for (name, oa, ba, ta), (_, ob, bb, tb) in zip(ma, mb):
            p = '%s.%s' % (path, name)
            if oa != ob or ba != bb:
                pos = lambda o, b: ('byte %d' % (o // 8)) if b is None else ('bit %d:%d' % (o, b))
                diffs.append('%s: at %s vs %s' % (p, pos(oa, ba), pos(ob, bb)))
            compare_types(la, la.describe(ta), lb, lb.describe(tb), p, diffs, limit)


def compare(instrument_path, desktop_path, instrument_dirs, desktop_dirs,
            instrument_unit='linnstrument-firmware.ino.cpp'):
    """Returns a report dict: identical, pointer (differ only in structs holding pointers), differ,
    missing (in one build only: 'name (instrument only)' or 'name (desktop only)')."""
    ia = parse(instrument_path, unit_filter=instrument_unit)
    da = parse(desktop_path)
    agg_i = aggregates(ia, instrument_dirs)
    agg_d = aggregates(da, desktop_dirs)
    for required in ('Configuration', 'TouchInfo'):
        if required not in agg_i or required not in agg_d:
            raise RuntimeError('layout: %s not found among the sketch\'s structs (instrument dirs %s, desktop dirs %s); '
                               'the check would be meaningless' % (required, instrument_dirs, desktop_dirs))
    li, ld = Layout(ia), Layout(da)
    report = {'identical': [], 'pointer': [], 'differ': [], 'missing': []}
    report['missing'] = ['%s (desktop only)' % n for n in sorted(set(agg_d) - set(agg_i))]
    for name in sorted(agg_i):
        if name not in agg_d:
            report['missing'].append('%s (instrument only)' % name)
            continue
        diffs = []
        desc_i, desc_d = ('agg', agg_i[name]), ('agg', agg_d[name])
        compare_types(li, desc_i, ld, desc_d, name, diffs)
        if not diffs:
            report['identical'].append((name, agg_i[name].num('DW_AT_byte_size')))
        elif _has_pointer(li, desc_i):
            report['pointer'].append((name, diffs))
        else:
            report['differ'].append((name, diffs))
    return report
