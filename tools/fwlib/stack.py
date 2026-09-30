"""Static worst-case stack depth of the instrument build.

Works on the linked .elf of an analysis build (the reference build plus -fstack-usage, which leaves
the .bin identical), so it covers everything that runs: the sketch, the Arduino core, libsam, newlib
and libgcc. The model and its assumptions:

- Frames: from each function's prologue in the disassembly (push, sub sp), checked against the
  compiler's -fstack-usage figures wherever a function matches one entry unambiguously; the
  compiler's figure is then used.
- Calls: bl, and branches that leave the function (tail calls). Indirect calls (blx/bx rN) are
  resolved by a data flow over each function's control flow graph, which tracks function addresses
  loaded from literal pools through registers, stack slots and branch joins, and C++ virtual calls
  (vtable pointer, then slot) to every vtable's function at that slot. INDIRECT_TARGETS resolves the
  few calls through function-pointer variables. Anything else may reach any function whose address
  appears in the program image (except interrupt vectors, main and Reset_Handler); those call sites
  are listed.
- Re-entry: the call graph is cut at the scan loop (modeLoopPerformance) and at
  performContinuousTasks, which the firmware re-enters by design (delayUsecWithScanning() runs the
  scan loop while text scrolls; performContinuousTasks runs inside delayUsec(), inside setLed()
  while LED updates are buffered, and directly from about 30 other places, mostly long loops).
  Every call into either one, from wherever, counts as an entry. Each call performContinuousTasks
  makes belongs to one of its tasks,
  identified from the debug info's inline records, and a flag-guarded task (GUARDED_TASKS) never
  runs inside itself. Nothing else bounds the nesting, so there's no static worst case: the levels
  are counted, the headline is the normal structure (HEADLINE_LEVELS: one of each), and a table
  (REPORTED_LEVELS) shows what re-entry adds.
- INFEASIBLE_CALLS lists calls that can't happen while the instrument is playing, with the reason.
- Interrupts: SysTick at the lowest priority, the UART and USB interrupts at priority 0 (the SAM
  core's defaults; the sketch sets none). So the main thread can be interrupted by SysTick, and
  SysTick by the UART or USB interrupt, each adding an exception frame of 8 words plus 4 bytes of
  alignment.

Known limits of the call resolution (neither affects v0.1.0):
- In C++ code, a call through a function pointer stored in a struct looks like a virtual call (a
  load from the object, then a load at an offset, then blx) and is resolved to the vtable functions
  at that offset instead. In v0.1.0, all 18 calls resolved as virtual are virtual calls (Print and
  Stream methods, PluggableUSB modules, the sketch's Serial.write(buffer, size)).
- Only a plain str to the stack updates what the tool remembers about a function address held in
  that stack slot. Other stores to the stack (strd, stm, strb/strh, pre-indexed stores) leave it
  unchanged, so a stale address could be taken for the slot's content. In v0.1.0 no call is resolved
  through a stack slot at all: the resolution is identical with stack-slot tracking switched off.
"""
import bisect
import collections
import os
import re
import subprocess

EXCEPTION_FRAME = 36
VECTOR_WORDS = 16 + 45           # Cortex-M3 system exceptions + SAM3X8E peripheral interrupts
FLASH_BASE = 0x80000

# performContinuousTasks() (ls_rtos.ino) runs each of these under its own static flag, so a task
# never runs inside itself. (checkAdvanceArpeggiator and checkAdvanceSequencer are guarded inside
# their perform* wrappers.) A call in performContinuousTasks that belongs to no listed task is
# treated as unguarded and reported.
GUARDED_TASKS = (
    'checkRefreshLedColumn', 'checkStopBlinkingLeds', 'checkTimeToRefreshTouchAnim',
    'checkLegendDisplayTimeout', 'checkTimeToReadFootSwitches', 'checkRefreshGlobalSettingsDisplay',
    'checkSleep', 'checkUpdateClock', 'performCheckAdvanceArpeggiator', 'performCheckAdvanceSequencer',
    'handleSerialIO', 'handleMidiInput', 'handlePendingMidi',
)
UNGUARDED_LEAVES = ('millis', 'micros')          # called by performContinuousTasks itself; harmless

# How deep the schedulers may nest. performContinuousTasks() runs again inside delayUsec(), inside
# setLed() while LED updates are buffered, and wherever it's called directly (about 30 places), so
# any task that waits, paints or runs a long loop can nest it; the scan loop runs again while text
# scrolls (delayUsecWithScanning()). Nothing but the task flags bounds
# this, so there's no static worst case: the headline is the normal structure (one level of each),
# and the table shows what each extra level of re-entry can add.
HEADLINE_LEVELS = (1, 1)          # (performContinuousTasks levels, scan loop levels)
REPORTED_LEVELS = ((1, 1), (1, 2), (2, 1), (2, 2), (3, 3))
MAX_SCHEDULER_ENTRIES = 32        # a safety bound on nesting; reaching it is reported

# Calls that can't happen while the instrument plays: (caller, callee) by name without arguments.
INFEASIBLE_CALLS = {
    ('loop', 'modeLoopManufacturingTest'):
        'factory test mode, entered only by holding a switch at power-up',
    ('delayUsecWithScanning', 'delayUsec'):
        'only before setup() finishes, when performContinuousTasks() returns at once',
}

# Indirect calls through function-pointer variables, and their only targets.
INDIRECT_TARGETS = {
    'UOTGHS_Handler': ('USB_ISR',),   # libsam calls gpf_isr, which only UDD_SetStack() sets, called
                                      # once, from USBDevice_'s constructor, with USB_ISR
}

# Watched functions, on the touch, note and MIDI paths; the report shows the stack below each.
WATCHED = ('handleNewTouch', 'handleXYZupdate', 'handleTouchRelease', 'handleZExpression',
           'handleXExpression', 'prepareNewNote', 'takeChannel', 'sendNewNote', 'sendReleasedNote',
           'preSendLoudness', 'transferFromSameRowCell', 'handleFootSwitchState', 'performContinuousTasks')

MAX_EXHAUSTIVE_CYCLE = 12
PRINTF_FAMILY = ('_svfprintf_r', '_vfprintf_r', '_svfiprintf_r', '_vfiprintf_r')

_FUNC_HDR = re.compile(r'^([0-9a-f]+) <(.+)>:$')
_INSN = re.compile(r'^\s+([0-9a-f]+):\s+(\S+)(?:\s+(.*))?$')
_REGLIST = re.compile(r'\{([^}]*)\}')
_TARGET = re.compile(r'\b([0-9a-f]+) <')
_LITERAL = re.compile(r';\s*\(([0-9a-f]+) <')
_LOAD = re.compile(r'^(r\d+|ip|lr|sl|fp),\s*\[(r\d+|ip|sp|lr|sl|fp)(?:,\s*#(-?(?:0x[0-9a-f]+|\d+)))?\]$')
_STORE_SP = re.compile(r'^(r\d+|ip|lr|sl|fp),\s*\[sp(?:,\s*#(\d+))?\]$')
_MOVE = re.compile(r'^(r\d+|ip|lr|sl|fp),\s*(r\d+|ip|lr|sl|fp)$')
_REG = re.compile(r'^(r\d+|ip|lr|sl|fp|sp|pc)$')

_CONDS = ('eq', 'ne', 'cs', 'hs', 'cc', 'lo', 'mi', 'pl', 'vs', 'vc', 'hi', 'ls', 'ge', 'lt', 'gt', 'le', 'al')
_COND_BRANCHES = {'b' + c for c in _CONDS}
_CONDITIONABLE = {'bl', 'blx', 'bx', 'b', 'pop', 'push', 'ldr', 'ldrb', 'ldrh', 'ldrsb', 'ldrsh', 'ldrd',
                  'str', 'strb', 'strh', 'strd', 'mov', 'mvn', 'add', 'sub', 'orr', 'and', 'eor', 'bic',
                  'lsl', 'lsr', 'asr', 'cmp', 'cmn', 'tst', 'mul', 'ldm', 'ldmia', 'stm', 'stmia', 'neg',
                  'rsb', 'uxtb', 'uxth', 'sxtb', 'sxth', 'movw', 'movt', 'ubfx', 'adc', 'sbc'}
_NOT_WRITING = {'str', 'strb', 'strh', 'strd', 'cmp', 'cmn', 'tst', 'teq', 'b', 'bl', 'blx', 'bx', 'cbz',
                'cbnz', 'push', 'stmdb', 'stm', 'stmia', 'nop', 'tbb', 'tbh', 'dmb', 'dsb', 'isb', 'cpsie',
                'cpsid', 'wfi', 'wfe', 'bkpt', 'svc', 'msr', 'sev'}
_CALL_CLOBBERED = ('r0', 'r1', 'r2', 'r3', 'ip', 'lr')


def _run(args):
    return subprocess.run(args, capture_output=True, text=True, check=True).stdout


def _regs_in_list(text):
    m = _REGLIST.search(text)
    if not m:
        return []
    out = []
    for part in m.group(1).split(','):
        part = part.strip()
        if '-' in part:
            a, b = part.split('-')
            out += ['r%d' % i for i in range(int(a.strip()[1:]), int(b.strip()[1:]) + 1)]
        elif part:
            out.append(part)
    return out


def _imm(text):
    m = re.search(r'#(-?(?:0x[0-9a-f]+|\d+))', text)
    return int(m.group(1), 0) if m else None


def _mnemonic(op):
    """'bleq' -> ('bl', True); 'bls.n' -> ('b', True); 'pop' -> ('pop', False); 'movs' -> ('movs', False)"""
    m = op.split('.')[0]
    if m in _COND_BRANCHES:
        return 'b', True
    if len(m) > 2 and m[-2:] in _CONDS and m[:-2] in _CONDITIONABLE:
        return m[:-2], True
    return m, False


class Function:
    __slots__ = ('name', 'addr', 'size', 'frame', 'frame_source', 'insns', 'sites', 'dynamic', 'late_frame')

    def __init__(self, name, addr, size):
        self.name, self.addr, self.size = name, addr, size
        self.frame, self.frame_source = 0, 'prologue'
        self.insns = []          # (addr, mnemonic, conditional, args)
        self.sites = []          # call sites: (addr, targets, tail, unresolved)
        self.dynamic = False
        self.late_frame = 0


def _symbols(tc, elf):
    """FUNC symbols: {addr: (name, size)}, choosing global names over local aliases."""
    out = _run([tc.tool('readelf'), '-sW', elf])
    funcs = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 8 and parts[3] == 'FUNC' and parts[6] != 'UND':
            addr = int(parts[1], 16) & ~1
            size = int(parts[2], 16) if parts[2].startswith('0x') else int(parts[2])
            name = parts[7]
            if addr not in funcs or (parts[4] == 'GLOBAL' and funcs[addr][2] != 'GLOBAL'):
                funcs[addr] = (name, size, parts[4])
    return {a: (n, s) for a, (n, s, _) in funcs.items()}


def _demangle(tc, names):
    names = list(names)
    out = subprocess.run([tc.tool('c++filt')], input='\n'.join(names), capture_output=True, text=True, check=True).stdout
    return dict(zip(names, out.splitlines()))


def _su_frames(build_path):
    """{base function name: [(size, kind)]} from every .su file of the build."""
    frames = collections.defaultdict(list)
    for root, _, files in os.walk(build_path):
        for f in files:
            if f.endswith('.su'):
                for line in open(os.path.join(root, f), errors='replace'):
                    parts = line.rstrip('\n').split('\t')
                    if len(parts) != 3:
                        continue
                    loc, size, kind = parts
                    frames[_base_name(loc.split(':', 3)[-1])].append((int(size), kind))
    return frames


def _base_name(sig):
    """'int32_t FXD_MUL(int32_t, int32_t)' or 'FXD_MUL(long, long)' -> 'FXD_MUL'; keeps Class::"""
    sig = re.sub(r'\s*\[with .*\]$', '', sig)
    sig = re.sub(r'\s*\[clone [^\]]*\]$', '', sig)
    depth, cut = 0, len(sig)
    for i in range(len(sig) - 1, -1, -1):          # the parameter list is the last top-level (...)
        ch = sig[i]
        if ch == ')':
            depth += 1
        elif ch == '(':
            depth -= 1
            if depth == 0:
                cut = i
                break
    head = sig[:cut].strip()
    head = re.sub(r'<[^<>]*>', '', head)            # template arguments
    return head.split(' ')[-1].lstrip('*&') if head else sig


# ---------------------------------------------------------------- per-function data flow

def _join(a, b):
    """Register/stack-slot facts after a join: a key missing from either side is unknown."""
    return {k: a[k] | b[k] for k in a.keys() & b.keys()}


def _transfer(ins, st, word):
    addr, m, cond, args = ins
    out = dict(st)
    dest = args.split(',')[0].strip() if args else ''
    if m == 'ldr' and '[pc' in args:
        lit = _LITERAL.search(args)
        v = word(int(lit.group(1), 16)) if lit else None
        if v is None:
            out.pop(dest, None)
        else:
            out[dest] = frozenset([('lit', v)])
    elif m == 'ldr' and _LOAD.match(args):
        mm = _LOAD.match(args)
        src, off = mm.group(2), int(mm.group(3) or '0', 0)
        if src == 'sp':
            if ('sp', off) in st:
                out[dest] = st[('sp', off)]
            else:
                out.pop(dest, None)
        else:
            facts = st.get(src)
            if facts and all(f[0] == 'vptr' for f in facts):
                out[dest] = frozenset([('slot', off)])
            elif off == 0:
                out[dest] = frozenset([('vptr',)])
            else:
                out.pop(dest, None)
    elif m == 'str' and _STORE_SP.match(args):
        mm = _STORE_SP.match(args)
        key = ('sp', int(mm.group(2) or '0'))
        if mm.group(1) in st:
            out[key] = st[mm.group(1)]
        else:
            out.pop(key, None)
    elif m in ('mov', 'movs') and _MOVE.match(args):
        mm = _MOVE.match(args)
        if mm.group(2) in st:
            out[mm.group(1)] = st[mm.group(2)]
        else:
            out.pop(mm.group(1), None)
    elif m in ('bl', 'blx'):
        for r in _CALL_CLOBBERED:
            out.pop(r, None)
    elif m in ('pop', 'ldm', 'ldmia'):
        for r in _regs_in_list(args):
            out.pop(r, None)
    elif m in ('ldrd', 'umull', 'smull', 'umlal', 'smlal'):
        for r in [x.strip() for x in args.split(',')[:2]]:
            out.pop(r, None)
    elif m not in _NOT_WRITING and _REG.match(dest):
        out.pop(dest, None)
    if m in ('push', 'pop') or (m in ('sub', 'add') and args.split(',')[0].strip() == 'sp'):
        # the stack pointer moved: stack-slot offsets no longer line up
        out = {k: v for k, v in out.items() if not (isinstance(k, tuple) and k[0] == 'sp')}
    if cond and m not in ('b', 'bl', 'blx', 'bx'):
        out = _join(st, out)                        # an IT-conditional instruction may not execute
    return out


def _analyze_function(f, funcs, starts, owner, word, image_byte):
    """Frame from the prologue; control flow graph; data flow; call sites."""
    insns = f.insns
    index = {ins[0]: i for i, ins in enumerate(insns)}

    # ---- frame: stack pointer decrements before the first branch or call (the prologue), plus any
    # later push (after an early return), which can only overestimate
    in_prologue = True
    for addr, m, cond, args in insns:
        dec = 0
        if m == 'push' or (m == 'stmdb' and args.startswith('sp!')):
            dec = 4 * len(_regs_in_list(args))
        elif m in ('sub', 'subw') and re.match(r'sp,\s*(sp,\s*)?#', args):
            dec = _imm(args) or 0
        elif m in ('sub', 'subw') and re.match(r'sp,\s*(sp,\s*)?r', args):
            f.dynamic = True
        elif m == 'str' and re.search(r'\[sp, #-(\d+)\]!', args):
            dec = int(re.search(r'\[sp, #-(\d+)\]!', args).group(1))
        if dec:
            if in_prologue:
                f.frame += dec
            else:
                f.late_frame += dec
        if m in ('pop', 'ldmia', 'ldm', 'b', 'bl', 'blx', 'bx', 'cbz', 'cbnz', 'tbb', 'tbh') or \
                (m == 'add' and args.startswith('sp')):
            in_prologue = False
    f.frame += f.late_frame

    # ---- successors
    def target(args):
        t = _TARGET.search(args)
        return int(t.group(1), 16) if t else None

    def successors(i):
        addr, m, cond, args = insns[i]
        nxt = [i + 1] if i + 1 < len(insns) else []
        if m == 'b':
            t = target(args)
            inside = [index[t]] if t in index else []
            return inside + (nxt if cond else [])
        if m in ('cbz', 'cbnz'):
            t = target(args)
            return ([index[t]] if t in index else []) + nxt
        if m == 'bx' or ((m in ('pop', 'ldm', 'ldmia')) and 'pc' in _regs_in_list(args)) or \
                (m.startswith('ldr') and args.startswith('pc')) or (m == 'mov' and args.startswith('pc')):
            return nxt if cond else []
        if m in ('tbb', 'tbh'):
            table = addr + 4
            targets, pos, limit = [], table, None
            while limit is None or pos < limit:
                if m == 'tbb':
                    off, pos = image_byte(pos), pos + 1
                else:
                    off, pos = image_byte(pos) | (image_byte(pos + 1) << 8), pos + 2
                t = table + 2 * off
                targets.append(t)
                limit = t if limit is None else min(limit, t)
                if len(targets) > 1024:
                    break
            return [index[t] for t in targets if t in index]
        return nxt

    # ---- data flow (facts: literal values, vtable pointers and slots), from the entry
    states = [None] * len(insns)
    if insns:
        states[0] = {}
        work = [0]
        while work:
            i = work.pop()
            out = _transfer(insns[i], states[i], word)
            for j in successors(i):
                new = out if states[j] is None else _join(states[j], out)
                if new != states[j]:
                    states[j] = new
                    work.append(j)

    # ---- call sites
    for i, (addr, m, cond, args) in enumerate(insns):
        st = states[i] if states[i] is not None else {}
        if m in ('bl', 'blx') and _TARGET.search(args):
            t = target(args)
            o = owner(t)
            if o is not None:
                f.sites.append((addr, frozenset([o.addr]), False, False))
        elif m in ('blx', 'bx') and not args.startswith('lr'):
            reg = args.strip()
            facts = st.get(reg)
            targets, unresolved = set(), facts is None
            for fact in facts or ():
                if fact[0] == 'lit' and (fact[1] & ~1) in funcs:
                    targets.add(fact[1] & ~1)
                elif fact[0] == 'slot' and f.name.startswith('_Z'):
                    targets.add(('slot', fact[1]))
                else:
                    unresolved = True
            f.sites.append((addr, frozenset(targets), m == 'bx', unresolved))
        elif m in ('b', 'cbz', 'cbnz'):
            t = target(args)
            if t is not None and t not in index:
                o = owner(t)
                if o is not None and o is not f:
                    f.sites.append((addr, frozenset([o.addr]), True, False))


def load(tc, elf, binf, build_path):
    image = open(binf, 'rb').read()
    word = lambda a: int.from_bytes(image[a - FLASH_BASE:a - FLASH_BASE + 4], 'little') \
        if FLASH_BASE <= a <= FLASH_BASE + len(image) - 4 else None
    image_byte = lambda a: image[a - FLASH_BASE] if FLASH_BASE <= a < FLASH_BASE + len(image) else 0

    syms = _symbols(tc, elf)
    starts = sorted(syms)
    funcs = {a: Function(n, a, s) for a, (n, s) in syms.items()}

    def owner(a):
        i = bisect.bisect_right(starts, a) - 1
        if i >= 0:
            fn = funcs[starts[i]]
            if a < fn.addr + max(fn.size, 2):
                return fn
        return None

    current = None
    for line in _run([tc.tool('objdump'), '-d', '--no-show-raw-insn', elf]).splitlines():
        h = _FUNC_HDR.match(line)
        if h:
            current = funcs.get(int(h.group(1), 16))
            continue
        m = _INSN.match(line)
        if not m or current is None:
            continue
        addr, op, args = int(m.group(1), 16), m.group(2), (m.group(3) or '')
        if op.startswith('.') or addr >= current.addr + current.size:
            continue                                    # data in the code: literal pools, jump tables
        mnem, cond = _mnemonic(op)
        current.insns.append((addr, mnem, cond, args.split(';')[0].strip() if mnem != 'ldr' else args))

    for fn in funcs.values():
        _analyze_function(fn, funcs, starts, owner, word, image_byte)
        fn.insns = None

    # ---- frames: the compiler's -fstack-usage figure where it matches one function unambiguously
    su = _su_frames(build_path)
    names = _demangle(tc, [fn.name for fn in funcs.values()])
    by_base = collections.defaultdict(list)
    for fn in funcs.values():
        by_base[_base_name(names[fn.name])].append(fn)
    checked, mismatches = 0, []
    for base, fs in by_base.items():
        entries = su.get(base)
        if not entries or len(fs) != 1 or len(entries) != 1:
            continue
        size, kind = entries[0]
        fn = fs[0]
        checked += 1
        if size != fn.frame:
            mismatches.append((names[fn.name], fn.frame, size))
        fn.frame, fn.frame_source = size, 'su'
        if 'dynamic' in kind:
            fn.dynamic = True

    # ---- C++ virtual calls: the functions in any vtable at that slot (the vtable pointer points
    # 8 bytes into the _ZTV object, past offset-to-top and the typeinfo pointer)
    slots = collections.defaultdict(set)
    for line in _run([tc.tool('readelf'), '-sW', elf]).splitlines():
        parts = line.split()
        if len(parts) >= 8 and parts[3] == 'OBJECT' and parts[7].startswith('_ZTV'):
            addr, size = int(parts[1], 16), int(parts[2], 0)
            for off in range(0, size - 8, 4):
                v = word(addr + 8 + off)
                if v and (v & ~1) in funcs:
                    slots[off].add(v & ~1)

    # ---- the functions an unresolved indirect call may reach: any function whose address is stored
    # in the image, except interrupt vectors (only the hardware calls those), main and Reset_Handler
    vectors = {word(FLASH_BASE + 4 * i) & ~1 for i in range(1, VECTOR_WORDS) if word(FLASH_BASE + 4 * i)}
    taken = set()
    for off in range(0, len(image) - 3, 2):
        v = int.from_bytes(image[off:off + 4], 'little')
        if v & 1 and (v & ~1) in funcs:
            taken.add(v & ~1)
    taken -= vectors
    taken -= {a for a, fn in funcs.items() if fn.name in ('main', 'Reset_Handler')}

    by_simple_name = collections.defaultdict(set)
    for a, fn in funcs.items():
        by_simple_name[_base_name(names[fn.name])].add(a)

    annotated = set()
    for fn in funcs.values():
        resolved = []
        for addr, targets, tail, unresolved in fn.sites:
            real = set()
            for t in targets:
                real |= slots.get(t[1], set()) if isinstance(t, tuple) else {t}
                if isinstance(t, tuple) and not slots.get(t[1]):
                    unresolved = True
            base = _base_name(names[fn.name])
            if unresolved and base in INDIRECT_TARGETS:
                for target_name in INDIRECT_TARGETS[base]:
                    real |= by_simple_name.get(target_name, set())
                annotated.add(base)
                unresolved = False
            resolved.append((addr, frozenset(real), tail, unresolved))
        fn.sites = resolved
    return funcs, names, taken, checked, mismatches, annotated


# ---------------------------------------------------------------- worst-case paths

def _addr2line_tasks(tc, elf, sites, pct_names):
    """For call sites in performContinuousTasks: the task each belongs to, from the inline records:
    the function inlined directly into performContinuousTasks, or None for a direct call (whose
    task is then its callee)."""
    addrs = sorted({a for a, _ in sites})
    if not addrs:
        return {}
    out = subprocess.run([tc.tool('addr2line'), '-a', '-f', '-i', '-C', '-e', elf] + ['0x%x' % a for a in addrs],
                         capture_output=True, text=True, check=True).stdout.splitlines()
    # for each address: the address, then (function, file:line) pairs from the innermost frame out
    chains = {}
    current = None
    i = 0
    while i < len(out):
        if re.match(r'^0x[0-9a-f]+$', out[i]):
            current = int(out[i], 16)
            chains[current] = []
            i += 1
            continue
        chains[current].append(out[i].split('(')[0])
        i += 2
    tasks = {}
    for a in addrs:
        chain = chains.get(a, [])
        outer = [k for k, fn in enumerate(chain) if fn in pct_names]
        if not outer:
            raise RuntimeError('stack: the inline records don\'t place 0x%x in performContinuousTasks' % a)
        k = outer[0]                                # the innermost performContinuousTasks frame
        tasks[a] = chain[k - 1] if k >= 1 else None
    return tasks


def _tarjan(graph):
    index, low, on, stack, out = {}, {}, set(), [], []
    counter = [0]
    for start in graph:
        if start in index:
            continue
        work = [(start, iter(sorted(graph[start], key=str)))]
        index[start] = low[start] = counter[0]
        counter[0] += 1
        stack.append(start)
        on.add(start)
        while work:
            v, it = work[-1]
            advanced = False
            for w in it:
                if w not in graph:
                    continue
                if w not in index:
                    index[w] = low[w] = counter[0]
                    counter[0] += 1
                    stack.append(w)
                    on.add(w)
                    work.append((w, iter(sorted(graph[w], key=str))))
                    advanced = True
                    break
                elif w in on:
                    low[v] = min(low[v], index[w])
            if advanced:
                continue
            work.pop()
            if work:
                low[work[-1][0]] = min(low[work[-1][0]], low[v])
            if low[v] == index[v]:
                comp = []
                while True:
                    w = stack.pop()
                    on.discard(w)
                    comp.append(w)
                    if w == v:
                        break
                out.append(comp)
    return out


def analyze(tc, elf, binf, build_path, without_float_formatting=False):
    funcs, names, taken, checked, mismatches, annotated = load(tc, elf, binf, build_path)
    base = {a: _base_name(names[fn.name]) for a, fn in funcs.items()}
    by_base = collections.defaultdict(set)
    for a, b in base.items():
        by_base[b].add(a)

    # ---- nodes: the functions, plus one node per guarded task of performContinuousTasks
    pct = by_base.get('performContinuousTasks', set())
    scan = by_base.get('modeLoopPerformance', set())
    frame = {a: fn.frame for a, fn in funcs.items()}
    label = {a: names[fn.name] for a, fn in funcs.items()}
    edges = {a: [] for a in funcs}             # (target, tail)
    unresolved_sites = []
    for a, fn in funcs.items():
        for addr, targets, tail, unresolved in fn.sites:
            ts = set(targets)
            if unresolved:
                ts |= taken
                unresolved_sites.append(label[a])
            for t in ts:
                edges[a].append((t, tail))

    infeasible_used = []
    for (caller, callee), reason in INFEASIBLE_CALLS.items():
        found = False
        for a in by_base.get(caller, ()):
            kept = [(t, tail) for t, tail in edges[a] if base.get(t) != callee]
            found |= len(kept) != len(edges[a])
            edges[a] = kept
        infeasible_used.append((caller, callee, reason, found))
    if without_float_formatting:
        dtoa = by_base.get('_dtoa_r', set())
        for name in PRINTF_FAMILY:
            for a in by_base.get(name, ()):
                edges[a] = [(t, tail) for t, tail in edges[a] if t not in dtoa]

    # split performContinuousTasks's calls by task
    pct_sites = [(addr, t) for a in pct for addr, targets, tail, _ in funcs[a].sites for t in targets]
    task_of_site = _addr2line_tasks(tc, elf, pct_sites, {'performContinuousTasks'})
    task_nodes, unguarded = {}, set()
    next_id = -1
    for a in pct:
        new_edges = []
        for addr, targets, tail, _ in funcs[a].sites:
            for t in targets:
                task = task_of_site.get(addr) or base.get(t)
                if t in pct or task not in GUARDED_TASKS:
                    if t not in pct and task not in UNGUARDED_LEAVES:
                        unguarded.add(task)
                    new_edges.append((t, tail))
                    continue
                if task not in task_nodes:
                    task_nodes[task] = next_id
                    frame[next_id], label[next_id], edges[next_id] = 0, '(task %s)' % task, []
                    next_id -= 1
                node = task_nodes[task]
                edges[node].append((t, tail))
                if (node, False) not in new_edges:
                    new_edges.append((node, False))
        edges[a] = new_edges
    guarded = set(task_nodes.values())
    cut = pct | scan | guarded                  # entered only through walk()

    # ---- the graph below the cut: cycles are searched exhaustively, or bounded if big
    plain = {a: {t for t, _ in es if t not in cut} for a, es in edges.items()}
    sccs = _tarjan(plain)
    cyclic = {a for comp in sccs if len(comp) > 1 for a in comp} | {a for a, ts in plain.items() if a in ts}
    big = {}
    for comp in sccs:
        if len(comp) > MAX_EXHAUSTIVE_CYCLE:
            for a in comp:
                big[a] = frozenset(comp)

    NONE = (-1, [])
    memo_depth, memo_entry = {}, {}

    def big_bound(a, inner):
        comp = big[a]
        total = sum(frame[m] for m in comp)
        best = (0, [])
        for m in comp:
            for t, _ in edges[m]:
                if t not in comp and t not in cut:
                    d = inner(t)
                    if d[0] > best[0]:
                        best = d
        return total + max(best[0], 0), [(a, total)] + best[1]

    def depth(a, on_path=frozenset()):
        """(bytes, path): deepest stack below a's entry without entering a cut node."""
        if a in memo_depth:
            return memo_depth[a]
        if a in big:
            result = big_bound(a, lambda t: depth(t))
            for m in big[a]:
                memo_depth[m] = result
            return result
        inner = on_path | {a} if a in cyclic else on_path
        best = (frame[a], [(a, frame[a])])
        for t, tail in edges[a]:
            if t in cut or t in inner:
                continue
            d, p = depth(t, inner)
            here = d if tail else frame[a] + d
            if here > best[0]:
                best = (here, [(a, 0 if tail else frame[a])] + p)
        if a not in cyclic:
            memo_depth[a] = best
        return best

    def entry(a, r, on_path=frozenset()):
        """(bytes, path): the deepest stack below a's entry at which cut node r is called."""
        key = (a, r)
        if key in memo_entry:
            return memo_entry[key]
        if a in big:
            direct = any(t == r for m in big[a] for t, _ in edges[m])
            d, p = big_bound(a, lambda t: entry(t, r))
            result = (d, p) if direct or len(p) > 1 else NONE
            for m in big[a]:
                memo_entry[(m, r)] = result
            return result
        inner = on_path | {a} if a in cyclic else on_path
        best = NONE
        for t, tail in edges[a]:
            if t == r:
                here = 0 if tail else frame[a]
                if here > best[0]:
                    best = (here, [(a, here)])
            elif t not in cut and t not in inner:
                d, p = entry(t, r, inner)
                if d >= 0:
                    here = d if tail else frame[a] + d
                    if here > best[0]:
                        best = (here, [(a, 0 if tail else frame[a])] + p)
        if a not in cyclic:
            memo_entry[key] = best
        return best

    memo_walk = {}
    capped = [False]

    def walk(a, used, scan_left, pct_left, budget=MAX_SCHEDULER_ENTRIES):
        """(bytes, path): deepest stack below a. Guarded tasks in `used` are running already; the
        scan loop and performContinuousTasks may be entered scan_left and pct_left more times
        (performContinuousTasks() calling performContinuousTasks(unsigned long) is one level)."""
        key = (a, used, scan_left, pct_left)
        if key in memo_walk:
            return memo_walk[key]
        best = depth(a)
        for r in cut:
            if r in guarded and r in used:
                continue
            if r in scan and scan_left == 0:
                continue
            new_level = r in pct and a not in pct
            if new_level and pct_left == 0:
                continue
            e, pe = entry(a, r)
            if e < 0:
                continue
            if budget == 0:
                capped[0] = True
                continue
            d, pd = walk(r, used | {r} if r in guarded else used, scan_left - 1 if r in scan else scan_left,
                         pct_left - 1 if new_level else pct_left, budget - 1)
            if e + d > best[0]:
                best = (e + d, pe + [(None, 0)] + pd)
        memo_walk[key] = best
        return best

    def show(path):
        return [('  (enters:)', 0) if a is None else (label[a], f) for a, f in path]

    reset = next(iter(by_base['Reset_Handler']))
    levels = {}
    for pct_levels, scan_levels in set(REPORTED_LEVELS) | {HEADLINE_LEVELS}:
        levels[(pct_levels, scan_levels)] = walk(reset, frozenset(), scan_levels, pct_levels)
    main_d, main_p = levels[HEADLINE_LEVELS]

    def isr(name):
        a = next(iter(by_base.get(name, ())), None)
        if a is None:
            return 0, []
        d, p = walk(a, frozenset(), 0, 0)
        return d, show(p)

    systick, uart, usb = isr('SysTick_Handler'), isr('UART_Handler'), isr('UOTGHS_Handler')
    high_name, high = ('UART_Handler', uart) if uart[0] >= usb[0] else ('UOTGHS_Handler', usb)
    interrupts = EXCEPTION_FRAME + systick[0] + EXCEPTION_FRAME + high[0]

    # the watched functions, as called inside the outer scan loop, with the headline's nesting
    watch = []
    for name in WATCHED:
        matches = by_base.get(name, set())
        if not matches:
            watch.append((name, None))
        else:
            watch.append((name, max(walk(a, frozenset(), HEADLINE_LEVELS[1] - 1,
                                         HEADLINE_LEVELS[0] - (1 if a in pct else 0))[0] for a in matches)))

    return {
        'total': main_d + interrupts,
        'main': main_d, 'main_path': show(main_p),
        'levels': [(k, levels[k][0] + interrupts) for k in REPORTED_LEVELS],
        'level_paths': {k: show(levels[k][1]) for k in REPORTED_LEVELS},
        'headline_levels': HEADLINE_LEVELS,
        'interrupts': interrupts,
        'systick': systick[0], 'systick_path': systick[1],
        'uart': uart[0], 'usb': usb[0], 'high_isr': high_name, 'high_path': high[1],
        'watch': watch,
        'tasks': sorted(task_nodes),
        'unguarded': sorted(unguarded),
        'infeasible': infeasible_used,
        'annotated': sorted(annotated),
        'capped': capped[0],
        'functions': len(funcs),
        'su_checked': checked, 'su_mismatches': mismatches,
        'unresolved_sites': sorted(set(unresolved_sites)),
        'address_taken': len(taken),
        'dynamic': sorted(names[fn.name] for fn in funcs.values() if fn.dynamic),
        'cycles': [sorted(label[a] for a in comp) for comp in sccs if len(comp) > 1],
    }
