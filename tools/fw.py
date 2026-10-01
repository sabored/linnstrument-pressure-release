#!/usr/bin/env python3
"""LinnStrument firmware tooling: the reference build, warnings, stack depth, and the desktop build.

  python3 tools/fw.py setup                       download the reference toolchain (once per computer)
  python3 tools/fw.py verify                      full rebuild of v0.1.0, checked against the released SHA-256
  python3 tools/fw.py build    [--ref REF]        reference build: .bin, SHA-256, flash and RAM
  python3 tools/fw.py warnings [--ref REF] [--against BASE]
  python3 tools/fw.py stack    [--ref REF] [-v]   static stack depth, without and with re-entry
  python3 tools/fw.py report   [--ref REF] [--against BASE] [--ram-budget N]
                                                  before/after: RAM, stack, .bin size, warnings diff
  python3 tools/fw.py desktop  [--ref REF] [--run [--settings EXPORT] [--calibration as-exported|stand-in]
                               [--seconds S] [--midi-log FILE]] [--harness FILE]
  python3 tools/fw.py layout   [--ref REF]        struct layouts: desktop build vs instrument build
  python3 tools/fw.py replay   [--ref REF] [--against BASE | --baseline DIR | --no-compare] --recordings DIR
                               [--config NAME]... [--recording NAME]... [--repeat] [--sanitize] [--save DIR]
                                                  replay sensor recordings through the firmware, compare MIDI

Without --ref, commands work on the working tree, uncommitted changes included. BASE defaults to
the merge-base of HEAD with fork/main, which is the build the current branch started from.
Output goes to build/fw/ in the repository (ignored by git). See tools/README.md.
"""
import argparse
import os
import shutil
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fwlib import arduino, desktop, layout, replay, sketch, stack, warnings  # noqa: E402

REPO = sketch.repo_root(os.path.dirname(os.path.abspath(__file__)))
STACK_BUILD_PREFS = ('compiler.cpp.extra_flags=-fstack-usage', 'compiler.c.extra_flags=-fstack-usage')
MIN_MARGIN = 256          # bytes of RAM that must be left after the stack without re-entry


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------- builds, cached per commit

_RUN_CACHE = {}          # builds done during this run, so the working tree is built once


def reference_build(tc, target):
    """The release build. Cached for commits."""
    key = (target.label, 'reference')
    if key in _RUN_CACHE:
        return _RUN_CACHE[key]
    _RUN_CACHE[key] = result = _reference_build(tc, target)
    return result


def _reference_build(tc, target):
    out = os.path.join(target.work_dir(), 'reference')
    cached = arduino.load_json(os.path.join(out, 'result.json'))
    if target.commit and cached and cached.get('toolchain') == tc.fingerprint():
        cached['path'] = out
        return cached
    src = target.stage()
    log('reference build of %s' % target.describe())
    arduino.compile_sketch(tc, src, out, warnings='none')
    result = arduino.summarize(tc, out)
    result['toolchain'] = tc.fingerprint()
    result['target'] = target.describe()
    result['path'] = out
    arduino.save_json(os.path.join(out, 'result.json'), result)
    return result


def analysis_build(tc, target):
    """The same build with -Wall -Wextra and -fstack-usage. Neither changes the code, so its .bin must
    be identical to the reference build's; that's checked, so warnings and stack depth describe the
    real binary."""
    key = (target.label, 'analysis')
    if key in _RUN_CACHE:
        return _RUN_CACHE[key]
    _RUN_CACHE[key] = result = _analysis_build(tc, target)
    return result


def _analysis_build(tc, target):
    out = os.path.join(target.work_dir(), 'analysis')
    ref = reference_build(tc, target)
    cached = arduino.load_json(os.path.join(out, 'result.json'))
    if not (target.commit and cached and cached.get('toolchain') == tc.fingerprint()):
        src = target.stage()
        log('analysis build (-Wall -Wextra, -fstack-usage) of %s' % target.describe())
        arduino.compile_sketch(tc, src, out, warnings='all', prefs=STACK_BUILD_PREFS)
        cached = arduino.summarize(tc, out)
        cached['toolchain'] = tc.fingerprint()
        arduino.save_json(os.path.join(out, 'result.json'), cached)
    if cached['sha256'] != ref['sha256']:
        raise RuntimeError('the analysis build of %s differs from its reference build (%s vs %s); '
                           'warnings and stack figures would not describe the real .bin'
                           % (target.describe(), cached['sha256'][:12], ref['sha256'][:12]))
    with open(os.path.join(out, 'build.log')) as f:
        cached['log'] = f.read()
    cached['path'] = out
    return cached


def stack_result(tc, target, without_float_formatting=False):
    a = analysis_build(tc, target)
    return stack.analyze(tc, a['elf'], a['bin'], a['path'], without_float_formatting)


# ---------------------------------------------------------------- commands

def cmd_setup(args):
    root = args.toolchain
    log('reference toolchain folder: %s' % root)
    for name, url, algo, _ in arduino.setup_plan():
        log('  %s  (checked against its %s)' % (url, algo))
    arduino.setup(root, log)
    log('toolchain ready. Check it with: python3 tools/fw.py verify')


def print_build(r):
    log('  .bin:        %s' % r['bin'])
    log('  SHA-256:     %s' % r['sha256'])
    log('  .bin size:   %d bytes, program ends at 0x%X; %d bytes left before the settings at 0x%X'
        % (r['bin_size'], r['program_end'], r['flash_free'], arduino.SETTINGS_BASE))
    log('  static RAM:  %d bytes (.data %d + .bss %d); %d of %d bytes left for the stack and heap'
        % (r['static_ram'], r['data'], r['bss'], r['ram_free'], arduino.RAM_SIZE))
    if r['flash_free'] < 0:
        log('  ERROR: the program overlaps the settings in the second flash bank')


def cmd_build(args):
    tc = arduino.Toolchain(args.toolchain).check()
    target = sketch.Target(REPO, args.ref)
    r = reference_build(tc, target)
    log('%s:' % target.describe())
    print_build(r)
    if r['sha256'] == arduino.V010_SHA256:
        log('  (identical to the released v0.1.0)')


def cmd_verify(args):
    """Always a full build from scratch, with nothing cached."""
    tc = arduino.Toolchain(args.toolchain).check()
    target = sketch.Target(REPO, arduino.V010_TAG)
    src = target.stage()
    out = os.path.join(REPO, 'build', 'fw', 'verify')
    if os.path.exists(out):
        shutil.rmtree(out)
    log('full build of %s with the toolchain in %s (fingerprint %s)' % (target.describe(), tc.root, tc.fingerprint()))
    arduino.compile_sketch(tc, src, out, warnings='none')
    r = arduino.summarize(tc, out)
    ok = r['sha256'] == arduino.V010_SHA256
    log('%s: SHA-256 %s' % (target.describe(), r['sha256']))
    log('expected:        %s' % arduino.V010_SHA256)
    log('OK: the reference build reproduces the released v0.1.0 .bin' if ok else
        'MISMATCH: this toolchain does not reproduce v0.1.0; don\'t use it for release builds')
    return 0 if ok else 1


def base_target(args):
    if args.against:
        return sketch.Target(REPO, args.against)
    commit, via = sketch.default_base(REPO)
    t = sketch.Target(REPO, commit)
    t.ref = 'merge-base with ' + via
    return t


def warning_diff(tc, before, after):
    wb = warnings.parse(analysis_build(tc, before)['log'])
    wa = warnings.parse(analysis_build(tc, after)['log'])
    added, removed = warnings.diff(wb, wa)
    return wb, wa, added, removed


def print_warning_diff(wb, wa, added, removed):
    log('warnings (-Wall -Wextra): %d before, %d after' % (len(wb), len(wa)))
    if added:
        log('NEW warnings (%d):' % len(added))
        for w in added:
            log('  + ' + warnings.format_warning(w))
    else:
        log('new warnings: none')
    if removed:
        log('warnings gone (%d):' % len(removed))
        for w in removed:
            log('  - ' + warnings.format_warning(w))


def cmd_warnings(args):
    tc = arduino.Toolchain(args.toolchain).check()
    target = sketch.Target(REPO, args.ref)
    if args.list:
        ws = warnings.parse(analysis_build(tc, target)['log'])
        for w in ws:
            log(warnings.format_warning(w))
        log('%d warnings in %s' % (len(ws), target.describe()))
        return 0
    base = base_target(args)
    log('comparing %s with %s' % (target.describe(), base.describe()))
    wb, wa, added, removed = warning_diff(tc, base, target)
    print_warning_diff(wb, wa, added, removed)
    return 1 if added else 0


def levels_name(k):
    pct, scan = k
    return '%d performContinuousTasks level%s, %d scan loop level%s' % (pct, '' if pct == 1 else 's',
                                                                        scan, '' if scan == 1 else 's')


def print_stack(r, verbose, no_float=None, ram_free=None):
    log('stack without re-entry (%s): %d bytes' % (levels_name(r['headline_levels']), r['total']))
    log('  main thread %d + SysTick %d + %s %d + 2 exception frames of %d'
        % (r['main'], r['systick'], r['high_isr'], max(r['uart'], r['usb']), stack.EXCEPTION_FRAME))
    if no_float is not None:
        log('  %d bytes without newlib\'s floating-point formatting (%%e, %%f, %%g), which only a float '
            'format reaches' % no_float['total'])
    log('  with re-entry (nothing but the task flags bounds it, so there is no static worst case):')
    for k, total in r['levels']:
        left = '' if ram_free is None else ', %+d left of the %d free' % (ram_free - total, ram_free)
        log('    %-52s %5d bytes%s' % (levels_name(k), total, left))
    log('  stack below the watched functions (called from the scan loop, no re-entry):')
    for name, d in r['watch']:
        log('    %-26s %s' % (name, '%d' % d if d is not None else 'inlined'))
    log('  %d functions; frames of %d checked against -fstack-usage, %d differ'
        % (r['functions'], r['su_checked'], len(r['su_mismatches'])))
    log('  tasks of performContinuousTasks: %s; unguarded: %s'
        % (', '.join(r['tasks']), ', '.join(r['unguarded']) or 'none'))
    for caller, callee, reason, found in r['infeasible']:
        log('  excluded: %s > %s (%s)%s' % (caller, callee, reason, '' if found else ' -- NOT FOUND, check it'))
    log('  indirect calls resolved by annotation: %s' % (', '.join(r['annotated']) or 'none'))
    log('  indirect calls that may reach any of the %d functions whose address is stored: %s'
        % (r['address_taken'], ', '.join(r['unresolved_sites']) or 'none'))
    if r['dynamic']:
        log('  dynamic stack frames (not bounded): %s' % ', '.join(r['dynamic']))
    if r['capped']:
        log('  WARNING: the nesting search hit its safety bound; the figures may be too low')
    if verbose:
        log('deepest main-thread path without re-entry ("(enters:)" marks a scheduler or task entry):')
        for name, frame in r['main_path']:
            log('  %5d  %s' % (frame, name))
        log('deepest SysTick path: %s' % ' > '.join('%s (%d)' % p for p in r['systick_path']))
        log('deepest %s path: %s' % (r['high_isr'], ' > '.join('%s (%d)' % p for p in r['high_path'])))
        for c in r['cycles']:
            log('call cycle (each function walked once): %s' % ', '.join(c))
        for n, a, b in r['su_mismatches']:
            log('frame differs: %s: prologue %d, -fstack-usage %d (used)' % (n, a, b))


def cmd_stack(args):
    tc = arduino.Toolchain(args.toolchain).check()
    target = sketch.Target(REPO, args.ref)
    r = stack_result(tc, target)
    log('%s:' % target.describe())
    print_stack(r, args.verbose, stack_result(tc, target, without_float_formatting=True),
                reference_build(tc, target)['ram_free'])


def cmd_report(args):
    tc = arduino.Toolchain(args.toolchain).check()
    after = sketch.Target(REPO, args.ref)
    before = base_target(args)
    rb, ra = reference_build(tc, before), reference_build(tc, after)
    sb, sa = stack_result(tc, before), stack_result(tc, after)
    wb, wa, added, removed = warning_diff(tc, before, after)

    reentry_b = dict(sb['levels']).get((2, 2))
    reentry_a = dict(sa['levels']).get((2, 2))
    rows = [
        ('static RAM (.data + .bss)', rb['static_ram'], ra['static_ram']),
        ('RAM left for stack + heap', rb['ram_free'], ra['ram_free']),
        ('stack, no re-entry', sb['total'], sa['total']),
        ('RAM left after that stack', rb['ram_free'] - sb['total'], ra['ram_free'] - sa['total']),
        ('stack, one re-entry of each', reentry_b, reentry_a),
        ('.bin size', rb['bin_size'], ra['bin_size']),
        ('flash left before 0xC0000', rb['flash_free'], ra['flash_free']),
        ('warnings (-Wall -Wextra)', len(wb), len(wa)),
    ]
    log('')
    log('before: %s' % before.describe())
    log('after:  %s' % after.describe())
    log('')
    log('%-34s %10s %10s %8s' % ('bytes', 'before', 'after', 'change'))
    for name, b, a in rows:
        log('%-34s %10d %10d %+8d' % (name, b, a, a - b))
    log('')
    changed = [(n, b, a) for (n, b), (_, a) in zip(sb['watch'], sa['watch']) if b != a]
    log('stack below watched functions: %s' % ('unchanged' if not changed else ', '.join(
        '%s %s -> %s' % (n, b, a) for n, b, a in changed)))
    log('SHA-256 before: %s' % rb['sha256'])
    log('SHA-256 after:  %s%s' % (ra['sha256'], '  (identical)' if ra['sha256'] == rb['sha256'] else ''))
    print_warning_diff(wb, wa, added, removed)

    failures = []
    if added:
        failures.append('new warnings')
    if ra['flash_free'] < 0:
        failures.append('the program overlaps the settings at 0xC0000')
    if ra['ram_free'] - sa['total'] < MIN_MARGIN:
        failures.append('the RAM left after the stack without re-entry is %d bytes, under the %d required'
                        % (ra['ram_free'] - sa['total'], MIN_MARGIN))
    if args.ram_budget is not None and ra['static_ram'] - rb['static_ram'] > args.ram_budget:
        failures.append('static RAM grew by %d bytes, over the budget of %d'
                        % (ra['static_ram'] - rb['static_ram'], args.ram_budget))
    if sa['capped'] or any(not found for _, _, _, found in sa['infeasible']):
        failures.append('the stack analysis needs attention (see fw.py stack)')
    if failures:
        log('FAILED: %s' % '; '.join(failures))
        return 1
    return 0


def cmd_desktop(args):
    tc = arduino.Toolchain(args.toolchain).check()
    target = sketch.Target(REPO, args.ref)
    src = target.stage()
    out = os.path.join(target.work_dir(), 'desktop')
    result = desktop.build(tc, src, out, harness=os.path.abspath(args.harness), log=log)
    changed = {f: c for f, c in result['rewrites'].items() if any(c.values())}
    log('desktop build of %s: %s' % (target.describe(), result['exe']))
    log('  32-bit long rewrites: %s' % ', '.join('%s %s' % (f, c) for f, c in changed.items()))
    log('  busy-waits whose clock reads take one pass each: %s'
        % ', '.join('%s (%d reads)' % (f, n) for f, n in result['busy_waits'].items()))
    log('  known firmware memory errors contained (fwlib/memfix.py): %s%s' % (', '.join(result['contained']) or 'none',
        '; not in this version of the sketch: ' + ', '.join(result['not_contained']) if result['not_contained'] else ''))
    if args.run:
        provision, run = desktop.boot_and_run(result['exe'], out, settings=args.settings, seconds=args.seconds,
                                              midi_log=os.path.abspath(args.midi_log) if args.midi_log else None,
                                              calibration=args.calibration)
        log('provisioning run (erased flash%s):' % (', settings restored' if args.settings else ''))
        for line in provision.splitlines():
            log('  ' + line)
        log('run (boot from the provisioned flash):')
        for line in run.splitlines():
            log('  ' + line)


def cmd_layout(args):
    tc = arduino.Toolchain(args.toolchain).check()
    target = sketch.Target(REPO, args.ref)
    ref = reference_build(tc, target)
    src = target.stage()
    out = os.path.join(target.work_dir(), 'desktop')
    result = desktop.build(tc, src, out, log=log)
    # the folders each build read the sketch's files from: the staged .ino files (named by #line), and
    # the .h files arduino-builder copies next to the combined sketch / the desktop build rewrites
    instrument_dirs = [src, os.path.join(ref['path'], 'sketch')]
    desktop_dirs = [src, result['src_dir']]
    r = layout.compare(ref['elf'], result['sketch_object'], instrument_dirs, desktop_dirs)
    log('%s: %d structs identical in the desktop and instrument builds' % (target.describe(), len(r['identical'])))
    for name, size in r['identical']:
        if args.verbose or name in ('Configuration', 'SequencerProject', 'TouchInfo'):
            log('  %-40s %6d bytes' % (name, size))
    for name, diffs in r['pointer']:
        log('  differs only through pointers (8 bytes on the desktop): %s' % name)
    for name, diffs in r['differ']:
        log('  DIFFERS: %s' % name)
        for d in diffs:
            log('      ' + d)
    for name in r['missing']:
        log('  MISSING from one build: %s' % name)
    return 1 if r['differ'] or r['missing'] else 0


def cmd_replay(args):
    tc = arduino.Toolchain(args.toolchain).check()
    recordings = replay.find_recordings(args.recordings, args.recording)
    calibration_from = args.calibration_from or os.path.join(args.recordings, 'linnstrument_settings.bin')
    plan = replay.Plan(recordings, calibration_from, configs=args.config, events_dir=os.path.abspath(args.events_dir),
                       timing=args.timing)
    target = sketch.Target(REPO, args.ref)
    log('recordings: %s (%s), with their sensor settings: %s'
        % (os.path.abspath(args.recordings), ', '.join(r.name for r in recordings), ', '.join('%s %s' % s for s in plan.sensor)))
    log('configurations: %s (%s)' % (os.path.relpath(plan.configurations_file), ', '.join(c.name for c in plan.configs)))
    log('calibration from: %s' % plan.calibration_from)
    log('scripted events: %s' % (', '.join(os.path.relpath(plan.events_file(r)) for r in recordings
                                           if plan.events_file(r)) or 'none'))
    fresh = args.fresh or args.repeat
    results = replay.replay(tc, target, plan, jobs=args.jobs, log=log, fresh=fresh)

    compared = None
    if args.baseline:
        compared = 'the logs saved in %s' % args.baseline
        base_logs = {(r.config.name, r.name): os.path.join(args.baseline, r.config.name, r.name, 'midi.txt') for r in results}
    elif not args.no_compare:
        base = base_target(args)
        compared = base.describe()
        base_results = replay.replay(tc, base, plan, jobs=args.jobs, log=log, fresh=args.fresh)
        base_logs = {(r.config.name, r.name): r.midi for r in base_results}

    log('')
    log('replayed: %s' % target.describe())
    contained = replay.contained(target)
    log('memory errors contained (fwlib/memfix.py): %s' % (', '.join(contained) or 'none'))
    if compared and not args.baseline and replay.contained(base) != contained:
        log('NOTE: %s had different ones contained: %s; a difference in runs that reach them can come from that'
            % (base.describe(), ', '.join(replay.contained(base)) or 'none'))
    if compared:
        log('compared with: %s (midi.txt, byte for byte)' % compared)
    log('')
    log('%-18s %-14s %6s %9s %8s  %s' % ('configuration', 'run', 'notes', 'messages', 'served', 'midi.txt' if compared else ''))
    reports, different, overflows, left_sounding = [], 0, 0, 0
    for r in results:
        notes, messages, served, overflow, sounding = r.summary_values()
        overflows += overflow
        left_sounding += sounding > 0
        verdict = ''
        if compared:
            diff = replay.compare_files(base_logs[(r.config.name, r.name)], r.midi)
            if diff is None:
                verdict = 'identical'
            else:
                different += 1
                verdict = 'DIFFERENT from line %d' % diff[0]
                reports.append(['%s, %s:' % (r.config.name, r.name)] + diff[1])
        log('%-18s %-14s %6d %9d %8s  %s%s%s' % (r.config.name, r.name, notes, messages, served, verdict,
                                                 '  (hammer-on list overflow, contained)' if overflow else '',
                                                 '  (%d notes left sounding)' % sounding if sounding else ''))
    for report in reports:
        log('')
        for line in report:
            log(line)
    log('')
    log('logs: %s/<configuration>/<run>/' % os.path.relpath(os.path.join(target.work_dir(), 'replay')))
    if overflows:
        log('%d runs overflow microLinn\'s hammer-on list, a firmware bug; the desktop build contains it '
            '(tools/fwlib/memfix.py)' % overflows)
    if left_sounding:
        log('%d runs leave notes sounding: a note-on with no note-off after it on its channel, by the end of the '
            'run (run.txt, "midi check")' % left_sounding)
    failed = []
    if compared:
        log('midi.txt: %s' % ('identical in all %d runs' % len(results) if not different else
                              '%d of %d runs differ' % (different, len(results))))
        if different:
            failed.append('the MIDI differs')

    if args.repeat:
        checks = replay.repeat_check(tc, target, plan, results, jobs=args.jobs, log=log)
        bad = [(name, what) for name, what in checks if what]
        for name, what in bad:
            log('NOT REPEATABLE: %s: %s differs' % (name, what))
        log('second build, provisioning and replay: %s' % (
            'identical flash images and files (settings.txt, provision.txt, midi.txt, touches.txt, run.txt) in all %d runs' % len(checks)
            if not bad else '%d of %d runs differ' % (len(bad), len(checks))))
        if bad:
            failed.append('a second replay differs')

    if args.sanitize:
        checks = replay.sanitize_check(tc, target, plan, results, jobs=args.jobs, log=log)
        bad = [(name, what) for name, what in checks if what]
        for name, what in bad:
            log('SANITIZER: %s: %s' % (name, what))
        log('AddressSanitizer build: %s' % (
            'no memory error, and midi.txt and touches.txt identical to the normal build\'s in all %d runs' % len(checks)
            if not bad else '%d of %d runs failed (their logs are in %s)'
            % (len(bad), len(checks), os.path.relpath(os.path.join(target.work_dir(), 'replay', 'sanitize')))))
        if bad:
            failed.append('the sanitizer check failed')

    if args.save:
        head = sketch.git(REPO, 'rev-parse', 'HEAD')
        description = ('Replay logs of %s, made by tools/fw.py replay with the tools at HEAD %s.\n'
                       'recordings: %s, with their sensor settings: %s\n'
                       'calibration from: %s\nconfigurations (%s):\n%s\n'
                       'midi.txt and touches.txt are gzipped; SHA256SUMS lists them uncompressed.\n'
                       % (target.describe(), head, ', '.join(r.name for r in recordings),
                          ', '.join('%s %s' % s for s in plan.sensor), os.path.basename(plan.calibration_from),
                          os.path.relpath(plan.configurations_file, REPO),
                          '\n'.join('  %s: %s' % (c.name, c.description) for c in plan.configs)))
        replay.save(results, args.save, description, log=log)

    if failed:
        log('FAILED: %s' % '; '.join(failed))
        return 1
    return 0

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--toolchain', default=arduino.default_toolchain_dir(REPO),
                   help='reference toolchain folder (default: $LINNSTRUMENT_TOOLCHAIN or ../toolchain)')
    sub = p.add_subparsers(dest='command')
    sub.required = True

    sub.add_parser('setup', help='download the reference toolchain').set_defaults(func=cmd_setup)
    sub.add_parser('verify', help='rebuild v0.1.0 and check its SHA-256').set_defaults(func=cmd_verify)

    s = sub.add_parser('build', help='reference build')
    s.add_argument('--ref', help='git commit, branch or tag (default: the working tree)')
    s.set_defaults(func=cmd_build)

    s = sub.add_parser('warnings', help='-Wall -Wextra warnings, compared with the previous build')
    s.add_argument('--ref')
    s.add_argument('--against', help='the previous build (default: merge-base of HEAD with fork/main)')
    s.add_argument('--list', action='store_true', help='just list the warnings')
    s.set_defaults(func=cmd_warnings)

    s = sub.add_parser('stack', help='worst-case stack depth')
    s.add_argument('--ref')
    s.add_argument('-v', '--verbose', action='store_true')
    s.set_defaults(func=cmd_stack)

    s = sub.add_parser('report', help='before/after numbers for a change')
    s.add_argument('--ref')
    s.add_argument('--against')
    s.add_argument('--ram-budget', type=int, help='fail if static RAM grows by more than this many bytes')
    s.set_defaults(func=cmd_report)

    s = sub.add_parser('desktop', help='desktop build of the whole sketch')
    s.add_argument('--ref')
    s.add_argument('--harness', default=desktop.DEFAULT_HARNESS, help='harness source (default: tools/desktop/smoke.cpp)')
    s.add_argument('--run', action='store_true', help='boot it and run it')
    s.add_argument('--settings', help='a settings export to restore when provisioning (export_settings.py)')
    s.add_argument('--calibration', choices=('as-exported', 'stand-in'), default='as-exported',
                   help='settings without a calibration: store them as they are (default), or give them a '
                        'stand-in calibration, which turns on the code only a calibrated instrument runs')
    s.add_argument('--seconds', type=float, default=3.0, help='virtual seconds to run (default 3)')
    s.add_argument('--midi-log', help='write the MIDI it sends to this file')
    s.set_defaults(func=cmd_desktop)

    s = sub.add_parser('layout', help='compare struct layouts of the desktop and instrument builds')
    s.add_argument('--ref')
    s.add_argument('-v', '--verbose', action='store_true')
    s.set_defaults(func=cmd_layout)

    s = sub.add_parser('replay', help='replay sensor recordings through the firmware and compare the MIDI')
    s.add_argument('--ref', help='the firmware to replay (default: the working tree)')
    s.add_argument('--against', help='compare with this firmware (default: merge-base of HEAD with fork/main)')
    s.add_argument('--baseline', help='compare with the logs saved in this folder (--save) instead')
    s.add_argument('--no-compare', action='store_true', help='just replay')
    s.add_argument('--recordings', default=os.environ.get('LINNSTRUMENT_RECORDINGS'),
                   help='folder with the recordings (NAME_samples.csv, NAME_settings.csv) and, by default, the '
                        'settings export (default: $LINNSTRUMENT_RECORDINGS)')
    s.add_argument('--calibration-from', help='settings export the runs take their calibration from, and nothing '
                                              'else (default: RECORDINGS/linnstrument_settings.bin)')
    s.add_argument('--recording', action='append', help='replay only this recording (repeatable)')
    s.add_argument('--config', action='append', help='replay only this configuration (repeatable; see '
                                                     'tools/replay/configurations.txt)')
    s.add_argument('--events-dir', default=replay.EVENTS_DIR,
                   help='folder with the scripted events, NAME.events (default: tools/replay)')
    s.add_argument('--timing', help='the clock model\'s costs, ADC_NS,PASS_NS (see desktop/hal.h)')
    s.add_argument('--repeat', action='store_true',
                   help='build, provision and replay everything a second time and check it is identical')
    s.add_argument('--sanitize', action='store_true',
                   help='also run everything with AddressSanitizer: no memory error, and the same logs')
    s.add_argument('--save', help='copy the logs to this folder, gzipped, with SHA256SUMS')
    s.add_argument('--fresh', action='store_true', help='ignore cached results')
    s.add_argument('--jobs', type=int, help='runs in parallel (default: the number of CPUs)')
    s.set_defaults(func=cmd_replay)

    args = p.parse_args()
    try:
        return args.func(args) or 0
    except (arduino.ToolchainError, RuntimeError) as e:
        print('error: %s' % e, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.setrecursionlimit(20000)
    sys.exit(main())
