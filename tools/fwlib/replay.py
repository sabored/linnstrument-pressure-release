"""Replays: sensor recordings played through the whole firmware on the desktop build, with scripted
events, in a set of configurations, and the MIDI of two firmware versions compared byte for byte.

A replay run is one configuration, one recording, and the recording's scripted events or none:
  1. the desktop build of the firmware with the replay harness (desktop/replay.cpp);
  2. per configuration, a provisioned flash image: the settings export restored the way the
     LinnStrument Updater does, with the recordings' sensor settings, the configuration's own
     changes and its calibration choice (desktop/harness.h);
  3. the replay, which boots from that image and plays the recording: midi.txt (what the comparison
     looks at), touches.txt and run.txt (the summary).

The same firmware, harness, recording, events and settings always give the same logs, byte for byte.
Results are cached per build label (a commit, or the working tree) under a key made of everything
that goes into them, so a run is only repeated when something it depends on has changed.
"""
import concurrent.futures
import gzip
import hashlib
import json
import os
import shutil
import subprocess

from . import desktop

TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HARNESS = os.path.join(desktop.DESKTOP_DIR, 'replay.cpp')
EVENTS_DIR = os.path.join(TOOLS_DIR, 'replay')
CACHE_VERSION = 1           # raise when the layout or meaning of the cached results changes

# name, calibration, settings changed from the export (desktop/harness.h, SETTINGS), description
CONFIGURATIONS = (
    ('export', 'as-exported', (), 'the settings export as it is'),
    ('col1', 'as-exported', (('colOffset', 1),), 'column offset 1'),
    ('no-edo', 'as-exported', (('edo', 4),), 'EDO off'),
    ('no-edo-col1', 'as-exported', (('edo', 4), ('colOffset', 1)), 'EDO off, column offset 1'),
    ('one-channel', 'as-exported', (('midiMode', 0),), 'One Channel'),
    ('hammer-ons', 'as-exported', (('hammerOnMode', 1), ('hammerOnZone', 20), ('hammerOnWait', 0)),
     'hammer-ons R (highest note wins), zone 200 cents, no wait'),
    ('calibrated', 'stand-in', (), 'the calibration data marked as valid, as a calibration would'),
)
CONFIG_NAMES = [c[0] for c in CONFIGURATIONS]

# a recording's sensor settings (NAME_settings.csv) and the named settings they become
SENSOR_SETTINGS = (
    ('sensor_sensitivity_z', 'sensorSensitivityZ'),
    ('sensor_lo_z', 'sensorLoZ'),
    ('sensor_feather_z', 'sensorFeatherZ'),
    ('sensor_range_z', 'sensorRangeZ'),
    ('pressure_sensitivity', 'pressureSensitivity'),
    ('velocity_sensitivity', 'velocitySensitivity'),
    ('pressure_aftertouch', 'pressureAftertouch'),
)
DESKTOP_MODEL = 200         # the hardware model reports a LinnStrument 200 (pin 38 HIGH)


class Recording:
    def __init__(self, folder, name):
        self.name = name
        self.prefix = os.path.join(folder, name)
        self.samples = self.prefix + '_samples.csv'
        self.settings_csv = self.prefix + '_settings.csv'
        self.sensor = self._sensor_settings()

    def _sensor_settings(self):
        with open(self.settings_csv) as f:
            lines = [l.strip().split(',') for l in f if l.strip()]
        header, rows = lines[0], lines[1:]
        if not rows:
            raise RuntimeError('%s has no settings' % self.settings_csv)
        values = set(tuple(r) for r in (tuple(row[header.index(c)] for c in ['model'] + [s[0] for s in SENSOR_SETTINGS])
                                        for row in rows))
        if len(values) != 1:
            raise RuntimeError('%s: the sensor settings change during the recording' % self.settings_csv)
        model, *rest = next(iter(values))
        if int(model) != DESKTOP_MODEL:
            raise RuntimeError('%s was recorded on a LinnStrument %s; the desktop build models a %d'
                               % (self.name, model, DESKTOP_MODEL))
        return tuple((name, int(v)) for (_, name), v in zip(SENSOR_SETTINGS, rest))


def find_recordings(folder, names=None):
    if not folder or not os.path.isdir(folder):
        raise RuntimeError('no recordings folder: pass --recordings DIR or set LINNSTRUMENT_RECORDINGS '
                           '(the folder with NAME_samples.csv and NAME_settings.csv)')
    found = sorted(f[:-len('_samples.csv')] for f in os.listdir(folder) if f.endswith('_samples.csv')
                   and os.path.exists(os.path.join(folder, f[:-len('_samples.csv')] + '_settings.csv')))
    if names:
        missing = [n for n in names if n not in found]
        if missing:
            raise RuntimeError('no recording %s in %s (found: %s)' % (', '.join(missing), folder, ', '.join(found)))
        found = [n for n in found if n in names]
    if not found:
        raise RuntimeError('no recordings (NAME_samples.csv with NAME_settings.csv) in %s' % folder)
    return [Recording(folder, n) for n in found]


class Plan:
    """What to replay: configurations x recordings x (no events, the recording's events)."""

    def __init__(self, recordings, settings_export, configs=None, events_dir=EVENTS_DIR, timing=None,
                 lead_in_ms=1000, tail_ms=5000):
        self.recordings = recordings
        self.settings_export = os.path.abspath(settings_export)
        if not os.path.exists(self.settings_export):
            raise RuntimeError('no settings export at %s (pass --settings)' % self.settings_export)
        unknown = [c for c in (configs or []) if c not in CONFIG_NAMES]
        if unknown:
            raise RuntimeError('unknown configuration %s (known: %s)' % (', '.join(unknown), ', '.join(CONFIG_NAMES)))
        self.configs = [c for c in CONFIGURATIONS if not configs or c[0] in configs]
        self.events_dir = events_dir
        self.timing = timing
        self.lead_in_ms = lead_in_ms
        self.tail_ms = tail_ms
        sensors = set(r.sensor for r in recordings)
        if len(sensors) != 1:
            raise RuntimeError('the recordings were made with different sensor settings; replay them separately')
        self.sensor = sensors.pop()

    def events_file(self, recording):
        path = os.path.join(self.events_dir, recording.name + '.events')
        return path if os.path.exists(path) else None

    def runs(self):
        """[(config, recording, events file or None, run name)]"""
        out = []
        for config in self.configs:
            for rec in self.recordings:
                out.append((config, rec, None, rec.name))
                ev = self.events_file(rec)
                if ev:
                    out.append((config, rec, ev, rec.name + '+events'))
        return out

    def provision_args(self, config):
        _, calibration, changes, _ = config
        args = ['--calibration', calibration]
        for name, value in self.sensor + changes:
            args += ['--set', '%s=%d' % (name, value)]
        return args

    def replay_args(self):
        args = ['--lead-in', str(self.lead_in_ms), '--tail', str(self.tail_ms)]
        if self.timing:
            args += ['--timing', self.timing]
        return args


def _hash_file(h, path, label):
    h.update(('%s\n' % label).encode())
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)


def _hash_tree(h, root):
    for d, dirs, names in os.walk(root):
        dirs[:] = sorted(x for x in dirs if x != '__pycache__')
        for n in sorted(names):
            if n != '.staged' and not n.endswith('.pyc'):
                path = os.path.join(d, n)
                _hash_file(h, path, os.path.relpath(path, root))


def _compiler_id():
    r = subprocess.run([desktop.compiler(), '--version'], capture_output=True, text=True)
    return r.stdout


def common_key(tc, sketch_dir, plan):
    """Everything that goes into every run: the firmware, the tools that build and run it, the
    toolchain that preprocesses it, the compiler, the settings export and the plan's parameters."""
    h = hashlib.sha256(('replay cache %d\n' % CACHE_VERSION).encode())
    h.update(('toolchain %s\ncompiler %s\n' % (tc.fingerprint(), _compiler_id())).encode())
    _hash_tree(h, sketch_dir)
    _hash_tree(h, desktop.DESKTOP_DIR)
    for f in ('desktop.py', 'longfix.py', 'arduino.py', 'replay.py'):
        _hash_file(h, os.path.join(TOOLS_DIR, 'fwlib', f), f)
    _hash_file(h, plan.settings_export, 'settings export')
    h.update(json.dumps([plan.sensor, plan.replay_args()]).encode())
    return h.hexdigest()


def run_key(common, plan, config, rec, events):
    h = hashlib.sha256(common.encode())
    h.update(json.dumps([config[0], plan.provision_args(config), rec.name]).encode())
    _hash_file(h, rec.samples, 'samples')
    _hash_file(h, rec.settings_csv, 'settings')
    if events:
        _hash_file(h, events, 'events')
    return h.hexdigest()


def _run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError('%s failed (exit %d):\n%s' % (' '.join(os.path.basename(c) for c in cmd[:1]) + ' ' +
                                                         ' '.join(cmd[1:]), r.returncode, (r.stdout + r.stderr)[-3000:]))
    return r.stdout


class Result:
    def __init__(self, config, rec, events, name, folder, cached):
        self.config, self.rec, self.events, self.name, self.folder, self.cached = config, rec, events, name, folder, cached
        self.midi = os.path.join(folder, 'midi.txt')
        self.touches = os.path.join(folder, 'touches.txt')
        self.summary = os.path.join(folder, 'run.txt')

    def summary_values(self):
        """notes, messages, % of recorded samples served, from run.txt"""
        notes = messages = served = None
        with open(self.summary) as f:
            for line in f:
                if line.startswith('midi: '):
                    parts = line.split()
                    messages, notes = int(parts[1]), int(parts[3])
                elif ' recorded samples served (' in line:
                    served = line.split('(')[1].split(')')[0]
        return notes, messages, served


def replay(tc, target, plan, jobs=None, log=print, fresh=False):
    """Builds the target with the replay harness and runs every run of the plan, reusing cached
    results whose key hasn't changed (never with fresh). Returns [Result]."""
    root = os.path.join(target.work_dir(), 'replay')
    src = target.stage()
    common = common_key(tc, src, plan)
    runs = plan.runs()
    keys = [run_key(common, plan, c, r, e) for c, r, e, _ in runs]
    results, todo = [], []
    for (config, rec, events, name), key in zip(runs, keys):
        folder = os.path.join(root, config[0], name)
        done = not fresh and _read(os.path.join(folder, 'key.txt')) == key and os.path.exists(os.path.join(folder, 'run.txt'))
        results.append(Result(config, rec, events, name, folder, done))
        if not done:
            todo.append((results[-1], key))
    if not todo:
        log('replay of %s: all %d runs cached' % (target.describe(), len(runs)))
        return results

    log('replay of %s: building with the replay harness' % target.describe())
    exe = build(tc, src, os.path.join(root, 'desktop'))
    configs = sorted(set(r.config for r, _ in todo), key=lambda c: CONFIG_NAMES.index(c[0]))
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs or os.cpu_count()) as pool:
        images = dict(pool.map(lambda c: (c[0], provision(exe, plan, c, os.path.join(root, c[0]))), configs))
        log('replay of %s: %d runs (%d cached)' % (target.describe(), len(todo), len(runs) - len(todo)))

        def run(item):
            result, key = item
            run_replay(exe, images[result.config[0]], plan, result.rec, result.events, result.folder)
            with open(os.path.join(result.folder, 'key.txt'), 'w') as f:
                f.write(key + '\n')
        list(pool.map(run, todo))
    return results


def build(tc, src, out):
    return desktop.build(tc, src, out, harness=HARNESS, log=lambda m: None)['exe']


def provision(exe, plan, config, folder):
    """Provisions the configuration's flash image in folder; returns its path."""
    os.makedirs(folder, exist_ok=True)
    image = os.path.join(folder, 'flash.bin')
    out = _run([exe, '--provision', '--restore', plan.settings_export] + plan.provision_args(config) +
               ['--flash-out', image])
    with open(os.path.join(folder, 'provision.txt'), 'w') as f:
        f.write(out)
    return image


def run_replay(exe, image, plan, rec, events, folder):
    if os.path.exists(folder):
        shutil.rmtree(folder)
    os.makedirs(folder)
    cmd = [exe, '--flash-in', image, '--recording', rec.prefix, '--midi-log', os.path.join(folder, 'midi.txt'),
           '--touch-log', os.path.join(folder, 'touches.txt')] + plan.replay_args()
    if events:
        cmd += ['--events', events]
    out = _run(cmd)
    with open(os.path.join(folder, 'run.txt'), 'w') as f:
        f.write(out)


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def _open_text(path):
    if os.path.exists(path):
        return open(path, 'rb')
    if os.path.exists(path + '.gz'):
        return gzip.open(path + '.gz', 'rb')
    return None


def compare_files(a, b, context=3, shown=6):
    """None if identical, else (line number, lines of a report) for the first difference."""
    fa, fb = _open_text(a), _open_text(b)
    if fa is None or fb is None:
        return 0, ['missing: %s' % (a if fa is None else b)]
    with fa, fb:
        la, lb = fa.read().splitlines(), fb.read().splitlines()
    if la == lb:
        return None
    n = next((i for i, (x, y) in enumerate(zip(la, lb)) if x != y), min(len(la), len(lb)))
    report = ['  first difference at line %d (%d lines before, %d after):' % (n + 1, len(la), len(lb))]
    for i in range(max(0, n - context), n):
        report.append('      %s' % la[i].decode(errors='replace'))
    for line in la[n:n + shown]:
        report.append('    - %s' % line.decode(errors='replace'))
    for line in lb[n:n + shown]:
        report.append('    + %s' % line.decode(errors='replace'))
    return n + 1, report


def repeat_check(tc, target, plan, results, jobs=None, log=print):
    """Builds the target again, provisions every configuration again and runs every replay again, in
    a scratch folder, and compares the flash images and every log with the first run's, byte for
    byte. Returns [(name, None or what differs)], one per configuration and one per run."""
    root = os.path.join(target.work_dir(), 'replay')
    scratch = os.path.join(root, 'repeat')
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    log('replay of %s: building, provisioning and replaying everything a second time' % target.describe())
    exe = build(tc, target.stage(), os.path.join(scratch, 'desktop'))
    configs = sorted(set(r.config for r in results), key=lambda c: CONFIG_NAMES.index(c[0]))
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs or os.cpu_count()) as pool:
        def image(config):
            second = provision(exe, plan, config, os.path.join(scratch, config[0]))
            same = _file_hash(second) == _file_hash(os.path.join(root, config[0], 'flash.bin'))
            return '%s flash image' % config[0], None if same else 'flash.bin'
        out = list(pool.map(image, configs))

        def again(result):
            folder = os.path.join(scratch, result.config[0], result.name)
            run_replay(exe, os.path.join(root, result.config[0], 'flash.bin'), plan, result.rec, result.events, folder)
            for f in ('midi.txt', 'touches.txt', 'run.txt'):
                if _file_hash(os.path.join(folder, f)) != _file_hash(os.path.join(result.folder, f)):
                    return '%s %s' % (result.config[0], result.name), f
            return '%s %s' % (result.config[0], result.name), None
        out += list(pool.map(again, results))
    shutil.rmtree(scratch)
    return out


def _file_hash(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def save(results, folder, description, log=print):
    """Copies the logs into folder/CONFIG/RUN/ (the logs gzipped, the summary as it is), with
    SHA256SUMS of the uncompressed logs and a manifest."""
    os.makedirs(folder, exist_ok=True)
    sums = []
    for r in results:
        dest = os.path.join(folder, r.config[0], r.name)
        os.makedirs(dest, exist_ok=True)
        for f in ('midi.txt', 'touches.txt'):
            src = os.path.join(r.folder, f)
            with open(src, 'rb') as fi, gzip.GzipFile(os.path.join(dest, f + '.gz'), 'wb', mtime=0) as fo:
                shutil.copyfileobj(fi, fo)
            sums.append('%s  %s' % (_file_hash(src), os.path.join(r.config[0], r.name, f)))
        shutil.copy(r.summary, os.path.join(dest, 'run.txt'))
    with open(os.path.join(folder, 'SHA256SUMS'), 'w') as f:
        f.write('\n'.join(sums) + '\n')
    with open(os.path.join(folder, 'manifest.txt'), 'w') as f:
        f.write(description)
    log('saved %d runs to %s' % (len(results), folder))
