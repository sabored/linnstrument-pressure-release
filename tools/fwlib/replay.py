"""Replays: sensor recordings played through the whole firmware on the desktop build, with scripted
events, in a set of test configurations, and the MIDI of two firmware versions compared byte for byte.

A replay run is one configuration, one recording, and the recording's scripted events or none:
  1. the desktop build of the firmware with the replay harness (desktop/replay.cpp);
  2. the run's flash image: the firmware's own defaults, the calibration of a settings export, then
     the configuration's settings (replay/configurations.txt) and the events file's starting settings,
     stored before the firmware boots (desktop/harness.h, provisionTestSettings());
  3. the replay, which boots from that image, checks that those settings are in effect, and plays the
     recording: midi.txt (what the comparison looks at), touches.txt and run.txt (the summary).

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
CONFIGURATIONS_FILE = os.path.join(EVENTS_DIR, 'configurations.txt')
CACHE_VERSION = 2           # raise when the layout or meaning of the cached results changes

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


class Config:
    def __init__(self, name, calibration, settings, description, recordings=None, events=None):
        self.name, self.calibration, self.settings, self.description = name, calibration, settings, description
        self.recordings = recordings            # the recordings it applies to, or None for all
        self.events = events                    # NAME: replayed only with RECORDING.NAME.events


def _words(line):
    code, _, comment = line.partition('#')
    return code.split(), comment.strip()


def read_configurations(path=CONFIGURATIONS_FILE):
    """Returns (base settings, [Config]); settings are (name, value) pairs of strings."""
    base, configs = [], []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            words, comment = _words(line)
            if not words:
                continue
            pairs = []
            for w in words[1:] if words[0] == 'base' else words[2:]:
                if '=' not in w:
                    raise RuntimeError('%s:%d: expected SETTING=VALUE, not %s' % (path, n, w))
                pairs.append(tuple(w.split('=', 1)))
            if words[0] == 'base':
                base += pairs
            elif words[0] == 'config' and len(words) >= 2:
                calibration = 'export'
                recordings = events = None
                settings = []
                for k, v in pairs:
                    if k == 'calibration':
                        if v not in ('export', 'other'):
                            raise RuntimeError('%s:%d: calibration is export or other' % (path, n))
                        calibration = v
                    elif k == 'recordings':
                        recordings = v.split(',')
                    elif k == 'events':
                        events = v
                    else:
                        settings.append((k, v))
                if any(c.name == words[1] for c in configs):
                    raise RuntimeError('%s:%d: configuration %s defined twice' % (path, n, words[1]))
                configs.append(Config(words[1], calibration, settings, comment, recordings, events))
            else:
                raise RuntimeError('%s:%d: expected `base SETTING=VALUE...` or `config NAME [SETTING=VALUE...]`' % (path, n))
    if not configs:
        raise RuntimeError('%s defines no configuration' % path)
    return base, configs


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
        values = set(tuple(row[header.index(c)] for c in ['model'] + [s[0] for s in SENSOR_SETTINGS]) for row in rows)
        if len(values) != 1:
            raise RuntimeError('%s: the sensor settings change during the recording' % self.settings_csv)
        model, *rest = next(iter(values))
        if int(model) != DESKTOP_MODEL:
            raise RuntimeError('%s was recorded on a LinnStrument %s; the desktop build models a %d'
                               % (self.name, model, DESKTOP_MODEL))
        return tuple((name, str(int(v))) for (_, name), v in zip(SENSOR_SETTINGS, rest))


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

    def __init__(self, recordings, calibration_from, configs=None, events_dir=EVENTS_DIR, timing=None,
                 lead_in_ms=1000, tail_ms=5000, configurations_file=CONFIGURATIONS_FILE):
        self.recordings = recordings
        self.calibration_from = os.path.abspath(calibration_from) if calibration_from else None
        if self.calibration_from and not os.path.exists(self.calibration_from):
            raise RuntimeError('no settings export at %s (it gives the runs their calibration; pass --calibration-from)'
                               % self.calibration_from)
        self.configurations_file = configurations_file
        self.base, all_configs = read_configurations(configurations_file)
        names = [c.name for c in all_configs]
        unknown = [c for c in (configs or []) if c not in names]
        if unknown:
            raise RuntimeError('unknown configuration %s (known: %s)' % (', '.join(unknown), ', '.join(names)))
        self.configs = [c for c in all_configs if not configs or c.name in configs]
        self.all_configs = all_configs
        self.config_names = names
        self.events_dir = events_dir
        self.timing = timing
        self.lead_in_ms = lead_in_ms
        self.tail_ms = tail_ms
        sensors = set(r.sensor for r in recordings)
        if len(sensors) != 1:
            raise RuntimeError('the recordings were made with different sensor settings; replay them separately')
        self.sensor = sensors.pop()

    def events_file(self, recording, config=None):
        """The recording's events file, or a configuration's own (events=NAME: RECORDING.NAME.events)."""
        name = recording.name + ('.' + config.events if config is not None and config.events else '') + '.events'
        path = os.path.join(self.events_dir, name)
        return path if os.path.exists(path) else None

    def runs(self):
        """[(config, recording, events file or None, run name)]; a configuration with its own events
        (events=NAME) is replayed only with them, as RECORDING+NAME."""
        out = []
        for config in self.configs:
            for rec in self.recordings:
                if config.recordings is not None and rec.name not in config.recordings:
                    continue
                ev = self.events_file(rec, config)
                if config.events:
                    if not ev:
                        raise RuntimeError('configuration %s: no %s' % (config.name, os.path.join(
                            self.events_dir, '%s.%s.events' % (rec.name, config.events))))
                    out.append((config, rec, ev, rec.name + '+' + config.events))
                    continue
                out.append((config, rec, None, rec.name))
                if ev:
                    out.append((config, rec, ev, rec.name + '+events'))
        return out

    def settings(self, config):
        """The settings a configuration sets, in order: base, the recordings' sensor settings, its own."""
        return list(self.base) + list(self.sensor) + list(config.settings)

    def provision_args(self, config, events):
        args = ['--calibration', config.calibration]
        if self.calibration_from:
            args += ['--calibration-from', self.calibration_from]
        for name, value in self.settings(config):
            args += ['--set', '%s=%s' % (name, value)]
        if events:
            args += ['--events', events]
        return args

    def replay_args(self, config):
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
    toolchain that preprocesses it, the compiler, and the settings export the calibration comes from."""
    h = hashlib.sha256(('replay cache %d\n' % CACHE_VERSION).encode())
    h.update(('toolchain %s\ncompiler %s\n' % (tc.fingerprint(), _compiler_id())).encode())
    _hash_tree(h, sketch_dir)
    _hash_tree(h, desktop.DESKTOP_DIR)
    for f in ('desktop.py', 'longfix.py', 'memfix.py', 'arduino.py', 'replay.py'):
        _hash_file(h, os.path.join(TOOLS_DIR, 'fwlib', f), f)
    if plan.calibration_from:
        _hash_file(h, plan.calibration_from, 'settings export')
    return h.hexdigest()


def run_key(common, plan, config, rec, events):
    h = hashlib.sha256(common.encode())
    h.update(json.dumps([config.name, plan.provision_args(config, None), plan.replay_args(config), rec.name]).encode())
    _hash_file(h, rec.samples, 'samples')
    _hash_file(h, rec.settings_csv, 'settings')
    if events:
        _hash_file(h, events, 'events')
    return h.hexdigest()


class RunFailed(RuntimeError):
    def __init__(self, cmd, code, output):
        RuntimeError.__init__(self, '%s failed (exit %d):\n%s' % (' '.join([os.path.basename(cmd[0])] + cmd[1:]), code,
                                                                  output[-3000:]))
        self.output = output


def _run(cmd, env=None):
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RunFailed(cmd, r.returncode, r.stdout + r.stderr)
    return r.stdout


class Result:
    def __init__(self, config, rec, events, name, folder, cached):
        self.config, self.rec, self.events, self.name, self.folder, self.cached = config, rec, events, name, folder, cached
        self.midi = os.path.join(folder, 'midi.txt')
        self.touches = os.path.join(folder, 'touches.txt')
        self.summary = os.path.join(folder, 'run.txt')

    def summary_values(self):
        """notes, messages, % of recorded samples served, whether the hammer-on list overflowed, how many
        notes were left sounding at the end, how many note-ons doubled a sounding note, whether the
        channel bucket still counted channels in use at the end, and how many untouched pads were still
        lit as played (run.txt)"""
        notes = messages = served = None
        overflow = counts_left = False
        lights = 0
        sounding = doubled = 0
        with open(self.summary) as f:
            for line in f:
                if line.startswith('midi: '):
                    parts = line.split()
                    messages, notes = int(parts[1]), int(parts[3])
                elif ' recorded samples served (' in line:
                    served = line.split('(')[1].split(')')[0]
                elif line.startswith('firmware check:') and 'OVERFLOW' in line:
                    overflow = True
                elif line.startswith('midi check: ') and ' left sounding ' in line:
                    sounding = int(line.split()[2])
                elif line.startswith('midi check: ') and ' doubled note-ons ' in line:
                    doubled = int(line.split()[2])
                elif line.startswith('firmware check: channel counts') and 'LEFT IN USE' in line:
                    counts_left = True
                elif line.startswith('firmware check: played lights'):
                    lights = int(line.split(':')[2].split()[0])
        return notes, messages, served, overflow, sounding, doubled, counts_left, lights


def build(tc, src, out, sanitize=False):
    return desktop.build(tc, src, out, harness=HARNESS, log=lambda m: None, sanitize=sanitize)['exe']


def run_one(exe, plan, config, rec, events, folder, env=None):
    """Provisions the run's flash image in folder, and replays it there."""
    if os.path.exists(folder):
        shutil.rmtree(folder)
    os.makedirs(folder)
    image = os.path.join(folder, 'flash.bin')
    settings = os.path.join(folder, 'settings.txt')
    out = _run([exe, '--provision'] + plan.provision_args(config, events) + ['--flash-out', image,
                                                                              '--settings-out', settings], env)
    with open(os.path.join(folder, 'provision.txt'), 'w') as f:
        f.write(out)
    cmd = [exe, '--flash-in', image, '--recording', rec.prefix, '--expect-file', settings,
           '--midi-log', os.path.join(folder, 'midi.txt'), '--touch-log', os.path.join(folder, 'touches.txt')] + plan.replay_args(config)
    if events:
        cmd += ['--events', events]
    out = _run(cmd, env)
    with open(os.path.join(folder, 'run.txt'), 'w') as f:
        f.write(out)


def replay(tc, target, plan, jobs=None, log=print, fresh=False):
    """Builds the target with the replay harness and runs every run of the plan, reusing cached
    results whose key hasn't changed (never with fresh). Returns [Result]."""
    root = os.path.join(target.work_dir(), 'replay')
    if os.path.isdir(root):                     # results of configurations or runs that no longer exist
        for d in os.listdir(root):
            if d not in plan.config_names and d not in ('desktop', 'repeat', 'sanitize'):
                shutil.rmtree(os.path.join(root, d))
        for c in plan.all_configs:
            folder = os.path.join(root, c.name)
            for run in os.listdir(folder) if os.path.isdir(folder) else []:
                path = os.path.join(folder, run)
                if not os.path.isdir(path):
                    os.remove(path)
                elif c.recordings is not None and run.split('+')[0] not in c.recordings:
                    shutil.rmtree(path)
    src = target.stage()
    common = common_key(tc, src, plan)
    runs = plan.runs()
    results, todo = [], []
    for config, rec, events, name in runs:
        key = run_key(common, plan, config, rec, events)
        folder = os.path.join(root, config.name, name)
        done = not fresh and _read(os.path.join(folder, 'key.txt')) == key and os.path.exists(os.path.join(folder, 'run.txt'))
        results.append(Result(config, rec, events, name, folder, done))
        if not done:
            todo.append((results[-1], key))
    if not todo:
        log('replay of %s: all %d runs cached' % (target.describe(), len(runs)))
        return results

    log('replay of %s: building with the replay harness' % target.describe())
    exe = build(tc, src, os.path.join(root, 'desktop'))
    log('replay of %s: %d runs (%d cached)' % (target.describe(), len(todo), len(runs) - len(todo)))

    def run(item):
        result, key = item
        run_one(exe, plan, result.config, result.rec, result.events, result.folder)
        with open(os.path.join(result.folder, 'key.txt'), 'w') as f:
            f.write(key + '\n')
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs or os.cpu_count()) as pool:
        list(pool.map(run, todo))
    return results


def contained(target):
    """The memory errors contained in the target's replay build (fwlib/memfix.py)."""
    text = _read(os.path.join(target.work_dir(), 'replay', 'desktop', 'contained.txt'))
    return text.splitlines() if text else []


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


def _file_hash(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def repeat_check(tc, target, plan, results, jobs=None, log=print):
    """Builds the target again, then for every run provisions its flash image again and replays that
    second image, in a scratch folder, and compares the image and every log with the first run's, byte
    for byte. Returns [(run, None or what differs)]."""
    root = os.path.join(target.work_dir(), 'replay')
    scratch = os.path.join(root, 'repeat')
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    log('replay of %s: building, provisioning and replaying everything a second time' % target.describe())
    exe = build(tc, target.stage(), os.path.join(scratch, 'desktop'))

    def again(result):
        folder = os.path.join(scratch, result.config.name, result.name)
        run_one(exe, plan, result.config, result.rec, result.events, folder)
        for f in ('flash.bin', 'settings.txt', 'provision.txt', 'midi.txt', 'touches.txt', 'run.txt'):
            if _file_hash(os.path.join(folder, f)) != _file_hash(os.path.join(result.folder, f)):
                return '%s %s' % (result.config.name, result.name), f
        return '%s %s' % (result.config.name, result.name), None
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs or os.cpu_count()) as pool:
        out = list(pool.map(again, results))
    shutil.rmtree(scratch)
    return out


SANITIZER_ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0')


def sanitize_check(tc, target, plan, results, jobs=None, log=print):
    """Builds the target with AddressSanitizer and repeats every run with it. The sanitizer stops a run
    at any access outside a variable or allocation; and since it also moves every variable, logs that
    differ from the normal build's show behaviour that depends on where variables are. Returns
    [(run, None or what went wrong)]; a failed run's logs are kept in replay/sanitize/."""
    root = os.path.join(target.work_dir(), 'replay')
    scratch = os.path.join(root, 'sanitize')
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    log('replay of %s: building with AddressSanitizer and replaying everything with it' % target.describe())
    exe = build(tc, target.stage(), os.path.join(scratch, 'desktop'), sanitize=True)

    def check(result):
        name = '%s %s' % (result.config.name, result.name)
        folder = os.path.join(scratch, result.config.name, result.name)
        try:
            run_one(exe, plan, result.config, result.rec, result.events, folder, env=SANITIZER_ENV)
        except RunFailed as e:
            lines = [l for l in e.output.splitlines() if 'ERROR: AddressSanitizer' in l or l.strip().startswith('#')]
            return name, 'the sanitizer stopped it:\n      ' + '\n      '.join(lines[:12] or e.output.splitlines()[-12:])
        for f in ('midi.txt', 'touches.txt'):
            if _file_hash(os.path.join(folder, f)) != _file_hash(os.path.join(result.folder, f)):
                return name, '%s differs from the normal build\'s' % f
        shutil.rmtree(folder)
        return name, None
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs or os.cpu_count()) as pool:
        out = list(pool.map(check, results))
    if not any(what for _, what in out):
        shutil.rmtree(scratch)
    return out


def save(results, folder, description, log=print):
    """Copies the logs into folder/CONFIG/RUN/ (the logs gzipped, the summaries as they are), with
    SHA256SUMS of the uncompressed logs and a manifest."""
    if os.path.exists(folder) and os.listdir(folder):
        if not os.path.exists(os.path.join(folder, 'manifest.txt')):
            raise RuntimeError('%s exists and isn\'t a previous --save; choose another folder' % folder)
        shutil.rmtree(folder)
    os.makedirs(folder, exist_ok=True)
    sums = []
    for r in results:
        dest = os.path.join(folder, r.config.name, r.name)
        os.makedirs(dest, exist_ok=True)
        for f in ('midi.txt', 'touches.txt'):
            src = os.path.join(r.folder, f)
            with open(src, 'rb') as fi, gzip.GzipFile(os.path.join(dest, f + '.gz'), 'wb', mtime=0) as fo:
                shutil.copyfileobj(fi, fo)
            sums.append('%s  %s' % (_file_hash(src), os.path.join(r.config.name, r.name, f)))
        for f in ('run.txt', 'provision.txt', 'settings.txt'):
            shutil.copy(os.path.join(r.folder, f), os.path.join(dest, f))
    with open(os.path.join(folder, 'SHA256SUMS'), 'w') as f:
        f.write('\n'.join(sums) + '\n')
    with open(os.path.join(folder, 'manifest.txt'), 'w') as f:
        f.write(description)
    log('saved %d runs to %s' % (len(results), folder))
