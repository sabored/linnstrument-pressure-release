"""The reference toolchain and the reference build.

The reference build is Arduino IDE 1.8.19's arduino-builder with the SAM core 1.6.11 and
arm-none-eabi-gcc 4.8.3-2014q1, board arduino:sam:arduino_due_x_dbg ("Arduino Due (Programming
Port)"), -ide-version=10819, and -libraries pointed at the repository's own libraries folder. It
reproduces the released v0.1.0 .bin byte for byte, from any folder.

The toolchain lives outside the repository, in the folder given by --toolchain, the environment
variable LINNSTRUMENT_TOOLCHAIN, or by default ../toolchain next to the repository. `fw.py setup`
downloads it there: nothing is installed system-wide.
"""
import hashlib
import json
import os
import platform
import shutil
import subprocess
import urllib.request

V010_SHA256 = '0587f532ecd3b98836a849e589747c59d075790b7d1064f4656c714e08507bf0'
V010_TAG = 'v0.1.0'

FQBN = 'arduino:sam:arduino_due_x_dbg'
IDE_VERSION = '10819'
GCC_VERSION = '4.8.3-2014q1'
SAM_VERSION = '1.6.11'

RAM_SIZE = 98304                 # SAM3X8E static RAM (0x20070000-0x20088000)
FLASH_BASE = 0x80000             # the program starts here
SETTINGS_BASE = 0xC0000          # settings live in the second flash bank; the program must end below it
RECIPE_VERSION = 1               # raise when the arduino-builder invocation below changes
PROGRAM_LIMIT = SETTINGS_BASE - FLASH_BASE

# name, url, hash algorithm, digest. The IDE's digest is from arduino-1.8.19.sha512sum.txt, the others
# from Arduino's package index (package_index.json).
_DOWNLOADS = {
    'Darwin': [
        ('ide', 'https://downloads.arduino.cc/arduino-1.8.19-macosx.zip', 'sha512',
         '053b0c1e70da9176680264e40fcb9502f45ca5a879aeb8b6f71282b38bfdb87c63ebc6b88e35ea70a73720ad439d828cc8cb110e4c6ab07357126a36ee396325'),
        ('gcc', 'https://downloads.arduino.cc/gcc-arm-none-eabi-4.8.3-2014q1-mac.tar.gz', 'sha256',
         '3598acf21600f17a8e4a4e8e193dc422b894dc09384759b270b2ece5facb59c2'),
    ],
    'Linux': [
        ('ide', 'https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz', 'sha512',
         '9328abf8778200019ed40d4fc0e6afb03a4cee8baaffbcea7dd3626477e14243f779eaa946c809fb153a542bf2ed60cf11a5f135c91ecccb1243c1387be95328'),
        ('gcc', 'https://downloads.arduino.cc/gcc-arm-none-eabi-4.8.3-2014q1-linux64.tar.gz', 'sha256',
         'd23f6626148396d6ec42a5b4d928955a703e0757829195fa71a939e5b86eecf6'),
    ],
}
_SAM_DOWNLOAD = ('sam', 'https://downloads.arduino.cc/cores/sam-1.6.11.tar.bz2', 'sha256',
                 'fb8e275f39622a5574a11cef85be3ed36a6995c38a19b20de6fb48e9c7f88b70')


class ToolchainError(RuntimeError):
    pass


def default_toolchain_dir(repo):
    return os.environ.get('LINNSTRUMENT_TOOLCHAIN') or os.path.join(os.path.dirname(repo), 'toolchain')


class Toolchain:
    def __init__(self, root):
        self.root = os.path.abspath(root)
        if platform.system() == 'Darwin':
            self.ide = os.path.join(self.root, 'arduino-1.8.19', 'Arduino.app', 'Contents', 'Java')
        else:
            self.ide = os.path.join(self.root, 'arduino-1.8.19')
        self.builder = os.path.join(self.ide, 'arduino-builder')
        self.packages = os.path.join(self.root, 'packages')
        self.gcc_bin = os.path.join(self.packages, 'arduino', 'tools', 'arm-none-eabi-gcc', GCC_VERSION, 'bin')
        self.sam = os.path.join(self.packages, 'arduino', 'hardware', 'sam', SAM_VERSION)
        self._fingerprint = None

    def tool(self, name):
        return os.path.join(self.gcc_bin, 'arm-none-eabi-' + name)

    def check(self):
        missing = [p for p in (self.builder, self.tool('g++'), os.path.join(self.sam, 'platform.txt'))
                   if not os.path.exists(p)]
        if missing:
            raise ToolchainError('reference toolchain not found in %s (missing %s).\n'
                                 'Run `python3 tools/fw.py setup` to download it there, or point '
                                 '--toolchain / LINNSTRUMENT_TOOLCHAIN at an existing one.'
                                 % (self.root, ', '.join(os.path.relpath(m, self.root) for m in missing)))
        return self

    def fingerprint(self):
        """Identifies the toolchain actually on disk: a hash of the path, size and modification time
        of every file of arduino-builder, ctags, the IDE's platform files, the SAM core and the
        compiler, plus the build recipe's version. A different, updated or re-installed toolchain
        gets a different fingerprint, so cached results and build folders aren't reused across
        toolchains."""
        if self._fingerprint is None:
            h = hashlib.sha256(('recipe %d\n' % RECIPE_VERSION).encode())
            count = 0
            for root in (self.builder, os.path.join(self.ide, 'tools-builder'), os.path.join(self.ide, 'hardware'),
                         self.packages):
                if os.path.isfile(root):
                    files = [root]
                else:
                    files = []
                    for d, dirs, names in os.walk(root):
                        dirs.sort()
                        files.extend(os.path.join(d, n) for n in sorted(names))
                for f in files:
                    st = os.lstat(f)
                    h.update(('%s\0%d\0%d\n' % (os.path.relpath(f, self.root), st.st_size, int(st.st_mtime))).encode())
                    count += 1
            self._fingerprint = '%s (%d files)' % (h.hexdigest()[:16], count)
        return self._fingerprint


# ---------------------------------------------------------------- setup

def _hash_file(path, algo):
    h = hashlib.new(algo)
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def setup_plan():
    system = platform.system()
    if system not in _DOWNLOADS:
        raise ToolchainError('fw.py setup supports macOS and Linux x86_64 only')
    if system == 'Linux' and platform.machine() not in ('x86_64', 'AMD64'):
        raise ToolchainError('the reference toolchain has no Linux %s build' % platform.machine())
    return _DOWNLOADS[system] + [_SAM_DOWNLOAD]


def setup(root, log=print):
    """Downloads, verifies and unpacks the reference toolchain into root."""
    plan = setup_plan()
    downloads = os.path.join(root, 'downloads')
    os.makedirs(downloads, exist_ok=True)
    files = {}
    for name, url, algo, digest in plan:
        dest = os.path.join(downloads, url.rsplit('/', 1)[1])
        if not (os.path.exists(dest) and _hash_file(dest, algo) == digest):
            log('downloading %s' % url)
            tmp = dest + '.part'
            with urllib.request.urlopen(url) as r, open(tmp, 'wb') as f:
                shutil.copyfileobj(r, f)
            os.replace(tmp, dest)
        got = _hash_file(dest, algo)
        if got != digest:
            raise ToolchainError('%s: %s mismatch (got %s, expected %s)' % (dest, algo, got, digest))
        log('verified %s (%s)' % (os.path.basename(dest), algo))
        files[name] = dest

    tc = Toolchain(root)
    ide_parent = os.path.join(root, 'arduino-1.8.19')
    if not os.path.exists(tc.builder):
        os.makedirs(ide_parent, exist_ok=True)
        if files['ide'].endswith('.zip'):
            subprocess.run(['unzip', '-q', '-o', files['ide'], '-d', ide_parent], check=True)
        else:
            subprocess.run(['tar', '-xf', files['ide'], '-C', root], check=True)   # contains arduino-1.8.19/
    gcc_parent = os.path.dirname(tc.gcc_bin.rstrip('/'))
    if not os.path.exists(tc.tool('g++')):
        tools_dir = os.path.dirname(gcc_parent)
        os.makedirs(tools_dir, exist_ok=True)
        subprocess.run(['tar', '-xf', files['gcc'], '-C', tools_dir], check=True)
        os.rename(os.path.join(tools_dir, 'gcc-arm-none-eabi-' + GCC_VERSION), gcc_parent)
    if not os.path.exists(os.path.join(tc.sam, 'platform.txt')):
        hw = os.path.dirname(tc.sam)
        os.makedirs(hw, exist_ok=True)
        subprocess.run(['tar', '-xf', files['sam'], '-C', hw], check=True)
        os.rename(os.path.join(hw, 'sam'), tc.sam)
    return tc.check()


# ---------------------------------------------------------------- building

def _builder_args(tc, sketch_dir, build_path, warnings, prefs):
    ino = os.path.join(sketch_dir, 'linnstrument-firmware.ino')
    args = [tc.builder,
            '-logger=human',
            '-hardware', os.path.join(tc.ide, 'hardware'),
            '-hardware', tc.packages,
            '-tools', os.path.join(tc.ide, 'tools-builder'),
            '-tools', os.path.join(tc.ide, 'hardware', 'tools', 'avr'),
            '-tools', tc.packages,
            '-built-in-libraries', os.path.join(tc.ide, 'libraries'),
            '-libraries', os.path.join(sketch_dir, 'libraries'),
            '-fqbn=' + FQBN,
            '-ide-version=' + IDE_VERSION,
            '-build-path', build_path,
            '-warnings=' + warnings,
            '-prefs=build.warn_data_percentage=75']
    args += ['-prefs=' + p for p in prefs]
    return args, ino


def compile_sketch(tc, sketch_dir, build_path, warnings='none', prefs=()):
    """Runs arduino-builder -compile. Returns the combined build log. A build folder left by another
    toolchain is emptied first, since arduino-builder would reuse its compiled core."""
    marker = os.path.join(build_path, 'toolchain.txt')
    if os.path.isdir(build_path):
        try:
            same = open(marker).read() == tc.fingerprint()
        except OSError:
            same = False
        if not same:
            shutil.rmtree(build_path)
    os.makedirs(build_path, exist_ok=True)
    with open(marker, 'w') as f:
        f.write(tc.fingerprint())
    args, ino = _builder_args(tc, sketch_dir, build_path, warnings, prefs)
    r = subprocess.run(args + ['-compile', ino], capture_output=True, text=True)
    log = r.stdout + r.stderr
    with open(os.path.join(build_path, 'build.log'), 'w') as f:
        f.write(log)
    if r.returncode != 0:
        errors = [l for l in log.splitlines() if 'error' in l.lower()]
        raise ToolchainError('arduino-builder failed (log in %s):\n%s' % (
            os.path.join(build_path, 'build.log'), '\n'.join((errors or log.splitlines())[-30:])))
    return log


def preprocess_sketch(tc, sketch_dir, build_path):
    """Runs arduino-builder -preprocess: returns the path of the combined sketch (.ino.cpp), which is
    identical to the one a -compile builds, prototypes and #line directives included."""
    os.makedirs(build_path, exist_ok=True)
    args, ino = _builder_args(tc, sketch_dir, build_path, 'none', ())
    r = subprocess.run(args + ['-preprocess', ino], capture_output=True, text=True)
    if r.returncode != 0:
        raise ToolchainError('arduino-builder -preprocess failed:\n' + (r.stdout + r.stderr)[-3000:])
    return os.path.join(build_path, 'sketch', 'linnstrument-firmware.ino.cpp')


def sizes(tc, elf):
    """Section sizes of the linked .elf, as `arm-none-eabi-size -A` reports them."""
    out = subprocess.run([tc.tool('size'), '-A', elf], capture_output=True, text=True, check=True).stdout
    sec = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0].startswith('.') and parts[1].isdigit():
            sec[parts[0]] = int(parts[1])
    return sec


def summarize(tc, build_path):
    elf = os.path.join(build_path, 'linnstrument-firmware.ino.elf')
    binf = os.path.join(build_path, 'linnstrument-firmware.ino.bin')
    with open(binf, 'rb') as f:
        data = f.read()
    sec = sizes(tc, elf)
    static_ram = sec.get('.relocate', 0) + sec.get('.bss', 0)
    return {
        'bin': binf,
        'elf': elf,
        'sha256': hashlib.sha256(data).hexdigest(),
        'bin_size': len(data),
        'program_end': FLASH_BASE + len(data),
        'flash_free': PROGRAM_LIMIT - len(data),
        'data': sec.get('.relocate', 0),
        'bss': sec.get('.bss', 0),
        'static_ram': static_ram,
        'ram_free': RAM_SIZE - static_ram,
    }


def save_json(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=1, sort_keys=True)


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None
