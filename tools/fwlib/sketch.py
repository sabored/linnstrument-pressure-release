"""Staging: copying a version of the sketch into a build folder.

arduino-builder needs the sketch folder to be named after its main file (linnstrument-firmware), and
the repository folder may be named anything. So every build first copies the sketch into
build/fw/<label>/src/linnstrument-firmware/: either a git commit (git archive) or the working tree
(every tracked or untracked file that .gitignore doesn't exclude, with uncommitted edits).
Builds are byte-identical whatever folder they're built in, so staging doesn't change the .bin.
"""
import os
import shutil
import subprocess

SKETCH_NAME = 'linnstrument-firmware'


def git(repo, *args, check=True):
    r = subprocess.run(['git', '-C', repo] + list(args), capture_output=True, text=True)
    if check and r.returncode != 0:
        raise RuntimeError('git %s failed: %s' % (' '.join(args), r.stderr.strip()))
    return r.stdout.strip() if r.returncode == 0 else None


def repo_root(path):
    return git(path, 'rev-parse', '--show-toplevel')


def resolve(repo, ref):
    """Full commit hash of ref."""
    return git(repo, 'rev-parse', '--verify', ref + '^{commit}')


def default_base(repo):
    """The build this branch started from: the merge-base of HEAD with the fork's main, else with main."""
    for candidate in ('fork/main', 'origin/main', 'main'):
        if git(repo, 'rev-parse', '--verify', '--quiet', candidate + '^{commit}', check=False):
            base = git(repo, 'merge-base', 'HEAD', candidate, check=False)
            if base:
                return base, candidate
    raise RuntimeError('no base found: pass --against REF')


class Target:
    """A version of the sketch to build: a commit, or the working tree (commit None)."""

    def __init__(self, repo, ref=None):
        self.repo = repo
        self.ref = ref
        self.commit = resolve(repo, ref) if ref else None
        self.label = self.commit[:12] if self.commit else 'worktree'
        self._staged = None

    def describe(self):
        if not self.commit:
            head = git(self.repo, 'rev-parse', '--short=12', 'HEAD')
            dirty = git(self.repo, 'status', '--porcelain', '--untracked-files=normal')
            return 'working tree (HEAD %s%s)' % (head, ', with changes' if dirty else '')
        return '%s (%s)' % (self.ref, self.commit[:12])

    def work_dir(self):
        return os.path.join(self.repo, 'build', 'fw', self.label)

    def stage(self):
        """Copies the sketch into work_dir()/src/linnstrument-firmware and returns that folder."""
        if self._staged:
            return self._staged
        self._staged = self._stage()
        return self._staged

    def _stage(self):
        dest = os.path.join(self.work_dir(), 'src', SKETCH_NAME)
        if self.commit and os.path.exists(os.path.join(dest, '.staged')):
            return dest                                     # a commit never changes
        if os.path.exists(dest):
            shutil.rmtree(dest)
        os.makedirs(dest)
        if self.commit:
            archive = subprocess.run(['git', '-C', self.repo, 'archive', self.commit], capture_output=True, check=True)
            subprocess.run(['tar', '-x', '-C', dest], input=archive.stdout, check=True)
        else:
            files = git(self.repo, 'ls-files', '-z', '--cached', '--others', '--exclude-standard')
            for rel in sorted(set(f for f in files.split('\0') if f)):
                src = os.path.join(self.repo, rel)
                if not os.path.isfile(src):
                    continue                                # deleted but not yet committed
                out = os.path.join(dest, rel)
                os.makedirs(os.path.dirname(out), exist_ok=True)
                shutil.copy2(src, out)
        if not os.path.exists(os.path.join(dest, SKETCH_NAME + '.ino')):
            raise RuntimeError('%s has no %s.ino' % (self.describe(), SKETCH_NAME))
        open(os.path.join(dest, '.staged'), 'w').close()
        return dest
