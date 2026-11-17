"""Give an arm its OWN engine runtime, so a concurrent rebuild cannot move it.

The cc_lane workspace reaches the engine at ``<REPO>/msys64/mingw64/bin/
omnisim-bin.exe`` through a junction chain. If both arms junction the live
tree, the engine agent's rebuild lands in the middle of a cell -- which the
sha pin would (correctly) abandon, at the cost of the cell.

So each arm gets:

  <arm>/msys64/                      real dir
        mingw64/                     real dir
                bin/                 real dir
                    <subdirs>        junctions into the live tree (vendored:
                                     newton-runtime, platforms, tls, ...)
                    <files>          HARDLINKS into the live tree (toolchain
                                     DLLs -- same volume, free, and nothing
                                     rebuilds them)
                    omnisim-bin.exe  a real COPY of this arm's frozen binary
                <other subdirs>      junctions
        <other subdirs>              junctions
  <arm>/lib/                         a real COPY (6.9 MB; libController is
                                     rebuilt by the engine agent and the
                                     engine<->libController ABI split hangs
                                     every controller at zero ticks)

Identical construction on both arms; only the frozen binary differs.
"""
import os
import shutil
import subprocess
import sys

LIVE = r"O:\omnisim"


def rm(path):
    if os.path.islink(path) or (os.path.isdir(path) and
                                os.path.exists(path) and
                                _is_reparse(path)):
        subprocess.run(["cmd", "/c", "rmdir", path], capture_output=True)
        return
    if os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    elif os.path.exists(path):
        os.chmod(path, 0o666)
        os.remove(path)


def _is_reparse(p):
    try:
        return bool(os.stat(p, follow_symlinks=False).st_file_attributes
                    & 0x400)
    except Exception:
        return False


def junction(link, target):
    r = subprocess.run(["cmd", "/c", "mklink", "/J", link, target],
                       capture_output=True, text=True)
    if r.returncode:
        raise SystemExit("mklink /J %s -> %s failed: %s"
                         % (link, target, r.stdout + r.stderr))


def hardlink(link, target):
    r = subprocess.run(["cmd", "/c", "mklink", "/H", link, target],
                       capture_output=True, text=True)
    if r.returncode:
        shutil.copy2(target, link)
        return "copy"
    return "hardlink"


def mirror_dir(dst, src, real_subdir=None):
    """Real dir at dst; children junctioned/hardlinked from src.

    ``real_subdir`` (one name) is created as a real dir instead of a junction
    so the recursion can continue into it.
    """
    os.makedirs(dst, exist_ok=True)
    for name in os.listdir(src):
        s = os.path.join(src, name)
        d = os.path.join(dst, name)
        if name == real_subdir:
            continue
        if os.path.isdir(s):
            junction(d, s)
        else:
            hardlink(d, s)


def freeze(arm_root, frozen_bin):
    live_msys = os.path.join(LIVE, "msys64")
    a_msys = os.path.join(arm_root, "msys64")
    rm(a_msys)
    mirror_dir(a_msys, live_msys, real_subdir="mingw64")

    a_mingw = os.path.join(a_msys, "mingw64")
    mirror_dir(a_mingw, os.path.join(live_msys, "mingw64"), real_subdir="bin")

    a_bin = os.path.join(a_mingw, "bin")
    live_bin = os.path.join(live_msys, "mingw64", "bin")
    n_link = n_copy = n_junc = 0
    os.makedirs(a_bin, exist_ok=True)
    for name in os.listdir(live_bin):
        s = os.path.join(live_bin, name)
        d = os.path.join(a_bin, name)
        if os.path.isdir(s):
            junction(d, s)
            n_junc += 1
        elif name.lower() == "omnisim-bin.exe":
            shutil.copy2(frozen_bin, d)
            n_copy += 1
        else:
            if hardlink(d, s) == "copy":
                n_copy += 1
            else:
                n_link += 1
    print("  msys64: %d hardlinks, %d copies, %d junctions" %
          (n_link, n_copy, n_junc))

    a_lib = os.path.join(arm_root, "lib")
    rm(a_lib)
    shutil.copytree(os.path.join(LIVE, "lib"), a_lib)
    print("  lib: real copy")


if __name__ == "__main__":
    arm_root, frozen_bin = sys.argv[1], sys.argv[2]
    print("freezing %s with %s" % (arm_root, frozen_bin))
    freeze(arm_root, frozen_bin)
    import hashlib
    p = os.path.join(arm_root, "msys64", "mingw64", "bin", "omnisim-bin.exe")
    print("  engine sha256: %s"
          % hashlib.sha256(open(p, "rb").read()).hexdigest())
    p = os.path.join(arm_root, "lib", "controller", "Controller.dll")
    if os.path.exists(p):
        print("  libController sha256: %s"
              % hashlib.sha256(open(p, "rb").read()).hexdigest())
