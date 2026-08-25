# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""bundle_newton_runtime — vendor the Newton physics runtime into an OmniSim tree.

Goal (default-flip-plan.md L6 / engine-migration-plan.md §8.1): make a *stock*
install able to run Newton physics by shipping the runtime (CPython + warp +
newton + the MuJoCo solver + USD deps) *inside* the install tree. Newton is the
only physics backend; an installer without this bundle has no working physics.

Why this can be done in the packaging layer alone (no C++ change)
----------------------------------------------------------------
`OmNewtonBackend` brings the interpreter up with a bare `Py_InitializeEx(0)` --
no `Py_SetPythonHome`, no `PyConfig`, no `sys.path` edits (verified in
src/omnisim/physics/OmNewtonBackend.cpp) -- then does `import warp` / `import
newton`. So whether Newton is available on a box is *purely* a function of which
`python3XX.dll` the process loads at start and what is on that interpreter's
sys.path. On the dev box that resolves to the developer's user CPython whose
user-site happens to have warp; on a clean box it resolves to a Python without
warp (or none), so Newton cannot initialise and the simulation cannot run.

We fix that with the standard CPython **embeddable-distribution** mechanism:
stage a `python3XX` runtime beside `omnisim-bin.exe` and drop a `python3XX._pth`
next to the loaded DLL. When CPython's path init finds `python3XX._pth`, it
switches to *isolated path config*: sys.path is built strictly from the lines of
that file -- the Windows registry, `PYTHONHOME`, and the per-user site are all
ignored. The embedded interpreter then deterministically resolves the bundled
`site-packages` and nothing else. This is exactly how every "Windows embeddable
package" works; it just has to live next to the binary (or next to the DLL),
which the existing recursive `msys64/` copy in windows_distro.py already ships.

Crucially the python version must match what the binary actually links. The
current bundled omnisim-bin.exe imports `python312.dll`; a binary rebuilt under
the current Makefile (PYTHON_HOME -> Python314) would import `python314.dll`.
So this script autodetects the version from the binary's PE import table rather
than hardcoding it -- a mismatch is the #1 way bundling silently breaks.

Modes
-----
  vendor    (default) -- pip-install warp/newton/deps INTO the bundle now, so the
                         shipped installer is fully offline-capable (~600 MB:
                         warp 314 MB + usd 48 MB + newton 33 MB + mujoco_warp
                         9.5 MB + the CPython runtime). One `make` artifact runs
                         Newton on a clean, offline box.
  bootstrap            -- stage only the python runtime + a first-run installer;
                         warp/newton are pip-installed into the bundle on first
                         launch (slim installer, needs network once). The honest
                         middle ground if a ~600 MB installer is undesirable.

Verification
------------
`--verify` launches the *staged* interpreter with a scrubbed environment
(PYTHONPATH / PYTHONHOME unset, PATH reduced so a stray system python cannot leak
in) and asserts `import warp, newton` succeed -- i.e. it proves the clean-box
story on this box. `--verify-binary` additionally runs omnisim-bin.exe on the
Newton smoke world and checks for the "[OmNewtonBackend]" runtime-up log line
(the same signal the pre-push gate's --require-newton uses).

Usage
-----
    # vendor the runtime into the in-repo tree, then prove it imports clean
    python scripts/packaging/bundle_newton_runtime.py --verify

    # explicit target + slim first-run-install variant
    python scripts/packaging/bundle_newton_runtime.py \
        --target msys64/mingw64/bin --mode bootstrap

    # just report what the binary needs and what is currently bundled
    python scripts/packaging/bundle_newton_runtime.py --inspect

Run from the repo root. Windows is the only platform with a wgpu/Newton binary
today; the script no-ops with a clear message elsewhere.
"""

import argparse
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# Packages that make up the Newton runtime, pinned to EXACT versions from the
# single source of truth in newton_runtime_pins.py. Version-pinning (not
# name-only `--upgrade`) is what keeps the deploy bundle bit-exact with the
# trainer — see that module's docstring and tests/test_newton_pins_parity.py.
#   warp-lang          -> the GPU kernel runtime (bundles its own slim CUDA subset)
#   newton             -> the rigid-body solver built on warp
#   mujoco-warp        -> the SolverMuJoCo path (frictional contact / grasping needs it)
#   usd-core           -> newton's asset/USD dependency (imported as pxr)
#   newton-usd-schemas -> newton's codeless USD schema plugin; add_usd hard-fails
#                         without it (require_newton_usd_schemas raises)
try:
    from newton_runtime_pins import bundle_requirements
    NEWTON_PACKAGES = bundle_requirements()
except ImportError:  # running from an odd CWD: fall back to name-only, but warn.
    NEWTON_PACKAGES = ["warp-lang", "newton", "mujoco-warp", "usd-core",
                       "newton-usd-schemas"]
    print("WARNING: newton_runtime_pins.py not importable; vendoring UNPINNED "
          "(latest-on-PyPI) Newton packages, which can desync the deploy bundle "
          "from the trainer. Run from scripts/packaging/ or fix sys.path.",
          file=sys.stderr)

# Modules that must import for the runtime to be considered live.  The OmniSim
# helper is deliberately a real bundled module (not a C++ string literal), so
# bundle verification must cover it too or packaging can silently drift.
VERIFY_IMPORTS = ["warp", "newton", "omnisim_newton_runtime"]

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TARGET = REPO_ROOT / "msys64" / "mingw64" / "bin"
DEFAULT_BINARY = DEFAULT_TARGET / "omnisim-bin.exe"
SITE_DIRNAME = "newton-runtime"  # bundle subdir: <target>/newton-runtime/...
OMNISIM_RUNTIME_SOURCE = (REPO_ROOT / "src" / "omnisim" / "physics"
                          / "omnisim_newton_runtime.py")


# --------------------------------------------------------------------------- #
# PE import-table parsing -- detect the python3XX.dll the binary actually needs #
# --------------------------------------------------------------------------- #
def _rva_to_offset(rva, sections):
    for va, vsize, raw_ptr, raw_size in sections:
        if va <= rva < va + max(vsize, raw_size):
            return raw_ptr + (rva - va)
    return None


def detect_python_dll(binary_path):
    """Return the python3XX.dll name imported by a PE binary, or None.

    Pure-Python PE parse (no objdump dependency -- objdump is absent from the
    runtime-only bundled msys64). Reads the import directory and returns the
    first DLL name matching python3<digits>.dll.
    """
    data = Path(binary_path).read_bytes()
    if data[:2] != b"MZ":
        raise ValueError(f"{binary_path} is not a PE image (no MZ header)")
    e_lfanew = struct.unpack_from("<I", data, 0x3C)[0]
    if data[e_lfanew:e_lfanew + 4] != b"PE\0\0":
        raise ValueError(f"{binary_path} has no PE signature")
    coff = e_lfanew + 4
    num_sections = struct.unpack_from("<H", data, coff + 2)[0]
    opt_size = struct.unpack_from("<H", data, coff + 16)[0]
    opt = coff + 20
    magic = struct.unpack_from("<H", data, opt)[0]
    # data directories start after the fixed optional-header body
    # (PE32 = 96 bytes, PE32+ = 112 bytes); import table is directory index 1.
    dir_off = opt + (112 if magic == 0x20B else 96)
    import_rva = struct.unpack_from("<I", data, dir_off + 8)[0]
    if import_rva == 0:
        return None
    sec = opt + opt_size
    sections = []
    for i in range(num_sections):
        base = sec + i * 40
        vsize = struct.unpack_from("<I", data, base + 8)[0]
        va = struct.unpack_from("<I", data, base + 12)[0]
        raw_size = struct.unpack_from("<I", data, base + 16)[0]
        raw_ptr = struct.unpack_from("<I", data, base + 20)[0]
        sections.append((va, vsize, raw_ptr, raw_size))
    off = _rva_to_offset(import_rva, sections)
    if off is None:
        return None
    found = []
    # IMAGE_IMPORT_DESCRIPTOR is 20 bytes; Name RVA is at +12; table ends at a
    # zeroed descriptor.
    while True:
        descriptor = data[off:off + 20]
        if len(descriptor) < 20 or descriptor == b"\0" * 20:
            break
        name_rva = struct.unpack_from("<I", descriptor, 12)[0]
        if name_rva:
            name_off = _rva_to_offset(name_rva, sections)
            if name_off is not None:
                end = data.index(b"\0", name_off)
                name = data[name_off:end].decode("ascii", "replace")
                found.append(name)
        off += 20
    for name in found:
        if re.fullmatch(r"python3\d+\.dll", name, re.IGNORECASE):
            return name
    return None


def python_tag_from_dll(dll_name):
    """'python312.dll' -> ('3', '12', '312')."""
    m = re.fullmatch(r"python3(\d+)\.dll", dll_name, re.IGNORECASE)
    if not m:
        raise ValueError(f"cannot parse python version from {dll_name}")
    minor = m.group(1)
    return "3", minor, f"3{minor}"


# --------------------------------------------------------------------------- #
# Locating the source CPython runtime to stage                                 #
# --------------------------------------------------------------------------- #
def find_cpython_home(tag, override):
    """Locate a full CPython install matching tag (e.g. '312') to copy from.

    Mirrors the Makefile's Windows PYTHON_HOME convention; override wins.
    """
    if override:
        home = Path(override)
        if not (home / f"python.exe").exists():
            raise FileNotFoundError(f"--cpython-home {home} has no python.exe")
        return home
    candidates = []
    for env in ("LOCALAPPDATA",):
        base = os.environ.get(env)
        if base:
            candidates.append(Path(base) / "Programs" / "Python" / f"Python{tag}")
    user = os.environ.get("USERNAME", "")
    candidates.append(
        Path(f"C:/Users/{user}/AppData/Local/Programs/Python/Python{tag}")
    )
    for c in candidates:
        if (c / "python.exe").exists():
            return c
    raise FileNotFoundError(
        f"no CPython {tag} install found ({[str(c) for c in candidates]}); "
        f"pass --cpython-home"
    )


# --------------------------------------------------------------------------- #
# Staging                                                                      #
# --------------------------------------------------------------------------- #
def stage_runtime(cpython_home, bundle_dir, tag, force):
    """Copy the CPython runtime (DLL + stdlib) into <bundle_dir>.

    Layout produced:
        <bundle_dir>/DLLs, Lib, python3.dll, python3XX.dll, vcruntime*.dll
        <bundle_dir>/site-packages   (created empty; pip --target fills it)
    The python3XX.dll ALSO has to sit next to omnisim-bin.exe (the loader finds
    it there); that copy is done by place_loader_dll().
    """
    bundle_dir = Path(bundle_dir)
    if bundle_dir.exists() and any(bundle_dir.iterdir()) and not force:
        raise FileExistsError(
            f"{bundle_dir} already populated; pass --force to overwrite"
        )
    bundle_dir.mkdir(parents=True, exist_ok=True)

    dll = f"python{tag}.dll"
    runtime_files = [dll, "python3.dll"]
    for f in cpython_home.glob("vcruntime*.dll"):
        runtime_files.append(f.name)
    for name in runtime_files:
        src = cpython_home / name
        if src.exists():
            shutil.copy2(src, bundle_dir / name)
        elif name == dll:
            raise FileNotFoundError(f"{src} missing -- wrong CPython for {tag}?")

    for sub in ("DLLs", "Lib"):
        src = cpython_home / sub
        if src.exists():
            dst = bundle_dir / sub
            if dst.exists():
                shutil.rmtree(dst)
            # __pycache__ and test trees bloat the bundle; drop them. The
            # source interpreter's own Lib/site-packages must NOT come
            # along either — on a normal (non-embeddable) install it holds
            # every package the user ever pip-installed (torch alone is
            # ~4 GB); the bundle gets its own fresh site-packages filled
            # by pip --target below.
            shutil.copytree(
                src, dst,
                ignore=shutil.ignore_patterns(
                    "__pycache__", "test", "tests", "site-packages"
                ),
            )
    (bundle_dir / "site-packages").mkdir(exist_ok=True)
    return bundle_dir


def write_pth(target_dir, bundle_dir, tag):
    """Write python3XX._pth next to the binary so CPython uses isolated path
    config rooted at the bundle. Paths are relative to the DLL's directory."""
    target_dir = Path(target_dir)
    rel = os.path.relpath(bundle_dir, target_dir).replace("\\", "/")
    pth = target_dir / f"python{tag}._pth"
    lines = [
        f"{rel}/python{tag}.zip",  # honored if an embeddable zip stdlib is used
        f"{rel}/Lib",
        f"{rel}/DLLs",
        f"{rel}/site-packages",
        ".",
        "import site",  # process .pth files inside site-packages
    ]
    pth.write_text("\n".join(lines) + "\n", encoding="ascii")
    return pth


def place_loader_dll(cpython_home, target_dir, tag):
    """Ensure python3XX.dll sits next to omnisim-bin.exe so the loader binds the
    bundled interpreter rather than a system one."""
    src = cpython_home / f"python{tag}.dll"
    dst = Path(target_dir) / f"python{tag}.dll"
    if src.exists():
        shutil.copy2(src, dst)
    return dst


def pip_install(python_exe, site_packages, packages):
    cmd = [str(python_exe), "-m", "pip", "install", "--upgrade",
           "--target", str(site_packages), *packages]
    print("  " + " ".join(cmd), flush=True)
    subprocess.run(cmd).check_returncode()


def stage_omnisim_runtime_module(site_packages):
    """Stage the engine-owned Python physics glue beside Newton's packages."""
    if not OMNISIM_RUNTIME_SOURCE.is_file():
        raise FileNotFoundError(
            f"missing engine runtime module: {OMNISIM_RUNTIME_SOURCE}"
        )
    target = Path(site_packages) / OMNISIM_RUNTIME_SOURCE.name
    shutil.copy2(OMNISIM_RUNTIME_SOURCE, target)
    return target


# --------------------------------------------------------------------------- #
# Verification                                                                 #
# --------------------------------------------------------------------------- #
def scrubbed_env():
    """A minimal environment with no python-path leakage, so a passing import
    proves the bundle is self-contained (the clean-box condition)."""
    env = {}
    for k in ("SystemRoot", "WINDIR", "TEMP", "TMP", "NUMBER_OF_PROCESSORS",
              "PROCESSOR_ARCHITECTURE", "COMSPEC", "PATHEXT"):
        if k in os.environ:
            env[k] = os.environ[k]
    sysroot = os.environ.get("SystemRoot", r"C:\Windows")
    env["PATH"] = f"{sysroot}\\System32;{sysroot}"
    env["PYTHONNOUSERSITE"] = "1"
    return env


def verify_interpreter(bundle_dir, tag):
    """Run the staged python with the _pth-equivalent path and assert imports.

    We invoke the bundled python with an explicit sys.path matching the _pth so
    this works even before a binary that loads the DLL exists.
    """
    bundle_dir = Path(bundle_dir)
    py = bundle_dir / "python.exe"
    if not py.exists():
        # the staged tree may carry only the DLL; fall back to copying the
        # source python.exe alongside for the check
        raise FileNotFoundError(
            f"{py} not present; pass --cpython-home so python.exe is staged"
        )
    paths = [bundle_dir / "Lib", bundle_dir / "DLLs",
             bundle_dir / "site-packages"]
    code = (
        "import sys; "
        + "; ".join(f"sys.path.insert(0, r'{p}')" for p in paths)
        + "; import importlib; "
        + "mods=" + repr(VERIFY_IMPORTS) + "; "
        + "[importlib.import_module(m) for m in mods]; "
        + "import warp; print('warp', warp.config.version if hasattr(warp,'config') else getattr(warp,'__version__','?')); "
        + "import newton; print('newton ok')"
    )
    r = subprocess.run([str(py), "-S", "-c", code], env=scrubbed_env(),
                       capture_output=True, text=True)
    print(r.stdout, end="")
    if r.returncode != 0:
        print(r.stderr, file=sys.stderr)
    return r.returncode == 0


def verify_binary(binary_path, target_dir, timeout=60):
    """Prove the packaged binary finalises Newton and advances one step.

    Import verification catches a malformed Python bundle, but it cannot prove
    that the executable loads that bundle or that the C++/Python boundary is
    sound. This check launches the exact binary on a controller-free smoke
    world, then requires both independent runtime signals produced by the
    engine: a finalised Newton verdict sidecar and a first-step log line.
    """
    binary_path = Path(binary_path).resolve()
    target_dir = Path(target_dir).resolve()
    repo_root = Path(__file__).resolve().parents[2]
    world = (repo_root / "projects" / "samples" / "demos" / "worlds" /
             "physics" / "newton_static_collider_smoke.omniworld")
    if not world.exists():
        print(f"BINARY VERIFY: FAIL (smoke world missing: {world})",
              file=sys.stderr)
        return False

    with tempfile.TemporaryDirectory(prefix="omnisim-bundle-verify-") as td:
        run_dir = Path(td)
        log_path = run_dir / "omnisim_log.txt"
        sidecar_path = Path(str(log_path) + ".newton.json")
        stdout_path = run_dir / "stdout.txt"
        stderr_path = run_dir / "stderr.txt"

        # Preserve the normal Windows/Qt environment, but remove every Python
        # path override. The adjacent DLL + _pth file must select the bundle.
        env = os.environ.copy()
        for key in ("PYTHONHOME", "PYTHONPATH", "PYTHONSTARTUP"):
            env.pop(key, None)
        env["PYTHONNOUSERSITE"] = "1"
        env["OMNISIM_HOME"] = str(repo_root)
        env["OMNISIM_LOG_PATH"] = str(log_path)
        env["OMNISIM_REQUIRE_NEWTON"] = "1"
        env["PATH"] = str(target_dir) + os.pathsep + env.get("PATH", "")

        cmd = [
            str(binary_path), str(world), "--minimize", "--batch",
            "--no-rendering", "--mode=fast", "--stdout", "--stderr",
        ]
        print("verifying packaged binary with Newton smoke world...")
        print("  binary:", binary_path)
        print("  world :", world)

        with stdout_path.open("wb") as stdout_sink, stderr_path.open("wb") as stderr_sink:
            try:
                proc = subprocess.Popen(cmd, env=env, stdout=stdout_sink,
                                        stderr=stderr_sink)
            except OSError as exc:
                print(f"BINARY VERIFY: FAIL (could not launch: {exc})",
                      file=sys.stderr)
                return False

            deadline = time.monotonic() + timeout
            finalised = False
            stepped = False
            sidecar = {}
            while time.monotonic() < deadline:
                if log_path.exists():
                    try:
                        log_text = log_path.read_text(encoding="utf-8",
                                                      errors="replace")
                    except OSError:
                        log_text = ""
                    finalised = ("[OmNewtonBackend] world finalised" in log_text or
                                 "[WbNewtonBackend] world finalised" in log_text)
                    stepped = bool(re.search(
                        r"\[(?:Om|Wb)NewtonBackend\] step\s+\d+", log_text))
                if sidecar_path.exists():
                    try:
                        import json
                        sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
                    except (OSError, ValueError):
                        sidecar = {}
                if finalised and stepped and sidecar.get("finalised") is True:
                    break
                if proc.poll() is not None:
                    break
                time.sleep(0.2)

            was_running = proc.poll() is None
            if was_running:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5)

        ok = finalised and stepped and sidecar.get("finalised") is True
        if ok:
            print("BINARY VERIFY: PASS "
                  f"(solver={sidecar.get('solver', 'recorded by engine')})")
            return True

        stderr_tail = ""
        try:
            stderr_tail = stderr_path.read_text(encoding="utf-8",
                                                 errors="replace")[-2000:]
        except OSError:
            pass
        print("BINARY VERIFY: FAIL "
              f"(finalised_log={finalised}, first_step={stepped}, "
              f"sidecar_finalised={sidecar.get('finalised')!r}, "
              f"process_exit={proc.returncode})", file=sys.stderr)
        if stderr_tail.strip():
            print("engine stderr tail:\n" + stderr_tail.strip(), file=sys.stderr)
        return False


# --------------------------------------------------------------------------- #
# Inspect / report                                                             #
# --------------------------------------------------------------------------- #
def inspect(binary_path, target_dir):
    print(f"binary : {binary_path}")
    if not Path(binary_path).exists():
        print("  (missing -- build omnisim-bin.exe first)")
        return 1
    dll = detect_python_dll(binary_path)
    print(f"  imports: {dll or '(no python3XX.dll import -- no Newton runtime link)'}")
    bundle = Path(target_dir) / SITE_DIRNAME
    print(f"bundle : {bundle}")
    if bundle.exists():
        sp = bundle / "site-packages"
        names = sorted(p.name for p in sp.glob("*")) if sp.exists() else []
        have = [m for m in VERIFY_IMPORTS
                if any(n == m or n.startswith(m + "-") for n in names)]
        print(f"  staged python : {(bundle / (dll or '')).exists() if dll else '?'}")
        print(f"  site-packages : {len(names)} entries; runtime modules present: {have}")
    else:
        print("  (not staged)")
    pth = Path(target_dir) / (f"python{python_tag_from_dll(dll)[2]}._pth" if dll else "")
    print(f"_pth   : {pth} {'(present)' if dll and pth.exists() else '(absent)'}")
    return 0


# --------------------------------------------------------------------------- #
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default=str(DEFAULT_TARGET),
                    help="dir holding omnisim-bin.exe (default: %(default)s)")
    ap.add_argument("--binary", default=None,
                    help="omnisim-bin.exe to read the python version from")
    ap.add_argument("--cpython-home", default=None,
                    help="full CPython install to stage from (default: autodetect)")
    ap.add_argument("--mode", choices=["vendor", "bootstrap"], default="vendor")
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing bundle")
    ap.add_argument("--verify", action="store_true",
                    help="after staging, prove imports under a scrubbed env")
    ap.add_argument("--verify-binary", action="store_true",
                    help="also prove the packaged binary finalises and steps Newton")
    ap.add_argument("--inspect", action="store_true",
                    help="report current state and exit (no changes)")
    args = ap.parse_args(argv)

    if os.name != "nt":
        print("Newton runtime bundling is Windows-only today (the only platform "
              "with a Newton/wgpu binary). No-op.")
        return 0

    binary = Path(args.binary) if args.binary else Path(args.target) / "omnisim-bin.exe"

    if args.inspect:
        return inspect(binary, args.target)

    if not binary.exists():
        print(f"ERROR: {binary} not found; build omnisim-bin.exe first.",
              file=sys.stderr)
        return 1
    dll = detect_python_dll(binary)
    if not dll:
        print(f"ERROR: {binary} imports no python3XX.dll -- this build has no "
              f"Newton runtime link and therefore no working physics. Nothing "
              f"to bundle.",
              file=sys.stderr)
        return 1
    _, _, tag = python_tag_from_dll(dll)
    print(f"binary needs {dll} -> staging CPython {tag} runtime")

    cpython_home = find_cpython_home(tag, args.cpython_home)
    print(f"source CPython: {cpython_home}")
    bundle = Path(args.target) / SITE_DIRNAME

    stage_runtime(cpython_home, bundle, tag, args.force)
    # stage python.exe too so --verify can drive the bundled interpreter
    for exe in ("python.exe", "pythonw.exe"):
        src = cpython_home / exe
        if src.exists():
            shutil.copy2(src, bundle / exe)
    place_loader_dll(cpython_home, args.target, tag)
    write_pth(args.target, bundle, tag)

    if args.mode == "vendor":
        print("vendoring Newton packages (~600 MB)...")
        # Drive pip from the SOURCE interpreter: the bundled copy ships
        # without pip (its Lib is stdlib-only by design), and the source
        # is the exact same CPython version so wheel tags resolve
        # identically for the --target install.
        pip_install(cpython_home / "python.exe", bundle / "site-packages",
                    NEWTON_PACKAGES)
        # pip --target does NOT remove the previous version's .dist-info on
        # upgrade, so re-staging accumulates one dist-info per historical
        # install and importlib.metadata.version() becomes ambiguous -- every
        # provenance consumer (env_fingerprint, skill manifests) then records
        # whichever version wins the directory scan. Found live 2026-07-16:
        # the shipped bundle carried stale dist-infos for newton, mujoco,
        # mujoco_warp AND warp_lang. RECORD-audit and delete the stale ones.
        import audit_dist_info
        anomalies, _ = audit_dist_info.audit(bundle / "site-packages",
                                             repair=True)
        if anomalies:
            print("ERROR: ambiguous dist-info metadata survived repair: "
                  + ", ".join(a[0] for a in anomalies), file=sys.stderr)
            return 3
    else:
        marker = bundle / "FIRST_RUN_INSTALL.txt"
        marker.write_text(
            "Newton runtime not yet installed. On first Newton use OmniSim will\n"
            "pip-install " + " ".join(NEWTON_PACKAGES) + " into site-packages.\n",
            encoding="ascii")
        print(f"bootstrap mode: wrote {marker} (warp/newton install on first run)")

    staged_helper = stage_omnisim_runtime_module(bundle / "site-packages")
    print("staged OmniSim Newton helper:", staged_helper)

    print("staged bundle at:", bundle)

    verified = True
    if args.verify:
        print("verifying imports under a scrubbed environment...")
        ok = verify_interpreter(bundle, tag)
        # metadata honesty is part of the verify contract: duplicate
        # dist-infos make the bundle's reported versions untrustworthy
        import audit_dist_info
        anomalies, _ = audit_dist_info.audit(bundle / "site-packages")
        if anomalies:
            print("VERIFY: FAIL (duplicate dist-info metadata: "
                  + ", ".join(a[0] for a in anomalies) + ")")
            return 2
        print("VERIFY:", "PASS" if ok else "FAIL")
        verified = verified and ok
    if args.verify_binary:
        verified = verify_binary(binary, args.target) and verified
    return 0 if verified else 2


if __name__ == "__main__":
    sys.exit(main())
