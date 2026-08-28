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

"""omnisim doctor — print ground-truth repo / runtime state for agents.

A new agent session (Claude Code, Codex, Cursor, Aider, …) that lands in
an OmniSim clone runs this on its first turn to know what is actually
true about this clone right now, instead of guessing from documentation.

AGENTS.md §3 (cold-start bootstrap) directs agents here.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
from pathlib import Path

from . import __version__
from .paths import DEMOS_WORLDS, REPO_ROOT, resolve_omnisim_binary


def invocation() -> str:
    """How to spell this CLI back to the user who just ran it.

    A Windows user with no system Python reaches the CLI through omnisim.bat,
    which uses the interpreter bundled beside the engine. Answering them with
    `python -m omnisim demo` hands them the one command they cannot run, so
    the launcher exports its own name and every "next command" echoes it.
    """
    return os.environ.get("OMNISIM_INVOKED_AS") or "python -m omnisim"


HARNESS_PORT = 6789
SUPERVISOR_PORT = 6790
CAPTURE_PORT = 6791

# The engine and libController must agree on the intern IPC pipe name. The engine
# exports a per-launch nonce (OmController::setProcessEnvironment) and names the pipe
# webots-<tmpId>-<nonce>-<robot>; libController falls back to the legacy
# webots-<tmpId>-<robot> when it cannot read the nonce (compute_socket_filename,
# src/controller/c/robot.c). An engine that HAS the nonce paired with a libController
# built BEFORE it therefore listen on two different pipes and never meet: every
# controller blocks forever in Robot(), the world finalises but never steps, and the
# run still exits 0. Probing for the symbol is exact, so this is a gate, not a guess.
_IPC_NONCE_TOKEN = b"OMNISIM_IPC_NONCE"

# lib name per platform; the engine links the same DLL/so that controllers load.
_CONTROLLER_LIB_NAMES = ("Controller.dll", "libController.so", "libController.dylib")


def _git(*args: str) -> str | None:
    try:
        out = subprocess.check_output(
            ["git", *args], cwd=REPO_ROOT, stderr=subprocess.DEVNULL
        )
        return out.decode("utf-8", errors="replace").strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def _port_status(port: int, host: str = "127.0.0.1") -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.2)
    try:
        return "in-use" if sock.connect_ex((host, port)) == 0 else "free"
    finally:
        sock.close()


def _worlds() -> list[str]:
    if not DEMOS_WORLDS.is_dir():
        return []
    # Recursive: most demos live in subdirs (showcase/, flagship/, chat/,
    # ...). A top-level-only glob reported 2 of 73 worlds and made fresh
    # clones look empty.
    # Dot-prefixed names are the harness's own scratch copies
    # (.harness_*.omniworld, gitignored). They sort first, so an unfiltered
    # listing showed a newcomer five temp files as the sample of "your worlds"
    # and inflated the count by 26 on a box where the harness had ever run.
    def _authored(path: Path) -> bool:
        rel = path.relative_to(DEMOS_WORLDS)
        return not any(part.startswith(".") for part in rel.parts)

    return sorted(
        p.relative_to(DEMOS_WORLDS).as_posix()
        for p in [*DEMOS_WORLDS.rglob("*.omniworld"), *DEMOS_WORLDS.rglob("*.wbt")]
        if _authored(p)
    )


def _physics_runtime(binary: str | None) -> dict:
    """Is there a physics backend at all?

    Newton has been the ONLY backend since `bdc02139` deleted ODE, so "absent"
    is not a degraded mode -- it is an install with no dynamics whatsoever:
    nothing falls, nothing collides, no grasp holds, and the engine still
    exits 0. README tells the user `doctor` reports this, so it must.

    It cannot be one existence test, because the runtime is resolved
    differently per platform:

    * Windows -- the release BUNDLES it next to the binary, and the embedded
      interpreter loads it through `python312._pth`. Presence is a file test.
    * Linux   -- there is no bundle. The embedded interpreter resolves the
      SYSTEM `python3` (and ignores venvs), so the wheels must be importable
      from that interpreter. Probed with `importlib.util.find_spec`, which
      resolves without executing the module (importing `warp` initialises
      CUDA and costs seconds).
    """
    info: dict = {"status": "unknown", "source": None, "detail": "", "fix": None}
    if not binary:
        info["detail"] = "no engine binary, so no runtime to check"
        return info

    if os.name == "nt":
        info["source"] = "bundle"
        root = Path(binary).parent / "newton-runtime" / "site-packages"
        missing = [m for m in ("warp", "newton", "mujoco") if not (root / m).exists()]
        if not missing:
            info["status"] = "present"
            info["detail"] = "Newton runtime bundled next to the engine"
        else:
            info["status"] = "absent"
            info["detail"] = (
                "Newton runtime NOT bundled (missing " + ", ".join(missing) + ") -- "
                "this install has NO physics at all"
            )
            info["fix"] = "make -C src/omnisim bundle-newton-runtime"
        return info

    info["source"] = "system-python3"
    probe = (
        "import importlib.util as u;"
        "print(','.join(m for m in ('warp','newton','mujoco') if u.find_spec(m) is None))"
    )
    try:
        out = subprocess.run(
            ["python3", "-c", probe],
            capture_output=True, text=True, timeout=30,
        )
    except (OSError, subprocess.SubprocessError):
        info["detail"] = "could not run `python3` to check the physics runtime"
        return info
    if out.returncode != 0:
        info["detail"] = "`python3 -c` failed, so the physics runtime is unverified"
        return info
    missing = [m for m in out.stdout.strip().split(",") if m]
    if not missing:
        info["status"] = "present"
        info["detail"] = "physics wheels importable from the system python3"
    else:
        info["status"] = "absent"
        info["detail"] = (
            "system python3 cannot import " + ", ".join(missing) + " -- "
            "this install has NO physics at all"
        )
        info["fix"] = (
            "pip install warp-lang newton mujoco mujoco-warp "
            "(into the SYSTEM python3 -- the engine ignores venvs)"
        )
    return info


def _python_runtime() -> dict:
    """The interpreter versions that decide whether physics and controllers work.

    Two different interpreters matter and they are not always the same one:
    this CLI's, and the `python`/`python3` on PATH -- which the engine spawns
    for every Python controller, and which on Linux also supplies the physics
    wheels. newton 1.5.0 raises `TypeError: Union[arg, ...]` at
    `ModelBuilder()` on CPython 3.10, so a 3.10 interpreter in the physics
    role is a world that loads and never moves.
    """
    info: dict = {
        "cli": "%d.%d.%d" % sys.version_info[:3],
        "spawned": None,
        "spawned_name": None,
        "too_old": False,
    }
    for name in ("python3", "python") if os.name != "nt" else ("python", "python3"):
        exe = shutil.which(name)
        if not exe:
            continue
        try:
            out = subprocess.run(
                [exe, "-c", "import sys;print('%d.%d.%d' % sys.version_info[:3])"],
                capture_output=True, text=True, timeout=15,
            )
        except (OSError, subprocess.SubprocessError):
            continue
        if out.returncode == 0 and out.stdout.strip():
            info["spawned"] = out.stdout.strip()
            info["spawned_name"] = name
            break
    for ver in (info["cli"], info["spawned"]):
        if not ver:
            continue
        parts = ver.split(".")
        if (int(parts[0]), int(parts[1])) < (3, 11):
            info["too_old"] = True
    return info


def _controller_lib() -> Path | None:
    lib_dir = REPO_ROOT / "lib" / "controller"
    for name in _CONTROLLER_LIB_NAMES:
        candidate = lib_dir / name
        if candidate.is_file():
            return candidate
    return None


def _has_token(path: Path, token: bytes, chunk: int = 1 << 20) -> bool | None:
    """Stream-scan a binary for `token`. None when the file can't be read."""
    try:
        overlap = len(token) - 1
        tail = b""
        with path.open("rb") as fh:
            while True:
                block = fh.read(chunk)
                if not block:
                    return False
                if token in tail + block:
                    return True
                tail = block[-overlap:] if overlap else b""
    except OSError:
        return None


def _build_provenance() -> dict:
    """Engine vs libController compatibility — the stale-lib hang, caught cheaply."""
    engine_path = resolve_omnisim_binary()
    engine = Path(engine_path) if engine_path else None
    lib = _controller_lib()

    info: dict = {
        "engine": str(engine) if engine else None,
        "controller_lib": str(lib) if lib else None,
        "verdict": "unknown",
        "detail": "",
    }
    if engine is None or lib is None:
        missing = "engine binary" if engine is None else "libController"
        info["detail"] = f"{missing} not found -- build the simulator"
        return info

    info["engine_mtime"] = int(engine.stat().st_mtime)
    info["controller_lib_mtime"] = int(lib.stat().st_mtime)

    engine_nonce = _has_token(engine, _IPC_NONCE_TOKEN)
    lib_nonce = _has_token(lib, _IPC_NONCE_TOKEN)
    info["engine_ipc_nonce"] = engine_nonce
    info["controller_lib_ipc_nonce"] = lib_nonce

    if engine_nonce is None or lib_nonce is None:
        info["detail"] = "could not read one of the binaries"
        return info

    if engine_nonce and not lib_nonce:
        info["verdict"] = "incompatible"
        info["detail"] = (
            "libController predates the engine's IPC nonce: controllers will hang "
            "forever in Robot() and the sim will never step (the run still exits 0). "
            "Rebuild: make -C src/controller/c release"
        )
        return info

    info["verdict"] = "ok"
    stale_s = info["engine_mtime"] - info["controller_lib_mtime"]
    if stale_s > 86400:
        info["detail"] = (
            f"libController is {stale_s // 86400}d older than the engine -- compatible "
            "on the IPC nonce, but rebuild if controllers misbehave"
        )
    return info


def _env_landmines() -> list[str]:
    """Env-var conditions that are latent traps but do NOT stop a run here.

    Deliberately conservative: only flag a condition that is unambiguously wrong
    (a var pointing at a path that does not exist), never one that is merely
    unusual (a var pointing somewhere else, an unset var). A gate that false-fires
    on a working install is worse than no gate, so these stay advisory.
    """
    out: list[str] = []
    webots_home = os.environ.get("WEBOTS_HOME")
    if webots_home and not Path(webots_home).is_dir():
        # The core runtime (libController, launcher, controller package) reads
        # OMNISIM_HOME and falls back to the repo root, so runs still work with a
        # stale WEBOTS_HOME. But qt_utils (robot windows) and ~20 Makefiles still
        # read WEBOTS_HOME and would resolve into this phantom path. Advisory,
        # because it does not stop a run through this CLI -- but a real landmine.
        fix = (
            "set OMNISIM_HOME to this checkout"
            if not os.environ.get("OMNISIM_HOME")
            else "clear WEBOTS_HOME"
        )
        out.append(
            f"WEBOTS_HOME='{webots_home}' is set but does not exist (stale). Runs use "
            f"OMNISIM_HOME/repo-root so they work, but qt_utils and the Makefiles read "
            f"WEBOTS_HOME and would resolve into a phantom install; {fix}."
        )
    return out


def _coherence(build: dict, physics: dict | None = None) -> dict:
    """Tier 0 -- is this install even wired coherently enough to run at all?

    The cross-machine determinism tiers (docs/developer/cross-machine-determinism.md)
    all ask whether a run BEHAVES the same: physics, numerics, solver bands. None
    can catch a stale DLL, because that failure produces NO tick to compare -- the
    world finalises and then every controller blocks forever in Robot(). This is the
    missing layer below them: coherence, not behaviour.

    Consumes ``_build_provenance()`` output (no re-probe) plus cheap env checks and
    splits problems into:

    - ``fatal``    -- the install cannot be trusted to run a controller-driven world;
                      ``--strict`` exits non-zero.
    - ``advisory`` -- real, but a run still proceeds here; printed, never gates.
    """
    fatal: list[str] = []
    advisory: list[str] = []

    verdict = build.get("verdict")
    if verdict == "incompatible":
        # Exact symbol probe, not a heuristic: the engine HAS the IPC nonce and the
        # lib does NOT, so they name different pipes and never meet. Definitive.
        fatal.append("engine/libController ABI mismatch (IPC nonce): " + build.get("detail", ""))
    elif build.get("engine") is None:
        # No binary -> literally nothing to run. 100% reliable (a file exists or not).
        fatal.append("engine binary not found -- nothing to run (build the simulator first)")
    elif build.get("controller_lib") is None:
        # No libController -> every controller process fails to load its API.
        fatal.append("libController not found -- controllers cannot load their API")
    elif build.get("engine_ipc_nonce") is None or build.get("controller_lib_ipc_nonce") is None:
        # A binary existed but could not be read (OSError). We cannot PROVE
        # incoherence, so we must not fail on it -- unproven != broken.
        advisory.append("could not read a binary to verify the IPC nonce -- coherence unproven")

    # Physics. Newton is the ONLY backend, so its absence is not a degraded
    # mode -- it is an install where nothing falls, while the engine still
    # exits 0. README promises doctor reports this, so it is FATAL, not a note.
    if physics and physics.get("status") == "absent":
        fatal.append(physics["detail"] + (" -- fix: " + physics["fix"] if physics.get("fix") else ""))

    advisory.extend(_env_landmines())

    return {"ok": not fatal, "fatal": fatal, "advisory": advisory}


def run(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        prog="omnisim doctor",
        description="Report ground-truth repo and runtime state.",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of text.")
    parser.add_argument(
        "--fingerprint",
        action="store_true",
        help="Also capture the install config fingerprint (resolved physics "
        "backend, Newton-runtime presence, GPU, fingerprint_id).",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Deprecated alias -- kept for .githooks/pre-push and existing CI. "
        "Plain `doctor` now exits non-zero on a blocking problem, so this flag "
        "no longer changes anything.",
    )
    args = parser.parse_args(argv)

    fingerprint = None
    if args.fingerprint:
        try:
            from .conformance import collect_fingerprint
            fingerprint = collect_fingerprint(scrub_paths=True)
        except Exception as exc:  # never let the fingerprint break doctor
            fingerprint = {"error": f"fingerprint failed: {exc}"}

    branch = _git("rev-parse", "--abbrev-ref", "HEAD") or "?"
    commit = _git("rev-parse", "--short", "HEAD") or "?"
    recent = _git("log", "-3", "--format=%h %s") or ""
    webots = resolve_omnisim_binary()
    worlds = _worlds()

    ports = {
        "harness": _port_status(HARNESS_PORT),
        "supervisor": _port_status(SUPERVISOR_PORT),
        "capture": _port_status(CAPTURE_PORT),
    }
    build = _build_provenance()
    physics = _physics_runtime(webots)
    python = _python_runtime()
    coherence = _coherence(build, physics)
    # A fatal coherence problem means the install cannot run a controller-driven
    # world. Reporting that and exiting 0 made every caller -- a script, a CI
    # lane, an agent branching on $? -- read a broken install as a pass, so the
    # exit code now follows the verdict. --strict is kept as an explicit alias
    # for callers (.githooks/pre-push) that already ask for gate semantics.
    exit_code = 0 if coherence["ok"] else 1

    if args.json:
        print(
            json.dumps(
                {
                    "omnisim_version": __version__,
                    "git_branch": branch,
                    "git_commit": commit,
                    "omnisim_binary": webots,
                    "webots_binary": webots,  # legacy alias for tools that still read the old key
                    "build": build,
                    "physics": physics,
                    "python": python,
                    "coherence": coherence,
                    "strict": args.strict,
                    "ports": ports,
                    "worlds_dir": str(DEMOS_WORLDS),
                    "worlds": worlds,
                    "recent_commits": recent.splitlines(),
                    "fingerprint": fingerprint,
                },
                indent=2,
            )
        )
        return exit_code

    print(f"omnisim     {__version__}")
    if commit == "?":
        # A packaged install ships no .git, so `? @ ?` was the first line the
        # primary README audience ever saw. Name the build instead.
        print(f"install     packaged (no git metadata) at {REPO_ROOT}")
    else:
        print(f"git         {branch} @ {commit}")
    if webots:
        print(f"binary      {webots}")
    else:
        print("binary      NOT FOUND -- this tree has no simulator to run")
        print("            build:  build_omni.bat                     (Windows)")
        print("            build:  bash scripts/install/linux_bootstrap.sh  (Linux)")
        print("            then:   make -C src/omnisim bundle-newton-runtime")
    # ASCII only: doctor must survive a cp1252 Windows console. A diagnostic that
    # UnicodeEncodeErrors while reporting the fault is worse than no diagnostic.
    if build["verdict"] == "incompatible":
        print(f"build       FAIL: ENGINE/libController MISMATCH -- {build['detail']}")
    elif build["verdict"] == "unknown":
        print(f"build       ?  {build['detail']}")
    elif build["detail"]:
        print(f"build       WARN: {build['detail']}")
    else:
        print("build       engine + libController compatible")
    # Physics is the check a newcomer most needs and could least guess at:
    # Newton is the only backend, so "absent" means nothing in any world will
    # ever move, and the engine reports that by exiting 0.
    if physics["status"] == "present":
        print(f"physics     Newton runtime OK ({physics['detail']})")
    elif physics["status"] == "absent":
        print(f"physics     FAIL: {physics['detail']}")
        if physics.get("fix"):
            print(f"            fix:  {physics['fix']}")
    else:
        print(f"physics     ?  {physics['detail']}")
    py_line = f"python      {python['cli']} (this CLI)"
    if python["spawned"] and python["spawned"] != python["cli"]:
        py_line += f"   {python['spawned_name']} on PATH: {python['spawned']}"
    elif not python["spawned"]:
        py_line += "   WARN: no python/python3 on PATH -- every Python controller will fail to start"
    print(py_line)
    if python["too_old"]:
        print("            WARN: newton 1.5.0 raises at ModelBuilder() on CPython 3.10 --")
        print("                  use 3.12 for the interpreter that supplies physics.")
    # Coherence advisories are things NOT already on the build line above (env
    # landmines, unreadable-binary). Shown always -- they are useful info -- but
    # they never change the exit code.
    for note in coherence["advisory"]:
        print(f"warn        {note}")
    print("ports")
    print(f"  6789 harness     {ports['harness']}")
    print(f"  6790 supervisor  {ports['supervisor']}")
    print(f"  6791 capture     {ports['capture']}")
    rel_worlds = DEMOS_WORLDS.relative_to(REPO_ROOT) if DEMOS_WORLDS.exists() else Path("(missing)")
    print(f"worlds      {len(worlds)} in {rel_worlds}")
    if worlds:
        head = ", ".join(worlds[:5])
        more = f" (+{len(worlds) - 5} more)" if len(worlds) > 5 else ""
        print(f"            {head}{more}")
    if recent:
        print("recent")
        for line in recent.splitlines():
            print(f"  {line}")
    if fingerprint is not None:
        from .conformance.report import format_fingerprint
        print(format_fingerprint(fingerprint))
    # THE VERDICT. This used to be printed only under --strict, so the default
    # report -- the one README, BETA and SUPPORT all tell a new user to run --
    # ended on a git log and left "am I OK to proceed?" unanswered. It is the
    # first question the command exists to answer, so it is always answered,
    # and it always names the next command.
    print()
    if coherence["ok"]:
        cli = invocation()
        print("VERDICT     READY.  Next:")
        print("              %-26s # see a robot move" % (cli + " demo"))
        print("              %-26s # the full catalogue" % (cli + " demos"))
    else:
        n = len(coherence["fatal"])
        print(f"VERDICT     NOT READY -- {n} blocking problem{'s' if n != 1 else ''}:")
        for i, msg in enumerate(coherence["fatal"], 1):
            print(f"  [{i}] {msg}")
        print("            Re-run `python -m omnisim doctor` after fixing.")
    return exit_code
