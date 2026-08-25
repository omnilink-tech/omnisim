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
import socket
import subprocess
from pathlib import Path

from . import __version__
from .paths import DEMOS_WORLDS, REPO_ROOT, resolve_omnisim_binary


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
    return sorted(
        p.relative_to(DEMOS_WORLDS).as_posix() for p in [*DEMOS_WORLDS.rglob("*.omniworld"), *DEMOS_WORLDS.rglob("*.wbt")]
    )


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


def _coherence(build: dict) -> dict:
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
        help="Tier 0 install-coherence gate: exit NON-ZERO if the install is not "
        "coherent enough to run (engine/libController ABI mismatch, missing binary). "
        "For use in hooks / CI / launchers as a preflight. Default output is unchanged "
        "and always exits 0.",
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
    coherence = _coherence(build)
    # --strict is the ONLY thing that changes the exit code; plain doctor still
    # always exits 0 (it is a first-turn report, not a gate).
    exit_code = 1 if (args.strict and not coherence["ok"]) else 0

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
    print(f"git         {branch} @ {commit}")
    print(f"binary      {webots or 'NOT FOUND  (set OMNISIM_HOME or build the simulator)'}")
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
    # Coherence advisories are things NOT already on the build line above (env
    # landmines, unreadable-binary). Shown always -- they are useful info -- but
    # they never change the exit code, even under --strict.
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
    if args.strict:
        if coherence["ok"]:
            print("strict      OK -- install is coherent (Tier 0)")
        else:
            print(f"strict      FAIL -- {len(coherence['fatal'])} fatal coherence problem(s):")
            for msg in coherence["fatal"]:
                print(f"  {msg}")
    return exit_code
