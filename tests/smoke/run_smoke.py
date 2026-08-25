#!/usr/bin/env python3
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

"""Run a fast smoke subset using the existing test suite."""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import re
import shutil
import socket
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SMOKE_FILE = Path(__file__).with_name("smoke_worlds.json")

# Matches `  controller "name"` lines in a .wbt. Names like "<extern>", "<none>"
# and "void" denote no compiled controller, so they are filtered out below.
_CONTROLLER_RE = re.compile(r'controller\s+"([^"]+)"')

# test_suite.py spawns its own "background OmniSim" on this port and then asserts
# that each test instance falls back to 1235. A running GUI OmniSim on 1234 breaks
# that assumption and the test workers crash with a confusing access violation
# instead of a port error — so we detect and abort before launching anything.
WEBOTS_EXTERN_PORT = 1234

# Written by test_suite.py as the last line of tests/output.txt. Its absence
# means the suite died mid-run (killed, crashed, deadlocked) — no verdict can
# be trusted then, whatever the exit code says.
SUITE_COMPLETE_MARKER = "Test suite complete"

# Only worlds under tests/<group>/worlds/ actually run through the suite
# (test_suite.py filters everything else out), so only those owe a verdict.
_TEST_WORLD_RE = re.compile(r"(^|/)tests/[^/]+/worlds/[^/]+\.(?:omniworld|wbt)$")


def check_verdicts(active_worlds: list, content: str | None = None,
                   expected_runs: int = 1) -> list[str]:
    """Cross-check the suite output against the worlds we asked for.

    Returns a list of problems (empty = pass). The suite's exit code alone is
    NOT trustworthy: a test_suite.py killed mid-run once propagated as exit 0
    through the Windows/msys layer and a deadlocked push went through
    vacuously (2026-07-12). So the gate demands positive evidence: the
    completion marker, zero FAILURE lines, and an explicit "OK: <world>" per
    smoke world that the suite actually runs.

    `content` is the ACCUMULATED suite output. The lane runs test_suite.py
    once per backend group (see main()) and each run TRUNCATES
    tests/output.txt, so the caller concatenates the groups' outputs and
    passes them in; omitting it reads the file (single-group / legacy use).
    """
    output_file = REPO_ROOT / "tests" / "output.txt"
    if content is None:
        try:
            content = output_file.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            return [f"{output_file} was not written — the suite never ran"]

    problems = []
    # One marker per test_suite.py invocation. Counting (rather than a bare
    # "in") keeps the died-mid-run guard honest now that the lane makes one
    # invocation per backend group: a group that dies contributes no marker.
    marker_count = content.count(SUITE_COMPLETE_MARKER)
    if marker_count < expected_runs:
        problems.append(
            f'saw {marker_count} "{SUITE_COMPLETE_MARKER}" marker(s), expected '
            f"{expected_runs} (one per backend group) — a suite run died before finishing"
        )
    failure_count = content.count("FAILURE")
    if failure_count:
        problems.append(f"{failure_count} FAILURE line(s) in {output_file}")
    for item in active_worlds:
        if not _TEST_WORLD_RE.search(item["world"].replace("\\", "/")):
            continue  # not routed through a test group; no verdict expected
        stem = Path(item["world"]).stem
        if not re.search(rf"^OK: {re.escape(stem)}\s*$", content, re.MULTILINE):
            problems.append(f'missing "OK: {stem}" verdict for {item["name"]}')
    return problems


def _bind_blocked(family: int, addr: str, port: int) -> bool:
    """True if a fresh wildcard server socket cannot bind (family, addr, port).

    Mirrors the engine's own bind as closely as possible: wildcard address
    (a specific-address bind can succeed on Windows even while another
    process holds the wildcard) and SO_EXCLUSIVEADDRUSE on Windows (plain
    bind() can succeed over a dying process's socket that Qt still can't
    displace).
    """
    try:
        sock = socket.socket(family, socket.SOCK_STREAM)
    except OSError:
        return False  # family unsupported on this host (e.g. no IPv6)
    with contextlib.closing(sock):
        if sys.platform == "win32":
            with contextlib.suppress(OSError, AttributeError):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        if family == socket.AF_INET6:
            with contextlib.suppress(OSError):
                sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
        try:
            sock.bind((addr, port))
            return False
        except OSError:
            return True


def _listener_answers(family: int, addr: str, port: int) -> bool:
    """True only if a probe connection actually SUCCEEDS.

    Only success is evidence: on some Windows configurations (firewall
    stealth mode) loopback SYNs to closed ports are silently dropped, so a
    refusal AND a timeout both have to count as "no listener seen".
    """
    try:
        sock = socket.socket(family, socket.SOCK_STREAM)
    except OSError:
        return False  # family unsupported on this host
    with contextlib.closing(sock):
        sock.settimeout(1.0)
        try:
            sock.connect((addr, port))
            return True
        except OSError:
            return False


def _netstat_listener(port: int) -> bool:
    """Windows only: True if the kernel's TCP table has a LISTEN entry on port.

    This is the one detector that catches a ZOMBIE listener — an omnisim-bin
    stuck in kernel teardown whose socket still swallows SYNs but which no
    longer conflicts with fresh bind() calls (observed 2026-07-12: exactly
    that deadlocked the pre-push hook for hours while the bind()-only check
    passed). Matched on the address columns, not the state word, because
    netstat localizes state names.
    """
    if sys.platform != "win32":
        return False
    lines = []
    for proto in ("tcp", "tcpv6"):
        try:
            result = subprocess.run(
                ["netstat", "-ano", "-p", proto],
                capture_output=True, text=True, timeout=15, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        lines += result.stdout.splitlines()
    suffix = f":{port}"
    for line in lines:
        parts = line.split()
        # LISTEN rows have a wildcard foreign endpoint; the address columns
        # are locale-independent.
        if (len(parts) >= 3 and parts[0].upper() == "TCP"
                and parts[1].endswith(suffix)
                and parts[2] in ("0.0.0.0:0", "[::]:0", "*:*")):
            return True
    return False


def is_port_in_use(port: int) -> bool:
    # Check BOTH address families (the engine listens dual-stack, so an
    # IPv6-only squatter breaks it just as hard as an IPv4 one), back the
    # bind probes with connect probes, and on Windows scan the kernel TCP
    # table for zombie listeners that neither probe can see.
    return (
        _bind_blocked(socket.AF_INET, "", port)
        or _bind_blocked(socket.AF_INET6, "::", port)
        or _listener_answers(socket.AF_INET, "127.0.0.1", port)
        or _listener_answers(socket.AF_INET6, "::1", port)
        or _netstat_listener(port)
    )


def _build_env(base_env: dict) -> dict:
    """Env for compiling controllers: prepend the real MSYS2 MinGW64 toolchain.

    The bundled ``$REPO_ROOT/msys64/mingw64/bin`` ships only runtime DLLs, not a
    compiler, so a `make` there fails. Mirror build_omni.bat: honour MSYS64_HOME
    (default C:\\msys64) and put its mingw64/bin + usr/bin ahead on PATH.
    """
    env = base_env.copy()
    if sys.platform == "win32":
        msys64 = Path(env.get("MSYS64_HOME", r"C:\msys64"))
        toolchain = [str(msys64 / "mingw64" / "bin"), str(msys64 / "usr" / "bin")]
        env["PATH"] = os.pathsep.join(toolchain) + os.pathsep + env.get("PATH", "")
    return env


def _controller_dirs_for_world(world_rel: str) -> list[Path]:
    """Return existing controller dirs (in the world's own project) that need
    compiling — i.e. those with a Makefile and C/C++ source but no built binary.

    OmniSim resolves a controller relative to its project root (the parent of the
    ``worlds/`` directory). Python controllers and controllers supplied by other
    projects (e.g. the shared ``sun_marker``) have no Makefile here and are
    skipped automatically.
    """
    world_path = REPO_ROOT / world_rel
    project_root = world_path.parent.parent  # .../<project>/worlds/x.wbt -> <project>
    try:
        text = world_path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return []

    needs_build = []
    for name in dict.fromkeys(_CONTROLLER_RE.findall(text)):  # de-dup, keep order
        if name.startswith("<") or name == "void":
            continue
        ctrl_dir = project_root / "controllers" / name
        if not (ctrl_dir / "Makefile").is_file():
            continue
        has_source = any(ctrl_dir.glob(f"*{ext}") for ext in (".c", ".cpp", ".cc", ".cxx"))
        if not has_source:
            continue
        binary = ctrl_dir / (f"{name}.exe" if sys.platform == "win32" else name)
        if not binary.exists():
            needs_build.append(ctrl_dir)
    return needs_build


def ensure_controllers_built(active_worlds: list, env: dict) -> bool:
    """Compile any missing C/C++ controllers the smoke worlds depend on.

    Without this, a fresh clone (or a tree after `make clean`) has no controller
    binaries, the test controller never runs, and the only symptom is a 30s
    "results file has not been written" timeout — easily misread as a renderer
    or physics regression. Returns True on success, False if a build failed.
    """
    to_build = []
    for item in active_worlds:
        for ctrl_dir in _controller_dirs_for_world(item["world"]):
            if ctrl_dir not in to_build:
                to_build.append(ctrl_dir)

    if not to_build:
        return True

    build_env = _build_env(env)
    if sys.platform == "win32" and not Path(build_env.get("MSYS64_HOME", r"C:\msys64")).exists():
        print(
            f"[smoke] {len(to_build)} controller(s) need compiling but the MSYS2 "
            "toolchain was not found. Set MSYS64_HOME or build manually.",
            file=sys.stderr,
        )
        return False

    # On Windows, subprocess resolves the program name against the *parent*
    # PATH, not the child env's PATH, so look the toolchain `make` up explicitly
    # and pass its absolute path.
    make_name = "mingw32-make" if sys.platform == "win32" else "make"
    make = shutil.which(make_name, path=build_env.get("PATH"))
    if make is None:
        print(
            f"[smoke] {len(to_build)} controller(s) need compiling but '{make_name}' "
            "was not found on the toolchain PATH (set MSYS64_HOME).",
            file=sys.stderr,
        )
        return False

    for ctrl_dir in to_build:
        print(f"[smoke] building missing controller: {ctrl_dir.relative_to(REPO_ROOT)}")
        result = subprocess.run(
            [make, "release"], cwd=ctrl_dir, env=build_env, check=False,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        if result.returncode != 0:
            print(result.stdout, file=sys.stderr)
            print(f"[smoke] FAILED to build {ctrl_dir.name}.", file=sys.stderr)
            return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Run OmniSim smoke worlds")
    parser.add_argument("--nomake", action="store_true", help="Skip the initial controller rebuild")
    args = parser.parse_args()

    if is_port_in_use(WEBOTS_EXTERN_PORT):
        print(
            f"[smoke] ABORT: TCP port {WEBOTS_EXTERN_PORT} is already in use — "
            "likely a running Webots/OmniSim GUI.",
            file=sys.stderr,
        )
        print(
            "[smoke] The smoke runner spawns its own background OmniSim on this "
            "port and asserts test instances fall back to 1235; a pre-existing "
            "process on 1234 makes the test workers crash with a misleading "
            "access violation.",
            file=sys.stderr,
        )
        print(
            "[smoke] Fix: close the running OmniSim window, or bypass with "
            "OMNISIM_SKIP_PUSH_CHECK=1 git push. If no OmniSim is visibly "
            "running, look for a zombie omnisim-bin still holding the port "
            "(Get-NetTCPConnection -LocalPort 1234) — an unkillable one "
            "needs a reboot to clear.",
            file=sys.stderr,
        )
        return 2

    # encoding is EXPLICIT: read_text() defaults to the locale codec, which on
    # Windows is cp1252 and raises UnicodeDecodeError the moment a skip_reason
    # picks up a non-ASCII character.
    worlds = json.loads(SMOKE_FILE.read_text(encoding="utf-8"))
    env = os.environ.copy()
    # Hard-set the install root to THIS clone so a system-wide
    # `OMNISIM_HOME=C:\Program Files\Webots` (from a separate upstream
    # Webots install) doesn't redirect the smoke run to the wrong tree.
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["OMNISIM_HOME"] = str(REPO_ROOT)  # legacy alias

    # On Windows, test_suite.py spawns webots.exe directly (no launch.bat
    # wrapper), so the msys64 mingw64 bin dir must already be on PATH for
    # the launcher to find Qt6 / MinGW runtime DLLs.
    if sys.platform == "win32":
        bin_dir = str(REPO_ROOT / "msys64" / "mingw64" / "bin")
        env["PATH"] = bin_dir + os.pathsep + env.get("PATH", "")

    active_worlds = [w for w in worlds if not w.get("skip")]
    skipped = [w for w in worlds if w.get("skip")]

    # ---- ONE BACKEND, ONE GROUP (re-goldened 2026-08-08) -----------------
    # HISTORY, because the shape of this lane is the interesting part:
    #
    #  1. It used to hard-set OMNISIM_FORCE_ODE=1 for EVERY world, on the theory
    #     that all the smoke worlds carry ODE-tuned goldens. That hid WHICH
    #     worlds actually needed ODE.
    #  2. It was then split into a "default backend" group and a short,
    #     individually-justified "ODE-pinned" group of three worlds.
    #  3. src/ode was DELETED (commit bdc02139). Both the blanket pin and the
    #     ODE group became useless, and for a while ACTIVELY MISLEADING -- see
    #     the contact_points measurement in smoke_worlds.json: on the build
    #     current that morning, OMNISIM_FORCE_ODE=1 made the engine build no
    #     physics at all, so the scene stayed frozen at its authored pose and a
    #     red physics test read GREEN. A green verdict from a frozen scene is
    #     worse than a red one, so the group is gone rather than retargeted.
    #     (The engine was rebuilt the same day and now IGNORES that var, so the
    #     trap is closed at the source too -- but the lane does not rely on that.)
    #
    # Current shape: one group, the engine default (Newton/MuJoCo). A world that
    # cannot pass on it is `skip: true` with a measured `skip_reason` naming the
    # engine-side gap -- never coaxed green with a pin or a retuned golden.
    #
    # ⚠️ GAP THIS LEAVES, stated rather than buried: of the three worlds that
    # were ODE-pinned, template_deterministic now genuinely PASSES on Newton
    # (the DistanceSensor raycast default going ON in 6eb35675 fixed it -- its
    # four sensors used to read a constant, collapsing the PROTO-determinism
    # verdict), while accelerometer and contact_points are skipped. The lane
    # therefore currently asserts NO dynamics anywhere. Restoring a physics
    # assertion here needs a tests/physics world with a Newton-valid golden;
    # the candidates checked on 2026-08-08 (damping, hinge_joint_damping,
    # rolling_friction, floating_point_precision) all fail for an unrelated
    # reason -- their C controllers are not built in this clone, which surfaces
    # only as a 30 s "results file has not been written" timeout.
    print("[smoke] Running worlds (engine default backend — Newton/MuJoCo):")
    for item in active_worlds:
        print(f"[smoke] - {item['name']}: {item['world']}")
    for item in skipped:
        reason = item.get("skip_reason", "(no reason given)")
        print(f"[smoke] - SKIPPED {item['name']}: {item['world']}  — {reason}")

    # A missing controller binary surfaces only as a 30s "results file not
    # written" timeout, so self-heal here before launching the suite. Treat a
    # build failure as an environment problem (exit 2) like the port check.
    if not ensure_controllers_built(active_worlds, env):
        print(
            "[smoke] Could not compile required controllers. Build them manually "
            "or bypass with OMNISIM_SKIP_PUSH_CHECK=1 git push.",
            file=sys.stderr,
        )
        return 2

    output_file = REPO_ROOT / "tests" / "output.txt"
    problems: list[str] = []
    accumulated = ""
    runs = 0

    # RETIRED KNOBS, SCRUBBED ON PURPOSE. OMNISIM_FORCE_ODE / OMNISIM_LEGACY name
    # a backend that no longer exists (src/ode deleted, bdc02139). The current
    # engine ignores them, but this lane does not depend on that: the same knob
    # DID produce a physics-free, vacuously-green run earlier the same day (see
    # the contact_points note in smoke_worlds.json), and a pre-push gate should
    # not be one engine revision away from grading a frozen scene. Warn, drop.
    for key in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
        if env.pop(key, None) is not None:
            print(f"[smoke] WARNING: {key} is set in your environment and is "
                  "RETIRED — src/ode was DELETED (commit bdc02139) and Newton "
                  "is the only physics backend. It is being REMOVED from the "
                  "child env rather than passed through: it cannot select a "
                  "backend, and a build that still honoured it would run the "
                  "worlds with no physics at all. Unset it.",
                  file=sys.stderr)

    if active_worlds:
        cmd = [sys.executable, str(REPO_ROOT / "tests" / "test_suite.py")]
        if args.nomake:
            cmd.append("--nomake")
        cmd.extend(str(REPO_ROOT / item["world"]) for item in active_worlds)

        print(f"[smoke] === running {len(active_worlds)} world(s) ===")
        suite_rc = subprocess.run(cmd, cwd=REPO_ROOT, env=env,
                                  check=False).returncode
        runs += 1
        if suite_rc != 0:
            # Normalize: test_suite.py exits with its raw failure COUNT, which
            # can exceed 255 and wrap to 0 through the shell on POSIX.
            problems.append(f"test_suite.py exited {suite_rc}")
        try:
            accumulated += output_file.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            problems.append(f"{output_file} was not written")

    problems.extend(check_verdicts(active_worlds, accumulated, expected_runs=runs))
    if problems:
        print("[smoke] FAILED:", file=sys.stderr)
        for p in problems:
            print(f"[smoke] - {p}", file=sys.stderr)
        return 1
    print(f"[smoke] All {len(active_worlds)} smoke world(s) passed with explicit "
          f"verdicts on the engine default backend "
          f"({len(skipped)} skipped, reasons printed above).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
