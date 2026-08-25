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

"""OmniBench shared per-platform launcher for direct omnisim-bin runs.

One place that owns, for every lane that launches the engine binary directly
(lane1/run_omnisim.py, lane3/determinism.py; lane2 tier C goes through the
production run_quad_walk_rl.sh launcher and only borrows bash_exe()):

  * binary resolution   — Windows: msys64/mingw64/bin/omnisim-bin.exe;
                          Linux: bin/omnisim-bin (RunPod layout), wrapped in
                          `xvfb-run -a`; falls back to omnisim.paths.
  * launch env          — OMNISIM_HOME/WEBOTS_HOME, per-run unique
                          OMNISIM_LOG_PATH, backend assert
                          (OMNISIM_REQUIRE_NEWTON; the retired
                          OMNISIM_FORCE_ODE / OMNISIM_LEGACY knobs are POPPED
                          from every child env so a stale export in the
                          operator's shell cannot steer a benchmark row), and
                          on Linux the runtime vars the `webots` launcher
                          shell would otherwise provide (LD_LIBRARY_PATH=
                          $OMNISIM_HOME/lib/webots, QT_QPA_PLATFORM=xcb, …).
                          Lane-specific scene knobs (OMNISIM_NEWTON_CONTACT_KD
                          etc.) pass through via `extra`.
  * timeout + tree-kill — taskkill /T on win32, killpg on POSIX (children are
                          started in their own session there).
  * Newton sidecar      — newton_verdict() reads the race-free
                          `<OMNISIM_LOG_PATH>.newton.json` backend verdict.
  * the retry loop      — launch_with_retries() for the back-to-back-launch
                          controller-connect race (growing settle, the
                          physics_oracle.py calibration).

Behavioral contract: this module reproduces exactly what the lane runners
did when verified individually — same argv shape, same env hygiene, same
sidecar rules. Do not "improve" the launch shape here without re-verifying
every lane.
"""
from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent          # .../omnibench/common
OMNIBENCH = HERE.parent
REPO = OMNIBENCH.parents[2]                     # tests/benchmarks/omnibench -> repo

IS_WIN = os.name == "nt"
IS_LINUX = sys.platform.startswith("linux")

# The proven headless launch shape (physics_oracle.py / run-headless): no
# --port (a local controller does not inherit it and then can't connect),
# stdout to a REAL file (a DEVNULL handle correlates with the flaky no-CSV
# first launch on Windows).
BASE_ARGS = ("--batch", "--mode=fast", "--no-rendering", "--minimize")


def resolve_binary(repo=None):
    """Locate omnisim-bin per-platform. Returns a Path or None."""
    repo = Path(repo or os.environ.get("OMNISIM_HOME") or REPO)
    if IS_WIN:
        candidates = [repo / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"]
    else:
        candidates = [repo / "bin" / "omnisim-bin", repo / "omnisim-bin"]
    for c in candidates:
        if c.exists():
            return c
    try:  # same resolver the CLI surface uses, as a last resort
        if str(repo) not in sys.path:
            sys.path.insert(0, str(repo))
        from omnisim.paths import resolve_webots_binary
        p = resolve_webots_binary()
        if p:
            return Path(p)
    except Exception:
        pass
    return None


def build_env(backend, log_path, repo=None, extra=None, base_env=None):
    """Child-process env for one engine run.

    backend: "newton" | None (None = leave the world's own choice).
        "ode" raises ValueError: src/ode was DELETED (commit bdc02139), so
        there is no such backend to launch. The current engine simply IGNORES
        OMNISIM_FORCE_ODE (verified 2026-08-08: the run comes up on Newton and
        writes a normal .newton.json sidecar), which means passing backend="ode"
        would silently produce a NEWTON row wearing an "ode" label. An earlier
        build the same day did honour it, and then the scene was FROZEN -- on
        tests/physics/worlds/contact_points.omniworld the cone sat at its authored pose
        (-5.951940 -4.392650 1.060230) for all 3000 ms while the Newton arm
        rolled it to (-5.976457 -5.292823 0.007408). Either way the row is a
        measurement of something other than what it claims, so it is refused.
    log_path: per-run unique OMNISIM_LOG_PATH (parallel children MUST NOT
    share one — the sidecar verdict and the log itself are per-run).
    extra: lane/scene-specific vars merged LAST (they win).
    """
    repo = Path(repo or REPO)
    env = dict(base_env if base_env is not None else os.environ)
    env["OMNISIM_HOME"] = str(repo)
    # a stale machine-scope WEBOTS_HOME can point at a phantom install
    env["WEBOTS_HOME"] = str(repo)
    env["OMNISIM_LOG_PATH"] = str(log_path)
    # Retired knobs are popped, never set: see the docstring.
    for k in ("OMNISIM_LEGACY", "OMNISIM_FORCE_ODE", "OMNISIM_REQUIRE_NEWTON"):
        env.pop(k, None)
    if backend is not None and backend != "newton":
        raise ValueError(
            "backend %r is not available: ODE was DELETED (src/ode, commit "
            "bdc02139) and Newton is the only physics backend. Pass 'newton' "
            "or None. Frozen ODE reference values: "
            "tests/goldens/ode_oracle_goldens.json" % backend)
    if backend == "newton":
        env["OMNISIM_REQUIRE_NEWTON"] = "1"  # fail loudly if Newton can't init
    if IS_LINUX:
        # running bin/omnisim-bin directly bypasses the `webots` launcher
        # shell — provide what it would (see omnisim/paths.linux_runtime_env)
        lib_webots = str(repo / "lib" / "webots")
        parts = [p for p in env.get("LD_LIBRARY_PATH", "").split(os.pathsep) if p]
        if lib_webots not in parts:
            env["LD_LIBRARY_PATH"] = os.pathsep.join([lib_webots] + parts)
        env.setdefault("QT_QPA_PLATFORM", "xcb")
        env.setdefault("WEBOTS_TMPDIR", "/tmp")
        env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    if extra:
        env.update({k: str(v) for k, v in extra.items()})
    return env


def kill_tree(proc):
    """Kill the engine and its whole child tree (controllers etc.)."""
    if IS_WIN:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       capture_output=True)
    else:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            proc.kill()


def launch_once(binpath, world, env, console_log, timeout_s, extra_args=(),
                cwd=None, base_args=None):
    """One engine launch, wall-clock capped, tree-killed on expiry.

    Returns (rc, wall_s, timed_out). On Linux the run is wrapped in
    `xvfb-run -a` (headless X server, the RunPod contract).

    base_args overrides BASE_ARGS for the rare probe that cannot use the
    standard headless shape. The only in-tree case is a RENDER-DEPENDENT
    device: `--no-rendering` leaves a Camera or Lidar with no image pipeline,
    and the controller then blocks on a frame that never arrives while the
    engine free-runs (measured: 61,440 steps on a 250-step lane-4 lidar probe,
    with an empty console because the controller's own stdout was not being
    forwarded either). Do NOT reach for this to "fix" a slow run.
    """
    argv = [str(binpath), str(world),
            *(BASE_ARGS if base_args is None else tuple(base_args)),
            *extra_args]
    if IS_LINUX and shutil.which("xvfb-run"):
        argv = ["xvfb-run", "-a"] + argv
    kwargs = {}
    if not IS_WIN:
        kwargs["start_new_session"] = True   # own pgid so killpg reaches all
    t0 = time.perf_counter()
    timed_out = False
    with open(console_log, "w", encoding="utf-8", errors="replace") as logf:
        proc = subprocess.Popen(argv, env=env, cwd=str(cwd or REPO),
                                stdout=logf, stderr=subprocess.STDOUT,
                                **kwargs)
        try:
            rc = proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            kill_tree(proc)
            rc = proc.wait()
    return rc, time.perf_counter() - t0, timed_out


def newton_verdict(log_path):
    """Read the race-free backend-verdict sidecar `<log>.newton.json` written
    at world finalize. Presence == Newton drove THIS run (OmLog deletes stale
    copies at startup); returns {"present": False} when absent."""
    sidecar = str(log_path) + ".newton.json"
    if not os.path.exists(sidecar):
        return {"present": False}
    try:
        with open(sidecar, "r", encoding="utf-8") as f:
            v = json.load(f)
        v["present"] = True
        return v
    except (OSError, ValueError) as e:
        return {"present": True, "error": str(e)}


def backend_proven(backend, verdict):
    """True iff the sidecar verdict proves the requested backend drove the run.

    Newton needs a present, non-degraded, finalised sidecar. The old "ode needs
    the sidecar ABSENT" rule is gone with the backend (bdc02139) -- and it was
    never proof of anything positive: an absent sidecar also describes a run
    that never reached world-finalize (AGENTS.md), so "ODE drove this" was
    always inferred from the absence of evidence rather than measured.
    """
    if backend == "newton":
        return bool(verdict.get("present") and not verdict.get("degraded")
                    and verdict.get("finalised", True))
    # backend None: caller left the world's own choice; nothing to prove.
    return True


def engine_attribution(verdict):
    """(engine label, reason-or-None) from a `newton_verdict()` result.

    THE FALLBACK IS NOT `"omnisim-ode"`. src/ode was DELETED (bdc02139) and
    Newton is the only physics backend, so a missing sidecar cannot mean "ODE
    drove this run" -- it means the backend was never VERIFIED: the load did not
    reach world-finalize (where the sidecar is written), the log path did not
    match, or the Newton runtime did not come up. A row published under the name
    of an engine that does not exist reads as evidence, which is worse than an
    unlabelled row (commit 17c92a211). The honest label is `omnisim-unverified`,
    and the reason travels into the row's `deviations` so the row explains
    itself.

    Lane 3 (`driveability.engine_attribution`) took a log PATH; this takes the
    already-read verdict so a caller that has one does not re-read the sidecar.
    Kept in `common/` because lane 1 published `omnisim-newton` unconditionally
    while lane 3 was being fixed -- one lane's fix is not the suite's.
    """
    if not verdict.get("present"):
        return "omnisim-unverified", (
            "engine=omnisim-unverified: no backend-verdict sidecar "
            "(<log>.newton.json) for this run. Newton is the only backend "
            "(src/ode deleted, bdc02139), so this means the backend was NOT "
            "VERIFIED -- the load never reached world-finalize, or the engine "
            "log path did not match -- NOT that ODE drove it.")
    if verdict.get("error"):
        return "omnisim-unverified", (
            "engine=omnisim-unverified: the backend-verdict sidecar exists but "
            "could not be read (%s). NOT an ODE run -- src/ode was deleted "
            "(bdc02139)." % (verdict["error"],))
    if verdict.get("degraded"):
        return "omnisim-unverified", (
            "engine=omnisim-unverified: the .newton.json backend verdict "
            "reports degraded=true (solver=%r), so Newton did not finalise "
            "cleanly. NOT an ODE run -- src/ode was deleted (bdc02139)."
            % (verdict.get("solver"),))
    if not verdict.get("finalised", True):
        return "omnisim-unverified", (
            "engine=omnisim-unverified: the .newton.json backend verdict "
            "reports finalised=false (solver=%r). NOT an ODE run -- src/ode "
            "was deleted (bdc02139)." % (verdict.get("solver"),))
    return "omnisim-newton", None


def launch_with_retries(*, attempts, attempt_fn, success_fn, label="",
                        log=None, settle=2.0):
    """Shared retry loop for the back-to-back-launch controller-connect race.

    attempt_fn(attempt_no) -> result (opaque to this loop);
    success_fn(result)     -> bool.
    Between failed attempts sleeps settle * attempt_no (growing settle, the
    physics_oracle.py calibration). Returns (result, attempts_used).
    """
    result = None
    for attempt in range(1, attempts + 1):
        result = attempt_fn(attempt)
        if success_fn(result):
            return result, attempt
        if attempt < attempts:
            if log:
                log("  [%s] attempt %d yielded no/short result; settling and "
                    "retrying..." % (label, attempt))
            time.sleep(settle * attempt)
    return result, attempts


def bash_exe():
    """A bash for the production shell launchers (lane2 tier C). On Windows a
    bare 'bash' can resolve to WSL's bash (System32), which mangles repo paths
    and runs the wrong python — measured failure mode. LANE2_BASH (legacy) or
    OMNIBENCH_BASH overrides."""
    if os.name != "nt":
        return "bash"
    cands = [os.environ.get("OMNIBENCH_BASH"), os.environ.get("LANE2_BASH"),
             r"C:\Program Files\Git\bin\bash.exe",
             r"C:\Program Files\Git\usr\bin\bash.exe"]
    for c in cands:
        if c and Path(c).exists():
            return c
    return shutil.which("bash") or "bash"
