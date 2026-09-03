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

"""Run a world headlessly, monitor log output, exit after duration or on error.

Usage:
    python scripts/dev/headless_runner.py <world> [--duration 10] [--fail-on-warning]

This is the supported headless execution contract for OmniSim.
It starts the simulator in batch mode, monitors omnisim_log.txt for errors,
runs for the specified duration, then terminates and reports results.
"""

from __future__ import annotations

import argparse
import atexit
import codecs
import json
import locale
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
from omnisim.paths import linux_runtime_env  # noqa: E402


class IncrementalLogText:
    """Tail a log file without re-reading it from byte 0 on every poll.

    The 0.5 s poll loop used to `read_text()` the whole engine log each
    iteration -- O(n^2) disk work over a run once the log reaches megabytes.
    This keeps the accumulated text in memory and reads only the bytes
    appended since the last poll, so callers can run the same whole-text
    regexes as before with identical results. If the file shrinks
    (truncate-on-reload: OmLog truncates at engine startup), the offset is
    reset and the text is rebuilt from byte 0. Decoding uses the same
    default locale encoding + errors="replace" that read_text() used, via
    an incremental decoder so a multi-byte char split across two polls
    still decodes correctly.
    """

    def __init__(self, path: Path) -> None:
        self.path = path
        self._offset = 0
        self._text = ""
        self._pending = ""  # a trailing "\r" held back until its "\n" arrives
        self._encoding = locale.getpreferredencoding(False)
        self._decoder = codecs.getincrementaldecoder(self._encoding)("replace")

    def _reset(self) -> None:
        self._offset = 0
        self._text = ""
        self._pending = ""
        self._decoder = codecs.getincrementaldecoder(self._encoding)("replace")

    def read(self) -> str:
        """Return the file's full text so far ("" if absent), reading only new bytes."""
        try:
            if not self.path.exists():
                self._reset()
                return ""
            size = self.path.stat().st_size
            if size < self._offset:
                self._reset()
            if size > self._offset:
                with open(self.path, "rb") as fh:
                    fh.seek(self._offset)
                    chunk = fh.read()
                self._offset += len(chunk)
                decoded = self._pending + self._decoder.decode(chunk)
                self._pending = ""
                # read_text() translated newlines universally; do the same,
                # holding back a chunk-final "\r" so a "\r\n" split across two
                # polls still collapses to one "\n".
                if decoded.endswith("\r"):
                    self._pending = "\r"
                    decoded = decoded[:-1]
                self._text += decoded.replace("\r\n", "\n").replace("\r", "\n")
        except OSError:
            # Mirror the old per-iteration read_text() behaviour: an OSError
            # poll yields "" for this iteration without poisoning the state.
            return ""
        return self._text


def find_binary(omnisim_home: Path) -> Path:
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        binary = Path(override).expanduser().resolve()
        if not binary.is_file():
            raise SystemExit(f"OMNISIM_BINARY does not exist: {binary}")
        return binary
    if sys.platform == "win32":
        candidates = [
            # Canonical name first; the legacy webots-named launcher is a
            # byte-identical copy the build still refreshes, kept as a
            # fallback for checkouts where only the old name exists.
            omnisim_home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
            omnisim_home / "msys64" / "mingw64" / "bin" / "webots.exe",
        ]
    elif sys.platform == "darwin":
        candidates = [
            omnisim_home / "Contents" / "MacOS" / "omnisim",
            omnisim_home / "Contents" / "MacOS" / "webots",
        ]
    else:
        candidates = [
            omnisim_home / "bin" / "omnisim-bin",
            omnisim_home / "omnisim",
            omnisim_home / "webots",
        ]

    for c in candidates:
        if c.exists():
            return c
    raise SystemExit(f"Cannot find simulator binary under {omnisim_home}")


# ── GROUND-TRUTH "did the sim actually step?" detection ──────────────────────
#
# The Newton backend logs this line UNCONDITIONALLY from OmNewtonBackend::step()
# (src/omnisim/physics/OmNewtonBackend.cpp) on the very first physics step
# (stepCount == 1) and at a handful of later checkpoints (30, 60, 120, ...):
#
#     INFO: [OmNewtonBackend] step 1 dt=0.016s b0=(...) b1=(...) ...
#
# It is ENGINE-emitted ground truth that the simulation clock ADVANCED -- not
# the controller's per-tick trace (which prints once per SIMULATED second, so a
# hung run and a slow run look identical there; see commit 6eea9d76). Because
# step 1 fires on the FIRST step, even a run so slow it manages a single step
# still logs it -- which is exactly what lets us tell a genuinely hung sim apart
# from a merely slow one WITHOUT a false FAIL.
#
# The precondition -- that the world got far enough that stepping SHOULD have
# begun -- is established by the Newton verdict sidecar (<log>.newton.json,
# written AT finalize) and/or the engine's own "world finalised" log line. Both
# come from the same finalize event; either is proof the build reached the point
# where the first step follows within milliseconds in any healthy run.
# The bracketed tag is named after the emitting C++ class, and those classes are
# being renamed Wb* -> Om*, so the same line will read "[OmNewtonBackend] ..."
# once the engine rename lands.  BOTH forms are accepted here, permanently: the
# runner must not care which side of the rename the engine is on, and log files
# captured before the rename must keep parsing forever.  Do not narrow these to
# one prefix -- _NEWTON_STEP_RE gates both the duration clock and the
# sim_never_stepped verdict, so a miss stalls every run to the startup timeout
# and then FAILs it.
_NEWTON_TAG = r"\[(?:Wb|Om)NewtonBackend\]"
_NEWTON_STEP_RE = re.compile(_NEWTON_TAG + r" step \d")
_NEWTON_FINALISED_RE = re.compile(_NEWTON_TAG + r" world finalised")

# The engine NAMES the GPU solver in its own log, twice and early:
#   INFO: [OmNewtonBackend] solver preference set to 'mujoco_warp'      (pre-finalize)
#   INFO: [OmNewtonBackend] world finalised (solver=MuJoCo (mujoco_warp, ...))
# Used only as a FALLBACK hint for the post-finalize wait below, when the verdict
# sidecar cannot be read. The sidecar is the honest signal (it reports the device
# the model actually lived on); the world file is not consulted at all, because
# WorldInfo.newtonSolver is not the last word -- env vars and defaults are.
_WARP_PATH_RE = re.compile(_NEWTON_TAG + r".*mujoco_warp")

# ── POST-FINALIZE WAIT FOR THE FIRST PHYSICS STEP ────────────────────────────
#
# --until-finalized used to end the run --completion-grace seconds (2 s) after
# the "world finalised" line. On the CPU solver that is generous: the first step
# follows finalize within milliseconds. On the GPU path (newtonSolver
# "mujoco_warp") it is not -- warp compiles its CUDA kernels AFTER finalize and
# the engine logs NOTHING while it does. MEASURED on machine 9722d23d12a3
# (RTX 3060 laptop, warehouse_husky.omniworld, newton 1.5.0 / warp 1.16.0,
# engine d6ae37417):
#
#     warm warp kernel cache   finalise  6.91 s -> first step   8.25 s  (gap  1.33 s)
#     cold warp kernel cache   finalise 69.12 s -> first step 164.54 s  (gap 95.42 s)
#
# So the 2 s grace can end the run in the middle of kernel compilation. The run
# then records ZERO steps, and the sim_never_stepped guard turns a healthy world
# into an exit-1 FAIL. The world is fine; the check was too short.
#
# The wait is therefore sized from the MODEL DEVICE the ENGINE reports in its
# verdict sidecar (<log>.newton.json -> runtime.device, "cpu" or "cuda:0").
# Crucially it ENDS the moment the first step appears, so a healthy run costs
# exactly what it cost before (the CPU control world is unchanged, measured):
# the budget is only ever spent by a run that is not stepping.
STEP_WAIT_CPU_S = 10.0
STEP_WAIT_GPU_S = 180.0


def is_gpu_device(device: str | None) -> bool:
    """True for a warp device that is not the CPU ("cuda:0", "cuda", ...).

    Unknown (None/empty) is NOT a GPU: an unreadable sidecar must not silently
    buy a 180 s budget. The caller passes the log-derived hint separately.
    """
    if not device:
        return False
    return not device.strip().lower().startswith("cpu")


def step_wait_budget_s(device: str | None, warp_in_log: bool = False,
                       override: float | None = None) -> float:
    """Seconds to wait for the FIRST physics step once the world has finalised.

    Precedence, most trustworthy first:
      1. an explicit --step-wait-timeout (0 restores the pre-2026-08-16 behaviour
         of stopping as soon as the grace expires),
      2. the sidecar's model device -- the engine's own report,
      3. the engine log naming mujoco_warp, used ONLY when the sidecar could not
         be read (it is written a few ms after the finalize line it follows).
    An explicit "cpu" device wins over the log hint on purpose: the sidecar
    describes what the model did, the log line only what was requested.
    """
    if override is not None and override >= 0:
        return float(override)
    if is_gpu_device(device):
        return STEP_WAIT_GPU_S
    if device is None and warp_in_log:
        return STEP_WAIT_GPU_S
    return STEP_WAIT_CPU_S

# These are controller-launch failures, not ordinary world warnings.  A world can
# still parse, finalise and step after a Python controller dies, which made the old
# runner print PASS for demos whose policy process never existed.
_CONTROLLER_START_FAILURES = (
    re.compile(r"failed to start(?: controller)?", re.IGNORECASE),
    re.compile(r"python (?:was|is) not found", re.IGNORECASE),
    re.compile(r"unable to find .*python.*executable", re.IGNORECASE),
    # NOTE the optional colon: the engine prints "exited with status: 1" (colon), which the
    # old space-only pattern missed -- so a dead controller (e.g. a BATON deploy whose import
    # crashed) was silently reported PASS. Caught by the go2 BATON dry-run before v5.3.0.
    re.compile(r"(?:controller|python).*exited with (?:status|code):? *[1-9]", re.IGNORECASE),
    # Phase I1/I2 (core-evolution-plan.md): IPC-handshake and pairing failures are ALWAYS
    # fatal, not --fail-on-warning-gated. Each one means a robot executed zero steps:
    # a build-mismatched engine/libController pair (handshake refused), a stale-instance
    # pipe crossing, or a controller that never paired (the zero-tick watchdog).
    re.compile(r"OmniSim IPC (?:handshake|protocol version)", re.IGNORECASE),
    re.compile(r"never paired with the simulator", re.IGNORECASE),
    re.compile(r"DIFFERENT simulator instance", re.IGNORECASE),
)


def controller_start_failures(log_text: str) -> list[str]:
    """Return distinct log lines proving that a controller failed to launch."""
    failed = []
    for line in log_text.splitlines():
        if any(pattern.search(line) for pattern in _CONTROLLER_START_FAILURES):
            if line not in failed:
                failed.append(line)
    return failed


# ── THE KNOWN BACK-TO-BACK LAUNCH RACE ───────────────────────────────────────
#
# A launch dies during engine startup with these Qt teardown warnings and NO
# diagnostics: no ERROR line, no stderr, exit code 1, log truncated right after
# the header. Rate: AgentBench's adapter records "roughly one launch in three"
# on this machine; this session saw 3 of ~15, all of them back-to-back runs.
# It is a startup race in the engine, NOT a fault in the world under test --
# tests/benchmarks/agentbench/adapters/omnisim/headless.py already retries it
# for the same reason. A bare "simulator exited early with code 1" sends the
# reader looking for a defect in their world (measured: a `--port=1300` run was
# reported as "port forwarding is broken in the headless lane" when a rerun of
# the identical command passed), so we name the race instead.
#
# RATE, DATED (public issue #3 asked for it): the "one in three" above is the
# AgentBench adapter's 2026-07 figure and batch_validate.py saw 1-of-19 and
# 3-of-10 on 2026-08-15 -- both on this same machine (9722d23d12a3), both on
# older binaries. Re-measured 2026-08-29 on the then-current binary: 0 of 80
# (25 raw back-to-back, 25 raw with the next engine starting while the previous
# was still tearing down, 15 through this runner with --port pinned, 15 through
# this runner rotating the 8-Husky swarm / warehouse_husky / husky smoke worlds).
# So the rate is NOT stable across builds, and the retry below stays because a
# 0-of-80 on one box is not a proof of absence. Re-measure on YOUR binary with
# scripts/dev/launch_race_stress.py -- it names the binary and machine in its
# output so the number can be compared later.
#
# ATTRIBUTED, 2026-08-29 (the same evening as the 0-of-80): the 0-of-80 was
# measured with engines started one AFTER another; started AGAINST an engine
# that was already running (`launch_race_stress.py --concurrent 2 --stagger 12`)
# the failure came back at 3 of 7 rounds, and it was never Qt's. On Windows the
# engine (a GUI-subsystem binary) used to AttachConsole() to its launcher's
# console whenever its stdout was anything but a pipe -- so an engine handed
# the null device or a FILE threw that handle away for a console it did not own
# and shared with every other process that launcher had spawned. In some of
# those engines fd 1 was dead by the time the embedded Python interpreter first
# wrote to it: warp's greeting raised "[Errno 9] Bad file descriptor" out of
# newton.ModelBuilder(), the FFI smoke read that as a broken runtime, FATAL,
# exit 1 -- and the Qt teardown marks are simply what any exit(1) during
# startup prints. The 2026-07 "1 in 3" is the same family: the cold-launch
# `Fatal Python error: init_sys_streams` (OmNewtonBackend.cpp) also came from
# CPython building sys.stdout on a console the engine had attached to. Fixed
# two ways in the engine: it no longer attaches when the launcher gave it ANY
# stdout (main.cpp RedirectIOToConsole), and the interpreter's stdio probe is
# now os.fstat() rather than a zero-length write that could not fail
# (OmNewtonBackend.cpp). The retry here still catches the Qt-marked shape on
# older binaries; a FATAL from the FFI smoke is an ERROR line and is correctly
# NOT retried -- on a fixed binary it is a real broken install.
_STARTUP_RACE_MARKS = (
    "QWaitCondition: Destroyed while threads are still waiting",
    "QThreadStorage: entry",
)


def looks_like_startup_race(log_text: str) -> bool:
    """True when an early non-zero exit carries the startup-race signature.

    Requires the Qt teardown marks AND the absence of any ERROR/FATAL line: a
    world that genuinely failed to load must never be excused as "the flake".

    THIS IS THE SINGLE DEFINITION OF THE RULE. It is unit-tested in
    tests/harness/test_headless_verdicts.py and is what the retry loop in
    `main()` gates on -- nothing else in this file, and nothing that imports
    this module, may re-derive it.

    Relationship to AgentBench (checked 2026-07-28, do not "unify" these by
    guesswork): tests/benchmarks/agentbench/adapters/omnisim/headless.py does
    retry the SAME race, but it does NOT use this signature. Its gate is
    `stack_broke(res)` -- "phase B produced no trajectory, the world was not
    refused for tampering, the engine named no ERROR:, and the recorder never
    attested completion". That is a strictly BROADER rule built on evidence
    only the grader has (a recorder CSV and a meta.json), so it cannot be
    imported here: this lane has no recorder, and a run that merely produced
    no CSV is not a fact this runner can observe. What the two DO share, and
    what matters, is the veto: neither retries once the engine has printed an
    ERROR: line, because a world that genuinely failed to load must never be
    retried into looking clean. Keep that half in lockstep.
    """
    if not any(mark in log_text for mark in _STARTUP_RACE_MARKS):
        return False
    for line in log_text.splitlines():
        if line.startswith("ERROR:") or line.startswith("FATAL:"):
            return False
    return True


# ── EARLY-EXIT CAUSE (public issue #6) ───────────────────────────────────────
#
# When the engine exits before the world loads, the reason is usually in the
# engine LOG, not on stderr -- and on Windows omnisim-bin.exe is a GUI-subsystem
# binary, so its stderr is discarded outright. The canonical case: Qt cannot
# initialise a platform plugin (no display / a partial libxcb set on Linux, a
# stale QT_QPA_PLATFORM anywhere). The engine's Qt message handler writes that
# as `Qt Fatal: This application failed to start because no Qt platform plugin
# could be initialized.` and abort()s (exit 3 on Windows, SIGABRT/134 on Linux)
# with a header-only log. This runner already FAILed on that (03e988c58) --
# but it printed only "simulator exited early with code N", which sent the
# reader to the world. Surface the lines that say why, then the fix.
_EARLY_EXIT_CAUSE_PREFIXES = ("Qt Fatal:", "Qt Critical:", "FATAL:", "ERROR:")
_PLATFORM_PLUGIN_MARK = "no Qt platform plugin could be initialized"


def early_exit_cause_lines(log_text: str, limit: int = 12) -> list[str]:
    """The engine-log lines that say WHY an early exit happened, in log order.

    Qt Fatal / Qt Critical / FATAL / ERROR lines only; capped at `limit` so a
    flood cannot bury the first one, which is nearly always the cause.
    """
    out = []
    for line in log_text.splitlines():
        stripped = line.strip()
        if stripped.startswith(_EARLY_EXIT_CAUSE_PREFIXES):
            out.append(stripped)
            if len(out) >= limit:
                break
    return out


def platform_plugin_hint(log_text: str) -> list[str]:
    """Actionable lines when the early exit is Qt failing to start a platform plugin.

    Empty when the signature is absent. The advice is what this tree has
    actually verified: the Linux CI smokes run under `xvfb-run -a` with the
    libxcb set `linux_bootstrap.sh deps` installs, and the runtime container
    wraps every invocation in xvfb-run for the same reason.
    """
    if _PLATFORM_PLUGIN_MARK not in log_text:
        return []
    qpa = os.environ.get("QT_QPA_PLATFORM")
    lines = [
        "QT PLATFORM PLUGIN FAILED: the engine aborted inside Qt's platform-plugin init,",
        "  BEFORE the world was opened. No simulation was executed; this says nothing about the world.",
    ]
    if sys.platform.startswith("linux"):
        lines += [
            "  The engine constructs a Qt application even under --no-rendering / --no-window, so it",
            "  needs a display. Run it under a virtual one:   xvfb-run -a python -m omnisim run-headless ...",
            "  and make sure the Qt xcb plugin's libraries are installed:   bash scripts/install/linux_bootstrap.sh deps",
            "  (libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 ...).",
        ]
    else:
        lines += [
            "  On this platform the usual cause is a QT_QPA_PLATFORM value naming a plugin this build",
            "  does not ship (the log's 'Available platform plugins are:' line lists what it has).",
        ]
    if qpa:
        lines.append(f"  QT_QPA_PLATFORM is currently set to '{qpa}' in this environment -- unset it unless you mean it.")
    return lines


def writable_log_path(preferred: "Path") -> tuple["Path", str | None]:
    """`preferred`, or a per-user temp fallback when its directory refuses writes.

    A stock Windows install lands in C:/Program Files, where a non-elevated
    process cannot create files -- so the default omnisim_log.txt (and the
    .stdout/.stderr sinks derived from it) died with PermissionError on the
    documented first-run path (public issue #14's report). Probe by actually
    creating a file: existence and os.access() both lie under Windows ACLs.
    Returns (path, note) -- note is None when the preferred path is used.
    """
    probe = preferred.parent / (preferred.name + ".probe")
    try:
        preferred.parent.mkdir(parents=True, exist_ok=True)
        with open(probe, "w"):
            pass
        probe.unlink()
        return preferred, None
    except OSError:
        import tempfile
        fallback = Path(tempfile.gettempdir()) / "omnisim" / preferred.name
        fallback.parent.mkdir(parents=True, exist_ok=True)
        return fallback, (
            f"{preferred.parent} is not writable (a Program Files install, most "
            f"likely) -- logging to {fallback} instead. Set OMNISIM_LOG_PATH to "
            f"choose the location yourself.")


# Distinct exit code for "every attempt hit the startup race". 75 is
# sysexits.h EX_TEMPFAIL ("temporary failure, the user is invited to retry"),
# which is exactly the claim: nothing was learned about the world. A caller
# that treats 1 as "this world is broken" must NOT treat 75 the same way.
EXIT_STARTUP_RACE = 75

# Attempts, and the pause between them. The race is a back-to-back-launch
# race, so the pause is the active ingredient, not politeness: omnibench's
# launch_with_retries and the AgentBench adapter both use a GROWING settle
# (2.0 * attempt), calibrated in scripts/dev/physics_oracle.py. Same here.
#
# WHY 5 AND NOT 3. Measured 2026-07-28 on warehouse_omnilink.omniworld (RTX 3060
# laptop), 44 back-to-back launches through this runner over four campaigns:
# 29 of 73 engine attempts raced (39.7%, in line with the ~1-in-3 the
# AgentBench adapter records) and 20 of 44 launches raced at least once --
# but the races are CLUSTERED, not independent. With a 3-attempt budget the
# very first launch of the first campaign raced THREE TIMES IN A ROW and
# exhausted it, while the following eight launches raced zero times.
# Clustering is the whole hazard: a budget sized for the average rate fails
# exactly when the machine is cold or busy, which is when a live demo is
# started. With 5 attempts, all 18 raced launches across the three later
# campaigns (34 launches) recovered and none exhausted the budget; the worst
# run of consecutive races seen in 73 attempts was 3, so 5 keeps two spare.
# A raced attempt is also cheap -- it dies in ~4 s, versus ~15 s for a
# healthy run -- so the pathological worst case is ~20 s of attempts plus
# 20 s of settle, and the median case pays nothing at all.
DEFAULT_RACE_ATTEMPTS = 5
RACE_SETTLE_S = 2.0


# ── PHYSICAL-PLAUSIBILITY VERDICT (--fail-on-runaway) ─────────────────────────
#
# Defaults are deliberately conservative: this lane's job is to stop certifying
# an IMPOSSIBLE world, not to referee marginal physics.
#
#   RUNAWAY_Z_LIMIT_M   absolute |z| past which no body in any OmniSim world is
#                       legitimately positioned (aerial demos fly <100 m).
#   FALL_MARGIN_M       how far BELOW the lowest static collision surface a body
#                       must be. A body still above the floor is falling, which
#                       is legal; a body 2 m under it has left the world.
#   NO_FLOOR_MARGIN_M   the same idea when the world has NO static body to
#                       measure against: fall back to the body's own start
#                       height, with a much bigger margin.
#   RUNAWAY_VZ_MIN_MS   downward speed at exit. Free fall reaches 5 m/s in 0.5 s.
#   RUNAWAY_WINDOW      consecutive vz samples that must each be MORE negative
#                       than the last, i.e. nothing is resisting gravity.
RUNAWAY_Z_LIMIT_M = 1000.0
FALL_MARGIN_M = 2.0
NO_FLOOR_MARGIN_M = 20.0
RUNAWAY_VZ_MIN_MS = 5.0
RUNAWAY_WINDOW = 3


def read_runaway_samples(path: Path) -> dict:
    """Parse the watchdog JSONL. Tolerates a truncated final line."""
    header: dict = {}
    rows: list[tuple[float, dict]] = []
    complete = False
    try:
        text = path.read_text(errors="replace")
    except OSError:
        return {"present": False, "header": {}, "rows": [], "complete": False}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except ValueError:
            continue  # a half-written final line: the kill landed mid-write
        kind = obj.get("kind")
        if kind == "header":
            header = obj
        elif kind == "floor":
            # Emitted AFTER the header (the static-collider scan is bounded but
            # not free, and the header must exist even if the scan is cut short).
            header.update({k: v for k, v in obj.items() if k != "kind"})
        elif kind == "s":
            rows.append((float(obj.get("t", 0.0)), obj.get("z") or {}))
        elif kind == "end":
            complete = True
    return {"present": True, "header": header, "rows": rows, "complete": complete}


def runaway_verdict(header: dict, rows: list, *,
                    z_limit: float = RUNAWAY_Z_LIMIT_M,
                    fall_margin: float = FALL_MARGIN_M,
                    vz_min: float = RUNAWAY_VZ_MIN_MS,
                    window: int = RUNAWAY_WINDOW) -> list[dict]:
    """Bodies whose END STATE is physically absurd. Pure function.

    Two independent criteria, either of which fails a body:

    * ``escaped``  -- |z| beyond `z_limit`. No world in this tree legitimately
      places a body a kilometre off the origin vertically.
    * ``unbounded_fall`` -- the body ended BELOW the lowest static collision
      surface (minus `fall_margin`) while still accelerating downward faster
      than `vz_min`. That is a body in free fall with nothing under it, which is
      what a missing `boundingObject` looks like from the outside. Requiring
      *accelerating* (each of the last `window` vz samples more negative than
      the one before) is what keeps a body that fell and LANDED, or one riding a
      lift/elevator, or a legitimately descending drone, from tripping it.

    Returns [] when there is nothing to say -- including when there are too few
    samples to compute a velocity, because a guess is worse than silence.
    """
    if len(rows) < window + 2:
        return []
    floor_z = header.get("static_floor_z")
    floor_z = float(floor_z) if isinstance(floor_z, (int, float)) else None

    keys: list[str] = []
    for _t, row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)

    # The floor reference is bounded from ABOVE by the world's own initial
    # layout. A supervisor cannot see inside a PROTO, so `static_floor_z` is the
    # lowest static collider it COULD see -- measured on warehouse_husky.omniworld that
    # is 0.345 while the real `Floor {}` is at 0.0, i.e. above where the robot
    # legitimately sits. Taking the lower of (detected static surface, lowest
    # body start height) keeps an unlucky detection from pushing the bar up into
    # legal territory.
    starts = [row[k][2] for _t, row in rows[:1] for k in row if len(row[k]) == 3]
    lowest_start = min(starts) if starts else None

    offenders: list[dict] = []
    for key in keys:
        series = [(t, row[key][2]) for t, row in rows if key in row and len(row[key]) == 3]
        if len(series) < window + 2:
            continue
        t_final, z_final = series[-1]
        z0 = series[0][1]
        vz: list[float] = []
        for i in range(1, len(series)):
            dt = series[i][0] - series[i - 1][0]
            if dt <= 0:
                continue
            vz.append((series[i][1] - series[i - 1][1]) / dt)
        if len(vz) < window:
            continue
        tail = vz[-window:]
        accelerating_down = all(b < a for a, b in zip(tail, tail[1:])) and tail[-1] < -vz_min

        if floor_z is not None:
            base = floor_z if lowest_start is None else min(floor_z, lowest_start)
            ref = base - fall_margin
            ref_source = (f"lowest of detected static surface z={floor_z:.3f} and "
                          f"lowest body start z={base:.3f}, minus {fall_margin:g} m")
        else:
            ref = z0 - NO_FLOOR_MARGIN_M
            ref_source = (f"no static collider visible in the world; the body's own "
                          f"start z={z0:.3f} minus {NO_FLOOR_MARGIN_M:g} m")

        reasons = []
        if abs(z_final) > z_limit:
            reasons.append(f"|z|={abs(z_final):.1f} m exceeds the {z_limit:g} m bound")
        if z_final < ref and accelerating_down:
            reasons.append(
                f"ended at z={z_final:.2f} m, below {ref:.2f} m ({ref_source}), "
                f"still accelerating downward (vz={tail[-1]:.1f} m/s and falling)")
        if not reasons:
            continue
        offenders.append({
            "body": key,
            "z_final": z_final,
            "z_start": z0,
            "z_min": min(z for _t, z in series),
            "vz_final": tail[-1],
            "t_final": t_final,
            "samples": len(series),
            "reasons": reasons,
        })
    return offenders


def runaway_coverage_gap(header: dict, rows: list, *,
                         window: int = RUNAWAY_WINDOW) -> str | None:
    """Why the sampled evidence cannot certify ANYTHING -- or None if it can.

    `runaway_verdict` is deliberately silent when it has too little to work
    with (a guess is worse than silence, and
    tests/harness/test_headless_verdicts.py enshrines that). The bug this
    function closes is on the CALLER side: an empty verdict list was read as
    "nothing left the world", which is the exact evidence-free PASS
    `--fail-on-runaway` exists to prevent. Two ways the lane used to hand one
    out, both measured:

    * NO BODY WAS TRACKED. `runaway_watchdog._top_level_bodies` is shallow by
      design (it now covers root children plus one level of Group/Transform/Pose
      descent), so a world whose dynamic bodies sit deeper -- or inside a PROTO,
      which no supervisor can see into -- still reports `bodies: []`. Every
      sample row is then `{"kind":"s","z":{}}`: non-empty rows, so the runner's
      `if not rows` guard misses, and the verdict is `[]` no matter what the
      physics did.
    * TOO FEW SAMPLES. Fewer than `window + 2` rows means no body can produce a
      velocity tail. Measured on the C2 fall-through tail: the SAME body 59 km
      below the floor and still accelerating yields `[]` at 4 samples and FAILs
      at 5.

    Pure function; the message is the FAIL reason the runner prints.
    """
    bodies = header.get("bodies")
    if not bodies:
        return ("the watchdog tracked ZERO dynamic bodies, so every sample row is "
                "empty and no verdict is possible -- an untracked world and a "
                "healthy world produce byte-identical evidence")
    if len(rows) < window + 2:
        return (f"only {len(rows)} pose sample(s) were collected; at least "
                f"{window + 2} are needed before any body can produce a velocity "
                f"tail, so no verdict is possible")
    keys = {k for _t, row in rows for k in row}
    judged = [b for b in bodies
              if str(b.get("key")) in keys
              and sum(1 for _t, row in rows if str(b.get("key")) in row) >= window + 2]
    if not judged:
        return (f"none of the {len(bodies)} tracked bodies produced {window + 2} "
                "usable pose samples, so no verdict is possible")
    return None


RUNAWAY_SIBLING_PREFIX = ".omnisim_runaway_"

# ── PERSPECTIVE SIBLINGS OF A TEMPORARY WORLD (dual-read, single-write) ──────
#
# The engine stores a world's UI state (viewpoint, dock layout, open editor
# tabs) as a HIDDEN sibling named after the world's stem:
#
#     worlds/my_world.omniworld  ->  worlds/.my_world.omniperspective
#
# Since the Webots -> OmniSim migration it WRITES ".omniperspective" and READS
# both that and the legacy ".wbproj". The policy is stated once, for the engine,
# in src/omnisim/core/OmPerspectiveFileFormat.hpp (writeExtension /
# legacyExtension / readExtensions); this tuple is its Python mirror, in the
# same preferred-first order.
#
# It matters HERE because --fail-on-runaway CREATES a temporary world in the
# user's own world directory and must delete everything the engine may have
# written beside it. The cleanup used to name ".wbproj" alone, which was correct
# until the migration and silently stops being correct the moment the engine
# saves a perspective: the ".omniperspective" would be left behind as litter in
# a tracked directory. Neither file has to exist -- both are deleted if present.
PERSPECTIVE_EXTENSIONS = (".omniperspective", ".wbproj")


def perspective_siblings(world: Path) -> tuple[Path, ...]:
    """Every perspective file the engine could have written beside ``world``.

    Mirrors OmPerspective's own naming (``absolutePath + "/." +
    completeBaseName + <ext>``, OmPerspective.cpp:43) once per accepted
    extension. ``Path.stem`` is the exact counterpart of Qt's
    ``completeBaseName()`` here, including for a name that already starts with a
    dot -- ``.omnisim_runaway_foo.omniworld`` yields
    ``..omnisim_runaway_foo.omniperspective``, which is what the engine writes.
    """
    return tuple(world.with_name(f".{world.stem}{ext}") for ext in PERSPECTIVE_EXTENSIONS)

_RUNAWAY_STANZA = """
# --- appended by run-headless --fail-on-runaway (deleted with this file) -----
# synchronization FALSE for the same reason the harness's injected supervisor
# uses it: a SYNCHRONIZED helper makes the engine WAIT for it, so any slow
# moment in the helper stalls the world under test. Measured on
# distribution/generated_worlds/mars.wbt: with synchronization TRUE the engine
# emitted ZERO physics steps in a 30 s run while the watchdog was still walking
# the scene. Unsynchronised sampling gives uneven sample spacing, which costs
# nothing here -- every velocity is computed from the sample's own timestamps.
Robot {
  name "runaway_watchdog"
  controller "runaway_watchdog"
  supervisor TRUE
  synchronization FALSE
  controllerArgs [
    "--out=%s"
    "--period-steps=%d"
  ]
}
"""


def write_runaway_sibling(world: Path, out_path: Path, period_steps: int = 8) -> Path:
    """Write ``<worlddir>/.omnisim_runaway_<stem>.wbt`` = world + watchdog.

    A SIBLING, in the world's own directory: `URDFRobot { url ... }` and
    relative texture/PROTO paths resolve against the world file, so a copy
    anywhere else silently loses references.
    """
    # Inherit the parent world's extension: a `.omniworld` world needs a
    # `.omniworld` sibling or the engine will not treat it as a world at all.
    sibling = world.with_name(f"{RUNAWAY_SIBLING_PREFIX}{world.stem}{world.suffix}")
    text = world.read_text(errors="replace")
    if not text.endswith("\n"):
        text += "\n"
    posix_out = str(out_path).replace("\\", "/")
    sibling.write_text(text + _RUNAWAY_STANZA % (posix_out, int(period_steps)))
    return sibling


_LOG_HEADER_PID_RE = re.compile(r"=== OmniSim Log Started \(pid=(\d+)\)")


def connect_sidecar_failures(log_text: str, log_path: Path) -> list[str]:
    """Return this run's entries from the append-only connect_error sidecar.

    libController appends to "<log>.connect_error.txt" when it cannot open its
    IPC pipe or fails the IPC handshake (failures that predate its stderr
    redirection, so they are invisible in the engine log). Entries are tagged
    run=<engine pid>; we only report the ones matching THIS run's log header --
    stale entries from earlier runs are someone else's history.
    """
    header = _LOG_HEADER_PID_RE.search(log_text)
    if header is None:
        return []
    run_tag = f"run={header.group(1)}:"
    sidecar = log_path.with_name(log_path.name + ".connect_error.txt")
    if not sidecar.exists():
        return []
    try:
        lines = sidecar.read_text(errors="replace").splitlines()
    except OSError:
        return []
    return [line for line in lines if line.startswith(run_tag)]


def sim_never_stepped(log_text: str, sidecar_finalised: bool) -> bool:
    """True IFF the world provably finalised under Newton but never stepped once.

    Fires only on POSITIVE evidence of finalize (sidecar finalised, or the
    "world finalised" log line) combined with ZERO "[OmNewtonBackend] step N"
    lines. When finalize was NOT proven -- an ODE run we cannot measure this
    way, or a cold load that never reached finalize inside the duration -- this
    returns False on purpose: a false FAIL is worse than the status quo, so we
    never guess.
    """
    finalised = bool(sidecar_finalised) or (_NEWTON_FINALISED_RE.search(log_text) is not None)
    if not finalised:
        return False
    return _NEWTON_STEP_RE.search(log_text) is None


def sidecar_verdict(sidecar_path: Path) -> dict:
    """The Newton verdict sidecar as a dict, or {} for anything unreadable.

    {backend, degraded, finalised, runtime{device, newton, warp, mujoco,
    mujoco_warp}, solver}, written by OmNewtonBackend::writeNewtonVerdictSidecar
    AT finalize. Every error is swallowed: a diagnostic that crashes while
    reporting a fault is worse than none.

    ⚠ It is written a few milliseconds AFTER the "world finalised" log line
    (OmNewtonBackend.cpp:2323 emits the line, :2341 writes the file), so a
    caller that reacts to the log line must tolerate {} once and re-read.
    """
    try:
        data = json.loads(sidecar_path.read_text(errors="replace"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def sidecar_device(sidecar_path: Path) -> str | None:
    """runtime.device from the verdict sidecar ("cpu" / "cuda:0"), else None.

    This is the honest answer to "was this run on the GPU path?" -- it reports
    the device the model actually lived on, which the world file cannot (env
    vars and defaults override WorldInfo.newtonSolver).
    """
    runtime = sidecar_verdict(sidecar_path).get("runtime")
    if isinstance(runtime, dict):
        device = runtime.get("device")
        if isinstance(device, str) and device.strip():
            return device.strip()
    return None


def _sidecar_finalised(sidecar_path: Path) -> bool:
    """Read finalised=true from the Newton verdict sidecar, tolerating anything.

    Its mere presence already means "Newton drove THIS run" (OmLog deletes a
    stale copy at startup), but we parse the flag explicitly.
    """
    return bool(sidecar_verdict(sidecar_path).get("finalised"))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a world headlessly and report results")
    parser.add_argument("world", help="Path to .wbt world file")
    parser.add_argument("--duration", type=int, default=None,
                        help="Seconds to run before stopping. OMIT IT and the run is treated "
                             "as a load check: it stops as soon as Newton finalises the world "
                             f"(see --until-finalized), with {DEFAULT_DURATION_S}s as the "
                             "ceiling. Pass an "
                             "explicit value when the run must OBSERVE the simulation for that "
                             "long -- an explicit value is always honoured verbatim.")
    parser.add_argument("--wait-for-step", action="store_true",
                        help="Start the --duration clock only after the engine emits its first "
                             "Newton physics step. Use for cold policy worlds so load time does "
                             "not consume the requested simulation observation window.")
    parser.add_argument("--startup-timeout", type=int, default=180,
                        help="Wall-clock seconds allowed to reach the first Newton step when "
                             "--wait-for-step is set (default: 180)")
    parser.add_argument("--completion-log",
                        help="Optional log file to watch for an outcome verdict")
    parser.add_argument("--completion-pattern",
                        help="Python regular expression that ends the run when it appears in "
                             "--completion-log")
    parser.add_argument("--profile", action="store_true",
                        help="Ask the engine where the load went and print it after the run: sets "
                             "OMNISIM_RELOAD_PROFILE=1 (stage split, PROTO phases, template-engine split), "
                             "OMNISIM_NEWTON_PRELOAD_PROFILE=1 (the runtime preload and how long the main "
                             "thread blocked on it) and OMNISIM_LOG_TIMESTAMPS=1 (every log line carries "
                             "[t+<ms>ms]), then echoes the engine's [runtime-cycle] lines. Zero engine cost "
                             "without the flag.")
    parser.add_argument("--settle-after-step", type=float, default=0.5,
                        help="--until-finalized only: seconds to keep watching the log AFTER the first "
                             "physics step has been observed, then stop. The run used to sit out the full "
                             "--completion-grace (2 s) after finalize even when the step had landed "
                             "milliseconds later -- pure sleep on every load check (measured: the engine "
                             "reaches step 1 ~50 ms after finalize on a CPU world, then the runner waited "
                             "~2 s doing nothing). 0.5 s is enough for a controller that crashes at import "
                             "to be reported. Pass a larger value to watch longer; --completion-grace still "
                             "bounds the no-step case.")
    parser.add_argument("--completion-grace", type=float, default=2.0,
                        help="Seconds to keep stepping after the completion pattern appears "
                             "before shutdown (default: 2)")
    parser.add_argument("--until-finalized", action="store_true",
                        help="End the run as soon as Newton finalises the world, instead of "
                             "sleeping out --duration. Equivalent to pointing "
                             "--completion-log/--completion-pattern at this run's own engine "
                             "log and its 'world finalised' line, but without having to know "
                             "either. Use for 'does this world still load?' checks -- the most "
                             "frequent check there is: measured 4.0 s to finalise a cloth world "
                             "and 5.4 s an 8-Husky swarm, against the >=15 s (>=45 s cold/"
                             "virtualised) that guessing a --duration costs. It stops at "
                             "finalize AND the first physics step -- a world that finalised "
                             "and never stepped is not a proven load, and the wait for that "
                             "step is sized from the engine's own reported device (see "
                             "--step-wait-timeout), so the GPU 'mujoco_warp' path is not cut "
                             "off mid-kernel-compilation. This proves LOAD "
                             "and FINALIZE only; a run that must observe behaviour still needs "
                             "--duration, and a claim about physics wants --fail-on-runaway.")
    parser.add_argument("--step-wait-timeout", type=float, default=None, metavar="S",
                        help="Seconds --until-finalized may wait for the FIRST physics step "
                             "after the world finalises, overriding the device-derived budget "
                             f"({STEP_WAIT_CPU_S:g}s when the verdict sidecar reports device "
                             f"'cpu', {STEP_WAIT_GPU_S:g}s when it reports a CUDA device). The "
                             "wait ends the instant the step appears, so it costs a healthy run "
                             "nothing; it exists because warp compiles its CUDA kernels AFTER "
                             "finalize with no log output -- measured 1.33 s (warm cache) and "
                             "95.42 s (cold) on machine 9722d23d12a3 -- so a fixed short grace "
                             "ends a mujoco_warp run mid-compilation and reports the zero steps "
                             "as a failure. 0 restores the old fixed-grace behaviour exactly.")
    parser.add_argument("--fail-on-warning", action="store_true", help="Treat warnings as failures")
    parser.add_argument("--fail-on-runaway", action="store_true",
                        help="Also check that the END STATE is physically possible: inject the "
                             "runaway_watchdog supervisor into a sibling copy of the world, sample "
                             "every top-level dynamic body's pose, and FAIL if one left the world "
                             "(|z| past --runaway-z-limit, or below the lowest static collision "
                             "surface and still accelerating downward -- the signature of a missing "
                             "boundingObject). ALSO FAILS when the evidence cannot support a "
                             "verdict at all (no dynamic body tracked, or too few samples to "
                             "compute a velocity), because a silent verdict is not a PASS. Off by "
                             "default because it injects a Robot into a world copy; on for any run "
                             "whose PURPOSE is to certify a physics fix.")
    parser.add_argument("--runaway-z-limit", type=float, default=RUNAWAY_Z_LIMIT_M,
                        help=f"|z| bound for --fail-on-runaway (default {RUNAWAY_Z_LIMIT_M:g} m)")
    parser.add_argument("--runaway-period-steps", type=int, default=8,
                        help="Sample the watchdog every N basic timesteps (default 8)")
    parser.add_argument("--no-window", action="store_true",
                        help="Run the engine in no-window headless mode (OMNISIM_NO_WINDOW=1): zero GUI "
                             "construction, ~20%% less RSS per instance at full functional parity "
                             "(core-evolution-plan.md, Phase Q1). Opt-in while the mode soaks.")
    parser.add_argument("--no-gl", action="store_true",
                        help="Run the engine in COMPUTE-ONLY headless mode (OMNISIM_NO_GL=1): no window, no GL "
                             "context, no WREN -- physics/controllers/IPC only, ~35%% less RSS. A controller "
                             "enabling a camera/range-finder/lidar is a FATAL, attributed error (vision worlds "
                             "must use --no-window instead). WARNING: EXPERIMENTAL (Phase Q1 Tier C spike): "
                             "the world finalises but does NOT reliably step on all backends yet -- do "
                             "not use it for demos or timing. Prefer --no-window for a working headless run.")
    parser.add_argument("--require-newton", action="store_true",
                        help="Set OMNISIM_REQUIRE_NEWTON=1 so the engine FAILS LOUDLY (non-zero exit) "
                             "if the Newton runtime can't initialise, instead of silently falling back "
                             "to ODE. Use for Newton deploy/CI runs that must assert Newton is active.")
    parser.add_argument("--cold", action="store_true",
                        help="Disable the one-time warm-up reload and test the RAW "
                             "cold first-load (which can behave differently from a "
                             "stabilised/post-reload session). See the cold-start "
                             "note printed at startup.")
    parser.add_argument("--performance-log", default="", help="Path to write performance log")
    parser.add_argument("--gui", action="store_true",
                        help="Open a REAL window (drop --minimize/--batch/--no-rendering) so "
                             "you can WATCH the run, while keeping the same reliable env "
                             "propagation (os.environ.copy -> Popen) the headless path uses. "
                             "Use to demo a Newton policy on the engine it was trained on.")
    parser.add_argument("--realtime", action="store_true",
                        help="Pace the simulation at 1.0x wall clock (--mode=realtime) "
                             "instead of as-fast-as-possible. Use with --gui so motion "
                             "plays at true speed now that Newton runs faster than "
                             "realtime.")
    parser.add_argument("--port", type=int, default=0,
                        help="Bind the extern-controller / robot-window server to this "
                             "port (passed through as --port=N). REQUIRED to run two "
                             "instances at once: a second instance otherwise collides "
                             "on the default 1234, falls back to 1235, and the GUI "
                             "tears down. 0 (default) leaves the port unset.")
    parser.add_argument("--race-attempts", type=int, default=DEFAULT_RACE_ATTEMPTS,
                        metavar="N",
                        help="Total launch attempts allowed when a launch dies in the KNOWN "
                             f"ENGINE STARTUP RACE (default {DEFAULT_RACE_ATTEMPTS}). A retry "
                             "is taken ONLY when the engine exited early AND the log carries "
                             "the Qt teardown signature AND there is NOT ONE ERROR/FATAL line "
                             "-- i.e. only when nothing was learned about the world. Every "
                             "other failure is a verdict: returned immediately, never retried. "
                             "Set 1 to disable retrying. If EVERY attempt races, the exit code "
                             f"is {EXIT_STARTUP_RACE} (EX_TEMPFAIL), not 1.")
    return parser


# The wall-clock ceiling applied when the caller expressed no opinion. It is
# only ever a CEILING: --until-finalized returns the moment Newton finalises,
# so a larger number costs nothing on a world that loads and buys correctness
# on one that is merely slow.
#
# It was 10 s, inherited from the old sleep-based default, and 10 s is right at
# the cold-load cost of the demo README leads with. MEASURED on machine
# 9722d23d12a3: omniarm6_real_pick_place finalises at 9.43 s WARM and misses
# the ceiling COLD -- and a miss is not a quiet one. It prints a five-line
# "OBSERVED / NOT DIAGNOSED" block explaining that the evidence is equally
# consistent with a world declaring no physics bodies, and then says PASS with
# no sidecar. So the documented load check was flaky on the documented demo,
# and its failure mode was a wall of text about a defect that was not there.
# 30 s clears every world measured here with margin.
DEFAULT_DURATION_S = 30


def _resolve_duration_default(args) -> None:
    """No --duration given => this is a LOAD CHECK, so stop at finalize.

    --duration is a wall-clock sleep, not a progress target. The old default of
    10 s was never a considered number for any particular world; it was just a
    number. Meanwhile the thing a bare `run-headless <world>` is nearly always
    asking -- "does this still load?" -- is answered the moment Newton
    finalises and writes its sidecar, MEASURED at 3985/4088 ms (cloth) and
    5419 ms (8-Husky) on machine 9722d23d12a3.

    So: an EXPLICIT --duration is honoured exactly as before (the caller means
    those seconds, and a behaviour run must keep them). Its ABSENCE now selects
    --until-finalized with the old 10 s as a ceiling.

    ⚠ This is announced on stdout rather than done silently, deliberately. It
    narrows the window in which a LATE error (a controller dying at t=8 s) can
    be counted into `errors`/`warnings`, and a run whose claim is behavioural
    must not inherit that quietly. Flags that mean "I am observing something"
    -- --fail-on-runaway, an explicit --completion-pattern, --wait-for-step --
    suppress the switch entirely.
    """
    if args.duration is not None:
        return
    args.duration = DEFAULT_DURATION_S
    observational = (args.fail_on_runaway or args.completion_pattern
                     or args.wait_for_step)
    if observational or args.until_finalized:
        return
    args.until_finalized = True
    print(f"[headless] no --duration given: treating this as a LOAD CHECK and "
          f"stopping at Newton finalize (ceiling {DEFAULT_DURATION_S}s). "
          f"Pass --duration N to observe behaviour for N seconds instead.")


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    _resolve_duration_default(args)
    if bool(args.completion_log) != bool(args.completion_pattern):
        parser.error("--completion-log and --completion-pattern must be supplied together")
    if args.completion_pattern:
        try:
            re.compile(args.completion_pattern)
        except re.error as exc:
            parser.error(f"invalid --completion-pattern: {exc}")
    # Validated ONCE, here, rather than inside the attempt: parser.error()
    # exits the process, and a flag conflict is not something a retry can fix.
    if args.no_window and args.gui:
        parser.error("--no-window and --gui are mutually exclusive")
    if args.no_gl and args.gui:
        parser.error("--no-gl and --gui are mutually exclusive")

    # ── THE RETRY LOOP ───────────────────────────────────────────────────────
    # Retries are taken ONLY on EXIT_STARTUP_RACE, which run_once returns only
    # when the engine exited early carrying the Qt teardown signature with NOT
    # ONE ERROR/FATAL line (`looks_like_startup_race`, the single definition of
    # the rule). Every other outcome -- PASS, a world that failed to load, a
    # dead controller, a sim that never stepped, a runaway body -- is a verdict
    # and is returned on the spot. Retrying any of those would launder a
    # result, which is the one thing this loop must never do.
    attempts = max(1, int(args.race_attempts))
    raced = 0
    for attempt in range(1, attempts + 1):
        if attempts > 1:
            print(f"[headless] === launch attempt {attempt}/{attempts} ===")
        rc = run_once(args)
        if rc != EXIT_STARTUP_RACE:
            if raced:
                print(f"[headless] RECOVERED: {raced} attempt(s) hit the known engine "
                      f"startup race; attempt {attempt} launched cleanly.")
            return rc
        raced += 1
        if attempt < attempts:
            settle = RACE_SETTLE_S * attempt
            print(f"[headless] STARTUP RACE on attempt {attempt}/{attempts} -- a launch "
                  f"flake, not a verdict about the world.")
            print(f"[headless]   Settling {settle:g}s (it is a BACK-TO-BACK-launch race, "
                  f"so the pause is the active ingredient) and retrying.")
            time.sleep(settle)

    print(f"[headless] FAIL: all {attempts} launch attempts died in the KNOWN ENGINE "
          f"STARTUP RACE.")
    print("[headless]   Every attempt showed the Qt teardown signature and NOT ONE ERROR "
          "line, so this says NOTHING about the world: it is")
    print("[headless]   not a load failure, not a physics failure and not a flag problem. "
          "Do not go looking for a defect in the world.")
    print("[headless]   Raise --race-attempts, and leave a few seconds between launches "
          "(the race is worst back-to-back).")
    print(f"[headless]   Exit code {EXIT_STARTUP_RACE} (EX_TEMPFAIL) marks this "
          "'retry later', deliberately distinct from a real FAIL (1).")
    return EXIT_STARTUP_RACE


def run_once(args) -> int:
    """One launch and its verdict.

    Returns EXIT_STARTUP_RACE -- and ONLY then -- when the launch died in the
    known engine startup race, i.e. when this attempt is not evidence of
    anything and the caller may discard it. Every other return value is a
    verdict about the world.
    """
    completion_re = None
    completion_path = None
    if args.completion_pattern:
        completion_re = re.compile(args.completion_pattern)
        completion_path = Path(args.completion_log).resolve()

    # OMNISIM_HOME is canonical. WEBOTS_HOME (the upstream name) is honoured
    # only when it actually points at an OmniSim checkout: many machines
    # carry a stale system WEBOTS_HOME from an old Webots install, and
    # silently resolving the binary there ran the wrong simulator.
    home_env = os.environ.get("OMNISIM_HOME")
    if not home_env:
        legacy = os.environ.get("WEBOTS_HOME")
        if legacy and (Path(legacy) / "omnisim" / "cli.py").exists():  # an OmniSim checkout, not upstream Webots
            home_env = legacy
        else:
            if legacy:
                print(f"[headless] note: ignoring WEBOTS_HOME={legacy} "
                      f"(not an OmniSim checkout); using this repo root.",
                      file=sys.stderr)
            home_env = str(REPO_ROOT)
    omnisim_home = Path(home_env)
    binary = find_binary(omnisim_home)
    world = Path(args.world)
    if not world.is_absolute():
        world = REPO_ROOT / world
    if not world.exists():
        # DUAL-READ (AGENTS.md): OmniSim reads .omniworld and .wbt
        # interchangeably and indefinitely -- the tree migrated 661 of its own
        # worlds, so every older tutorial, script and bookmark names a .wbt that
        # is now a .omniworld. The engine honours that; the Python side did not,
        # and a bare exists() turned a renamed file into a hard "World not
        # found". docker/Dockerfile.train shipped exactly that bug for months.
        _twin = {".wbt": ".omniworld", ".omniworld": ".wbt"}.get(world.suffix)
        if _twin and world.with_suffix(_twin).exists():
            world = world.with_suffix(_twin)
        else:
            raise SystemExit(f"World not found: {world}")

    # Use OMNISIM_LOG_PATH if set (per the parallel-runs convention in
    # AGENTS.md §3e); otherwise default to omnisim_log.txt at OMNISIM_HOME.
    env_log = os.environ.get("OMNISIM_LOG_PATH")
    log_path = Path(env_log) if env_log else omnisim_home / "omnisim_log.txt"
    log_path, log_note = writable_log_path(log_path)
    if log_note:
        print(f"[headless] NOTE: {log_note}")

    # --until-finalized: stop as soon as the world has PROVEN it loads.
    #
    # --duration is a wall-clock sleep, not a progress target, so a load check
    # costs whatever number was guessed -- and because a run that ends before
    # Newton finalises writes no verdict sidecar, the guidance is to guess HIGH
    # (AGENTS.md: >=15 s, >=45 s on cold or virtualised disks). MEASURED on
    # machine 9722d23d12a3 the wait is nothing like that: the cloth world
    # reaches "world finalised" in 3985/4088 ms and an 8-Husky swarm in 5419 ms.
    # Everything past that is sleep, on the single most frequent check in the
    # tree.
    #
    # The generic --completion-log/--completion-pattern pair could already
    # express this; nobody should have to know the incantation, or which log
    # file this run happens to be writing to. Default them here, AFTER
    # log_path is resolved, and only when the caller has not asked for a
    # different verdict of their own.
    #
    # ⚠ SCOPE: this proves load + finalize, NOT behaviour. A run whose point is
    # to observe the simulation still needs steps -- keep --duration for those,
    # and prefer --fail-on-runaway when the claim is about physics.
    # Reuse _NEWTON_FINALISED_RE rather than spelling the pattern again: it
    # already accepts both the [WbNewtonBackend] and [OmNewtonBackend] spellings
    # of the tag, and a second copy would rot on one side of that rename.
    until_finalized_mode = False
    if getattr(args, "until_finalized", False) and not args.completion_pattern:
        completion_re = _NEWTON_FINALISED_RE
        completion_path = log_path.resolve()
        until_finalized_mode = True
    # The engine's verdict sidecar for THIS run, polled live below to learn the
    # model device (and read again at verdict time for solver + device).
    sidecar_path = log_path.with_name(log_path.name + ".newton.json")

    # Clear old log. Tolerate WinError 32 (file locked by another process —
    # typically the user's interactive OmniSim holding the default log open):
    # in that case we honor OMNISIM_LOG_PATH for our own child and skip the
    # default-path cleanup.
    if log_path.exists():
        try:
            log_path.unlink()
        except PermissionError:
            print(f"[headless] {log_path} is locked by another process — "
                  f"set OMNISIM_LOG_PATH to a unique path for this run.",
                  file=sys.stderr)
            return 2

    # ── --fail-on-runaway: inject the pose sampler into a SIBLING world ──────
    # The engine log records nothing about a body leaving the world, so the only
    # honest way to know is to sample poses. The sibling lives in the world's own
    # directory (relative URDF / PROTO / texture references resolve against the
    # world file) and is deleted at exit.
    launch_world = world
    runaway_out = log_path.with_name(log_path.name + ".runaway.jsonl")
    if args.fail_on_runaway:
        try:
            runaway_out.unlink()
        except OSError:
            pass
        try:
            launch_world = write_runaway_sibling(world, runaway_out,
                                                 args.runaway_period_steps)
        except OSError as exc:
            print(f"[headless] FAIL: --fail-on-runaway could not write the "
                  f"instrumented sibling world next to {world}: {exc}",
                  file=sys.stderr)
            return 2
        sibling = launch_world

        # The temp world AND every perspective file the engine may have written
        # beside it -- both extensions, per the dual-read policy above.
        def _drop_sibling(paths=(sibling,) + perspective_siblings(sibling)):
            for p in paths:
                try:
                    p.unlink()
                except OSError:
                    pass
        atexit.register(_drop_sibling)
        print(f"[headless] RUNAWAY CHECK: sampling every top-level dynamic body "
              f"every {args.runaway_period_steps} basic steps via {launch_world.name}")

    # `--minimize` keeps the main window in a normal Qt event loop while
    # hiding it via the OS taskbar minimize. The alternative `--no-window`
    # mode skips main-window realization entirely. This comment used to say
    # that mode "deadlocks Newton's embedded CPython FFI at the first few
    # add_joint_revolute calls" (G1 + 20-husky, engine-migration-plan.md §15,
    # 2026-05-28) -- STALE, public issue #5: that was the XPBD-era engine. The
    # Linux CI smokes have run under OMNISIM_NO_WINDOW=1 since the 22.04 work,
    # and on 2026-08-29 (machine 9722d23d12a3, Windows) the 8-Husky swarm
    # (40 dynamic bodies, 9 controllers, --duration 12) and the G1 humanoid
    # (--until-finalized) both finalised and stepped under --no-window with
    # the same finalize/step lines as the --minimize control. --minimize stays
    # the default only because it is the longer-trodden path on desktops;
    # --no-window is the right call for containers and GPU-less hosts.
    # `--batch --no-rendering` keep the run cheap either way.
    mode = "--mode=realtime" if args.realtime else "--mode=fast"
    if args.gui:
        # Visible window: drop --minimize/--batch/--no-rendering so the 3D view
        # realises. (This line used to add "only --no-window deadlocks it" -- stale, see above.)
        cmd = [str(binary), str(launch_world), mode, "--stdout", "--stderr"]
    else:
        cmd = [
            str(binary),
            str(launch_world),
            "--minimize",
            "--batch",
            "--no-rendering",
            mode,
            "--stdout",
            "--stderr",
        ]
    if args.port:
        cmd.append(f"--port={args.port}")
    if args.performance_log:
        cmd.append(f"--log-performance={args.performance_log}")

    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(omnisim_home)
    # The GUI/render path resolves Qt plugins + render resources via WEBOTS_HOME;
    # without it a --gui launch crashes during render init (Qt teardown) even
    # though the headless --no-rendering path is fine. Point it at THIS checkout
    # (also overrides any stale system WEBOTS_HOME from an old Webots install).
    env["WEBOTS_HOME"] = str(omnisim_home)
    env["OMNISIM_LOG_PATH"] = str(log_path)
    if args.profile:
        env["OMNISIM_RELOAD_PROFILE"] = "1"
        env["OMNISIM_NEWTON_PRELOAD_PROFILE"] = "1"
        env["OMNISIM_LOG_TIMESTAMPS"] = "1"
    if args.require_newton:
        # Assert Newton actually initialises -- the engine OmLog::fatal()s (non-zero
        # exit) instead of silently degrading to ODE. Guards against the silent-ODE-
        # fallback class of bug (e.g. the warp-banner-vs-DEVNULL FFI-smoke failure).
        env["OMNISIM_REQUIRE_NEWTON"] = "1"
        print("[headless] REQUIRE-NEWTON: engine will fail loudly if Newton can't init "
              "(no silent ODE fallback).")
    # (--no-window/--no-gl vs --gui conflicts are rejected once, in main().)
    if args.no_window:
        env["OMNISIM_NO_WINDOW"] = "1"
        print("[headless] NO-WINDOW: zero GUI construction (no main view; camera devices "
              "still render offscreen through wgpu; ~20% less RSS).")
    if args.no_gl:
        env["OMNISIM_NO_GL"] = "1"
        print("[headless] NO-GL: compute-only (no window, no GL, no WREN; ~35% less RSS). "
              "Enabling a vision device is a fatal error in this mode. "
              "⚠️ EXPERIMENTAL: does not reliably STEP yet -- use --no-window for a working run.")

    # ── COLD-START TRAP (RESOLVED) ────────────────────────────────────────────
    # Historically a fresh cold first-load under-tracked the Newton/MuJoCo articulation
    # (~1 cm), so precise grasps failed cold but worked after a reload -- headless runs
    # therefore defaulted to a warm-up reload. That bug is FIXED (verified 2026-07-05:
    # cold == warm bit-identical; root fix eb86f888 + finalize-time solver re-assert;
    # see docs/developer/real-grasp-and-the-cold-first-load-trap.md), so warmup_reload
    # is now a no-op by default and there is nothing to warm. --cold is kept for
    # explicitness (forces the workaround fully off even if someone set FORCE_WARMUP).
    if args.cold:
        env["OMNISIM_NO_WARMUP"] = "1"
        print("[headless] COLD MODE: warm-up reload force-disabled (the cold-load "
              "under-track is fixed, so this matches the default behaviour).")
    if sys.platform == "win32":
        # Controllers (generic.exe et al.) are MinGW-built and load
        # libgcc_s_seh-1.dll / libstdc++-6.dll / libwinpthread-1.dll at launch.
        # Those live in msys64\mingw64\bin, so that dir MUST be on the child's
        # PATH or the controller dies with "DLL not found". binary.parent
        # already covers it when the canonical mingw64\bin binary resolves, but
        # inject it explicitly so a fallback binary path can't drop the runtime.
        mingw_bin = omnisim_home / "msys64" / "mingw64" / "bin"
        path_prefix = [str(binary.parent)]
        if mingw_bin.is_dir() and mingw_bin != binary.parent:
            path_prefix.append(str(mingw_bin))
        # "Those live in msys64\mingw64\bin" above was true of the DEV tree
        # only. The v8.1.5-v8.1.13 installers put libstdc++-6 / libgcc_s_seh-1 /
        # libwinpthread-1 under msys64\mingw64\bin\cpp and nothing but the exe
        # launcher (launcher.c) adds that directory to PATH -- so this runner,
        # reached from `omnisim.bat demo` through the conformance gate, launched
        # an engine that died in three "DLL not found" dialogs with no log
        # (public issue #9). The packager now ships copies beside the exe; this
        # covers the installs that already exist.
        mingw_cpp = mingw_bin / "cpp"
        if mingw_cpp.is_dir():
            path_prefix.append(str(mingw_cpp))
        # Release builds keep the bundled controller interpreter below the
        # engine directory.  Direct omnisim-bin launches bypass the launcher
        # that normally exposes it, so include it explicitly; otherwise every
        # Python controller fails with `"python.exe" was not found` even while
        # the engine's embedded Newton interpreter is healthy.
        runtime = mingw_bin / "newton-runtime"
        if (runtime / "python.exe").is_file():
            path_prefix.append(str(runtime))
            site_packages = runtime / "site-packages"
            if site_packages.is_dir():
                env["PYTHONPATH"] = str(site_packages) + ";" + env.get(
                    "PYTHONPATH", ""
                )
        env["PATH"] = ";".join(path_prefix) + ";" + env.get("PATH", "")
    # Linux: spawning omnisim-bin directly bypasses the `webots` launcher
    # shell, so supply the runtime env it would otherwise miss (bundled-Qt
    # LD_LIBRARY_PATH, QT_QPA_PLATFORM, the tmpdir, LIBGL_ALWAYS_SOFTWARE).
    # The tmpdir default lands on WEBOTS_TMPDIR (see omnisim/paths.py); the
    # engine reads OMNISIM_TMPDIR first and falls back to it, so an
    # OMNISIM_TMPDIR exported here still wins -- os.environ is copied above.
    # No-op on other platforms.
    env = linux_runtime_env(omnisim_home, env)

    print(f"[headless] Starting: {world.name}")
    print(f"[headless] Duration: {args.duration}s" +
          (f" after first physics step (startup timeout {args.startup_timeout}s)"
           if args.wait_for_step else ""))
    if completion_re is not None:
        # completion_pattern is None under --until-finalized (the regex is
        # supplied from _NEWTON_FINALISED_RE), so report the pattern actually
        # in force rather than printing "/None/".
        print(f"[headless] Completion: /{completion_re.pattern}/ in {completion_path} "
              f"(+{args.completion_grace:g}s grace)")
    print(f"[headless] Binary: {binary}")

    # stdout and stderr BOTH go to FILE sinks — never PIPE, never DEVNULL.
    #
    # - not PIPE: pipe buffers fill at ~64 KB on Windows and the simulator's
    #   writes then block, which manifested as the world load apparently
    #   stalling past 20 huskies (2026-05-28). A file sink has no such limit.
    # - not DEVNULL (2026-07-12): DEVNULL was the original fix for the stall,
    #   but the engine embeds CPython, and warp prints its banner from inside
    #   newton.ModelBuilder() during the Newton FFI smoke test. Against a
    #   DEVNULL stdout that print intermittently raises [Errno 9] Bad file
    #   descriptor -> the smoke test fails -> the engine SILENTLY FALLS BACK
    #   TO ODE. That is the "~25% launch flake": the run does not crash, it
    #   comes up on the wrong backend, so every mjw-dependent deploy policy
    #   (the whole RL/suction stack) is dead and the demo just stands there
    #   looking broken. Sidecar-verified: a flaked run writes NO
    #   <log>.newton.json at all. A real file has a real fd, so the banner
    #   write succeeds and the smoke test measures what it means to measure.
    #   stderr already had this treatment (an engine dying during
    #   embedded-CPython/Newton bring-up prints its reason ONLY there); the
    #   asymmetry was the bug.
    stdout_path = log_path.with_name(log_path.name + ".stdout")
    stderr_path = log_path.with_name(log_path.name + ".stderr")
    stdout_sink = open(stdout_path, "wb")
    stderr_sink = open(stderr_path, "wb")
    proc = subprocess.Popen(cmd, env=env, stdout=stdout_sink, stderr=stderr_sink)

    # Wait for the requested observation window.  Large Newton worlds can spend
    # most of a cold run compiling/finalising; --wait-for-step deliberately keeps
    # that startup budget separate from the useful post-step duration.
    start = time.time()
    run_start = None if args.wait_for_step else start
    startup_failed = False
    completion_at = None
    # --until-finalized state (see STEP_WAIT_CPU_S / STEP_WAIT_GPU_S above).
    # step_seen_at   when the engine logged its first physics step
    # device_seen    runtime.device from the verdict sidecar, once readable
    # warp_in_log    the engine itself named the mujoco_warp path (fallback hint)
    # step_deadline  when the post-finalize wait for that step runs out
    # warp_deadline  when the PRE-finalize extension (warp path only) runs out
    step_seen_at = None
    device_seen = None
    warp_in_log = False
    step_deadline = None
    warp_deadline = None
    step_wait_announced = False
    completion_text = ""
    # Incremental tailers for the 0.5 s poll loop: same text the old full
    # read_text() calls produced, without re-reading the whole file each poll.
    completion_tail = IncrementalLogText(completion_path) if completion_path else None
    log_tail = IncrementalLogText(log_path)
    while True:
        ret = proc.poll()
        if ret is not None:
            if ret != 0:
                print(f"[headless] FAIL: simulator exited early with code {ret}")
                # The cause of an early exit often lives only on stderr.
                try:
                    stderr_sink.close()
                    text = stderr_path.read_bytes()[-64_000:].decode(errors="replace").strip()
                    lines = text.splitlines()
                    if lines:
                        # A Rust panic prints its MESSAGE first and then ~40
                        # backtrace frames, so a last-15-lines tail kept frames
                        # 24-37 and threw away the one line that says why. That
                        # is exactly what happened to the Linux CI SIGABRT: the
                        # diagnostic was fifteen Qt symbols and no cause. Surface
                        # the panic line explicitly, then head AND tail.
                        panic = [ln for ln in lines
                                 if "panicked at" in ln or "non-unwinding panic" in ln]
                        if panic:
                            print("[headless]   RUST PANIC (the only Rust in the process is "
                                  "libwgpu_native.so): " + panic[0].strip())
                        print(f"[headless] engine stderr ({stderr_path}):")
                        head = lines[:8]
                        tail_lines = lines[-15:]
                        shown = head + (["   ... (%d lines elided) ..."
                                         % (len(lines) - len(head) - len(tail_lines))]
                                        if len(lines) > len(head) + len(tail_lines) else [])
                        shown += [ln for ln in tail_lines if ln not in head]                             if len(lines) <= len(head) + len(tail_lines) else tail_lines
                        for ln in shown:
                            print(f"[headless]   {ln}")
                except OSError:
                    pass
                # Attribute the known engine STARTUP RACE instead of letting the
                # reader hunt for a defect in their world / their CLI flags.
                try:
                    early_log = log_path.read_text(errors="replace") if log_path.exists() else ""
                except OSError:
                    early_log = ""
                # Name the cause from the LOG (the engine's stderr is discarded on
                # Windows, and the Qt platform-plugin abort lives only there).
                causes = early_exit_cause_lines(early_log)
                if causes:
                    print(f"[headless]   the engine log names the cause ({log_path}):")
                    for ln in causes:
                        print(f"[headless]     {ln}")
                elif not early_log.strip():
                    print("[headless]   the engine wrote NO log line at all before exiting.")
                for ln in platform_plugin_hint(early_log):
                    print(f"[headless]   {ln}")
                raced = looks_like_startup_race(early_log)
                if raced:
                    print("[headless]   KNOWN ENGINE STARTUP RACE, not a fault in this "
                          "world: the log carries the Qt teardown signature")
                    print("[headless]   ('QWaitCondition: Destroyed while threads are still "
                          "waiting') and NOT ONE ERROR line. AgentBench's adapter")
                    print("[headless]   measures this at ~1 launch in 3, most often "
                          "back-to-back. Nothing was learned about the world,")
                    print("[headless]   the flags or the ports -- so this attempt is "
                          "DISCARDED rather than reported as a verdict.")
                stdout_sink.close()
                if not stderr_sink.closed:
                    stderr_sink.close()
                return EXIT_STARTUP_RACE if raced else 1
            else:
                print(f"[headless] Simulator exited cleanly before timeout")
                break
        now = time.time()
        if completion_re is not None:
            completion_text = completion_tail.read()
            if until_finalized_mode and not warp_in_log:
                warp_in_log = _WARP_PATH_RE.search(completion_text) is not None
            if completion_re.search(completion_text):
                if completion_at is None:
                    completion_at = now
                    print(f"[headless] Completion verdict observed; allowing "
                          f"{args.completion_grace:g}s settle grace.")
                # ── WAIT FOR THE FIRST STEP, ON A DEVICE-SIZED BUDGET ────────
                # Only under --until-finalized, where THIS runner decides when
                # the load is proven. A world that finalised and never stepped
                # is not a proven load -- the guard after the loop refuses to
                # call it a PASS -- so stopping before the first step is how a
                # healthy mujoco_warp world got reported as a failure. The wait
                # ends the instant the step lands, so a CPU run is unchanged.
                waiting_for_step = False
                if until_finalized_mode and step_seen_at is None:
                    if _NEWTON_STEP_RE.search(completion_text):
                        step_seen_at = now
                        print(f"[headless] first physics step observed "
                              f"{now - completion_at:.2f}s after finalize.")
                    else:
                        if device_seen is None:
                            # Written a few ms after the finalize line, so this
                            # legitimately misses on the first pass; re-read
                            # every iteration until it lands.
                            device_seen = sidecar_device(sidecar_path)
                        budget = step_wait_budget_s(device_seen, warp_in_log,
                                                    args.step_wait_timeout)
                        step_deadline = completion_at + budget
                        if not step_wait_announced and budget > 0:
                            step_wait_announced = True
                            if device_seen:
                                which = f"model device {device_seen}"
                            elif warp_in_log:
                                which = ("model device not readable yet; the engine named "
                                         "the mujoco_warp path")
                            else:
                                which = "model device unknown"
                            print(f"[headless] waiting up to {budget:g}s for the first "
                                  f"physics step ({which}). warp compiles its CUDA kernels "
                                  f"after finalize and logs nothing while it does.")
                        # A controller that failed to start is the OTHER thing
                        # that produces zero steps, and the engine says so out
                        # loud. Stop waiting the moment it does: the verdict
                        # below reports it, and a real failure must not be made
                        # to cost the whole GPU budget.
                        if controller_start_failures(completion_text):
                            print("[headless] abandoning the wait for the first step: the "
                                  "engine reported a controller start/IPC failure "
                                  "(reported below).")
                        elif now < step_deadline:
                            waiting_for_step = True
                settled_after_step = (until_finalized_mode and step_seen_at is not None
                                      and now - step_seen_at >= max(0.0, args.settle_after_step))
                if settled_after_step or (now - completion_at >= max(0.0, args.completion_grace)
                                          and not waiting_for_step):
                    if settled_after_step:
                        print(f"[headless] first physics step landed and the log settled for "
                              f"{args.settle_after_step:g}s; stopping simulator (load + finalize + step proven).")
                        break
                    if until_finalized_mode and step_seen_at is None and step_deadline is not None \
                            and now >= step_deadline:
                        print(f"[headless] no first physics step within "
                              f"{step_deadline - completion_at:g}s of finalize; stopping and "
                              f"reporting what was observed.")
                    else:
                        print("[headless] Completion grace reached, stopping simulator...")
                    break
        if run_start is None:
            live_content = log_tail.read()
            if _NEWTON_STEP_RE.search(live_content):
                run_start = now
                print("[headless] First physics step observed; starting duration clock.")
            elif now - start >= args.startup_timeout:
                print(f"[headless] FAIL: no physics step within startup timeout "
                      f"({args.startup_timeout}s)")
                startup_failed = True
                break
        elif now - run_start >= args.duration:
            # ── THE CEILING, EXTENDED ONLY ON THE PROVEN WARP PATH ───────────
            # Under --until-finalized the duration is a CEILING, not a request
            # (see _resolve_duration_default). On the GPU path that ceiling can
            # expire before the engine has even finalised, because warp compiles
            # kernels first: MEASURED 69.12 s to finalize on a cold cache
            # (machine 9722d23d12a3) against the 10 s default ceiling. The run
            # then exits 0 having proven nothing at all -- no finalize line, no
            # sidecar, zero steps.
            #
            # So extend the wait, but ONLY when the ENGINE ITSELF has named the
            # mujoco_warp path in this run's log, only up to the same bounded
            # budget, and never silently. A CPU load check never reaches this
            # code path and is not slowed by one millisecond.
            if until_finalized_mode and completion_at is None and warp_in_log \
                    and not controller_start_failures(completion_text):
                if warp_deadline is None:
                    extra = step_wait_budget_s(None, True, args.step_wait_timeout)
                    warp_deadline = now + extra
                    if extra > 0:
                        print(f"[headless] {args.duration}s ceiling reached but the engine "
                              f"named the mujoco_warp (GPU) path and has not finalised yet; "
                              f"waiting up to a further {extra:g}s. Pass --step-wait-timeout 0 "
                              f"to stop at the ceiling instead.")
                if now < warp_deadline:
                    time.sleep(0.5)
                    continue
            # ── AND EXTENDED WHILE THE POST-FINALIZE STEP WAIT IS LIVE ───────
            # The branch above only covers the PRE-finalize case. After
            # finalize, the step wait announces a device-sized budget (180 s on
            # cuda) -- but this ceiling used to fire anyway, so every healthy
            # mujoco_warp world FAILED the bare load check: finalize ~9 s,
            # first step ~90-99 s (measured, machine 9722d23d12a3, warm cache),
            # ceiling 30 s. Honour the announced deadline; the wait still ends
            # the instant the step lands or a controller failure is reported.
            if until_finalized_mode and completion_at is not None and step_seen_at is None \
                    and step_deadline is not None and now < step_deadline \
                    and not controller_start_failures(completion_text):
                time.sleep(0.5)
                continue
            print("[headless] Duration reached, stopping simulator...")
            # --until-finalized asked to stop early and did not get to. Say so,
            # because the failure mode is SILENT: the run still PASSes, it just
            # cost the full --duration, and the caller has no way to tell that
            # from a world that legitimately took that long.
            #
            # ⚠ Report only what was OBSERVED. This note used to assert a cause
            # ("the world declares no physics bodies"), which is one explanation
            # of the evidence and not the only one -- a world still loading when
            # the ceiling expired produces byte-identical evidence, and on the
            # GPU path that is the common case, not the rare one. Naming the
            # wrong cause in a diagnostic is worse than naming none.
            if until_finalized_mode and completion_at is None:
                have_sidecar = sidecar_path.exists()
                print("[headless] note: --until-finalized never fired -- no 'world "
                      "finalised' line appeared, so the full --duration was paid.")
                print(f"[headless]   OBSERVED: no finalize line; verdict sidecar "
                      f"{'present' if have_sidecar else 'ABSENT'}; "
                      f"{'the engine named the mujoco_warp path' if warp_in_log else 'no GPU solver named'}"
                      f"; zero physics steps required for this verdict.")
                print("[headless]   NOT DIAGNOSED: that evidence is equally consistent with a "
                      "world that declares NO physics bodies (Newton finalises")
                print("[headless]   nothing -- resources/projects/worlds/empty.omniworld is the "
                      "canonical case, and its whole log is two lines) and with a")
                print("[headless]   world that had simply not finished loading. Re-run with "
                      "--duration N to tell them apart.")
            elif completion_re is not None and completion_at is None:
                print("[headless] note: the --completion-pattern never appeared, so the "
                      "full --duration was paid.")
            break
        # --until-finalized is a load check whose whole cost is latency: poll the log at 100 ms
        # so finalize and the first step are seen within ~0.1 s of landing, not up to 0.5 s.
        time.sleep(0.1 if until_finalized_mode else 0.5)

    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    stdout_sink.close()
    stderr_sink.close()

    if args.profile and log_path.exists():
        # The engine's own attribution, verbatim: world-load stages, PROTO phases, the template
        # engine split, the Newton preload. (The [t+..ms] stamps on every line are the timeline.)
        print("[headless] --profile: the engine's [runtime-cycle] lines:")
        for line in log_path.read_text(errors="replace").splitlines():
            if "[runtime-cycle]" in line:
                print("[headless]   " + line.split("[runtime-cycle]", 1)[1].strip())
    # Analyze log
    errors = 0
    warnings = 0
    if log_path.exists():
        content = log_path.read_text(errors="replace")
        for line in content.splitlines():
            if line.startswith("ERROR:") or line.startswith("FATAL:"):
                errors += 1
                print(f"[headless]   {line}")
            elif line.startswith("WARNING:"):
                warnings += 1
                if args.fail_on_warning:
                    print(f"[headless]   {line}")
    else:
        # No log means the simulator never started (bad DLL search path,
        # wrong binary, instant crash). A PASS here would be a verdict
        # with zero evidence — fail loudly instead.
        print("[headless] FAIL: no log file produced — the simulator never "
              "started or wrote nothing. Check the binary, PATH/DLLs and "
              "OMNISIM_LOG_PATH.")
        return 1

    print(f"[headless] Results: {errors} errors, {warnings} warnings")

    controller_failures = controller_start_failures(content)
    if controller_failures:
        print("[headless] FAIL: one or more controllers failed to start:")
        for line in controller_failures:
            print(f"[headless]   {line}")
        return 1

    # ── CONNECT-ERROR SIDECAR (Phase I2) ─────────────────────────────────────
    # libController writes "<log>.connect_error.txt" when it cannot open its IPC
    # pipe or fails the handshake -- failures that happen BEFORE the controller's
    # stderr stream is installed, so they never reach the engine log. The sidecar
    # is append-only across runs; only entries tagged with THIS run's engine pid
    # (run=<pid>, matching the log header) are ours. Any such entry means a robot
    # executed zero steps: always fatal.
    sidecar_failures = connect_sidecar_failures(content, log_path)
    if sidecar_failures:
        print("[headless] FAIL: libController recorded connect/handshake failures "
              "for this run:")
        for line in sidecar_failures:
            print(f"[headless]   {line}")
        return 1

    if startup_failed:
        return 1

    # ── "THE SIM NEVER STEPPED" GUARD ────────────────────────────────────────
    # A world that PARSES, LOADS and FINALISES but never advances a single
    # simulation step is the most deceptive failure this runner can hit: no
    # ERROR line fires, so the old code below reported PASS. That is exactly how
    # the Go2 BATON demo was called green while the sim sat frozen at t=0 -- an
    # engine <-> libController IPC-nonce mismatch left every controller blocked
    # in Robot() so the engine waited forever, never stepping (commit 6eea9d76).
    # We refuse to call a run that never ticked a PASS. This is ground truth
    # (the engine's own step counter), not a heuristic, and it stays silent
    # unless finalize was PROVEN -- so a merely slow or ODE-only run is safe.
    #
    # ⚠ WHAT THIS MESSAGE MAY SAY. It used to name a cause -- "MOST LIKELY
    # CAUSE: an engine <-> libController IPC mismatch" -- that this runner never
    # checked. On a machine where `python -m omnisim doctor` reported the nonce
    # pair COMPATIBLE, the two statements contradicted each other and sent a
    # reader to rebuild libController for a run whose real problem was that the
    # check stopped 2 s after finalize, before warp had finished compiling. A
    # tool that asserts a cause it did not measure installs a false belief, and
    # the reader has no independent way to catch it (AGENTS.md, "suspect the
    # tool before the prompt"). So: report the OBSERVATIONS, name the candidate
    # explanations as candidates, and claim none of them.
    verdict = sidecar_verdict(sidecar_path)
    if sim_never_stepped(content, bool(verdict.get("finalised"))):
        device = sidecar_device(sidecar_path) or device_seen
        solver = verdict.get("solver") or "not recorded (no readable sidecar)"
        finalize_proof = ("verdict sidecar + 'world finalised' log line" if verdict
                          else "'world finalised' log line (no readable sidecar)")
        print("[headless] FAIL: the world FINALISED but the simulation NEVER "
              "STEPPED (zero physics steps).")
        print("[headless]   OBSERVED -- and nothing beyond this was measured:")
        print(f"[headless]     * finalize proven by: {finalize_proof}")
        print(f"[headless]     * solver: {solver}")
        print(f"[headless]     * model device: {device or 'unknown'}")
        print("[headless]     * ZERO '[OmNewtonBackend] step N' lines: the sim clock "
              "never left t=0")
        if completion_at is not None:
            waited = (step_deadline - completion_at) if step_deadline is not None else 0.0
            print(f"[headless]     * the run ended {time.time() - completion_at:.1f}s after "
                  f"finalize, having waited {waited:.1f}s for a first step")
        else:
            print(f"[headless]     * the run ran {time.time() - start:.1f}s in total")
        print("[headless]   A run that never advanced one step is NOT a PASS; "
              "reporting it as one is a verdict with no evidence.")
        print("[headless]   NOT DIAGNOSED. This runner cannot see WHY and will not "
              "guess: several unrelated causes produce")
        print("[headless]   exactly this evidence -- among them a controller that "
              "never paired (the engine then waits for it")
        print("[headless]   for ever) and, on the GPU path, warp CUDA-kernel "
              "compilation still in progress (measured at 95.4s")
        print("[headless]   after finalize on a cold cache, machine 9722d23d12a3, "
              "with no log output during it).")
        print("[headless]   WORTH CHECKING: `python -m omnisim doctor` reports the "
              "engine/libController IPC-nonce pair (it is a")
        print("[headless]   check, not an accusation -- a compatible pair rules that "
              "cause out); re-running with a longer")
        print("[headless]   --step-wait-timeout, or --duration N, separates 'slow' "
              "from 'stuck'.")
        return 1

    # ── "THE END STATE IS IMPOSSIBLE" GUARD (--fail-on-runaway) ──────────────
    # An ERROR-free, WARNING-free, stepping run can still be nonsense: a body in
    # unbounded free fall through a floor that has no collision surface. The
    # engine logs nothing about it, so without the sampled poses below the lane
    # certifies the broken world and the fixed world identically.
    runaway_failed = ""
    if args.fail_on_runaway:
        data = read_runaway_samples(runaway_out)
        rows = data["rows"]
        header = data["header"]
        if not rows:
            print("[headless] FAIL: --fail-on-runaway collected NO pose samples "
                  f"({runaway_out}).")
            print("[headless]   The runaway_watchdog supervisor never ran or never "
                  "sampled, so there is no evidence either way -- and a PASS with")
            print("[headless]   no evidence is what this flag exists to prevent. "
                  "Check that projects/default/controllers/runaway_watchdog is")
            print("[headless]   present and that the run was long enough to reach "
                  "the first step.")
            runaway_failed = "the runaway check produced no evidence"
        else:
            bodies = header.get("bodies") or []
            skipped = header.get("skipped") or []
            floor_z = header.get("static_floor_z")
            offenders = runaway_verdict(header, rows, z_limit=args.runaway_z_limit)
            names = ", ".join(str(b.get("key")) for b in bodies[:6]) or "none"
            if len(bodies) > 6:
                names += f", +{len(bodies) - 6} more"
            print(f"[headless] Runaway check: {len(rows)} samples to "
                  f"t={rows[-1][0]:.2f}s; top-level dynamic bodies tracked: "
                  f"{len(bodies)} ({names}); lowest DETECTED static collider z="
                  f"{'none' if floor_z is None else format(floor_z, '.3f')} "
                  f"(a supervisor cannot see inside a PROTO)")
            # ── COVERAGE GATE ────────────────────────────────────────────────
            # An empty offender list means one of two very different things:
            # "every tracked body stayed in the world" or "there was nothing to
            # judge". Only the first is a PASS. Without this the flag certified
            # a world whose bodies live under a Group (bodies: [], 40 rows of
            # {}) exactly like a healthy one -- the evidence-free PASS its own
            # message says it exists to prevent.
            gap = runaway_coverage_gap(header, rows)
            if gap:
                print("[headless] FAIL: --fail-on-runaway has NO USABLE EVIDENCE.")
                print(f"[headless]   {gap}.")
                if skipped:
                    print(f"[headless]   The watchdog walked {header.get('skipped_total', len(skipped))} "
                          "node(s) it did NOT track:")
                    for entry in skipped[:8]:
                        print(f"[headless]     {entry.get('key')} "
                              f"({entry.get('type') or 'unknown type'}): "
                              f"{entry.get('reason')}")
                    if len(skipped) > 8:
                        print(f"[headless]     +{len(skipped) - 8} more")
                print("[headless]   A SILENT verdict is not a PASS: runaway_verdict "
                      "returns [] whenever it cannot compute a velocity, so an")
                print("[headless]   unmeasured world and a healthy world look "
                      "identical from here.")
                print("[headless]   REMEDY: run longer (the sampler needs at least "
                      f"{RUNAWAY_WINDOW + 2} samples at "
                      f"--runaway-period-steps={args.runaway_period_steps}), and/or "
                      "hoist the dynamic")
                print("[headless]   bodies to the top level of the .wbt -- a "
                      "supervisor cannot see inside a PROTO, and the watchdog "
                      "descends only one")
                print("[headless]   level into Group / Transform / Pose "
                      "containers.")
                runaway_failed = gap
            for off in offenders:
                print(f"[headless] FAIL: '{off['body']}' left the world.")
                for reason in off["reasons"]:
                    print(f"[headless]   {reason}")
                print(f"[headless]   start z={off['z_start']:.2f} -> exit z="
                      f"{off['z_final']:.2f} (min {off['z_min']:.2f}) at t="
                      f"{off['t_final']:.2f}s over {off['samples']} samples.")
                print("[headless]   A dynamic body accelerating downward with "
                      "nothing under it is the signature of a collision surface")
                print("[headless]   that does not exist: check that the floor / "
                      "terrain Solid under it has a boundingObject (visual")
                print("[headless]   `children` geometry is NEVER collidable on its "
                      "own), and that this body's own physics/boundingObject")
                print("[headless]   pair is complete.")
            if offenders:
                runaway_failed = "%d bod%s left the world" % (
                    len(offenders), "y" if len(offenders) == 1 else "ies")

    if errors > 0:
        print("[headless] FAIL")
        return 1
    if args.fail_on_warning and warnings > 0:
        print("[headless] FAIL (warnings treated as errors)")
        return 1
    if runaway_failed:
        print(f"[headless] FAIL ({runaway_failed})")
        return 1

    print("[headless] PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
