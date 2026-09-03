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

"""roll_check -- does a wheeled robot ROLL, or is it sliding with locked wheels?

WHY THIS EXISTS. Every other check in this repo asks whether the body MOVED. A
headless PASS, a displacement assertion, a "drove to the goal" verdict, an
AgentBench grader: a robot dragging itself along on four frozen wheels satisfies
all of them. That gap hid a real defect for a world's entire life -- a 4-wheel
rover crossing an arena at 1.0-1.6 m/s while its four wheel hinges turned at
~0.14 rad/s, which at the authored 0.08 m radius is 0.011 m/s of actual rolling.
99.3% of that robot's motion was slip, and nothing anywhere noticed, because
nothing anywhere asserted that a wheel turned.

WHAT IT ASSERTS -- no-slip consistency, measured, never modelled:

    | omega_wheel * r  -  v_body |  <=  TOL * max(|v_body|, V_EPS)

⚠ DO NOT REPLACE THIS WITH A STICTION/TORQUE FORMULA. The obvious one does not
survive contact with the engine: the failing rover had 0.4 N.m on each of four
wheels against 3.6 kg, i.e. ~5.6 m/s^2 of nominally available traction, which
should have been ample -- and it still slid, and `maxTorque 12` (the value the
proven-good rover in `projects/robot_combat/worlds/tests/drive_test.omniworld` uses)
fixed it. The mechanism is NOT established. So this is a behavioural check and
the only thing it trusts is the measurement.

HOW A WORLD IS MEASURED. The world itself is never modified. A throwaway
sibling `.omnisim_roll_<stem>.wbt` is written next to it (a sibling, because
`URDFRobot { url ... }` and relative PROTO/texture paths resolve against the
world file), with two edits: every `controller "..."` is swapped to the
uniform `roll_drive`, and a `roll_probe` supervisor Robot is appended. The
sweep sets `OMNISIM_ROLL_OMEGA` from the world's own statically-parsed wheel
radius so a 2.5 cm e-puck wheel and a 30 cm battlebot wheel are driven at
comparable GROUND speed. The sibling is deleted afterwards.

THE TOLERANCE, and how it was chosen. See `TOL` below -- it is set from
measurements on real worlds in this tree, not from a guess. The full write-up,
the corpus table and the caveats are in docs/developer/roll-check.md; the
machine-readable record is tests/goldens/roll_check_baseline.json.

USAGE
    python scripts/dev/roll_check.py scan                 # static candidates
    python scripts/dev/roll_check.py run <world.omniworld>      # measure one world
    python scripts/dev/roll_check.py sweep <world.omniworld>... # measure many
    python scripts/dev/roll_check.py run <w> --no-swap    # observe as authored
    python scripts/dev/roll_check.py grade <raw.json>...  # re-grade, no engine
    python scripts/dev/roll_check.py --self-test          # prove it can go RED
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wbt_wheels import parse_wbt, robots as static_robots  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
SIBLING_PREFIX = ".omnisim_roll_"

# ---------------------------------------------------------------------------
# THE TOLERANCE
# ---------------------------------------------------------------------------
# TOL is the fraction of the body's own speed that may go unaccounted for by
# wheel rotation. It is NOT a guess and NOT fitted to make anything pass. It was
# chosen from the MEASURED distribution over every hand-authored wheeled robot
# in this tree -- 63 graded robots across 57 worlds, 2026-08-10, on
# omnisim-bin-fixed.exe. Sorted, the slip ratios fall into three groups with two
# empty bands between them:
#
#     0.0004 .. 0.3018   16 robots   healthy: rovers, huskies, e-pucks,
#                                    battlebots, the gate world (0.0004)
#     ---- empty 0.302 .. 0.359 ----
#     0.3586 .. 0.4206    3 robots   MARGINAL -- and all three are MyBots in
#                                    worlds running the engine's DEFAULT 32 ms
#                                    step with one substep, i.e. they are early
#                                    cases of the defect, not healthy robots.
#                                    Re-graded after that world's timestep was
#                                    fixed they read 0.142 / 0.169 / 0.249.
#     ---- empty 0.421 .. 0.786 ----
#     0.7863 .. 15.7     44 robots   broken: sliding, wheelspin, launched
#
# 0.35 sits in the FIRST empty band, so no healthy robot measured here is within
# 14% of failing and the marginal three are on the correct side of it. The
# negative control measures 0.602-0.646, i.e. 1.7x over. It is deliberately
# LOOSE: a driven wheel legitimately spins slightly faster than ground speed
# (positive slip is how a tyre makes tractive force at all), a Cylinder
# collider's effective rolling radius is not exactly its authored radius, and a
# skid-steer chassis scrubs.
#
# ⚠ HONEST LIMIT, do not paper over it: the margin above the worst healthy robot
# is only 1.16x, and that worst case (0.3018) is a battlebot being SHOVED by its
# opponent in a multi-robot arena. Restricted to single-robot worlds -- the ones
# a gate is built on -- the healthy population tops out at 0.2324 and the margin
# is 1.5x. If a world ever lands between 0.30 and 0.35, do not retune this:
# investigate, because the only things measured in that neighbourhood are
# robot-on-robot collisions and coarse-timestep defects.
TOL = 0.35

#: Speeds below this are not credited as "moving" in either channel. Sized to
#: the corpus's slowest legitimate rover (the 2.5 cm-wheel e-puck family runs
#: at ~0.1 m/s), an order of magnitude below it so a slow robot is still judged
#: rather than excused.
V_EPS = 0.01

#: A robot slower than this over the whole window is reported IDLE rather than
#: graded -- it never drove, so there is no ratio to take. Distinguishing this
#: from a FAIL matters: "it did not move" and "it moved without rolling" have
#: completely different causes and a check that conflates them is noise.
MOVE_EPS = 0.03

#: Ground speed the sweep aims for, used to pick OMNISIM_ROLL_OMEGA per world.
TARGET_GROUND_SPEED = 0.45

VERDICT_PASS = ("ROLLING",)
VERDICT_FAIL = ("SLIDING", "REVERSED")


# ---------------------------------------------------------------------------
# static scan
# ---------------------------------------------------------------------------

#: Trees that are frozen evidence, not live worlds. Sweeping them would produce
#: a table of "failures" nobody can or should fix: a benchmark RESULT directory
#: records what an agent produced on a given day, and editing it destroys the
#: record. `.harness_*` / `_agentbench_*` files are transient scratch copies
#: another tool regenerates.
EXCLUDE_PARTS = (
    "/results/", "/runs/", "/.git/", "/deliverable/", "/artifact/",
    "/repo_artifacts/", "/scratch/", "/grade/", "/regrade/", "/phase_b/",
)
EXCLUDE_NAMES = (".harness_", "_agentbench_", SIBLING_PREFIX, ".omnisim_runaway_")


def is_live_world(path: Path) -> bool:
    posix = "/" + path.as_posix().replace("\\", "/").lstrip("/") + ""
    if any(part in posix for part in EXCLUDE_PARTS):
        return False
    return not any(path.name.startswith(prefix) for prefix in EXCLUDE_NAMES)


#: The marker every generator in the tree writes into its output's header
#: (`# GENERATED by <script> -- do not hand-edit.`). A generated world is not
#: hand-authored: its verdict belongs to the GENERATOR, and re-measuring every
#: emitted variant (ladder0 alone ships 26) records the same fact 26 times.
GENERATED_MARKER = "GENERATED by"
GENERATED_HEADER_LINES = 6


def is_generated_text(text: str) -> bool:
    """`is_generated_world` on text already read (same rule: the marker on a
    `#` line within the first GENERATED_HEADER_LINES lines)."""
    for line in text.split("\n", GENERATED_HEADER_LINES)[:GENERATED_HEADER_LINES]:
        if line.startswith("#") and GENERATED_MARKER in line:
            return True
    return False


def is_generated_world(path: Path) -> bool:
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for _ in range(GENERATED_HEADER_LINES):
                line = fh.readline()
                if not line:
                    break
                if line.startswith("#") and GENERATED_MARKER in line:
                    return True
    except OSError:
        return False
    return False


#: The world extensions the scan looks at (dual-read: `.wbt` is legacy-accepted).
WORLD_SUFFIXES = (".omniworld", ".wbt")


def git_tracked_files(repo_root: Path, suffixes=None) -> set[Path] | None:
    """Every path git tracks under `repo_root`, resolved; None when git is
    unavailable or this is not a checkout (callers then fall back to the disk).

    `suffixes` keeps only the paths git names with one of those extensions,
    BEFORE resolving: `Path.resolve()` is a filesystem call per path, and
    resolving all ~12.7k tracked files cost 2.4 s of a scan that only ever
    asks about the ~1k world files (MEASURED 2026-09-02, Windows)."""
    try:
        raw = subprocess.run(
            ["git", "-C", str(repo_root), "ls-files", "-z"],
            capture_output=True, check=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    names = [p.decode("utf-8", errors="replace") for p in raw.split(b"\0") if p]
    if suffixes is not None:
        names = [n for n in names if n.endswith(tuple(suffixes))]
    return {(repo_root / n).resolve() for n in names}


#: Text a world MUST contain for `wbt_wheels.robots()` to find a wheel in it:
#: the rule requires a `HingeJoint` whose `device` holds a `RotationalMotor`
#: (see `_wheels_in`), and a `USE` of either still leaves the `DEF` in the
#: text. A world naming neither cannot yield a row, so it is not parsed -- the
#: pure-Python parser was 5.7 s of a 10.5 s scan over 731 tracked worlds, of
#: which 103 name a HingeJoint (MEASURED 2026-09-02). Such a world is skipped
#: whole, parse errors included: it could not have been a wheeled world.
WHEEL_REQUIRED_TOKENS = ("HingeJoint", "RotationalMotor")


def scan(roots, include_frozen=False, include_generated=False, tracked_only=False):
    """Every hand-authored wheeled world under `roots`, with its static facts.

    `tracked_only` restricts a DIRECTORY root to the files git tracks: a bare
    disk walk also sweeps gitignored run residue (`projects/metazoa/_run/...`
    held 29 wheeled worlds on one box), which is how a corpus count drifts
    with whatever the machine happens to hold. A root given as a single FILE
    is always scanned, tracked or not.
    """
    out = []
    tracked = git_tracked_files(REPO_ROOT, suffixes=WORLD_SUFFIXES) if tracked_only else None
    for root in roots:
        root = Path(root)
        if root.is_file():
            paths = [root]
        elif tracked is not None:
            # The tracked set IS the corpus, so enumerate it instead of walking
            # the disk: an rglob finds every gitignored world only to drop it
            # again (4247 on disk against 977 tracked on one box), and the two
            # walks over projects/ + tests/ cost 2.8 s (MEASURED 2026-09-02).
            # Paths are re-rooted on `root` as given so rows read exactly as
            # the walk produced them.
            base = root.resolve()
            paths = sorted(root / p.relative_to(base)
                           for p in tracked if base in p.parents and p.is_file())
        else:
            paths = sorted([*root.rglob("*.omniworld"), *root.rglob("*.wbt")])
        for path in paths:
            if not include_frozen and not is_live_world(path):
                continue
            # One read serves the generated-header check and the wheel gate;
            # an unreadable world is reported exactly as an unparseable one.
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError as exc:
                out.append({"world": str(path), "error": repr(exc)})
                continue
            if not include_generated and is_generated_text(text):
                continue
            if not all(token in text for token in WHEEL_REQUIRED_TOKENS):
                continue
            try:
                found = static_robots(parse_wbt(path))
            except Exception as exc:  # pragma: no cover - unparseable world
                out.append({"world": str(path), "error": repr(exc)})
                continue
            if not found:
                continue
            out.append({
                "world": str(path),
                "robots": [{
                    "def": r.defname, "name": r.name, "controller": r.controller,
                    "n_wheels": len(r.wheels), "radius": r.radius,
                    "max_torque": sorted({w.max_torque for w in r.wheels},
                                         key=lambda x: (x is None, x)),
                    "max_velocity": sorted({w.max_velocity for w in r.wheels},
                                           key=lambda x: (x is None, x)),
                    "bare_cylinder": any(w.bare_cylinder for w in r.wheels),
                } for r in found],
            })
    return out


# ---------------------------------------------------------------------------
# sibling composition
# ---------------------------------------------------------------------------

_CONTROLLER_RE = re.compile(r'(\bcontroller\s+)"[^"]*"')

_PROBE_STANZA = """
# --- appended by scripts/dev/roll_check.py (this file is deleted afterwards) --
# synchronization FALSE for the same reason run-headless --fail-on-runaway uses
# it: a SYNCHRONIZED helper makes the engine WAIT for it, so a slow moment in
# the probe stalls the world it is measuring.
Robot {
  name "roll_probe"
  controller "roll_probe"
  supervisor TRUE
  synchronization FALSE
  controllerArgs [
    "--out=%(out)s"
    "--settle-steps=%(settle)d"
    "--drive-steps=%(drive)d"
    "--sample-every=%(every)d"
  ]
}
"""


def write_sibling(world: Path, out_json: Path, settle: int, drive: int,
                  every: int = 1, swap_controllers: bool = True) -> Path:
    """world + uniform drive controller + measuring supervisor, as a sibling."""
    text = world.read_text(encoding="utf-8", errors="replace")
    # Swap EVERY robot's controller, not just the wheeled ones. A world's own
    # controllers may steer, stop, quit the simulation or wait on a bridge that
    # is not running; replacing them all makes every world in the corpus
    # answer the SAME question -- "when all wheels are told to spin forward,
    # does the body roll?" -- instead of asking each world's author's question.
    #
    # `swap_controllers=False` (`--no-swap`) keeps the world's OWN controllers
    # and only observes. That is the control run you need before blaming a
    # world for a bad verdict: it separates "this world misbehaves" from "this
    # world misbehaves WHEN DRIVEN THE WAY THE SWEEP DRIVES IT".
    if swap_controllers:
        text = _CONTROLLER_RE.sub(r'\1"roll_drive"', text)
    if not text.endswith("\n"):
        text += "\n"
    stanza = _PROBE_STANZA % {
        "out": str(out_json).replace("\\", "/"),
        "settle": int(settle), "drive": int(drive), "every": int(every),
    }
    sibling = world.with_name(f"{SIBLING_PREFIX}{world.stem}{world.suffix}")
    sibling.write_text(text + stanza, encoding="utf-8")
    return sibling


def engine_binary(explicit=None) -> Path:
    if explicit:
        return Path(explicit)
    env = os.environ.get("OMNISIM_ROLL_BINARY")
    if env:
        return Path(env)
    # omnisim-bin-fixed.exe first: on this tree it carries an engine fix the
    # stock binary lacks, and another lane runs the stock one concurrently --
    # so never touch, launch or kill `omnisim-bin.exe` from here.
    for name in ("omnisim-bin-fixed.exe", "omnisim-bin.exe", "omnisim-bin"):
        candidate = REPO_ROOT / "msys64" / "mingw64" / "bin" / name
        if candidate.exists():
            return candidate
    for name in ("bin/omnisim-bin", "bin/omnisim"):
        candidate = REPO_ROOT / name
        if candidate.exists():
            return candidate
    raise SystemExit("roll_check: no OmniSim binary found; set OMNISIM_ROLL_BINARY")


# ---------------------------------------------------------------------------
# the run
# ---------------------------------------------------------------------------

def measure(world: Path, binary: Path, settle_steps=80, drive_steps=500,
            timeout=150.0, omega=None, keep=False, verbose=False,
            swap_controllers=True):
    """Run one world and return the probe's raw measurement document."""
    world = Path(world).resolve()
    try:
        parsed = static_robots(parse_wbt(world))
    except Exception as exc:
        # Name the world even in the failure branch. An error row that reports
        # `world: None` is unactionable, and a sweep of 60 of them is worse.
        return {"world": str(world), "error": f"world does not parse: {exc!r}"}
    radii = [r.radius for r in parsed if r.radius]
    if omega is None:
        radius = statistics.median(radii) if radii else 0.08
        omega = max(1.0, min(20.0, TARGET_GROUND_SPEED / radius))

    tmp = Path(tempfile.mkdtemp(prefix="rollchk_"))
    out_json = tmp / "roll.json"
    log_path = tmp / "omnisim_log.txt"
    sibling = write_sibling(world, out_json, settle_steps, drive_steps,
                            swap_controllers=swap_controllers)

    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["WEBOTS_HOME"] = str(REPO_ROOT)
    env["OMNISIM_LOG_PATH"] = str(log_path)
    env["OMNISIM_ROLL_OMEGA"] = "%.6f" % omega
    # Newton is the only backend; make a missing runtime a loud failure rather
    # than a world that silently never moves (which would read as IDLE here).
    env["OMNISIM_REQUIRE_NEWTON"] = "1"

    argv = [str(binary), str(sibling), "--batch", "--mode=fast",
            "--no-rendering", "--minimize", "--stdout", "--stderr"]
    started = time.time()
    proc = subprocess.Popen(argv, env=env, cwd=str(REPO_ROOT),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    killed = False
    try:
        # Poll for the probe's completion marker instead of waiting out the
        # timeout: several corpus worlds ignore simulationQuit, so "finished"
        # has to be read from the artefact, not from process exit.
        while time.time() - started < timeout:
            if proc.poll() is not None:
                break
            if out_json.exists():
                try:
                    if json.loads(out_json.read_text(encoding="utf-8")).get("complete"):
                        break
                except (OSError, ValueError):
                    pass
            time.sleep(0.25)
        else:
            killed = True
    finally:
        if proc.poll() is None:
            # Kill ONLY this child's tree, by pid. Never a name-based sweep:
            # another lane runs `omnisim-bin.exe` in this same tree.
            _kill_tree(proc.pid)
        proc.wait(timeout=20)
        if not keep:
            sibling.unlink(missing_ok=True)

    doc = {}
    if out_json.exists():
        try:
            doc = json.loads(out_json.read_text(encoding="utf-8"))
        except ValueError as exc:
            doc = {"error": f"probe JSON unreadable: {exc}"}
    else:
        doc = {"error": "probe wrote no JSON (controller never ran?)"}
    doc["world"] = str(world)
    doc["omega_cmd_rad_s"] = omega
    doc["wall_s"] = round(time.time() - started, 1)
    doc["timed_out"] = killed
    if verbose and log_path.exists():
        doc["log_tail"] = log_path.read_text(errors="replace")[-3000:]
    if not keep:
        for leftover in (out_json, log_path):
            leftover.unlink(missing_ok=True)
        for extra in tmp.glob("*"):
            extra.unlink(missing_ok=True)
        tmp.rmdir()
    else:
        doc["artifacts"] = str(tmp)
    return doc


def _kill_tree(pid: int):
    if sys.platform == "win32":
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       check=False)
    else:  # pragma: no cover - posix
        import signal
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass


# ---------------------------------------------------------------------------
# the verdict
# ---------------------------------------------------------------------------

#: Minimum number of MOVING samples needed before a ratio is credited.
MIN_MOVING_SAMPLES = 10


def grade_robot(slot, tol=TOL):
    """Grade one robot's sample series. Returns a dict, never raises.

    TWO THINGS HERE ARE NOT OBVIOUS AND BOTH WERE LEARNED FROM A BAD FIRST
    SWEEP, so do not "simplify" them back out:

    1. THE SIGN OF THE RESIDUAL IS THE DIAGNOSIS, not just its size. Wheels
       turning LESS than the body's motion needs is the sliding defect (the
       chassis is being dragged). Wheels turning MORE is wheelspin -- no
       traction, or the robot is blocked. A symmetric |omega*r - v| test calls
       both "slip" and then the failure message reads "wheels turn only 4.995
       rad/s" about a robot whose wheels were spinning ten times too fast,
       which is worse than no message.

    2. ONLY THE SAMPLES WHERE THE ROBOT WAS ACTUALLY MOVING ARE GRADED. Half
       this corpus is multi-robot combat arenas: driven straight ahead they
       reach a wall or each other inside the window and stop, and a median over
       the whole window then measures the wall, not the wheels. Filtering to
       the moving samples asks the question that is actually interesting --
       "WHILE it was moving, did its wheels account for the motion?" -- and
       `moving_fraction` records how much of the window that was, so a verdict
       resting on a brief window is visible rather than implied.
    """
    samples = slot.get("samples") or []
    if len(samples) < MIN_MOVING_SAMPLES:
        return {"verdict": "NO_DATA", "n_samples": len(samples),
                "detail": "fewer than %d samples: the probe never saw this robot run"
                          % MIN_MOVING_SAMPLES}

    moving = [s for s in samples if abs(s[1]) >= MOVE_EPS]
    graded_on_moving = len(moving) >= MIN_MOVING_SAMPLES
    window = moving if graded_on_moving else samples

    v_fwd = statistics.median(s[1] for s in window)
    v_roll = statistics.median(s[2] for s in window)
    speed = statistics.median(s[3] for s in window)
    spin = statistics.median(s[4] for s in window)
    yaw = statistics.median(abs(s[5]) for s in window)
    residual = abs(v_roll - v_fwd)
    ratio = residual / max(abs(v_fwd), V_EPS)
    rolled = (v_roll / v_fwd) if abs(v_fwd) > V_EPS else None
    signed = (v_roll - v_fwd) / max(abs(v_fwd), V_EPS)

    if not graded_on_moving and abs(v_fwd) < MOVE_EPS:
        verdict = "IDLE" if abs(v_roll) < MOVE_EPS else "WHEELSPIN"
    elif rolled is not None and rolled < 0:
        verdict = "REVERSED"
    elif ratio <= tol:
        verdict = "ROLLING"
    elif signed < 0:
        verdict = "SLIDING"      # the wheels turn LESS than the body moves
    else:
        verdict = "WHEELSPIN"    # the wheels turn MORE than the body moves

    return {
        "verdict": verdict,
        "n_samples": len(samples),
        "n_moving": len(moving),
        "moving_fraction": round(len(moving) / len(samples), 3),
        "graded_on_moving_only": graded_on_moving,
        "v_body_fwd_mps": round(v_fwd, 4),
        "v_roll_mps": round(v_roll, 4),
        "body_speed_mps": round(speed, 4),
        "omega_spin_rad_s": round(spin, 4),
        "yaw_rate_rad_s": round(yaw, 4),
        "residual_mps": round(residual, 4),
        "slip_ratio": round(ratio, 4),
        "rolled_fraction": None if rolled is None else round(rolled, 4),
        "radius_m": slot.get("radius"),
        "detail": _explain(verdict, v_fwd, v_roll, spin, slot.get("radius"), ratio,
                           len(moving), len(samples)),
    }


def _explain(verdict, v_fwd, v_roll, spin, radius, ratio, n_moving, n_total):
    """A failure has to explain itself or it will be dismissed as noise."""
    scope = ("" if n_moving >= n_total
             else " [graded on the %d of %d samples where it was moving]"
                  % (n_moving, n_total))
    if verdict == "ROLLING":
        return ("body %.3f m/s, wheels %.3f rad/s x r=%.4g = %.3f m/s of rolling "
                "(%.1f%% of body motion unaccounted for)%s"
                % (v_fwd, spin, radius or 0, v_roll, 100 * ratio, scope))
    if verdict == "SLIDING":
        need = (v_fwd / radius) if radius else float("nan")
        return ("body %.3f m/s but wheels turn only %.3f rad/s = %.4f m/s of rolling; "
                "%.1f%% of the motion is SLIP. Rolling at r=%.4g m would need "
                "%.1f rad/s. The chassis is being dragged, not driven.%s"
                % (v_fwd, spin, v_roll, 100 * ratio, radius or 0, need, scope))
    if verdict == "REVERSED":
        return ("body %.3f m/s but the wheels turn the OTHER WAY (%.3f rad/s = "
                "%.3f m/s): the body is not being propelled by these wheels at all%s"
                % (v_fwd, spin, v_roll, scope))
    if verdict == "WHEELSPIN":
        return ("wheels turn %.3f rad/s (= %.3f m/s of rolling) but the body moves "
                "only %.3f m/s: the wheels are spinning faster than the robot "
                "travels -- no traction, or it is blocked. NOT the sliding defect, "
                "and not a clean pass either%s"
                % (spin, v_roll, v_fwd, scope))
    if verdict == "IDLE":
        return ("neither the body (%.3f m/s) nor the wheels (%.3f rad/s) moved: the "
                "drive never took effect, so no-slip cannot be graded"
                % (v_fwd, spin))
    return "no usable samples"


def grade(doc, tol=TOL):
    """Grade a whole world. `verdict` is the worst robot's verdict."""
    if doc.get("error"):
        return {"world": doc.get("world"), "verdict": "ERROR",
                "detail": doc["error"], "robots": []}
    slots = doc.get("robots") or []
    graded = []
    for slot in slots:
        row = grade_robot(slot, tol)
        row["def"] = slot.get("def")
        row["name"] = slot.get("name")
        row["n_wheels"] = slot.get("n_wheels")
        graded.append(row)
    order = ["SLIDING", "REVERSED", "NO_DATA", "WHEELSPIN", "IDLE", "ROLLING"]
    if not graded:
        world_verdict = "NO_ROBOTS"
    else:
        world_verdict = min((r["verdict"] for r in graded),
                            key=lambda v: order.index(v) if v in order else 99)
    return {"world": doc.get("world"), "verdict": world_verdict,
            "omega_cmd_rad_s": doc.get("omega_cmd_rad_s"),
            "timed_out": doc.get("timed_out"), "wall_s": doc.get("wall_s"),
            "robots": graded}


def passed(result):
    return result["verdict"] in VERDICT_PASS


def failed(result):
    return result["verdict"] in VERDICT_FAIL


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _print_result(result, quiet=False):
    mark = {"ROLLING": "PASS", "SLIDING": "FAIL", "REVERSED": "FAIL"}.get(
        result["verdict"], "----")
    print(f"[{mark}] {result['verdict']:<10} {result['world']}")
    if quiet and mark == "PASS":
        return
    for row in result.get("robots", []):
        label = row.get("def") or row.get("name") or "?"
        print(f"         {row['verdict']:<10} {label:<22} {row.get('detail', '')}")
    if result.get("detail"):
        print(f"         {result['detail']}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", nargs="?", default="scan",
                        choices=["scan", "run", "sweep", "grade"])
    parser.add_argument("paths", nargs="*")
    parser.add_argument("--binary", default=None)
    parser.add_argument("--settle-steps", type=int, default=80)
    parser.add_argument("--drive-steps", type=int, default=500)
    parser.add_argument("--timeout", type=float, default=150.0)
    parser.add_argument("--omega", type=float, default=None)
    parser.add_argument("--tol", type=float, default=TOL)
    parser.add_argument("--json", default=None, help="write results here")
    parser.add_argument("--raw-dir", default=None,
                        help="also save each world's raw probe document here, so "
                             "a tolerance change can be re-graded without "
                             "re-measuring (`roll_check.py grade <raw>...`)")
    parser.add_argument("--keep", action="store_true", help="keep sibling + artifacts")
    parser.add_argument("--include-frozen", action="store_true",
                        help="also scan benchmark result / artifact trees")
    parser.add_argument("--include-generated", action="store_true",
                        help="also scan worlds whose header says a generator wrote them")
    parser.add_argument("--tracked-only", action="store_true",
                        help="restrict directory roots to files git tracks (skip run residue)")
    parser.add_argument("--no-swap", action="store_true",
                        help="keep the world's OWN controllers and only observe "
                             "(the control run for a suspicious verdict)")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--self-test", action="store_true",
                        help="prove the check can go RED before trusting its green")
    parser.add_argument("--regenerate-negative-control", action="store_true",
                        help="rewrite the negative-control world from the gate world")
    args = parser.parse_args(argv)

    if args.regenerate_negative_control:
        return regenerate_negative_control()

    if args.self_test:
        return self_test(args)

    if args.command == "scan":
        roots = args.paths or [REPO_ROOT / "projects", REPO_ROOT / "tests"]
        rows = scan(roots, include_frozen=args.include_frozen,
                    include_generated=args.include_generated,
                    tracked_only=args.tracked_only)
        if args.json:
            Path(args.json).write_text(json.dumps(rows, indent=2), encoding="utf-8")
        for row in rows:
            if row.get("error"):
                print(f"ERROR {row['world']}: {row['error']}")
                continue
            tq = sorted({t for r in row["robots"] for t in r["max_torque"]},
                        key=lambda x: (x is None, x))
            rad = sorted({r["radius"] for r in row["robots"]})
            print(f"{row['world']:<95} robots={len(row['robots'])} "
                  f"maxTorque={tq} r={rad}")
        print(f"\n{len(rows)} hand-authored wheeled world(s).")
        return 0

    if args.command == "grade":
        results = []
        for path in args.paths:
            doc = json.loads(Path(path).read_text(encoding="utf-8"))
            results.append(grade(doc, args.tol))
            _print_result(results[-1], args.quiet)
        return 0 if not any(failed(r) for r in results) else 1

    binary = engine_binary(args.binary)
    worlds = [Path(p) for p in args.paths]
    if args.command == "run" and len(worlds) != 1:
        parser.error("run takes exactly one world (use sweep for many)")
    raw_dir = Path(args.raw_dir) if args.raw_dir else None
    if raw_dir:
        raw_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for world in worlds:
        doc = measure(world, binary, settle_steps=args.settle_steps,
                      drive_steps=args.drive_steps, timeout=args.timeout,
                      omega=args.omega, keep=args.keep, verbose=args.verbose,
                      swap_controllers=not args.no_swap)
        if raw_dir:
            # Keep the raw samples. A sweep of this corpus costs the better part
            # of an hour, and a tolerance or verdict-rule change must be
            # re-gradeable offline (`roll_check.py grade <raw>...`) rather than
            # re-measured -- otherwise nobody will ever revisit the rule.
            # Stem + a path hash: this corpus has several worlds sharing a stem
            # (`example.wbt` under projects/languages/{cpp,python}), and keying
            # on the stem alone silently overwrites one with the other.
            stem = re.sub(r"[^A-Za-z0-9_.-]", "_", Path(world).stem)
            tag = hashlib.sha1(str(Path(world).resolve()).encode()).hexdigest()[:8]
            (raw_dir / f"{stem}.{tag}.json").write_text(json.dumps(doc),
                                                        encoding="utf-8")
        result = grade(doc, args.tol)
        results.append(result)
        _print_result(result, args.quiet)
        sys.stdout.flush()
    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=2), encoding="utf-8")
    bad = [r for r in results if failed(r)]
    print(f"\n{len(results)} world(s): "
          f"{sum(1 for r in results if passed(r))} ROLLING, {len(bad)} FAILING, "
          f"{len(results) - len(bad) - sum(1 for r in results if passed(r))} other.")
    return 1 if bad else 0


SELF_TEST_GOOD = "tests/physics/worlds/wheel_roll_noslip.omniworld"
SELF_TEST_BAD = "scripts/dev/roll_check_assets/wheel_roll_slip_negative_control.omniworld"

#: The ONLY differences between the gate world and its negative control, kept
#: as data so `--regenerate-negative-control` cannot drift from what is
#: documented.
#:
#: ⚠ IT TAKES ALL THREE PHYSICAL EDITS TO REPRODUCE THE DEFECT, and finding that
#: out is why this list is not just `maxTorque`. Measured on this tree with
#: `omnisim-bin-fixed.exe`, one factor at a time, everything else held at the
#: gate world's proven-good values (slip ratio = fraction of body motion NOT
#: accounted for by wheel rotation):
#:
#:   maxTorque 0.4 alone .............................. 0.000  ROLLS
#:   basicTimeStep 16 + no substeps alone ............. 0.001  ROLLS
#:   maxTorque 0.4 + basicTimeStep 16 + no substeps ... 0.602  SLIDES
#:
#: So "an under-powered motor" is only half the story: 0.4 N.m is ample at
#: dt=8 ms with 4 substeps, and a 16 ms single-substep integration is fine at
#: 12 N.m. The failure is the INTERACTION -- a torque budget too small to
#: correct the contact error a coarse integration accumulates in one step. This
#: is exactly why the check is behavioural: no stiction formula predicts a
#: dependence on the timestep.
NEGATIVE_CONTROL_EDITS = (
    ("maxTorque 12", "maxTorque 0.4"),
    ("basicTimeStep 8", "basicTimeStep 16"),
    ("  newtonSubsteps 4\n", ""),
    ('controller "wheel_roll_noslip"', 'controller "roll_drive"'),
)


def regenerate_negative_control():
    """Rewrite the negative control from the gate world + NEGATIVE_CONTROL_EDITS.

    The value of the control is that it differs from the gate world in ONE
    physical field. Regenerating mechanically is what keeps that true after
    someone edits the gate world -- a hand-maintained copy drifts, and a drifted
    control silently stops being a control.
    """
    good = REPO_ROOT / SELF_TEST_GOOD
    bad = REPO_ROOT / SELF_TEST_BAD
    text = good.read_text(encoding="utf-8")
    body = text[text.index("\nWorldInfo {"):]
    for needle, replacement in NEGATIVE_CONTROL_EDITS:
        if needle not in body:
            raise SystemExit(f"roll_check: {SELF_TEST_GOOD} no longer contains "
                             f"{needle!r}; the negative control cannot be derived")
        body = body.replace(needle, replacement)
    old = bad.read_text(encoding="utf-8")
    header = old[:old.index("\nWorldInfo {")]
    bad.write_text(header + body, encoding="utf-8")
    print(f"regenerated {SELF_TEST_BAD} from {SELF_TEST_GOOD}")
    return 0


def self_test(args):
    """Run the known-good and the known-bad world and require OPPOSITE verdicts.

    A gate that has only ever been seen to pass is not evidence of anything.
    The negative control is a byte-for-byte copy of the good world with ONE
    field changed (maxTorque 12 -> 0.4), so a green self-test says the check
    discriminates on the thing it claims to.
    """
    binary = engine_binary(args.binary)
    ok = True
    for rel, want_pass in ((SELF_TEST_GOOD, True), (SELF_TEST_BAD, False)):
        world = REPO_ROOT / rel
        if not world.exists():
            print(f"[self-test] MISSING {rel}")
            return 2
        doc = measure(world, binary, settle_steps=args.settle_steps,
                      drive_steps=args.drive_steps, timeout=args.timeout,
                      keep=args.keep)
        result = grade(doc, args.tol)
        _print_result(result)
        got_pass = passed(result)
        if got_pass != want_pass:
            print(f"[self-test] FAILED: {rel} expected "
                  f"{'PASS' if want_pass else 'FAIL'}, got {result['verdict']}")
            ok = False
    print("[self-test] " + ("OK -- the check discriminates: the good world rolls, "
                            "the under-torqued copy slides."
                            if ok else "BROKEN"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
