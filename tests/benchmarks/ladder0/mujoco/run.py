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

"""run.py -- the ladder's MuJoCo arm.

Runs each rung against a bare ``mujoco`` install with CPU ``mj_step``: no
OmniSim, no Newton, no warp, no GPU.  It MEASURES physical quantities -- rest
height in metres, fall interval in seconds, angular rate in rad/s, distance in
metres -- and hands them to the ladder's shared reduction and shared ground
truth.  It computes no expected value and owns no tolerance.

Why this arm is worth more than a competitor column: MuJoCo IS OmniSim's
physics.  ODE was deleted; the Newton backend's only solver is MuJoCo, reached
through a ``.wbt`` -> newton -> mjModel translation layer.  So where the two
columns disagree on a rung, the physics is common and the difference is in
that translation -- which makes a disagreement here a pointer, not a mystery.

    python run.py                 # every rung, table + JSON
    python run.py --rung 8        # one rung
    python run.py --rung 18       # the on-demand rung: 50 recorded cube tosses
    python run.py --integrator implicitfast   # solver-sensitivity variant
    python run.py --fault slide                # prove the rung can go red

It is also reachable through the shared runner --
``python ../run_ladder.py --sims mujoco`` -- via ``arm.py``, which is this
arm's CONTRACT.md section 5 face and does nothing but adapt what is here.

Usage as a library: ``run_rung(n)`` returns ``(samples, meta)`` where
``samples`` is the ladder's shared sample document, and ``run_cell(n, dir)``
returns the same for a MULTI-RUN rung.

ONE CELL IS NOT ALWAYS ONE RUN (CONTRACT.md amendment A).  Rungs 9, 11 and 18
are scene FAMILIES: the contract names the runs, ``scenes.run_specs`` builds
them, and ``run_cell`` drives them.  Rung 9's three replicas are spawned as
SEPARATE INTERPRETERS -- ``run.py --replica`` is that child mode -- because a
determinism rung whose replicas share one process measures this arm's ability
to copy an array rather than the engine's reproducibility, and the reducer
asserts it from the ``(pid, process-start)`` pair each run records for itself.
Rung 11's four fleets and rung 18's fifty tosses are library calls here, which
is honest and is all the contract asks of them.

THE SCENE IS THE CONTRACT, VERBATIM.  There is no floor-size override and no
other scene knob: a row produced from a scene the contract did not describe is
not comparable with the other two columns, and nothing in the row would say so.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import subprocess
import sys
import time

HERE = os.path.abspath(os.path.dirname(__file__))
RESULTS = os.path.join(HERE, "results")


def _load_shared():
    """Bootstrap this arm's loader by path -- never ``import shared``.

    All three arms are loaded into ONE process by ``run_ladder.py`` and
    ``sys.modules`` is global to it, so a bare sibling import can hand this arm
    another simulator's module.  See ``shared.py``'s docstring; ``run_ladder``
    refuses an arm that has done it.
    """
    key = "ladder0_mujoco_shared"
    if key in sys.modules:
        return sys.modules[key]
    sp = importlib.util.spec_from_file_location(
        key, os.path.join(HERE, "shared.py"))
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


shared = _load_shared()
spec = shared.spec
analysis = shared.analysis
ANALYSIS_AVAILABLE = shared.ANALYSIS_AVAILABLE
scenes = shared.sibling("scenes")
drivers = shared.sibling("drivers")

SIM = "mujoco"

# Faults exist so a green row can be shown to be capable of going red.  Each
# is a PHYSICAL corruption of the scene or the actuation, not a corruption of
# the measurement: the plumbing stays honest and the simulation misbehaves,
# which is the only kind of self-test that proves anything about the plumbing.
FAULTS = {
    # --- CONTRACT.md section 6: the five the shared self-test asks for ------
    "short_run": "rung 0 stops halfway -- steps_completed must go red",
    "no_floor": (
        "rung 1's floor geom is deleted -- there is no implicit ground plane "
        "in MJCF, so the box falls for ever and rest_z must go red"),
    "half_gravity": "gravity set to g/2 -- rung 2's fall interval must go red",
    "ignore_zero": (
        "rung 3 keeps commanding the driven rate after the command switches "
        "to zero -- the servo that ignores its input; omega_zero must go red "
        "while omega_driven stays green"),
    "slide": (
        "rung 4's wheel joints LOCKED (damping 1e6) and the chassis dragged "
        "along +X at exactly omega*r -- the right distance on wheels that "
        "never turn.  distance must stay GREEN and rolling_consistency must "
        "go red; that asymmetry is why rung 4 asserts two things"),
    "no_sweep": (
        "rung 5's carrier is never moved.  The ray stays honest and the SCENE "
        "stops changing, which is what a sensor frozen at t=0 looks like; "
        "range_final reds while range_static and range_tracks stay green"),
    "wall_shifted": (
        "rung 5's wall is authored RUNG5_FAULT_SHIFT further away -- the "
        "SCENE moves and the motion does not, so range_static reds and "
        "sweep_span stays green"),
    "no_stop": (
        "rung 6 never commands the stop.  The ACTING half breaks and the "
        "SENSING half must not: stop_gap reds while trigger_reading -- which "
        "the reducer re-derives from the recorded series, not from the "
        "driver's decision -- stays green.  NOTE that ``sensor_agrees`` also "
        "reds here, and it is a CONVENTION and not a defect: the rover ends "
        "up against the wall with its sensor 20 mm INSIDE it (the mount is "
        "20 mm proud of the bumper), and a MuJoCo ray that starts inside a "
        "box reports the distance to the box's FAR face.  Measured: reading "
        "0.18018 against a pose gap of -0.01982, a residual of exactly the "
        "0.2 m wall thickness"),
    "bounce": (
        "rung 6's rover overshoots by RUNG6_FAULT_BOUNCE_M and is then put "
        "back at RUNG6_FAULT_REST_GAP -- every tail-window check green, and "
        "only the whole-run min_gap can see it.  Rung 6's ``slide``"),
    "stalled_robot": (
        "rung 7's centre robot has its wheels commanded zero -- its own "
        "distance goes wrong, nobody else's does, and the fleet stays apart"),
    "lane_offset": (
        "rung 7's centre robot is held RUNG7_FAULT_OFFSET_M into its "
        "neighbour's lane without touching its wheels: min_separation reds "
        "while distance_worst and wheel_omega_worst stay green"),
    "no_grip": (
        "rung 8's fingers never close.  THE CAUSAL CONTROL: same scene, same "
        "schedule, and the part must stay on the table"),
    "no_traverse": (
        "rung 8's traverse is never commanded -- place_x reds and the lift "
        "and the grasp stay green"),
    "drop_mid_carry": (
        "rung 8's fingers reopen at the top of the lift.  The part falls "
        "0.15 m and arrives at 1.7 m/s; its final pose looks plausible and "
        "only part_speed_max sees it"),
    # --- this arm's own extras, kept for redcheck.py -----------------------
    "gravity_half": "alias of half_gravity (this arm's older name)",
    "slide_not_roll": "alias of slide (this arm's older name)",
    "soft_floor": (
        "contact solref timeconst 0.02 -> 0.40 s -- the box sinks into the "
        "floor and rung 1's rest height must go red"),
    "weak_grip": (
        "rung 8's pads squeeze BELOW the Coulomb bound m g / (2 mu).  The "
        "pads are in contact for the whole run and the part never leaves the "
        "table -- the case a contact COUNT scores as a successful grasp"),
    # --- rung 9: determinism, and the two ways of faking it ----------------
    "seed_nudge": (
        "replica b ALONE has its dropped cube spawned RUNG9_FAULT_NUDGE off.  "
        "repeat_delta must go red while the sensitivity control and the "
        "analytic anchor stay green.  NOTE the emitter writes rung 9's poses "
        "as exact float64 literals for this fault's sake: the ordinary %.6f "
        "rounds the nudge away and the fault silently would not happen -- "
        "redcheck.py re-proves that on every run rather than trusting it"),
    "frozen": (
        "rung 9's dropped cube loses its freejoint -- it keeps its geom, so "
        "it is still a collider, it simply cannot move.  RUNG 9'S ``slide``: "
        "a world that cannot move is PERFECTLY deterministic, so repeat_delta "
        "must stay GREEN while fall_interval and sensitivity_shortfall both "
        "go red.  Without it the rung would be satisfiable by an engine that "
        "simulated nothing"),
    "short_b": (
        "replica b ALONE runs RUNG9_FAULT_SHORT of the steps.  A truncated "
        "replica has a trivially small delta OVER THE OVERLAP, which is how "
        "one passes a determinism check; only repeat_length sees it"),
    # --- rung 18: replaying a recording ------------------------------------
    "ic_drop_velocity": (
        "rung 18 hands the engine ZERO velocities while the record keeps the "
        "RECORDED ones as what was wanted -- the shape of setVelocity being "
        "silently dropped at t=0, which this tree has shipped.  "
        "replay_ic_fidelity is the check that sees it; without that "
        "assertion a dropped velocity presents as poor real-world agreement "
        "and gets charged to the contact model"),
    "wrong_omega_frame": (
        "rung 18 hands the dataset's BODY-frame angular velocity through as "
        "if it were world frame -- lane1r's own documented silent error.  "
        "replay_ic_fidelity must stay GREEN: the engine accepted exactly what "
        "it was given, so only real_rot_err may see it"),
    "table_hologram": (
        "rung 18's table geom is deleted.  MJCF has no implicit ground plane, "
        "so the cube falls for ever; only the whole-run tunnel_depth sees it, "
        "because a replay that fell through the table can still score a "
        "plausible MEAN position error"),
}

#: Which rung each fault belongs to, where a rung's faults are implemented
#: OUTSIDE this file.  Today: rung 18, whose replay lives in ``rung18.py``.
#:
#: THE LIST LIVES HERE, WITH THE CATALOGUE, AND ``rung18.py`` READS IT.  A
#: second copy over there is a place for the two to disagree, and the way they
#: disagree is the worst available: the module answers "I do not implement that
#: fault" for one it does, and the self-test scores a real proof as MISSING
#: rather than as failed.  That is not hypothetical -- ``arm.SUPPORTED_FAULTS``
#: was a hand-written set that fell behind this dict, and nine implemented
#: faults reported themselves as unimplemented for a whole build.
RUNG_FAULTS = {18: ("ic_drop_velocity", "wrong_omega_frame",
                    "table_hologram")}
for _rung, _names in RUNG_FAULTS.items():
    _missing = [n for n in _names if n not in FAULTS]
    if _missing:                                    # pragma: no cover
        raise RuntimeError("rung %d claims faults absent from this arm's "
                           "catalogue: %s" % (_rung, _missing))

#: Rungs whose cell is more than one RUN (CONTRACT.md amendment A), read from
#: the contract.  Never re-listed: a second copy of this set is a place for the
#: two to disagree, and this arm has been bitten by exactly that before.
MULTI_RUN = frozenset(spec.MULTI_RUN)

#: Rungs whose replicas MUST come from distinct OS processes.  Rung 9 alone:
#: a determinism rung whose replicas share one interpreter measures the arm,
#: not the engine, and ``rungs.check_rung`` asserts it from the (pid,
#: process-start) pair each run records for ITSELF.
NEEDS_DISTINCT_PROCESSES = frozenset([9])

#: Sample stride per rung (CONTRACT.md amendment D), from the contract.
SAMPLE_EVERY = {9: spec.RUNG9_SAMPLE_EVERY, 11: spec.RUNG11_SAMPLE_EVERY}

#: This interpreter's own start, epoch seconds.  Recorded INSIDE each run so
#: the (pid, proc_start) pair is the running process's own fact and not the
#: parent's opinion of it -- a pid alone is reused within a session.
PROC_START = time.time()

# The arm's older fault names, mapped onto the contract's.  Kept because
# ``redcheck.py`` and the committed result files use them.
FAULT_ALIASES = {"gravity_half": "half_gravity", "slide_not_roll": "slide"}


def canonical_fault(fault):
    """Contract fault name for ``fault``; ``None``/``"none"`` -> ``None``."""
    if fault in (None, "none", ""):
        return None
    return FAULT_ALIASES.get(fault, fault)

# Why ``slide_not_roll`` locks the joint instead of just cutting the motor:
# MEASURED, and it is worth knowing before reading rung 4's green row.  The
# first version of this fault only zeroed the servo gain and pushed the
# chassis -- and the rung still PASSED, with the wheels reading 4.0006 rad/s.
# An UNDRIVEN wheel dragged across ground at mu = 1.0 rolls at exactly v/r,
# because rolling is the only motion that costs no friction work.  So
# ``wheel_omega`` and ``rolling_consistency`` prove the wheels are TURNING
# CONSISTENTLY WITH THE MOTION -- which is the anti-slide claim the rung makes
# and is exactly right -- but neither of them, nor any measurement in this
# rung, proves the wheels are what PROPELLED the robot.  A rung that wanted
# that would have to remove the push and check the robot still moves.
_LOCK_WHEEL_JOINT = ' damping="1e6"'


# --------------------------------------------------------------------------
# model construction
# --------------------------------------------------------------------------

def available():
    """(ok, detail) -- is a usable ``mujoco`` importable?"""
    try:
        import mujoco
    except Exception as exc:            # pragma: no cover - env dependent
        return False, "%s: %s" % (type(exc).__name__, exc)
    return True, mujoco.__version__


def _xml_for(rung, integrator=None, fault=None, run=None):
    """The scene, with any SCENE-level fault applied.

    Faults are injected into the RUN -- the scene here, or the actuation in the
    step loop -- and never into the measurement.  The recording path is
    byte-identical in a faulted and an unfaulted run.

    ``run`` is one entry of ``scenes.run_specs`` and ``fault`` reaches the
    emitter too: rungs 9, 11 and 18 have faults that are STRUCTURAL (a body
    that loses its joint, a robot spawned out of lane, a table that is not
    there) and a structural change is better expressed by the generator than
    by a text edit that a change to the template would silently stop matching.
    """
    fault = canonical_fault(fault)
    xml = scenes.mjcf(rung, run=run, fault=fault)
    if integrator:
        xml = xml.replace('<option timestep=',
                          '<option integrator="%s" timestep=' % integrator, 1)
    if fault == "half_gravity":
        xml = xml.replace("gravity=\"0 0 -%s\"" % spec.G,
                          "gravity=\"0 0 -%s\"" % (spec.G / 2.0), 1)
    if fault == "soft_floor":
        xml = xml.replace('<geom friction=',
                          '<geom solref="0.4 1" friction=', 1)
    if fault == "no_floor":
        # Delete the floor outright.  MJCF has no implicit ground plane -- the
        # solver simulates exactly the geoms in the file -- so this arm's box
        # falls for ever.  (On OmniSim the same fault settles the box at
        # z = 0.0999 on a surface that appears in no world file, which is the
        # phantom-plane defect this ladder's raised floor exists to expose.)
        i = xml.find('<geom name="floor"')
        if i >= 0:
            j = xml.find("/>", i)
            xml = xml[:i] + xml[j + 2:]
    if fault == "wall_shifted" and rung == 5:
        # Move the SCENE and leave the motion alone.  Emitted by re-running
        # the wall through the same helper with a shifted near face, so the
        # faulted scene is still generated from the contract rather than
        # patched by hand.
        good = scenes._wall("wall", spec.RUNG5_WALL_FACE_X,
                            spec.RUNG5_WALL_SIZE, spec.RUNG5_WALL_CENTER_Z)
        bad = scenes._wall("wall",
                           spec.RUNG5_WALL_FACE_X + spec.RUNG5_FAULT_SHIFT,
                           spec.RUNG5_WALL_SIZE, spec.RUNG5_WALL_CENTER_Z)
        if good not in xml:                             # pragma: no cover
            raise RuntimeError("rung 5's wall block is not where the fault "
                               "injector expects it")
        xml = xml.replace(good, bad, 1)
    if fault == "slide" and rung == 4:
        # Kill the servos AND lock the hinges (MuJoCo's Euler integrator folds
        # joint damping into the mass matrix, so a huge damping is stable and
        # simply stops the wheel).  The chassis is then dragged kinematically
        # in the step loop, so it covers the commanded distance on dead wheels.
        xml = xml.replace('kv="%.6f"' % scenes.RUNG4_KV, 'kv="0.0"')
        xml = xml.replace('type="hinge" axis="0 1 0"/>',
                          'type="hinge" axis="0 1 0"%s/>' % _LOCK_WHEEL_JOINT)
    return xml


def build(rung, integrator=None, fault=None, run=None):
    """Compile one rung.  Returns ``(mujoco, model, data, xml, compile_s)``."""
    import mujoco
    xml = _xml_for(rung, integrator=integrator, fault=fault, run=run)
    t0 = time.perf_counter()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)      # so the t=0 sample is the authored pose
    return mujoco, model, data, xml, time.perf_counter() - t0


_id = drivers.obj_id                    # kept as a local alias for readers


# --------------------------------------------------------------------------
# the run
# --------------------------------------------------------------------------

def run_rung(rung, integrator=None, fault=None, run=None):
    """Simulate one rung (one RUN of it, for a multi-run cell).

    Returns ``(samples, meta)``.

    ``samples`` is the ladder's shared sample document, so the shared
    reduction consumes this arm's output unmodified.

    THE STEP LOOP IS ``mj_step2`` THEN ``mj_step1``, NOT ``mj_step``, and that
    is load-bearing rather than stylistic.  ``mj_step`` runs the forward pass
    and then integrates; it does not re-run kinematics afterwards, so
    ``data.xpos`` / ``data.site_xpos`` / ``data.sensordata`` still describe the
    state at ``t`` while ``data.time`` already reads ``t+dt``.  A loop that
    steps and then samples therefore writes a sample LABELLED t+dt CARRYING THE
    POSE AT t.

    Measured: rung 2's ``fall_time_abs`` read 0.453524 s under that loop
    against an analytic 0.451524 -- i.e. +2 ms, which is one step of lag
    (+4 ms) against the integrator's own -dt/2 position bias.  With the split
    loop it reads 0.449524 s: -2 ms, the integrator bias alone, with nothing
    from the harness in it.

    Splitting costs nothing -- ``mj_step2`` (integrate, using the forward pass
    already computed at ``t``) followed by ``mj_step1`` (forward at ``t+dt``)
    is exactly one step's work and is bit-identical to ``mj_step``: verified
    over 750 steps on a box + hinge + servo scene, ``max |dqpos| = 0`` for both
    the default Euler integrator and ``implicitfast``.

    It also removes a whole step of latency from the CLOSED-LOOP rungs.  A
    controller reading a one-step-stale sensor brakes ``v*dt`` late, which on
    rung 6 lands directly in the quantity being measured -- injected by the
    harness, and indistinguishable in the result from an engine that brakes
    late.
    """
    rung = int(rung)
    fault = canonical_fault(fault)
    run = dict(run or {})
    t_start = time.perf_counter()
    wall_start = time.time()
    mj, model, data, xml, compile_s = build(rung, integrator=integrator,
                                            fault=fault, run=run)

    dt = scenes.scene_dt(rung)
    steps = int(round(spec.DURATION[rung] / dt))
    if fault == "short_run":
        steps = steps // 2
    if run.get("short"):
        # Rung 9's ``short_b``: replica b runs a CONTRACT-OWNED fraction of the
        # steps.  The fraction is in ``rungs.py`` and reaches here through
        # ``scenes.run_specs``; this arm never picks one.
        steps = int(round(steps * float(run["short"])))
    # CONTRACT.md amendment D.  A stride of 1 for every rung that had one, so
    # nothing about rungs 0-8 moves.
    stride = int(SAMPLE_EVERY.get(rung, 1))

    driver = drivers.make(rung, mj, model, data, fault=fault, run=run)
    series = drivers.blank_series(driver)
    ts = []

    def sample():
        ts.append(float(data.time))
        driver.record(series)

    # Two clocks on purpose.  ``perf_counter`` is monotonic and is what the
    # per-step costs below are measured with; the sample document's ``wall``
    # block is POSIX epoch seconds (CONTRACT.md section 4) because the shared
    # reducer subtracts it against a PROCESS spawn time, and mixing a
    # monotonic counter with an epoch stamp yields a startup figure that is
    # wrong by however long the machine has been up.
    #
    # ``build()`` left the model forward-consistent at t=0 via ``mj_forward``,
    # so this first sample is the pose the scene file authored -- CONTRACT.md
    # section 4 requires rungs 1 and 2 to observe it before the first step.
    sample()
    t_first_step = None
    wall_first_step = None
    for k in range(steps):
        driver.command()                # act on the state just sampled
        mj.mj_step2(model, data)        # integrate: t -> t+dt
        mj.mj_step1(model, data)        # forward at t+dt: pose, sensors, time
        if t_first_step is None:
            t_first_step = time.perf_counter()
            wall_first_step = time.time()
        if (k + 1) % stride == 0:
            sample()
    t_end = time.perf_counter()

    samples = {
        "rung": rung,
        "sim": SIM,
        "dt": dt,
        "steps": steps,
        "sim_time_end": float(data.time),
        "t": ts,
        "wall": {"t_start": wall_start, "t_first_step": wall_first_step,
                 "t_end": time.time()},
    }
    samples.update(series)

    # --- solver facts, read back from the COMPILED model -----------------
    # These are reported rather than asserted: they let a reader see exactly
    # which solver this column ran, and they turn the kv stability margin from
    # a claim in a docstring into a measured field.
    solver_facts = {
        "integrator": int(model.opt.integrator),
        "integrator_name": str(mj.mjtIntegrator(model.opt.integrator)).split(
            ".")[-1],
        "solver": str(mj.mjtSolver(model.opt.solver)).split(".")[-1],
        "cone": str(mj.mjtCone(model.opt.cone)).split(".")[-1],
        "iterations": int(model.opt.iterations),
        "impratio": float(model.opt.impratio),
        "timestep": float(model.opt.timestep),
        "gravity_z": float(model.opt.gravity[2]),
        "nbody": int(model.nbody), "nq": int(model.nq), "nv": int(model.nv),
        "ngeom": int(model.ngeom), "nu": int(model.nu),
        "nsensor": int(model.nsensor),
        "step_loop": "mj_step2+mj_step1 (forward-consistent samples)",
    }
    solver_facts.update(driver.facts())

    meta = {
        "sim": SIM,
        "rung": rung,
        "mujoco_version": __import__("mujoco").__version__,
        "integrator_requested": integrator or "default(Euler)",
        "fault": fault,
        "scene_is_contract_verbatim": True,
        "exit_code": 0,
        "compile_s": compile_s,
        "startup_s": (t_first_step - t_start) if t_first_step else None,
        "step_s": (t_end - t_first_step) if t_first_step else None,
        "total_s": t_end - t_start,
        "steps": steps,
        "us_per_step": ((t_end - t_first_step) / steps * 1e6
                        if t_first_step and steps else None),
        "solver": solver_facts,
        "sample_every": stride,
        "run": run,
        "xml": xml,
    }
    return samples, meta


# --------------------------------------------------------------------------
# multi-run cells -- CONTRACT.md amendment A
# --------------------------------------------------------------------------

def _run_record(spec_run, samples):
    """One RUN of a cell, in the shared sample document's run shape.

    ``pid`` and ``proc_start`` are recorded by the process that DID the run --
    this one -- because the reducer asserts distinctness from the pair and a
    parent's opinion of its child's identity is not the same fact.
    """
    rec = {"tag": spec_run.get("tag"),
           "params": {k: v for k, v in spec_run.items() if k != "tag"},
           "pid": os.getpid(), "proc_start": PROC_START,
           "t": samples["t"], "steps": samples["steps"],
           "sim_time_end": samples["sim_time_end"],
           "wall": samples.get("wall")}
    for key in ("bodies", "robots"):
        if key in samples:
            rec[key] = samples[key]
    return rec


def _replica_path(out_dir, tag):
    return os.path.join(out_dir, "run_%s.json" % tag)


def _spawn_replica(rung, spec_run, fault, out_dir, timeout_s):
    """Run ONE replica in a FRESH INTERPRETER and read its run record back.

    A determinism rung whose replicas share one process measures this arm's
    ability to copy an array.  The child is handed only ``(rung, tag, fault)``
    and re-derives its own parameters from the contract through
    ``scenes.run_specs``, so nothing about the scene travels through the
    command line and a child cannot be asked for a scene the contract does not
    describe.

    JSON, not a pickle and not a formatted dump: ``json`` round-trips float64
    EXACTLY (it writes ``repr``), and rung 9's whole content is whether two
    float64 series are identical.  A ``%.6f`` anywhere on this path would make
    every engine on earth look bitwise deterministic.
    """
    tag = str(spec_run.get("tag"))
    path = _replica_path(out_dir, tag)
    if os.path.exists(path):
        os.remove(path)
    argv = [sys.executable, os.path.join(HERE, "run.py"),
            "--replica", str(int(rung)), "--replica-tag", tag,
            "--replica-out", path]
    if fault:
        argv += ["--fault", str(fault)]
    t0 = time.time()
    try:
        proc = subprocess.run(argv, capture_output=True, text=True,
                              timeout=timeout_s, cwd=HERE)
    except subprocess.TimeoutExpired:
        return None, ("replica %s timed out after %.0f s" % (tag, timeout_s))
    if not os.path.exists(path):
        return None, ("replica %s produced no run record (rc=%s)\n%s\n%s"
                      % (tag, proc.returncode, proc.stdout[-800:],
                         proc.stderr[-800:]))
    with open(path, "r", encoding="utf-8") as fh:
        rec = json.load(fh)
    rec["wall_spawn"] = t0
    rec["exit_code"] = proc.returncode
    return rec, None


def run_cell(rung, out_dir, fault=None, subset=None, timeout_s=600.0,
             integrator=None):
    """One CELL of a multi-run rung.  Returns ``(samples, meta)``.

    Rung 9's replicas are spawned as separate interpreters; rung 11's fleets
    are library calls in this one, which is honest and is what the contract
    asks for -- only rung 9 asserts distinct processes, and it asserts them
    because only rung 9's claim is destroyed by sharing one.
    """
    rung = int(rung)
    if rung == 18:
        return shared.sibling("rung18").run(out_dir, fault=fault,
                                            subset=subset)
    fault = canonical_fault(fault)
    os.makedirs(out_dir, exist_ok=True)
    specs = scenes.run_specs(rung, fault)
    meta = {"sim": SIM, "rung": rung, "fault": fault, "multi_run": True,
            "error": None, "exit_code": 0,
            "mujoco_version": __import__("mujoco").__version__,
            "integrator_requested": integrator or "default(Euler)",
            "scene_is_contract_verbatim": True,
            "run_tags": [s.get("tag") for s in specs],
            "distinct_processes": rung in NEEDS_DISTINCT_PROCESSES,
            "compile_s": 0.0, "step_s": 0.0, "total_s": 0.0, "steps": 0}
    runs, per_run = [], []
    wall_start = time.time()
    for spec_run in specs:
        if rung in NEEDS_DISTINCT_PROCESSES:
            rec, err = _spawn_replica(rung, spec_run, fault, out_dir,
                                      timeout_s)
            if err:
                meta["error"] = meta["error"] or err
                continue
            per_run.append({"tag": spec_run.get("tag"),
                            "pid": rec.get("pid"),
                            "exit_code": rec.get("exit_code")})
            for k in ("compile_s", "step_s", "total_s"):
                meta[k] += float(rec.get("timing", {}).get(k) or 0.0)
            meta["steps"] += int(rec.get("steps") or 0)
            meta.setdefault("solver", rec.get("solver"))
            runs.append({k: v for k, v in rec.items()
                         if k not in ("timing", "solver")})
            continue
        samples, mt = run_rung(rung, integrator=integrator, fault=fault,
                               run=spec_run)
        rec = _run_record(spec_run, samples)
        runs.append(rec)
        per_run.append({"tag": spec_run.get("tag"),
                        "solver": mt.get("solver"),
                        "us_per_step": mt.get("us_per_step")})
        for k in ("compile_s", "step_s", "total_s"):
            meta[k] += float(mt.get(k) or 0.0)
        meta["steps"] += int(mt.get("steps") or 0)
        meta.setdefault("solver", mt.get("solver"))
    meta["runs"] = per_run
    meta["us_per_step"] = (meta["step_s"] / meta["steps"] * 1e6
                           if meta["steps"] else None)
    meta["startup_s"] = 0.0
    if not runs:
        meta["error"] = meta["error"] or "no run of this cell produced samples"
    stamps = [(r.get("wall") or {}).get("t_first_step") for r in runs]
    stamps = [s for s in stamps if s]
    samples = {"rung": rung, "sim": SIM, "fault": fault, "runs": runs,
               "wall": {"t_start": wall_start,
                        "t_first_step": min(stamps) if stamps else None,
                        "t_end": time.time()}}
    return samples, meta


def replica_main(args):
    """Child-process entry point: run ONE replica and write its record.

    Never prints the record to stdout.  A child that mixed its measurement into
    a stream anything else can write to is a child whose measurement can be
    corrupted by a warning.
    """
    rung = int(args.replica)
    fault = canonical_fault(args.fault)
    specs = {str(s.get("tag")): s for s in scenes.run_specs(rung, fault)}
    if args.replica_tag not in specs:
        print("no run %r in rung %d's family (have: %s)"
              % (args.replica_tag, rung, ", ".join(sorted(specs))),
              file=sys.stderr)
        return 2
    spec_run = specs[args.replica_tag]
    samples, mt = run_rung(rung, integrator=args.integrator, fault=fault,
                           run=spec_run)
    rec = _run_record(spec_run, samples)
    rec["timing"] = {k: mt.get(k) for k in ("compile_s", "step_s", "total_s")}
    rec["solver"] = mt.get("solver")
    rec["mujoco_version"] = mt.get("mujoco_version")
    with open(args.replica_out, "w", encoding="utf-8") as fh:
        json.dump(rec, fh)
    return 0


# --------------------------------------------------------------------------
# judging -- delegated entirely to the shared layer
# --------------------------------------------------------------------------

def judge(samples, meta):
    if not ANALYSIS_AVAILABLE:          # pragma: no cover - env dependent
        return None, None
    m = analysis.reduce_samples(samples, exit_code=meta.get("exit_code"))
    checks = [c.as_dict() for c in spec.check_rung(samples["rung"], m)]
    return m, checks


def run_all(rungs=None, integrator=None, fault=None, out_dir=None):
    rungs = list(rungs or spec.RUNGS)
    out_dir = out_dir or os.path.join(RESULTS, "cells")
    rows = []
    for rung in rungs:
        if rung in MULTI_RUN:
            samples, meta = run_cell(rung, os.path.join(out_dir, "rung%d"
                                                        % rung),
                                     fault=fault, integrator=integrator)
            n_samples = sum(len(r.get("t") or []) for r in samples["runs"])
        else:
            samples, meta = run_rung(rung, integrator=integrator, fault=fault)
            n_samples = len(samples["t"])
        m, checks = judge(samples, meta)
        rows.append({"rung": rung, "title": spec.RUNG_TITLE[rung],
                     "meta": {k: v for k, v in meta.items() if k != "xml"},
                     "measured": m, "checks": checks,
                     "n_samples": n_samples})
    return rows


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------

def _fmt(v, nd=6):
    if v is None:
        return "None"
    if isinstance(v, float):
        return ("%%.%df" % nd) % v
    return str(v)


def print_table(rows):
    all_ok = True
    for row in rows:
        checks = row["checks"] or []
        ok = all(c["ok"] for c in checks) if checks else False
        all_ok = all_ok and ok
        mt = row["meta"]
        print("")
        print("rung %d -- %s   [%s]" % (row["rung"], row["title"],
                                        "PASS" if ok else "FAIL"))
        print("  wall: compile %.1f ms | startup %.1f ms | step %.1f ms | "
              "total %.1f ms | %.1f us/step"
              % (mt["compile_s"] * 1e3, (mt["startup_s"] or 0) * 1e3,
                 (mt["step_s"] or 0) * 1e3, mt["total_s"] * 1e3,
                 mt["us_per_step"] or 0.0))
        for c in checks:
            unit = c["unit"]
            rel = " (rel)" if c["rel"] else ""
            print("  %-22s %s %-14s exp %-14s tol %-10s margin %-12s %s"
                  % (c["name"], "OK " if c["ok"] else "RED",
                     _fmt(c["measured"]), _fmt(c["expected"]),
                     _fmt(c["tol"], 4) + rel, _fmt(c["margin"]), unit))
    print("")
    print("OVERALL: %s" % ("PASS" if all_ok else "FAIL"))
    return all_ok


def main(argv=None):
    ap = argparse.ArgumentParser(description="ladder0 MuJoCo arm")
    ap.add_argument("--rung", type=int, action="append",
                    help="run only this rung (repeatable)")
    ap.add_argument("--integrator", default=None,
                    help="override MuJoCo's default Euler integrator "
                         "(e.g. implicitfast) -- sensitivity check")
    ap.add_argument("--fault", choices=sorted(FAULTS),
                    help="inject a physical fault; the named rung must go red")
    ap.add_argument("--out", default=None, help="write the JSON report here")
    ap.add_argument("--write-models", action="store_true",
                    help="regenerate models/rung*.xml and exit")
    # The child-process mode.  Rung 9's replicas MUST come from distinct
    # interpreters or the rung measures this arm rather than the engine, so the
    # cell re-invokes this file once per replica.  The child receives a rung
    # and a TAG -- never a scene number -- and re-derives everything else from
    # the contract.
    ap.add_argument("--replica", type=int, default=None,
                    help=argparse.SUPPRESS)
    ap.add_argument("--replica-tag", default=None, help=argparse.SUPPRESS)
    ap.add_argument("--replica-out", default=None, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    if args.replica is not None:
        if not (args.replica_tag and args.replica_out):
            ap.error("--replica needs --replica-tag and --replica-out")
        return replica_main(args)

    if args.write_models:
        for p in scenes.write_all():
            print(p)
        return 0

    # Time the import BEFORE anything else touches mujoco -- `available()`
    # imports it, so timing it afterwards measures a cache hit and reports
    # 0.0 ms.  (It did, until this was fixed: the arm's own startup cost is
    # the number the ladder's "how cheap is the cheap arm" question is about,
    # so a zero there would have understated it by the entire import.)
    _cold = "mujoco" not in sys.modules
    t_interp = time.perf_counter()
    import mujoco                                   # noqa: F401
    import_s = time.perf_counter() - t_interp if _cold else None

    ok, detail = available()
    if not ok:
        print("mujoco unavailable: %s" % detail, file=sys.stderr)
        return 2

    scenes.write_all()
    rows = run_all(args.rung, integrator=args.integrator, fault=args.fault)

    report = {
        "scene_is_contract_verbatim": True,
        "sim": SIM,
        "mujoco_version": mujoco.__version__,
        "mujoco_lib": mujoco.mj_versionString(),
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "machine": platform.node(),
        # ``shared`` is the module object this arm loaded BY PATH.  Never
        # ``__import__("shared")``: that resolves through ``sys.path`` and, run
        # as a script, builds a SECOND module object for the same file -- two
        # contracts in one process, which is the split CONTRACT.md section 2
        # exists to prevent.
        "spec_module": shared.SPEC_MODULE,
        "integrator_requested": args.integrator or "default(Euler)",
        "fault": args.fault,
        "import_mujoco_s": import_s,
        "scene_facts": scenes.scene_facts(),
        "rows": rows,
    }
    print("MuJoCo %s (lib %s), python %s, contract module %r"
          % (report["mujoco_version"], report["mujoco_lib"],
             report["python"], report["spec_module"]))
    print("import mujoco: %s | integrator: %s | fault: %s"
          % ("%.1f ms" % (import_s * 1e3) if import_s is not None
             else "already imported (not timed)",
             report["integrator_requested"], args.fault))
    all_ok = print_table(rows)
    report["ok"] = all_ok

    if not os.path.isdir(RESULTS):
        os.makedirs(RESULTS)
    name = args.out or os.path.join(
        RESULTS, "mujoco%s%s%s.json"
        % ("_" + args.integrator if args.integrator else "",
           "_fault_" + args.fault if args.fault else "", ""))
    with open(name, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(report, fh, indent=2, sort_keys=True)
    print("wrote %s" % name)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
