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

"""ladder0_wprobe -- the upstream-Webots arm's recorder.

It DRIVES and it RECORDS.  It does not judge: no threshold, no expected value
and no verdict appears anywhere in this file.  The reduction to physical
quantities is ``ladder0/analysis.py`` and the ground truth is
``ladder0/rungs.py``, both shared with the other two arms, so no arm gets to
define for itself what "the fall time" means (CONTRACT.md section 1).

Every scene constant and every commanded rate is imported from ``rungs.py``
rather than copied, so a change to the scene can never leave this driver
commanding the old rate against the new geometry.

Three recording decisions are load-bearing:

* **Rungs 1 and 2 take their first sample BEFORE the first ``step()``**, read
  from the scene tree rather than from a sensor.  That is the only chance to
  observe the pose the world file authored, and rung 2's ``spawn_z`` assertion
  is exactly that observation; sampling after a step would hide up to one step
  of free-run behind the measurement.  Rungs 3 and 4 start at t = dt because a
  Webots sensor has no valid value until a step has completed -- so they open
  ALL of their series one step in rather than padding one with a number nobody
  measured (CONTRACT.md section 4).
* **Times come from ``robot.getTime()``**, the simulated clock, never from a
  step counter multiplied by dt.  If an engine's clock and its step count ever
  disagree, that disagreement is a finding, and it is visible only because the
  two are recorded independently (``t`` and ``steps``).
* **Wheel angles come from each wheel's OWN PositionSensor**, never from the
  commanded rate.  A chassis sliding the right distance on wheels that never
  turned is the defect rung 4 exists to catch, and it is invisible to any
  measurement that trusts the command.

MULTI-RUN RUNGS (9 and 11, CONTRACT.md amendment A)
---------------------------------------------------
A cell of rung 9 or 11 is several runs of one contract-owned scene family, and
**each run is its own engine launch**, so this file runs once per run.  Three
things follow:

* ``pid`` and ``proc_start`` are recorded HERE, by the process that did the
  stepping.  ``distinct_processes`` is the one assertion on this ladder that
  checks the ARM rather than the engine, and it can only be honest if the
  provenance comes from inside the run: an arm that replayed a cached document,
  or ran its replicas in one interpreter, passes every determinism check ever
  written.
* The per-run values -- the tag, the sample stride, the short fraction --
  arrive in the **environment** (``LADDER0_TAG`` / ``LADDER0_STRIDE`` /
  ``LADDER0_SHORT``) and not in ``controllerArgs``, because rung 9's replicas
  ``a`` and ``b`` are deliberately the SAME world file and anything that
  reached them through the scene would make the two files differ.
* The stride is the CONTRACT's (``rungs.RUNG9_SAMPLE_EVERY`` /
  ``RUNG11_SAMPLE_EVERY``, carried by the arm); this driver never picks one,
  and it records the true simulated ``t`` of each sample rather than
  reconstructing a grid from the step number.

FAULTS (``$LADDER0_FAULT``, CONTRACT.md section 6)
--------------------------------------------------
``short_run`` (rung 0) stops halfway.  ``ignore_zero`` (rung 3) keeps driving
after the command switches to zero -- the servo that ignores its input.
``slide`` (rung 4) commands every wheel to ZERO and drags the chassis at
exactly ``rungs.rolling_speed(rungs.RUNG4_OMEGA_CMD)``, which must leave
``distance`` green and take ``rolling_consistency`` red; that asymmetry is the
entire argument for asserting two quantities on rung 4.  ``short_b`` (rung 9)
is the same idea one level up: replica ``b`` alone completes
``rungs.RUNG9_FAULT_SHORT`` of the steps, so the two replicas agree over the
overlap and differ only in LENGTH -- which is how a truncated replica passes a
determinism check that compares samples.  The remaining faults are scene-level
and live in ``worldgen.py``.

Faults are injected into the RUN, never into the measurement: the recording
path below is byte-identical in a faulted and an unfaulted run.

Output: a JSON sample document at ``$LADDER0_OUT``.  The final stdout line is
``LADDER0_DONE steps=<n> out=<path>``, so a run that produced no file can be
told apart from a run that produced no controller at all.
"""

import importlib.util
import json
import math
import os
import sys
import time

T_START = time.time()

from controller import Robot, Supervisor  # noqa: E402  (after T_START)

HERE = os.path.abspath(os.path.dirname(__file__))
# .../ladder0/webots/controllers/ladder0_wprobe -> .../ladder0
LADDER0 = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir, os.pardir))


def _load_rungs():
    """Import the shared contract by path.

    By path rather than by ``sys.path`` insertion so this arm can never shadow,
    or be shadowed by, a same-named module in another lane's tree.
    """
    path = os.path.join(LADDER0, "rungs.py")
    sp = importlib.util.spec_from_file_location("ladder0_rungs", path)
    mod = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(mod)
    return mod


rungs = _load_rungs()
FAULT = os.environ.get("LADDER0_FAULT", "none") or "none"


def arg(name, default=None):
    prefix = "--%s=" % name
    for a in sys.argv[1:]:
        if a.startswith(prefix):
            return a[len(prefix):]
    return default


def clamp01(v):
    return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)


def run_driver(robot, dt_ms, rung, idx, omega=None):
    """A rung-7 or rung-11 rover: drive at MY command, and NEVER end the run.

    It steps until the engine stops it rather than exhausting a step budget of
    its own.  Measured on this arm: when the drivers ran out their own budget
    and their processes exited, upstream ended the simulation and cut the
    recorder off before it could write anything -- all five heartbeats stopped
    at t = 2.50 s of a 3.00 s run and no sample document was produced.  The
    recorder alone owns the run's length.

    ``crosstalk`` is the rung-7 fault: every rover is given the MIDDLE rover's
    command instead of its own.  It is the failure the rung's five distinct
    commands exist to expose -- with one shared command the fleet still drives
    straight, still stays in its lanes and still never touches, so only the
    two per-robot command checks may go red.

    ``omega`` is rung 11's: its commands CYCLE ``rungs.RUNG11_OMEGA`` over up
    to sixteen rovers, so the scene writes each robot's rate into its own
    ``controllerArgs`` (which is also where that rung's ``stalled_robot`` fault
    lives) instead of this driver indexing a five-entry tuple.  Rung 7 passes
    nothing and keeps looking its own command up, unchanged.
    """
    if omega is None:
        omega = rungs.RUNG7_OMEGA[idx]
    if FAULT == "crosstalk":
        omega = rungs.RUNG7_OMEGA[len(rungs.RUNG7_OMEGA) // 2]
    for tag in rungs.WHEEL_TAGS:
        m = robot.getDevice("wheel_%s" % tag)
        if m is None:
            continue
        m.setPosition(float("inf"))
        m.setVelocity(omega)
    while robot.step(dt_ms) != -1:
        pass
    print("LADDER0_DRIVER idx=%d rung=%d omega=%.3f stopped by engine at t=%.3f"
          % (idx, rung, omega, robot.getTime()))
    sys.stdout.flush()
    return


def rel_hinge_angle(r_parent, r_child):
    """Angle of ``r_child`` about the PARENT's +Y axis, in radians.

    Both arguments are the 3x3 world orientations Webots reports for a Solid,
    row-major.  ``R_rel = R_parent^T R_child`` is a rotation about y for a
    wheel on a y-axis hinge, and a rotation by theta about +y has
    ``R[0][2] = sin`` and ``R[0][0] = cos``.

    This is how rung 7 reads twenty wheel angles from ONE controller.  Upstream
    Webots, exactly like OmniSim, lets a robot read only its own devices, so a
    supervisor cannot ask a sibling rover for its PositionSensor -- but the
    wheel's pose is a scene fact and the supervisor can always see that.
    Taking it RELATIVE to the chassis is what makes it the hinge's angle rather
    than the hinge's angle plus whatever the chassis did.

    Wrapped to (-pi, pi] by ``atan2``, which the shared reducer explicitly
    tolerates: it folds consecutive differences before accumulating, and at
    6 rad/s a 4 ms step is 0.024 rad, far inside the fold.
    """
    r02 = sum(r_parent[k * 3 + 0] * r_child[k * 3 + 2] for k in range(3))
    r00 = sum(r_parent[k * 3 + 0] * r_child[k * 3 + 0] for k in range(3))
    return math.atan2(r02, r00)


def world_info_facts(robot):
    """Read the loaded ``WorldInfo`` back out of the scene tree.

    THE VALUE THE ENGINE USED, not the value the file asked for.  It matters
    for exactly one field and only on rung 9: above 1, upstream splits ODE's
    island solve across threads, and a parallel island solve is the class of
    mechanism that rung's GPU refutation is an instance of.  A determinism row
    has to say which configuration produced it, and reading it back is the only
    spelling of that which cannot be wrong about its own scene.

    Best effort by design: a supervisor tree walk that fails must not cost the
    run its measurement, so this returns what it could read and nothing else.
    """
    out = {}
    try:
        children = robot.getRoot().getField("children")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node is None or node.getTypeName() != "WorldInfo":
                continue
            for name in ("optimalThreadCount", "randomSeed"):
                f = node.getField(name)
                if f is not None:
                    out[name] = f.getSFInt32()
            f = node.getField("basicTimeStep")
            if f is not None:
                out["basicTimeStep"] = f.getSFFloat()
            break
    except Exception as exc:                       # noqa: BLE001
        out["error"] = repr(exc)
    return out


def main():
    rung = int(arg("rung", "0"))
    role = arg("role", "recorder")
    idx = int(arg("idx", "0"))
    omega_arg = arg("omega")
    n_fleet = int(arg("n", "0"))
    duration = rungs.DURATION[rung]
    # Per-RUN values (amendment A).  They arrive in the ENVIRONMENT because
    # rung 9's replicas a and b are the same world file on purpose -- see the
    # module docstring.  The stride is the contract's, carried by the arm; this
    # driver never picks one.
    # ``run_tag``, not ``tag``: the per-rung wiring below rebinds ``tag`` as a
    # WHEEL tag, and a run tag silently overwritten by "rr" is the kind of
    # provenance bug that produces a document nothing downstream can question.
    run_tag = os.environ.get("LADDER0_TAG") or None
    stride = max(1, int(os.environ.get("LADDER0_STRIDE", "1") or 1))
    short = float(os.environ.get("LADDER0_SHORT", "1.0") or 1.0)

    # A rung-7 or rung-11 rover is a plain Robot: it drives and never records,
    # so it must not ask for a Supervisor it was not granted.
    robot = Robot() if role == "driver" else Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0
    n_steps = int(round(duration / dt))
    if FAULT == "short_run":
        n_steps = n_steps // 2
    if short != 1.0:
        n_steps = max(1, int(round(n_steps * short)))

    if role == "driver":
        return run_driver(robot, dt_ms, rung, idx,
                          None if omega_arg is None else float(omega_arg))

    rec = {
        "rung": rung,
        "sim": "webots",
        "dt": dt,
        "basic_time_step_ms": dt_ms,
        "requested_steps": n_steps,
        "fault": FAULT,
        "steps": 0,
        "t": [],
        "notes": [],
        # PROVENANCE, recorded by the process that does the stepping.  Rung 9's
        # ``distinct_processes`` reads the (pid, proc_start) PAIR, because a pid
        # is reused within a session.
        "pid": os.getpid(),
        "proc_start": T_START,
        "sample_every": stride,
        "world_info": world_info_facts(robot),
    }
    if run_tag is not None:
        rec["tag"] = run_tag

    # ---- per-rung wiring -------------------------------------------------
    box = None
    motors, sensors = {}, {}
    tags = rungs.WHEEL_TAGS

    if rung in (1, 2):
        box = robot.getFromDef("BOX")
        rec["box_z"] = []
        if box is None:
            rec["notes"].append("DEF BOX not found in the scene tree")
    elif rung == 3:
        motors["j0"] = robot.getDevice("j0")
        sensors["j0"] = robot.getDevice("j0_sensor")
        rec["joint_q"] = []
    elif rung == 4:
        rec["body_x"], rec["body_y"], rec["body_z"] = [], [], []
        rec["wheel_q"] = {tag: [] for tag in tags}
        for tag in tags:
            motors[tag] = robot.getDevice("wheel_%s" % tag)
            sensors[tag] = robot.getDevice("wheel_%s_sensor" % tag)
    elif rung == 5:
        sensors["ds"] = robot.getDevice("ds")
        rec["body_x"], rec["range"] = [], []
    elif rung == 6:
        sensors["ds"] = robot.getDevice("ds")
        rec["body_x"], rec["body_y"], rec["body_z"] = [], [], []
        rec["range"] = []
        rec["wheel_q"] = {tag: [] for tag in tags}
        for tag in tags:
            motors[tag] = robot.getDevice("wheel_%s" % tag)
            sensors[tag] = robot.getDevice("wheel_%s_sensor" % tag)
    elif rung == 7:
        rec["robots"] = {tag: {"x": [], "y": [], "z": [],
                               "wheel_q": {w: [] for w in tags}}
                         for tag in rungs.RUNG7_TAGS}
    elif rung == 8:
        rec["part_x"], rec["part_y"], rec["part_z"] = [], [], []
        rec["wrist_x"], rec["wrist_y"], rec["wrist_z"] = [], [], []
        for name in ("traverse", "lift", "pad_l", "pad_r"):
            motors[name] = robot.getDevice(name)
            sensors[name] = robot.getDevice("%s_sensor" % name)
    elif rung == 9:
        rec["bodies"] = {}
        for btag, _bx, _by in rungs.rung9_pile_xy():
            rec["bodies"][btag] = {"x": [], "y": [], "z": []}
        rec["bodies"]["drop"] = {"x": [], "y": [], "z": []}
    elif rung == 11:
        rec["n"] = n_fleet
        rec["robots"] = {"r%02d" % i: {"x": [], "y": [], "z": [],
                                       "wheel_q": {w: [] for w in tags}}
                         for i in range(n_fleet)}

    for name, dev in sensors.items():
        if dev is None:
            rec["notes"].append("device missing: sensor %s" % name)
        else:
            dev.enable(dt_ms)
    for name, dev in motors.items():
        if dev is None:
            rec["notes"].append("device missing: motor %s" % name)
        elif rung == 8:
            # Rung 8 is the one rung driven by POSITION, not rate: the gantry
            # follows a commanded schedule and the pads are held against the
            # part by a servo saturated at its maxForce.  Releasing the
            # position servo with setPosition(inf) here would leave the whole
            # stage limp.
            dev.setVelocity(dev.getMaxVelocity())
        else:
            # Velocity control: an infinite position target releases the
            # position servo so setVelocity commands the rate directly.
            dev.setPosition(float("inf"))
            dev.setVelocity(0.0)

    self_node = robot.getSelf() if rung in (4, 5, 6) else None
    if rung in (4, 5, 6) and self_node is None:
        rec["notes"].append("getSelf() returned None; body pose unavailable")

    # Rung 5 writes its carrier's pose instead of driving it (see worldgen);
    # rung 7's observer resolves the fleet it watches; rung 8 watches two
    # bodies it does not own.
    sweep_field = (self_node.getField("translation")
                   if (rung == 5 and self_node is not None) else None)
    fleet, pile, part, wrist = [], [], None, None
    if rung in (7, 11):
        # Rung 11 IS rung 7's machinery at four fleet sizes: one Robot per
        # rover with its own driver process, and one body-less supervisor that
        # reads every pose and every wheel ORIENTATION out of the scene tree,
        # because upstream (like OmniSim) lets a robot read only its own
        # devices.  The tags differ -- rung 7's ``r0``, rung 11's ``r%02d`` --
        # and the contract owns both, so they are read from it rather than
        # spelled here.
        fleet_tags = (rungs.RUNG7_TAGS if rung == 7
                      else tuple("r%02d" % i for i in range(n_fleet)))
        for i, rtag in enumerate(fleet_tags):
            body = robot.getFromDef("ROVER%d" % i)
            wheels = [robot.getFromDef("R%d_WHEEL_%s" % (i, w.upper()))
                      for w in tags]
            if body is None or any(w is None for w in wheels):
                rec["notes"].append("rung %d: DEF ROVER%d or a wheel of it is "
                                    "not in the scene tree" % (rung, i))
            fleet.append((rtag, body, wheels))
    elif rung == 9:
        for btag, _bx, _by in rungs.rung9_pile_xy():
            node = robot.getFromDef(btag.upper())
            if node is None:
                rec["notes"].append("rung 9: DEF %s is not in the scene tree"
                                    % btag.upper())
            pile.append((btag, node))
        drop = robot.getFromDef("DROP")
        if drop is None:
            rec["notes"].append("rung 9: DEF DROP is not in the scene tree")
        pile.append(("drop", drop))
    elif rung == 8:
        part = robot.getFromDef("PART")
        wrist = robot.getFromDef("WRIST")
        for nm, nd in (("PART", part), ("WRIST", wrist)):
            if nd is None:
                rec["notes"].append("DEF %s not found in the scene tree" % nm)
    drag_field = (self_node.getField("translation")
                  if (rung == 4 and FAULT == "slide"
                      and self_node is not None) else None)
    drag_v = rungs.rolling_speed(rungs.RUNG4_OMEGA_CMD)
    drag_z = rungs.ROBOT_Z
    nan3 = [float("nan")] * 3

    # ---- sampling --------------------------------------------------------
    def sample_supervisor():
        """Quantities the supervisor reads from the scene tree -- no device,
        and valid before the first step."""
        if rung in (1, 2):
            rec["box_z"].append(box.getPosition()[2] if box else float("nan"))
        elif rung in (4, 6):
            p = self_node.getPosition() if self_node else nan3
            rec["body_x"].append(p[0])
            rec["body_y"].append(p[1])
            rec["body_z"].append(p[2])
        elif rung == 5:
            p = self_node.getPosition() if self_node else nan3
            rec["body_x"].append(p[0])
        elif rung == 9:
            for btag, node in pile:
                slot = rec["bodies"][btag]
                p = node.getPosition() if node is not None else nan3
                slot["x"].append(p[0])
                slot["y"].append(p[1])
                slot["z"].append(p[2])
        elif rung in (7, 11):
            for tag, body, wheels in fleet:
                slot = rec["robots"][tag]
                p = body.getPosition() if body is not None else nan3
                slot["x"].append(p[0])
                slot["y"].append(p[1])
                slot["z"].append(p[2])
                # The wheel angle is read as a POSE, relative to its own
                # chassis -- a supervisor may not read a sibling robot's
                # PositionSensor, on upstream as on OmniSim.
                chassis = (body.getOrientation()
                           if body is not None else None)
                for w, wheel in zip(tags, wheels):
                    if chassis is None or wheel is None:
                        slot["wheel_q"][w].append(float("nan"))
                    else:
                        slot["wheel_q"][w].append(
                            rel_hinge_angle(chassis, wheel.getOrientation()))
        elif rung == 8:
            p = part.getPosition() if part is not None else nan3
            w = wrist.getPosition() if wrist is not None else nan3
            rec["part_x"].append(p[0])
            rec["part_y"].append(p[1])
            rec["part_z"].append(p[2])
            rec["wrist_x"].append(w[0])
            rec["wrist_y"].append(w[1])
            rec["wrist_z"].append(w[2])

    def sample_devices():
        """Quantities that need a completed step to have a value."""
        if rung == 3:
            s = sensors.get("j0")
            rec["joint_q"].append(s.getValue() if s else float("nan"))
        elif rung in (4, 6):
            for tag in tags:
                s = sensors.get(tag)
                rec["wheel_q"][tag].append(s.getValue() if s else float("nan"))
        if rung in (5, 6):
            s = sensors.get("ds")
            rec["range"].append(s.getValue() if s else float("nan"))

    def drive(t_now):
        """Apply the commanded rate for the current simulated time."""
        if rung == 3:
            m = motors.get("j0")
            if m is None:
                return
            driving = (t_now < rungs.RUNG3_ZERO_AT) or FAULT == "ignore_zero"
            m.setVelocity(rungs.RUNG3_OMEGA_CMD if driving else 0.0)
        elif rung == 4:
            cmd = 0.0 if FAULT == "slide" else rungs.RUNG4_OMEGA_CMD
            for tag in tags:
                m = motors.get(tag)
                if m is not None:
                    m.setVelocity(cmd)
            if drag_field is not None:
                # Kinematic drag: the chassis is teleported along a ramp at
                # exactly the speed rolling wheels would have produced, while
                # the wheels are commanded to stand still.
                drag_field.setSFVec3f([drag_v * t_now, 0.0, drag_z])
                self_node.resetPhysics()
        elif rung == 5:
            # The carrier is KINEMATIC: its pose is written, not simulated, so
            # the sensor's position at every instant is a scene fact.  The
            # schedule is the contract's own rung5_x_cmd -- dwell, sweep, park
            # -- so all three arms move the sensor identically.
            if sweep_field is not None:
                sweep_field.setSFVec3f([rungs.rung5_x_cmd(t_now), 0.0,
                                        rungs.RUNG5_SENSOR_Z])
        elif rung == 6:
            # Drive until the sensor first reads below the threshold, then
            # command every wheel to zero and leave it there.  ``late_stop``
            # acts on HALF the threshold: the sensor is read, believed and
            # correct, and the rover simply stops too late.
            trip = (rungs.RUNG6_STOP_GAP / 2.0 if FAULT == "late_stop"
                    else rungs.RUNG6_STOP_GAP)
            s = sensors.get("ds")
            # A Webots sensor has NO valid value until a step has completed,
            # and an unread DistanceSensor reports 0.0 -- which is below every
            # threshold.  Without this gate the rover "arrives" at t=0 and
            # never moves, and the failure looks like a dead motor.
            ready = rec["steps"] > 0
            if ready and s is not None and not state["stopped"] \
                    and s.getValue() < trip:
                state["stopped"] = True
                state["stop_t"] = t_now
            cmd = 0.0 if state["stopped"] else rungs.RUNG4_OMEGA_CMD
            for tag in tags:
                m = motors.get(tag)
                if m is not None:
                    m.setVelocity(cmd)
        elif rung == 8:
            # Settle -> close -> lift -> traverse -> hold, every segment's rate
            # fixed by the contract's own times and speeds.  The pads are
            # commanded to 0 (fully shut); the part blocks them at
            # RUNG8_PAD_TOUCH_Y, which is what holds the servo saturated at
            # exactly RUNG8_GRIP_N per pad.
            close = clamp01((t_now - rungs.RUNG8_T_SETTLE)
                            / (rungs.RUNG8_T_CLOSE - rungs.RUNG8_T_SETTLE))
            # A slider's position is a DISPLACEMENT from the authored pose, so
            # 0 is open and closing travels inward -- negative for the pad
            # authored at +y, positive for its mate.
            pad = rungs.RUNG8_PAD_OPEN_Y * close
            lift = rungs.RUNG8_LIFT_H * clamp01(
                (t_now - rungs.RUNG8_T_CLOSE)
                / (rungs.RUNG8_T_LIFT - rungs.RUNG8_T_CLOSE))
            trav = rungs.RUNG8_TRAVERSE_X * clamp01(
                (t_now - rungs.RUNG8_T_LIFT)
                / (rungs.RUNG8_T_TRAV - rungs.RUNG8_T_LIFT))
            for name, target in (("traverse", trav), ("lift", lift),
                                 ("pad_l", -pad), ("pad_r", pad)):
                m = motors.get(name)
                if m is not None:
                    m.setPosition(target)

    state = {"stopped": False, "stop_t": None}
    t_first_step = None
    if rung in (1, 2):
        rec["t"].append(robot.getTime())
        sample_supervisor()

    for _ in range(n_steps):
        drive(robot.getTime())
        if robot.step(dt_ms) == -1:
            rec["notes"].append("engine ended the run early (step returned -1)")
            break
        rec["steps"] += 1
        if t_first_step is None:
            t_first_step = time.time()
        # Contract-owned decimation (amendment D).  ``stride`` is 1 on every
        # rung that does not declare one, so this is the old every-step loop
        # there.  The recorded ``t`` is the true simulated time of the sample,
        # never a grid reconstructed from the step number: if an engine's clock
        # and its step count ever disagree, that disagreement is a finding, and
        # it is visible only because the two are recorded independently.
        if (rec["steps"] % stride) == 0:
            rec["t"].append(robot.getTime())
            sample_supervisor()
            sample_devices()

    rec["sim_time_end"] = robot.getTime()
    # CONTEXT, never a measurement: when the DRIVER believed it had arrived.
    # The reducer re-derives the trigger from the recorded range series,
    # because the driver's threshold is a command and the crossing is an
    # observation -- if the two ever disagree, that disagreement is the
    # finding, and it is visible only because both are kept.
    if rung == 6:
        rec["driver_stop_t"] = state["stop_t"]
    rec["wall"] = {
        "t_start": T_START,
        "t_first_step": t_first_step,
        "t_end": time.time(),
    }

    out = os.environ.get("LADDER0_OUT") or os.path.join(
        LADDER0, "webots", "results", "rung%d" % rung, "samples.json")
    try:
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(rec, f)
    except Exception as exc:                       # noqa: BLE001
        print("LADDER0_WRITE_FAILED %r" % (exc,))
        out = "<unwritten>"

    print("LADDER0_DONE steps=%d out=%s" % (rec["steps"], out))
    sys.stdout.flush()

    # END THE RUN.  A Webots controller that returns does NOT stop the engine:
    # the simulation keeps stepping with nothing driving it, and the process
    # lives until something outside kills it.  Measured here before this call
    # existed: every cell ran to the 240 s `timeout` and came back rc=124, for
    # a simulation whose own stepping took 0.05-0.26 s.  Rung 0's `exit_code`
    # assertion is what caught it -- a clean exit really is part of the rung.
    #
    # The step after the request is what flushes it to the engine; without it
    # the controller can exit first and the quit is never delivered.
    robot.simulationQuit(0)
    robot.step(dt_ms)


main()
