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

"""ladder0_probe -- the OmniSim arm's driver/recorder for all five rungs.

It RECORDS and it DRIVES.  It does not judge: no threshold, no expected value
and no verdict appears in this file.  Reduction to physical quantities is
``ladder0/analysis.py`` and the ground truth is ``ladder0/rungs.py``, so every
arm is scored by the same code from the same numbers.

The import is ``from controller import ...`` rather than the preferred
``from omnisim import ...`` on purpose: this file is meant to be readable as
the reference implementation by the arms running other simulators, and
``controller`` is the only spelling all of them export.

Scene constants and commanded rates come from ``rungs.py``, imported by
absolute path, so a scene change cannot leave the driver commanding the old
rate.

``--fault=<name>`` drives the self-test's live red proofs.  A fault is
injected into the RUN, never into the measurement -- see ``worldgen.FAULTS``.

Output: JSON at ``$LADDER0_OUT`` (falling back to the ladder's ``results/``).
The last stdout line is ``LADDER0_DONE steps=<n> out=<path>``, so a run that
produced no file can still be told apart from one that produced no controller
at all.
"""

import importlib.util
import json
import math
import os
import sys
import time

T_START = time.time()

HERE = os.path.abspath(os.path.dirname(__file__))


def _breadcrumb(stage, extra=""):
    """Append one line to ``$LADDER0_OUT.trace``.

    ``omnisim-bin.exe`` is a GUI-subsystem binary on Windows: a controller's
    stdout and stderr go nowhere, ``--stdout --stderr`` forwards nothing, and
    the engine log records a dead controller as ``Terminating.`` -- the same
    line a healthy one produces.  Without a file-based trace there is no way
    to tell "the controller crashed on import" from "the controller was never
    launched", and those need opposite fixes.
    """
    out = os.environ.get("LADDER0_OUT") or os.path.join(HERE, "_no_env_out")
    try:
        d = os.path.dirname(out)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(out + ".trace", "a", encoding="utf-8") as f:
            f.write("%.3f %s %s\n" % (time.time() - T_START, stage, extra))
    except OSError:
        pass


_breadcrumb("module-start", "python=%s cwd=%s"
            % (sys.version.split()[0], os.getcwd()))

from controller import Supervisor  # noqa: E402  (after the breadcrumb)

_breadcrumb("controller-imported")
# .../ladder0/omnisim/controllers/ladder0_probe -> .../ladder0
LADDER0 = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir, os.pardir))


def _load(name, filename):
    path = os.path.join(LADDER0, filename)
    sp = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(mod)
    return mod


rungs = _load("ladder0_rungs", "rungs.py")
_breadcrumb("rungs-loaded")

# The arm's own engine facts (rung 8's actuator).  Loaded by path under an
# arm-qualified name for the same reason everything else here is: this file
# runs inside a process the engine started, whose sys.path is not ours.
engine_facts = _load("ladder0_omnisim_engine_facts",
                     os.path.join("omnisim", "engine_facts.py"))
_breadcrumb("engine-facts-loaded")


def _wheel_angle(node):
    """A wheel's rotation about its own axle, read from its WORLD orientation.

    Rung 7 needs four wheel angles from each of five robots, and a supervisor
    may not read a sibling robot's PositionSensor -- so the angle comes from
    the scene graph instead.  For a wheel whose axle is +Y, the world rotation
    matrix is [[c,0,s],[0,1,0],[-s,0,c]] and the angle is atan2(R02, R00).

    The value is WRAPPED to (-pi, pi], which the reducer already handles: it
    differences consecutive samples and folds each difference before
    accumulating.  At 6 rad/s and dt = 4 ms one step is 0.024 rad, two orders
    of magnitude inside the fold.

    This is arguably the better measurement of the two.  A position sensor
    reports the servo's own account of the joint; this reports where the wheel
    actually is.
    """
    if node is None:
        return float("nan")
    r = node.getOrientation()
    if not r or len(r) < 9:
        return float("nan")
    return math.atan2(r[2], r[0])


def arg(name, default=None):
    pref = "--%s=" % name
    for a in sys.argv[1:]:
        if a.startswith(pref):
            return a[len(pref):]
    return default


def main():
    rung = int(arg("rung", "0"))
    fault = arg("fault", "none")
    duration = rungs.DURATION[rung]
    # Multi-run rungs (CONTRACT.md amendment A): the arm passes the run's tag
    # and the fraction of the run this replica is to complete.  Both are the
    # CONTRACT's -- worldgen.run_specs reads them from rungs.py -- and the
    # driver only carries them.
    #
    # THE SCENE OWNS ``--rung`` / ``--fault`` / ``--n`` and they arrive in the
    # world file's controllerArgs.  THE RUN owns the tag, the stride and the
    # short fraction, and they arrive in the ENVIRONMENT -- because rung 9's
    # replicas a and b are deliberately the SAME world file, so a per-run value
    # cannot come from the scene without making the two files differ, which is
    # the one thing that rung must rule out.
    tag = arg("tag", os.environ.get("LADDER0_TAG"))
    short = float(arg("short", os.environ.get("LADDER0_SHORT", "1.0")))
    stride = int(arg("stride", os.environ.get("LADDER0_STRIDE", "1")))
    n_fleet = int(arg("n", "0"))

    _breadcrumb("main-entered", "rung=%d fault=%s tag=%s" % (rung, fault, tag))
    robot = Supervisor()
    _breadcrumb("supervisor-constructed")
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    rec = {"rung": rung, "sim": "omnisim", "fault": fault, "dt": dt,
           "t": [], "steps": 0, "basic_time_step_ms": dt_ms}
    if tag is not None:
        rec["tag"] = tag
    # PROVENANCE for the determinism rung.  Recorded by the process that did
    # the stepping, not by the launcher: an arm that replayed a cached result,
    # or ran its replicas inside one interpreter, would otherwise pass every
    # determinism check ever written.
    rec["pid"] = os.getpid()
    rec["proc_start"] = T_START
    t_first_step = None

    box = None
    ds = None
    fleet = []                          # rungs 7, 11: [(tag, node, wheels)]
    pile = []                           # rung 9: [(tag, node)]
    part = wrist = None
    motors, sensors = {}, {}
    if rung in (1, 2):
        box = robot.getFromDef("BOX")
        rec["box_z"] = []
        if box is None:
            rec["error"] = "DEF BOX not found"
    elif rung == 3:
        motors["j0"] = robot.getDevice("j0")
        sensors["j0"] = robot.getDevice("j0_sensor")
        rec["joint_q"] = []
    elif rung in (4, 6):
        for tag in rungs.WHEEL_TAGS:
            motors[tag] = robot.getDevice("wheel_%s" % tag)
            sensors[tag] = robot.getDevice("wheel_%s_sensor" % tag)
        rec["body_x"], rec["body_y"], rec["body_z"] = [], [], []
        rec["wheel_q"] = {t: [] for t in rungs.WHEEL_TAGS}
        if rung == 6:
            ds = robot.getDevice("ds")
            rec["range"] = []
            if ds is None:
                rec["error"] = "DistanceSensor 'ds' not found"
    elif rung == 5:
        ds = robot.getDevice("ds")
        rec["body_x"], rec["range"] = [], []
        if ds is None:
            rec["error"] = "DistanceSensor 'ds' not found"
    elif rung == 7:
        rec["robots"] = {}
        for tag in rungs.RUNG7_TAGS:
            up = tag.upper()
            node = robot.getFromDef(up)
            wheels = {w: robot.getFromDef("WHEEL_%s_%s" % (up, w.upper()))
                      for w in rungs.WHEEL_TAGS}
            fleet.append((tag, node, wheels))
            rec["robots"][tag] = {
                "x": [], "y": [], "z": [],
                "wheel_q": {w: [] for w in rungs.WHEEL_TAGS}}
        absent = [tag for tag, node, _w in fleet if node is None]
        if absent:
            rec["error"] = "fleet members not found: %s" % ",".join(absent)
    elif rung == 8:
        for name in ("traverse", "lift", "finger_l", "finger_r"):
            motors[name] = robot.getDevice(name)
            sensors[name] = robot.getDevice("%s_sensor" % name)
        part = robot.getFromDef("PART")
        wrist = robot.getFromDef("WRIST")
        rec["part_x"], rec["part_y"], rec["part_z"] = [], [], []
        rec["wrist_x"], rec["wrist_y"], rec["wrist_z"] = [], [], []
        rec["finger_q"] = {"l": [], "r": []}
        if part is None or wrist is None:
            rec["error"] = "DEF PART / DEF WRIST not found"
        rec["engine_facts"] = engine_facts.facts()
    elif rung == 9:
        rec["bodies"] = {}
        for btag, _x, _y in rungs.rung9_pile_xy():
            node = robot.getFromDef("P_" + btag.upper())
            pile.append((btag, node))
            rec["bodies"][btag] = {"x": [], "y": [], "z": []}
        drop_node = robot.getFromDef("DROP")
        pile.append(("drop", drop_node))
        rec["bodies"]["drop"] = {"x": [], "y": [], "z": []}
        absent = [t for t, node in pile if node is None]
        if absent:
            rec["error"] = "bodies not found: %s" % ",".join(absent)
    elif rung == 11:
        rec["robots"] = {}
        rec["n"] = n_fleet
        for i in range(n_fleet):
            rtag = "r%02d" % i
            up = rtag.upper()
            node = robot.getFromDef(up)
            wheels = {w: robot.getFromDef("WHEEL_%s_%s" % (up, w.upper()))
                      for w in rungs.WHEEL_TAGS}
            fleet.append((rtag, node, wheels))
            rec["robots"][rtag] = {
                "x": [], "y": [], "z": [],
                "wheel_q": {w: [] for w in rungs.WHEEL_TAGS}}
        absent = [t for t, node, _w in fleet if node is None]
        if absent:
            rec["error"] = "fleet members not found: %s" % ",".join(absent)

    missing = [k for k, v in list(motors.items()) + list(sensors.items())
               if v is None]
    if missing:
        rec["error"] = "devices not found: %s" % ",".join(sorted(set(missing)))
    for s in sensors.values():
        if s is not None:
            s.enable(dt_ms)
    if ds is not None:
        ds.enable(dt_ms)
    if rung != 8:
        for m in motors.values():
            if m is not None:
                m.setPosition(float("inf"))      # velocity control
    else:
        # Rung 8 is the one POSITION-controlled rung.  Every stage is a
        # position target and the fingers develop their grip force as a known
        # interference against the part -- see omnisim/engine_facts.py for why
        # ``setForce`` is not the mechanism on this engine.
        for name, m in motors.items():
            if m is not None:
                m.setVelocity(rungs.RUNG8_LIFT_V if name == "lift"
                              else rungs.RUNG8_TRAVERSE_V if name == "traverse"
                              else 0.1)
                m.setPosition(0.0)

    if rung == 3 and motors.get("j0") is not None:
        motors["j0"].setVelocity(rungs.RUNG3_OMEGA_CMD)
    if rung in (4, 6):
        cmd = 0.0 if fault == "slide" else rungs.RUNG4_OMEGA_CMD
        for m in motors.values():
            if m is not None:
                m.setVelocity(cmd)

    def sample():
        rec["t"].append(robot.getTime())
        if rung in (1, 2):
            p = box.getPosition() if box is not None else [float("nan")] * 3
            rec["box_z"].append(p[2])
        elif rung == 3:
            s = sensors.get("j0")
            v = s.getValue() if s is not None else float("nan")
            rec["joint_q"].append(0.0 if (v is None or math.isnan(v)) else v)
        elif rung == 5:
            rec["body_x"].append(robot.getSelf().getPosition()[0])
            rec["range"].append(ds.getValue() if ds is not None
                                else float("nan"))
        elif rung in (4, 6):
            p = robot.getSelf().getPosition()
            rec["body_x"].append(p[0])
            rec["body_y"].append(p[1])
            rec["body_z"].append(p[2])
            for tag in rungs.WHEEL_TAGS:
                s = sensors.get(tag)
                v = s.getValue() if s is not None else float("nan")
                rec["wheel_q"][tag].append(
                    0.0 if (v is None or math.isnan(v)) else v)
            if rung == 6:
                rec["range"].append(ds.getValue() if ds is not None
                                    else float("nan"))
        elif rung in (7, 11):
            for rtag, node, wheels in fleet:
                d = rec["robots"][rtag]
                p = (node.getPosition() if node is not None
                     else [float("nan")] * 3)
                d["x"].append(p[0])
                d["y"].append(p[1])
                d["z"].append(p[2])
                for w in rungs.WHEEL_TAGS:
                    d["wheel_q"][w].append(_wheel_angle(wheels.get(w)))
        elif rung == 9:
            for btag, node in pile:
                d = rec["bodies"][btag]
                p = (node.getPosition() if node is not None
                     else [float("nan")] * 3)
                d["x"].append(p[0])
                d["y"].append(p[1])
                d["z"].append(p[2])
        elif rung == 8:
            pp = (part.getPosition() if part is not None
                  else [float("nan")] * 3)
            wp = (wrist.getPosition() if wrist is not None
                  else [float("nan")] * 3)
            rec["part_x"].append(pp[0])
            rec["part_y"].append(pp[1])
            rec["part_z"].append(pp[2])
            rec["wrist_x"].append(wp[0])
            rec["wrist_y"].append(wp[1])
            rec["wrist_z"].append(wp[2])
            for side in ("l", "r"):
                s = sensors.get("finger_%s" % side)
                v = s.getValue() if s is not None else float("nan")
                rec["finger_q"][side].append(
                    0.0 if (v is None or math.isnan(v)) else v)

    # A supervisor field read is valid before the first step, and rung 2's
    # spawn_z assertion needs it: it is the only chance to see the pose the
    # world FILE authored, before any integration has happened.  A sensor read
    # is not valid there (no measurement yet), so rungs 3-8 start at t = dt.
    if rung in (1, 2):
        sample()

    _breadcrumb("loop-start")
    n = int(round(duration / dt))
    if fault == "short_run":
        n = max(1, n // 2)
    if short != 1.0:
        # Rung 9's ``short_b``: one replica stops early.  The two runs then
        # agree over the OVERLAP and differ only in length, which is how a
        # truncated replica passes a determinism check that compares samples.
        n = max(1, int(round(n * short)))
    rec["sample_every"] = stride
    stopped = False
    bounced = False
    # ``bounce`` drives this much deeper than the honest threshold before it is
    # put back; the honest run never sees it.
    stop_at = (rungs.RUNG6_STOP_GAP - rungs.RUNG6_FAULT_BOUNCE_M
               if fault == "bounce" else rungs.RUNG6_STOP_GAP)
    bounce_x = (rungs.RUNG6_WALL_FACE_X - rungs.RUNG6_SENSOR_DX
                - rungs.RUNG6_FAULT_REST_GAP)

    for _ in range(n):
        if robot.step(dt_ms) == -1:
            break
        if t_first_step is None:
            t_first_step = time.time()
        rec["steps"] += 1
        t = robot.getTime()

        if rung == 3 and motors.get("j0") is not None:
            cmd = (0.0 if t >= rungs.RUNG3_ZERO_AT else rungs.RUNG3_OMEGA_CMD)
            if fault == "ignore_zero":
                cmd = rungs.RUNG3_OMEGA_CMD      # the stop command is dropped
            motors["j0"].setVelocity(cmd)

        if rung == 4 and fault == "slide":
            # Drag the chassis kinematically at exactly the speed a rolling
            # wheel would give, with every wheel commanded to zero: the
            # distance comes out RIGHT and only the rolling assertion can see
            # that nothing turned.
            node = robot.getSelf()
            node.getField("translation").setSFVec3f(
                [rungs.rolling_speed(rungs.RUNG4_OMEGA_CMD) * t, 0.0,
                 rungs.ROBOT_Z])
            node.resetPhysics()

        # Contract-owned decimation (amendment D).  ``stride`` comes from
        # rungs.py via the arm; the driver never picks one.  The recorded ``t``
        # is the true simulated time of the sample, never a reconstructed grid.
        if rung != 0 and (rec["steps"] % stride) == 0:
            sample()

        # --- commands, always AFTER the sample -------------------------
        # A sample taken from a pose the driver has just written but the engine
        # has not yet stepped would be a reading of the future, and rung 5's
        # whole-run residual is exactly the check that would notice.
        if rung == 5:
            x = (rungs.RUNG5_X0 if fault == "no_sweep"
                 else rungs.rung5_x_cmd(t))
            robot.getSelf().getField("translation").setSFVec3f(
                [x, 0.0, rungs.RUNG5_SENSOR_Z])

        elif rung == 6:
            reading = rec["range"][-1] if rec["range"] else float("nan")
            if (not stopped and fault != "no_stop"
                    and not math.isnan(reading) and reading < stop_at):
                for m in motors.values():
                    if m is not None:
                        m.setVelocity(0.0)
                stopped = True
                rec["driver_trigger_t"] = t
                rec["driver_trigger_reading"] = reading
            if fault == "bounce" and stopped:
                # Put the rover back at exactly the gap ``stop_gap`` expects.
                # The final state is then RIGHT and only ``min_gap``, which
                # looks at every sample, can see where it has been.
                node = robot.getSelf()
                node.getField("translation").setSFVec3f(
                    [bounce_x, 0.0, rungs.ROBOT_Z])
                node.resetPhysics()
                bounced = True

        elif rung == 8:
            closed = t >= rungs.RUNG8_T_SETTLE
            if fault == "no_grip":
                closed = False
            elif fault == "drop_mid_carry" and t >= rungs.RUNG8_T_LIFT:
                closed = False
            q = engine_facts.RUNG8_FINGER_CLOSE_Q if closed else 0.0
            if motors.get("finger_l") is not None:
                motors["finger_l"].setPosition(-q)
            if motors.get("finger_r") is not None:
                motors["finger_r"].setPosition(+q)
            if motors.get("lift") is not None:
                motors["lift"].setPosition(rungs.rung8_lift_z(t))
            if motors.get("traverse") is not None:
                motors["traverse"].setPosition(
                    0.0 if fault == "no_traverse"
                    else rungs.rung8_traverse_x(t))

    _breadcrumb("loop-done", "steps=%d bounced=%s" % (rec["steps"], bounced))
    rec["sim_time_end"] = robot.getTime()
    rec["wall"] = {"t_start": T_START, "t_first_step": t_first_step,
                   "t_end": time.time()}

    out = os.environ.get("LADDER0_OUT") or os.path.join(
        LADDER0, "results", "_last_rung%d_samples.json" % rung)
    try:
        d = os.path.dirname(out)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(rec, f)
        wrote = out
    except OSError as exc:                       # pragma: no cover
        wrote = "FAILED(%s)" % exc
    _breadcrumb("wrote", str(wrote))
    print("LADDER0_DONE steps=%d out=%s" % (rec["steps"], wrote), flush=True)
    robot.simulationQuit(0)


def _crash(exc):
    """A controller that dies must leave EVIDENCE.

    ``omnisim-bin.exe`` is a GUI-subsystem binary on Windows, so a controller's
    stdout and stderr are discarded -- ``--stdout --stderr`` forwards nothing
    and the engine log records only ``Terminating.``.  A traceback written to a
    file beside the samples is the only way a crashed controller can be told
    apart from one that never started, and telling those two apart is most of
    the debugging on this arm.
    """
    import traceback
    out = os.environ.get("LADDER0_OUT") or os.path.join(
        LADDER0, "results", "_last_samples.json")
    try:
        d = os.path.dirname(out)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(out + ".error", "w", encoding="utf-8") as f:
            f.write("argv: %r\npython: %s\ncwd: %s\n\n%s"
                    % (sys.argv, sys.version, os.getcwd(),
                       "".join(traceback.format_exception(
                           type(exc), exc, exc.__traceback__))))
    except OSError:
        pass


try:
    main()
except BaseException as _exc:                    # noqa: BLE001
    _crash(_exc)
    raise
