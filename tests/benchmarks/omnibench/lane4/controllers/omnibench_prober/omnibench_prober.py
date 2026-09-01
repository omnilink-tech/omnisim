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

"""omnibench_prober — OmniBench lane-4 generic capability prober (supervisor).

ONE controller drives every dynamic probe in lane4/capabilities.py. It knows
nothing about capabilities: it is told WHAT TO MEASURE and WHAT TO DO, records
raw physical quantities, and writes them out. All judgement lives in the
registry's assertions, on the Python side, so a verdict can be re-derived from
a recording without re-running the engine.

Measurement specs (--measure, semicolon-separated)
--------------------------------------------------
    pos:DEF             world position of DEF          -> pos_DEF     [N][3]
    quat:DEF            world orientation (wxyz)       -> quat_DEF    [N][4]
    jointpos:DEF        `position` field of a *JointParameters node
                                                       -> joint_DEF   [N]
    sensor:NAME         a device on THIS robot         -> sensor_NAME [N] or [N][k]
    contacts:DEF        len(getContactPoints())        -> contacts_DEF[N]
    particles:DEF       Node.getParticleStats(0)       -> particles_DEF [N]{dict}
                        per-recorded-step particle stats (stride 0 = stats
                        only, no sample dump): {status, count, min[3], max[3],
                        centroid[3], non_finite}. getattr-guarded: a
                        libController without the binding records None + ONE
                        meta.problems entry, so the scorer lands on
                        `inconclusive`, never `broken`. A frame whose status
                        is < 0 is recorded AS IS -- the status code is what
                        separates the INCONCLUSIVE causes (-5 GranularGroup
                        CUDA-inert, -9 stale libController).
    node_exists:DEF     one-shot bool                  -> node_exists_DEF
    device_exists:NAME  one-shot bool                  -> device_exists_NAME
    camera:NAME         one-shot image stats           -> camera_NAME {dict}
    radio:EMIT:RECV     one-shot send/receive result   -> radio_EMIT_RECV {dict}

Action specs (--act, semicolon-separated) — all times are sim seconds
---------------------------------------------------------------------
ONE-SHOT — fire once, the first step at which t >= T:

    motor_position:NAME:VALUE:T   setPosition(VALUE) at t >= T
    motor_velocity:NAME:VALUE:T   setPosition(inf) + setVelocity(VALUE) at
                                  t >= T (the infinite position target IS how
                                  velocity control is selected)
    motor_torque:NAME:VALUE:T     setTorque(VALUE) at t >= T (setPosition(inf)
                                  first, the documented prerequisite for
                                  torque control)
    delete_node:DEF:T             supervisor remove() of DEF at t >= T
    connector_lock:NAME:T         Connector.lock() at t >= T
    rebuild_physics:T             supervisor.simulationRebuildPhysics() at
                                  t >= T (the runtime-mutation purge verb,
                                  2026-09-01). getattr-guarded: a
                                  libController without the binding records a
                                  problem and publishes
                                  acted_rebuild_physics.binding_present=False
                                  so the assertion can name the stale
                                  libController instead of scoring the engine.

PERSISTENT — re-applied EVERY step from T to the end of the run:

    add_force:DEF:FX:FY:FZ:T                  Node.addForce([FX,FY,FZ], False)
    add_force_offset:DEF:FX:FY:FZ:OX:OY:OZ:T  Node.addForceWithOffset(...)

    These are persistent BY DESIGN and it is not a convenience. An external
    wrench queued through the supervisor is consumed into `state.body_f` and
    the accumulator is CLEARED after the tick (omnisim_newton_runtime.py,
    `add_body_force`), so a single call is a one-tick impulse — 4 ms of force,
    a velocity step of F*dt/m — and nothing an aerodynamic model does looks
    like that. A constant force is a call per step, so that is what these do,
    and the number of calls actually made is recorded (`acted_*`) so an
    assertion can check the premise instead of assuming it.

Output
------
A single JSON file at $OMNIBENCH_OUT (absolute — a controller's cwd is its own
directory, so a relative path writes somewhere nobody looks). Contains the
recorded arrays plus a `meta` block (steps, dt, wall ms/step, the device
inventory, and every non-fatal problem the probe hit).

Robustness contract: a missing node, a missing device or an unreadable field is
NEVER fatal and never silently skipped — it is recorded as null in the series
and named in `meta.problems`, so the scorer can return `inconclusive` instead
of mistaking an instrument failure for an engine defect. The one thing this
controller must always do is reach simulationQuit(0): without it the engine
free-runs until the runner's timeout, which cost lane 1R 36 hours of wall clock
before it was diagnosed.
"""

import json
import math
import os
import sys
import time

from omnisim import Supervisor


#: Devices whose reading is genuinely a vector. Everything else is read with
#: getValue(); see read_sensor() for why this is a class list and not an
#: attribute probe.
_VECTOR_DEVICES = frozenset((
    "GPS", "Gyro", "Accelerometer", "Compass", "Magnetometer"))


def _arg(argv, name, default=None):
    pref = "--%s=" % name
    for a in argv:
        if a.startswith(pref):
            return a[len(pref):]
    return default


class Prober:
    def __init__(self):
        self.sv = Supervisor()
        argv = sys.argv[1:]
        self.label = _arg(argv, "label", "probe")
        self.duration = float(_arg(argv, "duration", "3.0"))
        self.measures = [m for m in (_arg(argv, "measure", "") or "").split(";")
                         if m]
        self.acts = [a for a in (_arg(argv, "act", "") or "").split(";") if a]
        self.out_path = os.environ.get("OMNIBENCH_OUT") or _arg(argv, "out", "")
        self.dt_ms = int(round(self.sv.getBasicTimeStep()))
        if self.dt_ms <= 0:
            # Robot.step() takes an INTEGER millisecond count; a world with a
            # fractional basicTimeStep below 1 ms yields step(0), which is an
            # infinite no-op rather than an error (lane 1R hit exactly this).
            self.dt_ms = 1
            self.problems = ["basicTimeStep rounded to 0 ms; forced to 1 ms"]
        else:
            self.problems = []
        self.series = {}
        self.oneshot = {}
        self.t = []
        self.devices = {}

    # -- setup ------------------------------------------------------------
    def log(self, msg):
        print("[omnibench_prober] %s" % msg, flush=True)

    def problem(self, msg):
        self.problems.append(msg)
        self.log("PROBLEM: %s" % msg)

    def device(self, name):
        """Fetch + enable a device on this robot, cached. Returns None (and
        records a problem, once) when the device is not present."""
        if name in self.devices:
            return self.devices[name]
        dev = None
        try:
            dev = self.sv.getDevice(name)
        except Exception as exc:
            self.problem("getDevice(%r) raised %r" % (name, exc))
        if dev is None:
            self.problem("device %r not found on the prober robot" % name)
        else:
            try:
                dev.enable(self.dt_ms)
            except Exception:
                pass          # motors/emitters/connectors have no enable()
        self.devices[name] = dev
        return dev

    def node(self, defname):
        n = self.sv.getFromDef(defname)
        if n is None:
            self.problem("DEF %r not found in the scene tree" % defname)
        return n

    def setup(self):
        self.nodes = {}
        self.joint_fields = {}
        self.sensor_names = []
        self.contact_defs = []
        self.particle_defs = []
        #: DEFs whose getParticleStats read already recorded a problem, so a
        #: missing binding is reported ONCE, not once per recorded step (750
        #: identical strings is a recording, not a diagnosis).
        self._particle_flagged = set()
        for spec in self.measures:
            kind, _, rest = spec.partition(":")
            if kind in ("pos", "quat"):
                self.nodes.setdefault(rest, self.node(rest))
                self.series["%s_%s" % (kind, rest)] = []
            elif kind == "jointpos":
                n = self.node(rest)
                f = None
                if n is not None:
                    try:
                        f = n.getField("position")
                    except Exception as exc:
                        self.problem("getField('position') on %r raised %r"
                                     % (rest, exc))
                if f is None and n is not None:
                    self.problem("node %r has no `position` field (a "
                                 "BallJointParameters has none, which is a "
                                 "finding, not an instrument failure)" % rest)
                self.joint_fields[rest] = f
                self.series["joint_%s" % rest] = []
            elif kind == "sensor":
                self.device(rest)
                self.sensor_names.append(rest)
                self.series["sensor_%s" % rest] = []
            elif kind == "contacts":
                self.nodes.setdefault(rest, self.node(rest))
                self.contact_defs.append(rest)
                self.series["contacts_%s" % rest] = []
            elif kind == "particles":
                self.nodes.setdefault(rest, self.node(rest))
                self.particle_defs.append(rest)
                self.series["particles_%s" % rest] = []
            elif kind == "node_exists":
                self.oneshot["node_exists_%s" % rest] = (
                    self.sv.getFromDef(rest) is not None)
            elif kind == "device_exists":
                self.oneshot["device_exists_%s" % rest] = (
                    self.device(rest) is not None)
            elif kind == "camera":
                self.device(rest)
                self.camera_name = rest
            elif kind == "receiver":
                # Receiver on THIS robot, Emitter on another one: the engine
                # refuses same-robot delivery by design, so an in-robot pair
                # can never succeed and must never be used to test the radio.
                self.radio = ("", rest)
            else:
                self.problem("unknown measure spec %r" % spec)
        # actions: parse once, fire by sim time
        self.actions = []
        self.persistent = []
        for spec in self.acts:
            parts = spec.split(":")
            try:
                if parts[0] in ("motor_position", "motor_torque",
                                "motor_velocity"):
                    self.actions.append((float(parts[3]), parts[0], parts[1],
                                         float(parts[2])))
                elif parts[0] == "delete_node":
                    self.actions.append((float(parts[2]), parts[0], parts[1],
                                         None))
                elif parts[0] == "connector_lock":
                    self.actions.append((float(parts[2]), parts[0], parts[1],
                                         None))
                elif parts[0] == "rebuild_physics":
                    self.actions.append((float(parts[1]), parts[0], "", None))
                elif parts[0] == "add_force":
                    self.persistent.append(
                        (float(parts[5]), parts[0], parts[1],
                         [float(x) for x in parts[2:5]]))
                elif parts[0] == "add_force_offset":
                    self.persistent.append(
                        (float(parts[8]), parts[0], parts[1],
                         [float(x) for x in parts[2:8]]))
                else:
                    self.problem("unknown act spec %r" % spec)
            except (IndexError, ValueError) as exc:
                self.problem("malformed act spec %r (%r)" % (spec, exc))
        # Sort on the TIME only. The one-shot values are floats and the
        # persistent ones are lists, so a plain tuple sort would compare a list
        # against a float on any tie and die in the parser -- an instrument
        # failure that would present as "the world never ran".
        self.actions.sort(key=lambda a: a[0])
        self.fired = [False] * len(self.actions)
        #: Per persistent action: how many times the supervisor call was
        #: actually made, and the first exception it raised (once, not per
        #: step -- 750 identical problem strings is a recording, not a
        #: diagnosis). Published as `acted_<kind>_<DEF>` so an assertion can
        #: GUARD ITS PREMISE: "the force was applied 625 times and the body did
        #: not move" is a finding, "it was applied 0 times" is a rig failure.
        self.acted = {}
        for _, kind, target, _v in self.persistent:
            self.acted["acted_%s_%s" % (kind, target)] = {
                "calls": 0, "node_found": None, "error": None}

    # -- per-step measurement ---------------------------------------------
    def read_sensor(self, name):
        dev = self.devices.get(name)
        if dev is None:
            return None
        cls = type(dev).__name__
        try:
            if cls == "InertialUnit":
                return list(dev.getRollPitchYaw())
            if cls == "Lidar":
                img = dev.getRangeImage()
                if img is None:
                    return None
                # BOUND the read by the lidar's own declared size. Depending
                # on the binding, getRangeImage() can hand back a raw ctypes
                # float pointer, and list() over one of those never
                # terminates -- measured as a "lidar hang" that looked like
                # an engine defect for two runs and was this loop.
                n = int(dev.getHorizontalResolution()) * int(dev.getNumberOfLayers())
                return [float(img[i]) for i in range(n)]
            # Dispatch by CLASS, never by "does it have getValues()". Several
            # scalar devices expose BOTH accessors -- a TouchSensor has
            # getValues() for its 3-axis force variant while a bumper's real
            # reading is the scalar getValue() -- so an attribute-order probe
            # silently returns a vector where the scorer expects a number and
            # the probe dies in float() as an INSTRUMENT failure. Measured:
            # touch_bumper and touch_force both came back inconclusive on
            # exactly this.
            if cls in _VECTOR_DEVICES:
                v = dev.getValues()
                return list(v) if v is not None else None
            if hasattr(dev, "getValue"):
                return float(dev.getValue())
            if hasattr(dev, "getValues"):
                v = dev.getValues()
                return list(v) if v is not None else None
        except Exception as exc:
            self.problem("reading device %r (%s) raised %r" % (name, cls, exc))
        return None

    def read_particles(self, defname):
        """One getParticleStats frame for DEF, stats only (stride 0), or None.

        Written AGAINST the 2026-09-01 supervisor binding
        Node.getParticleStats(sample_stride=0) -> {'status', 'count', 'min',
        'max', 'centroid', 'non_finite', 'sample'} and getattr-guarded for the
        libControllers that predate it: a missing binding is a recorded
        problem, never a crash and never a fake number, per the robustness
        contract in the module docstring. A frame whose status is < 0 is kept
        AS RECORDED so the scorer can separate the refusal causes (-5 =
        GranularGroup CUDA-inert, -9 = stale libController). The (empty at
        stride 0) 'sample' array is dropped so 750 recorded frames stay a
        recording, not a dump.
        """
        n = self.nodes.get(defname)
        if n is None:
            return None
        fn = getattr(n, "getParticleStats", None)
        if fn is None:
            if defname not in self._particle_flagged:
                self._particle_flagged.add(defname)
                self.problem("Node.getParticleStats missing on %r -- this "
                             "libController predates the particle-stats "
                             "binding (stale libController; rebuild the "
                             "controller libs / run `omnisim doctor`)"
                             % defname)
            return None
        try:
            try:
                st = fn(0)         # sample_stride=0 -> stats only, no sample
            except TypeError:
                st = fn()
        except Exception as exc:
            if defname not in self._particle_flagged:
                self._particle_flagged.add(defname)
                self.problem("getParticleStats on %r raised %r"
                             % (defname, exc))
            return None
        if isinstance(st, dict):
            return {k: v for k, v in st.items() if k != "sample"}
        if defname not in self._particle_flagged:
            self._particle_flagged.add(defname)
            self.problem("getParticleStats on %r returned %s, not a dict"
                         % (defname, type(st).__name__))
        return None

    def record(self, t):
        self.t.append(t)
        for defname, n in self.nodes.items():
            if "pos_%s" % defname in self.series:
                self.series["pos_%s" % defname].append(
                    list(n.getPosition()) if n is not None else None)
            if "quat_%s" % defname in self.series:
                self.series["quat_%s" % defname].append(
                    _mat3_to_quat(n.getOrientation()) if n is not None else None)
        for defname, f in self.joint_fields.items():
            v = None
            if f is not None:
                try:
                    v = float(f.getSFFloat())
                except Exception:
                    v = None
            self.series["joint_%s" % defname].append(v)
        for name in self.sensor_names:
            self.series["sensor_%s" % name].append(self.read_sensor(name))
        for defname in self.contact_defs:
            n = self.nodes.get(defname)
            c = None
            if n is not None:
                try:
                    pts = n.getContactPoints()
                    c = len(pts) if pts is not None else 0
                except Exception as exc:
                    self.problem("getContactPoints on %r raised %r"
                                 % (defname, exc))
            self.series["contacts_%s" % defname].append(c)
        for defname in self.particle_defs:
            self.series["particles_%s" % defname].append(
                self.read_particles(defname))

    # -- actions ----------------------------------------------------------
    def fire(self, t):
        for i, (when, kind, target, value) in enumerate(self.actions):
            if self.fired[i] or t < when:
                continue
            self.fired[i] = True
            try:
                if kind == "motor_position":
                    dev = self.device(target)
                    if dev is not None:
                        dev.setPosition(value)
                        self.log("t=%.3f setPosition(%s) on %r"
                                 % (t, value, target))
                elif kind == "motor_torque":
                    dev = self.device(target)
                    if dev is not None:
                        # velocity/position control must be released first or
                        # the servo fights the torque command
                        dev.setPosition(float("inf"))
                        dev.setTorque(value)
                        self.log("t=%.3f setTorque(%s) on %r"
                                 % (t, value, target))
                elif kind == "motor_velocity":
                    dev = self.device(target)
                    if dev is not None:
                        # infinite position target IS how velocity control is
                        # selected; without it setVelocity only caps the speed
                        # at which a position target is approached
                        dev.setPosition(float("inf"))
                        dev.setVelocity(value)
                        self.log("t=%.3f setVelocity(%s) on %r"
                                 % (t, value, target))
                elif kind == "delete_node":
                    n = self.sv.getFromDef(target)
                    if n is None:
                        self.problem("delete_node: DEF %r not found" % target)
                    else:
                        n.remove()
                        self.log("t=%.3f removed DEF %r" % (t, target))
                elif kind == "connector_lock":
                    dev = self.device(target)
                    if dev is not None:
                        dev.lock()
                        self.log("t=%.3f lock() on %r" % (t, target))
                elif kind == "rebuild_physics":
                    # Premise record FIRST, so the assertion can tell "the
                    # verb is missing from this libController" (stale, ->
                    # inconclusive) from "the verb ran and changed nothing"
                    # (-> a capability verdict). Same acted_* pattern the
                    # persistent wrenches use.
                    slot = {"requested_t": when, "binding_present": None,
                            "called": False, "error": None}
                    self.oneshot["acted_rebuild_physics"] = slot
                    fn = getattr(self.sv, "simulationRebuildPhysics", None)
                    slot["binding_present"] = fn is not None
                    if fn is None:
                        self.problem(
                            "supervisor.simulationRebuildPhysics missing -- "
                            "this libController predates the rebuild verb "
                            "(2026-09-01; stale libController, rebuild the "
                            "controller libs / run `omnisim doctor`)")
                    else:
                        try:
                            fn()
                            slot["called"] = True
                            self.log("t=%.3f simulationRebuildPhysics()" % t)
                        except Exception as exc:
                            slot["error"] = repr(exc)
                            self.problem(
                                "simulationRebuildPhysics() raised %r" % exc)
            except Exception as exc:
                self.problem("action %s on %r raised %r" % (kind, target, exc))
        self.fire_persistent(t)

    def fire_persistent(self, t):
        """Re-apply every persistent wrench whose start time has passed.

        Called once per step from fire(). A supervisor wrench lives for exactly
        one tick, so 'constant force' means 'call it again every tick'; the
        call count is recorded so an assertion can tell a dropped force from a
        force that was never asked for.
        """
        for when, kind, target, value in self.persistent:
            if t < when:
                continue
            slot = self.acted["acted_%s_%s" % (kind, target)]
            n = self.nodes.get(target)
            if n is None:
                n = self.sv.getFromDef(target)
                self.nodes[target] = n
            if slot["node_found"] is None:
                slot["node_found"] = n is not None
                if n is None:
                    self.problem("%s: DEF %r not found in the scene tree"
                                 % (kind, target))
            if n is None:
                continue
            try:
                if kind == "add_force":
                    n.addForce(list(value), False)
                else:
                    n.addForceWithOffset(list(value[:3]), list(value[3:]),
                                         False)
                slot["calls"] += 1
            except Exception as exc:
                if slot["error"] is None:
                    slot["error"] = repr(exc)
                    self.problem("%s on %r raised %r" % (kind, target, exc))

    # -- one-shot probes at the end ---------------------------------------
    def finish_camera(self):
        name = getattr(self, "camera_name", None)
        if not name:
            return
        cam = self.devices.get(name)
        stats = {"device_present": cam is not None}
        if cam is not None:
            try:
                img = cam.getImage()
                w, h = cam.getWidth(), cam.getHeight()
                stats.update(width=w, height=h,
                             pixels=(len(img) // 4) if img else 0,
                             bytes=len(img) if img else 0,
                             distinct_values=len(set(img)) if img else 0)
            except Exception as exc:
                stats["error"] = repr(exc)
                self.problem("camera %r read raised %r" % (name, exc))
        self.oneshot["camera_%s" % name] = stats

    def run_radio(self):
        """Set up the radio probe. Draining happens inside the main loop
        because a packet is delivered on a later step than the one it was sent
        on. `emit` is empty when the transmitter lives on another robot (the
        only topology the engine actually delivers between)."""
        emit, recv = self.radio
        e = self.device(emit) if emit else None
        r = self.device(recv)
        res = {"receiver_present": r is not None,
               "emitter_on_this_robot": bool(emit),
               "emitter_present": (e is not None) if emit else None,
               "received": False, "queue_peak": 0}
        self.oneshot["radio_%s" % recv] = res
        return res

    # -- main -------------------------------------------------------------
    def main(self):
        self.setup()
        radio = self.run_radio() if hasattr(self, "radio") else None
        n_steps = max(1, int(round(self.duration * 1000.0 / self.dt_ms)))
        self.record(0.0)
        wall = 0.0
        done = 0
        sent = False
        for k in range(n_steps):
            t = k * self.dt_ms / 1000.0
            self.fire(t)
            if radio and self.radio[0] and radio["emitter_present"] and not sent:
                try:
                    self.devices[self.radio[0]].send(b"omnibench")
                    sent = True
                except Exception as exc:
                    self.problem("Emitter.send raised %r" % exc)
            w0 = time.perf_counter()
            rc = self.sv.step(self.dt_ms)
            wall += time.perf_counter() - w0
            if rc == -1:
                self.problem("step() returned -1 at step %d/%d" % (k + 1, n_steps))
                break
            done += 1
            if radio and radio["receiver_present"]:
                try:
                    r = self.devices[self.radio[1]]
                    q = r.getQueueLength()
                    radio["queue_peak"] = max(radio["queue_peak"], q)
                    while q > 0:
                        radio["received"] = True
                        r.nextPacket()
                        q = r.getQueueLength()
                except Exception as exc:
                    self.problem("Receiver drain raised %r" % exc)
            self.record((k + 1) * self.dt_ms / 1000.0)
        self.finish_camera()

        out = dict(self.series)
        out.update(self.oneshot)
        out.update(getattr(self, "acted", {}))
        out["t"] = self.t
        out["meta"] = {
            "label": self.label,
            "world": self.sv.getWorldPath(),
            "dt_ms": self.dt_ms,
            "steps": done,
            "sim_seconds": done * self.dt_ms / 1000.0,
            "wall_ms_per_step": (wall * 1000.0 / done) if done else None,
            "problems": self.problems,
            "devices_requested": sorted(self.devices),
            "devices_missing": sorted(k for k, v in self.devices.items()
                                      if v is None),
        }
        if self.out_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.out_path)),
                        exist_ok=True)
            with open(self.out_path, "w", encoding="utf-8") as f:
                json.dump(out, f)
            self.log("wrote %d steps (%d problems) -> %s"
                     % (done, len(self.problems), self.out_path))
        else:
            self.problem("no OMNIBENCH_OUT set; nothing written")
        # ALWAYS quit: without this the engine free-runs to the runner timeout
        self.sv.simulationQuit(0)
        self.sv.step(self.dt_ms)


def _mat3_to_quat(m):
    """Row-major 3x3 (the supervisor's getOrientation) -> [w, x, y, z]."""
    if m is None:
        return None
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = [float(v) for v in m]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]


def main():
    try:
        Prober().main()
    except Exception:
        # A traceback that never reaches disk is a probe that looks like an
        # engine hang. Print it (the runner captures stdout) and still exit.
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        raise


main()
