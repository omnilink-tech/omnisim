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

"""drivers.py -- one class per rung: what to hold, what to command, what to
record.

``run.py`` owns the step loop and is rung-agnostic; everything that differs
between rungs lives here.  A driver may:

* ``command()``   -- write ``data.ctrl`` (or, for a kinematic fault, ``qpos``)
                     for the interval that is about to be integrated;
* ``record(out)`` -- append ONE sample to the sample-document series it owns;
* ``facts()``     -- report read-back numbers from the COMPILED model, so the
                     result file says which solver produced it.

A driver NEVER computes an expected value, a tolerance or a verdict.  Those
live in the shared contract (``rungs.py``) and the shared reducer
(``analysis.py``); an arm that judged itself would be marking its own homework
and the whole ladder would be worth nothing.  The series names and units here
are CONTRACT.md section 4, verbatim.

THE STEP LOOP'S CONTRACT WITH A DRIVER
--------------------------------------
``run.py`` guarantees that when ``command()`` and ``record()`` are called,
``data`` is FORWARD-CONSISTENT: ``data.time``, ``data.xpos``, ``data.site_xpos``
and ``data.sensordata`` all describe the same instant.  That is not what a
naive loop gives you, and the difference is measurable:

    mj_step(m, d)      # d.time == t+dt, but d.xpos is still the pose at t

``mj_step`` runs the forward pass, then integrates; it does not re-run
kinematics afterwards.  So a loop that steps and then reads ``xpos`` records a
sample LABELLED t+dt CARRYING THE POSE AT t.  Measured on rung 2 before the
fix: ``fall_time_abs`` came out 2 ms late (+dt of lag against the integrator's
own -dt/2 bias); after it, -2 ms, which is the integrator bias alone and
nothing else.

For a passive rung that is a small bias.  For rungs 5-8 it is not, because
those rungs close a LOOP: rung 6 brakes on a sensor reading, and a controller
fed a reading that is one step stale brakes v*dt late -- 1.6 mm at 0.4 m/s,
injected into the very quantity the rung measures, by the harness rather than
by the engine.  ``run.py`` therefore drives ``mj_step2`` (integrate) followed
by ``mj_step1`` (forward at the new time), which costs exactly one step's work
and is bit-identical to ``mj_step`` -- verified for both the default Euler
integrator and ``implicitfast``, max |dqpos| = 0 over 750 steps.
"""

from __future__ import annotations

import importlib.util
import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))


def _load_shared():
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
scenes = shared.sibling("scenes")


def obj_id(mj, model, objtype, name):
    i = mj.mj_name2id(model, objtype, name)
    if i < 0:
        raise RuntimeError("no %s named %r in the compiled model"
                           % (objtype, name))
    return i


class Driver:
    """Base class.  A rung with no state to record still needs one."""

    def __init__(self, mj, model, data, fault=None, run=None):
        self.mj = mj
        self.model = model
        self.data = data
        self.fault = fault
        # The parameters of THIS RUN of a multi-run cell (CONTRACT.md
        # amendment A) -- rung 11's fleet size, rung 9's perturbation.  Always
        # one entry of ``scenes.run_specs``; a driver never invents one.
        self.run = dict(run or {})

    # -- overridables ------------------------------------------------------
    def command(self):
        """Write actuation for the interval about to be integrated."""

    def record(self, out):
        """Append one sample to ``out`` (a dict of lists)."""

    def facts(self):
        """Read-back numbers from the compiled model, for the result file."""
        return {}

    def series(self):
        """Sample-document keys this driver fills, in CONTRACT.md section 4."""
        return ()

    # -- helpers -----------------------------------------------------------
    def _joint(self, name):
        return obj_id(self.mj, self.model, self.mj.mjtObj.mjOBJ_JOINT, name)

    def _body(self, name):
        return obj_id(self.mj, self.model, self.mj.mjtObj.mjOBJ_BODY, name)

    def _act(self, name):
        return obj_id(self.mj, self.model, self.mj.mjtObj.mjOBJ_ACTUATOR, name)

    def _site(self, name):
        return obj_id(self.mj, self.model, self.mj.mjtObj.mjOBJ_SITE, name)

    def _sensor_adr(self, name):
        i = obj_id(self.mj, self.model, self.mj.mjtObj.mjOBJ_SENSOR, name)
        return int(self.model.sensor_adr[i])

    def _dof_M0(self, joint_name):
        """The compiled model's own inertia at a joint's DOF.

        Read back rather than assumed: it turns the deadbeat-gain claim
        ``kv dt / I == 1`` into a measured field in the result file instead of
        a sentence in a docstring.
        """
        jid = self._joint(joint_name)
        return float(self.model.dof_M0[self.model.jnt_dofadr[jid]])


# --------------------------------------------------------------------------
# rung 0 -- the null scene
# --------------------------------------------------------------------------

class Rung0(Driver):
    """No bodies, nothing to command, nothing to record.

    The rung's whole content is ``steps_completed`` / ``exit_code`` /
    ``finite_clock``, which ``run.py`` records for every rung: a run that
    exits 0 having stepped zero times is the stale-libController signature and
    it looks identical to a pass.
    """


# --------------------------------------------------------------------------
# rungs 1, 2 -- a box on / above a floor
# --------------------------------------------------------------------------

class BoxDriver(Driver):
    """Records ``box_z``: the world-frame z of the box's centre, metres."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.b_box = self._body("box")

    def series(self):
        return ("box_z",)

    def record(self, out):
        out["box_z"].append(float(self.data.xpos[self.b_box][2]))


# --------------------------------------------------------------------------
# rung 3 -- one hinge, velocity servo, no load
# --------------------------------------------------------------------------

class HingeDriver(Driver):
    """Records ``joint_q``: the hinge angle in radians.

    Commands ``RUNG3_OMEGA_CMD`` until ``RUNG3_ZERO_AT``, then zero.  The
    ``ignore_zero`` fault never makes the second transition -- the shape of a
    command that is accepted and silently dropped.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.q_adr = self.model.jnt_qposadr[self._joint("hinge")]
        self.act = self._act("hinge_motor")

    def series(self):
        return ("joint_q",)

    def command(self):
        driving = (self.data.time < spec.RUNG3_ZERO_AT
                   or self.fault == "ignore_zero")
        self.data.ctrl[self.act] = spec.RUNG3_OMEGA_CMD if driving else 0.0

    def record(self, out):
        out["joint_q"].append(float(self.data.qpos[self.q_adr]))

    def facts(self):
        inertia = self._dof_M0("hinge")
        return {"hinge_dof_M0": inertia, "kv": scenes.RUNG3_KV,
                "kv_dt_over_I": scenes.RUNG3_KV * spec.DT / inertia}


# --------------------------------------------------------------------------
# rung 4 -- four driven wheels, straight line
# --------------------------------------------------------------------------

class RoverDriver(Driver):
    """Records ``body_x``/``body_y``/``body_z`` and ``wheel_q``.

    ``body_z`` is the axle height and the contract's ``ride_height`` check
    reads it; omitting it once made that check RED for want of a number rather
    than because the rover misbehaved, which is an arm measuring less than it
    claims to.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.b_chassis = self._body("chassis")
        self.wheel_adr = {}
        self.act_wheel = []
        for name, _x, _y in scenes.WHEELS:
            self.wheel_adr[name] = self.model.jnt_qposadr[
                self._joint("%s_joint" % name)]
            self.act_wheel.append(self._act("%s_motor" % name))
        free = self._joint("chassis_free")
        self.free_qpos = self.model.jnt_qposadr[free]
        self.free_qvel = self.model.jnt_dofadr[free]

    def series(self):
        return ("body_x", "body_y", "body_z", "wheel_q")

    def command(self):
        for a in self.act_wheel:
            self.data.ctrl[a] = (0.0 if self.fault == "slide"
                                 else spec.RUNG4_OMEGA_CMD)
        if self.fault == "slide":
            # Drag the chassis along +X at exactly omega*r with every wheel
            # dead.  Set the POSE, not the velocity: the contract requires
            # ``distance`` to stay GREEN so ``rolling_consistency`` is the only
            # thing that reddens, and a velocity push loses several percent of
            # the distance to contact and gravity (measured: 1.484 m of 1.600).
            self.data.qpos[self.free_qpos] = spec.rolling_speed(
                spec.RUNG4_OMEGA_CMD) * self.data.time
            self.data.qpos[self.free_qpos + 1] = 0.0
            self.data.qpos[self.free_qpos + 2] = spec.ROBOT_Z
            for k in range(6):
                self.data.qvel[self.free_qvel + k] = 0.0

    def record(self, out):
        p = self.data.xpos[self.b_chassis]
        out["body_x"].append(float(p[0]))
        out["body_y"].append(float(p[1]))
        out["body_z"].append(float(p[2]))
        for name, adr in self.wheel_adr.items():
            out["wheel_q"][name].append(float(self.data.qpos[adr]))

    def facts(self):
        inertia = self._dof_M0("fl_joint")
        return {"wheel_dof_M0": inertia, "kv": scenes.RUNG4_KV,
                "kv_dt_over_I": scenes.RUNG4_KV * spec.DT / inertia,
                "ncon_final": int(self.data.ncon),
                "total_mass_kg": float(sum(self.model.body_mass))}


# --------------------------------------------------------------------------
# rung 5 -- a range sensor on a kinematically swept carrier
# --------------------------------------------------------------------------

class RangeSweepDriver(Driver):
    """Records ``body_x`` (carrier origin) and ``range`` (metres).

    The carrier is a MOCAP body, so its pose is WRITTEN rather than simulated:
    ``data.mocap_pos`` is the whole of its dynamics.  That is what the rung
    asks for -- a sensor rung whose sensor pose came out of a wheel model
    could not separate "the ray is wrong" from "the carrier did not get
    there".

    THE POSE WRITTEN IS THE ONE FOR THE NEXT SAMPLE.  ``command()`` runs at
    ``t`` and the sample that observes its effect is taken at ``t+dt``, so it
    writes ``rung5_x_cmd(t+dt)`` and the recorded pairs then satisfy the
    contract's schedule exactly rather than one step behind it.  Nothing in
    the reduction depends on that (the residual compares the ray against the
    pose recorded on the SAME step, and the dwell/park windows saturate), but
    an arm whose recorded x does not equal the function the contract publishes
    is an arm that would mislead the next person to plot it.

    ``range`` IS RECORDED RAW, IN METRES.  MuJoCo's rangefinder reports the
    distance from the site origin along the site's +Z to the first surface;
    -1 means the ray hit nothing.  Nothing is clamped, rescaled or
    substituted here -- the contract's ``range_tracks`` check is exactly the
    comparison of this number with the geometry.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.b_carrier = self._body("carrier")
        self.mocap = int(self.model.body_mocapid[self.b_carrier])
        self.rng = self._sensor_adr("range")

    def series(self):
        return ("body_x", "range")

    def command(self):
        t_next = float(self.data.time) + spec.DT
        if self.fault == "no_sweep":
            # The carrier is never moved.  The sensor stays honest and the
            # SCENE stops changing, which is observationally what a sensor
            # frozen at the authored t=0 pose looks like -- the defect this
            # rung exists to catch, and one this repo has measured on
            # OmniSim's raycast sensors under mujoco_warp.  Injecting it into
            # the MOTION rather than into the reading keeps the fault out of
            # the measurement path: ``range_tracks`` stays green precisely
            # because the ray still agrees with the (unchanged) pose.
            t_next = 0.0
        self.data.mocap_pos[self.mocap][0] = spec.rung5_x_cmd(t_next)

    def record(self, out):
        out["body_x"].append(float(self.data.xpos[self.b_carrier][0]))
        out["range"].append(float(self.data.sensordata[self.rng]))


# --------------------------------------------------------------------------
# rung 6 -- drive, and stop when the sensor reads below a threshold
# --------------------------------------------------------------------------

class ApproachDriver(RoverDriver):
    """The rung-4 rover with a forward range sensor and a stop rule.

    Drives at ``RUNG4_OMEGA_CMD`` until the sensor first reads below
    ``RUNG6_STOP_GAP``, then commands every wheel to zero and LATCHES -- a
    rule that could un-trigger would make the stopping distance depend on
    sensor noise instead of on the brake.

    ``r >= 0.0`` IS NOT DEFENSIVE PADDING, IT IS THE WHOLE RULE.  A MuJoCo
    rangefinder reports -1 when the ray hits nothing, and -1 is less than any
    positive threshold: a naive ``r < RUNG6_STOP_GAP`` brakes instantly on an
    EMPTY horizon and the rover stops 2.68 m from a wall it never saw, which
    scores as a rung that passed nothing.  Engines whose distance sensor
    saturates at a maximum instead (the Webots lineage) have the opposite
    hazard and never trigger.  The rung is written on a HIT.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.rng = self._sensor_adr("range")
        self.triggered = False
        self.trigger_t = None
        self.bounced = False
        # Where ``bounce`` parks the rover: the contract's own resting gap,
        # converted to a chassis-origin x.  Nothing is chosen here.
        self.rest_x = (spec.RUNG6_WALL_FACE_X - spec.RUNG6_FAULT_REST_GAP
                       - spec.RUNG6_SENSOR_DX)
        self.bounce_x = (spec.RUNG6_WALL_FACE_X
                         - (spec.RUNG6_STOP_GAP - spec.RUNG6_FAULT_BOUNCE_M)
                         - spec.RUNG6_SENSOR_DX)

    def series(self):
        return ("body_x", "body_y", "body_z", "wheel_q", "range")

    def command(self):
        r = float(self.data.sensordata[self.rng])
        x = float(self.data.xpos[self.b_chassis][0])

        if self.fault == "bounce":
            # Rung 6's answer to rung 4's ``slide``.  The rover overshoots the
            # threshold by RUNG6_FAULT_BOUNCE_M and is then PUT BACK at
            # exactly the resting place a correct stop would have reached, so
            # the tail-window checks -- stop_gap, wheel_stop, stop_creep --
            # are all green and only the whole-run ``min_gap`` can see that it
            # was ever closer.  This is the rung's whole argument for owning a
            # non-windowed invariant.
            if not self.bounced and x >= self.bounce_x:
                self.bounced = True
                self.trigger_t = float(self.data.time)
            for a in self.act_wheel:
                self.data.ctrl[a] = 0.0 if self.bounced else \
                    spec.RUNG4_OMEGA_CMD
            if self.bounced:
                self.data.qpos[self.free_qpos] = self.rest_x
                self.data.qpos[self.free_qpos + 1] = 0.0
                self.data.qpos[self.free_qpos + 2] = spec.ROBOT_Z
                for k in range(6):
                    self.data.qvel[self.free_qvel + k] = 0.0
            return

        if not self.triggered and r >= 0.0 and r < spec.RUNG6_STOP_GAP:
            # ``no_stop``: the reading is taken, the threshold is crossed, and
            # nothing is done about it.  The SENSING half of the rung must
            # stay green -- a controller that reads a sensor correctly and
            # then ignores it is a real and common bug, and it must not look
            # like a broken sensor.
            if self.fault != "no_stop":
                self.triggered = True
                self.trigger_t = float(self.data.time)
        drive = 0.0 if self.triggered else spec.RUNG4_OMEGA_CMD
        for a in self.act_wheel:
            self.data.ctrl[a] = drive

    def record(self, out):
        super().record(out)
        out["range"].append(float(self.data.sensordata[self.rng]))

    def facts(self):
        f = super().facts()
        f["triggered"] = self.triggered
        f["trigger_t_s"] = self.trigger_t
        return f


# --------------------------------------------------------------------------
# rung 7 -- five rovers, five commands
# --------------------------------------------------------------------------

class FleetDriver(Driver):
    """Five rung-4 rovers in lanes, each on its own constant wheel command.

    Records the contract's nested ``robots`` document.  Each robot's own
    distance is judged against what it would travel ALONE, so a command that
    leaked between robots, or a robot that was nudged by its neighbour, shows
    up as that robot's own number moving -- which needs no contact API at all.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.robots = []
        for i, tag in enumerate(spec.RUNG7_TAGS):
            p = tag + "_"
            free = self._joint(p + "chassis_free")
            self.robots.append({
                "tag": tag,
                "omega": spec.RUNG7_OMEGA[i],
                "body": self._body(p + "chassis"),
                "lane_y": spec.RUNG7_Y[i],
                "free_qpos": self.model.jnt_qposadr[free],
                "free_qvel": self.model.jnt_dofadr[free],
                "acts": [self._act("%s%s_motor" % (p, n))
                         for n, _x, _y in spec.WHEELS],
                "wheel_adr": {n: self.model.jnt_qposadr[
                    self._joint("%s%s_joint" % (p, n))]
                    for n, _x, _y in spec.WHEELS},
            })

    def series(self):
        return ("robots",)

    def blank(self):
        return {"robots": {r["tag"]: {"x": [], "y": [], "z": [],
                                      "wheel_q": {n: [] for n in
                                                  spec.WHEEL_TAGS}}
                           for r in self.robots}}

    def command(self):
        victim = spec.RUNG7_FAULT_ROBOT
        for i, r in enumerate(self.robots):
            omega = r["omega"]
            if self.fault == "stalled_robot" and i == victim:
                # One robot's wheels commanded zero.  Its own distance goes
                # wrong and nobody else's does, which is exactly the claim
                # "each robot is judged against what it would travel ALONE".
                omega = 0.0
            for a in r["acts"]:
                self.data.ctrl[a] = omega
        if self.fault == "lane_offset":
            # One robot is held RUNG7_FAULT_OFFSET_M into its neighbour's
            # lane, without touching its wheels: its distance and its wheel
            # rate stay exactly right and only the pairwise separation moves.
            # That asymmetry is why the rung asserts separation as well as
            # distance -- a fleet can be perfectly commanded and still be in
            # each other's way.
            r = self.robots[victim]
            self.data.qpos[r["free_qpos"] + 1] = (r["lane_y"]
                                                  + spec.RUNG7_FAULT_OFFSET_M)
            self.data.qvel[r["free_qvel"] + 1] = 0.0

    def record(self, out):
        for r in self.robots:
            dst = out["robots"][r["tag"]]
            p = self.data.xpos[r["body"]]
            dst["x"].append(float(p[0]))
            dst["y"].append(float(p[1]))
            dst["z"].append(float(p[2]))
            for n, adr in r["wheel_adr"].items():
                dst["wheel_q"][n].append(float(self.data.qpos[adr]))

    def facts(self):
        return {"n_robots": len(self.robots),
                "omega_cmd": [r["omega"] for r in self.robots],
                "ncon_final": int(self.data.ncon)}


# --------------------------------------------------------------------------
# rung 8 -- a gantry gripper lifts a part off a table
# --------------------------------------------------------------------------

class GripperDriver(Driver):
    """Records ``part_*`` and ``wrist_*``: the part's pose and the gripper's.

    Both are read from the SAME ``mjData``, so ``carry_rel`` is a genuine
    relative pose and not two measurements taken a step apart.  The wrist body
    origin is authored at the part's centre, so their separation is an
    analytic zero -- the reference comes from the SCENE and never from the run.

    THE PADS ARE COMMANDED IN METRES OF CLOSURE, not in newtons, and
    ``scenes.py`` has the measurement that forced that: MuJoCo's true force
    mode drives the pads 24 mm INTO a 60 mm part and ejects it at 1.36 m/s,
    because a force against an acceleration-referenced contact has no bounded
    steady state.  The commanded interference is the contract's own
    ``rung8_bite_m(kp, ke)``, and ``run.py`` reports the contact normal force
    that actually arrived so the grip is a measured number, not a command
    echoed back.

    THE CLOSURE IS RAMPED, over exactly the window the contract's schedule
    allows for it.  A step command on a 3000 N/m servo would put 93 N on a
    0.02 kg pad and arrive at 1.5 m/s; ramping makes the pad meet the part at
    the ramp speed, which is 31 mm/s.  The ramp uses the same shape as the
    contract's own ``rung8_lift_z`` and ``rung8_traverse_x``.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.b_part = self._body("part")
        self.b_wrist = self._body("wrist")
        self.a_trav = self._act("traverse_motor")
        self.a_lift = self._act("lift_motor")
        self.a_pads = [self._act("pad_l_motor"), self._act("pad_r_motor")]
        self.pad_q = [self.model.jnt_qposadr[self._joint("pad_l_joint")],
                      self.model.jnt_qposadr[self._joint("pad_r_joint")]]

    def series(self):
        return ("part_x", "part_y", "part_z", "wrist_x", "wrist_y", "wrist_z")

    def _closure(self, t, bite):
        """Commanded pad closure at ``t``: ramped over the schedule's window.

        Zero until ``T_SETTLE`` (fingers open while the part settles), then a
        constant-rate ramp to ``q_touch + bite`` by ``T_CLOSE``.
        """
        if t <= spec.RUNG8_T_SETTLE:
            return 0.0
        target = scenes.RUNG8_PAD_Q_TOUCH + bite
        span = spec.RUNG8_T_CLOSE - spec.RUNG8_T_SETTLE
        return target * min(1.0, (t - spec.RUNG8_T_SETTLE) / span)

    def command(self):
        t = float(self.data.time)
        self.data.ctrl[self.a_lift] = spec.rung8_lift_z(t)
        self.data.ctrl[self.a_trav] = (
            0.0 if self.fault == "no_traverse" else spec.rung8_traverse_x(t))
        grip = self._closure(t, scenes.RUNG8_BITE)
        if self.fault == "no_grip":
            # THE CAUSAL CONTROL, and the thing rung 4 does not have: the same
            # scene, the same schedule, the fingers never closed.  The part
            # must stay on the table.  Without it, a grasp asserted only from
            # the part's pose cannot tell friction from a weld.
            grip = 0.0
        elif self.fault == "drop_mid_carry" and t >= spec.RUNG8_T_LIFT:
            # The fingers reopen at the top of the lift.  The part falls
            # 0.15 m onto the table and arrives at sqrt(2 g h) = 1.7 m/s,
            # which is what part_speed_max sees; its FINAL pose, sitting on
            # the table under the gripper, looks entirely plausible.
            grip = 0.0
        elif self.fault == "weak_grip":
            # This arm's own extra, kept because it proves something none of
            # the contract's three do: a pinch BELOW the Coulomb bound
            # m g / (2 mu).  The pads are in contact for the whole run and the
            # part still never leaves the table -- the case a contact COUNT
            # scores as a successful grasp.  The bite is the one that develops
            # HALF the bound through the same series stiffness, so the fault
            # is stated in newtons and converted by the contract's own helper.
            weak = spec.rung8_bite_m(scenes.RUNG8_PAD_KP, scenes.RUNG8_KE) \
                * (0.5 * spec.rung8_grip_force_bound() / spec.RUNG8_GRIP_N)
            grip = self._closure(t, weak)
        for a in self.a_pads:
            self.data.ctrl[a] = grip

    def record(self, out):
        p = self.data.xpos[self.b_part]
        w = self.data.xpos[self.b_wrist]
        out["part_x"].append(float(p[0]))
        out["part_y"].append(float(p[1]))
        out["part_z"].append(float(p[2]))
        out["wrist_x"].append(float(w[0]))
        out["wrist_y"].append(float(w[1]))
        out["wrist_z"].append(float(w[2]))

    def facts(self):
        mj, model, data = self.mj, self.model, self.data
        out = {"pad_q_final": [float(data.qpos[a]) for a in self.pad_q],
               "grip_cmd_n": spec.RUNG8_GRIP_N,
               "grip_bound_n": spec.rung8_grip_force_bound(),
               "mu": spec.RUNG8_MU,
               "ncon_final": int(data.ncon)}
        # Explicit-Euler stability of the two position servos, from the
        # COMPILED model's own inertia -- reported, so "these gains are inside
        # the bound" is a measured field rather than a comment in scenes.py.
        for name, joint in (("traverse", "traverse"), ("lift", "lift")):
            act = self._act(name + "_motor")
            inertia = self._dof_M0(joint)
            kp = float(model.actuator_gainprm[act][0])
            kv = -float(model.actuator_biasprm[act][2])
            out[name + "_dof_M0"] = inertia
            out[name + "_kp"] = kp
            out[name + "_kv"] = kv
            out[name + "_kp_dt2_over_m"] = kp * spec.DT ** 2 / inertia
            out[name + "_kv_dt_over_m"] = kv * spec.DT / inertia
        # The measured normal force in each pad-part contact at the end of the
        # run.  The commanded value is RUNG8_GRIP_N; this is what arrived.
        forces = []
        buf = __import__("numpy").zeros(6, dtype=float)
        g_part = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "part_geom")
        for k in range(int(data.ncon)):
            c = data.contact[k]
            if g_part in (c.geom1, c.geom2):
                mj.mj_contactForce(model, data, k, buf)
                forces.append(abs(float(buf[0])))
        out["part_contact_normal_n"] = forces
        return out


# --------------------------------------------------------------------------
# rung 9 -- the pile, and the cube dropped on its outer corner
# --------------------------------------------------------------------------

class PileDriver(Driver):
    """Records ``bodies``: every cube's world-frame centre, metres.

    NOTHING IS COMMANDED.  Rung 9's scene is passive by construction -- 25
    cubes resting and a 26th falling onto them -- so the only actuation an arm
    could add here is a way for the two replicas to differ, which is the one
    thing the rung is measuring.

    The body tags are ``rungs.RUNG9_BODY_TAGS`` verbatim and in the contract's
    order, so an arm cannot silently compare one cube's track against another's
    and report the difference as a determinism failure (or, worse, report a
    reordered pile that happens to agree as a pass).
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.bodies = [(tag, self._body(tag)) for tag in spec.RUNG9_BODY_TAGS]

    def series(self):
        return ("bodies",)

    def blank(self):
        return {"bodies": {tag: {"x": [], "y": [], "z": []}
                           for tag in spec.RUNG9_BODY_TAGS}}

    def record(self, out):
        for tag, bid in self.bodies:
            p = self.data.xpos[bid]
            dst = out["bodies"][tag]
            dst["x"].append(float(p[0]))
            dst["y"].append(float(p[1]))
            dst["z"].append(float(p[2]))

    def facts(self):
        return {"n_bodies": len(self.bodies),
                "ncon_final": int(self.data.ncon),
                "drop_is_dynamic": bool(self.model.body_dofnum[
                    self._body("drop")]),
                "total_mass_kg": float(sum(self.model.body_mass))}


# --------------------------------------------------------------------------
# rung 11 -- the rung-4 rover at N = 1, 4, 8, 16
# --------------------------------------------------------------------------

class Rung11FleetDriver(Driver):
    """``n`` rovers in lanes, each on its OWN constant wheel command.

    ``FleetDriver`` with the fleet read from the contract's sweep rather than
    from rung 7's fixed five.  Each robot's distance is judged against what it
    would travel ALONE at every fleet size, with rung 4's tolerance and no
    N-dependent slack -- so a robot perturbed by a neighbour, or a fleet the
    solver quietly truncated, moves that robot's own number.

    ``stalled_robot`` is an ACTUATION fault and lives here; ``lane_offset`` is
    a SPAWN offset and lives in the scene.  CONTRACT.md section 6 records why
    the second one may not be a per-step write: it costs the body its state and
    reddens the two companions the fault exists to leave alone.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.n = int(self.run.get("n") or spec.RUNG11_N[0])
        self.robots = []
        for i in range(self.n):
            p = "r%02d_" % i
            self.robots.append({
                "tag": "r%02d" % i,
                "omega": spec.rung11_omega(i),
                "body": self._body(p + "chassis"),
                "acts": [self._act("%s%s_motor" % (p, name))
                         for name, _x, _y in spec.WHEELS],
                "wheel_adr": {name: self.model.jnt_qposadr[
                    self._joint("%s%s_joint" % (p, name))]
                    for name, _x, _y in spec.WHEELS},
            })

    def series(self):
        return ("robots",)

    def blank(self):
        return {"robots": {r["tag"]: {"x": [], "y": [], "z": [],
                                      "wheel_q": {n: [] for n in
                                                  spec.WHEEL_TAGS}}
                           for r in self.robots}}

    def command(self):
        for i, r in enumerate(self.robots):
            omega = r["omega"]
            if (self.fault == "stalled_robot" and i == spec.RUNG11_FAULT_ROBOT
                    and self.n == spec.RUNG11_FAULT_N):
                omega = 0.0
            for a in r["acts"]:
                self.data.ctrl[a] = omega

    def record(self, out):
        for r in self.robots:
            dst = out["robots"][r["tag"]]
            p = self.data.xpos[r["body"]]
            dst["x"].append(float(p[0]))
            dst["y"].append(float(p[1]))
            dst["z"].append(float(p[2]))
            for name, adr in r["wheel_adr"].items():
                dst["wheel_q"][name].append(float(self.data.qpos[adr]))

    def facts(self):
        return {"n_robots": self.n,
                "omega_cmd": [r["omega"] for r in self.robots],
                "ncon_final": int(self.data.ncon),
                # The cap this fleet ran under.  -1 is MuJoCo's default and
                # means the constraint arrays are sized from the arena, so the
                # contract's njmax budget has nothing to protect against here.
                "njmax": int(self.model.njmax),
                "nconmax": int(self.model.nconmax),
                "nefc_final": int(self.data.nefc)}


# --------------------------------------------------------------------------
# dispatch
# --------------------------------------------------------------------------

DRIVERS = {0: Rung0, 1: BoxDriver, 2: BoxDriver, 3: HingeDriver,
           4: RoverDriver, 5: RangeSweepDriver, 6: ApproachDriver,
           7: FleetDriver, 8: GripperDriver, 9: PileDriver,
           11: Rung11FleetDriver}


def make(rung, mj, model, data, fault=None, run=None):
    rung = int(rung)
    cls = DRIVERS.get(rung)
    if cls is None:
        raise ValueError("no driver for rung %r" % (rung,))
    return cls(mj, model, data, fault=fault, run=run)


def blank_series(driver):
    """Empty containers for the series a driver fills."""
    if hasattr(driver, "blank"):
        return driver.blank()
    out = {}
    for key in driver.series():
        out[key] = {t: [] for t in spec.WHEEL_TAGS} if key == "wheel_q" else []
    return out


__all__ = ["Driver", "DRIVERS", "make", "blank_series", "obj_id"]
