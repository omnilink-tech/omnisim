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

"""**The scripted T4 driver.** An alternating-leg biped gait, plus the rig.

This is the *controller half of the deliverable*: the phase-B runner copies
this file next to the scene it drives and imports it **by path, with no package
on ``sys.path``** -- which is why nothing here imports from ``ladder`` and the
only dependencies are ``math``, ``os`` and ``mujoco``. A driver that needed the
benchmark's own tree to run would not be a deliverable that stands alone.

**Nothing here is a ladder cell.** A human wrote this gait knowing the tier's
thresholds (``capability-ladder-plan.md`` §2). Its one claim is that the walk
the container describes is **physically achievable** on this column, which
``tasks/T4_humanoid/meta.json`` -> ``container.authored_here.before_the_freeze``
makes a precondition of the freeze. It is also the answer to a second question
the tier asks out loud: *training is permitted and is not required*, and this
reaches the outcome with **no learning of any kind**, ``method: scripted``.

--------------------------------------------------------------------------
The one place this driver differs from every other in this column
--------------------------------------------------------------------------

⚠ **It writes ``mjData.xfrc_applied``, on purpose, and that is the channel T4
measures.** The T1, T2 and T3 drivers write ``data.ctrl`` and nothing else, and
their bring-up records say so as a *guarantee*: nothing held the robot up. T4
is the tier that **permits a support rig and measures it instead of asserting
its absence** (§2 T4), so this driver applies one and the runner records it
tick by tick. The cell the run lands in is decided by that number and by
nothing else.

The rig, stated precisely so the disclosure rule can be honoured
---------------------------------------------------------------

``rig="wrench"`` (the default) applies, to the base and nothing else:

* an **attitude PD** on roll, pitch and yaw -- :data:`ATT_KP`, :data:`ATT_KD`;
* a **lateral catch** on y -- :data:`LAT_KP`, :data:`LAT_KD`;
* ⚠ **and nothing vertical. ``fx = fz = 0`` by construction**, on every tick,
  so the legs carry the whole 25.6 kg and the rig only keeps the trunk from
  toppling and from drifting sideways.

That shape is not a coincidence and not a flourish: it is the shape the only
real support rig this tree has ever measured per channel turned out to have
(``docs/developer/g1-endurance-2026-08-01.md`` §4 -- the vertical carrying
channel read 0.00 N for 0 % of a whole 10 m walk while the attitude channel ran
continuously). It is still a rig, this walk is still **not free-standing**, and
``AGENTS.md``'s humanoid disclosure rule binds every sentence about it.

``rig="none"`` is the identical script with that wrench switched off -- the
tier's ``T4-unsupported`` attempt, and the run whose *falling* is the cleanest
statement of what the two cells mean.

``rig="weld"`` is **not a technique and not a recommendation**: it is the
executable form of the open question the task file records. When the scene
carries the mocap anchor :data:`RIG_ANCHOR`, this driver drives that body
instead of applying any wrench at all, and ``xfrc_applied`` stays identically
zero while the robot is held rigidly upright. See ``BRINGUP_T4.md`` §6.

The gait
--------

Open loop, alternating single support, and every length it needs is **measured
off the compiled model** rather than typed (:func:`measure`) -- the T1, T2 and
T3 drivers' rule, for the same reason: *"a controller built on typed constants
is a controller that silently drives the wrong robot."*::

    cycle           0.9 s, the two legs exactly out of phase
    step length     0.18 m of foot travel per stance
    swing height    0.06 m, a half-sine lift
    stand height    0.50 m, hip pitch axis to sole
    settle          1.5 s standing still before the first step
    ankle           held level: ankle_pitch = -(hip_pitch + knee)
    hip yaw/roll    commanded to zero; the rig owns the lateral axis

⚠ **The recorded recipe's ankle-to-sole constant is wrong by 20 mm and this
driver does not copy it.** The scratch harness used ``ANKLE_TO_SOLE = 0.04``,
described as *"ankle_roll offset 0.02 + foot half height 0.02"* -- which drops
the foot box's own ``-0.02 m`` origin offset. Measured off the compiled model
the true distance is **0.06 m**, and the container's own ``PROVENANCE.txt``
agrees arithmetically (*"with every joint at zero ... the sole sits 0.725 m
below the base origin"*: 0.105 + 0.28 + 0.28 + 0.06). :func:`measure` reads it
rather than believing either.

⚠ **And the recorded recipe's leg IK walks the robot BACKWARDS.** Its hip-pitch
solution is ``hip = alpha + beta``; on this robot's ``+y`` hip pitch axis a
*positive* hip pitch swings the foot **backward**, so the correct solution is
``hip = beta - alpha``. The scratch harness's self-check only ever solved the
standing pose, where ``alpha = 0`` and the two agree -- so the error was
invisible there and produced a robot that walked 24.94 m in the direction
opposite to the one it was commanded. :func:`setup` here checks the IK at
**non-zero x** as well, which is what catches it. See ``BRINGUP_T4.md`` §5.1.
"""

from __future__ import annotations

import math
import os

# --- what the gait chooses (everything else is measured) ---------------------

STAND_HEIGHT_M = 0.50      # commanded hip-pitch-axis-to-sole distance
STEP_LENGTH_M = 0.18       # foot travel per stance phase
SWING_HEIGHT_M = 0.06      # peak lift of the swinging foot
CYCLE_S = 0.9              # one full cycle: one step with each leg
SETTLE_S = 1.5             # standing still before the first step

# The speed the STANCE FOOT is dragged backwards at, which is the speed a
# non-slipping foot drives the body forward at. Derived, not chosen: the foot
# covers STEP_LENGTH_M during the half-cycle it is planted.
NOMINAL_SPEED_MPS = STEP_LENGTH_M / (0.5 * CYCLE_S)

# The support rig. Attitude first, because that is what a biped needs and what
# the only measured rig in this tree spends its authority on.
ATT_KP = 400.0             # N.m/rad on roll, pitch and yaw
ATT_KD = 40.0              # N.m.s/rad
LAT_KP = 120.0             # N/m on y
LAT_KD = 20.0              # N.s/m

LEGS = ("l", "r")
PHASE = {"l": 0.0, "r": 0.5}          # exactly out of phase: single support
BASE_BODY = "base_link"
RIG_ANCHOR = "rig_anchor"             # only present when the scene built one
JOINTS = {leg: ("hip_yaw_%s_joint" % leg, "hip_roll_%s_joint" % leg,
                "hip_pitch_%s_joint" % leg, "knee_%s_joint" % leg,
                "ankle_pitch_%s_joint" % leg, "ankle_roll_%s_joint" % leg)
          for leg in LEGS}

# The ONE knob this driver exposes, and it exists for a specific published
# measurement rather than for convenience: the tier publishes TWO CELLS FROM
# ONE RECORDING, and the pair that makes the boundary legible is this script
# with the wrench on and the same script with it off. A measurement that can
# only be reproduced by editing a source file is not one a third party can
# re-derive. Unset, the rig is on.
#
# ``weld`` is not selected here -- it is detected from the scene, because a
# deliverable that carries a mocap anchor and a weld equality IS a welded
# deliverable whatever an environment variable says.
RIG_ENV = "LADDER_T4_RIG"
RIG_WRENCH = "wrench"
RIG_NONE = "none"
RIG_WELD = "weld"

_S = {}


def commanded_rig():
    """``"wrench"`` or ``"none"`` for this run. See :data:`RIG_ENV`."""
    want = (os.environ.get(RIG_ENV, "") or RIG_WRENCH).strip().lower()
    return want if want in (RIG_WRENCH, RIG_NONE) else RIG_WRENCH


# --- geometry, read off the compiled model -----------------------------------


def _jid(mujoco, model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def _bid(mujoco, model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def measure(mujoco, model, data):
    """Everything the gait needs, measured with the legs straightened.

    Called once. The hinges are temporarily zeroed so the chain is straight and
    the link lengths fall out of the joint anchors; ``data`` is restored before
    returning. Nothing in this function is a constant somebody typed, which is
    what makes the driver drive *this* robot rather than the one its author
    remembered.
    """
    jid, qadr, aid = {}, {}, {}
    for leg in LEGS:
        for name in JOINTS[leg]:
            i = _jid(mujoco, model, name)
            if i < 0:
                raise ValueError("the scene has no joint %r" % name)
            jid[name] = i
            qadr[name] = int(model.jnt_qposadr[i])
            a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                  "pos_%s" % name)
            if a < 0:
                raise ValueError("the scene has no actuator for %r" % name)
            aid[name] = a
    base = _bid(mujoco, model, BASE_BODY)
    if base < 0:
        raise ValueError("the scene has no body %r" % BASE_BODY)

    saved = data.qpos.copy()
    for name in jid:
        data.qpos[qadr[name]] = 0.0
    mujoco.mj_forward(model, data)

    origin = [float(v) for v in data.xpos[base]]
    hip_at, foot_geom = {}, {}
    l1 = l2 = ankle_to_sole = foot_forward = None
    for leg in LEGS:
        _yaw, _roll, hp, kn, ap, _ar = JOINTS[leg]
        hip = data.xanchor[jid[hp]]
        knee = data.xanchor[jid[kn]]
        ankle = data.xanchor[jid[ap]]
        hip_at[leg] = [float(hip[k]) - origin[k] for k in range(3)]
        l1 = float(abs(knee[2] - hip[2]))
        l2 = float(abs(ankle[2] - knee[2]))
        foot = int(model.jnt_bodyid[jid[JOINTS[leg][5]]])
        if int(model.body_geomnum[foot]) <= 0:
            raise ValueError("foot %r carries no collision geom, so it has no "
                             "sole to stand on" % leg)
        g = int(model.body_geomadr[foot])
        foot_geom[leg] = g
        # The sole is the BOTTOM of the foot box, not the ankle: the box sits
        # 0.02 m below its own body origin and is 0.04 m thick, and the recipe
        # this rebuilds dropped one of those two terms.
        ankle_to_sole = float(ankle[2] - (float(data.geom_xpos[g][2])
                                          - float(model.geom_size[g][2])))
        foot_forward = float(data.geom_xpos[g][0] - ankle[0])

    limits = {name: (float(model.jnt_range[jid[name]][0]),
                     float(model.jnt_range[jid[name]][1])) for name in jid}

    data.qpos[:] = saved
    mujoco.mj_forward(model, data)

    knee_lo, knee_hi = limits["knee_l_joint"]
    reach = tuple(sorted((math.sqrt(l1 * l1 + l2 * l2
                                    + 2 * l1 * l2 * math.cos(q)) + ankle_to_sole)
                         for q in (knee_lo, knee_hi)))
    return {"jid": jid, "qadr": qadr, "aid": aid, "base": base,
            "hip_at": hip_at, "foot_geom": foot_geom,
            "L1": l1, "L2": l2, "ankle_to_sole": ankle_to_sole,
            "foot_centre_ahead_of_ankle": foot_forward,
            "half_stance": abs(hip_at["l"][1]),
            "limits": limits, "reach_hip_to_sole": reach}


# --- inverse kinematics -------------------------------------------------------


def solve_leg(st, x, z):
    """``(hip_pitch, knee, ankle_pitch)`` putting the SOLE at ``(x, -z)``.

    Both are relative to that leg's own hip **pitch** axis, in the base's
    sagittal plane: ``x`` forward, ``z`` the downward distance to the sole.
    Exact, not iterative, and its sign convention is the one thing the recipe
    this rebuilds got wrong:

    on this robot the hip pitch axis is ``+y``, so rotating the thigh by
    ``+theta`` maps a point at ``(0, 0, -L)`` to ``(-L sin theta, 0,
    -L cos theta)`` -- **positive hip pitch swings the foot backwards**. The
    solution is therefore ``hip = beta - alpha`` and not ``alpha + beta``, and
    the two agree exactly when ``x = 0``, which is why a self-check that only
    ever solves the standing pose cannot see the difference.

    Returns ``None`` when the point is outside the leg's reach or outside a
    joint limit.
    """
    l1, l2 = st["L1"], st["L2"]
    za = float(z) - st["ankle_to_sole"]
    if za <= 0.0:
        return None
    d = math.hypot(float(x), za)
    c = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if abs(c) > 1.0:
        return None
    phi2 = math.acos(max(-1.0, min(1.0, c)))          # the knee, folded back
    alpha = math.atan2(float(x), za)
    beta = math.atan2(l2 * math.sin(phi2), l1 + l2 * math.cos(phi2))
    hip = beta - alpha
    knee = -phi2
    ankle = -(hip + knee)
    for name, v in (("hip_pitch", hip), ("knee", knee), ("ankle_pitch", ankle)):
        lo, hi = st["limits"]["%s_l_joint" % name]
        if not (lo <= v <= hi):
            return None
    return hip, knee, ankle


# --- the gait -----------------------------------------------------------------


def foot_target(leg, t_s, *, stand=STAND_HEIGHT_M, step=STEP_LENGTH_M,
                lift=SWING_HEIGHT_M, cycle=CYCLE_S, settle=SETTLE_S):
    """``(x, z, phase_name)`` for one leg's sole at simulated time ``t_s``.

    Relative to that leg's own hip pitch axis, which is the frame
    :func:`solve_leg` solves in, so the whole gait is expressed once and solved
    once. ``x`` forward, ``z`` the downward distance.

    The two legs are exactly out of phase and the duty factor is 0.5, so this
    is **single support**: at every instant one foot is planted and the other
    is in the air. That is the whole difficulty of the tier -- a four-legged
    robot can keep three feet down and be statically stable, and a two-legged
    one cannot.
    """
    if t_s < float(settle):
        return 0.0, float(stand), "settle"
    p = (((float(t_s) - float(settle)) / float(cycle)) + PHASE[leg]) % 1.0
    if p < 0.5:                                    # planted: tracks backwards
        u = p / 0.5
        return float(step) * (0.5 - u), float(stand), "stance"
    u = (p - 0.5) / 0.5                            # in the air: forward + up
    return (float(step) * (-0.5 + u),
            float(stand) - float(lift) * math.sin(math.pi * u), "swing")


def joint_targets(st, t_s, **kw):
    """``{joint: angle}`` for the whole robot at ``t_s``. Never raises."""
    out = {}
    for leg in LEGS:
        x, z, _phase = foot_target(leg, t_s, **kw)
        q = solve_leg(st, x, z)
        if q is None:
            q = st["last"].get(leg)
            if q is None:
                continue
        st["last"][leg] = q
        yaw, roll, hp, kn, ap, ar = JOINTS[leg]
        out[hp], out[kn], out[ap] = q
        # The rig owns the lateral axis; the gait commands these to zero and
        # says so rather than leaving them unwritten.
        out[yaw] = out[roll] = out[ar] = 0.0
    return out


# --- the support rig ----------------------------------------------------------


def base_attitude(model, data, base):
    """``(roll, pitch, yaw)`` of the base, from ``mjData.xmat`` directly.

    No quaternion and no Euler convention from anywhere else: ``xmat`` is a
    world-from-body 3x3 the engine integrated, and these three are read off it.
    """
    r = data.xmat[base].reshape(3, 3)
    roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
    pitch = -math.asin(max(-1.0, min(1.0, float(r[2, 0]))))
    yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    return roll, pitch, yaw


def support_wrench(model, data, base, *, att_kp=ATT_KP, att_kd=ATT_KD,
                   lat_kp=LAT_KP, lat_kd=LAT_KD):
    """The rig's applied wrench this tick: ``(fx, fy, fz, tx, ty, tz)``.

    ⚠ ``fx`` and ``fz`` are **zero by construction, not by tuning**: there is
    no expression in this function that could make either non-zero. The legs
    carry the whole robot; the rig keeps the trunk from toppling and from
    drifting sideways, and that claim is checkable by reading the return
    statement rather than by trusting a number.
    """
    roll, pitch, yaw = base_attitude(model, data, base)
    w = data.cvel[base]                     # [angular(3), linear(3)], world
    tx = -att_kp * roll - att_kd * float(w[0])
    ty = -att_kp * pitch - att_kd * float(w[1])
    tz = -att_kp * yaw - att_kd * float(w[2])
    fy = -lat_kp * float(data.xpos[base][1]) - lat_kd * float(w[4])
    return 0.0, fy, 0.0, tx, ty, tz


# --- the driver interface the phase-B runner calls ---------------------------


def setup(model, data):
    """Called once, after the model loads and before any step.

    Measures the robot, **falsifies its own inverse kinematics against the
    compiled forward kinematics over a whole gait cycle**, and puts the legs
    into the standing pose so the settle window is the robot standing still
    rather than the robot falling over.

    Sixteen of the seventeen probes are points on the trajectory this gait
    actually commands, and that matters: the recipe this rebuilds checked only
    the standing pose, where the sagittal offset is zero, both sign
    conventions agree and a hip solution that walks the robot backwards is
    invisible. Any commanded point the robot **cannot** reach is recorded by
    name in ``ik_targets_this_robot_cannot_reach`` rather than silently held
    at the previous solution.
    """
    import mujoco
    st = measure(mujoco, model, data)
    st["last"] = {}
    st["rig"] = (RIG_WELD if _bid(mujoco, model, RIG_ANCHOR) >= 0
                 else commanded_rig())
    st["mocap"] = None
    if st["rig"] == RIG_WELD:
        anchor = _bid(mujoco, model, RIG_ANCHOR)
        st["mocap"] = int(model.body_mocapid[anchor])
        st["anchor_z"] = float(data.xpos[anchor][2])
    _S.clear()
    _S.update(st)

    q = joint_targets(st, 0.0)
    if len(q) != 12:
        raise ValueError("the standing pose is outside this robot's joint "
                         "limits: solved %d of 12" % len(q))
    for name, v in q.items():
        data.qpos[st["qadr"][name]] = v
        data.ctrl[st["aid"][name]] = v
    mujoco.mj_forward(model, data)

    residual, checks, unsolved = 0.0, [], []
    probes = [(0.0, STAND_HEIGHT_M)]
    probes += [foot_target("l", SETTLE_S + k * CYCLE_S / 16.0)[:2]
               for k in range(16)]
    for x, z in probes:
        got = solve_leg(st, x, z)
        if got is None:
            unsolved.append({"x": round(x, 5), "z": round(z, 5)})
            checks.append({"x": round(x, 5), "z": round(z, 5),
                           "error": "no solution -- outside a joint limit or "
                                    "the leg's reach"})
            continue
        for name, v in zip(("hip_pitch_l_joint", "knee_l_joint",
                            "ankle_pitch_l_joint"), got):
            data.qpos[st["qadr"][name]] = v
        mujoco.mj_forward(model, data)
        g = st["foot_geom"]["l"]
        sole = [float(data.geom_xpos[g][0]),
                float(data.geom_xpos[g][2]) - float(model.geom_size[g][2])]
        hip = st["hip_at"]["l"]
        want = [float(data.xpos[st["base"]][0]) + hip[0] + x,
                float(data.xpos[st["base"]][2]) + hip[2] - z]
        # The foot box's centre sits ahead of the ankle; the IK places the
        # ankle, so the sole's own x is offset by exactly that much and the
        # check subtracts the measured offset rather than tolerating it.
        err = max(abs(sole[0] - st["foot_centre_ahead_of_ankle"] - want[0]),
                  abs(sole[1] - want[1]))
        checks.append({"x": round(x, 5), "z": round(z, 5),
                       "residual_m": round(err, 9)})
        residual = max(residual, err)

    for name, v in q.items():
        data.qpos[st["qadr"][name]] = v
    mujoco.mj_forward(model, data)

    for key, value in (("ik_residual_m", residual), ("ik_checks", checks),
                       ("ik_targets_this_robot_cannot_reach", unsolved),
                       ("stand_pose_rad",
                        {k: round(float(v), 6) for k, v in q.items()})):
        st[key] = value
        _S[key] = value
    return st


def control(model, data, t_s):
    """Called every physics step with the simulated time since recording began.

    Writes ``data.ctrl`` for the gait, and -- unlike every other driver in this
    column -- ``data.xfrc_applied`` for the rig, which is exactly the channel
    T4.4 measures. It reads the base's pose and angular velocity back to do it,
    so the RIG is a feedback controller even though the GAIT is open loop, and
    that distinction is stated rather than glossed: a support rig that could
    not see the robot would not be a support rig.
    """
    st = _S
    if not st:
        setup(model, data)
        st = _S
    for name, v in joint_targets(st, float(t_s)).items():
        data.ctrl[st["aid"][name]] = v

    rig = st.get("rig", RIG_WRENCH)
    if rig == RIG_WRENCH:
        data.xfrc_applied[st["base"]] = support_wrench(model, data, st["base"])
    elif rig == RIG_WELD and st.get("mocap") is not None:
        # ⚠ The demonstration rig: nothing is applied through xfrc_applied and
        # the robot is held rigidly anyway. See the module docstring.
        travelled = max(0.0, float(t_s) - SETTLE_S) * NOMINAL_SPEED_MPS
        data.mocap_pos[st["mocap"]] = [travelled, 0.0, st.get("anchor_z", 0.0)]
        data.mocap_quat[st["mocap"]] = [1.0, 0.0, 0.0, 0.0]


def describe():
    """What this driver measured and what it decided. Recorded in the run."""
    if not _S:
        return {}
    out = {k: _S.get(k) for k in
           ("hip_at", "L1", "L2", "ankle_to_sole",
            "foot_centre_ahead_of_ankle", "half_stance", "limits",
            "reach_hip_to_sole", "ik_residual_m", "ik_checks",
            "ik_targets_this_robot_cannot_reach", "stand_pose_rad")}
    out["chosen_constants"] = {
        "STAND_HEIGHT_M": STAND_HEIGHT_M, "STEP_LENGTH_M": STEP_LENGTH_M,
        "SWING_HEIGHT_M": SWING_HEIGHT_M, "CYCLE_S": CYCLE_S,
        "SETTLE_S": SETTLE_S, "NOMINAL_SPEED_MPS": NOMINAL_SPEED_MPS,
        "ATT_KP": ATT_KP, "ATT_KD": ATT_KD,
        "LAT_KP": LAT_KP, "LAT_KD": LAT_KD}
    out["rig"] = _S.get("rig")
    out["rig_env_var"] = RIG_ENV
    out["duty_factor"] = 0.5
    out["feet_on_the_ground_during_a_cycle"] = 1
    out["phase_offsets"] = dict(PHASE)
    out["gait_closed_loop"] = False
    out["rig_closed_loop"] = _S.get("rig") in (RIG_WRENCH, RIG_WELD)
    out["vertical_carry"] = (
        "ZERO BY CONSTRUCTION: support_wrench() returns fx = fz = 0 on every "
        "tick and there is no expression in it that could return anything else"
        if _S.get("rig") == RIG_WRENCH else
        ("the whole reaction is a WELD CONSTRAINT and none of it appears in "
         "xfrc_applied -- which is the hole this rig exists to demonstrate"
         if _S.get("rig") == RIG_WELD else
         "nothing is applied at all: the wrench is switched off"))
    out["writes"] = (
        "data.ctrl for the gait, and data.xfrc_applied for the rig -- the "
        "channel T4.4 measures. Never qpos and never qvel"
        if _S.get("rig") == RIG_WRENCH else
        ("data.ctrl for the gait and data.mocap_pos for the rig anchor. "
         "NOTHING reaches data.xfrc_applied" if _S.get("rig") == RIG_WELD
         else "data.ctrl only"))
    return out


__all__ = ["ATT_KD", "ATT_KP", "BASE_BODY", "CYCLE_S", "JOINTS", "LAT_KD",
           "LAT_KP", "LEGS", "NOMINAL_SPEED_MPS", "PHASE", "RIG_ANCHOR",
           "RIG_ENV", "RIG_NONE", "RIG_WELD", "RIG_WRENCH", "SETTLE_S",
           "STAND_HEIGHT_M", "STEP_LENGTH_M", "SWING_HEIGHT_M",
           "base_attitude", "commanded_rig", "control", "describe",
           "foot_target", "joint_targets", "measure", "setup", "solve_leg",
           "support_wrench"]
