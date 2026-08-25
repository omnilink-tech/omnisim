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

"""**The scripted T3 driver.** A four-beat crawl, open loop, no learning.

This is the *controller half of the deliverable*: the phase-B runner copies
this file next to the scene it drives and imports it **by path, with no package
on ``sys.path``** -- which is why nothing here imports from ``ladder`` and the
only dependencies are ``math`` and ``mujoco``. A driver that needed the
benchmark's own tree to run would not be a deliverable that stands alone.

**Nothing here is a ladder cell.** A human wrote this gait knowing the tier's
thresholds (``capability-ladder-plan.md`` §2). Its one claim is that the walk
the container describes is **physically achievable** on this column -- which
``tasks/T3_quadruped/meta.json`` -> ``container.authored_here.before_the_freeze``
makes a precondition of the freeze, because a robot no column can walk makes
the cell a measurement of our asset rather than of anybody's agent. It is also
the answer to a second question the tier asks out loud: *training is permitted
and is not required*, and this reaches the outcome with **no learning of any
kind**, ``method: scripted``.

Nothing is teleported and nothing is held up
--------------------------------------------

Every joint is driven through a position servo writing ``data.ctrl`` and
nothing else -- never ``qpos``, never ``qvel``, and in particular never
``xfrc_applied``, which is the channel T3.4 measures. The runner records the
applied wrench on the base for the whole run and it is identically zero; that
is what makes *"nothing held it up"* an observation rather than a claim.

The robot is measured, never typed
----------------------------------

Every length this gait needs -- the two link lengths, where each hip sits on
the body, the lateral offset of each thigh axis from its own hip axis, the foot
sphere's radius, every joint limit -- is read off the **compiled model** in
:func:`measure`. The T1 and T2 drivers' rule, for the same reason: *"a
controller built on typed constants is a controller that silently drives the
wrong robot."* :func:`setup` then **verifies its own inverse kinematics
numerically** against the compiled forward kinematics and records the residual,
so a solution that is algebraically wrong is caught in the run record rather
than blamed on the physics.

The gait
--------

A **four-beat crawl** with duty factor 0.75 -- three feet on the ground at all
times, which is what makes it *statically stable* and, on this robot, what
makes it work at all::

    cycle              2.2 Hz
    order              fl -> rr -> fr -> rl   (diagonal-ish, one foot at a time)
    commanded speed    0.50 m/s
    stance             the foot is planted: it tracks backwards relative to the
                       body at exactly the commanded speed, so a foot that does
                       not slip drives the body forward at that speed
    swing              a smoothstep return with a 0.07 m sinusoidal lift
    lateral stance     feet held +/- 0.14 m off centre
    body sway          a lateral sinusoid in phase with the swing order, so the
                       centre of mass moves toward the support triangle before
                       the leg that would otherwise be inside it lifts. Its
                       AMPLITUDE IS DERIVED, not chosen -- see below
    settle             1.5 s standing still before the first step

⚠ **The sway amplitude is derived from statics, and the recorded recipe's
number does not walk.** With the feet at ``+/- lateral`` and the hips at
``+/- hip_x``, lifting any one foot leaves a support triangle whose nearest
edge is the diagonal **through the body centre** -- so a level body standing on
three feet has a stability margin of exactly **zero**, and the whole job of the
sway is to buy some. :func:`sway_amplitude` solves for the lateral shift that
puts the body centre :data:`STABILITY_MARGIN_M` inside that diagonal, from the
hip spacing this driver **measured** off the model. On the shipped robot that
is **0.019 m**.

The scratch harness that first demonstrated this walk recorded *"sinusoidal
lateral body sway +/- 0.06 m"*. Re-implemented here, that number **does not
walk**: at 2.2 Hz a 0.06 m sinusoid demands ``A(2*pi*f)^2 = 11.5 m/s^2`` of
lateral acceleration against a friction ceiling of ``mu.g = 11.8 m/s^2``, so
the feet are asked for essentially the entire friction budget just to shake the
body sideways. Measured on the shipped scene, same everything else: **0.18 m
of forward travel in 20 s and a mean of 1.64 feet on the ground** against the
three the duty factor predicts. The derived 0.019 m walks the full run
(8.46 m over the same 20 s, 2.67 feet down). That divergence is reported
in ``BRINGUP_T3.md`` §4.2 rather than smoothed over, and the margin is stated
as a length in metres so a reader can attack the number rather than a fudge
factor.

⚠ **This gait is deliberately slow-ish and statically stable, and that had a
consequence for the tier.** At the commanded 0.50 m/s the base bobs about
0.009 m RMS; driven to 0.27 m/s the same crawl bobs **0.0018 m** while walking
16.5 m without falling. T3.3's vertical-bob conjunct used to require 0.005 m
and therefore *failed a robot that was plainly walking* -- which is what got
that conjunct retired as a pass condition on 2026-08-02. See
``BRINGUP_T3.md`` §5.2 and ``ladder/graders/t3_core.py`` reading 5b. This
driver's ``--speed`` is what reproduces both halves of that measurement.
"""

from __future__ import annotations

import math
import os

# --- what the gait chooses (everything else is measured) ---------------------

STAND_HEIGHT_M = 0.24      # commanded hip-to-foot-centre distance
LATERAL_STANCE_M = 0.14    # commanded foot offset from the body centre line
BODY_SPEED_MPS = 0.50      # commanded forward speed
CYCLE_HZ = 2.2
DUTY = 0.75                # fraction of the cycle a foot spends planted
SWING_HEIGHT_M = 0.07
# How far inside the nearest edge of the three-foot support triangle the body
# centre is put while a leg is in the air. This is what the lateral sway is
# FOR, so it is stated as the length it buys rather than as an amplitude
# somebody liked -- :func:`sway_amplitude` turns it into an amplitude using the
# hip spacing measured off the model.
STABILITY_MARGIN_M = 0.015
SETTLE_S = 1.5             # standing still before the first step
DURATION_S = 70.0          # the recorded recipe's own run length

LEGS = ("fl", "fr", "rl", "rr")
# Phase offsets, as fractions of a cycle. A leg swings when its own phase is in
# [DUTY, 1). With these four offsets exactly one foot is in the air at a time
# and the order round the body is fl -> rr -> fr -> rl.
PHASE = {"fl": 0.0, "rr": 0.25, "fr": 0.5, "rl": 0.75}
SIDE = {"fl": 1.0, "rl": 1.0, "fr": -1.0, "rr": -1.0}   # +1 == left

BASE_BODY = "base_link"
JOINTS = {leg: ("hip_%s_joint" % leg, "thigh_%s_joint" % leg,
                "knee_%s_joint" % leg) for leg in LEGS}

# The ONE knob this driver exposes, and it exists for a specific published
# measurement rather than for convenience: the bob-versus-speed sweep in
# ``BRINGUP_T3.md`` §5.2 is what retired T3.3's vertical-bob pass condition, and
# a measurement that can only be reproduced by editing a source file is not a
# measurement a third party can re-derive. Unset, the commanded speed is
# :data:`BODY_SPEED_MPS`.
SPEED_ENV = "LADDER_T3_SPEED_MPS"

_S = {}


def commanded_speed():
    """The commanded body speed for this run, m/s. See :data:`SPEED_ENV`."""
    try:
        v = float(os.environ.get(SPEED_ENV, "") or BODY_SPEED_MPS)
    except ValueError:
        return BODY_SPEED_MPS
    return v if v > 0.0 else BODY_SPEED_MPS


# --- geometry, read off the compiled model -----------------------------------


def _jid(mujoco, model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def _bid(mujoco, model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def measure(mujoco, model, data):
    """Everything the gait needs, measured with the legs straightened.

    Called once. The hinges are temporarily zeroed so the chain is straight and
    the link lengths fall out of the joint anchors; ``data`` is restored before
    returning.
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
    hip_at, thigh_off, foot_geom = {}, {}, {}
    l1 = l2 = None
    foot_r = None
    for leg in LEGS:
        hj, tj, kj = JOINTS[leg]
        hip = data.xanchor[jid[hj]]
        thigh = data.xanchor[jid[tj]]
        knee = data.xanchor[jid[kj]]
        hip_at[leg] = [float(hip[k]) - origin[k] for k in range(3)]
        thigh_off[leg] = float(thigh[1] - hip[1])
        l1 = float(abs(knee[2] - thigh[2]))
        shank = int(model.jnt_bodyid[jid[kj]])
        g = int(model.body_geomadr[shank])
        if int(model.body_geomnum[shank]) <= 0:
            raise ValueError("shank %r carries no collision geom, so it has no "
                             "foot to stand on" % leg)
        foot_geom[leg] = g
        foot_r = float(model.geom_size[g][0])
        l2 = float(abs(float(data.geom_xpos[g][2]) - knee[2]))

    limits = {name: (float(model.jnt_range[jid[name]][0]),
                     float(model.jnt_range[jid[name]][1])) for name in jid}

    data.qpos[:] = saved
    mujoco.mj_forward(model, data)
    return {"jid": jid, "qadr": qadr, "aid": aid, "base": base,
            "hip_at": hip_at, "thigh_offset": thigh_off,
            "foot_geom": foot_geom, "foot_radius": foot_r,
            "L1": l1, "L2": l2, "limits": limits,
            "reach": (abs(l1 - l2), l1 + l2)}


# --- inverse kinematics -------------------------------------------------------


def solve_leg(st, leg, px, py, pz):
    """Three joint angles putting this leg's FOOT CENTRE at ``(px, py, pz)``.

    The target is relative to that leg's own hip anchor, in the base's frame.
    Returns ``None`` when the point is outside the leg's reach or outside a
    joint limit -- and on this robot the second happens first: the knee is
    bounded to ``[-2.50, -0.20]`` rad, so hip-to-foot distance is confined to
    ``0.040 .. 0.396 m`` and the ``-0.20`` upper bound is what keeps the
    two-link solution off its own singularity.

    The abduction half is exact rather than a small-angle approximation, and it
    **includes the thigh axis's own lateral offset from the hip axis**: the
    thigh joint sits 0.055 m outboard of the hip joint on this robot, so the
    leg does not rotate about the hip axis in a plane through the hip. Rolling
    that offset in is the difference between a foot that lands where it was
    told and one that lands a centimetre inboard of it on every step.
    """
    d = st["thigh_offset"][leg]
    l1, l2 = st["L1"], st["L2"]
    rho = math.hypot(py, pz)
    if rho < abs(d) or rho <= 0.0:
        return None
    # (d, fz) rotated by the abduction angle must land on (py, pz)
    fz = -math.sqrt(max(rho * rho - d * d, 0.0))
    q0 = _wrap(math.atan2(pz, py) - math.atan2(fz, d))

    fx = float(px)
    r2 = fx * fx + fz * fz
    c = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if abs(c) > 1.0:
        return None
    q2 = -math.acos(max(-1.0, min(1.0, c)))          # knee bends backwards
    q1 = math.atan2(-fx, -fz) - math.atan2(l2 * math.sin(q2),
                                           l1 + l2 * math.cos(q2))
    hj, tj, kj = JOINTS[leg]
    out = {hj: q0, tj: _wrap(q1), kj: q2}
    for name, v in out.items():
        lo, hi = st["limits"][name]
        if not (lo <= v <= hi):
            return None
    return out


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# --- the gait -----------------------------------------------------------------


def _smoothstep(f):
    f = max(0.0, min(1.0, f))
    return f * f * (3.0 - 2.0 * f)


def sway_amplitude(st, lateral=LATERAL_STANCE_M, margin=STABILITY_MARGIN_M):
    """The lateral body shift that buys ``margin`` of static stability, metres.

    Lifting one foot leaves a support triangle whose nearest edge is the
    diagonal between the two feet on the opposite corners -- and on a
    rectangular stance that diagonal passes **through the body centre**, so a
    level body has a margin of exactly zero and any disturbance tips it. Shift
    the body by ``s`` along +y and the perpendicular distance to that diagonal
    becomes ``s * hip_x / hypot(hip_x, lateral)``; inverting it gives the shift
    a stated margin needs.

    ``hip_x`` is **measured** off the model (half the fore-aft hip spacing), so
    a robot with a longer body gets a different answer from the same rule
    rather than the same number.
    """
    hip_x = max(abs(st["hip_at"][leg][0]) for leg in LEGS)
    if hip_x <= 0.0:
        return 0.0
    return float(margin) * math.hypot(hip_x, float(lateral)) / hip_x


def foot_target(st, leg, t_s, *, speed=None, cycle=CYCLE_HZ,
                stand=STAND_HEIGHT_M, lateral=LATERAL_STANCE_M,
                lift=SWING_HEIGHT_M, sway=None, margin=STABILITY_MARGIN_M,
                settle=SETTLE_S):
    """``(px, py, pz, phase_name)`` for one leg at simulated time ``t_s``.

    Relative to that leg's hip anchor, in the base's frame -- which is the
    frame :func:`solve_leg` solves in, so the whole gait is expressed once and
    solved once.
    """
    hip = st["hip_at"][leg]
    if speed is None:
        speed = commanded_speed()
    half = 0.5 * float(speed) * float(DUTY) / float(cycle)   # half the stride
    if sway is None:
        sway = sway_amplitude(st, lateral=lateral, margin=margin)

    if t_s < float(settle):
        p = 0.0
        px, pz, name = 0.0, -float(stand), "settle"
        swayed = 0.0
    else:
        p = ((t_s - float(settle)) * float(cycle)) % 1.0
        swayed = float(sway) * math.sin(2.0 * math.pi * p)
        s = (p - PHASE[leg]) % 1.0
        if s < DUTY:                                   # planted
            px = half - 2.0 * half * (s / DUTY)
            pz = -float(stand)
            name = "stance"
        else:                                          # in the air
            f = (s - DUTY) / (1.0 - DUTY)
            px = -half + 2.0 * half * _smoothstep(f)
            pz = -float(stand) + float(lift) * math.sin(math.pi * f)
            name = "swing"
    py = SIDE[leg] * float(lateral) - hip[1] - swayed
    return px, py, pz, name


def joint_targets(st, t_s, **kw):
    """``{joint: angle}`` for the whole robot at ``t_s``. Never raises."""
    out = {}
    for leg in LEGS:
        px, py, pz, _phase = foot_target(st, leg, t_s, **kw)
        q = solve_leg(st, leg, px, py, pz)
        if q is None:
            q = st["last"].get(leg)
            if q is None:
                continue
        st["last"][leg] = q
        out.update(q)
    return out


# --- the driver interface the phase-B runner calls ---------------------------


def setup(model, data):
    """Called once, after the model loads and before any step.

    Measures the robot, checks its own IK against the compiled forward
    kinematics, and puts the legs into the standing pose so the settle window
    is the robot standing still rather than the robot falling over.
    """
    import mujoco
    st = measure(mujoco, model, data)
    st["last"] = {}
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

    # The IK's own falsifier, run on every deployment rather than promised:
    # solve the standing pose, apply it, and measure where the feet actually
    # ended up against where they were asked to be.
    residual = 0.0
    for leg in LEGS:
        px, py, pz, _n = foot_target(st, leg, 0.0)
        hip = st["hip_at"][leg]
        want = [float(data.xpos[st["base"]][k])
                + [hip[0] + px, hip[1] + py, hip[2] + pz][k] for k in range(3)]
        got = [float(v) for v in data.geom_xpos[st["foot_geom"][leg]]]
        residual = max(residual, max(abs(a - b) for a, b in zip(want, got)))
    # Written through to the module state as well as to the returned dict:
    # ``describe()`` reads ``_S``, and ``_S.update(st)`` above copied the keys
    # that existed at that moment rather than aliasing the dict. A residual
    # that reached the caller and not the run record would be a self-check
    # nobody could read afterwards.
    for key, value in (("ik_residual_m", residual),
                       ("stand_pose_rad",
                        {k: round(float(v), 6) for k, v in q.items()})):
        st[key] = value
        _S[key] = value
    return st


def control(model, data, t_s):
    """Called every physics step with the simulated time since recording began.

    Writes ``data.ctrl`` and nothing else. Open loop: nothing here reads a
    sensor, a pose or a contact back out of the simulator, which is what makes
    ``method: scripted`` an honest label rather than a modest one.
    """
    st = _S
    if not st:
        setup(model, data)
        st = _S
    for name, v in joint_targets(st, float(t_s)).items():
        data.ctrl[st["aid"][name]] = v


def describe():
    """What this driver measured and what it decided. Recorded in the run."""
    if not _S:
        return {}
    out = {k: _S.get(k) for k in
           ("hip_at", "thigh_offset", "L1", "L2", "foot_radius", "reach",
            "limits", "ik_residual_m", "stand_pose_rad")}
    out["chosen_constants"] = {
        "STAND_HEIGHT_M": STAND_HEIGHT_M,
        "LATERAL_STANCE_M": LATERAL_STANCE_M,
        "BODY_SPEED_MPS": BODY_SPEED_MPS, "CYCLE_HZ": CYCLE_HZ,
        "DUTY": DUTY, "SWING_HEIGHT_M": SWING_HEIGHT_M,
        "STABILITY_MARGIN_M": STABILITY_MARGIN_M,
        "SETTLE_S": SETTLE_S, "DURATION_S": DURATION_S}
    out["derived_sway_m"] = sway_amplitude(_S)
    out["commanded_speed_mps"] = commanded_speed()
    out["speed_env_var"] = SPEED_ENV
    out["stride_length_m"] = commanded_speed() * DUTY / CYCLE_HZ
    out["feet_on_the_ground_during_a_cycle"] = 3
    out["phase_offsets"] = dict(PHASE)
    out["closed_loop"] = False
    out["writes"] = "data.ctrl only -- never qpos, qvel or xfrc_applied"
    return out


__all__ = ["BASE_BODY", "BODY_SPEED_MPS", "CYCLE_HZ", "DUTY", "DURATION_S",
           "JOINTS", "LATERAL_STANCE_M", "LEGS", "PHASE", "SETTLE_S", "SIDE",
           "SPEED_ENV", "STABILITY_MARGIN_M", "STAND_HEIGHT_M",
           "SWING_HEIGHT_M", "commanded_speed", "control", "describe",
           "foot_target", "joint_targets", "measure", "setup", "solve_leg",
           "sway_amplitude"]
