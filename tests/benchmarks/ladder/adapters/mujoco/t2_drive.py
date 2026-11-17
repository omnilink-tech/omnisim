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

"""**The scripted T2 driver.** Pick the block up, carry it, hold, drop it in.

This is the *controller half of the deliverable*: the phase-B runner copies this
file next to the scene it drives and imports it **by path, with no package on
``sys.path``** -- which is why nothing here imports from ``ladder`` and the only
dependencies are ``math`` and ``mujoco``. A driver that needed the benchmark's
own tree to run would not be a deliverable that stands alone.

**Nothing here is a ladder cell.** A human wrote this control law knowing the
tier's thresholds (``capability-ladder-plan.md`` §2). Its one claim is that the
transfer the container describes is **physically achievable** on this column --
which the container's own ``PROVENANCE.txt`` records as never having been
demonstrated on any simulator, and which the plan requires settling before the
freeze, because a tier no scripted run can complete measured the asset rather
than the agents.

Nothing is teleported and nothing is welded
-------------------------------------------

Every joint is driven through a **force-limited position servo** whose force
range is the effort the shipped URDF declares. The object is never written to:
it moves only because two finger pads press on it and friction does the rest.
``mjModel.neq`` is 0 in the scene this drives, and the runner records that fact
next to the verdict, which is what makes ``hold_mechanism: friction`` an
observation rather than a claim.

The arm is measured, never typed
--------------------------------

Every length this control law needs -- the three link lengths, the shoulder
height, where the finger pads are in the tool's frame, how wide the object is,
where the table top and the bin floor are -- is read off the **compiled model**
in :func:`setup`. The T1 driver's rule, for the same reason: *"a controller
built on typed constants is a controller that silently drives the wrong
robot."* Change the description and this driver follows it.

The control law
---------------

The arm has six axes and the task needs five numbers (a point, plus "the
gripper points straight down"), so the redundancy is spent on a **closed-form**
solution instead of an iterative one: ``joint_1`` is the bearing to the target,
``joint_4`` and ``joint_6`` are held at zero, and ``joint_2``/``joint_3``/
``joint_5`` are a planar 3R chain solved exactly, with the wrist constrained by
``joint_2 + joint_3 + joint_5 = π`` so the tool axis is vertical.

On top of that sits a **per-joint integral term**. A position servo carries a
steady-state error of (gravity torque / kp) -- about 0.02 rad at the shoulder
here, which is a centimetre or two at the fingertips and quite enough to miss a
50 mm cube. The integrator removes it. It is clamped, and it is honest
closed-loop control on measured joint state, not a way of writing a pose onto
the arm.

The phases, and why each is that long
-------------------------------------

======================  =====  =====================================
phase                   s      why
======================  =====  =====================================
approach                2.0    to a point ``APPROACH_M`` above the grasp
descend                 1.5    slowly, jaws open
close                   1.2    the servo squeezes ``SQUEEZE_M`` past contact
lift                    1.8    to the carry height
steady                  3.0    let the arm settle before the yaw
swing                   5.0    90 degrees of ``joint_1`` -- **the object is
                               rotating in the world while rigid in the
                               tool's frame, which is the whole point of
                               T2.3 being stated in that frame**
hover                   4.0    over the bin
lower                   2.5    into the bin
release                 1.0    jaws open
retract                 2.5    lift clear
stand                   4.0    so the trailing at-rest window is real
======================  =====  =====================================

The grip is continuous from ``close`` to ``release`` -- **13.8 s**, against the
tier's 10.0 s -- and the 90-degree yaw sits inside it.

⚠ **One thing the run length is deliberately doing.** T2.3 asks whether *some*
10 s window exists in which the object barely moves in the end effector's frame,
and a run that parked a released object under a parked arm for 10 s would
satisfy it without holding anything. The final ``stand`` phase is therefore
**4.0 s**, so the only 10 s window in this run is the carry. That is a property
of this driver, not of the grader, and it is recorded in ``BRINGUP_T2.md`` as an
open question about the tier's wording rather than fixed by moving a threshold.
"""

from __future__ import annotations

import math

# --- what the control law chooses (the rest is measured) ---------------------

APPROACH_M = 0.13         # how far above the grasp the approach sits
PAD_CLEARANCE_M = 0.013   # fingertips clear the surface the object rests on
SQUEEZE_M = 0.010         # servo travel commanded PAST first contact
CARRY_CLEARANCE_M = 0.125  # object's lowest point above the table while carried
BIN_DROP_M = 0.015        # how far above the bin floor the jaws let go
KI = 3.0                  # integral gain, rad/s per rad of error
INTEGRAL_CLAMP = 0.35     # rad

ARM_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
FINGER_JOINTS = ("finger_joint_left", "finger_joint_right")
BASE_BODY = "base"
TOOL_BODY = "tool"
OBJECT_BODY = "block"
CONTAINER_BODY = "bin"
TABLE_BODY = "table"

# (name, seconds). Read with PHASES below.
PHASE_S = (("approach", 2.0), ("descend", 1.5), ("close", 1.2), ("lift", 1.8),
           ("steady", 3.0), ("swing", 5.0), ("hover", 4.0), ("lower", 2.5),
           ("release", 1.0), ("retract", 2.5), ("stand", 4.0))
DURATION_S = sum(s for _n, s in PHASE_S)

_S = {}


# --- geometry ----------------------------------------------------------------


def _jid(mujoco, model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def _bid(mujoco, model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def _body_geom_span_z(model, bid):
    """``(lowest, highest)`` z of a body's own geoms, in the body's frame."""
    lo, hi = float("inf"), float("-inf")
    adr, n = int(model.body_geomadr[bid]), int(model.body_geomnum[bid])
    for g in range(adr, adr + n):
        c = float(model.geom_pos[g][2])
        h = float(model.geom_size[g][2])
        lo = min(lo, c - h)
        hi = max(hi, c + h)
    if lo == float("inf"):
        return None, None
    return lo, hi


def _world_top_z(mujoco, model, data, bid):
    """Highest world z reached by a body's own geoms (axis-aligned boxes)."""
    top = float("-inf")
    adr, n = int(model.body_geomadr[bid]), int(model.body_geomnum[bid])
    for g in range(adr, adr + n):
        z = float(data.geom_xpos[g][2])
        R = data.geom_xmat[g].reshape(3, 3)
        h = model.geom_size[g]
        top = max(top, z + sum(abs(float(R[2][k])) * float(h[k])
                               for k in range(3)))
    return None if top == float("-inf") else top


def measure(mujoco, model, data):
    """Everything the control law needs, read off the compiled model.

    Called once, with the arm's pitches temporarily zeroed so the chain is
    straight and the link lengths fall out of the joint anchors. ``data`` is
    restored before returning.
    """
    jid = {n: _jid(mujoco, model, n) for n in ARM_JOINTS + FINGER_JOINTS}
    bid = {n: _bid(mujoco, model, n)
           for n in (BASE_BODY, TOOL_BODY, OBJECT_BODY, CONTAINER_BODY,
                     TABLE_BODY)}
    missing = ([n for n, i in jid.items() if i < 0]
               + [n for n, i in bid.items() if i < 0])
    if missing:
        raise ValueError("the scene is missing %s" % ", ".join(missing))
    qadr = {n: int(model.jnt_qposadr[i]) for n, i in jid.items()}

    saved = data.qpos.copy()
    for n in ARM_JOINTS:
        data.qpos[qadr[n]] = 0.0
    mujoco.mj_forward(model, data)

    def anchor(n):
        return data.xanchor[jid[n]]

    a1 = float(abs(anchor("joint_3")[2] - anchor("joint_2")[2]))
    a2 = float(abs(anchor("joint_5")[2] - anchor("joint_3")[2]))
    a3 = float(abs(data.xpos[bid[TOOL_BODY]][2] - anchor("joint_5")[2]))
    shoulder_z = float(anchor("joint_2")[2] - data.xpos[bid[BASE_BODY]][2])

    # The finger pads, in the tool's own frame: the tool axis is world +z with
    # the pitches zeroed, so a world-z difference IS a tool-z offset.
    tool_z = float(data.xpos[bid[TOOL_BODY]][2])
    pads = []
    for n in FINGER_JOINTS:
        fb = int(model.jnt_bodyid[jid[n]])
        lo, hi = _body_geom_span_z(model, fb)
        pads.append((float(data.xpos[fb][2]) + lo - tool_z,
                     float(data.xpos[fb][2]) + hi - tool_z))
    pad_near = max(p[0] for p in pads)
    pad_far = min(p[1] for p in pads)
    jaw_offset = max(abs(float(model.body_pos[int(model.jnt_bodyid[jid[n]])][1]))
                     for n in FINGER_JOINTS)
    jaw_thick = max(float(model.geom_size[int(model.body_geomadr[
        int(model.jnt_bodyid[jid[n]])])][1]) for n in FINGER_JOINTS)

    obj_g = int(model.body_geomadr[bid[OBJECT_BODY]])
    obj_half = [float(v) for v in model.geom_size[obj_g][:3]]

    data.qpos[:] = saved
    mujoco.mj_forward(model, data)

    base_xy = (float(data.xpos[bid[BASE_BODY]][0]),
               float(data.xpos[bid[BASE_BODY]][1]))
    obj = data.xpos[bid[OBJECT_BODY]]
    box = data.xpos[bid[CONTAINER_BODY]]
    table_top = _world_top_z(mujoco, model, data, bid[TABLE_BODY])
    bin_floor_top = _world_top_z(mujoco, model, data, bid[CONTAINER_BODY])

    # Where the object's centre sits along the tool's own axis when gripped:
    # far enough in that the fingertips clear whatever it is standing on.
    tool_to_object = pad_far - obj_half[2] + PAD_CLEARANCE_M
    close_cmd = jaw_offset - jaw_thick - obj_half[1] + SQUEEZE_M
    rng = model.jnt_range[jid[FINGER_JOINTS[0]]]
    close_cmd = max(float(rng[0]), min(float(rng[1]), close_cmd))

    return {
        "jid": jid, "bid": bid, "qadr": qadr,
        "aid": {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                     "pos_%s" % n)
                for n in ARM_JOINTS + FINGER_JOINTS},
        "a1": a1, "a2": a2, "a3": a3, "shoulder_z": shoulder_z,
        "pad_near": pad_near, "pad_far": pad_far,
        "jaw_offset": jaw_offset, "jaw_thick": jaw_thick,
        "object_half": obj_half, "tool_to_object": tool_to_object,
        "grip_open": float(rng[0]), "grip_close": close_cmd,
        "base_xy": base_xy, "table_top_z": table_top,
        "bin_floor_top_z": bin_floor_top,
        "object_start": [float(v) for v in obj],
        "object_radius": float(math.hypot(obj[0] - base_xy[0],
                                          obj[1] - base_xy[1])),
        "object_bearing": float(math.atan2(obj[1] - base_xy[1],
                                           obj[0] - base_xy[0])),
        "bin_radius": float(math.hypot(box[0] - base_xy[0],
                                       box[1] - base_xy[1])),
        "bin_bearing": float(math.atan2(box[1] - base_xy[1],
                                        box[0] - base_xy[0])),
        "limits": {n: (float(model.jnt_range[jid[n]][0]),
                       float(model.jnt_range[jid[n]][1]))
                   for n in ARM_JOINTS},
        "integral": {n: 0.0 for n in ARM_JOINTS},
        "last_q": None,
    }


# --- inverse kinematics ------------------------------------------------------


def solve_ik(st, radius, height, bearing):
    """Six joint angles putting the TOOL ORIGIN there, axis pointing down.

    ``radius``/``height`` are relative to the arm's own base frame. Returns
    ``None`` when the pose is outside the arm's reach **or** outside its wrist
    range -- and on this arm the second happens first (see ``t2_scene``).
    """
    a1, a2, a3 = st["a1"], st["a2"], st["a3"]
    u = float(radius)
    w = (float(height) + a3) - st["shoulder_z"]
    c = (u * u + w * w - a1 * a1 - a2 * a2) / (2.0 * a1 * a2)
    if abs(c) > 1.0:
        return None
    for sign in (1.0, -1.0):
        beta = sign * math.acos(max(-1.0, min(1.0, c)))
        phi = math.atan2(u, w) - math.atan2(a2 * math.sin(beta),
                                            a1 + a2 * math.cos(beta))
        q = {"joint_1": float(bearing), "joint_2": phi, "joint_3": beta,
             "joint_4": 0.0, "joint_5": math.pi - phi - beta, "joint_6": 0.0}
        lim = st["limits"]
        if all(lim[n][0] <= v <= lim[n][1] for n, v in q.items()):
            return q
    return None


# --- the schedule ------------------------------------------------------------


def waypoints(st):
    """``[(radius, height, bearing, grip)]``, one per phase boundary."""
    obj = st["object_start"]
    grasp_w = float(obj[2]) + st["tool_to_object"]
    pre_w = grasp_w + APPROACH_M
    carry_w = (st["table_top_z"] + CARRY_CLEARANCE_M + st["object_half"][2]
               + st["tool_to_object"])
    bin_w = (st["bin_floor_top_z"] + BIN_DROP_M + st["object_half"][2]
             + st["tool_to_object"])
    ro, rb = st["object_radius"], st["bin_radius"]
    bo, bb = st["object_bearing"], st["bin_bearing"]
    op, cl = st["grip_open"], st["grip_close"]
    return {
        "start": (ro, pre_w, bo, op),
        "approach": (ro, pre_w, bo, op),
        "descend": (ro, grasp_w, bo, op),
        "close": (ro, grasp_w, bo, cl),
        "lift": (ro, carry_w, bo, cl),
        "steady": (ro, carry_w, bo, cl),
        "swing": (rb, carry_w, bb, cl),
        "hover": (rb, carry_w, bb, cl),
        "lower": (rb, bin_w, bb, cl),
        "release": (rb, bin_w, bb, op),
        "retract": (rb, pre_w, bb, op),
        "stand": (rb, pre_w, bb, op),
    }


def _smoothstep(f):
    f = max(0.0, min(1.0, f))
    return f * f * (3.0 - 2.0 * f)


def target_at(st, t_s):
    """``(radius, height, bearing, grip, phase)`` at simulated time ``t_s``."""
    wp = st["waypoints"]
    prev = wp["start"]
    t0 = 0.0
    for name, dur in PHASE_S:
        nxt = wp[name]
        if t_s <= t0 + dur:
            s = _smoothstep((t_s - t0) / dur)
            return (tuple(p + s * (n - p) for p, n in zip(prev, nxt))
                    + (name,))
        t0 += dur
        prev = nxt
    return prev + ("done",)


# --- the driver interface the phase-B runner calls ---------------------------


def setup(model, data):
    """Called once, after the model loads and before any step."""
    import mujoco
    st = measure(mujoco, model, data)
    st["waypoints"] = waypoints(st)
    _S.clear()
    _S.update(st)

    # Start in the approach pose rather than folded flat, so the settle window
    # is the arm standing still and not the arm falling over.
    q = solve_ik(st, *st["waypoints"]["start"][:3])
    if q is None:
        raise ValueError("the approach pose is outside this arm's wrist range")
    for n, v in q.items():
        data.qpos[st["qadr"][n]] = v
        data.ctrl[st["aid"][n]] = v
    for n in FINGER_JOINTS:
        data.ctrl[st["aid"][n]] = st["grip_open"]
    mujoco.mj_forward(model, data)
    return st


def control(model, data, t_s):
    """Called every physics step with the simulated time since recording began.

    Writes ``data.ctrl`` and nothing else -- never ``qpos``, never ``qvel``,
    never ``xfrc_applied``.
    """
    st = _S
    if not st:
        setup(model, data)
    radius, height, bearing, grip, _phase = target_at(st, t_s)

    # The pedestal is a free body resting on the ground, so the arm's own frame
    # is measured every step rather than assumed: a base that shifts a
    # millimetre under load would otherwise put the whole plan a millimetre out.
    bx = float(data.xpos[st["bid"][BASE_BODY]][0])
    by = float(data.xpos[st["bid"][BASE_BODY]][1])
    bz = float(data.xpos[st["bid"][BASE_BODY]][2])
    base_yaw = math.atan2(float(data.xmat[st["bid"][BASE_BODY]][3]),
                          float(data.xmat[st["bid"][BASE_BODY]][0]))
    # The waypoints are stated in the WORLD (they were measured off the scene
    # at t=0), so the pose the arm has to solve for is the target expressed in
    # the base's CURRENT frame -- position and yaw both. A pedestal that has
    # shifted a millimetre or turned a milliradian under load would otherwise
    # put the whole plan out by that much, which is the difference between a
    # grasp and a nudge.
    tx = st["base_xy"][0] + radius * math.cos(bearing) - bx
    ty = st["base_xy"][1] + radius * math.sin(bearing) - by
    r_eff = math.hypot(tx, ty)
    b_eff = math.atan2(ty, tx) - base_yaw

    q = solve_ik(st, r_eff, height - bz, b_eff)
    if q is None:
        q = st["last_q"]
        if q is None:
            return
    st["last_q"] = q

    dt = float(model.opt.timestep)
    for n, v in q.items():
        err = v - float(data.qpos[st["qadr"][n]])
        acc = st["integral"][n] + KI * err * dt
        st["integral"][n] = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, acc))
        data.ctrl[st["aid"][n]] = v + st["integral"][n]
    for n in FINGER_JOINTS:
        data.ctrl[st["aid"][n]] = grip


def describe():
    """What this driver measured and what it decided. Recorded in the run."""
    if not _S:
        return {}
    keys = ("a1", "a2", "a3", "shoulder_z", "pad_near", "pad_far",
            "jaw_offset", "jaw_thick", "object_half", "tool_to_object",
            "grip_open", "grip_close", "table_top_z", "bin_floor_top_z",
            "object_start", "object_radius", "object_bearing", "bin_radius",
            "bin_bearing")
    out = {k: _S.get(k) for k in keys}
    out["phases_s"] = {n: s for n, s in PHASE_S}
    out["duration_s"] = DURATION_S
    out["waypoints"] = {k: list(v) for k, v in _S["waypoints"].items()}
    out["chosen_constants"] = {
        "APPROACH_M": APPROACH_M, "PAD_CLEARANCE_M": PAD_CLEARANCE_M,
        "SQUEEZE_M": SQUEEZE_M, "CARRY_CLEARANCE_M": CARRY_CLEARANCE_M,
        "BIN_DROP_M": BIN_DROP_M, "KI": KI}
    return out


__all__ = ["ARM_JOINTS", "CONTAINER_BODY", "DURATION_S", "FINGER_JOINTS",
           "OBJECT_BODY", "PHASE_S", "TOOL_BODY", "control", "describe",
           "measure", "setup", "solve_ik", "target_at", "waypoints"]
