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

"""One **scripted** T1 attempt on MuJoCo, and the run directory it leaves.

Scripted, not agent-driven, and the distinction is the whole point: this is the
column's *fixture*, the thing that proves the plumbing carries a run from a raw
URDF to a graded verdict without an agent, a token or a GPU. **A scripted run is
never a ladder cell** (``docs/developer/capability-ladder-plan.md`` §2: a cell
is an autonomous agent, one sentence, no human) and no number this module
produces may be reported as one.

Two entry points, mirroring the Webots arm's launcher/recorder split:

``main()``
    the in-process run. Builds (or re-uses) the scene, settles, drives to the
    commanded point, holds, and writes the artifact set below.

``launch()``
    spawns ``main()`` as a **subprocess** and writes ``process.json``. That is
    not ceremony: ``ProcessFacts`` wants a real exit code and a real captured
    error stream, and a function call inside the grader's own interpreter has
    neither.

The artifact set
----------------

===================  =======================================================
``build.json``       the URDF -> MJCF recipe, every edit and its citation
                     (:mod:`ladder.adapters.mujoco.model_build`)
``roster.json``      the frozen t=0 scan: every body, its world-space AABB in
                     metres, its mass, its joint count, whether it carries a
                     free joint
``trajectory.json``  every body's world position once per physics step, in
                     simulated seconds and metres. Not resampled
``contacts.json``    contact pairs over the **motion** window, named on both
                     sides, timestamped on the trajectory's clock, plus the
                     two vacuity counters
``completion.json``  the driver's own attestation: target simulated time
                     reached, steps taken, control writes made, and the
                     compiled model's shape
``model_saved.xml``  written by MuJoCo itself (``mj_saveLastXML``) from the
                     model that drove this run -- an engine-authored witness
                     that a model compiled, independent of our exit code
``process.json``     exit code, wall clock, the command, versions
``stdout.log`` /     the captured streams. MuJoCo writes no log file, so this
``stderr.log``       is the only channel carrying its warnings
===================  =======================================================

What "the run is real" rests on here
------------------------------------

MuJoCo has no build/finalize phase to witness: ``mj_loadXML`` either returns a
compiled ``mjModel`` or raises with the compiler's error string, and a model
that did not compile cannot be stepped at all. So T1.5's finalize question is
answered from two channels that are *not* the exit code -- the engine-authored
``model_saved.xml``, and the driver's own record that it reached the target
simulated time with ``mjData.time`` to match. Both are stated as such in the
evidence builder rather than asserted here.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder.adapters.mujoco import model_build as mb   # noqa: E402

# --- control-law constants (this scripted driver's, not the tier's) ---------
SETTLE_S = 1.0            # unrecorded: let the suspension find the floor
HOLD_S = 3.0              # recorded: station-keeping after arrival
DRIVE_TIMEOUT_S = 40.0    # simulated seconds before the drive phase gives up
ARRIVE_LATCH_M = 0.12     # once inside this, stop and hold
TURN_FIRST_RAD = 0.50     # above this heading error, start turning in place
TURN_EXIT_RAD = 0.20      # ...and below this one, start driving again
V_MAX = 1.0               # m/s
W_MAX = 1.5               # rad/s
K_V = 1.0
K_W = 2.0
V_MIN = 0.15              # m/s, so the last metre does not creep

# Contact records are emitted every this many physics steps per distinct pair;
# the vacuity counters still count EVERY contact at EVERY step. Emitting one
# record per pair per step would write hundreds of thousands of rows to say the
# same thing, and the stride is published so a reader can see the difference.
CONTACT_EMIT_STRIDE = 25
CONTACT_RECORD_CAP = 6000

# The teleport negative fixture jumps the base once every this many steps.
TELEPORT_PERIOD_STEPS = 250


# --- geometry ---------------------------------------------------------------


def yaw_of(xmat):
    """Yaw of a body from its 3x3 world rotation matrix (row-major, len 9)."""
    return math.atan2(float(xmat[3]), float(xmat[0]))


def drive_command(pos, yaw, goal, *, arrived, turning=True):
    """``(v, w, arrived, turning)`` -- the scripted skid-steer control law.

    Deliberately dull: turn in place until roughly pointed at the goal, then
    drive with a proportional term on both range and heading, then **latch**
    on arrival and hold zero. The latch is what makes T1.1's trailing dwell a
    property of the controller rather than of luck.

    The turn/drive switch is **hysteretic** (enter above
    :data:`TURN_FIRST_RAD`, leave below :data:`TURN_EXIT_RAD`) because a skid
    steer on a rigid plane holds a standing heading error of a few tenths of a
    radian: a single threshold makes the robot chatter between turning in
    place and driving, which is a control bug that reads as a physics one.
    """
    dx, dy = float(goal[0]) - float(pos[0]), float(goal[1]) - float(pos[1])
    dist = math.hypot(dx, dy)
    if arrived or dist <= ARRIVE_LATCH_M:
        return 0.0, 0.0, True, False
    herr = mb.wrap_pi(math.atan2(dy, dx) - yaw)
    turning = abs(herr) > (TURN_EXIT_RAD if turning else TURN_FIRST_RAD)
    w = max(-W_MAX, min(W_MAX, K_W * herr))
    if turning:
        return 0.0, w, False, True
    v = max(V_MIN, min(V_MAX, K_V * dist))
    return v, w, False, False


# --- the scene scan ---------------------------------------------------------


def body_world_aabb(mujoco, model, data, bid):
    """World-space AABB of one body from its geoms, or ``None``.

    Composed from ``mjModel.geom_aabb`` (the geom's own local box) carried
    through ``mjData.geom_xpos``/``geom_xmat``. The eight transformed corners
    are re-min/maxed, so the result is a genuine world-frame box and never a
    local one relabelled. A body with no geoms has no AABB and gets ``None``,
    which is the honest answer for a URDF frame link.
    """
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    n = int(model.body_geomnum[bid])
    if n <= 0:
        return None, None
    adr = int(model.body_geomadr[bid])
    for g in range(adr, adr + n):
        c = model.geom_aabb[g][:3]
        h = model.geom_aabb[g][3:]
        R = data.geom_xmat[g].reshape(3, 3)
        p = data.geom_xpos[g]
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    local = (float(c[0]) + sx * float(h[0]),
                             float(c[1]) + sy * float(h[1]),
                             float(c[2]) + sz * float(h[2]))
                    for k in range(3):
                        v = float(p[k]) + sum(float(R[k][j]) * local[j]
                                              for j in range(3))
                        lo[k] = min(lo[k], v)
                        hi[k] = max(hi[k], v)
    if lo[0] == float("inf"):
        return None, None
    return tuple(lo), tuple(hi)


def robot_subtree(mujoco, model, base_bid):
    """Body ids belonging to the robot rooted at ``base_bid`` (inclusive)."""
    out = {int(base_bid)}
    for i in range(model.nbody):
        j = i
        seen = 0
        while j > 0 and seen < model.nbody:
            if j == base_bid:
                out.add(i)
                break
            j = int(model.body_parentid[j])
            seen += 1
    return out


def free_joint_body(mujoco, model):
    """The id of the body carrying a free joint, or ``None`` if none does."""
    for j in range(model.njnt):
        if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
            return int(model.jnt_bodyid[j])
    return None


def scan_roster(mujoco, model, data, robot_ids):
    """The frozen body inventory. Call between ``mj_forward`` and any step.

    MuJoCo is single-threaded and the caller owns the loop, so "frozen" here is
    not a claim about a synchronization flag -- nothing can advance the clock
    while this function runs. That makes it the one channel of the three
    columns where a t=0 scan is frozen by construction rather than by
    arrangement.
    """
    bodies = []
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
        lo, hi = body_world_aabb(mujoco, model, data, i)
        njnt = int(model.body_jntnum[i])
        subtree_jnts = sum(int(model.body_jntnum[k]) for k in range(model.nbody)
                           if _is_descendant(model, k, i))
        has_free = any(int(model.jnt_type[int(model.body_jntadr[i]) + q])
                       == int(mujoco.mjtJoint.mjJNT_FREE)
                       for q in range(njnt)) if njnt else False
        bodies.append({
            "name": name,
            "id": int(i),
            "parent_id": int(model.body_parentid[i]),
            "type": "mjBODY",
            "position": [float(x) for x in data.xpos[i]],
            "aabb_min": list(lo) if lo else None,
            "aabb_max": list(hi) if hi else None,
            "mass_kg": float(model.body_mass[i]),
            "subtree_mass_kg": float(model.body_subtreemass[i]),
            "n_joints": int(njnt),
            "n_joints_subtree": int(subtree_jnts),
            "n_geoms": int(model.body_geomnum[i]),
            "has_free_joint": bool(has_free),
            "in_robot_subtree": bool(i in robot_ids),
        })
    return {"t_s": float(data.time), "frozen": True, "bodies": bodies}


def _is_descendant(model, k, i):
    """Is body ``k`` at or below body ``i``?"""
    if k == i:
        return True
    j = int(model.body_parentid[k])
    guard = 0
    while j > 0 and guard < model.nbody:
        if j == i:
            return True
        j = int(model.body_parentid[j])
        guard += 1
    return False


# --- the run ----------------------------------------------------------------


def run(scene_path, out_dir, goal_xy=None, *, goal_offset=None,
        settle_s=SETTLE_S, hold_s=HOLD_S,
        drive_timeout_s=DRIVE_TIMEOUT_S, base_link="base_link",
        drive_joints=None, wheel_radius=None, half_track=None,
        record_stride=1, contact_stride=CONTACT_EMIT_STRIDE, defect=None):
    """Load the scene FROM DISK, drive to the commanded point, write artifacts.

    Loading from the saved scene file rather than from an in-memory spec is
    deliberate: the deliverable a ladder cell hands over is a file, and phase B
    re-runs that file cold. A runner that graded an object it still had in
    memory would never notice a deliverable that does not stand alone.

    ``defect`` injects a deliberate fault so an assertion can be observed
    FAILING through the real path (the red-evidence rule; see
    :mod:`ladder.adapters.mujoco.negatives`). ``"teleport"`` writes the base's
    free-joint ``qpos`` straight toward the goal in 1 m jumps with every
    actuator at zero -- it arrives without driving, which is precisely what
    T1.3 exists to catch. It is recorded in ``completion.json`` so no run can
    carry a fault silently.

    ``goal_offset`` expresses the tier's *"five metres north of where it
    starts"* directly: the goal is resolved against the base's horizontal
    position at the **first recorded sample**, i.e. after the settle window, so
    "where it starts" is where the robot rests rather than where it was
    dropped. That is the same rule the task file states, applied here so the
    driver and the grader resolve one number the same way.
    """
    import mujoco

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    try:
        model = mujoco.MjModel.from_xml_path(str(scene_path))
    except (ValueError, RuntimeError) as exc:
        # A scene that does not compile is a MEASUREMENT, not a crash: it is
        # what a broken deliverable looks like, and the compiler's own message
        # is the most useful thing this run will ever produce. mj_loadXML
        # raises rather than returning a degraded model, so there is nothing
        # else to record.
        print("MuJoCo refused the scene %s: %s" % (scene_path, exc),
              file=sys.stderr)
        return 7
    data = mujoco.MjData(model)

    # Engine-authored witness that a model compiled, written before anything
    # else can fail. Independent of our exit code.
    saved_xml = out / "model_saved.xml"
    try:
        mujoco.mj_saveLastXML(str(saved_xml), model)
    except (ValueError, RuntimeError, OSError) as exc:
        print("mj_saveLastXML failed: %r" % (exc,), file=sys.stderr)
        saved_xml = None

    base_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base_link)
    if base_bid < 0:
        base_bid = free_joint_body(mujoco, model)
    if base_bid is None or base_bid < 0:
        print("no base body: neither %r nor a free-jointed body exists"
              % base_link, file=sys.stderr)
        return 2
    robot_ids = robot_subtree(mujoco, model, base_bid)

    drive_joints = list(drive_joints or [])
    act = []
    for jn in drive_joints:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                "vel_%s" % jn)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if aid < 0 or jid < 0:
            continue
        # Which side of the chassis this wheel is on, read off the model
        # rather than parsed out of the joint's name: +y is the robot's LEFT
        # in the ROS/REP-103 frame every URDF uses, and the wheel's own body
        # offset is the only channel that survives a renamed joint.
        left = float(model.body_pos[int(model.jnt_bodyid[jid])][1]) >= 0.0
        act.append((aid, bool(left)))
    if not act:
        print("no drive actuators found for %r" % (drive_joints,),
              file=sys.stderr)
        return 3

    free_qpos_adr = None
    for j in range(model.njnt):
        if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)                 and int(model.jnt_bodyid[j]) == int(base_bid):
            free_qpos_adr = int(model.jnt_qposadr[j])
            break
    if defect == "teleport" and free_qpos_adr is None:
        print("teleport defect asked for, but the base carries no free joint",
              file=sys.stderr)
        return 6

    dt = float(model.opt.timestep)
    r = float(wheel_radius or 0.0)
    b = float(half_track or 0.0)

    # -- settle (not recorded) --------------------------------------------
    for _ in range(int(round(settle_s / dt))):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # -- the frozen t=0 scan ----------------------------------------------
    roster = scan_roster(mujoco, model, data, robot_ids)
    (out / "roster.json").write_text(json.dumps(roster, indent=1),
                                     encoding="utf-8")

    # -- the commanded point, resolved against the settled start ----------
    start_xy = (float(data.xpos[base_bid][0]), float(data.xpos[base_bid][1]))
    if goal_xy is None:
        off = goal_offset or (0.0, 0.0)
        goal_xy = (start_xy[0] + float(off[0]), start_xy[1] + float(off[1]))

    # -- record ------------------------------------------------------------
    t0 = float(data.time)
    times = []
    tracks = [[] for _ in range(model.nbody)]
    pairs = []
    total_observed = 0
    distinct_named = 0
    truncated = False
    arrived = False
    turning = True
    arrived_t = None
    ctrl_writes = 0
    step = 0
    max_steps = int(round((drive_timeout_s + hold_s) / dt))

    # The FIRST sample is taken before the first step. The commanded point is
    # resolved against "the graded body's position at the first recorded
    # sample" (the task file's rule), so a series that begins one step late
    # resolves a goal one step of travel away from the one the driver aimed
    # at -- invisible at 2 ms of ordinary driving and exactly one metre off on
    # the teleport fixture, which is how this was found.
    times.append(0.0)
    for i in range(model.nbody):
        pt = data.xpos[i]
        tracks[i].append([round(float(pt[0]), 6), round(float(pt[1]), 6),
                          round(float(pt[2]), 6)])

    while step < max_steps:
        pos = data.xpos[base_bid]
        v, w, arrived, turning = drive_command(
            pos, yaw_of(data.xmat[base_bid]), goal_xy, arrived=arrived,
            turning=turning)
        if arrived and arrived_t is None:
            arrived_t = float(data.time) - t0
        if defect == "teleport":
            # DELIBERATE FAULT: arrive without driving. Actuators stay at zero
            # and the base pose is written directly, one metre at a time.
            for aid, _ in act:
                data.ctrl[aid] = 0.0
            if step % TELEPORT_PERIOD_STEPS == 0 and not arrived:
                dx = float(goal_xy[0]) - float(data.qpos[free_qpos_adr])
                dy = float(goal_xy[1]) - float(data.qpos[free_qpos_adr + 1])
                n = math.hypot(dx, dy) or 1.0
                k = min(1.0, n) / n
                data.qpos[free_qpos_adr] += dx * k
                data.qpos[free_qpos_adr + 1] += dy * k
                mujoco.mj_forward(model, data)
        else:
            wl, wr = mb.wheel_targets(v, w, r, b)
            for aid, is_left in act:
                data.ctrl[aid] = wl if is_left else wr
        ctrl_writes += 1

        mujoco.mj_step(model, data)
        step += 1

        if step % record_stride == 0:
            times.append(round(float(data.time) - t0, 6))
            for i in range(model.nbody):
                p = data.xpos[i]
                tracks[i].append([round(float(p[0]), 6), round(float(p[1]), 6),
                                  round(float(p[2]), 6)])

        emit = (step % contact_stride == 0)
        seen_here = set()
        for c in range(int(data.ncon)):
            con = data.contact[c]
            b1 = int(model.geom_bodyid[int(con.geom1)])
            b2 = int(model.geom_bodyid[int(con.geom2)])
            n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b1) or ""
            n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b2) or ""
            total_observed += 1
            if b1 != b2 and n1 and n2:
                distinct_named += 1
            if not emit or (b1, b2) in seen_here:
                continue
            seen_here.add((b1, b2))
            if len(pairs) >= CONTACT_RECORD_CAP:
                truncated = True
                continue
            pairs.append({
                "a": n1, "b": n2,
                "a_robot": bool(b1 in robot_ids),
                "b_robot": bool(b2 in robot_ids),
                "point": [round(float(x), 6) for x in con.pos],
                "step": int(step),
                "t_s": round(float(data.time) - t0, 6),
            })

        if arrived and arrived_t is not None \
                and (float(data.time) - t0) - arrived_t >= hold_s:
            break

    recorded_s = float(times[-1]) if times else 0.0
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
             for i in range(model.nbody)]
    (out / "trajectory.json").write_text(json.dumps({
        "dt_s": dt * record_stride,
        "recorded_s": recorded_s,
        "complete": bool(arrived),
        "record_stride_steps": int(record_stride),
        "physics_timestep_s": dt,
        "bodies": [{"name": names[i], "id": i, "t": times, "xyz": tracks[i]}
                   for i in range(model.nbody)],
    }), encoding="utf-8")

    (out / "contacts.json").write_text(json.dumps({
        "supported": True,
        "steps": int(step),
        "window_s": recorded_s,
        "total_observed": int(total_observed),
        "distinct_named": int(distinct_named),
        "emit_stride_steps": int(contact_stride),
        "records_truncated": bool(truncated),
        "pairs": pairs,
        "source": "mjData.contact over every physics step of the recorded "
                  "window; each contact's two geoms mapped to bodies through "
                  "mjModel.geom_bodyid and named with mj_id2name. The two "
                  "counters count EVERY contact at EVERY step; the pair "
                  "records are emitted once per distinct body pair every %d "
                  "steps" % contact_stride,
    }), encoding="utf-8")

    warnings = {}
    for i in range(len(data.warning)):
        n = int(data.warning[i].number)
        if n:
            warnings[_warning_name(mujoco, i)] = n

    (out / "completion.json").write_text(json.dumps({
        "complete": bool(arrived),
        "arrived_at_s": arrived_t,
        "recorded_s": recorded_s,
        "target_s": drive_timeout_s + hold_s,
        "steps": int(step),
        "dt_s": dt,
        "ctrl_writes": int(ctrl_writes),
        "sim_time_at_exit_s": float(data.time),
        "settle_s": settle_s,
        "hold_s": hold_s,
        "deliberate_defect": defect,
        "start_xy_m": [round(start_xy[0], 6), round(start_xy[1], 6)],
        "base_body": base_link,
        "base_body_id": int(base_bid),
        "robot_subtree_bodies": sorted(int(x) for x in robot_ids),
        "model": {"nq": int(model.nq), "nv": int(model.nv),
                  "nu": int(model.nu), "nbody": int(model.nbody),
                  "ngeom": int(model.ngeom), "njnt": int(model.njnt)},
        "engine": engine_facts(mujoco, model),
        "warnings": warnings,
        "saved_xml": str(saved_xml) if saved_xml else None,
        "scene": str(scene_path),
        # Provenance only. The commanded point is TASK data and the evidence
        # builder never reads it from here -- see ladder_evidence.Waypoint.
        "commanded_waypoint_provenance_only": [float(goal_xy[0]),
                                               float(goal_xy[1])],
    }, indent=1), encoding="utf-8")
    print("recorded %.3f s, %d steps, arrived=%s, contacts observed=%d"
          % (recorded_s, step, arrived, total_observed))
    return 0


def _warning_name(mujoco, i):
    try:
        return list(mujoco.mjtWarning.__members__)[i]
    except (AttributeError, IndexError):
        return "mjWARN_%d" % i


SOLVERS = {0: "PGS", 1: "CG", 2: "Newton"}
INTEGRATORS = {0: "Euler", 1: "RK4", 2: "implicit", 3: "implicitfast"}
CONE = {0: "pyramidal", 1: "elliptic"}


def engine_facts(mujoco, model):
    """Which MuJoCo, driven how -- read off the model that ran.

    ⚠ ``solver`` here is **MuJoCo's constraint solver**. Its ``Newton`` option
    is Newton's method on the constraint problem and has nothing to do with
    NVIDIA Newton, the GPU physics engine this repository's own default
    backend uses. Anyone reading a cross-column table will meet both words and
    the citation says which is which.
    """
    return {
        "library": "mujoco",
        "version": mujoco.__version__,
        "version_string": mujoco.mj_versionString(),
        "solver": SOLVERS.get(int(model.opt.solver), str(int(model.opt.solver))),
        "solver_note": "MuJoCo's own constraint solver -- NOT NVIDIA Newton",
        "integrator": INTEGRATORS.get(int(model.opt.integrator),
                                      str(int(model.opt.integrator))),
        "cone": CONE.get(int(model.opt.cone), str(int(model.opt.cone))),
        "timestep_s": float(model.opt.timestep),
        "gravity": [float(x) for x in model.opt.gravity],
        "iterations": int(model.opt.iterations),
        "module": mujoco.__file__,
        "executed_on": "CPU via the mujoco C library (mj_step); MJX/warp were "
                       "not imported by this runner",
    }


# --- CLI + subprocess launcher ----------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--description", help="the shipped robot package dir")
    ap.add_argument("--scene", help="an already-built MJCF scene (phase B)")
    ap.add_argument("--out", required=True)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"),
                   help="an ABSOLUTE commanded point in world metres")
    g.add_argument("--goal-offset", nargs=2, type=float, metavar=("DX", "DY"),
                   help="the commanded point relative to the settled start")
    ap.add_argument("--settle-s", type=float, default=SETTLE_S)
    ap.add_argument("--hold-s", type=float, default=HOLD_S)
    ap.add_argument("--drive-timeout-s", type=float, default=DRIVE_TIMEOUT_S)
    ap.add_argument("--record-stride", type=int, default=1)
    ap.add_argument("--contact-stride", type=int,
                    default=CONTACT_EMIT_STRIDE)
    ap.add_argument("--defect", choices=("teleport",),
                    help="inject a DELIBERATE fault (red-evidence fixtures; "
                         "see negatives.py). Recorded in completion.json")
    a = ap.parse_args(argv)

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    scene, joints, r, b = a.scene, None, None, None

    if scene is None:
        if not a.description:
            print("one of --scene or --description is required",
                  file=sys.stderr)
            return 4
        res = mb.build_scene(a.description, out / "workspace")
        mb.write_build_record(res, out / "build.json")
        for p in res.problems:
            print("build problem: %s" % p, file=sys.stderr)
        if not res.scene:
            print("the scene was not built", file=sys.stderr)
            return 5
        scene, joints = res.scene, res.drive_joints
        r, b = res.wheel_radius_m, res.half_track_m
    else:
        rec = out / "build.json"
        if rec.is_file():
            doc = json.loads(rec.read_text(encoding="utf-8"))
            joints = doc.get("drive_joints")
            r, b = doc.get("wheel_radius_m"), doc.get("half_track_m")

    goal = (a.goal[0], a.goal[1]) if a.goal else None
    off = (a.goal_offset[0], a.goal_offset[1]) if a.goal_offset else None
    return run(scene, out, goal, goal_offset=off, settle_s=a.settle_s,
               hold_s=a.hold_s, drive_timeout_s=a.drive_timeout_s,
               drive_joints=joints, wheel_radius=r, half_track=b,
               record_stride=a.record_stride,
               contact_stride=a.contact_stride, defect=a.defect)


def launch(out_dir, goal_xy=None, *, goal_offset=None, description=None,
           scene=None, timeout_s=900, python=None, extra_args=()):
    """Run :func:`main` as a subprocess; write ``process.json``. Never raises.

    Returns the parsed ``process.json`` dict.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    cmd = [python or sys.executable, str(Path(__file__).resolve()),
           "--out", str(out)]
    if goal_offset is not None:
        cmd += ["--goal-offset", "%g" % goal_offset[0], "%g" % goal_offset[1]]
    else:
        cmd += ["--goal", "%g" % goal_xy[0], "%g" % goal_xy[1]]
    if scene:
        cmd += ["--scene", str(scene)]
    if description:
        cmd += ["--description", str(description)]
    cmd += [str(x) for x in extra_args]

    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(Path(__file__).resolve().parents[3])]
        + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))

    t = time.time()
    timed_out = False
    try:
        p = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=timeout_s, env=env)
        rc, so, se = p.returncode, p.stdout, p.stderr
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        rc = None
        so = exc.stdout.decode("utf-8", "replace") if exc.stdout else ""
        se = exc.stderr.decode("utf-8", "replace") if exc.stderr else ""
    except OSError as exc:
        timed_out = False
        rc, so, se = None, "", "failed to launch: %r" % (exc,)
    wall = time.time() - t

    (out / "stdout.log").write_text(so or "", encoding="utf-8")
    (out / "stderr.log").write_text(se or "", encoding="utf-8")

    doc = {"exit_code": rc, "timed_out": bool(timed_out),
           "wall_s": round(wall, 3), "attempts_used": 1,
           "command": cmd, "python": cmd[0],
           "mujoco_version": _mujoco_version(),
           "platform": sys.platform}
    (out / "process.json").write_text(json.dumps(doc, indent=1),
                                      encoding="utf-8")
    return doc


def _mujoco_version():
    try:
        import mujoco
        return mujoco.__version__
    except ImportError:
        return None


__all__ = ["CONTACT_EMIT_STRIDE", "body_world_aabb", "drive_command",
           "engine_facts", "free_joint_body", "launch", "main", "run",
           "robot_subtree", "scan_roster", "yaw_of"]


if __name__ == "__main__":
    sys.exit(main())
