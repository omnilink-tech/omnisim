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

"""**Phase B for T2 on MuJoCo**: re-run a deliverable cold, with OUR sampler.

SPEC 2.3 and ``capability-ladder-plan.md`` §2: a ladder cell is scored from a
**standalone cold re-run** of whatever the attempt produced, with the grader's
own sampler injected and no agent present. On OmniSim that means launching the
engine with ``ladder_recorder`` appended to the world; MuJoCo has no world file,
no controller process and no engine to launch, so the equivalent here is:

    load the deliverable's scene, import the deliverable's driver **by path**,
    and step the loop ourselves -- writing down everything on the way.

What a T2 deliverable is on this column
---------------------------------------

Two files, because MuJoCo cannot express a controller in a scene::

    <deliverable>/scene.xml   an MJCF the compiler accepts
    <deliverable>/drive.py    a module exposing control(model, data, t_s),
                              optionally setup(model, data) and DURATION_S

Either the directory or the ``scene.xml`` inside it may be handed to
:func:`run_standalone`. The driver is imported with
``importlib.util.spec_from_file_location`` and **is not put on ``sys.path``** --
a driver that needed the benchmark's own package tree would not be a
deliverable that stands alone, and this is where that gets enforced rather than
assumed. A deliverable with no driver still runs (the scene is stepped with
zero control input) and produces an honest, failing row.

The eight channels this sampler exists to answer
------------------------------------------------

``ladder.graders.ladder_evidence`` gap 3 lists what T2 needs and the frozen
AgentBench bundle has no field for. Every one is reachable through MuJoCo's
public API, and here is where each is read:

=======================  ====================================================
channel                  where it comes from
=======================  ====================================================
``object_pose``          ``mjData.xpos`` + ``mjData.xmat`` for the body the
                         task names, once per ``record_stride`` steps. The
                         object is a plain body, so nothing that enumerates
                         *robots* would ever have recorded it.
``end_effector``         the same, for the carrier -- **with its rotation**.
                         ``mjData.xmat`` is a world-from-body 3x3 already, so
                         the convention conversion every other column needs
                         does not arise. Both series are written by ONE loop
                         on ONE clock, so the core's clock-skew guard reads 0.
``container_geometry``   the union of the named body's SUBTREE geom AABBs (the
                         bin's four walls are child bodies, so the body's own
                         box would be its floor slab and nothing else), plus a
                         **measured** rim -- see :func:`container_geometry`.
``support_surfaces``     every body MuJoCo welds to the world
                         (``body_weldid == 0``) that is not part of the arm
                         and carries geometry. Structural, not a name list.
``object_mass``          ``mjModel.body_mass`` -- kilograms, which the neutral
                         inventory has no field for at all.
``gravity``              ``mjModel.opt.gravity``, the model that actually ran.
``object_aabb``          the same world-AABB walk at t=0, so a centre can be
                         turned into a lowest point.
``grip``                 contacts naming an arm body and the object, timed on
                         the pose series' clock -- **plus** ``mjModel.neq``
                         and the equality table, which is what lets the
                         mechanism be *observed* as friction rather than
                         asserted. Recorded in every cell and graded in none.
=======================  ====================================================

Frozen by construction, still
-----------------------------

The t=0 scan happens between the settle window's last ``mj_step`` and the first
recorded one, inside a loop this process owns. There is no simulator to race
and no synchronisation flag to trust -- the same property the T1 bring-up
records as this column's genuine advantage.
"""

from __future__ import annotations

import ast

import argparse
import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder.adapters.mujoco import runner as t1_runner  # noqa: E402

TASK = "T2_transfer"
DRIVER_NAME = "drive.py"
SCENE_NAME = "scene.xml"

# One sample every this many physics steps. The tier's own witness bound is
# MAX_SAMPLE_DT_S = 0.05 s; at the 2 ms timestep this scene uses, 5 steps is
# 10 ms -- five times finer than the bound, so T2.4's "moved like a body"
# clause has a real witness rather than a formality.
RECORD_STRIDE = 5
CONTACT_RECORD_CAP = 20000

DEFAULT_ROLES = {"object": "block", "end_effector": "tool", "container": "bin"}

RIM_RULE = (
    "measured, not assumed: the container's rim is the highest point of any "
    "geom in its subtree whose HORIZONTAL FOOTPRINT DOES NOT COVER the "
    "subtree's own horizontal centre. An open-topped box's walls stand clear "
    "of the centre line and its floor does not, so the walls decide the rim "
    "and the floor cannot; a lid, a raised handle or a closed top sits OVER "
    "the centre and is excluded, which is the whole reason the rim is not just "
    "the top of the bounding box. LIMITS, stated: a container whose wall "
    "geometry is one revolved shell covering its own centre has no rim under "
    "this rule and reports None (the core then falls back to the permissive "
    "box top and says so), and the footprints are axis-aligned world boxes, so "
    "a container tilted off the world axes is over-estimated")

GRIP_SOURCE = (
    "mjData.contact scanned at every physics step of the recorded window, kept "
    "to pairs naming a body in the arm's kinematic subtree and the body the "
    "task calls the object; each carries the simulated time it was seen, on "
    "the same clock as the pose series. The MECHANISM is read from the model "
    "rather than declared: mjModel.neq is the number of equality constraints, "
    "and the run reports 'attachment' only if one of them binds the object to "
    "another body. With neq == 0 and contacts present, the only thing holding "
    "the object is friction at those contacts")

POSE_SOURCE = (
    "the grader-owned phase-B sampler: mjData.xpos and mjData.xmat for the "
    "named body, one sample every %d physics steps (%.3f s), in simulated "
    "seconds and metres, world frame. mjData.xmat is already a world-from-body "
    "3x3, so no quaternion or Euler convention is involved")

STATIC_SOURCE = (
    "the frozen t=0 scan: every body MuJoCo's compiler welds to the world "
    "(mjModel.body_weldid == 0) that is not in the arm's kinematic subtree and "
    "carries at least one geom. Structural, so a surface an agent named "
    "something unexpected is still found, and a body that can move is still "
    "excluded")


# --- the deliverable ---------------------------------------------------------


def resolve_deliverable(path):
    """``(scene, driver)`` from a directory or a scene file. Never raises."""
    p = Path(path)
    if p.is_dir():
        scene = p / SCENE_NAME
        if not scene.is_file():
            cands = sorted(p.glob("*.xml"))
            scene = cands[0] if cands else None
    else:
        scene = p if p.is_file() else None
    driver = None
    if scene is not None:
        driver = find_driver(scene.parent)
    return scene, driver


#: Names a driver's control entry point may have. Checked by READING the file,
#: never by importing it: importing to find out whether to import is how a
#: deliverable with a syntax error takes the grader down with it.
DRIVER_ENTRY_POINTS = ("control", "step", "policy", "act", "update")


def find_driver(folder):
    """The deliverable's controller, whatever the agent decided to call it.

    ``drive.py`` first, because that is the documented name -- then ANY python
    file in the folder that defines a control entry point.

    ⚠ WHY THIS IS NOT JUST A CONVENIENCE. Requiring the exact filename
    ``drive.py`` made this column measure a naming convention the agent is
    never told. Measured 2026-08-03: an agent built a working MuJoCo scene,
    named its controller ``run.py`` -- an entirely reasonable choice -- and
    phase B stepped the scene for 60 s with ``ctrl_writes: 0``. The arm never
    moved, three assertions went red, and none of it was about MuJoCo or about
    the agent. The prompt says nothing about ``drive.py``; the container says
    nothing about it; nothing the agent can read says it.

    It is also an asymmetry between columns. The OmniSim deliverable is "a
    .wbt", which is simply what that simulator's scene IS; the MuJoCo one was
    "scene.xml plus a file named drive.py", half of which is invented. A
    benchmark whose columns are held to conventions of different naturalness
    is not comparing the simulators.
    """
    folder = Path(folder)
    named = folder / DRIVER_NAME
    if named.is_file():
        return named
    for cand in sorted(folder.glob("*.py")):
        if cand.name.startswith("_"):
            continue
        if _defines_entry_point(cand):
            return cand
    return None


def _defines_entry_point(path):
    """Does this file define a control callback, without running it?

    Parsed with ``ast``, not grepped and not imported. Grepping for ``def
    control(`` matches a file that cannot compile, and importing to find out
    whether to import is how a deliverable with a syntax error takes the
    grader down with it. A file that will not parse cannot be anyone's driver,
    so it is passed over and the search continues.
    """
    try:
        tree = ast.parse(Path(path).read_text(encoding="utf-8",
                                              errors="replace"))
    except (OSError, SyntaxError, ValueError):
        return False
    for node in tree.body:
        if (isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name in DRIVER_ENTRY_POINTS):
            return True
    return False


def load_driver(path):
    """Import a deliverable's driver by path. ``None`` if there is not one.

    Deliberately NOT via ``sys.path``: the deliverable must stand alone.
    """
    if path is None:
        return None
    spec = importlib.util.spec_from_file_location("t2_deliverable_driver",
                                                  str(path))
    if spec is None or spec.loader is None:
        return None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --- geometry helpers --------------------------------------------------------


def subtree_bodies(model, root_bid):
    """Body ids at or below ``root_bid``."""
    out = {int(root_bid)}
    for i in range(model.nbody):
        j = i
        for _ in range(model.nbody):
            if j in out:
                out.add(i)
                break
            if j <= 0:
                break
            j = int(model.body_parentid[j])
    return out


def subtree_aabb(mujoco, model, data, bids):
    """World AABB over the geoms of a set of bodies, or ``(None, None)``."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for b in sorted(bids):
        a, c = t1_runner.body_world_aabb(mujoco, model, data, int(b))
        if a is None:
            continue
        for k in range(3):
            lo[k] = min(lo[k], a[k])
            hi[k] = max(hi[k], c[k])
    if lo[0] == float("inf"):
        return None, None
    return tuple(lo), tuple(hi)


def geom_world_aabb(model, data, g):
    """One geom's world-space AABB, from its local box carried through."""
    c = model.geom_aabb[g][:3]
    h = model.geom_aabb[g][3:]
    R = data.geom_xmat[g].reshape(3, 3)
    p = data.geom_xpos[g]
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
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
    return tuple(lo), tuple(hi)


def container_geometry(mujoco, model, data, name):
    """The container's world box and its **measured** rim. See :data:`RIM_RULE`."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        return {"body": name, "error": "no body named %r is in the scene"
                                       % name}
    bids = subtree_bodies(model, bid)
    lo, hi = subtree_aabb(mujoco, model, data, bids)
    if lo is None:
        return {"body": name, "error": "the container carries no geometry"}
    cx = 0.5 * (lo[0] + hi[0])
    cy = 0.5 * (lo[1] + hi[1])
    rim = None
    walls = []
    for b in sorted(bids):
        adr, n = int(model.body_geomadr[b]), int(model.body_geomnum[b])
        for g in range(adr, adr + n):
            glo, ghi = geom_world_aabb(model, data, g)
            covers = (glo[0] <= cx <= ghi[0]) and (glo[1] <= cy <= ghi[1])
            if covers:
                continue
            walls.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
                         or "#%d" % g)
            rim = ghi[2] if rim is None else max(rim, ghi[2])
    return {"body": name, "aabb_min": list(lo), "aabb_max": list(hi),
            "rim_z": (None if rim is None else float(rim)),
            "rim_rule": RIM_RULE,
            "rim_from_geoms": walls,
            "subtree_bodies": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                                 int(b)) or "#%d" % b
                               for b in sorted(bids)],
            "t_s": float(data.time)}


def support_surfaces(mujoco, model, data, exclude):
    """Static, bounded, non-arm bodies. See :data:`STATIC_SOURCE`."""
    out = []
    for i in range(1, model.nbody):
        if i in exclude or int(model.body_weldid[i]) != 0:
            continue
        if int(model.body_geomnum[i]) <= 0:
            continue
        lo, hi = t1_runner.body_world_aabb(mujoco, model, data, i)
        if lo is None:
            continue
        out.append({"body": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                              i) or "#%d" % i,
                    "aabb_min": list(lo), "aabb_max": list(hi),
                    "static": True})
    return out


def object_physics(mujoco, model, name):
    """Kilograms, and whether the body can move at all."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        return {"body": name, "error": "no body named %r is in the scene"
                                       % name}
    mass = float(model.body_mass[bid])
    # "dynamic" in the neutral schema is 'a mass model is attached AND it
    # moves'. A MuJoCo body moves exactly when it is not in the world's weld
    # group; mass is the separate question T2.4 asks in kilograms.
    return {"body": name, "mass_kg": mass,
            "subtree_mass_kg": float(model.body_subtreemass[bid]),
            "dynamic": bool(int(model.body_weldid[bid]) != 0 and mass > 0.0),
            "weldid": int(model.body_weldid[bid]),
            "n_joints": int(model.body_jntnum[bid])}


def equality_binds(mujoco, model, bid):
    """Does any equality constraint bind this body to another? (mechanism)"""
    for e in range(model.neq):
        t = int(model.eq_type[e])
        if t not in (int(mujoco.mjtEq.mjEQ_CONNECT), int(mujoco.mjtEq.mjEQ_WELD)):
            continue
        if int(model.eq_obj1id[e]) == int(bid) or \
                int(model.eq_obj2id[e]) == int(bid):
            return True
    return False


# --- the run -----------------------------------------------------------------


def run(scene_path, out_dir, *, driver_path=None, duration=60.0, settle=0.5,
        contact_stride=10, record_stride=RECORD_STRIDE, roles=None,
        surfaces=()):
    """Load the scene, drive it with the deliverable's own driver, record.

    Returns a process exit code. Never raises for a physical failure -- a
    scene that will not compile, a driver that will not import and a grasp
    that does not hold are all measurements.
    """
    import mujoco

    roles = dict(DEFAULT_ROLES, **(roles or {}))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    try:
        model = mujoco.MjModel.from_xml_path(str(scene_path))
    except (ValueError, RuntimeError) as exc:
        print("MuJoCo refused the scene %s: %s" % (scene_path, exc),
              file=sys.stderr)
        return 7
    data = mujoco.MjData(model)

    saved_xml = out / "model_saved.xml"
    try:
        mujoco.mj_saveLastXML(str(saved_xml), model)
    except (ValueError, RuntimeError, OSError) as exc:
        print("mj_saveLastXML failed: %r" % (exc,), file=sys.stderr)
        saved_xml = None

    driver = None
    driver_error = None
    try:
        driver = load_driver(driver_path)
    except Exception as exc:  # noqa: BLE001  (a broken driver is a result)
        driver_error = "the deliverable's driver would not import: %r" % (exc,)
        print(driver_error, file=sys.stderr)
    if driver is None and driver_error is None:
        driver_error = ("the deliverable carries no %s, so the scene was "
                        "stepped with no control input at all" % DRIVER_NAME)
        print(driver_error, file=sys.stderr)

    mujoco.mj_forward(model, data)
    if driver is not None and hasattr(driver, "setup"):
        try:
            driver.setup(model, data)
        except Exception as exc:  # noqa: BLE001
            driver_error = "the driver's setup() raised: %r" % (exc,)
            print(driver_error, file=sys.stderr)

    dt = float(model.opt.timestep)
    control = getattr(driver, "control", None) if driver is not None else None
    ctrl_writes = 0
    control_error = None

    def drive(t_rel):
        nonlocal ctrl_writes, control_error
        if control is None:
            return
        try:
            control(model, data, t_rel)
            ctrl_writes += 1
        except Exception as exc:  # noqa: BLE001
            if control_error is None:
                control_error = "the driver's control() raised: %r" % (exc,)
                print(control_error, file=sys.stderr)

    # -- settle (not recorded): the pedestal finds the floor ---------------
    for _ in range(int(round(float(settle) / dt))):
        drive(0.0)
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # -- the frozen t=0 scan ----------------------------------------------
    arm_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                roles["end_effector"])
    arm_root = arm_bid
    while arm_root > 0 and int(model.body_parentid[arm_root]) != 0:
        arm_root = int(model.body_parentid[arm_root])
    arm_ids = subtree_bodies(model, arm_root) if arm_root > 0 else set()

    roster = t1_runner.scan_roster(mujoco, model, data, arm_ids)
    (out / "roster.json").write_text(json.dumps(roster, indent=1),
                                     encoding="utf-8")

    obj_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                roles["object"])
    ee_bid = arm_bid
    obj_lo, obj_hi = ((None, None) if obj_bid < 0
                      else t1_runner.body_world_aabb(mujoco, model, data,
                                                     obj_bid))
    statics = support_surfaces(mujoco, model, data,
                               exclude=arm_ids | ({obj_bid} if obj_bid >= 0
                                                  else set()))
    t2doc = {
        "roles": dict(roles),
        "record_stride_steps": int(record_stride),
        "physics_timestep_s": dt,
        "container": container_geometry(mujoco, model, data,
                                        roles["container"]),
        "support_surfaces": statics,
        "support_surfaces_source": STATIC_SOURCE,
        "support_surfaces_requested": list(surfaces),
        "object_physics": object_physics(mujoco, model, roles["object"]),
        "world": {"gravity_vec_mps2": [float(x) for x in model.opt.gravity],
                  "gravity_mps2": float(sum(float(x) ** 2
                                            for x in model.opt.gravity) ** 0.5),
                  "source": ("mjModel.opt.gravity of the model that drove THIS "
                             "run, read after the scene compiled")},
        "object_aabb": ({"min": list(obj_lo), "max": list(obj_hi)}
                        if obj_lo else None),
        "driver": {"path": str(driver_path) if driver_path else None,
                   "error": driver_error,
                   "describe": (driver.describe()
                                if driver is not None
                                and hasattr(driver, "describe") else None)},
    }

    # -- record ------------------------------------------------------------
    t0 = float(data.time)
    times = []
    tracks = [[] for _ in range(model.nbody)]
    obj_xyz, obj_rot, ee_xyz, ee_rot = [], [], [], []
    pairs = []
    grip_contacts = []
    total_observed = 0
    distinct_named = 0
    grip_total = 0
    truncated = False

    def sample():
        times.append(round(float(data.time) - t0, 6))
        for i in range(model.nbody):
            p = data.xpos[i]
            tracks[i].append([round(float(p[0]), 6), round(float(p[1]), 6),
                              round(float(p[2]), 6)])
        if obj_bid >= 0:
            obj_xyz.append([round(float(v), 6) for v in data.xpos[obj_bid]])
            obj_rot.append([round(float(v), 6) for v in data.xmat[obj_bid]])
        if ee_bid >= 0:
            ee_xyz.append([round(float(v), 6) for v in data.xpos[ee_bid]])
            ee_rot.append([round(float(v), 6) for v in data.xmat[ee_bid]])

    sample()          # BEFORE the first step, so t=0 is the settled state

    want = float(duration)
    declared = getattr(driver, "DURATION_S", None) if driver is not None else None
    if isinstance(declared, (int, float)) and 0 < float(declared) < want:
        want = float(declared)
    max_steps = int(round(want / dt))

    for step in range(1, max_steps + 1):
        drive(float(data.time) - t0)
        mujoco.mj_step(model, data)

        emit = (step % int(contact_stride) == 0)
        seen = set()
        for c in range(int(data.ncon)):
            con = data.contact[c]
            b1 = int(model.geom_bodyid[int(con.geom1)])
            b2 = int(model.geom_bodyid[int(con.geom2)])
            n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b1) or ""
            n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b2) or ""
            total_observed += 1
            if b1 != b2 and n1 and n2:
                distinct_named += 1
            holder = None
            if b1 in arm_ids and b2 == obj_bid:
                holder = n1
            elif b2 in arm_ids and b1 == obj_bid:
                holder = n2
            if holder is not None:
                grip_total += 1
                if emit and len(grip_contacts) < CONTACT_RECORD_CAP:
                    grip_contacts.append({
                        "holder_body": holder, "held_body": roles["object"],
                        "point": [round(float(x), 6) for x in con.pos],
                        "step": int(step),
                        "t_s": round(float(data.time) - t0, 6)})
            if not emit or (b1, b2) in seen:
                continue
            seen.add((b1, b2))
            if len(pairs) >= CONTACT_RECORD_CAP:
                truncated = True
                continue
            pairs.append({"a": n1, "b": n2,
                          "a_robot": bool(b1 in arm_ids),
                          "b_robot": bool(b2 in arm_ids),
                          "point": [round(float(x), 6) for x in con.pos],
                          "step": int(step),
                          "t_s": round(float(data.time) - t0, 6)})

        if step % int(record_stride) == 0:
            sample()

    recorded_s = float(times[-1]) if times else 0.0
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
             for i in range(model.nbody)]
    (out / "trajectory.json").write_text(json.dumps({
        "dt_s": dt * record_stride, "recorded_s": recorded_s,
        "complete": True, "record_stride_steps": int(record_stride),
        "physics_timestep_s": dt,
        "bodies": [{"name": names[i], "id": i, "t": times, "xyz": tracks[i]}
                   for i in range(model.nbody)],
    }), encoding="utf-8")

    (out / "contacts.json").write_text(json.dumps({
        "supported": True, "steps": int(max_steps), "window_s": recorded_s,
        "total_observed": int(total_observed),
        "distinct_named": int(distinct_named),
        "emit_stride_steps": int(contact_stride),
        "records_truncated": bool(truncated), "pairs": pairs,
        "source": "mjData.contact over every physics step of the recorded "
                  "window; geoms mapped to bodies through mjModel.geom_bodyid",
    }), encoding="utf-8")

    t2doc["t_s"] = times
    t2doc["object"] = {"body": roles["object"], "xyz": obj_xyz,
                       "rot": obj_rot,
                       "source": POSE_SOURCE % (record_stride,
                                                dt * record_stride)}
    t2doc["end_effector"] = {"body": roles["end_effector"], "xyz": ee_xyz,
                             "rot": ee_rot,
                             "source": POSE_SOURCE % (record_stride,
                                                      dt * record_stride)}
    t2doc["grip"] = {
        "mechanism": ("attachment"
                      if (obj_bid >= 0 and equality_binds(mujoco, model,
                                                          obj_bid))
                      else ("friction" if grip_total else "unknown")),
        "attachment": (None if obj_bid < 0
                       else bool(equality_binds(mujoco, model, obj_bid))),
        "equality_constraints_in_the_model": int(model.neq),
        "supported": True, "steps": int(max_steps), "window_s": recorded_s,
        "total_observed": int(total_observed),
        "distinct_named": int(distinct_named),
        "contacts_naming_the_arm_and_the_object": int(grip_total),
        "emit_stride_steps": int(contact_stride),
        "contacts": grip_contacts, "source": GRIP_SOURCE}
    (out / "t2.json").write_text(json.dumps(t2doc), encoding="utf-8")

    warnings = {}
    for i in range(len(data.warning)):
        n = int(data.warning[i].number)
        if n:
            warnings[t1_runner._warning_name(mujoco, i)] = n  # noqa: SLF001

    (out / "completion.json").write_text(json.dumps({
        "complete": bool(control is not None and control_error is None),
        "recorded_s": recorded_s, "target_s": want,
        "steps": int(max_steps), "dt_s": dt,
        "record_stride_steps": int(record_stride),
        "ctrl_writes": int(ctrl_writes),
        "sim_time_at_exit_s": float(data.time), "settle_s": float(settle),
        "driver": str(driver_path) if driver_path else None,
        "driver_error": driver_error, "control_error": control_error,
        "roles": dict(roles),
        "arm_root_body": (names[arm_root] if arm_root > 0 else None),
        "model": {"nq": int(model.nq), "nv": int(model.nv),
                  "nu": int(model.nu), "nbody": int(model.nbody),
                  "ngeom": int(model.ngeom), "njnt": int(model.njnt),
                  "neq": int(model.neq)},
        "engine": t1_runner.engine_facts(mujoco, model),
        "warnings": warnings,
        "saved_xml": str(saved_xml) if saved_xml else None,
        "scene": str(scene_path),
    }, indent=1), encoding="utf-8")

    print("recorded %.3f s over %d steps; %d contacts observed, %d naming the "
          "arm and the object" % (recorded_s, max_steps, total_observed,
                                  grip_total))
    if control_error:
        return 8
    return 0


# --- the subprocess launcher -------------------------------------------------


class MujocoT2PhaseB:
    """One phase-B run, in the shape ``ladder.graders.t2`` expects back."""

    def __init__(self, run_dir):
        self.run_dir = Path(run_dir)
        self.scene = None
        self.driver = None
        self.rc = None
        self.wall_s = None
        self.timed_out = False
        self.attempts_used = 1
        self.error = None

    def as_dict(self):
        return {"run_dir": str(self.run_dir),
                "scene": str(self.scene) if self.scene else None,
                "driver": str(self.driver) if self.driver else None,
                "exit_code": self.rc, "wall_s": self.wall_s,
                "timed_out": bool(self.timed_out),
                "attempts_used": int(self.attempts_used),
                "error": self.error}


def launch(scene, out_dir, *, driver=None, duration=60.0, settle=0.5,
           contact_stride=10, record_stride=RECORD_STRIDE, roles=None,
           surfaces=(), timeout_s=1800.0, python=None):
    """Run :func:`run` as a subprocess and write ``process.json``.

    A subprocess and not a function call, for the reason the T1 launcher gives:
    ``ProcessFacts`` wants a real exit code and a real captured error stream,
    and neither exists inside the grader's own interpreter.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    cmd = [python or sys.executable, str(Path(__file__).resolve()),
           "--scene", str(scene), "--out", str(out),
           "--duration", "%g" % float(duration),
           "--settle", "%g" % float(settle),
           "--contact-stride", "%d" % int(contact_stride),
           "--record-stride", "%d" % int(record_stride)]
    if driver:
        cmd += ["--driver", str(driver)]
    for key, flag in (("object", "--object"),
                      ("end_effector", "--end-effector"),
                      ("container", "--container")):
        if roles and roles.get(key):
            cmd += [flag, str(roles[key])]
    if surfaces:
        cmd += ["--surfaces", ",".join(str(s) for s in surfaces)]

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
        rc, so, se = None, "", "failed to launch: %r" % (exc,)
    wall = time.time() - t

    (out / "stdout.log").write_text(so or "", encoding="utf-8")
    (out / "stderr.log").write_text(se or "", encoding="utf-8")
    doc = {"exit_code": rc, "timed_out": bool(timed_out),
           "wall_s": round(wall, 3), "attempts_used": 1, "command": cmd,
           "python": cmd[0], "mujoco_version": t1_runner._mujoco_version(),
           "platform": sys.platform}
    (out / "process.json").write_text(json.dumps(doc, indent=1),
                                      encoding="utf-8")
    return doc


def run_standalone(deliverable, run_dir, *, backend=None, duration=60.0,
                   settle=0.5, stride=10, surfaces=(), timeout_s=1800.0,
                   roles=None, record_stride=RECORD_STRIDE):
    """The hook ``ladder.graders.t2.run_and_grade`` calls. Never raises.

    ``backend`` is accepted and ignored: MuJoCo has one physics backend and no
    engine-selection field, which is exactly why this column's attribution is a
    per-run reading of ``mjOption`` rather than a sidecar.
    """
    res = MujocoT2PhaseB(run_dir)
    scene, driver = resolve_deliverable(deliverable)
    res.scene, res.driver = scene, driver
    if scene is None:
        res.error = ("no MJCF scene in the deliverable %s -- there is nothing "
                     "to re-run" % deliverable)
        return res
    if roles is None:
        roles = _roles_from_task()
    proc = launch(scene, run_dir, driver=driver, duration=duration,
                  settle=settle, contact_stride=stride,
                  record_stride=record_stride, roles=roles, surfaces=surfaces,
                  timeout_s=timeout_s)
    res.rc = proc.get("exit_code")
    res.wall_s = proc.get("wall_s")
    res.timed_out = bool(proc.get("timed_out"))
    return res


def _roles_from_task():
    """Which body is which, from the TASK file. Never from this adapter.

    ``ladder_evidence.RoleNames`` says why: an adapter that could choose which
    body counts as "the object" could choose the one that happens to be in the
    container. The sampler is *told* the names so it can record them, and it
    writes down the name it actually sampled; the core then checks that against
    the same task file and refuses a mismatch.
    """
    try:
        from ladder import tasks as ladder_tasks
        spec = ladder_tasks.get(TASK).roles
        return {"object": spec.get("object", ""),
                "end_effector": spec.get("end_effector", ""),
                "container": spec.get("container", "")}
    except Exception:  # noqa: BLE001  (adapter rule 1)
        return dict(DEFAULT_ROLES)


# --- CLI ---------------------------------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scene", required=True)
    ap.add_argument("--driver", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--settle", type=float, default=0.5)
    ap.add_argument("--contact-stride", type=int, default=10)
    ap.add_argument("--record-stride", type=int, default=RECORD_STRIDE)
    ap.add_argument("--object", default=DEFAULT_ROLES["object"])
    ap.add_argument("--end-effector", default=DEFAULT_ROLES["end_effector"])
    ap.add_argument("--container", default=DEFAULT_ROLES["container"])
    ap.add_argument("--surfaces", default="")
    a = ap.parse_args(argv)
    driver = a.driver
    if driver is None:
        _s, driver = resolve_deliverable(a.scene)
    return run(a.scene, a.out, driver_path=driver, duration=a.duration,
               settle=a.settle, contact_stride=a.contact_stride,
               record_stride=a.record_stride,
               roles={"object": a.object, "end_effector": a.end_effector,
                      "container": a.container},
               surfaces=[s for s in a.surfaces.split(",") if s])


__all__ = ["CONTACT_RECORD_CAP", "DEFAULT_ROLES", "DRIVER_NAME",
           "GRIP_SOURCE", "MujocoT2PhaseB", "POSE_SOURCE", "RECORD_STRIDE",
           "RIM_RULE", "SCENE_NAME", "STATIC_SOURCE", "container_geometry",
           "equality_binds", "geom_world_aabb", "launch", "load_driver",
           "main", "object_physics", "resolve_deliverable", "run",
           "run_standalone", "subtree_aabb", "subtree_bodies",
           "support_surfaces"]


if __name__ == "__main__":
    sys.exit(main())
