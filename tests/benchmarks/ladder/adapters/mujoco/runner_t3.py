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

"""**Phase B for T3 on MuJoCo**: re-run a deliverable cold, with OUR sampler.

SPEC §2.3 and ``capability-ladder-plan.md`` §2: a ladder cell is scored from a
**standalone cold re-run** of whatever the attempt produced, with the grader's
own sampler injected and no agent present. MuJoCo has no world file, no
controller process and no engine to launch, so the equivalent here is the same
one :mod:`runner_t2` uses:

    load the deliverable's scene, import the deliverable's driver **by path**,
    and step the loop ourselves -- writing down everything on the way.

A T3 deliverable on this column is two files::

    <deliverable>/scene.xml   an MJCF the compiler accepts
    <deliverable>/drive.py    a module exposing control(model, data, t_s),
                              optionally setup(model, data) and DURATION_S

The driver is imported with ``importlib.util.spec_from_file_location`` and
**is not put on ``sys.path``** -- a driver that needed the benchmark's own
package tree would not be a deliverable that stands alone.

The eight channels this sampler exists to answer
------------------------------------------------

``ladder.graders.t3_evidence`` lists what T3 needs and the frozen AgentBench
bundle has no field for. Every one is reachable through MuJoCo's public API:

======================  ====================================================
channel                 where it comes from
======================  ====================================================
``base_pose``           ``mjData.xpos`` **+ ``mjData.xmat``** for the body the
                        task names, once per ``record_stride`` steps.
                        ``xmat`` is a world-from-body 3x3 already, so no
                        quaternion or Euler convention is involved -- and
                        without it half of T3.2 (roll and pitch at every
                        sample) cannot be answered at all.
``standing_height``     the base's own world z at the end of the settle
                        window, before the first recorded step.
``gait_contacts``       ``mjData.contact`` scanned at every physics step,
                        emitted every ``contact_stride`` steps **together with
                        the times the query ran** -- because a transition is a
                        change of state and a list of touches carries no
                        evidence of a *not*-touch.
``applied_support``     ``mjData.xfrc_applied`` on the base, per recorded
                        sample, as force (N) and torque (N.m). See
                        :data:`SUPPORT_SOURCE` for why that array is the whole
                        answer on this column rather than part of it.
``arena``               the world AABB of the static, non-robot bodies the
                        task names as the walking surface. Exact, because the
                        floor this column builds is a **box** and not a plane
                        (``t3_scene``: a plane's AABB is 1e10 m).
``base_mass``           ``mjModel.body_mass`` -- kilograms, which the neutral
                        inventory has no field for at all.
``gravity``             ``mjModel.opt.gravity``, of the model that ran.
``controller_load``     the driver's path, its **sha256**, whether ``setup()``
                        ran, and how many times ``control()`` wrote actuator
                        commands. An exit code is not evidence and this is not
                        an exit code.
======================  ====================================================

⚠ **``other_is_ground`` is a structural claim and its limit is stated.** This
sampler answers it as *"the other body is welded to the world and is not part
of the robot's kinematic subtree"* -- so a static **wall** would satisfy it
too. That is exactly why the task's own name list is authoritative and why
``t3_evidence.classify_contacts`` takes the stricter of the two readings: an
adapter free to decide what counts as the ground could label the thing holding
the robot up as the floor and pass T3.4 outright.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder.adapters.mujoco import runner as t1_runner  # noqa: E402
from ladder.adapters.mujoco import runner_t2  # noqa: E402

TASK = "T3_quadruped"
DRIVER_NAME = runner_t2.DRIVER_NAME          # "drive.py" -- one shape per column
SCENE_NAME = runner_t2.SCENE_NAME            # "scene.xml"

# One pose sample every this many physics steps. The tier's own witness bound
# is MAX_SAMPLE_DT_S = 0.05 s; at the 2 ms timestep this scene uses, 5 steps is
# 10 ms -- five times finer than the bound, so T3.1's continuity clause and
# T3.3's speed have a real witness rather than a formality.
RECORD_STRIDE = 5
CONTACT_RECORD_CAP = 400000

DEFAULT_BASE = "base_link"
DEFAULT_SURFACES = ("ground", "floor", "terrain", "ground_plane", "plane",
                    "flat_ground", "arena", "arena_floor")

POSE_SOURCE = (
    "the grader-owned phase-B sampler: mjData.xpos and mjData.xmat for the "
    "body the task names as the base, one sample every %d physics steps "
    "(%.4f s), in simulated seconds and metres, world frame. mjData.xmat is "
    "already a world-from-body 3x3, so no quaternion or Euler convention is "
    "involved -- the rotation the grader decomposes into roll and pitch is the "
    "one the engine integrated")

STANDING_SOURCE = (
    "the base's own world z read between the last mj_step of the settle window "
    "and the first recorded one, in a loop this process owns. MuJoCo has no "
    "simulator process and no scene service, so nothing can advance the clock "
    "while the read happens: this is a settled standing height by construction "
    "rather than by arrangement")

CONTACT_SOURCE = (
    "mjData.contact scanned at EVERY physics step of the recorded window and "
    "emitted every %d steps, each contact's geom1/geom2 mapped to bodies "
    "through mjModel.geom_bodyid and named with mj_id2name. THE TIMES THE "
    "QUERY RAN are recorded beside the contacts, which is what makes a LIFTED "
    "foot observable: a list of touches carries no evidence of a not-touch. "
    "'the other side is the ground' is answered STRUCTURALLY -- the other body "
    "is welded to the world (mjModel.body_weldid == 0) and is not in the "
    "robot's kinematic subtree -- so it is true of a static wall too, which is "
    "why the task's own name list is authoritative and the grader takes the "
    "stricter of the two readings")

SUPPORT_SOURCE = (
    "mjData.xfrc_applied for the base body, per recorded sample, as a force "
    "(N) and a torque (N.m) in the world frame. THAT ARRAY IS THE WHOLE ANSWER "
    "ON THIS COLUMN, and here is the argument rather than the assertion: in "
    "MuJoCo a wrench can reach a body through exactly four routes -- gravity "
    "(excluded by the tier), contact (excluded by the tier, and counted "
    "separately above), an actuator transmission, or xfrc_applied. This "
    "scene's actuators are all mjTRN_JOINT transmissions on HINGES, none of "
    "them on the base's free joint, and mjModel.neq is recorded beside this so "
    "a reader can see that no equality constraint binds the base either. Both "
    "counts are written into the run: if either is non-zero the attestation is "
    "incomplete and the row should be read as such")

ARENA_SOURCE = (
    "the union of the world AABBs of the static, non-robot bodies the task "
    "names as the walking surface, computed by the sampler from "
    "mjModel.geom_aabb carried through mjData.geom_xpos/geom_xmat at t=0. "
    "EXACT on this column because the floor is a BOX: MuJoCo gives a plane "
    "geom an AABB of +/-1e10 m, so a column that built its floor as a plane "
    "cannot answer this channel with a number anybody can read")

CONTROLLER_SOURCE = (
    "the phase-B runner's own record of importing the deliverable's driver by "
    "path: the file, its sha256, whether setup() ran without raising, and the "
    "number of times control() wrote actuator commands. It is a POSITIVE "
    "attestation and it is not an exit code -- the trap T3.5 exists for is a "
    "deploy whose model runtime was missing, which runs a zero-residual "
    "baseline and exits 0")


# --- the deliverable ---------------------------------------------------------


def resolve_deliverable(path):
    """``(scene, driver)`` from a directory or a scene file. Never raises."""
    return runner_t2.resolve_deliverable(path)


def load_driver(path):
    """Import a deliverable's driver by path. ``None`` if there is not one."""
    return runner_t2.load_driver(path)


def sha256(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


# --- the run -----------------------------------------------------------------


def run(scene_path, out_dir, *, driver_path=None, duration=180.0, settle=1.0,
        contact_stride=2, record_stride=RECORD_STRIDE, base=DEFAULT_BASE,
        surfaces=DEFAULT_SURFACES, method="scripted"):
    """Load the scene, drive it with the deliverable's own driver, record.

    Returns a process exit code. Never raises for a physical failure -- a scene
    that will not compile, a driver that will not import and a robot that falls
    over on its first step are all measurements.
    """
    import mujoco

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    surfaces = tuple(surfaces or DEFAULT_SURFACES)

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
    setup_ok = None
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
            setup_ok = True
        except Exception as exc:  # noqa: BLE001
            setup_ok = False
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

    # -- settle (not recorded): the feet find the floor --------------------
    for _ in range(int(round(float(settle) / dt))):
        drive(0.0)
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # -- the frozen t=0 scan ----------------------------------------------
    base_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base)
    robot_ids = (t1_runner.robot_subtree(mujoco, model, base_bid)
                 if base_bid >= 0 else set())
    roster = t1_runner.scan_roster(mujoco, model, data, robot_ids)
    (out / "roster.json").write_text(json.dumps(roster, indent=1),
                                     encoding="utf-8")

    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
             for i in range(model.nbody)]
    ground_ids = _ground_bodies(mujoco, model, robot_ids)
    standing_z = (float(data.xpos[base_bid][2]) if base_bid >= 0 else None)

    t3doc = {
        "base_body": base, "record_stride_steps": int(record_stride),
        "contact_stride_steps": int(contact_stride),
        "physics_timestep_s": dt,
        "standing": {"z_m": standing_z, "body": base,
                     "t_s": float(data.time), "source": STANDING_SOURCE},
        "arena": _arena(mujoco, model, data, surfaces, robot_ids, names),
        "base_physics": _base_physics(mujoco, model, base, base_bid),
        "world": {"gravity_vec_mps2": [float(x) for x in model.opt.gravity],
                  "gravity_mps2": float(sum(float(x) ** 2
                                            for x in model.opt.gravity) ** 0.5),
                  "source": ("mjModel.opt.gravity of the model that drove THIS "
                             "run, read after the scene compiled")},
        "model": {"neq": int(model.neq), "nu": int(model.nu),
                  "actuators_on_the_base_free_joint": _actuators_on_base(
                      mujoco, model, base_bid),
                  "why_those_two_are_here": "they are what makes the applied-"
                                            "wrench total complete rather than "
                                            "partial -- see the support source"},
        "declared_efforts_nm": _declared_efforts(out),
        "structurally_static_non_robot_bodies": sorted(
            names[i] for i in ground_ids if names[i]),
    }

    # -- record ------------------------------------------------------------
    t0 = float(data.time)
    times, tracks = [], [[] for _ in range(model.nbody)]
    base_xyz, base_rot = [], []
    sup_t, sup_f, sup_m = [], [], []
    peak_ctrl_force = [0.0] * int(model.nu)
    gait_times, gait_contacts = [], []
    pairs = []
    total_observed = 0
    distinct_named = 0
    truncated = False

    def sample():
        times.append(round(float(data.time) - t0, 6))
        for i in range(model.nbody):
            p = data.xpos[i]
            tracks[i].append([round(float(p[0]), 6), round(float(p[1]), 6),
                              round(float(p[2]), 6)])
        if base_bid >= 0:
            base_xyz.append([round(float(v), 6) for v in data.xpos[base_bid]])
            base_rot.append([round(float(v), 6) for v in data.xmat[base_bid]])
            sup_t.append(round(float(data.time) - t0, 6))
            w = data.xfrc_applied[base_bid]
            sup_f.append([float(w[0]), float(w[1]), float(w[2])])
            sup_m.append([float(w[3]), float(w[4]), float(w[5])])

    sample()          # BEFORE the first step, so t=0 is the settled state

    want = float(duration)
    declared = getattr(driver, "DURATION_S", None) if driver is not None else None
    if isinstance(declared, (int, float)) and 0 < float(declared) < want:
        want = float(declared)
    max_steps = int(round(want / dt))

    for step in range(1, max_steps + 1):
        drive(float(data.time) - t0)
        mujoco.mj_step(model, data)
        for a in range(int(model.nu)):
            f = abs(float(data.actuator_force[a]))
            if f > peak_ctrl_force[a]:
                peak_ctrl_force[a] = f

        emit = (step % int(contact_stride) == 0)
        if emit:
            gait_times.append(round(float(data.time) - t0, 6))
        seen = set()
        for c in range(int(data.ncon)):
            con = data.contact[c]
            b1 = int(model.geom_bodyid[int(con.geom1)])
            b2 = int(model.geom_bodyid[int(con.geom2)])
            n1, n2 = names[b1], names[b2]
            total_observed += 1
            if b1 != b2 and n1 and n2:
                distinct_named += 1
            r1, r2 = b1 in robot_ids, b2 in robot_ids
            if emit and (r1 != r2) and len(gait_contacts) < CONTACT_RECORD_CAP:
                robot_side, other = ((b1, b2) if r1 else (b2, b1))
                gait_contacts.append({
                    "robot_body": names[robot_side],
                    "other_body": names[other],
                    "other_is_ground": bool(other in ground_ids),
                    "other_is_robot": False,
                    "point": [round(float(x), 5) for x in con.pos],
                    "step": int(step),
                    "t_s": round(float(data.time) - t0, 6)})
            if not emit or (b1, b2) in seen:
                continue
            seen.add((b1, b2))
            if len(pairs) >= CONTACT_RECORD_CAP:
                truncated = True
                continue
            pairs.append({"a": n1, "b": n2, "a_robot": bool(r1),
                          "b_robot": bool(r2),
                          "point": [round(float(x), 5) for x in con.pos],
                          "step": int(step),
                          "t_s": round(float(data.time) - t0, 6)})

        if step % int(record_stride) == 0:
            sample()

    recorded_s = float(times[-1]) if times else 0.0
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
        "source": CONTACT_SOURCE % int(contact_stride),
    }), encoding="utf-8")

    t3doc["t_s"] = times
    t3doc["base_pose"] = {
        "body": base, "xyz": base_xyz, "rot": base_rot,
        "source": POSE_SOURCE % (record_stride, dt * record_stride)}
    t3doc["gait"] = {
        "contacts": gait_contacts, "sample_times": gait_times,
        "supported": True, "steps": int(max_steps), "window_s": recorded_s,
        "total_observed": int(total_observed),
        "distinct_named": int(distinct_named),
        "emit_stride_steps": int(contact_stride),
        "records_truncated": bool(len(gait_contacts) >= CONTACT_RECORD_CAP),
        "source": CONTACT_SOURCE % int(contact_stride)}
    t3doc["support"] = {
        "attested": True, "t": sup_t, "force": sup_f, "torque": sup_m,
        "source": SUPPORT_SOURCE}
    t3doc["controller"] = {
        "declared_method": str(method),
        "loaded": bool(driver is not None and control is not None
                       and control_error is None and setup_ok is not False
                       and ctrl_writes > 0),
        "evidence": ("the runner imported %s by path, its setup() returned "
                     "without raising, and its control() wrote actuator "
                     "commands %d times over %d physics steps"
                     % (DRIVER_NAME, ctrl_writes, max_steps)
                     if driver is not None else
                     (driver_error or "no driver was imported")),
        "identity": ("%s sha256=%s" % (Path(driver_path).name, sha256(driver_path))
                     if driver_path else ""),
        "setup_ran": setup_ok, "ctrl_writes": int(ctrl_writes),
        "error": control_error or driver_error,
        "source": CONTROLLER_SOURCE}
    t3doc["actuator_peak_force"] = {
        (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or "#%d" % a):
            round(peak_ctrl_force[a], 4) for a in range(int(model.nu))}
    t3doc["driver"] = {
        "path": str(driver_path) if driver_path else None,
        "sha256": sha256(driver_path) if driver_path else None,
        "error": driver_error,
        "describe": (driver.describe()
                     if driver is not None and hasattr(driver, "describe")
                     else None)}
    (out / "t3.json").write_text(json.dumps(t3doc), encoding="utf-8")

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
        "base_body": base, "robot_subtree_bodies": sorted(
            names[i] for i in robot_ids if names[i]),
        "model": {"nq": int(model.nq), "nv": int(model.nv),
                  "nu": int(model.nu), "nbody": int(model.nbody),
                  "ngeom": int(model.ngeom), "njnt": int(model.njnt),
                  "neq": int(model.neq)},
        "engine": t1_runner.engine_facts(mujoco, model),
        "warnings": warnings,
        "saved_xml": str(saved_xml) if saved_xml else None,
        "scene": str(scene_path),
    }, indent=1), encoding="utf-8")

    print("recorded %.3f s over %d steps; base travelled %.3f m from its start; "
          "%d contacts observed, %d robot-to-world records over %d queries"
          % (recorded_s, max_steps,
             (((base_xyz[-1][0] - base_xyz[0][0]) ** 2
               + (base_xyz[-1][1] - base_xyz[0][1]) ** 2) ** 0.5
              if len(base_xyz) > 1 else 0.0),
             total_observed, len(gait_contacts), len(gait_times)))
    if control_error:
        return 8
    return 0


# --- the frozen t=0 reads ----------------------------------------------------


def _ground_bodies(mujoco, model, robot_ids):
    """Static, non-robot bodies carrying geometry -- the structural reading.

    See the module docstring: this is true of a static WALL as well as of the
    floor, which is why the task's name list is authoritative and the grader
    resolves every disagreement towards the stricter reading.
    """
    out = set()
    for i in range(1, model.nbody):
        if i in robot_ids or int(model.body_weldid[i]) != 0:
            continue
        if int(model.body_geomnum[i]) <= 0:
            continue
        out.add(int(i))
    return out


def _arena(mujoco, model, data, surfaces, robot_ids, names):
    """The walking region: the union AABB of the NAMED walking surfaces."""
    want = {str(s).strip().lower() for s in surfaces}
    lo = hi = None
    seen = []
    for i in range(1, model.nbody):
        if i in robot_ids or (names[i] or "").strip().lower() not in want:
            continue
        a, b = t1_runner.body_world_aabb(mujoco, model, data, i)
        if a is None:
            continue
        seen.append(names[i])
        lo = list(a) if lo is None else [min(x, y) for x, y in zip(lo, a)]
        hi = list(b) if hi is None else [max(x, y) for x, y in zip(hi, b)]
    if lo is None:
        return {"error": "no body the task names as the walking surface (%s) "
                         "is in the scene with geometry"
                         % ", ".join(sorted(want))}
    return {"aabb_min": lo, "aabb_max": hi, "bodies": seen,
            "boundary_bodies": [], "source": ARENA_SOURCE}


def _base_physics(mujoco, model, name, bid):
    if bid < 0:
        return {"body": name, "error": "no body named %r is in the scene"
                                       % name}
    mass = float(model.body_mass[bid])
    return {"body": name, "mass_kg": mass,
            "subtree_mass_kg": float(model.body_subtreemass[bid]),
            "dynamic": bool(int(model.body_weldid[bid]) != 0 and mass > 0.0),
            "source": ("mjModel.body_mass for the named body, read off the "
                       "compiled model that drove this run. 'dynamic' is "
                       "mjModel.body_weldid != 0 (the body is NOT in the "
                       "world's weld group, so it can move) AND mass > 0")}


def _actuators_on_base(mujoco, model, bid):
    """How many actuators act on the base's own free joint. Should be zero."""
    if bid < 0:
        return None
    n = 0
    for a in range(int(model.nu)):
        if int(model.actuator_trntype[a]) != int(mujoco.mjtTrn.mjTRN_JOINT):
            n += 1
            continue
        j = int(model.actuator_trnid[a][0])
        if j >= 0 and int(model.jnt_bodyid[j]) == int(bid):
            n += 1
    return int(n)


def _declared_efforts(out_dir):
    """The efforts the description declares, from the build record if present.

    Recorded next to the measured peak actuator forces so *"did the walk stay
    inside the robot's own limits"* is answerable from the run, whether or not
    the build clamped them.
    """
    for cand in (Path(out_dir) / "build.json",
                 Path(out_dir).parent / "build.json"):
        try:
            doc = json.loads(cand.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        got = doc.get("urdf_declared_effort_nm")
        if isinstance(got, dict):
            return got
    return {}


# --- the subprocess launcher -------------------------------------------------


class MujocoT3PhaseB:
    """One phase-B run, in the shape ``ladder.graders.t3`` expects back."""

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


def launch(scene, out_dir, *, driver=None, duration=180.0, settle=1.0,
           contact_stride=2, record_stride=RECORD_STRIDE, base=DEFAULT_BASE,
           surfaces=(), method="scripted", timeout_s=3600.0, python=None):
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
           "--record-stride", "%d" % int(record_stride),
           "--base", str(base), "--method", str(method)]
    if driver:
        cmd += ["--driver", str(driver)]
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
           "python": cmd[0],
           "mujoco_version": t1_runner._mujoco_version(),  # noqa: SLF001
           "platform": sys.platform}
    (out / "process.json").write_text(json.dumps(doc, indent=1),
                                      encoding="utf-8")
    return doc


def run_standalone(deliverable, run_dir, *, backend=None, duration=180.0,
                   settle=1.0, stride=2, surfaces=(), timeout_s=3600.0,
                   base=None, record_stride=RECORD_STRIDE, method="scripted"):
    """The T3 phase-B hook. Never raises.

    ``backend`` is accepted and ignored: MuJoCo has one physics backend and no
    engine-selection field, which is exactly why this column's attribution is a
    per-run reading of ``mjOption`` rather than a sidecar.
    """
    res = MujocoT3PhaseB(run_dir)
    scene, driver = resolve_deliverable(deliverable)
    res.scene, res.driver = scene, driver
    if scene is None:
        res.error = ("no MJCF scene in the deliverable %s -- there is nothing "
                     "to re-run" % deliverable)
        return res
    if base is None:
        base = _base_from_task()
    proc = launch(scene, run_dir, driver=driver, duration=duration,
                  settle=settle, contact_stride=stride,
                  record_stride=record_stride, base=base,
                  surfaces=surfaces or DEFAULT_SURFACES, method=method,
                  timeout_s=timeout_s)
    res.rc = proc.get("exit_code")
    res.wall_s = proc.get("wall_s")
    res.timed_out = bool(proc.get("timed_out"))
    return res


def _base_from_task():
    """Which body is the base, from the TASK file. Never from this adapter.

    T3's core says why: an adapter that could choose which body counts as "the
    base" could choose the body that travelled furthest. The sampler is *told*
    the name so it can record it, and it writes down the name it actually
    sampled; the core then checks that against the same task file and refuses a
    mismatch.
    """
    try:
        from ladder import tasks as ladder_tasks
        return ladder_tasks.get(TASK).robot_name or DEFAULT_BASE
    except Exception:  # noqa: BLE001  (adapter rule 1)
        return DEFAULT_BASE


# --- CLI ---------------------------------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scene", required=True)
    ap.add_argument("--driver", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--contact-stride", type=int, default=2)
    ap.add_argument("--record-stride", type=int, default=RECORD_STRIDE)
    ap.add_argument("--base", default=DEFAULT_BASE)
    ap.add_argument("--method", default="scripted")
    ap.add_argument("--surfaces", default="")
    a = ap.parse_args(argv)
    driver = a.driver
    if driver is None:
        _s, driver = resolve_deliverable(a.scene)
    return run(a.scene, a.out, driver_path=driver, duration=a.duration,
               settle=a.settle, contact_stride=a.contact_stride,
               record_stride=a.record_stride, base=a.base, method=a.method,
               surfaces=[s for s in a.surfaces.split(",") if s]
               or DEFAULT_SURFACES)


__all__ = ["ARENA_SOURCE", "CONTACT_RECORD_CAP", "CONTACT_SOURCE",
           "CONTROLLER_SOURCE", "DEFAULT_BASE", "DEFAULT_SURFACES",
           "DRIVER_NAME", "MujocoT3PhaseB", "POSE_SOURCE", "RECORD_STRIDE",
           "SCENE_NAME", "STANDING_SOURCE", "SUPPORT_SOURCE", "launch",
           "load_driver", "main", "resolve_deliverable", "run",
           "run_standalone", "sha256"]


if __name__ == "__main__":
    sys.exit(main())
