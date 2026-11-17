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

"""**Phase B for T4 on MuJoCo**: re-run a deliverable cold, with OUR sampler.

SPEC §2.3 and ``capability-ladder-plan.md`` §2: a ladder cell is scored from a
**standalone cold re-run** of whatever the attempt produced, with the grader's
own sampler injected and no agent present. MuJoCo has no world file, no
controller process and no engine to launch, so the equivalent here is the one
:mod:`runner_t2` and :mod:`runner_t3` use::

    load the deliverable's scene, import the deliverable's driver **by path**,
    and step the loop ourselves -- writing down everything on the way.

A T4 deliverable on this column is two files::

    <deliverable>/scene.xml   an MJCF the compiler accepts
    <deliverable>/drive.py    a module exposing control(model, data, t_s),
                              optionally setup(model, data) and DURATION_S

**T4's evidence contract is T3's** (``t4_evidence``: *"this tier replaces an
assertion, not an evidence contract"*), so the eight channels below are the
eight channels one rung down, sampled the same way. Everything genuinely new in
this file is in the **support** channel, and it is new for one reason.

--------------------------------------------------------------------------
⚠ A support rig built as a CONSTRAINT applies no ``xfrc_applied`` at all
--------------------------------------------------------------------------

``tasks/T4_humanoid/meta.json`` ->
``container.authored_here.an_open_question_the_demonstration_exposed`` states
the hole and states that no grader can close it:

    *"A rig implemented instead as a weld, an equality constraint, a mocap
    attachment or a kinematic base would hold the robot just as firmly and the
    wrench channel would read ZERO -- and the run would be published in
    T4-unsupported, which is the cell the plan says must be 'numerically
    nothing'. ... What a column MUST do to be honest here is count constraint
    forces on the base as applied support."*

**This column counts them**, and :func:`equality_reaction` is where. MuJoCo
assembles its constraint Jacobian in the fixed order *equality, friction loss,
limit, contact* and publishes the first two counts as ``mjData.ne`` and
``mjData.nf``, so the equality rows are exactly ``efc_J[:ne]`` and the
generalized force they apply is ``efc_J[:ne].T @ efc_force[:ne]``. Restricted
to the base's own free-joint DOFs that is a wrench about the base's centre of
mass -- **the translational part already in the world frame, the rotational
part in the body frame** (measured, not assumed: MuJoCo's free joint carries
its linear velocity globally and its angular velocity locally), so the torque
is rotated by ``mjData.xmat`` before it is added.

The contact rows are deliberately **not** included: T4.4 excludes contact by
its own wording and counts it separately, and mixing the two would publish the
floor as a support rig.

And where the reaction cannot be computed at all -- a **kinematic base**, one
welded structurally into the world's weld group, which has no DOFs for a
reaction to appear on -- this column **declines to attest** rather than
reporting zero. ``T4-support-unverified`` is a cell that says *"nobody knows"*;
``T4-unsupported`` is a cell that says *"nothing was applied"*, and a held
robot belongs in neither.

Both readings are written into the run
--------------------------------------

``t4.json`` carries the honest total **and** ``support_xfrc_only`` -- what a
column that attested ``mjData.xfrc_applied`` alone would have published. They
are identical whenever the scene has no equality constraints, and the flag
``xfrc_only_identical_to_total`` says which case a run is. That is what lets
``BRINGUP_T4.md`` §6 grade **one recording twice** and print the two cells side
by side, instead of arguing about what would have happened.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder.adapters.mujoco import runner as t1_runner  # noqa: E402
from ladder.adapters.mujoco import runner_t2, runner_t3  # noqa: E402

TASK = "T4_humanoid"
DRIVER_NAME = runner_t2.DRIVER_NAME          # "drive.py" -- one shape per column
SCENE_NAME = runner_t2.SCENE_NAME            # "scene.xml"

# One pose sample every this many physics steps. The tier's own witness bound
# is MAX_SAMPLE_DT_S = 0.05 s; at the 2 ms timestep this scene uses, 5 steps is
# 10 ms -- five times finer than the bound.
RECORD_STRIDE = runner_t3.RECORD_STRIDE
CONTACT_RECORD_CAP = runner_t3.CONTACT_RECORD_CAP

DEFAULT_BASE = "base_link"
DEFAULT_SURFACES = runner_t3.DEFAULT_SURFACES

POSE_SOURCE = runner_t3.POSE_SOURCE
STANDING_SOURCE = runner_t3.STANDING_SOURCE
ARENA_SOURCE = runner_t3.ARENA_SOURCE
CONTROLLER_SOURCE = runner_t3.CONTROLLER_SOURCE

CONTACT_SOURCE = (
    "mjData.contact scanned at EVERY physics step of the recorded window and "
    "emitted every %d steps, each contact's geom1/geom2 mapped to bodies "
    "through mjModel.geom_bodyid and named with mj_id2name, DEDUPLICATED to "
    "one record per (robot body, other body) pair per query -- this robot's "
    "feet are BOXES and a box resting flat on a plane produces four coplanar "
    "contact points that name the same pair, which multiplies the record by "
    "four and tells the tier nothing it did not already have. THE TIMES THE "
    "QUERY RAN are recorded beside the contacts, which is what makes a LIFTED "
    "foot observable: a list of touches carries no evidence of a not-touch. "
    "'the other side is the ground' is answered STRUCTURALLY -- the other body "
    "is welded to the world (mjModel.body_weldid == 0) and is not in the "
    "robot's kinematic subtree -- so it is true of a static wall too, which is "
    "why the task's own name list is authoritative and the grader takes the "
    "stricter of the two readings")

SUPPORT_SOURCE = (
    "the TOTAL non-gravitational, non-contact wrench on the base, per recorded "
    "sample, in newtons and newton-metres about the base's centre of mass in "
    "the world frame. It is the sum of TWO routes, because on this tier one of "
    "them alone would be dishonest: (1) mjData.xfrc_applied for the base body "
    "-- an explicitly applied rig; and (2) THE EQUALITY-CONSTRAINT REACTION on "
    "the base, computed as efc_J[:ne].T @ efc_force[:ne] restricted to the "
    "base's free-joint DOFs, with the rotational half rotated out of the body "
    "frame by mjData.xmat. MuJoCo assembles constraint rows in the order "
    "equality, friction, limit, contact and publishes mjData.ne, so the "
    "equality rows are separable from the CONTACT rows the tier excludes and "
    "counts elsewhere. A rig built as a weld, a connect, a mocap attachment or "
    "any other equality holds a robot exactly as firmly as an applied wrench "
    "and contributes NOTHING to xfrc_applied; LADDER_REQUIRED_EVIDENCE_T4 says "
    "a column must count it, and this is that count. The remaining routes into "
    "a body are gravity (excluded by the tier), contact (excluded and counted "
    "separately) and an actuator transmission -- mjModel.nu on the base's free "
    "joint and mjModel.ntendon are both written into the run beside this, so a "
    "reader can see the total is complete rather than take it on trust")

SUPPORT_XFRC_ONLY_SOURCE = (
    "mjData.xfrc_applied for the base body ALONE -- deliberately NOT this "
    "column's attestation. It is recorded so that the same recording can be "
    "graded twice and the difference published: it is what a column that "
    "reads only the explicitly-applied wrench would attest, and on a scene "
    "with an equality constraint on the base it reads ZERO while the robot is "
    "held rigidly upright")

CONSTRAINT_RIG_SOURCE = (
    "read off the compiled model and the solved constraint set: mjModel.neq "
    "with each equality's type and the bodies it names, mjModel.nmocap, "
    "whether the base body has a free joint at all (a KINEMATIC base has no "
    "DOFs and therefore no reaction to read), mjModel.ntendon, the number of "
    "actuators on the base's own free joint, and the peak mjData.ne observed "
    "over the run. Together they are the answer to 'is anything holding this "
    "robot up that the applied-wrench array cannot see'")


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


# --- the support channel, which is the only thing new on this rung -----------


def base_free_dofs(mujoco, model, bid):
    """``(dofadr, njnt_free)`` for the base's free joint, or ``(None, 0)``.

    A base with no free joint is a **kinematic** base: it is held by the model's
    own structure rather than by anything a constraint solver reports, and no
    reaction can be read off it at all. That is a refusal to attest, not a
    zero.
    """
    if bid is None or bid < 0:
        return None, 0
    n = 0
    adr = None
    for j in range(int(model.njnt)):
        if int(model.jnt_bodyid[j]) != int(bid):
            continue
        if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
            n += 1
            if adr is None:
                adr = int(model.jnt_dofadr[j])
    return adr, n


def equality_reaction(mujoco, model, data, bid, dofadr):
    """The wrench the EQUALITY constraints apply to the base, world frame.

    ``(force3, torque3)`` about the base's centre of mass. Zero when the model
    has no equality constraints -- and zero **by construction** in that case
    rather than by measurement, which is why the count is published beside it.

    Contact rows are excluded on purpose: T4.4 excludes contact by its own
    wording and counts it separately, and folding the floor's reaction in here
    would publish the ground as a support rig.
    """
    zero = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    ne = int(data.ne)
    if ne <= 0 or dofadr is None or int(model.neq) <= 0:
        return zero
    nefc = int(data.nefc)
    if nefc <= 0:
        return zero
    try:
        if mujoco.mj_isSparse(model):
            dense = np.zeros((nefc, int(model.nv)))
            mujoco.mju_sparse2dense(dense, data.efc_J, data.efc_J_rownnz,
                                    data.efc_J_rowadr, data.efc_J_colind)
            jac = dense
        else:
            jac = np.asarray(data.efc_J, dtype=float).reshape(nefc,
                                                              int(model.nv))
        qfrc = jac[:ne].T @ np.asarray(data.efc_force[:ne], dtype=float)
    except (AttributeError, ValueError, TypeError):
        return zero
    force = [float(x) for x in qfrc[dofadr:dofadr + 3]]
    # Measured on 3.8.1, not assumed: a free joint's linear DOFs are in the
    # world frame and its angular DOFs are in the BODY frame, so the dual
    # (generalized force) is a world force and a body-frame torque.
    rot = np.asarray(data.xmat[bid], dtype=float).reshape(3, 3)
    torque = [float(x) for x in rot @ np.asarray(qfrc[dofadr + 3:dofadr + 6],
                                                 dtype=float)]
    return force, torque


def base_physics(mujoco, model, name, bid):
    """The base's mass, with the one sentence this tier cannot do without.

    ⚠ **Which mass is "body weight" is genuinely ambiguous on a humanoid, and
    it moves the cell boundary.** The tier publishes the peak support force as
    *"a multiple of body weight"* and sets the unsupported cell at
    ``0.02 x m.g``; the grader reads that ``m`` from this channel's
    ``mass_kg``. MuJoCo offers two answers -- ``mjModel.body_mass`` (the base
    body's own mass) and ``mjModel.body_subtreemass`` (the base plus
    everything hanging off it) -- and on the shipped robot they are **12.0 kg
    and 25.6 kg**, a factor of 2.13.

    This column reports ``body_mass``, which is the literal reading of *"the
    base's mass"*, is what the tier below already reports, and is the
    **stricter** of the two (a smaller ``m`` means a tighter unsupported
    bound). The other number is recorded beside it so any figure here can be
    converted, and the ambiguity is written into the citation so it travels
    into the verdict rather than living in a bring-up note. It is the
    difference between *"peak 0.047 x body weight"* and *"peak 0.022 x body
    weight"* for one unchanged run.
    """
    doc = runner_t3._base_physics(mujoco, model, name, bid)  # noqa: SLF001
    if bid is not None and bid >= 0:
        doc["source"] = doc.get("source", "") + (
            ". NOTE ON WHICH MASS THIS IS, because the tier's cell boundary is "
            "0.02 x m.g and its published figure is a multiple of body weight: "
            "this is mjModel.body_mass for the BASE BODY ALONE (%.3f kg), NOT "
            "the whole robot (mjModel.body_subtreemass = %.3f kg, recorded "
            "beside it as subtree_mass_kg). They differ by a factor of %.2f on "
            "this robot, so every 'x body weight' figure derived from this "
            "channel is that much LARGER than the same force expressed against "
            "the robot's total mass, and the unsupported cell's force bound is "
            "that much tighter. The base-body reading is the literal one, is "
            "what the tier below already reports, and is the STRICTER of the "
            "two; which one the tier means is an open question for its owner"
            % (float(model.body_mass[bid]),
               float(model.body_subtreemass[bid]),
               (float(model.body_subtreemass[bid])
                / max(float(model.body_mass[bid]), 1e-9))))
    return doc


def constraint_rig_facts(mujoco, model, bid):
    """Everything about this scene that could hold the base without a wrench."""
    dofadr, nfree = base_free_dofs(mujoco, model, bid)
    eqs = []
    for e in range(int(model.neq)):
        try:
            kind = int(model.eq_type[e])
            o1 = int(model.eq_obj1id[e])
            o2 = int(model.eq_obj2id[e])
            objtype = int(model.eq_objtype[e]) if hasattr(model, "eq_objtype") \
                else int(mujoco.mjtObj.mjOBJ_BODY)
        except (AttributeError, IndexError, ValueError):
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, e) or ""
        names = []
        for o in (o1, o2):
            got = mujoco.mj_id2name(model, objtype, o) if o >= 0 else None
            names.append(got or "")
        eqs.append({"name": name, "type": int(kind), "objects": names,
                    "active": bool(int(model.eq_active0[e]))
                    if hasattr(model, "eq_active0") else None})
    return {
        "neq": int(model.neq), "equalities": eqs,
        "nmocap": int(model.nmocap), "ntendon": int(model.ntendon),
        "base_has_a_free_joint": bool(nfree == 1),
        "base_free_joint_dofadr": dofadr,
        "base_is_kinematic": bool(bid is not None and bid >= 0
                                  and int(model.body_weldid[bid]) == 0),
        "actuators_on_the_base_free_joint": runner_t3._actuators_on_base(  # noqa: SLF001
            mujoco, model, bid),
        "source": CONSTRAINT_RIG_SOURCE}


def support_attestation(facts):
    """``(attested, error)`` -- may this column say what was applied?

    Three things can each make the total incomplete, and each of them is a
    refusal rather than a zero:

    * the base has no free joint -- a **kinematic base**, held by the model's
      own structure, with no DOFs for a reaction to appear on;
    * an actuator acts on the base's free joint -- a wrench arriving through a
      transmission this sampler does not read;
    * the scene has tendons -- another route into a body that this sampler does
      not read, and which no scene this column builds has.
    """
    if facts.get("base_is_kinematic") or not facts.get("base_has_a_free_joint"):
        return None, (
            "the base is KINEMATIC -- it carries no free joint, so it is held "
            "by the model's own structure and there is no degree of freedom "
            "for a reaction to appear on. This column will not report zero for "
            "a robot it cannot weigh: a run like this is support-unverified, "
            "which says nobody knows, and NOT unsupported, which says nothing "
            "was applied")
    n = facts.get("actuators_on_the_base_free_joint")
    if n:
        return None, ("%d actuator(s) act on the base's own free joint, so a "
                      "wrench reaches it through a transmission this sampler "
                      "does not read and the total would be partial" % n)
    if int(facts.get("ntendon") or 0):
        return None, ("the scene carries %d tendon(s), another route into a "
                      "body that this sampler does not read"
                      % facts.get("ntendon"))
    return True, None


# --- the run -----------------------------------------------------------------


def run(scene_path, out_dir, *, driver_path=None, duration=300.0, settle=1.5,
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
    ground_ids = runner_t3._ground_bodies(mujoco, model, robot_ids)  # noqa: SLF001
    standing_z = (float(data.xpos[base_bid][2]) if base_bid >= 0 else None)

    rig = constraint_rig_facts(mujoco, model, base_bid)
    attested, attest_error = support_attestation(rig)
    dofadr = rig.get("base_free_joint_dofadr")

    t4doc = {
        "base_body": base, "record_stride_steps": int(record_stride),
        "contact_stride_steps": int(contact_stride),
        "physics_timestep_s": dt,
        "standing": {"z_m": standing_z, "body": base,
                     "t_s": float(data.time), "source": STANDING_SOURCE},
        "arena": runner_t3._arena(mujoco, model, data, surfaces,  # noqa: SLF001
                                  robot_ids, names),
        "base_physics": base_physics(mujoco, model, base, base_bid),
        "world": {"gravity_vec_mps2": [float(x) for x in model.opt.gravity],
                  "gravity_mps2": float(sum(float(x) ** 2
                                            for x in model.opt.gravity) ** 0.5),
                  "source": ("mjModel.opt.gravity of the model that drove THIS "
                             "run, read after the scene compiled")},
        "model": {"neq": int(model.neq), "nu": int(model.nu),
                  "ntendon": int(model.ntendon), "nmocap": int(model.nmocap),
                  "actuators_on_the_base_free_joint":
                      rig.get("actuators_on_the_base_free_joint"),
                  "why_those_are_here": "they are what makes the applied-"
                                        "wrench total complete rather than "
                                        "partial -- see the support source"},
        "constraint_rig": rig,
        "declared_efforts_nm": runner_t3._declared_efforts(out),  # noqa: SLF001
        "structurally_static_non_robot_bodies": sorted(
            names[i] for i in ground_ids if names[i]),
    }

    # -- record ------------------------------------------------------------
    t0 = float(data.time)
    times, tracks = [], [[] for _ in range(model.nbody)]
    base_xyz, base_rot = [], []
    sup_t, sup_f, sup_m = [], [], []
    xf_f, xf_m = [], []
    peak_ctrl_force = [0.0] * int(model.nu)
    gait_times, gait_contacts = [], []
    pairs = []
    total_observed = 0
    distinct_named = 0
    truncated = False
    peak_ne = 0
    eq_live_samples = 0

    def sample():
        nonlocal eq_live_samples
        times.append(round(float(data.time) - t0, 6))
        for i in range(model.nbody):
            p = data.xpos[i]
            tracks[i].append([round(float(p[0]), 6), round(float(p[1]), 6),
                              round(float(p[2]), 6)])
        if base_bid < 0:
            return
        base_xyz.append([round(float(v), 6) for v in data.xpos[base_bid]])
        base_rot.append([round(float(v), 6) for v in data.xmat[base_bid]])
        sup_t.append(round(float(data.time) - t0, 6))
        w = data.xfrc_applied[base_bid]
        af = [float(w[0]), float(w[1]), float(w[2])]
        am = [float(w[3]), float(w[4]), float(w[5])]
        ef, em = equality_reaction(mujoco, model, data, base_bid, dofadr)
        if any(abs(x) > 0.0 for x in ef + em):
            eq_live_samples += 1
        xf_f.append(af)
        xf_m.append(am)
        sup_f.append([af[i] + ef[i] for i in range(3)])
        sup_m.append([am[i] + em[i] for i in range(3)])

    sample()          # BEFORE the first step, so t=0 is the settled state

    want = float(duration)
    declared = getattr(driver, "DURATION_S", None) if driver is not None else None
    if isinstance(declared, (int, float)) and 0 < float(declared) < want:
        want = float(declared)
    max_steps = int(round(want / dt))

    for step in range(1, max_steps + 1):
        drive(float(data.time) - t0)
        mujoco.mj_step(model, data)
        peak_ne = max(peak_ne, int(data.ne))
        for a in range(int(model.nu)):
            f = abs(float(data.actuator_force[a]))
            if f > peak_ctrl_force[a]:
                peak_ctrl_force[a] = f

        emit = (step % int(contact_stride) == 0)
        if emit:
            gait_times.append(round(float(data.time) - t0, 6))
        seen = set()
        seen_gait = set()
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
                # One record per (robot body, other body) pair per query: a box
                # foot flat on a plane makes four coplanar points that name the
                # same pair, and the tier counts BODIES, not points.
                if (robot_side, other) not in seen_gait:
                    seen_gait.add((robot_side, other))
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

    identical = (eq_live_samples == 0)
    t4doc["t_s"] = times
    t4doc["base_pose"] = {
        "body": base, "xyz": base_xyz, "rot": base_rot,
        "source": POSE_SOURCE % (record_stride, dt * record_stride)}
    t4doc["gait"] = {
        "contacts": gait_contacts, "sample_times": gait_times,
        "supported": True, "steps": int(max_steps), "window_s": recorded_s,
        "total_observed": int(total_observed),
        "distinct_named": int(distinct_named),
        "emit_stride_steps": int(contact_stride),
        "records_truncated": bool(len(gait_contacts) >= CONTACT_RECORD_CAP),
        "source": CONTACT_SOURCE % int(contact_stride)}
    t4doc["support"] = {
        "attested": attested, "t": sup_t, "force": sup_f, "torque": sup_m,
        "source": SUPPORT_SOURCE, "error": attest_error,
        "equality_reaction_was_live_in_samples": int(eq_live_samples),
        "peak_equality_rows_observed": int(peak_ne),
        "xfrc_only_identical_to_total": bool(identical)}
    t4doc["support_xfrc_only"] = {
        "attested": attested,
        "t": (sup_t if not identical else []),
        "force": (xf_f if not identical else []),
        "torque": (xf_m if not identical else []),
        "identical_to_the_attested_total": bool(identical),
        "source": SUPPORT_XFRC_ONLY_SOURCE, "error": attest_error}
    t4doc["controller"] = {
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
    t4doc["actuator_peak_force"] = {
        (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or "#%d" % a):
            round(peak_ctrl_force[a], 4) for a in range(int(model.nu))}
    t4doc["driver"] = {
        "path": str(driver_path) if driver_path else None,
        "sha256": sha256(driver_path) if driver_path else None,
        "error": driver_error,
        "describe": (driver.describe()
                     if driver is not None and hasattr(driver, "describe")
                     else None)}
    (out / "t4.json").write_text(json.dumps(t4doc), encoding="utf-8")

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
                  "neq": int(model.neq), "nmocap": int(model.nmocap),
                  "ntendon": int(model.ntendon)},
        "engine": t1_runner.engine_facts(mujoco, model),
        "warnings": warnings,
        "saved_xml": str(saved_xml) if saved_xml else None,
        "scene": str(scene_path),
    }, indent=1), encoding="utf-8")

    print("recorded %.3f s over %d steps; base travelled %.3f m from its "
          "start; %d contacts observed, %d robot-to-world records over %d "
          "queries; support attested=%s, equality reaction live in %d of %d "
          "samples"
          % (recorded_s, max_steps,
             (((base_xyz[-1][0] - base_xyz[0][0]) ** 2
               + (base_xyz[-1][1] - base_xyz[0][1]) ** 2) ** 0.5
              if len(base_xyz) > 1 else 0.0),
             total_observed, len(gait_contacts), len(gait_times),
             attested, eq_live_samples, len(sup_t)))
    if control_error:
        return 8
    return 0


# --- the subprocess launcher -------------------------------------------------


class MujocoT4PhaseB:
    """One phase-B run, in the shape ``ladder.graders.t4`` expects back."""

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


def launch(scene, out_dir, *, driver=None, duration=300.0, settle=1.5,
           contact_stride=2, record_stride=RECORD_STRIDE, base=DEFAULT_BASE,
           surfaces=(), method="scripted", timeout_s=10800.0, python=None):
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


def run_standalone(deliverable, run_dir, *, backend=None, duration=300.0,
                   settle=1.5, stride=2, surfaces=(), timeout_s=10800.0,
                   base=None, record_stride=RECORD_STRIDE, method="scripted"):
    """The T4 phase-B hook. Never raises.

    ``backend`` is accepted and ignored: MuJoCo has one physics backend and no
    engine-selection field, which is exactly why this column's attribution is a
    per-run reading of ``mjOption`` rather than a sidecar.
    """
    res = MujocoT4PhaseB(run_dir)
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

    An adapter that could choose which body counts as "the base" could choose
    the body that travelled furthest. The sampler is *told* the name so it can
    record it, and it writes down the name it actually sampled; the core then
    checks that against the same task file and refuses a mismatch.
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
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--settle", type=float, default=1.5)
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


__all__ = ["ARENA_SOURCE", "CONSTRAINT_RIG_SOURCE", "CONTACT_RECORD_CAP",
           "CONTACT_SOURCE", "CONTROLLER_SOURCE", "DEFAULT_BASE",
           "DEFAULT_SURFACES", "DRIVER_NAME", "MujocoT4PhaseB", "POSE_SOURCE",
           "RECORD_STRIDE", "SCENE_NAME", "STANDING_SOURCE", "SUPPORT_SOURCE",
           "SUPPORT_XFRC_ONLY_SOURCE", "base_free_dofs", "base_physics",
           "constraint_rig_facts",
           "equality_reaction", "launch", "load_driver", "main",
           "resolve_deliverable", "run", "run_standalone", "sha256",
           "support_attestation"]


if __name__ == "__main__":
    sys.exit(main())
