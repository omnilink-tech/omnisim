#!/usr/bin/env python3
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

"""Stage-4 NORTH-STAR generator: the ONE canonical MuJoCo model for the G1 walker.

This is the Stage-4 single-model artifact of the G1 Newton walk single-source-of-
truth refactor. It emits ONE native MuJoCo (MJCF) file --
``projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf`` -- that captures the
*entire* physical model in a single file:

  * bodies / masses / inertials / joint tree         <- the prim URDF (geometry SoT)
  * collision geoms = the 2 foot boxes only          <- SPEC.FOOT_BOX_SIZE/ORIGIN
  * per-joint position + velocity limits             <- SPEC.urdf_limits / the URDF
  * actuators encoding the deploy PD (ke=100/kd=5)   <- SPEC.KE / SPEC.KD
  * per-dof armature                                 <- SPEC.ARMATURE
  * ground plane friction                            <- SPEC.GROUND_MU
  * <option timestep=...>                            <- SPEC.DT / SPEC.SUBSTEPS

Why MJCF is the north star
--------------------------
Both sides already run newton ``SolverMuJoCo`` (mjwarp): the trainer
(gpu_newton_g1_walk_trainer.py) builds a Newton articulation via
``ModelBuilder.add_urdf`` + ``joint_target_ke/kd`` and hands it to SolverMuJoCo;
the deploy (WbNewtonBackend) builds the articulation body-by-body from the
Webots node tree and hands THAT to SolverMuJoCo. In both cases SolverMuJoCo is
the thing that actually compiles a ``mujoco.MjModel`` and steps it. So MuJoCo's
own MJCF is the engines' native representation -- the natural single file to make
authoritative for geometry + inertia + gains + friction + limits + armature.

Today the two sides build that MjModel from the URDF via *two different import
routes*; this generator collapses both onto ONE on-disk MJCF. The deploy-from-
MJCF backend switch is design-complete but GATED ON A NATIVE REBUILD (the
worktree native build is blocked on this machine, see
docs/developer/g1-mjcf-single-model.md). The trainer-from-MJCF switch is a
behaviour change requiring a local GPU retrain to validate, so it is staged
AFTER the URDF-unification. This script only produces the artifact + proves it
loads in plain CPU MuJoCo; it does not flip either side.

The exact gain mapping (load-bearing)
-------------------------------------
The deploy + trainer both drive every revolute joint as a Newton
``JointTargetMode.POSITION_VELOCITY`` actuator with ``target_ke=SPEC.KE`` (=100)
and ``target_kd=SPEC.KD`` (=5). Newton's SolverMuJoCo translates ONE such
POSITION_VELOCITY dof into TWO MuJoCo `general` actuators
(newton/_src/solvers/mujoco/solver_mujoco.py, the POSITION_VELOCITY branch):

  position actuator:  gainprm=[ke, 0, ...]  biasprm=[0, -ke,  0, ...]
  velocity actuator:  gainprm=[kd, 0, ...]  biasprm=[0,   0, -kd, ...]

i.e. kp = SPEC.KE and a SEPARATE velocity damper kv = SPEC.KD, both UNBOUNDED
(forcerange and ctrlrange = [0,0]) -- SolverMuJoCo emits NO force/command clamp
for a POSITION_VELOCITY dof, so neither the trainer nor the deploy ModelBuilder
carries one (the golden test confirms P1==P2, both [0,0]). This generator emits
exactly that two-actuator-per-joint pair, unclamped, so the MJCF's joint drive
is byte-equivalent to what SolverMuJoCo builds from the Newton articulation on
both sides. (The combined single-affine actuator in
projects/policies/research/backends/_urdf_to_mjcf.py is a coarser approximation; we deliberately
do NOT use it here.)

Determinism
-----------
The build is deterministic and re-runnable: same prim URDF + same spec -> same
MJCF bytes. Run it after any change to the prim URDF or g1_physics.json.

Usage
-----
  python scripts/dev/make_g1_mjcf.py            # generate the MJCF
  python scripts/dev/make_g1_mjcf.py --verify   # generate, then CPU-validate
  python scripts/dev/make_g1_mjcf.py --check-only   # validate an existing MJCF

Run from a native shell (PowerShell) where ``mujoco`` (CPU) imports; mjwarp /
CUDA are NOT needed -- plain ``mujoco.MjModel.from_xml_path`` validates it.
"""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402

MJCF_OUT = REPO / "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf"

# Foot links carry the only collision geometry in the model (matches the prim
# URDF + the trainer's _build_g1_full_prim_builder).
_FOOT_LINKS = ("left_ankle_roll_link", "right_ankle_roll_link")


# ---------------------------------------------------------------------------
# 1. Build a mesh-free, foot-box-only URDF *string* from the prim URDF.
#    MjSpec.from_string can then parse it with NO meshdir dependency (we never
#    resolve an STL), and we know precisely which collisions survive.
#    This mirrors build_g1_native_prim._build_prim_urdf_xml so the MJCF's
#    collision set is identical to the trainer's batched model.
# ---------------------------------------------------------------------------
def _build_prim_urdf_xml() -> str:
    tree = ET.parse(str(SPEC.URDF_PRIM))
    root = tree.getroot()

    # Drop the <mujoco> compiler block (it sets meshdir; we strip all meshes so
    # it would only invite an unnecessary meshdir lookup).
    for mj in list(root.findall("mujoco")):
        root.remove(mj)

    fx, fy, fz = SPEC.FOOT_BOX_SIZE
    ox, oy, oz = SPEC.FOOT_BOX_ORIGIN
    for link in root.findall("link"):
        name = link.get("name")
        # No visuals -> no STL needed. This is a physics model, not a render one.
        for vis in list(link.findall("visual")):
            link.remove(vis)
        # Drop ALL collisions...
        for col in list(link.findall("collision")):
            link.remove(col)
        # ...then re-add the single primitive foot box on each ankle_roll link,
        # straight from the SPEC (the URDF authored the same values, but reading
        # them from SPEC keeps the spec the one knob).
        if name in _FOOT_LINKS:
            col = ET.SubElement(link, "collision")
            origin = ET.SubElement(col, "origin")
            origin.set("xyz", f"{ox} {oy} {oz}")
            origin.set("rpy", "0 0 0")
            geom = ET.SubElement(col, "geometry")
            box = ET.SubElement(geom, "box")
            box.set("size", f"{fx} {fy} {fz}")

    return ET.tostring(root, encoding="unicode")


# ---------------------------------------------------------------------------
# 2. URDF -> MjSpec, then wire in the spec-driven physics (ground/friction,
#    free joint, per-joint limits/velocity/armature, PD actuators, timestep).
# ---------------------------------------------------------------------------
def build_spec():
    import mujoco

    urdf_xml = _build_prim_urdf_xml()
    spec = mujoco.MjSpec.from_string(urdf_xml)

    # --- keep fixed-joint child links as jointless welded bodies -----------
    # The URDF has 33 links but only 24 are reached through a movable joint;
    # the other 9 (head, waist-support, logo, imu/d435/mid360 mounts) hang off
    # FIXED joints. MuJoCo's URDF importer sets ``compiler.fusestatic = 1``,
    # which fuses those jointless bodies into their parent at compile time. The
    # in-memory ``compile()`` then carries the full 34.1339 kg, BUT ``to_xml()``
    # serializes the post-fusion tree: the fused bodies vanish and each parent's
    # written <inertial> is its OWN inertia (saveinertial=0), NOT the fused
    # composite -- so the ON-DISK MJCF silently loses ~1.28 kg (torso 8.56 vs
    # 9.84, pelvis 3.813 vs 3.814). Disabling fusestatic emits all 9 fixed-joint
    # children as jointless <body> elements; MuJoCo rigidly welds a jointless
    # body to its parent on reload and folds its mass/inertia into the parent's
    # composite -- so the reloaded model carries the full 34.1339 kg and keeps
    # each link's true URDF COM. This matches the trainer's add_urdf (34 bodies)
    # and does NOT touch the revolute joint structure.
    spec.compiler.fusestatic = 0

    # --- model identity ----------------------------------------------------
    spec.modelname = "g1_23dof_omnisim"

    # --- <option timestep> -------------------------------------------------
    # The deploy + trainer step DT (0.016 s) split into SUBSTEPS (4) physics
    # sub-steps: each solver.step advances SUB_DT = DT / SUBSTEPS. The MJCF
    # timestep is therefore the SUB-STEP, not the control DT -- the consuming
    # side (trainer loop / deploy backend) is responsible for issuing SUBSTEPS
    # steps per control tick (both already do; see the trainer's substep loop
    # and OMNISIM_NEWTON_SUBSTEPS). We record SUB_DT here so a bare
    # `mujoco.mj_step` advances the same physics dt the engines use.
    sub_dt = SPEC.DT / SPEC.SUBSTEPS
    spec.option.timestep = sub_dt

    # --- ground plane ------------------------------------------------------
    has_ground = any(
        g.type == mujoco.mjtGeom.mjGEOM_PLANE for g in spec.worldbody.geoms
    )
    if not has_ground:
        ground = spec.worldbody.add_geom()
        ground.name = "ground"
        ground.type = mujoco.mjtGeom.mjGEOM_PLANE
        ground.size[0] = 50.0
        ground.size[1] = 50.0
        ground.size[2] = 0.1
        # friction = (tangential=SPEC.GROUND_MU, torsional, rolling).
        ground.friction[0] = SPEC.GROUND_MU
        ground.friction[1] = 0.005
        ground.friction[2] = 0.0001
        ground.condim = 3
        # Self-collision off: ground on group 1, robot geoms on group 2; they
        # only collide cross-group (SPEC.SELF_COLLISION is False).
        ground.contype = 1
        ground.conaffinity = 2

    # --- free joint on the root body (pelvis) ------------------------------
    root_body = spec.worldbody.first_body()
    if root_body is not None:
        has_free = any(
            j.type == mujoco.mjtJoint.mjJNT_FREE for j in root_body.joints
        )
        if not has_free:
            fj = root_body.add_freejoint()
            fj.name = "root_free"

    # --- per-joint limits + velocity + armature, and self-collision masking -
    if not SPEC.SELF_COLLISION:
        def _mask(body):
            for g in body.geoms:
                if g.type != mujoco.mjtGeom.mjGEOM_PLANE:
                    g.contype = 2
                    g.conaffinity = 1
            for child in body.bodies:
                _mask(child)
        for top in spec.worldbody.bodies:
            _mask(top)

    limits = SPEC.urdf_limits(SPEC.URDF_PRIM)
    # MjSpec.joints is a flat list of every joint in the model.
    spec_joints = {j.name: j for j in spec.joints}
    for jname, jl in limits.items():
        j = spec_joints.get(jname)
        if j is None:
            continue  # fixed joints MjSpec folds away leave no node; skip.
        if j.type == mujoco.mjtJoint.mjJNT_FREE:
            continue
        # Position range (lower==upper -> URDF "no limit" sentinel; leave open).
        if jl.lower != jl.upper:
            j.limited = True
            j.range[0] = jl.lower
            j.range[1] = jl.upper
        # Per-dof armature (SPEC.ARMATURE; 0.0 in the winning recipe).
        j.armature = SPEC.ARMATURE

    # --- PD actuators: the SolverMuJoCo POSITION_VELOCITY two-actuator pair --
    # For every actuated revolute joint emit:
    #   <position-style> gainprm=[ke,..] biasprm=[0,-ke,0,..]  (kp = SPEC.KE)
    #   <velocity-style> gainprm=[kd,..] biasprm=[0,0,-kd,..]  (kv = SPEC.KD)
    # both UNBOUNDED (forcerange/ctrlrange [0,0], PD-only). This is byte-
    # equivalent to what newton.SolverMuJoCo builds from joint_target_ke/kd in
    # POSITION_VELOCITY mode on both the trainer and deploy paths (see module
    # docstring); neither emits a force/command clamp.
    ke = float(SPEC.KE)
    kd = float(SPEC.KD)
    # Actuate exactly the 23 named joints, in the canonical SPEC order.
    actuated = list(SPEC.LEGS_JOINTS) + list(SPEC.ARM_JOINTS)
    for jname in actuated:
        j = spec_joints.get(jname)
        if j is None:
            raise RuntimeError(f"joint {jname!r} missing from MJCF -- prim URDF "
                               f"changed? regenerate.")

        # position actuator (kp spring) ------------------------------------
        a_pos = spec.add_actuator()
        a_pos.name = f"{jname}_pos"
        a_pos.target = jname
        a_pos.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a_pos.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a_pos.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a_pos.gainprm[0] = ke
        a_pos.biasprm[1] = -ke
        # biasprm[2] stays 0: velocity damping lives in the SEPARATE vel actuator
        # (this is the POSITION_VELOCITY split, not POSITION's combined form).
        # NO forcerange / ctrlrange: newton.SolverMuJoCo leaves BOTH at [0,0]
        # (unbounded, PD-only) for a POSITION_VELOCITY dof on BOTH the trainer
        # and the deploy ModelBuilder paths -- the golden test shows P1==P2,
        # both [0,0]. Emitting the URDF effort/position clamps here was the only
        # thing that made the MJCF (P3) diverge on actuator_forcerange/ctrlrange.
        # forcelimited / ctrllimited default False -> compiled ranges stay [0,0].
        a_pos.forcelimited = mujoco.mjtLimited.mjLIMITED_FALSE
        a_pos.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE

        # velocity actuator (kd damper) ------------------------------------
        a_vel = spec.add_actuator()
        a_vel.name = f"{jname}_vel"
        a_vel.target = jname
        a_vel.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a_vel.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a_vel.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a_vel.gainprm[0] = kd
        a_vel.biasprm[2] = -kd
        # Same as the position actuator: no force/ctrl clamp (PD-only, [0,0]).
        a_vel.forcelimited = mujoco.mjtLimited.mjLIMITED_FALSE
        a_vel.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE

    return spec, actuated


def generate() -> int:
    import mujoco

    spec, actuated = build_spec()
    model = spec.compile()
    print(f"[make_g1_mjcf] compile OK: nq={model.nq} nv={model.nv} "
          f"nu={model.nu} njnt={model.njnt} nbody={model.nbody} "
          f"ngeom={model.ngeom}")

    MJCF_OUT.parent.mkdir(parents=True, exist_ok=True)
    xml = spec.to_xml()
    MJCF_OUT.write_text(xml, encoding="utf-8")
    print(f"[make_g1_mjcf] wrote {MJCF_OUT.relative_to(REPO)} "
          f"({MJCF_OUT.stat().st_size} bytes)")

    # Round-trip: the bytes we just wrote must reload.
    m2 = mujoco.MjModel.from_xml_path(str(MJCF_OUT))
    print(f"[make_g1_mjcf] reload OK: nq={m2.nq} nv={m2.nv} nu={m2.nu}")
    return 0


# ---------------------------------------------------------------------------
# 3. CPU validation (DELIVERABLE 2). Loads the on-disk MJCF and asserts the
#    DOF counts, joint ranges, actuator gains, and ground friction all match
#    the SPEC. Runnable anywhere plain `mujoco` (CPU) is installed.
# ---------------------------------------------------------------------------
def verify() -> int:
    import mujoco
    import numpy as np

    if not MJCF_OUT.exists():
        print(f"[verify] ERROR: {MJCF_OUT} missing -- run the generator first.",
              file=sys.stderr)
        return 1

    print(f"[verify] loading {MJCF_OUT.relative_to(REPO)}")
    m = mujoco.MjModel.from_xml_path(str(MJCF_OUT))
    fails: list[str] = []

    def check(cond: bool, msg: str):
        status = "PASS" if cond else "FAIL"
        print(f"[verify] [{status}] {msg}")
        if not cond:
            fails.append(msg)

    # --- DOF counts --------------------------------------------------------
    # 23 hinge dofs + 1 free joint (6 dof / 7 qpos). nq = 23 + 7 = 30,
    # nv = 23 + 6 = 29, njnt = 23 + 1 = 24.
    check(m.nq == 30, f"nq == 30 (free 7 + 23 hinge qpos) -> got {m.nq}")
    check(m.nv == 29, f"nv == 29 (free 6 + 23 hinge dof)  -> got {m.nv}")
    check(m.njnt == 24, f"njnt == 24 (1 free + 23 hinge)   -> got {m.njnt}")
    # nu = 23 joints x 2 actuators (position + velocity) = 46.
    check(m.nu == 46, f"nu == 46 (23 joints x [pos,vel])  -> got {m.nu}")

    # --- joint ranges match SPEC limits -----------------------------------
    limits = SPEC.urdf_limits(SPEC.URDF_PRIM)
    actuated = list(SPEC.LEGS_JOINTS) + list(SPEC.ARM_JOINTS)
    range_ok = True
    armature_ok = True
    for jname in actuated:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            range_ok = False
            print(f"[verify]   joint {jname} NOT FOUND")
            continue
        jl = limits[jname]
        lo, hi = float(m.jnt_range[jid][0]), float(m.jnt_range[jid][1])
        if abs(lo - jl.lower) > 1e-4 or abs(hi - jl.upper) > 1e-4:
            range_ok = False
            print(f"[verify]   {jname} range ({lo:.4f},{hi:.4f}) != "
                  f"URDF ({jl.lower:.4f},{jl.upper:.4f})")
        # dof armature
        dofadr = int(m.jnt_dofadr[jid])
        arm = float(m.dof_armature[dofadr])
        if abs(arm - SPEC.ARMATURE) > 1e-6:
            armature_ok = False
            print(f"[verify]   {jname} armature {arm} != SPEC {SPEC.ARMATURE}")
    check(range_ok, "all 23 joint ranges match URDF/SPEC limits")
    check(armature_ok, f"all 23 dof armature == SPEC.ARMATURE ({SPEC.ARMATURE})")

    # --- actuator gains reflect SPEC.KE / SPEC.KD -------------------------
    gains_ok = True
    n_pos = n_vel = 0
    for aid in range(m.nu):
        aname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
        gp0 = float(m.actuator_gainprm[aid][0])
        bp1 = float(m.actuator_biasprm[aid][1])
        bp2 = float(m.actuator_biasprm[aid][2])
        if aname.endswith("_pos"):
            n_pos += 1
            # kp = ke: gainprm[0]==ke, biasprm[1]==-ke, biasprm[2]==0
            if (abs(gp0 - SPEC.KE) > 1e-4 or abs(bp1 + SPEC.KE) > 1e-4
                    or abs(bp2) > 1e-9):
                gains_ok = False
                print(f"[verify]   {aname} gain ({gp0},{bp1},{bp2}) "
                      f"!= position[ke={SPEC.KE}]")
        elif aname.endswith("_vel"):
            n_vel += 1
            # kv = kd: gainprm[0]==kd, biasprm[2]==-kd, biasprm[1]==0
            if (abs(gp0 - SPEC.KD) > 1e-4 or abs(bp2 + SPEC.KD) > 1e-4
                    or abs(bp1) > 1e-9):
                gains_ok = False
                print(f"[verify]   {aname} gain ({gp0},{bp1},{bp2}) "
                      f"!= velocity[kd={SPEC.KD}]")
    check(gains_ok, f"actuator gains == SPEC (kp={SPEC.KE}, kv={SPEC.KD})")
    check(n_pos == 23 and n_vel == 23,
          f"23 position + 23 velocity actuators -> pos={n_pos} vel={n_vel}")

    # --- ground friction[0] == SPEC.GROUND_MU -----------------------------
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    if gid < 0:
        # fall back: find the plane geom
        plane_ids = [g for g in range(m.ngeom)
                     if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE]
        gid = plane_ids[0] if plane_ids else -1
    if gid >= 0:
        mu = float(m.geom_friction[gid][0])
        check(abs(mu - SPEC.GROUND_MU) < 1e-6,
              f"ground geom friction[0] == SPEC.GROUND_MU ({SPEC.GROUND_MU}) "
              f"-> got {mu}")
    else:
        check(False, "ground plane geom present")

    # --- timestep == SUB_DT ------------------------------------------------
    sub_dt = SPEC.DT / SPEC.SUBSTEPS
    check(abs(float(m.opt.timestep) - sub_dt) < 1e-9,
          f"option.timestep == SPEC.DT/SUBSTEPS ({sub_dt})")

    # --- it actually steps (no NaN on one sub-step from the seed pose) -----
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    check(bool(np.isfinite(d.qpos).all()) and bool(np.isfinite(d.qvel).all()),
          "single mj_step produces finite state")

    print()
    if fails:
        print(f"[verify] RESULT: FAIL ({len(fails)} check(s) failed)")
        for f in fails:
            print(f"[verify]   - {f}")
        return 1
    print("[verify] RESULT: PASS -- canonical MJCF matches the SPEC.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--verify", action="store_true",
                    help="generate, then CPU-validate against the SPEC")
    ap.add_argument("--check-only", action="store_true",
                    help="validate an existing MJCF without regenerating")
    args = ap.parse_args()

    try:
        import mujoco  # noqa: F401
    except Exception as e:  # pragma: no cover
        print(f"[make_g1_mjcf] mujoco import failed: {e}\n"
              f"  Generation needs the MjSpec API (mujoco>=3). Install where "
              f"mujoco (CPU) is available; mjwarp/CUDA are NOT required.",
              file=sys.stderr)
        return 2

    if args.check_only:
        return verify()

    rc = generate()
    if rc != 0:
        return rc
    if args.verify:
        return verify()
    return 0


if __name__ == "__main__":
    sys.exit(main())
