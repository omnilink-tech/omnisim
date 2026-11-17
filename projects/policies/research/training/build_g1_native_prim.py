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

"""PRIMITIVE-COLLISION variant of the deploy-faithful native G1 (build_g1_native).

WHY: build_g1_native imports g1_legs_omnisim.urdf with add_urdf(floating=True),
which carries the URDF's `<collision>` MESH geometry (detailed STLs on pelvis,
hips, knees, ankle_pitch, torso, ...). newton's add_urdf loads `<collision>`
geoms as physics shapes (parse_shapes -> add_shape_mesh; see
newton/_src/utils/import_urdf.py:473-509), and replicating those meshes to many
worlds overflows the SolverMuJoCo collision broad-phase
("Triangle pair buffer overflowed 8,771,072 > 1,000,000" at 256 worlds) and
bloats the build. So the deploy-faithful native model is correct for ONE robot
but cannot be batched for on-engine RL.

This module builds a model that is DYNAMICS-IDENTICAL to build_g1_native --
same link inertias/masses (URDF `<inertial>` is parsed independently of any
shape; import_urdf overwrites body_mass/body_inertia from `<inertial>` whenever
ignore_inertial_definitions=False, regardless of collision geometry), same 13
native revolute joints, same ke/kd/effort, same NOMINAL13 seed -- but with
PRIMITIVE collision only:

  * We strip every `<collision>` MESH element from the URDF in-memory and keep
    only PRIMITIVE foot colliders. The g1_legs_omnisim.urdf already authors the
    feet (left/right_ankle_roll_link) as `<box size="0.17 0.06 0.012"/>` at
    xyz="0.035 0 -0.030" -- a real primitive, not a mesh -- so we keep those by
    default (and can override their size via env to the task's nominal foot).
  * We also strip `<visual>` (training model, no render) so NO STL needs to be
    resolved -> the modified URDF can be fed to add_urdf as an XML STRING with
    no meshdir dependency.
  * Add a ground plane.

So the only difference vs build_g1_native is the collision set: detailed STL
mesh colliders -> two primitive foot boxes. Inertias/masses/joints/control are
byte-identical because they come from the same `<inertial>`/`<joint>` data.

build_g1_prim_builder() returns the configured (un-finalized) ModelBuilder so it
can be ModelBuilder.replicate()'d for batched training; build_g1_native_prim()
finalizes one robot for the single-world self-test (mirrors build_g1_native).

Run from a native Windows shell (PowerShell) so warp/CUDA initialises
(reference_verify_newton_powershell). add_urdf collision-loading verified against
newton 1.2.0 / mujoco_warp 3.8.0.3.
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import warp as wp
import newton

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(_REPO))

# Reuse the deploy-faithful constants (URDF path, squat seed, gains, spawn,
# DOF layout) verbatim so the prim model stays dynamics-identical.
from projects.policies.research.training.build_g1_native import (  # noqa: E402
    URDF, NOMINAL13, KE, KD, EFFORT, SPAWN_Z, FREE_DOF, NJ, DOF0, QPOS0)
# SINGLE SOURCE OF TRUTH for the DEFAULT foot collision box, so this prebaked
# prim builder, the trainer's in-memory strip, and the deploy can never drift.
from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402


# --- foot primitive geometry --------------------------------------------------
# Default = KEEP the URDF's authored foot boxes (sole ~0.012 m thick, 0.17x0.06,
# centred 0.030 m below the ankle_roll origin), which already match the real G1
# foot. OMNISIM_G1PRIM_FOOT=task swaps to the task's nominal foot
# (~0.20 x 0.10 x 0.03 m, sole 0.03 m below origin). Either way the foot is a
# single primitive box, so the broad-phase stays tiny under replication.
_FOOT_LINKS = ("left_ankle_roll_link", "right_ankle_roll_link")


def _task_foot_box():
    # half-extents + centre offset for a ~0.20 x 0.10 x 0.03 m sole whose bottom
    # face sits ~0.03 m below the ankle_roll body origin.
    sx, sy, sz = 0.20, 0.10, 0.03
    # centre 0.035 m forward (toe-biased like the URDF), sole bottom at -0.03 m
    # => centre_z = -0.03 + sz/2 = -0.015.
    return (sx, sy, sz), (0.035, 0.0, -0.015)


def _build_prim_urdf_xml() -> str:
    """Return a URDF XML string identical to g1_legs_omnisim.urdf except:
      * every `<visual>` removed (no render in a training model),
      * every `<collision>` removed EXCEPT a primitive box on each foot link,
      * foot boxes optionally resized to the task nominal foot.
    Inertials, joints, links are untouched -> dynamics-identical."""
    tree = ET.parse(str(URDF))
    root = tree.getroot()

    use_task_foot = os.environ.get("OMNISIM_G1PRIM_FOOT", "").lower() == "task"

    for link in root.findall("link"):
        name = link.get("name")
        # drop all visuals
        for vis in list(link.findall("visual")):
            link.remove(vis)
        # drop all collisions...
        for col in list(link.findall("collision")):
            link.remove(col)
        # ...then re-add a PRIMITIVE foot box on the two ankle_roll links.
        if name in _FOOT_LINKS:
            col = ET.SubElement(link, "collision")
            origin = ET.SubElement(col, "origin")
            geom = ET.SubElement(col, "geometry")
            box = ET.SubElement(geom, "box")
            if use_task_foot:
                (sx, sy, sz), (ox, oy, oz) = _task_foot_box()
            else:
                # the URDF's own authored foot box, sourced from the SoT
                # (was hardcoded 0.17 0.06 0.012 @ 0.035 0 -0.030; UNCHANGED).
                sx, sy, sz = (float(c) for c in SPEC.FOOT_BOX_SIZE)
                ox, oy, oz = (float(c) for c in SPEC.FOOT_BOX_ORIGIN)
            origin.set("xyz", f"{ox} {oy} {oz}")
            origin.set("rpy", "0 0 0")
            box.set("size", f"{sx} {sy} {sz}")

    return ET.tostring(root, encoding="unicode")


def _configure_actuators(mb: newton.ModelBuilder, ke: float, kd: float,
                         effort, seed: bool) -> None:
    """Configure the 13 actuated revolute DOFs (6..18) as native
    POSITION_VELOCITY actuators matching the deploy -- identical to
    build_g1_native (same ke/kd/effort, same NOMINAL13 seed)."""
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    for d in range(DOF0, DOF0 + NJ):
        mb.joint_target_ke[d] = ke
        mb.joint_target_kd[d] = kd
        mb.joint_target_mode[d] = pv
        if effort is not None:
            mb.joint_effort_limit[d] = effort
        mb.joint_target_q[d] = float(NOMINAL13[d - DOF0])
    if seed:
        for i in range(NJ):
            mb.joint_q[QPOS0 + i] = float(NOMINAL13[i])


def build_g1_prim_builder(mu: float = 1.0, ke: float = KE, kd: float = KD,
                          effort=EFFORT, add_ground: bool = True
                          ) -> newton.ModelBuilder:
    """Build ONE prim-collision G1 into a fresh ModelBuilder and return the
    BUILDER (un-finalized) so callers can ModelBuilder.replicate() it for
    batched training. Set add_ground=False when you will replicate (add the
    ground once on the main builder instead) -- or leave True to get a
    per-world ground like build_g1_native / probe_newton_batch_throughput.

    Same args/env-tunability as build_g1_native: effort=None keeps the URDF's
    per-joint effort limits; G1NATIVE_NOSEED skips the squat seed."""
    mb = newton.ModelBuilder()
    urdf_xml = _build_prim_urdf_xml()
    mb.add_urdf(urdf_xml,
                xform=wp.transform((0.0, 0.0, SPAWN_Z), (0.0, 0.0, 0.0, 1.0)),
                floating=True)
    seed = not os.environ.get("G1NATIVE_NOSEED")
    _configure_actuators(mb, ke, kd, effort, seed)
    if add_ground:
        mb.add_ground_plane()
    return mb


def build_g1_native_prim(mu: float = 1.0, ke: float = KE, kd: float = KD,
                         effort=EFFORT):
    """Finalized single-robot prim model (mirrors build_g1_native()'s return)."""
    return build_g1_prim_builder(mu=mu, ke=ke, kd=kd, effort=effort,
                                 add_ground=True).finalize()


def _selftest(seconds: float = 2.0, substeps: int = 4) -> int:
    """Mirror build_g1_native._selftest but with collision ON (primitive feet):
    hold NOMINAL under the EXACT deploy solver and confirm finite + no explosion
    + the feet actually contact the ground. PASS is NOT 'stands forever' -- a
    static NOMINAL squat is an unstable inverted pendulum; we verify contact
    works (pelvis settles/holds briefly then the expected passive-biped tip
    ~1.5 s) with no NaN."""
    wp.init()
    model = build_g1_native_prim()
    n_shapes = len(model.shape_type.numpy()) if hasattr(model, "shape_type") else "?"
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False)
    state_a, state_b = model.state(), model.state()
    control = model.control()
    contacts = model.contacts() if hasattr(model, "contacts") else None
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)

    tp = control.joint_target_q.numpy()
    tp[DOF0:DOF0 + NJ] = NOMINAL13
    control.joint_target_q.assign(tp)

    dt = 0.016
    sub_dt = dt / substeps
    n = int(seconds / dt)
    z0 = float(state_a.body_q.numpy()[0][2])
    print(f"[g1_prim] start pelvis_z={z0:.3f}  steps={n} substeps={substeps} "
          f"shapes={n_shapes} collide={'ON' if contacts is not None else 'OFF'}")
    # Track the pelvis height in an EARLY window (before the passive biped tips):
    # with the feet contacting the ground the pelvis is HELD near its squat
    # height for the first ~0.6 s. WITHOUT contact it free-falls -- a free body
    # released at 0.78 m drops ~0.2 m in 0.2 s and is below the floor by ~0.4 s.
    # So "held well above free-fall through the early window" == contact engaged.
    z_at = {}                       # t (s, rounded) -> pelvis_z
    for k in range(n):
        for _ in range(substeps):
            state_a.clear_forces()
            if contacts is not None:
                model.collide(state_a, contacts)
                solver.step(state_a, state_b, control, contacts, sub_dt)
            else:
                solver.step(state_a, state_b, control, None, sub_dt)
            state_a, state_b = state_b, state_a
        t = k * dt
        if k % 12 == 0 or k == n - 1:
            bq = state_a.body_q.numpy()[0]
            z_at[round(t, 2)] = float(bq[2])
            print(f"[g1_prim] t={t:4.2f}s pelvis=({bq[0]:+.3f},{bq[1]:+.3f},{bq[2]:.3f})")
    bq_all = state_a.body_q.numpy()
    zf = float(bq_all[0][2])
    finite = bool(np.isfinite(bq_all).all())
    no_explosion = abs(zf) < 5.0
    # Contact gate: pelvis still high (> 0.6 m, vs free-fall < ~0.6 m) at t~0.6 s.
    z_early = max((v for t, v in z_at.items() if 0.4 <= t <= 0.8), default=z0)
    contact_worked = z_early > 0.6
    ok = finite and no_explosion and contact_worked
    print(f"[g1_prim] RESULT final_z={zf:.3f} z_early(0.4-0.8s)={z_early:.3f} "
          f"finite={finite} contact_worked={contact_worked} "
          f"{'PASS (prim feet contact ground, no NaN)' if ok else 'FAIL'}")
    print("[g1_prim] NOTE: passive NOMINAL-hold tips ~1.5 s (expected for a "
          "biped); the point is foot contact + finite, not indefinite standing.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(_selftest())
