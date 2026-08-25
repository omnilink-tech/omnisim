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

"""Foot-friction probe for the Newton/MuJoCo quadruped contact.

Question under test: is OmniQuad's lateral roll-tip (observed in Newton training:
height + pitch hold, but roll diverges to ~0.9 rad around step 60-90) caused
by the FEET SLIPPING SIDEWAYS under SolverMuJoCo's contact model, or is the
contact solid and the roll instability is purely a control problem?

This is pure newton + warp (no Webots). It reuses the quadruped from
newton_mujoco_repro.py (free root + 4 identical 3-revolute legs, sphere feet),
holds the standing pose with POSITION_VELOCITY motors, and:

  1. Introspects the ACTUAL MuJoCo model the solver built -- per-geom friction
     tuple [tangential, torsional, rolling] and condim. Newton's ShapeConfig.mu
     is a scalar; MuJoCo needs the full tuple. If torsional/rolling are ~0 the
     foot can pivot/roll even at mu=1.0.
  2. Stands undisturbed for N steps, tracking chassis roll + each foot's lateral
     (x,y) world position. Planted feet => |drift| ~ 0.
  3. Applies a lateral velocity kick to the chassis (sideways shove) and measures
     whether the feet slide (slip) and whether the body rolls / tips or recovers.

Sweep MU over a few values to see sensitivity: if higher friction does NOT
reduce foot drift / roll, friction is exonerated and the lever is control.

Run:
    python projects/policies/research/tools/newton_friction_probe.py
    REPRO_MU=2.0 python projects/policies/research/tools/newton_friction_probe.py   # single mu
"""
from __future__ import annotations

import os
import numpy as np
import warp as wp
import newton

KE = 250.0
KD = 25.0
EFFORT = 80.0
HIP_X = 0.3
HIP_Y = 0.3
KNEE = -0.6


def build(mu: float):
    b = newton.ModelBuilder()
    b.default_shape_cfg.mu = mu
    chassis = b.add_link(
        xform=wp.transform((0.0, 0.0, 0.7), (0.0, 0.0, 0.0, 1.0)),
        mass=10.0, inertia=wp.mat33((0.2, 0, 0), (0, 1.1, 0), (0, 0, 1.2)),
        label="chassis",
    )
    joint_indices = [b.add_joint_free(child=chassis)]
    slot_targets = {}
    foot_links = []
    leg_signs = [(+0.298, +0.055), (+0.298, -0.055), (-0.298, +0.055), (-0.298, -0.055)]

    def add_motor(parent, child, axis, p_anchor, lo, hi):
        idx = b.add_joint_revolute(
            parent=parent, child=child, axis=axis,
            parent_xform=wp.transform(p_anchor, (0.0, 0.0, 0.0, 1.0)),
            child_xform=wp.transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            target_pos=0.0, target_vel=0.0, target_ke=KE, target_kd=KD,
            armature=0.0, limit_ke=10000.0, limit_kd=100.0,
            limit_lower=lo, limit_upper=hi, effort_limit=EFFORT,
            actuator_mode=newton.JointTargetMode.POSITION_VELOCITY,
            collision_filter_parent=True,
        )
        joint_indices.append(idx)
        return idx

    for (ax, ay) in leg_signs:
        hip = b.add_link(xform=wp.transform((ax, ay, 0.7), (0, 0, 0, 1.0)), mass=1.0,
                         inertia=wp.mat33((0.01, 0, 0), (0, 0.01, 0), (0, 0, 0.01)), label="hip")
        thigh = b.add_link(xform=wp.transform((ax, ay, 0.72), (0, 0, 0, 1.0)), mass=1.0,
                           inertia=wp.mat33((0.02, 0, 0), (0, 0.02, 0), (0, 0, 0.02)), label="thigh")
        lower = b.add_link(xform=wp.transform((ax, ay, 0.40), (0, 0, 0, 1.0)), mass=0.5,
                           inertia=wp.mat33((0.01, 0, 0), (0, 0.01, 0), (0, 0, 0.01)), label="lower")
        b.add_shape_sphere(lower, xform=wp.transform((0.025, 0.0, -0.321), (0, 0, 0, 1.0)),
                           radius=0.05, cfg=newton.ModelBuilder.ShapeConfig(density=0.0, mu=mu))
        foot_links.append(lower)
        lo_x = 0.001 if ay > 0 else -0.6
        hi_x = 0.6 if ay > 0 else -0.001
        j_hipx = add_motor(chassis, hip, (1.0, 0.0, 0.0), (ax, ay, 0.0), lo_x, hi_x)
        j_hipy = add_motor(hip, thigh, (0.0, 1.0, 0.0), (0.0, 0.111 if ay > 0 else -0.111, 0.0), 0.001, 0.6)
        j_knee = add_motor(thigh, lower, (0.0, 1.0, 0.0), (0.025, 0.0, -0.321), -1.2, -0.01)
        slot_targets[j_hipx] = HIP_X if ay > 0 else -HIP_X
        slot_targets[j_hipy] = HIP_Y
        slot_targets[j_knee] = KNEE

    b.add_ground_plane()
    b.add_articulation(joint_indices, label="quadruped")
    model = b.finalize()
    return model, slot_targets, chassis, foot_links


def dump_mjc_friction(solver):
    """Read the friction tuple + condim out of the MuJoCo model the solver
    built. Attribute names differ across newton versions; probe a few."""
    mjm = None
    for attr in ("mj_model", "mjw_model", "mujoco_model", "_mj_model", "model_mjc"):
        mjm = getattr(solver, attr, None)
        if mjm is not None:
            break
    if mjm is None:
        # SolverMuJoCo often stores a wrapper; try .mjw_model.to_mjmodel() etc.
        wrap = getattr(solver, "mjw_model", None) or getattr(solver, "mj", None)
        mjm = getattr(wrap, "mj_model", None) if wrap is not None else None
    if mjm is None:
        print("  [mjc] could not locate MuJoCo model on solver; attrs:",
              [a for a in dir(solver) if not a.startswith("__")][:40])
        return
    try:
        gf = np.asarray(mjm.geom_friction)
        cd = np.asarray(mjm.geom_condim) if hasattr(mjm, "geom_condim") else None
        print(f"  [mjc] ngeom={gf.shape[0]} geom_friction[tangent,torsion,roll] rows:")
        for i, row in enumerate(gf):
            cdim = int(cd[i]) if cd is not None else "?"
            print(f"        geom{i}: friction={np.round(row,3).tolist()} condim={cdim}")
    except Exception as e:
        print("  [mjc] introspect failed:", e)


def step_world(model, solver, control, target_by_qdidx, contacts, a, b, dt=0.016):
    tpa = control.joint_target_q.numpy()
    for di, tgt in target_by_qdidx.items():
        if di < len(tpa):
            tpa[di] = tgt
    control.joint_target_q.assign(tpa)
    a.clear_forces()
    if contacts is not None:
        model.collide(a, contacts)
        solver.step(a, b, control, contacts, dt)
    else:
        solver.step(a, b, control, None, dt)
    return b, a  # swapped


def run_for_mu(mu: float):
    print(f"\n========== MU = {mu} ==========")
    model, slot_targets, chassis, foot_links = build(mu)
    q_start = model.joint_q_start.numpy()
    qd_start = model.joint_qd_start.numpy()
    jq = model.joint_q.numpy()
    for j, tgt in slot_targets.items():
        jq[int(q_start[j])] = tgt
    model.joint_q.assign(jq)

    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=True)
    dump_mjc_friction(solver)

    a = model.state(); b = model.state()
    a.joint_q.assign(jq)
    newton.eval_fk(model, model.joint_q, model.joint_qd, a)
    control = model.control()
    target_by_qdidx = {int(qd_start[j]): tgt for j, tgt in slot_targets.items()}
    contacts = model.contacts() if hasattr(model, "contacts") else None

    def foot_xy():
        bq = a.body_q.numpy()
        return np.array([[bq[fl, 0], bq[fl, 1]] for fl in foot_links])

    def chassis_rpy_z():
        bq = a.body_q.numpy()
        qx, qy, qz, qw = bq[chassis, 3:7]
        # roll (x), pitch (y)
        roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1, 1))
        return float(roll), float(pitch), float(bq[chassis, 2])

    # ---- Phase 1: settle + undisturbed stand ----
    foot0 = None
    for step in range(1, 121):
        a, b = step_world(model, solver, control, target_by_qdidx, contacts, a, b)
        if step == 30:
            foot0 = foot_xy()  # reference after settle
        if step in (30, 60, 120):
            roll, pitch, cz = chassis_rpy_z()
            drift = float(np.max(np.linalg.norm(foot_xy() - foot0, axis=1))) if foot0 is not None else 0.0
            print(f"  [stand] step {step:3d}: roll={roll:+.3f} pitch={pitch:+.3f} "
                  f"cz={cz:.3f} max_foot_drift={drift:.4f}")

    # ---- Phase 2: lateral kick (sideways shove on chassis) ----
    foot_pre = foot_xy()
    qd = a.joint_qd.numpy()
    # free root qd layout: [wx,wy,wz, vx,vy,vz] (angular first in newton)
    qd[4] += 0.6  # +0.6 m/s lateral (y) velocity kick on the chassis
    a.joint_qd.assign(qd)
    newton.eval_fk(model, a.joint_q, a.joint_qd, a)
    print("  [kick] +0.6 m/s lateral velocity applied to chassis")
    tipped = False
    for step in range(1, 121):
        a, b = step_world(model, solver, control, target_by_qdidx, contacts, a, b)
        if step in (10, 30, 60, 120):
            roll, pitch, cz = chassis_rpy_z()
            drift = float(np.max(np.linalg.norm(foot_xy() - foot_pre, axis=1)))
            tag = ""
            if abs(roll) > 0.5 and not tipped:
                tag = "  <-- TIPPING (|roll|>0.5)"; tipped = True
            print(f"  [kick ] step {step:3d}: roll={roll:+.3f} pitch={pitch:+.3f} "
                  f"cz={cz:.3f} max_foot_slip={drift:.4f}{tag}")
    roll, _, cz = chassis_rpy_z()
    verdict = "RECOVERED" if abs(roll) < 0.2 and cz > 0.45 else (
              "TIPPED" if abs(roll) > 0.5 or cz < 0.3 else "MARGINAL")
    print(f"  [verdict mu={mu}] final roll={roll:+.3f} cz={cz:.3f} -> {verdict}")


def main():
    wp.init()
    print(f"newton {newton.__version__}  warp {wp.__version__}  solver=SolverMuJoCo(cpu)")
    single = os.environ.get("REPRO_MU")
    mus = [float(single)] if single else [0.5, 1.0, 2.0]
    for mu in mus:
        run_for_mu(mu)


if __name__ == "__main__":
    main()
