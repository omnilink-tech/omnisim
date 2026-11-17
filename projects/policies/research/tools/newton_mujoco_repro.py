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

"""Minimal standalone reproducer for a SolverMuJoCo per-joint drift bug.

No Webots / OmniSim involved -- pure newton + warp. Builds a quadruped-like
articulation (free-floating root + 4 identical 3-revolute legs), all joints
configured IDENTICALLY as POSITION_VELOCITY motors, seeds every joint to a
nonzero hold target, then steps SolverMuJoCo with a constant position target
equal to the seed. Expectation: every joint holds its target. Observed
(newton 1.2.0 / warp 1.13.0, use_mujoco_cpu=True): one joint -- the first
knee in articulation order -- drifts away from its target while the three
geometrically/identically configured siblings hold.

Run:
    python projects/policies/research/tools/newton_mujoco_repro.py
"""
from __future__ import annotations

import numpy as np
import warp as wp
import newton

KE = 250.0
KD = 25.0
EFFORT = 80.0
# nominal hold targets per joint kind
HIP_X = 0.3
HIP_Y = 0.3
KNEE = -0.6


def build():
    b = newton.ModelBuilder()
    # Root chassis (free-floating).
    chassis = b.add_link(
        xform=wp.transform((0.0, 0.0, 0.7), (0.0, 0.0, 0.0, 1.0)),
        mass=10.0, inertia=wp.mat33((0.2, 0, 0), (0, 1.1, 0), (0, 0, 1.2)),
        label="chassis",
    )
    joint_indices = [b.add_joint_free(child=chassis)]
    slot_targets = {}   # builder-joint-index -> target angle
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
                           radius=0.05, cfg=newton.ModelBuilder.ShapeConfig(density=0.0, mu=1.0))
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
    import os
    if os.environ.get("REPRO_NOGRAV"):
        try:
            g = model.gravity.numpy()
            g[:] = 0.0
            model.gravity.assign(g)
        except Exception as e:
            print("gravity-off failed:", e)
    return model, slot_targets


def main():
    wp.init()
    model, slot_targets = build()
    q_start = model.joint_q_start.numpy()
    # Map builder joint index -> target. After finalize the joint order is the
    # articulation order; joint_q_start[j] is the q index for joint j.
    jq = model.joint_q.numpy()
    target_by_qidx = {}
    for j, tgt in slot_targets.items():
        qi = int(q_start[j])
        jq[qi] = tgt
        target_by_qidx[qi] = tgt
    model.joint_q.assign(jq)

    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=True)
    state_a = model.state()
    state_b = model.state()
    # seed state joint_q too (MuJoCo reads per-step state)
    state_a.joint_q.assign(jq)
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)

    control = model.control()
    # control.joint_target_pos is indexed by DOF (qd), NOT q. Build a
    # qd-indexed target map (the kernel reads joint_target_pos[qd_idx]).
    qd_start = model.joint_qd_start.numpy()
    target_by_qdidx = {}
    for j, tgt in slot_targets.items():
        target_by_qdidx[int(qd_start[j])] = tgt
    tp = control.joint_target_q.numpy()
    for di, tgt in target_by_qdidx.items():
        if di < len(tp):
            tp[di] = tgt
    control.joint_target_q.assign(tp)

    print(f"newton {newton.__version__}  warp {wp.__version__}  solver=SolverMuJoCo(cpu)")
    print("targets by q-index:", {k: round(v, 2) for k, v in sorted(target_by_qidx.items())})
    contacts = model.contacts() if hasattr(model, "contacts") else None

    def reapply_control():
        a = control.joint_target_q.numpy()
        for di, tgt in target_by_qdidx.items():
            if di < len(a):
                a[di] = tgt
        control.joint_target_q.assign(a)

    # Two-"episode" test: simulate, RESET (mirroring OmNewtonBackend's
    # reset_joints_to_defaults: re-seed standing joint_q to model + both
    # states, eval_fk, rebuild the MuJoCo solver), then simulate again.
    # If episode 2 collapses (chassis z drops) while episode 1 holds, we
    # have reproduced the training episode-2 reset bug standalone.
    nonlocal_state = {"a": state_a, "b": state_b, "solver": solver}

    def run_episode(label, n=40):
        a = nonlocal_state["a"]
        b = nonlocal_state["b"]
        sv = nonlocal_state["solver"]
        for step in range(1, n + 1):
            # re-apply control each step
            tpa = control.joint_target_q.numpy()
            for di, tgt in target_by_qdidx.items():
                if di < len(tpa):
                    tpa[di] = tgt
            control.joint_target_q.assign(tpa)
            a.clear_forces()
            if contacts is not None:
                model.collide(a, contacts)
                sv.step(a, b, control, contacts, 0.016)
            else:
                sv.step(a, b, control, None, 0.016)
            a, b = b, a
            if step % 8 == 0 or step == 1:
                bz = float(a.body_q.numpy()[0, 2])  # chassis z
                kq = a.joint_q.numpy()
                kmax = max(abs(float(kq[qi]) - tgt) for qi, tgt in target_by_qidx.items())
                print(f"  [{label}] step {step:2d}: chassis_z={bz:.3f}  max|q-tgt|={kmax:.3f}")
        nonlocal_state["a"] = a
        nonlocal_state["b"] = b

    def do_reset():
        # Mirror OmNewtonBackend.reset_joints_to_defaults: restore standing
        # joint_q to model + both states, eval_fk, rebuild MuJoCo solver.
        a = nonlocal_state["a"]
        b = nonlocal_state["b"]
        model.joint_q.assign(jq)
        zqd = model.joint_qd.numpy() * 0.0
        for st in (a, b):
            st.joint_q.assign(jq)
            st.joint_qd.assign(zqd)
        newton.eval_fk(model, model.joint_q, model.joint_qd, a)
        nonlocal_state["solver"] = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=True)
        print("  [reset] re-seeded standing joint_q + rebuilt solver")

    import os as _os
    cycles = int(_os.environ.get("REPRO_CYCLES", "1"))
    if cycles <= 1:
        print("=== EPISODE 1 (fresh seed + solver) ===")
        run_episode("ep1")
        print("=== RESET ===")
        do_reset()
        print("=== EPISODE 2 (after reset) ===")
        run_episode("ep2")
    else:
        # Many reset cycles: does repeated reset+solver-rebuild degrade the
        # stand over episodes (training does ~1 reset per episode)?
        def final_z(label):
            a = nonlocal_state["a"]; b = nonlocal_state["b"]; sv = nonlocal_state["solver"]
            for _ in range(40):
                tpa = control.joint_target_q.numpy()
                for di, tgt in target_by_qdidx.items():
                    if di < len(tpa):
                        tpa[di] = tgt
                control.joint_target_q.assign(tpa)
                a.clear_forces()
                if contacts is not None:
                    model.collide(a, contacts); sv.step(a, b, control, contacts, 0.016)
                else:
                    sv.step(a, b, control, None, 0.016)
                a, b = b, a
            nonlocal_state["a"] = a; nonlocal_state["b"] = b
            bz = float(a.body_q.numpy()[0, 2])
            kq = a.joint_q.numpy()
            kmax = max(abs(float(kq[qi]) - tgt) for qi, tgt in target_by_qidx.items())
            print(f"  cycle {label:2d}: end chassis_z={bz:.3f}  max|q-tgt|={kmax:.3f}")
        for c in range(1, cycles + 1):
            final_z(c)
            do_reset()


if __name__ == "__main__":
    main()
