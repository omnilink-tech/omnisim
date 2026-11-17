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

"""STAGE-0 probe: does newton `control.joint_f` (joint torque) route through
SolverMuJoCo (GPU mjwarp) and actually MOVE a G1 joint?

This validates the torque-control path end-to-end with ZERO native rebuild,
before touching OmNewtonBackend.cpp. If joint_f works here, the deploy torque
sink is worth building; if it doesn't move the joint, the whole torque path is
moot.

Method: build the deploy-faithful native G1 (build_g1_native), but set ONE joint
(left knee) to EFFORT mode (no PD actuator -> the user supplies torque via
joint_f) while the other 12 joints hold NOMINAL via the normal POSITION_VELOCITY
PD (so the robot stays roughly upright and the knee responds cleanly). Then run
the EXACT deploy substep loop twice -- once with +tau on the knee, once with
-tau -- and compare the knee angle. If +tau and -tau drive the knee in opposite
directions, joint_f routes through SolverMuJoCo. Gravity is identical in both
runs, so the +tau/-tau DIFFERENCE isolates the applied torque.

Run from PowerShell (native CUDA/warp path), e.g.:
  python projects/policies/research/training/probe_newton_joint_torque.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import warp as wp
import newton

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(_REPO))
from projects.policies.research.training.build_g1_native import (  # noqa: E402
    URDF, NOMINAL13, KE, KD, EFFORT, SPAWN_Z, DOF0, NJ, QPOS0)

# Left knee = index 3 in NOMINAL13 (LHP,LHR,LHY,LKN,...). DOF = DOF0+3; the
# generalized COORD (after the 7-wide free joint) = QPOS0+3.
KNEE_I = 3
KNEE_DOF = DOF0 + KNEE_I           # joint_f / joint_qd index
KNEE_COORD = QPOS0 + KNEE_I        # joint_q (coord) index


def _build(knee_effort: bool):
    """Native G1; if knee_effort, put the left knee in EFFORT mode (ke=kd=0)."""
    mb = newton.ModelBuilder()
    mb.add_urdf(str(URDF),
                xform=wp.transform((0.0, 0.0, SPAWN_Z), (0.0, 0.0, 0.0, 1.0)),
                floating=True)
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    eff = int(newton.JointTargetMode.EFFORT)
    for d in range(DOF0, DOF0 + NJ):
        is_knee = (d == KNEE_DOF)
        if knee_effort and is_knee:
            mb.joint_target_ke[d] = 0.0
            mb.joint_target_kd[d] = 0.0
            mb.joint_target_mode[d] = eff
        else:
            mb.joint_target_ke[d] = KE
            mb.joint_target_kd[d] = KD
            mb.joint_target_mode[d] = pv
        mb.joint_effort_limit[d] = 200.0
        mb.joint_target_q[d] = float(NOMINAL13[d - DOF0])
    for i in range(NJ):
        mb.joint_q[QPOS0 + i] = float(NOMINAL13[i])
    mb.add_ground_plane()
    return mb.finalize()


def _knee_angle(model, state, jq, jqd):
    newton.eval_ik(model, state, jq, jqd)
    return float(jq.numpy()[KNEE_COORD])


def _run(tau: float, steps: int = 30, substeps: int = 4):
    """Apply constant tau (Nm) to the left knee for `steps`; return the knee
    angle trajectory."""
    model = _build(knee_effort=True)
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False)
    state_a, state_b = model.state(), model.state()
    control = model.control()
    contacts = model.contacts() if hasattr(model, "contacts") else None
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)

    # Hold the 12 non-knee joints at NOMINAL.
    tp = control.joint_target_q.numpy()
    tp[DOF0:DOF0 + NJ] = NOMINAL13
    control.joint_target_q.assign(tp)

    jq = wp.zeros(model.joint_coord_count, dtype=wp.float32)
    jqd = wp.zeros(model.joint_dof_count, dtype=wp.float32)
    f_host = control.joint_f.numpy()

    dt = 0.016
    sub_dt = dt / substeps
    a0 = _knee_angle(model, state_a, jq, jqd)
    traj = [a0]
    for k in range(steps):
        # joint_f is NOT auto-zeroed across ticks -> set every tick.
        f_host[:] = 0.0
        f_host[KNEE_DOF] = tau
        control.joint_f.assign(f_host)
        for _ in range(substeps):
            state_a.clear_forces()
            if contacts is not None:
                model.collide(state_a, contacts)
                solver.step(state_a, state_b, control, contacts, sub_dt)
            else:
                solver.step(state_a, state_b, control, None, sub_dt)
            state_a, state_b = state_b, state_a
        traj.append(_knee_angle(model, state_a, jq, jqd))
    return np.array(traj)


def main() -> int:
    wp.init()
    print(f"[probe] knee DOF={KNEE_DOF} coord={KNEE_COORD}; EFFORT mode on left knee")
    print(f"[probe] newton.JointTargetMode.EFFORT = {int(newton.JointTargetMode.EFFORT)}")
    TAU = 60.0
    pos = _run(+TAU)
    neg = _run(-TAU)
    print(f"[probe] +{TAU}Nm: knee {pos[0]:+.4f} -> {pos[-1]:+.4f}  (Δ={pos[-1]-pos[0]:+.4f})")
    print(f"[probe] -{TAU}Nm: knee {neg[0]:+.4f} -> {neg[-1]:+.4f}  (Δ={neg[-1]-neg[0]:+.4f})")
    sep = float(pos[-1] - neg[-1])
    print(f"[probe] separation (+tau minus -tau) = {sep:+.4f} rad")
    finite = bool(np.isfinite(pos).all() and np.isfinite(neg).all())
    # joint_f works iff the two torque signs drive the knee meaningfully apart.
    ok = finite and abs(sep) > 0.05
    print(f"[probe] RESULT: {'PASS - joint_f routes through SolverMuJoCo (torque moves the joint)' if ok else 'FAIL - joint_f had no effect (torque path NOT usable)'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
