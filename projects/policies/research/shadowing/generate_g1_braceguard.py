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

"""Generate a FEASIBLE G1 "BraceGuard" defensive-stance + two-hand-block ghost (Shadowing).

A push-recovery companion to the stand-and-wave demo. The robot adopts a MORE-SECURE
defensive stance (deeper squat + abducted feet for a wider base) and HOLDS a relaxed
two-hand guard. On cue it performs a feedforward "orient-and-bar": the waist turns and
both arms drive OUT into a forearm bar (a two-hand block), then return to the guard rest.

As with the stand-wave ghost the BASE is KINEMATIC and STEADY (identity orientation, fixed
height): a free-base humanoid must continuously micro-balance, but that balancing is the
ROBOT's job (the solved standing primitive). The ghost shows the clean INTENT -- the secure
stance + the orient-and-bar -- and the RL tracker balances itself while shadowing it. The
wider abducted base + deeper squat make the stance MORE recoverable than the nominal stand;
the arm bar is a small, recoverable perturbation.

Per the ghost-first rule we generate + preview this BEFORE training the tracker. Saves the
ghost (with qvel, required by ghost_to_trainer_ref) to _scratch + prints the height/pose trace.
"""
import os
import sys
import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_generator import GhostGenerator, _interp_keys

MJCF = os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")
OUT = os.path.join(REPO, "_scratch", "g1_braceguard_ghost.npz")

# SECURE STANCE held throughout (the base pose for ALL frames): a deeper squat than the
# nominal stand, with abducted hips (wider feet) + matched counter-rotating ankle rolls so
# the soles stay flat on the floor, and a relaxed two-hand GUARD-REST for the arms.
SECURE = {
    # legs -- deeper squat, planted
    "left_hip_pitch_joint": -0.34, "right_hip_pitch_joint": -0.34,
    "left_hip_roll_joint": 0.10, "right_hip_roll_joint": -0.10,     # abduct => wider feet
    "left_hip_yaw_joint": 0.0, "right_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.60, "right_knee_joint": 0.60,
    "left_ankle_pitch_joint": -0.25, "right_ankle_pitch_joint": -0.25,
    "left_ankle_roll_joint": 0.10, "right_ankle_roll_joint": -0.10, # counter-rotate soles flat
    "waist_yaw_joint": 0.0,
    # arms -- relaxed two-hand guard rest (elbows up, hands in front)
    "left_shoulder_pitch_joint": 0.30, "left_shoulder_roll_joint": 0.22,
    "left_shoulder_yaw_joint": 0.20, "left_elbow_joint": 0.90, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.30, "right_shoulder_roll_joint": -0.22,
    "right_shoulder_yaw_joint": -0.20, "right_elbow_joint": 0.90, "right_wrist_roll_joint": 0.0,
}

# The 11 joints that MOVE during the orient-and-bar: waist_yaw + the 10 arm joints. The legs
# HOLD the SECURE stance for every keyframe (planted feet, steady base -- HARD RULE).
GUARD_REST = {
    "waist_yaw_joint": 0.0,
    "left_shoulder_pitch_joint": 0.30, "left_shoulder_roll_joint": 0.22,
    "left_shoulder_yaw_joint": 0.20, "left_elbow_joint": 0.90, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.30, "right_shoulder_roll_joint": -0.22,
    "right_shoulder_yaw_joint": -0.20, "right_elbow_joint": 0.90, "right_wrist_roll_joint": 0.0,
}

# BAR-OUT: both arms drive out into a forearm bar, waist turns to orient. Shoulders flatten
# (pitch -0.10), abduct slightly (roll +/-0.05), yaw OUT (+/-0.35), elbows extend (0.45).
BAR_OUT = {
    "waist_yaw_joint": 0.6,
    "left_shoulder_pitch_joint": -0.10, "left_shoulder_roll_joint": 0.05,
    "left_shoulder_yaw_joint": 0.35, "left_elbow_joint": 0.45, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": -0.10, "right_shoulder_roll_joint": -0.05,
    "right_shoulder_yaw_joint": -0.35, "right_elbow_joint": 0.45, "right_wrist_roll_joint": 0.0,
}


def pose(upper):
    """A full SECURE-stance pose with the moving (waist + arm) joints overridden."""
    p = dict(SECURE)
    p.update(upper)
    return p


def main():
    gen = GhostGenerator(MJCF, sim_dt=0.004, control_dt=0.016)
    mujoco.mj_resetData(gen.m, gen.d)
    gen.d.qpos[0:3] = [0.0, 0.0, 0.74]            # deeper squat -> lower pelvis than 0.78
    gen.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for nm, a in zip([n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos):
        if nm in SECURE:
            gen.d.qpos[a] = SECURE[nm]
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.5 / gen.sim_dt)):        # settle into a clean secure stance
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    print(f"[braceguard-gen] settled secure stance: base_z={init[2]:.3f}")

    # Orient-and-bar keyframes. ONLY waist_yaw + the 10 arm joints move; the legs hold the
    # SECURE stance (every pose() carries the full SECURE leg pose). Smoothstep-interpolated.
    KEYS = [
        (0.0, pose(GUARD_REST)),   # guard rest
        (1.0, pose(GUARD_REST)),   # settle / hold
        (1.6, pose(BAR_OUT)),      # BAR-OUT + waist turn (the two-hand block)
        (1.9, pose(BAR_OUT)),      # bar-out hold
        (2.4, pose(GUARD_REST)),   # back to guard rest, waist 0
        (4.0, pose(GUARD_REST)),   # guard rest hold
    ]

    dt = gen.control_dt                           # 0.016 s
    total_t = 4.0
    n_steps = int(round(total_t / dt))
    # Steady KINEMATIC base: identity orientation, fixed pelvis height = the settled squat.
    # (Use the deeper-squat spawn 0.74 as the steady height; the post-settle height is logged
    #  for reference. Keeping z fixed = "balance is the robot's job", the standwave rationale.)
    Z = 0.74
    nq = gen.m.nq
    nv = gen.m.nv
    jadr = {n.replace("_pos", ""): a for n, a in zip(gen.act_name, gen.act_jqpos)}
    # dof (qvel) address for each tracked joint -- for the finite-difference joint velocities
    jdof = {n.replace("_pos", ""): gen.m.jnt_dofadr[gen.m.actuator_trnid[a, 0]]
            for n, a in zip(gen.act_name, gen.pos_act)}
    jnames = list(SECURE.keys())

    Q = np.zeros((n_steps, nq), dtype=np.float64)
    BASE = np.zeros((n_steps, 7), dtype=np.float64)
    for k in range(n_steps):
        Q[k, 0:3] = [0.0, 0.0, Z]
        Q[k, 3:7] = [1.0, 0.0, 0.0, 0.0]          # identity orientation = upright, steady
        kf = _interp_keys(KEYS, k * dt, jnames)
        for nm, v in kf.items():
            Q[k, jadr[nm]] = v
        BASE[k] = Q[k, 0:7]

    # ---- qvel by finite difference (required by ghost_to_trainer_ref) ----
    # free-base linear vel = d(base xyz)/dt ; base angular vel from the quaternion difference;
    # joint velocities = d(joint angle)/dt. First frame = 0. qvel layout = nv (dof) order:
    #   [0:3] base linear (world), [3:6] base angular, [6:] joints by dof address.
    QVEL = np.zeros((n_steps, nv), dtype=np.float64)
    for k in range(1, n_steps):
        QVEL[k, 0:3] = (Q[k, 0:3] - Q[k - 1, 0:3]) / dt          # base linear velocity
        # base angular velocity from quaternion difference: w_world ~ 2 * (dq) * conj(q) (vec)
        qprev = Q[k - 1, 3:7]
        qcur = Q[k, 3:7]
        dq = (qcur - qprev) / dt
        # omega = 2 * dq (x) conj(qcur), take vector part (small-angle, base is identity here)
        w0, x0, y0, z0 = qcur
        cw, cx, cy, cz = w0, -x0, -y0, -z0        # conjugate of qcur
        dw, dx, dy, dz = dq
        # quaternion product (dq * conj(qcur)) vector part, times 2
        wx = dw * cx + dx * cw + dy * cz - dz * cy
        wy = dw * cy - dx * cz + dy * cw + dz * cx
        wz = dw * cz + dx * cy - dy * cx + dz * cw
        QVEL[k, 3:6] = [2.0 * wx, 2.0 * wy, 2.0 * wz]
        for nm in jnames:                          # joint velocities
            QVEL[k, jdof[nm]] = (Q[k, jadr[nm]] - Q[k - 1, jadr[nm]]) / dt

    np.savez(OUT, q=Q, base=BASE, qvel=QVEL, dt=dt, joints=np.array(jnames))
    print(f"[braceguard-gen] saved {OUT}  steps={n_steps}  steady base_z={Z:.3f}  "
          f"nq={nq} nv={nv}  (post-settle z was {init[2]:.3f})")

    wy = jadr["waist_yaw_joint"]
    lsy = jadr["left_shoulder_yaw_joint"]
    rsy = jadr["right_shoulder_yaw_joint"]
    lel = jadr["left_elbow_joint"]
    print("    t[s]   base_z   waist_yaw  l_sh_yaw  r_sh_yaw  l_elbow")
    for k in range(0, n_steps, max(1, n_steps // 16)):
        print(f"    {k*dt:5.2f}  {Q[k,2]:.3f}   {Q[k,wy]:+.3f}    {Q[k,lsy]:+.3f}   "
              f"{Q[k,rsy]:+.3f}   {Q[k,lel]:+.3f}")
    print(f"    base steady? z range=[{Q[:,2].min():.3f},{Q[:,2].max():.3f}]  "
          f"quat all-identity? {np.allclose(Q[:,3:7], [1,0,0,0])}  "
          f"|base_lin_vel|max={np.abs(QVEL[:,0:3]).max():.4f}  "
          f"|jvel|max={np.abs(QVEL[:,6:]).max():.3f}")


if __name__ == "__main__":
    main()
