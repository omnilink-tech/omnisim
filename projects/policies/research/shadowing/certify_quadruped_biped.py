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

"""Validate the locomotion feasibility certificate (ghost_verifier_dyn) on a
QUADRUPED (Unitree B2) walk ghost and a BIPED (Unitree G1) walk ghost --
showing it generalises across morphology. Builds a flat analytic walk ghost
from each robot's gait model and certifies it.
"""
import os
import sys
import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_verifier_dyn import verify_legged


def _hinge_name2adr(m):
    out = {}
    for j in range(m.njnt):
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            out[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)] = m.jnt_qposadr[j]
    return out


def build_b2_walk_ghost(mjcf, out, T=3.0, dt=0.02):
    from projects.policies.control.gait import b2_trot_gait as stg
    m = mujoco.MjModel.from_xml_path(mjcf); a = _hinge_name2adr(m)
    JN = [f"{l}_{p}" for l in ("FL", "FR", "RL", "RR") for p in ("hip", "thigh", "calf")]
    gp = stg.GaitParams(); FOOT_R = 0.032
    N = int(T / dt); q = np.zeros((N, m.nq)); qv = np.zeros((N, m.nv)); xs = np.zeros(N); x = 0.0
    for k in range(N):
        t = k * dt
        legs, _ = stg.targets_np(stg.QS_PHASE + 2 * np.pi * gp.freq * t, gp, t_since_start=t)
        x += gp.vx * min(1.0, t / gp.ramp_s) * dt; xs[k] = x
        q[k, 0] = x; q[k, 2] = gp.body_height + FOOT_R; q[k, 3] = 1.0
        for i in range(12):
            q[k, a[JN[i]]] = legs[i]
    _fill_vel(q, qv, xs, [a[j] for j in JN], dt)
    np.savez(out, q=q, qvel=qv, dt=np.float64(dt))


def build_g1_walk_ghost(mjcf, out, T=3.0, dt=0.02):
    from projects.policies.control.gait import g1_human_gait as hg
    m = mujoco.MjModel.from_xml_path(mjcf); a = _hinge_name2adr(m)
    LEG13 = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
             "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
             "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
             "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
             "waist_yaw_joint"]
    gp = hg.GaitParams()
    N = int(T / dt); q = np.zeros((N, m.nq)); qv = np.zeros((N, m.nv)); xs = np.zeros(N); x = 0.0
    for k in range(N):
        t = k * dt
        legs, arms, _ = hg.targets_np(hg.DS_PHASE + 2 * np.pi * gp.freq * t, gp, t_since_start=t)
        x += gp.vx * min(1.0, t / gp.ramp_s) * dt; xs[k] = x
        q[k, 0] = x; q[k, 2] = gp.pelvis_height; q[k, 3] = 1.0   # flat upright pelvis
        for i, nm in enumerate(LEG13):
            if nm in a:
                q[k, a[nm]] = legs[i]
        # arms left at 0 (neutral) -- low-mass, negligible for walk base-wrench feasibility
    _fill_vel(q, qv, xs, [a[j] for j in LEG13 if j in a], dt)
    np.savez(out, q=q, qvel=qv, dt=np.float64(dt))


def _fill_vel(q, qv, xs, joint_adrs, dt):
    N = q.shape[0]
    for k in range(N):
        km, kp = max(0, k - 1), min(N - 1, k + 1); den = (kp - km) * dt or 1.0
        qv[k, 0] = (xs[kp] - xs[km]) / den
        for adr in joint_adrs:
            qv[k, adr - 1] = (q[kp, adr] - q[km, adr]) / den


def main():
    sc = os.path.join(REPO, "_scratch")
    b2_mjcf = os.path.join(REPO, "projects/robots/unitree/b2/urdf/b2_planner.mjcf.xml")
    g1_mjcf = os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")
    b2_npz = os.path.join(sc, "b2_walk_ghost.npz")
    g1_npz = os.path.join(sc, "g1_walk_ghost.npz")

    print("=" * 78, "\nB2 (Unitree quadruped, ~60 kg) flat trot walk ghost\n" + "=" * 78)
    build_b2_walk_ghost(b2_mjcf, b2_npz)
    print(f"[verify-dyn] {os.path.basename(b2_npz)}", end="")
    verify_legged(np.load(b2_npz), b2_mjcf, verbose=True)

    print("=" * 78, "\nG1 (Unitree biped, single-support) flat human-gait walk ghost\n" + "=" * 78)
    build_g1_walk_ghost(g1_mjcf, g1_npz)
    print(f"[verify-dyn] {os.path.basename(g1_npz)}", end="")
    verify_legged(np.load(g1_npz), g1_mjcf, verbose=True)


if __name__ == "__main__":
    main()
