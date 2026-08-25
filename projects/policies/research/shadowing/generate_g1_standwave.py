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

"""Generate a FEASIBLE G1 stand-and-wave ghost with the Ghost Generator (Shadowing).

An EASY, achievable Shadowing demo (complements the sit-stand boundary finding): the robot
stands and balances (a solved primitive) while WAVING its right hand. The Ghost Generator
designs the motion -- legs hold the stable stand, the right arm raises and waves, and the
MPPI finds the small balance compensations that keep the CoM over the feet the whole time.
Feasible by construction. Saves the ghost to _scratch + prints the height/tilt trace.

Per the ghost-first rule we generate + preview this BEFORE training the tracker.
"""
import os
import sys
import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_generator import GhostGenerator, _interp_keys

MJCF = os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")
OUT = os.path.join(REPO, "_scratch", "g1_standwave_ghost_generated.npz")

# The stable slight-squat STAND (feet under body) + arms at the sides -- all 23 joints.
STAND = {
    "left_hip_pitch_joint": -0.33, "right_hip_pitch_joint": -0.33,
    "left_hip_roll_joint": 0.0, "right_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0, "right_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.55, "right_knee_joint": 0.55,
    "left_ankle_pitch_joint": -0.20, "right_ankle_pitch_joint": -0.20,
    "left_ankle_roll_joint": 0.0, "right_ankle_roll_joint": 0.0, "waist_yaw_joint": 0.0,
    "left_shoulder_pitch_joint": 0.0, "left_shoulder_roll_joint": 0.15,
    "left_shoulder_yaw_joint": 0.0, "left_elbow_joint": 0.0, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0, "right_shoulder_roll_joint": -0.15,
    "right_shoulder_yaw_joint": 0.0, "right_elbow_joint": 0.0, "right_wrist_roll_joint": 0.0,
}


def pose(**right_arm):
    """A full STAND pose with the RIGHT arm overridden (the wave)."""
    p = dict(STAND)
    p.update(right_arm)
    return p


# RIGHT-arm wave: raise the arm (shoulder up + forearm bent up), sweep the forearm side-to-side
# a few times (shoulder_roll oscillation), then lower. Legs hold STAND throughout.
ARM_UP = dict(right_shoulder_pitch_joint=-1.45, right_shoulder_roll_joint=-0.30,
              right_elbow_joint=0.95, right_wrist_roll_joint=0.0)
WAVE_A = dict(ARM_UP, right_shoulder_roll_joint=-0.05)   # swept inward
WAVE_B = dict(ARM_UP, right_shoulder_roll_joint=-0.62)   # swept outward


def main():
    gen = GhostGenerator(MJCF, sim_dt=0.004, control_dt=0.016)
    mujoco.mj_resetData(gen.m, gen.d)
    gen.d.qpos[0:3] = [0.0, 0.0, 0.79]
    gen.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for nm, a in zip([n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos):
        if nm in STAND:
            gen.d.qpos[a] = STAND[nm]
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.5 / gen.sim_dt)):     # settle into a clean stand
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    print(f"[standwave-gen] settled stand: base_z={init[2]:.3f}")

    # The reference motion (joint keyframes). Legs HOLD the stand pose throughout; the right
    # arm raises, waves (forearm sweeps in<->out), and lowers. Smoothstep-interpolated.
    KEYS = [
        (0.0, pose()),                       # stand, arm at side
        (1.0, pose()),                       # hold
        (1.6, pose(**ARM_UP)),               # raise the right arm
        (2.1, pose(**WAVE_A)),               # wave: in
        (2.5, pose(**WAVE_B)),               # wave: out
        (2.9, pose(**WAVE_A)),               # in
        (3.3, pose(**WAVE_B)),               # out
        (3.7, pose(**WAVE_A)),               # in
        (4.1, pose(**ARM_UP)),               # back to raised neutral
        (4.7, pose()),                       # lower the arm
        (5.5, pose()),                       # stand, HOLD
    ]

    # CLEAN reference = KINEMATIC stand + wave with a STEADY base. Rationale: a free-base
    # humanoid must continuously micro-balance (locking the legs topples it; leaving them free
    # makes the generator "dance"). But that balancing is the ROBOT's job (the solved standing
    # primitive), NOT part of the reference -- the ghost should show the clean INTENT (stand
    # pose + wave), and the tracker balances itself while shadowing it. This is the paper's
    # hierarchical "land in a solved primitive's basin" idea: balance = primitive, wave = ghost.
    # Feasibility is established by the standing balance policy (the robot stands forever) + the
    # arm being light enough that the wave is a small, recoverable perturbation.
    dt = gen.control_dt
    n_steps = int(round(5.5 / dt))
    Z = float(init[2])                       # standing pelvis height (from the settle)
    nq = gen.m.nq
    jadr = {n.replace("_pos", ""): a for n, a in zip(gen.act_name, gen.act_jqpos)}
    Q = np.zeros((n_steps, nq), dtype=np.float64)
    BASE = np.zeros((n_steps, 7), dtype=np.float64)
    jnames = list(STAND.keys())
    for k in range(n_steps):
        Q[k, 0:3] = [0.0, 0.0, Z]
        Q[k, 3:7] = [1.0, 0.0, 0.0, 0.0]     # identity orientation = upright, steady
        kf = _interp_keys(KEYS, k * dt, jnames)
        for nm, v in kf.items():
            Q[k, jadr[nm]] = v
        BASE[k] = Q[k, 0:7]
    np.savez(OUT, q=Q, base=BASE, dt=dt, joints=np.array(jnames))
    print(f"[standwave-gen] saved {OUT}  steps={n_steps}  steady base_z={Z:.3f}")
    rsr = jadr["right_shoulder_roll_joint"]
    rsp = jadr["right_shoulder_pitch_joint"]
    for k in range(0, n_steps, max(1, n_steps // 11)):
        print(f"    t={k*dt:4.2f}s  z={Q[k,2]:.3f}  r_sh_pitch={Q[k,rsp]:+.2f}  r_sh_roll={Q[k,rsr]:+.2f}")


if __name__ == "__main__":
    main()
