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

"""Generate a FEASIBLE G1 sit-stand ghost with the Ghost Generator (Shadowing Component 1).

We give the generator only the START (seated on the chair) + the GOAL (standing) + a balance
cost, and let receding-horizon MPPI DISCOVER the dynamically-feasible rise over the real
dynamics + chair/foot contacts -- instead of hand-drawing a (possibly infeasible) kinematic
ghost. Saves the ghost to _scratch and prints the achieved height/tilt trace.
"""
import os
import sys
import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_generator import GhostGenerator, Intent
import projects.policies.control.gait.g1_sitstand as GS

MJCF = os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_sit_kp100.mjcf.xml")
OUT = os.path.join(REPO, "_scratch", "g1_sitstand_ghost_generated.npz")


def main():
    gen = GhostGenerator(MJCF, sim_dt=0.004, control_dt=0.016)   # match trainer DT (feedforward)

    # --- seated init: perch pose on the chair, then settle so it rests in contact ---
    mujoco.mj_resetData(gen.m, gen.d)
    seated = GS.SEATED_POSE
    qp = gen.d.qpos
    qp[0:3] = [0.15, 0.0, 0.56]          # perch at the seat front edge
    qp[3:7] = [1.0, 0.0, 0.0, 0.0]
    for nm, a in zip([n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos):
        if nm in seated:
            qp[a] = seated[nm]
    gen.d.qpos[:] = qp
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.4 / gen.sim_dt)):    # settle onto the chair
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    print(f"[sitstand-gen] settled seated: base_z={init[2]:.3f}")

    # --- intent: rise to a slight-squat UPRIGHT stand, then HOLD it ---
    # full 23-joint targets so the ARMS are tracked too (no flail). Seated = the perch pose;
    # standing = slight-squat legs + arms at the sides.
    seated_full = dict(seated)
    stand_full = dict(seated)
    stand_full.update({
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
    })
    intent = Intent(
        total_time=6.0,                                   # rise (~3 s) + HOLD standing (~3 s)
        joint_keys=[(0.0, seated_full), (1.2, seated_full), (3.0, stand_full), (6.0, stand_full)],
        base_keys=[(0.0, {"x": 0.15, "z": 0.56, "pitch": 0.0}),
                   (1.5, {"x": 0.28, "z": 0.60, "pitch": 0.30}),  # lean + shift fwd over feet
                   (3.0, {"x": 0.40, "z": 0.76, "pitch": 0.0}),    # upright stand
                   (6.0, {"x": 0.40, "z": 0.76, "pitch": 0.0})],   # HOLD (x pinned -> no over-drift)
        weights=dict(joint=3.0, base_xy=3.0, base_z=10.0, pitch=4.0,
                     upright=3.0, balance=5.0, effort=0.02, smooth=0.05, alive=0.5),
    )

    g = gen.generate(intent, init, n_samples=96, horizon=20, noise=0.18,
                     temperature=0.5, seed=0)
    np.savez(OUT, **{k: v for k, v in g.items() if k != "joints"}, joints=np.array(g["joints"]))
    peak = g["base"][:, 2].max()
    print(f"[sitstand-gen] saved {OUT}  steps={g['q'].shape[0]}  peak_base_z={peak:.3f}  final_z={g['base'][-1,2]:.3f}")
    # trace
    dt = g["dt"]
    for k in range(0, g["q"].shape[0], max(1, g["q"].shape[0] // 10)):
        b = g["base"][k]
        tilt = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (b[4] ** 2 + b[5] ** 2)))))
        print(f"    t={k*dt:4.2f}s  base_z={b[2]:.3f}  x={b[0]:+.3f}  tilt={tilt:4.1f}deg")


if __name__ == "__main__":
    main()
