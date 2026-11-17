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

"""Generate a feasible G1 stand-up from a HIGH STOOL (Shadowing, easier-launch variant).

From a high stool the legs are already near-extended and the feet are ~under the body, so
standing is a SHALLOW straighten-up (pelvis ~0.70 -> 0.78) instead of the deep push-off +
forward step a low chair forces. This isolates the launch difficulty -- the shallow rise
should be RL-learnable where the chair launch was not (the Shadowing open problem).
"""
import os
import sys
import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_generator import GhostGenerator, Intent

MJCF = os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_stool_kp100.mjcf.xml")
OUT = os.path.join(REPO, "_scratch", "g1_stool_ghost_generated.npz")

# perched-on-HIGH-stool seated pose (top 0.68): nearly standing, shallow knee bend, feet
# ~under the body. The rise to 0.78 is a tiny ~8cm with a small lean+step -> minimal launch
# energy. This is the genuinely EASIER launch (isolates the contact-break, not deep push-off).
SEATED = {"left_hip_pitch_joint": -0.45, "right_hip_pitch_joint": -0.45,
          "left_knee_joint": 0.95, "right_knee_joint": 0.95,
          "left_ankle_pitch_joint": -0.40, "right_ankle_pitch_joint": -0.40,
          "left_shoulder_pitch_joint": 0.15, "right_shoulder_pitch_joint": 0.15,
          "left_shoulder_roll_joint": 0.15, "right_shoulder_roll_joint": -0.15,
          "left_elbow_joint": 0.25, "right_elbow_joint": 0.25}
STAND = {"left_hip_pitch_joint": -0.33, "right_hip_pitch_joint": -0.33,
         "left_hip_roll_joint": 0.0, "right_hip_roll_joint": 0.0,
         "left_hip_yaw_joint": 0.0, "right_hip_yaw_joint": 0.0,
         "left_knee_joint": 0.55, "right_knee_joint": 0.55,
         "left_ankle_pitch_joint": -0.20, "right_ankle_pitch_joint": -0.20,
         "left_ankle_roll_joint": 0.0, "right_ankle_roll_joint": 0.0, "waist_yaw_joint": 0.0,
         "left_shoulder_pitch_joint": 0.0, "left_shoulder_roll_joint": 0.15,
         "left_shoulder_yaw_joint": 0.0, "left_elbow_joint": 0.0, "left_wrist_roll_joint": 0.0,
         "right_shoulder_pitch_joint": 0.0, "right_shoulder_roll_joint": -0.15,
         "right_shoulder_yaw_joint": 0.0, "right_elbow_joint": 0.0, "right_wrist_roll_joint": 0.0}


def main():
    gen = GhostGenerator(MJCF, sim_dt=0.004, control_dt=0.016)
    mujoco.mj_resetData(gen.m, gen.d)
    gen.d.qpos[0:3] = [0.0, 0.0, 0.73]
    gen.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for nm, a in zip([n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos):
        if nm in SEATED:
            gen.d.qpos[a] = SEATED[nm]
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.5 / gen.sim_dt)):
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    print(f"[stool-gen] settled perch: base_z={init[2]:.3f} x={init[0]:+.3f}")

    seated_j = {k: SEATED.get(k, STAND[k]) for k in STAND}
    # Lean keyframe: pitch the torso forward to carry CoM over the feet before extending (the
    # momentum sit-to-stand -- still needed, but from a high perch the follow-on extension is
    # tiny). A modest forward step (x) lets the generator use its proven catch strategy.
    lean_j = dict(seated_j)
    lean_j.update({"left_hip_pitch_joint": -0.70, "right_hip_pitch_joint": -0.70})  # fold torso fwd
    intent = Intent(
        total_time=5.0,
        joint_keys=[(0.0, seated_j), (0.5, lean_j), (1.1, lean_j), (2.2, STAND), (5.0, STAND)],
        base_keys=[(0.0, {"x": 0.00, "z": init[2], "pitch": 0.00}),
                   (0.6, {"x": 0.10, "z": init[2], "pitch": 0.35}),       # LEAN fwd, CoM over feet
                   (1.2, {"x": 0.18, "z": init[2] + 0.02, "pitch": 0.25}),  # small step + start up
                   (2.2, {"x": 0.20, "z": 0.78, "pitch": 0.00}),          # extend to UPRIGHT stand
                   (5.0, {"x": 0.20, "z": 0.78, "pitch": 0.00})],         # HOLD
        weights=dict(joint=3.0, base_xy=3.0, base_z=9.0, pitch=4.0,
                     upright=2.5, balance=4.0, effort=0.02, smooth=0.05, alive=0.5),
    )
    g = gen.generate(intent, init, n_samples=200, horizon=45, noise=0.22, temperature=0.4, seed=0)
    np.savez(OUT, **{k: v for k, v in g.items() if k != "joints"}, joints=np.array(g["joints"]))
    dt = g["dt"]
    print(f"[stool-gen] saved {OUT}  steps={g['q'].shape[0]}  peak_z={g['base'][:,2].max():.3f}")
    for k in range(0, g["q"].shape[0], max(1, g["q"].shape[0] // 8)):
        b = g["base"][k]
        tilt = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (b[4] ** 2 + b[5] ** 2)))))
        print(f"    t={k*dt:4.2f}s  z={b[2]:.3f}  x={b[0]:+.3f}  tilt={tilt:4.1f}")


if __name__ == "__main__":
    main()
