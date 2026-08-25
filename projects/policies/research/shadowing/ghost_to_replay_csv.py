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

"""Convert a generated ghost .npz (full qpos per step, from ghost_generator) into the
ghost-replay CSV (x,z,axis-angle + 23 joints) that the g1_sitstand_ghost controller
animates in OmniSim. Reusable: pass --npz / --mjcf / --out.

  python projects/policies/research/shadowing/ghost_to_replay_csv.py \
      --npz _scratch/g1_sitstand_ghost_generated.npz \
      --mjcf projects/robots/unitree/g1/urdf/g1_sit_kp100.mjcf.xml \
      --out _scratch/g1_sitstand_generated_replay.csv
"""
import argparse
import csv
import math

import numpy as np
import mujoco

# G1 23-joint order the ghost controller reads (LEGS + ARMS).
_ALL23 = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint",
)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--npz", required=True)
    p.add_argument("--mjcf", required=True)
    p.add_argument("--out", required=True)
    a = p.parse_args()
    m = mujoco.MjModel.from_xml_path(a.mjcf)
    q = np.load(a.npz, allow_pickle=True)["q"]
    adr = {jn: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)] for jn in _ALL23}
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "z", "rx", "ry", "rz", "ra"] + list(_ALL23))
        for i in range(q.shape[0]):
            wq, xq, yq, zq = q[i, 3], q[i, 4], q[i, 5], q[i, 6]
            ang = 2.0 * math.acos(max(-1.0, min(1.0, wq)))
            s = math.sqrt(max(1e-9, 1.0 - wq * wq))
            ax, ay, az = (0.0, 0.0, 1.0) if s < 1e-6 else (xq / s, yq / s, zq / s)
            if s < 1e-6:
                ang = 0.0
            w.writerow([f"{q[i,0]:.5f}", f"{q[i,2]:.5f}", f"{ax:.5f}", f"{ay:.5f}", f"{az:.5f}", f"{ang:.5f}"]
                       + [f"{q[i, adr[jn]]:.5f}" for jn in _ALL23])
    print(f"wrote {a.out}  ({q.shape[0]} frames)")


if __name__ == "__main__":
    main()
