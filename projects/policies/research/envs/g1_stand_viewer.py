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

"""Watch the trained G1 standing policy in MuJoCo's interactive viewer.

Loads the MJCF that the GPU trainer used and the ONNX policy from
`projects/policies/research/training/runs/gpu_g1_stand/policy.onnx`, then steps the
sim continuously while showing it in MuJoCo's built-in 3D viewer.

Same baseline + residual recipe as the env / trainer:
    targets = NOMINAL + ankle balance PD + 0.3 * onnx(obs)

Press ESC in the viewer window to quit. The robot auto-resets when
it falls (matching the training env's reset rule).
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort


REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))


# Layout / gains — must match
# projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py exactly.
LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
NJ = 13
NOMINAL = np.array([
    -0.20, +0.00, +0.00, +0.42, -0.23, +0.00,
    -0.20, +0.00, +0.00, +0.42, -0.23, +0.00,
    +0.00,
], dtype=np.float32)
KP_AP, KD_AP, KP_AR, KD_AR = -1.5, -0.2, -1.5, -0.2
BAL_CLAMP = 0.2
RES_SCALE = 0.3
SPAWN_Z = 0.78
BZ_FAIL, ROLL_FAIL, PITCH_FAIL = 0.45, 0.8, 0.8
DT = 0.016
_L_AP = LEGS_JOINTS.index("left_ankle_pitch_joint")
_R_AP = LEGS_JOINTS.index("right_ankle_pitch_joint")
_L_AR = LEGS_JOINTS.index("left_ankle_roll_joint")
_R_AR = LEGS_JOINTS.index("right_ankle_roll_joint")


def quat_to_rp(qw, qx, qy, qz):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    return roll, pitch


def proj_gravity(qw, qx, qy, qz):
    gx = -2 * (qx * qz - qw * qy)
    gy = -2 * (qy * qz + qw * qx)
    gz = -(1 - 2 * (qx * qx + qy * qy))
    return gx, gy, gz


def main():
    mjcf = REPO / "projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml"
    onnx_path = REPO / "projects/policies/research/training/runs/gpu_g1_stand/policy.onnx"
    if not mjcf.exists():
        print(f"missing {mjcf} — run build_g1_mjcf.py", file=sys.stderr)
        return 1
    if not onnx_path.exists():
        print(f"missing {onnx_path} — run the trainer", file=sys.stderr)
        return 1

    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)

    # Resolve qpos / qvel / actuator indices in MJCF order.
    qpos_idx = []
    qvel_idx = []
    ctrl_pos_idx = []
    for jn in LEGS_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qpos_idx.append(model.jnt_qposadr[jid])
        qvel_idx.append(model.jnt_dofadr[jid])
        ctrl_pos_idx.append(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos"))
    qpos_idx = np.array(qpos_idx, dtype=np.int32)
    qvel_idx = np.array(qvel_idx, dtype=np.int32)
    ctrl_pos_idx = np.array(ctrl_pos_idx, dtype=np.int32)

    sess = ort.InferenceSession(str(onnx_path),
                                providers=["CPUExecutionProvider"])
    print(f"[viewer] loaded policy: {onnx_path.name}")

    def reset():
        mujoco.mj_resetData(model, data)
        data.qpos[0:3] = [0.0, 0.0, SPAWN_Z]
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        jitter = np.random.uniform(-0.05, 0.05, size=NJ).astype(np.float32)
        for k in range(NJ):
            data.qpos[qpos_idx[k]] = NOMINAL[k] + jitter[k]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

    reset()
    last_action = np.zeros(NJ, dtype=np.float32)
    prev_roll = 0.0
    prev_pitch = 0.0
    sim_t = 0.0
    ep_start_t = 0.0
    last_log = 0.0
    sim_dt = float(model.opt.timestep)
    substeps = max(1, int(round(DT / sim_dt)))

    print(f"[viewer] sim_dt={sim_dt}s, control={substeps*sim_dt}s "
          f"({substeps} substeps per policy step)")
    print("[viewer] opening MuJoCo viewer — press ESC to quit")

    with mujoco.viewer.launch_passive(model, data) as v:
        # Camera setup so the robot is centered + zoomed in.
        v.cam.distance = 3.5
        v.cam.elevation = -10
        v.cam.azimuth = 135
        v.cam.lookat[:] = [0.0, 0.0, 0.5]

        while v.is_running():
            # Observation.
            qw, qx, qy, qz = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
            roll, pitch = quat_to_rp(qw, qx, qy, qz)
            pg = np.array(proj_gravity(qw, qx, qy, qz), dtype=np.float32)
            lin_vel = data.qvel[0:3].astype(np.float32)
            ang_vel = data.qvel[3:6].astype(np.float32)
            joint_q = data.qpos[qpos_idx].astype(np.float32) - NOMINAL
            joint_qd = data.qvel[qvel_idx].astype(np.float32)
            obs = np.concatenate([lin_vel, ang_vel, pg, joint_q, joint_qd,
                                  last_action]).astype(np.float32)[None, :]

            # Policy inference.
            action = sess.run(None, {"obs": obs})[0][0]
            action = np.clip(action, -1.0, 1.0).astype(np.float32)

            # Baseline (NOMINAL + ankle balance PD).
            roll_rate = (roll - prev_roll) / DT
            pitch_rate = (pitch - prev_pitch) / DT
            prev_roll, prev_pitch = roll, pitch
            ap = float(np.clip(KP_AP * pitch + KD_AP * pitch_rate,
                               -BAL_CLAMP, BAL_CLAMP))
            ar = float(np.clip(KP_AR * roll + KD_AR * roll_rate,
                               -BAL_CLAMP, BAL_CLAMP))
            baseline = NOMINAL.copy()
            baseline[_L_AP] += ap
            baseline[_R_AP] += ap
            baseline[_L_AR] += ar
            baseline[_R_AR] += ar
            targets = baseline + RES_SCALE * action

            # Write to position actuators.
            for k in range(NJ):
                data.ctrl[ctrl_pos_idx[k]] = float(targets[k])

            # Step the sim.
            for _ in range(substeps):
                mujoco.mj_step(model, data)
            sim_t += substeps * sim_dt
            last_action = action

            # Health check + auto-reset.
            bz = data.qpos[2]
            fall = (abs(roll) > ROLL_FAIL or abs(pitch) > PITCH_FAIL or
                    bz < BZ_FAIL)
            if fall:
                print(f"[viewer] fell at sim_t={sim_t:.2f}s "
                      f"(survived {sim_t - ep_start_t:.2f}s); resetting")
                reset()
                last_action[:] = 0.0
                prev_roll = prev_pitch = 0.0
                ep_start_t = sim_t

            v.sync()
            if sim_t - last_log >= 1.0:
                last_log = sim_t
                print(f"[viewer] t={sim_t:6.1f}s bz={bz:.3f} "
                      f"roll={roll:+.3f} pitch={pitch:+.3f}  STANDING")
            # Throttle to real-time so the user sees the motion at 1×.
            time.sleep(max(0.0, substeps * sim_dt - 0.001))

    return 0


if __name__ == "__main__":
    sys.exit(main())
