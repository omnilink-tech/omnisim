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

"""OmniSim deploy of UNITREE'S OFFICIAL G1 walk policy (unitree_rl_gym).

Runs Unitree's pre-trained ``pre_train/g1/motion.pt`` with their EXACT control
stack, on Unitree's faithful 12-DOF model (g1_12dof_faithful.urdf):

  * 47-d observation, assembled byte-for-byte like deploy_mujoco.py:
      [ base_ang_vel_BODY*0.25 (3), projected_gravity (3),
        cmd*[2,2,0.25] (3), (q-default) (12), qd*0.05 (12),
        last_action (12), sin/cos(2*pi*phase) (2) ]   phase period 0.8s
  * torque PD every physics step:  tau = (target-q)*kp - qd*kd
      kp=[100,100,100,150,40,40]x2  kd=[2,2,2,4,2,2]x2  (per-joint, Unitree's)
  * action -> target:  target = action*0.25 + default_pose
      default (knees-bent) = [-0.1,0,0,0.3,-0.2,0]x2
  * policy runs every 10 physics steps (50 Hz) over 500 Hz physics, matching
    Unitree's simulation_dt=0.002 + control_decimation=10.

Their MuJoCo deploy walks 25 m / 60 s; OmniSim's deploy engine is mujoco_warp,
so with their faithful model + this exact control the same walk runs in OmniSim.
"""
import os
import sys
import math
import time
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())   # repo root
sys.path.append(str(REPO))

from omnisim import Supervisor   # OmniSim controller API
import torch

LEGS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
KP = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], dtype=np.float32)
KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], dtype=np.float32)
DEFAULT = np.array([-0.1, 0, 0, 0.3, -0.2, 0, -0.1, 0, 0, 0.3, -0.2, 0], dtype=np.float32)
CMD = np.array([float(os.environ.get("G1_CMD_VX", "0.5")),
                float(os.environ.get("G1_CMD_VY", "0.0")),
                float(os.environ.get("G1_CMD_WZ", "0.0"))], dtype=np.float32)
CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
# Heading-hold: a velocity-command policy sees yaw RATE but not absolute heading,
# so small biases integrate into a curve. The supervisor knows true yaw, so close
# the loop: feed a small wz command to null heading error and walk dead-straight.
# This is exactly how you steer such a policy straight on real hardware (in-dist).
HEADING_HOLD = os.environ.get("G1_HEADING_HOLD", "1") == "1"
YAW_KP = float(os.environ.get("G1_YAW_KP", "1.5"))
WZ_MAX = float(os.environ.get("G1_WZ_MAX", "0.5"))
DECIMATION = int(os.environ.get("G1_DECIMATION", "10"))
POLICY = os.environ.get("G1_UNITREE_POLICY",
                        str(Path(__file__).parent / "motion.pt"))
LOG = os.environ.get("G1_DEPLOY_LOG")
_jlog_path = os.environ.get("G1_DEPLOY_JOINT_LOG")
JLOG = open(_jlog_path, "w", buffering=1) if _jlog_path else None


def proj_gravity(ori):
    # ori = 9-float row-major body->world rotation; gravity in body frame.
    return np.array([-ori[6], -ori[7], -ori[8]], dtype=np.float32)


def world_ang_to_body(ori, w):
    # R^T @ w  (ori row-major body->world)
    return np.array([
        ori[0] * w[0] + ori[3] * w[1] + ori[6] * w[2],
        ori[1] * w[0] + ori[4] * w[1] + ori[7] * w[2],
        ori[2] * w[0] + ori[5] * w[1] + ori[8] * w[2],
    ], dtype=np.float32)


def main():
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0
    self_node = robot.getSelf()

    motors, sensors = [], []
    for j in LEGS:
        m = robot.getDevice(f"{j}_motor")
        if m is None:
            print(f"[g1_unitree] MISSING motor {j}_motor", flush=True)
        motors.append(m)
        s = m.getPositionSensor()
        s.enable(dt_ms)
        sensors.append(s)

    # full-body recording (G1_DEPLOY_JOINT_LOG): also read the PASSIVE arm joints so the
    # authentic official reference captures what the arms actually do (motion.pt is legs-only;
    # arms sit at their defaults -- record the truth of that).
    arm_sensors = []
    if JLOG is not None:
        for j in ("left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
                  "left_shoulder_roll_joint", "right_shoulder_roll_joint",
                  "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
                  "left_elbow_joint", "right_elbow_joint"):
            try:
                _am = robot.getDevice(f"{j}_motor")
                _as = _am.getPositionSensor() if _am is not None else None
                if _as is not None:
                    _as.enable(dt_ms)
                arm_sensors.append((j, _as))
            except Exception:
                arm_sensors.append((j, None))

    policy = torch.jit.load(POLICY)
    policy.eval()
    print(f"[g1_unitree] loaded {POLICY}  dt={dt}s decimation={DECIMATION} "
          f"cmd={CMD.tolist()}", flush=True)

    def read_q():
        return np.array([s.getValue() for s in sensors], dtype=np.float32)

    # settle one step so sensors read
    robot.step(dt_ms)
    prev_q = read_q()
    qd = np.zeros(12, dtype=np.float32)
    alpha = dt / (dt + 0.004)   # light low-pass on finite-diff velocity
    action = np.zeros(12, dtype=np.float32)
    target = DEFAULT.copy()
    obs = np.zeros(47, dtype=np.float32)
    counter = 0
    last_wall = time.time()
    last_sim = 0.0

    while robot.step(dt_ms) != -1:
        q = read_q()
        qd = alpha * ((q - prev_q) / dt) + (1 - alpha) * qd
        prev_q = q
        # torque PD every physics step (Unitree's pd_control)
        tau = (target - q) * KP - qd * KD
        for i, m in enumerate(motors):
            m.setTorque(float(tau[i]))
        counter += 1
        # G1_DEPLOY_JOINT_LOG: record (t, gait-phase, q[12]) per physics tick -- raw material for
        # a RECORD-REPLAY ghost (achievable-ghost rule: the reference = what a real policy DOES).
        if JLOG is not None:
            _ph = (counter * dt) % 0.8 / 0.8
            _ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            # base roll/pitch from the rotation matrix (world z of body axes): the reference
            # robot's OWN attitude -- the honest comparison target for an attitude-match metric.
            _rr = math.atan2(_ori[7], _ori[8]); _pp = -math.asin(max(-1.0, min(1.0, _ori[6])))
            _aq = " ".join("%.5f" % (sn.getValue() if sn is not None else 0.0) for _, sn in arm_sensors)
            JLOG.write("%.4f %.4f %.4f %.4f %s %s\n" % (counter * dt, _ph, _rr, _pp,
                                                        " ".join("%.5f" % v for v in q), _aq))
        if counter % DECIMATION == 0:
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            vel = self_node.getVelocity() or [0.0] * 6
            phase = (counter * dt) % 0.8 / 0.8
            # heading-hold: yaw of body +x axis (world col0 = [ori[0],ori[3]])
            cmd = CMD.copy()
            if HEADING_HOLD:
                yaw = math.atan2(ori[3], ori[0])
                cmd[2] = float(np.clip(-YAW_KP * yaw, -WZ_MAX, WZ_MAX))
            obs[0:3] = world_ang_to_body(ori, vel[3:6]) * 0.25
            obs[3:6] = proj_gravity(ori)
            obs[6:9] = cmd * CMD_SCALE
            obs[9:21] = (q - DEFAULT)
            obs[21:33] = qd * 0.05
            obs[33:45] = action
            obs[45:47] = [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)]
            with torch.no_grad():
                action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze().astype(np.float32)
            target = action * 0.25 + DEFAULT
            if counter % (DECIMATION * 25) == 0:
                pos = self_node.getPosition()
                now = time.time()
                sim_t = counter * dt
                rtf = (sim_t - last_sim) / max(now - last_wall, 1e-6)
                last_sim, last_wall = sim_t, now
                line = (f"[g1_unitree] t={sim_t:6.2f}s x={pos[0]:+.2f}m "
                        f"y={pos[1]:+.2f}m z={pos[2]:.3f} |act|max={np.abs(action).max():.2f} "
                        f"rtf={rtf:.2f}x")
                print(line, flush=True)
                if LOG:
                    with open(LOG, "a") as f:
                        f.write(line + "\n")


main()
