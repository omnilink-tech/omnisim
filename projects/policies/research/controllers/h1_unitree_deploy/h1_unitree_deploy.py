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

"""OmniSim deploy of UNITREE'S OFFICIAL H1 walk policy (unitree_rl_gym).

Same recipe as the G1 deploy, generalized to the H1's 10-DOF legs model:

  * 41-d observation, assembled byte-for-byte like deploy_mujoco.py:
      [ base_ang_vel_BODY*0.25 (3), projected_gravity (3),
        cmd*[2,2,0.25] (3), (q-default) (10), qd*0.05 (10),
        last_action (10), sin/cos(2*pi*phase) (2) ]   phase period 0.8s
  * torque PD every physics step:  tau = (target-q)*kp - qd*kd
      kp=[150,150,150,200,40]x2  kd=[2,2,2,4,2]x2  (per-joint, Unitree's h1.yaml)
  * action -> target:  target = action*0.25 + default_pose
      default (knees-bent) = [0,0,-0.1,0.3,-0.2]x2
  * policy runs every 10 physics steps (50 Hz) over 500 Hz physics.
  * heading-hold outer loop (supervisor yaw -> small wz) walks it dead-straight.

Joint order = Unitree's actuator order: per leg
  [hip_yaw, hip_roll, hip_pitch, knee, ankle].
Their MuJoCo deploy walks 25 m / 60 s; OmniSim's deploy engine is mujoco_warp.
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
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_joint",
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_joint",
]
KP = np.array([150, 150, 150, 200, 40, 150, 150, 150, 200, 40], dtype=np.float32)
KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 4, 2], dtype=np.float32)
DEFAULT = np.array([0, 0, -0.1, 0.3, -0.2, 0, 0, -0.1, 0.3, -0.2], dtype=np.float32)
CMD = np.array([float(os.environ.get("H1_CMD_VX", "0.5")),
                float(os.environ.get("H1_CMD_VY", "0.0")),
                float(os.environ.get("H1_CMD_WZ", "0.0"))], dtype=np.float32)
CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
HEADING_HOLD = os.environ.get("H1_HEADING_HOLD", "1") == "1"
YAW_KP = float(os.environ.get("H1_YAW_KP", "1.5"))
WZ_MAX = float(os.environ.get("H1_WZ_MAX", "0.5"))
# lateral-position hold: heading-hold nulls yaw, but a small lateral-velocity
# bias still crabs the robot off the centerline. Close the loop on world-y via
# the vy command (also in-distribution) so it tracks y=0.
LAT_KP = float(os.environ.get("H1_LAT_KP", "0.25"))
VY_MAX = float(os.environ.get("H1_VY_MAX", "0.2"))
DECIMATION = int(os.environ.get("H1_DECIMATION", "10"))
POLICY = os.environ.get("H1_UNITREE_POLICY",
                        str(Path(__file__).parent / "motion.pt"))
LOG = os.environ.get("H1_DEPLOY_LOG")


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
            print(f"[h1_unitree] MISSING motor {j}_motor", flush=True)
        motors.append(m)
        s = m.getPositionSensor()
        s.enable(dt_ms)
        sensors.append(s)

    policy = torch.jit.load(POLICY)
    policy.eval()
    print(f"[h1_unitree] loaded {POLICY}  dt={dt}s decimation={DECIMATION} "
          f"cmd={CMD.tolist()} n={len(LEGS)}", flush=True)

    def read_q():
        return np.array([s.getValue() for s in sensors], dtype=np.float32)

    robot.step(dt_ms)
    prev_q = read_q()
    qd = np.zeros(len(LEGS), dtype=np.float32)
    alpha = dt / (dt + 0.004)
    action = np.zeros(len(LEGS), dtype=np.float32)
    target = DEFAULT.copy()
    obs = np.zeros(41, dtype=np.float32)
    counter = 0
    last_wall = time.time()
    last_sim = 0.0
    # H1_DEPLOY_JOINT_LOG: full-body per-tick recorder (Shadowing workflow step 2 -- the raw
    # material for a phase-folded ghost): t phase roll pitch vx legs[10]. Mirrors the G1's.
    _jlog_path = os.environ.get("H1_DEPLOY_JOINT_LOG")
    _jlog = open(_jlog_path, "w", buffering=1) if _jlog_path else None

    while robot.step(dt_ms) != -1:
        q = read_q()
        qd = alpha * ((q - prev_q) / dt) + (1 - alpha) * qd
        prev_q = q
        tau = (target - q) * KP - qd * KD
        for i, m in enumerate(motors):
            m.setTorque(float(tau[i]))
        counter += 1
        if _jlog is not None:
            _ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            _vel = self_node.getVelocity() or [0.0] * 6
            _ph = (counter * dt) % 0.8 / 0.8 * 2.0 * math.pi
            _rr = math.atan2(_ori[7], _ori[8]); _pp = -math.asin(max(-1.0, min(1.0, _ori[6])))
            _jlog.write("%.4f %.5f %.5f %.5f %.4f " % (counter * dt, _ph, _rr, _pp, _vel[0])
                        + " ".join("%.5f" % v for v in q) + "\n")
        if counter % DECIMATION == 0:
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            vel = self_node.getVelocity() or [0.0] * 6
            phase = (counter * dt) % 0.8 / 0.8
            cmd = CMD.copy()
            if HEADING_HOLD:
                yaw = math.atan2(ori[3], ori[0])
                cmd[2] = float(np.clip(-YAW_KP * yaw, -WZ_MAX, WZ_MAX))
                ypos = self_node.getPosition()[1]
                cmd[1] = float(np.clip(-LAT_KP * ypos, -VY_MAX, VY_MAX))
            obs[0:3] = world_ang_to_body(ori, vel[3:6]) * 0.25
            obs[3:6] = proj_gravity(ori)
            obs[6:9] = cmd * CMD_SCALE
            obs[9:19] = (q - DEFAULT)
            obs[19:29] = qd * 0.05
            obs[29:39] = action
            obs[39:41] = [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)]
            with torch.no_grad():
                action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze().astype(np.float32)
            target = action * 0.25 + DEFAULT
            if counter % (DECIMATION * 25) == 0:
                pos = self_node.getPosition()
                now = time.time()
                sim_t = counter * dt
                rtf = (sim_t - last_sim) / max(now - last_wall, 1e-6)
                last_sim, last_wall = sim_t, now
                line = (f"[h1_unitree] t={sim_t:6.2f}s x={pos[0]:+.2f}m "
                        f"y={pos[1]:+.2f}m z={pos[2]:.3f} |act|max={np.abs(action).max():.2f} "
                        f"rtf={rtf:.2f}x")
                print(line, flush=True)
                if LOG:
                    with open(LOG, "a") as f:
                        f.write(line + "\n")


main()
