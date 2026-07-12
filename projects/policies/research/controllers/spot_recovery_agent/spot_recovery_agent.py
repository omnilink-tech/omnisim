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

"""Spot RL recovery-training agent — OmniSim-side TCP controller.

Trains a residual policy on top of the orientation-aware scripted
righting maneuver (`projects/policies/control/spot_recovery.py`). Episode
init spawns the body in a random fallen orientation; each tick the
controller reads body state, asks the trainer for a residual action,
combines that with the scripted pose for the current orientation,
and commands the motors. Reward is shaped to reach an upright
settled body quickly.

Protocol mirrors spot_residual_agent: fixed-size float32 packets,
tag byte (0=ACTION 1=RESET 2=QUIT) plus action vector trainer ->
controller, obs vector + reward + done controller -> trainer.

Observation (16D):
  [0:3]   body angular velocity (world)
  [3:6]   projected gravity in body frame (= body Z axis in world,
          negated; identifies fall orientation)
  [6:9]   body linear velocity (world)
  [9]     body z (world height)
  [10]    roll (rad, wrapped)
  [11]    pitch (rad, wrapped)
  [12]    one-hot encoding [LEFT_SIDE, RIGHT_SIDE, ON_BACK, OTHER]
          collapsed to a 4-vector at [12:16].

Action (12D):
  Per-joint position offset (rad, ±RES_SCALE). Added to the
  scripted-pose joint targets returned by
  `righting_joint_targets(committed_orient, JOINT_ORDER)`, then
  clipped to URDF joint limits before being sent to motors.
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.control.spot_recovery import (  # noqa: E402
    Orientation, classify_orientation, righting_joint_targets,
)
from projects.policies.control.spot_motor_safety import (  # noqa: E402
    apply_realistic_limits, RateLimitedMotorBank,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


URDF_LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_ORDER = []
for leg in URDF_LEGS:
    for joint in ("hip_x", "hip_y", "knee"):
        JOINT_ORDER.append((leg, joint))

OBS_DIM = 16
ACT_DIM = 12
RES_SCALE = 0.30  # max ±0.30 rad residual per joint (larger than walking
                  # since recovery poses are more aggressive)

# Joint limits per URDF (used to clamp model+residual before commanding).
JOINT_LIMITS_LO = np.array([
    -1.50, -0.50, -1.20,
    -1.50, -0.50, -1.20,
    -1.50, -0.50, -1.20,
    -1.50, -0.50, -1.20,
], dtype=np.float32)
JOINT_LIMITS_HI = np.array([
    +1.50, +3.20, -0.01,
    +1.50, +3.20, -0.01,
    +1.50, +3.20, -0.01,
    +1.50, +3.20, -0.01,
], dtype=np.float32)


def _env_float(key: str, default: float) -> float:
    v = os.environ.get(key)
    if v is None or v.strip() == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def _wrap_pi(a: float) -> float:
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--port", type=int, default=7100)
    p.add_argument("--host", default="127.0.0.1")
    args, _ = p.parse_known_args()
    env_port = os.environ.get("SPOT_RL_PORT")
    if env_port:
        try:
            args.port = int(env_port)
        except ValueError:
            pass
    return args


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf.extend(chunk)
    return bytes(buf)


def _orient_one_hot(orient: Orientation) -> np.ndarray:
    o = np.zeros(4, dtype=np.float32)
    if orient == Orientation.LEFT_SIDE:   o[0] = 1.0
    elif orient == Orientation.RIGHT_SIDE: o[1] = 1.0
    elif orient == Orientation.ON_BACK:    o[2] = 1.0
    else:                                  o[3] = 1.0
    return o


def main() -> int:
    args = parse_args()
    sys.stderr.write(f"[spot_recovery_agent] resolved port = {args.port}\n")
    sys.stderr.flush()

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = []
    for leg, joint in JOINT_ORDER:
        name = f"{leg}_{joint}_motor"
        m = robot.getDevice(name)
        if m is None:
            sys.stderr.write(f"[spot_recovery_agent] missing motor {name}\n")
            return 1
        try:
            if hasattr(m, "setControlPID"):
                m.setControlPID(_env_float("SPOT_MOTOR_KP", 20.0), 0.0,
                                _env_float("SPOT_MOTOR_KD", 0.3))
        except Exception:
            pass
        motors.append(m)

    # Realistic actuator limits + rate-limited target wrapper, same as
    # the deploy controller. Recovery torque profile (lower than
    # walking) keeps the simulated forces in a realistic range.
    apply_realistic_limits(motors,
                           max_torque_nm=_env_float("SPOT_MAX_TORQUE_NM", 30.0),
                           max_vel_rad_s=_env_float("SPOT_MAX_VEL_RAD_S", 6.0))
    motor_bank = RateLimitedMotorBank(
        motors, step_dt,
        max_vel_rad_s=_env_float("SPOT_TARGET_RATE_RAD_S", 4.0))

    self_node = robot.getSelf()
    if self_node is None:
        sys.stderr.write("[spot_recovery_agent] no supervisor handle\n")
        return 1
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")

    # ── Reward weights (env-tunable) ──
    # Sparse-ish reward: every step is NEGATIVE so the only way to
    # accumulate positive return is to reach the terminal upright
    # bonus before running out of episode budget. Dense shaping
    # (gravity alignment) is just enough to give a gradient toward
    # the goal, never positive on its own.
    R_UPRIGHT_BONUS  = _env_float("SPOT_REC_UPRIGHT_BONUS", 500.0)
    R_PROGRESS_WT    = _env_float("SPOT_REC_PROGRESS_WT", 0.0)
    R_GRAV_WT        = _env_float("SPOT_REC_GRAV_WT", 0.02)
    R_LEVEL_WT       = _env_float("SPOT_REC_LEVEL_WT", 0.0)
    R_ACTION_WT      = _env_float("SPOT_REC_ACTION_WT", -0.001)
    R_ALIVE          = _env_float("SPOT_REC_ALIVE", -0.50)
    R_CLIP_LO        = _env_float("SPOT_REC_CLIP_LO", -1.0)
    R_CLIP_HI        = _env_float("SPOT_REC_CLIP_HI", 1.0)
    # Curriculum env vars — restrict episode initial orientations.
    # SPOT_REC_INIT can be a comma-separated subset of
    # {face_plant, left_side, right_side, on_back}; default = all.
    init_set_env = os.environ.get(
        "SPOT_REC_INIT", "face_plant,left_side,right_side,on_back")
    INIT_SET = [s.strip() for s in init_set_env.split(",") if s.strip()]

    BZ_UPRIGHT = 0.55
    ROLL_UPRIGHT = 0.30
    PITCH_UPRIGHT = 0.30
    MAX_EPISODE_STEPS = int(_env_float("SPOT_REC_MAX_STEPS", 400))

    # ── TCP server ──
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    sys.stderr.write(f"[spot_recovery_agent] waiting on {args.host}:{args.port}\n")
    sys.stderr.flush()
    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sys.stderr.write(f"[spot_recovery_agent] trainer connected from {addr}\n")
    sys.stderr.flush()

    # ── Episode state ──
    sim_t = 0.0
    episode_step = 0
    prev_bz = 0.10
    last_action = np.zeros(ACT_DIM, dtype=np.float32)

    def random_fallen_pose():
        """Return (translation, axis_angle_rotation) for a random fallen
        initial pose. Mix is determined by the SPOT_REC_INIT
        curriculum env var; default is all four classes mixed."""
        kind = np.random.choice(INIT_SET)
        if kind == "left_side":
            roll = math.pi / 2 + np.random.uniform(-0.20, 0.20)
        elif kind == "right_side":
            roll = -math.pi / 2 + np.random.uniform(-0.20, 0.20)
        elif kind == "on_back":
            roll = math.pi + np.random.uniform(-0.30, 0.30)
        else:  # face_plant
            roll = np.random.uniform(-0.20, 0.20)
        pitch_for_face = (math.pi / 2 + np.random.uniform(-0.30, 0.30)
                          if kind == "face_plant" else
                          np.random.uniform(-0.15, 0.15))
        yaw = np.random.uniform(-0.5, 0.5)
        # Build a small rotation matrix from roll/pitch/yaw and extract
        # an axis-angle for Webots' setSFRotation.
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch_for_face), math.sin(pitch_for_face)
        cy, sy = math.cos(yaw), math.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr],
        ])
        # Axis-angle from rotation matrix.
        angle = math.acos(max(-1.0, min(1.0, (R[0,0]+R[1,1]+R[2,2]-1)*0.5)))
        if abs(angle) < 1e-6:
            axis = np.array([0.0, 0.0, 1.0])
        else:
            s = 2 * math.sin(angle)
            axis = np.array([(R[2,1]-R[1,2])/s,
                             (R[0,2]-R[2,0])/s,
                             (R[1,0]-R[0,1])/s])
            n = np.linalg.norm(axis)
            if n > 1e-6:
                axis /= n
            else:
                axis = np.array([0.0, 0.0, 1.0])
        # Spawn the body slightly above the ground; physics drops it.
        return ([0.0, 0.0, 0.18], [float(axis[0]), float(axis[1]),
                                    float(axis[2]), float(angle)])

    def reset_episode() -> None:
        nonlocal sim_t, episode_step, prev_bz, last_action
        sim_t = 0.0
        episode_step = 0
        last_action = np.zeros(ACT_DIM, dtype=np.float32)
        trans, rot = random_fallen_pose()
        try:
            if translation_field is not None:
                translation_field.setSFVec3f(trans)
            if rotation_field is not None:
                rotation_field.setSFRotation(rot)
            if hasattr(self_node, "resetPhysics"):
                self_node.resetPhysics()
        except Exception:
            pass
        # Let the body settle into its fallen pose for a few ticks
        # before the policy starts acting on it.
        motor_bank.reset()
        # Command a neutral (folded-in) pose during settle.
        SETTLE_POSE = [0.05, 0.05, -1.0,
                       -0.05, 0.05, -1.0,
                       0.05, 0.05, -1.0,
                       -0.05, 0.05, -1.0]
        for _ in range(15):
            motor_bank.set_pose(SETTLE_POSE)
            if robot.step(step_ms) == -1:
                break
        try:
            _p = self_node.getPosition() or [0, 0, 0]
            prev_bz = float(_p[2])
        except Exception:
            prev_bz = 0.10

    reset_episode()

    CMD_BYTES = 1 + ACT_DIM * 4

    while robot.step(step_ms) != -1:
        sim_t += step_dt
        episode_step += 1

        # Read body state.
        try:
            pos = self_node.getPosition() or [0, 0, 0]
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            vel = self_node.getVelocity() or [0]*6
        except Exception:
            pos = [0, 0, 0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6
        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll = math.atan2(ori[7], ori[8])
        pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
        v_lin = np.array(vel[:3], dtype=np.float32)
        v_ang = np.array(vel[3:6], dtype=np.float32)
        # Projected gravity = body Z axis in world frame, negated.
        proj_g = np.array([-ori[2], -ori[5], -ori[8]], dtype=np.float32)
        orient = classify_orientation(
            float(ori[2]), float(ori[5]), float(ori[8]))

        # ── Observation ──
        obs = np.zeros(OBS_DIM, dtype=np.float32)
        obs[0:3] = v_ang
        obs[3:6] = proj_g
        obs[6:9] = v_lin
        obs[9]   = bz
        obs[10]  = roll
        obs[11]  = pitch
        obs[12:16] = _orient_one_hot(orient)
        obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = np.clip(obs, -10.0, 10.0).astype(np.float32)

        # ── Reward ──
        # Potential-based progress shaping: bz - prev_bz (signed).
        # Telescopes so up-down oscillation does NOT accumulate reward;
        # only NET uplift matters. Far more stable training signal than
        # `max(0, ...)` which used to balloon ep_rew on body bounces.
        r_progress = R_PROGRESS_WT * (bz - prev_bz)
        prev_bz = bz
        # Body-Z component of projected gravity: -1 when upright, +1
        # when on back. So r_grav = R_GRAV_WT * (-proj_g_z) rewards
        # "body Z axis aligned with world up", giving a smooth dense
        # gradient toward upright independent of fall orientation.
        r_grav = R_GRAV_WT * (-float(proj_g[2]))
        # Levelness penalty (penalizes roll^2+pitch^2; complements
        # r_grav near upright).
        r_level = R_LEVEL_WT * (roll * roll + pitch * pitch)
        # Action smoothness penalty.
        r_action = R_ACTION_WT * float(np.sum(last_action * last_action))
        reward = R_ALIVE + r_progress + r_grav + r_level + r_action

        # Termination.
        done = False
        is_upright_settled = (bz > BZ_UPRIGHT
                              and abs(roll) < ROLL_UPRIGHT
                              and abs(pitch) < PITCH_UPRIGHT
                              and abs(v_lin[2]) < 0.4)
        if is_upright_settled:
            reward += R_UPRIGHT_BONUS
            done = True
        elif episode_step >= MAX_EPISODE_STEPS:
            done = True
        if not math.isfinite(reward):
            reward = -1.0; done = True
        reward = max(R_CLIP_LO, min(R_CLIP_HI, reward))

        # Send obs+reward+done to trainer.
        pkt = obs.tobytes() + struct.pack("<f", float(reward)) + \
            struct.pack("<B", 1 if done else 0)
        try:
            conn.sendall(pkt)
        except (ConnectionError, OSError):
            sys.stderr.write("[spot_recovery_agent] trainer disconnected\n")
            break

        # Read action.
        try:
            cmd = recv_exact(conn, CMD_BYTES)
        except (ConnectionError, OSError):
            break
        tag = cmd[0]
        if tag == 2:  # QUIT
            break
        if tag == 1 or done:  # RESET
            reset_episode()
            continue
        action = np.frombuffer(cmd, dtype=np.float32, offset=1, count=ACT_DIM).copy()
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        last_action = action.copy()

        # Apply scripted righting pose + residual.
        base_pose = righting_joint_targets(orient, JOINT_ORDER)
        residual_rad = (action * RES_SCALE).astype(np.float32)
        joint_targets = np.array(base_pose, dtype=np.float32) + residual_rad
        joint_targets = np.clip(joint_targets, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        motor_bank.set_pose(joint_targets.tolist())

    try: conn.close()
    except Exception: pass
    try: srv.close()
    except Exception: pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
