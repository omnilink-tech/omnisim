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

"""Spot residual-RL training agent — OmniSim-side controller.

Wraps the model walker (gait + IK + balance) with a TCP protocol the
trainer drives. Same wire protocol as spot_rl_agent but the action
space is interpreted differently:

  Action vector (12 dims, ±1 each, scaled by SPOT_RES_SCALE = 0.03 m):
      per-leg foot offsets (dx, dy, dz) added on top of the model
      walker's per-tick foot target before IK.

  Observation vector (18 dims):
    [0:3]   body angular velocity (world)
    [3:6]   projected gravity in body frame
    [6]     gait phase (FL leg, in [0, 1))
    [7:10]  base linear velocity (world)
    [10:13] velocity command (vx, vy, wz)
    [13]    yaw deviation from spawn (rad, wrapped to [-pi, pi])
    [14]    lateral deviation from spawn (m, body y)
    [15:18] last action's first 3 dims (FL foot offset)  -- partial
            action history to give the policy momentum context

The model walker does the heavy lifting; the policy just compensates
for asymmetry, slip, perturbations.
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.control.spot_kinematics import inverse_kinematics  # noqa: E402
from projects.policies.control.spot_gait import GaitParams, foot_targets  # noqa: E402
from projects.policies.control.spot_balance import (  # noqa: E402
    BalanceParams, balance_offsets,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


URDF_LEGS = ("front_left", "front_right", "rear_left", "rear_right")
URDF_TO_IK = {"front_left": "FL", "front_right": "FR",
              "rear_left": "RL", "rear_right": "RR"}
JOINT_ORDER = []
for leg in URDF_LEGS:
    for joint in ("hip_x", "hip_y", "knee"):
        JOINT_ORDER.append((leg, joint))

OBS_DIM = 18
ACT_DIM = 12
RES_SCALE = 0.03  # max ±3 cm residual on each foot axis


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


def main() -> int:
    args = parse_args()
    sys.stderr.write(f"[spot_residual_agent] resolved port = {args.port}\n")
    sys.stderr.flush()

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = []
    sensors = []
    for leg, joint in JOINT_ORDER:
        name = f"{leg}_{joint}_motor"
        m = robot.getDevice(name)
        if m is None:
            sys.stderr.write(f"[spot_residual_agent] missing motor {name}\n")
            return 1
        kp = _env_float("SPOT_MOTOR_KP", 20.0)
        kd = _env_float("SPOT_MOTOR_KD", 0.3)
        try:
            if hasattr(m, "setControlPID"):
                m.setControlPID(kp, 0.0, kd)
        except Exception:
            pass
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    self_node = robot.getSelf()
    if self_node is None:
        sys.stderr.write("[spot_residual_agent] no supervisor handle\n")
        return 1
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")
    SPAWN_TRANS = [0.0, 0.0, 0.70]
    SPAWN_ROT = [0.0, 0.0, 1.0, 0.0]

    # Model walker config (GaitParams defaults, env-overridable so the
    # Newton training recipe can raise the swing arc: under the post-W1
    # honest leg geometry the default 0.05 m swing GRAZES the ground and
    # drags the body BACKWARD; SPOT_GAIT_STEP_HEIGHT=0.09 restores forward
    # thrust. Must match the deploy controller's gait env.
    _gd = GaitParams()
    gait = GaitParams(
        step_height=_env_float("SPOT_GAIT_STEP_HEIGHT", _gd.step_height),
        ground_z=_env_float("SPOT_GAIT_GROUND_Z", _gd.ground_z),
    )
    balance = BalanceParams()
    # Feedforward hip_y trim — added uniformly to every hip_y joint
    # after IK. Used under Newton to counter a structural nose-down
    # moment that tips Spot pitch-forward from the standing pose
    # within ~25 ticks. Default 0.0 = legacy (ODE) behaviour.
    PITCH_TRIM = _env_float("SPOT_PITCH_TRIM", 0.0)

    # Heading + centreline hold on the BASE walker -- same formula and env
    # knobs as spot_residual_deploy's hold, so the TRAINING base equals the
    # DEPLOY base when both are launched with the same environment (the
    # May-era mismatch -- hold in deploy only -- meant the policy trained
    # on a curving base but deployed on a steered one). Active only while
    # the trainer's commanded wz is 0.
    HOLD = (os.environ.get("SPOT_HEADING_HOLD", "1").strip() != "0")
    HOLD_KP_YAW = _env_float("SPOT_HOLD_KP_YAW", 2.0)
    HOLD_KD_YAW = _env_float("SPOT_HOLD_KD_YAW", 0.4)
    HOLD_KP_LAT = _env_float("SPOT_HOLD_KP_LAT", 0.6)
    HOLD_KP_LAT2YAW = _env_float("SPOT_HOLD_KP_LAT2YAW", 0.18)
    HOLD_WZ_MAX = _env_float("SPOT_HOLD_WZ_MAX", 0.35)
    HOLD_VY_MAX = _env_float("SPOT_HOLD_VY_MAX", 0.12)

    # Reward weights (env-overridable for tuning iteration).
    R_VX_WT = _env_float("SPOT_RES_R_VX_WT", 2.0)        # exp tracking
    R_VX_SCALE = _env_float("SPOT_RES_R_VX_SCALE", 0.10)
    R_HEADING_WT = _env_float("SPOT_RES_R_HEADING_WT", -1.0)  # rad²
    R_LATERAL_WT = _env_float("SPOT_RES_R_LATERAL_WT", -0.5)  # m²
    R_ACTION_WT = _env_float("SPOT_RES_R_ACTION_WT", -0.05)   # ||a||²
    R_ALIVE = _env_float("SPOT_RES_R_ALIVE", 0.05)
    R_TERM = _env_float("SPOT_RES_R_TERM", -10.0)
    R_CLIP_HI = _env_float("SPOT_RES_R_CLIP_HI", 5.0)
    R_CLIP_LO = _env_float("SPOT_RES_R_CLIP_LO", -10.0)
    # Upright bonus: exp(-(roll²+pitch²)/SCALE). Sharp peak at level
    # that falls off fast, giving the policy a strong gradient toward
    # leveling the body — far more useful than a flat alive bonus when
    # external perturbations are pushing the body off-level. Default
    # weight 0 keeps legacy training unchanged.
    R_UPRIGHT_BONUS_WT = _env_float("SPOT_RES_R_UPRIGHT_BONUS_WT", 0.0)
    R_UPRIGHT_BONUS_SCALE = _env_float("SPOT_RES_R_UPRIGHT_BONUS_SCALE", 0.10)

    # Perturbation injection. Every PERTURB_INTERVAL_S of sim time
    # past PERTURB_START_S, apply a horizontal force impulse to the
    # chassis. Total impulse = mass × Δv (Δv uniform in [DV_MIN, DV_MAX]
    # at a random horizontal angle). The impulse is delivered as a
    # constant force applied over FORCE_TICKS consecutive ticks, NOT a
    # single instant velocity teleport — this matches the transient
    # profile of a real collision (e.g., a thrown cube), where peak
    # acceleration is much higher than the average. The single-tick
    # teleport that earlier v1 trained against doesn't generalize to
    # real cube collisions because the policy never saw the high-peak-
    # force transient.
    PERTURB_DV_MIN = _env_float("SPOT_PERTURB_DV_MIN", 0.0)
    PERTURB_DV_MAX = _env_float("SPOT_PERTURB_DV_MAX", 0.0)
    PERTURB_INTERVAL_S = _env_float("SPOT_PERTURB_INTERVAL_S", 3.0)
    PERTURB_START_S = _env_float("SPOT_PERTURB_START_S", 2.0)
    PERTURB_ENABLED = (PERTURB_DV_MAX > 0.0)
    PERTURB_FORCE_TICKS = int(_env_float("SPOT_PERTURB_FORCE_TICKS", 2.0))
    SPOT_MASS_KG = _env_float("SPOT_MASS_KG", 32.0)
    # Active multi-tick force: [fx, fy, ticks_remaining]. None when idle.
    _active_force = [0.0, 0.0, 0]

    # TCP server
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    sys.stderr.write(f"[spot_residual_agent] waiting on {args.host}:{args.port}\n")
    sys.stderr.flush()
    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sys.stderr.write(f"[spot_residual_agent] trainer connected from {addr}\n")
    sys.stderr.flush()

    # Episode state
    BZ_FAIL = 0.30
    ROLL_FAIL = 1.0
    PITCH_FAIL = 1.0
    MAX_EPISODE_STEPS = 1024
    SPOT_FIXED_VX = _env_float("SPOT_FIXED_VX", float("nan"))

    def reset_episode():
        nonlocal episode_step, vel_command, sim_time, episode_start_yaw
        nonlocal episode_start_by, prev_roll, prev_pitch, last_action
        nonlocal next_perturb_t, prev_yaw_hold
        try:
            if translation_field is not None:
                translation_field.setSFVec3f(SPAWN_TRANS)
            if rotation_field is not None:
                rotation_field.setSFRotation(SPAWN_ROT)
            if hasattr(self_node, "resetPhysics"):
                self_node.resetPhysics()
        except Exception:
            pass
        # Settle the body before resuming.
        for _ in range(20):
            if robot.step(step_ms) == -1:
                break
        episode_step = 0
        sim_time = 0.0
        if math.isfinite(SPOT_FIXED_VX):
            vel_command[0] = SPOT_FIXED_VX
        else:
            vel_command[0] = 0.3 + 0.4 * np.random.random()  # [0.3, 0.7]
        vel_command[1] = 0.0
        vel_command[2] = 0.0  # straight-walk training only
        try:
            _o = self_node.getOrientation() or [1,0,0,0,1,0,0,0,1]
            _p = self_node.getPosition() or [0,0,0]
            episode_start_yaw = math.atan2(float(_o[3]), float(_o[0]))
            episode_start_by = float(_p[1])
        except Exception:
            episode_start_yaw = 0.0
            episode_start_by = 0.0
        prev_roll = 0.0
        prev_pitch = 0.0
        prev_yaw_hold = episode_start_yaw
        last_action = np.zeros(ACT_DIM, dtype=np.float32)
        next_perturb_t = PERTURB_START_S

    # Outer scope state
    sim_time = 0.0
    episode_step = 0
    vel_command = np.array([0.5, 0.0, 0.0], dtype=np.float32)
    episode_start_yaw = 0.0
    episode_start_by = 0.0
    prev_roll = 0.0
    prev_pitch = 0.0
    prev_yaw_hold = 0.0
    last_action = np.zeros(ACT_DIM, dtype=np.float32)
    next_perturb_t = PERTURB_START_S  # first perturbation time in episode

    # Initial settle
    for _ in range(20):
        if robot.step(step_ms) == -1:
            return 0
    reset_episode()

    CMD_BYTES = 1 + ACT_DIM * 4
    OBS_BYTES = OBS_DIM * 4 + 4 + 1

    while robot.step(step_ms) != -1:
        sim_time += step_dt
        episode_step += 1

        # ── Perturbation injection (training-time disturbance) ──
        # At the schedule, compute the impulse (mass × Δv) and convert
        # it to a constant force applied over PERTURB_FORCE_TICKS
        # consecutive ticks. Tick-by-tick force application gives the
        # policy exposure to the transient peak forces of a real cube
        # collision — far higher per-tick acceleration than a single-
        # tick velocity teleport.
        if PERTURB_ENABLED and sim_time >= next_perturb_t:
            try:
                dv = PERTURB_DV_MIN + (PERTURB_DV_MAX - PERTURB_DV_MIN) * np.random.random()
                angle = 2.0 * math.pi * np.random.random()
                # Impulse = mass × Δv. Force = impulse / duration.
                duration_s = max(1, PERTURB_FORCE_TICKS) * step_dt
                force_mag = (SPOT_MASS_KG * dv) / duration_s
                _active_force[0] = force_mag * math.cos(angle)
                _active_force[1] = force_mag * math.sin(angle)
                _active_force[2] = max(1, PERTURB_FORCE_TICKS)
            except Exception:
                pass
            next_perturb_t = sim_time + PERTURB_INTERVAL_S

        # Apply active force this tick if any.
        if _active_force[2] > 0:
            try:
                if hasattr(self_node, "addForce"):
                    self_node.addForce(
                        [float(_active_force[0]), float(_active_force[1]), 0.0],
                        False,  # absolute (world) frame
                    )
            except Exception:
                pass
            _active_force[2] -= 1

        # Read body state.
        try:
            pos = self_node.getPosition() or [0, 0, 0]
            ori = self_node.getOrientation() or [1,0,0,0,1,0,0,0,1]
            vel = self_node.getVelocity() or [0]*6
        except Exception:
            pos = [0,0,0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6
        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll = math.atan2(ori[7], ori[8])
        pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
        yaw = math.atan2(ori[3], ori[0])
        v_lin = np.array(vel[:3], dtype=np.float32)
        v_ang = np.array(vel[3:6], dtype=np.float32)
        proj_g = np.array([-ori[2], -ori[5], -ori[8]], dtype=np.float32)
        # Gait phase for FL leg (used as observation conditioning).
        phase = (sim_time / gait.period_s) % 1.0
        dyaw = _wrap_pi(yaw - episode_start_yaw)
        dlat = by - episode_start_by

        # Build obs.
        obs = np.zeros(OBS_DIM, dtype=np.float32)
        obs[0:3] = v_ang
        obs[3:6] = proj_g
        obs[6] = phase
        obs[7:10] = v_lin
        obs[10:13] = vel_command
        obs[13] = dyaw
        obs[14] = dlat
        obs[15:18] = last_action[:3]
        obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = np.clip(obs, -10.0, 10.0).astype(np.float32)

        # ── Reward ──
        # Forward vx tracking (sharp exp).
        vx_err = float(v_lin[0] - vel_command[0]) ** 2
        r_vx = R_VX_WT * math.exp(-vx_err / R_VX_SCALE)
        r_heading = R_HEADING_WT * (dyaw * dyaw)
        r_lateral = R_LATERAL_WT * (dlat * dlat)
        r_action = R_ACTION_WT * float(np.sum(last_action * last_action))
        # Upright bonus: sharp exp peak at level body. Strong gradient
        # toward leveling under perturbation — what's missing from the
        # flat alive bonus when external forces push roll/pitch off-zero.
        r_upright = 0.0
        if R_UPRIGHT_BONUS_WT != 0.0:
            r_upright = R_UPRIGHT_BONUS_WT * math.exp(
                -(roll * roll + pitch * pitch) / R_UPRIGHT_BONUS_SCALE)
        reward = r_vx + r_heading + r_lateral + r_action + r_upright + R_ALIVE

        # Termination.
        done = False
        if bz < BZ_FAIL or abs(roll) > ROLL_FAIL or abs(pitch) > PITCH_FAIL:
            done = True
            reward += R_TERM
        elif episode_step >= MAX_EPISODE_STEPS:
            done = True
        if not math.isfinite(reward):
            reward = -1.0; done = True
        reward = max(R_CLIP_LO, min(R_CLIP_HI, reward))

        # Send packet to trainer.
        pkt = obs.tobytes() + struct.pack("<f", float(reward)) + \
            struct.pack("<B", 1 if done else 0)
        try:
            conn.sendall(pkt)
        except (ConnectionError, OSError):
            sys.stderr.write("[spot_residual_agent] trainer disconnected\n")
            break

        # Read action.
        try:
            cmd = recv_exact(conn, CMD_BYTES)
        except (ConnectionError, OSError):
            break
        tag = cmd[0]
        if tag == 2:  # QUIT
            break
        if tag == 1 or done:  # RESET or auto-reset
            reset_episode()
            continue
        # tag == 0: ACTION
        action = np.frombuffer(cmd, dtype=np.float32, offset=1, count=ACT_DIM).copy()
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        last_action = action.copy()

        # ── Model walker output ──
        # Base-walker heading/centreline hold (parity with the deploy
        # controller; see the HOLD_* knobs above).
        vy_base = float(vel_command[1])
        wz_base = float(vel_command[2])
        if HOLD and abs(wz_base) < 1e-6:
            yaw_rate = _wrap_pi(yaw - prev_yaw_hold) / step_dt
            wz_corr = (-HOLD_KP_YAW * dyaw - HOLD_KD_YAW * yaw_rate
                       - HOLD_KP_LAT2YAW * dlat)
            wz_base = max(-HOLD_WZ_MAX, min(HOLD_WZ_MAX, wz_corr))
            vy_corr = -HOLD_KP_LAT * dlat
            vy_base += max(-HOLD_VY_MAX, min(HOLD_VY_MAX, vy_corr))
        prev_yaw_hold = yaw
        feet = foot_targets(sim_time, vx=float(vel_command[0]),
                            vy=vy_base, wz=wz_base, p=gait)
        # Balance offsets on z.
        roll_rate = (roll - prev_roll) / step_dt
        pitch_rate = (pitch - prev_pitch) / step_dt
        prev_roll, prev_pitch = roll, pitch
        bal = balance_offsets(roll, pitch, roll_rate, pitch_rate, balance)

        # Apply residual: per-leg foot offset (dx, dy, dz) scaled by RES_SCALE.
        for i, leg_ik in enumerate(("FL", "FR", "RL", "RR")):
            fx, fy, fz = feet[leg_ik]
            offset = action[i*3:(i+1)*3] * RES_SCALE
            fx += float(offset[0])
            fy += float(offset[1])
            fz += float(offset[2]) + bal[leg_ik]
            feet[leg_ik] = (fx, fy, fz)

        # IK + motor command.
        for i, (leg, joint) in enumerate(JOINT_ORDER):
            if joint != "hip_x":
                continue
            ik_leg = URDF_TO_IK[leg]
            q = inverse_kinematics(ik_leg, feet[ik_leg])
            if q is None:
                continue
            motors[i + 0].setPosition(float(q.hip_x))
            motors[i + 1].setPosition(float(q.hip_y) + PITCH_TRIM)
            motors[i + 2].setPosition(float(q.knee))

    try: conn.close()
    except Exception: pass
    try: srv.close()
    except Exception: pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
