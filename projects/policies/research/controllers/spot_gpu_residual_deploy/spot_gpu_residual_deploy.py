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

"""Spot deploy controller for the GPU mjwarp residual policy.

Mirrors the env layer of `gpu_mjwarp_residual_trainer.py` so the
ONNX exported from that trainer can be run in OmniSim:

  * 49-dim observation (vlin, vang, proj_gravity, q[12], qd[12],
    last_action[12], vel_cmd[3], gait_clock[1]).
  * ±0.15 rad JOINT-SPACE residual on top of gait→IK joint
    targets — NOT the ±0.03 m foot-offset residual the
    `spot_residual_deploy` controller uses.
  * GaitParams tuned for the mjwarp MJCF: neutral_front_x=0.322,
    neutral_rear_x=-0.274, neutral_lateral_y=0.344,
    ground_z=-0.62, step_height=0.04.

ONNX path defaults to projects/policies/research/inference/policies/
gpu_spot_residual_main/policy.onnx.

This is a sim-to-sim deploy of a policy trained in MuJoCo onto
Webots-ODE physics. Contact model, actuator gains, and body
settle height all differ; the policy may not transfer cleanly.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.control.spot_kinematics import inverse_kinematics  # noqa: E402
from projects.policies.control.spot_gait import GaitParams, foot_targets  # noqa: E402
from projects.policies.control.spot_motor_safety import (  # noqa: E402
    apply_realistic_limits, RateLimitedMotorBank,
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

OBS_DIM = 49
ACT_DIM = 12
RES_SCALE = 0.15           # joint-space delta (radians)
NJ = 12

# Joint-target clamps matching the MJCF used in training.
JOINT_LIMITS_LO = np.array([
    -1.50, +0.00, -1.20,
    -1.50, +0.00, -1.20,
    -1.50, +0.00, -1.20,
    -1.50, +0.00, -1.20,
], dtype=np.float32)
JOINT_LIMITS_HI = np.array([
    +1.50, +0.60, -0.01,
    +1.50, +0.60, -0.01,
    +1.50, +0.60, -0.01,
    +1.50, +0.60, -0.01,
], dtype=np.float32)

# Nominal stand pose (controller order). Used when IK returns NaN.
NOMINAL_4x3 = np.array([
    [+0.30, +0.30, -0.60],
    [-0.30, +0.30, -0.60],
    [+0.30, +0.30, -0.60],
    [-0.30, +0.30, -0.60],
], dtype=np.float32)


def _env_float(key: str, default: float) -> float:
    v = os.environ.get(key)
    if v is None or v.strip() == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def find_policy_path() -> Path:
    p = os.environ.get("SPOT_POLICY_ONNX") or os.environ.get(
        "OMNISIM_POLICY_ONNX")
    if p:
        return Path(p)
    return (_REPO / "projects" / "rl" / "inference" / "policies"
            / "gpu_spot_residual_main" / "policy.onnx")


def main() -> int:
    side_log_path = os.environ.get("SPOT_DEPLOY_LOG") or os.environ.get(
        "OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg):
        try: sys.stderr.write(msg); sys.stderr.flush()
        except Exception: pass
        if side_log is not None:
            try: side_log.write(msg); side_log.flush()
            except Exception: pass

    policy_path = find_policy_path()
    say(f"[spot_gpu_residual_deploy] policy: {policy_path}\n")
    sess = None
    if policy_path.exists():
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(str(policy_path),
                                        providers=["CPUExecutionProvider"])
            say("[spot_gpu_residual_deploy] ONNX loaded\n")
        except Exception as e:
            say(f"[spot_gpu_residual_deploy] ONNX load failed ({e}); zero residual\n")
            sess = None
    else:
        say(f"[spot_gpu_residual_deploy] policy not found at {policy_path}; "
            "running model walker with zero residual\n")

    # Gait parameters -- MUST match the trainer's. ground_z=-0.57 keeps the
    # trot 100% IK-reachable across the cycle (the historic -0.62 exceeded
    # the legs' reach, silently collapsing the baseline to a constant
    # nominal pose). Env-overridable for recipe experiments.
    gait = GaitParams(
        neutral_front_x=_env_float("SPOT_GAIT_FRONT_X", 0.322),
        neutral_rear_x=_env_float("SPOT_GAIT_REAR_X", -0.274),
        neutral_lateral_y=_env_float("SPOT_GAIT_LATERAL_Y", 0.344),
        ground_z=_env_float("SPOT_GAIT_GROUND_Z", -0.57),
        step_height=_env_float("SPOT_GAIT_STEP_HEIGHT", 0.06))
    vx = _env_float("SPOT_VX", 0.5)
    vy = _env_float("SPOT_VY", 0.0)
    wz = _env_float("SPOT_WZ", 0.0)
    say(f"[spot_gpu_residual_deploy] command vx={vx} vy={vy} wz={wz}\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = []
    sensors = []
    for leg, joint in JOINT_ORDER:
        name = f"{leg}_{joint}_motor"
        m = robot.getDevice(name)
        if m is None:
            say(f"[spot_gpu_residual_deploy] missing motor {name}\n")
            return 1
        try:
            if hasattr(m, "setControlPID"):
                m.setControlPID(_env_float("SPOT_MOTOR_KP", 20.0), 0.0,
                                _env_float("SPOT_MOTOR_KD", 0.3))
        except Exception:
            pass
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None: s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    apply_realistic_limits(motors,
                           max_torque_nm=_env_float("SPOT_MAX_TORQUE_NM", 80.0),
                           max_vel_rad_s=_env_float("SPOT_MAX_VEL_RAD_S", 20.0))
    # Target slew default is OFF (1e6) for this controller: the mjwarp
    # trainer applies position targets INSTANTLY each control tick, so any
    # deploy-side rate limit is a filter the policy never trained with.
    # Re-enable with SPOT_TARGET_RATE_RAD_S for hardware-realism studies.
    motor_bank = RateLimitedMotorBank(
        motors, step_dt,
        max_vel_rad_s=_env_float("SPOT_TARGET_RATE_RAD_S", 1e6))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    # Stand pose = IK of the gait's neutral feet (== nominal joints
    # by construction of the GaitParams).
    _neutrals = {
        "FL": (+gait.neutral_front_x, +gait.neutral_lateral_y, gait.ground_z),
        "FR": (+gait.neutral_front_x, -gait.neutral_lateral_y, gait.ground_z),
        "RL": (+gait.neutral_rear_x,  +gait.neutral_lateral_y, gait.ground_z),
        "RR": (+gait.neutral_rear_x,  -gait.neutral_lateral_y, gait.ground_z),
    }
    # Settle pose: IK of the neutral stance, falling back to NOMINAL per
    # leg when IK is unreachable -- the SAME fallback the run loop uses.
    # The old code left _stand_q at 0.0 (straight legs) on IK failure, so
    # with the MJCF-tuned neutrals (unreachable at stance, like the
    # trainer's own batched IK) the robot settled standing TALL and then
    # CRASHED down into the crouch when the gait clock started -- a
    # transient the policy never trained on (the trainer resets settled
    # in the crouch at zero velocity).
    _stand_q = [0.0] * len(motors)
    for i, (leg, joint) in enumerate(JOINT_ORDER):
        if joint != "hip_x":
            continue
        ik_leg = URDF_TO_IK[leg]
        _q = inverse_kinematics(ik_leg, _neutrals[ik_leg])
        li = i // 3
        if _q is None:
            _stand_q[i + 0] = float(NOMINAL_4x3[li][0])
            _stand_q[i + 1] = float(NOMINAL_4x3[li][1])
            _stand_q[i + 2] = float(NOMINAL_4x3[li][2])
        else:
            _stand_q[i + 0] = _q.hip_x
            _stand_q[i + 1] = _q.hip_y
            _stand_q[i + 2] = _q.knee
    motor_bank.set_pose(_stand_q)

    trace_file = None
    if os.environ.get("SPOT_DEPLOY_TRACE") or os.environ.get(
            "OMNISIM_DEPLOY_TRACE"):
        trace_path = Path(r"C:\tmp\husky_trace\spot_gpu_deploy.csv")
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = open(trace_path, "w", buffering=1)
        trace_file.write("t_ms,bx,by,bz,roll,pitch,yaw,vx\n")
        say(f"[spot_gpu_residual_deploy] trace -> {trace_path}\n")

    # Settle so the robot drops to the floor and the bounce dies before the
    # gait clock starts -- the trainer resets every episode SETTLED (feet on
    # the floor, zero velocity, clock 0), so deploy must hand the policy the
    # same initial state. 1.5 s covers the spawn drop + ring-down.
    settle = max(1, int(_env_float("SPOT_SETTLE_S", 1.5) / step_dt))
    for _ in range(settle):
        if robot.step(step_ms) == -1:
            return 0

    sim_t = 0.0
    sim_ms = 0
    last_action = np.zeros(ACT_DIM, dtype=np.float32)
    vel_cmd = np.array([vx, vy, wz], dtype=np.float32)
    last_bx = 0.0
    last_t_ms = 0
    # Heading hold THROUGH THE COMMAND CHANNEL: policies trained with
    # --wz-range track vel_cmd[2], so deploy holds heading by writing the
    # yaw-rate command each tick (PD on yaw + steer-to-centreline). Only
    # active when the user command wz is 0. SPOT_HEADING_HOLD=0 disables.
    HOLD = (os.environ.get("SPOT_HEADING_HOLD", "1").strip() != "0")
    HOLD_KP_YAW = _env_float("SPOT_HOLD_KP_YAW", 1.0)
    HOLD_KP_LAT2YAW = _env_float("SPOT_HOLD_KP_LAT2YAW", 0.3)
    HOLD_WZ_MAX = _env_float("SPOT_HOLD_WZ_MAX", 0.35)
    yaw_ref = None
    lat_ref = 0.0

    while robot.step(step_ms) != -1:
        sim_t += step_dt
        sim_ms += step_ms

        if self_node is not None:
            try:
                pos = self_node.getPosition() or [0, 0, 0]
                ori = self_node.getOrientation() or [1,0,0,0,1,0,0,0,1]
                vel = self_node.getVelocity() or [0]*6
            except Exception:
                pos = [0,0,0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6
        else:
            pos = [0,0,0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6
        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll = math.atan2(ori[7], ori[8])
        pitch = math.asin(max(-1.0, min(1.0, -ori[6])))

        if HOLD and abs(wz) < 1e-6:
            _yaw_now = math.atan2(ori[3], ori[0])
            if yaw_ref is None:
                yaw_ref = _yaw_now
                lat_ref = by
            _dyaw = _yaw_now - yaw_ref
            while _dyaw > math.pi: _dyaw -= 2 * math.pi
            while _dyaw < -math.pi: _dyaw += 2 * math.pi
            _wz_cmd = -HOLD_KP_YAW * _dyaw - HOLD_KP_LAT2YAW * (by - lat_ref)
            vel_cmd[2] = max(-HOLD_WZ_MAX, min(HOLD_WZ_MAX, _wz_cmd))

        if trace_file is not None:
            dt_s = (sim_ms - last_t_ms) / 1000.0
            vx_obs = (bx - last_bx) / dt_s if dt_s > 1e-6 else 0.0
            yaw = math.atan2(ori[3], ori[0])
            trace_file.write(f"{sim_ms},{bx:.4f},{by:.4f},{bz:.4f},"
                             f"{roll:.4f},{pitch:.4f},{yaw:.4f},"
                             f"{vx_obs:.4f}\n")
            last_bx = bx
            last_t_ms = sim_ms

        # ── Build the 49-dim observation matching the trainer ──
        # Read joint positions and velocities from the sensors.
        q = np.zeros(NJ, dtype=np.float32)
        qd = np.zeros(NJ, dtype=np.float32)
        for i, s in enumerate(sensors):
            if s is None: continue
            try:
                q[i] = s.getValue()
            except Exception:
                q[i] = 0.0
        # Joint velocity by finite-diff (sensors don't expose qd direct).
        if not hasattr(main, "_last_q"):
            main._last_q = q.copy()
        qd = (q - main._last_q) / step_dt
        main._last_q = q.copy()

        v_lin = np.array(vel[:3], dtype=np.float32)
        # Trainer obs[3:6] is MuJoCo free-joint qvel angular = BODY-LOCAL
        # frame; Webots getVelocity() returns WORLD-frame angular velocity.
        # Rotate into the body frame (w_body = R^T @ w_world) -- feeding the
        # world frame here was a silent train/deploy mismatch (same class as
        # the G1 obs-frame bug).
        _w_world = np.array(vel[3:6], dtype=np.float32)
        _R = np.array(ori, dtype=np.float32).reshape(3, 3)
        v_ang = (_R.T @ _w_world).astype(np.float32)
        # Body-frame gravity unit vector: rotated world-z into body.
        # Webots orientation is a 3x3 rotation matrix (row-major flat).
        # World gravity is (0, 0, -1). In body frame:
        #   g_body = R^T @ g_world  -> third column of R (negated)
        proj_g = np.array([-ori[2], -ori[5], -ori[8]], dtype=np.float32)
        clock = np.float32((sim_t % gait.period_s) / gait.period_s)

        obs = np.concatenate([v_lin, v_ang, proj_g, q, qd,
                              last_action, vel_cmd, [clock]]
                             ).astype(np.float32)
        obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = np.clip(obs, -10.0, 10.0)
        if os.environ.get("SPOT_GPU_OBS_DEBUG") and sim_ms % 320 == 0:
            say(f"[obsdbg] t={sim_t:.2f} vlin={v_lin.round(3).tolist()} "
                f"vang={v_ang.round(3).tolist()} pg={proj_g.round(3).tolist()} "
                f"clock={float(clock):.2f} q0_3={q[:3].round(3).tolist()}\n")

        # ── Policy residual ──
        if sess is not None:
            try:
                action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
                action = np.clip(action, -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[spot_gpu_residual_deploy] inference failed: {e}\n")
                action = np.zeros(ACT_DIM, dtype=np.float32)
        else:
            action = np.zeros(ACT_DIM, dtype=np.float32)

        # ── Model layer: gait → IK → joint targets + joint residual ──
        feet = foot_targets(sim_t, vx=float(vel_cmd[0]),
                            vy=float(vel_cmd[1]),
                            wz=float(vel_cmd[2]), p=gait)
        q_target = np.zeros(NJ, dtype=np.float32)
        for i, leg_ik in enumerate(("FL", "FR", "RL", "RR")):
            q_ik = inverse_kinematics(leg_ik, feet[leg_ik])
            if q_ik is None:
                q_target[i*3:(i+1)*3] = NOMINAL_4x3[i]
            else:
                q_target[i*3 + 0] = q_ik.hip_x
                q_target[i*3 + 1] = q_ik.hip_y
                q_target[i*3 + 2] = q_ik.knee
        # Joint-space residual.
        q_cmd = q_target + action * RES_SCALE
        q_cmd = np.clip(q_cmd, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        last_action = action

        motor_bank.set_pose(q_cmd.tolist())

    if trace_file is not None: trace_file.close()
    if side_log is not None: side_log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
