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

"""Spot Raibert-trot controller — the MODELED gait base layer.

Drives the velocity-feedback trot from projects/policies/control/spot_raibert.py:
world-anchored stance feet, Raibert touchdown placement from the measured
body velocity, height/attitude regulation. This is the "model the gait so
it keeps going and doesn't fall" layer; the RL residual belongs on top.

Env knobs:
  SPOT_VX / SPOT_VY / SPOT_WZ      commanded body velocity (default 0.4/0/0)
  SPOT_RAIBERT_FREQ_HZ             trot cadence (default 2.0)
  SPOT_RAIBERT_H0                  body height over feet (default 0.52)
  SPOT_RAIBERT_STEP_H              swing apex (default 0.07)
  SPOT_RAIBERT_KV                  placement velocity gain (default derived)
  SPOT_HEADING_LOCK=1              hold spawn heading via wz placement
  SPOT_HEADING_KP / _CLIP / _LAT2YAW   lock gains (1.0 / 0.3 / 0.1)
  SPOT_DEPLOY_TRACE=1              chassis trace CSV for verify_straight_walk
  SPOT_DEPLOY_LOG=<path>           mirror stderr
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))  # lowest priority: don't shadow runtime `import omnisim`

from projects.policies.control.spot_kinematics import inverse_kinematics  # noqa: E402
from projects.policies.control.spot_raibert import (  # noqa: E402
    LEGS, RaibertGait, RaibertParams,
)
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
for _leg in URDF_LEGS:
    for _joint in ("hip_x", "hip_y", "knee"):
        JOINT_ORDER.append((_leg, _joint))


def _env_float(key: str, default: float) -> float:
    v = os.environ.get(key)
    if v is None or v.strip() == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def main() -> int:
    side_log_path = os.environ.get("SPOT_DEPLOY_LOG") or os.environ.get(
        "OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg):
        try:
            sys.stderr.write(msg); sys.stderr.flush()
        except Exception:
            pass
        if side_log is not None:
            try:
                side_log.write(msg); side_log.flush()
            except Exception:
                pass

    vx = _env_float("SPOT_VX", 0.4)
    vy = _env_float("SPOT_VY", 0.0)
    wz = _env_float("SPOT_WZ", 0.0)

    params = RaibertParams(
        freq_hz=_env_float("SPOT_RAIBERT_FREQ_HZ", 2.0),
        duty=_env_float("SPOT_RAIBERT_DUTY", 0.5),
        h0=_env_float("SPOT_RAIBERT_H0", 0.52),
        step_height=_env_float("SPOT_RAIBERT_STEP_H", 0.07),
        k_v=_env_float("SPOT_RAIBERT_KV", 0.0),
        k_height=_env_float("SPOT_RAIBERT_KH", 0.6),
        k_level=_env_float("SPOT_RAIBERT_KL", 0.5),
        lead_s=_env_float("SPOT_RAIBERT_LEAD", 0.12),
    )
    gait = RaibertGait(params)

    heading_lock = (os.environ.get("SPOT_HEADING_LOCK", "1").strip() != "0")
    heading_kp = _env_float("SPOT_HEADING_KP", 1.0)
    heading_clip = _env_float("SPOT_HEADING_CLIP", 0.3)
    heading_lat2yaw = _env_float("SPOT_HEADING_LAT2YAW", 0.1)

    say(f"[spot_raibert_walk] cmd vx={vx} vy={vy} wz={wz} "
        f"f={params.freq_hz}Hz h0={params.h0} step_h={params.step_height} "
        f"k_v={params.k_v:.3f} lock={'on' if heading_lock else 'off'}\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors, sensors = [], []
    for leg, joint in JOINT_ORDER:
        m = robot.getDevice(f"{leg}_{joint}_motor")
        if m is None:
            say(f"[spot_raibert_walk] missing motor {leg}_{joint}_motor\n")
            return 1
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    apply_realistic_limits(motors, max_torque_nm=80.0, max_vel_rad_s=20.0)
    motor_bank = RateLimitedMotorBank(
        motors, step_dt,
        max_vel_rad_s=_env_float("SPOT_TARGET_RATE_RAD_S", 1e6))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    # Settle at the gait's neutral stance.
    stand_q = [0.0] * len(motors)
    for i, (leg, joint) in enumerate(JOINT_ORDER):
        if joint != "hip_x":
            continue
        ik_leg = URDF_TO_IK[leg]
        q = inverse_kinematics(ik_leg, gait.neutral[ik_leg])
        if q is not None:
            stand_q[i + 0] = q.hip_x
            stand_q[i + 1] = q.hip_y
            stand_q[i + 2] = q.knee
    motor_bank.set_pose(stand_q)
    settle = max(1, int(_env_float("SPOT_SETTLE_S", 1.5) / step_dt))
    for _ in range(settle):
        if robot.step(step_ms) == -1:
            return 0

    trace_file = None
    if os.environ.get("SPOT_DEPLOY_TRACE") or os.environ.get(
            "OMNISIM_DEPLOY_TRACE"):
        tp = Path(r"C:\tmp\husky_trace\spot_deploy.csv")
        tp.parent.mkdir(parents=True, exist_ok=True)
        trace_file = open(tp, "w", buffering=1)
        trace_file.write("t_ms,bx,by,bz,roll,pitch,yaw,vx\n")
        say(f"[spot_raibert_walk] trace -> {tp}\n")

    main._prev_targets = None
    main._qd_filt = [0.0] * len(motors)
    # Start the gait clock at -lead so the lead-compensated phase begins at
    # exactly 0 (otherwise the first command teleports mid-swing).
    sim_t = -params.lead_s
    sim_ms = 0
    yaw_ref = None
    lat_ref = 0.0
    prev_pos = None
    last_bx = 0.0
    last_t_ms = 0

    while robot.step(step_ms) != -1:
        sim_t += step_dt
        sim_ms += step_ms

        pos = ori = vel = None
        if self_node is not None:
            try:
                pos = self_node.getPosition()
                ori = self_node.getOrientation()
                vel = self_node.getVelocity()
            except Exception:
                pass
        if pos is None or ori is None:
            continue
        if vel is None or all(abs(v) < 1e-12 for v in vel[:3]):
            # Finite-diff fallback if the velocity API is unavailable.
            if prev_pos is not None:
                vel = [(pos[k] - prev_pos[k]) / step_dt for k in range(3)] + [0, 0, 0]
            else:
                vel = [0.0] * 6
        prev_pos = list(pos)

        roll = math.atan2(ori[7], ori[8])
        pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
        yaw = math.atan2(ori[3], ori[0])

        if trace_file is not None:
            dt_s = (sim_ms - last_t_ms) / 1000.0
            vx_obs = (pos[0] - last_bx) / dt_s if dt_s > 1e-6 else 0.0
            trace_file.write(
                f"{sim_ms},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                f"{roll:.4f},{pitch:.4f},{yaw:.4f},{vx_obs:.4f}\n")
            last_bx = pos[0]
            last_t_ms = sim_ms

        wz_cmd = wz
        if heading_lock and abs(wz) < 1e-6:
            if yaw_ref is None:
                yaw_ref = yaw
                lat_ref = pos[1]
            dyaw = yaw - yaw_ref
            while dyaw > math.pi:
                dyaw -= 2 * math.pi
            while dyaw < -math.pi:
                dyaw += 2 * math.pi
            wz_cmd = -heading_kp * dyaw - heading_lat2yaw * (pos[1] - lat_ref)
            wz_cmd = max(-heading_clip, min(heading_clip, wz_cmd))

        feet, foot_vels = gait.targets(sim_t, ori, pos, vel[:3], roll, pitch,
                                       v_cmd=(vx, vy), wz_cmd=wz_cmd,
                                       return_vel=True)

        # Velocity-feedforward through the position channel. The servo force
        # is ke(q_cmd - q) + kd(0 - qd): the kd term is JOINT FRICTION that
        # rectifies the gait cycle into backward thrust (measured -1 m/s at
        # kd=200 stepping in place -- the "conveyor"). Commanding
        # q_cmd + tau*qd_des with tau = kd/ke is ALGEBRAICALLY IDENTICAL to
        # giving the servo a velocity target:
        #   ke(q_cmd + tau*qd_des - q) - kd*qd = ke(q_cmd - q) + kd(qd_des - qd)
        # qd_des comes from the gait's ANALYTIC foot velocities pushed
        # through the IK (closed form -> smooth; the earlier numerical
        # version spiked at swing transitions and kicked the robot over).
        # SPOT_RAIBERT_TAU must equal the deployed KD/KE ratio.
        # Stance and swing feedforward can be scaled independently: the
        # swing velocity is open-loop (curve derivative), but the stance
        # velocity is -v_body -- feeding it back at full gain closes a loop
        # through the body state and can destabilize.
        tau = _env_float("SPOT_RAIBERT_TAU", 0.12)
        tau_stance = _env_float("SPOT_RAIBERT_TAU_STANCE", tau)
        eps = 0.004  # seconds of foot motion for the IK directional diff

        targets = [0.0] * len(motors)
        cmd = [0.0] * len(motors)
        for i, leg in enumerate(URDF_LEGS):
            ik_leg = URDF_TO_IK[leg]
            ft = feet[ik_leg]
            q = inverse_kinematics(ik_leg, ft)
            if q is None:
                q = inverse_kinematics(ik_leg, gait.neutral[ik_leg])
                fv = (0.0, 0.0, 0.0)
            else:
                fv = foot_vels[ik_leg]
            if q is None:
                continue
            base = (q.hip_x, q.hip_y, q.knee)
            for k in range(3):
                targets[i * 3 + k] = base[k]
                cmd[i * 3 + k] = base[k]
            leg_tau = tau_stance if gait.legs[ik_leg].in_stance else tau
            if leg_tau > 0.0 and (fv[0] or fv[1] or fv[2]):
                q2 = inverse_kinematics(
                    ik_leg, (ft[0] + fv[0] * eps,
                             ft[1] + fv[1] * eps,
                             ft[2] + fv[2] * eps))
                if q2 is not None:
                    b2 = (q2.hip_x, q2.hip_y, q2.knee)
                    for k in range(3):
                        qd_des = (b2[k] - base[k]) / eps
                        cmd[i * 3 + k] = base[k] + leg_tau * qd_des
        motor_bank.set_pose(cmd)

    if trace_file is not None:
        trace_file.close()
    if side_log is not None:
        side_log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
