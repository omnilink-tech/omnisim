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

"""G1 open-loop bipedal walker — NOMINAL_POSE + CPG, no policy.

Same role as spot_model_walk for Spot: applies the analytical
controller layer at zero RL to see whether the baseline alone can
stand and walk. If yes → residual RL goes on top. If only stand
works → need to add active balance feedback (ankle strategy).
If neither → spec/CPG tuning required first.

Per-tick output:
  target_q[j] = NOMINAL[j] + CPG[j](t)

where CPG[j](t) = amp[j] * sin(2π * (freq*t + phase[j])) for hip_pitch
and knee joints; arms get a small counter-rotating swing.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.research.backends.g1_robot_spec import (
    JOINT_NAMES, NOMINAL_POSE, JOINT_LIMITS_LO, JOINT_LIMITS_HI,
    MOTOR_PID_PER_JOINT, CPG_LEG_PHASE, G1,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


def _env_float(k, d):
    v = os.environ.get(k)
    if v is None or v.strip() == "":
        return d
    try:
        return float(v)
    except ValueError:
        return d


def main() -> int:
    side_log_path = os.environ.get("SPOT_DEPLOY_LOG") or os.environ.get(
        "OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg):
        sys.stderr.write(msg)
        sys.stderr.flush()
        if side_log is not None:
            side_log.write(msg)
            side_log.flush()

    # CPG knobs (spec defaults overridable for tuning).
    freq_hz = _env_float("G1_CPG_FREQ_HZ", G1.cpg_freq_hz)
    hip_y_amp = _env_float("G1_CPG_HIP_Y_AMP", G1.cpg_hip_y_amp)
    knee_amp = _env_float("G1_CPG_KNEE_AMP", G1.cpg_knee_amp)
    ankle_amp = _env_float("G1_CPG_ANKLE_AMP", 0.10)
    arm_amp = _env_float("G1_CPG_ARM_AMP", 0.15)

    # Active balance — engages if G1_BALANCE=1 (default 1).
    # Ankle pitch feedback: positive pelvis pitch (forward lean) → add
    # positive ankle_pitch correction (toes flatter, CoP forward).
    balance = (os.environ.get("G1_BALANCE", "1").strip() != "0")
    kp_pitch = _env_float("G1_BAL_KP_PITCH", 1.5)
    kd_pitch = _env_float("G1_BAL_KD_PITCH", 0.15)
    kp_roll = _env_float("G1_BAL_KP_ROLL", 1.5)
    kd_roll = _env_float("G1_BAL_KD_ROLL", 0.15)
    bal_clamp = _env_float("G1_BAL_CLAMP", 0.3)

    say(f"[g1_model_walk] CPG freq={freq_hz} Hz hip_amp={hip_y_amp} "
        f"knee_amp={knee_amp} ankle_amp={ankle_amp} arm_amp={arm_amp}\n")
    say(f"[g1_model_walk] balance={'on' if balance else 'off'} "
        f"kp_pitch={kp_pitch} kd_pitch={kd_pitch} "
        f"kp_roll={kp_roll} kd_roll={kd_roll}\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    legs_only = (os.environ.get("G1_LEGS_ONLY", "0").strip() != "0")
    motors = {}
    sensors = {}
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            if legs_only:
                # Arm/waist joints absent in the legs-only URDF — skip silently.
                continue
            say(f"[g1_model_walk] missing motor {jn}_motor\n")
            return 1
        motors[jn] = m
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors[jn] = s
        except Exception:
            sensors[jn] = None

    # Per-joint PID per spec.
    pid_by_name = dict(zip(JOINT_NAMES, MOTOR_PID_PER_JOINT))
    for jn, m in motors.items():
        kp, ki, kd = pid_by_name[jn]
        try:
            if hasattr(m, "setControlPID"):
                m.setControlPID(kp, ki, kd)
        except Exception:
            pass

    nominal_by_name = dict(zip(JOINT_NAMES, NOMINAL_POSE))
    lo_by_name = dict(zip(JOINT_NAMES, JOINT_LIMITS_LO))
    hi_by_name = dict(zip(JOINT_NAMES, JOINT_LIMITS_HI))
    phase_by_name = dict(zip(JOINT_NAMES, CPG_LEG_PHASE))

    # Apply nominal pose to seed.
    for jn, q in nominal_by_name.items():
        if jn in motors:
            motors[jn].setPosition(float(q))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    trace_file = None
    if os.environ.get("SPOT_DEPLOY_TRACE") or os.environ.get("OMNISIM_DEPLOY_TRACE"):
        trace_path = Path(r"C:\tmp\husky_trace\g1_deploy.csv")
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = open(trace_path, "w", buffering=1)
        trace_file.write("t_ms,bx,by,bz,roll,pitch,yaw,vx\n")
        say(f"[g1_model_walk] trace -> {trace_path}\n")

    sim_t = 0.0
    sim_ms = 0
    last_log_t = 0
    last_bx = 0.0
    last_t_ms = 0
    prev_roll = 0.0
    prev_pitch = 0.0
    survived_ticks = 0
    fell_at = None

    # 0.5s settle on nominal before starting the CPG clock.
    settle_steps = max(1, int(0.5 / step_dt))
    for _ in range(settle_steps):
        if robot.step(step_ms) == -1:
            return 0
        sim_ms += step_ms

    say(f"[g1_model_walk] settle done, CPG engaging at t={sim_ms/1000:.2f}s\n")

    while robot.step(step_ms) != -1:
        sim_t += step_dt
        sim_ms += step_ms

        bx = by = bz = 0.0
        roll = pitch = yaw = 0.0
        if self_node is not None:
            try:
                pos = self_node.getPosition() or [0, 0, 0]
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
                roll = math.atan2(ori[7], ori[8])
                pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
                yaw = math.atan2(ori[3], ori[0])
            except Exception:
                pass

        roll_rate = (roll - prev_roll) / step_dt if step_dt > 1e-6 else 0.0
        pitch_rate = (pitch - prev_pitch) / step_dt if step_dt > 1e-6 else 0.0
        prev_roll, prev_pitch = roll, pitch

        # Per-joint CPG offset on top of nominal.
        targets = dict(nominal_by_name)
        for jn in JOINT_NAMES:
            phase = phase_by_name[jn]
            t_norm = freq_hz * sim_t + phase
            s = math.sin(2.0 * math.pi * t_norm)
            if "hip_pitch" in jn:
                targets[jn] += hip_y_amp * s
            elif "knee" in jn:
                # Knee bends MORE during swing (positive only-ish);
                # bias to positive sin for the swing phase.
                targets[jn] += knee_amp * max(0.0, s)
            elif "ankle_pitch" in jn:
                # Compensate the hip swing so the foot stays roughly flat.
                targets[jn] -= ankle_amp * s
            elif "shoulder_pitch" in jn:
                # Arm swing counter-rotates leg.
                targets[jn] += arm_amp * s
            elif "elbow" in jn:
                targets[jn] += 0.5 * arm_amp * s

        # Active ankle balance: closes a loop from pelvis pitch/roll to
        # ankle pitch/roll. Without it, even Spot-class lighter bipedals
        # topple within ~1s because the CPG produces zero corrective
        # torque to keep the CoM over the support polygon.
        if balance:
            ap_corr = kp_pitch * pitch + kd_pitch * pitch_rate
            ap_corr = max(-bal_clamp, min(bal_clamp, ap_corr))
            ar_corr = kp_roll * roll + kd_roll * roll_rate
            ar_corr = max(-bal_clamp, min(bal_clamp, ar_corr))
            targets["left_ankle_pitch_joint"] += ap_corr
            targets["right_ankle_pitch_joint"] += ap_corr
            targets["left_ankle_roll_joint"] += ar_corr
            targets["right_ankle_roll_joint"] += ar_corr

        # Clamp to URDF limits + apply.
        for jn, q in targets.items():
            if jn not in motors:
                continue
            q_clamped = max(lo_by_name[jn], min(hi_by_name[jn], q))
            motors[jn].setPosition(float(q_clamped))

        # Health check.
        upright = (bz > 0.45 and abs(roll) < 0.6 and abs(pitch) < 0.6)
        if upright:
            survived_ticks += 1
        else:
            if fell_at is None:
                fell_at = sim_t

        if trace_file is not None:
            dt_s = (sim_ms - last_t_ms) / 1000.0
            vx_obs = (bx - last_bx) / dt_s if dt_s > 1e-6 else 0.0
            trace_file.write(f"{sim_ms},{bx:.4f},{by:.4f},{bz:.4f},"
                             f"{roll:.4f},{pitch:.4f},{yaw:.4f},{vx_obs:.4f}\n")
            last_bx = bx
            last_t_ms = sim_ms

        if sim_ms - last_log_t >= 1000:
            tag = "OK" if upright else f"FALL@{fell_at:.2f}"
            say(f"[g1_model_walk] t={sim_t:5.1f}s bx={bx:+.2f} bz={bz:.2f} "
                f"roll={roll:+.2f} pitch={pitch:+.2f}  {tag}\n")
            last_log_t = sim_ms

    return 0


if __name__ == "__main__":
    sys.exit(main())
