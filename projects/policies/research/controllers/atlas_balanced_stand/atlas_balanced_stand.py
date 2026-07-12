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

"""Atlas active-balance standing controller — closes a feedback loop
from pelvis orientation to ankle pitch/roll so Atlas can hold an
upright squat against gravity.

Static PD on a fixed pose (atlas_stand.py) doesn't work — Atlas is an
inverted pendulum at CoM height ≈1m on feet ≈10×25cm. The CoM crosses
outside the support polygon faster than the body can recover.

Active balance (ankle strategy):

  ankle_pitch_correction = kp_pitch * pitch + kd_pitch * pitch_rate
  ankle_roll_correction  = kp_roll  * roll  + kd_roll  * roll_rate

  l_leg_aky_target = NOMINAL_aky + ankle_pitch_correction
  r_leg_aky_target = NOMINAL_aky + ankle_pitch_correction
  l_leg_akx_target = NOMINAL_l_akx - ankle_roll_correction   # mirrored
  r_leg_akx_target = NOMINAL_r_akx + ankle_roll_correction

Hip strategy fallback (engaged when ankle saturates) leans the torso
back via hip_y to push the pelvis CoM forward over the feet:

  if |pitch| > 0.2:
      hip_y_correction = kp_hip * (pitch - 0.0)
      l/r_leg_hpy_target = NOMINAL_hpy + hip_y_correction
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.research.backends.atlas_robot_spec import (
    ATLAS, JOINT_NAMES, NOMINAL_POSE, MOTOR_PID_PER_JOINT, JOINT_LIMITS_LO,
    JOINT_LIMITS_HI,
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

    # Balance gains (overridable via env for sweep).
    KP_PITCH = _env_float("ATLAS_BAL_KP_PITCH", 8.0)
    KD_PITCH = _env_float("ATLAS_BAL_KD_PITCH", 1.5)
    KP_ROLL = _env_float("ATLAS_BAL_KP_ROLL", 8.0)
    KD_ROLL = _env_float("ATLAS_BAL_KD_ROLL", 1.5)
    KP_HIP_PITCH = _env_float("ATLAS_BAL_KP_HIP_PITCH", 4.0)
    HIP_ENGAGE_THRESH = _env_float("ATLAS_BAL_HIP_ENGAGE", 0.15)
    ANKLE_CLAMP = _env_float("ATLAS_BAL_ANKLE_CLAMP", 0.5)  # rad
    HIP_CLAMP = _env_float("ATLAS_BAL_HIP_CLAMP", 0.5)  # rad

    say(f"[atlas_balanced_stand] kp_pitch={KP_PITCH} kd_pitch={KD_PITCH} "
        f"kp_roll={KP_ROLL} kd_roll={KD_ROLL}  "
        f"hip_engage_at_pitch={HIP_ENGAGE_THRESH} kp_hip={KP_HIP_PITCH}\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = {}
    sensors = {}
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[atlas_balanced_stand] missing motor {jn}_motor\n")
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

    # Initial command: nominal pose.
    nominal_by_name = dict(zip(JOINT_NAMES, NOMINAL_POSE))
    lo_by_name = dict(zip(JOINT_NAMES, JOINT_LIMITS_LO))
    hi_by_name = dict(zip(JOINT_NAMES, JOINT_LIMITS_HI))
    for jn, q in nominal_by_name.items():
        motors[jn].setPosition(float(q))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    trace_file = None
    if os.environ.get("SPOT_DEPLOY_TRACE") or os.environ.get("OMNISIM_DEPLOY_TRACE"):
        trace_path = Path(r"C:\tmp\husky_trace\atlas_stand.csv")
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = open(trace_path, "w", buffering=1)
        trace_file.write("t_ms,bx,by,bz,roll,pitch,yaw,vx\n")

    sim_ms = 0
    last_log_t = 0
    prev_roll = 0.0
    prev_pitch = 0.0
    survived_ticks = 0
    falls_at = None

    while robot.step(step_ms) != -1:
        sim_ms += step_dt * 1000

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

        # Finite-difference rates.
        roll_rate = (roll - prev_roll) / step_dt
        pitch_rate = (pitch - prev_pitch) / step_dt
        prev_roll, prev_pitch = roll, pitch

        # Build the target pose: start from nominal, then add balance
        # corrections to ankle pitch + ankle roll. Optionally engage
        # the hip pitch when ankles saturate.
        targets = dict(nominal_by_name)

        # Ankle pitch feedback. Sign convention: positive pitch =
        # forward lean → we want to shift the contact pressure forward
        # (toes), which means aky MORE POSITIVE (foot points more flat,
        # forward CoP shift). Add corr (don't subtract).
        ank_pitch_corr = KP_PITCH * pitch + KD_PITCH * pitch_rate
        ank_pitch_corr = max(-ANKLE_CLAMP, min(ANKLE_CLAMP, ank_pitch_corr))
        targets["l_leg_aky"] = nominal_by_name["l_leg_aky"] + ank_pitch_corr
        targets["r_leg_aky"] = nominal_by_name["r_leg_aky"] + ank_pitch_corr

        # Ankle roll feedback — positive roll = leaning right, so we
        # want both ankles rolled toward the right (akx more positive)
        # to shift contact pressure rightward and rotate body back.
        ank_roll_corr = KP_ROLL * roll + KD_ROLL * roll_rate
        ank_roll_corr = max(-ANKLE_CLAMP, min(ANKLE_CLAMP, ank_roll_corr))
        targets["l_leg_akx"] = nominal_by_name["l_leg_akx"] + ank_roll_corr
        targets["r_leg_akx"] = nominal_by_name["r_leg_akx"] + ank_roll_corr

        # Hip strategy — engages when pitch exceeds the threshold.
        if abs(pitch) > HIP_ENGAGE_THRESH:
            hip_corr = KP_HIP_PITCH * (pitch - 0.0)
            hip_corr = max(-HIP_CLAMP, min(HIP_CLAMP, hip_corr))
            targets["l_leg_hpy"] = nominal_by_name["l_leg_hpy"] - hip_corr
            targets["r_leg_hpy"] = nominal_by_name["r_leg_hpy"] - hip_corr

        # Clamp + apply.
        for jn, q in targets.items():
            q_clamped = max(lo_by_name[jn], min(hi_by_name[jn], q))
            motors[jn].setPosition(float(q_clamped))

        upright = (bz > 0.50 and abs(roll) < 0.4 and abs(pitch) < 0.4)
        if upright:
            survived_ticks += 1
        else:
            if falls_at is None:
                falls_at = sim_ms / 1000.0

        if trace_file is not None:
            trace_file.write(
                f"{int(sim_ms)},{bx:.4f},{by:.4f},{bz:.4f},"
                f"{roll:.4f},{pitch:.4f},{yaw:.4f},0.0\n")

        if sim_ms - last_log_t >= 1000:
            tag = "OK" if upright else f"FALL@{falls_at}"
            say(f"[atlas_balanced_stand] t={sim_ms/1000:5.1f}s "
                f"bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f} "
                f"corr_ay={ank_pitch_corr:+.2f} ax={ank_roll_corr:+.2f}  {tag}\n")
            last_log_t = sim_ms

    say(f"[atlas_balanced_stand] survived_ticks={survived_ticks} "
        f"falls_at={falls_at}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
