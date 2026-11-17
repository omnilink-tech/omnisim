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

"""Quick standing sweep — try different nominal-pose variants to find
one G1 can actually hold under static PD.

Reads G1_TEST_HIP_PITCH / G1_TEST_KNEE / G1_TEST_ANKLE_PITCH env vars
(defaults match the spec). Just commands that fixed pose and reports
roll/pitch/bz every second.
"""
from __future__ import annotations
import math, os, sys
from pathlib import Path

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.research.backends.g1_robot_spec import (
    JOINT_NAMES, NOMINAL_POSE, JOINT_LIMITS_LO, JOINT_LIMITS_HI,
    MOTOR_PID_PER_JOINT,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


def _envf(k, d):
    v = os.environ.get(k)
    if v is None or v.strip() == "":
        return d
    try: return float(v)
    except ValueError: return d


def main():
    sys.stderr.write("[g1_pose_sweep] starting\n")
    sys.stderr.flush()

    hip_pitch = _envf("G1_TEST_HIP_PITCH", -0.20)
    knee = _envf("G1_TEST_KNEE", 0.42)
    ankle_pitch = _envf("G1_TEST_ANKLE_PITCH", -0.23)
    arm_zero = (os.environ.get("G1_TEST_ARMS_ZERO", "0").strip() != "0")

    sys.stderr.write(f"[g1_pose_sweep] hip_pitch={hip_pitch} knee={knee} "
                     f"ankle_pitch={ankle_pitch} arms_zero={arm_zero}\n")
    sys.stderr.flush()

    nominal = dict(zip(JOINT_NAMES, NOMINAL_POSE))
    nominal["left_hip_pitch_joint"] = hip_pitch
    nominal["right_hip_pitch_joint"] = hip_pitch
    nominal["left_knee_joint"] = knee
    nominal["right_knee_joint"] = knee
    nominal["left_ankle_pitch_joint"] = ankle_pitch
    nominal["right_ankle_pitch_joint"] = ankle_pitch
    if arm_zero:
        for j in JOINT_NAMES:
            if "shoulder" in j or "elbow" in j or "wrist" in j:
                nominal[j] = 0.0

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    motors = {}
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            sys.stderr.write(f"missing motor {jn}\n"); return 1
        motors[jn] = m
    pid_by_name = dict(zip(JOINT_NAMES, MOTOR_PID_PER_JOINT))
    for jn, m in motors.items():
        kp, ki, kd = pid_by_name[jn]
        try: m.setControlPID(kp, ki, kd)
        except Exception: pass
    for jn, q in nominal.items():
        motors[jn].setPosition(float(q))

    self_node = None
    try: self_node = robot.getSelf()
    except Exception: pass

    sim_ms = 0
    last_log = -1000
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        # Re-command nominal each tick.
        for jn, q in nominal.items():
            motors[jn].setPosition(float(q))
        if self_node is not None and sim_ms - last_log >= 200:
            try:
                pos = self_node.getPosition() or [0,0,0]
                ori = self_node.getOrientation() or [1,0,0,0,1,0,0,0,1]
                roll = math.atan2(ori[7], ori[8])
                pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
                tag = "OK" if (pos[2] > 0.50 and abs(roll) < 0.4 and abs(pitch) < 0.4) else "FALL"
                sys.stderr.write(f"[g1_pose_sweep] t={sim_ms/1000:.2f}s "
                                 f"bz={pos[2]:.3f} roll={roll:+.3f} "
                                 f"pitch={pitch:+.3f} {tag}\n")
                sys.stderr.flush()
            except Exception:
                pass
            last_log = sim_ms
    return 0


if __name__ == "__main__":
    sys.exit(main())
