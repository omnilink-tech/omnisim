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

"""Atlas standing controller — holds NOMINAL_POSE under per-joint PID.

Pure sanity check, no policy: spawn Atlas in its nominal squat,
command every joint to the nominal angle, observe whether the
physics holds it up or collapses. If this can't stand, no walking
policy can.

Writes a chassis trace (same format as Spot) so verify_straight_walk
and analyze tools work against it too.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.research.backends.atlas_robot_spec import (
    ATLAS, JOINT_NAMES, NOMINAL_POSE, MOTOR_PID_PER_JOINT,
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

    say("[atlas_stand] starting — holds NOMINAL_POSE forever\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = []
    sensors = []
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[atlas_stand] missing motor {jn}_motor\n")
            return 1
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    # Per-joint PID per ATLAS spec.
    for m, (kp, ki, kd) in zip(motors, MOTOR_PID_PER_JOINT):
        try:
            if hasattr(m, "setControlPID"):
                m.setControlPID(kp, ki, kd)
        except Exception:
            pass

    # Command nominal pose every tick.
    for m, q in zip(motors, NOMINAL_POSE):
        m.setPosition(float(q))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    # Trace
    trace_file = None
    if os.environ.get("SPOT_DEPLOY_TRACE") or os.environ.get("OMNISIM_DEPLOY_TRACE"):
        trace_path = Path(r"C:\tmp\husky_trace\atlas_stand.csv")
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = open(trace_path, "w", buffering=1)
        trace_file.write("t_ms,bx,by,bz,roll,pitch,yaw,vx\n")
        say(f"[atlas_stand] trace -> {trace_path}\n")

    # Joint CSV
    joints_csv = None
    joints_csv_path = os.environ.get("ATLAS_DEBUG_JOINTS_CSV", "").strip()
    if joints_csv_path:
        joints_csv = open(joints_csv_path, "w", buffering=1)
        joints_csv.write("t_ms," + ",".join(f"q_{j}" for j in JOINT_NAMES) + "\n")
        say(f"[atlas_stand] joint CSV -> {joints_csv_path}\n")

    sim_ms = 0
    last_bx = 0.0
    last_t_ms = 0
    last_log_t = 0
    while robot.step(step_ms) != -1:
        sim_ms += step_ms

        # Re-command nominal each tick (in case anything resets targets).
        for m, q in zip(motors, NOMINAL_POSE):
            m.setPosition(float(q))

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

        if trace_file is not None:
            dt_s = (sim_ms - last_t_ms) / 1000.0
            vx = (bx - last_bx) / dt_s if dt_s > 1e-6 else 0.0
            trace_file.write(
                f"{sim_ms},{bx:.4f},{by:.4f},{bz:.4f},"
                f"{roll:.4f},{pitch:.4f},{yaw:.4f},{vx:.4f}\n")
            last_bx = bx
            last_t_ms = sim_ms

        if joints_csv is not None and sensors:
            try:
                qs = [float(s.getValue()) if s else 0.0 for s in sensors]
                joints_csv.write(f"{sim_ms}," + ",".join(f"{q:.5f}" for q in qs) + "\n")
            except Exception:
                pass

        # Stderr summary every sim-second.
        if sim_ms - last_log_t >= 1000:
            tag = "OK" if (bz > 0.4 and abs(roll) < 0.5 and abs(pitch) < 0.5) else "FALL"
            say(f"[atlas_stand] t={sim_ms/1000:5.1f}s bz={bz:.2f} "
                f"roll={roll:+.2f} pitch={pitch:+.2f}  {tag}\n")
            last_log_t = sim_ms

    return 0


if __name__ == "__main__":
    sys.exit(main())
