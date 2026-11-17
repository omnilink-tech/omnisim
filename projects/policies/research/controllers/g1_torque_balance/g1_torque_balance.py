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

"""G1 TORQUE-MODE balance controller (Newton deploy).

Tests the new joint-torque path (control.joint_f via motor.setTorque) end to
end through the rebuilt binary, AND attacks the root cause we proved: under
SolverMuJoCo NO position-mode controller can hold the G1 (bare PD, policies,
+aggressive-DR all collapse ~1.06s). Position mode is stuck between "too soft
(ke=100 sags 0.78->0.50 then tips)" and "too stiff (ke=300 NaN-crashes the
solver)". Torque mode escapes that: we compute joint torques ourselves and clamp
them, so we can use arbitrarily high effective stiffness without exciting the
contact solver.

Control law (per joint, every tick):
    tau = KP*(nom - q) - KD*qd + KI*int(nom - q)          # torque-space PID hold
plus an ANKLE CoP balance overlay (the classic stand strategy):
    ankle_pitch += -KAP*pitch - KADP*pitch_rate
    ankle_roll  += -KAR*roll  - KADR*roll_rate
The KI term counters gravity (no steady-state sag); the ankle overlay regulates
the divergent lean. Everything is env-tunable for sweeping.

REQUIRES the binary built with the joint-torque sink + launch with
OMNISIM_NEWTON_TORQUE_MODE=1 (joints built in EFFORT mode so no PD fights us).
Run via g1_torque_balance.omniworld. Supervisor reads pelvis pose for the survival
metric. G1_TQ_LOG=<path> for telemetry.
"""
from __future__ import annotations

import math
import os
import sys

try:
    from omnisim import Supervisor as _Robot
    _IS_SUP = True
except Exception:
    from omnisim import Robot as _Robot
    _IS_SUP = False

LEGS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
# Indices into LEGS for the balance overlay.
L_HP, L_HR, L_HY, L_KN, L_AP, L_AR = 0, 1, 2, 3, 4, 5
R_HP, R_HR, R_HY, R_KN, R_AP, R_AR = 6, 7, 8, 9, 10, 11


def _env(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def main() -> int:
    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    dt = step_ms / 1000.0

    # Deep squat nominal (env-tunable). Foot kept ~flat via ankle_pitch.
    hip = _env("G1_TQ_HIP", -0.30)
    knee = _env("G1_TQ_KNEE", 0.60)
    ank = _env("G1_TQ_ANKLE", -0.30)
    nominal = [0.0] * 13
    for base in (0, 6):
        nominal[base + 0] = hip      # hip_pitch
        nominal[base + 3] = knee     # knee
        nominal[base + 4] = ank      # ankle_pitch

    # Torque-space PID gains (per joint family). Stiff is fine -- we clamp.
    KP = _env("G1_TQ_KP", 200.0)
    KD = _env("G1_TQ_KD", 4.0)
    KI = _env("G1_TQ_KI", 60.0)
    I_CLAMP = _env("G1_TQ_ICLAMP", 30.0)      # anti-windup on the integral torque
    # Ankle CoP balance overlay.
    KAP = _env("G1_TQ_ANK_KP", 120.0)
    KADP = _env("G1_TQ_ANK_KD", 10.0)
    KAR = _env("G1_TQ_ANK_KR", 120.0)
    KADR = _env("G1_TQ_ANK_KDR", 10.0)
    # Hip lean overlay (engages for larger tilts; counters pitch via hip too).
    KHP = _env("G1_TQ_HIP_KP", 0.0)
    # Per-joint torque clamp = URDF effort limits. EFFORT mode does NOT enforce
    # them, so a flat clamp over-torques the weak 35 Nm ankle -> violent NaN.
    LIMITS = [88.0, 88.0, 88.0, 139.0, 35.0, 35.0,
              88.0, 88.0, 88.0, 139.0, 35.0, 35.0, 88.0]
    # Settle ramp: blend the target from the SPAWN pose to the squat over RAMP_S
    # so we don't yank straight legs into a deep squat (the violent fold).
    RAMP_S = _env("G1_TQ_RAMP_S", 2.5)

    motors, sensors = [], []
    for jn in LEGS:
        m = robot.getDevice(f"{jn}_motor")
        motors.append(m)
        s = m.getPositionSensor() if m is not None else None
        if s is not None:
            s.enable(step_ms)
        sensors.append(s)
    nfound = sum(1 for m in motors if m is not None)
    sys.stderr.write(f"[g1_torque_balance] {nfound}/13 motors; KP={KP} KD={KD} "
                     f"KI={KI} ANK_KP={KAP} squat=({hip},{knee},{ank})\n")
    sys.stderr.flush()

    self_node = robot.getSelf() if _IS_SUP else None
    log_path = os.environ.get("G1_TQ_LOG")
    log = open(log_path, "w", buffering=1) if log_path else None

    q = [0.0] * 13
    prev_q = [None] * 13
    integ = [0.0] * 13
    prev_roll = prev_pitch = 0.0
    sim_ms = 0
    last_log = -10000
    fell_at = None
    spawn_q = None

    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        t = sim_ms / 1000.0
        # Joint states.
        for i, s in enumerate(sensors):
            if s is not None:
                try:
                    q[i] = float(s.getValue())
                except Exception:
                    pass
        qd = [0.0] * 13
        for i in range(13):
            if prev_q[i] is not None:
                qd[i] = (q[i] - prev_q[i]) / dt
            prev_q[i] = q[i]

        # Pelvis orientation (roll/pitch) for the balance overlay + survival.
        roll = pitch = 0.0
        bz = 0.78
        if self_node is not None:
            try:
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                pos = self_node.getPosition() or [0, 0, 0.78]
                roll = math.atan2(ori[7], ori[8])
                pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
                bz = float(pos[2])
            except Exception:
                pass
        roll_rate = (roll - prev_roll) / dt
        pitch_rate = (pitch - prev_pitch) / dt
        prev_roll, prev_pitch = roll, pitch

        # Ramp the target from the spawn pose to the squat over RAMP_S.
        if spawn_q is None:
            spawn_q = list(q)
        a = min(1.0, t / RAMP_S) if RAMP_S > 0 else 1.0
        target = [spawn_q[i] + a * (nominal[i] - spawn_q[i]) for i in range(13)]
        # Per-joint torque-space PID hold.
        tau = [0.0] * 13
        for i in range(13):
            e = target[i] - q[i]
            integ[i] = max(-I_CLAMP, min(I_CLAMP, integ[i] + KI * e * dt))
            tau[i] = KP * e - KD * qd[i] + integ[i]

        # Ankle CoP overlay (push to drive roll/pitch -> 0).
        ap_cmd = -KAP * pitch - KADP * pitch_rate
        ar_cmd = -KAR * roll - KADR * roll_rate
        tau[L_AP] += ap_cmd; tau[R_AP] += ap_cmd
        tau[L_AR] += ar_cmd; tau[R_AR] += ar_cmd
        # Hip lean overlay (optional).
        if KHP != 0.0:
            hp_cmd = -KHP * pitch
            tau[L_HP] += hp_cmd; tau[R_HP] += hp_cmd

        for i, m in enumerate(motors):
            if m is not None:
                lim = LIMITS[i]
                m.setTorque(max(-lim, min(lim, tau[i])))

        upright = bz > 0.45 and abs(roll) < 0.8 and abs(pitch) < 0.8
        if not upright and fell_at is None:
            fell_at = t
        if log is not None and sim_ms - last_log >= 500:
            last_log = sim_ms
            status = "OK" if upright else f"FALL@{fell_at:.2f}s"
            log.write(f"t={t:5.2f} bz={bz:.3f} roll={roll:+.3f} pitch={pitch:+.3f} "
                      f"{status} | L_knee q={q[L_KN]:+.3f} tau={tau[L_KN]:+.1f} | "
                      f"L_ankP q={q[L_AP]:+.3f} tau={tau[L_AP]:+.1f}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
