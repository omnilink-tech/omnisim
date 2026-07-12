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

"""OmniSim deploy controller for the Atlas standing PPO policy.

Loads the ONNX policy trained by
`projects/policies/research/training/gpu_mjwarp_atlas_stand_trainer.py` and runs it
inside an OmniSim world. The observation/action layout must mirror the
training env exactly:

Obs (99-dim float32):
    pelvis_lin_vel(3) + pelvis_ang_vel(3) + proj_gravity(3) +
    joint_q_minus_nominal(30) + joint_qd(30) + last_action(30)

Action (30-dim, [-1,1]):
    Joint-space delta in radians, scaled by ACT_SCALE = 0.05 rad
    (matches RES_SCALE in the trainer), added to NOMINAL + analytic
    coupled ankle/hip/back balance PD.

The 30 joints are in the trainer's order (NOT the atlas_robot_spec
JOINT_NAMES order — see `gpu_mjwarp_atlas_stand_trainer.py`
ATLAS_JOINTS docstring). Reorder is fatal here: outputs land on the
wrong joints.

Env vars:
    ATLAS_POLICY_ONNX   path to policy.onnx (default: trained run path)
    ATLAS_BALANCE_FALLBACK=1
                        if ONNX missing/broken, fall back to NOMINAL + ankle PD
    ATLAS_DEPLOY_LOG    side-log path for chassis trace
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


# Trainer joint order — back, l_arm, neck, r_arm, l_leg, r_leg. Mirrors
# `gpu_mjwarp_atlas_stand_trainer.py::ATLAS_JOINTS` exactly.
ATLAS_JOINTS = (
    # Back (3)
    "back_bkz", "back_bky", "back_bkx",
    # Left arm (7)
    "l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx",
    "l_arm_uwy", "l_arm_mwx", "l_arm_lwy",
    # Neck (1)
    "neck_ay",
    # Right arm (7)
    "r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx",
    "r_arm_uwy", "r_arm_mwx", "r_arm_lwy",
    # Left leg (6)
    "l_leg_hpz", "l_leg_hpx", "l_leg_hpy",
    "l_leg_kny", "l_leg_aky", "l_leg_akx",
    # Right leg (6)
    "r_leg_hpz", "r_leg_hpx", "r_leg_hpy",
    "r_leg_kny", "r_leg_aky", "r_leg_akx",
)
NJ = 30
assert len(ATLAS_JOINTS) == NJ
OBS_DIM = 3 + 3 + 3 + NJ * 3   # = 99
ACT_SCALE = 0.05   # Match RES_SCALE in the trainer.

NOMINAL = np.array([
    +0.00, +0.00, +0.00,                                  # back
    +0.00, -0.30, +0.30, +0.10, +0.00, +0.00, +0.00,      # l_arm
    +0.00,                                                # neck
    +0.00, +0.30, +0.30, -0.10, +0.00, +0.00, +0.00,      # r_arm
    +0.00, +0.03, -0.10, +0.20, -0.10, -0.03,             # l_leg
    +0.00, -0.03, -0.10, +0.20, -0.10, +0.03,             # r_leg
], dtype=np.float32)
assert NOMINAL.shape == (NJ,)

# Analytic balance PD baseline — must match the trainer exactly.
# Coupled ankle + hip + back strategy with wide clamps so the kp=20
# MJCF actuators saturate to their effort limit (~360 N·m ankle) on
# meaningful tilts. See gpu_mjwarp_atlas_stand_trainer.py for the
# torque-budget math.
KP_ANKLE_PITCH = -20.0; KD_ANKLE_PITCH = -3.0
KP_ANKLE_ROLL  = -20.0; KD_ANKLE_ROLL  = -3.0
KP_HIP_PITCH   = -12.0; KD_HIP_PITCH   = -2.0
KP_HIP_ROLL    = -8.0;  KD_HIP_ROLL    = -1.2
KP_BACK_PITCH  = -8.0;  KD_BACK_PITCH  = -1.2
KP_BACK_ROLL   = -5.0;  KD_BACK_ROLL   = -0.8
ANKLE_CLAMP = 1.0
HIP_CLAMP   = 1.0
BACK_CLAMP  = 0.75
BAL_CLAMP = ANKLE_CLAMP   # back-compat alias

_L_AP = ATLAS_JOINTS.index("l_leg_aky")
_R_AP = ATLAS_JOINTS.index("r_leg_aky")
_L_AR = ATLAS_JOINTS.index("l_leg_akx")
_R_AR = ATLAS_JOINTS.index("r_leg_akx")
_L_HP = ATLAS_JOINTS.index("l_leg_hpy")
_R_HP = ATLAS_JOINTS.index("r_leg_hpy")
_L_HR = ATLAS_JOINTS.index("l_leg_hpx")
_R_HR = ATLAS_JOINTS.index("r_leg_hpx")
_BKY  = ATLAS_JOINTS.index("back_bky")
_BKX  = ATLAS_JOINTS.index("back_bkx")


def _baseline_targets(roll: float, pitch: float,
                      roll_rate: float, pitch_rate: float) -> np.ndarray:
    ap = float(np.clip(KP_ANKLE_PITCH * pitch + KD_ANKLE_PITCH * pitch_rate,
                       -ANKLE_CLAMP, ANKLE_CLAMP))
    ar = float(np.clip(KP_ANKLE_ROLL * roll + KD_ANKLE_ROLL * roll_rate,
                       -ANKLE_CLAMP, ANKLE_CLAMP))
    hp = float(np.clip(KP_HIP_PITCH * pitch + KD_HIP_PITCH * pitch_rate,
                       -HIP_CLAMP, HIP_CLAMP))
    hr = float(np.clip(KP_HIP_ROLL * roll + KD_HIP_ROLL * roll_rate,
                       -HIP_CLAMP, HIP_CLAMP))
    bp = float(np.clip(KP_BACK_PITCH * pitch + KD_BACK_PITCH * pitch_rate,
                       -BACK_CLAMP, BACK_CLAMP))
    br = float(np.clip(KP_BACK_ROLL * roll + KD_BACK_ROLL * roll_rate,
                       -BACK_CLAMP, BACK_CLAMP))
    targets = NOMINAL.copy()
    targets[_L_AP] += ap;  targets[_R_AP] += ap
    targets[_L_AR] += ar;  targets[_R_AR] += ar
    targets[_L_HP] += hp;  targets[_R_HP] += hp
    targets[_L_HR] += hr;  targets[_R_HR] += hr
    targets[_BKY]  += bp;  targets[_BKX]  += br
    return targets


def _ori_matrix_to_rpy(o):
    return (math.atan2(o[7], o[8]),
            math.asin(max(-1.0, min(1.0, -o[6]))),
            math.atan2(o[3], o[0]))


def _proj_gravity_from_ori(o):
    """Body-frame gravity from getOrientation() row-major R."""
    return (-float(o[6]), -float(o[7]), -float(o[8]))


def find_policy_path() -> Path:
    p = os.environ.get("ATLAS_POLICY_ONNX") or os.environ.get(
        "OMNISIM_POLICY_ONNX")
    if p:
        return Path(p)
    for cand in (
        _REPO / "projects/policies/research/training/runs/gpu_atlas_stand_robust/policy.onnx",
        _REPO / "projects/policies/research/training/runs/gpu_atlas_stand/policy.onnx",
    ):
        if cand.exists():
            return cand
    return _REPO / "projects/policies/research/training/runs/gpu_atlas_stand_robust/policy.onnx"


def main() -> int:
    side_log_path = os.environ.get("ATLAS_DEPLOY_LOG") or os.environ.get(
        "OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg: str) -> None:
        sys.stderr.write(msg)
        sys.stderr.flush()
        if side_log is not None:
            side_log.write(msg)
            side_log.flush()

    sess = None
    policy_path = find_policy_path()
    if policy_path.exists():
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(str(policy_path),
                                        providers=["CPUExecutionProvider"])
            say(f"[atlas_stand_deploy] loaded ONNX {policy_path.name}\n")
        except Exception as e:
            say(f"[atlas_stand_deploy] ONNX load failed ({e}); falling back\n")
    else:
        say(f"[atlas_stand_deploy] no ONNX at {policy_path}; falling back\n")

    fallback = (sess is None) or (
        os.environ.get("ATLAS_BALANCE_FALLBACK", "0").strip() != "0")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = {}
    sensors = {}
    for jn in ATLAS_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[atlas_stand_deploy] missing motor {jn}_motor\n")
            return 1
        motors[jn] = m
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors[jn] = s
        except Exception:
            sensors[jn] = None

    for jn, q in zip(ATLAS_JOINTS, NOMINAL):
        motors[jn].setPosition(float(q))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    last_action = np.zeros(NJ, dtype=np.float32)
    last_log_ms = 0
    survived = 0
    fell_at = None
    sim_ms = 0

    prev_joint_q = NOMINAL.copy()
    qd_alpha = max(0.05, min(1.0, step_dt / (step_dt + 0.030)))
    joint_qd_smoothed = np.zeros(NJ, dtype=np.float32)

    # 0.5 s settle at nominal before the policy takes over.
    for _ in range(max(1, int(0.5 / step_dt))):
        if robot.step(step_ms) == -1:
            return 0
        sim_ms += step_ms
    for i, jn in enumerate(ATLAS_JOINTS):
        s = sensors.get(jn)
        if s is not None:
            try:
                prev_joint_q[i] = float(s.getValue())
            except Exception:
                pass

    while robot.step(step_ms) != -1:
        sim_ms += step_ms

        if self_node is None:
            continue

        try:
            pos = self_node.getPosition() or [0.0, 0.0, 0.0]
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
        except Exception:
            continue

        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll, pitch, yaw = _ori_matrix_to_rpy(ori)

        try:
            vel6 = self_node.getVelocity() or [0.0] * 6
        except Exception:
            vel6 = [0.0] * 6
        lin_vel = np.array(vel6[0:3], dtype=np.float32)
        ang_vel = np.array(vel6[3:6], dtype=np.float32)

        joint_q = np.zeros(NJ, dtype=np.float32)
        for i, jn in enumerate(ATLAS_JOINTS):
            s = sensors.get(jn)
            if s is not None:
                try:
                    joint_q[i] = float(s.getValue())
                except Exception:
                    joint_q[i] = prev_joint_q[i]
            else:
                joint_q[i] = prev_joint_q[i]
        joint_qd_raw = (joint_q - prev_joint_q) / max(step_dt, 1e-6)
        joint_qd_smoothed = (qd_alpha * joint_qd_raw
                             + (1.0 - qd_alpha) * joint_qd_smoothed)
        joint_qd = joint_qd_smoothed.astype(np.float32)
        prev_joint_q = joint_q.copy()

        proj_g = np.array(_proj_gravity_from_ori(ori), dtype=np.float32)

        baseline = _baseline_targets(roll, pitch, ang_vel[0], ang_vel[1])
        if fallback:
            targets = baseline
            action = np.zeros(NJ, dtype=np.float32)
        else:
            obs = np.concatenate([
                lin_vel, ang_vel, proj_g,
                joint_q - NOMINAL, joint_qd, last_action,
            ]).astype(np.float32)
            assert obs.shape[0] == OBS_DIM, f"obs shape {obs.shape} != {OBS_DIM}"
            try:
                out = sess.run(None, {"obs": obs[None, :]})
                action = np.clip(out[0][0], -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[atlas_stand_deploy] ONNX inference failed: {e}; falling back\n")
                fallback = True
                action = np.zeros(NJ, dtype=np.float32)
            targets = baseline + ACT_SCALE * action

        for i, jn in enumerate(ATLAS_JOINTS):
            motors[jn].setPosition(float(targets[i]))

        last_action = action

        # Atlas pelvis sits ~0.92 m off the ground when standing; threshold
        # below that says it's dropped to a sitting / collapsed pose.
        upright = (bz > 0.55 and abs(roll) < 0.8 and abs(pitch) < 0.8)
        if upright:
            survived += 1
        elif fell_at is None:
            fell_at = sim_ms / 1000.0

        if sim_ms - last_log_ms >= 1000:
            tag = "OK" if upright else f"FALL@{fell_at:.2f}s"
            say(f"[atlas_stand_deploy] t={sim_ms/1000:5.1f}s "
                f"bx={bx:+.2f} bz={bz:.3f} roll={roll:+.3f} "
                f"pitch={pitch:+.3f}  {tag}  "
                f"{'ONNX' if not fallback else 'PD'}\n")
            last_log_ms = sim_ms

    return 0


if __name__ == "__main__":
    sys.exit(main())
