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

"""Unitree G1 (23-DOF variant) RobotSpec.

URDF source: github.com/unitreerobotics/unitree_ros, robots/g1_description
(file: g1_23dof.urdf). The 23-DOF variant has:
  * 6-DOF per leg (hip pitch/roll/yaw, knee, ankle pitch/roll)
  * 1 waist yaw
  * 5-DOF per arm (shoulder pitch/roll/yaw, elbow, wrist roll)
  * No grippers / finger DOFs.

Mass: 34.1 kg (vs OmniQuad 32 kg, Atlas 175 kg). G1 is intentionally in the
same weight class as our existing model+residual recipe targets — much
more tractable for the analytical-baseline + residual approach than
Atlas, where the small feet / high CoM ratio dominates.

Standing pose is a slight squat: hip_pitch + knee + ankle_pitch in a
coordinated bend so the CoM sits over the feet. Arms hang at relaxed
positions matching the rest pose in Unitree's reference MJX env.

Joint order is FIXED FOREVER — changing it breaks every trained
policy on this spec.
"""
from __future__ import annotations

from pathlib import Path

from .base import RobotSpec


REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())


# ──────────────────────────────────────────────────────────────────────
# Joint order — matches Unitree URDF naming, grouped by body region.
# ──────────────────────────────────────────────────────────────────────

JOINT_NAMES = (
    # Left leg (6), hip → ankle
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    # Right leg (6)
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    # Waist (1)
    "waist_yaw_joint",
    # Left arm (5), shoulder → wrist
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    # Right arm (5)
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)
assert len(JOINT_NAMES) == 23


# ──────────────────────────────────────────────────────────────────────
# Nominal standing pose. Slight squat for CoM-over-feet stability.
#
# hip_pitch negative = thigh tilted forward (knee comes forward)
# knee positive = bent
# ankle_pitch negative = compensates so the foot stays flat
# ──────────────────────────────────────────────────────────────────────

_NOMINAL = {
    # Legs: squat that puts pelvis ~0.7 m off the ground.
    "left_hip_pitch_joint":   -0.20,
    "left_hip_roll_joint":     0.00,
    "left_hip_yaw_joint":      0.00,
    "left_knee_joint":         0.42,
    "left_ankle_pitch_joint": -0.23,
    "left_ankle_roll_joint":   0.00,
    "right_hip_pitch_joint":  -0.20,
    "right_hip_roll_joint":    0.00,
    "right_hip_yaw_joint":     0.00,
    "right_knee_joint":        0.42,
    "right_ankle_pitch_joint":-0.23,
    "right_ankle_roll_joint":  0.00,
    # Waist upright.
    "waist_yaw_joint":         0.00,
    # Arms hanging straight down with a small outward roll.
    "left_shoulder_pitch_joint":  0.00,
    "left_shoulder_roll_joint":   0.20,
    "left_shoulder_yaw_joint":    0.00,
    "left_elbow_joint":           0.50,
    "left_wrist_roll_joint":      0.00,
    "right_shoulder_pitch_joint": 0.00,
    "right_shoulder_roll_joint": -0.20,
    "right_shoulder_yaw_joint":   0.00,
    "right_elbow_joint":          0.50,
    "right_wrist_roll_joint":     0.00,
}
NOMINAL_POSE = tuple(_NOMINAL[j] for j in JOINT_NAMES)
assert len(NOMINAL_POSE) == 23


# ──────────────────────────────────────────────────────────────────────
# Per-joint limits — pulled directly from the URDF <limit> attributes.
# ──────────────────────────────────────────────────────────────────────

_LIM = {
    "left_hip_pitch_joint":    (-2.531, +2.880),
    "left_hip_roll_joint":     (-0.524, +2.967),
    "left_hip_yaw_joint":      (-2.758, +2.758),
    "left_knee_joint":         (-0.087, +2.880),
    "left_ankle_pitch_joint":  (-0.873, +0.524),
    "left_ankle_roll_joint":   (-0.262, +0.262),
    "right_hip_pitch_joint":   (-2.531, +2.880),
    "right_hip_roll_joint":    (-2.967, +0.524),
    "right_hip_yaw_joint":     (-2.758, +2.758),
    "right_knee_joint":        (-0.087, +2.880),
    "right_ankle_pitch_joint": (-0.873, +0.524),
    "right_ankle_roll_joint":  (-0.262, +0.262),
    "waist_yaw_joint":         (-2.618, +2.618),
    "left_shoulder_pitch_joint":  (-3.089, +2.670),
    "left_shoulder_roll_joint":   (-1.588, +2.252),
    "left_shoulder_yaw_joint":    (-2.618, +2.618),
    "left_elbow_joint":           (-1.047, +2.094),
    "left_wrist_roll_joint":      (-1.972, +1.972),
    "right_shoulder_pitch_joint": (-3.089, +2.670),
    "right_shoulder_roll_joint":  (-2.252, +1.588),
    "right_shoulder_yaw_joint":   (-2.618, +2.618),
    "right_elbow_joint":          (-1.047, +2.094),
    "right_wrist_roll_joint":     (-1.972, +1.972),
}
JOINT_LIMITS_LO = tuple(_LIM[j][0] for j in JOINT_NAMES)
JOINT_LIMITS_HI = tuple(_LIM[j][1] for j in JOINT_NAMES)


# ──────────────────────────────────────────────────────────────────────
# Per-joint PD gains. G1 is much lighter than Atlas (34 kg vs 175 kg)
# but follows the same pattern: stiff hold on non-task joints (arms,
# waist) so they barely move from nominal, moderate gains on legs so
# the residual policy retains authority where it matters.
# Effort caps from URDF: knee 139 Nm, hip/waist 88 Nm, ankle 35 Nm,
# shoulder/elbow/wrist 25 Nm.
# ──────────────────────────────────────────────────────────────────────

def _per_joint_pid():
    pid = {}
    for j in JOINT_NAMES:
        if "shoulder" in j or "elbow" in j or "wrist" in j or "waist" in j:
            # Stiff hold — these are not RL-controlled in any meaningful
            # way; we want them held at nominal so the policy can focus
            # on legs.
            pid[j] = (400.0, 0.0, 15.0)
        elif "knee" in j:
            # Knee carries body weight; needs high gain.
            pid[j] = (300.0, 0.0, 10.0)
        elif "ankle_pitch" in j or "hip_pitch" in j:
            # Pitch joints: primary locomotion drivers.
            pid[j] = (200.0, 0.0, 8.0)
        elif "ankle_roll" in j or "hip_roll" in j:
            # Roll joints: lateral balance.
            pid[j] = (150.0, 0.0, 6.0)
        else:  # hip_yaw
            pid[j] = (120.0, 0.0, 5.0)
    return tuple(pid[j] for j in JOINT_NAMES)


MOTOR_PID_PER_JOINT = _per_joint_pid()


# ──────────────────────────────────────────────────────────────────────
# Bipedal CPG prior: alternate-step gait. Left leg phase 0.0, right
# leg phase 0.5 — opposite half-cycle. Arms counter-rotate (same-side
# arm opposite to leg).
# ──────────────────────────────────────────────────────────────────────

_PHASES = {j: 0.0 for j in JOINT_NAMES}
for j in JOINT_NAMES:
    if j.startswith("right_") and ("hip" in j or "knee" in j or "ankle" in j):
        _PHASES[j] = 0.5
for j in JOINT_NAMES:
    if j.startswith("left_") and ("shoulder" in j or "elbow" in j):
        _PHASES[j] = 0.5  # left arm forward when right leg forward
    # right arm already at 0.0 = forward when left leg forward
CPG_LEG_PHASE = tuple(_PHASES[j] for j in JOINT_NAMES)


# ──────────────────────────────────────────────────────────────────────

G1 = RobotSpec(
    name="g1",
    urdf_path=REPO_ROOT / "projects" / "robots" / "unitree" / "g1"
              / "urdf" / "g1_23dof_omnisim.urdf",
    joint_names=JOINT_NAMES,
    nominal_pose=NOMINAL_POSE,
    joint_limits_lo=JOINT_LIMITS_LO,
    joint_limits_hi=JOINT_LIMITS_HI,
    # G1 is more sensitive to action authority than OmniQuad — 0.15 rad
    # is the same scale Atlas uses, plenty for the legs and not so
    # much that the residual can violate the CPG into a fall.
    action_scale=0.15,
    # obs_dim = 9 (vel/grav) + 3*23 (q/qd/last_action) + 4 (cmd+clock)
    obs_dim=9 + 3 * 23 + 4,    # 82
    # Spawn pose: pelvis at z=0.76 for the squat nominal (leg length
    # ≈0.7 m, slight bend brings pelvis to ~0.7 m at static-stance FK).
    # Spawn 6 cm above the expected settle so the first physics step
    # has a clean foot contact.
    spawn_translation=(0.0, 0.0, 0.76),
    spawn_rotation=(0.0, 0.0, 1.0, 0.0),
    max_episode_steps=1024,
    obs_recipe="legged_locomotion",
    motor_pid=(200.0, 0.0, 8.0),
    motor_pid_per_joint=MOTOR_PID_PER_JOINT,
    # Bipedal trot prior: ~1.3 Hz step rate (a touch faster than Atlas's
    # 1.2 Hz to match G1's smaller body). Modest hip swing + knee lift.
    cpg_freq_hz=1.3,
    cpg_hip_y_amp=0.18,
    cpg_knee_amp=0.25,
    cpg_leg_phase=CPG_LEG_PHASE,
)
