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

"""CRAWL-mode config for the G1 army-crawl campaign (env CRAWL=1).

Single source of the prone/commando constants the trainer (g1_walk_recipe.py)
and deploy read when CRAWL is on, so the crawl hooks in the shared trainer stay
tiny (they pull values from here). The crawl reuses the trainer's mtrack
(motion-tracking) reward branch -- keypoint + whole-body imitation, balance
emergent -- with three things re-pointed for a prone body:

  * spawn: base pitched ~90 deg (torso horizontal, face-down), pelvis low,
    joints seeded to the commando anchor (the ghost's pose center);
  * fall: measure |pitch - CRAWL_PITCH| (a horizontal torso is NOT a fall) and
    use a low pelvis-collapse threshold;
  * height/attitude references: pull toward CRAWL_Z / CRAWL_PITCH, not standing.

The nominal crawl attitude is CRAWL_PITCH here; the small per-bin sway lives in
the ghost's att_lut, exactly as for the walk. Posture matches the probe-
validated, owner-signed-off commando pose (forearms+knees, arms ~50-70%).
"""
from __future__ import annotations

import math
import numpy as np

# ── nominal prone frame ───────────────────────────────────────────────
CRAWL_PITCH = math.radians(90.0)   # base pitch: torso horizontal, face toward floor
CRAWL_Z = 0.30                     # pelvis height (low profile, under-the-wire)

# ── prone-valid failure (vs upright FALL_ROLL/PITCH 0.8, FALL_BZ 0.45) ──
FALL_ROLL = 0.9                    # |roll|            -> rolled off the belly
FALL_PITCH = 1.0                   # |pitch-CRAWL_PITCH| -> tumbled fore/aft
FALL_BZ = 0.12                     # pelvis below this  -> collapsed flat

# ── prone harness (vs upright gate 0.35, Z0 0.72) ─────────────────────
HARNESS_GATE_Z = 0.12              # assist while pelvis is above this (crawl sits ~0.30)
HARNESS_Z0 = 0.30                  # hold pelvis at the crawl height

WB_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
]

# commando anchor = the ghost pose center (make_commando_ghost.ANCHOR)
_ANCHOR = {
    "left_hip_pitch_joint": -0.80, "right_hip_pitch_joint": -0.80,
    "left_hip_roll_joint":  0.22,  "right_hip_roll_joint": -0.22,
    "left_hip_yaw_joint":   0.00,  "right_hip_yaw_joint":  0.00,
    "left_knee_joint":      0.80,  "right_knee_joint":     0.80,
    "left_ankle_pitch_joint": -0.55, "right_ankle_pitch_joint": -0.55,
    "left_ankle_roll_joint":  0.00,  "right_ankle_roll_joint":  0.00,
    "waist_yaw_joint": 0.00,
    "left_shoulder_pitch_joint": -1.50, "right_shoulder_pitch_joint": -1.50,
    "left_shoulder_roll_joint":   0.25, "right_shoulder_roll_joint": -0.25,
    "left_shoulder_yaw_joint":    0.00, "right_shoulder_yaw_joint":   0.00,
    "left_elbow_joint":  0.35, "right_elbow_joint":  0.35,
    "left_wrist_roll_joint": 0.00, "right_wrist_roll_joint": 0.00,
}


def crawl_quat() -> np.ndarray:
    """Base spawn quaternion (w,x,y,z): pitch CRAWL_PITCH about +y (torso
    horizontal, face-down) -- matches the hologram + the feasibility render."""
    a = CRAWL_PITCH
    return np.array([math.cos(a / 2), 0.0, math.sin(a / 2), 0.0], np.float32)


def seed_legs() -> np.ndarray:
    """12 leg joints (L6+R6) at the commando anchor -- the trainer's leg seed."""
    order = WB_JOINTS[0:12]
    return np.array([_ANCHOR[j] for j in order], np.float32)


def seed_wb() -> np.ndarray:
    """All 23 joints at the commando anchor (WB_JOINTS order)."""
    return np.array([_ANCHOR[j] for j in WB_JOINTS], np.float32)
