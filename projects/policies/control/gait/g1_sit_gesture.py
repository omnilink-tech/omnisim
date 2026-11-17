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

"""g1_sit_gesture.py -- the seated upper-body "shadow" for the G1 sit-mimic demo.

ONE shared kinematic reference that BOTH the ghost (a kinematic, translucent
display robot) and the mimic (the physics-tracked real robot) consume, so the
real robot reproduces exactly what the ghost shows.

This is the "shadow" layer of the Ghost Method, deliberately isolated to a
statically-stable SEATED task (the pelvis is pinned, so there is no balance
problem) in order to validate the full pipeline end to end:

    shadow(t)  ->  ghost displays it (kinematic)
               ->  mimic tracks it under the Newton deploy physics

with zero biped-balance confound. A learned RL residual later rides on top of
full_targets() (mimic target = full_targets(t) + ACT_SCALE * residual(obs)),
which is the exact seam an RL policy plugs into.

All angles are radians and stay inside the g1_23dof URDF joint limits.
"""

import math

# ---------------------------------------------------------------------------
# 23-DOF seated hold pose (joint -> angle, rad). Legs bent ~90 deg as if on a
# chair; torso upright; arms relaxed at the sides. Within URDF limits:
#   hip_pitch [-2.531, 2.880]  knee [-0.087, 2.880]  ankle_pitch [-0.873, 0.524]
# ---------------------------------------------------------------------------
SEATED_POSE = {
    # --- legs (natural seated pose: thighs ~horizontal forward, shins down,
    #     feet near the floor). The robot SPAWNS in this pose (seed-pose env in
    #     the launcher) so the legs never fall straight and jam their feet into
    #     the floor; the joint servo then holds them here. ~90 deg hip + ~90 deg
    #     knee = a normal sit, not the knees-to-chest tuck the anti-jam workaround
    #     needed before seed-pose. ---
    "left_hip_pitch_joint":  -1.45, "right_hip_pitch_joint":  -1.45,
    "left_hip_roll_joint":     0.06, "right_hip_roll_joint":   -0.06,
    "left_hip_yaw_joint":      0.00, "right_hip_yaw_joint":     0.00,
    "left_knee_joint":         1.45, "right_knee_joint":        1.45,
    "left_ankle_pitch_joint":  0.00, "right_ankle_pitch_joint":  0.00,
    "left_ankle_roll_joint":   0.00, "right_ankle_roll_joint":   0.00,
    # --- waist (upright torso) ---
    "waist_yaw_joint":         0.00,
    # --- arms (relaxed, slight outward shoulder roll, small elbow bend) ---
    "left_shoulder_pitch_joint":  0.00, "left_shoulder_roll_joint":   0.18,
    "left_shoulder_yaw_joint":    0.00, "left_elbow_joint":           0.30,
    "left_wrist_roll_joint":      0.00,
    "right_shoulder_pitch_joint": 0.00, "right_shoulder_roll_joint": -0.18,
    "right_shoulder_yaw_joint":   0.00, "right_elbow_joint":          0.30,
    "right_wrist_roll_joint":     0.00,
}

WAVE_HZ = 0.65  # forearm wave speed (Hz) -- paced so the position-controlled
                # arm servo tracks it with low phase lag (a 0.9 Hz flick lagged
                # ~0.2 rad at the fastest part of the swing; 0.65 Hz reads as a
                # natural, friendly wave AND stays inside the actuator bandwidth).
RAISE_S = 1.5   # smooth raise-in time (s)


def _ease(x):
    """smoothstep on [0, 1] (clamped)."""
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return x * x * (3.0 - 2.0 * x)


def arm_gesture(t):
    """Return {joint: angle_rad} OVERRIDES (on top of SEATED_POSE) for the
    gesturing arms at sim time t (seconds).

    Right arm  -> raises out to the side and WAVES (the headline gesture).
    Left arm   -> a slow, gentle raise/lower (the "and something").

    shoulder_roll abducts the arm out to the side (mirror of the +0.18 nominal:
    LEFT raises with +roll, RIGHT raises with -roll); elbow + wrist_roll do the
    wave. All targets stay inside the URDF limits.
    """
    g = {}
    w = 2.0 * math.pi * WAVE_HZ

    # --- right arm: raise out to the side, wave the forearm ---
    r = _ease(t / RAISE_S)                                  # smooth raise-in
    g["right_shoulder_roll_joint"]  = -1.30 * r             # abduct ~75 deg
    g["right_shoulder_pitch_joint"] = -0.25 * r             # slightly forward
    g["right_elbow_joint"]          = (1.05 + 0.50 * math.sin(w * t)) * r
    g["right_wrist_roll_joint"]     = 0.55 * math.sin(w * t) * r

    # --- left arm: slow gentle raise/lower, 6 s period (calmer secondary motion) ---
    lift = (1.0 - math.cos(2.0 * math.pi * t / 6.0)) * 0.5  # 0..1
    g["left_shoulder_roll_joint"] = 0.18 + 0.70 * lift
    g["left_elbow_joint"]         = 0.30 + 0.55 * lift

    return g


def full_targets(t):
    """Complete 23-joint target dict at time t: the seated hold with the
    gesturing-arm overrides applied. Both the ghost and the mimic call this, so
    the real robot reproduces exactly what the ghost displays."""
    q = dict(SEATED_POSE)
    q.update(arm_gesture(t))
    return q
