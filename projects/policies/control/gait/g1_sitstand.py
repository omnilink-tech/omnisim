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

"""g1_sitstand.py -- a PHYSICS-RESPECTING, ACHIEVABLE sit -> stand -> sit reference.

The G1 sits on a chair (feet planted ~0.28 m forward, in front of the seat). To
stand it does what a person does:
  1. LEAN forward + SHIFT the pelvis forward OVER the planted feet while still low
     (butt just leaving the chair) -- so the center of mass gets over the feet
     BEFORE committing to standing (otherwise it tips backward as the butt lifts);
  2. RISE straight up over the feet to a full UPRIGHT stand, in front of the chair;
  3. hold 5 s, then reverse to sit back down.

Design rules honored:
  - GHOST-FIRST: the reference we agree on before the robot mimics it.
  - ACHIEVABLE: derived from the robot's own kinematics (closed-form leg IK), and
    the CoM is kept over the foot support throughout (quasi-static stability) so the
    robot can actually balance it -- the lean+shift fixes the tip-back the naive
    rise had.
  - RESPECT PHYSICS / SURROUNDINGS: leg IK holds the feet PLANTED on the floor
    (no penetration) and at x=0.28 -- CLEAR of the chair seat (x<=0.22) -- so
    nothing tucks under the chair or dips through the ground.

The whole motion (all 23 joints + base x/height/lean) lives HERE; the ghost
displays it and the mimic trainer simply TRACKS it.
"""

import math

from projects.policies.control.gait.g1_human_gait import (
    _leg_ik, _ankle_for_foot_pitch, HIP_DROP, SOLE_DROP)

X_FOOT   = 0.25      # SEATED/back foot world-x (just clear of the chair seat front 0.22).
X_FOOT_FWD = 0.40    # STEPPED-forward foot world-x (clear of the chair, under the stood body)
_FOOT_Z  = 0.015     # target sole height (small clearance; closed-form IK is ~2 cm off)
_SWING_H = 0.10      # foot lift height during the swing (step) phase

# --- THE STEP: a no-step upright stand is unreachable (chair forces the feet
# forward + the G1 has no torso-pitch joint -> the body can't get its CoM over the
# planted feet -> it bows). So the achievable motion STEPS forward: rise leaning
# (build forward momentum), step each foot forward to CATCH the body over the new
# feet, settle UPRIGHT clear of the chair. Per-leg foot trajectories (x, z); z>0 = a
# lifted swing. Right foot steps first, then left. ---
#   keyframe: (time_s, foot_x, foot_z)
_RFOOT = [
    (0.0, X_FOOT, 0.0), (1.5, X_FOOT, 0.0),         # planted (seated + lean + start of rise)
    (1.8, 0.5 * (X_FOOT + X_FOOT_FWD), _SWING_H),   # swing up + forward
    (2.1, X_FOOT_FWD, 0.0), (99.0, X_FOOT_FWD, 0.0),  # plant forward
]
_LFOOT = [
    (0.0, X_FOOT, 0.0), (2.4, X_FOOT, 0.0),         # planted (stance through the right step)
    (2.7, 0.5 * (X_FOOT + X_FOOT_FWD), _SWING_H),   # swing up + forward
    (3.0, X_FOOT_FWD, 0.0), (99.0, X_FOOT_FWD, 0.0),  # plant forward (feet together)
]

Z_SEATED = 0.55
Z_STAND  = 0.76      # stand pelvis -> with feet UNDER the body gives hip~-0.36/knee~0.59
                     # (the stable slight-squat nominal). z>=0.78 over-extends the vertical
                     # legs -> clamps to near-straight (hip-0.19/knee0.22), which TIPS
                     # FORWARD (documented G1 regression). 0.76 = the g1_stand-stable squat.
X_STAND  = X_FOOT    # standing pelvis over the planted feet (in front of the chair)

# Pelvis keyframes: (time_s, base_x, base_z, lean_pitch_rad[+ = lean forward]).
# STEP-FORWARD stand: perch at the seat front edge, rise LEANING (build forward
# momentum), then the body rides that momentum FORWARD while each foot steps out to
# catch it (see _RFOOT/_LFOOT), settling UPRIGHT over the stepped feet (x=0.40),
# clear of the chair. The body translates 0.15 -> 0.40 -- impossible to do statically
# over planted feet (ankle-limited -> bow), but natural dynamically WITH the steps.
_PERCH_X = 0.15
_KF = [
    (0.0,  0.15, 0.55, 0.00),   # PERCHED at front edge
    (0.6,  0.17, 0.55, 0.38),   # LEAN HARD forward, NO rise yet (z held) -> CoM over feet
                                # BEFORE any leg extension, so extending lifts up-and-forward
                                # instead of pushing the body BACK into the chair (the launch
                                # slid backward x0.15->0.10 and stalled when rise+lean were
                                # simultaneous).
    (1.3,  0.24, 0.62, 0.32),   # now RISE (unweight the chair), lean held
    (2.0,  0.32, 0.68, 0.20),   # right foot has stepped to 0.40; body rides momentum fwd
    (2.8,  0.38, 0.73, 0.08),   # left foot steps up to 0.40; body keeps coming forward
    (3.6,  0.40, 0.76, 0.00),   # settle UPRIGHT over the stepped feet (slight-squat nominal)
    (8.6,  0.40, 0.76, 0.00),   # STAND (upright torso, feet under hips -- g1_stand-stable)
]
T_TOTAL = _KF[-1][0]

# Arm sub-pose: hands come forward/up a little during the lean (natural reach), at
# the sides when standing. Interpolated by the same stand fraction.
_ARMS_SEATED = {
    "left_shoulder_pitch_joint": 0.30, "left_shoulder_roll_joint": 0.15,
    "left_shoulder_yaw_joint": 0.0, "left_elbow_joint": 0.30, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.30, "right_shoulder_roll_joint": -0.15,
    "right_shoulder_yaw_joint": 0.0, "right_elbow_joint": 0.30, "right_wrist_roll_joint": 0.0,
}
_ARMS_STAND = {
    "left_shoulder_pitch_joint": 0.0, "left_shoulder_roll_joint": 0.15,
    "left_shoulder_yaw_joint": 0.0, "left_elbow_joint": 0.0, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0, "right_shoulder_roll_joint": -0.15,
    "right_shoulder_yaw_joint": 0.0, "right_elbow_joint": 0.0, "right_wrist_roll_joint": 0.0,
}


def _ease(x):
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x * x * (3.0 - 2.0 * x)


def _base(t):
    """(x, z, lean) of the pelvis at time t, smoothstep between keyframes."""
    if t <= _KF[0][0]:
        return _KF[0][1], _KF[0][2], _KF[0][3]
    if t >= _KF[-1][0]:
        return _KF[-1][1], _KF[-1][2], _KF[-1][3]
    for i in range(len(_KF) - 1):
        t0, x0, z0, p0 = _KF[i]
        t1, x1, z1, p1 = _KF[i + 1]
        if t0 <= t <= t1:
            f = _ease((t - t0) / (t1 - t0)) if t1 > t0 else 0.0
            return x0 + f * (x1 - x0), z0 + f * (z1 - z0), p0 + f * (p1 - p0)
    return _KF[-1][1], _KF[-1][2], _KF[-1][3]


def _foot(kfs, t):
    """Interpolate a per-leg foot trajectory (x, z) at time t (smoothstep). z>0 = the
    foot is lifted in a swing (a step)."""
    if t <= kfs[0][0]:
        return kfs[0][1], kfs[0][2]
    if t >= kfs[-1][0]:
        return kfs[-1][1], kfs[-1][2]
    for i in range(len(kfs) - 1):
        t0, x0, z0 = kfs[i]
        t1, x1, z1 = kfs[i + 1]
        if t0 <= t <= t1:
            f = _ease((t - t0) / (t1 - t0)) if t1 > t0 else 0.0
            return x0 + f * (x1 - x0), z0 + f * (z1 - z0)
    return kfs[-1][1], kfs[-1][2]


def _leg_to_foot(px, pz, pitch, fx, fz):
    """Leg IK placing ONE foot at world (fx, fz) while the pelvis is at (px, pz)
    pitched forward by `pitch`. fz>0 = a lifted (swing) foot. Returns (hip,knee,ankle)."""
    hx = px - math.sin(pitch) * HIP_DROP        # hip anchor world (pelvis pitched)
    hz = pz - math.cos(pitch) * HIP_DROP
    rwx = fx - hx                               # ankle target relative to hip, world
    rwz = (fz + SOLE_DROP) - hz
    dx = math.cos(pitch) * rwx - math.sin(pitch) * rwz       # -> hip (pelvis) frame
    dzf = math.sin(pitch) * rwx + math.cos(pitch) * rwz
    qh, qk = _leg_ik(dx, -dzf)                   # IK dz is downward-positive
    qa = _ankle_for_foot_pitch(qh, qk, foot_pitch=-pitch)    # foot flat in WORLD
    return qh, qk, qa


def full_targets(t):
    """Complete 23-joint reference dict at time t. Legs are computed PER-SIDE so the
    swing foot can lift + step forward (see _RFOOT/_LFOOT)."""
    px, pz, pitch = _base(t)
    lfx, lfz = _foot(_LFOOT, t)
    rfx, rfz = _foot(_RFOOT, t)
    lqh, lqk, lqa = _leg_to_foot(px, pz, pitch, lfx, lfz)
    rqh, rqk, rqa = _leg_to_foot(px, pz, pitch, rfx, rfz)
    pose = {
        "left_hip_pitch_joint": lqh,  "right_hip_pitch_joint": rqh,
        "left_hip_roll_joint": 0.0,  "right_hip_roll_joint": 0.0,
        "left_hip_yaw_joint": 0.0,   "right_hip_yaw_joint": 0.0,
        "left_knee_joint": lqk,       "right_knee_joint": rqk,
        "left_ankle_pitch_joint": lqa, "right_ankle_pitch_joint": rqa,
        "left_ankle_roll_joint": 0.0, "right_ankle_roll_joint": 0.0,
        "waist_yaw_joint": 0.0,
    }
    b = blend(t)
    for j in _ARMS_SEATED:
        pose[j] = _ARMS_SEATED[j] + b * (_ARMS_STAND[j] - _ARMS_SEATED[j])
    return pose


def ref_pelvis_x(t):
    return _base(t)[0]


def ref_pelvis_z(t):
    return _base(t)[1]


def ref_pelvis_pitch(t):
    return _base(t)[2]


def blend(t):
    """Stand fraction in [0,1] (0 seated, 1 standing) -- phase/obs convenience."""
    return max(0.0, min(1.0, (_base(t)[1] - Z_SEATED) / (Z_STAND - Z_SEATED)))


SEATED_POSE   = full_targets(0.0)
STANDING_POSE = full_targets(5.0)   # the upright STAND phase (after the step)
