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

"""Human-like parametric gait model for the Unitree H1 (legs+waist).

H1 is a 5-DOF-per-leg biped (hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch --
NO ankle-roll), taller than G1 (~1.0 m leg reach). This is the G1 foot-space
gait shadow re-calibrated to H1's kinematics (see the constants block) and
scaled to its size. Because H1 has no ankle-roll it uses lateral="human" (hip
ab/adduction) for the frontal plane, not the LIPM ankle-counter-rotation mode.


THE GAIT IS PLANNED IN FOOT SPACE, NOT JOINT SPACE. The old reference was
joint-space sinusoids (hip = sine, knee = clipped sine) -- that is why it
looked robotic. Human walking is structured around foot trajectories:

  - stance foot: ON the ground, translating backward UNDER the pelvis at
    exactly -vx (this is what produces forward velocity without skating);
  - swing foot: a smooth arc -- lift, travel forward, touch down ahead of
    the body with ~zero relative velocity (no foot slap);
  - duty factor ~60% stance / 40% swing per leg, with double-support
    overlap (both feet down ~10% of the cycle, twice) -- human timing,
    not the sine's 50/50;
  - pelvis: small vertical bob (peaks at single-support mid-stance) and a
    lateral weight shift toward the stance foot;
  - legs realize the foot targets through closed-form 2-link IK -- the
    knee EXTENDS when the foot passes under the body (the tall, un-
    crouched stance falls out of the geometry);
  - arms swing counter-phase to their same-side leg, elbows softly bent.

A STRIDE RAMP grows the step length from zero over `ramp_s` seconds: the
gait starts as marching-in-place from the standing pose (which IS the
model's phase-0 posture) and strides out -- so a standing start has no
target snap and no out-of-distribution launch transient.

The same math is implemented twice -- `targets_np` (scalar, deploy
controller) and `targets_torch` (batched on GPU, trainer) -- and
`python g1_human_gait.py` self-tests both against each other and the IK
against MuJoCo forward kinematics (<5 mm foot-position error).

Kinematic constants are calibrated numerically in `calibrate()` against
projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml (the validated
deploy-matched model), then frozen here as defaults so the deploy
controller does not need mujoco installed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# -- Calibrated kinematics (H1; projects/policies/control/gait/tools/urdf_leg_calib.py,
# frozen 2026-06-24). H1's leg is a clean straight 2-link: thigh and shank are
# each exactly 0.40 m straight down (THIGH_OFF=SHANK_OFF=0); pitch axes are +y
# so the signs are all -1 (same family as G1). The stand cross-check is EXACT. --
L1 = 0.40000          # thigh: hip_pitch anchor -> knee anchor, sagittal plane
L2 = 0.40000          # shank: knee anchor -> ankle anchor
THIGH_OFF = 0.00000   # thigh straight down at q=0 (no kink)
SHANK_OFF = 0.00000   # shank straight down at q=0
HIP_SIGN = -1.0       # +q_hip rotates the thigh BACKWARD (so -q = forward swing)
KNEE_SIGN = -1.0      # +q_knee flexes the shank BACKWARD relative to the thigh
ANKLE_AX = -1.0       # +q_ankle pitches the foot toes-UP
HIP_DROP = 0.17420    # pelvis origin -> hip_pitch anchor, vertical drop
SOLE_DROP = 0.06200   # ankle anchor -> foot sole, foot flat

# Joint slots. H1's leg is hip_yaw/hip_roll/hip_pitch/knee/ankle (5 DOF, NO
# ankle-roll), but we keep the SAME 13-slot layout as G1 so the math is shared:
# the ankle-roll slots (_L_AR/_R_AR) simply stay 0 (H1 uses lateral="human", no
# ankle counter-rotation) and the ghost/trainer just don't map them to a joint.
NJ = 13
_L_HP, _L_HR, _L_HY, _L_KN, _L_AP, _L_AR = 0, 1, 2, 3, 4, 5
_R_HP, _R_HR, _R_HY, _R_KN, _R_AP, _R_AR = 6, 7, 8, 9, 10, 11
_WAIST = 12


@dataclass
class GaitParams:
    """All lengths in meters, angles in rad, frequencies in Hz."""
    vx: float = 0.45           # forward speed the reference encodes (H1: longer legs)
    freq: float = 1.2          # per-leg cycle frequency (slower than G1: longer legs)
    duty: float = 0.6          # stance fraction of the cycle (human ~0.6)
    step_height: float = 0.06  # swing foot apex clearance
    pelvis_height: float = 1.00   # MEAN pelvis z (reach 1.036; ~97% = tall walk)
    bob: float = 0.025         # vertical bob; peak at single-support mid-stance =
    #                            inverted-pendulum arc that makes humans walk TALL
    sway: float = 0.05         # hip-roll weight-shift amplitude (rad, proven)
    arm_swing: float = 0.25    # shoulder-pitch counter-swing (rad, proven)
    elbow_bend: float = 0.15   # constant natural elbow flexion (rad)
    ankle_clear: float = 0.08  # swing-phase toe-up (rad of foot pitch)
    x0: float = -0.02          # stride center relative to hip anchor (the proven
    #                            standing poses keep the ankle ~2cm behind the hip)
    ramp_s: float = 1.0        # stride-length ramp-in from standing start
    # â”€â”€ style: "ik" = foot-space plan + leg IK (v1); "winter" = measured
    # human joint kinematics (Winter normative curves) driven directly â”€â”€
    style: str = "ik"
    # Winter-style knobs. The normative curves are stored at HUMAN scale
    # (natural cadence, deg); these scale them onto the robot:
    winter_hip0: float = -0.17    # mean hip pitch (G1 sign: - = flexed fwd)
    winter_hip_scale: float = 0.75   # waveform amplitude scale (1.0 = human)
    winter_knee_scale: float = 0.85
    winter_ankle_scale: float = 0.9
    winter_knee0: float = 0.06    # knee angle at heel strike (near straight!)
    winter_ankle0: float = -0.18  # stance ankle = flat foot at the stand posture
    # ── FRONTAL/TRANSVERSE plane modes (the "improved shadow" upgrades) ──
    # The legacy shadow has a near-zero frontal plane (`sway` only) and zero
    # hip yaw -- a kinematically-perfect but DYNAMICALLY-INFEASIBLE target the
    # real biped can never match without falling. These give the reference a
    # principled, achievable lateral weight transfer + hip rotation:
    #   lateral = "sway"  : legacy hip-roll sine (default, back-compat)
    #             "lipm"  : (A) Linear-Inverted-Pendulum weight transfer over
    #                        the stance foot + ankle-roll counter-rotation
    #                        (feet stay flat); amplitude set by balance physics.
    #             "human" : (C) measured human hip ab/adduction (frontal plane).
    #   yaw     = "none"  : zero hip yaw (default)
    #             "human" : (C) measured human hip internal/external rotation.
    lateral: str = "human"     # H1 has NO ankle-roll -> use measured human hip
    #                            ab/adduction (LIPM mode needs ankle counter-rotation)
    yaw: str = "human"         # natural hip internal/external rotation
    com_height: float = 0.85   # CoM height above the ankle for the LIPM omega
    step_width: float = 0.21   # nominal lateral foot separation (m): hip_y=0.203
    lat_hip_amp: float = 0.09  # (A) peak hip-roll for LIPM weight transfer (rad,
    #                            ~5 deg). LIPM sets the WAVEFORM/timing; this pins
    #                            the amplitude to physiological pelvic sway (the
    #                            point-mass CoM model alone under-predicts it --
    #                            real pelvic obliquity adds the rest).
    lat_scale: float = 1.0     # overall scale on the frontal-plane amplitude
    yaw_scale: float = 1.0     # overall scale on the transverse-plane amplitude
    # ── CAPTURE-POINT foot placement (closed-loop balance) ──────────────────
    # The rigid shadow lands the swing foot at a FIXED +L/2 ahead regardless of
    # the CoM state, so it cannot take a recovery step -> it topples in ~2-3
    # cycles. cp_gain (when >0 AND the controller passes a measured forward
    # velocity v_meas) adds the DCM/capture-point correction to the swing landing:
    #   x_land = +L/2 + cp_gain * (v_meas - vx)   (step farther when falling faster).
    # cp_gain ~ 1/omega = 1/sqrt(g/com_height) ~ 0.3. Default 0 = legacy open-loop.
    cp_gain: float = 0.0       # capture-point step-placement gain (s); 0 = off
    cp_max: float = 0.18       # clamp on the per-step foot-x correction (m)
    cp_gain_y: float = 0.0     # lateral capture-point gain (damps side sway); 0 = off
    cp_max_y: float = 0.10     # clamp on the lateral foot-y correction (m)

    @property
    def step_len(self) -> float:
        # distance the body travels per leg cycle; each foot strides this far
        return self.vx / self.freq


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# WINTER normative joint kinematics (sagittal, natural cadence).
# Keyframes digitised from the canonical gait-analysis curves (Winter,
# Biomechanics and Motor Control of Human Movement): joint angle in DEG
# vs % gait cycle, 0% = heel strike of the same leg. These are the two
# signatures the IK style cannot produce and the eye reads as human:
#   - the KNEE DOUBLE-BEND (weight-acceptance flex ~18 deg at 15%,
#     re-extend, then the big ~62 deg swing flex at 70%);
#   - the ANKLE PUSH-OFF (plantarflexion kick ~-15 deg at toe-off, 62%).
# Stored at human scale; GaitParams.winter_* map them onto the robot.
# Sign conventions here: hip flexion +, knee flexion +, dorsiflexion +.
_W_HIP = [(0, 20), (10, 17), (20, 11), (30, 3), (40, -6), (50, -12),
          (55, -11), (62, -4), (70, 8), (80, 19), (88, 24), (95, 23), (100, 20)]
_W_KNEE = [(0, 5), (8, 15), (15, 18), (25, 12), (40, 5), (50, 8), (57, 20),
           (65, 45), (72, 62), (80, 45), (88, 22), (95, 8), (100, 5)]
_W_ANKLE = [(0, 0), (7, -5), (15, 0), (30, 6), (45, 10), (52, 6), (58, -5),
            (63, -15), (70, -8), (78, 0), (88, 3), (95, 2), (100, 0)]
_W_N = 256


def _winter_table(keys):
    """Periodic lookup table (length _W_N) from keyframes, smoothed with a
    circular Hann kernel so the linear-keyframe corners disappear. Both the
    numpy and torch paths interpolate THIS table -> exact parity."""
    ph = np.array([k[0] for k in keys], dtype=np.float64) / 100.0
    val = np.array([k[1] for k in keys], dtype=np.float64)
    s = np.linspace(0.0, 1.0, _W_N, endpoint=False)
    tab = np.interp(s, ph, val)
    win = np.hanning(13)
    win /= win.sum()
    ext = np.concatenate([tab[-6:], tab, tab[:6]])
    tab = np.convolve(ext, win, mode="same")[6:-6]
    return tab


HIP_TAB = _winter_table(_W_HIP)        # deg, human scale
KNEE_TAB = _winter_table(_W_KNEE)
ANKLE_TAB = _winter_table(_W_ANKLE)
_HIP_MEAN = float(HIP_TAB.mean())
_KNEE_MIN = float(KNEE_TAB.min())
# Ankle is normalised about its STANCE-phase mean (first 60% of the cycle),
# not the full-cycle mean: winter_ankle0 then equals the flat-foot stance
# ankle of the robot's posture, so mid-stance commands keep the foot flat
# and the push-off/swing deviations ride on top. (Normalising about the
# full-cycle mean biased every stance tick ~0.13 rad toward dorsiflexion --
# a permanent forward shank lean that lifted and tipped the launch.)
_ANKLE_STANCE_MEAN = float(ANKLE_TAB[: int(_W_N * 0.6)].mean())
_D2R = math.pi / 180.0


def _winter_lookup_np(tab, phi):
    """Linear interp into a periodic table, scalar phi in [0,1)."""
    x = (phi % 1.0) * _W_N
    i = int(x) % _W_N
    f = x - int(x)
    return tab[i] * (1.0 - f) + tab[(i + 1) % _W_N] * f


# The ramp blends from this STABLE standing pose (the proven intermediate
# posture) into the full Winter waveform: ramp 0 = stand (settle-safe),
# ramp 1 = measured human kinematics.
WINTER_STAND = (-0.23, 0.46, -0.23)   # hip, knee, ankle (flat foot: ankle=-(hip+knee))


def _winter_leg_np(leg_phi, p: GaitParams, ramp):
    """One leg's (q_hip, q_knee, q_ankle) from the Winter curves, G1 signs.
    ramp in [0,1] blends stand -> full human waveform (launch with no snap)."""
    h = _winter_lookup_np(HIP_TAB, leg_phi)
    k = _winter_lookup_np(KNEE_TAB, leg_phi)
    a = _winter_lookup_np(ANKLE_TAB, leg_phi)
    # G1 signs: hip - = flexion(fwd); knee + = flexion; ankle + = dorsiflex.
    q_hip = p.winter_hip0 - p.winter_hip_scale * (h - _HIP_MEAN) * _D2R
    q_knee = p.winter_knee0 + p.winter_knee_scale * (k - _KNEE_MIN) * _D2R
    q_ankle = p.winter_ankle0 + p.winter_ankle_scale * (a - _ANKLE_STANCE_MEAN) * _D2R
    sh, sk, sa = WINTER_STAND
    return (sh + ramp * (q_hip - sh),
            sk + ramp * (q_knee - sk),
            sa + ramp * (q_ankle - sa))


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# (A) LINEAR INVERTED PENDULUM lateral weight transfer.
# The legacy frontal plane is a tiny hip-roll sine (`sway`) -- the pelvis
# barely moves, so the reference says "stay centred" while balance needs the
# CoM to travel out over the stance foot each step. That contradiction is why
# the robot SPLAYS. Here the lateral CoM follows the textbook LIPM gait: the
# centre of pressure (ZMP) sits under the support foot, the CoM obeys
# y'' = w^2 (y - zmp), and we solve the PERIODIC orbit (the gait is a limit
# cycle, the open-loop LIPM is unstable so forward integration won't do).
# The resulting pelvis-lateral table -> common-mode hip-roll (both hips same
# sign = pelvis translation) + ankle-roll counter-rotation so the feet stay
# flat. Amplitude falls out of step width + cadence, not a magic constant.
_GRAV = 9.81
_lipm_cache = {}


def _seg_A(w, tau):
    """2x2 state-transition for y''=w^2(y-zmp) over duration tau (linear)."""
    ch, sh = math.cosh(w * tau), math.sinh(w * tau)
    return np.array([[ch, sh / w], [w * sh, ch]])


def _lipm_lateral_table(p: "GaitParams"):
    """Periodic LIPM lateral CoM offset y(phi) over the gait cycle, as a
    length-_W_N table (m). +y = toward the LEFT foot. Cached per param set."""
    w = math.sqrt(_GRAV / max(p.com_height, 0.05))
    d = 0.5 * p.step_width                     # foot lateral offset from midline
    key = (round(w, 5), round(p.duty, 4), round(p.freq, 4), round(d, 5))
    if key in _lipm_cache:
        return _lipm_cache[key]
    T = 1.0 / p.freq
    ss = (1.0 - p.duty) * T                    # single-support duration (per leg)
    ds = max(0.0, (p.duty - (1.0 - p.duty)) * 0.5 * T)   # each double-support
    # Cycle, starting at global phi=0. With phiL=(phi+(1-duty))%1, the LEFT leg
    # swings on phiL in [duty,1) -> the RIGHT foot is the sole support there.
    # Segment order from phi=0: [DS, right-SS(zmp=-d), DS, left-SS(zmp=+d)].
    segs = [(ds, 0.0), (ss, -d), (ds, 0.0), (ss, +d)]
    # Monodromy + affine accumulation over one period: x_T = M x_0 + cterm.
    M = np.eye(2)
    cterm = np.zeros(2)
    for tau, zmp in segs:
        if tau <= 1e-9:
            continue
        A = _seg_A(w, tau)
        cterm = A @ cterm + (np.eye(2) - A) @ np.array([zmp, 0.0])
        M = A @ M
    x0 = np.linalg.solve(np.eye(2) - M, cterm)   # periodic initial state
    # Walk the segments, sampling y at each table phase.
    tab = np.zeros(_W_N)
    phases = np.linspace(0.0, 1.0, _W_N, endpoint=False) * T
    bounds = np.cumsum([0.0] + [s[0] for s in segs])
    for j, tg in enumerate(phases):
        x = x0.copy()
        # advance through whole segments up to tg, then a partial step
        t_left = tg
        for tau, zmp in segs:
            step = min(tau, t_left)
            if step > 1e-12:
                A = _seg_A(w, step)
                x = A @ x + (np.eye(2) - A) @ np.array([zmp, 0.0])
            t_left -= step
            if t_left <= 1e-12:
                break
        tab[j] = x[0]
    # Normalise to unit peak: the LIPM gives the WAVEFORM; GaitParams.lat_hip_amp
    # sets the physiological amplitude (see the field note).
    peak = float(np.abs(tab).max())
    if peak > 1e-9:
        tab = tab / peak
    _lipm_cache[key] = tab
    return tab


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# (C) MEASURED HUMAN frontal + transverse plane (3D normative gait kinematics).
# The sagittal Winter curves already make the stride look human; real motion
# capture also has a frontal (hip ab/adduction) and transverse (hip rotation)
# signature, both ~0 in the legacy shadow. These normative curves are averaged
# human walking -- the honest "mocap" reference -- so the hip-roll AND the
# (entirely untracked) hip-yaw get a principled, achievable, non-zero target.
# Angle in DEG vs % gait cycle, 0% = heel strike. ADDuction +, INTERNAL rot +.
_W_HIP_ABD = [(0, -2), (8, 4), (15, 7), (30, 6), (45, 3), (55, 0),
              (62, -2), (72, -5), (82, -5), (92, -3), (100, -2)]
_W_HIP_ROT = [(0, -4), (12, 3), (30, 6), (50, 4), (62, 0),
              (74, -5), (88, -5), (100, -4)]
HIP_ABD_TAB = _winter_table(_W_HIP_ABD)
HIP_ROT_TAB = _winter_table(_W_HIP_ROT)
_HIP_ABD_MEAN = float(HIP_ABD_TAB.mean())     # de-bias so stance midline ~ 0
_HIP_ROT_MEAN = float(HIP_ROT_TAB.mean())


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# (B) DISTILL-FROM-ACHIEVED: a dynamically-CONSISTENT reference made from a
# walk the robot actually produced (the deployed champion). Feasible by
# construction -> tracking error has a real floor near zero. The table is
# built by build_achieved_gait.py (phase-binned, L/R-symmetrised,
# smoothed) and saved to datasets/g1_achieved_gait.npz: a (_W_N, 13) array of
# leg+waist q over one cycle at global phase. Loaded lazily.
import os as _os
_ACHIEVED_PATH = _os.path.join(_os.path.dirname(__file__), "datasets",
                               "g1_achieved_gait.npz")
_achieved_cache = {}


def _achieved_table():
    """Returns (table (_W_N,13), symmetric_stand (13,)). Lazily loaded."""
    if "tab" not in _achieved_cache:
        if not _os.path.exists(_ACHIEVED_PATH):
            raise FileNotFoundError(
                f"achieved-gait table missing: {_ACHIEVED_PATH}\n"
                "build it: python projects/policies/control/gait/build_achieved_gait.py")
        z = np.load(_ACHIEVED_PATH)
        _achieved_cache["tab"] = z["q"].astype(np.float64)   # (_W_N, 13)
        _achieved_cache["stand"] = z["stand"].astype(np.float64)
    return _achieved_cache["tab"], _achieved_cache["stand"]


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Scalar / numpy implementation (deploy controller)
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def _quintic(s):
    """Smoothstep with zero velocity AND acceleration at both ends."""
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def _leg_ik(dx, dz):
    """Sagittal 2-link IK. Inputs: ankle target relative to the hip_pitch
    anchor, dx forward (m), dz DOWNWARD (m, positive below the hip).
    Returns (q_hip_pitch, q_knee) in G1 joint conventions.

    FK model: theta_t = THIGH_OFF + HIP_SIGN*q_hip (thigh angle from
    vertical, + = forward); theta_s = theta_t + SHANK_OFF + KNEE_SIGN*q_knee;
    ankle = L1*(sin t, cos t) + L2*(sin s, cos s) in (x, z-down). The
    knee-BENT branch (theta_s < theta_t) is always chosen -- the human knee.
    """
    D = math.hypot(dx, dz)
    D = min(D, (L1 + L2) * 0.9995)            # keep off the singularity
    cos_d = (D * D - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_d = max(-1.0, min(1.0, cos_d))
    delta = math.acos(cos_d)                   # |theta_s - theta_t|
    gamma = math.atan2(dx, dz)                 # target line angle from vertical
    psi = math.asin(max(-1.0, min(1.0, L2 * math.sin(delta) / D)))
    theta_t = gamma + psi                      # thigh ahead of the target line
    q_hip = HIP_SIGN * (theta_t - THIGH_OFF)
    q_knee = KNEE_SIGN * (-delta - SHANK_OFF)  # theta_s - theta_t = -delta
    return q_hip, q_knee


def _theta_shank(q_hip, q_knee):
    theta_t = THIGH_OFF + HIP_SIGN * q_hip
    return theta_t + SHANK_OFF + KNEE_SIGN * q_knee


def _ankle_for_foot_pitch(q_hip, q_knee, foot_pitch=0.0):
    """Ankle pitch q that puts the foot at world pitch `foot_pitch`
    (0 = sole flat; negative = toes up)."""
    return ANKLE_AX * (foot_pitch - _theta_shank(q_hip, q_knee))


def _foot_xy(phase, p: GaitParams, t_since_start=None, v_meas=None):
    """Foot trajectory for ONE leg at cycle phase in [0,1).
    Returns (x_rel_pelvis, z_above_ground, swing_weight in [0,1]).
    v_meas: measured forward CoM velocity (m/s); enables the capture-point
    step-placement correction on the swing landing when p.cp_gain>0."""
    L = p.step_len
    if t_since_start is not None and p.ramp_s > 0:
        L *= min(1.0, max(0.0, t_since_start / p.ramp_s))
    if phase < p.duty:                          # STANCE: slide back under pelvis
        s = phase / p.duty
        x = L * (0.5 - s)                       # +L/2 -> -L/2, constant speed
        return x, 0.0, 0.0
    s = (phase - p.duty) / (1.0 - p.duty)       # SWING: smooth catch-up arc
    x = L * (_quintic(s) - 0.5)                 # -L/2 -> +L/2, soft ends
    if v_meas is not None and p.cp_gain > 0.0:
        # CAPTURE-POINT: land the swing foot farther forward when the CoM is
        # moving faster than the gait speed (and shorter when slower) -> the foot
        # steps toward where the CoM is falling, which self-stabilises the walk.
        # Blend the correction in over the swing (quintic) so it's full at touch-down.
        dx = p.cp_gain * (v_meas - p.vx)
        dx = max(-p.cp_max, min(p.cp_max, dx))
        x += _quintic(s) * dx
    z = p.step_height * math.sin(math.pi * s) ** 2
    return x, z, math.sin(math.pi * s)          # swing weight for blends


def _lookup_row_np(tab2d, phi):
    """Linear interp into a periodic (_W_N, K) table; returns (K,)."""
    x = (phi % 1.0) * _W_N
    i = int(x) % _W_N
    f = x - int(x)
    return tab2d[i] * (1.0 - f) + tab2d[(i + 1) % _W_N] * f


def _frontal_yaw_np(phase_rad, phi, phiL, phiR, p: GaitParams, ramp):
    """Frontal (hip-roll, ankle-roll) + transverse (hip-yaw) corrections for
    the lateral/yaw modes. Returns (hrL, hrR, arL, arR, hyL, hyR) in rad."""
    hrL = hrR = arL = arR = hyL = hyR = 0.0
    if p.lateral == "lipm":
        # (A) common-mode hip-roll from the periodic LIPM lateral CoM orbit
        # (+y = toward the LEFT foot; legacy +roll leans toward the RIGHT, so
        # the map carries a minus). Ankle-roll counter-rotates -> feet flat.
        shape = _winter_lookup_np(_lipm_lateral_table(p), phi)   # unit-peak waveform
        hr = -p.lat_hip_amp * shape * p.lat_scale * ramp
        hrL = hrR = hr
        arL = arR = -hr                          # counter-rotate -> feet flat
    elif p.lateral == "human":
        # (C) measured human hip ab/adduction, read at each leg's own phase;
        # L/R joint axes mirror, so the right leg takes the opposite sign.
        aL = (_winter_lookup_np(HIP_ABD_TAB, phiL) - _HIP_ABD_MEAN) * _D2R
        aR = (_winter_lookup_np(HIP_ABD_TAB, phiR) - _HIP_ABD_MEAN) * _D2R
        hrL = +aL * p.lat_scale * ramp
        hrR = -aR * p.lat_scale * ramp
    else:  # "sway" (legacy, no ramp -> preserves the proven standing pose)
        s = p.sway * math.sin(phase_rad)
        hrL = hrR = s
    if p.yaw == "human":
        rL = (_winter_lookup_np(HIP_ROT_TAB, phiL) - _HIP_ROT_MEAN) * _D2R
        rR = (_winter_lookup_np(HIP_ROT_TAB, phiR) - _HIP_ROT_MEAN) * _D2R
        hyL = +rL * p.yaw_scale * ramp
        hyR = -rR * p.yaw_scale * ramp
    return hrL, hrR, arL, arR, hyL, hyR


def _arms_np(phase_rad, p: GaitParams):
    arm = np.zeros(10, dtype=np.float64)
    arm_sw = p.arm_swing * math.sin(phase_rad)
    arm[0] = +arm_sw
    arm[1] = +0.20
    arm[3] = p.elbow_bend
    arm[5] = -arm_sw
    arm[6] = -0.20
    arm[8] = p.elbow_bend
    return arm


def _swing_weight(leg_phi, p: GaitParams):
    if leg_phi >= p.duty:
        s = (leg_phi - p.duty) / (1.0 - p.duty)
        return math.sin(math.pi * s)
    return 0.0


def targets_np(phase_rad, p: GaitParams, t_since_start=None, v_meas=None):
    """13 leg+waist joint targets at gait clock phase_rad (rad; the LEFT leg
    cycle starts its SWING at phase 0 -- same convention as the old CPG).
    Also returns the 10 arm targets used in hold-arms mode and the per-leg
    swing weights (for contact-schedule rewards). v_meas (forward CoM vel, m/s)
    enables the capture-point step-placement correction when p.cp_gain>0."""
    out = np.zeros(NJ, dtype=np.float64)
    phi = (phase_rad / (2.0 * math.pi)) % 1.0

    # Left leg swings first ([0, 1-duty)), mirroring the old sin(th)>0 window.
    phiL = (phi + (1.0 - p.duty)) % 1.0         # shift so swing = tail of cycle
    phiR = (phiL + 0.5) % 1.0

    # Inverted-pendulum pelvis arc: HIGHEST at each leg's mid-stance
    # (left mid-stance at phi=0.9, right at phi=0.4 -> cos(4pi(phi-0.4))).
    bob = p.bob * math.cos(4.0 * math.pi * (phi - 0.4))
    hip_h = (p.pelvis_height + bob) - HIP_DROP - SOLE_DROP

    ramp = 1.0
    if t_since_start is not None and p.ramp_s > 0:
        ramp = min(1.0, max(0.0, t_since_start / p.ramp_s))

    # (B) achieved: the whole 13-vector is the recorded feasible walk, blended
    # in from the standing pose by the launch ramp (no snap).
    if p.style == "achieved":
        tab, stand = _achieved_table()
        q = _lookup_row_np(tab, phi)
        out[:] = stand + ramp * (q - stand)
        return out, _arms_np(phase_rad, p), (_swing_weight(phiL, p),
                                             _swing_weight(phiR, p))

    swings = []
    for base, leg_phi in ((_L_HP, phiL), (_R_HP, phiR)):
        if p.style == "winter":
            # Measured human joint kinematics (Winter normative curves):
            # knee double-bend + ankle push-off included by construction.
            s_sw = (leg_phi - p.duty) / (1.0 - p.duty)
            sw = math.sin(math.pi * s_sw) if leg_phi >= p.duty else 0.0
            q_hip, q_knee, q_ankle = _winter_leg_np(leg_phi, p, ramp)
        else:
            fx, fz, sw = _foot_xy(leg_phi, p, t_since_start, v_meas=v_meas)
            q_hip, q_knee = _leg_ik(fx + p.x0, hip_h - fz)
            q_ankle = _ankle_for_foot_pitch(q_hip, q_knee, -p.ankle_clear * sw)
        out[base + 0] = q_hip
        out[base + 3] = q_knee
        out[base + 4] = q_ankle
        swings.append(sw)

    # Frontal (hip-roll, ankle-roll) + transverse (hip-yaw) plane: legacy
    # sway, LIPM weight transfer (A), or measured human kinematics (C).
    hrL, hrR, arL, arR, hyL, hyR = _frontal_yaw_np(phase_rad, phi, phiL, phiR,
                                                   p, ramp)
    out[_L_HR] += hrL
    out[_R_HR] += hrR
    out[_L_AR] += arL
    out[_R_AR] += arR
    out[_L_HY] += hyL
    out[_R_HY] += hyR

    # Arms: counter-phase to the same-side leg + soft elbows.
    return out, _arms_np(phase_rad, p), (swings[0], swings[1])


# The gait clock phase (rad) at which BOTH feet are in stance (double
# support): phiL = 0.05 (early stance), phiR = 0.55 (early stance). START
# the clock here -- at phase 0 the right foot is MID-SWING, so a settle/
# rest-start at phase 0 stands on a lifted foot and tips over (observed in
# deploy: roll 0.77 within 1 s).
DS_PHASE = 0.65 * 2.0 * math.pi


def standing_pose(p: GaitParams):
    """The standing pose the gait flows out of: EXACTLY the model's output
    at the DOUBLE-SUPPORT phase with zero stride (marching ramp start) --
    both feet planted under the hips, symmetric. Use as NOMINAL for obs
    centering and the deploy settle; start the gait clock at DS_PHASE."""
    legs, _, _ = targets_np(DS_PHASE, p, t_since_start=0.0)
    return legs


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Batched torch implementation (GPU trainer) -- same math, vectorized.
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def targets_torch(phase_t, p: GaitParams, t_since_start_t=None, device=None,
                  v_meas_t=None):
    """phase_t: (n,) torch tensor of gait clock rad. Returns (legs (n,13),
    arms (n,10), swingL (n,), swingR (n,)). v_meas_t: (n,) measured forward CoM
    velocity -> capture-point step-placement correction when p.cp_gain>0."""
    import torch
    n = phase_t.shape[0]
    dev = device or phase_t.device
    two_pi = 2.0 * math.pi
    phi = torch.remainder(phase_t / two_pi, 1.0)
    phiL = torch.remainder(phi + (1.0 - p.duty), 1.0)
    phiR = torch.remainder(phiL + 0.5, 1.0)

    L = torch.full_like(phi, p.step_len)
    if t_since_start_t is not None and p.ramp_s > 0:
        L = L * torch.clamp(t_since_start_t / p.ramp_s, 0.0, 1.0)

    bob = p.bob * torch.cos(4.0 * math.pi * (phi - 0.4))
    hip_h = (p.pelvis_height + bob) - HIP_DROP - SOLE_DROP

    def foot(leg_phi):
        stance = leg_phi < p.duty
        s_st = leg_phi / p.duty
        x_st = L * (0.5 - s_st)
        s_sw = (leg_phi - p.duty) / (1.0 - p.duty)
        q = s_sw * s_sw * s_sw * (10.0 - 15.0 * s_sw + 6.0 * s_sw * s_sw)
        x_sw = L * (q - 0.5)
        if v_meas_t is not None and p.cp_gain > 0.0:
            # capture-point: land farther forward when CoM faster than gait speed
            dx_cp = torch.clamp(p.cp_gain * (v_meas_t - p.vx), -p.cp_max, p.cp_max)
            x_sw = x_sw + q * dx_cp
        z_sw = p.step_height * torch.sin(math.pi * s_sw) ** 2
        sw = torch.sin(math.pi * torch.clamp(s_sw, 0.0, 1.0))
        x = torch.where(stance, x_st, x_sw)
        z = torch.where(stance, torch.zeros_like(z_sw), z_sw)
        swg = torch.where(stance, torch.zeros_like(sw), sw)
        return x, z, swg

    def leg_ik(dx, dz):
        D = torch.sqrt(dx * dx + dz * dz)
        D = torch.clamp(D, max=(L1 + L2) * 0.9995)
        cos_d = (D * D - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        delta = torch.acos(torch.clamp(cos_d, -1.0, 1.0))
        gamma = torch.atan2(dx, dz)
        psi = torch.asin(torch.clamp(L2 * torch.sin(delta) / D, -1.0, 1.0))
        theta_t = gamma + psi
        q_hip = HIP_SIGN * (theta_t - THIGH_OFF)
        q_knee = KNEE_SIGN * (-delta - SHANK_OFF)
        return q_hip, q_knee

    # Shared: launch ramp, device-cached tables, periodic 1-D lookup (the
    # torch mirror of the numpy helpers -> parity).
    ramp_t = torch.ones_like(phi)
    if t_since_start_t is not None and p.ramp_s > 0:
        ramp_t = torch.clamp(t_since_start_t / p.ramp_s, 0.0, 1.0)

    def _dtab(name, arr):
        key = f"_tab_{name}_{dev}"
        t = getattr(targets_torch, key, None)
        if t is None:
            t = torch.tensor(arr, dtype=torch.float32, device=dev)
            setattr(targets_torch, key, t)
        return t

    def _look1(tab_t, leg_phi):
        x = torch.remainder(leg_phi, 1.0) * _W_N
        i0 = x.long() % _W_N
        f = x - x.floor()
        return tab_t[i0] * (1.0 - f) + tab_t[(i0 + 1) % _W_N] * f

    arm_sw = p.arm_swing * torch.sin(phase_t)
    arms = torch.zeros(n, 10, dtype=torch.float32, device=dev)
    arms[:, 0] = arm_sw
    arms[:, 1] = 0.20
    arms[:, 3] = p.elbow_bend
    arms[:, 5] = -arm_sw
    arms[:, 6] = -0.20
    arms[:, 8] = p.elbow_bend

    # (B) achieved: the recorded feasible walk, ramped in from the stand.
    if p.style == "achieved":
        _at, _as = _achieved_table()
        tab = _dtab("achieved", _at)                        # (_W_N, 13)
        stand = _dtab("achieved_stand", _as).unsqueeze(0)   # (1, 13)

        def _lookrow(phiv):
            x = torch.remainder(phiv, 1.0) * _W_N
            i0 = x.long() % _W_N
            f = (x - x.floor()).unsqueeze(1)
            return tab[i0] * (1.0 - f) + tab[(i0 + 1) % _W_N] * f

        out = stand + ramp_t.unsqueeze(1) * (_lookrow(phi) - stand)
        s_swL = torch.clamp((phiL - p.duty) / (1.0 - p.duty), 0.0, 1.0)
        s_swR = torch.clamp((phiR - p.duty) / (1.0 - p.duty), 0.0, 1.0)
        swingL = torch.where(phiL >= p.duty, torch.sin(math.pi * s_swL),
                             torch.zeros_like(phiL))
        swingR = torch.where(phiR >= p.duty, torch.sin(math.pi * s_swR),
                             torch.zeros_like(phiR))
        return out, arms, swingL, swingR

    # Winter style: lookup tables on the device (cached per device).
    if p.style == "winter":
        key = f"_winter_tabs_{dev}"
        tabs = getattr(targets_torch, key, None)
        if tabs is None:
            tabs = tuple(torch.tensor(t, dtype=torch.float32, device=dev)
                         for t in (HIP_TAB, KNEE_TAB, ANKLE_TAB))
            setattr(targets_torch, key, tabs)
        hip_tab_t, knee_tab_t, ankle_tab_t = tabs
        ramp = torch.ones_like(phi)
        if t_since_start_t is not None and p.ramp_s > 0:
            ramp = torch.clamp(t_since_start_t / p.ramp_s, 0.0, 1.0)

        def w_lookup(tab, leg_phi):
            x = torch.remainder(leg_phi, 1.0) * _W_N
            i0 = x.long() % _W_N
            f = x - x.floor()
            return tab[i0] * (1.0 - f) + tab[(i0 + 1) % _W_N] * f

    out = torch.zeros(n, NJ, dtype=torch.float32, device=dev)
    swingL = swingR = None
    for base, leg_phi in ((_L_HP, phiL), (_R_HP, phiR)):
        if p.style == "winter":
            s_sw = (leg_phi - p.duty) / (1.0 - p.duty)
            sw = torch.where(leg_phi >= p.duty,
                             torch.sin(math.pi * torch.clamp(s_sw, 0.0, 1.0)),
                             torch.zeros_like(leg_phi))
            h = w_lookup(hip_tab_t, leg_phi)
            k = w_lookup(knee_tab_t, leg_phi)
            a = w_lookup(ankle_tab_t, leg_phi)
            q_hip_f = p.winter_hip0 - p.winter_hip_scale * (h - _HIP_MEAN) * _D2R
            q_knee_f = p.winter_knee0 + p.winter_knee_scale * (k - _KNEE_MIN) * _D2R
            q_ankle_f = p.winter_ankle0 + p.winter_ankle_scale * (a - _ANKLE_STANCE_MEAN) * _D2R
            sh, sk, sa = WINTER_STAND
            q_hip = sh + ramp * (q_hip_f - sh)
            q_knee = sk + ramp * (q_knee_f - sk)
            q_ankle = sa + ramp * (q_ankle_f - sa)
        else:
            fx, fz, sw = foot(leg_phi)
            q_hip, q_knee = leg_ik(fx + p.x0, hip_h - fz)
            theta_s = (THIGH_OFF + HIP_SIGN * q_hip) + SHANK_OFF + KNEE_SIGN * q_knee
            q_ankle = ANKLE_AX * ((-p.ankle_clear) * sw - theta_s)
        out[:, base + 0] = q_hip
        out[:, base + 3] = q_knee
        out[:, base + 4] = q_ankle
        if base == _L_HP:
            swingL = sw
        else:
            swingR = sw

    # Frontal (hip/ankle-roll) + transverse (hip-yaw) -- mirror of _frontal_yaw_np.
    if p.lateral == "lipm":
        shape = _look1(_dtab("lipm", _lipm_lateral_table(p)), phi)
        hr = -p.lat_hip_amp * shape * p.lat_scale * ramp_t
        out[:, _L_HR] += hr
        out[:, _R_HR] += hr
        out[:, _L_AR] += -hr
        out[:, _R_AR] += -hr
    elif p.lateral == "human":
        abd_t = _dtab("abd", HIP_ABD_TAB)
        aL = (_look1(abd_t, phiL) - _HIP_ABD_MEAN) * _D2R * p.lat_scale * ramp_t
        aR = (_look1(abd_t, phiR) - _HIP_ABD_MEAN) * _D2R * p.lat_scale * ramp_t
        out[:, _L_HR] += aL
        out[:, _R_HR] += -aR
    else:
        sway = p.sway * torch.sin(phase_t)
        out[:, _L_HR] += sway
        out[:, _R_HR] += sway
    if p.yaw == "human":
        rot_t = _dtab("rot", HIP_ROT_TAB)
        rL = (_look1(rot_t, phiL) - _HIP_ROT_MEAN) * _D2R * p.yaw_scale * ramp_t
        rR = (_look1(rot_t, phiR) - _HIP_ROT_MEAN) * _D2R * p.yaw_scale * ramp_t
        out[:, _L_HY] += rL
        out[:, _R_HY] += -rR
    return out, arms, swingL, swingR


# ---------------------------------------------------------------------------
# Self-test (ANALYTIC FK). H1 has no MuJoCo MJCF in-tree (its URDF will
# not load in MuJoCo), so the kinematic constants come straight from the URDF
# via projects/policies/control/gait/tools/urdf_leg_calib.py and the round-trip is checked
# against this model's OWN forward kinematics (the analytic inverse of _leg_ik).
# ---------------------------------------------------------------------------
def _fk_ankle(qh, qk):
    """2-link leg FK: ankle position relative to the hip_pitch anchor.
    Returns (dx forward, dz DOWN). Exact inverse of _leg_ik."""
    theta_t = THIGH_OFF + HIP_SIGN * qh
    theta_s = theta_t + SHANK_OFF + KNEE_SIGN * qk
    x = L1 * math.sin(theta_t) + L2 * math.sin(theta_s)
    z = L1 * math.cos(theta_t) + L2 * math.cos(theta_s)
    return x, z


# Per-joint limits in the 13-slot order (from the URDF; ankle-roll slots are
# kept wide where the robot has no such joint -- the model leaves them at 0).
LIM_LO = np.array([-3.14, -0.43, -0.43, -0.26, -0.87, -0.10, -3.14, -0.43, -0.43, -0.26, -0.87, -0.10, -2.35])
LIM_HI = np.array([ 2.53,  0.43,  0.43,  2.05,  0.52,  0.10,  2.53,  0.43,  0.43,  2.05,  0.52,  0.10,  2.35])


def calibrate():
    print("H1 gait constants are frozen from the URDF. Re-derive with:")
    print("  python projects/policies/control/gait/tools/urdf_leg_calib.py h1")


def _self_test():
    import torch
    p = GaitParams()

    # 1. IK -> FK round trip over the reachable gait workspace (analytic).
    worst = 0.0
    n = 0
    for dx in np.linspace(-0.30, 0.30, 13):
        for dz in np.linspace(0.6, 0.79, 9):
            if math.hypot(dx, dz) > 0.995 * (L1 + L2):
                continue
            qh, qk = _leg_ik(dx, dz)
            fx, fz = _fk_ankle(qh, qk)
            worst = max(worst, math.hypot(fx - dx, fz - dz))
            n += 1
    print(f"IK->FK worst foot-position error: {worst*1000:.3f} mm ({n} targets)")
    assert worst < 1e-6, "IK/FK not self-consistent"

    # 2. flat-ankle check: at the flat-foot ankle q, the foot pitch is ~0.
    qh, qk = _leg_ik(0.04, 0.72)
    qa = _ankle_for_foot_pitch(qh, qk, 0.0)
    foot_pitch = _theta_shank(qh, qk) + ANKLE_AX * qa   # world foot pitch
    print(f"flat-ankle foot pitch: {foot_pitch:+.2e} rad (0 = level)")
    assert abs(foot_pitch) < 1e-9, "ANKLE_AX/foot-level wrong"

    # 3. numpy vs torch parity across the cycle (both styles + 3D modes).
    phases = np.linspace(0, 2 * math.pi, 64, endpoint=False)
    for tag, pm in (("ik", GaitParams()),
                    ("winter", GaitParams(style="winter")),
                    ("3D", GaitParams(style="winter", lateral="human", yaw="human"))):
        legs_np = np.stack([targets_np(ph, pm)[0] for ph in phases])
        legs_t, _, _, _ = targets_torch(torch.tensor(phases, dtype=torch.float32), pm)
        dmax = np.abs(legs_np - legs_t.numpy()).max()
        print(f"[{tag}] numpy vs torch max diff: {dmax:.2e} rad")
        assert dmax < 1e-4, f"{tag} parity broken"
        assert (legs_np >= LIM_LO - 1e-6).all() and (legs_np <= LIM_HI + 1e-6).all(), \
            f"{tag} violates joint limits"

    # 4. stance knee EXTENDS (tall walk, not a crouch) + swing flexes.
    phis = np.linspace(0, 2 * math.pi, 256, endpoint=False)
    knees = np.array([targets_np(ph, p)[0][_L_KN] for ph in phis])
    print(f"knee over cycle: min {knees.min():.3f} (stance, TALL) "
          f"max {knees.max():.3f} (swing flex)")
    assert knees.min() < 0.55, "stance knee not extended -- crouch (raise pelvis_height)"
    assert knees.max() > 0.5, "swing knee not flexing"

    # 5. standing pose symmetric (both feet planted at DS_PHASE).
    stand = standing_pose(p)
    msign = np.array([1, -1, -1, 1, 1, -1])
    asym = np.abs(stand[:6] - msign * stand[6:12]).max()
    print(f"standing pose: hip {stand[_L_HP]:+.3f} knee {stand[_L_KN]:+.3f} "
          f"ankle {stand[_L_AP]:+.3f}, L/R asym {asym:.4f}")
    assert asym < 0.02, "standing pose asymmetric -- DS_PHASE wrong"

    # 6. WINTER style: the two human signatures (knee double-bend, ankle push-off).
    pw = GaitParams(style="winter")
    phis_f = np.linspace(0, 2 * math.pi, 512, endpoint=False)
    kw = np.array([targets_np(ph, pw)[0][_L_KN] for ph in phis_f])
    dk = np.diff(np.sign(np.diff(kw)))
    n_peaks = int((dk < 0).sum())
    print(f"[winter] knee min {kw.min():.3f} max {kw.max():.3f}, "
          f"{n_peaks} local maxima (need >=2: double-bend)")
    assert n_peaks >= 2, "knee double-bend missing"
    aw = np.array([targets_np(ph, pw)[0][_L_AP] for ph in phis_f])
    print(f"[winter] ankle dorsiflex max {aw.max():+.3f}, push-off min {aw.min():+.3f}")
    assert aw.min() < -0.15, "ankle push-off kick missing"

    # 7. 3D frontal/transverse signal present + bounded.
    pc = GaitParams(style="winter", lateral="human", yaw="human")
    lc = np.stack([targets_np(ph, pc)[0] for ph in phases])
    hr = np.degrees(np.abs(lc[:, _L_HR]).max())
    hy = np.degrees(np.abs(lc[:, _L_HY]).max())
    print(f"[3D] hip-roll amp {hr:.1f} deg, hip-yaw amp {hy:.1f} deg")
    assert hr > 1.0, "no lateral weight transfer"
    assert hy > 1.0, "no hip-yaw signal"

    print("SELF-TEST OK")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        calibrate()
    else:
        _self_test()
