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

"""g1_brain -- the DETERMINISTIC "brain" for the Unitree G1 walk.

This is the deterministic alternative to the PPO policy. It is, in spirit,
a hand-written neural net: a fixed function

        targets(13) = BRAIN( state )

evaluated every control tick (62.5 Hz). Instead of learned weights, every
"neuron" is a line of control code you can read, reason about, and tune.

WHY THIS EXISTS
---------------
The kinematic GHOST (projects/policies/control/gait/g1_human_gait.py) already proves the
FEEDFORWARD trajectory is good -- it walks 200+ m played back open-loop with
no physics. The instant you put that same trajectory on the PHYSICS robot it
falls, because the ghost has ZERO feedback: it never reacts to a tilt, a
velocity error, or a height sag. PPO's whole job was to learn that missing
feedback. The brain writes it out by hand.

        brain  =  ghost feedforward   +   deterministic balance feedback

CONTROL STACK (each layer is a method, applied in order; all express their
correction as POSITION-TARGET OFFSETS because the robot is position-PD
controlled at kp~100):

  0. feedforward        the ghost: foot-space plan + IK -> 13 joint targets
  1. foot_placement     capture-point / DCM: steer the SWING leg's hip so the
                        next foothold lands where the CoM is heading. This is
                        the dominant walking stabilizer -- it is what stops the
                        lateral topple ("roll 0.77 in 1 s") the open-loop ghost
                        dies from.
  2. ankle_strategy     fast, low-authority ankle pitch/roll on the STANCE foot
                        vs tilt + tilt-rate + CoM velocity.
  3. posture            stance-hip offsets to keep the pelvis level (the hip
                        strategy: catches what the saturated ankle cannot).
  4. height             knee extension to hold pelvis height against sag.
  5. phase_adaptation   slow the gait clock when off-balance, so the robot can
                        spend longer in double support to recover.

The brain OWNS the gait clock (self.phase): timing is part of behavior.

SIGN CONVENTIONS (G1, matching g1_human_gait + the deploy controller):
  hip_pitch : -q swings the thigh FORWARD   (HIP_SIGN=-1)
  knee      : +q FLEXES (bends, lowers)
  ankle_pitch: forward body-lean (pitch>0) is corrected by a NEGATIVE offset
  hip_roll  : weight-shift / lateral foot placement (sign tuned empirically)

The default gains below are a STARTING POINT for the tuning loop, not a proven
walk. Tune them from the launcher via G1_BRAIN_* env vars (BrainConfig.from_env)
and measure fall-time / distance in run_g1_brain.ps1 -- that is the research
loop. Findings get folded back into these defaults as they are proven.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, asdict

import numpy as np

from projects.policies.control.gait import g1_human_gait as ghg

# Joint indices in LEGS_JOINTS order (identical to g1_human_gait NJ ordering).
L_HP, L_HR, L_HY, L_KN, L_AP, L_AR = 0, 1, 2, 3, 4, 5
R_HP, R_HR, R_HY, R_KN, R_AP, R_AR = 6, 7, 8, 9, 10, 11
WAIST = 12
NJ = 13

# Leg length used to convert a desired foot offset (m) into a hip-joint angle
# offset (rad) by the small-angle approximation dq ~ d_foot / L_leg. ~hip-to-
# ankle reach at the walk posture (L1 + L2 minus the stance crouch).
L_LEG = 0.60
G = 9.81


@dataclass
class BrainState:
    """Everything the brain reads each tick. Mirrors exactly what the deploy
    controller already estimates from the OmniSim Supervisor."""
    roll: float = 0.0          # body roll  (rad)
    pitch: float = 0.0         # body pitch (rad, + = leaning forward)
    yaw: float = 0.0
    roll_rate: float = 0.0     # body-frame angular velocity (rad/s)
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    vx: float = 0.0            # pelvis world-frame linear velocity (m/s)
    vy: float = 0.0
    vz: float = 0.0
    bz: float = 0.78           # pelvis height (m)
    joint_q: np.ndarray = None   # (13,) measured joint positions
    joint_qd: np.ndarray = None  # (13,) measured joint velocities
    t: float = 0.0             # walk time since launch (drives the stride ramp)


@dataclass
class BrainConfig:
    """All deterministic-control gains. Env-overridable (G1_BRAIN_*) so the
    entire controller can be tuned from the launcher with no code edits."""
    # gait reference (the ghost)
    vx: float = 0.4
    freq: float = 1.3
    style: str = "winter"
    ramp_s: float = 2.0
    hip_scale: float = 0.9     # winter waveform amplitude (deploy pre-comp)
    a_lat: float = 0.05        # ghost lateral sway
    a_arm: float = 0.25

    # --- launch: hold a statically-stable DEEP SQUAT, then blend into the
    #     ghost walk. The ghost's own stand pose (winter hip -0.22/knee 0.40)
    #     is CoM-FORWARD and tips on the physics robot; the journey-proven
    #     statically-stable pose is the deeper hip -0.30/knee 0.52 squat. ---
    launch_s: float = 1.5         # hold the stable stand this long before walking
    launch_blend_s: float = 1.0   # blend stand-pose -> ghost-walk pose
    stand_only: bool = False      # never walk (pure stand-balance test)
    stand_hip: float = -0.30
    stand_knee: float = 0.52
    stand_ankle: float = -0.23

    # --- Layer 1: foot placement / capture point (swing leg) ---
    kfp_x: float = 0.10        # sagittal: foot step per (vx - vx_ref)  [m/(m/s)]
    kdcm_x: float = 0.10       # sagittal: foot step per capture-point  vx/omega
    kfp_y: float = 0.08        # lateral: foot step per vy
    kdcm_y: float = 0.12       # lateral: capture point vy/omega
    kroll_fp: float = 0.20     # lateral: extra step per roll angle
    fp_clamp: float = 0.30     # max hip offset from foot placement (rad)

    # --- Layer 2: ankle strategy (stance foot) ---
    #     DEFAULT OFF. KEY FINDING 2026-06-14 (and the prior RL journey): reactive
    #     joint-PD-on-tilt DESTABILISES the G1 stand in OmniSim regardless of sign
    #     -- the pure static pose is strictly more stable than any tilt-PD we tried
    #     (feedback-off fell ~2.0s; +ankle/+hip PD fell 0.75-1.4s). The finite-diff
    #     rate + soft contact make the servo kick rather than restore. Balance must
    #     come from a near-balanced POSE + FOOT PLACEMENT (stepping), not joint PD.
    #     Left as opt-in knobs for experiments / the plain-mujoco regime.
    kp_ap: float = 0.0         # ankle-pitch vs body pitch
    kd_ap: float = 0.0         # ankle-pitch vs pitch rate
    kv_ap: float = 0.0         # ankle-pitch vs CoM vx (DCM-ish)
    kp_ar: float = 0.0         # ankle-roll vs body roll
    kd_ar: float = 0.0         # ankle-roll vs roll rate
    kv_ar: float = 0.0         # ankle-roll vs CoM vy
    ankle_clamp: float = 0.25

    # --- stance width: abduct both hips so the feet sit wider than the hips.
    #     A wider lateral support polygon is the cheapest, strongest defence
    #     against the narrow-stance roll topple (the G1's feet are ~hip-width,
    #     a very narrow base at the tall CoM). Applied to stand AND walk. ---
    stance_width: float = 0.0   # rad of hip-roll abduction per leg (+ = wider)

    # --- Layer 3: posture / trunk (stance hip) ---
    #     DEFAULT OFF for the same reason as the ankle PD (see above): reactive
    #     hip-PD-on-tilt accelerated the topple in OmniSim at every sign/gain
    #     tried. Posture is better held by the pose itself; kept as opt-in.
    kp_post_p: float = 0.0     # stance hip-pitch vs body pitch
    kd_post_p: float = 0.0
    kp_post_r: float = 0.0     # stance hip-roll vs body roll
    kd_post_r: float = 0.0
    hip_clamp: float = 0.30

    # --- Layer 4: height regulation (knee) ---
    kk: float = 0.0            # knee offset per (z_ref - bz); off by default
    z_ref: float = 0.78
    knee_clamp: float = 0.30

    # --- Layer 5: phase adaptation ---
    phase_gate: float = 0.0    # clock slowdown per (roll^2 + pitch^2); 0 = fixed
    phase_floor: float = 0.3

    @classmethod
    def from_env(cls) -> "BrainConfig":
        c = cls()
        for k in asdict(c):
            ev = os.environ.get("G1_BRAIN_" + k.upper())
            if ev is None:
                continue
            cur = getattr(c, k)
            try:
                if isinstance(cur, bool):
                    setattr(c, k, ev.strip().lower() in ("1", "true", "yes", "on"))
                elif isinstance(cur, str):
                    setattr(c, k, ev)
                else:
                    setattr(c, k, type(cur)(ev))
            except (TypeError, ValueError):
                pass
        return c


class G1Brain:
    """Deterministic walk controller. Construct once, call step() each tick."""

    def __init__(self, cfg: BrainConfig | None = None):
        self.cfg = cfg or BrainConfig.from_env()
        c = self.cfg
        self.gp = ghg.GaitParams(
            vx=c.vx, freq=c.freq, style=c.style, ramp_s=c.ramp_s,
            sway=c.a_lat, arm_swing=c.a_arm, winter_hip_scale=c.hip_scale,
        )
        self.phase = float(ghg.DS_PHASE)   # start in double support
        self.arms = ghg.targets_np(self.phase, self.gp, t_since_start=0.0)[1]
        self.elapsed = 0.0                  # brain-owned launch clock
        self.dbg = {}                       # last-tick per-layer contributions

    def reset(self):
        self.phase = float(ghg.DS_PHASE)
        self.elapsed = 0.0

    def _stand_pose(self) -> np.ndarray:
        """The journey-proven deep-squat stand, widened by stance_width."""
        q = np.zeros(NJ, dtype=np.float64)
        q[L_HP] = q[R_HP] = self.cfg.stand_hip
        q[L_KN] = q[R_KN] = self.cfg.stand_knee
        q[L_AP] = q[R_AP] = self.cfg.stand_ankle
        q[L_HR] = +self.cfg.stance_width    # abduct outward (symmetric, wider base)
        q[R_HR] = -self.cfg.stance_width
        return q

    def standing_pose(self) -> np.ndarray:
        """Pose the deploy controller ramps into during its settle (= the
        brain's stable launch stand, so there is no step at hand-off)."""
        return self._stand_pose().astype(np.float32)

    # ---- the brain: state -> 13 joint targets -------------------------------
    def step(self, s: BrainState, dt: float) -> np.ndarray:
        c = self.cfg
        self.elapsed += dt
        # LAUNCH: for the first launch_s seconds (or forever in stand_only mode)
        # hold the statically-stable deep squat with the clock + stride frozen,
        # so the cold articulation settles upright BEFORE it tries to step. Then
        # blend the stand pose into the ghost-walk pose over launch_blend_s.
        standing = c.stand_only or (self.elapsed < c.launch_s)
        if standing:
            legs = self._stand_pose()
            swL = swR = 0.0
            blend = 0.0
        else:
            t_walk = self.elapsed - c.launch_s
            # 0. FEEDFORWARD -- the ghost reference (foot-space plan + IK).
            legs, arms, (swL, swR) = ghg.targets_np(self.phase, self.gp,
                                                    t_since_start=t_walk)
            legs = legs.astype(np.float64)
            self.arms = arms
            legs[L_HR] += c.stance_width          # widen the walking stance too
            legs[R_HR] -= c.stance_width
            blend = min(1.0, max(0.0, t_walk / max(c.launch_blend_s, 1e-6)))
            if blend < 1.0:                       # ease stand-pose -> walk-pose
                legs = (1.0 - blend) * self._stand_pose() + blend * legs
        stL, stR = 1.0 - swL, 1.0 - swR          # stance weights
        omega = math.sqrt(G / max(s.bz, 0.30))   # inverted-pendulum freq

        # 1. FOOT PLACEMENT / CAPTURE POINT (swing leg). Steer the swinging
        #    foot toward where the CoM is heading so it lands under the falling
        #    motion and arrests it. Expressed as a hip-angle offset on the
        #    swing leg (gated by its swing weight).
        dx_fp = c.kfp_x * (s.vx - c.vx) + c.kdcm_x * (s.vx / omega)
        dy_fp = c.kfp_y * s.vy + c.kdcm_y * (s.vy / omega) + c.kroll_fp * s.roll
        dq_hp = float(np.clip(-dx_fp / L_LEG, -c.fp_clamp, c.fp_clamp))  # +x foot -> -hip_pitch
        dq_hr = float(np.clip(dy_fp / L_LEG, -c.fp_clamp, c.fp_clamp))
        legs[L_HP] += dq_hp * swL
        legs[R_HP] += dq_hp * swR
        legs[L_HR] += dq_hr * swL
        legs[R_HR] += dq_hr * swR

        # 2. ANKLE STRATEGY (stance foot). Fast attitude hold; low authority
        #    because the foot is short. Applied to the foot in contact.
        ap = float(np.clip(c.kp_ap * s.pitch + c.kd_ap * s.pitch_rate
                           + c.kv_ap * s.vx, -c.ankle_clamp, c.ankle_clamp))
        ar = float(np.clip(c.kp_ar * s.roll + c.kd_ar * s.roll_rate
                           + c.kv_ar * s.vy, -c.ankle_clamp, c.ankle_clamp))
        legs[L_AP] += ap * stL
        legs[R_AP] += ap * stR
        legs[L_AR] += ar * stL
        legs[R_AR] += ar * stR

        # 3. POSTURE / TRUNK (stance hip). The hip strategy: rotate the body
        #    upright about the stance foot when the ankle saturates.
        hp = float(np.clip(c.kp_post_p * s.pitch + c.kd_post_p * s.pitch_rate,
                           -c.hip_clamp, c.hip_clamp))
        hr = float(np.clip(c.kp_post_r * s.roll + c.kd_post_r * s.roll_rate,
                           -c.hip_clamp, c.hip_clamp))
        legs[L_HP] += hp * stL
        legs[R_HP] += hp * stR
        legs[L_HR] += hr * stL
        legs[R_HR] += hr * stR

        # 4. HEIGHT REGULATION (knee). Extend the knees if the pelvis sags.
        if c.kk != 0.0:
            dk = float(np.clip(c.kk * (c.z_ref - s.bz), -c.knee_clamp, c.knee_clamp))
            legs[L_KN] += dk
            legs[R_KN] += dk

        # 5. PHASE ADAPTATION. Advance the clock only while walking; slow it
        #    when off-balance so the robot lingers in double support to recover.
        gate = 0.0 if standing else 1.0
        if not standing and c.phase_gate != 0.0:
            gate = max(c.phase_floor,
                       min(1.0, 1.0 - c.phase_gate * (s.roll ** 2 + s.pitch ** 2)))
        self.phase = (self.phase + 2.0 * math.pi * c.freq * dt * gate) % (2.0 * math.pi)

        self.dbg = dict(standing=float(standing), blend=blend, swL=swL, swR=swR,
                        dq_hp=dq_hp, dq_hr=dq_hr, ap=ap, ar=ar, hp=hp, hr=hr,
                        gate=gate)
        return legs.astype(np.float32)


if __name__ == "__main__":
    # Smoke test: the brain must produce finite, in-limit targets and, with
    # zero feedback gains, must reduce EXACTLY to the ghost reference.
    LIM_LO = np.array([-2.531, -0.524, -2.758, -0.087, -0.873, -0.262,
                       -2.531, -2.967, -2.758, -0.087, -0.873, -0.262, -2.618])
    LIM_HI = np.array([2.880, 2.967, 2.758, 2.880, 0.524, 0.262,
                       2.880, 0.524, 2.758, 2.880, 0.524, 0.262, 2.618])
    zero = BrainConfig(kfp_x=0, kdcm_x=0, kfp_y=0, kdcm_y=0, kroll_fp=0,
                       kp_ap=0, kd_ap=0, kv_ap=0, kp_ar=0, kd_ar=0, kv_ar=0,
                       kp_post_p=0, kd_post_p=0, kp_post_r=0, kd_post_r=0, kk=0)
    b = G1Brain(zero)
    s = BrainState(joint_q=np.zeros(NJ), joint_qd=np.zeros(NJ), t=3.0)
    worst = 0.0
    for _ in range(400):
        tg = b.step(s, 0.016)
        ref, _, _ = ghg.targets_np(b.phase, b.gp, t_since_start=s.t)  # phase already advanced
        assert np.isfinite(tg).all()
        assert (tg >= LIM_LO - 1e-6).all() and (tg <= LIM_HI + 1e-6).all(), "out of limits"
    print("zero-gain brain == ghost reference: OK")
    # With default feedback gains, perturb and confirm corrections are finite + clamped.
    b2 = G1Brain(BrainConfig())
    s2 = BrainState(roll=0.1, pitch=0.1, roll_rate=0.5, vx=0.5, vy=0.1,
                    joint_q=np.zeros(NJ), joint_qd=np.zeros(NJ), t=3.0)
    for _ in range(50):
        tg = b2.step(s2, 0.016)
        assert np.isfinite(tg).all()
    print("default-gain brain under perturbation: OK")
    print("dbg:", {k: round(v, 4) for k, v in b2.dbg.items()})
