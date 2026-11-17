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

"""g1_brain.py -- THE deterministic "brain" for the G1 walker.

A single hand-engineered function `state -> joint angle targets`, no learned
weights. Goal: make the real (Newton) robot walk as stably as the kinematic
GHOST reference (`g1_human_gait.py`) that RL never fully matched.

This is the ONE consolidated brain for the deterministic-brain effort -- the
several earlier attempts (the canonical capture-point brain here plus a parallel
`projects.policies.control.brain.g1_brain` scratch effort) were folded into this file. It is
the DEFAULT module the g1_walk_deploy harness loads (`G1_BRAIN=1`); it exports
the interface the controller's brain hook expects (`G1Brain`, `BrainState`,
`BrainConfig`). The module stays selectable (`G1_BRAIN_MODULE=...`) so a future
attempt can still ride the same harness + scorer.

APPROACH -- LIPM + capture point (classical humanoid balance)
-------------------------------------------------------------
The ghost is the FEEDFORWARD (ideal foot trajectories). The brain adds the
FEEDBACK -- the balance RL was approximating -- with the Linear Inverted
Pendulum Model and the capture point / Divergent Component of Motion (DCM):

  feedforward  : g1_human_gait foot-space trajectories -> leg IK   (the ghost)
  + capture-pt : step the SWING foot toward the DCM (the controlled fall);
                 lateral is the unstable axis and gets the authority
  + ankle CoP  : stance ankle pitch/roll holds the torso upright (fine balance)
  + weight xfer: hip-roll shifts the CoM over the stance foot (deliberate sway)
  + posture    : hip-pitch trim keeps the trunk vertical
  + adaptive   : the gait clock slows when off-balance (linger to recover)

When balanced, every feedback term -> 0 and the brain reduces EXACTLY to the
ghost, so it is never worse than the open-loop baseline.

`python g1_brain.py` self-tests standalone (no simulator).

FINDINGS LOG (for the deterministic-brain effort)
-------------------------------------------------
Tune offline: `python projects/policies/control/g1_brain_sim.py` (faithful plant, secs
per run). Validate milestones in OmniSim: scripts/dev/run_g1_brain_deploy.ps1.
- v1 (capture-point lateral foot placement + ankle CoP): FALL@~2.3s in ROLL,
  drifting BACKWARD (bx -0.4..-0.8 m). Feedback buys time but does NOT walk.
- The blocking failure mode was NO FORWARD PROPULSION: the body drifts backward
  and tips. We chased swing-foot reach + ankle push-off; both were dead ends
  (k_vtrack on the swing regressed it; ankle PD on compliant legs is too weak).
- FINDING #7 (the fix, 2026-06-14): forward drive is a STANCE-foot placement
  problem, not a push-off problem. Shifting the stride center BACK (`x0`, the
  GaitParams stance offset) puts the CoM ahead of the ankles so the stance slide
  PROPELS the body forward. x0 -0.02 -> -0.03 flips the launch from -0.53 m
  BACKWARD to +0.72 m FORWARD, survival 1.79 -> 2.32 s, mean|tilt| 0.248 ->
  0.161 (offline harness, robust across settle 0.30-0.50 s). The decoupled
  variant (stand at -0.02, walk at -0.04) scores higher on paper (+0.83/2.64)
  but is a SETTLE-TIMING FLUKE -- it collapses at any other settle time, so we
  ship the robust coupled x0=-0.03. Now the wall is a forward PITCH dive ~2.3s
  (the inherent biped-instability ceiling; RL deploy held only ~1.55 s too).
- Still open: a true CoM-over-stance-foot lateral DCM (leg FK) and a foot
  placement that actually catches the pitch dive -- both gated by the divergence
  timescale sqrt(z/g) ~ 0.28 s being << the 0.77 s step period.
ROADMAP: (a) faster corrective steps (higher cadence + small stride) to beat the
0.28 s divergence; (b) leg-FK foot positions -> true lateral DCM; then re-tune
(all gains env-overridable, G1_BRAIN_*).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np

from projects.policies.control.gait import g1_human_gait as ghg

# Joint indices into LEGS_JOINTS (== g1_human_gait order).
_L_HP, _L_HR, _L_HY, _L_KN, _L_AP, _L_AR = 0, 1, 2, 3, 4, 5
_R_HP, _R_HR, _R_HY, _R_KN, _R_AP, _R_AR = 6, 7, 8, 9, 10, 11
_WAIST = 12
NJ = 13

_G = 9.81
_LEG_LEN = ghg.L1 + ghg.L2            # hip_pitch anchor -> ankle, ~0.636 m
# Conservative G1 leg joint limits (rad); the brain clamps so no feedback term
# can slam a joint stop.
LIM_LO = np.array([-1.6, -0.50, -0.6, -0.10, -0.55, -0.30,
                   -1.6, -0.50, -0.6, -0.10, -0.55, -0.30, -0.3])
LIM_HI = np.array([+1.6, +0.50, +0.6, +2.30, +0.55, +0.30,
                   +1.6, +0.50, +0.6, +2.30, +0.55, +0.30, +0.3])


# ---------------------------------------------------------------------- #
#  Interface (matches the g1_walk_deploy brain hook)                     #
# ---------------------------------------------------------------------- #
@dataclass
class BrainState:
    """The robot's senses for one tick, as the deploy controller provides them.
    roll/pitch/yaw + their rates are pre-computed; vx/vy/vz are world-frame
    pelvis linear velocity; t is the gait-ramp time (grows from 0 at launch)."""
    roll: float
    pitch: float
    yaw: float
    roll_rate: float
    pitch_rate: float
    yaw_rate: float
    vx: float
    vy: float
    vz: float
    bz: float
    joint_q: np.ndarray
    joint_qd: np.ndarray
    t: float


@dataclass
class BrainConfig:
    """Every gain -- the 'weights' of the deterministic net. Stability-first."""
    style: str = "ik"               # "ik" foot-space (capture-point handles) / "winter"
    vx: float = 0.4
    freq: float = 1.3
    # forward CoM bias: stance-foot stride-center offset behind the hip anchor
    # (m). THE dominant propulsion lever (see findings #7). The ghost uses
    # x0=-0.02; shifting the stance foot further BACK puts the CoM ahead of the
    # ankles, so the stance slide PROPELS the body forward instead of letting it
    # drift backward and topple. x0=-0.03 flips the launch from -0.56 m BACKWARD
    # (the old default) to +0.72 m FORWARD, survival 1.86 s -> 2.32 s, and mean
    # |tilt| 0.231 -> 0.161 -- robust across settle 0.30-0.50 s (offline harness,
    # g1_brain_sim.py). Too far back (<= -0.05) over-propels into a forward pitch
    # dive; -0.025 still drifts backward. -0.03 is the robust survival optimum.
    x0: float = -0.03
    # capture-point foot placement (the primary stabiliser)
    k_cp_lat: float = 0.9
    k_cp_sag: float = 0.4
    cp_lat_max: float = 0.16
    cp_sag_max: float = 0.12
    # ankle CoP strategy (fine balance, stance foot)
    k_ank_p: float = 1.2
    kd_ank_p: float = 0.10
    k_ank_r: float = 1.6
    kd_ank_r: float = 0.12
    ank_max: float = 0.30
    # forward velocity tracking (research knob, DEFAULT OFF). Tried lengthening
    # the SWING step when below target speed to cancel the open-loop backward
    # drift -- but forward propulsion comes from the STANCE push-off, not the
    # swing reach, so this regressed (drifted back MORE). Left tunable; the real
    # fix is an ankle push-off / stance drive term (see g1-deterministic-brain notes).
    k_vtrack: float = 0.0
    vx_target: float = 0.4
    # lateral position term (research knob, DEFAULT OFF): roll as the lateral-
    # offset proxy. Theoretically the DCM needs it, but raw roll includes the
    # intended sway lean, so it over-drove lateral and rolled over. Needs the
    # CoM offset relative to the STANCE foot, not raw roll.
    k_lat_pos: float = 0.0
    # lateral weight transfer (closed-loop sway). 0.4 (was 0.6) is marginally
    # more stable with the x0 forward bias (+0.83 m / 2.64 s vs +0.80 / 2.56).
    k_wt: float = 0.4
    wt_max: float = 0.12
    # posture / torso pitch regulation
    k_torso_p: float = 0.5
    torso_max: float = 0.20
    # adaptive phase
    phase_gate_tilt: float = 3.0
    phase_gate_floor: float = 0.35
    # LIPM pendulum height
    com_height: float = 0.72

    @classmethod
    def from_env(cls, env=None):
        env = os.environ if env is None else env
        c = cls()

        def _f(name, cur):
            v = env.get(name)
            return float(v) if v not in (None, "") else cur
        c.style = env.get("G1_BRAIN_STYLE", c.style)
        c.vx = _f("G1_GAIT_VX", c.vx)
        c.freq = _f("G1_GAIT_FREQ", c.freq)
        c.x0 = _f("G1_BRAIN_X0", c.x0)
        c.k_cp_lat = _f("G1_BRAIN_KCP_LAT", c.k_cp_lat)
        c.k_cp_sag = _f("G1_BRAIN_KCP_SAG", c.k_cp_sag)
        c.k_ank_r = _f("G1_BRAIN_KANK_R", c.k_ank_r)
        c.k_ank_p = _f("G1_BRAIN_KANK_P", c.k_ank_p)
        c.k_wt = _f("G1_BRAIN_KWT", c.k_wt)
        c.k_vtrack = _f("G1_BRAIN_KVTRACK", c.k_vtrack)
        c.k_lat_pos = _f("G1_BRAIN_KLAT_POS", c.k_lat_pos)
        c.vx_target = _f("G1_GAIT_VX", c.vx_target)
        c.phase_gate_tilt = _f("G1_BRAIN_PHASE_GATE", c.phase_gate_tilt)
        return c


# ---------------------------------------------------------------------- #
#  The brain                                                             #
# ---------------------------------------------------------------------- #
class G1Brain:
    def __init__(self, cfg: BrainConfig | None = None):
        self.cfg = cfg or BrainConfig()
        self.gp = ghg.GaitParams(style=self.cfg.style, vx=self.cfg.vx,
                                 freq=self.cfg.freq, x0=self.cfg.x0)
        self.phase = ghg.DS_PHASE
        self._om = 2.0 * math.pi * self.gp.freq
        self._omega = math.sqrt(_G / max(self.cfg.com_height, 0.2))
        self._nominal = ghg.standing_pose(self.gp).astype(np.float64)
        # Arm targets (10), exposed for the deploy hold_arms(); the gait model
        # gives counter-phase swing. Updated each step().
        self.arms = ghg.targets_np(self.phase, self.gp, t_since_start=0.0)[1]

    def standing_pose(self) -> np.ndarray:
        return self._nominal.astype(np.float32)

    # ------------------------------------------------------------------ #
    def step(self, s: BrainState, dt: float) -> np.ndarray:
        cfg = self.cfg
        roll, pitch, yaw = s.roll, s.pitch, s.yaw

        # 1. state estimation: CoM velocity in the yaw-aligned horizontal frame
        cy, sy = math.cos(yaw), math.sin(yaw)
        com_vx = +cy * s.vx + sy * s.vy        # forward CoM velocity
        com_vy = -sy * s.vx + cy * s.vy        # leftward CoM velocity

        # 2. adaptive phase clock (slow when tilted -> linger to recover)
        tilt2 = roll * roll + pitch * pitch
        gate = max(cfg.phase_gate_floor, 1.0 - cfg.phase_gate_tilt * tilt2)
        self.phase = (self.phase + self._om * dt * gate) % (2.0 * math.pi)

        # 3. feedforward: the ghost (legs + arms)
        legs, self.arms, (swL, swR) = ghg.targets_np(self.phase, self.gp,
                                                     t_since_start=max(0.0, s.t))
        out = legs.astype(np.float64).copy()
        left_stance = swL <= swR

        # 4a. capture point: DCM = lateral_offset + velocity/omega. Lateral uses
        # the full state (roll as the position proxy + measured v_lat); sagittal
        # adds a VELOCITY-TRACKING drive (step forward harder when below target)
        # to cancel the open-loop gait's backward drift -- the residual's job.
        dcm_fwd = com_vx / self._omega
        dcm_lat = cfg.k_lat_pos * roll + com_vy / self._omega
        v_err = cfg.vx_target - com_vx                 # >0 == too slow / drifting back
        cp_lat = _clip(cfg.k_cp_lat * dcm_lat, cfg.cp_lat_max)
        cp_sag = _clip(cfg.k_cp_sag * dcm_fwd + cfg.k_vtrack * v_err, cfg.cp_sag_max)
        # place the SWING foot toward the capture point (faded by swing weight)
        if left_stance:
            self._place(out, base=_R_HP, hr=_R_HR, sw=swR,
                        cp_sag=cp_sag, cp_lat=cp_lat, foot_sign=-1.0)
        else:
            self._place(out, base=_L_HP, hr=_L_HR, sw=swL,
                        cp_sag=cp_sag, cp_lat=cp_lat, foot_sign=+1.0)

        # 4b. ankle CoP on the STANCE leg (fine balance)
        ap = _clip(cfg.k_ank_p * pitch + cfg.kd_ank_p * s.pitch_rate, cfg.ank_max)
        ar = _clip(cfg.k_ank_r * roll + cfg.kd_ank_r * s.roll_rate, cfg.ank_max)
        if left_stance:
            out[_L_AP] += ap; out[_L_AR] += ar
        else:
            out[_R_AP] += ap; out[_R_AR] += ar

        # 4c. lateral weight transfer: shift the CoM over the stance foot
        wt = _clip(cfg.k_wt * dcm_lat, cfg.wt_max)
        out[_L_HR] += wt
        out[_R_HR] += wt

        # 4d. posture: keep the trunk vertical (hip-pitch trim, both legs)
        tp = _clip(cfg.k_torso_p * pitch, cfg.torso_max)
        out[_L_HP] += tp
        out[_R_HP] += tp

        # 5. clamp -- no feedback term may slam a joint stop
        np.clip(out, LIM_LO, LIM_HI, out=out)
        return out.astype(np.float32)

    # ------------------------------------------------------------------ #
    def _place(self, out, *, base, hr, sw, cp_sag, cp_lat, foot_sign):
        """Re-IK the SWING leg with the capture-point correction on its foot
        target: sagittal shifts the landing fore/aft; lateral converts a foot-y
        offset to hip-roll (foot_y ~ LEG_LEN*sin(hip_roll))."""
        if sw <= 1e-3:
            return
        q_hip = out[base + 0]
        q_knee = out[base + 3]
        dx, dz = _fk_foot(q_hip, q_knee)
        nq_hip, nq_knee = ghg._leg_ik(dx + sw * cp_sag, dz)
        out[base + 0] = nq_hip
        out[base + 3] = nq_knee
        out[base + 4] = ghg._ankle_for_foot_pitch(nq_hip, nq_knee,
                                                  -self.gp.ankle_clear * sw)
        dy = foot_sign * sw * cp_lat
        out[hr] += math.asin(max(-0.6, min(0.6, dy / _LEG_LEN)))


# ---------------------------------------------------------------------- #
#  helpers                                                               #
# ---------------------------------------------------------------------- #
def _clip(v, m):
    return max(-m, min(m, v))


def _fk_foot(q_hip, q_knee):
    """Ankle position relative to the hip_pitch anchor (dx fwd, dz DOWN).
    Inverse of g1_human_gait._leg_ik."""
    theta_t = ghg.THIGH_OFF + ghg.HIP_SIGN * q_hip
    theta_s = theta_t + ghg.SHANK_OFF + ghg.KNEE_SIGN * q_knee
    dx = ghg.L1 * math.sin(theta_t) + ghg.L2 * math.sin(theta_s)
    dz = ghg.L1 * math.cos(theta_t) + ghg.L2 * math.cos(theta_s)
    return dx, dz


# ---------------------------------------------------------------------- #
#  Standalone self-test                                                  #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    dt = 0.004

    def mk(t, roll=0.0, pitch=0.0, vx=0.0, vy=0.0, rr=0.0, pr=0.0):
        return BrainState(roll=roll, pitch=pitch, yaw=0.0,
                          roll_rate=rr, pitch_rate=pr, yaw_rate=0.0,
                          vx=vx, vy=vy, vz=0.0, bz=0.74,
                          joint_q=np.zeros(NJ), joint_qd=np.zeros(NJ), t=t)

    print("=== g1_brain (control) self-test ===")
    brain = G1Brain()
    ghost_ref = ghg.targets_np(brain.phase + brain._om * dt, brain.gp,
                               t_since_start=dt)[0]
    out = brain.step(mk(dt), dt)
    dev = float(np.max(np.abs(out - ghost_ref)))
    print(f"balanced: max|brain-ghost| = {dev:.4f} rad "
          f"({'OK reduces to ghost' if dev < 0.05 else 'DEVIATES'})")

    o_bal = brain.step(mk(0.5), dt)
    o_fall = brain.step(mk(0.5 + dt, roll=0.10, vy=0.25), dt)
    dhr = float(o_fall[_L_HR] - o_bal[_L_HR])
    print(f"falling-left: hip-roll reaction = {dhr:+.4f} rad "
          f"({'OK' if abs(dhr) > 1e-3 else 'NONE'})")

    bad = 0
    b2 = G1Brain()
    for i in range(750):
        o = b2.step(mk(i * dt, vx=0.4), dt)
        if not np.all(np.isfinite(o)) or np.any(o < LIM_LO - 1e-6) or np.any(o > LIM_HI + 1e-6):
            bad += 1
    print(f"3 s walk: {bad} bad/limit-violating ticks ({'OK' if bad == 0 else 'FAIL'})")
    print(f"standing_pose: {np.round(b2.standing_pose(), 3)}")
    print("done.")
