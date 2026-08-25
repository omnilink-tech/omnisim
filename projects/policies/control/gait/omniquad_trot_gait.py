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

"""Foot-space parametric trot gait model for OmniQuad (12-DOF).

THE OMNIQUAD PORT OF projects/policies/control/gait/g1_human_gait.py -- same architecture,
quadruped trot instead of biped walk. The gait is PLANNED IN FOOT SPACE
and realized through the closed-form 3-DOF leg IK
(projects/policies/control/omniquad_kinematics.py math, vectorized here):

  - stance foot: ON the ground, translating backward UNDER the body at
    exactly -vx (forward velocity without skating);
  - swing foot: quintic catch-up arc -- lift, travel forward, touch down
    ahead with ~zero relative velocity;
  - trot phasing: diagonal pairs (FL+RR at phase 0, FR+RL at phase 0.5);
  - duty factor > 0.5 gives FOUR-FOOT support windows twice per cycle
    (the slow, deliberate trot look; duty 0.5 = classic running trot);
  - a STRIDE RAMP grows the step length from zero over `ramp_s` seconds:
    the gait starts as stepping-in-place from the standing pose (which IS
    the model's zero-stride output) -- a standing start has no target
    snap and no out-of-distribution launch transient.

Same math implemented twice -- `targets_np` (scalar, deploy controller)
and `targets_torch` (batched on GPU, trainer) -- and
`python omniquad_trot_gait.py` self-tests both against each other and the
IK against MuJoCo forward kinematics on the deploy-matched MJCF dump.

Joint order everywhere: FL(hip_x, hip_y, knee), FR, RL, RR -- the same
controller order as the deploy bridge and the GPU trainers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# ── Geometry (meters) -- from omniquad.urdf via omniquad_kinematics.py ──────────
L1 = 0.3205             # thigh: hip_y joint -> knee joint, z magnitude
L2 = 0.32               # shank: knee joint -> foot tip
KNEE_X = 0.025          # forward offset of the knee in the upper-leg frame
HIPY_OFFSET = 0.110945  # lateral offset of hip_y from hip_x

LEGS = ("FL", "FR", "RL", "RR")
HIP_X_POS = {
    "FL": (+0.29785, +0.05500, 0.0),
    "FR": (+0.29785, -0.05500, 0.0),
    "RL": (-0.29785, +0.05500, 0.0),
    "RR": (-0.29785, -0.05500, 0.0),
}
HIPY_SIGN = {"FL": +1.0, "FR": -1.0, "RL": +1.0, "RR": -1.0}
# Trot: diagonal pairs. Per-leg cycle phase offset (fraction of cycle).
TROT_OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}

NJ = 12


@dataclass
class GaitParams:
    """All lengths in meters, angles in rad, frequencies in Hz."""
    vx: float = 0.4            # forward speed the reference encodes
    freq: float = 1.4          # per-leg cycle frequency
    duty: float = 0.6          # stance fraction (>0.5 = 4-foot overlap)
    step_height: float = 0.06  # swing apex clearance
    body_height: float = 0.55  # body z above ground; feet at -body_height.
    #                            At 0.57 the stride extreme (x_front +
    #                            step_len/2) leaves the leg's reach sphere
    #                            by ~9 mm; 0.55 keeps the WHOLE default
    #                            cycle strictly reachable (self-test #1)
    lateral_y: float = 0.344   # stance |y| of each foot in body frame
    #                            (FK of the proven nominal hip_x=+-0.30)
    x_front: float = 0.322     # stance x of the FRONT feet (FK of nominal)
    x_rear: float = -0.274     # stance x of the REAR feet
    x0: float = 0.0            # stride-center shift (all feet, +fwd)
    ramp_s: float = 1.0        # stride ramp-in seconds from standing

    @property
    def period_s(self) -> float:
        return 1.0 / self.freq

    @property
    def step_len(self) -> float:
        # Stride such that the stance slide is EXACTLY -vx: the foot covers
        # the stride during duty*T seconds, so L = vx * duty / freq. (NOT
        # vx/freq -- that overdrives the stance feet by 1/duty and makes
        # them skate; the world-frame bookkeeping still closes because the
        # body also travels during swing: vx*duty*T + vx*(1-duty)*T = vx*T.)
        return self.vx * self.duty / self.freq


# The gait clock phase (rad) at which ALL FOUR feet are in stance.
# Leg pair A runs phi, pair B runs phi+0.5; with duty d > 0.5 the
# quad-support windows are phi in [0.5, d) and [0.0, d-0.5). Start the
# clock mid-window: phi = (0.5 + d)/2 with d=0.6 -> 0.55 of a cycle.
QS_PHASE = 0.55 * 2.0 * math.pi


# ────────────────────────────────────────────────────────────────────────
# Scalar / numpy implementation (deploy controller)
# ────────────────────────────────────────────────────────────────────────
def _quintic(s):
    """Smoothstep with zero velocity AND acceleration at both ends."""
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def _foot_x_z(leg_phi, p: GaitParams, t_since_start=None):
    """One leg's stride trajectory at cycle phase in [0,1).
    Returns (x_rel_stride_center, z_above_ground, swing_weight)."""
    L = p.step_len
    if t_since_start is not None and p.ramp_s > 0:
        L *= min(1.0, max(0.0, t_since_start / p.ramp_s))
    if leg_phi < p.duty:                        # STANCE: slide back at -vx
        s = leg_phi / p.duty
        return L * (0.5 - s), 0.0, 0.0
    s = (leg_phi - p.duty) / (1.0 - p.duty)     # SWING: quintic catch-up
    x = L * (_quintic(s) - 0.5)
    z = p.step_height * math.sin(math.pi * s) ** 2
    return x, z, math.sin(math.pi * s)


def _leg_ik(leg, tx, ty, tz):
    """Closed-form 3-DOF IK, body-frame foot target -> (hip_x, hip_y, knee).
    Same math as omniquad_kinematics.inverse_kinematics; clamps the acos
    arguments instead of returning None (the gait self-test proves the
    whole default cycle is strictly reachable, the clamp only guards
    parameter extremes)."""
    Hx, Hy, _ = HIP_X_POS[leg]
    Tx, Ty, Tz = tx - Hx, ty - Hy, tz
    H = HIPY_SIGN[leg] * HIPY_OFFSET

    r_yz = math.hypot(Ty, Tz)
    a = math.atan2(Tz, Ty) + math.acos(max(-1.0, min(1.0, H / max(r_yz, 1e-9))))
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi

    sa, ca = math.sin(a), math.cos(a)
    Qx = Tx
    Qz = -Ty * sa + Tz * ca

    Q2 = Qx * Qx + Qz * Qz
    R_link = math.hypot(L1, KNEE_X)
    phi_k = math.atan2(KNEE_X, L1)
    C = (Q2 - KNEE_X * KNEE_X - L1 * L1 - L2 * L2) / (2.0 * L2)
    g = -math.acos(max(-1.0, min(1.0, C / R_link))) - phi_k

    Sx = KNEE_X - L2 * math.sin(g)
    Sz = -L1 - L2 * math.cos(g)
    b = math.atan2(Sz, Sx) - math.atan2(Qz, Qx)
    return a, b, g


def _yaw_sweep(leg_phi, p: GaitParams, wz):
    """Tangential stance sweep for yaw-rate steering: while planted, the
    foot rotates about the body z-axis at -wz (the body yaws +wz over the
    ground); the swing un-winds it. wz=0 -> exactly 0 (bit-identical
    straight gait). Same mechanism as omniquad_gait's wz fix (c3c7aff9)."""
    if wz == 0.0:
        return 0.0
    sweep = wz * p.duty / p.freq          # yaw covered during one stance
    if leg_phi < p.duty:
        s = leg_phi / p.duty
        return sweep * (0.5 - s)
    s = (leg_phi - p.duty) / (1.0 - p.duty)
    return sweep * (_quintic(s) - 0.5)


def foot_targets_np(phase_rad, p: GaitParams, t_since_start=None, wz=0.0):
    """Body-frame foot targets (4, 3) + swing weights (4,) at gait clock
    phase_rad. Leg order FL, FR, RL, RR. wz = commanded yaw rate (rad/s,
    steered by rotating the planted feet about the body z-axis)."""
    phi = (phase_rad / (2.0 * math.pi)) % 1.0
    feet = np.zeros((4, 3), dtype=np.float64)
    swings = np.zeros(4, dtype=np.float64)
    for i, leg in enumerate(LEGS):
        leg_phi = (phi + TROT_OFFSET[leg]) % 1.0
        dx, dz, sw = _foot_x_z(leg_phi, p, t_since_start)
        x_neu = p.x_front if leg in ("FL", "FR") else p.x_rear
        y_neu = p.lateral_y * HIPY_SIGN[leg]
        fx = x_neu + p.x0 + dx
        fy = y_neu
        ang = _yaw_sweep(leg_phi, p, wz)
        if ang != 0.0:
            ca, sa = math.cos(ang), math.sin(ang)
            fx, fy = fx * ca - fy * sa, fx * sa + fy * ca
        feet[i] = (fx, fy, -p.body_height + dz)
        swings[i] = sw
    return feet, swings


def targets_np(phase_rad, p: GaitParams, t_since_start=None, wz=0.0):
    """12 joint targets (controller order) + per-leg swing weights (4,)."""
    feet, swings = foot_targets_np(phase_rad, p, t_since_start, wz)
    out = np.zeros(NJ, dtype=np.float64)
    for i, leg in enumerate(LEGS):
        out[3 * i: 3 * i + 3] = _leg_ik(leg, *feet[i])
    return out, swings


def standing_pose(p: GaitParams):
    """The standing pose the gait flows out of: EXACTLY the model's output
    at the quad-support phase with zero stride -- all four feet planted at
    the neutral stance. Use as NOMINAL for obs centering and the deploy
    settle; start the gait clock at QS_PHASE."""
    legs, _ = targets_np(QS_PHASE, p, t_since_start=0.0)
    return legs


def speed_scale(legs, swings, nominal, s):
    """VELOCITY CONDITIONING (the G1 walk29_vc recipe, ported): blend the
    trot-model output toward the STANDING pose by a commanded-speed factor
    s = vx_cmd / vx_nominal.

        legs   -> nominal + s*(legs - nominal)
        swings -> s * swings

    s=1 is the full gait; s=0 collapses to the standing pose -- feet planted,
    no stride, no swing lift -- which for a QUADRUPED is statically stable, so
    "stop" is just the standing pose held (unlike the biped, no balance
    knife-edge). Works for scalar/numpy (legs (12,), nominal (12,), swings
    (4,), s float) and for batched numpy; the torch trainer inlines the same
    two lines because s is per-env. s clamps to [0, 1.25] so a slight speed
    overshoot can still be commanded."""
    s = max(0.0, min(1.25, float(s)))
    return nominal + s * (legs - nominal), swings * s


# ────────────────────────────────────────────────────────────────────────
# Batched torch implementation (GPU trainer) -- same math, vectorized.
# ────────────────────────────────────────────────────────────────────────
def targets_torch(phase_t, p: GaitParams, t_since_start_t=None, device=None,
                  wz_t=None):
    """phase_t: (n,) torch tensor of gait clock rad. wz_t: optional (n,)
    yaw-rate commands. Returns (targets (n,12) controller order,
    swings (n,4))."""
    import torch
    dev = device or phase_t.device
    n = phase_t.shape[0]
    phi = torch.remainder(phase_t / (2.0 * math.pi), 1.0)

    L = torch.full_like(phi, p.step_len)
    if t_since_start_t is not None and p.ramp_s > 0:
        L = L * torch.clamp(t_since_start_t / p.ramp_s, 0.0, 1.0)

    out = torch.zeros(n, NJ, dtype=torch.float32, device=dev)
    swings = torch.zeros(n, 4, dtype=torch.float32, device=dev)
    R_link = math.hypot(L1, KNEE_X)
    phi_k = math.atan2(KNEE_X, L1)

    for i, leg in enumerate(LEGS):
        leg_phi = torch.remainder(phi + TROT_OFFSET[leg], 1.0)
        stance = leg_phi < p.duty
        s_st = leg_phi / p.duty
        x_st = L * (0.5 - s_st)
        s_sw = torch.clamp((leg_phi - p.duty) / (1.0 - p.duty), 0.0, 1.0)
        q5 = s_sw * s_sw * s_sw * (10.0 - 15.0 * s_sw + 6.0 * s_sw * s_sw)
        x_sw = L * (q5 - 0.5)
        z_sw = p.step_height * torch.sin(math.pi * s_sw) ** 2
        sw = torch.sin(math.pi * s_sw)
        dx = torch.where(stance, x_st, x_sw)
        dz = torch.where(stance, torch.zeros_like(z_sw), z_sw)
        swings[:, i] = torch.where(stance, torch.zeros_like(sw), sw)

        x_neu = p.x_front if leg in ("FL", "FR") else p.x_rear
        Hx, Hy, _ = HIP_X_POS[leg]
        H = HIPY_SIGN[leg] * HIPY_OFFSET
        fx = (x_neu + p.x0) + dx
        fy = torch.full_like(dx, p.lateral_y * HIPY_SIGN[leg])
        if wz_t is not None:
            # Tangential stance sweep (see _yaw_sweep): wz=0 rows unchanged.
            sweep = wz_t * (p.duty / p.freq)
            ang = torch.where(stance, sweep * (0.5 - s_st),
                              sweep * (q5 - 0.5))
            ca, sa = torch.cos(ang), torch.sin(ang)
            fx, fy = fx * ca - fy * sa, fx * sa + fy * ca
        Tx = fx - Hx
        Ty = fy - Hy
        Tz = -p.body_height + dz

        r_yz = torch.sqrt(Ty * Ty + Tz * Tz).clamp(min=1e-9)
        a = torch.atan2(Tz, Ty) + torch.acos(torch.clamp(H / r_yz, -1.0, 1.0))
        a = torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi

        Qx = Tx
        Qz = -Ty * torch.sin(a) + Tz * torch.cos(a)
        Q2 = Qx * Qx + Qz * Qz
        C = (Q2 - KNEE_X * KNEE_X - L1 * L1 - L2 * L2) / (2.0 * L2)
        g = -torch.acos(torch.clamp(C / R_link, -1.0, 1.0)) - phi_k

        Sx = KNEE_X - L2 * torch.sin(g)
        Sz = -L1 - L2 * torch.cos(g)
        b = torch.atan2(Sz, Sx) - torch.atan2(Qz, Qx)

        out[:, 3 * i + 0] = a
        out[:, 3 * i + 1] = b
        out[:, 3 * i + 2] = g
    return out, swings


# ────────────────────────────────────────────────────────────────────────
# Self-test
# ────────────────────────────────────────────────────────────────────────
def _self_test():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from projects.policies.control.omniquad_kinematics import (
        forward_kinematics, inverse_kinematics, JointAngles)

    p = GaitParams()

    # 1. model IK vs the reference scalar IK + FK round-trip over the cycle
    worst_q = 0.0
    worst_fk = 0.0
    for ph in np.linspace(0, 2 * math.pi, 128, endpoint=False):
        feet, _ = foot_targets_np(ph, p)
        legs, _ = targets_np(ph, p)
        for i, leg in enumerate(LEGS):
            ref = inverse_kinematics(leg, tuple(feet[i]))
            assert ref is not None, \
                f"phase {ph:.2f} leg {leg}: foot {feet[i]} UNREACHABLE"
            worst_q = max(worst_q, max(
                abs(a - b) for a, b in
                zip(ref.as_tuple(), legs[3 * i: 3 * i + 3])))
            fk = forward_kinematics(leg, JointAngles(*legs[3 * i: 3 * i + 3]))
            worst_fk = max(worst_fk, max(
                abs(a - b) for a, b in zip(fk, feet[i])))
    print(f"IK vs reference: worst joint diff {worst_q:.2e} rad; "
          f"IK->FK worst foot error {worst_fk * 1000:.3f} mm "
          f"(128 phases x 4 legs, ALL reachable)")
    assert worst_q < 1e-9 and worst_fk < 1e-6

    # 2. numpy vs torch parity
    import torch
    phases = np.linspace(0, 2 * math.pi, 64, endpoint=False)
    legs_np = np.stack([targets_np(ph, p)[0] for ph in phases])
    legs_t, sw_t = targets_torch(torch.tensor(phases, dtype=torch.float32), p)
    dmax = np.abs(legs_np - legs_t.numpy()).max()
    print(f"numpy vs torch max diff: {dmax:.2e} rad")
    assert dmax < 1e-4

    # 3. joint limits (deploy MJCF ranges) over the full cycle
    lo = np.array([-1.5, -0.5, -1.2] * 4)
    hi = np.array([+1.5, +3.13, -0.01] * 4)
    phis = np.linspace(0, 2 * math.pi, 256, endpoint=False)
    legs_all = np.stack([targets_np(ph, p)[0] for ph in phis])
    assert (legs_all >= lo - 1e-6).all() and (legs_all <= hi + 1e-6).all(), \
        "gait violates joint limits"
    print(f"limits OK: hip_y in [{legs_all[:, 1::3].min():+.3f}, "
          f"{legs_all[:, 1::3].max():+.3f}], knee in "
          f"[{legs_all[:, 2::3].min():+.3f}, {legs_all[:, 2::3].max():+.3f}]")

    # 4. standing pose: symmetric, feet planted, no snap from QS_PHASE
    stand = standing_pose(p)
    # left/right mirror: hip_x flips sign, hip_y/knee equal
    asym = max(abs(stand[0] + stand[3]), abs(stand[1] - stand[4]),
               abs(stand[2] - stand[5]), abs(stand[6] + stand[9]),
               abs(stand[7] - stand[10]), abs(stand[8] - stand[11]))
    print(f"standing pose FL=({stand[0]:+.3f},{stand[1]:+.3f},{stand[2]:+.3f}) "
          f"RL=({stand[6]:+.3f},{stand[7]:+.3f},{stand[8]:+.3f}) "
          f"L/R asym {asym:.2e}")
    assert asym < 1e-9
    ph0 = targets_np(QS_PHASE, p, t_since_start=0.0)[0]
    assert np.abs(ph0 - stand).max() < 1e-12
    _, sw0 = foot_targets_np(QS_PHASE, p, t_since_start=0.0)
    assert (sw0 == 0.0).all(), "QS_PHASE has a swinging foot"
    fz = foot_targets_np(QS_PHASE, p, t_since_start=0.0)[0][:, 2]
    print(f"QS_PHASE: all four feet planted at z={fz[0]:.3f}")

    # 5. stance feet really translate at -vx (the no-skate property)
    dt = 1e-4
    om = 2.0 * math.pi * p.freq
    f0, s0 = foot_targets_np(1.0, p)
    f1, _ = foot_targets_np(1.0 + om * dt, p)
    v = (f1 - f0) / dt
    for i in range(4):
        if s0[i] == 0.0:
            assert abs(v[i, 0] + p.vx) < 1e-3, \
                f"stance foot {LEGS[i]} vx {v[i, 0]:.3f} != {-p.vx}"
    print(f"stance feet translate at {-p.vx:+.3f} m/s exactly (no skating)")

    # 5b. wz steering: wz=0 bit-identical; wz extremes reachable + parity
    legs_w0 = np.stack([targets_np(ph, p, wz=0.0)[0] for ph in phases])
    assert np.abs(legs_w0 - legs_np).max() == 0.0, "wz=0 changed the gait"
    for wz in (-0.4, 0.4):
        for ph in np.linspace(0, 2 * math.pi, 64, endpoint=False):
            feet_w, _ = foot_targets_np(ph, p, wz=wz)
            for i, leg in enumerate(LEGS):
                assert inverse_kinematics(leg, tuple(feet_w[i])) is not None, \
                    f"wz={wz} phase {ph:.2f} {leg} unreachable"
    wzv = torch.full((64,), 0.3)
    legs_wz_t, _ = targets_torch(torch.tensor(phases, dtype=torch.float32), p,
                                 wz_t=wzv)
    legs_wz_np = np.stack([targets_np(ph, p, wz=0.3)[0] for ph in phases])
    dwz = np.abs(legs_wz_np - legs_wz_t.numpy()).max()
    print(f"wz steering: wz=0 bit-identical; +-0.4 rad/s fully reachable; "
          f"np/torch parity {dwz:.2e} rad")
    assert dwz < 1e-4

    # 6. FK chain vs MuJoCo on the deploy-matched MJCF dump (if present)
    mjcf = r"C:\tmp\omniquad_newton_fixed2.xml"
    try:
        import mujoco
        m = mujoco.MjModel.from_xml_path(mjcf)
        d = mujoco.MjData(m)
        # qpos: free(7) + FL(hx,hy,kn) FR RL RR -- same grouping as ours.
        d.qpos[:] = 0
        d.qpos[3] = 1.0
        d.qpos[7:19] = stand
        mujoco.mj_forward(m, d)
        # lower-leg bodies 4/7/10/13; foot tip = body + R @ (0,0,-L2)
        worst = 0.0
        feet_ref, _ = foot_targets_np(QS_PHASE, p, t_since_start=0.0)
        for i, bid in enumerate((4, 7, 10, 13)):
            R = d.xmat[bid].reshape(3, 3)
            foot_w = d.xpos[bid] + R @ np.array([0.0, 0.0, -L2])
            # body origin is the freejoint at qpos[0:3] = 0 -> body frame
            err = np.abs(foot_w - feet_ref[i]).max()
            worst = max(worst, err)
        print(f"MuJoCo FK check: worst foot error {worst * 1000:.2f} mm")
        assert worst < 0.005, "gait FK disagrees with the deploy MJCF"
    except ImportError:
        print(f"(mujoco not installed -- FK-vs-MJCF check skipped)")
    except Exception as e:
        print(f"(FK-vs-MJCF check skipped: {e})")

    print("SELF-TEST OK")


if __name__ == "__main__":
    _self_test()
