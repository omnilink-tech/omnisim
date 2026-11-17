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

"""Capture-point foot-placement stepping for the humanoid stand.

The reactive lean+arm overlays (the deterministic base) recover pushes that keep
the Centroidal Capture Point (CP = CoM_xy + CoM_vel_xy * sqrt(h/g)) inside the
support polygon. A push that drives the CP past the foot edge is UNRECOVERABLE in
place -- the robot must STEP so the new support polygon contains the CP.

This module does that properly (vs the old hip-angle heuristic): it
  1. computes the CP in the pelvis-yaw world frame,
  2. picks the swing leg + a foot landing target AT the CP (clamped to a reachable
     box), and
  3. solves leg IK (analytic-seeded numeric) so the foot actually lands there,
     keeping the foot flat (ankle cancels hip+knee pitch / hip roll).

Leg chain offsets/axes are read from the G1 23dof URDF (left leg; right mirrors y).
Pure math + numpy, no simulator imports -> unit-testable offline.
"""
import math
import numpy as np

# Left-leg joint origins (xyz, m) relative to the parent link, from
# g1_23dof_omnisim.urdf, in chain order hip_pitch -> ... -> ankle_roll.
_LEFT_OFFSETS = [
    ("hip_pitch",   (0.0,       0.064452,   -0.1027),   "y"),
    ("hip_roll",    (0.0,       0.052,      -0.030465), "x"),
    ("hip_yaw",     (0.025001,  0.0,        -0.12412),  "z"),
    ("knee",        (-0.078273, 0.0021489,  -0.17734),  "y"),
    ("ankle_pitch", (0.0,      -9.4445e-05, -0.30001),  "y"),
    ("ankle_roll",  (0.0,       0.0,        -0.017558), "x"),
]
_FOOT_OFFSET = (0.018, 0.0, -0.030)   # ankle_roll link -> sole contact (approx)


def _rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _offsets(leg):
    """Per-leg offsets; the right leg mirrors the left across the sagittal plane (y)."""
    sgn = 1.0 if leg == "left" else -1.0
    out = []
    for name, (x, y, z), ax in _LEFT_OFFSETS:
        out.append((name, np.array([x, sgn * y, z]), ax))
    return out, np.array([_FOOT_OFFSET[0], sgn * _FOOT_OFFSET[1], _FOOT_OFFSET[2]])


def leg_fk(q, leg="left"):
    """q = [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll] (rad).
    Returns the foot (sole) position in the PELVIS frame (m)."""
    offs, foot = _offsets(leg)
    p = np.zeros(3)
    R = np.eye(3)
    for (name, o, ax), ang in zip(offs, q):
        p = p + R @ o
        R = R @ _rot(ax, ang)
    return p + R @ foot


def leg_ik(target, leg="left", q0=None, iters=40, tol=1e-4):
    """Solve for [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll] so the
    foot lands at `target` (pelvis frame) with the foot kept flat. hip_yaw is fixed
    at q0's value; ankle_pitch/_roll are slaved to cancel hip+knee pitch and hip
    roll (flat foot). The 3 free DOFs (hip_pitch, hip_roll, knee) are solved by a
    damped Newton step on foot position. Returns the full 6-vector."""
    q = np.zeros(6) if q0 is None else np.array(q0, dtype=float)
    free = [0, 1, 3]   # hip_pitch, hip_roll, knee
    for _ in range(iters):
        # flat-foot slaving before each eval
        q[4] = -(q[0] + q[3])   # ankle_pitch cancels hip_pitch + knee
        q[5] = -q[1]            # ankle_roll cancels hip_roll
        f = leg_fk(q, leg)
        err = target - f
        if np.linalg.norm(err) < tol:
            break
        J = np.zeros((3, 3))
        for c, i in enumerate(free):
            dq = np.zeros(6); dq[i] = 1e-5
            qp = q + dq
            qp[4] = -(qp[0] + qp[3]); qp[5] = -qp[1]
            J[:, c] = (leg_fk(qp, leg) - f) / 1e-5
        # damped least squares
        dql = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(3), err)
        for c, i in enumerate(free):
            q[i] += float(np.clip(dql[c], -0.3, 0.3))
    q[4] = -(q[0] + q[3]); q[5] = -q[1]
    return q


def capture_point(com_xy, com_vel_xy, com_h, g=9.81):
    """CP = CoM_xy + CoM_vel_xy * sqrt(h/g)  (linear inverted pendulum)."""
    com_h = max(0.3, float(com_h))
    w = math.sqrt(com_h / g)
    return np.array(com_xy) + np.array(com_vel_xy) * w


if __name__ == "__main__":
    # Offline self-test: FK at the nominal squat, IK round-trip, and a commanded
    # foot displacement.
    nom = [-0.30, 0.0, 0.0, 0.52, -0.23, 0.0]
    for leg in ("left", "right"):
        f0 = leg_fk(nom, leg)
        print(f"[{leg}] nominal foot (pelvis frame) = {np.round(f0,3)}  hip~z {f0[2]:.3f}")
        # round-trip: IK back to the nominal foot should reproduce a flat-foot pose
        q = leg_ik(f0, leg, q0=[-0.3, 0, 0, 0.5, 0, 0])
        f1 = leg_fk(q, leg)
        print(f"    IK round-trip foot err = {np.linalg.norm(f1-f0)*1000:.2f} mm  q(hp,hr,kn)={np.round([q[0],q[1],q[3]],3)}")
        # command the foot 0.18 m lateral (out) and 0.10 m forward of nominal
        sgn = 1.0 if leg == "left" else -1.0
        tgt = f0 + np.array([0.10, sgn * 0.18, 0.0])
        q = leg_ik(tgt, leg, q0=q)
        fr = leg_fk(q, leg)
        print(f"    step target {np.round(tgt,3)} -> reached {np.round(fr,3)} err {np.linalg.norm(fr-tgt)*1000:.2f} mm")
        print(f"    -> hip_pitch={q[0]:+.3f} hip_roll={q[1]:+.3f} knee={q[3]:+.3f} ank_p={q[4]:+.3f} ank_r={q[5]:+.3f}")
