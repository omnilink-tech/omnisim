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

"""Loader + self-calibration for the DAIR ContactNets cube-toss dataset.

Lane 1R scores OmniSim against MEASURED motion instead of a closed form. This
module owns the dataset: what the columns mean, what the physical constants
are, and -- the part that matters -- a calibration that is RE-DERIVED from the
data every time rather than copied from a paper.

THE DATA
--------
550 tosses of an acrylic cube onto a wooden table, tracked by AprilTags on
every face through TagSLAM. Vendored under ``data/`` from
https://github.com/DAIRLab/dair_pll (BSD-3-Clause, (c) 2022 DAIR Lab); the
licence travels with it. Each ``<i>.pt`` is a float64 tensor of shape
(121, 13):

    cols 0:4   quaternion          (unit norm, verified on load)
    cols 4:7   position, m
    cols 7:10  angular velocity, rad/s
    cols 10:13 linear velocity, m/s

Row 0 is the initial condition -- pose AND both velocities, which is why this
dataset is replayable at all and most are not.

THE PUBLISHED BASELINES this lane exists to sit beside (Acosta, Yang & Posa,
"Validating Robotics Simulators on Real-World Impacts", RA-L 2022,
arXiv:2110.00541), rolling out every toss from its measured IC:

    Drake    13.5 +/- 8.2 % of cube width      rotation 16.5 +/- 20.0 deg
    Bullet   14.9 +/- 8.9 %                    rotation 16.5 +/- 20.2 deg
    MuJoCo   25.1 +/- 10.8 %                   rotation 21.7 +/- 21.4 deg

OmniSim's backend IS MuJoCo, so ~25% is the sanity check before it is a
result: land far from it and the harness is wrong, not the physics.

THREE LANDMINES, ALL MEASURED HERE RATHER THAN ASSUMED
------------------------------------------------------
1. **The inertia is published three ways and two of them are wrong for this
   cube.** The URDF says I = 8.1e-4 with a 0.1048 m box; the shipped MJCF
   says 6.167e-4 with a 0.10 m box; Acosta's Table I prints 8.1e-3 (10x, a
   typo). Arithmetic settles it: a UNIFORM 0.10 m cube of 0.37 kg has
   m*a^2/6 = 6.167e-4 exactly, so the MJCF is a convenience model that
   assumed uniform density. Uniform at 0.1048 m would be 6.77e-4, and the
   measured 8.1e-4 exceeds it -- mass toward the shell, which is what a
   hollow acrylic cube is. **The URDF is canonical.**

2. **The repo's own ``DT = 0.0068`` (examples/contactnets_simple.py) is 0.64%
   too large** and is what makes the data appear to have g = 9.48 m/s^2. The
   published rate, 148 Hz, is correct.

3. **The tracked lengths carry a ~2.2% scale factor** relative to the URDF's
   nominal cube. Two independent witnesses agree to 0.07%:

       settled rest height / half-width          = 0.9779
       free-flight gravity at 148 Hz / 9.81      = 0.9786

   One scale explains both. Left uncorrected it is a systematic bias that a
   scorer would silently charge to the contact model, so ``calibrate()``
   measures it on every run and ``load()`` can undo it.

   ⚠ The Acosta baselines above were computed WITHOUT this correction, so a
   scale-corrected score is NOT directly comparable to them. Run both:
   ``scale="none"`` to sit beside the paper, ``scale="metric"`` for a
   physically self-consistent replay. Reporting only one is how a benchmark
   quietly stops meaning anything.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"

#: Canonical physical constants -- from data/contactnets_cube.urdf, see §1.
CUBE_EDGE_M = 0.1048
CUBE_HALF_M = CUBE_EDGE_M / 2.0
CUBE_MASS_KG = 0.37
CUBE_INERTIA = 8.1e-4          # kg m^2, isotropic
CUBE_MU_STATIC = 0.15
CUBE_RESTITUTION = 0.125       # from the contact-nets experiment.json

#: Published sampling rate. NOT the repo's rounded DT=0.0068 -- see §2.
SAMPLE_HZ = 148.0
DT_S = 1.0 / SAMPLE_HZ

GRAVITY = 9.81
N_STEPS = 121                  # rows per trajectory
N_TRAJ = 550


def traj_paths():
    ps = sorted(DATA.glob("*.pt"), key=lambda p: int(p.stem))
    if not ps:
        raise FileNotFoundError(
            "no trajectories under %s -- the dataset is vendored, so an empty "
            "dir means the checkout is incomplete, not that you must download "
            "it" % DATA)
    return ps


def _load_raw(path):
    """-> (121,13) float64. torch is used only as a deserialiser."""
    import torch
    t = torch.load(str(path), weights_only=False)
    a = np.asarray(t.detach().cpu().numpy(), dtype=np.float64).squeeze()
    if a.ndim != 2 or a.shape[1] != 13:
        raise ValueError("%s: expected (N,13), got %r" % (path.name, a.shape))
    return a


def split(a):
    """(121,13) -> quat(...,4), pos(...,3), omega(...,3), vel(...,3)."""
    return a[:, 0:4], a[:, 4:7], a[:, 7:10], a[:, 10:13]


# ---------------------------------------------------------------------------
# conventions -- MEASURED, not assumed. See `probe_conventions()`.
# ---------------------------------------------------------------------------
#: Quaternion storage order and the frame the angular velocity lives in.
#: Both were determined from free-flight kinematics rather than read off a
#: README, because a cube settles flat under either quaternion convention --
#: a resting pose is geometrically plausible read as w-first OR as xyzw, so
#: eyeballing the data cannot distinguish them and a wrong guess silently
#: rotates every scored orientation.
#:
#: The discriminator is that during free flight there is no torque, so omega
#: is constant and qdot = 0.5 * q (x) [0,omega] must predict the next sample.
#: Measured over 1283 free-flight samples, one-step prediction error:
#:
#:     wxyz + body frame    median 0.0199 deg      <-- correct
#:     wxyz + world frame   median 1.8441 deg
#:     xyzw + body frame    median 1.9318 deg
#:     xyzw + world frame   median 2.4589 deg
#:
#: A 93x separation. Re-derive it any time with `--probe-conventions`.
QUAT_ORDER = "wxyz"
OMEGA_FRAME = "body"


def quat_mul(a, b):
    """Hamilton product, both operands (..., 4) in wxyz order."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    w1, x1, y1, z1 = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    w2, x2, y2, z2 = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], axis=-1)


def quat_to_matrix(q):
    """wxyz -> 3x3 rotation matrix (body -> world)."""
    w, x, y, z = np.asarray(q, dtype=np.float64)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def omega_to_world(q, omega_body):
    """Rotate a BODY-frame angular velocity into the world frame.

    ⚠ Required before handing omega to OmniSim. `Node.setVelocity()` takes a
    6-vector in WORLD coordinates, and this dataset stores omega in the BODY
    frame (see OMEGA_FRAME). Passing the raw column through is a silent error
    that leaves every toss spinning about the wrong axis while still looking
    like a plausible tumble.
    """
    return quat_to_matrix(q) @ np.asarray(omega_body, dtype=np.float64)


def quat_to_axis_angle(q):
    """wxyz -> ([ax, ay, az], angle_rad), the .wbt `rotation` field form."""
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q)
    if q[0] < 0:                      # canonical hemisphere; q and -q are one
        q = -q
    w = min(1.0, max(-1.0, q[0]))
    ang = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-12:
        return [0.0, 0.0, 1.0], 0.0   # .wbt rejects a zero axis
    return [float(q[1] / s), float(q[2] / s), float(q[3] / s)], float(ang)


def probe_conventions(paths=None, stride=5):
    """Re-measure QUAT_ORDER / OMEGA_FRAME from the data. -> {hypothesis: deg}"""
    paths = list(paths or traj_paths())[::stride]
    acc = {}
    for p in paths:
        a = _load_raw(p)
        q, pos, w, v = split(a)
        z, vz = pos[:, 2], v[:, 2]
        ok = (z > CUBE_HALF_M + 0.03) & (vz < -0.05)
        idx = np.where(ok)[0]
        if len(idx) < 8:
            continue
        brk = np.where(np.diff(idx) != 1)[0]
        idx = idx[:brk[0] + 1] if len(brk) else idx
        idx = idx[2:-1]
        if len(idx) < 5:
            continue
        for conv in ("wxyz", "xyzw"):
            Q = q if conv == "wxyz" else q[:, [3, 0, 1, 2]]
            for frame in ("body", "world"):
                qk, qn, om = Q[idx], Q[idx + 1], w[idx]
                pure = np.concatenate([np.zeros((len(idx), 1)), om], axis=1)
                qd = (0.5 * quat_mul(qk, pure) if frame == "body"
                      else 0.5 * quat_mul(pure, qk))
                pred = qk + DT_S * qd
                pred /= np.linalg.norm(pred, axis=1, keepdims=True)
                d = np.abs(np.sum(pred * qn, axis=-1)).clip(0, 1)
                acc.setdefault("%s_%s" % (conv, frame), []).append(
                    np.degrees(2 * np.arccos(d)))
    return {k: float(np.median(np.concatenate(v))) for k, v in acc.items()}


# ---------------------------------------------------------------------------
# calibration -- re-derived, never asserted
# ---------------------------------------------------------------------------

def calibrate(paths=None, min_r2=0.99999):
    """Measure the dataset's own length scale and effective gravity.

    Two independent estimators, deliberately kept separate so that their
    AGREEMENT is the evidence rather than an average being the answer:

      * ``scale_from_rest``    settled centre height / nominal half-width.
        Uses the dataset's documented convention that the table is z = 0.
      * ``scale_from_gravity`` free-flight vertical acceleration / 9.81, at
        the published 148 Hz.

    Only genuinely ballistic segments feed the second one -- a parabola fit
    gated at R^2 > `min_r2`. Without that gate the estimate is contaminated by
    bounces and slides and reads 6.3 +/- 4.0 m/s^2, i.e. noise.
    """
    paths = list(paths or traj_paths())
    rest, accel = [], []
    for p in paths:
        a = _load_raw(p)
        _, pos, _, vel = split(a)
        z, vz = pos[:, 2], vel[:, 2]

        tail_z, tail_v = pos[-10:, 2], vel[-10:]
        if np.abs(tail_v).max() < 0.02 and tail_z.std() < 1e-3:
            rest.append(float(tail_z.mean()))

        ok = (z > CUBE_HALF_M + 0.03) & (vz < -0.05)
        if not ok.any():
            continue
        idx = np.where(ok)[0]
        brk = np.where(np.diff(idx) != 1)[0]
        idx = idx[:brk[0] + 1] if len(brk) else idx
        if len(idx) > 10:
            idx = idx[2:]                    # drop the filter's edge transient
        if len(idx) < 8:
            continue
        k = np.arange(len(idx))
        y = z[idx]
        c = np.polyfit(k, y, 2)
        if y.var() <= 0:
            continue
        r2 = 1.0 - ((y - np.polyval(c, k)).var() / y.var())
        if r2 > min_r2:
            accel.append(-2.0 * c[0])        # m per step^2, assumption-free

    rest = np.asarray(rest)
    accel = np.asarray(accel)
    med_acc = float(np.median(accel))
    g_meas = med_acc / (DT_S ** 2)

    out = {
        "n_settled": int(rest.size),
        "n_ballistic": int(accel.size),
        "rest_z_m": float(np.median(rest)),
        "d2z_dstep2": med_acc,
        "d2z_dstep2_std": float(accel.std()),
        "g_measured_at_148hz": g_meas,
        "scale_from_rest": float(np.median(rest)) / CUBE_HALF_M,
        "scale_from_gravity": g_meas / GRAVITY,
        "dt_s": DT_S,
        "sample_hz": SAMPLE_HZ,
    }
    out["scale"] = 0.5 * (out["scale_from_rest"] + out["scale_from_gravity"])
    out["scale_witness_disagreement"] = abs(
        out["scale_from_rest"] - out["scale_from_gravity"])
    return out


def load(index, scale="none", calib=None):
    """One trajectory, ready to replay.

    scale:
      "none"   -- as published. Comparable to the Acosta baselines.
      "metric" -- positions and linear velocities divided by the measured
                  scale, so free flight integrates at 9.81 and the cube rests
                  at its true half-width. Quaternions and angular velocities
                  are untouched: a length rescale does not affect either.

    Returns a dict with `quat`, `pos`, `omega`, `vel`, `t`, and the initial
    state broken out for convenience.
    """
    p = DATA / ("%d.pt" % index) if isinstance(index, int) else Path(index)
    a = _load_raw(p)
    q, pos, w, v = split(a)

    n = np.linalg.norm(q, axis=1)
    if not np.allclose(n, 1.0, atol=1e-6):
        raise ValueError("%s: quaternion norm drifts to %.6g -- columns 0:4 "
                         "are not a unit quaternion, so the layout assumption "
                         "is wrong" % (p.name, float(np.abs(n - 1).max())))

    s = 1.0
    if scale == "metric":
        c = calib or calibrate()
        s = c["scale"]
        pos = pos / s
        v = v / s
    elif scale != "none":
        raise ValueError("scale must be 'none' or 'metric', got %r" % scale)

    return {
        "index": int(p.stem) if p.stem.isdigit() else -1,
        "quat": q, "pos": pos, "omega": w, "vel": v,
        "t": np.arange(a.shape[0]) * DT_S,
        "dt_s": DT_S, "n_steps": int(a.shape[0]),
        "scale_mode": scale, "scale_applied": s,
        "q0": q[0].copy(), "p0": pos[0].copy(),
        "w0": w[0].copy(), "v0": v[0].copy(),
    }


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(
        description="Inspect + self-calibrate the ContactNets cube dataset.")
    ap.add_argument("--json", help="write the calibration here")
    ap.add_argument("--limit", type=int, default=0,
                    help="calibrate on the first N trajectories only")
    ap.add_argument("--probe-conventions", action="store_true",
                    help="re-derive QUAT_ORDER / OMEGA_FRAME from free-flight "
                         "kinematics instead of trusting the constants")
    a = ap.parse_args(argv)

    if a.probe_conventions:
        r = probe_conventions()
        best = min(r, key=r.get)
        print("one-step quaternion prediction error in free flight (deg):")
        for k in sorted(r, key=r.get):
            print("  %-14s %10.5f %s" % (k, r[k], "<-- best" if k == best else ""))
        want = "%s_%s" % (QUAT_ORDER, OMEGA_FRAME)
        print("declared: %s" % want)
        if best != want:
            print("MISMATCH: the data disagrees with the declared convention")
            return 1
        return 0

    ps = traj_paths()
    if a.limit:
        ps = ps[:a.limit]
    c = calibrate(ps)

    print("ContactNets cube toss -- %d trajectories, %d steps at %.0f Hz"
          % (len(ps), N_STEPS, SAMPLE_HZ))
    print("  cube: %.4f m edge, %.3f kg, I=%.3g kg m^2, mu=%.2f, e=%.3f"
          % (CUBE_EDGE_M, CUBE_MASS_KG, CUBE_INERTIA, CUBE_MU_STATIC,
             CUBE_RESTITUTION))
    print("  settled samples %d | ballistic samples %d"
          % (c["n_settled"], c["n_ballistic"]))
    print("  rest z            %.5f m  (half-width %.5f)"
          % (c["rest_z_m"], CUBE_HALF_M))
    print("  g at 148 Hz       %.4f m/s^2" % c["g_measured_at_148hz"])
    print("  scale from rest   %.5f" % c["scale_from_rest"])
    print("  scale from gravity%.5f" % c["scale_from_gravity"])
    print("  witnesses differ  %.5f  <- agreement IS the evidence"
          % c["scale_witness_disagreement"])
    print("  => scale          %.5f" % c["scale"])
    if a.json:
        Path(a.json).write_text(json.dumps(c, indent=2), encoding="utf-8")
        print("[dataset] wrote %s" % a.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
