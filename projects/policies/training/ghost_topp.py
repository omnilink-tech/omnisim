#!/usr/bin/env python3
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

"""ghost_topp.py -- TIME-PARAMETERIZE a ghost: separate the PATH from its TIMING.

A ghost is a geometric path plus a clock. Those are independent, and only the clock decides
trackability. Measured, 3-arm controlled experiment (see build_step_turn_ghost.py docstring):

    peak lateral pelvis velocity is the controlling variable; COM margin is not.
    0.287 m/s -> the from-scratch policy over-spins and falls.  0.229 m/s -> it survives.

Our previous fix (`--vby-budget`) enforced that by scaling `cycle_s` UNIFORMLY -- it slowed the WHOLE
turn 10.40 -> 13.04 s to cure a peak that occurs only during the weight-shift phases. That is the
crudest possible time parameterization. Path-parameterization theory (TOPP / TOPP-RA, Pham & Pham,
IEEE T-RO 2018, arXiv:1707.07239) is exactly the primitive for doing it properly: take a FIXED
geometric path and solve for the speed profile along it subject to the robot's limits.

WHAT THIS IMPLEMENTS (and, honestly, what it does not)
Constraints handled: per-segment VELOCITY bounds -- body-frame lateral / forward pelvis velocity,
pelvis yaw rate, and per-joint velocity limits read from the URDF. For velocity-only constraints the
time-optimal profile is separable: each segment's duration is set by its own most-binding limit, so no
forward-backward integration is needed.

⛔ We deliberately do NOT speed any segment up. Pure time-optimality would drive dt -> 0 wherever the
path barely moves (our settle frames), which both erases the settles and manufactures large
accelerations that no acceleration constraint is here to catch. So this is MINIMAL-STRETCH
parameterization: `dt_i = max(dt_original, dt_required)`. It is never faster than the source ghost
anywhere, never violates the caps, and stretches only where the caps bite -- strictly better than the
uniform rescale, which stretches everywhere. Adding acceleration/torque bounds (the full TOPP-RA
forward-backward pass) would let us safely speed up the slack segments too; that is the next step.

The result is re-sampled back onto a UNIFORM time grid, because the deploy plays the lut at a constant
phase rate. Non-uniform timing therefore shows up as a non-uniform distribution of FRAMES along the
path: more frames where the motion must be slow, fewer where it may be quick.

Usage:
  python ghost_topp.py <in_lut.json> <out_lut.json> --vby 0.229 [--vfwd F] [--vyaw R]
                       [--joint-vel-frac 0.9] [--nb N]
"""
import argparse
import json
import math
import os

import numpy as np

RT = os.environ.get("OMNISIM_HOME", os.getcwd())
URDF = os.path.join(RT, "projects/robots/unitree/g1/urdf/g1_23dof_omnisim.urdf")

# per-frame arrays that must be resampled onto the new clock
FRAME_KEYS = ("leg_lut", "wb_lut", "att_lut", "root_lut", "arm_lut", "elbow_lut")


def _urdf_vel_limits():
    import xml.etree.ElementTree as ET
    lim = {}
    try:
        for j in ET.parse(URDF).getroot().iter("joint"):
            l = j.find("limit")
            if l is not None and l.get("velocity"):
                lim[j.get("name")] = float(l.get("velocity"))
    except Exception:
        pass
    return lim


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def segment_durations(lut, vby, vfwd, vyaw, jfrac):
    """Minimum duration of each path segment such that every velocity cap holds, but never faster
    than the source ghost's own timing (see the no-speedup note above)."""
    NB = int(lut["nb"])
    root = np.asarray(lut["root_lut"], float)
    wb = np.asarray(lut["wb_lut"], float)
    names = lut["wb_joints"]
    vlim = _urdf_vel_limits()
    dt0 = float(lut["cycle_s"]) / NB

    dts = np.zeros(NB - 1)
    binding = []
    for i in range(NB - 1):
        d = root[i + 1] - root[i]
        c, s = math.cos(root[i, 3]), math.sin(root[i, 3])
        d_lat = -s * d[0] + c * d[1]
        d_fwd = c * d[0] + s * d[1]
        d_yaw = _wrap(root[i + 1, 3] - root[i, 3])

        cand = [(abs(d_lat) / vby, "vby")] if vby > 0 else []
        if vfwd > 0:
            cand.append((abs(d_fwd) / vfwd, "vfwd"))
        if vyaw > 0:
            cand.append((abs(d_yaw) / vyaw, "vyaw"))
        for j, n in enumerate(names):
            L = vlim.get(n, 0.0) * jfrac
            if L > 0:
                cand.append((abs(wb[i + 1, j] - wb[i, j]) / L, "joint:" + n))
        cand.append((dt0, "original"))          # never speed a segment up
        dt, who = max(cand)
        dts[i] = dt
        binding.append(who)
    return dts, binding, dt0


def resample(lut, dts, nb_out):
    """Non-uniform timing -> uniform frame grid (the deploy plays the lut at constant phase rate)."""
    NB = int(lut["nb"])
    t = np.concatenate([[0.0], np.cumsum(dts)])
    T = float(t[-1])
    tn = np.linspace(0.0, T, nb_out)
    out = dict(lut)
    for k in FRAME_KEYS:
        if k not in lut:
            continue
        a = np.asarray(lut[k], float)
        if a.shape[0] != NB:
            continue
        if k == "root_lut":                      # unwrap yaw before interpolating
            a = a.copy()
            a[:, 3] = np.unwrap(a[:, 3])
        b = np.stack([np.interp(tn, t, a[:, c]) for c in range(a.shape[1])], axis=1)
        out[k] = [[float(v) for v in row] for row in b]
    out["nb"] = int(nb_out)
    out["cycle_s"] = T
    out["freq"] = 1.0 / T
    return out, T


def peak_lateral(lut):
    rl = np.asarray(lut["root_lut"], float)
    dtb = float(lut["cycle_s"]) / int(lut["nb"])
    vw = (np.roll(rl, -1, 0) - rl) / dtb
    vw[-1] = vw[-2]
    cy, sy = np.cos(rl[:, 3]), np.sin(rl[:, 3])
    return float(np.abs(-sy * vw[:, 0] + cy * vw[:, 1]).max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("dst")
    ap.add_argument("--vby", type=float, default=0.229,
                    help="cap on |body-frame lateral pelvis velocity| (m/s). MEASURED trackable envelope "
                         "for the 90deg turn family, from-scratch: 0.229 (0.287 falls).")
    ap.add_argument("--vfwd", type=float, default=0.0, help="cap on |forward pelvis velocity| (0 = off)")
    ap.add_argument("--vyaw", type=float, default=0.0, help="cap on |pelvis yaw rate| rad/s (0 = off)")
    ap.add_argument("--joint-vel-frac", type=float, default=0.9, help="fraction of URDF joint velocity limits")
    ap.add_argument("--nb", type=int, default=0, help="output frame count (0 = keep source nb)")
    a = ap.parse_args()

    lut = json.load(open(a.src))
    nb_out = a.nb or int(lut["nb"])
    T0 = float(lut["cycle_s"])
    v0 = peak_lateral(lut)

    # Re-sampling the non-uniform knots back onto a UNIFORM frame grid interpolates across them and can
    # overshoot the cap slightly (measured 0.231 vs a 0.229 cap). Close the loop: shrink the working cap
    # by the observed overshoot and re-solve until the RESAMPLED lut actually honours the contract.
    work = a.vby
    for _ in range(6):
        dts, binding, dt0 = segment_durations(lut, work, a.vfwd, a.vyaw, a.joint_vel_frac)
        out, T = resample(lut, dts, nb_out)
        v1 = peak_lateral(out)
        if v1 <= a.vby * 1.001:
            break
        work *= a.vby / v1
    out["source"] = lut.get("source", "") + " | ghost_topp.py minimal-stretch time-parameterization"
    json.dump(out, open(a.dst, "w"))
    uniform_T = T0 * (v0 / a.vby) if v0 > a.vby else T0
    stretched = sum(1 for w in binding if w != "original")
    from collections import Counter
    top = Counter(w for w in binding if w != "original").most_common(3)
    print("GHOST TOPP: %s -> %s" % (os.path.basename(a.src), os.path.basename(a.dst)))
    print("  peak |vby|   %.3f -> %.3f m/s   (cap %.3f)" % (v0, v1, a.vby))
    print("  cycle_s      %.2f -> %.2f s" % (T0, T))
    print("  vs UNIFORM rescale to the same cap: %.2f s  -> saved %.2f s (%.0f%% of the penalty)"
          % (uniform_T, uniform_T - T, 100.0 * (uniform_T - T) / max(uniform_T - T0, 1e-9)))
    print("  segments stretched: %d/%d   binding: %s" % (stretched, len(binding), top or "none"))


if __name__ == "__main__":
    main()
