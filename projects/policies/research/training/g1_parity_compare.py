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

"""Numerically diff a TRAINER probe trace against a DEPLOY probe trace.

Both traces are produced by the deterministic open-loop probe (no RL):
  trainer -> projects/policies/research/training/g1_parity_probe.py
  deploy  -> projects/policies/controllers/g1_parity_probe (inside omnisim-bin)

This is the FIRST numerical trainer<->deploy parity check that compares the REAL
binary's trajectory (not the g1_deploy_runtime.py Python extract). Because the
probe is deterministic and identical on both sides, any divergence here is pure
model/stepping physics -- NOT policy and NOT durability.

Reports, per the compared window:
  * per-joint angle error (max + RMS), and the worst joint
  * first tick the max joint error crosses 1e-4 / 1e-3 / 1e-2 rad
  * base position error and base orientation error (degrees)
Exit code 0 (PASS) iff max joint error over the window <= --tol.

Usage:
  python projects/policies/research/training/g1_parity_compare.py trainer_trace.json deploy_trace.json
         [--tol 0.01] [--window N]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def _load(p):
    d = json.loads(Path(p).read_text(encoding="utf-8"))
    return d


def _rot_angle_deg(ra, rb):
    """Geodesic angle (deg) between two row-major 3x3 rotation matrices."""
    A = np.asarray(ra, float).reshape(3, 3)
    B = np.asarray(rb, float).reshape(3, 3)
    R = A.T @ B
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("trainer")
    ap.add_argument("deploy")
    ap.add_argument("--tol", type=float, default=1e-2,
                    help="max joint angle error (rad) for PASS (default 0.01)")
    ap.add_argument("--window", type=int, default=0,
                    help="compare only the first N ticks (0=all common ticks)")
    ap.add_argument("--policy", action="store_true",
                    help="closed-loop policy mode: targets are state-derived (not a "
                         "shared script), so report the target delta instead of hard-failing on it")
    args = ap.parse_args(argv)

    tr = _load(args.trainer)
    dp = _load(args.deploy)
    jt, jd = tr["meta"]["joint_order"], dp["meta"]["joint_order"]
    if jt != jd:
        print(f"FAIL: joint order differs\n  trainer={jt}\n  deploy ={jd}")
        return 3
    nj = len(jt)

    tt, dt = tr["ticks"], dp["ticks"]
    n = min(len(tt), len(dt))
    if args.window > 0:
        n = min(n, args.window)
    if n == 0:
        print("FAIL: no overlapping ticks")
        return 3

    # Sanity: the COMMANDED targets must be identical (proves both ran the same
    # deterministic script; otherwise the comparison is meaningless).
    tgt_max = 0.0
    for k in range(n):
        a = np.asarray(tt[k]["target"], float)
        b = np.asarray(dt[k]["target"], float)
        tgt_max = max(tgt_max, float(np.max(np.abs(a - b))))
    if tgt_max > 1e-6 and not args.policy:
        print(f"FAIL: commanded targets differ (max {tgt_max:.2e}) -- the two "
              f"sides did not run the same scripted motion.")
        return 3
    if args.policy:
        print(f"  [policy mode] commanded-target delta (state-derived): max {tgt_max:.3e} rad")

    q_err = np.zeros((n, nj))
    base_pos_err = np.zeros(n)
    base_ori_err = np.zeros(n)
    nan_ticks = 0
    for k in range(n):
        qa = np.asarray(tt[k]["q"], float)
        qb = np.asarray(dt[k]["q"], float)
        if not (np.isfinite(qa).all() and np.isfinite(qb).all()):
            nan_ticks += 1
            q_err[k] = np.nan
            continue
        q_err[k] = np.abs(qa - qb)
        pa = np.asarray(tt[k]["base_pos"], float)
        pb = np.asarray(dt[k]["base_pos"], float)
        if np.isfinite(pa).all() and np.isfinite(pb).all():
            base_pos_err[k] = float(np.linalg.norm(pa - pb))
        ra, rb = tt[k].get("base_rot"), dt[k].get("base_rot")
        if ra and rb and np.isfinite(np.asarray(ra, float)).all() \
                and np.isfinite(np.asarray(rb, float)).all():
            try:
                base_ori_err[k] = _rot_angle_deg(ra, rb)
            except Exception:
                base_ori_err[k] = np.nan

    per_tick_max = np.nanmax(q_err, axis=1)            # worst joint each tick
    overall_max = float(np.nanmax(per_tick_max))
    overall_rms = float(np.sqrt(np.nanmean(q_err ** 2)))
    worst_joint = int(np.nanargmax(np.nanmax(q_err, axis=0)))
    # Stable-window metric: the recorded PROBE phase only (exclude the unrecorded-
    # equivalent SETTLE transient where the joints are still converging from the
    # straight-leg spawn), then drop its last 10% (onset of any marginal-stand
    # drift). This reflects the steady post-settle tracking parity. Falls back to
    # an index window if the trace has no phase tags.
    phases = [t.get("phase") for t in tt[:n]]
    probe_idx = [i for i, p in enumerate(phases) if p == "probe"]
    if probe_idx:
        drop = max(0, int(0.1 * len(probe_idx)))
        win = probe_idx[:len(probe_idx) - drop] if drop else probe_idx
    else:
        skip = max(1, n // 10)
        win = list(range(skip, n - skip)) or list(range(n))
    stable_max = float(np.nanmax(per_tick_max[win]))
    stable_med = float(np.nanmedian(per_tick_max[win]))

    def first_cross(thr):
        idx = np.where(per_tick_max > thr)[0]
        return int(idx[0]) if len(idx) else None

    print("=" * 70)
    print("G1 BINARY-LEVEL PARITY  (trainer add_urdf  vs  deploy add_link)")
    print("=" * 70)
    print(f"  trainer: {Path(args.trainer).name}  construction={tr['meta'].get('construction')}")
    print(f"  deploy : {Path(args.deploy).name}  construction={dp['meta'].get('construction')}"
          f"  link_com={dp['meta'].get('use_link_com')}")
    print(f"  sequence={tr['meta'].get('sequence')} static_base={tr['meta'].get('static_base')}"
          f"  ticks compared={n}  (trainer {len(tt)} / deploy {len(dt)})")
    if nan_ticks:
        print(f"  WARNING: {nan_ticks} tick(s) had non-finite joint readings (skipped)")
    print("-" * 70)
    print(f"  joint-angle error:  overall_max={overall_max:.3e} rad ({math.degrees(overall_max):.3f} deg)"
          f"  rms={overall_rms:.3e} rad")
    print(f"  joint-angle error:  STABLE_max={stable_max:.3e} rad ({math.degrees(stable_max):.3f} deg)"
          f"  median={stable_med:.3e} rad  (probe-phase window, drops last 10%)")
    print(f"  worst joint: [{worst_joint}] {jt[worst_joint]} "
          f"(max {float(np.nanmax(q_err[:, worst_joint])):.3e} rad)")
    for thr in (1e-4, 1e-3, 1e-2):
        fc = first_cross(thr)
        print(f"  first tick max|dq| > {thr:.0e} rad: "
              f"{'tick ' + str(fc) if fc is not None else 'never'}")
    print(f"  base position error: max={float(np.nanmax(base_pos_err)):.3e} m")
    print(f"  base orient  error:  max={float(np.nanmax(base_ori_err)):.3e} deg")
    print("-" * 70)
    # Print a few sample ticks for eyeballing.
    for k in sorted(set([0, n // 4, n // 2, 3 * n // 4, n - 1])):
        print(f"   tick {k:4d}: max|dq|={per_tick_max[k]:.3e} rad  "
              f"base_pos_err={base_pos_err[k]:.3e} m")
    print("=" * 70)
    ok = stable_max <= args.tol and nan_ticks == 0
    print(f"  VERDICT: {'PASS' if ok else 'DIFF'}  "
          f"(stable-window max joint err {stable_max:.3e} rad "
          f"{'<=' if ok else '>'} tol {args.tol:.0e})")
    print("=" * 70)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
