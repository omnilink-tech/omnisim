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

"""Analyze per-tick joint state CSV from spot_residual_deploy for unrealistic
motion patterns. Pair with SPOT_DEBUG_JOINTS_CSV=<path> during a deploy run.

Six categories flagged:
  1. Out-of-URDF-range joint positions (the post-step clamp in
     WbNewtonBackend should make this 0).
  2. Joint velocity > URDF velocity_limit (same — clamp guarantee).
  3. Per-tick position jumps > 0.15 rad in 16 ms (= >9.4 rad/s,
     within URDF velocity limit but visually fast for a leg).
  4. Sustained at-URDF-limit dwells (joint pinned at mechanical
     stop for 10+ consecutive ticks).
  5. Acceleration spikes > 300 rad/s² between consecutive ticks.
  6. Rapid direction reversals (qd sign flip with |qd|>5 rad/s
     on both sides — high jerk).

A run with 0 in all categories means the simulator is producing
physically valid joint trajectories.
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path

DEFAULT_CSV = Path.home() / "AppData/Local/Temp/spot_joints.csv"

# URDF limits — read from the WbNewtonBackend startup log lines like
# `hinge joint X ... lim=[lo, hi]`. Defaults below match the widened
# spot.urdf (post-2026-05-24). For the narrow spot.classic.urdf, pass
# --urdf classic.
URDF_RANGE_WIDENED = {}
URDF_RANGE_CLASSIC = {}
for leg in ("front_left", "front_right", "rear_left", "rear_right"):
    URDF_RANGE_WIDENED[f"q_{leg}_hip_x"] = (-1.5, +1.5)
    URDF_RANGE_WIDENED[f"q_{leg}_hip_y"] = (-0.5, +3.13159)
    URDF_RANGE_WIDENED[f"q_{leg}_knee"] = (-1.20, -0.01)
    side_sign = +1 if "left" in leg else -1
    URDF_RANGE_CLASSIC[f"q_{leg}_hip_x"] = (
        (0.001, 0.785) if side_sign > 0 else (-0.785, -0.001))
    URDF_RANGE_CLASSIC[f"q_{leg}_hip_y"] = (0.001, 0.60)
    URDF_RANGE_CLASSIC[f"q_{leg}_knee"] = (-1.20, -0.01)

VEL_LIMIT = 20.0       # rad/s URDF velocity_limit
JUMP_THRESHOLD = 0.15  # rad per 16 ms tick
ACCEL_THRESHOLD = 300.0  # rad/s² between consecutive ticks
DWELL_THRESHOLD = 10   # consecutive ticks at URDF limit


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", type=Path, default=DEFAULT_CSV,
                   help="Path to SPOT_DEBUG_JOINTS_CSV output.")
    p.add_argument("--urdf", choices=("widened", "classic"), default="widened",
                   help="URDF variant used by the deploy world.")
    args = p.parse_args()

    URDF_RANGE = (URDF_RANGE_WIDENED if args.urdf == "widened"
                  else URDF_RANGE_CLASSIC)

    with open(args.csv) as f:
        rdr = csv.DictReader(f)
        rows = list(rdr)
    if not rows:
        print("empty CSV")
        return

    q_cols = [c for c in rows[0].keys() if c.startswith("q_")]
    qd_cols = [c for c in rows[0].keys() if c.startswith("qd_")]
    print(f"loaded {len(rows)} ticks, {len(q_cols)} joints "
          f"(URDF={args.urdf})")
    print()

    # 1. Out-of-range positions
    out_of_range = defaultdict(int)
    for r in rows:
        for c in q_cols:
            lo, hi = URDF_RANGE.get(c, (-1e9, 1e9))
            v = float(r[c])
            if v < lo - 1e-4 or v > hi + 1e-4:
                out_of_range[c] += 1
    print("=== out-of-URDF-range positions (clamp should = 0) ===")
    if not out_of_range:
        print("  none OK")
    else:
        for c, n in sorted(out_of_range.items(), key=lambda x: -x[1]):
            print(f"  {c}: {n} ticks")
    print()

    # 2. Over-velocity-limit
    over_vel = defaultdict(lambda: {"count": 0, "peak": 0.0})
    for r in rows:
        for c in qd_cols:
            v = abs(float(r[c]))
            if v > VEL_LIMIT:
                over_vel[c]["count"] += 1
                over_vel[c]["peak"] = max(over_vel[c]["peak"], v)
    print("=== over-velocity-limit (>20 rad/s; clamp should = 0) ===")
    if not over_vel:
        print("  none OK")
    else:
        for c, d in sorted(over_vel.items(), key=lambda x: -x[1]["count"]):
            print(f"  {c}: {d['count']} ticks, peak {d['peak']:.1f} rad/s")
    print()

    # 3. Big position jumps
    big_jumps = defaultdict(list)
    prev_q = {c: None for c in q_cols}
    for r in rows:
        for c in q_cols:
            v = float(r[c])
            pv = prev_q[c]
            if pv is not None:
                dq = abs(v - pv)
                if dq > JUMP_THRESHOLD:
                    big_jumps[c].append((int(r["t_ms"]), dq))
            prev_q[c] = v
    print(f"=== big position jumps (>{JUMP_THRESHOLD} rad in 16 ms "
          f"= >{JUMP_THRESHOLD*1000/16:.1f} rad/s) ===")
    if not big_jumps:
        print("  none OK")
    else:
        for c, jumps in sorted(big_jumps.items(), key=lambda x: -len(x[1])):
            biggest = max(jumps, key=lambda j: j[1])
            print(f"  {c}: {len(jumps)} events, biggest {biggest[1]:.3f} rad "
                  f"@ t={biggest[0]/1000:.2f}s")
    print()

    # 4. Sustained at-limit dwells
    dwells = defaultdict(int)
    streaks = {c: 0 for c in q_cols}
    for r in rows:
        for c in q_cols:
            lo, hi = URDF_RANGE.get(c, (-1e9, 1e9))
            v = float(r[c])
            tol = 0.005
            if v < lo + tol or v > hi - tol:
                streaks[c] += 1
            else:
                if streaks[c] >= DWELL_THRESHOLD:
                    dwells[c] = max(dwells[c], streaks[c])
                streaks[c] = 0
    for c in q_cols:
        if streaks[c] >= DWELL_THRESHOLD:
            dwells[c] = max(dwells[c], streaks[c])
    print(f"=== sustained at-limit dwells (>={DWELL_THRESHOLD} consecutive ticks) ===")
    if not dwells:
        print("  none OK")
    else:
        for c, n in sorted(dwells.items(), key=lambda x: -x[1]):
            print(f"  {c}: longest streak {n} ticks ({n*16/1000:.2f}s)")
    print()

    # 5. Acceleration spikes
    accel_spikes = defaultdict(list)
    prev_qd = {c: None for c in qd_cols}
    for r in rows:
        for c in qd_cols:
            v = float(r[c])
            pv = prev_qd[c]
            if pv is not None:
                accel = abs(v - pv) / 0.016
                if accel > ACCEL_THRESHOLD:
                    accel_spikes[c].append((int(r["t_ms"]), accel))
            prev_qd[c] = v
    print(f"=== acceleration spikes (>{ACCEL_THRESHOLD} rad/s^2 between consecutive ticks) ===")
    if not accel_spikes:
        print("  none OK")
    else:
        for c, spikes in sorted(accel_spikes.items(),
                                key=lambda x: -len(x[1]))[:5]:
            biggest = max(spikes, key=lambda s: s[1])
            print(f"  {c}: {len(spikes)} events, peak {biggest[1]:.0f} rad/s^2 "
                  f"@ t={biggest[0]/1000:.2f}s")
    print()

    # 6. Direction reversals
    flips = defaultdict(int)
    prev_qd = {c: None for c in qd_cols}
    for r in rows:
        for c in qd_cols:
            v = float(r[c])
            pv = prev_qd[c]
            if pv is not None and abs(v) > 5.0 and abs(pv) > 5.0 \
                    and (v > 0) != (pv > 0):
                flips[c] += 1
            prev_qd[c] = v
    print("=== rapid direction reversals (sign flip with |qd|>5 rad/s on both sides) ===")
    if not flips:
        print("  none OK")
    else:
        for c, n in sorted(flips.items(), key=lambda x: -x[1]):
            print(f"  {c}: {n} reversals")


if __name__ == "__main__":
    main()
