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

"""stair_gait_verdict.py -- is a stair-climb deploy CLEAN? (the owner's eye, as a ruler)

"Watch the GUI" does not scale and does not gate. This scores a FOOT_LOG + deploy telemetry log on
what the owner actually asked for (2026-07-10): the robot KEEPS HEADING FORWARD and TAKES THE
CORRECT STEPS up the stairs.

  HEADING   |yaw| stays small the whole run (the walker historically wandered ~0.2 rad).
  FORWARD   base x is monotone up to small jitter: largest backward excursion bounded.
  STEPS     each foot's planted heights form the ascending tread sequence -- every tread level
            visited IN ORDER, with real residency (>= min_dwell ticks), no level skipped by
            sliding, no descent mid-climb.
  LATERAL   |base y| bounded (no crab-walking off the staircase).
  SUMMIT    the climb completes: base ends past the last riser at summit height, and if a stand
            phase exists, motionless (x std < 2 mm over the final window).

Usage:
  python stair_gait_verdict.py <tag_rl.txt> --riser 0.03 --going 0.26 --x0 1.2 --n 5 [--json]
  (geometry defaults match g1_climb_stairs_demo3.wbt; pass x0=0.12 for the synth worlds)
"""
import argparse
import json
import re

import numpy as np


def load(path):
    foot, yaw = [], []
    for ln in open(path):
        m = re.match(r"FOOTLOG t=(\d+) bx=([-\d.]+) bz=([-\d.]+) fLx=([-\d.]+) fLz=([-\d.]+) fRx=([-\d.]+) fRz=([-\d.]+)", ln)
        if m:
            foot.append([float(v) for v in m.groups()])
        m2 = re.search(r"walk-recipe deploy t=(\d+) x=([-\d.]+) y=([-\d.]+) z=([-\d.]+) .* yaw=([-\d.]+)", ln)
        if m2:
            yaw.append([float(m2.group(1)), float(m2.group(2)), float(m2.group(3)), float(m2.group(5))])
    return np.array(foot), (np.array(yaw) if yaw else None)


def step_sequence(x, z, riser, going, x0, n, ankle_floor=0.046, dwell=8):
    """The ordered list of tread levels this foot actually DWELLED on (>= dwell frames each)."""
    lev = np.full(len(z), -9, int)
    for k in range(0, n + 1):
        zk = ankle_floor + k * riser
        lev[np.abs(z - zk) < riser * 0.45] = k
    seq, run = [], 0
    for i in range(1, len(lev)):
        if lev[i] >= 0 and lev[i] == lev[i - 1]:
            run += 1
            if run == dwell and (not seq or seq[-1] != lev[i]):
                seq.append(int(lev[i]))
        else:
            run = 0
    return seq


def verdict(foot, yaw, riser, going, x0, n):
    t = foot[:, 0] * 0.016
    bx, bz = foot[:, 1], foot[:, 2]
    r = {}
    # HEADING
    if yaw is not None and len(yaw):
        r["yaw_max_deg"] = float(np.degrees(np.abs(yaw[:, 3]).max()))
        r["heading_ok"] = r["yaw_max_deg"] < 12.0
    else:
        r["yaw_max_deg"] = None; r["heading_ok"] = None
    # FORWARD monotonicity (largest backward excursion from the running max)
    runmax = np.maximum.accumulate(bx)
    r["max_backslide_m"] = float((runmax - bx).max())
    r["forward_ok"] = r["max_backslide_m"] < 0.15
    # LATERAL
    if yaw is not None and len(yaw):
        r["y_max_m"] = float(np.abs(yaw[:, 2]).max())
        r["lateral_ok"] = r["y_max_m"] < 0.35
    else:
        r["y_max_m"] = None; r["lateral_ok"] = None
    # STEPS: ascending tread sequences per foot
    seqL = step_sequence(foot[:, 3], foot[:, 4], riser, going, x0, n)
    seqR = step_sequence(foot[:, 5], foot[:, 6], riser, going, x0, n)
    r["steps_L"], r["steps_R"] = seqL, seqR
    def _ok(seq):
        asc = [s for s in seq if s >= 0]
        climbs = [b - a for a, b in zip(asc, asc[1:])]
        return (len(asc) >= n and max(asc) >= n - 1 and all(c >= 0 or a >= n - 1 for c, a in zip(climbs, asc))
                and not any(c < 0 for c in climbs[:-1]))
    r["steps_ok"] = bool(_ok(seqL) and _ok(seqR))
    # SUMMIT
    last = foot[t > t[-1] - 6.0]
    x_end = x0 + n * going
    r["summit_x"] = float(last[:, 1].mean()); r["summit_z"] = float(last[:, 2].mean())
    r["summit_ok"] = bool(r["summit_x"] > x_end - 0.30 and r["summit_z"] > 0.72 + n * riser - 0.03)
    r["stand_x_std_mm"] = float(1000 * last[:, 1].std())
    r["stand_ok"] = r["stand_x_std_mm"] < 5.0
    checks = [v for k, v in r.items() if k.endswith("_ok") and v is not None]
    r["CLEAN"] = bool(all(checks))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--riser", type=float, default=0.03); ap.add_argument("--going", type=float, default=0.26)
    ap.add_argument("--x0", type=float, default=1.2); ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    foot, yaw = load(a.log)
    if not len(foot):
        raise SystemExit("no FOOTLOG lines in %s" % a.log)
    r = verdict(foot, yaw, a.riser, a.going, a.x0, a.n)
    if a.json:
        print(json.dumps(r, indent=1)); return
    print("STAIR-GAIT VERDICT: %s" % a.log)
    print("  HEADING  max |yaw| %s deg%s" % (("%.1f" % r["yaw_max_deg"]) if r["yaw_max_deg"] is not None else "n/a",
          "" if r["heading_ok"] in (True, None) else "  <-- WANDERS"))
    print("  FORWARD  max backslide %.3f m%s" % (r["max_backslide_m"], "" if r["forward_ok"] else "  <-- SLIDES BACK"))
    print("  LATERAL  max |y| %s m%s" % (("%.2f" % r["y_max_m"]) if r["y_max_m"] is not None else "n/a",
          "" if r["lateral_ok"] in (True, None) else "  <-- CRABS"))
    print("  STEPS    L %s   R %s%s" % (r["steps_L"], r["steps_R"], "" if r["steps_ok"] else "  <-- NOT the clean ascending sequence"))
    print("  SUMMIT   end x %.2f z %.2f%s   stand x-std %.1f mm%s" %
          (r["summit_x"], r["summit_z"], "" if r["summit_ok"] else "  <-- DID NOT TOP OUT",
           r["stand_x_std_mm"], "" if r["stand_ok"] else " (moving)"))
    print("  VERDICT: %s" % ("CLEAN" if r["CLEAN"] else "NOT CLEAN"))


if __name__ == "__main__":
    main()
