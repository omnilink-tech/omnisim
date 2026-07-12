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

"""baton_metrics.py -- switch-quality metrics for a BATON run (docs/developer/policy-switching.md).

Parses a deploy run's rl+mpc logs and reports, per switch and worst-case:
  - switches fired / scheduled, falls (pelvis z < 0.45 anywhere),
  - transient excursion: peak |roll|, |pitch|, min pelvis z within WINDOW_S after each switch,
  - stand stillness: mean |vx| and mean |roll| over the tail of each stand segment,
  - walk progress: x advanced per walk segment.

Usage: python projects/policies/training/baton_metrics.py <tag>          # reads _scratch/foot_redesign/<tag>_{rl,mpc}.txt
       python projects/policies/training/baton_metrics.py <rl.txt> <mpc.txt>

FIELD POSITIONING (2026-07-08, see policy-switching.md "Where BATON stands vs the field"):
BATON's edge over one-policy methods (universal trackers / foundation models) is a WELL-POSED
OPEN GAP, not a proven win -- does an ENGINEERED handover (morph + phase-gate + recurrent
hidden-state mgmt) degrade more gracefully over long horizons than a single distilled policy?
The proof is the HORIZON-SCALING experiment: run BATON vs a single distilled policy vs a
naive-FSM hierarchy on a many-cycle task and plot success-rate vs # cycles (cf. LHM-Humanoid
Table 5, where a single policy fell 90->18% over 5 cycles). TODO here: add per-cycle success
vs cycle-index logging so this tool produces that headline plot.
"""
import re
import sys

WINDOW_S = 2.0
TICK_S = 0.016


def load(rl_path, mpc_path):
    sw = []
    for ln in open(rl_path, errors="replace"):
        m = re.search(r"BATON switch #(\d+) \((?:-> )?(\S+?)\) at t=(\d+) phase=([\d.]+)", ln)
        if m:
            sw.append((int(m.group(1)), m.group(2), int(m.group(3)), float(m.group(4))))
    tel = {}
    for ln in open(mpc_path, errors="replace"):
        m = re.search(r"deploy t=(\d+) x=([-\d.]+) y=([-\d.]+) z=([\d.]+) roll=([-\d.]+) pit=([-\d.]+) cmd=([-\d.]+) vx=([-\d.]+)", ln)
        if m:
            tel[int(m.group(1))] = tuple(float(m.group(i)) for i in range(2, 9))
    return sw, tel


def report(sw, tel):
    ts = sorted(tel)
    if not ts:
        print("no telemetry"); return
    falls = sum(1 for t in ts if tel[t][2] < 0.45)
    win = int(WINDOW_S / TICK_S)
    print(f"switches fired: {len(sw)}   telemetry ticks: {ts[0]}..{ts[-1]}   fall-ticks(z<0.45): {falls}")
    print("switch | dir         | phase | peak|roll| peak|pit|  min z")
    wr = wp = 0.0; mz_all = 9.0
    for n, d, t0, ph in sw:
        w = [tel[t] for t in ts if t0 <= t <= t0 + win]
        if not w:
            continue
        pr = max(abs(x[3]) for x in w); pp = max(abs(x[4]) for x in w); mz = min(x[2] for x in w)
        wr, wp, mz_all = max(wr, pr), max(wp, pp), min(mz_all, mz)
        print(" #%-2d   | %-11s | %.2f  |  %.3f    %.3f    %.3f" % (n, d, ph, pr, pp, mz))
    import math
    print("WORST: |roll| %.3f rad (%.1f deg)  |pit| %.3f  min z %.3f  falls %d"
          % (wr, math.degrees(wr), wp, mz_all, falls))
    # stand stillness: ticks where cmd==0 (fully in stand)
    still = [tel[t] for t in ts if abs(tel[t][5]) < 1e-6]
    if still:
        mvx = sum(abs(x[6]) for x in still) / len(still)
        mrl = sum(abs(x[3]) for x in still) / len(still)
        print("stand stillness (cmd==0 ticks, n=%d): mean|vx| %.3f m/s  mean|roll| %.4f rad" % (len(still), mvx, mrl))


if __name__ == "__main__":
    if len(sys.argv) == 2:
        tag = sys.argv[1]
        rl, mpc = f"_scratch/foot_redesign/{tag}_rl.txt", f"_scratch/foot_redesign/{tag}_mpc.txt"
    else:
        rl, mpc = sys.argv[1], sys.argv[2]
    report(*load(rl, mpc))
