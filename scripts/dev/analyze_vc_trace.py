#!/usr/bin/env python
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

"""Verify the velocity-conditioned STOP-IN-THE-MIDDLE milestone from a deploy
trace (G1_TRACE_REF csv: time,bx,phase,...). Reports the forward-velocity
profile and checks the robot (a) walked, (b) actually STOPPED during the
commanded stand window, (c) RESUMED walking after.

Usage: analyze_vc_trace.py <trace.csv> [stand_at_s] [stand_for_s]
"""
import sys
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "_scratch/g1_walk_vc_trace.csv"
stand_at = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
stand_for = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

t, bx = [], []
with open(path) as f:
    for line in f:
        p = line.split(",")
        if len(p) < 2:
            continue
        try:
            t.append(float(p[0])); bx.append(float(p[1]))
        except ValueError:
            continue
t = np.array(t); bx = np.array(bx)
if len(t) < 5:
    print("trace too short / empty"); sys.exit(1)

# smoothed forward velocity (central diff over ~0.25 s)
def vel_at(t0, t1):
    m = (t >= t0) & (t < t1)
    if m.sum() < 2:
        return float("nan")
    seg_t, seg_x = t[m], bx[m]
    return float((seg_x[-1] - seg_x[0]) / max(seg_t[-1] - seg_t[0], 1e-6))

t_end = t[-1]
pre_stop = stand_at if stand_at > 0 else t_end
v_walk1 = vel_at(max(0.0, pre_stop - 4.0), pre_stop - 0.5)       # walking before stop
v_stand = vel_at(stand_at + 1.5, stand_at + stand_for - 0.5)     # during stand (settled)
v_walk2 = vel_at(stand_at + stand_for + 1.5, t_end - 0.5)        # after resume

print(f"trace: {path}  ({len(t)} rows, {t_end:.1f}s, x: {bx[0]:+.2f} -> {bx[-1]:+.2f} m)")
print(f"  walk BEFORE stop  ({pre_stop-4:.1f}-{pre_stop-0.5:.1f}s): vx = {v_walk1:+.3f} m/s")
if stand_at > 0:
    print(f"  STAND window      ({stand_at+1.5:.1f}-{stand_at+stand_for-0.5:.1f}s): vx = {v_stand:+.3f} m/s   (want ~0)")
    print(f"  walk AFTER resume ({stand_at+stand_for+1.5:.1f}-{t_end-0.5:.1f}s): vx = {v_walk2:+.3f} m/s")
    stopped = abs(v_stand) < 0.07
    resumed = v_walk2 > 0.12
    walked1 = v_walk1 > 0.12
    print()
    print(f"  walked first?  {'YES' if walked1 else 'no '}  ({v_walk1:+.3f})")
    print(f"  STOPPED?       {'YES' if stopped else 'no '}  (|{v_stand:+.3f}| < 0.07)")
    print(f"  resumed?       {'YES' if resumed else 'no '}  ({v_walk2:+.3f} > 0.12)")
    print()
    print("  MILESTONE:", "PASS" if (walked1 and stopped and resumed) else "FAIL")
