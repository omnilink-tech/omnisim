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

"""Plot the velocity-conditioned STOP-IN-THE-MIDDLE milestone from a deploy
trace (G1_TRACE_REF csv: time,bx,phase,...): forward speed + distance vs time,
with the commanded stand window shaded. This is the graph that shows the
robot walking, stopping on command, and resuming.

Usage: plot_vc_milestone.py <trace.csv> [stand_at] [stand_for] [out.png]
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "_scratch/g1_walk_vc_trace.csv"
stand_at = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
stand_for = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
out = sys.argv[4] if len(sys.argv) > 4 else "_scratch/vc_milestone_profile.png"

t, bx = [], []
for line in open(path):
    p = line.split(",")
    if len(p) < 2:
        continue
    try:
        t.append(float(p[0])); bx.append(float(p[1]))
    except ValueError:
        continue
t = np.array(t); bx = np.array(bx)

# smoothed forward velocity (central diff over ~0.3 s window)
def smooth_vel(t, x, win=0.3):
    v = np.zeros_like(x)
    for i in range(len(t)):
        lo = np.searchsorted(t, t[i] - win / 2)
        hi = min(len(t) - 1, np.searchsorted(t, t[i] + win / 2))
        if hi > lo:
            v[i] = (x[hi] - x[lo]) / max(t[hi] - t[lo], 1e-6)
    return v

vx = smooth_vel(t, bx)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
fig.suptitle("G1 velocity-conditioned WALK → STAND → WALK (one policy)",
             fontsize=13)

for ax in (ax1, ax2):
    ax.axvspan(stand_at, stand_at + stand_for, color="orange", alpha=0.15,
               label="stand commanded (vx_cmd=0)")

ax1.plot(t, vx, lw=1.6, color="C0")
ax1.axhline(0.40, ls="--", lw=0.8, color="green", alpha=0.7, label="walk target 0.40")
ax1.axhline(0.0, ls="-", lw=0.6, color="k", alpha=0.3)
ax1.set_ylabel("forward speed (m/s)")
ax1.legend(loc="upper right", fontsize=9)
ax1.grid(alpha=0.3)

ax2.plot(t, bx, lw=1.6, color="C3")
ax2.set_ylabel("forward distance (m)")
ax2.set_xlabel("time (s)")
ax2.grid(alpha=0.3)
ax2.legend(loc="upper left", fontsize=9)

fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(out, dpi=110)
print(f"saved {out}")
