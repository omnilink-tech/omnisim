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

"""Plot the long-run training graph: (top) reward/step vs global steps from
every chunk's train log, live; (bottom) the per-chunk eval improvement curve
(walk distance + survival) from long_run_curve.csv. Re-runnable any time --
the picture fills in as chunks complete.

Usage: python plot_long_run.py [out.png]
"""
import glob
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

SCRATCH = str(_REPO / "_scratch")
OUT = sys.argv[1] if len(sys.argv) > 1 else f"{SCRATCH}/long_run_graph.png"
STEPS_PER_ITER = 4096 * 12
ITERS_PER_CHUNK = 1000
CHUNK_STEPS = STEPS_PER_ITER * ITERS_PER_CHUNK

_rx = re.compile(r"it\s+(\d+).*?ep_rew/step~([+-][0-9.]+).*?steps\s+([\d,]+)")


def read_text(path):
    for enc in ("utf-16", "utf-16-le", "utf-8"):
        try:
            with open(path, encoding=enc) as f:
                t = f.read()
            if "ep_rew" in t:
                return t
        except (UnicodeError, UnicodeDecodeError):
            continue
    return ""


# ── reward learning curve across all chunks ──
rew_x, rew_y = [], []
logs = sorted(glob.glob(f"{SCRATCH}/long_c*.train.log"),
              key=lambda p: int(re.search(r"long_c(\d+)", p).group(1)))
for path in logs:
    ci = int(re.search(r"long_c(\d+)", path).group(1))
    base = (ci - 1) * CHUNK_STEPS
    for m in _rx.finditer(read_text(path)):
        within = int(m.group(3).replace(",", ""))
        rew_x.append((base + within) / 1e6)        # million steps
        rew_y.append(float(m.group(2)))

# ── per-chunk curve (agent-run; cols: chunk, steps_m, fwd_dist_m,
#    firstfall_s, hip, knee, ankle amplitude-vs-ghost) ──
ev_steps, ev_dist, ev_ff = [], [], []
ev_hip, ev_knee, ev_ank = [], [], []
try:
    with open(f"{SCRATCH}/long_curve_mine.csv") as f:
        for line in f.readlines()[1:]:
            p = line.strip().split(",")
            if len(p) >= 4 and p[2] not in ("ERR", "NA", ""):
                ev_steps.append(float(p[1]))
                ev_dist.append(float(p[2]))
                ev_ff.append(float(p[3]) if p[3] not in ("ERR", "NA", "") else np.nan)
                ev_hip.append(float(p[4]) if len(p) > 4 and p[4] else np.nan)
                ev_knee.append(float(p[5]) if len(p) > 5 and p[5] else np.nan)
                ev_ank.append(float(p[6]) if len(p) > 6 and p[6] else np.nan)
except FileNotFoundError:
    pass

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 11), sharex=True)

# 1) reward learning curve (live)
ax1.plot(rew_x, rew_y, lw=1.1, color="#1f77b4")
ax1.set_ylabel("reward / step")
ax1.set_title(f"G1 long run — reward, GAIT STYLE (the goal), robustness  "
              f"[{rew_x[-1]:.0f}M steps]" if rew_x else "G1 long run")
ax1.grid(alpha=0.3)
for c in range(1, len(logs) + 1):
    ax1.axvline(c * CHUNK_STEPS / 1e6, color="gray", lw=0.5, ls=":", alpha=0.4)

# 2) STYLE — amplitude vs the ghost (target = 1.0); this is the objective
if ev_dist:
    ax2.axhline(1.0, color="k", lw=1, ls="--", alpha=0.6, label="ghost (target)")
    ax2.plot(ev_steps, ev_hip, "-o", color="#1f77b4", lw=2, label="hip")
    ax2.plot(ev_steps, ev_knee, "-o", color="#d62728", lw=2, label="knee")
    ax2.plot(ev_steps, ev_ank, "-o", color="#2ca02c", lw=2, label="ankle")
    ax2.axhline(0.49, color="#d62728", lw=0.8, ls=":", alpha=0.5)
    ax2.text(ev_steps[0], 0.50, "v3 knee (shipped)", fontsize=8, color="#d62728")
    ax2.legend(loc="upper left", ncol=4, fontsize=9)
ax2.set_ylabel("gait amplitude / ghost")
ax2.grid(alpha=0.3)

# 3) robustness — distance + survival
if ev_dist:
    ax3.plot(ev_steps, ev_dist, "-o", color="#2ca02c", lw=2, label="walk dist (m)")
    ax3b = ax3.twinx()
    ax3b.plot(ev_steps, ev_ff, "-s", color="#d62728", lw=1.3, ms=4, label="survival (s)")
    ax3b.set_ylabel("survival (s)", color="#d62728")
    ax3.set_ylabel("walk dist (m)", color="#2ca02c")
    ax3.legend(loc="upper left", fontsize=9)
ax3.set_xlabel("global training steps (millions)")
ax3.grid(alpha=0.3)

fig.tight_layout()
fig.savefig(OUT, dpi=110)
print(f"saved {OUT}  (reward pts={len(rew_y)}, chunks={len(ev_dist)})")
