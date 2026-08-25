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

"""Gait-style metric measured IN DEPLOY, from a G1_TRACE_REF csv
(t, bx, phase, joint_q[13], baseline[13], targets[13]).

Phase-bins the ACTUAL joint angles into a stride waveform and compares it to
the MODEL (baseline; for hip/knee the baseline IS the pure model -- the
ankle PD only touches ankles). Reports amplitude ratio, shape correlation,
phase lag, swing-knee peak. This is the style score of what the user SEES.

Usage: python analyze_deploy_style.py trace.csv [label]
"""
import sys

import numpy as np

NBINS = 36
rows = np.loadtxt(sys.argv[1], delimiter=",")
label = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1]
t_max = float(sys.argv[3]) if len(sys.argv) > 3 else 1e18   # cut at first fall
rows = rows[rows[:, 0] <= t_max]
t = rows[:, 0]
ph = rows[:, 2] % (2 * np.pi)
qact = rows[:, 3:16]
qmod = rows[:, 16:29]

# steady gait only: skip the first 6 s (launch ramp + settle)
m = t > t[0] + 6.0
ph, qact, qmod = ph[m], qact[m], qmod[m]
b = np.clip((ph / (2 * np.pi) * NBINS).astype(int), 0, NBINS - 1)

names = {0: "hip  ", 3: "knee ", 4: "ankle"}
print(f"deploy style: {label}  ({m.sum()} ticks, {t[-1]-t[0]-6:.0f}s steady)")
for j, nm in names.items():
    wa = np.array([qact[b == i, j].mean() if (b == i).any() else np.nan
                   for i in range(NBINS)])
    wm = np.array([qmod[b == i, j].mean() if (b == i).any() else np.nan
                   for i in range(NBINS)])
    ok = ~(np.isnan(wa) | np.isnan(wm))
    wa, wm = wa[ok], wm[ok]
    am, mm = wa - wa.mean(), wm - wm.mean()
    n = len(am)
    xc = [np.dot(np.roll(am, s), mm) for s in range(n)]
    s = int(np.argmax(xc))
    lag = (s if s <= n // 2 else s - n) / n * 100
    amp = wa.std() / max(wm.std(), 1e-9)
    a_al = np.roll(am, s)
    corr = float(np.dot(a_al, mm) / max(np.linalg.norm(a_al) * np.linalg.norm(mm), 1e-9))
    extra = ""
    if j == 3:
        extra = (f" | swing peak {np.degrees(wa.max()-wa.min()):4.0f} deg range "
                 f"(model {np.degrees(wm.max()-wm.min()):4.0f})")
    print(f"  L_{nm}: amplitude {amp:4.2f}x | corr {corr:5.2f} | "
          f"lag {lag:+5.1f}%{extra}")

# FRONTAL plane: hip-roll = stance width / leg spread (the front-view shape).
# cols: q[1]=L_hip_roll q[7]=R_hip_roll (in the 13-vec at offset 3).
lhr_a, rhr_a = qact[:, 1], qact[:, 7]
lhr_m, rhr_m = qmod[:, 1], qmod[:, 7]
L = 0.64
spread_a = L * (np.abs(np.sin(lhr_a)).mean() + np.abs(np.sin(rhr_a)).mean()) * 100
spread_m = L * (np.abs(np.sin(lhr_m)).mean() + np.abs(np.sin(rhr_m)).mean()) * 100
print(f"  STANCE WIDTH (leg spread): actual ~{spread_a:4.0f} cm  "
      f"model ~{spread_m:4.0f} cm   "
      f"[hip-roll actual L{np.degrees(lhr_a.mean()):+.0f} R{np.degrees(rhr_a.mean()):+.0f} deg, "
      f"model L{np.degrees(lhr_m.mean()):+.0f} R{np.degrees(rhr_m.mean()):+.0f}]")
