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

"""ghost_polish.py -- turn a FOLD-recorded achieved gait into a clean, symmetric reference.

Owner direction 2026-07-05: the ghost must show ONE smooth consistent motion (real-G1 look:
stable legs + clean pendulum sway). Pipeline: phase-folded recording (tremor already averaged
out ~1/sqrt(N) by the recorder) -> BILATERAL SYMMETRIZATION -> light harmonic smoothing ->
URDF limit clip -> smoothness report (bin-to-bin jerk proxy, before vs after).

Symmetrization: a symmetric gait satisfies L_c(phi) = s_c * R_c(phi+pi) with s_c = +1 for
pitch/knee channels and -1 for roll/yaw channels (mirror flips the sign of out-of-sagittal
motion). We average each side with the mirrored other side; waist yaw is self-mirrored
antisymmetric; attitude roll becomes a pure pendulum (antisymmetric), pitch symmetric.

Usage: python ghost_polish.py <lut.json> [--harmonics 6]
"""
import json
import re
import sys

import numpy as np

# canonical leg order (cols 0-5 left, 6-11 right); +1 pitch-like, -1 roll/yaw-like
LEG_SIGN = np.array([1.0, -1.0, -1.0, 1.0, 1.0, -1.0])
WB_SIGN = {  # per-joint mirror sign, canonical 23-joint wb order (by name lookup)
    "hip_pitch": 1, "hip_roll": -1, "hip_yaw": -1, "knee": 1, "ankle_pitch": 1, "ankle_roll": -1,
    "waist_yaw": -1,
    "shoulder_pitch": 1, "shoulder_roll": -1, "shoulder_yaw": -1, "elbow": 1, "wrist_roll": -1,
}


def _smooth(tab, H):
    F = np.fft.rfft(np.asarray(tab, np.float32), axis=0)
    F[H + 1:] = 0.0
    return np.fft.irfft(F, n=len(tab), axis=0).astype(np.float32)


def _jerk(tab):
    t = np.asarray(tab, np.float32)
    return float(np.sqrt(np.mean(np.diff(t, 2, axis=0) ** 2)))


def main(path, H=6):
    d = json.load(open(path))
    nb = d["nb"]; half = nb // 2
    legs = np.asarray(d["leg_lut"], np.float32)
    j0 = _jerk(legs)

    # 1. bilateral leg symmetrization
    L, R = legs[:, :6], legs[:, 6:12]
    Lr = np.roll(R, half, axis=0) * LEG_SIGN
    Rr = np.roll(L, half, axis=0) * LEG_SIGN
    legs = np.concatenate([0.5 * (L + Lr), 0.5 * (R + Rr)], axis=1)

    # 2. whole-body symmetrization by name
    if "wb_lut" in d and "wb_joints" in d:
        wb = np.asarray(d["wb_lut"], np.float32)
        names = d["wb_joints"]
        for i, n in enumerate(names):
            key = next((k for k in WB_SIGN if k in n), None)
            if key is None:
                continue
            if n.startswith("left_"):
                jr = names.index(n.replace("left_", "right_"))
                a, b = wb[:, i].copy(), wb[:, jr].copy()
                wb[:, i] = 0.5 * (a + WB_SIGN[key] * np.roll(b, half))
                wb[:, jr] = 0.5 * (b + WB_SIGN[key] * np.roll(a, half))
            elif not n.startswith("right_"):          # waist: self-mirror antisymmetric
                a = wb[:, i].copy()
                wb[:, i] = 0.5 * (a + WB_SIGN[key] * np.roll(a, half))
        # keep wb leg columns identical to the symmetrized leg table
        wb[:, :12] = legs
        d["wb_lut"] = wb.tolist()
        arm = np.stack([wb[:, names.index("left_shoulder_pitch_joint")],
                        wb[:, names.index("right_shoulder_pitch_joint")]], 1)
        d["arm_lut"] = arm.tolist()

    # 3. attitude: roll = clean pendulum (antisymmetric), pitch symmetric
    if "att_lut" in d:
        att = np.asarray(d["att_lut"], np.float32)
        att[:, 0] = 0.5 * (att[:, 0] - np.roll(att[:, 0], half))
        att[:, 1] = 0.5 * (att[:, 1] + np.roll(att[:, 1], half))
        d["att_lut"] = _smooth(att, max(3, H // 2)).tolist()

    # 4. light harmonic smoothing + URDF ankle-roll clip
    legs = _smooth(legs, H)
    u = open("projects/robots/unitree/g1/urdf/g1_23dof.urdf").read()
    for m in re.finditer(r'<joint name="((?:left|right)_ankle_roll)_joint".*?'
                         r'<limit[^>]*lower="([-\d.eE]+)"[^>]*upper="([-\d.eE]+)"', u, re.S):
        col = 5 if m.group(1).startswith("left") else 11
        legs[:, col] = np.clip(legs[:, col], float(m.group(2)) + 0.002, float(m.group(3)) - 0.002)
    d["leg_lut"] = legs.tolist()
    if "wb_lut" in d:
        wb = np.asarray(d["wb_lut"], np.float32)
        wb[:, :12] = legs
        d["wb_lut"] = _smooth(wb, H).tolist()
    if "arm_lut" in d:
        d["arm_lut"] = _smooth(np.asarray(d["arm_lut"], np.float32), H).tolist()

    j1 = _jerk(d["leg_lut"])
    d["source"] = (d.get("source", "") + f" | POLISHED: bilateral-symmetrized + H={H}")[:400]
    json.dump(d, open(path, "w"))
    print(f"[polish] {path}")
    print(f"  leg jerk (rad, bin-to-bin 2nd diff RMS): {j0:.4f} -> {j1:.4f}  ({100*(1-j1/max(j0,1e-9)):.0f}% smoother)")
    legs = np.asarray(d["leg_lut"])
    print(f"  hip swing L/R: {np.degrees(legs[:,0].max()-legs[:,0].min()):.1f} / "
          f"{np.degrees(legs[:,6].max()-legs[:,6].min()):.1f} deg (symmetric by construction)")


if __name__ == "__main__":
    H = 6
    if "--harmonics" in sys.argv:
        H = int(sys.argv[sys.argv.index("--harmonics") + 1])
    main(sys.argv[1], H)
