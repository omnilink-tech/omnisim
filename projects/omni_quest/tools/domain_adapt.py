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

"""Sim-to-real domain adaptation + randomisation for the camera stream.

Closes the measured sim-to-real image gap (domain_gap.py) by transforming sim
frames toward the real (KITTI) appearance distribution. The core is per-channel
HISTOGRAM MATCHING to the real distribution -- which aligns brightness, contrast,
dynamic range and colour together (a multiplicative exposure tweak compresses
contrast; histogram matching doesn't) -- then real-camera SENSOR NOISE and a light
unsharp for the high-frequency detail the low-res render lacks. With randomise=True
it jitters exposure/noise/sharpen per frame (domain RANDOMISATION) so a perception
model sees a band that COVERS real rather than one point.

    python tools/domain_adapt.py          # adapt the sim frames + re-measure the gap
"""

from __future__ import annotations

import glob
import os
import sys

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None

from domain_gap import REAL_DIR, SIM_DIR, SIZE, report

OUT_DIR = os.path.join(os.path.dirname(SIM_DIR), "sim_adapted")
RAND_DIR = os.path.join(os.path.dirname(SIM_DIR), "sim_random")
NOISE_SD = 6.0
SHARPEN = 0.7
SAT_PULL = 0.60    # per-channel match over-saturates; pull S back toward real's mean


def real_reference(real_dir=REAL_DIR, n=40):
    """Sorted per-channel pixel values from real frames, the histogram-match target."""
    paths = sorted(glob.glob(os.path.join(real_dir, "*.png")))[:n]
    chans = [[], [], []]
    for p in paths:
        im = cv2.imread(p, cv2.IMREAD_COLOR)
        if im is not None:
            im = cv2.resize(im, SIZE)
            for c in range(3):
                chans[c].append(im[:, :, c].ravel())
    return [np.sort(np.concatenate(chans[c])).astype(np.float32) for c in range(3)]


def _match(channel, ref_sorted):
    """Map a channel's pixels to the reference distribution by rank/quantile."""
    flat = channel.ravel()
    ranks = np.argsort(np.argsort(flat))
    q = ranks / max(flat.size - 1, 1)
    return ref_sorted[(q * (len(ref_sorted) - 1)).astype(np.int64)].reshape(channel.shape)


def domain_adapt(img, ref, rng=None, randomise=False):
    """BGR uint8 -> BGR uint8 matched to the real camera distribution."""
    f = np.stack([_match(img[:, :, c], ref[c]) for c in range(3)], axis=2)
    noise, sharp, sat = NOISE_SD, SHARPEN, SAT_PULL
    if randomise:
        rng = rng or np.random.default_rng()
        f *= rng.uniform(0.85, 1.15)            # exposure jitter (time-of-day)
        noise = rng.uniform(3.0, 10.0)          # ISO / sensor
        sharp *= rng.uniform(0.0, 1.3)
        sat *= rng.uniform(0.85, 1.2)
    hsv = cv2.cvtColor(np.clip(f, 0, 255).astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat, 0, 255)
    f = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR).astype(np.float32)
    if sharp > 0:                               # recover some high-frequency detail
        f = f + sharp * (f - cv2.GaussianBlur(f, (0, 0), 1.0))
    n = (rng or np.random).normal(0, noise, f.shape)
    return np.clip(f + n, 0, 255).astype(np.uint8)


def _apply_folder(src=SIM_DIR, dst=OUT_DIR, randomise=False):
    if cv2 is None:
        raise SystemExit("cv2 unavailable")
    os.makedirs(dst, exist_ok=True)
    ref = real_reference()
    rng = np.random.default_rng(0)
    paths = sorted(glob.glob(os.path.join(src, "*.png")))
    for p in paths:
        im = cv2.imread(p, cv2.IMREAD_COLOR)
        if im is not None:
            cv2.imwrite(os.path.join(dst, os.path.basename(p)),
                        domain_adapt(im, ref, rng, randomise))
    return len(paths)


if __name__ == "__main__":
    rand = "--randomise" in sys.argv
    dst = RAND_DIR if rand else OUT_DIR
    n = _apply_folder(dst=dst, randomise=rand)
    mode = "randomised" if rand else "adapted"
    print(f"[adapt] wrote {n} {mode} frames -> {os.path.relpath(dst, os.getcwd())}\n")
    print("=== BEFORE (raw sim vs real) ==="); before = report(SIM_DIR, REAL_DIR)
    print(f"\n=== AFTER  ({mode} sim vs real) ==="); after = report(dst, REAL_DIR)
    print(f"\n[adapt] median sim-to-real gap: {before:.2f} -> {after:.2f} sd "
          f"({100 * (1 - after / before):.0f}% closer)")
