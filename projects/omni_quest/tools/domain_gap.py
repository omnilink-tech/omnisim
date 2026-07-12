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

"""Objectively measure (and track the closing of) the sim-to-real image gap.

"Make the sim look more real" is meaningless without a number. This computes a set
of appearance statistics over a folder of images -- brightness, contrast, colour
balance, saturation, colourfulness, edge/texture density, sensor noise, dynamic
range -- and reports how far the SIM distribution sits from REAL (KITTI), per
feature, in pooled standard deviations (|mean_sim - mean_real| / pooled_std). The
overall gap is the mean across features: lower = more similar. Run it before and
after domain randomisation / sensor realism to prove the gap shrank.

    python tools/domain_gap.py [sim_dir] [real_dir]
"""

from __future__ import annotations

import glob
import math
import os
import sys

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(HERE, "..", "_kitti", "sim")
REAL_DIR = os.path.join(HERE, "..", "_kitti", "seq", "2011_09_26",
                        "2011_09_26_drive_0001_sync", "image_02", "data")
SIZE = (256, 192)
FEATURES = ["brightness", "contrast", "saturation", "colorfulness",
            "edge_density", "texture", "noise", "dyn_range", "R", "G", "B"]


def feats(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    b, gr, r = (img[:, :, 0].astype(np.float32), img[:, :, 1].astype(np.float32),
                img[:, :, 2].astype(np.float32))
    rg, yb = r - gr, 0.5 * (r + gr) - b                      # Hasler-Susstrunk
    colorful = (math.hypot(rg.std(), yb.std())
                + 0.3 * math.hypot(rg.mean(), yb.mean()))
    sx, sy = cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1)
    med = cv2.medianBlur(img, 3).astype(np.float32)          # noise = residual vs median
    p2, p98 = np.percentile(g, [2, 98])
    return {
        "brightness": float(g.mean()), "contrast": float(g.std()),
        "saturation": float(hsv[:, :, 1].mean()), "colorfulness": float(colorful),
        "edge_density": float(np.sqrt(sx * sx + sy * sy).mean()),
        "texture": float(cv2.Laplacian(g, cv2.CV_32F).var()),
        "noise": float(np.abs(img.astype(np.float32) - med).mean()),
        "dyn_range": float(p98 - p2),
        "R": float(r.mean()), "G": float(gr.mean()), "B": float(b.mean()),
    }


def folder_stats(folder, n=60):
    paths = sorted(glob.glob(os.path.join(folder, "*.png")))[:n]
    rows = []
    for p in paths:
        im = cv2.imread(p, cv2.IMREAD_COLOR)
        if im is not None:
            rows.append(feats(cv2.resize(im, SIZE)))
    if not rows:
        raise SystemExit(f"no images in {folder}")
    return {k: (float(np.mean([r[k] for r in rows])),
               float(np.std([r[k] for r in rows]))) for k in FEATURES}, len(rows)


def report(sim_dir=SIM_DIR, real_dir=REAL_DIR):
    if cv2 is None:
        raise SystemExit("cv2 unavailable")
    sim, ns = folder_stats(sim_dir)
    real, nr = folder_stats(real_dir)
    print(f"sim-to-real gap: {ns} sim frames vs {nr} real (KITTI) frames")
    print(f"{'feature':<14}{'sim':>12}{'real':>12}{'gap(sd)':>10}")
    gaps = []
    for k in FEATURES:
        (sm, ss), (rm, rs) = sim[k], real[k]
        pooled = 0.5 * (ss + rs) + 1e-6
        g = abs(sm - rm) / pooled
        gaps.append(g)
        print(f"{k:<14}{sm:>12.2f}{rm:>12.2f}{g:>10.2f}")
    overall = float(np.median(gaps))     # median: robust to low-variance outliers
    print(f"{'OVERALL GAP':<14}{'':>12}{'':>12}{overall:>10.2f}  "
          f"(median sd-distance; mean {float(np.mean(gaps)):.1f}; lower = more similar)")
    return overall


if __name__ == "__main__":
    a = sys.argv[1:]
    report(a[0] if len(a) > 0 else SIM_DIR, a[1] if len(a) > 1 else REAL_DIR)
