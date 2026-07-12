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

"""Learned free-space segmenter — portable numpy inference (no sklearn / torch).

A small MLP, trained offline by tools/train_segmenter.py to reproduce the STEREO
teacher's obstacle mask from a SINGLE image, with domain randomisation so it
transfers to a real camera. It replaces the hand-tuned colour thresholds in
omni_quest_eye (which break under real lighting and can't separate grey pavement
from a grey wall): the learned model uses colour AND local texture AND row prior,
and is trained across the randomised appearance band that brackets real KITTI.

Inference runs in the eye's EMBEDDED interpreter, so this module depends on only
numpy + cv2; the trained weights ship as a plain .npz (model_seg.npz) loaded here.
The feature function is shared with the trainer so the two can never drift.
"""

from __future__ import annotations

import os

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None

SEG_SIZE = (128, 96)        # (W, H) working resolution for features / inference
# colour (3 spaces) + grey + gradient magnitude + local texture energy + row prior.
# No column feature: it would overfit the scene layout; row is physically meaningful
# (a forward camera's ground is always low in frame — true in sim AND on real roads).
FEATURE_NAMES = ["B", "G", "R", "H", "S", "V", "grey", "grad", "texture", "row"]
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model_seg.npz")


def pixel_features(img_bgr, size=SEG_SIZE):
    """BGR uint8 image -> ((H*W, F) float32 per-pixel features, (H, W)). Deterministic
    per pixel (no per-image normalisation) so training and inference features match."""
    im = cv2.resize(img_bgr, size).astype(np.float32)
    w, h = size
    b, g, r = im[:, :, 0] / 255.0, im[:, :, 1] / 255.0, im[:, :, 2] / 255.0
    hsv = cv2.cvtColor(im.astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.float32)
    hh, ss, vv = hsv[:, :, 0] / 180.0, hsv[:, :, 1] / 255.0, hsv[:, :, 2] / 255.0
    grey = 0.114 * b + 0.587 * g + 0.299 * r
    gx = cv2.Sobel(grey, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(grey, cv2.CV_32F, 0, 1, ksize=3)
    grad = np.sqrt(gx * gx + gy * gy)                       # edges: walls/cars vs smooth road
    lap = cv2.Laplacian(grey, cv2.CV_32F)
    texture = cv2.boxFilter(lap * lap, -1, (7, 7))          # local high-freq energy
    row = np.repeat(np.linspace(0.0, 1.0, h)[:, None], w, axis=1)
    feats = np.stack([b, g, r, hh, ss, vv, grey, grad, texture, row], axis=2)
    return feats.reshape(-1, feats.shape[2]).astype(np.float32), (h, w)


def load(path=MODEL_PATH):
    """Load the exported MLP (StandardScaler + relu layers + sigmoid head)."""
    d = np.load(path)
    n = int(d["n_layers"])
    return {"size": (int(d["size"][0]), int(d["size"][1])),
            "mean": d["mean"].astype(np.float32),
            "std": d["std"].astype(np.float32),
            "weights": [d[f"w{i}"].astype(np.float32) for i in range(n)],
            "biases": [d[f"b{i}"].astype(np.float32) for i in range(n)]}


def _forward(x, model):
    a = (x - model["mean"]) / model["std"]
    last = len(model["weights"]) - 1
    for i, (wl, bl) in enumerate(zip(model["weights"], model["biases"])):
        a = a @ wl + bl
        if i < last:
            a = np.maximum(a, 0.0)                           # relu hidden
    return 1.0 / (1.0 + np.exp(-a[:, 0]))                    # sigmoid head -> P(obstacle)


def segment(img_bgr, model, thresh=0.5):
    """BGR uint8 -> (H, W) bool obstacle mask (True = obstacle), at SEG_SIZE."""
    x, (h, w) = pixel_features(img_bgr, model["size"])
    return (_forward(x, model) > thresh).reshape(h, w)


def prob_map(img_bgr, model):
    """BGR uint8 -> (H, W) float32 P(obstacle) — for visualisation / soft fusion."""
    x, (h, w) = pixel_features(img_bgr, model["size"])
    return _forward(x, model).reshape(h, w).astype(np.float32)
