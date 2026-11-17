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

"""Train the learned free-space segmenter (the ML perception loop).

Distils the classical STEREO+geometry teacher into a monocular MLP: input is a
single image, target is the stereo obstacle mask the eye captured alongside it
(tools/gen_city_nav.py eye, OQ_CAPTURE_DIR -> _kitti/seg_train). Every frame is
augmented with DOMAIN-RANDOMISED variants (tools/domain_adapt.py) that share the
same mask, so the model learns free-space that is invariant to lighting/colour and
transfers to a real camera. Trains with sklearn, then EXPORTS plain numpy weights
(model_seg.npz) the eye runs with numpy+cv2 only -- no sklearn/torch at inference.

    python tools/train_segmenter.py
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

HERE = os.path.dirname(os.path.abspath(__file__))
EYE = os.path.join(HERE, "..", "controllers", "omni_quest_eye")
sys.path.insert(0, EYE)
sys.path.insert(0, HERE)

from learned_seg import pixel_features, SEG_SIZE, FEATURE_NAMES, MODEL_PATH, load, segment  # noqa: E402
from domain_adapt import real_reference, domain_adapt  # noqa: E402
from domain_gap import REAL_DIR  # noqa: E402

TRAIN_DIR = os.path.join(HERE, "..", "_kitti", "seg_train")
EVAL_DIR = os.path.join(HERE, "..", "_kitti", "seg_eval")
PIX_PER = 1500            # balanced pixels sampled per frame-variant
N_VARIANTS = 2           # domain-randomised variants added per frame (+ the raw frame)
VAL_FRAC = 0.2


def load_pairs(d):
    pairs = []
    for ip in sorted(glob.glob(os.path.join(d, "sim_*.png"))):
        if ip.endswith("_mask.png"):
            continue
        mp = ip[:-4] + "_mask.png"
        if os.path.exists(mp):
            img = cv2.imread(ip, cv2.IMREAD_COLOR)
            msk = cv2.imread(mp, cv2.IMREAD_GRAYSCALE)
            if img is not None and msk is not None:
                pairs.append((img, msk))
    return pairs


def label_at(msk):
    m = cv2.resize(msk, SEG_SIZE, interpolation=cv2.INTER_NEAREST)
    return (m.reshape(-1) > 127).astype(np.int64)


def balanced_idx(label, n, rng):
    obs, free = np.where(label == 1)[0], np.where(label == 0)[0]
    k = min(n // 2, len(obs), len(free))
    if k == 0:
        return rng.choice(len(label), min(n, len(label)), replace=False)
    return np.concatenate([rng.choice(obs, k, replace=False),
                           rng.choice(free, k, replace=False)])


def build_xy(pairs, ref, rng):
    xs, ys = [], []
    for img, msk in pairs:
        label = label_at(msk)
        variants = [img] + [domain_adapt(img, ref, rng, randomise=True)
                            for _ in range(N_VARIANTS)]
        for var in variants:
            feats, _ = pixel_features(var)
            idx = balanced_idx(label, PIX_PER, rng)
            xs.append(feats[idx]); ys.append(label[idx])
    return np.concatenate(xs), np.concatenate(ys)


def export(scaler, mlp, path):
    arrs = {"n_layers": np.int64(len(mlp.coefs_)),
            "size": np.array(SEG_SIZE, dtype=np.int64),
            "mean": scaler.mean_.astype(np.float32),
            "std": scaler.scale_.astype(np.float32)}
    for i, (w, b) in enumerate(zip(mlp.coefs_, mlp.intercepts_)):
        arrs[f"w{i}"] = w.astype(np.float32)
        arrs[f"b{i}"] = b.astype(np.float32)
    np.savez(path, **arrs)


def iou_acc(pairs, model, ref=None, rng=None, randomise=False):
    inter = union = correct = total = 0
    for img, msk in pairs:
        var = domain_adapt(img, ref, rng, randomise=True) if randomise else img
        pred = segment(var, model)                       # (H,W) bool
        gt = label_at(msk).reshape(pred.shape).astype(bool)
        inter += int(np.logical_and(pred, gt).sum())
        union += int(np.logical_or(pred, gt).sum())
        correct += int((pred == gt).sum()); total += gt.size
    return (inter / union if union else 1.0), correct / total


def main():
    if cv2 is None:
        raise SystemExit("cv2 unavailable")
    from sklearn.neural_network import MLPClassifier
    from sklearn.preprocessing import StandardScaler

    pairs = load_pairs(TRAIN_DIR)
    if len(pairs) < 10:
        raise SystemExit(f"only {len(pairs)} pairs in {TRAIN_DIR}; capture more first")
    rng = np.random.default_rng(0)
    ref = real_reference()
    obst_frac = float(np.mean([label_at(m).mean() for _, m in pairs]))
    print(f"[seg] {len(pairs)} (image,mask) pairs; teacher obstacle fraction "
          f"{obst_frac:.1%}; features={FEATURE_NAMES}")

    n_val = max(4, int(len(pairs) * VAL_FRAC))
    val_pairs, train_pairs = pairs[:n_val], pairs[n_val:]
    X, y = build_xy(train_pairs, ref, rng)
    print(f"[seg] training on {len(X):,} pixels "
          f"({len(train_pairs)} frames x {1 + N_VARIANTS} variants), "
          f"{int(y.sum()):,} obstacle / {int((1 - y).sum()):,} free")

    scaler = StandardScaler().fit(X)
    mlp = MLPClassifier(hidden_layer_sizes=(32, 16), activation="relu",
                        alpha=1e-3, max_iter=400, random_state=0)
    mlp.fit(scaler.transform(X), y)
    print(f"[seg] train accuracy {mlp.score(scaler.transform(X), y):.3f} "
          f"({mlp.n_iter_} iters)")

    export(scaler, mlp, MODEL_PATH)
    model = load(MODEL_PATH)
    iou_s, acc_s = iou_acc(val_pairs, model)
    iou_r, acc_r = iou_acc(val_pairs, model, ref, rng, randomise=True)
    print(f"[seg] HELD-OUT sim   : pixel-acc {acc_s:.3f}  IoU {iou_s:.3f}")
    print(f"[seg] HELD-OUT +rand : pixel-acc {acc_r:.3f}  IoU {iou_r:.3f}  "
          f"(robustness to the appearance band)")

    # Real-camera transfer (qualitative): a real KITTI street frame should read mostly
    # FREE in its lower half (the road ahead is drivable). No labels exist for real, so
    # we report the free fraction + dump overlays for the user to inspect.
    reals = sorted(glob.glob(os.path.join(REAL_DIR, "*.png")))[:6]
    if reals:
        os.makedirs(EVAL_DIR, exist_ok=True)
        fr = []
        for i, p in enumerate(reals):
            im = cv2.imread(p, cv2.IMREAD_COLOR)
            pm = segment(im, model)
            lower = pm[pm.shape[0] // 2:, :]
            fr.append(1.0 - float(lower.mean()))
            vis = cv2.resize(im, SEG_SIZE)
            vis[pm] = (0.4 * vis[pm] + np.array([0, 0, 153])).astype(np.uint8)  # obstacle = red
            cv2.imwrite(os.path.join(EVAL_DIR, f"real_{i:02d}_overlay.png"), vis)
        print(f"[seg] REAL KITTI lower-half FREE fraction: "
              f"mean {np.mean(fr):.1%} (drivable road ahead); overlays -> "
              f"{os.path.relpath(EVAL_DIR, os.getcwd())}")
    print(f"[seg] exported -> {os.path.relpath(MODEL_PATH, os.getcwd())}")


if __name__ == "__main__":
    main()
