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

"""Train the AnyPick part classifier on in-sim camera crops.

Dataset: 48x48 grayscale PGM crops captured by anypick_cam in dataset mode
(ANYPICK_CAPTURE=<dir>), one subfolder per class label (labelled by the
recognition colour, which is ground truth in sim). Crops under unknown/ are
NOT trained on -- they are the out-of-distribution validation set for the
confidence threshold.

Trains a small CNN (2 conv + 2 fc) to classify part SHAPE from the image
(grayscale input, so colour cannot be the shortcut), reports in-distribution
accuracy + OOD rejection at the deploy threshold, and exports ONNX to
projects/policies/research/inference/policies/anypick_cls/cls.onnx (+ classes.json).

Usage:
  python scripts/dev/train_anypick_classifier.py [--data DIR] [--epochs 30]
"""

import argparse
import json
import os
import random
import sys

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEF_DATA = os.path.join(ROOT, "projects", "policies", "research", "training",
                        "datasets", "anypick_parts")
DEF_OUT = os.path.join(ROOT, "projects", "policies", "research", "inference",
                       "policies", "anypick_cls")
CONF_T = 0.75                    # deploy threshold: below -> "unknown"


def read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            return None
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = (int(v) for v in line.split())
        f.readline()                                 # maxval
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return img.astype(np.float32) / 255.0


class Net(nn.Module):
    def __init__(self, n_cls):
        super().__init__()
        self.c1 = nn.Conv2d(1, 16, 5, stride=2, padding=2)   # 48 -> 24
        self.c2 = nn.Conv2d(16, 32, 3, stride=2, padding=1)  # 24 -> 12
        self.c3 = nn.Conv2d(32, 32, 3, stride=2, padding=1)  # 12 -> 6
        self.f1 = nn.Linear(32 * 6 * 6, 64)
        self.f2 = nn.Linear(64, n_cls)

    def forward(self, x):
        x = F.relu(self.c1(x))
        x = F.relu(self.c2(x))
        x = F.relu(self.c3(x))
        x = F.relu(self.f1(x.flatten(1)))
        return self.f2(x)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=DEF_DATA)
    ap.add_argument("--out", default=DEF_OUT)
    ap.add_argument("--epochs", type=int, default=30)
    args = ap.parse_args()

    classes = sorted(d for d in os.listdir(args.data)
                     if os.path.isdir(os.path.join(args.data, d)) and d != "unknown")
    xs, ys = [], []
    for ci, cname in enumerate(classes):
        cdir = os.path.join(args.data, cname)
        for fn in os.listdir(cdir):
            img = read_pgm(os.path.join(cdir, fn))
            if img is not None and img.shape == (48, 48):
                xs.append(img)
                ys.append(ci)
    ood = []
    udir = os.path.join(args.data, "unknown")
    if os.path.isdir(udir):
        for fn in os.listdir(udir):
            img = read_pgm(os.path.join(udir, fn))
            if img is not None and img.shape == (48, 48):
                ood.append(img)
    print(f"classes={classes} n={len(xs)} ood={len(ood)}")
    if len(xs) < 50 * len(classes):
        print("WARNING: thin dataset")

    idx = list(range(len(xs)))
    random.Random(0).shuffle(idx)
    n_val = max(1, len(idx) // 6)
    val_i, tr_i = idx[:n_val], idx[n_val:]
    X = torch.tensor(np.stack(xs)[:, None, :, :])
    Y = torch.tensor(ys)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    net = Net(len(classes)).to(dev)
    opt = torch.optim.Adam(net.parameters(), lr=2e-3)
    Xtr, Ytr = X[tr_i].to(dev), Y[tr_i].to(dev)
    Xva, Yva = X[val_i].to(dev), Y[val_i].to(dev)

    for ep in range(args.epochs):
        net.train()
        perm = torch.randperm(len(Xtr), device=dev)
        tot = 0.0
        for k in range(0, len(perm), 128):
            b = perm[k:k + 128]
            xb = Xtr[b]
            # light augmentation: random flips (the camera sees parts at any yaw)
            if random.random() < 0.5:
                xb = torch.flip(xb, dims=[3])
            if random.random() < 0.5:
                xb = torch.flip(xb, dims=[2])
            opt.zero_grad()
            loss = F.cross_entropy(net(xb), Ytr[b])
            loss.backward()
            opt.step()
            tot += float(loss) * len(b)
        net.eval()
        with torch.no_grad():
            acc = float((net(Xva).argmax(1) == Yva).float().mean())
        print(f"epoch {ep + 1}: loss {tot / len(Xtr):.3f} val acc {acc:.3f}")

    # OOD rejection at the deploy threshold
    net.eval()
    with torch.no_grad():
        conf_va = float(F.softmax(net(Xva), 1).max(1).values.mean())
        rej = ood_acc = None
        if ood:
            Xo = torch.tensor(np.stack(ood)[:, None, :, :]).to(dev)
            co = F.softmax(net(Xo), 1).max(1).values
            rej = float((co < CONF_T).float().mean())
        idacc = float((F.softmax(net(Xva), 1).max(1).values >= CONF_T).float().mean())
    print(f"val mean conf {conf_va:.3f}; in-dist accepted@{CONF_T} {idacc:.3f}; "
          f"OOD rejected@{CONF_T} {rej if rej is not None else 'n/a'}")

    os.makedirs(args.out, exist_ok=True)
    onnx_path = os.path.join(args.out, "cls.onnx")
    torch.onnx.export(net.cpu(), torch.zeros(1, 1, 48, 48), onnx_path,
                      input_names=["img"], output_names=["logits"],
                      opset_version=17)
    with open(os.path.join(args.out, "classes.json"), "w") as f:
        json.dump({"classes": classes, "conf_threshold": CONF_T}, f)
    print(f"exported {onnx_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
