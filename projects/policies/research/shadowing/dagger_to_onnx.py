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

"""Export a DAgger-distilled launch policy (.pt from dagger_launch.py) to ONNX for OmniSim
deploy. The exported graph takes the RAW 60-dim observation and outputs ABSOLUTE position
targets (rad): obs -> (standardize) -> MLP -> tanh -> (unnormalize to joint range). So the
deploy controller just assembles the obs and applies the output directly -- no extra math.
"""
import argparse
import json
import os
import sys

import numpy as np
import torch
import torch.nn as nn

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.dagger_launch import build_policy, OUTDIR


class DeployNet(nn.Module):
    """Wraps the MLP with obs-standardization + action-unnormalization baked in."""

    def __init__(self, mlp, obs_mean, obs_std, ctrl_lo, ctrl_hi):
        super().__init__()
        self.mlp = mlp
        self.register_buffer("obs_mean", torch.tensor(obs_mean, dtype=torch.float32))
        self.register_buffer("obs_std", torch.tensor(obs_std, dtype=torch.float32))
        self.register_buffer("lo", torch.tensor(ctrl_lo, dtype=torch.float32))
        self.register_buffer("hi", torch.tensor(ctrl_hi, dtype=torch.float32))

    def forward(self, obs):
        a = self.mlp((obs - self.obs_mean) / self.obs_std)   # tanh in (-1,1)
        return self.lo + (a + 1.0) * 0.5 * (self.hi - self.lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="dagger_g1_launch")
    ap.add_argument("--total-time", type=float, default=6.0, help="intent total_time (phase clock denom)")
    ap.add_argument("--rise-time", type=float, default=3.5, help="phase freezes here for the stand HOLD")
    args = ap.parse_args()
    pt = os.path.join(OUTDIR, args.name + ".pt")
    ck = torch.load(pt, map_location="cpu")
    mlp = build_policy(ck["obs_dim"], ck["act_dim"])
    mlp.load_state_dict(ck["state_dict"])
    mlp.eval()
    net = DeployNet(mlp, ck["obs_mean"], ck["obs_std"], ck["ctrl_lo"], ck["ctrl_hi"])
    net.eval()
    dummy = torch.zeros(1, ck["obs_dim"])
    out = os.path.join(OUTDIR, args.name + ".onnx")
    torch.onnx.export(net, dummy, out, input_names=["obs"], output_names=["targets"],
                      dynamic_axes={"obs": {0: "n"}, "targets": {0: "n"}}, opset_version=13)
    # sanity: torch vs onnxruntime
    with torch.no_grad():
        t_out = net(dummy).numpy()
    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(out, providers=["CPUExecutionProvider"])
        o_out = sess.run(None, {"obs": dummy.numpy()})[0]
        err = float(np.max(np.abs(t_out - o_out)))
        print(f"[onnx] saved {out}  torch-vs-ort max|d|={err:.2e}  act_names={len(ck['act_names'])}")
    except Exception as e:
        print(f"[onnx] saved {out} (ort check skipped: {e})")
    # sidecar the deploy controller needs: joint order (== ONNX output order) + phase clock.
    joints = [n.replace("_pos", "") for n in ck["act_names"]]
    meta = {"joints": joints, "obs_dim": int(ck["obs_dim"]), "act_dim": int(ck["act_dim"]),
            "total_time": float(args.total_time), "rise_time": float(args.rise_time),
            "control_dt": 0.016}
    side = os.path.join(OUTDIR, args.name + ".json")
    with open(side, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"[onnx] sidecar {side}  joints={len(joints)}")


if __name__ == "__main__":
    main()
