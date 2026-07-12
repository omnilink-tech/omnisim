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

"""Export a trained SB3 PPO policy to ONNX for inference deploy.

Run from repo root:
    python projects/policies/research/inference/export_onnx.py \
        --model projects/policies/research/training/runs/<run>/spot_final.zip \
        --out projects/policies/research/inference/policies/<run>/policy.onnx
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

from stable_baselines3 import PPO


class OnnxablePolicy(nn.Module):
    """Wraps an SB3 PPO policy into a single-input single-output module suitable
    for ONNX export. Returns the deterministic action vector (mean of the
    Gaussian, before action squashing -- SB3 PPO with a Box action space
    doesn't squash by default, so this matches `model.predict(deterministic=True)`).
    """

    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        # mlp_extractor / actor head path used by ActorCriticPolicy
        features = self.policy.extract_features(obs)
        if self.policy.share_features_extractor:
            latent_pi, _ = self.policy.mlp_extractor(features)
        else:
            pi_features, _ = features
            latent_pi = self.policy.mlp_extractor.forward_actor(pi_features)
        mean_actions = self.policy.action_net(latent_pi)
        return mean_actions


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--model", type=Path, required=True,
                   help=".zip path saved by stable_baselines3")
    p.add_argument("--out", type=Path, required=True,
                   help="output .onnx path")
    p.add_argument("--obs-dim", type=int, default=49)
    args = p.parse_args()

    print(f"[export_onnx] loading {args.model}")
    model = PPO.load(str(args.model), device="cpu")
    wrapper = OnnxablePolicy(model.policy)
    wrapper.eval()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    dummy = torch.zeros(1, args.obs_dim, dtype=torch.float32)
    torch.onnx.export(
        wrapper,
        dummy,
        str(args.out),
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=14,
    )
    print(f"[export_onnx] wrote {args.out}")

    # Quick sanity check: run via onnxruntime, compare to SB3 predict.
    import onnxruntime as ort
    sess = ort.InferenceSession(str(args.out), providers=["CPUExecutionProvider"])
    test_obs = np.random.randn(1, args.obs_dim).astype(np.float32)
    onnx_action = sess.run(None, {"obs": test_obs})[0]
    sb3_action, _ = model.predict(test_obs[0], deterministic=True)
    diff = float(np.max(np.abs(onnx_action[0] - sb3_action)))
    print(f"[export_onnx] sanity diff (onnx vs SB3 predict): {diff:.6f}")
    if diff > 1e-3:
        print("[export_onnx] WARNING: ONNX and SB3 disagree -- inspect manually")
    return 0


if __name__ == "__main__":
    sys.exit(main())
