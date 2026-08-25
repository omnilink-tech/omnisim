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

"""Build a "zero-action" ONNX policy file.

The output ONNX is a constant-zero MLP: same input/output schema as the
SB3-trained policies (obs[1, obs_dim] -> action[1, act_dim]), but always
returns the all-zeros action vector. Used as the deploy fallback when
training fails — the deploy controller then drives the robot purely from
the CPG trot prior (action = CPG + 0.15 * 0 = pure CPG offsets).

Also writes the matching robot_spec.json sidecar so the deploy controller
can load it directly.

Usage (from repo root):
    python projects/policies/research/tools/make_zero_policy.py --robot omniquad \
        --out projects/policies/research/inference/policies/omniquad_cpg_zero/policy.onnx
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))

import numpy as np
import torch
import torch.nn as nn


class ZeroPolicy(nn.Module):
    def __init__(self, act_dim: int):
        super().__init__()
        self.act_dim = act_dim

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        # Returns zeros with batch dim from obs.
        return torch.zeros(obs.shape[0], self.act_dim, dtype=obs.dtype, device=obs.device)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--robot", default="omniquad")
    p.add_argument("--out", type=Path, required=True)
    args = p.parse_args()

    from projects.policies.research.backends.robot_registry import get_robot
    robot = get_robot(args.robot)
    act_dim = len(robot.joint_names)
    obs_dim = robot.obs_dim

    model = ZeroPolicy(act_dim)
    model.eval()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    dummy = torch.zeros(1, obs_dim, dtype=torch.float32)
    torch.onnx.export(
        model, dummy, str(args.out),
        input_names=["obs"], output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=14,
    )
    print(f"[zero_policy] wrote {args.out}")

    spec_path = robot.write_sidecar(args.out.parent)
    print(f"[zero_policy] wrote spec {spec_path}")

    import onnxruntime as ort
    sess = ort.InferenceSession(str(args.out), providers=["CPUExecutionProvider"])
    test = sess.run(None, {"obs": np.zeros((1, obs_dim), dtype=np.float32)})[0]
    print(f"[zero_policy] sanity: action shape={test.shape} all-zeros={np.all(test == 0)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
