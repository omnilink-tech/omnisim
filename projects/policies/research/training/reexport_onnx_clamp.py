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

"""Re-export existing .pt policies to ONNX with CLAMP (the trainer's exact
action squashing) instead of the old tanh wrap. No retraining needed.

OBS_DIM / NJ are auto-detected from the .pt state_dict (the pi MLP's first/last
Linear shapes), so this works for any policy size (50, 53, ...) -- the trainers'
auto-export wraps tanh, which diverges from the clamp(mu,-1,1) used in eval/step."""
import sys
from pathlib import Path

import torch
import torch.nn as nn


def _make_ac(obs_dim, nj):
    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(obs_dim, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(),
                                    nn.Linear(128, nj))
            self.v = nn.Sequential(nn.Linear(obs_dim, 256), nn.Tanh(),
                                   nn.Linear(256, 128), nn.Tanh(),
                                   nn.Linear(128, 1))
            self.log_std = nn.Parameter(-1.0 * torch.ones(nj))
    return AC()


class DeployPolicy(nn.Module):
    def __init__(self, pi):
        super().__init__()
        self.pi = pi

    def forward(self, obs):
        return torch.clamp(self.pi(obs), -1.0, 1.0)


for pt in sys.argv[1:]:
    sd = torch.load(pt, map_location="cpu")
    obs_dim = sd["pi.0.weight"].shape[1]          # first Linear: [256, obs_dim]
    nj = sd["pi.4.weight"].shape[0]               # last Linear:  [nj, 128]
    ac = _make_ac(obs_dim, nj)
    ac.load_state_dict(sd)
    wrapped = DeployPolicy(ac.pi)
    wrapped.eval()
    onnx_path = Path(pt).with_suffix(".onnx")
    df = Path(str(onnx_path) + ".data")           # drop stale external-data from a prior export
    if df.exists():
        df.unlink()
    torch.onnx.export(wrapped, torch.zeros(1, obs_dim), str(onnx_path),
                      input_names=["obs"], output_names=["action"],
                      dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                      opset_version=17)
    print(f"re-exported (clamp, obs={obs_dim}, nj={nj}): {onnx_path}")
