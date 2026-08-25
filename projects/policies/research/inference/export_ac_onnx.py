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

"""Export a raw-torch AC policy (from gpu_mjwarp_trainer / cpu_mj_trainer)
to ONNX for the omniquad_rl_deploy controller.

The trainer's actor is pi: Linear(49,256)->Tanh->Linear(256,128)->Tanh->
Linear(128,12). Deploy runs the DETERMINISTIC action = actor mean (no sample,
no squash), input name "obs", output name "action" -- matching what
omniquad_rl_deploy.py feeds: sess.run(None, {"obs": obs.reshape(1,-1)})[0][0].

Also writes robot_spec.json next to the .onnx so the deploy controller picks
up the exact CPG / action_scale / pitch_trim the policy trained with.

Run:
    python projects/policies/research/inference/export_ac_onnx.py \
        --pt projects/policies/research/training/runs/cpu_omniquad/policy_cpu.pt \
        --out projects/policies/research/inference/policies/gpu_omniquad/policy.onnx \
        --cpg-freq 2.6 --cpg-hipy 0.16 --cpg-knee 0.22 --action-scale 0.15 --pitch-trim 0.2
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

OBS_DIM, NJ = 49, 12


class ActorMean(nn.Module):
    def __init__(self):
        super().__init__()
        self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, NJ))

    def forward(self, obs):
        return self.pi(obs)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--pt", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--cpg-freq", type=float, default=2.6)
    p.add_argument("--cpg-hipy", type=float, default=0.16)
    p.add_argument("--cpg-knee", type=float, default=0.22)
    p.add_argument("--action-scale", type=float, default=0.15)
    p.add_argument("--pitch-trim", type=float, default=0.2)
    args = p.parse_args()

    sd = torch.load(str(args.pt), map_location="cpu")
    # keep only pi.* weights from the full AC state_dict
    pi_sd = {k: v for k, v in sd.items() if k.startswith("pi.")}
    m = ActorMean()
    missing, unexpected = m.load_state_dict(pi_sd, strict=False)
    if missing:
        raise SystemExit(f"missing actor weights: {missing}")
    m.eval()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    dummy = torch.zeros(1, OBS_DIM, dtype=torch.float32)
    torch.onnx.export(m, dummy, str(args.out), input_names=["obs"],
                      output_names=["action"],
                      dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                      opset_version=14)
    print(f"[export_ac_onnx] wrote {args.out}")

    spec = dict(cpg_freq_hz=args.cpg_freq, cpg_hip_y_amp=args.cpg_hipy,
                cpg_knee_amp=args.cpg_knee, action_scale=args.action_scale,
                pitch_trim=args.pitch_trim)
    (args.out.parent / "robot_spec.json").write_text(json.dumps(spec))
    print(f"[export_ac_onnx] wrote robot_spec.json: {spec}")

    import onnxruntime as ort
    sess = ort.InferenceSession(str(args.out), providers=["CPUExecutionProvider"])
    test = np.random.randn(1, OBS_DIM).astype(np.float32)
    a_onnx = sess.run(None, {"obs": test})[0]
    with torch.no_grad():
        a_torch = m(torch.from_numpy(test)).numpy()
    print(f"[export_ac_onnx] sanity diff onnx-vs-torch: {float(np.max(np.abs(a_onnx-a_torch))):.2e}")


if __name__ == "__main__":
    main()
