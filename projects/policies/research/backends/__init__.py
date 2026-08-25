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

"""Training backends for OmniSim RL.

A backend takes a RobotSpec (URDF + observation/action contract + reward
recipe) plus a TrainingConfig (steps, hyperparams), and produces a
policy.onnx that the OmniSim deploy controller can load.

Backends are pluggable: same CLI, same output. The shipped backends are:

  sb3    : stable-baselines3 PPO + OmniSim subprocesses. CPU, slow,
           always works. Default. ~150-500 steps/s aggregate.
  mjx    : MuJoCo MJX + custom JAX PPO. GPU-batched (1000+ envs in
           parallel). Best on Linux/WSL2 + NVIDIA. CPU fallback on
           bare Windows.
  isaac  : NVIDIA Isaac Lab adapter. Requires Isaac Sim install.
           Written from NVIDIA's docs; verification needs Isaac present.

Pick a backend via `train_omniquad.py --backend <name>`. All backends emit
the same ONNX schema (49-D input, 12-D output for OmniQuad) so the deploy
controller works regardless of where the policy was trained.
"""
from .base import TrainingBackend, RobotSpec, TrainingConfig, list_backends, get_backend

__all__ = ["TrainingBackend", "RobotSpec", "TrainingConfig",
           "list_backends", "get_backend"]
