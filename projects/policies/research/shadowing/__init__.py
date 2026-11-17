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

"""Shadowing -- learning deployable robot motion by shadowing dynamically-feasible ghosts.

Components (see docs/developer/shadowing.md):
  1. ghost_generator -- robot-agnostic feasible-ghost generation (trajectory optimization
     / predictive-sampling MPC over a MuJoCo model). (model, intent) -> feasible ghost.
  2. ghost_verifier  -- numerical feasibility certificate (re-sim + metrics), gate before RL.
  3. (tracker)       -- RL shadows the certified ghost; lives in projects/policies/research/training.
"""
