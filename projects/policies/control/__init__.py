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

"""Model-based control primitives for OmniSim robots.

`omniquad_kinematics`: analytic forward/inverse kinematics for OmniQuad legs.
`omniquad_gait`:       trot foot-trajectory generator.

The model layer produces feasible joint targets directly from a body
velocity command — no RL involved. A small residual policy can then sit
on top of this to compensate for model error / perturbations / terrain.
"""
