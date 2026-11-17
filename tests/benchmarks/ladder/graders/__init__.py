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

"""Ladder graders: one neutral core per tier, one thin entry point per tier.

Same split as ``agentbench/graders/``, for the same reason (SPEC 6.2.6)::

    adapters/<sim>/  ->  EvidenceBundle  ->  ladder/graders/t<N>_core.py
    (sim-specific)       (+ ladder channels)   (physical units only)

``t1_core`` may be read, argued with and re-run by a third party with no
simulator, no build and no network: every one of its inputs is a number in SI
units. ``test_neutral_core.py`` is the structural guard that keeps it that
way.
"""

from __future__ import annotations

__all__ = []
