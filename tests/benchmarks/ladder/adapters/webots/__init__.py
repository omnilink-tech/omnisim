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

"""The upstream-Webots column for the ladder / loopbench rungs.

Thin by design: AgentBench already brought this column up (its launcher
injects a recorder into a sibling world, runs Webots headless inside WSL2, and
parses what comes back), so this package is a **format shim** rather than a
second integration. All it does is turn that recorder's ``trajectory.json``
into the same wide-format ``phaseB.csv`` the other two columns write, so the
neutral graders receive byte-identical input shapes from all three and a
verdict difference cannot be a parsing difference.
"""
