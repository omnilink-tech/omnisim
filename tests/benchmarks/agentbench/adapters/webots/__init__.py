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

"""The upstream **Webots R2025a** evidence adapter -- AgentBench's control arm.

    ``evidence.build_bundle()``   one run's artifacts -> a neutral EvidenceBundle
    ``recording.read_run()``      finding and parsing those artifacts
    ``worldtext.scan()``          the ``.wbt`` declaration side + the world frame

Read ``evidence.py``'s module docstring for what upstream can and cannot
evidence, and ``recording.py``'s for the artifact contract the Webots lane has to
write. Install, headless invocation and the traps that invalidate a run are in
``docs/developer/webots-control-baseline.md``.

``build_bundle`` is re-exported here so ``import agentbench.adapters.webots``
is enough at a REPL; the registry in ``adapters/__init__.py`` resolves the
``.evidence`` submodule directly.
"""

from __future__ import annotations

from agentbench.adapters.webots.evidence import (ADAPTER, SIM, WEBOTS_RELEASE,
                                                 build_bundle)

__all__ = ["ADAPTER", "SIM", "WEBOTS_RELEASE", "build_bundle"]
