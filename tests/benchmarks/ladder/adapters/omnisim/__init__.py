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

"""The **OmniSim column**'s ladder-side additions. Everything else is reused.

The neutral bundle for this column is built by
``agentbench.adapters.omnisim.evidence.build_bundle``, unchanged and by
import. What lives here is the one thing that module cannot supply, because
the sampler it reads is part of a frozen task set: the **support-contact
channel** T1.4 rests on.

    ladder/controllers/ladder_recorder/   the grader-owned pose + contact
                                          sampler (why it is not the
                                          AgentBench one: that module's
                                          docstring)
    evidence.py                           the phase-B launcher that injects
                                          it, and the mapping from what it
                                          writes into both the frozen bundle's
                                          shape and the ladder's channel
"""

from __future__ import annotations

__all__ = []
