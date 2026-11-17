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

# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Shared helpers for OmniLink agents that ship co-located with OmniSim.

Each productized agent under ``agents/production/`` previously hand-rolled
~250 lines of identical boilerplate: omnilink-lib path discovery, a
threaded HTTP tool-callback server, profile push, UsageMeter wiring,
memory polling. This package extracts that into a small SDK so a new
agent's ``<name>_agent.py`` can be ~30 lines.

Pieces:

* :class:`OmniLinkAgentRunner` (``runner_base``) — the agent-side
  process. Owns the tool-callback HTTP server, profile management,
  UsageMeter, and memory-poll loop.

* :class:`OmniSimBridgeServer` (``bridge_base``) — the OmniSim-side
  controller. Owns the ``GET /state`` / ``GET /capabilities`` /
  ``POST /action`` HTTP surface, action dispatch table, and CORS.
  This is the **action-dispatch** bridge style — handlers receive a
  JSON body and return a dict.

* :class:`OmniLinkHTTPBridge` (re-exported from ``omnilink-lib`` when
  reachable) — the **text-command** bridge style for natural-language
  routing through :class:`omnilink.OmniLinkEngine`. Different surface:
  ``POST /command`` / ``GET /feedback`` / ``GET /context`` /
  ``POST /inline-code``. Use this when a bridge takes plain-English
  command strings and dispatches to handler functions by template
  match (e.g. ``"launch [vehicle]"``). Use ``OmniSimBridgeServer``
  when the bridge takes structured ``{action, ...args}`` JSON
  (the standard OmniSim robot-bridge pattern).

* :mod:`omnisim_damage` — the existing damage-system client (kept
  here for back-compat).
"""

from .runner_base import OmniLinkAgentRunner, locate_omnilink_lib
from .bridge_base import OmniSimBridgeServer, action
from .supervision import (
    ActionOutcome,
    execute_verified_sequence,
    failure_error,
    find_duplicate_resource_claims,
    normalize_outcome,
)
from .usage_log import USAGE_ROOT, UsageRecord, write_run_snapshot

# Best-effort re-export of OmniLinkHTTPBridge so that
#   from _lib import OmniLinkHTTPBridge
# works in both styles of bridge. omnilink-lib is the canonical home for
# the text-command bridge; we only re-export when it's importable.
try:  # pragma: no cover - depends on sibling olink/ checkout
    locate_omnilink_lib()
    from omnilink import OmniLinkHTTPBridge, OmniLinkEngine  # type: ignore[F401]
    _HAS_OMNILINK_HTTP_BRIDGE = True
except Exception:  # pragma: no cover
    OmniLinkHTTPBridge = None  # type: ignore[assignment]
    OmniLinkEngine = None  # type: ignore[assignment]
    _HAS_OMNILINK_HTTP_BRIDGE = False


__all__ = [
    "OmniLinkAgentRunner",
    "OmniSimBridgeServer",
    "OmniLinkHTTPBridge",
    "OmniLinkEngine",
    "ActionOutcome",
    "UsageRecord",
    "USAGE_ROOT",
    "action",
    "execute_verified_sequence",
    "failure_error",
    "find_duplicate_resource_claims",
    "locate_omnilink_lib",
    "normalize_outcome",
    "write_run_snapshot",
]
