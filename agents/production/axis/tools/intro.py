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

"""Self-introspection: report Axis's runtime config."""

from __future__ import annotations

import os
from typing import Any, Dict

from ._base import BRIDGE_URL, BRIDGE_TIMEOUT, SAFE, ToolSpec


def _impl_whoami(**_: Any) -> Dict[str, Any]:
    return {
        "agent": "Axis",
        "role": "robot-control agent for OmniSim",
        "dry_run": os.environ.get("AXIS_DRY_RUN", "0").strip() not in ("0", "false", "no", ""),
        "bridge": {
            "url": BRIDGE_URL,
            "timeout_seconds": BRIDGE_TIMEOUT,
        },
        "manifest_mode": os.environ.get("AXIS_MANIFEST_MODE", "full"),
        "auto_approve": True,
        "notes": (
            "Axis proxies motion commands to the OmniSim bridge. "
            "stop_robot is always safe. Validate telemetry freshness before "
            "any motion. Cap joint steps at IK_MAX_DQ; use plan_trajectory "
            "for larger moves."
        ),
    }


SPEC = ToolSpec(
    name="whoami",
    tier=SAFE,
    description="Report Axis's current runtime: bridge URL, dry-run flag, manifest mode, and safety posture.",
    parameters={"type": "object", "properties": {}},
    impl=_impl_whoami,
)
