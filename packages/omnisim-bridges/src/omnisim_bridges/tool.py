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

"""Tool -- a single named action the agent can invoke.

`dispatch(args: dict) -> dict` is the bridge-side action. The dict it
returns becomes the tool result the LLM sees on the next turn, so it
should be small and informative (e.g. `{"q": [...], "err": 0.001}`,
not raw motor objects).

Identical shape to the Tool class in OmniSim's omnilink_relay so a
real-robot bridge using this package, and a sim bridge using the
in-tree relay, register tools the same way.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, Optional

# ⚠️ THE LEGACY CLASSIFIER, KEPT ONLY AS A DEFAULT.
#
# Whether a tool can move a robot was decided by substring-matching its
# NAME, and that tuple was copy-pasted into five files (bridge_base,
# relay, and the mobile / arm / quadruped controllers). It is wrong in
# both directions -- `get_drive_status` matches "drive" and is read-only;
# `activate_sprayer` matches nothing and is not.
#
# It survives here as the fallback for a Tool constructed without an
# explicit `physical=`, so adding the field changed no behaviour on the
# day it landed. Declare `physical=` on new tools and this never runs.
_PHYSICAL_NAME_HINTS = (
    "drive", "turn", "move", "set_velocity", "stop", "reset", "resume",
    "trolley", "pick", "place", "gripper", "grasp", "release", "joint",
    "tcp", "wave", "sit", "stand", "walk", "takeoff", "land", "hover",
)


def _looks_physical(name: str) -> bool:
    """The pre-2026-09 heuristic. Only used when nothing declared better."""
    low = name.lower()
    return any(h in low for h in _PHYSICAL_NAME_HINTS)


class Tool:
    """One tool an OmniLink agent can call.

    Construct with a name, human description, JSON-schema parameter
    spec, and a callable that takes a dict of args and returns a result
    dict. `to_definition()` produces the OpenAI-style schema the
    OmniLink platform expects in `availableToolDetails`.

    `physical` and `surface` exist so the safety gate can be built FROM
    the registry instead of from a second, hand-maintained copy of it.
    `gate.SPECS` was that copy, and it had drifted: ten of the arm
    bridge's nineteen tools were absent from it, so `grasp`,
    `set_tcp_target` and `set_gripper_width` reached the robot with no
    schema, magnitude or deixis check at all -- while `place{xyz}` and
    `attach_trolley{def}`, whose real parameters the gate did not know,
    were refused outright. Both failures are one bug: the schema was
    already right here, in `parameters`, and the gate was reading
    somewhere else.
    """

    def __init__(
        self,
        name: str,
        description: str,
        parameters: Dict[str, Any],
        dispatch: Callable[[Dict[str, Any]], Dict[str, Any]],
        physical: Optional[bool] = None,
        surface: Optional[str] = None,
    ) -> None:
        self.name = name
        self.description = description
        self.parameters = parameters
        self.dispatch = dispatch
        # None means "nobody said", which is why the fallback is a
        # separate branch rather than a default argument value: a tool
        # that explicitly declares physical=False must stay read-only
        # even if its name contains "stop".
        self.physical = _looks_physical(name) if physical is None else bool(physical)
        self.declared_physical = physical is not None
        self.surface = surface

    def to_definition(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "parameters": self.parameters,
        }

    def to_definition(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "parameters": self.parameters,
        }
