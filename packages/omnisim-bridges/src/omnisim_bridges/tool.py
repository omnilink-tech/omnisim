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

from typing import Any, Callable, Dict


class Tool:
    """One tool an OmniLink agent can call.

    Construct with a name, human description, JSON-schema parameter
    spec, and a callable that takes a dict of args and returns a result
    dict. `to_definition()` produces the OpenAI-style schema the
    OmniLink platform expects in `availableToolDetails`.
    """

    def __init__(
        self,
        name: str,
        description: str,
        parameters: Dict[str, Any],
        dispatch: Callable[[Dict[str, Any]], Dict[str, Any]],
    ) -> None:
        self.name = name
        self.description = description
        self.parameters = parameters
        self.dispatch = dispatch

    def to_definition(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "parameters": self.parameters,
        }
