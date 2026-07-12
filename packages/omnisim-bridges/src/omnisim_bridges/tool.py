# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

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
