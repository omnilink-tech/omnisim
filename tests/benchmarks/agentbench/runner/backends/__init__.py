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

"""Backend registry.

``get_backend(name, **kw)`` is the only thing the loop imports, so a new
provider is a new module plus one line here. The registry is intentionally
tiny: the loop never branches on backend identity, and a backend never learns
which task it is running.

Shipped today:

``anthropic``
    The Messages API (:mod:`.anthropic_api`). Needs ``ANTHROPIC_API_KEY``.
``scripted``
    Replays a JSON list of tool calls (:mod:`.scripted`). No credential.

To add ``openai`` or ``omnilink``: implement ``ModelBackend.turn`` translating
to and from the canonical Anthropic-shaped message list inside that module,
then register it below. Nothing in ``loop.py`` changes.
"""

from __future__ import annotations

from agentbench.runner.backends.base import (           # noqa: F401
    BackendExhausted, BackendUnavailable, ModelBackend, ModelTurn, ToolCall,
    Usage)

KNOWN = ("anthropic", "scripted")


def get_backend(name, **kwargs) -> ModelBackend:
    name = (name or "").strip().lower()
    if name == "scripted":
        from agentbench.runner.backends.scripted import ScriptedBackend
        script = kwargs.pop("script", None)
        if not script:
            raise ValueError(
                "the scripted backend needs a script: pass script=<path> "
                "(AGENTBENCH_SCRIPT=<path>)")
        return ScriptedBackend(script, kwargs.pop("variables", None),
                               model=kwargs.pop("model", None))
    if name == "anthropic":
        from agentbench.runner.backends.anthropic_api import AnthropicBackend
        kwargs.pop("variables", None)
        kwargs.pop("script", None)
        return AnthropicBackend(**kwargs)
    raise ValueError("unknown backend %r (have: %s)"
                     % (name, ", ".join(KNOWN)))


__all__ = ["BackendExhausted", "BackendUnavailable", "KNOWN", "ModelBackend",
           "ModelTurn", "ToolCall", "Usage", "get_backend"]
