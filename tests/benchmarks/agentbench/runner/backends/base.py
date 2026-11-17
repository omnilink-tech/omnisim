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

"""The model-backend abstraction: one method, three value types.

    backend.turn(system, messages, tools) -> ModelTurn

That is the entire contract the loop depends on, which is what makes adding an
OpenAI-compatible or OmniLink backend a new file rather than a loop edit. The
*canonical message format is Anthropic's* -- content-block lists with
``text`` / ``tool_use`` / ``tool_result`` blocks -- because that is the
primary backend and translating in the loop would mean the loop knows about
providers. A non-Anthropic backend adapts inside its own module.

Cost (SPEC 3.1): ``usd`` is "cost at the model's list price **on the run
date**, pinned in the row". So prices live in a table with an ``as_of`` date,
and a model with no price entry yields ``None`` -- never ``0.0``, and never a
guess.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# List prices, USD per million tokens, as of the date below. Pinned into every
# row that computes a cost, so a later price change cannot silently rewrite a
# published number. A model absent from this table gets usd=None.
PRICES_AS_OF = "2026-06-24"
PRICES_USD_PER_MTOK = {
    # model id: (input, output, cache_read)
    "claude-opus-5":     (5.00, 25.00, 0.50),
    "claude-opus-4-8":   (5.00, 25.00, 0.50),
    "claude-opus-4-7":   (5.00, 25.00, 0.50),
    "claude-opus-4-6":   (5.00, 25.00, 0.50),
    "claude-fable-5":    (10.00, 50.00, 1.00),
    "claude-sonnet-5":   (3.00, 15.00, 0.30),
    "claude-sonnet-4-6": (3.00, 15.00, 0.30),
    "claude-haiku-4-5":  (1.00, 5.00, 0.10),
}


def usd_for(model, tokens_in, tokens_out, tokens_cache_read=None):
    """List-price cost, or ``None`` when the price is not pinned here.

    Returning ``None`` for an unknown model is deliberate: a benchmark that
    reports ``0.0`` for a cost it did not measure is lying in a way that
    happens to flatter whichever cell it happens to.
    """
    price = PRICES_USD_PER_MTOK.get(model)
    if price is None or tokens_in is None or tokens_out is None:
        return None
    pin, pout, pcache = price
    total = (tokens_in / 1e6) * pin + (tokens_out / 1e6) * pout
    if tokens_cache_read:
        total += (tokens_cache_read / 1e6) * pcache
    return round(total, 6)


class BackendExhausted(Exception):
    """A replay backend ran out of scripted turns while the loop wanted more.

    Distinct from a model that stopped: the script ended mid-conversation, so
    the run has no honest final message. Recorded as ``script_exhausted``.
    """


class BackendUnavailable(Exception):
    """The backend cannot run at all -- missing credential, missing SDK.

    Raised at construction, never mid-loop, so a run that cannot start is
    distinguishable from a run that started and failed.
    """


@dataclass
class Usage:
    """Token accounting for one model call. ``None`` means *not reported*."""

    tokens_in: int | None = None
    tokens_out: int | None = None
    tokens_cache_read: int | None = None
    tokens_cache_write: int | None = None

    def as_dict(self):
        return {"tokens_in": self.tokens_in, "tokens_out": self.tokens_out,
                "tokens_cache_read": self.tokens_cache_read,
                "tokens_cache_write": self.tokens_cache_write}


@dataclass
class ToolCall:
    """One tool invocation the model asked for."""

    id: str
    name: str
    arguments: dict = field(default_factory=dict)

    def as_dict(self):
        return {"id": self.id, "name": self.name, "arguments": self.arguments}


@dataclass
class ModelTurn:
    """One model response.

    ``assistant_content`` is the provider-native content list to append to the
    conversation verbatim. This matters on current Claude models: thinking
    blocks must be echoed back **unchanged** when continuing on the same
    model, so the loop appends what the provider returned rather than
    reconstructing it from ``text`` + ``tool_calls``.
    """

    text: str = ""
    tool_calls: list = field(default_factory=list)
    usage: Usage | None = None
    stop_reason: str | None = None       # provider's own stop_reason
    assistant_content: object = None     # provider-native, echoed verbatim
    raw: dict | None = None              # small provider metadata, traced

    @property
    def wants_tools(self) -> bool:
        return bool(self.tool_calls)


class ModelBackend:
    """Interface every backend satisfies. Subclasses override ``turn``."""

    #: short identifier recorded in the row ("anthropic", "scripted", ...)
    kind = "abstract"
    #: model identifier recorded in the row
    model = "unknown"
    #: temperature actually sent, or None when the provider rejects it
    temperature: float | None = None

    def turn(self, system: str, messages: list, tools: list) -> ModelTurn:
        raise NotImplementedError

    def price_table(self):
        """``(as_of, {model: (in, out, cache_read)})`` or ``None``.

        A backend with no list price (the scripted replay) returns ``None``,
        which propagates to ``usd = None`` in the row.
        """
        return None

    def usd(self, tokens_in, tokens_out, tokens_cache_read=None):
        if self.price_table() is None:
            return None
        return usd_for(self.model, tokens_in, tokens_out, tokens_cache_read)

    def describe(self) -> dict:
        """Provenance for the result row's ``agent`` block."""
        return {"backend": self.kind, "model": self.model,
                "temperature": self.temperature}

    def close(self):
        pass
