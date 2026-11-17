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

"""Anthropic Messages API backend (tool use, optional streaming).

Three things here are decisions, not incidental:

**1. Temperature is not always sendable, and the row must say so.** SPEC 4.4
says "temperature pinned (0 where the provider allows)". On current Claude
models (Opus 5 / 4.8 / 4.7, Fable 5, Sonnet 5) ``temperature`` is *removed* --
sending it returns a 400. So the backend consults
:data:`SAMPLING_REJECTED_PREFIXES`, omits the parameter on those models, and
reports ``temperature=None`` rather than claiming a pin it did not make. A row
that said ``temperature: 0`` for an Opus-5 run would be false.

**2. Assistant content is echoed back verbatim.** Thinking is on by default on
Opus 5 and thinking blocks must be replayed **unchanged** when continuing on
the same model. The loop therefore appends the provider's own ``content`` list
rather than rebuilding it from text + tool calls -- see
``ModelTurn.assistant_content``.

**3. There is no offline stub.** Without a credential this class refuses to
construct (:class:`BackendUnavailable`) instead of returning canned answers,
because a stub that "works" hides exactly the design problems a real call
would expose. What *can* be exercised offline -- client construction, tool
schema shape against the installed SDK's ``ToolParam``, request-body
assembly and JSON-serializability -- is exercised by :func:`preflight_offline`,
which is honest about being a shape check and not a round trip.

Credential: ``ANTHROPIC_API_KEY`` (or ``ANTHROPIC_AUTH_TOKEN``, or an
``ant auth login`` profile). Nothing in this repo can produce one.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from agentbench.runner.backends.base import (
    PRICES_AS_OF, PRICES_USD_PER_MTOK, BackendUnavailable, ModelBackend,
    ModelTurn, ToolCall, Usage)

# Default model. Pinned rather than "latest" so a comparison cannot silently
# change models between cells (SPEC 4.4: same model for every cell).
DEFAULT_MODEL = "claude-opus-5"

# Models that reject `temperature`/`top_p`/`top_k` outright (400).
SAMPLING_REJECTED_PREFIXES = (
    "claude-opus-5", "claude-opus-4-8", "claude-opus-4-7",
    "claude-fable-5", "claude-mythos-5", "claude-sonnet-5",
)

# Non-streaming requests above ~16k max_tokens risk the SDK's HTTP timeout.
DEFAULT_MAX_TOKENS = 16000
DEFAULT_MAX_TOKENS_STREAMING = 64000


def sampling_allowed(model: str) -> bool:
    return not any(model.startswith(p) for p in SAMPLING_REJECTED_PREFIXES)


def credential_source() -> str | None:
    """Which credential the SDK would use, or ``None``.

    An unset ``ANTHROPIC_API_KEY`` does not by itself mean "no credential":
    the SDK also honours ``ANTHROPIC_AUTH_TOKEN`` and an ``ant auth login``
    profile on disk. Reporting the *source* rather than a bool keeps the
    "SKIPPED: no credential" verdict verifiable.
    """
    if os.environ.get("ANTHROPIC_API_KEY"):
        return "env:ANTHROPIC_API_KEY"
    if os.environ.get("ANTHROPIC_AUTH_TOKEN"):
        return "env:ANTHROPIC_AUTH_TOKEN"
    roots = []
    cfg = os.environ.get("ANTHROPIC_CONFIG_DIR")
    if cfg:
        roots.append(Path(cfg))
    if os.name == "nt" and os.environ.get("APPDATA"):
        roots.append(Path(os.environ["APPDATA"]) / "Anthropic")
    roots.append(Path.home() / ".config" / "anthropic")
    for r in roots:
        creds = r / "credentials"
        if creds.is_dir() and any(creds.glob("*.json")):
            return "profile:%s" % creds
    return None


class AnthropicBackend(ModelBackend):
    """One ``turn()`` == one Messages API call (plus SDK-internal retries)."""

    kind = "anthropic"

    def __init__(self, *, model=None, temperature=0.0, max_tokens=None,
                 stream=False, max_retries=2, timeout_s=600.0,
                 thinking=None, effort=None, extra_params=None,
                 require_credential=True):
        try:
            import anthropic
        except ImportError as exc:                       # pragma: no cover
            raise BackendUnavailable(
                "the `anthropic` SDK is not installed: %s" % exc) from exc
        self._anthropic = anthropic
        self.model = model or DEFAULT_MODEL
        self.stream = bool(stream)
        self.max_tokens = int(max_tokens or (DEFAULT_MAX_TOKENS_STREAMING
                                             if self.stream
                                             else DEFAULT_MAX_TOKENS))
        self.thinking = thinking          # None | "adaptive" | "disabled"
        self.effort = effort              # None | low|medium|high|xhigh|max
        self.extra_params = dict(extra_params or {})
        self.credential = credential_source()
        if require_credential and self.credential is None:
            raise BackendUnavailable(
                "no Anthropic credential found. Set ANTHROPIC_API_KEY (or "
                "ANTHROPIC_AUTH_TOKEN, or run `ant auth login`) and re-run. "
                "This repository cannot produce one.")
        # Sampling params are removed on current models; claiming a pin we
        # did not send would put a false `temperature` in the row.
        if temperature is None or not sampling_allowed(self.model):
            self.temperature = None
        else:
            self.temperature = float(temperature)
        self.client = anthropic.Anthropic(max_retries=int(max_retries),
                                          timeout=float(timeout_s))

    # -- provenance ------------------------------------------------------
    def describe(self):
        d = super().describe()
        d.update({
            "max_tokens": self.max_tokens, "stream": self.stream,
            "credential_source": self.credential,
            "sampling_allowed": sampling_allowed(self.model),
            "thinking": self.thinking, "effort": self.effort,
            "sdk_version": getattr(self._anthropic, "__version__", None),
            "prices_as_of": PRICES_AS_OF,
        })
        return d

    def price_table(self):
        return PRICES_AS_OF, PRICES_USD_PER_MTOK

    # -- request assembly (separated so it can be tested offline) --------
    def build_kwargs(self, system, messages, tools) -> dict:
        kw = {"model": self.model, "max_tokens": self.max_tokens,
              "system": system, "messages": messages}
        if tools:
            kw["tools"] = tools
        if self.temperature is not None:
            kw["temperature"] = self.temperature
        if self.thinking == "adaptive":
            kw["thinking"] = {"type": "adaptive"}
        elif self.thinking == "disabled":
            kw["thinking"] = {"type": "disabled"}
        if self.effort:
            kw["output_config"] = {"effort": self.effort}
        kw.update(self.extra_params)
        return kw

    # -- the contract ----------------------------------------------------
    def turn(self, system, messages, tools) -> ModelTurn:
        kw = self.build_kwargs(system, messages, tools)
        if self.stream:
            with self.client.messages.stream(**kw) as s:
                resp = s.get_final_message()
        else:
            resp = self.client.messages.create(**kw)
        return self._to_turn(resp)

    def _to_turn(self, resp) -> ModelTurn:
        text_parts, calls = [], []
        for block in resp.content:
            btype = getattr(block, "type", None)
            if btype == "text":
                text_parts.append(block.text)
            elif btype == "tool_use":
                calls.append(ToolCall(id=block.id, name=block.name,
                                      arguments=dict(block.input or {})))
        u = getattr(resp, "usage", None)
        usage = Usage(
            tokens_in=getattr(u, "input_tokens", None),
            tokens_out=getattr(u, "output_tokens", None),
            tokens_cache_read=getattr(u, "cache_read_input_tokens", None),
            tokens_cache_write=getattr(u, "cache_creation_input_tokens", None))
        details = getattr(resp, "stop_details", None)
        return ModelTurn(
            text="\n".join(p for p in text_parts if p).strip(),
            tool_calls=calls, usage=usage,
            stop_reason=getattr(resp, "stop_reason", None),
            # Verbatim: thinking blocks must replay unchanged on this model.
            assistant_content=resp.content,
            raw={"id": getattr(resp, "id", None),
                 "model": getattr(resp, "model", None),
                 "stop_reason": getattr(resp, "stop_reason", None),
                 "stop_details": (
                     {"category": getattr(details, "category", None),
                      "explanation": getattr(details, "explanation", None)}
                     if details is not None else None),
                 "n_blocks": len(resp.content)})


# --------------------------------------------------------------------------
# Offline exercise: everything about this backend that does NOT need a key.
# --------------------------------------------------------------------------

def _typed_dict_keys(td):
    req = set(getattr(td, "__required_keys__", set()) or set())
    opt = set(getattr(td, "__optional_keys__", set()) or set())
    return req, opt


def preflight_offline(tool_definitions, *, model=DEFAULT_MODEL,
                      system="preflight", messages=None) -> dict:
    """Validate the request we *would* send, without sending it.

    What this genuinely proves:

    * the SDK imports and ``anthropic.Anthropic()`` constructs (it does so
      with ``api_key=None`` -- the failure is deferred to request time, which
      is why the backend gates on :func:`credential_source` itself);
    * every tool definition's keys are a subset of the installed SDK's
      ``ToolParam`` fields, and ``name``/``input_schema`` are present -- this
      catches an OpenAI-shaped ``parameters`` key, a missing schema, or a typo
      that would otherwise surface as a 400 on the first paid call;
    * each ``input_schema`` is an object schema with ``properties``;
    * message params match ``MessageParam``;
    * the assembled body is JSON-serializable (no stray Path/set/bytes).

    What it does **not** prove: that the API accepts the request. Anything
    that requires a round trip -- real ``usage`` numbers, ``stop_reason``
    handling, tool-use id round-tripping, streaming event assembly, refusal
    and ``pause_turn`` paths, rate-limit retries -- is untested until a
    credential exists.
    """
    import anthropic
    from anthropic.types import MessageParam, ToolParam

    report = {"sdk_version": getattr(anthropic, "__version__", None),
              "model": model, "checks": {}, "errors": [],
              "credential_source": credential_source()}

    def check(name, ok, detail=""):
        report["checks"][name] = {"ok": bool(ok), "detail": detail}
        if not ok:
            report["errors"].append("%s: %s" % (name, detail))
        return ok

    try:
        client = anthropic.Anthropic(api_key=os.environ.get(
            "ANTHROPIC_API_KEY") or "preflight-no-key")
        check("client_constructs", True, type(client).__name__)
    except Exception as exc:                             # noqa: BLE001
        check("client_constructs", False, repr(exc))
        return report

    tool_req, tool_opt = _typed_dict_keys(ToolParam)
    allowed = tool_req | tool_opt
    check("sdk_exposes_ToolParam_fields", bool(allowed),
          "fields=%s" % sorted(allowed))
    bad = []
    for t in tool_definitions:
        extra = set(t) - allowed
        if extra:
            bad.append("%s: unknown field(s) %s" % (t.get("name"),
                                                    sorted(extra)))
        for must in ("name", "input_schema"):
            if must not in t:
                bad.append("%s: missing %r" % (t.get("name"), must))
        schema = t.get("input_schema") or {}
        if schema.get("type") != "object":
            bad.append("%s: input_schema.type must be 'object'"
                       % t.get("name"))
        if "properties" not in schema:
            bad.append("%s: input_schema has no 'properties'" % t.get("name"))
    check("tool_schemas_match_sdk_ToolParam", not bad, "; ".join(bad))

    msgs = messages or [{"role": "user", "content": "preflight"}]
    mreq, mopt = _typed_dict_keys(MessageParam)
    mallowed = mreq | mopt
    mbad = [str(sorted(set(m) - mallowed)) for m in msgs
            if set(m) - mallowed]
    check("message_params_match_sdk_MessageParam", not mbad, "; ".join(mbad))

    backend = AnthropicBackend.__new__(AnthropicBackend)
    backend.model = model
    backend.max_tokens = DEFAULT_MAX_TOKENS
    backend.temperature = 0.0 if sampling_allowed(model) else None
    backend.thinking = None
    backend.effort = None
    backend.extra_params = {}
    kw = backend.build_kwargs(system, msgs, list(tool_definitions))
    check("temperature_policy",
          ("temperature" in kw) == sampling_allowed(model),
          "sampling_allowed=%s temperature_sent=%s"
          % (sampling_allowed(model), "temperature" in kw))
    try:
        blob = json.dumps(kw)
        check("request_body_json_serializable", True, "%d bytes" % len(blob))
    except (TypeError, ValueError) as exc:
        check("request_body_json_serializable", False, repr(exc))

    report["request_keys"] = sorted(kw)
    report["untested_until_credential"] = [
        "HTTP round trip / authentication",
        "response parsing of a real Message (stop_reason, content blocks)",
        "tool_use -> tool_result id round-trip accepted by the API",
        "real token usage numbers and therefore real usd",
        "streaming event assembly (client.messages.stream)",
        "refusal / pause_turn / max_tokens stop-reason paths",
        "rate-limit and 5xx retry behaviour",
    ]
    report["ok"] = not report["errors"]
    return report
