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

"""Everything about the Anthropic backend that can be tested with no key.

Which is more than it sounds: the client constructs, the request body is
assembled here rather than inside the SDK call, and the tool schemas are checked
against the *installed* SDK's ``ToolParam`` fields. What is left untested is
listed by :func:`preflight_offline` itself under
``untested_until_credential`` -- and the last test in this file asserts that
list is non-empty, so nobody can quietly claim full coverage.
"""

from __future__ import annotations

import pytest

from agentbench.runner.backends import BackendUnavailable, get_backend
from agentbench.runner.backends import anthropic_api as ab


def test_missing_credential_refuses_to_construct(monkeypatch):
    monkeypatch.setattr(ab, "credential_source", lambda: None)
    with pytest.raises(BackendUnavailable) as exc:
        get_backend("anthropic")
    # The message has to name the exact variable: "no credential" without
    # saying which one is a support ticket, not an error.
    assert "ANTHROPIC_API_KEY" in str(exc.value)


def test_credential_source_reports_which_source(monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.delenv("ANTHROPIC_AUTH_TOKEN", raising=False)
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-test")
    assert ab.credential_source() == "env:ANTHROPIC_API_KEY"
    monkeypatch.delenv("ANTHROPIC_API_KEY")
    monkeypatch.setenv("ANTHROPIC_AUTH_TOKEN", "tok")
    assert ab.credential_source() == "env:ANTHROPIC_AUTH_TOKEN"


def test_constructs_without_a_key_when_explicitly_allowed():
    b = ab.AnthropicBackend(require_credential=False)
    assert b.kind == "anthropic"
    assert b.model == ab.DEFAULT_MODEL
    assert b.describe()["sdk_version"]


@pytest.mark.parametrize("model,allowed", [
    ("claude-opus-5", False),
    ("claude-opus-4-8", False),
    ("claude-sonnet-5", False),
    ("claude-fable-5", False),
    ("claude-sonnet-4-6", True),
    ("claude-haiku-4-5", True),
])
def test_sampling_policy_matches_the_models(model, allowed):
    assert ab.sampling_allowed(model) is allowed


def test_temperature_is_dropped_on_models_that_reject_it():
    b = ab.AnthropicBackend(model="claude-opus-5", temperature=0.0,
                            require_credential=False)
    kw = b.build_kwargs("sys", [{"role": "user", "content": "x"}], [])
    assert "temperature" not in kw
    # ...and the row must not claim a pin that was never sent.
    assert b.temperature is None
    assert b.describe()["temperature"] is None


def test_temperature_is_sent_on_models_that_accept_it():
    b = ab.AnthropicBackend(model="claude-sonnet-4-6", temperature=0.0,
                            require_credential=False)
    kw = b.build_kwargs("sys", [{"role": "user", "content": "x"}], [])
    assert kw["temperature"] == 0.0


def test_thinking_and_effort_are_optional_and_shaped_correctly():
    b = ab.AnthropicBackend(require_credential=False)
    assert "thinking" not in b.build_kwargs("s", [], [])
    b2 = ab.AnthropicBackend(thinking="adaptive", effort="high",
                             require_credential=False)
    kw = b2.build_kwargs("s", [], [])
    assert kw["thinking"] == {"type": "adaptive"}
    assert kw["output_config"] == {"effort": "high"}


def test_extra_params_are_merged_last():
    b = ab.AnthropicBackend(extra_params={"max_tokens": 123},
                            require_credential=False)
    assert b.build_kwargs("s", [], [])["max_tokens"] == 123


def test_tools_are_omitted_when_empty():
    b = ab.AnthropicBackend(require_credential=False)
    assert "tools" not in b.build_kwargs("s", [], [])


def test_price_table_is_pinned_with_a_date():
    b = ab.AnthropicBackend(model="claude-opus-5", require_credential=False)
    as_of, table = b.price_table()
    assert as_of and "claude-opus-5" in table
    assert b.usd(1_000_000, 1_000_000) == 30.0


def test_unpriced_model_yields_none_not_zero():
    b = ab.AnthropicBackend(model="claude-something-unreleased",
                            require_credential=False)
    assert b.usd(1000, 1000) is None


# -- the offline preflight -------------------------------------------------

def test_preflight_passes_for_the_shell_condition(shell_tools):
    r = ab.preflight_offline(shell_tools.as_anthropic_tools())
    assert r["ok"], r["errors"]
    assert r["checks"]["client_constructs"]["ok"]
    assert r["checks"]["tool_schemas_match_sdk_ToolParam"]["ok"]
    assert r["checks"]["request_body_json_serializable"]["ok"]


def test_preflight_passes_for_the_full_surface(full_tools):
    r = ab.preflight_offline(full_tools.as_anthropic_tools())
    assert r["ok"], r["errors"]


def test_preflight_catches_an_openai_shaped_tool():
    bad = [{"name": "t", "description": "d",
            "parameters": {"type": "object", "properties": {}}}]
    r = ab.preflight_offline(bad)
    assert not r["ok"]
    assert not r["checks"]["tool_schemas_match_sdk_ToolParam"]["ok"]


def test_preflight_catches_a_non_object_schema():
    bad = [{"name": "t", "description": "d",
            "input_schema": {"type": "string"}}]
    r = ab.preflight_offline(bad)
    assert not r["ok"]


def test_preflight_admits_what_it_cannot_test(shell_tools):
    r = ab.preflight_offline(shell_tools.as_anthropic_tools())
    remaining = r["untested_until_credential"]
    assert remaining, "a shape check must not present itself as full coverage"
    joined = " ".join(remaining).lower()
    for topic in ("round trip", "usage", "streaming", "refusal"):
        assert topic in joined
