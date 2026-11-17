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

"""The `/prompt` envelope the MODEL path returns (PROTOCOL §5.7.2).

Three defects, all of them things a machine client cannot work around:

1. **No `via`.** §5.7.1 puts the stage that answered at the top level of
   the 200 body. `route.py` sets `"parser"` on its short-circuit and the
   relay path set nothing, so a model-answered turn reported no stage at
   all -- and the chat sweep, which refuses to INFER the column it exists
   to measure, records `null` and fails the comparison. The plan's central
   claim ("both doors are parser-first") is unmeasurable without this.

2. **Two spellings for a refusal.** The parser path said
   `result: "refused"`; this path said `result: "err"` with the rule
   behind a `"refused: "` prefix in prose. A client had to parse a
   sentence to tell "declined on purpose, do not retry" from "failed,
   retry".

3. **No top-level `error` on a refusal.** Measured: a two-hour endurance
   run lost ~5% of its orders to refusals visible only in `response`
   prose, logged zero errors from start to finish, and the robot drove off
   the floor.

Everything here runs offline: `_post_chat` is scripted, so the real
`_dispatch_one` runs against a fake model and a real gate.
"""

from __future__ import annotations

import pytest

from omnisim_bridges.bridge_base import GATE_RULES
from omnisim_bridges.relay import (
    DispatchHandle,
    OmniLinkRelay,
    _action_error_line,
    _refusal_rule,
    _result_refused,
)
from omnisim_bridges.tool import Tool

NUM = {"type": "object", "properties": {"distance": {"type": "number"}}}


def build(monkeypatch, tmp_path, dispatch, rounds):
    """A relay whose model says `rounds` and whose drive tool is `dispatch`."""
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="EnvelopeProbe", main_task="test",
        tools=[Tool("drive_forward", "drive a distance", NUM, dispatch)],
        usage_enabled=False, memory_enabled=False, surface="mobile")
    scripted = list(rounds)

    def _fake_chat(messages):
        return scripted.pop(0) if scripted else {"text": "done"}

    relay._post_chat = _fake_chat             # type: ignore[assignment]
    return relay


def call(name, args, text="acknowledged"):
    return [{"text": text,
             "toolCalls": [{"id": "1", "name": name, "arguments": args}]},
            {"text": "finished"}]


# ── via ──────────────────────────────────────────────────────────────

def test_via_is_reported_on_every_relay_answer(monkeypatch, tmp_path) -> None:
    relay = build(monkeypatch, tmp_path,
                  lambda a: {"commanded": 1.0, "achieved": 1.0, "error": 0.0},
                  call("drive_forward", {"distance": 1.0}))
    try:
        out = relay.dispatch_sync("drive forward one metre", timeout_s=20)
    finally:
        relay.close()
    assert out["via"] == "relay", (
        "a reply with no `via` is recorded as null by the sweep, and null "
        "cannot agree with anything -- the two-door comparison then fails")


def test_via_is_reported_even_with_no_tool_calls(monkeypatch, tmp_path) -> None:
    relay = build(monkeypatch, tmp_path, lambda a: {},
                  [{"text": "I am in the north aisle."}])
    try:
        out = relay.dispatch_sync("say where you are", timeout_s=20)
    finally:
        relay.close()
    assert out["via"] == "relay"
    assert out["response"] == "I am in the north aisle."


def test_via_is_one_of_exactly_two_values() -> None:
    """`route.py` owns "parser"; this class owns "relay". Anything else
    would be a third stage, and §5.7.1 forbids describing one."""
    assert OmniLinkRelay.VIA in ("parser", "relay")


def test_the_timeout_envelope_still_says_which_stage_had_it(
        monkeypatch, tmp_path) -> None:
    relay = build(monkeypatch, tmp_path, lambda a: {}, [])
    try:
        # A turn that never completes: the handle is returned and nothing
        # ever fires the "idle" event the sync wrapper waits on.
        relay.dispatch_async = lambda t, cb: DispatchHandle()
        out = relay.dispatch_sync("drive forward", timeout_s=0.2)
    finally:
        relay.close()
    assert out["via"] == "relay"
    assert out["error"] == "timeout"


# ── Refusals ─────────────────────────────────────────────────────────

def test_a_gate_refusal_is_its_own_result_with_a_rule(monkeypatch,
                                                      tmp_path) -> None:
    """300 m is past the 50 m rail. The frame must come back as REFUSED,
    with the rule as a field a program can branch on."""
    dispatched = []
    relay = build(monkeypatch, tmp_path, lambda a: dispatched.append(a),
                  call("drive_forward", {"distance": 300.0}))
    try:
        out = relay.dispatch_sync("drive forward 300 metres", timeout_s=20)
    finally:
        relay.close()

    assert dispatched == [], "a refused frame must never reach the tool"
    entry = out["actions"][0]
    assert entry["result"] == "refused", "not 'err' -- a refusal did not fail"
    assert entry["rule"] == "implausible"
    assert entry["rule"] in GATE_RULES, (
        "a gate refusal's rule must come from the §5.8.3 enumeration")
    assert out["error"] == "drive_forward: implausible", (
        "§5.7.2: the top-level error MUST be set on a refusal, and it names "
        "the rule, not the prose")


def test_the_rule_is_a_field_not_a_prefix_in_prose(monkeypatch,
                                                   tmp_path) -> None:
    relay = build(monkeypatch, tmp_path, lambda a: {},
                  call("drive_forward", {"distance": 300.0}))
    try:
        out = relay.dispatch_sync("drive forward 300 metres", timeout_s=20)
    finally:
        relay.close()
    entry = out["actions"][0]
    assert isinstance(entry.get("rule"), str) and entry["rule"]
    # The prose stays prose, and MAY change between releases -- which is
    # exactly why a program must not have to read it.
    assert isinstance(entry["summary"], str)


def test_a_bridge_refusal_is_also_a_refusal(monkeypatch, tmp_path) -> None:
    """`accepted: False` is the bridge's own refusal marker (§5.4.1 rule 5):
    an out-of-bounds target declined, nothing actuated. A caller must not
    retry that either."""
    relay = build(monkeypatch, tmp_path,
                  lambda a: {"accepted": False, "reason": "outside the arena"},
                  call("drive_forward", {"distance": 2.0}))
    try:
        out = relay.dispatch_sync("drive forward two metres", timeout_s=20)
    finally:
        relay.close()
    entry = out["actions"][0]
    assert entry["result"] == "refused"
    assert entry["rule"] == "bridge_refused", (
        "a bridge refusal must not borrow a GATE rule name -- that would say "
        "a sentence was judged when it was not")
    assert out["error"].startswith("drive_forward: ")


def test_a_bridge_that_names_its_own_rule_keeps_it(monkeypatch,
                                                   tmp_path) -> None:
    relay = build(monkeypatch, tmp_path,
                  lambda a: {"accepted": False, "rule": "out_of_bounds"},
                  call("drive_forward", {"distance": 2.0}))
    try:
        out = relay.dispatch_sync("drive forward two metres", timeout_s=20)
    finally:
        relay.close()
    assert out["actions"][0]["rule"] == "out_of_bounds"


# ── Failures are still failures ──────────────────────────────────────

def test_a_real_failure_is_still_err_and_still_sets_error(monkeypatch,
                                                          tmp_path) -> None:
    """Out of scope to rename: §5.7.2 requires clients to accept both `err`
    and `error`, and changing the success/failure spellings would churn
    every consumer for nothing."""
    relay = build(monkeypatch, tmp_path,
                  lambda a: {"error": "unreachable_target"},
                  call("drive_forward", {"distance": 2.0}))
    try:
        out = relay.dispatch_sync("drive forward two metres", timeout_s=20)
    finally:
        relay.close()
    entry = out["actions"][0]
    assert entry["result"] == "err"
    assert "rule" not in entry, "a failure has no gate rule"
    assert out["error"].startswith("drive_forward: ")


def test_an_unknown_tool_is_an_error_not_a_refusal(monkeypatch,
                                                   tmp_path) -> None:
    relay = build(monkeypatch, tmp_path, lambda a: {},
                  call("teleport", {}))
    try:
        out = relay.dispatch_sync("teleport", timeout_s=20)
    finally:
        relay.close()
    assert out["actions"][0]["result"] == "err"


# ── The float trap ───────────────────────────────────────────────────

def test_a_perfect_motion_sets_no_top_level_error(monkeypatch,
                                                  tmp_path) -> None:
    """⚠️ `error` in a RESULT is the control error, a float: a 2 m drive
    that lands at 1.9977 carries -0.00228. If the top-level `error` were
    driven by truthiness on that key, every successful prompt would report
    a failure -- the same trap that once printed "I could not stop: 4.3e-11"
    after a perfect stop."""
    relay = build(monkeypatch, tmp_path,
                  lambda a: {"verb": "drive", "commanded": 2.0,
                             "achieved": 1.9977,
                             "error": -0.0022885799407958984,
                             "settled": True},
                  call("drive_forward", {"distance": 2.0}))
    try:
        out = relay.dispatch_sync("drive forward two metres", timeout_s=20)
    finally:
        relay.close()
    assert out["actions"][0]["result"] == "ok"
    assert "error" not in out, f"a settled 2 m drive reported {out.get('error')!r}"


def test_the_error_line_is_built_from_result_never_from_a_residual() -> None:
    assert _action_error_line([
        {"tool": "drive_forward", "result": "ok",
         "summary": "drive, commanded=2.0, achieved=1.9977, residual=-0.0022"},
    ]) == ""
    assert _action_error_line([
        {"tool": "turn", "result": "refused", "rule": "interrogative",
         "summary": "the utterance asks a question"},
    ]) == "turn: interrogative"


def test_a_mixed_turn_reports_the_refusal_and_not_the_success() -> None:
    line = _action_error_line([
        {"tool": "get_robot_state", "result": "ok", "summary": "pose=..."},
        {"tool": "drive_forward", "result": "refused", "rule": "implausible",
         "summary": "300 m is past the rail"},
    ])
    assert line == "drive_forward: implausible"


def test_the_error_line_is_bounded() -> None:
    line = _action_error_line([
        {"tool": f"t{i}", "result": "err", "summary": "x" * 400}
        for i in range(20)])
    assert len(line) < 800


# ── The two helpers ──────────────────────────────────────────────────

@pytest.mark.parametrize("reason,rule", [
    ("interrogative: the utterance asks a question", "interrogative"),
    ("implausible: 300 m is past the 50 m rail", "implausible"),
    ("unresolved_referent: 'it' was never resolved", "unresolved_referent"),
])
def test_the_rule_is_read_from_the_gates_own_format(reason, rule) -> None:
    assert _refusal_rule(reason) == rule


@pytest.mark.parametrize("reason", [
    "safety gate unavailable (ImportError)",
    "safety gate errored (RuntimeError)",
    "",
    None,
    "Something Capitalised: with a colon",
])
def test_an_unparseable_refusal_never_invents_a_gate_rule(reason) -> None:
    """The fail-closed wrapper refuses a physical tool when the gate will
    not import at all. That is a real refusal with no rule name, and
    reporting it as `interrogative` would claim a sentence was judged when
    nothing was."""
    assert _refusal_rule(reason) == "gate_unavailable"


def test_result_refused_is_narrower_than_result_failed() -> None:
    assert _result_refused({"accepted": False}) is True
    assert _result_refused({"error": "dispatch failed: boom"}) is False
    assert _result_refused({"error": 4.3e-11}) is False
    assert _result_refused(None) is False
