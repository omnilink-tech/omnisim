#!/usr/bin/env python3
"""Mocked end-to-end test for the OmniLink relay.

Verifies the multi-turn chat-with-tools loop without hitting the live
OmniLink service. The relay calls OmniLinkClient.chat() — we swap the
client instance on the relay for a scripted fake that pops responses
from a list, so the test exercises the exact same dispatch + history +
event-emission code paths as a live run.

Run from the repo root:

    python tests/test_omnilink_relay.py
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "projects" / "samples" / "demos" / "controllers"))

from _omnilink_relay.omnilink_relay import (  # noqa: E402
    OmniLinkRelay,
    Tool,
    _from_memory_format,
    _to_memory_format,
)
from omnilink.client import OmniLinkAPIError  # noqa: E402


class FakeOmniLinkClient:
    """Scripted stand-in for OmniLinkClient.

    Each call to .chat(...) returns the next entry from ``responses``.
    If the entry is an exception instance, it gets raised — this lets a
    single test cover both successful HTTP and OmniLinkAPIError paths.
    """

    def __init__(self, responses):
        self._responses = list(responses)
        self.calls = 0

    def chat(self, **kwargs):
        i = self.calls
        self.calls += 1
        item = self._responses[i]
        if isinstance(item, Exception):
            raise item
        return item


def _make_relay(tools, responses):
    relay = OmniLinkRelay(
        omni_key="olink_test",
        agent_name="TestBot",
        main_task="test",
        tools=tools,
        memory_enabled=False,
        usage_enabled=False,
    )
    # Replace the live client with the scripted fake AFTER construction —
    # construction still requires a real OmniLinkClient to be importable
    # (matches the production guard), but the dispatch path uses
    # self._client.chat which we now own.
    relay._client = FakeOmniLinkClient(responses)
    return relay


def test_one_turn_tool_then_text() -> None:
    """LLM returns one tool call, then a final text."""
    dispatch_log = []

    def dispatch(args):
        dispatch_log.append(args)
        return {"accepted": True, "distance": args["distance"], "eta_s": 1.5}

    tool = Tool(
        name="drive_forward",
        description="drive forward N metres",
        parameters={"type": "object", "properties": {"distance": {"type": "number"}},
                    "required": ["distance"]},
        dispatch=dispatch,
    )

    relay = _make_relay([tool], [
        {"ok": True, "text": "", "toolCalls": [
            {"id": "c1", "name": "drive_forward", "arguments": {"distance": 1.5}},
        ]},
        {"ok": True, "text": "Driving forward 1.5 metres.", "toolCalls": []},
    ])

    events = []
    done = threading.Event()

    def on_event(kind, payload):
        events.append((kind, payload))
        if kind == "status" and payload.get("state") == "idle":
            done.set()

    relay.dispatch_async("drive forward 1.5 meters", on_event)
    assert done.wait(timeout=10), "relay did not finish in time"
    chat_calls = relay._client.calls
    relay.close()

    kinds = [k for k, _ in events]
    assert kinds == ["status", "tool", "agent", "status"], f"unexpected event sequence: {kinds}"
    assert dispatch_log == [{"distance": 1.5}], f"tool was not dispatched correctly: {dispatch_log}"
    assert chat_calls == 2, f"expected 2 chat calls, got {chat_calls}"
    print("[test_one_turn_tool_then_text] PASS")


def test_unknown_tool_surfaced_as_error() -> None:
    """If the LLM hallucinates a tool name, the relay reports it cleanly."""
    tool = Tool(
        name="drive_forward",
        description="drive forward",
        parameters={"type": "object", "properties": {}},
        dispatch=lambda a: {"ok": True},
    )

    relay = _make_relay([tool], [
        {"ok": True, "text": "", "toolCalls": [
            {"id": "c1", "name": "fly_to_moon", "arguments": {}},
        ]},
        {"ok": True, "text": "Sorry, I can't do that.", "toolCalls": []},
    ])

    events = []
    done = threading.Event()

    def on_event(kind, payload):
        events.append((kind, payload))
        if kind == "status" and payload.get("state") == "idle":
            done.set()

    relay.dispatch_async("fly to the moon", on_event)
    assert done.wait(timeout=10)
    relay.close()

    tool_events = [p for k, p in events if k == "tool"]
    assert len(tool_events) == 1 and tool_events[0]["status"] == "err", \
        f"unknown tool should be reported as err: {tool_events}"
    print("[test_unknown_tool_surfaced_as_error] PASS")


def test_http_error_surfaced_as_error() -> None:
    """OmniLinkAPIError (HTTP non-2xx) is mapped onto the relay's
    ``error`` event so the side menu can surface it."""
    err = OmniLinkAPIError(429, {"error": "rate limited"})

    tool = Tool("noop", "n", {"type": "object", "properties": {}}, lambda a: {"ok": True})

    relay = _make_relay([tool], [err])

    events = []
    done = threading.Event()

    def on_event(kind, payload):
        events.append((kind, payload))
        if kind == "error":
            done.set()

    relay.dispatch_async("anything", on_event)
    assert done.wait(timeout=10)
    relay.close()
    err_events = [p for k, p in events if k == "error"]
    assert err_events and "429" in str(err_events[0]["text"]), f"missing 429 surface: {err_events}"
    print("[test_http_error_surfaced_as_error] PASS")


def test_usage_event_emitted_after_agent_text() -> None:
    """When usage_enabled=True the relay must emit one `usage` event per
    turn, between the `agent` event and the idle `status`. The payload's
    one-line ``text`` is what the side menu renders."""
    tool = Tool("noop", "n", {"type": "object", "properties": {}}, lambda a: {"ok": True})

    relay = OmniLinkRelay(
        omni_key="olink_test",
        agent_name="TestBot",
        main_task="test",
        tools=[tool],
        memory_enabled=False,
        usage_enabled=True,
    )
    # Swap in the scripted chat fake + a fake usage source. UsageMeter
    # calls self._client.get_usage(limit=1); we feed it two snapshots
    # so the post-turn delta surfaces nonzero tokens.
    class FakeClientWithUsage(FakeOmniLinkClient):
        def __init__(self, responses):
            super().__init__(responses)
            self.usage_calls = 0
        def get_usage(self, **_kw):
            i = self.usage_calls
            self.usage_calls += 1
            # First call (at start()) baseline. Second call (snapshot)
            # shows +1000 input, +200 output, +0.05 credits.
            if i == 0:
                return {"rollup": {"input_units_24h": 10000, "output_units_24h": 500,
                                   "cached_input_units_24h": 0, "credits_24h": 1.0,
                                   "last_occurred_at": None}}
            return {"rollup": {"input_units_24h": 11000, "output_units_24h": 700,
                               "cached_input_units_24h": 100, "credits_24h": 1.05,
                               "last_occurred_at": None}}

    relay._client = FakeClientWithUsage([
        {"ok": True, "text": "done.", "toolCalls": []},
    ])
    # Re-baseline the meter against the new fake client.
    from omnilink.usage_meter import UsageMeter
    relay._meter = UsageMeter(relay._client)
    relay._meter.start()

    events = []
    done = threading.Event()

    def on_event(kind, payload):
        events.append((kind, payload))
        if kind == "status" and payload.get("state") == "idle":
            done.set()

    relay.dispatch_async("hi", on_event)
    assert done.wait(timeout=10)
    relay.close()

    kinds = [k for k, _ in events]
    assert kinds == ["status", "agent", "usage", "status"], f"unexpected sequence: {kinds}"
    usage_payload = [p for k, p in events if k == "usage"][0]
    assert usage_payload["input_units"] == 1000, usage_payload
    assert usage_payload["output_units"] == 200, usage_payload
    assert usage_payload["total_units"] == 1200, usage_payload
    # text is one-line summary -- exact contents depend on elapsed
    # window so we just assert it is non-empty.
    assert usage_payload["text"], usage_payload
    print("[test_usage_event_emitted_after_agent_text] PASS")


def test_voice_methods() -> None:
    """The relay's transcribe() and synthesize() are thin pass-throughs
    to OmniLinkClient. Verify they propagate text out / bytes back, and
    return safe defaults on errors."""
    class FakeVoiceClient(FakeOmniLinkClient):
        def __init__(self):
            super().__init__([])
        def transcribe(self, audio, **_kw):
            assert isinstance(audio, bytes) and audio == b"webm-bytes"
            return {"text": "go home now"}
        def synthesize_to_bytes(self, text, **_kw):
            assert text == "Heading home."
            return b"mp3-bytes"

    tool = Tool("noop", "n", {"type": "object", "properties": {}}, lambda a: {"ok": True})
    relay = OmniLinkRelay(
        omni_key="olink_test",
        agent_name="VoiceBot",
        main_task="test",
        tools=[tool],
        memory_enabled=False,
        usage_enabled=False,
    )
    relay._client = FakeVoiceClient()

    text = relay.transcribe(b"webm-bytes", mime_type="audio/webm")
    assert text == "go home now", text

    audio = relay.synthesize("Heading home.")
    assert audio == b"mp3-bytes", audio

    # Empty-text synthesis short-circuits to None.
    assert relay.synthesize("") is None
    relay.close()
    print("[test_voice_methods] PASS")


def test_memory_format_roundtrip() -> None:
    """Internal OpenAI-shape ↔ Gemini-shape memory format round-trips
    user-visible text and drops transient tool scaffolding."""
    internal = [
        {"role": "user", "content": "drive forward 1 m"},
        {"role": "assistant", "content": "",
         "tool_calls": [{"id": "c1", "name": "drive_forward",
                          "arguments": {"distance": 1.0}}]},
        {"role": "tool", "tool_call_id": "c1", "name": "drive_forward",
         "content": '{"accepted": true}'},
        {"role": "assistant", "content": "Driving forward 1 metre."},
        {"role": "user", "content": "now go back"},
    ]

    memory = _to_memory_format(internal)
    # Tool scaffolding (assistant with empty content + tool_calls, and
    # role:"tool" results) must be dropped — they aren't human-readable.
    assert memory == [
        {"role": "user",  "parts": [{"text": "drive forward 1 m"}]},
        {"role": "model", "parts": [{"text": "Driving forward 1 metre."}]},
        {"role": "user",  "parts": [{"text": "now go back"}]},
    ], f"to_memory dropped wrong rows: {memory}"

    restored = _from_memory_format(memory)
    # Restore keeps user/assistant text only.
    assert restored == [
        {"role": "user", "content": "drive forward 1 m"},
        {"role": "assistant", "content": "Driving forward 1 metre."},
        {"role": "user", "content": "now go back"},
    ], f"from_memory unexpected shape: {restored}"
    print("[test_memory_format_roundtrip] PASS")


if __name__ == "__main__":
    test_one_turn_tool_then_text()
    test_unknown_tool_surfaced_as_error()
    test_http_error_surfaced_as_error()
    test_usage_event_emitted_after_agent_text()
    test_voice_methods()
    test_memory_format_roundtrip()
    print("\nAll OmniLink-relay tests passed.")
