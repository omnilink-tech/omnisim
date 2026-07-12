"""Smoke tests for omnisim_bridges.

Run from the OmniSim repo root:

    pip install -e packages/omnisim-bridges
    python packages/omnisim-bridges/tests/test_smoke.py
"""

from __future__ import annotations

import json
import sys
import time
import threading
import urllib.request
from pathlib import Path

# Allow running this file directly without pip-installing.
PKG_SRC = Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from omnisim_bridges import BridgeBase, Tool, IntentRouter, serve_http, __version__  # noqa: E402


def test_imports_and_version() -> None:
    assert __version__ == "0.1.0"
    assert BridgeBase is not None
    assert Tool is not None
    assert IntentRouter is not None
    print("[test_imports_and_version] PASS")


def test_tool_to_definition() -> None:
    t = Tool(
        name="foo",
        description="do foo",
        parameters={"type": "object", "properties": {"x": {"type": "number"}}, "required": ["x"]},
        dispatch=lambda args: {"got": args["x"]},
    )
    d = t.to_definition()
    assert d == {
        "name": "foo",
        "description": "do foo",
        "parameters": {"type": "object", "properties": {"x": {"type": "number"}}, "required": ["x"]},
    }
    assert t.dispatch({"x": 1.5}) == {"got": 1.5}
    print("[test_tool_to_definition] PASS")


def test_intent_router() -> None:
    r = IntentRouter()
    log = []
    r.register(r"\bhello\b", lambda m: ("greet", "hi back"))
    r.register(r"\bstop\b",  lambda m: ("stop", log.append("STOP") or "halted"))
    assert r.dispatch("say hello") == ("greet", "hi back")
    intent, _ = r.dispatch("stop now")
    assert intent == "stop" and log == ["STOP"]
    assert r.dispatch("fly to moon") == ("unknown", "fly to moon")
    assert r.dispatch("") == ("empty", None)
    print("[test_intent_router] PASS")


class _StubArm(BridgeBase):
    robot_id = "stub_arm"
    model = "Stub"
    capabilities = {"joints": 6}

    def __init__(self) -> None:
        self.calls = []
        self.q = [0.0] * 6

    def act_stop(self):
        self.calls.append("stop")
        return {"halted_at": time.time(), "q": list(self.q)}

    def act_reset_to_home(self):
        self.calls.append("reset")
        self.q = [0.1] * 6
        return {"q": list(self.q)}

    def act_set_tcp_target(self, xyz):
        self.calls.append(("tcp", xyz))
        return {"accepted": True, "xyz": list(xyz)}

    def get_state(self):
        return {"id": self.robot_id, "model": self.model, "q": list(self.q)}


def _post(url, payload=None, timeout=2.0):
    body = json.dumps(payload or {}).encode()
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def test_serve_http_round_trip() -> None:
    bridge = _StubArm()
    server = serve_http(bridge, port=8794)
    try:
        # /list_robots
        out = _post("http://127.0.0.1:8794/list_robots")
        assert isinstance(out, list) and out[0]["id"] == "stub_arm", out

        # /stop_robot
        out = _post("http://127.0.0.1:8794/stop_robot")
        assert "halted_at" in out, out
        assert "stop" in bridge.calls

        # /reset_to_home
        out = _post("http://127.0.0.1:8794/reset_to_home")
        assert out["q"] == [0.1] * 6, out

        # /set_tcp_target
        out = _post("http://127.0.0.1:8794/set_tcp_target", {"xyz": [0.4, 0.0, 0.3]})
        assert out["accepted"] is True

        # /tool dispatches via the same act_* methods
        out = _post("http://127.0.0.1:8794/tool", {"tool": "reset_to_home"})
        assert out["status"] == "ok" and out["tool"] == "reset_to_home"

        # /usage returns enabled:false (no relay attached)
        with urllib.request.urlopen("http://127.0.0.1:8794/usage", timeout=2.0) as r:
            usage = json.loads(r.read().decode())
        assert usage == {"enabled": False}, usage

        # Unsupported action returns shaped error, not a crash.
        out = _post("http://127.0.0.1:8794/drive_forward", {"distance": 1.0})
        assert "error" in out and "not supported" in out["error"]

    finally:
        server.shutdown()
    print("[test_serve_http_round_trip] PASS")


if __name__ == "__main__":
    test_imports_and_version()
    test_tool_to_definition()
    test_intent_router()
    test_serve_http_round_trip()
    print("\nAll omnisim-bridges smoke tests passed.")
