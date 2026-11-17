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
import urllib.error
import urllib.request
from pathlib import Path

# Allow running this file directly without pip-installing.
PKG_SRC = Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from omnisim_bridges import BridgeBase, Tool, serve_http, __version__  # noqa: E402


def test_imports_and_version() -> None:
    assert __version__ == "0.1.0"
    assert BridgeBase is not None
    assert Tool is not None
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


def test_no_keyword_ladder_is_exported() -> None:
    """The regex fallback is GONE, and importing it must fail loudly.

    `IntentRouter` was a pre-LLM keyword ladder exported for external
    bridges to subclass. It was deleted on 2026-09-22 together with every
    in-tree copy: an OmniKey is required for every chat turn, so a router
    that answers a keyless /prompt is the exact fallback the access policy
    forbids. This pins the removal so nobody quietly re-adds it.
    """
    import omnisim_bridges
    assert not hasattr(omnisim_bridges, "IntentRouter")
    assert "IntentRouter" not in omnisim_bridges.__all__
    try:
        import omnisim_bridges.intent_router  # noqa: F401
    except ImportError:
        pass
    else:                                        # pragma: no cover
        raise AssertionError("omnisim_bridges.intent_router came back")
    # describe_state survived the deletion -- it answers a QUESTION rather
    # than actuating anything -- and now lives next to its one caller.
    from omnisim_bridges import describe_state
    assert "no state to report" in describe_state(None)
    print("[test_no_keyword_ladder_is_exported] PASS")


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
    server = serve_http(bridge, port=0)
    port = server.server_address[1]
    base = f"http://127.0.0.1:{port}"
    try:
        # /list_robots
        out = _post(f"{base}/list_robots")
        assert isinstance(out, list) and out[0]["id"] == "stub_arm", out

        # /stop_robot
        out = _post(f"{base}/stop_robot")
        assert "halted_at" in out, out
        assert "stop" in bridge.calls

        # /reset_to_home
        out = _post(f"{base}/reset_to_home")
        assert out["q"] == [0.1] * 6, out

        # /set_tcp_target
        out = _post(f"{base}/set_tcp_target", {"xyz": [0.4, 0.0, 0.3]})
        assert out["accepted"] is True

        # /tool dispatches via the same act_* methods
        out = _post(f"{base}/tool", {"tool": "reset_to_home"})
        assert out["status"] == "ok" and out["tool"] == "reset_to_home"

        # /usage returns enabled:false (no relay attached)
        with urllib.request.urlopen(f"{base}/usage", timeout=2.0) as r:
            usage = json.loads(r.read().decode())
        assert usage == {"enabled": False}, usage

        # Unsupported action returns shaped error, not a crash.
        try:
            _post(f"{base}/drive_forward", {"distance": 1.0})
            raise AssertionError("unsupported action must fail")
        except urllib.error.HTTPError as exc:
            assert exc.code == 501
            out = json.loads(exc.read().decode())
            assert out["error"] == "not_supported"

        # Malformed JSON must never become an empty command with defaults.
        req = urllib.request.Request(
            f"{base}/set_tcp_target",
            data=b"{not-json",
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            urllib.request.urlopen(req, timeout=2.0)
            raise AssertionError("malformed JSON must fail")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
            out = json.loads(exc.read().decode())
            assert out["error"] == "malformed_json"

    finally:
        server.shutdown()
    print("[test_serve_http_round_trip] PASS")


if __name__ == "__main__":
    test_imports_and_version()
    test_tool_to_definition()
    test_no_keyword_ladder_is_exported()
    test_serve_http_round_trip()
    print("\nAll omnisim-bridges smoke tests passed.")
