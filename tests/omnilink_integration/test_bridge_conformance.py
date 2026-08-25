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

"""Offline conformance checks for the canonical OmniLink robot bridge."""

from __future__ import annotations

import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PKG_SRC = REPO_ROOT / "packages" / "omnisim-bridges" / "src"
sys.path.insert(0, str(PKG_SRC))

from omnisim_bridges import BridgeBase, serve_http  # noqa: E402


class ConformanceBridge(BridgeBase):
    robot_id = "conformance_bot"
    model = "TestRobot"
    capabilities = {"class": "mobile", "actions": ["stop_robot", "drive_forward"]}

    def __init__(self) -> None:
        self.drives: list[float] = []

    def act_stop(self):
        return {"halted_at": time.time()}

    def act_drive_forward(self, distance, speed=None):
        self.drives.append(distance)
        return {"accepted": True, "distance": distance, "speed": speed}


def _request(base: str, path: str, *, payload=None, headers=None, method=None):
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    merged = dict(headers or {})
    if body is not None:
        merged.setdefault("Content-Type", "application/json")
    req = urllib.request.Request(base + path, data=body, headers=merged, method=method)
    with urllib.request.urlopen(req, timeout=3) as response:
        return response.status, dict(response.headers), json.loads(response.read().decode("utf-8"))


def _error(base: str, path: str, *, payload=None, raw=None, headers=None):
    data = raw if raw is not None else json.dumps(payload).encode("utf-8")
    merged = {"Content-Type": "application/json", **(headers or {})}
    req = urllib.request.Request(base + path, data=data, headers=merged, method="POST")
    try:
        urllib.request.urlopen(req, timeout=3)
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))
    raise AssertionError("request unexpectedly succeeded")


def test_protocol_headers_validation_and_error_envelope():
    bridge = ConformanceBridge()
    server = serve_http(bridge, port=0)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, headers, protocol = _request(base, "/protocol")
        assert status == 200
        assert protocol["omnisim_wire"] == "1.0"
        assert protocol["service"] == "robot_bridge"
        assert protocol["instance"]["robot_id"] == "conformance_bot"
        assert headers["X-OmniSim-Wire"] == "1.0"
        assert headers["X-OmniSim-Service"] == "robot_bridge"

        status, _, result = _request(
            base, "/drive_forward", payload={"distance": 1.25}, method="POST"
        )
        assert status == 200 and result["distance"] == 1.25
        assert bridge.drives == [1.25]

        status, _, _ = _request(
            base, "/drive_forward", payload={"id": "motion-2", "distance": 0.5}, method="POST"
        )
        assert status == 200
        status, duplicate = _error(
            base, "/drive_forward", payload={"id": "motion-2", "distance": 0.5}
        )
        assert status == 409 and duplicate["error"] == "duplicate_request"
        assert bridge.drives == [1.25, 0.5]

        status, error = _error(base, "/drive_forward", payload={})
        assert status == 400
        assert error == {
            "ok": False,
            "error": "missing_field",
            "message": "Required field 'distance' is missing.",
            "details": {"field": "distance"},
        }
        assert bridge.drives == [1.25, 0.5]

        status, error = _error(base, "/drive_forward", raw=b"{broken")
        assert status == 400 and error["error"] == "malformed_json"
        assert bridge.drives == [1.25, 0.5]
    finally:
        server.shutdown()
        server.server_close()


def test_cors_auth_and_version_negotiation():
    bridge = ConformanceBridge()
    server = serve_http(bridge, port=0, token="secret")
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, error = _error(base, "/stop_robot", payload={})
        assert status == 401 and error["error"] == "unauthorized"

        status, _, _ = _request(
            base,
            "/stop_robot",
            payload={},
            headers={"Authorization": "Bearer secret"},
            method="POST",
        )
        assert status == 200

        req = urllib.request.Request(
            base + "/protocol", headers={"Origin": "https://evil.example"}
        )
        try:
            urllib.request.urlopen(req, timeout=3)
        except urllib.error.HTTPError as exc:
            assert exc.code == 403
        else:
            raise AssertionError("untrusted Origin was accepted")

        req = urllib.request.Request(
            base + "/protocol", headers={"Accept-Protocol-Version": "2.0"}
        )
        try:
            urllib.request.urlopen(req, timeout=3)
        except urllib.error.HTTPError as exc:
            assert exc.code == 409
            assert json.loads(exc.read())["error"] == "protocol_unsupported"
        else:
            raise AssertionError("unsupported protocol version was accepted")
    finally:
        server.shutdown()
        server.server_close()


def test_non_loopback_bind_requires_token():
    try:
        serve_http(ConformanceBridge(), host="0.0.0.0", port=0, token="")
    except ValueError as exc:
        assert "requires OMNISIM_BRIDGE_TOKEN" in str(exc)
    else:
        raise AssertionError("unauthenticated non-loopback bridge was allowed")


if __name__ == "__main__":
    test_protocol_headers_validation_and_error_envelope()
    test_cors_auth_and_version_negotiation()
    test_non_loopback_bind_requires_token()
    print("All OmniLink bridge conformance tests passed.")
