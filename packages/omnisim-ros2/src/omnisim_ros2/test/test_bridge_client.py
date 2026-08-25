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

"""Tests for the per-robot bridge client. No ROS and no simulator required."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from omnisim_ros2.bridge_client import (
    BridgeClient,
    bridge_declares_sensors,
    bridge_servo_verb,
)

RECORDED = []


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        RECORDED.append((self.path, raw.decode() if raw else "", self.headers.get("Content-Type")))
        if self.path == "/get_robot_state":
            payload = {
                "id": "husky", "model": "Clearpath Husky", "mode": "idle",
                "x": 1.5, "y": -0.25, "yaw": 0.5,
                "v_linear": 0.4, "v_angular": -0.1, "sim_time": 12.5,
            }
        elif self.path == "/capabilities":
            payload = [{"id": "husky", "model": "Clearpath Husky",
                        "capabilities": {"actions": [{"name": "set_velocity"}]}}]
        elif self.path == "/read_sensor":
            payload = {"available": False,
                       "note": "OmniSim restricts device APIs to the owning controller"}
        elif self.path == "/list_sensors":
            payload = {"sensors": [
                {"name": "imu_data", "type": "InertialUnit",
                 "readable": True, "unit": "quaternion_xyzw", "shape": 4,
                 "mount": {"translation": [0.0, 0.0, 0.0],
                           "rotation_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1]}},
                {"name": "base_laser", "type": "Lidar",
                 "readable": True, "unit": "m", "shape": "n",
                 "mount": {"translation": [0.2012, 0.0, 0.505],
                           "rotation_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1]}},
            ], "count": 2, "mount_frame": "robot_base", "mounts_measured": True}
        else:
            payload = {"ok": True}
        body = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


@pytest.fixture()
def bridge():
    RECORDED.clear()
    server = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    host, port = server.server_address
    yield BridgeClient(f"http://{host}:{port}", timeout_s=5)
    server.shutdown()


def test_get_robot_state(bridge):
    r = bridge.get_robot_state()
    assert r.ok
    assert r.body["x"] == 1.5
    assert r.body["v_linear"] == 0.4


def test_set_velocity_field_names(bridge):
    """PROTOCOL.md 6.2 names these 'linear'/'angular'. Some older prose says
    'v'/'w'; the bridge rejects those, so the names are pinned here."""
    bridge.set_velocity(0.6, -0.3)
    path, raw, ctype = RECORDED[-1]
    assert path == "/set_velocity"
    assert json.loads(raw) == {"linear": 0.6, "angular": -0.3}
    # The bridge answers 415 without this header.
    assert ctype == "application/json"


def test_set_velocity_coerces_to_float(bridge):
    bridge.set_velocity(1, 0)
    assert json.loads(RECORDED[-1][1]) == {"linear": 1.0, "angular": 0.0}


def test_set_joint_positions(bridge):
    bridge.set_joint_positions([0.1, 0.2, 0.3])
    path, raw, _ = RECORDED[-1]
    assert path == "/set_joint_positions"
    assert json.loads(raw) == {"q": [0.1, 0.2, 0.3]}


def test_servo_joint_positions(bridge):
    """PROTOCOL.md 6.1: the streaming lane takes the same {"q": [...]} payload
    as the goal verb, on its own path."""
    bridge.servo_joint_positions([0.1, 0.2, 0.3])
    path, raw, ctype = RECORDED[-1]
    assert path == "/servo_joint_positions"
    assert json.loads(raw) == {"q": [0.1, 0.2, 0.3]}
    assert ctype == "application/json"


def test_servo_joint_positions_custom_verb_never_doubles_slash(bridge):
    bridge.servo_joint_positions([0.5], verb="/servo_joint_positions")
    assert RECORDED[-1][0] == "/servo_joint_positions"


# Captured shape: the arm bridge's /capabilities entry, as the transport wraps
# it ({"items": [...]}) -- capabilities.servo is the §6.1 self-description.
ARM_CAPS = {"items": [{"id": "ur5e", "model": "UR5e", "capabilities": {
    "busy_rejecting_actions": ["pick", "set_joint_positions"],
    "busy_overriding_actions": ["stop_robot", "reset_to_home",
                                "servo_joint_positions"],
    "servo": {"verb": "servo_joint_positions", "non_blocking": True,
              "last_write_wins": True, "done_tolerance_rad": 0.02},
}}]}


def test_bridge_servo_verb_reads_capabilities_servo():
    assert bridge_servo_verb(ARM_CAPS) == "servo_joint_positions"


def test_bridge_servo_verb_busy_overriding_fallback():
    """A bridge that predates capabilities.servo but lists the verb among its
    busy-overriding actions still advertises the streaming lane."""
    caps = {"items": [{"capabilities": {
        "busy_overriding_actions": ["stop_robot", "servo_joint_positions"]}}]}
    assert bridge_servo_verb(caps) == "servo_joint_positions"


def test_bridge_servo_verb_none_for_mobile_bridge():
    """A mobile bridge has actions but no servo block: the caller must fall
    back to the goal verb, signalled by None."""
    caps = {"items": [{"id": "husky", "capabilities": {
        "actions": [{"name": "set_velocity"}]}}]}
    assert bridge_servo_verb(caps) is None


def test_bridge_servo_verb_is_shape_tolerant():
    assert bridge_servo_verb({}) is None
    assert bridge_servo_verb({"items": []}) is None
    assert bridge_servo_verb({"items": ["nonsense", None, 5]}) is None
    assert bridge_servo_verb({"items": [{"capabilities": None}]}) is None
    # An empty or non-string verb must not be trusted.
    assert bridge_servo_verb({"items": [{"capabilities": {"servo": {"verb": ""}}}]}) is None
    assert bridge_servo_verb({"items": [{"capabilities": {"servo": {"verb": 5}}}]}) is None
    # A bare (unwrapped) capabilities reply still works.
    assert bridge_servo_verb(
        {"capabilities": {"servo": {"verb": "servo_joint_positions"}}}
    ) == "servo_joint_positions"


def test_read_sensor_unavailable_is_a_200_not_an_error(bridge):
    """A bridge that does not proxy sensors answers 200 with available:false --
    the structurally correct v1.0 answer. It must not read as a failure."""
    r = bridge.read_sensor("camera")
    assert r.ok
    assert r.body["available"] is False


def test_list_sensors_returns_catalog_with_measured_mounts(bridge):
    r = bridge.list_sensors()
    assert r.ok
    assert RECORDED[-1][0] == "/list_sensors"
    names = {s["name"]: s for s in r.body["sensors"]}
    assert names["base_laser"]["type"] == "Lidar"
    # The mount is what makes an honest TF frame possible; a lidar 0.5 m up
    # must not be published at the robot origin.
    assert names["base_laser"]["mount"]["translation"] == [0.2012, 0.0, 0.505]
    assert r.body["mounts_measured"] is True


def test_capabilities_array_is_wrapped(bridge):
    r = bridge.capabilities()
    assert r.ok
    assert isinstance(r.body["items"], list)
    assert r.body["items"][0]["id"] == "husky"


def test_bridge_declares_sensors():
    assert not bridge_declares_sensors({"items": [{"capabilities": {"actions": [{"name": "set_velocity"}]}}]})
    assert bridge_declares_sensors({"items": [{"capabilities": {"actions": [{"name": "read_sensor"}]}}]})
    assert bridge_declares_sensors({"items": [{"capabilities": {"sensors": ["camera"]}}]})
    assert not bridge_declares_sensors({})
    # Must not raise on shapes it has never seen.
    assert not bridge_declares_sensors({"items": ["nonsense", None, 5]})


def test_stop_robot(bridge):
    bridge.stop_robot()
    assert RECORDED[-1][0] == "/stop_robot"
