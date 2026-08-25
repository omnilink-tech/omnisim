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

"""Client for an OmniSim per-robot bridge (PROTOCOL.md §5-§6).

A *bridge* is an HTTP server started by the robot's own controller inside the
simulation (default ``127.0.0.1:8765``). It is a different surface from the World
Harness and exists for a structural reason:

    ``GET /robot/<def>/sensor/<name>`` on the harness returns **501 by design**.
    OmniSim, like upstream Webots, restricts device APIs to the controller that
    owns the device, so the supervisor genuinely cannot read another robot's
    camera, lidar or IMU. Anything device-shaped must come from the robot's own
    controller — which is exactly what a bridge is.

The same applies to actuation: the harness can teleport a body but cannot drive a
motor belonging to another robot. So ``cmd_vel`` and joint commands go here, not
to the harness.

Reuses :class:`~omnisim_ros2.harness_client.HarnessClient` for transport, since
both surfaces are the same stdlib-HTTP-and-JSON shape.
"""

from __future__ import annotations

from typing import Any

from omnisim_ros2.harness_client import HarnessClient, HarnessResponse

DEFAULT_BRIDGE_URL = "http://127.0.0.1:8765"

# The streaming setpoint verb (PROTOCOL.md §6.1). A bridge that has it says so
# in its capabilities (`capabilities.servo.verb`, and it also appears in
# `busy_overriding_actions`); a client must NOT assume it -- older bridges and
# non-arm bridges do not have the endpoint.
SERVO_VERB = "servo_joint_positions"


class BridgeClient(HarnessClient):
    """Typed calls against one per-robot bridge."""

    def __init__(self, base_url: str = DEFAULT_BRIDGE_URL, timeout_s: float = 10.0):
        super().__init__(base_url, timeout_s=timeout_s)

    def get_robot_state(self) -> HarnessResponse:
        """Measured pose and body velocity: ``x``, ``y``, ``yaw``, ``v_linear``,
        ``v_angular``, plus ``mode``, ``fault``, ``id``, ``model``, ``sim_time``."""
        return self.post("/get_robot_state", {})

    def capabilities(self) -> HarnessResponse:
        """Bridge capabilities. Note this returns a JSON *array*, which the
        transport wraps as ``{"items": [...]}``."""
        return self.post("/capabilities", {})

    def set_velocity(self, linear: float, angular: float) -> HarnessResponse:
        """Mobile base body-frame velocity command (PROTOCOL.md §6.2).

        The field names are ``linear``/``angular`` -- *not* ``v``/``w``, which
        appear in some older prose. Both are required by the bridge and must be
        finite; it rejects the call otherwise rather than substituting a default.
        """
        return self.post("/set_velocity", {"linear": float(linear), "angular": float(angular)})

    def stop_robot(self) -> HarnessResponse:
        """Escape hatch: cancels any running motion (never rejected as busy)."""
        return self.post("/stop_robot", {})

    def set_joint_positions(self, q: list[float]) -> HarnessResponse:
        """Arm joint position command, the GOAL verb (PROTOCOL.md §6.1).

        ⚠ Goal semantics: a second command arriving while the previous
        interpolation is still running answers **409 busy** and is NOT applied.
        For a setpoint *stream* (trajectory controller, teleop) use
        :meth:`servo_joint_positions` when the bridge advertises it -- see
        :func:`bridge_servo_verb`.
        """
        return self.post("/set_joint_positions", {"q": [float(v) for v in q]})

    def servo_joint_positions(self, q: list[float], verb: str = SERVO_VERB) -> HarnessResponse:
        """Arm joint setpoint, the STREAMING verb (PROTOCOL.md §6.1).

        Non-blocking, last-write-wins: it answers at dispatch (`achieved` is
        always null in the reply), a later servo command retargets the live
        stream in place, and a servo command never answers 409 -- it preempts
        an in-flight goal instead and names it in ``preempted``.

        Only bridges that advertise ``capabilities.servo`` have the endpoint;
        probe with :func:`bridge_servo_verb` and fall back to
        :meth:`set_joint_positions` when it returns None.
        """
        return self.post("/" + verb.lstrip("/"), {"q": [float(v) for v in q]})

    def read_sensor(self, sensor: str) -> HarnessResponse:
        """Sensor read-through (PROTOCOL.md §6.6).

        A bridge that does not proxy sensors answers
        ``{"available": false, "note": ...}`` with HTTP 200 -- the structurally
        correct answer, not an error. The same shape comes back for a sensor
        this robot does not carry, listing the ones it does.

        ``omnilink_mobile_bridge`` implements this; a reading in flight looks
        like ``{"available": true, "value": null, "warming_up": true}`` until
        the device has been enabled AND one simulation step has completed.
        """
        return self.post("/read_sensor", {"sensor": sensor})

    def list_sensors(self) -> HarnessResponse:
        """Sensor catalog (PROTOCOL.md §6.6): name, type, unit, shape, and the
        supervisor-measured ``mount`` pose of each device in the robot's frame.

        ``mount`` is ``null`` when the bridge could not measure it -- which is
        deliberately distinct from a zero offset.
        """
        return self.post("/list_sensors", {})


def bridge_servo_verb(caps: dict[str, Any]) -> str | None:
    """The streaming setpoint verb this bridge advertises, or None.

    Reads a ``/capabilities`` reply (the transport wraps the bridge's JSON
    array as ``{"items": [...]}``) and returns the verb named in
    ``capabilities.servo.verb`` -- the self-description PROTOCOL.md §6.1
    requires of any bridge that has the streaming lane. As a fallback it also
    recognises ``"servo_joint_positions"`` in ``busy_overriding_actions``,
    which the arm bridge published first.

    None means "goal verb only": send ``set_joint_positions`` and expect the
    409-busy behaviour on overlapping commands. Never probe by POSTing the
    servo verb blind -- an unknown path on some bridges is a 404 *after* the
    request was already refused a route, and tells you nothing a capabilities
    read does not.
    """
    items = caps.get("items") if isinstance(caps.get("items"), list) else [caps]
    for entry in items or []:
        if not isinstance(entry, dict):
            continue
        capabilities = entry.get("capabilities") or {}
        if not isinstance(capabilities, dict):
            continue
        servo = capabilities.get("servo")
        if isinstance(servo, dict):
            verb = servo.get("verb")
            if isinstance(verb, str) and verb:
                return verb
        overriding = capabilities.get("busy_overriding_actions")
        if isinstance(overriding, list) and SERVO_VERB in overriding:
            return SERVO_VERB
    return None


def bridge_declares_sensors(caps: dict[str, Any]) -> bool:
    """True when a bridge's capabilities advertise sensor read-through."""
    items = caps.get("items") if isinstance(caps.get("items"), list) else [caps]
    for entry in items or []:
        if not isinstance(entry, dict):
            continue
        capabilities = entry.get("capabilities") or {}
        if isinstance(capabilities, dict) and capabilities.get("sensors"):
            return True
        actions = capabilities.get("actions") if isinstance(capabilities, dict) else None
        if isinstance(actions, list) and any(
            (a.get("name") if isinstance(a, dict) else a) == "read_sensor" for a in actions
        ):
            return True
    return False
