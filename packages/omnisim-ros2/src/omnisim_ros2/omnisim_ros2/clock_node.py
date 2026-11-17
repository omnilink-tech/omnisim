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

"""Publish OmniSim's simulation clock on ``/clock`` (Tier 2).

Everything else in a ROS 2 stack that cares about simulated time depends on this:
with ``use_sim_time:=true``, every other node's ``get_clock().now()`` follows what
is published here.

Source, in order of preference:

1. With ``use_bridge_clock`` (the bring-up sets it whenever the bridge nodes run):
   the robot bridge's ``POST /get_robot_state`` -> ``sim_time``, the robot
   controller's own clock, refreshed every control step. It is the clock the
   bridge's sensor stamps are in, and it is FINE-GRAINED.
2. Otherwise ``GET /sim/state`` -> ``engine_time_ms``: the same engine clock, but
   sampled only when the harness's injected supervisor steps. Measured 2026-10-02
   on the Husky under ``--engine-mode realtime``: 11 distinct values in 30 wall
   seconds, ~4 s jumps every ~6 s, so ``use_sim_time:=true`` timers fired about
   once per jump. Right ruler, coarse sampling -- fine for "what time is it",
   not for driving 5 Hz timers.
3. An older harness without ``engine_time_ms``: ``sim_time_ms``, the supervisor's
   loop counter, with a warning. Until 2026-10-02 this was the only source, and it
   is a different ruler: it advanced 23 s while the engine advanced 281 s, which
   throttled every sim-time node to ~0.1 Hz (``conversions.simulated_time_ms``).

The harness names the reason when it has no reading, in ``sim_time_source``; this
node branches on that rather than trusting a number that may be stale.

⚠ THIS NODE MUST NOT USE SIM TIME ITSELF. It is the publisher of record, so
``use_sim_time`` is forced off for it — a clock source that waits on its own
output never ticks. The launch file sets it, and ``__init__`` re-asserts it so
running the node bare is safe too.

⚠ A WORLD (RE)LOAD SENDS THE CLOCK BACKWARDS, AND THAT IS CORRECT. The engine
clock restarts with the world, so subscribers see a time jump. ROS handles this
(it is the same thing a rosbag loop does), but a node that caches timestamps
across it will see negative durations. The node logs any rewind so it is
attributable. Note ``POST /sim/reset`` does NOT rewind ``engine_time_ms``
(PROTOCOL.md §7.39); only the supervisor counter rewinds there.
"""

from __future__ import annotations

import rclpy
from rclpy.clock import Clock as RclClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock

from omnisim_ros2.bridge_client import BridgeClient
from omnisim_ros2.conversions import sim_time_ms_to_ros, simulated_time_ms
from omnisim_ros2.harness_client import HarnessClient, HarnessUnreachable
from omnisim_ros2.node_support import guard_timer

# The harness's own literal for "this number came from the simulator just now".
# Anything else means the value is not a live reading.
LIVE_SOURCE = "supervisor sim_state RPC"


class ClockNode(Node):
    def __init__(self) -> None:
        super().__init__("omnisim_clock")

        self.declare_parameter("harness_url", "http://127.0.0.1:6789")
        self.declare_parameter("bridge_url", "http://127.0.0.1:8765")
        self.declare_parameter("use_bridge_clock", False)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("request_timeout_s", 5.0)

        url = self.get_parameter("harness_url").get_parameter_value().string_value
        bridge_url = self.get_parameter("bridge_url").get_parameter_value().string_value
        use_bridge = self.get_parameter("use_bridge_clock").get_parameter_value().bool_value
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        timeout = self.get_parameter("request_timeout_s").get_parameter_value().double_value

        # See the module docstring: the clock source cannot consume sim time.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, False)])

        self.client = HarnessClient(url, timeout_s=timeout)
        self.bridge = BridgeClient(bridge_url, timeout_s=timeout) if use_bridge else None
        self._warned_bridge = False
        # QoS: the ROS convention for /clock is a plain reliable publisher with a
        # small queue -- late joiners want the NEXT tick, never a stale one.
        self.pub = self.create_publisher(Clock, "/clock", 10)

        self._last_ms: float | None = None
        self._warned_source = False
        self._warned_fallback = False
        self._warned_unreachable = False

        # Poll independently of system realtime adjustments. Published stamps
        # still come from the simulator, including legitimate world-load rewinds.
        self._poll_clock = RclClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / max(rate, 0.1), self.tick, clock=self._poll_clock)
        src = f"the robot bridge {bridge_url} (fallback {url})" if self.bridge else url
        self.get_logger().info(
            f"publishing /clock from {src} at {rate:g} Hz "
            f"(set use_sim_time:=true on every other node to follow it)"
        )

    def _bridge_ms(self) -> float | None:
        """The robot controller's clock in ms, or None (then the harness serves)."""
        try:
            resp = self.bridge.get_robot_state()
        except HarnessUnreachable as exc:
            if not self._warned_bridge:
                self.get_logger().warn(
                    f"{exc} -- falling back to the harness engine clock, which is "
                    f"sampled only when its supervisor steps (coarse)"
                )
                self._warned_bridge = True
            return None
        t = resp.body.get("sim_time") if isinstance(resp.body, dict) else None
        if not isinstance(t, (int, float)):
            return None
        if self._warned_bridge:
            self.get_logger().info("robot bridge is back; /clock follows it again")
            self._warned_bridge = False
        return float(t) * 1000.0

    def _publish(self, ms: float) -> None:
        if self._last_ms is not None and ms < self._last_ms:
            self.get_logger().info(
                f"simulation time rewound {self._last_ms:.0f} -> {ms:.0f} ms "
                f"(a world reload); subscribers will see a backwards time jump"
            )
        self._last_ms = ms
        msg = Clock()
        sec, nanosec = sim_time_ms_to_ros(ms)
        msg.clock.sec = sec
        msg.clock.nanosec = nanosec
        self.pub.publish(msg)

    @guard_timer
    def tick(self) -> None:
        if self.bridge is not None:
            ms = self._bridge_ms()
            if ms is not None:
                self._publish(ms)
                return
        try:
            resp = self.client.sim_state()
        except HarnessUnreachable as exc:
            if not self._warned_unreachable:
                self.get_logger().warn(f"{exc} -- /clock is stalled until it returns")
                self._warned_unreachable = True
            return
        if self._warned_unreachable:
            self.get_logger().info("harness is back; resuming /clock")
            self._warned_unreachable = False

        ms, field = simulated_time_ms(resp.body)
        source = resp.body.get("sim_time_source") or ""
        if ms is not None and field != "engine_time_ms" and not self._warned_fallback:
            self.get_logger().warn(
                "harness reports no engine_time_ms (older harness); publishing the "
                "supervisor counter sim_time_ms, which can run far slower than the "
                "engine and disagree with sensor stamps"
            )
            self._warned_fallback = True
        if ms is None:
            # No world loaded, or a load is in flight. Publishing a fabricated
            # zero would look like a reset to every subscriber, so publish
            # nothing and say why -- once.
            if not self._warned_source:
                self.get_logger().warn(
                    f"no simulation time available ({source!r}); not publishing /clock"
                )
                self._warned_source = True
            return
        if source != LIVE_SOURCE and not self._warned_source:
            self.get_logger().warn(
                f"sim_time_source is {source!r}, not {LIVE_SOURCE!r}; "
                f"the clock may not be a live reading"
            )
            self._warned_source = True

        self._publish(ms)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
