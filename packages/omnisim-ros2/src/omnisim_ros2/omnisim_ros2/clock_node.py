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

Source: ``GET /sim/state`` -> ``sim_time_ms``. The harness reports it as
best-effort and names the reason when it cannot, in ``sim_time_source``; this node
branches on that rather than trusting a number that may be stale.

⚠ THIS NODE MUST NOT USE SIM TIME ITSELF. It is the publisher of record, so
``use_sim_time`` is forced off for it — a clock source that waits on its own
output never ticks. The launch file sets it, and ``__init__`` re-asserts it so
running the node bare is safe too.

⚠ THE CLOCK GOES BACKWARDS ON RESET, AND THAT IS CORRECT. ``ResetSimulation``
rewinds sim time to zero, so subscribers see a time jump. ROS handles this (it is
the same thing a rosbag loop does), but a node that caches timestamps across a
reset will see negative durations. That is a property of resetting a simulator,
not a defect here; the node logs the rewind so it is attributable.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock

from omnisim_ros2.conversions import sim_time_ms_to_ros
from omnisim_ros2.harness_client import HarnessClient, HarnessUnreachable
from omnisim_ros2.node_support import guard_timer

# The harness's own literal for "this number came from the simulator just now".
# Anything else means the value is not a live reading.
LIVE_SOURCE = "supervisor sim_state RPC"


class ClockNode(Node):
    def __init__(self) -> None:
        super().__init__("omnisim_clock")

        self.declare_parameter("harness_url", "http://127.0.0.1:6789")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("request_timeout_s", 5.0)

        url = self.get_parameter("harness_url").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        timeout = self.get_parameter("request_timeout_s").get_parameter_value().double_value

        # See the module docstring: the clock source cannot consume sim time.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, False)])

        self.client = HarnessClient(url, timeout_s=timeout)
        # QoS: the ROS convention for /clock is a plain reliable publisher with a
        # small queue -- late joiners want the NEXT tick, never a stale one.
        self.pub = self.create_publisher(Clock, "/clock", 10)

        self._last_ms: float | None = None
        self._warned_source = False
        self._warned_unreachable = False

        self.create_timer(1.0 / max(rate, 0.1), self.tick)
        self.get_logger().info(
            f"publishing /clock from {url} at {rate:g} Hz "
            f"(set use_sim_time:=true on every other node to follow it)"
        )

    @guard_timer
    def tick(self) -> None:
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

        ms = resp.body.get("sim_time_ms")
        source = resp.body.get("sim_time_source") or ""
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

        if self._last_ms is not None and ms < self._last_ms:
            self.get_logger().info(
                f"simulation time rewound {self._last_ms:.0f} -> {ms:.0f} ms "
                f"(a reset); subscribers will see a backwards time jump"
            )
        self._last_ms = ms

        msg = Clock()
        sec, nanosec = sim_time_ms_to_ros(ms)
        msg.clock.sec = sec
        msg.clock.nanosec = nanosec
        self.pub.publish(msg)


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
