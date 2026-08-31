#!/usr/bin/env python3
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

"""Relay /cmd_vel_smoothed -> /cmd_vel to close Nav2's command chain on OmniSim.

Jazzy's navigation_launch chain is:
    controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
                -> collision_monitor -> cmd_vel   (what OmniSim's command_node reads)

If you don't run collision_monitor (or it won't hold ACTIVE under load), the final
`/cmd_vel` is never published and the robot won't move on nav goals. Run this node to
forward the smoothed command straight to `/cmd_vel`:

    ros2 run omnisim_ros2_nav2 cmd_vel_relay

OmniSim publishes/consumes plain geometry_msgs/Twist (enable_stamped_cmd_vel: false),
so this relays Twist.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self.declare_parameter("in_topic", "/cmd_vel_smoothed")
        self.declare_parameter("out_topic", "/cmd_vel")
        src = self.get_parameter("in_topic").value
        dst = self.get_parameter("out_topic").value
        self.pub = self.create_publisher(Twist, dst, 10)
        self.sub = self.create_subscription(Twist, src, self._cb, 10)
        self.get_logger().info(f"relaying {src} -> {dst}")

    def _cb(self, msg: Twist) -> None:
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
