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

"""Publish ``nav_msgs/Odometry`` and the ``odom -> base_link`` TF (Tier 2).

Source: the robot bridge's ``POST /get_robot_state``, which returns ``x``, ``y``,
``yaw``, ``v_linear`` and ``v_angular`` -- **measured** from the robot's own
controller, not integrated by this node.

That matters, and it is the reason this node exists rather than deriving odometry
from the scene tree. The harness reports body poses but no body velocities, so
``simulation_interfaces``' ``EntityState.twist`` is unavailable (Tier 1 returns
zeros and says so). The bridge *does* measure velocity, so this is the one place
in the ROS surface where a real twist is available -- and ``/odom`` with a real
twist is exactly what Nav2 consumes.

WHAT THIS IS AND IS NOT
-----------------------
This is **ground-truth** odometry: the pose comes from the simulator's supervisor,
not from wheel encoders, so it has no drift. That makes it perfect for bringing a
navigation stack up and **wrong for evaluating a localisation algorithm**, which
needs the drift it is designed to correct. Nothing here adds noise; if you want
noisy odometry, add it downstream.

The covariance matrices are left at zero, which in ROS convention means "unknown"
rather than "perfectly certain". Publishing a small fabricated covariance would be
a claim about accuracy that nothing here measured.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from omnisim_ros2.bridge_client import BridgeClient, DEFAULT_BRIDGE_URL
from omnisim_ros2.conversions import sim_time_ms_to_ros, yaw_to_quaternion
from omnisim_ros2.harness_client import HarnessUnreachable
from omnisim_ros2.node_support import guard_timer


class OdomNode(Node):
    def __init__(self) -> None:
        super().__init__("omnisim_odom")

        self.declare_parameter("bridge_url", DEFAULT_BRIDGE_URL)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("request_timeout_s", 5.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        url = self.get_parameter("bridge_url").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        timeout = self.get_parameter("request_timeout_s").get_parameter_value().double_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.do_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value

        self.bridge = BridgeClient(url, timeout_s=timeout)
        self.pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.do_tf else None
        self._warned_unreachable = False

        self.create_timer(1.0 / max(rate, 0.1), self.tick)
        self.get_logger().info(
            f"publishing odom from {url}/get_robot_state at {rate:g} Hz "
            f"({self.odom_frame} -> {self.base_frame}); this is GROUND TRUTH, "
            f"drift-free -- not a wheel-encoder model"
        )

    def _stamp(self, sim_time_s):
        from builtin_interfaces.msg import Time

        if sim_time_s is None:
            return self.get_clock().now().to_msg()
        # The bridge reports sim_time in SECONDS (the harness uses ms).
        sec, nanosec = sim_time_ms_to_ros(float(sim_time_s) * 1000.0)
        t = Time()
        t.sec = sec
        t.nanosec = nanosec
        return t

    @guard_timer
    def tick(self) -> None:
        try:
            resp = self.bridge.get_robot_state()
        except HarnessUnreachable as exc:
            if not self._warned_unreachable:
                self.get_logger().warn(f"{exc} -- odom is stalled until it returns")
                self._warned_unreachable = True
            return
        if self._warned_unreachable:
            self.get_logger().info("bridge is back; resuming odom")
            self._warned_unreachable = False
        if not resp.ok:
            return

        body = resp.body
        x = body.get("x")
        y = body.get("y")
        yaw = body.get("yaw")
        if x is None or y is None or yaw is None:
            # Not a mobile bridge, or it has not ticked yet. Publishing a zero
            # pose would put a phantom robot at the origin.
            return

        stamp = self._stamp(body.get("sim_time"))
        q = yaw_to_quaternion(float(yaw))

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        # Twist is in the CHILD frame per REP-103, which is what the bridge
        # measures: forward speed and yaw rate in the robot's own frame.
        msg.twist.twist.linear.x = float(body.get("v_linear") or 0.0)
        msg.twist.twist.angular.z = float(body.get("v_angular") or 0.0)
        # Covariances stay zero == "unknown"; see the module docstring.
        self.pub.publish(msg)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNode()
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
