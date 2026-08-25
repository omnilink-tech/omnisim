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

"""Publish ``/tf``, ``/tf_static`` and ``sensor_msgs/JointState`` (Tier 2).

TF
--
``GET /scene/tree`` is a flat, pre-order list where every node carries its
**world** pose plus ``parent_def``. That is enough to rebuild a real transform
tree rather than a flat pile of ``world -> thing`` transforms: each node's
transform is published relative to its nearest DEF-bearing ancestor, computed as
``T_parent^-1 . T_child`` (see :func:`conversions.relative_transform`).

Nodes whose ``parent_def`` is ``None`` are published relative to ``world``.

⚠ **``parent_def`` is the nearest DEF-BEARING ancestor, not the immediate
parent.** A robot's links are usually anonymous, so a whole subtree can report
the robot's DEF as its parent. The resulting tree is therefore correct but
flatter than the URDF's -- the transforms are right, the intermediate frames are
simply not there because OmniSim did not name them. Frames are only ever created
for nodes the harness gave a DEF.

Joint states
------------
``GET /robot/<def>/joints`` gives ``position`` (radians or metres) and
``velocity``.

⚠ **The harness computes ``velocity`` by finite difference, not from the engine**,
and returns ``null`` on the first read of a joint and whenever two reads land in
the same simulation instant. ``sensor_msgs/JointState`` has no way to say "this
one value is unknown", but an **empty** ``velocity`` array is the standard idiom
for "not provided". So the velocity array is published only when *every* joint
has a real number, and omitted entirely otherwise -- rather than substituting a
0.0 that a consumer would read as "this joint is stationary".

``effort`` is never published: OmniSim exposes no joint effort through this
surface at all.
"""

from __future__ import annotations

from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from omnisim_ros2 import entities as ent
from omnisim_ros2.conversions import (
    orientation_to_quaternion,
    position_to_xyz,
    relative_transform,
    sanitize_frame_id,
    sim_time_ms_to_ros,
)
from omnisim_ros2.harness_client import HarnessClient, HarnessUnreachable
from omnisim_ros2.node_support import guard_timer


class RobotStateNode(Node):
    def __init__(self) -> None:
        super().__init__("omnisim_robot_state")

        self.declare_parameter("harness_url", "http://127.0.0.1:6789")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("request_timeout_s", 15.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_joint_states", True)
        # Empty = every robot the harness reports. Naming robots explicitly keeps
        # the HTTP cost down on a many-robot world.
        self.declare_parameter("robots", [""])
        # Full tree by default; root-only is the cheap mode for a large scene.
        self.declare_parameter("tf_root_only", False)

        url = self.get_parameter("harness_url").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        timeout = self.get_parameter("request_timeout_s").get_parameter_value().double_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.do_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.do_joints = (
            self.get_parameter("publish_joint_states").get_parameter_value().bool_value
        )
        self.tf_root_only = self.get_parameter("tf_root_only").get_parameter_value().bool_value
        wanted = [
            r for r in self.get_parameter("robots").get_parameter_value().string_array_value if r
        ]
        self.wanted_robots = wanted

        self.client = HarnessClient(url, timeout_s=timeout)
        self.tf_broadcaster = TransformBroadcaster(self)
        # tf_static exists so a consumer can resolve `world` without waiting for
        # the first dynamic tick; see _publish_static_root.
        self.tf_static = StaticTransformBroadcaster(self)

        self._joint_pubs: dict[str, Any] = {}
        self._robot_defs: list[str] = []
        self._warned_unreachable = False
        self._warned_velocity = False

        self._publish_static_root()
        self.create_timer(1.0 / max(rate, 0.1), self.tick)
        self.get_logger().info(
            f"publishing {'tf ' if self.do_tf else ''}"
            f"{'joint_states ' if self.do_joints else ''}from {url} at {rate:g} Hz"
        )

    def _publish_static_root(self) -> None:
        """Anchor the world frame to ``map`` on ``/tf_static``.

        OmniSim has exactly one global frame and it is fixed, so the relationship
        is genuinely static. Publishing it means an RViz2 configured for the
        conventional ``map`` root resolves without any extra setup, and it costs
        one latched message.
        """
        if not self.do_tf or self.world_frame == "map":
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = self.world_frame
        t.transform.rotation.w = 1.0
        self.tf_static.sendTransform(t)

    def _stamp(self, sim_time_ms: float | None):
        from builtin_interfaces.msg import Time

        if sim_time_ms is None:
            return self.get_clock().now().to_msg()
        sec, nanosec = sim_time_ms_to_ros(sim_time_ms)
        t = Time()
        t.sec = sec
        t.nanosec = nanosec
        return t

    @guard_timer
    def tick(self) -> None:
        try:
            state = self.client.sim_state()
            sim_ms = state.body.get("sim_time_ms")
            if self.do_tf:
                self._publish_tf(sim_ms)
            if self.do_joints:
                self._publish_joints(sim_ms)
        except HarnessUnreachable as exc:
            if not self._warned_unreachable:
                self.get_logger().warn(f"{exc} -- output is stalled until it returns")
                self._warned_unreachable = True
            return
        if self._warned_unreachable:
            self.get_logger().info("harness is back; resuming")
            self._warned_unreachable = False

    def _publish_tf(self, sim_ms: float | None) -> None:
        resp = self.client.scene_tree()
        if not resp.ok:
            return
        nodes = resp.body.get("nodes") or []
        # Index world poses by DEF so a child can find its parent's frame.
        by_def = {
            n["def"]: n for n in nodes if isinstance(n, dict) and n.get("def")
        }
        stamp = self._stamp(sim_ms)
        transforms = []
        for node in nodes:
            if not isinstance(node, dict) or not ent.has_pose(node):
                continue
            def_name = node.get("def")
            if not def_name:
                # No DEF means no stable name; a frame id that changes between
                # ticks is worse than no frame at all.
                continue
            parent_def = node.get("parent_def")
            if self.tf_root_only and parent_def is not None:
                continue

            t = TransformStamped()
            t.header.stamp = stamp
            t.child_frame_id = sanitize_frame_id(def_name)
            parent_node = by_def.get(parent_def) if parent_def else None
            if parent_node is None:
                t.header.frame_id = self.world_frame
                x, y, z = position_to_xyz(node.get("position"))
                q = orientation_to_quaternion(node.get("orientation"))
            else:
                t.header.frame_id = sanitize_frame_id(parent_def)
                (x, y, z), q = relative_transform(
                    parent_node.get("position"),
                    parent_node.get("orientation"),
                    node.get("position"),
                    node.get("orientation"),
                )
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.translation.z = z
            t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
            t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
            transforms.append(t)
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def _robot_list(self) -> list[str]:
        if self.wanted_robots:
            return self.wanted_robots
        if self._robot_defs:
            return self._robot_defs
        resp = self.client.robots()
        if not resp.ok:
            return []
        # Only robots with joints are worth a JointState publisher.
        self._robot_defs = [
            str(r["def"])
            for r in (resp.body.get("robots") or [])
            if r.get("def") and (r.get("num_joints") or 0) > 0
        ]
        if self._robot_defs:
            self.get_logger().info(
                f"publishing joint states for: {', '.join(self._robot_defs)}"
            )
        return self._robot_defs

    def _publish_joints(self, sim_ms: float | None) -> None:
        for def_name in self._robot_list():
            resp = self.client.robot_joints(def_name)
            if not resp.ok:
                continue
            joints = resp.body.get("joints") or []
            if not joints:
                continue
            msg = JointState()
            msg.header.stamp = self._stamp(sim_ms)
            msg.header.frame_id = sanitize_frame_id(def_name)
            names, positions, velocities = [], [], []
            complete_velocity = True
            for i, j in enumerate(joints):
                names.append(str(j.get("name") or f"{def_name}_joint_{i}"))
                positions.append(float(j.get("position") or 0.0))
                v = j.get("velocity")
                if v is None:
                    complete_velocity = False
                else:
                    velocities.append(float(v))
            msg.name = names
            msg.position = positions
            # See the module docstring: an empty array means "not provided",
            # which is honest; a zero would read as "stationary", which is not.
            msg.velocity = velocities if complete_velocity else []
            if not complete_velocity and not self._warned_velocity:
                self.get_logger().info(
                    "at least one joint velocity is null (the harness finite-"
                    "differences it and has no sample yet); publishing an empty "
                    "velocity array rather than a fabricated zero"
                )
                self._warned_velocity = True

            topic = f"{sanitize_frame_id(def_name).lower()}/joint_states"
            pub = self._joint_pubs.get(topic)
            if pub is None:
                pub = self.create_publisher(JointState, topic, 10)
                self._joint_pubs[topic] = pub
                self.get_logger().info(f"publishing {len(names)} joints on {topic}")
            pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotStateNode()
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
