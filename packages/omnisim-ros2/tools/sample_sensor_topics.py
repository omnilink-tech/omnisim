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

"""Print a one-line summary of each sensor topic, then exit.

Verification helper for the Tier 2 sensor lane. Exists because the bar for a
sensor topic is not "it publishes" but "it publishes values that CHANGE when
the robot moves" -- and a full `ros2 topic echo` of a 541-ray scan buries that
signal. Run it before and after a motion and diff the lines.

    python3 tools/sample_sensor_topics.py --timeout 15
"""

from __future__ import annotations

import argparse
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


class Sampler(Node):
    def __init__(self, timeout: float) -> None:
        super().__init__("omnisim_sensor_sampler")
        self.got: dict[str, str] = {}
        self.timeout = timeout
        self.create_subscription(Imu, "imu/data", self._imu, 10)
        self.create_subscription(LaserScan, "scan", self._scan, 10)
        self.create_subscription(PointStamped, "gps/local", self._gps, 10)

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _imu(self, m: Imu) -> None:
        self.got["imu/data"] = (
            f"quat=({m.orientation.x:+.4f},{m.orientation.y:+.4f},"
            f"{m.orientation.z:+.4f},{m.orientation.w:+.4f}) "
            f"yaw={self._yaw(m.orientation):+.4f} rad  "
            f"ang_vel_cov[0]={m.angular_velocity_covariance[0]:+.0f} "
            f"lin_acc_cov[0]={m.linear_acceleration_covariance[0]:+.0f}"
        )

    def _scan(self, m: LaserScan) -> None:
        finite = [r for r in m.ranges if math.isfinite(r)]
        rng = (f"min={min(finite):.3f} max={max(finite):.3f}"
               if finite else "no finite returns")
        self.got["scan"] = (
            f"n={len(m.ranges)} finite={len(finite)} inf={len(m.ranges) - len(finite)} "
            f"{rng}  fov=[{m.angle_min:+.3f},{m.angle_max:+.3f}] "
            f"range=[{m.range_min:.2f},{m.range_max:.2f}] frame={m.header.frame_id}"
        )

    def _gps(self, m: PointStamped) -> None:
        self.got["gps/local"] = (
            f"x={m.point.x:+.4f} y={m.point.y:+.4f} z={m.point.z:+.4f} "
            f"frame={m.header.frame_id}"
        )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    rclpy.init()
    node = Sampler(args.timeout)
    wanted = {"imu/data", "scan", "gps/local"}
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while (rclpy.ok() and set(node.got) != wanted
           and node.get_clock().now().nanoseconds < deadline):
        rclpy.spin_once(node, timeout_sec=0.2)

    if args.label:
        print(f"--- {args.label} ---")
    for topic in ("imu/data", "scan", "gps/local"):
        print(f"  {topic:12s} {node.got.get(topic, 'NO MESSAGE RECEIVED')}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
