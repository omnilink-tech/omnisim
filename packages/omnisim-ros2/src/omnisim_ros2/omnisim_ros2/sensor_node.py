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

"""Publish ``sensor_msgs`` topics from a robot bridge's ``/read_sensor`` (Tier 2).

WHY THIS TALKS TO THE BRIDGE AND NOT THE HARNESS
------------------------------------------------
``GET /robot/<def>/sensor/<name>`` on the World Harness returns **501 by
design**: OmniSim restricts device APIs to the controller that owns the device,
so a supervisor cannot honestly read another robot's IMU or lidar. The robot's
own controller is the only source, which is why this node speaks to the bridge
(``PROTOCOL.md`` §6.6) exactly as :mod:`omnisim_ros2.odom_node` does.

WHAT IS PUBLISHED, AND WHAT IS DELIBERATELY NOT
-----------------------------------------------
Measured on the shipped Husky (2026-08-17, machine ``9722d23d12a3``, CPU
``mj_step``), sensor by sensor:

=================  ==========================================================
``InertialUnit``   **live** -- tracked the supervisor's yaw to 4 decimals
                   (0.0606 / 0.1224 / 0.1360 against 0.0595 / 0.1204 / 0.1360)
``GPS``            **live** -- moved 5.59 m during a drive
``Lidar``          **live** -- 541 finite returns, changed under motion
``PositionSensor`` **live** -- wheel angle reached 34.5 rad
``Gyro``           **DEAD** -- read exactly ``[0, 0, 0]`` while the robot was
                   demonstrably rotating (yaw 0 -> 0.136 rad)
``Accelerometer``  **DEAD** -- never produced a sample at all; ``getValues()``
                   stays ``None`` indefinitely, so not even gravity is read
=================  ==========================================================

So ``sensor_msgs/Imu`` goes out with a **real orientation** and with its
angular-velocity and linear-acceleration covariances set to ``-1``. That is not
a placeholder: ``-1`` in ``[0]`` is the ROS-wide convention for "this component
is not available", and it is the honest encoding for a component the simulator
does not measure. Publishing a zero there would claim the robot is not rotating
and is in free fall, neither of which anything measured.

⚠ The yaw RATE is genuinely available -- but from the bridge's own pose
differencing, not from the gyro -- and it is already published on ``/odom`` as
``twist.twist.angular.z``. It is deliberately not copied into the Imu message,
because an ``Imu`` whose fields come from two different sources is exactly the
kind of quiet blend that makes a later reader trust a number they should not.

LIDAR LAYERS
------------
``sensor_msgs/LaserScan`` is single-layer by definition. OmniSim's ``Lidar``
node defaults to **4** layers and the URDF importer only writes
``numberOfLayers`` when the URDF asks for more than one, so a URDF-declared
planar scanner still arrives as a 4-layer device. ``lidar_layer`` selects which
layer becomes the scan; the default ``-1`` picks the layer with the most finite
returns and **logs which one it chose**, so the choice is visible rather than
silent. Set it explicitly for a genuinely multi-layer device.

NO-RETURN ENCODING
------------------
A lidar ray that hits nothing reads ``+inf``, which is not valid JSON, so the
bridge's response sanitizer sends ``null``. This node maps ``null`` back to
``inf`` -- the value ROS consumers already treat as "out of range". It is never
mapped to 0.0, which every consumer reads as an obstacle touching the sensor.
"""

from __future__ import annotations

from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus
from tf2_ros import StaticTransformBroadcaster

from omnisim_ros2.bridge_client import BridgeClient, DEFAULT_BRIDGE_URL
from omnisim_ros2.conversions import (
    lidar_layer_ranges,
    matrix_to_quaternion,
    select_lidar_layer,
    sim_time_ms_to_ros,
)
from omnisim_ros2.harness_client import HarnessUnreachable
from omnisim_ros2.node_support import guard_timer

# ROS convention: covariance[0] == -1 means "this component is not measured".
UNAVAILABLE = -1.0


class SensorNode(Node):
    def __init__(self) -> None:
        super().__init__("omnisim_sensors")

        self.declare_parameter("bridge_url", DEFAULT_BRIDGE_URL)
        # Deliberately modest. Every tick costs one HTTP request PER SENSOR;
        # see the throughput note in the launch file.
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("request_timeout_s", 5.0)
        self.declare_parameter("base_frame", "base_link")
        # Empty means "name the frame after the sensor itself".
        self.declare_parameter("imu_frame", "")
        self.declare_parameter("scan_frame", "")
        self.declare_parameter("lidar_layer", -1)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("publish_gps", True)
        # Webots orders a scan left-to-right; ROS LaserScan starts at
        # angle_min and increases. See _publish_scan.
        self.declare_parameter("reverse_ranges", True)

        url = self.get_parameter("bridge_url").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        timeout = self.get_parameter("request_timeout_s").get_parameter_value().double_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.imu_frame = self.get_parameter("imu_frame").get_parameter_value().string_value
        self.scan_frame = self.get_parameter("scan_frame").get_parameter_value().string_value
        self.lidar_layer = self.get_parameter("lidar_layer").get_parameter_value().integer_value
        self.do_imu = self.get_parameter("publish_imu").get_parameter_value().bool_value
        self.do_scan = self.get_parameter("publish_scan").get_parameter_value().bool_value
        self.do_gps = self.get_parameter("publish_gps").get_parameter_value().bool_value
        self.reverse_ranges = (
            self.get_parameter("reverse_ranges").get_parameter_value().bool_value
        )

        self.bridge = BridgeClient(url, timeout_s=timeout)
        self.tf_static = StaticTransformBroadcaster(self)

        self._imu_pub = None
        self._scan_pub = None
        self._fix_pub = None
        self._local_pub = None
        self._discovered = False
        self._imu_name: str | None = None
        self._lidar_name: str | None = None
        self._gps_name: str | None = None
        self._chosen_layer: int | None = None
        self._warned_unreachable = False
        self._warned_no_sensors = False
        self._warmup_logged: set[str] = set()

        self.create_timer(1.0 / max(rate, 0.1), self.tick)
        self.get_logger().info(
            f"reading sensors from {url}/read_sensor at {rate:g} Hz"
        )

    # -- discovery ---------------------------------------------------------

    def _discover(self) -> bool:
        """Ask the bridge what this robot carries, once.

        Discovery rather than assumption: the sensor set depends on the world,
        and on a URDF robot it depends on OMNISIM_URDF_USE_SENSORS being set at
        world load. Guessing device names would produce a node that publishes
        nothing and cannot say why.
        """
        resp = self.bridge.list_sensors()
        if not resp.ok:
            return False
        sensors = resp.body.get("sensors") or []
        if not sensors:
            if not self._warned_no_sensors:
                self.get_logger().warn(
                    "the bridge reports NO readable sensors. A URDF robot only "
                    "gets devices when the world was loaded with "
                    "OMNISIM_URDF_USE_SENSORS=1 -- measured on the shipped "
                    "Husky: 0 devices without it, 5 with it."
                )
                self._warned_no_sensors = True
            return False

        mounts: dict[str, Any] = {}
        for entry in sensors:
            if not isinstance(entry, dict) or not entry.get("readable", True):
                continue
            name, kind = entry.get("name"), entry.get("type")
            if not name:
                continue
            if kind == "InertialUnit" and self._imu_name is None:
                self._imu_name = name
            elif kind == "Lidar" and self._lidar_name is None:
                self._lidar_name = name
            elif kind == "GPS" and self._gps_name is None:
                self._gps_name = name
            else:
                continue
            if entry.get("mount"):
                mounts[name] = entry["mount"]

        if self.do_imu and self._imu_name:
            self._imu_pub = self.create_publisher(Imu, "imu/data", 10)
        if self.do_scan and self._lidar_name:
            self._scan_pub = self.create_publisher(LaserScan, "scan", 10)
        if self.do_gps and self._gps_name:
            # The message type depends on what the GPS actually reports; both
            # publishers are created lazily in _publish_gps once we know.
            pass

        self._publish_mount_tf(mounts)
        found = [n for n in (self._imu_name, self._lidar_name, self._gps_name) if n]
        self.get_logger().info(
            f"sensors: {', '.join(found) if found else 'none usable'} "
            f"(of {len(sensors)} reported)"
        )
        if self._imu_name:
            self.get_logger().info(
                "Imu: orientation is REAL; angular_velocity and "
                "linear_acceleration are published with covariance[0] = -1 "
                "('not available') because OmniSim's Gyro reads a constant "
                "zero and its Accelerometer never produces a sample"
            )
        return True

    def frame_for(self, sensor: str, override: str) -> str:
        return override or sensor

    def _publish_mount_tf(self, mounts: dict[str, Any]) -> None:
        """Latch base_frame -> sensor_frame from the bridge's MEASURED mounts.

        The bridge reads each device's pose through the supervisor, so these
        are real offsets. A sensor whose mount could not be measured gets no
        transform at all rather than an identity one -- a fabricated zero would
        put the lidar at the robot's origin, half a metre from where it is.
        """
        transforms = []
        for sensor, frame_override in (
            (self._imu_name, self.imu_frame),
            (self._lidar_name, self.scan_frame),
            (self._gps_name, ""),
        ):
            if not sensor:
                continue
            mount = mounts.get(sensor)
            if not mount:
                self.get_logger().warn(
                    f"no measured mount for {sensor!r}; publishing no "
                    f"{self.base_frame} -> {self.frame_for(sensor, frame_override)} "
                    f"transform. TF lookups against it will fail until you "
                    f"supply one (normally from the URDF via robot_state_publisher)."
                )
                continue
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = self.frame_for(sensor, frame_override)
            xyz = mount.get("translation") or [0.0, 0.0, 0.0]
            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])
            q = matrix_to_quaternion(mount.get("rotation_matrix") or
                                     [1, 0, 0, 0, 1, 0, 0, 0, 1])
            t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
            t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
            transforms.append(t)
        if transforms:
            self.tf_static.sendTransform(transforms)

    # -- helpers -----------------------------------------------------------

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

    def _read(self, sensor: str) -> dict | None:
        """One sensor read, or None when there is nothing worth publishing."""
        resp = self.bridge.read_sensor(sensor)
        if not resp.ok:
            return None
        body = resp.body
        if not body.get("available"):
            return None
        if body.get("value") is None:
            # Warm-up: the device was enabled on first read and has no sample
            # until the next step. Say so once, then stay quiet.
            if body.get("warming_up") and sensor not in self._warmup_logged:
                self.get_logger().info(f"{sensor}: warming up, no sample yet")
                self._warmup_logged.add(sensor)
            return None
        return body

    # -- tick --------------------------------------------------------------

    @guard_timer
    def tick(self) -> None:
        try:
            if not self._discovered:
                self._discovered = self._discover()
                if not self._discovered:
                    return
            if self._imu_pub is not None:
                self._publish_imu()
            if self._scan_pub is not None:
                self._publish_scan()
            if self.do_gps and self._gps_name:
                self._publish_gps()
        except HarnessUnreachable as exc:
            if not self._warned_unreachable:
                self.get_logger().warn(f"{exc} -- sensors are stalled until it returns")
                self._warned_unreachable = True
            return
        if self._warned_unreachable:
            self.get_logger().info("bridge is back; resuming sensors")
            self._warned_unreachable = False

    def _publish_imu(self) -> None:
        body = self._read(self._imu_name)
        if body is None:
            return
        q = body.get("value")
        if not isinstance(q, list) or len(q) != 4 or any(v is None for v in q):
            return
        msg = Imu()
        msg.header.stamp = self._stamp(body.get("sim_time"))
        msg.header.frame_id = self.frame_for(self._imu_name, self.imu_frame)
        msg.orientation.x = float(q[0])
        msg.orientation.y = float(q[1])
        msg.orientation.z = float(q[2])
        msg.orientation.w = float(q[3])
        # Zero covariance == "unknown accuracy", which is true: the orientation
        # is exact ground truth and nothing here measured an error bound.
        # -1 in [0] is the ROS convention for "this component is absent", which
        # is the honest statement for both of these. See the module docstring.
        msg.angular_velocity_covariance[0] = UNAVAILABLE
        msg.linear_acceleration_covariance[0] = UNAVAILABLE
        self._imu_pub.publish(msg)

    def _select_layer(self, values: list, layers: int, per: int) -> int:
        """Which lidar layer becomes the LaserScan: explicit, or auto-picked."""
        if self.lidar_layer >= 0:
            return min(self.lidar_layer, layers - 1)
        return select_lidar_layer(values, layers, per)

    def _publish_scan(self) -> None:
        body = self._read(self._lidar_name)
        if body is None:
            return
        values = body.get("value")
        layout = body.get("layout") or {}
        if not isinstance(values, list) or not values:
            return
        layers = max(int(layout.get("number_of_layers") or 1), 1)
        per = max(int(layout.get("horizontal_resolution") or len(values)), 1)
        if len(values) < layers * per:
            return

        if self._chosen_layer is None:
            self._chosen_layer = self._select_layer(values, layers, per)
            if layers > 1:
                counts = [
                    sum(1 for v in values[i * per:(i + 1) * per] if v is not None)
                    for i in range(layers)
                ]
                self.get_logger().info(
                    f"lidar has {layers} layers (finite returns per layer: "
                    f"{counts}); publishing layer {self._chosen_layer} as "
                    f"/scan. Override with -p lidar_layer:=<n>."
                )
        ranges = lidar_layer_ranges(values, layers, per, self._chosen_layer,
                                    reverse=self.reverse_ranges)
        fov = float(layout.get("fov") or 0.0)
        msg = LaserScan()
        msg.header.stamp = self._stamp(body.get("sim_time"))
        msg.header.frame_id = self.frame_for(self._lidar_name, self.scan_frame)
        msg.angle_min = -fov / 2.0
        msg.angle_max = fov / 2.0
        msg.angle_increment = fov / max(len(ranges) - 1, 1)
        # Left at zero: the bridge exposes no per-ray timing, and a fabricated
        # value would be used by consumers to de-skew a moving scan.
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = float(layout.get("min_range") or 0.0)
        msg.range_max = float(layout.get("max_range") or 0.0)
        msg.ranges = ranges
        self._scan_pub.publish(msg)

    def _publish_gps(self) -> None:
        body = self._read(self._gps_name)
        if body is None:
            return
        vals = body.get("value")
        if not isinstance(vals, list) or len(vals) != 3 or any(v is None for v in vals):
            return
        stamp = self._stamp(body.get("sim_time"))
        frame = self.frame_for(self._gps_name, "")

        if body.get("coordinate_system") == "WGS84":
            if self._fix_pub is None:
                self._fix_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
                self.get_logger().info(
                    "GPS is WGS84; publishing sensor_msgs/NavSatFix on gps/fix"
                )
            msg = NavSatFix()
            msg.header.stamp = stamp
            msg.header.frame_id = frame
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = float(vals[0])
            msg.longitude = float(vals[1])
            msg.altitude = float(vals[2])
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self._fix_pub.publish(msg)
            return

        # A LOCAL GPS reports metres in the world frame, not degrees.
        # Publishing those as NavSatFix latitude/longitude would place the
        # robot a few metres off West Africa, so it goes out as a point.
        if self._local_pub is None:
            self._local_pub = self.create_publisher(PointStamped, "gps/local", 10)
            self.get_logger().info(
                "GPS reports LOCAL metres, not WGS84 degrees; publishing "
                "geometry_msgs/PointStamped on gps/local rather than a "
                "NavSatFix whose lat/lon would be metres mislabelled as degrees"
            )
        msg = PointStamped()
        msg.header.stamp = stamp
        # A local GPS reads a WORLD-frame position, so its frame is the world,
        # not the sensor body.
        msg.header.frame_id = "world"
        msg.point.x = float(vals[0])
        msg.point.y = float(vals[1])
        msg.point.z = float(vals[2])
        self._local_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorNode()
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
