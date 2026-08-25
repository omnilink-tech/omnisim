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

"""Bring up the full OmniSim ROS 2 surface (Tiers 1 + 2) in one command.

    ros2 launch omnisim_ros2 omnisim_bringup.launch.py

Starts:
  * ``simulation_interfaces_node`` -- the standard's services + SimulateSteps
  * ``clock_node``                 -- /clock from the simulation clock
  * ``robot_state_node``           -- /tf, /tf_static, <robot>/joint_states
  * ``command_node``               -- cmd_vel + joint_command -> the robot bridge
  * ``odom_node``                  -- /odom + odom->base_link from the bridge

Requires a running OmniSim harness (``python -m omnisim harness --auto-port``)
with a world loaded. The bridge nodes additionally need that world's robot to be
running a bridge controller; the shipped example is::

    projects/samples/demos/worlds/chat/omnilink_husky.omniworld   (bridge on :8765)

Useful arguments::

    harness_url:=http://127.0.0.1:6789   the World Harness
    bridge_url:=http://127.0.0.1:8765    the per-robot bridge
    use_sim_time:=true                   make every node but the clock follow /clock
    bridge:=false                        harness-only: skip command_node + odom_node
    tf_root_only:=true                   cheap TF mode on a large scene

⚠ ``use_sim_time`` is applied to every node EXCEPT ``clock_node``, which is the
publisher of record and would deadlock waiting on its own output.

⚠ RATES COST TCP CONNECTIONS. Neither the harness nor the bridge sets
``protocol_version``, so both speak HTTP/1.0 and **close the connection after
every response** -- keep-alive is not available and each publish tick is a fresh
socket. The defaults here (20 + 5x3 + 10 ~= 45 requests/s) are chosen to sit well
inside that budget. Raise them only if you have measured headroom, and LOWER them
when running through ``tools/wsl_harness_link.py``, which doubles the
connection count on the Windows side.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = [
    ("harness_url", "http://127.0.0.1:6789", "Base URL of the OmniSim World Harness."),
    ("bridge_url", "http://127.0.0.1:8765", "Base URL of the per-robot bridge."),
    ("use_sim_time", "true", "Make nodes follow /clock (never applied to clock_node)."),
    ("bridge", "true", "Start the bridge-backed nodes (command_node, odom_node)."),
    ("clock_rate_hz", "20.0", "/clock publish rate. Each tick is one HTTP request."),
    ("state_rate_hz", "5.0", "TF + joint_states rate. Each tick is ~3 HTTP requests."),
    ("odom_rate_hz", "10.0", "/odom publish rate. Each tick is one HTTP request."),
    ("tf_root_only", "false", "Publish only root-level frames instead of the full tree."),
    ("odom_frame", "odom", "Odometry frame id."),
    ("base_frame", "base_link", "Robot base frame id."),
    ("sensors", "true", "Start sensor_node (Imu / LaserScan / GPS from the bridge)."),
    ("sensor_rate_hz", "10.0",
     "Sensor publish rate. Each tick is one HTTP request PER SENSOR."),
    ("lidar_layer", "-1",
     "Lidar layer to publish as /scan. -1 auto-picks the layer with the most "
     "returns and logs the choice."),
]


def generate_launch_description() -> LaunchDescription:
    cfg = {name: LaunchConfiguration(name) for name, _, _ in ARGS}
    use_bridge = IfCondition(cfg["bridge"])
    # Sensors need the bridge too, but are separately switchable: they are the
    # heaviest client (one request per sensor per tick).
    use_sensors = IfCondition(cfg["sensors"])

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [
            Node(
                package="omnisim_ros2",
                executable="simulation_interfaces_node",
                name="omnisim_simulation_interfaces",
                output="screen",
                parameters=[
                    {"harness_url": cfg["harness_url"], "use_sim_time": cfg["use_sim_time"]}
                ],
            ),
            Node(
                package="omnisim_ros2",
                executable="clock_node",
                name="omnisim_clock",
                output="screen",
                # No use_sim_time here, deliberately: this node produces it.
                parameters=[
                    {"harness_url": cfg["harness_url"], "publish_rate_hz": cfg["clock_rate_hz"]}
                ],
            ),
            Node(
                package="omnisim_ros2",
                executable="robot_state_node",
                name="omnisim_robot_state",
                output="screen",
                parameters=[
                    {
                        "harness_url": cfg["harness_url"],
                        "publish_rate_hz": cfg["state_rate_hz"],
                        "tf_root_only": cfg["tf_root_only"],
                        "use_sim_time": cfg["use_sim_time"],
                    }
                ],
            ),
            Node(
                package="omnisim_ros2",
                executable="command_node",
                name="omnisim_command",
                output="screen",
                condition=use_bridge,
                parameters=[
                    {"bridge_url": cfg["bridge_url"], "use_sim_time": cfg["use_sim_time"]}
                ],
            ),
            Node(
                package="omnisim_ros2",
                executable="odom_node",
                name="omnisim_odom",
                output="screen",
                condition=use_bridge,
                parameters=[
                    {
                        "bridge_url": cfg["bridge_url"],
                        "publish_rate_hz": cfg["odom_rate_hz"],
                        "odom_frame": cfg["odom_frame"],
                        "base_frame": cfg["base_frame"],
                        "use_sim_time": cfg["use_sim_time"],
                    }
                ],
            ),
            Node(
                package="omnisim_ros2",
                executable="sensor_node",
                name="omnisim_sensors",
                output="screen",
                condition=use_sensors,
                parameters=[
                    {
                        "bridge_url": cfg["bridge_url"],
                        "publish_rate_hz": cfg["sensor_rate_hz"],
                        "base_frame": cfg["base_frame"],
                        "lidar_layer": cfg["lidar_layer"],
                        "use_sim_time": cfg["use_sim_time"],
                    }
                ],
            ),
        ]
    )
