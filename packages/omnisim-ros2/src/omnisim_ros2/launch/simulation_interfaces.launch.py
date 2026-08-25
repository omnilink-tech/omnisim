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

"""Launch the simulation_interfaces server alone (Tier 1).

    ros2 launch omnisim_ros2 simulation_interfaces.launch.py
    ros2 launch omnisim_ros2 simulation_interfaces.launch.py harness_url:=http://127.0.0.1:6889

Requires a running OmniSim harness. Start one with:

    python -m omnisim harness --auto-port

For the full surface (clock, TF, joint states, cmd_vel) use
``omnisim_bringup.launch.py`` instead, which includes this node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    harness_url = LaunchConfiguration("harness_url")
    request_timeout_s = LaunchConfiguration("request_timeout_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "harness_url",
                default_value="http://127.0.0.1:6789",
                description="Base URL of the OmniSim World Harness.",
            ),
            DeclareLaunchArgument(
                "request_timeout_s",
                default_value="30.0",
                description=(
                    "HTTP timeout for harness calls. Raise it for cold, "
                    "asset-heavy world loads."
                ),
            ),
            Node(
                package="omnisim_ros2",
                executable="simulation_interfaces_node",
                name="omnisim_simulation_interfaces",
                output="screen",
                parameters=[
                    {
                        "harness_url": harness_url,
                        "request_timeout_s": request_timeout_s,
                    }
                ],
            ),
        ]
    )
