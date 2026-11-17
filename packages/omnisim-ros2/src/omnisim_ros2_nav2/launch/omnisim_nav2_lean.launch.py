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

"""Lean Nav2 navigation launch for the OmniSim Husky (the launch that closed M6).

Jazzy's stock `nav2_bringup/navigation_launch.py` also starts `collision_monitor`,
`docking_server`, `route_server` and `waypoint_follower` — each ABORTS the whole
lifecycle bringup if it has no config. This launch instantiates ONLY the five core
servers needed to plan and drive, plus a `lifecycle_manager` over exactly those, so
the bringup can't be aborted by an unconfigured optional node.

cmd_vel routing is kept trivial: `controller_server` and `behavior_server` publish
`/cmd_vel` DIRECTLY (no velocity_smoother / collision_monitor in the chain), which is
exactly what OmniSim's `command_node` subscribes to — so no relay is needed.

Prerequisite: a running OmniSim Tier-2 bringup (publishing `/scan /odom /tf /clock`)
AND a `map→odom` transform. For OmniSim you don't need SLAM/AMCL: `/odom` is
simulator ground truth, so a static identity map→odom is enough:

    ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom \
        --ros-args -p use_sim_time:=true

Then:

    ros2 launch omnisim_ros2_nav2 omnisim_nav2_lean.launch.py use_sim_time:=true
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5}, orientation: {w: 1.0}}}}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("omnisim_ros2_nav2"), "params", "omnisim_nav2_params.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]
    common = [params_file, {"use_sim_time": use_sim_time}]

    servers = [
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=common),
        Node(package="nav2_smoother", executable="smoother_server",
             name="smoother_server", output="screen", parameters=common),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=common),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=common),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=common),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{
                 "use_sim_time": use_sim_time,
                 "autostart": autostart,
                 "node_names": lifecycle_nodes,
             }]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        *servers,
    ])
