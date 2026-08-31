"""Launch slam_toolbox in online-async mapping mode for the OmniSim Husky.

This convenience launcher starts ``slam_toolbox``'s asynchronous online SLAM
node so the OmniSim Husky can build a map from its 2D laser scan. The node
subscribes to ``/scan`` and the transform tree, and publishes the ``map`` frame
plus the ``map -> odom`` correction transform (in addition to the occupancy
grid on ``/map``).

Prerequisite: a running OmniSim Tier-2 bringup must already be up
(``ros2 launch omnisim_ros2 omnisim_bringup.launch.py``), publishing ``/scan``,
``/odom``, ``/tf`` and ``/clock``. Because this file is meant to be launched by
absolute path and is NOT part of an installed ament package, it resolves its
sibling params file relative to ``__file__`` rather than via the ament index.

Example::

    ros2 launch /abs/path/to/omnisim_slam.launch.py

Combine with ``omnisim_nav2_bringup.launch.py localization:=false`` for the
SLAM-first navigation flow (live ``map -> odom`` from this node).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_slam_params_file = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "params",
            "mapper_params_online_async.yaml",
        )
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (OmniSim /clock) time if true.",
    )

    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_slam_params_file,
        description="Full path to the slam_toolbox params YAML file to load.",
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_slam_params_file,
            slam_toolbox_node,
        ]
    )
