"""Bring up the Nav2 navigation stack (and optionally AMCL) for the OmniSim Husky.

This convenience launcher includes the stock ``nav2_bringup`` launch files and
feeds them OmniSim's tuned params, running on simulation time. It always starts
the navigation stack (planner, controller, behaviors, BT navigator, etc.) and
can optionally also start AMCL localization against a saved map.

Two supported flows:

    (A) SLAM-first (default):
        1. ros2 launch /abs/path/to/omnisim_slam.launch.py
           (slam_toolbox provides a live ``map -> odom`` transform)
        2. ros2 launch /abs/path/to/omnisim_nav2_bringup.launch.py \\
               localization:=false
        No AMCL and no saved map are needed; slam_toolbox owns ``map -> odom``.

    (B) Saved-map + AMCL (no SLAM):
        ros2 launch /abs/path/to/omnisim_nav2_bringup.launch.py \\
            localization:=true map:=/path/to/map.yaml
        The ``nav2_bringup`` localization stack (map_server + AMCL) provides the
        ``map -> odom`` transform from the saved map.

Prerequisite: a running OmniSim Tier-2 bringup must already be up
(``ros2 launch omnisim_ros2 omnisim_bringup.launch.py``), publishing ``/scan``,
``/odom``, ``/tf`` and ``/clock``. Nav2's ``/cmd_vel`` output is consumed by that
bringup's command_node to drive the simulated Husky.

Because this file is meant to be launched by absolute path and is NOT part of an
installed ament package, it resolves its sibling params file relative to
``__file__``. It still relies on the installed ``nav2_bringup`` package (located
via ``FindPackageShare``) for the stock launch files.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "params",
            "omnisim_nav2_params.yaml",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    localization = LaunchConfiguration("localization")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (OmniSim /clock) time if true.",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the Nav2 params YAML file to load.",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start and configure the Nav2 lifecycle nodes.",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Path to a saved map YAML file; only used when localization:=true.",
    )

    declare_localization = DeclareLaunchArgument(
        "localization",
        default_value="false",
        description="If true, also start AMCL localization via nav2_bringup.",
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
            )
        ),
        condition=IfCondition(localization),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "map": map_yaml,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            declare_autostart,
            declare_map,
            declare_localization,
            navigation_launch,
            localization_launch,
        ]
    )
