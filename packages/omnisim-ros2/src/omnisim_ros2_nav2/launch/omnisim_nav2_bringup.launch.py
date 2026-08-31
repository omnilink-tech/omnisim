"""Full Nav2 navigation stack (and optionally AMCL) for the OmniSim Husky.

Includes the stock `nav2_bringup` launch files with OmniSim's tuned params, on sim time.
NOTE: Jazzy's `navigation_launch.py` bundles optional nodes (collision_monitor,
docking_server, route_server, waypoint_follower) whose lifecycle bringup can be brittle
under load — for a robust, minimal navigation stack use `omnisim_nav2_lean.launch.py`
instead (that is the launch that reached M6).

Two supported flows:

    (A) SLAM-first (default):
        1. ros2 launch omnisim_ros2_nav2 omnisim_slam.launch.py
        2. ros2 launch omnisim_ros2_nav2 omnisim_nav2_bringup.launch.py localization:=false

    (B) Saved-map + AMCL (no SLAM):
        ros2 launch omnisim_ros2_nav2 omnisim_nav2_bringup.launch.py \
            localization:=true map:=/path/to/map.yaml

A running OmniSim Tier-2 bringup must publish /scan /odom /tf /clock; Nav2's /cmd_vel is
consumed by that bringup's command_node.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("omnisim_ros2_nav2"), "params", "omnisim_nav2_params.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    localization = LaunchConfiguration("localization")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    localization_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"])
        ),
        condition=IfCondition(localization),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "map": map_yaml,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("localization", default_value="false"),
        navigation,
        localization_inc,
    ])
