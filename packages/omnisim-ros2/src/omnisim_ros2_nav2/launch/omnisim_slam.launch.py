"""slam_toolbox online-async mapping for the OmniSim Husky.

Maps from /scan and publishes map->odom. Needs a running OmniSim Tier-2 bringup
(`ros2 launch omnisim_ros2 omnisim_bringup.launch.py`) already publishing
/scan /odom /tf /clock.

    ros2 launch omnisim_ros2_nav2 omnisim_slam.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_slam = PathJoinSubstitution(
        [FindPackageShare("omnisim_ros2_nav2"), "params", "mapper_params_online_async.yaml"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_params_file", default_value=default_slam),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
        ),
    ])
