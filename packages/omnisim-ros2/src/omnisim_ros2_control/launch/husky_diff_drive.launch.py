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

"""ros2_control on the OmniSim Husky, in one command (Tier 3).

    ros2 launch omnisim_ros2_control husky_diff_drive.launch.py

Prerequisites -- both on the OmniSim side, neither started by this file:

    python -m omnisim harness --auto-port
    curl -X POST http://127.0.0.1:6789/world/load -H 'Content-Type: application/json' \\
      -d '{"path":"projects/samples/demos/worlds/chat/omnilink_husky.omniworld","light":true}'

That world's robot runs the ``omnilink_mobile_bridge`` controller, which is what
puts the command surface on ``127.0.0.1:8765``.

Starts:
  * ``robot_state_publisher``  -- the description, so tf and RViz resolve
  * ``clock_node``             -- /clock, because everything below is on sim time
  * ``ros2_control_node``      -- controller_manager + the OmniSim hardware plugin
  * ``joint_state_broadcaster``
  * ``diff_drive_controller``  -- /diff_drive_controller/cmd_vel_unstamped -> OmniSim

Then::

    ros2 control list_hardware_interfaces
    ros2 control list_controllers
    ros2 topic pub -r 20 /diff_drive_controller/cmd_vel_unstamped \\
        geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

Arguments::

    harness_url:=http://127.0.0.1:6789
    bridge_url:=http://127.0.0.1:8765
    robot_def:=HUSKY          the DEF the harness knows the robot by (GET /robots)
    comms_rate_hz:=25.0       the plugin's HTTP rate -- the REAL control bandwidth
    rviz:=false

⚠ DO NOT ALSO RUN ``omnisim_bringup.launch.py`` AGAINST THE SAME ROBOT. Three
collisions, all silent:

  * ``diff_drive_controller`` and Tier 2's ``odom_node`` both publish
    ``odom -> base_link``;
  * ``joint_state_broadcaster`` and ``robot_state_node`` both publish joint
    states, and worse, both poll ``GET /robot/<def>/joints``. The harness
    finite-differences velocity between ITS OWN successive reads, so two
    independent pollers halve each other's dt and each sees a velocity that is
    a function of the other's polling;
  * ``command_node`` forwards ``/cmd_vel`` straight to the bridge, fighting the
    controller for the same actuator.

If you want the Tier 1 ``simulation_interfaces`` services alongside this, run
``ros2 launch omnisim_ros2 simulation_interfaces.launch.py`` -- that node is
service-only and shares nothing with the control loop.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGS = [
    ("harness_url", "http://127.0.0.1:6789", "Base URL of the OmniSim World Harness."),
    ("bridge_url", "http://127.0.0.1:8765", "Base URL of the Husky's own bridge."),
    ("robot_def", "HUSKY", "DEF name the harness knows the robot by (GET /robots)."),
    ("comms_rate_hz", "25.0", "Plugin HTTP rate. This, not update_rate, is the bandwidth."),
    ("clock_rate_hz", "20.0", "/clock publish rate. One HTTP request per tick."),
    ("rviz", "false", "Also start RViz2."),
]


def generate_launch_description() -> LaunchDescription:
    cfg = {name: LaunchConfiguration(name) for name, _, _ in ARGS}

    description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("omnisim_ros2_control"),
            "description",
            "husky_omnisim.urdf.xacro",
        ]),
        " harness_url:=", cfg["harness_url"],
        " bridge_url:=", cfg["bridge_url"],
        " robot_def:=", cfg["robot_def"],
        " comms_rate_hz:=", cfg["comms_rate_hz"],
    ])
    # `Command` yields a plain str; ParameterValue keeps launch from guessing a
    # type for a multi-kilobyte XML blob (it would otherwise try yaml).
    robot_description = {"robot_description": ParameterValue(description, value_type=str)}

    controllers = PathJoinSubstitution([
        FindPackageShare("omnisim_ros2_control"), "config", "husky_controllers.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # ⚠ robot_description as a controller_manager PARAMETER. Newer
        # ros2_control prefers the /robot_description topic and warns about
        # this, but the parameter path is the one that works on every Humble
        # patch release, including the ones that predate the topic support.
        parameters=[robot_description, controllers],
        output="screen",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    clock = Node(
        package="omnisim_ros2",
        executable="clock_node",
        name="omnisim_clock",
        output="screen",
        # No use_sim_time here, deliberately: this node PRODUCES it.
        parameters=[{
            "harness_url": cfg["harness_url"],
            "publish_rate_hz": cfg["clock_rate_hz"],
        }],
    )

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", *extra],
            output="screen",
        )

    jsb = spawner("joint_state_broadcaster")
    diff = spawner("diff_drive_controller")

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [
            clock,
            robot_state_pub,
            control_node,
            # Serialise the spawners: two `ros2 control load_controller` calls
            # racing on a controller manager that is itself still activating
            # hardware is a documented flake, and here the hardware's activation
            # blocks on a real HTTP round trip to the harness.
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[diff])),
            jsb,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(cfg["rviz"]),
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
