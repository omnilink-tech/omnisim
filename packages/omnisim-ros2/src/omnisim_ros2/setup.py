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

from glob import glob

from setuptools import find_packages, setup

package_name = "omnisim_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OmniLink",
    maintainer_email="info@omnilink-agents.com",
    description=(
        "ROS 2 sidecar for the OmniSim simulator: the simulation_interfaces "
        "standard plus /clock, /tf, JointState and cmd_vel over OmniSim's HTTP surface."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simulation_interfaces_node = omnisim_ros2.simulation_interfaces_node:main",
            "clock_node = omnisim_ros2.clock_node:main",
            "robot_state_node = omnisim_ros2.robot_state_node:main",
            "command_node = omnisim_ros2.command_node:main",
            "odom_node = omnisim_ros2.odom_node:main",
            "sensor_node = omnisim_ros2.sensor_node:main",
        ],
    },
)
