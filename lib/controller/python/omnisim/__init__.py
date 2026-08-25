# Copyright 1996-2024 Cyberbotics Ltd.
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
#
# Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

"""OmniSim's Python controller API -- the canonical implementation.

This package holds the real controller runtime (Robot, Supervisor, the
device classes and the ``wb`` CFFI/ctypes binding). Import it as:

    from omnisim import Robot, Supervisor

``omnisim`` is the only module name. The legacy ``controller`` package --
a shim that re-exported these names and aliased every submodule -- was
removed on 2026-08-16, so ``from controller import Robot`` now raises
``ModuleNotFoundError``. Port a pre-rename controller by rewriting
``controller`` to ``omnisim`` in its import line; nothing else changes,
because the shim never defined anything of its own.
"""

from omnisim.field import Field                             # noqa
from omnisim.proto import Proto                             # noqa
from omnisim.node import Node, ContactPoint                 # noqa
from omnisim.ansi_codes import AnsiCodes                    # noqa
from omnisim.accelerometer import Accelerometer             # noqa
from omnisim.altimeter import Altimeter                     # noqa
from omnisim.brake import Brake                             # noqa
from omnisim.camera import Camera, CameraRecognitionObject  # noqa
from omnisim.compass import Compass                         # noqa
from omnisim.connector import Connector                     # noqa
from omnisim.display import Display                         # noqa
from omnisim.distance_sensor import DistanceSensor          # noqa
from omnisim.emitter import Emitter                         # noqa
from omnisim.gps import GPS                                 # noqa
from omnisim.gyro import Gyro                               # noqa
from omnisim.inertial_unit import InertialUnit              # noqa
from omnisim.led import LED                                 # noqa
from omnisim.lidar import Lidar                             # noqa
from omnisim.lidar_point import LidarPoint                  # noqa
from omnisim.light_sensor import LightSensor                # noqa
from omnisim.motor import Motor                             # noqa
from omnisim.position_sensor import PositionSensor          # noqa
from omnisim.radar import Radar                             # noqa
from omnisim.radar_target import RadarTarget                # noqa
from omnisim.range_finder import RangeFinder                # noqa
from omnisim.receiver import Receiver                       # noqa
from omnisim.robot import Robot                             # noqa
from omnisim.skin import Skin                               # noqa
from omnisim.speaker import Speaker                         # noqa
from omnisim.supervisor import Supervisor                   # noqa
from omnisim.touch_sensor import TouchSensor                # noqa
from omnisim.vacuum_gripper import VacuumGripper            # noqa
from omnisim.keyboard import Keyboard                       # noqa
from omnisim.mouse import Mouse                             # noqa
from omnisim.mouse import MouseState                        # noqa
from omnisim.joystick import Joystick                       # noqa
from omnisim.motion import Motion                           # noqa

__all__ = [
    "Accelerometer", "Altimeter", "AnsiCodes", "Brake", "Camera",
    "CameraRecognitionObject", "Compass", "Connector", "ContactPoint",
    "Display", "DistanceSensor", "Emitter", "Field", "GPS", "Gyro",
    "InertialUnit", "Joystick", "Keyboard", "LED", "Lidar", "LidarPoint",
    "LightSensor", "Motion", "Motor", "Mouse", "MouseState", "Node",
    "PositionSensor", "Proto", "Radar", "RadarTarget", "RangeFinder",
    "Receiver", "Robot", "Skin", "Speaker", "Supervisor", "TouchSensor",
    "VacuumGripper",
]
