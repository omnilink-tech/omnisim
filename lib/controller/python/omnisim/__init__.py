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

"""OmniSim's Python controller API.

This is the canonical import path for new code:

    from omnisim import Robot, Supervisor

The legacy import ``from controller import Robot`` continues to work
unchanged -- this module simply re-exports everything from ``controller``
so both names resolve to the same classes. A future phase will flip the
implementation here and turn ``controller`` into the deprecation shim.
"""

from controller import (                                          # noqa: F401
    Accelerometer, Altimeter, AnsiCodes, Brake, Camera,
    CameraRecognitionObject, Compass, Connector, ContactPoint,
    Display, DistanceSensor, Emitter, Field, GPS, Gyro,
    InertialUnit, Joystick, Keyboard, LED, Lidar, LidarPoint,
    LightSensor, Motion, Motor, Mouse, MouseState, Node,
    PositionSensor, Proto, Radar, RadarTarget, RangeFinder,
    Receiver, Robot, Skin, Speaker, Supervisor, TouchSensor,
    VacuumGripper,
)

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
