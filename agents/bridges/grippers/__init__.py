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

# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Real-robot gripper drivers for OmniLink bridges.

Self-contained (no Webots / OmniSim-internal imports). The gripper ids
match the sim registry in `omnilink_arm_bridge/_gripper_configs.py`, so
`--gripper <id>` selects the same physical gripper in sim and on real
hardware.

    from grippers import make_driver, GRIPPER_SPECS
    g = make_driver("robotiq_2f85")     # DryTransport by default
    g.connect(); g.grasp(); g.state()

To talk to real hardware, build the driver with a concrete transport:

    from grippers.robotiq import Robotiq2FDriver
    from my_modbus import PymodbusTransport
    g = Robotiq2FDriver(max_width=0.085, transport=PymodbusTransport(host=...))
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .driver_base import RealGripperDriver, Transport, DryTransport
from .robotiq import Robotiq2FDriver, Robotiq3FDriver
from .vacuum import VacuumIODriver
from .onrobot import OnRobotRGDriver
from .schunk import SchunkEGxDriver
from .mock import MockGripperDriver

# id -> (driver class, default kwargs). Ids mirror the sim GRIPPER_CONFIGS.
GRIPPER_SPECS: Dict[str, Dict[str, Any]] = {
    "robotiq_2f85":  {"driver": Robotiq2FDriver, "model": "Robotiq 2F-85",  "max_width": 0.085},
    "robotiq_2f140": {"driver": Robotiq2FDriver, "model": "Robotiq 2F-140", "max_width": 0.140},
    "robotiq_3f":    {"driver": Robotiq3FDriver, "model": "Robotiq 3F",     "max_width": 0.155},
    "onrobot_rg2":   {"driver": OnRobotRGDriver, "model": "OnRobot RG2",    "max_width": 0.110},
    "onrobot_rg6":   {"driver": OnRobotRGDriver, "model": "OnRobot RG6",    "max_width": 0.160},
    "schunk_egk40":  {"driver": SchunkEGxDriver, "model": "Schunk EGK 40",  "max_width": 0.084},
    "vacuum":        {"driver": VacuumIODriver,  "model": "Vacuum Suction"},
    # No dedicated hardware driver yet -> Mock (keeps the surface working).
    "panda_hand":    {"driver": MockGripperDriver, "model": "Panda Hand",   "max_width": 0.08},
    "magnetic":      {"driver": MockGripperDriver, "model": "Magnetic Coupler"},
    "mock":          {"driver": MockGripperDriver, "model": "MockGripper",  "max_width": 0.08},
}


def make_driver(gripper_id: str, *, transport: Optional[Transport] = None,
                **overrides: Any) -> RealGripperDriver:
    """Build a real-gripper driver by id. Falls back to MockGripperDriver
    for unknown ids so a bridge never hard-fails on a typo -- the driver's
    state() reports the substitution via its model name."""
    spec = GRIPPER_SPECS.get(gripper_id)
    if spec is None:
        return MockGripperDriver(model=f"unknown:{gripper_id}", transport=transport)
    kwargs = {k: v for k, v in spec.items() if k != "driver"}
    kwargs.update(overrides)
    if transport is not None:
        kwargs["transport"] = transport
    return spec["driver"](**kwargs)


__all__ = [
    "make_driver", "GRIPPER_SPECS",
    "RealGripperDriver", "Transport", "DryTransport",
    "Robotiq2FDriver", "Robotiq3FDriver", "VacuumIODriver",
    "OnRobotRGDriver", "SchunkEGxDriver", "MockGripperDriver",
]
