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

"""The desk robot's motor model: a DC motor brakes above no-load speed, it does not go free.

Until 2026-11-03 the drive controller set the available torque to
max(0, stall x (1 - |w| / no-load)). setAvailableTorque(0) is a PASSIVE joint, so a wheel
that crossed no-load speed went free. In the spin, the left wheel crossed first, the right
kept driving (0.0066 N.m, ~0.43 N of traction) and swung the robot round the free wheel,
which the ground back-drove to 58 rad/s (motor limit 34.56) while the robot yawed at
17.6 rad/s (13.87 predicted). Not an engine fault: the solver's own record showed the free
wheel carrying 0.0000 N.m of actuator torque, the yaw acceleration matched the right wheel's
traction (21.6 predicted, 22 rad/s^2 measured), and setting both wheels passive made the yaw
DECAY. tests/test_desk_robot_drive.py checks the spin itself on the engine.
"""
import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "projects/samples/demos/controllers/desk_robot_drive/desk_robot_drive.py"
MAX_VELOCITY = 34.557519          # the world's RotationalMotor maxVelocity


def _controller():
    """Import the drive controller without the controller library (it only resolves in a run)."""
    import types
    stub = types.ModuleType("omnisim")
    stub.Supervisor = type("Supervisor", (), {})
    saved = sys.modules.get("omnisim")
    sys.modules["omnisim"] = stub
    try:
        spec = importlib.util.spec_from_file_location("_desk_robot_drive_probe", CONTROLLER)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        if saved is not None:
            sys.modules["omnisim"] = saved
        else:
            sys.modules.pop("omnisim", None)


def test_the_motor_model_brakes_above_no_load_speed():
    c = _controller()
    w0, stall = c.NO_LOAD_RAD_S, c.STALL_NM
    assert c.available_torque(1.0, 0.0) == pytest.approx(stall)
    assert c.available_torque(-1.0, -0.5 * w0) == pytest.approx(0.5 * stall)
    # over no-load the DC line is a brake, not a free wheel: the servo aims at no-load
    # speed, so this magnitude acts against the over-speed
    assert c.available_torque(-1.0, -1.5 * w0) == pytest.approx(0.5 * stall)
    assert c.available_torque(1.0, 1.1 * w0) > 0.0
    assert c.available_torque(0.0, 2 * w0) == pytest.approx(stall)    # stopping: full torque
    assert w0 == pytest.approx(MAX_VELOCITY, rel=1e-6)


