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

"""The assembled desk robot on Newton: does it drive as its motors predict? (engine)

The spin checks pin the 2026-11-03 fix in test_desk_robot_motor_model.py: with the
controller's DC model clamped at zero above no-load speed, one wheel went free and was
back-driven to 58 rad/s (limit 34.56) while the robot yawed at 17.6 rad/s (13.87 predicted).
"""
import json
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/desk_robot"
sys.path.insert(0, str(PRODUCT))

import design  # noqa: E402
import robot as robot_mod  # noqa: E402

R = design.Robot()
MAX_VELOCITY = 34.557519          # the world's RotationalMotor maxVelocity


@pytest.mark.engine
def test_the_robot_drives_as_its_motors_predict(tmp_path):
    out = tmp_path / "drive.json"
    world = robot_mod.WORLD.parent / ".desk_robot_drive_pytest.omniworld"
    import os
    url = Path(os.path.relpath(robot_mod.MESHES, world.parent)).as_posix()
    world.write_text(robot_mod.world(R, url, [f"--result={out.as_posix()}", "--quit"]), encoding="utf-8")
    try:
        subprocess.run([sys.executable, "-m", "omnisim", "run-headless", str(world), "--duration", "300"],
                       cwd=ROOT, capture_output=True, timeout=600)
    finally:
        world.unlink(missing_ok=True)
    d = json.loads(out.read_text(encoding="utf-8"))
    p = robot_mod.predictions(R)
    assert 0.8 * p["no_load_speed_m_s"] < d["straight_speed_m_s"] <= p["no_load_speed_m_s"]
    assert d["straight_effective_radius_mm"] == pytest.approx(p["wheel_radius_mm"], abs=0.5)
    assert abs(d["heading_drift_deg_straight"]) < 2.0
    # the spin: no wheel past its motor's no-load speed (the peak is a 2 ms finite
    # difference of the position sensor, so it may read a hair over), and the yaw the
    # wheels' rolling predicts
    assert d["spin_wheel_rad_s"] <= MAX_VELOCITY
    assert d["spin_wheel_peak_rad_s"] <= 1.002 * MAX_VELOCITY
    assert 0.9 * p["spin_no_load_rad_s"] < d["spin_yaw_rate_rad_s"] <= 1.02 * p["spin_no_load_rad_s"]
