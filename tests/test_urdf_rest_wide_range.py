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

"""A URDF <rest> pose on a joint whose range reaches past +/-pi is read correctly.

The defect this pins (2026-10-07, found on the UFACTORY xArm lane 2026-10-04):
the importer writes a joint's child link ALREADY POSED at its initial angle
(the <rest> value, or the midpoint of a range that excludes 0), and must then
tell the joint it starts there (HingeJointParameters `position`). That line was
written only inside the branch for ranges that fit in (-pi, pi). A range that
reaches past +/-pi at one end (xArm joint3 -3.927..0.192, joint5
-1.693..3.14159) took the other branch, so the link was posed at the rest angle
while the joint believed it was at 0: the PositionSensor read 0 at the rest
pose, every setPosition() landed `rest` radians off, and the xArm controller
drove joint3 to a physical -245.6 deg, past the URDF's own -225 deg stop.

The second defect (2026-10-23, found on the Ti5 T170 lane 2026-10-19): a
<rest> on a joint whose limits cover a FULL TURN (+/-3.14, +/-3.1416, or wider)
was ignored with nothing logged -- the importer's full-revolution early return
came before it read <rest>, so the joint started at 0 (the T170 shoulders read
q_start 0.0 at +/-3.14 and the rest angle at +/-3.13). A `continuous` joint
dropped its <rest> the same way. Both now start at the rest angle.

The same run pins the effort="0" log: the importer's "passive" warning appears
ONCE (the world pre-scan used to expand every URDFRobot a second time, so every
importer warning was printed twice), and the engine no longer follows it with a
"SYNTHETIC position gain" warning claiming OmniSim drives the joint.

Probe: horizontal fingers on a fixed base, each with <rest>: a range inside
+/-pi (the control), one past -pi, one past +pi, three covering a full turn
(+/-3.14, +/-3.1416, +/-2 pi with a rest beyond pi) and a continuous joint.
Each finger carries a tip link 0.2 m out, whose world position gives the
PHYSICAL angle. The controller reads the sensors on the first step (they must
equal <rest>), then commands rest + 0.3 and checks sensor and physical angle
agree. One more finger is passive (effort="0") and is left alone.

    python -m pytest tests/test_urdf_rest_wide_range.py -v

OMNISIM_BINARY selects a scratch build. One engine launch, a few seconds.
"""

from __future__ import annotations

import math
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)
TOL = 0.02
STEP = 0.3

# name: (lower, upper, rest, y offset of the finger on the base); no limits = continuous
JOINTS = {
    "inside": (-1.5, 1.5, 0.7, 0.45),
    "past_lo": (-3.927, 0.192, -1.2, 0.3),
    "past_hi": (-1.693, 3.14159, 1.0, 0.15),
    "full_314": (-3.14, 3.14, 0.9, 0.0),
    "full_31416": (-3.1416, 3.1416, -0.8, -0.15),
    "full_2pi": (-6.2832, 6.2832, 4.0, -0.3),
    "spin": (None, None, -1.0, -0.45),
}
# A passive finger (effort="0"), for the log checks; nothing commands it.
PASSIVE = ("limp", -0.6)

_I = '<inertia ixx=".001" iyy=".001" izz=".001" ixy="0" ixz="0" iyz="0"/>'


def _urdf():
    parts = ['<?xml version="1.0"?>', '<robot name="rest_probe">',
             f'<link name="base"><inertial><mass value="2"/>{_I}</inertial></link>']
    for n, (lo, hi, rest, y) in JOINTS.items():
        kind, limits = ("continuous", "") if lo is None else ("revolute", f'lower="{lo}" upper="{hi}" ')
        parts += [
            f'<link name="{n}_finger"><inertial><origin xyz=".1 0 0"/><mass value=".2"/>{_I}</inertial></link>',
            f'<link name="{n}_tip"><inertial><mass value=".01"/>{_I}</inertial></link>',
            f'<joint name="{n}" type="{kind}"><parent link="base"/><child link="{n}_finger"/>'
            f'<origin xyz="0 {y} 0"/><axis xyz="0 1 0"/><limit {limits}effort="50" '
            f'velocity="3"/><rest>{rest}</rest></joint>',
            f'<joint name="{n}_tip_fix" type="fixed"><parent link="{n}_finger"/><child link="{n}_tip"/>'
            f'<origin xyz=".2 0 0"/></joint>',
        ]
    n, y = PASSIVE
    parts += [
        f'<link name="{n}_finger"><inertial><origin xyz=".1 0 0"/><mass value=".2"/>{_I}</inertial></link>',
        f'<joint name="{n}" type="revolute"><parent link="base"/><child link="{n}_finger"/>'
        f'<origin xyz="0 {y} 0"/><axis xyz="0 1 0"/><limit lower="-1.5" upper="1.5" effort="0" '
        f'velocity="3"/></joint>',
    ]
    parts.append("</robot>")
    return "\n".join(parts)


WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -2 0 1 }
URDFRobot { url "rest.urdf" translation 0 0 1 staticBase TRUE supervisor TRUE controller "rest_probe" }
"""

CONTROLLER = """import os
from omnisim import Supervisor
NAMES = %r
STEP = %r
r = Supervisor(); dt = int(r.getBasicTimeStep())
motors = {n: r.getDevice(n + "_motor") for n in NAMES}
sensors = {n: r.getDevice(n + "_sensor") for n in NAMES}
for s in sensors.values():
    s.enable(dt)
r.step(dt)
first = {n: sensors[n].getValue() for n in NAMES}
for n in NAMES:
    motors[n].setPosition(first[n] + STEP)
for _ in range(375):
    r.step(dt)
with open(os.environ["PROBE_OUT"], "w") as f:
    for n in NAMES:
        f.write("%%s %%.9g %%.9g %%.9g\\n" %% (n, first[n], first[n] + STEP, sensors[n].getValue()))
r.simulationQuit(0)
""" % (list(JOINTS), STEP)


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    root = tmp_path_factory.mktemp("rest_wide")
    worlds, ctrl = root / "worlds", root / "controllers" / "rest_probe"
    worlds.mkdir(parents=True)
    ctrl.mkdir(parents=True)
    (worlds / "rest.urdf").write_text(_urdf(), encoding="utf-8")
    (worlds / "rest.omniworld").write_text(WORLD, encoding="utf-8")
    (ctrl / "rest_probe.py").write_text(CONTROLLER, encoding="utf-8")
    out, log, traj = root / "out.txt", root / "engine.log", root / "traj.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), PROBE_OUT=str(out),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_ALL="1", OMNISIM_PROBE_TRAJ_MS="3000")
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(worlds / "rest.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not out.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
        pytest.fail("the rest probe produced no output:\n%s" % text[-1500:])
    readings = {}
    for line in out.read_text(encoding="utf-8").splitlines():
        n, first, target, final = line.split()
        readings[n] = (float(first), float(target), float(final))
    # Physical angle of each finger from its tip's last recorded world position
    # (joint axis +y: a finger at angle q puts its tip at x = 0.2 cos q, z = -0.2 sin q
    # relative to the joint).
    last = {}
    for row in traj.read_text(encoding="utf-8").splitlines() if traj.is_file() else []:
        r = row.split("\t")
        if len(r) == 6:
            last[r[1]] = (float(r[2]), float(r[4]))
    physical = {}
    for n in JOINTS:
        if n + "_tip" in last:
            x, z = last[n + "_tip"]
            physical[n] = math.atan2(-(z - 1.0), x)
    return readings, physical, text


def _wrapped(a):
    """An angle difference folded into [-pi, pi): a tip position cannot tell 4.3 from 4.3 - 2 pi."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@pytest.mark.parametrize("name", list(JOINTS))
def test_sensor_reads_rest_on_the_first_step(run, name):
    readings, _, _ = run
    rest = JOINTS[name][2]
    assert abs(readings[name][0] - rest) < TOL, (
        "%s: the PositionSensor must read the <rest> pose (%.3f) the link was spawned at; got %.4f"
        % (name, rest, readings[name][0]))


@pytest.mark.parametrize("name", list(JOINTS))
def test_sensor_and_physical_angle_agree(run, name):
    readings, physical, log = run
    _, target, final = readings[name]
    assert abs(final - target) < TOL, "%s: the joint must reach its command %.3f; sensor %.4f" % (
        name, target, final)
    assert name in physical, "no trajectory for %s_tip:\n%s" % (name, log[-1500:])
    assert abs(_wrapped(physical[name] - target)) < TOL, (
        "%s: commanded %.3f and the sensor agrees, but the link is physically at %.4f rad"
        % (name, target, physical[name]))


def test_passive_warning_is_printed_once(run):
    """The world pre-scan used to expand the URDF too, doubling every importer warning."""
    _, _, log = run
    n = PASSIVE[0]
    lines = [ln for ln in log.splitlines() if "URDF joint '%s'" % n in ln and "no torque" in ln]
    assert len(lines) == 1, "expected the passive warning for %r once, got %d:\n%s" % (
        n, len(lines), "\n".join(lines))


def test_no_synthetic_gain_warning_for_a_passive_joint(run):
    """effort="0" is an OFF motor: the engine must not then claim it drives the joint."""
    _, _, log = run
    assert "SYNTHETIC position gain" not in log, [ln for ln in log.splitlines() if "SYNTHETIC" in ln]
    stated = [ln for ln in log.splitlines() if "Motor '%s_motor'" % PASSIVE[0] in ln]
    assert all("passive" in ln for ln in stated), stated
