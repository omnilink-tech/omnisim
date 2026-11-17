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

"""A moving joint whose PARENT is an empty URDF frame must load, and move exactly
as if the frame were folded into its ancestor.

The defect this pins (2026-10-06, found running companies' own published robots):
a URDF link with no <inertial> on a fixed joint imports as a Physics-less frame.
setSolidMerger() already walked through such frames (2026-09-20), but a moving
joint whose OWN parent was the frame resolved its parent body with
effectiveNewtonBodyIndex() alone, found no body, and the load FATALed:

    [newton-enforce] A joint's parent body 'fl_wheel_base_link' resolved to
    Newton but never registered a Newton body

Husarion Lynx (body_link -fixed-> fl_wheel_base_link -continuous-> wheel) and
every Dexmate Vega (torso_flip_link -fixed-> arm_center -revolute-> arm) could
not load. OmSolid::newtonJointParentBodySolid() now walks up through such frames
to the body that carries them, and the joint's anchor and axis are re-expressed
in that body's frame.

Each probe drops a robot with UNPOWERED joints (effort 0) so the links swing
under gravity, and records every link's world position per step with the
engine's OMNISIM_PROBE_TRAJ_ALL dump. The same robot written WITHOUT the empty
frame (its offset folded into the joint origin) must trace the same path. On
CPU MuJoCo the two were bitwise identical on 2026-10-06 (0.0 m over 376 steps,
0.4 m of travel); the bound below is 1e-6 m. A wrong anchor or axis moves the
arm by centimetres.

    python -m pytest tests/test_newton_joint_parent_empty_frame.py -v

OMNISIM_BINARY selects a scratch build. Five engine launches, a few seconds each.
"""

from __future__ import annotations

import os
import subprocess
from collections import defaultdict
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)
FATAL = "never registered a Newton body"
TOL_M = 1e-6
MIN_TRAVEL_M = 0.1


def _inertial(mass, i=0.05):
    return (f'<inertial><mass value="{mass}"/><inertia ixx="{i}" iyy="{i}" izz="{i}" '
            f'ixy="0" ixz="0" iyz="0"/></inertial>')


def _box(x, y, z):
    return f'<collision><geometry><box size="{x} {y} {z}"/></geometry></collision>'


def _link(name, body=""):
    return f'<link name="{name}">{body}</link>'


def _joint(name, kind, parent, child, xyz="0 0 0", rpy="0 0 0", axis="0 1 0"):
    limit = ('<limit lower="-3" upper="3" effort="0" velocity="10"/>' if kind == "revolute"
             else '<limit effort="0" velocity="10"/>' if kind == "continuous" else "")
    ax = f'<axis xyz="{axis}"/>' if kind != "fixed" else ""
    return (f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz}" rpy="{rpy}"/>{ax}{limit}</joint>')


_ARM = [_link("base", _inertial(10) + _box(.4, .4, .1)), _link("torso", _inertial(3) + _box(.1, .1, .4)),
        _link("arm", _inertial(1, .01) + _box(.3, .05, .05)),
        _joint("t", "revolute", "base", "torso", "0 0 0.3", axis="0 0 1")]
_CART = [_link("base", _inertial(10) + _box(.6, .4, .1)), _link("wheel", _inertial(1, .01) + _box(.2, .05, .2))]

PAIRS = {
    # Dexmate-shaped: the empty frame hangs below a joint child and is ROTATED, so
    # both the anchor and the axis need the re-expression.
    "under_joint_child_rotated": (
        _ARM + [_link("center"), _joint("c", "fixed", "torso", "center", "0 0 0.3", "1.5708 0 0"),
                _joint("a", "revolute", "center", "arm", "0.15 0 0", axis="0 0 1")],
        _ARM + [_joint("a", "revolute", "torso", "arm", "0.15 0 0.3", "1.5708 0 0", axis="0 0 1")],
        "arm"),
    # Husarion-shaped, two empty frames deep, directly under the robot root.
    "under_root_two_frames": (
        _CART + [_link("m1"), _link("m2"), _joint("m1", "fixed", "base", "m1", "0.2 0 0"),
                 _joint("m2", "fixed", "m1", "m2", "0 0.25 0"),
                 _joint("w", "continuous", "m2", "wheel", "0 0.05 0")],
        _CART + [_joint("w", "continuous", "base", "wheel", "0.2 0.3 0")],
        "wheel"),
}

WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -3 0 1 }
Solid { translation 0 0 -0.5 boundingObject Box { size 10 10 1 } }
URDFRobot { url "probe.urdf" translation 0 0 0.3 rotation 0 1 0 0.4 name "probe" controller "<none>" }
"""


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


def _run(tmp_path, tag, links, extra_env=None):
    d = tmp_path / tag
    d.mkdir()
    (d / "probe.urdf").write_text('<?xml version="1.0"?>\n<robot name="probe">\n' + "\n".join(links)
                                  + "\n</robot>\n", encoding="utf-8")
    (d / "probe.omniworld").write_text(WORLD, encoding="utf-8")
    traj, log = d / "traj.tsv", d / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_ALL="1", OMNISIM_PROBE_TRAJ_MS="1500",
               **(extra_env or {}))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(d / "probe.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    for sig in _BRINGUP_SIGNATURES:
        if sig in text:
            pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
    paths = defaultdict(dict)
    if traj.is_file():
        for line in traj.read_text(encoding="utf-8").splitlines():
            t, name, x, y, z, _ = line.split("\t")
            paths[name][float(t)] = (float(x), float(y), float(z))
    return paths, text


@pytest.mark.parametrize("case", sorted(PAIRS))
def test_joint_on_empty_frame_moves_like_the_folded_robot(tmp_path, case):
    with_frame, folded, moving = PAIRS[case]
    a, log = _run(tmp_path, "frame", with_frame)
    assert FATAL not in log, "the empty-frame robot must load:\n" + log[-1500:]
    b, _ = _run(tmp_path, "folded", folded)
    steps = sorted(set(a[moving]) & set(b[moving]))
    assert len(steps) > 100, "too few common steps to compare: %d" % len(steps)
    travel = max(abs(a[moving][steps[-1]][i] - a[moving][steps[0]][i]) for i in range(3))
    assert travel > MIN_TRAVEL_M, "the probe link barely moved (%.4f m); the test proves nothing" % travel
    for name in (moving, "probe"):
        worst = max(max(abs(p - q) for p, q in zip(a[name][t], b[name][t])) for t in steps)
        assert worst < TOL_M, (
            "%s: '%s' strays %.6f m from the same robot with the frame folded in -- the joint is "
            "anchored or oriented in the wrong body frame" % (case, name, worst))


def test_hatch_restores_the_old_fatal(tmp_path):
    with_frame, _, _ = PAIRS["under_root_two_frames"]
    _, log = _run(tmp_path, "hatch", with_frame, {"OMNISIM_NEWTON_JOINT_PARENT_FRAME_WALK": "0"})
    assert FATAL in log, "OMNISIM_NEWTON_JOINT_PARENT_FRAME_WALK=0 must restore the pre-fix behaviour"
