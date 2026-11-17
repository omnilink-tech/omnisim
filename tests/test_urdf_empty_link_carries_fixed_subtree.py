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

"""An empty link on a MOVING joint must carry the links welded below it.

The defect this pins (2026-10-06, found running Kinova's Gen3 lite): a URDF link
that is the child of a revolute joint and declares neither <inertial> nor
<collision> imported with no Physics. It was a kinematic frame, so its joint had
no dynamic endpoint, and the links fixed below it -- whose merge walk stops at a
joint endpoint -- registered as a FREE body of their own. Measured on the probe
below: the 1 kg "gripper" fell away to z = -11 m while the arm swung on without
it. On the Gen3 lite (end_effector_link) the gripper and a 0.5 kg tool payload
vanished from the arm's torques: joint_2 held 8.675 N*m with or without the
payload; plain MuJoCo, which fuses fixed children into their parent body, needs
9.996 / 13.712.

The importer now gives such a link the same 1 g synthetic body it gives a
collision-only link (plus the non-colliding inertia placeholder sphere), so the
fixed subtree merges into it. The probe arm must then swing like the same arm
with the gripper's mass folded into the end-effector link: measured within
4.4e-5 m over 0.3 m of travel (the extra gram); the bound is 1e-3 m.
OMNISIM_URDF_EMPTY_LINK_BODY=0 restores the old import.

    python -m pytest tests/test_urdf_empty_link_carries_fixed_subtree.py -v

OMNISIM_BINARY selects a scratch build. Three engine launches, a few seconds each.
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
TOL_M = 1e-3


def _inertial(mass, i=0.01, xyz="0 0 0"):
    return (f'<inertial><origin xyz="{xyz}" rpy="0 0 0"/><mass value="{mass}"/><inertia ixx="{i}" '
            f'iyy="{i}" izz="{i}" ixy="0" ixz="0" iyz="0"/></inertial>')


def _box(x, y, z):
    return f'<collision><geometry><box size="{x} {y} {z}"/></geometry></collision>'


def _joint(name, kind, parent, child, xyz):
    extra = ('<axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="0" velocity="10"/>'
             if kind == "revolute" else "")
    return (f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz}" rpy="0 0 0"/>{extra}</joint>')


_ARM = [f'<link name="base">{_inertial(5)}{_box(.2, .2, .1)}</link>',
        f'<link name="l1">{_inertial(1, 0.01, "0.15 0 0")}{_box(.3, .04, .04)}</link>',
        _joint("j1", "revolute", "base", "l1", "0 0 0.1")]
EMPTY_EE = _ARM + ['<link name="ee"/>', f'<link name="grip">{_inertial(1, 0.001)}</link>',
                   _joint("j2", "revolute", "l1", "ee", "0.3 0 0"),
                   _joint("g", "fixed", "ee", "grip", "0.1 0 0")]
FOLDED = _ARM + [f'<link name="ee">{_inertial(1, 0.001, "0.1 0 0")}</link>',
                 _joint("j2", "revolute", "l1", "ee", "0.3 0 0")]

WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -3 0 1 }
URDFRobot { url "probe.urdf" translation 0 0 0.5 name "probe" staticBase TRUE controller "<none>" }
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
    assert paths.get("ee"), "the probe recorded no trajectory:\n" + text[-1500:]
    return paths


def test_empty_link_carries_its_fixed_subtree(tmp_path):
    a = _run(tmp_path, "empty", EMPTY_EE)
    b = _run(tmp_path, "folded", FOLDED)
    steps = sorted(set(a["ee"]) & set(b["ee"]))
    assert len(steps) > 100, "too few common steps to compare: %d" % len(steps)
    travel = max(abs(a["ee"][steps[-1]][i] - a["ee"][steps[0]][i]) for i in range(3))
    assert travel > 0.1, "the end-effector barely moved (%.4f m); the test proves nothing" % travel
    worst = max(max(abs(p - q) for p, q in zip(a["ee"][t], b["ee"][t])) for t in steps)
    assert worst < TOL_M, (
        "the empty end-effector strays %.6f m from the same arm with the gripper's mass folded in -- "
        "the fixed subtree is not carried by the joint's endpoint" % worst)
    gap = sum((p - q) ** 2 for p, q in zip(a["grip"][steps[-1]], a["ee"][steps[-1]])) ** 0.5
    assert abs(gap - 0.1) < TOL_M, "the gripper must stay 0.1 m from the end-effector; it is %.4f m" % gap


def test_hatch_restores_the_loose_subtree(tmp_path):
    a = _run(tmp_path, "hatch", EMPTY_EE, {"OMNISIM_URDF_EMPTY_LINK_BODY": "0"})
    end = a["grip"][max(a["grip"])]
    assert end[2] < -1.0, ("OMNISIM_URDF_EMPTY_LINK_BODY=0 must restore the old import, where the "
                           "gripper falls away as a free body; it ended at z=%.3f" % end[2])
