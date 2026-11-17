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

"""A URDF link with <inertial> but no <collision> must not collide.

The defect this pins (2026-10-06, found running companies' own published robots):
the importer gives such a link a 1 cm sphere boundingObject, because the engine
needs SOME boundingObject on a body (without one it falls back to a 12 cm
sphere). The sphere collided. URDF semantics -- and MuJoCo's own URDF import --
give the link no contact at all. Measured before the fix: LimX HU_D04's arm
stopped against a box at -43.98 deg (plain MuJoCo: -67.6, the hand passes into
it), and TRON1 stood on the spheres under its feet with 12.03 N*m at the knee
(MuJoCo: 0.54). After: -67.57 deg and 0.5425519 N*m.

The importer now marks the sphere `DEF URDF_INERTIA_PLACEHOLDER`, and the engine
keeps it on the body (its mass role is untouched) with collision switched off.

Probe: a 0.2 m box robot with three inertial-only "feet" fixed 0.05 m below its
bottom face, dropped on a floor. With the fix the box rests on the floor (root
z = 0.10); on colliding placeholders it stands on the three 1 cm spheres
(root z ~= 0.16). OMNISIM_NEWTON_URDF_PLACEHOLDER_COLLIDES=1 restores that.

    python -m pytest tests/test_newton_urdf_inertia_placeholder.py -v

OMNISIM_BINARY selects a scratch build. Three engine launches, a few seconds each.
"""

from __future__ import annotations

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

BOX_REST_Z = 0.10        # half the 0.2 m box: resting on its own collider
FEET_REST_Z = 0.16       # 0.15 m to the feet + the 1 cm placeholder radius
TOL_M = 0.01


def _inertial(mass, i=0.01):
    return (f'<inertial><mass value="{mass}"/><inertia ixx="{i}" iyy="{i}" izz="{i}" '
            f'ixy="0" ixz="0" iyz="0"/></inertial>')


URDF = ('<?xml version="1.0"?>\n<robot name="probe">\n'
        f'<link name="base">{_inertial(5)}'
        '<collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>\n'
        + "".join(f'<link name="foot{i}">{_inertial(0.1, 1e-4)}</link>\n'
                  f'<joint name="f{i}" type="fixed"><parent link="base"/><child link="foot{i}"/>'
                  f'<origin xyz="{x} {y} -0.15" rpy="0 0 0"/></joint>\n'
                  for i, (x, y) in enumerate(((0.08, 0.0), (-0.06, 0.07), (-0.06, -0.07))))
        + "</robot>\n")

WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -3 0 1 }
Solid { translation 0 0 -0.5 boundingObject Box { size 10 10 1 } }
URDFRobot { url "probe.urdf" translation 0 0 0.4 name "probe" controller "<none>" }
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


def _rest_z(tmp_path, tag, extra_env=None, urdf=None):
    d = tmp_path / tag
    d.mkdir()
    (d / "probe.urdf").write_text(urdf or URDF, encoding="utf-8")
    (d / "probe.omniworld").write_text(WORLD, encoding="utf-8")
    traj, log = d / "traj.tsv", d / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_MS="2000", **(extra_env or {}))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(d / "probe.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    for sig in _BRINGUP_SIGNATURES:
        if sig in text:
            pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
    rows = [line.split("\t") for line in traj.read_text(encoding="utf-8").splitlines()] if traj.is_file() else []
    rows = [r for r in rows if len(r) == 6 and r[1] == "probe"]
    assert rows, "the probe recorded no trajectory:\n" + text[-1500:]
    return float(rows[-1][4]), text


def test_inertial_only_links_do_not_collide(tmp_path):
    z, log = _rest_z(tmp_path, "fixed")
    assert abs(z - BOX_REST_Z) < TOL_M, (
        "the box must rest on its own collider at z=%.2f; it rests at %.4f -- the inertial-only "
        "feet are still colliding (1 cm placeholder spheres)" % (BOX_REST_Z, z))
    assert "URDF inertia placeholder" in log, "the engine should say how many placeholders it quieted"


MAVIC_URDF = ('<?xml version="1.0"?>\n<robot name="probe">\n'
              f'<link name="base">{_inertial(1)}'
              '<collision><geometry><box size="0.3 0.1 0.05"/></geometry></collision></link>\n'
              + "".join(f'<link name="prop{i}">{_inertial(0.01, 1e-5)}</link>\n'
                        f'<joint name="p{i}" type="continuous"><parent link="base"/><child link="prop{i}"/>'
                        f'<origin xyz="{x} 0 -0.032" rpy="0 0 0"/><axis xyz="0 0 1"/>'
                        '<limit effort="1" velocity="100"/></joint>\n'
                        for i, x in enumerate((0.12, -0.12)))
              + "</robot>\n")


def test_robot_whose_only_child_links_are_inertial_only_rests_on_its_own_collider(tmp_path):
    # DJI Mavic 2 Pro shape: the root link has the only <collision>; its propellers
    # (and gimbal) have none, 7 mm below the belly. The wrapper policy skips a Robot
    # root's own collider whenever a descendant has one -- and the quieted spheres
    # must not count, or the craft rests on the 1 mm wrapper sphere at its origin
    # (measured z=0.001 on the first cut of this fix, the box half sunk).
    z, _ = _rest_z(tmp_path, "mavic", urdf=MAVIC_URDF)
    assert abs(z - 0.025) < 0.005, (
        "the body must rest on its own 0.05 m box at z=0.025; got %.4f" % z)


def test_hatch_restores_colliding_placeholders(tmp_path):
    z, _ = _rest_z(tmp_path, "hatch", {"OMNISIM_NEWTON_URDF_PLACEHOLDER_COLLIDES": "1"})
    assert abs(z - FEET_REST_Z) < TOL_M, (
        "OMNISIM_NEWTON_URDF_PLACEHOLDER_COLLIDES=1 must restore the old contact (standing on the "
        "feet at z~=%.2f); got %.4f" % (FEET_REST_Z, z))
