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

"""A Solid's own low friction is used, and a URDF <gazebo> <mu1>/<mu2> reaches it.

The defects this pins (2026-10-10):

1. MuJoCo combines two geoms' friction by taking the LARGER unless one has a
   higher priority, so Solid.newtonFriction could only ever RAISE friction: a box
   at newtonFriction 0 on a 20 deg ramp at newtonGroundMu 1 stuck exactly like a
   box at 1. A shape that states its own friction now gets geom priority 1 and
   its value wins against unstated surfaces (OMNISIM_NEWTON_FRICTION_PRIORITY=0
   restores the max rule).
2. The URDF importer ignored <gazebo reference="LINK"> <mu1>/<mu2>, so casters
   Gazebo models at 0.2 dragged at the world's friction (Neobotix MP-400: 0.48 of
   commanded travel and turning the wrong way). They now become the link's
   newtonFriction (the smaller of the two; values above 2 import as 2, because
   MuJoCo's cone degenerates at Gazebo's "never slip" 100).

3. Cylinder and Capsule colliders took the world friction whatever the Solid
   stated (only Box, Sphere and mesh colliders received it), so the URDF wheel
   tags of Husky and Jackal were inert. OMNISIM_NEWTON_ROUND_SOLID_FRICTION=0
   restores that.
4. A stated friction of exactly 0 LAUNCHED the body (~850 m/s along the ramp
   normal): it is floored at 0.001. A bare "> 0.3 m" bound passed that launch,
   so every slide below is bounded on both sides by the closed form: frictionless
   on 20 deg for 1.2 s is 0.5 * g * sin(20) * 1.2^2 * cos(20) = 2.27 m along x.

Probe: on one ramp, a plain box and a plain cylinder (stick); a box, an upright
Cylinder and a Capsule lying down the slope at newtonFriction 0 (slide); and a
one-link URDF robot whose box link carries <gazebo><mu1 value="0"/> (slides).

    python -m pytest tests/test_newton_stated_friction_wins.py -v

OMNISIM_BINARY selects a scratch build. Two engine launches, a few seconds each.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

# The tagged link is a fixed CHILD (a skid under a collider-less body), as casters and
# wheels always are: the robot root is a Robot node, which has no newtonFriction field.
URDF = """<?xml version="1.0"?>
<robot name="slider">
  <link name="body">
    <inertial><mass value="1.0"/><inertia ixx="0.002" iyy="0.002" izz="0.002" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="skid">
    <inertial><mass value="0.1"/><inertia ixx="0.0002" iyy="0.0002" izz="0.0002" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="skid_mount" type="fixed"><parent link="body"/><child link="skid"/></joint>
  <gazebo reference="skid"><mu1 value="0"/><mu2 value="0"/></gazebo>
</robot>
"""


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")


def _world(urdf: Path) -> str:
    return f"""#VRML_SIM R2025a utf8
WorldInfo {{ basicTimeStep 4 newtonGroundMu 1.0 }}
Viewpoint {{ orientation 0 0 1 0 position -3 0 1 }}
Solid {{ rotation 0 1 0 0.349 name "RAMP" boundingObject Box {{ size 4 3 0.1 }} }}
Solid {{ translation -0.6 -1.0 0.33 rotation 0 1 0 0.349 name "STICKY"
  boundingObject Box {{ size 0.1 0.1 0.1 }} physics Physics {{ density 500 }} }}
Solid {{ translation -0.6 0.0 0.33 rotation 0 1 0 0.349 name "SLIPPERY" newtonFriction 0.0
  boundingObject Box {{ size 0.1 0.1 0.1 }} physics Physics {{ density 500 }} }}
Solid {{ translation -0.6 -1.3 0.33 rotation 0 1 0 0.349 name "STICKY_CYL"
  boundingObject Cylinder {{ radius 0.05 height 0.1 }} physics Physics {{ density 500 }} }}
Solid {{ translation -0.6 -0.5 0.33 rotation 0 1 0 0.349 name "SLIPPERY_CYL" newtonFriction 0.0
  boundingObject Cylinder {{ radius 0.05 height 0.1 }} physics Physics {{ density 500 }} }}
Solid {{ translation -0.6 0.5 0.33 rotation 0 1 0 1.9198 name "SLIPPERY_CAP" newtonFriction 0.0
  boundingObject Capsule {{ radius 0.04 height 0.1 }} physics Physics {{ density 500 }} }}
URDFRobot {{ url "{urdf.as_posix()}" translation -0.6 1.0 0.33 rotation 0 1 0 0.349
  name "URDF_SLIDER" controller "<none>" }}
"""


def _travel(tmp_path: Path, priority: str | None, round_friction: str | None = None) -> dict:
    urdf = tmp_path / "slider.urdf"
    urdf.write_text(URDF, encoding="utf-8")
    world = tmp_path / "ramp.omniworld"
    world.write_text(_world(urdf), encoding="utf-8")
    tag = (priority or "default") + ("_round" + round_friction if round_friction else "")
    traj = tmp_path / f"traj_{tag}.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(tmp_path / f"log_{tag}.txt"),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_MS="1200")
    for k in ("OMNISIM_NEWTON_FRICTION_PRIORITY", "OMNISIM_URDF_GAZEBO_FRICTION",
              "OMNISIM_NEWTON_ROUND_SOLID_FRICTION", "OMNISIM_NEWTON_CYLINDER_NATIVE"):
        env.pop(k, None)
    if priority is not None:
        env["OMNISIM_NEWTON_FRICTION_PRIORITY"] = priority
    if round_friction is not None:
        env["OMNISIM_NEWTON_ROUND_SOLID_FRICTION"] = round_friction
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=120, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    first, last = {}, {}
    if traj.is_file():
        for line in traj.read_text(encoding="utf-8").splitlines():
            c = line.split("\t")
            if len(c) >= 5:
                first.setdefault(c[1], float(c[2]))
                last[c[1]] = float(c[2])
    return {n: last[n] - first[n] for n in last}


def _slid(x: float) -> bool:
    # Frictionless closed form 2.27 m; a launched body travels hundreds of metres.
    return 2.0 < x < 2.5


def test_stated_low_friction_slides_and_gazebo_mu_reaches_the_link(tmp_path):
    d = _travel(tmp_path, None)
    for name in ("STICKY", "SLIPPERY", "URDF_SLIDER", "STICKY_CYL", "SLIPPERY_CYL", "SLIPPERY_CAP"):
        assert name in d, f"{name} missing from the trajectory probe: {sorted(d)}"
    assert abs(d["STICKY"]) < 0.02, f"the mu=1 box should stick, moved {d['STICKY']:.3f} m"
    assert _slid(d["SLIPPERY"]), f"the newtonFriction 0 box should slide ~2.27 m, moved {d['SLIPPERY']:.3f} m"
    assert _slid(d["URDF_SLIDER"]), (
        f"the URDF link with <gazebo> mu 0 should slide ~2.27 m, moved {d['URDF_SLIDER']:.3f} m")
    assert abs(d["STICKY_CYL"]) < 0.02, f"the mu=1 cylinder should stick, moved {d['STICKY_CYL']:.3f} m"
    assert _slid(d["SLIPPERY_CYL"]), (
        f"a Cylinder collider at newtonFriction 0 should slide ~2.27 m, moved {d['SLIPPERY_CYL']:.3f} m")
    assert _slid(d["SLIPPERY_CAP"]), (
        f"a Capsule collider at newtonFriction 0 should slide ~2.27 m, moved {d['SLIPPERY_CAP']:.3f} m")


def test_priority_hatch_restores_the_max_rule(tmp_path):
    d = _travel(tmp_path, "0")
    assert abs(d["SLIPPERY"]) < 0.02, f"with OMNISIM_NEWTON_FRICTION_PRIORITY=0 the max rule holds it, moved {d['SLIPPERY']:.3f} m"


def test_round_friction_hatch_restores_the_world_value(tmp_path):
    d = _travel(tmp_path, None, round_friction="0")
    assert abs(d["SLIPPERY_CYL"]) < 0.02, (
        f"with OMNISIM_NEWTON_ROUND_SOLID_FRICTION=0 the cylinder takes the world mu, moved {d['SLIPPERY_CYL']:.3f} m")
    assert _slid(d["SLIPPERY"]), f"the hatch must not touch Box colliders, moved {d['SLIPPERY']:.3f} m"
