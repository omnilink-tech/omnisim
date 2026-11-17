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

"""A Cylinder collider off the joint graph rests at its AUTHORED height.

The defect this pins (measured 2026-09-15, fixed 2026-10-09): every Cylinder
boundingObject reached the solver as a capsule stand-in (W1.2, chosen in June
because a native cylinder locked wheels on the XPBD pipeline of the time). When
radius >= half the height -- a disc, a puck, a table top, a chassis plate -- the
stand-in keeps a hemisphere of radius r at each end, so the body rested r above
where it was authored: an r=0.08 h=0.01 disc rested at z=0.0846 instead of
0.005, and a kiwi-drive chassis plate held all three wheels off the ground
while run-headless printed PASS.

Now a cylinder on a body that is not a joint's child (props, statics, robot
roots) is a real cylinder; one on a jointed link (wheels, arm links) keeps the
capsule, because there it is not neutral -- an open-loop Husky pivot turned
132 deg on capsules and 82 deg on real cylinders. OMNISIM_NEWTON_CYLINDER_NATIVE
=0 restores capsules everywhere, =1 makes every cylinder real.

Probe: free bodies dropped 2 mm onto the floor; the rest height of each centre
is read from the trajectory probe. Sphere and Box are controls. The =0 run
must reproduce the old capsule height, which proves the probe can see the bug.

    python -m pytest tests/test_newton_cylinder_collider_rest.py -v

OMNISIM_BINARY selects a scratch build. Two engine launches, ~10 s each.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

# name, geometry, rotation (axis-angle) or None, drop height of the centre, expected rest z
CASES = [
    ("SPHERE_REF", "Sphere { radius 0.033 }", None, 0.035, 0.033),
    ("BOX_REF", "Box { size 0.066 0.066 0.066 }", None, 0.035, 0.033),
    ("CYL_FLAT", "Cylinder { radius 0.080 height 0.010 }", None, 0.007, 0.005),
    ("CYL_Z", "Cylinder { radius 0.033 height 0.022 }", None, 0.013, 0.011),
    ("CYL_EQ", "Cylinder { radius 0.040 height 0.080 }", None, 0.042, 0.040),
    ("CYL_TALL", "Cylinder { radius 0.030 height 0.120 }", None, 0.062, 0.060),
    ("CYL_SIDE", "Cylinder { radius 0.033 height 0.022 }", "1 0 0 1.5707963", 0.035, 0.033),
]
TOL = 0.0015  # the contact sinks ~0.4 mm on every body; a capsule end is >= 11 mm off


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")


def _world() -> str:
    parts = ["""#VRML_SIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/floors/protos/RectangleArena.proto"
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" newtonGroundMu 1.0 }
Viewpoint { orientation 0 0 1 0 position -2 0 0.5 }
RectangleArena { floorSize 6 6 wallHeight 0.02 }
"""]
    for i, (name, geom, rot, z, _) in enumerate(CASES):
        bo = f"Pose {{ rotation {rot} children [ {geom} ] }}" if rot else geom
        parts.append(f'Solid {{ translation {-1.2 + 0.35 * i:.3f} 0 {z} name "{name}" '
                     f'boundingObject {bo} physics Physics {{ density 500 }} }}\n')
    return "".join(parts)


def _rest_heights(tmp_path: Path, mode: str | None) -> dict:
    world = tmp_path / "cylinder_rest.omniworld"
    world.write_text(_world(), encoding="utf-8")
    tag = mode or "default"
    traj = tmp_path / f"traj_{tag}.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(tmp_path / f"log_{tag}.txt"),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_MS="2000")
    env.pop("OMNISIM_NEWTON_CYLINDER_NATIVE", None)
    if mode is not None:
        env["OMNISIM_NEWTON_CYLINDER_NATIVE"] = mode
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=120, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    last = {}
    if traj.is_file():
        for line in traj.read_text(encoding="utf-8").splitlines():
            cols = line.split("\t")
            if len(cols) >= 5:
                last[cols[1]] = float(cols[4])
    return last


def test_cylinder_off_the_joint_graph_rests_at_its_authored_height(tmp_path):
    z = _rest_heights(tmp_path, None)
    for name, _, _, _, expected in CASES:
        assert name in z, f"{name} missing from the trajectory probe"
        assert abs(z[name] - expected) < TOL, (
            f"{name} rests at z={z[name]:.4f}, authored {expected:.4f} (a capsule stand-in rests r higher)")


def test_hatch_zero_restores_the_capsule_stand_in(tmp_path):
    z = _rest_heights(tmp_path, "0")
    # r=0.08, h=0.01 as a capsule: h/2 + r = 0.085
    assert abs(z["CYL_FLAT"] - 0.085) < TOL, f"CYL_FLAT rests at {z['CYL_FLAT']:.4f}; =0 must keep the capsule"
