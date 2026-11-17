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

"""Every collider of a multi-shape boundingObject registers by default.

The defect this pins (open since the Newton move, fixed 2026-10-09): a
boundingObject that is a Group of several shapes collided as its FIRST shape
only unless WorldInfo.newtonCompoundColliders was TRUE, and that field also
switches the body's inertia source, so its default could not be flipped. Leo
Rover's wheels rolled on the 0.057 m hub listed first instead of the 0.0625 m
tyre listed second; a Chair.proto was a seat slab with no legs.

Collider registration is now decoupled: a boundingObject holding more than one
shape registers all of them by default, the inertia source still follows
newtonCompoundColliders alone, and a single-shape body takes the same walker
as before. OMNISIM_NEWTON_COMPOUND_SHAPES=0 restores first-shape-only.

Probe: a table whose boundingObject is a Group of a top slab (first) and four
legs, dropped onto the floor. With every shape registered it stands on its legs
(origin at z = 0.39); with the slab alone it lies on the floor (z = 0.01).

    python -m pytest tests/test_newton_compound_shapes_default.py -v

OMNISIM_BINARY selects a scratch build. Two engine launches, a few seconds each.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

LEG = "Box {{ size 0.04 0.04 0.38 }}"
LEGS = "".join(f"Pose {{ translation {x} {y} -0.2 children [ {LEG.format()} ] }} "
               for x, y in ((0.17, 0.17), (0.17, -0.17), (-0.17, 0.17), (-0.17, -0.17)))
WORLD = f"""#VRML_SIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/floors/protos/RectangleArena.proto"
WorldInfo {{ basicTimeStep 4 }}
Viewpoint {{ orientation 0 0 1 0 position -2 0 0.5 }}
RectangleArena {{ floorSize 4 4 wallHeight 0.02 }}
Solid {{
  translation 0 0 0.40
  name "TABLE"
  boundingObject Group {{ children [ Box {{ size 0.4 0.4 0.02 }} {LEGS}] }}
  physics Physics {{ density 500 }}
}}
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


def _rest_z(tmp_path: Path, shapes_env: str | None, world_text: str = WORLD) -> tuple[float, str]:
    world = tmp_path / "compound_table.omniworld"
    world.write_text(world_text, encoding="utf-8")
    tag = shapes_env or "default"
    log, traj = tmp_path / f"log_{tag}.txt", tmp_path / f"traj_{tag}.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_MS="1500")
    env.pop("OMNISIM_NEWTON_COMPOUND_SHAPES", None)
    env.pop("OMNISIM_NEWTON_COMPOUND_COLLIDERS", None)
    if shapes_env is not None:
        env["OMNISIM_NEWTON_COMPOUND_SHAPES"] = shapes_env
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=120, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    rows = [r.split("\t") for r in traj.read_text(encoding="utf-8").splitlines()] if traj.is_file() else []
    z = [float(r[4]) for r in rows if len(r) > 4 and r[1] == "TABLE"]
    assert z, "no trajectory for the table; log tail:\n" + text[-1500:]
    return z[-1], text


def test_every_shape_of_a_group_collides_by_default(tmp_path):
    z, text = _rest_z(tmp_path, None)
    assert abs(z - 0.39) < 0.01, f"the table should stand on its legs at z=0.39, ended at {z:.4f}"
    assert "only the FIRST is registered" not in text


def test_hatch_zero_restores_first_shape_only(tmp_path):
    z, text = _rest_z(tmp_path, "0")
    assert abs(z - 0.01) < 0.01, f"with OMNISIM_NEWTON_COMPOUND_SHAPES=0 the slab alone lies on the floor, ended at {z:.4f}"
    assert "only the FIRST is registered" in text, "the =0 path must still name the dropped shapes"


def test_world_field_false_keeps_first_shape_only(tmp_path):
    # WorldInfo.newtonCompoundShapes FALSE is the per-world pin (the B2/Go2 policy
    # worlds, trained on foot-only calf contact, set it).
    pinned = WORLD.replace("WorldInfo { basicTimeStep 4 }", "WorldInfo { basicTimeStep 4 newtonCompoundShapes FALSE }")
    assert pinned != WORLD
    z, _ = _rest_z(tmp_path, None, pinned)
    assert abs(z - 0.01) < 0.01, f"newtonCompoundShapes FALSE must keep the slab alone, ended at {z:.4f}"
