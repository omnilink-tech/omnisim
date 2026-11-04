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

"""The desk robot's physical assembly (assemble.py): its part data, and the run on Newton."""
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
from shapely.geometry import MultiPolygon, Polygon

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/desk_robot"
sys.path.insert(0, str(PRODUCT))

import assemble as A  # noqa: E402
import design  # noqa: E402
import robot as robot_mod  # noqa: E402

R = design.Robot()


@pytest.fixture(scope="module")
def parts():
    return A.kit(R, "meshes")


def test_every_step_has_parts_in_order(parts):
    assert sorted({p.step for p in parts}) == list(range(len(robot_mod.STEPS)))
    names = [p.name for p in parts]
    assert len(names) == len(set(names))
    for p in parts:
        if p.mate not in ("ASSY", "STAND"):
            mate = next(q for q in parts if q.name == p.mate)
            assert mate.step <= p.step, f"{p.name} mates {mate.name}, which comes later"


def test_every_screw_has_a_hole_in_both_parts(parts):
    joints = [f for p in parts for f in p.fasteners if f.joint]
    assert len(joints) == len(A._joint_rows(R)[0])
    for p in parts:
        for f in p.fasteners:
            if not f.check_hole:
                continue
            assert f.tol_mm >= 0
            if f.joint:
                assert None not in f.holes.values(), f"{f.name}: no hole in {f.holes}"
        assert not [c for c in p.design_checks if "a hole for the screw" in c["check"] and not c["pass"]]


def test_hole_clearances_come_from_the_geometry(parts):
    plate = parts[0]
    sx, sy = R.standoff_xy
    assert plate.hole_at(sx, sy, R.plate_top_z - 1) == pytest.approx(design.CLEARANCE["M3"], abs=0.01)
    assert plate.hole_at(0.0, 0.0, R.plate_top_z - 1) is None          # the plate is solid at its centre


def test_printed_footprints_decompose_exactly():
    for bands in (design.motor_clamp(R), design.tray(R)):
        for _, _, poly in bands:
            polys = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
            filled = sum(Polygon(g.exterior, [i for i in g.interiors if Polygon(i).area >= 20]).area for g in polys)
            got = sum((x1 - x0) * (y1 - y0) for x0, y0, x1, y1 in A.rect_boxes(poly))
            assert got == pytest.approx(filled, rel=1e-9)


def test_penetration_measures_overlap():
    a = [A.box_mm((0, 0, 0), (10, 10, 10))]
    T = A.Pose()
    assert A.penetration(a, T, [A.box_mm((0, 0, 10), (10, 10, 10))], T)[0] == pytest.approx(0.0, abs=1e-9)
    assert A.penetration(a, T, [A.box_mm((0, 0, 9), (10, 10, 10))], T)[0] == pytest.approx(1e-3, abs=1e-9)
    cyl = [A.cyl_mm((0, 0, 0), (0, 0, 1), 2.0, 4.0)]
    assert A.penetration(cyl, T, [A.sphere_mm((0, 0, 2.5), 1.0)], T)[0] == pytest.approx(0.5e-3, abs=1e-6)


def test_a_ring_passes_a_shaft_smaller_than_its_bore():
    ring = A.ring_mm((0, 0, 0), 1, 1.55, 8.0, 0.0, 4.0)
    T = A.Pose()
    assert A.penetration(ring, T, [A.cyl_mm((0, 2, 0), (0, 1, 0), 1.5, 10.0)], T)[0] == 0.0
    assert A.penetration(ring, T, [A.cyl_mm((0, 2, 0), (0, 1, 0), 1.6, 10.0)], T)[0] == pytest.approx(0.05e-3, abs=1e-5)


def test_inertia_about_the_centre_of_mass():
    com, phys = A._physics([(1000.0, [A.box_mm((10, 0, 0), (100, 200, 300))])])
    assert np.allclose(com, [0.01, 0, 0])
    I = [float(v) for v in phys.split("inertiaMatrix [")[1].split("]")[0].split()]
    assert I[0] == pytest.approx(1.0 / 12 * (0.2 ** 2 + 0.3 ** 2), rel=1e-5)
    assert I[3:] == [0.0, 0.0, 0.0]


@pytest.mark.engine
def test_the_robot_assembles_step_by_step(tmp_path):
    out = tmp_path / "assembly"
    r = subprocess.run([sys.executable, str(PRODUCT / "assemble.py"), "--out", str(out), "--port", "6913"],
                       cwd=ROOT, capture_output=True, text=True, timeout=1800)
    log = json.loads((out / "assembly_log.json").read_text(encoding="utf-8"))
    assert len(log["steps"]) == len(robot_mod.STEPS), r.stdout[-3000:]
    failed = {s["step"]: s["reasons"] for s in log["steps"] if not s["pass"]}
    assert not failed, json.dumps(failed, indent=1)
    assert log["all_pass"]
    for k in range(1, len(robot_mod.STEPS) + 1):
        assert (out / f"physical_step_{k}.png").exists()
