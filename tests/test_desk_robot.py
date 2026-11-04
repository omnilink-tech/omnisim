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

"""The desk robot (projects/manufacturing/products/desk_robot): its fits, its machine files,
its meshes, its drawings and its world. Engine-free (the drive test is test_desk_robot_drive.py)."""
import json
import math
from pathlib import Path
import re
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/desk_robot"
sys.path.insert(0, str(PRODUCT))

import design  # noqa: E402
import robot as robot_mod  # noqa: E402

R = design.Robot()


def test_the_bought_parts_fit():
    c = design.checks(R)
    assert abs(c["level_error_mm"]) < 0.05                 # wheels and caster stand the plate level
    assert c["tyre_stretch_pct"] < 5.0                      # Parker: no more than 5 % stretch
    assert c["shaft_overlap_with_milled_bore_mm2"] == 0.0  # the D-shaft enters the bore a 1 mm end mill cuts
    assert c["shaft_clear_of_rim_mm"] > 0 and c["shaft_end_inside_rim_mm"] > 0
    assert c["clamp_holes_from_plate_edge_mm"] > 2.0
    assert c["speed_no_load_m_s"] == pytest.approx(330 / 60 * math.pi * R.wheel_d / 1000, rel=1e-3)


def test_laser_jobs_stay_on_their_sheet():
    for name, g in design.laser_jobs(R).items():
        lines = g.splitlines()
        assert "G10 L20 P1 X0 Y0 Z0" in lines and "M4 S0" in lines and g.count("; cut pass") == 3
        xs = [float(v) for v in re.findall(r"X(-?[\d.]+)", g)]
        ys = [float(v) for v in re.findall(r"Y(-?[\d.]+)", g)]
        assert max(map(abs, xs)) < 49 and max(map(abs, ys)) < 44, name   # the 98 x 88 mm sheet
    assert "; engrave" in design.laser_jobs(R)["top_plate"]


def test_hub_program_changes_tools_and_stops_at_the_spoilboard():
    g = design.hub_program(R, 12.5)
    assert g.index("T3 M6") < g.index("T1 M6")             # the bores while the hubs are still held
    assert "G10 L20 P1 X0 Y0 Z12.500" in g
    zs = [float(v) for v in re.findall(r"Z(-?[\d.]+)", g.split("G54", 1)[1])]
    assert min(zs) == pytest.approx(-(R.hub_t + design.THROUGH_MM), abs=1e-6)


def test_printed_parts_are_closed_and_match_their_volume():
    for fn in (design.motor_clamp, design.rim, design.tray):
        bands = fn(R)
        tris = design.part_triangles(bands)
        # Divergence theorem: the signed volume of closed prisms is the bands' volume.
        v = np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6.0
        assert v == pytest.approx(design.band_volume_mm3(bands), rel=1e-6), fn.__name__


def test_bom_adds_up():
    b = design.bom(R)
    total = sum(p["g"] * p["qty"] for p in b["made"]) + sum(p["g"] * p["qty"] for p in b["bought"])
    assert b["total_g"] == pytest.approx(total, abs=0.05)
    assert all(p.get("source") for p in b["bought"])
    assert robot_mod.predictions(R)["mass_g"] == pytest.approx(b["total_g"], abs=0.2)


def test_world_regenerates():
    import os
    url = Path(os.path.relpath(robot_mod.MESHES, robot_mod.WORLD.parent)).as_posix()
    assert robot_mod.world(R, url).replace("\r", "") == robot_mod.WORLD.read_text(encoding="utf-8").replace("\r", "")
    for name in ("bottom_plate", "top_plate", "engraving", "motor_clamp", "rim", "tray", "hub", "tyre"):
        assert (robot_mod.MESHES / f"{name}.stl").exists(), name


def test_drawings_render(tmp_path):
    import release
    files = release.write_drawings(R, tmp_path)
    assert sorted(files) == sorted(f"{p['part']}.svg" for p in design.made_parts(R))
    assert "H1" in (tmp_path / "drawings/hub.svg").read_text(encoding="utf-8")
