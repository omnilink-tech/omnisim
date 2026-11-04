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

"""SparkRover Mini's make.py (projects/manufacturing/products/spark_rover/make.py) without an
engine: every laser and CNC job is planned, its drawn layout agrees with its program, the
CNC worlds put the tool tip where each program says, and the measurements read a known
height map right. Engine-free."""
import sys
from pathlib import Path

import numpy as np
import pytest
import shapely
from shapely.geometry import Point, box

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
sys.path.insert(0, str(PRODUCT))

import make  # noqa: E402


@pytest.fixture(scope="module")
def jobs():
    return make.all_jobs(None, ("laser", "cnc"))


def test_every_machine_job_is_planned(jobs):
    names = {j.name for j in jobs}
    laser = {k for k in make.chassis.jobs() if k != "spine_foot"}
    assert laser <= names
    assert {"spine_foot", "shoulder_brackets", "face_cover"} <= names
    by = {j.name: j for j in jobs}
    assert by["spine_side_r"].same_as == "spine_side_l"           # the same part, cut once
    assert all(j.same_as is None for j in jobs if j.name != "spine_side_r")
    assert by["face_cover"].material == "acrylic" and by["spine_foot"].material == "aluminium_alloy"
    assert len(by["shoulder_brackets"].drawn) == 2 and len(by["spacers"].drawn) == 4


def test_laser_layouts_match_their_programs(jobs):
    for j in jobs:
        if j.process == "laser":
            assert make.check_layout(j)["max_err_mm"] <= 0.15, j.name
            x0, y0, x1, y1 = make.unary_union(list(j.drawn.values())).bounds
            assert j.sheet_mm[0] >= x1 - x0 + 2 and j.sheet_mm[1] >= y1 - y0 + 2


def test_cnc_parts_fit_their_blanks(jobs):
    for j in jobs:
        if j.process == "cnc":
            gx0, gy0, gx1, gy1 = make._gcode_xy_extent(j.program)
            sx, sy, _ = j.sheet_mm
            assert -sx / 2 < gx0 and gx1 < sx / 2 and -sy / 2 < gy0 and gy1 < sy / 2, j.name
            for poly in j.drawn.values():
                b = poly.bounds
                assert gx0 < b[0] and b[2] < gx1 and gy0 < b[1] and b[3] < gy1, j.name


def test_tool_tip_stands_where_the_program_says(jobs):
    pack = make._pack()
    for j in jobs:
        if j.process == "cnc":
            t = j.sheet_mm[2]
            s0 = make.s0_z_for(j.program, t)
            tip = pack["tool_tip"][2] * 1000.0 + (s0 - pack["cad_pose_s"][2])
            assert tip - (pack["bed_top_m"] * 1000.0 + t) == pytest.approx(10.0, abs=1e-6)
            assert 0.0 < s0 < 120.0 - 20.0                         # room for the programs' Z20 retract


def test_cover_tool_is_the_one_the_program_calls():
    tool = make.COVER_TOOLS["tools"][0]
    import shell
    prog = shell.cover_program(shell.config(None))
    assert tool["number"] == 1 and "T1 M6" in prog
    assert f"{tool['diameter_mm']}" in prog                           # "1/8 in ... 3.175 mm"


def _zmap(res_mm, sx, sy, t_mm):
    nx, ny = int(round(sx / res_mm)), int(round(sy / res_mm))
    h = np.full((ny, nx), t_mm / 1000.0)
    return {"nx": nx, "ny": ny, "res": res_mm / 1000.0, "top": t_mm / 1000.0, "h": h}


def _xy(z):
    return make._cells_mm(z, *np.mgrid[0:z["ny"], 0:z["nx"]])


def test_measure_reads_a_known_cut():
    z = _zmap(0.05, 40.0, 30.0, 3.0)
    X, Y = _xy(z)
    hole = Point(2.0, 1.0).buffer(2.0, quad_segs=64)
    part = box(-10, -5, 10, 5).difference(hole)
    outer = box(-10, -5, 10, 5)
    kerf = (shapely.contains_xy(outer.buffer(0.3), X, Y) & ~shapely.contains_xy(outer, X, Y)) | \
           (shapely.contains_xy(hole, X, Y) & ~shapely.contains_xy(hole.buffer(-0.3), X, Y))
    z["h"][kerf] = 0.0
    job = make.Job("t", "laser", "test", "", (40.0, 30.0, 3.0), "plywood", {"plate": part})
    rep = make.measure_job(z, job)
    p = rep["parts"]["plate"]
    assert p["cut_free"] and p["holes_cut"] == "1/1"
    assert p["size_mm"] == pytest.approx([20.0, 10.0], abs=0.06)
    assert abs(p["outline_offset_mm"]) < 0.03
    assert p["holes"][0]["cut_d_mm"] == pytest.approx(4.0, abs=0.05)
    assert p["thickness_mm"] == pytest.approx(3.0)
    assert rep["loose_other"] == 1                                    # the hole's slug


def test_measure_reports_a_part_still_attached():
    z = _zmap(0.05, 40.0, 30.0, 3.0)
    X, Y = _xy(z)
    outer = box(-10, -5, 10, 5)
    ring = shapely.contains_xy(outer.buffer(0.3), X, Y) & ~shapely.contains_xy(outer, X, Y)
    ring &= ~((np.abs(X) < 1.0) & (Y > 0))                             # a bridge left uncut
    z["h"][ring] = 0.0
    rep = make.measure_job(z, make.Job("t", "laser", "test", "", (40.0, 30.0, 3.0), "plywood", {"plate": outer}))
    assert rep["parts"]["plate"] == {"cut_free": False, "material_at_part": True, "free_at_skin_mm": None}


def test_measure_reports_the_web_that_holds_a_part():
    z = _zmap(0.05, 40.0, 30.0, 3.0)
    X, Y = _xy(z)
    outer = box(-10, -5, 10, 5)
    hole = Point(2.0, 1.0).buffer(2.0, quad_segs=64)
    ring = shapely.contains_xy(outer.buffer(0.3), X, Y) & ~shapely.contains_xy(outer, X, Y)
    hring = shapely.contains_xy(hole, X, Y) & ~shapely.contains_xy(hole.buffer(-0.3), X, Y)
    z["h"][ring | hring] = 0.0
    z["h"][ring & (np.abs(X) < 1.0) & (Y > 0)] = 0.12e-3              # a 0.12 mm web under the kerf
    z["h"][hring & (X > 3.5)] = 0.2e-3                                 # the slug on a 0.2 mm web
    job = make.Job("t", "laser", "test", "", (40.0, 30.0, 3.0), "plywood", {"plate": outer.difference(hole)})
    p = make.measure_job(z, job)["parts"]["plate"]
    assert not p["cut_free"] and p["free_at_skin_mm"] == pytest.approx(0.12, abs=0.003)
    assert p["holes_held_by_skin"] == 1 and p["hole_skin_max_mm"] == pytest.approx(0.2, abs=0.003)
    assert p["holes"][0]["cut_d_mm"] == pytest.approx(4.0, abs=0.06)
    assert p["size_mm"] == pytest.approx([20.0, 10.0], abs=0.06)


def test_measure_reads_a_blind_pilot():
    z = _zmap(0.05, 40.0, 30.0, 8.0)
    X, Y = _xy(z)
    outer = box(-10, -5, 10, 5)
    z["h"][shapely.contains_xy(outer.buffer(3.0), X, Y) & ~shapely.contains_xy(outer, X, Y)] = 0.0
    pilot = np.hypot(X - 3.0, Y + 1.0) <= 1.25
    z["h"][pilot] = (8.0 - 4.8) / 1000.0
    job = make.Job("t", "cnc", "test", "", (40.0, 30.0, 8.0), "aluminium_alloy", {"b": outer},
                   {"b": [make.Blind(3.0, -1.0, 2.5, 4.8)]})
    b = make.measure_job(z, job)["parts"]["b"]["blind"][0]
    assert b["cut"] and not b["through"]
    assert b["depth_mm"] == pytest.approx(4.8, abs=1e-6)
    assert b["cut_d_mm"] == pytest.approx(2.5, abs=0.06)


def test_print_is_not_implemented_here():
    assert make.main(["--process", "print", "--release", ""]) == 2


def test_windows_fit_the_engine_and_cover_the_sheet(jobs):
    for j in jobs:
        wins = make.windows(j)
        sx, sy, _ = j.sheet_mm
        assert sum((w[2] - w[0]) * (w[3] - w[1]) for w in wins) == pytest.approx(sx * sy)
        if j.process == "laser":
            # OmMillingStock coarsens a side over 1984 cells: every window stays at 0.05 mm
            assert all(max(w[2] - w[0], w[3] - w[1]) / 0.05 <= 1984 for w in wins), j.name
    by = {j.name: j for j in jobs}
    assert len(make.windows(by["spacers"])) == 1 and len(make.windows(by["deck_upper"])) == 4


def test_crop_keeps_every_burn_that_reaches_the_window(jobs):
    import re
    j = next(j for j in jobs if j.name == "deck_upper")
    w = make.windows(j)[0]
    cropped = make.crop_program(j.program, w)
    assert "G10 L20 P1 X0 Y0 Z0" in cropped and cropped.count("; cut pass") == 3
    x = 0.0
    for ln in cropped.splitlines():
        m = re.match(r"G([01]) X(-?[\d.]+) Y(-?[\d.]+)", ln)
        if not m:
            continue
        nx = float(m.group(2))
        if m.group(1) == "1":                       # a kept burn states its power and feed
            assert "S255" in ln and "F132" in ln
            assert max(x, nx) >= w[0] - 2.0 and min(x, nx) <= w[2] + 2.0
        x = nx
    full_burns = sum(1 for ln in j.program.splitlines() if ln.startswith("G1 X"))
    kept = sum(1 for ln in cropped.splitlines() if ln.startswith("G1 X"))
    assert 0.15 * full_burns < kept < 0.5 * full_burns


def test_stitch_rebuilds_the_sheet():
    job = make.Job("t", "laser", "test", "", (150.0, 60.0, 3.0), "plywood", {})
    wins = make.windows(job)
    assert len(wins) == 2
    full = np.random.default_rng(1).random((int(60 / 0.05) + 1, int(150 / 0.05) + 1))
    maps = []
    for wx0, wy0, wx1, wy1 in wins:
        i0, j0 = int(round((wx0 + 75) / 0.05)), int(round((wy0 + 30) / 0.05))
        nx, ny = int(round((wx1 - wx0) / 0.05)) + 1, int(round((wy1 - wy0) / 0.05)) + 1
        maps.append({"nx": nx, "ny": ny, "res": 5e-5, "top": 0.003, "h": full[j0:j0 + ny, i0:i0 + nx]})
    z = make.stitch(job, maps)
    assert z["h"].shape == full.shape and np.array_equal(z["h"], full)
