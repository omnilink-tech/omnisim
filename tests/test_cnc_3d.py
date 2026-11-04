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

"""3D milling on the CNC twin: tool table and M6, STL surfaces, drop-cutter, roughing and
finishing, and the ideal-cut check. Engine-free."""
from pathlib import Path
import sys

import numpy as np
import pytest

SERVICE = Path(__file__).resolve().parents[1] / "projects/manufacturing/cnc"
MACHINES = SERVICE.parent / "machines"
sys.path.insert(0, str(SERVICE))

import gcode  # noqa: E402
import inspect_part  # noqa: E402
import machine  # noqa: E402
import parts  # noqa: E402
import preflight  # noqa: E402
import toolpath3d as tp  # noqa: E402
import tool_table as tools  # noqa: E402

LONGMILL = MACHINES / "longmill_mk2_30x30/longmill_mk2_30x30.grbl"
TABLE = MACHINES / "longmill_mk2_30x30/longmill_mk2_tools.json"


def test_tool_table_and_the_m6_split():
    table = tools.load(TABLE)
    assert table[1].stickout_mm == 0.0 and table[2].shape == "ball"
    assert table[2].surface_mm(table[2].radius_mm) == pytest.approx(table[2].radius_mm)
    assert table[3].surface_mm(1.0) == pytest.approx(1.0 / np.tan(np.radians(30.0)))
    assert tools.tool_change_in("T2 M6") == ("T2", True, 2)
    assert tools.tool_change_in("M06 T3 (ball)") == ("T3", True, 3)
    assert tools.tool_change_in("M60") == ("M60", False, None)       # not M6
    with pytest.raises(gcode.GcodeError) as err:                     # GRBL 1.1 has no M6
        gcode.Interpreter().execute("M6")
    assert err.value.code == 20


def test_preflight_models_the_tool_change_and_its_length_offset():
    m = machine.load(LONGMILL)
    table = tools.load(TABLE)
    prog = "G21 G90\nM3 S18000\nG1 Z-1 F500\nG0 Z5\nT2 M6\nG1 Z-1 F500\nG0 Z5\nM2\n"
    rep = preflight.preflight(m, prog, wco=[-400, -400, -100], tools=table)
    assert [c["tool"]["number"] for c in rep["tool_changes"]] == [2]
    assert rep["bounds"]["z"]["work"] == [-1.0, 5.0]
    assert rep["bounds"]["z"]["machine"] == [-101.0, -91.0]      # 4 mm longer: 4 mm higher
    # The sender stops the spindle for the change: the cut after it has none.
    assert rep["warning_counts"]["feed_with_spindle_off"] == 1
    rep = preflight.preflight(m, "T9 M6\n", tools=table)
    assert rep["verdict"] == "fail" and rep["errors"][0]["code"] == 38


def test_stl_round_trip_and_surface_raster(tmp_path):
    xs = np.arange(-10, 10.01, 0.5)
    X, Y = np.meshgrid(xs, xs)
    z = -2.0 - 0.02 * (X ** 2 + Y ** 2) / 4
    stl = tmp_path / "bowl.stl"
    tp.write_stl(stl, tp.heightfield_solid(z, xs[0], xs[0], 0.5, -8.0))
    tris = tp.load_stl(stl)
    g = tp.surface_grid(tris, -10, -10, 201, 201, 0.1)
    Xg, Yg = np.meshgrid(g.xs, g.ys)
    truth = -2.0 - 0.02 * (Xg ** 2 + Yg ** 2) / 4
    assert np.abs(g.z - truth).max() < 0.01                       # the mesh's own chord error


def test_drop_cutter_and_staircase_never_put_the_tool_into_a_wall():
    res = 0.1
    z = np.zeros((101, 101))
    z[:, 50:] = -5.0                                              # a 5 mm step at x = 5
    g = tp.Grid(z, 0, 0, res)
    ball = tools.Tool(2, "ball", 3.175)
    tip = tp.tip_heights(g, ball)
    assert tip.z[50, 90] == pytest.approx(-5.0)                    # far from the wall: on the floor
    assert tip.z[50, 40] == pytest.approx(0.0)                     # on the top: on the top
    path = tp.staircase(np.column_stack([g.xs, np.full(101, 5.0), tip.z[50]]), res)
    # Between consecutive points the path never dips below the higher of the two.
    for a, b in zip(path[:-1], path[1:]):
        assert min(a[2], b[2]) >= -5.0 - 1e-9
        if abs(b[2] - a[2]) > res:
            assert a[0] == b[0] or a[2] == b[2]                    # vertical, or level


def test_the_mould_job_never_gouges_the_part():
    """Rough and finish a part, cut the program into a grid, and compare it with the part."""
    res = 0.2
    xs = -50 + np.arange(501) * res
    X, Y = np.meshgrid(xs, xs)
    g = tp.Grid(parts.mould(X, Y), -50, -50, res)
    table = tools.load(TABLE)
    prog, info = tp.job(g, table[1], table[2], finish_stepover=1.0, step_down=4.0)
    assert info["rough_paths"] > 0 and info["finish_paths"] > 0
    assert prog.count("M6") == 2
    ideal = inspect_part.cut_program(prog, -50, -50, 501, 501, res, table)
    inner = (slice(5, -5), slice(5, -5))
    gap = (ideal - g.z)[inner]
    assert gap.min() > -0.01                                       # no gouge
    assert np.median(gap) < 0.01                                   # and the part is there


def test_inspect_3d_separates_the_twin_from_the_program():
    part = np.full((50, 50), -2.0)
    ideal = part + 0.02                                            # the program leaves 20 um
    target = part.copy()
    zmap = {"depth_mm": -(ideal - 0.005), "res_mm": 0.1, "removed_mm3": 0.0}   # the twin cut 5 um deeper
    rep = inspect_part.inspect_3d(zmap, target, ideal, np.ones_like(part, dtype=bool))
    assert rep["twin_vs_ideal_mm"]["mean"] == pytest.approx(-0.005)
    assert rep["ideal_vs_stl_mm"]["mean"] == pytest.approx(0.02)
    assert rep["gouged_points"] == 0


def test_cutting_load_from_the_material_removed():
    import cutting
    al = cutting.MATERIALS["aluminium_alloy"]
    meter = cutting.LoadMeter(al, cutting.Spindle("router", 700.0, (10000, 30000)))
    # A 6.35 mm slot, 3 mm deep, at 1000 mm/min removes 6.35 * 3 * 1000 / 60 = 317.5 mm^3/s.
    for k in range(21):
        meter.update(0.005 * k, 317.5 * 0.005 * k)
    rep = meter.report(rpm=18000, feed_mm_min=1000, diameter_mm=6.35, flutes=2, t=0.1)
    assert rep["mrr_mm3_s"] == pytest.approx(317.5)
    assert rep["power_w"] == pytest.approx(0.75 * 317.5, rel=1e-3)           # u 0.7-0.8 J/mm^3
    assert rep["power_range_w"] == [pytest.approx(222.2, abs=0.1), pytest.approx(254.0, abs=0.1)]
    assert rep["torque_nm"] == pytest.approx(rep["power_w"] / (2 * np.pi * 18000 / 60), rel=1e-3)
    assert rep["chip_load_mm"] == pytest.approx(1000 / (2 * 18000), rel=1e-3)
    assert rep["spindle_load"] < 1.0 and not meter.overloads
    # Three times the feed asks for more than the router's 700 W: flagged, with the time.
    meter.update(0.2, 317.5 * 0.1 + 3 * 317.5 * 0.1)
    meter.update(0.25, 317.5 * 0.1 + 3 * 317.5 * 0.15)
    rep = meter.report(rpm=18000, feed_mm_min=3000, diameter_mm=6.35, flutes=2, t=0.25)
    assert rep["spindle_load"] > 1.0 and meter.overloads[0]["t"] == 0.25
    # A material with no published coefficient reports the load as unknown, not a guess.
    acrylic = cutting.LoadMeter(cutting.MATERIALS["acrylic"])
    acrylic.update(0.0, 0.0)
    acrylic.update(0.1, 10.0)
    rep = acrylic.report(rpm=18000, feed_mm_min=1000, diameter_mm=6.35, flutes=2)
    assert rep["power_w"] is None and "no published" in rep["note"]
    assert cutting.MATERIALS["mdf"].estimated


def test_the_drawn_tools_replace_the_cad_bit():
    """The pack drops the CAD's own bit (it cannot spin or change); the world draws the table's."""
    import json
    pack = json.loads((MACHINES / "longmill_mk2_30x30/cad/pack.json").read_text(encoding="utf-8"))
    assert pack["tool_tip"] == [0.084227, -0.061818, 0.030274]       # the cutting tip did not move
    collet = pack["collet"]
    assert collet["nose_radius_m"] == pytest.approx(0.003163, abs=1e-5)   # the 1/4" bit's shank
    assert pack["groups"]["z"]["bbox"][0][2] >= collet["nose_z_m"] - 1e-5  # nothing of it is left below (bbox: 10 um)
    text = (SERVICE.parents[1] / "samples/demos/worlds/manufacturing/cnc_longmill_mk2_3d.omniworld").read_text(
        encoding="utf-8")
    for name in ("DEF COLLET Pose", "DEF TOOL_SPIN Pose", "DEF TOOL_1 Pose", "DEF TOOL_2 Pose", "DEF TOOL_3 Pose",
                 "DEF BLUR_1 Pose { translation 0 0 0.12 "):          # the spin-blur sleeves start hidden
        assert name in text
    # Tool 1 is in the spindle at power-up; the others wait inside the router body.
    assert "DEF TOOL_1 Pose { translation 0 0 0 " in text
    assert "DEF TOOL_2 Pose { translation 0 0 0.116 " in text          # -4 mm stickout + 120 mm
