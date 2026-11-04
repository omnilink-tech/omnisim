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

"""Engine-free tests for the FDM printer twin: Klipper config, planner, hotend, slicer, preflight, world."""
import importlib.util
import json
import math
from pathlib import Path
import re
import struct
import sys

import numpy as np
import pytest

SERVICE = Path(__file__).resolve().parents[1] / "projects/manufacturing/printing"
sys.path.insert(0, str(SERVICE))

import cura_slice  # noqa: E402
import hotend  # noqa: E402
import inspect_print  # noqa: E402
import klipper  # noqa: E402
import printer  # noqa: E402


def _load(name):
    """preflight.py and world.py share their names with the CNC twin's: load them apart."""
    spec = importlib.util.spec_from_file_location(f"printer_twin_{name}", SERVICE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


preflight = _load("preflight")
world = _load("world")

SV08 = printer.load(printer.DEFAULT_PROFILE)
CUBE_GCODE = SERVICE / "jobs/cube20_sv08.gcode"
HOT = ["M104 S220", "M109 S220", "G28", "G90", "M83"]


def _job(lines, start=(175.0, 175.0, 50.0)):
    return klipper.load_job(SV08, "\n".join(lines), start=list(start))


def test_klipper_config_parser_reads_sections_continuations_and_comments():
    cfg = """
[printer]
kinematics: corexy   # inline comment
max_velocity: 300
max_accel = 3000
[stepper_x]
rotation_distance: 40
microsteps: 16
position_max: 250
[gcode_macro START]
gcode:
    G28
    G1 Z10
[extruder]
rotation_distance: 22.6789511
gear_ratio: 50:10
microsteps: 16
nozzle_diameter: 0.4
filament_diameter: 1.75
"""
    sec = printer.parse_klipper_cfg(cfg)
    assert sec["printer"]["kinematics"] == "corexy"
    assert sec["printer"]["max_accel"] == "3000"
    assert sec["gcode_macro START"]["gcode"].splitlines() == ["", "G28", "G1 Z10"]
    kept = printer.extract(sec)
    assert "gcode_macro START" not in kept and kept["stepper_x"]["position_max"] == "250"
    assert printer._ratio("50:10") == 5.0


def test_sv08_profile_matches_the_vendor_config():
    s = SV08.summary()
    assert s["kinematics"] == "corexy" and s["max_velocity"] == 700 and s["max_accel"] == 40000
    assert s["steps_per_mm"] == {"x": 80.0, "y": 80.0, "z": 533.3333}       # 40 mm, 1/16, 80:12 on Z
    assert s["e_steps_per_mm"] == pytest.approx(492.3077, abs=1e-3)          # 6.5 mm rotation distance
    assert s["envelope"]["z"] == [-5.0, 347.0]
    assert s["input_shaper"]["x"] == ["mzv", 35.0, 0.001]
    assert SV08.extruder.max_cross_section == pytest.approx(0.64)            # Klipper's 4 * d^2 default
    assert "GPL" in SV08.profile["source"]["licence"]


def test_square_corner_junction_is_klippers_square_corner_velocity():
    job = _job(HOT + ["G1 X100 Y100 F30000", "G1 X200 Y100 E5 F12000", "G1 X200 Y200 E5"])
    corner = job.moves[-1]
    # Klipper's junction_deviation is built so a 90 degree corner runs at square_corner_velocity.
    assert corner.start_v == pytest.approx(SV08.square_corner_velocity, rel=1e-6)
    straight = _job(HOT + ["G1 X100 Y100 F30000", "G1 X150 Y100 E2 F12000", "G1 X200 Y100 E2"]).moves[-1]
    assert straight.start_v == pytest.approx(200.0, rel=1e-6)                # no corner, no slowdown


def test_a_single_move_is_a_trapezoid_with_the_right_time():
    # Homing ends at X191 Y165; a dwell is a full stop, so the move starts from rest.
    job = _job(["G28", "G90", "M204 S1000", "G4 P1", "G1 X91 Y165 F6000"])
    m = job.moves[-1]
    # 100 mm at 100 mm/s, 1000 mm/s^2: 0.1 s up, 0.1 s down, 0.9 s cruising.
    assert (m.accel_t, m.cruise_t, m.decel_t) == pytest.approx((0.1, 0.9, 0.1), rel=1e-9)
    traj = klipper.Trajectory(job, SV08, shaper=False)
    end = traj.raw([m.t0 + m.duration])[0]
    assert end[0] == pytest.approx(91.0) and traj.raw([m.t0 + 0.1])[0][0] == pytest.approx(m.start[0] - 5.0)


def test_klipper_refusals_stop_the_job_at_their_line():
    cold = _job(["G28", "G90", "G1 X10 Y10 E1 F3000"])
    assert cold.errors and cold.errors[0].message == "Extrude below minimum temp" and cold.errors[0].line == 3
    out = _job(HOT + ["G1 X400 Y10 F3000"])
    assert out.errors[0].message.startswith("Move out of range")
    unhomed = _job(["M109 S220", "G1 X10 Y10 F3000"])
    assert unhomed.errors[0].message.startswith("Must home axis first")
    blob = _job(HOT + ["G1 E200 F300"])
    assert blob.errors[0].message.startswith("Extrude only move too long")
    fat = _job(HOT + ["G1 X150 Y150 F3000", "G1 X160 E10"])                  # 2.4 mm^2 of section
    assert fat.errors[0].message.startswith("Move exceeds maximum extrusion")


def test_homing_runs_the_override_sequence_and_ends_where_it_says():
    job = _job(["G28"], start=(100.0, 100.0, 50.0))
    assert not job.errors
    assert job.end_position[:3] == pytest.approx([191.0, 165.0, 10.0])      # homing_override ends at G0 X191 Y165, Z10
    homes = [m for m in job.moves if m.end[0] == 355.0 or m.end[1] == 364.0]
    assert homes, "X and Y home toward their maximum endstops"


def test_mzv_shaper_is_normalised_and_centred():
    a, t = klipper.shaper_impulses("mzv", 35.0, 0.1)
    assert a.sum() == pytest.approx(1.0) and float(np.dot(a, t)) == pytest.approx(0.0, abs=1e-12)
    assert len(a) == 3 and t.max() - t.min() == pytest.approx(0.75 / (35.0 * math.sqrt(1 - 0.01)))


def test_step_sampler_is_the_full_sampling_a_window_at_a_time():
    # The controller samples the job a window at a time (a 3.3 h print sampled whole took
    # 1.55 GB); the values must be exactly the whole-job ones, in the controller's own
    # access pattern (k - 1 .. k + 2 each step) and across every window edge.
    lines = HOT + ["G1 X100 Y100 F30000"] + [f"G1 X{100 + 10 * (i % 2)} Y{100 + i} E0.2 F9000" for i in range(60)]
    traj = klipper.Trajectory(_job(lines), SV08)
    dt = 0.002
    n = int(traj.duration / dt) + 3
    ts = np.arange(n + 1) * dt
    full, fe = traj.toolhead(ts), traj.extruder(ts)
    sampler = klipper.StepSampler(traj, dt, window=37)
    assert sampler.n == n
    for step in range(n + 5):
        for k in (max(step - 1, 0), step, step + 1, step + 2):
            assert sampler.toolhead(k) == [float(v) for v in full[min(k, n)]]
            assert sampler.extruder(k) == float(fe[min(k, n)])
    assert sampler._toolhead.shape[0] <= 37                 # memory is the window's, not the job's


def test_hotend_with_matched_pressure_advance_delivers_the_request():
    h = hotend.Hotend(filament_area=2.405, lag_s=0.025, max_mm3_s=21.0)
    pa, dt, rate = 0.025, 0.001, 2.0                  # 2 mm/s of filament = 4.8 mm^3/s
    deposited = []
    for k in range(2001):
        t = k * dt
        e = rate * t if t < 1.0 else rate * 1.0
        de = rate if 0.0 < t < 1.0 else 0.0     # at rest at the first sample, which sets the baseline
        deposited.append(h.step(e + pa * de, dt))
    assert h.deposited == pytest.approx(2.0 * 2.405, rel=0.002)
    # PA cancels the lag: flow is at full rate within a couple of time constants of the start.
    assert sum(deposited[1:50]) / (49 * dt) == pytest.approx(rate * 2.405, rel=0.05)


def test_hotend_without_pressure_advance_lags_and_the_melt_limit_skips():
    h = hotend.Hotend(filament_area=2.405, lag_s=0.025, max_mm3_s=21.0)
    first = [h.step(2.0 * k * 0.001, 0.001) for k in range(26)]
    assert sum(first) < 0.7 * 2.0 * 0.025 * 2.405   # the first 25 ms come out short: the line starts thin
    fast = hotend.Hotend(filament_area=2.405, lag_s=0.025, max_mm3_s=21.0)
    for k in range(1001):
        fast.step(15.0 * k * 0.001, 0.001)           # asks 36 mm^3/s of a 21 mm^3/s hotend
    assert fast.skipped > 10.0 and fast.peak_flow == pytest.approx(21.0, rel=0.05)


def test_preflight_of_the_cube_job():
    report = preflight.preflight(SV08, CUBE_GCODE.read_text(encoding="utf-8"), start=[177.5, 182.0, 100.0])
    assert report["stopped"] == []
    assert report["layers"] == 100
    assert 300.0 < report["time_s"] < 450.0
    assert 3.0 < report["filament_g"] < 4.0          # a 20 mm cube, 10 % infill, 2 walls
    assert report["over_limit_moves"] == 0
    # Everything the job asks for leaves the nozzle (priming is extrude-only and counted too).
    assert report["deposited_mm3"] == pytest.approx(report["requested_mm3"], rel=0.05)
    assert any("not modelled" in n or "warm-up" in n for n in report["not_modelled"])


def test_cube_gcode_is_sliced_for_175_mm_filament():
    """CuraEngine keeps extruder settings per extruder: the job must not be sliced for 2.85 mm."""
    text = CUBE_GCODE.read_text(encoding="utf-8")
    job = klipper.load_job(SV08, text, start=[177.5, 182.0, 100.0])
    walls = [m for m in job.moves if m.feature == "WALL-OUTER" and m.end[2] > 1.0 and m.move_d > 5.0]
    area = np.median([m.axes_r[3] * SV08.extruder.filament_area for m in walls])
    assert area == pytest.approx(0.4 * 0.2 * 0.98, rel=0.05)   # line width x layer height x flow


@pytest.mark.skipif(not any(Path(p.rpartition("/")[0]).is_dir() for p in cura_slice.CURA_GLOBS),
                    reason="no Cura install")
def test_cura_slice_resolves_every_formula_and_caps_flow(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    try:
        cura_slice.find_cura()
    except SystemExit:
        pytest.skip("no CuraEngine")
    stl = tmp_path / "box.stl"
    trimesh.creation.box(extents=[10, 10, 2]).export(stl)
    report = cura_slice.slice_stl(stl, SV08.profile, tmp_path / "box.gcode", at=[100.0, 120.0])
    assert report["unresolved"] == [] and report["engine_errors"] == []
    assert report["speed_caps"]["speed_infill"] == (300.0, pytest.approx(233.3, abs=0.1))
    job = klipper.load_job(SV08, (tmp_path / "box.gcode").read_text(encoding="utf-8"), start=[175, 175, 50])
    printed = [m for m in job.moves if m.feature.startswith("WALL") and m.axes_r[3] > 0]
    xs = [m.end[0] for m in printed]
    assert min(xs) > 94.0 and max(xs) < 106.0          # placed at --at
    assert not job.errors


def test_world_drives_three_sliders_and_writes_only_printedpart_fields():
    text = world.build_world(SV08, "projects/manufacturing/machines/sovol_sv08/sovol_sv08.json",
                             "jobs/x.gcode", [177.5, 182.0, 100.0])
    for axis in "xyz":
        assert f'LinearMotor {{ name "{axis}_motor"' in text and f'PositionSensor {{ name "{axis}_sensor" }}' in text
    assert 'controller "fdm_twin"' in text and "supervisor TRUE" in text
    # Every axis lighter than dt * kd moves like dt * kd (Newton's servo-armature bound).
    assert '"--accel-ff" "5e-05,5e-05,5e-05"' in text
    model = (SERVICE.parents[2] / "resources/nodes/PrintedPart.wrl").read_text(encoding="utf-8")
    declared = set(re.findall(r"^\s*(?:field|hiddenField)\s+\w+\s+(\w+)", model, re.M))
    body = re.search(r"PrintedPart \{(.*?)\}", text, re.S).group(1)
    written = set(re.findall(r"\b([a-zA-Z]+)\s+(?:\"|-?\d)", body))
    assert written <= declared, written - declared
    assert {"extrudedVolume", "depositedVolume", "beadCount", "nozzleCrashes"} <= declared


def test_pack_world_puts_the_bed_under_the_cad_nozzle_and_nests_the_axes():
    """A CAD pack world: machine X0 Y0 Z0 is the CAD's nozzle less its machine position."""
    pack = {
        "name": "fake", "source": {"licence": "test"},
        "cad_pose_s": [100.0, 50.0, -2.0], "tool_tip": [0.01, -0.02, 0.105],
        "axis_dirs": {"x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "z": [0.0, 0.0, 1.0]},
        "bodies": [{"group": "gantry", "axis": "z", "parent": "frame"},
                   {"group": "beam", "axis": "y", "parent": "gantry"},
                   {"group": "toolhead", "axis": "x", "parent": "beam"}],
        "groups": {g: {"meshes": [{"file": f"meshes/{g}_0.stl", "rgba": rgba, "faces": 12}],
                       "bbox": [[-0.1, -0.1, 0.0], [0.1, 0.1, 0.2]]}
                   for g, rgba in (("frame", [0.239, 0.627, 1.0, 1.0]), ("gantry", [0.72, 0.72, 0.77, 1.0]),
                                   ("beam", [1.0, 1.0, 1.0, 1.0]), ("toolhead", [0.03, 0.03, 0.03, 1.0]))},
    }
    text = world.build_pack_world(SV08, "p.json", None, [100.0, 50.0, -2.0], pack, "../pack")
    m = re.search(r"PrintedPart \{ translation (\S+) (\S+) (\S+) .*?nozzleTip (\S+) (\S+) (\S+)", text)
    origin, tip = [float(v) for v in m.groups()[:3]], [float(v) for v in m.groups()[3:]]
    assert origin == pytest.approx([0.01 - 0.1, -0.02 - 0.05, 0.105 + 0.002])
    assert tip == pytest.approx(pack["tool_tip"])
    # At power-up = the CAD pose every body sits at identity; the toolhead nests in the beam in the gantry.
    assert text.count("translation 0 0 0") >= 3
    assert text.index('DEF GANTRY') < text.index('DEF BEAM') < text.index('DEF TOOLHEAD')
    # SolidWorks' default blue becomes dark plastic; grey, white and black stay as drawn.
    assert world._finish([0.239, 0.627, 1.0, 1.0]) == [*world.DARK_PLASTIC, 1.0]
    for kept in ([0.72, 0.72, 0.77, 1.0], [1.0, 1.0, 1.0, 1.0], [0.03, 0.03, 0.03, 1.0]):
        assert world._finish(kept) == kept
    assert 'url "../pack/meshes/toolhead_0.stl"' in text


def _beads_file(path, beads, origin=(0.0, 0.0, 0.0), deposited_m3=1e-6):
    arr = np.asarray(beads, dtype="<f4")
    arr[:, :8] /= 1000.0
    path.write_bytes(b"OMBEADS1" + struct.pack("<i", len(arr)) + struct.pack("<5d", *origin, deposited_m3, 2e-4)
                     + arr.tobytes())


def test_inspect_print_says_when_the_dump_is_not_the_whole_part(tmp_path):
    # Before 2026-11-03 PrintedPart stopped recording beads at 523,776 and the dump of a
    # 145 g PETG shell section held 52 % of its plastic; the inspector measured the third that
    # was there and said nothing. The beads' own volume against the part's count says it.
    trimesh = pytest.importorskip("trimesh")
    stl = tmp_path / "block.stl"
    trimesh.creation.box(extents=[20, 20, 1.0]).export(stl)
    mesh = inspect_print.place_drawing(stl, [350, 350])
    wall = [[165.2, 175.0, 0.2, 184.8, 175.0, 0.2, 0.45, 0.2, 0.0]]            # 19.6 mm of stadium
    area = (0.45 - 0.2) * 0.2 + math.pi * 0.2 ** 2 / 4.0
    assert inspect_print.bead_volume_mm3(np.array(wall, dtype=float)) == pytest.approx(19.6 * area, rel=1e-6)
    whole = tmp_path / "whole.beads"
    _beads_file(whole, wall, deposited_m3=19.6 * area * 1e-9)
    assert inspect_print.inspect(inspect_print.load_beads(whole), mesh)["unrecorded_fraction"] < 1e-4
    cut = tmp_path / "cut.beads"
    _beads_file(cut, wall, deposited_m3=2 * 19.6 * area * 1e-9)               # half the plastic has no bead
    assert inspect_print.inspect(inspect_print.load_beads(cut), mesh)["unrecorded_fraction"] == pytest.approx(0.5, abs=1e-4)


def test_inspect_print_folds_a_bead_laid_while_z_rises_into_its_layer(tmp_path):
    # The first bead of a layer is laid while the Z servo still trails the layer change
    # by ~20 um (a 0.045 mm bead, 2026-11-03 shell section). Measured as a layer of its
    # own it compared one bead with a whole section: outer error -1 mm, footprint -170 mm.
    trimesh = pytest.importorskip("trimesh")
    stl = tmp_path / "block.stl"
    trimesh.creation.box(extents=[20, 20, 1.0]).export(stl)
    mesh = inspect_print.place_drawing(stl, [350, 350])
    lo, hi = 165.2, 184.8
    pts = [(lo, lo), (hi, lo), (hi, hi), (lo, hi), (lo, lo)]
    layer = lambda z: [[*pts[k], z, *pts[k + 1], z, 0.4, 0.2, 0.0] for k in range(4)]
    rising = [[lo, lo, 0.372, lo + 0.045, lo, 0.381, 0.4, 0.18, 0.0]]
    dump = tmp_path / "rising.beads"
    _beads_file(dump, layer(0.2) + rising + layer(0.4))
    r = inspect_print.inspect(inspect_print.load_beads(dump), mesh)
    assert r["layers_measured"] == 2
    assert abs(r["size_error_mm"]["x_mean"]) < 0.01 and r["outer_surface_error_mm"]["min"] > -0.05


def test_inspect_print_measures_a_perfect_wall_and_a_shrunk_one(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    stl = tmp_path / "block.stl"
    trimesh.creation.box(extents=[20, 20, 1.0]).export(stl)
    mesh = inspect_print.place_drawing(stl, [350, 350])

    def layer(z, inset):
        lo, hi = 165.0 + 0.2 + inset, 185.0 - 0.2 - inset   # the outer wall's centre, half a 0.4 line in
        pts = [(lo, lo), (hi, lo), (hi, hi), (lo, hi), (lo, lo)]
        return [[*pts[k], z, *pts[k + 1], z, 0.4, 0.2, 0.0] for k in range(4)]

    good = tmp_path / "good.beads"
    _beads_file(good, layer(0.2, 0.0) + layer(0.4, 0.0) + layer(0.6, 0.0))
    r = inspect_print.inspect(inspect_print.load_beads(good), mesh)
    assert r["layers_measured"] == 3
    assert abs(r["outer_surface_error_mm"]["mean"]) < 0.01
    assert abs(r["size_error_mm"]["x_mean"]) < 0.01
    shrunk = tmp_path / "shrunk.beads"
    _beads_file(shrunk, layer(0.2, 0.1) + layer(0.4, 0.1))
    r = inspect_print.inspect(inspect_print.load_beads(shrunk), mesh)
    assert r["outer_surface_error_mm"]["mean"] == pytest.approx(-0.1, abs=0.01)
    assert r["size_error_mm"]["x_mean"] == pytest.approx(-0.2, abs=0.01)
