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

"""Engine-free tests for the printer twin's ideal print mode (printing/ideal_print.py)."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

SERVICE = Path(__file__).resolve().parents[1] / "projects/manufacturing/printing"
sys.path.insert(0, str(SERVICE))

import hotend  # noqa: E402
import ideal_print  # noqa: E402
import inspect_print  # noqa: E402
import klipper  # noqa: E402
import printer  # noqa: E402

SV08 = printer.load(printer.DEFAULT_PROFILE)
CUBE_GCODE = SERVICE / "jobs/cube20_sv08.gcode"
CUBE_STL = SERVICE / "jobs/cube20.stl"
S0 = [177.5, 182.0, 100.0]          # world.py's power-up position on the SV08
HOT = ["M104 S220", "M109 S220", "G28", "G90", "M83"]

# Two 0.2 mm layers of a 20 mm square wall, centred on the bed.
SQUARE = HOT + ["G1 Z0.2 F600", "G1 X167.5 Y172 F12000",
                "G1 X187.5 Y172 E0.8 F3000", "G1 X187.5 Y192 E0.8", "G1 X167.5 Y192 E0.8", "G1 X167.5 Y172 E0.8",
                "G1 Z0.4 F600",
                "G1 X187.5 Y172 E0.8 F3000", "G1 X187.5 Y192 E0.8", "G1 X167.5 Y192 E0.8", "G1 X167.5 Y172 E0.8",
                "G1 Z5 F600"]


def _cube_job():
    job = klipper.load_job(SV08, CUBE_GCODE.read_text(encoding="utf-8"), start=S0)
    return job, klipper.Trajectory(job, SV08)


def test_steps_are_sampled_exactly_as_the_twins_step_sampler_samples_them():
    job, traj = _cube_job()
    sampler = klipper.StepSampler(traj, 0.002)
    k = np.r_[0:3000, 40000:43000]                            # the first moves, and printing at speed
    toolhead, extruder = ideal_print.sample_steps(traj, 0.002, 43000, jobs=4, chunk=1000)
    assert np.array_equal(toolhead[k], np.array([sampler.toolhead(i) for i in k]))
    assert np.array_equal(extruder[k], np.array([sampler.extruder(i) for i in k]))


def test_the_compiled_hotend_is_hotend_py_bit_for_bit():
    job, traj = _cube_job()
    e = traj.extruder(np.arange(40000, 60000) * 0.002)         # walls and infill, pressure advance on
    ref = hotend.Hotend.for_printer(SV08)
    expected = []
    for v in e:
        ref.step(float(v), 0.002)
        expected.append(ref.deposited)
    got, skipped, peak = ideal_print.run_hotend(hotend.Hotend.for_printer(SV08), e, 0.002)
    assert np.array_equal(got, np.array(expected))
    assert skipped == ref.skipped and peak == ref.peak_flow
    # and at the melt limit, where the gear skips
    fast = hotend.Hotend(SV08.extruder.filament_area, 0.025, 5.0)
    ramp = np.linspace(0.0, 40.0, 2000)
    got, skipped, _ = ideal_print.run_hotend(fast, ramp, 0.002)
    for v in ramp:
        fast.step(float(v), 0.002)
    assert got[-1] == fast.deposited and skipped == fast.skipped > 0


def _lay(tips_mm, volumes_mm3, jit=True):
    size, res, nozzle, nx, ny = ideal_print.part_frame(SV08)
    top = np.zeros(nx * ny, dtype=np.float32)
    fn = ideal_print._lay_all if jit or not hasattr(ideal_print._lay_all, "py_func") else ideal_print._lay_all.py_func
    beads, count, laid, crashes, dropped = fn(np.ascontiguousarray(np.asarray(tips_mm, float) / 1000.0),
                                              np.ascontiguousarray(np.asarray(volumes_mm3, float) * 1e-9),
                                              top, nx, ny, res, nozzle, np.zeros((4, 9), dtype=np.float32))
    return beads[:count].astype(float) * np.r_[np.full(8, 1000.0), 1.0], laid * 1e9, crashes, dropped * 1e9


def test_laying_follows_printed_parts_rule():
    # A straight line at a constant flow on the bed: one merged bead, as tall as the gap
    # and as wide as its volume needs as a flattened stadium (PrintedPart::lay).
    steps = 200
    xs = 100.0 + 0.6 * np.arange(steps + 1)
    tips = np.c_[xs, np.full(steps + 1, 150.0), np.full(steps + 1, 0.2)]
    area = 0.45 * 0.2 - 0.2 * 0.2 + math.pi * 0.2 ** 2 / 4.0
    vols = np.r_[0.0, np.cumsum(np.full(steps, 0.6 * area))]
    beads, laid, crashes, dropped = _lay(tips, vols)
    assert len(beads) == 1 and crashes == 0 and dropped == 0.0
    assert beads[0, 6] == pytest.approx(0.45, rel=1e-5) and beads[0, 7] == pytest.approx(0.2, rel=1e-5)
    assert beads[0, 3] - beads[0, 0] == pytest.approx(120.0, rel=1e-6)
    assert laid == pytest.approx(inspect_print.bead_volume_mm3(beads), rel=1e-5)
    # The same line again 0.2 mm up stands on the first: 0.2 mm tall again, not 0.4.
    up = tips + [0.0, 0.0, 0.2]
    beads2, _, crashes, _ = _lay(np.r_[tips, up], np.r_[vols, vols[-1] + vols])
    assert len(beads2) == 2 and crashes == 0 and beads2[1, 7] == pytest.approx(0.2, rel=1e-4)
    # A nozzle standing still makes a blob; 3 mm over nothing, a round hanging strand.
    still = np.tile([120.0, 120.0, 0.2], (3, 1))
    blob, _, _, _ = _lay(still, [0.0, 0.0, 0.5])
    assert len(blob) == 1 and blob[0, 8] == pytest.approx(0.5e-9, rel=1e-6)
    high = tips + [0.0, 0.0, 3.0]
    bridge, _, _, _ = _lay(high, vols)
    assert bridge[0, 6] == pytest.approx(bridge[0, 7]) == pytest.approx(math.sqrt(4 * area / math.pi), rel=1e-5)
    # Too little for a quarter-nozzle strand is ooze: dropped, as the engine drops it.
    _, laid, _, dropped = _lay(tips[:3], [0.0, 1e-4, 2e-4])
    assert laid == 0.0 and dropped == pytest.approx(2e-4)


def test_a_slow_nozzle_lays_a_line_and_no_bead_is_wider_than_the_face():
    # The 2026-11-04 bead fix. At 10 mm/s (0.02 mm a 2 ms step) the nozzle used to count as
    # standing still: its plastic was held and laid on the next faster step as one bead up
    # to 7 mm wide, or as a dome above the tip that the next step counted as a crash.
    area = 0.45 * 0.2 - 0.2 * 0.2 + math.pi * 0.2 ** 2 / 4.0
    slow = np.c_[100.0 + 0.02 * np.arange(501), np.full(501, 150.0), np.full(501, 0.2)]
    vols = np.r_[0.0, np.cumsum(np.full(500, 0.02 * area))]
    beads, laid, crashes, dropped = _lay(slow, vols)
    assert len(beads) == 1 and beads[0, 8] == 0.0 and crashes == 0 and dropped == 0.0
    assert beads[0, 6] == pytest.approx(0.45, rel=1e-4) and beads[0, 3] - beads[0, 0] == pytest.approx(10.0)
    # The same creep with four times the plastic (a corner, where the melt outruns the
    # nozzle): every line is at most the nozzle's face, the rest are blobs, nothing is lost.
    face = ideal_print.FACE_RATIO * 0.4
    beads, laid, crashes, dropped = _lay(slow, 4.0 * vols)
    line = beads[:, 8] == 0.0
    assert beads[line, 6].max() == pytest.approx(face, rel=1e-6) and crashes == 0 and dropped == 0.0
    assert (~line).any() and laid == pytest.approx(4.0 * vols[-1], rel=1e-12)
    assert inspect_print.bead_volume_mm3(beads) == pytest.approx(4.0 * vols[-1], rel=1e-5)
    # Held plastic at a stop is a blob where the nozzle stood, not smeared onto the next
    # move: 0.04 mm^3 waits under the blob threshold, then the nozzle moves off at 5 mm/s.
    stop = np.r_[np.tile([120.0, 120.0, 0.2], (20, 1)),
                 np.c_[120.0 + 0.01 * np.arange(1, 11), np.full(10, 120.0), np.full(10, 0.2)]]
    held = np.r_[0.0, np.full(19, 0.04), 0.04 + 0.01 * area * np.arange(1, 11)]
    beads, laid, crashes, _ = _lay(stop, held)
    assert beads[:, 6].max() <= face * (1 + 1e-6) and crashes == 0
    assert laid == pytest.approx(held[-1], rel=1e-9)
    blob = beads[beads[:, 8] > 0.0]
    assert len(blob) == 1 and 0.5 * (blob[0, 0] + blob[0, 3]) == pytest.approx(120.0, abs=1e-4)
    # pressed under the face into a disc as tall as the gap: its top is the nozzle tip
    assert blob[0, 2] == pytest.approx(0.2, abs=1e-6) and blob[0, 7] == pytest.approx(0.2, abs=1e-6)


def test_the_compiled_laying_is_the_python_one():
    res = ideal_print.ideal_print(SV08, "\n".join(SQUARE), S0)
    if not ideal_print.HAVE_NUMBA:
        pytest.skip("numba not installed: the Python kernel is the only one")
    job = klipper.load_job(SV08, "\n".join(SQUARE), start=S0)
    traj = klipper.Trajectory(job, SV08)
    n = int(traj.duration / 0.002) + 3
    th, ex = ideal_print.sample_steps(traj, 0.002, n)
    vols, _, _ = ideal_print.run_hotend(hotend.Hotend.for_printer(SV08), ex, 0.002)
    a, la, _, _ = _lay(th, vols, jit=True)
    b, lb, _, _ = _lay(th, vols, jit=False)
    assert np.array_equal(a, b) and la == lb
    assert res["done"]["beads"] >= 8


def test_the_dump_round_trips_through_inspect_print(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    res = ideal_print.ideal_print(SV08, "\n".join(SQUARE), S0)
    assert not res["errors"] and res["done"]["nozzle_crashes"] == 0
    dump = tmp_path / "square.beads"
    origin = ideal_print.schematic_origin(SV08)
    ideal_print.write_dump(dump, res["beads"], origin, res["deposited_m3"], res["resolution_m"])
    back = inspect_print.load_beads(dump)
    assert back["origin_m"] == pytest.approx(origin) and back["resolution_mm"] == pytest.approx(0.2)
    assert back["deposited_mm3"] == pytest.approx(res["done"]["laid_mm3"], abs=0.01)
    expected = res["beads"].astype(float) * np.r_[np.full(8, 1000.0), 1.0]
    assert np.array_equal(back["beads"], expected)                       # float32 in, the same float32 out
    # 4 x 0.8 mm of 1.75 mm filament per layer
    assert res["done"]["extruded_mm3"] == pytest.approx(2 * 3.2 * SV08.extruder.filament_area, rel=0.01)
    stl = tmp_path / "square.stl"
    box = trimesh.creation.box(extents=[20.4, 20.4, 0.4])
    box.apply_translation([0, 0, 0.2])
    box.export(stl)
    r = inspect_print.inspect(back, inspect_print.place_drawing(stl, [350, 350], [177.5, 182.0]))
    assert r["layers_measured"] == 2 and r["unrecorded_fraction"] < 0.005
    assert r["height_mm"]["printed"] == pytest.approx(0.4) and r["beads"] == res["done"]["beads"]
    # one 0.52 mm line round a 20 mm square: the walls 0.06 mm outside the 20.4 mm drawing
    assert 0.0 < r["outer_surface_error_mm"]["p05"] < 0.1


# The twin's 20 mm cube, printed on the engine 2026-11-04 after the bead fix (fdm_twin +
# PrintedPart, schematic SV08 world, Newton/MuJoCo CPU at 2 ms, scratch engine
# omnisim-bin-v10beads, RTX 3060 Laptop / Ryzen 7 5800H machine), measured by inspect_print.py
# (blobs in the outline); and how far the ideal mode was from it on the cube, the desk robot's
# print plate and a 145 g PETG shell section (printing/README.md, "Ideal print"). The bounds
# are those measured differences, rounded up. Before the fix the twin cube read 24,488 beads,
# outer mean +0.0453, footprint +0.2005 / +0.1914 (wide beads at the corners, up to 3.3 mm of
# one layer's footprint).
CUBE_TWIN = {
    "laid_mm3": 2850.27, "height": 20.05, "layers": 100, "beads": 27128,
    "outer_mean": 0.0406, "p05": 0.0141, "p95": 0.0699, "fx": 0.1641, "fy": 0.1543,
    # measured ideal - twin on the cube after the fix: laid +0.03 mm^3, outer mean -0.027, p05
    # -0.019, p95 -0.051, footprint -0.10 and -0.11, beads +2.1 %. Before it (cube / plate /
    # shell): laid +0.04 / +0.05 / -0.08 mm^3; outer mean -0.027 / -0.021 / -0.003; p05 and p95
    # -0.020 and -0.051 / -0.017 and -0.038 / -0.027 and +0.026; footprint -0.09 and -0.10 /
    # -0.06 and -0.07 / -0.01 and -0.06; beads +2.2 / +3.0 / +2.4 %
    "laid_tol": 0.5, "outer_mean_tol": 0.035, "pct_tol": 0.06, "footprint_tol": 0.12, "beads_tol": 0.05,
}


def test_the_ideal_cube_agrees_with_the_twins_cube():
    pytest.importorskip("trimesh")
    res = ideal_print.ideal_print(SV08, CUBE_GCODE.read_text(encoding="utf-8"), S0, jobs=4)       # ~1.5 s
    beads = res["beads"].astype(float) * np.r_[np.full(8, 1000.0), 1.0]
    dump = {"origin_m": [0.0, 0.0, 0.0], "deposited_mm3": res["deposited_m3"] * 1e9, "resolution_mm": 0.2,
            "beads": beads}
    r = inspect_print.inspect(dump, inspect_print.place_drawing(CUBE_STL, SV08.profile["bed"]["size_mm"]), 0.4, 1)  # ~4 s
    t = CUBE_TWIN
    assert res["done"]["laid_mm3"] == pytest.approx(t["laid_mm3"], abs=t["laid_tol"])
    assert res["done"]["nozzle_crashes"] == 0
    assert r["height_mm"]["printed"] == pytest.approx(t["height"], abs=0.001)
    assert r["layers_measured"] == t["layers"]
    o = r["outer_surface_error_mm"]
    assert o["mean"] == pytest.approx(t["outer_mean"], abs=t["outer_mean_tol"])
    assert o["p05"] == pytest.approx(t["p05"], abs=t["pct_tol"]) and o["p95"] == pytest.approx(t["p95"], abs=t["pct_tol"])
    s = r["size_error_mm"]
    assert s["x_mean"] == pytest.approx(t["fx"], abs=t["footprint_tol"])
    assert s["y_mean"] == pytest.approx(t["fy"], abs=t["footprint_tol"])
    assert abs(len(beads) - t["beads"]) < t["beads_tol"] * t["beads"]
