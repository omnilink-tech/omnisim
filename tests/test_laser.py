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

"""The laser process (projects/manufacturing/laser): GRBL's laser pin, the kerf model and
its calibration against Sienci's table, the world, the job and the sheet inspector.

Engine-free except `test_tag_cuts_free_in_the_engine`, which runs the job headless."""
import math
from pathlib import Path
import re
import struct
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SECTOR = ROOT / "projects/manufacturing"
for _p in (SECTOR / "laser", SECTOR / "cnc", SECTOR / "core"):
    sys.path.insert(0, str(_p))

import grbl  # noqa: E402
import inspect_cut  # noqa: E402
import job as job_mod  # noqa: E402
import laser  # noqa: E402
import machine  # noqa: E402
from twin import Plant  # noqa: E402

LONGMILL = SECTOR / "machines/longmill_mk2_30x30"
MODULE = LONGMILL / "sienci_laserbeam_7w.json"
WORLD = ROOT / "projects/samples/demos/worlds/manufacturing/laser_longmill_laserbeam.omniworld"


class LaserRig:
    def __init__(self):
        self.m = machine.load(LONGMILL / "longmill_mk2_30x30.grbl")
        self.module = laser.LaserModule.load(MODULE)
        for k, v in self.module.grbl.items():
            self.m.apply_setting(k, v)
        s0 = [a.travel / 2 for a in self.m.axes.values()]
        self.ctl = grbl.GrblController(self.m, Plant.build(self.m, initial_s=s0), out=lambda s: None)
        self.out = laser.LaserOutput(self.ctl, self.module)
        self.ctl.planner.on_substep = self.out.substep
        sync = self.ctl.planner.on_sync

        def on_sync(b):
            if b.kind in ("spindle", "end"):
                self.out.on_sync(b)
            sync(b)

        self.ctl.planner.on_sync = on_sync

    def run(self, text, seconds, dt=0.002):
        """Feed `text`, run, and return (in a G0, pwm, speed) samples."""
        self.ctl.feed(text.encode())
        samples = []
        for k in range(int(round(seconds / dt))):
            self.ctl.tick(dt)
            b = self.ctl.planner.current_block()
            samples.append((b is not None and b.kind == "line" and b.rapid, self.out.pwm, self.ctl.planner.velocity))
        return samples


def test_pwm_follows_grbl_spindle_compute_pwm_value():
    assert laser.pwm_value(0, 0, 255) == 0                    # S0 is off
    assert laser.pwm_value(255, 0, 255) == 255
    assert laser.pwm_value(400, 0, 255) == 255                # above $30 clamps to full
    assert laser.pwm_value(1, 10, 255) == laser.PWM_MIN       # at or below $31: the minimum, not off
    assert laser.pwm_value(127.5, 0, 255) == math.floor(127.5 * 254 / 255) + 1
    assert laser.pwm_value(255, 0, 255, override_pct=50) == laser.pwm_value(127.5, 0, 255)


def test_module_settings_put_grbl_in_laser_mode():
    r = LaserRig()
    assert r.m.settings[32] == 1 and r.m.settings[30] == 255 and r.m.settings[31] == 0
    assert r.module.power_w == pytest.approx(7.0 * 0.95)


def test_g0_never_lights_and_m4_follows_speed():
    r = LaserRig()
    s = r.run("G21 G91 M4 S255\nG0 X10\nG1 X20 F600\n", 6.0)
    rapid = [p for g0, p, v in s if g0]
    assert rapid and max(rapid) == 0                         # the G0 is dark
    cruising = [p for g0, p, v in s if not g0 and abs(v - 10.0) < 1e-6]
    assert cruising and min(cruising) >= 254                 # at the programmed 10 mm/s: full
    ramp = [(p, v) for g0, p, v in s if not g0 and 1.0 < v < 9.0 and p > 0]
    assert ramp and all(abs(p / 255 - v / 10.0) < 0.02 for p, v in ramp)   # power ~ speed
    assert s[-1][1] == 0                                     # M4 at rest after the line: off


def test_m3_holds_power_at_rest_and_lights_on_a_bare_m3():
    r = LaserRig()
    s = r.run("G21 G91 M3 S255\nG1 X2 F600\n", 2.0)
    assert s[-1][1] == 255 and s[-1][2] == 0                 # the buffer ran dry: still lit
    r2 = LaserRig()
    s2 = r2.run("M3 S128\n", 0.2)
    assert s2[-1][1] == laser.pwm_value(128, 0, 255)         # spindle_set_state lights it at once
    s3 = r2.run("M5\n", 0.2)
    assert s3[-1][1] == 0


def test_a_completed_feed_hold_turns_the_laser_off():
    r = LaserRig()
    r.run("G21 G91 M3 S255\nG1 X50 F600\n", 1.0)
    assert r.out.pwm == 255
    r.ctl.feed(b"!")
    r.run("", 3.0)
    assert not r.ctl.planner.moving() and r.out.pwm == 0   # DISABLE_LASER_DURING_HOLD


def test_energy_per_millimetre_is_power_over_programmed_feed_in_m4():
    """M4 lays the same energy on every millimetre, accelerating or not."""
    r = LaserRig()
    r.run("G21 G91 M4 S255\nG1 X30 F300\n", 10.0)
    assert r.out.total_j == pytest.approx(r.module.power_w / 5.0 * 30.0, rel=0.01)


def test_kerf_model_reproduces_sienci_table():
    """Calibrated on the 3 mm row; the 6 mm and 1/4" rows are predictions."""
    rows = {row["row"]: row for row in laser.kerf_check(laser.LaserModule.load(MODULE))}
    for name, passes in (("3 mm plywood", 3), ("6 mm plywood", 5)):
        assert rows[name]["model_passes_wide"] == passes and rows[name]["model_passes_narrow"] == passes
    quarter = rows['1/4" plywood']
    assert quarter["sienci_passes"] == 16 and 13 <= quarter["model_passes_wide"] <= 16


def test_two_passes_do_not_cut_3mm_plywood():
    """The calibration is a bracket: Sienci's third pass is needed."""
    mod = laser.LaserModule.load(MODULE)
    for spot in mod.spot_mm:
        r = laser.kerf(3.0, laser.table_passes(3.0, mod.power_w, 2.2, 1.0, 2), laser.MATERIALS["plywood"], spot,
                       mod.beam_half_angle_rad, res_mm=0.005)
        assert r.through_at is None


def test_the_job_is_lightburn_shaped():
    g = job_mod.tag()
    lines = g.splitlines()
    assert "G10 L20 P1 X0 Y0 Z0" in lines and lines.index("M4 S0") < lines.index("; engrave")
    assert g.count("; cut pass") == 3 and "G0 Z-2" in g
    # Inner shape first: every pass cuts the hole before the outline.
    for block in g.split("; cut pass")[1:]:
        assert block.index("G3 X-5 Y0 I-3 J0") < block.index("G1 X12 Y-10")
    assert (SECTOR / "laser/jobs/tag_3mm_plywood.gcode").read_text(encoding="utf-8") == g


def test_world_regenerates_and_matches_the_engine_node():
    import importlib.util
    spec = importlib.util.spec_from_file_location("laser_world", SECTOR / "laser/world.py")
    laser_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(laser_world)
    text = laser_world.build(LONGMILL / "longmill_mk2_30x30.grbl", LONGMILL / "cad", MODULE, WORLD)
    assert text.replace("\r", "") == WORLD.read_text(encoding="utf-8").replace("\r", "")
    node = (ROOT / "resources/nodes/MillingStock.wrl").read_text(encoding="utf-8")
    body = re.search(r"MillingStock \{(.*?)\}\n", text, re.S).group(1)
    for name in re.findall(r"\b([a-z][A-Za-z]+) (?=[-\d\"])", body):
        assert re.search(rf"field\s+\S+\s+{name}\b", node), name
    assert 'toolShape "laser"' in body and "--laser=" in text and "LASER_GLOW" in text


def _zmap(h, fluence, res=0.0001, top=0.003):
    ny, nx = h.shape
    head = b"OMZMAP2\0" + struct.pack("<2i", nx, ny) + struct.pack("<6d", res, top, 0, 0, 0, 0) + struct.pack("<2d", 1.0, 1e6)
    return head + h.astype("<f4").tobytes() + fluence.astype("<f4").tobytes()


def test_inspector_finds_the_piece_that_fell_out(tmp_path):
    n = 201                                  # 20 mm at 0.1 mm, work 0,0 at the centre
    h = np.full((n, n), 0.003)
    f = np.zeros((n, n))
    # A 10 x 6 mm rectangle cut through, one cell wide, plus a slanted line that only
    # meets the rest at cell corners.
    h[70:131, 50] = h[70:131, 150] = 0.0
    h[70, 50:151] = h[130, 50:151] = 0.0
    p = tmp_path / "sheet.zmap"
    p.write_bytes(_zmap(h, f))
    z = inspect_cut.load(p)
    loose = [q for q in inspect_cut.pieces(z) if q["fell_out"]]
    assert len(loose) == 1 and loose[0]["size_mm"] == pytest.approx([9.8, 5.8], abs=1e-6)
    h2 = h.copy()
    h2[100, 50] = 0.003                      # one tab left: the part stays in the sheet
    p.write_bytes(_zmap(h2, f))
    assert not any(q["fell_out"] for q in inspect_cut.pieces(inspect_cut.load(p)))


def test_engine_burns_with_the_laser_model():
    src = (ROOT / "src/omnisim/nodes/OmMillingStock.cpp").read_text(encoding="utf-8")
    assert 'file.write("OMZMAP2", 8);' in src and "burnSweep" in src
    # The same physics as laser.kerf: a Gaussian of 1/e^2 radius sqrt(w0^2 + (tan(theta) dz)^2).
    assert "std::exp(-2.0 * (dx * dx / (wx * wx) + dy * dy / (wy * wy)))" in src
    assert "std::sqrt(w0x * w0x + spread * spread)" in src


@pytest.mark.engine
def test_tag_cuts_free_in_the_engine(tmp_path):
    """The whole loop: Sienci's settings, streamed over TCP, cut the tag free."""
    import run_job
    r = run_job.run(WORLD, SECTOR / "laser/jobs/tag_3mm_plywood.gcode", tmp_path, timeout_s=900)
    assert r["job"]["state"] == "done"
    z = inspect_cut.load(tmp_path / "sheet.zmap")
    rep = inspect_cut.part_report(z, [30, 20], 6.0, 2 * (24 + 14) + 2 * math.pi * 3)
    assert rep["part"] is not None
    assert rep["part"]["size_mm"] == pytest.approx([30, 20], abs=0.2)
    assert rep["part"]["hole_d_mm"] == pytest.approx(6.0, abs=0.2)
