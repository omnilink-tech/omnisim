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

"""Engine-free tests for the CNC twin: config parsing, G-code, planner, virtual GRBL, preflight."""
import math
from pathlib import Path
import socket
import sys
import time

import pytest

SERVICE = Path(__file__).resolve().parents[1] / "projects/manufacturing/cnc"
MACHINES = SERVICE.parent / "machines"
sys.path.insert(0, str(SERVICE))
sys.path.insert(0, str(SERVICE.parent / "core"))   # cad_pack, worldkit, drives

import gcode  # noqa: E402
import grbl  # noqa: E402
import machine  # noqa: E402
import preflight  # noqa: E402
import server  # noqa: E402
import world  # noqa: E402
from twin import Plant  # noqa: E402

EXAMPLE = MACHINES / "example_3018/example_3018_with_switches.grbl"

FLUIDNC = """
name: test lowrider
arc_tolerance_mm: 0.002
junction_deviation_mm: 0.01
axes:
  x:
    steps_per_mm: 200
    max_rate_mm_per_min: 3000
    acceleration_mm_per_sec2: 100
    max_travel_mm: 1220
    soft_limits: true
    homing: {cycle: 2, positive_direction: false, mpos_mm: 0, feed_mm_per_min: 100, seek_mm_per_min: 800}
    motor0: {limit_neg_pin: gpio.35, hard_limits: false, pulloff_mm: 2}
  y:
    steps_per_mm: 200
    max_rate_mm_per_min: 3000
    acceleration_mm_per_sec2: 100
    max_travel_mm: 2440
    soft_limits: true
    homing: {cycle: 2, positive_direction: false, mpos_mm: 0}
  z:
    steps_per_mm: 800
    max_rate_mm_per_min: 800
    acceleration_mm_per_sec2: 50
    max_travel_mm: 100
    soft_limits: true
    homing: {cycle: 1, positive_direction: true, mpos_mm: 0}
"""


class Rig:
    def __init__(self, m=None, **plant_kw):
        self.m = m or machine.load(EXAMPLE)
        self.buf = []
        self.ctl = grbl.GrblController(self.m, Plant.build(self.m, **plant_kw), out=self.buf.append)

    def send(self, text, seconds=0.05):
        self.ctl.feed(text.encode("latin-1"))
        return self.run(seconds)

    def run(self, seconds, dt=0.01):
        start = len(self.buf)
        for _ in range(int(round(seconds / dt))):
            self.ctl.tick(dt)
        return "".join(self.buf[start:])

    def until_idle(self, limit=600.0):
        out = []
        t = 0.0
        while t < limit:
            out.append(self.run(0.1))
            t += 0.1
            if self.ctl.state == "Idle" and self.ctl.planner.idle() and self.ctl._wait_block is None:
                break
        return "".join(out)

    def homed(self):
        assert "ok" in self.send("$H\n", 0.01) + self.until_idle()
        assert self.ctl.homed
        return self


def test_grbl_dump_maps_to_envelope_and_home():
    m = machine.load(EXAMPLE)
    x = m.axes["x"]
    assert (x.env_min, x.env_max, x.home_mpos, x.home_positive) == (-300.0, 0.0, -1.0, True)
    assert m.soft_limits and m.hard_limits and m.homing
    m.apply_setting(23, 1)          # X homes negative
    assert m.axes["x"].home_mpos == -299.0 and not m.axes["x"].home_positive


def test_fluidnc_yaml_reads_envelope_from_mpos_and_direction():
    m = machine.from_fluidnc_yaml(FLUIDNC)
    assert m.name == "test lowrider"
    assert (m.axes["x"].env_min, m.axes["x"].env_max) == (0.0, 1220.0)
    assert (m.axes["z"].env_min, m.axes["z"].env_max) == (-100.0, 0.0)
    assert m.soft_limits and m.homing and not m.hard_limits
    assert m.pulloff == 2.0


def test_boot_alarm_locks_gcode_until_unlock_or_home():
    r = Rig()
    assert "[MSG:'$H'|'$X' to unlock]" in "".join(r.buf)
    assert "error:9" in r.send("G0 X-10\n")
    assert "[MSG:Caution: Unlocked]" in r.send("$X\n")
    assert r.ctl.state == "Idle"


def test_homing_puts_belief_and_rail_in_agreement():
    r = Rig(initial_s=[120, 60, 30]).homed()
    assert r.ctl.planner.position == [-1.0, -1.0, -1.0]
    rep = r.ctl.twin_report()
    assert all(abs(e) < 1e-6 for e in rep["error_mm"])
    assert all(abs(rep["rails"][a]["offset_mm"]) < 1e-6 for a in "xyz")
    assert "<Idle|MPos:-1.000,-1.000,-1.000|" in r.send("?")


def test_job_runs_and_sync_commands_ack_after_motion():
    r = Rig().homed()
    out = r.send("G21 G90\nG0 X-100 Y-50\nM3 S8000\n", 0.02)
    assert out.count("ok") == 2               # M3 waits for the rapid to finish
    out += r.until_idle()
    assert out.count("ok") == 3
    assert r.ctl.spindle == ("cw", 8000.0)
    out = r.send("G1 Z-10 F300\nG2 X-80 Y-50 I10 J0 F800\nM5\nM30\n", 0.02) + r.until_idle()
    assert "[MSG:Pgm End]" in out
    assert [round(v, 3) for v in r.ctl.planner.position] == [-80.0, -50.0, -10.0]
    assert r.ctl.spindle[0] == "off"


def test_soft_limit_alarm_needs_reset():
    r = Rig().homed()
    out = r.send("G0 X-305\n")
    assert "ALARM:2" in out and "[MSG:Reset to continue]" in out
    assert r.ctl.state == "Alarm"
    assert "ok" not in r.send("G0 X-10\n")     # ignored until reset
    out = r.send("\x18")
    assert "Grbl 1.1h" in out and "[MSG:'$H'|'$X' to unlock]" in out
    assert r.ctl.homed                         # stopped before the limit: position kept
    assert "[MSG:Caution: Unlocked]" in r.send("$X\n")


def test_jog_beyond_travel_is_refused_and_jog_cancel_stops_short():
    r = Rig().homed()
    assert "error:15" in r.send("$J=G91 X-400 F1000\n")
    assert "ok" in r.send("$J=G91 X-100 F1000\n", 1.0)
    assert r.ctl.state == "Jog"
    r.send("\x85", 0.5)
    assert r.ctl.state == "Idle"
    x = r.ctl.planner.position[0]
    assert -101 < x < -2
    r.run(2.0)
    assert r.ctl.planner.position[0] == x       # the rest of the jog was flushed


def test_feed_hold_and_cycle_start():
    r = Rig().homed()
    r.send("G1 X-200 F600\n", 2.0)
    r.send("!", 0.01)
    assert r.ctl.state == "Hold:1"
    r.run(1.0)
    assert r.ctl.state == "Hold:0"
    held = r.ctl.planner.position[0]
    r.run(1.0)
    assert r.ctl.planner.position[0] == held
    r.send("~", 0.01)
    r.until_idle()
    assert round(r.ctl.planner.position[0], 3) == -200.0


def test_reset_during_motion_loses_position():
    r = Rig().homed()
    r.send("G0 X-200\n", 0.5)
    out = r.send("\x18")
    assert "ALARM:3" in out and not r.ctl.homed and r.ctl.state == "Alarm"


def test_short_rail_trips_hard_limit_inside_the_soft_envelope():
    # $130 says 300 mm but the rail is 290: soft limits pass, the switch trips.
    r = Rig(rail_lengths={"x": 290.0}).homed()
    assert abs(r.ctl.twin_report()["rails"]["x"]["offset_mm"] + 10.0) < 1e-6
    out = r.send("G53 G0 X-295\n") + r.until_idle(20)
    assert "ALARM:1" in out
    assert any(e["kind"] == "hard_limit" for e in r.ctl.events)


def test_never_homed_machine_stalls_and_loses_steps():
    m = machine.load(EXAMPLE)
    for k in (20, 21, 22):
        m.apply_setting(k, 0)
    r = Rig(m, initial_s=[150, 90, 22.5])       # GRBL believes MPos 0 at mid-rail
    out = r.send("G0 X-200\n", 0.01) + r.until_idle()
    assert "ALARM" not in out
    rep = r.ctl.twin_report()
    assert rep["belief_mpos"][0] == -200.0
    assert rep["rails"]["x"]["lost_steps_mm"] < -40            # 50 mm driven into the stop
    assert rep["stalls"] and rep["stalls"][0]["axis"] == "x"


def test_arcs_full_circle_and_bad_endpoint():
    it = gcode.Interpreter()
    it.execute("G1 X10 Y0 F500")
    blocks = it.execute("G2 X10 Y0 I-10 J0")
    assert len(blocks) > 20 and blocks[-1].target == (10.0, 0.0, 0.0)
    # Every segment end lies on the circle within the arc tolerance.
    assert all(abs(math.hypot(b.target[0], b.target[1]) - 10) < 0.003 for b in blocks)
    with pytest.raises(gcode.GcodeError) as exc:
        it.execute("G2 X20 Y0 I-10 J0")
    assert exc.value.code == 33


def test_interpreter_errors_and_units():
    it = gcode.Interpreter()
    for line, code in (("G1 X10", 22), ("G0 G1 X1", 21), ("G38.2 Z-5 F10", 20), ("X1 X2", 25)):
        with pytest.raises(gcode.GcodeError) as exc:
            it.execute(line)
        assert exc.value.code == code, line
    it.execute("G20 G0 X1")
    assert it.position[0] == pytest.approx(25.4)
    it.execute("G21 G10 L20 P1 X0")
    assert it.work_position()[0] == pytest.approx(0.0)


def test_settings_and_reports():
    r = Rig()
    out = r.send("$$\n")
    assert "$130=300.000" in out and "$22=1" in out
    assert "ok" in r.send("$110=1500\n") and r.ctl.machine.axes["x"].max_rate == 1500
    assert "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]" in r.send("$G\n")
    assert "[VER:1.1h.20190825:OmniSim twin]" in r.send("$I\n")
    first = r.send("?")
    assert "WCO:" in first


def test_telnet_negotiation_cannot_soft_reset():
    state = {}
    # IAC DO TERMINAL-TYPE (0x18), then "?".
    assert server.strip_telnet(bytes([0xFF, 0xFD, 0x18]) + b"?", state) == b"?"


def test_tcp_round_trip():
    r = Rig()
    srv = server.StreamServer(r.ctl, port=0)
    port = srv.start()
    try:
        c = socket.create_connection(("127.0.0.1", port), timeout=2)
        time.sleep(0.1)
        c.sendall(b"$I\n")
        deadline = time.time() + 2
        data = b""
        while time.time() < deadline and b"[OPT:" not in data:
            r.ctl.tick(0.01)
            c.settimeout(0.05)
            try:
                data += c.recv(4096)
            except socket.timeout:
                pass
        assert b"Grbl 1.1h" in data and b"[VER:1.1h" in data
        c.close()
    finally:
        srv.stop()


JOB = """(test job)
G21 G90 G54
G0 Z5
G0 X10 Y10
M3 S10000
G1 Z-1 F200
G1 X60 F800
G3 X60 Y40 I0 J15
G1 X10
G1 Y10
G0 Z5
M5
M30
"""


def test_preflight_passes_a_job_inside_the_envelope():
    m = machine.load(EXAMPLE)
    rep = preflight.preflight(m, JOB, wco=[-200, -120, -40])
    assert rep["verdict"] == "pass", rep
    # Bounds cover the true path, including the first moves away from home (X -1 = work 199).
    assert rep["bounds"]["x"]["work"] == [10.0, 199.0]
    assert rep["program_end"]
    assert 15 < rep["estimated_seconds"] < 60


def test_preflight_flags_envelope_feed_and_spindle_problems():
    m = machine.load(EXAMPLE)
    # Work X260 is machine +60: past the envelope's 0 edge.
    bad = JOB.replace("X60", "X260").replace("F800", "F5000").replace("M3 S10000", "")
    rep = preflight.preflight(m, bad, wco=[-200, -120, -40])
    assert rep["verdict"] == "fail" and rep["error_count"] == 0
    assert rep["envelope_violations"][0]["axis"] == "x"
    assert rep["warning_counts"]["feed_clamped"] >= 1
    assert rep["warning_counts"]["feed_with_spindle_off"] >= 1
    no_offset = preflight.preflight(m, JOB)
    assert not no_offset["envelope_checked"] and no_offset["notes"]


def test_preflight_time_matches_the_executed_job():
    m = machine.load(EXAMPLE)
    estimate = preflight.preflight(m, JOB, wco=[-200, -120, -40])["estimated_seconds"]
    r = Rig().homed()
    r.send("G10 L2 P1 X-200 Y-120 Z-40\n")
    start = r.ctl.time
    r.send(JOB, 0.01)
    r.until_idle()
    assert abs((r.ctl.time - start) - estimate) < 0.15 * estimate + 1.0


def _synthetic_assembly(path):
    """A named-part gantry assembly in millimetres, the way a CAD export arrives."""
    import trimesh
    scene = trimesh.Scene()
    parts = {
        "Frame_Base": ((600, 400, 40), (0, 0, 20)),
        "Gantry_Beam": ((560, 60, 60), (0, -50, 300)),
        "X_Carriage_Plate": ((90, 20, 120), (-100, -90, 300)),
        "Z_Axis_Spindle": ((60, 60, 160), (-100, -130, 220)),
    }
    for name, (size, at) in parts.items():
        box = trimesh.creation.box(extents=size)
        box.apply_translation(at)
        box.visual.face_colors = [200, 120, 40, 255] if "Spindle" in name else [90, 90, 100, 255]
        scene.add_geometry(box, node_name=name, geom_name=name)
    scene.export(str(path))


def test_cad_pack_cuts_groups_and_world_offsets_bodies_from_the_cad_pose(tmp_path):
    import cad_pack
    _synthetic_assembly(tmp_path / "machine.glb")
    manifest = {
        "name": "synthetic", "units": "mm", "up": "z", "parts": "machine.glb",
        "source": {"url": "test", "licence": "CC0", "attribution": "test"},
        "groups": {"gantry": ["(?i)gantry"], "carriage": ["(?i)x_carriage"], "z": ["(?i)z_axis"]},
        "bodies": [{"group": "gantry", "axis": "y", "parent": "frame"},
                   {"group": "carriage", "axis": "x", "parent": "gantry"},
                   {"group": "z", "axis": "z", "parent": "carriage"}],
        "axis_dirs": {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]},
        "cad_pose_s": [100, 50, 40],
    }
    pack = cad_pack.build_pack(manifest, tmp_path, tmp_path / "pack", tmp_path)
    assert {g: v["parts"] for g, v in pack["groups"].items()} == {"frame": 1, "gantry": 1, "carriage": 1, "z": 1}
    assert pack["extent_m"] == pytest.approx([0.6, 0.4, 0.36], abs=1e-6)       # mm -> m, standing on z=0
    assert pack["groups"]["frame"]["bbox"][0][2] == pytest.approx(0.0)
    assert (tmp_path / "pack" / pack["groups"]["z"]["meshes"][0]["file"]).is_file()
    m = machine.load(EXAMPLE)
    text = world.build_pack_world(m, "cfg.grbl", pack, "pack", s0=[150, 90, 40])
    assert text.count("SliderJoint") == 3 and text.count("Mesh {") == 4
    # The gantry sits 40 mm further along +Y than in the CAD, the carriage 50 mm along +X.
    assert 'DEF GANTRY Solid { name "gantry" translation 0 0.04 0' in text
    assert 'DEF CARRIAGE Solid { name "carriage" translation 0.05 0 0' in text
    assert 'DEF Z Solid { name "z" translation 0 0 0' in text
    with pytest.raises(cad_pack.PackError):
        cad_pack.build_pack({**manifest, "groups": {**manifest["groups"], "z": ["nomatch"]}},
                            tmp_path, tmp_path / "bad", tmp_path)


def test_world_generator_emits_three_sliders_and_the_controller():
    m = machine.load(EXAMPLE)
    text = world.build_world(m, "projects/manufacturing/machines/example_3018/example_3018_with_switches.grbl")
    assert text.count("SliderJoint") == 3
    for name in ("x_motor", "y_motor", "z_motor", 'controller "grbl_twin"', "OmniSimSky"):
        assert name in text
    # The window is capped while the machine moves (an unchanged frame is held anyway).
    assert f"FPS {world.RENDER_FPS} }}" in text
    assert "field    SFFloat                       FPS " in (
        SERVICE.parents[2] / "resources/nodes/WorldInfo.wrl").read_text(encoding="utf-8")
    checked = SERVICE.parents[1] / "samples/demos/worlds/manufacturing/cnc_3018_twin.omniworld"
    assert checked.read_text(encoding="utf-8") == world.build_world(
        m, "projects/manufacturing/machines/example_3018/example_3018_with_switches.grbl", s0=[120, 60, 30])


def test_longmill_world_matches_its_pack_and_config():
    import json
    pack_dir = MACHINES / "longmill_mk2_30x30/cad"
    pack = json.loads((pack_dir / "pack.json").read_text(encoding="utf-8"))
    for group in pack["groups"].values():
        for entry in group["meshes"]:
            assert (pack_dir / entry["file"]).is_file(), entry["file"]
            assert entry["faces"] <= 21_845           # the renderer's shadow limit
    assert pack["faces"] <= 400_000
    assert (pack_dir / "PROVENANCE.md").is_file()     # CC BY-SA meshes need their own trail
    m = machine.load(MACHINES / "longmill_mk2_30x30/longmill_mk2_30x30.grbl")
    assert [m.axes[a].travel for a in "xyz"] == [810.0, 855.0, 120.0]
    assert not (m.homing or m.soft_limits or m.hard_limits)    # as Sienci ships it
    checked = SERVICE.parents[1] / "samples/demos/worlds/manufacturing/cnc_longmill_mk2_twin.omniworld"
    text = world.build_pack_world(
        m, "projects/manufacturing/machines/longmill_mk2_30x30/longmill_mk2_30x30.grbl", pack,
        "../../../../manufacturing/machines/longmill_mk2_30x30/cad", s0=[394, 441, 100],
        stock={"size": [180, 180, 12], "spacing": 1.0, "tool_d": 6.35, "renderer": "native",
               "resolution_mm": 0.2})
    assert checked.read_text(encoding="utf-8") == text
    # The block is centred under the bit at power-up, sits on the spoilboard, and is the
    # engine's MillingStock carved by the Solid that carries the router (body "z").
    assert '"--stock=-5.773,-151.818,19.5,180,180,12,1"' in text and "supervisor TRUE" in text
    assert ("DEF STOCK MillingStock { translation -0.005773 -0.151818 0.0195 size 0.18 0.18 0.012 "
            "resolution 0.0002") in text
    assert 'toolSolid "z" toolTip 0.084227 -0.061818 0.030274 toolShape "flat" toolDiameter 0.00635' in text
    grid = world.build_pack_world(m, "cfg", pack, "p", s0=[394, 441, 100],
                                  stock={"size": [180, 180, 12], "spacing": 1.0, "renderer": "grid"})
    assert "DEF STOCK_GRID ElevationGrid { xDimension 181 yDimension 181" in grid


def test_stock_cut_is_a_tool_wide_slot_at_the_cut_depth():
    import stock as stock_mod
    s = stock_mod.Stock(100, 80, 12, 1.0)
    s.cut((20, 40, 9), (80, 40, 9), 3.175)
    assert s.depth_at(50, 40) == pytest.approx(3.0)
    assert s.depth_at(50, 43) == pytest.approx(3.0) and s.depth_at(50, 44) == 0.0   # radius 3.175
    assert s.depth_at(10, 40) == 0.0
    assert s.h[0, 50] == 0.0 and s.h[40, 0] == 0.0          # pinned border draws the sides
    s.cut((50, 10, 2), (50, 70, 2), 3.175, spindle_on=False)
    assert s.summary()["dry_cut_count"] == 1                  # plunging with the spindle stopped


def test_the_twin_mills_the_star_pocket_job():
    """End to end, engine-free: GRBL runs the CAM job and the tool carves the stock."""
    import stock as stock_mod
    import toolpath
    job = toolpath.pocket(toolpath.star_in_circle(), 6.35, 3.0, 5.0, 3.0, feed=3000)
    m = machine.load(MACHINES / "longmill_mk2_30x30/longmill_mk2_30x30.grbl")
    r = Rig(m)
    block = stock_mod.Stock(180, 180, 12, 1.0)
    r.send("G10 L20 P1 X0 Y0 Z20\n", 0.05)          # tool 20 mm above the stock top, over its centre
    wco = r.ctl.interp.work_offset()
    r.ctl.feed(job.encode())
    last = None
    for _ in range(int(400 / 0.02)):
        r.ctl.tick(0.02)
        w = [r.ctl.planner.position[i] - wco[i] for i in range(3)]
        tip = (w[0] + 90, w[1] + 90, w[2] + 12)
        if last is not None:
            block.cut(last, tip, 3.175, spindle_on=r.ctl.spindle[0] != "off")
        last = tip
        if r.ctl.state == "Idle" and r.ctl.planner.idle() and r.ctl._wait_block is None and not r.ctl._lines:
            break
    assert "[MSG:Pgm End]" in "".join(r.buf)
    assert block.depth_at(90, 90) == 0.0                       # star centre stands
    assert block.depth_at(90, 90 + 30) == 0.0                  # inside the top star point
    assert block.depth_at(90 + 55, 90) == pytest.approx(3.0)   # pocket floor between star and wall
    assert block.depth_at(90 - 60, 90 - 20) == pytest.approx(3.0)
    assert block.depth_at(90 + 75, 90) == 0.0                  # outside the 70 mm circle
    assert block.depth_at(5, 5) == 0.0
    summary = block.summary()
    assert summary["deepest_mm"] == pytest.approx(3.0) and summary["dry_cut_count"] == 0
    # Removed volume is the pocket: (circle - star) x 3 mm, within the height map's resolution.
    expected = toolpath.star_in_circle().area * 3.0
    assert abs(summary["removed_mm3"] - expected) < 0.08 * expected


def test_world_writes_only_fields_the_millingstock_node_declares():
    """world.py and the engine's MillingStock model must not drift apart."""
    import re
    model = (SERVICE.parents[2] / "resources/nodes/MillingStock.wrl").read_text(encoding="utf-8")
    declared = set(re.findall(r"^\s*(?:field|hiddenField)\s+\w+\s+(\w+)", model, re.M))
    assert {"translation", "size", "resolution", "toolSolid", "toolTip", "toolShape", "toolDiameter",
            "removedVolume", "deepestCut"} <= declared
    import json
    pack = json.loads((MACHINES / "longmill_mk2_30x30/cad/pack.json").read_text(encoding="utf-8"))
    m = machine.load(MACHINES / "longmill_mk2_30x30/longmill_mk2_30x30.grbl")
    text = world.build_pack_world(m, "cfg", pack, "p", s0=[394, 441, 100],
                                  stock={"size": [180, 180, 12], "tool_shape": "vbit"})
    body = re.search(r"MillingStock \{(.*?)\}", text, re.S).group(1)
    written = set(re.findall(r"\b([a-zA-Z]+)\s+(?:\"|-?\d)", body))
    assert written <= declared, written - declared
    assert 'toolShape "vbit"' in body


def _write_zmap(path, cut_region, depth_mm, size_mm=180.0, res_mm=0.2, top_mm=12.0, dip=None):
    """A MillingStock height-map dump of `cut_region` (work XY = block centre) pocketed flat."""
    import struct

    import numpy as np
    import shapely

    n = int(round(size_mm / res_mm)) + 1
    xs = np.arange(n) * res_mm - size_mm / 2
    X, Y = np.meshgrid(xs, xs)
    depth = np.where(shapely.contains_xy(cut_region, X, Y), depth_mm, 0.0)
    if dip is not None:  # (x, y, radius, extra depth) mm
        depth += np.where(np.hypot(X - dip[0], Y - dip[1]) < dip[2], dip[3], 0.0) * (depth > 0)
    heights = ((top_mm - depth) / 1000.0).astype("<f4")
    path.write_bytes(b"OMZMAP1\0" + struct.pack("<2i", n, n)
                     + struct.pack("<6d", res_mm / 1000, top_mm / 1000, 0, 0, 0, 1e-5) + heights.tobytes())


def test_inspect_part_measures_walls_floor_and_untouched_stock(tmp_path):
    import inspect_part
    import toolpath

    # The reader matches the layout the engine writes.
    src = (SERVICE.parents[2] / "src/omnisim/nodes/OmMillingStock.cpp").read_text(encoding="utf-8")
    assert 'file.write("OMZMAP1", 8);' in src
    drawing = toolpath.star_in_circle()
    reachable = drawing.buffer(-3.175, quad_segs=64).buffer(3.175, quad_segs=64)

    perfect = tmp_path / "perfect.zmap"
    _write_zmap(perfect, reachable, 3.0)
    rep = inspect_part.inspect(inspect_part.load_zmap(perfect), drawing, 3.0, 6.35, [90.0, 90.0])
    assert rep["inside_corner_material_mm2"] > 0  # a round tool leaves the star's inside corners
    assert rep["floor_error_mm"]["max"] < 1e-4 and rep["untouched_depth_mm"]["max"] < 1e-4
    assert rep["wall_out"]["max_mm"] < 1e-3 and rep["wall_in"]["max_mm"] < 1e-3

    # Walls 0.1 mm oversize and one 0.05 mm dip in the floor are both found and sized.
    flawed = tmp_path / "flawed.zmap"
    _write_zmap(flawed, reachable.buffer(0.1, quad_segs=64), 3.0, dip=(-50.0, 0.0, 3.0, 0.05))
    rep = inspect_part.inspect(inspect_part.load_zmap(flawed), drawing, 3.0, 6.35, [90.0, 90.0])
    assert 0.08 < rep["wall_out"]["max_mm"] <= 0.1
    assert rep["wall_in"]["points"] == 0
    assert abs(rep["floor_error_mm"]["max"] - 0.05) < 1e-4
