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

"""SparkRover Mini electrical design: netlist, pins, ratings, placement, contract (engine-free)."""
import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
sys.path.insert(0, str(PRODUCT))
_spec = importlib.util.spec_from_file_location("spark_rover_electrical", PRODUCT / "electrical.py")
el = importlib.util.module_from_spec(_spec)
sys.modules["spark_rover_electrical"] = el
_spec.loader.exec_module(el)
import skeleton as sk  # noqa: E402


@pytest.fixture(scope="module")
def rules():
    return el.rules()


def test_every_rule_passes(rules):
    failed = {k: v for k, v in rules.items() if not v["ok"]}
    assert not failed, failed


@pytest.mark.parametrize("name", [
    "every_net_physically_connected", "no_loops_in_any_net", "every_pin_on_a_net_or_no_connect",
    "no_5v_output_on_a_3v3_gpio", "servo_rail_within_servo_limits", "pack_never_on_the_servos",
    "servo_fault_limited_by_its_regulator", "pi_gets_its_5a_supply", "motoron_survives_a_stall",
    "fuse_clears_a_short_fast", "fuse_interrupts_the_short", "wires_within_ampacity",
    "solids_inside_electrical_zones", "solids_closed", "no_interference_between_our_parts",
    "screen_active_area_centred_in_face_window", "camera_lens_on_the_camera_axis", "mic_ports_under_mic_holes",
    "cables_long_enough"])
def test_the_rules_that_matter_are_checked(rules, name):
    assert name in rules and rules[name]["ok"], rules.get(name)


def test_checks_follow_the_contract_form():
    c = el.checks()
    assert c and all(isinstance(ok, bool) and isinstance(d, str) for ok, d in c.values())


def test_netlist_integrity():
    comps, nets = el.components(), el.netlist()
    pins = [p for ps in nets.values() for p in ps]
    assert not [p for p, n in Counter(pins).items() if n > 1], "a pin is on two nets"
    every = {(ref, p) for ref, c in comps.items() for p in c["pins"]}
    assert set(pins) <= every
    assert set(pins) | set(el.NO_CONNECT) == every


def test_servos_never_see_the_pack_and_the_pi_has_its_own_regulator():
    nets = el.netlist()
    assert ("U6", "VOUT") in nets["SERVO_5V"] and not any(r == "U5" for r, _ in nets["SERVO_5V"])
    assert ("U1", "5V.2") in nets["5V"] and ("U5", "VOUT") in nets["5V"]
    for net in ("VBAT", "VBAT_RAW", "VSW"):
        assert not any(r in ("JS1", "JS2") for r, _ in nets[net])
    assert el.PACK["v_end"] > el.SERVO["v_abs"][1]       # why the regulator is needed at all


def test_pin_assignment():
    nets = el.netlist()
    for k in range(4):        # PIO quadrature: A and B on adjacent pins
        assert ("U2", f"GP{6 + 2 * k}") in nets[f"ENC{k + 1}_A_3V3"] and ("U2", f"GP{7 + 2 * k}") in nets[f"ENC{k + 1}_B_3V3"]
    assert ("U2", "GP20") in nets["I2C0_SDA"] and ("U2", "GP21") in nets["I2C0_SCL"]
    assert ("U1", "GPIO18") in nets["PCM_CLK"] and ("U1", "GPIO21") in nets["PCM_DOUT"]
    for k in range(1, 5):     # every encoder reaches the Pico only through the LVC245
        for c in ("A", "B"):
            assert not any(r in ("U1", "U2") for r, _ in nets[f"ENC{k}_{c}"])


def test_positions_cover_every_wire_end_and_lengths_estimated():
    pos = el.positions()
    wires = el.wire_list()
    assert len({w["id"] for w in wires}) == len(wires)
    for w in wires:
        assert w["from"] in pos and w["to"] in pos
        assert w["colour"] in el.WIRE_RGB
        if w["kind"] == "wire":
            assert w["awg"] in el.AWG and w["length_mm"] >= w["straight_mm"] and w["length_source"] == "estimate"
        else:
            assert w["length_source"] == "bought" and w["length_mm"] >= w["route_mm"]
    assert all(w["colour"] == "black" for w in wires if w["net"] == "GND")
    routed = el.wire_list(None, {"W10": 77.0})
    assert next(w for w in routed if w["id"] == "W10")["length_mm"] == 77.0


def test_interfaces_are_met():
    pl = el.placements()
    lens = pl["U9"].at(12.5, el.CAMERA_MOD["lens_from_far_edge"], 8.9)
    assert math.isclose(lens[1], sk.CAMERA["lens_at"][1], abs_tol=1e-6) and math.isclose(lens[2], sk.CAMERA["lens_at"][2], abs_tol=1e-6)
    usb = pl["CHG1"].at(-4.2, 9.0, 3.2)
    assert math.isclose(usb[1], sk.USB_C["at"][1], abs_tol=1e-6) and math.isclose(usb[2], sk.USB_C["at"][2], abs_tol=1e-6)
    d = el.solids()["el_U8"].reshape(-1, 3)
    assert abs((d[:, 2].min() + d[:, 2].max()) / 2 - sk.FACE_WINDOW["centre"][2]) < 1e-6


def test_power_budget_and_runtime():
    pb = el.power_budget()
    T = {(op["case"], op["state"]): op for op in pb["table"]}
    assert T[("idle", "nominal")]["i_bat"] < T[("driving", "nominal")]["i_bat"] < T[("motor_stall", "nominal")]["i_bat"]
    assert T[("servo_stall", "full")]["i6"] <= el.REG6["i_fault"]
    assert 1.5 < pb["runtime"]["idle"]["hours"] < 6.0
    assert 0.3 < pb["runtime"]["driving"]["hours"] < pb["runtime"]["classroom mix"]["hours"] < pb["runtime"]["idle"]["hours"]
    assert el.FUSE["i_rated"] * 3.5 <= pb["short_circuit_a"] <= el.FUSE["i_interrupt"]
    h = el.heat()
    assert h["idle"]["p_bay_w"] < h["driving"]["p_bay_w"] < h["everything"]["p_bay_w"]


def test_chassis_constant_drives_the_stall_figures(monkeypatch):
    base = max(op["i_motor_channel"] for op in el.power_budget()["table"] if op["case"] == "motor_stall")
    monkeypatch.setitem(el.MOTOR, "r_ohm", el.MOTOR["r_ohm"] / 2)
    hot = max(op["i_motor_channel"] for op in el.power_budget()["table"] if op["case"] == "motor_stall")
    assert hot > 1.5 * base


def test_contract_parts_solids_mounts_jobs():
    import trimesh
    parts, sol, jobs = el.parts(), el.solids(), el.jobs()
    labels = {p["label"] for p in parts}
    for p in parts:
        assert p["kind"] in ("made", "bought") and p["process"] in ("print", "bought") and len(p["centre_mm"]) == 3
    assert set(sol) <= labels | {"el_pi_standoffs"}
    for k, tris in sol.items():
        assert tris.ndim == 3 and tris.shape[1:] == (3, 3)
        m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(tris))
        assert all(b.is_watertight for b in m.split(only_watertight=False)), k
    made = {p["label"] for p in parts if p["kind"] == "made"}
    assert made == set(jobs)
    for text in jobs.values():
        assert text.startswith("solid ") and "facet normal" in text
    names = {m.name for m in el.mounts()}
    assert {"bay_pi5_0", "bay_pi5_3", "bay_row_0", "bay_row_4"} <= names
    assert all(m.holder == "electrical" for m in el.mounts())
    total = sum(p["mass_g"] for p in parts)
    assert 300.0 < total < 700.0


def test_solids_in_zones_except_interface_parts(rules):
    assert rules["solids_inside_electrical_zones"]["ok"], rules["solids_inside_electrical_zones"]
    sol = el.solids()
    for ref in el.INTERFACE_PARTS:
        if ref != "J_TOP":
            assert f"el_{ref}" in sol, ref
    ts = sk.TOP_SLOT
    v = sol["el_B3"].reshape(-1, 3)
    assert ts.x0 <= v[:, 0].min() and v[:, 0].max() <= ts.x1 and ts.z0 <= v[:, 2].min() and v[:, 2].max() <= ts.z1


def test_svgs_parse_and_name_every_part():
    sch, wir = el.schematic_svg(), el.wiring_svg()
    for text in (sch, wir):
        assert ET.fromstring(text).tag.endswith("svg")
    for ref, c in el.components().items():
        if c["pins"]:
            assert f">{ref}<" in sch, ref
    for w in el.wire_list():
        assert f">{w['id']}<" in wir


def test_write_produces_the_release_files(tmp_path):
    result = el.write(tmp_path)
    assert all(ok for ok, _ in result.values())
    for name in ("schematic.svg", "wiring.svg", "WIRES.md", "POWER.md", "PLACEMENT.md"):
        assert (tmp_path / name).stat().st_size > 1000, name
    assert "| W1 |" in (tmp_path / "WIRES.md").read_text(encoding="utf-8")


def test_vendor_cad_is_pinned():
    for src in el.VENDOR_CAD.values():
        assert src["url"].startswith("https://datasheets.raspberrypi.com/") and len(src["sha256"]) == 64
