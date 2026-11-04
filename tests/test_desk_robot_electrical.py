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

"""The desk robot's electrical design: netlist integrity, pin assignment, ratings, outputs (engine-free)."""
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/desk_robot"
sys.path.insert(0, str(PRODUCT))

import design  # noqa: E402
import electrical as el  # noqa: E402

R = design.Robot()


@pytest.fixture(scope="module")
def checks():
    return el.checks(R)


def test_every_check_passes(checks):
    failed = {k: v for k, v in checks.items() if not v["ok"]}
    assert not failed, failed


@pytest.mark.parametrize("name", [
    "every_net_has_two_pins", "no_pin_on_two_nets", "every_pin_on_a_net_or_no_connect", "wires_join_pins_of_their_net",
    "every_net_physically_connected", "no_loops_in_any_net", "driver_inputs_on_pwm_capable_gpio",
    "driver_inputs_not_strapping", "vsense_on_adc1", "gpio_high_wakes_nsleep", "driver_asleep_while_mcu_resets",
    "vbus_within_xiao_buck_input", "stall_per_channel_within_peak", "fuse_holds_double_stall_at_23c",
    "fuse_trips_on_short", "bus_within_regulator_input", "new_wires_within_ampacity_at_stall"])
def test_the_rules_that_matter_are_checked(checks, name):
    assert name in checks and checks[name]["ok"], checks.get(name)


def test_netlist_integrity():
    comps, nets = el.components(), el.netlist()
    pins = [p for ps in nets.values() for p in ps]
    assert all(len(ps) >= 2 for ps in nets.values())
    assert not [p for p, n in Counter(pins).items() if n > 1], "a pin is on two nets"
    every = {(ref, p) for ref, c in comps.items() for p in c["pins"]}
    assert set(pins) <= every
    assert set(pins) | set(el.NO_CONNECT) == every and not set(pins) & set(el.NO_CONNECT)


def test_the_xiao_never_sees_the_pack_and_the_driver_sees_the_xiao():
    nets = el.netlist()
    assert ("U3", "5V") in nets["VBUS"] and ("D1", "K") in nets["VBUS"]        # through the Schottky only
    assert ("U3", "BAT+") in el.NO_CONNECT                                       # the LiPo charger pad
    for sig in ("AIN1", "AIN2", "BIN1", "BIN2"):
        assert ("U3", el.PINS[sig]) in nets[sig] and ("U2", sig) in nets[sig]
    assert len(set(el.PINS.values())) == len(el.PINS)


def test_wire_list_shape_and_lengths():
    wires = el.wire_list(R)
    assert len({w["id"] for w in wires}) == len(wires)
    for w in wires:
        assert w["colour"] in el.WIRE_RGB and w["awg"] in el.AWG
        assert w["length_mm"] >= w["straight_mm"] and w["length_source"] == "estimate"
        assert w["connector_from"] and w["connector_to"]
    power = [w for w in wires if w["net"] in ("VBAT_RAW", "VBAT", "VSW")]
    assert all(w["colour"] == "red" for w in power)
    assert all(w["colour"] == "black" for w in wires if w["net"] == "GND")
    routed = el.wire_list(R, {"W6": 123.0})
    w6 = next(w for w in routed if w["id"] == "W6")
    assert w6["length_mm"] == 123.0 and w6["length_source"] == "routed"


def test_no_header_under_a_board():
    """The boards lie in pockets: every wire end on U2/U3 is soldered from the top, every connector is on a wire."""
    for w in el.wire_list(R):
        for end, conn in ((w["from"], w["connector_from"]), (w["to"], w["connector_to"])):
            if end[0] in ("U2", "U3"):
                assert "from the top" in conn, (w["id"], conn)


def test_power_budget_numbers():
    pb = el.power_budget(R)
    w = pb["worst"]
    stall = max(w["stall"]["i_motor_a"], w["stall"]["i_motor_b"])
    assert 0.8 < stall < el.DRV["i_peak"]                          # a stall is real current, under the peak rating
    assert w["stall"]["i_bat"] < el.FUSE["i_hold"] < el.FUSE["i_trip"] < min(pb["short_circuit_a"].values())
    drive = [op for op in pb["table"] if op["motors"] == "free"]
    assert all(0.25 < op["i_bat"] < 0.6 for op in drive)
    for k, v in pb["runtime"].items():
        assert 1.0 < v["hours"] < 40.0, k
    # The firmware's soft start is what keeps the XIAO up on a tired pack.
    assert any("soft start" in op["case"] for op in pb["table"])
    assert any("ramp every start" in x for x in el.warnings(R))


def test_adc_divider_range():
    a = el.adc_sense()
    assert a["adc_at_7v2"] < el.ESP["adc_atten3_mv"] / 1000 < a["full_scale_pack_v"]
    assert a["adc_at_3v2"] > 0.5


def test_svgs_parse_and_name_every_part(tmp_path):
    sch, wir = el.schematic_svg(R), el.wiring_svg(R)
    for text in (sch, wir):
        root = ET.fromstring(text)
        assert root.tag.endswith("svg")
    for ref in el.components():
        assert f">{ref}<" in sch
    for w in el.wire_list(R):
        assert f">{w['id']}<" in wir


def test_write_produces_the_release_files(tmp_path):
    result = el.write(tmp_path)
    assert all(v["ok"] for v in result.values())
    for name in ("schematic.svg", "wiring.svg", "WIRES.md", "POWER.md"):
        assert (tmp_path / name).stat().st_size > 1000, name
    assert "| W1 |" in (tmp_path / "WIRES.md").read_text(encoding="utf-8")
