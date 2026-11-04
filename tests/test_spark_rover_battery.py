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

"""SparkRover Mini battery model (battery.py): curve, sag, cut-off, runtimes, energy balance (engine-free)."""
import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
_spec = importlib.util.spec_from_file_location("spark_rover_battery", PRODUCT / "battery.py")
bt = importlib.util.module_from_spec(_spec)
sys.modules["spark_rover_battery"] = bt
_spec.loader.exec_module(bt)


@pytest.fixture(scope="module")
def el():
    return bt._electrical()


@pytest.fixture(scope="module")
def runtimes():
    return bt.validate_runtimes()


def test_mirrored_constants_are_electricals(el):
    assert bt.CELL_C_TYP_MAH == el.CELL["c_typ_mah"]
    assert bt.CELL_V_CHARGE == el.CELL["v_charge"] and bt.CELL_V_NOM == el.CELL["v_nom"]
    assert (bt.SERIES, bt.PARALLEL) == (el.PACK["series"], el.PACK["parallel"])
    assert bt.PACK_V_CUT == el.PACK["v_end"] and bt.PACK_V_NOM == el.PACK["v_nom"]
    assert bt.BMS_R_OHM == el.BMS["r"]
    assert (bt.REG5_VOUT, bt.REG5_ETA, bt.REG5_IQ) == (el.REG5["vout"], el.REG5["eta"], el.REG5["iq"])
    assert (bt.REG6_VOUT, bt.REG6_ETA) == (el.REG6["vout"], el.REG6["eta"])
    assert bt.MOTORON_I_VIN_Q == el.MOTORON["i_vin_q"] and bt.SWITCH_I_ON_PER_V == el.SWITCH["i_on_per_v"]
    assert "rosebatteries.com/hubfs/INR18650P28A-V2-80093.pdf" in el.CELL["source"]
    assert "rosebatteries.com/hubfs/INR18650P28A-V2-80093.pdf" in bt.SOURCE


def test_curve_is_monotone_and_ends_where_the_data_sheet_says():
    for _, v_lo, v_hi in bt.CURVE_TWO_RATES:
        assert v_lo > v_hi                      # more current, lower voltage, at every point
    qs = [r[0] for r in bt.CURVE_TWO_RATES] + [r[0] for r in bt.CURVE_LOW_RATE_TAIL]
    assert qs == sorted(qs)
    socs = [i / 1000 for i in range(1001)]
    ocv = [bt.cell_ocv(s) for s in socs]
    assert all(b > a for a, b in zip(ocv, ocv[1:]))            # strictly rising with SOC
    assert all(bt.cell_r1(s) >= 0.0 for s in socs)
    assert 4.18 < bt.cell_ocv(1.0) <= bt.CELL_V_CHARGE         # charged to 4.2 V, rested
    assert 3.6 < bt.cell_ocv(0.5) < 3.75                      # the 3.6 V nominal is the mean, near mid-charge
    assert bt.cell_ocv(0.0) < 2.7


def test_model_reproduces_both_fitted_curves_at_steady_current():
    for col, i_cell in ((1, bt.I_LOW), (2, bt.I_HIGH)):
        b = bt.Battery(capacity_mah=bt.FIGURE_CAPACITY_MAH, r_series_ohm=0.0, v_cut=0.0)
        for q, *v in bt.CURVE_TWO_RATES:
            if q < 30.0:
                continue
            while (1.0 - b.soc) * bt.FIGURE_CAPACITY_MAH < q - 1e-9:
                left = (q - (1.0 - b.soc) * bt.FIGURE_CAPACITY_MAH) / 1000.0 / i_cell * 3600.0
                b.step(2 * i_cell, min(1.0, left))
            assert b.v_terminal / 2 == pytest.approx(v[col - 1], abs=0.003), (i_cell, q)


def test_sag_is_i_times_r():
    b = bt.Battery(soc=0.5)
    v0 = b.voltage_at(0.0)
    assert v0 == pytest.approx(b.v_ocv)
    assert b.r_ohmic == pytest.approx(2 * bt.R0_OHM / 2 + bt.BMS_R_OHM)
    for i in (0.5, 2.0, 5.0, 10.0):
        assert v0 - b.voltage_at(i) == pytest.approx(i * b.r_ohmic, rel=1e-12)      # instantaneous: R0 only
    # held: the RC branch adds i x R1 over TAU
    i = 4.0
    v_first = b.step(i, 1e-3)
    assert v0 - v_first == pytest.approx(i * b.r_ohmic, rel=0.01)
    t = 0.0
    while t < bt.TAU_S:
        b.step(i, 0.01)
        t += 0.01
    target = i / 2 * bt.cell_r1(b.soc)
    assert b.v1 == pytest.approx(target * (1 - math.exp(-1.0)), rel=0.03)
    for _ in range(600):
        b.step(i, 0.1)
    steady = b.v_ocv - b.v_terminal
    assert steady == pytest.approx(i * b.r_ohmic + 2 * i / 2 * bt.cell_r1(b.soc), rel=0.02)
    # and it recovers when the load goes
    v_rest = b.step(0.0, 10 * bt.TAU_S)
    assert v_rest == pytest.approx(b.v_ocv, abs=1e-3)


def test_cut_off_triggers_and_latches():
    b = bt.Battery()
    prev = b.v_terminal
    while not b.cut_off:
        prev, v = b.v_terminal, b.step(4.0, 1.0)
        assert b.time_s < 5 * 3600
    assert v <= bt.PACK_V_CUT < prev
    assert 0.0 < b.soc < 0.2
    assert b.step(0.0, 60.0) > bt.PACK_V_CUT        # the voltage recovers at rest...
    assert b.cut_off                                 # ...and the cut-off stays
    # a hold time rides through a short dip
    h = bt.Battery(soc=0.12, cut_off_hold_s=2.0)
    h.step(30.0, 0.5)
    assert h.v_terminal <= bt.PACK_V_CUT and not h.cut_off
    h.step(0.5, 0.5)
    h.step(30.0, 0.5)
    assert not h.cut_off
    for _ in range(4):
        h.step(30.0, 0.5)
    assert h.cut_off


def test_pack_load_reproduces_electricals_operating_points(el):
    for case, p in bt.design_profiles()["cases"].items():
        i = bt.pack_load_a(p["motor_currents_a"], p["electronics_a"], servo_a=p["servo_a"], v_bus=p["design_v_bus"])
        assert i == pytest.approx(p["design_i_bat"], rel=1e-6), case
    # duty scales the motors' share; the regulators' share rises as the bus falls
    assert bt.pack_load_a([1.0, 1.0], 0.0, duties=[0.5, 0.25], v_bus=7.2) == pytest.approx(0.75 + bt.quiescent_a(7.2))
    lo, hi = bt.pack_load_a([], 2.0, v_bus=6.4), bt.pack_load_a([], 2.0, v_bus=8.4)
    assert lo - bt.quiescent_a(6.4) == pytest.approx(10.0 / bt.REG5_ETA / 6.4)
    assert lo > hi
    with pytest.raises(ValueError):
        bt.pack_load_a([1.0], 0.0, duties=[1.0, 1.0])


def test_runtimes_within_band(runtimes, el):
    # the model's own validated numbers, +-1 %
    for k, h in bt.MODEL_RUNTIME_H.items():
        assert runtimes[k]["hours"] == pytest.approx(h, rel=0.01), k
    # against electrical's budget: the model is 2-10 % shorter (README "Battery model": the design counts
    # 95 % of 41.2 Wh; the curve leaves 10-12 % of the charge below the 3.2 V cut-off at these loads)
    for k, r in runtimes.items():
        assert r["design_hours"] == pytest.approx(el.power_budget()["runtime"][k]["hours"])
        assert 0.90 <= r["ratio"] <= 0.98, (k, r["ratio"])
        assert 0.86 <= r["usable_frac"] <= 0.92, k
    assert runtimes["driving"]["hours"] < runtimes["classroom mix"]["hours"] < runtimes["idle"]["hours"]
    # the model's power at the cells agrees with the budget's power within 2 %: the difference is the energy
    for k, r in runtimes.items():
        assert r["p_chem_w"] == pytest.approx(r["design_p_pack_w"], rel=0.02), k


def test_energy_balance_closes():
    b = bt.Battery()
    n = 0
    drawn = 0.0
    while not b.cut_off:
        i = 3.0 + 2.0 * math.sin(n / 50.0) + (6.0 if n % 97 < 9 else 0.0)    # a varying load with bursts
        b.step(i, 0.5)
        drawn += i * 0.5 / 3600.0
        n += 1
    assert b.charge_ah_used == pytest.approx(drawn, rel=1e-12)                     # coulomb counting
    assert b.soc == pytest.approx(1.0 - drawn / 2 / b.capacity_ah, rel=1e-12)
    assert b.energy_wh_chem == pytest.approx(b.energy_wh_used + b.loss_wh, rel=1e-12)
    # the chemical energy is the OCV curve's integral over the charge drawn
    m = 20000
    integral = sum(bt.cell_ocv(1.0 - (k + 0.5) / m * (1.0 - b.soc)) for k in range(m)) / m * (1.0 - b.soc)
    assert b.energy_wh_chem == pytest.approx(2 * 2 * b.capacity_ah * integral, rel=1e-3)
    assert 0.0 < b.loss_wh < 0.05 * b.energy_wh_chem
    # a full 0.2 C discharge to the data sheet's 2.5 V delivers its 10.3 Wh a cell (typical), within 2 %
    f = bt.Battery(r_series_ohm=0.0, v_cut=2 * 2.5)
    while not f.cut_off:
        f.step(2 * bt.I_LOW, 5.0)
    assert f.energy_wh_used / 4 == pytest.approx(10.3, rel=0.02)
    assert f.charge_ah_used / 2 == pytest.approx(bt.CELL_C_TYP_MAH / 1000.0, rel=0.01)
    # charging stops at full
    c = bt.Battery(soc=0.99)
    c.step(-5.6, 3600.0)
    assert c.soc == 1.0 and c.charge_ah_used == pytest.approx(-0.056, rel=1e-9)


def test_model_is_conservative_at_high_current():
    for q, vs in bt.CURVE_HIGH_RATES.items():
        if q == 0.0:
            continue
        soc = 1.0 - q / bt.FIGURE_CAPACITY_MAH
        for i, v in zip(bt.HIGH_RATES_A, vs):
            v_model = bt.cell_ocv(soc) - i * (bt.R0_OHM + bt.cell_r1(soc))
            assert -0.5 < v_model - v <= 0.0, (q, i)


def test_drive_controller_rail_loads_match_the_design():
    """The drive controller's rail loads (copied so it never imports electrical.py) are
    battery.design_profiles()'s, and its wiring resistance is electrical's."""
    import ast
    src = (ROOT / "projects/samples/demos/controllers/spark_rover_drive/spark_rover_drive.py").read_text(encoding="utf-8")
    consts = {}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            if node.targets[0].id in ("ELECTRONICS_A", "SERVO_A", "R_EXT_OHM"):
                consts[node.targets[0].id] = ast.literal_eval(node.value)
    d = bt.design_profiles()
    for case, v in d["cases"].items():
        assert consts["ELECTRONICS_A"][case] == pytest.approx(v["electronics_a"], abs=1e-3), case
        assert consts["SERVO_A"][case] == pytest.approx(v["servo_a"], abs=1e-3), case
    assert consts["R_EXT_OHM"] == pytest.approx(d["r_ext_ohm"], abs=1e-3)
