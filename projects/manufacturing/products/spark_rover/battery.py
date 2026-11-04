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

"""SparkRover Mini: a battery model of its 2S2P Molicel INR-18650-P28A pack, for the controller.

The pack (electrical.PACK): four P28A cells, two in series, two in parallel, a 2S protection
board, and the firmware's cut-off at 3.2 V a cell, 6.4 V a pack.

What is modelled, per cell (the two parallel cells share the current equally, the two series
cells add their voltages):

    v_cell = OCV(soc) - i * R0 - v1          the terminal voltage of one cell
    v1 = i1 * R1(soc),  di1/dt = (i - i1) / TAU     one RC branch: the polarisation that builds in seconds
    soc    = 1 - (charge drawn) / capacity   coulomb counting

and the pack's terminal (P+/P-) is 2 * v_cell - i_pack * r_series, r_series being the protection
board's MOSFETs (electrical.BMS, ASSUMED 10 mohm).

Where the numbers come from: the Molicel P28A product data sheet (the copy electrical.CELL cites,
rosebatteries.com/hubfs/INR18650P28A-V2-80093.pdf, fetched 2026-11-04). Its "Discharge Rate
Characteristics" figure is a vector drawing: each curve is a polyline in the PDF. The points below
were DIGITIZED FROM THAT FIGURE by reading those polylines' coordinates (PyMuPDF get_drawings) and
mapping them through the figure's own axis ticks (0 and 3000 mAh, 0.0 and 5.0 V). Reading the
path data adds less than 1 mAh / 1 mV; the figure's own drawing is the limit: its lines are
0.74 pt wide, +-6 mAh and +-16 mV, so the stated uncertainty of each point is +-10 mAh and
+-20 mV. It is ONE vendor curve, typical, of one cell at 23 C: the data sheet's own spread
(2700 mAh minimum, 2800 typical) is larger than the digitizing error.

From the two curves closest to this robot's loads (0.56 A and 2.8 A per cell; the robot draws
0.5-2 A a cell), at each capacity point q:

    Rss(q) = (V_0.56(q) - V_2.8(q)) / (2.8 - 0.56)      the steady (R0 + R1) resistance
    OCV(q) = V_0.56(q) + 0.56 * Rss(q)                  the curves extrapolated to zero current

so the model reproduces both curves exactly at steady current. R0 and TAU come from the curves'
first 400 mAh, where the gap between them builds from 41 mV (the ohmic step) to 62 mV (ohmic plus
polarisation): a least-squares fit gives R0 18.8 mohm, R1 9.2 mohm, TAU 8.5 s (rms 1.4 mV;
TAU from 3 to 20 s fits within +25 % rms, so TAU is the loosest number here). Two checks
against the data sheet's own figures: the curves' starting points (q = 0, all five currents)
fall on one line, 4.190 V - I x 17.4 mohm; and R0 + R1 (1 - exp(-1 s / TAU)) = 19.8 mohm, the
data sheet's "DC (10A/1s) 20 mohm". R1 is not constant: Rss is 25-33 mohm over most of the discharge and climbs to
212 mohm below about 12 % state of charge (the curves' knee), so cell_r1(soc) = Rss(soc) - R0.

Capacity: the figure's cell gives 2849 mAh to 2.5 V at 0.56 A (0.2 C, the rating current). The
model scales the figure's capacity axis to the data sheet's TYPICAL 2800 mAh (pass
capacity_mah=2700 for the minimum cell, or 2849.4 for the figure's own cell).

What is NOT modelled: temperature (the data sheet's temperature figure is recorded below,
TEMPERATURE_CAPACITY_MAH, as data), ageing (the cycle figure: 80-85 % after 500 cycles at
10-20 A), cell imbalance between the two series pairs (the protection board's balancing), the
protection board's own trips (over-current, under-voltage at its ASSUMED 2.8 V a cell), and
self-heating: above about 5 A a cell the figure's cells run warm and sag LESS than this model's
linear resistance says (the model reads 36-80 mV low at 10 A, 0.13-0.26 V at 20 A, 0.25-0.46 V
at 30 A, between 250 and 2400 mAh): conservative, and only the stall cases reach those currents.
No Peukert term: the 10, 20 and 30 A curves reach 2.5 V at 2728-2778 mAh, no less than the 2.8 A
curve (self-heating), so capacity does not fall with current at a fixed cut-off; what does fall
is the charge above the 3.2 V firmware cut-off, and the sag term gives that.

    python projects/manufacturing/products/spark_rover/battery.py     # the runtime table
"""
from __future__ import annotations

import bisect
import importlib.util
import math
import sys
from pathlib import Path
from typing import Callable, Dict, Optional, Sequence

HERE = Path(__file__).resolve().parent

SOURCE = ("Molicel INR-18650-P28A product data sheet, 'Discharge Rate Characteristics' (23 C; charge CC-CV 2.8 A, "
          "4.2 V, 50 mA; cut-off 2.5 V): https://www.rosebatteries.com/hubfs/INR18650P28A-V2-80093.pdf (electrical.CELL)")
DIGITIZED = ("digitized from the figure's vector polylines through its axis ticks; +-10 mAh, +-20 mV a point (the drawn "
             "line width); one typical cell")

# ---- the data sheet's figure, digitized -------------------------------------------------------
# (capacity drawn mAh, V at 0.56 A, V at 2.8 A): one cell, the figure's own capacity axis.
# The 2.8 A curve ends at 2733.7 mAh (2.487 V); the 23 C curve of the temperature figure is the
# same polyline.
CURVE_TWO_RATES = (
    (0.0, 4.182, 4.141), (10.0, 4.167, 4.111), (25.0, 4.156, 4.094), (50.0, 4.139, 4.077),
    (100.0, 4.114, 4.053), (150.0, 4.095, 4.033), (200.0, 4.078, 4.015), (300.0, 4.042, 3.979),
    (400.0, 4.004, 3.938), (500.0, 3.965, 3.897), (600.0, 3.929, 3.858), (700.0, 3.896, 3.824),
    (800.0, 3.865, 3.794), (900.0, 3.833, 3.764), (1000.0, 3.799, 3.733), (1100.0, 3.764, 3.702),
    (1200.0, 3.731, 3.669), (1300.0, 3.700, 3.637), (1400.0, 3.671, 3.606), (1500.0, 3.645, 3.578),
    (1600.0, 3.621, 3.551), (1700.0, 3.599, 3.526), (1800.0, 3.576, 3.502), (1900.0, 3.550, 3.477),
    (2000.0, 3.519, 3.448), (2100.0, 3.482, 3.419), (2200.0, 3.444, 3.387), (2300.0, 3.415, 3.348),
    (2400.0, 3.358, 3.290), (2450.0, 3.321, 3.248), (2500.0, 3.278, 3.199), (2550.0, 3.227, 3.134),
    (2600.0, 3.167, 3.048), (2650.0, 3.098, 2.926), (2700.0, 3.024, 2.724), (2733.7, 2.962, 2.487),
)
# (capacity drawn mAh, V at 0.56 A) past the 2.8 A curve's end, to the 0.56 A curve's end at 2.5 V
CURVE_LOW_RATE_TAIL = ((2750.0, 2.928), (2800.0, 2.774), (2849.4, 2.498))
FIGURE_CAPACITY_MAH = 2849.4     # the 0.56 A curve's end (2.498 V): SOC 0 of the model
I_LOW, I_HIGH = 0.56, 2.8        # the two curves' currents, A a cell
# The high-rate curves, for the check only (capacity mAh -> V at 10 / 20 / 30 A): the model is
# not fitted to them.
CURVE_HIGH_RATES = {0.0: (4.011, 3.836, 3.668), 250.0: (3.830, 3.647, 3.480), 500.0: (3.745, 3.583, 3.433),
                    1000.0: (3.589, 3.438, 3.298), 1500.0: (3.430, 3.287, 3.155), 2000.0: (3.300, 3.163, 3.042),
                    2400.0: (3.139, 3.016, 2.895)}
HIGH_RATES_A = (10.0, 20.0, 30.0)
# The temperature figure (2.8 A, charged at 23 C), digitized the same way: deg C -> (mAh to 2.5 V,
# mAh above 3.2 V). NOT used by the model. At -40 C the voltage dips to 2.77 V in the first
# 500 mAh before the cell warms itself: a 3.2 V cut-off ends that run at once.
TEMPERATURE_CAPACITY_MAH = {-40: (2235, 153), -30: (2396, 1514), -20: (2465, 1880), 0: (2577, 2330),
                            23: (2732, 2499), 45: (2845, 2530), 60: (2834, 2520)}

# ---- the fitted constants (see the docstring) ---------------------------------------------------
R0_OHM = 0.0188        # ohmic, a cell
R1_HIGH_SOC_OHM = 0.0092   # the RC branch's resistance over the first 400 mAh (from the fit; R1(soc) below)
TAU_S = 8.5            # the RC branch's time constant (fit; 3-20 s within +25 % rms)

# ---- the figures electrical.py owns, mirrored so the controller need not import electrical (it
# imports OpenCascade). tests/test_spark_rover_battery.py pins each one to electrical's value.
CELL_C_TYP_MAH = 2800.0          # electrical.CELL["c_typ_mah"]
CELL_V_CHARGE = 4.2              # electrical.CELL["v_charge"]
CELL_V_NOM = 3.6                 # electrical.CELL["v_nom"]
SERIES, PARALLEL = 2, 2          # electrical.PACK["series"], ["parallel"]
PACK_V_CUT = 6.4                 # electrical.PACK["v_end"]: the firmware's 3.2 V a cell
PACK_V_NOM = 7.2                 # electrical.PACK["v_nom"]
BMS_R_OHM = 0.010                # electrical.BMS["r"] (ASSUMED there: a generic 2S board)
REG5_VOUT, REG5_ETA, REG5_IQ = 5.0, 0.85, 0.4e-3     # electrical.REG5: the D42V55F5, the 5 V rail
REG6_VOUT, REG6_ETA = 5.0, 0.85                      # electrical.REG6: the D36V28F5, the servo rail
MOTORON_I_VIN_Q = 0.005          # electrical.MOTORON["i_vin_q"] (ASSUMED there), per controller, two of them
SWITCH_I_ON_PER_V = 120e-6       # electrical.SWITCH["i_on_per_v"]


def _tables():
    """(soc ascending, OCV, Rss) for one cell, from the digitized curves."""
    rows = []
    for q, v_lo, v_hi in CURVE_TWO_RATES:
        rss = (v_lo - v_hi) / (I_HIGH - I_LOW)
        # at q = 0 neither curve has polarised yet: only R0 has dropped
        ocv = v_lo + I_LOW * (R0_OHM if q == 0.0 else rss)
        rows.append((q, ocv, rss))
    rss_end = rows[-1][2]
    for q, v_lo in CURVE_LOW_RATE_TAIL:
        rows.append((q, v_lo + I_LOW * rss_end, rss_end))   # EXTRAPOLATED: below any load's 3.2 V cut-off
    # the first point's Rss is the ohmic step only; the steady figure is the next one's
    rows[0] = (rows[0][0], rows[0][1], rows[1][2])
    soc = [1.0 - q / FIGURE_CAPACITY_MAH for q, _, _ in rows]
    return soc[::-1], [r[1] for r in rows][::-1], [r[2] for r in rows][::-1]


SOC_TABLE, OCV_TABLE, RSS_TABLE = _tables()


def _interp(xs: Sequence[float], ys: Sequence[float], x: float) -> float:
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    j = bisect.bisect_right(xs, x)
    x0, x1, y0, y1 = xs[j - 1], xs[j], ys[j - 1], ys[j]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def cell_ocv(soc: float) -> float:
    """Open-circuit voltage of one cell at a state of charge (0-1), V."""
    return _interp(SOC_TABLE, OCV_TABLE, soc)


def cell_r1(soc: float) -> float:
    """The RC branch's resistance of one cell at a state of charge: Rss - R0 (it climbs near empty), ohm."""
    return max(_interp(SOC_TABLE, RSS_TABLE, soc) - R0_OHM, 0.0)


class Battery:
    """The 2S2P pack. Currents in A, positive discharging; times in s; voltages in V at P+/P-.

    step(current_a, dt_s) advances the state by dt_s at that pack current and returns the terminal
    voltage at the end of the step. voltage_at(current_a, dt_s) predicts it without advancing (for a
    load that depends on the voltage: iterate it). cut_off latches once the terminal voltage has been
    at or below v_cut for cut_off_hold_s (0: the first sample, as the firmware's simplest check).
    """

    def __init__(self, soc: float = 1.0, *, capacity_mah: float = CELL_C_TYP_MAH, series: int = SERIES,
                 parallel: int = PARALLEL, r_series_ohm: float = BMS_R_OHM, v_cut: float = PACK_V_CUT,
                 cut_off_hold_s: float = 0.0):
        if not 0.0 <= soc <= 1.0:
            raise ValueError(f"soc {soc} outside 0-1")
        self.capacity_ah = capacity_mah / 1000.0       # a cell
        self.series, self.parallel = series, parallel
        self.r_series_ohm, self.v_cut, self.cut_off_hold_s = r_series_ohm, v_cut, cut_off_hold_s
        self._soc = soc
        self.i1 = 0.0                  # the RC branch's state: the cell current filtered by TAU (A; rested: 0)
        self.time_s = 0.0
        self.current_a = 0.0
        self.charge_ah_used = 0.0      # pack charge drawn (Ah, net)
        self.energy_wh_used = 0.0      # energy delivered at P+/P- (Wh, net)
        self.energy_wh_chem = 0.0      # energy drawn from the cells' OCV (Wh): delivered + the losses below
        self.loss_wh = 0.0             # in R0, in the RC branch's resistor and in r_series (Wh)
        self.cut_off = False
        self._below_s = 0.0
        self.v_terminal = self.voltage_at(0.0)

    # ---- state ------------------------------------------------------------------------------
    @property
    def soc(self) -> float:
        return self._soc

    @property
    def v_ocv(self) -> float:
        """The pack's open-circuit voltage at the present state of charge."""
        return self.series * cell_ocv(self._soc)

    @property
    def r_ohmic(self) -> float:
        """The pack's instantaneous resistance at P+/P- (cells' R0 and the protection board)."""
        return self.series * R0_OHM / self.parallel + self.r_series_ohm

    @property
    def v1(self) -> float:
        """The RC branch's voltage, a cell (V): i1 x R1(soc). The state is the filtered current, not the
        voltage, so a steady current gives exactly i x Rss(soc) even where R1 climbs in the knee."""
        return self.i1 * cell_r1(self._soc)

    def _i1_after(self, i_cell: float, dt_s: float) -> float:
        a = math.exp(-dt_s / TAU_S) if dt_s > 0.0 else 1.0
        return self.i1 * a + i_cell * (1.0 - a)

    def _soc_after(self, i_cell: float, dt_s: float) -> float:
        return min(max(self._soc - i_cell * dt_s / 3600.0 / self.capacity_ah, 0.0), 1.0)

    def voltage_at(self, current_a: float, dt_s: float = 0.0) -> float:
        """The terminal voltage this current would give after dt_s, without advancing the state."""
        i_cell = current_a / self.parallel
        soc1 = self._soc_after(i_cell, dt_s)
        v1 = self._i1_after(i_cell, dt_s) * cell_r1(soc1)
        return self.series * (cell_ocv(soc1) - i_cell * R0_OHM - v1) - current_a * self.r_series_ohm

    def step(self, current_a: float, dt_s: float) -> float:
        """Draw current_a (A, pack, + discharging) for dt_s seconds; return the terminal voltage (V)."""
        if dt_s < 0.0:
            raise ValueError("dt_s < 0")
        i_cell = current_a / self.parallel
        soc0 = self._soc
        soc1 = self._soc_after(i_cell, dt_s)
        mid = 0.5 * (soc0 + soc1)
        v1_0 = self.v1
        i1 = self._i1_after(i_cell, dt_s)
        v1_1 = i1 * cell_r1(soc1)
        # energy, on the step's midpoint: chem = delivered + losses, exactly
        drawn_ah = (soc0 - soc1) * self.capacity_ah * self.parallel   # what the counter took (clamped at 0 and 1)
        h = dt_s / 3600.0
        i_eff = drawn_ah / h if h > 0.0 else 0.0
        drop = self.series * (i_eff / self.parallel * R0_OHM + 0.5 * (v1_0 + v1_1)) + i_eff * self.r_series_ohm
        self.energy_wh_chem += self.series * cell_ocv(mid) * drawn_ah
        self.energy_wh_used += (self.series * cell_ocv(mid) - drop) * drawn_ah
        self.loss_wh += drop * drawn_ah
        self.charge_ah_used += drawn_ah
        self._soc, self.i1 = soc1, i1
        self.time_s += dt_s
        self.current_a = current_a
        self.v_terminal = self.series * (cell_ocv(soc1) - i_cell * R0_OHM - v1_1) - current_a * self.r_series_ohm
        if self.v_terminal <= self.v_cut:
            self._below_s += dt_s
            if self._below_s >= self.cut_off_hold_s:
                self.cut_off = True
        else:
            self._below_s = 0.0
        return self.v_terminal


# ---- the load: motor currents and the 5 V rails, to pack current ----------------------------------

def quiescent_a(v_bus: float) -> float:
    """The bus's standing draw: two Motorons' VIN, the switch's control, the 5 V regulator's Iq (A).
    The same three terms as electrical.operating_point."""
    return 2 * MOTORON_I_VIN_Q + SWITCH_I_ON_PER_V * v_bus + 2 * REG5_IQ


def pack_load_a(motor_currents_a: Sequence[float], electronics_a: float, *, duties: Optional[Sequence[float]] = None,
                servo_a: float = 0.0, v_bus: float = PACK_V_NOM) -> float:
    """The pack current (A) for the motors' and the electronics' currents.

    motor_currents_a: each motor's current at its terminals (A). The Motoron is an H-bridge: a
        near-direct switch, so its input draws the motor current times the PWM duty (on average,
        the bridge's own I^2 R included, switching losses not). Sign: positive when the motor
        takes power; a motor driven backwards by its load returns it (negative).
    duties: each motor's duty, |V applied| / V bus, 0-1. None: 1.0, full duty, which is what
        electrical's budget assumes and what the drive controller's motor model does (its motor is
        on the full pack voltage); pass the real duty for a speed below no-load.
    electronics_a: the 5 V rail's load at the D42V55F5's output (the Pi, display, Pico, LEDs,
        audio, encoders: electrical's p5 / 5 V), through its efficiency REG5_ETA (0.85, the low end
        of Pololu's 80-95 %).
    servo_a: the servo rail's load at the D36V28F5's output, through REG6_ETA (0.85).
    v_bus: the regulators' input voltage. The battery's last terminal voltage is close enough: the
        wiring, fuse and switch drop ~40 mohm x I (0.15 V at the driving load); pass that less
        I x 0.0415 for the exact figure.

    The regulators are constant-power loads, so the current rises as the pack sags. Plus
    quiescent_a(v_bus).
    """
    if duties is None:
        duties = [1.0] * len(motor_currents_a)
    if len(duties) != len(motor_currents_a):
        raise ValueError("one duty per motor")
    v = max(v_bus, 0.5)
    i_m = sum(i * min(max(abs(d), 0.0), 1.0) for i, d in zip(motor_currents_a, duties))
    i_reg = (REG5_VOUT * electronics_a / REG5_ETA + REG6_VOUT * servo_a / REG6_ETA) / v
    return i_m + i_reg + quiescent_a(v)


# ---- validation against electrical.py's power budget ----------------------------------------------

MIX_CYCLE_S = 60.0      # the classroom mix as a one-minute cycle: 36 s idle, 15 s arms, 9 s driving
# The model's own runtimes to the 6.4 V cut-off at P+/P-, typical cell, 23 C, from validate_runtimes()
# (2026-11-04, electrical's loads of that day). The test re-runs and holds them to +-1 %.
MODEL_RUNTIME_H = {"idle": 4.60, "classroom mix": 2.68, "driving": 1.40}


def _electrical():
    """electrical.py, loaded once (it imports OpenCascade: a design-time dependency, never the controller's)."""
    name = "spark_rover_electrical"
    if name not in sys.modules:
        if str(HERE) not in sys.path:
            sys.path.insert(0, str(HERE))
        spec = importlib.util.spec_from_file_location(name, HERE / "electrical.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    return sys.modules[name]


def design_profiles() -> Dict[str, dict]:
    """Each of electrical's runtime cases as the loads pack_load_a takes, from its operating point at the
    pack's nominal voltage; with electrical's own pack current and runtime for the comparison."""
    el = _electrical()
    pb = el.power_budget()
    out = {}
    for case in ("idle", "arms", "driving"):
        op = el.operating_point(case, el.PACK["v_nom"])
        n = el.MOTOR["count"]
        out[case] = {"motor_currents_a": [op["i_motors_bus"] / n] * n, "electronics_a": op["p5"] / el.REG5["vout"],
                     "servo_a": op["p6"] / el.REG6["vout"], "design_i_bat": op["i_bat"], "design_v_bus": op["v_bus"],
                     "design_p_pack_w": op["p_pack_w"]}
    hours = {k: v["hours"] for k, v in pb["runtime"].items()}
    r_ext = pb["r_shared"]["total"] - el.PACK["r"] - el.BMS["r"]     # connectors, fuse, switch, splices, wires
    return {"cases": out, "design_hours": hours, "mix": dict(el.MIX), "r_ext_ohm": r_ext,
            "design_energy_wh": pb["energy_wh"]}


def run_to_cut_off(load: Callable[[float], dict], *, r_ext_ohm: float, dt_s: float = 1.0, cut_on: str = "pack",
                   max_h: float = 24.0, battery: Optional[Battery] = None) -> dict:
    """Run a load (time s -> pack_load_a's arguments) from full to the cut-off. The regulators see the bus,
    the terminal less r_ext_ohm x I; the pack current is solved against that each step. cut_on "pack":
    the firmware reads P+/P- (Battery.cut_off); "bus": it reads the bus (the Motoron's VIN reading)."""
    b = battery or Battery()
    i = 0.0
    while b.time_s < max_h * 3600.0:
        kw = load(b.time_s)
        for _ in range(50):
            v_bus = b.voltage_at(i, dt_s) - i * r_ext_ohm
            i_new = pack_load_a(v_bus=v_bus, **kw)
            if abs(i_new - i) < 1e-7:
                break
            i = i_new
        v = b.step(i, dt_s)
        if (cut_on == "pack" and b.cut_off) or (cut_on == "bus" and v - i * r_ext_ohm <= b.v_cut):
            break
    h = b.time_s / 3600.0
    return {"hours": h, "soc_end": b.soc, "charge_ah": b.charge_ah_used, "energy_wh_terminal": b.energy_wh_used,
            "energy_wh_chem": b.energy_wh_chem, "loss_wh": b.loss_wh, "p_chem_w": b.energy_wh_chem / h,
            "usable_frac": b.charge_ah_used / (b.capacity_ah * b.parallel)}


def validate_runtimes(dt_s: float = 1.0, cut_on: str = "pack") -> Dict[str, dict]:
    """electrical's three runtime profiles through the model to the cut-off, against electrical's hours."""
    d = design_profiles()
    cases, mix = d["cases"], d["mix"]

    def const(case):
        kw = {k: cases[case][k] for k in ("motor_currents_a", "electronics_a", "servo_a")}
        return lambda t: kw

    order, edges, acc = ["idle", "arms", "driving"], [], 0.0
    for c in order:
        acc += mix[c] * MIX_CYCLE_S
        edges.append(acc)

    def mixed(t):
        u = t % MIX_CYCLE_S
        for c, e in zip(order, edges):
            if u < e:
                return const(c)(t)
        return const(order[-1])(t)

    out = {}
    for name, fn in (("idle", const("idle")), ("classroom mix", mixed), ("driving", const("driving"))):
        r = run_to_cut_off(fn, r_ext_ohm=d["r_ext_ohm"], dt_s=dt_s, cut_on=cut_on)
        r["design_hours"] = d["design_hours"][name]
        r["design_p_pack_w"] = (sum(mix[c] * cases[c]["design_p_pack_w"] for c in mix) if name == "classroom mix"
                                else cases[name]["design_p_pack_w"])
        r["design_energy_wh"] = d["design_energy_wh"]
        r["ratio"] = r["hours"] / r["design_hours"]
        out[name] = r
    return out


def main() -> None:
    print(f"pack OCV full {2 * cell_ocv(1.0):.3f} V, nominal-soc (50 %) {2 * cell_ocv(0.5):.3f} V; "
          f"R at P+/P- {Battery().r_ohmic * 1000:.1f} mohm instantaneous")
    for cut_on in ("pack", "bus"):
        res = validate_runtimes(cut_on=cut_on)
        print(f"\ncut-off read at the {cut_on}:")
        print(f"{'profile':15s} {'model h':>8s} {'design h':>9s} {'ratio':>6s} {'chem Wh':>8s} {'design Wh':>9s} "
              f"{'P chem W':>8s} {'P design W':>10s} {'charge used':>11s}")
        for k, r in res.items():
            print(f"{k:15s} {r['hours']:8.2f} {r['design_hours']:9.2f} {r['ratio']:6.3f} {r['energy_wh_chem']:8.2f} "
                  f"{r['design_energy_wh']:9.2f} {r['p_chem_w']:8.2f} {r['design_p_pack_w']:10.2f} "
                  f"{r['usable_frac']:10.1%}")


if __name__ == "__main__":
    main()
