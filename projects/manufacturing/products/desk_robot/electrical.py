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

"""The desk robot's electrical design: parts with pins, the netlist, the wires, the power budget.

Four AA cells feed, through a PTC fuse, a wire-to-wire connector and a MOSFET slide switch
with reverse-voltage protection: the DRV8833 carrier directly (2.7-10.8 V) and a 5 V
buck-boost regulator, whose output reaches the XIAO ESP32-S3's 5V pin through a Schottky
diode. Why each part, from the vendors' own pages:

- The XIAO's 5V pin IS its USB VBUS net (Seeed's XIAO ESP32S3 schematic: header pin 14 is
  VBUS, which feeds the LiPo charger and, through the board's own Schottky D2, the SGM6029
  3.3 V buck). The SGM6029 runs from 1.95 to 5.5 V. Four fresh alkaline cells are 6.4 V:
  even through two Schottkys that is over 5.5 V, so the pack cannot go on the 5V pin. Four
  NiMH cells end at 4.0 V, where a plain 5 V buck would drop out: hence a buck-BOOST
  (Pololu S7V7F5, 2.7-11.8 V in), which keeps the XIAO at 5 V over both chemistries' life.
- Seeed: "you must have some sort of diode ... between your external power source and this
  pin". With USB plugged in on the bench, VBUS would otherwise push into the regulator's
  output (and through it toward the cells). D1 (1N5817) blocks that; the higher of USB and
  the regulator feeds the board.
- The XIAO's BAT pads are a LiPo charger's (Seeed wiki): never the AA pack.
- The boards lie flat in printed pockets (design.TRAY): nothing may stick out below them,
  so every wire is soldered from the top into a through-hole or onto a castellated pad and
  every connector is wire-to-wire (JST SM), never a header on a board.

Run:

    python projects/manufacturing/products/desk_robot/electrical.py
        # writes release/electrical/{schematic.svg, wiring.svg, WIRES.md, POWER.md}

Units: mm, V, A, ohm, W unless a name says otherwise. The robot frame is design.py's:
x forward, y left, z up from the desk.
"""
from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

import design
from design import BOUGHT, TRAY, Robot

HERE = Path(__file__).resolve().parent
OUT = HERE / "release" / "electrical"
POLOLU = "https://www.pololu.com"

# ---- the parts, with the numbers the design uses and where they come from ---------------
# Every figure below was read from the vendor's page or datasheet at the URL beside it
# (2026-11-03). "ASSUMED" marks a number no vendor gives; it is used, and said so.

MOTOR = {   # Pololu #3065, 100:1 Micro Metal Gearmotor HPCB 6V: {POLOLU}/product/3065/specs
    "v_rated": 6.0, "i_stall": 1.5, "i_free": 0.15, "i_free_tol": 0.5,   # free current "Typical, +-50%"
    "rpm_free": 330.0, "stall_kgcm": 1.6, "ratio": 100.37, "max_power_w": 1.3,
    "source": f"{POLOLU}/product/3065/specs",
    "note": "stall figures are theoretical; Pololu: stalls could damage the motor or gearbox",
}
MOTOR["r_term"] = MOTOR["v_rated"] / MOTOR["i_stall"]                                  # 4.0 ohm
MOTOR["w_free"] = MOTOR["rpm_free"] * 2 * math.pi / 60                                  # rad/s, output
MOTOR["ke"] = (MOTOR["v_rated"] - MOTOR["i_free"] * MOTOR["r_term"]) / MOTOR["w_free"]  # V s/rad at the output
MOTOR["kt"] = MOTOR["stall_kgcm"] * 0.0980665 / (MOTOR["i_stall"] - MOTOR["i_free"])     # N m/A at the output

DRV = {     # Pololu #2130 carrier: {POLOLU}/product/2130 and /specs; TI DRV8833 SLVSAR1E (2015)
    "vm_min": 2.7, "vm_max": 10.8, "i_cont": 1.2, "i_peak": 2.0,              # per channel, Pololu
    "i_ocp_min": 2.0, "i_ocp_max": 3.3, "t_tsd_min_c": 150.0,                # TI 6.5
    "rds_hs": 0.200, "rds_ls": 0.160, "rds_hs_85c": 0.325, "rds_ls_85c": 0.275,   # TI 6.5, VM = 5 V, 500 mA
    "i_vm": 0.0017, "i_vm_max": 0.003, "i_sleep": 1.6e-6,                     # TI 6.5
    "vil": 0.7, "vih": 2.0, "vil_sleep": 0.5, "vih_sleep": 2.5, "vin_max": 5.75,   # TI 6.3 / 6.5
    "rpd_in": 150e3, "rpd_sleep": 500e3,                                      # TI 6.5
    "slp_pullup": 47e3,   # the carrier's nSLEEP pull-up, to VMM: Pololu staff, forum.pololu.com/t/drv8833-sleep-mode/6196
    "sources": [f"{POLOLU}/product/2130", f"{POLOLU}/product/2130/specs", "https://www.ti.com/lit/ds/symlink/drv8833.pdf"],
    "note": "reverse-voltage FET on VIN; VMM is VIN after it; AISEN/BISEN tied to GND on the board "
            "(no current limit); xIN pulled low on chip, nSLEEP pulled high on the board",
}
DRV["r_bridge"] = DRV["rds_hs"] + DRV["rds_ls"]                 # one high side + one low side in the path
DRV["r_bridge_85c"] = DRV["rds_hs_85c"] + DRV["rds_ls_85c"]

ESP = {     # Espressif ESP32-S3 Series Datasheet v2.2, https://documentation.espressif.com/esp32-s3_datasheet_en.pdf
    "vdd": 3.3, "voh_frac": 0.8, "vol_frac": 0.1, "vih_frac": 0.75, "vil_frac": 0.25,   # Table 5-4
    "pin_max_over_vdd": 0.3,                    # VIH max = VDD + 0.3 (Table 5-4)
    "r_pull": 45e3,                             # internal weak pull-up / -down (Table 5-4)
    "adc_atten3_mv": 2900.0, "adc_err_mv": 50.0, "adc_bits": 12,   # Table 5-6, ATTEN3 after calibration
    "i_tx_peak": 0.340,                         # 802.11b 1 Mbps @21 dBm, peak (Table 5-7)
    "ledc_channels": 8, "apb_hz": 80e6,         # "LED PWM controller, up to 8 channels"; routed via the GPIO matrix
    "strapping": {0, 3, 45, 46},                # GPIO3 is ignored with the default eFuses (Table 3-5)
    "powerup_glitch_low": set(range(1, 15)),    # Table 2-2: GPIO1-14 glitch LOW for ~60 us at power-up
}
XIAO = {    # Seeed XIAO ESP32-S3: https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
    # pad -> (GPIO, ADC channel or None, other roles). All of D0-D10 are listed PWM-capable ("11x GPIOs (PWM)").
    "pads": {"D0": (1, "ADC1_CH0", ""), "D1": (2, "ADC1_CH1", ""), "D2": (3, "ADC1_CH2", "strapping (JTAG source)"),
             "D3": (4, "ADC1_CH3", ""), "D4": (5, "ADC1_CH4", "I2C SDA"), "D5": (6, "ADC1_CH5", "I2C SCL"),
             "D6": (43, None, "UART0 TX (boot log)"), "D7": (44, None, "UART0 RX"),
             "D8": (7, "ADC1_CH6", "SPI SCK"), "D9": (8, "ADC1_CH7", "SPI MISO"), "D10": (9, "ADC1_CH8", "SPI MOSI")},
    "p_wifi_w": 3.8 * 0.100, "p_modem_sleep_w": 3.8 * 0.027,   # Seeed table: 100 mA Wi-Fi active, 27 mA modem sleep, at 3.8 V
    "vbus_max": 5.5,           # SGM6029 input range 1.95-5.5 V: https://sg-micro.com/product/SGM6029
    "vf_onboard": 0.35,        # its own Schottky D2 (LMBR4010BST5G) from VBUS: ASSUMED typical drop at 100 mA
    "i_3v3_max": 0.700,        # "You can draw 700mA"
    "schematic": "https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/new-res/202003753_XIAO%20ESP32S3%20Sense_V1.51_SCH_260611.pdf",
}
REG = {     # Pololu #2119, 5V Step-Up/Step-Down Voltage Regulator S7V7F5: {POLOLU}/product/2119/specs
    "vin_min": 2.7, "vin_max": 11.8, "vout": 5.0, "vout_tol": (-0.03, 0.05), "i_out_max": 1.0, "i_out_boost": 0.5,
    "iq": 0.1e-3, "eta": 0.85,          # efficiency "typically 85% to 95%": the low end is used
    "source": f"{POLOLU}/product/2119/specs", "note": "no reverse-voltage protection (Pololu): SW1 provides it",
}
SWITCH = {  # Pololu #2810, Mini MOSFET Slide Switch with Reverse Voltage Protection, LV: {POLOLU}/product/2810
    "v_min": 1.8, "v_max": 16.0, "i_cont_55c": 3.0, "i_cont_150c": 6.0, "rds_max": 0.026, "rds_4v5": 0.016,
    "i_on_per_v": 210e-6,       # "~210 uA/V" (the larger of the two table figures, the LED's)
    "source": f"{POLOLU}/product/2810", "note": "the slide only switches the MOSFET; load current never goes through it",
}
FUSE = {    # Bourns MF-R300 radial PTC: https://www.bourns.com/docs/product-datasheets/mf-r.pdf
    # (the MF-R250's 2.5 A hold sat under the worst-case double stall, 2.59 A: it was rejected)
    "part": "MF-R300", "v_max": 30.0, "i_max": 40.0, "i_hold": 3.00, "i_trip": 6.00, "i_hold_40c": 2.49,
    "r_min": 0.020, "r_max": 0.050, "r1_max": 0.08, "t_trip_s": 10.8, "t_trip_at": 15.0, "size_mm": (12.0, 18.3),
}
DIODE = {   # Vishay 1N5817 (DO-41): https://www.vishay.com/docs/88525/1n5817.pdf
    "vrrm": 20.0, "if_av": 1.0, "vf_1a": 0.450, "vf_3a": 0.750, "vf_min": 0.20,   # vf_min: ASSUMED floor at light load
}
CELLS = {
    # "end" is the firmware's cut-off, 1.0 V per cell (4.0 V): below it the motors run at
    # two thirds of their speed and a full-duty start browns the bus out (see checks()).
    "alkaline": {"spec": "Energizer E91 AA alkaline", "v_fresh": 1.60, "v_mid": 1.25, "v_end": 1.00,
                 "r_cell": (0.150, 0.300), "g": 23.0, "c_ref_mah": 2850.0, "i_ref": 0.025, "peukert": 1.2,
                 "source": "https://courses.egr.uh.edu/ECE/ECE2100/Archive/Spring%202024%20Honors/E91.pdf (Energizer E91, EBC-1102Q)",
                 "note": "IR 150-300 mohm fresh, 23 g, capacity to 0.8 V/cell; 2850 mAh at 25 mA from Energizer's "
                         "engineering data; the Peukert exponent 1.2 is ASSUMED (the sheet gives capacity only as a chart)"},
    "nimh": {"spec": "Energizer NH15-2300 AA NiMH", "v_fresh": 1.40, "v_mid": 1.20, "v_end": 1.00,
             "r_cell": (0.030, 0.040), "g": 27.0, "c_ref_mah": 2300.0, "i_ref": 0.460, "peukert": 1.0,
             "source": "https://data.energizer.com/pdfs/nh15-2300gl1220.pdf",
             "note": "IR 30 mohm charged, 40 mohm half discharged (+-20 %), 2300 mAh at 460 mA (0.2C), 27 g"},
}
HOLDER = {"lead_awg": 26, "lead_mm": 150.0, "r_contacts": 4 * 0.010,   # ASSUMED 10 mohm per spring contact
          "source": "https://www.keyelco.com/userAssets/file/M65p28.pdf (wire leads: #26 AWG, 6.0 in)"}
# Wire: Powerstream's table, ohm/km and "chassis wiring" amps (a rule of thumb, it says).
AWG = {22: (52.94, 7.0), 24: (84.20, 3.5), 26: (133.86, 2.2), 28: (212.87, 1.4)}
AWG_SOURCE = "https://www.powerstream.com/Wire_Size.htm"
JST_SM = {  # JST SM series, 2.5 mm, wire-to-wire: https://www.jst-mfg.com/product/pdf/eng/eSM.pdf
    "i_rating": 3.0, "v_rating": 250.0, "awg": (28, 22), "r_contact": 0.010,
    "plug": "SMP-02V-BC + 2x SHF-001T-0.8BS socket crimp", "receptacle": "SMR-02V-B + 2x SYM-001T-P0.6 pin crimp",
}
PWM_HZ = 20e3            # above hearing; the DRV8833's 1.1 us propagation and 450 ns dead time are 3 % of a period
NOMINAL_VSENSE_MAX = 7.2  # V: four fresh lithium AA (1.8 V/cell) as the divider's design ceiling


# ---- 1. components, with every pin -------------------------------------------------------

def components() -> Dict[str, dict]:
    """ref -> {part, pins, source, ...}. Every pin is listed; unused ones are no-connects."""
    pads = list(XIAO["pads"])
    return {
        "BT1": {"part": "4 x AA holder with wire leads (BOUGHT['battery_holder'])", "pins": ["+", "-"],
                "source": BOUGHT["battery_holder"]["source"]},
        "F1": {"part": "Bourns MF-R300 PTC resettable fuse, 3.0 A hold, 6.0 A trip, 30 V", "pins": ["1", "2"],
               "source": "https://www.bourns.com/docs/product-datasheets/mf-r.pdf"},
        "J1": {"part": "JST SM 2-pin wire-to-wire pair (battery): " + JST_SM["plug"] + " / " + JST_SM["receptacle"],
               "pins": ["1", "2"], "source": "https://www.jst-mfg.com/product/pdf/eng/eSM.pdf"},
        "SW1": {"part": "Pololu #2810 Mini MOSFET Slide Switch with Reverse Voltage Protection, LV",
                "pins": ["VIN", "GND", "VOUT", "ON"], "source": SWITCH["source"]},
        "U1": {"part": "Pololu #2119 5V Step-Up/Step-Down Voltage Regulator S7V7F5", "pins": ["VIN", "GND", "VOUT"],
               "source": REG["source"]},
        "D1": {"part": "1N5817 Schottky, 20 V 1 A, DO-41", "pins": ["A", "K"], "source": "https://www.vishay.com/docs/88525/1n5817.pdf"},
        "U2": {"part": "Pololu #2130 DRV8833 dual motor driver carrier", "source": DRV["sources"][0],
               "pins": ["VIN", "VMM", "GND", "AOUT1", "AOUT2", "BOUT1", "BOUT2", "AIN1", "AIN2", "BIN1", "BIN2",
                        "SLP", "FLT", "AISEN", "BISEN"]},
        "U3": {"part": "Seeed Studio XIAO ESP32-S3", "pins": pads + ["5V", "GND", "3V3", "BAT+", "BAT-"],
               "source": "https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/"},
        "C1": {"part": "100 uF 16 V low-ESR aluminium electrolytic, radial 6.3 mm (bulk, at the driver)", "pins": ["+", "-"],
               "source": f"{POLOLU}/docs/0J16 (LC spikes); TI SLVSAR1E pin table (VM bypass)"},
        "C2": {"part": "100 nF X7R ceramic, 5.08 mm leads (ADC filter)", "pins": ["1", "2"], "source": "-"},
        "C3": {"part": "100 nF ceramic across motor M1's terminals", "pins": ["1", "2"], "source": f"{POLOLU}/docs/0J15/9"},
        "C4": {"part": "100 nF ceramic across motor M2's terminals", "pins": ["1", "2"], "source": f"{POLOLU}/docs/0J15/9"},
        "R1": {"part": "100 kohm 1 % 0.25 W axial (battery sense, top)", "pins": ["1", "2"], "source": "-"},
        "R2": {"part": "47 kohm 1 % 0.25 W axial (battery sense, bottom)", "pins": ["1", "2"], "source": "-"},
        "R3": {"part": "3.3 kohm 5 % 0.25 W axial (nSLEEP pull-down: the driver sleeps until the firmware wakes it)",
               "pins": ["1", "2"], "source": "-"},
        "J2": {"part": "JST SM 2-pin wire-to-wire pair (left motor)", "pins": ["1", "2"], "source": "https://www.jst-mfg.com/product/pdf/eng/eSM.pdf"},
        "J3": {"part": "JST SM 2-pin wire-to-wire pair (right motor)", "pins": ["1", "2"], "source": "https://www.jst-mfg.com/product/pdf/eng/eSM.pdf"},
        "M1": {"part": "Pololu #3065 gearmotor, LEFT (+y)", "pins": ["1", "2"], "source": MOTOR["source"]},
        "M2": {"part": "Pololu #3065 gearmotor, RIGHT (-y), mounted mirrored", "pins": ["1", "2"], "source": MOTOR["source"]},
    }


# ---- 3. the pin assignment on the XIAO ----------------------------------------------------
# Kept free: D4/D5 (I2C, for a later sensor), D6/D7 (UART0: the ROM's boot log is on D6).
# No driver input is on a strapping pin. GPIO1-9 glitch LOW at power-up (Table 2-2): a low
# xIN is coast, and the driver is asleep then anyway (R3).
PINS = {
    "AIN1": "D0", "AIN2": "D1",          # left motor, LEDC channels 0 and 1
    "BIN1": "D8", "BIN2": "D9",          # right motor, LEDC channels 2 and 3
    "nSLEEP": "D3",                      # push-pull output, HIGH = awake
    "nFAULT": "D2",                      # input with the internal pull-up; GPIO3's strap is ignored by default eFuses
    "VSENSE": "D10",                     # ADC1_CH8: ADC1 works with Wi-Fi on (ADC2 does not)
}
DIVIDER = {"r_top": 100e3, "r_bot": 47e3, "c": 100e-9}
R3_PULLDOWN = 3.3e3


def adc_sense() -> dict:
    """The battery-voltage divider on D10 and what the ADC sees."""
    k = DIVIDER["r_bot"] / (DIVIDER["r_top"] + DIVIDER["r_bot"])
    rth = DIVIDER["r_top"] * DIVIDER["r_bot"] / (DIVIDER["r_top"] + DIVIDER["r_bot"])
    full = ESP["adc_atten3_mv"] / 1000 / k
    tau = rth * DIVIDER["c"]
    return {"ratio": k, "full_scale_pack_v": full, "adc_at_7v2": NOMINAL_VSENSE_MAX * k,
            "adc_at_6v4": 6.4 * k, "adc_at_3v2": 3.2 * k,
            "lsb_pack_mv": ESP["adc_atten3_mv"] / 2 ** ESP["adc_bits"] / k,
            "err_pack_mv": ESP["adc_err_mv"] / k, "r_thevenin": rth, "tau_ms": tau * 1e3,
            "pwm_ripple_attenuation": 1 / math.hypot(1, 2 * math.pi * PWM_HZ * tau),
            "i_divider_ua_at_6v4": 6.4 / (DIVIDER["r_top"] + DIVIDER["r_bot"]) * 1e6}


def sleep_pin(vmm: float) -> dict:
    """nSLEEP: the carrier's 47k pull-up to VMM against R3 to GND, the GPIO driving it."""
    pu, pd = DRV["slp_pullup"], R3_PULLDOWN
    released = vmm * pd / (pu + pd)                       # GPIO high-Z (reset, boot, unpowered)
    driven_hi = ESP["voh_frac"] * ESP["vdd"]              # the GPIO's worst VOH
    i_gpio = ESP["vdd"] / pd - (vmm - ESP["vdd"]) / pu    # what the GPIO sources when high
    return {"released_v": released, "driven_high_v_min": driven_hi, "gpio_source_ma": i_gpio * 1e3,
            "i_awake_ma": ESP["vdd"] / pd * 1e3, "i_asleep_ua": vmm / (pu + pd) * 1e6}


def ledc_bits() -> int:
    return int(math.floor(math.log2(ESP["apb_hz"] / PWM_HZ)))


# ---- 2. the netlist ------------------------------------------------------------------------

def netlist() -> Dict[str, List[Tuple[str, str]]]:
    """net -> [(ref, pin)]. M2 is mounted mirrored, so its leads are crossed (BOUT1 -> M2.2):
    xIN1 = PWM, xIN2 = 0 then drives both wheels forward. Check on the bench; re-pin J3 if not."""
    x = PINS
    return {
        "VBAT_RAW": [("BT1", "+"), ("F1", "1")],
        "VBAT": [("F1", "2"), ("J1", "1"), ("SW1", "VIN")],
        "VSW": [("SW1", "VOUT"), ("U2", "VIN"), ("C1", "+"), ("U1", "VIN")],
        "VMM": [("U2", "VMM"), ("R1", "1")],
        "VSENSE": [("R1", "2"), ("R2", "1"), ("C2", "1"), ("U3", x["VSENSE"])],
        "5V_REG": [("U1", "VOUT"), ("D1", "A")],
        "VBUS": [("D1", "K"), ("U3", "5V")],
        "GND": [("BT1", "-"), ("J1", "2"), ("SW1", "GND"), ("U2", "GND"), ("C1", "-"), ("R3", "2"),
                ("U3", "GND"), ("R2", "2"), ("C2", "2"), ("U1", "GND")],
        "AIN1": [("U3", x["AIN1"]), ("U2", "AIN1")],
        "AIN2": [("U3", x["AIN2"]), ("U2", "AIN2")],
        "BIN1": [("U3", x["BIN1"]), ("U2", "BIN1")],
        "BIN2": [("U3", x["BIN2"]), ("U2", "BIN2")],
        "nSLEEP": [("U3", x["nSLEEP"]), ("U2", "SLP"), ("R3", "1")],
        "nFAULT": [("U3", x["nFAULT"]), ("U2", "FLT")],
        "MA1": [("U2", "AOUT1"), ("J2", "1"), ("M1", "1"), ("C3", "1")],
        "MA2": [("U2", "AOUT2"), ("J2", "2"), ("M1", "2"), ("C3", "2")],
        "MB1": [("U2", "BOUT1"), ("J3", "1"), ("M2", "2"), ("C4", "2")],
        "MB2": [("U2", "BOUT2"), ("J3", "2"), ("M2", "1"), ("C4", "1")],
    }


NO_CONNECT = {
    ("U2", "AISEN"): "tied to GND on the carrier (current limiting off)",
    ("U2", "BISEN"): "tied to GND on the carrier (current limiting off)",
    ("U3", "3V3"): "the XIAO's 3.3 V output: unused (the driver logic needs no supply)",
    ("U3", "D4"): "free: I2C SDA for a later sensor", ("U3", "D5"): "free: I2C SCL",
    ("U3", "D6"): "free: UART0 TX, carries the ROM's boot log", ("U3", "D7"): "free: UART0 RX",
    ("U3", "BAT+"): "LiPo charger pad: NEVER the AA pack", ("U3", "BAT-"): "LiPo charger pad",
    ("SW1", "ON"): "external control input: unused, the slide switch controls the MOSFET",
}


# ---- positions (robot frame, mm), for the wiring diagram and the length estimates -----------

# The parts this design adds that must be MOUNTED: footprint (x, y, height) in mm, mass in g,
# the proposed centre on the bottom plate's top (robot frame) and how it is held. No hole in
# the plates exists for any of them: the lead makes the holes. Masses marked ASSUMED are not
# on the vendor's page.
MOUNTS = {
    "SW1": {"footprint_mm": (15.2, 15.2, 2.5), "g": 0.6, "at": (-20.0, 27.0), "slider": "toward +y (the left side, by the battery leads), "
            "reachable through the 25 mm gap between the plates", "fix": "foam tape, or two M2 if the vendor drawing shows holes",
            "source": f"{POLOLU}/product/2810 (0.6 x 0.6 x 0.1 in, 0.6 g; height without the slide actuator)"},
    "U1": {"footprint_mm": (12.0, 9.0, 3.0), "g": 0.4, "at": (-36.0, 22.0), "fix": "foam tape (no mounting holes)",
           "source": f"{POLOLU}/product/2119/specs (0.35 x 0.475 x 0.1 in, 0.4 g)"},
    "F1": {"footprint_mm": (12.0, 3.0, 18.3), "g": 1.0, "at": "hangs on the battery lead under the top plate, sleeved",
           "fix": "heat-shrink over the disc and both joints; mass ASSUMED",
           "source": "Bourns MF-R300: 12.0 wide, 18.3 tall max, leads 5.1 apart"},
    "C1": {"footprint_mm": (11.0, 6.3, 6.3), "g": 0.6, "at": "lying flat on the tray beside the DRV8833, legs in VIN/GND",
           "fix": "a dab of hot glue to the tray; mass ASSUMED", "source": "radial 6.3 x 11 mm, 16 V"},
    "J1": {"footprint_mm": (11.0, 6.0, 4.0), "g": 0.6, "at": "hangs between the plates below the battery hole",
           "fix": "free (it is the top plate's disconnect); mass ASSUMED", "source": "JST eSM.pdf, 2-circuit housings"},
    "J2": {"footprint_mm": (11.0, 6.0, 4.0), "g": 0.6, "at": "free in the motor lead", "fix": "free; mass ASSUMED",
           "source": "JST eSM.pdf"},
    "J3": {"footprint_mm": (11.0, 6.0, 4.0), "g": 0.6, "at": "free in the motor lead", "fix": "free; mass ASSUMED",
           "source": "JST eSM.pdf"},
}


def _placed(r: Robot) -> Dict[str, Tuple["object", "object"]]:
    """label -> (R, t) of the bought parts as bought_parts.py places them (local -> robot)."""
    import bought_parts
    return {p.label: (p.R, p.t) for p in bought_parts.placements(r)}


def _at(Rt, local) -> Tuple[float, float, float]:
    R, t = Rt
    v = R @ list(local) + t
    return (float(v[0]), float(v[1]), float(v[2]))


# Measured on the vendors' STEP models, in bought_parts' local frames:
#  XIAO: the top-face pads of the two castellated rows at local y = +-7.87, z = 1.24 (the top
#        copper), 2.54 mm pitch, the first pad 7.555 mm from the board centre toward the USB end.
#  DRV8833 carrier: 16 plated holes, two rows of 8 at local y = +-5.08 (0.4 in), 2.54 mm pitch,
#        local x = -9.14 + 2.54 k; its top copper at z = 1.57.
XIAO_PAD_Y = 7.87
XIAO_PAD_X0 = -7.555
XIAO_TOP = 1.24
DRV_ROW_Y = 5.08
DRV_TOP = 1.57
# The carrier's pin order, from its silkscreen (Pololu's photo 0J3865 of the label side) and
# the STEP (the bulk capacitor and the reverse-protection FET at the GND / VIN end, +x here;
# the label side is the underside, so its left column is the -y row seen from the top):
#   -y row, k = 7 (the GND end) .. 0: GND, VIN, BOUT1, BOUT2, AOUT2, AOUT1, AISEN, BISEN
#   +y row, k = 7 .. 0:               GND, VMM, BIN1,  BIN2,  AIN2,  AIN1,  SLP,   FLT
DRV_PINS = {"GND": (7, -1), "VIN": (6, -1), "BOUT1": (5, -1), "BOUT2": (4, -1), "AOUT2": (3, -1), "AOUT1": (2, -1),
            "AISEN": (1, -1), "BISEN": (0, -1), "GND2": (7, 1), "VMM": (6, 1), "BIN1": (5, 1), "BIN2": (4, 1),
            "AIN2": (3, 1), "AIN1": (2, 1), "SLP": (1, 1), "FLT": (0, 1)}
MOTOR_TAB = (2.7, -25.7, 0.0)     # gearmotor local frame: tabs at x +-2.2..3.2, y -26.2..-25.2 (bought_parts)


def positions(r: Robot) -> Dict[Tuple[str, str], Tuple[float, float, float]]:
    """Where each pin is, robot frame. The XIAO, the DRV8833 carrier and the motors come from
    bought_parts.placements (the boards lie in the tray's pockets, the XIAO's USB-C toward -x);
    the battery leads come down through the top plate's 8 mm hole at (0, 34). SW1 and U1 are
    where MOUNTS proposes them. The XIAO's and the carrier's pins are at their measured holes
    and pads (DRV_PINS, XIAO_PAD_*)."""
    pl = _placed(r)
    zb = r.plate_top_z                                  # bottom plate top
    zt = r.plate_top_z + r.standoff_len                 # top plate underside
    p: Dict[Tuple[str, str], Tuple[float, float, float]] = {}
    # XIAO: Seeed's top view (USB up) has D0-D6 down its left column, which with USB at -x
    # lies at -y; 5V, GND, 3V3, D10, D9, D8, D7 down the right column (+y).
    left = ["D0", "D1", "D2", "D3", "D4", "D5", "D6"]
    right = ["5V", "GND", "3V3", "D10", "D9", "D8", "D7"]
    for k, (a, b) in enumerate(zip(left, right)):
        x = XIAO_PAD_X0 + 2.54 * k
        p[("U3", a)] = _at(pl["xiao"], (x, -XIAO_PAD_Y, XIAO_TOP))
        p[("U3", b)] = _at(pl["xiao"], (x, XIAO_PAD_Y, XIAO_TOP))
    p[("U3", "BAT+")] = p[("U3", "BAT-")] = _at(pl["xiao"], (4.0, 0.0, 0.0))   # pads on the back
    for pin in components()["U2"]["pins"]:
        k, side = DRV_PINS[pin]
        p[("U2", pin)] = _at(pl["drv8833"], (-9.14 + 2.54 * k, side * DRV_ROW_Y, DRV_TOP))
    for ref, side in (("M1", "left"), ("M2", "right")):
        tx, ty, tz = MOTOR_TAB
        p[(ref, "1")] = _at(pl[f"gearmotor_{side}"], (tx, ty, tz))
        p[(ref, "2")] = _at(pl[f"gearmotor_{side}"], (-tx, ty, tz))
    drv_x = pl["drv8833"][1][0]
    zpad = zb + TRAY["base"] + DRV_TOP
    p[("C3", "1")], p[("C3", "2")] = p[("M1", "1")], p[("M1", "2")]
    p[("C4", "1")], p[("C4", "2")] = p[("M2", "1")], p[("M2", "2")]
    pitch = 2.5                                          # JST SM: its two contacts 2.5 mm apart
    p[("J2", "1")], p[("J2", "2")] = (drv_x + 6.0 - pitch / 2, 10.0, zb + 6.0), (drv_x + 6.0 + pitch / 2, 10.0, zb + 6.0)
    p[("J3", "1")], p[("J3", "2")] = (drv_x + 6.0 - pitch / 2, -10.0, zb + 6.0), (drv_x + 6.0 + pitch / 2, -10.0, zb + 6.0)
    hz = zt + r.plate_t + 8.0                            # the holder's leads leave it at mid-height
    p[("BT1", "+")], p[("BT1", "-")] = (2.0, 29.0, hz), (-2.0, 29.0, hz)
    p[("F1", "1")], p[("F1", "2")] = (-2.0, 33.0, zt - 4.0), (-2.0, 33.0, zt - 12.0)
    p[("J1", "1")], p[("J1", "2")] = (-8.0 - pitch / 2, 30.0, zt - 16.0), (-8.0 + pitch / 2, 30.0, zt - 16.0)
    (sx, sy), (ux, uy) = MOUNTS["SW1"]["at"], MOUNTS["U1"]["at"]
    zs = zb + 1.0 + 1.57                                 # on 1 mm of foam tape, the boards' top copper
    p[("SW1", "VIN")], p[("SW1", "GND")] = (sx - 7.0, sy - 3.0, zs), (sx - 7.0, sy + 3.0, zs)
    p[("SW1", "VOUT")], p[("SW1", "ON")] = (sx + 7.0, sy - 3.0, zs), (sx + 7.0, sy + 3.0, zs)
    for k, pin in enumerate(("VIN", "GND", "VOUT")):          # one row on the edge toward -x
        p[("U1", pin)] = (ux - 5.0, uy - 3.0 + 3.0 * k, zs)
    p[("D1", "K")] = p[("U3", "5V")]
    p[("D1", "A")] = (p[("U3", "5V")][0], p[("U3", "5V")][1] + 8.0, p[("U3", "5V")][2] + 1.0)
    p[("R1", "2")] = p[("U3", PINS["VSENSE"])]
    p[("R1", "1")] = (p[("U3", PINS["VSENSE"])][0], p[("U3", PINS["VSENSE"])][1] + 9.0, p[("U3", PINS["VSENSE"])][2] + 1.0)
    for ref in ("R2", "C2"):
        p[(ref, "1")], p[(ref, "2")] = p[("U3", PINS["VSENSE"])], p[("U3", "GND")]
    for ref, a, b in (("R3", "SLP", "GND"), ("C1", "VIN", "GND")):
        pins = components()[ref]["pins"]
        p[(ref, pins[0])], p[(ref, pins[1])] = p[("U2", a)], p[("U2", b)]
    return p


# ---- 4. the wires --------------------------------------------------------------------------
# (id, from, to, net, colour, AWG, connector at from, connector at to, what current it carries)
SOLDER_THT = "soldered from the top into the board's through-hole"
SOLDER_PAD = "soldered onto the XIAO's castellated pad, from the top"
_SMP, _SMR = "JST " + JST_SM["plug"].split(" +")[0] + " (socket crimp SHF-001T-0.8BS)", \
             "JST " + JST_SM["receptacle"].split(" +")[0] + " (pin crimp SYM-001T-P0.6)"
WIRES = [
    ("W1", ("BT1", "+"), ("F1", "1"), "VBAT_RAW", "red", 26, "holder's own lead", "soldered to the fuse leg, heat-shrink", "battery"),
    ("W2", ("BT1", "-"), ("J1", "2"), "GND", "black", 26, "holder's own lead", _SMP, "battery"),
    ("W3", ("F1", "2"), ("J1", "1"), "VBAT", "red", 22, "soldered to the fuse leg, heat-shrink", _SMP, "battery"),
    ("W4", ("J1", "1"), ("SW1", "VIN"), "VBAT", "red", 22, _SMR, SOLDER_THT, "battery"),
    ("W5", ("J1", "2"), ("SW1", "GND"), "GND", "black", 22, _SMR, SOLDER_THT, "battery"),
    ("W6", ("SW1", "VOUT"), ("U2", "VIN"), "VSW", "red", 22, SOLDER_THT, SOLDER_THT + " (C1's + leg in the same hole)", "motors"),
    ("W7", ("SW1", "GND"), ("U2", "GND"), "GND", "black", 22, SOLDER_THT, SOLDER_THT, "battery"),
    ("W8", ("SW1", "VOUT"), ("U1", "VIN"), "VSW", "red", 26, SOLDER_THT, SOLDER_THT, "regulator"),
    ("W9", ("U1", "VOUT"), ("D1", "A"), "5V_REG", "orange", 26, SOLDER_THT, "soldered to D1's anode, heat-shrink", "logic"),
    ("W10", ("U2", "GND"), ("U3", "GND"), "GND", "black", 26, SOLDER_THT, SOLDER_PAD, "regulator"),
    ("W11", ("U3", "GND"), ("U1", "GND"), "GND", "black", 26, SOLDER_PAD, SOLDER_THT, "regulator"),
    ("W12", ("U2", "VMM"), ("R1", "1"), "VMM", "red", 28, SOLDER_THT, "soldered to R1's leg, heat-shrink", "sense"),
    ("W13", ("U3", PINS["AIN1"]), ("U2", "AIN1"), "AIN1", "yellow", 28, SOLDER_PAD, SOLDER_THT, "signal"),
    ("W14", ("U3", PINS["AIN2"]), ("U2", "AIN2"), "AIN2", "green", 28, SOLDER_PAD, SOLDER_THT, "signal"),
    ("W15", ("U3", PINS["BIN1"]), ("U2", "BIN1"), "BIN1", "blue", 28, SOLDER_PAD, SOLDER_THT, "signal"),
    ("W16", ("U3", PINS["BIN2"]), ("U2", "BIN2"), "BIN2", "violet", 28, SOLDER_PAD, SOLDER_THT, "signal"),
    ("W17", ("U3", PINS["nSLEEP"]), ("U2", "SLP"), "nSLEEP", "brown", 28, SOLDER_PAD, SOLDER_THT + " (R3's leg in the same hole)", "signal"),
    ("W18", ("U3", PINS["nFAULT"]), ("U2", "FLT"), "nFAULT", "grey", 28, SOLDER_PAD, SOLDER_THT, "signal"),
    ("W19", ("U2", "AOUT1"), ("J2", "1"), "MA1", "white", 24, SOLDER_THT, _SMP, "motor"),
    ("W20", ("U2", "AOUT2"), ("J2", "2"), "MA2", "blue", 24, SOLDER_THT, _SMP, "motor"),
    ("W21", ("J2", "1"), ("M1", "1"), "MA1", "white", 24, _SMR, "soldered to the motor tab (C3 across the tabs)", "motor"),
    ("W22", ("J2", "2"), ("M1", "2"), "MA2", "blue", 24, _SMR, "soldered to the motor tab", "motor"),
    ("W23", ("U2", "BOUT1"), ("J3", "1"), "MB1", "white", 24, SOLDER_THT, _SMP, "motor"),
    ("W24", ("U2", "BOUT2"), ("J3", "2"), "MB2", "blue", 24, SOLDER_THT, _SMP, "motor"),
    ("W25", ("J3", "1"), ("M2", "2"), "MB1", "white", 24, _SMR, "soldered to the motor tab (C4 across the tabs; leads crossed)", "motor"),
    ("W26", ("J3", "2"), ("M2", "1"), "MB2", "blue", 24, _SMR, "soldered to the motor tab", "motor"),
]
# Pins joined by a component's own lead, not by a wire: (pin a, pin b, how).
JOINTS = [
    (("D1", "K"), ("U3", "5V"), "D1's cathode (banded end) soldered onto the 5V pad"),
    (("R1", "2"), ("U3", PINS["VSENSE"]), "R1's leg soldered onto the D10 pad"),
    (("R2", "1"), ("U3", PINS["VSENSE"]), "R2 from the D10 pad ..."), (("R2", "2"), ("U3", "GND"), "... to the GND pad, sleeved over 3V3"),
    (("C2", "1"), ("U3", PINS["VSENSE"]), "C2 beside R2, D10 pad ..."), (("C2", "2"), ("U3", "GND"), "... to the GND pad"),
    (("R3", "1"), ("U2", "SLP"), "R3 from SLP ..."), (("R3", "2"), ("U2", "GND"), "... to GND on the carrier, sleeved"),
    (("C1", "+"), ("U2", "VIN"), "C1 + leg into VIN, lying flat on the tray"), (("C1", "-"), ("U2", "GND"), "C1 - leg into GND"),
    (("C3", "1"), ("M1", "1"), "C3 across M1's tabs"), (("C3", "2"), ("M1", "2"), ""),
    (("C4", "1"), ("M2", "1"), "C4 across M2's tabs"), (("C4", "2"), ("M2", "2"), ""),
]


PASSIVE_G = {"D1": 0.35, "R1": 0.2, "R2": 0.2, "R3": 0.2, "C2": 0.1, "C3": 0.1, "C4": 0.1}   # ASSUMED, axial / disc parts


def electrical_mass_g(r: Robot, lengths=None) -> dict:
    """What the electrical design adds on top of BOUGHT (the XIAO, driver, holder and cells are there)."""
    parts = sum(m["g"] for m in MOUNTS.values()) + sum(PASSIVE_G.values())
    wire = 0.0
    for w in wire_list(r, lengths):
        if w["connector_from"] == "holder's own lead":
            continue                                   # part of the holder's mass
        d_mm = {22: 0.6438, 24: 0.5106, 26: 0.4049, 28: 0.3211}[w["awg"]]
        wire += math.pi * (d_mm / 2) ** 2 * w["length_mm"] / 1000.0 * 8.96 * 1.6
    return {"parts": parts, "wire": wire, "total": parts + wire}


def _ceil5(v: float) -> float:
    return 5.0 * math.ceil(v / 5.0 - 1e-9)


def wire_list(r: Robot, lengths: Optional[Dict[str, float]] = None) -> List[dict]:
    """Every physical wire. `lengths` (mm, by wire id) comes from the 3D routing when it
    exists; until then each length is ESTIMATED: the straight line between its two pins
    x 1.3, plus 15 mm for stripping, the joint and a service loop, rounded up to 5 mm.
    The holder's own leads are what is left after cutting them to that length."""
    pos = positions(r)
    out = []
    for wid, a, b, net, colour, awg, ca, cb, carries in WIRES:
        d = math.dist(pos[a], pos[b])
        est = _ceil5(1.3 * d + 15.0)
        given = (lengths or {}).get(wid)
        out.append({"id": wid, "from": a, "to": b, "net": net, "colour": colour, "awg": awg,
                    "connector_from": ca, "connector_to": cb, "carries": carries,
                    "straight_mm": round(d, 1), "length_mm": float(given) if given is not None else est,
                    "length_source": "routed" if given is not None else "estimate",
                    "ohm": AWG[awg][0] * (given if given is not None else est) / 1e6})
    return out


# ---- 5. the power budget -------------------------------------------------------------------

def _paths(wires: Sequence[dict]) -> dict:
    w = {x["id"]: x["ohm"] for x in wires}
    c = JST_SM["r_contact"]
    return {
        # Battery to the driver, out and back: everything every amp of the motors goes through.
        "shared": w["W1"] + w["W3"] + w["W4"] + w["W2"] + w["W5"] + w["W6"] + w["W7"] + 2 * c + HOLDER["r_contacts"],
        "motor_a": w["W19"] + w["W21"] + w["W20"] + w["W22"] + 2 * c,
        "motor_b": w["W23"] + w["W25"] + w["W24"] + w["W26"] + 2 * c,
        "logic_out": w["W9"] + w["W10"] + w["W11"],
    }


LOGIC = {"off": 0.0, "modem_sleep": XIAO["p_modem_sleep_w"], "wifi": XIAO["p_wifi_w"],
         "tx_peak": ESP["vdd"] * ESP["i_tx_peak"] / 0.9}     # 340 mA at 3.3 V through a ~90 % buck (ASSUMED)


SOFT_START_DUTY = 0.5    # the firmware ramps a start from rest: at most this duty while a wheel is still


def operating_point(r: Robot, chem: str, state: str, motors: str, logic: str, worst: bool = False,
                    hot: bool = False, lengths=None, duty: float = 1.0) -> dict:
    """Solve the circuit: pack (EMF, internal R) -> shared path -> driver + regulator.

    motors: 'off' | 'free' (both at free-run current, full duty) | 'stall' (both held: a
    start from rest, or a wheel against a wall) at `duty`. With slow-decay PWM the motor's
    current recirculates in the off time: it averages duty x V / R, and the pack supplies
    duty x that. worst=True takes the low end of every resistance, so the currents are the
    highest the parts could see (for the ratings); otherwise typical values."""
    cell = CELLS[chem]
    n = 4
    v_cell = cell[f"v_{state}"]
    r_cell = cell["r_cell"][0] * (0.8 if chem == "nimh" else 1.0) if worst else sum(cell["r_cell"]) / 2
    if chem == "nimh" and not worst:
        r_cell = cell["r_cell"][0] if state == "fresh" else cell["r_cell"][1]
    paths = _paths(wire_list(r, lengths))
    r_fuse = FUSE["r_min"] if worst else (FUSE["r_min"] + FUSE["r_max"]) / 2
    r_sw = SWITCH["rds_4v5"] if worst else SWITCH["rds_max"]
    r_shared = n * r_cell + paths["shared"] + r_fuse + r_sw
    r_bridge = DRV["r_bridge_85c"] if hot else DRV["r_bridge"]
    r_ma = MOTOR["r_term"] + r_bridge + paths["motor_a"]
    r_mb = MOTOR["r_term"] + r_bridge + paths["motor_b"]
    emf = n * v_cell
    p_logic = LOGIC[logic]
    vf = DIODE["vf_min"] if worst else DIODE["vf_1a"]
    i_bat, vb = 0.0, emf
    for _ in range(200):
        vb = emf - i_bat * r_shared
        d = duty
        if motors == "stall":
            ia, ib = d * vb / r_ma, d * vb / r_mb
        elif motors == "free":
            i0 = MOTOR["i_free"] * ((1 + MOTOR["i_free_tol"]) if worst else 1.0)
            ia, ib, d = i0, i0, 1.0
        else:
            ia = ib = 0.0
        v_vbus = REG["vout"] - vf
        i_vbus = p_logic / max(v_vbus, 1e-6)
        p_reg_out = REG["vout"] * i_vbus
        i_reg_in = (p_reg_out / REG["eta"]) / max(vb, 0.1) + REG["iq"]
        slp = sleep_pin(vb)
        awake = logic != "off"
        i_misc = (DRV["i_vm_max"] if worst else DRV["i_vm"]) + SWITCH["i_on_per_v"] * vb \
            + vb / (DIVIDER["r_top"] + DIVIDER["r_bot"]) + (max(vb - ESP["vdd"], 0) / DRV["slp_pullup"] if awake
                                                            else slp["i_asleep_ua"] * 1e-6)
        i_new = d * (ia + ib) + i_reg_in + i_misc
        if abs(i_new - i_bat) < 1e-9:
            break
        i_bat = 0.5 * i_bat + 0.5 * i_new
    v_ma = d * vb - ia * (r_bridge + paths["motor_a"])     # the average voltage across the motor
    w = (v_ma - ia * MOTOR["r_term"]) / MOTOR["ke"] if motors == "free" else 0.0
    return {"chem": chem, "state": state, "motors": motors, "logic": logic, "worst": worst, "duty": d if motors != "off" else 0.0,
            "emf_v": emf, "r_pack": n * r_cell, "r_shared": r_shared, "v_bus": vb, "i_bat": i_bat,
            "i_motor_a": ia, "i_motor_b": ib, "v_motor_a": v_ma, "i_reg_in": i_reg_in, "i_vbus": i_vbus,
            "p_reg_loss": p_reg_out / REG["eta"] - p_reg_out, "p_diode": vf * i_vbus,
            "v_vbus": REG["vout"] - vf, "p_driver": 2 * ia ** 2 * r_bridge if motors != "off" else 0.0,
            "speed_m_s": w * r.wheel_d / 2000.0, "reg_in_range": REG["vin_min"] <= vb <= REG["vin_max"],
            "vm_in_range": DRV["vm_min"] <= vb <= DRV["vm_max"]}


def capacity_mah(chem: str, i: float) -> float:
    c = CELLS[chem]
    return c["c_ref_mah"] * (c["i_ref"] / max(i, 1e-6)) ** (c["peukert"] - 1.0)


def power_budget(r: Robot, lengths=None) -> dict:
    cases = {
        "idle (Wi-Fi on, motors off)": ("off", "wifi", 1.0),
        "driving straight (both free-running, Wi-Fi on)": ("free", "wifi", 1.0),
        f"start from rest, soft start ({SOFT_START_DUTY:.0%} duty, TX peak)": ("stall", "tx_peak", SOFT_START_DUTY),
        "both stalled at full duty (TX peak)": ("stall", "tx_peak", 1.0),
    }
    table = []
    for label, (mot, logic, duty) in cases.items():
        for chem in ("alkaline", "nimh"):
            for state in ("fresh", "mid", "end"):
                op = operating_point(r, chem, state, mot, logic, lengths=lengths, duty=duty)
                op["case"] = label
                table.append(op)
    # The highest currents the parts could see: fresh NiMH (lowest internal resistance), cold driver.
    worst = {k: operating_point(r, "nimh", "fresh", m, l, worst=True, lengths=lengths)
             for k, (m, l) in {"idle": ("off", "tx_peak"), "drive": ("free", "tx_peak"), "stall": ("stall", "tx_peak")}.items()}
    worst["stall_alkaline"] = operating_point(r, "alkaline", "fresh", "stall", "tx_peak", worst=True, lengths=lengths)
    runtime = {}
    for chem in ("alkaline", "nimh"):
        for label, mot in (("idle, Wi-Fi on", "off"), ("driving continuously, Wi-Fi on", "free")):
            op = operating_point(r, chem, "mid", mot, "wifi", lengths=lengths)
            c = capacity_mah(chem, op["i_bat"])
            runtime[(chem, label)] = {"i_bat": op["i_bat"], "capacity_mah": c, "hours": c / 1000.0 / op["i_bat"]}
    # PWM charge ripple on C1: both motors at 50 % duty with the current one stalled motor takes
    # at that duty (slow decay keeps it flowing), the worst case for I D (1 - D) / (f C).
    i_pulse = 2 * 0.5 * worst["stall"]["v_bus"] / (MOTOR["r_term"] + DRV["r_bridge"])
    ripple = i_pulse * 0.25 / (PWM_HZ * 100e-6)
    return {"table": table, "worst": worst, "runtime": runtime, "ripple_v": ripple, "i_pulse": i_pulse,
            # A short at J1, through the fuse: the pack's TYPICAL resistance (alkaline 225, NiMH 30 mohm a cell).
            "short_circuit_a": {c: 4 * CELLS[c]["v_fresh"] / (4 * (sum(CELLS[c]["r_cell"]) / 2 if c == "alkaline"
                                                                   else CELLS[c]["r_cell"][0]) + HOLDER["r_contacts"] + FUSE["r_max"])
                                for c in CELLS}}


# ---- 7. the checks -------------------------------------------------------------------------

MARGIN = 0.8     # a steady current must stay at or under 80 % of its continuous rating


def _union(pairs, nodes):
    parent = {n: n for n in nodes}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    loops = 0
    for a, b in pairs:
        ra, rb = find(a), find(b)
        if ra == rb:
            loops += 1
        else:
            parent[ra] = rb
    return {find(n) for n in nodes}, loops


def checks(r: Robot, lengths=None) -> Dict[str, dict]:
    """Every rule the electrical design must keep, each {ok, value, limit, why}."""
    comps, nets = components(), netlist()
    out: Dict[str, dict] = {}

    def rule(name, ok, value=None, limit=None, why=""):
        out[name] = {"ok": bool(ok), "value": value, "limit": limit, "why": why}

    every_pin = {(ref, p) for ref, c in comps.items() for p in c["pins"]}
    seen = defaultdict(list)
    for net, pins in nets.items():
        for p in pins:
            seen[p].append(net)
    rule("every_net_has_two_pins", all(len(v) >= 2 for v in nets.values()),
         min(len(v) for v in nets.values()), 2)
    rule("no_pin_on_two_nets", all(len(v) == 1 for v in seen.values()),
         [p for p, v in seen.items() if len(v) > 1], [])
    rule("every_net_pin_exists", set(seen) <= every_pin, sorted(set(seen) - every_pin), [])
    rule("every_pin_on_a_net_or_no_connect",
         every_pin == set(seen) | set(NO_CONNECT) and not (set(seen) & set(NO_CONNECT)),
         sorted(every_pin - set(seen) - set(NO_CONNECT)), [])
    # Wires and joints: each joins two pins of the same net; together they connect every net's
    # pins into one piece, with no loop (a ground loop, or a wire doubling a joint).
    wires = wire_list(r, lengths)
    net_of = {p: v[0] for p, v in seen.items()}
    bad = [w["id"] for w in wires if not (net_of.get(w["from"]) == net_of.get(w["to"]) == w["net"])]
    bad += [f"{a}-{b}" for a, b, _ in JOINTS if net_of.get(a) != net_of.get(b)]
    rule("wires_join_pins_of_their_net", not bad, bad, [])
    # A connector pair J is one part: both its halves are its pin; a wire on each side meets there.
    split, loops = [], 0
    for net, pins in nets.items():
        pairs = [(w["from"], w["to"]) for w in wires if w["net"] == net] + \
                [(a, b) for a, b, _ in JOINTS if net_of.get(a) == net]
        roots, n_loops = _union(pairs, pins)
        if len(roots) != 1:
            split.append(net)
        loops += n_loops
    rule("every_net_physically_connected", not split, split, [])
    rule("no_loops_in_any_net", loops == 0, loops, 0, "a ground loop would share motor current with logic ground")
    # Pin assignment.
    pads = XIAO["pads"]
    used = [PINS[k] for k in PINS]
    rule("no_gpio_used_twice", len(used) == len(set(used)), used)
    drive = [PINS[k] for k in ("AIN1", "AIN2", "BIN1", "BIN2")]
    rule("driver_inputs_on_pwm_capable_gpio", all(p in pads for p in drive), drive, "D0-D10 (Seeed: 11x GPIOs (PWM))")
    rule("driver_inputs_not_strapping", all(pads[p][0] not in ESP["strapping"] for p in drive + [PINS["nSLEEP"]]),
         [pads[p][0] for p in drive], sorted(ESP["strapping"]))
    rule("pwm_channels_available", len(drive) <= ESP["ledc_channels"], len(drive), ESP["ledc_channels"])
    rule("vsense_on_adc1", (pads[PINS["VSENSE"]][1] or "").startswith("ADC1"), pads[PINS["VSENSE"]][1], "ADC1_*",
         "ADC2 cannot be used with Wi-Fi on (ESP32-S3 datasheet)")
    # Logic levels.
    voh, vol = ESP["voh_frac"] * ESP["vdd"], ESP["vol_frac"] * ESP["vdd"]
    rule("gpio_high_drives_xin_high", voh >= DRV["vih"], round(voh, 3), DRV["vih"])
    rule("gpio_low_drives_xin_low", vol <= DRV["vil"], round(vol, 3), DRV["vil"])
    rule("gpio_high_wakes_nsleep", voh >= DRV["vih_sleep"], round(voh, 3), DRV["vih_sleep"])
    rule("gpio_low_sleeps_nsleep", vol <= DRV["vil_sleep"], round(vol, 3), DRV["vil_sleep"])
    rule("driver_inputs_tolerate_3v3", ESP["vdd"] <= DRV["vin_max"], ESP["vdd"], DRV["vin_max"])
    slp_hi = sleep_pin(4 * CELLS["alkaline"]["v_fresh"])
    slp_li = sleep_pin(NOMINAL_VSENSE_MAX)
    rule("driver_asleep_while_mcu_resets", slp_li["released_v"] <= DRV["vil_sleep"], round(slp_li["released_v"], 3),
         DRV["vil_sleep"], "R3 holds nSLEEP low against the carrier's 47k to VMM until the GPIO drives it")
    rule("nsleep_never_above_gpio_rail", max(slp_li["released_v"], ESP["vdd"]) <= ESP["vdd"] + ESP["pin_max_over_vdd"],
         round(slp_li["released_v"], 3), ESP["vdd"] + ESP["pin_max_over_vdd"])
    rule("nsleep_gpio_sources_current", 0 < slp_hi["gpio_source_ma"] < 40.0, round(slp_hi["gpio_source_ma"], 3), 40.0,
         "positive: no back-feed from VMM into the 3V3 rail through the 47k")
    a = adc_sense()
    rule("vsense_within_adc_range", a["adc_at_7v2"] * 1000 <= ESP["adc_atten3_mv"], round(a["adc_at_7v2"], 3),
         ESP["adc_atten3_mv"] / 1000)
    rule("nfault_pulled_to_gpio_rail", True, "internal pull-up to 3.3 V", "open drain, VOL 0.5 V at 5 mA")
    # Supplies.
    vb_max = REG["vout"] * (1 + REG["vout_tol"][1]) - DIODE["vf_min"]
    vb_min = REG["vout"] * (1 + REG["vout_tol"][0]) - DIODE["vf_1a"]
    rule("vbus_within_xiao_buck_input", vb_max - 0.0 <= XIAO["vbus_max"], round(vb_max, 3), XIAO["vbus_max"],
         "upper bound ignores the XIAO's own Schottky, so it is conservative")
    rule("vbus_enough_for_3v3", vb_min - XIAO["vf_onboard"] >= ESP["vdd"] + 0.3, round(vb_min - XIAO["vf_onboard"], 3),
         ESP["vdd"] + 0.3)
    pb = power_budget(r, lengths)
    # A full-duty stall on a pack at its cut-off is what the firmware's soft start rules out
    # (warnings() reports how far it would sag); every other row must keep the bus up.
    kept = [op for op in pb["table"] if not (op["motors"] == "stall" and op["duty"] > SOFT_START_DUTY and op["state"] == "end")]
    rule("bus_within_driver_vm_range", all(op["vm_in_range"] for op in kept),
         [round(op["v_bus"], 2) for op in kept if not op["vm_in_range"]], [DRV["vm_min"], DRV["vm_max"]])
    rule("bus_within_regulator_input", all(op["reg_in_range"] for op in kept),
         round(min(op["v_bus"] for op in kept), 3), REG["vin_min"], "else the XIAO browns out")
    rule("pack_within_switch_range", 4 * CELLS["alkaline"]["v_fresh"] <= SWITCH["v_max"], 4 * CELLS["alkaline"]["v_fresh"], SWITCH["v_max"])
    # Currents: steady cases at <= 80 % of continuous ratings, stalls within peak ratings.
    w = pb["worst"]
    drive_i = max(w["drive"]["i_motor_a"], w["drive"]["i_motor_b"])
    stall_i = max(w["stall"]["i_motor_a"], w["stall"]["i_motor_b"])
    rule("driving_per_channel_within_continuous", drive_i <= MARGIN * DRV["i_cont"], round(drive_i, 3), MARGIN * DRV["i_cont"])
    rule("stall_per_channel_within_peak", stall_i <= DRV["i_peak"], round(stall_i, 3), DRV["i_peak"])
    rule("stall_battery_within_switch", w["stall"]["i_bat"] <= SWITCH["i_cont_55c"], round(w["stall"]["i_bat"], 3), SWITCH["i_cont_55c"])
    rule("stall_battery_within_connector", w["stall"]["i_bat"] <= JST_SM["i_rating"], round(w["stall"]["i_bat"], 3), JST_SM["i_rating"])
    rule("stall_motor_within_connector", stall_i <= MARGIN * JST_SM["i_rating"], round(stall_i, 3), MARGIN * JST_SM["i_rating"])
    rule("fuse_holds_double_stall_at_23c", w["stall"]["i_bat"] < FUSE["i_hold"], round(w["stall"]["i_bat"], 3), FUSE["i_hold"])
    rule("fuse_trips_on_short", min(pb["short_circuit_a"].values()) > FUSE["i_trip"],
         {k: round(v, 1) for k, v in pb["short_circuit_a"].items()}, FUSE["i_trip"])
    rule("regulator_output_within_rating", w["stall"]["i_vbus"] <= MARGIN * REG["i_out_boost"], round(w["stall"]["i_vbus"], 3),
         MARGIN * REG["i_out_boost"], "the step-up rating (500 mA) applies once the pack is under 5 V")
    rule("diode_within_rating", w["stall"]["i_vbus"] <= MARGIN * DIODE["if_av"], round(w["stall"]["i_vbus"], 3), MARGIN * DIODE["if_av"])
    carry = {"battery": w["stall"]["i_bat"], "motors": w["stall"]["i_motor_a"] + w["stall"]["i_motor_b"],
             "motor": stall_i, "regulator": w["stall"]["i_reg_in"], "logic": w["stall"]["i_vbus"],
             "sense": 1e-4, "signal": 2e-3}
    over = [(x["id"], round(carry[x["carries"]], 3), AWG[x["awg"]][1]) for x in wires
            if x["connector_from"] != "holder's own lead" and carry[x["carries"]] > AWG[x["awg"]][1]]
    rule("new_wires_within_ampacity_at_stall", not over, over, "Powerstream chassis wiring", AWG_SOURCE)
    rule("wires_fit_their_connectors", all(JST_SM["awg"][1] <= x["awg"] <= JST_SM["awg"][0] for x in wires
                                           if "JST" in x["connector_from"] + x["connector_to"]),
         sorted({x["awg"] for x in wires if "JST" in x["connector_from"] + x["connector_to"]}), JST_SM["awg"])
    rule("bulk_cap_ripple", pb["ripple_v"] <= 0.25, round(pb["ripple_v"], 3), 0.25, "C1 charge ripple at 20 kHz, 50 % duty")
    return out


def warnings(r: Robot, lengths=None) -> List[str]:
    """What the design passes but someone must know: ratings met only as transients, and
    places where design.py's assumptions and the datasheets disagree."""
    pb = power_budget(r, lengths)
    w = pb["worst"]
    stall_i = max(w["stall"]["i_motor_a"], w["stall"]["i_motor_b"])
    out = []
    if stall_i > DRV["i_cont"]:
        out.append(f"A stalled motor draws {stall_i:.2f} A per channel, over the DRV8833 carrier's {DRV['i_cont']} A continuous "
                   f"rating (within its {DRV['i_peak']} A peak). The chip's over-current trip is {DRV['i_ocp_min']}-"
                   f"{DRV['i_ocp_max']} A, so it never trips on a stall: thermal shutdown (die {DRV['t_tsd_min_c']:.0f} C min) "
                   "is what ends a long one (Pololu: 1.5 A lasts about a minute). The carrier's current sensing is off and "
                   "the motors have no encoders, so the firmware must cap full-duty time (stall timeout).")
    else:
        out.append(f"A stalled motor draws {stall_i:.2f} A per channel: within the carrier's {DRV['i_cont']} A continuous "
                   f"rating by only {100 * (1 - stall_i / DRV['i_cont']):.0f} %. The chip's over-current trip is "
                   f"{DRV['i_ocp_min']}-{DRV['i_ocp_max']} A, so it never trips on a stall; thermal shutdown "
                   f"(die {DRV['t_tsd_min_c']:.0f} C min) ends a long one. Current sensing is off and the motors have no "
                   "encoders: the firmware must cap full-duty time (stall timeout).")
    sag = [op for op in pb["table"] if op["motors"] == "stall" and op["duty"] == 1.0 and op["state"] == "end"]
    low = [op for op in sag if not op["reg_in_range"]]
    if low:
        out.append("A full-duty start from rest on a pack at its 1.0 V/cell cut-off sags the bus to "
                   + ", ".join(f"{op['v_bus']:.2f} V ({op['chem']})" for op in low)
                   + f", under the regulator's {REG['vin_min']} V and the DRV8833's {DRV['vm_min']} V: the XIAO would reset. "
                   f"The firmware must ramp every start (at most {SOFT_START_DUTY:.0%} duty while the wheels are still) "
                   "and stop driving when D10 reads the pack under 4.0 V.")
    if w["stall"]["i_bat"] > AWG[HOLDER["lead_awg"]][1]:
        out.append(f"Both motors stalled draw {w['stall']['i_bat']:.2f} A from the pack, over the {AWG[HOLDER['lead_awg']][1]} A "
                   f"rule-of-thumb for the holder's own #{HOLDER['lead_awg']} AWG leads: acceptable as a transient, not as "
                   "a sustained stall.")
    if w["stall"]["i_bat"] > FUSE["i_hold_40c"]:
        out.append(f"F1 holds {FUSE['i_hold']} A at 23 C but {FUSE['i_hold_40c']} A at 40 C: a sustained double stall "
                   f"({w['stall']['i_bat']:.2f} A) in a warm room may trip it, which protects the motors.")
    fresh = [op for op in pb["table"] if op["chem"] == "alkaline" and op["state"] == "fresh" and op["motors"] == "free"][0]
    if fresh["v_motor_a"] > MOTOR["v_rated"]:
        out.append(f"Fresh alkaline cells put {fresh['v_motor_a']:.2f} V on the 6 V motors at full duty: cap the duty at "
                   "6.0 / V_battery (D10 measures it).")
    nimh = [op for op in pb["table"] if op["chem"] == "nimh" and op["state"] == "mid" and op["motors"] == "free"][0]
    out.append(f"design.py / README quote 0.68 m/s at 6 V. The pack gives the motors {fresh['v_motor_a']:.2f} V fresh alkaline "
               f"but {nimh['v_motor_a']:.2f} V on NiMH mid-discharge: {nimh['speed_m_s']:.2f} m/s, not 0.68.")
    out.append(f"BOUGHT['cells'] is 4 x 23 g alkaline; NiMH cells are {CELLS['nimh']['g']:.0f} g each (+{4 * (CELLS['nimh']['g'] - 23):.0f} g).")
    out.append("BOUGHT['battery_holder'] cites Keystone 2478 on catalogue page M65p28, which covers 1/2 AA, AAA and N "
               "holders, not 4 x AA: check the part number (that page's wire-lead holders have #26 AWG, 150 mm leads, "
               "the figure used here).")
    out.append("BOUGHT['gearmotor'] matches Pololu's page (330 rpm, 1.5 A, 1.6 kg cm, 9.5 g); the page also gives "
               "0.15 A free current (+-50 %) and 100.37:1, and lists the body as 10 x 12 x 25 mm (design.py: 25.6 from the drawing).")
    out.append("The plates have no place for SW1, U1, F1 or C1 yet: MOUNTS gives each one's footprint, mass and proposed "
               f"place (SW1 at {MOUNTS['SW1']['at']}, U1 at {MOUNTS['U1']['at']} on the bottom plate's top); the holes, "
               "if any, are the mechanical design's to make.")
    out.append("The bottom plate's lead slot at (15, 0) is not needed: the motors and the tray sit on the same face.")
    return out


# ---- 6. the schematic and the wiring diagram as SVG ----------------------------------------

INK, MUTED, PAPER, ACCENT = "#1b1f24", "#5d6873", "#ffffff", "#c0392b"
WIRE_RGB = {"red": "#d62828", "black": "#1b1b1b", "orange": "#f08c00", "yellow": "#e9c400", "green": "#2b9348",
            "blue": "#1d4ed8", "violet": "#7b2cbf", "brown": "#8b5a2b", "grey": "#8d99ae", "white": "#f5f5f5"}
SCH_LEFT = ["BT1", "F1", "J1", "SW1", "U1", "D1", "U3", "R1", "R2", "C2"]
SCH_RIGHT = ["U2", "C1", "R3", "J2", "M1", "C3", "J3", "M2", "C4"]


def _t(x, y, s, size=11, anchor="start", fill=INK, weight="normal", family="Helvetica, Arial, sans-serif"):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-family="{family}" font-size="{size}" text-anchor="{anchor}" '
            f'fill="{fill}" font-weight="{weight}">{escape(str(s))}</text>')


def schematic_svg(r: Robot) -> str:
    """Block-and-pin schematic: components in two columns, each pin facing the channel between
    them; every net is one vertical trunk in the channel with a horizontal stub to each of its
    pins, a dot where a stub joins its trunk, and the net's name at the top of the trunk and
    beside every pin. Stubs cross other trunks without a dot (no connection)."""
    comps, nets = components(), netlist()
    net_of = {p: n for n, ps in nets.items() for p in ps}
    pitch, box_w, gap = 18.0, 150.0, 26.0
    lx, top = 40.0, 140.0
    order_nets = sorted(nets, key=lambda n: (
        0 if all(p[0] in SCH_LEFT for p in nets[n]) else 2 if all(p[0] in SCH_RIGHT for p in nets[n]) else 1,
        min(SCH_LEFT.index(p[0]) if p[0] in SCH_LEFT else 50 + SCH_RIGHT.index(p[0]) for p in nets[n])))
    trunk_dx = 16.0
    ch_x0 = lx + box_w + 70.0
    trunk_x = {n: ch_x0 + i * trunk_dx for i, n in enumerate(order_nets)}
    rx = ch_x0 + len(order_nets) * trunk_dx + 70.0
    pin_xy, body = {}, []

    def column(refs, x, pins_on_right):
        y = top
        for ref in refs:
            c = comps[ref]
            h = pitch * len(c["pins"]) + 40.0
            body.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{box_w}" height="{h:.1f}" rx="4" fill="#f7f4ee" '
                        f'stroke="{INK}" stroke-width="1.4"/>')
            body.append(_t(x + box_w / 2, y + 15, ref, 12, "middle", weight="bold"))
            short = c["part"].split(",")[0]
            short = short if len(short) <= 30 else short[:29] + "..."
            body.append(_t(x + (box_w - 8 if pins_on_right else 8), y + h - 6, short, 7.5,
                           "end" if pins_on_right else "start", MUTED))
            for k, p in enumerate(c["pins"]):
                py = y + 30 + pitch * k
                px = x + box_w if pins_on_right else x
                pin_xy[(ref, p)] = (px, py)
                body.append(_t(px + (-6 if pins_on_right else 6), py + 4, p, 10, "end" if pins_on_right else "start"))
                if (ref, p) in NO_CONNECT:
                    sx = px + (14 if pins_on_right else -14)
                    body.append(f'<line x1="{px:.1f}" y1="{py:.1f}" x2="{sx:.1f}" y2="{py:.1f}" stroke="{MUTED}" stroke-width="1"/>')
                    body.append(f'<path d="M{sx - 4:.1f},{py - 4:.1f} l8,8 m0,-8 l-8,8" stroke="{ACCENT}" stroke-width="1.4"/>')
            y += h + gap
        return y

    y_end = max(column(SCH_LEFT, lx, True), column(SCH_RIGHT, rx, False))
    lines = []
    for n in order_nets:
        tx = trunk_x[n]
        ys = [pin_xy[p][1] for p in nets[n]]
        col = "#1d4ed8" if n not in ("GND",) else INK
        col = "#b02a2a" if n in ("VBAT_RAW", "VBAT", "VSW", "VMM", "5V_REG", "VBUS") else col
        lines.append(f'<line x1="{tx:.1f}" y1="{min(ys):.1f}" x2="{tx:.1f}" y2="{max(ys):.1f}" stroke="{col}" stroke-width="1.6"/>')
        lines.append(f'<g transform="translate({tx + 3:.1f},{min(ys) - 6:.1f}) rotate(-60)">{_t(0, 0, n, 9, "start", col, "bold")}</g>')
        for p in nets[n]:
            px, py = pin_xy[p]
            lines.append(f'<line x1="{px:.1f}" y1="{py:.1f}" x2="{tx:.1f}" y2="{py:.1f}" stroke="{col}" stroke-width="1.2"/>')
            lines.append(f'<circle cx="{tx:.1f}" cy="{py:.1f}" r="2.8" fill="{col}"/>')
            right_side = px < tx
            lines.append(_t(px + (10 if right_side else -10), py - 3, n, 7.5, "start" if right_side else "end", col))
    width, height = rx + box_w + 40.0, y_end + 30.0
    head = [_t(lx, 30, "OmniSim desk robot: schematic", 18, weight="bold"),
            _t(lx, 46, "Generated by electrical.py; do not hand-edit.", 10, fill=MUTED),
            _t(lx, 60, "A dot joins a pin to its net; a crossing without a dot is no connection; a red X is a no-connect.",
               10, fill=MUTED),
            _t(lx, 74, f"XIAO: AIN1 {PINS['AIN1']}, AIN2 {PINS['AIN2']}, BIN1 {PINS['BIN1']}, BIN2 {PINS['BIN2']}, "
               f"nSLEEP {PINS['nSLEEP']}, nFAULT {PINS['nFAULT']}, battery sense {PINS['VSENSE']}. "
               f"PWM {PWM_HZ / 1e3:.0f} kHz, {ledc_bits()}-bit LEDC.", 10, fill=MUTED)]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
            f'viewBox="0 0 {width:.0f} {height:.0f}">\n<rect width="100%" height="100%" fill="{PAPER}"/>\n'
            + "\n".join(head + body + lines) + "\n</svg>\n")


def wiring_svg(r: Robot, lengths=None) -> str:
    """Top view of the bottom plate (x to the right = forward, y up = left), every wire as a
    straight run between its pins in its colour and gauge, labelled with its id and length."""
    s, m = 6.0, 60.0
    hx, hy = r.plate_x / 2, r.plate_y / 2
    W, H = 2 * hx * s + 2 * m + 470, 2 * hy * s + 2 * m + 60

    def P(x, y):
        return m + (x + hx) * s, m + 40 + (hy - y) * s
    g = []
    x0, y0 = P(-hx, hy)
    g.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{2 * hx * s:.1f}" height="{2 * hy * s:.1f}" rx="{r.corner_r * s:.1f}" '
             f'fill="#f1e4cf" stroke="{INK}" stroke-width="1.2"/>')

    def rect(cx, cy, w, h, fill, label, sub=""):
        a, b = P(cx - w / 2, cy + h / 2)
        g.append(f'<rect x="{a:.1f}" y="{b:.1f}" width="{w * s:.1f}" height="{h * s:.1f}" fill="{fill}" stroke="{INK}" '
                 f'stroke-width="0.9" rx="2"/>')
        cxp, cyp = P(cx, cy)
        g.append(_t(cxp, cyp - (2 if sub else -4), label, 10, "middle", weight="bold"))
        if sub:
            g.append(_t(cxp, cyp + 11, sub, 8, "middle", MUTED))
    mb = BOUGHT["gearmotor"]["body_mm"]
    for j in (1, -1):
        rect(r.axle_x, j * (r.faceplate_y - mb[2] / 2), mb[0], mb[2], "#c9ccd1", "M1" if j > 0 else "M2", "left" if j > 0 else "right")
    sx, sy = r.standoff_xy
    for i in (-1, 1):
        for j in (-1, 1):
            cx, cy = P(i * sx, j * sy)
            g.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{2.75 * s:.1f}" fill="#d0d4d9" stroke="{INK}" stroke-width="0.8"/>')
    rect(r.tray_x, 0, TRAY["x"], TRAY["y"], "#cfe0f2", "")
    rect(r.tray_x + TRAY["pockets"][0][0], 0, 21.0, 17.8, "#1f6f43", "U3 XIAO", "USB-C <- -x")
    rect(r.tray_x + TRAY["pockets"][1][0], 0, 20.3, 12.7, "#2f8f4f", "U2 DRV8833")
    for ref, label in (("SW1", "SW1"), ("U1", "U1 5V")):
        fx, fy, _ = MOUNTS[ref]["footprint_mm"]
        rect(*MOUNTS[ref]["at"], fx, fy, "#e6d3a3", label, "proposed")
    cx_, cy_ = P(r.caster_x, 0)
    g.append(f'<circle cx="{cx_:.1f}" cy="{cy_:.1f}" r="{4.76 * s:.1f}" fill="none" stroke="{MUTED}" stroke-dasharray="2 2"/>')
    g.append(_t(cx_, cy_ + 4.76 * s + 12, "caster (below)", 8, "middle", MUTED))
    hcx, hcy = P(0, 34)
    g.append(f'<circle cx="{hcx:.1f}" cy="{hcy:.1f}" r="{4 * s:.1f}" fill="none" stroke="{INK}" stroke-dasharray="4 3"/>')
    g.append(_t(hcx + 28, hcy - 20, "top-plate hole: battery leads down (BT1, F1, J1 above)", 8, fill=MUTED))
    pos = positions(r)
    wires = wire_list(r, lengths)
    for w in wires:
        (ax, ay, _), (bx, by, _) = pos[w["from"]], pos[w["to"]]
        (pa, pb), (qa, qb) = P(ax, ay), P(bx, by)
        width = {22: 3.4, 24: 2.8, 26: 2.2, 28: 1.6}[w["awg"]]
        g.append(f'<line x1="{pa:.1f}" y1="{pb:.1f}" x2="{qa:.1f}" y2="{qb:.1f}" stroke="#202020" stroke-width="{width + 1.4}" '
                 f'stroke-linecap="round"/>')
        g.append(f'<line x1="{pa:.1f}" y1="{pb:.1f}" x2="{qa:.1f}" y2="{qb:.1f}" stroke="{WIRE_RGB[w["colour"]]}" '
                 f'stroke-width="{width}" stroke-linecap="round"/>')
    placed: List[Tuple[float, float]] = []
    for w in wires:
        (ax, ay, _), (bx, by, _) = pos[w["from"]], pos[w["to"]]
        # The label goes where it is farthest from the labels already placed, along its wire.
        cands = [P(ax + t * (bx - ax), ay + t * (by - ay)) for t in (0.5, 0.35, 0.65, 0.2, 0.8, 0.27, 0.73)]
        lx, ly = max(cands, key=lambda c: min([math.hypot(c[0] - q[0], (c[1] - q[1]) * 1.8) for q in placed] or [1e9]))
        placed.append((lx, ly))
        g.append(f'<rect x="{lx - 11:.1f}" y="{ly - 7:.1f}" width="22" height="12" rx="2" fill="#ffffff" fill-opacity="0.85"/>')
        g.append(_t(lx, ly + 3, w["id"], 8, "middle", weight="bold"))
    for (a, b), lab in ((P(hx + 4, 0), "front (+x)"), (P(-hx - 4, 0), "rear")):
        g.append(_t(a, b, lab, 9, "start" if lab.startswith("front") else "end", MUTED))
    # The legend: every wire, one line.
    lx0, ly0 = m + 2 * hx * s + 90, m + 40
    g.append(_t(lx0, ly0 - 14, "Wire   from -> to   AWG   length", 10, weight="bold"))
    for k, w in enumerate(wires):
        y = ly0 + 14 * k
        g.append(f'<line x1="{lx0:.1f}" y1="{y - 3:.1f}" x2="{lx0 + 22:.1f}" y2="{y - 3:.1f}" stroke="#202020" stroke-width="4.4"/>')
        g.append(f'<line x1="{lx0:.1f}" y1="{y - 3:.1f}" x2="{lx0 + 22:.1f}" y2="{y - 3:.1f}" stroke="{WIRE_RGB[w["colour"]]}" stroke-width="3"/>')
        g.append(_t(lx0 + 28, y, f"{w['id']}  {w['from'][0]}.{w['from'][1]} -> {w['to'][0]}.{w['to'][1]}  "
                    f"#{w['awg']}  {w['length_mm']:.0f} mm{'' if w['length_source'] == 'routed' else ' (est.)'}", 8.5,
                    family="Consolas, Menlo, monospace"))
    head = [_t(m, 30, "OmniSim desk robot: wiring (top view of the bottom plate)", 18, weight="bold"),
            _t(m, 48, "Straight runs between pins, coloured as the wire; the 3D routing replaces the estimated lengths. "
               "Battery, fuse and J1 hang under the top plate (not drawn to height).", 10, fill=MUTED)]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{max(H, ly0 + 14 * len(wires) + 30):.0f}" '
            f'viewBox="0 0 {W:.0f} {max(H, ly0 + 14 * len(wires) + 30):.0f}">\n<rect width="100%" height="100%" fill="{PAPER}"/>\n'
            + "\n".join(head + g) + "\n</svg>\n")


# ---- the release files ---------------------------------------------------------------------

def wires_md(r: Robot, lengths=None) -> str:
    comps = components()
    L = ["# Desk robot: wire list", "",
         "Generated by `electrical.py`; do not hand-edit. Every wire is soldered from the TOP: the boards lie flat in the "
         "tray's pockets, so nothing may stick out below them, and every connector is wire-to-wire (JST SM, 2.5 mm, "
         f"{JST_SM['i_rating']:g} A, AWG {JST_SM['awg'][1]}-{JST_SM['awg'][0]}). Lengths marked *est.* are the straight "
         "line between the pins x 1.3 + 15 mm, until the 3D routing fills them in.", "",
         "| Wire | From | To | Net | Colour | AWG | End at From | End at To | Length (mm) |",
         "|---|---|---|---|---|---|---|---|---|"]
    for w in wire_list(r, lengths):
        L.append(f"| {w['id']} | {w['from'][0]}.{w['from'][1]} | {w['to'][0]}.{w['to'][1]} | {w['net']} | {w['colour']} | "
                 f"{w['awg']} | {w['connector_from']} | {w['connector_to']} | {w['length_mm']:.0f}"
                 f"{'' if w['length_source'] == 'routed' else ' *est.*'} |")
    L += ["", "Gauge: #22 where the battery and both motors' current runs (up to the double-stall current, see POWER.md), "
          "#24 on each motor (a stalled motor, within JST SM's 3 A), #26 for the regulator and the XIAO's 5 V and its "
          "return, #28 for signals (milliamps). Ampacity: Powerstream's chassis-wiring figures "
          f"({AWG_SOURCE}): #22 7 A, #24 3.5 A, #26 2.2 A, #28 1.4 A.", "",
          "## Joints (a part's own lead, no wire)", "", "| Pin | Pin | How |", "|---|---|---|"]
    for a, b, how in JOINTS:
        L.append(f"| {a[0]}.{a[1]} | {b[0]}.{b[1]} | {how} |")
    L += ["", "## Pin assignment on the XIAO ESP32-S3", "", "| Signal | Pad | GPIO | Why |", "|---|---|---|---|"]
    why = {"AIN1": "LEDC PWM, left motor", "AIN2": "LEDC PWM, left motor", "BIN1": "LEDC PWM, right motor",
           "BIN2": "LEDC PWM, right motor", "nSLEEP": "push-pull, HIGH wakes the driver (R3 keeps it asleep through reset)",
           "nFAULT": "input, internal pull-up; GPIO3's strap is ignored with default eFuses",
           "VSENSE": "ADC1_CH8, usable with Wi-Fi on"}
    for k, pad in PINS.items():
        L.append(f"| {k} | {pad} | GPIO{XIAO['pads'][pad][0]} | {why[k]} |")
    L += ["", "Free: " + ", ".join(f"{p[1]} ({why_})" for p, why_ in NO_CONNECT.items() if p[0] == "U3" and p[1].startswith("D")), "",
          "## Parts", "", "| Ref | Part | Source |", "|---|---|---|"]
    for ref, c in comps.items():
        L.append(f"| {ref} | {c['part']} | {c['source']} |")
    L += ["", "## Parts to mount (no hole exists for them yet)", "",
          "| Ref | Footprint x, y, h (mm) | Mass (g) | Proposed place | Held by | Source |", "|---|---|---|---|---|---|"]
    for ref, mnt in MOUNTS.items():
        L.append(f"| {ref} | {' x '.join(f'{v:g}' for v in mnt['footprint_mm'])} | {mnt['g']:g} | {mnt['at']} "
                 f"{mnt.get('slider', '')} | {mnt['fix']} | {mnt['source']} |")
    m = electrical_mass_g(r, lengths)
    L += ["", f"Mass the electrical design adds to the BOM: {m['total']:.1f} g ({m['parts']:.1f} g of parts above and "
          f"passives, {m['wire']:.1f} g of wire: copper x 1.6 for the insulation, ASSUMED)."]
    L += ["", "Connectors to crimp: J1, J2, J3, each one SMP-02V-BC plug (2 x SHF-001T-0.8BS) on the battery / driver side "
          "and one SMR-02V-B receptacle (2 x SYM-001T-P0.6) on the switch / motor side: the side that stays live carries "
          "the shrouded sockets.", ""]
    return "\n".join(L)


def power_md(r: Robot, lengths=None) -> str:
    pb = power_budget(r, lengths)
    a = adc_sense()
    L = ["# Desk robot: power budget", "",
         "Generated by `electrical.py`; do not hand-edit. Solved from the parts' datasheet figures (sources in "
         "`electrical.py`): the pack's EMF and internal resistance, the fuse, connectors, switch and wires (estimated "
         "lengths), the DRV8833's on-resistance at 25 C, the motor's 4.0 ohm (6 V / 1.5 A stall), and the XIAO as a "
         "constant-power load behind the 5 V regulator (85 % efficient) and the Schottky. **Nothing here is measured.**", "",
         f"Motor model (Pololu #3065 at 6 V): R = {MOTOR['r_term']:.2f} ohm, free {MOTOR['i_free']} A (+-50 %), "
         f"Ke = {MOTOR['ke']:.4f} V s/rad and Kt = {MOTOR['kt']:.4f} N m/A at the output shaft.", "",
         "| Case | Cells | State | EMF (V) | Pack R (ohm) | V at driver (V) | Battery (A) | Per channel (A) | V at motor, average (V) | Speed (m/s) | Regulator loss (mW) |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for op in pb["table"]:
        flag = " **(below 2.7 V)**" if not op["reg_in_range"] else ""
        L.append(f"| {op['case']} | {op['chem']} | {op['state']} | {op['emf_v']:.2f} | {op['r_pack']:.2f} | {op['v_bus']:.2f}{flag} | "
                 f"{op['i_bat']:.3f} | {op['i_motor_a']:.3f} | {op['v_motor_a']:.2f} | "
                 f"{op['speed_m_s']:.2f} | {op['p_reg_loss'] * 1e3:.0f} |")
    w = pb["worst"]
    L += ["", "States: alkaline 1.60 / 1.25 / 1.00 V per cell (1.0 V is the firmware's cut-off), internal resistance 0.225 ohm (E91: 150-300 mohm fresh; it "
          "rises as the cell drains, so the end-of-life rows are optimistic); NiMH 1.40 / 1.20 / 1.00 V, 30 / 40 / 40 mohm.", "",
          "## The highest currents the parts can see", "",
          "Fresh NiMH (lowest internal resistance, -20 %), fuse and switch at their minimum resistance, cold driver, "
          "free current at +50 %, the XIAO at its Wi-Fi TX peak.", "",
          "| Part | Rating | Driving | Both stalled | Margin at stall |", "|---|---|---|---|---|"]

    def row(part, rating, drive, stall):
        L.append(f"| {part} | {rating:g} A | {drive:.3f} A | {stall:.3f} A | {100 * (1 - stall / rating):+.0f} % |")
    sm = max(w["stall"]["i_motor_a"], w["stall"]["i_motor_b"])
    dm = max(w["drive"]["i_motor_a"], w["drive"]["i_motor_b"])
    row("DRV8833 channel, continuous (Pololu)", DRV["i_cont"], dm, sm)
    row("DRV8833 channel, peak (Pololu)", DRV["i_peak"], dm, sm)
    row("DRV8833 over-current trip, minimum (TI)", DRV["i_ocp_min"], dm, sm)
    row("SW1 MOSFET switch, 55 C", SWITCH["i_cont_55c"], w["drive"]["i_bat"], w["stall"]["i_bat"])
    row("F1 hold, 23 C", FUSE["i_hold"], w["drive"]["i_bat"], w["stall"]["i_bat"])
    row("F1 hold, 40 C", FUSE["i_hold_40c"], w["drive"]["i_bat"], w["stall"]["i_bat"])
    row("J1 JST SM", JST_SM["i_rating"], w["drive"]["i_bat"], w["stall"]["i_bat"])
    row("J2/J3 JST SM (one motor)", JST_SM["i_rating"], dm, sm)
    row("Holder leads #26 AWG (chassis rule)", AWG[26][1], w["drive"]["i_bat"], w["stall"]["i_bat"])
    row("U1 output, stepping up", REG["i_out_boost"], w["drive"]["i_vbus"], w["stall"]["i_vbus"])
    row("D1 1N5817", DIODE["if_av"], w["drive"]["i_vbus"], w["stall"]["i_vbus"])
    alk = w["stall_alkaline"]
    L += ["", f"With fresh alkaline cells (150 mohm each) a double stall draws {alk['i_bat']:.2f} A, {alk['i_motor_a']:.2f} A "
          "per channel: the alkaline pack's own resistance limits it.",
          f"Driver dissipation, both stalled (NiMH fresh): {w['stall']['p_driver']:.2f} W in a 20 x 13 mm carrier.",
          f"Short circuit at the pack (what F1 must stop): alkaline {pb['short_circuit_a']['alkaline']:.1f} A, NiMH "
          f"{pb['short_circuit_a']['nimh']:.1f} A; F1 trips above {FUSE['i_trip']} A ({FUSE['t_trip_s']} s at "
          f"{FUSE['t_trip_at']} A).",
          f"Bulk capacitor C1 (100 uF): PWM charge ripple at {PWM_HZ / 1e3:.0f} kHz, 50 % duty, {pb['i_pulse']:.2f} A pulses: "
          f"{pb['ripple_v']:.2f} V (I D (1 - D) / f C).", "",
          "## Runtime", "",
          "Mid-discharge pack. Alkaline capacity falls with drain: 2850 mAh at 25 mA (Energizer E91) scaled with an "
          "ASSUMED Peukert exponent of 1.2; NiMH 2300 mAh at 460 mA (Energizer NH15-2300) taken flat. Estimates, +-30 %.", "",
          "| Cells | Use | Battery (A) | Capacity (mAh) | Runtime (h) |", "|---|---|---|---|---|"]
    for (chem, label), v in pb["runtime"].items():
        L.append(f"| {chem} | {label} | {v['i_bat']:.3f} | {v['capacity_mah']:.0f} | {v['hours']:.1f} |")
    L += ["", "## Battery sense (D10)", "",
          f"R1 100 kohm over R2 47 kohm: ratio {a['ratio']:.4f}; the ADC's 0-2.9 V range (ATTEN3, 12 dB) reads the pack up to "
          f"{a['full_scale_pack_v']:.2f} V. 7.2 V (fresh lithium) -> {a['adc_at_7v2']:.2f} V, 6.4 V -> {a['adc_at_6v4']:.2f} V, "
          f"3.2 V -> {a['adc_at_3v2']:.2f} V. One LSB is {a['lsb_pack_mv']:.1f} mV of pack, the calibrated error +-50 mV is "
          f"+-{a['err_pack_mv']:.0f} mV of pack (+-{a['err_pack_mv'] / 4:.0f} mV per cell). Source resistance "
          f"{a['r_thevenin'] / 1e3:.1f} kohm with C2 100 nF: {a['tau_ms']:.1f} ms, PWM ripple x {a['pwm_ripple_attenuation']:.4f}. "
          f"It draws {a['i_divider_ua_at_6v4']:.0f} uA.", "",
          "## Flags", ""] + [f"- {x}" for x in warnings(r, lengths)] + [""]
    return "\n".join(L)


def write(out: Path = OUT, lengths=None) -> dict:
    r = Robot()
    out.mkdir(parents=True, exist_ok=True)
    (out / "schematic.svg").write_text(schematic_svg(r), encoding="utf-8", newline="\n")
    (out / "wiring.svg").write_text(wiring_svg(r, lengths), encoding="utf-8", newline="\n")
    (out / "WIRES.md").write_text(wires_md(r, lengths), encoding="utf-8", newline="\n")
    (out / "POWER.md").write_text(power_md(r, lengths), encoding="utf-8", newline="\n")
    return checks(r, lengths)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write the desk robot's electrical design (schematic, wiring, wires, power).")
    p.add_argument("--out", default=str(OUT))
    a = p.parse_args(argv)
    r = Robot()
    result = write(Path(a.out))
    pb = power_budget(r)
    for name, v in result.items():
        print(f"{'PASS' if v['ok'] else 'FAIL'}  {name}: {v['value']} (limit {v['limit']})")
    w = pb["worst"]
    print(f"stall, worst case: {w['stall']['i_motor_a']:.2f} A per channel, {w['stall']['i_bat']:.2f} A from the pack")
    for (chem, label), v in pb["runtime"].items():
        print(f"runtime {chem}, {label}: {v['hours']:.1f} h at {v['i_bat']:.3f} A")
    for x in warnings(r):
        print(f"FLAG  {x}")
    print(f"wrote {a.out}")
    return 0 if all(v["ok"] for v in result.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
