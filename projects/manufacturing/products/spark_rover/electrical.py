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

"""SparkRover Mini: the electrical and power design.

Parts with pins, the netlist, the pin assignment, every wire (gauge, colour, connector,
estimated length), every pin's position in the robot frame, the power budget and runtime,
the heat, the schematic and wiring diagrams, and the skeleton CONTRACT (parts, solids,
mounts, jobs, checks). The skeleton (skeleton.py) is the frame: x forward, y left, z up
from the floor under the wheels' centre, mm.

The architecture, decided here (each reason from the vendors' figures below):

- POWER. A 2S2P pack of four Molicel INR-18650-P28A cells (2800 mAh, 35 A each) with a 2S
  protection board, in the chassis's battery volume under the deck. Pack -> XT30 ->
  15 A MINI blade fuse -> Pololu #2813 pushbutton MOSFET switch (the rear button is its
  external pushbutton; the Pi's GPIO26 can switch the robot off after a shutdown) -> the
  switched bus VSW, which feeds:
    * a Pololu D42V55F5 (5 V, 6 A) for the Pi 5 (on its GPIO 5 V pins: the Pi wants 5 A for
      full USB current), the IO board, the LED ring, the amplifier and the accessory slot;
    * a second, separate 5 V regulator, a Pololu D36V28F5 (2.5-4 A, current-limited), for the
      four DYNAMIXEL XC330-M288-T servos (3.7-6.0 V, 5.0 V recommended): the 2S pack (6.0-8.4
      V) cannot feed them, and a servo fault (stall with its current limit off: 4 x 1.8 A)
      is held by this regulator's over-current limit, away from the Pi's rail;
    * two Pololu Motoron M2T256 dual motor controllers (4.5-48 V, 1.8 A continuous, 6.4 A
      peak per channel), straight from VSW: the 12 V motors never exceed their rating.
  The charger (an Injoinic IP2326 2S boost charger module, 5 V USB-C in, CC/CV 8.4 V out)
  sits on the pack side of the switch, so the robot charges switched off.
- MOTION I/O. A Raspberry Pi Pico (RP2040) is the motion co-processor: four quadrature
  encoders (48 CPR on the motor shaft, up to ~10 kHz an edge stream) on PIO state machines,
  the two Motorons on its I2C0, the servo bus on its UART1 through a half-duplex buffer
  (SN74LVC125A, direction from GP3), and the WS2812 LED ring on PIO through an SN74AHCT125.
  Why not the Pi directly: Linux does not count four 10 kHz quadrature streams or time a
  1 Mbps half-duplex turnaround reliably; PIO does both in hardware, and a Pico that stops
  hearing from the Pi stops the motors (the Motoron's own command timeout backs it up).
  The Pi talks to the Pico on a 1 Mbps UART (Pi uart0) and can reset (RUN) and reflash
  (SWD) it without opening the robot.
- HUMAN I/O. A Waveshare 5" 800 x 480 DSI panel (50H-800480-IPS, no touch) on the Pi's
  DSI1; the Raspberry Pi Camera Module 3 on CAM0, its lens on the skeleton's CAMERA axis;
  two SPH0645 I2S MEMS mics and a MAX98357A I2S amp sharing the Pi's PCM pins (the Google
  AIY Voice HAT arrangement); an Adafruit 3 W 4 ohm enclosed speaker behind the grille; an
  MPR121 capacitive controller with three copper-tape pads inside the chest window; two 3 mm
  status LEDs in the camera bar; 14 WS2812B pixels behind the bumper's diffuser; a 2 x 4
  2.54 mm accessory header in the top slot (5 V fused 0.5 A, 3.3 V, I2C1, uart2).

Every figure carries its source; "ASSUMED" marks a number no vendor page gave (it is used
and said so). Constants other subsystems own are in one place each: MOTOR and ENCODER
(chassis), SERVO and SERVO_PLUG (arms), PACK_VOLUME (chassis).

    python projects/manufacturing/products/spark_rover/electrical.py
        # writes release/electrical/{schematic.svg, wiring.svg, WIRES.md, POWER.md, PLACEMENT.md}
        # and meshes/electrical/*.stl
"""
from __future__ import annotations

import argparse
import functools
import hashlib
import math
import sys
import urllib.request
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Vec
from OCP.GProp import GProp_GProps
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.TopTools import TopTools_ListOfShape
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import skeleton as sk  # noqa: E402
from skeleton import Box, Mount  # noqa: E402

ROOT = HERE.parents[3]
OUT = HERE / "release" / "electrical"
MESHES = HERE / "meshes" / "electrical"
VENDOR_CACHE = ROOT / ".tmp" / "vendor_cad" / "spark_rover"
POLOLU = "https://www.pololu.com"
ADA = "https://www.adafruit.com/product"


@dataclass(frozen=True)
class Params:
    """The few dimensions of this design that are choices, not vendor figures."""
    carrier_t: float = 3.0          # printed PETG carrier plates
    pi_standoff: float = 12.0       # M2.5 F-F brass standoffs, spine plate -> Pi: 12 mm (chassis.PI5_STANDOFF): the
                                    # chassis's M2.5 x 16 takes 3.75 mm, the Pi's M2.5 x 6 4.4 mm, 0.5 mm between
    board_standoff: float = 6.0     # printed bosses with M2.5 heat-set inserts, plate -> IO board
    wing_standoff: float = 2.0      # bosses on the bay's side wings


def _p(r) -> Params:
    return r if isinstance(r, Params) else Params()


# =========================================================================================
# 1. The parts' figures, each from the vendor (read 2026-11-03)
# =========================================================================================

CELL = {    # Molicel INR-18650-P28A product data sheet (copy: rosebatteries.com/hubfs/INR18650P28A-V2-80093.pdf)
    "c_typ_mah": 2800.0, "c_min_mah": 2700.0, "e_typ_wh": 10.3, "v_nom": 3.6, "v_charge": 4.2, "v_cut": 2.5,
    "i_charge_std": 2.8, "i_charge_max": 8.4, "i_dis_max": 35.0, "r_dc": 0.020, "r_ac_max": 0.020,
    "d_max": 18.6, "h_max": 65.2, "g": 46.0, "t_charge_c": (0.0, 60.0),
    "source": "https://www.rosebatteries.com/hubfs/INR18650P28A-V2-80093.pdf (Molicel P28A data sheet)",
}
PACK = {    # our 2S2P pack: four P28A, nickel strip, a 2S protection board, shrink, XT30U pigtail
    "series": 2, "parallel": 2, "v_full": 8.4, "v_end": 6.4,   # 6.4 V = the firmware's 3.2 V/cell cut-off
    # four cells flat, side by side along x (4 x 18.6 = 74.4 + shrink), axes along y (65.2 + nickel + the
    # protection board on the -y end + shrink), one cell high: the owner's 2S2P (runtime over mass)
    "outline_mm": (75.0, 72.0, 20.0), "g": 4 * 46.0 + 8.0 + 8.0,    # cells + BMS + strip/shrink/leads (ASSUMED)
    "usable_frac": 0.95,          # ASSUMED: energy above 3.2 V/cell of the data sheet's 10.3 Wh (to 2.5 V)
    "leads_exit": (-30.0, -36.0, 45.0),   # the P+/P- leads leave the -y end (the BMS end) at x -30
    "note": "2S2P; leads (#18, 100 mm) from the -y end along the top face to the deck's hole at (-35, 0)",
}
# The one nominal voltage of the pack: the cell data sheet's 3.6 V nominal x 2 in series = 7.2 V. (chassis.py's
# Chassis.battery_v agrees since 2026-11-04; it said 7.4 V, the generic 2S figure.)
PACK["v_nom"] = PACK["series"] * CELL["v_nom"]
PACK["r"] = PACK["series"] * CELL["r_dc"] / PACK["parallel"]          # 20 mohm: two pairs of 20 mohm cells in parallel
PACK["c_mah"] = PACK["parallel"] * CELL["c_typ_mah"]
PACK["e_wh"] = PACK["series"] * PACK["parallel"] * CELL["e_typ_wh"]   # 4 x 10.3 Wh typical
BMS = {     # ASSUMED: a generic 2S Li-ion protection board (no vendor data sheet found); the numbers are its
    # typical class and must be checked against the board bought
    "i_cont": 25.0, "ocp_a": 50.0, "ovp_cell": 4.25, "uvp_cell": 2.8, "r": 0.010, "g": 8.0,
    "spec": "2S 25 A Li-ion protection board with balance (HY2120-class IC), for the 18 A all-stalled case",
    "source": "ASSUMED (generic)",
}
PACK_VOLUME = {"box": Box(-37.5, 37.5, -36.0, 36.0, 29.5, 49.5), "leads_hole": (-35.0, 0.0, 56.0),
               "source": "the 2S2P outline, centred under the deck (the chassis widens BATTERY_HOLDER to it); "
                         "leads up the deck's 10 mm hole"}

CHARGER = {  # Injoinic IP2326 (LCSC C2832094 data sheet): 5 V-input synchronous boost charger for 2S/3S
    "vin": 5.0, "p_max_w": 15.0, "v_cv": 8.4, "eta": 0.94, "f_sw_khz": 500.0,
    "i_charge": 1.5,              # ASSUMED module setting: 8.4 V x 1.5 A / 94 % = 13.4 W, inside the 15 W a 5 V 3 A
                                  # USB-C port gives (a "2 A" IP2326 module cannot hold 2 A at 8.4 V: 17.9 W)
    "board_mm": (22.0, 18.0, 1.6),    # ASSUMED generic module outline
    "spec": "IP2326 2S USB-C boost charger module (5 V in, 8.4 V CC/CV)",
    "source": "https://datasheet.lcsc.com/lcsc/2304062030_INJOINIC-IP2326_C2832094.pdf",
    "note": "not USB PD: it takes 5 V at up to 3 A, which every USB-C charger (PD ones too) supplies",
}
FUSE = {    # Littelfuse MINI 297 series, 0297015: 32 VDC, interrupting 1000 A @ 32 VDC, 10.9 x 3.8 x 8.8 mm
    "part": "0297015.WXNV", "i_rated": 15.0, "v_max": 32.0, "i_interrupt": 1000.0,
    "r": 0.0046,                  # ASSUMED: scaled from the 10 A part's 7.42 mohm (Future Electronics listing)
    "holder": "inline MINI blade fuse holder, 16 AWG leads", "holder_mm": (34.0, 14.0, 12.0), "g": 8.0,
    "source": "https://www.littelfuse.com/products/fuses-overcurrent-protection/fuses/automotive-fuses/blade-fuses-shunt/mini/297",
}
SWITCH = {  # Pololu #2813 Big Pushbutton Power Switch with Reverse Voltage Protection, HP
    "v_min": 4.5, "v_max": 32.0, "i_55c": 6.0, "i_150c": 16.0, "i_max": 90.0, "rds": 0.0086, "i_on_per_v": 120e-6,
    "ctrl_v": 1.0, "size_mm": (25.4, 20.3, 4.1), "g": 2.7, "source": f"{POLOLU}/product/2813",
    "note": "the continuous figures are the currents at which the board reaches 55 C / 150 C (22 C, still air)",
}
REG5 = {    # Pololu #5571 5V, 6A Step-Down Voltage Regulator D42V55F5
    "vin_min": 5.0, "vin_max": 60.0, "vout": 5.0, "tol": 0.03, "i_max": 6.0, "eta": 0.85,   # 80-95 %: low end
    "iq": 0.4e-3, "dropout_ohm": 0.08,   # ASSUMED: dropout is only a graph on Pololu's page (~0.4 V at 5 A)
    "size_mm": (25.4, 25.4, 9.0), "g": 7.0, "source": f"{POLOLU}/product/5571",
    "note": "reverse protection, current limit, thermal shutdown, soft start; i_max is at 42 V in (higher at 7 V)",
}
REG6 = {    # Pololu #3782 5V, 3.2A Step-Down Voltage Regulator D36V28F5: the servo rail
    "vin_min": 5.3, "vin_max": 50.0, "vout": 5.0, "tol": 0.04, "eta": 0.85,          # 80-90 %: the low end (rounded)
    "i_max": 2.5,                 # the low end of Pololu's "2.5 A to 4 A" typical maximum: the continuous rating used
    "i_fault": 4.0,               # the high end: what a fault can draw before its over-current protection limits it
    "iq": 3e-3, "dropout_ohm": 0.15,   # dropout ASSUMED (graph only)
    "size_mm": (20.3, 17.8, 8.8), "g": 3.3, "source": f"{POLOLU}/product/3782",
    "note": "over-current and short-circuit protection, thermal shutoff, reverse protection; 5 V +-4 %",
}
MOTORON = {  # Pololu #5065 Motoron M2T256 Dual I2C Motor Controller; user's guide 0J84 section 4.1 for the pins
    "vin_min": 4.5, "vin_max": 48.0, "i_cont": 1.8, "i_peak": 6.4, "vdd": (3.0, 5.5), "pwm_khz_max": 80.0,
    "size_mm": (15.24, 20.32, 3.0), "g": 1.5, "addr": (16, 17), "i_logic": 0.010, "i_vin_q": 0.005,   # ASSUMED currents
    "r_bridge": 0.10,             # ASSUMED on-resistance high + low side (not published)
    "source": f"{POLOLU}/product/5065/specs; {POLOLU}/docs/0J84",
    "note": "command timeout stops the motors; motor supply voltage is readable over I2C; no current sense on M2T",
}
PI5 = {     # Raspberry Pi 5: product brief, mechanical drawing, documentation (power supply table)
    "board_mm": (85.0, 56.0, 1.6), "holes": (58.0, 49.0), "hole_inset": 3.5, "hole_d": 2.7,
    "header_u0": 32.5,            # the 40-pin header centred 29 mm from the mounting hole (drawing), 3.5 mm in
    "i_psu": 5.0, "i_usb_max": 1.6, "i_usb_3a": 0.6, "i_typ": 0.8,
    "i_busy": 1.6,                # ASSUMED: all four cores plus camera pipeline plus fan (no vendor figure)
    "g": 46.0,                    # ASSUMED
    "v_low_warn": 4.63,           # ASSUMED (the PMIC's under-voltage warning, community-reported)
    "sources": ["https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-product-brief.pdf",
                "https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf",
                "https://www.raspberrypi.com/documentation/computers/raspberry-pi.html (power supply)"],
    "note": "powered on the GPIO 5V pins; EEPROM PSU_MAX_CURRENT=5000 (skip PD negotiation) for the 1.6 A USB limit",
}
COOLER = {"size_mm": (63.5, 42.5, 13.7), "cfm": 1.09, "rpm": 8000, "g": 20.0,     # g ASSUMED
          "source": "https://pip-assets.raspberrypi.com/categories/993-raspberry-pi-active-cooler/documents/"
                    "RP-008188-DS-2-raspberry-pi-active-cooler-product-brief.pdf"}
PICO = {    # Raspberry Pi Pico datasheet (Rev3)
    "size_mm": (51.0, 21.0, 1.0), "vsys": (1.8, 5.5), "i_3v3_max": 0.300, "hole_d": 2.1, "io_v": 3.3,
    "i_typ": 0.025, "g": 3.0,     # current ASSUMED (busy PIO + UARTs), mass ASSUMED
    "source": "https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf",
    "note": "VBUS -> Schottky -> VSYS on board: an external supply on VSYS through a second Schottky ORs with USB",
}
LOGIC_IC = {
    "SN74LVC245A": {"vcc": (1.65, 3.6), "vi_max": 5.5, "source": "https://www.ti.com/product/SN74LVC245A"},
    "SN74LVC125A": {"vcc": (1.65, 3.6), "vi_max": 5.5, "vo_off_max": 5.5, "source": "https://www.ti.com/product/SN74LVC125A"},
    "SN74AHCT125": {"vcc": (4.5, 5.5), "vih": 2.0, "voh_min": 4.4, "source": f"{ADA}/1787; https://www.ti.com/product/SN74AHCT125"},
}
DISPLAY = {  # Waveshare 50H-800480-IPS: 5" DSI, 800 x 480, thin (driver on the FPC, "only needs 5mm thickness")
    "active_mm": (108.0, 64.8), "outline_mm": (120.70, 75.80, 3.0), "p_w": 1.5, "g": 55.0,  # t, p_w, g ASSUMED
    "cable": "Raspberry Pi display cable for Pi 5 (22-pin to 15-pin), 300 mm (ASSUMED length on sale)", "cable_mm": 300.0,
    "source": "https://waveshare.com/50H-800480-IPS.htm",
    "note": "Pi 5 on DSI1 with a 22-to-15 pin cable, as Waveshare's 43H wiki says for the same family (ASSUMED for 50H)",
}
CAMERA_MOD = {  # Raspberry Pi Camera Module 3 (standard): product brief + vendor STEP (measured)
    "board_mm": (25.0, 23.862), "depth_mm": 11.5, "lens_from_far_edge": 14.4, "lens_from_fpc_edge": 9.46,
    "fov_deg": (66.0, 41.0), "p_w": 0.5, "g": 4.0,      # power and mass ASSUMED
    "cable": "Raspberry Pi Camera Cable for Pi 5 (22-pin to 15-pin), 200 mm", "cable_mm": 200.0,
    "source": "https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf",
}
MIC = {"part": "SPH0645LM4H", "v": (1.6, 3.6), "size_mm": (16.7, 12.7, 1.8), "g": 0.4, "i": 0.0006,   # i ASSUMED
       "source": f"{ADA}/3421", "note": "bottom-ported: the board is mounted port side up, under the shell's hole"}
AMP = {"part": "MAX98357A", "v": (2.7, 5.5), "p_out_4ohm": 3.2, "gain_db": 9, "size_mm": (19.4, 17.8, 3.0), "g": 1.2,
       "eta": 0.85, "iq": 0.003, "source": f"{ADA}/3006"}                                    # eta, iq ASSUMED
SPEAKER = {"ohm": 4.0, "w": 3.0, "size_mm": (70.0, 30.0, 17.0), "g": 26.4, "holes": (63.0, 24.0), "hole_d": 3.4,
           "source": f"{ADA}/3351", "note": "Mono Enclosed Speaker 3 W 4 ohm, JST PH lead (cut to length)"}
TOUCH = {"part": "MPR121", "addr": 0x5A, "vin": (3.0, 5.5), "size_mm": (33.0, 25.4, 4.0), "g": 2.0, "i": 0.0003,
         "source": f"{ADA}/1982", "note": "outline, mass and current ASSUMED; pads: copper foil tape (Adafruit's advice)"}
STRIP = {"part": "Adafruit NeoPixel 60 LED/m white (WS2812B or SK6812)", "pitch": 16.67, "width": 12.5, "t": 4.0,
         "v_max": 6.0, "i_led": 0.060, "g_per_m": 39.0, "source": f"{ADA}/1138"}
LED_FRONT, LED_REAR = 6, 6       # 100 mm each, behind the bumper's 112 mm window (shell.py led_window_half 56)
LED_CHANNEL_X = 87.2              # the bumper's channel, x 87.2-91.3, z 23.4-36.6 (shell.bumper_channel): slid in from its end
LED_CAP = 0.33          # the firmware's brightness cap (full white would be 60 mA a pixel)
STATUS_LED = {"d": 3.0, "l": 5.3, "vf": 2.0, "i": 0.002, "r_ohm": 680.0, "source": "generic 3 mm LED (ASSUMED Vf)"}
BUTTON = {"d": 9.8, "h": 8.0, "sw": (12.0, 0.050), "i_led": 0.010, "g": 0.4, "source": f"{ADA}/3104",
          "note": "illuminated tactile button; its LED has no resistor (Adafruit: ~1 k)"}
WAGO = {"221-413": {"slots": 3, "mm": (18.7, 18.3, 8.1)}, "221-415": {"slots": 5, "mm": (29.8, 18.3, 8.1)},
        "i": 32.0, "awg": (24, 12), "r": 0.001, "g": 4.0,
        "source": "https://www.wago.com (221 series; outlines and contact resistance ASSUMED, not re-read)"}
CONN = {    # connector ratings used by the checks (ASSUMED where marked)
    "XT30": {"i": 15.0, "r": 0.001, "source": "Amass XT30U (15 A continuous: ASSUMED, catalogue figure)"},
    "MINIFIT": {"i": 9.0, "source": "Molex Mini-Fit Jr 5557/5559 wire-to-wire (9 A: ASSUMED)"},
    "XH": {"i": 3.0, "source": "JST XH 2.5 mm (3 A: ASSUMED from JST's eXH.pdf)"},
    "DUPONT": {"i": 3.0, "source": "2.54 mm crimp on the Pi's header (3 A: ASSUMED)"},
}
# Powerstream wire table: ohm/km and its "chassis wiring" amps (a rule of thumb, it says)
AWG = {18: (20.95, 16.0), 20: (33.31, 11.0), 22: (52.94, 7.0), 24: (84.20, 3.5), 26: (133.86, 2.2), 28: (212.87, 1.4)}
AWG_SOURCE = "https://www.powerstream.com/Wire_Size.htm"
AWG_D = {18: 1.024, 20: 0.812, 22: 0.644, 24: 0.511, 26: 0.405, 28: 0.321}

# ---- the figures other subsystems own: ONE constant each, updated when they report ---------
MOTOR = {   # CHASSIS: Pololu #4844 25D HP 12 V 34:1 with 48 CPR encoder (chassis.py drive(), 2026-11-04)
    "r_ohm": 2.4,                 # 2.5 A stall at 6.0 V, 3.0 A at 7.2 V, 3.5 A at 8.4 V: 2.4 ohm
    # per motor at the pack's 7.2 V nominal and the skeleton's 2.4 kg (chassis.drive()['currents_per_motor_a'];
    # at the 7.4 V chassis.py used before 2026-11-04: 0.316 / 0.455 / 0.818). Until 2026-11-04 these were
    # the 2.0 kg figures (0.31 / 0.43 / 0.73).
    "i_cruise": 0.316, "i_ramp": 0.454, "i_spin": 0.816, "count": 4,
    # Measured: a Newton run of robot.py's drive test (release/drive/drive_mu1.json) gave 0.832 A on the worst
    # motor (rr) in a spin turn at mu 1.0 and 0.645 A on average; the rules check the worst of the two figures.
    "i_spin_measured_worst": 0.832, "i_spin_measured_mean": 0.645,
    "source": "chassis.py BOUGHT['gearmotor'] and drive(); Pololu #4844; release/drive/drive_mu1.json (Newton)",
}
ENCODER = {"v": (3.5, 20.0), "v_supply": 5.0, "i": 0.010, "cpr_motor": 48, "rpm_motor_max": 10200.0,  # i ASSUMED
           "source": "Pololu 25D 48 CPR encoder (via the chassis agent)"}
SERVO = {   # ARMS: ROBOTIS DYNAMIXEL XC330-M288-T at all four joints (the arms agent, 2026-11-03)
    "model": "ROBOTIS DYNAMIXEL XC330-M288-T", "v_rec": 5.0, "v_abs": (3.7, 6.0), "i_stall": 1.80,   # 1.80 A at 5.0 V
    "i_standby": 0.017, "i_move": 0.20, "i_hold": 0.10, "count": 4, "g": 23.0,
    "logic": "TTL half-duplex multidrop, 3.3 V logic (5 V compatible), Protocol 2.0", "baud": (9600, 4_000_000),
    "connector": "JST EHR-03 (pins 1 GND, 2 VDD, 3 DATA), B3B-EH-A header, SEH-001T-P0.6 crimps, 21 AWG recommended",
    "source": "https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/",
    "note": "i_move is the arms agent's current limit (0.15-0.2 A); i_hold ASSUMED; daisy-chained shoulder -> elbow",
}
SERVO_PLUG = {"i": 3.0, "spec": "JST EHR-03 housing on the shoulder servo's B3B-EH-A (3 A: ASSUMED from JST's eEH)"}
SHOULDER_PLUG_AT = {"JS1": (sk.SHOULDERS["left"][0], sk.SHOULDERS["left"][1], sk.SHOULDERS["left"][2]),
                    "JS2": (sk.SHOULDERS["right"][0], sk.SHOULDERS["right"][1], sk.SHOULDERS["right"][2])}

# The vendors' CAD (fetched by `python electrical.py fetch`, verified, never committed). The
# Camera Module 3's lens and connector were measured on its STEP (lens 14.4 mm from the far
# edge, 9.46 mm from the FPC edge, the FPC connector on the back).
VENDOR_CAD = {
    "pi5": {"url": "https://datasheets.raspberrypi.com/rpi5/RaspberryPi5-step.zip",
            "sha256": "6841637b4cfa97637bf34b529bfe896f24d9634b2c40b57a156781f55d88f06f", "member": "rpi-5b_no_graphics.step"},
    "pico": {"url": "https://datasheets.raspberrypi.com/pico/Pico-R3-step.zip",
             "sha256": "4008e557bd80d75119872d10460d9f30f2e0faf210606d058bbe78cd6598c931", "member": "Pico-R3.step"},
    "camera3": {"url": "https://datasheets.raspberrypi.com/camera/camera-module-3-step.zip",
                "sha256": "b2e2026ea5322bddf1ddcbae97c094822ee9e0ca92d7bc5c748f60a8b5ef6683",
                "member": "Camera_module_3_std_model_simple.stp"},
}


# =========================================================================================
# 2. Components, with every pin
# =========================================================================================

PI_HEADER = {1: "3V3.1", 2: "5V.2", 3: "GPIO2", 4: "5V.4", 5: "GPIO3", 6: "GND.6", 7: "GPIO4", 8: "GPIO14", 9: "GND.9",
             10: "GPIO15", 11: "GPIO17", 12: "GPIO18", 13: "GPIO27", 14: "GND.14", 15: "GPIO22", 16: "GPIO23",
             17: "3V3.17", 18: "GPIO24", 19: "GPIO10", 20: "GND.20", 21: "GPIO9", 22: "GPIO25", 23: "GPIO11",
             24: "GPIO8", 25: "GND.25", 26: "GPIO7", 27: "ID_SD", 28: "ID_SC", 29: "GPIO5", 30: "GND.30", 31: "GPIO6",
             32: "GPIO12", 33: "GPIO13", 34: "GND.34", 35: "GPIO19", 36: "GPIO16", 37: "GPIO26", 38: "GPIO20",
             39: "GND.39", 40: "GPIO21"}
PICO_PINOUT = {1: "GP0", 2: "GP1", 3: "GND.3", 4: "GP2", 5: "GP3", 6: "GP4", 7: "GP5", 8: "GND.8", 9: "GP6", 10: "GP7",
               11: "GP8", 12: "GP9", 13: "GND.13", 14: "GP10", 15: "GP11", 16: "GP12", 17: "GP13", 18: "GND.18",
               19: "GP14", 20: "GP15", 21: "GP16", 22: "GP17", 23: "GND.23", 24: "GP18", 25: "GP19", 26: "GP20",
               27: "GP21", 28: "GND.28", 29: "GP22", 30: "RUN", 31: "GP26", 32: "GP27", 33: "AGND", 34: "GP28",
               35: "ADC_VREF", 36: "3V3", 37: "3V3_EN", 38: "GND.38", 39: "VSYS", 40: "VBUS"}
DIP20_245 = {1: "DIR", 2: "A1", 3: "A2", 4: "A3", 5: "A4", 6: "A5", 7: "A6", 8: "A7", 9: "A8", 10: "GND", 11: "B8",
             12: "B7", 13: "B6", 14: "B5", 15: "B4", 16: "B3", 17: "B2", 18: "B1", 19: "OE", 20: "VCC"}
DIP14_125 = {1: "1OE", 2: "1A", 3: "1Y", 4: "2OE", 5: "2A", 6: "2Y", 7: "GND", 8: "3Y", 9: "3A", 10: "3OE", 11: "4Y",
             12: "4A", 13: "4OE", 14: "VCC"}
MOTORS = {"M1": "front_left", "M2": "front_right", "M3": "rear_left", "M4": "rear_right"}
REG_PINS = ["VIN", "VRP", "GND", "VOUT", "EN", "PG"]
MOTORON_PINS = ["VIN", "GND", "M1A", "M1B", "M2A", "M2B", "VM", "VDD", "GND2", "SCL", "SDA", "ERR", "RST", "JMP1"]


def components() -> Dict[str, dict]:
    """ref -> {part, pins, source, where}. Every pin is listed; unused ones are no-connects."""
    c: Dict[str, dict] = {}

    def add(ref, part, pins, source="-", where="bay", **kw):
        c[ref] = {"part": part, "pins": list(pins), "source": source, "where": where, **kw}

    add("BT1", "2S2P pack: 4 x Molicel INR-18650-P28A + 2S 25 A protection board, XT30U-F pigtail", ["P+", "P-"],
        CELL["source"], "chassis battery volume (under the deck)")
    add("J1", "Amass XT30U pair (pack disconnect)", ["1", "2"], CONN["XT30"]["source"], "at the pack, under the deck")
    add("F1", f"Littelfuse {FUSE['part']} 15 A MINI blade fuse in an inline holder", ["1", "2"], FUSE["source"])
    add("SW1", "Pololu #2813 Big Pushbutton Power Switch, Reverse Voltage Protection, HP",
        ["VIN", "GND", "VOUT", "A", "B", "ON", "OFF", "CTRL"], SWITCH["source"])
    add("S1", "Adafruit #3104 illuminated momentary pushbutton (the power button)", ["1", "2", "LED+", "LED-"], BUTTON["source"])
    add("CHG1", CHARGER["spec"], ["BAT+", "BAT-", "USBC"], CHARGER["source"])
    add("X1", "WAGO 221-413 lever connector (VBAT hub)", ["1", "2", "3"], WAGO["source"])
    add("X2", "WAGO 221-415 lever connector (GND hub)", ["1", "2", "3", "4", "5"], WAGO["source"])
    add("X3", "WAGO 221-415 lever connector (VSW hub)", ["1", "2", "3", "4", "5"], WAGO["source"])
    add("X4", "WAGO 221-415 lever connector (GND, motor side)", ["1", "2", "3", "4", "5"], WAGO["source"])
    add("C2", "470 uF 16 V low-ESR electrolytic, radial 10 x 16 (bulk on VSW)", ["+", "-"])
    add("U5", "Pololu #5571 D42V55F5 5 V 6 A step-down regulator", REG_PINS, REG5["source"])
    add("U6", "Pololu #3782 D36V28F5 5 V 3.2 A step-down regulator (servo rail)", REG_PINS, REG6["source"])
    add("MA", "Pololu #5065 Motoron M2T256 dual I2C motor controller, LEFT (M1 front, M3 rear), address 16",
        MOTORON_PINS, MOTORON["source"])
    add("MB", "Pololu #5065 Motoron M2T256 dual I2C motor controller, RIGHT (M2 front, M4 rear), address 17",
        MOTORON_PINS, MOTORON["source"])
    pi = list(PI_HEADER.values()) + ["CAMDISP0", "CAMDISP1", "FAN", "USBC_PWR"]
    add("U1", "Raspberry Pi 5", pi, PI5["sources"][0])
    add("U10", "Raspberry Pi Active Cooler", ["FAN"], COOLER["source"])
    add("B1", "IO board: 2.54 mm FR4 perfboard cut to 90 x 44 mm (rails and hub pads)",
        ["5V", "3V3", "GND", "H_BCLK", "H_LRCK"])
    add("U2", "Raspberry Pi Pico (RP2040), soldered by its castellations onto B1",
        list(PICO_PINOUT.values()) + ["SWCLK", "SWDIO", "SWGND", "USB"], PICO["source"])
    add("U3", "TI SN74LVC245AN (DIP-20, in a socket): encoder level shift 5 V -> 3.3 V", list(DIP20_245.values()),
        LOGIC_IC["SN74LVC245A"]["source"])
    add("U4", "TI SN74LVC125AN (DIP-14, socket): half-duplex servo bus buffer", list(DIP14_125.values()),
        LOGIC_IC["SN74LVC125A"]["source"])
    add("U7", "TI SN74AHCT125N (DIP-14, socket): LED data 3.3 V -> 5 V", list(DIP14_125.values()),
        LOGIC_IC["SN74AHCT125"]["source"])
    for ref, what in (("C4", "U3"), ("C5", "U4"), ("C6", "U7")):
        add(ref, f"100 nF X7R ceramic, 2.54 mm (decoupling {what})", ["1", "2"])
    for ref, what in (("R3", "4.7 kohm I2C0 SDA pull-up"), ("R4", "4.7 kohm I2C0 SCL pull-up"),
                      ("R5", "10 kohm servo bus pull-up to 3.3 V"), ("R6", "100 ohm servo bus series"),
                      ("R7", "330 ohm LED data series"), ("R11", "10 kohm: holds the bus buffer off through reset")):
        add(ref, f"{what}, 0.25 W axial", ["1", "2"])
    add("D5", "1N5817 Schottky (B1 5V -> Pico VSYS)", ["A", "K"], "https://www.vishay.com/docs/88525/1n5817.pdf")
    add("F2", "Bourns MF-R050 PTC, 0.5 A hold (accessory 5 V)", ["1", "2"], "https://www.bourns.com/docs/product-datasheets/mf-r.pdf")
    add("F3", "Bourns MF-R110 PTC, 1.1 A hold (LED ring 5 V)", ["1", "2"], "https://www.bourns.com/docs/product-datasheets/mf-r.pdf")
    for k in range(1, 5):
        add(f"JE{k}", f"JST B4B-XH-A header on B1 (encoder {MOTORS[f'M{k}']})", ["GND", "VCC", "A", "B"], CONN["XH"]["source"])
        add(f"JM{k}", f"Molex Mini-Fit Jr 2-circuit wire-to-wire pair (motor {MOTORS[f'M{k}']})", ["1", "2"],
            CONN["MINIFIT"]["source"])
        add(f"M{k}", f"Pololu #4844 gearmotor with encoder, {MOTORS[f'M{k}']} (the chassis's part)",
            ["M+", "M-", "GND", "VCC", "A", "B"], MOTOR["source"], "chassis")
    add("JS1", "DYNAMIXEL bus plug (JST EHR-03) on the LEFT shoulder servo", ["GND", "V+", "DATA"], SERVO_PLUG["spec"], "arms handoff")
    add("JS2", "DYNAMIXEL bus plug (JST EHR-03) on the RIGHT shoulder servo", ["GND", "V+", "DATA"], SERVO_PLUG["spec"], "arms handoff")
    add("U8", "Waveshare 50H-800480-IPS 5 in DSI panel, 800 x 480", ["DSI"], DISPLAY["source"], "head")
    add("U9", "Raspberry Pi Camera Module 3 (standard lens)", ["CSI"], CAMERA_MOD["source"], "head")
    add("MK1", "Adafruit #3421 SPH0645 I2S MEMS mic (LEFT, SEL low)", ["3V", "GND", "BCLK", "DOUT", "LRCL", "SEL"], MIC["source"], "head")
    add("MK2", "Adafruit #3421 SPH0645 I2S MEMS mic (RIGHT, SEL high)", ["3V", "GND", "BCLK", "DOUT", "LRCL", "SEL"], MIC["source"], "head")
    add("D3", "3 mm red LED (camera on)", ["A", "K"], STATUS_LED["source"], "head")
    add("D4", "3 mm green LED (status)", ["A", "K"], STATUS_LED["source"], "head")
    add("R8", "680 ohm 0.25 W axial (D3), inline, sleeved", ["1", "2"], "-", "head")
    add("R9", "680 ohm 0.25 W axial (D4), inline, sleeved", ["1", "2"], "-", "head")
    add("U11", "Adafruit #3006 MAX98357A I2S 3 W class-D amp", ["LRC", "BCLK", "DIN", "GAIN", "SD", "GND", "Vin", "SPK+", "SPK-"],
        AMP["source"], "chest")
    add("LS1", "Adafruit #3351 mono enclosed speaker 3 W 4 ohm", ["+", "-"], SPEAKER["source"], "chest")
    add("U12", "Adafruit #1982 MPR121 12-key capacitive touch breakout",
        ["VIN", "3Vo", "GND", "SCL", "SDA", "IRQ", "ADDR"] + [str(i) for i in range(12)], TOUCH["source"], "chest")
    for k in range(1, 4):
        add(f"TP{k}", f"touch pad {k}: {TOUCH_PADS[k][1] - TOUCH_PADS[k][0]:g} x 30 mm copper foil tape inside the chest "
            f"window ({TOUCH_PAD_HALF[k]})", ["1"], TOUCH["source"], "shell (inside)")
    add("D1", f"WS2812B strip, {LED_FRONT} pixels, FRONT (behind the bumper's diffuser)", ["5V", "DIN", "GND", "DOUT"],
        STRIP["source"], "bumper line")
    add("D2", f"WS2812B strip, {LED_REAR} pixels, REAR", ["5V", "DIN", "GND", "DOUT"], STRIP["source"], "bumper line")
    add("C3", "100 uF 10 V electrolytic at the strip input", ["+", "-"], STRIP["source"], "bumper line")
    add("R10", "1 kohm 0.25 W axial (button LED), on B2", ["1", "2"])
    add("B2", "button board: 14 x 14 mm perfboard behind the power button", [])
    add("B3", "top-slot board: 20 x 14 mm perfboard in the accessory socket", [], "-", "top slot")
    add("J_TOP", "2 x 4 female header, 2.54 mm (the accessory socket)", ["5V", "GND", "3V3", "GND2", "SDA", "SCL", "TX", "RX"],
        "-", "top slot")
    return c


# =========================================================================================
# 3. Pin assignment
# =========================================================================================

PI_USE = {
    "GPIO2": "I2C1 SDA (MPR121, accessory)", "GPIO3": "I2C1 SCL", "GPIO4": "uart2 TX (accessory)",
    "GPIO5": "uart2 RX (accessory)", "GPIO14": "uart0 TX -> Pico GP1", "GPIO15": "uart0 RX <- Pico GP0",
    "GPIO17": "MPR121 IRQ (input, pull-up)", "GPIO18": "PCM_CLK (mics + amp BCLK)", "GPIO19": "PCM_FS (LRCLK)",
    "GPIO20": "PCM_DIN (mics)", "GPIO21": "PCM_DOUT (amp)", "GPIO22": "red LED (camera on)", "GPIO23": "green LED",
    "GPIO24": "Pico SWCLK (openocd, linuxgpiod)", "GPIO25": "Pico SWDIO", "GPIO26": "power off (gpio-poweroff -> SW1 OFF)",
    "GPIO27": "Pico RUN (reset)",
}
PICO_USE = {
    "GP0": "UART0 TX -> Pi", "GP1": "UART0 RX <- Pi", "GP3": "servo bus direction (low = transmit)",
    "GP4": "UART1 TX -> servo bus", "GP5": "UART1 RX <- servo bus", "GP6": "encoder FL A (PIO0 SM0)",
    "GP7": "encoder FL B", "GP8": "encoder FR A (PIO0 SM1)", "GP9": "encoder FR B", "GP10": "encoder RL A (PIO0 SM2)",
    "GP11": "encoder RL B", "GP12": "encoder RR A (PIO0 SM3)", "GP13": "encoder RR B", "GP14": "WS2812 data (PIO1 SM0)",
    "GP15": "Motoron A ERR", "GP16": "Motoron B ERR", "GP20": "I2C0 SDA (Motorons)", "GP21": "I2C0 SCL",
}
PICO_FUNCS = {"UART0_TX": {0, 12, 16}, "UART0_RX": {1, 13, 17}, "UART1_TX": {4, 8}, "UART1_RX": {5, 9},
              "I2C0_SDA": {0, 4, 8, 12, 16, 20}, "I2C0_SCL": {1, 5, 9, 13, 17, 21}}   # Pico datasheet pinout
PI_FUNCS = {"I2C1": ("GPIO2", "GPIO3"), "uart0": ("GPIO14", "GPIO15"), "uart2": ("GPIO4", "GPIO5"),
            "PCM": ("GPIO18", "GPIO19", "GPIO20", "GPIO21")}                         # Pi 5 GPIO alt functions


# =========================================================================================
# 4. The netlist
# =========================================================================================

def netlist() -> Dict[str, List[Tuple[str, str]]]:
    """net -> [(ref, pin)]. The right-hand motors are mounted mirrored: their leads are
    crossed (MxA -> M-), so 'forward' is the same command on both sides; check on the bench."""
    n: Dict[str, List[Tuple[str, str]]] = {}
    n["VBAT_RAW"] = [("BT1", "P+"), ("J1", "1"), ("F1", "1")]
    n["VBAT"] = [("F1", "2"), ("X1", "1"), ("X1", "2"), ("X1", "3"), ("SW1", "VIN"), ("CHG1", "BAT+")]
    n["VSW"] = [("SW1", "VOUT"), ("C2", "+")] + [("X3", str(k)) for k in range(1, 6)] + \
        [("U5", "VIN"), ("U6", "VIN"), ("MA", "VIN"), ("MB", "VIN")]
    n["5V"] = [("U5", "VOUT"), ("U1", "5V.2"), ("U1", "5V.4"), ("B1", "5V"), ("D5", "A"), ("U7", "VCC"),
               ("U7", "2OE"), ("U7", "3OE"), ("U7", "4OE"), ("C6", "1"), ("F2", "1"), ("F3", "1"), ("U11", "Vin"),
               ("R10", "1")] + [(f"JE{k}", "VCC") for k in range(1, 5)] + [(f"M{k}", "VCC") for k in range(1, 5)]
    n["SERVO_5V"] = [("U6", "VOUT"), ("JS1", "V+"), ("JS2", "V+")]
    n["PICO_3V3"] = [("U2", "3V3"), ("B1", "3V3"), ("U3", "VCC"), ("U3", "DIR"), ("U4", "VCC"), ("U4", "3OE"),
                     ("U4", "4OE"), ("C4", "1"), ("C5", "1"), ("R3", "1"), ("R4", "1"), ("R5", "1"), ("R11", "1"),
                     ("MA", "VDD"), ("MB", "VDD"), ("J_TOP", "3V3")]
    n["PI_3V3"] = [("U1", "3V3.1"), ("U1", "3V3.17"), ("U12", "VIN"), ("MK1", "3V"), ("MK2", "3V"), ("MK2", "SEL")]
    n["VSYS"] = [("D5", "K"), ("U2", "VSYS")]
    gnd = [("BT1", "P-"), ("J1", "2"), ("SW1", "GND"), ("C2", "-"), ("CHG1", "BAT-"), ("U5", "GND"), ("U6", "GND"),
           ("MA", "GND"), ("MA", "GND2"), ("MB", "GND"), ("MB", "GND2"), ("B1", "GND")]
    gnd += [("X2", str(k)) for k in range(1, 6)] + [("X4", str(k)) for k in range(1, 6)]
    gnd += [("U1", p) for p in PI_HEADER.values() if p.startswith("GND")]
    gnd += [("U2", p) for p in PICO_PINOUT.values() if p.startswith("GND")] + [("U2", "AGND"), ("U2", "SWGND")]
    gnd += [("U3", "GND"), ("U3", "OE"), ("U4", "GND"), ("U4", "2OE"), ("U4", "3A"), ("U4", "4A"), ("U7", "GND"),
            ("U7", "1OE"), ("U7", "2A"), ("U7", "3A"), ("U7", "4A"), ("C4", "2"), ("C5", "2"), ("C6", "2"), ("C3", "-")]
    gnd += [(f"JE{k}", "GND") for k in range(1, 5)] + [(f"M{k}", "GND") for k in range(1, 5)]
    gnd += [("D1", "GND"), ("D2", "GND"), ("MK1", "GND"), ("MK1", "SEL"), ("MK2", "GND"), ("U11", "GND"),
            ("U12", "GND"), ("D3", "K"), ("D4", "K"), ("S1", "LED-"), ("J_TOP", "GND"), ("J_TOP", "GND2"),
            ("JS1", "GND"), ("JS2", "GND")]
    n["GND"] = gnd
    for drv, ch, m, crossed in (("MA", 1, "M1", False), ("MA", 2, "M3", False), ("MB", 1, "M2", True), ("MB", 2, "M4", True)):
        j = "JM" + m[1]
        n[f"MOT_{m}_A"] = [(drv, f"M{ch}A"), (j, "1"), (m, "M-" if crossed else "M+")]
        n[f"MOT_{m}_B"] = [(drv, f"M{ch}B"), (j, "2"), (m, "M+" if crossed else "M-")]
    for k in range(1, 5):
        for c, ai in (("A", 2 * k - 1), ("B", 2 * k)):
            n[f"ENC{k}_{c}"] = [(f"M{k}", c), (f"JE{k}", c), ("U3", f"A{ai}")]
            n[f"ENC{k}_{c}_3V3"] = [("U3", f"B{ai}"), ("U2", f"GP{5 + ai}")]
    n["SERVO_TX"] = [("U2", "GP4"), ("U4", "1A")]
    n["SERVO_RX"] = [("U4", "2Y"), ("U2", "GP5")]
    n["SERVO_DIR"] = [("U2", "GP3"), ("U4", "1OE"), ("R11", "2")]
    n["SERVO_LOCAL"] = [("U4", "1Y"), ("U4", "2A"), ("R6", "1"), ("R5", "2")]
    n["SERVO_DATA"] = [("R6", "2"), ("JS1", "DATA"), ("JS2", "DATA")]
    n["I2C0_SDA"] = [("U2", "GP20"), ("R3", "2"), ("MA", "SDA"), ("MB", "SDA")]
    n["I2C0_SCL"] = [("U2", "GP21"), ("R4", "2"), ("MA", "SCL"), ("MB", "SCL")]
    n["MA_ERR"] = [("MA", "ERR"), ("U2", "GP15")]
    n["MB_ERR"] = [("MB", "ERR"), ("U2", "GP16")]
    n["LED_3V3"] = [("U2", "GP14"), ("U7", "1A")]
    n["LED_5V_DATA"] = [("U7", "1Y"), ("R7", "1")]
    n["LED_DIN"] = [("R7", "2"), ("D1", "DIN")]
    n["LED_LINK"] = [("D1", "DOUT"), ("D2", "DIN")]
    n["LED_5V"] = [("F3", "2"), ("D1", "5V"), ("D2", "5V"), ("C3", "+")]
    n["PI_TX"] = [("U1", "GPIO14"), ("U2", "GP1")]
    n["PICO_TX"] = [("U2", "GP0"), ("U1", "GPIO15")]
    n["SWCLK"] = [("U1", "GPIO24"), ("U2", "SWCLK")]
    n["SWDIO"] = [("U1", "GPIO25"), ("U2", "SWDIO")]
    n["PICO_RUN"] = [("U1", "GPIO27"), ("U2", "RUN")]
    n["I2C1_SDA"] = [("U1", "GPIO2"), ("U12", "SDA"), ("J_TOP", "SDA")]
    n["I2C1_SCL"] = [("U1", "GPIO3"), ("U12", "SCL"), ("J_TOP", "SCL")]
    n["TOP_TX"] = [("U1", "GPIO4"), ("J_TOP", "TX")]
    n["TOP_RX"] = [("U1", "GPIO5"), ("J_TOP", "RX")]
    n["TOP_5V"] = [("F2", "2"), ("J_TOP", "5V")]
    n["TOUCH_IRQ"] = [("U12", "IRQ"), ("U1", "GPIO17")]
    for i in range(3):
        n[f"TOUCH_E{i}"] = [("U12", str(i)), (f"TP{i + 1}", "1")]
    n["PCM_CLK"] = [("U1", "GPIO18"), ("B1", "H_BCLK"), ("U11", "BCLK"), ("MK1", "BCLK"), ("MK2", "BCLK")]
    n["PCM_FS"] = [("U1", "GPIO19"), ("B1", "H_LRCK"), ("U11", "LRC"), ("MK1", "LRCL"), ("MK2", "LRCL")]
    n["PCM_DIN"] = [("U1", "GPIO20"), ("MK1", "DOUT"), ("MK2", "DOUT")]
    n["PCM_DOUT"] = [("U1", "GPIO21"), ("U11", "DIN")]
    n["SPK_P"] = [("U11", "SPK+"), ("LS1", "+")]
    n["SPK_N"] = [("U11", "SPK-"), ("LS1", "-")]
    n["LED_R_DRV"] = [("U1", "GPIO22"), ("R8", "1")]
    n["LED_R"] = [("R8", "2"), ("D3", "A")]
    n["LED_G_DRV"] = [("U1", "GPIO23"), ("R9", "1")]
    n["LED_G"] = [("R9", "2"), ("D4", "A")]
    n["PWR_OFF"] = [("U1", "GPIO26"), ("SW1", "OFF")]
    n["BTN_A"] = [("S1", "1"), ("SW1", "A")]
    n["BTN_B"] = [("S1", "2"), ("SW1", "B")]
    n["BTN_LED"] = [("R10", "2"), ("S1", "LED+")]
    n["DSI"] = [("U1", "CAMDISP1"), ("U8", "DSI")]
    n["CSI"] = [("U1", "CAMDISP0"), ("U9", "CSI")]
    n["FAN"] = [("U1", "FAN"), ("U10", "FAN")]
    return n


NO_CONNECT: Dict[Tuple[str, str], str] = {
    ("U5", "EN"): "pulled up to VIN on the board (1 Mohm): always on with the switch",
    ("U5", "PG"): "power-good: unused", ("U5", "VRP"): "VIN after reverse protection: unused",
    ("U6", "EN"): "always on with the switch", ("U6", "PG"): "unused", ("U6", "VRP"): "unused",
    ("SW1", "ON"): "the pushbutton (A/B) turns it on", ("SW1", "CTRL"): "unused",
    ("MA", "VM"): "reverse-protected VIN output: unused", ("MA", "RST"): "unused (the Pico resets it over I2C)",
    ("MA", "JMP1"): "open: address 16 (default)", ("MB", "VM"): "unused", ("MB", "RST"): "unused",
    ("MB", "JMP1"): "open: address 17 is written to its EEPROM once (Pololu's address-setting example)",
    ("U1", "USBC_PWR"): "NOT used: the Pi is powered on its GPIO 5V pins; never plug a supply into it while the robot is on",
    ("U1", "ID_SD"): "HAT ID EEPROM bus: keep free", ("U1", "ID_SC"): "HAT ID EEPROM bus: keep free",
    ("U2", "VBUS"): "USB 5 V: only when a laptop is plugged in for service (ORs with VSYS by its own Schottky)",
    ("U2", "3V3_EN"): "pulled up on the Pico", ("U2", "ADC_VREF"): "unused", ("U2", "USB"): "service port, reachable through the hatch",
    ("U4", "3Y"): "unused gate (input tied low, output disabled)", ("U4", "4Y"): "unused gate",
    ("U7", "2Y"): "unused gate", ("U7", "3Y"): "unused gate", ("U7", "4Y"): "unused gate",
    ("U11", "GAIN"): "open: 9 dB", ("U11", "SD"): "open: on, (L+R)/2 mono",
    ("U12", "3Vo"): "regulator output: unused", ("U12", "ADDR"): "open: address 0x5A",
    ("D2", "DOUT"): "end of the chain", ("CHG1", "USBC"): "the robot's USB-C charging port (skeleton USB_C)",
}
for _g in ("GPIO6", "GPIO7", "GPIO8", "GPIO9", "GPIO10", "GPIO11", "GPIO12", "GPIO13", "GPIO16"):
    NO_CONNECT[("U1", _g)] = "free"
for _g in ("GP2", "GP17", "GP18", "GP19", "GP22", "GP26", "GP27", "GP28"):
    NO_CONNECT[("U2", _g)] = "free"
for _i in range(3, 12):
    NO_CONNECT[("U12", str(_i))] = "free electrode"

# Pins of one part joined inside it (board copper, a lever connector's busbar).
INTERNAL: List[Tuple[Tuple[str, str], ...]] = [
    tuple(("X1", str(k)) for k in range(1, 4)), tuple(("X2", str(k)) for k in range(1, 6)),
    tuple(("X3", str(k)) for k in range(1, 6)), tuple(("X4", str(k)) for k in range(1, 6)),
    tuple(("U1", p) for p in PI_HEADER.values() if p.startswith("GND")), (("U1", "5V.2"), ("U1", "5V.4")),
    (("U1", "3V3.1"), ("U1", "3V3.17")),
    tuple(("U2", p) for p in PICO_PINOUT.values() if p.startswith("GND")) + (("U2", "AGND"), ("U2", "SWGND")),
    (("MA", "GND"), ("MA", "GND2")), (("MB", "GND"), ("MB", "GND2")),
]


# =========================================================================================
# 5. Placement: every part in a local frame, its pins, its solid
# =========================================================================================

@dataclass(frozen=True)
class Place:
    """Local (u, v, n) -> robot: origin + u eu + v ev + n (eu x ev). A board's local box is
    u 0..size[0], v 0..size[1], n 0..size[2] (n away from what it is mounted on)."""
    origin: Tuple[float, float, float]
    eu: Tuple[float, float, float]
    ev: Tuple[float, float, float]
    zone: str

    @property
    def R(self) -> np.ndarray:
        eu, ev = np.asarray(self.eu, float), np.asarray(self.ev, float)
        return np.column_stack([eu, ev, np.cross(eu, ev)])

    def at(self, u, v, n) -> Tuple[float, float, float]:
        p = np.asarray(self.origin, float) + self.R @ np.array([u, v, n], float)
        return (float(p[0]), float(p[1]), float(p[2]))


# Directions: a board on the bay's back plate faces the hatch (-x); on the left wing faces -y;
# on the right wing +y; on the rear lip +x; in the head and chest +x.
_BACK = ((0.0, 1.0, 0.0), (0.0, 0.0, -1.0))      # Pi 5: header at the top, its ports toward +y
_BAYB = ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0))      # IO board on the back plate
_LW = ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))         # left wing (n = -y)
_RW = ((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))        # right wing (n = +y)
_LIP = ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0))        # rear lip (n = +x)
_FWD = ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0))        # head / chest (n = +x)
_UP = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))         # horizontal, n = +z

BACK_X = -15.0                                    # the spine's rear face
FRONT_X = 15.0                                    # the spine's front face
CH = sk.CAMERA["lens_at"]
FW = sk.FACE_WINDOW
# The mics' boards: their port face 0.5 mm under the shell's mic ducts (which end at the head zone's top, z 262),
# so the upper halves slide across them clear (until 2026-11-04 the faces met at z 262 and the duct dragged over
# the board as each half went on). A 1 mm closed-cell foam ring, compressed to the 0.5 mm, seals each port.
MIC_Z = 262.0 - 0.5 - 1.8                         # the board's underside, z 259.7 (MIC size_mm[2] = 1.8)
# The accessory board sits ON the top socket's floor (shell.py Shell.socket_floor_t = 4.0 above TOP_SLOT.z0):
# until 2026-11-04 it was drawn at z 273, inside that floor (2.5 mm into shell/top_socket). Its wires come up
# through the floor's header slot (shell.py header_slot 5.4 x 13) to its pads underneath.
SOCKET_FLOOR_Z = sk.TOP_SLOT.z0 + 4.0
# The speaker 1 mm up (z 86-116, centre 101 vs the grille's 100): its lower posts then hold an M3 insert's
# 4.0 mm hole with 2 mm of wall above the chest zone's floor (z 85).
LS1_Z = 86.0


def placements(r=None) -> Dict[str, Place]:
    p = _p(r)
    plate = BACK_X - p.carrier_t                  # the bay carrier's front face, x -18
    pi_n0 = plate - p.pi_standoff                 # the Pi's PCB underside, x -24
    io_n0 = plate - p.board_standoff
    wl, wr = 51.0 - p.wing_standoff, -51.0 + p.wing_standoff
    cam_x = CH[0] - 5.0 - 8.9                     # the camera PCB's front face: the lens top 5 mm behind the shell's face
    dw, dh, dt = DISPLAY["outline_mm"]
    fcx, fcz = FW["centre"][1], FW["centre"][2]
    return {
        # the Pi: holes on the chassis's pattern (y +-29, z 73.5 / 122.5), header at the top
        "U1": Place((pi_n0, -32.5, 126.0), *_BACK, "bay"),
        "B1": Place((io_n0, 45.0, 129.0), *_BAYB, "bay"),
        # The 5 V regulator on the rear lip (the Pi's port stack, 12 mm out from the spine, takes the left
        # wing's lower front); the servo regulator, switch and VSW hub on the left wing, clear of it.
        "U5": Place((-71.0 + p.wing_standoff, 20.0, 62.0), *_LIP, "bay"),
        "U6": Place((-68.7, wl, 90.0), *_LW, "bay"),
        "SW1": Place((-68.7, wl, 128.0), *_LW, "bay"),
        "X3": Place((-68.7, wl, 150.0), *_LW, "bay"),
        "MA": Place((-48.38, wr, 64.0), *_RW, "bay"),
        "MB": Place((-48.38, wr, 88.0), *_RW, "bay"),
        "F1": Place((-34.7, wr, 113.0), *_RW, "bay"),
        "X1": Place((-50.0, wr, 131.0), *_RW, "bay"),
        "X2": Place((-38.9, wr, 152.0), *_RW, "bay"),
        "X4": Place((-71.0, 11.8, 62.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), "bay"),
        # Panel-mounted behind the lower body's rear face (skeleton USB_C / POWER_SWITCH): the receptacle's
        # and the button's faces on the skin's inner face, x -90.
        "CHG1": Place((sk.LOWER_BODY.x0 + PORT_FACE_IN + 4.2, -29.0, 68.8), *_UP, "interface"),
        "B2": Place((sk.LOWER_BODY.x0 + PORT_FACE_IN + 1.6 + BUTTON["h"], 27.0, 65.0), *_BAYB, "interface"),
        "U12": Place((FRONT_X + p.carrier_t + 3.0, -16.5, 120.0), *_FWD, "chest"),
        "U11": Place((FRONT_X + p.carrier_t + 3.0, 19.0, 120.0), *_FWD, "chest"),
        "LS1": Place((58.0, -35.0, LS1_Z), *_FWD, "chest"),
        "U8": Place((75.0 - dt, fcx - dw / 2, fcz - dh / 2), *_FWD, "head"),
        # FPC edge DOWN (the lens 9.46 mm above it), so the board (z 151.5-175.4) stays in the head zone
        "U9": Place((cam_x - 0.7, CH[1] + 12.5, CH[2] + CAMERA_MOD["lens_from_far_edge"]),
                    (0.0, -1.0, 0.0), (0.0, 0.0, -1.0), "head"),
        "MK1": Place((62.0 - 8.35, 22.0 - 6.35, MIC_Z), *_UP, "head"),
        "MK2": Place((62.0 - 8.35, -22.0 - 6.35, MIC_Z), *_UP, "head"),
        "D1": Place((LED_CHANNEL_X, -LED_FRONT * STRIP["pitch"] / 2, sk.LED_STRIP_Z - STRIP["width"] / 2), *_FWD, "interface"),
        "D2": Place((-LED_CHANNEL_X, LED_REAR * STRIP["pitch"] / 2, sk.LED_STRIP_Z - STRIP["width"] / 2),
                    (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), "interface"),
        "B3": Place((-4.0, -9.0, SOCKET_FLOOR_Z), *_UP, "top_slot"),
    }


SIZES = {"U1": PI5["board_mm"], "B1": (90.0, 44.0, 1.6), "U5": REG5["size_mm"], "U6": REG6["size_mm"],
         "SW1": SWITCH["size_mm"], "X3": WAGO["221-415"]["mm"], "X2": WAGO["221-415"]["mm"], "X4": WAGO["221-415"]["mm"],
         "X1": WAGO["221-413"]["mm"], "MA": MOTORON["size_mm"], "MB": MOTORON["size_mm"], "F1": FUSE["holder_mm"],
         "CHG1": CHARGER["board_mm"], "B2": (14.0, 14.0, 1.6), "U12": TOUCH["size_mm"], "U11": AMP["size_mm"],
         "LS1": (SPEAKER["size_mm"][0], SPEAKER["size_mm"][1], SPEAKER["size_mm"][2]),
         "U8": DISPLAY["outline_mm"], "MK1": MIC["size_mm"], "MK2": MIC["size_mm"],
         "D1": (LED_FRONT * STRIP["pitch"], STRIP["width"], STRIP["t"]), "D2": (LED_REAR * STRIP["pitch"], STRIP["width"], STRIP["t"]),
         "B3": (8.0, 18.0, 1.6)}

# Parts whose solids reach past the electrical zones to meet a skeleton interface (the
# integrator's zone check must expect them), and parts with no solid of ours.
INTERFACE_PARTS = {"CHG1": "the USB-C charger, its receptacle in the shell's port at USB_C (x -90 to -66: past the bay)",
                   "B2": "the power button's board, behind the shell's POWER_SWITCH hole (x -90 to -80.4)",
                   "S1": "the power button (on B2)", "R10": "on B2",
                   "D1": "behind the bumper's diffuser at LED_STRIP_Z: inside the chassis's base zone",
                   "D2": "behind the bumper's diffuser at LED_STRIP_Z: inside the chassis's base zone",
                   "TP1": "copper tape on the shell's inside, CHEST_TOUCH", "TP2": "as TP1", "TP3": "as TP1",
                   "B3": "the accessory board on the top socket's floor (z 276): its 8.5 mm female header reaches z 286.1, "
                         "past the top_slot zone's 284 but inside the shell's TOP_SLOT socket (z 290)",
                   }
NO_SOLID = {"BT1": "the chassis places it (its battery volume)", "J1": "at the pack", "M1": "chassis", "M2": "chassis",
            "M3": "chassis", "M4": "chassis", "JS1": "arms harness handoff", "JS2": "arms harness handoff",
            "JM1": "harness", "JM2": "harness", "JM3": "harness", "JM4": "harness", "C2": "hangs on SW1",
            "R8": "inline in the harness", "R9": "inline in the harness", "C3": "on the strip's input"}


def _pico_pin(k: int) -> Tuple[float, float]:
    """Pico pin k (1..40) in B1's local frame: the Pico's USB end at B1 u = 5."""
    if k <= 20:
        return 5.0 + 1.37 + 2.54 * (k - 1), 22.0 + 1.61
    return 5.0 + 1.37 + 2.54 * (40 - k), 22.0 + 19.39


def _dip(pinmap, u0, v0=3.0, row=7.62):
    half = len(pinmap) // 2
    out = {}
    for k, name in pinmap.items():
        out[name] = (u0 + 2.54 * (k - 1), v0) if k <= half else (u0 + 2.54 * (2 * half - k), v0 + row)
    return out


U3_AT, U4_AT, U7_AT = 6.5, 34.0, 56.0       # U3 from 5.0 (2026-11-04): clear of the M2.5 screw head at B1 (2.5, 2.5)
B1_PASSIVES = {"R3": (60.0, 25.0), "R4": (60.0, 29.0), "R5": (60.0, 33.0), "R11": (60.0, 37.0), "R6": (72.0, 25.0),
               "R7": (72.0, 29.0), "D5": (72.0, 33.0), "C4": (72.0, 37.0), "C5": (72.0, 41.0), "C6": (60.0, 41.0)}
B1_FUSES = {"F2": (60.0, 17.0), "F3": (69.0, 17.0)}
MOTOR_AT_Z = sk.AXLE_Z


def _local_pins() -> Dict[str, Dict[str, Tuple[float, float, float]]]:
    """ref -> {pin: (u, v, n)} in the ref's placement frame (ASSUMED layouts are marked)."""
    L: Dict[str, Dict[str, Tuple[float, float, float]]] = {}
    # Pi 5: the header (pin 1 inner row, 3.5 mm from the edge); USB-C / HDMI on the far edge (drawing:
    # USB-C 11.2 mm in); CAM/DISP and FAN connectors ASSUMED positions.
    pi = {}
    for k, name in PI_HEADER.items():
        col, row = (k - 1) // 2, (k - 1) % 2
        pi[name] = (PI5["header_u0"] - 24.13 + 2.54 * col, 3.5 + (1.27 if row == 0 else -1.27), 10.1)
    pi.update({"CAMDISP0": (47.0, 52.0, 3.0), "CAMDISP1": (55.0, 52.0, 3.0), "FAN": (66.0, 49.0, 3.0),
               "USBC_PWR": (11.2, 56.0, 3.2)})
    L["U1"] = pi
    b1 = {"5V": (87.0, 15.0, 1.6), "3V3": (87.0, 19.0, 1.6), "GND": (87.0, 23.0, 1.6),     # the I2S hub pads moved
          "H_BCLK": (87.0, 29.0, 1.6), "H_LRCK": (87.0, 33.0, 1.6)}                        # off the top hook (87.5, 41.5)
    L["B1"] = b1
    pico = {name: (*_pico_pin(k), 2.6) for k, name in PICO_PINOUT.items()}
    pico.update({"SWCLK": (5.0 + 49.6, 22.0 + 8.0, 2.6), "SWGND": (5.0 + 49.6, 22.0 + 10.5, 2.6),
                 "SWDIO": (5.0 + 49.6, 22.0 + 13.0, 2.6), "USB": (4.0, 22.0 + 10.5, 4.0)})
    L["U2"] = pico
    L["U3"] = {k: (*v, 2.0) for k, v in _dip(DIP20_245, U3_AT).items()}
    L["U4"] = {k: (*v, 2.0) for k, v in _dip(DIP14_125, U4_AT).items()}
    L["U7"] = {k: (*v, 2.0) for k, v in _dip(DIP14_125, U7_AT).items()}
    for k in range(1, 5):
        u0 = 5.0 + 12.0 * (k - 1)
        L[f"JE{k}"] = {p: (u0 + 1.25 + 2.5 * j, 17.0, 8.0) for j, p in enumerate(("GND", "VCC", "A", "B"))}
    for ref, (u, v) in {**B1_PASSIVES, **B1_FUSES}.items():
        a, b = ("A", "K") if ref == "D5" else ("1", "2")
        L[ref] = {a: (u - 3.8, v, 2.6), b: (u + 3.8, v, 2.6)}
    # ASSUMED order along one 0.1 in row (VIN, VRP, GND, VOUT doubled on the board)
    L["U5"] = {"VIN": (3.2, 1.5, 1.6), "VRP": (7.0, 1.5, 1.6), "GND": (12.7, 1.5, 1.6), "VOUT": (18.4, 1.5, 1.6),
               "EN": (5.0, 23.9, 1.6), "PG": (8.0, 23.9, 1.6)}
    L["U6"] = {"VIN": (2.5, 1.5, 1.6), "VRP": (6.3, 1.5, 1.6), "GND": (10.1, 1.5, 1.6), "VOUT": (13.9, 1.5, 1.6),
               "EN": (4.0, 16.3, 1.6), "PG": (7.0, 16.3, 1.6)}
    L["SW1"] = {"VIN": (2.5, 3.0, 1.6), "GND": (2.5, 17.0, 1.6), "VOUT": (22.9, 3.0, 1.6), "A": (8.0, 17.8, 1.6),
                "B": (10.5, 17.8, 1.6), "ON": (13.0, 17.8, 1.6), "OFF": (15.5, 17.8, 1.6), "CTRL": (18.0, 17.8, 1.6)}
    for ref, size in (("X1", "221-413"), ("X2", "221-415"), ("X3", "221-415"), ("X4", "221-415")):
        n_ = WAGO[size]["slots"]
        Lx = WAGO[size]["mm"][0]
        L[ref] = {str(k + 1): (3.0 + (Lx - 6.0) / (n_ - 1) * k, 0.0, 4.0) for k in range(n_)}
    for ref in ("MA", "MB"):          # ASSUMED layout of the two 0.1 in rows (the guide's pinout is an image)
        L[ref] = {"VIN": (1.6, 1.3, 3.0), "GND": (4.6, 1.3, 3.0), "M1A": (8.5, 1.3, 3.0), "M1B": (11.0, 1.3, 3.0),
                  "M2A": (14.6, 1.3, 3.0), "M2B": (17.2, 1.3, 3.0), "GND2": (1.27, 13.9, 3.0), "VDD": (3.81, 13.9, 3.0),
                  "SCL": (6.35, 13.9, 3.0), "SDA": (8.89, 13.9, 3.0), "ERR": (11.43, 13.9, 3.0), "RST": (13.97, 13.9, 3.0),
                  "JMP1": (16.51, 13.9, 3.0), "VM": (19.05, 13.9, 3.0)}
    L["F1"] = {"1": (34.0, 7.0, 6.0), "2": (0.0, 7.0, 6.0)}
    L["CHG1"] = {"BAT+": (19.0, 4.0, 1.6), "BAT-": (19.0, 14.0, 1.6), "USBC": (-4.2, 9.0, 3.2)}
    L["B2"] = {}
    L["U12"] = {**{p: (4.0 + 2.54 * j, 1.3, 2.0) for j, p in enumerate(("VIN", "3Vo", "GND", "SCL", "SDA", "IRQ", "ADDR"))},
                **{str(i): (30.4 - 2.54 * i, 24.1, 2.0) for i in range(12)}}
    L["U11"] = {**{p: (2.1 + 2.54 * j, 1.3, 1.6) for j, p in enumerate(("LRC", "BCLK", "DIN", "GAIN", "SD", "GND", "Vin"))},
                "SPK+": (7.2, 15.5, 6.0), "SPK-": (12.2, 15.5, 6.0)}
    L["LS1"] = {"+": (30.0, 15.0, 0.0), "-": (40.0, 15.0, 0.0)}
    L["U8"] = {"DSI": (DISPLAY["outline_mm"][0] / 2, 4.0, 0.0)}    # ASSUMED: the FPC leaves the back, bottom centre
    L["MK1"] = {p: (2.1 + 2.54 * j, 1.3, 0.0) for j, p in enumerate(("3V", "GND", "BCLK", "DOUT", "LRCL", "SEL"))}
    L["MK2"] = dict(L["MK1"])
    L["D1"] = {"5V": (1.0, 3.0, 0.0), "DIN": (1.0, 6.25, 0.0), "GND": (1.0, 9.5, 0.0), "DOUT": (SIZES["D1"][0] - 1.0, 6.25, 0.0)}
    L["D2"] = {"5V": (1.0, 3.0, 0.0), "DIN": (1.0, 6.25, 0.0), "GND": (1.0, 9.5, 0.0), "DOUT": (SIZES["D2"][0] - 1.0, 6.25, 0.0)}
    L["B3"] = {}
    return L


def positions(r=None) -> Dict[Tuple[str, str], Tuple[float, float, float]]:
    """Every pin's position, robot frame, mm. Boards from their placements; the parts on B1
    from B1's frame; the chassis-side parts (pack, motors) where the chassis puts them or,
    where it has not said, ASSUMED; the arms' plugs at their shoulder zone's boundary."""
    pl = placements(r)
    L = _local_pins()
    out: Dict[Tuple[str, str], Tuple[float, float, float]] = {}
    on_b1 = {"U2", "U3", "U4", "U7", "JE1", "JE2", "JE3", "JE4", *B1_PASSIVES, *B1_FUSES}
    for ref, pins in L.items():
        frame = pl["B1"] if ref in on_b1 else pl.get(ref)
        if frame is None:
            continue
        for pin, (u, v, n) in pins.items():
            out[(ref, pin)] = frame.at(u, v, n)
    # The camera's CSI connector and the LEDs, the button, the cooler.
    cam = pl["U9"]
    out[("U9", "CSI")] = cam.at(12.5, 20.2, -2.6)
    out[("U10", "FAN")] = out[("U1", "FAN")]
    b2 = pl["B2"]
    out[("S1", "1")], out[("S1", "2")] = b2.at(3.5, 3.5, 0.0), b2.at(10.5, 3.5, 0.0)
    out[("S1", "LED+")], out[("S1", "LED-")] = b2.at(3.5, 10.5, 0.0), b2.at(10.5, 10.5, 0.0)
    out[("R10", "1")], out[("R10", "2")] = b2.at(1.0, 1.0, 0.0), out[("S1", "LED+")]
    for ref, y in (("D3", 20.0), ("D4", -20.0)):
        out[(ref, "A")] = (59.0, y + 1.27, CH[2])
        out[(ref, "K")] = (59.0, y - 1.27, CH[2])
    out[("R8", "2")], out[("R8", "1")] = out[("D3", "A")], (45.0, 21.27, CH[2])
    out[("R9", "2")], out[("R9", "1")] = out[("D4", "A")], (45.0, -18.73, CH[2])
    out[("C2", "+")], out[("C2", "-")] = out[("SW1", "VOUT")], out[("SW1", "GND")]
    out[("C3", "+")], out[("C3", "-")] = out[("D1", "5V")], out[("D1", "GND")]
    # Touch pads: copper tape on the inside of the chest wall, each wholly on one upper half.
    for k, (y0, y1) in TOUCH_PADS.items():
        out[(f"TP{k}", "1")] = (TOUCH_PAD_X - 0.1, (y0 + y1) / 2, 118.0)
    # Top slot header: 2 x 4, 2.54 mm, its pins soldered through B3; the wires on B3's underside, in the floor's slot.
    zt = SOCKET_FLOOR_Z
    for j, (a, b) in enumerate((("5V", "GND"), ("3V3", "GND2"), ("SDA", "SCL"), ("TX", "RX"))):
        y = -3.81 + 2.54 * j
        out[("J_TOP", a)], out[("J_TOP", b)] = (-1.27, y, zt), (1.27, y, zt)
    # Chassis side (ASSUMED until the chassis gives its motor terminal positions): each motor's
    # encoder end inboard of its wheel; the pack's leads at the deck's lead hole.
    for k, name in ((1, "front_left"), (2, "front_right"), (3, "rear_left"), (4, "rear_right")):
        x, y, z = sk.WHEELS[name]
        sy = 1.0 if y > 0 else -1.0
        ye = sy * 8.0
        out[(f"M{k}", "M+")], out[(f"M{k}", "M-")] = (x + 3.0, ye, z), (x - 3.0, ye, z)
        for j, pin in enumerate(("GND", "VCC", "A", "B")):
            out[(f"M{k}", pin)] = (x - 3.75 + 2.5 * j, ye, z + 9.0)
        jy = 8.0 * (k - 2.5)
        out[(f"JM{k}", "1")], out[(f"JM{k}", "2")] = (-39.0, jy - 2.1, 64.0), (-39.0, jy + 2.1, 64.0)
    hx, hy, hz = PACK_VOLUME["leads_hole"]
    ex, ey, ez = PACK["leads_exit"]
    out[("BT1", "P+")], out[("BT1", "P-")] = (ex + 2.0, ey, ez), (ex - 2.0, ey, ez)
    out[("J1", "1")], out[("J1", "2")] = (hx, 3.0, 50.0), (hx, -3.0, 50.0)
    for ref in ("JS1", "JS2"):             # the shoulder servo's B3B-EH-A header, at the shoulder axis
        x, y, z = SHOULDER_PLUG_AT[ref]
        for j, pin in enumerate(("GND", "V+", "DATA")):
            out[(ref, pin)] = (x - 8.0, y, z + 2.5 * (j - 1))
    return out


TOUCH_PAD_X = sk.UPPER_BODY.x1 - sk.SHELL_WALL   # the chest wall's flat inside face, x 78.0 (2.0 mm PETG)
# The three pads' y spans. The upper body splits LEFT / RIGHT at y 0 and the right half's seam wedge stands
# out of its wall at y -1.76..0 (shell.py _ub_tongue): a pad must sit whole on one half, flat. Until 2026-11-04
# TP2 (y -9..9) straddled the seam, 1.27 mm into the wedge. Now: TP1 right, TP2 and TP3 left; TP2 is narrower
# (10 mm, its centre 6 mm left of the seam) and 4 mm from TP3. Each pad is taped to its half on the bench.
TOUCH_PADS = {1: (-31.0, -13.0), 2: (1.0, 11.0), 3: (15.0, 31.0)}
TOUCH_PAD_Z = (117.0, 147.0)
TOUCH_PAD_HALF = {1: "shell_upper_right", 2: "shell_upper_left", 3: "shell_upper_left"}


# ---- the solids ------------------------------------------------------------------------------

_FACES = np.array([(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
                   (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)])


def _box(lo, hi) -> np.ndarray:
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    return v[_FACES]


def _cyl(r: float, h0: float, h1: float, n: int = 16) -> np.ndarray:
    """A closed n-gon prism about local z, z h0..h1."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.column_stack([r * np.cos(a), r * np.sin(a)])
    tris = []
    for i in range(n):
        p, q = ring[i], ring[(i + 1) % n]
        tris.append([(0, 0, h1), (p[0], p[1], h1), (q[0], q[1], h1)])
        tris.append([(0, 0, h0), (q[0], q[1], h0), (p[0], p[1], h0)])
        tris.append([(p[0], p[1], h0), (q[0], q[1], h0), (q[0], q[1], h1)])
        tris.append([(p[0], p[1], h0), (q[0], q[1], h1), (p[0], p[1], h1)])
    return np.asarray(tris, float)


def _xf(tris: np.ndarray, place: Place) -> np.ndarray:
    return tris @ place.R.T + np.asarray(place.origin, float)


def _lbox(place: Place, lo, hi) -> np.ndarray:
    return _xf(_box(lo, hi), place)


def _model_boxes(ref: str) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """Local boxes of one board's drawn model (PCB first)."""
    su, sv, sn = SIZES.get(ref, (0, 0, 0))
    if ref == "U1":   # Pi 5: PCB, header pins, ports block, HDMI/USB-C edge (dimensions from the drawing; heights ASSUMED)
        return [((0, 0, 0), (85, 56, 1.6)), ((7.0, 0.9, 1.6), (58.0, 6.1, 10.1)), ((65.0, 2.0, 1.6), (87.5, 54.0, 17.6)),
                ((3.0, 54.0, 1.6), (45.0, 57.5, 5.0))]
    if ref == "B1":
        return [((0, 0, 0), (su, sv, sn))]
    # Boards held by screws: the PCB (its holes cut in _board_models) and the parts on it, kept clear of the
    # screw heads (outlines ASSUMED where marked in BOARD_HOLES).
    if ref == "U5":
        return [((0, 0, 0), (su, sv, 1.6)), ((3.0, 3.0, 1.6), (20.0, sv - 3.0, sn))]
    if ref == "U6":
        return [((0, 0, 0), (su, sv, 1.6)), ((3.0, 3.0, 1.6), (15.5, sv - 3.0, sn))]
    if ref == "U12":
        return [((0, 0, 0), (su, sv, 1.6)), ((4.0, 0.0, 1.6), (29.0, sv, sn))]
    if ref == "U11":
        return [((0, 0, 0), (su, sv, 1.6)), ((4.0, 0.0, 1.6), (15.4, 12.5, sn))]
    if ref == "LS1":     # the enclosure between its two mounting ears (ears 3 mm thick at its back: ASSUMED)
        return [((7.0, 0, 0), (su - 7.0, sv, sn)), ((0, 0, 0), (7.0, sv, LS1_EAR_T)), ((su - 7.0, 0, 0), (su, sv, LS1_EAR_T))]
    if ref in ("MA", "MB", "SW1", "CHG1", "B2", "B3", "MK1", "MK2", "D1", "D2", "U8"):
        return [((0, 0, 0), (su, sv, sn))]
    if ref in ("X1", "X2", "X3", "X4", "F1"):
        return [((0, 0, 0), (su, sv, sn))]
    raise KeyError(ref)


def _board_models(r=None) -> Dict[str, List[np.ndarray]]:
    """Every board's drawn model, the fastened ones with their screw holes cut (_holed_boards)."""
    p = _p(r)
    out = {k: [x.copy() for x in v] for k, v in _board_models_plain(p).items()}
    out.update({k: [x.copy() for x in v] for k, v in _holed_boards(p).items()})
    return out


def _board_models_plain(r=None) -> Dict[str, List[np.ndarray]]:
    pl = placements(r)
    out: Dict[str, List[np.ndarray]] = {}
    for ref in SIZES:
        out[ref] = [_lbox(pl[ref], lo, hi) for lo, hi in _model_boxes(ref)]
    # extras on some boards
    out["U11"].append(_lbox(pl["U11"], (4.7, 13.0, 1.6), (14.7, 17.8, 10.0)))          # its speaker terminal block
    out["CHG1"].append(_lbox(pl["CHG1"], (-4.2, 4.53, 1.6), (3.15, 13.47, 4.8)))       # the USB-C receptacle
    out["B3"].append(_lbox(pl["B3"], (1.46, 3.92, 1.6), (6.54, 14.08, 10.1)))           # the 2 x 4 female header
    # Parts on B1 (B1's frame).
    b1 = pl["B1"]
    out["U2"] = [_lbox(b1, (5.0, 22.0, 1.6), (56.0, 43.0, 2.6)), _lbox(b1, (3.7, 28.5, 2.6), (11.0, 36.5, 5.8))]
    out["U3"] = [_lbox(b1, (U3_AT - 1.5, 1.0, 1.6), (U3_AT + 24.4, 12.6, 9.0))]
    out["U4"] = [_lbox(b1, (U4_AT - 1.5, 1.0, 1.6), (U4_AT + 16.7, 12.6, 9.0))]
    out["U7"] = [_lbox(b1, (U7_AT - 1.5, 1.0, 1.6), (U7_AT + 16.7, 12.6, 9.0))]
    for k in range(1, 5):
        u0 = 5.0 + 12.0 * (k - 1)
        out[f"JE{k}"] = [_lbox(b1, (u0, 14.3, 1.6), (u0 + 10.0, 20.05, 8.6))]
    for ref, (u, v) in B1_FUSES.items():
        out[ref] = [_lbox(b1, (u - 3.7, v - 1.6, 1.6), (u + 3.7, v + 1.6, 12.6))]
    for ref, (u, v) in B1_PASSIVES.items():
        out[ref] = [_lbox(b1, (u - 3.0, v - 1.2, 1.6), (u + 3.0, v + 1.2, 4.0))]
    # The cooler on the Pi.
    out["U10"] = [_lbox(pl["U1"], (1.5, 9.0, 1.6), (64.5, 51.5, 1.6 + COOLER["size_mm"][2]))]
    # The camera (measured on its STEP): PCB, lens housing, FPC connector at the back.
    cam = pl["U9"]
    out["U9"] = [_lbox(cam, (0, 0, 0), (25.0, 23.862, 0.7)), _lbox(cam, (7.1, 9.0, 0.7), (17.9, 19.8, 8.9)),
                 _lbox(cam, (1.04, 16.95, -2.59), (23.96, 23.49, 0.0))]
    # The status LEDs, the button.
    for ref, y in (("D3", 20.0), ("D4", -20.0)):
        out[ref] = [_cyl(STATUS_LED["d"] / 2, 0.0, STATUS_LED["l"]) @ _ROT_Z_TO_X.T + np.array([75.0 - STATUS_LED["l"], y, CH[2]])]
    b2 = pl["B2"]
    out["S1"] = [_xf(_cyl(BUTTON["d"] / 2, 1.6, 1.6 + BUTTON["h"]) + np.array([7.0, 7.0, 0.0]), b2)]
    out["R10"] = [_lbox(b2, (0.5, 0.2, -2.5), (8.0, 2.2, 0.0))]
    # Hanging parts the harness carries, drawn where they hang.
    out["C2"] = [_box((-62.0, 30.0, 125.0), (-46.0, 40.0, 135.0))]
    out["R8"] = [_box((45.0, 20.0, CH[2] - 1.25), (52.0, 22.5, CH[2] + 1.25))]
    out["R9"] = [_box((45.0, -20.0, CH[2] - 1.25), (52.0, -17.5, CH[2] + 1.25))]
    for k, (y0, y1) in TOUCH_PADS.items():
        out[f"TP{k}"] = [_box((TOUCH_PAD_X - 0.06, y0, TOUCH_PAD_Z[0]), (TOUCH_PAD_X, y1, TOUCH_PAD_Z[1]))]
    for k in range(1, 5):
        jy = 8.0 * (k - 2.5)
        out[f"JM{k}"] = [_box((-44.0, jy - 3.0, 61.0), (-34.0, jy + 3.0, 68.0))]
    return out


_ROT_Z_TO_X = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])   # local z -> robot +x


# ---- the made parts: three printed PETG carriers ---------------------------------------------

def _carrier_boxes(r=None) -> Dict[str, List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]]:
    """Robot-frame boxes of each printed carrier (each box inside one electrical zone). The holes
    (fasteners()) are cut from their union in _carrier_shapes()."""
    p = _p(r)
    t = p.carrier_t
    x0, x1 = BACK_X - t, BACK_X
    bay = []
    # The back plate, against the spine's rear face; a window above the spine's rear plate
    # (z 136..170) lets the head/chest harness and both FPCs into the spine.
    bay += [((x0, -54.0, 60.5), (x1, 54.0, 136.0)), ((x0, -54.0, 170.0), (x1, 54.0, 174.0)),
            ((x0, -54.0, 136.0), (x1, -35.0, 170.0)), ((x0, 35.0, 136.0), (x1, 54.0, 170.0))]
    # Wings (the regulators, switch, Motorons, fuse, hubs) on bridges: they stop at x -43 to
    # clear the Pi's ports. Above z WING_STEP_Z they start at x WING_X0, not -74: each upper shell
    # half slides on along y, and its two rear-hatch insert bosses (x -77.5..-69.7, z 97.4..106.6 and
    # 153.4..162.6) sweep through |y| 37..77 on the way; at x -74 the wings stood in that path (an FCL
    # sweep, 2026-11-04: hit 4.5-17 mm out). Below the step the wings keep x -74 and hold the rear lip.
    for s in (1.0, -1.0):
        ya, yb = sorted((s * 51.0, s * 54.0))
        if s > 0:     # the left wing stops short of the Pi's port stack (x -47.6 on 12 mm standoffs) below z 128
            bay.append(((-74.0, ya, 60.5), (-49.0, yb, WING_STEP_Z)))
            bay.append(((WING_X0, ya, WING_STEP_Z), (-49.0, yb, 128.0)))
            bay.append(((WING_X0, ya, 128.0), (-43.0, yb, 174.0)))
        else:
            bay.append(((-74.0, ya, 60.5), (-43.0, yb, WING_STEP_Z)))
            bay.append(((WING_X0, ya, WING_STEP_Z), (-43.0, yb, 174.0)))
        bay.append(((-49.0 if s > 0 else -43.0, ya, 60.5), (x0, yb, 64.0)))   # bottom bridge
        bay.append(((-43.0, ya, 171.0), (x0, yb, 174.0)))               # top bridge
    # The rear lip, with openings for the USB-C receptacle and the button; the charger's ledge;
    # the button board's posts.
    lip_x = (-74.0, -71.0)
    usb_y, usb_z = (-30.0, -10.0), (68.4, 71.5)          # the charger board's slot
    btn_y, btn_z = (13.0, 19.0), (67.0, 77.0)            # the button board's wires
    z_lip = (60.5, 95.0)
    for ya, yb in ((-51.0, usb_y[0]), (usb_y[1], btn_y[0]), (btn_y[1], 51.0)):
        bay.append(((lip_x[0], ya, z_lip[0]), (lip_x[1], yb, z_lip[1])))
    for (ya, yb), (za, zb) in ((usb_y, usb_z), (btn_y, btn_z)):
        bay.append(((lip_x[0], ya, z_lip[0]), (lip_x[1], yb, za)))
        bay.append(((lip_x[0], ya, zb), (lip_x[1], yb, z_lip[1])))
    bay.append(((-71.0, -29.0, 64.0), (-55.0, -11.0, 68.8)))
    # The duct's two insert pads on the lip's top (RX-M2x4's 3.2 mm hole: 1.3 mm of wall in x, 2.4 in y).
    for y in DUCT_SCREW_Y:
        bay.append(((DUCT_PAD_X[0], y - 4.0, 85.0), (DUCT_PAD_X[1], y + 4.0, z_lip[1])))
    # Bosses. The IO board's: two lower ones with M2.5 heat-set inserts, inside the hatch opening;
    # two upper HOOKS, a 2.4 mm pin through each of the board's top holes (it hangs on them while
    # its lower screws go in). The spine's row at z 64: M3 inserts in 8 mm bosses (an insert's 4.0
    # hole leaves 1.5-2 mm of wall; the 6 mm ones left 1.0).
    so = p.board_standoff
    pl = placements(r)
    for u, v in B1_HOLES["screws"]:
        _, y, z = pl["B1"].at(u, v, 0.0)
        bay.append(((x0 - so, y - 4.0, z - 4.0), (x0, y + 4.0, z + 4.0)))
    for u, v in B1_HOLES["hooks"]:
        _, y, z = pl["B1"].at(u, v, 0.0)
        bay.append(((x0 - so, y - 2.5, z - 2.5), (x0, y + 2.5, z + 2.5)))
    for y in BAY_ROW_Y:
        bay.append(((x0 - 6.0, y - 4.0, 60.5), (x0, y + 4.0, 68.0)))
    # Wing bosses (2 mm) under each wing board: at the screw holes of the screwed ones (U5 on the lip,
    # U6 on the left wing), 5 mm round the 1.7 mm pilot; at two corners of the taped ones (pads).
    for ref in ("U5", "U6"):
        for u, v in BOARD_HOLES[ref]["at"]:
            c = np.array(pl[ref].at(u, v, 0.0))
            if ref == "U5":
                bay.append(((-71.0, c[1] - 2.5, c[2] - 2.5), (c[0], c[1] + 2.5, c[2] + 2.5)))
            else:
                bay.append(((c[0] - 2.5, c[1], c[2] - 2.5), (c[0] + 2.5, 51.0, c[2] + 2.5)))
    for ref in ("SW1", "X3", "MA", "MB", "F1", "X1", "X2"):
        su, sv, _ = SIZES[ref]
        for u, v in ((2.0, 2.0), (su - 2.0, sv - 2.0)):
            c = np.array(pl[ref].at(u, v, 0.0))
            # on the wing: X3, X2 and F1 run past the wing's end (x -43), where a corner pad stood alone (a
            # separate body in the print, found when the carriers became one solid, 2026-11-04)
            x_hi = (-49.0 if c[1] > 0 and c[2] < 128.0 else -43.0) - 2.0
            x_lo = (-74.0 if c[2] < WING_STEP_Z else WING_X0) + 2.0
            c[0] = min(max(c[0], x_lo), x_hi)
            ywall = 51.0 if c[1] > 0 else -51.0
            ya, yb = sorted((c[1], ywall))
            bay.append(((c[0] - 2.0, ya, c[2] - 2.0), (c[0] + 2.0, yb, c[2] + 2.0)))
    front = []
    f0, f1 = FRONT_X, FRONT_X + t
    HW = DISPLAY["outline_mm"][0] / 2 + 5.65                              # the head's half-width (rails' outer faces)
    # One plate for chest and head (the spine's front grid is at z 190-250 only), a window at
    # z 140..178 for the harness out of the spine.
    # Lightened to a frame: bands where boards are fixed, side strips to the grid's screws.
    front += [((f0, -38.0, 86.0), (f1, 38.0, 95.0)), ((f0, -38.0, 95.0), (f1, -28.0, 115.0)),
              ((f0, 28.0, 95.0), (f1, 38.0, 115.0)), ((f0, -38.0, 115.0), (f1, 38.0, 140.0)),
              ((f0, -38.0, 140.0), (f1, -30.0, 150.0)), ((f0, 30.0, 140.0), (f1, 38.0, 150.0)),
              ((f0, -HW, 150.0), (f1, -30.0, 240.0)), ((f0, 30.0, 150.0), (f1, HW, 240.0)),
              ((f0, -HW, 240.0), (f1, HW, 261.0))]
    for y, z in HEAD_GRID_YZ:                                           # M3 insert bosses on the grid (8 mm)
        front.append(((f1, y - 4.0, z - 4.0), (f1 + 6.0, y + 4.0, z + 4.0)))
    # Head: rails, display ring, shelf with mic pillars, camera beam, plate, posts, LED towers.
    dw, dh, dt = DISPLAY["outline_mm"]
    zc = FW["centre"][2]
    ring_x0, ring_x1 = 75.0 - dt - 3.1, 75.0 - dt
    ro_y, ri_y = dw / 2 + 2.65, dw / 2 - 6.0                            # ring outer / opening half-widths
    ro_z0, ro_z1, ri_z0, ri_z1 = zc - dh / 2 - 2.5, zc + dh / 2 + 2.5, zc - dh / 2 + 6.0, zc + dh / 2 - 6.0
    for s_ in (1.0, -1.0):
        ya, yb = sorted((s_ * ro_y, s_ * HW))
        front.append(((f1, ya, 151.0), (ring_x0, yb, 257.0)))           # rails
        # ring sides, out to the rails' outer faces: the ring sits on the rails' ends (until 2026-11-04 the two
        # met along an edge only, and the printed carrier was two bodies)
        ya, yb = sorted((s_ * ri_y, s_ * HW))
        front.append(((ring_x0, ya, ri_z0), (ring_x1, yb, ri_z1)))
    front += [((ring_x0, -HW, ro_z0), (ring_x1, HW, ri_z0)), ((ring_x0, -HW, ri_z1), (ring_x1, HW, ro_z1))]
    front.append(((52.0, -HW, 254.0), (ring_x0, HW, 257.0)))           # mic shelf
    for yc in (22.0, -22.0):
        front.append(((56.0, yc - 5.0, 257.0), (68.0, yc + 5.0, MIC_Z)))
    cam_top = CH[2] + CAMERA_MOD["lens_from_far_edge"]
    front.append(((56.0, -HW, 151.0), (59.0, HW, 155.0)))              # camera beam
    front.append(((56.0, -27.0, 155.0), (59.0, 27.0, cam_top)))        # camera plate
    cam_back = pl["U9"].origin[0]
    for u, v in BOARD_HOLES["U9"]["at"]:                                # the CM3's holes: 2.0 and 14.5 from that edge
        _, y, z = pl["U9"].at(u, v, 0.0)
        front.append(((59.0, y - 2.2, z - 2.2), (cam_back, y + 2.2, z + 2.2)))
    for yc in (20.0, -20.0):
        front.append(((59.0, yc - 4.0, CH[2] - 4.0), (75.0 - STATUS_LED["l"], yc + 4.0, CH[2] + 4.0)))
    # Chest: speaker posts (8 mm: an M3 insert each), the MPR121's and amp's bosses (5 mm round a pilot).
    for u, v in BOARD_HOLES["LS1"]["at"]:
        _, y, z = pl["LS1"].at(u, v, 0.0)
        front.append(((f1, y - 4.0, z - 4.0), (pl["LS1"].origin[0], y + 4.0, z + 4.0)))
    for ref in ("U12", "U11"):
        o = pl[ref].origin
        for u, v in BOARD_HOLES[ref]["at"]:
            _, y, z = pl[ref].at(u, v, 0.0)
            front.append(((f1, y - 2.5, z - 2.5), (o[0], y + 2.5, z + 2.5)))
    return {"bay_carrier": bay, "front_carrier": front, "cooler_duct": _duct_boxes(r)}


# The Active Cooler's blower draws its air in through its face (the fan side, facing the hatch
# here) and throws it out through the heatsink's fins along the board: the brief dimensions the
# part (63.5 x 42.5 x 13.7) but not where the fins open, so the duct is built on the side that IS
# known: a hood sealed round the cooler's face, fed by a tube from the rear hatch's REAR_INLET.
# The blower then breathes outside air and pressurises the bay; the warm air leaves through
# REAR_VENT, the bay's only other opening. A 3 mm closed-cell foam gasket on the tube's end
# seals against the hatch's flat seat (DUCT_SEAT) when the hatch is closed.
DUCT = {"wall": 1.6, "gasket_mm": 3.0}


def _duct_geometry(r=None) -> dict:
    p = _p(r)
    pi_face = BACK_X - p.carrier_t - p.pi_standoff - PI5["board_mm"][2] - COOLER["size_mm"][2]   # the cooler's face, x
    cool = placements(r)["U1"]
    y0, y1 = sorted((cool.at(1.5, 0, 0)[1], cool.at(64.5, 0, 0)[1]))
    z0, z1 = sorted((cool.at(0, 9.0, 0)[2], cool.at(0, 51.5, 0)[2]))
    ins = sk.REAR_INLET
    iy, iz = ins["size"][0] / 2, ins["size"][1] / 2
    return {"hood_x": (-59.8, pi_face - 0.2), "hood_y": (y0 - 1.0, min(y1 + 0.3, 32.3)), "hood_z": (z0 - 1.5, z1 + 2.0),
            "tube_x": (-75.0, -59.8), "tube_in_y": (ins["centre"][1] - iy, ins["centre"][1] + iy),
            "tube_in_z": (ins["centre"][2] - iz, ins["centre"][2] + iz)}


def _duct_boxes(r=None):
    g = _duct_geometry(r)
    w = DUCT["wall"]
    (hx0, hx1), (hy0, hy1), (hz0, hz1) = g["hood_x"], g["hood_y"], g["hood_z"]
    (tx0, tx1), (ty0, ty1), (tz0, tz1) = g["tube_x"], g["tube_in_y"], g["tube_in_z"]
    b = [((hx0, hy0, hz1 - w), (hx1, hy1, hz1)), ((hx0, hy0, hz0), (hx1, hy1, hz0 + w)),            # hood top, bottom
         ((hx0, hy0, hz0 + w), (hx1, hy0 + w, hz1 - w)), ((hx0, hy1 - w, hz0 + w), (hx1, hy1, hz1 - w)),   # sides
         # the hood's front plate, with the tube's window
         ((hx0, hy0 + w, hz0 + w), (hx0 + w, hy1 - w, tz0)), ((hx0, hy0 + w, tz1), (hx0 + w, hy1 - w, hz1 - w)),
         ((hx0, hy0 + w, tz0), (hx0 + w, ty0, tz1)), ((hx0, ty1, tz0), (hx0 + w, hy1 - w, tz1)),
         # the tube to the hatch
         ((tx0, ty0 - w, tz1), (tx1, ty1 + w, tz1 + w)), ((tx0, ty0 - w, tz0 - w), (tx1, ty1 + w, tz0)),
         ((tx0, ty0 - w, tz0), (tx1, ty0, tz1)), ((tx0, ty1, tz0), (tx1, ty1 + w, tz1)),
         # the foot on the rear lip's top: 2 x M2 x 10 down through it, beside the tube, into the lip's pads
         ((DUCT_PAD_X[0], -DUCT_FOOT_HALF_Y, 95.0), (DUCT_PAD_X[1], DUCT_FOOT_HALF_Y, tz0 - w))]
    return b


def duct_seat() -> dict:
    """The flat seat the shell gives round REAR_INLET on the hatch's inside, for the duct's gasket."""
    g = _duct_geometry()
    w = DUCT["wall"]
    (ty0, ty1), (tz0, tz1) = g["tube_in_y"], g["tube_in_z"]
    outer = (ty1 - ty0 + 2 * w, tz1 - tz0 + 2 * w)
    return {"centre": sk.REAR_INLET["centre"], "opening": sk.REAR_INLET["size"],
            "seat_outer_mm": (round(outer[0] + 6.0, 1), round(outer[1] + 6.0, 1)),     # 3 mm of flat beyond the tube
            "tube_outer_mm": (round(outer[0], 1), round(outer[1], 1)), "gasket": "3 mm closed-cell foam, 2.5 mm gap",
            "face_x": sk.UPPER_BODY.x0 + HATCH_T}


PORT_FACE_IN = 2.5  # the port faces sit 2.5 behind the lower rear face, x -90 (shell.py PORT_FACE_IN; the wall is
                    # thickened there), so the button board's face lands on the shell's posts at x -82.0
HATCH_T = 2.5      # the hatch's thickness at its land (shell.py: hatch_boss_x1 = x0 + hatch_t + 6.5 + 1.3 = -69.7)
HATCH_BOSSES = {"yz": [(41.0, 102.0), (41.0, 158.0), (-41.0, 102.0), (-41.0, 158.0)], "d": 8.0, "x": (-80.0, -69.7),
                "source": "shell.py hatch_insert_points() and hatch_boss_x1()"}


CARRIER_ZONE = {"bay_carrier": "bay", "front_carrier": ("head", "chest"), "cooler_duct": "bay"}
STANDOFF = {"spec": "M2.5 x 12 mm brass hex standoff, female-female, 5 mm across flats, tapped M2.5 through (ASSUMED "
                    "generic)", "g": 1.1, "af": 5.0}
# (The four M2.5 x 16 screws from inside the spine into these standoffs are the chassis's parts and mass:
# chassis.pi5_joints().)

# ---- fasteners: every screw that passes through or into a printed carrier, and its hole -------------
# Screws are ISO 4762 socket head cap screws (fasteners.eu tables, as desk_robot/bought_parts.SHCS and
# chassis.SHCS give them). Drawn as the shell draws its own (shell.py _screw_solid): the head at its
# diameter, the shank at the thread's MINOR diameter (ISO 724 d1), so a screw in its insert or its
# thread-forming pilot does not overlap it; inserts are tubes 0.05 mm inside their hole and 0.05 mm
# outside the screw's minor diameter. Clearance holes are ISO 273 normal fit (chassis.CLEARANCE,
# desk_robot/design.CLEARANCE).
SCREW = {"M2": {"head_d": 3.8, "head_k": 2.0, "d": 2.0, "minor": 1.567, "clear": 2.4},
         "M2.5": {"head_d": 4.5, "head_k": 2.5, "d": 2.5, "minor": 2.013, "clear": 2.9},
         "M3": {"head_d": 5.5, "head_k": 3.0, "d": 3.0, "minor": 2.459, "clear": 3.4}}
SCREW_SOURCE = ("ISO 4762 heads: https://www.fasteners.eu/standards/ISO/4762/; ISO 724 minor diameters "
                "(shell.py MINOR_D); ISO 273 normal-fit clearance holes (chassis.py CLEARANCE)")
STEEL_G_MM3 = 7.85e-3
BRASS_G_MM3 = 8.5e-3
# Brass heat-set inserts, ruthex RX series (the shell uses the same M3 and M2 parts: shell.py INSERT,
# INSERT_M2). hole_depth: the insert's length plus room for the plastic it displaces and the screw's tip.
INSERTS = {
    "M3": {"spec": "ruthex RX-M3x5.7 brass heat-set insert (OD 4.6, 5.7 long)", "od": 4.6, "length": 5.7,
           "hole_d": 4.0, "hole_depth": 6.5, "wall_min": 1.6,
           "source": "https://www.ruthex.de (RX-M3x5.7: hole 4.0 mm), as shell.py INSERT"},
    "M2.5": {"spec": "ruthex RX-M2.5x5.7 brass heat-set insert (OD 4.6, 5.7 long)", "od": 4.6, "length": 5.7,
             "hole_d": 4.0, "hole_depth": 6.5, "wall_min": 1.6,
             "source": "https://www.3djake.com/ruthex/threaded-insert-m25-70-pieces (ruthex table: D1 4.6, D3 4.0, "
                       "L 5.7, wall min 1.6; hole depth ASSUMED as the M3's)"},
    "M2": {"spec": "ruthex RX-M2x4 brass heat-set insert (OD 3.6, 4.0 long)", "od": 3.6, "length": 4.0,
           "hole_d": 3.2, "hole_depth": 4.5, "wall_min": 1.3,
           "source": "https://www.ruthex.de (RX-M2x4: hole 3.2 mm), as shell.py INSERT_M2"},
}
for _k, _ins in INSERTS.items():      # brass between the knurl's OD and the thread (as shell.py INSERT["g"])
    _ins["g"] = round(math.pi / 4 * (_ins["od"] ** 2 - {"M3": 2.5, "M2.5": 2.0, "M2": 1.6}[_k] ** 2) * _ins["length"]
                      * BRASS_G_MM3, 2)
# An M2 screw forms its own thread in a 1.7 mm printed hole (desk_robot/design.M2_TAP_PLA_MM, the desk
# robot's measured figure): the small boards' screws, where a 3.2 mm insert hole would leave no wall.
PILOT_M2 = {"d": 1.7, "source": "desk_robot/design.py M2_TAP_PLA_MM (an M2 screw cuts its own thread in a 1.7 mm hole)"}

# The fixing points, in each board's own frame (u, v), and its hole. Boards whose drawings give the holes say
# so; the rest are ASSUMED (placed clear of the board's pins) and must be checked against the board bought.
SPINE_T = 6.0           # chassis.Chassis.spine_t: the spine's plywood plates (the head of a screw from inside sits on them)
BAY_ROW_Y, BAY_ROW_Z = (-40.0, -20.0, 0.0, 20.0, 40.0), 64.0          # chassis.REAR_ROW
HEAD_GRID_YZ = ((-40.0, 190.0), (40.0, 190.0), (-40.0, 250.0), (40.0, 250.0))   # of chassis.FRONT_GRID: the ones used
B1_HOLES = {"screws": ((2.5, 2.5), (87.5, 2.5)), "hooks": ((2.5, 41.5), (87.5, 41.5)), "d": 2.9, "pin_d": 2.4,
            "pin_out": 3.0, "source": "our perfboard, drilled 2.9 mm 2.5 mm in from its edges"}
BOARD_HOLES = {
    "U5": {"at": ((22.9, 6.0), (22.9, 19.4)), "d": 2.2,
           "source": "ASSUMED: Pololu's #2 / M2 mounting holes (0.086 in), positions not on its page"},
    "U6": {"at": ((17.8, 5.0), (17.8, 12.8)), "d": 2.2,
           "source": "ASSUMED: Pololu's #2 / M2 mounting holes (0.086 in), positions not on its page"},
    "U9": {"at": ((2.0, 2.0), (23.0, 2.0), (2.0, 14.5), (23.0, 14.5)), "d": 2.2,
           "source": "Camera Module 3 product brief: 4 x 2.2 mm holes, 21 x 12.5 mm"},
    "U12": {"at": ((2.0, 12.7), (31.0, 12.7)), "d": 2.5, "source": "ASSUMED: Adafruit #1982's 2.5 mm holes, mid-ends"},
    "U11": {"at": ((2.0, 15.5), (17.4, 15.5)), "d": 2.5, "source": "ASSUMED: Adafruit #3006's 2.5 mm holes"},
    "LS1": {"at": ((3.5, 3.0), (66.5, 3.0), (3.5, 27.0), (66.5, 27.0)), "d": SPEAKER["hole_d"],
            "source": f"{SPEAKER['source']}: 63 x 24 mm, 3.4 mm (ear thickness ASSUMED 3 mm)"},
}
LS1_EAR_T = 3.0
# The cooler duct's foot: two M2 x 10 down through it (beside the tube, clear of the hatch bosses at |y| >= 37.25)
# into RX-M2x4 inserts in two pads on the rear lip's top; and the wings' step (see _carrier_boxes).
DUCT_SCREW_Y = (34.0, -34.0)
DUCT_PAD_X = (-75.0, -69.2)
DUCT_FOOT_HALF_Y = 36.5
WING_X0 = HATCH_BOSSES["x"][1] + 0.5          # -69.2: 0.5 mm clear of the hatch bosses' inner faces
WING_STEP_Z = 96.0                             # the bosses' lowest point is z 97.4 (shell.py, measured on its mesh)


def _fastener(label, size, length, at, axis, holder, carrier, mouth, through, step, into, *, owner="electrical",
              interface=False, insert=None, pilot_depth=None, grip=None):
    s = SCREW[size]
    ax = tuple(float(a) for a in axis)
    hole = None
    if holder == "insert":
        ins = INSERTS[insert]
        hole = {"d": ins["hole_d"], "depth": ins["hole_depth"], "what": f"{ins['hole_d']:g} mm insert hole, "
                f"{ins['hole_depth']:g} deep, for an {ins['spec']}"}
    elif holder == "pilot":
        hole = {"d": PILOT_M2["d"], "depth": pilot_depth, "what": f"{PILOT_M2['d']:g} mm pilot, {pilot_depth:g} deep "
                "(thread-forming)"}
    elif holder == "through":
        hole = {"d": s["clear"], "depth": None, "what": f"{s['clear']:g} mm clearance hole (ISO 273 normal)"}
    return {"label": label, "size": size, "length": length, "head": "ISO 4762", "at": tuple(float(v) for v in at),
            "axis": ax, "grip_mm": grip, "thread_in_mm": None if grip is None else round(length - grip, 2),
            "holder": holder, "insert": insert, "carrier": carrier, "mouth": tuple(float(v) for v in mouth),
            "hole": hole, "through": through, "step": step, "owner": owner, "interface": interface, "into": into}


def fasteners(r=None) -> List[dict]:
    """Every screw through or into a printed carrier: the screw, the head's bearing point and the shank's
    direction, what it passes (each with its clearance hole) and what holds its thread (an insert, a
    thread-forming pilot, the Pi's standoff), and the hole that gives it in the carrier."""
    p = _p(r)
    pl = placements(r)
    x0 = BACK_X - p.carrier_t
    out = []
    # The Pi 5: the chassis's M2.5 x 16 from inside the spine (head on the rear plate's inner face, x -9),
    # through the spine (6) and our back plate (3): a 9 mm grip, then 7 mm into the 12 mm F-F standoff.
    sx, sz = PI5["holes"]
    for i, (dy, dz) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        y, z = dy * sx / 2, 98.0 + dz * sz / 2
        out.append(_fastener(f"screw_interface_bay_pi5_{i}", "M2.5", 16, (BACK_X + SPINE_T, y, z), (-1, 0, 0), "standoff",
                             "bay_carrier", (BACK_X, y, z), [("spine rear plate", SCREW["M2.5"]["clear"]),
                                                             ("bay_carrier", SCREW["M2.5"]["clear"])], 1,
                             "the Pi's 12 mm F-F standoff: 7 mm of its thread", owner="chassis", interface=True,
                             grip=SPINE_T + p.carrier_t))
    # The bay carrier's bottom row on the spine: M3 x 12 from inside the spine, into an insert pressed into
    # the carrier's face against the spine (6 mm of thread in the 5.7 mm insert's hole).
    for i, y in enumerate(BAY_ROW_Y):
        out.append(_fastener(f"el_screw_bay_row_{i}", "M3", 12, (BACK_X + SPINE_T, y, BAY_ROW_Z), (-1, 0, 0), "insert",
                             "bay_carrier", (BACK_X, y, BAY_ROW_Z), [("spine rear plate", SCREW["M3"]["clear"])], 1,
                             "an RX-M3x5.7 in the bay carrier's boss", interface=True, insert="M3", grip=SPINE_T))
    for i, (y, z) in enumerate(HEAD_GRID_YZ):
        out.append(_fastener(f"el_screw_head_grid_{i}", "M3", 12, (FRONT_X - SPINE_T, y, z), (1, 0, 0), "insert",
                             "front_carrier", (FRONT_X, y, z), [("spine front plate", SCREW["M3"]["clear"])], 4,
                             "an RX-M3x5.7 in the front carrier's grid boss", interface=True, insert="M3", grip=SPINE_T))
    # The IO board: M2.5 x 6 through its 2.9 mm holes into RX-M2.5x5.7 inserts in the two lower bosses.
    b1 = pl["B1"]
    n1 = -np.asarray(b1.R[:, 2])
    for k, (u, v) in enumerate(B1_HOLES["screws"]):
        out.append(_fastener(f"el_screw_B1_{k}", "M2.5", 6, b1.at(u, v, SIZES["B1"][2]), n1, "insert", "bay_carrier",
                             b1.at(u, v, 0.0), [("B1", B1_HOLES["d"])], 3, "an RX-M2.5x5.7 in the IO board's lower boss",
                             insert="M2.5", grip=SIZES["B1"][2]))
    # The two regulators: M2 x 5 through the board into a 1.7 mm pilot, 4 mm deep (2 mm boss + 3 mm wall).
    for ref in ("U5", "U6"):
        pr = pl[ref]
        nr = -np.asarray(pr.R[:, 2])
        for k, (u, v) in enumerate(BOARD_HOLES[ref]["at"]):
            out.append(_fastener(f"el_screw_{ref}_{k}", "M2", 5, pr.at(u, v, 1.6), nr, "pilot", "bay_carrier",
                                 pr.at(u, v, 0.0), [(ref, BOARD_HOLES[ref]["d"])], 2,
                                 "a 1.7 mm pilot in its boss and the wall behind", pilot_depth=4.0, grip=1.6))
    # The cooler duct: M2 x 10 down through the foot (6.4 mm) into RX-M2x4 inserts in the lip's pads.
    w = DUCT["wall"]
    foot_top = _duct_geometry(r)["tube_in_z"][0] - w
    xc = (DUCT_PAD_X[0] + DUCT_PAD_X[1]) / 2
    for k, y in enumerate(DUCT_SCREW_Y):
        out.append(_fastener(f"el_screw_duct_{k}", "M2", 10, (xc, y, foot_top), (0, 0, -1), "insert", "bay_carrier",
                             (xc, y, 95.0), [("cooler_duct", SCREW["M2"]["clear"])], 3,
                             "an RX-M2x4 in the rear lip's pad", insert="M2", grip=foot_top - 95.0))
    # The camera: M2 x 5 through its 2.2 mm holes into 1.7 mm pilots in its four posts.
    for ref, size, length, depth, step in (("U9", "M2", 5, 5.5, 4), ("U12", "M2", 6, 5.5, 5), ("U11", "M2", 6, 5.5, 5)):
        pr = pl[ref]
        nr = -np.asarray(pr.R[:, 2])
        t_b = 0.7 if ref == "U9" else 1.6
        for k, (u, v) in enumerate(BOARD_HOLES[ref]["at"]):
            out.append(_fastener(f"el_screw_{ref}_{k}", size, length, pr.at(u, v, t_b), nr, "pilot", "front_carrier",
                                 pr.at(u, v, 0.0), [(ref, BOARD_HOLES[ref]["d"])], step,
                                 "a 1.7 mm pilot in its post / boss", pilot_depth=depth, grip=t_b))
    # The speaker: M3 x 8 through its ears (3 mm) into RX-M3x5.7 inserts at the posts' ends.
    pr = pl["LS1"]
    nr = -np.asarray(pr.R[:, 2])
    for k, (u, v) in enumerate(BOARD_HOLES["LS1"]["at"]):
        out.append(_fastener(f"el_screw_LS1_{k}", "M3", 8, pr.at(u, v, LS1_EAR_T), nr, "insert", "front_carrier",
                             pr.at(u, v, 0.0), [("LS1", BOARD_HOLES["LS1"]["d"])], 5, "an RX-M3x5.7 at its post's end",
                             insert="M3", grip=LS1_EAR_T))
    return out


# ---- OpenCascade: the carriers, the holed boards, the standoffs, screws and inserts as closed solids ------

def _o_box(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    return BRepPrimAPI_MakeBox(gp_Pnt(min(x0, x1), min(y0, y1), min(z0, z1)),
                               gp_Pnt(max(x0, x1), max(y0, y1), max(z0, z1))).Shape()


def _o_cyl(at, axis, d, length):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*[float(v) for v in at]), gp_Dir(*[float(v) for v in axis])),
                                    d / 2.0, float(length)).Shape()


_PLANE = {0: (1, 2), 1: (0, 2), 2: (0, 1)}


def _o_prism(poly, k: int, lo: float, hi: float):
    """A shapely polygon (holes allowed) in the plane normal to axis k, extruded from lo to hi along it."""
    ia, ib = _PLANE[k]

    def pnt(a, b, c):
        v = [0.0, 0.0, 0.0]
        v[ia], v[ib], v[k] = a, b, c
        return gp_Pnt(*v)

    def wire(ring):
        mp = BRepBuilderAPI_MakePolygon()
        for a, b in list(ring.coords)[:-1]:
            mp.Add(pnt(a, b, lo))
        mp.Close()
        return mp.Wire()
    face = BRepBuilderAPI_MakeFace(wire(poly.exterior), True).Face()
    vec = [0.0, 0.0, 0.0]
    vec[k] = hi - lo
    solid = BRepPrimAPI_MakePrism(face, gp_Vec(*vec)).Shape()
    holes = [_o_prism(Polygon(h), k, lo - 1.0, hi + 1.0) for h in poly.interiors]
    return _o_cut(solid, holes) if holes else solid


def _o_list(shapes):
    out = TopTools_ListOfShape()
    for s in shapes:
        out.Append(s)
    return out


def _o_bool(op, base, tools):
    tools = [t for t in tools if t is not None]
    if not tools:
        return base
    op.SetArguments(_o_list([base]))
    op.SetTools(_o_list(tools))
    op.SetFuzzyValue(1e-5)
    op.Build()
    if not op.IsDone():
        raise RuntimeError("boolean failed")
    return op.Shape()


def _o_fuse(shapes):
    return _o_clean(_o_bool(BRepAlgoAPI_Fuse(), shapes[0], shapes[1:]))


def _o_cut(base, tools):
    return _o_bool(BRepAlgoAPI_Cut(), base, tools)


def _o_clean(shape):
    u = ShapeUpgrade_UnifySameDomain(shape, True, True, True)
    u.Build()
    return u.Shape()


def _o_volume(shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return float(props.Mass())


def _o_tris(shape, lin: float = 0.02, ang: float = 0.2) -> np.ndarray:
    """Triangles (n, 3, 3), outward-wound: OpenCascade meshes each edge once, so the faces share their
    boundary vertices and the result is closed (as shell.py tessellate())."""
    BRepMesh_IncrementalMesh(shape, lin, False, ang, True)
    out = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            T = loc.Transformation()
            nodes = np.empty((tri.NbNodes(), 3))
            for i in range(1, tri.NbNodes() + 1):
                q = tri.Node(i).Transformed(T)
                nodes[i - 1] = (q.X(), q.Y(), q.Z())
            idx = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)], dtype=np.int64) - 1
            if f.Orientation() == TopAbs_REVERSED:
                idx = idx[:, [0, 2, 1]]
            out.append(nodes[idx])
        ex.Next()
    return np.concatenate(out)


def _teardrop(cx: float, cy: float, d: float, toward) -> Polygon:
    """A round hole with a 90 degree point toward `toward` (the print's up): it prints without a sagging roof."""
    t = np.asarray(toward, float)
    t = t / np.linalg.norm(t)
    r = d / 2.0
    n = np.array([-t[1], t[0]])
    tip = np.array([cx, cy]) + t * r * math.sqrt(2.0)
    a = np.array([cx, cy]) + n * r / math.sqrt(2.0) + t * r / math.sqrt(2.0)
    b = np.array([cx, cy]) - n * r / math.sqrt(2.0) + t * r / math.sqrt(2.0)
    return unary_union([Point(cx, cy).buffer(r, quad_segs=16), Polygon([tuple(a), tuple(tip), tuple(b)])])


def _o_hole(mouth, axis, d, depth, up=None):
    """A hole from 0.5 mm outside its mouth to `depth` past it along a principal `axis` (None: through, 40 mm).
    Horizontal in the print (axis normal to `up`): a teardrop pointing up."""
    ax = np.asarray(axis, float)
    k = int(np.argmax(np.abs(ax)))
    start = np.asarray(mouth, float) - ax * 0.5
    length = (depth if depth is not None else 40.0) + 0.5
    if depth is None:
        start = np.asarray(mouth, float) - ax * 20.0
        length = 40.0
    if up is None or abs(float(np.dot(ax, up))) > 0.5:
        return _o_cyl(start, ax, d, length)
    ia, ib = _PLANE[k]
    poly = _teardrop(start[ia], start[ib], d, (up[ia], up[ib]))
    lo, hi = sorted((start[k], start[k] + ax[k] * length))
    return _o_prism(poly, k, lo, hi)


def _carrier_holes(r=None) -> Dict[str, list]:
    """carrier -> [(mouth, axis, d, depth or None for through)]: every fastener's hole in it."""
    out: Dict[str, list] = defaultdict(list)
    for f in fasteners(r):
        h = f["hole"]
        if f["holder"] == "standoff":            # the Pi's screws pass our plate: a clearance hole through it
            out[f["carrier"]].append((f["mouth"], f["axis"], SCREW[f["size"]]["clear"], None))
        elif f["carrier"] and h:
            out[f["carrier"]].append((f["mouth"], f["axis"], h["d"], h["depth"]))
        for what, d in f["through"]:
            if what in CARRIER_ZONE and what != f["carrier"]:     # the duct's foot: a clearance hole through it
                out[what].append((f["at"], f["axis"], d, None))
    return out


def _print_up(name: str) -> np.ndarray:
    return np.asarray(_PRINT_ROT[name][2], float)       # the robot direction that becomes the bed's +z


@functools.lru_cache(maxsize=4)
def _carrier_shapes(p: Params) -> Dict[str, object]:
    boxes = _carrier_boxes(p)
    holes = _carrier_holes(p)
    out = {}
    for name, bxs in boxes.items():
        parts_ = [_o_box(lo, hi) for lo, hi in bxs]
        if name == "bay_carrier":            # the IO board's top hooks: a pin through each of its holes
            b1 = placements(p)["B1"]
            for u, v in B1_HOLES["hooks"]:
                parts_.append(_o_cyl(b1.at(u, v, 0.0), b1.R[:, 2], B1_HOLES["pin_d"], SIZES["B1"][2] + B1_HOLES["pin_out"]))
        shape = _o_fuse(parts_)
        up = _print_up(name)
        cut = [_o_hole(m, a, d, dep, up) for m, a, d, dep in holes.get(name, [])]
        out[name] = _o_clean(_o_cut(shape, cut)) if cut else shape
    return out


@functools.lru_cache(maxsize=4)
def _carrier_tris(p: Params) -> Dict[str, np.ndarray]:
    return {name: _o_tris(s) for name, s in _carrier_shapes(p).items()}


def carrier_volumes_mm3(r=None) -> Dict[str, float]:
    return {name: _o_volume(s) for name, s in _carrier_shapes(_p(r)).items()}


def _hole_spots(r=None) -> Dict[str, list]:
    """ref -> [(point on the board's mounting face, axis, d)]: the holes its screws (or the hooks) pass."""
    pl = placements(r)
    out: Dict[str, list] = defaultdict(list)
    for f in fasteners(r):
        for what, d in f["through"]:
            if what in pl:
                out[what].append((f["mouth"], f["axis"], d))
    for u, v in B1_HOLES["hooks"]:
        out["B1"].append((pl["B1"].at(u, v, 0.0), (1.0, 0.0, 0.0), B1_HOLES["d"]))
    return out


@functools.lru_cache(maxsize=4)
def _holed_boards(p: Params) -> Dict[str, List[np.ndarray]]:
    """The fastened boards' shells with their holes cut (each shell a box: placements are axis-aligned)."""
    models = _board_models_plain(p)
    out = {}
    for ref, spots in _hole_spots(p).items():
        shells = []
        for sh in models[ref]:
            v = sh.reshape(-1, 3)
            lo, hi = v.min(0), v.max(0)
            tools = []
            for at, ax, d in spots:
                c = np.asarray(at, float)
                k = int(np.argmax(np.abs(ax)))
                rest = [i for i in range(3) if i != k]
                if all(lo[i] - d < c[i] < hi[i] + d for i in rest):
                    tools.append(_o_hole(c, ax, d, None))
            shells.append(_o_tris(_o_clean(_o_cut(_o_box(lo, hi), tools))) if tools else sh)
        out[ref] = shells
    return out


def _standoff_shape(at_mouth, length: float):
    """One F-F hex standoff along -x from the carrier's face: 5 mm across flats, its M2.5 thread drawn at the
    tapped bore's minor diameter (ISO 724 d1 2.013), so the chassis's screw (drawn at its 2.5 mm) reads as
    threaded in it."""
    x, y, z = at_mouth
    rc = STANDOFF["af"] / math.sqrt(3.0)
    hexa = Polygon([(y + rc * math.cos(a), z + rc * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 7)[:-1]])
    hexa = hexa.difference(Point(y, z).buffer(SCREW["M2.5"]["minor"] / 2.0, quad_segs=16))
    return _o_prism(hexa, 0, x - length, x)


@functools.lru_cache(maxsize=4)
def _standoff_tris(p: Params) -> Tuple[np.ndarray, ...]:
    x0 = BACK_X - p.carrier_t
    sx, sz = PI5["holes"]
    return tuple(_o_tris(_standoff_shape((x0, y, z), p.pi_standoff))
                 for y in (-sx / 2, sx / 2) for z in (98.0 - sz / 2, 98.0 + sz / 2))


def _standoffs(r=None) -> List[np.ndarray]:
    return [t.copy() for t in _standoff_tris(_p(r))]


def _screw_shape(f: dict):
    s = SCREW[f["size"]]
    at, ax = np.asarray(f["at"], float), np.asarray(f["axis"], float)
    head = _o_cyl(at - ax * s["head_k"], ax, s["head_d"], s["head_k"])
    shank = _o_cyl(at, ax, s["minor"], f["length"])
    return _o_fuse([head, shank])


def _insert_shape(f: dict):
    ins = INSERTS[f["insert"]]
    m, ax = np.asarray(f["mouth"], float), np.asarray(f["axis"], float)
    tube = _o_cyl(m, ax, ins["hole_d"] - 0.1, ins["length"])
    return _o_clean(_o_cut(tube, [_o_cyl(m - ax, ax, SCREW[f["size"]]["minor"] + 0.1, ins["length"] + 2.0)]))


@functools.lru_cache(maxsize=4)
def _fastener_tris(p: Params) -> Dict[str, np.ndarray]:
    """label -> triangles of every screw we own and every insert (the Pi's screws are the chassis's)."""
    out = {}
    for f in fasteners(p):
        if f["owner"] != "electrical":
            continue
        out[f["label"]] = _o_tris(_screw_shape(f))
        if f["insert"]:
            out[f["label"].replace("el_screw_", "el_insert_")] = _o_tris(_insert_shape(f))
    return out


def fastener_mass_g(f: dict) -> float:
    s = SCREW[f["size"]]
    return (math.pi / 4 * s["head_d"] ** 2 * s["head_k"] + math.pi / 4 * (0.92 * s["d"]) ** 2 * f["length"]) * STEEL_G_MM3


def solids(r=None) -> Dict[str, np.ndarray]:
    """label -> triangles (N, 3, 3), robot frame, mm: every electrical part we place, closed shells.
    Labels are prefixed 'el_'. The carriers are OpenCascade solids with every fastener's hole cut;
    our screws and inserts are here too, except the ones from inside the spine (interface_solids())."""
    p = _p(r)
    out: Dict[str, np.ndarray] = {}
    for ref, shells in _board_models(p).items():
        out[f"el_{ref}"] = np.vstack(shells)
    for name, tris in _carrier_tris(p).items():
        out[f"el_{name}"] = tris.copy()
    out["el_pi_standoffs"] = np.vstack(_standoffs(p))
    inter = {f["label"] for f in fasteners(p) if f["interface"]}
    for label, tris in _fastener_tris(p).items():
        if label not in inter:
            out[label] = tris.copy()
    return out


def interface_solids(r=None) -> Dict[str, np.ndarray]:
    """Our screws from inside the spine into our inserts (bay_row, head_grid): they cross the chassis's
    zone by design, so the integrator checks them for interference but not for zones."""
    p = _p(r)
    inter = {f["label"] for f in fasteners(p) if f["interface"] and f["owner"] == "electrical"}
    return {k: v.copy() for k, v in _fastener_tris(p).items() if k in inter}


# =========================================================================================
# 6. The wires
# =========================================================================================
# (id, from, to, net, colour, AWG, end at from, end at to, what current it carries)
_T = "soldered into the board's through-hole"
_PAD = "2.54 mm crimp housing on B1's pin header (unplugs)"
_B1PWR = "JST XHP-2 housing on B1's B2B-XH-A power header (unplugs)"
_DUP = "2.54 mm crimp housing on the Pi's header"
_WG = "WAGO 221 lever slot"
_MF_R, _MF_P = "Molex Mini-Fit Jr 39-01-2020 receptacle (5556 crimps)", "Molex Mini-Fit Jr 39-01-2021 plug (5558 crimps)"
_XH = "JST XHP-4 housing (SXH-001T-P0.6 crimps) on B1's header"
_OWN_M = "the motor's own lead (chassis)"
_FPC = "FPC, ZIF connector"


def _w():
    W = []

    def w(a, b, net, colour, awg, ca, cb, carries, kind="wire"):
        W.append([None, a, b, net, colour, awg, ca, cb, carries, kind])
    # Battery to the switched bus.
    w(("BT1", "P+"), ("J1", "1"), "VBAT_RAW", "red", 18, "the pack's own lead", "XT30U-F", "pack")
    w(("BT1", "P-"), ("J1", "2"), "GND", "black", 18, "the pack's own lead", "XT30U-F", "pack")
    w(("J1", "1"), ("F1", "1"), "VBAT_RAW", "red", 18, "XT30U-M", "fuse holder lead, butt splice", "pack")
    w(("F1", "2"), ("X1", "1"), "VBAT", "red", 18, "fuse holder lead", _WG, "pack")
    w(("X1", "2"), ("SW1", "VIN"), "VBAT", "red", 18, _WG, "SW1's large VIN hole", "pack")
    w(("X1", "3"), ("CHG1", "BAT+"), "VBAT", "red", 22, _WG, "the charger's B+ pad", "charge")
    w(("J1", "2"), ("X2", "1"), "GND", "black", 18, "XT30U-M", _WG, "pack")
    w(("CHG1", "BAT-"), ("X2", "2"), "GND", "black", 22, "the charger's B- pad", _WG, "charge")
    w(("SW1", "VOUT"), ("X3", "1"), "VSW", "red", 18, "SW1's large VOUT hole (C2 soldered here too)", _WG, "pack")
    w(("X3", "2"), ("U5", "VIN"), "VSW", "red", 20, _WG, _T, "reg5_in")
    w(("X3", "3"), ("U6", "VIN"), "VSW", "red", 20, _WG, _T, "reg6_in")
    w(("X3", "4"), ("MA", "VIN"), "VSW", "red", 18, _WG, "the Motoron's VIN hole", "motoron_in")
    w(("X3", "5"), ("MB", "VIN"), "VSW", "red", 18, _WG, "the Motoron's VIN hole", "motoron_in")
    w(("X2", "3"), ("U5", "GND"), "GND", "black", 20, _WG, _T, "reg5_in")
    w(("X2", "4"), ("U6", "GND"), "GND", "black", 20, _WG, _T, "reg6_in")
    w(("X2", "5"), ("X4", "1"), "GND", "black", 18, _WG, _WG, "motors_total")
    w(("X4", "2"), ("MA", "GND"), "GND", "black", 18, _WG, "the Motoron's GND hole", "motoron_in")
    w(("X4", "3"), ("MB", "GND"), "GND", "black", 18, _WG, "the Motoron's GND hole", "motoron_in")
    w(("X4", "4"), ("SW1", "GND"), "GND", "black", 24, _WG, _T + " (C2's - leg too)", "signal")
    # 5 V.
    w(("U5", "VOUT"), ("U1", "5V.2"), "5V", "orange", 22, _T, _DUP, "pi5v")
    w(("U5", "VOUT"), ("U1", "5V.4"), "5V", "orange", 22, _T, _DUP, "pi5v")
    w(("U5", "GND"), ("U1", "GND.6"), "GND", "black", 22, _T, _DUP, "pi5v")
    w(("U5", "GND"), ("U1", "GND.9"), "GND", "black", 22, _T, _DUP, "pi5v")
    w(("U5", "VOUT"), ("B1", "5V"), "5V", "orange", 22, _T, _B1PWR, "b1_5v")
    w(("U5", "GND"), ("B1", "GND"), "GND", "black", 22, _T, _B1PWR, "b1_5v")
    # Servo rail and bus.
    _EH = "JST EHR-03 housing, SEH-001T-P0.6 crimp, on the shoulder servo"
    w(("U6", "VOUT"), ("JS1", "V+"), "SERVO_5V", "pink", 22, _T, _EH, "servo_arm")
    w(("U6", "VOUT"), ("JS2", "V+"), "SERVO_5V", "pink", 22, _T, _EH, "servo_arm")
    w(("U6", "GND"), ("JS1", "GND"), "GND", "black", 22, _T, _EH, "servo_arm")
    w(("U6", "GND"), ("JS2", "GND"), "GND", "black", 22, _T, _EH, "servo_arm")
    w(("R6", "2"), ("JS1", "DATA"), "SERVO_DATA", "yellow", 26, _PAD, _EH, "signal")
    w(("R6", "2"), ("JS2", "DATA"), "SERVO_DATA", "yellow", 26, _PAD, _EH, "signal")
    # Motoron logic.
    w(("B1", "3V3"), ("MA", "VDD"), "PICO_3V3", "violet", 26, _PAD, _T, "logic")
    w(("MA", "VDD"), ("MB", "VDD"), "PICO_3V3", "violet", 26, _T, _T, "logic")
    w(("U2", "GP20"), ("MA", "SDA"), "I2C0_SDA", "blue", 28, _PAD, _T, "signal")
    w(("MA", "SDA"), ("MB", "SDA"), "I2C0_SDA", "blue", 28, _T, _T, "signal")
    w(("U2", "GP21"), ("MA", "SCL"), "I2C0_SCL", "yellow", 28, _PAD, _T, "signal")
    w(("MA", "SCL"), ("MB", "SCL"), "I2C0_SCL", "yellow", 28, _T, _T, "signal")
    w(("MA", "ERR"), ("U2", "GP15"), "MA_ERR", "grey", 28, _T, _PAD, "signal")
    w(("MB", "ERR"), ("U2", "GP16"), "MB_ERR", "grey", 28, _T, _PAD, "signal")
    # Motors: driver -> Mini-Fit pair -> the motor's own leads.
    for drv, ch, m, crossed in (("MA", 1, "M1", False), ("MA", 2, "M3", False), ("MB", 1, "M2", True), ("MB", 2, "M4", True)):
        j = "JM" + m[1]
        w((drv, f"M{ch}A"), (j, "1"), f"MOT_{m}_A", "white", 20, "the Motoron's output hole", _MF_R, "motor")
        w((drv, f"M{ch}B"), (j, "2"), f"MOT_{m}_B", "brown", 20, "the Motoron's output hole", _MF_R, "motor")
        w((j, "1"), (m, "M-" if crossed else "M+"), f"MOT_{m}_A", "white", 20, _MF_P, _OWN_M, "motor")
        w((j, "2"), (m, "M+" if crossed else "M-"), f"MOT_{m}_B", "brown", 20, _MF_P, _OWN_M, "motor")
    for k in range(1, 5):
        for pin, col in (("GND", "black"), ("VCC", "orange"), ("A", "green"), ("B", "blue")):
            net = "GND" if pin == "GND" else "5V" if pin == "VCC" else f"ENC{k}_{pin}"
            w((f"M{k}", pin), (f"JE{k}", pin), net, col, 26, "the motor's own encoder lead", _XH, "signal")
    # Pi <-> Pico.
    w(("U1", "GPIO14"), ("U2", "GP1"), "PI_TX", "green", 28, _DUP, _PAD, "signal")
    w(("U2", "GP0"), ("U1", "GPIO15"), "PICO_TX", "blue", 28, _PAD, _DUP, "signal")
    w(("U1", "GPIO24"), ("U2", "SWCLK"), "SWCLK", "yellow", 28, _DUP, "the Pico's SWD pad", "signal")
    w(("U1", "GPIO25"), ("U2", "SWDIO"), "SWDIO", "brown", 28, _DUP, "the Pico's SWD pad", "signal")
    w(("U1", "GPIO27"), ("U2", "RUN"), "PICO_RUN", "grey", 28, _DUP, _PAD, "signal")
    # Accessory slot.
    w(("U1", "GPIO4"), ("J_TOP", "TX"), "TOP_TX", "green", 28, _DUP, "soldered under B3", "signal")
    w(("U1", "GPIO5"), ("J_TOP", "RX"), "TOP_RX", "blue", 28, _DUP, "soldered under B3", "signal")
    w(("F2", "2"), ("J_TOP", "5V"), "TOP_5V", "orange", 24, _PAD, "soldered under B3", "top5v")
    w(("B1", "GND"), ("J_TOP", "GND"), "GND", "black", 24, _PAD, "soldered under B3", "top5v")
    w(("B1", "3V3"), ("J_TOP", "3V3"), "PICO_3V3", "violet", 26, _PAD, "soldered under B3", "logic")
    w(("U12", "SDA"), ("J_TOP", "SDA"), "I2C1_SDA", "blue", 28, "MPR121 header hole", "soldered under B3", "signal")
    w(("U12", "SCL"), ("J_TOP", "SCL"), "I2C1_SCL", "yellow", 28, "MPR121 header hole", "soldered under B3", "signal")
    # Touch.
    w(("U1", "GPIO2"), ("U12", "SDA"), "I2C1_SDA", "blue", 28, _DUP, "MPR121 header hole", "signal")
    w(("U1", "GPIO3"), ("U12", "SCL"), "I2C1_SCL", "yellow", 28, _DUP, "MPR121 header hole", "signal")
    w(("U1", "3V3.1"), ("U12", "VIN"), "PI_3V3", "violet", 26, _DUP, "MPR121 header hole", "logic")
    w(("U1", "GND.14"), ("U12", "GND"), "GND", "black", 26, _DUP, "MPR121 header hole", "logic")
    w(("U12", "IRQ"), ("U1", "GPIO17"), "TOUCH_IRQ", "grey", 28, "MPR121 header hole", _DUP, "signal")
    for i in range(3):
        w(("U12", str(i)), (f"TP{i + 1}", "1"), f"TOUCH_E{i}", "white", 28, "MPR121 electrode hole",
          "soldered to the copper tape's tab", "signal")
    # Audio.
    w(("U1", "GPIO18"), ("B1", "H_BCLK"), "PCM_CLK", "green", 28, _DUP, _PAD, "signal")
    w(("U1", "GPIO19"), ("B1", "H_LRCK"), "PCM_FS", "brown", 28, _DUP, _PAD, "signal")
    w(("B1", "H_BCLK"), ("U11", "BCLK"), "PCM_CLK", "green", 28, _PAD, _T, "signal")
    w(("B1", "H_LRCK"), ("U11", "LRC"), "PCM_FS", "brown", 28, _PAD, _T, "signal")
    w(("B1", "H_BCLK"), ("MK1", "BCLK"), "PCM_CLK", "green", 28, _PAD, _T, "signal")
    w(("B1", "H_LRCK"), ("MK1", "LRCL"), "PCM_FS", "brown", 28, _PAD, _T, "signal")
    w(("MK1", "BCLK"), ("MK2", "BCLK"), "PCM_CLK", "green", 28, _T, _T, "signal")
    w(("MK1", "LRCL"), ("MK2", "LRCL"), "PCM_FS", "brown", 28, _T, _T, "signal")
    w(("U1", "GPIO20"), ("MK1", "DOUT"), "PCM_DIN", "white", 28, _DUP, _T, "signal")
    w(("MK1", "DOUT"), ("MK2", "DOUT"), "PCM_DIN", "white", 28, _T, _T, "signal")
    w(("U1", "GPIO21"), ("U11", "DIN"), "PCM_DOUT", "white", 28, _DUP, _T, "signal")
    w(("B1", "5V"), ("U11", "Vin"), "5V", "orange", 22, _PAD, _T, "amp5v")
    w(("B1", "GND"), ("U11", "GND"), "GND", "black", 22, _PAD, _T, "amp5v")
    w(("U11", "SPK+"), ("LS1", "+"), "SPK_P", "red", 22, "the amp's 3.5 mm terminal block", "the speaker's own lead", "speaker")
    w(("U11", "SPK-"), ("LS1", "-"), "SPK_N", "black", 22, "the amp's 3.5 mm terminal block", "the speaker's own lead", "speaker")
    w(("U1", "3V3.17"), ("MK1", "3V"), "PI_3V3", "violet", 28, _DUP, _T, "logic")
    w(("MK1", "3V"), ("MK2", "3V"), "PI_3V3", "violet", 28, _T, _T, "logic")
    w(("U1", "GND.20"), ("MK1", "GND"), "GND", "black", 28, _DUP, _T, "logic")
    w(("MK1", "GND"), ("MK2", "GND"), "GND", "black", 28, _T, _T, "logic")
    # Status LEDs.
    w(("U1", "GPIO22"), ("R8", "1"), "LED_R_DRV", "white", 28, _DUP, "R8's lead, sleeved", "signal")
    w(("U1", "GPIO23"), ("R9", "1"), "LED_G_DRV", "green", 28, _DUP, "R9's lead, sleeved", "signal")
    w(("U1", "GND.25"), ("D3", "K"), "GND", "black", 28, _DUP, "the LED's cathode, sleeved", "signal")
    w(("D3", "K"), ("D4", "K"), "GND", "black", 28, "the LED's cathode", "the LED's cathode, sleeved", "signal")
    # LED ring.
    w(("R7", "2"), ("D1", "DIN"), "LED_DIN", "green", 26, _PAD, "the strip's DIN pad", "signal")
    w(("F3", "2"), ("D1", "5V"), "LED_5V", "orange", 22, _PAD, "the strip's 5V pad", "led5v")
    w(("B1", "GND"), ("D1", "GND"), "GND", "black", 22, _PAD, "the strip's GND pad", "led5v")
    w(("D1", "DOUT"), ("D2", "DIN"), "LED_LINK", "green", 26, "the strip's DOUT pad", "the strip's DIN pad", "signal")
    w(("D1", "5V"), ("D2", "5V"), "LED_5V", "orange", 22, "the strip's 5V pad", "the strip's 5V pad", "led5v")
    w(("D1", "GND"), ("D2", "GND"), "GND", "black", 22, "the strip's GND pad", "the strip's GND pad", "led5v")
    # Power button, power-off.
    w(("U1", "GPIO26"), ("SW1", "OFF"), "PWR_OFF", "grey", 28, _DUP, _T, "signal")
    w(("S1", "1"), ("SW1", "A"), "BTN_A", "white", 28, "B2", _T, "signal")
    w(("S1", "2"), ("SW1", "B"), "BTN_B", "blue", 28, "B2", _T, "signal")
    w(("B1", "5V"), ("R10", "1"), "5V", "orange", 28, _PAD, "B2", "signal")
    w(("S1", "LED-"), ("B1", "GND"), "GND", "black", 28, "B2", _PAD, "signal")
    # Cables (bought, fixed length).
    w(("U1", "CAMDISP1"), ("U8", "DSI"), "DSI", "grey", 0, _FPC, _FPC, "signal", "cable")
    w(("U1", "CAMDISP0"), ("U9", "CSI"), "CSI", "grey", 0, _FPC, _FPC, "signal", "cable")
    w(("U1", "FAN"), ("U10", "FAN"), "FAN", "black", 0, "the Pi's FAN header", "the cooler's own lead", "signal", "cable")
    for k, row in enumerate(W, 1):
        row[0] = f"W{k}"
    return [tuple(x) for x in W]


WIRES = _w()
CABLE_MM = {"DSI": DISPLAY["cable_mm"], "CSI": CAMERA_MOD["cable_mm"], "FAN": 80.0}

# Pins joined by a part's own lead or by the board, not by a wire.
JOINTS: List[Tuple[Tuple[str, str], Tuple[str, str], str]] = []


def _j(a, b, how):
    JOINTS.append((a, b, how))


for _a, _b in ((("U2", "GND.38"), ("B1", "GND")), (("U2", "3V3"), ("B1", "3V3")), (("D5", "A"), ("B1", "5V")),
               (("D5", "K"), ("U2", "VSYS")), (("U3", "VCC"), ("B1", "3V3")), (("U3", "DIR"), ("B1", "3V3")),
               (("U3", "GND"), ("B1", "GND")), (("U3", "OE"), ("B1", "GND")), (("U4", "VCC"), ("B1", "3V3")),
               (("U4", "GND"), ("B1", "GND")), (("U4", "2OE"), ("B1", "GND")), (("U4", "3A"), ("B1", "GND")),
               (("U4", "4A"), ("B1", "GND")), (("U4", "3OE"), ("B1", "3V3")), (("U4", "4OE"), ("B1", "3V3")),
               (("U7", "VCC"), ("B1", "5V")), (("U7", "GND"), ("B1", "GND")), (("U7", "1OE"), ("B1", "GND")),
               (("U7", "2A"), ("B1", "GND")), (("U7", "3A"), ("B1", "GND")), (("U7", "4A"), ("B1", "GND")),
               (("U7", "2OE"), ("B1", "5V")), (("U7", "3OE"), ("B1", "5V")), (("U7", "4OE"), ("B1", "5V")),
               (("C4", "1"), ("U3", "VCC")), (("C4", "2"), ("U3", "GND")), (("C5", "1"), ("U4", "VCC")),
               (("C5", "2"), ("U4", "GND")), (("C6", "1"), ("U7", "VCC")), (("C6", "2"), ("U7", "GND")),
               (("R3", "1"), ("B1", "3V3")), (("R3", "2"), ("U2", "GP20")), (("R4", "1"), ("B1", "3V3")),
               (("R4", "2"), ("U2", "GP21")), (("R5", "1"), ("B1", "3V3")), (("R5", "2"), ("U4", "1Y")),
               (("R6", "1"), ("U4", "1Y")), (("U4", "2A"), ("U4", "1Y")), (("R11", "1"), ("B1", "3V3")),
               (("R11", "2"), ("U4", "1OE")), (("U2", "GP3"), ("U4", "1OE")), (("U2", "GP4"), ("U4", "1A")),
               (("U4", "2Y"), ("U2", "GP5")), (("U2", "GP14"), ("U7", "1A")), (("U7", "1Y"), ("R7", "1")),
               (("F2", "1"), ("B1", "5V")), (("F3", "1"), ("B1", "5V"))):
    _j(_a, _b, "on B1 (solder bridge or a bare-wire link under the board)")
for _k in range(1, 5):
    _j((f"JE{_k}", "GND"), ("B1", "GND"), "on B1")
    _j((f"JE{_k}", "VCC"), ("B1", "5V"), "on B1")
    _j((f"JE{_k}", "A"), ("U3", f"A{2 * _k - 1}"), "on B1")
    _j((f"JE{_k}", "B"), ("U3", f"A{2 * _k}"), "on B1")
for _i in range(1, 9):
    _j(("U3", f"B{_i}"), ("U2", f"GP{5 + _i}"), "on B1")
for _a, _b, _h in ((("C2", "+"), ("SW1", "VOUT"), "C2's + leg in SW1's VOUT hole"),
                   (("C2", "-"), ("SW1", "GND"), "C2's - leg in SW1's GND hole"),
                   (("C3", "+"), ("D1", "5V"), "C3 across the strip's input pads"), (("C3", "-"), ("D1", "GND"), ""),
                   (("MK1", "SEL"), ("MK1", "GND"), "solder bridge SEL-GND: left channel"),
                   (("MK2", "SEL"), ("MK2", "3V"), "solder bridge SEL-3V: right channel"),
                   (("R8", "2"), ("D3", "A"), "R8's lead on the LED's anode, sleeved"),
                   (("R9", "2"), ("D4", "A"), "R9's lead on the LED's anode, sleeved"),
                   (("R10", "2"), ("S1", "LED+"), "R10 on B2"), (("J_TOP", "GND2"), ("J_TOP", "GND"), "on B3")):
    _j(_a, _b, _h)

# Where wires cross between regions (robot frame). The spine (chassis.py) is a box: its rear
# plate z 60-130, a mid rung at z 130-136 with a 14 mm harness hole, its front plate z
# 180-256, a top rung with a 10 mm hole. Front <-> bay: through the spine's open section
# between the mid rung and the front plate; motor leads: up through the deck's slots at
# x +-55 (the front ones through the mid rung's hole); the pack's leads through the deck's
# 10 mm hole at (-35, 0).
PASS = {
    "front": [(20.0, 0.0, 158.0), (0.0, 0.0, 158.0), (-20.0, 0.0, 156.0)],
    "base_front": [(55.0, 0.0, 56.0), (5.0, 0.0, 90.0), (2.0, 0.0, 133.0), (-20.0, 0.0, 145.0)],
    "base_rear": [(-55.0, 0.0, 56.0), (-55.0, 0.0, 62.0)],
    "pack": [(-35.0, 0.0, 56.0), (-35.0, 0.0, 62.0)],
    "top": [(0.0, 0.0, 262.0), (0.0, 0.0, 200.0), (-20.0, 0.0, 156.0)],
    # out of the bay beside the wings' top bridge, past the spine's side face, to the shoulder
    "shoulder_l": [(-12.0, 60.0, 160.0), (-20.0, 50.0, 160.0)],
    "shoulder_r": [(-12.0, -60.0, 160.0), (-20.0, -50.0, 160.0)],
}
SERVICE_MM = 15.0       # strip, joint and a service loop, both ends together
ROUTE_FACTOR = 1.2      # a harness is not a straight line


def _region(ref: str, p) -> str:
    if ref in ("BT1", "J1"):
        return "pack"
    if ref in ("JS1", "JS2"):
        return "shoulder_l" if ref == "JS1" else "shoulder_r"
    x, _, z = p
    if z > sk.TOP_SLOT.z0 - 2.0:
        return "top"
    if z < 58.0:
        return "base_front" if x > 0 else "base_rear"
    if x < BACK_X:
        return "bay"
    return "front"


def route(a_ref: str, a, b_ref: str, b) -> List[Tuple[float, float, float]]:
    """A polyline from a to b through the pass points (an estimate; the integrator routes)."""
    ra, rb = _region(a_ref, a), _region(b_ref, b)
    if ra == rb or {ra, rb} == {"base_front", "base_rear"}:      # the LED ring's link runs under the deck
        return [a, b]
    pa = PASS.get(ra, [])
    pb = PASS.get(rb, [])
    if ra == "bay":
        return [a] + list(reversed(pb)) + [b]
    if rb == "bay":
        return [a] + pa + [b]
    return [a] + pa + list(reversed(pb)) + [b]


def _ceil5(v: float) -> float:
    return 5.0 * math.ceil(v / 5.0 - 1e-9)


def wire_list(r=None, lengths: Optional[Dict[str, float]] = None) -> List[dict]:
    """Every physical wire. `lengths` (mm, by wire id) comes from the 3D routing; until then
    each is ESTIMATED: the polyline through the pass points x 1.2 + 15 mm, up to 5 mm. A
    bought cable (FPC, fan lead) has its own length; its route is reported beside it."""
    pos = positions(r)
    out = []
    for wid, a, b, net, colour, awg, ca, cb, carries, kind in WIRES:
        path = route(a[0], pos[a], b[0], pos[b])
        poly = sum(math.dist(p, q) for p, q in zip(path[:-1], path[1:]))
        est = _ceil5(ROUTE_FACTOR * poly + SERVICE_MM)
        given = (lengths or {}).get(wid)
        if kind == "cable":
            length, src = CABLE_MM[net], "bought"
        elif given is not None:
            length, src = float(given), "routed"
        else:
            length, src = est, "estimate"
        out.append({"id": wid, "from": a, "to": b, "net": net, "colour": colour, "awg": awg, "kind": kind,
                    "connector_from": ca, "connector_to": cb, "carries": carries, "path": path,
                    "straight_mm": round(math.dist(pos[a], pos[b]), 1), "route_mm": round(poly, 1),
                    "length_mm": length, "length_source": src,
                    "ohm": AWG[awg][0] * length / 1e6 if awg in AWG else 0.0})
    return out


# =========================================================================================
# 7. The power budget
# =========================================================================================

CASES = {
    "idle": "idle: screen on, Wi-Fi, servo torque off (arms hang), LEDs dim (10 %), camera off",
    "driving": "driving: cruise on four motors, vision (Pi busy, camera on), LEDs at the cap",
    "spin": "spin turn: four motors at 0.9 A, Pi busy, LEDs at the cap",
    "arms": "gesturing and talking: four servos moving, speaker 0.5 W, Pi busy",
    "servo_stall": "all four servos stalled (limits failed: the servo regulator limits it), otherwise idle",
    "motor_stall": "all four motors stalled at full duty, otherwise driving",
    "everything": "everything at once: motors and servos stalled, LEDs full white, speaker full, accessory 0.5 A",
}
MIX = {"idle": 0.60, "arms": 0.25, "driving": 0.15}      # ASSUMED classroom duty cycle


def _loads(case: str) -> dict:
    """Powers at the regulators' outputs (W) and the motors' state for one case."""
    busy = case not in ("idle", "servo_stall")
    i_pi = PI5["i_busy"] if busy else PI5["i_typ"]
    led = {"idle": 0.10, "everything": 1.0}.get(case, LED_CAP)
    n_led = LED_FRONT + LED_REAR
    p5 = {
        "Pi 5 board": 5.0 * i_pi,
        "display (Pi's 3.3 V, ~90 %)": DISPLAY["p_w"] / 0.9,
        "camera (Pi's 3.3 V)": (CAMERA_MOD["p_w"] / 0.9) if case not in ("idle", "servo_stall") else 0.0,
        "Pico, buffers, Motoron logic": 5.0 * PICO["i_typ"] + 3.3 * 2 * MOTORON["i_logic"] / 0.9,
        "encoders (4)": 5.0 * ENCODER["i"] * 4,
        "LED ring": 5.0 * STRIP["i_led"] * n_led * led,
        "amp + speaker": 5.0 * AMP["iq"] + ({"arms": 0.5, "everything": AMP["p_out_4ohm"]}.get(case, 0.0) / AMP["eta"]),
        "mics, touch, status LEDs, button LED": 3.3 * (2 * MIC["i"] + TOUCH["i"] + 2 * STATUS_LED["i"]) + 5.0 * 0.003,
        "accessory slot": 2.5 if case == "everything" else 0.0,
    }
    i_servo = {"idle": SERVO["i_standby"], "arms": SERVO["i_move"], "servo_stall": SERVO["i_stall"],
               "everything": SERVO["i_stall"]}.get(case, SERVO["i_hold"])      # idle: torque off, arms hang
    i6 = min(SERVO["count"] * i_servo, REG6["i_fault"])
    p6 = {"servos (4)": REG6["vout"] * i6}
    motors = {"idle": ("off", 0.0), "driving": ("run", MOTOR["i_cruise"]), "spin": ("run", MOTOR["i_spin"]),
              "arms": ("off", 0.0), "servo_stall": ("off", 0.0), "motor_stall": ("stall", 0.0),
              "everything": ("stall", 0.0)}[case]
    return {"p5": p5, "p6": p6, "i6": i6, "motors": motors}


def _shared_r(wires) -> Dict[str, float]:
    w = {x["id"]: x["ohm"] for x in wires}
    by = {(x["from"], x["to"]): x["id"] for x in wires}

    def ohm(a, b):
        return w[by[(a, b)]]
    pos_chain = ohm(("BT1", "P+"), ("J1", "1")) + ohm(("J1", "1"), ("F1", "1")) + ohm(("F1", "2"), ("X1", "1")) \
        + ohm(("X1", "2"), ("SW1", "VIN")) + ohm(("SW1", "VOUT"), ("X3", "1"))
    neg_chain = ohm(("BT1", "P-"), ("J1", "2")) + ohm(("J1", "2"), ("X2", "1"))
    parts = PACK["r"] + BMS["r"] + 2 * CONN["XT30"]["r"] + FUSE["r"] + SWITCH["rds"] + 3 * WAGO["r"]
    return {"pack": PACK["r"] + BMS["r"], "parts": parts, "wires": pos_chain + neg_chain,
            "total": parts + pos_chain + neg_chain,
            "to_fuse_short": PACK["r"] + BMS["r"] + 2 * CONN["XT30"]["r"] + FUSE["r"]
            + ohm(("BT1", "P+"), ("J1", "1")) + ohm(("J1", "1"), ("F1", "1")) + neg_chain}


def operating_point(case: str, v_emf: float, r=None, lengths=None) -> dict:
    """Solve the pack (EMF, internal and wiring resistance) against constant-power regulators
    and the motors (resistive at stall, at their measured current when running)."""
    L = _loads(case)
    R = _shared_r(wire_list(r, lengths))["total"]
    p5, p6 = sum(L["p5"].values()), sum(L["p6"].values())
    mode, i_run = L["motors"]
    i_bat, vb = 0.0, v_emf
    for _ in range(300):
        vb = v_emf - i_bat * R
        if mode == "run":
            i_m = MOTOR["count"] * i_run
            i_ch = i_run
        elif mode == "stall":
            i_ch = vb / (MOTOR["r_ohm"] + MOTORON["r_bridge"])
            i_m = MOTOR["count"] * i_ch
        else:
            i_m = i_ch = 0.0
        i_new = (p5 / REG5["eta"] + p6 / REG6["eta"]) / max(vb, 0.5) + i_m + 2 * MOTORON["i_vin_q"] \
            + SWITCH["i_on_per_v"] * vb + 2 * REG5["iq"]
        if abs(i_new - i_bat) < 1e-9:
            break
        i_bat = 0.5 * i_bat + 0.5 * i_new
    i5 = p5 / REG5["vout"]
    return {"case": case, "emf_v": v_emf, "v_bus": vb, "i_bat": i_bat, "p_pack_w": v_emf * i_bat,
            "i5": i5, "p5": p5, "i6": L["i6"], "p6": p6, "i_motor_channel": i_ch, "i_motors_bus": i_m,
            "p5_items": L["p5"], "reg5_in_regulation": vb >= REG5["vout"] + REG5["dropout_ohm"] * i5,
            "reg6_in_regulation": vb >= REG6["vout"] + REG6["dropout_ohm"] * L["i6"],
            "p_loss_reg5": p5 / REG5["eta"] - p5, "p_loss_reg6": p6 / REG6["eta"] - p6,
            "p_motoron": MOTOR["count"] * i_ch ** 2 * MOTORON["r_bridge"]}


def power_budget(r=None, lengths=None) -> dict:
    states = {"full": PACK["v_full"], "nominal": PACK["v_nom"], "end": PACK["v_end"]}
    table = [dict(operating_point(c, v, r, lengths), state=s, label=CASES[c]) for c in CASES for s, v in states.items()]
    energy = PACK["e_wh"] * PACK["usable_frac"]
    runtime = {}
    for c in ("idle", "driving", "arms"):
        op = operating_point(c, PACK["v_nom"], r, lengths)
        runtime[c] = {"p_pack_w": op["p_pack_w"], "hours": energy / op["p_pack_w"]}
    p_mix = sum(f * runtime[c]["p_pack_w"] for c, f in MIX.items())
    runtime["classroom mix"] = {"p_pack_w": p_mix, "hours": energy / p_mix, "mix": MIX}
    sr = _shared_r(wire_list(r, lengths))
    short = PACK["v_full"] / sr["to_fuse_short"]
    charge_h = PACK["c_mah"] / 1000.0 / CHARGER["i_charge"] * 1.15   # ASSUMED +15 % for the CV tail
    return {"table": table, "energy_wh": energy, "runtime": runtime, "short_circuit_a": short, "r_shared": sr,
            "charge_h": charge_h, "charge_w_in": CHARGER["v_cv"] * CHARGER["i_charge"] / CHARGER["eta"]}


def heat(r=None) -> dict:
    """What the bay dissipates and the air temperature rise a stack (chimney) flow through
    the rear vent would give: dT^1.5 = P / (rho c Cd A sqrt(2 g h / T)), with an inlet of the
    same area low on the rear (ASSUMED: the skeleton has no inlet)."""
    out = {}
    a_out = sk.REAR_VENT["size"][0] * sk.REAR_VENT["size"][1] * 1e-6
    a_in = sk.REAR_INLET["size"][0] * sk.REAR_INLET["size"][1] * 1e-6
    a_vent = 1.0 / math.sqrt(1.0 / a_out ** 2 + 1.0 / a_in ** 2)       # the two openings in series
    h = (sk.REAR_VENT["centre"][2] - sk.REAR_INLET["centre"][2]) * 1e-3
    k = 1.2 * 1005.0 * 0.6 * a_vent * math.sqrt(2 * 9.81 * h / 298.0)
    for c in ("idle", "driving", "everything"):
        op = operating_point(c, PACK["v_nom"], r)
        p_pi = op["p5_items"]["Pi 5 board"]
        p = p_pi + op["p_loss_reg5"] + op["p_loss_reg6"] + op["p_motoron"] + 0.3      # 0.3 W: B1, fuse, switch (ASSUMED)
        out[c] = {"p_bay_w": p, "p_pi_w": p_pi, "dT_chimney_K": (p / k) ** (2.0 / 3.0)}
    out["vent_mm2"] = a_out * 1e6
    out["inlet_mm2"] = a_in * 1e6
    out["stack_height_mm"] = h * 1e3
    out["cooler_flow_l_s"] = COOLER["cfm"] * 0.4719
    out["dT_if_cooler_flow_exits_K"] = {c: v["p_bay_w"] / (1.2 * 1005.0 * out["cooler_flow_l_s"] * 1e-3)
                                        for c, v in out.items() if isinstance(v, dict) and "p_bay_w" in v}
    # With the duct: the blower takes outside air through REAR_INLET and pushes it through the bay
    # to REAR_VENT. Its flow at full speed (1.09 CFM) and at half (ASSUMED proportional to speed); the
    # duct's own loss is ignored (ASSUMED small against the blower's).
    out["ducted"] = {pct: {c: out[c]["p_bay_w"] / (1.2 * 1005.0 * out["cooler_flow_l_s"] * pct / 100 * 1e-3)
                           for c in ("idle", "driving", "everything")} for pct in (100, 50)}
    return out


# =========================================================================================
# 8. The contract: parts, solids (above), mounts, jobs, checks
# =========================================================================================

# The electrical steps, in the real build order (skeleton.ASSEMBLY_NOTE: shoulder brackets onto the spine first,
# then the shell and the electronics, the arm modules last; shell.STEPS 1-7). Step 0 is bench work done with
# shell step 1: the parts that live ON a shell section go onto it before that section goes on (until 2026-11-04
# the LED strips were step 6, the charger and button boards step 2 and the touch pads step 5: each after the
# shell part that carries it). Steps 1-5 populate the two carriers ON THE BENCH (1-3 the bay carrier, 4-5 the
# front carrier); each loaded carrier then goes onto the spine as one unit at the end of its last step, screwed
# from inside the spine, before shell step 2 (assemble.py finds every board's path by taking the populated
# carrier apart, 2026-11-04); the harness (6) is plugged before shell step 2 and finished as each section goes
# on; the accessory board (7) goes on with the top socket.
STEPS = {0: "bench, with shell step 1: the LED strips (D1 with C3 at its input, D2) into the bumpers' channels; the charger "
            "(CHG1) and the button board (B2 with S1, R10) screwed onto the lower rear half's M2 inserts; the touch pads "
            "taped inside the upper halves (TP1 right, TP2 and TP3 left); every one with its leads soldered on, left long",
         1: "bench, the bay carrier (its inserts heat-set): the Pi on its standoffs with its Active Cooler, as one unit, "
            "slid in sideways from the left (+y); it sits loose until step 3's screws hold its standoffs",
         2: "bench, the bay carrier's power: the regulators (2 x M2 x 5 each) and the Motorons lowered in from above, "
            "the switch, fuse and the WAGO hubs X1-X3 in from behind (-x), X4 from above",
         3: "bench: the IO board (Pico, logic ICs, headers: populated first) lowered from above, the last 5 mm forward "
            "onto its hooks, 2 x M2.5 x 6 into the lower inserts; then the cooler duct from above (2 x M2 x 10 into the "
            "lip's inserts). Then the loaded bay carrier onto the spine from behind (-x), after the shoulder brackets: "
            "5 x M3 x 12 from inside the spine into its inserts, then the chassis's 4 x M2.5 x 16 from inside the spine "
            "through the carrier into the Pi's standoffs",
         4: "bench, the front carrier (its inserts heat-set): display from the front and camera up from below, in "
            "either order (4 x M2 x 5), LEDs, mics",
         5: "bench: the chest, from the front: speaker (4 x M3 x 8), amp and MPR121 (2 x M2 x 6 each). Then the loaded "
            "front carrier onto the spine from the front (+x): 4 x M3 x 12 from inside the spine into its inserts",
         6: "harness: pack, motors, encoders, servo plugs, FPCs, all before shell step 2; then the shell-mounted parts' "
            "leads as their sections go on: the LED strips', charger's and button's with the lower halves (shell step 2), "
            "the touch pads' with the upper halves (shell step 4)",
         7: "accessory board on the top socket's floor (VHB), its wires up through the floor's slot: with shell step 5"}
JOINS_SHELL_STEP = {0: "shell step 1 (bench)", 1: "before shell step 2", 2: "before shell step 2", 3: "before shell step 2",
                    4: "before shell step 2", 5: "before shell step 2", 6: "before shell step 2; finished at shell steps 2 and 4",
                    7: "shell step 5"}
_STEP_OF = {"U1": 1, "U10": 1, "B1": 3, "U2": 3, "U3": 3, "U4": 3, "U7": 3, "U8": 4, "U9": 4, "MK1": 4, "MK2": 4, "D3": 4,
            "D4": 4, "R8": 4, "R9": 4, "U11": 5, "U12": 5, "LS1": 5, "TP1": 0, "TP2": 0, "TP3": 0, "B3": 7, "J_TOP": 7,
            "D1": 0, "D2": 0, "C3": 0, "CHG1": 0, "B2": 0, "S1": 0, "R10": 0,
            "BT1": 6, "J1": 6, "M1": 6, "M2": 6, "M3": 6, "M4": 6, "JS1": 6, "JS2": 6}
PETG_G_MM3 = 1.27e-3
PRINT_FILL = 0.6        # ASSUMED effective density of a printed part (walls + infill)
_MASS = {"BT1": PACK["g"], "J1": 3.0, "F1": FUSE["g"], "SW1": SWITCH["g"], "S1": BUTTON["g"], "CHG1": 3.0,
         "X1": WAGO["g"] * 0.7, "X2": WAGO["g"], "X3": WAGO["g"], "X4": WAGO["g"], "C2": 2.0, "U5": REG5["g"], "U6": REG6["g"],
         "MA": MOTORON["g"], "MB": MOTORON["g"], "U1": PI5["g"], "U10": COOLER["g"], "B1": 9.0, "U2": PICO["g"],
         "U3": 2.5, "U4": 2.0, "U7": 2.0, "U8": DISPLAY["g"], "U9": CAMERA_MOD["g"], "MK1": MIC["g"], "MK2": MIC["g"],
         "U11": AMP["g"], "LS1": SPEAKER["g"], "U12": TOUCH["g"], "D1": STRIP["g_per_m"] * SIZES["D1"][0] / 1000,
         "D2": STRIP["g_per_m"] * SIZES["D2"][0] / 1000, "B2": 0.5, "B3": 0.8, "J_TOP": 0.8,
         "JM1": 2.0, "JM2": 2.0, "JM3": 2.0, "JM4": 2.0, "JE1": 0.5, "JE2": 0.5, "JE3": 0.5, "JE4": 0.5}
# Masses of small parts not in _MASS: 0.3 g (ASSUMED). Motors (M1-M4) are the chassis's mass.


def _bbox_centre(tris: np.ndarray) -> Tuple[float, float, float]:
    v = tris.reshape(-1, 3)
    c = (v.min(0) + v.max(0)) / 2
    return (round(float(c[0]), 2), round(float(c[1]), 2), round(float(c[2]), 2))


def _box_volume(boxes) -> float:
    return sum((hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]) for lo, hi in boxes)


def wire_mass_g(r=None, lengths=None) -> float:
    g = 0.0
    for w in wire_list(r, lengths):
        if w["awg"] in AWG_D and "own" not in w["connector_from"]:
            g += math.pi * (AWG_D[w["awg"]] / 2) ** 2 * w["length_mm"] * 8.96e-3 * 1.6    # copper x 1.6 for insulation
    return g


def parts(r=None) -> List[dict]:
    """Every electrical part: {label, step, kind, process, material, mass_g, centre_mm, source, ...}."""
    sol = solids(r)
    pos = positions(r)
    out = []
    for ref, c in components().items():
        if ref.startswith("M") and ref[1:].isdigit():
            continue                                   # the chassis's gearmotors
        label = f"el_{ref}"
        if ref == "BT1":     # no solid of ours (the chassis places the pack): its mass at the pack's volume, not its leads
            b = PACK_VOLUME["box"]
            centre = ((b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, (b.z0 + b.z1) / 2)
        elif label in sol:
            centre = _bbox_centre(sol[label])
        else:
            pts = [v for (rr, _), v in pos.items() if rr == ref]
            centre = tuple(round(float(x), 2) for x in np.mean(pts, axis=0)) if pts else (0.0, 0.0, 0.0)
        out.append({"label": label, "ref": ref, "step": _STEP_OF.get(ref, 2 if c["where"] == "bay" else 6),
                    "kind": "bought", "process": "bought", "material": "-", "mass_g": round(_MASS.get(ref, 0.3), 2),
                    "centre_mm": centre, "source": c["source"], "spec": c["part"], "where": c["where"],
                    "solid": label in sol, "interface_part": ref in INTERFACE_PARTS})
    out.append({"label": "el_pi_standoffs", "ref": "-", "step": 1, "kind": "bought", "process": "bought", "material": "brass",
                "mass_g": 4 * STANDOFF["g"], "centre_mm": _bbox_centre(sol["el_pi_standoffs"]), "source": STANDOFF["spec"],
                "spec": "4 x " + STANDOFF["spec"], "where": "bay", "solid": True, "interface_part": False})
    shapes = _carrier_shapes(_p(r))
    for name in _carrier_boxes(r):
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(shapes[name], props)
        c = props.CentreOfMass()
        out.append({"label": f"el_{name}", "ref": "-", "step": {"bay_carrier": 1, "front_carrier": 4, "cooler_duct": 3}[name], "kind": "made",
                    "process": "print", "material": "PETG",
                    "mass_g": round(float(props.Mass()) * PETG_G_MM3 * PRINT_FILL, 1),
                    "centre_mm": (round(c.X(), 2), round(c.Y(), 2), round(c.Z(), 2)), "source": "this file (printed on the SV08 twin)",
                    "spec": f"printed PETG {name.replace('_', ' ')} with its fasteners' holes (FASTENERS in PLACEMENT.md)",
                    "where": str(CARRIER_ZONE[name]), "solid": True, "interface_part": False})
    ftris = _fastener_tris(_p(r))
    for f in fasteners(r):
        if f["owner"] != "electrical":
            continue
        out.append({"label": f["label"], "ref": "-", "step": f["step"], "kind": "bought", "process": "bought",
                    "material": "steel", "mass_g": round(fastener_mass_g(f), 2), "centre_mm": _bbox_centre(ftris[f["label"]]),
                    "source": SCREW_SOURCE, "spec": f"{f['size']} x {f['length']} ISO 4762 into {f['into']}",
                    "where": f["carrier"], "solid": True, "interface_part": f["interface"]})
        if f["insert"]:
            ins = INSERTS[f["insert"]]
            lab = f["label"].replace("el_screw_", "el_insert_")
            out.append({"label": lab, "ref": "-", "step": f["step"], "kind": "bought", "process": "bought",
                        "material": "brass", "mass_g": ins["g"], "centre_mm": _bbox_centre(ftris[lab]), "source": ins["source"],
                        "spec": ins["spec"], "where": f["carrier"], "solid": True, "interface_part": False})
    out.append({"label": "el_harness", "ref": "-", "step": 6, "kind": "bought", "process": "bought", "material": "copper/PVC",
                "mass_g": round(wire_mass_g(r), 1), "centre_mm": (-30.0, 0.0, 120.0), "source": AWG_SOURCE,
                "spec": f"{len(WIRES)} wires (WIRES.md)", "where": "everywhere", "solid": False, "interface_part": False})
    return out


def mounts(r=None) -> List[Mount]:
    """The chassis's electrical mounting points this design uses (by the chassis's names in
    chassis.electrical_mounts(): the thread is in OUR standoff or heat-set insert, the screw
    comes from inside the spine)."""
    out = []
    sx, sz = PI5["holes"]
    for i, (dy, dz) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        out.append(Mount(f"bay_pi5_{i}", (BACK_X, dy * sx / 2, 98.0 + dz * sz / 2), (-1.0, 0.0, 0.0), "M2.5",
                         "electrical", "chassis"))
    for i, y in enumerate((-40.0, -20.0, 0.0, 20.0, 40.0)):
        out.append(Mount(f"bay_row_{i}", (BACK_X, y, 64.0), (-1.0, 0.0, 0.0), "M3", "electrical", "chassis"))
    grid_y, grid_z = (-40.0, -20.0, 0.0, 20.0, 40.0), (190.0, 210.0, 230.0, 250.0)
    used = {(-40.0, 190.0), (40.0, 190.0), (-40.0, 250.0), (40.0, 250.0)}
    for i, (y, z) in enumerate((y, z) for z in grid_z for y in grid_y):
        if (y, z) in used:
            out.append(Mount(f"head_grid_{i}", (FRONT_X, y, z), (1.0, 0.0, 0.0), "M3", "electrical", "chassis"))
    return out


def _stl_ascii(name: str, tris: np.ndarray) -> str:
    lines = [f"solid {name}"]
    for t in tris:
        nrm = np.cross(t[1] - t[0], t[2] - t[0])
        L = np.linalg.norm(nrm)
        nrm = nrm / L if L > 0 else nrm
        lines.append(f"  facet normal {nrm[0]:.6e} {nrm[1]:.6e} {nrm[2]:.6e}\n    outer loop")
        for v in t:
            lines.append(f"      vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
        lines.append("    endloop\n  endfacet")
    lines.append(f"endsolid {name}")
    return "\n".join(lines) + "\n"


_PRINT_ROT = {"bay_carrier": np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]]),   # back plate on the bed
              "front_carrier": np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]),
              "cooler_duct": np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])}      # hood's open side down


def print_orientation(name: str, tris: np.ndarray) -> np.ndarray:
    """The carrier laid with its plate on the bed, its footprint centred on the origin."""
    t = tris @ _PRINT_ROT[name].T
    v = t.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    return t - np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])


def jobs(r=None) -> Dict[str, str]:
    """Made part label -> ASCII STL (mm) in print orientation, for the slicer."""
    sol = solids(r)
    return {f"el_{n}": _stl_ascii(f"el_{n}", print_orientation(n, sol[f"el_{n}"])) for n in _carrier_boxes(r)}


# ---- checks -------------------------------------------------------------------------------------

MARGIN = 0.8


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


def _node_map() -> Dict[Tuple[str, str], Tuple[str, str]]:
    m = {}
    for group in INTERNAL:
        for p in group:
            m[p] = group[0]
    return m


def _zone_boxes(zone) -> List[Box]:
    names = zone if isinstance(zone, tuple) else (zone,)
    return [b for n, b in sk.ZONES["electrical"] if n in names]


def _in_boxes(lo, hi, boxes, tol=1e-6) -> bool:
    return any(b.x0 - tol <= lo[0] and hi[0] <= b.x1 + tol and b.y0 - tol <= lo[1] and hi[1] <= b.y1 + tol
               and b.z0 - tol <= lo[2] and hi[2] <= b.z1 + tol for b in boxes)


def clearance(r=None) -> dict:
    """FCL across our own solids: pairs that touch (a 0.05 mm nudge frees them) are seats;
    anything else is interference."""
    import trimesh
    sol = solids(r)
    meshes = {k: trimesh.Trimesh(**trimesh.triangles.to_kwargs(t)) for k, t in sol.items()}
    cm = trimesh.collision.CollisionManager()
    for k, m in meshes.items():
        cm.add_object(k, m)
    _, names = cm.in_collision_internal(return_names=True)
    touching, interference = [], []
    for a, b in sorted(tuple(sorted(p)) for p in names):
        one = trimesh.collision.CollisionManager()
        one.add_object(b, meshes[b])
        freed = None
        for axis in range(3):
            for s in (1, -1):
                T = np.eye(4)
                T[axis, 3] = s * 0.05
                if not one.in_collision_single(meshes[a], transform=T):
                    freed = "+-"[s < 0] + "xyz"[axis]
                    break
            if freed:
                break
        (touching if freed else interference).append({"a": a, "b": b, "freed_by": freed})
    return {"solids": len(sol), "touching": touching, "interference": interference}


def rules(r=None, lengths=None, fcl: bool = True) -> Dict[str, dict]:
    """Every rule, {ok, value, limit, why}."""
    comps, nets = components(), netlist()
    out: Dict[str, dict] = {}

    def rule(name, ok, value=None, limit=None, why=""):
        out[name] = {"ok": bool(ok), "value": value, "limit": limit, "why": why}

    every_pin = {(ref, p) for ref, c in comps.items() for p in c["pins"]}
    seen = defaultdict(list)
    for net, pins in nets.items():
        for p in pins:
            seen[p].append(net)
    rule("every_net_has_two_pins", all(len(v) >= 2 for v in nets.values()), min(len(v) for v in nets.values()), 2)
    rule("no_pin_on_two_nets", all(len(v) == 1 for v in seen.values()), [p for p, v in seen.items() if len(v) > 1], [])
    rule("every_net_pin_exists", set(seen) <= every_pin, sorted(set(seen) - every_pin), [])
    rule("every_pin_on_a_net_or_no_connect", every_pin == set(seen) | set(NO_CONNECT) and not set(seen) & set(NO_CONNECT),
         sorted(every_pin - set(seen) - set(NO_CONNECT)) + sorted(set(seen) & set(NO_CONNECT)), [])
    wires = wire_list(r, lengths)
    net_of = {p: v[0] for p, v in seen.items()}
    bad = [w["id"] for w in wires if not (net_of.get(w["from"]) == net_of.get(w["to"]) == w["net"])]
    bad += [f"{a}-{b}" for a, b, _ in JOINTS if net_of.get(a) != net_of.get(b) or a not in net_of]
    rule("wires_join_pins_of_their_net", not bad, bad, [])
    nm = _node_map()
    split, loops = [], 0
    for net, pins in nets.items():
        nodes = {nm.get(p, p) for p in pins}
        edges = {tuple(sorted((nm.get(w["from"], w["from"]), nm.get(w["to"], w["to"])))) for w in wires if w["net"] == net}
        edges |= {tuple(sorted((nm.get(a, a), nm.get(b, b)))) for a, b, _ in JOINTS if net_of.get(a) == net}
        roots, n_loops = _union(edges, nodes)
        if len(roots) != 1:
            split.append(net)
        loops += n_loops
    rule("every_net_physically_connected", not split, split, [])
    rule("no_loops_in_any_net", loops == 0, loops, 0, "parallel conductors between the same two points count once")
    pos = positions(r)
    missing = sorted({e for w in wires for e in (w["from"], w["to"]) if e not in pos})
    rule("every_wired_pin_has_a_position", not missing, missing, [])
    # Pin assignment.
    pi_used = [p[1] for p in seen if p[0] == "U1" and p[1].startswith("GPIO")]
    rule("pi_gpio_assignment_matches_table", sorted(pi_used) == sorted(PI_USE), sorted(set(pi_used) ^ set(PI_USE)), [])
    pico_used = [p[1] for p in seen if p[0] == "U2" and p[1].startswith("GP")]
    rule("pico_gpio_assignment_matches_table", sorted(pico_used) == sorted(PICO_USE), sorted(set(pico_used) ^ set(PICO_USE)), [])
    rule("pico_uart_pins_are_uart_pins", 0 in PICO_FUNCS["UART0_TX"] and 1 in PICO_FUNCS["UART0_RX"]
         and 4 in PICO_FUNCS["UART1_TX"] and 5 in PICO_FUNCS["UART1_RX"], "GP0/GP1 UART0, GP4/GP5 UART1", "Pico pinout")
    rule("pico_i2c_pins_are_i2c0", 20 in PICO_FUNCS["I2C0_SDA"] and 21 in PICO_FUNCS["I2C0_SCL"], "GP20/GP21", "Pico pinout")
    enc_ok = all(("U2", f"GP{6 + 2 * k}") in nets[f"ENC{k + 1}_A_3V3"] and ("U2", f"GP{7 + 2 * k}") in nets[f"ENC{k + 1}_B_3V3"]
                 for k in range(4))
    rule("encoders_on_consecutive_pins", enc_ok, "A on GP6+2k, B on GP7+2k", "the PIO quadrature program reads two adjacent pins")
    rule("pio_state_machines_enough", 4 + 1 <= 8, 5, 8, "RP2040: 2 PIO x 4 state machines")
    rule("pi_functions_on_their_pins", all(p in PI_USE and PI_USE[p].lower().startswith(f.lower()[:3])
                                           for f, ps in PI_FUNCS.items() for p in ps), sorted(PI_FUNCS), "Pi 5 alt functions")
    # Levels: no 5 V output on any Pico or Pi GPIO net.
    five_v_out = {(f"M{k}", c) for k in range(1, 5) for c in ("A", "B")} | {("U7", "1Y"), ("D1", "DOUT")}
    exposed = [n for n, ps in nets.items() if any(p[0] in ("U1", "U2") and ("GP" in p[1]) for p in ps)
               and any(p in five_v_out for p in ps)]
    rule("no_5v_output_on_a_3v3_gpio", not exposed, exposed, [], "encoders pass the LVC245 (3.3 V VCC, 5.5 V tolerant inputs)")
    rule("lvc_inputs_tolerate_encoder_level", ENCODER["v_supply"] <= LOGIC_IC["SN74LVC245A"]["vi_max"], ENCODER["v_supply"],
         LOGIC_IC["SN74LVC245A"]["vi_max"])
    rule("encoder_supply_in_range", ENCODER["v"][0] <= REG5["vout"] * (1 - REG5["tol"]) and REG5["vout"] * (1 + REG5["tol"]) <= ENCODER["v"][1],
         REG5["vout"], ENCODER["v"])
    rule("ws2812_data_level_from_ahct", LOGIC_IC["SN74AHCT125"]["voh_min"] >= 0.7 * 5.0 and PICO["io_v"] * 0.8 >= LOGIC_IC["SN74AHCT125"]["vih"],
         LOGIC_IC["SN74AHCT125"]["voh_min"], 3.5, "WS2812B VIH 0.7 VDD; AHCT VIH 2.0 V below the Pico's VOH")
    rule("servo_buffer_off_through_reset", ("R11", "2") in nets["SERVO_DIR"] and ("R11", "1") in nets["PICO_3V3"],
         "R11 pulls 1OE high", "the Pico's GPIOs float at reset")
    # Supplies.
    pb = power_budget(r, lengths)
    vmax = PACK["v_full"]
    rule("pack_within_switch", SWITCH["v_min"] <= PACK["v_end"] and vmax <= SWITCH["v_max"], vmax, (SWITCH["v_min"], SWITCH["v_max"]))
    rule("pack_within_motoron", MOTORON["vin_min"] <= PACK["v_end"] - 1.0 and vmax <= MOTORON["vin_max"], vmax,
         (MOTORON["vin_min"], MOTORON["vin_max"]))
    rule("pack_within_fuse_voltage", vmax <= FUSE["v_max"], vmax, FUSE["v_max"])
    rule("pi_gets_its_5a_supply", REG5["i_max"] >= PI5["i_psu"], REG5["i_max"], PI5["i_psu"],
         "the Pi 5 wants a 5 A supply for the 1.6 A USB budget (EEPROM PSU_MAX_CURRENT=5000)")
    kept = [op for op in pb["table"] if op["case"] not in ("everything", "motor_stall")]
    low5 = [(op["case"], op["state"], round(op["v_bus"], 2)) for op in kept if not op["reg5_in_regulation"]]
    rule("reg5_in_regulation_except_stalls", not low5, low5, "VSW >= 5 V + dropout")
    worst5 = max(op["i5"] for op in pb["table"])
    rule("reg5_load_within_rating", worst5 <= MARGIN * REG5["i_max"], round(worst5, 2), MARGIN * REG5["i_max"])
    run6 = max(op["i6"] for op in pb["table"] if op["case"] in ("idle", "driving", "arms", "spin"))
    rule("reg6_running_load_within_rating", run6 <= MARGIN * REG6["i_max"], round(run6, 2), MARGIN * REG6["i_max"])
    v6 = (REG6["vout"] * (1 - REG6["tol"]), REG6["vout"] * (1 + REG6["tol"]))
    rule("servo_rail_within_servo_limits", SERVO["v_abs"][0] <= v6[0] and v6[1] <= SERVO["v_abs"][1],
         tuple(round(v, 2) for v in v6), SERVO["v_abs"], "XC330: 3.7-6.0 V")
    rule("servo_rail_at_recommended_voltage", abs(REG6["vout"] - SERVO["v_rec"]) < 1e-9, REG6["vout"], SERVO["v_rec"])
    rule("pack_never_on_the_servos", PACK["v_end"] > SERVO["v_abs"][1] or ("U6", "VOUT") in nets["SERVO_5V"],
         "servos on U6 only", f"pack {PACK['v_end']}-{PACK['v_full']} V > {SERVO['v_abs'][1]} V")
    fault = SERVO["count"] * SERVO["i_stall"]
    rule("servo_fault_limited_by_its_regulator", fault > REG6["i_max"] and REG6["i_fault"] < fault, fault,
         REG6["i_fault"], "4 servos stalled with their current limits off: U6's over-current protection holds it")
    rule("servo_bus_logic_compatible", True, "3.3 V (SN74LVC125A at 3.3 V)", SERVO["logic"])
    vsys = (REG5["vout"] * (1 + REG5["tol"]) - 0.20, REG5["vout"] * (1 - REG5["tol"]) - 0.45)
    rule("pico_vsys_in_range", PICO["vsys"][0] <= vsys[1] and vsys[0] <= PICO["vsys"][1], tuple(round(v, 2) for v in vsys), PICO["vsys"])
    rule("led_strip_supply_ok", REG5["vout"] * (1 + REG5["tol"]) <= STRIP["v_max"], REG5["vout"] * (1 + REG5["tol"]), STRIP["v_max"])
    rule("amp_supply_ok", AMP["v"][0] <= REG5["vout"] * (1 - REG5["tol"]) and REG5["vout"] * (1 + REG5["tol"]) <= AMP["v"][1],
         REG5["vout"], AMP["v"])
    rule("mics_on_3v3", MIC["v"][0] <= 3.3 <= MIC["v"][1], 3.3, MIC["v"])
    i_pico3 = 2 * MOTORON["i_logic"] + 0.05 + 0.010 + 3.3 / 4700 * 2      # Motorons, accessory 3.3 V budget, ICs, pull-ups
    rule("pico_3v3_load_within_300ma", i_pico3 <= PICO["i_3v3_max"], round(i_pico3, 3), PICO["i_3v3_max"],
         "the accessory slot's 3.3 V is budgeted at 50 mA")
    rule("charger_cv_is_pack_full", abs(CHARGER["v_cv"] - PACK["v_full"]) < 1e-9, CHARGER["v_cv"], PACK["v_full"])
    rule("charge_current_within_cell_standard", CHARGER["i_charge"] / PACK["parallel"] <= CELL["i_charge_std"],
         CHARGER["i_charge"] / PACK["parallel"], CELL["i_charge_std"], "per cell")
    po = PACK["outline_mm"]
    vb = PACK_VOLUME["box"]
    rule("pack_fits_its_volume", po[0] <= vb.x1 - vb.x0 + 1e-9 and po[1] <= vb.y1 - vb.y0 + 1e-9 and po[2] <= vb.z1 - vb.z0 + 1e-9,
         po, (vb.x1 - vb.x0, vb.y1 - vb.y0, vb.z1 - vb.z0))
    rule("bms_holds_all_stalled", BMS["i_cont"] >= max(op["i_bat"] for op in pb["table"]),
         round(max(op["i_bat"] for op in pb["table"]), 2), BMS["i_cont"])
    rule("charger_within_usb_c_5v_3a", pb["charge_w_in"] <= CHARGER["p_max_w"], round(pb["charge_w_in"], 2), CHARGER["p_max_w"])
    # Currents.
    T = {(op["case"], op["state"]): op for op in pb["table"]}
    busy = max(T[(c, "end")]["i_bat"] for c in ("driving", "spin", "arms"))
    combo = operating_point("spin", PACK["v_end"], r, lengths)["i_bat"] + operating_point("arms", PACK["v_end"], r, lengths)["i_bat"] \
        - operating_point("idle", PACK["v_end"], r, lengths)["i_bat"]
    rule("fuse_holds_the_busiest_steady_use", combo <= MARGIN * FUSE["i_rated"], round(combo, 2), MARGIN * FUSE["i_rated"],
         "spin turn + gesturing + talking together, empty pack (highest current)")
    rule("switch_cool_in_any_single_use", busy <= SWITCH["i_55c"], round(busy, 2), SWITCH["i_55c"])
    rule("switch_within_150c_figure_at_busiest", combo <= SWITCH["i_150c"], round(combo, 2), SWITCH["i_150c"])
    ev = T[("everything", "full")]
    rule("pack_peak_within_cell_rating", ev["i_bat"] <= PACK["parallel"] * CELL["i_dis_max"], round(ev["i_bat"], 2),
         PACK["parallel"] * CELL["i_dis_max"])
    rule("bms_holds_the_fuse_rating", BMS["i_cont"] >= FUSE["i_rated"], BMS["i_cont"], FUSE["i_rated"])
    rule("fuse_clears_a_short_fast", pb["short_circuit_a"] >= 3.5 * FUSE["i_rated"], round(pb["short_circuit_a"], 1),
         3.5 * FUSE["i_rated"], "MINI blade fuses open in well under a second at 350 % (ASSUMED class figure)")
    rule("fuse_interrupts_the_short", pb["short_circuit_a"] <= FUSE["i_interrupt"], round(pb["short_circuit_a"], 1), FUSE["i_interrupt"])
    stall_ch = max(max(op["i_motor_channel"] for op in pb["table"] if op["case"] == "motor_stall"),
                   PACK["v_full"] / MOTOR["r_ohm"])          # the chassis's 3.5 A at 8.4 V, no sag
    rule("motoron_survives_a_stall", stall_ch <= MOTORON["i_peak"], round(stall_ch, 2), MOTORON["i_peak"],
         "the chassis: every channel must survive 3.5 A")
    i_spin = max(MOTOR["i_spin"], MOTOR["i_spin_measured_worst"])
    rule("motoron_running_within_continuous", i_spin <= MARGIN * MOTORON["i_cont"], i_spin, MARGIN * MOTORON["i_cont"],
         "the worst of the chassis's spin-turn figure and the Newton run's worst motor")
    rule("connector_xt30_holds_the_fuse", CONN["XT30"]["i"] >= FUSE["i_rated"], CONN["XT30"]["i"], FUSE["i_rated"])
    rule("motor_connector_holds_a_stall", stall_ch <= CONN["MINIFIT"]["i"], round(stall_ch, 2), CONN["MINIFIT"]["i"])
    pi_pin = PI5["i_psu"] / 2
    rule("pi_5v_pins_within_crimp", pi_pin <= CONN["DUPONT"]["i"], pi_pin, CONN["DUPONT"]["i"], "5 A on two header pins")
    arm_move = 2 * SERVO["i_move"]
    rule("servo_plug_holds_moving_arm", arm_move <= SERVO_PLUG["i"], arm_move, SERVO_PLUG["i"])
    # Wire ampacity: what each can carry, capped by its protection.
    i_led = STRIP["i_led"] * (LED_FRONT + LED_REAR)
    carry = {"pack": FUSE["i_rated"], "charge": CHARGER["i_charge"] * 1.2, "reg5_in": ev["p5"] / REG5["eta"] / PACK["v_end"],
             "reg6_in": REG6["i_max"] * REG6["vout"] / REG6["eta"] / PACK["v_end"], "motoron_in": min(2 * stall_ch, FUSE["i_rated"]),
             "motors_total": FUSE["i_rated"], "motor": stall_ch, "pi5v": PI5["i_psu"] / 2,
             "b1_5v": i_led + AMP["p_out_4ohm"] / AMP["eta"] / 5.0 + 0.5 + 0.2, "servo_arm": min(2 * SERVO["i_stall"], REG6["i_fault"]),
             "amp5v": AMP["p_out_4ohm"] / AMP["eta"] / 5.0, "speaker": math.sqrt(AMP["p_out_4ohm"] / SPEAKER["ohm"]),
             "led5v": i_led, "top5v": 2 * 0.5, "logic": 0.4, "signal": 0.02}
    over = [(w["id"], round(carry[w["carries"]], 2), AWG[w["awg"]][1]) for w in wires
            if w["awg"] in AWG and carry[w["carries"]] > AWG[w["awg"]][1]]
    rule("wires_within_ampacity", not over, over, "Powerstream chassis wiring", AWG_SOURCE)
    rule("wago_takes_the_gauges", all(WAGO["awg"][0] >= w["awg"] >= WAGO["awg"][1] for w in wires
                                      if "WAGO" in w["connector_from"] + w["connector_to"]),
         sorted({w["awg"] for w in wires if "WAGO" in w["connector_from"] + w["connector_to"]}), WAGO["awg"])
    # Placement.
    sol = solids(r)
    outside = []
    for ref, shells in _board_models(r).items():
        if ref in INTERFACE_PARTS:
            continue
        zone = placements(r)[ref].zone if ref in placements(r) else _zone_for(ref)
        for s in shells:
            v = s.reshape(-1, 3)
            if not _in_boxes(v.min(0), v.max(0), _zone_boxes(zone)):
                outside.append(ref)
                break
    for name, boxes in _carrier_boxes(r).items():
        zb = _zone_boxes(CARRIER_ZONE[name])
        if not all(_in_boxes(lo, hi, zb) for lo, hi in boxes):
            outside.append(name)
    so = np.vstack(_standoffs(r)).reshape(-1, 3)
    if not _in_boxes(so.min(0), so.max(0), _zone_boxes("bay")):
        outside.append("pi_standoffs")
    ftris = _fastener_tris(_p(r))
    for f in fasteners(r):
        if f["owner"] != "electrical" or f["interface"]:
            continue
        for lab in (f["label"], f["label"].replace("el_screw_", "el_insert_")):
            if lab in ftris:
                v = ftris[lab].reshape(-1, 3)
                if not _in_boxes(v.min(0), v.max(0), _zone_boxes(CARRIER_ZONE[f["carrier"]])):
                    outside.append(lab)
    # Every fastener has its hole: a screw's thread is held by an insert, a pilot or a standoff, and the
    # carrier it enters has the hole cut (checked on the solid: no screw or insert overlaps it, by FCL).
    holes = _carrier_holes(r)
    unholed = [f["label"] for f in fasteners(r)
               if not any(np.allclose(m, f["mouth"]) for m, *_ in holes.get(f["carrier"], []))]
    rule("every_fastener_has_its_hole", not unholed, unholed, [],
         f"{len(fasteners(r))} fasteners: " + ", ".join(sorted({f"{f['size']} x {f['length']} ({f['holder']})"
                                                               for f in fasteners(r)})))
    rule("solids_inside_electrical_zones", not outside, outside, "skeleton.ZONES['electrical']",
         "interface parts excepted: " + ", ".join(sorted(INTERFACE_PARTS)))
    try:
        import trimesh
        nonwt = [k for k, t in sol.items() if not all(m.is_watertight for m in
                                                       trimesh.Trimesh(**trimesh.triangles.to_kwargs(t)).split(only_watertight=False))]
    except Exception as e:             # pragma: no cover
        nonwt = [f"trimesh unavailable: {e}"]
    rule("solids_closed", not nonwt, nonwt, [])
    if fcl:
        cl = clearance(r)
        rule("no_interference_between_our_parts", not cl["interference"], cl["interference"], [],
             f"{len(cl['touching'])} touching pairs (seats) allowed")
    disp = sol["el_U8"].reshape(-1, 3)
    act_c = ((disp[:, 1].min() + disp[:, 1].max()) / 2, (disp[:, 2].min() + disp[:, 2].max()) / 2)
    rule("screen_active_area_centred_in_face_window", abs(act_c[0] - FW["centre"][1]) < 0.5 and abs(act_c[1] - FW["centre"][2]) < 0.5,
         tuple(round(v, 2) for v in act_c), FW["centre"][1:], "the active area is centred on the module outline (ASSUMED)")
    rule("screen_active_area_inside_face_window", DISPLAY["active_mm"][0] <= FW["size"][0] and DISPLAY["active_mm"][1] <= FW["size"][1],
         DISPLAY["active_mm"], FW["size"])
    cam = placements(r)["U9"]
    lens = cam.at(12.5, CAMERA_MOD["lens_from_far_edge"], 8.9)
    rule("camera_lens_on_the_camera_axis", abs(lens[1] - CH[1]) < 0.01 and abs(lens[2] - CH[2]) < 0.01
         and np.allclose(cam.R[:, 2], sk.CAMERA["axis"]), tuple(round(v, 2) for v in lens), CH,
         "lens top 5 mm behind the shell's face (the shell's 2.5 mm wall and 1 mm gap, and 1.5 mm to spare)")
    ports = [placements(r)[m].at(8.35, 6.35, 1.8) for m in ("MK1", "MK2")]
    mic_err = max(min(math.dist(p[:2], h[:2]) for h in sk.MIC_HOLES) for p in ports)
    rule("mic_ports_under_mic_holes", mic_err < 0.01, round(mic_err, 3), 0.01, "MK1/MK2 under the outer holes (y +-22)")
    gr = sk.SPEAKER_GRILLE
    lsv = sol["el_LS1"].reshape(-1, 3)
    rule("speaker_covers_the_grille", lsv[:, 1].min() <= gr["centre"][1] - gr["size"][0] / 2 and lsv[:, 1].max() >= gr["centre"][1] + gr["size"][0] / 2
         and lsv[:, 2].min() <= gr["centre"][2] - gr["size"][1] / 2 and lsv[:, 2].max() >= gr["centre"][2] + gr["size"][1] / 2,
         (round(lsv[:, 1].min(), 1), round(lsv[:, 1].max(), 1), round(lsv[:, 2].min(), 1), round(lsv[:, 2].max(), 1)), gr)
    ct = sk.CHEST_TOUCH
    tpv = np.vstack([sol[f"el_TP{k}"].reshape(-1, 3) for k in (1, 2, 3)])
    rule("touch_pads_inside_the_chest_window", tpv[:, 1].min() >= ct["centre"][1] - ct["size"][0] / 2 - 1e-6
         and tpv[:, 1].max() <= ct["centre"][1] + ct["size"][0] / 2 + 1e-6 and tpv[:, 2].min() >= ct["centre"][2] - ct["size"][1] / 2 - 1e-6
         and tpv[:, 2].max() <= ct["centre"][2] + ct["size"][1] / 2 + 1e-6, TOUCH_PADS, ct["size"])
    seam = 1.8          # the right half's seam wedge stands out of its wall at y -1.76..0 (shell.py _ub_tongue)
    rule("touch_pads_each_on_one_half", all(y1 <= -seam or y0 >= 0.5 for y0, y1 in TOUCH_PADS.values())
         and all((TOUCH_PAD_HALF[k] == "shell_upper_left") == (y0 > 0) for k, (y0, y1) in TOUCH_PADS.items()),
         TOUCH_PAD_HALF, "each pad flat on one half: clear of the seam wedge (y -1.8..0)")
    led_ok = all(abs(abs(y) - 20.0) < 1e-9 for y in (20.0, -20.0)) and 2 * 20.0 + STATUS_LED["d"] <= sk.CAMERA["bar_size"][0]
    rule("status_leds_in_the_camera_bar", led_ok, "y +-20, z at the lens", sk.CAMERA["bar_size"])
    usb = placements(r)["CHG1"].at(-4.2, 9.0, 3.2)
    skin_in = sk.LOWER_BODY.x0 + PORT_FACE_IN
    rule("usb_c_at_the_interface", abs(usb[1] - sk.USB_C["at"][1]) < 0.01 and abs(usb[2] - sk.USB_C["at"][2]) < 0.01
         and abs(usb[0] - skin_in) < 0.01, tuple(round(v, 2) for v in usb), sk.USB_C["at"],
         "the receptacle's face on the skin's inner face, behind the USB_C hole")
    btn = placements(r)["B2"].at(7.0, 7.0, 1.6 + BUTTON["h"])
    rule("power_button_at_the_interface", abs(btn[1] - sk.POWER_SWITCH["at"][1]) < 0.01 and abs(btn[2] - sk.POWER_SWITCH["at"][2]) < 0.01
         and abs(btn[0] - skin_in) < 0.01, tuple(round(v, 2) for v in btn), sk.POWER_SWITCH["at"])
    b3 = sol["el_B3"].reshape(-1, 3)
    ts = [b for n, b in sk.ZONES["electrical"] if n == "top_slot"][0]
    pcb = _board_models(r)["B3"][0].reshape(-1, 3)
    T = sk.TOP_SLOT
    rule("accessory_board_in_the_top_slot", _in_boxes(pcb.min(0), pcb.max(0), [ts])
         and _in_boxes(b3.min(0), b3.max(0), [Box(T.x0, T.x1, T.y0, T.y1, SOCKET_FLOOR_Z, T.z1)]),
         _bbox_centre(sol["el_B3"]), (ts, "header inside TOP_SLOT above the socket floor"),
         "the board in the top_slot zone, on the socket's floor; its header inside the socket")
    strip_z = [(sol[f"el_{d}"].reshape(-1, 3)[:, 2].min() + sol[f"el_{d}"].reshape(-1, 3)[:, 2].max()) / 2 for d in ("D1", "D2")]
    rule("led_strip_on_its_line", all(abs(z - sk.LED_STRIP_Z) < 0.01 for z in strip_z), strip_z, sk.LED_STRIP_Z)
    fpc = {w["net"]: w for w in wires if w["kind"] == "cable"}
    short = [(n, w["route_mm"], w["length_mm"]) for n, w in fpc.items() if w["route_mm"] * 1.1 > w["length_mm"]]
    rule("cables_long_enough", not short, short, "route x 1.1 <= cable")
    rule("pi_has_active_cooler", "U10" in comps, "Raspberry Pi Active Cooler", "")
    hw, hh = sk.REAR_HATCH["size"]
    hz0, hz1 = sk.REAR_HATCH["centre"][2] - hh / 2, sk.REAR_HATCH["centre"][2] + hh / 2
    b1 = sol["el_B1"].reshape(-1, 3)
    lower_screws_z = placements(r)["B1"].at(*B1_HOLES["screws"][0], 0.0)[2]
    ok_b1 = (b1[:, 1].max() - b1[:, 1].min() <= hw and SIZES["B1"][1] <= hh and hz0 <= lower_screws_z <= hz1
             and all(not w["connector_from"].startswith("soldered to the B1") and not w["connector_to"].startswith("soldered to the B1")
                     for w in wires))
    duct_top = max(hi[2] for lo, hi in _duct_boxes(r))
    ok_b1 = ok_b1 and hz1 - max(duct_top, hz0) >= SIZES["B1"][1]
    rule("io_board_out_through_the_hatch", ok_b1, (round(float(b1[:, 1].max() - b1[:, 1].min()), 1), SIZES["B1"][1], lower_screws_z),
         (hw, hh, (hz0, hz1)), "every B1 wire on an unpluggable housing; lower screws inside the opening; top holes on "
         f"hooks; it passes over the duct's tube (top z {duct_top:.1f})")
    hb = [((HATCH_BOSSES["x"][0], y - HATCH_BOSSES["d"] / 2, z - HATCH_BOSSES["d"] / 2),
           (HATCH_BOSSES["x"][1], y + HATCH_BOSSES["d"] / 2, z + HATCH_BOSSES["d"] / 2)) for y, z in HATCH_BOSSES["yz"]]
    prims = [(ref, sh.reshape(-1, 3).min(0), sh.reshape(-1, 3).max(0)) for ref, shells in _board_models(r).items() for sh in shells]
    prims += [(n, np.array(lo), np.array(hi)) for n, bxs in _carrier_boxes(r).items() for lo, hi in bxs]
    hit = sorted({n for n, lo, hi in prims for blo, bhi in hb
                  if all(lo[i] < bhi[i] and blo[i] < hi[i] for i in range(3))})
    rule("clear_of_the_hatch_insert_bosses", not hit, hit, HATCH_BOSSES["source"])
    hd = heat(r)
    g = _duct_geometry(r)
    rule("duct_tube_on_the_inlet", abs((g["tube_in_z"][0] + g["tube_in_z"][1]) / 2 - sk.REAR_INLET["centre"][2]) < 1e-6
         and abs((g["tube_in_y"][0] + g["tube_in_y"][1]) / 2 - sk.REAR_INLET["centre"][1]) < 1e-6, g["tube_in_z"],
         sk.REAR_INLET["centre"])
    face = COOLER["size_mm"][0] * COOLER["size_mm"][1]
    w_ = DUCT["wall"]
    inner = (g["hood_y"][1] - g["hood_y"][0] - 2 * w_) * (g["hood_z"][1] - g["hood_z"][0] - 2 * w_)
    face_x = BACK_X - _p(r).carrier_t - _p(r).pi_standoff - PI5["board_mm"][2] - COOLER["size_mm"][2]
    rule("duct_hood_covers_the_cooler_face", inner >= 0.8 * face and abs(g["hood_x"][1] - face_x) <= 0.5,
         (round(inner), round(g["hood_x"][1], 2)), (round(0.8 * face), round(face_x, 2)),
         "the hood's rim seals on the face's edge (a foam strip, 0.2 mm gap)")
    rule("bay_rise_driving_with_fan_full_below_20K", hd["ducted"][100]["driving"] < 20.0, round(hd["ducted"][100]["driving"], 1),
         20.0, "the Pi 5 itself breathes outside air through the duct")
    return out


def _zone_for(ref: str) -> str:
    on_b1 = {"U2", "U3", "U4", "U7", "JE1", "JE2", "JE3", "JE4", *B1_PASSIVES, *B1_FUSES}
    if ref in on_b1 or ref in ("U10", "S1", "R10", "C2", "JM1", "JM2", "JM3", "JM4"):
        return "bay"
    if ref in ("U9", "D3", "D4", "R8", "R9"):
        return "head"
    return "chest"


def checks(r=None) -> Dict[str, Tuple[bool, str]]:
    """The skeleton CONTRACT's form: {name: (ok, detail)}."""
    return {k: (v["ok"], f"{v['value']} (limit {v['limit']}){'; ' + v['why'] if v['why'] else ''}") for k, v in rules(r).items()}


def warnings(r=None) -> List[str]:
    pb = power_budget(r)
    T = {(op["case"], op["state"]): op for op in pb["table"]}
    h = heat(r)
    out = []
    ev = T[("everything", "end")]
    ms = T[("motor_stall", "end")]
    out.append(f"All four motors stalled draw {T[('motor_stall', 'full')]['i_motor_channel']:.2f} A a channel on a full pack "
               f"(Motoron peak {MOTORON['i_peak']} A, continuous {MOTORON['i_cont']} A). The M2T256 has no current sense: the "
               "Pico must call a stall from the encoders (no counts at > 30 % duty for 250 ms) and cut that channel. With "
               f"everything stalled the pack gives {T[('everything', 'full')]['i_bat']:.1f} A (fuse {FUSE['i_rated']:g} A: it opens "
               "in seconds, which is its job).")
    if not ev["reg5_in_regulation"] or not ms["reg5_in_regulation"]:
        out.append(f"On an empty pack, stalls sag VSW to {ev['v_bus']:.2f} V (everything) / {ms['v_bus']:.2f} V (motors): under "
                   f"the 5 V regulator's ~{REG5['vout'] + REG5['dropout_ohm'] * ev['i5']:.2f} V. Its output follows VSW down "
                   f"(Pi 5 warns near {PI5['v_low_warn']} V); the stall cut-off above is what prevents a brownout.")
    low6 = [op for op in pb["table"] if not op["reg6_in_regulation"]]
    if low6:
        out.append("The 5 V servo regulator would leave regulation in: "
                   + ", ".join(sorted({f"{op['case']}@{op['state']}" for op in low6}))
                   + " (it passes VSW through minus its dropout, still inside the XC330's 3.7-6.0 V).")
    out.append(f"Servo fault: four XC330s stalled with their current limits off would draw {SERVO['count'] * SERVO['i_stall']:.1f} A "
               f"at 5 V; the D36V28F5's over-current protection holds the rail at roughly {REG6['i_max']}-{REG6['i_fault']} A "
               "(Pololu gives only 'over-current and short-circuit protection' and a 2.5-4 A typical maximum), so the "
               "servos brown out instead of the Pi, and the XC330's own overload shutdown (its Shutdown register's "
               "overload bit, on by default) stops a stalled servo. A stalled arm (two servos) can push ~3.6 A through "
               f"its EH plug ({SERVO_PLUG['i']:g} A): only in that fault, and briefly.")
    out.append(f"Heat: the bay dissipates {h['idle']['p_bay_w']:.1f} W idle, {h['driving']['p_bay_w']:.1f} W driving. "
               f"By stack effect alone through the {h['inlet_mm2']:.0f} mm2 inlet and the {h['vent_mm2']:.0f} mm2 vent in series "
               f"({h['stack_height_mm']:.0f} mm apart) the bay air would rise ~{h['idle']['dT_chimney_K']:.0f} K idle, "
               f"~{h['driving']['dT_chimney_K']:.0f} K driving: too much. If the Active Cooler's {h['cooler_flow_l_s']:.2f} L/s "
               f"is forced through the bay (the duct: REAR_INLET -> the blower's face -> bay -> REAR_VENT), "
               f"{h['ducted'][100]['idle']:.0f} K idle / {h['ducted'][100]['driving']:.0f} K driving / "
               f"{h['ducted'][100]['everything']:.0f} K everything at full fan; at half fan {h['ducted'][50]['idle']:.0f} / "
               f"{h['ducted'][50]['driving']:.0f} K. The fan is temperature-controlled: set its curve to run at full "
               "speed from a modest SoC temperature (config.txt fan_temp0..3 dtparams) so the bay is always flushed.")
    out.append(f"Charging: {CHARGER['i_charge']:g} A (ASSUMED module setting) fills the pack in ~{pb['charge_h']:.1f} h; idle "
               f"draws {T[('idle', 'nominal')]['i_bat']:.2f} A, so a robot left ON while plugged in barely charges: charge it OFF.")
    out.append("The charger is a generic IP2326 module and the protection board a generic 2S board: no vendor data sheet "
               "covers the boards themselves (the IP2326 chip's does). Buy both from a seller that documents them, or "
               "replace by a documented board, and re-check BMS/CHARGER here.")
    out.append("USB-C here is 5 V Type-C charging (15 W), not PD: the skeleton's 'USB-C PD charging' decision is met "
               "for any PD charger (PD sources give 5 V first) but not at PD's higher power. A PD-input 2S charger "
               "(IP2368 class) would charge faster but is a larger, again undocumented, module.")
    return out


def interface_requests() -> List[str]:
    """What this design asks of the other subsystems and the skeleton (for the integrator)."""
    return [
        f"CHASSIS: battery: our 2S2P pack, {PACK['outline_mm'][0]:g} (x) x {PACK['outline_mm'][1]:g} (y) x "
        f"{PACK['outline_mm'][2]:g} (z) mm, {PACK['g']:g} g, box {PACK_VOLUME['box']}; cells along y, the protection "
        f"board and the P+/P- leads (#18, 100 mm, XT30 at the end) at the -y end, leaving at "
        f"{PACK['leads_exit']}, along the top face to the deck's 10 mm hole at (-35, 0).",
        "CHASSIS: used mounts: bay_pi5_0..3 (12 mm M2.5 F-F standoffs, through our 3 mm carrier plate's 2.9 mm holes: "
        "your M2.5 x 16 takes 7 mm of the standoff's thread), bay_row_0..4 (RX-M3x5.7 inserts in our bay carrier), "
        "head_grid at (+-40, 190) and (+-40, 250) (RX-M3x5.7 in our front carrier). The four M2.5 screws are yours "
        "(not in our mass). The nine M3 x 12 into our inserts (5 bay_row, 4 head_grid), heads inside the spine on its "
        "6 mm plates, are drawn and counted HERE (interface_solids()) because chassis.interface_joints() has none: "
        "take them over (and their mass) if you prefer, by your convention for the Pi's.",
        "CHASSIS: front motor and encoder leads: up through the front deck slot, inside the spine and through the mid "
        "rung's 14 mm hole into the bay (12 conductors); rear ones up the rear slot. Motor leads need ~300 mm "
        "(front) / ~80 mm (rear) to their Mini-Fit plugs, encoder leads ~240 / ~150 mm to the IO board: Pololu's own "
        "leads are 200 mm (ASSUMED), so the front ones need an extension (ours or yours: say).",
        "CHASSIS: the front<->bay harness (both FPCs, 16 mm wide, and ~20 thin wires) passes through the spine's open "
        "section between the mid rung (z 136) and the front plate (z 180): please keep it open.",
        f"SHELL: the LED strips (D1 front, D2 rear: {LED_FRONT} + {LED_REAR} pixels, {LED_FRONT * STRIP['pitch']:.0f} mm "
        f"each, 12.5 x 4 mm sleeved) slide into the bumpers' channels from their ends: x +-{LED_CHANNEL_X:g}.."
        f"{LED_CHANNEL_X + STRIP['t']:g}, z {sk.LED_STRIP_Z - STRIP['width'] / 2:g}-{sk.LED_STRIP_Z + STRIP['width'] / 2:g} "
        "(interface parts), input end at -y on the front and +y on the rear.",
        "SHELL: ports, confirmed to your holes: the charger board (22 x 18, its near edge at x -85.8, underside z 68.8) "
        "has M2 holes at (x -83.3, y -26.5) and (-83.3, -13.5), screws down into the shelf's inserts (the M2 heads "
        "clear the receptacle, y -24.5..-15.5, by 0.1 mm: use low heads or washers no wider than 3.8 mm); the "
        "button board (14 x 14, its front face at x -82, y 13-27, z 65-79) has M2 holes at (y 14, z 66) and (26, 78), "
        "screws along -x into the posts; both clear the 9.8 mm cap.",
        "SHELL: the touch pads are copper tape on the chest wall's flat inside face (x 78.0), z 117-147, each whole on "
        "one half, taped on the bench (shell step 1): TP1 y -31..-13 (right half), TP2 y 1..11 and TP3 y 15..31 (left "
        "half). If the chest window marks the pads, the centre one is at y 6.",
        "SKELETON: the top_slot zone (z 272-284) does not fit the shell's socket: its floor fills z 272-276, so our "
        "accessory board B3 (8 x 18 mm, x -4..4, y -9..9) sits ON it at z 276-277.6 and its 2 x 4 female header "
        "(5.08 x 10.16 mm, 8.5 mm tall) reaches z 286.1. Suggest top_slot = x +-4, y +-9, z 276-287 (B3 is an "
        "interface part until then).",
        "SHELL: mic acoustic ports: the mic boards' port faces are at z 261.5, 0.5 mm under your ducts' ends (z 262), "
        "so the upper halves slide over them clear; a 1 mm closed-cell foam ring (compressed to 0.5) seals each.",
        f"SHELL: the cooler duct's seat: a flat ring on the hatch's inside (x {duct_seat()['face_x']:g}) round REAR_INLET, "
        f"{duct_seat()['seat_outer_mm'][0]:g} x {duct_seat()['seat_outer_mm'][1]:g} mm outer, the inlet's "
        f"{sk.REAR_INLET['size'][0]:g} x {sk.REAR_INLET['size'][1]:g} opening inside it, centred at y 0, z "
        f"{sk.REAR_INLET['centre'][2]:g}; our tube ({duct_seat()['tube_outer_mm'][0]:g} x {duct_seat()['tube_outer_mm'][1]:g} "
        "outer) ends at x -75 with a 3 mm foam gasket that the closed hatch compresses. REAR_VENT needs no seat: it "
        "stays a plain opening (the bay is the plenum).",
        f"ARMS: we deliver a separate 5.0 V +-4 % servo rail ({REG6['i_max']} A continuous, current-limited near "
        f"{REG6['i_fault']} A) and the half-duplex DATA at 3.3 V logic, 1 Mbps (set every servo's Baud Rate to 3), "
        "one 3-wire bus run (#22 power, #26 data, JST EHR-03 at the servo) per side from the bay, out beside the "
        "spine's side face at z 160, to the shoulder servo's B3B-EH-A at (0, +-82, 160). Please keep the current "
        "limits on and the overload shutdown bit set; Protocol 2.0 IDs 1-2 left, 3-4 right (ours to agree).",
    ]


# =========================================================================================
# 9. Diagrams
# =========================================================================================

INK, MUTED, PAPER, ACCENT = "#1b1f24", "#5d6873", "#ffffff", "#c0392b"
WIRE_RGB = {"red": "#d62828", "black": "#1b1b1b", "orange": "#f08c00", "yellow": "#e9c400", "green": "#2b9348",
            "blue": "#1d4ed8", "violet": "#7b2cbf", "brown": "#8b5a2b", "grey": "#8d99ae", "white": "#d8d8d8", "pink": "#e75480"}
POWER_NETS = {"VBAT_RAW", "VBAT", "VSW", "5V", "SERVO_5V", "PICO_3V3", "PI_3V3", "VSYS", "LED_5V", "TOP_5V"}


def _t(x, y, s, size=11, anchor="start", fill=INK, weight="normal", family="Helvetica, Arial, sans-serif"):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-family="{family}" font-size="{size}" text-anchor="{anchor}" '
            f'fill="{fill}" font-weight="{weight}">{escape(str(s))}</text>')


def schematic_svg(r=None) -> str:
    """Block-and-pin schematic: components in two columns, pins facing the channel between;
    each net is a vertical trunk in the channel with a stub and a dot to each of its pins."""
    comps, nets = components(), netlist()
    refs = [k for k, c in comps.items() if c["pins"]]
    half, acc, total = [], 0, sum(len(comps[k]["pins"]) + 2 for k in refs)
    for k in refs:
        if acc < total / 2:
            half.append(k)
        acc += len(comps[k]["pins"]) + 2
    left = half
    right = [k for k in refs if k not in left]
    pitch, box_w, gap, lx, top = 14.0, 170.0, 16.0, 40.0, 120.0
    order = sorted(nets, key=lambda n: (0 if n == "GND" else 1, n))
    trunk_dx = 9.0
    ch_x0 = lx + box_w + 60.0
    trunk_x = {n: ch_x0 + i * trunk_dx for i, n in enumerate(order)}
    rx = ch_x0 + len(order) * trunk_dx + 60.0
    pin_xy, body = {}, []

    def column(refs_, x, pins_on_right):
        y = top
        for ref in refs_:
            c = comps[ref]
            h = pitch * len(c["pins"]) + 30.0
            body.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{box_w}" height="{h:.1f}" rx="3" fill="#f7f4ee" stroke="{INK}" stroke-width="1.2"/>')
            body.append(_t(x + box_w / 2, y + 13, ref, 11, "middle", weight="bold"))
            short = c["part"].split(",")[0]
            short = short if len(short) <= 34 else short[:33] + "..."
            body.append(_t(x + (box_w - 6 if pins_on_right else 6), y + h - 5, short, 6.5, "end" if pins_on_right else "start", MUTED))
            for k, p in enumerate(c["pins"]):
                py = y + 26 + pitch * k
                px = x + box_w if pins_on_right else x
                pin_xy[(ref, p)] = (px, py)
                body.append(_t(px + (-5 if pins_on_right else 5), py + 3.5, p, 8.5, "end" if pins_on_right else "start"))
                if (ref, p) in NO_CONNECT:
                    sx = px + (12 if pins_on_right else -12)
                    body.append(f'<line x1="{px:.1f}" y1="{py:.1f}" x2="{sx:.1f}" y2="{py:.1f}" stroke="{MUTED}" stroke-width="1"/>')
                    body.append(f'<path d="M{sx - 3:.1f},{py - 3:.1f} l6,6 m0,-6 l-6,6" stroke="{ACCENT}" stroke-width="1.2"/>')
            y += h + gap
        return y

    y_end = max(column(left, lx, True), column(right, rx, False))
    lines = []
    for n in order:
        tx = trunk_x[n]
        ys = [pin_xy[p][1] for p in nets[n]]
        col = "#b02a2a" if n in POWER_NETS else INK if n == "GND" else "#1d4ed8"
        lines.append(f'<line x1="{tx:.1f}" y1="{min(ys):.1f}" x2="{tx:.1f}" y2="{max(ys):.1f}" stroke="{col}" stroke-width="1.3"/>')
        lines.append(f'<g transform="translate({tx + 2:.1f},{min(ys) - 5:.1f}) rotate(-60)">{_t(0, 0, n, 7.5, "start", col, "bold")}</g>')
        for p in nets[n]:
            px, py = pin_xy[p]
            lines.append(f'<line x1="{px:.1f}" y1="{py:.1f}" x2="{tx:.1f}" y2="{py:.1f}" stroke="{col}" stroke-width="0.8" stroke-opacity="0.75"/>')
            lines.append(f'<circle cx="{tx:.1f}" cy="{py:.1f}" r="2.2" fill="{col}"/>')
    width, height = rx + box_w + 40.0, y_end + 30.0
    head = [_t(lx, 30, "SparkRover Mini: schematic", 18, weight="bold"),
            _t(lx, 46, "Generated by electrical.py; do not hand-edit. A dot joins a pin to its net; a red X is a no-connect.", 10, fill=MUTED),
            _t(lx, 60, "Pi 5: " + ", ".join(f"{k} {v.split(' (')[0]}" for k, v in list(PI_USE.items())[:9]), 9, fill=MUTED),
            _t(lx, 73, "Pico: " + ", ".join(f"{k} {v.split(' (')[0]}" for k, v in list(PICO_USE.items())[:9]), 9, fill=MUTED)]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">\n'
            f'<rect width="100%" height="100%" fill="{PAPER}"/>\n' + "\n".join(head + body + lines) + "\n</svg>\n")


def wiring_svg(r=None, lengths=None) -> str:
    """Two views (side: x right, z up; rear: y right (from behind: +y on the left), z up) of the
    zones, our parts' outlines and every wire as its estimated route, coloured as the wire."""
    s, m = 2.6, 40.0
    sol = solids(r)
    wires = wire_list(r, lengths)
    vw = (sk.DEPTH + 40) * s
    views = [("side (from the left: x forward to the right)", lambda p: (p[0], p[2]), -112.5, 112.5),
             ("rear (from behind: +y to the left)", lambda p: (-p[1], p[2]), -122.5, 122.5)]
    g = []
    H = (sk.HEIGHT + 30) * s

    def P(ix, a, z):
        ox = m + ix * (vw + 60) + (a + views[ix][2] * -1) * s
        return ox, m + 50 + (sk.HEIGHT + 10 - z) * s
    for ix, (title, proj, _, _) in enumerate(views):
        g.append(_t(m + ix * (vw + 60), m + 40, title, 11, weight="bold"))
        for owner, zs in sk.ZONES.items():
            for nme, b in zs:
                corners = [(b.x0, b.y0, b.z0), (b.x1, b.y1, b.z1)]
                (a0, z0), (a1, z1) = proj(corners[0]), proj(corners[1])
                (X0, Y0), (X1, Y1) = P(ix, min(a0, a1), max(z0, z1)), P(ix, max(a0, a1), min(z0, z1))
                col = "#c9e4c5" if owner == "electrical" else "#eeeeee"
                g.append(f'<rect x="{X0:.1f}" y="{Y0:.1f}" width="{X1 - X0:.1f}" height="{Y1 - Y0:.1f}" fill="{col}" fill-opacity="0.45" stroke="#9aa5ad" stroke-width="0.6"/>')
        for label, tris in sol.items():
            v = tris.reshape(-1, 3)
            pts = [proj(p) for p in (v.min(0), v.max(0))]
            (X0, Y0), (X1, Y1) = P(ix, min(pts[0][0], pts[1][0]), max(pts[0][1], pts[1][1])), P(ix, max(pts[0][0], pts[1][0]), min(pts[0][1], pts[1][1]))
            g.append(f'<rect x="{X0:.1f}" y="{Y0:.1f}" width="{X1 - X0:.1f}" height="{Y1 - Y0:.1f}" fill="none" stroke="{MUTED}" stroke-width="0.4"/>')
        for w in wires:
            pts = [P(ix, *proj(p)) for p in w["path"]]
            d = " ".join(f"{'M' if k == 0 else 'L'}{x:.1f},{y:.1f}" for k, (x, y) in enumerate(pts))
            width = {18: 2.4, 20: 2.0, 22: 1.6, 24: 1.3, 26: 1.0, 28: 0.8}.get(w["awg"], 1.4)
            g.append(f'<path d="{d}" fill="none" stroke="{WIRE_RGB[w["colour"]]}" stroke-width="{width}" stroke-opacity="0.85"/>')
            if ix == 0:
                x, y = pts[len(pts) // 2]
                g.append(_t(x + 2, y - 2, w["id"], 5.5, fill=INK))
    lx0 = m + 2 * (vw + 60)
    g.append(_t(lx0, m + 40, "Wire  from -> to  AWG  length", 10, weight="bold"))
    for k, w in enumerate(wires):
        y = m + 56 + 11 * k
        g.append(f'<line x1="{lx0:.1f}" y1="{y - 3:.1f}" x2="{lx0 + 18:.1f}" y2="{y - 3:.1f}" stroke="{WIRE_RGB[w["colour"]]}" stroke-width="3"/>')
        g.append(_t(lx0 + 24, y, f"{w['id']}  {w['from'][0]}.{w['from'][1]} -> {w['to'][0]}.{w['to'][1]}  "
                    f"{('#' + str(w['awg'])) if w['awg'] else w['kind']}  {w['length_mm']:.0f} mm ({w['length_source']})", 7.5,
                    family="Consolas, Menlo, monospace"))
    W = lx0 + 430
    Ht = max(H + m + 60, m + 70 + 11 * len(wires))
    head = [_t(m, 24, "SparkRover Mini: wiring (estimated routes through the spine's passages)", 16, weight="bold")]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{Ht:.0f}" viewBox="0 0 {W:.0f} {Ht:.0f}">\n'
            f'<rect width="100%" height="100%" fill="{PAPER}"/>\n' + "\n".join(head + g) + "\n</svg>\n")


# =========================================================================================
# 10. Release files
# =========================================================================================

def wires_md(r=None, lengths=None) -> str:
    L = ["# SparkRover Mini: wire list", "",
         "Generated by `electrical.py`; do not hand-edit. Lengths marked *estimate* are the route through the spine's "
         f"passages x {ROUTE_FACTOR} + {SERVICE_MM:.0f} mm, until the integrator's 3D routing replaces them.", "",
         "| Wire | From | To | Net | Colour | AWG | End at From | End at To | Length (mm) |", "|---|---|---|---|---|---|---|---|---|"]
    for w in wire_list(r, lengths):
        L.append(f"| {w['id']} | {w['from'][0]}.{w['from'][1]} | {w['to'][0]}.{w['to'][1]} | {w['net']} | {w['colour']} | "
                 f"{w['awg'] or w['kind']} | {w['connector_from']} | {w['connector_to']} | {w['length_mm']:.0f} *{w['length_source']}* |")
    L += ["", "## Joints (a part's own lead or the board, no wire)", "", "| Pin | Pin | How |", "|---|---|---|"]
    L += [f"| {a[0]}.{a[1]} | {b[0]}.{b[1]} | {how} |" for a, b, how in JOINTS]
    L += ["", "## Pin assignment", "", "| Raspberry Pi 5 | Use |", "|---|---|"] + [f"| {k} | {v} |" for k, v in PI_USE.items()]
    L += ["", "| Pico | Use |", "|---|---|"] + [f"| {k} | {v} |" for k, v in PICO_USE.items()]
    L += ["", "## Parts", "", "| Ref | Part | Where | Source |", "|---|---|---|---|"]
    L += [f"| {k} | {c['part']} | {c['where']} | {c['source']} |" for k, c in components().items()]
    L += ["", "## No-connects", "", "| Pin | Why |", "|---|---|"] + [f"| {a}.{b} | {why} |" for (a, b), why in NO_CONNECT.items()]
    return "\n".join(L) + "\n"


def power_md(r=None) -> str:
    pb = power_budget(r)
    h = heat(r)
    L = ["# SparkRover Mini: power budget", "",
         "Generated by `electrical.py`; do not hand-edit. Solved from the vendors' figures: the pack (2S2P of P28A, 20 mohm DC "
         "a cell, 20 mohm the pack, + protection board), fuse, switch, connectors and wires; the regulators as constant-power loads at "
         f"{REG5['eta']:.0%}; the motors at the chassis's currents (2.4 ohm at stall). **Nothing here is measured.**", "",
         "| Case | Pack | EMF (V) | VSW (V) | Pack (A) | Pack (W) | 5 V rail (A) | servo rail (A) | Motor channel (A) | 5 V reg OK |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for op in pb["table"]:
        L.append(f"| {op['case']} | {op['state']} | {op['emf_v']:.1f} | {op['v_bus']:.2f} | {op['i_bat']:.2f} | {op['p_pack_w']:.1f} | "
                 f"{op['i5']:.2f} | {op['i6']:.2f} | {op['i_motor_channel']:.2f} | {'yes' if op['reg5_in_regulation'] else '**no**'} |")
    L += ["", "Cases: " + "; ".join(f"**{k}** = {v}" for k, v in CASES.items()), "",
          "## Runtime", "", f"Usable energy {pb['energy_wh']:.1f} Wh (4 x 10.3 Wh typical x {PACK['usable_frac']:.0%} above the "
          "3.2 V/cell cut-off, ASSUMED). The sheets ask 2-4 h.", "", "| Use | Pack (W) | Runtime (h) |", "|---|---|---|"]
    for k, v in pb["runtime"].items():
        L.append(f"| {k}{' ' + str(v.get('mix')) if 'mix' in v else ''} | {v['p_pack_w']:.1f} | {v['hours']:.2f} |")
    L += ["", f"Charging: {CHARGER['i_charge']:g} A into the pack (ASSUMED setting), ~{pb['charge_h']:.1f} h from empty, "
          f"{pb['charge_w_in']:.1f} W from the USB-C source.", "",
          f"Short circuit after the fuse: {pb['short_circuit_a']:.0f} A (pack {pb['r_shared']['pack'] * 1e3:.0f} mohm, wiring "
          f"and parts {(pb['r_shared']['to_fuse_short'] - pb['r_shared']['pack']) * 1e3:.0f} mohm); the 15 A MINI fuse "
          "interrupts up to 1000 A, the protection board also trips.", "", "## 5 V rail, everything at once (full pack)", "",
          "| Load | W |", "|---|---|"]
    ev = [op for op in pb["table"] if op["case"] == "everything" and op["state"] == "full"][0]
    L += [f"| {k} | {v:.2f} |" for k, v in ev["p5_items"].items()]
    L += ["", "## Heat", "", "| Case | Bay (W) | of which the Pi (W) | Chimney dT (K) |", "|---|---|---|---|"]
    L += [f"| {c} | {h[c]['p_bay_w']:.1f} | {h[c]['p_pi_w']:.1f} | {h[c]['dT_chimney_K']:.0f} |" for c in ("idle", "driving", "everything")]
    L += ["", "## Ratings", "", "| Rule | Value | Limit | OK |", "|---|---|---|---|"]
    for k, v in rules(r, fcl=False).items():
        if any(s in k for s in ("fuse", "switch", "motoron", "pack", "reg", "connector", "servo", "wires", "pi_5v", "charger")):
            L.append(f"| {k} | {v['value']} | {v['limit']} | {'yes' if v['ok'] else '**no**'} |")
    L += ["", "## Flags", ""] + [f"- {x}" for x in warnings(r)]
    return "\n".join(L) + "\n"


def placement_md(r=None) -> str:
    pl = placements(r)
    sol = solids(r)
    L = ["# SparkRover Mini: electrical placement", "", "Generated by `electrical.py`. Robot frame, mm (skeleton.py).", "",
         "| Part | Zone | Bounding box min | max | Mounting |", "|---|---|---|---|---|"]
    how = {"U1": "4 x M2.5 F-F standoffs on the spine's Pi pattern (screws from inside the spine through our plate)",
           "B1": "hangs by its two top holes on the carrier's hook pins (z 170.5), two M2.5 x 6 into RX-M2.5x5.7 inserts at "
                 "the bottom (z 131.5): removable through the rear hatch (see the rule io_board_out_through_the_hatch)",
           "U5": "2 x M2 x 5 into 1.7 mm pilots in the rear lip's bosses", "U6": "2 x M2 x 5 into 1.7 mm pilots in wing bosses",
           "SW1": "VHB foam tape on the wing", "MA": "VHB on the wing", "MB": "VHB on the wing", "X1": "VHB", "X2": "VHB", "X3": "VHB",
           "X4": "VHB on the rear lip", "F1": "cable ties to the wing", "CHG1": "receptacle in the shell's USB_C pocket (2 x M2 shell bosses requested), inner end on the bay's ledge",
           "B2": "2 x M2 to shell bosses behind the POWER_SWITCH hole (requested)",
           "cooler_duct": "2 x M2 x 10 down through its foot, beside the tube, into RX-M2x4 inserts in the rear lip's pads "
                          "(a 1.5 mm L-key from the hatch); a foam strip seals the hood on the cooler's face; lifts out "
                          "through the hatch (fitted after, removed before, the Pi)", "U8": "VHB tape to the front carrier's ring",
           "U9": "4 x M2 x 5 into 1.7 mm pilots in the camera posts",
           "B3": "VHB on the top socket's floor, its wires up through the floor's header slot",
           "MK1": "VHB on the shelf's pillars, port up", "MK2": "VHB on the shelf's pillars, port up",
           "U12": "2 x M2 x 6 into 1.7 mm pilots in chest bosses", "U11": "2 x M2 x 6 into 1.7 mm pilots in chest bosses",
           "LS1": "4 x M3 x 8 through its ears into RX-M3x5.7 inserts at the speaker posts' ends"}
    fz = {}
    for f in fasteners(r):
        fz[f["label"][3:]] = fz[f["label"].replace("el_screw_", "el_insert_")[3:]] = CARRIER_ZONE.get(f["carrier"])
    for label, tris in sol.items():
        v = tris.reshape(-1, 3)
        ref = label[3:]
        zone = pl[ref].zone if ref in pl else (CARRIER_ZONE.get(ref) or fz.get(ref) or
                                               ("interface" if ref in INTERFACE_PARTS else _zone_for(ref)))
        L.append(f"| {label} | {zone} | {tuple(round(float(x), 1) for x in v.min(0))} | {tuple(round(float(x), 1) for x in v.max(0))} | "
                 f"{how.get(ref, '-')} |")
    L += ["", "## Fasteners into the printed carriers", "",
          "Every screw that passes through or into a carrier, and the hole the carrier gives it (cut in the printed "
          "solid that `jobs()` prints). " + SCREW_SOURCE + "; inserts: " + "; ".join(
              f"{k}: {v['spec']} ({v['source']})" for k, v in INSERTS.items()) + "; M2 pilots: " + PILOT_M2["source"] + ".", "",
          "| Screw | Size | Head at | Axis | Grip | Thread | Passes (clearance) | Thread in | Carrier hole | Step | Owner |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for f in fasteners(r):
        h = f["hole"]
        hole = (f"{SCREW[f['size']]['clear']:g} mm clearance through" if f["holder"] == "standoff" else h["what"])
        L.append(f"| {f['label']} | {f['size']} x {f['length']} | {tuple(round(v, 2) for v in f['at'])} | "
                 f"{tuple(int(v) for v in f['axis'])} | {f['grip_mm']:g} | {f['thread_in_mm']:g} | "
                 f"{', '.join(f'{w} {d:g}' for w, d in f['through'])} | {f['into']} | {f['carrier']}: {hole} | "
                 f"{f['step']} | {f['owner']} |")
    L += ["", "## Assembly steps (in the build order)", ""] + [f"- {k} ({JOINS_SHELL_STEP[k]}): {v}" for k, v in STEPS.items()]
    L += ["", "## Mounts used (the chassis's)", ""] + [f"- {mt.name}: {mt.at} {mt.size}, thread in {mt.holder}" for mt in mounts(r)]
    L += ["", "## Interface requests and skeleton issues", ""] + [f"- {x}" for x in interface_requests()]
    L += ["", "## Constants other subsystems own", "",
          f"- MOTOR (chassis): {MOTOR}", f"- ENCODER (chassis): {ENCODER}", f"- SERVO (arms, ASSUMED): {SERVO}",
          f"- SERVO_PLUG (arms, ASSUMED): {SERVO_PLUG}", f"- PACK_VOLUME (chassis): {PACK_VOLUME['source']}"]
    return "\n".join(L) + "\n"


def write(out: Path = OUT, lengths=None) -> Dict[str, Tuple[bool, str]]:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "schematic.svg").write_text(schematic_svg(), encoding="utf-8", newline="\n")
    (out / "wiring.svg").write_text(wiring_svg(None, lengths), encoding="utf-8", newline="\n")
    (out / "WIRES.md").write_text(wires_md(None, lengths), encoding="utf-8", newline="\n")
    (out / "POWER.md").write_text(power_md(), encoding="utf-8", newline="\n")
    (out / "PLACEMENT.md").write_text(placement_md(), encoding="utf-8", newline="\n")
    return checks()


def write_meshes(dest: Path = MESHES) -> List[Path]:
    dest.mkdir(parents=True, exist_ok=True)
    paths = []
    for label, text in jobs().items():
        p = dest / f"{label}.stl"
        p.write_text(text, encoding="utf-8", newline="\n")
        paths.append(p)
    return paths


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(names: Sequence[str] = tuple(VENDOR_CAD)) -> Dict[str, Path]:
    """Download the vendors' STEP archives into .tmp/vendor_cad/spark_rover (never committed)
    and verify each SHA-256; a file that does not match is refused."""
    VENDOR_CACHE.mkdir(parents=True, exist_ok=True)
    got = {}
    for name in names:
        src = VENDOR_CAD[name]
        raw = VENDOR_CACHE / src["url"].rsplit("/", 1)[1]
        if not raw.exists() or _sha256(raw) != src["sha256"]:
            with urllib.request.urlopen(src["url"], timeout=300) as resp:
                raw.write_bytes(resp.read())
        if _sha256(raw) != src["sha256"]:
            raise RuntimeError(f"{name}: {raw.name} does not match its SHA-256; the vendor changed it: check, then update VENDOR_CAD")
        got[name] = raw
    return got


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SparkRover Mini electrical design")
    ap.add_argument("cmd", nargs="?", default="write", choices=("write", "fetch", "checks"))
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args(argv)
    if a.cmd == "fetch":
        for k, v in fetch().items():
            print(f"{k}: {v}")
        return 0
    res = write(Path(a.out)) if a.cmd == "write" else checks()
    if a.cmd == "write":
        write_meshes()
    for k, (ok, d) in res.items():
        print(f"{'PASS' if ok else 'FAIL'}  {k}: {d}")
    pb = power_budget()
    for k, v in pb["runtime"].items():
        print(f"runtime {k}: {v['hours']:.2f} h at {v['p_pack_w']:.1f} W")
    for x in warnings():
        print(f"FLAG  {x}")
    return 0 if all(ok for ok, _ in res.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
