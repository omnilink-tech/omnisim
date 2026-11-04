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

"""Printer description for an FDM twin, read from the machine's own Klipper config.

Everything downstream (the Klipper model in klipper.py, the hotend model, preflight,
the world generator, the slicer wrapper) reads one `Printer`. Units are Klipper's own:
mm, mm/s, mm/s^2, seconds, degrees C.

A profile (projects/manufacturing/machines/<name>/<name>.json) holds the numbers this twin uses, extracted from the
vendor's printer.cfg with `python printer.py extract printer.cfg`, plus what a config
file does not say: the homing sequence its `homing_override` runs, the motion its
start/end macros make, the hotend's melt limit, the frame the world is drawn from and
the slicer settings. Only the numbers are kept, with the source and commit they came
from: a vendor's config file is usually GPL and is not copied into this repository.

    python printer.py extract printer.cfg            # the "klipper" block of a profile
    python printer.py show ../machines/sovol_sv08/sovol_sv08.json  # what the twin will use
"""
from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

AXES = ("x", "y", "z")
HERE = Path(__file__).resolve().parent
DEFAULT_PROFILE = HERE.parent / "machines/sovol_sv08/sovol_sv08.json"

# The config sections and keys the twin reads. Everything else in a printer.cfg (pins,
# drivers, displays, LEDs) is wiring, not behaviour, and is left out of a profile.
USED_KEYS = {
    "printer": ["kinematics", "max_velocity", "max_accel", "max_accel_to_decel", "minimum_cruise_ratio",
                "square_corner_velocity", "max_z_velocity", "max_z_accel"],
    "stepper_x": ["rotation_distance", "microsteps", "full_steps_per_rotation", "gear_ratio", "position_min",
                  "position_max", "position_endstop", "homing_speed", "homing_positive_dir"],
    "extruder": ["rotation_distance", "microsteps", "full_steps_per_rotation", "gear_ratio", "nozzle_diameter",
                 "filament_diameter", "max_extrude_only_distance", "max_extrude_cross_section",
                 "max_extrude_only_velocity", "max_extrude_only_accel", "instantaneous_corner_velocity",
                 "pressure_advance", "pressure_advance_smooth_time", "min_extrude_temp", "max_temp"],
    "heater_bed": ["max_temp"],
    "input_shaper": ["shaper_type", "shaper_type_x", "shaper_type_y", "shaper_freq_x", "shaper_freq_y",
                     "damping_ratio_x", "damping_ratio_y"],
    "probe": ["x_offset", "y_offset"],
}
USED_KEYS["stepper_y"] = USED_KEYS["stepper_x"]
USED_KEYS["stepper_z"] = USED_KEYS["stepper_x"]


def parse_klipper_cfg(text: str) -> Dict[str, Dict[str, str]]:
    """Klipper's config format: [section], `key: value` or `key = value`, indented
    continuation lines, `#` / `;` comments (inline after whitespace). A repeated key
    keeps its last value, as Klipper's parser does."""
    sections: Dict[str, Dict[str, str]] = {}
    current: Optional[Dict[str, str]] = None
    last_key = None
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith(("#", ";")):
            continue
        if raw[0] in " \t" and current is not None and last_key is not None:
            current[last_key] += "\n" + raw.strip()          # a gcode: block or a list
            continue
        line = re.split(r"\s[#;]", raw, maxsplit=1)[0].strip()
        m = re.match(r"^\[([^\]]+)\]$", line)
        if m:
            current = sections.setdefault(m.group(1).strip(), {})
            last_key = None
            continue
        m = re.match(r"^([^:=]+?)\s*[:=]\s*(.*)$", line)
        if m and current is not None:
            last_key = m.group(1).strip().lower()
            current[last_key] = m.group(2).strip()
    return sections


def extract(sections: Dict[str, Dict[str, str]]) -> Dict[str, Dict[str, str]]:
    out = {}
    for name, keys in USED_KEYS.items():
        sec = sections.get(name)
        if sec is None:
            continue
        kept = {k: sec[k] for k in keys if k in sec}
        if kept:
            out[name] = kept
    return out


def _num(sec: Dict[str, str], key: str, default: Optional[float] = None) -> Optional[float]:
    v = sec.get(key)
    return float(v) if v not in (None, "") else default


def _ratio(text: Optional[str]) -> float:
    """Klipper gear_ratio "80:12" (or "80:12, 2:1"): output turns per motor turn, inverted."""
    if not text:
        return 1.0
    r = 1.0
    for pair in text.split(","):
        a, b = pair.split(":")
        r *= float(a) / float(b)
    return r


@dataclass
class Axis:
    name: str
    position_min: float
    position_max: float
    position_endstop: float
    homing_speed: float
    homing_positive_dir: bool
    rotation_distance: float
    microsteps: int
    full_steps: int
    gear_ratio: float

    @property
    def travel(self) -> float:
        return self.position_max - self.position_min

    @property
    def steps_per_mm(self) -> float:
        return self.full_steps * self.microsteps * self.gear_ratio / self.rotation_distance


@dataclass
class Extruder:
    nozzle_diameter: float
    filament_diameter: float
    steps_per_mm: float
    pressure_advance: float
    pressure_advance_smooth_time: float
    max_extrude_only_distance: float
    max_cross_section: float
    max_e_velocity: float
    max_e_accel: float
    instant_corner_v: float
    min_extrude_temp: float
    max_temp: float

    @property
    def filament_area(self) -> float:
        return math.pi * (0.5 * self.filament_diameter) ** 2


@dataclass
class Shaper:
    kind: str          # "mzv", "zv", "ei", "2hump_ei", "3hump_ei", "zvd" or "" (off)
    freq: float
    damping: float


@dataclass
class Printer:
    name: str
    kinematics: str
    max_velocity: float
    max_accel: float
    max_accel_to_decel: Optional[float]
    minimum_cruise_ratio: Optional[float]
    square_corner_velocity: float
    max_z_velocity: float
    max_z_accel: float
    axes: Dict[str, Axis]
    extruder: Extruder
    shapers: Dict[str, Shaper]
    bed_max_temp: float
    probe_offset: List[float]
    profile: dict = field(default_factory=dict, repr=False)

    def summary(self) -> dict:
        return {
            "name": self.name, "kinematics": self.kinematics, "max_velocity": self.max_velocity,
            "max_accel": self.max_accel, "max_accel_to_decel": self.max_accel_to_decel,
            "square_corner_velocity": self.square_corner_velocity,
            "max_z_velocity": self.max_z_velocity, "max_z_accel": self.max_z_accel,
            "envelope": {a: [ax.position_min, ax.position_max] for a, ax in self.axes.items()},
            "steps_per_mm": {a: round(ax.steps_per_mm, 4) for a, ax in self.axes.items()},
            "nozzle_mm": self.extruder.nozzle_diameter, "filament_mm": self.extruder.filament_diameter,
            "e_steps_per_mm": round(self.extruder.steps_per_mm, 4),
            "pressure_advance": self.extruder.pressure_advance,
            "input_shaper": {a: [s.kind, s.freq, s.damping] for a, s in self.shapers.items()},
        }


def from_sections(name: str, k: Dict[str, Dict[str, str]], profile: Optional[dict] = None) -> Printer:
    p = k["printer"]
    max_velocity = _num(p, "max_velocity")
    max_accel = _num(p, "max_accel")
    axes = {}
    for a in AXES:
        s = k.get(f"stepper_{a}", {})
        lo, hi = _num(s, "position_min", 0.0), _num(s, "position_max")
        endstop = _num(s, "position_endstop")
        axes[a] = Axis(a, lo, hi, endstop if endstop is not None else lo,
                       _num(s, "homing_speed", 5.0), s.get("homing_positive_dir", "false").lower() == "true",
                       _num(s, "rotation_distance"), int(_num(s, "microsteps")),
                       int(_num(s, "full_steps_per_rotation", 200)), _ratio(s.get("gear_ratio")))
    e = k["extruder"]
    nozzle = _num(e, "nozzle_diameter")
    filament = _num(e, "filament_diameter")
    area = math.pi * (0.5 * filament) ** 2
    # Klipper's defaults (klippy/kinematics/extruder.py): a cross-section of 4 nozzle
    # diameters squared, and extrude-only limits scaled from the toolhead's by that ratio.
    def_cross = 4.0 * nozzle * nozzle
    cross = _num(e, "max_extrude_cross_section", def_cross)
    e_steps = (int(_num(e, "full_steps_per_rotation", 200)) * int(_num(e, "microsteps"))
               * _ratio(e.get("gear_ratio")) / _num(e, "rotation_distance"))
    extruder = Extruder(nozzle, filament, e_steps, _num(e, "pressure_advance", 0.0),
                        _num(e, "pressure_advance_smooth_time", 0.040), _num(e, "max_extrude_only_distance", 50.0),
                        cross, _num(e, "max_extrude_only_velocity", max_velocity * def_cross / area),
                        _num(e, "max_extrude_only_accel", max_accel * def_cross / area),
                        _num(e, "instantaneous_corner_velocity", 1.0), _num(e, "min_extrude_temp", 170.0),
                        _num(e, "max_temp"))
    sh = k.get("input_shaper", {})
    shapers = {}
    for a in ("x", "y"):
        kind = sh.get(f"shaper_type_{a}", sh.get("shaper_type", "mzv" if f"shaper_freq_{a}" in sh else ""))
        shapers[a] = Shaper(kind.lower() if f"shaper_freq_{a}" in sh else "",
                            _num(sh, f"shaper_freq_{a}", 0.0), _num(sh, f"damping_ratio_{a}", 0.1))
    probe = k.get("probe", {})
    return Printer(name, p.get("kinematics", "cartesian"), max_velocity, max_accel,
                   _num(p, "max_accel_to_decel"), _num(p, "minimum_cruise_ratio"),
                   _num(p, "square_corner_velocity", 5.0), _num(p, "max_z_velocity", max_velocity),
                   _num(p, "max_z_accel", max_accel), axes, extruder, shapers,
                   _num(k.get("heater_bed", {}), "max_temp", 120.0),
                   [_num(probe, "x_offset", 0.0), _num(probe, "y_offset", 0.0)], profile or {})


def load(path) -> Printer:
    path = Path(path)
    if not path.is_absolute() and not path.exists():
        path = HERE / path
    if path.suffix == ".json":
        prof = json.loads(path.read_text(encoding="utf-8"))
        return from_sections(prof["name"], prof["klipper"], prof)
    return from_sections(path.stem, parse_klipper_cfg(path.read_text(encoding="utf-8")))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    ex = sub.add_parser("extract", help="print the 'klipper' block of a profile from a printer.cfg")
    ex.add_argument("cfg")
    sh = sub.add_parser("show", help="print what the twin reads from a profile or config")
    sh.add_argument("path")
    args = ap.parse_args(argv)
    if args.cmd == "extract":
        print(json.dumps(extract(parse_klipper_cfg(Path(args.cfg).read_text(encoding="utf-8"))), indent=2))
    else:
        print(json.dumps(load(args.path).summary(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
