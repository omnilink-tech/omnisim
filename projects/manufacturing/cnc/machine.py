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

"""Machine description for a 3-axis CNC twin, read from a GRBL `$$` dump or a FluidNC YAML config.

Everything downstream (the G-code interpreter, the planner, the virtual GRBL
controller, the world generator and preflight) reads one `Machine`. Units are
the controller's own: mm, mm/min for rates, mm/s^2 for acceleration.

Coordinates. Each axis has a soft envelope [env_min, env_max] in machine
coordinates (GRBL: [-travel, 0]; FluidNC: from `homing/mpos_mm`). The
physical rail of the twin runs s in [0, travel], s = mpos - env_min once the
machine is homed, with a limit switch at each end. `home_mpos` is the machine
position GRBL assigns after homing, at the pulled-off location.
"""
from __future__ import annotations

import copy
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

AXES = ("x", "y", "z")

# GRBL 1.1 defaults (config.h / defaults.h generic), used when a `$$` dump omits a key.
GRBL_DEFAULTS: Dict[int, float] = {
    0: 10, 1: 25, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 10: 1, 11: 0.010, 12: 0.002, 13: 0,
    20: 0, 21: 0, 22: 0, 23: 0, 24: 25.0, 25: 500.0, 26: 250, 27: 1.0,
    30: 1000.0, 31: 0.0, 32: 0,
    100: 250.0, 101: 250.0, 102: 250.0,
    110: 500.0, 111: 500.0, 112: 500.0,
    120: 10.0, 121: 10.0, 122: 10.0,
    130: 200.0, 131: 200.0, 132: 200.0,
}
INTEGER_SETTINGS = {0, 1, 2, 3, 4, 5, 6, 10, 13, 20, 21, 22, 23, 26, 32}


class MachineConfigError(ValueError):
    pass


@dataclass
class Axis:
    name: str
    travel: float              # mm
    max_rate: float            # mm/min
    accel: float               # mm/s^2
    steps_per_mm: float
    env_min: float             # soft envelope, machine coordinates
    env_max: float
    home_positive: bool = True
    home_mpos: float = 0.0     # machine position after homing (pulled off)

    def inside(self, mpos: float, tol: float = 1e-6) -> bool:
        return self.env_min - tol <= mpos <= self.env_max + tol


@dataclass
class Machine:
    name: str
    flavour: str                         # "grbl" or "fluidnc" (the config source)
    axes: Dict[str, Axis]
    settings: Dict[int, float]           # the GRBL `$` settings the twin reports and honours
    source: str = ""
    notes: List[str] = field(default_factory=list)

    @property
    def soft_limits(self) -> bool:
        return bool(self.settings[20])

    @property
    def hard_limits(self) -> bool:
        return bool(self.settings[21])

    @property
    def homing(self) -> bool:
        return bool(self.settings[22])

    @property
    def pulloff(self) -> float:
        return float(self.settings[27])

    def copy(self) -> "Machine":
        return copy.deepcopy(self)

    def apply_setting(self, key: int, value: float) -> None:
        """Store a `$key=value` and keep the per-axis view in step with it."""
        if key not in GRBL_DEFAULTS:
            raise MachineConfigError(f"unknown setting ${key}")
        self.settings[key] = int(value) if key in INTEGER_SETTINGS else float(value)
        if key in (23, 27) or 100 <= key <= 132:
            _sync_axes_from_settings(self)

    def summary(self) -> dict:
        return {
            "name": self.name, "flavour": self.flavour,
            "axes": {k: {"travel_mm": a.travel, "max_rate_mm_min": a.max_rate,
                         "accel_mm_s2": a.accel, "envelope": [a.env_min, a.env_max],
                         "home_positive": a.home_positive, "home_mpos": a.home_mpos}
                     for k, a in self.axes.items()},
            "soft_limits": self.soft_limits, "hard_limits": self.hard_limits,
            "homing": self.homing, "notes": list(self.notes),
        }


def _sync_axes_from_settings(m: Machine) -> None:
    s = m.settings
    for i, name in enumerate(AXES):
        a = m.axes[name]
        a.steps_per_mm = float(s[100 + i])
        a.max_rate = float(s[110 + i])
        a.accel = float(s[120 + i])
        a.travel = float(s[130 + i])
        if m.flavour == "grbl":
            # GRBL 1.1 (no HOMING_FORCE_SET_ORIGIN): envelope is [-travel, 0] whatever the
            # homing direction; homing leaves the machine at -pulloff or -travel+pulloff.
            a.home_positive = not bool(int(s[23]) >> i & 1)
            a.env_min, a.env_max = -a.travel, 0.0
            a.home_mpos = -float(s[27]) if a.home_positive else -a.travel + float(s[27])


_SETTING_LINE = re.compile(r"^\s*\$(\d+)\s*=\s*([-+]?\d*\.?\d+)")


def from_grbl_settings(text: str, name: str = "grbl machine") -> Machine:
    """Parse the output of `$$` (one `$n=value` per line; `(comments)` and `ok` ignored)."""
    settings = dict(GRBL_DEFAULTS)
    seen = 0
    for line in text.splitlines():
        match = _SETTING_LINE.match(line)
        if not match:
            continue
        key, value = int(match.group(1)), float(match.group(2))
        if key in GRBL_DEFAULTS:
            settings[key] = int(value) if key in INTEGER_SETTINGS else value
            seen += 1
    if not seen:
        raise MachineConfigError("no $n=value settings found")
    axes = {n: Axis(n, 0, 0, 0, 0, 0, 0) for n in AXES}
    m = Machine(name=name, flavour="grbl", axes=axes, settings=settings, source="grbl $$")
    _sync_axes_from_settings(m)
    _validate(m)
    if m.soft_limits and not m.homing:
        m.notes.append("$20=1 needs $22=1 on real GRBL (error:10); soft limits are judged anyway")
    return m


def from_fluidnc_yaml(text: str, name: Optional[str] = None) -> Machine:
    """Read the motion-relevant part of a FluidNC config.yaml.

    Only axes x, y, z are modelled. Pins are read just to know whether limit
    switches exist; FluidNC-only commands ($Config/Dump etc.) are not emulated.
    """
    import yaml  # PyYAML ships with the OmniSim Python environment

    try:
        doc = yaml.safe_load(text) or {}
    except yaml.YAMLError as exc:
        raise MachineConfigError(f"invalid YAML: {exc}") from exc
    cfg_axes = (doc.get("axes") or {})
    settings = dict(GRBL_DEFAULTS)
    settings[11] = float(doc.get("junction_deviation_mm", settings[11]))
    settings[12] = float(doc.get("arc_tolerance_mm", settings[12]))
    axes: Dict[str, Axis] = {}
    any_soft = any_hard = any_homing = False
    pulloff = None
    notes: List[str] = []
    for i, n in enumerate(AXES):
        a = cfg_axes.get(n)
        if not isinstance(a, dict):
            raise MachineConfigError(f"axes/{n} missing")
        travel = float(a.get("max_travel_mm", 1000.0))
        rate = float(a.get("max_rate_mm_per_min", 1000.0))
        accel = float(a.get("acceleration_mm_per_sec2", 25.0))
        spm = float(a.get("steps_per_mm", 320.0))
        homing = a.get("homing") or {}
        positive = bool(homing.get("positive_direction", True))
        mpos = float(homing.get("mpos_mm", 0.0))
        env_min, env_max = (mpos - travel, mpos) if positive else (mpos, mpos + travel)
        motor = a.get("motor0") or {}
        any_soft |= bool(a.get("soft_limits", False))
        any_hard |= bool(motor.get("hard_limits", False))
        if homing and int(homing.get("cycle", 0) or 0) > 0:
            any_homing = True
        if pulloff is None and "pulloff_mm" in motor:
            pulloff = float(motor["pulloff_mm"])
        axes[n] = Axis(n, travel, rate, accel, spm, env_min, env_max, positive, mpos)
        settings[100 + i], settings[110 + i], settings[120 + i], settings[130 + i] = spm, rate, accel, travel
        if homing:
            settings[24] = float(homing.get("feed_mm_per_min", settings[24]))
            settings[25] = float(homing.get("seek_mm_per_min", settings[25]))
    settings[20], settings[21], settings[22] = int(any_soft), int(any_hard), int(any_homing)
    settings[27] = 1.0 if pulloff is None else pulloff
    settings[23] = sum((0 if axes[n].home_positive else 1) << i for i, n in enumerate(AXES))
    spindle = next((v for k, v in doc.items() if isinstance(v, dict) and "speed_map" in v), None)
    if spindle is None and "laser" in doc:
        settings[32] = 1
    m = Machine(name=name or str(doc.get("name") or "fluidnc machine"), flavour="fluidnc",
                axes=axes, settings=settings, source="fluidnc yaml", notes=notes)
    _validate(m)
    return m


def load(path: str | Path, name: Optional[str] = None) -> Machine:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    if p.suffix.lower() in (".yaml", ".yml"):
        return from_fluidnc_yaml(text, name)
    return from_grbl_settings(text, name or p.stem)


def _validate(m: Machine) -> None:
    for a in m.axes.values():
        for label, v in (("travel", a.travel), ("max_rate", a.max_rate), ("accel", a.accel)):
            if not v > 0:
                raise MachineConfigError(f"{a.name} {label} must be positive, got {v}")
