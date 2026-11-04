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

"""The physical side of the twin: where each carriage really is.

GRBL only ever knows its *belief* (MPos, counted from steps). The rail model
keeps the carriage's physical position s on each axis, so the twin can show
what a real machine would do when belief and reality disagree:

- at power-up GRBL believes MPos 0 wherever the carriage actually sits;
- a limit switch at each rail end trips when the carriage passes it;
- past the switch is a mechanical end stop. Driving into it stalls the
  carriage while GRBL keeps counting (lost steps): belief and reality part.

s = mpos - env_min + offset. `offset` is 0 on a homed machine whose rail
matches its configured travel. A rail shorter or longer than `$13x` (the
"as-built" length, optional) shows up as a nonzero offset after homing, which
is how a wrong travel setting becomes a crash the twin can report.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from machine import AXES, Machine

SWITCH_OVERTRAVEL_MM = 0.05     # a switch trips this far past the rail end
END_STOP_MARGIN_MM = 5.0        # mechanical stop beyond the switch


@dataclass
class Rail:
    axis: str
    length: float                 # physical rail between the two switches, mm
    env_min: float
    offset: float = 0.0
    commanded: float = 0.0        # physical command after the end stops (s, mm)
    measured: float = 0.0         # what the plant reports back (ideal: = commanded)
    lost_steps_mm: float = 0.0    # accumulated stall, signed

    def switch(self, s: Optional[float] = None) -> Optional[str]:
        s = self.measured if s is None else s
        if s <= -SWITCH_OVERTRAVEL_MM:
            return "min"
        if s >= self.length + SWITCH_OVERTRAVEL_MM:
            return "max"
        return None

    def stop_min(self) -> float:
        return -END_STOP_MARGIN_MM

    def stop_max(self) -> float:
        return self.length + END_STOP_MARGIN_MM


@dataclass
class Plant:
    """Ideal plant: the carriage goes where it is told, unless an end stop is in the way.

    The simulator controller replaces `measured` with the PositionSensor reading
    each tick; everything else is shared.
    """
    machine: Machine
    rails: Dict[str, Rail] = field(default_factory=dict)
    stalls: List[dict] = field(default_factory=list)

    @classmethod
    def build(cls, machine: Machine, initial_s: Optional[Sequence[float]] = None,
              rail_lengths: Optional[Dict[str, float]] = None) -> "Plant":
        rails = {}
        for i, a in enumerate(AXES):
            ax = machine.axes[a]
            length = float((rail_lengths or {}).get(a, ax.travel))
            s0 = length / 2.0 if initial_s is None else float(initial_s[i])
            # Power-up: GRBL believes MPos 0 wherever the carriage is.
            rail = Rail(a, length, ax.env_min, offset=s0 - (0.0 - ax.env_min))
            rail.commanded = rail.measured = s0
            rails[a] = rail
        return cls(machine, rails)

    def to_physical(self, axis: str, mpos: float) -> float:
        r = self.rails[axis]
        return mpos - r.env_min + r.offset

    def command(self, mpos: Sequence[float], time_s: float = 0.0) -> List[float]:
        """Apply a belief position; returns the physical command per axis (mm)."""
        out = []
        for i, a in enumerate(AXES):
            r = self.rails[a]
            s = self.to_physical(a, mpos[i])
            clamped = min(max(s, r.stop_min()), r.stop_max())
            if clamped != s:
                # The motor keeps stepping against the stop: GRBL's count moves on, the
                # carriage does not. Fold the difference into the offset permanently.
                excess = s - clamped
                r.offset -= excess
                r.lost_steps_mm += excess
                if not self.stalls or self.stalls[-1].get("axis") != a or time_s - self.stalls[-1]["t"] > 0.5:
                    self.stalls.append({"t": round(time_s, 4), "axis": a,
                                        "end": "max" if excess > 0 else "min"})
                s = clamped
            r.commanded = s
            out.append(s)
        return out

    def settle_ideal(self) -> None:
        for r in self.rails.values():
            r.measured = r.commanded

    def switches(self) -> str:
        return "".join(a.upper() for a in AXES if self.rails[a].switch())

    def rebase(self, axis: str, mpos: float) -> None:
        """Homing: the commanded physical position is now `mpos` in GRBL's belief."""
        r = self.rails[axis]
        r.offset = r.commanded - (mpos - r.env_min)

    def measured_mpos(self) -> List[float]:
        """The real carriage position expressed in GRBL's current belief frame."""
        return [self.rails[a].measured + self.rails[a].env_min - self.rails[a].offset for a in AXES]

    def snapshot(self) -> dict:
        return {a: {"rail_mm": r.length, "commanded_s": round(r.commanded, 4),
                    "measured_s": round(r.measured, 4), "offset_mm": round(r.offset, 4),
                    "lost_steps_mm": round(r.lost_steps_mm, 4), "switch": r.switch()}
                for a, r in self.rails.items()}
