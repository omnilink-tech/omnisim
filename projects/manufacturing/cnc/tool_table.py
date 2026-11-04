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

"""The machine's tool table: which cutters exist and how long each one sticks out.

A tool's `stickout_mm` is how much LOWER its tip sits than tool 1's in the same
collet height, so tool 1 is 0 by definition. Work Z is zeroed with tool 1 in the
spindle (the bridge and the demos do), so after a change the sender applies
`G43.1 Z<stickout>`: the same work Z then puts the new tip at the same height.
On a real machine that number comes from a tool setter; the twin knows it.

JSON layout:
    {"tools": [{"number": 1, "name": "...", "shape": "flat", "diameter_mm": 6.35,
                "angle_deg": 90, "flutes": 2, "stickout_mm": 0.0,
                "flute_mm": 24, "shank_mm": 6.35}, ...],
     "spindle": {"name": "...", "power_w": 700, "rpm": [10000, 30000], "source": "..."}}
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import re
from typing import Dict, Optional, Tuple

from gcode import strip_line

SHAPES = ("flat", "ball", "vbit")
_M6 = re.compile(r"M0*6(?![0-9.])")
_T = re.compile(r"T(\d+)")


@dataclass(frozen=True)
class Tool:
    number: int
    shape: str = "flat"
    diameter_mm: float = 6.35
    angle_deg: float = 90.0        # V-bit included angle
    flutes: int = 2
    stickout_mm: float = 0.0       # tip below tool 1's, mm
    name: str = ""
    flute_mm: float = 0.0          # cutting length, for drawing it (0: three diameters)
    shank_mm: float = 0.0          # shank diameter, for drawing it (0: the cutting diameter)

    def __post_init__(self):
        if self.shape not in SHAPES:
            raise ValueError(f"tool {self.number}: shape must be one of {SHAPES}, not {self.shape!r}")
        if not 1 <= self.number <= 255:
            raise ValueError(f"tool {self.number}: GRBL tool numbers are 1..255")
        if self.diameter_mm <= 0:
            raise ValueError(f"tool {self.number}: diameter must be positive")

    @property
    def radius_mm(self) -> float:
        return self.diameter_mm / 2.0

    def surface_mm(self, d: float) -> float:
        """Height of the cutter's bottom above its tip at radial distance d (d <= radius)."""
        if self.shape == "ball":
            r = self.radius_mm
            return r - math.sqrt(max(0.0, r * r - d * d))
        if self.shape == "vbit":
            return d / math.tan(math.radians(max(1.0, min(179.0, self.angle_deg))) / 2.0)
        return 0.0

    def summary(self) -> dict:
        return {k: v for k, v in asdict(self).items() if v != ""}


def default_tools(diameter_mm: float = 6.35) -> Dict[int, Tool]:
    """A one-tool table: the flat end mill the machine powers up with."""
    return {1: Tool(1, "flat", diameter_mm, name=f"{diameter_mm:g} mm flat end mill")}


def load(path: Optional[Path]) -> Dict[int, Tool]:
    if path is None:
        return default_tools()
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    tools = {int(t["number"]): Tool(**t) for t in data["tools"]}
    if 1 not in tools:
        raise ValueError(f"{path}: the table needs tool 1 (the tool work Z is zeroed with)")
    if tools[1].stickout_mm != 0.0:
        raise ValueError(f"{path}: tool 1 is the length reference; its stickout_mm must be 0")
    return tools


def load_spindle(path: Optional[Path]) -> Optional[dict]:
    """The table's `spindle` entry (rating for the cutting-load model), or None."""
    if path is None:
        return None
    return json.loads(Path(path).read_text(encoding="utf-8")).get("spindle")


def tool_change_in(raw: str) -> Tuple[str, bool, Optional[int]]:
    """Split a G-code line the way a sender does before GRBL sees it.

    Returns (what to send to GRBL, whether the line asks for M6, the T number on it).
    GRBL 1.1 answers M6 with error:20, so the M6 word is removed and the sender
    performs the change itself.
    """
    s = strip_line(raw)
    t = _T.search(s)
    number = int(t.group(1)) if t else None
    if not _M6.search(s):
        return raw.strip(), False, number
    return _M6.sub("", s), True, number
