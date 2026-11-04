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

"""Cutting load: spindle power, torque, cutting force and chip load from material removed.

The engine's MillingStock measures how much material the cutter removes every physics
step. The power a cut takes is that removal rate times the material's SPECIFIC CUTTING
ENERGY u (J/mm^3, numerically N/mm^2 / 1000; 1 MPa = 0.001 J/mm^3):

    P  = u * MRR                     W      (MRR in mm^3/s)
    T  = P / (2 pi n / 60)           N m    spindle torque at n rpm
    Ft = P / vc,  vc = pi D n / 60000  N    mean tangential force at the cutter's rim
    fz = F / (z n)                   mm     chip load per tooth (feed F mm/min, z flutes)

This is the classic mechanistic estimate. Its limits, stated so nobody reads more into it:
u grows as chips get thinner (the size effect), so a fine finishing cut takes more energy
per mm^3 than the table says; the force is a mean over a revolution, not its peaks; and the
table values come from handbooks and papers, not from this machine. One calibration cut on
the real machine replaces them.

Every material carries its source. Where no published figure was found the entry says so
(`u_j_mm3` is None) and the load is reported as unknown rather than guessed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Optional, Tuple

MSU = "MSU ME478 machining notes, specific energy table (0.25 mm chip): https://www.egr.msu.edu/~pkwon/me478/machining.pdf"
MMC = ("Mitsubishi Materials, milling power formula, Kc by feed per tooth: "
       "https://www.mmc-carbide.com/us/technical_information/formula/tec_milling_power_formula")
PB = ("Cutting Forces in Peripheral Up-Milling of Particleboard (Materials, 2021), specific principal "
      "cutting force 32.0-37.6 N/mm^2: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8123317/")


@dataclass(frozen=True)
class Material:
    name: str
    u_j_mm3: Optional[Tuple[float, float]]     # (low, high) specific cutting energy; None = unknown
    source: str
    estimated: bool = False                    # borrowed from a similar material

    @property
    def u_nominal(self) -> Optional[float]:
        return None if self.u_j_mm3 is None else 0.5 * (self.u_j_mm3[0] + self.u_j_mm3[1])


MATERIALS: Dict[str, Material] = {m.name: m for m in (
    Material("aluminium_alloy", (0.7, 0.8), MSU + " (aluminium 0.7, aluminium alloys 0.8)"),
    Material("brass", (0.63, 1.15), MMC + " (brass 630-1150 MPa)"),
    Material("cast_iron", (1.1, 1.6), MSU),
    Material("carbon_steel", (1.6, 2.8), MSU),
    Material("stainless_steel", (2.8, 2.8), MSU),
    Material("particleboard", (0.032, 0.0376), PB),
    Material("mdf", (0.032, 0.0376), PB + " -- MDF itself: no published figure found; particleboard's is used",
             estimated=True),
    Material("acrylic", None, "no published specific cutting energy found for PMMA"),
)}


@dataclass
class Spindle:
    """The spindle's rating. For a router, the rated power is its input power."""
    name: str = "spindle"
    power_w: Optional[float] = None
    rpm: Tuple[float, float] = (0.0, 30000.0)

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "Spindle":
        if not d:
            return cls()
        return cls(d.get("name", "spindle"), d.get("power_w"), tuple(d.get("rpm", (0.0, 30000.0))))


@dataclass
class LoadMeter:
    """Turns the engine's running removed volume into cutting load, averaged over `window_s`."""
    material: Material
    spindle: Spindle = field(default_factory=Spindle)
    window_s: float = 0.1
    _samples: list = field(default_factory=list)
    peak_power_w: float = 0.0
    overloads: list = field(default_factory=list)

    def update(self, t: float, removed_mm3: float) -> None:
        self._samples.append((t, removed_mm3))
        while len(self._samples) > 2 and self._samples[1][0] <= t - self.window_s:
            self._samples.pop(0)

    def mrr_mm3_s(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        (t0, v0), (t1, v1) = self._samples[0], self._samples[-1]
        return max(0.0, (v1 - v0) / (t1 - t0)) if t1 > t0 else 0.0

    def report(self, rpm: float, feed_mm_min: float, diameter_mm: float, flutes: int, t: float = 0.0) -> dict:
        mrr = self.mrr_mm3_s()
        out = {"material": self.material.name, "material_estimated": self.material.estimated,
               "mrr_mm3_s": round(mrr, 2),
               "chip_load_mm": round(feed_mm_min / (flutes * rpm), 4) if rpm > 0 and flutes > 0 else None}
        u = self.material.u_nominal
        if u is None or mrr <= 0.0:
            out.update(power_w=None if u is None else 0.0, torque_nm=None if u is None else 0.0,
                       force_n=None if u is None else 0.0)
            if u is None:
                out["note"] = self.material.source
            return out
        power = u * mrr
        omega = 2.0 * math.pi * rpm / 60.0
        vc = math.pi * diameter_mm * rpm / 60000.0
        out.update(power_w=round(power, 1),
                   power_range_w=[round(self.material.u_j_mm3[0] * mrr, 1), round(self.material.u_j_mm3[1] * mrr, 1)],
                   torque_nm=round(power / omega, 4) if omega > 0 else None,
                   force_n=round(power / vc, 2) if vc > 0 else None)
        self.peak_power_w = max(self.peak_power_w, power)
        out["peak_power_w"] = round(self.peak_power_w, 1)
        if self.spindle.power_w:
            out["spindle_load"] = round(power / self.spindle.power_w, 3)
            if power > self.spindle.power_w and (not self.overloads or t - self.overloads[-1]["t"] > 1.0):
                self.overloads.append({"t": round(t, 3), "power_w": round(power, 1)})
        return out
