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

"""Material removal on a block of stock, as a height map carved by a flat end mill.

Geometry only: where the tool has been, the stock is gone. No cutting forces,
chip load, deflection, burning or tear-out. The tool position is whatever the
caller passes, so in the twin it is the MEASURED carriage position: a lagging
or stalled axis cuts where the tool really went, not where GRBL believed.

The block is a grid of vertex heights (mm above the stock's bottom). The outer
ring of vertices is pinned to 0, which draws the block's sides when the grid is
rendered as an ElevationGrid (OmniSim's renderer ignores `thickness`).
Index convention matches ElevationGrid: height[i + j * nx] sits at
(i * spacing, j * spacing).
"""
from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np


class Stock:
    def __init__(self, size_x: float, size_y: float, thickness: float, spacing: float = 1.0):
        if min(size_x, size_y, thickness, spacing) <= 0:
            raise ValueError("stock dimensions and spacing must be positive")
        self.spacing = float(spacing)
        self.nx = int(round(size_x / spacing)) + 1
        self.ny = int(round(size_y / spacing)) + 1
        self.thickness = float(thickness)
        self.h = np.full((self.ny, self.nx), self.thickness, dtype=np.float64)
        self._pin_border()
        self.initial = self.h.copy()
        self.removed_mm3 = 0.0
        self.dry_cuts: List[dict] = []    # material entered with the spindle stopped
        self.engine: Optional[dict] = None  # the engine MillingStock's own figures, when it carves

    def _pin_border(self) -> None:
        self.h[0, :] = self.h[-1, :] = 0.0
        self.h[:, 0] = self.h[:, -1] = 0.0

    def flat_heights(self) -> List[float]:
        """ElevationGrid order: i (x) fastest, then j (y)."""
        return [float(v) for v in self.h.ravel()]

    def cut(self, p0: Sequence[float], p1: Sequence[float], radius: float,
            spindle_on: bool = True, t: float = 0.0) -> np.ndarray:
        """Sweep the tool tip from p0 to p1 (stock-local mm). Returns flat indices that changed."""
        x0, y0, z0 = (float(v) for v in p0)
        x1, y1, z1 = (float(v) for v in p1)
        if min(z0, z1) >= self.thickness:
            return np.empty(0, dtype=np.int64)
        s = self.spacing
        i_lo = max(1, int(np.floor((min(x0, x1) - radius) / s)))
        i_hi = min(self.nx - 2, int(np.ceil((max(x0, x1) + radius) / s)))
        j_lo = max(1, int(np.floor((min(y0, y1) - radius) / s)))
        j_hi = min(self.ny - 2, int(np.ceil((max(y0, y1) + radius) / s)))
        if i_lo > i_hi or j_lo > j_hi:
            return np.empty(0, dtype=np.int64)
        ii, jj = np.meshgrid(np.arange(i_lo, i_hi + 1), np.arange(j_lo, j_hi + 1))
        px, py = ii * s, jj * s
        dx, dy = x1 - x0, y1 - y0
        seg2 = dx * dx + dy * dy
        if seg2 > 1e-12:
            u = np.clip(((px - x0) * dx + (py - y0) * dy) / seg2, 0.0, 1.0)
        else:
            u = np.zeros_like(px, dtype=np.float64)
        cx, cy = x0 + u * dx, y0 + u * dy
        inside = (px - cx) ** 2 + (py - cy) ** 2 <= radius * radius
        if not inside.any():
            return np.empty(0, dtype=np.int64)
        z = np.maximum(z0 + u * (z1 - z0), 0.0)        # a through-cut stops at the bottom
        block = self.h[j_lo:j_hi + 1, i_lo:i_hi + 1]
        lower = inside & (z < block)
        if not lower.any():
            return np.empty(0, dtype=np.int64)
        removed = float(np.sum(block[lower] - z[lower])) * s * s
        if not spindle_on:
            self.dry_cuts.append({"t": round(t, 4), "at_mm": [round(x1, 2), round(y1, 2), round(z1, 2)],
                                  "depth_mm": round(float(np.max(block[lower] - z[lower])), 3)})
        block[lower] = z[lower]
        self.removed_mm3 += removed
        return (jj[lower] * self.nx + ii[lower]).astype(np.int64)

    def depth_at(self, x: float, y: float) -> float:
        """Material removed below the original top at the nearest vertex, mm."""
        i = int(round(x / self.spacing))
        j = int(round(y / self.spacing))
        return float(self.initial[j, i] - self.h[j, i])

    def summary(self) -> dict:
        cut = self.h < self.initial - 1e-9
        return {"removed_mm3": round(self.removed_mm3, 1), "cut_vertices": int(cut.sum()),
                "deepest_mm": round(float(np.max(self.initial - self.h)), 3),
                "dry_cuts": self.dry_cuts[-5:], "dry_cut_count": len(self.dry_cuts),
                "engine": self.engine}


def tool_tip(tip_at_cad: Sequence[float], cad_s: Sequence[float], axis_dirs: Sequence[Sequence[float]],
             s: Sequence[float]) -> List[float]:
    """Tool tip (world mm) for rail positions s, from where it sits in the CAD snapshot."""
    return [tip_at_cad[k] + sum(axis_dirs[a][k] * (s[a] - cad_s[a]) for a in range(3)) for k in range(3)]


def parse_triple(text: Optional[str]) -> Optional[List[float]]:
    return [float(v) for v in text.split(",")] if text else None
