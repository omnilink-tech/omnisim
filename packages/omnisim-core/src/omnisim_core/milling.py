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

"""The engine's MillingStock for the in-process core.

A float32 height map that the cutter's Solid carves at the end of every
simulation tick: the same grid, the same stamps, the same sweep rule, the same
running totals and the same height-map file as src/omnisim/nodes/
OmMillingStock.cpp. CUTTERS ONLY: a laser (toolShape "laser", laserPower) is
not modelled and is refused.
"""

from __future__ import annotations

import math
import os
import struct
import time

import numpy as np

from .errors import CoreUnsupported

K_DEFAULT_RES, K_MIN_RES, K_TELEPORT, K_MAX_STAMPS = 0.0002, 0.00005, 0.05, 4000
SHAPES = {"flat": 0, "ball": 1, "vbit": 2}

try:
    from .milling_jit import sweep_jit
except Exception:          # no numba here: the numpy version below gives the same map, slowly
    sweep_jit = None


class _F:
    """One field of the node, as the controller API exposes it."""

    def __init__(self, get, set_=None):
        self._g, self._s = get, set_

    def getSFFloat(self):
        return float(self._g())

    def getSFVec3f(self):
        return [float(x) for x in self._g()]

    def getSFString(self):
        return str(self._g())

    def getSFBool(self):
        return bool(self._g())

    def _set(self, v):
        if self._s is None:
            raise CoreUnsupported("writing a read-only MillingStock field")
        self._s(v)

    def setSFFloat(self, v):
        self._set(float(v))

    def setSFVec3f(self, v):
        self._set([float(x) for x in v])

    def setSFString(self, v):
        self._set(str(v))

    def setSFBool(self, v):
        self._set(bool(v))


class MillingStock:
    def __init__(self, cfg, core, dump_path=None, use_jit=True):
        if cfg["toolShape"] not in SHAPES:
            raise CoreUnsupported("MillingStock toolShape %r (the core cuts with flat, ball and vbit tools)" % cfg["toolShape"])
        self.core, self.body = core, cfg.get("tool_body")
        sx, sy, sz = cfg["size"]
        self.res = max(cfg["resolution"], K_MIN_RES) if cfg["resolution"] > 0 else K_DEFAULT_RES
        self.nx = int(round(sx / self.res)) + 1
        self.ny = int(round(sy / self.res)) + 1
        self.top = float(sz)
        self.origin = np.array(cfg["translation"], dtype=np.float64)
        self.h = np.full((self.ny, self.nx), np.float32(self.top), dtype=np.float32)
        self.tool_tip = np.array(cfg["toolTip"], dtype=np.float64)
        self.shape, self.diameter, self.angle = cfg["toolShape"], float(cfg["toolDiameter"]), float(cfg["toolAngle"])
        self.cutting = bool(cfg["cutting"])
        self.removed = self.deepest = 0.0
        self.last = None
        self.dump_path = str(dump_path) if dump_path else None
        self.dirty, self.dumped_at = False, None
        self.name = cfg.get("def") or "STOCK"
        self._jit = sweep_jit if use_jit else None
        if self._jit is not None:
            # compile (or load from the on-disk cache) now, before any controller starts talking to a sender
            self._jit(np.full((3, 3), np.float32(1.0), dtype=np.float32), 3, 3, 1.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 2.0,
                      True, 0, 0.5, 1.0, 0.0, 0.0)

    # ---- what a controller sees
    def _set_shape(self, v):
        if v not in SHAPES:
            raise CoreUnsupported("MillingStock toolShape %r" % v)
        self.shape = v

    def _set_laser(self, v):
        if v:
            raise CoreUnsupported("MillingStock laserPower: the core does not model a laser")

    def getField(self, name):
        f = {"removedVolume": _F(lambda: self.removed), "deepestCut": _F(lambda: self.deepest),
             "toolTip": _F(lambda: self.tool_tip, lambda v: setattr(self, "tool_tip", np.array(v, dtype=np.float64))),
             "toolShape": _F(lambda: self.shape, self._set_shape),
             "toolDiameter": _F(lambda: self.diameter, lambda v: setattr(self, "diameter", v)),
             "toolAngle": _F(lambda: self.angle, lambda v: setattr(self, "angle", v)),
             "cutting": _F(lambda: self.cutting, lambda v: setattr(self, "cutting", v)),
             "laserPower": _F(lambda: 0.0, self._set_laser)}.get(name)
        if f is None:
            raise CoreUnsupported("MillingStock field %r" % name)
        return f

    def getDef(self):
        return self.name

    def getPosition(self):
        return [float(x) for x in self.origin]

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        raise CoreUnsupported("MillingStock.%s" % k)

    # ---- the cut (numpy version; milling_jit.sweep_jit is the same rule as compiled loops)
    def _surface(self, d2, tip_z):
        """The tool's surface height over each cell, from the squared distance to the tool axis."""
        radius = 0.5 * self.diameter
        if self.shape == "ball":
            return tip_z + radius - np.sqrt(np.maximum(0.0, radius * radius - d2))
        if self.shape == "vbit":
            half = 0.5 * min(max(self.angle, 0.05), 3.0)
            return tip_z + np.sqrt(d2) / math.tan(half)
        return np.full_like(d2, tip_z)

    def stamp(self, cx, cy, tip_z):
        if tip_z >= self.top:
            return
        radius = 0.5 * self.diameter
        if radius <= 0.0:
            return
        res = self.res
        i0 = max(0, int(math.floor((cx - radius) / res)))
        i1 = min(self.nx - 1, int(math.ceil((cx + radius) / res)))
        j0 = max(0, int(math.floor((cy - radius) / res)))
        j1 = min(self.ny - 1, int(math.ceil((cy + radius) / res)))
        if i0 > i1 or j0 > j1:
            return
        dx = np.arange(i0, i1 + 1) * res - cx
        dy = np.arange(j0, j1 + 1) * res - cy
        d2 = (dx * dx)[None, :] + (dy * dy)[:, None]
        z = np.maximum(0.0, self._surface(d2, tip_z))
        sub = self.h[j0:j1 + 1, i0:i1 + 1]
        lower = (d2 <= radius * radius) & (z < sub)
        if not lower.any():
            return
        zl = z[lower]
        # the engine adds cell by cell, row by row, to its running total: cumsum keeps that order
        self.removed = float(np.cumsum(np.concatenate(([self.removed], (sub[lower].astype(np.float64) - zl) * (res * res))))[-1])
        sub[lower] = zl.astype(np.float32)
        self.deepest = max(self.deepest, float((self.top - zl).max()))
        self.dirty = True

    def sweep(self, a, b):
        d = b - a
        length = float(math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]))
        if length > K_TELEPORT:
            self.stamp(b[0], b[1], b[2])
            return
        n = min(K_MAX_STAMPS, max(1, int(math.ceil(length / (0.5 * self.res)))))
        for k in range(1, n + 1):
            t = k / n
            self.stamp(a[0] + d[0] * t, a[1] + d[1] * t, a[2] + d[2] * t)

    def carve(self, tip):
        """Carve from the last tip position to `tip` (stock coordinates)."""
        if self._jit is not None:
            a = self.last if self.last is not None else tip
            self.removed, self.deepest, ch = self._jit(
                self.h, self.nx, self.ny, self.res, self.top, a[0], a[1], a[2], tip[0], tip[1], tip[2],
                self.last is None, SHAPES[self.shape], self.diameter, self.angle, self.removed, self.deepest)
            if ch:
                self.dirty = True
        elif self.last is not None:
            self.sweep(self.last, tip)
        else:
            self.stamp(tip[0], tip[1], tip[2])
        self.last = tip

    def update(self):
        """End of a simulation tick: carve from where the cutter was to where it is."""
        if self.body is None:
            return
        d = self.core.d
        # the engine places the cutter from the float32 body pose its physics layer hands back
        pos = d.xpos[self.body].astype(np.float32).astype(np.float64)
        tip = pos + d.xmat[self.body].reshape(3, 3) @ self.tool_tip - self.origin
        if not self.cutting:
            self.last = tip
            return
        self.carve(tip)
        if self.dump_path and self.dirty and (self.dumped_at is None or time.perf_counter() - self.dumped_at >= 1.0):
            self.dump()

    def dump(self):
        """The height map as the engine writes it (OMZMAP1: counts, grid, origin, removed volume, float32 heights)."""
        if not self.dump_path:
            return
        self.dirty, self.dumped_at = False, time.perf_counter()
        part = self.dump_path + ".partial"
        with open(part, "wb") as f:
            f.write(b"OMZMAP1\0")
            f.write(struct.pack("<2i", self.nx, self.ny))
            f.write(struct.pack("<6d", self.res, self.top, self.origin[0], self.origin[1], self.origin[2], self.removed))
            f.write(self.h.tobytes())
        os.replace(part, self.dump_path)
