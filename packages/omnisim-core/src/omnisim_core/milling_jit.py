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

"""The MillingStock stamp and sweep as compiled loops.

Written line for line after src/omnisim/nodes/OmMillingStock.cpp: the same
cell order and the same running sums, so the totals accumulate exactly as the
engine's do. Importing this module needs numba; milling.py falls back to its
numpy version without it.
"""

import math

import numpy as np
from numba import njit


@njit(cache=True)
def sweep_jit(h, nx, ny, res, top, ax, ay, az, bx, by, bz, single, shape, diameter, angle, removed, deepest):
    radius = 0.5 * diameter
    changed = False
    if radius <= 0.0:
        return removed, deepest, changed
    r2 = radius * radius
    cell = res * res
    half = 0.5 * min(max(angle, 0.05), 3.0)
    tan_half = math.tan(half)
    dx_ = bx - ax
    dy_ = by - ay
    dz_ = bz - az
    length = math.sqrt(dx_ * dx_ + dy_ * dy_ + dz_ * dz_)
    if single or length > 0.05:
        n = 1
    else:
        n = min(4000, max(1, int(math.ceil(length / (0.5 * res)))))
    for k in range(1, n + 1):
        if single or length > 0.05:
            cx = bx
            cy = by
            tip = bz
        else:
            t = k / n
            cx = ax + dx_ * t
            cy = ay + dy_ * t
            tip = az + dz_ * t
        if tip >= top:
            continue
        i0 = max(0, int(math.floor((cx - radius) / res)))
        i1 = min(nx - 1, int(math.ceil((cx + radius) / res)))
        j0 = max(0, int(math.floor((cy - radius) / res)))
        j1 = min(ny - 1, int(math.ceil((cy + radius) / res)))
        if i0 > i1 or j0 > j1:
            continue
        for j in range(j0, j1 + 1):
            dy = j * res - cy
            for i in range(i0, i1 + 1):
                dx = i * res - cx
                d2 = dx * dx + dy * dy
                if d2 > r2:
                    continue
                if shape == 1:
                    z = tip + radius - math.sqrt(max(0.0, r2 - d2))
                elif shape == 2:
                    z = tip + math.sqrt(d2) / tan_half
                else:
                    z = tip
                if z < 0.0:
                    z = 0.0
                hv = np.float64(h[j, i])
                if z < hv:
                    removed += (hv - z) * cell
                    h[j, i] = np.float32(z)
                    if top - z > deepest:
                        deepest = top - z
                    changed = True
    return removed, deepest, changed
