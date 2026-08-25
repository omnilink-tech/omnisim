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

"""Known-map navigation for the OMNITUG500 warehouse courier.

Unlike the explore controller (which builds its map online by lidar frontier
exploration), the courier has the FACILITY MAP up front — exactly like a real
warehouse AGV. We seed a 2D occupancy grid once from the static obstacle
footprints in the layout JSON, inflate it by the rover's circumscribed radius,
and A* over the free cells to any station anchor. Routing is therefore instant
and deterministic; the rover's safety lidar (handled in the bridge) is the
reactive layer on top, not the source of the route.

This module is pure geometry/search — no Webots dependency — so it is unit
testable and shared verbatim by the offline reachability precheck.
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import List, Optional, Tuple

Pt = Tuple[float, float]


class CourierNav:
    def __init__(self, layout: dict) -> None:
        self.x0, self.y0, self.x1, self.y1 = layout["bounds"]
        self.res = float(layout["res"])
        self.inflate = float(layout["inflate"])
        self.nx = int(round((self.x1 - self.x0) / self.res))
        self.ny = int(round((self.y1 - self.y0) / self.res))
        # raw occupied (static obstacles) and inflated-blocked (planner space)
        self._occ = bytearray(self.nx * self.ny)
        for ob in layout["obstacles"]:
            self._stamp_rect(ob["x0"], ob["y0"], ob["x1"], ob["y1"])
        self._blocked = self._inflate(self.inflate)

    # ── cell <-> world ────────────────────────────────────────────
    def w2c(self, x: float, y: float) -> Tuple[int, int]:
        return int((x - self.x0) / self.res), int((y - self.y0) / self.res)

    def c2w(self, ix: int, iy: int) -> Pt:
        return self.x0 + (ix + 0.5) * self.res, self.y0 + (iy + 0.5) * self.res

    def inb(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def _i(self, ix: int, iy: int) -> int:
        return iy * self.nx + ix

    # ── grid build ────────────────────────────────────────────────
    def _stamp_rect(self, x0: float, y0: float, x1: float, y1: float) -> None:
        ix0, iy0 = self.w2c(min(x0, x1), min(y0, y1))
        ix1, iy1 = self.w2c(max(x0, x1), max(y0, y1))
        for iy in range(max(0, iy0), min(self.ny, iy1 + 1)):
            for ix in range(max(0, ix0), min(self.nx, ix1 + 1)):
                self._occ[self._i(ix, iy)] = 1

    def _inflate(self, rad_m: float) -> bytearray:
        rc = int(math.ceil(rad_m / self.res))
        disk = [(dx, dy) for dy in range(-rc, rc + 1) for dx in range(-rc, rc + 1)
                if dx * dx + dy * dy <= rc * rc]
        blk = bytearray(self.nx * self.ny)
        for iy in range(self.ny):
            row = iy * self.nx
            for ix in range(self.nx):
                if self._occ[row + ix]:
                    for dx, dy in disk:
                        nx, ny = ix + dx, iy + dy
                        if self.inb(nx, ny):
                            blk[self._i(nx, ny)] = 1
        return blk

    # ── queries ───────────────────────────────────────────────────
    def is_blocked_cell(self, ix: int, iy: int) -> bool:
        return (not self.inb(ix, iy)) or bool(self._blocked[self._i(ix, iy)])

    def is_occupied(self, x: float, y: float) -> bool:
        ix, iy = self.w2c(x, y)
        return (not self.inb(ix, iy)) or bool(self._occ[self._i(ix, iy)])

    def blocked_within(self, x: float, y: float, r: float) -> bool:
        """True if any raw-occupied static cell lies within r of (x, y).
        The bridge uses this as a final safety guard before a move."""
        rc = int(math.ceil(r / self.res))
        ci, cj = self.w2c(x, y)
        for jy in range(cj - rc, cj + rc + 1):
            for ix in range(ci - rc, ci + rc + 1):
                if self.inb(ix, iy=jy) and self._occ[self._i(ix, jy)]:
                    wx, wy = self.c2w(ix, jy)
                    if math.hypot(wx - x, wy - y) <= r:
                        return True
        return False

    def _nearest_free(self, cell: Tuple[int, int]) -> Tuple[int, int]:
        if not self.is_blocked_cell(*cell):
            return cell
        best = None
        for r in range(1, 12):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    s = (cell[0] + dx, cell[1] + dy)
                    if not self.is_blocked_cell(*s):
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, s)
            if best is not None:
                return best[1]
        return cell

    # ── A* path over free cells ───────────────────────────────────
    def plan(self, start_xy: Pt, goal_xy: Pt) -> Optional[List[Pt]]:
        start = self._nearest_free(self.w2c(*start_xy))
        goal = self._nearest_free(self.w2c(*goal_xy))
        if start == goal:
            return [self.c2w(*goal)]
        openh = [(0.0, start)]
        came = {start: None}
        g = {start: 0.0}
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        while openh:
            _, cur = heapq.heappop(openh)
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(self.c2w(*cur))
                    cur = came[cur]
                return path[::-1]
            cx, cy = cur
            for dx, dy in nbrs:
                nx, ny = cx + dx, cy + dy
                if self.is_blocked_cell(nx, ny):
                    continue
                step = 1.4142 if dx and dy else 1.0
                ng = g[cur] + step
                if (nx, ny) not in g or ng < g[(nx, ny)]:
                    g[(nx, ny)] = ng
                    h = math.hypot(nx - goal[0], ny - goal[1])
                    heapq.heappush(openh, (ng + h, (nx, ny)))
                    came[(nx, ny)] = cur
        return None

    def reachable_from(self, start_xy: Pt) -> set:
        """Flood-fill of free cells reachable from a start (for prechecks)."""
        start = self._nearest_free(self.w2c(*start_xy))
        seen = {start}
        dq = deque([start])
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        while dq:
            cx, cy = dq.popleft()
            for dx, dy in nbrs:
                n = (cx + dx, cy + dy)
                if n not in seen and not self.is_blocked_cell(*n):
                    seen.add(n)
                    dq.append(n)
        return seen
