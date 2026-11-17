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
"""Route planning for a wheeled base: around keep-out zones and known obstacles.

A drive_to used to be "turn, then drive straight". Straight through a
pedestrian aisle the robot has been told to keep out of, it was clipped at
the aisle's edge and stopped short; straight into a pallet, it stalled and
reported blocked. A warehouse cart plans instead: it keeps a map of where it
may not go (the operator's keep-out rules) and of what it has run into (a
blocked drive leaves an obstacle behind), and routes around both.

The planner is deliberately small: A* on an 8-connected grid over the site,
every forbidden region inflated by the robot's radius, then the cell path
shortened to the fewest straight legs that stay clear. It knows nothing
about any particular task.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


class NavMap:
    """What the robot may not enter, in world metres."""

    def __init__(self, half_x: float, half_y: float, robot_radius: float = 0.55,
                 resolution: float = 0.1, wall_margin: float = 0.1):
        self.half_x, self.half_y = float(half_x), float(half_y)
        self.robot_radius = float(robot_radius)
        self.res = float(resolution)
        self.wall_margin = float(wall_margin)
        self.nx = int(round(2 * self.half_x / self.res)) + 1
        self.ny = int(round(2 * self.half_y / self.res)) + 1
        # Obstacles the robot has met: [{x, y, radius, note}]
        self.obstacles: List[Dict] = []

    # ── what is forbidden ───────────────────────────────────────────────

    def learn_obstacle(self, x: float, y: float, radius: float = 0.7, note: str = "") -> Dict:
        for ob in self.obstacles:
            if math.hypot(ob["x"] - x, ob["y"] - y) < 0.3:
                ob["radius"] = max(ob["radius"], radius)
                return ob
        ob = {"x": x, "y": y, "radius": radius, "note": note}
        self.obstacles.append(ob)
        return ob

    def forget_obstacles(self) -> int:
        n = len(self.obstacles)
        self.obstacles = []
        return n

    def blocked(self, x: float, y: float, zones: Sequence[Dict] = (),
                bounds: Sequence[Dict] = (), clearance: Optional[float] = None) -> Optional[str]:
        """Why the robot's CENTRE may not be at (x, y), or None if it may."""
        r = self.robot_radius if clearance is None else clearance
        if abs(x) > self.half_x - self.wall_margin - r or abs(y) > self.half_y - self.wall_margin - r:
            return "site edge"
        for z in zones:
            if (z["x_min"] - r < x < z["x_max"] + r) and (z["y_min"] - r < y < z["y_max"] + r):
                return f"zone {z.get('name') or z.get('id') or ''}".strip()
        for b in bounds:
            v = x if b.get("axis") == "x" else y
            # A line the robot's CENTRE must not cross; the robot may drive
            # right up to it (that is what "stop at the line" means).
            if "max" in b and v > b["max"] + 1e-6:
                return f"boundary {b.get('axis')} <= {b['max']:.2f}"
            if "min" in b and v < b["min"] - 1e-6:
                return f"boundary {b.get('axis')} >= {b['min']:.2f}"
        for ob in self.obstacles:
            if math.hypot(x - ob["x"], y - ob["y"]) < ob["radius"] + r:
                return f"obstacle at ({ob['x']:.2f}, {ob['y']:.2f})"
        return None

    def depth(self, x: float, y: float, zones: Sequence[Dict] = (),
              bounds: Sequence[Dict] = ()) -> float:
        """How far inside the forbidden margins (x, y) lies; 0 when clear."""
        r = self.robot_radius
        d = 0.0
        d = max(d, abs(x) - (self.half_x - self.wall_margin - r), abs(y) - (self.half_y - self.wall_margin - r))
        for z in zones:
            dx = min(x - (z["x_min"] - r), (z["x_max"] + r) - x)
            dy = min(y - (z["y_min"] - r), (z["y_max"] + r) - y)
            if dx > 0 and dy > 0:
                d = max(d, min(dx, dy))
        for b in bounds:
            v = x if b.get("axis") == "x" else y
            if "max" in b: d = max(d, v - b["max"])
            if "min" in b: d = max(d, b["min"] - v)
        for ob in self.obstacles:
            d = max(d, ob["radius"] + r - math.hypot(x - ob["x"], y - ob["y"]))
        return max(0.0, d)

    # ── planning ────────────────────────────────────────────────────────

    def _cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(round((x + self.half_x) / self.res)), int(round((y + self.half_y) / self.res)))

    def _xy(self, c: Tuple[int, int]) -> Point:
        return (c[0] * self.res - self.half_x, c[1] * self.res - self.half_y)

    def segment_clear(self, a: Point, b: Point, zones=(), bounds=(), clearance=None) -> bool:
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res / 2)) + 1)
        for i in range(n + 1):
            t = i / n
            if self.blocked(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t,
                            zones, bounds, clearance):
                return False
        return True

    def plan(self, start: Point, goal: Point, zones: Sequence[Dict] = (),
             bounds: Sequence[Dict] = ()) -> Dict:
        """Plan with the full clearance; a goal that lies OUTSIDE every zone
        but inside its clearance margin (a charger 0.2 m from a fire-door
        corridor) is still reachable: the route is planned again with the
        margin shrunk to what the goal allows, never below MIN_CLEARANCE_M."""
        out = self._plan(start, goal, zones, bounds)
        if out["ok"] or out.get("hard") or not out.get("goal_margin"):
            return out
        room = self.clear_distance(goal[0], goal[1], zones, bounds)
        tight = max(self.MIN_CLEARANCE_M, room - 0.05)
        if tight >= self.robot_radius:
            return out
        full, self.robot_radius = self.robot_radius, tight
        try:
            again = self._plan(start, goal, zones, bounds)
        finally:
            self.robot_radius = full
        if again["ok"]:
            again["tight_clearance_m"] = round(tight, 3)
        return again

    MIN_CLEARANCE_M = 0.1

    def clear_distance(self, x: float, y: float, zones: Sequence[Dict] = (),
                       bounds: Sequence[Dict] = ()) -> float:
        """Distance from (x, y) to the nearest zone edge or line (inf if none)."""
        d = math.inf
        for z in zones:
            dx = max(z["x_min"] - x, 0.0, x - z["x_max"])
            dy = max(z["y_min"] - y, 0.0, y - z["y_max"])
            d = min(d, math.hypot(dx, dy))
        for b in bounds:
            v = x if b.get("axis") == "x" else y
            if "max" in b: d = min(d, b["max"] - v)
            if "min" in b: d = min(d, v - b["min"])
        return d

    def _plan(self, start: Point, goal: Point, zones: Sequence[Dict] = (),
              bounds: Sequence[Dict] = ()) -> Dict:
        """{"ok": True, "waypoints": [...]} or {"ok": False, "why": ...}.

        The start may lie inside an inflated region (the robot stopped right
        against an obstacle, or was pushed into a margin): leaving it is
        allowed, entering one is not. The GOAL must be clear.
        """
        why = self.blocked(goal[0], goal[1], zones, bounds)
        if why:
            # Strictly inside a zone or across a line: impossible as asked.
            hard = self.blocked(goal[0], goal[1], zones, bounds, clearance=0.0)
            return {"ok": False, "why": f"the goal is inside {hard or why}", "hard": bool(hard),
                    "goal_margin": not hard and (why.startswith("zone") or why.startswith("boundary"))}
        if self.segment_clear(start, goal, zones, bounds):
            return {"ok": True, "waypoints": [goal], "direct": True}
        s, g = self._cell(*start), self._cell(*goal)
        free_cache: Dict[Tuple[int, int], bool] = {}

        def free(c):
            if c not in free_cache:
                x, y = self._xy(c)
                free_cache[c] = (0 <= c[0] < self.nx and 0 <= c[1] < self.ny
                                 and self.blocked(x, y, zones, bounds) is None)
            return free_cache[c]

        depth_cache: Dict[Tuple[int, int], float] = {}

        def depth_of(c):
            if c not in depth_cache:
                depth_cache[c] = self.depth(*self._xy(c), zones, bounds)
            return depth_cache[c]

        start_inside = not free(s)
        steps = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
        openq = [(0.0, 0.0, s)]
        came: Dict[Tuple[int, int], Tuple[int, int]] = {}
        cost = {s: 0.0}
        found = False
        while openq:
            _, gc, c = heapq.heappop(openq)
            if c == g:
                found = True; break
            if gc > cost.get(c, math.inf):
                continue
            here_free = free(c)
            for dx, dy, w in steps:
                n = (c[0] + dx, c[1] + dy)
                if not (0 <= n[0] < self.nx and 0 <= n[1] < self.ny):
                    continue
                # Leaving a forbidden margin the robot already stands in is
                # allowed, but only by getting SHALLOWER at every step --
                # never by tunnelling through the thing it is up against.
                if not free(n):
                    if here_free or not start_inside:
                        continue
                    if depth_of(n) >= depth_of(c):
                        continue
                ng = gc + w
                if ng < cost.get(n, math.inf):
                    cost[n] = ng; came[n] = c
                    h = math.hypot(g[0] - n[0], g[1] - n[1])
                    heapq.heappush(openq, (ng + h, ng, n))
        if not found:
            return {"ok": False, "why": "no route stays clear of the keep-out zones and obstacles",
                    "hard": False}
        cells = [g]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        cells.reverse()
        pts = [start] + [self._xy(c) for c in cells[1:-1]] + [goal]
        if start_inside:
            # Keep the escape legs as planned: every point on them is inside
            # a margin, so a line-of-sight shortcut cannot be tested there.
            k = next((i for i, c in enumerate(cells) if free(c)), len(cells) - 1)
            head = [self._xy(c) for c in cells[1:k + 1]]
            tail = self._shorten([self._xy(cells[k])] + pts[k + 1:], zones, bounds) if k < len(cells) - 1 else []
            # Merge only straight runs of the escape; its corners stay, so
            # the robot follows the same ever-shallower path the search took.
            bends = [head[i] for i in range(len(head))
                     if i == len(head) - 1 or i == 0
                     or (round(head[i][0] - head[i - 1][0], 6), round(head[i][1] - head[i - 1][1], 6))
                     != (round(head[i + 1][0] - head[i][0], 6), round(head[i + 1][1] - head[i][1], 6))]
            wps = [(round(x, 3), round(y, 3)) for x, y in bends[1:] if bends] + tail
            return {"ok": True, "waypoints": wps, "direct": False, "escaped": True}
        return {"ok": True, "waypoints": self._shorten(pts, zones, bounds), "direct": False}

    def _shorten(self, pts: List[Point], zones, bounds) -> List[Point]:
        """Fewest straight legs: from each kept point, jump to the farthest
        later point still in clear line of sight."""
        out, i = [], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.segment_clear(pts[i], pts[j], zones, bounds):
                j -= 1
            out.append(pts[j]); i = j
        return [(round(x, 3), round(y, 3)) for x, y in out]


def obstacle_ahead(x: float, y: float, yaw: float, half_length: float,
                   gap: float = 0.2, radius: float = 0.7) -> Dict:
    """Where a drive that stalled head-on has most likely met something."""
    d = half_length + gap
    return {"x": x + d * math.cos(yaw), "y": y + d * math.sin(yaw), "radius": radius}
