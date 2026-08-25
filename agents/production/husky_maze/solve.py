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

"""Standalone husky-maze solver — drives the bridge directly without OmniLink.

This script exists for two reasons:

1. It's the fastest way to validate that the OmniSim-side bridge works.
2. It demonstrates the gap that makes the OmniLink agent worth its keep:
   the script has to commit to a strategy *at compile time* via if/else,
   while the agent picks the strategy at runtime by reading
   `capabilities.map_available` and reasoning about it. See
   docs/why-an-agent.md for the full discussion.

Strategies (selected by the bridge's map_available flag):

* **BFS over the known map** — fast, deterministic. Used when
  try_get_known_map returns {available: true}. The bridge's gate is the
  world title: "Husky Maze" reveals the map; "Husky Maze (Unknown)"
  doesn't.

* **Right-hand-rule wall-follow on lidar** — slower, but works on any
  perfect maze without prior knowledge. Used when the bridge says the
  map is unavailable.

Usage:
    launch.bat projects\\samples\\demos\\worlds\\flagship\\husky_maze.omniworld
    python agents/production/husky_maze/solve.py

    # or, for the unknown world:
    launch.bat projects\\samples\\demos\\worlds\\flagship\\husky_maze_unknown.omniworld
    python agents/production/husky_maze/solve.py

Environment overrides:
    HUSKY_BRIDGE_URL    default http://127.0.0.1:6070
    HUSKY_SOLVE_SPEED   fraction of max_linear, default 0.5
    HUSKY_SOLVE_TIMEOUT per-cell timeout in seconds, default 25
    HUSKY_SOLVE_MAX_STEPS  hard cap on wall-follow steps, default 400
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Tuple

BRIDGE_URL = os.environ.get("HUSKY_BRIDGE_URL", "http://127.0.0.1:6070").rstrip("/")
SPEED = float(os.environ.get("HUSKY_SOLVE_SPEED", "0.5"))
PER_CELL_TIMEOUT = float(os.environ.get("HUSKY_SOLVE_TIMEOUT", "25"))
MAX_WALLFOLLOW_STEPS = int(os.environ.get("HUSKY_SOLVE_MAX_STEPS", "400"))


# ---------------------------------------------------------------------------
# HTTP plumbing
# ---------------------------------------------------------------------------

def _http(method: str, endpoint: str, payload=None, timeout=5.0):
    url = f"{BRIDGE_URL}/{endpoint.lstrip('/')}"
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", errors="replace")
    except urllib.error.URLError as e:
        raise RuntimeError(
            f"bridge unreachable at {url}: "
            f"{e.reason if hasattr(e, 'reason') else e}"
        )
    return json.loads(raw) if raw else {}


def _wait_until(predicate, timeout: float, poll_s: float = 0.2):
    deadline = time.time() + timeout
    last = {}
    while time.time() < deadline:
        last = _http("GET", "state")
        if predicate(last):
            return True, last
        time.sleep(poll_s)
    return False, last


# ---------------------------------------------------------------------------
# Cell helpers
# ---------------------------------------------------------------------------

def world_to_cell(x: float, y: float, maze: dict) -> Tuple[int, int]:
    cs = maze["cell_size_m"]
    col = round((x - maze["origin_x"]) / cs)
    row = round((y - maze["origin_y"]) / cs)
    cells = maze["cols"]
    col = max(0, min(cells - 1, col))
    row = max(0, min(cells - 1, row))
    return col, row


# ---------------------------------------------------------------------------
# Strategy A: BFS over the known map
# ---------------------------------------------------------------------------

def bfs(adjacency: Dict[str, List[List[int]]], start, goal):
    if start == goal:
        return [start]
    seen = {start: None}
    q = deque([start])
    while q:
        here = q.popleft()
        if here == goal:
            break
        for nb in adjacency.get(f"{here[0]},{here[1]}", []):
            n = (nb[0], nb[1])
            if n in seen:
                continue
            seen[n] = here
            q.append(n)
    if goal not in seen:
        return []
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = seen[cur]
    path.reverse()
    return path


def solve_known_map(caps: dict) -> int:
    print("[solve] strategy: BFS over known map")
    maze_resp = _http("GET", "maze")
    if not maze_resp.get("available"):
        print(f"[solve] map turned out unavailable: {maze_resp}")
        return 11
    state = _http("GET", "state")
    here = world_to_cell(state["x"], state["y"], caps["maze"])
    goal = (caps["maze"]["goal"]["col"], caps["maze"]["goal"]["row"])
    path = bfs(maze_resp["adjacency"], here, goal)
    if not path:
        print(f"[solve] no path {here} -> {goal}")
        return 12
    print(f"[solve] plan: {len(path)} cells from {here} to {goal}")
    return _drive_path(path)


# ---------------------------------------------------------------------------
# Strategy B: right-hand-rule wall-follow on lidar
# ---------------------------------------------------------------------------

# Heading enum: 0=east, 1=north, 2=west, 3=south. Maps to (dcol, drow).
_HEADING_DELTAS = [(1, 0), (0, 1), (-1, 0), (0, -1)]
_HEADING_NAMES = ["E", "N", "W", "S"]


def _yaw_to_heading(yaw: float) -> int:
    """Snap a continuous yaw to the nearest cardinal {0,1,2,3}."""
    # +x = east -> heading 0; +y = north -> heading 1.
    options = [0.0, math.pi / 2, math.pi, -math.pi / 2]
    best = 0
    best_err = float("inf")
    for i, theta in enumerate(options):
        err = abs(math.atan2(math.sin(yaw - theta), math.cos(yaw - theta)))
        if err < best_err:
            best_err = err
            best = i
    return best


def _is_clear(ranges, angles, body_angle: float, threshold: float) -> bool:
    """Return True if the lidar range at the cardinal body angle is past
    `threshold`. Picks the ray closest to body_angle (16 rays = ~22.5 deg
    spacing, so the worst-case error is ~11 deg)."""
    target = math.atan2(math.sin(body_angle), math.cos(body_angle))
    best_i = 0
    best_err = float("inf")
    for i, a in enumerate(angles):
        err = abs(math.atan2(math.sin(a - target), math.cos(a - target)))
        if err < best_err:
            best_err = err
            best_i = i
    return ranges[best_i] >= threshold


def _try_directions(heading: int, lidar: dict, threshold: float) -> List[int]:
    """Return relative-direction preference list. Right-hand rule:
    [right, forward, left, back]. Each entry is an absolute heading 0..3."""
    angles = lidar["angles_rad"]
    ranges = lidar["ranges_m"]
    # Body angles for relative right/forward/left/back.
    rel_pairs = [
        (-math.pi / 2, (heading - 1) % 4),  # right
        (0.0, heading),                      # forward
        (+math.pi / 2, (heading + 1) % 4),   # left
        (math.pi, (heading + 2) % 4),        # back
    ]
    return [hd for body_angle, hd in rel_pairs
            if _is_clear(ranges, angles, body_angle, threshold)]


def solve_unknown_map(caps: dict) -> int:
    print("[solve] strategy: right-hand-rule wall-follow on lidar")
    cell_size = caps["maze"]["cell_size_m"]
    clearance = 0.4
    open_threshold = cell_size + clearance  # >2.4 m means the next cell is free
    goal_cell = (caps["maze"]["goal"]["col"], caps["maze"]["goal"]["row"])

    state = _http("GET", "state")
    if state.get("goal_reached"):
        print("[solve] already at the goal")
        return 0
    here = world_to_cell(state["x"], state["y"], caps["maze"])
    heading = _yaw_to_heading(state["yaw"])
    print(f"[solve] start cell={here} heading={_HEADING_NAMES[heading]} goal={goal_cell}")

    visited = {here: 1}
    for step in range(1, MAX_WALLFOLLOW_STEPS + 1):
        lidar = _http("GET", "lidar")
        candidates = _try_directions(heading, lidar, open_threshold)
        if not candidates:
            print(f"[solve] step {step}: no open direction (truly stuck). state={state}")
            _http("POST", "action", {"action": "stop"})
            return 21

        # Tie-breaker: prefer the candidate whose visit count is lowest,
        # so a plain right-hand rule that revisits gets nudged toward
        # genuinely new ground. This is still purely local (no global map).
        def candidate_cell(hd):
            dc, dr = _HEADING_DELTAS[hd]
            return (here[0] + dc, here[1] + dr)
        candidates.sort(key=lambda hd: visited.get(candidate_cell(hd), 0))
        next_heading = candidates[0]
        next_cell = candidate_cell(next_heading)

        cells = caps["maze"]["cols"]
        if not (0 <= next_cell[0] < cells and 0 <= next_cell[1] < cells):
            print(f"[solve] step {step}: would step out of grid; aborting")
            _http("POST", "action", {"action": "stop"})
            return 22

        # Same turn + drive_forward pattern as the BFS path. Each primitive
        # comes to a full stop, so there's no momentum carry-over.
        print(f"[solve] step {step:>3}: at {here} heading {_HEADING_NAMES[heading]} "
              f"-> turn {_HEADING_NAMES[next_heading]} -> cell {next_cell}")
        target_yaw_options = [0.0, math.pi / 2, math.pi, -math.pi / 2]
        target_yaw = target_yaw_options[next_heading]
        delta = math.atan2(
            math.sin(target_yaw - state["yaw"]),
            math.cos(target_yaw - state["yaw"]),
        )
        if abs(delta) > 0.05:
            _http("POST", "action", {"action": "turn", "angle": delta, "speed": 0.6})
            ok, state = _wait_until(
                lambda s: s.get("mode") == "idle" or s.get("fault"),
                PER_CELL_TIMEOUT,
            )
            if state.get("fault") or not ok:
                print(f"[solve] step {step}: turn fault/timeout {state.get('fault')}")
                _http("POST", "action", {"action": "stop"})
                return 23
        _http("POST", "action", {"action": "drive_forward", "distance": 2.0, "speed": SPEED})
        ok, state = _wait_until(
            lambda s: s.get("mode") == "idle" or s.get("fault") or s.get("goal_reached"),
            PER_CELL_TIMEOUT,
        )
        if state.get("goal_reached"):
            print(f"[solve] goal_reached after {step} steps")
            _http("POST", "action", {"action": "stop"})
            return 0
        if state.get("fault"):
            print(f"[solve] step {step}: drive fault {state['fault']}")
            _http("POST", "action", {"action": "stop"})
            return 23
        if not ok:
            print(f"[solve] step {step}: drive timeout at {state}")
            _http("POST", "action", {"action": "stop"})
            return 24

        # Snap to the grid so drift doesn't compound. The wall-follower
        # depends on accurate cardinal-headed lidar reads each step, so
        # re-anchoring is even more important here than for BFS.
        _http("POST", "action", {
            "action": "snap_to_cell",
            "col": next_cell[0], "row": next_cell[1], "yaw": target_yaw,
        })
        time.sleep(0.4)
        # Update internal state from telemetry — pose is the truth.
        state = _http("GET", "state")
        here = world_to_cell(state["x"], state["y"], caps["maze"])
        heading = _yaw_to_heading(state["yaw"])
        visited[here] = visited.get(here, 0) + 1

    print(f"[solve] hit max steps ({MAX_WALLFOLLOW_STEPS}); giving up")
    _http("POST", "action", {"action": "stop"})
    return 25


# ---------------------------------------------------------------------------
# Shared cell-by-cell driver (used by Strategy A)
# ---------------------------------------------------------------------------

def _required_yaw(prev, here) -> float:
    dc = here[0] - prev[0]
    dr = here[1] - prev[1]
    if dc == 1:  return 0.0
    if dc == -1: return math.pi
    if dr == 1:  return math.pi / 2
    if dr == -1: return -math.pi / 2
    return 0.0  # same cell — won't happen on a real path step


def _drive_path(path: List[Tuple[int, int]]) -> int:
    """Walk the BFS path one cell at a time. Each step:
    1. Read fresh pose.
    2. Turn to the cardinal heading toward the next cell.
    3. Compute drive distance as the projection of (cell_centre - pose)
       onto the cardinal heading. This *cancels accumulated drift* every
       step — the husky always aims at the absolute world coordinate,
       not "2 m from where I happen to be"."""
    for i, cell in enumerate(path):
        if i == 0:
            continue
        prev = path[i - 1]
        target_yaw = _required_yaw(prev, cell)
        cell_x = -10.0 + 2.0 * cell[0]
        cell_y = -10.0 + 2.0 * cell[1]

        state = _http("GET", "state")
        delta = math.atan2(
            math.sin(target_yaw - state["yaw"]),
            math.cos(target_yaw - state["yaw"]),
        )
        if abs(delta) > 0.05:
            print(f"[solve] step {i:>2}/{len(path)-1}: turn {math.degrees(delta):+5.0f}° "
                  f"to face ({cell[0]},{cell[1]})")
            _http("POST", "action", {"action": "turn", "angle": delta, "speed": 0.6})
            ok, state = _wait_until(
                lambda s: s.get("mode") == "idle" or s.get("fault"),
                PER_CELL_TIMEOUT,
            )
            if state.get("fault") or not ok:
                print(f"[solve] turn fault/timeout: {state.get('fault')}")
                _http("POST", "action", {"action": "stop"})
                return 5
            # Re-anchor after the turn — skid-steer pivots drift the body
            # by ~0.5-1 m which can wedge the husky against walls before the
            # following drive even starts.
            _http("POST", "action", {
                "action": "snap_to_cell",
                "col": prev[0], "row": prev[1], "yaw": target_yaw,
            })
            time.sleep(0.4)
            state = _http("GET", "state")

        # Distance to drive = projection of (cell_centre - current_pose)
        # onto the cardinal heading. Always positive in normal flow; can
        # be tiny / slightly negative if the husky already drifted past.
        hx = math.cos(target_yaw)
        hy = math.sin(target_yaw)
        dist_to_drive = (cell_x - state["x"]) * hx + (cell_y - state["y"]) * hy
        print(f"[solve] step {i:>2}/{len(path)-1}: drive {dist_to_drive:+.2f} m "
              f"to centre of ({cell[0]},{cell[1]}) -> ({cell_x:+.1f},{cell_y:+.1f})")
        if abs(dist_to_drive) < 0.05:
            print("[solve]              already at target; skipping drive")
            continue
        _http("POST", "action", {
            "action": "drive_forward",
            "distance": dist_to_drive,
            "speed": SPEED,
        })
        ok, state = _wait_until(
            lambda s: s.get("mode") == "idle" or s.get("fault") or s.get("goal_reached"),
            PER_CELL_TIMEOUT,
        )
        ex = state["x"] - cell_x
        ey = state["y"] - cell_y
        print(f"[solve]              actual ({state['x']:+.2f},{state['y']:+.2f}) "
              f"yaw={state['yaw']:+.2f}  err dx={ex:+.2f} dy={ey:+.2f}")
        if state.get("goal_reached"):
            print(f"[solve] goal_reached at step {i}")
            _http("POST", "action", {"action": "stop"})
            return 0
        if state.get("fault"):
            print(f"[solve] FAULT during drive: {state['fault']}")
            _http("POST", "action", {"action": "stop"})
            return 5
        if not ok:
            print(f"[solve] drive timeout at step {i}")
            _http("POST", "action", {"action": "stop"})
            return 6

        # Snap to the cell grid so cumulative drift from skid-steer pivots
        # doesn't compound across cells. This is a deliberate demo
        # concession (see bridge's snap_to_cell action) — the value of the
        # demo is in *strategy selection*, not in fighting OmniSim's wheel
        # friction model.
        _http("POST", "action", {
            "action": "snap_to_cell",
            "col": cell[0], "row": cell[1], "yaw": target_yaw,
        })
        time.sleep(0.4)  # let the supervisor settle
    print(f"[solve] path exhausted without goal_reached. final state: {state}")
    _http("POST", "action", {"action": "stop"})
    return 7


# ---------------------------------------------------------------------------
# Top-level
# ---------------------------------------------------------------------------

def main() -> int:
    print(f"[solve] bridge: {BRIDGE_URL}")
    try:
        caps = _http("GET", "capabilities")
    except RuntimeError as e:
        print(f"[solve] {e}")
        print("[solve] launch a husky_maze world first.")
        return 4

    print(f"[solve] world_title = {caps.get('world_title')!r}  "
          f"map_available = {caps.get('map_available')}")
    print(f"[solve] max_linear = {caps['max_linear_m_s']:.2f} m/s, "
          f"max_angular = {caps['max_angular_r_s']:.2f} rad/s, "
          f"lidar = {caps['lidar']['num_rays']} rays @ {caps['lidar']['max_range_m']} m")

    if caps.get("map_available"):
        return solve_known_map(caps)
    return solve_unknown_map(caps)


if __name__ == "__main__":
    sys.exit(main())
