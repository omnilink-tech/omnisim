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

"""Autonomous, sensor-only Husky controller for the Agent Build maze film.

The process has an explicit trust boundary:

* :class:`SimulatedLidar` is the only component allowed to retain world wall
  geometry.  It emits 32 ray ranges, matching a planar 360-degree scanner.
* :class:`MazePlanner` receives ranges, pose-derived cell coordinates, maze
  dimensions, and the goal coordinate.  It never receives wall nodes, the
  authored seed, an adjacency graph, or an optimal path.

``frontier`` mode keeps a discovered topological graph and replans through it.
``memoryless`` is the single-variable negative control: the same sensor and
wheel controller remain active, but the discovered graph is disabled.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import tempfile
import time
from collections import deque
from pathlib import Path

from omnisim import Supervisor


WHEEL_RADIUS_M = 0.1651
HALF_TRACK_M = 0.2854
MAX_WHEEL_SPEED = 6.0
WHEEL_MOTORS = (
    "front_left_wheel_motor",
    "rear_left_wheel_motor",
    "front_right_wheel_motor",
    "rear_right_wheel_motor",
)
HEADINGS = ((1, 0), (0, 1), (-1, 0), (0, -1))  # E, N, W, S
HEADING_NAMES = ("E", "N", "W", "S")
LIDAR_RAYS = 32
LIDAR_MAX_RANGE_M = 12.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_pi(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--size", type=int, default=17)
    parser.add_argument("--cell-size", type=float, default=2.4)
    parser.add_argument("--origin", type=float, default=-19.2)
    parser.add_argument("--goal", default="16,0")
    args, _ = parser.parse_known_args()
    col, row = (int(v) for v in args.goal.split(","))
    args.goal = (col, row)
    return args


def find_link_by_name(node, wanted: str):
    if node is None:
        return None
    try:
        name_field = node.getField("name")
        if name_field is not None and name_field.getSFString() == wanted:
            return node
    except Exception:
        pass
    try:
        children = node.getField("children")
        count = children.getCount() if children is not None else 0
    except Exception:
        count = 0
    for index in range(count):
        child = children.getMFNode(index)
        if child is None:
            continue
        match = find_link_by_name(child, wanted)
        if match is not None:
            return match
    try:
        endpoint = node.getField("endPoint")
        endpoint_node = endpoint.getSFNode() if endpoint is not None else None
    except Exception:
        endpoint_node = None
    if endpoint_node is not None:
        match = find_link_by_name(endpoint_node, wanted)
        if match is not None:
            return match
    return None


def find_first_physical_child(node, depth: int = 0):
    if node is None or depth > 8:
        return None
    try:
        position = node.getPosition()
        if depth > 0 and position and all(math.isfinite(float(v)) for v in position):
            return node
    except Exception:
        pass
    try:
        children = node.getField("children")
        count = children.getCount() if children is not None else 0
    except Exception:
        count = 0
    for index in range(count):
        child = children.getMFNode(index)
        if child is None:
            continue
        match = find_first_physical_child(child, depth + 1)
        if match is not None:
            return match
    try:
        endpoint = node.getField("endPoint")
        endpoint_node = endpoint.getSFNode() if endpoint is not None else None
    except Exception:
        endpoint_node = None
    if endpoint_node is not None:
        match = find_first_physical_child(endpoint_node, depth + 1)
        if match is not None:
            return match
    return None


def yaw_from_orientation(matrix) -> float:
    # Local +X is the vehicle's forward direction.
    return math.atan2(float(matrix[3]), float(matrix[0]))


def collect_walls(supervisor: Supervisor) -> list[dict]:
    walls: list[dict] = []
    root = supervisor.getRoot()
    children = root.getField("children") if root is not None else None
    if children is None:
        return walls
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if node is None:
            continue
        try:
            if node.getTypeName() != "Wall":
                continue
            translation = node.getField("translation").getSFVec3f()
            size = node.getField("size").getSFVec3f()
            name = node.getField("name").getSFString()
        except Exception:
            continue
        walls.append({
            "name": name,
            "cx": float(translation[0]),
            "cy": float(translation[1]),
            "sx": float(size[0]),
            "sy": float(size[1]),
        })
    return walls


def ray_aabb_distance(ox: float, oy: float, dx: float, dy: float,
                      wall: dict, max_range: float) -> float | None:
    min_x = wall["cx"] - wall["sx"] * 0.5
    max_x = wall["cx"] + wall["sx"] * 0.5
    min_y = wall["cy"] - wall["sy"] * 0.5
    max_y = wall["cy"] + wall["sy"] * 0.5
    if abs(dx) <= 1e-12:
        if ox < min_x or ox > max_x:
            return None
        tx1, tx2 = -math.inf, math.inf
    else:
        tx1, tx2 = (min_x - ox) / dx, (max_x - ox) / dx
    if abs(dy) <= 1e-12:
        if oy < min_y or oy > max_y:
            return None
        ty1, ty2 = -math.inf, math.inf
    else:
        ty1, ty2 = (min_y - oy) / dy, (max_y - oy) / dy
    if tx1 > tx2:
        tx1, tx2 = tx2, tx1
    if ty1 > ty2:
        ty1, ty2 = ty2, ty1
    entry = max(tx1, ty1)
    exit_distance = min(tx2, ty2)
    if exit_distance < 0 or entry > exit_distance:
        return None
    distance = exit_distance if entry < 1e-6 else entry
    return distance if 0 < distance <= max_range else None


class SimulatedLidar:
    """World-facing sensor adapter.  No planner method can access ``walls``."""

    def __init__(self, walls: list[dict]):
        self._walls = tuple(walls)

    def scan(self, x: float, y: float, yaw: float) -> dict:
        angles = [2 * math.pi * index / LIDAR_RAYS - math.pi for index in range(LIDAR_RAYS)]
        ranges: list[float] = []
        hits: list[str | None] = []
        for body_angle in angles:
            world_angle = yaw + body_angle
            dx, dy = math.cos(world_angle), math.sin(world_angle)
            nearest = LIDAR_MAX_RANGE_M
            hit = None
            for wall in self._walls:
                distance = ray_aabb_distance(x, y, dx, dy, wall, LIDAR_MAX_RANGE_M)
                if distance is not None and distance < nearest:
                    nearest = distance
                    hit = wall["name"]
            ranges.append(nearest)
            hits.append(hit)
        return {"angles_rad": angles, "ranges_m": ranges, "hits": hits, "yaw": yaw}

    @staticmethod
    def absolute_cardinal_ranges(scan: dict) -> dict[int, float]:
        result: dict[int, float] = {}
        targets = (0.0, math.pi / 2, math.pi, -math.pi / 2)
        for heading, target_world in enumerate(targets):
            best_index = min(
                range(len(scan["angles_rad"])),
                key=lambda index: abs(wrap_pi(scan["yaw"] + scan["angles_rad"][index] - target_world)),
            )
            result[heading] = float(scan["ranges_m"][best_index])
        return result


class MazePlanner:
    """Planner-visible state: cells, goal, and local range-derived openings."""

    def __init__(self, size: int, goal: tuple[int, int], mode: str):
        self.size = size
        self.goal = goal
        self.mode = mode
        self.visited: set[tuple[int, int]] = set()
        self.graph: dict[tuple[int, int], set[tuple[int, int]]] = {}
        self.previous: tuple[int, int] | None = None
        self.memoryless_states: dict[tuple[tuple[int, int], tuple[int, int] | None], int] = {}
        self.decisions: list[dict] = []

    def _valid(self, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def _manhattan(self, cell: tuple[int, int]) -> int:
        return abs(cell[0] - self.goal[0]) + abs(cell[1] - self.goal[1])

    def observe(self, cell: tuple[int, int], cardinal_ranges: dict[int, float],
                open_threshold_m: float) -> list[tuple[int, int]]:
        opened: list[tuple[int, int]] = []
        self.graph.setdefault(cell, set())
        for heading, (dc, dr) in enumerate(HEADINGS):
            nxt = (cell[0] + dc, cell[1] + dr)
            if self._valid(nxt) and cardinal_ranges[heading] > open_threshold_m:
                self.graph[cell].add(nxt)
                self.graph.setdefault(nxt, set()).add(cell)
                opened.append(nxt)
        self.visited.add(cell)
        return opened

    def _path_to_best_frontier(self, start: tuple[int, int]) -> list[tuple[int, int]]:
        queue = deque([start])
        parent: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        candidates: list[tuple[float, int, tuple[int, int]]] = []
        while queue:
            here = queue.popleft()
            for nxt in sorted(self.graph.get(here, ())):
                if nxt not in parent:
                    parent[nxt] = here
                    queue.append(nxt)
        for cell in parent:
            if cell not in self.visited:
                distance = 0
                cursor = cell
                while parent[cursor] is not None:
                    distance += 1
                    cursor = parent[cursor]  # type: ignore[assignment]
                # Distance dominates; Manhattan breaks ties toward the goal.
                candidates.append((distance + 0.18 * self._manhattan(cell), distance, cell))
        if not candidates:
            return []
        _, _, target = min(candidates)
        path = [target]
        while path[-1] != start:
            predecessor = parent[path[-1]]
            if predecessor is None:
                break
            path.append(predecessor)
        return list(reversed(path))

    def choose(self, cell: tuple[int, int], opened: list[tuple[int, int]]) -> tuple[str, tuple[int, int] | None]:
        if cell == self.goal:
            return "goal", None
        if self.mode == "memoryless":
            state = (cell, self.previous)
            if state in self.memoryless_states:
                return "cycle", None
            self.memoryless_states[state] = len(self.decisions)
            candidates = [nxt for nxt in opened if nxt != self.previous] or list(opened)
            if not candidates:
                return "no_open_cell", None
            candidates.sort(key=lambda nxt: (self._manhattan(nxt), -nxt[0], nxt[1]))
            nxt = candidates[0]
            strategy = "local_goal_greedy"
        else:
            path = self._path_to_best_frontier(cell)
            if len(path) < 2:
                return "frontier_exhausted", None
            nxt = path[1]
            strategy = "frontier_replan"
        self.decisions.append({
            "step": len(self.decisions) + 1,
            "cell": list(cell),
            "open_neighbours": [list(value) for value in sorted(opened)],
            "next_cell": list(nxt),
            "strategy": strategy,
            "visited_cells": len(self.visited),
            "discovered_edges": sum(len(v) for v in self.graph.values()) // 2,
        })
        self.previous = cell
        return "move", nxt


def axis_angle_to_target(position: tuple[float, float, float],
                         target: tuple[float, float, float]) -> list[float]:
    px, py, pz = position
    tx, ty, tz = target
    fx, fy, fz = tx - px, ty - py, tz - pz
    norm = math.sqrt(fx * fx + fy * fy + fz * fz)
    if norm < 1e-9:
        return [0, 0, 1, 0]
    fx, fy, fz = fx / norm, fy / norm, fz / norm
    ux, uy, uz = 0.0, 0.0, 1.0
    dot = fx * ux + fy * uy + fz * uz
    ux, uy, uz = ux - dot * fx, uy - dot * fy, uz - dot * fz
    unorm = math.sqrt(ux * ux + uy * uy + uz * uz)
    if unorm < 1e-9:
        ux, uy, uz, unorm = 0.0, 1.0, 0.0, 1.0
    ux, uy, uz = ux / unorm, uy / unorm, uz / unorm
    yx, yy, yz = uy * fz - uz * fy, uz * fx - ux * fz, ux * fy - uy * fx
    matrix = ((fx, yx, ux), (fy, yy, uy), (fz, yz, uz))
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    angle = math.acos(clamp((trace - 1) / 2, -1, 1))
    if angle < 1e-9:
        return [0, 0, 1, 0]
    denom = 2 * math.sin(angle)
    return [
        (matrix[2][1] - matrix[1][2]) / denom,
        (matrix[0][2] - matrix[2][0]) / denom,
        (matrix[1][0] - matrix[0][1]) / denom,
        angle,
    ]


class CameraDirector:
    """Purposeful, horizon-locked coverage for evidence and editorial capture.

    ``zone`` is the simulator-first story profile: the camera stays completely
    locked while the robot traverses a small region, then makes an intentional
    direct cut when it enters the next region.  That keeps a time-compressed
    long run readable without the disorienting high-speed chase-camera motion.
    """

    MODES = {"track", "zone", "wide", "close_start", "story"}

    def __init__(self, supervisor: Supervisor, enabled: bool, mode: str,
                 origin: float, cell_size: float, size: int,
                 story_wide_end_s: float, story_motion_start_s: float):
        self.enabled = enabled
        self.mode = mode
        self.origin = origin
        self.cell_size = cell_size
        self.size = size
        self.story_wide_end_s = story_wide_end_s
        self.story_motion_start_s = story_motion_start_s
        self.position: list[float] | None = None
        self.target: list[float] | None = None
        self.zone: tuple[int, int] | None = None
        self.start: tuple[float, float] | None = None
        self.viewpoint = None
        if not enabled:
            return
        root = supervisor.getRoot()
        children = root.getField("children") if root is not None else None
        for index in range(children.getCount() if children is not None else 0):
            node = children.getMFNode(index)
            if node is not None and node.getTypeName() == "Viewpoint":
                self.viewpoint = node
                break

    def _apply(self, position: list[float], target: list[float]) -> None:
        self.position = position
        self.target = target
        assert self.viewpoint is not None
        self.viewpoint.getField("position").setSFVec3f(position)
        self.viewpoint.getField("orientation").setSFRotation(
            axis_angle_to_target(tuple(position), tuple(target))
        )

    def update(self, sim_time: float, x: float, y: float, phase: str, dt: float) -> None:
        if not self.enabled or self.viewpoint is None or sim_time < 4.0:
            return
        if self.start is None:
            self.start = (x, y)

        maze_center = self.origin + (self.size - 1) * self.cell_size * 0.5
        coverage_mode = self.mode
        if self.mode == "story":
            if sim_time < self.story_wide_end_s:
                coverage_mode = "wide"
            elif sim_time < self.story_motion_start_s:
                coverage_mode = "close_start"
            else:
                coverage_mode = "zone"

        if coverage_mode == "wide":
            # A single locked establishing frame.  The whole authored maze is
            # visible and the camera never performs a decorative orbit.
            self._apply(
                [maze_center + 27.0, maze_center - 31.0, 38.0],
                [maze_center, maze_center, 0.15],
            )
            return

        if coverage_mode == "close_start":
            # A fixed high three-quarter shot inside the start cell.  Keeping
            # the lateral offset below half a cell prevents a wall from
            # crossing between the camera and the robot.
            sx, sy = self.start
            inward_x, inward_y = maze_center - sx, maze_center - sy
            norm = max(1e-9, math.hypot(inward_x, inward_y))
            inward_x, inward_y = inward_x / norm, inward_y / norm
            tangent_x, tangent_y = -inward_y, inward_x
            self._apply(
                [
                    sx + inward_x * 0.72 + tangent_x * 0.52,
                    sy + inward_y * 0.72 + tangent_y * 0.52,
                    3.7,
                ],
                [sx, sy, 0.38],
            )
            return

        if coverage_mode == "zone" and not phase.startswith("finished"):
            # Four-cell coverage zones are large enough for motion to develop
            # within a shot and small enough to keep the robot judgeable.  The
            # shot is static inside a zone; a zone change is a direct cut.
            col = int(clamp(round((x - self.origin) / self.cell_size), 0, self.size - 1))
            row = int(clamp(round((y - self.origin) / self.cell_size), 0, self.size - 1))
            zone = (col // 4, row // 4)
            if zone != self.zone:
                self.zone = zone
                max_cell = self.size - 1
                center_col = min(max_cell, zone[0] * 4 + 1.5)
                center_row = min(max_cell, zone[1] * 4 + 1.5)
                center_x = self.origin + center_col * self.cell_size
                center_y = self.origin + center_row * self.cell_size
                self._apply(
                    [center_x + 3.5, center_y - 4.0, 18.0],
                    [center_x, center_y, 0.28],
                )
                print(f"[husky_unseen_maze] CAMERA_CUT zone={zone} cell=({col},{row})")
            return

        # Keep the camera on the maze-interior side of the robot so perimeter
        # walls cannot hide it.  The radial direction changes only as the robot
        # changes its location in the build; the horizon remains level.
        inward_x, inward_y = -x, -y
        inward_norm = math.hypot(inward_x, inward_y)
        if inward_norm < 1.0:
            inward_x, inward_y, inward_norm = 0.7, -0.7, 1.0
        inward_x, inward_y = inward_x / inward_norm, inward_y / inward_norm
        desired_position = [x + inward_x * 6.2, y + inward_y * 6.2, 8.1]
        desired_target = [x, y, 0.38]
        if phase.startswith("finished"):
            # Move laterally off the robot-to-beacon axis.  The success marker
            # and robot remain side-by-side instead of occluding one another.
            tangent_x, tangent_y = -inward_y, inward_x
            desired_position = [
                x + inward_x * 5.0 + tangent_x * 3.2,
                y + inward_y * 5.0 + tangent_y * 3.2,
                5.2,
            ]
            desired_target = [x, y, 0.45]
        if coverage_mode == "zone":
            self._apply(desired_position, desired_target)
            return
        alpha = 1.0 - math.exp(-dt * 2.2)
        if self.position is None:
            self.position = desired_position
            self.target = desired_target
        else:
            self.position = [a + (b - a) * alpha for a, b in zip(self.position, desired_position)]
            assert self.target is not None
            self.target = [a + (b - a) * alpha for a, b in zip(self.target, desired_target)]
        self.viewpoint.getField("position").setSFVec3f(self.position)
        orientation = axis_angle_to_target(tuple(self.position), tuple(self.target or desired_target))
        self.viewpoint.getField("orientation").setSFRotation(orientation)


def write_result(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[husky_unseen_maze] RESULT {json.dumps(payload, separators=(',', ':'))}")
    print(f"[husky_unseen_maze] evidence={path}")


def main() -> None:
    args = parse_args()
    mode = os.environ.get("OMNISIM_MAZE_MODE", "frontier").strip().lower()
    if mode not in {"frontier", "memoryless"}:
        print(f"[husky_unseen_maze] invalid mode {mode!r}")
        return
    result_default = Path(tempfile.gettempdir()) / f"husky_unseen_maze_{mode}_{os.getpid()}.json"
    result_path = Path(os.environ.get("OMNISIM_MAZE_RESULT", str(result_default))).resolve()
    max_hops = int(os.environ.get("OMNISIM_MAZE_MAX_HOPS", "700"))
    auto_quit = os.environ.get("OMNISIM_MAZE_AUTO_QUIT", "1") != "0"
    direct_camera = os.environ.get("OMNISIM_MAZE_DIRECT_CAMERA", "0") == "1"
    camera_mode = os.environ.get("OMNISIM_MAZE_CAMERA_MODE", "track").strip().lower()
    if camera_mode not in CameraDirector.MODES:
        print(f"[husky_unseen_maze] invalid camera mode {camera_mode!r}")
        return
    movie_value = os.environ.get("OMNISIM_MAZE_MOVIE", "").strip()
    movie_path = Path(movie_value).resolve() if movie_value else None
    movie_acceleration = max(1, int(os.environ.get("OMNISIM_MAZE_MOVIE_ACCELERATION", "8")))
    frame_dir_value = os.environ.get("OMNISIM_MAZE_FRAME_DIR", "").strip()
    frame_dir = Path(frame_dir_value).resolve() if frame_dir_value else None
    frame_acceleration = max(1, int(os.environ.get("OMNISIM_MAZE_FRAME_ACCELERATION", "8")))
    frame_idle_acceleration = max(
        1, int(os.environ.get("OMNISIM_MAZE_FRAME_IDLE_ACCELERATION", str(frame_acceleration)))
    )
    frame_start_s = max(0.0, float(os.environ.get("OMNISIM_MAZE_FRAME_START_S", "15")))
    start_delay_s = max(0.8, float(os.environ.get("OMNISIM_MAZE_START_DELAY_S", "0.8")))
    story_wide_end_s = max(
        frame_start_s,
        float(os.environ.get("OMNISIM_MAZE_STORY_WIDE_END_S", str(frame_start_s + 10.0))),
    )
    finish_hold_s = max(2.5, float(os.environ.get("OMNISIM_MAZE_FINISH_HOLD_S", "2.5")))
    frame_fps = 30

    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0
    self_node = robot.getSelf()
    if self_node is None:
        print("[husky_unseen_maze] getSelf() failed")
        return

    motors = []
    for name in WHEEL_MOTORS:
        motor = robot.getDevice(name)
        if motor is None:
            print(f"[husky_unseen_maze] missing motor {name}")
            return
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        motors.append(motor)
    left_motors, right_motors = motors[:2], motors[2:]

    robot.step(dt_ms)
    robot.step(dt_ms)
    pose_node = (
        find_link_by_name(self_node, "base_link")
        or find_link_by_name(self_node, "base_footprint")
        or find_first_physical_child(self_node)
    )
    if pose_node is None:
        print("[husky_unseen_maze] no physical Husky link")
        return

    walls = collect_walls(robot)
    lidar = SimulatedLidar(walls)
    # The world-facing wall collection is now owned only by the sensor adapter.
    wall_geometry_sha = hashlib.sha256(
        json.dumps(walls, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    del walls

    planner = MazePlanner(args.size, args.goal, mode)
    camera = CameraDirector(
        robot, direct_camera, camera_mode, args.origin, args.cell_size, args.size,
        story_wide_end_s, start_delay_s,
    )
    open_threshold = args.cell_size * 0.72
    goal_x = args.origin + args.goal[0] * args.cell_size
    goal_y = args.origin + args.goal[1] * args.cell_size
    goal_beacon = robot.getFromDef("GOAL_BEACON")

    phase = "settle"
    phase_started = 0.0
    target_cell: tuple[int, int] | None = None
    current_cell: tuple[int, int] | None = None
    trail: list[list[int]] = []
    distance_m = 0.0
    hops = 0
    replans = 0
    stuck_ticks = 0
    current_left = current_right = 0.0
    start_wall_clock = time.time()

    initial_position = pose_node.getPosition()
    previous_x, previous_y = float(initial_position[0]), float(initial_position[1])
    previous_yaw = yaw_from_orientation(pose_node.getOrientation())
    previous_v = previous_w = 0.0
    finish_payload: dict | None = None
    finish_sim_time = 0.0
    movie_started = False
    movie_stopped = False
    frame_index = 0
    next_frame_sim_time = frame_start_s
    if frame_dir is not None:
        frame_dir.mkdir(parents=True, exist_ok=True)

    print(
        f"[husky_unseen_maze] ready mode={mode} grid={args.size}x{args.size} "
        f"goal={args.goal} rays={LIDAR_RAYS} walls_behind_sensor={len(lidar._walls)} "
        f"camera={camera_mode}"
    )

    while robot.step(dt_ms) != -1:
        sim_time = float(robot.getTime())
        pos = pose_node.getPosition()
        x, y = float(pos[0]), float(pos[1])
        yaw = yaw_from_orientation(pose_node.getOrientation())
        distance_m += math.hypot(x - previous_x, y - previous_y)
        v_linear = math.hypot(x - previous_x, y - previous_y) / max(dt, 1e-6)
        v_angular = wrap_pi(yaw - previous_yaw) / max(dt, 1e-6)
        previous_x, previous_y, previous_yaw = x, y, yaw
        previous_v, previous_w = v_linear, v_angular

        # Let wgpu finish its first-frame resource setup and OmniLight bake
        # before recording. Starting movie readback during that initialization
        # window can race bind-group creation on wgpu-native.
        if movie_path is not None and not movie_started and sim_time >= 5.0:
            movie_path.parent.mkdir(parents=True, exist_ok=True)
            robot.movieStartRecording(
                str(movie_path), 1920, 1080, 0, 96,
                movie_acceleration, False,
            )
            movie_started = True
            print(
                f"[husky_unseen_maze] wgpu main-view movie={movie_path} "
                f"acceleration={movie_acceleration}x"
            )

        camera.update(sim_time, x, y, phase, dt)
        if frame_dir is not None and sim_time + 1e-9 >= next_frame_sim_time:
            frame_path = frame_dir / f"frame_{frame_index:06d}.png"
            robot.exportImage(str(frame_path), 100)
            frame_index += 1
            capture_acceleration = (
                frame_idle_acceleration if sim_time < start_delay_s else frame_acceleration
            )
            next_frame_sim_time += capture_acceleration / frame_fps
        linear = angular = 0.0

        if finish_payload is not None:
            phase = "finished_" + finish_payload["outcome"]
            if sim_time - finish_sim_time >= finish_hold_s:
                for motor in motors:
                    motor.setVelocity(0.0)
                if movie_path is not None and movie_started and not movie_stopped:
                    robot.movieStopRecording()
                    movie_stopped = True
                movie_done = (
                    movie_path is None
                    or not movie_started
                    or robot.movieIsReady()
                    or robot.movieFailed()
                    or sim_time - finish_sim_time >= max(32.0, finish_hold_s)
                )
                if auto_quit and movie_done:
                    robot.simulationQuit(0)
                    return
            continue

        if phase == "settle":
            if sim_time - phase_started >= start_delay_s:
                phase = "scan"

        if phase == "scan":
            col = int(round((x - args.origin) / args.cell_size))
            row = int(round((y - args.origin) / args.cell_size))
            current_cell = (col, row)
            if not (0 <= col < args.size and 0 <= row < args.size):
                outcome, reason = "failed", "pose_left_grid"
            else:
                scan = lidar.scan(x, y, yaw)
                cardinals = lidar.absolute_cardinal_ranges(scan)
                opened = planner.observe(current_cell, cardinals, open_threshold)
                if not trail or trail[-1] != [col, row]:
                    trail.append([col, row])
                action, target_cell = planner.choose(current_cell, opened)
                outcome = "running"
                reason = action
                if action == "goal":
                    outcome, reason = "success", "goal_reached"
                elif action != "move":
                    outcome, reason = "failed", action
                elif target_cell is not None:
                    hops += 1
                    replans += 1
                    phase = "move"
                    phase_started = sim_time
                    print(
                        f"[husky_unseen_maze] hop={hops:03d} cell={current_cell} "
                        f"next={target_cell} visited={len(planner.visited)} "
                        f"edges={sum(len(v) for v in planner.graph.values()) // 2}"
                    )
            if outcome != "running":
                final_error = math.hypot(x - goal_x, y - goal_y)
                if outcome == "success" and goal_beacon is not None:
                    # The beacon and robot share the mathematical goal center.
                    # Once the measured run is complete, move only the visual
                    # beacon toward the maze interior so the success hold can
                    # show robot, pad, and marker without occlusion.  Physics,
                    # planning, and the recorded goal error are already final.
                    maze_center = args.origin + (args.size - 1) * args.cell_size * 0.5
                    inward_x, inward_y = maze_center - goal_x, maze_center - goal_y
                    inward_norm = max(1e-9, math.hypot(inward_x, inward_y))
                    goal_beacon.getField("translation").setSFVec3f([
                        goal_x + 0.82 * inward_x / inward_norm,
                        goal_y + 0.82 * inward_y / inward_norm,
                        0.0,
                    ])
                finish_payload = {
                    "version": 1,
                    "experiment": "husky_unseen_maze",
                    "mode": mode,
                    "outcome": outcome,
                    "reason": reason,
                    "goal_reached": outcome == "success",
                    "grid": [args.size, args.size],
                    "goal": list(args.goal),
                    "final_cell": list(current_cell) if current_cell is not None else None,
                    "goal_error_m": round(final_error, 4),
                    "hops": hops,
                    "visited_cells": len(planner.visited),
                    "discovered_edges": sum(len(v) for v in planner.graph.values()) // 2,
                    "replans": replans,
                    "distance_m": round(distance_m, 3),
                    "final_pose": {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 5)},
                    "wheel_command_rad_s": {"left": round(current_left, 4), "right": round(current_right, 4)},
                    "sim_time_s": round(sim_time, 3),
                    "wall_time_s": round(time.time() - start_wall_clock, 3),
                    "sensor": {
                        "type": "simulated_360_planar_raycast",
                        "rays": LIDAR_RAYS,
                        "max_range_m": LIDAR_MAX_RANGE_M,
                        "open_threshold_m": open_threshold,
                        "wall_geometry_sha256": wall_geometry_sha,
                    },
                    "planner_inputs": ["pose", "grid_dimensions", "goal_coordinate", "ray_ranges"],
                    "planner_denied": ["world_seed", "wall_nodes", "authored_adjacency", "optimal_path"],
                    "capture": {
                        "camera_mode": camera_mode,
                        "frame_acceleration": frame_acceleration,
                        "frame_idle_acceleration": frame_idle_acceleration,
                        "frame_start_s": frame_start_s,
                        "success_beacon_visual_offset_m": 0.82,
                    },
                    "trail": trail,
                    "decisions": planner.decisions,
                }
                write_result(result_path, finish_payload)
                finish_sim_time = sim_time
                for motor in motors:
                    motor.setVelocity(0.0)

        if phase == "move" and target_cell is not None:
            tx = args.origin + target_cell[0] * args.cell_size
            ty = args.origin + target_cell[1] * args.cell_size
            dx, dy = tx - x, ty - y
            distance = math.hypot(dx, dy)
            desired_yaw = math.atan2(dy, dx)
            heading_error = wrap_pi(desired_yaw - yaw)
            if distance < 0.20:
                linear = angular = 0.0
                if abs(previous_v) < 0.06 and abs(previous_w) < 0.12:
                    phase = "scan"
                    stuck_ticks = 0
            elif sim_time - phase_started > 90.0:
                finish_payload = {
                    "version": 1,
                    "experiment": "husky_unseen_maze",
                    "mode": mode,
                    "outcome": "failed",
                    "reason": "wheel_controller_timeout",
                    "goal_reached": False,
                    "final_cell": list(current_cell) if current_cell else None,
                    "target_cell": list(target_cell),
                    "hops": hops,
                    "visited_cells": len(planner.visited),
                    "distance_m": round(distance_m, 3),
                    "final_pose": {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 5)},
                    "wheel_command_rad_s": {"left": round(current_left, 4), "right": round(current_right, 4)},
                    "sim_time_s": round(sim_time, 3),
                    "trail": trail,
                    "decisions": planner.decisions,
                }
                write_result(result_path, finish_payload)
                finish_sim_time = sim_time
            elif abs(heading_error) > 0.32:
                # Newton's contact-rich Husky rig can bind under a perfectly
                # opposed zero-radius skid turn.  A 0.20 m/s crawl produces a
                # tight, fully wheel-driven arc (radius ~= 0.18 m at the cap)
                # that stays comfortably inside a 2.4 m cell and keeps tire
                # contact moving.  This is motion-control tuning, not a pose
                # correction or grid snap.
                linear = 0.025
                angular = clamp(heading_error * 3.2, -2.0, 2.0)
            else:
                linear = min(0.90, max(0.15, distance * 1.05)) * max(0.35, math.cos(heading_error))
                angular = clamp(heading_error * 2.4, -0.75, 0.75)
            commanded = abs(linear) + abs(angular) > 0.08
            moving = abs(previous_v) + abs(previous_w) > 0.045
            stuck_ticks = stuck_ticks + 1 if commanded and not moving else 0
            if stuck_ticks > 170:
                # Fail closed.  A stuck robot is evidence against the build;
                # it is never teleported or snapped back to a grid centre.
                finish_payload = {
                    "version": 1,
                    "experiment": "husky_unseen_maze",
                    "mode": mode,
                    "outcome": "failed",
                    "reason": "physically_stuck",
                    "goal_reached": False,
                    "final_cell": list(current_cell) if current_cell else None,
                    "target_cell": list(target_cell),
                    "hops": hops,
                    "visited_cells": len(planner.visited),
                    "distance_m": round(distance_m, 3),
                    "final_pose": {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 5)},
                    "wheel_command_rad_s": {"left": round(current_left, 4), "right": round(current_right, 4)},
                    "sim_time_s": round(sim_time, 3),
                    "trail": trail,
                    "decisions": planner.decisions,
                }
                write_result(result_path, finish_payload)
                finish_sim_time = sim_time

        if hops > max_hops and finish_payload is None:
            finish_payload = {
                "version": 1,
                "experiment": "husky_unseen_maze",
                "mode": mode,
                "outcome": "failed",
                "reason": "hop_budget",
                "goal_reached": False,
                "hops": hops,
                "visited_cells": len(planner.visited),
                "distance_m": round(distance_m, 3),
                "sim_time_s": round(sim_time, 3),
                "trail": trail,
                "decisions": planner.decisions,
            }
            write_result(result_path, finish_payload)
            finish_sim_time = sim_time

        left_target = clamp((linear - angular * HALF_TRACK_M) / WHEEL_RADIUS_M,
                            -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        right_target = clamp((linear + angular * HALF_TRACK_M) / WHEEL_RADIUS_M,
                             -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        current_left += clamp(left_target - current_left, -0.32, 0.32)
        current_right += clamp(right_target - current_right, -0.32, 0.32)
        for motor in left_motors:
            motor.setVelocity(current_left)
        for motor in right_motors:
            motor.setVelocity(current_right)


if __name__ == "__main__":
    main()
