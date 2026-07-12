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

"""Husky-control tools - thin HTTP proxies to the husky_omnilink_bridge.

Every tool here forwards to the bridge over HTTP and returns the bridge
response unchanged (with a defensive wrapper for transport failures).
The agent never imports the OmniSim `controller` module.

The map graph is owned by the bridge, not the agent. The bridge decides
whether to expose it (based on the world's title — "Unknown" worlds get
{available: false}). This is what keeps the "the agent doesn't know the
map" property honest in the husky_maze_unknown.wbt demo: the agent only
ever sees lidar, never the wall list.
"""

from __future__ import annotations

import json as _json
import re as _re
import sqlite3 as _sqlite3
from pathlib import Path as _Path
from typing import Any, Dict, List

from ._base import (
    ALWAYS, GUARDED, ON_DEMAND, SAFE, ToolSpec, bridge_get, bridge_post,
)


# ----- Memory lookup (used by replay_recalled_path) ---------------------

_MEMORY_ROOT = _Path(__file__).resolve().parent.parent / "long_term_memory"
_INDEX_PATH = _MEMORY_ROOT / "_index.sqlite"
_PATH_LINE_RE = _re.compile(r"Full ordered path:\s*(\[\[.*?\]\])", _re.DOTALL)


def _load_memory_by_id(memory_id: str) -> Dict[str, Any]:
    """Look up a saved memory's title + body + path by id, without going
    through the LLM. Returns {"error": "..."} on failure or
    {"id", "title", "body", "extracted_path"} on success. extracted_path
    is the parsed cell list when the body's `Full ordered path:` line
    contains a JSON-array literal; None otherwise."""
    if not _INDEX_PATH.exists():
        return {"error": "no memory index — save_local_memory has not been called yet"}
    try:
        with _sqlite3.connect(str(_INDEX_PATH)) as c:
            row = c.execute(
                "SELECT id, title, body, tags FROM memories WHERE id = ?",
                (memory_id,),
            ).fetchone()
    except Exception as e:
        return {"error": f"sqlite read failed: {e.__class__.__name__}: {e}"}
    if not row:
        return {"error": f"memory id not found: {memory_id!r}"}
    mid, title, body, tags_json = row
    extracted_path = None
    m = _PATH_LINE_RE.search(body or "")
    if m:
        try:
            cells = _json.loads(m.group(1))
            if isinstance(cells, list) and cells and all(
                isinstance(c, list) and len(c) == 2 for c in cells
            ):
                extracted_path = [[int(c[0]), int(c[1])] for c in cells]
        except Exception:
            extracted_path = None
    try:
        tags = _json.loads(tags_json) if tags_json else []
    except Exception:
        tags = []
    return {
        "id": mid,
        "title": title,
        "tags": tags,
        "body": body,
        "extracted_path": extracted_path,
    }


# ----- Tool implementations ----------------------------------------------

def _impl_get_capabilities(**_: Any) -> Dict[str, Any]:
    return bridge_get("capabilities")


def _impl_get_state(**_: Any) -> Dict[str, Any]:
    return bridge_get("state")


def _impl_read_mission_brief(**_: Any) -> Dict[str, Any]:
    """Operator's free-form mission brief, read from WorldInfo.info."""
    return bridge_get("mission")


def _impl_complete_mission(rationale: str = "", claimed_cells: Any = None, **_: Any) -> Dict[str, Any]:
    """Mark the mission as complete. Bridge logs the claim — operator audits.

    Tool-level guard: refuse the claim when the husky is mid-fault. The
    over-claim failure mode we keep seeing is the agent calling this from a
    `goto_cell_timeout` state — i.e. claiming success because it's stuck,
    not because it's done. If goal_reached is true, the legacy check
    already verified arrival and we allow the claim regardless of fault."""
    if not rationale or not rationale.strip():
        return {"error": "rationale is required: a one-sentence justification"}
    try:
        state = bridge_get("state") or {}
    except Exception:
        state = {}
    fault = state.get("fault")
    goal_reached = bool(state.get("goal_reached"))
    if fault and not goal_reached:
        cur = state.get("current_cell") or {}
        return {
            "error": "complete_mission refused: bridge has an active fault",
            "fault": fault,
            "current_cell": [cur.get("col"), cur.get("row")],
            "pose": {"x": state.get("x"), "y": state.get("y")},
            "hint": (
                "You called complete_mission while state.fault is set "
                f"({fault!r}) and goal_reached=false. That is the signature "
                "of claiming success while stuck. Do not retry "
                "complete_mission until you have: (1) stop_husky to clear "
                "the active task, (2) re-read state, (3) actually driven to "
                "the cell the brief requires. If the brief uses the legacy "
                "SE-corner goal, only call complete_mission when "
                "state.goal_reached == true. If the brief is multi-objective "
                "or custom, only call complete_mission AFTER the final "
                "motion settled with no fault."
            ),
        }
    payload: Dict[str, Any] = {
        "action": "complete_mission",
        "rationale": rationale.strip(),
    }
    if claimed_cells:
        payload["claimed_cells"] = claimed_cells
    return bridge_post("action", payload)


def _impl_try_get_known_map(**_: Any) -> Dict[str, Any]:
    """Ask the bridge for the maze graph. Returns {available: false, ...}
    when the bridge declines (world title contains 'Unknown')."""
    return bridge_get("maze")


def _impl_plan_path(
    from_col: int = None, from_row: int = None,
    to_col: int = None, to_row: int = None,
    **_: Any,
) -> Dict[str, Any]:
    """Deterministic BFS over the bridge's maze adjacency. Returns the
    shortest legal cell-sequence from `from` to `to`; the agent passes it
    straight into `execute_path`. `from_col`/`from_row` are optional — when
    omitted, the tool reads /state and uses the husky's actual current
    cell, so the agent never has to track pose itself. The agent's only
    real job is "where do I want to end up next"."""
    try:
        tc, tr = int(to_col), int(to_row)
    except (TypeError, ValueError):
        return {"error": "plan_path needs integer to_col, to_row"}
    if from_col is None or from_row is None:
        state = bridge_get("state") or {}
        cur = (state.get("current_cell") or {})
        try:
            fc, fr = int(cur.get("col")), int(cur.get("row"))
        except (TypeError, ValueError):
            return {
                "error": "plan_path could not infer from_cell from /state",
                "hint": "pass from_col + from_row explicitly",
            }
    else:
        try:
            fc, fr = int(from_col), int(from_row)
        except (TypeError, ValueError):
            return {"error": "from_col and from_row must be integers (or omit both to auto-detect)"}
    maze = bridge_get("maze")
    if not maze.get("available"):
        return {
            "error": "map_unavailable",
            "hint": maze.get("hint", "world has no known map; fall back to read_lidar"),
        }
    adj = maze.get("adjacency", {})
    start = (fc, fr)
    goal = (tc, tr)
    if start == goal:
        return {"path": [[fc, fr]], "hops": 0, "from": [fc, fr], "to": [tc, tr]}
    # Dijkstra with direction-change cost. Pure BFS gives the shortest
    # cell-count path, but each direction-change forces an in-place 90°
    # pivot — the controller's most wedge-prone maneuver. Adding a small
    # turn penalty biases the planner toward straight corridors. Going
    # 1 extra cell to skip a pivot is a clear win on this controller;
    # going 4 extra cells is usually not. The 3-per-turn penalty
    # interpolates that trade-off (k extra cells for 1 saved pivot is
    # accepted iff k <= 3).
    import heapq as _heapq
    TURN_PENALTY = 3
    def _dir_of(a, b):
        if b[0] > a[0]: return "E"
        if b[0] < a[0]: return "W"
        if b[1] > a[1]: return "N"
        if b[1] < a[1]: return "S"
        return None
    # State: (cell, incoming_direction). incoming_direction is None at the
    # start since the husky has no prior pivot to compare against.
    best_cost = {(start, None): 0}
    parent = {(start, None): None}
    pq = [(0, 0, start, None)]  # (cost, tie-breaker, cell, incoming_dir)
    counter = 0
    while pq:
        cost, _, here, incoming = _heapq.heappop(pq)
        if here == goal:
            end_state = (here, incoming)
            break
        if best_cost.get((here, incoming), float("inf")) < cost:
            continue
        for n in adj.get(f"{here[0]},{here[1]}", []):
            nt = (int(n[0]), int(n[1]))
            new_dir = _dir_of(here, nt)
            step_cost = 1
            if incoming is not None and new_dir != incoming:
                step_cost += TURN_PENALTY
            new_cost = cost + step_cost
            key = (nt, new_dir)
            if new_cost < best_cost.get(key, float("inf")):
                best_cost[key] = new_cost
                parent[key] = (here, incoming)
                counter += 1
                _heapq.heappush(pq, (new_cost, counter, nt, new_dir))
    else:
        return {
            "error": "no_path",
            "from": [fc, fr], "to": [tc, tr],
            "hint": "those cells are not reachable from each other in this maze",
        }
    # Reconstruct path from the best end-state.
    rev = []
    cur = end_state
    while cur is not None:
        rev.append([cur[0][0], cur[0][1]])
        cur = parent.get(cur)
    rev.reverse()
    # The first entry is `from`; execute_path treats the husky's CURRENT
    # cell as the start and only drives the rest. Return both shapes so
    # the agent can paste either straight in.
    return {
        "from": [fc, fr], "to": [tc, tr],
        "hops": len(rev) - 1,
        "path": rev,                # includes start; useful for narration
        "execute_path_cells": rev[1:],  # paste this into execute_path.cells
    }


# Module-level memory of where prior drive_to_cell calls ended up.
# Used to detect "agent called us again from the SAME wedged cell" and
# inject a perpendicular shake-free maneuver — a short drive to a
# different open neighbour, which reorients the husky and breaks the
# repeating-pivot deadlock that the wheel-only wedge escape sometimes
# can't crack on its own. Keyed by (col, row) -> consecutive-hit count.
_DRIVE_TO_CELL_PRIOR_WEDGE: Dict[tuple, int] = {}


def _impl_drive_to_cell(
    to_col: int = None, to_row: int = None,
    max_replans: int = 6,
    max_wall_seconds: float = 240.0,
    speed: float = 0.5,
    **_: Any,
) -> Dict[str, Any]:
    """High-level: drive the husky from its CURRENT cell to (to_col, to_row).

    Internally: read state → plan_path → execute_path. If execute_path
    times out mid-leg (goto_cell_timeout on a legal hop), read fresh
    state and re-plan from the new current cell, then continue.

    Bounded TWO ways so the tool always returns within the agent's HTTP
    budget:
      * max_replans (default 3) caps wedge retries
      * max_wall_seconds (default 300) caps total wall time

    Returns:
      {arrived: true,  cell: [c,r], replans, hops_driven, log}  on success
      {arrived: false, reason: "...", cell: [c,r], remaining_hops_estimate, log}
                                                              when out of budget

    On `arrived: false` the agent should re-call drive_to_cell with the
    same target to continue — the husky is closer than it was, and the
    next call will pick up from the new current cell.
    """
    import time as _time
    try:
        tc, tr = int(to_col), int(to_row)
    except (TypeError, ValueError):
        return {"error": "drive_to_cell needs integer to_col, to_row"}
    try:
        max_replans = max(0, int(max_replans))
    except (TypeError, ValueError):
        max_replans = 3
    try:
        max_wall_seconds = max(10.0, float(max_wall_seconds))
    except (TypeError, ValueError):
        max_wall_seconds = 300.0
    speed = float(speed) if speed is not None else 0.5

    t0 = _time.time()
    replans = 0
    hops_driven = 0
    last_fault = None
    log = []
    shake_attempted = False

    def _state_cell():
        st = bridge_get("state") or {}
        c = st.get("current_cell") or {}
        try:
            return (int(c.get("col")), int(c.get("row"))), st
        except (TypeError, ValueError):
            return None, st

    while True:
        elapsed = _time.time() - t0
        if elapsed > max_wall_seconds:
            cell, _ = _state_cell()
            # Bump the wedge counter so the NEXT drive_to_cell call from
            # the same cell triggers shake-free.
            if cell:
                _DRIVE_TO_CELL_PRIOR_WEDGE[cell] = _DRIVE_TO_CELL_PRIOR_WEDGE.get(cell, 0) + 1
            return {
                "arrived": False,
                "reason": "wall_time_budget_exhausted",
                "cell": list(cell) if cell else None,
                "wall_seconds": round(elapsed, 1),
                "replans": replans,
                "hops_driven": hops_driven,
                "last_fault": last_fault,
                "log": log,
                "hint": "Re-call drive_to_cell with the same target to continue from the new current cell.",
            }
        cell, state = _state_cell()
        if cell is None:
            return {
                "arrived": False,
                "reason": "could_not_read_current_cell",
                "state": state,
                "log": log,
            }
        fc, fr = cell
        if (fc, fr) == (tc, tr):
            _DRIVE_TO_CELL_PRIOR_WEDGE.clear()  # success — reset memory
            return {
                "arrived": True,
                "cell": [tc, tr],
                "replans": replans,
                "hops_driven": hops_driven,
                "wall_seconds": round(_time.time() - t0, 1),
                "last_fault": last_fault,
                "log": log,
            }
        # SHAKE-FREE: if a prior drive_to_cell call wedged at this exact
        # cell, briefly drive to a different open neighbour first. The
        # change of pose + heading often breaks the geometry that caused
        # the wedge. One attempt per drive_to_cell call so we don't waste
        # the wall budget.
        prior_wedges = _DRIVE_TO_CELL_PRIOR_WEDGE.get((fc, fr), 0)
        if prior_wedges >= 1 and not shake_attempted:
            shake_attempted = True
            maze_resp = bridge_get("maze")
            if maze_resp.get("available"):
                adj = maze_resp.get("adjacency", {})
                neighbours = adj.get(f"{fc},{fr}", []) or []
                # Prefer a neighbour we haven't tried as a wedge before AND
                # that isn't the target direction (a perpendicular detour
                # is more likely to unstick the body than charging again
                # straight at the same wall).
                best = None
                for n in neighbours:
                    nt = (int(n[0]), int(n[1]))
                    if _DRIVE_TO_CELL_PRIOR_WEDGE.get(nt, 0) > 0:
                        continue
                    best = n
                    if nt != (tc, tr):  # not the target direction
                        break
                if best is None and neighbours:
                    best = neighbours[0]
                if best is not None:
                    log.append({"shake_to": [int(best[0]), int(best[1])], "from_wedge": [fc, fr]})
                    bridge_post("action", {
                        "action": "goto_cell",
                        "col": int(best[0]), "row": int(best[1]),
                        "wait": True, "speed": speed,
                    })
                    # Re-read state next iteration; loop will plan from
                    # the new cell.
                    continue
        plan = _impl_plan_path(from_col=fc, from_row=fr, to_col=tc, to_row=tr)
        if "error" in plan:
            return {
                "arrived": False,
                "reason": f"plan_path: {plan['error']}",
                "cell": [fc, fr],
                "replans": replans,
                "hops_driven": hops_driven,
                "log": log,
            }
        remaining = plan.get("hops", 0)
        cells = plan.get("execute_path_cells", [])
        if not cells:
            return {
                "arrived": False,
                "reason": "empty_plan",
                "cell": [fc, fr],
                "log": log,
            }
        resp = bridge_post(
            "action",
            {"action": "execute_path", "cells": cells, "speed": speed},
        )
        log.append({
            "from": [fc, fr],
            "planned_hops": len(cells),
            "executed": resp.get("cells_executed", 0),
            "fault": resp.get("fault"),
        })
        hops_driven += int(resp.get("cells_executed", 0) or 0)
        last_fault = resp.get("fault")
        if resp.get("fault") is None and resp.get("cells_executed") == resp.get("cells_total"):
            continue  # re-verify arrival via /state next iteration
        # Both goto_cell_timeout (controller's own sim-time deadline) and
        # http_wait_timeout (client-side wall-time deadline waiting for
        # task idle) mean the same thing here: the controller stalled.
        # Re-plan and try again.
        if resp.get("fault") in ("goto_cell_timeout", "http_wait_timeout", "drive_forward_timeout", "turn_timeout"):
            replans += 1
            if replans > max_replans:
                cell_after, _ = _state_cell()
                if cell_after:
                    _DRIVE_TO_CELL_PRIOR_WEDGE[cell_after] = _DRIVE_TO_CELL_PRIOR_WEDGE.get(cell_after, 0) + 1
                return {
                    "arrived": False,
                    "reason": "max_replans_exhausted",
                    "cell": list(cell_after) if cell_after else [fc, fr],
                    "remaining_hops_estimate": max(0, remaining - resp.get("cells_executed", 0)),
                    "replans": replans,
                    "hops_driven": hops_driven,
                    "wall_seconds": round(_time.time() - t0, 1),
                    "log": log,
                    "hint": "Re-call drive_to_cell with the same target to continue from the new current cell.",
                }
            continue
        return {
            "arrived": False,
            "reason": resp.get("fault") or "execute_path_failed",
            "cell": [fc, fr],
            "replans": replans,
            "hops_driven": hops_driven,
            "bridge_response": resp,
            "log": log,
        }


def _impl_read_lidar(**_: Any) -> Dict[str, Any]:
    """16-ray supervisor-raycast lidar in the body frame.
    angles_rad[i] = body angle (0=forward, +pi/2=left, -pi/2=right);
    ranges_m[i]   = nearest wall along that ray, or max_range if free."""
    return bridge_get("lidar")


def _impl_read_camera(**_: Any) -> Dict[str, Any]:
    """Front camera frame (320x240 by default) as a base64 PNG.

    Only available when the world ships a Camera node named
    'front_camera'. For vision-only worlds the agent uses this to
    identify what's around it (colours, shapes, text on signs)."""
    return bridge_get("camera")


_WORLD_CARDINALS = ("east", "north", "west", "south")
_BODY_QUARTER_OFFSET = {"front": 0, "left": 1, "back": 2, "right": -1}
_CARDINAL_DELTAS = {"east": (1, 0), "west": (-1, 0), "north": (0, 1), "south": (0, -1)}


def _yaw_to_cardinal_index(yaw: float) -> int:
    import math as _m
    return int(round((yaw or 0.0) / (_m.pi / 2))) % 4


def _body_to_world_cardinal(body_cam: str, yaw: float) -> str:
    base = _yaw_to_cardinal_index(yaw)
    return _WORLD_CARDINALS[(base + _BODY_QUARTER_OFFSET[body_cam]) % 4]


def _digest_scan(raw: Dict[str, Any]) -> Dict[str, Any]:
    """Compress the raw eye-scan output into a small map keyed by world
    cardinal. Drops the per-cardinal score / patch_std / mean_brightness
    fields the LLM does not reason about, and replaces them with one of
    {open, blocked, ambiguous}. Same compression `walk_one_cell` /
    `follow_corridor` apply server-side; doing it here lets the existing
    `scan_surroundings` tool benefit too without a bridge change."""
    if not isinstance(raw, dict) or "cameras" not in raw:
        return raw
    pose = raw.get("pose") or raw.get("tracking_pose") or {}
    yaw = float(pose.get("yaw", 0.0) or 0.0)
    cur = raw.get("current_cell") or {}
    cur_col, cur_row = cur.get("col"), cur.get("row")
    visited = raw.get("visited_cells") or []
    cams = raw.get("cameras") or {}
    by_world: Dict[str, str] = {}
    marker = None
    for body_name in ("front", "right", "back", "left"):
        c = cams.get(body_name) or {}
        wc = _body_to_world_cardinal(body_name, yaw)
        score = float(c.get("wall_close_score", 0.0) or 0.0)
        wall = bool(c.get("wall_close", False))
        if wall or score >= 0.6:
            tag = "blocked"
        elif score >= 0.3:
            tag = "ambiguous"
        else:
            tag = "open"
        by_world[wc] = tag
        m_color = c.get("marker")
        try:
            m_frac = float(c.get("marker_fraction") or 0.0)
        except Exception:
            m_frac = 0.0
        if m_color and m_frac >= 0.005 and (marker is None or m_frac > marker.get("fraction", 0.0)):
            cent = c.get("marker_centroid") or {}
            marker = {
                "color": m_color,
                "world_cardinal": wc,
                "fraction": round(m_frac, 4),
                "centroid_x_norm": (round(float(cent.get("x_norm", 0.5)), 2) if cent else None),
                "approach_recommended": (not wall) and m_frac > 0.05,
                "adjacent": wall and m_frac > 0.20,
            }
    cols, rows = 11, 11  # demo maze size — bridge maze constants
    visited_set = {(int(c[0]), int(c[1])) for c in visited if isinstance(c, (list, tuple)) and len(c) == 2}
    open_cards = [c for c, t in by_world.items() if t == "open"]
    unvisited_open = []
    if cur_col is not None and cur_row is not None:
        for c in open_cards:
            dc, dr = _CARDINAL_DELTAS[c]
            tc, tr = cur_col + dc, cur_row + dr
            if 0 <= tc < cols and 0 <= tr < rows and (tc, tr) not in visited_set:
                unvisited_open.append(c)
    return {
        "current_cell": [cur_col, cur_row] if cur_col is not None else None,
        "facing": _WORLD_CARDINALS[_yaw_to_cardinal_index(yaw)],
        "open": sorted(open_cards),
        "blocked": sorted([c for c, t in by_world.items() if t == "blocked"]),
        "ambiguous": sorted([c for c, t in by_world.items() if t == "ambiguous"]),
        "unvisited_open": sorted(unvisited_open),
        "marker": marker,
    }


def _impl_scan_surroundings(**_: Any) -> Dict[str, Any]:
    """Compact perception digest. Returns world-cardinal tags
    (open/blocked/ambiguous), unvisited open cardinals, and any marker
    spotted — see _digest_scan. ~250 bytes vs the raw eye scan's
    ~1.3 KB. Use this whenever you want a 'what's around me' check.
    For most navigation you should call `walk_one_cell` or
    `follow_corridor` instead — those move AND return a fresh digest."""
    raw = bridge_get("scan")
    if not isinstance(raw, dict) or "error" in raw or "cameras" not in raw:
        return raw
    return _digest_scan(raw)


def _impl_walk_one_cell(cardinal: str = "", speed: float = 0.5, **_: Any) -> Dict[str, Any]:
    """Drive one cell in a world cardinal (north/south/east/west).
    Bridge handles the body-frame mapping, the in-place pivot pre-snap,
    the 2-m drive, and the destination snap-to-cell. Refuses (without
    moving) if the eye flags wall_close in the target direction.
    Returns: {ok, from_cell, to_cell, fault, scan (digest), final_pose}."""
    if not cardinal:
        return {"error": "cardinal required: one of north, south, east, west"}
    return bridge_post("action", {
        "action": "walk_one_cell",
        "cardinal": cardinal,
        "speed": float(speed),
    })


def _impl_follow_corridor(cardinal: str = "", max_cells: int = 8,
                          stop_on_marker: bool = True, speed: float = 0.5,
                          **_: Any) -> Dict[str, Any]:
    """Walk in a world cardinal until: (a) a wall blocks, (b) a coloured
    marker becomes prominent (when stop_on_marker), or (c) max_cells hit.
    Replaces the per-cell scan→drive loop with one tool call for a whole
    straight corridor. Returns: {ok, cardinal, cells_walked, cells_total,
    stop_reason, scan (digest at final cell), final_pose, first_refusal}."""
    if not cardinal:
        return {"error": "cardinal required: one of north, south, east, west"}
    return bridge_post("action", {
        "action": "follow_corridor",
        "cardinal": cardinal,
        "max_cells": int(max_cells),
        "stop_on_marker": bool(stop_on_marker),
        "speed": float(speed),
    })


def _impl_auto_explore(target_color: str = "", max_cells: int = 50,
                       step_into_marker: bool = True, speed: float = 0.5,
                       **_: Any) -> Dict[str, Any]:
    """DFS frontier-explorer that runs the entire maze-solve in one tool
    call. The bridge handles per-cell scan, cardinal selection, walking,
    and backtracking. Optional `target_color` (red/green/blue) makes the
    explorer chase a coloured cylinder marker when one becomes visible
    in the digest. With `step_into_marker=True` (default), drives into
    the marker cell once adjacent. Returns:
        {ok, stop_reason ∈ {marker_reached, marker_adjacent,
        marker_adjacent_at_start, exhausted_no_unvisited, max_cells, …},
        marker_found, cells_walked, cells_total, final_cell, scan,
        final_pose, dead_ends_hit}.
    USE THIS as the primary tool for the blind world. The agent's job
    shrinks to: read brief, call auto_explore(target_color="red"),
    inspect result, complete_mission with rationale + claimed cells."""
    payload: Dict[str, Any] = {
        "action": "auto_explore",
        "max_cells": int(max_cells),
        "step_into_marker": bool(step_into_marker),
        "speed": float(speed),
    }
    if target_color:
        if target_color not in {"red", "green", "blue"}:
            return {"error": "target_color must be one of red, green, blue (or omit for pure exploration)"}
        payload["target_color"] = target_color
    return bridge_post("action", payload)


def _impl_stop_husky(**_: Any) -> Dict[str, Any]:
    return bridge_post("action", {"action": "stop"})


def _impl_reset_husky(**_: Any) -> Dict[str, Any]:
    return bridge_post("action", {"action": "reset"})


def _impl_snap_to_cell(col: int = -1, row: int = -1, yaw: float = 0.0, **_: Any) -> Dict[str, Any]:
    if col < 0 or row < 0:
        return {"error": "col and row are required (0..10)"}
    return bridge_post("action", {
        "action": "snap_to_cell",
        "col": int(col),
        "row": int(row),
        "yaw": float(yaw),
    })


def _impl_set_velocity(linear: float = 0.0, angular: float = 0.0, **_: Any) -> Dict[str, Any]:
    return bridge_post("action", {
        "action": "set_velocity",
        "linear": float(linear),
        "angular": float(angular),
    })


def _impl_drive_forward(distance: float = 0.0, speed: float = 0.5, **_: Any) -> Dict[str, Any]:
    return bridge_post("action", {
        "action": "drive_forward",
        "distance": float(distance),
        "speed": float(speed),
    })


def _impl_turn(angle: float = 0.0, speed: float = 0.6, **_: Any) -> Dict[str, Any]:
    return bridge_post("action", {
        "action": "turn",
        "angle": float(angle),
        "speed": float(speed),
    })


def _impl_goto_cell(col: int = -1, row: int = -1, speed: float = 0.5,
                    wait: bool = False, auto_snap_threshold_m: Any = None,
                    prev_cell: Any = None, **_: Any) -> Dict[str, Any]:
    """Drive to the centre of cell (col, row).

    Three calling conventions are supported, one per husky_maze variant:

    - **v1 (default, fire-and-forget):** call without `wait` and the
      bridge returns immediately. The agent must poll `get_state` until
      `mode == "idle"` and call `snap_to_cell` after each move to
      re-anchor.
    - **v2 (blocking + auto-snap):** pass `wait=True` and optionally
      `auto_snap_threshold_m=0.30` plus `prev_cell=[c,r]`. The bridge
      blocks until the husky has arrived (or faulted), pre-snaps to
      `prev_cell` with the new cardinal yaw before the move, and
      auto-snaps to the destination centre when residual drift exceeds
      the threshold. The agent gets one response per move.
    - **v3 (recovery only):** call with `wait=True` after a partial
      `execute_path` fault to drive a single cell. Equivalent to the
      v2 form."""
    if col < 0 or row < 0:
        return {"error": "col and row are required (0..10)"}
    payload: Dict[str, Any] = {
        "action": "goto_cell",
        "col": int(col),
        "row": int(row),
        "speed": float(speed),
    }
    if wait:
        payload["wait"] = True
        if auto_snap_threshold_m is not None:
            payload["auto_snap_threshold_m"] = float(auto_snap_threshold_m)
        if isinstance(prev_cell, (list, tuple)) and len(prev_cell) == 2:
            payload["prev_cell"] = [int(prev_cell[0]), int(prev_cell[1])]
    return bridge_post("action", payload)


def _impl_execute_path(cells: Any = None, speed: float = 0.5,
                       snap_drift_threshold_m: float = 0.40,
                       per_cell_timeout_s: float = 30.0,
                       **_: Any) -> Dict[str, Any]:
    """v3: drive a list of [col, row] cells in sequence on the bridge,
    with internal pose-settle waits and drift-gated auto-snap. One tool
    call replaces dozens of goto_cell + get_state + snap_to_cell calls
    in v1 and v2. Returns a per-cell summary plus aggregate counts."""
    if not isinstance(cells, list) or not cells:
        return {"error": "cells: non-empty list of [col, row] pairs required"}
    norm = []
    for i, c in enumerate(cells):
        try:
            norm.append([int(c[0]), int(c[1])])
        except Exception:
            return {"error": f"bad cell at index {i}: expected [col, row] integers"}
    return bridge_post("action", {
        "action": "execute_path",
        "cells": norm,
        "speed": float(speed),
        "snap_drift_threshold_m": float(snap_drift_threshold_m),
        "per_cell_timeout_s": float(per_cell_timeout_s),
    })


def _impl_replay_recalled_path(memory_id: str = "", drop_head: bool = True,
                               speed: float = 0.5,
                               snap_drift_threshold_m: float = 0.40,
                               per_cell_timeout_s: float = 30.0,
                               **_: Any) -> Dict[str, Any]:
    # Default snap threshold raised 0.10 -> 0.40 to match the bridge's
    # default and cut visible snap-pop frequency. See bridge action
    # execute_path's comment for the rationale: forward-drive cells stay
    # well under the gate; only in-place pivots trip it. Agent can pass
    # a tighter threshold for missions that need stricter alignment.
    """Replay the cell list from a saved memory directly through
    execute_path, without ever asking the LLM to retype the cells.

    Background: in direction A's A2 retry the LLM truncated the
    72-cell saved path to 56 cells when emitting the JSON of an
    execute_path tool call — even with the parsed list provided as a
    structured `extracted_paths` field in the recall response, even
    with the prompt demanding verbatim copy. The bottleneck is the
    model's output token budget for tool-call arguments, not the
    input pipeline.

    This tool removes the LLM from the path-emission loop. The agent
    passes a memory_id (which it gets from the recall response's
    long_term-tier hits — small, easy to pass faithfully). The driver
    looks up the saved memory by id locally, parses the cells from
    the body, optionally drops the start cell, and posts execute_path
    with the full list. Truncation is impossible because the LLM is
    no longer the carrier."""
    mid = (memory_id or "").strip()
    if not mid:
        return {"error": "memory_id is required (one of recall.tiers.long_term[].id)"}
    looked_up = _load_memory_by_id(mid)
    if "error" in looked_up:
        return looked_up
    cells = looked_up.get("extracted_path")
    if not cells:
        return {
            "error": (
                "memory has no parseable 'Full ordered path:' line in its "
                "body. save_local_memory body must include a line like "
                "'Full ordered path: [[0, 10], [0, 9], ..., [10, 0]]' for "
                "this tool to work."
            ),
            "id": mid,
            "title": looked_up.get("title"),
        }
    # Validate the stored path actually starts from the husky's current
    # cell. A saved "recovery remainder" memory (e.g. partial path
    # captured after a fault forced re-planning mid-mission) starts
    # somewhere mid-maze; replaying it from the start cell would drive
    # the husky into walls. Refuse loudly so the agent can pick a
    # different memory or fall back to try_get_known_map.
    state_resp = bridge_get("state")
    cur_x = state_resp.get("x") if isinstance(state_resp, dict) else None
    cur_y = state_resp.get("y") if isinstance(state_resp, dict) else None
    head_cell = cells[0]
    head_x = -10.0 + 2.0 * head_cell[0]
    head_y = -10.0 + 2.0 * head_cell[1]
    if cur_x is not None and cur_y is not None:
        import math as _math
        head_dist = _math.hypot(cur_x - head_x, cur_y - head_y)
        if head_dist > 1.0:
            cur_col = int(round((cur_x - (-10.0)) / 2.0))
            cur_row = int(round((cur_y - (-10.0)) / 2.0))
            return {
                "error": (
                    "saved path's start cell does not match husky's "
                    "current cell — refusing to replay. Pick a "
                    "different memory_id whose stored path starts at "
                    "the husky's current cell, or fall back to "
                    "try_get_known_map + execute_path."
                ),
                "id": looked_up.get("id"),
                "title": looked_up.get("title"),
                "stored_path_starts_at": [int(head_cell[0]), int(head_cell[1])],
                "husky_current_cell": [cur_col, cur_row],
                "head_distance_m": round(head_dist, 2),
            }
    payload_cells = list(cells[1:]) if drop_head and len(cells) > 1 else list(cells)
    bridge_resp = bridge_post("action", {
        "action": "execute_path",
        "cells": payload_cells,
        "speed": float(speed),
        "snap_drift_threshold_m": float(snap_drift_threshold_m),
        "per_cell_timeout_s": float(per_cell_timeout_s),
    })
    # Wrap with replay metadata so the agent sees what was replayed.
    return {
        "replayed_from": {
            "id": looked_up.get("id"),
            "title": looked_up.get("title"),
            "tags": looked_up.get("tags"),
            "stored_path_length": len(cells),
            "head_dropped": drop_head and len(cells) > 1,
            "executed_path_length": len(payload_cells),
        },
        "bridge_response": bridge_resp,
    }


# ----- SPECS --------------------------------------------------------------

SPECS = [
    ToolSpec(
        name="get_capabilities",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Read the bridge capabilities: robot kinematics, max speeds, "
            "the maze constants, the world title, and a flag "
            "`map_available` that says whether try_get_known_map will "
            "return data for this world. Call once at session start. "
            "Use `map_available` to decide between the BFS strategy and "
            "the lidar-explore strategy — that's the whole point of the "
            "agent layer."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_get_capabilities,
        tags=["bridge", "discovery"],
    ),
    ToolSpec(
        name="get_state",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Live snapshot of the husky: x, y, yaw, v_linear, v_angular, "
            "wheel commands, mode, fault, sim_time, target, goal_reached "
            "(legacy: hardcoded 0.8 m radius around (10,-10)), and "
            "mission_complete (true once you have called complete_mission). "
            "Always read this before issuing the next motion."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_get_state,
        tags=["bridge", "telemetry"],
    ),
    ToolSpec(
        name="read_mission_brief",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Read the operator's mission brief — free-form natural language "
            "from WorldInfo.info that tells you what success looks like. "
            "ALWAYS call this right after get_capabilities. The brief is "
            "the only authoritative source of intent — neither map_available "
            "nor goal_reached can substitute for it. Different worlds carry "
            "different briefs (drive to a corner, visit all corners, find "
            "the deepest dead-end, etc.); your job is to interpret the "
            "brief, plan to satisfy it, then call complete_mission."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_read_mission_brief,
        tags=["mission"],
    ),
    ToolSpec(
        name="complete_mission",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Tell the bridge you have satisfied the mission brief. Pass a "
            "one-sentence `rationale` summarising why, and `claimed_cells` "
            "as a list of [col,row] pairs you visited that prove completion. "
            "The bridge VERIFIES the claim against its ground-truth visited "
            "trail: every cell in claimed_cells must appear in visited, the "
            "world's required cells (e.g. all four corners on the Corners "
            "Tour) must be in visited, and legacy SE-goal worlds need "
            "state.goal_reached=true. A failed claim returns HTTP 409 with "
            "`reasons` telling you what is missing; replan, drive there, then "
            "re-call. Call this exactly once per accepted mission, AFTER "
            "the husky's final motion settled."
        ),
        parameters={
            "type": "object",
            "properties": {
                "rationale": {"type": "string", "description": "One-sentence why the mission is satisfied."},
                "claimed_cells": {
                    "type": "array",
                    "items": {"type": "array", "items": {"type": "integer"}},
                    "description": "Optional list of [col,row] pairs you visited.",
                },
            },
            "required": ["rationale"],
        },
        impl=_impl_complete_mission,
        tags=["mission"],
    ),
    ToolSpec(
        name="try_get_known_map",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Ask the bridge for the maze adjacency graph. Returns "
            "{available: true, adjacency: {'col,row': [[c,r], ...]}, "
            "start, goal, ...} when the world reveals its map; returns "
            "{available: false, hint: ...} otherwise. If unavailable, "
            "FALL BACK TO read_lidar — never assume any wall layout. "
            "When available, prefer `plan_path` over reading the adjacency "
            "yourself — `plan_path` is a deterministic BFS that returns "
            "the legal hop sequence directly."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_try_get_known_map,
        tags=["maze", "planning"],
    ),
    ToolSpec(
        name="drive_to_cell",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "High-level: drive the husky from its current cell to "
            "(to_col, to_row). Reads /state, plan_paths, execute_paths, "
            "and re-plans on wheel-controller wedges — all in one call. "
            "Bounded by max_replans (default 8). For known-map worlds "
            "this is the only navigation tool you need: one call per "
            "waypoint. Returns {arrived: true|false, cell: [c,r], "
            "replans, hops_driven, log}. Falls back via map_unavailable "
            "errors on Unknown worlds (use lidar primitives there)."
        ),
        parameters={
            "type": "object",
            "properties": {
                "to_col": {"type": "integer", "description": "Target col."},
                "to_row": {"type": "integer", "description": "Target row."},
                "max_replans": {"type": "integer", "description": "Wedge-replan budget (default 8)."},
                "speed": {"type": "number", "description": "Fraction of max_linear, 0..1 (default 0.5)."},
            },
            "required": ["to_col", "to_row"],
        },
        impl=_impl_drive_to_cell,
        tags=["maze", "navigation"],
    ),
    ToolSpec(
        name="plan_path",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Deterministic BFS shortest path over the maze adjacency. "
            "Returns {path: [[c,r], ...], execute_path_cells: [[c,r], ...], "
            "hops: int}. `execute_path_cells` is exactly what `execute_path` "
            "expects in its `cells` field. `from_col`/`from_row` are "
            "OPTIONAL: omit them and the tool reads /state and plans from "
            "the husky's actual current cell — you almost always want this. "
            "`to_col`/`to_row` are required. Returns {error: 'map_unavailable'} "
            "on Unknown worlds (fall back to read_lidar there)."
        ),
        parameters={
            "type": "object",
            "properties": {
                "to_col": {"type": "integer", "description": "Target col."},
                "to_row": {"type": "integer", "description": "Target row."},
                "from_col": {"type": "integer", "description": "Optional source col; omit to use the husky's current cell."},
                "from_row": {"type": "integer", "description": "Optional source row; omit to use the husky's current cell."},
            },
            "required": ["to_col", "to_row"],
        },
        impl=_impl_plan_path,
        tags=["maze", "planning"],
    ),
    ToolSpec(
        name="read_lidar",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "16-ray supervisor-raycast lidar around the husky. Returns "
            "{angles_rad: float[16], ranges_m: float[16], max_range_m, "
            "pose}. angles_rad[i] is the BODY-FRAME angle (0 = forward, "
            "+pi/2 = left, -pi/2 = right, +/-pi = back). ranges_m[i] is "
            "the distance to the nearest wall along that ray, or "
            "max_range_m if no wall is within range. Use this when "
            "try_get_known_map says available=false: a 4-cell adjacency "
            "check is `range > cell_size_m + clearance` along the four "
            "cardinal body angles."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_read_lidar,
        tags=["sensor", "lidar"],
    ),
    ToolSpec(
        name="read_camera",
        tier=SAFE,
        # ON_DEMAND so the agent NEVER sees this in its tool surface. Vision
        # is perception-as-tool: the sidecar's BGRA analyser runs in Python,
        # the agent reads the symbolic digest via scan_surroundings, and
        # raw frames never enter chat context (where they'd cost 30-70 K
        # input tokens each). read_camera stays implementable for operator
        # diagnostics — POST /camera directly if you need pixels.
        surface=ON_DEMAND,
        description=(
            "Raw camera frames (320x240 base64 PNG). NOT exposed to the "
            "agent — perception goes through scan_surroundings, which "
            "returns ~250 bytes of pre-extracted tags vs ~100 KB of "
            "pixels. Operator-only diagnostic; do not re-enable in the "
            "agent surface without redesigning the vision pipeline."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_read_camera,
        tags=["sensor", "camera", "vision"],
    ),
    ToolSpec(
        name="scan_surroundings",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Compact world-cardinal perception digest. Returns "
            "{current_cell, facing, open: [cardinals…], blocked: "
            "[cardinals…], ambiguous: [cardinals…], unvisited_open: "
            "[cardinals…], marker: null OR {color, world_cardinal, "
            "fraction, centroid_x_norm, approach_recommended, "
            "adjacent}}. Each cardinal is one of north/south/east/"
            "west; status is open (drive freely), blocked (wall_close "
            "detected — do not drive), or ambiguous (uncertain — "
            "consider read_camera fallback). Use for 'what's around "
            "me' queries when you don't intend to move yet. To MOVE, "
            "prefer `walk_one_cell` or `follow_corridor` — those "
            "drive AND return a fresh digest in one tool call."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_scan_surroundings,
        tags=["sensor", "vision", "perception"],
    ),
    ToolSpec(
        name="walk_one_cell",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Drive one cell in a world cardinal (north/south/east/west). "
            "Bridge handles body-frame mapping (you don't compute yaw), "
            "in-place pivot pre-snap, the 2-m drive, and the destination "
            "snap-to-cell. Refuses without moving if the eye flags "
            "wall_close in the target cardinal. Returns "
            "{ok, from_cell, to_cell, fault, scan (digest), final_pose}. "
            "USE THIS as the primary motion primitive on the blind world "
            "— one chat round-trip per cell instead of "
            "scan→turn→drive→snap→scan (4-5 round-trips)."
        ),
        parameters={
            "type": "object",
            "properties": {
                "cardinal": {
                    "type": "string",
                    "enum": ["north", "south", "east", "west"],
                    "description": "World cardinal to step toward.",
                },
                "speed": {
                    "type": "number",
                    "description": "fraction of max linear speed, default 0.5",
                },
            },
            "required": ["cardinal"],
        },
        impl=_impl_walk_one_cell,
        tags=["motion", "vision", "perception"],
    ),
    ToolSpec(
        name="follow_corridor",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Walk in a world cardinal until (a) a wall blocks the way, "
            "(b) a coloured marker becomes prominent (when "
            "stop_on_marker=true), or (c) max_cells hit. Replaces the "
            "per-cell scan/drive loop for a whole straight corridor. "
            "Returns {ok, cardinal, cells_walked: [[c,r],...], "
            "cells_total, stop_reason ∈ {marker_seen, next_blocked, "
            "max_cells, first_step_blocked, …}, scan (digest at final "
            "cell), final_pose, first_refusal}. USE THIS when scan "
            "shows a long open corridor in some cardinal — collapses "
            "4 cells of east-driving into 1 chat round-trip."
        ),
        parameters={
            "type": "object",
            "properties": {
                "cardinal": {
                    "type": "string",
                    "enum": ["north", "south", "east", "west"],
                    "description": "World cardinal to walk along.",
                },
                "max_cells": {
                    "type": "integer",
                    "description": "Stop after this many cells. Default 8.",
                },
                "stop_on_marker": {
                    "type": "boolean",
                    "description": "Stop when a marker becomes prominent (approach_recommended OR adjacent). Default true.",
                },
                "speed": {
                    "type": "number",
                    "description": "fraction of max linear speed, default 0.5",
                },
            },
            "required": ["cardinal"],
        },
        impl=_impl_follow_corridor,
        tags=["motion", "vision", "perception", "batch"],
    ),
    ToolSpec(
        name="auto_explore",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "DFS frontier-explorer that runs the entire blind-world maze "
            "solve in ONE tool call. Bridge handles per-cell scan, "
            "cardinal selection, walking, and parent-chain backtracking. "
            "Optional `target_color` (red/green/blue) makes the explorer "
            "chase a coloured cylinder when one becomes visible. With "
            "`step_into_marker=true` (default), drives into the marker "
            "cell once adjacent. Returns "
            "{ok, stop_reason ∈ {marker_reached, marker_adjacent, "
            "marker_adjacent_at_start, exhausted_no_unvisited, max_cells, "
            "…}, marker_found, cells_walked: [[c,r],...], cells_total, "
            "final_cell, scan, final_pose, dead_ends_hit}. USE THIS as "
            "the PRIMARY tool on the blind world — your job shrinks to "
            "(1) read brief, (2) call auto_explore with the target "
            "colour, (3) on success call complete_mission with the "
            "marker_found cell as the rationale."
        ),
        parameters={
            "type": "object",
            "properties": {
                "target_color": {
                    "type": "string",
                    "enum": ["red", "green", "blue"],
                    "description": "Marker colour to chase. Omit for pure exploration.",
                },
                "max_cells": {
                    "type": "integer",
                    "description": "Hard cap on the explorer's step count. Default 50.",
                },
                "step_into_marker": {
                    "type": "boolean",
                    "description": "When true, drive INTO the marker cell once adjacent. Default true.",
                },
                "speed": {
                    "type": "number",
                    "description": "fraction of max linear speed, default 0.5",
                },
            },
        },
        impl=_impl_auto_explore,
        tags=["motion", "vision", "perception", "batch", "explore"],
    ),
    ToolSpec(
        name="stop_husky",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Emergency halt: zero both wheels, mode -> stopped. ALWAYS "
            "AVAILABLE - never gated. Idempotent. Call immediately on "
            "fault, stale telemetry, or any unsafe state."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_stop_husky,
        tags=["safety"],
    ),
    ToolSpec(
        name="reset_husky",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Teleport the husky back to the start cell (col=0, row=10) "
            "and reset its physics. Use only on operator request."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_reset_husky,
        tags=["admin"],
    ),
    ToolSpec(
        name="snap_to_cell",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Teleport the husky to the centre of cell (col, row) with "
            "the given cardinal yaw (0=east, +pi/2=north, +/-pi=west, "
            "-pi/2=south). DELIBERATE DEMO CONCESSION: skid-steer pivots "
            "in OmniSim accumulate ~0.5 m of drift per 90 deg turn, which "
            "compounds across cells in tight maze corridors. Call this "
            "after each successful goto_cell / drive_forward to re-anchor "
            "to the grid so subsequent steps don't wedge against walls. "
            "Use it the same way the standalone solver does — verify with "
            "get_state that you actually reached the cell first; never "
            "snap to a cell you didn't earn by driving there."
        ),
        parameters={
            "type": "object",
            "properties": {
                "col": {"type": "integer", "description": "0..10"},
                "row": {"type": "integer", "description": "0..10"},
                "yaw": {"type": "number", "description": "radians; cardinal heading: 0, +/-pi/2, or +/-pi"},
            },
            "required": ["col", "row", "yaw"],
        },
        impl=_impl_snap_to_cell,
        tags=["admin", "grid"],
    ),
    ToolSpec(
        name="set_velocity",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Command an open-loop body twist: linear m/s along the husky's "
            "+x heading, angular rad/s around z (positive = left turn). "
            "Bridge clamps to {max_linear_m_s, max_angular_r_s}. Holds "
            "until the next action. Prefer goto_cell for maze navigation; "
            "use this only to escape stuck states or for fine adjustments."
        ),
        parameters={
            "type": "object",
            "properties": {
                "linear": {"type": "number", "description": "m/s, ~[-0.99, 0.99]"},
                "angular": {"type": "number", "description": "rad/s, ~[-3.47, 3.47]"},
            },
        },
        impl=_impl_set_velocity,
        tags=["motion"],
    ),
    ToolSpec(
        name="drive_forward",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Drive the husky forward by `distance` metres at `speed * "
            "max_linear_m_s` (default 0.5). Closed-loop on pose, returns "
            "when within 0.15 m or after 30 s. Heading is held constant. "
            "Negative distance reverses."
        ),
        parameters={
            "type": "object",
            "properties": {
                "distance": {"type": "number", "description": "metres; negative = reverse"},
                "speed": {"type": "number", "description": "fraction of max linear speed, default 0.5"},
            },
            "required": ["distance"],
        },
        impl=_impl_drive_forward,
        tags=["motion"],
    ),
    ToolSpec(
        name="turn",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Rotate in place by `angle` radians at `speed * max_angular_r_s` "
            "(default 0.6). Positive = left turn. Closed-loop on yaw, "
            "returns when within 0.05 rad or after 30 s."
        ),
        parameters={
            "type": "object",
            "properties": {
                "angle": {"type": "number", "description": "radians; positive = left"},
                "speed": {"type": "number", "description": "fraction of max angular speed, default 0.6"},
            },
            "required": ["angle"],
        },
        impl=_impl_turn,
        tags=["motion"],
    ),
    ToolSpec(
        name="goto_cell",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Drive to the centre of cell (col, row). Three modes, one per "
            "husky_maze variant: "
            "(v1) fire-and-forget — call with no `wait` and the bridge "
            "returns immediately; the agent must poll `get_state` and call "
            "`snap_to_cell` after each move. "
            "(v2) blocking + auto-snap — pass `wait=true`, `prev_cell=[c,r]`, "
            "and `auto_snap_threshold_m=0.30`; the bridge pre-snaps to the "
            "previous cell with the upcoming cardinal yaw, drives, and "
            "auto-snaps on drift; one response per move. "
            "(v3) recovery — same as v2, used after a partial `execute_path` "
            "fault. For normal v3 navigation use `execute_path` instead."
        ),
        parameters={
            "type": "object",
            "properties": {
                "col": {"type": "integer", "description": "0..10 (west to east)"},
                "row": {"type": "integer", "description": "0..10 (south to north)"},
                "speed": {"type": "number", "description": "fraction of max linear, default 0.5"},
                "wait": {"type": "boolean", "description": "v2/v3: block until arrived or faulted. Default false (v1 behaviour)."},
                "auto_snap_threshold_m": {"type": "number", "description": "v2/v3: auto-snap to cell centre when drift exceeds this. Default off."},
                "prev_cell": {"type": "array", "items": {"type": "integer"}, "description": "v2/v3: [col, row] of the cell we are leaving. Used to pick the cardinal yaw on pre-snap."},
            },
            "required": ["col", "row"],
        },
        impl=_impl_goto_cell,
        tags=["motion", "maze"],
    ),
    ToolSpec(
        name="execute_path",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "v3 batch driver: drive a list of [col, row] cells in order. "
            "Bridge handles per-cell pose-settle waits and drift-gated "
            "auto-snap internally. Returns ONCE the whole path is done "
            "(success or fault on a specific cell). This replaces the "
            "~290-turn cell-by-cell loop with a single LLM round-trip. "
            "The cells array is the full BFS plan with the start cell "
            "DROPPED from the head (the husky is already there). "
            "Refuse paths with a non-adjacent step — every consecutive "
            "pair must be 4-neighbours sharing an open passage. Read "
            "the response's `fault` and `cells_executed` fields; on a "
            "partial fault, replan a remainder from the final pose and "
            "issue a new execute_path."
        ),
        parameters={
            "type": "object",
            "properties": {
                "cells": {
                    "type": "array",
                    "items": {"type": "array", "items": {"type": "integer"}},
                    "description": "ordered list of [col, row] cells — the BFS plan minus the start cell.",
                },
                "speed": {"type": "number", "description": "fraction of max linear, default 0.5"},
                "snap_drift_threshold_m": {
                    "type": "number",
                    "description": "Per-cell drift threshold above which the bridge auto-snaps. Default 0.30 m.",
                },
                "per_cell_timeout_s": {
                    "type": "number",
                    "description": "Bridge-side timeout per cell move. Default 30 s.",
                },
            },
            "required": ["cells"],
        },
        impl=_impl_execute_path,
        tags=["motion", "maze", "batch"],
    ),
    ToolSpec(
        name="replay_recalled_path",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Replay the cell list saved in a recalled memory directly "
            "via execute_path WITHOUT having to retype the cells. The "
            "agent passes a `memory_id` (from a recall hit's "
            "long_term tier — short string, easy to copy faithfully); "
            "the driver looks up the saved body, parses the "
            "`Full ordered path: [[c,r], ...]` line, optionally drops "
            "the start cell, and posts execute_path with the full "
            "list. Use this whenever recall returns a usable plan for "
            "the current world — it is more reliable than emitting "
            "the cells yourself, because LLM output budgets can "
            "truncate long JSON arrays. Returns "
            "`{replayed_from: {id, title, stored_path_length, "
            "head_dropped, executed_path_length}, bridge_response: "
            "<execute_path response>}`. Inspect "
            "`bridge_response.cells_executed` and "
            "`bridge_response.fault` exactly as you would after a "
            "direct execute_path call."
        ),
        parameters={
            "type": "object",
            "properties": {
                "memory_id": {
                    "type": "string",
                    "description": "id field of the long_term memory hit returned by recall.",
                },
                "drop_head": {
                    "type": "boolean",
                    "description": "Drop the first cell of the saved path before driving (true when the saved path includes the start cell, which is the common case). Default true.",
                },
                "speed": {
                    "type": "number",
                    "description": "fraction of max linear speed, default 0.5",
                },
                "snap_drift_threshold_m": {
                    "type": "number",
                    "description": "Per-cell drift threshold above which the bridge auto-snaps. Default 0.10 m.",
                },
                "per_cell_timeout_s": {
                    "type": "number",
                    "description": "Bridge-side timeout per cell move. Default 30 s.",
                },
            },
            "required": ["memory_id"],
        },
        impl=_impl_replay_recalled_path,
        tags=["motion", "maze", "batch", "memory"],
    ),
]
