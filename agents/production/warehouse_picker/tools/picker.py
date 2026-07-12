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

"""Warehouse Picker — husky-control tools for continuous-space waypoint
navigation in `warehouse_logistics.wbt`.

Sister of `husky_maze/tools/husky.py` but with a smaller, warehouse-
shaped surface:

  - drive_to_waypoint  (continuous-space x, y) — primary motion primitive
  - get_state, get_capabilities, read_mission_brief — discovery
  - read_camera        — front-camera frame for tag-colour identification
  - stop_husky, complete_mission — housekeeping

Maze-specific tools (try_get_known_map, goto_cell, walk_one_cell,
follow_corridor, auto_explore, scan_surroundings, …) are deliberately
absent — they would mislead the agent on a continuous-space world.
"""

from __future__ import annotations

from typing import Any, Dict

from ._base import (
    ALWAYS, GUARDED, SAFE, ToolSpec, bridge_get, bridge_post,
)


# ----- Tool implementations ----------------------------------------------

def _impl_get_capabilities(**_: Any) -> Dict[str, Any]:
    return bridge_get("capabilities")


def _impl_get_state(**_: Any) -> Dict[str, Any]:
    """Live snapshot of the husky: x, y, yaw, mode, fault. The
    `current_cell` field is computed against the maze constants and is
    meaningless on the warehouse — use raw (x, y) instead."""
    return bridge_get("state")


def _impl_read_mission_brief(**_: Any) -> Dict[str, Any]:
    """Operator's free-form mission brief, read from WorldInfo.info."""
    return bridge_get("mission")


def _impl_complete_mission(rationale: str = "", claimed_cells: Any = None,
                           **_: Any) -> Dict[str, Any]:
    if not rationale or not rationale.strip():
        return {"error": "rationale is required: a one-sentence justification"}
    payload: Dict[str, Any] = {
        "action": "complete_mission",
        "rationale": rationale.strip(),
    }
    if claimed_cells:
        payload["claimed_cells"] = claimed_cells
    return bridge_post("action", payload)


def _impl_drive_to_waypoint(x: float = 0.0, y: float = 0.0, speed: float = 0.5,
                            arrival_tolerance_m: float = 1.00,
                            look_at: Any = None,
                            **_: Any) -> Dict[str, Any]:
    """Drive the husky to a continuous-space (x, y) waypoint. Bridge
    handles the two-phase turn-then-drive controller, the deceleration
    window, and the settle gate. Synchronous: returns when the husky
    has arrived (within `arrival_tolerance_m` of the target) or
    faulted. Default tolerance 1.0 m is sized for warehouse waypoints
    where 'in the area' is enough; pass a tighter value for precision
    docking. Pass `look_at: [lx, ly]` to spin the husky on arrival so
    the front camera frames that point — essential before read_camera
    when the agent's planned approach heading does not match the
    direction it wants to look. Returns: {x, y, done, fault,
    final_pose, distance_remaining_m, arrival_tolerance_m, looked_at}."""
    payload: Dict[str, Any] = {
        "action": "drive_to_waypoint",
        "x": float(x),
        "y": float(y),
        "speed": float(speed),
        "arrival_tolerance_m": float(arrival_tolerance_m),
        "wait": True,
    }
    if look_at is not None:
        try:
            payload["look_at"] = [float(look_at[0]), float(look_at[1])]
        except (TypeError, ValueError, IndexError):
            return {"error": f"look_at must be [x, y] of two numbers, got {look_at!r}"}
    return bridge_post("action", payload)


def _impl_stop_husky(**_: Any) -> Dict[str, Any]:
    return bridge_post("action", {"action": "stop"})


def _impl_read_camera(**_: Any) -> Dict[str, Any]:
    """Snapshot the husky's front camera. Returns base64 PNG (320x240)
    plus the husky's tracking pose. The picker uses this to identify
    which coloured tag is on the pallet directly ahead — drive to a
    vantage ~2 m south of the pallet first, otherwise the tag fills the
    frame too completely to read confidently."""
    return bridge_get("camera")


def _impl_scan_for_tag(**_: Any) -> Dict[str, Any]:
    """Run the husky_eye's pure-Python frame analyser on the current
    front-camera view and return a small structured digest:
    {tag_color, marker_pixels, marker_fraction, marker_centroid,
    color_fractions, tracking_pose, sim_time}. The agent never
    sees pixels — the eye sidecar classifies the dominant
    coloured-cube tag (red/green/blue/yellow/magenta/cyan) by
    counting pixels that match each colour's RGB signature.

    Use this INSTEAD of read_camera for routine tag identification.
    A 320x240 PNG attached as an inline image_url part costs ~12k
    Gemini input tokens; this tool returns ~80 tokens of structured
    JSON. Same identification, ~150x cheaper per query.

    Same vantage discipline as read_camera applies: the agent must
    have driven to a position ~4 m from the pallet with look_at
    pointing at the pallet's coords before calling this — otherwise
    the camera doesn't see the tag and the response will have
    tag_color=null."""
    raw = bridge_get("scan")
    if not isinstance(raw, dict) or "cameras" not in raw:
        return {"error": "scan endpoint returned unexpected shape", "raw": raw}
    front = (raw.get("cameras") or {}).get("front") or {}
    return {
        "tag_color": front.get("marker"),
        "marker_pixels": front.get("marker_pixels"),
        "marker_fraction": front.get("marker_fraction"),
        "marker_centroid": front.get("marker_centroid"),
        "color_fractions": front.get("color_fractions") or {},
        "wall_close": front.get("wall_close"),
        "tracking_pose": raw.get("tracking_pose"),
        "sim_time": raw.get("sim_time"),
        "hint": (
            "tag_color is the dominant tag colour visible in the front "
            "camera, or null if none qualifies. color_fractions shows the "
            "non-zero per-colour pixel ratios so you can see ambiguous "
            "frames (e.g. two tags both partially visible)."
        ),
    }


def _impl_push_pallet_to(source_x: float = 0.0, source_y: float = 0.0,
                         target_x: float = 0.0, target_y: float = 0.0,
                         **_: Any) -> Dict[str, Any]:
    """Push a pallet from (source_x, source_y) to (target_x, target_y)
    using up to two axis-aligned push phases. Skid-steer pushing is
    only reliable straight-on, so a diagonal push is decomposed into
    a longer-axis leg followed by a shorter-axis leg.

    Each phase: position the husky on the side OPPOSITE the push
    direction (with `look_at` aimed past the pallet so the husky
    arrives heading toward the target), then drive forward through
    the pallet's bounding box until the husky overshoots the target
    by half a pallet length — that lands the pallet on the target.

    Pallet dimensions: 1.20 (x) x 0.80 (y) x 0.14 (z). Husky is
    ~0.99 long, so APPROACH_GAP = pallet_half + husky_half + clearance
    keeps the husky behind the pallet at start. PUSH_OVERRUN ≈
    pallet_half so the pallet ends near (target_x, target_y).
    """
    import math
    APPROACH_GAP = 1.6      # husky stand-off behind pallet at phase start
    PUSH_OVERRUN = 0.3      # husky drives this far past target — pallet trails
    # Warehouse interior bounds — keep approach poses inside so the
    # husky never tries to drive into a wall stub. Walls at x=+/-15
    # and y=+/-9; leave 1.5 m clearance.
    SAFE_X_MIN, SAFE_X_MAX = -13.5, 13.5
    SAFE_Y_MIN, SAFE_Y_MAX = -7.5, 7.5

    def _clamp_safe(p):
        return (max(SAFE_X_MIN, min(SAFE_X_MAX, p[0])),
                max(SAFE_Y_MIN, min(SAFE_Y_MAX, p[1])))

    def _push_segment(sx: float, sy: float, tx: float, ty: float,
                      seg_label: str) -> Dict[str, Any]:
        dx, dy = tx - sx, ty - sy
        d = math.hypot(dx, dy)
        if d < 0.05:
            return {"label": seg_label, "skipped": True, "reason": "source==target"}
        ux, uy = dx / d, dy / d
        approach = _clamp_safe((sx - ux * APPROACH_GAP, sy - uy * APPROACH_GAP))
        push_end = _clamp_safe((tx + ux * PUSH_OVERRUN, ty + uy * PUSH_OVERRUN))
        # Look past the source so arrival heading points at the pallet.
        look = (sx + ux * 5.0, sy + uy * 5.0)

        # Side-detour staging. The approach pose is on the OPPOSITE side
        # of the pallet from the push direction, but the husky's current
        # position can be on the SAME side — in which case driving
        # straight to the approach pose runs the husky THROUGH the
        # pallet, shoving it in the WRONG direction. This bug bit a
        # full E2E run: husky was at (3, 1) after camera scan, drove
        # straight north to approach (3, 6.6), pushed the green pallet
        # from (3, 5) to (3, 7.3). Fix: route via a side-detour at the
        # approach Y level, offset 2.5 m perpendicular to the push axis,
        # on whichever side the husky is currently closer to.
        try:
            husky_st = bridge_get("state")
            hx, hy = float(husky_st.get("x", 0.0)), float(husky_st.get("y", 0.0))
        except Exception:
            hx = hy = 0.0
        # Perpendicular to push direction: rotate (ux, uy) by 90 deg.
        px, py = -uy, ux  # left-perpendicular
        # Pick the side closer to the husky (dot product with husky offset).
        dot = (hx - sx) * px + (hy - sy) * py
        if dot < 0:
            px, py = -px, -py  # use the other perpendicular
        SIDE_OFFSET = 2.5
        detour = _clamp_safe((approach[0] + px * SIDE_OFFSET,
                              approach[1] + py * SIDE_OFFSET))
        # Skip the detour if the husky is already on the right side
        # (i.e., on the OPPOSITE side of the pallet from the push
        # direction). Husky on opposite side: dot of (husky - source)
        # with the negative push direction (-ux, -uy) should be > 0.
        opposite_dot = (hx - sx) * (-ux) + (hy - sy) * (-uy)
        skip_detour = opposite_dot > 0.5
        if not skip_detour:
            r0 = bridge_post("action", {
                "action": "drive_to_waypoint",
                "x": detour[0], "y": detour[1],
                "speed": 0.5,
                "arrival_tolerance_m": 0.6,
                "wait": True,
            })
            if not r0.get("done"):
                return {"label": seg_label, "phase": "detour", "result": r0,
                        "error": "side-detour failed"}

        r1 = bridge_post("action", {
            "action": "drive_to_waypoint",
            "x": approach[0], "y": approach[1],
            "speed": 0.5,
            "arrival_tolerance_m": 0.4,
            "look_at": [look[0], look[1]],
            "wait": True,
        })
        if not r1.get("done"):
            return {"label": seg_label, "phase": "approach", "result": r1,
                    "error": "approach failed"}
        r2 = bridge_post("action", {
            "action": "drive_to_waypoint",
            "x": push_end[0], "y": push_end[1],
            "speed": 0.3,           # slow + steady for more reliable contact
            "arrival_tolerance_m": 0.6,
            "wait": True,
        })
        return {
            "label": seg_label,
            "approach_pose": r1.get("final_pose"),
            "push_end_pose": r2.get("final_pose"),
            "approach_target": list(approach),
            "push_end_target": list(push_end),
            "done": bool(r2.get("done")),
            "fault": r2.get("fault"),
        }

    segments = []
    cur_x, cur_y = float(source_x), float(source_y)

    # Resolve which DEF'd Solid this push is targeting so we can
    # ground-truth its position via /solid between segments and
    # detect when contact has been lost mid-push.
    WAREHOUSE_PALLETS = {
        (-3, 5): "LOAD_RED",   (3, 5): "LOAD_GREEN",   (9, 5): "LOAD_BLUE",
        (-3, -5): "LOAD_YELLOW", (3, -5): "LOAD_MAGENTA", (9, -5): "LOAD_CYAN",
    }
    sx_r0, sy_r0 = round(float(source_x)), round(float(source_y))
    pallet_def = (str(_.get("pallet_def") or _.get("source_def") or "") or "").strip()
    if not pallet_def:
        pallet_def = WAREHOUSE_PALLETS.get((sx_r0, sy_r0), "")

    def _refresh_pallet_pose():
        if not pallet_def:
            return None
        try:
            r = bridge_get(f"solid?def={pallet_def}")
            if isinstance(r, dict) and "world_position" in r:
                wp = r["world_position"]
                return float(wp[0]), float(wp[1])
        except Exception:
            pass
        return None

    # Retry loop: each iteration reads the pallet's actual position,
    # picks an axis-aligned push toward (target_x, target_y), and
    # executes one segment. Loop until the pallet is within tolerance
    # of the target OR we hit the retry cap. The "shorter axis first"
    # heuristic is captured by the per-iteration choice — once the
    # pallet is in the y=0 corridor (perpendicular axis done), the
    # remaining error is along the corridor (long axis), and the
    # next iteration handles it.
    #
    # Why retry: a single straight-line push can lose contact early
    # if the husky and pallet drift laterally over a long traversal.
    # The pallet typically ends up still aligned with the push axis
    # but several metres short of the target. Re-reading ground truth
    # and re-pushing closes the gap iteration by iteration.
    # Dock zone is 3 m x 4 m centred at (-11, 0); the pallet is 1.2 m
    # x 0.8 m. "On the dock" means the pallet's centroid lands within
    # the dock interior with a small margin — anywhere within ~2 m of
    # (-11, 0) puts the pallet visibly on the yellow plate. Tighter
    # delivery is possible but burns more time on diminishing returns.
    DELIVERY_TOL = 2.0
    AXIS_DONE_TOL = 0.4
    MAX_RETRIES = 3          # cap so we don't shove a stuck pallet forever

    def _err():
        actual = _refresh_pallet_pose()
        if actual is None:
            return None, None
        return actual, ((actual[0] - target_x) ** 2 + (actual[1] - target_y) ** 2) ** 0.5

    for retry in range(MAX_RETRIES):
        actual, err = _err()
        if actual is None:
            # Can't measure — assume the in-memory cur_x/cur_y is right.
            actual = (cur_x, cur_y)
            err = ((cur_x - target_x) ** 2 + (cur_y - target_y) ** 2) ** 0.5
        cur_x, cur_y = actual
        if err < DELIVERY_TOL:
            break
        dx, dy = target_x - cur_x, target_y - cur_y
        # Pick an axis: the larger remaining absolute delta. But
        # when source row (y=+/-5) and target row (y~0) differ
        # AND |dy| is still substantial, prefer Y first to clear
        # the row before any X push.
        in_row = abs(cur_y) > 2.0  # heuristic: still in a pallet row
        prefer_y = in_row and abs(dy) > AXIS_DONE_TOL
        if prefer_y or (abs(dy) > abs(dx) and abs(dy) > AXIS_DONE_TOL):
            label = f"y_push_{retry}"
            seg = _push_segment(cur_x, cur_y, cur_x, target_y, label)
        elif abs(dx) > AXIS_DONE_TOL:
            label = f"x_push_{retry}"
            seg = _push_segment(cur_x, cur_y, target_x, cur_y, label)
        else:
            # Both axes within their done-tolerance but error > delivery
            # tolerance — fine-tune the larger remaining axis.
            label = f"final_{retry}"
            if abs(dx) > abs(dy):
                seg = _push_segment(cur_x, cur_y, target_x, cur_y, label)
            else:
                seg = _push_segment(cur_x, cur_y, cur_x, target_y, label)
        segments.append(seg)
        if seg.get("error"):
            return {"status": "fault", "segments": segments,
                    "current_pose_estimate": [cur_x, cur_y]}
        actual_after = _refresh_pallet_pose()
        if actual_after:
            seg["pallet_actual_after"] = [round(actual_after[0], 2),
                                           round(actual_after[1], 2)]
            # If the pallet didn't move at all, give up — the husky is
            # probably wedged against the dock wall stub or a neighbour.
            moved = ((actual_after[0] - cur_x) ** 2 + (actual_after[1] - cur_y) ** 2) ** 0.5
            if moved < 0.3:
                seg["stalled"] = True
                break

    final_state = bridge_get("state")
    # Ground-truth verify the pallet's final position via /solid. Each
    # push segment trusts the husky's waypoint arrival, but the pallet
    # can collide with neighbours, slip off the bumper, or get stuck
    # against a wall — none of which fault the husky's controller.
    # `pallet_def` was resolved at the top of this function.
    delivered = None
    if pallet_def:
        solid = bridge_get(f"solid?def={pallet_def}")
        if isinstance(solid, dict) and "world_position" in solid:
            wp = solid["world_position"]
            import math as _m
            err = _m.hypot(wp[0] - float(target_x), wp[1] - float(target_y))
            delivered = {
                "pallet_def": pallet_def,
                "actual_position": [round(wp[0], 2), round(wp[1], 2), round(wp[2], 2)],
                "target": [float(target_x), float(target_y)],
                "delivery_error_m": round(err, 2),
                "delivered": err < 2.0,
            }
        else:
            delivered = {"pallet_def": pallet_def, "error": "could not read pallet pose",
                         "raw": solid}

    payload = {
        "status": "ok" if (not delivered or delivered.get("delivered")) else "off_target",
        "source": [source_x, source_y],
        "target": [target_x, target_y],
        "segments": segments,
        "husky_final_pose": {
            "x": final_state.get("x"),
            "y": final_state.get("y"),
            "yaw": final_state.get("yaw"),
        },
        "delivery": delivered,
    }
    if delivered and not delivered.get("delivered"):
        payload["error"] = (
            f"pallet ended at {delivered.get('actual_position')} which is "
            f"{delivered.get('delivery_error_m')} m from target "
            f"{[float(target_x), float(target_y)]} — did NOT reach the dock. "
            "Likely caused by a collision with another pallet, the husky "
            "losing contact mid-push, or a wall-stub blocking the approach. "
            "Do NOT claim mission complete; report this honestly to the "
            "Foreman or retry with a different push order."
        )
    return payload


# ----- SPECS --------------------------------------------------------------

SPECS = [
    ToolSpec(
        name="get_capabilities",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Read the bridge capabilities: robot kinematics, max speeds, "
            "world title, and (on maze worlds) the maze constants. On "
            "warehouse_logistics.wbt the maze fields are present but "
            "meaningless — ignore current_cell / unvisited_neighbours."
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
            "Live snapshot of the husky's pose and mode. Use raw "
            "(x, y, yaw) for warehouse navigation — current_cell is "
            "maze-specific and not meaningful here."
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
            "Read the operator's mission brief from WorldInfo.info — "
            "tells you which colour pallet to fetch and where to deliver "
            "it. Always call this first."
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
            "Mark the mission as complete. Pass a one-sentence "
            "`rationale` summarising what you did (which colour pallet, "
            "delivered to which dock zone). Operator audits the log."
        ),
        parameters={
            "type": "object",
            "properties": {
                "rationale": {"type": "string"},
                "claimed_cells": {
                    "type": "array",
                    "items": {"type": "array", "items": {"type": "integer"}},
                    "description": "Optional list of [col, row] pairs visited (legacy maze concept; pass [] on warehouse).",
                },
            },
            "required": ["rationale"],
        },
        impl=_impl_complete_mission,
        tags=["mission"],
    ),
    ToolSpec(
        name="drive_to_waypoint",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Drive the husky to a continuous-space (x, y) waypoint. "
            "Synchronous — blocks until the husky reaches the target "
            "(within arrival_tolerance_m, default 1.0 m) or faults. "
            "PRIMARY MOTION PRIMITIVE for the warehouse. Pallet "
            "coordinates: red (-3, 5), green (3, 5), blue (9, 5), "
            "yellow (-3, -5), magenta (3, -5), cyan (9, -5). Loading "
            "dock centre is (-11, 0). When approaching a pallet for "
            "camera tag identification, drive to a vantage ~4 m from "
            "the pallet (e.g. (3, 1) for the green tag at (3, 5)) AND "
            "pass look_at: [pallet_x, pallet_y] so the husky's front "
            "camera frames the tag on the pallet's top. Closer than "
            "~3 m the pallet face occludes the tag; farther than ~6 m "
            "the tag becomes too small to read confidently. Without "
            "look_at the husky parks facing whichever direction it was "
            "last driving — usually wrong for tag reading."
        ),
        parameters={
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "World x-coordinate target (m)."},
                "y": {"type": "number", "description": "World y-coordinate target (m)."},
                "speed": {"type": "number", "description": "Fraction of max linear speed, default 0.5."},
                "arrival_tolerance_m": {
                    "type": "number",
                    "description": "Stop when within this many metres of (x, y). Default 1.0 m.",
                },
                "look_at": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 2, "maxItems": 2,
                    "description": (
                        "Optional [lx, ly] world point to face after "
                        "arrival. The husky spins in place to point its "
                        "front camera at this point. Use the pallet's "
                        "coordinates here when read_camera will follow."
                    ),
                },
            },
            "required": ["x", "y"],
        },
        impl=_impl_drive_to_waypoint,
        tags=["motion", "warehouse"],
    ),
    ToolSpec(
        name="stop_husky",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Emergency halt: zero both wheels, mode -> stopped. ALWAYS "
            "AVAILABLE. Idempotent. Call immediately on fault or any "
            "unsafe state."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_stop_husky,
        tags=["safety"],
    ),
    ToolSpec(
        name="scan_for_tag",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "PREFERRED tag-identification tool. Returns a structured "
            "{tag_color, marker_pixels, marker_fraction, marker_centroid, "
            "color_fractions, ...} digest from the husky_eye sidecar's "
            "pure-Python frame analyser — never sends pixels to you. "
            "tag_color is the dominant cube colour visible "
            "(red/green/blue/yellow/magenta/cyan) or null when no tag "
            "is in view. Same vantage discipline as read_camera: drive "
            "to ~4 m from the pallet with look_at pointing at the tag "
            "first. Use this for routine identification — it's about "
            "150x cheaper per query than read_camera (80 tokens vs "
            "~12k for an inline PNG)."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_scan_for_tag,
        tags=["vision"],
    ),
    ToolSpec(
        name="read_camera",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "FALLBACK only. Snapshot the husky's front camera (320x240 "
            "PNG, base64) when scan_for_tag returns ambiguous results "
            "(e.g. tag_color=null but you expect a tag, or two colours "
            "with similar fractions). The image then lets you visually "
            "disambiguate. Costly (~12k input tokens per snap) so do "
            "not call this if scan_for_tag already gave you a confident "
            "tag_color."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_read_camera,
        tags=["vision"],
    ),
    ToolSpec(
        name="push_pallet_to",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Physically push a pallet from (source_x, source_y) to "
            "(target_x, target_y) using up to two axis-aligned shove "
            "phases. The husky drives behind the pallet, contacts it "
            "with its bumper, and translates it across the floor — "
            "skid-steer pushing only works straight-on, so a diagonal "
            "delivery is decomposed into a longer-axis push followed "
            "by a shorter-axis push. Synchronous: blocks until both "
            "segments complete or fault. Pallets weigh 15 kg each; "
            "the husky pushes at 0.4 m/s during contact. Returns "
            "{status, source, target, segments[], husky_final_pose}. "
            "Use this AFTER read_camera has identified the target "
            "pallet — pass that pallet's known coordinates as source "
            "(red (-3, 5), green (3, 5), blue (9, 5), yellow (-3, -5), "
            "magenta (3, -5), cyan (9, -5)) and the dock centre "
            "(-11, 0) as target."
        ),
        parameters={
            "type": "object",
            "properties": {
                "source_x": {"type": "number", "description": "Pallet's current x in metres."},
                "source_y": {"type": "number", "description": "Pallet's current y in metres."},
                "target_x": {"type": "number", "description": "Where the pallet should end up (x, m)."},
                "target_y": {"type": "number", "description": "Where the pallet should end up (y, m)."},
            },
            "required": ["source_x", "source_y", "target_x", "target_y"],
        },
        impl=_impl_push_pallet_to,
        tags=["motion", "warehouse", "manipulation"],
    ),
]
