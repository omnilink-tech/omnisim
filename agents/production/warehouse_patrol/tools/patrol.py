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

"""Warehouse Patrol — husky-control + sweep-comparison tools for
`warehouse_patrol.wbt`. Sister of `warehouse_picker/tools/picker.py`
with a different mission shape:

  - Picker: drive to ONE pallet, identify, push to dock, done.
  - Patrol: walk a sector covering N crates, identify each, persist
    a manifest, then on the NEXT sweep recall the prior manifest and
    narrate "what moved since last time?".

Reused from picker (verbatim — same drive controller, same eye sidecar):
  - drive_to_waypoint, get_state, get_capabilities, read_mission_brief
  - scan_for_tag, read_camera (perception-as-tool + fallback)
  - stop_husky

Patrol-specific (added at the bottom of this file):
  - sweep_summary    — walk the sector waypoints, scan_for_tag at each,
                       compose + persist a manifest via save_local_memory
  - recall_last_sweep — pull the prior sweep's manifest from local memory
                       (keyed by sector_name)
  - diff_sweeps      — pure-Python comparator: returns moved / new /
                       missing items between two manifests
  - complete_mission — housekeeping (overrides picker's so it carries
                       the patrol-specific narration shape)

Push tool deliberately absent — patrol doesn't manipulate crates. The
husky is a sensor platform here, not an actuator one.
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
]


# ──────────────────────────────────────────────────────────────────────
# Patrol-specific tools: sweep + recall + diff
# ──────────────────────────────────────────────────────────────────────
#
# Sector waypoints are baked into the tool because (a) the layout is
# fixed for warehouse_patrol.wbt, (b) putting them in the tool keeps
# the agent's prompt small (no per-vantage table to render), (c) the
# expected_def field lets each waypoint cross-reference the supervisor's
# /solid endpoint for ground-truth crate position, which gives the
# diff something deterministic to compare against the prior sweep.
#
# If a crate moved between sweeps, the husky still drives to the OLD
# vantage (these waypoints are scan positions, not crate positions),
# the camera may or may not see the moved crate, and /solid?def=...
# returns the crate's NEW world position regardless. The diff then
# compares prior_position vs current_position — that's the moved-by-X-m
# alert the demo exists to surface.

SECTOR_WAYPOINTS = {
    "north": [
        # (label, husky_x, husky_y, look_at_x, look_at_y, expected_def)
        ("red",    -6.0, +2.0, -6.0, +5.0, "CRATE_RED"),
        ("green",  +2.0, +2.0, +2.0, +5.0, "CRATE_GREEN"),
        ("blue",   +9.0, +3.0, +9.0, +6.0, "CRATE_BLUE"),
        ("orange", +5.0, -1.0, +5.0, +2.0, "CRATE_ORANGE"),
    ],
    "south": [
        ("yellow",  -6.0, -2.0, -6.0, -5.0, "CRATE_YELLOW"),
        ("magenta", +2.0, -2.0, +2.0, -5.0, "CRATE_MAGENTA"),
        ("cyan",    +9.0, -3.0, +9.0, -6.0, "CRATE_CYAN"),
        ("white",   +5.0, +1.0, +5.0, -2.0, "CRATE_WHITE"),
    ],
}


def _impl_sweep_summary(sector_name: str = "north", **_: Any) -> Dict[str, Any]:
    """Drive the named sector, scan each waypoint, persist a manifest of
    (expected_label, observed_color, actual_position) for every crate
    in the sector. Saves the manifest to local memory under tag
    "patrol_sweep,<sector_name>" so a future sweep's recall_last_sweep
    can fetch it. Returns the manifest in-line so the agent can
    immediately diff against a prior sweep without a second tool call.

    Auto-saves to local memory because that's the always-correct next
    action — every sweep should be persisted, no exceptions. Splitting
    into "compute" + "save" tools would just give the agent a footgun.
    """
    sector = (sector_name or "").strip().lower()
    waypoints = SECTOR_WAYPOINTS.get(sector)
    if not waypoints:
        return {
            "error": f"unknown sector: {sector_name!r}",
            "known_sectors": sorted(SECTOR_WAYPOINTS.keys()),
        }

    observations: list = []
    for label, hx, hy, lx, ly, expected_def in waypoints:
        # Drive to scan vantage
        drive_result = bridge_post("action", {
            "action": "drive_to_waypoint",
            "x": float(hx), "y": float(hy),
            "speed": 0.5,
            "arrival_tolerance_m": 0.6,
            "look_at": [float(lx), float(ly)],
            "wait": True,
        })
        # Scan for tag (perception-as-tool)
        scan_result = bridge_get("scan")
        front = (scan_result.get("cameras") or {}).get("front") or {}
        observed_color = front.get("marker")
        marker_fraction = front.get("marker_fraction", 0.0)
        # Ground-truth crate position from supervisor
        solid = bridge_get(f"solid?def={expected_def}")
        actual_position = None
        if isinstance(solid, dict) and "world_position" in solid:
            wp = solid["world_position"]
            actual_position = [round(wp[0], 2), round(wp[1], 2), round(wp[2], 2)]

        observations.append({
            "expected_label": label,
            "expected_def": expected_def,
            "observed_color": observed_color,
            "marker_fraction": marker_fraction,
            "actual_position": actual_position,
            "husky_pose_at_scan": drive_result.get("final_pose"),
            "drive_done": drive_result.get("done"),
        })

    # Time-stamp from the bridge
    state = bridge_get("state")
    sim_time = state.get("sim_time")

    manifest = {
        "sector": sector,
        "sim_time": sim_time,
        "observations": observations,
    }

    # Auto-persist to local memory. Title carries the sector + sim_time
    # so the agent can read multiple historic sweeps; tags scope
    # search_local_memory queries.
    import json as _json
    from .local_memory import _impl_save_local_memory  # type: ignore
    save_result = _impl_save_local_memory(
        title=f"patrol sweep {sector} t={sim_time:.0f}s" if isinstance(sim_time, (int, float)) else f"patrol sweep {sector}",
        body=_json.dumps(manifest, indent=2),
        tags=["patrol_sweep", sector],
    )

    return {
        "manifest": manifest,
        "persisted": {
            "memory_id": (save_result or {}).get("id"),
            "title": (save_result or {}).get("title"),
            "error": (save_result or {}).get("error"),
        },
        "n_observations": len(observations),
        "hint": (
            "Manifest persisted to local memory. Call diff_sweeps() with "
            "the prior sweep's manifest (from recall_last_sweep) and this "
            "one's `manifest` field to get the change report."
        ),
    }


def _impl_recall_last_sweep(sector_name: str = "north",
                            before_this_sim_time: Any = None,
                            **_: Any) -> Dict[str, Any]:
    """Pull the most recent persisted sweep manifest for the named
    sector. Returns the parsed manifest dict, or {error: ...} if no
    prior sweep was found (first run case — agent should report
    "no prior sweep, baseline established").

    `before_this_sim_time` (optional) — exclude sweeps with sim_time
    >= this value. Use the current sweep's sim_time so recall returns
    the PRIOR one, not the one we just persisted in this turn.
    """
    sector = (sector_name or "").strip().lower()
    if sector not in SECTOR_WAYPOINTS:
        return {
            "error": f"unknown sector: {sector_name!r}",
            "known_sectors": sorted(SECTOR_WAYPOINTS.keys()),
        }

    from .local_memory import _impl_search_local_memory  # type: ignore
    # Search for the most recent patrol_sweep memory tagged with this sector.
    # _impl_search_local_memory returns its result list under "hits", not
    # "matches" — naming was inconsistent in the husky_maze ancestor.
    # Use a high k so we don't truncate before sorting newest-first.
    search = _impl_search_local_memory(
        query=f"patrol sweep manifest sector {sector}",
        tags=["patrol_sweep", sector],
        k=20,
    )
    hits = (search or {}).get("hits") or []
    if not hits:
        return {
            "error": "no prior sweep found",
            "sector": sector,
            "hint": "First run for this sector — establish a baseline by calling sweep_summary now.",
        }

    import json as _json
    threshold = None
    if isinstance(before_this_sim_time, (int, float)):
        threshold = float(before_this_sim_time)

    # Hits carry a `snippet` (truncated body, 400 chars max) and a
    # `path` to the markdown file on disk where the full body lives.
    # Sweep manifests are longer than 400 chars (4 crates × ~150 chars
    # each + headers), so we MUST read from path to round-trip the JSON.
    # The markdown file format is YAML frontmatter + blank line + body.
    hits = sorted(hits, key=lambda h: h.get("updated_at") or "", reverse=True)
    from pathlib import Path as _Path
    for h in hits:
        path_str = h.get("path") or ""
        if not path_str:
            continue
        try:
            full = _Path(path_str).read_text(encoding="utf-8")
        except Exception:
            continue
        # Strip YAML frontmatter if present (---\n...\n---\n\n<body>)
        body = full
        if body.startswith("---\n"):
            close = body.find("\n---\n", 4)
            if close >= 0:
                body = body[close + 5:].lstrip()
        try:
            manifest = _json.loads(body)
        except Exception:
            continue
        if not isinstance(manifest, dict) or manifest.get("sector") != sector:
            continue
        st = manifest.get("sim_time")
        if threshold is not None and isinstance(st, (int, float)) and st >= threshold:
            continue
        return {
            "manifest": manifest,
            "memory_id": h.get("id"),
            "memory_title": h.get("title"),
            "memory_updated_at": h.get("updated_at"),
        }

    return {
        "error": "no prior sweep before threshold",
        "sector": sector,
        "threshold_sim_time": threshold,
    }


def _impl_diff_sweeps(prior: Any = None, current: Any = None,
                      moved_threshold_m: float = 0.30,
                      **_: Any) -> Dict[str, Any]:
    """Pure-Python comparator. Returns a list of {label, status,
    delta_m, prior_position, current_position} for every crate that
    appears in either manifest.

    status values:
      - "moved": same crate appears in both manifests but world position
        differs by more than moved_threshold_m
      - "stationary": same crate, position within threshold
      - "missing": crate present in prior but not current (agent didn't
        observe it this sweep — could be moved out of sector OR scan
        miss)
      - "new": crate present in current but not prior (just added, OR
        the prior manifest excluded it)

    moved_threshold_m default 0.30 m — small enough to catch a real
    operator-bumped crate, large enough to not flag scan-pose jitter
    (the husky's vantage may not be perfectly identical between sweeps,
    so the world_position read can wobble by a few cm).
    """
    if not isinstance(prior, dict) or not isinstance(current, dict):
        return {
            "error": "prior and current must be manifest dicts",
            "got": {"prior_type": type(prior).__name__, "current_type": type(current).__name__},
        }

    def _by_label(manifest: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        out = {}
        for obs in manifest.get("observations") or []:
            label = obs.get("expected_label")
            if isinstance(label, str):
                out[label] = obs
        return out

    prior_by_label = _by_label(prior)
    current_by_label = _by_label(current)
    all_labels = sorted(set(prior_by_label.keys()) | set(current_by_label.keys()))

    diffs: list = []
    import math as _m
    for label in all_labels:
        p = prior_by_label.get(label)
        c = current_by_label.get(label)
        if p and not c:
            diffs.append({"label": label, "status": "missing",
                          "prior_position": p.get("actual_position")})
            continue
        if c and not p:
            diffs.append({"label": label, "status": "new",
                          "current_position": c.get("actual_position")})
            continue
        # Both — compare positions
        pp = p.get("actual_position") if p else None
        cp = c.get("actual_position") if c else None
        if not (isinstance(pp, list) and isinstance(cp, list) and len(pp) >= 2 and len(cp) >= 2):
            diffs.append({"label": label, "status": "no_position",
                          "prior": p, "current": c})
            continue
        delta = _m.hypot(cp[0] - pp[0], cp[1] - pp[1])
        if delta > float(moved_threshold_m):
            diffs.append({
                "label": label, "status": "moved",
                "delta_m": round(delta, 2),
                "prior_position": [round(pp[0], 2), round(pp[1], 2)],
                "current_position": [round(cp[0], 2), round(cp[1], 2)],
            })
        else:
            diffs.append({
                "label": label, "status": "stationary",
                "delta_m": round(delta, 2),
                "current_position": [round(cp[0], 2), round(cp[1], 2)],
            })

    moved = [d for d in diffs if d["status"] == "moved"]
    summary = {
        "n_total": len(diffs),
        "n_moved": len(moved),
        "n_missing": len([d for d in diffs if d["status"] == "missing"]),
        "n_new": len([d for d in diffs if d["status"] == "new"]),
        "n_stationary": len([d for d in diffs if d["status"] == "stationary"]),
    }
    return {
        "summary": summary,
        "diffs": diffs,
        "moved_threshold_m": float(moved_threshold_m),
        "headline": (
            f"{summary['n_moved']} crate(s) moved since last sweep"
            if moved else "Nothing moved since last sweep."
        ),
    }


SPECS.extend([
    ToolSpec(
        name="sweep_summary",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Drive the named sector covering all crates assigned to it, "
            "scan each one with the front camera, and persist a per-crate "
            "manifest (label, observed colour, actual world position) to "
            "local memory tagged with the sector. Returns the manifest in-"
            "line so you can immediately call diff_sweeps against a prior "
            "sweep recalled via recall_last_sweep. Sectors: 'north' (red, "
            "green, blue, orange — y > 0) and 'south' (yellow, magenta, "
            "cyan, white — y < 0). Synchronous: each waypoint takes ~30 s "
            "to drive + scan, so a 4-crate sweep is ~2 min wall-clock."
        ),
        parameters={
            "type": "object",
            "properties": {
                "sector_name": {"type": "string", "enum": ["north", "south"],
                                "description": "Which sector to sweep."},
            },
            "required": ["sector_name"],
        },
        impl=_impl_sweep_summary,
        tags=["patrol", "memory"],
    ),
    ToolSpec(
        name="recall_last_sweep",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Fetch the most recent persisted sweep manifest for the named "
            "sector from local memory. Returns {manifest, memory_id, "
            "memory_title, memory_age_s} on hit, or {error: 'no prior "
            "sweep found', hint: '...'} on first-run miss. Pass "
            "before_this_sim_time = current_sweep.sim_time when you've "
            "ALREADY persisted this sweep and want the PRIOR one (otherwise "
            "you'd recall the manifest you just saved). Tag-scoped to "
            "'patrol_sweep,<sector>' so cross-sector queries don't pollute."
        ),
        parameters={
            "type": "object",
            "properties": {
                "sector_name": {"type": "string", "enum": ["north", "south"]},
                "before_this_sim_time": {"type": "number",
                    "description": "Exclude sweeps with sim_time >= this value."},
            },
            "required": ["sector_name"],
        },
        impl=_impl_recall_last_sweep,
        tags=["patrol", "memory"],
    ),
    ToolSpec(
        name="diff_sweeps",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Pure-Python comparator. Pass two sweep manifests (typically "
            "the prior sweep recalled via recall_last_sweep and the current "
            "sweep returned by sweep_summary) and get back a list of "
            "{label, status, delta_m, prior_position, current_position} "
            "for every crate. status is one of moved / stationary / "
            "missing / new. Returns a summary block with headline counts. "
            "moved_threshold_m default 0.30 m — small enough to catch a "
            "real bump, large enough to ignore scan-pose jitter."
        ),
        parameters={
            "type": "object",
            "properties": {
                "prior": {"type": "object",
                    "description": "Prior sweep's manifest dict (from recall_last_sweep.manifest)."},
                "current": {"type": "object",
                    "description": "Current sweep's manifest dict (from sweep_summary.manifest)."},
                "moved_threshold_m": {"type": "number", "default": 0.30,
                    "description": "Position delta in metres above which a crate is flagged 'moved'."},
            },
            "required": ["prior", "current"],
        },
        impl=_impl_diff_sweeps,
        tags=["patrol", "memory"],
    ),
])
