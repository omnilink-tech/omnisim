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

"""Drone-control tools — thin HTTP proxies to the mavic_omnilink_bridge.

Every tool here forwards to the bridge over HTTP and returns the bridge
response unchanged (with a defensive wrapper for transport failures).
The agent never imports the OmniSim `controller` module.

Tool surface intentionally mirrors `husky_maze/tools/husky.py`'s shape:
- one tool per bridge action, plus the SAFE read-only family (state,
  capabilities, mission_brief, scan, camera, solid).
- GUARDED tier on anything that moves a motor or teleports the drone.
- All scan/perception lives behind `scan_for_markers` (perception-as-tool
  per AGENT_PATTERNS.md #1) — `read_camera` is the explicit fallback for
  ambiguous frames.
"""

from __future__ import annotations

from typing import Any, Dict, List

from ._base import (
    ALWAYS, GUARDED, SAFE, ToolSpec, bridge_get, bridge_post,
)


# ----- SAFE: pure reads ----------------------------------------------------

def _impl_get_capabilities(**_: Any) -> Dict[str, Any]:
    return bridge_get("capabilities")


def _impl_get_state(**_: Any) -> Dict[str, Any]:
    return bridge_get("state")


def _impl_read_mission_brief(**_: Any) -> Dict[str, Any]:
    return bridge_get("mission")


def _impl_scan_for_markers(**_: Any) -> Dict[str, Any]:
    """Pure-Python perception classifier on the latest camera frame, with
    image-space centroids projected to world (x, y) coordinates using the
    drone pose + gimbal pitch + camera FoV. Returns
    {markers: [{color, world_x, world_y, fraction, distance_m, ...}],
     frame_summary, pose, hint}. The agent NEVER sees pixels via this
    path — the structured digest is ~150x cheaper than a raw camera frame.
    """
    return bridge_get("scan")


def _impl_read_camera(**_: Any) -> Dict[str, Any]:
    """Raw camera frame (400x240 by default) as base64 PNG. FALLBACK ONLY.
    Use scan_for_markers first; only escalate to read_camera when the
    structured digest is genuinely ambiguous (zero detections in an area
    you expected markers, or you need to verify a colour the classifier
    might be tagging differently from how the operator described it)."""
    return bridge_get("camera")


def _impl_check_marker_position(def_name: str = "", **_: Any) -> Dict[str, Any]:
    """Ground-truth pose lookup of any DEF'd Solid in the world via the
    bridge's /solid endpoint. The advertised marker DEFs are listed in
    capabilities.ground_truth_def_names (e.g. MARKER_RED_1).

    USE THIS as a sanity check on a scan_for_markers detection — does the
    projected world position match the marker's actual world position to
    within a few meters? Don't use it as the primary source of truth for
    the survey result; the operator wants the agent's PERCEPTION-DERIVED
    count + positions, not a list of DEFs read out of the world file."""
    if not def_name:
        return {"error": "def_name required (e.g. MARKER_RED_1)"}
    return bridge_get(f"solid?def={def_name}")


# ----- GUARDED: motion + gimbal + admin -----------------------------------

def _impl_takeoff(altitude: float = 12.0, wait: bool = True,
                   timeout_s: float = 60.0, **_: Any) -> Dict[str, Any]:
    """Spool propellers and climb to `altitude` m. With wait=true (default)
    blocks until the drone is within tolerance of altitude. The bridge
    seeds target_x = target_y = current x/y so the drone holds position
    while climbing."""
    return bridge_post("action", {
        "action": "takeoff",
        "altitude": float(altitude),
        "wait": bool(wait),
        "timeout_s": float(timeout_s),
    })


def _impl_land(wait: bool = True, timeout_s: float = 30.0, **_: Any) -> Dict[str, Any]:
    """Smooth descent to the ground; idle motors below 0.4 m altitude.
    With wait=true (default) blocks until the drone has touched down."""
    return bridge_post("action", {
        "action": "land",
        "wait": bool(wait),
        "timeout_s": float(timeout_s),
    })


def _impl_hover(**_: Any) -> Dict[str, Any]:
    """Pin the drone to its current xy + altitude. Useful between
    decisions when you want to think without drifting."""
    return bridge_post("action", {"action": "hover"})


def _impl_goto_waypoint(x: float = 0.0, y: float = 0.0,
                         altitude: float = 12.0, wait: bool = True,
                         timeout_s: float = 60.0,
                         yaw_to: Any = None, **_: Any) -> Dict[str, Any]:
    """Fly to world (x, y) at the given altitude. With wait=true (default)
    blocks until the drone is within WAYPOINT_REACH_TOL_M (0.6 m) of the
    target. Optional yaw_to=[lx, ly] aims the nose at (lx, ly) on arrival
    — useful to avoid a 180-degree spin between back-to-back waypoints."""
    payload: Dict[str, Any] = {
        "action": "goto_waypoint",
        "x": float(x),
        "y": float(y),
        "altitude": float(altitude),
        "wait": bool(wait),
        "timeout_s": float(timeout_s),
    }
    if isinstance(yaw_to, (list, tuple)) and len(yaw_to) == 2:
        payload["yaw_to"] = [float(yaw_to[0]), float(yaw_to[1])]
    return bridge_post("action", payload)


def _impl_set_gimbal_pitch(pitch_rad: float = 1.5707963, **_: Any) -> Dict[str, Any]:
    """Drive the camera's pitch motor. 0 = forward, pi/2 (~1.5708) = straight
    down. Range per capabilities.camera.pitch_range_rad (typically [-0.5, 1.7]).
    For ground surveys you want pi/2 — that's what the bridge seeds on init,
    and it's what scan_for_markers's world projection assumes. Values far
    from pi/2 introduce projection error proportional to 1/sin(pitch)."""
    return bridge_post("action", {
        "action": "set_gimbal_pitch",
        "pitch_rad": float(pitch_rad),
    })


def _impl_set_yaw(yaw_rad: float = 0.0, **_: Any) -> Dict[str, Any]:
    """Rotate the body to absolute world yaw `yaw_rad` (radians). 0 = +x
    (east), pi/2 = +y (north). The PID will spin the drone in place; for
    waypoint navigation prefer goto_waypoint's yaw_to= parameter."""
    return bridge_post("action", {"action": "set_yaw", "yaw_rad": float(yaw_rad)})


def _impl_stop_drone(**_: Any) -> Dict[str, Any]:
    """Cut all attitude inputs; the drone idles motors and falls. Last
    resort — use only when the drone is in an unrecoverable state. Prefer
    `land` for a controlled descent."""
    return bridge_post("action", {"action": "stop"})


def _impl_reset_drone(**_: Any) -> Dict[str, Any]:
    """Teleport the drone back to its start pose, clear mission_complete,
    clear the mission_log. Use only on operator request."""
    return bridge_post("action", {"action": "reset"})


def _impl_complete_mission(rationale: str = "", payload: Any = None,
                             **_: Any) -> Dict[str, Any]:
    """Mark the mission complete. Bridge logs the claim — operator audits;
    bridge does NOT score whether your claim is correct.

    `rationale` is a one-sentence justification. `payload` is structured
    data the operator can grep — for the count-the-red-markers mission,
    the canonical shape is {red_count: int, red_positions: [[x, y], ...]}."""
    if not rationale or not rationale.strip():
        return {"error": "rationale is required: a one-sentence justification"}
    body: Dict[str, Any] = {
        "action": "complete_mission",
        "rationale": rationale.strip(),
    }
    if payload is not None:
        body["payload"] = payload
    return bridge_post("action", body)


# ----- Spec list ----------------------------------------------------------

SPECS: List[ToolSpec] = [
    ToolSpec(
        name="get_capabilities",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Read the bridge's capability advertisement: robot model, mass, "
            "max horizontal/vertical speed, default takeoff altitude, "
            "camera spec (width, height, FoV, gimbal pitch range, "
            "down_pitch_rad), world_title, mission_brief, "
            "ground_truth_def_names, perception_hint. CALL FIRST on every "
            "session — your survey workflow should read default_takeoff_altitude_m, "
            "camera.down_pitch_rad, and ground_truth_def_names from here, "
            "not hard-code them."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_get_capabilities,
        tags=["sensor", "info"],
    ),
    ToolSpec(
        name="get_state",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Read the drone's live pose + telemetry: x, y, z (world frame), "
            "roll, pitch, yaw, v_xy, v_z, target_altitude_m, "
            "gimbal_pitch_rad, mode (idle | takeoff | hover | goto | land | "
            "landed), fault, sim_time, target, mission_complete. Use to "
            "verify takeoff finished, to see arrival pose after a "
            "goto_waypoint, or to check for a fault before retrying."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_get_state,
        tags=["sensor", "info"],
    ),
    ToolSpec(
        name="read_mission_brief",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Operator's free-form mission brief, read from WorldInfo.info. "
            "Returns {brief, world_title, complete, log, hint}. CALL EARLY "
            "on every session — your workflow choices (which markers to "
            "count, which colour, what to report in complete_mission) "
            "should come from this brief, not from the world title alone."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_read_mission_brief,
        tags=["info", "mission"],
    ),
    ToolSpec(
        name="scan_for_markers",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "PREFERRED perception tool. Pure-Python BGRA classifier on the "
            "latest camera frame, with image-space centroids projected to "
            "world (x, y) coordinates using drone pose + gimbal pitch + "
            "camera FoV. Returns {markers: [{color, world_x, world_y, "
            "fraction, centroid_x_norm, centroid_y_norm, distance_m, "
            "projection_valid}], frame_summary, pose, hint}. ~150x cheaper "
            "than read_camera (a structured digest, not pixels). For the "
            "perimeter-survey mission this is your main sensor — call it "
            "after every goto_waypoint with the gimbal pointed down. "
            "Aggregate detections from multiple waypoints by deduplicating "
            "within ~1.5 m: detections of the same marker from different "
            "vantages cluster within that radius."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_scan_for_markers,
        tags=["sensor", "vision", "perception"],
    ),
    ToolSpec(
        name="read_camera",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "FALLBACK ONLY. Returns the latest camera frame as a base64 "
            "PNG plus pose. Each frame is ~30 KB base64 and decodes to "
            "~70 K input tokens at the model — expensive. Use ONLY when "
            "scan_for_markers returns 0 detections at a vantage where you "
            "expect markers, or when the operator explicitly asks you to "
            "describe what's in the frame. For routine survey work, "
            "scan_for_markers is the right tool."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_read_camera,
        tags=["sensor", "vision", "fallback"],
    ),
    ToolSpec(
        name="check_marker_position",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Ground-truth pose lookup of any DEF'd Solid in the scene "
            "(advertised in capabilities.ground_truth_def_names — e.g. "
            "MARKER_RED_1). Returns {def, world_position: [x, y, z], "
            "world_orientation_3x3_row_major}. USE TO SANITY-CHECK a "
            "scan_for_markers detection's projected world position. NEVER "
            "use as the source of the survey result — the operator wants "
            "your perception-derived count + positions, not a DEF list "
            "lifted from the world file. (If you cheat with this, the "
            "discriminator that justifies an LLM agent disappears.)"
        ),
        parameters={
            "type": "object",
            "properties": {
                "def_name": {
                    "type": "string",
                    "description": "DEF name from capabilities.ground_truth_def_names (e.g. MARKER_RED_1).",
                },
            },
            "required": ["def_name"],
        },
        impl=_impl_check_marker_position,
        tags=["sensor", "ground_truth"],
    ),
    ToolSpec(
        name="takeoff",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Spool propellers and climb to `altitude` m (default 12). With "
            "wait=true (default) blocks until the drone is within "
            "ALTITUDE_REACH_TOL_M (0.4 m) of the target. The bridge seeds "
            "target_x/target_y = current x/y so the drone holds position "
            "while climbing. CALL THIS BEFORE any goto_waypoint — the "
            "controller refuses horizontal motion until the drone is "
            "within 1 m of target_altitude."
        ),
        parameters={
            "type": "object",
            "properties": {
                "altitude": {"type": "number", "description": "metres above ground; default 12"},
                "wait": {"type": "boolean", "description": "block until at altitude; default true"},
                "timeout_s": {"type": "number", "description": "wall-clock seconds to wait; default 60"},
            },
        },
        impl=_impl_takeoff,
        tags=["motion"],
    ),
    ToolSpec(
        name="land",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Controlled descent to the ground; bridge idles motors below "
            "0.4 m altitude. With wait=true (default) blocks until the "
            "drone has touched down. CALL THIS AT THE END of a survey — "
            "leaving the drone hovering wastes battery in a real "
            "deployment, and in the sim it's the polite way to mark "
            "mission completion."
        ),
        parameters={
            "type": "object",
            "properties": {
                "wait": {"type": "boolean", "description": "block until landed; default true"},
                "timeout_s": {"type": "number", "description": "wall-clock seconds to wait; default 30"},
            },
        },
        impl=_impl_land,
        tags=["motion"],
    ),
    ToolSpec(
        name="hover",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Pin the drone to its current xy + altitude. Useful between "
            "decisions when you want to think without drifting (e.g. "
            "after a scan returned an unexpected result and you want to "
            "consult recall before deciding the next waypoint)."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_hover,
        tags=["motion"],
    ),
    ToolSpec(
        name="goto_waypoint",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Fly to world (x, y) at the given altitude. With wait=true "
            "(default) blocks until within WAYPOINT_REACH_TOL_M (0.6 m). "
            "Optional yaw_to=[lx, ly] aims the nose at (lx, ly) on "
            "arrival — useful to avoid a 180-degree spin between back-"
            "to-back waypoints. Bridge clamps speed and runs a heading-"
            "to-target P controller; the drone takes ~10-30 s per leg "
            "depending on distance. RETURNS the final pose so you can "
            "scan_for_markers from the actual arrival vantage, not the "
            "commanded one."
        ),
        parameters={
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "world x (metres)"},
                "y": {"type": "number", "description": "world y (metres)"},
                "altitude": {"type": "number", "description": "world z (metres); default 12"},
                "wait": {"type": "boolean", "description": "block until arrived; default true"},
                "timeout_s": {"type": "number", "description": "wall-clock seconds; default 60"},
                "yaw_to": {
                    "type": "array",
                    "items": {"type": "number"},
                    "description": "Optional [lx, ly] — aim nose at this world point on arrival.",
                },
            },
            "required": ["x", "y"],
        },
        impl=_impl_goto_waypoint,
        tags=["motion", "survey"],
    ),
    ToolSpec(
        name="set_gimbal_pitch",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Drive the camera pitch motor. 0 = forward (horizontal), "
            "pi/2 (~1.5708) = straight down. Range per "
            "capabilities.camera.pitch_range_rad (typically [-0.5, 1.7]). "
            "For ground surveys you want pi/2 — that's what the bridge "
            "seeds on init, and what scan_for_markers's world projection "
            "assumes. Values far from pi/2 introduce projection error "
            "proportional to 1/sin(pitch). After a non-survey use case "
            "(e.g. inspecting a wall side-on), reset to "
            "capabilities.camera.down_pitch_rad before scanning again."
        ),
        parameters={
            "type": "object",
            "properties": {
                "pitch_rad": {
                    "type": "number",
                    "description": "radians; 0=forward, pi/2=down. Default pi/2.",
                },
            },
            "required": ["pitch_rad"],
        },
        impl=_impl_set_gimbal_pitch,
        tags=["sensor", "gimbal"],
    ),
    ToolSpec(
        name="set_yaw",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Rotate the body to absolute world yaw `yaw_rad`. 0 = +x "
            "(east), pi/2 = +y (north). For waypoint navigation prefer "
            "goto_waypoint's yaw_to=[lx, ly] parameter — that bundles "
            "arrival + facing into one tool call. Use set_yaw on its own "
            "when you want to spin in place without flying anywhere."
        ),
        parameters={
            "type": "object",
            "properties": {
                "yaw_rad": {"type": "number", "description": "absolute world yaw in radians"},
            },
            "required": ["yaw_rad"],
        },
        impl=_impl_set_yaw,
        tags=["motion"],
    ),
    ToolSpec(
        name="stop_drone",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Cut all attitude inputs and idle propellers. The drone will "
            "fall — last-resort emergency stop. ALWAYS AVAILABLE — never "
            "gated. Idempotent. PREFER `land` for a controlled descent; "
            "use `stop_drone` only when the drone is in an unrecoverable "
            "fault state and a hard stop is the safer outcome."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_stop_drone,
        tags=["safety"],
    ),
    ToolSpec(
        name="reset_drone",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Teleport the drone back to its start pose, clear "
            "mission_complete, clear the mission_log. Use only on "
            "operator request — surveys can re-launch by calling takeoff "
            "again from wherever the drone landed."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_reset_drone,
        tags=["admin"],
    ),
    ToolSpec(
        name="complete_mission",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Mark the mission complete. Bridge logs the claim — operator "
            "audits; bridge does NOT score whether your claim is correct. "
            "`rationale` is a one-sentence justification ('Surveyed all 6 "
            "waypoints, detected 3 unique red markers within the 1.5 m "
            "dedup radius'). `payload` is structured data the operator "
            "can grep — for the count-the-red-markers mission, the "
            "canonical shape is {red_count: int, red_positions: [[x, y], "
            "...], all_detections: [{color, world_x, world_y, sightings}, "
            "...]}. STRICT HONESTY: never call this with a count that "
            "exceeds what scan_for_markers actually returned. If you "
            "didn't see 3 reds, don't claim 3 reds — report what you saw, "
            "even if the brief said there should be 3."
        ),
        parameters={
            "type": "object",
            "properties": {
                "rationale": {
                    "type": "string",
                    "description": "One-sentence justification for the claim.",
                },
                "payload": {
                    "type": "object",
                    "description": "Structured data the operator audits. Canonical: {red_count, red_positions: [[x,y]], all_detections: [...]}.",
                },
            },
            "required": ["rationale"],
        },
        impl=_impl_complete_mission,
        tags=["mission"],
    ),
]
