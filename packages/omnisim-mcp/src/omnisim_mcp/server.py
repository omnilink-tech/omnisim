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

"""omnisim-mcp — a Model Context Protocol server over the OmniSim harness.

Why this exists
---------------
OmniSim already ships the thing every *other* simulator only gets through
third-party glue: a first-party, agent-facing HTTP surface for authoring and
debugging worlds (the World Harness, PROTOCOL.md §world_harness). But the agent
ecosystem — Claude Desktop, Cursor, the tool marketplaces — standardized on the
**Model Context Protocol (MCP)**, and until now OmniSim was invisible to it. The
competitors' community servers (`omni-mcp/isaac-sim-mcp`, `kvgork/gazebo-mcp`)
wrap a *non*-agent-native simulator in MCP; this wraps an *already* agent-native
one, so it is a thin, honest adapter rather than a re-plumbing.

This server is a **stateless proxy**: every tool call is one HTTP request to a
running harness (default `http://127.0.0.1:6789`). It holds no simulator state
of its own, which is why it needs no heavy runtime — it is pure stdlib (stdio
JSON-RPC + urllib), matching the harness's own zero-dependency design, so it
runs on a fresh clone with nothing installed.

Transport
---------
MCP stdio: newline-delimited JSON-RPC 2.0 on stdin/stdout, logs on stderr. We
implement the tools-only subset directly (initialize / tools/list / tools/call)
rather than depending on the `mcp` SDK, so there is no install step and no
version-skew risk with the client.

Start it
--------
Point your MCP client at either of:

    omnisim-mcp                     # console entry point (after `pip install -e .`)
    python -m omnisim_mcp           # no install needed, from this package's src/

Claude Desktop / Cursor config (`mcpServers`):

    "omnisim": { "command": "omnisim-mcp",
                 "env": { "OMNISIM_HARNESS_URL": "http://127.0.0.1:6789" } }

The harness itself must be running (`python scripts/harness/omnisim_harness.py`
or `python scripts/dev/omnisim_dev.py harness`). Call the `harness_status` tool
first — it reports whether the harness is reachable and how to start it if not.
"""
from __future__ import annotations

import base64
import json
import os
import sys
import urllib.error
import urllib.request

PROTOCOL_VERSION = "2024-11-05"  # widely supported; we also echo the client's
SERVER_INFO = {"name": "omnisim-mcp", "version": "0.1.0"}
DEFAULT_HARNESS = os.environ.get("OMNISIM_HARNESS_URL", "http://127.0.0.1:6789")
HTTP_TIMEOUT_S = float(os.environ.get("OMNISIM_MCP_TIMEOUT", "60"))


def log(msg: str) -> None:
    print(f"[omnisim-mcp] {msg}", file=sys.stderr, flush=True)


# --------------------------------------------------------------------------- #
# Harness HTTP client (stdlib only)                                            #
# --------------------------------------------------------------------------- #
class HarnessError(Exception):
    """A transport-level failure talking to the harness (not a tool-logic error)."""


def _request(method: str, path: str, body: dict | None = None,
             base: str | None = None):
    """One HTTP call to the harness. Returns (status, headers, raw_bytes)."""
    base = base or DEFAULT_HARNESS  # resolved at call time so tests/env can override
    url = base.rstrip("/") + path
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as resp:
            return resp.status, dict(resp.headers), resp.read()
    except urllib.error.HTTPError as e:
        # The harness returns structured JSON error bodies (diagnostics); pass
        # the body through so the agent sees the real code, not just the status.
        return e.code, dict(e.headers), e.read()
    except urllib.error.URLError as e:
        raise HarnessError(
            f"cannot reach the OmniSim harness at {base} ({e.reason}). "
            f"Start it with `python scripts/harness/omnisim_harness.py` and retry."
        ) from e


def _json_call(method: str, path: str, body: dict | None = None) -> dict:
    """HTTP call whose response is JSON. Returns a dict with the parsed body
    plus an `http_status` field so non-200s (structured diagnostics) surface."""
    status, _headers, raw = _request(method, path, body)
    try:
        parsed = json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        parsed = {"raw": raw.decode("utf-8", "replace")}
    if isinstance(parsed, list):
        parsed = {"items": parsed}
    parsed.setdefault("http_status", status)
    return parsed


# --------------------------------------------------------------------------- #
# Tool implementations. Each returns an MCP `content` list.                    #
# --------------------------------------------------------------------------- #
def _text(obj) -> list:
    return [{"type": "text", "text": obj if isinstance(obj, str)
             else json.dumps(obj, indent=2)}]


def t_harness_status(_args: dict) -> list:
    try:
        status, _h, _raw = _request("GET", "/healthz")
        state = _json_call("GET", "/sim/state")
        return _text({
            "reachable": True,
            "harness_url": DEFAULT_HARNESS,
            "healthz_status": status,
            "sim_state": state,
        })
    except HarnessError as e:
        return _text({
            "reachable": False,
            "harness_url": DEFAULT_HARNESS,
            "hint": str(e),
        })


def t_load_world(args: dict) -> list:
    body = {"path": args["path"]}
    for k in ("wait_s", "with_supervisor", "light", "settle_steps",
              "reset_physics"):
        if k in args:
            body[k] = args[k]
    # The backwards-compatible tool name is now the safe iteration front door:
    # first load and every non-pose edit still become a normal /world/load,
    # while proven pose-only edits avoid a parse/physics rebuild. An explicit
    # force_reload (or bare load without a supervisor) preserves the old reset
    # semantics for controller restarts and other deliberate reloads.
    force_reload = bool(args.get("force_reload")) or args.get("with_supervisor") is False
    endpoint = "/world/load" if force_reload else "/world/sync"
    if force_reload:
        body.pop("settle_steps", None)
        body.pop("reset_physics", None)
    return _text(_json_call("POST", endpoint, body))


def t_get_scene_tree(args: dict) -> list:
    suffix = "?bounds=1" if args.get("bounds") else ""
    return _text(_json_call("GET", "/scene/tree" + suffix))


def t_get_scene_node(args: dict) -> list:
    suffix = "?bounds=1" if args.get("bounds") else ""
    return _text(_json_call("GET", f"/scene/node/{args['def']}" + suffix))


def t_look_at(args: dict) -> list:
    body = {"position": args["position"], "target": args["target"]}
    if "push" in args:
        body["push"] = args["push"]
    return _text(_json_call("POST", "/scene/look_at", body))


def t_screenshot(args: dict) -> list:
    """Render a PNG. With no `path`, returns the image inline (base64) so a
    vision-capable agent can *see* the scene; with `path`, writes it
    server-side and returns the path."""
    body = {}
    if "path" in args:
        body["path"] = args["path"]
    if "quality" in args:
        body["quality"] = args["quality"]
    status, headers, raw = _request("POST", "/world/screenshot", body)
    if status != 200:
        return [{"type": "text",
                 "text": f"screenshot failed (HTTP {status}): "
                         f"{raw.decode('utf-8', 'replace')[:500]}"}]
    if "path" in args:
        return _text({"written": args["path"], "bytes": len(raw)})
    ctype = headers.get("Content-Type", "")
    if raw[:8] == b"\x89PNG\r\n\x1a\n" or "image/png" in ctype:
        return [{"type": "image",
                 "data": base64.b64encode(raw).decode("ascii"),
                 "mimeType": "image/png"}]
    # server wrote to a path and returned JSON instead of bytes
    try:
        return _text(json.loads(raw))
    except json.JSONDecodeError:
        return _text({"bytes": len(raw), "note": "non-PNG response"})


def t_render_stats(_args: dict) -> list:
    return _text(_json_call("GET", "/world/render_stats"))


def t_sim_step(args: dict) -> list:
    body = {"steps": args["steps"]} if "steps" in args else {}
    return _text(_json_call("POST", "/sim/step", body))


def t_sim_reset(_args: dict) -> list:
    return _text(_json_call("POST", "/sim/reset", {}))


def t_get_events(args: dict) -> list:
    q = []
    for k in ("since", "log_since", "limit", "types"):
        if k in args:
            q.append(f"{k}={urllib.request.quote(str(args[k]))}")
    path = "/sim/events" + ("?" + "&".join(q) if q else "")
    return _text(_json_call("GET", path))


def t_list_robots(_args: dict) -> list:
    return _text(_json_call("GET", "/robots"))


def t_get_robot_joints(args: dict) -> list:
    return _text(_json_call("GET", f"/robot/{args['def']}/joints"))


def t_get_contacts(_args: dict) -> list:
    return _text(_json_call("GET", "/sim/contacts"))


def t_get_diagnostics(_args: dict) -> list:
    return _text(_json_call("GET", "/world/diagnostics"))


def t_get_viewpoint(_args: dict) -> list:
    return _text(_json_call("GET", "/scene/viewpoint"))


def t_frame(args: dict) -> list:
    body = {k: v for k, v in args.items()
            if k in ("def", "defs", "target", "radius", "mode", "margin",
                     "aspect", "fov", "push", "subject_relative")}
    return _text(_json_call("POST", "/scene/frame", body))


def t_orbit(args: dict) -> list:
    body = {k: v for k, v in args.items()
            if k in ("azimuth_deg", "elevation_deg", "dolly", "pan", "center",
                     "def", "distance", "push")}
    return _text(_json_call("POST", "/scene/orbit", body))


def t_visible(args: dict) -> list:
    q = []
    for k in ("defs", "all", "limit"):
        if k in args:
            q.append(f"{k}={urllib.request.quote(str(args[k]))}")
    path = "/scene/visible" + ("?" + "&".join(q) if q else "")
    return _text(_json_call("GET", path))


# name -> (handler, description, inputSchema). Mirrors PROTOCOL.md §world_harness
# and AGENTS.md §5 so the tool surface stays honest to the real endpoints.
_VEC3 = {"type": "array", "items": {"type": "number"},
         "minItems": 3, "maxItems": 3}

TOOLS = {
    "harness_status": (
        t_harness_status,
        "Check whether the OmniSim harness is running and what world it holds. "
        "Call this first; it also tells you how to start the harness if it is down.",
        {"type": "object", "properties": {}},
    ),
    "load_world": (
        t_load_world,
        "Default world iteration tool. The first call loads the .wbt; later calls "
        "live-apply proven root-node pose-only edits in one batch and automatically "
        "hot-reload every other edit. Returns mode=live_pose|no_change|full_reload. "
        "Set force_reload=true only when you deliberately need controllers restarted "
        "or a full reparse.",
        {"type": "object",
         "properties": {
             "path": {"type": "string",
                      "description": "repo-relative or absolute path to the .wbt"},
             "wait_s": {"type": "number", "description": "load timeout seconds"},
              "with_supervisor": {"type": "boolean",
                                  "description": "inject the harness supervisor (default true)"},
              "light": {"type": "boolean",
                        "description": "use the low-overhead supervisor mode"},
              "settle_steps": {"type": "integer", "minimum": 0,
                               "description": "steps after a live pose batch (default 1)"},
              "reset_physics": {"type": "boolean",
                                "description": "clear moved-body velocity (default true)"},
              "force_reload": {"type": "boolean",
                               "description": "bypass safe live sync and restart the world"},
         },
         "required": ["path"]},
    ),
    "get_scene_tree": (
        t_get_scene_tree,
        "Flat list of every node in the loaded scene (type, DEF, position, "
        "orientation). Pass bounds=true to also get each node's world-space "
        "bounding box, centre and radius — the numbers you need to aim a camera. "
        "Use to confirm placement before chasing a visual bug.",
        {"type": "object",
         "properties": {"bounds": {
             "type": "boolean",
             "description": "attach world-space geometric bounds per node "
                            "(slower: walks geometry and reads mesh files)"}}},
    ),
    "get_scene_node": (
        t_get_scene_node,
        "Full field dump + contact points for one node by its DEF name. "
        "Pass bounds=true for its world-space bounding box / centre / radius.",
        {"type": "object",
         "properties": {"def": {"type": "string", "description": "the node's DEF name"},
                        "bounds": {"type": "boolean",
                                   "description": "attach world-space bounds"}},
         "required": ["def"]},
    ),
    "get_viewpoint": (
        t_get_viewpoint,
        "Read the live camera: position, orientation, fieldOfView, near/far, "
        "follow settings, plus derived forward/up/right unit vectors and the "
        "resolved horizontal + vertical FOV for the real viewport aspect. "
        "Call this before nudging the camera — every other camera API writes "
        "to a camera you otherwise cannot read.",
        {"type": "object", "properties": {}},
    ),
    "frame": (
        t_frame,
        "Put a subject in frame: computes BOTH the aim and the distance and "
        "pushes the pose to the live Viewpoint. Give it a DEF ({\"def\": "
        "\"HUSKY\"}), several DEFs, or an explicit target+radius. Returns the "
        "chosen pose plus a numeric verification (angular offset vs available "
        "half-FOV) proving the subject is inside the frame. This is the camera "
        "verb to reach for first — prefer it over guessing a look_at position.",
        {"type": "object",
         "properties": {
             "def": {"type": "string", "description": "DEF of the subject node"},
             "defs": {"type": "array", "items": {"type": "string"},
                      "description": "frame several nodes at once (union of bounds)"},
             "target": dict(_VEC3, description="explicit centre [x,y,z] (with radius)"),
             "radius": {"type": "number", "description": "explicit subject radius (m)"},
             "mode": {"type": "string",
                      "description": "hero (default) | top_down | front | back | "
                                     "left | right | top | bottom — the "
                                     "directional ones are subject-relative when "
                                     "a single def is given"},
             "margin": {"type": "number", "description": "framing slack, default 1.3"},
             "aspect": {"type": "number",
                        "description": "viewport aspect override (defaults to the real one)"},
             "push": {"type": "boolean", "description": "push to the live Viewpoint (default true)"},
         }},
    ),
    "orbit": (
        t_orbit,
        "Nudge the camera RELATIVE to its current view: azimuth_deg swings "
        "around, elevation_deg raises/lowers, dolly multiplies the distance "
        "(>1 pulls back), pan [dx,dy] slides in screen space. Orbits the "
        "current look-at point, or an explicit center / def. Use this when the "
        "framing is nearly right and you want a small correction.",
        {"type": "object",
         "properties": {
             "azimuth_deg": {"type": "number", "description": "swing around +Z, degrees"},
             "elevation_deg": {"type": "number", "description": "raise (+) / lower (-), degrees"},
             "dolly": {"type": "number", "description": "distance multiplier, >1 pulls back"},
             "pan": {"type": "array", "items": {"type": "number"},
                     "minItems": 2, "maxItems": 2,
                     "description": "[dx, dy] metres in screen space (right, up)"},
             "center": dict(_VEC3, description="explicit orbit centre"),
             "def": {"type": "string", "description": "orbit around this node's bounds centre"},
             "distance": {"type": "number",
                          "description": "if no center/def: metres ahead to treat as the pivot"},
             "push": {"type": "boolean", "description": "push to the live Viewpoint (default true)"},
         }},
    ),
    "visible": (
        t_visible,
        "What is in frame right now. Per node: inside-the-frustum flag, "
        "screen-space bbox and centroid in pixels, distance, and the angular "
        "offset from the view axis with a hint like 'off-screen: 34 deg to the "
        "left, 12 deg up'. This is the closed-loop feedback signal for aiming — "
        "screenshot, read this, correct with orbit/frame.",
        {"type": "object",
         "properties": {
             "defs": {"type": "string", "description": "comma-separated DEFs to restrict to"},
             "all": {"type": "boolean", "description": "include DEF-less nodes too"},
             "limit": {"type": "integer", "description": "max rows (default 200)"},
         }},
    ),
    "look_at": (
        t_look_at,
        "Aim the live Viewpoint from a camera position at a target point "
        "(computes the axis-angle and pushes it), so the next screenshot uses it.",
        {"type": "object",
         "properties": {
             "position": dict(_VEC3, description="camera position [x,y,z]"),
             "target": dict(_VEC3, description="look-at target [x,y,z]"),
             "push": {"type": "boolean", "description": "push to live Viewpoint (default true)"},
         },
         "required": ["position", "target"]},
    ),
    "screenshot": (
        t_screenshot,
        "Render the current view to PNG. With no `path`, returns the image "
        "inline so you can see it; with `path`, writes it server-side.",
        {"type": "object",
         "properties": {
             "path": {"type": "string", "description": "server-side output path (optional)"},
             "quality": {"type": "integer", "description": "PNG quality hint (optional)"},
         }},
    ),
    "render_stats": (
        t_render_stats,
        "Exposure/brightness stats for the current view (mean_brightness, "
        "saturated_pct, black_pct, warnings) — catch blown-out lighting without eyeballing.",
        {"type": "object", "properties": {}},
    ),
    "sim_step": (
        t_sim_step,
        "Advance the simulation by N basic timesteps (default 1).",
        {"type": "object",
         "properties": {"steps": {"type": "integer", "minimum": 1}}},
    ),
    "sim_reset": (
        t_sim_reset,
        "Reset the world to t=0 without re-parsing it.",
        {"type": "object", "properties": {}},
    ),
    "get_events": (
        t_get_events,
        "Poll the unified runtime event stream (controller.log, contact.*, "
        "joint.limit_hit, grip.*, damage.*, world.warning/error). Two cursors: "
        "`since` (supervisor) and `log_since` (controller log).",
        {"type": "object",
         "properties": {
             "since": {"type": "integer", "description": "supervisor-event cursor"},
             "log_since": {"type": "integer", "description": "controller-log cursor"},
             "limit": {"type": "integer"},
             "types": {"type": "string",
                       "description": "comma-separated filter, e.g. contact.began,joint.limit_hit"},
         }},
    ),
    "list_robots": (
        t_list_robots,
        "Enumerate every Robot in the scene with pose and joint count.",
        {"type": "object", "properties": {}},
    ),
    "get_robot_joints": (
        t_get_robot_joints,
        "Per-joint snapshot for one robot (position, velocity, limits, hit_limit).",
        {"type": "object",
         "properties": {"def": {"type": "string", "description": "the robot's DEF name"}},
         "required": ["def"]},
    ),
    "get_contacts": (
        t_get_contacts,
        "Global contact set: [{a_def, b_def, point}].",
        {"type": "object", "properties": {}},
    ),
    "get_diagnostics": (
        t_get_diagnostics,
        "Re-fetch the structured load diagnostics from the current world load.",
        {"type": "object", "properties": {}},
    ),
}


# --------------------------------------------------------------------------- #
# JSON-RPC / MCP plumbing                                                      #
# --------------------------------------------------------------------------- #
def _tools_list() -> dict:
    return {"tools": [{"name": n, "description": d, "inputSchema": s}
                      for n, (_h, d, s) in TOOLS.items()]}


def _tools_call(params: dict) -> dict:
    name = params.get("name")
    args = params.get("arguments") or {}
    entry = TOOLS.get(name)
    if entry is None:
        return {"content": [{"type": "text", "text": f"unknown tool: {name}"}],
                "isError": True}
    handler = entry[0]
    try:
        return {"content": handler(args)}
    except HarnessError as e:
        return {"content": [{"type": "text", "text": str(e)}], "isError": True}
    except KeyError as e:
        return {"content": [{"type": "text",
                             "text": f"missing required argument: {e}"}],
                "isError": True}
    except Exception as e:  # a tool bug must not kill the server
        log(f"tool {name} raised: {e!r}")
        return {"content": [{"type": "text", "text": f"{type(e).__name__}: {e}"}],
                "isError": True}


def _handle(msg: dict) -> dict | None:
    """Dispatch one JSON-RPC request. Returns a response dict, or None for
    notifications (which must not be answered)."""
    method = msg.get("method")
    mid = msg.get("id")
    is_notification = "id" not in msg

    if method == "initialize":
        client_proto = (msg.get("params") or {}).get("protocolVersion")
        result = {
            "protocolVersion": client_proto or PROTOCOL_VERSION,
            "capabilities": {"tools": {}},
            "serverInfo": SERVER_INFO,
        }
        return {"jsonrpc": "2.0", "id": mid, "result": result}

    if method in ("notifications/initialized", "initialized"):
        return None  # notification, no reply

    if method == "ping":
        return {"jsonrpc": "2.0", "id": mid, "result": {}}

    if method == "tools/list":
        return {"jsonrpc": "2.0", "id": mid, "result": _tools_list()}

    if method == "tools/call":
        return {"jsonrpc": "2.0", "id": mid, "result": _tools_call(msg.get("params") or {})}

    if is_notification:
        return None  # unknown notification: ignore silently per spec
    return {"jsonrpc": "2.0", "id": mid,
            "error": {"code": -32601, "message": f"method not found: {method}"}}


def main(argv=None) -> int:
    log(f"starting; harness = {DEFAULT_HARNESS}. Reading MCP stdio.")
    # Line-delimited JSON on stdin; one JSON response line per request on stdout.
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as e:
            log(f"dropping non-JSON line: {e}")
            continue
        try:
            resp = _handle(msg)
        except Exception as e:  # never let one message kill the loop
            log(f"handler crashed: {e!r}")
            mid = msg.get("id") if isinstance(msg, dict) else None
            resp = {"jsonrpc": "2.0", "id": mid,
                    "error": {"code": -32603, "message": f"internal error: {e}"}}
        if resp is not None:
            sys.stdout.write(json.dumps(resp) + "\n")
            sys.stdout.flush()
    log("stdin closed; exiting.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
