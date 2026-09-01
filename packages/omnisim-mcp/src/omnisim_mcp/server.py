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
running harness (default `http://127.0.0.1:6789`), over one pooled
`http.client` connection (per-request when the harness speaks HTTP/1.0, which
it does today — the pool detects that and degrades automatically). It holds no
simulator state of its own, which is why it needs no heavy runtime — it is
pure stdlib (stdio JSON-RPC + http.client), matching the harness's own
zero-dependency design, so it runs on a fresh clone with nothing installed.

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
import http.client
import json
import os
import sys
import threading
import urllib.parse
import urllib.request

PROTOCOL_VERSION = "2024-11-05"  # widely supported; we also echo the client's
SERVER_INFO = {"name": "omnisim-mcp", "version": "0.1.0"}
DEFAULT_HARNESS = os.environ.get("OMNISIM_HARNESS_URL", "http://127.0.0.1:6789")
# Default sits ABOVE the harness's own SUPERVISOR_RPC_TIMEOUT_S (120 s): if the
# wrapper gave up first, the harness would still faithfully finish the request
# (a load, a long step batch) and the agent's world-model would silently desync
# from the real simulator state. `OMNISIM_MCP_TIMEOUT` is the legacy spelling.
HTTP_TIMEOUT_S = float(os.environ.get(
    "OMNISIM_MCP_TIMEOUT_S", os.environ.get("OMNISIM_MCP_TIMEOUT", "130")))
# Set OMNISIM_MCP_KEEPALIVE=0 to force a fresh TCP connection per request (the
# pre-pooling behaviour) for an A/B.
KEEP_ALIVE = os.environ.get("OMNISIM_MCP_KEEPALIVE", "1") not in ("0", "false", "off")


def log(msg: str) -> None:
    print(f"[omnisim-mcp] {msg}", file=sys.stderr, flush=True)


# --------------------------------------------------------------------------- #
# Harness HTTP client (stdlib only)                                            #
# --------------------------------------------------------------------------- #
class HarnessError(Exception):
    """A transport-level failure talking to the harness (not a tool-logic error)."""


class _Pool:
    """One pooled ``http.client`` connection per harness base URL.

    This replaces ``urllib.request.urlopen``, which opens a NEW TCP connection
    for every tool call and gives the caller no way to reuse one (the mechanism
    behind the ROS 2 client's measured 17,487 ``TIME_WAIT`` sockets — see
    ``packages/omnisim-ros2/.../harness_client.py``, whose proven shape this
    copies). Two rules carried over from there:

    * The retry is deliberately limited to a **reused** connection. A server is
      free to close an idle keep-alive socket at any moment, so a send that
      fails on a connection we did not just open almost certainly never
      arrived — safe to repeat exactly once. A failure on a FRESHLY opened
      connection means the harness is down (raise), and retrying a POST there
      could execute a mutation twice.
    * An HTTP/1.0 peer (``resp.will_close``) closes the socket anyway; the pool
      honours that and degrades to per-request connections automatically. The
      World Harness is such a peer today, so pooling is a free upgrade the
      moment it grows keep-alive, and correct either way.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._conns: dict[str, http.client.HTTPConnection] = {}

    def _drop(self, base: str) -> None:
        conn = self._conns.pop(base, None)
        if conn is not None:
            try:
                conn.close()
            except Exception:  # closing must never raise
                pass

    def _connect(self, base: str,
                 timeout: float) -> tuple[http.client.HTTPConnection, bool]:
        """Return (connection, reused). Opens one if the pool is empty."""
        conn = self._conns.get(base)
        if conn is not None:
            conn.timeout = timeout
            if conn.sock is not None:
                conn.sock.settimeout(timeout)
            return conn, True
        parts = urllib.parse.urlsplit(base)
        https = parts.scheme == "https"
        cls = http.client.HTTPSConnection if https else http.client.HTTPConnection
        conn = cls(parts.hostname or "127.0.0.1",
                   parts.port or (443 if https else 80), timeout=timeout)
        self._conns[base] = conn
        return conn, False

    def request(self, method: str, base: str, path: str, data: bytes | None,
                headers: dict[str, str], timeout: float):
        """One request over the pooled connection, reconnecting once if stale.

        Returns (status, headers_dict, raw_bytes); raises HarnessError only on
        a transport failure. Non-2xx statuses are RETURNED — the harness puts
        structured diagnostics in 4xx/5xx bodies and they must reach the agent.
        """
        last: Exception | None = None
        with self._lock:
            for _ in (0, 1):
                conn, reused = self._connect(base, timeout)
                try:
                    conn.request(method, path, body=data, headers=headers)
                    resp = conn.getresponse()
                    raw = resp.read()
                    status = resp.status
                    resp_headers = dict(resp.getheaders())
                    # Honour the server's intent; an HTTP/1.0 reply or an
                    # explicit `Connection: close` means this socket is done.
                    if resp.will_close or not KEEP_ALIVE:
                        self._drop(base)
                    return status, resp_headers, raw
                except (http.client.HTTPException, OSError) as exc:
                    last = exc
                    self._drop(base)
                    if not reused:
                        break
        raise HarnessError(
            f"cannot reach the OmniSim harness at {base} ({last}). "
            f"Start it with `python scripts/harness/omnisim_harness.py` and retry."
        ) from last


_POOL = _Pool()


def _request(method: str, path: str, body: dict | None = None,
             base: str | None = None):
    """One HTTP call to the harness. Returns (status, headers, raw_bytes)."""
    base = (base or DEFAULT_HARNESS).rstrip("/")  # resolved at call time so tests/env can override
    data = json.dumps(body).encode() if body is not None else None
    headers = {"Accept": "application/json"}
    if data is not None:
        headers["Content-Type"] = "application/json"
    return _POOL.request(method, base, path, data, headers, HTTP_TIMEOUT_S)


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
    # One request: /sim/state alone answers both "is it up?" (any answer at
    # all = reachable) and "what world does it hold?".
    try:
        state = _json_call("GET", "/sim/state")
        return _text({
            "reachable": True,
            "harness_url": DEFAULT_HARNESS,
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
        # The harness answers a server-side write with honestly-measured JSON
        # ({path, bytes, pixels, render, ...}). Pass it through verbatim —
        # this used to fabricate {"written": <echo of the argument>, "bytes":
        # <length of the JSON body>}, a wrong byte count on an unverified path.
        try:
            return _text(json.loads(raw))
        except json.JSONDecodeError:
            return _text({"bytes": len(raw), "note": "non-JSON response"})
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


def t_sim_reset(args: dict) -> list:
    body = {k: args[k] for k in ("restore", "verify", "settle_steps")
            if k in args}
    return _text(_json_call("POST", "/sim/reset", body))


def t_rebuild_physics(args: dict) -> list:
    body = {k: args[k] for k in ("settle_steps",) if k in args}
    return _text(_json_call("POST", "/sim/rebuild_physics", body))


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


def t_get_capabilities(args: dict) -> list:
    suffix = "?probe_step=1" if args.get("probe_step") else ""
    return _text(_json_call("GET", "/capabilities" + suffix))


def t_world_sync(args: dict) -> list:
    body = {k: args[k] for k in ("path", "settle_steps", "reset_physics",
                                 "wait_s", "light") if k in args}
    return _text(_json_call("POST", "/world/sync", body))


def t_scene_spawn(args: dict) -> list:
    body = {k: args[k] for k in ("vrml", "type", "fields", "urdf", "clone",
                                 "def", "name", "translation", "rotation",
                                 "parent", "index", "settle_steps",
                                 "reset_physics") if k in args}
    return _text(_json_call("POST", "/scene/spawn", body))


def t_scene_delete(args: dict) -> list:
    body = {k: args[k] for k in ("def", "defs", "settle_steps") if k in args}
    return _text(_json_call("POST", "/scene/delete", body))


def t_scene_set_pose(args: dict) -> list:
    body = {k: args[k] for k in ("def", "translation", "rotation",
                                 "reset_physics", "settle_steps") if k in args}
    return _text(_json_call("POST", "/scene/set_pose", body))


def t_sim_snapshot(args: dict) -> list:
    return _text(_json_call("POST", "/sim/snapshot", {"name": args["name"]}))


def t_sim_restore(args: dict) -> list:
    body = {"name": args["name"]}
    if "settle_steps" in args:
        body["settle_steps"] = args["settle_steps"]
    return _text(_json_call("POST", "/sim/restore", body))


def t_list_snapshots(_args: dict) -> list:
    return _text(_json_call("GET", "/sim/snapshots"))


def t_get_grips(_args: dict) -> list:
    return _text(_json_call("GET", "/sim/grips"))


def t_robot_devices(args: dict) -> list:
    return _text(_json_call("GET", f"/robot/{args['def']}/devices"))


def t_robot_joints_set(args: dict) -> list:
    body = {"joints": args["joints"]}
    if "settle_steps" in args:
        body["settle_steps"] = args["settle_steps"]
    return _text(_json_call("POST", f"/robot/{args['def']}/joints/set", body))


def t_robot_ik(args: dict) -> list:
    body = {"effector": args["effector"], "targets": args["targets"]}
    for k in ("rotations", "tool_offset", "iterations"):
        if k in args:
            body[k] = args[k]
    return _text(_json_call("POST", f"/robot/{args['def']}/ik", body))


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
        "Reset the world to t=0 AND restore the authored scene, without "
        "re-parsing. Note it also re-pins every motor and restarts no "
        "controller, so a robot commanded once at start-up stops moving for "
        "good — the response's `actuation` block explains. "
        "Pass restore=null for the old clock-only rewind.",
        {"type": "object",
         "properties": {
             "restore": {"description": "state name to restore (default the "
                                        "authored '__init__'; null = clock-only)"},
             "verify": {"type": "boolean",
                        "description": "measure how far the restore landed"},
             "settle_steps": {"type": "integer", "minimum": 0,
                              "description": "steps after the restore"},
         }},
    ),
    "rebuild_physics": (
        t_rebuild_physics,
        "W1.7 (2026-09-01): rebuild the Newton world at the scene's CURRENT "
        "poses in ~0.1-0.3 s, so runtime-spawned nodes gain physics and "
        "deleted ones lose it (the frozen-model physics_warning on "
        "scene_spawn/scene_delete is the problem this fixes; those verbs also "
        "accept physics='rebuild' inline). Refused with 409 REBUILD_REFUSED "
        "on Cloth/SoftBody/GranularBed worlds (reload those). Engaged "
        "Connector/VacuumGripper welds are DROPPED loudly -- re-lock from "
        "the controller.",
        {"type": "object",
         "properties": {
             "settle_steps": {"type": "integer", "minimum": 1,
                              "description": "engine steps after the rebuild "
                                             "(default 8)"},
         }},
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
    "get_capabilities": (
        t_get_capabilities,
        "What this harness can and will not do — the endpoint the harness docs "
        "say to call first. Returns the verified physics backend (from the "
        "engine's own .newton.json sidecar), measured per-step cost + a "
        "recommended step budget (probe_step=true advances ONE step to measure "
        "it), the live event-type list with what a light-mode session "
        "suppresses, every endpoint, every gap under not_supported (each with "
        "a reason + workaround), and the diagnostic-code enums.",
        {"type": "object",
         "properties": {"probe_step": {
             "type": "boolean",
             "description": "advance one step to measure real step cost"}}},
    ),
    "world_sync": (
        t_world_sync,
        "Explicitly re-sync the loaded world with its edited file: proven "
        "root-DEF pose-only edits land live in one batch (mode=live_pose, "
        "~325 ms measured); every other edit automatically hot-reloads "
        "(mode=full_reload). Do not pre-classify the edit yourself. Also "
        "returns mode=no_change | rejected (422) | busy (409, retry). "
        "load_world already routes here by default; use this when you want "
        "the sync semantics by name, or to sync without repeating the path.",
        {"type": "object",
         "properties": {
             "path": {"type": "string",
                      "description": "world file (defaults to the loaded one)"},
             "settle_steps": {"type": "integer", "minimum": 0,
                              "description": "steps after a live pose batch (default 1)"},
             "reset_physics": {"type": "boolean",
                               "description": "clear moved-body velocity (default true)"},
             "wait_s": {"type": "number", "description": "reload timeout seconds"},
             "light": {"type": "boolean",
                       "description": "low-overhead supervisor on a full reload"},
         }},
    ),
    "scene_spawn": (
        t_scene_spawn,
        "Import a node into the live scene from raw VRML, a type+fields spec, "
        "or clone of an existing DEF. ⛔ A SCENE-GRAPH VERB, NOT A PHYSICS "
        "VERB: the solver model is frozen at world finalize, so the spawned "
        "node has NO physics until the world is reloaded (a dynamic body "
        "never falls, a static one never collides — the response's "
        "physics_warning says so). Use it for cameras/markers/visual props or "
        "to stage a scene you will then reload. Cloning needs a unique `name` "
        "or the clone's controller silently dies on an IPC collision; a "
        "URDFRobot cannot be spawned from a string — clone one.",
        {"type": "object",
         "properties": {
             "vrml": {"type": "string", "description": "raw VRML node text"},
             "type": {"type": "string", "description": "node type (with fields)"},
             "fields": {"type": "object", "description": "field map for `type`"},
             "clone": {"type": "string", "description": "DEF of a node to clone"},
             "def": {"type": "string", "description": "DEF for the new node"},
             "name": {"type": "string",
                      "description": "unique Solid/Robot name (required for robot clones)"},
             "translation": dict(_VEC3, description="initial position [x,y,z]"),
             "rotation": {"type": "array", "items": {"type": "number"},
                          "minItems": 4, "maxItems": 4,
                          "description": "axis-angle [x,y,z,rad]"},
             "parent": {"type": "string", "description": "DEF of the parent node"},
             "index": {"type": "integer", "description": "insertion index in the parent"},
             "settle_steps": {"type": "integer", "minimum": 0},
             "reset_physics": {"type": "boolean"},
         }},
    ),
    "scene_delete": (
        t_scene_delete,
        "Remove nodes by DEF (unknown DEFs come back named rather than "
        "failing the batch). ⛔ The frozen solver model KEEPS the deleted "
        "colliders as phantoms until the world is reloaded — a deleted wall "
        "still blocks rays and robots, a deleted floor still holds bodies up, "
        "silently (the response's physics_warning says so). Reload after "
        "deleting anything collidable.",
        {"type": "object",
         "properties": {
             "def": {"type": "string", "description": "DEF of the node to remove"},
             "defs": {"type": "array", "items": {"type": "string"},
                      "description": "several DEFs at once"},
             "settle_steps": {"type": "integer", "minimum": 0},
         }},
    ),
    "scene_set_pose": (
        t_scene_set_pose,
        "Move an existing node by DEF. Defaults settle_steps=1 and "
        "reset_physics=true (a teleported body otherwise keeps its velocity "
        "and drifts, which reads as 'the pose did not stick'). ⚠ Nothing "
        "checks interpenetration — placed inside static geometry a dynamic "
        "body can tunnel through the floor; check bounds first with "
        "get_scene_node bounds=true.",
        {"type": "object",
         "properties": {
             "def": {"type": "string", "description": "DEF of the node to move"},
             "translation": dict(_VEC3, description="new position [x,y,z]"),
             "rotation": {"type": "array", "items": {"type": "number"},
                          "minItems": 4, "maxItems": 4,
                          "description": "axis-angle [x,y,z,rad]"},
             "reset_physics": {"type": "boolean",
                               "description": "clear the body's velocity (default true)"},
             "settle_steps": {"type": "integer", "minimum": 0},
         },
         "required": ["def"]},
    ),
    "sim_snapshot": (
        t_sim_snapshot,
        "Save a named engine-side state snapshot of the whole scene — a "
        "rollback point that is not t=0. Names die with the world (every "
        "load restarts the registry); '__'-prefixed names are reserved.",
        {"type": "object",
         "properties": {"name": {"type": "string", "description": "snapshot name"}},
         "required": ["name"]},
    ),
    "sim_restore": (
        t_sim_restore,
        "Restore a named snapshot WITHOUT rewinding the clock, and report how "
        "far it landed (verification.vs_snapshot.max_pose_delta_m). An "
        "unknown name is refused (404 SNAPSHOT_NOT_FOUND) on purpose — an "
        "unguarded miss would teleport the scene to the origin. For the "
        "authored t=0 state use sim_reset instead.",
        {"type": "object",
         "properties": {
             "name": {"type": "string", "description": "snapshot name to restore"},
             "settle_steps": {"type": "integer", "minimum": 0},
         },
         "required": ["name"]},
    ),
    "list_snapshots": (
        t_list_snapshots,
        "List the named state snapshots taken in this world.",
        {"type": "object", "properties": {}},
    ),
    "get_grips": (
        t_get_grips,
        "Inferred grips: [{gripper_def, held_def, since_t_ms}]. ⚠ Empty in a "
        "light-mode session (the grip tracker is dropped with light=true) — "
        "and a contact read is always a weaker claim than proving the grasp "
        "geometrically (the part is airborne and tracks the gripper).",
        {"type": "object", "properties": {}},
    ),
    "robot_devices": (
        t_robot_devices,
        "Device inventory of one robot's subtree (name + type per device). "
        "⚠ A URDF robot loaded without OMNISIM_URDF_USE_SENSORS=1 has ZERO "
        "devices — 'no sensors' is almost always that gate, not a robot that "
        "carries nothing.",
        {"type": "object",
         "properties": {"def": {"type": "string", "description": "the robot's DEF name"}},
         "required": ["def"]},
    ),
    "robot_joints_set": (
        t_robot_joints_set,
        "Command joint position targets on one robot, settle-and-verify: each "
        "joint returns measured {commanded, achieved, error, moved, clamped}, "
        "never the argument echoed back. NOT a teleport — the write re-pins "
        "the motor's PD setpoint and converges over the settled steps. ⚠ A "
        "motor with no position limits is a velocity wheel whose position "
        "targets the physics ignores (reported per joint as "
        "position_controllable=false); ⚠ an active bridge in hold mode "
        "re-asserts its own targets and WINS — command bridge-owned robots "
        "through their bridge.",
        {"type": "object",
         "properties": {
             "def": {"type": "string", "description": "the robot's DEF name"},
             "joints": {"type": "object",
                        "description": "{joint_name: target_rad} — names as "
                                       "reported by get_robot_joints"},
             "settle_steps": {"type": "integer", "minimum": 0,
                              "description": "settle-and-measure steps (default 16)"},
         },
         "required": ["def", "joints"]},
    ),
    "robot_ik": (
        t_robot_ik,
        "Batched inverse-kinematics PREVIEW against the exact model the "
        "solver steps — nothing moves. Returns per-target joint angles "
        "(clamped to authored limits, keyed by get_robot_joints names) plus "
        "residual_m measured by forward kinematics on the returned angles: "
        "reject a target on its residual instead of driving to it, then apply "
        "via robot_joints_set. ⚠ First solve per world compiles a warp kernel "
        "(~8 s cold, ~150 ms warm; solve_ms in the response). Hinge/Slider "
        "joints only; verified on the CPU 'mujoco' solver only.",
        {"type": "object",
         "properties": {
             "def": {"type": "string", "description": "the robot's DEF name"},
             "effector": {"type": "string",
                          "description": "DEF of the end-effector Solid"},
             "targets": {"type": "array",
                         "items": dict(_VEC3, description="world-frame [x,y,z]"),
                         "description": "world-frame target positions"},
             "rotations": {"type": "array",
                           "items": {"type": "array", "items": {"type": "number"},
                                     "minItems": 4, "maxItems": 4},
                           "description": "optional per-target quaternions [qx,qy,qz,qw]"},
             "tool_offset": dict(_VEC3, description="offset in the effector's frame"),
             "iterations": {"type": "integer", "minimum": 1,
                            "description": "solver iterations (optional)"},
         },
         "required": ["def", "effector", "targets"]},
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


def _cli_help() -> int:
    """Answer --help/--version without entering the stdio loop.

    `main()` ignored argv entirely, so `omnisim-mcp --help` -- the obvious way
    to sanity-check an install -- printed a startup line and then blocked
    forever on a TTY reading stdin. A server that hangs when asked for help
    reads as a broken install.
    """
    print("omnisim-mcp %s -- MCP server over the OmniSim harness." % SERVER_INFO["version"])
    print()
    print("It is a PROXY. It needs a running harness, which needs a built engine:")
    print("  python -m omnisim doctor      # is this install able to run a world?")
    print("  python -m omnisim harness     # start the harness on :6789")
    print()
    print("harness: %s  (override with OMNISIM_HARNESS_URL)" % DEFAULT_HARNESS)
    print("tools (%d): %s" % (len(TOOLS), ", ".join(sorted(TOOLS))))
    print()
    print("Normally you do not run this by hand -- an MCP client spawns it and")
    print("speaks JSON-RPC over stdin/stdout. See packages/omnisim-mcp/README.md.")
    return 0


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if "--version" in argv or "-V" in argv:
        print(SERVER_INFO["version"])
        return 0
    if "--help" in argv or "-h" in argv:
        return _cli_help()
    if "--self-test" in argv:
        # Validate the install without an MCP client: does the harness answer?
        result = _tools_call({"name": "harness_status", "arguments": {}})
        print(json.dumps(result, indent=2))
        return 0 if not result.get("isError") else 1
    log(f"starting; harness = {DEFAULT_HARNESS}. Reading MCP stdio.")
    # Line-delimited JSON on stdin; one JSON response line per request on stdout.
    #
    # Tool calls run on ONE worker thread (per-tool serialization preserved:
    # the queue keeps them strictly in arrival order), while the reader thread
    # answers control messages (ping / initialize / tools/list) immediately —
    # otherwise a long harness call (a 13 s world load, a big step batch)
    # blocks every other MCP message including the client's keep-alive ping,
    # which reads as a dead server.
    import queue

    out_lock = threading.Lock()

    def write(resp: dict) -> None:
        with out_lock:
            sys.stdout.write(json.dumps(resp) + "\n")
            sys.stdout.flush()

    def dispatch(msg: dict) -> None:
        try:
            resp = _handle(msg)
        except Exception as e:  # never let one message kill the loop
            log(f"handler crashed: {e!r}")
            mid = msg.get("id") if isinstance(msg, dict) else None
            resp = {"jsonrpc": "2.0", "id": mid,
                    "error": {"code": -32603, "message": f"internal error: {e}"}}
        if resp is not None:
            write(resp)

    work: queue.Queue = queue.Queue()

    def worker() -> None:
        while True:
            msg = work.get()
            if msg is None:
                return
            dispatch(msg)

    wt = threading.Thread(target=worker, name="tool-worker", daemon=True)
    wt.start()

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as e:
            log(f"dropping non-JSON line: {e}")
            continue
        if isinstance(msg, dict) and msg.get("method") == "tools/call":
            work.put(msg)  # serialized on the worker; may be slow
        else:
            dispatch(msg)  # control-plane: answered immediately
    work.put(None)
    wt.join(timeout=5.0)  # let an in-flight tool call finish writing
    log("stdin closed; exiting.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
