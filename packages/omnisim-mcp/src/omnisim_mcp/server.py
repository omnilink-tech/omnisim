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
Robotics has almost no observability. You cannot set a breakpoint on a robot,
and when one misbehaves the state of the art is to watch it closely and guess.
OmniSim ships the answer as a first-party, agent-facing HTTP surface — the World
Harness, PROTOCOL.md §world_harness — where you run a controller and then ask
the scene what actually happened: contacts, joint limits hit, grips, damage
events and the controller's own log lines on one cursor-paged event stream, plus
joint, device and bounds inspection that internally holds the engine for the
duration of its own walk, so each answer is one consistent instant. This package is how an agent
reaches that surface: the agent ecosystem — Claude Desktop, Cursor, the tool
marketplaces — standardized on the **Model Context Protocol (MCP)**, and until
now OmniSim was invisible to it.

The instruments say when they cannot see, which is the part worth having in a
debugger: `get_contacts` returns `completeness` and `empty_set_reasons[]` rather
than an empty list that reads as "nothing touched", and `get_capabilities`
publishes what the simulator refuses to do, with a reason and a workaround per
gap. Pause is real: `sim_pause` holds the engine across calls under a lease and
`sim_break` arms a break on an event, so single-stepping and breakpoints work —
and they state their own blind spots too. Break detection is **never sub-step**:
held and driven by `sim_step` it is per basic step, free-running it is one
supervisor tick (8 ms to ~600 ms of engine time), so pause first and then step.
A break armed on an event type this session silences is **refused**, not armed
and left unable to fire. Record, replay, run-diff and a true checkpoint stay
absent (`sim_snapshot` saves poses and joint angles only, never velocity), and
`sim.watch` — a polled predicate over pose or joint state — is declared
unsupported rather than approximated. `load_world` is **light by
default**, silencing contact/grip/joint-limit events — pass `light: false` for a
debugging session. The community servers wrap a simulator that was not built for
this; here the surface already existed, so this is a thin adapter, not a rewire.

Two surfaces, not one
---------------------
Most tools here reach the **World Harness** (`:6789`) — the authoring and
debugging surface. Four reach a robot's **OmniLink bridge** instead
(`robot_prompt`, `robot_tool`, `robot_state`, `robot_events`), which is how you
TALK to the robot and hear back from it: an operator sentence, a typed tool
call, its own state, and its own event ring. Only `/prompt` and `/tool` are
gated (`/get_robot_state` and `/events` are pure reads); the bridge's direct REST
verbs (`/drive_forward`, `/turn`, `/set_velocity`, `/stop_robot`) are not, and
this server never calls them — see packages/omnisim-bridges/GATE_COVERAGE.md.

This server is a **stateless proxy**: every tool call is one HTTP request to a
running harness (default `http://127.0.0.1:6789`), over one pooled
`http.client` connection. The harness speaks HTTP/1.1 with keep-alive since
2026-09-01 (measured on the flip: connection reuse 0/229 -> 229/229, `GET
/healthz` 5.09 -> 0.31 ms), so the pool genuinely reuses the socket; against
an older HTTP/1.0 harness it still detects `will_close` and degrades to
per-request connections automatically. It holds no simulator state of its
own, which is why it needs no heavy runtime — it is pure stdlib (stdio
JSON-RPC + http.client), matching the harness's own zero-dependency design,
so it runs on a fresh clone with nothing installed.

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

The harness itself must be running: `python -m omnisim harness` (the module
form pins OMNISIM_HOME and, on Windows, puts the bundled Qt DLLs on PATH for the
engine subprocess; the raw script form fails the first world load with
LAUNCHER_DLL_NOT_FOUND there). Call the `harness_status` tool first — it
reports whether the harness is reachable and how to start it if not.
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
SERVER_INFO = {"name": "omnisim-mcp", "version": "0.2.0"}
# OMNISIM_HARNESS_URL names the World Harness this process should talk to, as a
# base URL. It has three consumers and one meaning: this MCP server, the ROS 2
# harness client, and (since v9) a robot bridge, which uses its PRESENCE as the
# signal that a harness exists at all and re-emits that harness's events onto
# its own ring tagged `source: "harness"`. Unset means "no harness": the MCP
# server falls back to the documented default port, and a bridge simply raises
# no forwarded events rather than polling something that is not there.
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

# --- the command surface (a robot's OmniLink bridge, NOT the harness) --------
# Every bridge serves on loopback. 8765 is the convention across the chat
# demos; the Mavic drone serves 6090 and the langsoak Husky 8775.
BRIDGE_HOST = "127.0.0.1"
DEFAULT_BRIDGE_PORT = 8765
# The bridge's own auth token, when one is configured. A loopback bridge needs
# none (check_authorization returns early on an empty token), but a bridge
# started with OMNISIM_BRIDGE_TOKEN set answers 401 unauthorized without it, so
# read the same variable the bridges do rather than inventing a second name.
BRIDGE_TOKEN = os.environ.get("OMNISIM_BRIDGE_TOKEN", "").strip()
# ⛔ THE ONLY FOUR BRIDGE PATHS THIS SERVER MAY CALL. `/prompt` and `/tool`
# are the gate-vetted ones; `/get_robot_state` and `/events` are pure reads.
# The direct REST actuators (`/drive_forward`, `/turn`, `/set_velocity`,
# `/stop_robot`, `/reset_to_home`, ...) bypass the safety gate entirely
# (packages/omnisim-bridges/GATE_COVERAGE.md) and are deliberately unreachable
# from here: this allowlist is checked at call time so a future edit cannot add
# one by accident.
BRIDGE_PATHS = ("/prompt", "/tool", "/get_robot_state", "/events")
# Transport fields that must NEVER travel as tool ARGUMENTS. A bridge's /tool
# handler pops "tool" and dispatches the REST of the body as the arguments, so
# an MCP request id riding along arrives at the safety gate as `unknown_arg` --
# which is how the Mavic once refused a perfectly good takeoff.
_TRANSPORT_KEYS = ("id", "tool", "port")


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
      World Harness WAS such a peer until 2026-09-01 and speaks HTTP/1.1
      keep-alive now, so the pool reuses one socket per harness; the detection
      stays so an older harness is still driven correctly.
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
             base: str | None = None, timeout: float | None = None,
             extra_headers: dict | None = None):
    """One HTTP call to the harness. Returns (status, headers, raw_bytes)."""
    base = (base or DEFAULT_HARNESS).rstrip("/")  # resolved at call time so tests/env can override
    data = json.dumps(body).encode() if body is not None else None
    headers = {"Accept": "application/json"}
    if data is not None:
        headers["Content-Type"] = "application/json"
    if extra_headers:
        headers.update(extra_headers)
    return _POOL.request(method, base, path, data, headers,
                         HTTP_TIMEOUT_S if timeout is None else timeout)


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


def _bridge_port(args: dict) -> int:
    try:
        return int(args.get("port") or DEFAULT_BRIDGE_PORT)
    except (TypeError, ValueError):
        raise ValueError(f"port must be an integer, got {args.get('port')!r}")


def _bridge_call(path: str, port: int, body: dict | None,
                 timeout: float | None = None, method: str = "POST") -> dict:
    """One HTTP call to a robot's OmniLink bridge, envelope passed through.

    ⚠️ THE BODY IS RETURNED VERBATIM. Whatever the bridge answered — a
    `refused_by_gate` refusal, a `{commanded, achieved, error, settled}`
    control result, a 401 `omnikey_required` — reaches the agent unedited.
    The single added key is `http_status`, and only via `setdefault`, so a
    bridge that sends its own keeps it. Interpreting the envelope here is
    exactly the bug this guards against: `error` inside a control result is
    the CONTROL error in metres or radians, not a failure flag, and a wrapper
    that read it as one once reported "I could not stop: 4.3e-11" after a
    textbook stop.
    """
    # The allowlist is checked on the ROUTE, with any query string cut off:
    # `/events?since=12` is the same route as `/events`, and a check that saw
    # the whole string would either refuse a legitimate cursor read or invite
    # someone to "fix" it by loosening the test to a prefix match — at which
    # point `/stop_robot?x=1` walks straight through.
    route = path.split("?", 1)[0]
    if route not in BRIDGE_PATHS:  # see BRIDGE_PATHS: the ungated verbs stay out
        raise ValueError(f"refusing to call the bridge path {path!r}: this "
                         f"server only calls {', '.join(BRIDGE_PATHS)} "
                         f"(the other REST verbs bypass the safety gate)")
    base = f"http://{BRIDGE_HOST}:{int(port)}"
    headers = {"Authorization": f"Bearer {BRIDGE_TOKEN}"} if BRIDGE_TOKEN else None
    try:
        status, _headers, raw = _request(method, path, body, base=base,
                                         timeout=timeout,
                                         extra_headers=headers)
    except HarnessError as exc:
        raise HarnessError(
            f"cannot reach an OmniLink bridge at {base}{path} ({exc.__cause__ or exc}). "
            f"The robot's world must be RUNNING and its bridge controller "
            f"listening: launch a chat world (e.g. "
            f"projects/samples/demos/worlds/chat/omnilink_husky.omniworld) and "
            f"retry. Ports: 8765 for most robots, 6090 for the Mavic drone, "
            f"8775 for the langsoak Husky."
        ) from exc
    try:
        parsed = json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        parsed = {"raw": raw.decode("utf-8", "replace")}
    if isinstance(parsed, list):        # /list_robots-shaped answers
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
    for k in ("wait_s", "with_supervisor", "light", "tracking", "settle_steps",
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
    else:
        # /world/sync has no per-tracker toggle; the per-tracker `tracking`
        # object is a /world/load contract (PROTOCOL.md, public issue #4).
        # Asking for one on the sync path is a deliberate reload.
        if "tracking" in body:
            endpoint = "/world/load"
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


def t_read_bench(args: dict) -> list:
    suffix = f"?n={int(args['n'])}" if "n" in args else ""
    return _text(_json_call("GET", "/debug/read_bench" + suffix))


def t_scene_node_particles(args: dict) -> list:
    suffix = f"?sample={int(args['sample'])}" if "sample" in args else ""
    return _text(_json_call("GET", f"/scene/node/{args['def']}/particles" + suffix))


def t_robot_damage(_args: dict) -> list:
    return _text(_json_call("GET", "/robot/damage"))


def t_robot_damage_events(args: dict) -> list:
    q = []
    for k in ("since", "limit"):
        if k in args:
            q.append(f"{k}={urllib.request.quote(str(args[k]))}")
    path = "/robot/damage/events" + ("?" + "&".join(q) if q else "")
    return _text(_json_call("GET", path))


def t_robot_damage_reset(_args: dict) -> list:
    return _text(_json_call("POST", "/robot/damage/reset", {}))


def t_robot_damage_inject(args: dict) -> list:
    body = {"part": args["part"]}
    for k in ("state", "hp_delta"):
        if k in args:
            body[k] = args[k]
    return _text(_json_call("POST", "/robot/damage/inject", body))


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


def t_get_contacts(args: dict) -> list:
    suffix = ""
    if "settle_steps" in args:
        suffix = f"?settle_steps={int(args['settle_steps'])}"
    return _text(_json_call("GET", "/sim/contacts" + suffix))


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
                                 "reset_physics", "physics") if k in args}
    return _text(_json_call("POST", "/scene/spawn", body))


def t_scene_delete(args: dict) -> list:
    body = {k: args[k] for k in ("def", "defs", "settle_steps", "physics")
            if k in args}
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


# --- the pause primitive and the breakpoint built on it (harness) ----------- #
def t_sim_pause(args: dict) -> list:
    body = {}
    if args.get("lease_ms") is not None:
        body["lease_ms"] = int(args["lease_ms"])
    return _text(_json_call("POST", "/sim/pause", body))


def t_sim_resume(_args: dict) -> list:
    return _text(_json_call("POST", "/sim/resume", {}))


def t_sim_break(args: dict) -> list:
    types = args["types"]
    if isinstance(types, str):  # forgiving: "contact.began,grip.began"
        types = [t.strip() for t in types.split(",") if t.strip()]
    body: dict = {"types": list(types)}
    for k in ("filter", "lease_ms", "once"):
        if k in args and args[k] is not None:
            body[k] = args[k]
    return _text(_json_call("POST", "/sim/break", body))


def t_sim_breaks(_args: dict) -> list:
    return _text(_json_call("GET", "/sim/breaks"))


def t_sim_break_clear(args: dict) -> list:
    """Disarm one break.

    ⚠️ BOTH SPELLINGS SHIPPED (verified live 2026-09-22, PROTOCOL.md §7.40):
    the harness serves ``DELETE /sim/break/<id>`` *and* its exact twin
    ``POST /sim/break/delete {"break_id": ...}`` — same 200 body
    (``{break_id, removed, rpc_ms, ok}``), same ``404 BREAK_NOT_FOUND``. The
    twin exists for clients whose HTTP layer cannot route a bodyless DELETE.
    So this is not a guess about which one is real; DELETE is preferred and the
    POST is the compatibility path.

    The fall-back is kept anyway, and it is deliberately narrow: it fires only
    when the answer looks like a MISSING ROUTE (an error with no ``break_id``
    in it), never when it is the route refusing a genuinely unknown id —
    re-sending that as a POST would turn one honest 404 into two confusing
    ones. That keeps this client working against an older harness.
    """
    bid = str(args["break_id"])
    out = _json_call("DELETE", "/sim/break/" + urllib.parse.quote(bid, safe=""))
    if int(out.get("http_status") or 0) >= 400 and "break_id" not in out:
        alt = _json_call("POST", "/sim/break/delete", {"break_id": args["break_id"]})
        alt.setdefault("route", "POST /sim/break/delete")
        return _text(alt)
    out.setdefault("route", "DELETE /sim/break/<id>")
    return _text(out)


# --- the command surface: a robot's OmniLink bridge, not the harness -------- #
def t_robot_prompt(args: dict) -> list:
    body = {"text": args["text"]}
    timeout = None
    if args.get("timeout_s") is not None:
        body["timeout_s"] = float(args["timeout_s"])
        # Never abandon a request the bridge is still faithfully serving; the
        # socket must outlive the bridge's own budget for the turn.
        timeout = max(HTTP_TIMEOUT_S, float(args["timeout_s"]) + 15.0)
    return _text(_bridge_call("/prompt", _bridge_port(args), body, timeout))


def t_robot_tool(args: dict) -> list:
    tool = args["tool"]
    raw_args = args.get("args") or {}
    if not isinstance(raw_args, dict):
        raise ValueError("args must be an object of the tool's own parameters")
    # The bridge's /tool takes the arguments FLATTENED beside "tool", and then
    # dispatches everything that is not "tool" as the argument map. So the
    # transport fields are stripped HERE, on the way in, not hopefully ignored
    # on the way out.
    body = {"tool": tool}
    for k, v in raw_args.items():
        if k in _TRANSPORT_KEYS:
            continue
        body[k] = v
    return _text(_bridge_call("/tool", _bridge_port(args), body))


def t_robot_state(args: dict) -> list:
    return _text(_bridge_call("/get_robot_state", _bridge_port(args), {}))


def t_robot_events(args: dict) -> list:
    """The robot's OWN event ring (`GET /events` on its bridge), cursor-paged.

    ⚠️ TWO RINGS, TWO CURSORS. This is the BRIDGE's ring — what the robot's
    own detectors saw, stamped in SIM time — and `get_events` is the HARNESS's
    stream. A bridge with `OMNISIM_HARNESS_URL` set re-emits some harness
    events into its own ring tagged `source: "harness"`, so an event can
    legitimately appear on both with different cursors. Never carry a cursor
    from one to the other.
    """
    q = []
    for key in ("since", "limit", "types"):
        val = args.get(key)
        if val is None:
            continue
        if key == "types" and isinstance(val, (list, tuple)):
            val = ",".join(str(t).strip() for t in val if str(t).strip())
        q.append(f"{key}={urllib.request.quote(str(val))}")
    path = "/events" + ("?" + "&".join(q) if q else "")
    # A GET with no body: the ring is a pure read and nothing about it is
    # gated, because reading what already happened cannot actuate anything.
    return _text(_bridge_call(path, _bridge_port(args), None, method="GET"))


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
        "Default world iteration tool. The first call loads the world (.omniworld, "
        "or legacy .wbt); later calls live-apply proven root-node pose-only edits in "
        "one batch and automatically hot-reload every other edit. Returns "
        "mode=live_pose|no_change|full_reload. Set force_reload=true only when you "
        "deliberately need controllers restarted or a full reparse. ⭐ LIGHT "
        "TRACKING IS THE HARNESS DEFAULT SINCE 2026-09-02: omit both `light` and "
        "`tracking` and the load runs light, and the response says so "
        "(tracking.default_applied=true + a sentence naming the way back). It was "
        "flipped because with the trackers on the harness is slower than just "
        "re-running run-headless: measured on the 309-node fleet arena (2026-08-29), "
        "a full-tracking step costs 573-606 ms against 6-35 ms light (~17x), 10 steps "
        "2855-3187 ms vs 48-67 ms (~47x), and the load itself 12.1 s vs 4.1 s. The "
        "trade: the contact.*/grip.*/joint.limit_hit EVENTS and get_grips go quiet; "
        "get_contacts still answers. Need the trackers? An explicit value ALWAYS "
        "wins: light=false runs all three, `tracking` (per-tracker toggles, "
        "2026-09-01) exactly the ones you name; OMNISIM_HARNESS_LIGHT=0 on the "
        "harness restores the old full default process-wide.",
        {"type": "object",
         "properties": {
             "path": {"type": "string",
                      "description": "repo-relative (to the harness's clone) or absolute "
                                     "path to the .omniworld / .wbt"},
             "wait_s": {"type": "number", "description": "load timeout seconds"},
              "with_supervisor": {"type": "boolean",
                                  "description": "inject the harness supervisor (default true)"},
              "light": {"type": "boolean",
                        "description": "drop ALL per-step trackers (contacts, joint "
                                       "limits, grips). THE HARNESS DEFAULT since "
                                       "2026-09-02 when neither this nor `tracking` is "
                                       "passed (2.3x cheaper single steps and an 11% faster load on the fleet arena, measured 2026-09-02 on the current engine; the 2026-08-29 figures of 12.1 s vs 4.1 s and 17-47x predate the controller-probe cache and immediate-burst fixes; "
                                       "~4x on a 10-node cloth world; get_contacts still "
                                       "works). Pass false to run all three trackers "
                                       "(get_grips + contact/grip/joint-limit events)."},
              "tracking": {"type": "object",
                           "description": "per-tracker toggles, e.g. {\"contacts\": false, "
                                          "\"joint_limits\": true, \"grips\": false} -- "
                                          "keep joint.limit_hit while paying no contact "
                                          "walk (partial mode steps at ~light cost). "
                                          "Keys: contacts, joint_limits, grips. Forces "
                                          "a /world/load (the sync path has no toggle).",
                           "properties": {
                               "contacts": {"type": "boolean"},
                               "joint_limits": {"type": "boolean"},
                               "grips": {"type": "boolean"}}},
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
        "inline so you can see it; with `path`, writes it server-side. Call "
        "`frame` first (it computes aim AND distance from the subject's real "
        "bounds and returns a numeric proof it is in frame) -- do not guess a "
        "pose and iterate on screenshots; `visible` tells you what is on screen "
        "and by how many degrees you are off, `render_stats` catches a blown-out "
        "or black frame as numbers.",
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
        "Advance the simulation by N basic timesteps (default 1). Size N from "
        "get_capabilities -> limits.recommended_max_steps_per_request (a rolling "
        "median of the MEASURED per-step cost on THIS world; probe_step=true "
        "measures one) instead of discovering the harness's 120 s RPC timeout by "
        "hitting it. Per-step cost is dominated by tracking mode, not node count: "
        "light is the harness default since 2026-09-02 (see load_world); a "
        "light=false or `tracking` load pays about 2.3x per single step on the fleet arena (measured 2026-09-02; the older 17-47x figure predates the 2026-09-02 engine fixes). "
        "⭐ UNDER A PAUSE (sim_pause, or a break that fired) this still advances "
        "and then RE-HOLDS -- that is single-stepping -- and the response "
        "reports `paused` and `lease_remaining_ms`. ⚠️ IT STOPS EARLY on the "
        "step an armed break fires, which is what makes it "
        "CONTINUE-TO-BREAKPOINT: read `steps_executed` and `stopped_on_break`, "
        "never `steps_requested` (measured: requested 400, executed 118). "
        "⚠️ The advance is exact on the SUPERVISOR clock (`sim_time_ms`) and NOT "
        "on the engine clock: 10 steps moved the supervisor clock by exactly "
        "80 ms and the engine clock by 80-112 ms (0-4 basic steps of overshoot), "
        "because lifting and re-taking the pause around the step is a race the "
        "supervisor binding cannot close. That is why the response REPORTS "
        "`engine_advanced_ms` rather than asserting it -- do not assert it "
        "either. `sim_time_ms` and `engine_time_ms` are different rulers.",
        {"type": "object",
         "properties": {"steps": {"type": "integer", "minimum": 1,
                                  "description": "basic timesteps to advance; keep at or "
                                                 "below recommended_max_steps_per_request"}}},
    ),
    "read_bench": (
        t_read_bench,
        "Diagnostic: measure the cost of ONE supervisor read on this session "
        "(n getPosition round-trips, free-running vs paused) -- "
        "GET /debug/read_bench. Measured, never echoed. Use it to size a polling "
        "loop or to check the 2026-09-01 immediate-burst fix is in effect "
        "(7.4 -> 0.6 ms/read on the 309-node fleet arena).",
        {"type": "object",
         "properties": {"n": {"type": "integer", "minimum": 1, "maximum": 1000,
                              "description": "reads per arm (default 50)"}}},
    ),
    "scene_node_particles": (
        t_scene_node_particles,
        "Particle statistics for one Cloth / SoftBody / GranularBed / "
        "GranularGroup node by DEF (GET /scene/node/<def>/particles): count, "
        "world-frame min/max/centroid over the FINITE particles, and non_finite "
        "-- a diverging cloth reads as a rising non_finite, never a NaN "
        "centroid. Pure read off the engine's per-step particle cache; "
        "sample=N adds every N-th particle's xyz.",
        {"type": "object",
         "properties": {"def": {"type": "string", "description": "the particle node's DEF"},
                        "sample": {"type": "integer", "minimum": 1,
                                   "description": "include every N-th particle's xyz"}},
         "required": ["def"]},
    ),
    "robot_damage": (
        t_robot_damage,
        "Damage state of the tracked robot (GET /robot/damage): per-part HP and "
        "state. Only meaningful in a world whose supervisor tracks damage (the "
        "robot_combat / damage demos); elsewhere the response says so.",
        {"type": "object", "properties": {}},
    ),
    "robot_damage_events": (
        t_robot_damage_events,
        "Filtered view of the damage.* events (GET /robot/damage/events) -- the "
        "same records get_events carries, with their own `since` cursor.",
        {"type": "object",
         "properties": {"since": {"type": "integer", "description": "event cursor"},
                        "limit": {"type": "integer"}}},
    ),
    "robot_damage_reset": (
        t_robot_damage_reset,
        "Heal every part of the tracked robot WITHOUT resetting the simulation "
        "(POST /robot/damage/reset).",
        {"type": "object", "properties": {}},
    ),
    "robot_damage_inject": (
        t_robot_damage_inject,
        "Set a part's damage state directly (POST /robot/damage/inject) -- the "
        "fault-injection verb, so a damage-response path can be tested without "
        "staging a collision.",
        {"type": "object",
         "properties": {"part": {"type": "string", "description": "part name"},
                        "state": {"type": "string", "description": "damage state to set"},
                        "hp_delta": {"type": "number", "description": "HP change to apply"}},
         "required": ["part"]},
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
        "poses (97-267 ms measured; velocities replayed, motor targets "
        "re-pushed, so a driving robot keeps driving), so runtime-spawned "
        "nodes gain physics and deleted ones lose it (the frozen-model "
        "physics_warning on scene_spawn/scene_delete is the problem this "
        "fixes; those verbs also accept physics='rebuild' inline). Refused "
        "with 409 REBUILD_REFUSED on Cloth/SoftBody/GranularBed worlds "
        "(reload those). Engaged Connector/VacuumGripper welds are DROPPED "
        "loudly -- do not rebuild mid-grasp. Bitwise step-for-step "
        "continuation across a rebuild is not claimed.",
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
        "Global contact set (GET /sim/contacts): `contacts` = [{a_def, b_def, "
        "point, paired}] -- `paired` false means only one side answered "
        "(b_def null), an honest half-contact rather than a dropped one -- "
        "plus a `tracking` block naming what was walked (solids walked, "
        "contacts_paired/unpaired, empty_set_reasons, inert_pinned_solids). "
        "WORKS IN LIGHT MODE: it is walked per call and never reads the "
        "ContactTracker that light=true drops (only the contact.* EVENTS and "
        "get_grips go quiet). An EMPTY list is never proof of no contact -- "
        "read tracking.empty_set_reasons. There is no body sleep on Newton, "
        "so ?wake=1 is a no-op and is not exposed here.",
        {"type": "object",
         "properties": {"settle_steps": {
             "type": "integer", "minimum": 0,
             "description": "advance this many steps before the walk (optional)"}}},
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
                       "description": "low-overhead supervisor on a full reload -- the "
                                      "harness default since 2026-09-02; omitted keeps the "
                                      "running session's mode, false runs all trackers"},
         }},
    ),
    "scene_spawn": (
        t_scene_spawn,
        "Import a node into the live scene from raw VRML, a type+fields spec, "
        "or clone of an existing DEF. ⛔ BY DEFAULT A SCENE-GRAPH VERB, NOT A "
        "PHYSICS VERB: the solver model is frozen at world finalize, so "
        "without opting in the spawned node has NO physics (a dynamic body "
        "never falls, a static one never collides — the response's "
        "physics_warning says so). Pass physics='rebuild' (or call "
        "rebuild_physics afterwards) and the node IS simulated: the Newton "
        "world is rebuilt at the scene's current poses in 97-267 ms; refused "
        "on cloth/soft/granular worlds, engaged welds dropped. Without it, use "
        "spawn for cameras/markers/visual props or to stage a scene you will "
        "then reload. Cloning needs a unique `name` or the clone's controller "
        "silently dies on an IPC collision; a URDFRobot cannot be spawned from "
        "a string — clone one.",
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
             "physics": {"type": "string", "enum": ["rebuild"],
                         "description": "'rebuild' = rebuild the Newton world after the "
                                        "spawn so the node has physics (W1.7)"},
         }},
    ),
    "scene_delete": (
        t_scene_delete,
        "Remove nodes by DEF (unknown DEFs come back named rather than "
        "failing the batch). ⛔ BY DEFAULT the frozen solver model KEEPS the "
        "deleted colliders as phantoms — a deleted wall still blocks rays and "
        "robots, a deleted floor still holds bodies up, silently (the "
        "response's physics_warning says so). Pass physics='rebuild' (or call "
        "rebuild_physics afterwards) and the deleted geometry genuinely stops "
        "colliding -- same 97-267 ms rebuild and the same caveats as "
        "scene_spawn. Otherwise reload after deleting anything collidable.",
        {"type": "object",
         "properties": {
             "def": {"type": "string", "description": "DEF of the node to remove"},
             "defs": {"type": "array", "items": {"type": "string"},
                      "description": "several DEFs at once"},
             "settle_steps": {"type": "integer", "minimum": 0},
             "physics": {"type": "string", "enum": ["rebuild"],
                         "description": "'rebuild' = rebuild the Newton world after the "
                                        "delete so the phantoms are gone (W1.7)"},
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
        "light-mode session -- the harness DEFAULT since 2026-09-02, so unless "
        "the world was loaded with light=false (or a `tracking` object keeping "
        "grips) the tracker is NOT RUNNING: the answer carries "
        "tracking.enabled=false (NOT MEASURED, not 'nothing gripped') and the "
        "first such read puts one world.warning TRACKER_NOT_RUNNING on "
        "get_events. A contact read is always a weaker claim than proving the "
        "grasp geometrically (the part is airborne and tracks the gripper).",
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
    "sim_pause": (
        t_sim_pause,
        "Hold the engine PAUSED ACROSS CALLS, so the scene stops moving between "
        "your own requests. Without it the engine free-runs between HTTP calls "
        "(~88-112 ms of sim time per idle poll), so two reads are two different "
        "instants. The pause is LEASED -- default 30 s, cap 300 s -- and that "
        "deadline is a safety property, not a limitation to work around: a "
        "client that pauses and then dies would otherwise freeze the simulation "
        "with no way back but killing the engine. ⭐ sim_step STILL ADVANCES "
        "while held, and re-holds after: that is SINGLE-STEPPING, and the step "
        "response reports `paused` and `lease_remaining_ms`. Pausing again while "
        "held EXTENDS the lease rather than erroring. Release early with "
        "sim_resume, or simply let the lease expire. ⚠️ The step is exact on the "
        "SUPERVISOR clock and not on the engine clock (0-4 basic steps of "
        "overshoot) -- see sim_step. A held lease does NOT survive a load_world: "
        "the harness releases it and clears every armed break.",
        {"type": "object",
         "properties": {"lease_ms": {
             "type": "integer", "minimum": 1,
             "description": "how long to hold before the engine resumes itself. "
                            "Default 30000; silently clamped to [1000, 300000]. "
                            "The response's `lease_ms` is the EFFECTIVE value"}}},
    ),
    "sim_resume": (
        t_sim_resume,
        "Release the pause lease early and let the engine free-run again. "
        "Idempotent -- resuming a session that is not paused is not an error.",
        {"type": "object", "properties": {}},
    ),
    "sim_break": (
        t_sim_break,
        "⭐ ARM A BREAKPOINT ON THE EVENT STREAM. When a matching event is "
        "emitted the supervisor takes the pause lease, records "
        "{break_id, event, paused_at_sim_ms} and emits a `break.hit` event on "
        "get_events, so a poller learns WHY it stopped. Then read a frozen scene "
        "(get_scene_tree, get_contacts, get_robot_joints), single-step with "
        "sim_step, and sim_resume or let the lease expire. ⚠️ TWO HONEST LIMITS. "
        "(1) LATENCY is not sub-step in either regime, and the two regimes are "
        "two orders of magnitude apart. HELD (you called sim_pause first) and "
        "driven by sim_step, detection is per BASIC STEP: hold_latency_ms 0.0, "
        "hold_latency_steps 0, at most one basic step of engine time. "
        "FREE-RUNNING, the hold happens on the NEXT SUPERVISOR TICK after the "
        "event, and a tick is NOT a basic step -- measured from 8 ms to about "
        "600 ms of engine time depending on load, and on a 3-body world a whole "
        "ONE-SECOND drop fitted inside a single tick and no contact.began fired "
        "at all. So PAUSE FIRST, THEN STEP; free-running, a transient can be "
        "missed entirely. Every hit carries its own hold_latency_ms (supervisor "
        "clock, ~0 by construction) and hold_latency_engine_ms_max (the "
        "engine-clock width of that tick -- read THIS one). "
        "(2) LIGHT SESSIONS: light is the harness default and it silences "
        "contact.*, grip.* and joint.limit_hit (5 of the 11 event types), so a "
        "break armed on one of those in a light session is REFUSED with "
        "`400 BREAK_EVENT_TYPE_UNAVAILABLE` and the diagnostic code "
        "`event_type_silenced_in_light_mode` rather than armed and left unable "
        "to fire -- load the world with {\"light\": false} for a debugging "
        "session. The refusal is SCOPED: damage.impact / damage.state_transition "
        "survive light mode and still arm in the same session. A refused break "
        "is NOT armed. Call sim_breaks first to read this session's "
        "breakable_types / silenced_types. The lease cap applies here too, so a "
        "crashed agent cannot leave the engine held. Returns "
        "{break_id, types, filter, once, armed, hits, lease_ms, "
        "armed_at_sim_ms, last_hit_sim_ms, armed_types, diagnostics, ok}.",
        {"type": "object",
         "properties": {
             "types": {"type": "array", "items": {"type": "string"},
                       "description": "event types to break on. The breakable "
                                      "set is contact.began, contact.ended, "
                                      "joint.limit_hit, grip.acquired, "
                                      "grip.released, damage.impact, "
                                      "damage.state_transition -- minus "
                                      "whatever this session silences "
                                      "(sim_breaks reports both lists)"},
             "filter": {"type": "object",
                        "description": "narrow the match: {\"def\": \"BOX\"} "
                                       "(the subject), {\"counterpart\": "
                                       "\"PLATE\"} (the other side of a "
                                       "contact), {\"joint\": \"elbow\"}",
                        "properties": {
                            "def": {"type": "string"},
                            "counterpart": {"type": "string"},
                            "joint": {"type": "string"}}},
             "lease_ms": {"type": "integer", "minimum": 1,
                          "description": "pause lease taken when it fires "
                                         "(default 30000, cap 300000)"},
             "once": {"type": "boolean",
                      "description": "disarm after the first hit. DEFAULT TRUE "
                                     "-- the harness treats an absent `once` as "
                                     "true. Pass false for a break that keeps "
                                     "firing."},
         },
         "required": ["types"]},
    ),
    "sim_breaks": (
        t_sim_breaks,
        "List the breaks currently armed in this session (GET /sim/breaks): "
        "id, types, filter, lease and whether each has fired. ⭐ CALL IT BEFORE "
        "sim_break: it also reports `breakable_types` and `silenced_types` for "
        "THIS session (light mode, the harness default, silences contact.*, "
        "grip.* and joint.limit_hit), plus `break_hit` (the last break that "
        "froze the engine -- it survives the resume) and `paused`. Reading it "
        "first is how you avoid arming a break the harness will refuse.",
        {"type": "object", "properties": {}},
    ),
    "sim_break_clear": (
        t_sim_break_clear,
        "Disarm one armed break by its break_id. Both spellings ship -- "
        "DELETE /sim/break/<id> and its exact twin POST /sim/break/delete "
        "{\"break_id\"}, same 200 body and same 404 BREAK_NOT_FOUND -- and the "
        "response's `route` names which one this call used. An id that is not "
        "armed is an honest 404, not a missing route.",
        {"type": "object",
         "properties": {"break_id": {"type": "string",
                                     "description": "id returned by sim_break"}},
         "required": ["break_id"]},
    ),
    "robot_prompt": (
        t_robot_prompt,
        "⭐ TALK TO THE ROBOT. Posts an operator SENTENCE to a robot's OmniLink "
        "bridge (POST /prompt on 127.0.0.1:<port>) -- \"drive forward 1 m\", "
        "\"go home\", \"where did the package land?\". A deterministic parser "
        "interprets first (free); a model is called only on what the parser "
        "declines; and the safety gate vets whatever produced the frames. This "
        "and robot_tool are the two VETTED paths onto a robot. NOTE this is NOT "
        "the harness: the robot's world must already be running with its bridge "
        "controller. PORTS: 8765 by default; the Mavic drone serves 6090 and the "
        "langsoak Husky 8775. ⚠️ THE BRIDGE'S ENVELOPE IS PASSED THROUGH "
        "UNTOUCHED, including `refused_by_gate` and each action's {commanded, "
        "achieved, error, settled}. `error` THERE IS THE CONTROL ERROR (a float "
        "in metres or radians), NOT a failure indicator -- reading it as one "
        "once printed \"I could not stop: 4.3e-11\" after a textbook stop. Judge "
        "the result by `settled` and by `achieved` against `commanded`. ⚠️ A 401 "
        "`omnikey_required` or a 503 `omnilink_unavailable` is NOT a crash: "
        "OmniLink requires an OmniKey on EVERY plan including Free, and the "
        "envelope's `response` and `setup_url` say how to connect one "
        "(`python -m omnisim key`). Report that message to the user as it "
        "stands.",
        {"type": "object",
         "properties": {
             "text": {"type": "string",
                      "description": "the operator sentence, verbatim"},
             "port": {"type": "integer",
                      "description": "bridge port (default 8765; Mavic 6090, "
                                     "langsoak Husky 8775)"},
             "timeout_s": {"type": "number",
                           "description": "the bridge's budget for the turn "
                                          "(default 90, max 600) -- raise it "
                                          "for a long physical sequence"},
         },
         "required": ["text"]},
    ),
    "robot_tool": (
        t_robot_tool,
        "⭐ A TYPED TOOL CALL on a robot's OmniLink bridge (POST /tool): "
        "{\"tool\": \"drive_forward\", \"args\": {\"distance\": 1.0}}. Reach for "
        "this instead of the bare REST actuators: `/drive_forward`, `/turn`, "
        "`/set_velocity` and `/stop_robot` BYPASS the safety gate "
        "(packages/omnisim-bridges/GATE_COVERAGE.md) and this server never calls "
        "them, while `/tool` is vetted. A tool must be REGISTERED on the bridge "
        "to be checked at all -- an unregistered name comes back 503 "
        "`tool_not_registered` and nothing dispatches, which is a refusal, not a "
        "silent pass. Tool names come from the robot's own set (robot_prompt's "
        "`actions` name them; robot_state reports the robot's capabilities). ⚠️ "
        "TRANSPORT FIELDS NEVER TRAVEL AS ARGUMENTS: `id`, `tool` and `port` are "
        "stripped from `args` here, because the bridge dispatches everything "
        "beside `tool` as the argument map and a stray `id` reaches the safety "
        "gate as `unknown_arg` -- which is how the Mavic once refused a "
        "legitimate takeoff. PORTS: 8765 by default; the Mavic drone serves "
        "6090 and the langsoak Husky 8775. Same envelope rules as robot_prompt: "
        "the reply is passed through untouched (`refused_by_gate` and a 401 "
        "`omnikey_required` included), and `error` inside the result is the "
        "CONTROL ERROR in metres or radians, not a failure flag.",
        {"type": "object",
         "properties": {
             "tool": {"type": "string",
                      "description": "registered tool name, e.g. drive_forward, "
                                     "turn, set_gripper_width, takeoff"},
             "args": {"type": "object",
                      "description": "the tool's own parameters, e.g. "
                                     "{\"distance\": 1.0}. Magnitudes are "
                                     "railed by the gate, which does not know "
                                     "the arena -- a rail is 'nobody meant "
                                     "this', not a bound on the floor."},
             "port": {"type": "integer",
                      "description": "bridge port (default 8765; Mavic 6090, "
                                     "langsoak Husky 8775)"},
         },
         "required": ["tool"]},
    ),
    "robot_state": (
        t_robot_state,
        "Read a robot's own state from its OmniLink bridge (POST "
        "/get_robot_state): pose, wheel/joint state, fault, last tick, autonomy "
        "hold and whatever counters that robot keeps. A pure read -- nothing "
        "moves and no gate is involved. Ports as robot_prompt (8765 default, "
        "Mavic 6090, langsoak Husky 8775). This is the ROBOT's view through its "
        "bridge; harness_status and list_robots are the SIMULATOR's view of the "
        "same scene, and when the two disagree that disagreement is the finding.",
        {"type": "object",
         "properties": {"port": {
             "type": "integer",
             "description": "bridge port (default 8765; Mavic 6090, langsoak "
                            "Husky 8775)"}}},
    ),
    "robot_events": (
        t_robot_events,
        "⭐ WHAT THE ROBOT NOTICED, without asking it (GET /events on its "
        "OmniLink bridge): a cursor-paged ring of `motion.timed_out`, "
        "`motion.unsettled`, `joint.limit_hit`, `fault.*`, `gate.refused`, "
        "`contact.began`, and — when the bridge has a harness attached — "
        "`break.hit` and `damage.*` re-emitted with `source: \"harness\"`. "
        "Each event carries `seq` (the cursor unit), `type`, `sim_time`, "
        "`step`, `robot`, `source` and `detail`. "
        "Page it: pass the previous reply's `next_since` as `since`; the "
        "envelope is {events, next_since, dropped, total}. ⚠️ `dropped` "
        "NON-ZERO MEANS YOU LOST EVENTS -- the ring is bounded and in-process, "
        "so poll more often or raise `limit`; it is not a warning you may "
        "ignore, it is the count of what you will never see. ⚠️ THIS IS NOT "
        "get_events: that one is the HARNESS's stream on :6789 with its own "
        "two cursors, and a cursor from one ring is meaningless in the other. "
        "⚠️ The ring is in the bridge's process and does NOT survive a bridge "
        "restart or a world reload -- an empty ring after a restart means "
        "'nothing since boot', never 'nothing happened'. ⚠️ `contact.began` "
        "here is TOP-LEVEL SCOPE ONLY (the event says so): the supervisor's "
        "getContactPoints() is blind to a URDF robot's sub-links, so a "
        "gripper-finger contact is invisible to this detector. A pure read -- "
        "nothing moves, no gate is involved. PORTS: 8765 by default; the Mavic "
        "drone serves 6090 and the langsoak Husky 8775.",
        {"type": "object",
         "properties": {
             "since": {"type": "integer",
                       "description": "cursor: the previous reply's "
                                      "`next_since`. Omit for the whole ring."},
             "limit": {"type": "integer",
                       "description": "max events in this page"},
             "types": {"type": "string",
                       "description": "comma-separated filter, e.g. "
                                      "\"joint.limit_hit,motion.timed_out\" "
                                      "(a JSON array is accepted too)"},
             "port": {"type": "integer",
                      "description": "bridge port (default 8765; Mavic 6090, "
                                     "langsoak Husky 8775)"},
         }},
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
    print("Four of them (robot_prompt, robot_tool, robot_state, robot_events) do NOT")
    print("go to the harness: they reach a robot's OmniLink bridge on 127.0.0.1:%d"
          % DEFAULT_BRIDGE_PORT)
    print("(Mavic 6090, langsoak Husky 8775), so the robot's world must be running.")
    print("Only /prompt and /tool are vetted by the safety gate; this server never")
    print("calls the ungated REST verbs. An OmniKey is required on every plan.")
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
