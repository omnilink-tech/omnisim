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

"""omnitug500_courier — OmniLink warehouse-courier bridge for the OmniTug 500.

Runs on the OMNITUG500's scanner sidecar (the Newton physics body, supervisor TRUE)
in omnitug500_courier.omniworld. Drives the rover through the warehouse aisle grid to
named pickup bays and dropoff docks, loads/unloads packages on its deck, and
exposes that as a natural-language action surface:

  * Robot window  (right-click the rover -> Show Robot Window): the omnilink_chat
    side panel. Type "take the package from bay B to dock 2" and watch it run.
    Language control requires an OmniKey; when connected it
    routes through the OmniLink agent (courier_tools).
  * HTTP on 127.0.0.1:<port> (default 8765): the same surface for the productized
    agent under agents/production/omnitug500_warehouse/ and for scripting.

        POST /list_robots | /capabilities   -> robot + station + package catalogue
        POST /get_robot_state               -> live pose / mode / carrying / queue
        POST /goto_station   {station}
        POST /pick_package   {station?, package?}
        POST /deliver_package{station, package?}
        POST /run_route      {steps:[{action,station,package?}]}
        POST /stop  |  /reset
        POST /prompt {text}                 -> natural language (OmniKey required)
        POST /tool   {tool, ...}            -> OmniLink platform tool callback
        GET  /healthz

Args:  --layout <path-to-layout.json>  --port <n>  [--name <agent-id>]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

from omnisim import Supervisor

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)
# Reuse the canonical OmniLink relay that the demo chat bridges use.
_DEMO_CTRL = os.path.abspath(os.path.join(
    _THIS_DIR, "..", "..", "..", "..", "..", "samples", "demos", "controllers"))
if _DEMO_CTRL not in sys.path:
    sys.path.insert(0, _DEMO_CTRL)

from courier_bridge import CourierBridge          # noqa: E402
from courier_tools import build_courier_tools, build_courier_main_task  # noqa: E402

# THE ONE /tool implementation. Do not copy it back in here: this file's own
# handler was ungated for its whole life precisely because it was a sixth
# copy nobody audited.
from omnisim_bridges.bridge_base import serve_tool  # noqa: E402

# The deterministic interpreter, on both live /prompt paths. Order: access
# check -> parser -> model -> gate. The OmniKey check runs in FRONT of the
# parser on both, and the gate is inside `route.execute`, so a
# parser-produced frame is vetted on the same "mobile" rail serve_tool
# uses here. ⚠️ This is emphatically NOT the keyless fallback that
# `courier_intent.CourierIntent` advertised and that was deleted on
# 2026-09-22: no relay, no parse.
try:
    from omnisim_bridges.route import (  # noqa: E402
        parser_first_window as _shared_parser_window,
        reply_payload as _reply_payload,
        short_circuit as _shared_short_circuit,
    )
except ImportError:                                # pragma: no cover
    _shared_parser_window = None
    _reply_payload = None
    _shared_short_circuit = None

try:
    from _omnilink_relay import (OmniLinkRelay, Tool,  # noqa: E402
                                 is_enabled as omnilink_enabled, get_omni_key)
except Exception:
    OmniLinkRelay = None        # type: ignore[assignment]
    Tool = None                 # type: ignore[assignment]
    def omnilink_enabled() -> bool: return False
    def get_omni_key() -> str: return ""


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--layout", default=None)
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--name", default="omnitug500_courier")
    args, _ = p.parse_known_args()
    return args


def _load_layout(path: Optional[str]) -> dict:
    if not path:
        path = os.path.join(_THIS_DIR, "..", "..", "worlds", "omnitug500_courier_layout.json")
    if not os.path.isabs(path):
        # controllerArgs paths are relative to the world dir; resolve from there.
        cand = os.path.normpath(os.path.join(_THIS_DIR, "..", "..", "worlds",
                                             os.path.basename(path)))
        path = cand if os.path.exists(cand) else os.path.normpath(
            os.path.join(_THIS_DIR, path))
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ── HTTP ──────────────────────────────────────────────────────────────
def make_handler(bridge: CourierBridge, relay: Any):
    class _H(BaseHTTPRequestHandler):
        def log_message(self, *a):  # silence
            return

        def _json(self, code, obj):
            data = json.dumps(obj, default=str).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _body(self) -> Dict[str, Any]:
            n = int(self.headers.get("Content-Length", "0"))
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8") or "{}")
            except Exception:
                return {}

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self):
            p = self.path.rstrip("/") or "/"
            if p == "/healthz":
                return self._json(200, {"ok": True})
            if p in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if p in ("/capabilities", "/list_robots", "/stations"):
                return self._json(200, bridge.capabilities())
            if p == "/usage":
                return self._json(200, {"enabled": relay is not None,
                                        "latest": relay.latest_usage() if relay else None})
            return self._json(404, {"error": "not_found"})

        def do_POST(self):
            body = self._body()
            p = self.path.rstrip("/") or "/"
            if p in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if p in ("/capabilities", "/list_robots", "/stations"):
                return self._json(200, bridge.capabilities())
            if p == "/goto_station":
                return self._json(200, bridge.act_goto(body.get("station")))
            if p == "/pick_package":
                return self._json(200, bridge.act_pick(body.get("station"), body.get("package")))
            if p == "/deliver_package":
                return self._json(200, bridge.act_deliver(body.get("station"), body.get("package")))
            if p == "/run_route":
                return self._json(200, bridge.act_run_route(body.get("steps") or []))
            if p == "/stop":
                return self._json(200, bridge.act_stop())
            if p == "/reset":
                return self._json(200, bridge.act_reset())
            if p == "/prompt":
                text = (body.get("text") or "").strip()
                if not text:
                    return self._json(400, {"error": "text required"})
                if relay is not None:
                    # ── PARSER FIRST ──────────────────────────────────
                    # Only WITH a relay. The keyless branch below is the
                    # access check, and nothing above it interprets
                    # anything. A non-None result has been through the
                    # gate inside route.execute.
                    _early = (_shared_short_circuit(bridge, text, "mobile")
                              if _shared_short_circuit is not None else None)
                    if _early is not None and _reply_payload is not None:
                        return self._json(200, _reply_payload(
                            _early.get("agent", ""),
                            _early.get("tools") or [], via="parser"))
                    return self._json(200, relay.dispatch_sync(text))
                from omnisim_bridges.access import connection_error
                error = connection_error()
                return self._json(401 if error["error"] == "omnikey_required" else 503, error)
            if p == "/tool":
                name = (body.get("tool") or "").strip()
                # ⚠️ THIS HANDLER WAS COMPLETELY UNGATED until 2026-09-22.
                # It reached `relay.tools[name].dispatch(body)` with no
                # safety check of any kind, and this controller does not
                # import `gate` at all -- it was the one /tool surface the
                # 2026-09-21 audit did not find, because it is not one of
                # the four demo bridges. serve_tool is the single shared
                # implementation: strip the transport fields, refuse an
                # unregistered tool, vet on the mobile rail, fail closed.
                code, payload = serve_tool(
                    name, body,
                    lambda args: relay.tools[name].dispatch(args),
                    surface="mobile",
                    registered=(relay is not None
                                and name in getattr(relay, "tools", {})))
                return self._json(code, payload)
            return self._json(404, {"error": "not_found", "path": p})
    return _H


def start_http(bridge, relay, port: int) -> ThreadingHTTPServer:
    srv = ThreadingHTTPServer(("127.0.0.1", port), make_handler(bridge, relay))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[omnitug500_courier] HTTP on http://127.0.0.1:{port}")
    return srv


# ── OmniLink relay ────────────────────────────────────────────────────
def setup_relay(bridge: CourierBridge, agent_id: str, http_port: int) -> Optional[Any]:
    if OmniLinkRelay is None or not omnilink_enabled():
        return None
    try:
        agent_name = f"OmniSim-{agent_id}"
        tools = build_courier_tools(bridge, Tool)
        main_task = build_courier_main_task(bridge)
        # surface="mobile": the relay hands it to gate.register_tools(), so
        # every courier tool is registered against the ground rails instead
        # of being registered surface-less and judged on the strictest set.
        relay = OmniLinkRelay(omni_key=get_omni_key(), agent_name=agent_name,
                              main_task=main_task, tools=tools,
                              surface="mobile")
        try:
            from _omnilink_relay import profile_sync
            if profile_sync.is_enabled():
                profile_sync.ensure_profile(
                    client=relay._client, agent_name=agent_name, main_task=main_task,
                    tool_defs=[t.to_definition() for t in tools], engine=relay.engine,
                    tool_callback_url=f"http://127.0.0.1:{http_port}/tool")
        except Exception as e:
            print(f"[omnitug500_courier] profile sync skipped: {e}")
        print(f"[omnitug500_courier] OmniLink relay ON (agent='{agent_name}')")
        return relay
    except Exception as e:
        print(f"[omnitug500_courier] OmniLink relay setup failed: {e}")
        return None


# ── WWI (robot-window chat) ───────────────────────────────────────────
def push_configure(bridge: CourierBridge, relay: Any) -> None:
    agent_label = (f"OmniLink relay ({os.environ.get('OMNILINK_ENGINE', 'g1-engine')})"
                   if relay is not None else "OmniLink connection required")
    from omnisim_bridges.access import chat_config
    cfg = {
        **chat_config(relay),
        "robot": "OmniTug 500 Courier",
        "robot_class": "warehouse AGV",
        "agent": agent_label,
        "suggestions": [
            "take the package from bay B to dock 2",
            "pick up the red package",
            "collect from bay A and bay C, deliver to dock 3",
            "go to bay E",
            "return to the charging dock",
            "status",
        ],
    }
    bridge.queue_window("configure:" + json.dumps(cfg))
    bridge.queue_window("status:connected")
    bridge.window_configured = True


def on_relay_event(bridge: CourierBridge, kind: str, payload: Dict[str, Any]) -> None:
    if kind == "status":
        bridge.queue_window(f"status:{payload.get('state', 'idle')}")
    elif kind == "tool":
        bridge.queue_window(
            f"tool:{payload.get('name', '?')}:{payload.get('status', 'ok')}:{payload.get('summary', '')}")
    elif kind == "agent":
        bridge.queue_window("agent:" + str(payload.get("text", "")))
    elif kind == "usage":
        bridge.queue_window("usage:" + json.dumps(payload, default=str))
    elif kind == "error":
        bridge.queue_window("error:" + str(payload.get("text", "")))


def _parser_first_window(bridge: CourierBridge, relay: Any, text: str,
                         to_model: Any, spawn: Any = None) -> bool:
    """Parser-first on the robot-window path. True = the turn is taken.

    ⚠️ handle_wwi is drained on the SIM THREAD, so the shared helper
    decides here (pure regex) and executes on a worker.

    ⚠️ The caller checks `relay is None` FIRST and refuses. This is never
    reached without an OmniKey, and must never be made reachable without
    one -- see the 2026-09-22 access policy.
    """
    if _shared_parser_window is None or relay is None:
        return False
    try:
        return bool(_shared_parser_window(
            bridge, text, "mobile", bridge.queue_window, to_model,
            spawn=spawn))
    except Exception as exc:               # never take the demo down
        print(f"[omnitug500_courier] parser-first skipped: {exc!r}", flush=True)
        return False


def handle_wwi(bridge: CourierBridge, relay: Any, msg: str) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(bridge, relay)
        return
    if msg.startswith("stop"):
        bridge.act_stop()
        bridge.queue_window("agent:Stopped and cleared the queue.")
        bridge.queue_window("tool:stop_rover:ok:halted")
        bridge.queue_window("status:idle")
        return
    if msg.startswith("prompt:"):
        text = msg[len("prompt:"):]
        if relay is not None:
            def _to_model() -> None:
                relay.dispatch_async(
                    text, lambda k, p: on_relay_event(bridge, k, p))

            # PARSER FIRST, same order as HTTP: the keyless branch below
            # is the access check and nothing above it interprets.
            if _parser_first_window(bridge, relay, text, _to_model):
                return
            _to_model()
            return
        from omnisim_bridges.access import reject_window_prompt
        reject_window_prompt(bridge)
        return
    bridge.queue_window("system:Unknown window message: " + msg[:160])


# ── Main ──────────────────────────────────────────────────────────────
def main() -> int:
    args = _parse_args()
    layout = _load_layout(args.layout)
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    bridge = CourierBridge(robot, layout, ts)
    relay = setup_relay(bridge, args.name, args.port)
    # ⚠️ NOT "local". There is no local mode: with no OmniKey the chat
    # surface refuses (401 omnikey_required) and only the typed HTTP verbs
    # and Stop remain. The regex router that used to make "local" true --
    # courier_intent.CourierIntent -- was constructed and threaded through
    # this file without ever being called, and was deleted on 2026-09-22
    # with the other four. It was the worst of the five to leave lying
    # around: this controller never imports `gate`, so re-wiring it would
    # have produced a path that was keyless AND ungated.
    _link = ("OmniLink connected" if relay else
             "no OmniKey: chat disabled, typed HTTP verbs and Stop only")
    start_http(bridge, relay, args.port)
    print(f"[omnitug500_courier] ready: {len(bridge.station_names('pickup'))} bays, "
          f"{len(bridge.station_names('dropoff'))} docks, "
          f"{len(bridge.packages)} packages ({_link})")

    while robot.step(ts) != -1:
        while True:
            m = robot.wwiReceiveText()
            if not m:
                break
            try:
                handle_wwi(bridge, relay, m)
            except Exception as e:
                bridge.queue_window(f"error:bridge_exception: {e!r}")
        with bridge.lock:
            outbox = bridge.window_outbox
            bridge.window_outbox = []
        for line in outbox:
            try:
                robot.wwiSendText(line)
            except Exception:
                pass
        try:
            bridge.tick()
        except Exception as e:
            import traceback
            print("[omnitug500_courier] tick error:\n" + traceback.format_exc())
            bridge.fault = repr(e)
    return 0


if __name__ == "__main__":
    sys.exit(main())
