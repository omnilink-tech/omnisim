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
    Offline it uses the local regex router (courier_intent); with OMNI_KEY set it
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
        POST /prompt {text}                 -> natural language (regex or OmniLink)
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
from courier_intent import CourierIntent          # noqa: E402
from courier_tools import build_courier_tools, build_courier_main_task  # noqa: E402

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
def make_handler(bridge: CourierBridge, intent: CourierIntent, relay: Any):
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
                    return self._json(200, relay.dispatch_sync(text))
                res = intent.dispatch(text)
                return self._json(200, {
                    "response": res["agent"],
                    "actions": [{"tool": t[0], "result": t[1], "summary": t[2]}
                                for t in res["tools"]],
                })
            if p == "/tool":
                name = (body.pop("tool", None) or "").strip()
                if relay is None or name not in getattr(relay, "tools", {}):
                    return self._json(503, {"status": "err", "tool": name,
                                            "error": "tool_not_registered"})
                try:
                    return self._json(200, {"status": "ok", "tool": name,
                                            "result": relay.tools[name].dispatch(body)})
                except Exception as e:
                    return self._json(500, {"status": "err", "tool": name, "error": repr(e)})
            return self._json(404, {"error": "not_found", "path": p})
    return _H


def start_http(bridge, intent, relay, port: int) -> ThreadingHTTPServer:
    srv = ThreadingHTTPServer(("127.0.0.1", port), make_handler(bridge, intent, relay))
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
        relay = OmniLinkRelay(omni_key=get_omni_key(), agent_name=agent_name,
                              main_task=main_task, tools=tools)
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
                   if relay is not None else "local intent (regex)")
    cfg = {
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


def handle_wwi(bridge: CourierBridge, intent: CourierIntent, relay: Any, msg: str) -> None:
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
            relay.dispatch_async(text, lambda k, p: on_relay_event(bridge, k, p))
            return
        bridge.queue_window("status:thinking")
        res = intent.dispatch(text)
        for (tool, status, summary) in res["tools"]:
            bridge.queue_window(f"tool:{tool}:{status}:{summary}")
        bridge.queue_window("agent:" + res["agent"])
        bridge.queue_window("status:idle")
        return
    bridge.queue_window("system:Unknown window message: " + msg[:160])


# ── Main ──────────────────────────────────────────────────────────────
def main() -> int:
    args = _parse_args()
    layout = _load_layout(args.layout)
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    bridge = CourierBridge(robot, layout, ts)
    intent = CourierIntent(bridge)
    relay = setup_relay(bridge, args.name, args.port)
    start_http(bridge, intent, relay, args.port)
    print(f"[omnitug500_courier] ready: {len(bridge.station_names('pickup'))} bays, "
          f"{len(bridge.station_names('dropoff'))} docks, "
          f"{len(bridge.packages)} packages ({'OmniLink' if relay else 'local'})")

    while robot.step(ts) != -1:
        while True:
            m = robot.wwiReceiveText()
            if not m:
                break
            try:
                handle_wwi(bridge, intent, relay, m)
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
