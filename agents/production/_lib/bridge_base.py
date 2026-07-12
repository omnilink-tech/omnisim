# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""OmniSim-side bridge SDK for OmniLink agents.

The OmniSim bridge controller (e.g. ``husky_omnilink_bridge.py``) is a
supervisor-mode OmniSim controller that owns the robot and exposes an
HTTP surface. Every existing bridge re-implements the same scaffolding:

* A ``BaseHTTPRequestHandler`` subclass with ``_json``/``_ok``/``_err``
  helpers and CORS.
* A long ``do_POST`` chain dispatching on ``body["action"]``.
* A ``ThreadingHTTPServer`` started in a daemon thread.
* ``GET /state`` / ``GET /capabilities`` / ``GET /healthz`` boilerplate.

This module hoists that into ``OmniSimBridgeServer`` plus an ``@action``
decorator. New bridges become substantially shorter; existing bridges
can adopt it incrementally.

Choosing between this and ``omnilink.OmniLinkHTTPBridge``
--------------------------------------------------------

OmniLink ships **two** bridge primitives. Pick by transport style:

* :class:`OmniSimBridgeServer` (this module) — **action-dispatch**
  style. Tools post structured ``{"action": "name", ...args}`` JSON;
  handlers return a dict. Standard for OmniSim robot bridges
  (Husky, Spot, Mavic). Endpoints:
  ``GET /state``, ``GET /capabilities``, ``GET /healthz``,
  ``POST /action``.

* :class:`omnilink.OmniLinkHTTPBridge` (in the OmniLink Python
  library, in the separate OmniLink repo) — **text-command** style
  driven by :class:`omnilink.OmniLinkEngine`. Clients post plain-English
  strings; the engine matches templates like ``"launch [vehicle]"`` and
  dispatches to handler functions. Endpoints:
  ``GET /context``, ``GET /feedback``, ``POST /command``,
  ``POST /inline-code``.

For interop, ``agents/production/_lib`` re-exports
:class:`OmniLinkHTTPBridge` and :class:`OmniLinkEngine` when the
sibling olink checkout is reachable, so ``from _lib import ...`` works
for either style.

Example
-------

::

    class HuskyBridge(OmniSimBridgeServer):
        def get_state(self):
            return {"x": self.x, "y": self.y, "yaw": self.yaw}

        def get_capabilities(self):
            return {"world_title": "husky_maze", "map_available": True}

        @action("stop")
        def _stop(self, body):
            self.target_velocity = 0.0
            return {"halted_at": time.time()}

        @action("drive_forward")
        def _drive(self, body):
            distance = float(body.get("distance", 0.0))
            ...
            return {"distance_m": distance}

    bridge = HuskyBridge(host="127.0.0.1", port=6070)
    bridge.serve_in_background()
    while robot.step(time_step) != -1:
        bridge.tick()  # optional per-step hook

The ``@action`` decorator marks a method as a handler for a specific
action verb. The class collects all decorated methods and builds the
dispatch table at instantiation time, so the actual HTTP path
(``POST /action``) handler is generic.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Optional


# ---------------------------------------------------------------------------
# @action decorator

_ACTION_ATTR = "_omnilink_action_name"


def action(name: str) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Mark a method as the handler for a specific ``action`` verb.

    The decorated method receives the JSON request body as its sole
    argument and should return a JSON-serialisable dict (or raise to
    surface a 500). The framework wraps the return in
    ``{"status": "ok", ...result}`` and adds standard headers.
    """

    def decorate(fn: Callable[..., Any]) -> Callable[..., Any]:
        setattr(fn, _ACTION_ATTR, name)
        return fn

    return decorate


# ---------------------------------------------------------------------------
# Server


class OmniSimBridgeServer:
    """Subclassable HTTP bridge for an OmniSim robot supervisor.

    Subclasses implement :meth:`get_state`, :meth:`get_capabilities`,
    and any number of ``@action``-decorated methods. The framework
    handles HTTP plumbing (CORS, error formatting, JSON parsing).

    Standard endpoints exposed on every bridge:

    * ``GET  /healthz``       → ``{"ok": true, "uptime_s": ...}``
    * ``GET  /state``         → result of ``get_state()``
    * ``GET  /capabilities``  → result of ``get_capabilities()``
    * ``POST /action``        → dispatched on ``body["action"]``

    Custom endpoints can be added by overriding :meth:`extra_get` or
    :meth:`extra_post` — they receive the parsed path and JSON body and
    return ``(status_code, payload_dict)`` or ``None`` to fall through.
    """

    def __init__(self, *, host: str = "127.0.0.1", port: int = 6060) -> None:
        self.host = host
        self.port = port
        self._started_at = time.time()
        self._actions = self._collect_actions()
        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    # --- subclass hooks ------------------------------------------------

    def get_state(self) -> Dict[str, Any]:
        """Return the bridge's live state snapshot. Override."""
        return {}

    def get_capabilities(self) -> Dict[str, Any]:
        """Return the bridge's capabilities/world metadata. Override."""
        return {}

    def extra_get(
        self, path: str
    ) -> Optional[tuple]:  # pragma: no cover - subclass surface
        """Custom ``GET`` paths. Return ``(status, payload)`` or ``None``."""
        return None

    def extra_post(
        self, path: str, body: Dict[str, Any]
    ) -> Optional[tuple]:  # pragma: no cover
        """Custom ``POST`` paths. Return ``(status, payload)`` or ``None``."""
        return None

    def tick(self) -> None:  # pragma: no cover - subclass surface
        """Optional per-physics-step hook. Default is no-op."""

    # --- lifecycle -----------------------------------------------------

    def serve_in_background(self) -> int:
        """Start the HTTP server in a daemon thread. Returns the bound port."""
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: ARG002
                pass

            def _cors(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")

            def _send(self, status: int, payload: Any) -> None:
                body = json.dumps(payload, default=str).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self._cors()
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self) -> Dict[str, Any]:
                length = int(self.headers.get("Content-Length", 0))
                if length <= 0:
                    return {}
                raw = self.rfile.read(length)
                try:
                    return json.loads(raw.decode("utf-8"))
                except json.JSONDecodeError as exc:
                    raise ValueError(f"invalid JSON: {exc}") from exc

            def do_OPTIONS(self):  # noqa: N802
                self.send_response(200)
                self._cors()
                self.end_headers()

            def do_GET(self):  # noqa: N802
                if self.path == "/healthz":
                    self._send(200, {"ok": True, "uptime_s": time.time() - outer._started_at})
                    return
                if self.path == "/state":
                    try:
                        self._send(200, outer.get_state())
                    except Exception as exc:
                        self._send(500, {"error": f"get_state failed: {exc}"})
                    return
                if self.path == "/capabilities":
                    try:
                        self._send(200, outer.get_capabilities())
                    except Exception as exc:
                        self._send(500, {"error": f"get_capabilities failed: {exc}"})
                    return
                custom = outer.extra_get(self.path)
                if custom is not None:
                    status, payload = custom
                    self._send(status, payload)
                    return
                self._send(404, {"error": "not found", "path": self.path})

            def do_POST(self):  # noqa: N802
                try:
                    body = self._read_json()
                except ValueError as exc:
                    self._send(400, {"error": str(exc)})
                    return
                if self.path == "/action":
                    name = str(body.get("action") or "").strip()
                    if not name:
                        self._send(
                            400,
                            {
                                "error": "'action' field required",
                                "available_actions": sorted(outer._actions.keys()),
                            },
                        )
                        return
                    handler = outer._actions.get(name)
                    if handler is None:
                        self._send(
                            404,
                            {
                                "error": f"unknown action: {name!r}",
                                "available_actions": sorted(outer._actions.keys()),
                            },
                        )
                        return
                    try:
                        result = handler(body)
                    except Exception as exc:
                        self._send(
                            500,
                            {
                                "error": f"action {name!r} crashed: "
                                f"{exc.__class__.__name__}: {exc}",
                            },
                        )
                        return
                    payload = {"status": "ok", "action": name}
                    if isinstance(result, dict):
                        payload.update(result)
                    elif result is not None:
                        payload["result"] = result
                    self._send(200, payload)
                    return
                custom = outer.extra_post(self.path, body)
                if custom is not None:
                    status, payload = custom
                    self._send(status, payload)
                    return
                self._send(404, {"error": "not found", "path": self.path})

        try:
            server = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as exc:
            raise RuntimeError(
                f"OmniSim bridge cannot bind {self.host}:{self.port} ({exc}). "
                "Pick a different port or kill the previous bridge process."
            ) from exc
        self._server = server
        self._thread = threading.Thread(target=server.serve_forever, daemon=True)
        self._thread.start()
        bound = server.server_address[1]
        self.port = bound
        return bound

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None

    # --- introspection -------------------------------------------------

    def list_actions(self) -> list:
        return sorted(self._actions.keys())

    # --- internal ------------------------------------------------------

    def _collect_actions(self) -> Dict[str, Callable[[Dict[str, Any]], Any]]:
        table: Dict[str, Callable[[Dict[str, Any]], Any]] = {}
        for attr_name in dir(self):
            if attr_name.startswith("__"):
                continue
            attr = getattr(self, attr_name, None)
            if not callable(attr):
                continue
            name = getattr(attr, _ACTION_ATTR, None)
            if name:
                table[name] = attr  # bound method
        return table
