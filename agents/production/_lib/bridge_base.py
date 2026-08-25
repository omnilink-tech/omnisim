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
  (Husky, OmniQuad, Mavic). Endpoints:
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
            # sim time, not wall clock -- an agent correlates this with /sim/state
            return {"halted_at": self.robot.getTime()}

        @action("drive_forward")
        def _drive(self, body):
            commanded = float(body.get("distance", 0.0))
            start = self.measure_pose()
            ...                                  # drive, and WAIT for it to settle
            achieved = self.distance_from(start)
            # ⚠ NEVER return the commanded argument under a measurement-shaped key.
            # `{"distance_m": commanded}` is the single most expensive mistake a
            # bridge can make: the agent has no independent view of the world, so it
            # adopts that number as fact and reports it confidently. Measured in this
            # repo -- a `turn` that delivered 56.7% of the commanded angle while
            # echoing the request read as a perfect turn to every agent that called
            # it (docs/developer/tool-design-for-agents.md). Always report both, and
            # use None -- never a plausible number -- for anything you did not
            # measure.
            return {"commanded_m": commanded,
                    "achieved_m": achieved,          # None if unmeasurable
                    "error_m": None if achieved is None else achieved - commanded,
                    "settled": True}

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

from .http_security import (
    RequestError,
    allowed_origins,
    check_authorization,
    check_protocol_version,
    checked_origin,
    configured_token,
    error_envelope,
    read_json,
    validate_bind,
    WIRE_SERVICE,
    WIRE_VERSION,
)


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

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 6060,
        token: Optional[str] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.token = configured_token(token)
        validate_bind(self.host, self.token)
        self._trusted_origins = allowed_origins()
        self._action_lock = threading.RLock()
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

            def _cors(self, origin: Optional[str]):
                if origin:
                    self.send_header("Access-Control-Allow-Origin", origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header(
                    "Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token"
                )

            def _send(self, status: int, payload: Any, origin: Optional[str] = None) -> None:
                body = json.dumps(payload, default=str).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("X-OmniSim-Wire", WIRE_VERSION)
                self.send_header("X-OmniSim-Service", WIRE_SERVICE)
                self._cors(origin)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _guard(self) -> Optional[str]:
                check_protocol_version(self.headers)
                origin = checked_origin(self.headers, outer._trusted_origins)
                check_authorization(self.headers, outer.token)
                return origin

            def _failure(self, exc: RequestError, origin: Optional[str] = None) -> None:
                self._send(exc.status, error_envelope(exc.code, exc.message, exc.details), origin)

            def do_OPTIONS(self):  # noqa: N802
                try:
                    origin = checked_origin(self.headers, outer._trusted_origins)
                    self.send_response(204)
                    self._cors(origin)
                    self.end_headers()
                except RequestError as exc:
                    self._failure(exc)

            def do_GET(self):  # noqa: N802
                origin: Optional[str] = None
                try:
                    origin = self._guard()
                    path = self.path.split("?", 1)[0].rstrip("/") or "/"
                    if path == "/healthz":
                        self._send(200, {"ok": True, "uptime_s": time.time() - outer._started_at}, origin)
                        return
                    if path == "/protocol":
                        self._send(200, {
                            "ok": True,
                            "omnisim_wire": WIRE_VERSION,
                            "service": WIRE_SERVICE,
                            "service_versions": {WIRE_SERVICE: WIRE_VERSION},
                            "instance": {"name": outer.__class__.__name__},
                            "extensions": [],
                        }, origin)
                        return
                    if path == "/state":
                        self._send(200, outer.get_state(), origin)
                        return
                    if path == "/capabilities":
                        self._send(200, outer.get_capabilities(), origin)
                        return
                    custom = outer.extra_get(path)
                    if custom is not None:
                        status, payload = custom
                        self._send(status, payload, origin)
                        return
                    self._send(404, error_envelope("not_found", "Endpoint not found.", {"path": path}), origin)
                except RequestError as exc:
                    self._failure(exc, origin)
                except Exception as exc:
                    self._send(
                        500,
                        error_envelope("internal_error", "Bridge read failed.", {"type": type(exc).__name__}),
                        origin,
                    )

            def do_POST(self):  # noqa: N802
                origin: Optional[str] = None
                try:
                    origin = self._guard()
                    body = read_json(self, allow_empty=False)
                    path = self.path.split("?", 1)[0].rstrip("/") or "/"
                    if path == "/action":
                        action_value = body.get("action")
                        if not isinstance(action_value, str) or not action_value.strip():
                            raise RequestError(
                                400,
                                "missing_field",
                                "A non-empty 'action' field is required.",
                                {"available_actions": sorted(outer._actions)},
                            )
                        name = action_value.strip()
                        handler = outer._actions.get(name)
                        if handler is None:
                            raise RequestError(
                                404,
                                "invalid_action",
                                f"Unknown action {name!r}.",
                                {"available_actions": sorted(outer._actions)},
                            )
                        try:
                            if name in ("stop", "stop_robot", "emergency_stop"):
                                result = handler(body)
                            else:
                                with outer._action_lock:
                                    result = handler(body)
                        except Exception as exc:
                            self._send(
                                500,
                                error_envelope(
                                    "internal_error",
                                    f"Action {name!r} failed.",
                                    {"type": exc.__class__.__name__},
                                ),
                                origin,
                            )
                            return
                        payload = {"status": "ok", "action": name}
                        if isinstance(result, dict):
                            payload.update(result)
                            if result.get("error"):
                                payload["status"] = "err"
                        elif result is not None:
                            payload["result"] = result
                        self._send(200, payload, origin)
                        return
                    custom = outer.extra_post(path, body)
                    if custom is not None:
                        status, payload = custom
                        self._send(status, payload, origin)
                        return
                    self._send(404, error_envelope("not_found", "Endpoint not found.", {"path": path}), origin)
                except RequestError as exc:
                    self._failure(exc, origin)

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
