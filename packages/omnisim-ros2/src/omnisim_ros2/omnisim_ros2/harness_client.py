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

"""Stdlib-only HTTP client for the OmniSim World Harness and per-robot bridges.

Deliberately has no ROS dependency, so it is unit-testable without a ROS
environment and reusable outside one. It follows the conventions already proven
in ``packages/omnisim-mcp``:

* a non-2xx response is **returned**, not raised, because the harness puts its
  structured diagnostics in the body of a 4xx/5xx and an exception throws them
  away;
* only a transport failure raises (``HarnessUnreachable``), because that is the
  one case where retrying or restarting the harness is the right response.

Harness error bodies are not uniform (``PROTOCOL.md`` §16): endpoints that
forward caller arguments to the supervisor answer
``{"ok": false, "error": ..., "code": ...}``, while harness-side validation
answers a bare ``{"error": ...}``. :class:`HarnessResponse` normalises the read
so callers do not have to care which they got.

CONNECTION REUSE
----------------
This used to call ``urllib.request.urlopen``, which opens a **new TCP
connection for every request** and gives the caller no way to reuse one. That
is what made publish rates expensive: a socket then sits in ``TIME_WAIT`` for
120 s on Windows against a 16,384-port ephemeral range, and a ~50 Hz bringup
measured 17,487 sockets in ``TIME_WAIT`` before ``connect()`` started failing
with ``WinError 10048``.

The transport is now ``http.client`` with one pooled connection per client, so
a node costs one connection instead of one per tick. ⚠ The server has to agree:
a peer that speaks HTTP/1.0 closes the socket anyway and the pool degrades to
the old behaviour automatically (``resp.will_close``). ``omnilink_mobile_bridge``
sets ``protocol_version = "HTTP/1.1"``; **the World Harness does not yet**, so
harness-facing nodes still pay a connection per request.
"""

from __future__ import annotations

import http.client
import json
import os
import threading
import urllib.parse
from dataclasses import dataclass, field
from typing import Any

DEFAULT_HARNESS_URL = os.environ.get("OMNISIM_HARNESS_URL", "http://127.0.0.1:6789")
DEFAULT_TIMEOUT_S = float(os.environ.get("OMNISIM_ROS2_TIMEOUT", "30"))
# Connection reuse is the difference between one TCP connection per node and
# one per REQUEST. Set OMNISIM_ROS2_KEEPALIVE=0 to go back to a fresh
# connection every time (the pre-2026-08-17 behaviour) for an A/B.
KEEP_ALIVE_DEFAULT = os.environ.get("OMNISIM_ROS2_KEEPALIVE", "1") not in ("0", "false", "off")


class HarnessUnreachable(RuntimeError):
    """The harness could not be contacted at all (connection refused, DNS, timeout)."""


@dataclass
class HarnessResponse:
    """One harness reply, with the HTTP status kept alongside the parsed body."""

    status: int
    body: dict[str, Any] = field(default_factory=dict)
    raw: bytes = b""

    @property
    def ok(self) -> bool:
        """True when the harness both answered 2xx and did not set ``ok: false``.

        The harness sets ``ok: false`` on some coded errors and omits ``ok``
        entirely on success, so ``body.get("ok", True)`` is the correct read —
        a missing key must not be treated as failure.
        """
        return 200 <= self.status < 300 and self.body.get("ok", True) is not False

    @property
    def error(self) -> str:
        """Best available human-readable error text, or '' when the call succeeded."""
        if self.ok:
            return ""
        msg = self.body.get("error") or self.body.get("message")
        if not msg:
            msg = f"HTTP {self.status}"
        code = self.body.get("code")
        return f"[{code}] {msg}" if code else str(msg)

    @property
    def code(self) -> str:
        """The harness's structured error code, or '' when it did not supply one."""
        return str(self.body.get("code") or "")


class HarnessClient:
    """Thin request layer over one harness (or one per-robot bridge) base URL."""

    def __init__(
        self,
        base_url: str = DEFAULT_HARNESS_URL,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        keep_alive: bool = KEEP_ALIVE_DEFAULT,
    ):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s
        self.keep_alive = keep_alive
        parts = urllib.parse.urlsplit(self.base_url)
        self._https = parts.scheme == "https"
        self._host = parts.hostname or "127.0.0.1"
        self._port = parts.port or (443 if self._https else 80)
        # The base URL may carry a path prefix; every request is relative to it.
        self._prefix = parts.path.rstrip("/")
        self._conn: http.client.HTTPConnection | None = None
        # rclpy runs timer callbacks on one thread, but a node is free to call
        # from elsewhere and a half-written request would corrupt the stream.
        self._lock = threading.Lock()

    # -- transport ---------------------------------------------------------

    def close(self) -> None:
        """Drop the pooled connection, if any. Safe to call at any time."""
        with self._lock:
            self._drop()

    def _drop(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001 -- closing must never raise
                pass
            self._conn = None

    def _connect(self, timeout: float) -> tuple[http.client.HTTPConnection, bool]:
        """Return (connection, reused). Opens one if the pool is empty."""
        if self._conn is not None:
            self._conn.timeout = timeout
            return self._conn, True
        cls = http.client.HTTPSConnection if self._https else http.client.HTTPConnection
        self._conn = cls(self._host, self._port, timeout=timeout)
        return self._conn, False

    def _send(
        self, method: str, url: str, data: bytes | None, headers: dict[str, str], timeout: float
    ) -> tuple[int, bytes]:
        """One request over the pooled connection, reconnecting once if stale.

        ⚠ THE RETRY IS DELIBERATELY LIMITED TO A **REUSED** CONNECTION. A
        server is free to close an idle keep-alive connection at any moment, so
        a send that fails on a connection we did not just open almost certainly
        means the request never arrived -- safe to repeat. A failure on a
        FRESHLY opened connection means the far side is down, and retrying a
        POST there could execute a motion command twice. So that case raises.
        """
        last: Exception | None = None
        for _ in (0, 1):
            conn, reused = self._connect(timeout)
            try:
                conn.request(method, url, body=data, headers=headers)
                resp = conn.getresponse()
                raw = resp.read()
                status = resp.status
                # Honour the server's intent; an HTTP/1.0 reply or an explicit
                # `Connection: close` means this socket is finished.
                if resp.will_close or not self.keep_alive:
                    self._drop()
                return status, raw
            except (http.client.HTTPException, OSError, TimeoutError) as exc:
                last = exc
                self._drop()
                if not reused:
                    break
        raise HarnessUnreachable(
            f"could not reach OmniSim at {self.base_url}: {last}. "
            f"Start it with: python -m omnisim harness --auto-port"
        ) from last

    def request(
        self,
        method: str,
        path: str,
        body: dict[str, Any] | None = None,
        query: dict[str, Any] | None = None,
        timeout_s: float | None = None,
    ) -> HarnessResponse:
        url = self._prefix + path
        if query:
            # Drop None so callers can pass optional params positionally without
            # sending the literal string "None" to the harness.
            clean = {k: v for k, v in query.items() if v is not None}
            if clean:
                url += "?" + urllib.parse.urlencode(clean)
        data = None
        headers = {"Accept": "application/json"}
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        # http.client sets Content-Length for a bytes body itself.
        timeout = timeout_s or self.timeout_s
        with self._lock:
            status, raw = self._send(method, url, data, headers, timeout)
        try:
            parsed = json.loads(raw.decode("utf-8")) if raw else {}
        except (ValueError, UnicodeDecodeError):
            # Screenshots and other binary payloads land here; keep the bytes.
            parsed = {}
        if not isinstance(parsed, dict):
            parsed = {"items": parsed}
        return HarnessResponse(status=status, body=parsed, raw=raw)

    def get(self, path: str, **kw: Any) -> HarnessResponse:
        return self.request("GET", path, **kw)

    def post(self, path: str, body: dict[str, Any] | None = None, **kw: Any) -> HarnessResponse:
        return self.request("POST", path, body=body if body is not None else {}, **kw)

    # -- Harness endpoints used by the ROS 2 nodes -------------------------

    def sim_state(self) -> HarnessResponse:
        return self.get("/sim/state")

    def capabilities(self) -> HarnessResponse:
        # Never pass probe_step=1 here: it advances the simulation by one step.
        return self.get("/capabilities")

    def sim_step(self, steps: int = 1, timeout_s: float | None = None) -> HarnessResponse:
        return self.post("/sim/step", {"steps": int(steps)}, timeout_s=timeout_s)

    def sim_reset(
        self, restore: str | None = "__init__", settle_steps: int | None = None
    ) -> HarnessResponse:
        body: dict[str, Any] = {"restore": restore}
        if settle_steps is not None:
            body["settle_steps"] = int(settle_steps)
        return self.post("/sim/reset", body)

    def scene_tree(self, bounds: bool = False) -> HarnessResponse:
        return self.get("/scene/tree", query={"bounds": 1 if bounds else None})

    def scene_node(self, def_name: str, bounds: bool = False) -> HarnessResponse:
        quoted = urllib.parse.quote(def_name, safe="")
        return self.get(f"/scene/node/{quoted}", query={"bounds": 1 if bounds else None})

    def scene_spawn(self, body: dict[str, Any]) -> HarnessResponse:
        return self.post("/scene/spawn", body)

    def scene_delete(self, defs: list[str]) -> HarnessResponse:
        return self.post("/scene/delete", {"defs": list(defs)})

    def scene_set_pose(self, body: dict[str, Any]) -> HarnessResponse:
        return self.post("/scene/set_pose", body)

    def robots(self, include_harness: bool = False) -> HarnessResponse:
        return self.get("/robots", query={"include_harness": 1 if include_harness else None})

    def robot_joints(self, def_name: str) -> HarnessResponse:
        quoted = urllib.parse.quote(def_name, safe="")
        return self.get(f"/robot/{quoted}/joints")

    def robot_devices(self, def_name: str) -> HarnessResponse:
        quoted = urllib.parse.quote(def_name, safe="")
        return self.get(f"/robot/{quoted}/devices")

    def world_load(
        self, path: str, light: bool | None = None, wait_s: float | None = None
    ) -> HarnessResponse:
        body: dict[str, Any] = {"path": path}
        if light is not None:
            body["light"] = bool(light)
        if wait_s is not None:
            body["wait_s"] = float(wait_s)
        # A cold asset-heavy load can exceed the default client timeout well
        # before the harness gives up, so allow generous headroom over wait_s.
        return self.post("/world/load", body, timeout_s=max(self.timeout_s, (wait_s or 30.0) + 60.0))
