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

"""OmniLink agent-runner SDK.

Replaces the ~250 lines of identical boilerplate every productized
agent under ``agents/production/`` previously hand-rolled. A new agent's
runner script becomes ~30 lines:

    from _lib import OmniLinkAgentRunner

    def dispatch(tool: str, args: dict) -> dict:
        ...

    runner = OmniLinkAgentRunner(
        agent_name="My Agent",
        profile_path=Path(__file__).parent / "profile.json",
        port=51530,
        dispatch=dispatch,
        query_tools=[{"name": "do_thing", "description": "...", "parameters": {...}}],
    )
    runner.run()

The runner owns:

* ``omnilink-lib`` path discovery (sibling-repo lookup)
* HTTP tool-callback server (``POST /tool``, ``GET /activity``, ``GET /status``)
* CORS for browser clients (the OmniLink web UI)
* Profile push (create or update by name)
* :class:`omnilink.usage_meter.UsageMeter` baseline + ``/status.usage`` block
* Memory-poll heartbeat loop with SIGINT handling

Agents only override what's actually agent-specific: the dispatch
function, the query-tool list, and an optional status-snapshot builder.
"""

from __future__ import annotations

import http.server
import json
import math
import os
import socket
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

from .http_security import (
    RequestError,
    RequestIdGuard,
    allowed_origins,
    check_authorization,
    checked_origin,
    configured_token,
    error_envelope,
    read_json,
    validate_bind,
    validate_request_id,
)


# ---------------------------------------------------------------------------
# omnilink-lib path discovery
# ---------------------------------------------------------------------------

def locate_omnilink_lib(env_var: str = "OMNILINK_LIB") -> Optional[Path]:
    """Locate ``omnilink-lib/src`` and prepend it to ``sys.path``.

    Tries (in order):

    1. ``$OMNILINK_LIB`` (or the agent-specific override passed in)
    2. ``../olink/omnilink-lib/src`` (sibling-of-OmniSim)
    3. ``../OmniLink/omnilink-lib/src``
    4. ``../omnilink/omnilink-lib/src``

    Returns the first path that exists, or ``None``. Importing
    ``omnilink`` after this helper succeeds is the caller's job — the
    helper just makes the import resolvable when an editable install
    isn't available.
    """
    candidates: List[Path] = []
    env_val = os.environ.get(env_var, "").strip()
    if env_val:
        candidates.append(Path(env_val))

    # Walk up from this file: omnisim/agents/production/_lib/runner_base.py
    # → omnisim/omnilink-agents → omnisim → <parent that holds olink/>.
    here = Path(__file__).resolve()
    omnisim_root = here.parents[3]
    parent = omnisim_root.parent
    for sibling in ("olink", "OmniLink", "omnilink"):
        candidates.append(parent / sibling / "omnilink-lib" / "src")

    for cand in candidates:
        if cand.exists():
            cand_str = str(cand)
            if cand_str not in sys.path:
                sys.path.insert(0, cand_str)
            return cand
    return None


# Reconfigure stdio to UTF-8 once on import. Windows defaults to cp1252;
# agent banners use box-drawing characters and emoji-adjacent symbols
# that crash with UnicodeEncodeError otherwise.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="backslashreplace")  # type: ignore[attr-defined]
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

DispatchFn = Callable[[str, Dict[str, Any]], Dict[str, Any]]
StatusFn = Callable[[List[Dict[str, Any]]], Dict[str, Any]]


class OmniLinkAgentRunner:
    """Thin runner for an OmniLink agent backed by an OmniSim bridge.

    The runner reads the agent profile from ``profile_path``, pushes it
    to OmniLink under ``agent_name``, and starts a tool-callback HTTP
    server on ``port`` that dispatches incoming ``POST /tool`` calls
    through the user-supplied ``dispatch`` function. It also exposes
    ``GET /activity`` (recent tool calls) and ``GET /status`` (synthesised
    snapshot) so operators and other agents can introspect the runner
    without scanning the whole memory log.

    Parameters
    ----------
    agent_name:
        Human-readable name. Used to find/update the OmniLink profile.
    profile_path:
        Path to ``profile.json`` containing ``{"name", "settings"}``.
    port:
        Default tool-callback port. Falls back to a random free port if
        taken. Override per-runner via the ``port_env`` env var.
    dispatch:
        Callable ``(tool_name, args) -> result_dict``. Receives the JSON
        body of ``POST /tool``; returns a JSON-serialisable result.
    query_tools:
        List of tool descriptors (``{name, description, parameters}``).
        These get inlined into the agent profile under
        ``availableToolDetails`` so the OmniLink UI knows which tools
        the runner serves.
    base_url:
        OmniLink platform base URL. Default is the production endpoint.
    engine:
        Default engine. ``g1-engine`` (Gemini) is the OmniSim default
        because it doesn't require a BYOK OpenAI key.
    poll_interval:
        Seconds between memory-poll heartbeats.
    port_env / lib_env / dry_run_env:
        Env-var names the runner consults for per-instance overrides.
    classify_result:
        Optional ``(tool_name, args, result) -> (kind, summary)`` to tag
        activity-feed entries. ``kind`` is one of
        ``info / success / warning / critical``.
    status_snapshot:
        Optional ``(activity_log) -> dict`` builder for ``/status``.
    """

    def __init__(
        self,
        *,
        agent_name: str,
        profile_path: Path,
        port: int,
        dispatch: DispatchFn,
        query_tools: Sequence[Dict[str, Any]],
        base_url: str = "https://www.omnilink-agents.com",
        engine: str = "g1-engine",
        poll_interval: float = 3.0,
        port_env: Optional[str] = None,
        lib_env: str = "OMNILINK_LIB",
        dry_run_env: Optional[str] = None,
        classify_result: Optional[Callable[[str, Dict[str, Any], Any], tuple]] = None,
        status_snapshot: Optional[StatusFn] = None,
        host: str = "127.0.0.1",
        token: Optional[str] = None,
    ) -> None:
        self.agent_name = agent_name
        self.profile_path = Path(profile_path)
        self.default_port = port
        self.dispatch = dispatch
        self.query_tools = list(query_tools)
        self.base_url = base_url
        self.engine = engine
        self.poll_interval = poll_interval
        self.port_env = port_env
        self.lib_env = lib_env
        self.dry_run_env = dry_run_env
        self.classify_result = classify_result or _default_classifier
        self.status_snapshot = status_snapshot
        self.host = host
        self.token = configured_token(token)
        validate_bind(self.host, self.token)
        self._trusted_origins = allowed_origins([self.base_url])
        self._action_lock = threading.RLock()
        self._tool_schemas = {
            str(tool.get("name")): tool.get("parameters") or {}
            for tool in self.query_tools
            if tool.get("name")
        }

        self.activity_log: List[Dict[str, Any]] = []
        self._client: Any = None
        self._usage_meter: Any = None
        self._usage_baseline: Any = None
        self._tool_port: int = 0
        self._stop = threading.Event()

    # ------------------------------------------------------------------
    # public surface

    def run(self) -> None:
        """Push the profile, start the tool server, run the main loop.

        Blocks until SIGINT (Ctrl+C). Returns cleanly so callers can
        wrap the runner in a parent script if needed.
        """
        omni_key = os.environ.get("OMNI_KEY", "").strip()
        if not omni_key:
            print("Error: set OMNI_KEY.")
            print('  Windows: set OMNI_KEY=olink_YOUR_KEY_HERE')
            print('  bash:    export OMNI_KEY="olink_YOUR_KEY_HERE"')
            sys.exit(1)

        # Some Windows boxes run TLS-intercepting antivirus (AVG, ESET,
        # ...) whose CA is in the Windows cert store but not certifi.
        # truststore.inject_into_ssl() makes Python's SSL use the OS
        # trust store. Safe no-op when truststore isn't installed.
        try:
            import truststore  # type: ignore[import-not-found]
            truststore.inject_into_ssl()
        except Exception:
            pass

        if not _omnilink_importable(lib_env=self.lib_env):
            print("Error: omnilink-lib is not importable.")
            print("  Either: pip install omnilink>=0.6.1")
            print(f"  Or:     set {self.lib_env}=<path-to-olink/omnilink-lib/src>")
            print("  Or:     place an OLink checkout next to this OmniSim repo.")
            sys.exit(1)

        from omnilink.client import OmniLinkClient  # type: ignore

        try:
            from omnilink.usage_meter import UsageMeter  # type: ignore
        except Exception:
            UsageMeter = None  # type: ignore[assignment]

        profile_doc = self._load_profile()
        configured_name = profile_doc.get("name", self.agent_name)
        base_settings = profile_doc.get("settings", {})

        self._client = OmniLinkClient(
            omni_key=omni_key, base_url=self.base_url, timeout=60
        )
        if UsageMeter is not None:
            try:
                self._usage_meter = UsageMeter(self._client)
                self._usage_baseline = self._usage_meter.start()
            except Exception as exc:  # pragma: no cover - network blip
                print(f"  [WARN] usage meter init failed: {exc}")

        self._print_banner(configured_name)

        self._tool_port = self._start_tool_server()
        callback_host = "127.0.0.1" if self.host in ("0.0.0.0", "::") else self.host
        tool_callback_url = f"http://{callback_host}:{self._tool_port}/tool"
        print(f"  Tool callback server: {tool_callback_url}")
        print(f"    POST /tool      - tool execution")
        print(f"    GET  /activity  - live activity feed")
        print(f"    GET  /status    - status snapshot")
        print()

        settings = self._build_settings(base_settings, tool_callback_url)
        self._ensure_profile(configured_name, settings)
        print()
        print("-" * 64)
        print(f"  {configured_name} is live at: {self.base_url}")
        print(f"  Pick '{configured_name}' in the UI and start chatting.")
        print("-" * 64)
        print()
        print("  Main loop (Ctrl+C to stop)...")
        print()

        try:
            self._main_loop()
        except KeyboardInterrupt:
            print("\n  Interrupted.")
        finally:
            self._stop.set()

        print()
        print("=" * 64)
        print(f"  Activities: {len(self.activity_log)}")
        print("=" * 64)

    # ------------------------------------------------------------------
    # private

    def _resolve_port(self) -> int:
        if self.port_env:
            raw = os.environ.get(self.port_env, "").strip()
            if raw:
                try:
                    return int(raw)
                except ValueError:
                    pass
        return self.default_port

    def _load_profile(self) -> Dict[str, Any]:
        with open(self.profile_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _build_settings(
        self, base_settings: Dict[str, Any], tool_callback_url: str
    ) -> Dict[str, Any]:
        settings = dict(base_settings)
        settings["availableTools"] = ", ".join(t["name"] for t in self.query_tools)
        settings["availableToolDetails"] = list(self.query_tools)
        settings["toolCallbackUrl"] = tool_callback_url
        settings.setdefault("engine", self.engine)
        return settings

    def _ensure_profile(self, name: str, settings: Dict[str, Any]) -> str:
        profiles = self._client.list_profiles()
        existing = next(
            (p for p in profiles if (p.get("name") or "").lower() == name.lower()),
            None,
        )
        if existing:
            pid = existing["id"]
            self._client.update_profile(pid, name=name, settings=settings)
            print(f"  Profile updated: {name} (id={pid})")
            return pid
        result = self._client.create_profile(name, settings=settings)
        pid = result.get("id", "")
        print(f"  Profile created: {name} (id={pid})")
        return pid

    def _print_banner(self, name: str) -> None:
        print()
        print("=" * 64)
        print(f"  {name} - OmniLink agent")
        print("=" * 64)
        print(f"  Profile:     {self.profile_path}")
        if self.dry_run_env:
            dry = os.environ.get(self.dry_run_env, "0").strip() not in ("0", "false", "no", "")
            print(f"  Dry-run:     {dry}  ({self.dry_run_env})")
        print(f"  Tools:       {len(self.query_tools)} registered")
        print()

    def _start_tool_server(self) -> int:
        port = self._resolve_port()
        runner = self
        log = self.activity_log
        request_ids = RequestIdGuard()

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: ARG002 - silence default access log
                pass

            def _cors(self, origin: Optional[str]):
                if origin:
                    self.send_header("Access-Control-Allow-Origin", origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header(
                    "Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token"
                )

            def _guard(self) -> Optional[str]:
                origin = checked_origin(self.headers, runner._trusted_origins)
                check_authorization(self.headers, runner.token)
                return origin

            def _send(self, status: int, payload: Any, origin: Optional[str]) -> None:
                body = json.dumps(payload, default=str).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self._cors(origin)
                self.end_headers()
                self.wfile.write(body)

            def _failure(self, exc: RequestError, origin: Optional[str] = None) -> None:
                self._send(exc.status, error_envelope(exc.code, exc.message, exc.details), origin)

            def do_OPTIONS(self):  # noqa: N802 - http stdlib
                try:
                    origin = checked_origin(self.headers, runner._trusted_origins)
                    self.send_response(204)
                    self._cors(origin)
                    self.end_headers()
                except RequestError as exc:
                    self._failure(exc)

            def do_GET(self):  # noqa: N802 - http stdlib
                origin: Optional[str] = None
                try:
                    origin = self._guard()
                    path = self.path.split("?", 1)[0].rstrip("/") or "/"
                    if path == "/activity":
                        return self._send(200, {"status": "ok", "entries": log[-100:]}, origin)
                    if path == "/status":
                        return self._send(200, runner._build_status_snapshot(), origin)
                    if path == "/healthz":
                        return self._send(200, {"ok": True, "agent": runner.agent_name}, origin)
                    self._send(404, error_envelope("not_found", "Endpoint not found."), origin)
                except RequestError as exc:
                    self._failure(exc, origin)

            def do_POST(self):  # noqa: N802 - http stdlib
                origin: Optional[str] = None
                try:
                    origin = self._guard()
                    path = self.path.split("?", 1)[0].rstrip("/") or "/"
                    if path != "/tool":
                        return self._send(404, error_envelope("not_found", "Endpoint not found."), origin)
                    data = read_json(self, allow_empty=False)
                    tool_value = data.pop("tool", None)
                    if not isinstance(tool_value, str) or not tool_value.strip():
                        raise RequestError(400, "missing_field", "A non-empty 'tool' field is required.")
                    tool_name = tool_value.strip()
                    request_id = validate_request_id(data.pop("id", None))
                    schema = runner._tool_schemas.get(tool_name)
                    if schema is None:
                        raise RequestError(
                            404,
                            "unknown_tool",
                            f"Unknown tool {tool_name!r}.",
                            {"known_tools": sorted(runner._tool_schemas)},
                        )
                    _validate_tool_args(tool_name, data, schema)
                    request_ids.claim(f"/tool/{tool_name}", request_id)
                    print(f"  [TOOL] {tool_name}({data})")
                    try:
                        if tool_name in ("stop", "stop_robot", "emergency_stop"):
                            result = runner.dispatch(tool_name, data)
                        else:
                            with runner._action_lock:
                                result = runner.dispatch(tool_name, data)
                    except Exception as exc:  # pragma: no cover - user dispatch crash
                        import traceback
                        print(f"  [CRASH] {tool_name}: {exc}\n{traceback.format_exc()}")
                        result = {
                            "error": "tool_execution_failed",
                            "tool": tool_name,
                        }
                    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
                    kind, summary = runner.classify_result(tool_name, data, result)
                    from .supervision import failure_error, normalize_outcome
                    outcome = normalize_outcome(result).to_dict()
                    log.append(
                        {
                            "action": "tool_call",
                            "detail": summary,
                            "kind": kind,
                            "timestamp": ts,
                            "data": {
                                "tool": tool_name,
                                "args": data,
                                "result": result,
                                "outcome": outcome,
                            },
                        }
                    )
                    # `error` is not always a failure: the maze bridge names
                    # its numeric RESIDUAL `error` (a healthy 1 m drive returns
                    # error: -0.03), so keying the HTTP verdict off the field's
                    # presence reported every good blocking drive as "err".
                    status = "err" if failure_error(result) else "ok"
                    self._send(200, {"status": status, "tool": tool_name, "result": result}, origin)
                except RequestError as exc:
                    self._failure(exc, origin)

        try:
            server = http.server.ThreadingHTTPServer((self.host, port), Handler)
        except OSError as exc:
            print(f"  [WARN] Port {port} taken ({exc}); falling back to random port.")
            server = http.server.ThreadingHTTPServer((self.host, 0), Handler)
        bound = server.server_address[1]
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return bound

    def _build_status_snapshot(self) -> Dict[str, Any]:
        if self.status_snapshot is not None:
            try:
                snap = self.status_snapshot(self.activity_log)
            except Exception as exc:  # pragma: no cover
                snap = {"error": f"status_snapshot crashed: {exc}"}
        else:
            snap = {
                "agent": self.agent_name,
                "tools_registered": len(self.query_tools),
                "activity_log_size": len(self.activity_log),
            }
        snap["usage"] = self._usage_for_status()
        return snap

    def _usage_for_status(self) -> Dict[str, Any]:
        if self._usage_meter is None or self._usage_baseline is None:
            return {"available": False, "reason": "meter not initialised"}
        try:
            delta = self._usage_meter.snapshot(baseline=self._usage_baseline)
        except Exception as exc:
            return {
                "available": False,
                "reason": f"snapshot failed: {exc.__class__.__name__}: {exc}",
            }
        payload = delta.to_dict()
        payload["available"] = True
        return payload

    def _main_loop(self) -> None:
        tick = 0
        while not self._stop.is_set():
            time.sleep(self.poll_interval)
            tick += 1
            if tick % 20 == 0:
                print(
                    f"  [heartbeat] {len(self.activity_log)} activities, "
                    f"port={self._tool_port}"
                )


# ---------------------------------------------------------------------------
# helpers

def _validate_tool_args(tool_name: str, args: Dict[str, Any], schema: Dict[str, Any]) -> None:
    """Validate the useful JSON-Schema subset emitted by shipped tools."""
    required = schema.get("required") or []
    for name in required:
        if name not in args:
            raise RequestError(
                400,
                "missing_field",
                f"Tool {tool_name!r} requires field {name!r}.",
                {"tool": tool_name, "field": name},
            )
    properties = schema.get("properties") or {}
    if schema.get("additionalProperties") is False:
        unknown = sorted(set(args) - set(properties))
        if unknown:
            raise RequestError(
                400,
                "unknown_field",
                f"Tool {tool_name!r} received unsupported fields.",
                {"tool": tool_name, "fields": unknown},
            )
    for name, value in args.items():
        spec = properties.get(name)
        if not isinstance(spec, dict):
            continue
        expected = spec.get("type")
        expected_types = expected if isinstance(expected, list) else [expected]
        if value is None and "null" in expected_types:
            continue
        type_ok = (
            ("number" in expected_types and isinstance(value, (int, float)) and not isinstance(value, bool))
            or ("integer" in expected_types and isinstance(value, int) and not isinstance(value, bool))
            or ("string" in expected_types and isinstance(value, str))
            or ("boolean" in expected_types and isinstance(value, bool))
            or ("array" in expected_types and isinstance(value, list))
            or ("object" in expected_types and isinstance(value, dict))
            or expected in (None, [])
        )
        if not type_ok:
            raise RequestError(
                400,
                "invalid_type",
                f"Field {name!r} has the wrong type for tool {tool_name!r}.",
                {"tool": tool_name, "field": name, "expected": expected},
            )
        if "enum" in spec and value not in spec["enum"]:
            raise RequestError(
                400,
                "value_out_of_range",
                f"Field {name!r} is not one of the allowed values.",
                {"tool": tool_name, "field": name, "allowed": spec["enum"]},
            )
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if not math.isfinite(float(value)):
                raise RequestError(400, "value_out_of_range", f"Field {name!r} must be finite.")
            if "minimum" in spec and value < spec["minimum"]:
                raise RequestError(400, "value_out_of_range", f"Field {name!r} is below its minimum.")
            if "maximum" in spec and value > spec["maximum"]:
                raise RequestError(400, "value_out_of_range", f"Field {name!r} exceeds its maximum.")


def _omnilink_importable(lib_env: str) -> bool:
    locate_omnilink_lib(env_var=lib_env)
    try:
        import omnilink.client  # noqa: F401
    except Exception:
        return False
    return True


def _default_classifier(tool: str, args: Dict[str, Any], result: Any) -> tuple:
    """Conservative default: success unless ``result`` is a dict with ``error``."""
    if isinstance(result, dict):
        if result.get("error"):
            return "warning", f"{tool} -> error: {result['error']}"
    return "info", f"{tool}: ok"


def _free_port_hint() -> int:
    """Return an unused TCP port in the ephemeral range (best-effort)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]
    finally:
        sock.close()
