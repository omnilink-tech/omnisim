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
import os
import socket
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence


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
    omnisim_root = here.parent.parent.parent
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
        tool_callback_url = f"http://127.0.0.1:{self._tool_port}/tool"
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

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: ARG002 - silence default access log
                pass

            def _cors(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")

            def do_OPTIONS(self):  # noqa: N802 - http stdlib
                self.send_response(200)
                self._cors()
                self.end_headers()

            def do_GET(self):  # noqa: N802 - http stdlib
                if self.path == "/activity":
                    body = json.dumps(
                        {"status": "ok", "entries": log[-100:]}, default=str
                    ).encode()
                elif self.path == "/status":
                    body = json.dumps(runner._build_status_snapshot(), default=str).encode()
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self._cors()
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):  # noqa: N802 - http stdlib
                if self.path != "/tool":
                    self.send_error(404)
                    return
                length = int(self.headers.get("Content-Length", 0))
                raw = self.rfile.read(length).decode("utf-8", errors="replace")
                try:
                    data = json.loads(raw) if raw else {}
                except json.JSONDecodeError:
                    data = {}
                tool_name = data.pop("tool", "")
                print(f"  [TOOL] {tool_name}({data})")
                try:
                    result = runner.dispatch(tool_name, data)
                except Exception as exc:  # pragma: no cover - user dispatch crash
                    import traceback
                    print(f"  [CRASH] {tool_name}: {exc}\n{traceback.format_exc()}")
                    result = {
                        "error": f"tool crashed: {exc.__class__.__name__}: {exc}",
                        "tool": tool_name,
                    }
                ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
                kind, summary = runner.classify_result(tool_name, data, result)
                log.append(
                    {
                        "action": "tool_call",
                        "detail": summary,
                        "kind": kind,
                        "timestamp": ts,
                        "data": {"tool": tool_name, "args": data, "result": result},
                    }
                )
                body = json.dumps(
                    {"status": "ok", "tool": tool_name, "result": result}, default=str
                ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self._cors()
                self.end_headers()
                self.wfile.write(body)

        try:
            server = http.server.ThreadingHTTPServer(("0.0.0.0", port), Handler)
        except OSError as exc:
            print(f"  [WARN] Port {port} taken ({exc}); falling back to random port.")
            server = http.server.ThreadingHTTPServer(("0.0.0.0", 0), Handler)
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
