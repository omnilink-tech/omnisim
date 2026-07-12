# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.
"""Per-agent runner smoke child (one subprocess per agent).

Imports a single production agent module, builds an OmniLinkAgentRunner from
the module's dispatch/classify/status, starts the real tool-callback HTTP
server (no OMNI_KEY / no OmniLink network needed), and exercises the live
endpoints over HTTP:

    POST /tool   {"tool": "__smoke_unknown__"}  -> {status: ok, result.error}
    GET  /status                                 -> {agent, usage, ...}
    GET  /activity                               -> {status: ok, entries:[>=1]}

Run ONE agent per process: every agent ships its own ``tools`` package, so
importing two in the same interpreter would alias the second to the first.

Usage:  python _runner_smoke_child.py <path-to-agent_agent.py>
Exit 0 + "SMOKE_PASS <agent>" on success; non-zero on any failure.
"""

from __future__ import annotations

import importlib.util
import json
import sys
import urllib.request
from pathlib import Path


def _get(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


def _post(url: str, payload: dict) -> dict:
    req = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST",
    )
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


def main(agent_path: Path) -> int:
    # The agent module's own top-level code inserts its tools/ dir and
    # agents/production (for `_lib`) onto sys.path during exec_module.
    spec = importlib.util.spec_from_file_location("agent_under_test", agent_path)
    assert spec and spec.loader, f"cannot load {agent_path}"
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    from _lib import OmniLinkAgentRunner

    for attr in ("dispatch", "classify_result", "status_snapshot", "QUERY_TOOLS"):
        assert hasattr(mod, attr), f"{agent_path.name} missing {attr}"

    runner = OmniLinkAgentRunner(
        agent_name="smoke",
        profile_path=agent_path.parent / "profile.json",
        port=0,  # random free port
        dispatch=mod.dispatch,
        query_tools=mod.QUERY_TOOLS,
        classify_result=mod.classify_result,
        status_snapshot=mod.status_snapshot,
    )
    port = runner._start_tool_server()
    base = f"http://127.0.0.1:{port}"

    # 1. POST /tool with an unknown tool — exercises dispatch + activity logging
    #    + result serialization without needing a live bridge.
    tool_resp = _post(f"{base}/tool", {"tool": "__smoke_unknown__", "x": 1})
    assert tool_resp.get("status") == "ok", tool_resp
    assert isinstance(tool_resp.get("result"), dict), tool_resp
    assert "error" in tool_resp["result"], tool_resp
    assert "known_tools" in tool_resp["result"], tool_resp

    # 2. GET /status — agent-specific snapshot + runner-injected usage.
    status = _get(f"{base}/status")
    assert status.get("agent"), status
    assert "usage" in status, "runner must inject usage into /status"
    assert status.get("tools_registered", 0) == len(mod.REGISTRY), status

    # 3. GET /activity — the POST above must be logged.
    activity = _get(f"{base}/activity")
    assert activity.get("status") == "ok", activity
    assert len(activity.get("entries", [])) >= 1, activity
    assert activity["entries"][-1]["data"]["tool"] == "__smoke_unknown__", activity

    print(f"SMOKE_PASS {status['agent']} "
          f"(tools={status.get('tools_registered')}, port={port})")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: _runner_smoke_child.py <agent_agent.py>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(Path(sys.argv[1]).resolve()))
