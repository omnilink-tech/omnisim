# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Smoke tests for the OmniSim ↔ OmniLink integration surface.

These tests verify three things, in increasing scope:

1. The `agents/production/_lib/` SDK imports cleanly (no missing imports,
   no syntax errors, no circular references).
2. The `@action` decorator + dispatch table behave correctly.
3. If `omnilink-lib` is reachable (either pip-installed or sibling
   checkout), `OmniLinkClient` is constructible.

The tests are deliberately offline-only — they do NOT make any network
calls to the OmniLink platform. A network-bound integration test would
need a real OMNI_KEY and would belong in a separate marked suite.

Run with:
    pytest tests/omnilink/test_omnilink_smoke.py
"""

from __future__ import annotations

import json
import sys
import threading
import urllib.request
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "omnilink-agents"))


# ---------------------------------------------------------------------------
# 1. SDK importability


def test_sdk_imports_clean():
    from _lib import (  # noqa: F401
        OmniLinkAgentRunner,
        OmniSimBridgeServer,
        action,
        locate_omnilink_lib,
    )


def test_locate_omnilink_lib_returns_path_or_none():
    from _lib import locate_omnilink_lib

    result = locate_omnilink_lib()
    assert result is None or result.exists()


# ---------------------------------------------------------------------------
# 2. Bridge SDK


def test_action_decorator_collects_handlers():
    from _lib import OmniSimBridgeServer, action

    class B(OmniSimBridgeServer):
        def get_state(self):
            return {}

        def get_capabilities(self):
            return {}

        @action("alpha")
        def _alpha(self, body):
            return {"echo": body}

        @action("beta")
        def _beta(self, body):
            return {"v": body.get("v")}

    bridge = B(host="127.0.0.1", port=0)
    assert bridge.list_actions() == ["alpha", "beta"]


def test_bridge_serves_state_and_action():
    from _lib import OmniSimBridgeServer, action

    class B(OmniSimBridgeServer):
        def get_state(self):
            return {"x": 1, "y": 2}

        def get_capabilities(self):
            return {"actions": self.list_actions()}

        @action("ping")
        def _ping(self, body):
            return {"pong": body.get("payload")}

    bridge = B(host="127.0.0.1", port=0)
    bound = bridge.serve_in_background()
    try:
        # /healthz
        with urllib.request.urlopen(f"http://127.0.0.1:{bound}/healthz", timeout=5) as r:
            health = json.loads(r.read())
            assert health["ok"] is True

        # /state
        with urllib.request.urlopen(f"http://127.0.0.1:{bound}/state", timeout=5) as r:
            state = json.loads(r.read())
            assert state == {"x": 1, "y": 2}

        # /capabilities
        with urllib.request.urlopen(
            f"http://127.0.0.1:{bound}/capabilities", timeout=5
        ) as r:
            caps = json.loads(r.read())
            assert caps == {"actions": ["ping"]}

        # /action ping
        body = json.dumps({"action": "ping", "payload": "hello"}).encode()
        req = urllib.request.Request(
            f"http://127.0.0.1:{bound}/action",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=5) as r:
            result = json.loads(r.read())
            assert result["status"] == "ok"
            assert result["pong"] == "hello"
    finally:
        bridge.shutdown()


def test_bridge_unknown_action_returns_404():
    from _lib import OmniSimBridgeServer, action

    class B(OmniSimBridgeServer):
        @action("known")
        def _known(self, body):
            return {}

    bridge = B(host="127.0.0.1", port=0)
    bound = bridge.serve_in_background()
    try:
        body = json.dumps({"action": "missing"}).encode()
        req = urllib.request.Request(
            f"http://127.0.0.1:{bound}/action",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with pytest.raises(urllib.error.HTTPError) as exc_info:
            urllib.request.urlopen(req, timeout=5)
        assert exc_info.value.code == 404
        payload = json.loads(exc_info.value.read())
        assert "available_actions" in payload
        assert "known" in payload["available_actions"]
    finally:
        bridge.shutdown()


# ---------------------------------------------------------------------------
# 3. omnilink-lib reachability (skip when lib isn't available)


def _omnilink_available() -> bool:
    from _lib import locate_omnilink_lib

    locate_omnilink_lib()
    try:
        import omnilink.client  # noqa: F401
    except Exception as exc:
        # Surface the underlying failure so a developer running the
        # smoke suite can fix the missing dep instead of seeing a bare
        # "skipped" line. The skip reason embeds the exception class.
        global _OMNILINK_IMPORT_ERR
        _OMNILINK_IMPORT_ERR = f"{exc.__class__.__name__}: {exc}"
        return False
    return True


_OMNILINK_IMPORT_ERR: str = ""


@pytest.mark.skipif(
    not _omnilink_available(),
    reason=f"omnilink-lib not reachable: {_OMNILINK_IMPORT_ERR or '(pip install omnilink>=0.6.1 or place an olink/ checkout next to this repo)'}",
)
def test_omnilink_client_constructible():
    from omnilink.client import OmniLinkClient

    # Constructor must not require a network call. Use a placeholder key.
    client = OmniLinkClient(omni_key="olink_smoke_test", base_url="http://127.0.0.1:1")
    assert client is not None


@pytest.mark.skipif(
    not _omnilink_available(),
    reason="omnilink-lib not reachable",
)
def test_omnilink_usage_meter_constructible():
    from omnilink.client import OmniLinkClient
    from omnilink.usage_meter import UsageMeter

    client = OmniLinkClient(omni_key="olink_smoke_test", base_url="http://127.0.0.1:1")
    meter = UsageMeter(client)
    assert meter is not None
