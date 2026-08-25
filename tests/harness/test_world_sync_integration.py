# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0

"""Opt-in real-engine coverage for the default agent world-sync path.

Run with an isolated harness, for example:

    OMNISIM_HARNESS_INTEGRATION_URL=http://127.0.0.1:16889 \
      pytest tests/harness/test_world_sync_integration.py -q
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import urllib.request

import pytest


BASE = os.environ.get("OMNISIM_HARNESS_INTEGRATION_URL", "").rstrip("/")
FIXTURES = Path(__file__).resolve().parent / "worlds"
pytestmark = pytest.mark.skipif(not BASE, reason="set OMNISIM_HARNESS_INTEGRATION_URL")


def _post(path: str, body: dict) -> dict:
    request = urllib.request.Request(
        BASE + path, data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(request, timeout=40) as response:
        return json.load(response)


def _get(path: str) -> dict:
    with urllib.request.urlopen(BASE + path, timeout=10) as response:
        return json.load(response)


def test_real_solids_robot_and_reload_fallback(tmp_path):
    solid = tmp_path / "sync_solid.wbt"
    solid.write_text((FIXTURES / "world_sync_solid.omniworld").read_text(encoding="utf-8"),
                     encoding="utf-8")
    loaded = _post("/world/load", {"path": str(solid), "light": True})
    assert loaded["ok"] and loaded["supervisor_connected"]

    text = solid.read_text(encoding="utf-8")
    text = text.replace("translation -0.5 0 1.2", "translation -0.7 0 1.6")
    text = text.replace("translation 0.5 0 1.4", "translation 0.7 0 1.8")
    solid.write_text(text, encoding="utf-8")
    synced = _post("/world/sync", {"path": str(solid), "settle_steps": 120})
    assert synced["mode"] == "live_pose"
    assert synced["verification"]["applied"] == 2
    assert synced["verification"]["single_settle_for_batch"] is True
    positions = {item["def"]: item["position"] for item in synced["changes"]}
    assert positions["SYNC_BOX_A"][2] == pytest.approx(0.2, abs=1e-3)
    assert positions["SYNC_BOX_B"][2] == pytest.approx(0.15, abs=1e-3)

    text = solid.read_text(encoding="utf-8").replace(
        "size 0.4 0.4 0.4", "size 0.5 0.4 0.4")
    solid.write_text(text, encoding="utf-8")
    fallback = _post("/world/sync", {"path": str(solid)})
    assert fallback["mode"] == "full_reload"
    assert fallback["fallback"] is True
    assert fallback["supervisor_connected"] is True

    robot = tmp_path / "sync_robot.wbt"
    robot.write_text((FIXTURES / "world_sync_robot.omniworld").read_text(encoding="utf-8"),
                     encoding="utf-8")
    assert _post("/world/load", {"path": str(robot), "light": True})["ok"]
    text = robot.read_text(encoding="utf-8")
    text = text.replace("translation 0 0 1.2", "translation 0.6 -0.4 1.5")
    text = text.replace("rotation 0 0 1 0", "rotation 0 0 1 0.7")
    robot.write_text(text, encoding="utf-8")
    moved = _post("/world/sync", {"path": str(robot), "settle_steps": 120})
    assert moved["mode"] == "live_pose"
    assert moved["changes"][0]["type"] == "Robot"
    node = _get("/scene/node/SYNC_ROBOT")
    assert node["position"] == pytest.approx([0.6, -0.4, 0.1], abs=1e-3)
    assert node["fields"]["rotation"][3] == pytest.approx(0.7, abs=1e-5)
