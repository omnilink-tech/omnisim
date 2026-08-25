#!/usr/bin/env python3
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

"""Tests for the smart-house agent package + tier benchmark.

pytest-style, and runnable directly with no pytest installed::

    python agents/production/smart_house/benchmark/test_compare_tiers.py

Covers:
  1. tool-spec schema validity (the 19 contract names, unique, valid
     JSON-schema parameter blocks, correct tiers, client-side refusals);
  2. mock_hub contract conformance (every endpoint answers the frozen
     contract's shape, fault codes included, physics calibration);
  3. one full ``--mock --fake-llm`` run of s2_oven_left_on asserting the
     PERSISTENT arm detects the oven within 120 house-min while NORMAL
     does not until the return turn.
"""

from __future__ import annotations

import json
import shutil
import sys
import tempfile
import urllib.error
import urllib.request
from pathlib import Path

_THIS = Path(__file__).resolve().parent
_PKG = _THIS.parent
sys.path.insert(0, str(_THIS))   # mock_hub, compare_tiers
sys.path.insert(0, str(_PKG))    # tools package

from tools import load_all                                        # noqa: E402
from tools._base import CONFIRM_REQUIRED, GUARDED, SAFE           # noqa: E402
from mock_hub import MockHub                                      # noqa: E402
import compare_tiers                                              # noqa: E402

CONTRACT_TOOLS = {
    "list_rooms", "list_devices", "read_sensors", "get_device_state",
    "check_anomalies", "get_energy_report", "get_weather", "set_device",
    "toggle_device", "set_scene", "adjust_thermostat", "set_schedule",
    "lock_door", "unlock_door", "arm_security", "disarm_security",
    "shut_water_main", "shut_gas_main", "notify_occupant",
}

CONTRACT_DEVICES = {
    "lights.living_ceiling", "lights.living_lamp", "lights.kitchen_ceiling",
    "lights.bedroom_ceiling", "lights.bedroom_lamp", "lights.hallway",
    "appliance.oven", "appliance.coffee_maker", "appliance.tv",
    "thermostat.main", "hvac.heater", "lock.front_door", "door.front",
    "blinds.living", "security.system",
}


def _post(base: str, endpoint: str, payload=None, raw: bytes = None):
    url = f"{base}/{endpoint.lstrip('/')}"
    body = raw if raw is not None else json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8") or "{}")


def _get(base: str, endpoint: str):
    with urllib.request.urlopen(f"{base}/{endpoint.lstrip('/')}", timeout=10) as r:
        return r.status, json.loads(r.read().decode("utf-8"))


# ---------------------------------------------------------------------------
# 1. Tool specs
# ---------------------------------------------------------------------------

def test_tool_specs_schema():
    registry = load_all()
    names = set(registry.keys())
    assert names == CONTRACT_TOOLS, (
        f"tool set drifted: missing={CONTRACT_TOOLS - names}, "
        f"extra={names - CONTRACT_TOOLS}")
    assert len(registry) == 19

    for name, spec in registry.items():
        p = spec.parameters
        assert isinstance(p, dict), f"{name}: parameters not a dict"
        assert p.get("type") == "object", f"{name}: parameters.type != object"
        props = p.get("properties", {})
        assert isinstance(props, dict), f"{name}: properties not a dict"
        for pname, pspec in props.items():
            assert isinstance(pspec, dict), f"{name}.{pname}: spec not a dict"
        for req in p.get("required", []):
            assert req in props, f"{name}: required {req!r} not in properties"
        assert spec.tier in (SAFE, GUARDED, CONFIRM_REQUIRED), \
            f"{name}: bad tier {spec.tier!r}"
        assert spec.description.strip(), f"{name}: empty description"
        qt = spec.to_query_tool()
        assert set(qt) == {"name", "description", "parameters"}

    confirm = {n for n, s in registry.items() if s.tier == CONFIRM_REQUIRED}
    assert confirm == {"unlock_door", "disarm_security",
                       "shut_water_main", "shut_gas_main"}
    safe = {n for n, s in registry.items() if s.tier == SAFE}
    assert {"lock_door", "arm_security", "notify_occupant"} <= safe


def test_confirm_required_refuse_client_side():
    """No authorization arg -> refusal WITHOUT any hub call (no hub running)."""
    registry = load_all()
    for name, kwargs in (
        ("unlock_door", {"id": "lock.front_door"}),
        ("disarm_security", {}),
        ("shut_water_main", {}),
        ("shut_gas_main", {}),
    ):
        res = registry[name].impl(**kwargs)
        assert res.get("error") == "authorization required", (name, res)
        assert "hint" in res, name


# ---------------------------------------------------------------------------
# 2. Mock hub contract conformance
# ---------------------------------------------------------------------------

def test_mock_hub_contract():
    hub = MockHub(port=0).start()
    b = hub.url
    try:
        # Rooms + devices
        st, rooms = _post(b, "list_rooms")
        assert st == 200 and isinstance(rooms, list) and len(rooms) == 4
        assert {r["id"] for r in rooms} == {"living_room", "kitchen",
                                            "bedroom", "hallway"}
        assert all({"id", "name", "device_count"} <= set(r) for r in rooms)

        st, devs = _post(b, "list_devices")
        assert st == 200 and isinstance(devs, list)
        assert {d["id"] for d in devs} == CONTRACT_DEVICES
        assert all({"id", "type", "room_id", "state", "capabilities"} <= set(d)
                   for d in devs)
        st, kdevs = _post(b, "list_devices", {"room": "kitchen"})
        assert {d["id"] for d in kdevs} == {"lights.kitchen_ceiling",
                                            "appliance.oven",
                                            "appliance.coffee_maker"}
        st, bad = _post(b, "list_devices", {"room": "garage"})
        assert bad == {"accepted": False, "error": "room_unknown",
                       "message": bad["message"]}

        # Sensors
        st, sens = _post(b, "read_sensors")
        assert st == 200 and "readings" in sens
        rooms_seen = {r["room_id"] for r in sens["readings"]}
        assert "outside" in rooms_seen and "kitchen" in rooms_seen
        types = {r["type"] for r in sens["readings"]}
        assert types == {"temperature", "motion"}
        st, bad = _post(b, "read_sensors", {"room": "attic"})
        assert bad["error"] == "room_unknown"

        # Device state + set/toggle
        st, ds = _post(b, "get_device_state", {"id": "appliance.oven"})
        assert {"state", "last_change", "changed_by", "online"} <= set(ds)
        st, res = _post(b, "set_device", {"id": "lights.living_ceiling",
                                          "state": {"on": True, "brightness": 40}})
        assert res["accepted"] is True
        assert res["realized_state"] == {"on": True, "brightness": 40}
        st, res = _post(b, "set_device", {"id": "ghost.device", "state": "on"})
        assert res["error"] == "device_offline" and res["accepted"] is False
        st, res = _post(b, "toggle_device", {"id": "appliance.tv"})
        assert res["accepted"] is True and res["new_state"] == "on"
        st, res = _post(b, "set_device", {"id": "appliance.oven",
                                          "state": "broil"})
        assert res["error"] == "state_rejected"

        # Scenes
        st, res = _post(b, "set_scene", {"scene": "movie"})
        assert res["accepted"] is True
        affected = {a["id"]: a["state"] for a in res["affected"]}
        assert affected["appliance.tv"] == "on"
        assert affected["blinds.living"] == "closed"
        st, res = _post(b, "set_scene", {"scene": "disco"})
        assert res["error"] == "scene_unknown"
        # 'away' includes the thermostat -> its affected entry must carry the
        # measured thermostat state (regression: adjust_thermostat returns
        # {target, mode}, not realized_state).
        st, res = _post(b, "set_scene", {"scene": "away"})
        assert res["accepted"] is True
        away_states = {a["id"]: a["state"] for a in res["affected"]}
        assert away_states["thermostat.main"]["mode"] == "eco"
        assert away_states["security.system"] == "armed"
        assert away_states["lock.front_door"] == "locked"

        # Thermostat clamping
        st, res = _post(b, "adjust_thermostat", {"id": "thermostat.main",
                                                 "target": 50})
        assert res["accepted"] and res["target"] == 30.0 and res["clamped"]

        # Honest refusals
        st, res = _post(b, "set_schedule", {"id": "thermostat.main",
                                            "schedule": []})
        assert res == {"accepted": False, "error": "state_rejected",
                       "message": "this hub does not execute device schedules"}
        st, res = _post(b, "shut_water_main", {"authorization": "tok"})
        assert res["error"] == "state_rejected" and "not plumbed" in res["message"]

        # Security auth gates (hub-side)
        st, res = _post(b, "unlock_door", {"id": "lock.front_door"})
        assert res["error"] == "authorization_required"
        st, res = _post(b, "unlock_door", {"id": "lock.front_door",
                                           "authorization": "occupant-ok"})
        assert res["accepted"] and res["state"] == "unlocked"
        st, res = _post(b, "disarm_security", {})
        assert res["error"] == "authorization_required"
        st, res = _post(b, "arm_security", {"zone": "perimeter"})
        assert res["accepted"] and res["state"] == "armed"
        st, res = _post(b, "lock_door", {"id": "lock.front_door"})
        assert res["accepted"] and res["state"] == "locked"

        # Weather + energy + anomalies shapes
        st, res = _post(b, "get_weather")
        assert {"temp_c", "condition", "house_time"} <= set(res)
        st, res = _post(b, "get_energy_report", {"range": "24h"})
        assert {"total_kwh", "by_category", "outliers"} <= set(res)
        assert set(res["by_category"]) == {"hvac", "lighting",
                                           "appliances", "standby"}
        st, res = _post(b, "check_anomalies")
        assert set(res) == {"active", "history"}

        # Notify -> recorded in metrics
        st, res = _post(b, "notify_occupant", {"message": "hello",
                                               "severity": "low"})
        assert res == {"delivered": True, "channel": "in_app_sim"}

        # Scenario engine
        st, res = _post(b, "scenario/start", {"name": "t", "time_scale": 60,
                                              "start_time": "08:00"})
        assert res["ok"] and res["house_time"].endswith("08:00:00")
        st, res = _post(b, "scenario/advance", {"house_minutes": 60})
        assert res["ok"] and res["house_time"].endswith("09:00:00")
        st, res = _post(b, "scenario/status")
        assert res["name"] == "t" and res["elapsed_house_min"] == 60.0
        st, res = _post(b, "scenario/resident", {"room": "away"})
        assert res == {"ok": True, "resident": "away"}
        st, res = _post(b, "scenario/event", {"type": "device_set",
                                              "id": "appliance.oven",
                                              "state": "on"})
        assert res["ok"] and res["changed_by"] == "scenario"
        st, res = _post(b, "scenario/metrics")
        assert {"energy_wh_total", "energy_wh_by_device", "room_temps",
                "temp_timeline", "notifications", "device_log",
                "anomalies_history", "house_time"} <= set(res)
        assert any(e["id"] == "appliance.oven" and e["changed_by"] == "scenario"
                   for e in res["device_log"])
        assert any(n["message"] == "hello" for n in res["notifications"]) is False, \
            "scenario/start must reset the notification log"

        # Physics calibration: oven on + away -> kitchen 20 -> 35+ in ~2 h
        st, res = _post(b, "scenario/advance", {"house_minutes": 120})
        st, m = _post(b, "scenario/metrics")
        assert m["room_temps"]["kitchen"] >= 35.0, m["room_temps"]
        # ...and the oven alone must NOT trip energy_spike (agent-inference
        # is the demo; the hub does not hand the oven over).
        st, res = _post(b, "check_anomalies")
        assert not any(a["type"] == "energy_spike" for a in res["active"])

        # door_open_while_armed anomaly
        _post(b, "arm_security")
        _post(b, "scenario/event", {"type": "door", "id": "door.front",
                                    "state": "open"})
        _post(b, "scenario/advance", {"house_minutes": 2})
        st, res = _post(b, "check_anomalies")
        assert any(a["type"] == "door_open_while_armed" for a in res["active"])

        # Transport-level contract
        st, res = _post(b, "no_such_endpoint")
        assert st == 404 and res == {"ok": False, "error": "unknown_endpoint"}
        st, res = _post(b, "list_rooms", raw=b"{not json")
        assert st == 400 and res["error"] == "bad_json"
        st, res = _get(b, "healthz")
        assert st == 200 and res["ok"] is True
        st, res = _get(b, "protocol")
        assert res["omnisim_wire"] == "1.0" and res["service"] == "robot_bridge"

        # PROTOCOL safety stop
        _post(b, "scenario/event", {"type": "device_set",
                                    "id": "appliance.oven", "state": "on"})
        st, res = _post(b, "stop_robot")
        assert res["ok"] and "halted_at" in res
        st, ds = _post(b, "get_device_state", {"id": "appliance.oven"})
        assert ds["state"] == "off"
    finally:
        hub.stop()


# ---------------------------------------------------------------------------
# 3. Full offline benchmark run — s2, both arms
# ---------------------------------------------------------------------------

def test_s2_mock_fake_llm_run():
    out_dir = Path(tempfile.mkdtemp(prefix="smart_house_bench_"))
    try:
        rc = compare_tiers.main([
            "--mock", "--fake-llm",
            "--scenarios", "s2_oven_left_on",
            "--out", str(out_dir),
        ])
        assert rc == 0
        results = json.loads((out_dir / "results.json").read_text(encoding="utf-8"))
        s2 = results["results"]["s2_oven_left_on"]
        normal, persistent = s2["normal"], s2["persistent"]

        # PERSISTENT detects the oven within 120 house-min of the incident.
        assert persistent["detection_latency_house_min"] is not None
        assert persistent["detection_latency_house_min"] <= 120, persistent

        # NORMAL cannot see it until the return-home turn (480 min away).
        assert normal["detection_latency_house_min"] is not None
        assert normal["detection_latency_house_min"] >= 400, normal

        # The energy story follows: 2.4 kW for 8 h vs ~1 h.
        assert persistent["energy_wh"] < normal["energy_wh"], (
            persistent["energy_wh"], normal["energy_wh"])
        assert normal["energy_wh"] - persistent["energy_wh"] > 10000

        # The kitchen physically overheated in the NORMAL arm; the
        # PERSISTENT arm caught it before it crossed the 35 C line the
        # contract calibrates against (the oven still ran ~1 wake period).
        assert normal["peak_temps"]["kitchen"] >= 35.0, normal["peak_temps"]
        assert persistent["peak_temps"]["kitchen"] < 35.0, persistent["peak_temps"]
        assert (normal["peak_temps"]["kitchen"]
                - persistent["peak_temps"]["kitchen"]) > 8.0

        # Both arms acted through measured device_log entries.
        assert any(a["id"] == "appliance.oven" for a in normal["agent_actions"])
        assert any(a["id"] == "appliance.oven" for a in persistent["agent_actions"])

        # Report artifacts exist.
        assert (out_dir / "report.md").exists()
        assert (out_dir / "transcript_normal_s2_oven_left_on.jsonl").exists()
        assert (out_dir / "transcript_persistent_s2_oven_left_on.jsonl").exists()
    finally:
        shutil.rmtree(out_dir, ignore_errors=True)


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    tests = [
        test_tool_specs_schema,
        test_confirm_required_refuse_client_side,
        test_mock_hub_contract,
        test_s2_mock_fake_llm_run,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS  {t.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL  {t.__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"ERROR {t.__name__}: {exc.__class__.__name__}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
