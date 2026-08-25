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

"""Tests for the smart-house hub model behind ``smart_house_bridge``.

``house_model.py`` is deliberately pure stdlib (no ``omnisim`` import),
so it is tested directly with a fake SceneInterface — the same seam the
live bridge uses to drive the Supervisor. Covers the frozen demo
contract: device validation + fault codes, the thermal calibration
(oven curve, heater hold), energy integration and categories, the two
anomaly types, the scenario engine (canonical start times, start_time
override, changed_by attribution) and the sensors sweep.

Run with:
    pytest tests/harness/test_smart_house_bridge.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "projects" / "samples" / "demos"
                       / "controllers" / "smart_house_bridge"))

import house_model as hm  # noqa: E402
from house_model import (  # noqa: E402
    HouseModel,
    SceneInterface,
    parse_start_time,
    room_from_position,
)


class FakeScene(SceneInterface):
    """Records apply_* calls; serves a teleportable resident position."""

    ANCHOR = {
        "living_room": (-3.0, 2.5, 0.0),
        "kitchen": (2.5, 2.7, 0.0),
        "bedroom": (3.0, -1.5, 0.0),
        "hallway": (-2.5, -2.5, 0.0),
        "away": (0.0, 0.0, -10.0),
    }

    def __init__(self):
        self.calls = []
        self.pos = self.ANCHOR["living_room"]
        self.offline = set()

    def apply_light(self, dev_id, on, brightness):
        self.calls.append(("light", dev_id, on, brightness))

    def apply_switch(self, dev_id, on):
        self.calls.append(("switch", dev_id, on))

    def apply_heater(self, on):
        self.calls.append(("heater", on))

    def apply_door(self, is_open):
        self.calls.append(("door", is_open))

    def apply_blinds(self, is_open):
        self.calls.append(("blinds", is_open))

    def apply_security(self, armed):
        self.calls.append(("security", armed))

    def apply_resident(self, room_or_away):
        self.pos = self.ANCHOR[room_or_away]
        self.calls.append(("resident", room_or_away))

    def read_resident_position(self):
        return self.pos

    def is_online(self, dev_id):
        return dev_id not in self.offline


def make_model():
    scene = FakeScene()
    model = HouseModel(scene)
    model._last_sim_t = 0.0
    return model, scene


def run(model, sim_seconds, step_s=0.064):
    """Advance the model as the main loop would (default scale: 1 sim
    second == 1 house minute)."""
    t = model._last_sim_t
    end = t + sim_seconds
    while t < end:
        t += step_s
        model.advance(t)


# ── Defaults / reset ──────────────────────────────────────────────────

def test_reset_defaults():
    model, _ = make_model()
    assert model.house_time() == "2026-08-19T08:00:00"
    assert model.resident == "living_room"
    assert all(t == 20.0 for t in model.temps.values())
    for dev in hm.LIGHTS:
        assert model.devices[dev]["on"] is False
    assert model.devices["appliance.oven"] == "off"
    assert model.devices["thermostat.main"]["mode"] == "off"
    assert model.devices["security.system"] == "disarmed"
    assert model.devices["door.front"] == "closed"


# ── set_device validation + fault codes ──────────────────────────────

def test_set_device_light_forms():
    model, _ = make_model()
    assert model.apply_set_device("lights.hallway", "on", "agent") is None
    assert model.devices["lights.hallway"]["on"] is True
    assert model.apply_set_device(
        "lights.hallway", {"on": True, "brightness": 250}, "agent") is None
    assert model.devices["lights.hallway"]["brightness"] == 100
    err = model.apply_set_device("lights.hallway", 42, "agent")
    assert err["error"] == "state_rejected" and err["accepted"] is False


def test_set_device_unknown_is_device_offline():
    model, _ = make_model()
    err = model.apply_set_device("nonexistent.device", "on", "agent")
    assert err == {"accepted": False, "error": "device_offline",
                   "message": err["message"]}


def test_set_device_offline_scene_binding():
    model, scene = make_model()
    scene.offline.add("appliance.tv")
    err = model.apply_set_device("appliance.tv", "on", "agent")
    assert err["error"] == "device_offline"


def test_door_is_motorized_and_settable():
    # Contract update #2: the s3 persistent arm closes the breached door.
    model, scene = make_model()
    assert model.apply_set_device("door.front", "open", "agent") is None
    assert model.apply_set_device("door.front", "closed", "agent") is None
    assert ("door", True) in scene.calls and ("door", False) in scene.calls


def test_heater_is_internal():
    model, _ = make_model()
    err = model.apply_set_device("hvac.heater", "on", "agent")
    assert err["error"] == "state_rejected"


def test_unlock_and_disarm_need_authorization_via_set_device():
    model, _ = make_model()
    model.devices["lock.front_door"] = "locked"
    err = model.apply_set_device("lock.front_door", "unlocked", "agent")
    assert err["error"] == "authorization_required"
    model.devices["security.system"] = "armed"
    err = model.apply_set_device("security.system", "disarmed", "agent")
    assert err["error"] == "authorization_required"


def test_toggle_device():
    model, _ = make_model()
    assert model.apply_toggle_device("appliance.tv", "agent") is None
    assert model.devices["appliance.tv"] == "on"
    assert model.apply_toggle_device("appliance.tv", "agent") is None
    assert model.devices["appliance.tv"] == "off"
    err = model.apply_toggle_device("thermostat.main", "agent")
    assert err["error"] == "state_rejected"


def test_thermostat_clamped():
    model, _ = make_model()
    err, clamped = model.apply_adjust_thermostat(45.0, "heat", "agent")
    assert err is None and clamped is True
    assert model.devices["thermostat.main"]["target"] == 30.0
    err, _ = model.apply_adjust_thermostat(None, "tropical", "agent")
    assert err["error"] == "state_rejected"


def test_scenes():
    model, _ = make_model()
    err, affected = model.apply_scene("goodnight", "agent")
    assert err is None
    assert model.devices["security.system"] == "armed"
    assert model.devices["lock.front_door"] == "locked"
    assert model.devices["lights.bedroom_lamp"] == {"on": True, "brightness": 10}
    assert "thermostat.main" in affected
    err, _ = model.apply_scene("no_such_scene", "agent")
    assert err["error"] == "scene_unknown"


# ── Clock & thermal calibration (the contract's two named curves) ─────

def test_house_clock_scale():
    model, _ = make_model()
    run(model, 60.0)  # 60 sim-s @ scale 60 = 60 house-min
    assert model.house_time().startswith("2026-08-19T09:00")


def test_oven_curve_reaches_35C_within_two_house_hours():
    model, _ = make_model()
    assert model.scenario_start("s2_oven_left_on") is None
    model._last_sim_t = 0.0
    run(model, 120.0)  # 2 house-hours
    assert model.temps["kitchen"] >= 35.0
    assert model.temps["hallway"] > model.temps["living_room"]  # 20% spill
    assert model.resident == "away"


def test_heater_holds_21C_against_8C_outside():
    model, _ = make_model()
    model.outside_override = 8.0
    model.apply_adjust_thermostat(21.0, "heat", "agent")
    run(model, 360.0)  # 6 house-hours; sample the last 4
    model2_band = []
    t = model._last_sim_t
    for _ in range(2400):
        t += 0.1
        model.advance(t)
        model2_band.append(model.temps["hallway"])
    assert min(model2_band) >= 20.3 and max(model2_band) <= 21.8


def test_hysteresis_transitions_are_logged_as_hub():
    model, _ = make_model()
    model.outside_override = 8.0
    model.apply_adjust_thermostat(21.0, "heat", "agent")
    run(model, 400.0)
    heater_entries = [e for e in model.device_log if e["id"] == "hvac.heater"]
    assert len(heater_entries) >= 2  # at least one on + one off cycle
    assert all(e["changed_by"] == "hub" for e in heater_entries)


def test_eco_mode_setback():
    model, _ = make_model()
    model.outside_override = 8.0
    model.apply_adjust_thermostat(21.0, "eco", "agent")
    run(model, 600.0)
    # eco target = 21 - 4 = 17; the house must NOT be held at 21
    assert model.temps["hallway"] < 18.5


# ── Energy ────────────────────────────────────────────────────────────

def test_energy_integration_and_categories():
    model, _ = make_model()
    model.apply_set_device("appliance.oven", "on", "agent")
    model.apply_set_device("lights.hallway",
                           {"on": True, "brightness": 50}, "agent")
    run(model, 60.0)  # one house-hour
    assert model.energy_wh["appliance.oven"] == pytest.approx(2400.0, rel=0.02)
    assert model.energy_wh["lights.hallway"] == pytest.approx(20.0, rel=0.05)
    assert model.energy_wh["standby"] == pytest.approx(30.0, rel=0.02)
    cats = model.energy_by_category()
    assert cats["appliances"] == pytest.approx(2400.0, rel=0.02)
    assert cats["lighting"] == pytest.approx(20.0, rel=0.05)
    report = model.energy_report("all")
    assert report["outliers"] and report["outliers"][0]["id"] == "appliance.oven"


# ── Anomalies ────────────────────────────────────────────────────────

def test_door_open_while_armed_anomaly_lifecycle():
    model, _ = make_model()
    model.apply_set_device("security.system", "armed", "agent")
    model.apply_set_device("door.front", "open", "scenario")
    run(model, 1.0)
    assert [a["type"] for a in model.anomalies_active] == ["door_open_while_armed"]
    model.apply_set_device("door.front", "closed", "agent")
    run(model, 1.0)
    assert model.anomalies_active == []
    assert model.anomalies_history[0]["ended_house_time"] is not None


def test_energy_spike_needs_sustain_and_away():
    model, _ = make_model()
    # oven alone (2400 + 30 standby) must NEVER trip the spike
    model._set_resident("away", "scenario")
    model.apply_set_device("appliance.oven", "on", "scenario")
    run(model, 120.0)
    assert all(a["type"] != "energy_spike" for a in model.anomalies_history)
    # oven + heater > 3000 W: below 30 house-min sustained -> no spike yet
    model.apply_adjust_thermostat(30.0, "heat", "agent")
    run(model, 20.0)
    assert all(a["type"] != "energy_spike" for a in model.anomalies_active)
    run(model, 15.0)  # now sustained past 30 house-min
    assert any(a["type"] == "energy_spike" for a in model.anomalies_active)


def test_energy_spike_suppressed_when_home():
    model, _ = make_model()
    model.apply_set_device("appliance.oven", "on", "agent")
    model.apply_adjust_thermostat(30.0, "heat", "agent")
    run(model, 60.0)  # > 3 kW for an hour, but the resident is home
    assert all(a["type"] != "energy_spike" for a in model.anomalies_history)


# ── Scenario engine ──────────────────────────────────────────────────

def test_scenario_canonical_start_times():
    expect = {"s1_movie_night": "19:00", "s2_oven_left_on": "08:00",
              "s3_night_door": "20:00", "s4_morning_prep": "21:00"}
    for name, hhmm in expect.items():
        model, _ = make_model()
        assert model.scenario_start(name) is None
        assert model.house_time().endswith(f"T{hhmm}:00")


def test_scenario_start_resets_metrics_and_accepts_start_time():
    model, _ = make_model()
    model.notify("hello", "info", None, None)
    model.apply_set_device("appliance.tv", "on", "agent")
    assert model.scenario_start("s1_movie_night", start_time="18:15") is None
    assert model.house_time().endswith("T18:15:00")
    assert model.notifications == []
    # the log holds only the scenario's own resident placement
    assert all(e["changed_by"] == "scenario" for e in model.device_log)
    err = model.scenario_start("s1_movie_night", start_time="25:99")
    assert err["error"] == "state_rejected"
    err = model.scenario_start("nope")
    assert err["error"] == "state_rejected"


def test_s3_timeline_breaches_door_at_0210():
    # 02:10, deliberately BETWEEN hourly wakes (which land on :00), so the
    # benchmark's detection-latency claim is bounded by cadence, never
    # gifted by an incident coinciding with a wake.
    model, scene = make_model()
    assert model.scenario_start("s3_night_door") is None
    model._last_sim_t = 0.0
    assert model.resident == "bedroom"  # asleep at home, NOT away
    run(model, 375.0)  # 20:00 + 375 house-min > 02:10
    assert model.devices["security.system"] == "armed"
    assert model.devices["door.front"] == "open"
    door_evt = next(e for e in model.device_log
                    if e["id"] == "door.front" and e["state"] == "open")
    assert door_evt["changed_by"] == "scenario"
    assert door_evt["house_time"].startswith("2026-08-20T02:10")
    assert [a["type"] for a in model.anomalies_active] == ["door_open_while_armed"]


def test_s4_return_at_0730():
    model, scene = make_model()
    assert model.scenario_start("s4_morning_prep") is None
    model._last_sim_t = 0.0
    assert model.resident == "away"
    run(model, 631.0)
    assert model.resident == "hallway"
    assert model.house_time().startswith("2026-08-20T07:3")
    run(model, 5.0)
    assert model.resident == "kitchen"
    assert model.scenario_status()["phase"] == "complete"


def test_scenario_event_and_room_validation():
    model, _ = make_model()
    err = model.scenario_event({"type": "resident", "room": "atlantis"})
    assert err["error"] == "room_unknown"
    assert model.scenario_event(
        {"type": "device_set", "id": "appliance.oven", "state": "on"}) is None
    assert model.meta["appliance.oven"]["changed_by"] == "scenario"


# ── Sensors / misc ───────────────────────────────────────────────────

def test_read_sensors_motion_is_measured_from_position():
    model, scene = make_model()
    out = model.read_sensors()
    motion = {r["room_id"]: r["value"] for r in out["readings"]
              if r["type"] == "motion"}
    assert motion == {"living_room": True, "kitchen": False,
                      "bedroom": False, "hallway": False}
    model._set_resident("away", "scenario")
    out = model.read_sensors()
    assert all(r["value"] is False for r in out["readings"]
               if r["type"] == "motion")
    rooms = {r["room_id"] for r in out["readings"]}
    assert "outside" in rooms
    err = model.read_sensors("atlantis")
    assert err["error"] == "room_unknown"


def test_coffee_ready_after_five_house_minutes():
    model, _ = make_model()
    model.apply_set_device("appliance.coffee_maker", "on", "agent")
    run(model, 3.0)
    assert model.coffee_ready is False
    run(model, 3.0)
    assert model.coffee_ready is True
    model.apply_set_device("appliance.coffee_maker", "off", "agent")
    assert model.coffee_ready is False


def test_room_from_position():
    assert room_from_position(-3.0, 2.5, 0.0) == "living_room"
    assert room_from_position(2.5, 2.7, 0.0) == "kitchen"
    assert room_from_position(3.0, -1.5, 0.0) == "bedroom"
    assert room_from_position(-2.5, -2.5, 0.0) == "hallway"
    assert room_from_position(0.0, 0.0, -10.0) == "away"
    assert room_from_position(50.0, 0.0, 0.0) == "away"


def test_parse_start_time():
    assert parse_start_time("07:30") == 450.0
    assert parse_start_time("2026-08-20T02:00:00") == 24 * 60 + 120.0
    assert parse_start_time("25:99") is None
    assert parse_start_time("gibberish") is None
    assert parse_start_time(None) is None


def test_hold_clock_default_and_skip_to():
    model, _ = make_model()
    # Interactive default: free-running.
    assert model.clock_held is False
    assert model.scenario_status()["clock"] == "free_running"
    # A scenario holds the clock by default (benchmark mode).
    assert model.scenario_start("s2_oven_left_on") is None
    assert model.clock_held is True
    assert model.scenario_status()["clock"] == "held"
    # Held: the bridge calls skip_to between advance windows — house time
    # must not move, no scenario event may fire, and the anchor must stay
    # fresh so the next window opens without a time jump.
    model._last_sim_t = 0.0
    t0 = model.clock_hmin
    for t in range(1, 61):  # 60 sim-seconds pass while the model holds
        model.skip_to(float(t))
    assert model.clock_hmin == t0
    assert model._scenario_fired == 0
    # The advance window then integrates exactly its own dt.
    run(model, 30.0)  # 30 sim-s = 30 house-min inside the window
    assert model.clock_hmin - t0 == pytest.approx(30.0, abs=0.2)
    assert model.devices["appliance.oven"] == "on"  # +5 min event fired


def test_hold_clock_false_keeps_free_running():
    model, _ = make_model()
    assert model.scenario_start("s1_movie_night", hold_clock=False) is None
    assert model.clock_held is False
    assert model.scenario_status()["clock"] == "free_running"
    # /scenario/reset returns to the interactive default.
    assert model.scenario_start("s1_movie_night") is None
    assert model.clock_held is True
    model.reset()
    assert model.clock_held is False


def test_outside_temperature_sinusoid():
    assert hm.outside_temp_c(4 * 60.0) == pytest.approx(8.0)
    assert hm.outside_temp_c(16 * 60.0) == pytest.approx(14.0)
