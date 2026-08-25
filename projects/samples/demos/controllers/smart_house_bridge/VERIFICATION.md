# smart_house_bridge — measured verification (2026-08-19)

Machine: RTX 3060 laptop (`9722d23d12a3`), Windows. Engine:
`msys64/mingw64/bin/omnisim-bin.exe`, Newton/MuJoCo CPU `mj_step`.
World: `projects/samples/demos/worlds/flagship/omnilink_smart_house.omniworld`.
Every response below is a verbatim live response from the bridge on
`127.0.0.1:8766` while the world ran under
`python -m omnisim run-headless <world> --duration 900` (fast mode).

## 1. Load checks

```
python -m omnisim run-headless <world> --until-finalized --fail-on-warning
[headless] Results: 0 errors, 0 warnings
[headless] PASS

python -m omnisim run-headless <world> --duration 12 --fail-on-runaway
[headless] Results: 0 errors, 0 warnings
[headless] Runaway check: 1375 samples to t=207.50s; top-level dynamic bodies tracked: 1 (BOOK); ...
[headless] PASS
```

The one dynamic body (the book, released at z=0.75 over the bed) settles at
**z = 0.577** on the bed's compound collider — measured via the harness's
`GET /scene/node/BOOK`.

## 2. Protocol + identity

```
== GET /healthz
{"ok": true, "service": "smart_house_bridge", "sim_time": 208.384}
== GET /protocol
{"ok": true, "omnisim_wire": "1.0", "service": "robot_bridge", "service_versions": {"robot_bridge": "1.0", "smart_home_hub": "1.0"}, "instance": {"name": "smart_house_bridge", "robot_id": "smart_house", "world": "O:/omnisim/projects/samples/demos/worlds/flagship/omnilink_smart_house.omniworld", "pid": 12492}, "extensions": ["x-omnilink-smart-home-hub"]}
== GET /state
{"ok": true, "robot_id": "smart_house", "sim_time": 215.296, "last_tick_at": 215.296, "mode": "hub", "fault": null, "state_source": "sim", "house_time": "2026-08-19T11:35:16", "scenario": null, "resident": "living_room"}
```

`GET /capabilities` returns class `smart_home_hub`, the 19 hub verbs as
`actions`, the sensors catalog (5 temperature + 4 motion), the device and
room inventories, and the honesty notes.

## 3. Devices: measured realized_state, faults, authorization

```
== POST /set_device {"id": "lights.kitchen_ceiling", "state": {"on": true, "brightness": 60}}
   {"accepted": true, "realized_state": {"on": true, "brightness": 60.0}}    <- read back from the PointLight (intensity 4.8/8.0), not echoed
== POST /toggle_device {"id": "appliance.tv"}
   {"accepted": true, "new_state": "on"}                                     <- read back from the screen emissive
== POST /set_device {"id": "door.front", "state": "open"}
   {"accepted": true, "realized_state": "open"}                              <- read back from the leaf rotation
== POST /get_device_state {"id": "door.front"}
   {"id": "door.front", "state": "open", "last_change": "2026-08-19T14:45:08", "changed_by": "agent", "online": true}
== POST /set_device {"id": "nonexistent.device", "state": "on"}
   {"accepted": false, "error": "device_offline", "message": "unknown device id 'nonexistent.device'"}
== POST /unlock_door {}
   {"accepted": false, "error": "authorization_required", "message": "unlock_door requires an 'authorization' token"}
== POST /unlock_door {"authorization": "owner-pin-1234"}
   {"accepted": true, "state": "unlocked"}
== POST /adjust_thermostat {"target": 45, "mode": "heat"}
   {"accepted": true, "target": 30.0, "mode": "heat", "clamped": true, "heater": "on"}
== POST /set_scene {"scene": "movie"}
   {"accepted": true, "scene": "movie", "affected": [{"id": "lights.living_ceiling", "state": {"on": true, "brightness": 10.0}}, {"id": "lights.living_lamp", "state": {"on": true, "brightness": 10.0}}, {"id": "blinds.living", "state": "closed"}, {"id": "appliance.tv", "state": "on"}]}
== POST /set_scene {"scene": "no_such_scene"}
   {"accepted": false, "error": "scene_unknown", ...}
== POST /read_sensors {"room": "atlantis"}
   {"accepted": false, "error": "room_unknown", ...}
== POST /set_schedule {...}
   {"accepted": false, "error": "state_rejected", "message": "this hub does not execute device schedules"}
== POST /shut_water_main {}
   {"accepted": false, "error": "state_rejected", "message": "not plumbed in this house"}
== POST /some_bogus_endpoint {}
   [404] {"ok": false, "error": "unknown_endpoint", "path": "/some_bogus_endpoint"}
== POST /stop_robot {}
   {"accepted": true, "ok": true, "halted_at": 408.416, "safety_stop": {"appliance.oven": "off", "appliance.coffee_maker": "off", "hvac.heater": "off"}}
```

## 4. Scenario s2 (oven left on) — thermal + energy calibration, live

```
== POST /scenario/start {"name": "s2_oven_left_on"}
   {"accepted": true, "ok": true, "name": "s2_oven_left_on", "house_time": "2026-08-19T08:00:00", "time_scale": 60.0, "resident": "kitchen"}
== POST /scenario/advance {"house_minutes": 30}
   {"ok": true, "house_time": "2026-08-19T08:30:41", "advanced_house_min": 30.016, "sim_seconds_stepped": 30.016}
== POST /scenario/advance {"house_minutes": 90}
   {"ok": true, "house_time": "2026-08-19T10:01:54", "advanced_house_min": 90.016, "sim_seconds_stepped": 90.016}
== POST /scenario/metrics (after +120 house-min)
   kitchen temp: 36.45   hallway: 17.6   living: 12.89        <- kitchen 20 -> 36.45 degC in 2 house-hours (contract: 35+); 20% hallway spill visible
   energy_wh_total: 4739.38   oven Wh: 4678.4                 <- 2400 W integrated over ~1.95 house-hours
   resident: away
   device_log: oven on at 08:05 (scenario), door open/closed 08:17/08:18, resident away 08:18 — all changed_by "scenario"
   timeline samples: 25                                       <- 5-house-min cadence
== POST /read_sensors {"room": "kitchen"}
   {"readings": [{"room_id": "kitchen", "type": "temperature", "value": 36.46, ...}, {"room_id": "kitchen", "type": "motion", "value": false, ...}]}   <- motion measured from the RESIDENT prop position (away)
== POST /get_energy_report {"range": "2h"}
   {"total_kwh": 4.7414, "by_category": {"hvac": 0.0, "lighting": 0.0, "appliances": 4.6829, "standby": 0.0585}, "outliers": [{"id": "appliance.oven", "wh": 4682.9, "share": 0.988}], "window_house_min": 117.1, ...}
== POST /check_anomalies
   {"active": [], "history": [], ...}                          <- oven alone (2430 W) correctly does NOT trip energy_spike
```

Heater hold (model-level, `pytest` + fake scene): thermostat 21 °C heat
against a forced 8 °C outside holds the hallway in the **20.5–21.5 °C**
hysteresis band, heater duty ~58% (7.1 kWh over 6 house-hours).

## 5. Scenario s3 (night door) — anomaly lifecycle

```
== POST /scenario/start {"name": "s3_night_door"}
   {"accepted": true, "ok": true, ..., "house_time": "2026-08-19T20:00:00", "resident": "bedroom"}
== POST /scenario/advance {"house_minutes": 365}
   {"ok": true, "house_time": "2026-08-20T02:05:05", ...}
== POST /check_anomalies
   {"active": [{"type": "door_open_while_armed", "id": "door.front", "started_house_time": "2026-08-20T02:00:00", "ended_house_time": null}], ...}
== POST /set_device {"id": "door.front", "state": "closed"}      <- the agent's fix: the door is motorized
   {"accepted": true, "realized_state": "closed"}
== POST /check_anomalies
   {"active": [], "history": [{"type": "door_open_while_armed", ..., "ended_house_time": "2026-08-20T02:05:16"}], ...}
== POST /disarm_security {}
   {"accepted": false, "error": "authorization_required", ...}
== POST /disarm_security {"authorization": "owner-pin-1234"}
   {"accepted": true, "state": "disarmed"}
== POST /scenario/advance {"house_minutes": 600}
   {"accepted": false, "error": "state_rejected", "message": "advance is capped at 480 house-minutes per call; chunk longer advances"}
```

## 6. energy_spike anomaly (sustain + away gating)

```
s2 started, advanced past away (+25), then adjust_thermostat 30/heat -> oven 2400 + heater 2000 = 4430 W:
   after 20 house-min > 3 kW:  active []                       <- 30-house-min sustain not yet met
   after 40 house-min > 3 kW:  {"active": [{"type": "energy_spike", "watts": 4430.0, "started_house_time": "2026-08-19T08:25:44", ...}]}
```

## 7. Screenshots + render stats (harness, light mode)

- `docs/smart_house_hero.png` — baked hero viewpoint; render_stats
  `mean_brightness 106.8, saturated 0.56%, black 0.0%`, no warnings.
- `docs/smart_house_living.png` — oblique interior view over the walls
  (living room, RESIDENT visible); render_stats
  `mean_brightness 172.8, saturated 4.5%, black 0.0%`, no warnings.

Trap worth recording: `/scene/frame {"def": "RESIDENT", "mode": "front"}`
puts the camera at person height *outside the house* staring at an exterior
wall — frame modes cannot see through walls, so interior shots need an
elevated `/scene/look_at` over the (deliberately absent) ceiling.

## 8. Model self-test

```
pytest tests/harness/test_smart_house_bridge.py
31 passed in 0.43s
```

## 9. Clock modes — held vs free-running (integration fix, live)

The house clock used to free-run with the engine unconditionally (~13–32
house-min per wall-second in fast mode), which would let uncontrolled time
elapse during every LLM round-trip of a benchmark turn. `/scenario/start`
now holds the clock by default (`hold_clock: true`): house time integrates
only inside an explicit `/scenario/advance` window, while `skip_to()` keeps
the sim-time anchor fresh between windows. Live verbatim, same engine
session, world under `run-headless --duration 600` (fast mode):

```
== POST /scenario/start {"name": "s2_oven_left_on"}
   {"accepted": true, "ok": true, ..., "house_time": "2026-08-19T08:00:00", "clock": "held"}
== POST /scenario/status                       <- then 6 WALL seconds of sleep (fast mode would free-run 75+ house-min)
   {"..., "house_time": "2026-08-19T08:00:00", "elapsed_house_min": 0.0, "clock": "held"}
== POST /scenario/status
   {"..., "house_time": "2026-08-19T08:00:00", "elapsed_house_min": 0.0, "clock": "held"}     <- HELD: unchanged
== POST /scenario/advance {"house_minutes": 30}
   {"ok": true, "house_time": "2026-08-19T08:30:00", "advanced_house_min": 30.0, "sim_seconds_stepped": 30.016, "clock": "held"}
                                                <- EXACTLY 30.0 (free-run bleed used to make this 30.016+); s2's +5/+18 events fired inside the window (resident "away", phase "complete")
== POST /scenario/status                       <- after 5 more wall seconds
   {"..., "house_time": "2026-08-19T08:30:00", ...}                                           <- still held at the advance target
== POST /scenario/reset
   {"accepted": true, "ok": true, "house_time": "2026-08-19T08:00:00", ...}
== POST /scenario/status  x2, 4 wall-seconds apart
   "2026-08-19T08:00:27" -> "2026-08-19T10:10:06", "clock": "free_running"                    <- reset restores free-run (130 house-min in 4 wall-s, fast mode)
```

Post-change re-checks: `pytest tests/harness/test_smart_house_bridge.py`
**31 passed** (2 new held-clock tests), and
`run-headless --until-finalized --fail-on-warning` **PASS, 0 errors,
0 warnings**.
