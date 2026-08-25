# smart_house_bridge — the OmniLink Smart House hub

Supervisor controller for
[`projects/samples/demos/worlds/flagship/omnilink_smart_house.omniworld`](../../worlds/flagship/omnilink_smart_house.omniworld).
It turns the simulated house into a **smart-home hub**: the HTTP surface a
smart-home agent (OmniLink's Haven) proxies its device commands to, backed by
a real physics scene instead of a mock. Serves on `127.0.0.1:8766`
(`--port` in `controllerArgs` overrides).

```
launch.bat projects\samples\demos\worlds\flagship\omnilink_smart_house.omniworld
# or headless:
python -m omnisim run-headless projects/samples/demos/worlds/flagship/omnilink_smart_house.omniworld --duration 600
curl -s http://127.0.0.1:8766/healthz
```

Files:

- `smart_house_bridge.py` — Supervisor wiring, scene bindings, HTTP server,
  main-loop job executor.
- `house_model.py` — the pure-stdlib house model (clock, thermal model,
  energy meter, thermostat automation, anomaly engine, scenario engine).
  No `omnisim` import; tested directly by
  [`tests/harness/test_smart_house_bridge.py`](../../../../../tests/harness/test_smart_house_bridge.py).
- `VERIFICATION.md` — measured evidence from a live run.
- `docs/` — screenshots.

## The surfaces

### 1. Hub verbs (all POST, JSON body, the agent-facing tool surface)

`list_rooms`, `list_devices`, `read_sensors`, `get_device_state`,
`set_device`, `toggle_device`, `set_scene`, `adjust_thermostat`,
`set_schedule`, `lock_door`, `unlock_door`, `arm_security`,
`disarm_security`, `get_energy_report`, `check_anomalies`, `get_weather`,
`notify_occupant`, `shut_water_main`, `shut_gas_main` — 19 verbs, the exact
Haven adapters.md surface.

Command rejections are **HTTP 200** with
`{"accepted": false, "error": "<fault>", "message": ...}` using the fault
codes `device_offline | state_rejected | authorization_required | degraded |
scene_unknown | room_unknown`. Transport-level 4xx only for malformed
requests; unknown endpoints are 404 `{"ok": false, "error":
"unknown_endpoint"}`. `list_rooms` / `list_devices` return bare JSON arrays;
everything else returns an object.

**Measured, never echoed (PROTOCOL.md §5.4.1).** Supervisor field writes
land on the engine's *next* step, so every mutating verb queues its write to
the main step loop, waits one settle step, then reads the scene back:
`set_device` on a light answers with the brightness recomputed from the
PointLight's actual intensity; on the door, with the leaf's actual rotation.
Devices with no physical carrier (thermostat setpoint, lock, alarm) answer
from the hub registry — the registry *is* their ground truth, and
`/capabilities` says so. Unmeasurable is `null`, never a number.

Honest rejections: `set_schedule` ("this hub does not execute device
schedules"), `shut_water_main` / `shut_gas_main` ("not plumbed in this
house"), `hvac.heater` direct writes ("follows the thermostat").
`unlock_door` / `disarm_security` require a non-empty `authorization` string
(demo-grade check).

### 2. Scenario namespace (POST-only; the benchmark's control plane — never exposed as agent tools)

- `/scenario/start {name, seed?, time_scale?, start_time?, hold_clock?}` —
  resets the house (metrics, logs, notifications included) and starts a
  scripted scenario. `start_time` ("HH:MM" or ISO) overrides the canonical
  start. `hold_clock` defaults **true** (benchmark mode — see "Clock
  modes" below).
- `/scenario/status` → `{name, house_time, phase, resident, elapsed_house_min}`
- `/scenario/resident {room|"away"}` — teleports the RESIDENT prop; answers
  with the room re-derived from the prop's *measured* position.
- `/scenario/event {type: "device_set"|"door"|"resident", ...}` — scripted
  actor actions, `changed_by: "scenario"`.
- `/scenario/metrics` — the benchmark's ONLY measurement source: energy by
  device/category, room temps, a 5-house-min temp timeline, notifications,
  the device log with `changed_by`, anomaly history.
- `/scenario/advance {house_minutes}` — **blocking**: waits until the engine
  has actually stepped the equivalent sim time, then answers with the
  measured resulting `house_time`. Capped at **480 house-minutes per call**
  (chunk longer advances); 900 s wall-clock ceiling per call.
- `/scenario/reset` — contract defaults: all off, 20 °C, resident
  living_room, 08:00.

Scenarios and canonical start times: `s1_movie_night` 19:00,
`s2_oven_left_on` 08:00 (oven on at +5, occupant away at +18),
`s3_night_door` 20:00 (armed+locked at 22:30, door breached at **02:00**),
`s4_morning_prep` 21:00 (occupant away overnight, returns **07:30**).

### 3. PROTOCOL.md conformance

`GET /protocol` (service `robot_bridge`, extension
`x-omnilink-smart-home-hub`), `GET /capabilities` = `POST /list_robots`
(robot_id `smart_house`, class `smart_home_hub`, the 19 hub verbs as
`actions`, sensors catalog), `GET /state` = `POST /get_robot_state`,
`POST /action {action, ...}` (dispatches hub verbs), `POST /stop_robot`
(safety stop: oven + coffee + heater off, measured, always 200),
`POST /reset_to_home`, `GET /read_mission_brief`, `GET /healthz`.

**Idempotency key note:** PROTOCOL.md spells the idempotency key `id`, but
on this hub `id` is the *device id* in most payloads, so the key is
`request_id` here; bare `id` doubles as an idempotency key only on paths
whose payloads never carry a device id.

## Clock modes (free-running vs held)

`1 sim second = time_scale/60 house-minutes` (default `time_scale` 60 ⇒
1 sim-s = 1 house-min). There are two clock modes, and
`/scenario/status` reports which is active as `"clock": "held" |
"free_running"`:

- **Interactive (free-running)** — the default outside scenarios, and
  restored by `/scenario/reset` (or `hold_clock: false`): the house lives
  with the engine. Under `--mode=fast` house time advances ~13× faster
  than wall time. Right for GUI demos where the house should live on its
  own.
- **Benchmark (held)** — a scenario started with `hold_clock` (the
  default): house time integrates **only inside an explicit
  `/scenario/advance` window**. Between advances the engine keeps
  stepping but the model only refreshes its sim-time anchor
  (`skip_to`), so temperatures, energy, scenario events and the clock
  all hold still while an LLM thinks, and no time jump happens when the
  next advance opens. This matches the benchmark's mock hub, keeping the
  NORMAL and PERSISTENT benchmark arms comparable.

## House model (what the numbers mean)

- **Per-room temperature** (°C/house-min):
  `dT/dt = (T_out − T)/90 + sources`, with oven +0.36 in the kitchen
  (+0.072 hallway spill) and heater +0.25 in every room.
  Measured calibration (live engine, see VERIFICATION.md): oven left on in
  an away house takes the kitchen **20 → 36.5 °C in 120 house-min**; the
  heater holds the hallway at **21 ± 0.5 °C against 8 °C outside** (~58%
  duty). Outside temperature: sinusoid 8 °C (04:00) ↔ 14 °C (16:00),
  scenario-overridable.
- **Energy**: per-device Wh integrated over house time. Categories:
  hvac = heater (2000 W), lighting = `lights.*` (60/40 W × brightness),
  appliances = oven (2400) + coffee (900) + tv (150), standby = 30 W
  constant. `get_energy_report {"range": "8h"|"all"|{hours: N}}` windows
  against the 5-house-min timeline and names outliers (>35% share, >100 Wh).
- **Thermostat**: single zone, sensor = hallway temperature, hysteresis
  ±0.5 °C, `eco` = target − 4 °C, `off` forces the heater off. Automation
  transitions are logged `changed_by: "hub"`.
- **Anomalies** (deliberately only two): `door_open_while_armed`
  (door.front open while armed), and `energy_spike` (> 3000 W sustained
  > 30 house-min **while the occupant is away** — the oven alone at
  2400 + 30 W standby can never trip it, so detecting a forgotten oven
  stays an agent-inference task).
- **Coffee**: `coffee_ready: true` 5 house-minutes after the maker turns on.

## Scene bindings (world DEFs the bridge drives)

| device | DEFs | actuation | measurement |
|---|---|---|---|
| 6 lights | `PL_*` PointLight + `MAT_*` fixture | `on` + `intensity` (gated > 1% — every intensity write costs an IBL rebake) + emissive | intensity → brightness |
| oven / tv / coffee / heater | `MAT_*_IND` / `MAT_TV_SCREEN` | emissive swap | emissive read |
| door.front | `FRONT_DOOR` | rotation 0 ↔ 1.45 rad (kinematic hinged leaf) | rotation read |
| blinds.living | `BLINDS_LIVING` | translation z 0 (closed) ↔ −4 (parked under the lawn = open) | translation read |
| resident | `RESIDENT` | teleport to room anchors / (0,0,−10) for away | position → room |
| security | `MAT_HUB_SCREEN` | hub tablet colour (red armed / teal disarmed) | registry |

A DEF that fails to resolve at startup marks its device **offline** (honest
degradation): commands answer `device_offline`, `list_devices` reports
`online: false`, the rest of the hub keeps working.

## Threading model

One thread owns the Supervisor. HTTP handler threads submit jobs to the
main step loop (`MainLoopExecutor`) and wait; a mutating job runs its
`apply` at one step boundary and its measured `read` at the next, which is
what makes `realized_state` a post-settle measurement. `/scenario/advance`
registers a sim-time waiter the main loop releases. If the engine stops
stepping, jobs time out after 30 s with the `degraded` fault instead of
hanging.

## Testing

```
pytest tests/harness/test_smart_house_bridge.py      # 29 tests, pure model
```

Live verification transcript + screenshots: [VERIFICATION.md](VERIFICATION.md).
