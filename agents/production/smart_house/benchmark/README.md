# Smart-house tier benchmark — NORMAL vs PERSISTENT

Measures what a **persistent** agent (wakes every 60 house-minutes while the
occupant is unavailable — the OmniLink Builder cadence, honestly) is worth
over a **normal** interactive-only agent, on the same house, same scenarios,
same tools, same model. Every number is read from the simulator
(`POST /scenario/metrics`): device log, energy integral, temperature
timeline, notifications. Nothing is scored from what the agent says about
itself.

## Files

| file | what |
|---|---|
| `compare_tiers.py` | the benchmark driver (stdlib only) |
| `mock_hub.py` | contract-faithful offline hub — also a standalone dev server: `python mock_hub.py --port 8766` |
| `plan_walls_probe.py` | live demo of the plan entitlement walls (402 `WAKE_CADENCE_NOT_ON_PLAN`, 402 `PERSISTENT_AGENT_LIMIT_REACHED`), with mandatory cleanup — orchestrator-run only |
| `test_compare_tiers.py` | tests: tool-spec schemas, mock-hub contract conformance, one full offline s2 run. Runnable directly or under pytest |
| `results/` | run outputs (gitignored): `results.json`, `report.md`, transcripts, redacted API logs |

## Running

```bash
# Offline, deterministic, no key, no simulator, no network (CI mode):
python compare_tiers.py --mock --fake-llm

# One scenario:
python compare_tiers.py --mock --fake-llm --scenarios s2_oven_left_on

# Live LLM against the real Lane-A bridge (world running on :8766):
python compare_tiers.py --hub-url http://127.0.0.1:8766
# Live flags: --scenarios --arms --model --engine --base-url --out
# Refuses to run live without OMNI_KEY (env) or the local key file.
```

## Scenarios

| name | shape | what it measures |
|---|---|---|
| `s1_movie_night` | occupant present all evening | parity control — both arms must behave and cost the same |
| `s2_oven_left_on` | 2.4 kW oven left on, away 8 house-h | detection latency, energy burned, kitchen peak temp. `check_anomalies` deliberately does NOT report it — the agent must infer it from device state |
| `s3_night_door` | front door opens 02:10 while armed, occupant asleep | detection latency on a hub-flagged anomaly (`door_open_while_armed`) |
| `s4_morning_prep` | away overnight, announced 07:30 return | comfort at return (living-room temp, coffee) vs energy spent — the honest cost of pre-warming |

Detection latency = first agent-attributed `device_log` entry addressing the
incident (fallback: a matching notification), minus the incident timestamp.
The NORMAL arm's s2/s3 "detection" is whenever the returning-occupant turn
finally sees the problem — that is the point of the comparison.

## Reference result (mock + fake LLM, deterministic)

| scenario | metric | NORMAL | PERSISTENT |
|---|---|---|---|
| s1_movie_night | energy (Wh) | 3200.3 | 3200.3 |
| s2_oven_left_on | detection latency (house-min) | 480 | 60 |
| s2_oven_left_on | energy (Wh) | 20223.3 | 8466.7 |
| s2_oven_left_on | peak kitchen temp (C) | 45.94 | 32.15 |
| s3_night_door | detection latency (house-min) | 285 | 50 |
| s4_morning_prep | living room at return (C) | 16.7 | 19.8 |
| s4_morning_prep | energy (Wh) | 9555 | 10605 |

(s4 shows the honest trade: the persistent arm SPENDS more to deliver the
warm house + ready coffee the occupant asked for.)

## Contract deviations / assumptions (mirrored by the real bridge)

1. **`/scenario/start` gained an optional `start_time`** (`"HH:MM"` or ISO).
   The frozen contract only has `{name, seed?, time_scale?}`; scenarios need
   different clock starts (19:00 / 08:00 / 20:00 / 21:00). If the real
   bridge won't take it, the driver falls back cleanly: it reads the
   returned `house_time` and computes offsets from whatever it got — but
   energy/temp baselines then include a pre-roll if the bridge can't be
   told the start. Preferred fix: accept `start_time`.
2. **`set_device("door.front", "closed") is ACCEPTED** (motorized-door
   assumption) so the agent can close a breached door in s3. If the bridge
   keeps the door physical-only, s3's persistent response becomes
   notify-only and the detection matcher must switch to notifications.
3. **Wake windows generalise "away" to "unavailable"**: s3's occupant is
   home but asleep (goodnight → morning), and the persistent arm wakes
   during that window too. Each scenario declares its windows explicitly.
4. **`energy_spike` threshold** in the mock: total draw > 3000 W sustained
   > 30 house-min while away — deliberately NOT tripped by the oven alone
   (2430 W incl. standby), so s2 stays an agent-inference test. The bridge
   should keep the oven below whatever threshold it picks.
5. **Unknown device id** maps to fault code `device_offline` (closest of
   the six standardized codes).
6. **`list_rooms`/`list_devices` return bare JSON arrays** per the contract
   table; the tools' `hub_call` wraps a top-level array as
   `{"items": [...]}` so every tool RESULT is a dict.
7. `/api/chat` response text is read defensively from
   `message`/`content`/`text`/`response` — the contract only specifies
   `toolCalls`. The raw response is logged either way.
