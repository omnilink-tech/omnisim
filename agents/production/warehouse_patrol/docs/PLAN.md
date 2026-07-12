# Warehouse Patrol — build plan

Iteration-by-iteration scope for the patrol demo. Mirrors the structure of `warehouse_foreman/README.md`'s build trajectory.

Read [`agents/AGENT_PATTERNS.md`](../../../AGENT_PATTERNS.md) before starting any iteration — it's the cross-demo cheat sheet.

## Iteration 0 — world + folder scaffold *(this commit)*

Goal: have a runnable warehouse with two huskies + decoy crates that can be moved between sweeps.

- [x] `warehouse_patrol.wbt` — same 30 × 18 m walled warehouse as `warehouse_logistics.wbt`, but:
  - No loading dock zone, no arm
  - Two `URDFRobot` huskies — `HUSKY_NORTH` parked at (-12, +5, 0.1), `HUSKY_SOUTH` at (-12, -5, 0.1)
  - Two `husky_eye` Robot sidecars, one per husky, each tracking its own `DEF`
  - Six `DEF CRATE_*` solids in scattered positions (the things that move between sweeps)
- [x] Agent folder scaffold — `agents/production/warehouse_patrol/` with README, profile.json placeholder, tools/ + scripts/ + docs/ skeleton.
- [x] Mark in [`DEMOS.md`](../../../../DEMOS.md) as iter-0-shipped.

## Iteration 1 — bridge port multiplexing

Goal: two `husky_omnilink_bridge` + two `husky_eye` instances coexist in the same OmniSim world on different ports.

- [x] `husky_omnilink_bridge.py` — accept `--port` and `--eye-port` controllerArgs; default to 6070/6071 if absent (preserves single-husky worlds).
- [x] `husky_eye.py` — accept `--port` controllerArgs; default to 6071. Also accept `--track-def` so the eye can track a specific `DEF` rather than always `HUSKY`.
- [x] World file passes the args:
  - `HUSKY_NORTH` controller bridge → port 6070, eye-port 6071
  - `husky_eye_north` → port 6071, track-def HUSKY_NORTH
  - `HUSKY_SOUTH` controller bridge → port 6080, eye-port 6081
  - `husky_eye_south` → port 6081, track-def HUSKY_SOUTH
- [x] Patch `WbUrdfImporter.cpp` so URDFRobot doesn't strip `controllerArgs` during import (discovered mid-iteration: without this, both huskies share port 6070 regardless of world-file config).
- [x] Verify both bridges respond to `/state` independently.

## Iteration 2 — patrol agent + sweep_summary tool

Goal: a single `Patrol Husky` profile that two runners (north + south) push to OmniLink with sector-specific environment overrides.

- [x] `warehouse_patrol_agent.py` runner — same shape as warehouse_picker_agent.py but reads `PATROL_BRIDGE_URL` (defaults to 6070) and `PATROL_SECTOR` env var (default "north"). Two runners can run concurrently on ports 51530 (north) + 51531 (south).
- [x] `tools/patrol.py` — the patrol-specific tool surface:
  - `drive_to_waypoint(x, y, look_at=...)` — same as picker
  - `scan_for_tag()` — same as picker
  - `sweep_summary(sector_name)` — composes the per-crate manifest by walking the sector's known waypoints, calling scan + `/solid?def=...` at each, returning `{sector, sim_time, observations: [...]}`. Calls `save_local_memory` internally with `tags=[patrol_sweep, sector]`.
  - `recall_last_sweep(sector_name)` — searches local memory by tags, sorts by `updated_at` desc, takes latest. Reads full manifest from disk via the `path` field (not `snippet`, which truncates at 400 chars).
  - `diff_sweeps(prior, current, moved_threshold_m=0.30)` — pure-Python comparator returning `{summary, diffs, headline}`.
- [x] Profile `mainTask` instructs: "(1) recall the prior sweep, (2) drive each waypoint in your sector calling scan_for_tag at each, (3) call sweep_summary to compose + persist this sweep's manifest, (4) call diff_sweeps to compare prior vs current, (5) narrate the diff via complete_mission rationale."

## Iteration 3 — end-to-end run, RESULTS.md

Goal: verify the cross-session memory dividend is real and document the cost.

- [x] Sweep #1: agent inspects all crates, persists manifest. Reports "baseline established" when recall returns no prior.
- [x] Move two crates by ~2 m via supervisor (`POST /admin/teleport_solid` on the bridge).
- [x] Sweep #2: recall returns baseline, sweep_summary writes new manifest, diff_sweeps detects movement. North sweep #4 narrated `3 crate(s) moved` correctly (BLUE 2.83 m + RED 2.83 m matching teleports + ORANGE 0.31 m incidental husky-bump). South sweep #2 narrated `1 crate(s) moved` (CYAN 2.5 m) correctly.
- [x] Document the memory persistence: runner process killed and restarted between sweeps; manifests survived in `long_term_memory/` and were re-indexed by SQLite on next search. Both north and south runners share one profile but persist sector-tagged manifests independently.
- [x] Capture verification trace + architecture + lessons in [`docs/RESULTS.md`](RESULTS.md).

## Pre-flight checklist (from AGENT_PATTERNS.md)

- [x] Ground-truth verification endpoint exists: `/solid?def=CRATE_NN` already provided by the husky bridge from the warehouse_foreman build
- [x] Perception sidecar: husky_eye + `scan_for_tag` already in place
- [x] Strict honesty contract in mainTask: "never claim a crate moved unless diff_sweeps says it did; never claim 'all clear' if you skipped diff_sweeps because recall failed"
- [x] Caching plumbed: shared via Supabase from warehouse_foreman work
- [x] Runner /status surfaces tokens/hour, cache hit %, and dollars/hour: inherit from picker runner
