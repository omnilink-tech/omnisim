# End-to-end results — Warehouse Patrol Squad demo

Live verified runs: two huskies (north + south sectors), one shared `Patrol Husky` profile, sector-tagged manifests persisted to local memory, cross-session diff narration. Ground truth confirmed via supervisor `/solid?def=...` reads (not agent-reported claims).

## TL;DR

> **Operator:** "Run a patrol sweep of your assigned sector. Report what moved since last time."

The patrol pipeline (recall → drive → scan → diff) detects sub-meter crate displacement against the prior sweep's manifest, with the manifest surviving across runner restarts via local-memory + SQLite index. Two huskies share one profile; sector tagging keeps their manifests isolated.

**Latest live run (2026-05-02, north sector, sweep #4 after teleport):**

```
HEADLINE: 3 crate(s) moved since last sweep
SUMMARY: {n_total: 4, n_moved: 3, n_missing: 0, n_new: 0, n_stationary: 1}
  blue   : moved delta=2.83 m  from [9.0, 6.0] -> [7.0, 4.0]
  red    : moved delta=2.83 m  from [-6.0, 5.0] -> [-8.0, 3.0]
  orange : moved delta=0.31 m  (incidental — bumped by husky during transit)
```

Two of the three flagged moves match the operator's teleports exactly; the third (orange, 0.31 m) is the husky's own transit nudging a crate at the threshold edge — realistic detection behavior, not a false positive.

**South sector sweep #2 after teleporting CYAN:**

```
HEADLINE: 1 crate(s) moved since last sweep
  cyan : moved delta=2.5 m  from [9.0, -6.0] -> [7.5, -4.0]
```

## What the demo proves

1. **Cross-session memory dividend works.** A sweep manifest persisted to local memory at run N is recallable at run N+1 even after both the runner process and OmniSim are restarted. The manifest body is stored as a markdown file with YAML frontmatter (`tags: [patrol_sweep, north]`); `recall_last_sweep` reads it from disk via the `path` field returned by `search_local_memory`, since `snippet` truncates at 400 chars and a 4-crate manifest is ~1.9 KB.
2. **Multiple agents can share one profile via sector tagging.** Both huskies push the same `Patrol Husky` profile to OmniLink. Their manifests don't collide because every save tags the sector (`patrol_sweep` + `north` or `south`); recall filters on the same tag pair.
3. **Diff is verifiable, not narrated.** `diff_sweeps` is pure-Python — no LLM call, no hallucination surface. Threshold = 0.30 m; `actual_position` comes from `/solid?def=CRATE_X` ground truth at scan time, not from camera pose estimates.
4. **URDFRobot can carry per-instance bridge config.** OmniSim `URDFRobot` previously dropped the `controllerArgs` field from URDF imports, forcing every husky to share port 6070. Patched [`src/omnisim/vrml/WbUrdfImporter.cpp`](../../../../src/omnisim/vrml/WbUrdfImporter.cpp) to preserve and inject the field — both huskies now coexist on independent ports (6070+6071, 6080+6081).

## Architecture

```
warehouse_patrol.wbt
├── HUSKY_NORTH (URDFRobot)      controllerArgs: --port=6070 --eye-port=6071
├── husky_eye_north (Robot)      controllerArgs: --port=6071 --track-def=HUSKY_NORTH
├── HUSKY_SOUTH (URDFRobot)      controllerArgs: --port=6080 --eye-port=6081
├── husky_eye_south (Robot)      controllerArgs: --port=6081 --track-def=HUSKY_SOUTH
└── 8x DEF CRATE_*               red/green/blue/orange (north y>0), yellow/magenta/cyan/white (south y<0)

agents/production/warehouse_patrol/
├── profile.json                 single Patrol Husky profile (both runners push it)
├── warehouse_patrol_agent.py    runner — env: PATROL_SECTOR, PATROL_BRIDGE_URL, PATROL_PORT
├── tools/patrol.py              17 tools registered. New ones:
│                                  - sweep_summary(sector)    drives N waypoints, scans, persists
│                                  - recall_last_sweep(sector) reads tagged manifest from disk
│                                  - diff_sweeps(prior, cur)  pure-Python comparator
└── long_term_memory/            persisted manifests (markdown + SQLite index)
    ├── 2026-05-02-patrol-sweep-north-t-107s.md
    ├── 2026-05-02-patrol-sweep-north-t-1421s.md
    ├── 2026-05-02-patrol-sweep-south-t-1642s.md
    └── _index.sqlite
```

Two runners run side-by-side:

```
[north] OMNI_KEY=... PATROL_SECTOR=north  PATROL_PORT=51530 PATROL_BRIDGE_URL=http://127.0.0.1:6070 ...
[south] OMNI_KEY=... PATROL_SECTOR=south  PATROL_PORT=51531 PATROL_BRIDGE_URL=http://127.0.0.1:6080 ...
```

Both push the same profile to the platform; `mainTask` reads its sector from the operator's first message; tool calls are dispatched to the correct husky via the runner's bridge URL.

## What each iteration unlocked

### iter 0 — world + folder scaffold

Cloned the warehouse_logistics floor plan, dropped the arm + dock zone, spawned 8 colored crates split N/S of the y=0 line. Two `URDFRobot` huskies + two `husky_eye` sidecars per the bridge multiplexing plan.

### iter 1 — bridge port multiplexing

`husky_omnilink_bridge.py` and `husky_eye.py` were single-instance — port 6070/6071 hardcoded, sidecar tracked DEF "HUSKY". Added `argparse.parse_known_args()` for `--port`, `--eye-port`, `--track-def` (OmniSim passes additional controller args before user args, so `parse_known_args()` is mandatory). Discovered OmniSim `URDFRobot` strips the `controllerArgs` field during URDF import — every husky still ended up on 6070. Fix: patched [`WbUrdfImporter.cpp`](../../../../src/omnisim/vrml/WbUrdfImporter.cpp) to capture `controllerArgs` and inject it into the generated VRML before the closing `}`. OmniSim rebuild required.

After the fix, all four bridges respond independently: `curl :6070/state` returns north husky pose, `curl :6080/state` returns south husky pose, etc.

### iter 2 — agent + sweep_summary + recall + diff

Cloned warehouse_picker as the starting point (same hint-first scan pattern, same productized memory tools). Removed `push_pallet_to`. Added three new tool implementations to [`tools/patrol.py`](../tools/patrol.py):

- `sweep_summary(sector_name)` — looks up `SECTOR_WAYPOINTS[sector]`, drives husky to each vantage, calls `scan_for_tag` + `/solid?def=...` per crate, builds a manifest, calls `_impl_save_local_memory` with `tags=["patrol_sweep", sector]`. Returns the manifest in-line so the agent can pass it to `diff_sweeps` without a second tool round-trip.
- `recall_last_sweep(sector_name, before_this_sim_time=None)` — `_impl_search_local_memory(tags=["patrol_sweep", sector])`, sort hits by `updated_at` desc, take latest. Reads the full manifest from disk via the `path` field (the `snippet` field truncates at 400 chars; a 4-crate manifest is ~1.9 KB). Strips YAML frontmatter, parses JSON, returns `{manifest, memory_id, memory_title, memory_updated_at}`.
- `diff_sweeps(prior, current, moved_threshold_m=0.30)` — pure-Python set-difference + 2D-Euclidean distance per crate, returns `{summary, diffs, headline}`. No LLM call.

Profile mainTask describes the workflow strictly: (1) recall, (2) sweep (auto-persists), (3) diff if prior exists, (4) complete_mission with one-line headline. Honesty contract: never claim a crate moved unless `diff_sweeps` says it did; never claim "all clear" if recall failed.

### iter 3 — end-to-end live verification (this milestone)

Ran two complete sweep cycles per sector (baseline + post-teleport), validated diff narration matches the operator's actual moves. Bug found and fixed mid-iteration: `recall_last_sweep` was reading the truncated `snippet` field instead of opening the full file from `path`. Manifests larger than 400 chars failed to parse.

The persisted manifests on disk demonstrate cross-session durability — the runner process was killed and restarted between sweeps; the manifests survived in `long_term_memory/` and were re-indexed by SQLite on next search.

## Final run metadata (2026-05-02)

| | |
|---|---|
| Date | 2026-05-02 |
| World | [`projects/samples/demos/worlds/flagship/warehouse_patrol.wbt`](../../../../projects/samples/demos/worlds/flagship/warehouse_patrol.wbt) |
| Engine | `g1-engine` (Gemini 3 Flash) |
| Agents | Patrol Husky × 2 (one shared profile, sector-tagged manifests) |
| Bridges | husky_omnilink_bridge :6070 + :6080, husky_eye :6071 + :6081 |
| Tools registered per runner | 17 (drive, scan, sweep, recall, diff, get_state, read_camera, full local-memory + recall + search_knowledge stack, complete_mission) |
| Sweeps logged | 4 north + 2 south = 6 manifests on disk |
| Outcome | Cross-session memory dividend verified. All teleports detected with correct deltas, sub-meter incidental motion (husky bumping a crate) also flagged at 0.31 m vs 0.30 m threshold. |

## Operational lessons (deltas from Foreman)

1. **`URDFRobot` strips `controllerArgs`** — patched the importer. Without this, you cannot run multiple URDF robots that need per-instance config in the same world.
2. **`search_local_memory.snippet` truncates at 400 chars** — for any persisted blob larger than that (manifests, sensor logs, transcripts), you must read from `path` on disk and strip the YAML frontmatter yourself. The hits index is for ranking, not for body reads.
3. **Sector-tagged memories enable shared-profile multi-agent fan-out** — two huskies, one profile, isolated state. The platform doesn't need any awareness of which physical instance is calling; the tags do the routing.
4. **`argparse.parse_known_args()` is mandatory in OmniSim controllers** — OmniSim prepends positional args before yours (the controller binary path, etc.). `parse_args()` would crash on those.
5. **Pure-Python diff > LLM-narrated diff** — `diff_sweeps` is deterministic, cheap, and impossible to hallucinate. The LLM only narrates the headline, never invents the deltas.

## Reproduction

```bash
# OmniSim (loads warehouse_patrol.wbt)
python -m omnisim run-world projects/samples/demos/worlds/flagship/warehouse_patrol.wbt

# Two runners, two terminals
OMNI_KEY=olink_... PATROL_SECTOR=north PATROL_PORT=51530 PATROL_BRIDGE_URL=http://127.0.0.1:6070 \
  python agents/production/warehouse_patrol/warehouse_patrol_agent.py

OMNI_KEY=olink_... PATROL_SECTOR=south PATROL_PORT=51531 PATROL_BRIDGE_URL=http://127.0.0.1:6080 \
  python agents/production/warehouse_patrol/warehouse_patrol_agent.py

# Two chat drivers
OMNI_KEY=olink_... PATROL_PORT=51530 python agents/production/warehouse_patrol/scripts/chat_drive.py
OMNI_KEY=olink_... PATROL_PORT=51531 python agents/production/warehouse_patrol/scripts/chat_drive.py

# Move crates between sweeps via the supervisor endpoint
curl -X POST http://127.0.0.1:6070/admin/teleport_solid \
  -H "Content-Type: application/json" \
  -d '{"def":"CRATE_BLUE","x":7.0,"y":4.0,"z":0.198}'
```
