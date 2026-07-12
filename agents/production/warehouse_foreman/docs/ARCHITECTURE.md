# Warehouse Foreman — system architecture

How the agent fabric, the bridges, the OmniSim world, and the cost-control plumbing fit together. This is the reference for "what is this demo actually made of, and why is each piece shaped the way it is" — useful both for hand-off and for cribbing patterns into the next demo (Patrol Squad, Drone Surveyor, etc.).

## One-line summary

> Two OmniLink agents (Foreman + Picker) collaborate over HTTP to drive one OmniSim robot (Husky) on `warehouse_logistics.wbt` in response to a single English sentence from the operator. End-to-end ground-truth verified, $1.50/hr at Gemini 3 Flash list pricing with cache discount.

## Process topology

Four processes run on the operator's machine; nothing is in the cloud except the OmniLink chat() endpoint.

```
                    +--------------------------------------------------+
                    |  OmniSim (warehouse_logistics.wbt)               |
                    |                                                   |
  ┌──────────┐      |  ┌────────────────────────────────────────────┐  |
  │ HUD pane │◀─── HTTP poll /status ────────────────────────────┐  |  |
  │ (WbAgent │      |  │  husky_omnilink_bridge      :6070  ──┐ │  |  |
  │  Hud)    │      |  │  (supervisor: husky URDF +          │ │  |  |
  └──────────┘      |  │   load_red/green/.../cyan,          │ │  |  |
                    |  │   /state /capabilities /action      │ │  |  |
                    |  │   /camera /scan /solid /mission)    │ │  |  |
                    |  ├─────────────────────────────────────┘ │  |  |
                    |  │  husky_eye sidecar          :6071  ──┐ │  |  |
                    |  │  (supervisor: front_camera +         │ │  |  |
                    |  │   _analyze_bgra → 6-color tag        │ │  |  |
                    |  │   classifier; /image /scan)          │ │  |  |
                    |  └─────────────────────────────────────┘ │  |  |
                    +──────────────────────────────────────────│──+  |
                                                                │     |
                                                                ▼     ▼
   chat_drive ──┐                                      ┌─ Picker runner :51520
   (operator)   │  HTTP POST /tool                      │  pushes "Warehouse Picker"
                │                                       │  profile to OmniLink, dispatches
                ▼                                       │  picker tools against bridges
   Foreman runner :51521 ──delegate_to_agent─→         │
   pushes "Warehouse Foreman"                          │
   profile to OmniLink, runs sub-chat-loops            │
   against the Picker specialist                       │
                │
                │  HTTP chat()
                ▼
        OmniLink platform (api.omnilink-agents.com)
        - LLM engine with explicit context caching (Gemini 3 Flash)
        - two-tier cache (process-local map → durable shared store)
        - meters cached vs. fresh input tokens per Omni Key
```

Four sockets to remember: `:6070 :6071` are the two OmniSim-side bridges (husky + eye), `:51520 :51521` are the agent-side runners. The HUD (in OmniSim) polls `:51521`. Both runners + chat_drive talk to `api.omnilink-agents.com`.

## The agents

### Warehouse Foreman *(orchestrator)*

[`warehouse_foreman/`](../). 9 tools, no robot of its own. Sole job: take an operator sentence, decompose into ordered legs, delegate each via `delegate_to_agent`, audit the returns, compose a final `complete_mission` claim.

Key shape decisions:
- **Picks specialists, doesn't drive robots.** The Foreman has no `drive_to_waypoint` or `set_tcp_target`. The hardware-control surface lives in the specialists. Keeps the orchestrator small enough to fit cleanly in a single `mainTask` and means the Foreman's profile doesn't need to change when a specialist gains a new tool.
- **Local sub-chat-loops, not server-side delegation.** [`tools/orchestration.py`](../tools/orchestration.py)'s `delegate_to_agent` runs the specialist's chat loop in the Foreman's own process (the platform can't reach `127.0.0.1:51520` from the internet). Each turn: chat → dispatch tool → check `/status` for completion → repeat. Exits when the specialist's `complete_calls_this_session` increments OR after 2 consecutive no-tool-call turns (graceful handling for specialists that have no `complete_mission` tool).
- **Strict honesty contract.** `mainTask` enforces: "if any leg's `success` is `false`, NEVER call `complete_mission` — investigate via `query_agent_status`, optionally re-delegate, or report the failure honestly." This was a hard lesson from an earlier iteration where the Foreman claimed success based on agent narration while the pallet was still in the rack.

### Warehouse Picker *(husky specialist)*

[`warehouse_picker/`](../../warehouse_picker/). 15 tools. Drives the Clearpath Husky on the warehouse via three primitives:

- `drive_to_waypoint(x, y, look_at=[lx, ly])` — continuous-space goto with optional teleport-snap-to-face on arrival
- `scan_for_tag()` — **PREFERRED** vision call; returns the eye sidecar's structured `{tag_color, marker_fraction, color_fractions, ...}` digest
- `read_camera()` — fallback PNG snapshot for ambiguous frames
- `push_pallet_to(source_x, source_y, target_x, target_y)` — physical shove with axis-aligned segments + side-detour staging + ground-truth verification

The push tool is the most non-trivial piece. Documented separately in the [push physics](#push_pallet_to-internals) section below.

## The world

[`projects/samples/demos/worlds/flagship/warehouse_logistics.wbt`](../../../../projects/samples/demos/worlds/flagship/warehouse_logistics.wbt) — a 30 × 18 m walled warehouse with:

- **Six pallet "loads"** in a 2 × 3 grid at `(±3, ±5)` and `(±9, ±5)`. Each load is a single `Solid` containing a `WoodenPallet` visual + a coloured emissive cube `Pose` for the tag, with a single `boundingObject` and `Physics { mass 15 }`. The whole load behaves as one rigid body — when the husky pushes the pallet, the tag rides along instead of staying at its original position.
- **Loading dock zone** at world `(-11, 0)` — yellow 3 × 4 m floor patch with black-and-yellow border stripes, on the open western edge.
- **Husky** parked at the dock entrance facing east; uses `husky_omnilink_bridge` (port 6070) and the `husky_eye` sidecar Robot (port 6071, supervisor-tracks the husky's pose each tick).
- **Truck bed** decorative wood platform south of the dock — visual dressing for the loading zone.

## The bridges

Two distinct bridge processes, all controllers attached to OmniSim Robot nodes via the `controller` field. Each owns its robot's actuators / sensors and exposes an HTTP API; agents never touch the OmniSim `controller` Python module directly.

### `husky_omnilink_bridge` (port 6070)

[`projects/samples/demos/controllers/husky_omnilink_bridge/`](../../../../projects/samples/demos/controllers/husky_omnilink_bridge/). Same bridge as the maze demos. Warehouse-relevant endpoints:

| endpoint | purpose |
|---|---|
| `GET /state` | Husky pose, mode, fault, sim_time |
| `GET /capabilities` | Wheel kinematics, max speeds, world title |
| `POST /action {action: "drive_to_waypoint", x, y, look_at?, ...}` | Continuous-space goto with optional teleport-snap-to-face |
| `POST /action {action: "stop"}` | Zero wheels |
| `POST /action {action: "complete_mission", rationale, ...}` | Bridge-side success claim, sets `mission_complete=true` |
| `GET /camera` | Proxies to `husky_eye:6071/image` — base64 PNG + tracking pose |
| `GET /scan` | Proxies to `husky_eye:6071/scan` — structured per-camera tag digest |
| `GET /mission` | Reads `WorldInfo.info` so the agent can read the operator's brief |
| `GET /solid?def=NAME` | **Ground truth**: returns supervisor-read world position of any DEF'd Solid. Used by `push_pallet_to` to verify pallets actually reached their target. |

The `look_at` parameter on `drive_to_waypoint` is a teleport-snap rather than an in-place pivot — a 180° physical pivot accumulates ~1 m of skid drift on this husky, which puts whatever the agent meant to look at outside the camera FoV. The teleport is acceptable here because `look_at` is an explicit, infrequent framing call.

### `husky_eye` sidecar (port 6071)

[`projects/samples/demos/controllers/husky_eye/husky_eye.py`](../../../../projects/samples/demos/controllers/husky_eye/husky_eye.py). Separate Robot node with its own controller — it's a small `Solid` that supervisor-tracks the husky's pose every tick and carries a `Camera` device (the URDFRobot can't host a Camera reliably). Exposes:

| endpoint | purpose |
|---|---|
| `GET /image` | Captures the front camera, returns 320×240 PNG as base64 |
| `GET /scan` | Captures + runs `_analyze_bgra` → returns structured digest |
| `GET /status` | Camera readiness, dimensions, last tracking pose |

The `_analyze_bgra` classifier is the **perception-as-tool** core: it counts pixels matching each of 6 colour signatures (red/green/blue + yellow/magenta/cyan) and reports the dominant tag if the count is above a 0.2% noise floor. The agent never sees pixels — it sees `{tag_color: "green", marker_fraction: 0.0038, color_fractions: {green: 0.0038}, ...}`. ~150× cheaper per query than sending the PNG to the LLM, and deterministic across runs.

## The agent runners

Each agent has a Python runner that:

1. Pushes the agent's profile (mainTask + tool descriptions) to OmniLink at startup
2. Starts a local HTTP server (`/tool`, `/status`, `/activity`) so OmniLink can dispatch tool calls (locally — the platform can't reach 127.0.0.1, so the dispatch is done by chat_drive or the Foreman's `delegate_to_agent`)
3. Polls `/status` for self-introspection (UsageMeter snapshot, complete-call counter, last action)

The runners share a common shape across all OmniLink agents:

```
{agent_name}_agent.py
├── load_all() — discover tool specs from tools/ submodules
├── start_tool_server() — http.server with /tool /status /activity
├── ensure_profile() — push or update the agent's profile on OmniLink
└── main loop — sleep + heartbeat (chat() is driven by external chat_drive)
```

Plus `scripts/chat_drive.py` per agent: runs the chat round-trip locally, dispatches tool calls against the runner's `/tool` endpoint, exits when the agent claims completion or hits max-turns.

## The cache architecture

Three layers:

1. **Server-side cache layer** (in the OmniLink platform) — wraps Gemini's explicit context-cache API. Hashes `(model, systemInstruction, tools)` into a stable key. On lookup miss, calls `ai.caches.create()` with the stable system prefix as the cache content. Subsequent calls reference the cache by name and pay 75% off on input tokens.

2. **Two-tier lookup** — process-local `Map` (warm-instance fast path, no round-trip) → a durable shared table (e.g. Supabase/Postgres), which survives serverless cold starts and parallel container instances → create new on miss. The shared tier is what makes caching useful on serverless, where the in-process map alone gave 1–4 % hit rate because each cold-start instance re-created the cache instead of finding the existing one.

3. **End-to-end measurement plumbing**:
   - Server: extracts `cachedContentTokenCount` from Gemini's response and meters it per Omni Key (cached vs. fresh input units), summed across windows for rollups.
   - Library: `omnilink-lib.usage_meter.UsageDelta` carries `cached_input_units`, `fresh_input_units`, derived `cache_hit_ratio`.
   - Runner: `/status.usage` exposes the new fields automatically through the existing `delta.to_dict()` pass-through.
   - HUD: [`WbAgentHud`](../../../../src/omnisim/gui/WbAgentHud.cpp) polls `/status` every 1.5s, renders cache hit % + cached / fresh tokens + dollars-per-hour at both gross and effective (cache-discounted) rates.

## `push_pallet_to` internals

The most non-trivial picker tool. Real physics with collision-driven pallet shoving + ground-truth verification. Lives in [`warehouse_picker/tools/picker.py`](../../warehouse_picker/tools/picker.py).

```
push_pallet_to(source_x, source_y, target_x, target_y)
├── resolve pallet DEF (LOAD_RED, LOAD_GREEN, ...) from coords
├── retry loop (≤ 3 iterations):
│   ├── /solid GET — read actual pallet position
│   ├── if within DELIVERY_TOL (2.0 m): break, declare delivered
│   ├── pick axis: prefer Y when |y| > 2 (still in row); otherwise larger remaining
│   ├── _push_segment(actual.x, actual.y, axis_target_x, axis_target_y):
│   │   ├── compute approach pose (1.6 m back, opposite to push direction)
│   │   ├── compute side-detour pose (2.5 m perpendicular at approach Y)
│   │   ├── if husky on wrong side of pallet, drive to side-detour first
│   │   ├── drive to approach pose with look_at past the pallet
│   │   ├── teleport-snap to face the push direction
│   │   └── drive forward through the pallet to push_end (target + 0.3 m overrun)
│   ├── /solid GET — read post-segment pallet position
│   └── if pallet didn't move ≥ 0.3 m: stalled, abort
├── final /solid verify: compute distance to target
└── return {status: ok / off_target, segments[], delivery: {actual_position, delivered}}
```

Why each piece:

- **Side-detour** — the husky finishes its camera scan facing the pallet from south. Without a detour, driving north to the approach pose runs the husky THROUGH the pallet, shoving it the wrong direction. The detour routes around.
- **Ground-truth verification** — the husky's `drive_to_waypoint` returns `done=True` when the husky reaches its waypoint, but the pallet may have lost contact mid-push or hit a wall. Re-reading the pallet's actual position via `/solid` is the only honest delivery check.
- **Retry loop** — long pushes (~12 m) sometimes lose contact partway. The loop reads the actual pallet position and shoves again from where it actually stopped.
- **Stall detect** — if the pallet didn't move at all between two segments, the husky is wedged against a wall stub or another pallet. Bail rather than spin forever.

## Operational lessons

Things this demo cost us to learn — keep these in mind when building the next agent:

1. **Trust ground truth, not agent reports.** Agents will claim success based on tool-return optimism. Always provide a supervisor-side verification path (`/solid`, `/state`, etc.) and have the tools call it before declaring success. Earlier in this build the Picker reported "GREEN delivered to dock" while the pallet was actually 16 m off in a wall corner.

2. **Perception-as-tool > pixels-to-LLM.** Sending images to the LLM is 100–150× more expensive than sending the eye sidecar's structured digest, AND it's stochastic (the LLM may misread on any given call) AND it tanks the cache hit rate (the system prefix gets dwarfed by image tokens). Default to a tool that returns structured tags; keep `read_camera` as a fallback for genuinely ambiguous frames.

3. **Cache plumbing needs durable storage on serverless.** Process-local cache maps will give you ≤4% hit rate. A durable shared table (Supabase / Redis / KV) for the `(cache_key → cache_name)` lookup is the difference between cache-as-decoration and cache-as-cost-saver. With our setup the difference between iter-2 and iter-3 cost was ~30%.

4. **Prompt over-specification wastes time and tokens.** Telling the picker to "scan all 6 pallets" when the Foreman already passed the target's coordinates costs 80% of the demo time and 87% of the input tokens. Trust the orchestrator's hints; verify with one snap; fall back to scan-all only on miss.

5. **Multi-agent fabric needs a strict honesty contract.** Without the Foreman's "never claim if any leg's `success=false`" rule, the orchestrator becomes a credulous narrator. The orchestration code's completion check (counter increment from baseline OR `mission_complete` transition) is what makes this verifiable.

## Cost trajectory (verified live)

From [`RESULTS.md`](RESULTS.md):

| iter | what changed | $/hr gross | $/hr w/cache |
|---|---|---:|---:|
| 0 | original (scan-all + image-attached + in-process cache) | $10.22 | $10.14 |
| 1 | hint-first scan | $2.86 | $2.78 |
| 2 | shared cache lookup (durable store) | $2.43 | $2.32 |
| **3** | **perception-as-tool (`scan_for_tag`)** | **$1.65** | **$1.50** |

85% cheaper than iter-0, same physical mission outcome (GREEN delivered to dock, ground-truth verified).

## File map

Quick reference for where each piece lives:

```
omnisim/
├── projects/samples/demos/
│   ├── worlds/warehouse_logistics.wbt
│   └── controllers/
│       ├── husky_omnilink_bridge/        # husky :6070
│       └── husky_eye/                    # eye sidecar :6071
├── agents/production/
│   ├── warehouse_foreman/
│   │   ├── warehouse_foreman_agent.py    # runner :51521
│   │   ├── profile.json                  # mainTask + 9 tools
│   │   ├── tools/                        # orchestration, foreman, recall, etc.
│   │   ├── scripts/chat_drive.py
│   │   └── docs/
│   │       ├── ARCHITECTURE.md           # this file
│   │       ├── RESULTS.md                # measured runs
│   │       └── _run_log_*.txt            # raw chat_drive output per iter
│   └── warehouse_picker/
│       ├── warehouse_picker_agent.py     # runner :51520
│       ├── profile.json                  # 15 tools
│       └── tools/picker.py               # drive, scan_for_tag, push_pallet_to, ...
└── src/omnisim/gui/
    ├── WbAgentHud.{hpp,cpp}              # live HUD pane
    └── WbMainWindow.cpp                  # tabified with text editor

(OmniLink platform code — LLM engines, the cache layer, usage metering,
 and DB migrations — lives in the separate OmniLink repository.)
```
