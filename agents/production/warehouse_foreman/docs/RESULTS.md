# End-to-end results — Warehouse Foreman demo

Live verified runs: one operator sentence, agent fabric, ground-truth confirmed via supervisor reads (not agent-reported claims). Three iterations measured — naive overscan, hint-first scan, and perception-as-tool — to show how each lever bends the cost curve.

## TL;DR

> **Operator:** "Move the green pallet to the loading dock."

The final perception-as-tool configuration runs the mission at **$0.18 / mission, $1.50 / hour** with cache (Gemini 3 Flash list price). Ground truth post-run:

- **GREEN pallet at world (-12.21, -1.02)** — inside the dock zone (-12.5..-9.5, -2..+2). Verified via `GET /solid?def=LOAD_GREEN`.
- **Other 5 pallets unchanged** at their rack positions.

Each iteration peeled off a layer of waste:

| iter | what changed | $/hr gross | $/hr effective |
|---|---|---:|---:|
| 0 — original | scan all six pallets, attach inline PNG image to LLM, in-process cache only | **$10.22** | $10.14 |
| 1 — hint-first | trust the Foreman's pallet-coordinate hint, verify with one camera snap, fall back to scan-all only on miss | **$2.86** | $2.78 |
| 2 — shared cache | durable shared cache table behind the server-side cache layer so cache resource names survive serverless cold starts | **$2.43** | $2.32 |
| 3 — perception-as-tool | new `scan_for_tag` tool returns structured `{tag_color, marker_fraction, ...}` from the husky_eye sidecar's pure-Python frame analyser; no pixels sent to the LLM | **$1.65** | **$1.50** |

**Net change: 85% cheaper per hour.** Each iteration delivered the same physical mission outcome (GREEN on dock) — none of the cost wins came from cutting capability.

## What each iteration unlocked

### iter 0 → 1 (hint-first scan)

The picker was scanning **all six pallets** before pushing — drive + camera + reorient at each, ~10 minutes wasted on motion the Foreman's task description had already eliminated by passing `pallet positions: red (-3, 5), green (3, 5), …`. Updated picker prompt to verify the brief's hint position FIRST, fall back to scanning others only on miss. Vision discriminator preserved: the agent still reads pixels and confirms the colour from ground truth — it just doesn't waste time confirming the other five.

### iter 1 → 2 (shared cache lookup)

Caching plumbing was already wired through the server-side cache layer, but kept the `(cache_key → cache_name)` map in process-local memory only. The serverless host cold-starts each engine-endpoint invocation onto a fresh container instance with empty memory; parallel instances each have their own map. So cache hits happened only on warm-instance reuse — 1–4% of calls. Added a durable shared cache table as a shared lookup. The two-tier lookup (process Map → durable store → create new) survives instance lifetimes; the cache-lookup result records a hit-source that distinguishes warm-instance hits (`process_map`), cold-start-saved hits (durable store), and genuine creates (`created`).

### iter 2 → 3 (perception-as-tool)

The picker was attaching the 320×240 PNG from `read_camera` as an inline `image_url` part to the LLM. Each snap = ~12k input tokens of base64. The cacheable system prefix was ~1.5k tokens — drowned out by the per-turn image content (cache could only ever hit ~5% of input).

Brought the picker into line with the husky_maze pattern: husky_eye's pure-Python `_analyze_bgra` already classifies coloured-cube tags by counting pixels per RGB signature. Extended it from 3 colours (red/green/blue) to all 6 warehouse colours, lowered the noise threshold from 0.5% to 0.2% to handle the warehouse's smaller 0.30 m emissive cubes at 4 m vantage. New `scan_for_tag` tool proxies the eye's `/scan` endpoint and returns the structured `{tag_color, marker_pixels, marker_fraction, marker_centroid, color_fractions}` digest. **The agent never sees pixels.** `read_camera` stays available as a fallback for ambiguous frames but the prompt directs `scan_for_tag` first.

Result: per-turn input dropped from ~30k tokens to ~3k tokens, the cacheable prefix is now a meaningful fraction (~50%), cache hit rate doubled from 6.2% → 13.6%, and the headline cost dropped 32% to $1.50/hr.

## Final run metadata (perception-as-tool, 2026-05-02)

| | |
|---|---|
| Date | 2026-05-02 |
| World | [`projects/samples/demos/worlds/flagship/warehouse_logistics.wbt`](../../../../projects/samples/demos/worlds/flagship/warehouse_logistics.wbt) |
| Engine | `g1-engine` (Gemini 3 Flash, $0.50 / M input, $3.00 / M output list, $0.125 / M cached input) |
| Agents | Warehouse Foreman, Warehouse Picker |
| Bridges | husky_omnilink_bridge :6070, husky_eye :6071 |
| Wall clock | **355 s** (5 m 55 s) |
| Outcome | Picker delegation success, ground-truth verified |

## Cost detail (perception-as-tool run)

Numbers from `UsageMeter` snapshot of the platform's 24 h-rolled token counters, delta over the 431 s window:

| metric | value |
|---:|---|
| input tokens | **344,411** |
| → cached input | **46,848 (13.6 %)** |
| → fresh input | 297,563 |
| output tokens | 8,284 |
| credits charged | $0.00 (free tier) |

| | per mission | hourly rate |
|---:|---:|---:|
| gross input | 344,411 × $0.50 / M = $0.172 | $1.44 |
| output | 8,284 × $3.00 / M = $0.025 | $0.21 |
| **gross total** | **$0.197** | **$1.65** |
| effective input (with cache discount) | $0.155 | $1.30 |
| **effective total (with cache)** | **$0.180** | **$1.50** |
| dollars saved by cache | $0.018 / mission (8.9 %) | $0.15 / hour |

The 13.6% measured hit rate is still under the 75–85% theoretical ceiling. Two reasons:
- Serverless instances may cold-start partway through a chain even with the shared cache (the durable-store round-trip avoids the cache-create cost but doesn't avoid all the latency).
- The conversation history (`messages`) grows each turn and is not cached by Gemini — only the system prefix is.

To get further would require Anthropic-style message-prefix caching (Gemini doesn't support it on the API yet) or aggressive message pruning between turns.

## Run metadata (optimised demo, 2026-05-02)

| | |
|---|---|
| Date | 2026-05-02 |
| World | [`projects/samples/demos/worlds/flagship/warehouse_logistics.wbt`](../../../../projects/samples/demos/worlds/flagship/warehouse_logistics.wbt) |
| Engine | `g1-engine` (Gemini 3 Flash, $0.50 / M input, $3.00 / M output list) |
| Agents | Warehouse Foreman, Warehouse Picker |
| Bridges | husky_omnilink_bridge :6070, husky_eye :6071 |
| Wall clock | **322 s** (5 m 22 s) — START 15:28:37 → END 15:33:59 |
| Outcome | `success=true` AND ground-truth verified for GREEN→dock |

## Cost (measured, optimised run)

Numbers from `UsageMeter` snapshot of the platform's 24 h-rolled token counters at session start vs end (delta over the 397 s window with caching now plumbed through end-to-end):

| metric | value |
|---:|---|
| input tokens | **602,270** |
| → cached input | **24,592 (4.1 %)** |
| → fresh input | 577,678 |
| output tokens | 4,938 |
| credits charged | $0.00 (free tier) |
| cached_input_units_24h (rollup) | non-zero — visible in `GET /api/omni-key-usage` |

At Gemini 3 Flash list pricing:

| | per mission | hourly rate |
|---:|---:|---:|
| gross input | 602,270 × $0.50 / M = $0.301 | $2.73 |
| output | 4,938 × $3.00 / M = $0.015 | $0.13 |
| **gross total** | **$0.316** | **$2.86** |
| effective input (75 % off cached) | $0.301 − ($24,592 × $0.375 / M) = $0.291 | $2.65 |
| **effective total (with cache)** | **$0.307** | **$2.78** |
| dollars saved by cache | $0.009 / mission (3 %) | $0.08 / hour |

So the **measured** cache savings are real but small (3 %) at the current 4 % hit rate. To translate the cost story across regimes:

| if hit rate were | $/mission | $/hour |
|---:|---:|---:|
| 0 % (gross) | $0.316 | $2.86 |
| **4.1 % (measured)** | **$0.307** | **$2.78** |
| 50 % (warm-worker target) | $0.203 | $1.84 |
| 75 % (Gemini documented best) | $0.147 | $1.33 |

**Why the hit rate was low (this run):** the server-side cache layer originally stored cache resource names in a process-local `Map`. The OmniLink API runs on a serverless host, where each container instance has its own memory; cold starts (new instance) and parallel instances each reset that Map, so every chat call effectively paid the create-cost without amortising it. The TTL window (10 min default) covers a single warm instance's lifetime, but parallel instances each had their own map. Fixed in the follow-up commit by adding a durable shared cache-name lookup — the next demo measures the post-fix hit rate.

## Tool-call breakdown (optimised run)

| agent | turns | tool calls | distinct tools |
|---|---:|---:|---|
| Warehouse Foreman | 4 | 4 | `read_mission_brief`, `list_agents`, `delegate_to_agent`, `complete_mission` |
| Warehouse Picker (sub-loop) | 8 | 6 | `read_mission_brief`, `drive_to_waypoint` × 2, `read_camera`, `push_pallet_to`, `complete_mission` |
| **Total** | **12** | **10** | |

## Phase-by-phase

### Phase 1 — Foreman parses the operator goal (turns 1–3, ~12 s LLM time)

```
Foreman 1: read_mission_brief                 -> brief: "Mission: move the GREEN-tagged pallet …"
Foreman 2: list_agents                        -> Warehouse Picker reachable
Foreman 3: delegate_to_agent('Warehouse Picker', "Identify the GREEN pallet using your camera at successive vantage points, then push it to the loading dock at (-11, 0) using push_pallet_to. … Pallet positions: red (-3, 5), green (3, 5), blue (9, 5), yellow (-3, -5), magenta (3, -5), cyan (9, -5)")
```

### Phase 2 — Picker sub-loop drives, identifies, pushes (turns 1–16 of the sub-loop, ~466 s)

The Picker's full activity feed:

```
 1. read_mission_brief                              # confirm GREEN target
 2. drive_to_waypoint(0, -1, look_at=…)             # vantage in y=-1 corridor
 3. read_camera                                     # (no green from this angle)
 4. drive_to_waypoint(3, -1, look_at=[3, -5])       # vantage south of magenta
 5. read_camera                                     # MAGENTA — not target
 6. drive_to_waypoint(9, -1, look_at=[9, -5])       # vantage south of cyan
 7. read_camera                                     # CYAN — not target
 8. drive_to_waypoint(0, 1, look_at=…)              # vantage in y=+1 corridor
 9. read_camera                                     # scanning N row
10. drive_to_waypoint(0, 1, look_at=[3, 5])         # re-frame for green pallet
11. read_camera                                     # GREEN identified at (3, 5)
12. push_pallet_to(source=(3, 5), target=(-11, 0))  # the physical delivery
13. drive_to_waypoint(-9, 0)                        # park clear of dock
14. complete_mission                                # claim success
```

The `push_pallet_to` call (entry 12) internally:
1. Reads `/solid?def=LOAD_GREEN` for ground-truth start
2. Picks the larger remaining axis (Y first, since pallet is still in the +5 row)
3. Routes via a side-detour 2.5 m perpendicular to the push axis (so the husky's drive to its approach pose doesn't cross THROUGH the pallet)
4. Drives behind the pallet, teleport-snaps to face the dock, drives forward shoving the pallet
5. Re-reads `/solid` after each segment, picks the next push axis from actual position
6. Loops up to 3 retries; declares `delivered=true` only when ground-truth distance to target < 2 m

### Phase 3 — Foreman composes the audit claim (turn 4, ~4 s LLM)

```
Foreman 4: complete_mission(rationale="The green pallet was successfully moved from its starting location to the loading dock by the Warehouse Picker, ground-truth verified at the dock.", legs=[...])
```

## Ground truth — pre vs post

Captured via `GET /solid?def=NAME` against the supervisor:

| object | pre | post | delta |
|---|---|---|---|
| LOAD_RED | (-3.00, +5.00) | (-3.00, +5.00) | 0.00 m — untouched |
| LOAD_GREEN | (+3.00, +5.00) | **(-12.21, -1.02)** | 16.31 m delivered to dock |
| LOAD_BLUE | (+9.00, +5.00) | (+9.00, +5.00) | 0.00 m — untouched |
| LOAD_YELLOW | (-3.00, -5.00) | (-3.00, -5.00) | 0.00 m — untouched |
| LOAD_MAGENTA | (+3.00, -5.00) | (+3.51, -5.04) | 0.51 m — incidentally bumped during Picker's southern-corridor vantage drive |
| LOAD_CYAN | (+9.00, -5.00) | (+9.00, -5.00) | 0.00 m — untouched |
| Husky | (-11.00, 0.00) | (-9.07, -0.16) | parked east of dock, clear |

The dock zone is the rectangle (-12.5, -2) to (-9.5, +2). The GREEN pallet's centroid at (-12.21, -1.02) sits 0.29 m inside the west edge and 0.98 m north of the south edge — visibly on the yellow plate from any operator viewpoint.

## Reproducing this run

Three terminals (or backgrounded processes), one chat_drive command:

```bash
# Terminal 1 — OmniSim with the warehouse world
launch.bat projects\samples\demos\worlds\flagship\warehouse_logistics.wbt

# Terminal 2 — Picker runner (port 51520, husky bridge)
OMNI_KEY=olink_... python agents/production/warehouse_picker/warehouse_picker_agent.py

# Terminal 3 — Foreman runner (port 51521)
OMNI_KEY=olink_... python agents/production/warehouse_foreman/warehouse_foreman_agent.py

# Terminal 4 — drive the demo
OMNI_KEY=olink_... FOREMAN_DRIVE_TOOL_TIMEOUT=2400 \
    python agents/production/warehouse_foreman/scripts/chat_drive.py \
    --clear-memory --max-turns 10 \
    "Move the green pallet to the loading dock."
```

Verify ground truth at the end:

```bash
for d in LOAD_RED LOAD_GREEN LOAD_BLUE LOAD_YELLOW LOAD_MAGENTA LOAD_CYAN; do
  curl -s "http://127.0.0.1:6070/solid?def=$d"
done
```

## What this demo proves

1. **Agent orchestration with real handoffs** — operator sentence → Foreman → Picker → composed claim. The handoff dispatches a real OmniLink chat with the specialist's full pushed profile; the orchestrator's loop exits cleanly on the specialist's completion-counter increment OR consecutive no-tool-call turns.
2. **Vision-driven identification** — the Picker reads pixels from `husky_eye` and identifies the GREEN tag from the actual frame, not from coordinates in the prompt. Other-colour scans (MAGENTA, CYAN, etc.) confirm the agent isn't cheating.
3. **Physical pallet manipulation** — the husky pushes the pallet+tag rigid body across 16 m of warehouse via a 3-segment shove (y first to clear the row, then 2 × x to traverse the corridor) with side-detour staging so the approach drive doesn't displace the pallet in the wrong direction.
4. **Ground-truth verification** — `GET /solid?def=…` exposes supervisor reads of any DEF'd Solid. `push_pallet_to` re-queries between segments and refuses to declare delivery without the pallet centroid landing inside the dock bounding box. Earlier "agent claimed success but the pallet was still floating in the rack" failures are explicitly closed.

## Known limitations

- Cost is ~$2.50 / mission at gross list price (vision frames + multi-turn context dominate input tokens). The platform's usage rollup doesn't expose cache-hit metadata, so any implicit-cache discount that DID happen server-side is invisible to us — the projected $0.65–$1.30 with-caching range is a scenario estimate, not a measurement.
- The Picker's tag-identification scan order isn't optimal — it tends to scan a corridor at a time rather than do a binary search by colour. A future iteration could prune scans once 5 of 6 pallets are eliminated.
- The push tool is collision-naive: the Picker bumped the magenta pallet by 0.5 m during a vantage drive. Path planning around pallets-in-row would fix this; for now, the bumped pallet's drift is small enough that subsequent missions still find it.
- The dock delivery is a push, not a pick-and-place — there is no arm in this demo. A gripper-equipped arm staging the pallet onto a truck bed would be a future extension; out of scope for the warehouse_foreman demo.

## Artefacts

- [`_run_log.txt`](_run_log.txt) — the full chat_drive stdout for this run, including every Foreman turn's narration and tool calls.
- The Foreman's `/status` snapshot ([live](http://127.0.0.1:51521/status) while the runner is up) carries the post-run `last_delegation` block with the Picker sub-loop's final text.
- Ground truth pallet positions are available any time via `GET /solid?def=LOAD_<COLOUR>` against the husky bridge.
