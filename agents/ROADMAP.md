# Demo roadmap — what to build next

> **Status (2026-07): this build queue is complete — kept as a historical record of the 2026-05 wave.**
> Warehouse Foreman, Patrol Squad, and Drone Surveyor all shipped in that wave (Drone Surveyor
> iter 0–2; its iter-3 operator-chat LLM run never happened). **All three were RETIRED in
> v5.3.0** — sections 1–3 below are a historical record, not a current inventory.
>
> ⚠️ **Warehouse Foreman, Warehouse Picker and Patrol Squad were REMOVED from the tree on
> 2026-07-19, and ship as RETIRED in v5.3.0**, along with their worlds
> (`warehouse_logistics.wbt`, `warehouse_patrol.wbt`).
> Their shipped results rested on locomotion numbers that predated the wedge-recovery
> teleport removal (`b18bd7a3`) and were never re-measured. Sections 1 and 2 below are kept as a
> historical record, and the `production/warehouse_*` paths they name are **no longer in the
> tree** — recover them from git history if you need them. The transferable design lessons
> survive in
> [`AGENT_PATTERNS.md`](AGENT_PATTERNS.md).
>
> ⚠️ **Drone Surveyor was REMOVED from the tree on 2026-07-22, and ships as RETIRED in
> v5.3.0.** `agents/production/drone_surveyor/`
> is gone; section 3 below is kept as a historical record, and the paths it names are **no longer
> in the tree** — recover them from git history if you need them. Its world,
> [`chat/omnilink_mavic.omniworld`](../projects/samples/demos/worlds/chat/omnilink_mavic.omniworld), and its
> bridge, `mavic_omnilink_bridge`, were **kept** — they still serve the Mavic chat demo.
>
> For the current agent inventory, see [`README.md`](README.md).

Working list of OmniLink agents + demos worth shipping after the husky_maze + mission_captain pair. Ordered roughly by cool-to-effort ratio. Pick the next one off the top, build it end-to-end, ship it, then come back to this list.

The bar for inclusion: each demo must do at least one thing **only OmniLink can do well** — multi-agent orchestration, cross-session memory dividend, natural-language brief interpretation, or visual-reasoning-driven navigation. Pure motion demos belong in stock OmniSim samples; this is the agent showcase.

## Selection criteria

Every entry below is graded on the same axes:

- **OmniLink-specific value** — what does it show that raw OmniSim can't?
- **Visual reach** — does a non-engineer get it within 5 seconds?
- **Effort to working video** — small (≤2 days), medium (2-5 days), high (1-2 weeks).
- **Reused infrastructure** — what already exists vs what we'd write fresh.

## 1. Warehouse Foreman — multi-agent orchestration *(RETIRED in v5.3.0 — historical)*

> ⚠️ Retired in v5.3.0 — removed from the tree on 2026-07-19, together with the Warehouse
> Picker specialist described below. `production/warehouse_foreman/` no longer exists, so its docs
> are named as plain paths rather than links; recover them from git history if you need them.

**Status at retirement:** end-to-end working, ground-truth verified, $1.50/hr at Gemini 3 Flash list. The architecture write-up (`production/warehouse_foreman/docs/ARCHITECTURE.md`) and the measured runs (`production/warehouse_foreman/docs/RESULTS.md`) were removed with the agent — its results rested on locomotion numbers that predated the wedge-recovery teleport removal (`b18bd7a3`) and were never re-measured. The transferable patterns survive in [`AGENT_PATTERNS.md`](AGENT_PATTERNS.md).

**Pitch.** Two agents, one robot, one operator command:
> *"Foreman, move the green pallet to the loading dock."*

- **Foreman** — orchestrator. Decomposes the operator's goal into legs, picks specialists, audits results. Mostly the existing Mission Captain template with a warehouse-aware `mainTask`.
- **Picker** — Husky drives to a labelled pallet, snaps a camera frame, identifies it by colour, then physically pushes it to the loading dock. ~90% of Husky Maze v1 (vision branch) with the maze swapped for `warehouse_husky.omniworld` and the brief swapped for "find SKU X / colour Y".

**OmniLink-specific value.** This IS the agent-fabric story made concrete: one agent calling another live, with a real robot moving in response to text from the orchestrator. Mission Captain already proved the pattern; Warehouse Foreman is the first user-facing demo where the orchestration is doing real work, not narrating progress.

**Visual reach.** High. The video reads in 5 seconds: husky drives to a pallet, identifies it by colour, pushes it to the dock. Operator never moves a finger after the initial sentence.

**Effort.** Medium (2-3 days):
- New world: extend `warehouse_husky.omniworld` with labelled pallet markers and a loading-dock zone (~50 lines of WBT).
- New agent folder: `warehouse_foreman/` (mostly copies of Mission Captain).
- Picker is Husky Maze v1 with a different system prompt + a `find_sku` tool that combines `read_camera` + visited_cells search.
- Foreman's `delegate_to_agent` already covers cross-agent calls.

**Reused infrastructure.**
- `husky_omnilink_bridge.py` (drive primitives, camera proxy, mission-brief)
- `husky_eye` sidecar (camera frames)
- Mission Captain template (`agents/production/mission_captain/`)
- `warehouse_husky.omniworld` world

**What's new.**
- Pallet markers + dock zone in the warehouse world
- A "cargo manifest" surface on the bridge (which pallet has which colour box)
- Foreman's `mainTask` for warehouse logistics
- Picker's vision-driven SKU recognition (extends the perception-as-tool architecture from blind world)

## 2. Patrol Squad — cross-session memory dividend in action *(RETIRED in v5.3.0 — historical)*

> ⚠️ Retired in v5.3.0 — removed from the tree on 2026-07-19 along with its world
> (`warehouse_patrol.wbt`). `production/warehouse_patrol/` no longer exists; recover its results
> trace from git history if you need it.

**Status at retirement:** end-to-end working, dividend verified. Two huskies (north + south), one shared `Patrol Husky` profile, sector-tagged manifests persisted across runner restarts. Diff narration matched the operator's actual teleports. Live results were recorded in `production/warehouse_patrol/docs/RESULTS.md`, removed with the agent.

**Pitch.** Two Huskies in the warehouse, same Husky Maze v3 profile, running in parallel. Mission:
> *"Patrol all five aisles every 30 minutes; report anything that moved since the last sweep."*

Each Husky covers a sector. Each sweep, the agent saves what it saw to long-term memory. On the next sweep it compares against the previous snapshot and flags any new / moved / missing crate.

**OmniLink-specific value.** This is the most direct demo of cross-session memory doing something genuinely useful — change detection over time, with the agent's own `recall` returning the prior state on each round. Also shows two agents running the SAME profile in parallel without stepping on each other (good "fabric scales horizontally" story). And it's the first long-running demo: the value compounds over multiple sweeps, not within one chat.

**Visual reach.** Medium. The split-screen showing two Huskies patrolling in parallel reads instantly; the change-detection alert ("crate_07 moved 2 m east since last sweep") is the satisfying payoff but it requires watching for ~2 sweeps to land.

**Effort.** Low-medium (1-2 days):
- Bridge multiplexing: spawn a second `husky_omnilink_bridge` instance on port 6080 (and a second `husky_eye` on 6081). Most of the work is making the bridge accept a port-suffix env var so two instances co-exist.
- Two profiles pushed to OmniLink: `Patrol Husky North` and `Patrol Husky South` with sector assignments in their `mainTask`.
- A small `sweep_summary` tool that takes the current visited_cells + camera snapshots and writes a memory entry.
- Recall + diff logic: the agent's prompt instructs it to recall the prior sweep's manifest and narrate diffs.

**Reused infrastructure.**
- 100% of Husky Maze v3 (no agent changes)
- `warehouse_husky.omniworld` (already crowded with crates that can be subtly moved between sweeps)
- Long-term memory stack (already shipped, hybrid retrieval)

**What's new.**
- Bridge port multiplexing (one-line env-var change)
- Sector-assigned mainTasks
- A scripted "moved a crate" sweep-trigger so the demo has interesting diffs

## 3. Drone Surveyor — vision + autonomy on a different motion model *(RETIRED in v5.3.0 — historical)*

> ⚠️ Retired in v5.3.0 — removed from the tree on 2026-07-22. `production/drone_surveyor/` no
> longer exists, so its docs are named as plain paths rather than links; recover them from git
> history if you need them. The world and bridge below were kept for the Mavic chat demo.

**Status at retirement:** world (`chat/omnilink_mavic.omniworld`) + bridge (`mavic_omnilink_bridge`) + LLM-free `solve.py` shipped iter 0; iter-1 verified end-to-end (3/3 red markers detected at 0.06 m mean projection error, recorded in `production/drone_surveyor/docs/RESULTS.md`); iter-2 productized agent shipped — full profile + system prompt + 21-tool surface + runner + chat_drive script, smoke-tested against the live bridge. Iter 3 (operator chat → live LLM-driven survey → cost numbers) never ran. The full plan (`production/drone_surveyor/docs/PLAN.md`) was removed with the agent.

**Pitch.** Mavic 2 Pro on `chat/omnilink_mavic.omniworld`. Operator:
> *"Fly the perimeter of the warehouse, count the red markers on the ground, report their world positions."*

Drone autonomously follows a polygon, points its gimbal camera down at fixed waypoints, the agent's vision narrates what it sees, `complete_mission` reports the count + cell positions.

**OmniLink-specific value.** A non-husky robot, totally different motion model, but the SAME perception-as-tool architecture from the blind world (just adapted to gimbal + altitude). Demonstrates that the OmniLink agent layer is robot-agnostic — the agent prompt doesn't change much, the bridge does.

**Visual reach.** Highest of the four. Drones are crowd-pleasers in demo videos. Top-down camera view + autonomous flight + on-screen "I see 3 red crates" narration is striking.

**Effort.** Medium-high (3-5 days):
- New bridge: `mavic_omnilink_bridge.py` with takeoff/land/waypoint primitives. Mavic uses 4 propellers + gimbal, very different from skid-steer.
- New eye sidecar (or reuse the existing pattern with a different mounted camera).
- Waypoint motion: drive_to_position + altitude_hold rather than goto_cell. Could simplify by giving the agent fixed waypoints (drone flies a programmed path; agent only reasons about vision).
- A new agent folder: `drone_surveyor/`.

**Reused infrastructure.**
- Mavic world is already there
- Perception-as-tool architecture (scan_surroundings analogue with gimbal frames)
- Mission-brief + complete_mission contract
- chat_drive script (zero changes — just a different agent name)

**What's new.**
- Mavic motion controllers + bridge
- Altitude management + gimbal pointing
- A "drop a pin on the world map" report format

## What we're NOT building (and why)

- **Multi-robot competitive games.** Flashy but not the OmniLink story — the value is in the perception + planning, not the agents-talking-to-each-other piece. Save for later.
- **Cooking / kitchen manipulation.** Manipulation-heavy demos are a different problem (grasp planning, contact dynamics) and OmniSim's gripper sim is shaky. Better as a sim-only future direction.
- **Rough-terrain (`desert_ruins.omniworld`, `moon.wbt`).** Outdoor navigation looks cool but adds little over the maze story we already tell. Maybe later as a "multi-environment Husky" pack.
- **Anything requiring BYOK GPT/Claude keys to function.** Free-tier accessibility matters for demos; demos that only work with a paid Anthropic / OpenAI key are not first-touch material.

## Build order

1. **Warehouse Foreman** ✅ shipped 2026-05-02 — ⚠️ **retired in v5.3.0** (removed from the tree 2026-07-19)
2. **Patrol Squad** ✅ shipped 2026-05-02 — world + bridge multiplexing + sector-tagged shared profile + sweep/recall/diff pipeline + cross-session dividend live verified — ⚠️ **retired in v5.3.0** (removed from the tree 2026-07-19)
3. **Drone Surveyor** iter 0-2 shipped 2026-05-02 — world + bridge + verified perception (3/3 red markers, 0.06 m mean error) + productized agent (21-tool surface, runner on :51521, chat_drive); iter-3 LLM run never happened — ⚠️ **retired in v5.3.0** (agent removed from the tree 2026-07-22)

Patrol Squad and Drone Surveyor are being built in parallel because they don't share the same world or bridge — Patrol works on `warehouse_patrol.wbt` with the husky stack, Drone Surveyor works on `chat/omnilink_mavic.omniworld` with a brand-new mavic stack.

Each should ship its own README + a short docs/RESULTS-style verification trace alongside its own folder under `agents/production/`, mirroring the husky_maze layout. Read [`AGENT_PATTERNS.md`](AGENT_PATTERNS.md) before starting #2 — it's the cross-demo cheat sheet for "patterns that paid off, defaults to use, mistakes to skip."
