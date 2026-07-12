# Warehouse Foreman — warehouse-logistics orchestrator

> **Status: end-to-end working, ground-truth verified.** One operator sentence drives two agents (Foreman → Picker), one robot (Husky), in a few minutes wall-clock. See [`docs/RESULTS.md`](docs/RESULTS.md) for the full verified run + token/cost breakdown.

## Mission

Operator says, in natural language:

> *"Move the green pallet to the loading dock."*

Two OmniLink agents handle it without further operator input:

- **Warehouse Foreman** *(this folder)* — orchestrator. Reads the world's mission brief, decomposes the goal into legs, delegates each to the right specialist, audits results, claims `complete_mission` when done. No direct robot control; pure routing.
- **Warehouse Picker** — Husky-driving specialist (next folder over: [`warehouse_picker/`](../warehouse_picker/)). Identifies coloured pallet tags via the husky's camera, drives the husky to the target pallet, then pushes it to the dock.

## What this demos that nothing else does

This is the **agent-fabric** story made concrete: one operator sentence → a Foreman agent decomposes → delegates to a Picker agent → which drives a real Husky to a coloured target → which physically pushes it into a dock zone. Two agents, one robot, one sentence. Mission Captain proved the pattern; Warehouse Foreman is the first user-facing demo where the orchestration is doing real work, not narrating progress on a single-agent task.

## World substrate

[`projects/samples/demos/worlds/flagship/warehouse_logistics.wbt`](../../../projects/samples/demos/worlds/flagship/warehouse_logistics.wbt) — a 30 × 18 m walled warehouse with:

- **Six pallet stacks** in a 2 × 3 grid: rows at y = +5 and y = -5; columns at x = -3, +3, +9.
- **Coloured tags** on top of each pallet (a 0.30 m emissive cube): red, green, blue, yellow, magenta, cyan. The husky's camera reads the colour from any aisle position.
- **Loading dock zone**: a yellow-painted 4 × 6 m floor patch at x ≈ -12, framed by black-and-yellow border stripes. The west wall has a gap so the dock is on an open edge.
- **Husky** parked at the dock entrance (-11, 0), facing east toward the pallets. Drop height 0.1 m. Controller currently `husky_random` until the Picker bridge is wired in.

The Viewpoint is the canonical top-down (`orientation -0.5773 0.5773 0.5773 2.0944, position 0 0 50`) — same as every other demo since [`aa7845b`](https://example/commit-link).

## Build trajectory

Tracked in [`DEMOS.md`](../../../DEMOS.md). All iterations land:

| iter | scope | status |
|---|---|---|
| 0 | World + agent-folder scaffolds + profile.json + README | ✅ |
| 1 | Picker bridge: continuous-space `drive_to_waypoint {x, y}` action | ✅ |
| 2a | Picker agent runner + first end-to-end "drive to green vantage" | ✅ |
| 2b | husky_eye sidecar + camera-driven tag identification | ✅ |
| 3 | Foreman runner + `delegate_to_agent` integration with Picker | ✅ |
| 4 | Pallets become rigid bodies + `push_pallet_to` tool — green pallet physically delivered to dock end-to-end with ground-truth `/solid` verification | ✅ |

Verified run, full token/cost breakdown, and ground-truth pre/post object positions: [`docs/RESULTS.md`](docs/RESULTS.md).

## Run the demo

Two runners, one chat_drive command:

```bash
# 1. Launch the world
launch.bat projects\samples\demos\worlds\flagship\warehouse_logistics.wbt

# 2. Start the Picker runner (drives the husky on bridge :6070)
set OMNI_KEY=olink_...
python agents\production\warehouse_picker\warehouse_picker_agent.py

# 3. Start the Foreman runner (orchestrator on :51521)
python agents\production\warehouse_foreman\warehouse_foreman_agent.py

# 4. Send the operator goal
set FOREMAN_DRIVE_TOOL_TIMEOUT=2400
python agents\production\warehouse_foreman\scripts\chat_drive.py ^
    --clear-memory --max-turns 10 ^
    "Move the green pallet to the loading dock."
```

The Foreman's `delegate_to_agent` runs each sub-loop locally (the OmniLink platform can't reach 127.0.0.1 from the internet), so the chain stays inside this machine. Verify ground truth at any time:

```bash
curl http://127.0.0.1:6070/solid?def=LOAD_GREEN
```
