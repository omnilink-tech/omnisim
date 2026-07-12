# Warehouse Picker — Husky-driving specialist for warehouse logistics

> **Status: scaffolded, not yet runnable.** First commit ships the agent profile + system-prompt skeleton. The bridge for continuous-space waypoint navigation comes next.

## Mission

Receives sub-tasks from the [Warehouse Foreman](../warehouse_foreman/) like:

> *"Drive to the green-tagged pallet, identify the colour from the camera, deliver to the dock."*

Drives the Clearpath Husky in [`warehouse_logistics.wbt`](../../../projects/samples/demos/worlds/flagship/warehouse_logistics.wbt). Six coloured pallets on a 2 × 3 grid; the agent identifies tags via the husky's front camera and routes to the matching pallet, then to the loading dock zone.

## Why this is its own agent (vs reusing Husky Maze)

The maze world uses a fixed 11 × 11 grid; the agent navigates cell-by-cell with `goto_cell`. The warehouse is **continuous-space with obstacles** — the husky needs to drive arbitrary (x, y) waypoints around pallet stacks, not navigate a perfect maze. So the Picker uses a different bridge action surface (`drive_to_waypoint`) and a different `mainTask` shape:

- No `goto_cell` / `try_get_known_map` / `execute_path` (those are maze-grid concepts).
- Adds `drive_to_waypoint {x, y}` + a `cargo_manifest` query surface for "what colour tag is on which pallet name".
- Camera-driven colour recognition is load-bearing — the agent has to actually read the tag colour from a frame, not look it up in a static dict (the Foreman could give the colour, but the Picker validates by sight before claiming completion).

The Husky Maze v1 vision protocol (`scan_surroundings` returning structured tags) is the closest cousin and should inspire the Picker's perception layer when the bridge lands.

## Build trajectory

Tracked in [`DEMOS.md`](../../../DEMOS.md) and the Foreman README. The Picker comes online in **iteration 2** (after the Picker bridge in iteration 1).

## Once runnable

(Stub for now — filled in when the runner lands.)

```bash
# Standalone Picker test (no Foreman):
launch.bat projects\samples\demos\worlds\flagship\warehouse_logistics.wbt
set OMNI_KEY=olink_...
python agents\production\warehouse_picker\warehouse_picker_agent.py
python agents\production\warehouse_picker\scripts\chat_drive.py \
    "Drive to the green pallet, then to the dock."
```
