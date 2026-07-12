# Warehouse Patrol — cross-session memory dividend in action

> **Status: shipped 2026-05-02.** End-to-end working: two huskies (north + south), one shared `Patrol Husky` profile, sector-tagged manifests persisted to local memory across runner restarts, `diff_sweeps` correctly narrates moved/missing/new crates against the prior sweep. Live results in [`docs/RESULTS.md`](docs/RESULTS.md).

## The pitch

Two Clearpath Huskies on `warehouse_patrol.wbt`, same `Patrol Husky` profile, running in parallel from separate runners:

> *"Patrol your sector every 30 minutes; report anything that moved since the last sweep."*

Each husky covers a sector (north or south). Each sweep, the agent uses `scan_for_tag` and `get_state` to build a manifest of what it saw + where, then `save_local_memory` writes that manifest to long-term memory. On the next sweep, the agent calls `recall` to pull the prior manifest, compares against what it now sees, and narrates the diff: *"crate_07 moved 1.8 m east since last sweep; crate_03 is now missing."*

## What this demos that nothing else does

Cross-session memory **doing something useful**. Husky Maze proved memory survives sessions, but the value compounds slowly there ("here's a path I plotted before"). Patrol Squad makes the dividend concrete: on every sweep the agent's `recall` returns something the operator wants to see RIGHT NOW (a diff, an alert), and the value scales with the number of sweeps.

It's also the first **horizontal-scale** demo: two agents share the SAME profile but cover different sectors, demonstrating that the OmniLink fabric scales out by spinning up more runners against more bridges, not by complicating any single agent.

## Build order

Tracked in [`DEMOS.md`](../../../DEMOS.md). Iterations:

| iter | scope | status |
|---|---|---|
| 0 | world (warehouse_patrol.wbt with 2 huskies + scattered decoy crates) + agent folder scaffold + profile | ✅ shipped |
| 1 | bridge port multiplexing — both bridges (husky_omnilink_bridge, husky_eye) accept `--port` controllerArgs so two instances coexist on different ports | ✅ shipped (incl. WbUrdfImporter.cpp patch to preserve controllerArgs on URDF import) |
| 2 | agent: `Patrol Husky` profile with sector assignment, `sweep_summary` + `recall_last_sweep` + `diff_sweeps` tools, sector-tagged local memory | ✅ shipped |
| 3 | end-to-end run: sweep, move crates, re-sweep, verify diff narration matches teleports — both north + south sectors verified | ✅ shipped |

## Reused infrastructure

- 100 % of `husky_omnilink_bridge` (with one new env var for port multiplexing)
- 100 % of `husky_eye` (same env-var change)
- 100 % of the `husky_maze` agent stack (memory tools, recall, knowledge); inherited via folder copy
- `warehouse_logistics.wbt` style world (walls + dock zone) minus the arm + dock — patrol doesn't need them

## What's new

- Sector assignment in the agent's `mainTask` (north sector y > 0; south y < 0)
- `sweep_summary` tool: walks the sector, records visible tag colours + their world positions, returns a structured manifest
- Recall+diff prompt: agent's first action on each sweep is to recall the prior sweep's manifest and stage a diff against it
