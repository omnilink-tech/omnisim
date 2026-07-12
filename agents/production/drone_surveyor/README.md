# Drone Surveyor — vision + autonomy on a different motion model

> **Status: scaffolding (iteration 0).** First commit ships the world (`chat/omnilink_mavic.wbt` with a Mavic 2 Pro + warehouse footprint + 8 coloured ground markers), the bridge controller (`mavic_omnilink_bridge`), the agent folder skeleton, and a `solve.py` that drives the bridge end-to-end without OmniLink. The productized agent (profile push, OmniLink runner, chat_drive) lands in iteration 2 — see [`docs/PLAN.md`](docs/PLAN.md) for the build order.

## The pitch

A DJI Mavic 2 Pro on `chat/omnilink_mavic.wbt` flies the perimeter of a warehouse footprint at 12 m altitude with its gimbal pointed straight down. Operator command:

> *"Fly the perimeter of the warehouse, count the RED ground markers, report their world positions."*

The drone takes off, points the gimbal down, flies waypoints around the building, and at each waypoint calls a `scan_for_markers` tool. The tool is the perception sidecar (here folded into the bridge — Mavic2Pro owns its own camera, so the eye-sidecar Robot needed by the husky's URDF doesn't exist on this robot). The tool returns structured `{color, world_x, world_y, fraction}` for every detected colour blob — projected from image-space centroid + drone pose + gimbal pitch + camera FoV. The agent aggregates RED detections across waypoints, deduplicates within ~1.5 m, and reports the count + positions.

## What this demos that nothing else does

A **completely different motion model** (quadcopter, four propellers, gimbal pitch axis, altitude management) running the **same OmniLink agent architecture** as the warehouse demos: bridge owns motion, perception is a tool returning structured tags (not pixels), the agent's job is reasoning over world-coordinate state. This is the demo that proves the OmniLink agent layer is **robot-agnostic** — the bridge changes, the agent prompt barely does.

It's also the most **visually striking** demo in the agent pack. Top-down camera view + autonomous flight + on-screen "I see 3 red markers at (-10, 6), (12, -4), (-3, -8)" narration is the highest-visual-reach pitch we have. Marketing video material, not just engineering proof-of-life.

## Reused infrastructure

- 100 % of the **perception-as-tool** pattern from `husky_maze` / `warehouse_foreman` (same BGRA classifier gates, same noise-floor threshold, same structured-digest contract)
- 100 % of the **mission-brief + complete_mission** contract (bridge surfaces brief in `/capabilities`, agent calls `complete_mission` with rationale + payload)
- 100 % of the **ground-truth verification** contract (`/solid?def=NAME` for each marker DEF — agent or test can verify a detection's projected world position against the actual marker pose)
- 100 % of the runner template that `husky_maze_agent.py` and `warehouse_foreman_agent.py` share (drops in once the tool surface is finalised)

## What's new

- **`mavic_omnilink_bridge`** — quadcopter flight stabiliser (PID adapted from the stock `mavic2pro.py` controller) + waypoint goto + altitude hold + gimbal pitch control. Single-process design (vs. husky's bridge + eye-sidecar split): the Mavic2Pro proto natively includes the Camera in its cameraSlot, so the bridge can call `getDevice("camera")` directly. The husky's URDFRobot can't host a Camera (OmniSim URDF importer drops it), which is why husky_eye exists as a separate Robot.
- **World-coordinate projection** — `_project_to_world(centroid_x_norm, centroid_y_norm, drone_x, drone_y, drone_alt, drone_yaw, gimbal_pitch_rad, fov_h, fov_v)`. Maps an image-space marker centroid to ground (x, y) coords. Critical for the "report their world positions" half of the mission — without this the agent would only know "I see something red somewhere in this frame."
- **Three RED markers as the count target**, plus five distractor colours (green, blue, yellow, magenta, cyan + the orange/white "ambiguous" markers that test the classifier's edge cases). Single-pass perimeter survey forces the agent to actually look at all sides of the building.

## Running iteration 0

The world + bridge run today; the productized agent doesn't yet (lands iter 2). For now you can validate the bridge end-to-end via the LLM-free `solve.py`:

```bash
# 1. Launch the world (in another terminal):
launch.bat projects\samples\demos\worlds\chat\omnilink_mavic.wbt

# 2. Run the deterministic perimeter survey:
python agents/production/drone_surveyor/solve.py
```

`solve.py` mirrors the role of `agents/production/husky_maze/solve.py`: it commits to a strategy at compile time (fly a fixed 4-corner perimeter, dedup detections within 1.5 m), proves the bridge wires up end-to-end, and prints the red-marker count + positions. The LLM-driven version (lands iter 2) replaces the hard-coded waypoints with planning over `capabilities.mission_brief`.

## Build order

Tracked in [`docs/PLAN.md`](docs/PLAN.md). Iterations:

| iter | scope | status |
|---|---|---|
| 0 | world + bridge + folder scaffold + LLM-free solve.py | shipped |
| 1 | RESULTS.md run from solve.py — verify the 3 red markers get counted, ground-truth-checked against `/solid?def=MARKER_RED_*` | **shipped — 3/3 reds detected, mean error 0.06 m, see [`docs/RESULTS.md`](docs/RESULTS.md)** |
| 2 | productized agent — profile.json, prompts/system.md, tools/{drone,recall,memory,knowledge}.py, drone_surveyor_agent.py runner, scripts/chat_drive.py | **shipped — 21 tools registered, runner + dispatcher verified against the live bridge in dry-run** |
| 3 | end-to-end LLM run — operator chat → agent plans waypoints → live perimeter survey → complete_mission with red count + positions; capture cost numbers into docs/RESULTS.md | next (requires OMNI_KEY) |
