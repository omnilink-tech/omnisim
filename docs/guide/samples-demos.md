## Demos

This section provides a list of interesting worlds that broadly illustrate OmniSim capabilities.
You will find the corresponding ".wbt" files in the "[OMNISIM\_HOME/projects/samples/demos/worlds]({{ url.github_tree }}/projects/samples/demos/worlds)" directory, and their controller source code in the "[OMNISIM\_HOME/projects/samples/demos/controllers]({{ url.github_tree }}/projects/samples/demos/controllers)" directory.

The OmniLink-driven demos are the canonical starting points; see [OmniLink chat demos](omnilink-chat-demos.md) for the agent-facing walkthrough. The list below picks out a few additional worlds that highlight specific OmniSim capabilities.

### [warehouse\_husky.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/showcase/warehouse_husky.omniworld)

**Keywords**: URDF importer, Husky, supervisor, reactive control

The default onboarding demo. A supervisor-enabled Husky (URDFRobot, `husky_random` controller) random-walks a 30 × 18 m warehouse with reactive collision recovery. Good showcase of the URDF importer, supervisor APIs, motor torque/sensor pipeline, and the camera follow (F key).

### [husky\_maze.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/flagship/husky_maze.omniworld)

**Keywords**: URDF importer, OmniLink bridge, navigation

A single Husky placed at the NW corner of an 11 × 11-cell walled maze. The `husky_omnilink_bridge` controller exposes an HTTP control surface so an external OmniLink agent can drive the robot. See sibling worlds (`husky_maze_blind.omniworld`, `husky_maze_corners.omniworld`, `husky_maze_unknown.omniworld`, `husky_maze_visual.omniworld`) for progressively harder briefs.

### [omnilink\_mavic.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/chat/omnilink_mavic.omniworld)

**Keywords**: Quadrotor, waypoint flight, perimeter survey, marker detection

A DJI Mavic 2 Pro quadrotor demo with two surfaces on one world: right-click chat verbs ("takeoff", "forward 1 m", "land") for a human operator, and a survey tool set (`scan_for_markers`, `goto_waypoint`, `complete_mission`) an OmniLink agent can drive to fly a coloured-marker perimeter survey at 12 m. The bridge exposes the survey surface; no agent ships against it today.

### [cuda\_particles\_smoke\_test.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/physics/cuda_particles_smoke_test.omniworld)

**Keywords**: CUDA particles, GPU compute, smoke effects

Demonstrates the CUDA-accelerated particle pool — debris bursts on contact events. Requires a CUDA-capable GPU.

### [granular\_sand\_demo.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/physics/granular_sand_demo.omniworld)

**Keywords**: Granular media (rigid-body approximation), contact physics, angle of repose

~300 rigid `Sphere` bodies drop into a square pit and pile under gravity, behaving like coarse
granular media at pebble scale — it settles, holds an angle of repose, and can be displaced by
anything pushing through it.

⚠️ **Two corrections to what this page used to say.** There is **no Husky** in this world (no
`Robot` node of any kind), and it does **not** use a granular-media solver — it contains zero
`GranularGroup` nodes and is ordinary rigid-body physics. The separate CUDA `GranularGroup`
solver does exist and is benchmarked (100k particles at 4.50 ms/step), but no demo world uses
it, and its robot↔particle coupling is currently non-functional in both directions. The only
`GranularGroup` worlds in the tree are under `tests/cuda/`.

### [newton\_husky\_head\_on.wbt]({{ url.github_tree }}/projects/robot_combat/worlds/tests/newton_husky_head_on.omniworld)

**Keywords**: Newton physics backend, multi-robot collision, robot combat

Two teams of four Huskies driven head-on under the Newton physics backend — which, since ODE was deleted on 2026-08-08, is the only backend, so `physicsBackend "auto"` (the default) is all any world needs. Do **not** write `physicsBackend "ode"`: the value still parses but no longer selects a working solver, and a Solid pinned to it is not simulated. Lives under [`projects/robot_combat/`]({{ url.github_tree }}/projects/robot_combat/), the development environment for OmniSim's robot-combat demos. Companion worlds: `newton_husky_head_on_2.omniworld`, `newton_husky_head_on_damage.omniworld`, `newton_husky_combat_2.omniworld`.

### Full list

`ls projects/samples/demos/worlds/` for the complete set, and `ls distribution/generated_worlds/` for procedurally generated terrains (Mars, indoor apartment, urban block, etc.).
