## Demos

This section provides a list of interesting worlds that broadly illustrate OmniSim capabilities.
You will find the corresponding ".wbt" files in the "[OMNISIM\_HOME/projects/samples/demos/worlds]({{ url.github_tree }}/projects/samples/demos/worlds)" directory, and their controller source code in the "[OMNISIM\_HOME/projects/samples/demos/controllers]({{ url.github_tree }}/projects/samples/demos/controllers)" directory.

The OmniLink-driven demos are the canonical starting points; see [OmniLink chat demos](omnilink-chat-demos.md) for the agent-facing walkthrough. The list below picks out a few additional worlds that highlight specific OmniSim capabilities.

### [warehouse\_husky.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/showcase/warehouse_husky.wbt)

**Keywords**: URDF importer, Husky, supervisor, reactive control

The default onboarding demo. A supervisor-enabled Husky (URDFRobot, `husky_random` controller) random-walks a 30 × 18 m warehouse with reactive collision recovery. Good showcase of the URDF importer, supervisor APIs, motor torque/sensor pipeline, and the camera follow (F key).

### [husky\_maze.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/flagship/husky_maze.wbt)

**Keywords**: URDF importer, OmniLink bridge, navigation

A single Husky placed at the NW corner of an 11 × 11-cell walled maze. The `husky_omnilink_bridge` controller exposes an HTTP control surface so an external OmniLink agent can drive the robot. See sibling worlds (`husky_maze_blind.wbt`, `husky_maze_corners.wbt`, `husky_maze_unknown.wbt`, `husky_maze_visual.wbt`) for progressively harder briefs.

### [omnilink\_mavic.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/chat/omnilink_mavic.wbt)

**Keywords**: Quadrotor, waypoint flight, perimeter survey, marker detection

A DJI Mavic 2 Pro quadrotor demo with two surfaces on one world: right-click chat verbs ("takeoff", "forward 1 m", "land") for a human operator, and the full Drone Surveyor flagship agent tool set (`scan_for_markers`, `goto_waypoint`, `complete_mission`) for an OmniLink agent flying a coloured-marker perimeter survey at 12 m.

### [cuda\_particles\_smoke\_test.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/physics/cuda_particles_smoke_test.wbt)

**Keywords**: CUDA particles, GPU compute, smoke effects

Demonstrates the CUDA-accelerated particle pool — debris bursts on contact events. Requires a CUDA-capable GPU.

### [granular\_sand\_demo.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/physics/granular_sand_demo.wbt)

**Keywords**: Granular media, contact physics, Husky traversal

A Husky drives across a sandy bed simulated with the granular-media solver.

### [newton\_husky\_head\_on.wbt]({{ url.github_tree }}/projects/robot_combat/worlds/tests/newton_husky_head_on.wbt)

**Keywords**: Newton physics backend, multi-robot collision, robot combat

Two teams of four Huskies driven head-on under the Newton physics backend. Newton is the **default** where its runtime is present (`physicsBackend "auto"`); set `physicsBackend "ode"` to pin a Solid to the legacy solver. Lives under [`projects/robot_combat/`]({{ url.github_tree }}/projects/robot_combat/), the development environment for OmniSim's robot-combat demos. Companion worlds: `newton_husky_head_on_2.wbt`, `newton_husky_head_on_damage.wbt`, `newton_husky_combat_2.wbt`.

### [warehouse\_logistics.wbt]({{ url.github_tree }}/projects/samples/demos/worlds/flagship/warehouse_logistics.wbt)

**Keywords**: Husky fleet, Warehouse Foreman, pallet logistics

Move the GREEN-tagged pallet to the loading dock: six colour-tagged pallets in a 2×3 grid, with the Warehouse Foreman running the Picker-only mission over OmniLink.

### Full list

`ls projects/samples/demos/worlds/` for the complete set, and `ls distribution/generated_worlds/` for procedurally generated terrains (Mars, indoor apartment, urban block, etc.).
