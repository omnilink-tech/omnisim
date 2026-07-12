# OmniSim Demo Worlds

Index of every demo world under `projects/samples/demos/worlds/`, grouped by
folder. Descriptions are drawn from each world's `WorldInfo` title/info.

Launch any world from the editor, or via the dev helper:

```bash
python scripts/dev/omnisim_dev.py run-world projects/samples/demos/worlds/<path>.wbt
```

Robot-combat demos (head-on, damage arenas, brawls/duels, hunt-AI matches) have moved to their own development environment at [`projects/robot_combat/`](../../../robot_combat/) — see its [README](../../../robot_combat/README.md).

## Top level

| World | Description |
|-------|-------------|
| [omnilink_launcher.wbt](omnilink_launcher.wbt) | Demo launcher — right-click the orb → Show Robot Window, pick a demo, click Launch. |
| [omnilink_mission_control.wbt](omnilink_mission_control.wbt) | 6-Husky fleet on a 30×30 m logistics campus with 12 named zones; dispatch via natural-language operator missions over OmniLink. |

## Flagship (`flagship/`)

| World | Description |
|-------|-------------|
| [husky_maze.wbt](flagship/husky_maze.wbt) | Drive the Husky to the goal cell at the SE corner of the maze; BFS the revealed map and execute cell by cell. |
| [husky_maze_blind.wbt](flagship/husky_maze_blind.wbt) | Husky maze, blind variant (no map reveal). |
| [husky_maze_corners.wbt](flagship/husky_maze_corners.wbt) | Husky maze, corners-tour variant. |
| [husky_maze_unknown.wbt](flagship/husky_maze_unknown.wbt) | Husky maze, unknown-map variant (explore to find the goal). |
| [husky_maze_visual.wbt](flagship/husky_maze_visual.wbt) | Husky maze, vision-based variant. |
| [omnilink_husky_swarm.wbt](flagship/omnilink_husky_swarm.wbt) | Four Huskies (NE/NW/SE/SW) under the OmniLink mobile bridge. |
| [warehouse_industrial.wbt](flagship/warehouse_industrial.wbt) | Industrial warehouse environment. |
| [warehouse_logistics.wbt](flagship/warehouse_logistics.wbt) | Move the GREEN-tagged pallet to the loading dock; six colour-tagged pallets in a 2×3 grid. |
| [warehouse_patrol.wbt](flagship/warehouse_patrol.wbt) | Patrol your sector and report what moved since the last sweep; Huskies split north/south sectors. |

## Chat / OmniLink (`chat/`)

Conversational-control worlds, one per robot (see [OMNILINK_CHAT_DEMOS.md](chat/OMNILINK_CHAT_DEMOS.md)).

| World | Robot |
|-------|-------|
| [omnilink_husky.wbt](chat/omnilink_husky.wbt) | Husky (mobile) |
| [omnilink_jackal.wbt](chat/omnilink_jackal.wbt) | Jackal (mobile) |
| [omnilink_rosbot.wbt](chat/omnilink_rosbot.wbt) | Rosbot (mobile) |
| [omnilink_rosbot_xl.wbt](chat/omnilink_rosbot_xl.wbt) | Rosbot XL (mobile) |
| [omnilink_tb3_burger.wbt](chat/omnilink_tb3_burger.wbt) | TurtleBot3 Burger (mobile) |
| [omnilink_tb3_waffle.wbt](chat/omnilink_tb3_waffle.wbt) | TurtleBot3 Waffle (mobile) |
| [omnilink_tb3_waffle_pi.wbt](chat/omnilink_tb3_waffle_pi.wbt) | TurtleBot3 Waffle Pi (mobile) |
| [omnilink_spot.wbt](chat/omnilink_spot.wbt) | Spot (quadruped) |
| [omnilink_mavic.wbt](chat/omnilink_mavic.wbt) | Mavic 2 Pro drone surveyor |

## Physics / Newton (`physics/`)

| World | Description |
|-------|-------------|
| [newton_smoke_test.wbt](physics/newton_smoke_test.wbt) | Newton backend smoke test. |
| [newton_husky_smoke_test.wbt](physics/newton_husky_smoke_test.wbt) | Newton Husky smoke test. |
| [newton_husky_swarm_drive.wbt](physics/newton_husky_swarm_drive.wbt) | Newton Husky swarm, 8× driving. |
| [cuda_particles_smoke_test.wbt](physics/cuda_particles_smoke_test.wbt) | CUDA particle pool smoke test. |
| [granular_sand_demo.wbt](physics/granular_sand_demo.wbt) | Granular sand demo. |

## Showcase (`showcase/`)

| World | Description |
|-------|-------------|
| [husky_fleet_arena.wbt](showcase/husky_fleet_arena.wbt) | Husky fleet arena. |
| [husky_rocks_traverse.wbt](showcase/husky_rocks_traverse.wbt) | Husky traversing a rock-strewn hill (terrain-nav controller). |
| [jackal_drive.wbt](showcase/jackal_drive.wbt) | Jackal URDF drive demo. |
| [turtlebot3_drive.wbt](showcase/turtlebot3_drive.wbt) | TurtleBot3 trio (URDF import). |
| [warehouse_husky.wbt](showcase/warehouse_husky.wbt) | Warehouse Husky. |

## Environments (`environments/`)

| World | Description |
|-------|-------------|
| [city.wbt](environments/city.wbt) | Mixed urban street block — avenue + side street, mixed buildings, traffic lights, pocket park; clear midday. |
| [desert_ruins.wbt](environments/desert_ruins.wbt) | Desert ruins environment. |
| [forest.wbt](environments/forest.wbt) | Forest environment. |

## Dev (`dev/`)

Editable scratch worlds for iterating on assets.

| World | Description |
|-------|-------------|
| [construction_site_dev.wbt](dev/construction_site_dev.wbt) | Construction site, fully-editable dev world. |
| [site_env_preview.wbt](dev/site_env_preview.wbt) | Construction-site environment preview. |
| [vehicle_preview.wbt](dev/vehicle_preview.wbt) | Procedural construction-vehicle preview. |

## Misc (`misc/`)

| World | Description |
|-------|-------------|
| [cylinder_stack.wbt](misc/cylinder_stack.wbt) | Cylinder-stack physics scene (Khepera III). |
