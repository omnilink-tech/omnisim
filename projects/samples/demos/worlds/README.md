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
| [omnilink_launcher.omniworld](omnilink_launcher.omniworld) | Demo launcher — right-click the orb → Show Robot Window, pick a demo, click Launch. |

## Flagship (`flagship/`)

| World | Description |
|-------|-------------|
| [husky_maze.omniworld](flagship/husky_maze.omniworld) | Drive the Husky to the goal cell at the SE corner of the maze; BFS the revealed map and execute cell by cell. |
| [husky_maze_blind.omniworld](flagship/husky_maze_blind.omniworld) | Husky maze, blind variant (no map reveal). |
| [husky_maze_corners.omniworld](flagship/husky_maze_corners.omniworld) | Husky maze, corners-tour variant. |
| [husky_maze_unknown.omniworld](flagship/husky_maze_unknown.omniworld) | Husky maze, unknown-map variant (explore to find the goal). |
| [husky_maze_visual.omniworld](flagship/husky_maze_visual.omniworld) | Husky maze, vision-based variant. |
| [omnilink_husky_swarm.omniworld](flagship/omnilink_husky_swarm.omniworld) | Four Huskies (NE/NW/SE/SW) under the OmniLink mobile bridge. |
| [warehouse_industrial.omniworld](flagship/warehouse_industrial.omniworld) | Industrial warehouse environment. |

## Chat / OmniLink (`chat/`)

Conversational-control worlds, one per robot (see [OMNILINK_CHAT_DEMOS.md](chat/OMNILINK_CHAT_DEMOS.md)).

| World | Robot |
|-------|-------|
| [omnilink_husky.omniworld](chat/omnilink_husky.omniworld) | Husky (mobile) |
| [omnilink_jackal.omniworld](chat/omnilink_jackal.omniworld) | Jackal (mobile) |
| [omnilink_rosbot.omniworld](chat/omnilink_rosbot.omniworld) | Rosbot (mobile) |
| [omnilink_rosbot_xl.omniworld](chat/omnilink_rosbot_xl.omniworld) | Rosbot XL (mobile) |
| [omnilink_tb3_burger.omniworld](chat/omnilink_tb3_burger.omniworld) | TurtleBot3 Burger (mobile) |
| [omnilink_tb3_waffle.omniworld](chat/omnilink_tb3_waffle.omniworld) | TurtleBot3 Waffle (mobile) |
| [omnilink_tb3_waffle_pi.omniworld](chat/omnilink_tb3_waffle_pi.omniworld) | TurtleBot3 Waffle Pi (mobile) |
| [omnilink_omniquad.omniworld](chat/omnilink_omniquad.omniworld) | OmniQuad (quadruped) |
| [omnilink_mavic.omniworld](chat/omnilink_mavic.omniworld) | Mavic 2 Pro drone — chat + perimeter survey |

## Physics / Newton (`physics/`)

| World | Description |
|-------|-------------|
| [newton_smoke_test.omniworld](physics/newton_smoke_test.omniworld) | Newton backend smoke test. |
| [newton_husky_smoke_test.omniworld](physics/newton_husky_smoke_test.omniworld) | Newton Husky smoke test. |
| [newton_husky_swarm_drive.omniworld](physics/newton_husky_swarm_drive.omniworld) | Newton Husky swarm, 8× driving. |
| [cuda_particles_smoke_test.omniworld](physics/cuda_particles_smoke_test.omniworld) | CUDA particle pool smoke test. |
| [granular_sand_demo.omniworld](physics/granular_sand_demo.omniworld) | ~300 rigid spheres piling at pebble scale. Not the `GranularGroup` solver. |

## Showcase (`showcase/`)

| World | Description |
|-------|-------------|
| [husky_fleet_arena.omniworld](showcase/husky_fleet_arena.omniworld) | Husky fleet arena. |
| [husky_rocks_traverse.omniworld](showcase/husky_rocks_traverse.omniworld) | Husky traversing a rock-strewn hill (terrain-nav controller). |
| [jackal_drive.omniworld](showcase/jackal_drive.omniworld) | Jackal URDF drive demo. |
| [turtlebot3_drive.omniworld](showcase/turtlebot3_drive.omniworld) | TurtleBot3 trio (URDF import). |
| [warehouse_husky.omniworld](showcase/warehouse_husky.omniworld) | Warehouse Husky. |

## Environments (`environments/`)

| World | Description |
|-------|-------------|
| [city.omniworld](environments/city.omniworld) | Mixed urban street block — avenue + side street, mixed buildings, traffic lights, pocket park; clear midday. |
| [desert_ruins.omniworld](environments/desert_ruins.omniworld) | Desert ruins environment. |
| [forest.omniworld](environments/forest.omniworld) | Forest environment. |

## Dev (`dev/`)

Editable scratch worlds for iterating on assets.

| World | Description |
|-------|-------------|
| [construction_site_dev.omniworld](dev/construction_site_dev.omniworld) | Construction site, fully-editable dev world. |
| [site_env_preview.omniworld](dev/site_env_preview.omniworld) | Construction-site environment preview. |
| [vehicle_preview.omniworld](dev/vehicle_preview.omniworld) | Procedural construction-vehicle preview. |

## Misc (`misc/`)

| World | Description |
|-------|-------------|
| [cylinder_stack.omniworld](misc/cylinder_stack.omniworld) | Cylinder-stack physics scene (Khepera III). |
