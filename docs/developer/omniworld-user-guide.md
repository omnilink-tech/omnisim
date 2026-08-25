# omniworld User Guide

`omniworld` is the OmniSim procedural world generation library. This guide documents the parts that have actually landed. For the forward-looking roadmap — biomes, scenarios, real-world import, learned generation — see [procedural-world-generation-plan.md](procedural-world-generation-plan.md).

## Status

Landed (T1.1 – T1.4 + all five biomes from T1.6, plus `mars`):

- Pure-Python library at [src/python/omniworld/](../../src/python/omniworld/).
- `omniworld` CLI launched via [scripts/dev/omniworld.py](../../scripts/dev/omniworld.py).
- Seven recipes total:
  - `flat_ground` — a flat rectangular arena for pipeline smoke-tests.
  - `outdoor_forest` — fBm terrain, Poisson-scattered trees, sparse rocks, optional road, exclusive spawn clearing.
  - `outdoor_desert` — ridged+fbm mesa terrain, sparse sandy rocks with per-instance scale and tint.
  - `warehouse` — axis-aligned pallet-stack racks in rows, Poisson-scattered boxes in the aisles, loading-dock spawn.
  - `urban_block` — 2×2 city block with a central cross road, one customised `Building` per lot, street lights and benches via `along_path` scatter.
  - `indoor_apartment` — three-room strip (living / bedroom / bathroom) with perimeter walls, door-split dividers, one furniture prop per room.
  - `mars` — ridged+fbm relief with impact craters, three-layer rock scatter, ochre sky, and atmospheric fog.
- Seed manifest (`.wbt.seed.json`) alongside every generated world.
- Three in-process validators: asset locality, prop overlap, spawn reachability.
- Heightmap primitives in [primitives/heightmap.py](../../src/python/omniworld/primitives/heightmap.py): `Heightmap` container with bilinear `sample`; noise generators `fbm`, `ridged`, `worley`; masks `mask_radial`, `mask_rect`, `mask_polygon`; composition `blend`, `apply_mask`, `stamp`; erosion `erode_thermal`, `erode_hydraulic`.
- Scatter primitives in [primitives/scatter.py](../../src/python/omniworld/primitives/scatter.py): `poisson_annulus`, `poisson_polygon` (spatial-hash-backed, handles concave polygons), `grid_jittered`, `clustered`, `along_path`, `on_surface` (heightmap projection with optional slope filter), plus `pick` / `pick_weighted` helpers.
- Layout DSL in [core/layout.py](../../src/python/omniworld/core/layout.py): `Zone`, `Path`, `Stamp`, `PropGroup`, `Layout` dataclasses with reference validation.
- Solver in [core/solver.py](../../src/python/omniworld/core/solver.py): turns a `Layout` plus a seed into a `SolvedWorld` (heightmap + prop placements + spawns). Honors zone priority, `exclusive` zones, explicit include/exclude, path corridors with margin. All sub-RNGs derived by name so insertions don't perturb existing output.
- JSON schema in [core/schema.py](../../src/python/omniworld/core/schema.py): lossless `to_dict` / `from_dict` / `to_json` / `from_json`. Unknown keys raise; versioned by `SCHEMA_VERSION`.
- ElevationGrid emitter in [emit/elevation_grid.py](../../src/python/omniworld/emit/elevation_grid.py): `render_elevation_grid` for bare geometry, `render_terrain_solid` for a wrapped `Solid > Shape`.
- Scaffold and primitive tests under [tests/python/omniworld/](../../tests/python/omniworld/).
- Biome cookbook docs (T1.8): [omniworld-biome-cookbook.md](omniworld-biome-cookbook.md).

Not yet landed. Called out for context only:

- PROTO-scanning asset catalog — auto-derive entries from `.proto` files (bounding boxes, collision tier, surface manifests). T1.5 today ships a hand-authored `catalog/assets.json` with a validator, which is the consolidation half of the plan's T1.5 scope.
- Headless-sim validators: the load-time check (T1.7) has landed (opt-in via `with_sim=True`); the contact-pair budget still needs a custom supervisor.
- Physics settling (T1.9).

## Install and launch

The library is pure Python 3 and has zero runtime dependencies. No build or install step is required. From the repo root:

```
python scripts/dev/omniworld.py --version
python scripts/dev/omniworld.py list-recipes
python scripts/dev/omniworld.py describe flat_ground
```

Equivalent module form:

```
python -m omniworld --version
```

For the module form to work without fiddling, put `src/python` on `PYTHONPATH`, or use the [scripts/dev/omniworld.py](../../scripts/dev/omniworld.py) launcher which handles `sys.path` for you.

## Generate a world

```
python scripts/dev/omniworld.py generate flat_ground \
    --seed 7 \
    --out /tmp/flat.wbt
```

Output:

- `/tmp/flat.wbt` — the world file.
- `/tmp/flat.wbt.seed.json` — the manifest: library version, recipe, seed, resolved parameters, SHA256 of the world bytes.

Every generator call runs validation by default. Pass `--skip-validate` to suppress it (debugging only; the CI gate always validates).

Generate an `outdoor_forest` world with a robot spawn:

```
python scripts/dev/omniworld.py generate outdoor_forest \
    --seed 42 \
    --out /tmp/forest.omniworld \
    --param size=60 \
    --param tree_count=120 \
    --param spawn_urdf=\"../robots/cube_bot.urdf\"
```

Override recipe parameters with `--param KEY=VALUE`, repeated. Values are parsed as JSON (with a string fallback):

```
python scripts/dev/omniworld.py generate flat_ground \
    --seed 7 \
    --out /tmp/flat.wbt \
    --param size=20 \
    --param spawn_urdf=\"../robots/cube_bot.urdf\" \
    --param spawn_controller=\"omnibot_agent\"
```

## Validate a world

```
python scripts/dev/omniworld.py validate /tmp/flat.wbt
```

Exit status is `0` when every check passes and `1` when any fail. Tier 1 ships three in-process checks:

| Check                | What it catches                                                                 |
|----------------------|----------------------------------------------------------------------------------|
| `asset_locality`     | Remote `http(s)://` or `ftp://` references in the world.                         |
| `prop_overlap`       | Two placed nodes closer than 0.3 m (full bounding-volume check lands with T1.5). |
| `spawn_reachability` | `URDFRobot` translations with NaN coordinates or below-floor z.                 |

## Python API

```python
import omniworld

result = omniworld.generate("flat_ground", seed=7, out="/tmp/flat.wbt")
print(result.manifest.world_sha256)

report = omniworld.validate(result.world_path)
assert report.ok, report.format()
```

Everything the CLI does is reachable through the Python API.

## Determinism contract

- Same `(recipe, seed, params)` → byte-identical `.wbt`.
- Same `(recipe, seed, params, out_basename)` → byte-identical `.seed.json` (no absolute paths in the manifest).
- No primitive reads the global RNG or the wall clock.
- Sub-RNGs are derived by name from the parent seed (`core.rng.derive_seed`), so adding a new named consumer does not perturb existing consumers.

See `tests/python/omniworld/test_scaffold.py` for the executable form of the contract.

## Heightmap primitives

All heightmap primitives live in [primitives/heightmap.py](../../src/python/omniworld/primitives/heightmap.py) and are also re-exported from `omniworld.primitives`.

```python
from omniworld.primitives.heightmap import (
    Heightmap,
    fbm, ridged, worley,
    mask_radial, mask_rect, mask_polygon,
    blend, apply_mask, stamp,
    erode_thermal, erode_hydraulic,
)

h = fbm(256, 256, seed=42, octaves=5)           # rolling terrain
ridge = ridged(256, 256, seed=7)                # sharp canyons
cells = worley(256, 256, seed=11, variant="f1") # cellular
m = mask_radial(256, 256, radius=100.0)         # flatten centre
carved = erode_hydraulic(h, seed=42, droplets=1000, max_steps=30)
terrain = apply_mask(carved, m, mode="replace", strength=0.0)
```

Each generator takes an explicit `seed` (or operates on a `Heightmap` produced from one). No global RNG, no clock reads. See the plan's cross-tier determinism section for the full contract.

Performance notes (pure Python, no numpy acceleration yet):

- `fbm(256, 256, octaves=5)` ≈ 2.5 s
- `erode_thermal(steps=5)` on 256² ≈ 0.3 s
- `erode_hydraulic(droplets=1000, max_steps=30)` on 256² ≈ 0.2 s (well under the plan's 500 ms budget)

Numpy acceleration is planned for when recipes start compounding per-run costs; the existing API is fallback-friendly.

## Scatter primitives

All scatter primitives take an explicit `random.Random` — no global state, no clock reads.

```python
import random
from omniworld.primitives.scatter import (
    poisson_annulus, poisson_polygon, grid_jittered,
    clustered, along_path, on_surface, pick_weighted,
)

rng = random.Random(42)

rocks = poisson_annulus(rng, count=48, r_min=2.0, r_max=24.0, min_dist=1.2)
orchard = grid_jittered(rng, polygon=lot, cell_size=3.0, jitter=0.2)
stand = clustered(rng, centres=[(-10, 0), (10, 0)], count_per=12, sigma=1.5, min_dist=0.8)
roadside = along_path(rng, path=[(0, 0), (100, 0)], density=0.25, offset=2.5, side="both")

# Lift to 3D by sampling terrain.
lifted = on_surface(rocks, hm, grid_origin=(-32, -32),
                    x_spacing=0.5, y_spacing=0.5, height_scale=4.0)

# Weighted asset selection (catalog queries land with T1.5).
tree_protos = ["OakTree", "BirchTree", "PineTree"]
weights = [0.5, 0.3, 0.2]
chosen = [tree_protos[pick_weighted(rng, weights)] for _ in lifted]
```

`poisson_polygon` uses a spatial-hash-backed rejection sampler, so cost is near `O(count)` even on narrow or concave shapes. When the polygon can't accommodate the requested count at the given spacing, the function returns fewer points rather than looping forever.

## Realism: variation and weathering (T1.10)

Every biome takes an optional `world_age` parameter in `[0, 1]` — `0.0` is pristine, `1.0` is abandoned. Per-instance variation and weathering derive deterministically from `(seed, group_name, instance_index)` so two runs at the same age produce byte-identical output.

```python
# Same seed, different age -> visibly different rock colours.
generate("outdoor_forest", seed=42, out="young.wbt", params={"world_age": 0.0})
generate("outdoor_forest", seed=42, out="old.wbt",   params={"world_age": 0.8})
```

The API under `omniworld.realism`:

- `variation_for_instance(seed, group, idx, ...)` — returns a `Variation` (scale, hue shift, rotation offset).
- `compute_grade(seed, group, idx, world_age, ...)` — returns a `WeatheringGrade` 4-tuple (dirt, wear, moisture, oxidation) ready for the extended-BRDF shader planned in the renderer tier plan.
- `grade_to_color_multiplier(grade)` — fallback darkening factor for PROTOs that only support a `color` field.
- `jitter_hue_rgb(base_rgb, hue_deg)` — HSL-preserving hue shift used by the forest rocks.

Biomes vary in what they can take advantage of today: Rock has a `color` field so the forest shifts rock hue and darkens per-instance; Tree PROTOs do not, so trees vary only in scale via yaw rotation already applied by the solver. Adding a color field to a new PROTO is enough for the existing pipeline to pick it up.

## Nested scatter (T1.11)

Catalog entries can declare flat top surfaces ("desk top", "pallet top", "shelf tier") as `{ "z_offset": float, "half_extents": [x, y] }`. The `scatter_on_surface` primitive takes a parent placement + surface name and produces child placements on top, honoring the parent's yaw so a tipped parent gets tipped clutter.

The shipped `warehouse` biome uses this to place cardboard boxes and plastic crates on top of a subset of pallet stacks:

```python
# Default is 60% of racks get 2 boxes on top. Disable with fraction=0.
generate("warehouse", seed=42, out="w.wbt",
         params={"rack_top_clutter_fraction": 0.0})
```

## Asset catalog

Every PROTO the shipped biomes place is declared in [catalog/assets.json](../../src/python/omniworld/catalog/assets.json). Biomes look up URLs through the query API:

```python
from omniworld.catalog import get_url, iter_tagged

# Single lookup.
tree_url = get_url("Oak")

# Tag-filtered search.
forest_trees = iter_tagged("tree", "forest")
names = [a.name for a in forest_trees]
```

Validate the catalog with:

```
python scripts/dev/build_asset_catalog.py
```

The validator confirms the shipped biomes' PROTO needs are covered, URLs are `omnisim://`-scheme, and the schema version matches. Full auto-generation from raw `.proto` files lands later.

## Layout DSL and solver

Biome recipes describe a world declaratively with zones, paths, stamps, prop groups, and spawns, then hand the `Layout` to the solver:

```python
from omniworld.core.layout import Layout, Zone, Path, Stamp, PropGroup
from omniworld.core.recipe import Spawn
from omniworld.core.solver import solve

layout = Layout(
    world_size=(64.0, 64.0),
    terrain_params={"kind": "fbm", "grid_resolution": 128, "height_scale": 2.0},
)
layout.add_zone(Zone(name="forest",
                     polygon=((0, 0), (64, 0), (64, 64), (0, 64))))
layout.add_zone(Zone(name="spawn_clearing",
                     polygon=((28, 28), (36, 28), (36, 36), (28, 36)),
                     priority=10, exclusive=True))
layout.add_path(Path(name="road",
                     polyline=((0, 32), (64, 32)),
                     width=4.0))
layout.add_prop_group(PropGroup(
    name="trees",
    proto_weights=(("OakTree", 0.6), ("BirchTree", 0.4)),
    scatter="poisson",
    include_zones=("forest",),
    exclude_paths=("road",),
    exclude_margin=0.5,
    params=(("count", 200), ("min_dist", 1.2)),
))
layout.add_spawn(Spawn(name="robot", translation=(32.0, 32.0, 0.1),
                       urdf_url="../robots/cube_bot.urdf"))

solved = solve(layout, seed=42)
print(f"{len(solved.props)} props placed")
```

`solve()` honors:

- **Zone priority / exclusivity.** An `exclusive=True` zone with higher priority than a `PropGroup`'s include zones acts as an implicit excluder — spawn clearings stay clear automatically.
- **Path corridors.** Points closer than `path.width / 2 + group.exclude_margin` to an excluded path's polyline are rejected.
- **Determinism.** Same `(layout, seed)` gives byte-identical output. Sub-RNGs are derived by name (`propgroup.trees.pick`, `propgroup.trees.rot`, etc.) so adding a new prop group does not perturb existing ones.

A `Layout` round-trips through JSON losslessly:

```python
from omniworld.core.schema import to_json, from_json

text = to_json(layout)
restored = from_json(text)
assert solve(restored, seed=42).props == solve(layout, seed=42).props
```

## ElevationGrid emission

```python
from omniworld.emit import render_elevation_grid, render_terrain_solid

grid = render_elevation_grid(h, x_spacing=0.25, y_spacing=0.25)
solid = render_terrain_solid(h, x_spacing=0.25, y_spacing=0.25, name="ground")
```

`render_terrain_solid` wraps the geometry in a `DEF <name.upper()> Solid > Shape { appearance, geometry }` with a matching `boundingObject` so collision works for free.

## Writing a recipe

See [omniworld-biome-cookbook.md](omniworld-biome-cookbook.md) for a full walkthrough: reading a shipped recipe, zone priority rules, a step-by-step template for adding a new biome, and common pitfalls.
