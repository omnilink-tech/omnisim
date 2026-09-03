# omniworld Biome Cookbook

This guide walks through how a biome recipe is structured and how to add a new one. It complements the user-facing [omniworld-user-guide.md](omniworld-user-guide.md) and the long-range procedural-world-generation-plan.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)).

If you have not read them, start there — the user guide tells you what the library does today, the plan tells you where it is going, and this cookbook tells you how to extend it.

## How a recipe is shaped

Every shipped recipe follows the same four-step shape:

1. **Parameter dict.** A `DEFAULT_PARAMS` mapping declares every knob the recipe accepts, with default values. Unknown keys are rejected at `build` time.
2. **Layout builder.** `_build_layout(params)` returns a `Layout` populated with zones, paths, stamps, prop groups, and spawns. No randomness, no file I/O — just declarative data.
3. **Solver call.** `solve(layout, seed=seed)` resolves the Layout into a `SolvedWorld` (heightmap + prop placements + spawns). All stochastic work happens here.
4. **Emission mapping.** The recipe translates each `PropPlacement` into a `PlacedProto` (catalog URL, PROTO type, world transform, extra fields) and returns a `WorldDescription`.

Some recipes hand-place mechanically precise structures (rack grids in `warehouse`, buildings in `urban_block`, walls in `indoor_apartment`) outside the solver. That is a first-class pattern — the solver is for stochastic placements; anything whose geometry is exactly specified belongs in the recipe body.

## Reading a recipe: `outdoor_forest`

The shipped `outdoor_forest` recipe is the canonical example. Path: [src/python/omniworld/biomes/outdoor_forest.py](../../src/python/omniworld/biomes/outdoor_forest.py).

Structural outline:

```python
DEFAULT_PARAMS = {
    "size": 50.0,
    "grid_resolution": 128,
    "height_scale": 2.0,
    ...
    "tree_count": 80,
    "tree_weights": {"Oak": 0.45, "Pine": 0.30, ...},
    "world_age": 0.0,
    ...
}


def _build_layout(params):
    layout = Layout(
        world_size=(size, size),
        terrain_params={"kind": "fbm", "grid_resolution": 128, ...},
    )
    layout.add_zone(Zone(name="forest", polygon=...))
    layout.add_zone(Zone(name="spawn_clearing", polygon=...,
                         priority=10, exclusive=True))
    layout.add_stamp(Stamp(name="clearing_pad", zone="spawn_clearing",
                           mode="replace", height=...))
    if road_width > 0:
        layout.add_path(Path(name="road", polyline=..., width=road_width))
    layout.add_prop_group(PropGroup(
        name="trees", proto_weights=..., scatter="poisson",
        include_zones=("forest",),
        exclude_paths=("road",),
        exclude_margin=0.5,
        params=(("count", tree_count), ("min_dist", tree_min_dist)),
    ))
    ...
    return layout


@dataclass
class OutdoorForest:
    name = "outdoor_forest"

    def build(self, seed, params):
        merged = {**DEFAULT_PARAMS, **params}
        layout = _build_layout(merged)
        solved = solve(layout, seed=seed)
        # Translate every PropPlacement into a PlacedProto with catalog URLs.
        props = [...]
        return WorldDescription(
            title="outdoor_forest (omniworld)",
            heightmap=solved.heightmap,
            heightmap_spacing=..., heightmap_origin=(0.0, 0.0),
            props=props,
            spawns=list(solved.spawns),
            metadata={"recipe": self.name, "prop_count": len(props)},
        )


register_recipe(OutdoorForest())
```

Key points:

- **Exclusive zones clear everything.** The `spawn_clearing` zone has `priority=10, exclusive=True`, so the solver implicitly excludes it from every prop group that operates on `forest` (lower priority). Recipes get spawn clearings for free — no per-group exclude_zones boilerplate.
- **Path exclusion is explicit.** A prop group that must avoid a road declares `exclude_paths=("road",)` and an `exclude_margin` in metres. The solver checks perpendicular distance against the polyline.
- **Scatter is parameterised via `PropGroup.params`.** Key-value tuples carry `count`, `min_dist`, `density`, etc. The solver's primitive dispatch reads these keys by name.
- **Per-instance realism is a separate pass.** The recipe consults `variation_for_instance` and `compute_grade` when building `PlacedProto` records — the solver does not bake variation in. This keeps the solver's output stable and makes the variation parameters swappable at emission time.

## Zone priority and conflict rules

When two zones overlap, the solver's `_effective_zones_for` applies this rule:

> An `exclusive=True` zone whose `priority` is strictly higher than the maximum priority of a PropGroup's `include_zones` acts as an implicit exclude zone for that group.

Practical consequences:

- A spawn clearing at `priority=10, exclusive=True` over a forest zone at `priority=0` clears every prop group that targets the forest.
- Two zones at the same priority can overlap — the solver treats both as valid.
- `exclude_zones` is still evaluated; an explicit exclude always wins.
- Paths are not zones; they have their own `exclude_paths` mechanism.

## Adding a new biome

### Step 1 — decide what's stochastic vs mechanical

Ask: what parts of this world are "random within bounds" versus "exactly specified"?

- Forests, rock fields, clutter piles → **stochastic**, use `PropGroup`.
- Building grids, wall perimeters, rack rows → **mechanical**, compute coords in the recipe and emit `PlacedProto` directly.
- Mixed: `warehouse` is mechanical for racks, stochastic for boxes. `urban_block` is mechanical for buildings, stochastic for street furniture.

### Step 2 — inventory the PROTOs you need

Check [src/python/omniworld/catalog/assets.json](../../src/python/omniworld/catalog/assets.json). If every PROTO you plan to use is already there, move on. If not, add an entry:

```json
{
  "name": "SomeNewProto",
  "url": "omnisim://projects/objects/.../SomeNewProto.proto",
  "tags": ["category", "biome", "..."],
  "settle": true,
  "surfaces": {
    "top": {
      "z_offset": 0.75,
      "half_extents": [0.4, 0.3]
    }
  }
}
```

Then run the validator:

```
python scripts/dev/build_asset_catalog.py
```

It will flag missing entries for any biome that has been updated to reference them. (For a brand-new biome, add your biome's expected PROTO set to the `REQUIRED_BY_BIOMES` dict in [scripts/dev/build_asset_catalog.py](../../scripts/dev/build_asset_catalog.py).)

### Step 3 — write the recipe

Put it under [src/python/omniworld/biomes/my_biome.py](../../src/python/omniworld/biomes/). Minimal template:

```python
from dataclasses import dataclass
from typing import Any, Mapping

from ..catalog import get_url
from ..core.layout import Layout, Zone, PropGroup
from ..core.recipe import PlacedProto, Spawn, WorldDescription
from ..core.registry import register_recipe
from ..core.solver import solve


DEFAULT_PARAMS: dict[str, Any] = {
    "size": 30.0,
    "spawn_urdf": None,
    "spawn_controller": None,
    "spawn_height": 0.2,
}


def _build_layout(params: Mapping[str, Any]) -> Layout:
    size = float(params["size"])
    layout = Layout(world_size=(size, size))
    layout.add_zone(Zone(
        name="floor",
        polygon=((0.0, 0.0), (size, 0.0), (size, size), (0.0, size)),
    ))
    # ... add your zones / paths / prop groups / spawns
    return layout


@dataclass
class MyBiome:
    name: str = "my_biome"

    def default_params(self) -> dict[str, Any]:
        return dict(DEFAULT_PARAMS)

    def build(self, seed: int, params: Mapping[str, Any]) -> WorldDescription:
        merged = dict(DEFAULT_PARAMS)
        for k, v in params.items():
            if k not in merged:
                raise ValueError(
                    f"unknown param for recipe {self.name!r}: {k!r}; "
                    f"known: {sorted(merged)}"
                )
            merged[k] = v

        layout = _build_layout(merged)
        solved = solve(layout, seed=seed)

        props: list[PlacedProto] = []
        for placement in solved.props:
            props.append(PlacedProto(
                proto_url=get_url(placement.proto),
                proto_type=placement.proto,
                translation=placement.translation,
                rotation=placement.rotation,
            ))

        return WorldDescription(
            title="my_biome (omniworld)",
            floor_size=(float(merged["size"]), float(merged["size"])),
            props=props,
            spawns=list(solved.spawns),
            metadata={"recipe": self.name},
        )


register_recipe(MyBiome())
```

### Step 4 — register the biome

Add the import to [src/python/omniworld/biomes/\_\_init\_\_.py](../../src/python/omniworld/biomes/__init__.py):

```python
from . import my_biome  # noqa: F401
```

The `register_recipe(...)` call in your module runs at import time.

### Step 5 — write tests

Copy a shipped biome's test file as a template (e.g. [tests/python/omniworld/test_outdoor_forest.py](../../tests/python/omniworld/test_outdoor_forest.py)). The invariants every biome should test:

- Registration: `assert "my_biome" in list_recipes()`.
- Generation + validation: generate a world, check it exists, check validators pass.
- Determinism: two runs with the same seed produce byte-identical output.
- Seed variation: two runs with different seeds produce different output.
- Clearing / exclusion invariants: no prop violates the zones you declared exclusive.
- URDF spawn option: if your recipe has a `spawn_urdf` param, verify it emits `URDFRobot {`.
- Unknown param rejection: passing a bogus param raises `ValueError`.

Run just your tests:

```
python -m pytest tests/python/omniworld/test_my_biome.py -x -q
```

### Step 6 — hand-run the CLI

```
python scripts/dev/omniworld.py generate my_biome --seed 42 --out /tmp/my.wbt
python scripts/dev/omniworld.py validate /tmp/my.wbt
```

Load the `.wbt` in OmniSim to visually confirm. Fix anything that looks wrong, then:

```
python -m pytest tests/python/omniworld/ -q
```

## Recipe patterns in the shipped biomes

| Biome | Terrain | Scatter strategy | Notable pattern |
|---|---|---|---|
| `flat_ground` | none | none | stub; exercises the pipeline for its own sake |
| `outdoor_forest` | fbm | Poisson in a zone minus a road and a clearing | classic "zone + path + scatter" pattern |
| `outdoor_desert` | ridged + fbm blend | Poisson scatter of tinted, scaled Rocks | per-instance variation applied via `extra_fields` |
| `warehouse` | none | mixed: mechanical racks, stochastic boxes, nested-scatter top clutter | hand-placed grid + solver scatter + `scatter_on_surface` |
| `urban_block` | none | mixed: hand-placed Buildings, `along_path` street furniture | street lights + benches ride the solver's `along_path` scatter |
| `indoor_apartment` | none | mechanical: wall segments with door gaps, one furniture per room | walls derive from room rectangles; `_split_wall_around_door` is the reusable-candidate helper |

Common patterns worth copying:

- **`_rect(x0, y0, x1, y1)`** — every biome defines a tiny helper to build axis-aligned rectangular polygons. Worth pulling into `core.layout` eventually; today it's fine inline.
- **Deterministic sub-RNGs via `rng_from_seed(derive_seed(seed, "some.name"))`** — gives each sub-system its own reproducible stream that does not shift when you add another sub-system.
- **Hand-placed props outside the solver** — any structure that must look axis-aligned (racks, buildings, walls) is cheaper and clearer written directly than trying to coax the solver.

## Common pitfalls

- **Registering the biome twice.** If you import the module in both `biomes/__init__.py` and somewhere else, the `register_recipe` call fires twice; the second one rejects the duplicate. Import once.
- **Reading the global RNG.** Don't. Always take an explicit `rng` or derive one from the seed. Tests catch this because two runs of the same seed will diverge.
- **Forgetting EXTERNPROTO.** You don't have to emit EXTERNPROTO lines yourself — the WBT emitter de-duplicates them from `PlacedProto.proto_url` automatically. If a prop doesn't show up, check that its catalog entry has the right `url`.
- **Heightmap coordinates vs world coordinates.** A heightmap's `(0, 0)` is the grid corner, in grid cells. The recipe's world is in metres. When lifting 2D scatter onto a heightmap, use the `heightmap_spacing` parameter on `WorldDescription` to translate cleanly.
- **Zones without `exclusive=True` don't implicitly clear lower-priority zones.** Always mark spawn clearings exclusive, or the solver happily places trees on the robot.

## See also

- [omniworld-user-guide.md](omniworld-user-guide.md) — user-facing CLI + Python API reference.
- procedural-world-generation-plan.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) — long-range tier plan.
- [src/python/omniworld/catalog/assets.json](../../src/python/omniworld/catalog/assets.json) — the shipped PROTO catalog.
- [scripts/dev/build_asset_catalog.py](../../scripts/dev/build_asset_catalog.py) — catalog validator.
