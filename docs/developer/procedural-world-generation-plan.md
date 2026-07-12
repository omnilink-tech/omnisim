# OmniSim World-Class Procedural World Generation Plan

## Purpose

This document is the long-range plan for building a world-class procedural world generation system for OmniSim. The target is an engine that can produce, on demand, simulation-ready worlds across the biomes OmniSim cares about — outdoor terrain, urban blocks, warehouses, apartments, construction sites, agricultural fields — that are simultaneously **physically plausible**, **deterministically reproducible**, **validated for the task at hand**, and **authored from either a CLI, a Python library, or an OmniLink agent tool call**.

This plan began from a single one-off fBm-heightmap-plus-scatter seed script that emitted one outdoor `.wbt`. That seed has since been generalized into the shipped [`omniworld`](../../src/python/omniworld/) library (see the Tier 1 status block below). This plan is about turning that seed into a library, then a scenario engine, then a language- and learning-driven generator, without ever breaking existing hand-authored worlds or the sensor/determinism contracts the rest of the simulator depends on.

The plan is organized as three tiers. Each tier is a self-contained delivery with its own milestones, deliverables, validation, and exit criteria. A team or small group can stop at the end of any tier and ship a coherent generator — each tier is a real release, not a partial refactor.

This plan is paired with but does not replace:

- [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md) — textures, caches, world-authoring rules that any generator output must obey
- [simulation-authoring-for-coding-agents.md](simulation-authoring-for-coding-agents.md) — current best workflow for agent-driven world authoring, which this plan evolves
- [world-loading-and-template-performance.md](world-loading-and-template-performance.md) — load-path constraints any generated world must meet
- [engine-migration-plan.md](engine-migration-plan.md) — renderer roadmap; generated worlds must look good on both the WREN and OmniRender paths
- [benchmark-authoring.md](benchmark-authoring.md) — stability rules for generated benchmark worlds
- [physics-contact-and-collision-complexity.md](physics-contact-and-collision-complexity.md) — contact and collision budget any generator must respect

## Goals and Non-Goals

### Goals

1. **World-class physical and perceptual realism.** The generator is judged not by primitive count but by whether the worlds it produces are mistakable, at a glance, for photographs of real locations. Every subsystem below is in service of this goal.
2. **One library, many biomes.** A single Python library (`omniworld`) produces indoor, outdoor, urban, industrial, and agricultural worlds from a shared set of primitives.
3. **Determinism as a contract.** Given a seed and a manifest, a generator run always produces a byte-identical `.wbt`. Regression tests depend on this.
4. **Validation built in.** Every generator output is checked for spawn reachability, collision overlap, load-time budget, contact-pair budget, and a minimum visual-quality bar before it is declared a valid world.
5. **Agent-callable.** An OmniLink agent can call "make me a warehouse with three pallet racks, a robot spawn near the loading door, and two obstacles in the aisle" and get back a world path it can immediately step.
6. **Composable.** The same primitives (heightmap, scatter, zone, stamp, path, validator) compose into every biome. No biome ships its own bespoke engine.
7. **Real-world import.** Tier 2 brings OpenStreetMap, GeoTIFF DEMs, floor-plan rasters, and photoscanned assets into the same pipeline, so a user can generate a world from a real location, a scanned site, or a reference image set.
8. **Training-loop friendly.** Generators expose domain-randomization hooks so training runs can request "1000 variants of this scenario, seeds 0–999" and stream them into the simulator.

### Realism Pillars

Every tier in this plan is measured against these nine pillars. A feature that does not advance at least one pillar does not ship.

1. **Asset fidelity.** PBR-correct, photoscanned where possible. No placeholder primitives standing in for structure at authoring time.
2. **Gravity-settled placement.** No prop floats, clips into a wall, or rests on a single point. The physics solver is the final authority on where things sit.
3. **Weathering and aging.** No surface is pristine unless it has a reason to be. Edge wear, drip stains, oxidation, dust, scuffs are first-class outputs of the generator, not a texture artist's afterthought.
4. **Ecological coherence.** Vegetation matches slope, soil, water proximity, and aspect. Rocks match terrain lithology. Erosion carves realistic drainage. Competition thins stands.
5. **Human and machine traces.** Worn floor paths to the door, scuffs at hand height on frames, tire tracks at warehouse entries, coffee rings on desks. Use leaves evidence.
6. **Clutter realism.** Desks are not empty. Shelves have bric-a-brac. Floors have debris. Scattering is nested: a desk has a laptop, the laptop has cables, the cables tangle with pens.
7. **Environmental coherence.** Sun direction, shadow length, sky color, surface wetness, puddle placement, leaf color, and air fog are a single consistent world state, not independent knobs.
8. **Micro-variation.** No two placed instances of the same PROTO are visually identical. Small per-instance randomization on scale, hue shift, weathering grade, decal placement.
9. **Reference-driven distributions.** Statistical priors (building heights by region, clutter density in offices, tree species by climate) are drawn from real-world datasets, not intuition.

### Non-Goals

1. Replacing hand-authored reference worlds. The sample worlds under `projects/samples/` and `projects/robots/*/worlds/` are canonical and untouched.
2. A full 3D authoring UI. The plan ships CLI and Python library; any GUI is a Tier 3 optional deliverable.
3. Real-time streaming of infinite open worlds during interactive use — this shows up only in Tier 3 and is bounded.
4. Training our own neural generator from scratch. Tier 3 uses learned distributions but leans on existing pretrained models for heightmap/texture synthesis rather than training them end-to-end.
5. Procedural animation. Generators place props and robots; they do not choreograph them — that is the scenario controller's job.
6. Rewriting Webots' PROTO system. We build on top of the existing PROTO catalog under [projects/objects/](../../projects/objects/).

### Constraints

- Must run on the existing MSYS2 / MinGW64 / Python toolchain on Windows, plus Linux and macOS (see [quickstart.md](quickstart.md)).
- Must not require new native binaries. The generator is pure Python; any C++ extension must be optional and have a pure-Python fallback.
- Must not introduce remote-asset dependencies. Generated worlds reference PROTOs already in-repo (see [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md) Rule 1).
- Must preserve determinism for headless runs and sensor captures — two runs with the same seed must produce the same `.wbt` bytes and the same first-frame sensor output.
- Must land a working CLI before any agent/MCP integration — the CLI is the validation surface everything else checks against.

## Current Baseline

Before planning upgrades, this is what exists and must be preserved or evolved, not thrown away.

### Working procedural seed

- The original one-off seed script (fBm heightmap, gaussian flat-center, Poisson-disk rock and tree scatter, explicit `ElevationGrid` emission) has been superseded by the shipped [`omniworld`](../../src/python/omniworld/) library; its outdoor lineage now lives in the `outdoor_forest` recipe (see [distribution/generated_worlds/outdoor_forest.wbt](../../distribution/generated_worlds/outdoor_forest.wbt) for a generated output). This is the seed the Tier 1 library generalized.

### Adjacent tooling

- [scripts/packaging/generate_projects_files.py](../../scripts/packaging/generate_projects_files.py) — asset-cache generation, adjacent but not procedural.

### PROTO catalog

An extensive hand-authored library ready to be consumed by a generator, with zero new asset work needed for Tier 1:

- [projects/objects/buildings/](../../projects/objects/buildings/) — 30+ building PROTOs (houses, towers, warehouses, barns, churches, gas stations).
- [projects/objects/apartment_structure/](../../projects/objects/apartment_structure/) — walls, doors, windows, radiators — the primitives for interior generation.
- [projects/objects/trees/](../../projects/objects/trees/), [projects/objects/rocks/](../../projects/objects/rocks/), [projects/objects/animals/](../../projects/objects/animals/) — outdoor scatter.
- [projects/objects/factory/](../../projects/objects/factory/) — containers, conveyors, shelves for industrial biomes.
- [projects/objects/bathroom/](../../projects/objects/bathroom/), [projects/objects/bedroom/](../../projects/objects/bedroom/), [projects/objects/chairs/](../../projects/objects/chairs/), [projects/objects/computers/](../../projects/objects/computers/) — interior props.
- [projects/objects/traffic/](../../projects/objects/traffic/), [projects/objects/street_furniture/](../../projects/objects/street_furniture/) — urban biome.
- [projects/objects/backgrounds/](../../projects/objects/backgrounds/) — sky.

The generator's job is to compose these, not to author new ones. Any new PROTO work is explicitly scoped and called out.

### Robot import

- `URDFRobot` node — a built-in simulator node (registered by the OmniSim core, not a PROTO file), already consumed by the URDF demos. The generator uses this to spawn any URDF from [projects/samples/demos/robots/](../../projects/samples/demos/robots/) or user-supplied locations.

### Known gaps vs a world-class generator

1. **One algorithm, one biome.** fBm heightmap + Poisson scatter produces one look — outdoor rough terrain. No urban, no indoor, no industrial.
2. **No semantic layout.** The scatter cannot express "a road runs through here, trees grow along it, not across it." Layout is purely statistical.
3. **No interior generation.** There is no way to produce an apartment, warehouse, or factory interior. Every indoor demo is hand-authored.
4. **No validation pipeline.** Nothing checks whether the robot spawn is reachable, whether props overlap, whether the ElevationGrid has a mesh the physics engine can handle, or whether the total contact budget is sane.
5. **Not composable.** The single script mixes heightmap, scatter, PROTO emission, and WBT header in one file. Reusing the scatter for a different biome means copy-paste.
6. **No CLI, no API.** You run one script with hardcoded constants. You cannot parameterize it from an agent, a test, or CI.
7. **No determinism manifest.** The seed is a single int; there is no record of which library version, which PROTO catalog version, or which parameter set produced a given world.
8. **No performance budget.** Nothing tells the generator "this world must load in < 3 s on the mid tier" or "keep contact pairs under 2,000."
9. **No real-world ingress.** OSM works for vehicles only and is not plugged into the general generator story. GeoTIFF DEMs, indoor floor plans, and BIM imports do not exist.
10. **No scenario layer.** The generator outputs static worlds. There is no "make a navigation task with 5 goals and 3 obstacles that change every episode" layer. Training loops have to author this by hand.
11. **No domain randomization.** Nothing supports "give me 1000 variants of this scenario with the lighting, textures, and prop positions randomized within these bounds."
12. **No agent tool.** Despite OmniSim being pitched as the sim for OmniLink agents (see [main docs](../guide/index.md)), no agent can actually request a world.
13. **No learned component.** All generation is procedural-classical. No diffusion heightmaps, no learned room layouts, no language conditioning.
14. **No LOD / impostor story for dense scatter.** 48 trees work; 48,000 trees would melt WREN.
15. **Every prop floats or clips.** The scatter assumes 2D positions and a single terrain-height lookup. Rocks rest on one point of their bounding box; boxes stacked in a warehouse are placed into a parametric pile, not dropped.
16. **Every surface is pristine.** Materials ship at factory-new weathering. There is no mechanism to age a scene by 10 years, 50 years, or "abandoned."
17. **Every prop is identical to its siblings.** The same PROTO instance renders the same pixels at every placement. Real scenes have per-instance hue drift, scratch position, label wear.
18. **No ecology.** Trees scatter in an annulus. They do not cluster where soil is deep, thin where it is rocky, lean away from prevailing wind, or age along a Weibull distribution.
19. **No human traces.** The generator has no concept of "this is a high-traffic zone" or "this surface is at hand height". A busy warehouse floor looks as clean as an unused one.
20. **No time or weather.** Every world is noon on a clear day. There is no diurnal cycle, no weather state, no season, no wetness.
21. **Empty surfaces.** A generated office has desks. The desks have nothing on them. A generated kitchen has counters with no utensils.
22. **Statistics from intuition, not data.** Building heights, tree density per climate, clutter per desk are all hand-tuned constants. There is no pipeline pulling real-world distributions into the parameter space.
23. **Asset catalog is structural, not photoreal.** PROTOs under [projects/objects/](../../projects/objects/) are clean, hand-authored primitives. They do not match the fidelity of photoscanned libraries (Megascans, Polyhaven).
24. **No reference matching.** A user who wants a scene to look like a specific real place has no path beyond OSM geometry — no way to match the *look* of that place.
25. **No acoustic signal.** Scenarios that depend on sound (following a machine by hum, echo localization) have no generator support.

Tiers 1 through 3 close these gaps in order of perceptual and functional ROI per unit of engineering effort.

## Architectural Decision: Script Collection vs Library vs Engine

Before any tier begins, one decision determines the shape of the rest of the program.

**Option A — More scripts.** Keep adding one-off `generate_*.py` files next to each world. No library, no CLI, no validation.

- Pro: zero investment, works immediately for one-off demos.
- Pro: no new dependency, no new directory structure.
- Con: every new biome is from-scratch. Scatter logic gets copy-pasted seven times. There is no shared validation, no shared determinism contract, no agent story.
- Con: this is the status quo and it is already a maintenance problem with two scripts.

**Option B — Python library under `src/python/omniworld/` with a CLI, consumed by generator scripts and an MCP tool.**

- Pro: one place for heightmap, scatter, zone, validator, PROTO catalog. Every biome is composition.
- Pro: a CLI `omniworld generate <biome>` becomes the validation surface — hand tests, CI, and agents all call the same thing.
- Pro: explicit separation between the generator library (stable API) and the biome recipes (flexible, expected to grow).
- Pro: a Python MCP tool can expose the library to OmniLink agents in Tier 2 without extra work.
- Con: introduces a new Python package under the repo and a new location in [scripts/dev/](../../scripts/dev/) for the CLI entry point.
- Con: requires landing a real package layout — not a hurdle, just work.

**Option C — New C++ engine component (`src/omniworld/`) that talks to the rest of the simulator in-process.**

- Pro: fastest execution; could theoretically generate worlds at runtime without a `.wbt` round-trip.
- Con: build-time cost, a separate C++ subsystem to maintain, worse iteration loop, much harder for an agent to call.
- Con: premature — we do not yet need runtime-speed generation. Tier 3 revisits this only for streaming worlds.

**Recommendation: Option B, starting in Tier 1.** A Python library is the right cost/benefit for where we are. It matches how the original seed script worked, keeps iteration fast, and slots naturally into both the agent tool story and CI. Tier 3 can extract a C++ core for runtime streaming if and only if the Python generator demonstrably bottlenecks a training loop.

All following tiers assume Option B. Paths labeled `src/python/omniworld/` are new and do not yet exist.

## Cross-Tier Workstreams

Four workstreams run alongside every tier and are not tier-specific.

### Determinism and Seed Manifests

Every generated world must be reproducible. Specific cross-tier work:

- Every generator run emits a **seed manifest** alongside the `.wbt`: a JSON file with the seed, the library version, the PROTO catalog hash, the full parameter set, the biome recipe name, and a SHA256 of the output. Filename: `<world>.seed.json`.
- CI regression test: run each shipped recipe at a pinned seed, assert the SHA256 matches a checked-in reference. Location: `tests/generators/` under the existing test harness ([test-harness-and-scenario-architecture.md](test-harness-and-scenario-architecture.md)).
- Every primitive in the library takes an explicit `rng: random.Random` parameter. No primitive reads the global RNG. No primitive reads the wall clock.
- Any floating-point accumulation that could drift between platforms (e.g. summing over millions of triangles) uses the **Kahan summation** primitive in the library.

### Validation Harness

A generator output that crashes the simulator, overlaps props, or leaves the robot in an unreachable spawn is a failure, not a feature. Specific cross-tier work:

- `omniworld validate <world.wbt>` CLI command exits non-zero if any validator fails.
- Validators ship as independent Python checks; the CLI runs them all and reports per-check status.
- Tier 1 lands the five validators listed below. Tier 2 and Tier 3 add biome-specific ones.
- Every generator invocation runs `validate` by default before declaring success. `--skip-validate` exists as an escape hatch for debugging.

Tier 1 validator set:

1. **Spawn reachability** — the robot spawn is not inside a prop, not below the terrain, and has a 0.5 m navigable radius around it.
2. **Prop overlap** — no two placed props have overlapping bounding volumes (a small tolerance is allowed for deliberate stacking).
3. **Load-time budget** — the world loads in the headless simulator in under a recipe-defined budget (default 5 s on mid tier).
4. **Contact-pair budget** — initial contact pairs under a recipe-defined ceiling (default 2,000). See [physics-contact-and-collision-complexity.md](physics-contact-and-collision-complexity.md).
5. **Asset locality** — every asset referenced is local to the repo, not a remote URL. See [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md) Rule 1.

### Asset Catalog

The generator needs metadata the PROTO files do not carry today. Specific cross-tier work:

- A machine-readable catalog file (`src/python/omniworld/catalog/assets.json`) describes every PROTO the generator may place: bounding box, weight / sink depth hint, preferred rotation axis, biome affinity tags, footprint polygon, collision complexity, suggested scale range.
- The catalog is generated by a tool (`scripts/dev/build_asset_catalog.py`) that inspects [projects/objects/](../../projects/objects/) PROTO files; it is **not** hand-edited.
- Catalog changes are versioned and the hash is pinned in the seed manifest (so upgrading the catalog shows up as a diff in generated outputs under CI).
- Tier 1 covers all of `projects/objects/`. Tier 2 extends to imported OSM and BIM assets. Tier 3 extends to learned assets.

### Agent and CLI API

OmniSim is pitched as the sim for OmniLink agents. The generator must be agent-native from day one. Specific cross-tier work:

- CLI: `omniworld generate <recipe> --seed N --out path.wbt [--param k=v ...]`. Ships in Tier 1.
- Python API: `omniworld.generate(recipe, seed=N, params={...}) -> WorldHandle`. Ships in Tier 1.
- OmniLink agent tool: a `ToolRunner` tool that wraps the library, registered alongside the existing OmniLink bridge controllers under [projects/samples/demos/controllers/](../../projects/samples/demos/controllers/). Ships in Tier 2 once the recipes are stable.
- Schema-first: every recipe declares its parameter schema as a JSON Schema, so the CLI, the Python API, the agent tool, and the documentation are all generated from the same source.

### Realism and Plausibility

This workstream is the spine of the realism goal. It is not a single feature; it is a set of passes that run between the layout solver and the final `.wbt` emission, each of which turns a correct-but-sterile placement into a correct-and-believable one.

- **Physics settling.** After the layout solver finishes, every non-`DEF`-critical prop is handed to a headless physics sim for a short settle pass. Rocks rest on their actual contact polygon. Boxes in a stack lean. Chairs end up on four legs. Output translations and rotations are baked into the emitted world. Spawn points, scenario goals, and named anchors are skipped.
- **Weathering pass.** Every placed prop's material selection is perturbed by a weathering grade (0.0 pristine → 1.0 abandoned) sampled from the world-level age parameter and the per-zone use intensity. Drives base-color darkening, roughness increase at edges, dirt accumulation at ambient-occlusion cavities, oxidation on metal exposed to moisture, algae on north-facing shaded surfaces.
- **Per-instance variation.** Every placement gets a tiny per-instance seed. Derived variations: scale jitter (±5%), hue shift within a per-material tolerance, rotation micro-offset, weathering grade draw, decal/sticker placement, label rotation. No two instances of the same PROTO are pixel-identical.
- **Ecological coherence.** Vegetation scatter consults slope, aspect, soil, water proximity, and simulated competition. Rock types follow a lithology map derived alongside the heightmap.
- **Human-trace overlay.** A per-zone footfall field is computed from the layout graph (doors, counters, spawn, goals) and used to darken floor tiles, wear paint, scuff baseboards, and place scattered detritus along paths.
- **Environmental state.** Every world carries a `WorldState`: time of day, date (for sun azimuth), weather (sunny/overcast/rain/snow/fog), wind, season. Downstream passes consume this: wetness shader gets puddle masks, snow accumulation runs on up-facing normals, leaf color follows season.
- **Reference calibration.** Parameter priors (building heights, clutter density, tree species weights) load from versioned real-world statistics tables in `src/python/omniworld/data/priors/`, not from hand-tuned code constants.

Each pass is a separate module under `src/python/omniworld/realism/`. Each is individually toggleable. Each is individually regression-tested. The tiers below schedule when each pass lands and at what fidelity.

---

## Tier 1 — Composable Generator Library with Biomes and Validation

> **Status: substantially shipped.** The `omniworld` library lives at
> [`src/python/omniworld/`](../../src/python/omniworld/) with the full
> module layout (`core/`, `primitives/`, `biomes/`, `catalog/`,
> `emit/`, `realism/`, `validation/`, `cli.py`). Seven biome recipes
> ship: `flat_ground`, `outdoor_forest`, `outdoor_desert`, `urban_block`,
> `indoor_apartment`, `warehouse`, `mars`. The realism layer
> (`ground_fit`, `variation`, `weathering`) and validation suite
> (`spawn`, `overlap`, `headless`, `asset_locality`) are in place.
> **For current usage, read the [omniworld user guide](omniworld-user-guide.md)
> and [biome cookbook](omniworld-biome-cookbook.md), not the design
> sections below** — those are kept as historical context for the
> architectural decisions, not as a roadmap.
>
> The detailed T1.1–T1.11 sections that follow are preserved as the
> design record (decisions on layout, dependency footprint, exit
> criteria) and may not match the shipped implementation in every
> detail. Treat them as background, not a checklist.

**Original tier goal.** Replace the one-off script with a Python library that can generate five biomes from reproducible recipes, validated end to end, driven by a CLI. Every Tier 1 recipe exports a world any existing OmniSim controller can load. No new renderer features required. No new required PROTOs.

**Original tier exit criteria.**

1. Five recipes ship and produce valid, varied worlds from any seed: `outdoor_forest`, `outdoor_desert`, `urban_block`, `indoor_apartment`, `warehouse`.
2. Every shipped recipe has a pinned CI regression at seeds 0, 1, 42.
3. `omniworld validate` runs on every generated world in CI and passes.
4. Generated worlds load in under the per-biome budget on the mid tier; no new contact-pair regressions on [the existing benchmark worlds](benchmark-authoring.md).
5. A blind comparison panel rates the outdoor recipes at parity or better with the original one-off generator script.
6. The original one-off script is either retired or becomes a thin wrapper around `omniworld.generate(...)`. Either way the script's output stays runnable.

### T1.1 — Library scaffold and CLI

**Why first.** Every other Tier 1 deliverable ships inside this library. Until it exists with a working CLI we cannot land any recipe.

**Scope.**

- Create `src/python/omniworld/` with the initial module layout: `core/`, `primitives/`, `biomes/`, `catalog/`, `emit/`, `validate/`, `cli.py`.
- Minimal dependency footprint: standard library only for Tier 1. `numpy` is allowed but must be optional — fallback code paths without `numpy`.
- CLI entry point registered in [scripts/dev/](../../scripts/dev/) so `omniworld` resolves the same way the other dev scripts do.
- Seed manifest emission (JSON alongside `.wbt`) as described in the cross-tier section.
- Package version pinned in `src/python/omniworld/_version.py`, surfaced in the manifest.
- Shim the original seed script's behavior as a temporary recipe so one known-good output survives from day one.

**Files touched / created.**

- new: `src/python/omniworld/` (entire subtree)
- new: `scripts/dev/omniworld.py` (CLI entry)
- new: `docs/developer/omniworld-user-guide.md` (referenced from this plan, written in T1.8)
- modify: `docs/developer/README.md` to link the new user guide

**Milestones.**

1. **M1.1.a** — Empty library imports, `omniworld --version` prints.
2. **M1.1.b** — `omniworld generate outdoor_forest --seed 7 --out /tmp/w.wbt` produces a file byte-identical to the original seed script's output at seed 7.
3. **M1.1.c** — Seed manifest emitted; CI regression gate active.

**Deliverables.** Library skeleton, CLI, shim recipe, manifest, CI gate.

**Risks.** Python path / import story on Windows. Mitigate by publishing the library as an editable install with a launcher script that pins `sys.path`.

### T1.2 — Heightmap primitives

**Why second.** Outdoor biomes all need heightmaps. Generalize the fBm from the original seed script into a composable set.

**Scope.**

- `primitives/heightmap.py` with:
  - `fbm(n, seed, octaves=5, lacunarity=2.0, gain=0.5, base_period=None)` — multi-octave value noise, port of the existing implementation.
  - `ridged(n, seed, ...)` — ridged multifractal for sharp desert/canyon features.
  - `worley(n, seed, ...)` — cellular noise for desert dune separation and rocky fields.
  - `stamp(base, pattern, x, y, mode)` — additive / subtractive / max / min stamping for craters, pits, plateaus.
  - `mask_radial(shape, sigma)`, `mask_rect(shape, bounds)`, `mask_polygon(shape, polygon)` — zone masks.
  - `blend(a, b, mask)`, `apply_mask(h, mask, strength)`.
  - `erode_hydraulic(h, steps)` and `erode_thermal(h, steps)` — cheap Tier 1 erosion, single-channel, no sediment transport. Full erosion is a Tier 2 item.
- `emit/elevation_grid.py` — consolidates the WBT ElevationGrid emission from the shim recipe.
- Bilinear sampling primitive `sample(h, x, y)` reused across all scatter and validation.

**Files touched / created.**

- new: `src/python/omniworld/primitives/heightmap.py`
- new: `src/python/omniworld/emit/elevation_grid.py`
- new: `tests/python/omniworld/test_heightmap.py`

**Milestones.**

1. **M1.2.a** — fBm byte-reproducible across platforms (CI matrix: Win, Linux, macOS).
2. **M1.2.b** — Stamp and mask round-trip tests pass.
3. **M1.2.c** — `erode_hydraulic` runs in < 500 ms on a 256×256 grid on the mid tier.
4. **M1.2.d** — ElevationGrid emitter parity check against current shim recipe output.

**Deliverables.** Heightmap library, ElevationGrid emitter, tests.

**Risks.** `math.sin` and `math.cos` small-scale differences across platforms. Mitigate by restricting the determinism contract to bitwise identity on a single (platform, Python version) pair; cross-platform identity is a best-effort goal, not a gate.

### T1.3 — Scatter primitives

**Why.** Every biome, outdoor or indoor, needs to place many objects without them overlapping. Generalize the Poisson-disk scatter.

**Scope.**

- `primitives/scatter.py` with:
  - `poisson_annulus(rng, count, r_min, r_max, min_dist, existing=None)` — the existing scatter, lifted verbatim.
  - `poisson_polygon(rng, count, polygon, min_dist)` — scatter inside an arbitrary 2D polygon.
  - `grid_jittered(rng, polygon, cell_size, jitter)` — for orchard-like or rack-like layouts.
  - `clustered(rng, centers, count_per, sigma, min_dist)` — tree stands, rock piles.
  - `along_path(rng, path, density, jitter, side)` — for roadside trees or warehouse aisles.
  - `on_surface(rng, count, mesh_or_heightmap, ...)` — projects scatter onto a surface.
- Weighted asset selection: `pick(rng, weights)`, `pick_from_biome(rng, biome_tag, catalog)`.
- Collision rejection against an existing asset list uses the **asset-catalog bounding boxes**, not the 2D proximity disk — so a wide-canopy tree does not embed into a narrow sapling even though their centers are 1 m apart.

**Files touched / created.**

- new: `src/python/omniworld/primitives/scatter.py`
- new: `tests/python/omniworld/test_scatter.py`

**Milestones.**

1. **M1.3.a** — `poisson_polygon` produces valid, non-overlapping placements on pathological polygon shapes (narrow, concave).
2. **M1.3.b** — `clustered` produces visibly clumped distributions — clumpiness metric matches target.
3. **M1.3.c** — Catalog-aware collision rejection eliminates a class of overlap bugs visible today in outdoor generation.

**Deliverables.** Scatter library, catalog integration, tests.

**Risks.** Time complexity of general-polygon Poisson scatter. Mitigate with a spatial-hash-backed rejection sampler.

### T1.4 — Layout DSL

**Why.** Beyond flat scatter, biomes need to express **zones**: "this rectangle is the spawn clearing", "this strip is the road", "this polygon is the building footprint". Tier 1 introduces a small layout DSL — zones, paths, stamps, props — that every biome speaks.

**Scope.**

- `core/layout.py` defines:
  - `Zone` — named 2D polygon (or axis-aligned rect or disk) with attached tags and parameters.
  - `Path` — named polyline with width, tags, and parameters.
  - `Stamp` — heightmap modifier anchored to a zone.
  - `PropGroup` — a named scatter operation constrained by zones.
  - `Spawn` — a robot spawn declaration with attached URDF, controller, and orientation.
- A `Layout` is an ordered list of these declarations. The biome recipe builds a Layout, which is then baked into a heightmap + prop list + spawn list by the **solver** (`core/solver.py`), which honors zone constraints (e.g. road clears trees, spawn zone clears everything).
- YAML / JSON serialization so layouts are inspectable, diffable, and agent-editable.

**Files touched / created.**

- new: `src/python/omniworld/core/layout.py`
- new: `src/python/omniworld/core/solver.py`
- new: `src/python/omniworld/core/schema.py`
- new: `tests/python/omniworld/test_layout.py`

**Milestones.**

1. **M1.4.a** — A hand-coded Layout for the shim recipe produces the same `.wbt` as T1.1.
2. **M1.4.b** — Zone conflicts resolve in declared priority order (road clears trees, spawn clears everything).
3. **M1.4.c** — Layout round-trips through JSON without loss.

**Deliverables.** Layout types, solver, schema, tests.

**Risks.** Solver complexity growing into a geometry engine. Keep Tier 1 solver intentionally naive — O(N·M) zone-vs-prop checks, spatial hash backing, no general CSG. Upgrade only if a biome demands it.

### T1.5 — Asset catalog and tagged metadata

**Why.** The scatter and layout primitives need per-PROTO data the PROTOs do not expose: footprint, collision tier, biome tags, preferred rotation axis.

**Scope.**

- `catalog/build.py` — offline tool that scans [projects/objects/](../../projects/objects/) PROTOs and emits `catalog/assets.json`. Metadata is derived where possible (bounding box from the PROTO's geometry fields) and hand-annotated where required (biome tags, "do not scatter me upside down").
- `catalog/assets.json` — the single source of truth for the generator. Versioned; hash included in the seed manifest.
- `catalog/overrides/` — per-asset YAML files holding metadata that cannot be derived. Flat key-value, trivially diffable.
- `catalog/query.py` — API: `catalog.filter(biome="forest", max_collision_tier=2, min_height=1.0)`.
- A scan pass that flags PROTOs with ambiguous or missing metadata; CI fails a PR that adds a new PROTO under `projects/objects/` without a catalog update (or an explicit opt-out).

**Files touched / created.**

- new: `src/python/omniworld/catalog/`
- new: `scripts/dev/build_asset_catalog.py`
- new: `tests/python/omniworld/test_catalog.py`
- modify: PR check under [ci-and-fast-feedback.md](ci-and-fast-feedback.md) to run the catalog coverage gate

**Milestones.**

1. **M1.5.a** — Catalog covers all forest/outdoor PROTOs needed by the shim recipe.
2. **M1.5.b** — Catalog covers every PROTO under [projects/objects/](../../projects/objects/), with overrides for anything not derivable.
3. **M1.5.c** — Coverage gate lands in CI.

**Deliverables.** Catalog builder, data files, query API, CI gate.

**Risks.** PROTO parsing complexity — Webots PROTOs can be template-expanded. Mitigate by parsing only the fields we need (bounding, fields) without a full PROTO interpreter. Fall back to overrides for anything too complex.

### T1.6 — Biome recipes (five shipped)

**Why.** Five biomes force the library's APIs to be generic. A single biome would tempt the library to be overfit.

**Scope.** Each recipe is a Python module in `biomes/<name>.py` that returns a Layout. Each ships a JSON Schema for its parameters.

- **`outdoor_forest`** — existing fBm-plus-scatter, but now composed out of the Tier 1 primitives. Adds path-following for a dirt road, variable tree density by altitude, grass patches.
- **`outdoor_desert`** — ridged + Worley heightmap, sparse rock scatter, no trees, hot sky preset.
- **`urban_block`** — axis-aligned road grid, building lots filled from [projects/objects/buildings/](../../projects/objects/buildings/), sidewalks from [projects/objects/street_furniture/](../../projects/objects/street_furniture/), street lights, parked vehicles.
- **`indoor_apartment`** — room-adjacency graph solved into an axis-aligned floor plan, walls/doors/windows from [projects/objects/apartment_structure/](../../projects/objects/apartment_structure/), furniture from [projects/objects/bedroom/](../../projects/objects/bedroom/), [projects/objects/bathroom/](../../projects/objects/bathroom/), [projects/objects/chairs/](../../projects/objects/chairs/).
- **`warehouse`** — axis-aligned aisles, pallet racks built from [projects/objects/factory/](../../projects/objects/factory/) PROTOs, loading dock, floor markings, stacked cardboard boxes.

Each recipe:
- Takes 8–15 tunable parameters.
- Renders a "signature" demo world at seed 42 that ships into [projects/samples/demos/worlds/](../../projects/samples/demos/worlds/).
- Has a CI regression at seeds 0, 1, 42.
- Comes with a README describing its parameters and showing reference screenshots.

**Files touched / created.**

- new: `src/python/omniworld/biomes/outdoor_forest.py`, `outdoor_desert.py`, `urban_block.py`, `indoor_apartment.py`, `warehouse.py`
- new: `src/python/omniworld/biomes/_schemas/` (one JSON Schema per biome)
- new: `projects/samples/demos/worlds/generated_*.wbt` (one signature demo per biome, generated, checked in)
- modify: the demo catalog ([DEMOS.md](../../DEMOS.md)) to link the new demos

**Milestones.**

1. **M1.6.a** — `outdoor_forest` via library is parity with the shim.
2. **M1.6.b** — `outdoor_desert` ships; renderer tolerates the new height profile.
3. **M1.6.c** — `urban_block` ships; contact-pair budget respected.
4. **M1.6.d** — `indoor_apartment` ships; 3-room minimum, load time budget respected.
5. **M1.6.e** — `warehouse` ships; 4-aisle layout with pallet racks.

**Deliverables.** Five recipes, five signature demos, five JSON Schemas, five READMEs.

**Risks.** Warehouse and urban contact pair counts blowing the physics budget. Mitigate by tiering PROTO collision geometry and using `boundingObject` simplifications on scattered props.

### T1.7 — Validation harness

**Why.** Tier 1 is only credible if every recipe is verified good before it leaves the generator.

**Scope.**

- Implement the five Tier 1 validators listed in the cross-tier section.
- The CLI runs them in-process (fast) for layout and spawn checks, and launches a headless simulator for load-time, contact-pair, and asset-locality checks.
- Headless mode uses the `omnisim --batch --mode=fast --no-rendering` invocation documented in [validation-playbook.md](validation-playbook.md), wrapped so the validator parses the output and extracts metrics.
- A single command `omniworld validate <world.wbt>` is all a user or CI needs.

**Files touched / created.**

- new: `src/python/omniworld/validate/`
- new: `src/python/omniworld/validate/runner.py` (headless sim wrapper)
- new: `tests/python/omniworld/test_validate.py`

**Milestones.**

1. **M1.7.a** — Spawn, overlap, asset-locality checks run in-process on the Tier 1 recipes.
2. **M1.7.b** — Load-time check runs under the headless simulator on Windows + Linux CI.
3. **M1.7.c** — Contact-pair check reads metrics from headless sim output, compares to recipe budget.
4. **M1.7.d** — Every Tier 1 recipe passes at seeds 0, 1, 42.

**Deliverables.** Validator set, headless runner, CI integration.

**Risks.** Headless sim crash / hang in CI. Mitigate by the same watchdog pattern used in [test-harness-and-scenario-architecture.md](test-harness-and-scenario-architecture.md); treat a hang as a validation failure with a retriable tag.

### T1.8 — Documentation and reference

**Scope.**

- Write `docs/developer/omniworld-user-guide.md`: CLI usage, library API, every biome's parameters, how to author a new biome.
- Write `docs/developer/omniworld-biome-cookbook.md`: walkthroughs of recipes, annotated, so a contributor can author a new one.
- Write a "What to call from an agent" section in [simulation-authoring-for-coding-agents.md](simulation-authoring-for-coding-agents.md) cross-linking to the Python API.
- Screenshots of each biome at representative seeds.

**Files touched / created.**

- new: `docs/developer/omniworld-user-guide.md`
- new: `docs/developer/omniworld-biome-cookbook.md`
- modify: [docs/developer/simulation-authoring-for-coding-agents.md](simulation-authoring-for-coding-agents.md)
- modify: [docs/developer/README.md](README.md)

**Deliverables.** User guide, cookbook, cross-links, screenshots.

### T1.9 — Physics-settled placement

**Why.** The single biggest perceptual lie in the original seed script is that rocks rest on a single terrain-height sample. A moderately sloped rock looks correct from above and floats or clips the moment the camera lowers. Physics-settling at bake time erases the entire class.

**Scope.**

- After layout solve and scatter, every prop tagged `settle: true` in the catalog is handed to a short headless physics run (50–100 steps) in a throwaway world that contains only the terrain and that prop. The final transform is baked back into the emitted `.wbt`.
- Batch settle: props in a connected region (a box stack, a rock pile) settle together so they rest against each other, not individually.
- `DEF`-marked critical placements — spawns, goals, named anchors, scenario waypoints — are excluded from settle. Their positions are authored, not solved.
- A conservative "sinking" budget: any prop that settles more than a recipe-defined drop is flagged — catches buried-in-terrain bugs.
- Settle output is cached by prop-hash + terrain-hash. A regenerated world with unchanged terrain reuses settled placements.
- Settle integrates with the existing contact-pair validator — if settling blows the contact budget, the run fails.

**Files touched / created.**

- new: `src/python/omniworld/realism/settle.py`
- new: `src/python/omniworld/realism/settle_cache/` (content-hashed)
- modify: every Tier 1 biome recipe's emission path to route through settle
- modify: [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md) catalog schema to include the `settle` flag

**Milestones.**

1. **M1.9.a** — Single rock on an uneven terrain settles and the rest position is visibly correct vs the naive height-lookup.
2. **M1.9.b** — A 20-box pallet-stack settles as a stack, not as 20 independent boxes.
3. **M1.9.c** — Settle cache hit rate > 80% on a regeneration of the same recipe with a changed seed-independent parameter.
4. **M1.9.d** — Every shipped Tier 1 biome passes settle without blowing the contact budget.

**Deliverables.** Settle pass, cache, per-biome integration, flagged-sinking metric.

**Risks.** Headless-sim cost at bake time. Mitigate with aggressive caching and by settling in a trimmed mini-world that contains only the local terrain patch and the local props, not the whole scene.

### T1.10 — Weathering pass and per-instance micro-variation

**Why.** The second biggest perceptual lie is that every generated instance of a PROTO is pixel-identical to its siblings and ships at factory-new weathering. A forest of 48 identical oaks reads as a rendering, not as a forest. Ten identical rusting barrels read as a stage set.

**Scope.**

- `realism/weathering.py` computes per-prop weathering parameters:
  - Base weathering grade from world `age` parameter (0.0 new → 1.0 abandoned).
  - Zone modifiers: outdoor exposure, moisture, heavy use.
  - Per-instance jitter within a catalog-defined range.
- Weathering grade drives a single 4-component material perturbation emitted alongside the prop: `[dirt, wear, moisture, oxidation]`. The extended-BRDF work in the renderer ([engine-migration-plan.md](engine-migration-plan.md) T3.1) consumes these values; materials without that support fall back to a cheap base-color multiplier.
- `realism/variation.py` produces per-instance jitter that every emission path applies:
  - Scale jitter within a catalog-defined tolerance (default ±3% indoor, ±8% outdoor natural props).
  - Hue shift in HSL within a tight tolerance (kills the "identical tree" effect).
  - Small rotation offset on the non-dominant axis.
  - Decal/sticker picking from a per-material pool (crates get different shipping labels).
- Variation is **seeded per-instance**, not globally — two instances in the same world, same seed, still differ.
- Catalog extension: every PROTO declares allowed variation ranges; a clean chrome part has a much tighter hue range than a wooden crate.

**Files touched / created.**

- new: `src/python/omniworld/realism/weathering.py`
- new: `src/python/omniworld/realism/variation.py`
- modify: `src/python/omniworld/catalog/assets.json` schema — add `variation` and `weathering` blocks
- new: `resources/omniworld/textures/decal_pool/` — curated per-material decal library for Tier 1
- modify: emit path to thread variation through every instantiated PROTO
- collaboration: renderer PBR shader consumes the 4-component weathering vector (defined in [engine-migration-plan.md](engine-migration-plan.md))

**Milestones.**

1. **M1.10.a** — `age=0` produces the same output as today; `age=0.5` visibly ages the outdoor forest recipe.
2. **M1.10.b** — In a stand of 48 trees, no two are pixel-identical; the spread is visually convincing.
3. **M1.10.c** — Catalog coverage: every PROTO used by Tier 1 biomes has variation and weathering ranges declared.
4. **M1.10.d** — Weathering parameter byte-reproducible under the seed manifest.

**Deliverables.** Weathering + variation modules, catalog schema extension, decal pool, per-biome integration.

**Risks.** Material proliferation when every instance gets its own hue shift. Mitigate by encoding variation as a 4-byte per-instance uniform, not a new material — one material, many instances, each with its own uniform block.

### T1.11 — Micro-clutter and nested scatter

**Why.** Empty desks and empty shelves are the third perceptual lie. Real surfaces have things on them. Tier 1 lands the primitive that makes nested clutter possible, with a curated small-prop library and per-biome clutter profiles.

**Scope.**

- `primitives/surface_manifest.py` — every catalog entry can declare named flat regions on its surface: "desk.top", "shelf.tier_1", "crate.lid". The scatter primitives can target these regions by name.
- `primitives/nested_scatter.py` — scatter-on-surface that accepts a placed parent prop and a surface-manifest key, and places children on the declared region. Respects the parent's world transform (so a desk tilted by settle gets clutter that also tilts).
- `biomes/_clutter_profiles/` — per-room-type clutter definitions:
  - `office_desk` — laptop, papers, mug, monitor, keyboard, pens, cables.
  - `warehouse_floor` — cardboard scraps, banding, pallet debris, spilled packaging.
  - `kitchen_counter` — utensils, dishware, towel, spice jars.
  - `bathroom_counter` — toiletries.
  - `workbench` — tools, hardware scraps, wire coils.
- New small-prop PROTOs added only where the catalog genuinely lacks them. Before authoring any new PROTO we exhaust [projects/objects/](../../projects/objects/) and only then create. Every new PROTO is small, collision-cheap, and follows the existing PROTO conventions.
- Micro-clutter is wired through settle (T1.9) so clutter rests on the parent's settled transform, not on the idealized authoring transform.

**Files touched / created.**

- new: `src/python/omniworld/primitives/surface_manifest.py`
- new: `src/python/omniworld/primitives/nested_scatter.py`
- new: `src/python/omniworld/biomes/_clutter_profiles/` (one file per profile)
- new PROTOs: only what is unavoidable; each listed in the PR that introduces it with justification.
- modify: `indoor_apartment`, `indoor_office` (pulled earlier from Tier 2 if bandwidth allows, otherwise Tier 2 unchanged), `warehouse` recipes to consume clutter profiles

**Milestones.**

1. **M1.11.a** — Surface manifest declared for every desk/shelf/crate PROTO used by Tier 1.
2. **M1.11.b** — A generated office-desk scene has believable clutter at varied seeds.
3. **M1.11.c** — Clutter respects settle output: clutter on a tipped crate tips with the crate.
4. **M1.11.d** — Contact-pair budget still holds after clutter is added.

**Deliverables.** Surface-manifest schema, nested-scatter primitive, clutter profiles, updated biomes.

**Risks.** Contact-pair explosion from a hundred tiny clutter items per desk. Mitigate by shipping clutter items with coarse `boundingObject` simplifications (one box per clutter item, not the visual mesh).

### Tier 1 combined deliverables

At Tier 1 end the simulator ships with:

1. `omniworld` Python library under `src/python/omniworld/` with a stable public API.
2. `omniworld` CLI: generate, validate, and inspect recipes.
3. Five biome recipes — `outdoor_forest`, `outdoor_desert`, `urban_block`, `indoor_apartment`, `warehouse` — each schema-driven, documented, and CI-pinned.
4. Asset catalog covering every PROTO under [projects/objects/](../../projects/objects/), with a CI coverage gate on new PROTOs and a schema that includes variation, weathering, settle, and surface-manifest metadata.
5. Five-validator harness with headless-simulator integration and pinned budgets per recipe.
6. Seed manifest emitted with every world, byte-identical regression gates in CI.
7. One signature demo world per biome under [projects/samples/demos/worlds/](../../projects/samples/demos/worlds/), checked in as proof the pipeline is stable.
8. **Physics settling at bake time** — no prop floats or clips, box stacks lean correctly, rocks nestle.
9. **Weathering pass + per-instance variation** — world-level age parameter, per-instance hue/scale/decal jitter so no two PROTO instances are identical.
10. **Nested-scatter micro-clutter** — desks, shelves, counters, and workbenches are populated from curated clutter profiles that respect settled transforms.
11. User guide, cookbook, and screenshots at representative seeds and ages (new, 10y, 50y).

Practically this closes the gap from "we have one generator script" to "OmniSim has a procedural world engine that produces plausible, weathered, lived-in scenes any contributor, test, or agent can drive." Architecturally it leaves us with a pure-Python pipeline, a hand-authored catalog augmented with realism metadata, and three distinct perceptual levers (age, clutter density, variation strength) — the ceiling Tier 2 breaks through by admitting real-world geometry, real statistical priors, scenario-level authoring, and environmental state.

---

## Tier 2 — Real-World Import, Semantic Scenarios, and Agent Authoring

**Tier goal.** Break the "hand-authored recipe" ceiling. Accept inputs from the real world (OSM, DEMs, floor plans) on one end and natural-language-driven agent calls on the other. Layer a scenario engine on top of worlds so training loops and eval benchmarks can ask for tasks, not just scenes. Integrate the full generator into OmniLink as a first-class agent tool.

**Tier duration estimate.** 6–9 months full-time, or about a year split. The OSM unification and scenario engine are the long-pole workstreams.

**Tier exit criteria.**

1. An OSM export of any ~1 km² area of the real world produces a runnable `urban_block` world with street topology, buildings, and traffic furniture placed correctly.
2. A GeoTIFF DEM (SRTM or user-supplied) is ingested into `outdoor_forest` / `outdoor_desert` with the real terrain profile and matching satellite-derived tree cover (when the user supplies a cover raster).
3. The `indoor_apartment` recipe can be seeded by a raster floor-plan image and reproduce the room layout.
4. A scenario layer ships: every world can have a task definition (goals, obstacles, adversaries, success criteria) that the sim supervisor reads and enforces.
5. An OmniLink agent can call `generate_world(biome="warehouse", prompt="a messy warehouse with 3 knocked-over pallets near the loading door")` and receive a validated, runnable world.
6. A domain-randomization API produces 1,000 seeded variants of any recipe + scenario for training loops, streaming results into the simulator at > 1 world/s on the mid tier.

### T2.1 — Advanced heightmap primitives

**Why.** Tier 1 shipped cheap single-channel erosion. Real terrain needs sediment transport, real river networks, and multi-material layering.

**Scope.**

- Hydraulic erosion with sediment transport (Musgrave / Hans Beyer schemes), multiple passes, produces both a height and a sediment channel.
- Thermal erosion with talus angles tuned per biome.
- River network extraction: flow accumulation → stream ordering → river polyline. Drives a road-and-bridge subsystem indirectly.
- Layered material output: alongside the heightmap we also emit a **terrain material index** per cell, used by the forward shader to blend rock / soil / sand / grass.

**Files touched / created.**

- new: `src/python/omniworld/primitives/erosion.py`
- new: `src/python/omniworld/primitives/flow.py`
- new: `src/python/omniworld/emit/splatmap.py`
- modify: `biomes/outdoor_forest.py`, `biomes/outdoor_desert.py` to consume the new primitives

**Milestones.**

1. **M2.1.a** — Hydraulic erosion produces visually convincing erosion networks; side-by-side comparison with reference images.
2. **M2.1.b** — River polylines extract and close on themselves correctly.
3. **M2.1.c** — Splatmap drives a material blend on the terrain shader (requires collaboration with [engine-migration-plan.md](engine-migration-plan.md) T3.1 extended BRDF).

**Deliverables.** Erosion / flow primitives, splatmap emission, updated biomes.

**Risks.** Erosion compute cost at 1024×1024. Mitigate with optional `numpy` acceleration and a coarser default grid.

### T2.2 — OpenStreetMap and DEM ingress

**Why.** The repo already has standalone OSM tooling — e.g. the nav-map builder at [projects/omni_quest/tools/build_osm_map.py](../../projects/omni_quest/tools/build_osm_map.py), which pulls a real walkable network and emits a routable graph. It is siloed. Unify this OSM ingress under `omniworld` so urban and outdoor biomes can accept real geography.

**Scope.**

- `ingress/osm.py` — parse a `.osm` XML or PBF file, extract: road graph (with tags), building footprints (with heights where available), water bodies, parks, traffic features. Feeds a layout that the existing `urban_block` recipe consumes.
- `ingress/dem.py` — read GeoTIFF DEM (`rasterio` optional; else a minimal GeoTIFF parser), resample to the heightmap grid, reproject to local ENU coordinates.
- `ingress/raster.py` — read land-cover rasters (forest, water, built-up) and drive biome-specific scatter masks.
- Coordinate system: pick ENU with user-selected origin (lat/lon) and store it in the seed manifest so two runs of the same area produce byte-identical output.

**Files touched / created.**

- new: `src/python/omniworld/ingress/osm.py`
- new: `src/python/omniworld/ingress/dem.py`
- new: `src/python/omniworld/ingress/raster.py`
- modify: automobile OSM importer to call the unified library; behavior preserved
- new: `biomes/real_world_block.py` composed from ingress + urban_block primitives

**Milestones.**

1. **M2.2.a** — OSM parse of a known small area (a university campus or similar) produces a road graph matching the source.
2. **M2.2.b** — Building footprints render as boxes at real-world heights; no misplacement vs OSM.
3. **M2.2.c** — DEM ingress on an SRTM tile produces a matching heightmap at 30 m resolution.
4. **M2.2.d** — Land-cover raster drives forest scatter density.
5. **M2.2.e** — Automobile OSM importer regression test unchanged.

**Deliverables.** OSM, DEM, raster ingress modules; `real_world_block` recipe; preserved automobile flow.

**Risks.** Projection edge cases near poles / date line — out of scope; error out cleanly on those.

### T2.3 — Indoor floor plans and wave-function collapse

**Why.** Tier 1 shipped a basic `indoor_apartment`. Real productivity requires reproducing a specific floor plan and generating larger layouts that feel authored rather than random.

**Scope.**

- Wave Function Collapse (WFC) over a tile set describing room adjacencies: bedroom-next-to-bathroom, kitchen-next-to-dining, hall-connects-N-rooms. Produces a larger apartment / office layout than Tier 1 could.
- Floor plan raster ingress: take a user-supplied image (black = wall, white = floor, colored = room type), vectorize into a floor plan, feed into the existing `indoor_apartment` solver.
- Door and window placement: semantic rules (bathroom has one door, apartments have a front door on the exterior wall, windows only on exterior walls).
- Furniture placement: per-room-type scatter rules consumed from the asset catalog's biome tags.
- New recipe: `indoor_office` — open-plan cubicles, conference rooms, kitchenette — composed entirely out of Tier 1 primitives plus WFC.

**Files touched / created.**

- new: `src/python/omniworld/primitives/wfc.py`
- new: `src/python/omniworld/ingress/floorplan.py`
- new: `biomes/indoor_office.py`
- modify: `biomes/indoor_apartment.py` to optionally accept a raster input

**Milestones.**

1. **M2.3.a** — WFC produces valid, connected floor plans; contradiction rate < 1% over 10k seeds.
2. **M2.3.b** — Raster ingress reconstructs a hand-drawn reference floor plan faithfully.
3. **M2.3.c** — `indoor_office` ships.

**Deliverables.** WFC primitive, raster ingress, extended apartment, office recipe.

**Risks.** WFC contradiction storms. Mitigate with a fallback greedy solver and a cap on backtracking.

### T2.4 — Scenario engine

**Why.** A world without a task is a diorama. Training and evaluation need worlds that **do something**: robot starts here, goal is there, dynamic obstacle crosses at t=5 s, success when the robot reaches the goal without collision.

**Scope.**

- `scenario/` subpackage describing tasks: `Start`, `Goal`, `Waypoint`, `DynamicObstacle`, `Trigger`, `Timer`, `SuccessCondition`, `FailureCondition`.
- A scenario is authored as a YAML / JSON file paired with a world. The sim-side supervisor — a new supervisor controller under [projects/samples/demos/controllers/](../../projects/samples/demos/controllers/) — reads the scenario and enforces it.
- Predefined scenario templates per biome: `warehouse.pick_place`, `outdoor_forest.navigate_to_goal`, `urban_block.traffic_light_cross`, `indoor_apartment.fetch_object`.
- Scenario recording and replay: a scenario run can dump its full state trace, which can later drive a deterministic replay.
- Integration with [test-harness-and-scenario-architecture.md](test-harness-and-scenario-architecture.md) so scenarios slot in as a new scenario lane.

**Files touched / created.**

- new: `src/python/omniworld/scenario/`
- new: `projects/samples/demos/controllers/omniworld_supervisor/` (Python supervisor)
- new: `projects/samples/demos/worlds/scenario_*.wbt` per template
- modify: [test-harness-and-scenario-architecture.md](test-harness-and-scenario-architecture.md) to document the new lane

**Milestones.**

1. **M2.4.a** — A warehouse pick-and-place scenario runs end-to-end, success/failure detected.
2. **M2.4.b** — Scenario trace replay is deterministic vs the live run.
3. **M2.4.c** — Every Tier 1 biome has at least one bundled scenario template.

**Deliverables.** Scenario engine, supervisor controller, bundled templates, test-lane integration.

**Risks.** Supervisor lockstep with controllers. Mitigate by using the same supervisor step pattern as a debug supervisor controller (e.g. `projects/samples/demos/controllers/contact_points_supervisor/`).

### T2.5 — Domain randomization

**Why.** Training loops care less about one perfect world and more about **many plausible variants**. Domain randomization is the point where procedural generation and ML training meet.

**Scope.**

- `dr/` subpackage with:
  - `RandomizableParameter` — a parameter with a distribution attached.
  - `Randomizer` — applies a distribution to a recipe's parameter set.
  - Common distributions: uniform, log-uniform, truncated normal, categorical, seeded-by-seed.
- Per-biome "randomization profiles" — safe, pre-tuned parameter ranges per biome (so a novice user does not get a world with impossible physics).
- Texture randomization: swap the appearance PROTOs bound to scattered props within a catalog-tagged family (e.g. "any asphalt-family appearance").
- Lighting randomization: sun elevation, sky preset, shadow strength.
- Batch-generation CLI: `omniworld batch --recipe warehouse --seeds 0-999 --out-dir /tmp/worlds`. Parallelized, streaming, produces 1000 validated worlds with manifests.
- Generator-to-sim streaming: a Python helper that feeds generated worlds into a headless simulator as fast as they are produced — the "1 world/s on mid tier" exit criterion.

**Files touched / created.**

- new: `src/python/omniworld/dr/`
- new: `src/python/omniworld/cli/batch.py`
- new: example training driver under `projects/samples/demos/controllers/omniworld_dr_example/`

**Milestones.**

1. **M2.5.a** — Any Tier 1 biome runs 100 randomized seeds without validation failure.
2. **M2.5.b** — Batch generation hits 1 world/s on the mid tier for `outdoor_forest`.
3. **M2.5.c** — Streaming driver feeds 100 worlds into headless sim back-to-back without state leakage.

**Deliverables.** DR subpackage, batch CLI, streaming driver, example training loop.

**Risks.** Seed collision between DR-randomized sub-parameters. Mitigate with strict parent-seed derivation — every sub-seed is deterministically derived from the parent seed and a name.

### T2.6 — Agent tool integration

**Why.** OmniSim is the sim environment for OmniLink agents. A world generator that agents cannot call is a missed opportunity.

**Scope.**

- `omniworld.agent_tool` — a `ToolRunner` implementation that registers the following tools on the OmniLink side:
  - `list_biomes() -> [biome_name]`
  - `describe_biome(name) -> JSONSchema`
  - `generate_world(biome, params, seed) -> WorldHandle`
  - `validate_world(handle) -> ValidationReport`
  - `run_scenario(handle, scenario) -> ScenarioResult`
- Natural-language params: a `prompt` field on `generate_world` maps free-text to biome + parameters using a small LLM pass (uses the Claude API; see the `claude-api` agent skill if your harness provides it). Strictly optional; no-prompt callers still get deterministic param-only generation.
- Controller bundled under [projects/samples/demos/controllers/](../../projects/samples/demos/controllers/) — a sibling of the existing OmniLink bridge controllers there that hosts the generator tools alongside the robot tools.
- Documentation: "How an OmniLink agent authors a world" section in [simulation-authoring-for-coding-agents.md](simulation-authoring-for-coding-agents.md).

**Files touched / created.**

- new: `src/python/omniworld/agent_tool/`
- new: `projects/samples/demos/controllers/omniworld_agent/`
- modify: [docs/developer/simulation-authoring-for-coding-agents.md](simulation-authoring-for-coding-agents.md)

**Milestones.**

1. **M2.6.a** — Agent tool lists and describes biomes via MCP.
2. **M2.6.b** — Agent generates a world from parameters and loads it into a live sim session.
3. **M2.6.c** — Agent generates a world from a natural-language prompt and the result validates.

**Deliverables.** Agent tool, controller, documentation.

**Risks.** LLM-produced parameters out of safe range. Mitigate by clamping every param to biome-defined safe ranges before generation; log out-of-range attempts.

### T2.7 — BIM and CAD ingress (optional)

**Why.** Industry partners regularly have BIM (IFC) or CAD (STEP, DXF) data for a site they want to simulate. Admitting them as inputs closes the last "but I have real data" objection.

**Scope.** Optional; shipped only if a partner project requests it.

- `ingress/ifc.py` — read IFC via `ifcopenshell`. Extract walls, slabs, openings, structural elements. Feed into the indoor solver.
- `ingress/dxf.py` — read 2D DXF floor plans.
- Conservative mesh conversion: BIM meshes are simplified to collision-friendly forms before being consumed.

**Files touched / created.** `src/python/omniworld/ingress/ifc.py`, `dxf.py`, tests.

**Deliverables.** Optional ingress modules, tests.

**Risks.** IFC complexity — `ifcopenshell` dependency is heavy. Treat as optional install.

### T2.8 — Ecological simulation

**Why.** Tier 1 scatter is statistically uniform. Real vegetation is not. Pines cluster where drainage is good; oaks dominate flatter soil; willows hug streams; a north-facing slope wears moss while its south face wears wildflowers. Closing this gap is the single strongest outdoor realism lift after weathering.

**Scope.**

- `realism/ecology.py` runs after heightmap generation and before scatter:
  - Per-cell **soil depth** derived from slope (flat → deep, steep → thin) and erosion (accumulation → deep, scour → thin).
  - Per-cell **soil type** from the lithology map (introduced with T2.1 splatmaps).
  - Per-cell **moisture** from flow accumulation (T2.1) + distance to water bodies.
  - Per-cell **aspect** (compass direction of slope).
  - Per-cell **light exposure** from terrain-occlusion ray-march against the average sun path (cheap — a few rays per cell).
- **Species suitability maps.** Each plant species declares a suitability function over these fields (pine likes shallow soil and any aspect; willow needs high moisture; oak prefers deep soil and moderate slope). The ecology module produces one probability map per species, then samples scatter from their combined distribution.
- **Competition pass.** Initial sampling overproduces seedlings; a simple competition model (mutual exclusion radius scaled by expected mature size) thins the stand; survivors are placed with an age drawn from a species-specific Weibull distribution. Age drives height, canopy radius, and trunk thickness.
- **Prevailing wind.** A world-level wind direction tilts tall species slightly away from the wind; exposed ridges have shorter, more gnarled trees.
- **Animal traces.** From T2.4 scenario hooks and biome profile: deer paths through forests, rabbit warrens near hedges, bird nests in tall trees. These are not simulated; they are placed using the footfall-field and heuristic rules.

**Files touched / created.**

- new: `src/python/omniworld/realism/ecology.py`
- new: `src/python/omniworld/realism/species/` — one file per canonical species with suitability and competition parameters
- modify: `biomes/outdoor_forest.py`, `biomes/outdoor_desert.py`, `biomes/real_world_block.py` (urban ecology: street trees by sidewalk presence)
- modify: asset catalog to include species and age-scaling metadata

**Milestones.**

1. **M2.8.a** — Suitability maps produce visibly different distributions for pine / oak / willow on the same terrain.
2. **M2.8.b** — Competition pass eliminates implausible stand densities; final density matches reference forest data within a tolerance.
3. **M2.8.c** — A wind-exposed ridge has short trees; a sheltered valley has tall ones.
4. **M2.8.d** — An `ecology=off` escape hatch lands for benchmarks that want the old uniform distribution.

**Deliverables.** Ecology module, species library, wind integration, updated outdoor biomes.

**Risks.** Species suitability tuning is empirical. Mitigate by shipping three curated species sets (temperate forest, boreal, desert) rather than attempting universal coverage in Tier 2.

### T2.9 — Human- and machine-trace overlay

**Why.** A warehouse floor that shows no tire tracks from the loading door reads as a fresh build. An office with no wear between the door and the coffee machine reads as uninhabited. Use leaves evidence; evidence is what sells the world.

**Scope.**

- `realism/traces.py` computes a **footfall field** over the layout:
  - Sources: doors, spawns, goals, scenario waypoints, named "use points" declared by biome recipes (warehouse loading door, office coffee machine, kitchen sink).
  - Sinks: seats, resting positions.
  - Edges: the zone graph from the T1.4 layout DSL.
  - Field value at a cell = combined traversal probability (a cheap random-walk or Dijkstra-with-decay).
- **Floor wear decal pass.** The footfall field drives a per-cell wear intensity that is baked into a **floor wear decal atlas** laid over the floor shader. High-traffic cells darken; baseboards near high-traffic walls scuff; linoleum shows polish paths.
- **Hand-height wear.** Door frames, push-plates, handrails, and light switches carry a wear decal at 0.9–1.2 m; generated automatically from catalog-declared touch points on the relevant PROTOs.
- **Tire-track pass** (industrial biomes). Paths between loading doors and racking aisles are decorated with procedural tire tracks (width matching declared forklift or truck width), biased by usage frequency.
- **Use-trace clutter.** Where footfall is high and a flat surface is present (break-room table, workshop workbench), micro-clutter density is boosted: more coffee rings, more scattered papers, more tool scatter.
- Trace intensity scales with the world `age` parameter from T1.10 so "new" worlds have barely-visible traces and "abandoned" worlds show dense wear.

**Files touched / created.**

- new: `src/python/omniworld/realism/traces.py`
- new: `src/python/omniworld/realism/trace_decals/` — curated decal atlas
- modify: relevant biome recipes (`warehouse`, `indoor_office`, `indoor_apartment`, `urban_block`) to declare use points

**Milestones.**

1. **M2.9.a** — Footfall field visibly matches intuition on the warehouse reference world.
2. **M2.9.b** — Floor wear decal atlas renders correctly under every preset.
3. **M2.9.c** — Hand-height wear auto-places on door frames and handrails.
4. **M2.9.d** — Tire tracks lay down realistic curved paths between loading door and racks.
5. **M2.9.e** — Side-by-side panel test: trace-overlay-enabled worlds rate significantly more inhabited than disabled.

**Deliverables.** Trace module, decal atlas, biome use-point declarations, documented authoring guide for adding use points.

**Risks.** Decal overlap and visual noise. Mitigate by capping per-region decal density and by tuning falloff so traces blend into base material smoothly.

### T2.10 — Environmental state: time, weather, and season

**Why.** A world fixed at noon on a clear day is a rendering of a moment. A world that exists in a time and weather state, where sun, sky, wetness, leaf color, and interior lighting all follow from a single declaration, is a place.

**Scope.**

- `realism/world_state.py` defines a **`WorldState`** value object: `time_of_day`, `date`, `latitude`, `weather`, `wind_speed`, `temperature`, `season` (can be explicit or derived from date + latitude).
- A recipe can set a `WorldState` as an authoring input; a scenario can override it per episode; domain-randomization can sample it.
- Downstream passes consume `WorldState` coherently:
  - **Sun position** from time + date + latitude (Spencer's equations) drives the directional light orientation and intensity.
  - **Sky preset** selected from weather: sunny / partly-cloudy / overcast / fog / rain / storm / clear-night.
  - **Atmospheric fog** parameters come from weather and humidity.
  - **Surface wetness** shader is enabled by rain or recently-rain weather; puddle masks are generated by flow-accumulation at bake time and activated by the shader at runtime.
  - **Rain / snow particle systems** are enabled per weather; wired into the renderer's existing post FX stack.
  - **Snow accumulation pass** at bake: every horizontal-facing mesh vertex gets a snow-accumulation value proportional to `max(dot(up, normal), 0)` and wind shadow, written as a vertex attribute the shader reads.
  - **Foliage color** per season (bright green summer → yellow/orange autumn → leafless winter → light green spring). Implemented as a season-blend uniform on the foliage shader.
  - **Ground foliage and undergrowth** density follows season.
  - **Interior light schedule**: lamps on at dusk/night, office fluorescents on during workday hours.
- Scenario engine gains a **`TimeFlow`** declaration: scenarios can run at wall-clock speed, at an accelerated clock, or freeze at a moment.
- The renderer side consumes all of this through the planned sun-driven atmospheric sky in [engine-migration-plan.md](engine-migration-plan.md) T1.3 and the wet/snow shaders in extended BRDF (T3.1).

**Files touched / created.**

- new: `src/python/omniworld/realism/world_state.py`
- new: `src/python/omniworld/realism/weather/` — one module per weather preset
- new: `src/python/omniworld/realism/seasons.py`
- new: `src/python/omniworld/emit/snow_accumulation.py`
- modify: all biome recipes to accept `WorldState` as an optional input
- modify: scenario schema to include `TimeFlow`

**Milestones.**

1. **M2.10.a** — Sun angle at (latitude, date, time) matches an authoritative reference to < 0.1°.
2. **M2.10.b** — Rain weather on an outdoor world produces puddles at terrain low points and a visibly wet material response.
3. **M2.10.c** — Snow accumulation on a warehouse roof shows correct up-facing bias.
4. **M2.10.d** — Season blend visibly shifts foliage; "winter" outdoor forest looks correct.
5. **M2.10.e** — Interior lights track a workday schedule under scenario-driven `TimeFlow`.

**Deliverables.** `WorldState` module, weather presets, season blending, snow accumulation, scenario `TimeFlow`.

**Risks.** Renderer dependency — wetness and snow shaders require the extended BRDF work in Tier 1 of the rendering plan. Mitigate by shipping a cheap fallback (base-color darken + roughness reduce) that works on the WREN path.

### T2.11 — Reference-driven statistical priors

**Why.** Hand-tuned constants like "tree density = 0.3 per m²" or "average office desk has 4 items on it" give Tier 1 results that feel *approximately* right in a single biome and *wildly* wrong when a user asks for "Tokyo" versus "Helsinki". Priors grounded in real-world data are how the generator stops being a toy.

**Scope.**

- `data/priors/` hosts versioned, signed JSON/Parquet prior tables keyed by region and biome:
  - **Building priors** — height distributions, footprint size, setback, density. Derived from OSM + city open-data portals (licensed, redistributable subsets only).
  - **Vegetation priors** — species weights, density per m², canopy-radius distribution. Derived from Copernicus land-cover + national forest inventories where available.
  - **Clutter priors** — items-per-desk, items-per-shelf, per-desk item category distribution. Derived from a hand-annotated subset of public indoor-scene datasets (e.g. Scan2CAD, ScanNet at small scale, with licenses respected).
  - **Urban furniture priors** — street-light spacing, crosswalk frequency, advertising density. Derived from OSM statistics.
- A `RegionProfile` declares prior-table selections: `Tokyo`, `Stockholm`, `Cairo`, `Lagos`, `SaoPaulo`, etc. Generator picks consume priors through the profile.
- Priors are **versioned** with content hashes captured in the seed manifest. Upgrading a prior table bumps the manifest hash.
- A **prior audit CLI** (`omniworld audit-priors <recipe>`) shows which priors a recipe consumes and at what weight, so a user can see where "Tokyo" differs from "Stockholm" in parameter space.
- Prior derivation scripts live under `training/priors/` — outside the runtime path. They are reproducible but only re-run when the source data updates.

**Files touched / created.**

- new: `src/python/omniworld/data/priors/`
- new: `src/python/omniworld/realism/priors.py`
- new: `training/priors/` (offline derivation)
- new: `scripts/dev/audit_priors.py`
- modify: all biome recipes to consume priors through the active `RegionProfile`
- new: `docs/developer/omniworld-region-profiles.md`

**Milestones.**

1. **M2.11.a** — Building priors for Tokyo and Stockholm ship and visibly differ on `real_world_block` generation.
2. **M2.11.b** — Vegetation priors for temperate, boreal, tropical climates ship and differ on `outdoor_forest`.
3. **M2.11.c** — Clutter priors for "home office" vs "shared workspace" produce visibly different desk populations.
4. **M2.11.d** — `omniworld audit-priors` exposes per-recipe prior consumption.

**Deliverables.** Versioned prior tables, `RegionProfile` API, audit CLI, region-profile documentation.

**Risks.** Data licensing. Only redistributable subsets land in the repo; everything else is a reproducible recipe that a user can run locally. No scraped-from-closed-sources data.

### Tier 2 combined deliverables

At Tier 2 end the simulator ships with:

1. Real-world ingress: OSM, DEM, land-cover rasters, floor-plan rasters, optional BIM/CAD.
2. Advanced heightmap primitives: hydraulic and thermal erosion, river networks, splatmaps.
3. Wave Function Collapse for interior layouts.
4. Scenario engine with supervisor controller and bundled templates per biome.
5. Domain randomization and batch generation CLI hitting 1 world/s on mid tier.
6. OmniLink agent tool registered via `ToolRunner`; natural-language prompts map to validated generation.
7. Unified automobile OSM importer behind the new ingress library.
8. **Ecological simulation** — vegetation, soil, moisture, aspect, competition, wind.
9. **Human- and machine-trace overlay** — footfall-driven floor wear, hand-height scuffs, tire tracks, use-trace clutter.
10. **Environmental state** — time of day, date, latitude, weather, season. Coherent sun, sky, wetness, snow, foliage, interior lighting schedules.
11. **Region profiles and data-grounded priors** — Tokyo vs Stockholm vs Cairo visibly differ in building height, vegetation, and clutter distributions.
12. Updated cookbook, region-profile guide, and a "realism grading rubric" used by the blind panel.

Practically Tier 2 turns OmniSim's procedural engine from a generator into a platform that produces inhabited, weathered, weather-aware worlds from real data — anyone can drop in a real location, describe a task, pick a climate and age, and get a training-ready scenario that matches real-world statistics. Architecturally it is still Python-land with stable APIs, ready for the Tier 3 learned, runtime-streaming, and style-matching work.

---

## Tier 3 — Learned Generation, Runtime Streaming, and Path-to-Production

**Tier goal.** Two themes. First, dissolve the ceiling on "authored-looking" generation by admitting learned distributions — diffusion heightmaps, language-conditioned prop placement, neural texture synthesis. Second, dissolve the "everything through a `.wbt` file" bottleneck by supporting runtime-streamed worlds that change as the camera moves. Together these put OmniSim in the top tier of simulation world generators publicly available.

**Tier duration estimate.** 9–12 months full-time, or ~18 months split, depending on whether the learned subsystems are trained in-repo or lean on pretrained external models.

**Tier exit criteria.**

1. A diffusion-based heightmap generator produces terrain visually distinguishable (in a blind panel) from Tier 1 procedural at equal or better plausibility.
2. A language-conditioned generator turns "a cluttered mechanic's garage with two workbenches, a car on a lift, and scattered tools" into a runnable, validated world.
3. A streaming world demo: an outdoor scene extending to the horizon, with chunks generated on demand as the camera moves, 60 FPS on the mid tier.
4. Learned generators run either fully locally (small models) or through a hosted API pipe cleanly abstracted behind the same library interface.
5. Sensor determinism and headless-reproducibility contracts still hold for any recipe that does not explicitly opt into online learned generation.

### T3.1 — Streaming and chunked worlds

**Scope.**

- `streaming/` subpackage: world is a grid of chunks; each chunk is a separate Layout that shares only a border contract with its neighbors.
- On-demand chunk generation: as a simulated camera moves, the streaming supervisor requests chunks, generates them (or loads cached ones), and dynamically adds their contents to the live scene via a supervisor API.
- Chunk eviction with LRU on a memory budget.
- Deterministic chunk-seed derivation: chunk at integer coords (i, j) has seed = H(world_seed, i, j).
- Integrates with the renderer's LOD / impostor system from [engine-migration-plan.md](engine-migration-plan.md) Tier 3.
- First streaming biome: `outdoor_open_world` — effectively infinite outdoor.

**Files touched / created.**

- new: `src/python/omniworld/streaming/`
- new: `projects/samples/demos/controllers/omniworld_streaming_supervisor/`
- requires: supervisor API enhancements in the sim to allow dynamic node insertion/removal efficiently — coordinated with [scene-tree-selection-and-runtime-mutation.md](scene-tree-selection-and-runtime-mutation.md)

**Milestones.**

1. **M3.1.a** — Static 3×3 chunk grid generates and loads correctly with matching borders.
2. **M3.1.b** — Dynamic chunk load/unload keeps memory under budget as the camera moves.
3. **M3.1.c** — 60 FPS demo on the mid tier with view-distance 500 m.

**Deliverables.** Streaming subpackage, supervisor, open-world biome, perf demo.

**Risks.** Scene-tree mutation cost. Mitigated by the concurrent work under [scene-tree-selection-and-runtime-mutation.md](scene-tree-selection-and-runtime-mutation.md).

### T3.2 — LOD and impostor pipeline

**Scope.**

- Offline tool that bakes LODs for scattered PROTOs at authoring time: reduced-mesh LOD1 and LOD2, plus a billboard impostor atlas for the farthest tier.
- Scatter primitives tag placements with LOD preference; the scene picks the right variant based on distance.
- Instanced rendering path for scattered props — requires the renderer work in [engine-migration-plan.md](engine-migration-plan.md) §14.4 T2.2.d (instanced draw submission; T2.2.d was attempted on WREN, reverted, and is now expected to land natively in wgpu Phase γ/δ) plus an InstanceField PROTO wrapper for scatter authoring.

**Files touched / created.**

- new: `scripts/dev/bake_lods.py`
- new: `src/python/omniworld/catalog/lod.py`
- renderer collaboration in `src/omnirender/Instancing.*`

**Milestones.**

1. **M3.2.a** — Offline LOD baking runs on the tree PROTO family.
2. **M3.2.b** — LOD selection at runtime visibly reduces draw calls in profiler.
3. **M3.2.c** — 48,000 impostor trees render at 60 FPS on mid tier.

**Deliverables.** LOD baker, scatter LOD binding, instanced path.

**Risks.** Impostor popping at LOD boundaries. Mitigate with cross-fade.

### T3.3 — Learned heightmap generation

**Scope.**

- Two interchangeable backends: a small in-repo diffusion model trained on a curated heightmap dataset, or a pretrained external model accessible through a thin adapter.
- `learned/heightmap.py` with a common interface: `generate_heightmap(size, seed, style, condition=None) -> heightmap`.
- `style` selects between "rolling hills", "alpine", "canyon", "mesa", etc.
- `condition` optionally accepts a low-resolution sketch and upscales.
- Caching: learned generation is slow; cache by hash of (model version, style, seed, condition).

**Files touched / created.**

- new: `src/python/omniworld/learned/`
- new: `src/python/omniworld/learned/heightmap.py`
- new: `tests/python/omniworld/test_learned_heightmap.py`
- dataset & training scripts in `training/heightmap/` — outside the runtime path

**Milestones.**

1. **M3.3.a** — Backend abstraction works; procedural fallback when no model is available.
2. **M3.3.b** — One learned style ships; blind panel prefers it vs procedural at least half the time.
3. **M3.3.c** — Three styles ship; all hash-cached for reproducibility.

**Deliverables.** Learned heightmap backend, styles, caching, optional training scripts.

**Risks.** Platform install friction for model runtimes (torch, onnx). Mitigate by shipping with the learned backends off by default and a small pure-procedural fallback always available.

### T3.4 — Language-conditioned world generation

**Scope.**

- A wrapper that takes natural language and produces a full world: chooses biome, sets parameters, seeds scenario, picks props from the asset catalog.
- Two-stage: (1) LLM produces a structured `WorldSpec` JSON, (2) deterministic generation consumes the spec. The LLM stage is optional; a power user can author the spec directly.
- Safety rails: the LLM must not invent PROTOs that do not exist in the catalog; the spec is schema-validated before the generator consumes it.
- Sits on top of the Tier 2 agent tool: the `prompt` field from T2.6 now has a full pipeline behind it.
- Documented "what the prompt can and cannot do" section in the user guide, with worked examples.

**Files touched / created.**

- new: `src/python/omniworld/learned/language.py`
- new: `src/python/omniworld/schema/world_spec.py`
- modify: agent tool to route `prompt` through the language pipeline
- modify: user guide with a prompt cookbook

**Milestones.**

1. **M3.4.a** — Prompt → WorldSpec → world pipeline works on 10 representative prompts.
2. **M3.4.b** — Hallucinated PROTO names are caught and remapped to catalog-valid alternates.
3. **M3.4.c** — End-to-end agent demo: an OmniLink agent hands a user a world purely from a prompt.

**Deliverables.** Language pipeline, WorldSpec schema, prompt cookbook, agent demo.

**Risks.** LLM cost in CI. Mitigate by caching the WorldSpec outputs in a fixture file for CI tests; live prompting runs only in a nightly lane.

### T3.5 — Learned texture and appearance synthesis

**Scope.**

- Per-material neural texture synthesis: give the generator a style prompt ("weathered concrete", "fresh asphalt after rain") and receive a tileable texture set (albedo, normal, roughness).
- Cached per-prompt hash; synthesis happens once, cached forever.
- Integration with the splatmap work from T2.1 — a terrain can request a learned material bed.

**Files touched / created.**

- new: `src/python/omniworld/learned/texture.py`
- storage convention: generated textures land under `resources/generated_textures/` with content-hashed filenames

**Milestones.**

1. **M3.5.a** — One material prompt produces a usable tileable set.
2. **M3.5.b** — Terrain splatmap picks up learned materials.
3. **M3.5.c** — Cached lookups are fast enough that a world with 6 learned materials generates in under 3 seconds on cache-hit.

**Deliverables.** Texture synthesis module, storage convention, caching.

**Risks.** Texture quality. Mitigate by keeping the learned pipeline opt-in and the procedural path authoritative by default.

### T3.6 — Photogrammetry and 3D scan ingress

**Scope.**

- Import 3D scans (PLY, OBJ, GLTF point clouds or meshes) as a scene seed.
- Automatic mesh decimation and collision-geometry extraction.
- Semantic segmentation pass (if a pretrained model is available) tagging portions of the scan as "floor", "wall", "furniture" so the generator can reason about them.
- Hybrid worlds: scan the back wall of a real workshop, procedurally generate the rest of the scene around it.

**Files touched / created.**

- new: `src/python/omniworld/ingress/scan.py`
- new: `src/python/omniworld/primitives/mesh_decimate.py`

**Milestones.**

1. **M3.6.a** — A test PLY scan imports into a runnable world.
2. **M3.6.b** — Collision geometry extracted automatically.
3. **M3.6.c** — Hybrid scene: scan-based environment plus procedural props.

**Deliverables.** Scan ingress, decimator, hybrid demo.

**Risks.** Scan license handling. Keep this path strictly local-file; no online scan sources in repo.

### T3.7 — Runtime C++ core (conditional)

**Why.** If Tier 2's streaming and DR batch use cases bottleneck on Python, we extract the hot path. Conditional — not a guaranteed deliverable.

**Scope.**

- `src/omniworld_core/` — C++ implementation of the hottest primitives: heightmap fBm / erosion, scatter rejection sampling, chunk borders.
- Python bindings via pybind11, drop-in compatible with the Tier 1 API.
- Only extracted once a profile shows > 30% of time in a single Python primitive during a realistic streaming run.

**Files touched / created.** New C++ subtree, bindings, build integration into the existing [build-and-iteration.md](build-and-iteration.md) flow.

**Milestones.** Only written if T3.7 is triggered by profile evidence.

**Deliverables.** Faster hot path, same Python API.

**Risks.** Build surface area. Keep the binding narrow and the subset small.

### T3.8 — Photoscanned asset library

**Why.** Hand-authored PROTOs under [projects/objects/](../../projects/objects/) are clean, simple, and low-poly. They will never be mistaken for photographs. Photoscanned assets (Megascans, Polyhaven, scans produced in-house) close the fidelity gap that procedural generation alone cannot.

**Scope.**

- Native import of PBR scan sets: albedo, normal, roughness, displacement, AO, plus mesh + LODs + collision.
- `scripts/dev/import_scan.py` — a batch tool that takes a directory with a scan-kit-style layout and emits:
  - A new PROTO under a new `projects/objects/scanned/<category>/` tree.
  - Correct bounding volumes and collision approximation.
  - Catalog entry with variation / weathering / settle / surface-manifest metadata filled from heuristics.
  - LODs baked (T3.2).
- A curated **realism pack**: ~300 scanned assets covering the most common props across Tier 1/2 biomes. Delivery mechanism follows [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md) rules — either in-repo (for redistributable licenses) or as a reproducible download script (for gated sources).
- Per-asset license metadata; the generator can filter "use only MIT-licensed scans" for users with strict license requirements.
- A fallback rule: every scanned asset declares a hand-authored PROTO fallback; if the realism pack is not installed, the generator still produces a valid world, just at lower fidelity.

**Files touched / created.**

- new: `projects/objects/scanned/` (PROTOs)
- new: `scripts/dev/import_scan.py`
- new: `resources/realism_pack/` or a download manifest under `scripts/packaging/`
- modify: catalog query to accept `prefer_scanned=true`
- modify: [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md) to document the realism pack

**Milestones.**

1. **M3.8.a** — Import tool produces a valid PROTO from one reference scan kit.
2. **M3.8.b** — 50-asset starter pack ships; recipe prefers scanned variants when available.
3. **M3.8.c** — 300-asset full pack; performance presets ensure `balanced` still hits FPS.
4. **M3.8.d** — Hand-authored fallback verified under a "realism pack missing" CI lane.

**Deliverables.** Scan import tool, realism pack, license metadata, fallback validation.

**Risks.** Per-asset memory cost on realism pack. Mitigate with LODs from T3.2 and with texture streaming prep from [engine-migration-plan.md](engine-migration-plan.md) cross-tier work.

### T3.9 — Neural surface imperfection and material response

**Why.** Tier 1 weathering uses a 4-channel parameter (dirt, wear, moisture, oxidation) blended procedurally. That covers a lot of ground but cannot match the local detail a scanned or learned imperfection map captures — the specific drip pattern under a gutter, the specific scuff pattern at a loading dock door.

**Scope.**

- `learned/imperfection.py` — a conditional generator that takes (material base, weathering params, use context, seed) and produces a per-instance imperfection texture set (dirt overlay, scratch pattern, drip pattern, patina) at the material's texel scale.
- Backends: in-repo small diffusion model trained on a labeled imperfection dataset, or external model adapter.
- Caching: aggressively cached by (material_hash, weathering_params, use_context, seed). Once generated, imperfection maps are reused across many scenes.
- Integrates with the asset catalog: every PROTO declares whether it supports learned imperfection and at what texture resolution.
- Offline bake mode: a user can pre-generate the imperfection set for a realism-pack asset so runtime costs are zero.
- Works alongside T2.9 traces — imperfection provides the material detail, traces provide the placement pattern.

**Files touched / created.**

- new: `src/python/omniworld/learned/imperfection.py`
- new: `training/imperfection/` — dataset prep and training scripts
- modify: catalog to declare per-PROTO imperfection support

**Milestones.**

1. **M3.9.a** — Backend produces a usable imperfection set for a reference material.
2. **M3.9.b** — A crate rendered with learned imperfection rates higher than procedural-only in a blind panel.
3. **M3.9.c** — Offline bake reduces runtime cost to cache hit on a second generation.

**Deliverables.** Learned imperfection pipeline, dataset prep, catalog integration, offline bake.

**Risks.** Dataset licensing for training imperfection textures. Use only openly-licensed texture datasets; document the provenance.

### T3.10 — Style transfer and reference match

**Why.** The closest we have to "make this scene look like Berlin in autumn 1985" today is a prompt-and-hope in T3.4. A reference-match pipeline that takes images and tunes the generator to match them is what turns OmniSim into a sim that can reproduce a *specific* real place on demand.

**Scope.**

- `learned/reference_match.py` — a two-stage pipeline:
  - (1) **Style encoder** — take 3–20 reference images; extract a style vector covering lighting, color palette, material character (rough/polished, clean/weathered), prop density, architectural period.
  - (2) **Parameter solver** — given a target style vector and a base recipe, produce a `WorldSpec` with parameter values that minimize the style distance between the generated world and the reference set.
- Implementation approach: use a pretrained vision model (CLIP-class) to embed both the reference images and rendered samples from the generator; optimize the generator parameters via a few-shot search (cmaes, or a small learned parameter-prior network).
- A **location pack** concept: curated style vectors for well-known locations shipped with the realism pack ("Berlin tenement 1985", "Tokyo shotengai 1990", "Cairo market").
- Scoped to **appearance and density**, not geometry. Geometry match stays with OSM/DEM/BIM ingress in Tier 2.
- Sensor-determinism contract: a matched world seed is cached; subsequent runs reproduce the exact style-matched generation byte-for-byte.

**Files touched / created.**

- new: `src/python/omniworld/learned/reference_match.py`
- new: `src/python/omniworld/learned/style_encoder.py`
- new: `src/python/omniworld/learned/location_packs/`
- modify: agent tool to expose `match_style(reference_images)`

**Milestones.**

1. **M3.10.a** — Style vectors cluster correctly on a labeled reference set.
2. **M3.10.b** — Parameter solver reduces style distance by a measurable amount over 20 optimization steps.
3. **M3.10.c** — Three location packs ship and visibly reproduce their reference character.
4. **M3.10.d** — Blind panel rates reference-matched worlds as significantly closer to the reference than a random generation.

**Deliverables.** Style encoder, reference-match solver, location packs, agent endpoint.

**Risks.** Vision-model dependency size. Mitigate by shipping style vectors as precomputed embeddings and not requiring the vision model at runtime for `generate_from_style_vector`.

### T3.11 — Acoustic authoring (optional)

**Why.** Scenarios that depend on sound — echo-based navigation, following a machine by its hum, localizing voices — need a world that declares acoustic properties. This is optional because only a subset of OmniSim use cases need it.

**Scope.**

- Per-material absorption and reflection coefficients in the catalog.
- Per-enclosed-region impulse response bake: cheap ray-traced room-acoustics using the scene BVH from the renderer's DDGI work ([engine-migration-plan.md](engine-migration-plan.md) T3.4).
- Scenario supervisor exposes an "acoustic query" (IR at a listener position from a source position).
- `realism/acoustics.py` wires it all together.

**Files touched / created.**

- new: `src/python/omniworld/realism/acoustics.py`
- modify: catalog schema for per-material acoustic properties
- modify: scenario supervisor API

**Milestones.**

1. **M3.11.a** — Impulse responses vary correctly between tiled, carpeted, and outdoor regions.
2. **M3.11.b** — Scenario query returns correct IR for a moving listener.

**Deliverables.** Acoustic pass, catalog extension, scenario API.

**Risks.** Scope creep into a full acoustic engine. Keep it to static bake with per-scenario queries; real-time moving-source acoustics is out of scope.

### T3.12 — Regional and cultural authenticity packs

**Why.** A "generic urban" world is always slightly wrong for every user. Explicit regional content — European, North American, East Asian, Latin American, sub-Saharan African — tuned with T2.11 priors and T3.10 location packs lets users get a world that is not just correct statistically but correct culturally.

**Scope.**

- Region-specific asset sub-catalogs: European signage fonts, North American right-drive lanes and stop signs, East Asian vertical signage, African market structures.
- Region-specific building heuristics: European tenement floor-plan, North American single-family lot, Japanese multi-story compact, Brazilian favela cluster.
- Region-specific clutter profiles: what is on a bedside table in a Stockholm apartment vs a Tokyo apartment vs a Cairo apartment.
- Regional scenarios: culturally plausible waypoint sets and goals (e.g. "navigate to the konbini" in Tokyo; "navigate to the bodega" in NYC).
- A non-trivial authoring commitment — each region pack is a deliberate, researched body of work, not an afterthought. Ship packs individually; do not gate the generator on any one.

**Files touched / created.**

- new: `projects/objects/regional/<region>/` (PROTOs per region where existing PROTOs do not fit)
- new: `src/python/omniworld/realism/region_packs/`
- new: docs/ region guides per shipped region pack

**Milestones.**

1. **M3.12.a** — First region pack ships (pick one — likely European, matching existing PROTO bias).
2. **M3.12.b** — Second region pack differs visibly from the first on the same biome.
3. **M3.12.c** — Scenario templates use region-appropriate cultural references.

**Deliverables.** Per-region content packs, region-guide docs, region-aware scenario templates.

**Risks.** Cultural representation done badly is worse than not done. Each region pack is reviewed by at least one person with lived familiarity with the region; authoring guidance spells this out.

### Tier 3 combined deliverables

At Tier 3 end the simulator ships with:

1. Streaming, chunked worlds with LRU eviction and view-distance 500 m at 60 FPS.
2. LOD/impostor pipeline allowing dense outdoor scatter (10,000+ objects).
3. Learned heightmap styles with a procedural fallback.
4. Language-conditioned world generation: prompt to validated runnable world.
5. Learned texture and appearance synthesis.
6. 3D-scan and photogrammetry ingress; hybrid real/procedural worlds.
7. Optionally, a C++ core for hot-path generation.
8. **Photoscanned asset library (~300 assets)** with LODs, collision, catalog metadata, and hand-authored fallbacks.
9. **Neural surface imperfection** giving every placed instance local drip / scuff / patina detail beyond the Tier 1 procedural weathering.
10. **Style transfer and reference match** — 3–20 reference images or a location pack produce a world visibly in that style.
11. **Optional acoustic authoring** for scenarios that need sound propagation.
12. **Regional / cultural authenticity packs** — European, North American, East Asian, etc. — as a continuing body of work, each independently reviewed.

Practically Tier 3 moves OmniSim from "state of the art procedural" to "state of the art period": the generator is agent-native, learned-model-augmented, style-matchable, runtime-streaming, and pulls from a photoscanned library that makes produced worlds indistinguishable-at-a-glance from photographs — while preserving every Tier 1 determinism guarantee for the paths that do not opt into learned or streamed generation.

---

## Cross-Cutting Concerns

### Determinism

- Each tier's primitives take an explicit `rng` and never read clocks, threads, or environment.
- The seed manifest captures library version, catalog hash, recipe name, recipe parameter set, and (from Tier 3) learned-model versions.
- CI regression: pinned-seed runs assert byte-identical `.wbt` outputs.
- Exception: learned-generation paths (Tier 3) are cache-first; their cached outputs are byte-identical, but first-time generation depends on model versions captured in the manifest.

### Performance budgets

Each recipe declares:

- A maximum load time for the default-size world on the mid tier.
- A maximum initial contact-pair count (see [physics-contact-and-collision-complexity.md](physics-contact-and-collision-complexity.md)).
- A minimum headless FPS for a benchmark run.
- A maximum generator wall-clock for a single `generate` call (so agents and batch loops have SLA expectations).

All four are checked by the validator; all four are regression-tested in CI.

### Compatibility with existing worlds

No existing world under [projects/samples/](../../projects/samples/), [projects/robots/](../../projects/robots/), or elsewhere is modified by this plan. Hand-authored worlds stay hand-authored; the generator only ever adds new worlds under clearly named paths.

The one exception is the original one-off seed script, whose outdoor lineage was folded into `omniworld.generate("outdoor_forest", ...)` — the equivalent output is preserved through the shipped recipe.

### Sensor determinism

Sensor-facing cameras treat generated worlds exactly like hand-authored ones. A generator run that fails the sensor determinism check is a generator bug. The scenario engine must treat sensor outputs with the same rigor as the rest of the simulator; scenarios that depend on randomized prop positions must use deterministic seed derivation so a sensor capture at seed N is always the same.

### Relationship with the renderer

The procedural generator and the renderer evolve independently, but several Tier 2 / Tier 3 items depend on rendering features:

- T2.1 splatmaps need the extended-BRDF terrain shader from [engine-migration-plan.md](engine-migration-plan.md) T3.1.
- T3.2 LOD/impostor pipeline depends on the instancing path in OmniRender Tier 3.
- T3.3–T3.5 learned outputs benefit from the tonemap and atmospherics in rendering Tier 1 but do not require them.

Renderer tiers and generator tiers are independently shippable; each works on a world built by the other at its own tier.

### Testing strategy

- **Unit tests** on every primitive (heightmap, scatter, catalog query, scenario parser).
- **Golden-file tests** for each shipped recipe at seeds 0, 1, 42.
- **Validator tests**: synthetic bad worlds must fail the right validator.
- **Headless integration**: each recipe loads and steps 100 times without error.
- **Blind panel** (outside CI): periodic human evaluation of recipe aesthetic quality.

### Documentation

Each tier delivers or updates:

- `docs/developer/omniworld-user-guide.md` — user-facing CLI and library API.
- `docs/developer/omniworld-biome-cookbook.md` — how to write a recipe.
- `docs/developer/omniworld-scenario-guide.md` — from Tier 2 onward; scenario authoring.
- `docs/developer/omniworld-agent-playbook.md` — from Tier 2 onward; what OmniLink agents can do with the generator.
- Updates to [simulation-authoring-for-coding-agents.md](simulation-authoring-for-coding-agents.md), [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md), and [test-harness-and-scenario-architecture.md](test-harness-and-scenario-architecture.md).

## Open Questions

1. **Python vs C++ split.** Tier 1 is pure Python. If Tier 2 or Tier 3 streaming usage reveals a real bottleneck, do we extract primitives to C++ or lean on optional `numpy` / `scipy`? The plan reserves T3.7 for this decision and explicitly does not force it earlier.
2. **Learned model hosting.** Tier 3 learned components: run locally, hosted by OmniLink, or pluggable? Decision deferred to Tier 3 kickoff; both paths fit behind the same interface.
3. **Relationship to the automobile vertical.** Tier 2 unifies OSM ingress but keeps the automobile docs and tools as a consumer of the unified library. If the automobile importer's specific coordinate handling clashes with the general one, we prioritize preserving the automobile behavior and extend the general pipeline to accommodate.
4. **BIM/CAD partner dependency.** T2.7 is only undertaken if a partner engagement asks for it. It is a genuine optional.
5. **Scan licensing.** T3.6 scan ingress is strictly local-file; do we want a curated gallery of open-license scans shipped with the repo? Revisit at Tier 3 kickoff.
6. **Benchmark interaction.** How do generated worlds coexist with the stable-benchmark requirement in [benchmark-authoring.md](benchmark-authoring.md)? Generated worlds used as benchmarks must be pinned — same seed, same recipe version, same catalog hash — and regenerated only on explicit version bumps, never at CI time.

## Summary

Tier 1 gives the repo a real library, five biomes, physics-settled placement, per-instance variation, weathering, and micro-clutter — replacing one sterile generator script with an engine that produces believable, inhabited scenes. Tier 2 pulls in the real world on one end and agent calls on the other, adds scenarios so the generator produces tasks rather than only scenes, and grounds everything in ecological simulation, human-trace overlays, environmental state (time, weather, season), and data-driven regional priors. Tier 3 dissolves the last three ceilings: streaming for open worlds, learned generation for fidelity and prompt-driven authoring, and photoscanned assets plus style transfer for visual indistinguishability from reality. Each tier is independently shippable, each has clear exit criteria, each advances at least one realism pillar, and each preserves the simulator's existing determinism and sensor contracts.

Realism is the organizing goal, not a late polish. Every primitive the library adds — heightmap, scatter, layout, catalog, scenario, ingress, learned, streaming — is designed to feed into the settling, weathering, variation, ecology, trace, and state passes that make the output look real. A recipe that produces a correct-but-sterile scene is a Tier 1 bug, not a Tier 3 feature request. The plan above is how OmniSim's generator reaches the bar where a user can show a generated screenshot to a colleague and be asked where the photograph was taken.
