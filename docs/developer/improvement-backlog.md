# OmniSim Improvement Backlog

This is the actionable backlog for improving simulator performance, visual quality, correctness, and developer iteration speed.

It is intentionally organized by impact and dependency rather than by team boundary.

> **Freshness:** items below were drafted as a rolling backlog and individual entries may have been partially or fully implemented since they were written. Before picking one up, grep the named files to confirm the underlying problem still exists. When you finish an item, delete its entry rather than marking it done — the live code is the record.

## How To Use This Backlog

Each item is phrased as a change slice that could become an issue, milestone, or implementation task.

When choosing work:

- prefer items that improve measurement and narrow fast paths first
- prefer changes that reduce coupling before changes that only optimize a coupled path
- validate each item against representative worlds

## Now

These are high-value, low-to-medium risk improvements that should be possible within the current architecture.

### 2. Fix the COLD-FIRST-LOAD articulation-tracking bug (Newton/MuJoCo)

Area:

- `src/omnisim/physics/WbNewtonBackend.cpp` (finalize / world build ordering)
- `src/omnisim/nodes/WbSolid.cpp` (ODE body-disable + `setSolverPreference` timing)

Why:

On a COLD first world load the Newton/MuJoCo articulation under-tracks position
targets — an arm undershoots its commanded IK pose by ~1 cm — so precise-contact tasks
(grasps, pinches, insertions) can FAIL on the first load yet work perfectly after a
world reload. Every headless run is a cold load, so this silently produced wrong
"the physics can't do this" conclusions and burned a lot of time (full story:
[real-grasp-and-the-cold-first-load-trap.md](real-grasp-and-the-cold-first-load-trap.md)).
Confirmed: rebuilding only the `SolverMuJoCo` (at finalize OR after N steps) does NOT
fix it; only a full world rebuild does — so it is the broader C++ world build, suspect
the ODE↔Newton body-disable ordering / node finalize on first load. Related GUI-only
symptom: the first build can fall back to `SolverXPBD` because the `newtonSolver
"mujoco"` preference is plumbed from `WbSolid`'s Newton flush and can arrive after the
solver is built. Current mitigation (not a root fix): `omnilink_arm_bridge.warmup_reload`
reloads once at controller startup, and `headless_runner.py` defaults to warm. A real
fix removes the need for the warm-up reload entirely.

### 3. Add one explicit sensor-heavy benchmark world

Area:

- `tests/benchmarks`

Why:

- current benchmark coverage is small and only approximates sensor-heavy cost

### 6. Add a documented deterministic-vs-throughput benchmark note

Area:

- docs and benchmark output conventions

Why:

- physics multithreading already has known determinism tradeoffs

### 7. Profile and document contact-heavy worlds separately

Area:

- `tests/benchmarks`
- docs

Why:

- contact complexity is a different bottleneck from pure rigid-body stepping

### 9. Add a world-load warning budget for core benchmark worlds

Area:

- tests and docs

Why:

- benchmark worlds should be warning-light and stable

### 10. Add a developer-facing “world load profile” command shape later

Area:

- developer CLI

Why:

- contributors need a short path for load-focused work, not just generic performance logging

## Next

These items are still high-value but depend on better measurement or some cleanup first.

### 11. Narrow template regeneration instrumentation

Area:

- `src/omnisim/nodes/utils/WbTemplateManager.*`

Why:

- regeneration is broad and signal-driven; cost is not visible enough today

### 12. Make dictionary updates more transactional

Area:

- `src/omnisim/nodes/utils/WbDictionary.*`
- `src/omnisim/nodes/utils/WbNodeOperations.*`

Why:

- regeneration and DEF/USE maintenance can amplify one another

### 13. Reduce repeated descendant scans in node utilities

Area:

- `src/omnisim/nodes/utils/WbNodeUtilities.*`

Why:

- the code explicitly documents at least one very slow path

### 14. Add a cheap visibility or scene-query cache for node filtering

Area:

- `src/omnisim/nodes`
- `src/omnisim/nodes/utils`

Why:

- some visibility-sensitive logic still pays for broad graph walks

### 15. Clarify or remove questionable runtime and physics mutexes

Area:

- `src/omnisim/nodes/utils/WbWorld.*`
- `src/omnisim/engine/WbSimulationCluster.*`

Why:

- throughput work is risky while lock ownership remains ambiguous

### 16. Add a first-frame render metric

Area:

- performance logging
- main view startup

Why:

- current load and render metrics do not separate “world loaded” from “first useful frame rendered”

### 17. Move background and irradiance rotation to the GPU

Area:

- `src/omnisim/nodes/WbBackground.*`
- renderer shader path

Why:

- the code explicitly calls out the current CPU-side path as a performance problem

### 18. Reduce CPU-side overlay and depth conversion work

Area:

- `src/omnisim/wren/WbWrenTextureOverlay.*`
- camera and display paths

Why:

- `gpuMemoryTransfer` remains a critical metric in device-heavy worlds

### 19. Add better recognition candidate filtering

Area:

- `src/omnisim/nodes/WbCamera.*`
- world recognition-target management

Why:

- camera recognition still scales too directly with scene content

### 20. Add an asset-quality checklist to world review

Area:

- docs
- contributor workflow

Why:

- asset quality and simulation quality are tightly linked

## Later

These items are larger architectural changes or follow-on work after the current fast paths and measurements are solid.

### 21. Introduce a real simulation-core boundary

Area:

- runtime architecture

Why:

- headless automation should not depend on the full desktop shell forever

### 22. Reduce the multi-pass world-load model

Area:

- tokenizer, parser, node reader, world construction

Why:

- current load work is safe but likely duplicative for large worlds

### 23. Separate sensor rendering from desktop rendering policy

Area:

- renderer architecture

Why:

- sensor correctness and desktop visual quality should not be one policy surface

### 24. Replace broad singleton coupling with narrower service boundaries

Area:

- runtime, renderer, application, world, preferences, logging

Why:

- global access patterns make hot-path reasoning and isolated testing harder

### 25. Turn logical developer targets into true build targets

Area:

- build system

Why:

- `sim-core`, `sim-gui`, and `renderer` should eventually represent real modular build boundaries rather than wrappers

### 26. Add machine-readable benchmark comparison tooling

Area:

- tests
- CI

Why:

- performance regressions should be easier to compare automatically

### 27. Add memory-footprint benchmarks for large worlds

Area:

- performance testing

Why:

- load time is not the only cost that scales poorly with scene complexity

### 28. Add better renderer debug and pass diagnostics

Area:

- WREN
- simulator integration

Why:

- developers need a clearer picture of where render time goes

### 29. Add contact- and ray-count diagnostics for representative worlds

Area:

- runtime instrumentation

Why:

- many “slow” worlds are slow for content reasons that are not obvious without better counters

### 30. Add a better contributor handbook for world authors

Area:

- docs

Why:

- much of the simulator’s practical performance depends on how worlds are authored, not only on engine code

### 31. Add controller wait, retry, and packet-pressure telemetry

Area:

- controller runtime
- performance logging

Why:

- per-controller timing exists, but the runtime still does not show how much time is lost waiting for controllers, how often retries happen, or how much packet pressure is being generated

### 32. Separate asset wait time from world construction and reset timing

Area:

- startup path
- reset path
- asset lifecycle

Why:

- asset download still shapes startup and reset behavior, which weakens benchmark trust and the deterministic headless story

### 33. Split `tests/test_suite.py` into discovery, execution, and reporting layers

Area:

- test harness
- CI
- developer workflow

Why:

- the newer smoke and benchmark wrappers are thin, but the core runner is still broad and file-state heavy, making one-world validation harder than it should be

### 34. Add desktop-shell transaction and invalidation metrics

Area:

- scene tree
- desktop shell
- telemetry

Why:

- current measurement is strong on simulation and rendering, but weak on scene-tree churn, selection restore work, and UI transaction cost

### 35. Add first-frame and controller-heavy benchmark scenarios

Area:

- benchmarks
- profiling

Why:

- the current benchmark set still lacks explicit coverage for first-frame latency and controller-heavy runtime coupling

### 36. Add contact truncation and collision-complexity diagnostics

Area:

- physics
- profiling

Why:

- the collision path already warns when only the deepest contact points can be kept, but contributors still lack aggregate counters for raw contacts, retained joints, and truncation frequency

### 37. Make template regeneration and dictionary updates more transactional

Area:

- PROTO regeneration
- dictionary maintenance
- runtime mutation

Why:

- regeneration already behaves like a coarse transaction system, but the current contract still depends on broad blocking flags, nested-update avoidance, and some full recomputation paths

### 38. Reduce scene-tree `layoutChanged()` usage and selection churn

Area:

- desktop shell
- scene tree

Why:

- value updates still trigger broad layout invalidation in some paths, and selection clearing remains a frequent recovery mechanism during regeneration and runtime mutation

### 39. Replace global texture-lifetime scans with narrower residency accounting

Area:

- rendering
- memory
- texture pipeline

Why:

- texture lifetime is currently spread across node ownership, WREN cache behavior, and the CPU-side `gImagesMap`, making memory behavior harder to reason about and optimize

### 40. Batch recognition-overlay and other partial texture updates

Area:

- sensors
- GPU transfer
- overlays

Why:

- camera recognition and overlay paths still perform repeated texture clears and region updates, which makes sensor-heavy worlds pay avoidable GPU-transfer cost

### 41. Make environment-map and irradiance transforms GPU-side

Area:

- rendering
- startup
- asset pipeline

Why:

- `WbBackground.cpp` still contains explicit `FIXME` markers for CPU-side texture and HDR irradiance rotation that should move into OpenGL or shader-side handling

## Candidate Milestone Packs

### Milestone A: Better measurement

- items 3, 6, 7, 16

### Milestone B: Better world quality

- items 9, 20, 30

### Milestone C: Better runtime hot paths

- items 13, 14, 15, 19

### Milestone D: Better rendering efficiency

- items 17, 18, 28

### Milestone E: Better architecture

- items 21, 22, 23, 24, 25

## Finish Criteria For Backlog Items

An item is only complete when:

- the code change lands
- the relevant docs are updated
- the narrowest appropriate validation path is documented
- before/after impact is measured when the item is performance-related

That keeps the simulator getting better in a way contributors can sustain.
