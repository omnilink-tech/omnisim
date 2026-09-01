# OmniSim Agent Map

This file is written for coding agents and for humans working in an "agent-like" mode. It is intentionally direct and operational.

> ✅ **WREN was DELETED on 2026-08-23** (commit `976b9449d`: `src/wren` + `include/wren` + `src/omnisim/wren`, ~31k lines). wgpu-native is the only renderer, and it is compiled **into the engine** from `src/omnisim/render/` (plus `src/omnisim/nodes/OmWgpuSceneRenderer.*` for camera-family devices) — there is no separate renderer subsystem. Any `src/wren` path in an older doc or session memory is dead. (Banner added 2026-09-01.)

## Source of Truth

Edit first:
- `src/omnisim`: simulator product code (the wgpu renderer lives inside it, under `src/omnisim/render/`)
- `src/controller`: controller runtime and APIs
- `tests`: validation inputs and harnesses
- `resources`: runtime assets and metadata

Treat as reference-heavy or low-priority for normal code search:
- `docs/guide`, `docs/reference` (the imported Webots manuals)
- `src/glm`
- `src/stb`
- generated/vendor documentation trees under vendored code

Do not edit directly unless the task is explicitly about vendored code:
- `src/glm`
- `src/stb`

## Read Before Editing

If the task is about:

- faster local iteration or smaller rebuilds: read `build-and-iteration.md` and `header-hygiene-and-rebuild-reduction.md`
- checking or comparing build-loop speed: read `archive/build-baselines.md` (2026-04-11 snapshot)
- subsystem ownership or file placement: read `architecture.md` and `module-dependency-map.md`
- runtime throughput and benchmark strategy: read `performance-handbook.md` and `profiling-playbook.md`
- PROTO regeneration or DEF/USE behavior: read `template-regeneration-and-dictionary-coherence.md`
- controller IPC or step scheduling: read `controller-ipc-and-step-loop.md`
- startup, reset, or asset wait behavior: read `startup-reset-and-asset-lifecycle.md`
- telemetry and benchmark fidelity: read `observability-and-performance-telemetry.md`
- test harness structure and scenario targeting: read `test-harness-and-scenario-architecture.md`
- contact-heavy physics or instability: read `physics-contact-and-collision-complexity.md`
- texture uploads, overlays, or memory churn: read `asset-pipeline-and-world-quality.md`
- scene-tree invalidation, selection churn, or desktop-shell responsiveness: read `scene-tree-selection-and-runtime-mutation.md`
- risky cross-cutting optimization work: read `performance-anti-patterns.md`
- CI and fast feedback design: read `ci-and-fast-feedback.md`

## Search Strategy

If the task is about:

### startup, application mode, main window
Search:
- `src/omnisim/gui/main.cpp`
- `src/omnisim/gui/OmGuiApplication.*`
- `src/omnisim/gui/OmMainWindow.*`

### simulation stepping, pause/reset, timing
Search:
- `src/omnisim/engine/OmSimulationWorld.*`
- `src/omnisim/control/OmControlledWorld.*`
- `src/omnisim/nodes/OmWorldInfo.*`

### world loading, parser, PROTO, template regeneration
Search:
- `src/omnisim/vrml/OmTokenizer.*`
- `src/omnisim/vrml/OmParser.*`
- `src/omnisim/vrml/OmNodeReader.*`
- `src/omnisim/vrml/OmNode.*`
- `src/omnisim/nodes/utils/OmTemplateManager.*`
- `src/omnisim/nodes/utils/OmWorld.*`

### DEF/USE dictionary and scene mutation coherence
Search:
- `src/omnisim/nodes/utils/OmDictionary.*`
- `src/omnisim/nodes/utils/OmTemplateManager.*`
- `src/omnisim/scene_tree/OmSceneTree.*`
- `src/omnisim/scene_tree/OmSceneTreeModel.*`

### controller API or extern controller behavior
Search:
- `src/omnisim/control/OmController.*`
- `src/controller/launcher/omnisim_controller.c`
- `src/controller/c`
- `src/controller/cpp`
- `include/controller`

### contact-heavy physics, instability, and solver pressure
Search:
- `src/omnisim/engine/OmSimulationCluster.*`
- `src/omnisim/nodes/OmWorldInfo.*`
- `src/omnisim/nodes/utils/OmMassChecker.*`

### rendering, sensors, visual effects
Search:
- `src/omnisim/render` (the wgpu backend: surfaces, targets, shaders, mesh/texture caches)
- `src/omnisim/nodes/OmWgpuSceneRenderer.*` (offscreen rendering for Camera/RangeFinder/Lidar devices)
- `src/omnisim/gui/OmView3D.*`

### texture lifetime, overlays, and memory-heavy rendering paths
Search:
- `src/omnisim/nodes/OmImageTexture.*`
- `src/omnisim/nodes/OmBackground.*`
- `src/omnisim/nodes/OmCamera.*`
- `src/omnisim/render/OmWgpuTextureCache.*` and `src/omnisim/render/OmWgpuImageAdapter.*`

### scene tree/editor behavior
Search:
- `src/omnisim/scene_tree`
- `src/omnisim/editor`
- `src/omnisim/widgets`

### build graph, make targets, workflow surface
Search:
- `Makefile`
- `src/omnisim/Makefile`
- `scripts/dev/omnisim_dev.py`
- `.github/workflows`
- `docs/developer/build-and-iteration.md`
- `docs/developer/ci-and-fast-feedback.md`

## Preferred Fast Paths

### Build
- full product: `python scripts/dev/omnisim_dev.py build all`
- core/runtime-oriented path: `python scripts/dev/omnisim_dev.py build core`
- renderer: there is no separate renderer build any more — `build renderer` refuses with an explanation (the wgpu backend compiles into the engine, so use `build core` / `build gui`)
- desktop shell: `python scripts/dev/omnisim_dev.py build gui`
- controller libs: `python scripts/dev/omnisim_dev.py build controller-libs`

### Validate
- fast smoke suite: `python scripts/dev/omnisim_dev.py test-smoke`
- one existing test group: `python scripts/dev/omnisim_dev.py test-group api`
- one world: `python scripts/dev/omnisim_dev.py test-world tests/api/worlds/accelerometer.omniworld`
- one supported headless world run: `python scripts/dev/omnisim_dev.py run-headless tests/api/worlds/accelerometer.omniworld`
- performance log for one world: `python scripts/dev/omnisim_dev.py profile-world tests/rendering/worlds/normals.omniworld`

## Safe Assumptions

- `OMNISIM_HOME` is still the canonical root environment variable.
- `tests/test_suite.py` remains the main integration harness.
- logical targets such as `sim-core` are wrappers over the legacy make graph, not proof of deep modularization.

## High-Risk Areas

- `src/omnisim/control/OmController.*`
- `src/omnisim/control/OmControlledWorld.*`
- `src/omnisim/vrml/*`
- `src/omnisim/nodes/utils/OmWorld.*`
- `src/omnisim/nodes/utils/OmTemplateManager.*`
- rendering/runtime interactions between `src/omnisim/engine` and `src/omnisim/render`
- broad headers such as `src/omnisim/vrml/OmNode.hpp`, `src/omnisim/nodes/OmBaseNode.hpp`, and `src/omnisim/engine/OmSimulationWorld.hpp`

These areas affect broad behavior. Prefer targeted validation plus smoke coverage after edits.
