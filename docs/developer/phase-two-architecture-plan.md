# OmniSim Phase Two: Architecture Deep Dive and Later Implementation Plan

## Purpose
This document goes deeper than the phase-one roadmap. It describes the simulator's current architecture, the constraints that shape any realistic refactor, and the concrete phase-two implementation plan to pursue after the first tooling and contributor-experience improvements land.

For milestone sequencing and execution order, read [phase-two-execution-program.md](phase-two-execution-program.md) together with this document.

The goal of phase two is not to replace the simulator. The goal is to split the existing product into clearer operational layers so that:
- simulation work can run headlessly without dragging the desktop shell into every workflow
- rendering work can be changed and measured in isolation
- controller/runtime protocols become easier to reason about and test
- coding agents can make narrower, safer changes with faster validation

## Current Architecture

### 1. Product shell and process startup
The shipped simulator is primarily assembled inside `src/omnisim`.

The current binary is not just a "core simulator" with an optional UI layered on top. The main target combines:
- Qt application lifecycle and startup task routing
- desktop windows and editor features
- scene tree and property editing
- simulation stepping and world state
- controller orchestration
- rendering integration and sensor rendering
- packaging assumptions and platform-specific launchers

Important signals from the current layout:
- `src/omnisim/gui/main.cpp` is the executable entrypoint.
- `src/omnisim/gui/WbGuiApplication.*` owns task dispatch such as normal run, conversion, sysinfo, version, and update flows.
- `src/omnisim/Makefile` compiles a single large product target from `app`, `control`, `core`, `editor`, `engine`, `gui`, `maths`, `nodes`, `plugins`, `scene_tree`, `sound`, `user_commands`, `vrml`, and `wren`.

This means phase two should avoid pretending there is already a clean headless core. There is not. The headless path has to be extracted from a product-oriented binary.

### 2. World model, parser, and node graph
The world pipeline is centered on the VRML/PROTO stack and the `WbWorld` object model.

Key pieces:
- `src/omnisim/vrml/WbTokenizer.*`: token stream and file/token error reporting
- `src/omnisim/vrml/WbParser.*`: syntax parsing
- `src/omnisim/vrml/WbNodeReader.*`: node creation during world loading
- `src/omnisim/vrml/WbNode.*`: base node class, field reading, cloning, export, reset, PROTO awareness
- `src/omnisim/vrml/WbProtoManager.*`: EXTERNPROTO / PROTO registry and lookup
- `src/omnisim/nodes/utils/WbTemplateManager.*`: template regeneration control
- `src/omnisim/nodes/utils/WbWorld.*`: world loading, root creation, saving, reset, robot registry, W3D export

Important architectural properties:
- world loading is tightly coupled to application progress reporting
- node creation and finalization are intertwined with PROTO/template regeneration
- asset resolution, caching, and export logic are mixed into node/world infrastructure
- `WbWorld` is already the effective simulation state root, but it is still product-aware

This layer is structurally important because it is the only realistic seam for a future simulation-core API.

### 3. Simulation engine and step orchestration
Runtime stepping is centered on:
- `src/omnisim/engine/WbSimulationWorld.*`
- `src/omnisim/control/WbControlledWorld.*`
- `src/omnisim/nodes/WbRobot.*`
- `src/omnisim/nodes/WbWorldInfo.*`
- physics support in `src/ode`

The current split is roughly:
- `WbWorld`: world graph, load/save/reset, root ownership
- `WbSimulationWorld`: simulation timer, physics stepping, plugin stepping, camera render triggers, reset behavior
- `WbControlledWorld`: controller lifecycle, synchronization, request/answer flow, extern controller connection state

Important details visible in the current code:
- assets are downloaded during world setup before normal simulation flow
- physics plugins are loaded per world
- step execution is tied to Qt event/timer behavior
- controller orchestration can pause or delay stepping depending on synchronization state
- camera textures are updated from the simulation layer before main rendering

This is the actual runtime heart of the simulator. Any phase-two plan should treat it as the main extraction candidate for a supported headless runtime.

### 4. Controller runtime and protocol boundary
Controller support is spread across both `src/omnisim/control` and `src/controller`.

The split today is:
- `src/omnisim/control/WbController.*`: process launch, IPC/TCP server setup, request parsing, answers, stdout/stderr forwarding, extern controller lifecycle
- `src/controller/launcher/webots_controller.c`: standalone launcher for external controllers
- `src/controller/c/*`: C controller runtime, device APIs, request/answer protocol implementation
- `src/controller/cpp/*`: C++ wrapper over the C runtime

Important current constraints:
- the controller protocol is stateful and deeply integrated with the simulator step loop
- local IPC and remote TCP are both first-class runtime paths
- the supervisor API is large, stateful, and tightly coupled to world internals
- controller logs are funneled back into simulator logging/UI flows

This is one of the hardest areas for agents because the boundary is real but not small. Phase two should make this boundary more explicit before trying to redesign it.

### 5. Rendering stack
Rendering is layered in two parts:
- low-level renderer in `src/wren`
- simulator integration and effect orchestration in `src/omnisim/wren`

The shape today:
- `src/wren`: renderer objects, textures, materials, frame buffers, scene graph, shader programs, mesh types
- `src/omnisim/wren`: simulator-facing integration such as camera, bloom, SMAA, depth of field, HDR, GTAO, lens effects, overlays, picker, rendering context
- `src/omnisim/gui/WbView3D.*` and related classes tie rendering into the desktop UI

Important current properties:
- the renderer has its own object graph and cache behavior
- sensor rendering and main viewport rendering are not independent concerns
- rendering modes and optional passes are controlled through a shared rendering context
- world and node code often know too much about rendering side effects

This suggests that phase two should not start with a graphics rewrite. It should define a narrower simulator-to-renderer contract and isolate sensor rendering from desktop rendering where possible.

### 6. Desktop shell and editing surface
A large part of `src/omnisim` is the editor/product shell:
- `gui`
- `scene_tree`
- `editor`
- `widgets`
- parts of `user_commands`

This layer owns:
- main window composition
- scene tree editing and field editors
- project and world editing UX
- dialogs, guided flows, wizards
- undo/redo and many direct edit operations

This is valuable product functionality, but it is also the main reason the simulator remains difficult to use as a narrow programmable runtime. Phase two should make this layer a clearer consumer of the simulation/runtime layer instead of a peer mixed into it.

### 7. Assets, projects, and test corpus
The repository contains far more than engine code:
- `projects`: sample worlds, robots, objects, assets, plugins
- `resources`: runtime assets and packaging-time resources
- `tests`: controller-based and world-based validation across API, physics, rendering, parser, cache, and PROTOs
- `docs`: product docs, historical release docs, and large archival content

This is a strength for product coverage, but it also means:
- repo navigation has a high noise floor
- the build/test system naturally skews toward broad whole-product runs
- many simulation changes require judgment about assets, samples, and docs side effects

Phase two should not try to eliminate this breadth. It should make source-of-truth boundaries much clearer.

## Architectural Problems to Solve in Phase Two

### 1. There is no supported simulation-core API
The closest thing to a core runtime exists, but it is not presented as a stable layer. The effective core is spread across `WbWorld`, `WbSimulationWorld`, `WbControlledWorld`, controller protocol code, and node internals.

Impact:
- headless automation depends on product behavior, not a narrow runtime contract
- agents must understand too many classes before they can safely automate a scenario
- testing stays expensive because narrow runtime workflows are not official

### 2. The world model is overloaded
The node/PROTO/template system owns parsing, instantiation, regeneration, export, field aliasing, and parts of asset resolution. It is powerful, but the responsibilities are mixed.

Impact:
- parsing and runtime mutation are difficult to separate
- serialization/export changes are risky
- phase-two headless APIs will be brittle unless world-loading concerns are made more explicit

### 3. Controller lifecycle is too entangled with simulation stepping
`WbControlledWorld` and `WbController` jointly manage synchronization, process startup, request buffering, extern connection state, and parts of runtime retry logic.

Impact:
- step semantics are hard to reason about in isolation
- deterministic headless stepping is harder than it should be
- controller-related bugs require knowledge of both simulator and runtime protocol internals

### 4. Rendering and simulation are coupled in both directions
Simulation step code triggers render-relevant work, while nodes and product code often encode rendering assumptions directly.

Impact:
- "no rendering" modes still carry more product coupling than desirable
- sensor rendering and main viewport rendering are difficult to measure independently
- renderer refactors are higher risk because contracts are implicit

### 5. Desktop concerns are mixed into runtime concerns
World loading progress, UI task flows, editing state, and application-level tasks are woven into runtime classes.

Impact:
- hard to run the simulator as a library-like runtime
- difficult to produce a clean agent-facing CLI surface
- more of the codebase becomes "critical path" for simple automation

## Phase Two Target Architecture

Phase two should move the codebase toward four explicit layers.

### Layer A: World and model layer
Responsibilities:
- tokenize, parse, and validate world/PROTO input
- instantiate the node graph
- track node metadata, fields, PROTO lineage, and serialization
- provide load/save/reset semantics without depending on desktop UI

Likely owners:
- `vrml`
- `nodes`
- selected utilities currently in `nodes/utils`

Rules:
- no Qt desktop widgets
- no renderer-specific side effects during parsing
- progress reporting through callbacks or signals that do not require the desktop shell

### Layer B: Simulation runtime layer
Responsibilities:
- simulation clock and mode handling
- physics stepping and world updates
- controller scheduling and synchronization
- plugin stepping
- deterministic/headless run loop

Likely owners:
- `engine`
- `control`
- subset of `nodes` behavior related to step/update

Rules:
- owns the authoritative `step()` contract
- can run without the desktop shell
- exposes a narrow programmatic interface for load, configure, step, pause, reset, and teardown

### Layer C: Rendering layer
Responsibilities:
- Wren scene graph and GPU resources
- viewport rendering
- sensor rendering
- post-processing
- object picking and overlays

Likely owners:
- `src/wren`
- `src/omnisim/wren`

Rules:
- consumes world/runtime state through explicit integration points
- can be switched off in headless mode cleanly
- separates main viewport rendering from sensor rendering where possible

### Layer D: Desktop shell layer
Responsibilities:
- main window
- scene tree and editors
- project workflows
- dialogs and wizards
- developer UX around the simulator

Likely owners:
- `gui`
- `scene_tree`
- `editor`
- `widgets`
- selected `user_commands`

Rules:
- uses the runtime layer instead of co-owning it
- no hidden dependencies on direct internal world state mutation where a runtime API would suffice

## Phase Two Implementation Plan

### Workstream 1: Establish a real simulation-runtime boundary
Deliverables:
- Define a `simulation runtime` interface around world lifecycle and stepping.
- Make `WbSimulationWorld` and `WbControlledWorld` the initial implementation behind that interface.
- Introduce an explicit headless runner entrypoint that loads a world, configures runtime mode, steps, and exits without building the desktop shell into the contract.

Implementation direction:
- extract a runtime-facing service object rather than renaming everything
- keep the current classes, but wrap them behind a smaller orchestrator
- make load, step, reset, controller synchronization, and teardown explicit API calls

Why this comes first:
- it is the prerequisite for faster tests, stable automation, and a true agent-facing CLI

### Workstream 2: Untangle world loading from product UI
Deliverables:
- Move world-loading progress reporting behind callbacks/signals owned by the runtime layer
- reduce direct dependence on `WbApplication::instance()` from load/parsing paths
- document the world-loading phases: tokenize, parse, instantiate, finalize, asset download, runtime-ready

Implementation direction:
- preserve existing UI behavior by adapting desktop code to the new callbacks
- avoid changing parsing semantics in the first pass

Why this matters:
- headless workflows cannot be first-class until world loading stops assuming desktop application state

### Workstream 3: Formalize controller protocol and lifecycle state
Deliverables:
- document the controller state machine: created, waiting, running, disconnected extern, terminating
- isolate protocol framing/parsing logic from process-launch concerns where possible
- create narrower tests for controller lifecycle and request/answer framing without requiring full product tests

Implementation direction:
- keep IPC/TCP support
- split "controller transport/protocol" from "controller process management" conceptually and in code where feasible
- add structured debug logging for controller state transitions

Why this matters:
- controller/runtime complexity is one of the highest-risk areas for simulator regressions and one of the hardest for agents to reason about

### Workstream 4: Separate sensor rendering from desktop rendering paths
Deliverables:
- inventory where simulation step code triggers rendering side effects
- define clear APIs for:
  - render sensors
  - render main viewport
  - update renderer state from world state
- benchmark sensor-heavy worlds independently from viewport-heavy worlds

Implementation direction:
- keep Wren as the renderer
- focus on boundaries and measurement, not feature removal
- avoid breaking current rendering modes while clarifying ownership

Why this matters:
- rendering cost is currently hard to attribute cleanly
- future performance work depends on better separation

### Workstream 5: Reduce mutation coupling in the desktop shell
Deliverables:
- identify editor and scene-tree operations that directly mutate deep runtime state
- route those operations through clearer world/runtime services where practical
- document "safe extension seams" for new tools and automation

Implementation direction:
- prioritize high-frequency edit flows and supervisor-related workflows
- do not try to redesign the whole editor in one phase

Why this matters:
- the simulator will remain hard to maintain if the UI continues to share too much implicit ownership with runtime code

### Workstream 6: Introduce benchmark-grade scenario coverage
Deliverables:
- canonical benchmark worlds for:
  - startup
  - empty world load
  - sensor-heavy load/frame
  - physics-heavy stepping
  - large asset load/memory
- a stable benchmark runner and output format
- CI thresholds or trend reporting for the most stable metrics

Implementation direction:
- start with existing worlds from `resources/projects` and `tests`
- avoid synthetic benchmarks that do not reflect real simulator behavior

Why this matters:
- phase two needs proof that refactors improve or preserve runtime cost

## Recommended Phase Two Milestones

### Milestone 1: Headless runtime contract
- define the runtime interface
- add a supported headless runner
- migrate smoke-style automation to use it

### Milestone 2: World-loading cleanup
- remove direct desktop assumptions from parsing/finalization progress paths
- document and test the load pipeline

### Milestone 3: Controller boundary cleanup
- document lifecycle and protocol
- improve protocol/lifecycle tests
- reduce mixed responsibilities in controller orchestration

### Milestone 4: Rendering boundary cleanup
- separate sensor and viewport rendering pathways conceptually and in benchmarks
- clarify simulator-to-renderer contract

### Milestone 5: Desktop shell decoupling
- move common edit flows onto clearer runtime services
- reduce deep cross-layer mutation paths

## What Phase Two Should Not Do
- Do not replace ODE, Wren, or the controller APIs in this phase.
- Do not migrate every build path at once.
- Do not attempt a full C++ package/module reorganization before the runtime boundaries are defined.
- Do not rewrite PROTO/template behavior without benchmark and compatibility coverage.

## Later Implementation Backlog

### A. Runtime API surface
- `loadWorld(path, options)`
- `start(mode, headlessOptions)`
- `step(count or duration)`
- `pause()`
- `reset(restartControllers)`
- `shutdown()`
- structured event stream for warnings, controller messages, and step completion

### B. Narrow architecture docs to create later
- `docs/developer/quickstart.md`
- `docs/developer/architecture.md`
- `docs/developer/agent-map.md`
- controller protocol state machine doc
- world loading pipeline doc
- rendering pipeline doc

### C. Test additions to schedule later
- world-load contract tests
- controller transport/lifecycle tests
- headless runtime smoke tests
- benchmark worlds with stored baselines or trends

## Decision Summary
Phase one should improve usability around the current monolith. Phase two should make the monolith easier to split operationally by extracting an explicit simulation runtime, clarifying world-load and controller boundaries, and making rendering and desktop concerns better-behaved consumers of that runtime.
