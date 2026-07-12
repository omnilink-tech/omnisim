# Simulation Authoring For Coding Agents

This guide covers the contributor surface that matters when a human or coding agent is trying to build a new simulation, extend an existing one, or reduce the amount of repo knowledge required to do that safely.

It focuses on two questions:

- what is the best supported workflow today
- what should the simulator expose next so world authoring becomes cheaper, more deterministic, and more machine-checkable

Use this together with:

- [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md) for local assets, texture discipline, and sample-world quality
- [test-harness-and-scenario-architecture.md](test-harness-and-scenario-architecture.md) for scenario manifests and validation lanes
- [physics-contact-and-collision-complexity.md](physics-contact-and-collision-complexity.md) for contact-heavy worlds and instability patterns
- [urdf-import-debugging.md](urdf-import-debugging.md) when the simulation starts from imported robot structure rather than hand-authored Webots nodes
- [phase-two-architecture-plan.md](phase-two-architecture-plan.md) for the longer runtime-boundary work that makes a true agent-facing authoring surface possible

## Why This Matters

Much of the simulator's real value is created by adding or evolving:

- worlds
- PROTOs
- controllers
- assets
- scenario manifests

That work is often harder for coding agents than changing an isolated C++ function.

The main reasons are structural:

- the authoring contract is spread across parser, node, template, asset, controller, and test-harness code
- ~~important failures still surface as broad log text rather than stable machine-readable diagnostics~~ — **partially addressed by the [agent-facing validation harness](../../scripts/harness/README.md)**: `POST /world/load` returns structured diagnostic codes (`PROTO_NOT_FOUND`, `WORLD_PARSE_SYNTAX_ERROR`, `TEXTURE_READ_FAILED`, …) instead of free-text stderr
- ~~the best headless path is still a mode of the product, not a first-class authoring API~~ — **addressed by the harness**: long-running HTTP service on `127.0.0.1:6789` with `/world/load`, `/world/screenshot`, `/scene/tree`, `/scene/look_at`, `/world/render_stats`, `/sim/step`, and ~600 ms hot reload via `Supervisor.worldLoad`. See [AGENTS.md §5](../../AGENTS.md#5-iterating-on-worlds-with-the-validation-harness) for the day-to-day workflow.
- content quality rules are documented in several places, but only a subset are enforced automatically

If OmniSim becomes easier for coding agents to build simulations with, it also becomes easier for human contributors to:

- create reliable repro cases
- add stable benchmarks
- build smaller sample worlds
- debug controller or asset problems without opening the full desktop shell first

## Best Supported Workflow Today

Today the safest authoring loop is:

1. Start from an existing small world or PROTO that is already local-asset only.
2. Keep the first change narrow: one robot, one controller, one sensor path, or one asset decision at a time.
3. Validate with the repo wrappers before widening scope:
   `python scripts/dev/omnisim_dev.py run-headless <world>`
   `python scripts/dev/omnisim_dev.py test-world <world>`
   `python scripts/dev/omnisim_dev.py profile-world <world>`
4. Use the desktop shell only for questions that actually need it:
   main-view rendering
   scene-tree behavior
   editor interaction
5. For imported robots, generate a preflight report first with `scripts/dev/urdf_import.py --report --strict` before relying on runtime behavior.
6. Keep smoke and benchmark candidates warning-light, deterministic, and free of remote assets.
7. Give the world a camera that frames its subject — don't eyeball the `Viewpoint`. Generated worlds get this automatically; for hand-authored worlds use `scripts/dev/set_viewpoint.py`. See [viewpoint-convention.md](viewpoint-convention.md).

That is workable, but it still assumes the author understands several hidden rules:

- which warnings are cosmetic versus behavior-changing
- whether a scenario truly needs desktop rendering
- whether a missing asset will distort startup or reset timing
- whether a controller problem is actually a world-load or runtime contract problem

## What Coding Agents Need

An agent can build simulations reliably only if the simulator provides a small number of explicit contracts.

Those contracts should include:

- a stable way to load or lint a world without launching the entire desktop workflow
- structured diagnostics with warning codes, severity, source location, and affected node or asset path
- scenario metadata that says whether a world needs headless stepping, sensor rendering, full desktop rendering, or controller synchronization
- predictable exit codes and structured run results
- an easy way to see imported or generated structure before debugging steady-state runtime behavior
- templates or starter scenarios that are intentionally small and local

Without these, the agent has to infer meaning from broad text logs and from code spread across too many subsystems.

## Current Friction In The Codebase

### 1. The authoring contract is distributed across too many layers

Building or changing a simulation can involve all of these at once:

- `src/omnisim/vrml/WbTokenizer.*`
- `src/omnisim/vrml/WbParser.*`
- `src/omnisim/vrml/WbNodeReader.*`
- `src/omnisim/nodes/utils/WbWorld.*`
- `src/omnisim/nodes/utils/WbTemplateManager.*`
- `src/omnisim/engine/WbSimulationWorld.*`
- `src/omnisim/control/WbControlledWorld.*`
- `tests/test_suite.py`
- `tests/smoke/*`
- `tests/benchmarks/*`

That breadth is manageable for a long-time maintainer. It is expensive for an agent trying to answer a narrower question such as:

- did the world load correctly
- is the imported structure faithful
- is the controller handshake correct
- is the scenario benchmark-worthy

### 2. Headless automation is useful, but still product-shaped

`run-headless` is already the right public direction.

The limitation is that it still depends on the main simulator process and a CLI flag bundle rather than a smaller runtime contract. The supported flags are useful:

- `--mode=fast`
- `--batch`
- `--no-rendering`
- `--stdout`
- `--stderr`
- `--minimize`

But they do not yet give agents a first-class answer to simpler authoring questions such as:

- parse this world and report warnings
- expand this PROTO and show the resulting node tree
- resolve this asset set and report remote dependencies
- run exactly one scenario and emit structured results

### 3. Diagnostics are still too text-heavy

The new URDF report is a good example of the direction OmniSim should take more broadly.

Most other authoring failures still require scraping:

- console output
- `omnisim_log.txt`
- ad hoc test-harness files
- warning text with unstable wording

That creates avoidable failure modes:

- agents overfit to exact message strings
- CI jobs cannot easily budget or classify warnings
- the difference between load failure, content warning, and runtime instability is blurred

### 4. Scenario capabilities are not yet explicit enough

`test-harness-and-scenario-architecture.md` already identifies the need for richer scenario manifests.

That matters for authoring because an agent should not have to guess whether a scenario needs:

- pure headless stepping
- sensor rendering only
- full main-view rendering
- desktop-shell behavior
- controller synchronization
- cache-sensitive restart behavior

Without explicit capabilities, world authors either under-validate or accidentally force every change through an expensive path.

### 5. World-quality rules are documented, but only partly enforced

The repo already knows several content rules matter:

- remote assets distort startup and benchmark trust
- overscaled textures create memory and upload cost
- collision complexity and contact count can dominate physics behavior
- unstable controller chatter can distort step timing

Those rules appear in docs, but an agent still has to remember them manually in many cases.

That means the simulator is still too willing to accept a world that is:

- technically loadable
- behaviorally fragile
- expensive to validate
- unsuitable as a smoke or benchmark scenario

### 6. Importers and generated structure expose too little intermediate state

URDF is one example, but the broader pattern matters:

- imported structure should be inspectable before runtime debugging starts
- generated or expanded nodes should be easy to report in machine-readable form
- mutation-heavy scenarios should make regeneration and warning sources obvious

If the only way to understand generated structure is to launch the desktop app and inspect the scene tree manually, the authoring loop is still too human-only.

## Recommended Improvements

### 1. Add a first-class world lint or report command

This is the highest-value near-term improvement.

Even if it remains a wrapper over existing code, the public contract should be explicit:

- input: world path plus optional controller and asset checks
- output: JSON report plus non-zero exit code on selected warning classes
- scope: tokenizer, parser, node creation, template regeneration, asset resolution, scenario capability inference

Useful report fields would include:

- warning code
- severity
- message
- file and line when available
- node path or DEF name when available
- asset paths and whether they are local, cached, or remote
- required scenario capabilities
- imported or generated-structure notes

Likely implementation areas:

- `scripts/dev/omnisim_dev.py`
- `src/omnisim/vrml/*`
- `src/omnisim/nodes/utils/WbWorld.*`
- `src/omnisim/nodes/utils/WbTemplateManager.*`

### 2. Enrich scenario manifests so authoring and validation use the same metadata

The smoke and benchmark manifests should grow beyond a bare world list.

They should declare:

- scenario name
- world path
- category
- capabilities
- expected warning budget
- allowed platforms
- benchmark relevance
- controller expectations

That gives agents a stable answer to "what is the cheapest correct validation lane for this scenario?"

Likely implementation areas:

- `tests/smoke/smoke_worlds.json`
- `tests/benchmarks/benchmark_worlds.json`
- `tests/smoke/run_smoke.py`
- `tests/benchmarks/run_benchmarks.py`
- `tests/test_suite.py`

### 3. Standardize structured run results

One-world execution should be able to produce a compact structured result instead of only console text.

Useful fields:

- exit status
- load success
- elapsed startup time
- elapsed step time
- warning count by code
- controller timeout or retry counters
- asset wait time
- scenario capability used for the run

This would help both humans and agents distinguish:

- content problems
- runtime problems
- harness problems
- environment problems

### 4. Turn documented content rules into automated checks

The docs already describe many authoring hazards. More of them should become optional or default checks in the lint path.

High-value checks:

- remote assets in smoke or benchmark candidates
- warning budgets for benchmark worlds
- oversized textures relative to scenario use
- suspicious collision complexity
- missing or noisy controller configuration for benchmark scenarios

The goal is not to reject every creative world. The goal is to stop fragile scenarios from silently becoming validation infrastructure.

### 5. Extend the importer-report pattern beyond URDF

`urdf-import-debugging.md` establishes a strong precedent:

- preflight report
- strict mode
- machine-readable output
- runtime debug flag for deeper inspection

That same pattern should be used for other authoring flows:

- PROTO expansion and regeneration
- asset dependency reporting
- scenario capability inference
- controller launch and synchronization summaries

### 6. Add intentionally small starter worlds and templates

Agents work best when they can start from examples designed for mutation rather than from large showcase scenes.

The repo should expose a few canonical minimal templates for:

- one rigid-body world
- one controller-only world
- one camera or sensor world
- one supervisor mutation world
- one imported-robot world

Those templates should be:

- local-asset only
- warning-light
- fast to run headlessly
- easy to promote into smoke or benchmark coverage later

### 7. Keep desktop behavior a consumer of runtime, not the authoring gate

A coding agent should not need the GUI to answer routine questions about structure or correctness.

The desktop app should remain necessary for:

- visual inspection
- scene-tree UX
- editor workflows
- interactive debugging

It should not remain the only practical way to:

- inspect generated structure
- classify world-load warnings
- determine which validation lane is required

## Suggested Near-Term Work Sequence

1. Add a `world-report` or `lint-world` developer command with JSON output.
2. Give smoke and benchmark manifests explicit capability metadata.
3. Make one-world runs emit structured summaries in addition to text logs.
4. Add automated checks for remote assets and warning budgets on validation scenarios.
5. Introduce a small library of starter worlds and template controllers aimed at mutation and automation.
6. Extend importer and regeneration debug reports so generated structure is inspectable before GUI launch.

That order improves the authoring loop immediately without waiting for the full phase-two runtime split.

## Phase-Two Direction

The longer architectural goal is not only "better docs for agents." It is a cleaner simulator boundary.

The key phase-two outcomes for simulation authoring are:

- a real runtime surface around load, step, reset, and structured reporting
- clearer separation between world/model concerns and desktop-shell concerns
- controller synchronization that is easier to classify and measure
- rendering capabilities that can be declared and tested separately from the desktop viewport

Until those exist, agent-facing workflows will remain wrappers over a larger product boundary than they should be.

## Exit Criteria

This area is getting better when a coding agent can:

- create or modify a small simulation starting from a documented template
- run a supported lint or report command before launching the simulator
- tell whether the scenario requires headless, sensor, or desktop validation
- receive structured diagnostics for missing assets, unsupported structure, or warning-heavy worlds
- validate one scenario and obtain structured results without scraping broad logs
- promote a good scenario into smoke or benchmark coverage without first reverse-engineering the harness

That is the standard to aim for.
