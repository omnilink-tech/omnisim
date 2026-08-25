# OmniSim Performance Handbook

This document is the top-level guide for improving simulator speed, simulation quality, and developer iteration speed without breaking determinism or architectural clarity.

It is written for contributors who need to answer three questions:

1. Where is the time going?
2. Which improvements are safe and high-value?
3. How do we improve the simulator without making it harder to reason about?

## The Performance Model

Treat OmniSim as five performance systems, not one:

1. build and link time
2. world load and reset time
3. simulation step throughput
4. sensor and viewport rendering cost
5. authoring and developer workflow overhead

Most slowdowns in practice are caused by interactions between these systems rather than by one isolated algorithm.

Examples:

- a remote texture can make startup slower, bench results noisier, and debugging harder
- template regeneration can make runtime edits slower and also invalidate scene-tree state
- renderer-side texture uploads can dominate sensor-heavy worlds even when physics is light
- a broad include dependency can turn a tiny code change into a product rebuild

## The Current Pipeline

### Build pipeline

Today the public build targets are wrappers over a large make-based product build.

The normal release path still means:

1. platform dependencies
2. `src/glad`
3. `src/wren`
4. `src/omnisim`
5. controller libraries
6. resources and projects

That means code organization and header hygiene still have a direct impact on edit-time speed.

### World-load pipeline

At a high level:

1. tokenize the world file
2. syntax-parse the world
3. instantiate nodes through `OmNodeReader`
4. insert nodes into the world
5. update template and dictionary state
6. finalize nodes
7. download assets if needed
8. enter the simulation step loop

A world-load optimization that ignores parser, template, dictionary, or asset behavior will miss most of the real cost.

### Simulation pipeline

Per step, the simulator roughly does:

1. pre-physics bookkeeping
2. controller synchronization and message flow
3. physics stepping
4. post-physics updates
5. sensor rendering
6. main viewport rendering

The runtime is therefore not “physics only.” Controller orchestration, template behavior, sensor updates, and GPU transfers all matter.

## Core Principles

### 1. Make the fast path explicit

Developers and coding agents need a supported path for:

- build a narrow target
- run a small world
- run headless
- profile one scenario

If the only reliable path is the full desktop product, iteration will remain slow.

### 2. Keep determinism and throughput as separate modes

The code already warns that physics multithreading can improve or hurt speed and can break replicability.

That means the simulator should not hide “faster but less reproducible” settings behind silent heuristics.

### 3. Remove work before optimizing work

The best performance improvement is often deleting a pass, a scan, a copy, or a blocking wait:

- avoid a second parse or scan
- avoid CPU-side texture transforms when the GPU can do them
- avoid remote assets in benchmark worlds
- avoid broad rebuilds when only one target changed

### 4. Measure by scenario, not by intuition

Use small benchmark worlds for:

- startup
- world load
- physics-heavy stepping
- sensor-heavy stepping
- rendering-heavy stepping

Do not assume that an optimization helps all world types.

### 5. Treat asset and world quality as performance work

The simulator can only be fast and stable if worlds are authored to support that:

- sane mass ratios
- bounded contact complexity
- local assets for core content
- texture sizes proportional to actual need

### 6. Optimize for contributor time too

A simulator that is fast at runtime but slow to rebuild is still slow to improve.

## Highest-Value Current Targets

### A. Build and link time

Most effective near-term improvements:

- keep using narrow targets such as `build gui`, `build renderer`, and `build controller-libs`
- keep new GUI dependencies out of runtime-facing headers
- reduce header blast radius in `src/omnisim`
- keep compile database generation working for editors and agents
- use explicit CI fast lanes so small changes do not wait on full packaging by default

### B. World load and reset

Most effective near-term improvements:

- reduce repeated parsing and scanning
- formalize template-regeneration boundaries
- batch dictionary updates where possible
- keep asset download out of benchmark-critical worlds
- split world-load timing into finer buckets

### C. Runtime throughput

Most effective near-term improvements:

- make controller scheduling less entangled with stepping
- document synchronization ownership
- reduce expensive scene traversal and repeated descendant checks
- make visibility information cheaper to query

### D. Sensor and render cost

Most effective near-term improvements:

- move CPU-side texture work to the GPU
- reduce repeated texture upload paths
- separate desktop rendering from sensor rendering more clearly
- add better render-side observability

### E. World quality and content

Most effective near-term improvements:

- remove network asset dependence from shipped performance scenarios
- reduce extreme mass ratios
- reduce excessive contact counts
- prefer local deterministic assets for tests and benchmarks

## Subsystem Map

Use these follow-up docs:

- [build-and-iteration.md](build-and-iteration.md)
- [header-hygiene-and-rebuild-reduction.md](header-hygiene-and-rebuild-reduction.md)
- [validation-playbook.md](validation-playbook.md)
- [ci-and-fast-feedback.md](ci-and-fast-feedback.md)
- [test-harness-and-scenario-architecture.md](test-harness-and-scenario-architecture.md)
- [profiling-playbook.md](profiling-playbook.md)
- [observability-and-performance-telemetry.md](observability-and-performance-telemetry.md)
- [module-dependency-map.md](module-dependency-map.md)
- [controller-protocol.md](controller-protocol.md)
- [controller-ipc-and-step-loop.md](controller-ipc-and-step-loop.md)
- [startup-reset-and-asset-lifecycle.md](startup-reset-and-asset-lifecycle.md)
- [world-loading-and-template-performance.md](world-loading-and-template-performance.md)
- [template-regeneration-and-dictionary-coherence.md](template-regeneration-and-dictionary-coherence.md)
- [physics-and-determinism.md](physics-and-determinism.md)
- [physics-contact-and-collision-complexity.md](physics-contact-and-collision-complexity.md)
- [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md)
- [engine-migration-plan.md](engine-migration-plan.md) (the unified physics + rendering master plan)
- [sensor-and-device-performance.md](sensor-and-device-performance.md)
- [scene-tree-selection-and-runtime-mutation.md](scene-tree-selection-and-runtime-mutation.md)
- [rendering-and-visual-quality.md](rendering-and-visual-quality.md)
- [performance-anti-patterns.md](performance-anti-patterns.md)
- [improvement-backlog.md](improvement-backlog.md)
- [phase-two-execution-program.md](phase-two-execution-program.md)

## Recommended Order Of Work

### Phase 1: Better measurement and safer fast paths

- keep the fast build/test commands stable
- use smoke and benchmark worlds consistently
- improve performance-log fidelity
- document the supported headless contract
- make cheap CI lanes the default feedback path

### Phase 2: Make world load and controller flow cheaper

- reduce parser and node-reader duplication
- narrow template regeneration
- simplify dictionary updates
- move controller work off the critical stepping path where possible
- separate asset wait cost from world construction cost

### Phase 3: Clean up rendering and sensor costs

- eliminate CPU-side environment texture transforms
- reduce GPU transfer overhead
- separate sensor rendering policy from desktop quality policy
- add first-frame and UI-facing performance telemetry

### Phase 4: Narrow architecture boundaries

- introduce a real simulation-core surface
- reduce singleton and global-state coupling
- make the build graph reflect the logical subsystem map
- keep desktop-shell dependencies out of simulation-core headers

## Rules For Performance Changes

- always state whether the change targets build speed, load speed, runtime throughput, or render cost
- validate the narrowest affected world first
- benchmark at least one representative world when changing hot paths
- do not trade away determinism silently
- document new fast paths and new constraints in `docs/developer`

## What To Avoid

- broad rewrites without new measurement hooks
- hidden asynchronous behavior in determinism-sensitive paths
- performance changes that only help the empty-world case
- new remote-asset dependencies in benchmarks, smoke worlds, or shipped critical paths
- new monolithic headers in `src/omnisim`

## Definition Of Progress

The simulator is getting better when all of these improve together:

- smaller rebuild scope for targeted edits
- lower startup and world-load variance
- better physics throughput on representative worlds
- lower GPU transfer cost on sensor-heavy worlds
- better visual quality predictability
- clearer developer and agent workflows

The goal is not only a faster simulator. The goal is a simulator that becomes easier to improve release after release.
