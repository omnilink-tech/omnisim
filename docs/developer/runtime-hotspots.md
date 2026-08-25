# OmniSim Runtime Hotspots and Blockers

This document is the deeper implementation review for developers working on simulator speed, correctness, and headless automation.

## End-to-End Runtime Path

At a high level the simulator still behaves like one large application boundary:

1. `src/omnisim/gui` starts the product and parses CLI mode
2. `src/omnisim/nodes/utils/OmWorld.*` and `src/omnisim/vrml/*` load the world model
3. `src/omnisim/engine/OmSimulationWorld.*` owns world startup, mode transitions, and the main simulation step
4. `src/omnisim/control/OmControlledWorld.*` and `src/omnisim/control/OmController.*` synchronize controller traffic with stepping
5. `src/omnisim/wren` and `src/wren` handle main rendering and sensor rendering

This is why the codebase still feels "product-shaped" rather than "simulation-core-shaped".

## Primary Architectural Blockers

### 1. No first-class simulation-core boundary

There is still no dedicated public runtime surface that says:

- load this world
- step it
- inject controller messages
- collect outputs and metrics

Instead, headless automation still runs through the main simulator CLI with batch and no-rendering flags.

Impact:

- harder to build a fast automation lane
- harder to benchmark startup and stepping in isolation
- harder for coding agents to work against a stable runtime contract

### 2. Controller stepping is entangled with world stepping

`OmControlledWorld` is still driven directly by controller request flow, and `OmController` still mixes:

- packet parsing
- supervisor-side mutation concerns
- template-regeneration blocking
- timing measurement
- immediate-message flushing

Impact:

- changes in controller behavior can affect runtime determinism
- supervisor mutations still leak into controller read/write paths
- the "step the world" contract is not independent of controller protocol details

### 3. World loading is still synchronous and cache-sensitive

World startup currently combines:

- parsing
- node creation
- template work
- remote asset handling
- finalization

Remote asset download is still part of the blocking world-load path.

Impact:

- startup time is noisy
- benchmark comparisons are cache-sensitive
- one slow or missing asset can distort simulation startup behavior

### 4. Threading ownership is not explicit enough

There are still synchronization sites in the runtime and physics code that are marked as questionable or under-explained.

Impact:

- hard to reason about determinism
- hard to increase throughput safely
- performance work risks introducing subtle ordering bugs

### 5. Scene traversal work is still too expensive in some node utilities

Some node utility paths still do repeated graph queries that are called out as slow.

Impact:

- world mutation and scene queries cost more than they should
- some runtime and editing paths still scale poorly with scene size

## Concrete Hotspots To Prioritize

### Headless contract

Near-term implementation goal:

- keep using the current CLI flags
- formalize them as the supported headless execution contract
- stop treating the desktop shell as the only real entrypoint

### Controller/runtime untangling

Break apart the current responsibilities into:

- packet decode
- controller lifecycle
- simulation step scheduling
- supervisor mutation handling
- logging and determinism policy

The important rule is that controller traffic should not be the only thing that makes the runtime stepable.

### World-load timing split

Separate current `loading` time into:

- parse/tokenize
- PROTO/template regeneration
- asset download/cache
- node finalization
- first step / first frame

Until this is done, startup regressions remain hard to attribute.

### Runtime data structures

High-value cleanup work:

- reduce repeated descendant checks and full-tree scans
- introduce cheap visibility and mutability metadata where node traversal depends on it
- keep parser/runtime utilities from reallocating or rescanning broad state on every mutation

## Correctness Risks Worth Fixing Early

These are not just optimization ideas. They are real behavior risks.

### Fog semantics

The code notes that only the first `Fog` node should be honored, but that rule is not enforced at the type boundary yet.

Why it matters:

- world semantics can become ambiguous
- scene correctness can depend on load order rather than explicit validation

### Controller stream buffering

The controller-side pipe read path still uses a fixed 1024-byte buffer.

Why it matters:

- large controller output can be truncated or mishandled
- diagnostics become less reliable under noisy controllers

### Mutable caches in otherwise const-facing node APIs

Some base-node caching still relies on mutable state in ways the code itself flags as problematic.

Why it matters:

- harder reasoning about side effects
- more hidden invalidation behavior
- easier to break correctness while chasing performance

## Suggested Runtime Refactor Order

### Phase 1: Make the current runtime measurable

- restore missing performance log hooks
- split world-load timing
- make headless runs first-class in docs and CI

### Phase 2: Untangle controller scheduling from stepping

- isolate request decode
- isolate supervisor mutation handling
- make the world step loop invocable independently

### Phase 3: Narrow the runtime surface

- define a supported simulation-core API boundary
- keep desktop-only concerns out of that layer
- make test and benchmark harnesses target the runtime boundary first

### Phase 4: Simplify synchronization

- document thread ownership
- remove or rename mutexes whose intent is unclear
- keep determinism mode and throughput mode explicit

## What Developers Should Avoid For Now

- do not silently introduce more background work into world loading
- do not add new GUI dependencies to runtime-facing headers
- do not hide deterministic behavior changes behind performance tweaks
- do not rely on the desktop shell as the only validation path for runtime changes

## Practical Guidance

If you are changing:

- `src/omnisim/control`: validate both API behavior and headless stepping
- `src/omnisim/engine`: validate smoke plus at least one benchmark world
- `src/omnisim/vrml` or `src/omnisim/nodes/utils`: validate deterministic world-load paths
- synchronization or physics code: compare performance logs before and after, not just pass/fail tests
