# OmniSim Performance Anti-Patterns

This document is a concentrated list of patterns that repeatedly make OmniSim slower to build, slower to run, harder to validate, or harder for coding agents to improve safely.

Use it as a review checklist. A surprising amount of performance work is simply refusing to add one more instance of a known bad pattern.

## 1. Multi-Pass Work On The Critical Path

The world-load path already does more than one pass through world data:

- tokenization
- syntax parsing
- node reading and instantiation
- template and dictionary follow-up work

That means every extra pass, scan, or rebuild hurts startup and reset latency quickly.

Avoid:

- reparsing or rescanning when one earlier pass already computed the needed information
- broad follow-up scans after localized node mutations
- repeated descendant or dictionary walks in frequently used paths

Prefer:

- carrying forward metadata from earlier passes
- batching follow-up work
- caching results with explicit invalidation

## 2. Remote Assets In Core Development Or Benchmark Paths

Remote asset fetches are one of the worst kinds of slowness because they add:

- startup variance
- offline failure modes
- flaky benchmarks
- pauses in simulation flow

Avoid:

- benchmark worlds that depend on network assets
- smoke tests that fetch resources during validation
- making a common sample world slower because its assets are not local

Prefer:

- local deterministic assets for tests and benchmark content
- explicit prefetch or packaging steps outside the hot path

## 3. CPU-Side Work That Belongs On The GPU

Some rendering and texture paths still do CPU-side transforms, conversions, or upload preparation that should be renderer-native.

Avoid:

- rotating or transforming environment data on the CPU when the shader can do it
- repeated CPU-side copies before texture upload
- per-frame conversion work that could be cached or expressed in GPU state

Prefer:

- one-time preprocessing
- GPU-side transforms
- cached immutable render resources where possible

## 4. Broad Headers For Narrow Behavior

This is one of the main edit-time anti-patterns in the repo.

Avoid:

- adding a new include to a high-fanout header because one new method needs it
- putting non-trivial logic inline in common node or world headers
- letting utility headers become global dependency magnets

Prefer:

- forward declarations
- implementation-only includes
- helper extraction into `.cpp` or narrow facades

This is how we prevent tiny changes from causing wide rebuilds.

## 5. Mixing Desktop Concerns Into Runtime Paths

The desktop shell must observe runtime state, but runtime code should not become slower or more coupled because the editor wants immediate updates.

Avoid:

- runtime logic that emits broad GUI work on every mutation
- parser, world, or stepping code depending on scene-tree or widget behavior
- data model choices made mainly for desktop refresh convenience

Prefer:

- structured runtime events
- desktop adapters
- coalesced GUI updates

## 6. Full-Scene Or Full-World Scans In Repeated Paths

Many correctness-oriented implementations start as full scans. That is acceptable once. It becomes expensive when it lands in per-step, per-device, or repeated-edit paths.

Avoid:

- repeated world scans for information that changes infrequently
- repeated descendant checks when identity or ancestry could be cached
- repeated per-object work inside every sensor update when coarse rejection could happen first

Prefer:

- precomputed sets
- dirty tracking
- coarse spatial or logical filtering before fine-grained work

## 7. Global Singletons That Hide Expensive Coupling

Singletons are not automatically slow, but in OmniSim they often hide broad ownership and invalidation costs.

Avoid:

- adding cross-subsystem behavior to an already global manager
- letting one singleton become the easiest way for every subsystem to reach every other subsystem
- using global state when a narrower service boundary would make dependencies explicit

Prefer:

- explicit ownership
- subsystem-local services
- well-defined update and invalidation contracts

## 8. Quality Heuristics That Are Not Observable

The simulator already changes rendering quality based on coarse heuristics. That can be useful, but it becomes hard to reason about if contributors cannot see what happened.

Avoid:

- silent quality changes with no logging
- heuristics that alter several settings at once without a clear trace
- "automatic optimization" that hides determinism or visual tradeoffs

Prefer:

- explicit mode reporting
- performance logging that includes the quality policy in effect
- user-visible or developer-visible reasoning for degraded settings

## 9. Benchmarks Without A Stable Contract

Benchmark data is only useful when the scenario and collection method are stable.

Avoid:

- worlds that depend on remote data
- benchmarks that mix startup, load, and runtime without telling you which changed
- ad hoc local timing numbers that cannot be reproduced in CI

Prefer:

- documented benchmark worlds
- explicit collection commands
- clear output fields and comparison rules

## 10. Making CI Discover Problems Too Late

A long feedback loop is a performance bug in the development system.

Avoid:

- defaulting every PR to the most expensive validation path
- relying on full package jobs to catch narrow subsystem regressions
- undocumented validation steps that only one maintainer knows

Prefer:

- fast default lanes
- subsystem-targeted builds
- clearly documented commands that both humans and agents can run

## 11. Content Pathologies Treated As "User Error"

World quality is part of simulator performance and stability. The engine should not assume content problems are someone else's job.

Avoid:

- ignoring extreme mass ratios
- shipping overly large textures for small visual benefit
- allowing excessive contact complexity in performance-sensitive sample worlds

Prefer:

- authored performance guidelines
- warnings that are tied to actionable docs
- benchmark worlds that model good content hygiene

## 12. Changes Without A Narrow Validation Story

If a contributor cannot explain how to validate a change cheaply, the architecture is probably still too entangled.

Avoid:

- changes that require the full desktop product for a parser-only edit
- renderer changes that cannot be built without unrelated desktop surfaces
- refactors that claim to improve modularity but keep the same all-or-nothing workflow

Prefer:

- a documented narrow build target
- one world or smoke scenario that proves the behavior
- headless validation for simulation-core work whenever possible

## How To Use This Document In Review

Before approving a performance-sensitive change, ask:

1. Which anti-patterns does this change remove?
2. Which anti-patterns might it accidentally add?
3. Does the validation path stay narrow?
4. Does the change improve only runtime cost, or also contributor iteration cost?

If the answer to the last question is "runtime only," consider whether the same work can also reduce rebuild, validation, or architecture cost.

## The Standard To Aim For

The simulator improves fastest when each change moves it toward:

- fewer critical-path passes
- fewer hidden cross-subsystem dependencies
- smaller rebuild scope
- clearer headless validation
- better measurement of both runtime and authoring cost

That is the bar for performance work in OmniSim.
