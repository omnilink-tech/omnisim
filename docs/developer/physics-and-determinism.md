# OmniSim Physics, Determinism, and Runtime Throughput

This guide covers the physics side of simulator performance and the rules for improving it without degrading stability or reproducibility.

## The Right Goal

The goal is not “maximum raw speed at any cost.”

The goal is:

- faster representative worlds
- predictable controller timing
- fewer physics instabilities
- explicit tradeoffs when multithreading or plugins change runtime behavior

## Current Runtime Reality

The main simulation step is not just the solver call. It includes:

- pre-physics bookkeeping
- controller/runtime coordination
- collision and contact handling
- physics stepping (Newton `SolverMuJoCo` — CPU `mj_step` or batched-GPU `mujoco_warp`)
- post-physics work
- device and rendering-side follow-up

> ⚠ **2026-08-08:** this list used to read "Newton or ODE depending on backend". `bdc02139`
> deleted the ODE backend; there is one physics backend and no fallback. See
> [ode-retirement-campaign.md](ode-retirement-campaign.md).

That means physics optimization should be done with awareness of controller, sensor, and synchronization overhead.

## Key Current Constraints

### 1. Determinism is already known to be conditional

`WorldInfo.optimalThreadCount` and preferences-driven thread limits already warn that multithreading can change performance positively or negatively and that replicability is not guaranteed.

Implication:

- any performance effort that uses more concurrency must stay explicit
- benchmark results should record whether they were run in deterministic or throughput-oriented settings

### 2. Contact cost is capped in some situations

`OmSimulationCluster` warns when only the deepest contact points are turned into joints rather than all contact points.

Implication:

- contact complexity is already a quality and performance constraint
- worlds with excessive contact counts are both slower and less representative

### 3. Mass ratios affect stability directly

`OmMassChecker` warns when the ratio between the heaviest and lightest solids becomes extreme.

Implication:

- authoring quality and runtime quality are tightly coupled
- “physics performance bugs” are often world-quality bugs

### 4. Some synchronization still has unclear intent

There are still mutexes in runtime and simulation-cluster code whose necessity is questioned by the code itself.

Implication:

- throughput work must include ownership clarification
- otherwise optimization can just move cost and uncertainty around

## Physics Performance Problems To Work On

### A. Hidden cost in contact-heavy worlds

Worlds with many simultaneous contacts can pay heavily in:

- collision handling
- joint creation
- warning generation
- post-contact bookkeeping

Near-term work:

- benchmark one explicitly contact-heavy world
- track contact counts in performance investigations
- document contact-budget guidelines for shipped worlds

### B. ~~Fluid and immersion handling~~ — WORKSTREAM CLOSED (feature deleted 2026-08-08)

> ⚠ **Dropped, not deferred.** `f0574cbe` deleted the `Fluid` node and
> `ImmersionProperties` along with the fork-only ODE buoyancy subsystem they ran on
> (`src/ode/ode/src/fluid_dynamics/`, ~4,758 lines + ~1,085 of headers — never upstream
> ODE). There is no immersion path to instrument and no immersion-heavy world to benchmark.
> The mutex-of-unclear-necessity below was resolved by removal.
>
> Historical statement of the item, for the record: immersion-link creation sat in the
> collision path guarded by a mutex with unclear necessity; the near-term work was to
> document fluid/immersion scenarios as their own benchmark class and measure
> immersion-heavy worlds separately from rigid-contact ones.

### C. Controller/runtime interaction inside the step loop

Physics throughput can look worse than it is if controller scheduling and request handling are mixed into the same performance conversation.

Near-term work:

- compare `physics`, `prePhysics`, `postPhysics`, and per-controller buckets together
- avoid describing a slowdown as “physics” until the profile proves it

## Determinism Rules

### Supported modes should be explicit

Treat these as separate operating intents:

- deterministic mode
- throughput mode

Deterministic mode should prioritize:

- repeatable stepping
- explicit thread counts
- stable benchmark behavior

Throughput mode may accept:

- more threads
- lower reproducibility
- more aggressive graphics or device scheduling choices

### Do not hide the mode change

A performance patch should say clearly if it:

- changes thread behavior
- changes contact ordering risk
- changes timing of controller replies
- changes world-load ordering

## World Authoring Rules For Better Physics

### Mass distribution

- keep mass ratios sane
- avoid tiny dynamic parts with negligible mass attached to heavy systems unless truly needed
- use warnings from `OmMassChecker` as design feedback, not noise

### Contact complexity

- simplify collision geometry where fine detail does not matter
- reduce redundant contact surfaces
- avoid stacking high-detail collision meshes where primitive proxies are enough

### ~~Fluid and buoyancy scenarios~~ — N/A, the feature was deleted (`f0574cbe`)

- there is no `Fluid` node and no `ImmersionProperties`; nothing to author, isolate or
  benchmark. The guidance below is retained only as a record of what it said: treat
  immersion-heavy content as special-purpose worlds, benchmark them separately, and keep
  them out of general startup or smoke scenarios unless the feature is under test.

### Threading

- document expected thread configuration for benchmark worlds
- do not compare deterministic and multithreaded runs as if they were the same mode

## What To Measure

For physics-oriented changes, compare:

- `prePhysics`
- `physics`
- `postPhysics`
- per-controller buckets
- warning behavior

Also record:

- world name
- thread configuration
- whether rendering was enabled
- whether the world is contact-heavy, sensor-heavy, or mixed

## Safe Improvement Order

### 1. Improve observability

- measure contact-heavy worlds explicitly
- add better phase attribution
- keep deterministic vs throughput runs distinct

### 2. Fix authoring pathologies

- reduce bad mass ratios
- reduce avoidable contact complexity
- remove pathological worlds from generic benchmarks

### 3. Clarify synchronization ownership

- explain or remove questionable mutexes
- document which code can run concurrently and which must not

### 4. Optimize true hot paths

- only after the above
- use benchmark worlds to validate gains

## Developer Checklist For Physics Changes

- build with the narrowest safe target
- run smoke plus one physics-oriented world
- compare performance logs before and after
- record whether multithreading changed
- note if warning behavior changed
- check whether the change affects determinism-sensitive paths

## What “Better Physics Performance” Should Mean Later

Later, a better physics subsystem should have:

- clearer deterministic and throughput modes
- better contact instrumentation (immersion instrumentation is moot — the `Fluid` feature
  was deleted in `f0574cbe`)
- less ambiguous synchronization
- benchmark-grade physics scenario coverage
- tighter guidance for world authors so bad content does not masquerade as engine failure
