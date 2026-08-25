# Physics, Contacts, And Collision Complexity

This document focuses on the most expensive and least intuitive part of physics performance in OmniSim: contact generation, collision filtering, and the quality of the resulting joint set.

It is grounded in:

- `src/omnisim/engine/OmSimulationCluster.cpp`
- `src/omnisim/nodes/utils/OmMassChecker.cpp`
- `src/omnisim/nodes/OmWorldInfo.cpp`

## Why This Area Matters

When a world is "physics slow," the bottleneck is often not the raw integrator step in isolation. It is the amount of collision work needed to construct and solve contact constraints.

That cost is shaped by:

- how many collision pairs exist
- how many contact points are produced per pair
- how many contact joints are actually kept
- how complicated the contact surface parameters are
- whether the world content itself is numerically hostile

This is why performance, stability, and authoring quality are inseparable in the physics stack.

## What The Current Code Shows

### Contact generation is deliberately capped

`OmSimulationCluster` contains a warning path for when the number of contact points exceeds the allowed number of contact joints between two materials.

When that happens, the code warns that joints will only be created for the deepest contact points instead of for all contact points.

This is an important detail:

- the runtime is already making a quality-versus-throughput tradeoff
- that tradeoff is local and implicit
- contributors currently do not get enough aggregate telemetry to know how often it occurs

### Contact selection already relies on depth ordering

The cluster code uses `std::nth_element(...)` in several places to prioritize contact points.

That means collision complexity is not just "more contacts equals more cost." It is also:

- how many contacts must be generated before truncation
- how often the runtime has to sort or partition them
- whether the retained subset still produces stable behavior

### Contact-surface setup is rich and not cheap

`fillSurfaceParameters(...)` in `OmSimulationCluster` applies a fairly large amount of per-contact policy:

- friction parameters
- asymmetric contact handling
- friction direction setup
- rolling friction
- slip
- bounce
- soft CFM and ERP
- conveyor-belt-like contact surface motion

This is correct and necessary behavior, but it confirms that each retained contact joint is not a tiny unit of work.

### The code still contains threading uncertainty

The necessity of `cJointCreationMutex` itself has now been resolved (the comment at its contact-callback use site states "Mutex IS necessary" — ODE dispatches per-pair contact callbacks across multiple threads, and the joint/immersion-link groups are not thread-safe). What remains is an `Is this necessary?` comment on the destructor's lock around the contact-joint-group teardown.

> **⚠ 2026-08-08 — this whole rationale is deleted, not resolved.** `bdc02139` removed the vendored ODE library, so there is no multi-threaded ODE contact-callback dispatch to protect against; and the **immersion-link group is gone too** (`f0574cbe` deleted `Fluid`/buoyancy). Both halves of the "Mutex IS necessary" argument therefore no longer exist. Whether the surviving Newton contact path needs any equivalent synchronisation is a **separate, unanswered question** — do not carry this conclusion over to it.

That is a signal that:

- thread ownership is not fully self-evident in this path
- performance-sensitive synchronization still needs clarification
- the physics cluster would benefit from a more explicit threading model before more concurrency is added

### Content quality is already known to affect stability

`OmMassChecker` warns when the ratio between the heaviest and lightest dynamic solids exceeds `1e5`.

The comment there explicitly notes:

- problems were observed around `1e6`
- a regular humanoid model is closer to `1e3`

So world authoring is already part of the physics performance contract.

### Multithreading is not free

`OmWorldInfo.cpp` warns that simulation replicability is not guaranteed when multithreading is enabled.

That means the simulator already recognizes that:

- faster physics settings
- deterministic benchmarking
- cross-run reproducibility

are separate operating modes rather than one universal default.

## The Main Performance Risks

### 1. Collision pairs explode before the solver even becomes the problem

If too many broad-phase pairs survive to narrow-phase work, the runtime pays for:

- collision testing
- contact generation
- contact partitioning
- per-contact parameter filling
- joint creation

even if the final joint count is capped.

### 2. Contact truncation can hide both performance and quality problems

When the runtime keeps only the deepest contact points:

- performance may remain barely acceptable
- but contact quality can change
- and contributors still do not get a clear aggregate view of how often truncation happened

That makes regression analysis harder than it should be.

### 3. Complex material policy amplifies per-contact cost

Material and contact properties are a feature, but they also mean the runtime does real work for every retained contact.

That raises the value of:

- reducing unnecessary contact points
- reducing excessive collision pairs
- making world content simpler where possible

### 4. Numerical pathologies become performance problems

Extreme mass ratios, unstable stacks, and excessive contact depth churn do not only reduce correctness. They also waste runtime effort because the solver and contact pipeline must work harder on a hostile problem.

## What To Measure Next

The physics stack needs more contributor-facing diagnostics for:

- number of contact pairs per step
- number of raw contact points generated per step
- number of contact joints actually created per step
- count of truncation events where only deepest contacts are kept
- total ray tests and sensor-collision queries
- time spent in collision handling before joint creation

Without those counters, contributors still have to infer too much from overall step time.

## Recommended Direction

### 1. Expose contact-complexity diagnostics first

Before changing solver or threading behavior, add diagnostics that explain:

- which worlds are contact-heavy
- which material pairs are repeatedly truncated
- whether the cost is dominated by pair count, contact count, or joint creation

This is the cheapest way to improve the physics workflow.

### 2. Keep determinism and throughput as named modes

Do not treat multithreading or contact heuristics as invisible optimizations.

Instead:

- keep deterministic benchmarking explicit
- keep throughput-oriented options explicit
- log the active mode when collecting performance data

### 3. Reduce collision complexity through content and query hygiene

A large part of physics speed will come from:

- fewer unnecessary collision pairs
- simpler collision geometry
- fewer pathological stacks
- more disciplined world authoring

This is often higher leverage than micro-optimizing one solver branch.

### 4. Clarify synchronization before scaling concurrency

The contact path already shows uncertainty around locking. That should be resolved before adding more parallel behavior in the collision pipeline.

### 5. Add one benchmark specifically for contact truncation pressure

The current benchmark set has a contact-heavy world, but phase two should add metrics that explicitly report whether truncation happened and how many joints were retained.

## Review Checklist For Physics Changes

When reviewing a physics or collision change, ask:

1. Does it reduce collision pairs, raw contacts, or joints created?
2. Does it alter truncation behavior or only hide it?
3. Does it change determinism or only throughput?
4. Does it make pathological worlds cheaper, or only average worlds?
5. Can the change be validated with a contact-heavy benchmark and not just a generic smoke test?

## Validation Guidance

After contact or collision changes, validate with:

- `python scripts/dev/omnisim_dev.py benchmarks`
- a contact-heavy world such as `tests/physics/worlds/contact_points.omniworld`
- one deterministic baseline run
- one throughput-oriented run if multithreading or collision scheduling changed

The important output is not only step time. It is whether contact complexity became more visible and more controlled than before.

## Controller API caveats: `wb_supervisor_node_get_contact_points`

The engine-side notes above describe the cost model. The controller-facing API has its own gotchas that are easy to discover the hard way; documenting them here so future contributors don't repeat them.

### The receiving Solid must have no Solid parent

Per [`docs/reference/supervisor.md`](../reference/supervisor.md#wb_supervisor_node_get_contact_points), the queried node "must be a `Solid` node (or a derived node), which moreover has no `Solid` parent." This rules out calling `getContactPoints(False)` on individual links inside a `URDFRobot` or any other Solid hierarchy — those links *do* have a Solid parent (the robot root).

What you observe when you violate this contract: the call still returns a list of `ContactPoint`s, but the values are malformed. In the case that motivated this section, every `node_id` field came back pointing at the very Solid that had been queried (`wheel_rl.getContactPoints(False)` reported each contact's "other" body as `wheel_rl` itself). That looks like a bug, but it is just undefined behaviour from passing a node the API doesn't accept.

The correct pattern for a multi-link robot is to call the API on the top-level Robot/URDFRobot with `includeDescendants=True`, then attribute each returned point back to a logical body using either node-id lookup or contact-point geometry.

### `ContactPoint.node_id` is the **this-side** body, not the other body

The reference docs describe `node_id` as "the unique identifier of the Solid that is in contact with the parent Solid on which the `getContactPoints` method was called." Reading the description quickly suggests `node_id` is the *other* body in the contact pair. In practice on a URDFRobot it returns the descendant Solid that owns the colliding `boundingObject` — i.e. the side belonging to the queried subtree.

This matters for two reasons:

1. **Self-contact filtering.** A naive "ignore contacts where the other id is also part of my robot" filter ends up dropping every legitimate external contact, because every reported `node_id` is in the robot's id set. Don't filter that way.
2. **Mass and velocity for impulse calculation.** You can't read the external body's mass via `node_id`, so the obvious `m_other × |Δv_other|` impulse formula has nothing to plug in. Use momentum conservation on the receiving body instead: `m_robot_part × |Δv_robot_part|` over the step. The magnitude is correct without ever needing to identify the other body.

### URDFRobot consolidates contact reporting onto leaf rigid bodies

Even with the API used correctly, the shape of the returned data was tied to ODE's rigid-body grouping. URDF `fixed` joints fused links into a single ODE body; `continuous`/`revolute` joints don't. For a Husky, that means the four wheels are separate rigid bodies but base_link, top_chassis_link, the bumpers, and the top plate are fused into one chassis body — and contacts on the chassis area get reported with a `node_id` belonging to that fused body. Worse, when an external object hits the chassis, the impulse propagates through joint constraints to the wheels, and contacts on the wheels' own collision cylinders pick up large impulses from the same physical event — so a chassis hit looks like a wheel hit at the API level.

> **⚠ 2026-08-08 — the mechanism cited here is deleted; the observable behaviour is UNVERIFIED.** `bdc02139` removed ODE, so "a single ODE body" no longer describes anything in the engine, and contact points now come from the native Newton source (default-ON). Newton is reduced-coordinate and groups links differently, so **whether the observable consolidation described above still holds — chassis hits reported on wheel bodies, `node_id` naming a fused body — has not been re-measured.** Do not assume it does, and do not assume it doesn't: treat this paragraph as a hypothesis to re-test on Newton before relying on the workaround below.

The practical workaround is a coarse-to-fine attribution pass on the receiving end:

1. Use `node_id` for an O(1) "which rigid body" attribution.
2. If the resulting label is a wheel but the contact-point Z is well above the wheel's geometry (e.g. above the wheel's top), reattribute to whichever non-wheel part is closest in horizontal distance.

This is a heuristic, not a proof — but it correctly separates "box landed on wheel" from "box landed on chassis and wheel transmitted the impulse" without needing engine-level changes.

### Reference: the cylinder_stack demo

The example at [`projects/samples/demos/controllers/contact_points_supervisor/contact_points_supervisor.c`](../../projects/samples/demos/controllers/contact_points_supervisor/contact_points_supervisor.c) is a clean example of the correct pattern: it calls `wb_supervisor_node_get_contact_points` on a top-level Solid (no Solid parent) and uses contact-point world position to attribute each contact to a region of the queried body. New contact-driven code should follow this shape.
