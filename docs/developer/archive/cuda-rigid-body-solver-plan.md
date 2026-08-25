# CUDA Rigid-Body Solver — Plan to Replace ODE for Multi-Husky Demos

> **Superseded.** Newton was chosen and the migration shipped; the current
> canonical plan is [engine-migration-plan.md](../engine-migration-plan.md).
> The Newton-vs-PhysX-vs-custom decision is summarised in its
> [solver decision log](../engine-migration-plan.md).
> The pre-Newton ODE-optimization plan
> and the CUDA compute infrastructure plan (formerly `cuda-compute-infrastructure-plan.md`)
> are both folded into engine-migration-plan.md.
> This doc is preserved for the full Q1-2026 survey and decision context.

This doc plans the major architectural move of replacing ODE
(single-threaded CPU rigid-body solver inside `omnisim-bin`) with a
GPU-resident rigid-body solver, enabling **10–20 huskies colliding at
30+ fps real-time** — a scene that's structurally impossible on ODE.

It is the natural successor to:
- pre-Newton ODE-optimization plan (now folded into [engine-migration-plan.md](../engine-migration-plan.md))
- CUDA compute infrastructure plan (now folded into [engine-migration-plan.md](../engine-migration-plan.md))
- [`fps-optimization-journey.md`](fps-optimization-journey.md) — measured the wall.

OmniSim is a Cyberbotics fork — anything in the binary is on the
table to redesign.

---

## Goal

| metric | current (ODE) | target (GPU solver) |
|---|---|---|
| 2-husky head-on | 7.9 fps | 60+ fps |
| 10-husky scene | est. ~1.6 fps | **≥30 fps** |
| 20-husky head-on | est. <1 fps | **≥30 fps** |
| Damage-system fidelity | Full | Full |
| Joint types supported | URDFRobot's fixed + continuous + revolute | Same set |
| Determinism | Bit-exact across runs | Per-run reproducible (best effort across hardware) |

Out of scope: differentiable physics (RL training), soft-body, fluid coupling, Vulkan/Metal compute. All future-extensible from this base.

---

## Why ODE can't get there

ODE is *fundamentally* single-threaded. The solver is iterative
Gauss-Seidel over coupled joint+contact constraints; that algorithm
is hard to parallelise without changing the math. Even if we
multi-thread parts of it, the constraint solver itself is the
bottleneck and doesn't decompose cleanly.

For our case:
- Per-step cost grows roughly **O(joints + contact_pairs)** in narrowphase, **O(N²)** in broadphase pair tests.
- 2 huskies × 6 collision shapes = 12 dynamic shapes today, ~100 ms/step.
- 10 huskies × 6 = 60 shapes, ~1800 contact-pair tests if all huskies overlap. Easily 1+ sec/step.
- Husky has 4 continuous joints (wheels). 10 × 4 = 40 active joint constraints. Plus contact joints (capped at 10 per material pair, so plenty during a pile-up).

ODE is also stuck at single-precision rigid bodies with single-threaded Gauss-Seidel iteration. Modern GPU solvers run **PGS / TGS in parallel batches**, finishing the same number of iterations in microseconds.

---

## Solver-selection survey

Three credible options. Phase 0 picks one based on bench numbers,
license fit, and integration cost.

### Option A — NVIDIA PhysX 5.x

| dimension | assessment |
|---|---|
| Maturity | Production. 10+ years of game/sim deployment. Used in Unreal, Unity. |
| Rigid-body | First-class. Articulated bodies (`PxArticulationReducedCoordinate`) handle robot-like jointed kinematic chains directly. |
| Performance | ~1000 articulated bodies at 60 fps on a single A100; RTX 3060 likely 200-500. |
| License | BSD-3-Clause. Free for commercial use. |
| Determinism | Per-run stable; cross-machine *not* guaranteed without specific config. |
| OmniSim integration | C++ API. Plays nicely with our existing `OmCudaContext`. CUDA backend toggle. |
| Risks | Large library (~50 MB). Steep API surface. Older versions had Windows-specific quirks. |

### Option B — NVIDIA Newton / Warp

| dimension | assessment |
|---|---|
| Maturity | Newer (open-sourced 2024). Powers NVIDIA Isaac Lab / Isaac Sim 5.0. |
| Rigid-body | Designed *for robotics from the ground up*. Articulated bodies via Featherstone. Built-in contact handling. |
| Performance | Comparable to or better than PhysX for many-robot scenes. Targets 1000s of robots in parallel. |
| License | Apache 2.0. Permissive. |
| Determinism | Run-to-run reproducible on same hardware. |
| OmniSim integration | Python/C++ APIs. Newton runs on Warp (NVIDIA's CUDA Python toolkit). Could run alongside our existing `OmCudaContext` or replace parts of it. |
| Risks | Younger codebase. Smaller community. NVIDIA-only (no AMD/Intel). |

### Option C — Custom CUDA solver

| dimension | assessment |
|---|---|
| Maturity | Zero — built from scratch. |
| Rigid-body | Whatever we implement. Start with PGS for rigid bodies, add joints as needed. |
| Performance | Unbounded by external library. Same ceiling as A or B if done well. |
| License | Ours. Zero external constraint. |
| Determinism | Whatever we design for. |
| OmniSim integration | Native — no FFI overhead, no library coupling. |
| Risks | **6–12 months** of physics-engine engineering. Uniformity with `OmCudaContext` for free. |

### Recommendation

**Start with Newton/Warp** unless P0 benchmarks reveal a specific
PhysX advantage. Reasons:

1. Designed for our exact use case (multi-robot scenes).
2. Apache 2.0 fits OmniSim's Apache 2.0 license cleanly.
3. NVIDIA actively invests in it (Isaac platform, robotics focus).
4. Differentiable physics is built-in, opening future RL-training use.
5. Avoids the 6–12 month custom-build cost.

PhysX is a fine fallback if Newton's joint feature set or
determinism story doesn't match URDFRobot's needs.

---

## Architecture

### High-level

```
                +-----------------------------------------------+
                |             OmApplication / scene             |
                +----+--------------------------+---------------+
                     |                          |
                     v                          v
            +-----------------+        +------------------+
            |  OmSolid /      |        | URDFRobot import |
            |  OmBasicJoint   |        | -> GPU bodies    |
            +--------+--------+        +--------+---------+
                     |                          |
                     v                          v
                +----------------------------------+
                |       OmPhysicsBackend           |
                |  (abstract: ODE | GPU)           |
                +----------------+-----------------+
                                 |
                  +------------+ + +-----------------+
                  |  ODE       |   |  GPU (Newton)   |
                  |  legacy    |   |  primary        |
                  +------------+   +-----------------+
                  |                |                 |
                  v                v                 v
              walls/floor      huskies          particles
              static colliders dynamic bodies   (debris field)
```

### Key idea: dual-backend, body-level routing

Each `OmSolid` has a `physicsBackend` attribute (default `ODE` for
backward compat). Worlds opt huskies into the GPU backend explicitly.
Static colliders (floor, walls) stay on ODE because GPU broadphase
doesn't pay off for stationary geometry.

Cross-backend contacts (husky vs. floor) handled by a small bridge:
the GPU body's AABB gets pushed into an ODE-side proxy each step;
contacts found by ODE flow back to the GPU side as external contact
constraints. Ugly but pragmatic for the migration.

### What the existing damage system sees

The damage tracker (after P1 of the parent plan, when it's C++
inside the binary) reads contacts via `WbContactPoint`. We extend
the contact-point source to merge ODE-found and GPU-found contacts
into a single iteration. Damage system code is unchanged.

`getVelocity()`, `getPosition()`, `setVelocity()` etc. dispatch
through `OmPhysicsBackend`. ODE-backed bodies hit the existing path;
GPU-backed bodies fetch from the GPU buffer (with a lazy host
mirror for read-heavy paths like the supervisor's per-step polls).

---

## Phases

| phase | scope | effort | exit criterion |
|---|---|---|---|
| **P0** | Solver bake-off: 10-husky-equivalent benchmark on Newton, PhysX, custom-stub | 1 week | Pick one. Bench numbers documented. |
| **P1** | `OmPhysicsBackend` abstraction + Newton bring-up. One rigid sphere on GPU. | 2 weeks | Sphere falls under gravity, position visible in WREN, velocity readable from a controller. |
| **P2** | Articulated husky on GPU (chassis + 4 wheel joints, drive_forward applies torque) | 2–3 weeks | One husky drives across the floor. Contact with floor handled (cross-backend bridge). |
| **P3** | 10 huskies in parallel, no inter-husky collision yet | 1 week | 10 huskies driving in straight lines. fps measured. |
| **P4** | Inter-husky GPU contact resolution. Two-husky head-on then ten-husky pile-up. | 2–3 weeks | 20 huskies colliding at ≥30 fps. |
| **P5** | Damage-system contact integration (depends on P1 of the pre-Newton ODE-optimization plan (now folded into [engine-migration-plan.md](../engine-migration-plan.md))). Existing damage phases work over GPU bodies unchanged. | 1–2 weeks | `husky_head_on.omniworld` runs on GPU backend with full damage fidelity. |
| **P6** | Wire-protocol harness compatibility audit. simulationReset, getContactPoints, etc. work for GPU bodies. | 1–2 weeks | Existing tests pass, existing harnessed agents (husky_maze, etc.) load and run. |
| **P7** | (Optional) Migrate static colliders to GPU too — full single-backend mode. | 2–4 weeks | ODE removed from build. |

### Total

**P1–P5 = 8–11 weeks** for "the demo we want": 10–20 huskies, full
damage, 30+ fps. **P0 + P6 + buffer = +3–4 weeks** for production
hardening. Realistic estimate: **3 months focused work, 4–5 months
real-world calendar**.

P7 is optional and can come later — keeping ODE for static colliders
indefinitely is fine.

---

## What this unlocks

### Immediate (P5)
- 10–20 huskies head-on at 30+ fps real-time.
- Multi-robot training scenarios (RL with many agents in parallel).
- Larger debris fields without the per-fragment cost spiking.
- Phase 17 fragment spawning becomes safe again because GPU bodies don't share the URDFRobot subtree memory model that triggered the access-violation crashes.

### Medium-term (after P5)
- Differentiable physics (Newton's killer feature). Enables gradient-based RL, system identification, controller tuning where gradients flow through the simulator.
- Massive granular-coupling demos (combine `OmGranularGroup` and `OmPhysicsBackend` GPU side).
- A path to GPU-resident sensor simulation (depth cameras, lidars rendered against the scene's GPU state).

### Long-term (P7+)
- ODE removed entirely. No more single-threaded constraint solver.
- Path to MPM / SPH soft-body coupling on the same compute substrate.
- 100+ robot scenes (factory floors, swarm demos) become tractable.

---

## What this costs

### Engineering
- 3 months focused work (1 senior simulation engineer, full-time)
- Or 5 months at 60 % allocation
- Or 12 months at 25 % allocation

### Risk
- Determinism: GPU solvers are typically per-run reproducible on a fixed device but not bit-exact across hardware or driver versions. **Impact**: existing test harness uses determinism for validation. Mitigation: keep ODE-backed test path indefinitely; tag GPU-backend tests as "approximate" or "best-effort".
- Joint feature parity: URDFRobot uses `fixed`, `continuous`, `revolute`, sometimes `prismatic`. All four are bread-and-butter for any modern GPU solver. Verify in P0 the chosen library handles all four cleanly.
- Cross-backend physics fidelity: husky-on-floor contact via an ODE↔GPU bridge introduces interpolation error at the boundary. Acceptable for visual demos; may not be acceptable for closed-loop control work. Mitigation: world authors can opt to put the floor on GPU too (P7) when fidelity matters.
- Library churn: Newton is young. Possible API breaks in next 12 months. Mitigation: pin a specific Newton commit; upgrade deliberately.

### Maintenance
- Adding ~50–100k LOC to the binary (Newton-bound) or comparable (custom).
- Build complexity: NVCC + Warp + Newton requires a CUDA toolchain on every dev box that builds the simulator. Already true for `OmGranularGroup` so the bar is mostly already paid.
- Per-Webots-version maintenance: when we pull from Cyberbotics upstream, patches in physics integration code conflict more than usual. Mitigation: namespace our changes behind `OmPhysicsBackend` so the diff stays localised.

---

## Performance expectations, with reasoning

### Why we believe 30+ fps for 20 huskies is achievable

Reference data points:
- Isaac Sim demos run **2048 ANYmal robots** in parallel at 60+ fps on an RTX 3090. Each ANYmal has more joints than a husky.
- NVIDIA Flex (PhysX's predecessor) handles **10 000+ rigid bodies** with friction and contacts at 60 fps on a 1080 Ti.
- `OmGranularGroup` already handles **100 000 contact-coupled spheres** at 4.5 ms/step on the same RTX 3060 we're targeting (4× real-time headroom).

20 huskies × 6 collision shapes = 120 shapes. That's **3 orders of magnitude smaller** than the granular benchmarks. We're not even pushing the GPU.

The remaining cost will be:
- Render pipeline (WREN rendering 20 husky meshes). Solvable by P5 of the pre-Newton ODE-optimization plan (now folded into [engine-migration-plan.md](../engine-migration-plan.md)) (instanced rendering).
- Cross-backend bridge for husky-vs-floor contacts. Sub-ms cost per husky.
- Damage system per-step work. After P1 of the parent plan, runs in C++ inside the binary, sub-ms.

So our expected fps for the 10-husky scene after P5: **80–120 fps real-time**. The 30 fps target is conservative.

### The catch

Cold start (first step) on Newton / PhysX includes JIT compilation
of articulation kernels — ~5-10 seconds added to world load. That's
a UX regression we'll need to message clearly. After cold start,
steady-state fps is what's quoted above.

---

## Recommended sequencing

**Don't start this without the pre-Newton ODE-optimization plan (now folded into [engine-migration-plan.md](../engine-migration-plan.md)) P0–P2 first.**
The P0 profile tells us whether ODE is actually as bottlenecking as
the bisect suggested (vs. WREN render or supervisor IPC). The P1 C++
damage tracker is needed regardless — having damage logic outside
the binary makes it hard to integrate cleanly with a new physics
backend.

So: **do the pre-Newton ODE-optimization plan (now folded into [engine-migration-plan.md](../engine-migration-plan.md)) P0 + P1 first** (~1 week),
*then* commit to this plan if the numbers say physics is the wall
that justifies a new solver. **The fastest path to 30 fps for 2
huskies is the parent plan; the path to 30 fps for 20 huskies is
this one.**

The two plans share P0 and ideally share P1 — write them once.

---

## Decision points along the way

| moment | question | if yes | if no |
|---|---|---|---|
| End of P0 | Newton bench beats PhysX bench | go Newton | go PhysX (or revisit custom) |
| End of P2 | One articulated husky on GPU, ≥30 fps single-husky | proceed | escalate (likely solver-config issue) |
| End of P4 | 20 huskies colliding at ≥30 fps | ship | revisit broadphase, joint config, or scale back to 10 huskies |
| End of P5 | Damage system works over GPU bodies | ship | block on damage-tracker integration; may need to refactor `WbDamageManager`'s contact source |

---

## Summary

This plan is *the* path for many-robot OmniSim demos. ODE simply
cannot get us to 20 huskies at any reasonable frame rate; the
algorithm is wrong for the workload. Newton (or PhysX as fallback)
is the proven answer.

3 months focused work. Massive payoff. Foundation for differentiable
physics, RL-at-scale, and MPM/SPH soft bodies on the same substrate.

The right time to commit: **after the pre-Newton ODE-optimization plan (now folded into [engine-migration-plan.md](../engine-migration-plan.md)) P0 + P1
land** and we have hard numbers about ODE vs. supervisor vs. render
share of the per-step budget. Expected outcome of that data: ODE is
the dominant cost, and this plan is the right next step.
