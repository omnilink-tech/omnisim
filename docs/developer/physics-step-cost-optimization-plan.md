# Physics step-cost optimization plan — 2026-08-08, revised 2026-08-09

**Question this answers:** can OmniSim physics get 2–5× faster, and by what mechanisms?

**Answer, measured end-to-end: 2.1–3.6× is SHIPPED**, workload-dependent, against the 2026-08-08 engine — from four physics-neutral changes, each of which hit its predicted value. Interleaved binary A/B, machine `9722d23d12a3`:

| workload | 2026-08-08 | today | speedup |
|---|---:|---:|---:|
| 5 boxes | 1.582 ms/step | 0.446 | **3.55×** |
| 50 boxes | 2.287 | 1.110 | 2.06× |
| 8 Huskies, motorised (n=3/arm, spread 19%/22%) | 2.083 | 0.827 | **2.52×** |

The shape is the one the decomposition predicts: the smaller the scene, the larger the glue fraction that was removed, and the closer to the `mj_step` floor you land.

**4× was the target and it is NOT reached.** Two of the remaining items are now measured *closed* rather than pending, which is the useful part of the answer:

- the **body half** of the state-serving idea (worth ~0.19 ms) is **unsafe on articulated robots** — `|xpos − body_q|` reaches **2.8e-02 m** there (2.7e-07 on box stacks). Do not ship it;
- the **rendering-transform writeback** inside the post bracket is **not the cost** — priced with `OMNISIM_NEWTON_SKIP_WREN` and the effect is below a 2× block-variance noise floor (the skip arm even measured slower).

What is left is `mj_step` itself (now **~30% of the robot tick**, up from 9%), the newton↔mjc state conversion (~0.17, reachable only by resolving the frame question above), and a post-writeback bucket whose composition is **not yet attributed**. So the honest forward statement is: **4× needs the frame question answered or the post bracket decomposed — it is no longer a matter of executing a known list.** Beyond that lies `mj_step`, i.e. physics changes (solver iterations/tolerance/dt) or batched GPU training, which is a different product question.

### Shipped ladder (8-Husky world, steady-state, differenced)

| stage | commit | tick | cumulative |
|---|---|---:|---:|
| 2026-08-08 engine | — | ~1.75 ms | 1.00× |
| + CPU-device pin | `0fc15a998` | ~0.85 | 2.05× |
| + flush gate, motor change-detect, cached flags | `55164f986` | ~0.70 | 2.51× |
| + clamp pre-checks mj `qpos`/`qvel` | `8e3ea87fa` | ~0.69 | 2.54× |
| **end-to-end A/B verdict** | | **0.827** | **2.52×** |

## 0. Read this first: the first campaign's headline was WRONG, and how

The original version of this document said the tick was dominated by "FK re-derivation" inside `_update_newton_state` and projected ~2× from replacing it. Tier 1 shipped that (`6e55cc777`) and the interleaved A/B measured **~1.0×** (`0f7233344`). The diagnosis was a **cost-attribution error**: `_update_newton_state`'s cost was never the FK *math*, so a surgical replacement that still went through warp paid the same toll and won nothing.

The real cause, found by *pricing the primitives instead of reading the code* (2026-08-09): **warp defaults to `cuda:0` when a GPU is present, and nothing in the engine ever overrode it — so the newton Model/State arrays lived on the GPU while `mj_step` ran on the CPU.** Every tick round-tripped state across PCIe for a simulation that never touched the GPU.

Measured on machine `9722d23d12a3` (RTX 3060 Laptop, warp 1.13, mujoco 3.8.1), per call:

| warp API call | on `cuda:0` | on `cpu` | ratio |
|---|---:|---:|---:|
| `.numpy()` readback | 0.0538 ms | 0.0020 ms | **27×** |
| `wp.array(np)` create | 0.0444 | 0.0158 | 2.8× |
| `wp.launch` (trivial kernel) | 0.0184 | 0.0074 | 2.5× |
| `.assign(host)` | 0.0221 | 0.0097 | 2.3× |

A tick makes about a dozen such calls (joint clamp 2, base guard 2 + copies, state conversion 2 launches + 2 creates, readback caches, control drain on motorised worlds). **That is the tick.**

**The transferable rule: attribute cost to the API-CALL LAYER before attributing it to the named computation.** Read-the-code reasoning produced a confident, wrong answer twice in this tree (the "FK is the cost" claim, and before it the "17–33× faster than ODE" claim). A 40-line microbenchmark of the primitives settled it in minutes.

## 1. The measured decomposition (2026-08-09, robot world, chrono clock)

`projects/samples/demos/worlds/physics/newton_husky_swarm_drive.omniworld` — 8 motorised Huskies, 51,000 ticks so one-time costs are negligible. Timed with a `steady_clock` sub-profiler inside the PHYSICS bracket (`OMNISIM_NEWTON_STEP_PROFILE=1`), which reconciles against a whole-bracket timer on the same clock to **0.0008 ms unaccounted**.

| component | today (`cuda:0`) | with CPU-device pin | what it is |
|---|---:|---:|---|
| `mj_step` | 0.1496 | 0.1454 | **the floor — the actual physics** |
| newton↔mjc state conversion | 0.2881 | 0.1687 | marshaling, not solving |
| other per-tick Python glue | 0.7725 | 0.1799 | clamp, base guard, caches, control drain |
| `flushPendingNewtonRegistrations` | 0.1249 | 0.0973 | per-tick walk of ALL Solids |
| motor target push | 0.0269 | 0.0252 | one FFI call per motorised joint |
| C++ post writeback (bucket units) | 0.3824 | 0.2340 | scene-graph fields + WREN transforms |
| **≈ tick total** | **≈1.75** | **≈0.85** | floor is **9% → 17%** of it |

Box worlds (generated `step_cost` scenes) decompose the same way, more extremely: at 5 boxes the physics bracket is 0.290 ms/tick of which `newton->step` is 0.278 (96%), `flushRegistrations` 0.0115, motor push 0.0003 — and bare `mj_step` is 0.036.

**Floor reference, measured standalone with no engine at all** (same mujoco build, boxes on a plane): `mj_step` = 0.0060 ms at 1 body, 0.0196 at 5, 0.0889 at 20, 0.2063 at 50; `mj_step1` = 0.0040 / 0.0116 / 0.0397 / 0.0938.

⚠ **Instrument note:** the engine's own `OmPerformanceLog` PHYSICS bucket reads ~1.8× higher than a direct `steady_clock` around the identical code region (0.52 vs 0.29 ms/step at 5 boxes, same runs). Every **ratio** in this document is safe (both A/B arms use one instrument), but do not mix absolute numbers across the two instruments. Root cause not chased; `OmPerformanceLog` accumulates microseconds (`nsecsElapsed()*1e-3`) and is a candidate.

## 2. What is BANKED (measured, physics-neutral, interleaved A/B on an idle box)

**Pin the newton model to the CPU device whenever the solver is CPU `mj_step`.** `builder.finalize(device="cpu")`; `state()`/`control()` follow `model.device`. `mujoco_warp` (batched GPU training) is untouched and still builds on the GPU. Override: `OMNISIM_NEWTON_MODEL_DEVICE=cpu|cuda|auto`.

| world | today (`cuda:0`) | CPU-device pin | speedup |
|---|---:|---:|---:|
| 1 box | 0.981 ms/step | 0.563 | **1.74×** |
| 5 boxes | 1.355 | 0.577 | **2.35×** |
| 20 boxes | 1.164 | 0.739 | 1.57× |
| 50 boxes | 1.368 | 0.957 | 1.43× |
| 8 Huskies, motorised (3 blocks, spread 18%/3%) | 1.590 | 0.973 | **1.63×** |

Correctness gate — all differential, same binary, device flipped:
- **Trajectory identity** on a moving articulated scene: max |Δ| **1.0e-06 m over 10,507 samples** (the probe's print resolution — i.e. identical).
- **Friction grasp** (force-mode): part lifted and held at z=0.128, byte-identical between arms.
- **8-Husky swarm**: max planar displacement **57.092 m on both arms**, 27 bodies moved, both PASS under `--fail-on-runaway`.
- Newton verdict sidecar present and non-degraded on every run.

## 2b. Status of the §3 list after executing it (2026-08-09)

| item | predicted | outcome |
|---|---|---|
| 2. gate `flushPendingNewtonRegistrations` | 0.097 ms | **LANDED**, 0.0973 → 0.0035 (−96%). Generation counter bumped by `OmSolid::postFinalize` + world teardown. Spawn path proven live: a Solid spawned at z=4.0 into a running world fell to z=0.0995. |
| 5. motor-target change detection | 0.025 ms | **LANDED**, 0.0252 → 0.0032 (−87%). Exact compare, NaN always pushes, cache dropped on every solver-world rebuild. |
| 3. cache per-tick env lookups | ~0.09 ms (over-estimated) | **LANDED**, −0.030 on the python call. ⚠ The first cut broke every tick by moving `import os as _impco` inside the caching branch while 8 later lines used the alias — and `run-headless` still printed **PASS** with the huskies motionless, because a log-only lane cannot see stalled physics. |
| 1. stop materialising newton State | 0.169 ms | **HALF LANDED.** Joint half shipped (clamp pre-checks mj `qpos`/`qvel`; equivalence 1.5e-05 rad / 1.2e-07 rad/s) for −0.013. **Body half BLOCKED**: `|xpos − body_q|` up to 2.8e-02 m on articulated robots. |
| 4. trim the C++ post writeback | part of 0.234 ms | **NOT THE COST (rendering part).** `OMNISIM_NEWTON_SKIP_WREN` A/B shows no win above a 2× block-variance floor. The bucket's real composition is unattributed — decompose it before attempting again. |
| 6. conditional `mj_step1` | 0.107 ms @50 boxes | **NOT ATTEMPTED** — it is the freshness precondition for raycast sensors, and with the body half blocked there is no second consumer to justify the gating work yet. |

## 3. What is REACHABLE, priced against the measurements above

Robot-world tick after the device pin ≈ **0.85 ms**. Remaining removable mass, in priority order:

1. **Stop materialising newton State on read-only ticks — ~0.17 ms. ⚠ HALF DONE, HALF BLOCKED (see §2b).** The joint half shipped. The body half requires answering: *why does newton's `body_q` differ from MuJoCo's `xpos` by up to 2.8e-02 m on an articulated robot, when they agree to 2.7e-07 on free bodies?* Candidate explanations — a COM/inertial-frame offset (`xipos` vs `xpos` vs newton's body origin), a joint-frame convention difference, or the comparison sampling the two at different points in the tick. Until one is confirmed **with a number**, the body half stays blocked; the probe term to reproduce it is in `_mj_pose_check` under `OMNISIM_NEWTON_MJ_POSE_CHECK`.
2. **Gate `flushPendingNewtonRegistrations` behind a pending counter — 0.097 ms (11%).** After world build it still walks every Solid re-running ancestor `dynamic_cast` chains, forever. Early-return when nothing is pending. Low risk (only `/scene/spawn` adds mid-run). Worth 4% on box scenes, **11% on robot scenes** — it scales with Solid count, so it grows with fleet size.
3. **Trim the remaining per-tick Python glue — up to ~0.09 ms of 0.180.** Cache the ~12–15 per-tick `os.environ.get` lookups; drop the base guard's two full-array copies via double-buffering (⚠ it is booby-trapped: the guard's reads double as the readback caches, which is why removing it *slowed things down* in the ablation).
4. **The C++ post writeback — ~0.23–0.32 ms, and now the LARGEST unattributed item.** Tier 1a (one packed FFI crossing) landed and helped; Tier 1b (bitwise skip) never fires; the rendering pushes are **measured not to be the cost** (§2b). What remains inside it is unmeasured: field writes + `changedByOde` signal emits, `setMatrixNeedUpdate` cache invalidation, the frame projection math, and the velocity field writes. **The next step here is a sub-profiler inside `OmSolid::postPhysicsStep`, not another guess** — that is precisely how `flushRegistrations` went from "0.097, worth fixing" to "0.0035, done" and how the WREN theory was killed in one run.
5. **Motor-target change detection — 0.025 ms (3%).** Push only deltas. Small but nearly free to implement.
6. **Make the extra `mj_step1` conditional — 0.107 ms at 50 boxes (14%), ~0 at 5.** Measured via `OMNISIM_NEWTON_STALE_CARTESIAN=1`. It exists to keep `mj_data` Cartesian arrays fresh for raycast sensors (DistanceSensor/LightSensor/Receiver/Radar) — so gate it on a raycast consumer existing, don't delete it.

**Arithmetic, revised after executing the list.** The pre-execution estimate was 0.85 − (0.17 + 0.09 + 0.09 + 0.05 + 0.02) ≈ 0.43 ms ⇒ ~4×. Actual: **0.827 ms ⇒ 2.52×**. The gap is entirely the two items that turned out closed rather than pending — the body half (0.17, blocked on a frame discrepancy) and the writeback (0.05 assumed recoverable from rendering, measured ~0). The items that *were* mechanically true all delivered within noise of their estimates.

So the remaining path to 4× on a robot world (0.827 → ~0.52) is **not a list to execute** but two questions to answer first:
1. the `body_q` vs `xpos` frame discrepancy on articulated robots (unlocks ~0.17), and
2. the composition of the post bracket (~0.23–0.32, currently unattributed).

The **hard floor** is `mj_step` (0.145–0.15) plus irreducible writeback — call it ~0.25 ms, ≈8× vs the 2026-08-08 engine — and it requires the engine to do nothing but step and draw, which is not a real target.

`mj_step` is now **~33% of the PHYSICS bracket** (0.1499 of 0.4525, both on the chrono clock) and **~18–22% of the whole tick** depending on which instrument the post bracket is taken from — up from ~9% before this campaign. ⚠ Quote whichever you mean and say which; an earlier revision of this line said "~30% of the tick", which silently mixed the two instruments §1 warns about. Either way: the era of easy multiples on this path is over.

## 3b. Re-measured 2026-08-15 on the CURRENT runtime — the §1 table is STALE

§1 is dated 2026-08-09 and predates both the newton 1.5.0 upgrade (worth ~20% on
this exact path) and the fixes §2b landed. Re-run on the same world
(`newton_husky_swarm_drive.omniworld`), same instrument, 27,500 steps steady state,
machine `9722d23d12a3`:

| component | 2026-08-09 (§1, CPU pin) | 2026-08-15 | note |
|---|---:|---:|---|
| `mj_step` | 0.1454 | 0.1614 | the floor |
| newton↔mjc conversion (`state_out`) | 0.1687 | **0.2683** | **now the LARGEST item, 44%** |
| `flushPendingNewtonRegistrations` | 0.0973 | 0.0113 | §2b fix holding |
| motor target push | 0.0252 | 0.0044 | §2b fix holding |
| whole PHYSICS bracket | ≈0.85 | **0.6134** | 1.39× better than the table above |

**The ranking changed: `state_out` overtook the post writeback.** `mj_step` is
26% of the bracket. Quote these numbers, not §1's.

**⚠ THE NOISE FLOOR ON THIS BENCH IS ~6%, MEASURED.** Two interleaved runs of the
IDENTICAL arm read 0.6207 and 0.6598 ms/tick. Nothing under ~10% is claimable
here without many repeats — which retires a whole class of micro-optimisation on
this path (e.g. hoisting the per-tick `max(slot_to_real_idx)` and Python loop out
of `readback_packed`, worth an estimated 2–5%: real, but unmeasurable here, so
shipping it would be shipping a belief).

### Re-tested: `OMNISIM_NEWTON_MJ_DIRECT` is STILL a loss (verdict holds)

§4 marks the hand-rolled mj_data fill dead on a 2026-08-09 measurement. Because
that predates the runtime upgrade, it was re-run rather than inherited —
interleaved, 2 pairs:

    fk (default)   bracket 0.6403   state_out 0.2785
    direct         bracket 0.6521   state_out 0.2841

Still a loss, and the same-arm drift (0.6207→0.6598) exceeds the arm difference.
**The verdict stands; do not re-propose.**

### `setMatrixNeedUpdate` — the top remaining suspect, and why the OBVIOUS fix is UNSOUND

`OmSolid::postPhysicsStep` calls `setMatrixNeedUpdate()`
([OmSolid.cpp:2158](../../src/omnisim/nodes/OmSolid.cpp)) per body per tick.
`OmPose::setMatrixNeedUpdate` sets the flag then recurses into EVERY child
([OmPose.cpp:166](../../src/omnisim/nodes/OmPose.cpp) →
[OmGroup.cpp:232](../../src/omnisim/nodes/OmGroup.cpp)) with **no early-out**, and
`OmBasicJoint` carries the recursion through joints into child Solids. On a Husky
the chassis invalidates the whole robot subtree, then each of the 4 wheels
invalidates its own subtree again — every tick. Same shape as
`flushPendingNewtonRegistrations` before its gate.

⚠ **But `if (mMatrixNeedUpdate) return;` is NOT a valid early-out.** It assumes
"flag set ⇒ all descendants set", and `updateMatrix()` clears the flag PER NODE
([OmAbstractPose.cpp:239](../../src/omnisim/nodes/OmAbstractPose.cpp)). The moment
any descendant's `matrix()` is queried, that descendant is clear while its
ancestor is still set — so the early-out would skip re-invalidating it and leave
it serving a STALE world matrix to supervisors and sensors, silently and
intermittently. A sound version needs an epoch/generation stamp on the pose cache
rather than a per-node bool, which is a real refactor of every matrix consumer.

**Price it before building it**, and by ABLATION rather than by probe: a
measurement-only `OMNISIM_NEWTON_SKIP_MATRIX_INVAL` around
[OmSolid.cpp:2158](../../src/omnisim/nodes/OmSolid.cpp) costs zero instrument
overhead, exactly as `OMNISIM_NEWTON_SKIP_WREN` did when it killed the rendering
theory in one run. ⚠ Also note the post bucket is measured on `OmPerformanceLog`,
which §1 records reading ~1.8× high vs `steady_clock` — so re-instrumenting it on
the chrono clock will show a "shrink" that is an instrument change, not a win.

### 3c. The post bucket, PRICED by ablation (2026-08-15) — and the verdict on this axis

Two measurement-only switches added to `OmSolid::postPhysicsStep`
([OmSolid.cpp](../../src/omnisim/nodes/OmSolid.cpp)), following the
`OMNISIM_NEWTON_SKIP_WREN` precedent — ablation, not a probe, because a
per-Solid timer would itself be ~15–20% of the quantity being measured:

| switch | what it drops | measured |
|---|---|---:|
| `OMNISIM_NEWTON_SKIP_MATRIX_INVAL` | the per-body subtree matrix invalidation | **0.034 ms, 5.5%** |
| `OMNISIM_NEWTON_SKIP_VELOCITY` | velocity readback + the two field writes | **0.048 ms, 7.9%** |

⚠ **Both are measurement switches, never shipping modes** — they make
`supervisor.getPosition` / `getVelocity` stale. Read WITHIN-round (the arms were
interleaved): skipmatrix scored −4.8 / −6.2 / −5.5 % across three rounds, i.e.
consistent; the absolute means drift because the box drifts.

**So the post bucket is ~13% of the tick in observer-only work, not a multiple.**

### What was RULED OUT this round, each by measurement

| lever | result |
|---|---|
| substep batching | **already ACTIVE and already worth 1.54×** (0.833 unbatched → 0.541 batched). Not a new lever; verify it stays on. |
| `OMNISIM_NEWTON_MJ_DIRECT` (§4 "dead") | re-tested on the new runtime because the verdict predated it — **still a loss** (0.652 vs 0.640). Verdict stands. |
| `OMNISIM_NEWTON_STALE_CARTESIAN` | **~0 on a robot world** (0.573 vs 0.563). The extra `mj_step1` is not meaningfully paid here, so "the freshness for a direct mj_data readback is already bought" is FALSE on this world. |
| hoisting `max(slot_to_real_idx)` + the Python loop out of `readback_packed` | real, but ~2–5% — **below this bench's noise floor**, so shipping it would be shipping a belief. |

### `state_out` is 43% of the tick and it is UPSTREAM

The largest single item is `SolverMuJoCo._update_newton_state`
(`newton/_src/solvers/mujoco/solver_mujoco.py:4869`), **vendored newton, not our
code**. Its CPU branch allocates two fresh warp arrays per call
(`wp.array([mj_data.qpos] …)`, `[mj_data.qvel]`) — the "2 launches + 2 creates"
this file already notes at the device-pin comment, ~0.032 ms at the measured
0.0158 ms/create. Reducing it means either forking the vendored tree (a
liability this repo has deliberately avoided) or not calling it — and the
readback consumes the State it produces.

### ⚠ MEASUREMENT HYGIENE: this box cannot resolve <10% while it is in use

The noise floor is **~6% on an idle box** and degraded to **~24%** during
interactive use (Chrome + VS Code + cpptools consuming CPU; GPU was 62 °C at 0%,
so this is CPU contention, NOT thermal). A combined-ablation ceiling test taken
in that window returned paired savings of −15.7 %, −5.0 %, +20.9 % — noise, not
data, and it was discarded rather than reported. Take step-cost numbers on a
quiet box or not at all.

### Verdict on this axis

Between the plan's own conclusion, the items above and §4, **there is no
remaining breakthrough on the CPU tick path.** Every large lever is either
banked (device pin, batching), dead (MJ_DIRECT, FK replacement, WREN), upstream
(`state_out`), or blocked on a refactor. The two candidates that could still
yield a real multiple are both PROJECTS, not tweaks:

1. **An epoch/generation stamp on the pose-matrix cache** — unlocks the 5.5%
   soundly (see §3b for why the one-line early-out is unsound) and removes a
   per-tick subtree walk that grows with fleet size. Touches every matrix
   consumer.
2. **Cloth → pure VBD + CUDA graph capture.** Cloth is the worst runtime in the
   tree by two orders of magnitude (44 ms/step vs 0.54 ms/step for 8 robots) and
   `_cloth_graph_ok()` refuses capture solely because the "mjc" entry is the CPU
   `mj_step` solver, whose per-substep device→host memcpy cannot be recorded.
   A cloth world whose rigid bodies are ALL STATIC needs no rigid dynamics at
   all; routing it to the existing `n_bodies == 0` pure-VBD path would unlock
   the capture the runtime already documents as 6.7 → 164 fps. ⚠ Physics-
   affecting, and it is the cloth lane's active surface — plan it with them.

## 4. What is DEAD — measured, do not re-propose

| item | verdict | evidence |
|---|---|---|
| Replace `eval_articulation_fk` with a hand-rolled mj_data fill (Tier 1c) | **Loss on both devices** (+0.19 ms at N=5, +0.29 at N=50 on CPU device) | ablation 2026-08-09; kept opt-in `OMNISIM_NEWTON_MJ_DIRECT=1`, values verified identical to FK at 1.6e-07 |
| Vectorise the joint-limit clamp | **Free already** on the CPU device (delta within noise) | ablation `noclamp` |
| Remove/cheapen the base-divergence guard | **Removing it is SLOWER** (+0.14 ms at N=50) — its reads double as the readback caches | ablation `noguard`; also two prior failed attempts (`a6aa9e54`, `7b3762f7`) |
| Bitwise dirty-skip of the scene-graph writeback (Tier 1b) | **Almost never fires** — no body sleep in MuJoCo, resting bodies jitter in low bits | shipped in `6e55cc777`, no measurable gain |
| `mjThreadPool` CPU threading | **Unavailable** — the bundled mujoco 3.8.1 wheel exposes no ThreadPool binding | verified `dir(mujoco)` |
| The "redundant ODE pass" | Structurally zero — `OmSimulationCluster::step()` is an empty stub | code + prior measurement |
| C++→Python crossing overhead | 0.004–0.011 ms/tick. Not the problem, never was | step profile |

## 5. Untouched by this work (and therefore unquantified — do not quote a number)

- **Batched GPU training (`mujoco_warp`) — MEASURED 2026-08-11 on this box, and the headline lever is DEAD.**

  Raw rows: [`tests/benchmarks/omnibench/lane2/results/throughput.jsonl`](../../tests/benchmarks/omnibench/lane2/results/throughput.jsonl),
  the ten `tier: "sim_train"` records stamped `utc` 2026-08-11T19:41–19:55Z (machine
  `9722d23d12a3`, engine binary sha256 `82d5964335feaeaf` on **all ten** — two different
  `build` labels appear because an unrelated commit landed mid-sweep, so the 7.15× is a
  single-binary comparison despite that). ⚠ **The rows do not record the solver-iteration
  arm**, so the row→config mapping in the table below lives only in this prose; it is
  recoverable because each rate below is unique, and a future sweep should stamp the arm
  into the record instead of relying on that. This was written up as 2026-08-09 until the
  rows were committed and dated it 08-11.

  | config (lane-2 tier C, go2, 3060 6 GB) | env-steps/s |
  |---|---:|
  | 256 envs, default iterations (100/50) | 10,068 / 10,576 / 11,182 |
  | 256 envs, capped (10/8) | 9,253 / 10,284 |
  | 256 envs, **extreme (1/1)** | **10,950** |
  | 1024 envs, default → capped | 39,091 → 37,049 |
  | 2048 envs, default | 54,413 |
  | 4096 envs, default | **75,659** |

  **Solver-iteration caps buy NOTHING here, and the mechanism is confirmed:** `iterations=1` performs the same as `iterations=100`, so the cap was never binding — mjwarp's solver early-exits on convergence (`graph_conditional`) long before 100. Upstream's "5–10 iterations" is a convergence budget, not a speed lever on this stack. ⚠ Scope: measured to 1024 envs on a 6 GB 3060; this is **not** a refutation at 4096+ on a 4090-class GPU where the solver may dominate. Knob verified live via `OMNISIM_NEWTON_DUMP_MJMODEL` (`iterations=100` vs `7`) — an unverified knob makes an A/B meaningless.

  **What DOES scale is batch size, and it needs no physics validation at all** (each env steps identically; parallelism is not a physics change, so no champion re-validation): **256 → 4096 is 7.15×** on ONE GPU (10,576 → 75,659). This decomposes the "~33× spread" lane 2 flags between the 3060 @256 and the 4090 @4096: roughly **7× is batch size and ~4.6× is the GPU**. Note the training recipes already default to `QUAD_ENVS=4096`, so real training runs at the good end — it is the *benchmark's* 256 default that understates our throughput by ~7×, and quoting "10,228 env-steps/s on a 3060" as "our training throughput" is wrong by that factor.

  **NEWTON RUNTIME UPGRADE — TAKEN, 2026-08-09 (commit `b56be84a0`). The evaluation below said DECLINE; the owner overrode it and the upgrade both worked and was FASTER.** Now on **newton 1.5.0 / warp-lang 1.16.0 / mujoco-warp 3.11.0 / mujoco 3.11.0**.
  - **The evaluation's central perf claim was WRONG.** All three audits concluded 1.5.0's gains were *initialization*-time and *batched-GPU* only and would not reach a CPU single-world engine. Measured on the step_cost bench: **5 boxes 0.446 → 0.3785 ms/step (1.18×), 50 boxes 1.110 → 0.9184 (1.21×)** — ~20% off the per-step cost of exactly that path. Cumulative against the 2026-08-08 engine: **~4.2× at 5 bodies, ~2.5× at 50**. Read this as the standing caution about upstream release notes: they describe the workloads upstream measured, not yours.
  - **#3805 was worked around, not waited out.** Disabling MuJoCo's mid-phase on the CPU solver is sound because mid-phase is a pure *acceleration* structure — the resulting contact set is a superset of the culled one, so correctness improves and only pair-test count rises. Auto-scoped to newton ≥1.3 + CPU; `OMNISIM_NEWTON_MIDPHASE` overrides; the log line reports `iquat_nonidentity` (32 on the husky world, so the stale-BVH sync had real material). A forced on/off A/B produced identical rest heights.
  - **Physics verified unchanged**: husky swarm 56.579 m (identical), friction grasp z=0.128 (identical), rest height 0.600 (identical), determinism **bitwise** on both pairs, pinned suites 33 passed / 1 pre-existing failure, GPU `mujoco_warp` finalises on `cuda:0` and drives.
  - **Four API breaks, three unpredicted by the audits** — `add_link(armature=)` removed; **`Control.joint_target_pos/vel` removed** (a 64,775-warning flood on the first run); `builder.joint_target_vel` renamed; and **`mj_fullM`'s signature changed** in mujoco 3.9+. All migrated *version-tolerantly*, so one source drives both runtimes and rollback is two directory renames.
  - ⚠ **Traps for the next bump**: the bundle is gitignored, so the commit upgrades nobody by itself; a cold warp kernel cache made the GPU world need `--duration 300` to finalise the first time (a missing sidecar there proves nothing); and an empty orphan controller directory shadowed a moved controller, making a robot silently not move — which looked exactly like a physics regression until the log was read.

### Superseded evaluation (kept because its reasoning was sound even where its conclusion was not)

**NEWTON RUNTIME UPGRADE — EVALUATED AND DECLINED, 2026-08-09.** Three parallel audits (upstream release research, in-tree API blast radius, migration runbook) priced a 1.2.0 → 1.5.0 bump. **Verdict: hold at 1.2.0.** Reasons, in order of weight:
  1. **[newton#3805](https://github.com/newton-physics/newton/issues/3805) is open, unfixed, and lands on our default path** — `SolverMuJoCo(use_mujoco_cpu=True)` leaves MuJoCo's collision BVH in a stale inertial frame after a post-compile `body_iquat` sync, **silently culling valid contacts**. Source-verified: the offending `_sync_mjw_inertias_to_mjc_cpu` has **0 occurrences at tag v1.2.0, 2 at v1.4.0 and v1.5.0**. Upgrading imports a regression onto the CPU `mj_step` path that every demo, CI lane and determinism claim runs on.
  2. **The gains do not accrue to this engine.** 1.5.0's headline is *initialization* time (11–24%) and init memory (10–15%) on **batched replicated** workloads; 1.4.0's 15–26% is **large-batch GPU**. The much-quoted 16× CPU graph-replay figure is an **XPBD** workload — a solver this repo deleted. A CPU single-world engine collects essentially none of it.
  3. **1.4.0+ propagates `shape_gap` into MuJoCo `geom_gap`** with `rigid_gap = 0.1` inherited by every shape. Combined with MuJoCo 3.9's changed semantics (detection at `dist < margin + gap`), every geom gains a **10 cm detection envelope**. Forces stay correct, but our contact readback does not filter on `efc_address >= 0`, so `/sim/contacts`, `getContactPoints` and `/sim/grips` would report every pair within 10 cm as *touching* — silent grip-inference corruption. ⚠ The obvious mitigation is the wrong one: `legacy_margin_gap=True` applies only to `add_mjcf()`/`add_usd()` and we build programmatically; the correct lever is `builder.rigid_gap = 0.0`.
  4. **newton ≥1.4 removed `ModelBuilder.add_link(armature=...)`**, which the engine passes at three sites (all `0.0`, so the fix is deletion), and three RunPod launchers hard-assert `newton == "1.2.0"`.
  **Revisit trigger: #3805 fixed.** If it moves, go to **1.5.0, not 1.4.0** (1.4.0 carries the same bug and pins `mujoco~=3.10.0`, a dead-end with one release). The version quad is clean when the day comes — `newton==1.5.0`, `warp-lang==1.16.0`, `mujoco-warp==3.11.0`, `mujoco==3.11.0`, all with cp312 Windows wheels, and every `SolverMuJoCo` kwarg we pass survives — but **all four must move atomically**: a newton-1.5/mujoco-3.8.1 mix yields 10 cm of free penetration behind a `RuntimeWarning` our GUI-subsystem binary discards.
  **Banked from the evaluation** (`f4f0f960c`): the tree's only `newton._src` dependency is deleted, a transposed-argument `mj_fullM` bug is fixed at two sites, and the drifted deploy-runtime mirror is re-synced. Two hazards recorded rather than fixed — the engine hand-inlines `SolverMuJoCo.step()`'s CPU branch **default-ON** (upstream reordering it would diverge physics silently, so any future bump wants a startup assert), and a bundle-only bump would split the engine's embedded interpreter from the system python every controller and trainer uses, which `test_newton_pins_parity.py` structurally cannot see.

  Still unmeasured and still physics-affecting: `ls_parallel`, `nconmax`/`njmax` right-sizing (**deferred by owner decision, 2026-08-09** — it changes contact defaults and would invalidate the champion evidence base; evaluate in a scratch environment before adopting).
- **`mujoco_warp` at nworld=1** remains ~9× slower than CPU `mj_step` by upstream design; the CPU path is and stays the single-world answer.
- **`finalizeWorld` — PROFILED 2026-08-09, and it is ONE THING.** Phase timers (`OMNISIM_NEWTON_STEP_PROFILE=1`, reported as `finalize phases:` at world finalize) decompose it as: `topology` 1–9 ms + `add_articulation` 0 ms + `builder.finalize` 8–10 ms + **`SolverMuJoCo` construction 1205–1402 ms (98%)** + `state_alloc` 1 ms. Measured 1.22–1.33 s total on 5 boxes, 50 boxes and the 8-Husky world alike — **constant in scene size**, which is the signature of module loading rather than model compilation (warp's own kernel-module load measured 1036 ms on cuda / 1542 ms on cpu standalone).
  **It is a per-PROCESS cost, not a per-world one**: a second world load in the SAME process costs **27 ms** for that phase (45× cheaper). So harness hot-reload loops already avoid it (32 ms finalize, and their ~1.37 s/load wall time is parse + controller start + supervisor connect, not physics), while every FRESH engine process pays it — `run-headless`, CI smoke, AgentBench cells, and each benchmark window.
  Two corrections this measurement forces: the long-quoted **"1.9–4.9 s" is high for these worlds** (1.2–1.5 s measured today, 98% in one call), and the **CPU-device pin does NOT tax startup** — cpu 1283–1328 ms vs cuda 1337–1485 ms, with `builder.finalize` 9 ms on cpu against 67–80 ms on cuda.
  **Actionable, in order:** (a) overlap the module load with world parsing / controller startup on a background thread — it is ~1.2 s of work that depends on nothing in the `.wbt`; (b) check whether `SolverMuJoCo` loads mjwarp kernel modules the CPU `mj_step` path never executes, which would shrink the load itself rather than hide it.
- Force-mode substep batching (RL torque deploy) — still gated on the unexplained grasp-slip divergence.

## 6. Measurement protocol (binding — this is how the first campaign went wrong)

1. **Idle box only.** Verify no foreign `omnisim-bin` and CPU < 30%. A whole-repo `git blame` sweep once masqueraded as ambient load and would have poisoned everything.
2. **Interleave arms at cell level** and report per-block spread. Sequential A/Bs in this tree have drifted 5.7→9.7 ms and been discarded.
3. **Difference two windows** (200/1200 steps) so finalize and warm-up cancel; cumulative averages are contaminated by one-time cost (`flushRegistrations` reads 1.17 ms/tick cumulative and **0.0115 ms/tick** steady-state — a 100× misread if quoted raw).
4. **Prefer an env-flip A/B on ONE binary** over two builds when the change allows it: it removes build skew entirely.
5. **Price the primitive before optimising the computation.** `scratchpad/floor_bench.py`-style microbenchmarks (bare `mj_step`, each warp call, on each device) cost minutes and are what separated this campaign's real answer from the first one's wrong answer.
6. Every physics-affecting claim needs the differential gate in §2 (trajectory identity on a *moving* scene, force-mode grasp, swarm displacement, `--fail-on-runaway`, sidecar-proven Newton).

## 7. Instruments built for this (reusable)

- **`projects/default/controllers/perf_window_runner`** — steps any world exactly N times then quits, from the global controller path, so the two-window differencing method works on **shipped robot worlds**, not just generated box stacks. This closes the "no articulated-robot step_cost sweep" gap the benchmark docs have carried since the first campaign.
- **Bracket sub-profiler** in `OmSimulationWorld::step` (`OMNISIM_NEWTON_STEP_PROFILE=1`): `flushRegistrations + motorTargets + newton->step` against a whole-bracket timer on the same clock, so "unattributed" is visible rather than assumed.
- **`OMNISIM_NEWTON_MODEL_DEVICE`** — the device A/B switch.

If this doc and the code disagree, the code wins — update this doc in the same change.
