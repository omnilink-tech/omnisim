# v6 readiness — retiring ODE, and publishing measured competitor comparisons

> ## ⚠️ SUPERSEDED 2026-08-08 — GOAL (a) EXECUTED, AND THIS DOCUMENT'S CENTRAL WARNING CAME TRUE
>
> **Read [ode-retirement-campaign.md](ode-retirement-campaign.md) instead** for what
> happened. `bdc02139` deleted the ODE backend (`src/ode/` + `include/ode/`, 106,283
> lines) two days after this assessment was written. Newton with `SolverMuJoCo` is the
> only physics backend.
>
> This document's single most important finding was that the two v6 goals are **not
> independent** — that retiring ODE *"destroys the methodology (b) currently rests
> on."* **That was correct, and it is now realised rather than hypothetical:**
>
> - **OmniBench lane 1 used ODE as its second in-engine arm** and its generator
>   emitted an `ode_pin` per scene. There is **no second in-engine path to
>   cross-check the plumbing any more** — the oracle itself, analytic ground truth,
>   is untouched, and bare MuJoCo and PyBullet still run ([correctness-scope.md](../benchmarks/correctness-scope.md)).
>   Frozen values survive in
>   [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json)
>   — a golden file, not an oracle.
> - The honest form of the batching claim (K-world Newton vs K sequential ODE runs)
>   is **no longer buildable**; the only remaining substitute measures *the batching*,
>   not *the engine choice*.
> - Goal (b) — publishing measured competitor comparisons — therefore proceeds with
>   **one fewer in-house arm than when this was written**. The surviving cross-*simulator*
>   arms (MuJoCo 3.8.1, PyBullet) are now the only external check on lane-1 scenes.
> - The four external commitments this doc flagged (**D2** AgentBench Phase W's
>   hash-frozen `defaultPhysicsBackend "ode"` fixtures, **D3** the external WP5 no-GPU
>   deliverable that names ODE as the mechanism, **D4** macOS having no physics
>   backend without ODE, **D5** the plugin-ABI break) were **not resolved before the
>   deletion shipped.** D5 executed; D2, D3 and D4 remain open and are now live
>   consequences. See the D2–D5 block in the campaign doc.
>
> Everything below is preserved as the 2026-08-06 assessment. Its evidence and
> citations stand; its *proposals* have been overtaken and its present-tense
> statements about ODE being available are false.

**Status (as written 2026-08-06): assessment, not a plan. Nothing here is a decision.** Produced 2026-08-06 at
HEAD `85bf57da` by a read-only sweep of the tree. Every claim below cites a file; where the
tree does not contain the evidence, it says so. Implementation work is happening in other
sessions — this document exists so that work is not planned against stale assumptions.

Two things are proposed for v6: **(a)** retire the ODE physics backend, and **(b)** publish
measured benchmarks against Webots, MuJoCo and Isaac Sim. The single most important finding
is that these two goals are not independent — **(a) destroys the methodology (b) currently
rests on.** That is the first section.

---

## 1. The collision: (a) invalidates (b)

The competitor comparison that is furthest along is AgentBench **Phase W**, against upstream
Webots R2025a. Its methodology is explicit that both arms must run the same solver family:

> "Both cells run ODE… The physics is genuinely held constant — and Phase W therefore says
> **nothing about our Newton default**." — `docs/developer/agent-edge-validation-plan.md:755`

Four scored task fixtures hard-pin it in-file — `tasks/{B1_overlap_audit,B2_subject_in_frame,
C1_parse_error_fix,C2_fall_through_floor}/initial/*.wbt` carry `defaultPhysicsBackend "ode"`,
and `six_huskies.wbt:11` names the rule in a comment. **Upstream Webots' engine *is* ODE.**

Remove ODE and there is no shared solver family left to hold constant. The comparison does not
merely need re-running — its stated basis for fairness disappears. And it cannot be quietly
patched: the campaign is **pre-registered and hash-frozen** (`preregister/freeze_manifest.json`,
enforced by `test_freeze.py`, which fails on drift). Amending it is a deliberate, on-the-record
act; publishing it unamended after retiring ODE is not defensible.

**Decision required:** does Phase W get re-run on Newton against Webots-on-ODE and its
methodology section rewritten to own the asymmetry, or is it amended under its own freeze
procedure, or is it dropped from v6? Someone must choose, in writing.

---

## 2. "ODE" names two different things, and only one is retirable

This is the finding that most changes the size of the work.

**As a solver**, ODE is selectable and bounded: `OmPhysicsBackendRegistry::resolve()`
(`src/omnisim/physics/OmPhysicsBackend.cpp:304`) is nine lines, and retiring it means deleting a
branch and re-pointing 28 worlds.

**As the collision, ray-casting and sensing kernel, it is not a backend at all.** It runs
unconditionally on every tick of every world, whatever a world file says:

- `OmSimulationWorld.cpp:61` — `mOdeContext(new OmOdeContext())`, in the constructor, ungated.
- `OmSolid::createOdeObjects()` (`OmSolid.cpp:902`) — builds ODE bodies, geoms, masses and
  joints for **every** Solid, with no backend check.
- `OmSimulationCluster::step` (`:112`) — `dWorldStepAndSpaceCollide(...)`, ungated.

The engine's own comment is explicit: Newton is driven *"alongside ODE"* (`OmSimulationWorld.cpp:246`).
Newton registers a subset of bodies, steps them, writes poses back, and then
`setBodyArtificiallyDisabled(true)` freezes ODE's copy (`OmSolid.cpp:3548`).

Everything that perceives the world through collision still reads ODE's world: **DistanceSensor**
(`OmDistanceSensor.cpp:454`), **camera recognition occlusion** (`OmObjectDetection.cpp:72`),
**LightSensor**, **Receiver** line-of-sight, plus TouchSensor, Connector, VacuumGripper,
ContactSound and ElevationGrid. Scale: **50 files in `src/omnisim/` `#include <ode/...>`;
210 use ODE handle types** (`dBodyID`, `dWorldID`, …).

Two further consequences:

- **The public physics-plugin C API *is* the ODE API.** `include/plugins/physics.h:19` is
  `#include <ode/ode.h>` and its exported surface is ODE handles; `include/ode/ode/*.h` ships to
  users precisely so they can compile against it (`scripts/packaging/files_core.txt:9-11`).
  Removing ODE **deletes a documented public API** — an API-compatibility event, and ~9 reference
  pages describing it.
- **Fluid dynamics / buoyancy has no Newton implementation at all** — zero hits for
  `immersion|fluid|buoyan` in `src/omnisim/physics/`. `Fluid` and `ImmersionProperties` are
  implemented directly against a forked `ode/fluid_dynamics/` subsystem that does not exist
  upstream. Three shipped assets use it.

**Decision required:** is v6 retiring the *solver* (weeks, ODE still ships, ~106k lines stay) or
removing the *kernel* (a re-platforming that must first rebuild ray-cast sensing, contact sensing
and immersion on Newton)? These differ by an order of magnitude and the release notes cannot be
written until this is answered.

---

## 3. The no-GPU question — unmeasured, and load-bearing

`README.md` promises, six times (`:50, :79, :102, :114, :116, :235, :238`), that
**"everything loads and runs without a GPU"**, with ODE named as the mechanism.

A CPU path does exist without ODE: `newtonSolver "mujoco"` resolves to
`SolverMuJoCo(use_mujoco_cpu=True)` — reference CPU `mj_step`, no CUDA in the solve
(`OmNewtonBackend.cpp:1682`). Encouragingly, **341 tracked worlds already pin it.**

But three things are true at once and they do not add up to the promise:

1. **It is not the default.** The `newtonSolver` schema default is `""` → `SolverXPBD` on the
   GPU (`resources/nodes/WorldInfo.wrl:29`). ~82% of the tree's `.wbt` files pin no solver.
2. **Nothing in the init path checks for a GPU.** The availability gate tests only `import warp`,
   `import newton` and an FFI smoke; `builder.finalize()` takes warp's implicit default device and
   the `.newton.json` sidecar records no device field. Meanwhile `OmNewtonBackend.hpp:34` and
   `OmPhysicsBackend.cpp:298` *assert* that a box without NVIDIA hardware falls back to ODE —
   comments the code does not implement.
3. **It has never been run.** No test, no CI job, no measurement anywhere in the tree exercises a
   machine without CUDA. The three active workflows run on `ubuntu-latest` and never launch the
   engine.

**This is the cheapest high-value action available: run `omnisim-bin` on a GPU-less box with
`newtonSolver "mujoco"` and record the sidecar.** An afternoon's work that converts the largest
open question in the release from opinion to fact. Until then, `README.md:79` is an unverified
promise and v6 cannot honestly restate or retract it.

Note also `README.md:235`: macOS today is "untested for Newton, **falls back to ODE**". Without
ODE and without a macOS Newton bundling story (`scripts/packaging/` has none), macOS has no
physics backend at all.

---

## 4. What must close before retirement is honest

Newton gaps that are the *reason* worlds pin ODE. Each is a tree measurement, not an opinion.

| # | Gap | Evidence |
|---|---|---|
| 1 | Contact queries return nothing under Newton — **1008 vs 0** on the same scene; the engine's own warning tells users to pin ODE | `OmSolid.cpp:4140`, `:4157`; native contacts still opt-in at HEAD |
| 2 | Static floors/tables/walls **intangible by default** (`newtonStatics` FALSE) — ball settles at z=0.0996 on a phantom plane instead of 0.6496 | `WorldInfo.wrl:40`; 47 worlds work around it, none set it FALSE |
| 3 | `coulombFriction` — the standard inherited field, in **202 worlds** — silently ignored on Newton | `WorldInfo.wrl:32`; registration-order bug |
| 4 | Default friction cone gives a **wrong answer to a textbook statics problem**: 181 mm slide where elliptic holds at 0.6 mm | `WorldInfo.wrl:31`; T2 at 26° |
| 5 | External body forces (`add_force`, propeller thrust, mouse drag) do not act on Newton bodies | `docs/guide/newton-physics-backend.md:85` |
| 6 | Supervisor writes clobber solver state — why **16 of 28** ODE-pinned worlds are pinned | `docs/developer/policy-switching.md:353` |
| 7 | Velocity-mode motors and limit-less servos: ODE 0.500000 rad vs Newton **0.000000** | commit `4f2621bb` |
| 8 | **T5 momentum is structural, not a bug** — ODE conserves exactly by construction; Newton to integrator order. Not closable | commit `58856fa8` |
| 9 | GPU determinism **refuted**: 0 bitwise of 24 pairs, 9.152 m apart by 1000 steps | `determinism-scope.md` §1–2 |
| 10 | Correctness ordering still favours ODE: at default substeps **ODE 10 metrics to 2**; with substeps=4 Newton wins 4 of 11, **loses 5**, and no configuration wins overall | `omnibench-2026-07-24.md:213-272` |
| 11 | Constraint buffers overflow silently at fleet scale | `newtonNjmax` default 256 vs peak 320 |
| 12 | Cold-launch bring-up ~95%; the residual 1-in-21 is uncaptured — and with ODE gone it becomes **no simulation at all**, not a degraded one | commit `3f81bbc8`, own words: *"NOT a proof of a fix"* |
| 13 | Corpus is **~35–40% Newton-faithful**; the capability gate routes non-Hinge/Slider joints and mesh colliders to ODE | `newton-ode-replacement-plan.md:35` |

**Where Newton genuinely wins**, for balance: batched GPU throughput (ODE has no row at all),
T3 rolling accuracy, T4 energy and T7 angular momentum under substeps, bitwise determinism at
336 contacts on the CPU MuJoCo path, and train==deploy bit-exactness for the RL pipeline.

### The measurement that is missing and matters most

Every lane-1 world pins `newtonSolver "mujoco"`, and T2/T6 additionally pin `newtonCone
"elliptic"` + `newtonImpratio 10`. **A user's world gets none of that.** Newton's correctness
numbers are measured in its best configuration; the shipped default — XPBD, pyramidal cone,
intangible statics, invisible contacts, ignored friction — **has never been scored on the
correctness lane.** Until it is, "Newton is good enough to replace ODE" is a claim about a
configuration, not about the product.

---

## 5. What the tree has already committed to, in writing

v6(a) reverses four standing commitments. They must be retracted deliberately, not by silence:

- `engine-migration-plan.md:1662` — *"ODE is **never removed** — it stays the permanent fallback forever"*
- `engine-migration-plan.md:1222` — risk register: *"ODE remains canonical fallback **forever**"*
- `newton-ode-replacement-plan.md:23` — *"'Complete replacement' … **not 'remove ODE.'**"*
- `README.md:79`, `AGENTS.md:60`, and an **external funding commitment** whose work-package 5
  deliverable is a non-RTX/CPU fallback (the application itself is in the private ops
  tree, not in the public snapshot)

The last one is a commitment to a third party and should be checked by whoever owns that
relationship before the change is announced.

---

## 6. Three tools will assert "ODE drove this run" in a build with no ODE

The repo's own rule is that a tool returning a plausible wrong value installs a false belief an
agent then reports confidently (`AGENTS.md:51`). Three places turn that on themselves:

- `scripts/harness/omnisim_harness.py:594` — `/capabilities` fallthrough returns
  `"backend": "ode"` with the detail *"this means ODE drove the world OR the load never reached
  finalize"*. **Served to agents over HTTP.**
- `projects/policies/common/env_fingerprint.py:257` — default verdict on any unreadable log is
  the string `"ODE or unknown (no Newton finalise)"`.
- `omnisim/conformance/advisor.py:31` — emits `NEWTON_FELL_BACK_TO_ODE` at severity `"drift"`
  with the message *"ODE is supported"*. Surfaced by `doctor`.

Under v6 each fires on the *actual* failure — Newton failed to initialise — and reports it as a
benign supported configuration. Compounding it, the harness greps for the literal string
`"falling back to ODE"` (`OmNewtonBackend.cpp:4276`), which v6 deletes: the detector then fails
silently rather than loudly.

---

## 7. Competitor benchmark: what exists, what does not

| Target | State | Note |
|---|---|---|
| **Webots** | **Partial, furthest along** — verified install, launcher, recorder, evidence adapter, 72 passing tests, an `EXCLUSIONS.md` enforced by a completeness test, and a hash-frozen pre-registration. Published rows exist. | Physics comparison: **absent** — there is no Webots runner in OmniBench lane 1. See §1 for the collision. |
| **MuJoCo** | **Partial** — real runner, committed rows | The fairness problem is **documented but not neutralised**: OmniSim's per-step wall is timed inside the supervisor controller (IPC included); MuJoCo's is `perf_counter` around `mj_step`. |
| **Isaac Sim** | **Absent entirely** | No install, no adapter, no scene port, no row. Needs RTX 4080 / **16 GB VRAM**; owned machines are a 6 GB RTX 3060 laptop and a 12 GB 5070 Ti. **No owned machine clears the floor.** Cloud-only, owner-gated. |

**The published Webots rows do not currently favour us** — OmniSim 0/10 on the flagship authoring
task against Webots 4/5 on two others, with **15 of 35 OmniSim rows INVALID against 0 of 35
Webots rows**, and unequal n per cell. Publishing that honestly is defensible and probably
strengthens the project's credibility; publishing it selectively would not.

### The self-comparison trap

OmniSim's Newton backend **embeds mujoco-warp**, and lane 1 pins `newtonSolver "mujoco"` on all
156 worlds — so "OmniSim-Newton vs MuJoCo" is largely the same integrator answering twice. The
data shows the signature: T4 energy agrees to **1.6%**, T2 stick to five figures. `SPEC.md:149`
already forbids the "beating MuJoCo" framing — but that is a *prose rule about phrasing a row*,
and the row itself is labelled `engine: mujoco` vs `engine: omnisim-newton` in the same table
with the same metric. **The rule will not survive a screenshot.**

Worth noting: **OmniSim/ODE vs MuJoCo *is* a genuinely independent comparison** (different solver
family, no shared code) — and it is where we win most rows. v6(a) removes exactly the arm that
made the comparison honest.

### Methodology gaps that would be noticed publicly

- **n ≥ 3 does not exist in OmniBench.** `run_all.py` runs one combo per (backend, test, dt); no
  `--repeat`. The data proves it is needed: the same cell on the same machine recorded
  `wall_ms_per_step` **2.2273** and **0.1716** on consecutive days — a **13× spread**.
- **`--parallel` (default 4 on Linux) silently contaminates wall-clock**; nothing excludes or
  stamps timing rows with the concurrency they ran under.
- **Raw traces are not committed** — `omnibench_out.npz` is untracked, so a critic cannot
  re-score. The project's own rule is *"publish the traces or do not publish the number."*
- **PyBullet's version reads `"unknown"`** in every committed row (`getattr(p, "__version__", …)`
  on a C extension).
- **Machine ids fork under transient failure** — a timed-out `nvidia-smi` mints a new fingerprint,
  and machine id is the audit key for every published number.
- **Scaffolding bias has been pro-OmniSim twice**, both found only by running it (`1ae8f40a`,
  `39b74677` — one made a MuJoCo cell measure our naming convention; one filed *every* MuJoCo cell
  as our-scaffolding-broken). The base rate of undiscovered ones is not zero.
- Three of four self-imposed credibility commitments are **unexecuted**: no non-OmniSim reviewer,
  no 30-day correction window, no reproduce-or-drop. `SPEC` §8.2 names the SimBenchmark precedent
  — a suite discredited partly because its authors won most of its rows — as *"the single largest
  credibility risk in the design."*

### Minimum credible v6 scope

Publish **one** comparison, narrower than readers expect:

- **Engines:** OmniSim (both backends, reported separately) vs MuJoCo 3.8.1 vs PyBullet in
  lane 1; Webots in AgentBench Phase W as a **separate document with its own methodology**.
- **Drop Isaac Sim from v6 and say so:** *"not measured — we do not run this engine, our hardware
  does not meet its floor, and its terms have not been reviewed."*
- **Publish no cross-engine wall-clock.** The process-vs-library asymmetry has no fix in the
  current harness and the 13× intra-cell spread has no repeat discipline. Throughput only as an
  overhead ratio against raw mujoco-warp, never as a MuJoCo comparison.
- **n ≥ 3** with median and spread, run `--parallel 1`.
- **Ship alongside:** the frozen SPEC, `results.jsonl` *and* the raw recordings, machine
  fingerprints + binary sha256 + real library versions, the per-row deviations (including the
  non-symmetric friction-cone overrides), and a **hostile summary** — the strongest honest case
  that the comparison is unfair to competitors.

---

## 8. Licensing and packaging

**True and worth saying:** removing `src/ode/` removes the **only copyleft-licensed source in the
repository** (an LGPL-3-or-later unit-test framework inside `libccd/src/testsuites/`, already
deny-listed and never compiled). The licence scan can then drop its exclusion list.

**False — do not ship this claim:** *"OmniSim is now copyleft-free."* The distribution still
carries **Qt 6 (LGPL-3.0, 71 DLLs)**, **OpenAL Soft (LGPL-2.1)** and the MSYS2 runtime.

Concrete items:

- **`include/ode/` (34 files) must go with `src/ode/`** — otherwise headers survive pointing at
  two licence files that no longer exist.
- **NOTICE currently misstates the linkage**: it says ODE is *"linked statically"*; release and
  debug builds link `-lode` against a **shared** library (`src/omnisim/Makefile:128-133`; the
  1.34 MB `ode.dll` ships). Harmless under the BSD election, but false in a legal notice.
- **`NOTICE:504` says the CU framework is "LGPL-3-ONLY"**; its header says *"or (at your option)
  any later version"*. Wrong today, in a shipped file.
- **Four derived MJCF files owe attribution** — `projects/policies/research/training/mjcf/`
  contains Unitree- and Clearpath-derived models with **no licence text beside them**, two of them
  carrying inline `<mesh vertex="…">` geometry, and `go2_newton.xml` is the model **lane 2
  publishes throughput against**. `NOTICE:527` promises the opposite.
- **The dangling-reference checker cannot catch this removal.** `publish_snapshot.sh:424` builds
  its manifest only from *deny-list* matches; files removed by an ordinary `git rm` are invisible
  to it, and the check is advisory unless `PUBLISH_STRICT_DANGLING=1`. **Run
  `git grep -n 'src/ode' -- . ':!src/ode'` manually against the v6 snapshot before publishing.**
- **`LICENSE`, `NOTICE` and `THIRD_PARTY_NOTICES.md` are in no packaging manifest** — a
  pre-existing gap v6 does not fix and could cheaply close.

---

## 9. The cheapest next actions

Ranked by information gained per hour:

1. **Run OmniSim on a GPU-less box** with `newtonSolver "mujoco"`; record the sidecar. Answers §3.
2. **Re-run `scripts/dev/newton_coverage.py`** for a current corpus number (the ~35–40% figure is
   the KPI the whole retirement plan is scored against). *Note: it needs a built binary at
   `WEBOTS_HOME`; it fails with `FileNotFoundError` if the engine is not where it expects.*
3. **Score lane 1 under the shipped Newton default** — no `newtonSolver` pin, no cone pin. This is
   the missing measurement that decides whether retirement is defensible at all.
4. **Decide §1** — Phase W's fate — before any further competitor-benchmark work is planned.
5. **Decide §2** — solver retirement vs kernel removal — before any release note is drafted.

---

## Appendix: claim surface

A full file:line inventory of every statement v6 invalidates was produced alongside this
document — roughly 230 entries across five groups (ODE-as-fallback, ODE-as-correctness-reference,
determinism scoped to ODE, competitor claims, and version-pinned prose), filtered from ~31,000 raw
matches. The highest-risk four:

1. **Phase W's ODE pins** (§1) — the two halves of v6 colliding.
2. **`AGENTS.md:60` and `README.md:79`** — the fallback promise on the two turn-one surfaces. An
   agent that believes them will debug a v6 hard-refusal as a bug and reach for four env vars and
   a build flag that no longer exist.
3. **The three tools that assert ODE drove a run** (§6).
4. **`omnibench-2026-07-24.md:255` vs `:276`** — the benchmark report already contradicts itself
   (Newton wins 4 of 11 at `:255`; *"omnisim-ode is the best-scoring integration"* at `:276`
   — reframed 2026-08-08, [correctness-scope.md](../benchmarks/correctness-scope.md)), and the
   entry-point docs all quote the second. The 109× T6 creep gap is declared **unquotable** at
   `:267` yet still printed as like-for-like in two other docs.

**Propagation note:** `performance-comparison.md:178` and
`archive/newton-default-and-omnisim-rename-plan.md:21` do not merely *state* the
ODE-as-correctness-reference framing — they **instruct other docs to adopt it**. Re-scope those
two first or the claim regrows.
