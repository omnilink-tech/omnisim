# OmniSim Developer Docs

This directory is the implementation-facing documentation set for OmniSim contributors. It is meant to be read while changing code, validating a fix, or planning subsystem work.

## Start Here

- [quickstart.md](quickstart.md): first local build, runtime setup, and the basic fast-path commands.
- [architecture.md](architecture.md): short architectural map of the desktop shell, runtime, parser, renderer, and controller layers.
- [agent-map.md](agent-map.md): repository navigation guide for coding agents and humans doing targeted code search.
- [simulator-comparison.md](simulator-comparison.md): **how OmniSim compares to Isaac Sim/Lab, Gazebo, MuJoCo/MJX, Webots, Genesis, ManiSkill, PyBullet, CoppeliaSim, and Unity/Colosseum** — licence, physics, rendering, RL, ROS, hardware floor, and maintenance status, with each claim marked verified / unaudited / vendor-claim. Carries the honest losses (no ROS 2 bridge, not photoreal, sim-to-real unproven) and the defensible positioning paragraph. The throughput companion is [../benchmarks/performance-comparison.md](../benchmarks/performance-comparison.md). **Read before making any public comparative claim.**
- [ros2-integration.md](ros2-integration.md): **the ROS 2 stance — a deliberate, documented non-goal.** OmniSim's agent interface is the HTTP wire protocol, not ROS; this doc states the decision, says what it costs (no `webots_ros2`, absent from the `ros2_control` registry), recommends Gazebo for ROS-native work, and gives a working external ROS 2 ↔ OmniSim bridge recipe for the cases that need it. The MCP front door to the same surface is [`packages/omnisim-mcp/`](../../packages/omnisim-mcp/).

## Robot Learning & Motion

**Read in this order.** ⭐ **[shadowing.md](shadowing.md)** is the FLAGSHIP method; **[rl-current-state.md](rl-current-state.md)** is the CANONICAL status (never quote a result from any other doc).

- [shadowing.md](shadowing.md): ⭐ **SHADOWING — the flagship algorithm for legged-robot motion (maintainer directive 2026-07-03). READ THIS FIRST for any new motion-control / imitation effort.** Train in-engine (train == deploy bit-exact) to shadow a **ghost** (a dynamically-feasible reference) via **corridors** + **WBMATCH** + **GHOST-MORPH**. The thesis: *ghost feasibility, not RL tuning, is the bottleneck* — so verify the reference **before** you learn. Implementation: [`projects/policies/training/`](../../projects/policies/training/README.md). Training is **in-engine and LOCAL** — there is no cloud path.
- [ghost-design-rules.md](ghost-design-rules.md): ⭐ **the ghost doctrine — the FOUR GATES + the CORRIDOR-vs-TORQUE LAW + the 7 design rules, and the ghost toolchain.** Gates 1–3 (closure / support / FWP) are static; **gate 4** (`ghost_funnel.py`) asks whether a deploy-grade PD plant can actually *track* the reference. The LAW: **a tracking corridor must exceed `τ_ff/kp`, or the reference is untrackable BY CONSTRUCTION** — it retro-explains every corridored run that failed. Run [`ghost_validator.py`](../../projects/policies/training/ghost_validator.py) on ANY new ghost **before** training.
- [skill-library.md](skill-library.md): **the SKILL LIBRARY — the standard way to make a skill and compose skills.** Wraps Shadowing + BATON into one pipeline (design → validate → preview → train → verify → register → sequence); a *skill* (walk / turn / carry / stand / climb, + H1 / Go2 / Spot) is one versioned manifest binding its ghost + validator verdict + deploy env + champion checkpoint + provenance. Front door: `projects/policies/skills/skill_lib.py` (`list`/`sequence`/`preview`/`train`/`verify-demos` + `handover`/`blendable`/`adapt`/`freeze`). **Start here to add a new skill or build a BATON demo.**
- [policy-switching.md](policy-switching.md): **BATON — runtime handover between separately-trained Shadowing specialists** (walk / stand / carry / turn). The handover machinery ships and is live-verified (four sequences: `box_delivery`, `box_delivery_classic`, `walk_turn_walk`, `turn_solo`; the 90° footwork turn is SOLVED, `72a7bb19`). ⚠️ The *switching-beats-a-monolith* **thesis remains an open hypothesis** — the success-vs-horizon experiment is unrun. Read its "Where BATON stands vs the field" section before making any novelty claim.
- [ghost-tracking-pipeline.md](ghost-tracking-pipeline.md): *(superseded as the how-to — see `shadowing.md` above.)* The original pre-rename architecture write-up of the planner→tracker split ("planning describes, control solves"). Kept for provenance; do not start here.
- [g1-universal-tracker.md](g1-universal-tracker.md): the north-star objective (follow the ghost for any motion) + the G1 walking test-bed ledger.
- [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md): how to cross the mujoco_warp→OmniSim-Newton gap (domain randomization) — the deploy stage of the pipeline.
- [g1-improved-shadow.md](g1-improved-shadow.md): earlier "achievable shadow" work (record/retarget feasible references).
- [g1-sitstand-journey.md](g1-sitstand-journey.md): the first sit→stand→sit test-bed — why a hand-drawn ghost left the seated start on a contact knife-edge, motivating the planner.

## Daily Workflow Guides

- [build-and-iteration.md](build-and-iteration.md): what each public build target really does, when a relink is required, and how to keep rebuilds narrow.
- [header-hygiene-and-rebuild-reduction.md](header-hygiene-and-rebuild-reduction.md): practical rules for shrinking header blast radius and avoiding unnecessary recompilation.
- [validation-playbook.md](validation-playbook.md): which validation lane to use for a given change, including smoke tests, test groups, and headless runs.
- [ci-and-fast-feedback.md](ci-and-fast-feedback.md): how to structure cheap PR-default checks, targeted subsystem lanes, and later-stage regression gates.
- [test-harness-and-scenario-architecture.md](test-harness-and-scenario-architecture.md): how the current smoke, benchmark, and legacy test harness layers fit together and how they should evolve.
- [profiling-playbook.md](profiling-playbook.md): how to collect and compare performance logs and benchmark runs.
- [benchmark-authoring.md](benchmark-authoring.md): how to add stable benchmark worlds and keep them useful over time.
- [simulation-authoring-for-coding-agents.md](simulation-authoring-for-coding-agents.md): current best workflow and recommended simulator changes for agent-friendly world and scenario authoring.
- [urdf-import-debugging.md](urdf-import-debugging.md): how to diagnose URDF import loss, joint-frame mismatches, and bad settling behavior.
- [step-to-urdf.md](step-to-urdf.md): convert a STEP/STP CAD assembly into a colour-correct OmniSim URDF robot (`scripts/dev/step_to_urdf.py`), with the orientation trap and numerical verification.

For the agent-facing HTTP harness itself, see [scripts/harness/README.md](../../scripts/harness/README.md) and [AGENTS.md §5](../../AGENTS.md#5-iterating-on-worlds-with-the-validation-harness).

## Deep Dives

- [performance-handbook.md](performance-handbook.md): the top-level map for build, load, runtime, rendering, sensor, and contributor-facing performance work.
- [module-dependency-map.md](module-dependency-map.md): where the current subsystem boundaries actually are, where they leak, and how to keep new code in the right layer.
- [controller-protocol.md](controller-protocol.md): packet format, lifecycle states, and transport-level controller behavior.
- [controller-ipc-and-step-loop.md](controller-ipc-and-step-loop.md): controller transport, scheduling, and step-loop coupling risks plus the cleanup path.
- [startup-reset-and-asset-lifecycle.md](startup-reset-and-asset-lifecycle.md): world-load, reset, and asset-download critical-path behavior and how to separate them.
- [observability-and-performance-telemetry.md](observability-and-performance-telemetry.md): what the current performance log captures, what it misses, and the next telemetry milestones.
- [world-loading-and-template-performance.md](world-loading-and-template-performance.md): parser, node reader, template regeneration, dictionary, and startup-path performance.
- [template-regeneration-and-dictionary-coherence.md](template-regeneration-and-dictionary-coherence.md): why PROTO regeneration, DEF/USE maintenance, and downstream UI costs are still tightly coupled.
- [engine-migration-plan.md](engine-migration-plan.md): **single unified master plan that ends with OmniSim architecturally complete** — Newton-canonical physics + wgpu-canonical renderer, with ODE+WREN as forever-fallbacks. Covers vision, sequencing (six phases α→ζ), performance targets (research-grounded), compatibility contract, the full physics arm phase status (P0–P9 + Phases A–E + CUDA/granular/particles/damage adjacent subsystems), the full rendering arm phase status (R0–R6 + Tier 1–5 fidelity tracks + §16.2 re-litigation triggers), the decision log, and the 2×2 test matrix. Replaces eleven prior plans (six physics-roadmap absorbed, five rendering-roadmap absorbed).
- [physics-and-determinism.md](physics-and-determinism.md): physics throughput, contact complexity, multithreading, and determinism tradeoffs.
- [physics-contact-and-collision-complexity.md](physics-contact-and-collision-complexity.md): deeper guidance on contact-joint pressure, collision filtering, truncation, and physics authoring hazards.
- [real-grasp-and-the-cold-first-load-trap.md](real-grasp-and-the-cold-first-load-trap.md): how we got a real (no-weld) friction bin grasp working, and the COLD-FIRST-LOAD trap it exposed — a fresh process under-tracks articulation targets, so precise grasps/insertions fail on the first load but work after a reload. **Read this before concluding a headless grasp/contact "can't work."**
- [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md): texture, cache, remote asset, and world-authoring guidance.
- [sensor-and-device-performance.md](sensor-and-device-performance.md): cameras, distance sensors, overlays, recognition, and GPU transfer costs.
- [runtime-hotspots.md](runtime-hotspots.md): deeper runtime, controller, parser, and synchronization review with concrete bottlenecks and bug risks.
- [scene-tree-selection-and-runtime-mutation.md](scene-tree-selection-and-runtime-mutation.md): why selection churn, layout invalidation, and runtime mutation still leak into desktop-shell cost.
- [rendering-and-visual-quality.md](rendering-and-visual-quality.md): current rendering stack, visual quality issues, and the highest-value rendering improvements (the authoring-guide companion to engine-migration-plan.md §14).
- [r3-rendering-backend-evaluation.md](r3-rendering-backend-evaluation.md): R3 implementation design for the wgpu-native migration — refreshed 2026-05-27, supersedes the bgfx-leaning April archive.
- [procedural-world-generation-plan.md](procedural-world-generation-plan.md): long-range three-tier plan for a world-class procedural world generation system with realism as the organizing goal — composable biome library + physics-settled placement + weathering + per-instance variation + micro-clutter → real-world import + ecology + human traces + weather/season + data-driven regional priors + scenarios + agent authoring → streaming + learned generation + photoscanned assets + style transfer.
- [omniworld-user-guide.md](omniworld-user-guide.md): user-facing guide to the `omniworld` procedural generation library — CLI, Python API, determinism contract, and the currently landed slice of the plan.
- [omniworld-biome-cookbook.md](omniworld-biome-cookbook.md): contributor guide to writing a new biome — walkthrough of a shipped recipe, zone priority rules, and a step-by-step template.
- [performance-anti-patterns.md](performance-anti-patterns.md): recurring design and implementation mistakes that slow builds, runtime, validation, or contributor iteration.
- [improvement-backlog.md](improvement-backlog.md): prioritized actionable backlog for performance, quality, and architecture work.
- [phase-two-architecture-plan.md](phase-two-architecture-plan.md): later implementation plan for a cleaner simulator architecture.
- [phase-two-execution-program.md](phase-two-execution-program.md): a more concrete sequencing of phase-two workstreams, milestones, and exit criteria.
- [rollback.md](rollback.md): baseline and recovery workflow.
- [copyright-headers.md](copyright-headers.md): per-file copyright header convention for unmodified Webots files, modified Webots files, and net-new OmniLink files.
- [codebase-stats.md](codebase-stats.md): the honest, reproducible answer to "how many lines did we write?" — ~120k lines of original source (mostly Python) + ~33k lines of reworked Webots C++ on top of the import, with the exact git commands to regenerate the numbers.

### Engine migration (Newton physics + wgpu render)

Companion docs to [engine-migration-plan.md](engine-migration-plan.md) (the master plan, in Deep Dives above). **For current code-verified status, read its §8.1 "Status refresh — 2026-06-08" — the canonical snapshot the others point to.**

- [migration-parallel-lanes.md](migration-parallel-lanes.md): **multi-session work split** — how to run the remaining migration across parallel Claude Code sessions on `main` without collisions (file-ownership map, coordination protocol, and copy-paste per-lane kickoff prompts).
- [architectural-baseline.md](architectural-baseline.md): the **architectural-baseline milestone** (COMPLETE 2026-06-07; documented, but **not** git-tagged — `architectural-baseline-v1` does not exist) — both backends dispatched, reversible, and gated; the structural foundation the rest builds on.
- [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md): the Newton **capability-completeness** plan (W0–W7) — the coverage-meter-driven climb from ~35–40% toward ~100% faithful world coverage.
- [default-flip-plan.md](default-flip-plan.md): the **safety harness + sequencing** for flipping the *default* physics/render backend without breaking the simulator (legacy kept as fallback + oracle).
- [dispatcher-surface-signoff.md](dispatcher-surface-signoff.md): sign-off that the `WbPhysicsBackend` / `WbRenderBackend` dispatcher surfaces are final — ordinary feature work overrides existing virtuals, never adds new ones.
- [physics-p8-statics-design.md](physics-p8-statics-design.md): design for moving static colliders onto Newton (P8) to retire the cross-backend ODE collision bridge.
- [physics-contact-impulse-api.md](physics-contact-impulse-api.md): the per-contact depth/impulse API for the damage tracker (the ODE wire format + the native Newton contact source).
- [r4-completion-checklist.md](r4-completion-checklist.md): the R4 task list to make **wgpu the default main-view renderer** (Phase ζ).
- [r4-step3c-plan.md](r4-step3c-plan.md): the coupling map + risk-ordered plan for replacing WREN as the main viewport (R4 step-3c).
- [rendering-arm-checklist.md](rendering-arm-checklist.md): the render-arm completion checklist (AgX / CSM / TAA fidelity + R4) toward Phase ζ.

### RL training & sim-to-deploy

- [rl-journey.md](rl-journey.md): **START HERE — the complete RL narrative** (current to 2026-06-14). The master story tying every thread together: the meta-principle, Spot → G1 stand → G1 walk (now stops-in-the-middle on command) → Spot walk → Atlas → manipulation, the infrastructure, the cross-cutting lessons, and a current status table. The per-topic docs below carry the deep ledgers.
- [rl-current-state.md](rl-current-state.md): **CANONICAL — the single source of truth for RL status; read its top banner FIRST.** If any other doc, script comment, or commit disagrees with this file, **this file is right** — including everything in this index. ⛔ Never quote an RL result (distance, survival, fall count, WBMATCH) from another doc; quote this one. The narrative `rl-journey.md` **defers** to it, and the method docs ([shadowing.md](shadowing.md), [ghost-design-rules.md](ghost-design-rules.md), [policy-switching.md](policy-switching.md), [skill-library.md](skill-library.md)) describe *how*, not *how well*.
- [rl-two-layer-architecture.md](rl-two-layer-architecture.md): "The Ghost Method" — the interface contract, per-robot plan, and phased gates. *This is the earlier name for what is now called **Shadowing**; for the method itself start at [shadowing.md](shadowing.md) + [ghost-design-rules.md](ghost-design-rules.md), which are the maintained flagship docs.*
- [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md): **the default RL recipe for any new robot deploying to OmniSim Newton.** Heavy-DR pure PPO on GPU mujoco_warp + the dump-the-deploy-MJCF trick. *Absorbs much of* the sim-to-deploy gap by training to be invariant to the wrapper drift — but for stability-margin bipeds a residual deploy gap can remain (see G1). Includes the 5-speedup template (~132 k env-steps/s on the original training box).
- [train-deploy-gap.md](train-deploy-gap.md): **synthesis + recipe — read after rl-current-state.md.** Frames the train→deploy problem as *two* gaps (pipeline-parity vs durability), gives the enumerated + re-verified divergence table (COM, qd, launch-IC, contact), and the **Unitree gold-standard obs/action/reward recipe** as the durability answer. Routes to the owners below.
- [train-deploy-unification.md](train-deploy-unification.md): the loop-unification mechanics — one `mujoco_warp` engine at N=1 (deploy) vs N (train); Layer A/B/C (physics-spec / `g1_env_core` obs+IC / shared step core), Phase 0/1/2, and the qd + launch-IC divergences with the parity tests.
- [g1-single-source-of-truth.md](g1-single-source-of-truth.md): the **model** single-source contract — `g1_physics.json` + `g1_physics_spec.py` + prim URDF → CI conformance; the three-way golden parity proof; the "import the spec, never re-declare a constant" rule.
- [locomotion-shadowing-vs-pure-rl.md](locomotion-shadowing-vs-pure-rl.md): **(superseded)** the 2026-06-25 architecture-choice course-correction — why a kinematic ghost (Shadowing) supposedly could not stabilise a continuous-balance walk, and the full-authority-RL + reward-shaping alternative it proposed. Since **superseded**: Shadowing became the flagship method it argued against (canonical status: [rl-current-state.md](rl-current-state.md)).
- [g1-stand-rl-playbook.md](g1-stand-rl-playbook.md): full case study — the 8-iteration journey from "G1 can't stand 1 second" to a robust stand. (The OmniSim Newton deploy now stands **indefinitely** — solved 2026-06-10 with the deeper-squat NOMINAL; see `rl-journey.md` §2 / the deploy-regression note. Earlier editions of this doc that say "holds to t ≈ 1.55 s" predate that fix.)
- [g1-walk-rl-journey.md](g1-walk-rl-journey.md): the full G1 **walking** arc (§1–§15) — the 20× stiffness-mismatch breakthrough, the foot-space human gait + IK, the four stacked deploy-gap causes, gait style (winter / swing / shape reward), the ghost, the drunk-gait diagnosis, and the velocity-conditioned **stop-in-the-middle** milestone. The deepest deploy-gap ledger in the repo.
- [atlas-stand-rl-journey.md](atlas-stand-rl-journey.md): porting the G1 recipe to a 30-DOF, 175 kg humanoid. Six things changed (mass-DR ceiling, residual scale, NaN sanitization, reward/value clipping, ONNX dynamo flag, the discovery that heavy DR breaks Atlas learning entirely).
- [spot-residual-rl.md](spot-residual-rl.md): the quadruped recipe (residual on a model walker). **Use for quadrupeds only** — doesn't port to bipeds without modification (see G1 playbook).
- [humanoid-balance-gap.md](humanoid-balance-gap.md): historical "why bipeds are hard" analysis (its original LIPM conclusion was wrong; kept for context). G1 **stand** is solved in deploy (deterministic pure pose, holds indefinitely). ⛔ This doc's G1 **walk** verdict is stale — the "finite ~34 s bout" figure is the *research-era* deploy path, superseded by the in-engine Shadowing walk. **Take the walk status from `rl-current-state.md` (canonical), never from here.**
- [rl-accelerated-training.md](rl-accelerated-training.md): training backends overview (sb3 / mjx / isaac / mujoco_warp) and when to pick which.

## Archived

Plans whose work is substantially done, or that have been superseded, live under [archive/](archive/) — see [archive/README.md](archive/README.md) for the convention. Anything in there may be out of date; the live code is authoritative.

## Machine-Readable Guidance

- [search-map.json](search-map.json): repository-local search guidance for agents and tools.

## Suggested Reading Order

If you are:

- making a legged robot perform a motion (walk, turn, carry, climb): read `shadowing.md`, then `ghost-design-rules.md`, then `skill-library.md` — and check `rl-current-state.md` for what is actually working today
- composing skills into a demo (a BATON sequence): read `skill-library.md`, then `policy-switching.md`
- building locally for the first time: read `quickstart.md`, then `build-and-iteration.md`
- changing simulator code in `src/omnisim`: read `architecture.md`, then `validation-playbook.md`
- trying to keep rebuilds narrow: read `header-hygiene-and-rebuild-reduction.md`, then `module-dependency-map.md`
- changing PROTO regeneration or DEF/USE behavior: read `template-regeneration-and-dictionary-coherence.md`
- changing controller, IPC, or step scheduling code: read `controller-ipc-and-step-loop.md`
- improving startup or reset behavior: read `startup-reset-and-asset-lifecycle.md`
- changing renderer code in `src/wren` or `src/omnisim/wren`: read `rendering-and-visual-quality.md`, then `profiling-playbook.md`
- looking for what's left to implement across all rendering subsystems: read `engine-migration-plan.md` §14
- debugging runtime behavior, controller synchronization, or parser issues: read `runtime-hotspots.md`, then `world-loading-and-template-performance.md`
- debugging imported URDF robot structure or settling issues: read `urdf-import-debugging.md`
- improving runtime or physics throughput: read `performance-handbook.md`, then `physics-and-determinism.md`
- looking for what's left to implement across all physics subsystems: read `engine-migration-plan.md` §13
- improving contact-heavy or unstable physics worlds: read `physics-contact-and-collision-complexity.md`
- improving texture memory, uploads, or overlays: read `asset-pipeline-and-world-quality.md` (texture/cache/upload rules) — the engine-migration-plan covers strategy, the asset-pipeline doc covers the day-to-day rules
- improving scene-tree, selection, or editor responsiveness: read `scene-tree-selection-and-runtime-mutation.md`
- improving PR and CI turnaround: read `ci-and-fast-feedback.md`, then `validation-playbook.md`
- improving tests, smoke lanes, or benchmark structure: read `test-harness-and-scenario-architecture.md`
- building or improving simulation worlds, PROTOs, or agent-driven scenarios: read `simulation-authoring-for-coding-agents.md`, then `asset-pipeline-and-world-quality.md`
- using or extending the agent-facing HTTP harness: read `../../scripts/harness/README.md` and `../../AGENTS.md` (section 5)
- improving measurement fidelity: read `observability-and-performance-telemetry.md`
- improving asset, startup, or sample-world quality: read `asset-pipeline-and-world-quality.md`
- improving sensors or device-heavy worlds: read `sensor-and-device-performance.md`
- reviewing a risky optimization or refactor: read `performance-anti-patterns.md`
- working through the broader modernization effort: read `phase-two-architecture-plan.md` and `phase-two-execution-program.md`

## Scope

These docs are intentionally pragmatic:

- they describe the codebase as it exists now
- they call out the fast paths that are actually available today
- they note where a path is a wrapper over legacy makefiles rather than a fully modular system

When the code and docs disagree, update the docs in the same change.
