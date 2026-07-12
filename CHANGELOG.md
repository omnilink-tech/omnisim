# Changelog

All notable changes to OmniSim are recorded here.

The format roughly follows
[Keep a Changelog](https://keepachangelog.com), and OmniSim follows
[Semantic Versioning](https://semver.org).

OmniSim is built on [Webots](https://github.com/cyberbotics/webots) — see
the [Attribution](README.md#attribution) section of the README for the
relationship to upstream. Entries here cover OmniLink's contributions on
top of that foundation.

---

## [Unreleased]

_Nothing yet — changes land here before the next tagged release._

---

## [v5.0.0] — 2026-07-11

**The robot-learning release.** OmniSim gains a complete, robot-agnostic pipeline for
*making a legged robot do a motion*: design a dynamically-feasible reference (a **ghost**),
prove it feasible **before** training, learn to track it (**Shadowing**), package the result
as a versioned **skill**, and compose skills into task sequences (**BATON**). Training moved
**in-engine** — policies now train *through* `omnisim-bin`, so train == deploy bit-exact — and
the cloud training path was **removed**: everything runs on the GPU you already own.

This release supersedes **v4.5.0**, which was version-bumped but never published; all of its
content ships here.

> **A note on honesty, up front.** The humanoid results below are real but **assisted**: the
> G1 walk, box-delivery, and turn demos run on a *visible balance harness* (a "puppet" rig)
> that carries part of the robot's weight and stabilises its attitude. **A durable,
> free-standing humanoid walk remains OPEN.** Every capability claim in this file is
> caveated inline, and the single canonical per-robot answer to "is it actually done in
> deploy?" always lives in
> [docs/developer/rl-current-state.md](docs/developer/rl-current-state.md) — if any headline
> here disagrees with that file, **that file is right**. See *Known limitations* at the end.

~370 commits since v4.0.0.

### ⚠️ Breaking changes

- **The `cloud/` Modal-H100 training path is REMOVED.** Training is **in-engine and local by
  policy**. The 16 Modal wrappers were thin subprocess shims over trainers that already ran
  locally. Use [`projects/policies/training/run_walk_rl.sh`](projects/policies/training/run_walk_rl.sh)
  (in-engine, train == deploy) or the standalone-but-still-local trainers in
  `projects/policies/research/training/`. The `OMNISIM_MODAL_GPU` env var is dead.
- **`projects/rl/` → `projects/policies/`** (685 files), with a `control` / `controllers` /
  `worlds` split from `research/`. Update any external paths and imports.
- **Robot removed:** an experimental humanoid model and its demo.
- **Robot asset packages withdrawn pending confirmation of redistribution terms.** We audited the
  provenance of every robot model we redistribute. A few carried no stated licence for their
  source geometry. Rather than keep redistributing CAD whose terms we cannot evidence, those
  packages and the demos that exist only to drive them are withdrawn from distribution until the
  terms are confirmed in writing. The simulator, the arm bridge, the grasping stack and the URDF
  importer are unaffected — point them at any URDF you have the rights to. We would rather ship
  fewer demos than redistribute a manufacturer's CAD we cannot account for.
- **Upstream licences now ship with the robots that need them.** The same audit found geometry we
  redistribute under BSD-3 and NASA-1.3 whose licence text we were not reproducing — a real
  compliance gap, not a formality. Each robot package that carries a third-party licence now ships
  that licence verbatim beside its meshes, and `NOTICE` / `THIRD_PARTY_NOTICES.md` name the actual
  licence and copyright holder for every one.
- **Arms: the openly-licensed lineup is back.** v4.5.0 had narrowed the supported arms down to a
  single vendor's. With that vendor's package now withheld (above), the **Universal Robots
  UR3e / UR5e / UR10e** (BSD-3) are restored, along with their chat demos, the
  `omnilink_multi_arm` world (3× UR5e), and the UR5e loader in `warehouse_logistics`. All three
  are verified driving over the arm bridge (joint tracking to ~0.02–0.03 rad; DLS IK reaches a
  Cartesian target to ~4.6 cm). Every arm OmniSim ships is now one whose redistribution terms we
  can point at. *(The Franka Panda is restored in-tree but **not** shipped in v5.0.0: its URDF
  declares no inertials, so the importer synthesises gram-scale links and the arm will not hold a
  pose. It ships when it works.)* Robotiq / OnRobot / Schunk / vacuum / magnetic
  grippers are unaffected. *(Carried over from the unreleased v4.5.0.)*
- **`book_delivery` → `box_delivery`** across skill manifests, sequences, and demo scripts.
- **Demos retired:** the arm `bin_grab` demo (a one-wall friction grasp is fundamentally marginal)
  and the standalone construction-site demo + benchmark (the *environment* world is kept).
- **The GUI now defaults to the dark Night theme** on all platforms.
- **`run-world` gained a first-run conformance gate** — it can block an interactive launch on
  FAIL. It fails *open*, and `OMNISIM_SKIP_CONFORMANCE=1` bypasses it.
- **The controllers' `warmup_reload` helper is now a no-op** — the cold-first-load articulation
  bug it worked around is fixed, so the startup reload is gone.

### Highlights

- **The three train→deploy gaps are CLOSED — train == deploy is now bit-exact.** The
  discretization mismatch (the live engine steps 4×0.004 s; the trainer was doing 4×0.002 s),
  the settle-and-go handoff, and a post-step joint-limit clamp were the last three. Result: a
  durable straight real-foot G1 walk in the engine. v4.5.0 shipped this as ⚠️ OPEN.
- **In-engine training is the flagship venue.** Policies train *through* `omnisim-bin`
  (Newton / `mujoco_warp`), so there is no second physics stack to keep in sync. GPU-resident
  in-engine PPO runs **~44× faster** than the previous path (~140–200 k env-steps/s sustained on a
  laptop GPU, via zero-copy `wp.to_torch` + CUDA-graph capture).
- **Shadowing** — the robot-agnostic motion method: a *generator* produces a
  dynamically-feasible reference ("the ghost"), a *verifier* numerically certifies it
  **before** any RL, and a *tracker* learns to follow it. The thesis: **reference feasibility
  is the bottleneck** — so verify, then learn.
  ([docs/developer/shadowing.md](docs/developer/shadowing.md))
- **Ghost design formalised as gates.** Feasibility is now four checkable gates — kinematic
  closure, COM support, force-wrench membership, and **PD-realizability** — and `ghost_synth`
  builds ghosts where they hold **by construction**. The accompanying **corridor-vs-torque
  law** (a tracking corridor must exceed τ_ff/kp or the reference is untrackable *by
  construction*) retro-explains a long run of previously mysterious training failures.
  ([docs/developer/ghost-design-rules.md](docs/developer/ghost-design-rules.md))
- **The Skill Library** — the standard packaging of Shadowing + BATON. One versioned manifest
  per skill binds its ghost, validator verdict, deploy env, champion checkpoint, and
  provenance; `skill_lib.py` covers `list` / `preview` / `train` / `run` / `sequence` /
  `verify-demos`. 10 skills across G1 / H1 / Go2 / Spot, 4 BATON sequences.
  ([docs/developer/skill-library.md](docs/developer/skill-library.md))
- **BATON policy switching** — composes specialist policies (walk / turn / carry / stand) into
  task sequences with engineered handovers.
- **First-party MCP server** ([`packages/omnisim-mcp/`](packages/omnisim-mcp/)) — a
  dependency-free stdio JSON-RPC proxy to the `:6789` harness (14 tools), so Claude Desktop
  and Cursor drive OmniSim natively.
- **Newton is now honest about being Newton.** A silent Newton→ODE downgrade is **fatal**
  instead of quiet, and the engine writes a race-free `<log>.newton.json` verdict sidecar at
  world finalisation — its presence is proof Newton drove *that* run.

### Robot learning — Shadowing, skills, BATON

- **Skill Library** (`projects/policies/skills/`): manifests + `skill_lib.py`, cross-cadence
  adapters, per-edge handovers, freeze, and cross-robot reuse. `verify-demos` asserts each
  manifest reproduces its hand-written demo script **key-for-key on the assembled launch env**.
- **BATON** sequences: `box_delivery`, `box_delivery_classic`, `walk_turn_walk`, `turn_solo`.
  ⚠️ BATON's *thesis* — that switching specialists degrades more gracefully over a long
  horizon than one distilled monolith — remains a **well-posed open hypothesis**; the
  success-vs-horizon experiment is unrun. ([docs/developer/policy-switching.md](docs/developer/policy-switching.md))
- **Ghost toolchain**: `ghost_synth`, `ghost_validator`, `ghost_doctor` (a prescriptive
  classifier), `ghost_polish`, `ghost_close`, `ghost_funnel` (the gate-4 PD-realizability
  funnel), `ghost_ff`, `ghost_topp`. The **WBMATCH** similarity metric reached v4 — an honest
  *shape-only* ruler that no longer flatters a policy for tracking a corridor it was handed.
- **`ghost_synth` motion library** — one method (plan the contacts, solve the base + joints),
  one judge, generalising across **walk, squat, kneel, push-up, and 3 cm + 7 cm stairs**. It
  also *measured* the quasi-static walking speed limit (vx ≈ 0.131 m/s), which retroactively
  explained why an earlier "0.45 m/s" reference was only achievable on a crane.
- **In-engine quad RL** — Go2 now trains *through* the engine with no MJCF reparse, so train ==
  deploy bit-exact. On a **trainer-side batched eval** of the deploy-identical model (4096 envs):
  **94.8 % never fell over 48 s, 16.6 m mean, 0.357 m·s⁻¹**; Spot smoke 69.5 %.
  ⚠️ **That is a batched evaluation inside the trainer, not a live single-robot deploy run** — the
  in-engine champions have not yet been given an equivalent live long-run. The live Newton deploy
  walks on record are still the *standalone*-trained policies (Go2 **+86.7 m**, Spot +47.8 m,
  B2 +110.7 m, 0 falls). ⚠️ B2 stiffness is **unreconciled**.
- **Terrain-curriculum quad RL** — blind (observations unchanged), Spot clears 18 cm bumps and
  rubble with 0 falls.
- **Binary-level train↔deploy parity probe**, generalised robot-agnostically — all 6 legged
  robots PASS.
- **Unitree's own official G1 and H1 policies now run inside OmniSim**, which is where the
  recorded reference ghost comes from.

### Demos & worlds

- ⭐ **The Decent Walker** — the flagship humanoid demo: the G1 walks the official Unitree gait
  beside its ghost hologram, with natural thigh-clearing arm swing (LSTM + foresight champion,
  WBMATCH4 **0.868** on the shape-only ruler). ⚠️ Runs on the **visible balance-harness puppet
  rig** — an overhead support that carries part of the robot's weight and stabilises attitude.
  **This is not yet a free-standing walk.** Known-open: live stride runs below the trainer.
- ⭐ **Box delivery** (BATON) — the G1 walks to a cart, lifts a 1.5 kg box, carries it, sets it
  down on a second cart, takes a real ~90° footwork corner, and walks away. **0 falls.**
  ⚠️ Harnessed. ⚠️ **The carry is kinematic, not a grasp.** The box is a real rigid body under
  gravity, but it is an ODE body while the robot runs on Newton — so hand↔box contact is
  structurally impossible. During the carry the box is posed to the hand centroid each tick and
  holds ~1 cm hand clearance by design: *the hands never touch it*. The payload is real to the
  **policy** (it is the trained carry plant, `CARRY_PAYLOAD_KG`), and the locomotion, the corner
  and the fall-free record are real. The grasp is not — see
  [policy-switching.md](docs/developer/policy-switching.md).
- ⭐ **Walk-turn-walk** — a genuine 90° footwork turn in sequence: **90.6–95.6° actual, 3/3,
  0 falls**. ⚠️ Harnessed.
- **G1 stair climb** — a full **5-step live climb**, and the one demo where **the legs do all the
  *vertical* work** (`HARNESS_KZ=0`: no vertical crane assist). A companion demo climbs all five
  treads and then holds a near-motionless stand on the top landing via a position-gated BATON
  handover.
  ⚠️ **The crane is off vertically, but not in attitude — and the champion leans on it.** Our own
  motion-legitimacy verifier (which ships:
  [`verify_motion_legitimacy.py`](projects/policies/training/verify_motion_legitimacy.py)) **FAILS**
  this champion: the attitude springs sustain **|ty| ≈ 77.5 N·m on 77 % of climb ticks** (the crane
  carries the lean), and its **knees contact the treads on 13.6 % of climb ticks**. The stand demo
  passes roughly **1 run in 2** (retry on a fall). It clears the *kinematic* gates; it does not
  clear the *dynamic* ones. Treat this as a promising result under scrutiny, **not a finished
  demo**. Full write-up: [motion-legitimacy.md](docs/developer/motion-legitimacy.md).
  ⚠️ **3 cm risers is the measured ceiling** for the stock-foot G1 — 4 cm degrades to ~2 steps and
  5 cm to none. Taller risers need a foot-morphology change or a vertical assist.
- **Ghost-follow holograms** — the ghost now renders the deploy's *active* reference
  end-to-end through walk → turn → walk, plus dedicated "show" worlds.
- **G1**: manipulation while standing, cube-defense stand, one-leg balance, deterministic squat
  overlay, arm-motion skill.
- ⚠️ **G1 army-crawl** — feasibility is a GO and the reference is designed, but it is **not a
  working motion**: the 25 N·m arms are the wall.

### Agent & bridge surface

- **The arm bridge's real-hardware path is now a pluggable backend, not a hardcoded vendor.**
  Drop a `<name>_backend.py` next to the controller and select it with
  `--hardware-backend <name>` / `--hardware-ip <addr>` (or `OMNILINK_HARDWARE_BACKEND` /
  `OMNILINK_HARDWARE_IP`); the bridge discovers it by module name against a small
  `HardwareBackend` protocol (start/shutdown, status, joint + linear moves, home, grasp/release,
  stop). With no backend installed the bridge is pure simulation and the option is not offered;
  asking for a backend that is not installed fails loudly rather than silently running sim-only.
  **Breaking:** `--neura-ip` → `--hardware-backend`/`--hardware-ip`, `GET /neura_status` →
  `GET /hardware_status`, and the `neura` key in `/get_robot_state` and the robot-window payload
  → `hardware` (now carrying the backend name). No backend ships in the public snapshot, so no
  published consumer breaks.
- **The `/sim/events` stream no longer drowns itself.** A Newton registration census intended to
  fire once per world build was re-firing every tick, so **~99 % of the `controller.log` traffic on
  `GET /sim/events` was one repeated line** and real controller output was being dropped
  (`dropped_log` in the hundreds of thousands on a long run). It now fires once per build, still
  re-fires on reload, and stays silent for worlds with no Newton bodies. This was breaking the
  exact HTTP-harness debugging loop [AGENTS.md](AGENTS.md) and [PROTOCOL.md](PROTOCOL.md) tell
  agents to use.
- **The in-app demo launcher no longer offers a button that cannot work.** The policy demos need a
  deploy environment that only their shell script exports; their cards previously had a live
  *Launch* button that loaded a bare world and left the robot lifeless. They now show the exact
  command with a copy button instead.

### Engine & physics

- **Cross-tree contact — characterized, not solved.** `mujoco_warp`'s *island* path (active only
  when a free prop exists — i.e. exactly when you try to grasp something) produces NaNs on the
  first robot↔free-body contact. We now understand it and can reproduce it on demand: a box
  *does* rise under real palm contact, but some runs still go non-finite, so **friction grasping
  is parked**. The working pick-and-place path is a contact-free suction coupling instead.
  This is a diagnosis, not a fix.
- **No silent Newton→ODE fallback** — capability-gate downgrades, orphaned Newton joints, and
  a MuJoCo→XPBD solver downgrade are now **fatal**. Escape hatches: `OMNISIM_ALLOW_ODE_FALLBACK`,
  `OMNISIM_FORCE_ODE`.
- **Newton verdict sidecar** — the engine writes `<log>.newton.json` at world finalisation, so
  "did Newton actually drive this run?" is answerable without scraping logs (the old
  log-scrape was fooled by large logs and could falsely report ODE).
- **Newton compound-body inertia** corrected; degenerate welded-static and dynamic-bin inertias
  de-degenerated (they were causing a contact drop / sink).
- **Launch-flake fixed** — the Windows IPC pipe name was salted only by TCP port, so
  back-to-back launches could cross-connect. Now folds in the PID: **105/105 launches connect
  first-try** (was ~90 %).
- `staticBase` robots were being welded at the origin instead of their spawn pose — fixed.
- **G1 stand deploy observation bug** — the engine's angular velocity is a different frame and
  scale than MuJoCo's `qvel`; using it raw caused a ~1.8 s deploy fall.

### Performance

- **GPU-resident in-engine PPO: ~44×** (~140–200 k env-steps/s sustained on a laptop RTX 5070 Ti;
  ~218 k peak at K=2048).
- **Realtime**: even `newtonSubsteps` re-enables CUDA-graph capture (0.25× → **1.16× realtime**);
  forcing the wgpu/Vulkan main view takes the live GUI from ~0.4× to **~1.2× realtime**.
- **Persistent-sim job server** — keeps one simulator alive across queued jobs: evaluations go
  from 4–6 minutes to **9 seconds** (~30×).
- Harness `render_stats` vectorized (**17×**).

### Packaging, tooling & conformance

- **`omnisim verify-install`** — a per-backend acceptance manifest with hard canaries and soft
  physics bands, plus a fingerprint and an advisor. Wired into `run-world` as a first-run gate.
- **Newton runtime version-parity guard** + CI test — the bundler was `pip install --upgrade`-ing
  while the trainer pinned exact versions, so "bit-exact train == deploy" could silently rot.
- **Reproducible Newton scaling benchmark**, replacing a deleted headline probe.
- **Warp contact-kernel validation harness** — steps physics-only worlds on both `mujoco_warp`
  and CPU `mj_step`. Tiers 1–2 are **clean (10/10 cells)**, which *rules out* the prop contact
  kernels as the grasp defect.
- **~1 GB of tracked training checkpoints purged** — only the referenced champions stay tracked,
  enforced by a pre-push guard.
- Agent runners consolidated onto a shared `OmniLinkAgentRunner` / `_lib` (−4.4 k LOC).

### Documentation

New: [`simulator-comparison.md`](docs/developer/simulator-comparison.md) (10 simulators, every
claim marked verified / unaudited / vendor-claim, adversarially checked),
[`ros2-integration.md`](docs/developer/ros2-integration.md) (**ROS 2 is an explicit non-goal** —
with a working external-bridge recipe, and a pointer to Gazebo for ROS-centric work),
[`skill-library.md`](docs/developer/skill-library.md),
[`ghost-design-rules.md`](docs/developer/ghost-design-rules.md),
[`policy-switching.md`](docs/developer/policy-switching.md),
[`train-deploy-gap.md`](docs/developer/train-deploy-gap.md),
[`closed-loop-chaos-diagnostic.md`](docs/developer/closed-loop-chaos-diagnostic.md) (classifies a
train-vs-deploy divergence as BUG / CHAOS / MATCH before you theorise),
[`install-conformance.md`](docs/developer/install-conformance.md), and the
`projects/policies/training/` + `projects/policies/skills/` READMEs.

The **Shadowing paper** ([`docs/developer/shadowing_paper/`](docs/developer/shadowing_paper/)) —
the method writeup, published here for the first time. Every reported result is a real deploy
rollout traceable to
[rl-current-state.md](docs/developer/rl-current-state.md), and the G1 results state up front that
they run on a partial-support balance harness.

### Legal & licensing

We audited the provenance of every robot model OmniSim redistributes, and fixed what the audit
found. This was a real compliance gap, not paperwork.

- **Upstream licence texts now ship with the geometry they cover**, as BSD-3 clause 1 and NOSA §3
  require: **Unitree** G1 / H1 / Go2 / B2 (BSD-3), **Spot** (BSD-3, Clearpath), **Atlas** (BSD-3,
  Robot Locomotion Group @ CSAIL), **Valkyrie** (NASA-1.3), **Universal Robots** UR3e / UR5e /
  UR10e (BSD-3), **Franka Panda** (Apache-2.0, plus the upstream `NOTICE`). Previously we shipped
  187 Unitree geometry files with no licence text at all.
- **`NOTICE` and `THIRD_PARTY_NOTICES.md` rewritten** to name, per robot, the upstream URL, the
  licence, the copyright holder, and the path to the licence text in this tree. **Spot and Atlas
  leave the "unverified terms" carve-out** — both are cleanly BSD-3 (Spot's meshes were authored
  by Clearpath, not supplied by Boston Dynamics). The carve-out is now **Valkyrie alone**, plus
  the Code2000 fonts.
- **Valkyrie is NASA-1.3 (NOSA)** — OSI-approved but GPL-incompatible and **not relicensable**,
  and it obliges every redistributor to carry the agreement. Labelled accordingly.
- **Universal Robots licenses its UR20-class meshes under separate, restrictive terms.** OmniSim
  ships only the BSD-3 ur3e / ur5e / ur10e families. Do not add a UR20 without re-reading them.
- **Spot renders textured again** — 13 of its meshes referenced a `spot_mat.png` we were not
  shipping. It comes from the same BSD-3 package; it now ships.
- **Trademarks** are used nominatively only. "TurtleBot" is a registered OSRF mark.

### Known limitations (read before quoting any result)

- **No durable free-standing humanoid walk *from a policy we trained*.** Our G1 walk / delivery /
  turn demos run on a weight- and attitude-bearing balance harness; removing it is the open
  problem. For calibration: **Unitree's own official G1/H1 policies, re-hosted unchanged in
  OmniSim, walk free-standing with no harness** (G1: 33.7 m, 0 falls, 0.48 m/s). The engine
  carries an unassisted humanoid walk — our *training* is what doesn't, yet.
- **Stair climbing caps at 3 cm risers** on the stock-foot G1. The 7 cm ghost passes every
  feasibility gate — but **gate-pass is not a climb**: no policy climbs 7 cm live; it plateaus
  at ~2 of 5 steps against a propulsion wall.
- **The stair champion does not pass our own dynamic audit.** The climb is legs-only *vertically*
  (`HARNESS_KZ=0`), but the harness still has attitude authority, and
  [`verify_motion_legitimacy.py`](projects/policies/training/verify_motion_legitimacy.py) — which
  we ship — **FAILS** the champion: the attitude springs sustain ~77.5 N·m on 77 % of climb ticks,
  and its knees contact the treads on 13.6 % of them. It clears the kinematic gates and not the
  dynamic ones. We built the verifier, it caught our own demo, and we are shipping both.
- **We do not have a contact-physics grasp.** Friction grasping goes non-finite on palm contact
  (`mujoco_warp`'s island path) and is **parked**. What works is a *contact-free* suction coupling,
  and the G1 box-delivery carry is **kinematic** — the box is posed to the hand each tick with
  ~1 cm clearance; the hands never touch it. Verified pick-and-place exists on the arm side; a
  humanoid that closes its fingers on an object and holds it by friction does not.
- **The arm demos are pick-and-place, not dexterous manipulation.** The arms that ship (UR3e /
  UR5e / UR10e, Franka Panda) do position control, IK and suction/parallel-jaw picking. There is
  no in-hand manipulation, no force control, and — per the previous point — no friction grasp.
- **BATON's switching advantage is unproven** — an open hypothesis, not a result.
- **B2 quadruped stiffness is unreconciled.**
- **Newton physics, the RL pipeline and every legged demo are Windows + NVIDIA only** today.
  Linux/macOS run the simulator on the ODE fallback. See the platform-support table in the README.
- **Sim-to-real is unproven.** OmniSim demonstrates train == deploy parity *in-engine*. That is
  not the same as zero-shot transfer to physical hardware, and **no policy trained in OmniSim has
  been validated on physical hardware.**

---

## [v4.5.0] — 2026-06-24 *(version-bumped but NEVER PUBLISHED — its content ships in v5.0.0 above; retained for provenance)*

The **Shadowing** release. This cycle is mostly a new, robot-agnostic motion-control
*method* and the demos built on it, plus more Unitree robots. It also **narrows the
supported arm lineup to a single 6-DOF cobot arm** (removing the Universal Robots and
Franka Emika Panda arms — see Removed) and adds a reusable **STEP/CAD → URDF converter**. A note on honesty up front: much of the motion/RL work
below is **sim-validated with an open deploy gap**, flagged inline. The single
canonical, per-robot "is it actually done in deploy?" answer always lives in
[docs/developer/rl-current-state.md](docs/developer/rl-current-state.md); if any
headline here disagrees with that file, that file is right. ~340 commits since v4.0.0.

### Highlights
- **Shadowing** (the headline; formerly "ghost-tracking") — a robot-agnostic motion
  pipeline. A *generator* (trajectory optimization) produces a dynamically-feasible
  reference (the "ghost"); a *verifier* numerically certifies feasibility **before**
  any RL; an RL *tracker* then learns to follow ("shadow") it through to deploy.
  "Planning describes, control solves." Method writeup + research-paper scaffold:
  [docs/developer/shadowing.md](docs/developer/shadowing.md).
- **Arm toss-to-place via Shadowing** — the arm *throws* a cube into a bin **beyond
  its reach** (impossible to carry or solve with IK), landing ~1.5 cm from centre;
  full generate → verify → deploy in OmniSim Newton. The verifier *rejecting* too-far
  bins (the feasibility frontier) is the point of the demo.
- **More Unitree robots** — Unitree **Go2** walks in Newton deploy via the Spot
  residual stack (plus a retarget recipe for other Unitree quadrupeds); Unitree
  **B2** get-up (rise) via Shadowing.

### Demos & worlds
- **Arm toss-to-place** and **arm throw-and-catch** — flagship Shadowing
  manipulation demos.
- **Spot velocity-conditioned walk / stop / walk** — one policy takes a commanded
  speed (including 0 = stop); walks, stops, then resumes. Deploy verdict in
  rl-current-state.md.
- **G1 stand-and-wave**, **G1 sit→stand→sit**, **seated G1 arm-mimic** — Shadowing
  test-beds on the 23-DOF humanoid; sim-validated. On-screen deploy of a *stationary*
  full-body stand remains an open project gap (walking deploys; a static full-body
  stand does not yet).
- **Hill-walk** ghost pipeline for Spot + B2 (gait pitched to the incline, with a
  live preview); the RL tracker is still blocked at the flat→ramp transition.

### Robots & PROTOs
- Unitree **Go2** (walk); Unitree **B2** (walk + get-up scaffolding). Retarget recipe
  for additional Unitree quadrupeds (Go1 / A1 / Aliengo / B1).
- **`scripts/dev/step_to_urdf.py`** — a reusable STEP/STP → URDF converter (tessellate →
  colour-split → URDF; doc: [docs/developer/step-to-urdf.md](docs/developer/step-to-urdf.md)).
  Turns any CAD assembly into a simulatable robot.

### Reinforcement learning
- **Shadowing realized end-to-end** (generator + verifier + tracker) on the arm
  toss, and documented as the canonical recipe.
- **Spot** velocity-conditioned walk/stop/walk; **Go2** walk; **B2** rise — see
  rl-current-state.md for the honest deploy verdict on each.
- ⚠️ A **durable G1 humanoid deploy walk remains OPEN**; the G1 Shadowing demos
  (sit-stand, stand-and-wave) are sim-validated, with the on-screen full-body-stand
  deploy gap noted above.
- **H1 + Valkyrie walking shadows** — feasible walking references (designed +
  ghost-verified) for the Unitree **H1** (5-DOF leg) and NASA **Valkyrie** (130 kg,
  6-DOF leg), plus an **H1 Phase-2 deploy-physics fine-tune trainer** (batched MuJoCo
  solver). ⚠️ sim-validated; the RL tracker that closes the sim-to-deploy gap is
  pending — see rl-current-state.md.

### Removed
- **Universal Robots (UR3e / UR5e / UR10e) and the Franka Emika Panda arms** — OmniSim's
  manipulator lineup is narrowed to a **single 6-DOF cobot arm**. This
  removes the `projects/robots/universal_robots/` and `projects/robots/franka_emika/`
  asset packages; the `omnilink_ur3e` / `omnilink_ur5e` / `omnilink_ur10e` /
  `omnilink_panda` chat demos; the `omnilink_multi_arm` (3× UR5e) world and its
  `OmniSim-Foreman` agent template; the UR5e-specific controllers (`ur5e_omnilink_bridge`,
  `ur5e_ik_slave`, `ur5e_teleop`); and the `panda_hand` gripper preset. The
  `omnilink_arm_bridge` arm registry now contains a single arm. The `warehouse_logistics`
  flagship loses its dock-side UR5e Loader (the Warehouse Foreman now runs the mission
  Picker-only), and the `Axis` control agent is re-pointed to that arm. The Robotiq /
  OnRobot / Schunk / vacuum / magnetic grippers are unaffected.
- **Construction-site demo + benchmark** — the standalone construction-site demo and its
  benchmark were removed; the construction-site *environment* world is kept for reuse.

### Fixed
- **Free-body velocity writes under Newton** — `node.setVelocity()` on a *free*
  (non-articulated) body was a no-op under the Newton/MuJoCo backend (it wrote the
  body twist instead of the free joint's `joint_qd`); fixed so Supervisor velocity
  sets on free bodies take effect. This is what makes the toss release-velocity land.

### Documentation
- [docs/developer/shadowing.md](docs/developer/shadowing.md) — the Shadowing method +
  paper scaffold.
- [docs/developer/rl-current-state.md](docs/developer/rl-current-state.md) consolidated
  as the single canonical RL status; the v4.0.0 RL headlines were corrected against it
  (see the **Errata** in the v4.0.0 section below).
- **`CLAUDE.md`** added at the repo root (a one-line `@AGENTS.md` import) so Claude
  Code loads the same project instructions every other agent already reads from
  `AGENTS.md` — no duplicated content, other tools unaffected.

### Build / packaging
- Newton runtime bundle size documented consistently as **~600 MB** (the CHANGELOG and
  the Makefile comment previously said ~450 MB).

---

## [v4.0.0] — 2026-06-12

The engine flips and the walking release. **Newton becomes the default
physics solver** (`physicsBackend "auto"` resolves to Newton; the build
ships `OMNISIM_WITH_NEWTON=ON`); the wgpu render arm reaches its
architectural baseline and is compiled in by default, but **WREN remains
the runtime default main-view renderer** — the Phase ζ flip is deferred
because wgpu is not yet a feature superset of WREN. `OMNISIM_LEGACY=1`
demonstrably reverts the whole stack to ODE + WREN. On top of those
arms: **G1 walks 212 m / 10 min in deploy with zero falls** (mathematical
foot-space planner + IK + residual RL); the first Newton-native Spot
walker walks STRAIGHT with locked heading; the **AnyPick Line** demo
sorts mixed parts with a learned ONNX classifier; **Omni Quest** routes
a Husky across the whole city via an OpenStreetMap-derived sidewalk
graph and a cost-optimal A* router; **The Living City** ships as a
generator-driven 4×4 urban grid; Open Robot Combat (ORC) and arm
bin-picking land as polished demos. 652 commits since v3.0.0.

> **Errata (updated 2026-06-23):** three reinforcement-learning headlines in this
> v4.0.0 entry were later found to overstate the *deploy* result. The honest,
> canonical per-robot status is
> [docs/developer/rl-current-state.md](docs/developer/rl-current-state.md) — where it
> disagrees with the bullets below, it is right.
> - **G1 walk** — "G1 walks 212 m / 10 min, zero falls" does **not reproduce in
>   deploy** (that policy topples ~1 s). **No from-scratch G1 policy walks durably
>   free-standing**; the deployed ones topple in ~1.3–1.7 s, and the good-looking G1
>   gait is harness-supported. A durable free-standing deploy walk is **still open**.
>   (Later figures quoting a "finite ~34 s bout" are also not reproducible — that
>   checkpoint is absent from the repo. Quote no G1 walk distance from this section.)
> - **G1 stand** — the deploy stand is real, but it is solved by a **deterministic
>   classical pose (statics), NOT RL**; the heavy-DR RL residual actually
>   *destabilises* it (~2.4 s vs 12 s+ for the pure pose).
> - **Spot "walks straight under Newton"** — under **Newton** the chassis tips (roll
>   instability); at v4.0.0 Spot walked straight only under **forced ODE**, and even
>   there the learned residual was a *passenger* (≈ the same distance with no policy
>   at all). Spot/Newton walking improved in later cycles — see the v4.5 notes and
>   rl-current-state.md for current status.

### Breaking changes

- **`OMNISIM_WITH_NEWTON` defaults to ON.** The build links the Newton
  XPBD solver by default. Distributions that need the old behavior must
  build with `OMNISIM_WITH_NEWTON=OFF` explicitly. **Source builds also
  need the Newton Python runtime** (`newton`, `warp`) on the embedded
  CPython's `sys.path` — run `make -C src/omnisim bundle-newton-runtime`
  to vendor it next to the binary (one-time, ~600 MB). Without it,
  worlds silently fall back to ODE; confirm Newton is live via the
  `[WbNewtonBackend]` line in startup log. Release installers ship the
  runtime pre-bundled. See
  [docs/developer/newton-runtime-bundle.md](docs/developer/newton-runtime-bundle.md).
- **`Solid.physicsBackend "auto"` resolves to Newton.** Worlds that
  depend on ODE-specific contact behavior must pin `physicsBackend
  "ode"` on the Solid (or via the world template). The capability gate
  auto-falls-back to ODE for Newton-unsupported features (e.g. mixed
  hinge+ball articulations) and warns; check `omnisim_log.txt`.
- **`OMNISIM_WITH_VULKAN` defaults to ON** (the build flag that gates
  the wgpu render arm). Builders without a `wgpu-native` toolchain must
  pass `OMNISIM_WITH_VULKAN=OFF` explicitly. **The runtime default
  main-view renderer remains WREN** — wgpu is compiled in but opt-in
  per world via `Viewpoint.renderBackend "wgpu"` (or test-only
  `OMNISIM_WGPU_MAINVIEW_FORCE=1`). The Phase ζ default flip is
  deferred because wgpu is not yet a feature superset of WREN. See
  [docs/developer/wgpu-renderer-status.md](docs/developer/wgpu-renderer-status.md).
  In short, the v4 engine defaults are **Newton physics + WREN
  rendering**.
- **`physicsBackend` ancestor resolution is now strict.** An explicit
  ancestor backend governs descendant `"auto"` — previously articulation
  could split solvers and leave sensors frozen. Worlds that relied on
  the loose behavior may need an explicit per-Solid override.
- **The full-range revolute importer reclassification.** URDF joints
  with a full ±2π range were previously misclassified as velocity wheels
  under Newton; they now register as revolutes. Worlds whose behavior
  depended on the misclassification need to be re-tuned.
- **`OMNISIM_FORCE_ODE=1`** is the new escape hatch for ODE-only
  determinism tests on a Newton-ON build. `OMNISIM_LEGACY=1` reverts
  both physics *and* rendering to the legacy ODE+WREN combo and is the
  documented full-stack rollback.
- **Sponsor button + funding wiring.** A repository-level Sponsorships
  toggle is now expected; `SPONSORS.md` auto-refreshes daily via a
  GitHub Action against the Sponsors API.

### Headlines (new since v3.0.0)

- **G1 humanoid WALKS in deploy** — `+212 m / 10 min, zero falls` _(⚠️ does not
  reproduce in deploy — see Errata above)_ via a mathematical human-gait model (foot-space planner + IK + residual
  RL on top). Earlier in the cycle: G1 STANDS in deploy; then G1 walks
  the platform `+25.9 m / 68.5 s, zero falls`; then full-body 23-DOF
  walk with natural arm swing. The deploy bridge is 56× faster than
  the first cut (`0.08× → 4.5× realtime`); the previous bottleneck was
  launch/sync, not the solver.
- **AnyPick Line** — a perception-driven, learned-classification
  end-of-arm-tool bin-picking line. L-conveyor feeds yawed bins of
  mixed parts; the arm sorts with a 200 KB ONNX classifier (gear / bar /
  T-bracket / tube + a model-free unknown lane). 5-seed reliability
  sweep: 5/5 PASS, 96% emptied / 92% sorted. Cycle time `318 s → 168 s`
  for 18 parts (1.9×). Breakable suction (shear budget), overlapped
  bin staging, contact-honest vacuum (no magnet grabs), strict
  top-down tray placement.
- **Omni Quest full-city navigator** — Husky routes the whole road
  grid via `build_city_graph` (OpenStreetMap → sidewalk graph derived
  from the road grid) and a general cost-optimal A* router with faster
  reroute. A learned free-space segmenter transfers sim→real (ML
  perception loop), and a histogram-match adaptation closes the
  camera sim-to-real gap by 96%. Stereo cameras bumped 128×96 →
  256×192 to close the structural sim-to-real gap. A Locomotion
  actuation seam (twist commands, not wheel velocities) makes the
  algorithm cross-platform (Husky + Jackal, same code).
- **Spot deploy hardened** — six root causes fixed across trainer /
  eval / deploy; Spot now walks with LOCKED heading (gait yaw-steering
  fix + env-port self-collision deadlock fix).
- **city_traffic perf 3×** — `0.34× → 1.0× realtime` end-to-end on
  the showcase city.
- **All RL worlds migrated to Newton** — no ODE stragglers; robot-
  combat demos migrated too; the arm assembly-line demo retired.

### Engine architecture (the migration baseline lands)

- **`architectural-baseline-v1` tag** on commit `bfbe1262` — both arms
  of the engine migration are *structurally* done; the runtime default
  flip for rendering (Phase ζ) is deferred (see
  [wgpu-renderer-status.md](docs/developer/wgpu-renderer-status.md)).
- **`OMNISIM_LEGACY=1` reversibility proven** via `scripts/dev/
  reversibility_check.py` + `OMNISIM_PROBE_BACKENDS` — flipping the
  knob restores ODE+WREN end-to-end.
- **P1.6 joint-op widening COMPLETE** — hinge angle reads, slider +
  AMotor reads, user-defined force/torque writes, per-step
  `setParam(FMax, Velocity)`, joint enable/disable lifecycle,
  world-load `setParam` family, Hinge2 + Ball `setParam`. WbSolid is
  fully migrated off direct ODE joint ops.
- **Unified engine plan** — the five rendering plans and six physics
  plans were consolidated into a single canonical
  [engine-migration-plan.md](docs/developer/engine-migration-plan.md).
- **Default-flip migration framework** — Stage 0 (escape hatches +
  dual-backend oracle + physics CI gate via invariants), Stage 1
  (capability-gate auto→ODE for Newton-unsupported features), Stage 2
  (Newton safe-default base guard + `WorldInfo.substeps`/`statics`),
  Stage 3 (Newton build default flips ON), Phase D FIRE (Solid.wrl +
  Robot.wrl `physicsBackend` default → `"auto"`).
- **`WorldInfo.defaultPhysicsBackend` / `defaultRenderBackend`** —
  pin a whole world to a specific backend pair without touching every
  Solid/Camera.
- **Symmetric legacy escape hatches** — `OMNISIM_FORCE_ODE`,
  `OMNISIM_FORCE_WREN`, `OMNISIM_LEGACY` (sets both).
- **Multi-session migration plan** — parallel-lanes split for engine
  work spread across sessions, with sanctioned isolated git worktrees
  and atomic path-scoped commits.

### Rendering (wgpu — the new render arm, opt-in)

- **wgpu architectural baseline COMPLETE** — A1/A2/B1/B2/B4/C1/C2 all
  signed off. The `WbRenderBackend` C2 surface is final; the
  `WbVulkanBackend` (a thin shim) routes through the wgpu pane.
- **wgpu beats WREN on the city benchmark.** Quality gate at 65%
  within-tolerance (gate threshold: 55%); 2.7× faster main view via
  draw-list and bind-group caches + async pipelined readback.
- **Phase ζ runtime default flip DEFERRED.** WREN remains the default
  main-view renderer; wgpu opt-in via `Viewpoint.renderBackend "wgpu"`
  or `OMNISIM_WGPU_MAINVIEW_FORCE=1`. Decision recorded
  2026-06-11.
- **Native window-swap** — wgpu presentation for the main view via the
  native swapchain; non-sRGB swapchain fix kills the double-gamma
  washout on window swap; Mailbox present for frame pacing.
- **Reversed-Z depth** in the wgpu main view — kills far-field
  z-fighting (city orbit distance).
- **Authored near-plane** — eliminates road z-fighting at orbit
  distance.
- **SSAO** in the wgpu main view — contact darkening with depth weight.
- **Bloom** post-process in the wgpu main view.
- **HDR + AgX tonemap infrastructure** for the main view (opt-in via
  `OMNISIM_WGPU_AGX`).
- **Distance fog + anisotropic filtering** in the wgpu main view;
  linear-space fog composition fixes the zoom-out fade; WREN-exact fog
  curve restores distant street/grass colour.
- **Camera-following fitted shadow frustum** — 4× near-shadow density
  on the same atlas; city-scale shadow frustum widened (12 m → 90 m
  half-extent, light back 130 m).
- **Main view un-gated** — `renderBackend "wgpu"` renders live (3c-B);
  the offscreen → GL blit path landed first, then the surface path
  superseded it (Option B interop ruled infeasible on pinned
  wgpu-native v29).
- **Material fidelity ladder** — albedo, roughness, metalness, normal
  maps; Cook-Torrance GGX specular BRDF; sRGB color management on lit
  shaders; specular highlight and worldPos plumbing landed via T1.1.
- **AgX filmic tonemap** ported to engine WGSL with pre-tonemap
  exposure, emissive HDR source, and a golden-image regression gate.
- **CSM cascaded shadow maps + PCF** (T1.2) — per-cascade
  orthographic light viewProjs, clip-depth shadow shader, shadow-pipeline
  3-binding group, two-pass shadowed render method, 3×3 percentage-closer
  filtering, multi-cascade receiver shader. Soft shadow edges observed;
  cross-object cast shadow verified by controlled A/B.
- **TAA** (T1.4) — sub-pixel Halton jitter + ping-pong history
  accumulator + temporal-resolve pass; `OMNISIM_PROBE_TAA_JITTER` /
  `OMNISIM_PROBE_TAA` probes.
- **Distance fog** (T1.3) — analytic distance-fog resolve;
  `OMNISIM_PROBE_FOG`.
- **Hemisphere-IBL ambient** (T1.x) — world-general hemisphere ambient
  in the wgpu main view + no-reference quality measure.
- **Atmospheric sky + day/night** in the wgpu main view.
- **Emissive term** in the wgpu textured-shadow path — city lights at
  night.
- **Soft shadows** — natural wgpu shadows with contact bias, 5×5 PCF,
  strength 0.8.
- **MSAA 4×** in the textured-shadowed pass.
- **wgpu sensor family** (R5) — RangeFinder (R32Float real-meters
  depth), Camera (sRGB sensor output kept linear), Lidar
  (single-layer → multi-layer → wide-FOV → multi-frustum → tilt →
  rotating-head → wide-FOV rotating). All with regression guards.
- **3c-A interaction kit** — picking, selection highlight, line +
  wireframe pipeline (bounding objects, COM, contact points, joint
  axes, surface normals, lidar rays, camera/sensor view frustums),
  translate / rotate / scale gizmos with hit-test drag, distance-scaled
  handles, depth-independent always-on-top markers, two-batch live-pane
  overlays, full-screen overlay compositing, screenshot pipeline,
  per-pixel roughness map, support polygon overlay.
- **Newton → wgpu interop** demonstrated end-to-end (Phase δ) — bulk
  Newton body-translation snapshot API feeds a wgpu storage-buffer
  instanced draw.
- **wgpu-native v29 integration** — sync-callback fix; LIBS-not-LFLAGS
  link fix; `OMNISIM_PROBE_WGPU` smoke knob; on-screen window present;
  texture cache LRU cap + path-keyed dedup.
- **R3.x runtime-verified path** — vertex/uniform/bind-group/pipeline-
  layout path; mesh cache + vertex buffer pipeline; texture bridge with
  cache + sampler + textured pipeline; QImage → wgpu texture adapter;
  golden-image regression harness; `WrStaticMesh → wgpu` byte-stream
  adapter; `WbCamera` routes through `WbWgpuRenderTarget`.
- **WREN → engine bridge** — `WbCamera` migrated onto the shared
  `WbWgpuSceneRenderer`; collect articulated-robot geometry (joints +
  CadShape) in the main view; WREN mesh-readback validation + plain-
  Appearance fallback in the wgpu collector.

### CUDA / physics (Newton — the new default)

- **Phase D FIRE** — `Solid.wrl` + `Robot.wrl` `physicsBackend`
  default flipped to `"auto"`. Worlds without an explicit backend now
  run on Newton.
- **W-series native primitives** — native plane collision (W1.1),
  native cylinder as oriented capsule (W1.2), native triangle-mesh
  collision (W1), Hinge2 2-DoF support (W2), ball/spherical joint (W2.2),
  external force/torque injection (W3.1 spine), mid-step body velocity
  sets (W3.2), supervisor force/torque API routed to Newton.
- **Native contact readback (W4)** — `get_contacts()` verified
  in-binary; C++ contact accessor + native-vs-ODE comparison harness;
  native contacts feed the supervisor API (verified 0 → 4 transitions);
  native contacts REPLACE the ODE source for multi-body verified.
- **Coverage meter (N-MEASURE)** — Newton-coverage dashboard; current
  reading ~60% of the ODE feature surface.
- **Real per-contact penetration depth** — `ContactPoint.depth`; the
  damage subsystem can opt in via `cp.depth` scoring (`L4`).
- **Arm friction grasp under Newton** via `SolverMuJoCo` — pick-place
  works on Newton (commit `6beae669`).
- **Newton actuates `staticBase` arms** (pin base, fixed-joint root);
  Newton unwraps Shape `boundingObject`s (no more `r=0.12` placeholder);
  prismatic joints + effort-scaled position gains; joint sensors read
  live angle (eval_ik under XPBD).
- **MuJoCo solver-stability engine knobs** — `OMNISIM_NEWTON_SUBSTEPS`
  fixes XPBD NaN at high drive speed; `OMNISIM_DAMAGE_VEL_SMOOTH`
  de-jitters contact velocity (57k → 58 events).
- **`OMNISIM_NEWTON_MJWARP`** — deploy via mujoco_warp (GPU) vs CPU
  mj_step; seed-rebuild uses the deploy engine, not hardcoded CPU.
- **statics-on-Newton dispatch (P8)** — top-level colliders register
  as Newton static bodies (opt-in); fixes Newton chassis-freeze caused
  by static furniture wrongly getting dynamic bodies.
- **Control-mode-aware joint ke/kd** — restores legged position-hold
  (G1 stand regression fix).
- **Articulation single-solver rule** — explicit ancestor backend
  governs descendant `"auto"`; fixes Spot frozen-sensor regression.
- **Rescue full-range revolute arms** from velocity-wheel
  misclassification (the `<rest>` trick generalized).
- **Multi-husky world-load hang fixed** — WbLog message accumulation
  was the culprit.
- **Newton 1.2.0 stable installed and verified** — Phase D gate #2 met.
- **Newton runtime bundling** — `bundle-newton-runtime` make target
  + warn at package time if unbundled.
- **Rolling-friction knob** — `OMNISIM_NEWTON_ROLL_MU` (AnyPick tubes
  stop rolling under it).
- **World-reload singleton teardown** — world reload silently dropped
  robots to ODE because the Newton backend singleton wasn't torn down.
  Fixed.
- **Collide-skip perf restored** — earlier exoneration audit confirmed
  it; restoring it brings back the throughput.
- **G1 walk deploy perf 56×** — `0.08× → 4.5× realtime`. The bridge
  was launch/sync-bound, not the solver.

### Reinforcement learning (humanoid + manipulation)

- **G1 WALKS in deploy — HUMAN GAIT.** _(⚠️ Corrected: does not reproduce in
  deploy — see Errata above.)_ `+212 m / 10 min, zero falls` in deploy. Recipe: mathematical foot-space planner + IK +
  residual RL on top, gait-v2 levers, the four launch fixes, and the
  deploy-gap ledger. Earlier in the cycle: G1 walks the platform
  (`+25.9 m / 68.5 s, zero falls`), G1 walks with arms (full-body
  23-DOF deploy), G1 walks with natural ARM SWING.
- **G1 humanoid (Unitree, 23-DOF) stands in deploy.** _(⚠️ This is a deterministic
  classical pose — statics, not RL; see Errata above.)_ First stable
  humanoid deploy stand on OmniSim. Recipe: heavy domain randomization,
  train on Newton-exact MJCF, deploy with stiff PD + planted feet +
  tuned CoM/ankle PD. The root cause of the original deploy gap was
  forward CoM + destabilising ankle PD, not a sim-to-sim gap.
- **GPU-native G1 trainer** — 5 stacked speedups vs the prior version;
  GPU mujoco_warp residual standing trainer.
- **Active capture-point balance law** for G1 deploy; deterministic
  G1 balancer prototypes (Phase A validation log).
- **Atlas RL pipeline** — port G1 stand recipe to Atlas; baseline-
  equivalent stand policy + DR curriculum.
- **Trainer initial DR** — base-tilt and base-velocity init randomization
  (`--dr-init-tilt-band` / `--dr-init-vel-band`).
- **Canonical two-layer control architecture** — deterministic
  controller + RL residual; documented in [docs/developer/](docs/developer/).
- **Sim-to-deploy gap closing** — three deploy obs-frame fixes for
  Newton; deploy obs feeds body-frame ang-vel to match the trainer.
- **First Newton-native Spot walker** _(⚠️ see Errata above: under Newton the chassis
  tips; straight-line walking was forced-ODE only at v4.0.0)_ — `spot_residual_main` walks
  STRAIGHT under Newton via the model+residual recipe (analytic CPG
  prior + learned residual correction), with heading + steer-to-
  centreline hold for path tracking. Six root causes fixed across
  trainer / eval / deploy; gait yaw-steering fix; env-port
  self-collision deadlock fix; Spot walks with LOCKED heading.
- **All RL worlds migrated to Newton** — no ODE stragglers left.
- **Arm bin-picking journey** — friction grasp under Newton; suction
  end-effector empties dense bin 36/36 via grab+shake singulation;
  tilt-and-pour bin emptying; colour-sort 36 cubes into trays; real
  collider walls + 18-part pile; camera-driven randomly-filled bin
  routine; residual-RL grasp layer.
- **Manipulation push/declutter exploration** — residual lifts
  emptying 15 → 18 cubes; pushing capped by gripper geometry.

### Demos & worlds

- **The Living City** (`showcase/city_traffic.wbt`) — 4×4 generator-
  driven urban grid: 48 cars routing with right-turns + 2-phase signals,
  36 pedestrians on zebra crossings, wall-to-wall mixed-use blocks,
  shops and restaurants with cafe seating, central park and landmarks,
  city bus that pulls up at stops, day/night cycle with a lit-up night.
  Regenerable via [`scripts/dev/gen_city_traffic.py`](scripts/dev/gen_city_traffic.py)
  + `city_grid.json`.
- **`environments/city.wbt`** — mixed urban street block as a default
  environment biome; `Car.proto` + parked/queued fleet.
- **Omni Quest** (`projects/omni_quest/`) — outdoor GPS+camera nav:
  M1 GPS waypoints, off-road course (rough terrain + obstacle avoidance),
  real camera-based obstacle avoidance (M3), cross-platform navigation
  (Husky + Jackal, same algorithm), interacting swarm, pedestrian
  sidewalk navigation, stereo-camera + GPS local planner, OpenStreetMap
  → routable walking graph, KITTI stereo + GPS + IMU sensor adapter,
  native EKF state estimator, deploy-with-reroute live in the city.
- **G1 deploy worlds** — `rl/g1_stand_deploy.wbt`,
  `rl/g1_stand_arms_deploy.wbt`.
- **Spot Newton deploys** — `rl/spot_residual_deploy_newton.wbt` (+
  perturb variant); the model+residual recipe live.
- **Open Robot Combat (ORC)** — `robot_combat/orc/orc_open_field.wbt`
  (2v2), `orc_forest_war.wbt` (3v3 across a 40 m wooded battlefield),
  `orc_queen_defense.wbt` (protect-the-queen mode); 20v20 verified
  end-to-end through the harness.
- **BattleBox** — contact-point-gated damage, physical damage +
  immobilization win condition, loud winner banner; tribute matches
  (Hydra vs Gravedigger flipper-vs-spinner, Gravedigger vs BiteForce
  heavyweight, --weapon-mode pulse for flippers).
- **Arm bin-pick worlds** — dense bin emptying, suction + grab/shake,
  tilt + pour, colour sort, robot-tilts-bin demo.
- **AnyPick Line** — perception-driven sorting line. Shape-agnostic
  suction bin picking + shape sort for the arm; L-conveyor feeds yawed
  bins of mixed parts (18/18 sorted); LEARNED part recognition via a
  200 KB ONNX classifier (gear / bar / T-bracket / tube + a model-free
  unknown lane). Reliability sweep: 5 seeds, 5/5 PASS, 96% emptied /
  92% sorted. Cycle time 318 s → 168 s (1.9×). BREAKABLE suction (seal
  has a shear budget). Overlapped flow (next bin stages during
  picking). Contact-honest vacuum (no magnet grabs). Strict top-down
  tray placement (no joint-arc swing). Containment (no part ever
  falls out). Sticky-gum gripper + model-free unknown-part lane. Gears
  + bars route to the BLACK bin.
- **`omni_quest` full-city navigator** — Husky routes the whole road
  grid. `build_city_graph` derives the sidewalk graph from the road
  grid; general cost-optimal A* router + faster reroute; learned
  free-space segmenter transfers sim→real (ML perception loop); camera
  sim-to-real gap closed 96% (histogram-match adaptation); stereo
  cameras bumped 128×96 → 256×192; Locomotion actuation seam (twist
  commands, not wheel velocities) makes the algorithm cross-platform
  (Husky + Jackal, same code).
- **city_traffic perf 3×** — `0.34× → 1.0× realtime` end-to-end on
  the showcase city.
- **`robot_combat` demos migrated to Newton**;
  the arm assembly-line demo retired.

### Procedural worlds (`omniworld`)

- **`city` biome** — joins forest, desert, urban_block, warehouse,
  indoor_apartment, and mars in the recipe set.
- Living-city regeneration via `gen_city_traffic.py` + readable
  `city_grid.json` (resize via XS/YS, re-run).

### Authoring & harness

- **`#include` directive for world files** (VRML extension).
- **`--minimize` + DEVNULL stdout/stderr** in headless runner —
  unblocks Newton 20v20 in batch.
- **Sensor parity oracle** (R5) — wgpu Camera path verified against
  WREN with retry + duration knobs.
- **Capability-gate harness** — Tier B verified (mixed hinge+ball
  articulations auto-fall-back to ODE with a warning); Tier A
  best-effort-safe.
- **Physics dual-backend oracle** — legacy verifies Newton via
  `scripts/dev/render_oracle.py`.
- **Physics CI gate via physical invariants** — `scripts/dev/
  physics_oracle.py` validates both backends agree on energy /
  momentum / quasi-static equilibrium.
- **Render-arm completion checklist** + golden-image regression gate.
- **Newton coverage meter** (`scripts/dev/newton_coverage.py`).
- **wgpu probe knobs** — `OMNISIM_PROBE_WGPU`, `_PROBE_PICK`,
  `_PROBE_INSET`, `_PROBE_CSM`, `_PROBE_TAA`, `_PROBE_TAA_JITTER`,
  `_PROBE_FOG`, `_PROBE_BACKENDS`, `_PROBE_TEX`.
- **MinGW runtime DLLs shipped into `lib/controller/`** so Windows
  controllers load.

### Damage system

- **Contact-point-gated damage** — opt-in `cp.depth` scoring in
  `battlebot_damage_director` (L4).
- **`cp.depth` restricted to non-load-bearing parts** to avoid spurious
  chassis-on-chassis inflation.
- **`OMNISIM_DAMAGE_VEL_SMOOTH`** — de-jitter Newton contact velocity
  (57k → 58 events).
- **DamageTracker reset** clears `vel_ema` per its docstring.
- **damage_director** strips leading DEF from exported part before
  reinserting.

### Build / packaging

- **`OMNISIM_WITH_NEWTON ?= ON`** is the new build default (Stage 3).
- **`OMNISIM_WITH_VULKAN ?= ON`** is the new build default (C1
  architectural baseline).
- **`wgpu-ON` builds auto-ship `wgpu_native.dll`** (supported config).
- **Newton runtime bundling tool** — `bundle-newton-runtime` make
  target; package-time warning if unbundled.
- **`build_with_cd.sh`** forwards `OMNISIM_WITH_VULKAN` and
  `WGPU_NATIVE_HOME` to make.
- **`setup_wgpu_native.sh`** repaired — `wgpu-native v29.0.0.0 (gnu)`
  pinned.
- **Sponsors automation** — `.github/workflows/update_sponsors.yml`
  auto-refreshes `SPONSORS.md` daily from the GitHub Sponsors GraphQL
  API (four tiers via marker pairs).

### Release infrastructure

- **`publish_snapshot.sh` bypasses the local pre-push smoke hook** via
  `OMNISIM_SKIP_PUSH_CHECK=1`. The hook ships that env var as its
  documented escape hatch for binary-less worktrees.
- **`tmp_*.err` scratch files added to `publish_deny.txt`** — agent-run
  stderr captures stay private.
- **Sponsors auto-refresh GitHub Action** runs daily; opt-in tier
  markers in `SPONSORS.md`.

### Documentation

- **Consolidated engine plan** — five rendering plans and six physics
  plans absorbed into a single master plan
  ([docs/developer/engine-migration-plan.md](docs/developer/engine-migration-plan.md)).
- **R3 wgpu-native + Newton-interop design** rewritten and pinned.
- **R4 → wgpu-default completion checklist** as a living tracker.
- **G1 standing playbook** + general sim-to-deploy RL recipe.
- **Canonical RL state doc** — correct stale "stands forever" headlines
  and add base-divergence guard.
- **Humanoid balance gap** — the actual blocker for Atlas + G1 RL,
  characterized.
- **Contact API consumer inventory** + coordination requirement +
  migration entanglement notes.
- **Newton runtime bundling guide** + Newton-as-default release
  procedure.
- **Sponsors automation runbook** (private).
- **Architectural baseline checklist** + final default-flip plan.
- **Parallel-lanes split** for multi-session migration work.
- **Reproducible codebase-stats.md** ("lines we wrote").
- Dropped the unverifiable "first" claim from public positioning.
- **Webots-named icons + web frontend files** renamed to OmniSim.

### Fixed

- **`physicsBackend` ancestor governs descendant `"auto"`** — Spot
  frozen-sensor regression.
- **Articulation must use one solver** — split-solver articulation was
  a regression source.
- **Newton chassis-freeze** — static furniture wrongly got dynamic
  bodies.
- **Newton motor ke/kd wiring** — env defaults silently overrode
  `WbBasicJoint` values.
- **Newton head-on XPBD NaN** at high drive speed — `_SUBSTEPS` knob.
- **G1 floor-contact deploy regression** in the recipe.
- **Spot W6 deploy collapse** — diagnosis pinned to ROLL/lateral
  stability + control-bridge issues.
- **Newton multi-husky world-load hang** — WbLog message accumulation.
- **Newton world-reload silently dropped robots to ODE** — singleton
  world teardown missing on reload.
- **wgpu reversed-Z + authored near plane** — far-field z-fighting and
  road-at-orbit-distance z-fighting on the city.
- **wgpu periodic motion hitch** in the main view eliminated.
- **wgpu non-sRGB swapchain** — window-swap double-gamma washout.
- **wgpu linear-space fog composition** — zoom-out fade fixed.
- **wgpu WREN-exact fog curve** — distant street/grass colour stops
  fading to a wrong tone.
- **wgpu `TextureTransform` applied** — omni_quest grass was one tile
  smeared over 200 m.
- **wgpu `castShadows` honored** — sun marker was shadow-bombing
  spot.wbt.
- **wgpu corpus-sweep fixes** — dither, normal-offset shadows,
  diagnostic switches.
- **Deterministic offscreen parity golden** — crash fix + gate
  recalibrated.
- **Render-parity gate clarification** — WREN is legacy-dark
  (advisory, not a regression).
- **Spot GPU pipeline sim2sim fidelity** — six root causes fixed
  across trainer / eval / deploy. The pipeline was never sim2sim-
  faithful before this.
- **Spot env-port self-collision deadlock** + **gait yaw-steering
  break** — Spot now walks with locked heading.
- **AnyPick LINE queue** — yawed bins no longer interpenetrate.
- **AnyPick LINE containment** — no part ever falls out.
- **AnyPick LINE strict top-down tray placement** — no joint-arc
  swing.
- **AnyPick LINE true-surface gauge** for model-free unknowns + long-
  part wall corridor + flush model-free picks (no hover).
- **AnyPick LINE tube rolling** — fixed via the Newton rolling-friction
  knob.
- **AnyPick LINE contact-honest vacuum** — no magnet grabs.
- **WbCamera FOV match** — wgpu pane frames at any aspect (R4-3b
  polish).
- **wgpu shadow-render non-determinism** — dangling `modelMatrix16`
  per-draw uniform; pinned via headless RenderDoc capture.
- **Textured-shadowed floor-drop** — pinned to GROUND-plane skip; root
  cause is a timing/sync race, fix landed.
- **Camera sensor sRGB regression** — keep camera SENSOR output linear
  (sRGB is display-only); R5 regression green.
- **wgpu texture-cache key bug** — path-key the cache to dedupe
  shared-file textures; multi-world soak now clean across 6 worlds.
- **wgpu main-view reload crash** + NaN-normal handling + live blit.
- **`build_with_cd.sh` engine-flag forwarding** — exporting before the
  build no longer reaches the child make.
- **`isolated-worktree engine build`** — Qt5+Qt6 coexistence link
  failure documented; junction the gitignored vendored deps.
- **G1 STANDS in deploy** — root cause was forward CoM + destabilising
  ankle PD (not a sim2sim gap).
- **G1 RL deploy obs-frame** — body-frame ang-vel matches the trainer.
- **Spot deploy heading + steer-to-centreline hold** — walk straight
  down the path.
- **Newton actuator position-target writes** were silently no-op'ing.
- **Newton chassis visual** now follows physics (WREN-push fix).
- **MinGW runtime DLLs** ship into `lib/controller/` so Windows
  controllers load.

### Removed

- **Splash images for non-shipped robots.**
- **Stale Cyberbotics path strings** + dead-language pages
  (docs cleanup).
- **`WEBOTS_HOME` self-reference bugs** purged from dev docs ahead of
  public publish.
- **Pre-rebrand `Webots-named` icons + web frontend files** renamed to
  OmniSim equivalents.
- The legacy single-solver articulation policy (now strictly enforced).
- **The arm assembly-line demo retired** — superseded by the AnyPick
  Line and the bin-pick suite.

---

## [v3.0.0] — 2026-05-25

OmniSim stops introducing itself as a Webots fork and starts behaving as
its own engine. Five "dual-accept" compatibility shims that the v2.x
rebrand phases left in place are now strict OmniSim-only (env var, URL
scheme, binary name, project manifest, canonical header tree), the
`src/webots/` source folder is renamed to `src/omnisim/`, and the
physics default flips from ODE to Newton. The new Spot walker — straight
walking via a model+residual recipe — is the first piece of locomotion
that runs end-to-end on the Newton path. A render-backend abstraction
lands as the seam for the upcoming Vulkan migration.

### Breaking changes

- **Source-tree rename: `src/webots/` → `src/omnisim/`.** External code
  that builds against OmniSim sources (private forks, extension modules)
  must update include paths. `omnisim/*.h` and `omnisim/*.hpp` are
  canonical; `webots/*.h` shims remain for one release window only.
- **`WEBOTS_HOME` env var is no longer read.** Set `OMNISIM_HOME` instead.
- **`webots://` URL scheme is no longer accepted.** Use `omnisim://`.
- **`webots-bin.exe` binary alias is gone.** The shipped binary is
  `omnisim-bin.exe` (and the controller equivalent). Anything launching
  `webots-bin.exe` by name will fail.
- **`webots.yaml` project manifest is no longer accepted.** Rename to
  `omnisim.yaml`.
- **Default physics solver flips from ODE to Newton.** `physicsBackend
  "auto"` now resolves to Newton, and new empty worlds nudge Newton for
  freshly imported robots. ODE remains available as the documented
  legacy fallback by setting `physicsBackend "ode"` explicitly on the
  Solid (or via the world template). Worlds that depend on ODE-specific
  contact behaviour may need that flag added.
- **`rename_audit` ceilings lowered** — the policy now enforces
  omnisim-only naming across the surfaces it covers; legacy occurrences
  that previously passed will fail audit.

### Engine architecture (the migration plan reaches a milestone)

- **WbPhysicsBackend dispatcher — P1.5 milestone:** WbSolid is fully
  migrated off direct ODE body ops. Position, quaternion, velocity,
  point-velocity, force, torque, body enable/disable, and
  setGeomAndBodyPositions all route through the backend dispatcher.
- WbGyro, WbAccelerometer, and WbGps dispatch via
  `WbSolid::bodyHandle()` — sensors no longer reach into ODE directly.
- WbNewtonBackend implements the dispatcher's pose-read methods
  (`getBodyPointVel`, position, quaternion) for the body-ops surface.
- `WbConnector::rotateBodies` and `WbSolidMerger::setGeomAndBodyPositions`
  converted; force/torque/velocity application widened to the dispatcher.
- `wb_supervisor_node_add_force[_with_offset]` and
  `wb_supervisor_node_add_torque` are now polymorphic across backends.
- Engine-migration plan tracks P1.5 as COMPLETE; cuda-newton-physics
  plan tracks P7 PARTIAL and P9 COMPLETE.

### Rendering (Vulkan migration seam)

- **R0** — `WbRenderBackend` abstraction lands as the unified seam
- **R1** — `OMNISIM_WITH_VULKAN` build flag + `WbVulkanBackend` extracted
- **R2** — `renderBackend` SFString field on Viewpoint + Camera
- R3 — rendering-backend evaluation + design doc written up
- chassis visual now follows physics (WREN-push fix) + verify-as-shown
  tooling

### Newton physics

- **Real fix for the joint-glitch** — clean-build flag + widened URDF
  deploy + analyzer
- Hard post-step joint-limit clamp + stress test
- `armature`, `limit_ke`, `limit_kd` env vars for joint physics tuning
- `getVelocity()` root-cause fix — was returning 0, broke deploy
- NaN-safe ODE pose writeback in `WbSolid::applyPhysicsTransform`
- `OMNISIM_NEWTON_MJWARP` to deploy via mujoco_warp (vs CPU mj_step)
- broadphase "auto" mode now resolves via top-level Solid AABB
- `worldinfo.broadphase` is a real SFString field; ODE switching plumbed
  end-to-end
- `gatherSleepingStats` — sleeping-island verification telemetry
- WREN instancing-candidate run detector (item 1 of large-world plan)

### Reinforcement learning (the residual recipe)

- **First Spot walker that walks STRAIGHT under Newton.** Method:
  model+residual recipe (analytic CPG prior + learned residual
  correction). Shipped as the canonical `spot_residual_main`.
- Model-based Spot walker (analytic IK + trot gait) — beats v12_200k
  PPO with zero neurons; ships as Phase 4 balance PD + heading-lock
- Custom residual-RL system on the model walker (Phase 5)
- GPU residual deploy controller + docs for the full Spot-walk journey
- GPU mjwarp residual trainer on RTX 5070 with backwards-walking fixes
- Recovery agent: leg-based self-righting on fall (no supervisor
  teleport), model-based righting (orientation-aware geometric), realistic
  motor limits (torque cap + rate-limited targets), reward redesign +
  curriculum, training pipeline scaffold
- Push-recovery perturbation experiment + Newton joint-limit diagnostic
- Action-magnitude penalty (`--act-pen`) for smoother gait
- Live MuJoCo viewer for the GPU residual Spot walker
- Spot URDF: `<rest>` tag extension; widen `hip_x` and `hip_y` to ±1.5
  rad for self-righting
- Heading-lock control + verify harness + reward-shaping knobs
- Extended obs vector to 50 dims (added heading-deviation);
  `SPOT_OBS_DIM` env override
- Deploy fixes: clear finite-diff vel history on loop-reset; faithful
  Spot walker walks 0.45 m/s forward in OmniSim

### Demos & worlds

- **BattleBox** — BattleBots-style combat sport scene
- **robot_combat** — new project for all combat demos; 10 worlds + 2
  controllers moved
- 20-husky top-down arena variant + double-the-huskies marketing scene
- husky_fleet_arena top-down capture variant + shotlists
- husky_maze: cell-level loop detection + min-pivot path planning,
  perception-as-tool (hide `read_camera`, scan_surroundings only),
  shake-free + smarter wedge-escape (basic maze now reliably solves),
  wheels-only navigation (no teleport recovery), agent-written memories
  from 2026-05-24 successful runs, refuses `complete_mission` while a
  bridge fault is live, restored `goto_cell` recovery
- Arm assembly line: real physics pick-and-place + grip-confirm sensors
- Arm bridge: working physics pick-and-place — 6-DOF IK + real grasp
- Spot demos: hide-on-start sun marker + higher initial camera
- walk_demo: restored default dark PBR ground (dropped tile pattern)
- fleet_cam: headless Camera-device recorder for husky fleet arena
- Saved default view perspectives for Spot Newton worlds

### Authoring & harness

- WbProject resolves project root for worlds nested in subdirectories
- Smoke gate auto-builds missing controller binaries before push
- Scripts self-locate the repo root in dev build scripts
- `broadphase_auto.wbt` smoke world

### Branding & social

- **Brand book**: particle orb, OMNI/SIM wordmark, mimosa palette
- Replaced Webots-branded textures with OmniSim equivalents
- Omnivoice — voiceover tool for the youtube_videos scripts
- Husky combat video pipeline + house video style guide
- Tier 1 video polish — LUFS master, captions, tracking camera
- Tier 3 — Claude critic, smoke gate, b-roll, Discord poster
- Multi-angle combat assembler + 2 new b-roll entries
- Original synthesized soundtrack for the topdown video
- ODE "spot walks straight" journey video + ODE walk demo world
- "Teaching a robot dog to walk" journey video — before/after Spot gait
- husky_maze journey video — 5 top-down shotlists + builder
- Recovery-prompt terminal scene + typing SFX mux
- FUNDING.yml points at the omnilink-tech org; omnilink-agents.com
  custom link added

### Build / packaging

- Full release-build unblockers + URDF path fix
- Rename migration script: `scripts/.../rename_webots_to_omnisim.sh`
- Cleanup: prune empty howto/tutorials samples; rename `protogen_demo`
  → `protogen`
- Move retired/experimental worlds to `samples/_archive/`
- `gitignore`: `.wbproj` globally, `MUJOCO_LOG.TXT`, leftover scratch
- Removed orphan `src/webots/Makefile` left after Phase G

### Documentation

- User-facing Newton physics backend guide
- Migration perf comparison with measured Newton vs ODE numbers
- Newton XPBD scaling sweep (1–50 huskies) — benchmark + writeup
- Engine-migration plan kept in sync as P1.5 closed; item 6 closed
- cuda-newton-physics-plan — phase status block
- Newton-as-default + `src/webots` → `src/omnisim` rename plan doc;
  Phase J documents the rename as historic
- Spot residual-RL writeup: old PPO method vs new model+residual
- Physics pick-and-place + grasp-mode docs
- Scrubbed Java/MATLAB code blocks + language-list mentions
- Dropped dead-language pages + fixed stale Cyberbotics path strings
- Archived `REBRAND_PLAN.md` (all phases shipped); deleted
  `MIGRATION_PLAN.md`; references repointed at AGENTS.md §0

### Fixed

- Newton `getVelocity()` returning 0 (root cause of broken deploy)
- Newton joint-glitch — clean-build flag + URDF deploy widening
- NaN ODE pose writeback in `WbSolid::applyPhysicsTransform`
- WREN visual lagging chassis physics
- husky_maze: shake-free + smarter wedge-escape; reliable basic-maze
  solve
- GPU residual trainer: backwards-walking fix on mjwarp
- GPU residual deploy: cleared finite-diff vel history on loop-reset
  (no spike)
- rendering-normals smoke skip root cause documented

### Removed

- `webots://` URL scheme (use `omnisim://`)
- `webots-bin.exe` binary alias (use `omnisim-bin.exe`)
- `webots.yaml` project manifest alias (use `omnisim.yaml`)
- `WEBOTS_HOME` env-var read (set `OMNISIM_HOME`)
- MATLAB engine support — all docs/code-blocks cleaned up
- `REBRAND_PLAN.md` (archived after all phases shipped)
- `MIGRATION_PLAN.md` (all phases shipped)
- Orphan `src/webots/Makefile` left after Phase G
- Empty howto/tutorials samples

---

## [v2.2.0] — 2026-05-21

The physics-and-learning release. Newton finally holds Spot upright and
walking — a chain of solver fixes (qd-indexed joint targets, inherited-Solid
overwrites, shape-center offsets, collision filtering) turned the "parts
falling off" and "no-stand" bugs into a robot that stands, resets cleanly,
and trains. On top of that lands a GPU-batched MuJoCo-Warp PPO trainer
(~125k env-steps/s that actually learns), first-class pluggable gripper
support on arm robots, and a worlds reorganization into starter / showcase /
environments with three new cinematic environments (forest, desert ruins,
high-rise construction site). No breaking changes; several sample demos
were retired.

### CUDA / physics (Newton)

- **Spot STANDS** — collision filter + shape centers + position spring +
  live feedback. The milestone fix after a long bisect.
- Fixed the no-stand bug: `joint_target_pos` is qd-indexed, not q-indexed
- Fixed "parts falling off": Newton overwrite was skipping inherited
  Solids; scene-tree joint angle now read from Newton
- Fixed actuator position-target writes silently no-op'ing
- Realistic per-body inertia + per-joint effort/velocity/position limits
- Pass Pose translation into shape offsets so feet collide where rendered
- Pass URDF joint limits (minStop/maxStop) into the articulation
- Translate URDF mesh collisions to Newton AABB boxes (wrapper opt-in)
- Seed `joint_q` to the standing pose + rebuild solver; pose-seed reaches
  MuJoCo
- Supervisor-driven resets push pose into Newton's `body_q`; reset clears
  the spawn-pose freeze loop
- Settle Spot actuator on ke=200 / kd=5 (best stability + leg mobility)
- `setValueFromOde` overwrite now invalidates the matrix cache
- `OMNISIM_NEWTON_GROUND_MU` foot-friction knob
- Fixed a GPU-array leak that crashed training at ~200k steps
- XPBD-on-GPU by default for training (3× throughput); MuJoCo CPU fallback
  remains for NaN-prone scenes
- Per-call / per-joint / per-step diagnostics added during the bisect

### Reinforcement learning

- GPU-batched MuJoCo-Warp PPO trainer — ~125k env-steps/s and learns
  (PoC peaked at 813k env-steps/s); env-tunable solver iterations
  (default 10/8) for throughput
- GPU trainer auto-configures from MJCF + Spot export hook + eval
- Real trot gait + standing-regression diagnostics; legacy sin² gait stays
  default, trot is opt-in
- Stand-first training curriculum (from-scratch launcher seeds standing
  pose + stiff leg gain); value-warmup uses a critic-only optimizer for a
  correct actor freeze
- Warm-start launcher for SB3 `continue_training` under Newton physics
- Session isolation so parallel training agents can't kill each other
- Anti-B-mode reward shaping + forward-distance episode logging
- Knobs: `SPOT_PITCH_TRIM`, `SPOT_ACTION_SCALE`, uprightness weight, kd=60
  damping; fixed structural CPG nose-down
- Fixed train/deploy CPG mismatch and a deploy-eval bug that silently
  tested the wrong policy
- Opt-in per-episode body trace (`OMNISIM_AGENT_TRACE`)

### Grippers & arm bridge

- First-class gripper support on arm robots (plan → ship):
  - Phase 1 — decouple grippers into a pluggable effector layer
  - Phase 2 — richer surface: grasp / release / set_width
  - Phases 3 & 5 — kinematic grasp weld + pick-place demo + docs
  - Phase 4 — pluggable gripper drivers
  - Physics-grasp gripper with real 2F-85 fingers (WIP)
- Arm demo: refined gripper into a proper Robotiq 2F-85 shape, mounted
  visibly, anchored to the real flange node (fix clipping), grasp radius
  widened to 0.16 m for reliable picks

### Demos & worlds

- Worlds reorganized into `starter/`, `showcase/`, `environments/`
- New environments:
  - **forest** — real-mesh bushes + floor litter, hand-placed background
    ring, everything grounded (no hovering)
  - **desert_ruins** — ground + dune system, ancient architecture,
    cinematic golden-hour overhaul
  - **construction site** — realistic high-rise, more buildings + plant,
    real procedural construction vehicles, fully-editable dev world
- Unify lighting across all user-facing worlds via a 3-PROTO recipe
- Damage: driving arena sheds parts from real box impacts; husky_damage_arena
  tears parts off under falling weights; detached parts keep the robot's
  real colour
- Mavic 2 Pro: chat-style demo (chat_aerial); chat + Drone Surveyor
  flagship consolidated onto one world
- warehouse_patrol: 2026-05-21 patrol sweep manifests
- ConstructionFrameBuilding replaces Building + BuildingUnderConstruction

### Robots & PROTOs

- Inline pipe/torus boundingObject math; drop `projects/bounding_objects`
- street_furniture PublicToilet shows OmniSim branding, not Webots

### Authoring & harness

- `--no-window` for true background mode (no taskbar entry); suppress the
  world-loading progress dialog in `--no-window`
- Always inject `mingw64/bin` on the Windows controller PATH
- Allow in-place editing of bundled worlds/projects by default
- Smoke: pre-flight abort when port 1234 is held by a running Webots;
  per-world skip flag; mark rendering-normals broken
- Husky bridge: remove teleport-snapping globally — drive on wheels only

### Build / packaging

- CI: local pre-push smoke hook + WORLDS.md drift refresh
- gitignore the NVIDIA Corporation/ driver crash dir

### Documentation

- Worlds index; flip docs/README.md to `OMNISIM_HOME` canonical
- Spot + Newton session-state captures + spot_newton_v4 results writeup
- Plan for first-class gripper support; drop stale control_showcases doc

### Fixed

- Newton no-stand bug (qd-indexed joint targets)
- Newton "parts falling off" (inherited-Solid overwrite + scene-tree joint
  angle)
- Newton actuator position-target writes silently no-op'ing
- GPU-array leak crashing training at ~200k steps
- train/deploy CPG mismatch; deploy-eval testing the wrong policy
- mounted gripper clipping (anchor to real flange node)
- forest objects hovering above the ground

### Removed

- Demos: arm digital-twin, all_urdf_robots, urdf_ur5e, urdf_epuck,
  urdf_tiago, urdf_showcase, mobile_robots_showcase, omnibot_combat,
  two_omnibots, cube_bot, ur5e_omnilink kinematics, object_gallery,
  husky_fleet_outdoor (Axis repointed at warehouse_logistics)
- `projects/bounding_objects` PROTO directory (math inlined)
- Building + BuildingUnderConstruction PROTOs (use ConstructionFrameBuilding)

---

## [v2.1.0] — 2026-05-18

A consolidation release on top of v2.0.0. The headline themes: atmospheric
sky becomes the default backdrop across every world (with a draggable sun
marker), the Webots→OmniSim rebrand reaches into the engine layer via an
alias-not-rename phase plan (binaries, env vars, URL scheme, project
config all dual-accept the new names), a full reinforcement-learning
pipeline ships for Spot (and scaffolding for Atlas), and the OmniLink
integration picks up voice I/O, short-term memory, per-turn telemetry,
and a starter PyPI package. No breaking changes — every rebranded entry
point keeps its prior alias.

### Engine rebrand (Webots → OmniSim, alias-not-rename)

- REBRAND_PLAN.md staged in [Phase 0](docs/) — inventory + safety net,
  no functional change
- Phase A — cosmetic display strings rebranded
- Phase B — `omnisim-bin.exe` / `omnisim-controller.exe` binary aliases
- Phase D — `omnisim/*.h` and `omnisim/*.hpp` C/C++ header forwarders
- Phase E — `omnisim` Python controller package (forwarder)
- Phase F — `OMNISIM_HOME` canonical; `WEBOTS_HOME` dual-read alias
- Phase G — `webots://` URL scheme also accepts `omnisim://`
- Phase H — `projects/*/webots.yaml` also accepts `omnisim.yaml`
- Phase C (`Wb*` → `Om*` class rename) intentionally skipped to preserve
  the "built on Webots" attribution surface
- Doc + in-app sweep so new agent sessions read OmniSim, not Webots
- Two pre-existing breakages flagged during the verification gate, fixed

### Demos & worlds

- MissionControl: 6-Husky fleet on a logistics campus, agent-only
- HuskySwarm: 4-Husky OmniLink coordinator → upgraded to 34-tool
  meta-tool-pattern coordinator
- Construction-site logistics benchmark — scripted vs agentic control
- Multi-arm demo world (3× UR5e on a shared stage)
- Three specialist OmniLink agents shipped as examples — Foreman, Picker,
  Roomba
- Real-robot bridge starter kit example (no Webots, no OmniSim)
- In-sim demo gallery launcher (world + supervisor + Robot Window)
- Worlds subdivided by category; path-refs refreshed
- spot_newton_demo: switched to atmosphericSky + draggable sun marker
- Sweep removed classic-Webots demos that no longer fit OmniSim (twice —
  a merge restored them and they were re-deleted)
- Cinematic atmospheric-sky playground worlds + headless bench

### Atmospheric sky (the new default)

- Atmospheric sky installed on every world with a Background — full sweep
  across 330 atmospheric worlds (last 24 needed surrogate-escape for
  non-UTF-8 .wbt files)
- Legacy photo-cubemap backgrounds ripped out
- Draggable sun marker — Unreal-style glowing sphere; doesn't cast a
  shadow; bound to DirectionalLight position
- DirectionalLight.color → atmospheric sky `sunIlluminance` binding
- Night sky: procedural starfield, moon with maria, marker overhead,
  Milky Way band, varied stars, stars constrained to upper hemisphere,
  realistic moon
- Cinematic tonemap: S-curve contrast + saturation lift
- Per-world IBL auto re-bake on DirectionalLight edits
- Real Lambertian diffuse-IBL bake for atmospheric sky
- IBL white-balance pushed to 0.95 → 1.0 so PBR materials read true albedo
- Damped IBL bake against atmospheric-sky over-tint
- Perlin terrain + fix for IBL axis-swap bug in sky shaders
- Reverted one engine IBL iteration to restore a working baseline

### Robots & PROTOs

- Removed PR2 support entirely (no live demo consumers, blocked sweeps)
- Spot: end-to-end demo runs (articulation + position-bridge wired)
- Spot URDF inertia diagnosis from the RL deploy path

### CUDA / physics

- Newton: multi-parent articulation bug fixed on Spot tree (leaf-first
  joints)
- Newton MuJoCo-CPU env-var override

### Reinforcement learning (new in v2.1.0)

- End-to-end RL pipeline scaffolding for Spot — env wrapper, trainer,
  deploy controller, eval workflow
- Trained Spot policy: 200k-step PPO checkpoint (320 KB), then 100k
  walk-focused checkpoint
- Spot OmniSim walker — verified +9.79 m forward in ODE, never falls
- CPG trot prior + GPU env
- Atlas (Boston Dynamics v5) RL pipeline + MJX deploy-parity fixes
- Pluggable training backends: sb3 / mjx / isaac
- Platform-ify: generic deploy, robot registry, reward recipes
- Headless MuJoCo deploy
- MJX trainer NaN-defensive fixes
- MJX PPO loss fix — sum log_std along act dim, not full batch
- `kp=500` fix unlocks MJX deploy
- SpotEnv connect timeout bumped 30s → 90s
- Deploy controller path resolution + eval workflow
- Body trace from deploy controller + headless eval script
- env-var reward weights for the walk-focused trainer

### Agents (`agents/`)

- All OmniLink agents consolidated under a single `agents/` tree
- Voice I/O in the chat panel (mic in + agent voice out)
- Short-term memory: cross-session chat continuity
- Per-turn usage telemetry (tokens + credits) on every bridge
- Auto-register per-robot profiles + tool callback endpoint
- Demos depend on omnilink-lib explicitly + warn on outdated versions
- OmniLinkClient + sim-to-real docs
- Agent benchmark suite scaffold (3 tasks)

### Authoring & harness

- Agent-driven cinematic capture pipeline
- Marketing slate driver scripts + 4 shotlists
- Cinematic-pipeline 5 fixes (unblock end-to-end)
- Agent gallery page + demo-video capture script
- `headless_runner` honors `OMNISIM_LOG_PATH` for parallel runs
- URDF sensor-segfault bisect narrowed + device-smoke harness
- Untrack harness scratch worlds + gitignore them

### Build / packaging

- omnisim-bridges PyPI package (skeleton, locally installable)
- Prune vehicles target from `projects/Makefile` + relay Makefile stub
- gitignore runtime artifacts (`fps_sweep` summary, `*.egg-info`)
- Drop stray baseline artifacts from rebrand Phase 0

### GUI / UX

- View > Theme submenu — Light / Dark (Night) / Dark (Dusk)

### Release infrastructure

- `publish_snapshot.sh` auto-bumps `omniSimVersionString` in
  `WbApplicationInfo.cpp` to match the release tag before snapshotting
  (idempotent no-op when already current). Was a hand-maintained constant
  that drifted in both v1.0.10 and v2.0.0.

### Licensing

- TRADEMARKS policy added (open code, protected brand model)
- DCO contributor flow (not CLA)

### Documentation

- REBRAND_PLAN.md staging doc — engine refactor phases
- Top-level DEMOS, WORLDS, ARCHITECTURE, MIGRATION_PLAN indexes
- OmniLink integration roadmap — 13 items across 5 phases
- omnilink-roadmap.md annotated with status + commit hashes
- README + AGENTS.md + chat-demos surface every OmniLink artefact
- RL pipeline: README, smoke_deploy, AGENTS.md pointer, eval_policy
  usage + plateau notes
- RL handoff doc — resume-here section with commit SHA, GPU-box handoff

### Fixed

- Newton: 'multi-parent' articulation on Spot tree (leaf-first joints)
- MJX: PPO loss log_std summed along act dim, not full batch
- engine IBL: damp bake against atmospheric-sky over-tint
- sky shaders: IBL axis-swap bug
- two pre-existing breakages flagged during the rebrand verification gate

### Removed

- PR2 robot (no live consumers)
- Classic-Webots demos that no longer fit OmniSim
- Legacy photo-cubemap backgrounds (replaced by atmospheric sky)
- Stray baseline artifacts from rebrand Phase 0

---

## [v2.0.0] — 2026-05-14

OmniSim's first major release. The simulator's centre of gravity moves from
PROTO-authored Webots robots to a URDF-native, agent-driven platform: 14
canonical robots now load from URDF, every shipped demo is wired to the
OmniLink agent platform via the new Wire Protocol, the physics layer gains a
pluggable Newton/XPBD backend with CUDA-accelerated particles and rigid-body
contact, and the rendering path picks up an atmospheric sky stack with
real-world perf gains on the existing forest, mars, and warehouse scenes.

### Breaking changes

- Removed Pico TTS, Java, and MATLAB controller support. Python and C/C++
  remain. Worlds, samples, and build scripts that referenced these are gone.
- Removed the integrated text editor (-3,190 LOC). External editors only.
- Removed the Classic light theme. OmniSim is dark-only.
- Removed i18n machinery. UI strings are English-only for now.
- Removed Cyberbotics menu items and the legacy Webots.cloud upload flow.
- Removed ~30 legacy robots that had no canonical URDF and no live world
  consumers: e-puck, elisa, mir100, hoap2, k-team kheperas/hemisson/koala,
  pioneer2/3, p-rob3, thymio, boebot, fabtino, nao, aibo, qrio, crazyflie,
  shrimp, biorob, robotnik (summit_xl_steel), irb, scara_t6, youbot, ipr,
  ned, tinkerbots, kondo khrs, mantis, scout, tiago family, saeon, sphero
  bb8, surveyor, puma, sojourner, bioloiddog, firebird6, heron USV, atlas,
  jetbot, darwin-op and its dependents (robocup, humanoid_marathon,
  supervisor_set_position_loop test), `projects/humans/` tree.
- Prefs migration: `webots_*.qss` theme keys are read as `omnisim_*.qss`.
  Manually-edited theme overrides need re-saving once.

### Demos & worlds

- 14 per-URDF-robot OmniLink chat demos, each wired end-to-end to the
  OmniLink agent platform (g1-engine default, local fallback)
- OmniLinkStage PROTO adopted across all 14 omnilink_* worlds
- warehouse_foreman: iterations 0-4 — Picker, Loader, vision-driven tag ID,
  orchestrator runs end-to-end, side-detour push fix, ground-truth pallet
  delivery verified
- warehouse_patrol: iterations 0-3 — two-husky patrol squad, bridge port
  multiplexing
- drone_surveyor: iterations 0-3, verified end-to-end
- tour_guide: TIAGo four-room apartment, verified end-to-end
- Arm assembly line: three-arm cobot demo with Robotiq 3F-styled,
  parallel-jaw, and realistic hinged-finger grippers; ARM_BRIDGE_HOST/PORT
  env override; iter-1 vision + NL goals + recovery, end-to-end LLM verified
- Arm digital twin: shadow-mode bridge + reference demo
- husky_maze: drift-gate snap-to-cell smoother corridors, fix corners
  replan loop, unknown-lidar thrash, tokens-per-hour metric
- husky_fleet_arena: 10-husky open-arena variant
- husky rough-terrain hill: 840 HD rocks + heading-PID controller
- mars_max stress benchmark: 8 huskies + 160m world; renderer well within
  budget; regenerate with hills and valleys
- Husky combat: husky_hunt AI, 2-husky head-on demo, 4v4 head-on collision,
  match runner
- two-OmniBot combat arena demo + random walk demo
- CUDA M2: boundsHalfWidth PROTO field, Husky-meets-spheres demo, 10k
  broadphase showcase, two-way robot/particle coupling
- Cinematic capture service: agent-facing render service, deterministic
  stepping, /shutdown endpoint, drone shotlists, 4K static top-down shotlist
  for the husky fleet arena
- Canonical top-down Viewpoint across all 37 worlds

### Robots & PROTOs

- Native URDF support: WbUrdfImporter + URDFRobot world node, inertia +
  sensors gated, fallback colors, composite bounding, smarter joint limits
- PROTO → URDF migrations: Boston Dynamics Spot, Franka Panda, Mavic 2 Pro
  (with custom rotor physics), PR2, Husarion Rosbot + Rosbot XL,
  UR3e/UR5e/UR10e, TurtleBot3 Burger/Waffle/Waffle Pi
- Agent-first PROTO tooling: schemas, validation, authoring, hot-reload,
  tests
- Devices: replace PROTO catalog with xacro macros
- Spot URDF: walk progression — exploding-spawn fix → stable stance →
  supervisor-driven wave gait → IK foot-trajectory walk → closed-loop
  Raibert balance → pure-physics wide-stance walk → statically-stable
  crawl with CoM shift + roll feedback → ported CHAMP control algorithms
  to Python controller
- UR5e: IK keyboard teleop + OmniLink bridge supervisor

### CUDA / physics

- Newton physics backend (additive, ODE remains default): WbPhysicsBackend
  abstraction (P0) → concrete WbOdeBackend + WbNewtonBackend + registry
  (P1) → embed CPython, import warp + newton (P2) → FFI smoke at
  newton.ModelBuilder() (P3.0) → per-world simulation surface (P3.1) →
  smoke sphere registers and steps live (P3.2) → numerical sphere drop
  + land verify (P3.2.e) → rotation readback (P3.4) → sphere radius
  from boundingObject (P3.5) → WbBox bounding + tumbling-box verify
  (P3.6) → WbHingeJoint → Newton revolute, in-binary verified (P3.7.b)
  → motor FFI surface + controller drives wheel (P3.8) → WbCylinder +
  WbCapsule bounding (P3.9) → physicsBackend inheritance from ancestor
  Solid + URDFRobot pass-through, husky on Newton (P3.10–P3.10f) → 10
  huskies sustain 165 fps (P4) → 8-husky watchable demo + scaling-cliff
  investigation → 4v4 head-on collision demo (P5) → damage demo + perf
  instrumentation (P6) → XPBD as primary solver, MuJoCo CPU as fallback
- CUDA GranularGroup: ENU-aware kernel + bouncing scatter demo (M2),
  one-way Husky pushes spheres → uniform-grid broadphase (100x more
  particles, real-time) → two-way coupling (robot pushes balls AND balls
  push robot), bowling-balls + 10k showcase worlds
- CUDA plans: husky head-on (30+ fps target), CUDA rigid-body solver
  (replace ODE for 10-20 husky scenes), CUDA particle effects, CUDA
  compute infrastructure
- CUDA particles: damage_tracker pool client + numba.cuda particle pool
  prototype

### Agents (`omnilink-agents/`)

- omnilink-agents shared SDK + omnisim-runner launcher
- omnisim Python package + CLI + doctor; omnisim.dev and omnisim.damage
  lifted into the package
- Agent runtime observability layer (snapshots + unified event stream)
- omnisim live agent HUD docked alongside the text editor; user-controllable
  font size; bigger default
- Damage system phases 0-21 + repair: contact detection + impulse via
  supervisor (P1) → HP/state model + chassis-impact detection (P2) → HTTP
  query endpoints (P3) → visual damage markers (P4) → behavioral
  consequences via customData gate (P5) → debris bursts on broken
  transitions (P6) → per-part appearance darkening (P7) → cumulative
  impact decals (P8) → wheel detachment on broken transition (P9) →
  per-state mesh swap (P10, Tier A) → particle effects: smoke, sparks,
  fluid stains (P11) → generic damage trait via DamageProfile (P12) →
  agent SDK (P13) → procedural body deformation (P14a-c) → impact-localized
  mesh deformation (P15) → headless full-realism harness (P16a-e) →
  topology fracture: strain detection → island selection → fragment spawn
  + chassis hole → regression test (P17a-d) → repair mechanics: HP regen
  + mesh regen + heal-to-pristine (P18) → generic part detachment + realism
  polish (P19) → slab attribution + car-crash physics (P20+P21); spawn-drop
  suppression; defensive guards against breaking other harness-loaded worlds
- mission_captain + husky_maze: tokens-per-hour metric + COSTS docs
- Tour-guide, foreman, patrol, surveyor agents (see Demos)

### Authoring & harness

- Headless harness self-detects parallel-session collisions, prints
  agent-actionable guidance
- sim-instances: scope kill to our PIDs and add --auto-port
- scripts: scope webots-bin kills to spawned instances (don't touch the
  user's running Webots)
- omnisim CLI pins WEBOTS_HOME to this clone in webots_env()
- Drone surveyor, warehouse foreman, patrol multi-iteration scaffolds
- Per-instance log file + dynamic controller stdout buffer

### Build / packaging

- OmniSim Wire Protocol v1.0 — canonical PROTOCOL.md
- AGENTS.md as the canonical agent entry point at repo root
- Auto-purge orphan robot/asset dirs after pulls; clean_orphans wipes stale
  .o/.d build outputs
- Prune hollow controller dirs + stub Makefiles for Python controllers
- Cleanup: ~90 MB pruned from projects tree (upstream Webots dead weight)
- Cascade-delete darwin-op dependents + trim test source skip-lists
- glm submodule bumps (0af55cce, bf71a834)

### Release infrastructure

- Auto-create GitHub Release pages on publish + backfill helper for
  v1.0.0–v1.0.7
- Switch GitHub Release POST from curl to python urllib (Schannel TLS
  revocation workaround)
- ASCII-only output for backfill_release_pages.py (Windows cp1252)
- Pass `--root` to git diff-tree so orphan first-release dry-runs show file
  list; capture diff-tree output to temp file so the total-files line
  survives pipefail
- Repoint publish target at github.com/omnilink-tech/omnisim
- CHANGELOG.md + auto-generated release-note flow
- omnilink-reports/ rename (was omnilink-bugs/) and add OPERATIONS report
- SECURITY.md responsible-disclosure policy
- youtube_videos/ private marketing-scripts folder (deny-listed)
- ci: disable inherited Webots workflows until validated against OmniSim tree

### Documentation

- AGENTS.md: agent-first positioning, multi-instance support documented
- Beginner guide for the 14 OmniLink chat demos
- OmniSim Wire Protocol v1.0 reference (PROTOCOL.md)
- Long-range plans: Unreal-fidelity rendering, procedural world generation,
  CUDA Newton physics backend, CUDA husky head-on, CUDA rigid-body solver,
  CUDA particle effects, large-world optimization, granular physics
- guide/ + reference/ rebranded to OmniSim; archived inherited Webots
  changelog history (R2020–R2025) under upstream-webots-history/
- Damage system plans (phases 10-18) — mesh deform, particles, generic,
  SDK, procedural deformation, impact-localized deformation, headless
  full-realism dev, topology fracture, repair mechanics
- README: fresh-user onboarding (prereqs, submodule clone, verify, agentic
  setup); README section on capturing videos & screenshots; headless runner
  surfaced in README + quickstart
- URDF: import sensor-gate crash bisect + Python PATH guidance; record
  sensor-gate crash as upstream Webots bug
- benchmarks: outdoor_forest baseline + revised architecture finding;
  peak-RSS sampling per run; --repeats/median + chunky scenario
- OmniLink platform reference docs
- omnilink-agents DEMOS roadmap — 4 next demo agents, ranked + scoped

### Rendering

- Modern renderer backend via OMNISIM_RENDERER env selector
- WbRenderBackend seam, WREN scene iteration + mesh accessor C API
- OmniRender ForwardBackend scaffold; shared WREN viewpoint camera
- Atmospheric sky: Hillaire 2020 multi-scattering LUT, end-to-end procedural
  sky live on mars, procedural PBR irradiance + cubemap-free worlds, HDR
  cubemap for mars, per-pixel sky_apply matching preview HTML byte-for-byte,
  ENU→Y-up axis swap fix
- AgX tonemap, GPU-side auto-exposure, output dither, temporal smoothing
  (all under OMNISIM_RENDERER=modern)
- Mars perf: sun stencil shadows off by default (-51% forward GPU on
  mars_big), rock subdiv 3→2 (-38% mars_big / -46% mars), rock-template
  cache dedup, per-instance displaced mesh geometry
- Cross-world: shadows-off default (-49% on forest), bloom + GTAO defaults
  off (mars_big render 5.0 → 1.3 ms, -74%)
- T2 instrumentation: GPU timestamps for main scene, CPU companion timer
  + FullRenderNow, WREN-internal forward/post-process GPU breakdown,
  per-frame triangle counter, per-geometry draw histogram, forward-pass
  sub-bucket breakdown, aggregate per-viewport scene-render timing

### OmniWorld procedural worlds

- Scaffold + heightmap primitives (T1.1, T1.2)
- Scatter primitives (T1.3); nested scatter + surface manifests (T1.11)
- Layout DSL, solver, JSON schema (T1.4)
- Asset catalog (T1.5)
- Headless-simulator validator (T1.7)
- Biome cookbook (T1.8)
- Per-instance variation + weathering (T1.10)
- Biomes: outdoor_forest (first real biome), outdoor_desert, urban_block,
  warehouse, indoor_apartment, mars (with atmosphere overrides)

### GUI / UX

- WASD free-fly camera + FPS mouselook + numpad view snaps
- Splash screen: orb-centered OmniSim composition, drop Webots robot
  screenshots
- Live agent HUD (WbAgentHud) docked alongside text editor
- Web viewer rebranded to OmniSim, dropped dead Cyberbotics image links
- Streaming viewer vendors wrenjs/enum.js, serves wwi/ tree locally
- Auto-update notifier repointed to GitHub Releases for OmniSim
- mWebotsLogo + webots_icon.png renamed to OmniSim equivalents
- Branding polish: startup-update bug fixed, welcome/updated/About dialogs
  finished
- Docs URLs route Help-menu and node-help to GitHub-hosted OmniSim docs;
  load icons.svg + viewer.js locally instead of from cyberbotics.com
- CLI bug-report URL rewritten to OmniSim issues; --log-performance
  rebranded; metainfo URL typo fixed; Apache modification line; snap-MATLAB
  string cleaned
- Copyright backfill: OmniLink modification line on Cyberbotics-derived
  edits

### Fixed

- URDF importer: silence inertia matrix warning, fix TB3 motion bug,
  inertia gate, sensor emission restructure, TurtleBot3 waffle DAE crash,
  sensor gate progresses past world-load crash via carrier-Solid +
  replace-in-place
- spot.wbt: world description matches statically-stable wave-gait walk
- husky_random controller: strip broken sensor-based stuck-escape
- Newton: project joint anchors through parent/child rotations
- damage_system: align detached wheel mesh axis with collision cylinder;
  detached wheel uses real Husky wheel.dae mesh; IFS field + env-var
  disable hooks for stability
- Fix rotation pipeline: initialize Wren quats to identity
- prefs migration handles legacy webots_*.qss theme key

### Removed

- Pico TTS, Java, and MATLAB controller support (controllers, build
  scripts, samples, launcher strings)
- Integrated text editor (-3,190 LOC)
- Classic light theme (dark-only)
- i18n machinery
- Cyberbotics menu items + Webots.cloud upload feature
- Lua/Java/MATLAB overclaims in branding
- ~30 legacy robots without canonical URDFs (see Breaking changes above)
- `projects/humans/` tree (no URDF equivalents)
- Sample trees whose target robot was removed
- Mixed-dir dead worlds + scaffolding; residual refs to deleted
  robots/humans/worlds

---

## [v1.0.10] — 2026-05-08

### Demos & worlds

- cuda_particles + multi_robot_damage: P6 closed (fps gap remains)
- Arm assembly line: Robotiq 3F-styled grippers + TCP rotation tracking

### Robots & PROTOs

- damage_system: phase 19 — generic part detachment + realism polish
- damage_system: phases 20+21 -- slab attribution + car-crash physics

### Agents (`omnilink-agents/`)

- damage_system: phase 13 — agent SDK + drop accidental scratch PNGs
- Arm assembly line: parallel-jaw grippers + truststore patch + /reload
- Arm assembly line: realistic hinged-finger grippers, single-Robot host
- docs: rebrand pass — omnilink-agents + video script Webots → OmniSim

### Build / packaging

- omnisim.dev: lift dev CLI into the package, shim the script

### Release infrastructure

- release: record v1.0.9 published private SHA

### Documentation

- damage_system: plan v2 — Phases 10-13 (mesh deform, particles, generic, SDK)
- damage_system: plan — phase 14 (procedural body deformation)
- damage_system: plan — phase 15 (impact-localized mesh deformation)
- large_world: plan — CPU-side instancing, async load, scene streaming, broadphase audit
- damage_system: plan — phase 16 (headless full-realism development)
- damage_system: plan — phase 17 (topology fracture)
- damage_system: plan — phase 18 (repair mechanics)
- observability: agent runtime observability layer (snapshots + unified event stream)
- agents_md: document multi-instance support for fresh agent sessions
- omnilink-agents: shared SDK + omnisim-runner launcher
- cuda_particles: P1 -- bench + plan justifying GPU particle field
- cuda_particles: P1.5 -- python+numba.cuda particle pool prototype
- cuda_particles: P6 -- damage_tracker pool client (followup needed)
- Arm digital twin: shadow-mode bridge + reference demo
- deeper rebrand: GUI strings, CLI --help, AGENTS.md, dev plans
- rebrand pass: OPERATIONS.md + CHANGELOG + world-comments + matlab launcher
- harness: self-detect parallel-session collisions, print agent-actionable guidance
- fps: lite-damage env hook + pool optimisation + journey doc
- plan: cuda husky head-on -- 30+ fps target via binary-level work
- plan: cuda rigid-body solver -- replace ODE for 10-20 husky scenes
- plan: nvidia newton physics backend -- additive, can't break simulator

### Removed

- remove i18n machinery
- remove dead Cyberbotics menu items + Webots.cloud upload feature

---

## [v1.0.9] — 2026-05-04

### Demos & worlds

- damage_system: phase 1 — contact detection + impulse via supervisor

### Agents (`omnilink-agents/`)

- Arm assembly line: iter-1 — vision + NL goals + recovery, end-to-end LLM verified

### Release infrastructure

- release: record v1.0.8 published private SHA

### Documentation

- damage_system: phase 0 — arena world + box-dropper supervisor
- damage_system: phase 2 — HP/state model + chassis-impact detection
- damage_system: phase 3 — HTTP endpoints for damage queries
- damage_system: phase 4 — visual damage markers on state transitions
- damage_system: phase 5 — behavioral consequences via customData gate
- damage_system: phase 6 — debris bursts on broken transitions
- qa cleanup: rename OLink-agents → omnilink-agents in AGENTS.md, fix Linux build, document Linux setup

---

## [v1.0.8] — 2026-05-03

### Release infrastructure

- release: record v1.0.7 published private SHA
- release: switch GitHub Release POST from curl to python urllib

---

## [v1.0.7] — 2026-05-03

### Release infrastructure

- release: record v1.0.6 published private SHA
- release: ascii-only output for backfill_release_pages.py

### Documentation

- release: auto-create GitHub Release pages on publish + backfill helper

---

## [v1.0.6] — 2026-05-03

### Release infrastructure

- release: record v1.0.5 published private SHA

### Documentation

- capture: agent-facing cinematic render service — sister to the validation harness
- capture: playback_speed knob — high-level alternative to settle_steps_per_frame
- perf: per-instance log file + dynamic controller stdout buffer
- bench: cleanup + --repeats/median + chunky scenario; record item-4 second attempt
- Arm assembly line: three-arm cobot demo, iter-0 end-to-end verified
- docs: README section on capturing videos & screenshots
- bench: peak-RSS sampling per run; size item 3 + write session summary

---

## [v1.0.5] — 2026-05-02

### Agents (`omnilink-agents/`)

- warehouse_foreman: hint-first picker prompt — 56% faster, 86% cheaper
- warehouse_foreman docs: Cloud Run, not Vercel
- warehouse_foreman: perception-as-tool — $1.50/hr verified end-to-end
- warehouse_foreman: ship documentation — ARCHITECTURE.md + AGENT_PATTERNS.md
- warehouse_patrol: iteration 0 — world + folder scaffold
- warehouse_patrol: iterations 2 + 3 — Patrol Squad shipped end-to-end
- drone_surveyor: ship iter 0-3 + verify end-to-end
- local_memory: propagate _write_file mkdir fix to 5 sibling agents
- tour_guide: iteration 0 — TIAGo four-room apartment tour, end-to-end verified

### Authoring & harness

- warehouse_patrol: iteration 1 — bridge port multiplexing for two-husky worlds

### Release infrastructure

- release: record v1.0.4 published private SHA

---

## [v1.0.4] — 2026-05-02

### Highlights

- **A 6-DoF cobot arm.** First cobot-class arm in the stock URDF library, complete with per-link visual + collision meshes, a sample world, and a wave demo controller that exercises all six joints.
- **CUDA broadphase.** `GranularGroup`'s O(N²) brute-force collision test is replaced by a uniform-grid linked-list broadphase. Real-time particle ceiling moves from ~2 000 to well past 100 000 — the new `tests/cuda/warehouse_husky_granular_massive.wbt` showcase runs the larger budget end-to-end on commodity NVIDIA hardware.
- **Warehouse-logistics demo + agents.** A new `warehouse_logistics.wbt` scene driven by two co-located OmniLink agents: `warehouse_foreman` (supervisor scaffolding) and `warehouse_picker` (first end-to-end picking agent — profile, knowledge, memory, picker tool, chat-driven runner).

### Demos & worlds

- `projects/samples/demos/worlds/flagship/warehouse_logistics.wbt` — pallet / forklift logistics world driven by `warehouse_picker`.
- `tests/cuda/warehouse_husky_granular_massive.wbt` — 10 k-sphere CUDA broadphase showcase world.
- `tests/cuda/launch_warehouse_granular.bat` and `launch_warehouse_granular_massive.bat` — Windows launchers for the granular CUDA demos.

### Robots & PROTOs

- A 6-DOF cobot arm package: URDF, per-link visual (`.glb` / `.stl`) and convex-hull collision (`.obj`) meshes, a wave demo controller, a sample world, and a `webots.yaml` registry entry.
- `URDFRobot.staticBase` — new field on the URDF importer. When `TRUE`, the emitted `Robot` has its root `Physics` block stripped so OmniSim treats the base as a kinematic root. This is the bolted-to-the-floor semantics that arms like the UR5e and Panda need to keep their base from skating around under joint torque.

### CUDA / physics

- `WbGranularGroup` uniform-grid linked-list broadphase. Cell size is `2 · radius` so any contacting pair of centres lands in the same or an adjacent cell; the build pass uses `atomicExchange` chains so each cell ends up pointing at a chain of every particle inside it; the force pass walks the 27 neighbour cells per particle. Grid is reused across substeps.
- Refreshed `tests/cuda/bench_results.md` with the post-broadphase numbers.

### Agents (`omnilink-agents/`)

- `warehouse_foreman/` — supervisor agent scaffolding.
- `warehouse_picker/` — full picking agent: `profile.json`, README, knowledge tool, local-memory + recall tools, `picker` action tool, `scripts/chat_drive.py` runner, agent entry point.
- `DEMOS.md` — roadmap for the next four demo agents, ranked and scoped.

### Authoring & harness

- `husky_omnilink_bridge`: new `drive_to_waypoint` action exposed to the bridge protocol — the foundation the warehouse picker drives the Husky on top of.
- `husky_omnilink_bridge`: drift-gated cell-snap. The husky now snaps to the destination cell only when residual drift exceeds 0.40 m, so clean forward drives no longer pop visually after every cell while post-pivot drift still corrects. The threshold sits well under the 0.5 m wall-clearance buffer.

### Fixed

- `husky_maze`: corners-mode replan loop and unknown-world lidar thrash.

---

## [v1.0.3] — 2026-05-01

### Documentation

- README hero replaces the inline `<video>` embed with an animated GIF preview of the CUDA `GranularGroup` showcase. The full three-minute MP4 stays linked underneath. GitHub's README rewriter strips relative-source `<video>` tags, which made the original embed render as a broken icon on the public landing page.

---

## [v1.0.2] — 2026-05-01

### Highlights

- Three-minute CUDA `GranularGroup` showcase video added to the README hero — `docs/media/videos/cuda_showcase.mp4`.

### Authoring & harness

- `tests/cuda/.harness_granular_group_load.wbt` — minimal hot-reload test world for the granular-group load path through the validation harness.

### Documentation

- README polish around the CUDA hero block.

---

## [v1.0.1] — 2026-05-01

### Highlights

- README rewritten around an OmniSim demo screenshot gallery: warehouse Husky, industrial warehouse, the five Husky maze variants (`husky_maze`, `_unknown`, `_corners`, `_visual`, `_blind`), and the CUDA `GranularGroup` demo — all captured headlessly via the validation harness.
- `GranularGroup` PROTO fully wired into the scene tree: `resources/nodes/GranularGroup.wrl` declaration plus `WbGranularGroup.cpp/.hpp` machinery.

### Build / packaging

- Inherited Webots CI workflows moved to `.github/workflows.disabled/` until validated against the OmniSim tree. The release build job stays active on the public side.

---

## [v1.0.0] — 2026-05-01

First public release of OmniSim. OmniSim is a fork of
[Webots](https://github.com/cyberbotics/webots) repositioned around
agent-driven robotics simulation, distributed by OmniLink under the
Apache License 2.0.

### Highlights

- **Agent-first product surface.** [`AGENTS.md`](AGENTS.md) at the repo root is the canonical entry point for AI coding agents (Claude Code, Codex, Cursor) following the [agents.md open standard](https://agents.md/). It hands a fresh-clone agent everything it needs: build, launch, demo selection, headless-validation contract, HTTP-bridge driving, world iteration via the harness, and validation lanes.
- **Validation harness for agent-driven authoring.** Long-running HTTP service at [`scripts/harness/omnisim_harness.py`](scripts/harness/omnisim_harness.py) wraps a headless simulator subprocess and exposes endpoints for loading `.wbt` files, hot-reloading (~600 ms), screenshots, scene-tree inspection, viewpoint aiming (`/scene/look_at`), exposure stats (`/world/render_stats`), and stepping. Load failures come back as structured diagnostic codes (`PROTO_NOT_FOUND`, `WORLD_PARSE_SYNTAX_ERROR`, `TEXTURE_READ_FAILED`, …) so callers branch on codes rather than regex-matching stderr.
- **`omniworld` procedural world generator.** Recipes: `flat_ground`, `outdoor_forest`, `outdoor_desert`, `warehouse`, `urban_block`, `indoor_apartment`, `mars`. Same `(recipe, seed, params)` always produces a byte-identical `.wbt`. Backed by a Layout DSL + solver + JSON schema, asset catalog, scatter primitives, heightmap primitives, nested scatter + surface manifests, per-instance variation + weathering, and a headless validator.
- **Native URDF support.** New `WbUrdfImporter` plus a `URDFRobot` world-file syntax: drop a URDF + meshes into `projects/robots/<name>/`, reference it from a `.wbt`, and the simulator imports it on world load. Includes mesh loader (STL / Collada / glb), fallback colors, composite bounding, smarter joint limits, motor effort/velocity emission, position sensors, and supervisor flags. Stock URDF library covers UR5e, E-puck, Tiago, Husky, OmniBot.
- **HTTP bridges for runtime agents.** Sample bridges (`ur5e_omnilink_bridge`, `husky_omnilink_bridge`) expose robot control over a local HTTP API so an OmniLink agent can drive a robot model without writing controller code. Reference UR5e bridge listens on `127.0.0.1:6060` with `/state`, `/capabilities`, and `/action` (`set_joint_positions`, `set_tcp_target`, `solve_ik`, `reset_home`, …).
- **`omnilink-agents/` co-located agents.** Agent definitions versioned alongside the worlds and controllers they drive — same productized layout as OmniLink's first-party agents (`profile.json`, `prompts/`, `knowledge/`, `long_term_memory/`, auto-discovered `tools/`, runner). Reference agents at v1.0.0:
  - `husky_maze/` drives the Clearpath Husky across four maze worlds with progressively harder briefs (`husky_maze.wbt` trivial, `_unknown.wbt` lidar wall-follow, `_corners.wbt` mission-brief, `_visual.wbt` camera-only). Episodic memory of visited / unvisited cells, structured ops view, long-term memory for cross-session compounding.
  - `mission_captain/` provides cross-agent composability via local delegation; live-verified after credentials refresh, with resilience patches.
- **CUDA acceleration layer.** Two-tier additive infrastructure:
  - **CUDA M0** — context, buffer, and dispatch primitives gated behind `OMNISIM_CUDA_SMOKE=1`.
  - **CUDA M2** — `GranularGroup` PROTO with brute-force collision response (~320× CPU speedup at N=400 on the bench), real gravity-integration kernel via NVRTC + CUDA Driver API, Coulomb-capped tangential friction, ENU-aware kernel, WREN host-readback rendering so particles are visible end-to-end, two-way coupling between robots and granular media, `boundsHalfWidth` PROTO field.
- **Modern renderer baseline.** `WbRenderBackend` seam selectable via `OMNISIM_RENDERER`. Modern backend ships pass-through forward + post-process injection, AgX tonemap, GPU-side auto-exposure with temporal smoothing, output dither — all gated to opt-in until A/B-verified-better. Default backend stays WREN to preserve byte-identical visuals.
- **Realism + performance pass on outdoor worlds.** Per-instance displaced rocks, per-instance variation + weathering, log-uniform rock size distribution. Mars biome: fog, craters, ground-fit, multi-layer scatter, Husky fleet option, drivable-crater geometry, drop-height fix, motor-stall escape. `mars_big`: forward GPU 5.0 → 1.3 ms (-74%) via bloom / GTAO defaults off; rock subdivision 3 → 2 (-38%); shadows-off cross-world (-49% on forest, -51% on `mars_big`).
- **Branding.** OmniLink dot-sphere orb established as the canonical OmniSim mark. Splash screen, About box, GUI icons, color palette (black / cream / mimosa) all driven from a single source-of-truth tree under [`resources/branding/omnilink/`](resources/branding/omnilink/). Cyberbotics telemetry pings stripped; share-to-cloud dialog neutered; CLI bug-report URL points at the OmniSim issue tracker; `--log-performance` rebrand.

### Demos & worlds

Ships ~37 demo worlds with a canonical top-down `Viewpoint` across the
set. Featured worlds:

- `warehouse_husky.wbt` — onboarding demo. Husky random-walks a 26 × 12 m warehouse with reactive, position-based collision recovery.
- `warehouse_industrial.wbt` — pallet-rack columns, central conveyor, forklift, crates; harder collision scenario. Built end-to-end via the harness.
- `husky_maze.wbt` + `_unknown.wbt` + `_corners.wbt` + `_visual.wbt` + `_blind.wbt` — progressively harder mazes, including a vision-only world where pixels become tags via a four-cardinal-camera `husky_eye` sidecar.
- `desert_ruins.wbt` — abandoned-city-in-desert demo built via the harness; real 3D dune terrain, no image backgrounds.
- `husky_fleet_outdoor.wbt`, `mars.wbt` (generated), `swarm_control_showcase.wbt`, `mobile_robot_control_showcase.wbt`, `two_omnibots.wbt`, `ur5e_ik_test.wbt`, `hexapod.wbt`, `stewart_platform.wbt`, `soccer.wbt`.

### Build, launch & developer tooling

- `build_omni.bat` — Windows one-command build. Derives `WEBOTS_HOME` automatically. Wraps the MSYS2 / MinGW64 toolchain.
- `launch.bat` — Windows zero-args launcher. Opens the warehouse Husky demo by default; accepts any `.wbt` path and forwards extra simulator flags.
- `scripts/dev/omnisim_dev.py` — cross-platform helper. Subcommands: `build {core|renderer|gui|controller-libs|all}`, `run-world`, `run-headless` (the headless-validation contract: `--batch --mode=fast --no-rendering --minimize`, monitored log, structured exit codes), `harness`, `test-world`, `test-smoke`, `test-group`, `profile-world`.
- WASD free-fly camera + FPS mouselook, `F`-key follow toggle for the selected object, numpad view snaps in the 3D view.
- Structured runtime log file (`omnisim_log.txt`) at repo root.

### Documentation

- `AGENTS.md` — canonical agent entry point.
- `docs/developer/quickstart.md` — full local build / run walkthrough.
- `docs/developer/agent-map.md` — code-search and subsystem map for agents.
- `docs/developer/simulation-authoring-for-coding-agents.md` — best workflow for building new simulations.
- `docs/developer/omniworld-user-guide.md` and `omniworld-biome-cookbook.md` — procedural world generation.
- Inherited Webots `guide` / `reference` / `automobile` docs rebranded; dead Cyberbotics URLs scrubbed; chapter titles updated.

### Release infrastructure

- `scripts/release/publish_snapshot.sh` — one-commit-per-release publishing script. Operates inside a throwaway worktree under `.build_tmp/release-publish/` and never rewrites history. Defaults to dry-run; `--push` is the only way to actually publish.
- Apache 2.0 file headers brought into line with the OmniLink `copyright-headers.md` convention.
- `SECURITY.md` at repo root.

### Removed

- Cyberbotics-hosted telemetry pings.
- Share-to-cloud dialog wired up to dial out (UI surface kept; outbound network removed).
- Inherited Webots CI workflows moved to `.github/workflows.disabled/` until validated against the OmniSim tree.
- Hardcoded personal paths and machine-specific configuration removed from the tracked tree.
