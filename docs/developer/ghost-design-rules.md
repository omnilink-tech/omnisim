# Ghost Design Rules — the formalized reference-gait doctrine

> ## ⭐ 2026-07-09/10 — THE FOUR GATES. Read this before anything below.
>
> Everything under this banner still applies, but it was written when we believed a ghost's job was to
> *look* right. A ghost's job is to be **physically consistent with itself** — and then to be
> **realizable by the controller that will actually track it**. Four gates, in order; a
> ghost that fails gate *N* makes gate *N+1* meaningless.
>
> | | gate | what it asks | tool | repair |
> |---|---|---|---|---|
> | 1 | **CLOSURE** | does a planted contact stay still? | `ghost_dynamics.py` → `[CLOSURE]` | `ghost_close.py` |
> | 2 | **SUPPORT** | is the COM inside the support hull? | `ghost_dynamics.py` → `[COM]` | **solve** the base |
> | 3 | **FEASIBLE** | can the contacts supply the wrench? | `ghost_dynamics.py` → `[FWP]`/`[SUPPORT]` | slow it, or replan |
> | 4 | **PD-REALIZABLE** | can the deploy-grade PD plant *track* it, from its own state, under perturbation? | [`ghost_funnel.py`](../../projects/policies/training/ghost_funnel.py) → the funnel | retime (`ghost_topp.py`), declare feedforward (`ghost_ff.py`), or widen the corridor past the LAW's threshold |
>
> **Gates 1–3 are STATIC** (does a force distribution *exist*?). **Gate 4 is CLOSED-LOOP** (does the
> tracker *stay near* the reference?). They are different questions, and the difference is measurable:
> the 7 cm stair ghost passes 1–3 cleanly and still defeated seven training configurations. The
> gate-4 section below — **and the CORRIDOR-vs-TORQUE LAW, which retro-explains every corridored
> training we have run** — is the 2026-07-10 addition (`000699c0`).
>
> ### ⛔ THE BASE IS AN OUTPUT, NOT A DESIGN VARIABLE
> Every builder we wrote before this date sets the base by formula in the foot positions —
> `pelvis_x = 0.5*(xL+xR)`, `pelvis_z = min(zL,zR) + ride`, `pelvis_y = 0` — then fills the joints in
> with a closed-form leg IK. That **over-determines** the motion, and the residual comes out as
> levitation. Plan the *contacts*; solve everything else. `ghost_synth.py` does this.
>
> ### The evidence, one tool, three ghosts (external help the contacts cannot supply)
>
> | ghost | how its base was chosen | demand |
> |---|---|---|
> | `ghost_turn_90` | numerically solved so the COM stays in support | **0.0 N** |
> | `ghost_official_full_v3` (flagship walk) | recorded under the crane | ~21 N·m |
> | `ghost_stair_climb` (shipped) | formula | **62 N + 56 N·m** |
> | `ghost_stair_climb_synth` (`ghost_synth.py`) | planned contacts, solved base | **0.0 N + 3.2 N·m** on 4 of 2050 frames |
>
> ### The method generalizes (2026-07-09, one evening, one judge)
>
> Five more motions synthesized by the same recipe, every one passing all three gates in
> `ghost_dynamics.py` (closure ≈ 0 mm, COM inside the hull, FWP ≈ 0 infeasible, external help → 0):
>
> | motion | builder | first-of-its-kind extension | headline number |
> |---|---|---|---|
> | walk (cyclic) | `ghost_synth_walk.py` | cyclic seam closure (2e-11 rad) | **0.0 N + 0.0 N·m**, and the measured law: quasi-static self-support on the 6 cm sole caps at **vx ≈ 0.131 m/s** (CoP-lever bound; μ-insensitive) — the flagship's 0.45 m/s *was* the crane |
> | squat | `ghost_synth_squat.py` | base-z profile + arm counterweight | deepest pelvis **0.380 m** (ankle-stop limited); the solve *reproduced* the 2026-07-01 deterministic hip/knee/ankle ratios unprompted |
> | kneel (half) | `ghost_synth_kneel.py` | **contact-set switching** + **solved base pitch** (11.3°) + two-pass COM | knee patch on the shin's **+x face** (`axes` field in `ghost_contacts`); needs the crawl URDF's knee collider at train time |
> | push-up ×2 | `ghost_synth_pushup.py` | **hands+toes** contact set, **arm chains solved**, base pitch 77–107° | FWP 0/662, arms at 0.59 of their 25 N·m limit, wrench robustness 247 N |
> | stairs 3 cm | `ghost_synth.py --terrain riser=0.03` | riser curriculum start | first **live base motion ever from a climb-ghost tracker**: real steps onto tread 1–2 |
>
> ### The reference is necessary, NOT sufficient — the trainer's CLOCK is the other half
>
> Training the tracker on the (gate-passing) 7 cm climb ghost failed across five incentive schemes,
> each isolating one cause (runs `stairsynth2..8`, 2026-07-09): pure clock races ahead → high-gmatch
> shuffle at the base; `SEQ_LEASH_LEAD` (reference waits) → standing scores surv=1.0 because nothing
> pays for progress; `+W_PROG` (pay per leashed-phase bin) → still no first step; segment isolation
> (first step only, completion = success) → stands at gmatch 0.956 and never attempts it. On **3 cm**
> the same stack climbs (live: feet planted on tread 1, base +0.49 m). Verdict: at 7 cm the wall is
> **NOT reference physics** (FWP says torque 0.55).
>
> ⭐ **SHARPENED 2026-07-10 by gate 4 + the LAW (see below):** the "discrete step-up discovery" story
> was only half right. The 7 cm ghost is **1.000 PD-trackable** under the training crane once the
> feedforward is supplied (`ghost_funnel.py`), and the corridor those seven runs trained with (0.12
> rad) was **narrower than the stance-knee torque the reference itself demands** (0.233 rad at
> kp=200) — the policy could not hold its own reference inside its own corridor, **by construction**.
> The 3 cm "curriculum win" was really the corridor crossing that threshold (0.20 > 0.192). ⛔ This
> does **not** mean 7 cm is solved: the LAW explains the *failure*, but with the fix in hand no policy
> has yet climbed 7 cm live (the live riser refusal is a separate, still-open closed-loop wall).
> **Gate-pass ≠ climb.**
>
> A correct ghost also EXPOSES latent trainer traps that broken ghosts masked: RSI teleports at exact
> contact depth (+2 cm fix), reset joint noise `IC_RAND_JOINT=0.03` intersects flush feet with terrain
> (use ≤ 0.005 for closed ghosts). Both are commented at their fix sites in `g1_walk_recipe.py`.
>
> ### ⛔ "Recorded" is NOT a feasibility certificate
> Rule 1 below offers "recorded from a policy" as a provenance class that guarantees achievability. It
> does not. Our recordings come from policies running under the **crane harness**, so a recording proves
> only that the motion is feasible *with the crane*. The flagship walk ghost keeps its COM on the
> centreline (±15 mm) while its stance foot sits 80–100 mm to the side; no contact force distribution
> holds that. Measure the debt with `[SUPPORT]`; zero means free-standing.
>
> ### ⛔ Validate against the FORWARD KINEMATICS, never against the plan
> The stair builder's analytic IK places the **ankle joint** and assumes a constant 36 mm drop to the
> sole. The true drop ranges 10–64 mm, because the G1's contact patch sits 35 mm *ahead* of the ankle and
> swings on a lever. Its "planted" feet drifted 62 mm per stance; only 55% of its declared 0.35 m climb
> came from its legs; it was airborne on 638 of 1112 frames. It printed *"FK-feasible by construction"*
> the whole time — because it checked the analytic chain, not the FK.
>
> ### ⛔ `ghost_doctor.py`'s T1/T2/T3 are calibrated on PERIODIC WALK recordings
> Run them on a quasi-static stair climb and they report a 0.02 Hz "cadence", a 215 %-over hip swing, and
> a −832 mm static margin for a ghost whose measured COM margin is **+25.6 mm inside** the support hull.
> T0 (hard URDF limits) is the only part of that tool that generalizes. Same lesson as the RULER LAW:
> a metric is only valid on the distribution it was calibrated on.
>
> Detail and the four traps that produced convincing wrong numbers: the module headers of
> [`ghost_dynamics.py`](../../projects/policies/training/ghost_dynamics.py),
> [`ghost_close.py`](../../projects/policies/training/ghost_close.py) and
> [`ghost_synth.py`](../../projects/policies/training/ghost_synth.py).

**Status: canonical** (maintainer directive 2026-07-03: formalize ghost design before generalizing to
H1 / other humanoids / dancing ghosts, so infeasible references are rejected in seconds instead of
discovered after GPU-hours).

A **ghost** is the phase-indexed full-body reference (`leg_lut` + optional `arm_lut`/`att_lut`,
`nb` bins on the gait clock) that the flagship trainer
([`projects/policies/training/`](../../projects/policies/training/README.md)) corridors, rewards,
scores (WBMATCH), and displays (hologram) against.

> The ghost + this validation step are stage 1–2 of the **skill pipeline**
> ([skill-library.md](skill-library.md)). Once a ghost passes here, its skill's manifest records the
> verdict, and `projects/policies/skills/ghost_lut.py` gives the ghost a typed, repo-wide schema
> (`python skill_lib.py ghost <lut>` for the structural check; this validator for the physical one).

## The validator (run it BEFORE any training)

```bash
python projects/policies/training/ghost_validator.py <ghost.json> --baseline <the recording it derives from>
```

FAIL = do not train (redesign or re-record). WARN = train with eyes open. Calibrated against the
2026-07-03 campaign: it passes the ghosts that trained (official, v3c/WARN) and fails the ones
that collapsed (v3e/v3f/v3g) or were scratched on sight (v4) — from pure math, <1 s each.

---

## GATE 4 — PD-realizability (the funnel), and the CORRIDOR-vs-TORQUE LAW

*Shipped 2026-07-10 (`000699c0`), from the maintainer's directive to attack ghost design **numerically**:
many simulations, linear algebra on the results. The finding rewrites the diagnosis of every
corridored training run in the project's history.*

### Why gates 1–3 are not enough

Gates 1–3 are **static**: they ask whether a contact-force distribution *exists* that explains the
motion. They never ask whether the **closed-loop system — the reference tracked by the same PD the
deploy uses — stays near the reference under a small perturbation.** That is a *funnel* question,
and it is answerable by pure simulation, with no RL in the loop.

The measured proof that the distinction matters: the **7 cm** stair-climb synth ghost passes all
three static gates (closure 0.0 mm, COM in hull, FWP feasible at 0.55 of torque) and yet **seven
training configurations produced zero climbing**, while the **3 cm** ghost — same method, same
gates — climbed on the first try.

### The tool: [`ghost_funnel.py`](../../projects/policies/training/ghost_funnel.py)

> Roll the **real model** forward with a **deploy-grade joint PD** tracking the ghost's clock, from
> the ghost's own state at phase *b* plus a small perturbation, and measure how long the base stays
> inside a tube around the reference. Repeat over phases and samples.

- **Plant = the deploy plant**: `τ = KP·(q_ref − q) − KD·q̇`, clipped to the URDF effort limits, at
  4 × 0.004 s substeps per 16 ms tick (the trainer's DT-parity numbers; PLANT-ECHO kp=200, kd=30).
- **Judged in the trainer's own kill coordinates**: base z within `--tube` of the phase-advanced
  reference height, base xy within `--leash`, not fallen.
- **Feedforward** = the quasi-static actuator torque under the FWP's own force split — *gate 3's
  feasibility certificate, cashed as control.*
- `--crane` adds the training harness's lateral catch + attitude PD, i.e. measures the funnel the
  **trainer** actually sees.

Output is a **per-phase survival profile** — the funnel. A phase where the funnel collapses is a
place the reference demands closed-loop behaviour that plain tracking cannot supply; that is exactly
where an RL tracker stalls, because early training *is* approximately PD tracking plus noise. Scores
(crane + ff):

| ghost | funnel score |
|---|---|
| squat | **1.000** |
| stairs 3 cm | 0.942 |
| stairs 7 cm | **1.000** |
| walk | 0.590 (the seam bins are weakest) |

**THE 7 CM CLIMB IS PERFECTLY TRACKABLE BY PLAIN PD UNDER THE TRAINING CRANE.** It was never a
physics wall and never a control wall.

⚠️ **Necessary, not sufficient.** A wide funnel can still hide a reward-hacking optimum, and a
gate-4 PASS is **not** a live climb (the 7 cm riser refusal is still open). But a *collapsed* funnel
is a hard prediction of a stalled tracker, and it says **which bins** to fix.

⛔ **Three plant bugs, each now a comment at its fix site — reuse the lessons:**
1. **`implicitfast` is mandatory.** Raw-MuJoCo Euler rings the kd=30 velocity servo into a ±139 N·m
   bang-bang on the low-inertia ankle chain; the deploy engine integrates actuator damping implicitly.
2. **Disable self-collision.** MuJoCo-default self-collision blasts a crouch spawn apart.
3. **Joint-space PD alone cannot balance ANY upright motion.** Base rotation is invisible to joint
   error (λ = √(g/h) ≈ 3.7/s tips it from 1 mm in ~1.3 s). Realizability must be judged with
   base-state feedback present — here, the training harness itself.

### ⭐ THE CORRIDOR-vs-TORQUE LAW

> **A tracking corridor must exceed `τ_ff / kp`, or the reference is untrackable BY CONSTRUCTION.**

A Shadowing policy expresses torque **only** as position-target offsets through the PD plant
(`dq = τ / kp`), and the corridor (`GHOST_RESIDUAL`) **clamps those offsets**. So the quasi-static
torque the reference *itself* demands already spends corridor width before the policy has done
anything. Computed by [`ghost_ff.py`](../../projects/policies/training/ghost_ff.py) from the same LP
that certifies gate 3, at kp = 200:

| ghost | peak `dq` required (knee) | corridor trained with | outcome |
|---|---|---|---|
| stairs 7 cm | **0.233 rad** | 0.12 | never climbed (7 configs) |
| stairs 3 cm | **0.192 rad** | 0.20 | the ONLY run that climbed |
| walk | **0.226 rad** | 0.12 (historic) | chronically crane-dependent |

**One number retro-explains three campaigns**: the 7-run stair ladder; why the 3 cm "curriculum win"
was really *the corridor crossing the threshold* (0.20 > 0.192), not the riser; and why every walk
campaign leaned on the crane — the policy physically could not hold its own stance-knee torque inside
the corridor, and `HARNESS_FY`/`KP` made up the difference.

**Before you train a corridored run: compute `τ_ff/kp` for the reference and compare it to your
corridor.** If the corridor is narrower, the run is dead before it starts.

### GHOST-FF — the ghost declares its own feedforward

The fix is **not a wider corridor** (width *is* the mimicry guarantee). It is to shift the corridor
**centre** onto the feedforward:

```
q_cmd = q_ref + ffdq(phase) + a · GHOST_RESIDUAL          ffdq = τ_ff / kp
```

Gravity/stance compensation then costs the policy nothing, the corridor keeps its narrow meaning,
and rewards still score against `q_ref` (the **pose**, not the command).

- [`ghost_ff.py`](../../projects/policies/training/ghost_ff.py) computes `ffdq_lut` (`nb × 12`, leg
  joint order) and writes it into the lut. Emitted for the walk and both stair ghosts (LP-miss
  frames: **0** on all three).
- `GHOST_FF=1` in [`g1_walk_recipe.py`](../../projects/policies/training/g1_walk_recipe.py) loads it
  and shifts all four corridor-centre sites (rollout, eval, both `VC_REST` blends). **Opt-in,
  default off, zero per-tick cost.**

⛔ **Status, honestly:** the mechanism is clean-plant validated (7/7), and the trainer hook is
shipped and gated. **Trainer integration is still OPEN** — warm-starting a champion that already
learned to pay the torque itself makes it *double-compensate*, so the honest test is a fresh policy.
No policy has yet climbed the 7 cm riser live.

> ⚠️ **Correction (2026-07-13): "the fix is *not* a wider corridor" is only half true.**
> GHOST-FF is the right fix for the **centre**. But the **width** has an independent physical floor —
> see the corridor-adequacy law below — and the shipped corridors are *below* it. And the stated reason
> not to widen (mimicry) did not survive measurement: widening 0.12 → 0.24 on an identical ghost made
> the ghost match **better** (gmatch 0.931 → 0.946), not worse. Width is not the mimicry guarantee we
> thought it was; a *starved* corridor is what destroys mimicry.

### ⭐⭐ THE CORRIDOR-ADEQUACY LAW — the WIDTH has a floor, and it is computable

> **A tracking corridor must also exceed the robot's own ankle-CoP authority, `m·g·(sole/2) / kp`, or
> the policy cannot reach the edge of its own foot — and the crane pays the difference.**

The corridor-vs-torque law above is about the corridor's **centre**: it must carry the torque the
*reference* demands. This is about its **width**: it must carry the torque *balance* demands.

To balance on the ankle, a robot drives its centre of pressure toward the edge of the sole. The
furthest it can go is bounded by the sole itself, and reaching that bound costs

```
τ_cop = m · g · (sole_length / 2)          G1: 34.1 kg × 9.81 × 0.085 m = 28.5 N·m
dq_cop = τ_cop / kp                        G1 at kp = 200:  0.142 rad
```

A corridor narrower than `dq_cop` **structurally forbids the policy from commanding full ankle
authority.** Whatever it is denied, the harness supplies.

| corridor | % of `dq_cop` | measured |
|---|---|---|
| **0.100** — *every shipped G1 skill* | **70%** | crane-dependent; collapses at λ=0.2 (base z 0.07 m) |
| 0.120 | 84% | **43.6% of action-dims saturated**, crane lean 25%, wean stalled |
| 0.220 — stair champion | 155% | — |
| **0.240** — `fs3`, and `stairsynth17` | **169%** | **0.0% saturated**, crane lean 6.4%, gmatch **0.946** |

`stairsynth17` is the only run in this repo's history to pass the L1 legitimacy gate. It ran at 0.24.

**Why a narrow corridor makes things worse, not safer.** This is actuator windup. A clipped corrector
cannot fix an error while it is *small*, so the error grows, so the correction it needs grows, until
the correction it needs is one the corridor refuses. Measured: with **double** the room the policy took
**four times less** deviation (0.026 rad vs 0.097 rad). It does not want a *large* correction — it wants
a *timely* one. **The narrow corridor manufactures the deviation it appears to prevent.**

### ⛔⛔ BUT WIDTH IS INERT WITHOUT A REGIME WHERE FALLING IS POSSIBLE (2026-07-13)

> **A wider corridor gives the policy the ABILITY to carry itself. It gives it no REASON to.**

The corridor law was tested directly, and **the naive version of it FAILED.** Retraining the flagship
walk at 0.240 instead of 0.100 — at the shipped `HARNESS_LAM0=0.9` — did not reduce crane dependence.
It **collapsed**: after ~500 iterations the policy abandoned the reference (gmatch 0.93 → 0.65),
saturated the *wider* corridor anyway (74%), stopped walking (7.4 m → 0.8 m of progress) and in the end
walked *backwards* while hanging 57 N on the vertical bungee. A crane-reliance reward penalty
(`W_CRANE`) did not prevent it either.

**The reason is the regime, and it is the deeper finding.** At λ=0.9 with `HARNESS_KZ=2000`, the bungee
shoves the pelvis back up long before the fall threshold (`bz < 0.45`) can be reached. Measured across
**every** arm trained that way:

```
episodes ended during training:  0.   ZERO.  The robot NEVER FALLS.
```

So `FALL_PEN=50` — the largest single term in the reward — **never fires once**, `epret` prints `0.0`
forever (`done_sum/done_cnt` with `done_cnt = 0`), and the policy is never shown the consequence of
losing balance. In that world, "thrash and hang on the rope" is not a degenerate accident: **it is a
correct solution to the reward it was actually given.** Extra corridor width just buys more freedom to
find it. By contrast the weaning run `fs3` ended **363,142** episodes, and its first fall appeared at
it=620 — exactly when the crane graduated to λ=0.6.

⛔ **Every shipped G1 Shadowing skill sets `HARNESS_GRAD_SURV=2.0`** — the crane never weans, λ never
leaves 0.9. **They are trained in a world where balance is optional.**

**So the corridor law holds only where it can mean anything — on the weaning ladder.** The honest
evidence for it is:

| run | corridor | % of `dq_cop` | deepest λ held (surv ≥ 0.90) | crane lean |
|---|---|---|---|---|
| `fs2` | 0.120 | 84% | stalled ≈0.5, **43.6% saturated** | 25% |
| `fs3` | 0.240 | 169% | **0.100** | **0.2%** |
| `sW1` | 0.100 | 70% | collapsed at 0.200 (surv 0.261) | 16.3% |

⚠️ **That is n=1 per configuration**, against a failure mode that fires stochastically (identical
configs collapsed at different iterations, and one collapsed *with* the reward penalty). A replication
attempt on a 4-GPU cloud fleet was **discarded — its logs were corrupted** by duplicate training
processes writing the same files. **Treat the ordering above as a strong signal, not a proven law,
until it is replicated with per-seed isolation.**

**Enforced.** [`ghost_validator.py`](../../projects/policies/training/ghost_validator.py) gate
**`T3.corridor-adequacy`**: pass the corridor you intend to train with and it grades it *before* a GPU
hour is spent. Quadrupeds are `n/a` by construction (point feet have no ankle lever — balance is
footstep placement), and a robot whose sole cannot be measured **fails closed**.

```bash
python projects/policies/training/ghost_validator.py <lut.json> --corridor 0.100   # -> FAIL (70%)
python projects/policies/training/ghost_validator.py <lut.json> --corridor 0.240   # -> PASS (169%)
```

The law is `RR.corridor_min_rad(robot, kp)` in
[`robot_registry.py`](../../projects/policies/common/robot_registry.py) — derived from the robot's mass
and sole, so it needs **no training run to compute**.

⛔ **Do not read this as "the corridor was the only thing wrong."** Widening it is *necessary, not
sufficient*: the free-standing campaign still stalled at λ≈0.10, and a durable free-standing humanoid
walk remains **OPEN**. What the width fixes is the policy's *ability* to carry itself; whether it
*learns* to is a separate problem (see `W_CRANE` — a gate is not a gradient), and it cannot learn it at
all in a regime where falling is impossible (see above).

**The one-line takeaway, if you read nothing else here:**

> **Train with a crane that can be fallen out of, or you are not training balance at all.**

---

## The ghost toolchain (`projects/policies/training/`)

The design/validation tools, and what each one is *for*. Read a tool's module header before using
it — each carries the measured lesson that produced it.

| tool | what it does |
|---|---|
| [`ghost_synth.py`](../../projects/policies/training/ghost_synth.py) (+ `_walk` / `_squat` / `_kneel` / `_pushup`) | **the generator.** Plan the *contacts*, **solve** the base + joints. Passes gates 1–3 by construction. |
| [`ghost_dynamics.py`](../../projects/policies/training/ghost_dynamics.py) | **gates 1–3**: `[CLOSURE]` / `[COM]` / `[FWP]`. |
| [`ghost_close.py`](../../projects/policies/training/ghost_close.py) | gate-1 repair — pin a drifting planted contact. |
| [`ghost_validator.py`](../../projects/policies/training/ghost_validator.py) | the calibrated **7-rule** pre-training gate (run it on ANY new ghost). |
| [`ghost_funnel.py`](../../projects/policies/training/ghost_funnel.py) | ⭐ **GATE 4** — batched Monte-Carlo PD rollouts of the real model under the deploy-grade plant; returns a per-phase survival profile (the funnel) that predicts *where* a tracker will stall. `--crane` measures the funnel the trainer sees. |
| [`ghost_ff.py`](../../projects/policies/training/ghost_ff.py) | ⭐ computes the ghost's **own feedforward** `ffdq = τ_ff/kp` (`nb × 12`) from gate 3's LP and writes it into the lut; consumed by `GHOST_FF=1`, which shifts the **corridor centre**. This is the tool that measures the CORRIDOR-vs-TORQUE LAW. |
| [`ghost_topp.py`](../../projects/policies/training/ghost_topp.py) | ⭐ **time-parameterization: separates the PATH from its CLOCK** (TOPP / TOPP-RA, Pham & Pham 2018). Only the clock decides trackability — measured in a 3-arm controlled experiment: peak *lateral pelvis velocity* is the controlling variable (0.287 m/s → the policy over-spins and falls; 0.229 m/s → it survives); COM margin is **not**. Enforces per-segment velocity caps (body-frame lateral/forward pelvis velocity, yaw rate, per-joint URDF velocity limits) and re-samples back onto a uniform time grid. ⛔ Deliberately **minimal-stretch**: `dt_i = max(dt_original, dt_required)` — it never speeds a segment up (pure time-optimality would erase the settle frames), so it is strictly better than the old uniform `cycle_s` rescale, which stretched everywhere. Adding acceleration/torque bounds (the full forward-backward pass) is the next step. |
| [`ghost_contacts.py`](../../projects/policies/training/ghost_contacts.py) | ⭐ **the contact-set spec + the Contact-Wrench-Cone test** — the shared substrate under gates 3 and 4. Replaces the old hardcoded "two G1 ankles, flat ground at z=0, CoP on the z=0 plane" assumptions, which are silently *wrong* for stairs (treads at different heights), crawl (contacts are hands + knees) and carry (the payload isn't in the model). Drops CoP-in-support as the primitive and instead solves the general linear feasibility program: *do there exist contact forces, each inside its own friction cone and unilateral, whose resultant equals the demanded centroidal wrench?* (Caron et al. ICRA 2015; FWP = AWP ∩ CWC, Orsolino et al. RA-L 2018). Terrain types: `flat` \| `stairs` \| `ramp` \| `heightmap`. ⛔ Two things a contact spec MUST say and we used to get both wrong: **where the patch is** (the G1 sole is 35 mm *ahead* of and 36 mm *below* `*_ankle_roll_link` — using the body origin biases every COM/CoP number by 3.5 cm) and **what it rests on** (a scalar `surface_z` grades the wrong world for any stair/ramp ghost). ⛔ The CWC is a **check, never an objective** — maximizing a stability margin measured *worse than useless*. |
| [`ghost_root.py`](../../projects/policies/training/ghost_root.py) | ⭐ **recovers a ghost's base trajectory from its joints + contact schedule** — contact-aided kinematic odometry. Most luts (walk, crawl) carry no `root_lut`, and *inventing* one (a glide at `vx`) floats the robot so every contact reads "flight" and the validator lies or refuses. But the base is not missing information, it is **implied**: a planted contact is stationary in the world, so `p_wb(i+1) = p_wb(i) + R_wb(i)·p_bc(i) − R_wb(i+1)·p_bc(i+1)`, where `p_bc` comes from FK on the **joints alone**. Which contacts are planted is likewise derivable (lowest, within `swing_tol`) from attitude alone — no circularity. ⛔ Assumes **constant base yaw** (straight-line ghosts: walk, crawl) — turning ghosts already carry a `root_lut`; do not run this on them. |
| [`ghost_doctor.py`](../../projects/policies/training/ghost_doctor.py) | prescriptive T0–T3 classifier. ⛔ T1/T2/T3 are calibrated on **periodic walk recordings** — only T0 (hard URDF limits) generalizes. |
| [`ghost_polish.py`](../../projects/policies/training/ghost_polish.py) | symmetrize + harmonic-smooth + URDF-clip a folded recording. |
| [`ghost_screen.py`](../../projects/policies/training/ghost_screen.py) / [`ghost_balance_gate.py`](../../projects/policies/training/ghost_balance_gate.py) | choreography screening + the per-beat COM/support audit (sequence ghosts). |
| [`seq_ghost_retarget.py`](../../projects/policies/training/seq_ghost_retarget.py) | BVH → verified sequence lut (refuses to emit on gate failure). |

---

## The rules (each anchored to a measured outcome)

1. **Achievable by construction — two valid provenance classes.** The requirement is that the
   robot *can physically do the reference*; there are two ways to guarantee it:
   (a) **Recorded** — record a stable behavior in-engine → phase-fold → harmonic-smooth. This is
   the original class and the right one for *expressive* gaits (walk, dance) where you're copying
   a specific look.
   (b) **Solved (constructed-feasible)** — *compute* the trajectory by satisfying the robot's own
   physics: footstep plan + inverse kinematics against the robot model, with the **centre of mass
   kept over the support foot** (statically stable) and joint/velocity limits respected, checked
   numerically per frame. This is the right class for *functional* maneuvers that we have no
   stable in-engine recording of yet — e.g. **turning**. Proven 2026-07-07:
   [`build_step_turn_ghost.py`](../../projects/policies/training/build_step_turn_ghost.py) solves a
   90° step-turn (IK 0.3 mm, COM over the stance foot 54/54 single-support frames); it **shadows
   with ZERO NaN and deploys as real footwork (wtz=0)** — where five runs on an *infeasible* human
   mocap turn (LAFAN1, kinematic-only retarget) NaN'd at it≈55 every time. The failure was never
   "turns/sequences can't train" — it was reference infeasibility. ⛔ **The banned class is
   *eyeballed* hand-design** (typing joint angles by feel), which gets the dynamics subtly wrong.
   A trajectory *solved* to satisfy the feasibility equations is NOT eyeballed hand-design — it is
   achievable by construction, by a second, equally-valid route. Folds of *unstable* behavior
   remain invalid (the freearm v4 "panic attack" ghost — asymmetric arms 0.20, leg phase-mirror 1.2).
2. **The edit-envelope rule (dominant).** Parametric edits to a recorded gait (mean shifts, scales)
   are safe only within ~10–15% of the joint's own range. Measured: stance narrowed 40% and 25%
   collapsed training every time (falls 47–80%) — *including* with a physically-motivated sway
   compensation (v3g). Compensating a big edit on one channel does NOT rescue it. Beyond the
   envelope: change the *policy's world* instead (morph + free coupled channels) and **re-record**.
3. **Couplings are real, but re-recording beats hand-balancing them.** Known couplings: stance
   width ↔ sway amplitude; elbow extension ↔ arm-swing amplitude (straight arms ≈ 2× hand arc —
   the recorded bent-arm amplitude over-torques balance, measured falls at elbow ≥1.2 rad in 4/4
   runs); speed ↔ cadence ↔ stride. The validator flags budget violations, but the *fix* is rule 4.
4. **When a channel changes regime, FREE the coupled channels, then re-record.** Proven: elbows
   morphed to straight with shoulders corridored to the bent-arm swing → 4/4 collapses; same morph
   with shoulders FREE → survival 1.0, calmest gait of the project. The achieved behavior is then
   re-recorded as the new ghost (subject to rule 1's stability requirement).
5. **Morph, never snap.** Swapping a mastered control reference abruptly collapses a warm-started
   policy (bisection: elbow-snap surv 0.25, reference-snap 0.035). `GHOST_MORPH_JSON` +
   `MORPH_ITERS` interpolates references over training; validated to carry survival ~1.0 through
   full transformations.
6. **Ghost-first, maintainer-first.** Preview every new ghost solo (hologram world) and get sign-off
   BEFORE it touches training, metrics, or demos. The eye catches what the numbers miss (and
   vice versa — hence the validator AND the preview).
7. **Symmetry is a hard requirement for display ghosts.** L/R amplitude ratio ≥0.7, phase-mirror
   error ≤0.25. A stabilizing-but-asymmetric gait may be *functional* and still be an unusable
   reference. If free discovery yields asymmetry, symmetrize via soft REWARD (never corridor)
   and re-record. **Exception: a TURN is legitimately asymmetric** (one leg pivots) — the validator
   WARNs and that's expected; don't force symmetry on a turn/pivot ghost.

8. **The reference is only half of feasibility — the CONTACT MODEL is the other half.** A ghost can
   be perfectly achievable and still fail to *execute* because the physics under-models contact.
   Proven on turning (2026-07-07): MuJoCo foot contacts default to `condim=3` — slide friction only,
   **ZERO torsional (spin) resistance** — so a planted flat foot freewheels about its vertical axis
   and hip-yaw rotation never becomes base rotation (a step-turn lost ~2/3 of its yaw; the legs
   tracked the ghost, the base didn't rotate). `OMNISIM_FOOT_TORSION=<coef>` (condim=4 + real
   torsion on the feet, train + deploy) unblocks it. **When a shadow tracker tracks the joints but
   the ROOT doesn't move as the ghost's root_lut says, suspect the contact model (spin/roll
   friction), not the policy.** Full method + the torsion sweep: [step-turn-method.md](step-turn-method.md).

## Workflow for a NEW motion (H1 walk, dance, ...)

1. Obtain a stable source behavior in-engine (re-hosted official policy, trained champion, or a
   morph-trained variant).
2. Record full-body (`WALK_GAIT_LOG`), filter to the stable segment, fold at 256 bins, smooth
   (5 harmonics legs / 4 arms+attitude).
3. `ghost_validator.py` — must not FAIL.
4. Solo hologram preview → maintainer sign-off (iterate amplitudes/centers only inside the envelope).
5. Train via the flagship recipe (corridors + WBMATCH pinned to this ghost via
   `GHOST_METRIC_JSON`); reach the new gait by MORPH from the nearest mastered reference.
6. Live verify + side-by-side; keep the best checkpoint.

## Sequence ghosts (mocap-retargeted, spatial) — the 3-gate chain

For **non-periodic spatial sequences** (dance, expressive motion: time is the axis, root position
x/y/z/yaw is a variable via `root_lut`), use
[`projects/policies/training/seq_ghost_retarget.py`](../../projects/policies/training/seq_ghost_retarget.py)
(one command: BVH → verified lut; it REFUSES to emit on gate failure). The maintainer-mandated order
(2026-07-04):

1. **GATE 1 — fidelity vs the SOURCE**: full-skeleton FK of the mocap → robot-scaled foot-trajectory
   targets → **IK retarget** (DLS on the robot's own model), scored quantitatively: foot RMS < 6 cm
   (achieved: 1 mm), foot-height correlation > 0.85, ground-contact timing agreement > 85%.
   NEVER angle-copy between skeletons — conventions differ; the angle-copied draft was rejected
   on sight by the maintainer while measuring nothing.
2. **GATE 2 — grounding**: FK sweep of the assembled sequence — floating < 8%, penetration < 8%,
   root speed < 2 m/s (no glides/teleports).
3. **GATE 1.5 — DYNAMIC FEASIBILITY MAP (added 2026-07-04 after the Charleston campaign).**
   Gates 1–2 are KINEMATIC; they cannot see that a motion's momentum fabric is un-robotic. The
   map measures it: `SEQ_EVAL_MAP=1 EVAL_ONLY=1` spawns worlds at every routine bin in the ghost's
   pose+velocity and reports mean survival per routine-16th with the best available policy.
   Calibration: the raw retargeted Charleston read 0.36–0.76 s across its entire core (perpetually
   ~½ s from falling) after 6 training runs and 3 exonerated hypotheses (practice/RSI, entry/intro,
   tempo ×0.75, posture lift) — the reference itself was the wall, exactly the paper's thesis.
   **A raw mocap retarget is a STYLE TARGET, not a training reference.** The trainable ghost is
   built from it: blend toward the sequence's own calm skeleton (heavy time-smoothing) with
   per-bin weights driven by the map, restore the arm momentum budget, re-run gates — and after
   training, RE-RECORD the achieved dance as the true reference (rule 4 lineage).
4. **GATE 3 — the maintainer's visual verdict, ALWAYS LAST.** The hologram's sequence mode drives the
   full base pose from `root_lut`.

Note: several `ghost_validator.py` checks (leg symmetry, harmonic-jitter, phase-wrap) are
calibrated for CYCLIC gaits and mis-apply to sequences — a motion-class distinction is the known
next validator improvement. Pick recognizable sources: motion legibility survives retargeting
only when it lives in the channels mapped faithfully (the Charleston's footwork ✓; modern
ballet's expressive arms ✗ under the current shoulder-pitch-only arm mapping).

History and war stories: [g1-walk-recipe.md](g1-walk-recipe.md) (the 2026-07-03 sections).


## Addendum (2026-07-04, sequence campaign): dynamics rules

**Rule 8 — BALANCE-AUDIT BEFORE TRAINING.** Kinematic gates cannot see dynamic infeasibility.
Run `ghost_balance_gate.py <lut> --robot-spec <spec>` (GATE 1d): the robot's own COM (mass
model) vs its own support rectangles, per beat. A reference whose COM leaves the support for
whole beats will plateau no matter how long you train (measured: per-beat margins rank-match
per-segment training survival).

**Rule 9 — SCREEN CHOREOGRAPHY, DON'T FIGHT IT.** For any new motion, run `ghost_screen.py`
over the candidate pool first. Human motions differ enormously in robot-feasibility
(11-dance screen: 66 pts to 13 pts); picking the right source is cheaper than any alteration
stack. Never bend a gate to pass a motion — swap the source or re-plan the dynamics.

**Rule 10 — ISOLATE SEGMENTS TO LOCALIZE INFEASIBILITY.** `SEQ_BIN_LO/SEQ_BIN_HI` trains one
slice of a routine alone: a segment that stays weak in isolation is infeasible content (alter
the ghost THERE); all-segments-strong with a failing full routine means the transitions are
the problem (stitch progressively instead).

---

## The 2026-07-05/06 laws — the COMPLETE ghost design loop (maintainer-driven, all measured)

The walking-ghost campaign (v5 → v14, five maintainer corrections, ceiling 0.784 → 0.934) upgraded
the doctrine from "validate then train" to a full closed loop:

```
idealize target  →  ghost_doctor.py (gate + prescribe)  →  burst-prove (reward-side refs)
      ↑                                                            ↓
maintainer previews hologram  ←  ghost_polish.py (symmetrize+smooth+clip)  ←  REC_FOLD re-record
```

**Front door: [`ghost_doctor.py`](../../projects/policies/training/ghost_doctor.py)** — the
prescriptive classifier (maintainer design). Four tiers, each violation carries a computed fix:
T0 whole-body URDF limits at the TRUE cadence · T1 self-consistency laws calibrated from the
robot's own achieved library · T2 proven-envelope distance · T3 balance-gate RANK. Verdicts:
PASS / TARGET (burst-verify) / FAIL. Retro-validated: catches both hum70 killers in 30 s.

**Rule 8 — FOLD, never single-cycle.** A single recorded cycle preserves one world's balance
TREMOR as if it were the gait (unreplicable by definition). `EVAL_RECORD` + `REC_FOLD=1` folds
all steady ticks of all survivors per phase bin (~200 samples/bin): the systematic gait
survives, tremor cancels 1/√N. Then `ghost_polish.py`: bilateral symmetrization (mirror-sign
map: pitch +, roll/yaw −), light harmonic smoothing, URDF clip.

**Rule 9 — the corridor center is INVIOLABLE; references change on the reward side.** A
policy's competence = corridor center + learned offsets, inseparable (five failures, including
re-centering on the policy's OWN gait: surv 0.057 zero-training). Style changes ride
`REWARD_REFS_METRIC=1` (legs/arms imitation vs the metric ghost) with the control corridor
untouched. Amplitude changes that exceed reward's reach use **hop-and-settle**: morph a
proven-safe step, FREEZE the center at the exact tracked tables+cadence, let offsets re-adapt.

**Rule 10 — channels resist reward; corridors deliver (3-for-3).** Arms symmetry, swing
amplitude, elbow pose: flat under reward at any weight, instant under a corridor. The one
reward-responsive channel is base attitude — and only down to its physics floor.

**Rule 11 — self-consistency is multi-way and PER-ROBOT.** amplitude↔cadence↔speed
(vx ≈ k·swing·cadence, k measured from the robot's recordings — the doctor computes it);
sway↔stance (measured floor: 6.1° roll rock at the G1's stance — below it is structural:
wider stance or faster cadence); arm-momentum budget (amplitude × extension ≤ proven max;
⛔ G1 elbow convention: 0 = 90° bent carry, straight = +1.6 — bitten twice).

**Rule 12 — MEASURE-BEFORE-CLAIM.** Shape claims come only from the folded recording (scores
rise while gaits don't change; readouts inflate with transients and multi-cycle wander — three
false breakthroughs caught in one day). Mimicry exams run CLEAN physics (`MOTOR_RAND=0
OBS_NOISE=0`): DR leaks into evals by default and the ruler is one draw from a noised
population. The exam's self-match ceiling is MEASURABLE (score the source policy against its
own recording) — report scores against it.
