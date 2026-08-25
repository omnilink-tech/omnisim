# Shadowing — Learning Deployable Robot Motion by Shadowing Dynamically-Feasible Ghosts

> **Status (2026-07-03): VALIDATED + PROMOTED TO FLAGSHIP.** Shadowing is the canonical
> algorithm for legged-robot motion policies in OmniSim (maintainer directive). The shipped
> implementation lives at [`projects/policies/training/`](../../projects/policies/training/README.md);
> the formal design rules + the CALIBRATED Ghost Verifier (this paper's component 2, now
> concrete: `ghost_validator.py` — reproduces a 4-collapse/2-success training campaign from
> pure math in <1 s per ghost) are in [ghost-design-rules.md](ghost-design-rules.md).
> This doc remains the paper's working scaffold; the paper source is in
> [`shadowing_paper/`](shadowing_paper/).
>
> ## ⭐ SHADOW ITERATION — the operator has a STOPPING CONDITION, and it is FEASIBILITY (2026-07-13)
>
> Round 1 folds a champion's own achieved gait into a ghost and shadow-trains a better champion
> (+12.6% on the Go2). The obvious next question — **does it compound?** — was never asked.
> **It does not, and the reason is the most useful thing the method has told us about itself:**
> the round-2 ghost, folded from the *faster* shadow champion, **FAILS its feasibility gates**
> (closure p95 24.3 mm vs the 20 mm gate; support base_frac95 0.174 vs 0.0743). That champion is
> faster *because it rides its torque limit twice as often* — dynamics a kinematic, phase-indexed
> reference cannot represent. Fold it and a bare servo can no longer execute it.
> **The very thing that made the policy better made its gait un-ghostable.**
>
> And the gate **predicted it**: training on the failing ghost anyway (3 seeds) gives **0.170 m/s
> vs the control's 0.380 m/s** and 2.6× the drift — while a *single deploy rollout flatters it*
> (197 m, no fall, faster than the incumbent). Full write-up, ablation and reproduction:
> **[shadow-iteration.md](shadow-iteration.md)**.
>
> **Sibling paper — BATON** ([policy-switching.md](policy-switching.md)): the runtime *handover*
> between separately-trained Shadowing specialists (walk/stand/carry/turn). Its field-positioning
> analysis (2026-07-08) is settled — the switching-superiority claim is a **well-posed open gap**,
> not a demonstrated win (the field trends to one unified policy; the one head-to-head, LHM-Humanoid,
> used a naive oracle-FSM handoff and is sim-only). BATON's novel contribution is **recurrent
> hidden-state management at the switch boundary**; the edge is proven only by a pending
> success-vs-horizon experiment. See BATON's canonical "Where BATON stands vs the field" section.
>
> **Packaged as the SKILL LIBRARY** ([skill-library.md](skill-library.md) ·
> [`projects/policies/skills/`](../../projects/policies/skills/)): Shadowing produces the ghosts +
> champions, BATON composes them, and the skill library binds each skill's ghost + validator verdict
> + deploy env + checkpoint + provenance into one versioned manifest so skills are discoverable,
> runnable, and sequenceable across the simulator. `skill_lib.py` is the pipeline front door.
>
> **Terminology (settled 2026-06-21; extended 2026-07-03):** the method is **Shadowing**. The
> dynamically-feasible reference motion being followed is **the ghost**. The robot/policy
> **shadows the ghost**. Post-validation vocabulary: the tracker binds to the ghost via
> **corridors** (`ghost(phase) ± residual` — style structural, balance learned); match is
> scored by **WBMATCH** (legs/arms/attitude/speed vs the ghost the *eye* compares against);
> reference changes happen by **GHOST-MORPH** (interpolate the control references over
> training — snap swaps collapse a warm-started tracker); parametric ghost edits are governed
> by the **edit-envelope rule** (>10–15% of a joint's range ⇒ re-record, never hand-edit).
>
> **The thesis below is now empirically settled in-engine**: the 2026-07-03 campaign showed
> feasibility violations (edit-envelope, coupling budgets, asymmetry) predict training
> collapse 6/6, and the achievable-recorded ghost class trained to a live-verified durable
> walk (G1, WBMATCH 0.913). The remaining paper work is presentation, not proof-of-concept.

## Thesis (one paragraph)

The bottleneck in learning *deployable* whole-body robot motion is **not the RL** — it is
the **dynamic feasibility of the ghost** being imitated. If you ask a policy to shadow a
ghost the robot cannot physically execute, no amount of reward shaping, curriculum, or
compute will make it succeed; the policy parks in a degenerate local optimum. **Shadowing**
makes feasibility a first-class, *certified* property: (1) **generate** a dynamically-
feasible ghost from the robot's own dynamics; (2) **verify** its feasibility numerically,
before any learning; (3) **shadow** the certified ghost with RL, robust to the sim-to-deploy
gap. *Planning describes the problem; control learns to solve it.*

## Why this is non-obvious / the key empirical claim

**Ghost feasibility predicts learnability and deploy success — independent of RL tuning.**
We have a clean, controlled ablation already in hand (same robot, same RL algorithm, only
the ghost's feasibility differs):

| Motion | Ghost source | Feasible? | RL outcome |
|---|---|---|---|
| G1 walk | recorded achievable gait (foot-space model in the research era) | **dynamically feasible** | learnable **and** deployable — the in-engine Shadowing walk is live-verified durable (WBMATCH 0.913 vs the approved ghost). *(The older research-era path — standalone trainer → Newton deploy — managed only a finite bout, ~33.8 s / +5.9 m; that was a trainer↔deploy **durability** gap, never a ghost defect.)* **Do not restate a walk verdict from this doc — quote [rl-current-state.md](rl-current-state.md), which is canonical and always current.** |
| G1 sit-stand step | hand-drawn keyframes | only **quasi-static** | RL parks in stay-seated local optimum; falls in deploy |

Same learner, opposite result, because the ghost's feasibility differs. This isolates
feasibility as the causal factor.

## Contributions

1. **Shadowing architecture** — Ghost Generator → Ghost Verifier → Tracker; a general recipe
   to make any robot perform any motion in deploy (*planning describes, control solves*).
2. **A robot-agnostic feasible-ghost generator** — trajectory optimization over the robot's
   full dynamics/contacts/torque/balance; interface is strictly *(robot model, intent) →
   ghost*, nothing robot-specific.
3. **A ghost-feasibility certificate** — a numerical, RL-independent metric that predicts
   whether a ghost is shadowable, run *before* training (novel: a feasibility gate for
   imitation references).
4. **Empirical results** — feasibility predicts learnability + sim-to-deploy success;
   generality across motions and robots (OmniQuad walk **47.8 m / 0 falls**, Go2 walk, B2 walk, arm
   toss — all deployed in the Newton SIM). The G1 *biped* walk crosses the deploy gap: the
   **in-engine** Shadowing walk (train == deploy bit-exact; corridors + WBMATCH + GHOST-MORPH) is
   **live-verified durable** at WBMATCH 0.913 against the approved ghost, and composes into BATON
   sequences (box delivery, walk→turn→walk). The **older research-era** path (standalone trainer →
   Newton deploy) crossed only as a finite bout (~33.8 s, +5.9 m) — a trainer↔deploy **durability**
   gap, *not* a ghost-feasibility defect. ⛔ Retired figures ("G1 walk 200 m+ / zero falls") are
   trainer numbers that do **not** reproduce in deploy — never quote them.
   **Canonical, always-current status: [rl-current-state.md](rl-current-state.md)** — cite it rather
   than this paragraph. The running case study (G1 sit-stand): an infeasible ghost is *unlearnable*;
   the generated feasible ghost makes the *body* of the motion shadowable (tracks to 2.5 cm).
5. **A delineated boundary (headline negative result)** — a feasible reference + tracking is
   **necessary but not sufficient for an unstable, contact-rich *initiation*.** We show this is
   not an RL-tuning artifact: the dead-seated sit-stand *launch* resists **both** reward-RL
   (21 PPO-residual runs) **and** MPC-distillation/DAgger — both stall at the same ~0.60
   deepest-stable-crouch. Completing such a launch needs **predictive (lookahead) commitment**
   to a temporarily-unstable extension, which a purely *reactive* learned policy lacks. This
   sharpens exactly where the "planning describes, control solves" split holds and where it
   does not.

## Scope — when Shadowing is the right tool (and when it is NOT)

> **⚠️ Course-correction (2026-06-25): Shadowing is the wrong tool for *continuous
> dynamic-balance locomotion* (walking).** Shadowing hard-tracks a hand-designed **kinematic**
> ghost with a small bounded RL residual. For a walk that is backwards: a kinematic ghost is *not*
> a balance solution (balance lives in **reactive foot placement + push-off timing**, absent from a
> joint-angle curve), and the bounded residual (res_scale 0.3) has too little authority *and* fights
> the ghost — so it produces the natural gait but **cannot stabilise** (H1 falls ~2 s; no tuning
> fixes it). The field's natural-AND-stable answer inverts the priority: **full control authority +
> stability primary (task reward + early fall termination) + naturalness as a SOFT reward** (pure RL
> + reward shaping / DeepMimic soft imitation / AMP). **For walking, use that, not Shadowing**; if a
> reference is used at all, demote the ghost to a soft style term. **Shadowing IS still right when
> the reference is dynamically feasible and the task is NOT a continuous-balance problem** — get-up /
> rise (B2, G1), reaching, sit-to-stand, toss-to-place, replaying a recorded motion — where the
> ghost is a genuine plan the policy can track and the bounded residual suffices. Full reasoning +
> the H1 evidence: [locomotion-shadowing-vs-pure-rl.md](locomotion-shadowing-vs-pure-rl.md);
> canonical status: [rl-current-state.md](rl-current-state.md).

## Method

### Component 1 — Ghost Generator  *(build first)*
**Input:** robot model (URDF/MJCF) + intent (keyframes / task cost / contact schedule).
**Output:** a dynamically-feasible ghost = time-indexed `{q[t], base_pose[t], contacts[t],
torques[t]}`.
**How (as built):** **receding-horizon predictive-sampling MPC (MPPI)** over the full MuJoCo
dynamics, with the control space = the robot's *position-target actuators* (the same interface
deploy uses) → feasible w.r.t. the actual actuators/torque-limits/contacts **by construction**.
This is the standard MuJoCo-MPC formulation and it **works** — it discovered the feasible G1
sit→lean→rise→step→stand that hand-drawing + 14 RL runs could not. *Note:* the earlier
*open-loop, whole-trajectory* predictive sampling (optimize all knots at once) went chaotic at
stiff-contact knife-edges; **receding-horizon** re-planning (execute first action, shift) is
what fixed it. Constraints handled via the per-step cost: joint/base tracking, upright, balance
(CoM over the foot-support centroid), effort, velocity limits, fell-over. Stays robot-agnostic
by depending only on the model + an `Intent`. (Gradient-based contact-implicit TO —
Crocoddyl/OCS2 — is a drop-in alternative for the same interface.)
*If the optimizer cannot satisfy the constraints, the intent itself is unachievable — relax
it (this is the achievable-ghost rule, now enforced automatically).*

### Component 2 — Ghost Verifier  *(build second)*
**Purpose:** certify feasibility **numerically and independently of the RL**, so a training
failure is never ambiguous. TO self-certifies against *its own model*; the verifier catches
model ≠ sim ≠ deploy gaps by re-deriving the ghost's required forces in the target physics.

**The certificate (the per-step feasibility LP).** The core question, asked at every
time-step of the ghost, is a *one-step inverse-dynamics feasibility* test:

> Given the ghost's state and its finite-differenced acceleration, do there exist
> **friction-cone contact forces** and **within-limit joint torques** that explain
> `M(q)q̈ + bias` under rigid-body dynamics?

Splitting the equations of motion into the 6 unactuated **base** rows and the actuated
joint rows:
- the **base rows (0:6)** must be met by *contacts alone* — this is exactly the
  Zero-Moment-Point / centroidal-feasibility condition (Vukobratović & Borovac 2004;
  Orin, Goswami & Lee 2013): the ground reaction must produce the base's linear and
  angular momentum rate;
- the **actuated rows** define the required `τ`, gated by `|τ_j| ≤ τ_limit,j` (the
  torque-box constraint of trajectory optimization, cf. Tassa, Erez & Todorov 2012);
- contacts obey a **friction pyramid** (linearized Coulomb cone, `f_z ≥ 0`,
  `|f_x|,|f_y| ≤ μ f_z` — Stewart & Trinkle 1996), the standard LCP/LP contact model.

Per step we solve a small LP: minimize the base-wrench slack subject to the friction
pyramid and the torque box. A bounded slack (≈ 0) means the ghost's accelerations are
*physically supportable* at that instant; a large residual (≈ `mg`) is the unmistakable
signature of an unsupported / levitating body. This is the same LP at the heart of the
committed `ghost_verifier_dyn.verify_legged`.

**The hardened, motion-agnostic certificate (2026-06-23).** The original LP front-end only
worked for *flat-ground walking*: it looked at name-matched `foot`/`ankle_roll` geoms and
gated on absolute *center* height (`geom_xpos.z > contact_z`). That under-reports feasibility
on every contact-rich motion outside walking — it false-FAILed genuinely-deployed get-ups,
ballistic jumps, hill walks, and chair-supported sit-stands. A new additive module,
[`projects/policies/research/shadowing/feasibility_certificate.py`](../../projects/policies/research/shadowing/feasibility_certificate.py)
(`certify(ghost, mjcf, motion='auto', **opts)`), keeps the LP core *in spirit* and reproduces
the flat-walk verdicts *byte-for-byte*, while replacing the contact front-end and the verdict
logic. The committed `ghost_verifier*.py` and `ghosts/*` are left untouched. Four changes:

1. **Robust support-contact reconstruction** (`support_contacts`). Instead of named feet at a
   hard center-height, it unions three layered sources and feeds the resulting world points to
   `mj_jac`: (a) **all** robot collision geoms by *size-aware bottom-surface* proximity to a
   terrain-aware local ground (sphere/capsule/box/mesh lowest-point math, contact point z-clamped
   to the local ground) — this finds belly/knee/elbow support in a get-up and OmniQuad's foot spheres
   (OmniQuad has no foot-*named* body; a kinematic-leaf-body fallback selects geoms 4/7/10/13); (b) a
   **named/leaf foot fast-path** used only for `motion='walk'` to preserve the byte-stable walk
   behaviour; (c) **external world-fixed support surfaces** (`chair_seat`/`chair_back`, already in
   the sit MJCF, just never counted). Temporal **±2-step hysteresis** fills single-frame contact
   dropouts.
2. **Flight-aware base feasibility** (ballistic vs levitation). A no-contact step is no longer an
   automatic violation. We classify it from the *non-gravitational* base wrench `‖rhs[:6]‖`
   (gravity is already in `bias`): **free fall** (`‖rhs[:6]‖ ≤ flight_tol·mg`, gravity + inertia
   alone explain the base motion — `flight_tol = 0.10`) is FEASIBLE; **levitation** (≈ `mg`
   required against nothing) is INFEASIBLE. The old `no_contact ≤ 5%` hard gate becomes a
   total-flight-fraction sanity cap (≤ 60%), so a true ballistic jump certifies but an
   airborne-the-whole-time hover does not. This is the centroidal free-flight condition
   (Orin, Goswami & Lee 2013): in flight the only external wrench is gravity.
3. **Terrain-aware ground** (`ground_h(x,y)` callable). Flat `z=0` by default (byte-identical to
   the committed walk path); for hills the terrain is injected from the ghost's stored `hill_*`
   params via `HillProfile` and the **dynamics are computed in the terrain MJCF**, not the flat
   planner (the flat model reads feet as off-ground → false 101%-mg FAIL).
4. **A literature margin layer** computed from the LP primal and *reported* in `metrics`:
   - **ZMP / CoP edge distance** `d_zmp` — signed distance of the centre of pressure to the
     support-polygon edge (Vukobratović & Borovac 2004);
   - **required friction** `req_mu` and friction slack — the minimum coefficient the motion
     demands vs the available cone (the grasp/contact-wrench-cone view, Ferrari & Canny 1992;
     Orin et al. 2013);
   - **torque headroom** `min_j (τ_limit,j − |τ_j|)/τ_limit,j` and the binding joint
     (box-DDP margin, Tassa et al. 2012);
   - **capturability** `ξ = x_com + ẋ_com/√(g/z_com)` vs the support polygon (Pratt et al. 2006;
     Koolen et al. 2012) — **reported but deliberately NOT gated**. Capture is the documented
     *necessary-not-sufficient* signal: gating it would flip the byte-compatible (and feasible)
     G1 walk ghost to FAIL, so it is surfaced as a margin, not a verdict.

**Output:** a pass/fail gate + a single calibrated scalar **feasibility score** in `[0,1]`
(`score = min(s_base, s_tau, s_contact)`, the binding-constraint shape inherited from
`verify_legged`, so walk scores reproduce exactly) plus the full margin vector. Only certified
ghosts go to RL. **Motion auto-detection** resolves `arm` (fixed-base → delegate to the
committed `verify_arm`, unchanged) / `hill` / `sit` / `getup` / `jump` / `walk` from the MJCF
joints and ghost keys; all per-motion thresholds are overridable via `opts`.

*Honest caveats (from adversarial audit, see [shadowing-verification.md](shadowing-verification.md)):*
the **binary verdict** is sound across the repertoire — no impossible motion (levitation,
beyond-reach toss, frozen-apex jump, wrong-model) was made to PASS, and every deployed motion
PASSes. But the **scalar score is only weakly informative** (`s_tau` pins to 0 whenever a single
DOF rides its torque limit, so several genuinely-feasible ghosts score ≈ 0 yet PASS — the same
artifact as the committed G1-walk row). And the layered margins (`req_mu`, `fric_slack`,
`tau_headroom`) are read from a **non-unique LP primal** (the objective constrains only the base
slack, leaving contact forces free), so they are directional indicators, not bounding guarantees.
An early draft of the get-up branch was **exploitable** — its transition mask dropped *every*
judged frame (a get-up is entirely partial-stance), so it PASSed on an empty stance array, and an
8 Hz base shake / 3 m/s drift grafted onto real get-up joints still PASSed. **Fixed** (adversarial
review): the mask now excludes only a tight window around contact-count *switches* + the
takeoff/landing impulse, and an **abstain guard** returns INDETERMINATE (never PASS) when too few
support frames are judged — both regression-guarded in the self-test (`omniquad getup+shake → INDET`,
`+drift → FAIL`). **Trust the verdict; treat the scalar and the friction/torque margins as soft
diagnostics.** An *optional* model-based tracking-drift replay (`opts['track']`) re-simulates the
ghost under inverse-dynamics + PD for the DeepMimic-style drift check (Peng et al. 2018).

### Component 3 — Tracker (the shadower)  *(build last; largely already exists)*
RL policy shadows the certified ghost under domain randomization; crosses sim-to-deploy
(see [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md)). Reward = match-the-ghost +
**milestone/progress** terms (dense kinematic tracking alone admits degenerate optima) +
alive − penalties. **Hierarchical composition (funnels, Burridge–Tedrake):** decompose a
long motion into phase-ghosts that each land in the *basin of a solved primitive* — e.g.
sit-stand = `stand-up` (transition) → `g1_stand` (terminal balance, already solved) →
`sit-down`; the transition policy only has to *reach the basin*, so it can be imprecise.

## Experiments

- **E1 — Feasibility is causal (the spine):** same robot + RL, feasible vs infeasible ghost
  → success vs failure (walk vs hand-drawn sit-stand). Already essentially in hand.
- **E2 — Feasibility predicts learnability:** correlate the verifier's feasibility score
  with RL success / deploy robustness across many ghosts. The headline result. *(ran — see
  "Certificate — verified across the motion repertoire" below; finding: the certificate
  partitions failures by root cause — `cert FAIL` ⇒ fix the reference, `cert PASS + deploy FAIL`
  ⇒ fix the tracker — rather than yielding a learnability curve. n is too small for a regression.)*
- **E3 — Generality:** one tracker, many generated ghosts → G1 {walk, sit-stand, reach-
  while-standing}; second robot (OmniQuad). Robot-/motion-agnostic generator + tracker. **Non-legged
  + manipulation datapoint:** an *arm toss-to-place* — a fixed-base 6-DOF arm throws a
  part into a bin *beyond its reach* (impossible for carry/IK). Shows the pipeline spans a fixed-base
  manipulator — AND the honest limit: on a fully-actuated arm the generator degenerates to designed-
  playback and the tracker to classical control, so the *feasibility certificate* is what earns its
  keep. The full architecture is only indispensable when the motion needs an optimizer to discover
  AND can't be tracked open-loop (the underactuated / contact-rich cases — e.g. a throw-and-catch).
- **E4 — Sim-to-deploy:** deploy success/robustness of policies shadowing certified ghosts.
  **Quadrupeds crossed the gap** (OmniQuad walk 47.8 m / 0 falls, Go2/B2 walk, arm toss — verified in
  the Newton SIM deploy). The **G1 biped walk crosses it too, once the trainer *is* the deploy**:
  the in-engine Shadowing recipe (`run_walk_rl.sh` → `g1_walk_recipe.py`, train == deploy bit-exact)
  yields a live-verified durable walk at WBMATCH 0.913. The **research-era** trainer→deploy path
  crossed only as a finite bout (~33.8 s, +5.9 m, then topples) — the E2 `cert PASS + deploy FAIL`
  partition localized that correctly to the tracker/pipeline, *not* the ghost, and closing the
  pipeline gap (one engine, one physics spec) is what fixed it. ⛔ The retired "G1 walk 200 m+,
  zero falls" figures do **not** reproduce in deploy. Canonical status:
  [rl-current-state.md](rl-current-state.md).
  - **H1 walk (2026-06-24) — the tempting "just train *in* the deploy solver" fix was tried
    and it REGRESSED the deploy.** A Phase-2 trainer (`gpu_newton_h1_walk_trainer.py`,
    `cf200cdc`) fine-tunes the champion through the exact deploy solver
    (`newton.solvers.SolverMuJoCo`). Versus the mjwarp-trained champion (run 3: deploy
    **2.03 s / +1.45 m forward**), the in-solver fine-tune deployed **worse** — 1.58 s/back on
    a fresh-URDF model, 0.66 s/back on the matched dumped-MJCF model. Two lessons reinforce
    the `cert PASS + deploy FAIL` partition (the ghost is fine; the gap is the tracker/launch):
    (1) **"same solver ≠ matched physics"** — a fresh `add_urdf` build uses newton's *default*
    friction, not the deploy's `mu=2.0`; load the dumped MJCF via `newton add_mjcf` (`da8b171a`);
    (2) even matched model **+** matched solver isn't enough — run 3 and the fine-tune are
    byte-identical in the batched trainer yet diverge in deploy. The binding gap is the **launch
    initial condition** (the deploy's ~0.3 s settle lean + residual velocity, absent in the
    batched reset) and the **observation pipeline** (world-frame `getVelocity` + finite-diff `qd`
    vs the trainer's exact MuJoCo-frame `qvel`). Full writeup:
    [h1-walk-rl-journey.md](h1-walk-rl-journey.md); canonical status:
    [rl-current-state.md](rl-current-state.md).
- **E5 — The boundary (necessary-but-not-sufficient):** the G1 sit-stand *launch* — feasible
  ghost in hand, yet **neither** reward-RL (21 PPO-residual runs) **nor** MPC-distillation/DAgger
  learns the dead-seated push-off; both stall at the ~0.60 deepest-stable-crouch. The negative
  control that delineates the method (Contribution 5).
- **Running case study:** G1 sit-stand — the infeasible→feasible ghost fix turns an *unlearnable*
  task into one whose **body shadows to 2.5 cm**, isolating the residual hard part to the unstable
  contact-rich *initiation* (E5). Honest scope: the upright sit→stand is **not** yet deployed
  end-to-end from dead-seated; the launch is the open problem.

## Certificate — verified across the motion repertoire (2026-06-23)

The hardened certificate was exercised on every regenerable ghost in the library under **real
MuJoCo 3.8.1**, plus a battery of adversarial negative controls, and audited for numerics. The
self-test (`python projects/policies/research/shadowing/feasibility_certificate.py`) is a self-checking artifact
that exits non-zero on any verdict mismatch; it reports **ALL CORRECT**.

**Repertoire (deployed and reference motions — every row is the physically-correct verdict):**

| ghost | mjcf | motion | verdict | score | key margin |
|---|---|---|---|---|---|
| go2 walk | go2_planner | walk | **PASS** | 0.673 | base p95 1.7% mg |
| b2 walk | b2_planner | walk | **PASS** | 0.472 | base p95 3.1% mg |
| g1 walk | g1_full_kp100 | walk | **PASS** | 0.000 | base p95 4.6% mg (score 0 = `s_tau` pins) |
| omniquad getup | omniquad_newton_fixed2 | getup | **PASS** | 0.05 | judged on reconstructed leg support (93 stance frames) |
| b2 getup | b2_planner | getup | **PASS** | 0.40 | reconstructed leg support; b2 has no trunk collider (model gap) |
| omniquad jump | omniquad_newton_fixed2 | jump | **FAIL → boundary** | 0.00 | flight clean (8.6% mg) but the kinematic ghost's explosive *takeoff* is not contact-consistent (33% mg loading) — flagged, not rubber-stamped |
| b2 hill6 | b2_planner (+terrain) | hill | **PASS** | 0.06 | base p95 5.8% mg, 1317/1317 stance |
| omniquad hill8 | omniquad_newton_fixed2 (+terrain) | hill | **PASS** | 0.05 | base p95 0% mg, 0 bad-nocontact |
| g1 sitstand | g1_sit_kp100 | sit | **PASS** | 0.501 | seated mean ~1% mg w/ chair contacts |
| arm toss | 6-DOF cobot arm (fixed-base) | arm | **PASS** | 0.079 | delegated to `verify_arm` |

**Negative controls (must FAIL — proving the loosened contact reconstruction does not
rubber-stamp):** all FAIL with the levitation signature (`base ≈ mg`, hundreds of
no-contact-non-flight frames):

| control | verdict | signature |
|---|---|---|
| go2 levitate twin | **FAIL** 0.000 | base ≈ 101% mg phantom body-weight |
| arm toss_far (beyond reach) | **FAIL** 0.079 | (delegated; out-of-workspace) |
| frozen-apex jump (held at apex) | **FAIL** 0.000 | 76 bad-nocontact, ≈ mg required against nothing |
| levitating hill (base +0.5 m over terrain) | **FAIL** 0.000 | feet peel off local surface, ~all frames bad |
| levitating get-up (body +0.8 m over feet) | **FAIL** 0.000 | 196–346 bad-nocontact |
| atlas levitate twin | **FAIL** 0.000 | ~100% mg (a robot the cert was *not* tuned on) |
| atlas raw-qpos0 "stand" (CoM outside feet) | **FAIL** | 13.5% mg base residual — honestly un-standable pose |

**Robot-agnostic generality.** A robot never tuned on certifies correctly through the same
free-joint-detect + leaf-foot + per-step-LP path. Atlas certifies
correctly *once given an achievable pose* (its stock `qpos0` is statically infeasible — CoM ~12 cm
outside the point-sphere feet — and the cert rightly FAILs it). One latent generality wart noted
in verification: Atlas's MJCF carries two unactuated placeholder free-bodies whose DOFs get the
LP's default unlimited torque; harmless at 0.25 kg, but the cert does not yet flag spurious free
joints beyond the base.

**Backward-compatibility (regression guard).** `certify(motion='walk')` reproduces the committed
`verify_legged` **byte-for-byte** on flat walks — verified by a side-by-side numeric diff:
go2 score 0.6733 / base_frac95 0.0166 / tau95 0.3267; b2 0.4725 / 0.0309 / 0.5275;
g1 0.0000 / 0.0460 / 1.0000 (match to 4 dp), and the levitate twin FAILs 0.000. `motion='arm'`
delegates to `verify_arm` unchanged (arm toss PASS / toss_far FAIL).

### E2 — Does feasibility predict deploy outcome?  *(harness: `projects/policies/research/shadowing/e2_feasibility_vs_outcome.py`)*

We paired each ghost's certificate verdict with its **known deploy/trainer outcome** (from
local session logs (`_scratch/*.log`, not committed — `_scratch/` is gitignored) and the
canonical [rl-current-state.md](rl-current-state.md)). The
honest finding, which **directly supports the paper's E5 claim**:

> **The certificate is a one-way feasibility triage, not a deploy oracle. Its predictive content
> is a partition of *failures by root cause*, not a learnability curve.**

Confusion matrix over the 8 rows with a known real deploy verdict:
- **cert PASS & deployed** (true positive): 4 — go2 walk, b2 walk, b2 get-up, arm toss.
- **cert PASS & deploy-FAIL** (the E5 zone): 4 — **g1 walk** (the crux row), b2 hill6, omniquad hill8,
  g1 sitstand.
- **cert FAIL & deployed** (a cert MISS): **0** — none observed.
- **cert FAIL & deploy-fail** (true negative): 0 real (the two adversarial controls cover this).

The single most important row is **g1 walk: cert PASS, deploy FAIL**. The certificate certifies the
**ghost**, and the G1-walk ghost genuinely *is* dynamically feasible (base residual 4.6% mg). The
deploy topple therefore lives entirely in the **trainer↔deploy gap** (byte-matched model, drift
compounds on the inverted-pendulum biped), not in the reference. This is exactly *necessary but not
sufficient* (E5): a passing certificate **rules out** an infeasible ghost — so when deploy still
fails, the fix is the tracker / sim-to-deploy, *not* the reference. Concretely the partition lets you
spend compute correctly:

- `cert FAIL` → the ghost is infeasible → **fix the reference** (caught *before* any training compute).
- `cert PASS + deploy FAIL` → the ghost is fine → the gap is trainer / sim-to-deploy (g1 walk; the
  hill trackers stall at the flat→ramp transition) **or** a deeper structural limit of reactive
  tracking (g1 sit-stand's dead-seated launch resists *both* 21 PPO runs *and* MPC-distillation —
  the E5 headline). The certificate cannot tell these two apart, but it removes the third
  possibility — an infeasible ghost — which is its job.

**Honesty about statistical power (do not over-read this):** n is tiny — 11 real ghosts, 8 with a
known deploy verdict, 4 deploy-successes, and exactly **1** cert-PASS-but-deploy-FAIL *legged-locomotion*
case (g1 walk). There is **no** cert-MISS and **no** cert-PASS ghost independently confirmed unlearnable
beyond the G1 prose, so **both error directions are untested**. Several outcomes are documented-only
(rl-current-state.md flags the G1 deploy numbers as such; the B2-hill / omniquad-get-up verdicts come from
engineering notes, not regenerable logs); the flat-walk ghosts are *constructed* kinematic gaits
(representative, not the exact ghost each champion trained on). You **cannot** fit a
feasibility→learnability regression from this — only state the partition. A full E2 study needs
≥ 20–30 ghosts spanning a graded feasibility de-rate, a trained policy + logged deploy-survival per
ghost, and at least one engineered cert-FAIL-but-tracked and one cert-PASS-but-truly-unlearnable case
to probe both error directions. The script is the harness for that study; today it reports the partition.

## Results & findings to date (G1 sit-to-stand test-bed, 2026-06-21)

### What works
- **Ghost Generator** — receding-horizon MPPI over MuJoCo (control = position-target
  actuators) *discovered* a dynamically-feasible **sit → lean → rise → step-forward →
  stand → hold** (peak pelvis 0.81 m, torso tilt 2–5°). Feasible by construction. This is
  the exact upright motion that hand-drawing + 14 prior from-scratch RL runs could **not**
  produce (they bowed 35–47°). Rendered + maintainer-confirmed in OmniSim.
- **Tracking** — with **DeepMimic-style velocity tracking** (reference joint + base
  velocities, not just poses), the RL policy shadows the feasible ghost to within
  **2.5 cm** pelvis-height error from mid-motion states.
- **Throughput** — training is **in-engine and LOCAL**. The flagship trainer runs *through*
  `omnisim-bin` (Newton / mujoco_warp), so **train == deploy bit-exact**, batched on the local
  GPU: **K ≈ 4096 worlds, ~200k env-steps/s on a laptop 5070 Ti**
  ([`run_walk_rl.sh`](../../projects/policies/training/run_walk_rl.sh) →
  [`g1_walk_recipe.py`](../../projects/policies/training/g1_walk_recipe.py)). ⛔ **There is no
  cloud path** — the `cloud/` Modal-H100 wrappers were **removed** (`ef46a52e`); do not reach for
  Modal/H100. The older standalone mjwarp trainers under
  [`projects/policies/research/training/`](../../projects/policies/research/training/) are a
  separate, parity-locked path that also runs locally.

### The open problem (an honest, informative negative result)
The **dead-seated push-off-the-chair launch** (~first 0.2 s) is **not** learned by
RL-on-reference. Across **21 training runs** spanning every lever — hand-drawn *vs*
feasible ghost; uniform *vs* concentrated reverse curriculum; 4096 *vs* 16384 envs;
pose-only *vs* DeepMimic velocity tracking; plus open-loop and hybrid launch-assist — the
policy reaches a ~0.60 m crouch and will not extend the legs / push off. Everything *after*
the launch tracks cleanly (2.5 cm).

**Why** (each ruled in by experiment):
1. **stay-seated is a stable, rewarded local optimum** — attempting the unstable rise risks
   a fall, so the policy retreats to the crouch (alive + partial-tracking reward).
2. **the launch is open-loop unstable AND the MPC's launch controls don't transfer across
   engines** (mujoco→warp) — so feedforward/replay/hybrid-assist all drift and fail.
3. **scale doesn't help** — 4× the envs (16384) changed nothing → it is a *structural*
   optimization problem, not an exploration-*amount* one.

**Finding for the paper:** *a dynamically-feasible reference + RL tracking is necessary but
not sufficient for an unstable, contact-rich INITIATION.* The body of a motion tracks
robustly; an open-loop-unstable contact-transition launch needs more than tracking — a
learned closed-loop initiation (MPC distillation / DAgger) or an easier launch.

### Easier-launch attempt (higher seat / stool) — a diagnosed dead end
We built a stool variant (`generate_g1_stool.py`, `g1_stool_kp100.mjcf.xml`) and swept seat
heights (top 0.50 → 0.56 → 0.68) with feet-planted, lean-only, and lean+step intents. **None
produced an easy rise**, and the reason is a clean geometric tension, not a tuning miss:

- **A higher seat ⇒ near-extended legs at the perch ⇒ the feet tuck under/behind the CoM**
  (measured: feet at x = −0.084 with the pelvis at x = 0 on the 0.68 stool). Leaning forward
  then carries the CoM *away* from the feet, so the robot cannot get over its feet to rise
  *without a step* — the generator correctly refuses (leans 6–20° and sits back down).
- **A lower seat ⇒ the feet land forward of the CoM** (you *can* lean over them and rise) —
  but that is exactly the **deep push-off** launch we were trying to avoid.

So "shallow rise" and "feet-forward-enough-to-rise-without-a-step" are mutually exclusive for
this morphology. The chair ghost is feasible *only* because it is a low sit **with a
catch-step** (feet swing forward to catch the falling CoM). **Seat height never touches the
actual wall:** the dead-seated launch the RL could not learn (21 runs) is independent of it —
raising the seat just trades a deep push-off for a no-rise perch. *Easier-launch via seat
geometry is a dead end.*

### DAgger / MPC-distillation — built, and it hits the SAME wall (a two-method result)
We built the principled fix (`dagger_launch.py`): distill the generator's receding-horizon
MPPI — which *does* execute the launch closed-loop (reaches the full **0.74** stand) — into a
reactive policy, training on the expert's action at the states the **policy** visits (DAgger,
Ross et al. 2011). Two real sub-problems were solved en route:
- **Action representation:** absolute targets over the full joint range give ~0.11 rad (6°) BC
  error per joint — too coarse for balance. Fixed with a **residual anchored on the achieved
  joint trajectory `q_ref`** (position control tracks `q_ref` stably; replaying the MPC's
  *commanded* ctrl open-loop diverges — the closed-loop-controls-don't-replay trap, confirmed
  here in-engine).
- **Stochastic expert:** the MPPI needs high noise to *discover* the unstable launch, but that
  noise makes it a noisy BC teacher (low noise → clean labels but only a 0.60 crouch). Fixed by
  **discovering** the reference once at high noise, then **tracking** in-loop at low noise
  *warm-started on the reference* → clean, near-deterministic labels (loss ≈ 1e-3).

**Result: it does not crack the launch.** Across a full 12-iteration run (and several probes),
the distilled reactive policy reaches a **0.62–0.67 peak then topples** (tilt 60–117°), with
**no improvement across iterations** — even though the in-loop MPC labels reproduce 0.74 and
the labels are clean. This **mirrors the 21 PPO-residual runs**, which stalled at the same
**~0.60 deepest-stable-crouch**.

**Why (sharpened):** completing the stand requires a *predictive commitment to a temporarily-
unstable dynamic extension* — the CoM must pass briefly outside the foot support on the way up.
The lookahead MPC makes this commitment (it sees the stable stand 20 steps ahead); a
**reactive** learned policy will not — small errors on the unstable extension compound faster
than feedforward feedback corrects, so both BC-distillation and reward-RL retreat to the
deepest *statically* stable crouch (~0.60). **A feasible reference + tracking (PPO *or* MPC-
distillation) is necessary but not sufficient for an unstable contact-rich initiation; the
missing ingredient is predictive (closed-loop, lookahead) commitment, which a purely reactive
policy lacks.**

### Next levers (open)
- **BC-then-PPO:** warm-start PPO from the DAgger policy — which already *commits* to the rise
  to ~0.62 (escaping the stay-seated optimum that trapped plain PPO) — and let reward push it
  through to 0.74. The most promising untried combination. *(candidate next build)*
- **Online MPC-in-the-loop deploy:** run the MPPI live in OmniSim (it reaches 0.74) — but the
  generator's plant is plain MuJoCo while deploy is Newton/warp, so a model gap remains.
- **Accept + ship:** the launch is THE documented open problem; the pipeline + the rest of the
  motion (tracks to 2.5 cm) deploy. The negative result *delineates the method's boundary* and
  is itself a contribution.

## Second case study — G1 stand-and-wave (method validates in sim; a deploy-gap finding)

An intentionally **easy, achievable** Shadowing demo to complement the sit-stand boundary:
the robot stands and **waves its right hand**. This is the paper's *hierarchical* idea —
balance is a solved primitive, the wave is the ghost shadowed on top.

**What works (in sim, the deploy engine `mujoco_warp`):**
- **Ghost** — a clean *kinematic* stand+wave reference (steady base, legs at the stand pose,
  the right arm raises → waves → lowers). Maintainer-approved. Design lesson reinforced: the base
  *sway* is the **robot's** job (the solved standing primitive), **not** the reference — a
  free-base humanoid cannot stand rigidly (locking all legs topples it; leaving them free
  makes the *generator* "dance"). The ghost shows the clean intent; the tracker balances
  itself. New reusable generator feature: `Intent.prescribed_joints` (joints forced to the
  keyframe, not optimized — a prescribed upper-body trajectory the MPPI balances *around*).
- **Tracker** — the trainer's new `--wave-ref` drives the arms along the ghost (looped, random
  per-env phase) so the 13-DOF legs policy learns to **balance *through* the waving arm**.
  Result: **987/1000 survival** in the trainer, doing the full wave — the method works.

**The deploy-gap finding (honest):** the on-screen *stationary* stand+wave does **not** deploy,
and the cause is a **pre-existing project gap, not the wave**: robust full-body G1 deploy in
this codebase exists **only for walking**. The dedicated active-balance stand policy
(`g1_stand_arms_deploy`, an "Experiment") does not survive sim-to-deploy — it falls ~0.4 s even
after matching engine, model (`g1_full_kp100`), gains (KE=100/KD=5), NOMINAL (deeper squat),
obs (body-frame ang-vel, proj-g), and joint order. The proven VC-walk *walks* with zero falls
but its *stop-into-stand* is finicky. **A standing humanoid is an inverted pendulum that exposes
every sim-to-deploy discrepancy; this project crossed that knife-edge for walking, never for a
stationary stand.** Two time-sinks masked this: a **stale engine binary** (the physics backend
was changed after the last build — always diff `omnisim-bin.exe` vs `OmNewtonBackend.cpp`
mtimes first) and a **broken training MJCF** (`g1_full.mjcf.xml` has inert position actuators;
use `g1_full_kp100.mjcf.xml`). *Deploying a stationary full-body stand is the open follow-up;
the highest-confidence on-screen demo is walk-and-wave on the proven walk policy.*

## Build order (maintainer-agreed) — ALL THREE COMPONENTS ARE SHIPPED

The original build order was: (1) Ghost Generator — "the missing piece"; (2) Ghost Verifier;
(3) Tracker. **All three now exist**; the paper's components are code, not a plan:

| # | component | shipped as |
|---|---|---|
| 1 | **Ghost Generator** | [`ghost_synth.py`](../../projects/policies/training/ghost_synth.py) + the per-motion builders (`ghost_synth_walk/_squat/_kneel/_pushup.py`), `build_step_turn_ghost.py`, `seq_ghost_retarget.py` — plan the *contacts*, **solve** the base (the base is an output, never a design variable). |
| 2 | **Ghost Verifier** | [`ghost_validator.py`](../../projects/policies/training/ghost_validator.py) (the calibrated pre-training rule gate), [`ghost_dynamics.py`](../../projects/policies/training/ghost_dynamics.py) (gates 1–3: closure / support / FWP), [`ghost_funnel.py`](../../projects/policies/training/ghost_funnel.py) (**gate 4** — PD-realizability), and the research-side `feasibility_certificate.py` (the per-step LP described above). |
| 3 | **Tracker / shadower** | the in-engine flagship recipe — [`run_walk_rl.sh`](../../projects/policies/training/run_walk_rl.sh) → [`g1_walk_recipe.py`](../../projects/policies/training/g1_walk_recipe.py) (corridors + WBMATCH + GHOST-MORPH, train == deploy). |

The whole loop — design → validate → preview → train → verify → register → sequence — is packaged
as the **skill library** ([skill-library.md](skill-library.md)). The design doctrine (ghost-first,
achievable-by-construction, respect-physics, the four gates, the corridor-vs-torque law) is
[ghost-design-rules.md](ghost-design-rules.md).

What is still **open** is not a component but a result: a long-horizon composition study
(BATON's success-vs-horizon experiment) and the E2 feasibility→learnability regression, which
needs ≥ 20–30 graded ghosts.

## Related
- [ghost-design-rules.md](ghost-design-rules.md) — the 7 rules, the FOUR gates, the corridor-vs-torque law, and the ghost toolchain.
- [skill-library.md](skill-library.md) — the packaged pipeline (manifest per skill; BATON sequences).
- [policy-switching.md](policy-switching.md) — BATON, the handover protocol between specialists.
- [rl-current-state.md](rl-current-state.md) — **canonical RL status; cite this for any result claim.**
- [ghost-tracking-pipeline.md](ghost-tracking-pipeline.md) — the original architecture write-up (pre-rename; superseded as the how-to).
- [g1-sitstand-journey.md](g1-sitstand-journey.md) — the running case study + why hand-drawn ghosts fail.
- [g1-universal-tracker.md](g1-universal-tracker.md) — the north-star objective Shadowing realizes.
