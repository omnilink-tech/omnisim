# BATON — explicit policy switching between specialist skills (paper scaffold)

**Status (2026-07-10): the MECHANISM SHIPS; the THESIS is still an open hypothesis.**
This document is the working scaffold for the second OmniSim research paper, alongside
[Shadowing](shadowing.md). Working title: **BATON: Runtime Handover Between Specialist Policies
for Humanoid Skills** ("Policy Switching" is the plain technical name; the maintainer may rename either.)

> ### What is DONE vs what is OPEN — read this before quoting anything below
>
> **DONE — the handover machinery, live-verified, packaged as data.** Four sequences ship as
> [skill-library](skill-library.md) manifests (each reproduces a hand-written demo script, asserted
> **key-for-key on the assembled launch env** by `skill_lib.py verify-demos`):
>
> | sequence | chain | status |
> |---|---|---|
> | [`box_delivery`](../../projects/policies/skills/sequences/box_delivery.json) | walk → stand → pick → carry → **real 90° corner** → place → walk → stand | **verified**, 0 falls |
> | [`box_delivery_classic`](../../projects/policies/skills/sequences/box_delivery_classic.json) | walk → stand → pick → carry → place → walk → stand (no corner — the stable baseline) | **verified**, 0 falls |
> | [`walk_turn_walk`](../../projects/policies/skills/sequences/walk_turn_walk.json) | walk → **90° footwork turn** → walk on the new heading | **verified**, 3/3, 0 falls |
> | [`turn_solo`](../../projects/policies/skills/sequences/turn_solo.json) | settle → 90° footwork turn → decel-stop → hold | experimental |
>
> ⭐ **THE 90° TURN IS SOLVED** (`72a7bb19`, 2026-07-10). The turn is **real footwork** (`wtz=0` —
> zero crane yaw torque), lands at **90.6 / 91.5 / 95.6° actual, 3/3, 0 falls**, and runs **inside a
> BATON sequence**. Every "the crisp 90° corner is the paper's open thread" verdict in the status log
> below is **superseded** — see the 2026-07-10 TURN-LOOP entry at the end of the log. Everything runs
> on the weight-bearing balance harness (`HARNESS_LAM0=0.9`, `HARNESS_KZ=2000`), which is part of the
> shipped configuration, not a hidden crutch — but the *rotation* is the robot's own.
>
> **OPEN — the thesis.** BATON's *switching-beats-a-monolith* claim remains a **well-posed open
> hypothesis**, NOT a demonstrated win: the success-vs-horizon experiment that would prove it is
> **unrun**. The "Where BATON stands vs the field" section below is canonical on this and must not be
> softened. A working handover is not evidence that handover is the better architecture.

## Thesis

The dominant way to make one robot stop, walk, run, and turn is to train ONE policy
conditioned on a command vector (velocity conditioning) and sweep the command. That works
for skills that live on a shared continuum, but it taxes every skill with every other
skill's training distribution, and it cannot absorb a genuinely different skill (carry,
dance, get-up) without retraining the monolith. BATON keeps **independently-trained
specialist policies** — each excellent in its own regime — and makes the **handover
itself** the engineered, verified object: switch windows, reference morphs, and recurrent
state seeding. If the handover is reliable, the skill library becomes open-ended: stop,
walk, run, turn, carry — always switch to the policy you need.

> **The skill library is now concrete** ([skill-library.md](skill-library.md) ·
> [`projects/policies/skills/`](../../projects/policies/skills/)). BATON's specialist registry is
> factored out of the deploy env into per-skill manifests: each skill declares its `baton` role
> (`blend: cyclic` = element-wise blendable | `solo_swap` = context swap), its `mode`, `vx`, and
> `attractor`. `skill_lib.py sequence <name>` assembles the `BATON_SPECIALISTS` / course-or-schedule
> bundle from the manifests, and `handover <seq>` derives the per-edge warm/cold plan from attractors
> (the stand-attractor-lock law as data). `verify-demos` proves the assembly reproduces the proven
> demo scripts. This is where a new specialist is registered and composed.

## Honest related work (position precisely, not grandiosely)

Skill composition is studied: transition policies (Lee et al. 2019, "Learning Transition
Policies for Composing Complex Skills"), skill chaining, the options framework, and
latent-skill spaces (ASE, CALM) that blend skills inside one network. The velocity-
conditioned monolith is the de-facto standard for legged locomotion (and is exactly what
our own VC walk-stop-walk arc built, 2026-07-04 — which is why it is our baseline).

> **⚠️ Thesis under revision (2026-07-08 — adversarial reality check, run w1xgvqeep).**
> A 79-agent both-sides verification pass tested the *strong* form of the thesis above —
> "a monolith cannot scale to long composed jobs; it needs an impractically large network
> and massive compute." **The strong form is NOT supported by 2025–2026 evidence, and in
> one case is directly contradicted:** LHM-Humanoid ([arXiv 2508.16943](https://arxiv.org/abs/2508.16943))
> runs the *exact* walk→pick→carry→place warehouse loop on a **single** policy and beats a
> five-controller hierarchical-RL baseline **72.4% vs 20.8%**. Generalists are also small
> (HOVER = a 3-layer MLP), cheap to run (50 Hz on an onboard Jetson), cheap-ish to train
> (GMT ≈ 4 GPU-days on one RTX 4090), and often *beat* specialists on their own skills.
> **⚠️ IMPORTANT correction (same day, after reading the full LHM paper):** that 72/21
> result is *weak* counter-evidence — LHM's hierarchical baseline used a **naive hard-coded
> oracle-FSM handoff**, not an engineered one, and LHM's *own* Table 5 shows a single policy
> degrades hard with horizon (90→76→61→38→21% over 5 cycles). So the strong "monolith can't
> scale" thesis is **unproven, not disproven**, and the switching thesis survives — re-cast as
> a well-posed open gap. See the 2026-07-08 status-log entries below (the reality-check table
> **and** the "Correction after reading the full LHM paper" that follows it). The original
> thesis text above is kept for provenance; the re-cast version below supersedes it.

BATON's contribution is NOT "nobody thought of switching"; it is:
1. **The handover protocol as a first-class, measured artifact** on a humanoid:
   phase-windowed switch points, reference-corridor morphs (snap = fall, morph = survive
   — the GHOST-MORPH law, measured repeatedly in this repo), and warm-vs-cold recurrent
   state seeding for LSTM specialists.
2. **A like-for-like empirical comparison** against the velocity-conditioned baseline
   *on the same robot, same engine, same references* (our VC arc: EMA-latch gating,
   stopv metric, verified walk↔stand transitions in one net).
3. **Specialists stay specialists**: the walker is the flagship shape champion
   (WBMATCH4 0.868 vs the official gait); adding stop/carry/turn never degrades it,
   because it is never retrained.

## Where BATON stands vs the field, and how we prove the edge (CANONICAL, 2026-07-08)

*Load-bearing summary of the BATON-vs-field analysis (2026-07-07/08). Full evidence + chronology
are in the status-log entries at the bottom. **Read this before making any claim about BATON's
novelty or its edge over one-policy methods.** Two earlier positions — "switching wins" and, later,
"the monolith wins / thesis retired" — were both over-claims; this is the corrected, symmetric
position. For how OmniSim's approach compares against competing simulators and methods, see
[simulator-comparison.md](simulator-comparison.md).*

**The field's direction.** The dominant paradigm is ONE policy for everything — universal motion
trackers (GMT, HOVER, ExBody2, BeyondMimic) and promptable latent foundation models (BFM-Zero,
Task Tokens). They are small (HOVER = a 3-layer MLP [512,256,128]), real-time on onboard compute
(ExBody2 at 50 Hz on a Jetson Orin NX), cheap-ish to train (GMT ≈ 4 GPU-days on one RTX 4090), and
often *beat* specialists on their own skills (HOVER 11/12 real-world metrics). Explicit switching
between separately-trained policies is established prior art — Lee 2019, Byun 2021, Tidd 2021
(switch-estimator + setup policy), RPG 2026 (frozen experts + gating) — but **all of it is
feedforward; none manages recurrent hidden state across the switch.**

**The honest verdict (symmetric — neither side proven).**
- *"Switching beats the monolith / one policy can't scale to a warehouse shift"* — **UNPROVEN.**
  The one published head-to-head on that exact task (LHM-Humanoid, [arXiv 2508.16943](https://arxiv.org/abs/2508.16943),
  walk→pick→carry→place ×N objects in cluttered rooms/warehouse) had a *single* policy **beat** a
  hierarchical baseline 72% vs 21%.
- *But that result is weak, and the maintainer was right to distrust it:* LHM's hierarchical baseline
  used a **naive hard-coded oracle-FSM handoff** (no blend, no learned transition, no recurrent
  state) — exactly the thing BATON is built to replace — so it says nothing about an *engineered*
  handoff. LHM's OWN Table 5 shows a single policy **degrades hard with horizon** (per-cycle
  90→76→61→38→21%, all-five 18%); it is **sim-only**; and LHM itself **distills two teacher
  policies** (internal specialization). So *"the monolith wins"* is **also unproven.** The
  uncertainty is symmetric — and note LHM's *naive* hierarchy degraded even faster, so a modular
  edge is not automatic; it has to be earned with a real handover.

**What is genuinely ours — the claim the paper makes:** a **WELL-POSED OPEN GAP** —
> *Does an ENGINEERED handover between specialists (reference morph + phase-gated timing + action
> crossfade + **recurrent hidden-state management**) degrade more gracefully over long horizons
> than a single distilled policy?*

No one has tested an *engineered* handoff against a single policy on a long horizon; the only
comparison used a naive FSM (which lost, and lost faster on long horizons). The **recurrent
hidden-state handover** — the STAND-ATTRACTOR LOCK and its cold-zero fix (all prior switching work
is feedforward, so this is unaddressed prior art) — is the novel mechanism that could stop the
boundary error-compounding that sank LHM's naive hierarchy. Secondary, honest advantages:
train==deploy, no mocap corpus, one-burst skill add — a small-lab modular alternative, **not** a
performance-superiority claim.

| axis | The field (one policy) | BATON (switched specialists) |
|---|---|---|
| skills per network | many (one tracker) | one per specialist |
| add a new skill | retrain / distill, or frozen-base + adapter (LoRA, Task Tokens) | one training burst, others untouched |
| handoff between skills | none, or a **naive FSM** in hierarchical baselines | **engineered: morph + phase-gate + crossfade + recurrent-state mgmt** |
| long-horizon evidence | degrades with horizon (LHM 90→18% over 5 cycles); **no *engineered*-handoff comparison exists** | **unproven — this gap is exactly what we claim** |
| compute to run | small, real-time on a Jetson | same (per-specialist nets are small) |

### Next steps to PROVE the edge (turn the hypothesis into a result)
1. **The horizon-scaling experiment (the headline plot).** On a long, many-cycle task
   (walk→turn→carry→place → repeat, N cycles), run three systems on the SAME task + engine:
   (a) BATON's engineered handover, (b) a single distilled policy trained on the same skills with
   a real budget, (c) a naive hard-FSM hierarchy. Plot **success-rate vs horizon length (# cycles)**.
   The edge is *demonstrated* iff BATON's curve decays MORE SLOWLY than both — the direct analogue
   of LHM's Table 5. [`baton_metrics.py`](../../projects/policies/training/baton_metrics.py) already
   harvests switches/falls/transients; extend it to log per-cycle success vs cycle index.
2. **Ablate the recurrent-state handover** on that same long task (warm vs cold hidden at each
   switch, DOWN the λ-ladder — the λ=0.9 harness masks it, per the 2026-07-06 ablation). Show the
   stand-attractor lock is what would otherwise cause late-horizon collapse.
3. **A fair, non-strawman single-policy baseline** (the reviewer's first objection): train the
   monolith honestly on the full skill set — do NOT repeat LHM's crippled-baseline mistake in
   reverse.
4. Until (1) exists, the paper states the edge as a **HYPOTHESIS**, not a demonstrated result.

## The v1 demo (walk → stand → walk on the flagship)

- **Specialist A (walker)**: `runs/wr_decent_walker.pt` — the flagship LSTM+foresight
  champion on the λ=0.9 puppet ([rl-current-state.md](rl-current-state.md) banner).
- **Specialist B (stander)**: trained in the SAME era/arch/obs family (LSTM, REF_OBS,
  same 120-dim obs, same harness) against a CONSTANT stand ghost
  (`controllers/g1_ghost/ghost_stand_v1_lut.json`: unitree-default pose = the walker's
  own nominal, arms on the v3 hang, attitude level). Same obs family means the switcher
  swaps only weights + reference lut — the obs assembly, corridors, and harness are
  shared infrastructure. (The walk-calibrated ghost_validator does not apply to constant
  stand luts — motion-class awareness is a known validator TODO.)
- **The switcher** (deploy-side, `g1_walk_recipe_deploy`): a schedule
  (`BATON_SCHEDULE="walk:8,stand:6,..."`) arbitrates per tick which specialist computes
  actions and which reference lut feeds the corridors + REF_OBS block.

## Protocol axes (the experiment matrix)

| axis | variants | hypothesis |
|---|---|---|
| switch timing | any-tick vs phase-windowed (walk→stand only at double support; stand→walk at a canonical phase) | phase windows cut transient attitude excursion |
| reference handover | hard swap vs corridor-center morph over N ticks | snap swaps fall (GHOST-MORPH law); N≈30 survives |
| action handover | hard swap vs linear crossfade over N ticks | crossfade smooths torque discontinuity |
| recurrent state | cold (zero hidden at switch-in) vs warm (feed BOTH nets every tick; switch-in inherits live hidden) | warm handover removes the settle transient |
| phase clock | freeze during stand + reset-to-canonical on walk resume vs free-run | reset gives a deterministic first stride |

## Metrics (verify-before-show applies)

- **Switch success rate** (N scheduled switches, zero falls) — the headline number.
- **Transient excursion**: peak |roll|/|pitch| and pelvis-z dip within 2 s of each switch.
- **Recovery time**: ticks until WBMATCH4-instantaneous returns to the specialist's
  steady band.
- **Stand quality during stand phases**: stopv (the VC arc's stillness metric).
- **Baseline**: the same schedule driven through the VC champion (cmd sweep 0.45↔0)
  — same metrics, same arena.

## Status log

- 2026-07-06: campaign started. Stand ghost minted; stander specialist training queued
  (same era as the flagship walker). Switcher implementation next. Known context: the
  flagship's live stride gap (see rl-current-state banner) applies to BOTH specialists
  equally — switching experiments are valid on-puppet regardless.
- 2026-07-06 (same evening): **FIRST LIGHT — 9/9 switches, zero falls, first attempt.**
  - Stander specialist trained in minutes at λ=0.9 (both seeds: WBMATCH4 0.950/0.946 vs
    the stand ghost, attitude ≈1.0) — `runs/wr_stander.pt` (= wr_stand_1_it50).
  - Switcher shipped in `g1_walk_recipe_deploy`: BATON rides the proven `WALK_SCHEDULE`
    arbiter; when armed (`BATON_POLICY_B` + `BATON_LUT_B`), the stand mechanism is the
    LEARNED specialist instead of the deterministic hold. Per tick: reference tables
    (corridors + REF_OBS + harness-att) BLEND walker-lut ↔ stand-lut over
    `BATON_MORPH_TICKS` (default 30; morph-never-snap), actions crossfade, and BOTH
    LSTMs run every tick (warm recurrent handover). Walk→stand exits are DS-gated
    (`BATON_DS_GATE=1`): measured switch-out phases cluster at the two double-support
    windows exactly as designed.
  - **First-run metrics** (walk:8/stand:6 ×5 cycles, λ=0.9 puppet, compound feet):
    9/9 scheduled switches, 0 falls; worst-case transient within 2 s of ANY switch:
    |roll| 2.6°, |pitch| 1.9°, min pelvis z 0.740 (the normal walking band — the
    switches are invisible in attitude telemetry). Stand phases dead-still (vx 0.00,
    |roll| 0.001, the stander even rises to full height 0.778).
  - Next: protocol-axis A/Bs (morph length, DS gate off, cold hidden, hard swap) to
    fill the experiment matrix; VC-baseline run on the same schedule; GUI demo.
- 2026-07-06 (later): **protocol ablations complete** (same schedule walk:8/stand:6 ×5,
  9 switches each, λ=0.9 puppet; harvested with `baton_metrics.py`):

  | config | switches | falls | worst \|roll\| | worst \|pitch\| | min z | stand mean \|vx\| / \|roll\| |
  |---|---|---|---|---|---|---|
  | **morph 30 (baseline)** | 9/9 | 0 | 2.6° | 1.9° | 0.740 | 0.006 / 0.0004 |
  | hard swap (morph 1) | 9/9 | 0 | **3.4°** | 2.1° | 0.740 | 0.006 / 0.0003 |
  | morph 10 | 9/9 | 0 | 2.3° | 1.9° | 0.740 | 0.006 / 0.0003 |
  | morph 60 | 9/9 | 0 | 2.4° | 1.8° | 0.740 | 0.006 / 0.0001 |
  | DS gate OFF | 9/9 | 0 | 2.6° | 1.8° | 0.739 | 0.006 / 0.0005 |
  | cold hidden | 9/9 | 0 | 2.6° | 1.9° | 0.740 | 0.006 / 0.0003 |

  **Honest interpretation.** At λ=0.9, switching is extremely forgiving: every config
  achieves 9/9 with zero falls. The only differentiated axis is the reference morph —
  a hard swap raises the worst-case transient ~40% (3.4° vs 2.3–2.6°); ten ticks of
  morph already suffice. The DS gate and warm-vs-cold hidden show NO effect *at this
  harness level* — the pelvis wrench masks their contribution. The July-4 bare-robot
  lessons (snap-to-nominal fell in 3.5 s; mid-swing freezes fell in 1 s) say these
  axes bite hard at low λ, so **the ablation matrix must be re-run down the λ-ladder**
  before the paper can claim which protocol elements are load-bearing. λ=0.9 result:
  the protocol's headroom is real, the discriminating regime is below it.
- 2026-07-06 (same session): **VC-monolith baseline on the IDENTICAL schedule** — one
  velocity-conditioned policy (same era/arch/obs, `VC_REST` corridor-blend + mid-episode
  cmd resampling, 800 iters ≈ the specialists' combined budget; ckpt it100; live cmd via
  `WALK_CMD_PROFILE` matching walk:8/stand:6 ×5 exactly):

  | | BATON (specialists) | VC monolith |
  |---|---|---|
  | falls | 0 | 0 |
  | worst transition \|roll\| | 2.6° | 3.3° |
  | worst transition \|pitch\| | 1.9° | 2.3° |
  | stand stillness mean \|vx\| | **0.006 m/s** | 0.022 m/s (3.7×) |
  | stand stillness mean \|roll\| | **0.0004 rad (0.02°)** | 0.0154 rad (0.88°, 38×) |
  | walker shape (WBMATCH4, own exam) | **0.868** (never retrained) | 0.866 peak, oscillating |

  **Reading.** At λ=0.9 both survive; transitions are mildly better under BATON (the
  monolith's edges resemble BATON's hard-swap ablation). The decisive difference is
  WITHIN-skill quality: the monolith fidgets while "stopped" (38× the specialist's roll
  noise) because one network trades stand stillness against walk shape, while BATON's
  stander is dead-still and its walker is the untouched flagship. The thesis in one
  table: **conditioning taxes every skill with every other skill; switching doesn't.**
  Caveats to close before the paper: single seed each, VC ckpt selection (it100), and
  the λ-ladder re-run above.
- 2026-07-06 (evening 2): **THE THIRD SPECIALIST — carry, and the open-endedness claim.**
  - Carry ghost minted in one FK-design pass (`ghost_carry_v1_lut`: the flagship's authentic
    legs + sway, CONSTANT carry arms — hands chest-high 0.28 m apart, elbows bent; FK-verified
    hand placement before any training). `--shyaw` added to build_keypoints for the ruler tables.
  - Payload as plant modification (`CARRY_PAYLOAD_KG=1.5`): +0.75 kg per hand body in the batched
    model (subtree masses updated). The engine-compiled model has GENERIC body names, so hands
    resolve STRUCTURALLY (leaf of the elbow chain — matched the FK-derived link ids exactly).
    A payload-free control (0.91) and the payload specialist (0.906 @ it100, `runs/wr_carrier.pt`)
    train in one 400-iter burst each.
  - BATON generalized to an N-SPECIALIST registry (`BATON_SPECIALISTS="name|ckpt|lut|vx;..."` —
    pipes because Windows paths carry colons). Regression: the 2-specialist config reproduces its
    metrics exactly through the new code path (2.6°, 0.741, 0.006/0.0007).
  - **Three-specialist verification** (walk:8 → stand:4 → carry:10 → stand:4 → walk ... ×2):
    **7/7 switches, zero falls, worst transient 2.3° roll / 1.7° pitch, min z 0.741, stand
    stillness 0.0001 rad** — the carry handovers are as clean as the stand ones.
  - The open-endedness datum for the paper: a semantically NEW skill (carry — no command value
    in a velocity-conditioned monolith means "hold a box") joined the team in ~1 hour end-to-end
    (FK design → ghost → burst → registry entry), with the flagship walker's weights untouched.
  - Honest v1 caveat: the LIVE demo box is a visual prop that tracks the hands (the 1.5 kg payload
    dynamics exist in the TRAINING plant only); v2 = a real welded box body in the world (the
    weld-grasp machinery exists; warm-solver bounce trap is documented).
- 2026-07-06 (evening 3): **THE REAL SEQUENTIAL DEMO — a physical box through the full
  walk → stand → carry → stand cycle.** The requirement was that every element be physically
  real (no supervisor fakery). The delivery
  set: a static cart with real collision beside the arena; a REAL 1.5 kg box that
  genuinely RESTS on it (contact physics, zero supervisor writes), transfers to the
  hands at each stand→carry handover, rides the FK hand centroid through the carried
  walk, and is released 4 cm above the cart at placement — the landing/settle is
  genuine contact physics. Verified: **full sim speed, 7/7 switches, ZERO falls, worst
  transient 2.1° — the best switching numbers of the campaign, with the real box.**
  - ⛔ **THE BACKEND LESSON (measured twice, the day's most expensive discovery):** a
    Newton-backed prop makes every supervisor write dirty the solver state. Per-tick
    writes = ~40× re-import slowdown + an untrained physics regime (robot can't walk);
    even a ONE-TIME write mid-run clobbered the robot to a stale state (instant
    face-plant at the place swap). **Fix: props that supervisors move must be
    `physicsBackend "ode"` Robot nodes** (the ghost-hologram precedent) — they live
    outside the Newton state entirely, so writes are safe and cheap while gravity and
    contact stay real. Mixed-backend caveat: no box↔robot contact across backends (the
    carried pose keeps hand clearance by design; no runtime weld exists in the engine,
    so the carried phase is kinematic-honest and the payload dynamics are the carry
    policy's trained plant).
  - The deploy harness also gained free-joint offset resolution (`joint_qd_start` ×
    `joint_child` scan) — it no longer assumes the robot's base owns joint_f dofs 0-5,
    which is correct-by-construction for any future multi-free-body Newton scene.
- 2026-07-06 (evening 4): **THE WARM-HANDOVER STAND-ATTRACTOR LOCK — the ablation's
  "no effect" verdict was schedule-scoped, and the carrier proved it.** The maintainer's eye
  caught the carry leg "dancing"; retraining the carrier with translation actually paid
  (`W_TRACK_LIN=10`, the treadmill-optimum fix) produced a specialist that walks 12.9 m
  live SOLO — yet the same checkpoint MARCHED IN PLACE inside the course, on two runs,
  regardless of leg bearing. The knife: `BATON_COLD_HIDDEN=1` — cold-zeroing the
  incoming specialist's recurrent state at switch-in — and the carry leg immediately
  strode the full 4.5 m (course19: 3/3 arrivals, 5 switches, 0 falls, worst tilt 7.8°,
  ends standing). Reading: the warm all-LSTM handover feeds every specialist the live
  obs stream, so during a stand segment the carrier's hidden settles into the STAND
  attractor; at switch-in the warm state *is* the lock, and a translating gait never
  re-emerges (the hidden-state-gated family — same phenomenology as the live-dead
  heading channel). The July-6 ablation saw no cold-vs-warm difference because
  walk↔stand×5 with the BASE walker never hands INTO a warmed translating specialist
  after a long stand. **Law: warm handover for transient smoothness is only safe INTO
  attractor-compatible regimes; switching into a locomotion specialist after a stand
  requires cold (or stand-uncontaminated) hidden.** The paper's protocol section gains
  its first genuinely load-bearing axis at λ=0.9.
- 2026-07-06 (evening 5, session close): **THE SQUARE PATROL — the maintainer's next shape,
  and the 90°-turn wall it exposed.** Request: walk a square, BATON-switching
  walk→stand at each corner, then walk again. Corner arrival + both switches verify
  cleanly (the cold-hidden protocol carries over). The TURN is the open piece, three
  attempts in:
  - square1: course slews the heading target 90° right after the walk re-entry — the
    crane's yaw trim twisting a freshly-restarted gait trips it (~2200 ticks in).
  - square2: PIVOT-DURING-STAND (arbiter pre-slews the target toward the next leg
    while the stander holds). Failed for a physical reason worth recording: planted
    feet at μ=1.5 beat the ~18 N·m yaw trim — the ROBOT never rotates, only the
    target does, so the walk re-entry inherits the full 90° error at tick one. Same
    fall. **Law: never slew a target ahead of a plant that cannot follow it.**
  - square3 (in flight at close): the CARROT WINDOW — `BATON_TGT_WIN` caps the
    heading target within ±0.30 rad of the robot's ACTUAL yaw, so the turn proceeds
    exactly at the achieved rate and the heading obs stays in-distribution. Plus
    `BATON_SLEW` (slew-rate knob) and yaw/htgt in the deploy telemetry line.
  - If the 90° corner holds as a wall it borders the live-dead heading channel
    (the paper's open research thread); the honest fallback demo is the same course
    with softened (octagon) corners — every vertex still a real walk↔stand↔walk
    switch. Next session: harvest square3, iterate from the yaw telemetry.
  - square3 verdict (harvested at close): the carrot window as a SYMMETRIC clamp is
    wrong — capping the target to yaw±0.30 lets the walker's natural yaw wander DRAG
    the target with it (leg 1: yaw walked off to −1.26 rad; every anchored run held
    ~0). The window must be ONE-SIDED: the target steps only TOWARD the course
    bearing, capped in lead, never following the robot's wander backward. **Law: the
    heading anchor is load-bearing — nav4's drift cure IS the fixed target; any
    steering scheme must preserve it.** Next session: one-sided window, then the
    square (or octagon fallback).
- 2026-07-07: **SQUARE campaign continued — the 90° turn characterized, and it is the
  live-steering wall.** Ten instrumented runs (square1–10) to make the G1 walk a square,
  BATON-switching walk↔stand at the corners. The switching verifies every time; the
  **turn** is the open piece, now fully diagnosed:
  - **Cornering while WALKING is budget-bound.** The crane's yaw torque turns the puppet
    only as fast as the feet follow (~55 N·m before a walking robot tumbles). At that
    rate a 3.5 m square can't be cornered — the robot traces a ~5 m-radius arc and
    spirals out (square5). Wide arcs only; sharp corners no.
  - **The pelvis↔travel yaw offset is GAIT-DEPENDENT** (square4 telemetry): ~0 rad while
    standing, ~−1.44 rad mid-gait. So the crane comparing raw pelvis yaw to a world
    bearing can't tell a real 90° turn from the baked-in offset — and no *static*
    `HARNESS_YAW_OFFSET` fixes it (square5 spiraled once the constant made stand/walk
    inconsistent). This is the same signal the policy's own (dead) heading obs carries,
    so it's of a piece with the live-dead heading channel.
  - **In-place pivot during the STAND is STICK-SLIP** (square6–10). Rotating the
    stationary robot needs yaw torque **above the static breakaway (~108 N·m)** to move
    at all; below it the feet don't slip (square7 @72 N·m, square8's slew-capped 90 N·m).
    Above it the feet break free — but with kinetic friction lower than static, the slip
    overshoots and rings unless heavily damped (square6 @108 N·m, KYD 40, rang 1.7 rad).
    And strong damping to kill the ring keeps the torque below breakaway so it never
    frees (square7/10, KYD 110–140, stalled). Breaking stiction wants an unopposed
    impulse; stopping the slip wants damping; the two fight. Worse, the outcome depends
    on the exact arrival state (square9 broke the *wrong* way) — **not demo-robust.**
  - **Verdict (as of 2026-07-07; ⭐ SUPERSEDED — the turn was SOLVED on 2026-07-10 by `72a7bb19`,
    see the last log entry):** BATON walk↔stand switching is solid (the delivery is the standing
    proof, 0 falls). Sharp *live* turning was, at this date, unsolved. The
    stand-pivot is filed as *promising but not robust* — the right fix is a trained TURN
    specialist (an arc-reference ghost, the earlier path (a)), not a hand-tuned crane
    against stick-slip friction — **and that is exactly the path that closed it.** All the square
    machinery (stand-pivot, carrot window,
    yaw telemetry, `HARNESS_KYAW_STAND`/`_KYD_STAND`/`BATON_PIVOT_SLEW`/`BATON_TGT_WIN`/
    `HARNESS_YAW_OFFSET`) ships env-gated and **default-off**, so the proven walk↔stand
    path is unchanged.
- 2026-07-07 (later): **TURN SPECIALIST — first LIVE turn in the lineage.** Maintainer chose the
  principled fix (train a turn specialist, not a crane vs stick-slip). Design: an
  **unconditional** turner (every training env forced to a fixed low forward speed +
  fixed yaw rate via `TURN_VX`/`TURN_WZ`), so turning is baked into the policy like the
  stander's stand — no live command needed, sidestepping the dead heading channel that
  left nav4 turning only in-trainer. Trained in one 300-iter burst (`wr_turner1`, walker
  ghost, λ0.9 puppet). **Result: it turns ~38° LIVE, dead upright (z=0.751 solid), where
  nav4 turned 0° live** — the unconditional spin transferred. Limit: the turn saturates at
  a ~38° transient per trigger (the recurrent hidden then converges to a straight-walk
  attractor — the same recurrent-attractor phenomenon as the warm-handover stand lock;
  in training the *reward* drove continuous turning, absent live). Crane-assist made it
  worse (turner+crane fought → a wandering wrong-way arc + more body stress). So the
  corner mechanism is the **solo** turner: switch it in for one clean ~38° turn.
  Consequence for the demo: the shape adapts to the live turn range (a ~40°-corner polygon
  via a BATON `turn` segment), not a 90° square — or a follow-up burst tuned for a larger
  live turn. `wr_turner.pt` saved; `TURN_MODE`/`TURN_DEPLOY_WZ` deploy hooks added
  (env-gated, default off).
- 2026-07-07 (delivered): **WIDE-ARC TURNING PATROL — the maintainer-chosen turning demo, live,
  0 falls.** Given the crisp 90° corner is the open live-steering wall, the maintainer chose a
  wide-arc patrol using the steering that *works*: the gentle crane (`HARNESS_KYAW=150`,
  ±0.12 budget) trims a walker toward course waypoints, turning ~40-60° across a rounded arc
  the way the delivery already did. Two findings closed it: (1) keep the turn OFF the
  stand→walk re-entry (the cold-restart gait trips on an immediate turn — arcpatrol1 fell
  there); put turns on continuous-walk arrivals (the delivery proved ~36° is safe). (2) the
  steering-trained nav4 has a live durability limit (~7 m before a fall); the **champion
  walker `wr_decent_walker`** (0.868 flagship) carries the loop — the crane steers whichever
  policy walks, so the durable one completes it. Verified course
  `walkto,3,0;stand,2;walkto,5.5,0;walkto,7,1.3;stand,0`: **4 arrivals, 3 BATON walk↔stand
  switches, a ~58° crane-steered turn, 0 falls, worst tilt 2.1°, ends standing.** Shipped as
  the honest turning-patrol demo (no code change — existing BATON course arbiter + crane +
  champion walker + stand specialist). At this date the crisp 90° corner was still the documented
  open thread (needing the live-dead heading channel solved, or a fast intrinsic turn specialist).
  ⭐ **SUPERSEDED 2026-07-10** — the intrinsic turn specialist + TURN-LOOP closed it (`72a7bb19`);
  the shipped `walk_turn_walk` / `box_delivery` corners are pure footwork (`wtz=0`), not this crane
  arc.
- 2026-07-07 (review challenge: the turn was suspected of being driven by the harness rather than
  by footwork, with a requirement to turn by real footwork, walk the same in all directions, and
  verify numerically): **CONFIRMED + PIVOT-STEP SHADOWING CAMPAIGN
  started.** The wide-arc patrol's turn WAS the crane (harness yaw torque) -- valid catch.
  - **Numerical proof (the wtz instrument).** Added per-tick logging of the harness yaw
    torque `wtz`. Deployed the turn specialist with the crane OFF (`HARNESS_KYAW=0`):
    `wtz=0.00` every tick -> the harness contributes ZERO rotation. This both confirms the
    old turn was the rope AND proves the WALK is genuine footwork (the harness applies no
    forward force either -- only vertical/attitude support to stay upright).
  - **A real footwork turn exists but is weak/slow live.** The unconditional turn specialists
    (turner1/turner2, trained with no yaw torque so the turn MUST be footwork) turn ~19-38deg
    over ~30s live then plateau (the recurrent hidden settles to straight-walk; in training a
    reward drove continuous turning, absent live). Real footwork (wtz=0) but not demo-crisp.
  - **Maintainer chose the pivot-step Shadowing campaign** (the principled fix). Per the
    ghost-design rules (Rule 1: achievable-by-construction, no hand-designed ghosts), the
    reference is a REAL HUMAN TURN retargeted from LAFAN1. Found clean discrete walk-turn-walk
    windows in the in-repo LAFAN1 clips; extracted **a LAFAN1-retargeted turn reference** (walk1
    frames 2988:3114 = t99.6-103.8s): a 4.2s, 126-bin sequence -- straight walk -> ~99deg
    footwork turn -> straight walk, upright (z 0.74-0.80), real human sway. Validator: WARN
    (leg-symmetry expected-asymmetric for a turn; harmonic-jitter = kinematic-retarget
    signature; provenance false-flag -- it IS a recording). NEXT PHASE: train a reference-in-obs
    tracker (community-tracker plan) to follow this ghost -> a real footwork walk-turn-walk,
    then verify wtz=0 live and build the demo.
- 2026-07-07 (later): **Turn-tracker training -- STRUCTURAL NaN blocker (5 attempts).** Set up the
  Shadowing tracker on `ghost_pivot_turn` with GHOST_SEQ (the sequence machinery feeds a yaw-rate
  command from the reference root; the turn is meant to emerge from leg-imitation of the human's
  turning steps). Two real fixes landed: (a) the raw LAFAN1 pace (|vx|1.43, |wz|1.45) is infeasible
  for the puppet -> retimed the ghost 3.6-5.5x slower (feasible |vx|~0.3, |wz|~0.3); (b) seq_mode
  now drives `ytgt` from the reference yaw so the harness lateral-catch/velocity frame FOLLOWS the
  turn (else it fights the rotation), and `hold_end` is honored in TRAINING (pin the phase at the
  end -- a walk-turn-walk does not loop, so the cyclic wrap snapped the reference -99->0deg).
  ⛔ DESPITE all of it, training diverges to NaN at **it~55-60 every run**, config-independent
  (LR 1e-4..2e-4, attitude-ghost on/off, pace 3.6-5.5x, leg-vs-wb ref obs, soft-vs-hard weights).
  The reference data is clean (all finite, no bad bins) -- it is a TRAINING-DYNAMICS instability:
  the tracker drives the robot into a physics blowup as it learns the turn. This is the same class
  of failure that PARKED the earlier sequence/mocap campaign (sequences fail on dynamics content).
  It needs dedicated trainer research (value/repro-stabilization, or a different tracker
  formulation -- e.g. the community MTRACK keypoint tracker with adaptive sampling), not a config
  sweep. STATE: bug fixed (no more rope-turn), reference authored+validated+retimed+committed, seq
  frame-following + training hold_end committed (env-gated, don't touch the shipped path). The
  stable turn-tracker is the open next step.
- 2026-07-07 (CORRECTION): the earlier "the turn ghost is not achievable /
  sequences NaN structurally" claim was **WRONG**, and this entry supersedes it. The NaN was
  *reference infeasibility*, not turns-or-sequences. A turn is a solvable design problem — specify
  how the G1 should step and the ghost IS achievable. **`build_step_turn_ghost.py`
  SOLVES a statically-stable step-turn** (footstep plan + IK against the G1 model, COM kept over the
  stance foot; IK 0.3 mm, COM-inside 54/54 single-support frames, joint+vel limits pass). It
  **shadows with ZERO NaN** (trained 200 iters, surv 1.0, gmatch 0.86) and **deploys as real
  footwork** (crane off, wtz=0, upright, in place) -- where five runs on the *infeasible* LAFAN1
  human turn NaN'd at it≈55 every time. First live footwork turn was ~25° (under-executed: the
  ghost's weight-shift was discretely-set per keyframe -> a 1.5 m/s lateral spike; smoothed to
  0.29 m/s, retraining for a full 90°). ⭐ **The general lesson (corrected across the sim):**
  Shadowing needs a *feasible* reference; feasibility has TWO valid routes -- (a) RECORD a stable
  in-engine behavior, or (b) SOLVE the trajectory to satisfy the physics (COM-over-support + limits).
  Route (b) is NOT the banned "eyeballed hand-design"; it is achievable by construction. See the
  corrected ghost-design-rules.md Rule 1 + the validator's T2.provenance / T0.arm-symmetry fixes.
- 2026-07-08: **THE 90° FOOTWORK TURN NOW RUNS INSIDE A BATON SEQUENCE (SOLO-TURN context-swap).**
  The goal: use BATON to combine the 90° turn with walk/carry for the box demo. The obstacle was
  structural: the 90° step-turner (`wr_stepturn`) is a SLOW WHOLE-BODY SEQUENCE (nb≈225 seq,
  REF_OBS_WB **153-dim**, ω≈0.3) — it cannot element-wise blend with the fast cyclic **120-dim**
  walk/carry/stand family (the blend does `(1-u)·a + u·b` on the reference tables, which needs a
  shared `nb` AND cadence). Two dead ends were ruled out first and are worth recording: the only
  *blend-compatible* turner (`wr_turner`, 120-dim) turns **~15° and reverts** (recurrent state →
  straight-walk, non-cumulative even chained cold-hidden — too weak); and a **carry-turn** specialist
  (turn WITH carry arms, so the box is held THROUGH the corner) trains fine but **falls live** — the
  raised carry-arm COM destabilizes the dynamic turn (both obs families, ~t150, though the in-trainer
  DEPLOY-EVAL survives 0.87). Filed as a hard case.
  - **SOLO-TURN** is the fix (`g1_walk_recipe.py`, gated `BATON_TURN_CKPT`/`BATON_TURN_LUT`,
    default-off → the shipped blend path is byte-identical). On a `turn` segment, `_wr_turn_swap`
    swaps EVERY `world._wr_*` deploy field — net, hidden, ref tables, ghost lut, phase clock, cmd,
    arm tables, and the sequence machinery (`seqcmd`/`seq_yaw` that drive the turn) — to the turner's
    context, so the entire deploy transparently runs it solo; the blend + the warm all-LSTM loop are
    both skipped while `_wr_in_turn`; restore on exit, cold-hidden both directions.
  - **VERIFIED LIVE** (`g1_walk_puppet`, `OMNISIM_FOOT_TORSION=0.25`, `HARNESS_KYAW=0` ⇒ `wtz=0`
    pure footwork): **walk → turn → carry → stand, 0 falls**, upright z 0.66–0.78, the turn climbs
    12→35→59→86→**99°** by real footwork and `TURN_TO_DEG` decelerates it to a stop. The FIRST time
    the 90° footwork turn combines with walk/carry through BATON.
  - Four gated bugs found + fixed (all in the commit msg): (a) size the turn net to the turner's own
    OBS_DIM 153 via a scoped global swap, not the primary's 120; (b) the warm all-LSTM loop must SKIP
    while solo-turn is in (153-dim obs ≠ 120-dim blend nets); (c) reset `_wr_yaw_prev` by DELETING it
    (setting `None` → `_ynow - None` TypeError in TURN_TO_DEG → froze the phase, the turn silently
    no-op'd with no telemetry); (d) force a fresh re-entry on swap-out (`walk_entered=False`).
  - **Two honest walls remained at this date** (both closed on 2026-07-10 — see the next entry).
    (1) HEADING RETENTION after the turn is weak: the carrier drifts back
    toward its default travel direction (the live-dead heading channel), so the 90° is clearly visible
    DURING the turn segment but the new heading isn't held through the carry. (2) turn→stand FALLS (a
    stand catch can't grab the settled post-turn pose) whereas turn→carry is stable (the walking motion
    settles it) — hence the demo shape is walk→turn→carry(→stand). Both trace to the same open thread
    the square campaign named: making the walker *maintain* an arbitrary heading live.
- 2026-07-10: ⭐ **THE 90° FOOTWORK TURN IS FINALIZED — `walk_turn_walk` ships (`72a7bb19`; deploy
  mechanism landed with `fde9b36c`).** The G1 walks 5.2 m on +x, turns a real ~90° by **pure
  footwork** (`wtz=0` — the crane contributes zero rotation), and walks a clean straight leg along
  the new heading. The defect open since `21e672d5` — the turn settled at ~60–65°, so leg 2 headed
  ~62° — is **closed**. Demo config + docs only; no change to the shipped blend path.
  - **Root cause, measured.** The 90° step-turn ghost, played **once**, banks only ~2/3 of its yaw
    live: the legs track the ghost but the base under-rotates (foot slip; gain **0.67–0.73**, stable
    across runs). ⛔ A *slower reference clock cannot add rotation* — slip is per-STEP. **More steps
    can.**
  - **TURN-LOOP** (the loop-the-turn, stop-on-the-*actual*-heading law, implemented at the
    sequence level): the ghost is a **modular staircase of 15° feet-together mini-pivots**, so the
    deploy replays **partial passes** — restarting at the plateau whose remaining staircase ≈
    (remaining angle ÷ measured gain) — until the **ACTUAL accumulated heading** reaches
    `TURN_TO_DEG`.
  - ⛔ **NEVER stop a sequence-ghost mid-lut.** Two mid-lut arrest designs were killed by telemetry:
    freeze-on-plateau **recoiled −19°**; jump-to-end-hold **spun and fell**. The robot *lags* the
    ghost, so a ghost plateau says nothing about the robot's stance. The surviving rule: **every pass
    plays THROUGH the ghost's own final decel into its end-hold.**
  - ⛔ **`TURN_HOLD_LOCK=0` for this turner.** The completed-turn pelvis-xy clamp (built for the
    continuous turner's post-decel slide) is OOD for a feet-together in-place pivot: measured, the
    settled end-hold stayed perfect ~1 s and then the clamp wound it into a **spin + fall (2/2)**.
    (The stand-segment anti-slide hold is unaffected.)
  - **Verified 3/3 headless** (telemetry, not screenshots): the turn lands **90.6 / 91.5 / 95.6°
    actual** in 2–3 adaptive passes; keep-heading pins leg 2 at 84–90°; leg 2 walks straight at an
    82–90° bearing (best: 0.3 m x-drift over 6.4 m of +y); min z ≥ 0.662; **0 falls**.
    `verify-demos` MATCHes (the `walk_turn_walk` manifest reproduces the script's 39 env keys).
  - **Consequences for this doc:** `turn_in_place` is promoted `experimental → verified`; the
    "crisp live 90° corner is the paper's open thread" verdict (2026-07-07) and the two "honest
    walls" above (heading retention, turn→stand) are **superseded**. The **thesis** (below) is
    untouched: a working handover is not evidence that handover beats a monolith.

---

## 2026-07-08 — REALITY CHECK: does "one policy for everything" actually fail to scale?

**Why this section exists.** The maintainer's motivating intuition for BATON is that a single
monolithic policy cannot scale to a long, composed job — "a warehouse shift: walk somewhere,
stop, carry a box, place it, do another task" — without becoming an impractically large network
that needs massive compute to train *and* run; therefore a modular library of switchable
specialists must be the better substrate. We tested that intuition adversarially (run
`w1xgvqeep`: 7 both-sides web-research finders → 3 skeptical verifier votes per claim, 21/24
claims confirmed against primary arXiv sources). **The strong form of the intuition did not
survive.** No confirmed evidence supported it; the confirmed evidence runs the other way.

### What the 2025–2026 literature actually shows (all verified against primary sources)

| Maintainer's premise | Verified finding | Source |
|---|---|---|
| A monolith can't do the long walk→pick→carry→place warehouse loop | **LHM-Humanoid trains one policy for exactly that**, "a single general policy that directly outputs actions rather than invoking pre-trained skill libraries," and beats a 5-controller hierarchical-RL baseline **72.4% vs 20.8%** on a 350-task 2–5-object warehouse benchmark | [2508.16943](https://arxiv.org/abs/2508.16943) |
| A modular skill library is the *safer* way to compose long horizons | Same paper: the **modular** baseline was the *brittle* one — "individual low-level controllers cover narrow action manifolds; small changes in object pose or clutter induce OOD conditions that compound across stages" | [2508.16943](https://arxiv.org/abs/2508.16943) |
| It would need an impractically large network | HOVER covers **15+ control modes with a 3-layer MLP [512,256,128]**; even SONIC, a scaled "foundation" tracker on 700 h of mocap, is only **1.2M–42M params** | [2410.21229](https://arxiv.org/abs/2410.21229), [2511.07820](https://arxiv.org/abs/2511.07820) |
| Massive compute to *run* | ExBody2 (arbitrary-motion generalist) deploys at **50 Hz on an onboard Jetson Orin NX**; SONIC runs its policy forward pass in **1–2 ms on a Jetson Orin** | [2412.13196](https://arxiv.org/abs/2412.13196), [2511.07820](https://arxiv.org/abs/2511.07820) |
| Impractical to *train* | GMT trains one unified whole-body tracker (8,925 clips) in **≈4 GPU-days on a single RTX 4090** | [2506.14770](https://arxiv.org/abs/2506.14770) |
| Cramming skills into one net taxes each skill (interference) | Interference is real but **shrinks with scale** — "the rate of gradient conflicts decreases as the number of parameters increases; high-capacity models can partly substitute for gradient surgery" | [2505.23150](https://arxiv.org/abs/2505.23150) |
| Specialists keep a quality edge | Generalists now *beat* them: HOVER wins **11/12 real-world metrics** vs ExBody/HumanPlus/OmniH2O; GR00T-N1 (2.2B) **76.8% vs 46.4%** vs a task-specific Diffusion Policy; SONIC **98.5% survival vs a locomotion specialist's 43%** | [2410.21229](https://arxiv.org/abs/2410.21229), [2503.14734](https://arxiv.org/abs/2503.14734), [2511.07820](https://arxiv.org/abs/2511.07820) |

### Where the intuition *does* retain a grain of truth
- **Naive monolith fine-tuning to add a skill DOES catastrophically forget** (full-fine-tune
  regressed prior manipulation suites, e.g. 0.79→0.53). *But* the field's answer is not a
  library of separate specialists — it is a **frozen shared base + a tiny per-skill module**:
  TAIL-LoRA adds a task with ~1% of the params and **zero backward-transfer forgetting (BWT=0)**
  ([2310.05905](https://arxiv.org/abs/2310.05905)); Task Tokens adds a humanoid task with **~200K
  params on a frozen base, 125× fewer params, 6× faster** ([2503.22886](https://arxiv.org/abs/2503.22886)).
  This is parameter-level modularity on a *shared* network, not BATON's separate-network + runtime-handover modularity.
- **Long-horizon jobs are hierarchical** — but the winning hierarchy is *high-level planner +
  ONE unified low-level policy*, and the one time someone built a modular low level (LHM's
  baseline) it lost badly.

### Correction after reading the full LHM paper (2026-07-08, verified twice — Explore agent + WebFetch)

The one-line "72% vs 21%" claim above is **much weaker counter-evidence than it first looked**, and
the maintainer's pushback was right on the specifics. Reading arXiv 2508.16943 in full (two independent
verifications, all quotes reproduced across ≥3 fetches):
- **The hierarchical baseline it beat used a NAIVE handoff** — "*five low-level controllers and a
  privileged FSM for oracle-based stage switching*." A hard-coded oracle finite-state machine: no
  blending, no learned transition, no recurrent-state management. That is exactly the naive
  hard-switch BATON is built to improve on — so LHM beating it says **nothing** about whether an
  *engineered* handover beats a single policy.
- **Horizon scaling degrades sharply in LHM's own data** (Table 5, unified student, trained on 2
  objects, tested zero-shot to 5): per-cycle success **90.8 → 76.1 → 61.0 → 38.5 → 20.9%**, all-five
  **18.1%**. The maintainer's core intuition — one policy struggles as the horizon grows — is supported
  *by the counter-paper itself*.
- **Sim-only** (Isaac Gym), 72% train / 63% unseen — not "solved," and never on hardware.
- **LHM is not even a pure monolith**: it distills **two** goal-conditioned teacher policies into the
  student and uses an FSM to pick which teacher supervises — internal specialization during training.
  It criticizes the baseline's run-time oracle FSM while using a training-time FSM itself.

**But keep the honesty symmetric** (do not over-correct into telling the maintainer what he wants): in
LHM's tests the *naive* hierarchy degraded **faster** than the single policy (baselines ≈0% by object
4–5 vs the student's 38/21%), so naive modularity was the worse long-horizon choice *there*. And the
degradation was zero-shot length generalization; a policy *trained* on longer sequences might scale
better. So "one policy can't scale" is **unproven, not disproven** — and crucially, **BATON has not
been shown to scale to long horizons either.** The uncertainty is symmetric.

### Revised, defensible positioning (this is what the paper should claim)

The switching thesis is **not "retired" — it is re-cast as a well-posed OPEN GAP**, which is a
*stronger* and more honest paper than either "switching wins" or "we conceded":

1. **The thesis: does an *engineered* handover between specialists degrade more gracefully over long
   horizons than a single distilled policy?** The only published head-to-head tested a *naive*
   oracle-FSM handoff (which lost, and lost *faster* on long horizons), while single policies
   demonstrably lose ~4× success from 1→5 cycles (LHM Table 5). The regime where a well-engineered
   handoff (reference morph, phase gating, **recurrent-state management**) meets a single policy on a
   long horizon is **unstudied**. BATON is precisely that engineered handoff. That gap is the
   contribution — stated as a hypothesis to test, not a settled win.
   **What claiming it costs us: one experiment.** BATON vs a single distilled policy vs a naive-FSM
   hierarchy, on a task with many cycles, plotting success-vs-horizon. If BATON's curve decays more
   slowly, the edge is real and *demonstrated*; if not, we learned something true. Until that plot
   exists, the edge is a plausible hypothesis, not a result.
2. **The recurrent-hidden-state handover is the concrete mechanism that could make switching survive
   long horizons.** The STAND-ATTRACTOR LOCK + cold-zero fix is unaddressed prior art (all prior
   switching work is feedforward). It is exactly a fix for error-compounding at each boundary — the
   failure mode that sank LHM's naive hierarchy. Lead with it: novel *and* on-thesis.
3. **Resource-constrained framing is a secondary, honest advantage** (train == deploy, no mocap
   corpus, one-burst skill add, per-skill debuggability) — a real pitch for a small lab, not a
   performance claim.
4. The within-skill idle-quality (38× stand stillness) is a minor curiosity, not a headline.

**Maintainer's steer (2026-07-08):** the switching thesis is alive, re-framed as *long-horizon graceful
degradation via an engineered + recurrent handover*. The next real step is the horizon-scaling
experiment in (1) — that is what converts the argument from "plausible" to "shown." The original
thesis text at the top of this doc is preserved for provenance; this re-cast version supersedes it.
