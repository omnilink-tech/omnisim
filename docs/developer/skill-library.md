# The OmniSim Skill Library

**A skill is a named, composable robot behaviour** — walk, turn-in-place, carry-box,
stand, climb-stairs — produced by the [Shadowing](shadowing.md) method (or, for static
balance/posture, by the deterministic stand core) and packaged as a **versioned
manifest** so it is discoverable, runnable, verifiable, and sequenceable across the
whole simulator. This doc is the canonical reference for the library's model,
pipeline, and CLI. Pure algorithms and contracts live in
[`omnisim.policy`](../../omnisim/policy/); robot assets and compatibility launchers remain in
[`projects/policies/`](../../projects/policies/).

If a doc and the code disagree, the code wins. `python -m omnisim policy audit` is the release gate;
`verify-demos` is its config-equivalence component for reproduced demo bundles.

---

## Why this exists — the five-scattered-artifacts problem

Before the library, a "skill" was spread across **five disconnected places**, with
nothing tying them together:

| # | artifact | lived in |
|---|---|---|
| 1 | the **ghost** (achievable reference) | a `ghost_*_lut.json` under `ghosts/g1/` |
| 2 | the **validator verdict** (is the ghost feasible?) | ephemeral stdout from `ghost_validator.py` |
| 3 | the **deploy env bundle** (corridors, harness gains, course) | inline in a hand-written `demos/run_*.sh` |
| 4 | the **champion checkpoint** | `training/runs/wr_*.pt` (uncommitted, ~1800 files) |
| 5 | the **provenance / lineage** | engineering notes + commit messages |

The cost was concrete: reproducing the box-delivery demo took hours because the exact
launch config lived only in an archived run's banners, and "a demo = launch config,
not code" became a hard-won law. The library **binds all five into one manifest** and
reuses the already-generic trainer / corridor / WBMATCH / BATON / deploy stack
**unchanged**. It reimplements no engine — it factors the per-skill and per-demo config
out of shell scripts into data.

---

## The model — three artifact types

```
projects/policies/skills/
  ghost_lut.py            # canonical typed ghost-LUT schema + loader/validator
  manifest.py             # Profile / SkillManifest / SequenceManifest + env assembly
  skill_lib.py            # implements the pipeline verbs (`omnisim policy` delegates here)
  skill_verbs.py          # the shared verb table both front doors build their parser from
  run_skill.py            # back-compat shim -> skill_lib.py
  registry.json           # human-readable catalogue (runtime source of truth = filesystem discovery)
  profiles/
    g1_shadow_deploy.json # runtime env identical across every demo
  humanoid/
    g1_walk/skill.json       g1_carry_box/skill.json   g1_stand_rl/skill.json
    g1_turn_in_place/skill.json   g1_climb_stairs/skill.json
    balance_two_legs/skill.json   g1_arm_motion/skill.json   # deterministic overlays
  sequences/
    box_delivery.json   turn_solo.json   walk_turn_walk.json
```

### 1. Profile (`profiles/<name>.json`)
The runtime env that is **identical across every demo** — the recipe deploy pymod, the
LSTM+REF_OBS whole-body obs family, the crane/harness gains. Extracted from the common
core of the three demo scripts. Skill- and sequence-specific env layers on top.

### 2. SkillManifest (`<class>/<skill>/skill.json`, schema 2)
The standard unit. Binds the five artifacts:

- **identity** — `name`, `class`, `robots`, `kind` (`rl` | `deterministic`),
  `method` (`shadowing` | `deterministic-overlay`), `status`
  (`verified` | `experimental` | `open`), `motion_class`
  (`cyclic` | `sequence` | `static`).
- **`ghost`** — `lut` path, `provenance` (`recorded` | `solved` | `retargeted`),
  `validator` verdict, `preview_world`, `owner_signoff`.
- **`policy`** — `checkpoint`, `arch`, `hidden`, `obs {family, ref_obs_k, ref_obs_wb, dim}`,
  `action {dim, space, decode}`.
- **`baton`** — `blend` (`cyclic` = element-wise blendable | `solo_swap` = context swap),
  `mode` (the short name the arbiter switches to), `vx`, `attractor`
  (`locomotion` | `stand`), `handover_in` (`warm` | `cold`), `nb`.
- **`train`** — `trainer` pymod, `recipe_env`, `train_world`, `status`, `notes`.
- **`deploy`** — `primary_env` (emitted when this skill is the BATON primary) /
  `specialist` (emitted as a `BATON_SPECIALISTS` entry) / `solo_swap_env` (the
  `BATON_TURN_*` block).
- **`verification`** — measured result, date, engine.

### 3. SequenceManifest (`sequences/<name>.json`, schema 2)
A BATON demo, referencing skills by name: `{profile, world, primary, skills[],
arbiter, env}`. Replaces a hand-written `demos/run_*.sh`.

`motion_class` is the load-bearing axis: **cyclic** skills share one 120-dim obs family
and blend element-wise (`BATON_SPECIALISTS`); **sequence** skills (turn, climb) run a
different cadence/obs-width and so are swapped in solo (`BATON_TURN_*` context swap).
`ghost_lut.py` derives `motion_class` from the lut (`seq` flag, `vx`, constant track),
so it and the manifest agree.

---

## The pipeline — one front door per stage

```
 design ─▶ validate ─▶ preview ─▶ train ─▶ run / verify ─▶ register ─▶ sequence
 ghost      gates      hologram   recipe    deploy          manifest    BATON demo
```

| stage | command | what it does |
|---|---|---|
| **design** | `design_*.py` / `build_*.py` (existing) → a `ghost_*_lut.json` | author the achievable reference (record / solve / retarget) |
| **validate (structure)** | `python -m omnisim policy ghost <lut>` | typed schema round-trip: shapes, widths, motion class |
| **validate (feasibility)** | `python training/ghost_validator.py <lut> --baseline …` | the calibrated 7-rule physical gate (COM, limits, edit-envelope) |
| **preview** | `python -m omnisim policy preview <skill>` | the `g1_ghost` hologram beside the robot — **design → show maintainer → agree, before training** |
| **train** | `python -m omnisim policy train <skill>` | assemble the recipe trainer env from the manifest, launch `run_walk_rl.sh … train` |
| **run** | `python -m omnisim policy run <skill>` | deploy one skill solo |
| **verify** | `python -m omnisim policy verify-demos` | prove the assembled env == the reproduced shell scripts (regression guard) |
| **audit** | `python -m omnisim policy audit` | run legacy and public-core release gates |
| **benchmark** | `python -m omnisim policy benchmark ...` | run or score thresholded end-to-end acceptance cases |
| **sequence** | `python -m omnisim policy sequence <name>` | run a BATON demo (walk→turn→walk, box delivery, …) |
| **graph / IR** | `python -m omnisim policy graph <seq>` / `ir <skill>` | inspect typed composition and robot-independent intent |
| **promotion** | `python -m omnisim policy promote sequence <name>` | content-address all inputs and evaluate evidence tier |
| **freeze** | `python -m omnisim policy freeze <skill>` | promote a run's captured env lock into the manifest (`train.status: frozen`) |
| **adapt** | `python -m omnisim policy adapt <skill> --to-nb N` | resample a skill's ghost to a target cadence (cross-cadence blend) |
| **blendable** | `python -m omnisim policy blendable <a> <b>` | report whether two skills can element-wise blend + what's needed |
| **handover** | `python -m omnisim policy handover <seq>` | show the resolved per-edge warm/cold plan |

The **load-bearing operation** is `manifest.assemble_deploy_env(sequence)`: it merges
`profile → primary skill → cyclic specialists (BATON_SPECIALISTS) → solo-swap skills
(BATON_TURN_*) → arbiter (BATON_COURSE | WALK_SCHEDULE) → sequence overrides →
WALK_WORLD` into the exact `KEY=VALUE` bundle the demo scripts pass to
`run_walk_rl.sh`.

> ⚠️ **What `verify-demos` actually proves — state it precisely.** It asserts that the manifest
> and the hand-written script agree **key-for-key on the assembled launch env**: it parses the
> script's `KEY=VALUE` bundle, assembles the manifest's, and requires the two to be equal (no key
> only-in-manifest, no key only-in-script, no differing value). It is **NOT** a byte-for-byte file
> comparison, and it does not run the simulator — it is a config-equivalence regression guard.
> ⛔ Do not repeat the "byte-for-byte" phrasing; it overclaims what the check does.

> **Additive, non-destructive.** The hand-written `demos/run_*.sh` scripts stay as the
> ground truth; the library reproduces them. `verify-demos` is green today for **all five
> reproduced sequences**; the sixth sequence is manifest-only. Only once a manifest path is
> proven equivalent should a script be retired.

### The policy block's release and performance contract

Run `python -m omnisim policy audit` before publishing a policy change.
It fails on duplicate identities, schema or robot incompatibility, invalid versioned ghosts,
demo-env drift, a stale generated registry, or an invalid benchmark specification. Untracked
ghost experiments do not poison this release gate; `ghost --all` remains the exhaustive
workspace audit.

End-to-end acceptance cases live in
[`projects/policies/benchmarks/`](../../projects/policies/benchmarks/). Their JSON thresholds
are applied to machine-readable `BATON-CYCLE` verdicts, fall telemetry, duration, and required
physical events. A verified case must record the machine, venue, engine, support configuration,
date, and measured result. The initial physical box-delivery benchmark passes at 14/14 segments,
155.0 simulated seconds, minimum pelvis z 0.720 m, and zero fall ticks on machine
`9722d23d12a3`; it uses the G1 lambda=0.9 weight-bearing balance harness and is not a
free-standing humanoid result. A second, free-standing Go2 walk→turn→walk case exercises the
same scorer across morphology; it remains a candidate until a fresh run can be recorded with
resolvable machine provenance.

---

## Current catalogue (2026-07-19)

| skill | robot | kind | method | status | motion | notes |
|---|---|---|---|---|---|---|
| `g1_walk` | g1 | rl | shadowing | verified | cyclic | the BATON primary; heading-aware; `wr_navigator.pt` |
| `g1_carry_box` | g1 | rl | shadowing | verified | cyclic | 1.5 kg payload; `wr_carrier.pt` |
| `g1_stand_rl` | g1 | rl | shadowing | verified | static | dead-still BATON pause; `wr_stander.pt` |
| `g1_turn_in_place` | g1 | rl | shadowing | experimental | sequence | natural four-footfall 90° policy is the catalog default but still needs end-to-end certification; published turn sequences explicitly pin their older verified assets. All G1 results are on the weight-bearing harness; yaw torque is zero. |
| `g1_walk_backward` | g1 | rl | shadowing | experimental | cyclic | RunPod-trained reverse retreat specialist; catalogued but not routed into the published box demo. |
| `g1_carry_backward` | g1 | rl | shadowing | experimental | cyclic | 1 kg payload reverse specialist; catalogued but not routed into the published box demo. |
| `g1_climb_stairs` | g1 | rl | shadowing | **open** | sequence | the **7 cm climb-ghost** skill: walk-in + ascend a 5-tread staircase (riser 0.07). **OPEN** — plateaus at ~2 of 5 steps; live riser refusal. ⚠️ **Not the same artifact as the shipped stair DEMO** — see the note below. |
| `h1_walk` | h1 | rl | unitree-rehost | verified | cyclic | official H1 re-host; `deploy.run: world`; ghost ready for a native train |
| `balance_two_legs` | g1/h1/valkyrie | deterministic | overlay | verified | static | the balance core (Valkyrie requires the NASA package, which is held from the public snapshot) |
| `g1_arm_motion` | g1 | deterministic | overlay | verified | static | arms in 3-D while balancing |
| `go2_walk` | go2 | rl | residual-rl | verified | cyclic | earlier residual PPO trot; `deploy.run: world`; +86 m |
| `omniquad_walk` | omniquad | rl | residual-rl | verified | cyclic | earlier residual PPO trot (+vc/rough variants); **47.8 m, 0 falls** |

> ⛔ **`g1_climb_stairs` (the SKILL) and the stair-climb DEMO are DIFFERENT ARTIFACTS.** They look like
> a contradiction between this doc and [DEMOS.md](../../DEMOS.md); they are not.
>
> | | the **skill** (`humanoid/climb_stairs/skill.json`) | the **demo** ([`demos/run_climb_stairs.sh`](../../projects/policies/demos/run_climb_stairs.sh)) |
> |---|---|---|
> | reference | a **bespoke climb-ghost** — composed walk→stand→climb, `root_lut` rising with the treads | the **existing WALKING ghost** (`ghost_official_full_v3_lut`), no climb ghost at all |
> | riser | **7 cm** (5 × 0.07 = 0.35 m) | **3 cm** |
> | terrain | stairs batched into the trainer | stairs batched into the trainer (terrain curriculum, shallow risers first) |
> | result | ❌ **OPEN** — plateaus at ~2/5 steps across every trainer lever; live riser refusal | ✅ **full 5-step live climb**, legs-only (`HARNESS_KZ=0`, base z 0.72 → 0.88) |
>
> The demo works precisely *because* it abandoned the bespoke climb-ghost (seven approaches all
> ended with the feet shuffling at the base) and warm-started the existing walker onto stair terrain
> instead. **3 cm is the measured live CEILING** for the stock-foot G1 climbing legs-only — 4 cm gets
> ~2 steps, 5 cm gets 0 (small-foot propulsion wall). The 7 cm climb-ghost *passes* the ghost gates
> (including gate 4 — it is PD-realizable under the crane), and no policy climbs it live:
> **gate-pass ≠ climb.** See [ghost-design-rules.md](ghost-design-rules.md) (gate 4 + the
> corridor-vs-torque law).

`deploy.run` selects how `run <skill>` launches: **recipe** (`run_walk_rl.sh`, the g1
Shadowing skills), **world** (launch the baked-in-controller `.wbt` — re-host /
residual RL), or **powershell** (the deterministic overlays). This is what lets one
runner drive skills across robot classes and training methods.

**Sequences — six; five reproduced launchers are green under `verify-demos`:**

| sequence | chain | status | reproduces |
|---|---|---|---|
| `box_delivery` | physical suction pick → carry arc → zero-command place arrest → release → clearance → forward U-arc → stand | experimental route; latest acceptance PASS 14/14, 155.0 s, min z 0.720, 0 falls | `demos/run_box_delivery.sh` |
| `box_delivery_classic` | walk → stand → pick → carry → place → walk → stand (no corner — the stable baseline) | verified, 0 falls | `demos/run_box_delivery_classic.sh` |
| `walk_turn_walk` | walk → **90° footwork turn** → walk on the new heading | verified, 3/3, 0 falls (`72a7bb19`) | `demos/run_walk_turn_walk.sh` |
| `turn_solo` | settle → 90° footwork turn → decel-stop → hold | experimental | `demos/run_turn_solo.sh` |
| `go2_walk_stand_walk` | quad walk → deterministic stand hold → walk | verified, 0 falls | `scripts/dev/run_go2_baton_deploy.sh` |
| `go2_walk_turn_walk` | quad walk → **turn-in-place ~169°** → walk on the reversed heading | verified 2026-07-17, 3/3, 0 falls | assembled by `skill_lib sequence` |

The four **G1** sequences run on the weight-bearing balance harness (`HARNESS_LAM0=0.9`,
`HARNESS_KZ=2000` — up to ~700 N of upward support, ≈2× the 34 kg G1's weight, plus ±350 N·m of
attitude authority). That is part of the shipped configuration, not a hidden crutch — but the
**rotation** in the turns is the robot's own footwork (`wtz = 0`, zero crane yaw torque, measured
per tick), and the **stair-climb demo** runs `HARNESS_KZ=0` so the legs do all the vertical lift.
The two **Go2** sequences carry **no harness at all** — quadrupeds are free-standing, which makes
`go2_walk_turn_walk` the library's first fully unsupported walk→turn→walk.

---

## Adding a skill

1. **Design the ghost** and get it green:
   `python -m omnisim policy ghost <lut>` (structure) **and**
   `python training/ghost_validator.py <lut> --baseline <recording>` (feasibility).
   Ghost feasibility is the Shadowing bottleneck — see [ghost-design-rules.md](ghost-design-rules.md).
2. **Preview it** and get maintainer sign-off: `python -m omnisim policy preview <skill>`
   (author the manifest with `ghost.preview_world` first). Ghost-first is a hard rule.
3. **Train** it: `python -m omnisim policy train <skill>` (the manifest's `train.recipe_env`
   assembles the recipe trainer launch). Record the champion in `policy.checkpoint`.
4. **Verify** the deploy and record the measured result in `verification`.
5. **Write** `<class>/<skill>/skill.json` (copy a sibling), declaring the `baton`
   role (`cyclic` for a same-cadence gait, `solo_swap` for a different-cadence
   sequence). Add a `registry.json` line (`python -m omnisim policy index` regenerates it).
6. **Sequence** it: add or extend a `sequences/*.json`; run `verify-demos` if it
   reproduces an existing script.

### The two regimes (why humanoid and quadruped libraries aren't symmetric)

Static balance/posture is **deterministic** (the shipped stand core); locomotion and
functional maneuvers are **RL/Shadowing**. The manifest *interface* is shared (both
declare identity, effectors, success criteria, verification); the *implementation*
differs by `kind`. Don't force one regime across both.

---

## Extensions (built) and the in-engine frontier

Four extensions landed at the library layer, each verified offline; the remaining
work for each is an **in-engine** step (a retrain, a deploy-code hook, or real runs)
that must be verified in-sim, not on paper.

1. **Cross-cadence blend adapter** — `adapter.py` phase-resamples any ghost to a target
   nb (`resample_lut`), and `blendable <a> <b>` reports the two blend conditions
   (cadence + obs family) with the exact fix. *Frontier:* the obs half — a 153-dim
   `REF_OBS_WB` turn ref can't feed a 120-dim cyclic net, so the turner must be
   **retrained in the 120-dim leg-REF family**; then only cadence differs and the
   resampler closes it. Until then, sequence skills correctly run via solo context-swap.
2. **Per-edge handover** — `resolve_handover` derives warm/cold **per transition edge**
   from skill attractors (the stand-attractor-lock law as data), and
   `assemble_deploy_env` auto-sets the deploy's `BATON_COLD_HIDDEN` from it (box_delivery
   no longer hand-sets it — it is derived, and `verify-demos` confirms the derivation
   reproduces the proven script). *Frontier:* the deploy still consumes a single global
   knob; a true **per-edge deploy reader** (honor a per-switch plan) is the engine hook
   that makes the paper's mechanism fully live.
3. **Freeze train bundles** — `train` writes an env lock at launch; `freeze <skill>`
   promotes it into the manifest (`train.status: frozen`, verbatim `train.frozen_env`),
   and `train` then reuses the frozen env. *Frontier:* the g1 skills stay
   `reconstructed` until a **real verified training run** populates the lock — freeze is
   the mechanism, not a substitute for training.
4. **Cross-robot** — `h1_walk` (Unitree re-host) and `go2_walk` / `omniquad_walk` (residual
   PPO) prove the manifest + runner span robots and methods via `deploy.run: world`.
   *Frontier:* a **native shadowed H1** (corridor-train the recipe on the already-valid
   `ghost_h1_lut`) and unifying the quadruped generated/`.npz` ghost lineage under one
   `ghost_lut` schema.

⭐ **CLOSED (2026-07-10, `72a7bb19`): turn reliability / the crisp live 90° corner.** The old
"turn reliability and the live-dead heading channel are still open" verdict is **superseded**.
TURN-LOOP replays partial passes of a modular 15°-staircase turn ghost — restarting at the plateau
whose remaining staircase ≈ (remaining angle ÷ the *measured* 0.67 slip gain) — until the **actual**
accumulated heading reaches target, always finishing through the ghost's own end-hold. Verified 3/3:
**90.6–95.6° actual, 0 falls**, and the walker then holds the new heading through a straight leg.
(⛔ Two mid-lut arrest designs were killed by telemetry: freeze-on-plateau recoils −19°;
jump-to-end-hold spins and falls. **Never stop a sequence-ghost mid-lut.**)

Still open: **`ghost_validator` motion-class awareness** (it is walk-cyclic-calibrated;
constant-stand and sequence luts trip its symmetry/clearance checks — a known TODO), the **7 cm
`g1_climb_stairs` skill** (see the catalogue note above), and — at the *thesis* level —
BATON's unrun **success-vs-horizon** experiment ([policy-switching.md](policy-switching.md)):
a working handover is not yet evidence that handover beats a monolith.

---

## See also

- [shadowing.md](shadowing.md) — the method the library packages
- [ghost-design-rules.md](ghost-design-rules.md) — the 7 rules + the feasibility validator
- [policy-switching.md](policy-switching.md) — BATON, the handover protocol
- [step-turn-method.md](step-turn-method.md) — the solo-swap turn skill's method
- [rl-current-state.md](rl-current-state.md) — canonical RL status
