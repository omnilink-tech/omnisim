# OmniSim Policy Training — the flagship method

**This is OmniSim's canonical way to make robot policies** (promoted from
`projects/policies/research/` on 2026-07-03 after the method was validated end-to-end on the
Unitree G1: live-verified durable walking, WBMATCH 0.913 vs an owner-approved reference, zero
sim-to-deploy gap by construction).

> ⚖️ **THE TWO-RULERS DOCTRINE (2026-07-11 — read before claiming ANY motion works).** Every
> champion must pass BOTH: the motion's **kinematic ruler** (trajectory: heading, placement,
> end state) AND the **legitimacy ruler**
> ([`verify_motion_legitimacy.py`](verify_motion_legitimacy.py) — who does the work, what
> touches the world, is it self-supporting). A stair champion once passed every kinematic gate
> while the crane carried its lean (77.5 N·m sustained pitch torque, 77% of climb ticks) and
> its knees pressed the treads. Two standing trainer rules fall out of it: **(1) the crane
> must graduate** (`HARNESS_GRAD_SURV` < 1.0 — an unreachable bar means PPO learns to lean on
> the springs; the trainer now warns loudly), and **(2) audit the constraints before blaming
> the learner** (a "weird" learned strategy is usually the only one the constraints allowed —
> the corridor-torque law). Shaping helper: `W_KNEE_LOW` penalizes knee-support postures.
> Full doctrine: [docs/developer/motion-legitimacy.md](../../../docs/developer/motion-legitimacy.md).

## The method in one paragraph

Train **inside OmniSim's own physics** (the Newton/mujoco-warp engine the deploy runs — bit-exact
`train == deploy`), batched on GPU (K≈4096 worlds, ~200k steps/s on a laptop 5070 Ti), with a
**ghost**: a phase-indexed reference gait recorded from a real walking policy in this same engine
(record → phase-fold → harmonic-smooth). The policy's leg actions are **corridors** around the
ghost (`ghost(phase) ± GHOST_RESIDUAL`) so style is structural, balance is learned. Score with
**WBMATCH** (legs/arms/attitude/speed vs the reference the *eye* compares against — pin it with
`GHOST_METRIC_JSON`). Change gaits with **GHOST-MORPH** (`GHOST_MORPH_JSON` + `MORPH_ITERS`):
references interpolate over training — snap swaps of a mastered reference collapse a warm-started
walker; morphs carry it at survival ~1.0.

**Sibling method — BATON** (runtime switching between the specialists this trainer produces):
[docs/developer/policy-switching.md](../../../docs/developer/policy-switching.md). Field-positioning
settled 2026-07-08: BATON's edge over one-policy universal trackers is a **well-posed open gap**
(does an *engineered* handover — morph + phase-gate + **recurrent hidden-state management** — degrade
more gracefully over long horizons than a single distilled policy?), **not** a demonstrated win —
proof pending a success-vs-horizon experiment (`baton_metrics.py`). Read BATON's canonical
"Where BATON stands vs the field" section before claiming any BATON edge.

**Packaged as the SKILL LIBRARY** — [`../skills/`](../skills/) ·
[docs/developer/skill-library.md](../../../docs/developer/skill-library.md). This trainer produces
the ghosts + champions; the skill library binds each into a versioned manifest (ghost + validator
verdict + deploy env + checkpoint + provenance) and composes them with BATON. `skill_lib.py train
<skill>` assembles a training launch from a skill's manifest, and `freeze <skill>` records the exact
env of a run back into it. **To make a new skill end-to-end, drive it from `skills/`.**

## Files

| file | role |
|---|---|
| `g1_walk_recipe.py` | THE trainer + live-deploy hook (`:g1_walk_recipe_step` / `:g1_walk_recipe_deploy`) |
| `run_walk_rl.sh` | **THE canonical launcher** — carries the complete proven engine env (GROUND_MU/KE/KD/INERTIA/SUBSTEPS/MJWARP + stand spec). NEVER hand-assemble this env: three of its variables compile the *physics*; omitting them silently degrades a champion to surv 0.53. |
| `g1_walk_mpc.py`, `g1_walk_ppo.py`, `g1_residual_inengine.py`, `g1_walk_free.py` | engine bridge / PPO scaffolding / logging / whole-body maps |
| `runs/` | checkpoints (dir gitignored; champions force-added at milestones). ⭐ **`wr_decent_walker.pt` = THE FLAGSHIP champion** (owner-designated 2026-07-06: LSTM + REF_OBS foresight on the λ=0.9 puppet, ghost-attitude harness; **WBMATCH4 0.868** honest shape-only ruler, exam-verified vs `ghost_official_full_v3_lut` — natural thigh-clearing arms; demo: [`../worlds/run_g1_decent_walker.ps1`](../worlds/run_g1_decent_walker.ps1)). Previous-era champions (WBMATCH2 ruler, MLP, bare-robot): `wr_v13_it250.pt` (0.933 vs ghost v14, live 32.4 m), `wr_v11_it200.pt` (functional swing), `wr_showpiece.pt` (durability 45.6 m), `wr_calm_champion.pt` (old style ruler), `wr_vc9_it100.pt` (walk↔stand). NOTE the era split: WBMATCH4-era policies train with `POLICY_ARCH=lstm REF_OBS=1` + harness envs and their obs dim differs — a champion only replays under its own era env (see the flagship script for the full set). |
| `verify_walkstop.py` | **automated PASS/FAIL verdict on deploy telemetry — run BEFORE showing any live demo** (owner rule 2026-07-04: no GUI until PASS) |
| `ghost_doctor.py` | **THE FRONT DOOR for any new ghost** (owner design 2026-07-05): 4-tier prescriptive classifier calibrated from the robot's achieved-recording library. PASS / TARGET(burst-verify) / FAIL + computed fixes. |
| `ghost_polish.py` | fold-recording → clean reference: bilateral symmetrization (mirror-sign map) + harmonic smoothing + URDF clip + jerk report |
| `build_keypoints.py` | `--links`: FK link-position tables (`lp_lut`) for the WBMATCH2 links component; `--elbow` for luts without a wb table |
| `ablation.py`, `deploy_curve.py`, `exp_ledger.py` | evaluation toolkit (deploy-prediction gate: judge on `DEPLOY-EVAL`, never a training metric) |

Ghost references + the hologram controller live in
[`projects/policies/controllers/g1_ghost/`](../controllers/g1_ghost/).

## THE demo (ghost hologram beside the real robot — current champion)

```bash
bash projects/policies/training/run_walk_rl.sh 130 ghost_demo deploy gui \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  WALK_WORLD=projects/policies/worlds/g1_walk_ghost_wide.wbt WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.75 STAND_POSE=unitree \
  PPO_HID=256 GHOST_LUT_JSON=$OMNISIM_HOME/projects/policies/controllers/g1_ghost/ghost_hop1v11_lut.json \
  G1_GHOST_LUT=$OMNISIM_HOME/projects/policies/controllers/g1_ghost/ghost_v14_lut.json \
  G1_GHOST_LOCK=clock GHOST_RESIDUAL=0.11 GHOST_RESIDUAL_LAT=0.28 ARM_RESIDUAL=0.10 ARM_SWING_A=0.20 \
  ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10 SHRY_TARGET=0.0 SHRY_RESIDUAL=0.10 \
  VX_MAX=0.45 WALK_MJW_RESET=0 WALK_WARM_TICKS=30 OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1 \
  RES_POLICY=$OMNISIM_HOME/projects/policies/training/runs/wr_v13_it250.pt
```

The complete ghost design loop (five owner corrections → ceiling 0.784→0.934, all measured):
**idealize → `ghost_doctor.py` gates → burst-prove (reward-side refs; the control corridor
NEVER moves) → `REC_FOLD` re-record → `ghost_polish.py` → owner previews → repeat.**
Laws + mechanisms: [docs/developer/ghost-design-rules.md](../../../docs/developer/ghost-design-rules.md).

⚠ **Two separate ghost envs — set BOTH to the same lut**: `GHOST_LUT_JSON` feeds the *policy's*
corridor; `G1_GHOST_LUT` feeds the *hologram* controller. If `G1_GHOST_LUT` is unset the hologram
silently replays the legacy hand-designed gait — the shadow on screen is then NOT the reference
the policy mimics (owner-caught, 2026-07-04). For the flat-world long walk (45+ m), swap in
`WALK_WORLD=projects/policies/worlds/g1_walk_orig.wbt` and drop the two `G1_GHOST_*` envs. The
ghost-arena floor is 20 m — the robot walks its full length and steps off the far edge; that edge
drop is not a balance fall. **Verify headless first** (same command with `deploy headless` and a
telemetry check) — no demo without a numerical PASS.

## Quick start (G1)

```bash
# evaluate the flagship walker (fresh-process eval needs no training loop)
bash projects/policies/training/run_walk_rl.sh 900 myeval train headless \
  WALK_WORLD=projects/policies/worlds/g1_walk_orig.wbt WHOLE_BODY=1 STAND_SEED=1 STAND_Z=0.75 STAND_POSE=unitree \
  PPO_NENV=2048 PPO_HID=256 EVAL_ONLY=1 EVAL_H=1500 EVAL_IC=0.02 OBS_NOISE=0.01 MOTOR_RAND=0.03 \
  GHOST_LUT_JSON=$OMNISIM_HOME/projects/policies/controllers/g1_ghost/ghost_unitree_lut.json \
  GHOST_METRIC_JSON=$OMNISIM_HOME/projects/policies/controllers/g1_ghost/ghost_v3c_lut.json \
  GHOST_RESIDUAL=0.11 GHOST_RESIDUAL_LAT=0.28 ARM_RESIDUAL=0.10 W_ARMGHOST=2.0 ARM_SWING_A=0.3 \
  VX_START=0.45 VX_MAX=0.45 VX_CURR_ITERS=1 \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step \
  RES_POLICY=$OMNISIM_HOME/projects/policies/training/runs/wr_calm_champion.pt

# live GUI side-by-side with the ghost hologram
bash projects/policies/training/run_walk_rl.sh 1800 demo deploy gui \
  WALK_WORLD=projects/policies/worlds/g1_walk_ghost2.wbt ... \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy \
  G1_GHOST_LUT=.../ghost_v3c_lut.json G1_GHOST_LOCK=clock
```

## ⚠ Trainer selection — the launch trap

`run_walk_rl.sh`'s **default** `OMNISIM_INENGINE_PYMOD` is the *legacy ES residual trainer*
(`g1_walk_residual_inengine`, pop=16, 144 params — kept for the original probe/deploy lanes).
**Every recipe/Shadowing run must select the recipe trainer explicitly**, as in the examples
above:

```
OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step
```

If you pass recipe-only envs (`GHOST_LUT_JSON`, `GHOST_SEQ`, `GHOST_RESIDUAL`, …) without it,
the legacy trainer runs and **silently ignores all of them** — the robot falls within seconds
and the ES grinds on the corpse for your whole budget. Tell-tales in `<tag>_rl.txt`:

- wrong trainer: `TRAIN start pop=16 sigma=... N_PARAM=144`
- right trainer: `GHOST=RECORDED ...`, `GHOST-SEQ: timed sequence mode ...`, `WALK-GPU it=...`

The launcher now **fails fast** on this mismatch (bypass: `ALLOW_MISMATCHED_PYMOD=1`).

## Run lifecycle — status heartbeat + watchdog

The in-engine trainer cannot stop the simulator itself, so historically a finished run
**idled silently** until its wall budget expired, and a stalled simulator looked identical
to a training one. Structural fix (2026-07-04):

- the recipe trainer writes `<RES_LOG>.status` — `{"state": "TRAINING"|"DONE", "it", "iters",
  "ts"}` — at every log interval and at every terminal point (train complete, eval-only,
  feasibility map);
- `run_walk_rl.sh` (headless) watches it: **DONE → the sim tree is stopped immediately**
  (no idle tail); **no heartbeat for `STALL_S` (default 900 s) → the sim tree is killed and
  the launcher exits 3 loudly** (same for a silent startup after `STARTUP_S`).

To check what a run is doing, read the status file — not the process list: a live
`omnisim-bin` proves nothing.


## Timed sequences (dances, routines) — GHOST_SEQ

A sequence ghost (from `seq_ghost_retarget.py`, carrying `root_lut` + `seq: true`) trains with
the same corridor machinery plus spawn-on-reference, a time-varying command from the routine's
own root trajectory, and reference-state initialization. Canonical launch (the 2026-07-04 salsa
campaign):

```bash
bash projects/policies/training/run_walk_rl.sh 14400 wr_salsa1 train headless \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step \
  WALK_WORLD=projects/policies/worlds/g1_walk_orig.wbt \
  GHOST_SEQ=1 GHOST_RESIDUAL=0.11 GHOST_RESIDUAL_LAT=0.28 ARM_RESIDUAL=0.2 \
  W_ATTGHOST=4 W_VSHORT=0 Z_TGT=0.73 \
  PPO_ITERS=2500 EVAL_EVERY=100 EVAL_H=1100 CKPT_EVERY=100 \
```

Notes for sequence runs:
- `Z_TGT` = the ghost's own median `root_lut` z (not the walking default 0.72).
- `EVAL_H` must cover at least one full routine from the top: `cycle_s / 0.008`
  (the trainer tick is 8 ms -- sizing it with 16 ms silently examines only HALF the
  routine; a walk-stop-walk eval once reported 44% 'full completions' of a horizon
  that ended mid-stand).
- The trainer's arm corridor reads a 2-column `arm_lut` (shoulder pitches); derive it from
  `wb_lut` if the retarget tool didn't emit it.
- Reading the logs: `neps` in `WALK-GPU` lines is a PER-10-ITERATION window (mean episode
  length = `10·K·T/neps` steps) — misreading it 10× has caused phantom bug hunts. Judge
  per-state health on the `epret` slope, and the routine itself ONLY on `DEPLOY-EVAL`
  `surv` from the top.
- If survival plateaus with healthy per-state stats, map WHICH beats are infeasible with
  `SEQ_EVAL_MAP=1 EVAL_ONLY=1` (per-beat survival from on-reference starts), then alter the
  ghost there — never train harder against an un-robotic reference (the Charleston lesson).

Full recipe, deploy env, and the debugging war stories:
[docs/developer/g1-walk-recipe.md](../../../docs/developer/g1-walk-recipe.md).

## The load-bearing rules (each one paid for in GPU-hours)

**Formal doctrine + pre-training validator:** [docs/developer/ghost-design-rules.md](../../../docs/developer/ghost-design-rules.md) -- run `ghost_validator.py <ghost.json> --baseline <recording.json>` BEFORE any training; it reproduces the 2026-07-03 campaign outcomes (4 collapses, 2 successes) from pure math in <1 s.

1. **Launch only via `run_walk_rl.sh`** — the engine env is part of the physics.
2. **Ghost-first**: design the reference, preview it solo to the owner, get sign-off, THEN train.
3. **References must be achievable AND self-consistent** (a recorded gait is; a hand-mix of regimes
   is not: narrow stance needs *more* sway; straight elbows need *smaller* shoulder amplitude).
4. **Morph, never snap** a mastered reference; when a channel changes regime, **free the coupled
   channels** and re-record the achieved behavior as the new reference.
5. **Judge on `DEPLOY-EVAL` / live runs, keep the best checkpoint** (PPO is non-monotonic; apparent
   plateaus break).
6. **When a proven policy suddenly measures broken, audit the measurement first** (a fresh-process
   eval once reported the champion at surv 0.038 over a zeroed command vector).

## Roadmap (owner direction, 2026-07-03)

- **H1 ghost** via record-replay of the proven official H1 re-host → same corridor recipe.
- **Other humanoids** → generalize the leg-map/spec layer (the knobs are already robot-agnostic).
- **Dancing ghosts** (G1 + H1): expressive-motion regime — ghost-anchored exploration, achievability
  checks, morphs from the walking champions.
- Straight-elbow polish (symmetrize the self-discovered swing via soft reward) + durability
  hardening (longer horizons, push-DR).
