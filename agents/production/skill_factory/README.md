# Skill Factory — the agent-driven skill pipeline, end to end

A demo orchestrator for showing an audience what OmniSim's skill pipeline looks
like when an **AI agent drives it** instead of a human typing commands. One run
walks a named robot skill through the whole factory loop with live narration:

```
DESIGN  ->  VALIDATE  ->  TRAIN  ->  CERTIFY  ->  REGISTER  ->  COMPOSE
(ghost)     (gates)      (GPU)      (the bar)    (manifest)    (BATON)
```

That is the skill-marketplace-shaped loop: skills are **authored** as machine-checkable
references, **screened** before any GPU spend, **trained** in the deploy engine
(train == deploy bit-exact), **certified** against a published numeric bar,
**registered** as one versioned manifest binding all five artifacts (ghost lut,
validator verdict, deploy env, champion checkpoint, provenance), and finally
**composed** with other skills into a BATON sequence. Every gate is a number in
an artifact — the agent cannot talk a skill past one, and the demo says so.

## Run it

```bash
# The flagship lane: go2_turn, the skill that went design -> certified TODAY (2026-07-17)
python agents/production/skill_factory/skill_factory_agent.py --robot go2 --replay

# The humanoid lane: g1_walk / box_delivery -- same pipeline shape, with the
# mandatory balance-harness disclosure baked into the narration
python agents/production/skill_factory/skill_factory_agent.py --robot g1 --replay
```

Both are **fully offline, deterministic, CPU-only** and finish in seconds.
"Offline" does not mean "canned": the ghost validator and the skill-library
registry checks are cheap enough to run for real, so they run for real, every
time, and their verbatim output is what the audience reads.

### What is live vs recorded

| stage | what actually runs / is read |
|---|---|
| DESIGN | `skill.json` + the ghost lut itself, incl. the builder-gate numbers stamped in the lut (live) |
| VALIDATE | `projects/policies/training/ghost_validator.py` **executed live** on the lut (CPU, sub-second); verdict cross-checked against the design-time stamp. The G1 lane runs it twice — the second pass with the shipped corridor, so T3's "the crane will pay the difference" FAIL is shown as the machine-checkable form of the harness disclosure |
| TRAIN | `--replay` (default): quotes the real campaign logs from `_scratch/s3_results/campaigns/go2_turn/` when present (they are, today), else clearly-labelled recorded constants. `--live`: launches the real trainer (below) |
| CERTIFY | the champion's `launch_env.txt` + `skill.json`, read live, checked against the published bar (never_fell >= 99%, \|vx\| <= 0.10 m/s for the turn) |
| REGISTER | `skill_lib.py validate` **executed live** (every manifest + sequence + all ~118 ghost luts), plus an on-disk existence/size check of the five bound artifacts (including the `.onnx.data` externalized-weights sidecar check) |
| COMPOSE | `skill_lib.py sequence <name> --dry-run` **executed live**; the specialist wiring is parsed out of the assembled env bundle |

## `--live` (GPU sessions only)

`--live --robot go2` assembles and **launches** the real in-engine trainer:

```
bash projects/policies/training/run_quad_walk_rl.sh 0 go2_turn_factory train headless \
  QUAD_ROBOT=go2 QUAD_GHOST=projects/policies/ghosts/go2/go2_turn_ghost_lut.json \
  QUAD_GHOST_FF=1 QUAD_W_GMATCH=2.0 QUAD_RES_SCALE=0.15 QUAD_VX_TARGET=0.0 QUAD_YAW=0.0 \
  QUAD_ENVS=16384 QUAD_FAST_RESET=1 MPC_NJMAX=64 MPC_NCONMAX=64 QUAD_ITERS=400 \
  QUAD_WARMSTART=projects/policies/research/inference/policies/gpu_go2_shadow_r2_main/policy.pt \
  RES_POLICY=projects/policies/research/inference/policies/gpu_go2_turn_main/policy.pt
```

Requirements: a CUDA GPU (4090-class comfortable), the Newton/mujoco_warp stack
reachable by the engine's interpreter, and the discipline in
`projects/policies/skills/quadruped/go2_turn/TRAIN_PLAN.md`. One 400-iter leg is
~79 M env-steps — about two minutes at **727,583 env-steps/s**, which is the rate
measured for *exactly* this config (RunPod **RTX 4090**, `QUAD_ENVS=16384` +
`QUAD_FAST_RESET=1` + `MPC_NJMAX/NCONMAX=64`, cumulative over a full 400-iteration
run; one env-step = one 16 ms control step = 8 physics
substeps). **Budget from your own box, not from that line** — the same in-engine
trainer measures 10,228 env-steps/s at `QUAD_ENVS=256` on a laptop RTX 3060
(OmniBench lane-2 tier C), so the rate is a property of (GPU, K, flags), not of the
pipeline. Assert
the eval block and the Newton sidecar, never the exit code. `--live --robot g1`
is refused by design (the G1 champion's train env is `reconstructed`, not a
certified re-train bundle; the agent prints the dry-run command instead).

## Honest status (as of 2026-07-17, evening)

- **go2_turn** went design → `ghost_validator` PASS → trained → **certified at
  the in-engine eval bar today**: the zero-hook ladder ended at 97.5% (under the
  99% bar, and the campaign log says FAIL), the designed WZ_FIXED escalation
  (TRAIN_PLAN §4 / `campaign_go2_turn_cert.sh`) closed it at **never_fell 99.2%,
  vx −0.003 m/s** (`gpu_go2_turn_main/launch_env.txt`). The manifest status is
  still **`open`** and the `go2_walk_turn_walk` sequence still **`draft`**: the
  final rung — the live BATON walk→turn→walk exam ("ONNX loaded:" on both
  policies, sustained yaw ≥ 0.5 rad/s, |xy drift| < 0.3 m, 0 falls) — has not
  run. Both deploy hooks are in-tree (gait clock `8dcb293d`, wz-obs `5b64a27d`).
  Note: `skill.json`'s prose still describes the pre-cert 97.5% champion — the
  training lane is mid-update; the champion's own `launch_env.txt` is current.
- **g1_walk / box_delivery** are `verified` — **on the weight-bearing balance
  harness** (λ=0.9 carried, `HARNESS_KZ=2000`, up to ~700 N + ±350 N·m). That
  disclosure is hard-wired into the CERTIFY narration and is non-negotiable
  house policy; a durable free-standing humanoid walk remains open.

## Narration architecture (LLM-ready, not LLM-dependent)

All audience-facing text flows through one `Narrator` object that renders
deterministic templates — no network, no keys, runs on a plane. `Narrator.restyle`
is the hook where an OmniLink LLM route (`OMNI_KEY` +
`agents/production/_lib`'s `OmniLinkAgentRunner`, the house pattern used by
`husky_maze` et al.) can later rewrite the same facts in a live voice. The
facts and gate verdicts are computed *before* the hook sees the text, so a
styling model can never change a verdict — only the phrasing.
