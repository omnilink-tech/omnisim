# OmniSim Policy Skills

A **skill** is a *named, composable behaviour* a robot can perform — "walk",
"turn in place", "carry a box", "stand", "climb stairs", "balance on two legs while
moving the arms". This folder is the **catalogue + contract + runner** for them: it
binds each skill's ghost, validator verdict, deploy config, champion checkpoint, and
provenance into **one versioned manifest**, and gives the whole
[Shadowing](../../../docs/developer/shadowing.md) + [BATON](../../../docs/developer/policy-switching.md)
pipeline a single front door.

Full design + pipeline reference: **[docs/developer/skill-library.md](../../../docs/developer/skill-library.md).**

```
skills/
  ghost_lut.py      # canonical typed ghost-LUT schema + loader/validator (round-trips every ghost)
  manifest.py       # Profile / SkillManifest / SequenceManifest + deploy-env assembly
  skill_lib.py      # THE CLI  (list / show / validate / preview / train / run / sequence / verify-demos)
  run_skill.py      # back-compat shim -> skill_lib.py
  registry.json     # human-readable catalogue (runtime source of truth = filesystem discovery)
  profiles/         # runtime env identical across every demo (g1_shadow_deploy)
  humanoid/<skill>/ # skill.json + (deterministic skills) README
  sequences/        # BATON demos: box_delivery, turn_solo, walk_turn_walk
```

## Quick start

```bash
cd projects/policies/skills

python skill_lib.py list                       # every skill + BATON sequence
python skill_lib.py show walk                  # a skill's full manifest + ghost summary
python skill_lib.py validate                   # validate all manifests + round-trip all ghost luts
python skill_lib.py verify-demos               # PROVE the manifests reproduce demos/run_*.sh

python skill_lib.py preview walk               # the ghost hologram (design -> show -> agree)
python skill_lib.py sequence box_delivery      # run a BATON demo (walk->stand->carry->place)
python skill_lib.py run turn_in_place          # one skill solo
python skill_lib.py sequence box_delivery --dry-run   # print the launch command, don't run

python skill_lib.py ghost --all                # validate every ghost lut in the repo

python skill_lib.py handover box_delivery      # per-edge warm/cold plan (attractor-derived)
python skill_lib.py blendable walk carry_box   # can two skills element-wise blend? what's needed?
python skill_lib.py adapt turn_in_place --to-nb 64 --out /tmp/turn64.json   # cross-cadence resample
python skill_lib.py freeze walk                # promote a training run's env lock into the manifest
python skill_lib.py run go2_walk               # cross-robot: launches the baked-controller world
```

Any launch command takes `--dry-run` to print the assembled `run_walk_rl.sh` command
instead of executing it.

## What makes something a skill (the contract)

Every skill is one `skill.json` manifest (schema 2) declaring:

1. **Identity + regime** — `kind` (`rl` | `deterministic`), `method`
   (`shadowing` | `deterministic-overlay`), `status`, and `motion_class`
   (`cyclic` | `sequence` | `static`). `motion_class` is the BATON axis: cyclic skills
   blend element-wise, sequence skills swap in solo.
2. **Its ghost** (`ghost.lut` + provenance + validator verdict + preview world +
   owner sign-off) — the achievable reference it shadows. Ghost feasibility is the
   Shadowing bottleneck; verify **before** training.
3. **Its policy** (`policy.checkpoint` + obs/action spec + arch) — the frozen champion.
4. **Its BATON role** (`baton.blend` / `mode` / `vx` / `attractor` / `handover_in`) —
   how the sequencer drives it and hands over into it.
5. **Its deploy env** (`deploy.primary_env` / `specialist` / `solo_swap_env`) — the
   corridors/knobs the runner emits depending on the skill's role in a sequence.
6. **Success criteria + a verification record** — the measured result (0 falls,
   duration, upright), so a skill's status is earned, not asserted.

Deterministic skills additionally follow the **balance-core-underneath** contract:
they ride on the stiff-squat stand + reactive balancers and declare **effector
arbitration** (hand-off vs superposition) — see
[`humanoid/arm_motion/README.md`](humanoid/arm_motion/README.md).

## The two regimes (why the libraries aren't symmetric)

| | humanoid | quadruped |
|---|---|---|
| stand / posture / balance | **deterministic** (verified) | **deterministic** |
| arm / hand motion while balancing | **deterministic** (`arm_motion`) | n/a |
| locomotion (walk / turn / carry / climb) | **RL / Shadowing** | **RL** |

The manifest *interface* is shared; the *implementation* differs by `kind`. Don't force
one regime across both.

## Adding a skill

`design ghost → validate → preview (owner sign-off) → train → verify → write skill.json
→ register → sequence`. Step-by-step in
[docs/developer/skill-library.md § Adding a skill](../../../docs/developer/skill-library.md).
`python skill_lib.py index` regenerates `registry.json` from the filesystem.

## Verification

`skill_lib.py verify-demos` is the regression guard: it asserts that the env bundle the
manifests assemble is byte-identical (key-for-key) to what the reproduced
`demos/run_*.sh` scripts pass to `run_walk_rl.sh`. Green today for all three sequences
(30 / 35 / 39 keys). The scripts remain the ground truth — the library is additive.
