# OmniSim Policy Skills

A **skill** is a *named, composable behaviour* a robot can perform — "walk",
"turn in place", "carry a box", "stand", "climb stairs", "balance on two legs while
moving the arms". This folder is the **asset catalogue + compatibility runner** for them: it
binds each skill's ghost, validator verdict, deploy config, champion checkpoint, and
provenance into **one versioned manifest**, and gives the whole
[Shadowing](../../../docs/developer/shadowing.md) + [BATON](../../../docs/developer/policy-switching.md)
pipeline a single front door.

The installable brain is [`omnisim.policy`](../../../omnisim/policy): morphology-free BATON,
MotionIR, typed skill graphs, benchmark-matrix validation, and content-addressed promotion.
Use `python -m omnisim policy ...` as the stable public CLI. Robot-specific manifests,
ghosts, checkpoints, profiles, worlds, and legacy launchers remain here so existing G1 demos
retain the exact same deployment contract.

Full design + pipeline reference: **[docs/developer/skill-library.md](../../../docs/developer/skill-library.md).**

```
skills/
  ghost_lut.py      # canonical typed ghost-LUT schema + loader/validator (round-trips every ghost)
  manifest.py       # Profile / SkillManifest / SequenceManifest + deploy-env assembly
  skill_lib.py      # THE CLI  (audit / benchmark / list / train / run / sequence)
  run_skill.py      # back-compat shim -> skill_lib.py
  registry.json     # human-readable catalogue (runtime source of truth = filesystem discovery)
  profiles/         # runtime env identical across every demo (g1_shadow_deploy)
  humanoid/<robot>_<skill>/   # skill.json + (deterministic skills) README
  quadruped/<robot>_<skill>/
  sequences/        # BATON demos: box_delivery, turn_solo, walk_turn_walk
```

## Naming: the robot is always named

* **Single-robot skill → `<robot>_<skill>`** — `g1_walk`, `g1_carry_box`, `go2_walk`, `omniquad_walk`.
* **Genuinely multi-robot skill → `<skill>`** — `balance_two_legs` (g1 + h1).
* **The method is a FIELD, not a name** — `method: shadowing | residual-rl | unitree-rehost |
  deterministic-overlay`. Branch on the field. (`go2_walk` and `go2_shadow_walk` are the same
  skill by two methods; the *field* is what says which.)

`skill_lib.py validate` **enforces** this. It matters more than it looks: the humanoid skills used
to be named for the skill alone (`walk`, `carry_box`) with the **G1 as the unnamed default**, while
the quadrupeds were robot-prefixed. In a flat namespace that makes `walk` the G1's forever — an H1
Shadowing walk has no name left to take, and `h1_walk` was already spent on the re-host. **A robot
that is never named is a robot you never generalize away from.**

## Quick start

Run these **from the repo root** -- `python -m omnisim` resolves the package from the
working directory, so a `cd` into this folder breaks every line below with
`No module named omnisim`. `python -m omnisim policy --help` is the live verb list.

```bash
python -m omnisim policy list                  # every skill + BATON sequence
python -m omnisim policy show g1_walk          # a skill's full manifest + ghost summary
python -m omnisim policy audit                 # release gate, including public-core contracts
python -m omnisim policy validate              # validate versioned manifests + contract ghost luts
python -m omnisim policy verify-demos          # PROVE the manifests reproduce demos/run_*.sh
python -m omnisim policy benchmark list        # machine-readable end-to-end acceptance cases
python -m omnisim policy graph box_delivery    # typed graph; exact legacy round-trip
python -m omnisim policy ir g1_walk            # task-space intent + robot binding
python -m omnisim policy matrix                # cross-morphology benchmark coverage
python -m omnisim policy promote sequence box_delivery --require release

python -m omnisim policy preview g1_walk       # the ghost hologram (design -> show -> agree)
python -m omnisim policy sequence box_delivery # run a BATON demo (walk->stand->carry->place)
python -m omnisim policy run g1_turn_in_place  # one skill solo
python -m omnisim policy sequence box_delivery --dry-run   # print the launch command, don't run

python -m omnisim policy ghost --all           # validate every ghost lut in the repo

python -m omnisim policy handover box_delivery # per-edge warm/cold plan (attractor-derived)
python -m omnisim policy blendable g1_walk g1_carry_box   # can two skills element-wise blend? what's needed?
python -m omnisim policy adapt g1_turn_in_place --to-nb 64 --out /tmp/turn64.json   # cross-cadence resample
python -m omnisim policy freeze g1_walk        # promote a training run's env lock into the manifest
python -m omnisim policy run go2_walk          # cross-robot: launches the baked-controller world
```

The pipeline verbs are implemented by `skill_lib.py`, which the front door delegates to;
it stays directly runnable (`python projects/policies/skills/skill_lib.py list`) for
anyone who scripted against it. Both build their subcommands from one shared verb table
(`skill_verbs.py`), so the two surfaces cannot drift apart.

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
[`humanoid/g1_arm_motion/README.md`](../../../docs/developer/g1-arm-motion-skill.md).

## The two regimes (why the libraries aren't symmetric)

| | humanoid | quadruped |
|---|---|---|
| stand / posture / balance | **deterministic** (verified) | **deterministic** |
| arm / hand motion while balancing | **deterministic** (`g1_arm_motion`) | n/a |
| locomotion (walk / turn / carry / climb) | **RL / Shadowing** | **RL** |

The manifest *interface* is shared; the *implementation* differs by `kind`. Don't force
one regime across both.

## Adding a skill

`design ghost → validate → preview (owner sign-off) → train → verify → write skill.json
→ register → sequence`. Step-by-step in
[docs/developer/skill-library.md § Adding a skill](../../../docs/developer/skill-library.md).
`python skill_lib.py index` regenerates `registry.json` from the filesystem.

## Verification

`skill_lib.py verify-demos` is the config-equivalence guard: it asserts that the env bundle
the manifests assemble is equal key-for-key to every reproduced recipe script or world
launcher. It is green for all five reproduced sequences (92 / 30 / 26 / 35 / 40 keys), plus
every Shadowing solo-launch contract. `skill_lib.py audit` additionally checks unique names,
schema/robot compatibility, every versioned ghost identity, the generated registry, and the
benchmark specification. `ghost --all` is deliberately broader: it also checks untracked
experiments in a developer's workspace.

End-to-end sequence acceptance lives in [`../benchmarks/`](../benchmarks/). The benchmark
runner scores machine-readable cycle verdicts, falls, height, duration, and required physical
events; it fails closed when evidence is missing. G1 locomotion results explicitly disclose
the lambda=0.9 weight-bearing balance harness and are not free-standing claims.
