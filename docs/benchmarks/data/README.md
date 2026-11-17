# `docs/benchmarks/data/` — third-party published numbers, kept as evidence

Raw source data behind the ◐ *vendor-published* cells in
[../../developer/simulator-comparison.md](../../developer/simulator-comparison.md) §2.4.

**Nothing here was measured by us**, nothing here is read by any code, and nothing here is
a test fixture. These files exist so that a cited competitor figure can be traced back to
the row it came from instead of to a number someone typed once.

| file | what it is | rows |
|---|---|---|
| `maniskill3-benchmark-rtx4090.csv` | **ManiSkill 3** measuring **itself** | 186 rows, 6 envs |
| `isaaclab-via-maniskill-benchmark-rtx4090.csv` | **ManiSkill 3 measuring Isaac Lab** — *not* Isaac Lab's own publication | 217 rows, 2 envs |

Both carry ManiSkill's benchmarking-harness schema (`env.step/{dt,fps,psps,total_steps,cpu_mem_use,gpu_mem_use}`,
`env.step+env.reset/*`, `env_id`, `obs_mode`, `num_envs`, `gpu_type`, and `control_mode` on
the ManiSkill arm only). Every row reports `gpu_type = NVIDIA GeForce RTX 4090`.

## What these back

The one live citation today is the **ManiSkill 3** row of §2.4's competitor table:

> | **ManiSkill 3** ◐ | RTX 4090, state-only | FrankaMove **330k** @4096 |

which is this row of `maniskill3-benchmark-rtx4090.csv`:

```
env_id=FrankaMoveBenchmark-v1  obs_mode=state  num_envs=4096  env.step/fps=330095.63
```

The Isaac Lab file is **not cited anywhere** at present. It is kept because it is the paired
arm of the same run: the §2.4 Isaac Lab row quotes *Isaac Lab's own documentation table*
(Cartpole-Direct / Velocity-Rough-G1 / Repose-Cube-Shadow), which is a different measurement
by a different party on different environments. Do not merge the two — a third party's
measurement of a competitor and that competitor's self-published figure are not the same
kind of claim, and the second-hand one deserves the weaker mark.

## Provenance, including the gap

Downloaded **2026-08-12** during the ladder0 comparison work. ⚠️ **The source URL was not
recorded at download time**, so the provenance is "ManiSkill's published benchmarking
results, fetched on that date" and no more — the env ids (`*Benchmark-v1`) and the
`pd_joint_delta_pos` control mode identify the harness unambiguously, but the exact upstream
path and commit are unrecovered. Re-derive and record both before leaning on these for any
*new* claim; the existing 330k citation is checked against the row above and stands.

## Units — do not divide these by our numbers

These are ManiSkill's `env.step` FPS from ManiSkill's harness, on ManiSkill's scenes. The
separator in §2.4 applies in full: different robot, different GPU, different contact count,
and a step here is not a control step of 8 substeps. Cross-simulator FPS without a contact
count next to it is meaningless.
