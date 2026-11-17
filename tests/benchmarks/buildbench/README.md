# BuildBench — the capability suite

**Read [`SPEC.md`](SPEC.md) before anything else.** This file is orientation
only.

## What it is in one paragraph

BuildBench asks **"what robotics work can each simulator express?"** It is a
**capability suite, not a fairness benchmark** — it produces a matrix of
expressibility with citations, never a score, a ranking or a comparative claim.
It is a **sibling** of [`../agentbench/`](../agentbench/SPEC.md), which asks a
different question ("given one sentence and no human, did an agent get the job
done") under much stronger machinery, and which holds this tree's only credible
cross-simulator results. Nothing here replaces it.

## Status

**DECLARATION ONLY.** Five tasks are registered. None has a world, a grader, an
oracle or a null. **No cell has been run and nothing here may be quoted as a
result.** Of 15 expressibility claims, **1 is publishable** — and it is a
`NOT_EXPRESSIBLE` against OmniSim.

```
$ python -m buildbench.tasks            # the matrix   (run from tests/benchmarks)
$ python -m buildbench.tasks --list     # the task list
$ python -m buildbench.tasks --json     # the whole registry
$ python -m pytest tests/benchmarks/buildbench/test_declarations.py   # the honesty gate
```

## The five tasks

| id | what it demonstrates | build status |
|---|---|---|
| B1 `trained_locomotion_deploy` | train a policy in-engine, deploy it on a sensor-guided course | declared |
| B2 `granular_traversal` | graded interaction with granular media | **BLOCKED — measured, does not work** |
| B3 `robustness_distribution` | a success rate with an interval over ~1000 randomised draws | declared |
| B4 `multi_robot_radio` | ~20 robots coordinating over a radio device, collision-free | declared |
| B5 `procedural_generalization` | train on seeded worlds, grade on an unseen seed | declared |

## The four rules

1. **Every task must be genuine robotics work**, arguable without naming a
   simulator. A task reverse-engineered from a competitor's gaps does not
   belong here. Mechanically enforced.
2. **`NOT_EXPRESSIBLE` must carry evidence** — the specific missing capability,
   cited to that simulator's own docs. A single credible counter-example flips
   the label and the task gets run.
3. **Where a competitor CAN express a task, that is recorded plainly.** A suite
   in which we win everything is not credible. Two tasks we expect to lose are
   named in SPEC §5 and are still owed.
4. **The oracle/null gate** (AgentBench §7.1): a task nobody can demonstrably
   complete is not a capability claim. Per `(task, simulator)` — our own arm is
   not exempt.

## `verification_status`

Every claim carries one, and it starts at `UNVERIFIED`.

- `UNVERIFIED` — a belief. May not be shown without the word beside it.
- `CITED` — a resolvable citation to that simulator's own docs or source.
- `MEASURED` — a run on a named machine with a record under `evidence/`. **The
  only status that licenses a statement about behaviour.**
- `REFUTED` — a counter-example flipped it.

## The one thing that has been measured

[`evidence/2026-08-11-granular.md`](evidence/2026-08-11-granular.md) — the
granular subsystem does not support a graded task, on four independent grounds
(inert without a CUDA build and no CPU fallback; an always-empty robot-collider
list since the ODE deletion; a reverse force written into an empty function; no
particle-state readback at all). **B2 was therefore not authored.** It stays
registered at `BLOCKED` with its evidence, because a blocked task is the most
useful row in a capability matrix and deleting it destroys the record.

The same check found five documentation and world defects, listed at the end of
that record, including two docs that describe the granular demo as something it
has never been and one lane-4 probe world authoring fields that do not exist.

## Open risks

Recorded in `tasks.py::RISKS` and SPEC §6.1, because they are assumptions:

1. **Do camera/lidar sensors work in the batched `mujoco_warp` GPU path?**
   `UNVERIFIED`. Threatens B1, B3, B5 — i.e. it threatens **our own** claims
   first. The highest-value check outstanding.
2. **`newtonNjmax` / `newtonNconmax` overflow silently at ~20 wheeled robots.**
   B4 must set them deliberately or it measures our own unset default.
3. **Granular.** Closed, negatively. See above.
