# Capability ladder — first grid (2026-08-02)

**Read the second table before quoting the first.** This is the
*achievability* grid: for each (tier, simulator), can a **scripted control** —
a controller a human wrote, knowing the thresholds — get the shipped container
to the physical outcome the tier demands? It is deliberately not the agent
question. It exists so that when an agent later fails a cell, the failure is
attributable to the agent rather than to an impossible asset.

Machine `9722d23d12a3` (RTX 3060 laptop, Windows 11), mujoco 3.8.1,
python 3.12.9, engine `msys64/mingw64/bin/omnisim-bin.exe`.

## The grid

| tier | what it demands | omnisim | mujoco | webots |
|---|---|---|---|---|
| **T1 arrive** | reach a point 5 m north, stop, dwell ≥ 2 s | **achieved** `PASS 5/5` | **achieved** `PASS 5/5` | NOT-BUILT |
| **T2 transfer** | pick a block up, hold it 10 s, put it in a bin | *no oracle yet* | **achieved** `PASS 5/5` | NOT-BUILT |
| **T3 quadruped** | walk 10 m unsupported without falling | *no oracle yet* | **achieved** `PASS 5/5`, 31.54 m | NOT-BUILT |
| **T4 humanoid** | the same, on two legs, support declared | *no oracle yet* | **achieved** `PASS 5/5`, 73.30 m, T4-supported | NOT-BUILT |

## What the empty cells mean — and what they do not

**Every blank on the omnisim column reads `not_achieved(scaffolding_defect_ours)`.**
That code is load-bearing. It means *we have not written the scripted control
yet*. It is **not** a measurement that OmniSim cannot do these things, and this
grid must never be quoted as "MuJoCo 4, OmniSim 1".

There is direct evidence against that reading:

- **T2.** A Claude Opus 5 agent, given one sentence and no help, produced a
  working two-finger friction pinch on this column — its own README claims a
  10.0 s airborne hold and the block landing dead-centre in the bin. The cell
  was **capped 90 s after the agent said "task complete"**, and the recovered
  deliverable could not be regraded because phase B re-runs a world bare and
  that world needs eight environment variables to reproduce. Recorded in
  [ladder-findings-2026-08-02.md](ladder-findings-2026-08-02.md) §6.
- **T3 / T4.** This repository ships *measured* legged locomotion on OmniSim —
  a Go2 that walks 12.09 m and a G1 whose endurance run is 6/6 with no falls.
  Neither is wired to a ladder oracle yet, and neither may be counted here
  until it is: **no row, no result.**

So the honest one-line summary is: **MuJoCo's column is finished and OmniSim's
is a quarter finished.** That is a statement about our build-out, not about the
simulators.

## What the grid does establish

1. **The instrument works on two columns.** The same three shipped
   descriptions, the same neutral grader, two different engines, and the
   per-simulator adapters kept out of the graded core by the AST vocabulary
   guard.
2. **T1 is achievable on OmniSim and on MuJoCo**, with the arrival, dwell,
   stayed-up, no-teleport and run-is-real clauses all green on both.
3. **T2–T4 are achievable at all.** Before this, "the block cannot be
   grasped in a simulator" and "our agent could not grasp it" were
   indistinguishable. Now they are not.

## Open items this grid exposed

- **`defaultPhysicsBackend "newton"` on the T1 world records a 296 kB motion
  file with an empty body roster**, so the grader reports *"the run reported no
  bodies at all"* where the identical world on ODE passes 5/5. The oracle
  therefore pins ODE and says so. This is unexplained and is the next thing to
  look at on this column: our default backend failing the cheapest tier would
  matter.
  > **⚠ 2026-08-08 — the mitigation is gone and the T1 failure is now unmitigated.**
  > `bdc02139` deleted ODE, so (a) the "identical world on ODE passes 5/5" control no longer
  > exists and cannot be re-run — it is preserved here as a dated observation only — and
  > (b) **the oracle's `"ode"` pin still loads and still scores, on no physics at all**: an
  > explicit `"ode"` still wins and resolves to an inert stub, with no FATAL, no ERROR and no
  > warning, so the T1 oracle now emits 5/5-shaped output from a world where nothing moves.
  > That is worse than a broken oracle. T1 on this column has **no trustworthy oracle at all**. **The T1 oracle must be re-based on Newton**, which
  > requires the empty-body-roster defect to be diagnosed and fixed rather than routed
  > around. This moved from "next thing to look at" to blocking.
- **`physicsBackend` is not a `WorldInfo` field.** The world-level name is
  `defaultPhysicsBackend`; `physicsBackend` is per-`Solid`. An unknown field
  logs `ERROR:` and takes the headless run's exit code to 1, which reads as a
  crash. AGENTS.md's phrasing invites the mistake.
- **The base-name question is settled** (it was pre-registered): a tier names
  its base by the root-link name of its own description, and a converter is
  free not to reproduce that. The grader now falls back to **the body whose
  mass matches the declared mass**, uniquely and within 1 % — an identity
  check, not a guess — and refuses when two bodies match. Without it, T3/T4 on
  this column scored the converter's naming convention and reported it as the
  robot failing to walk.

## Next, in order

1. T2 oracle on omnisim (the recovered agent deliverable is a starting point,
   once the self-contained-vs-manifest question is settled).
2. T3 oracle on omnisim (the MuJoCo gait is analytic and engine-agnostic; it
   is a port, not a research problem).
3. T4 oracle on omnisim.
4. The webots column, which is what the whole comparison is for.
