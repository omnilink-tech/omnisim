# The 152 worlds whose declared friction disagrees with the run that uses them

**2026-08-09.** The corpus sweep
([`translation_audit.py --sweep`](../../tests/benchmarks/omnibench/lane1/translation_audit.py))
found 152 worlds under `projects/policies` and `projects/robot_combat` that
declare a friction the solver never receives. They were deliberately **not**
migrated, and this page is the reason why, plus the decision needed to close them.

It exists because "needs an owner" is not an action. Below is what a decision
would have to answer.

## The mechanism, in one line

`ContactProperties.coulombFriction` is an **ODE-path** field the Newton backend
does not read. `newtonGroundMu` is the field that sets friction, and it defaults
to 1.0. So a world declaring `coulombFriction 1.5` and no `newtonGroundMu` runs
at **1.0** — unless a launcher exports `OMNISIM_NEWTON_GROUND_MU`, which beats
both.

## Why the automatic migration refuses these

The migration's rule is: **never migrate a world whose launcher forces a
different value than it declares.** Writing the world's own number is inert
wherever the launcher agrees (env > field > default) and only repairs the bare
load — but where they disagree it makes the file *authoritatively* state a
friction the sanctioned experiment does not use. More authoritative, no more
truthful.

For `projects/samples` the migration can check this automatically, because a demo
keeps its launcher beside it as `run_<stem>.ps1`. **These 152 have no such
sibling** — 0 of 152 — because their env vars come from *recipe* scripts
(`run_walk_rl.sh`, `push_ab.sh`, `run_g1_*.ps1`, …) that drive many worlds and do
not follow the world's name. Nothing in the tree maps world → recipe, so the tool
cannot tell agreement from disagreement here, and it stops.

## What the numbers say

**Declared in the worlds:**

| declared `coulombFriction` | worlds |
|---|---|
| 1.5 | 85 |
| 2.0 | 46 |
| 0.9 | 10 |
| 0.8 | 9 |
| 5.0 | 2 |

**Exported by the recipes** that drive them: overwhelmingly
`OMNISIM_NEWTON_GROUND_MU=2.0` (the G1/quadruped deploy and training scripts —
`push_ab.sh`, `run_g1_stand_deploy.ps1`, `run_g1_brain*.ps1`,
`eval_push_recovery.py`, `continue_omniquad_newton.py`, …), with a few at **1.0**
(`run_g1_ghost_walk_gui.ps1`, `run_g1_unitree_walk.ps1`).

So the three groups, and the question each poses:

1. **46 worlds declare 2.0 and the dominant recipe exports 2.0.** These agree,
   and migrating them is inert for the sanctioned run while repairing the bare
   load. ⚠ Still not automatic: a handful of recipes export 1.0, and without a
   world → recipe map we cannot prove *which* recipe drives a given world.
   **Question: is any of these 46 driven by a 1.0 recipe?** If not, they can be
   migrated in one command.
2. **85 worlds declare 1.5 while the recipes export 2.0.** A real disagreement.
   The experiments — including the trained champions — ran at 2.0; the files say
   1.5. **Question: which number is intended?** If 2.0, the worlds should declare
   2.0 (and the recipes' export becomes redundant belt-and-braces). If 1.5, the
   recipes are wrong and every champion trained under them is attributed to the
   wrong contact model.
3. **21 worlds declare 0.8 / 0.9 / 5.0.** Quadruped and manipulation scenes whose
   recipes were not identified here. **Question: same as (2), per family.**

## Why this is not cosmetic

The friction a legged robot stands on is not a detail. `worldinfo.md` records a
measured probe: sphere feet slide **~1 m while merely standing** at μ=1.0 and
plant within **4 cm** at μ=2.0. Every one of these 152 worlds is running at 1.0
on a bare load. Any of them opened directly — by a user, by an agent, by
`run-headless`, or by a grader that re-runs the world without the recipe — is
simulating a different contact model from the one the results were produced
under. That is the same failure the `newton*` fields were added to fix, quoted in
the engine's own source:

> a world file was therefore NOT a complete description of its own physics … the
> working configuration could not be handed to anybody, including to our own
> grader, which re-ran the world bare and scored the result a failure.

## How to close it

Once the owning workstream answers the question for a family:

```bash
# inert where the launcher agrees; repairs the bare load
python tests/benchmarks/omnibench/lane1/translation_audit.py \
    --sweep projects/policies --fix

# verify a specific world got what it declares (reads the real mjModel)
python tests/benchmarks/omnibench/lane1/translation_audit.py --world <world.omniworld>
```

If the answer is "the recipe value wins", edit the world to declare the
**recipe's** number rather than its current `coulombFriction`, then delete the
`contactProperties` block so the file has one friction, not two.

**The cheap structural fix that would prevent recurrence:** give each world a
launcher whose name derives from it, or record the recipe in the world (a comment
naming the driving script is enough). Then the migration can check agreement
automatically, as it already does for `projects/samples`.
