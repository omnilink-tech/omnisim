# Provenance — `g1_achieved_gait.npz`

**Status: RESOLVED. Original work of this repository**, Apache-2.0
(© OmniLink). Not obtained from anywhere, and it contains no motion-capture
data.

This file exists because `scripts/release/publish_snapshot.sh` publishes a
squashed single commit, so git history does not travel to the public repository.

## What it is

One NumPy archive holding exactly two arrays, both `float32` and both purely
numeric — verified 2026-08-24:

| member | dtype | shape | what |
|---|---|---|---|
| `q` | float32 | (256, 13) | one phase-binned gait cycle: 256 phase bins × 13 joint targets |
| `stand` | float32 | (13,) | the symmetric standing anchor pose |

No string array, no author field, no tool tag, no source path.

## Where the numbers came from

It is **the robot's own recorded walk**, not a human's. A trained policy was
rolled out in this repository's engine, its achieved joint trajectory sampled
from its own joint sensors, then phase-binned, left/right symmetrised and
smoothed by the committed
[`../build_achieved_gait.py`](../build_achieved_gait.py). The result is a
*feasible-by-construction* reference — the tracking floor is near zero because
the robot has already executed it.

Full method, and the honest limit of what it bought (it inherits the recorded
~30° hip-roll splay, so on its own it does not fix the gait's appearance):
[`docs/developer/g1-improved-shadow.md`](../../../../../docs/developer/g1-improved-shadow.md)
and [`docs/developer/g1-deploy-walk.md`](../../../../../docs/developer/g1-deploy-walk.md).

## Not to be confused with the LAFAN1-lineage ghosts

`projects/policies/ghosts/g1/` holds reference motions that **are** derived from
Ubisoft La Forge's LAFAN1 motion-capture dataset (CC BY-NC-ND 4.0) and are
excluded from the public snapshot by `scripts/release/publish_deny.txt`. This
dataset is not one of them: every sample in it was produced by the simulator.
