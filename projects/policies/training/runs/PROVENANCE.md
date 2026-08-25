# Provenance — in-engine RL training run output (`training/runs/`)

**Status: RESOLVED. Everything here is original work of this repository**,
Apache-2.0 (© OmniLink). Nothing in this directory was obtained from anywhere.

This file exists because `scripts/release/publish_snapshot.sh` publishes a
squashed single commit, so git history does not travel to the public repository.

> ⚠ **This directory is gitignored** (`.gitignore:143`, `projects/policies/training/runs/`)
> and the checkpoints in it were **force-added**. This record had to be
> force-added the same way, and that is not optional bookkeeping: both the
> provenance gate and `publish_snapshot.sh` operate on **tracked** files, so an
> untracked `PROVENANCE.md` here confers no coverage and never reaches the
> public snapshot — it is a local note, not a record. If you ever find these
> files uncovered again, check `git ls-files -- projects/policies/training/runs/PROVENANCE.md`
> first.

## What is here — measured 2026-08-24

| count | files | what |
|---:|---|---|
| 35 | `*.pt` | PyTorch policy checkpoints written by this repository's own in-engine RL pipeline (`projects/policies/training/`), which trains *through* `omnisim-bin` so train == deploy bit-exact |
| 2 | `g1_walk_mlp.npz`, `g1_walk_res.npz` | flat parameter vectors from the same pipeline's evolution-strategy trainer — `theta` (588 and 144 float32) plus a 3-element `meta`. No layer names, no optimiser state, no metadata of any kind |

Both `.npz` files are **weights only**: `np.load(...).files` is exactly
`['theta', 'meta']`, both `float32`, and there is no string array in either.

## Why the checkpoints are ours

They are produced by training, in this repository, against this repository's own
robot models and reward functions. The gate at
[`tests/sources/test_asset_provenance.py`](../../../../tests/sources/test_asset_provenance.py)
carries the same claim as an `OWN_WORK` entry scoped to
`('projects/policies/', ('.pt', '.onnx'))` — deliberately extension-scoped, so a
claim about trained weights cannot silently start covering an image dropped into
the same tree. That scoping is exactly why the two `.npz` files needed this
file: they are the same kind of artifact, produced the same way, and the gate
cannot see them.

## Twelve of the `.pt` files are PUBLISH-DENIED, and not for a provenance reason

`scripts/release/publish_deny.txt` excludes `wr_hop1_it100.pt`,
`wr_v6calm_it150.pt`, `wr_v7trk_it150.pt`, `wr_v9b_it150.pt`, `wr_v9s2_it500.pt`,
`wr_v11_it200.pt`, `wr_v13_it250.pt`, `wr_v16_it200.pt`, `wr_v16c.pt`,
`wr_smooth_it150.pt`, `wr_rhythm_champion.pt` and `wr_metronome_champion.pt`.

Those are **ours** — no LAFAN1 frame is extractable from a `.pt` — but they were
*trained against* a HOP-1 reference whose lineage reaches Ubisoft La Forge's
CC BY-NC-ND LAFAN1 dataset, and that licence's NonCommercial term limits the
**use**, not merely the sharing. The exclusion is **precautionary and says so**;
the argument is in
[`docs/developer/motion-data-provenance.md`](../../../../docs/developer/motion-data-provenance.md)
and in the deny-list's own Class C block.

The flagship `wr_decent_walker.pt` is **not** in that set and must not be added
to it — it trains against `ghost_official_full_v3_lut`, which is Unitree
lineage, not LAFAN1.

So a public reader sees 23 of the 35 checkpoints. That is a *distribution*
decision recorded in the deny-list, not a gap in this record: the terms of all
35 are the same, and they are stated above.
