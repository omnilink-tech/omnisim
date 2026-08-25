# Provenance — `model_seg.npz`

**Status: RESOLVED. Original work of this repository**, Apache-2.0
(© OmniLink).

This file exists because `scripts/release/publish_snapshot.sh` publishes a
squashed single commit, so git history does not travel to the public repository.

## What it is

The trained weights of the free-space segmenter that `learned_seg.py` runs at
inference — numpy + cv2 only, no sklearn and no torch, deliberately, so the
controller carries no heavyweight dependency. 6,096 bytes. Verified 2026-08-24
to contain exactly ten arrays, all numeric, none of them a string:

| member | dtype | shape |
|---|---|---|
| `n_layers` | int64 | () |
| `size` | int64 | (2,) |
| `mean`, `std` | float32 | (10,) |
| `w0`, `b0` | float32 | (10, 32), (32,) |
| `w1`, `b1` | float32 | (32, 16), (16,) |
| `w2`, `b2` | float32 | (16, 1), (1,) |

A 10 → 32 → 16 → 1 MLP plus its input normalisation. No author field, no tool
tag, no dataset path, no vendor string.

## How it was produced

Trained by the committed
[`../../tools/train_segmenter.py`](../../tools/train_segmenter.py) on frames
rendered by OmniSim from this repository's own Omni Quest worlds. No third-party
dataset, no pretrained backbone, and no transfer from an external model — the
network is small enough that the whole thing is reproducible from the script and
the worlds beside it.
