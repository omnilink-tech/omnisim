# Provenance — `sim-to-real/so101/videos/`

Six delivered videos from the SO-101 pick-and-place archive saved 2026-09-19.
They fall into two groups with **different licensing positions**, so they are
recorded separately rather than under one blanket claim.

## OmniLink's own work

Rendered by OmniSim from this repository's own worlds and controllers. No
third-party footage is present in any frame.

| file | what it is |
|---|---|
| `authored-demo.mp4` | the authored trajectory, with labels |
| `simulation-authored.mp4` | the same run, unlabelled native capture for re-editing |
| `simulation-recorded-replay.mp4` | the recorded-action replay, unlabelled native capture |

Licensed under Apache-2.0 with the rest of this repository.

## Composites that embed third-party footage

These are side-by-side edits. The simulation half is OmniLink's own render; the
**real half is frames from a third-party dataset** and carries that dataset's
terms.

| file | embeds |
|---|---|
| `real-vs-authored.mp4` | real right-camera recording + authored simulation |
| `real-vs-recorded-replay.mp4` | real right-camera recording + replay simulation |
| `recorded-comparison-original.mp4` | earlier layout of the same comparison |
| `comparison-poster.jpg` | frame at 5 seconds of `real-vs-authored.mp4`, used as the repository README thumbnail |

Source of the real footage, copied byte-for-byte into `../source/` and hashed in
`../manifest.json`:

- dataset `hbseong/record-pick-and-place-so101`
- revision `3c0578a2dd3eb843b861b9398687b3c012d3070e`, episode 0
- declared **Apache-2.0**; the licence text is kept at `../source/LICENSE-2.0.txt`
- attribution and source hashes: `../source/README.md`, `../source/source.json`

Apache-2.0 permits the derivative, and the attribution above is how §4 is
discharged for these three files. Anyone redistributing them must keep that
attribution with them.

## What these videos do and do not show

Stated here because a video separated from `../README.md` invites the wrong
reading: the authored simulation picks the cube and places it in the box; the
**recorded action replay fails** under estimated calibration. **No policy here
has been validated on hardware**, and the successful run is an authored
trajectory, not a replay of the real episode. See `../README.md` and
`../evidence/verification.json` for the measured results.
