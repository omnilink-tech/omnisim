# SO101 pick and place

Saved on 2026-09-19. The authored OmniSim trajectory physically picks the green
cube and places it inside the pink box. The recorded action replay still fails
with estimated calibration. The successful simulation is not an exact replay
or a validated transfer to hardware.

## Videos

| Video | Description |
|---|---|
| [Real vs. authored simulation](videos/real-vs-authored.mp4) | 21 seconds, original speed. The real clip holds its last frame after 15.23 seconds. |
| [Real vs. recorded replay](videos/real-vs-recorded-replay.mp4) | 15.23 seconds. Estimated calibration; simulated grasp fails. |
| [Standalone authored demo](videos/authored-demo.mp4) | Successful simulation, with labels. |
| [Original comparison edit](videos/recorded-comparison-original.mp4) | Earlier layout, retained with its failure label. |
| [Native authored capture](videos/simulation-authored.mp4) | Unlabelled simulation footage for re-editing. |
| [Native replay capture](videos/simulation-recorded-replay.mp4) | Unlabelled simulation footage for re-editing. |
| [Real right camera](source/videos/observation.images.right/chunk-000/file-000.mp4) | Original recording used in the comparisons. |
| [Real top camera](source/videos/observation.images.top/chunk-000/file-000.mp4) | Second original camera recording. |

## Saved work

- [Verification](evidence/verification.json): machine fingerprint, measured
  results, repeatability and test outcomes. The authored run carried the cube
  between both fingers for 4.13 seconds with 6.61 mm maximum relative slip.
- Run results, trajectories and contact/pose traces:
  [authored](evidence/authored/result.json),
  [open-gripper control](evidence/open-gripper-control/result.json),
  [recorded replay](evidence/recorded-replay/result.json).
- [Calibration estimate](inputs/estimated_calibration.json) and
  [authored motion](inputs/pickplace_motion.json).
- [Dataset attribution](source/README.md), [source hashes](source/source.json),
  [episode parquet](source/data/chunk-000/file-000.parquet) and original metadata.
  Source: `hbseong/record-pick-and-place-so101`, revision
  `3c0578a2dd3eb843b861b9398687b3c012d3070e`, episode 0, declared Apache-2.0
  ([license](source/LICENSE-2.0.txt)).
- [Archive manifest](manifest.json): file sizes and SHA-256 hashes, including
  hashes of the canonical implementation at the time of saving.

Historical evidence retains the original run paths. The delivered comparisons
and source recordings are copied byte-for-byte. Native captures are saved as
H.264 at 30 fps with one bottom row padded for the codec; re-encoding can change
pixels. The videos here do not depend on the original temporary folder.

## Code and reproduction

The runnable implementation remains in the
[robot project](../../projects/robots/therobotstudio/README.md), preserving its
controller paths and launcher entries. Its
[provenance](../../projects/robots/therobotstudio/so101/PROVENANCE.md) records the
model changes and missing real calibration.

Rebuild a comparison from these saved clips into a new file (from repo root;
requires FFmpeg and ffprobe):

```sh
python sim-to-real/so101/compose_comparisons.py authored --output outputs/so101/real-vs-authored.mp4
python sim-to-real/so101/compose_comparisons.py replay --output outputs/so101/real-vs-recorded-replay.mp4
```

To rerun the simulation, follow the robot project instructions. For a recorded
replay, use the parquet and calibration under this folder; the estimate still
does not reproduce a successful grasp.
