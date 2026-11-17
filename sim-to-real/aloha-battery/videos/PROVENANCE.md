# Provenance — `sim-to-real/aloha-battery/videos/`

Nine delivered files from the ALOHA battery-insertion study, plus the subtitle
script one of them is burned from. They fall into two groups with **different
licensing positions**, so they are recorded separately rather than under one
blanket claim. This mirrors `../../so101/videos/PROVENANCE.md`, which covers the
sibling study.

## OmniLink's own work

Rendered by OmniSim from this repository's own worlds and controllers. No
third-party footage is present in any frame.

| file | what it is |
|---|---|
| `native-spring-insertion.mp4` | the filmed spring-insertion run, unlabelled native capture |
| `native-authored.mp4` | the authored insertion trajectory, native capture |
| `native-authored-slot-view.mp4` | the same authored run from the slot-facing camera |
| `native-recorded-actions.mp4` | the recorded-action replay, native capture |

Licensed under Apache-2.0 with the rest of this repository.

## Composites that embed third-party footage

These are side-by-side edits. The simulation half is OmniLink's own render; the
**real half is frames from a third-party dataset** and carries that dataset's
terms.

| file | embeds |
|---|---|
| `real-vs-spring-insertion.mp4` | real low-camera recording + the spring-insertion reconstruction |
| `real-vs-authored.mp4` | real low-camera recording + the authored insertion |
| `real-vs-authored-slot-placement.mp4` | real low-camera recording + the authored slot placement |
| `real-vs-recorded-actions.mp4` | real low-camera recording + the recorded-action replay |
| `spring-insertion-poster.png` | a frame of `real-vs-spring-insertion.mp4`, used as the repository README thumbnail |

`real-vs-spring-insertion.ass` is the subtitle script burned into the composite
of the same name. It is OmniLink's own text and carries no third-party content,
but it is listed here because it is meaningless apart from that composite.

Source of the real footage, copied byte-for-byte into `../source/` and hashed in
`../manifest.json`:

- dataset [`lerobot/aloha_static_battery`](https://huggingface.co/datasets/lerobot/aloha_static_battery)
- revision `06dc3da83c4fd3d1889b00f1dfd3780da8421f64`, episode 0 — 600 frames at
  50 Hz, 12 seconds, task "Place the battery into the slot of the remote controller"
- declared **Apache-2.0**; the licence text is kept at `../source/LICENSE-2.0.txt`
- recording credit: ALOHA, Tony Zhao and collaborators; dataset distribution by
  LeRobot
- attribution, original URLs, pinned revisions, byte sizes and SHA-256 hashes:
  `../source/README.md`, `../source/downloads.json`

Apache-2.0 permits the derivative, and the attribution above is how §4 is
discharged for these five files. Anyone redistributing them must keep that
attribution with them.

## What these videos do and do not show

Stated here because a video separated from `../README.md` invites the wrong
reading. The simulation compresses a moving negative terminal, releases the
tilted battery and seats it with a separate fingertip press, from motor commands
and contact forces alone. But the actions and timing are **authored** — they are
not an exact replay of the real episode and not a learned policy — the
**recorded actions do not reproduce the insertion**, the geometry and mechanism
parameters are **estimates**, the controller reads privileged simulator poses
and spring displacement that hardware would have to sense, and there is **no
electrical simulation and no hardware validation**. The real clip runs its
original 12 seconds and then holds its final frame under an explicit label while
the 28-second simulation continues; the hold is labelled in the frame so the
two halves are never read as the same duration. See `../README.md` and
`../evidence/spring-insertion/result.json` for the measured results and the
machine they were measured on.
