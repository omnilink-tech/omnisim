# ALOHA battery insertion

## Spring mechanism reconstruction

The new version physically compresses a moving negative terminal, releases
the tilted battery, and uses a separate fingertip press to seat it against
the positive contact and compartment floor. The spring remains compressed
after the gripper withdraws. All task motion comes from motor commands and
contact forces; no battery pose is prescribed during the task.

- [Watch the real/simulation comparison](videos/real-vs-spring-insertion.mp4).
- [Native OmniSim capture](videos/native-spring-insertion.mp4).
- [Mechanism, parameters and reproduction instructions](../../projects/robots/trossen/aloha/MECHANISM.md).

The comparison is **28 seconds, 1920 x 1080, 50 fps**. The real low-camera
recording plays its original 12 seconds, then holds its final frame with an
explicit label. The simulation plays all 28 seconds at original speed, with
measured spring displacement overlaid. The actions and timing are authored;
they are not an exact replay or a learned policy.

On machine `9722d23d12a3` (RTX 3060 Laptop GPU, Windows 11), both the filmed
run and an independent repeat passed. All **1,400 trace records match exactly**.
These two runs do not establish a general success rate.

| Measurement | Result |
|---|---:|
| Bilateral airborne grip | 4.30 s |
| Horizontal transport during sustained contact | 99.91 mm |
| Maximum carry slip | 0.58 mm |
| Peak spring compression, measured every physics step | 9.92 mm |
| Final spring compression | 5.98 mm |
| Maximum negative-terminal penetration | 0.50 mm |
| Maximum housing penetration | 0.44 mm |
| Contact audit | 29,001 samples at 1,000 Hz |
| Final state | Contained, released, settled; floor and both terminals touching |

The spring is a declared linear-force model on a passive slider. Its wire
turns are a visual indicator, and the remote dimensions, friction, pads and
spring parameters are estimates. **There is no electrical simulation or
hardware validation.** The controller uses privileged simulator poses and
spring displacement; hardware would need corresponding sensing.

Evidence is retained in [the filmed run](evidence/spring-insertion/result.json)
and [the repeat](evidence/spring-repeat/result.json), including target commands,
pose/contact traces, a physics-step audit, exact controller/model snapshots,
hashes and the Newton engine sidecar. The [machine fingerprint](evidence/environment.json)
identifies this host; each new result separately records the actual runtime settings.

The [open-gripper control](evidence/spring-no-grip/result.json) executes the
filmed run's exact arm commands, with only the right fingers held open. The
battery remains on the table. [Independent re-evaluation](evidence/spring-verification.json)
checks the saved traces, source hashes, physics audits and command equality.

The [isolated mechanism checks](evidence/spring-probes/verification.json) also
meet their expected outcomes: a 0.75 N load compresses the spring 6.00 mm and
it returns after unloading; disabling the spring leaves the terminal at its
travel stop. A seated battery stays retained under a 0.6 N upward force for
0.4 s. The spring-disabled and gravity-drop controls fail retention. A flat
battery settles at the analytic table height. These probes test the declared
mechanism; they do not calibrate it against physical hardware.

The final validation included 12 study unit tests, two environment-reference
checks, a warning-free world-load check and the three-run engine regression for the multiple-contact switch.
The broader unit suite was also attempted: this environment lacks the optional
OmniLink SDK required by nine relay tests, and an unrelated native-IK reachable
target test still fails. The demo uses its own tested URDF solver.

```sh
python projects/robots/trossen/aloha/verify_insertion.py --run sim-to-real/aloha-battery/evidence/spring-insertion --repeat sim-to-real/aloha-battery/evidence/spring-repeat --no-grip sim-to-real/aloha-battery/evidence/spring-no-grip --output outputs/aloha-spring-verification.json
```

Rebuild the comparison from the saved native footage:

```sh
python sim-to-real/aloha-battery/compose_comparison.py --run sim-to-real/aloha-battery/evidence/spring-insertion --real-view low --native-video sim-to-real/aloha-battery/videos/native-spring-insertion.mp4 --output outputs/aloha-spring-comparison.mp4
```

## Archived placement prototype — checkpoint `4d8ffea56`

Saved on 2026-09-20. OmniSim physically picks up a battery, carries it while the
other hand braces a remote controller, and releases it into the battery slot.
The comparison pairs this **authored simulation trajectory** with a real ALOHA
recording of the same task. It is not exact recorded-action replay or validated
transfer to hardware.

**This is a grasp-and-placement prototype, not spring-loaded battery insertion.**
The remote has rigid slot walls and a floor, with no springs, electrical
terminals or retaining contacts. The authored trajectory lowers the battery,
opens the gripper and allows gravity to settle it. The slot-containment success
check does not establish spring compression, seating or electrical contact.

### Videos

| Video | What it shows |
|---|---|
| [Real vs. simulation — slot close-up](videos/real-vs-authored-slot-placement.mp4) | Closer side view of the pickup and release into rigid slots. Explicitly labelled as a placement prototype without spring-loaded insertion. Same authored motion, 12 seconds at original speed. |
| [Real vs. authored simulation](videos/real-vs-authored.mp4) | 12 seconds, 1920 x 1080, 50 fps. Both sides play at original speed; trajectories and task timing differ. |
| [Real vs. recorded actions](videos/real-vs-recorded-actions.mp4) | The unsuccessful replay, explicitly labelled. The estimated command mapping does not reproduce the real insertion. |
| [Native authored capture](videos/native-authored.mp4) | Unlabelled OmniSim footage for re-editing. |
| [Native slot close-up](videos/native-authored-slot-view.mp4) | Unlabelled close side view for re-editing. |
| [Native recorded-action capture](videos/native-recorded-actions.mp4) | Unlabelled failed replay. |
| [Real overhead camera](source/episode-000-high.mp4) | The exact real episode used in the comparison. |
| [Real low camera](source/episode-000-low.mp4) | A second synchronized view of the same episode. |

No frames are invented or motion-retimed. The simulation footage is native
OmniSim capture, with a fixed crop in the comparison. The underlying 2 ms
physics is sampled at 50 Hz for video and evidence. A further second is allowed
after the episode for the final settling check.

### What was measured

On machine `9722d23d12a3` (RTX 3060 Laptop GPU, Windows 11), the authored run
maintained bilateral airborne finger contact for **3.40 seconds**, transported
the battery approximately **144 mm**, and braced the remote for **5.62 seconds**.
The battery ended fully inside the slot, released and settled. The same motion
with the right gripper held open leaves the battery on the table.
An independent repeat also passed and matched all 600 sampled pose, joint and
contact records. These two runs do not establish a general success rate.

**Grip quality still needs improvement:** maximum movement of the battery
relative to the gripper was **17.96 mm**. The task passes; the separate strict
10 mm grip-stability check fails. The pads, scene dimensions, servo parameters
and gripper-command mapping are estimates. No trained policy or hardware
transfer is claimed.

Evidence:

- [Verification and repeatability](evidence/verification.json),
  [machine fingerprint](evidence/environment.json).
- [Authored result](evidence/authored/result.json),
  [open-gripper control](evidence/open-gripper/result.json),
  [recorded-action result](evidence/recorded-actions/result.json).
- [Close camera verification](evidence/insertion-angle/camera-verification.json):
  all 600 sampled physical records and the final result match the original
  authored run; only the viewpoint and presentation change.
- Each run folder includes target configuration, contact and pose traces,
  generated-world text and a manifest. The captured authored run also retains
  its engine log and controller source.
- [Dataset attribution and licenses](source/README.md),
  [archive hashes](manifest.json), and
  [robot/scene provenance](../../projects/robots/trossen/aloha/PROVENANCE.md).

The source is `lerobot/aloha_static_battery`, revision
`06dc3da83c4fd3d1889b00f1dfd3780da8421f64`, episode 0. Its real videos and
joint data are from the same episode. No physical SO101 or ALOHA is required
to run this simulation.

### Reproduce

Use the [robot project instructions](../../projects/robots/trossen/aloha/README.md)
to run the authored motion, its negative control or the recorded actions.
To rebuild the comparison from the saved native video, with FFmpeg installed:

```sh
python sim-to-real/aloha-battery/compose_comparison.py --run sim-to-real/aloha-battery/evidence/authored --native-video sim-to-real/aloha-battery/videos/native-authored.mp4 --output outputs/aloha-comparison.mp4
python sim-to-real/aloha-battery/compose_comparison.py --run sim-to-real/aloha-battery/evidence/insertion-angle --native-video sim-to-real/aloha-battery/videos/native-authored-slot-view.mp4 --output outputs/aloha-slot-comparison.mp4
```

Saved clips do not depend on temporary files. Historical run configurations
retain their original output paths for audit; the replay CLI writes new paths
when reproducing an experiment. The manifest hashes the current canonical
implementation as well as the archived files. Archive hashes preserve exact
bytes; canonical text hashes normalize CRLF to LF for portability. Run
`python sim-to-real/aloha-battery/update_manifest.py` to verify the archive.
