# SO101 provenance and limitations

The two URDFs and 15 visual STL meshes were imported from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), commit
`eecbe3e0a9ebb23e25ad7b2759b03884c6660903`, `Simulation/SO101/`, on 2026-09-15.
They are redistributed under Apache-2.0; see [LICENSE.upstream](LICENSE.upstream)
and [CITATION.upstream.cff](CITATION.upstream.cff). No endorsement is implied.
`so101.urdf` is upstream's `so101_new_calib.urdf`; `so101_camera.urdf` is its
camera variant. Visual meshes and joint transforms remain upstream's.

Local changes to both URDFs:

- The fixed TCP frame has mass `1e-4` kg and diagonal inertia `1e-8`, replacing
  upstream's `1e-9` kg and zero tensor so physics validation passes.
- The fixed and moving jaw meshes use seven convex collision slices, generated
  by [build_collision_meshes.py](build_collision_meshes.py). The visual STLs
  are unchanged. [The manifest](assets/collision/manifest.json) records the
  slicing planes and bounds. These approximations retain the finger taper
  which a whole-mesh convex hull fills in.
- The gripper effort is `0.3` N m instead of the upstream placeholder `10`.
  This is a **simulation tuning assumption**, not a sourced servo rating.
  Other joints retain upstream's placeholder effort/velocity `10/10`.
  Neither the torque limits nor the resulting servo gains validate hardware
  dynamics. Both variants pass `python -m omnisim validate-urdf`.

`newtonCompoundColliders TRUE` is required: otherwise only the first child
collider of a compound body is registered. The authored demo uses the CPU
MuJoCo solver through Newton, a 2 ms step, friction 3, elliptic contacts,
impratio 10 and condim 4. Cube size 40 mm, mass 20 g, and the box dimensions
are assumptions. Contact parameters are specific to this scene.

## Recorded source

The attempted comparison uses
[hbseong/record-pick-and-place-so101](https://huggingface.co/datasets/hbseong/record-pick-and-place-so101),
revision `3c0578a2dd3eb843b861b9398687b3c012d3070e`, episode 0:
457 frames at 30 Hz, task “Pick the green cube, and place it in the pink box”.
The dataset metadata declares Apache-2.0. The episode's parquet SHA-256 is
`1389458555b049b5e444230293d9fdd570f175b24fdd4bd3271a6503fa7cceb0`.

LeRobot's body values can be degrees or calibrated normalised values; the
gripper is normalised 0–100 even in degree mode. Normalised values depend on
the physical robot's calibration ranges and signs. **The dataset does not
publish the calibration file.** Numerical range alone cannot determine units,
and normalised ranges must not be mapped onto URDF joint limits.

[estimated_calibration.json](../estimated_calibration.json) is explicitly an
estimate: follower encoder spans inferred from the float quantisation are
2441, 2316, 2231, 2392, 3864 and 907 ticks. Conversion assumes 4096 ticks per
turn; signs are assumed positive and zero offsets are estimated. This is not
a recovery of the actual calibration or evidence of accurate joint angles.
Scene positions inferred through this mapping are estimates too.

The old replay used `/robot/SO101/joints/set`. That route reaches supervisor
joint-pose updates; it is inappropriate for evaluating a dynamic grasp.
The replacement drives `Motor.setPosition` from the owning controller on every
simulation tick. It moves the cube aside during initialisation, settles the
arm, restores the cube, and then starts the timed episode. There are no cube
pose writes or attachments during the episode. Measured angles, contacts and
cube poses are flushed incrementally, and an incomplete run cannot pass.

## What has been established

The **authored simulation trajectory** in [pickplace_motion.json](../pickplace_motion.json)
physically picks, carries and releases the cube into the box. It is a new
trajectory, not the public episode's recorded action stream. The same motion
with the gripper held open leaves the cube on the table. See
[verification.json](../verification.json) for the machine, result and hashes.

The **full recorded approach has not been physically reproduced**. Estimated
calibrations tested so far push the cube out of the grasp before lifting.
The improved replay pipeline completes the episode, but completion is not
grasp success. An exact real-versus-simulation comparison still needs the
recording robot's calibration and measured scene/model parameters. No policy
or dynamics result from this work has been validated on physical hardware.
