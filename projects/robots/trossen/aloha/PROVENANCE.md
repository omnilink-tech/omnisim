# ALOHA battery study: provenance and limits

This reconstructs a public real robot task in OmniSim. It is **not a policy
trained in simulation and transferred to hardware**. No physical ALOHA or
SO101 was used for this work.

## Robot model

The original ALOHA uses two ViperX 300S follower arms. The source MJCF and STL
files come from [ACT](https://github.com/tonyzhaozh/act), commit
`742c753c0d4a5d87076c8f69e5628c79a8cc5488`. See
[the upstream MIT license](source/act/LICENSE). Upstream source is retained
under `source/act`; modified files are outside that directory.

[build_model.py](build_model.py) converts link transforms and diagonal inertias
to URDF. MJCF intrinsic XYZ Euler rotations become URDF extrinsic RPY; source
millimetre mesh scales are baked into metre-valued STLs. Each arm has its own
fixed base at x = +/-0.469 m. The common source y = 0.5 m becomes y = 0; the
right base yaw is rounded from 3.1416 to pi. Independent MuJoCo forward
kinematics checks 12 source poses against the URDF, within 1e-5 in position
and rotation-matrix elements. This validates kinematics, not hardware dynamics.

Explicit simulation assumptions and changes:

- Arm efforts are 80 N m and finger efforts 15 N, with velocity limits of
  5 rad/s and 0.2 m/s. These are not verified servo ratings. Source actuator
  gains, friction loss and hardware linkage dynamics are not reproduced.
- Finger slider limits are widened to +/-[0.005, 0.065] m so negative
  normalized source gripper values remain visible rather than silently clamped.
- Shaped finger meshes use five convex slices each. A whole-mesh convex hull
  fills gaps in the finger shape. Slices are approximations generated from
  source triangles; visual meshes retain the original geometry.
- Each finger additionally has a **visible, estimated contact pad**,
  14 x 4 x 12 mm, at link x = 78 mm, y = +/-19 mm. These pads are a simulation
  addition, not established hardware from the recording. Original finger mass
  and inertia are used as estimates for the assembled finger and pad.
- The authored run uses `OMNISIM_NEWTON_FINGER_KE=2000` (N/m). The engine's
  default slider spring was too compliant for this setup. The 15 N limit
  remains in the model; the higher gain does not establish real servo behavior.

## Recorded episode and gripper mapping

The recording is [lerobot/aloha_static_battery](https://huggingface.co/datasets/lerobot/aloha_static_battery),
revision `06dc3da83c4fd3d1889b00f1dfd3780da8421f64`, episode 0: 600 frames at
50 Hz. The task is to place a battery into the slot of a remote controller.
The left arm braces the remote in the real recording while the right inserts
the battery. The dataset declares Apache-2.0. Saved videos, parquet, metadata,
license and source URLs are in the [study folder](../../../../sim-to-real/aloha-battery/source).

The 14-dimensional action/state layout is six arm joints and one normalized
gripper value per arm. Body angles are passed through in radians. The
hardware constants and `real_env.py` are pinned from
[ALOHA](https://github.com/tonyzhaozh/aloha), commit
`06369f03cd8e0a47e16d3a90167853fd33af7557`; see its
[MIT license](source/hardware/LICENSE).

Measured gripper state is denormalized with `0.01844 + value * 0.03956` metres.
The real action instead denormalizes to a gripper servo angle. Applying the
same linear formula to actions is an **estimated slider-command mapping**;
it does not reconstruct the nonlinear hardware linkage. Each slider value is
mirrored to the second finger. Negative source values are retained.

Three modes remain separate:

1. `recorded-actions`: recorded arm actions plus the estimated gripper mapping.
2. `measured-arms`: measured arm states used as new motor targets, plus recorded
   gripper commands. This is not recorded-action replay.
3. `authored`: an independently authored Cartesian/IK trajectory. It uses a
   vertical grasp and slower carry instead of reproducing the real wrist path.

## Scene and evidence

The table is estimated 10 mm above the arm mounting plane in the comparison.
Battery dimensions are estimated at 49 x 13.6 mm, mass 23 g. Its inertia is
the analytic solid-cylinder estimate, with explicit centre of mass. The remote
is an estimated 134 x 41 x 19 mm compound body, mass 55 g. The receiving slot
has 54 x 16.5 mm clear horizontal dimensions and a 6 mm floor. The already
installed battery is visual geometry rigidly attached to the remote.

There are **no battery springs, electrical terminals or retaining contacts**.
The authored motion lowers the battery above the compartment and opens the
gripper; gravity completes placement. It does not perform spring compression
or the angled seating motion of a spring-loaded battery compartment. The
reported task success certifies this simplified grasp-and-placement task only,
not faithful reproduction of the real insertion mechanism. Finger servo
stiffness is unrelated to a battery-compartment spring.

Physics uses Newton/MuJoCo CPU, a 2 ms step, friction 3, elliptic contacts,
impratio 10, five noslip iterations, contact dimension 6, and battery rolling
friction 0.0001 m. None of these values were calibrated against the real robot.

Both arms receive `Motor.setPosition` targets at 50 Hz. Props are moved aside
during initial arm settling and restored before the timed episode. There are
no prop pose writes, attachments or welds during the episode. Captures and
contact/pose traces are sampled after each 20 ms command interval; the physics
still steps at 2 ms. Reported speed maxima are sampled values, not bounds on
unobserved substeps.

The task check requires sustained bilateral airborne finger contact, at least
80 mm of horizontal transport, full-cylinder containment in the remote frame,
release and settling. A separate strict grip-quality check flags more than
10 mm of relative movement within the fingers. Task completion can pass while
that stricter quality check fails; the measured slip is always reported.
An open-gripper run uses the same motion to test whether the fingers caused
the carry. See the saved [verification](../../../../sim-to-real/aloha-battery/evidence/verification.json)
for outcomes, machine attribution and remaining limitations.
## Spring reconstruction variant

The separate `_spring.urdf` models keep the arm joints, inertias and source
meshes. Their estimated pads sit at finger-local y = +/-26 mm to clear the
tapered jaw tips. The spring compartment, flat-ended battery collider,
friction estimates, passive terminal force law and state-feedback controller
are new authored components; their full parameters and limitations are in
[MECHANISM.md](MECHANISM.md). They are not CAD or calibrated force data from
the filmed remote. Mechanical terminal contact does not establish electrical
connection or hardware transfer.
