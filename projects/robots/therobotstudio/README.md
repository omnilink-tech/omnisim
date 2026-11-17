# SO101 physical pick and place

Saved videos, source recordings and run evidence are in
[sim-to-real/so101](../../../sim-to-real/so101/README.md).

The authored demo approaches a 40 mm cube, pinches it, lifts and carries it,
then releases it inside a pink box. It uses contact friction and motor targets;
there is no payload attachment or timed pose correction. Its 21-second motion
is authored for simulation. **It is not an exact replay of the real LeRobot
recording**; that comparison remains unverified because calibration is missing.

Run the demo directly:

```sh
python -m omnisim run-world projects/robots/therobotstudio/worlds/so101_pickplace.omniworld
```

For capture and machine-readable verification, start a dedicated harness and
use an empty output directory for each run:

```sh
python -m omnisim harness --port 6889 --supervisor-port 6890
python projects/robots/therobotstudio/pickplace_demo.py --harness http://127.0.0.1:6889 --out-dir outputs/so101/demo
python projects/robots/therobotstudio/pickplace_demo.py --harness http://127.0.0.1:6889 --out-dir outputs/so101/control --no-grip --no-render
python projects/robots/therobotstudio/make_video.py --run-dir outputs/so101/demo --output outputs/so101/demo.mp4
```

Python's standard library suffices for this demo; video encoding requires
FFmpeg on PATH. The robot controller uses the simulator's Python bindings.
Opening the world directly writes evidence to a new `omnisim-so101-*` temporary
directory, printed by the controller. The CLI gives you a predictable output
directory and a nonzero exit status if verification fails. A no-grip run passes
its control check only when the cube remains settled without a lift, carry or placement.

`trace.jsonl` contains every frame's achieved joints, cube pose and attributed
contacts. `result.json` requires at least 0.5 seconds of bilateral airborne
contact, at most 15 mm of relative slip, no cube speed above 2 m/s, full cube
containment, release from the jaws and a settled final state. The 2 m/s gate is
a gross launch detector, not a hardware speed specification. `manifest.json`
records source/world/model hashes. Capture is native OmniSim rendering at 30 Hz.

Measured on machine `9722d23d12a3` (RTX 3060 Laptop GPU, CPU MuJoCo path):
4.13 seconds of continuous bilateral airborne contact, 6.61 mm maximum relative
slip, successful settled placement; the open-gripper control did not lift.
Repeated runs with and without capture produced identical final cube positions.
See [verification.json](verification.json) and [model provenance](so101/PROVENANCE.md).

## Investigating the recorded episode

Install `pandas` and `pyarrow`, obtain the episode parquet from the pinned
[dataset revision](https://huggingface.co/datasets/hbseong/record-pick-and-place-so101/tree/3c0578a2dd3eb843b861b9398687b3c012d3070e),
and provide an explicit calibration profile:

```sh
python projects/robots/therobotstudio/lerobot_replay.py --parquet episode.parquet --episode 0 --calibration projects/robots/therobotstudio/estimated_calibration.json --harness http://127.0.0.1:6889 --out-dir outputs/so101/replay
python projects/robots/therobotstudio/make_video.py --run-dir outputs/so101/replay --real episode_right.mp4 --output outputs/so101/comparison.mp4
```

The example calibration is **estimated and currently does not reproduce a
successful grasp**. The script returns exit code 2 for that physical failure,
even when all 457 frames complete. Units are never auto-detected, the gripper
has its own conversion, and out-of-limit commands are rejected rather than
silently clamped. Calibration profiles declare `units`, `status`, and an
affine degree scale/offset for every joint. An `observation.state` replay is a
kinematic reference, not the same commanded-input experiment as `action`.

The side-by-side encoder labels estimated calibration and measured failure.
It refuses to present the authored demo as a replay comparison. The original
real video's successful grasp does not make the simulated replay successful.

Run the focused engine-free checks with:

```sh
python -m unittest discover -s tests -p test_so101_replay.py -v
```
