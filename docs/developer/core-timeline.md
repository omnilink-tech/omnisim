# Time control on the in-process core: record, replay, branch, diff

A run on the in-process core (`packages/omnisim-core`) is one MuJoCo state stepped by one loop, and
everything that reaches the physics from outside goes through two doors: the actuator commands and
the supervisor's pose writes. `omnisim_core/timeline.py` records those per tick, fingerprints the
state after every tick, and keeps the full physics state every 250 ticks. A finished run can then be
replayed exactly, opened at any tick, branched with a change, and compared with another run.

```bash
python -m omnisim core record  <world> --out RUN --seconds 45      # run it on the core, keep the timeline
python -m omnisim core replay  RUN                                  # step it again from its commands; every tick checked
python -m omnisim core events  RUN                                  # contacts that began and ended, joints at a limit
python -m omnisim core inspect RUN --time 12                        # joints, bodies, contacts with forces at 12 s
python -m omnisim core branch  RUN --at-time 9 --set friction:BLOCK=0.05 --out SLIP   # the rest again, with a change
python -m omnisim core diff    RUN SLIP                             # where the two part, why, and how far
```

**Scope, first.** This is the core, not the engine and not the harness: a world has to run on the
core to be recorded (`core run`; the flagship pick-and-place does in strict mode). The engine still
has no record, no replay and no run-diff, and `POST /sim/snapshot` is
still not a checkpoint. What `docs/developer/positioning.md` says about OmniSim as a whole is unchanged
until this covers more than the core.

## What each verb promises

| Verb | Promise | How it is kept |
|---|---|---|
| `record` | Every command and pose write of the run, and a fingerprint of the positions and velocities after every tick | `Recorder`, hooked into `Core.step` |
| `replay` | The same commands on the same model give the same state, **bit for bit**, and the recording proves it | every tick's fingerprint is compared; every keyframe is restored and compared; a seek from the middle must arrive at the recorded end |
| `inspect` | The run as it stood at that tick, not a new run | the nearest keyframe is restored (the full MuJoCo integration state) and stepped forward with the recorded commands |
| `events` | What began and ended between two ticks | found by replaying and watching the contact set and the joint limits |
| `branch` | The parent, bit for bit, up to the change; a run of its own from there | the first N ticks are the parent's own records; the rest is stepped and recorded |
| `diff` | The first tick two runs differ, and whether the commands or the physics parted first | fingerprints and command rows compared tick by tick, then both runs opened where it matters |

A tampered recording is caught at the tick tampered with (pinned: one command changed by 1e-9 at tick
222 is reported as tick 222).

## A branch does not contain the controller

A branch steps the rest of the run with the **recorded** commands. The controller is not there to
react. It answers "with exactly these commands, would the grasp have held?", which is the right
question for a parameter (friction, mass, a servo gain) and the wrong one for a strategy.

To let the controller react, run the world again with the same change scheduled into the live run:

```bash
python -m omnisim core record <world> --out LIVE --seconds 45 --at-time 9 --set friction:BLOCK=0.05 --against RUN
```

`--against RUN` reports up to which tick the new run matched the old one. If that is the tick of the
change, the controller is deterministic and the comparison after it means something. If it is earlier,
the controller is not (it read a clock, a socket, a random number), and no replay by re-execution can
be trusted for it.

## The changes a branch or a live run can carry (`--set`)

| Change | Meaning |
|---|---|
| `friction:BODY=mu` | The body's colliders get this friction **and priority**, so its contacts really use it. MuJoCo otherwise takes the larger friction of two touching colliders, and in many worlds every surface carries the same world-level value: a per-body edit without priority changes nothing. |
| `mass:BODY=kg` | Mass and inertia scaled together. |
| `gain:MOTOR=scale` | The servo's stiffness and damping scaled. |
| `qpos:MOTOR=v`, `qvel:MOTOR=v` | A joint's position or velocity set at that tick. |
| `pos:BODY=x,y,z`, `vel:BODY=vx,vy,vz` | A free body moved or shoved. |
| `ctrl:MOTOR=v` | A command held from that tick on (a branch then shows the commands parting first). |

Bodies are named by their DEF, by the robot link a motor moves, or as `#id`; motors by their device
name. A name the run does not have is refused with the names it does have.

## Worked example: the gripper that drops the block

`omniarm6_real_pick_place.omniworld`, 45 s, recorded on the core (strict mode, no unserved call).

`core events` reads the run back: the fingers touch the block at 8.59 s and 8.61 s, the block leaves
the table at 9.58 s, the fingers let go at 18.18 s, it lands at 18.21 s.

`core inspect --time 12`, mid-carry: eight contacts between the two fingers and the block, 6.64 N of
normal force a side, the finger servos pushing 6.65 N, the block 0.46 m up. Its weight is 1.47 N, so
the grasp needs a friction of about 0.11.

`core branch --at-time 9 --set friction:BLOCK=0.05`: the block slips out of the fingers at 10.10 s and
stays on the pick table. `core diff` says the physics parted first, at tick 1125, with the commands
still identical, and that the block ends 476 mm from where the original run put it.

`core record ... --at-time 9 --set friction:BLOCK=0.05 --against RUN`, the controller in the loop:
identical to the original for the first 1,125 ticks, different from the change on, and the controller's
own report now says `carried: false, placed: false`. Against the open-loop branch it is identical until
tick 1127, where its commands part: the controller noticed.

## Measured

The owner's laptop, 2026-11-16, quiet machine. The run: 5,625 ticks, 45 s of simulation.

| | |
|---|---|
| Recording on disk | 112 kB (23 keyframes) |
| Cost of recording | within run-to-run noise: three paired runs, +8 %, +6 %, -5 % per tick |
| Replay of the whole run | 0.31 s (18,400 ticks per second, 147 times real time) |
| Replay with every check (`core replay`) | 0.54 s |
| Open the run at a tick | 8.5 ms median, 16.9 ms worst of 40 random ticks |
| Joints, bodies and contacts with forces at that tick | 0.8 ms |
| Events of the whole run | 0.6 s |
| Branch from 9 s to the end (36 s of simulation) | 0.30 s |
| Diff of two runs | 0.27 s |

For scale: the same world from the world file to the controller's own quit (22.6 s of simulation) took
8.5 s on the engine and 3.4 s on the core with recording on. Before this, a what-if on a finished run
meant editing the world and running it again, with nothing to say where the two runs parted.

## Limits, stated

- **The core's run, not the engine's.** A recording is exact about the run the core made. Whether that
  run equals the engine's is a separate measurement per world. On this pick-and-place the two agree on
  the outcome and on the block's final position to 0.18 µm, and are **not** bit-identical.
- **A controller's memory is not in a recording.** There is no "resume the controller from tick N":
  the open-loop branch has no controller, and the live rerun starts the controller from the beginning.
- **A MillingStock's height map is not in a recording.** A run records the axes that cut it, not the part.
- **One change tick per run.** A branch of a branch is refused: branch from the original with all the
  changes.
- **Force mode is recorded; its commands cannot be edited.** A run with torque-driven (EFFORT) joints keeps
  every tick's torques, the gearbox bindings and each rotor's state at every keyframe, so replay, seek and
  branch stay exact. A `ctrl:` or `gain:` edit on an effort joint is refused.
- **Open-loop replay needs the same MuJoCo build.** The fingerprints are of bits; another build of the
  physics library may round differently, and `replay` will say so at the first tick it does.
- **Inputs the recording never saw** (a controller reading a socket, a clock, a file) make a live rerun
  differ before the change; `--against` reports the tick.

Pinned by `tests/test_omnisim_core_timeline.py`.
