# The roll-check: making "the wheels actually roll" a testable property

**Status:** shipped 2026-08-10. Enforced by the pre-push smoke lane
(`tests/physics/worlds/wheel_roll_noslip.omniworld`) and by
`tests/test_wheels_actually_roll.py`. Tooling: `scripts/dev/roll_check.py`.

---

## 1. The defect class, and why nothing caught it

A hand-authored 4-wheel rover in this repo drove its body forward at 1.0–1.6 m/s
while its four wheel hinge DOFs sat at ~0.14 rad/s. At the authored 0.08 m wheel
radius, rolling that fast would need 13–20 rad/s. **The body was sliding, not
rolling** — motor torque going into the chassis through the joint reaction
instead of into the ground through the tyre. It had been that way for the
world's entire life.

The interesting part is not the bug, it is why it survived. **Every check in
this repo asks only whether the body MOVED.**

| check | what it asserts | does a sliding robot pass? |
|---|---|---|
| `run-headless` PASS | the world loaded and stepped, no ERROR lines | yes |
| `run-headless --fail-on-runaway` | no body left the world | yes |
| displacement assertions (`mobile_drive_1m`, swarm demos) | the robot got ≥ N m from where it started | yes |
| AgentBench R1 graders | reached the goal, hit nothing | yes |
| OmniBench lane 1 | analytic scenes (falling, rolling ball, pendulum) — **no driven wheel anywhere** | n/a |
| the smoke lane before this change | gravity, rest height, one hinge angle | n/a |

Nothing anywhere asserted that **a wheel turned**. That is the hole this closes.

---

## 2. What the check asserts

Behavioural no-slip consistency, on a robot that is being driven straight:

```
| omega_wheel * r  -  v_body |  <=  TOL * max(|v_body|, V_EPS)
```

Both sides are measured. Neither is predicted.

> ⚠️ **Do not replace this with a stiction/torque formula.** The obvious one is
> wrong. The failing rover had `maxTorque 0.4` on each of four wheels against
> 3.6 kg, i.e. roughly 5.6 m/s² of nominally available traction, which should
> have been ample — and it slid anyway. Worse, the same 0.4 N·m **rolls
> perfectly** on the gate world (§5). The failure is not a torque threshold at
> all; it is an interaction between the torque budget and the integration step,
> and no friction algebra predicts that.

### Directions and signs

Each wheel defines its own forward unit vector `f = a_world × up`, where
`a_world` is the authored `HingeJointParameters.axis` rotated into world
coordinates by the **robot's** orientation. For the ENU-standard `axis 0 1 0`
wheel on a +X-forward robot this yields +X, and it stays correct for a wheel
authored on any other axis or a robot spawned at any yaw. `v_roll = omega * r`
is compared against `v_body · f`, so both sides live on the same axis and the
sign is meaningful: a wheel turning backwards relative to the body's travel is
not "nearly rolling", it is 200 % slip (verdict `REVERSED`).

### Where `omega` comes from

The wheel Solid's own rigid-body angular velocity, **minus the chassis's**:

```
omega_spin = (omega_wheel - omega_body) · a_world
```

Subtracting the chassis is not optional — a robot yawing at 2 rad/s would
otherwise read 2 rad/s of "wheel spin" while its wheels were welded solid.

A `PositionSensor` would also work and is the more obvious route, but it belongs
to the robot that owns it and only that robot's own controller may read it
(`/robot/<def>/sensor/<name>` is 501 by design), so a sensor-based sweep would
have to inject a controller into every robot in the corpus. The supervisor route
measures every robot in a scene from one appended node. **The smoke gate uses
both** and cross-checks them against each other, so a sensor echoing its own
setpoint cannot pass (§4, assertion 4).

### Verdicts — the SIGN of the residual is the diagnosis

A symmetric `|omega*r - v|` test is not enough. Wheels turning **less** than the
body's motion needs is the sliding defect; wheels turning **more** is wheelspin.
Both are "slip", they have opposite causes, and conflating them produced failure
messages like *"body 0.152 m/s but wheels turn only 4.995 rad/s"* about a robot
whose wheels were spinning three times too fast — worse than no message at all.
So the verdict branches on `rolled_fraction = (omega*r) / v_body`:

| verdict | condition | meaning | sweep treats as |
|---|---|---|---|
| `ROLLING` | `|1 - rolled| <= TOL` | wheel rotation accounts for body motion | **PASS** |
| `SLIDING` | `rolled < 1 - TOL` | body outruns the wheels — **the defect** | **FAIL** |
| `WHEELSPIN` | `rolled > 1 + TOL` | wheels outrun the body: blocked, or no traction | reported, not failed |
| `REVERSED` | `rolled < 0` | wheels turn opposite to travel | **FAIL** |
| `IDLE` | neither channel moved | the drive never took effect | reported, not failed |
| `NO_DATA` | < 10 samples | the probe never saw the robot | reported, not failed |

`WHEELSPIN` and `IDLE` are deliberately **not** failures in a corpus sweep. Many
worlds park their robot against a wall, in a maze, or on a deliberately
frictionless floor, and a check that cried wolf on those would be switched off
within a week. They are still not passes, and the baseline requires a written
reason for each (§6).

### Only the samples where the robot was moving are graded

Half this corpus is multi-robot combat arenas. Driven straight ahead they reach
a wall or each other inside the measurement window and stop, and a median over
the whole window then measures the wall rather than the wheels. The grader
therefore filters to samples with `|v_body| >= MOVE_EPS` and asks the question
that is actually interesting — *while it was moving, did its wheels account for
the motion?* — provided at least 10 such samples exist. `moving_fraction` is
recorded in every row, so a verdict resting on a brief window is visible rather
than implied.

---

## 3. The tolerance, and how it was chosen

`TOL = 0.35` — the fraction of the body's own speed that may go unaccounted for
by wheel rotation.

It is not a guess and it is not fitted to make anything pass. It was chosen from
the **measured distribution** over every hand-authored wheeled robot in the tree
— 63 graded robots across 57 worlds. Sorted, the slip ratios fall into three
groups separated by two empty bands:

| slip ratio | n | what lives there |
|---|---|---|
| 0.0004 – 0.3018 | 16 | healthy: rovers, huskies, e-pucks, battlebots; the gate world at 0.0004 |
| *empty: 0.302 – 0.359* | | |
| 0.3586 – 0.4206 | 3 | **marginal — and all three are MyBots in worlds running the engine's default 32 ms step with one substep**, i.e. early cases of the defect, not healthy robots. Re-graded after that world's timestep was fixed they read 0.142 / 0.169 / 0.249. |
| *empty: 0.421 – 0.786* | | |
| 0.7863 – 15.7 | 44 | broken: sliding, wheelspin, launched |

0.35 sits in the first empty band, so no healthy robot measured here is within
14 % of failing, the marginal three are on the correct side of it, and the
negative control (0.602–0.646) is 1.7× over.

It is deliberately **loose**: a driven wheel legitimately spins slightly faster
than ground speed (positive slip is how a tyre makes tractive force at all), a
skid-steer chassis scrubs, and a Cylinder collider's effective rolling radius is
not exactly its authored radius.

> ⚠️ **Honest limit, not papered over.** The margin above the worst healthy robot
> is only **1.16×**, and that worst case (0.3018) is `HUGE` in
> `huge_vs_bitebot.omniworld` — a battlebot being *shoved by its opponent*, not a
> clean roller. Restricted to single-robot worlds, the ones a gate is actually
> built on, the healthy population tops out at **0.2324** and the margin is
> 1.5×. **If a world ever lands between 0.30 and 0.35, do not retune the
> tolerance — investigate**, because the only things measured in that
> neighbourhood are robot-on-robot collisions and coarse-timestep defects.

One measurement worth keeping for its own sake: in `drive_test.omniworld`, `DEF FLAP`
(a bare `Cylinder` boundingObject with the 90° rotation on the wheel Solid's own
body frame) measures **0.181**, while its sibling `DEF ROLL` — identical except
that the rotation and the collider are wrapped in a `Pose` — measures **0.001**.
The check quantifies the cost of the bad authoring convention at ~180× more
slip, without failing it.

Two supporting constants:

* `V_EPS = 0.01 m/s` — the floor of the denominator, an order of magnitude below
  the corpus's slowest legitimate rover (the 2.5 cm-wheel e-puck family runs at
  ~0.1 m/s), so a slow robot is still graded rather than excused.
* `MOVE_EPS = 0.03 m/s` — below this in both channels the verdict is `IDLE`
  rather than a ratio. "It did not move" and "it moved without rolling" have
  completely different causes and a check that conflates them is noise.

---

## 4. What runs where

### The gate (pre-push, every push)

`tests/physics/worlds/wheel_roll_noslip.omniworld`, in `tests/smoke/smoke_worlds.json`,
graded by `tests/physics/controllers/wheel_roll_noslip/`. ~3 s wall clock.

The rover is **copied** from `DEF ROLL` in
`projects/robot_combat/worlds/tests/drive_test.omniworld` — the repo's known-good
roller — so a red here means the engine changed, not that the gate was
mis-authored. Four assertions:

1. **It actually drives.** `|v| >= 0.15 m/s`. A world where nothing moves
   satisfies no-slip trivially (`0 == 0`), so grading one is refused. A vacuous
   green is worse than a red.
2. **No-slip on velocities.** `|omega*r - v| <= 0.35*|v|`.
3. **No-slip on the integrals.** Wheel angle travelled × r vs the chassis's net
   displacement over the same window. Same physical property, computed with no
   differentiation anywhere, so it cannot be a sampling artefact. The defect
   misses this one by ~100×.
4. **The two readings of `omega` agree.** The `PositionSensor` differentiated,
   vs the wheel Solid's own angular velocity de-rotated by the chassis. A sensor
   that merely echoed its setpoint would pass 2 and 3 and fail this.

The wheel **radius is read from the scene's own `boundingObject`**, never
hardcoded — otherwise the gate stops checking the world and starts checking that
the controller agrees with itself.

### The sweep (on demand)

```bash
python scripts/dev/roll_check.py scan                  # static candidates
python scripts/dev/roll_check.py run <world.omniworld>       # measure one
python scripts/dev/roll_check.py sweep <world.omniworld>...  # measure many
python scripts/dev/roll_check.py --self-test           # prove it can go RED
```

The world is **never modified**. A throwaway sibling `.omnisim_roll_<stem>.wbt`
is written next to it (a sibling because `URDFRobot { url ... }` and relative
PROTO/texture paths resolve against the world file), with two edits: every
`controller "..."` is swapped to the uniform `roll_drive`, and a `roll_probe`
supervisor Robot is appended. `OMNISIM_ROLL_OMEGA` is set from the world's own
statically-parsed wheel radius so a 2.5 cm e-puck wheel and a 30 cm battlebot
wheel are driven at comparable *ground* speed. The sibling is deleted afterwards.

Swapping *every* controller — not just the wheeled robots' — is deliberate: a
world's own controllers may steer, stop, quit the simulation, or block on a
bridge that is not running. Replacing them all makes every world answer the same
question ("when all wheels are told to spin forward, does the body roll?")
instead of each world's author's question.

### The cheap guards (`pytest`, milliseconds, no engine)

`tests/test_wheels_actually_roll.py`:

* the gate is still in the smoke lane and not silently `skip: true`;
* the negative control is still **derivable from** the gate world by the
  recorded edit list — a drifted control turns `--self-test` into two runs of
  the same world, which passes forever and proves nothing;
* **every hand-authored wheeled world in the tree is in
  `tests/goldens/roll_check_baseline.json`** — a new one fails the test until it
  has been measured;
* every non-`ROLLING` baseline entry carries a written reason.

---

## 5. Proving it can go red

`scripts/dev/roll_check_assets/wheel_roll_slip_negative_control.omniworld` is generated
from the gate world (`roll_check.py --regenerate-negative-control`).
`roll_check.py --self-test` runs the pair and requires **opposite** verdicts.

**It takes all three physical edits to make the rover slide**, and that is the
most useful thing this exercise produced. Measured on `omnisim-bin-fixed.exe`,
one factor at a time, everything else held at the gate world's proven-good
values:

| variant | slip ratio | verdict |
|---|---|---|
| gate world (`maxTorque 12`, `basicTimeStep 8`, `newtonSubsteps 4`) | 0.000 | ROLLING |
| `maxTorque 0.4` alone | 0.000 | ROLLING |
| `basicTimeStep 16`, `newtonSubsteps` removed — alone | 0.001 | ROLLING |
| **all three together** | **0.602–0.646** | **SLIDING** |

So "the motor was under-powered" is only half the story. 0.4 N·m is ample at
dt = 8 ms with 4 substeps, and a 16 ms single-substep integration is fine at
12 N·m. The failure is the **interaction** — a torque budget too small to
correct the contact error a coarse integration accumulates in one step.

This also explains, in retrospect, why the fix to the original world changed
`basicTimeStep 16 → 8` and added `newtonSubsteps 4` *alongside* the torque: all
three mattered.

A fourth, non-physical substitution swaps the control's `controller` field to
`roll_drive` so a stray direct run cannot append a `FAILURE` line to
`tests/output.txt`. `roll_check.py` swaps every controller anyway, so both worlds
are measured under identical drive conditions.

---

## 6. The corpus sweep

**62 hand-authored wheeled worlds** across `projects/` and `tests/`.
`tests/goldens/roll_check_baseline.json` is the machine-readable record (verdict,
body speed, wheel speed, slip ratio and a written reason for every non-pass).

Scope: a `Robot` with ≥ 2 `HingeJoint`s that each have a `RotationalMotor` and an
endPoint `Solid` with a `Cylinder`/`Sphere` `boundingObject`. `URDFRobot` and
PROTO-instanced robots never qualify — their wheels are not in the world file —
which is the intended scope, since this defect class is about hand-written wheel
stanzas. Frozen trees are excluded by path (`/results/`, `/artifact/`,
`/deliverable/`, `/repo_artifacts/`, `.harness_*`, `_agentbench_*`): a benchmark
result directory records what an agent produced on a given day, and editing it
destroys the record.

### 6.1 What the sweep found

**The `projects/samples/devices/` family — 31 shipped sample worlds — was
physically broken, and had been for as long as they have run on Newton.** These
are the converted Webots MyBot/e-puck device examples. They declare no
`basicTimeStep`, so they ran at the engine default **32 ms with one substep**,
and a 2.5 cm wheel cannot survive that. Measured peak body speeds *before* the
fix, for robots whose wheels were being told to turn at 10 rad/s (= 0.25 m/s):

| | before | after |
|---|---|---|
| peak body speed across the family | **48.35 m/s** (`connector.omniworld`, wheel at 708 rad/s) | **0.304 m/s** |
| worlds where a 5 cm robot exceeded 1 m/s | **21** | **0** |
| `bumper.omniworld` | 5.16 m/s body, 2.91 rad/s wheels | 0.30 m/s body, 10.12 rad/s wheels, **ROLLING** |

This is **not** an artifact of the sweep's drive protocol. Run with `--no-swap`,
i.e. with its own shipped controller and no command from the sweep at all,
`bumper.omniworld` moved its MyBot at **2.96 m/s while the wheels turned 1.03 rad/s**
— 99 % slip. Nothing caught it because these are device-API tests: they assert
what a sensor reads, never that the robot moved sanely.

The fix applied to 33 worlds (31 device + 2 `projects/languages/*/worlds/example.wbt`)
is `basicTimeStep ≤ 8` + `newtonSubsteps 4`. **24 of 33 strictly improved, 9
unchanged, none regressed.** Two reached a clean `ROLLING`; most of the rest
moved from `REVERSED`/launched to `WHEELSPIN`, which on re-grading the same
samples over the first 1.2 s (before the robot reaches its very small arena's
wall) becomes `ROLLING` for five more. The residual is arena length, not wheels.

Two worlds still fail after the fix and are recorded as such:
`projects/languages/{cpp,python}/worlds/example.wbt`, where the launch is gone
(12.6 → 0.14 m/s) but the wheels now barely turn at all (0.01 rad/s). Not
root-caused.

**Multi-robot combat arenas are inconclusive by construction.** The sweep drives
every robot forward simultaneously, so in `robot_combat` worlds they reach each
other and the walls inside the window; the body-speed side of the ratio then
measures the collision. Their wheels all track the commanded rate, and the
isolated equivalent of the same chassis —
`projects/robot_combat/worlds/tests/drive_test.omniworld` — measures `ROLLING` at
0.001 slip. Grading these properly needs a one-robot-at-a-time protocol that
does not exist yet.

### 6.2 Deliberately not fixed

* **Frictionless by design** — `accelerometer.wbt`, `altimeter.wbt`,
  `supervisor_set_position_of_dynamic_object.omniworld` declare `newtonGroundMu 0`. A
  robot that cannot generate traction is the point of the scene.
* **`tests/` worlds with goldens** — `tests/api/worlds/*`,
  `tests/physics/worlds/dynamic_*_rays.wbt`, `ball_joint_vs_hinge_joints.omniworld`.
  Several are as badly broken as the device family (`DYNAMIC_ROBOT` reads
  **25.2 m/s** with its wheels at 0.09 rad/s), and the same timestep change would
  very likely fix them — but it can move numbers those tests assert, so it
  belongs to whoever owns them.
* **`tests/benchmarks/agentbench/.../r1_*.wbt`** — another lane's tree this
  session. Measured from copies in a scratch directory instead, so the tree was
  never written to. Their fix has landed and works: `r1_probe` (the same rover on
  open ground) measures **ROLLING at 0.001 slip** with its wheels at the
  commanded 5.625 rad/s; the other four read `WHEELSPIN`/`REVERSED` only because
  they are obstacle courses.
* **`projects/robot_combat/**`** — see the multi-robot caveat above.
* **`coupled_motors.omniworld`** — the probe never produced a document, before or after
  the fix (2.5 mm wheels; the run does not finish inside a 100 s budget).
  Recorded as `ERROR`, not as a pass.

---

## 7. What was NOT measured

* **The stock `omnisim-bin.exe`.** Everything here was measured on
  `omnisim-bin-fixed.exe`. Another lane was running the stock binary
  concurrently and it was off-limits, so it was never launched. `tests/test_suite.py`
  hardcodes `omnisim-bin.exe`, which means **the smoke gate has not been run
  end-to-end on the binary the pre-push hook will actually use.** The gate world
  is a copy of a configuration that rolled on older builds (`drive_test.omniworld`'s
  `DEF ROLL`, 11 m of measured rolling in `36ae8fa86`), so it is expected to be
  green there — but expected is not measured. First person with the stock binary
  free: run `python tests/smoke/run_smoke.py --nomake`.
* **The full smoke lane.** `run_smoke.py` aborts when TCP 1234 is occupied, and
  it was, throughout. The gate world was instead run directly against the engine
  and verified to write exactly one `OK: wheel_roll_noslip` line to
  `tests/output.txt` — which is the thing `run_smoke.py` greps for — but the lane
  itself was not exercised.
* **Whether the 33 fixed sample worlds still do what they demonstrate.** The fix
  changes only `basicTimeStep`/`newtonSubsteps`, and every controller's
  `TIME_STEP` in that family is a multiple of 8, so no controller loop changes
  shape. Each world was re-run and loads and steps. But nobody watched a camera
  demo to check it still frames what it used to.
* **`mujoco_warp`.** Every measurement is CPU `mj_step`. The GPU path is
  known non-deterministic (`docs/benchmarks/determinism-scope.md`) and was not
  swept.
* **Turning.** The check drives straight. Skid-steer *turning* legitimately
  produces large slip and is out of scope; a future `--turn` mode would need its
  own tolerance derived from track width.
* **The mechanism of the interaction in §5.** It is reproduced and bracketed,
  not root-caused. Nothing here should be read as an explanation of *why* a
  coarse step plus a small torque budget produces sliding.
