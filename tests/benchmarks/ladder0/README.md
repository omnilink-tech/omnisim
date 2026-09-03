# ladder0 — a first-principles physics ladder

Twelve rungs. Each has a numeric ground truth **derived from Newtonian
mechanics and the geometry the scene declares** — with exactly one exception,
rung 18, whose ground truth is a **recorded measurement of physical reality**.
Checkable without judgement. Hand authored, no agent in the loop. Fast,
deterministic, runnable on every commit.

```
python tests/benchmarks/ladder0/run_ladder.py                 # every arm present
python tests/benchmarks/ladder0/run_ladder.py --sims omnisim  # one arm
python tests/benchmarks/ladder0/run_ladder.py --rungs 3,4
python tests/benchmarks/ladder0/run_ladder.py --rungs everything  # + the on-demand rungs
python tests/benchmarks/ladder0/run_ladder.py --self-test     # prove it can go RED
python tests/benchmarks/ladder0/run_ladder.py --regen-worlds  # after editing rungs.py
python tests/benchmarks/ladder0/run_ladder.py --list-binaries # engine A/B candidates
python tests/benchmarks/ladder0/run_ladder.py --sims omnisim \
    --omnisim-bin msys64/mingw64/bin/omnisim-bin.next.exe     # A/B one build
python tests/benchmarks/ladder0/omnisim/variants.py r11warp   # the GPU study
```

`--rungs all` is the **every-commit** set (0–9, 11). Rung 18 launches one
engine per toss and costs minutes; ask for it by number, or with
`--rungs everything`.

Exit code is 0 only when every requested cell is green. An arm that is present
but produced nothing is **red**; an arm that is absent is reported as *not run*,
which is not a pass — an unrun arm is an unknown. A check an arm has
**declared** it cannot express is `N/E`: not green, not red, and it does not
set the exit code (see [CONTRACT.md §5a](CONTRACT.md)).

## Layout

| path | what |
|---|---|
| `rungs.py` | **the contract.** Scene constants, analytic ground truth, tolerances with their physical derivations, `check_rung()` |
| `analysis.py` | the only reducer: sample document → measurement |
| `run_ladder.py` | entry point, arm discovery, table, exit code |
| `selftest.py` | the red proofs |
| `CONTRACT.md` | the sample-document schema, the arm interface, the module-loading rule |
| `test_ladder_hygiene.py` | tests of the ladder's own plumbing (`pytest`, no simulator) |
| `omnisim/` | the OmniSim arm: `arm.py`, `worldgen.py`, `rung18.py`, `worlds/`, `controllers/` |
| `webots/`, `mujoco/` | the other arms, same interface |
| `omnisim/variants.py` | runs a rung under NON-DEFAULT engine settings, one field at a time. **Never a ladder row** — it exists so a red can be chased to a mechanism |
| `PLAN_9_20.md` | the design for rungs 9 and up. Three of its eleven are built; read its status banner for where the design and the build diverged |

The MuJoCo arm also has its own entry point, `mujoco/run.py` (table + JSON +
`--integrator` sensitivity runs), and `mujoco/redcheck.py` proves its rows can
go red. `mujoco/arm.py` is the thin adapter the shared runner discovers.

`tests/benchmarks/ladder/` is a **different, unrelated** suite. Do not confuse
them.

## The rungs

| rung | scene | asserted |
|---|---|---|
| 0 | empty world | `steps_completed`, `exit_code`, `finite_clock` |
| 1 | 0.2 m cube resting on a floor whose top is at z = 0.5 | `rest_z`, `contact_penetration`, `z_drift` |
| 2 | same cube released at z = 1.6 (1.0 m of free fall) | `spawn_z`, `fall_interval`, `fall_time_abs`, `rest_z` |
| 3 | one hinge about a **vertical** axis, velocity-mode motor, no floor | `omega_driven`, `omega_zero`, `angle_travelled` |
| 4 | four driven wheels, straight line | `distance`, `wheel_omega`, `rolling_consistency`, `roll_overrun`, `ride_height`, `lateral_drift` |
| 5 | a distance sensor on a kinematically swept carrier, facing a wall | `range_static`, `range_final`, `range_tracks`, `sweep_span` |
| 6 | the rung-4 rover + a forward sensor: drive, stop below a threshold | `stop_gap`, `min_gap`, `trigger_reading`, `sensor_agrees`, `stop_creep`, `wheel_stop` |
| 7 | five rung-4 rovers in parallel lanes, five different commands | `distance_worst`, `wheel_omega_worst`, `min_separation`, `lateral_worst`, `roll_overrun_worst`, `ride_worst` |
| 8 | a Cartesian gantry gripper lifts a 0.06 m payload off a table and carries it 0.45 m | `part_rest_z`, `carry_rel`, `lift_height`, `place_x`, `hold_clearance`, `part_speed_max` |
| **9** | a 5×5 pile of cubes + a 26th dropped on the pile's outer corner, run **three times** from three processes | `repeat_delta`, `repeat_length`, `sensitivity_shortfall`, `fall_interval`, `distinct_processes` |
| **11** | the rung-4 rover at **N = 1, 4, 8, 16**, each robot with its own command and its own controller process | `distance_worst`, `wheel_omega_worst`, `roll_overrun_worst`, `ride_worst`, `lateral_worst`, `separation_shortfall`, `robots_seen` |
| **18** | replaying **50 recorded real cube tosses** onto a table | `real_pos_err`, `real_rot_err`, `tunnel_depth`, `replay_ic_fidelity`, `tosses_unscored`, **`embed_gap`** |

Every expected value is analytic — **except rung 18's, which come from a
measurement of physical reality** and are admitted by exactly one narrow
amendment ([CONTRACT.md §8](CONTRACT.md), F). `rungs.py` carries the derivation
next to each constant; nothing in it was ever read out of a running simulator,
and a measurement of a simulator remains forbidden, including ours and
including a previous build. A golden captured from today's behaviour would have
certified every defect found in the week before this was written.

**Rungs 0–8 ask *can it do this?* and that axis has converged** — three mature
engines agree to microseconds, which is the correct outcome and worth stating
plainly. **Rungs 9, 11 and 18 ask three questions where our own answer is
unknown or known-bad**, which is the only reason they were built:
[`PLAN_9_20.md`](PLAN_9_20.md) designs eleven rungs above 8 and eight of them
are deliberately *not* built, because on correctness assertions alone this
ladder cannot differentiate us from upstream Webots anywhere in tiers C or D
and can never beat MuJoCo on fidelity — MuJoCo *is* our solver.

Three design points that are not arbitrary:

* **The floor top is at z = 0.5, not z = 0** — an implicit ground plane at the
  origin is invisible to a scene whose floor is already there.
* **Rung 3's axis is vertical in a floorless scene** — gravity exerts no torque
  about the joint and nothing can touch anything, so an ideal velocity servo
  has exactly zero steady-state error and "commanding 0 gives 0" is a clean
  assertion.
* **Rung 4 asserts the whole run, not just a window.** See below.

### Why rung 4 has six checks

The brief asked for two: distance, and that the wheels actually rotate. Two was
not enough, and the ladder proved it on its first honest run.

`distance`, `wheel_omega` and `rolling_consistency` are measured over a
steady-state window (2–6 s), which is the right way to remove the spin-up
transient — but **a steady-state window can only see steady state.** Measured on
OmniSim 2026-08-12: the rover's wheels held the commanded 4.000 rad/s while the
chassis travelled at up to 4.10 m/s — a ratio of 10.25 against a rolling speed
of 0.400 m/s — for the first ~1.2 s, gaining 1.37 m of free distance, then
settled into perfect rolling for the remaining 5 s. **Every windowed check was
green.**

Two whole-run invariants catch it:

* **`roll_overrun`** — a wheel in contact with the ground can push a body at
  most `v = ωr`, and Coulomb slip can only make it slower. `v/(ωr) > 1` is
  unphysical under any friction model, so the excess over 1 is asserted to be
  zero. One-sided, needs no calibration.
* **`ride_height`** — a suspensionless rover on flat ground keeps its axle at
  `floor_top + wheel_radius`. Catches a rover being launched, and one that has
  driven off the edge of the floor.

### Rungs 5–8 — design points, and what each does NOT prove

Full argument in [CONTRACT.md](CONTRACT.md) §3a. In short:

* **Rung 5 isolates the sensor.** The carrier has no physics and its pose is
  *written* on the schedule `rungs.rung5_x_cmd(t)`, so the sensor's position is
  a scene fact rather than a simulation result. It dwells at each end because
  the three engines disagree about whether a pose written during step *k* is
  visible to a sensor read at step *k* or *k+1*.
* **Rung 6 measures the gap from the POSE, never from the sensor** — a frozen or
  fabricated sensor would otherwise report the gap it was supposed to produce.
  Its stopping budget is derived: 3 steps of latency plus friction-limited
  braking `v²/2µg`, carried at 3×.
* **Rung 7 judges each robot against the distance it would travel ALONE**, which
  *is* the non-interference assertion, and asserts separation **geometrically
  from the poses, not as a contact count** — this tree has shipped a contact read
  that returned an empty set for a scene containing 1008 contacts. It carries no
  radio: MuJoCo has no Emitter/Receiver, and a rung one arm structurally cannot
  express would produce a `NOT_EXPRESSIBLE` verdict that says nothing about the
  physics.
* **Rung 8 authors the wrist origin at the payload's centre**, so `carry_rel` is
  an analytic zero rather than a reference taken from the run. A grasp check
  that takes its reference from the run cannot tell a payload gripped correctly
  from one gripped in the wrong place and then held there.

### Rungs 9, 11 and 18 — design points, and what each does NOT prove

Full argument in [CONTRACT.md §3c](CONTRACT.md). In short:

* **Rung 9's scene is a PILE, and the geometry of the drop is load-bearing.**
  Determinism is trivially satisfied by a frozen world, so the rung carries a
  third run seeded 1 µm away that must *not* stay 1 µm away — and the known GPU
  refutation needs many simultaneous contact pairs to bite (its mechanism is an
  `atomic_add` on the contact-pair index), while a single-contact scene was
  measured reproducing bit-identical on the same path. ⚠️ **The obvious drop
  placement was measured wrong**: over the corner of the *centre* cube, a 0.2 m
  cube on a 0.201 m pitch lands squarely on a 2×2 group, and a 1 µm seed grew
  to 1.0058 µm in 8 s. The control read as *"this engine damps perturbations"*
  when the truth was *"this scene has nothing to amplify"*. Dropped on the
  pile's **outer** corner the same seed reaches **0.1455 m**.
* **Rung 11 does not assert bit-identity across N**, and refusing to is a
  decision rather than an omission: adding robots changes the size and ordering
  of the constraint system, so robot *i* can differ in the last ULP at N = 16
  with no defect at all, and a red that means nothing trains everyone to ignore
  the row. The quantity is measured and **reported** instead — and on the CPU
  path it comes back **exactly 0.0**, which is stronger than the rung is
  willing to demand.
* **Rung 18's headline is `embed_gap`, not the agreement.** Our solver *is*
  MuJoCo, so the gap between our error and the **bare MuJoCo arm's** on the same
  tosses is our translation layer, and it is the only check in this ladder we
  can lose and cannot win. The two agreement bounds are floors, set from a
  published baseline plus one published standard deviation by a paper that
  never heard of this project — and **they are not a good score**: the best
  engine measured on this data manages 13.5 % of cube width.

**Rung 9's `repeat_delta == 0` is scoped to the precision the engine's pose
readback carries, which is MEASURED to be single**: a cube authored at
x = 0.502 reads back as 0.50199997425079346 = `float32(0.502)`, so a divergence
living entirely below ~6e-8 m would be invisible to it. The refutation it
exists to see is 9.152 m by 1000 steps, eight orders above that. **Rung 9 says
nothing about cross-machine identity**, which is untested. **Rung 11 inherits
rung 4's limit** — the wheels are kinematically consistent with the motion, not
shown to have propelled it. **Rung 18 does not prove general fidelity**: every
engine handles inelastic impacts and all of them fail on elastic ones, and the
Drake > Bullet > MuJoCo ordering on rigid impacts *inverts* on cloth.

**Rungs 5 and 6 cannot distinguish a real ray from a correct bookkeeping
computation of the same quantity.** Closing that needs a scene whose correct
reading is *not an affine function of the sensor's pose*, and the "no hit"
sentinel differs across engines, so normalising it would put a decision in the
arm. Not claimed, not closed. **Rung 7 does not prove inter-robot contacts are
detected** — it proves they do not happen, and it inherits rung 4's limit below.
**Rung 8 does not measure grip force**, so it says nothing quantitative about
the engine's friction model.

### What rung 4 does NOT prove

`wheel_omega` and `rolling_consistency` establish that the wheels turn
*consistently with the motion*. **Nothing here establishes that they propelled
it.** Measured on the MuJoCo arm: cutting the wheel motors entirely still
passed, because an undriven wheel dragged over µ=1 ground rolls at exactly
`v/r`. Proving propulsion needs a different signal (motor torque, or a
zero-friction control in which a driven robot must *not* move). It is not
asserted and must not be claimed.

## The self-test

```
python tests/benchmarks/ladder0/run_ladder.py --self-test
```

A green that has never been shown to go red is worth nothing. Two proofs:

**A. Assertion mutation** (pure, milliseconds, no simulator). For every rung,
build the measurement an ideal engine would produce, confirm every check is
green, then perturb one quantity past its tolerance and confirm exactly that
check goes red and the others stay green. Plus: an unmeasured quantity must be
**red**, never skipped; and the tolerance boundary is bracketed (99.9% green,
110% red).

**B. Live fault injection** through a real engine, end to end — launch, load,
step, sample, reduce, judge. Each fault reproduces a defect this repo has
actually shipped:

| rung | fault | must go RED | must stay GREEN |
|---|---|---|---|
| 0 | `short_run` | `steps_completed` | |
| 1 | `no_floor` (the floor Solid loses its `boundingObject`) | `rest_z` | |
| 2 | `half_gravity` | `fall_interval` | `spawn_z` |
| 3 | `ignore_zero` (the stop command is dropped) | `omega_zero` | `omega_driven` |
| 4 | `slide` (chassis dragged at the right speed, wheels at zero) | `rolling_consistency` | **`distance`** |
| 5 | `no_sweep` (the carrier is never moved) | `range_final` | `range_static`, `range_tracks` |
| 5 | `wall_shifted` (the wall is authored 0.15 m further away) | `range_static` | `sweep_span` |
| 6 | `no_stop` (the stop is never commanded) | `stop_gap` | `trigger_reading` |
| 6 | `bounce` (hits the wall, then is put back at the right resting place) | **`min_gap`** | `stop_gap`, `wheel_stop`, `stop_creep` |
| 7 | `stalled_robot` (one robot's wheels commanded zero) | `distance_worst` | `min_separation`, `lateral_worst` |
| 7 | `lane_offset` (one robot spawned half a lane out of place) | `min_separation` | `distance_worst`, `wheel_omega_worst` |
| 8 | `no_grip` (the fingers never close) — **the causal control** | `lift_height` | `part_rest_z`, `part_speed_max` |
| 8 | `no_traverse` (the traverse is never commanded) | `place_x` | `lift_height`, `carry_rel`, `hold_clearance` |
| 8 | `drop_mid_carry` (the fingers reopen at the top of the lift) | `part_speed_max` | `part_rest_z` |
| 9 | `seed_nudge` (replica **b only** spawned 0.1 µm off) | `repeat_delta` | `sensitivity_shortfall`, `fall_interval` |
| 9 | `frozen` (the dropped cube loses its physics) — **rung 9's `slide`** | `fall_interval` **and** `sensitivity_shortfall` | **`repeat_delta`** |
| 9 | `short_b` (replica **b only** runs half the steps) | `repeat_length` | `fall_interval` |
| 11 | `stalled_robot` at N = 16 | `distance_worst` | `separation_shortfall`, `lateral_worst` |
| 11 | `lane_offset` at N = 16 (a **spawn** offset, never a per-step write) | `separation_shortfall` | `distance_worst`, `wheel_omega_worst` |
| 18 | `ic_drop_velocity` (the engine is handed zero velocities, the record keeps the recorded ones) | `replay_ic_fidelity` | `tunnel_depth` |
| 18 | `wrong_omega_frame` (body-frame ω handed through as world) | `real_rot_err` | **`replay_ic_fidelity`**, `tunnel_depth` |
| 18 | `table_hologram` (the table loses its collider) | `tunnel_depth` | `replay_ic_fidelity` |

**`frozen` is rung 9's `slide`**: a world that cannot move is perfectly
deterministic, so `repeat_delta` must stay **green** while the sensitivity
control and the analytic anchor both go red. Without it, rung 9 would be
satisfiable by an engine that simulated nothing.

**Rung 18's `wrong_omega_frame` must leave `replay_ic_fidelity` GREEN**, and
that is the seam: the engine accepted exactly what it was handed, so the IC
check correctly does not blame it for the harness's frame error. Only agreement
with the recording sees it. Measured: `real_rot_err` **79.26°** against a 43.1°
bound, with `replay_ic_fidelity` at **1.0e-07** and `tunnel_depth` at 0.

**`seed_nudge` found a real engine fact before it ever went red.** It was
designed at 1e-12 m — obviously unphysical — and produced a **bitwise
identical** run. Sweeping the magnitude separated "the fault never reached the
engine" from "the engine cannot represent it":

| nudge | max \|x − honest\| over the run |
|---|---|
| 1e-12 m | 0 — bitwise identical |
| 1e-09 m | 0 — bitwise identical |
| 1e-08 m | 0.0109 m |
| 1e-07 m | 0.0228 m |
| 1e-06 m | 0.1455 m |

**The scene pose reaches the solver in single precision.** The direct evidence
is the readback, not the sweep: a cube authored at x = 0.502 comes back as
0.50199997425079346, exactly `float32(0.502)` — re-checked on the campaign
binary. (The sweep itself was taken on the previous build; its 1e-7 row,
0.0228 m, reproduces to every digit as the live `seed_nudge` fault on the
campaign binary.) The threshold sits between 1e-9
and 1e-8 because 0.502 happens to lie 4.05e-9 below the midpoint of its two
neighbouring float32 values — so the smallest expressible perturbation depends
on the **coordinate**, not only on the exponent. The fault is now 1e-7 m, which
is **greater than one float32 ulp at that coordinate** (5.96e-8) and therefore
a guarantee rather than a margin; `test_ladder_hygiene` asserts exactly that.

The `slide` asymmetry is the entire argument for asserting more than distance,
and **`bounce` is the same argument on rung 6**: the rover hits the wall and is
then placed at exactly the gap `stop_gap` expects, so the final state is right
and only the whole-run `min_gap` can see where it has been.

**`no_grip` is rung 8's causal control and it is the thing rung 4 lacks.** Pose
assertions alone cannot tell a friction grasp from a weld; the identical scene
with the fingers never closed must leave the payload on the table, and it does
(peak payload speed 0.0016 m/s). That differential is what licenses rung 8 to
claim the fingers did the lifting.

**A `must_red` may name more than one check**, and rung 9's `frozen` is why:
a frozen world satisfies determinism, so both the sensitivity control *and* the
analytic anchor have to go red before the fault has proved anything. **A fault
row may also carry per-fault kwargs** — rung 18's faults run the contract's
small toss subset, and the baseline they are judged against runs the same
subset, or the two would not be comparable.

**Detector validation is now reported per rung** (CONTRACT.md §6a, adopted from
OmniBench lane 4b): `--self-test` prints `VALIDATED` / `UNVALIDATED` for every
rung, and **`UNVALIDATED` is not a pass**. A fault declared for a *variant*
validates the variant, not the row: rung 11's constraint starvation is a
`mujoco_warp` study in `variants.py`, so if it could not be made to bite the
**GPU variant** would be unvalidated and the CPU row — whose detectors
`stalled_robot` and `lane_offset` do validate — would not.

**A must-green companion only counts if it was green without the fault.** The
self-test runs each rung's honest cell once and reports a companion that was
*already* red as `MASKED`, so an honest fault on a rung the engine already fails
is not mis-reported as a broken ladder. The failure stays loud on that rung's own
row in the table.

Two fault-design errors the battery itself caught, both worth not repeating:
`lane_offset` began as a per-step supervisor write of the robot's `y` — which
cost the body its state and reddened both must-green companions (`distance_worst`
0.94, `wheel_omega_worst` 0.93), destroying exactly what it was meant to isolate;
and `no_stop` originally required `sensor_agrees` to stay green, but a rover that
never stops ends with its sensor **buried inside the wall**, where "distance to
the near face" is a distance to a surface behind the ray (residual 0.293 m there
against 2.8e-06 on the honest run).

The self-test has already found two defects in the ladder itself before any
simulator ran: an exact-tolerance-boundary case that tested floating-point
rounding rather than the tolerance, and a check whose mutation key did not
match the key it read (so it could not be made to fail).

## Cost and cadence

Measured on machine `9722d23d12a3`, warm, one arm at a time.

| rung | sim seconds | bodies | omnisim | webots (ODE) | mujoco | cadence |
|---|---|---|---|---|---|---|
| 9 | 3 × 8 s | 26 | 22.6 s | 10.6 s | 2.3 s | **every commit** |
| 11 | 4 runs, 6.5 s each | up to 80 | 43.8 s | 30.5 s | 2.1 s | **every commit** for N ≤ 16; N = 32 is a variant |
| 18 | 50 × 0.818 s | 2 | **341 s** | *not implemented* | 5.2 s | **on demand / release**, never a commit hook |

Rungs 9 and 11 add ~65 s per commit on the slowest arm, the same order as
rungs 0–8. **Rung 18 is dominated by 50 process launches** — 6.8 s of wall per
0.8 s of physics — which is why it is not in `--rungs all` and has to be asked
for by number.

The fault batteries cost more than the rows: rung 9's is 4 cells × 3 launches,
rung 11's is 3 cells × 4 launches (N = 16 twice), and rung 18's runs the
contract's 6-toss subset rather than 50, which is the only reason it is
minutes instead of half an hour.

Two per-arm notes that "cheap in steps" hides. The upstream-Webots arm runs
R2025a under WSL2, and its **cold** first run was ~1.7× its warm one (26.0 s of
stepping at N = 16 against 15.4 s). At N = 16 it runs 2.4× slower than
realtime, with a marginal cost of ~0.95 s of wall per rover. The MuJoCo arm has
no process to start at all, which is why its startup column is genuinely ~0
rather than missing.

## The plumbing tests

```
python -m pytest tests/benchmarks/ladder0/test_ladder_hygiene.py
```

The self-test proves the ladder's *assertions* can go red. These prove its
*plumbing* is honest — and every one of them is a defect that was live after
the three arms were built in parallel, none of which any green row could see:

The tier-C/E rungs added eight more, each pinning a defect a green row cannot
see: rung 9's two replicas must resolve to **one file**; the 0.1 µm fault must
survive the scene file **and clear one float32 ulp at its own coordinate**;
rung 11's lanes and budget must **move when the contract moves**; the budget
must be generous rather than the peak; rung 18's toss subset must be one the
**contract owns**; no dataset constant may be re-declared here; the ladder's
stdlib reducer must **agree with lane1r's numpy scorer** on a closed-form case;
`embed_gap` must never be computed against ourselves; and a sample document
must round-trip float64 exactly.

* an arm that fails to import must be **reported**, not crash the runner
  (`load_arm` read the `except ... as exc` name from a closure that outlives
  the block, so the reporting path raised `NameError` — and that crash hid the
  next defect);
* no arm may hold another arm's modules. All three ship a `worldgen`;
  `sys.modules` is global to the runner's process, so a bare `import worldgen`
  in the second arm loaded returns the **first** arm's generator. That arm
  would generate *and measure* another simulator's scenes and report them as
  its own. `run_ladder.load_arm` now refuses it (`ArmImportCollision`, exit 2)
  and reports the weaker "an arm-local module is registered under a collidable
  name" case as an IMPORT HAZARD;
* every scene number comes from `rungs.py`. Not just equal to it *today* —
  the test moves the contract and requires the emitted scene to move with it,
  because a hard-coded copy passes an equality check. The MuJoCo arm's
  committed models had drifted to a floor from an earlier `FLOOR_SIZE`, and a
  scene that has drifted is judged against the wrong expectation.

## Measured — 2026-08-13, all three arms (rungs 9, 11 and 18 — the tier-C/E campaign)

Machine `9722d23d12a3` (RTX 3060 laptop, AMD64 16 core), engine
`msys64/mingw64/bin/omnisim-bin.exe` sha256 **`cbd80861c9f0d314`** (2026-08-13
10:34:49), MuJoCo 3.8.1 CPU `mj_step`, upstream Webots R2025a / ODE under WSL2
at `optimalThreadCount 1`. **A DIFFERENT BINARY from both earlier campaigns
below** — every row records its own sha, which is the only reason the three are
separable.

⚠️ **The binary was rebuilt by another lane in the middle of this campaign**
(`1b82affcd3956d95` → `cbd80861c9f0d314`), which is precisely the case the
per-row sha exists for. Every number published here was re-run afterwards and
**every published row records `cbd80861c9f0d314`** — checked by reading the sha
out of each `cell.json` rather than by reasoning from timestamps. Two figures
below were first measured on the earlier build and reproduce on this one to
every digit quoted (the 145,509× amplification, and the `seed_nudge` sweep's
0.0228295 m at 1e-7); they are marked where they appear.

| rung | omnisim | webots (ODE) | mujoco |
|---|---|---|---|
| **9** determinism | **PASS** | **PASS** | **PASS** |
| **11** scale | **PASS** | **PASS** | **PASS** |
| **18** reality | **PASS** | *not run* — declared `UNIMPLEMENTED` | **PASS** |

### The cross-arm numbers, side by side

| quantity | omnisim | webots (ODE) | mujoco | judged? |
|---|---|---|---|---|
| rung 9 `repeat_delta` | **0.0** | **0.0** | **0.0** | yes, tol 0 |
| rung 9 `fall_interval` (analytic 0.147821 s) | 0.147685 | 0.147685 | 0.147685 | yes, tol 8 ms |
| rung 9 amplification of a 1 µm seed | **145,509×** | **202×** | **145,347×** | **no — reported** |
| rung 11 `distance_worst` | 8.55e-04 | 2.82e-14 | 2.48e-03 | yes, tol 0.05 |
| rung 11 `wheel_omega_worst` | 4.92e-08 | 4.80e-05 | 2.26e-03 | yes, tol 0.01 |
| rung 11 `lateral_worst` (m) | 1.86e-17 | 5.91e-06 | 6.28e-04 | yes, tol 0.10 |
| rung 11 `solo_deviation_max` (m) | **0.0** | 1.67e-15 | **3.21e-03** | **no — reported** |
| rung 18 `real_pos_err` (% of cube width) | **24.845** | — | **24.845** | yes, tol 35.9 |

Three things in that table are worth more than the verdicts.

**`fall_interval` agrees to six decimals across three independently authored
scenes and three different solvers** — 0.147685 s, a 136 µs residual against
the closed form. That is the correct outcome for rungs whose physics is
Newtonian, and it is the reason rungs 0–8 do not differentiate anybody.

**The amplification spread is 720×, and it is not a defect in anyone.** ODE
amplifies a 1 µm seed 202-fold over 8 s; the two MuJoCo-family arms amplify it
~145,000-fold. On the ODE run the dropped cube topples off the corner and the
pile is essentially scenery (`p44` moves 1.68 mm, every other cube 16 µm); the
MuJoCo-family runs put the same cube 0.145 m away. Nothing in mechanics says
how chaotic a pile *ought* to be, so this is published and **not judged** — the
check only asks that the scene amplified at all.

**`solo_deviation_max` is why rung 11 refuses to assert bit-identity across N.**
It is robot 0's along-track history at N = 4/8/16 against its solo run.
OmniSim's is **exactly 0**; ODE's is one ULP; **bare MuJoCo's is 3.2 mm**. An
assertion of bit-identity would therefore have *reddened the reference engine
and greened ours* — which is exactly the shape of a check that means nothing.
The plan's §4.5 argument was theoretical when it was written; this is the
measurement.

### Rung 9 — bitwise on CPU `mj_step`, in a scene that provably amplifies

`repeat_delta` = **0.0** exactly. That is the number the rung exists to
produce, and the two checks beside it are what stop it meaning nothing:

* `sensitivity_shortfall` = 0 — the perturbed replica reaches **0.1455 m** of
  separation from a **1 µm** seed, an amplification of **145,509×**. Float64
  round-off over 2000 steps reaches ~1e-14 relative, ten orders short, so this
  is the scene and not arithmetic.
* `fall_interval` = **0.120786 s** against an analytic 0.118286 (tol 8 ms).
  Without it, an engine that is deterministic and *wrong* would pass — gravity
  at 5 m/s² is exactly as reproducible as 9.81.
* `distinct_processes` = 3, read from the pid and process-start time each
  driver records **for itself**.

**⚠️ The gates were wrong on the first design and the control arm found it.**
They were rung 2's — 1.2 and 0.8 — where the lower gate *is* the first-contact
height. That works at rung 2's stride of 1 and does not work at rung 9's stride
of 5: the sample after the lower crossing is already contact-decelerated
(measured on ODE, t = 0.400 z = 0.807352 → t = 0.420 z = 0.792858 against a
free-fall 0.7265), so linear interpolation drags the crossing up to a full
sample interval late. It read **0.126670 s** on ODE — RED — and the *same
replica* re-run at stride 1 read **0.118282 s**, 4 µs from analytic. **The
check was grading contact hardness as fall time**: a soft contact decelerates
the cube earlier in the straddling interval than a hard one, and OmniSim's
2.5 ms and ODE's 8.4 ms differed in exactly that direction. Both gates now sit
in provable free fall, 0.2 m (2.9 sample intervals) clear of first contact, and
all three arms read the same 0.147685 s.

**⚠️ Scope, and it is not a footnote.** `repeat_delta == 0` is bitwise *at the
precision the pose readback carries*, and that precision is measured to be
**single** (see the `seed_nudge` table above). A divergence living entirely
below ~6e-8 m is invisible here. Determinism on `mujoco_warp` is **not**
measured by this row and remains refuted elsewhere; cross-machine identity is
untested.

### Rung 11 — and the njmax cliff, reproduced

The row is CPU `mj_step` and it is clean at every N up to 16, with no
N-dependent slack anywhere. `solo_deviation_max` — robot 0's along-track
history at N = 4/8/16 against its N = 1 run — is **0.0 exactly**. That is the
property the rung deliberately does *not* assert (a correct engine is not
required to produce it) and it is reported rather than judged.

**The GPU study is where the finding is.** `omnisim/variants.py r11warp`
`r11budget` `r11cpu`, all on `newtonSolver "mujoco_warp"` except the last:

| variant | budget | peak `nefc` / cap | distance_worst | lateral_worst |
|---|---|---|---|---|
| `n8_warp` | declared 4096 | 256 / 4096 | 0.000877 | **0** |
| `n8_warp_nobudget` | engine default | **256 / 256** | 0.000877 | **0** |
| `n16_warp` | declared 4096 | 512 / 4096 | 0.000877 | **0** |
| `n16_warp_nobudget` | engine default | **512 / 256 — OVERFLOW** | **0.00322 / 0.00168** | **0.0348 / 0.00511** |
| `n16_warp_starve` | engine default + spawned clear | **512 / 256 — OVERFLOW** | **0.00322 / 0.00244** | **0.0268 / 0.0175** |
| `n8_cpu_starve` / `n16_cpu_starve` | engine default, CPU `mj_step` | 256 allocated, no overflow reported | 0.000855 | 0 |

The two values in the overflowed rows are **two runs of the identical
configuration**, and their spread is the sixth finding: the degradation is
itself irreproducible, by a factor of 3 to 7. That is exactly what the engine's
own warning predicts — *"the drop order is a nondeterministic GPU atomic
race"* — so **no single degradation number should be quoted from this study**,
only its sign and its order.

Five things that settles, and one it corrects:

1. **The overflow reproduces, and the engine's own detector fires.** Peak
   `nefc` = 512 against a 256 cap at tick 25, with the latched
   `CONSTRAINT BUFFER OVERFLOW` warning. OmniBench lane 4b tried three ways to
   force this at N = 12 and honestly reported `cliff_detector_validated:
   false`; a second instrument now has it.
2. **`nefc = 32·N` exactly** — 256 at N = 8, 512 at N = 16 — so the cap runs
   out between N = 8 and N = 9. **The documented ~9-robot threshold is
   confirmed**, on a rover whose analytic target is known.
3. **The degradation is real and every assertion still passes.** Lateral drift
   goes from **exactly 0** to 5–35 mm and distance error grows 2–4×, and all of
   it stays inside rung 11's tolerances. So this rung, at N = 16, **can see the
   overflow only as a variant-to-variant difference, not as a red row** — which
   is the rung's honest limit and is written here rather than discovered later.
   The documented 9 % displacement error is not what this scene produces; it
   produces 0.17–0.32 %, and the range is the point (see above).
4. **⚠️ The briefed mechanism is corrected.** The fault was designed on the
   premise that `newtonNjmax` is a *floor* — cap = `max(requested_or_256,
   nefc_at_t0)` — so a fleet spawning **in contact** would have its cap
   auto-raised to the peak and could never overflow, and a fleet spawning
   **clear** would keep the 256 default. The floor is real and is in the
   runtime's own source. **But `initial nefc = 0` is reported on the
   in-contact scene too**, so the floor never rises above the request and the
   spawn clearance is not what matters: removing the *declaration* alone
   reproduces the overflow exactly. The first version of this variant changed
   both at once; `n16_warp_nobudget` is the single-change control that
   separates them.
5. **On CPU `mj_step` the cliff does not exist.** N = 16 with the declaration
   removed runs correctly and reports no overflow — `mj_step` sizes its own
   arena. So the honest reading is *"this is a `mujoco_warp` property"*, with
   the CPU run as the control rather than as an assumption.

**What this does NOT settle.** The ~9-robot figure was originally measured on a
10-Husky world, not on these rovers; what is confirmed here is the *mechanism*
(`nefc = 32·N`, a 256 default, truncation above it) on a different scene, which
is a second instrument agreeing rather than the same measurement repeated. And
the degradation it produces here — a third of a percent of distance — is an
order of magnitude smaller than the 9 % the original write-up quotes, so the
**size** of the effect is scene-dependent and the 9 % figure must not be
attached to this one.

### The fault battery, whole

**19 live faults on the every-commit set, all 19 red as required, 197/197
self-test cases, and every one of rungs 0–9 and 11 reports `VALIDATED`.** Rung
18's three run separately (they need one engine per toss) and are 3/3. Rung 8's
companions are no longer `MASKED` on this binary — the grasp holds, so its
faults are demonstrably surgical in the ordinary battery rather than only in
the `graspfaults` variants.

The eight new ones, with the numbers they produced:

| fault | went red at | companions |
|---|---|---|
| 9 `seed_nudge` | `repeat_delta` **0.0228 m** | `sensitivity_shortfall` 0, `fall_interval` 0.147685 — both unchanged from honest |
| 9 `frozen` | `fall_interval` **None** *and* `sensitivity_shortfall` **9** (amplification exactly 1.0) | **`repeat_delta` stayed 0.0** |
| 9 `short_b` | `repeat_length` **1000** | `fall_interval` 0.147685 |
| 11 `stalled_robot` | `distance_worst` **1.0** | `separation_shortfall` 0, `lateral_worst` 1.86e-17 |
| 11 `lane_offset` | `separation_shortfall` **0.5** | `distance_worst` 8.55e-04, `wheel_omega_worst` 4.92e-08 |
| 18 `ic_drop_velocity` | `replay_ic_fidelity` **1.0** | `tunnel_depth` 0 |
| 18 `wrong_omega_frame` | `real_rot_err` **79.26°** | `replay_ic_fidelity` **1.01e-07**, `tunnel_depth` 0 |
| 18 `table_hologram` | `tunnel_depth` **3.459 m** | `replay_ic_fidelity` 2.32e-07 |

`frozen` is the one to read: the world is made static, both the sensitivity
control and the analytic anchor go red, and **`repeat_delta` stays exactly
0.0** — which is the demonstration that a determinism check on its own can be
satisfied by an engine that simulated nothing.

⚠️ `table_hologram` also produced an incidental datum that does **not** match
rung 1's: the cube falls **3.459 m** past the table top in 818 ms, which is
unobstructed free fall. The phantom z = 0 collision plane that catches rung 1's
box at z = 0.0999 did **not** catch this one. That is one observation on one
scene and is offered as nothing more.

### Rung 18 — where the field is, and what our layer costs

50 recorded tosses, replayed from row 0 of each, scored against the recording
by the metric the published baselines use.

| | position err (% of cube width) | rotation err (deg) |
|---|---|---|
| **OmniSim / Newton → SolverMuJoCo, CPU** | **24.845** | **23.104** |
| **bare MuJoCo 3.8.1, CPU** | **24.845** | 23.104 |
| Drake (published) | 13.5 ± 8.2 | 16.5 ± 20.0 |
| Bullet (published) | 14.9 ± 8.9 | 16.5 ± 20.2 |
| MuJoCo (published) | 25.1 ± 10.8 | 21.7 ± 21.4 |
| upstream Webots / ODE | **not measured by anyone, as far as this ladder knows** | |

**`embed_gap` = 0.000127 percentage points of cube width**, against a
5-point bound. That is **0.13 µm** of disagreement between OmniSim's
Newton→SolverMuJoCo path and the bare MuJoCo it embeds, on 50 recorded real
tosses, against the same recording, reduced by the same code. It is the only
check in this ladder we can lose and cannot win, and this build does not lose
it. ⚠️ It says **nothing** about whether either engine is accurate — the two
being identical is exactly what a faithful translation layer looks like, and
both of them are 24.8 % of a cube width away from what the cube actually did.

**Read the bound before the number.** 35.9 % is the published MuJoCo row plus
one published standard deviation, set by a paper that never heard of this
project. Passing it means *our translation layer did not lose what the solver
gave us*. It does **not** mean OmniSim is accurate: the best engine measured on
this data manages 13.5 %, so the honest sentence is "OmniSim is where the field
is, and the field is not good at tossed cubes". `tunnel_depth` came back **0**
on both arms, so neither number is a cube that fell through the table.

One asymmetry worth naming rather than absorbing: `replay_ic_fidelity` reads
**3.0e-07** on OmniSim and **1.1e-15** on bare MuJoCo. Both are eight and
sixteen orders inside the 1 % bound, so neither is a defect — but the
difference is real, and it is the same single-precision pose path rung 9's
`seed_nudge` measured. Writing an initial condition through a supervisor costs
about eight digits; writing it into `data.qvel` costs none.

**⚠️ The declarations matter more than the engine here, and that is the
finding.** Rung 18's scene is lane1r's, which declares `newtonCone "elliptic"`
and `newtonImpratio 10` — R2-admissible under the fair-defaults rule, and both
are carried into the MuJoCo arm's MJCF because translating *part* of a
`WorldInfo` is replaying a different scene. The bare-MuJoCo arm's R5 defaults
sweep is unambiguous about what each one buys:

| configuration | position err (%) | rotation err (deg) |
|---|---|---|
| MuJoCo's own defaults (**the R5 datum**) | **43.023** — red | 27.864 |
| `cone = elliptic` alone | 24.845 | 23.080 |
| `impratio = 10` alone | 43.036 | 27.928 |
| elliptic + impratio 10 (**the row**) | **24.845** | 23.104 |

**The friction-cone shape is worth 18.18 points of agreement with reality and
the impedance ratio is worth 0.013.** A pyramidal cone is a polygon inscribed
in the Coulomb cone; asking for the ellipse is asking for the model the
contract already declared, not for a better one. That this shows up as an
18-point swing against *recorded reality* — not against a golden, not against
another simulator — is the most externally checkable thing in this ladder.

The MuJoCo arm discloses its order of decision, which the fair-defaults rule
requires: it measured the defaults first, saw the gap, investigated, and *then*
adopted "translate the whole `WorldInfo` or you are replaying a different
scene". That rule is derivable without the measurement; the measurement is why
anyone looked.

## Measured — OmniSim, 2026-08-12 (rungs 5–8 campaign)

Machine `9722d23d12a3` (RTX 3060 laptop, AMD64 16 core), engine
`msys64/mingw64/bin/omnisim-bin.exe` sha256 `f12be1cc3418174e`, repo `12e547b9d`.
Newton verdict sidecar on every physics rung: `finalised: true,
degraded: false`, `mujoco 3.8.1 / newton 1.2.0 / warp 1.13.0 /
mujoco_warp 3.8.0.3`, model device `cpu`. (Runtime at the time of the run; the pinned runtime is newton 1.5.0 / warp 1.16.0 / mujoco 3.11.0 — `scripts/packaging/newton_runtime_pins.py`.)

**⚠️ This is a DIFFERENT BINARY from the rung-0–4 campaign below, and rung 4's
verdict changed with it.** Nothing in a row says which build produced it unless
the build is written down, which is why both shas are here.

| rung | verdict | headline |
|---|---|---|
| 0–3 | PASS | unchanged from the earlier campaign |
| 4 | **PASS** (was FAIL) | `roll_overrun` **10.77 → 0.0322**, `ride_height` 0.0379 → 0.000171 m. The launch transient is gone on this build. |
| 5 | PASS | `range_static` 2.9, `range_final` 0.9, whole-run residual **9.7e-08 m** |
| 6 | PASS | stops at **0.4847 m** against a 0.5 m threshold — 15 mm of overshoot inside a 29 mm derived budget |
| 7 | PASS | worst per-robot distance error **0.086 %**, worst wheel-rate error 4.9e-08, min separation exactly 1.5 m |
| 8 | **PASS** (was FAIL) | `carry_rel` **0.0026 m** over a 0.45 m carry, once the scene declares the friction model per [CONTRACT.md §3b](CONTRACT.md). At the engine's own defaults it still fails, and that datum is published beside the row rather than instead of it |

### Rung 8 — a friction pinch with a 9× Coulomb margin does not hold at the defaults

The fingers close (commanded 26.7 mm, reached 26.5 mm under load). The payload
*starts* to come up with the wrist and then **creeps out**: at t = 2.0 s it
trails the wrist by 17 mm, at 2.5 s by 36 mm, at 3.0 s by 64 mm, and by t = 4 s
it is back on the table. It slides down at ~43 % of the lift speed.

This is **not** insufficient normal force. Reading the pad back: it sits 2.47 mm
inside the part's surface with 0.23 mm of servo error, so the servo and the
contact are in equilibrium at **~6 N** per pad — twice the commanded 3 N, and
18× the 0.33 N the Coulomb bound needs at µ = 3.

`omnisim/variants.py grasp` changes one `WorldInfo` field at a time against the
identical scene, so the red is attributed rather than guessed at:

| variant | `carry_rel` | `lift_height` | `place_x` | outcome |
|---|---|---|---|---|
| engine defaults (the R5 datum) | 0.4747 | 0.7278 | 0.0 | payload never leaves the table |
| `newtonCone "elliptic"` alone | 0.0213 | 0.8585 | **0.4496** | **carried the full 0.45 m**, creeps 21 mm |
| `newtonImpratio 10` alone (pyramidal) | 0.0178 | 0.8617 | 0.4492 | carried, creeps 18 mm |
| `newtonImpratio 100` alone (pyramidal) | 0.3597 | 0.7299 | 0.1226 | **worse than the default** — dropped |
| **elliptic + `newtonImpratio 10`** | **0.00256** | 0.8769 | 0.45 | **ALL GREEN** — what the arm declares |
| `+ newtonContactKe 8000 / Kd 200` | 0.1497 | 0.7315 | 0.4620 | ejected at **1.07 m/s** |
| `+ newtonIterations 150 / Ls 50` | 0.00256 | 0.8769 | 0.45 | **bit-identical** — the iteration count changes nothing |
| `+ newtonCondim 4` | 0.00256 | 0.8769 | 0.45 | bit-identical — no torque about the normal to resist here |
| `+ newtonNoslipIterations 5` | 0.00265 | 0.8789 | 0.45 | +0.09 mm — not the mechanism on this engine |

**Two fields carry the whole result, and neither is discoverable from the
scene.** The pyramidal friction cone is an inscribed approximation of the
Coulomb cone the ground truth is derived from, and at `impratio 1` the
frictional constraint is as soft as the normal one, so a held part slides while
its normal force sits exactly at the commanded value. ⚠ **They are not
separable and not additive**: each alone leaves ~20 mm of slip, and a high
`impratio` on the *pyramidal* cone measured worse than changing nothing.

The full `impratio` sweep is in [CONTRACT.md §3b](CONTRACT.md); the short form
is that 1→2 goes the wrong way, 4 is the knee, and 10→300 is a plateau 0.09 mm
wide — which is what makes 10 a converged budget rather than a fitted value.

The fourth row is a finding about the *shipped recipe*, not about the engine:
[docs/guide/friction-grasp.md](../../../docs/guide/friction-grasp.md) prescribes
`newtonContactKe 8000`, and at this scale it **breaks a grasp that was
working** — with the interference recomputed for the new stiffness, so the grip
force is held at 3 N and the ejection is attributable to contact stiffness
alone. (The first run of this sweep did *not* recompute it and raised the force
44 % at the same time; the variant runner now passes `LADDER0_NEWTON_KE` to the
driver so the sweep changes one thing. The uncorrected run reported the same
ejection for the wrong reason.)

### The fair-defaults question, and how it was settled

This row used to be **red while the MuJoCo arm's was green**, and that comparison
was not honest: the two arms were running rung 8 under different solver
configurations and the contract never said which was allowed. That is settled in
[CONTRACT.md §3b](CONTRACT.md) — **rung 8 declares a friction MODEL, each arm may
spell it in whatever its engine requires, and every arm must also publish what
the same scene does at that engine's own defaults.** The rule was written before
this arm was changed, and it draws a boundary: only settings that change *how
accurately* the declared Coulomb model is enforced, each shown by a published
sweep to be a converged budget, each named in `meta` with the engine default it
departs from.

What the R5 defaults column then says is worth more than the row:

| arm | engine | declared | at pure defaults |
|---|---|---|---|
| `webots` | ODE | nothing | **passes**, `carry_rel` 22 µm |
| `mujoco` | MuJoCo 3.8.1 CPU | `noslip_iterations 5` | fails — 44 mm of slip, payload on the table |
| `omnisim` | Newton → `SolverMuJoCo` CPU | `newtonCone "elliptic"` + `newtonImpratio 10` | fails — 56 mm of creep, payload dropped |

**Two of the three engines cannot hold this grasp out of the box, and they are
the two that solve contacts as soft constraints.** The one that passes with
nothing declared uses a hard-constraint LCP. That is a statement about contact
formulations, not about which project is better, and it is the sentence to quote
from this rung.

**An engine gap the rule exposed.** The MuJoCo arm predicted rung 8 might be
unreachable on OmniSim if Newton could not spell `noslip_iterations` — and it
could not. It does now (`WorldInfo.newtonNoslipIterations`, hatch
`OMNISIM_NEWTON_NOSLIP`, default 0 so every existing world is byte-identical),
which is what let the prediction be *tested*: on this engine's contact
parameterisation the pass does not rescue the grasp at any count from 1 to 20,
while it fixes the same scene outright in bare MuJoCo. Why the two disagree is
open, and the obvious suspect — the two arms derive `solref` differently — has
not been tested.

### Rung 8's assertions are surgical — proved where the grasp holds

While the honest row was red it failed five checks at once, which on its own
cannot distinguish "the grasp is broken" from "these five checks always fail
together" — and the self-test could only report the must-green companions as
`MASKED`. Now that the row is green the battery runs in the ordinary
`--self-test` and every fault is surgical:

* **`no_grip`** (the causal control) → `lift_height` red; the payload stays at
  **0.729892 m** on the table with a peak speed of **0.0016 m/s** — it is not so
  much as nudged — while the gripper carries away.
* **`no_traverse`** → `place_x` red **alone**; `lift_height` 0.876889,
  `carry_rel` 0.002559 and `hold_clearance` 0.144945 all green.
* **`drop_mid_carry`** → `part_speed_max` **1.4896 m/s** red alone;
  `part_rest_z` green.

The first is the differential rung 4 does not have. It is what licenses rung 8
to claim the *fingers* did the lifting rather than something else in the scene,
and until this row went green it could not be demonstrated at all.

### `mujoco_warp`: the documented raycast freeze does NOT reproduce — but the brakes do something worse

The brief for this work recorded that under `newtonSolver "mujoco_warp"` the
raycast-backed sensors freeze at the authored t = 0 scene (a `DistanceSensor`
reading 0.75 forever). **On this build that does not happen.** Both probes ran
genuinely on the GPU path — sidecar `solver: MuJoCo (mujoco_warp,
WorldInfo.newtonSolver)`, `device: cuda:0`.

* **rung 5 under `mujoco_warp`: ALL GREEN**, whole-run residual 9.68575e-08 m —
  *bit-identical to the CPU run*. But rung 5's scene is entirely static, so a
  ray cast against a stale copy of it would still come back right. **This probe
  cannot refute a stale-scene freeze and is not offered as doing so.**
* **rung 6 under `mujoco_warp` is the decisive one**, because there the sensor
  rides a body whose pose the solver produces. The sensor **tracks correctly**:
  1253 distinct readings over 2000 samples, residual ~2e-05 m through the whole
  approach and after settling.

What goes wrong there is not the sensor:

> **Commanding the wheels to zero launches the rover 1.09 m BACKWARDS at up to
> 2.98 m/s.** The stop triggers correctly at t = 2.328 s with the sensor reading
> 0.4990 m; the rover then recoils from x = 2.181 to x = 1.090 and stops there,
> ending **1.590 m** from the wall against an expected 0.485 m.
> The CPU run of the identical scene stops in 15 mm.

The 0.605 m `sensor_agrees` residual is confined to that 0.3 s recoil, and a
constant sample lag does not explain it (the best-fitting 37-step lag still
leaves 0.41 m). Outside the transient the residual is ~2e-05 m. **The mechanism
of the recoil is not determined.** It is an impulsive artefact at a velocity
discontinuity — the same *family* as rung 4's launch overrun, but at a stop
rather than a start and on the GPU path rather than the CPU one. Calling them
the same bug is not supported by anything measured here.

### The fault battery

14 live faults, **all 14 red as required** (144/144 self-test cases). Rung 8's
`no_grip` and `no_traverse` report three companions as `MASKED` — already red on
the honest scene, so those faults cannot be shown surgical on them *here*; the
`graspfaults` variants above show that they are, where the grasp works.

## Measured — OmniSim, 2026-08-12 (rungs 0–4 campaign, EARLIER BINARY)

Engine `omnisim-bin.exe` (08-12 13:16), Newton/MuJoCo CPU `mj_step`, verdict
sidecar `finalised: true, degraded: false`, `mujoco 3.8.1 / newton 1.2.0 /
warp 1.13.0`, model device `cpu` (the runtime then; pinned today: newton 1.5.0 / warp 1.16.0 / mujoco 3.11.0).

| rung | verdict | headline |
|---|---|---|
| 0 | PASS | 250/250 steps, clock exact |
| 1 | PASS | rest_z 0.599892 m vs 0.600000 (penetration 0.108 mm) |
| 2 | PASS | fall_interval 0.165951 s vs 0.165955 — **4 µs** |
| 3 | PASS | ω 2.0000001 vs 2, and commanding 0 gives exactly 0 |
| 4 | **FAIL** | `roll_overrun` 10.77 (tol 0.05), `ride_height` 0.0379 m (tol 0.02) |

Rung 2's `fall_time_abs` lands 2.0 ms early against a 0.451524 s expectation —
which is `dt/2` exactly, the predicted semi-implicit-Euler phase bias, and it
cancels out of `fall_interval` as the derivation says it must.

**Rung 4 is a real OmniSim defect and it predates the perf commits** (it
reproduces identically on the 08-11 build). The steady state is excellent —
0.400 m/s, `v/(ωr)` = 1.00, lateral drift 7e-15 m — the launch is not.

Controls run against it, so it is not an artefact of this scene:

| control | peak v (m/s) | `v/(ωr)` | axle deviation (m) |
|---|---|---|---|
| baseline | 4.165 | 10.41 | 0.0379 |
| wheels moved 10 mm outboard (no wheel/chassis tangency) | 4.165 | 10.41 | 0.0380 |
| rover spawned 5 mm high (no zero-penetration initial contact) | 3.949 | 10.02 | 0.0331 |

Neither a self-collision between the chassis `boundingObject` and its own
wheels, nor an exact zero-penetration spawn, accounts for it.

**The mechanism is not determined.** The obvious hypothesis — a missing wheel-
radius factor, since the 4.10 m/s peak sits suspiciously close to the 4.0 rad/s
command — was tested and is **wrong**: sweeping the wheel radius gives peak
speeds of 2.78 / 4.17 / 3.72 m/s at R = 0.20 / 0.10 / 0.05 m, so the peak is
roughly radius-*independent* while `v/(ωr)` swings from 3.5 to 22.8. That is
the signature of an impulsive launch artefact, not a mis-scaled rolling
constraint, and the "10×" figure is specific to R = 0.1 m rather than a
universal factor.

### A second finding: an undeclared collision plane at z = 0

The rung-1 `no_floor` fault removes the floor's `boundingObject` and nothing
else. The box does not fall for ever: it settles at **z = 0.099892 m**, i.e.
resting on a surface at z ≈ 0 that appears nowhere in the world file, with the
same 0.108 mm penetration it shows on a real floor. The world declares
`newtonStatics TRUE`.

This matters twice over. It is why this ladder's floor is at z = 0.5 (at z = 0
the phantom and the authored floor coincide and the bug is unobservable). And
it means the AgentBench C2 "fall through the floor" defect no longer presents
as a body reaching z = −69 km — it presents as a body resting quietly on a
surface that does not exist, which `--fail-on-runaway` cannot see either,
because nothing runs away.
