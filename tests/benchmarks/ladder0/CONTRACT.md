# ladder0 — the shared contract

**Read this before writing an arm.** It is the whole interface between the
simulator-specific code (`ladder0/<sim>/`) and the shared, simulator-neutral
code (`ladder0/*.py`).

Owner of the shared files: the OmniSim lane. `webots/` and `mujoco/` are owned
by their own lanes and nothing outside them may be edited by those lanes.

---

## 1. The rule that makes the ladder worth running

> **Every expected value is derived from first principles. None was ever read
> out of a running simulator, and none may be.**

A golden captured from today's behaviour would have certified every engine
defect this repo found in the week before the ladder was written: a phantom
`z=0` collision plane that caught bodies which should have fallen,
`setVelocity` silently ignored after world finalize, and wheels that did not
rotate while the chassis slid the right distance.

Corollary for arm authors: **an arm never computes a verdict.** It drives the
scene, records physical quantities, and hands them over. `analysis.py` reduces
them and `rungs.py` judges them. If you find yourself writing a threshold in
`<sim>/`, it belongs in `rungs.py` instead — and it needs a physical
derivation, not a value that happens to pass.

---

## 2. Files

| file | owner | what it is |
|---|---|---|
| `rungs.py` | shared | scene constants, analytic ground truth, tolerances + their derivations, `check_rung()` |
| `analysis.py` | shared | sample document → measurement dict. The only reducer. |
| `run_ladder.py` | shared | entry point, arm discovery, table, exit code |
| `selftest.py` | shared | the red proofs |
| `<sim>/arm.py` | per-sim | launch + record |
| `<sim>/worldgen.py` | per-sim | emits that sim's scene files **from `rungs.py`** |
| `<sim>/worlds/` | per-sim | the generated, committed scene files |
| `<sim>/rung18.py` | per-sim | the recorded-reality replay: a **bridge to lane1r**, never a second copy of its constants |
| `omnisim/variants.py` | per-sim | non-default engine settings, one field at a time. **Never a ladder row** — §3E |

Import the contract from an arm like this (works whether the arm is imported
as a package member or executed directly):

```python
import os, sys
LADDER0 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if LADDER0 not in sys.path:
    sys.path.insert(0, LADDER0)
import rungs
```

**Do not re-declare a scene number.** Read `rungs.FLOOR_TOP`, `rungs.WHEEL_R`,
`rungs.RUNG4_OMEGA_CMD` and so on. A scene that drifts from the contract is
measured against the wrong expectation and the row is silently meaningless.
That includes an *override*: an arm must not offer a knob that changes a scene
number, however well documented — the moment a row can be produced from a
scene the contract did not describe, the table is no longer comparable and
nothing in it says which scene it came from.

### Loading your own modules: BY PATH, under an arm-qualified name

`run_ladder.py` loads all three arms into **one** process, and `sys.modules`
is global to it. All three arms ship a module called `worldgen`, so:

```python
import worldgen                     # NO. Resolves to whichever arm loaded first.
```

The second arm loaded gets the **first arm's** generator, and would then
generate *and measure* another simulator's scenes while reporting them as its
own — a row that is wrong in a way no assertion here can see, because every
number in it is internally consistent. Do this instead:

```python
sp = importlib.util.spec_from_file_location(
    "ladder0_<sim>_worldgen", os.path.join(HERE, "worldgen.py"))
worldgen = importlib.util.module_from_spec(sp)
sys.modules["ladder0_<sim>_worldgen"] = worldgen
sp.loader.exec_module(worldgen)
```

`run_ladder.load_arm()` enforces this: an arm holding a module that lives
under a *different* arm's directory is refused with `ArmImportCollision` and
the whole run stops (exit 2) rather than reporting a measurement of unknown
provenance. Registering an arm-local module under a generic name is reported
as an IMPORT HAZARD in the run output — nothing is wrong yet, but it is the
precondition for the collision.

The same rule protects the arm from the outside: `ladder0/mujoco/` must never
be reachable from `sys.path`, or `import mujoco` finds *it* instead of the
simulator (measured — the arm died on `module 'mujoco' has no attribute
'__version__'`), which is why that directory has no `__init__.py`.

---

## 3. The twelve rungs

| rung | scene | asserted |
|---|---|---|
| 0 | empty world | `steps_completed`, `exit_code`, `finite_clock` |
| 1 | 0.2 m cube resting on a floor whose top is at z = 0.5 | `rest_z`, `contact_penetration`, `z_drift` |
| 2 | same cube released at z = 1.6 (1.0 m of free fall) | `spawn_z`, `fall_interval`, `fall_time_abs`, `rest_z` |
| 3 | one hinge about a **vertical** axis, motor in velocity mode, no floor | `omega_driven`, `omega_zero`, `angle_travelled` |
| 4 | four driven wheels, straight line on the rung-1 floor | `distance`, `wheel_omega`, `rolling_consistency`, `roll_overrun`, `ride_height`, `lateral_drift` |
| 5 | a distance sensor on a kinematically swept carrier, facing a wall | `range_static`, `range_final`, `range_tracks`, `sweep_span` |
| 6 | the rung-4 rover + a forward sensor: drive, stop below a threshold | `stop_gap`, `min_gap`, `trigger_reading`, `sensor_agrees`, `stop_creep`, `wheel_stop` |
| 7 | five rung-4 rovers in parallel lanes, five different commands | `distance_worst`, `wheel_omega_worst`, `min_separation`, `lateral_worst`, `roll_overrun_worst`, `ride_worst` |
| 8 | a Cartesian gantry gripper lifts a 0.06 m payload off a table and carries it 0.45 m | `part_rest_z`, `carry_rel`, `lift_height`, `place_x`, `hold_clearance`, `part_speed_max` |
| 9 | a 5×5 pile + a 26th cube dropped on the pile's outer corner, **three runs from three processes** | `repeat_delta`, `repeat_length`, `sensitivity_shortfall`, `fall_interval`, `distinct_processes` |
| 11 | the rung-4 rover at **N = 1, 4, 8, 16**, one command and one controller process each | `distance_worst`, `wheel_omega_worst`, `roll_overrun_worst`, `ride_worst`, `lateral_worst`, `separation_shortfall`, `robots_seen` |
| 18 | replaying **50 recorded real cube tosses** onto a table | `real_pos_err`, `real_rot_err`, `tunnel_depth`, `replay_ic_fidelity`, `tosses_unscored`, **`embed_gap`** |

`rungs.RUNGS` is the **every-commit** set (0–9, 11); `rungs.RUNGS_ON_DEMAND` is
rung 18, which launches one engine per toss. `rungs.MULTI_RUN` names the rungs
whose cell is more than one run (§4a). There is no rung 10 and no rungs 12–17:
they are designed in [`PLAN_9_20.md`](PLAN_9_20.md) and deliberately not built
(§3c).

Three design points that are not arbitrary:

* **The floor top is at z = 0.5, not z = 0.** An implicit ground plane at z = 0
  is invisible to any scene whose floor is already there. Lifting the floor
  makes the phantom plane separable from the authored one.
* **Rung 3's axis is vertical and the scene has no floor.** Gravity then exerts
  no torque about the joint and nothing can touch anything, so an ideal
  velocity servo has *exactly* zero steady-state error and the tolerance does
  not have to absorb a load term.
* **Rung 4 asserts distance AND wheel rotation.** A chassis that slid the right
  distance on wheels that never turned passed every check this repo had. That
  is why `rolling_consistency` (`v_body / (omega_wheel · r) == 1`) exists.

Rung 4 uses **four driven wheels rather than two plus a caster**: a passive
caster needs a low-friction contact beside a high-friction one, Newton exposes
exactly one global friction value (`newtonGroundMu`), and a caster sliding at
µ = 1 would inject an unbounded engine-specific drag term into an analytic
ground truth.

### 3a. Rungs 5–8 — design points, and what each one does NOT prove

Two rules carried forward from rungs 0–4, and they are not optional:

> **Every rung carries at least one invariant that holds over the WHOLE run.**
> Rung 4's original two assertions were measured over 2–6 s and passed a rover
> that launched at 10.25× rolling speed and banked 1.37 m of free distance
> before settling. A steady-state window can only see steady state.

> **For every rung, ask what would still pass it while being broken**, and
> either close that hole or write it down as the rung's limit. Rung 4's
> anti-slide fault cut the wheel motors entirely and still passed, because an
> undriven wheel dragged over µ = 1 ground rolls at exactly *v/r*.

**Rung 5 — the sensor, isolated.** The carrier has no physics and its pose is
*written* by the driver on the schedule `rungs.rung5_x_cmd(t)`, so the sensor's
position at every instant is a scene fact rather than a simulation result. A
sensor rung whose sensor pose comes out of a wheel model cannot separate "the
ray is wrong" from "the rover did not get where it was told". It **dwells**
0.25 s at the start and parks 0.5 s at the end because the three engines do not
agree on whether a pose written during step *k* is visible to a sensor read at
step *k* or *k+1*; the static readings are therefore taken where the carrier is
provably stationary and are insensitive to that ordering.

*The whole-run invariant is `range_tracks`* — the reading differenced against
the pose recorded on the same step, maximised over every sample. It is the only
check that can see a sensor which reads correctly at both ends and stops
updating in between, which is exactly the OmniSim `mujoco_warp` behaviour this
rung was built to measure.

*What rung 5 does NOT prove:* it cannot distinguish a genuine geometric ray from
a correct bookkeeping computation of the same quantity. Closing that needs a
scene whose correct reading is **not an affine function of the sensor's pose** —
an occluder crossing the ray, or a target the ray can miss — and the "no hit"
sentinel differs across engines (a lookup-table maximum here, `-1` in MuJoCo),
so normalising it would put a decision in the arm. Not claimed, not closed.

**Rung 6 — sensing and acting, together.** The rung-4 rover, unchanged, plus a
forward-facing sensor. **The final gap is measured from the POSE, never from
the sensor**: a sensor that is frozen, offset or fabricated would otherwise
report the gap it was supposed to produce and the rung would be grading the
sensor with the sensor. The stopping budget `RUNG6_STOP_BOUND` is derived in
`rungs.py` from latency (3 steps of cruise) plus friction-limited braking
`v²/2µg` carried at 3×; it is 5.9 % of the threshold and 17× smaller than the
error a rover that ignores the sensor makes.

*The whole-run invariant is `min_gap`* — the smallest gap over every sample. A
rover that ran into the wall and rebounded to a plausible resting place passes
`stop_gap` and fails only here. This is rung 6's `slide`.

*What rung 6 does NOT prove:* the same bookkeeping hole as rung 5, and it does
not establish that the rover *could* have stopped from a higher speed or that
its braking is friction-limited rather than servo-limited.

**Rung 7 — five robots, five commands.** Each rover is commanded a **different**
wheel rate, so each has its own analytic target and a command that leaked from
one robot to another reads as a wrong distance rather than as a plausible fleet
average. Non-interference is asserted by judging every robot's distance against
the value it would travel **alone** — a robot perturbed by a neighbour misses
its own solo expectation — and separation is asserted **geometrically from the
poses, not as a contact count**. This tree has shipped a contact read that
returned an empty set for a scene containing 1008 contacts; a geometric
separation cannot do that.

**No radio.** Coordination is deliberately not part of this rung. MuJoCo has no
Emitter/Receiver, and a rung one arm structurally cannot express would produce a
`NOT_EXPRESSIBLE` verdict that says nothing about the physics. Five four-wheel
rovers is ≈ 160 constraint rows, well under the 256 `njmax` default, so no arm
needs to raise it and none may pin `mujoco_warp`.

*The whole-run invariants are `min_separation`, `lateral_worst`,
`roll_overrun_worst` and `ride_worst`* — all four are maxima/minima over every
sample, not window statistics.

*What rung 7 does NOT prove:* that inter-robot contacts are *detected*. It
proves they do not happen. It also inherits rung 4's limit — the wheels are
shown to be kinematically consistent with the motion, not to have propelled it.

**Rung 8 — a grasp, proved geometrically.** A **Cartesian gantry, not an
articulated arm**: the claim is about the grasp, and an articulated arm would
add inverse kinematics and its own pose error to a measurement that is about
neither. Two prismatic stages plus two prismatic fingers put the gripper exactly
where the schedule says, so any deviation of the *payload* is the payload's.

**The wrist origin is authored at the payload's centre.** That is what makes
`carry_rel` an analytic zero instead of a self-reference: the two coincide at
t = 0 by construction and must still coincide at the end of the carry. A grasp
check that takes its reference *from the run* cannot tell a payload gripped
correctly from one gripped in the wrong place and then held there.

**Friction is raised to `RUNG8_MU` = 3.0 for this scene only, on purpose.** At
the global µ = 1.0 the required pinch force is marginal and the rung would be
grading the operator's tuning. At µ = 3 the Coulomb bound needs 0.33 N per pad
and the commanded `RUNG8_GRIP_N` is 9× that, so the measurement is *"does a
grasp with a large Coulomb margin hold"* and **not** *"what is the smallest
friction this engine can grip at"*. The latter is a sweep and is not claimed.

The contract owns the **grip force in newtons**; each arm owns the actuator that
produces it, because the mechanism is genuinely engine-specific (`setForce` does
not put a Newton joint in force mode — it stays a PD servo anchored at the last
`setPosition`). `rungs.rung8_bite_m(kp, ke)` writes the series-stiffness algebra
once for arms that must realise the force as a position interference; an arm
with a true force mode should use it and ignore the helper. **Record the
mechanism you used in `meta`.**

### 3b. Rung 8 declares a friction MODEL — the fair-defaults rule

*Decided 2026-08-12. It replaces an OPEN item that said the two arms were
running rung 8 under different solver configurations and the contract never
said which was allowed. It was written **before** the OmniSim arm was changed,
and the measurements that follow it were taken afterwards; the reverse order
is choosing the rule to fit the result.*

> **Rung 8 declares a friction MODEL, not a solver configuration. An arm may
> spell that model in whatever its engine requires, and must publish what the
> same scene does at that engine's own defaults.**

This is the same split the paragraph above already makes for the grip force —
*the contract owns the force in newtons, each arm owns the actuator that
produces it* — applied to the other half of a Coulomb contact. µ, the masses,
the geometry, the schedule and the normal force are physics and belong here;
whether an engine reaches that physics through a cone shape, a constraint
impedance, an iteration budget or a post-solve pass is machinery, and this file
has no business naming it.

The two rejected alternatives, and why:

* **"each arm declares whatever it needs"** has no stopping rule. A weld, a
  raised µ, a heavier payload and a kinematic part are all things an engine
  "needed", and the rung quietly becomes a tuning contest won by whoever tuned
  hardest. The rule below is the same option with a boundary drawn around it.
* **"pure defaults everywhere"** is a genuinely good question, but it is a
  *different* question from the one every assertion on this rung asks, and
  retrofitting it would leave the assertions measuring something they were not
  written for. It also has a nearly uniform answer, which is now measured on
  all three arms and recorded in the table below: the two **MuJoCo-family**
  arms both fail at their own defaults, in the same way (drift in a soft
  tangential constraint) — and the **ODE** arm, a hard-constraint LCP, passes
  with nothing declared. That row describes each engine's contact
  *formulation*; it cannot answer "can this engine hold a grasp".

So the ladder row is the declared-model run, and the defaults run is published
next to it. Neither is allowed to stand in for the other.

**R1 — what the contract owns, and no arm may touch.** µ = `RUNG8_MU` on every
contact of the scene, the payload's mass and geometry, every other body's mass
and geometry, `DT`, gravity, the commanded schedule, and the per-pad normal
force `RUNG8_GRIP_N`.

**R2 — what an arm may declare.** Only settings whose effect is *how accurately
the solver enforces the Coulomb model R1 declares*: the friction-cone shape,
the frictional constraint's impedance relative to the normal one, solver
iteration or refinement budgets, and post-solve friction passes. Explicitly
**not** admissible, whatever it would do for the row: changing anything in R1;
adding a mechanism the contract does not describe (a weld, a constraint, an
adhesion term, a `condim` that adds torsional or rolling friction the contract
never declared); making the payload kinematic; or raising the grip force.

The line R2 draws is *approximation error vs. model change*. A pyramidal
friction cone is not the Coulomb cone — it is a polygon inscribed in it, and
declaring the ellipse is asking for the model the contract already wrote down,
not for a better one. An extra friction iteration removes solver residual, and
residual is error. A weld is a different physics and is out, no matter how
small a change it looks like in the file.

**R3 — a declared value must be a BUDGET, not a fit.** Sweep it, report the
sweep, and show the measurement is insensitive to the exact value over a stated
range. A setting that works at one value and not at its neighbours is fitted to
this scene, and it is not admissible however green it is. (The MuJoCo arm's
`noslip_iterations 5` is the pattern: identical in every digit from 3 upward.)

**R4 — it is declared in the open or it did not happen.** Every departure from
an engine default goes in that arm's `meta` as `rung8_solver_declarations`,
each entry carrying the field, the value, **the engine's default for it**, and
one line of why; and it gets a row in the table below. A cell whose meta does
not list its declarations is not a comparable cell.

**R5 — the defaults datum is mandatory.** Each arm must also run rung 8 with
every R2 declaration removed, and publish it. Not a ladder row (the ladder is
one scene per rung; see §2), but a required, reproducible companion
measurement, and the only thing anyone may quote for "out of the box".

#### What each arm declares, and what it does with nothing declared

| arm | engine | declared under R2 | R5 datum — the same scene with nothing declared |
|---|---|---|---|
| `webots` | upstream Webots R2025a, ODE | **nothing** | **passes** — `carry_rel` 22 µm, all six green |
| `mujoco` | MuJoCo 3.8.1, CPU | `noslip_iterations 5` (default 0) | fails: 44 mm of slip down the pads, payload back on the table, `carry_rel` 0.440 m |
| `omnisim` | OmniSim Newton → `SolverMuJoCo`, CPU `mj_step` | `newtonCone "elliptic"` (default `""` = pyramidal) + `newtonImpratio 10` (default 0 → MuJoCo's stock 1) | fails: creeps 56 mm through the pads during the 1.5 s lift, drops the payload at the top, `carry_rel` 0.4747 m |

Read the last column before quoting this rung anywhere: **two of the three
engines cannot hold this grasp at their own defaults**, and they are the two
that solve contacts as soft constraints; the one that passes with nothing
declared solves them as a hard-constraint LCP. That is the honest headline, and
it is a statement about contact formulations rather than about which project is
better.

**The `omnisim` sweeps, in full** (machine `9722d23d12a3`, RTX 3060 laptop, CPU
`mj_step`, newton 1.2.0 / warp 1.13.0 / mujoco 3.8.1 — the runtime when the sweep ran; the pinned runtime is newton 1.5.0 / warp 1.16.0 / mujoco 3.11.0; every row is the honest
rung-8 scene with the named `WorldInfo` fields changed and nothing else, and
every row is reproducible with `python omnisim/variants.py <family>`):

| configuration | `carry_rel` (m) | verdict |
|---|---|---|
| engine defaults (R5 datum) | 0.474671 | red — payload dropped, `part_speed_max` 1.109 m/s |
| `newtonCone "pyramidal"` (explicit) | 0.474671 | red — identical, confirming `""` *is* the pyramid |
| `newtonCone "elliptic"` alone | 0.021294 | red — carried the full 0.45 m, slipped 21 mm |
| pyramidal + `newtonImpratio 10` | 0.017804 | red — carried, slipped 18 mm |
| pyramidal + `newtonImpratio 100` | 0.359707 | red — **worse than default**, payload dropped |
| elliptic + `newtonImpratio` 1 / 2 | 0.021294 / 0.042371 | red — and 2 is worse than 1 |
| elliptic + `newtonImpratio` 4 | 0.004448 | green — the knee |
| **elliptic + `newtonImpratio` 10 / 30 / 100 / 300** | **0.002559 / 0.002615 / 0.002640 / 0.002648** | **green — 0.09 mm of spread over a 30× range** |
| declared config + `newtonNoslipIterations 5` | 0.002652 | green — 0.09 mm from not declaring it |
| default cone + `newtonNoslipIterations` 0/1/3/5/8/20 | 0.4747 / 0.4855 / 0.4802 / 0.4796 / 0.4790 / 0.4779 | red at every count — the payload is dropped in all six |

Three things that table settles, and one it does not:

1. **R3 is satisfied and it took the whole sweep to show it.** The plateau is
   10→300; 4 is the knee; and 1→2 goes the *wrong way*, which is exactly why a
   single working value is not evidence. `10` is the first value inside the
   plateau, so it is declared with 30× of headroom above it.
2. **The cone and the impedance are not interchangeable, and the pair is not
   additive.** Either alone leaves ~20 mm of slip. Worse, a high `impratio` on
   the *pyramidal* cone is actively harmful (0.36 m, payload dropped) — a
   pyramid is a fixed set of friction directions, and stiffening a contact's
   tangential response along them is not the same operation as stiffening it
   against a circle. Anyone quoting "raise impratio" without the cone is
   quoting a setting that measured worse than doing nothing.
3. **The noslip pass is not the mechanism here, and that is a refuted
   prediction rather than an untested one.** The MuJoCo arm predicted rung 8
   might be unreachable on OmniSim if Newton could not spell
   `noslip_iterations`. Newton could not — so it was plumbed
   (`WorldInfo.newtonNoslipIterations`, `OMNISIM_NEWTON_NOSLIP` as the
   value-parsed hatch), verified to reach `mjOption` and to be reported in the
   engine's own backend sidecar, and then **measured not to fix this**. A rule
   that admits a class of declaration and an engine that cannot express it is
   an engine gap either way, and closing it is what let the prediction be
   tested at all.
The engine change carries the exact-revert hatch this tree requires, and it is
verified rather than asserted: the field defaults to `0`, which is *also*
MuJoCo's own stock value, and the new binary reproduces the undeclared rung-8
scene **bit-for-bit** against the pre-change one (`carry_rel`
0.47467107170711154 on both, and all five other measurements to the last digit).
On a world that *does* declare the field, `OMNISIM_NEWTON_NOSLIP=0` reproduces
the same numbers and the finalise line drops its `+noslip=` marker. On
`newtonSolver "mujoco_warp"` the request is declined — a WARNING in the engine
log, `noslip_UNSUPPORTED_ON_WARP` in the backend sidecar, and the run continues
— because `mujoco_warp` has no such field and its `put_model` *raises* on a
non-zero one, so passing it down would abort the solver build instead.

4. **What it does not settle: why the two MuJoCo-family arms disagree about the
   remedy.** The `mujoco` arm measured elliptic+impratio making its grasp
   *worse* and noslip fixing it; this arm measured the reverse. Both are on
   MuJoCo 3.8.1. The arms do not run the same contact parameters — the
   `mujoco` arm uses MuJoCo's stock acceleration-referenced `solref (0.02, 1)`,
   while OmniSim's Newton path derives each geom's `solref` from newton's
   `ke`/`kd` — so "the same solver" is doing two different things at the
   contact level. Naming which of those is responsible needs the two contact
   parameterisations compared directly, which nobody has done. Until then, do
   not generalise either arm's remedy to the other's engine.

*The whole-run invariants are `carry_rel` and `part_speed_max`*, and
`hold_clearance` spans the whole carry. `part_speed_max` exists because this
repo has a measured case of a pinch that ejected its part at 3.5 m/s and left it
sitting on the gripper's own wrist plate — an outcome whose *final pose* can look
entirely plausible.

*What rung 8 does NOT prove, and how the hole is closed:* the pose assertions
alone cannot distinguish a friction grasp from a weld. The **`no_grip` fault is
the causal control** — the identical scene with the fingers never closed must
leave the payload on the table. It is the differential rung 4 lacks, and it is
why rung 8 may claim the fingers did the lifting. Rung 8 does *not* measure grip
force, so it says nothing quantitative about the engine's friction model.

### 3c. Rungs 9, 11 and 18 — the three tier-C/E rungs that were built

[`PLAN_9_20.md`](PLAN_9_20.md) designs eleven rungs above 8. **Three are built,
and the other eight are deliberately not.** The reason is that plan's own
strategic finding (§0.1) and it is worth restating here, because it is the rule
for anything added above rung 8:

> On correctness assertions alone this ladder **cannot differentiate us from
> upstream Webots anywhere in tiers C or D, and it can never beat MuJoCo on
> fidelity** — MuJoCo *is* our solver. A rung whose only possible outcome is
> "three mature engines agree" costs days and confirms that nobody has a bug.

The three built are the ones where **our own answer is unknown or known-bad**.
Each is a row we can lose on our own name, which is the only kind worth the
days.

**⚠ Numbering.** `PLAN_9_20.md` numbers the recorded-reality rung **19** and a
closed-kinematic-loop rung **18**. The build brief's numbering is the one in
the tree: **rung 18 is the recorded-reality rung**. The plan's closed-loop
design is not built and is referred to by name rather than by number, so no
built rung and no designed rung share one.

**Rung 9 — determinism, with its own sensitivity control.** Same scene, two
fresh processes, bitwise identity — plus a third run seeded 1 µm away that must
*not* stay 1 µm away.

*The scene is a pile and that is the whole design.* Determinism is trivially
satisfied by a frozen world, and the known GPU refutation has a **named
mechanism** — mujoco_warp assigns contact pairs with `pairid =
wp.atomic_add(...)`, so the pair *order* is a race — which needs many
simultaneous pairs to express itself. The same campaign that found 0 of 24
bitwise pairs across six scenes also recorded a **single-contact scene
reproducing bit-identical 3/3** on the same path. A two-body rung 9 would
therefore return a *false green* on the GPU variant, in exactly the way rung
5's static scene "cannot refute a stale-scene freeze and is not offered as
doing so".

*The geometry of the drop is load-bearing and the obvious placement is wrong.*
The 26th cube is released over the **outer corner of the pile**, where a
quarter of its footprint is supported, its centre of mass sits exactly over the
edge of that support and the rest overhangs a 0.2 m drop to the floor.
Dropping it over the corner of the *centre* cube — which is what the design
originally said — puts it squarely on a 2×2 group, because the pile's 1 mm gaps
make its pitch 0.201 against a 0.2 m cube. **Measured**: it landed at
z = 0.799800, stayed, the pile never moved, and a 1 µm seed produced 1.0058 µm
of separation after 8 s. The control read as *"this engine damps
perturbations"* when the true answer was *"this scene has nothing to amplify"*.
With the drop at the pile's corner the same seed reaches **0.1455 m — an
amplification of 145,509×**.

*The sensitivity is measured over the WHOLE RUN, not at `t_end`*, and that is a
deliberate departure from the plan. Friction is dissipative: a pile that
settles is an attractor, and two runs seeded 1 µm apart can legitimately
reconverge to the same resting configuration. At `t_end` that reads as "the
scene is not chaotic"; over the whole run it reads as what it is.

*What rung 9 does NOT prove.* A frozen world is perfectly deterministic —
closed by the sensitivity control and by the analytic anchor, and `frozen` is
the fault that proves it. An engine that is deterministic and **wrong** passes
both determinism checks; gravity at 5 m/s² is exactly as reproducible as 9.81,
and `fall_interval` is the only check that sees it. An arm replaying a cached
result passes everything, which is why `distinct_processes` asserts the
replicas came from distinct `(pid, process start)` pairs recorded *by the
driver process itself*. **Declared, not closed:** a red
`sensitivity_shortfall` is ambiguous between "this engine damps perturbations"
and "this scene is not chaotic on this engine" — read it only alongside
`repeat_delta`, and the measured amplification factor is published beside the
row so the ambiguity can be sized rather than argued about. **Scoped:** rung 9
says nothing about cross-machine identity, which is untested, and a sibling
census already found 56 of 180 lane-1 cells differing between two machines.

**Rung 11 — fidelity at scale.** Every robot in a fleet meets the **same**
analytic target it would meet alone, with the **same** tolerance, at every
fleet size.

*No N-dependent slack.* The failure this rung exists to catch — a silently
truncated constraint vector — is documented at 9 % displacement error, nearly
twice `DISTANCE_TOL`. A tolerance that grew with N would be pre-authorised to
miss exactly the defect it was built for.

*Bit-identity of robot i across N is NOT asserted, and refusing to assert it is
a decision.* Adding robots changes the size and the ordering of the constraint
system; floating-point summation is not associative; robot *i* can differ in
the last ULP at N = 16 versus N = 1 in an engine with no defect at all, and a
red that means nothing trains everyone to ignore the row. The quantity is
**measured and reported** beside the row instead (`solo_deviation_max`) — and
on the CPU path it comes back **exactly 0.0**, which is a stronger statement
than the rung is willing to demand.

*One controller process per robot*, declared under R4 because it changes what
the row measures: the alternative — one supervisor commanding every wheel — is
not available, because a Webots-family supervisor may not write a sibling
robot's devices. At N = 16 the world starts 16 Python interpreters, and the
largest fleet with *live* controllers previously measured anywhere in this tree
is ten.

*The constraint budget is declared GENEROUS and the STARVE is a variant.*
`RUNG11_NJMAX` = 4096 against a 1024-row requirement at the largest fleet in
the family, because sizing a budget at a scene's own measured peak moved
results 8.81 m. ⚠ **`newtonNjmax` is a FLOOR, not a cap** — the runtime raises
a too-small request to the initial `nefc` at construction — so *setting it
lower cannot starve anything*, and a fault that tried would report a green it
could not make red. The starve works by **removing the declaration**, and the
briefed refinement of the floor mechanism did not survive measurement: `initial
nefc = 0` is reported whether the fleet spawns in contact or clear, so the
spawn clearance is irrelevant and the declaration is the whole story. Measured
consequences are in [README.md](README.md); the short form is that the overflow
reproduces at N = 16 on `mujoco_warp`, does not exist on CPU `mj_step`, and
**degrades the physics measurably while leaving every one of rung 11's
assertions green** — which is this rung's honest limit and is written down
rather than discovered later.

*What rung 11 does NOT prove.* It inherits rung 4's limit — the wheels are
shown kinematically consistent with the motion, never to have propelled it. An
engine that silently drops robots is closed by `robots_seen`; one that
simulates a single robot and copies it is closed by the **cycled commands**,
which land every clone on one target.

**Rung 18 — agreement with recorded reality.** The only rung whose ground truth
is not ours, and the only check in this ladder we can lose and cannot win.

*The dataset is not rebuilt and no physical constant of it is re-declared.*
`tests/benchmarks/omnibench/lane1r` owns the recording, its licence, the cube's
mass / geometry / inertia, the sampling rate, the quaternion convention and a
calibration it **re-derives on every run** rather than trusting the dataset's
own metadata. The ladder reads them through `rungs.rung18_dataset()`. **If this
ladder and lane1r ever disagree about the cube's inertia, lane1r is right.**

*The acceptance band is external and it is not a good score.* The bound is the
published MuJoCo row plus one published standard deviation (Acosta, Yang &
Posa, RA-L 2022). Nothing in this repo can move it; it was not chosen after
seeing our number; it is the row for the engine we **embed**; and the best
engine measured on this data manages 13.5 % of cube width. A rung a user could
read as "OmniSim is accurate" would be lying. This one says "OmniSim is where
the field is", and the field is not good at tossed cubes.

*`embed_gap` is the headline and the other two are floors.* Our solver **is**
MuJoCo, so a gap between our arm's error and the **bare MuJoCo arm's** error on
the same tosses, against the same recording, reduced by the same code, is our
translation layer and not the physics. It is computed across two cells by
`rungs.check_rung18_embed_gap`, called once by the runner — never inside an
arm — and **the reference arm does not get the check at all**, because
`|its error − its own error|` is zero by construction and a free green on the
one unwinnable row would be the worst possible way to report it.

#### What each arm declares on rung 18, and what it does with nothing declared

R1–R5 apply here exactly as they do on rung 8, generalised from "the friction
model" to "the physical model". Rung 18's scene is lane1r's, so its
declarations are lane1r's and each arm's job is to carry them faithfully rather
than to choose them.

| arm | engine | declared under R2 | R5 datum — the same 50 tosses with nothing declared |
|---|---|---|---|
| `omnisim` | Newton → `SolverMuJoCo`, CPU | `newtonCone "elliptic"` (default `""` = pyramidal) + `newtonImpratio 10` (default 0 → MuJoCo's stock 1), both from lane1r's world | — |
| `mujoco` | MuJoCo 3.8.1, CPU | the same two, translated to MJCF `cone` / `impratio` | **fails: 43.023 %** against a 35.9 % bound |
| `webots` | ODE | `UNIMPLEMENTED` | — |

> **Translate the whole `WorldInfo`, or you are replaying a different scene.**
> An arm that carried lane1r's geometry and masses but dropped its cone shape
> would score 18 points worse and the deficit would be charged to its
> translation layer.

The sweep behind that, from the bare-MuJoCo arm: defaults 43.023 %,
`cone=elliptic` alone 24.845 %, `impratio=10` alone 43.036 %, both 24.845 %.
**The cone is worth 18.18 points of agreement with recorded reality and the
impedance ratio is worth 0.013.** A pyramidal cone is a polygon inscribed in
the Coulomb cone; asking for the ellipse is asking for the model the contract
already declared, which is exactly what R2 admits.

⚠ **Order of decision, disclosed** because §3b is explicit that choosing a rule
after seeing a result is choosing it to fit: the defaults were measured first,
the gap was seen, it was investigated, and *then* the translate-the-whole-block
rule was written down. The rule is derivable without the measurement; the
measurement is why anyone looked. Both runs ship.

*What rung 18 does NOT prove.* An engine that reproduces inelastic impacts and
nothing else scores well here — lane1r records Acosta's finding that every
engine handles inelastic impacts and *all of them* fail on elastic ones, and
that sentence travels with the number wherever it is quoted. **A better score
is not evidence of a better simulator**: the Drake > Bullet > MuJoCo ordering
on rigid impacts *inverts* on cloth. And the row is the **authored** contact
parameters, not lane1r's identified ones — a fitted µ is a fit, which R3
forbids in a row; the identification is lane1r's own published companion.

---

## 4. The sample document

Every arm produces this, and nothing else. JSON, one file per cell.

```jsonc
{
  "rung": 2,
  "sim": "omnisim",
  "dt": 0.004,                 // seconds actually used
  "steps": 750,                // steps the driver COMPLETED
  "sim_time_end": 3.0,         // simulated clock at the end
  "t":       [0.0, 0.004, …],  // simulated seconds, one per sample

  // rungs 1 and 2 — world-frame z of the box's centre, metres
  "box_z":   [1.6, 1.5999, …],

  // rung 3 — joint angle in radians. MAY be wrapped; see below.
  "joint_q": [0.0, 0.008, …],

  // rung 4 — world-frame chassis position, metres
  "body_x": [...], "body_y": [...], "body_z": [...],
  // rungs 4 and 6 — per-wheel angle in radians, keys exactly rungs.WHEEL_TAGS
  "wheel_q": {"fl": [...], "fr": [...], "rl": [...], "rr": [...]},

  // rungs 5 and 6 — the range reading in METRES, one per sample.  The raw
  // geometric range, NOT the engine's lookup-table / sentinel value: declare
  // an identity lookup table (Webots-family) or convert the units, but never
  // substitute a computed distance for a sensed one.
  "range": [2.9, 2.9, ...],

  // rung 7 — five robots, keys exactly rungs.RUNG7_TAGS, in that order
  "robots": {"r0": {"x": [...], "y": [...], "z": [...],
                    "wheel_q": {"fl": [...], "fr": [...],
                                "rl": [...], "rr": [...]}},
             "r1": {...}, "r2": {...}, "r3": {...}, "r4": {...}},

  // rung 8 — the payload and the wrist the pads hang from, world frame, m
  "part_x":  [...], "part_y":  [...], "part_z":  [...],
  "wrist_x": [...], "wrist_y": [...], "wrist_z": [...],

  "wall": {"t_start": 1.7e9, "t_first_step": 1.7e9, "t_end": 1.7e9}
}
```

### 4a. Multi-run cells — rungs 9, 11 and 18

A rung owns **one generator**. A cell is **one or more runs** of scenes that
generator produced, from one `arm.run()` call, and the rung is green only when
every run in the cell meets its own analytic target. `rungs.MULTI_RUN` names
the rungs and their tags.

```jsonc
{
  "rung": 9,
  "runs": [
    {"tag": "a", "params": {"eps_m": 0.0},   "pid": 12345, "proc_start": 1.7e9,
     "t": [...], "steps": 2000, "sim_time_end": 8.0,
     "bodies": {"p00": {"x": [...], "y": [...], "z": [...]}, …,
                "drop": {"x": [...], "y": [...], "z": [...]}}},
    {"tag": "b", "params": {"eps_m": 0.0},   "pid": 12346, …},
    {"tag": "c", "params": {"eps_m": 1e-6},  "pid": 12347, …}
  ],
  "wall": {…}                                  // the CELL's, spanning every run
}

{ "rung": 11,
  "runs": [{"tag": "n4", "params": {"n": 4}, "pid": …, "proc_start": …,
            "t": [...], "steps": …,
            "robots": {"r00": {"x": [], "y": [], "z": [],
                               "wheel_q": {"fl": [], "fr": [],
                                           "rl": [], "rr": []}}, …}}] }

{ "rung": 18,
  "requested": [0, 1, …, 49],                  // MUST be a rungs.RUNG18_SUBSETS entry
  "runs": [{"tag": "toss0000", "params": {"index": 0}, "pid": …,
            "t": [...], "pos": [[x,y,z], …], "quat": [[w,x,y,z], …],
            "ic": {"want_vel": [3], "got_vel": [3],
                   "want_omega_world": [3], "got_omega_world": [3],
                   "grav_step": 0.00981},
            "reference": {"index": 0, "t": [...], "pos": [[…]], "quat": [[…]],
                          "cube_edge_m": 0.1048, "table_top": 0.5,
                          "scale_mode": "none", "source": "<dataset + licence>"}}] }
```

Three rules, none optional:

* **The tags and `params` are the CONTRACT's, not the arm's.** Rung 9's replica
  set, rung 11's N sweep and rung 18's toss subset are declared in `rungs.py`
  and read by the arm, exactly as `rungs.rung5_x_cmd(t)` is. An arm that chose
  its own N — or its own tosses — produces a row that is not comparable and
  nothing in the table says so. Rung 18 enforces it: a `requested` list that is
  not one of `rungs.RUNG18_SUBSETS` leaves `tosses_missing` unmeasured, which
  is judged **red for provenance**.
* **Replicas must come from DISTINCT PROCESSES**, with the `pid` and process
  start time recorded **by the driver process itself**. A determinism rung whose
  two replicas are one process — or one array copied — measures the arm. This
  is the one place the ladder checks the arm rather than the engine, and it is
  cheap. `rungs.check_rung(9, …)` asserts it.
* **Rung 18's `got_*` must be a genuine READBACK**, never an echo of the
  request. The reducer subtracts one step of gravity from the linear-velocity
  error before comparing, because the readback happens after one step.

### 4b. Decimation, and the float round-trip

* A rung may declare `SAMPLE_EVERY` in `rungs.py` (steps between samples,
  default 1); the arm samples on that stride and records the **true simulated
  `t`** of each sample, never a reconstructed grid. **The stride is the
  contract's**: `RUNG9_SAMPLE_EVERY` = 5, `RUNG11_SAMPLE_EVERY` = 2. Each is
  set by the fastest thing its reduction has to see, and each carries that
  derivation next to it.
* **Sample documents must round-trip float64 exactly.** Python's `json` writes
  `repr(float)` and does. An arm that formats through `"%.6f"` — or records
  float32 — destroys the very quantity rung 9 measures, and the result would
  look like an engine that is deterministic to six decimals. The same applies
  to a *scene* number: rung 9's 1 pm `seed_nudge` writes as `0` under `%.6f`,
  and the fault would silently not happen while the self-test reported a ladder
  defect that is not one.

Rules:

* **`t` is the SIMULATED clock**, in seconds, and every series is the same
  length as `t`.
* **Rungs 1 and 2 must include a sample at t = 0 taken BEFORE the first step**,
  read from the scene description rather than from a sensor. It is the only
  chance to observe the pose the scene file authored, and rung 2's `spawn_z`
  assertion is exactly that observation. Rungs 3–8 start at t = dt, because a
  sensor has no valid reading before the first step.
* **Rungs 5 and 8 drive a commanded schedule that the contract owns.** Read
  `rungs.rung5_x_cmd(t)`, `rungs.rung8_lift_z(t)` and
  `rungs.rung8_traverse_x(t)`; do not re-implement the ramps. Sample **before**
  writing the next command inside the loop, so a sample is never taken from a
  pose the driver has just changed and the engine has not yet stepped.
* **Rung 6's threshold is a COMMAND, not a verdict.** The driver stops when the
  range first reads below `rungs.RUNG6_STOP_GAP`; the reducer independently
  re-derives the crossing from the recorded `range` series. Record the series;
  do not report your own trigger as the measurement.
* **Joint angles may be wrapped or unbounded** — the reducer differences
  consecutive samples and folds each difference into (−π, π] before
  accumulating, so both conventions recover the same travelled angle. At ≤ 4
  rad/s and dt = 4 ms one step is ≤ 0.016 rad, far inside the fold.
* **Never substitute the commanded value for a measured one.** If a quantity
  could not be measured, leave the series out. `None` is judged RED; a
  fabricated number is judged green and is worse than no row at all.
* `wall.*` are POSIX epoch seconds (`time.time()`), used only for the wall-clock
  report.

---

## 5. The arm interface

`ladder0/<sim>/arm.py` must define exactly three names:

```python
NAME = "webots"          # matches the directory

def available() -> tuple[bool, str | None]:
    """(True, None) if this arm can run here; (False, why) if it cannot."""

def run(rung: int, out_dir: str, fault: str = "none", **kw) -> tuple[dict, dict]:
    """Run one cell.  MUST NOT RAISE — a broken simulator is a measurement.

    Returns (samples, meta).  `samples` is the document in §4.
    `meta` carries at least:
        exit_code : int | None
        error     : str | None    # human text; a non-None error reds the cell
        proc_t0   : float | None  # epoch seconds at process spawn
        proc_t1   : float | None  # epoch seconds at process exit
    plus anything else worth recording (binary sha, engine version, …).
    """
```

`run_ladder.py` discovers arms by looking for `ladder0/<name>/arm.py`. An arm
that does not exist is reported as *not run*, which is **not** a pass — an
unrun arm is an unknown. An arm whose `arm.py` **fails to import** is likewise
reported (as unavailable, with the import error) and does not stop the other
arms: `available()` and `run()` must both keep working on a module that could
not be loaded, so the runner can say *which* arm broke.

`**kw` will receive `timeout_s`, and rung 18's fault battery passes
`subset="fault"`; ignore what you do not use.

### 5a. Three ways a cell can be neither green nor red

They are different facts about different things, and collapsing any two of them
loses the one piece of information the row carries.

**`NOT_EXPRESSIBLE`, per CHECK.** A capability the SIMULATOR structurally does
not have. Declared in the arm's `meta`:

```python
meta["not_expressible"] = {
  "radio_follow_distance": {
     "missing": "a communication device model: MuJoCo has no Emitter/Receiver "
                "node.  A message between two robots would be a variable "
                "inside the driver process, so the SCENE cannot express it and "
                "the check would grade the arm rather than the simulator.",
     "citation": "<resolvable reference, quoted>",
     "status": "CITED",           # buildbench's verification_status vocabulary
  },
}
```

Runner semantics, all load-bearing:

1. an `N/E` check is **not green and not red**; it does not set the exit code;
2. a cell with ≥ 1 `N/E` prints `PARTIAL (n/m)`; all-`N/E` prints
   `NOT_EXPRESSIBLE`. Neither is ever drawn in the same colour as a failure;
3. **an arm that declares `N/E` for a check it also produced a number for stops
   the run** (`rungs.NotExpressible`, the `ArmImportCollision` severity). A
   refusal and a measurement of the same quantity cannot both be true;
4. a declaration missing `missing` or `citation` is judged **RED**. That is the
   existing "`None` is red, never skipped" rule extended: *we did not look*
   must never read as *nothing was wrong*;
5. **`N/E` is declared in the arm's SOURCE, never inferred from a failed run.**
   An arm that tried and failed reports RED. This distinction is the whole
   value of the verdict and the easiest one to lose, so nothing in
   `rungs.apply_not_expressible` can produce an `N/E` the arm did not write
   down;
6. `N/E` against our own name is recorded as prominently as anyone else's.

**`UNIMPLEMENTED`, per RUNG.** Nobody has written the code yet. Declared as
`UNIMPLEMENTED = {rung: "why"}` on the arm module; the runner reports it under
*arms not run*. It is an **unknown, and an unknown is not a pass** — but it is
not `N/E` either, and using `N/E` for it would blame an engine for our backlog.
The upstream-Webots arm declares it for rung 18: ODE against that recording has
not been measured by us or, as far as this ladder knows, by anyone.

**`UNVALIDATED`, per rung per arm.** See §6a: the rung's must-red faults have
not been shown to go red *here*. Also not a pass.

Make every result **self-describing**: record the engine version/build the row
came from in `meta`. Results produced by two different builds are
indistinguishable after the fact without it.

---

## 6. `fault=` and the self-test

`selftest.py` proves the ladder can go red, two ways:

* **assertion mutation** — pure, no simulator, covers every assertion of every
  rung, plus "unmeasured must be RED" and "the tolerance boundary is where the
  derivation says it is";
* **live fault injection** — a real run broken in a way that reproduces a
  defect this repo actually shipped.

The live faults are:

| rung | `fault` | must go RED | must stay GREEN |
|---|---|---|---|
| 0 | `short_run` | `steps_completed` | |
| 1 | `no_floor` | `rest_z` | |
| 2 | `half_gravity` | `fall_interval` | `spawn_z` |
| 3 | `ignore_zero` | `omega_zero` | `omega_driven` |
| 4 | `slide` | `rolling_consistency` | **`distance`** |
| 5 | `no_sweep` — the carrier is never moved | `range_final` | `range_static`, `range_tracks` |
| 5 | `wall_shifted` — the wall is authored `RUNG5_FAULT_SHIFT` further away | `range_static` | `sweep_span` |
| 6 | `no_stop` — the stop is never commanded | `stop_gap` | `trigger_reading` |
| 6 | `bounce` — runs into the wall, then is put back at the right resting place | **`min_gap`** | `stop_gap`, `wheel_stop`, `stop_creep` |
| 7 | `stalled_robot` — one robot's wheels are commanded zero | `distance_worst` | `min_separation`, `lateral_worst` |
| 7 | `lane_offset` — one robot is **spawned** half a lane out of place | `min_separation` | `distance_worst`, `wheel_omega_worst` |
| 8 | `no_grip` — the fingers never close | `lift_height` | `part_rest_z`, `part_speed_max` |
| 8 | `no_traverse` — the traverse is never commanded | `place_x` | `lift_height`, `carry_rel`, `hold_clearance` |
| 8 | `drop_mid_carry` — the fingers reopen at the top of the lift | `part_speed_max` | `part_rest_z` |
| 9 | `seed_nudge` — replica **b only** spawned `RUNG9_FAULT_NUDGE` (1 pm) off | `repeat_delta` | `sensitivity_shortfall`, `fall_interval` |
| 9 | `frozen` — the dropped cube loses its physics | **`fall_interval` AND `sensitivity_shortfall`** | **`repeat_delta`** |
| 9 | `short_b` — replica **b only** runs half the steps | `repeat_length` | `fall_interval` |
| 11 | `stalled_robot` at N = 16 | `distance_worst` | `separation_shortfall`, `lateral_worst` |
| 11 | `lane_offset` at N = 16 — a **spawn** offset, never a per-step write | `separation_shortfall` | `distance_worst`, `wheel_omega_worst` |
| 18 | `ic_drop_velocity` — the engine is handed zero velocities while the record keeps the recorded ones | `replay_ic_fidelity` | `tunnel_depth` |
| 18 | `wrong_omega_frame` — the body-frame ω handed through as world | `real_rot_err` | **`replay_ic_fidelity`**, `tunnel_depth` |
| 18 | `table_hologram` — the table loses its collider | `tunnel_depth` | `replay_ic_fidelity` |

A `must_red` may name **more than one** check: rung 9's `frozen` has to redden
both the sensitivity control and the analytic anchor, because a frozen world
satisfies determinism and only those two can say so. A fault row may also carry
per-fault kwargs for `arm.run` — rung 18's faults run the contract's small toss
subset, and the **baseline they are judged against runs the same subset**, or
the two would not be comparable.

Three of the new faults are worth reading for what they must leave GREEN:

* **rung 9's `frozen` is rung 9's `slide`.** A world that cannot move is
  perfectly deterministic, so `repeat_delta` must stay green while the two
  checks that exist to stop that both go red. Without it, rung 9 would be
  satisfiable by an engine that simulated nothing.
* **rung 18's `wrong_omega_frame` must leave `replay_ic_fidelity` green**, and
  that is the seam: the engine accepted exactly what it was handed, so the IC
  check correctly does not blame it for the harness's frame error. Only
  agreement with the recording sees it. Reversing that would make the IC check
  a second, weaker copy of the position check.
* **rung 11's `lane_offset` is a SCENE offset, not a per-step write** — the
  correction recorded below for rung 7 applies unchanged at N = 16.

### 6a. Detector validation — a green that cannot be made red is not a pass

Adopted from OmniBench lane 4b, which reported `cliff_detector_validated:
false` and refused to call its own green a pass.

> For every rung and every arm, that rung's **must-red** faults must have been
> shown to go red **on that arm**. A rung whose battery has never been run on
> an arm — or was run and did not go red — reports that arm's row as
> `UNVALIDATED`, and `UNVALIDATED` is not a pass.

`selftest.detector_validation()` produces that record and `--self-test` prints
it. Note what it is scoped to: a fault declared for a **variant** validates the
variant, not the row. Rung 11's constraint-starvation fault is a `mujoco_warp`
study and lives in `variants.py`; if it cannot be made to bite, the **GPU
variant** is `UNVALIDATED`, and the CPU row — whose detectors `stalled_robot`
and `lane_offset` do validate — is not.

Each fault is **surgical along a seam that matters**. Rung 5's pair moves the
*scene* with the motion intact and stops the *motion* with the scene intact.
Rung 6's `no_stop` breaks the acting half and must leave the sensing half green
— a controller that reads a sensor correctly and then does nothing with it is a
real bug and must not read as a broken sensor. Rung 6's `bounce` is rung 4's
`slide`: the final state is exactly right and only the whole-run invariant can
see it. Rung 8's `no_grip` is the **causal control** and is the reason rung 8
may claim the fingers did the lifting.

Two design corrections worth not repeating, both found by running the battery:

* **`lane_offset` is a scene offset, not a driver-side walk.** The first
  version had the supervisor write the robot's `y` every step while reading `x`
  and `z` back, on the theory that its forward motion would be untouched.
  Measured: a per-step field write costs the body its state — `distance_worst`
  came back **0.94** and `wheel_omega_worst` **0.93**, so both must-green
  companions went red and the fault destroyed everything it was meant to
  isolate.
* **`no_stop` may not require `sensor_agrees` to stay green.** A rover that
  never stops ends with its sensor *buried inside the wall*, where "distance to
  the near face" is a distance to a surface behind the ray; the residual reads
  **0.293 m** there against **2.8e-06** on the honest run. That is a property
  of the fault, not of the sensor.

**A must-green companion only means something if it was green without the
fault.** `selftest.live_cases` runs each rung's honest cell once, caches it, and
reports a companion that was *already* red as `MASKED` rather than counting it
against the fault. Without that, an honest fault on a rung the engine already
fails reads as "the fault did not go red" when its must-red went red exactly as
required — which looks like a defect in the ladder and is not one. The failure
still shows up loudly where it belongs: on that rung's own row in the table.

Supporting `fault=` is **optional** for an arm — if you do not, return a `meta`
with an `error` saying so and the self-test will report that arm's live proofs
as missing rather than as passing. If you do support it, the fault must be
injected into the **run** (the scene or the driver), never into the
measurement.

`rung 4 / slide` is the important one: drag the chassis at exactly
`rungs.rolling_speed(rungs.RUNG4_OMEGA_CMD)` with every wheel commanded to
zero. `distance` must stay green and `rolling_consistency` must go red. That
asymmetry is the entire argument for asserting two things on rung 4.

---

## 7. House rules for this tree

* Three lanes share this working tree. Kill only your own PIDs.
* A running engine locks `omnisim-bin.exe`.
* Never run a git worktree/checkout operation while `projects/` is a junction.
* Keep the GPU under 75 °C; do not pin `mujoco_warp` — CPU `mj_step` is the
  default and is faster at this scale anyway.

---

## 8. Rungs 9 and up — the plan, and the six amendments (ADOPTED)

[`PLAN_9_20.md`](PLAN_9_20.md) designs eleven rungs for tiers C (stress), D (the
robot system) and E (the frontier). **Three of them are built — 9, 11 and 18 —
and the other eight are deliberately not.** §3c above records why, and it is the
rule for anything added next: a rung whose only possible outcome is "three
mature engines agree" is not worth the days, because on correctness assertions
alone this ladder cannot differentiate us from upstream Webots anywhere in
tiers C or D and can never beat MuJoCo on fidelity — MuJoCo *is* our solver.
Read the plan before adding anything above rung 8; **nothing in §5 of it is a
result**, and its predictions carry confidences rather than numbers.

**The six amendments it identified are now ADOPTED**, and each is specified
where it bites rather than only here:

| # | amendment | where it now lives | status |
|---|---|---|---|
| A | a rung is a scene **family**; a cell may contain more than one **run**, tagged and parameterised by `rungs.py`, from **distinct processes** | §4a; `rungs.MULTI_RUN`, `analysis.runs_of` | **adopted** — rungs 9, 11, 18 |
| B | **`NOT_EXPRESSIBLE` per CHECK**, declared in the arm's source with the capability named and cited — never inferred from a failed run, never green, never counted as a failure | §5a; `rungs.apply_not_expressible`, `run_ladder.cell_verdict` | **adopted** |
| C | **detector validation** — a rung's must-red faults must have been shown to go red *on that arm*, or that arm's row is `UNVALIDATED` and is not a pass | §6a; `selftest.detector_validation` | **adopted** |
| D | contract-owned **sample decimation**, and a **lossless float round-trip** in the sample document | §4b; `RUNG9_SAMPLE_EVERY`, `RUNG11_SAMPLE_EVERY` | **adopted** |
| E | **variants stay variants** — never the headline; a declared constraint budget is generous, never sized at a scene's measured peak | `omnisim/variants.py`; `RUNG11_NJMAX` = 4096 | **adopted** |
| F | an expected value may also come from an external **measurement of reality** (vendored, licensed, self-calibrating). A measurement of a *simulator* remains forbidden, including ours and including a previous build | §3c rung 18; `rungs.rung18_dataset()` | **adopted** |

Amendment F is the only one that touches §1, and it survives §1 intact: a
golden captured from today's behaviour certifies today's defects; **a tracked
cube does not know what a simulator is**. The narrowness is the point — the
recording must be external to this project, vendored with its licence, and
carry its own calibration record, and lane1r meets all three because it
re-derives its scale factor, timestep and quaternion convention on every run
rather than trusting the dataset's metadata.

Amendment E carries one rule that is easy to read as a caution and is not:
**a constraint budget is declared GENEROUS, never sized at a scene's measured
peak.** Setting `newtonNjmax` to a scene's own peak (320) moved results 8.81 m
versus every other size, with a 1.71 m run-to-run spread, while 512 / 2048 /
4096 agreed to 1e-4. Rung 11 declares 4096 against a largest-fleet requirement
of 1024.
