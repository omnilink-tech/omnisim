# MuJoCo-column bring-up record — **T4 `humanoid`**

**2026-08-02.** The T4 half of the MuJoCo column: what it takes to bring the
shipped two-legged description into a scene, whether the walk is **achievable
at all**, what the tier's two cells actually print, and the one hole in the
support measurement that this rung's own task file flagged and nobody had
tested. Companion to [`BRINGUP.md`](BRINGUP.md) (T1),
[`BRINGUP_T2.md`](BRINGUP_T2.md) (T2) and [`BRINGUP_T3.md`](BRINGUP_T3.md)
(T3), and to
[`docs/developer/capability-ladder-plan.md`](../../../../../docs/developer/capability-ladder-plan.md)
§2 T4.

> **Nothing here is a result.** Every number below comes from a **scripted**
> run — a human wrote the gait knowing the thresholds. A ladder cell is an
> autonomous agent given one sentence and no help (plan §2), and no figure in
> this file may be reported as one. What is proven here is that the *task* is
> achievable **in its supported cell**, that it is achievable there **with no
> learning of any kind**, and that the *instrument* works on this column.

> ⚠ **AND IT IS A SUPPORTED CELL.** The walk below runs with an attitude-and-
> lateral rig on the trunk. It is **not a free-standing walk** and no sentence
> about it may say otherwise — `AGENTS.md`'s humanoid disclosure rule binds
> this file exactly as it binds every G1 result in the tree. What the rig does
> *not* do is carry weight: `fx = fz = 0` on every tick, by construction and by
> test. That narrows the claim; it does not remove the harness.

---

## 1. What this settles, and why it had to be settled twice

`tasks/T4_humanoid/meta.json` → `container.authored_here.before_the_freeze`
makes it a precondition of the freeze: *"Demonstrate the walk once, by hand, on
at least one column, and publish the recipe alongside the scaffolding. If it
cannot be demonstrated anywhere, the tier is unachievable-as-shipped and that is
a finding about this file, not about any simulator."*

It was demonstrated on 2026-08-02 — **from a scratch harness outside this
tree**. The task file said what that was worth, in its own words:

> *"the numbers above are a claim about this container that a reader cannot yet
> re-derive from the repository … Until that lands, this tier's achievability is
> DEMONSTRATED BUT NOT REPRODUCIBLE, and it should be read as the weaker of the
> two."*

This file is that gap closed: the recipe rebuilt inside the tree, committed, and
run end to end.

```
python tests/benchmarks/ladder/adapters/mujoco/run_t4.py --out <dir>
```

≈ 23 s of wall clock for the task's own 300 s window (phase B is 15 s of it).
CPU only. No GPU, no network, no tokens.

The three runs this record is made of, each of which exists for a measurement:

```
run_t4.py --out <dir>                     # the supported cell (default)
run_t4.py --out <dir> --rig none          # the same script, wrench off
run_t4.py --out <dir> --rig weld          # the constraint rig -- SEE §6
run_t4.py --out <dir> --skip-run          # re-grade the artifacts on disk
run_t4.py --out <dir> --ground plane      # the recorded recipe's floor
run_t4.py --out <dir> --unlimited-actuators   # the recipe's actuators
```

---

## 2. The two cells, verbatim

Both come from **one script**. The only difference between them is whether the
rig's wrench is applied, and the number that separates them is printed in both.

### 2.1 `--rig wrench` (the default) — **T4-supported**

`T4_humanoid` **PASS 5/5** · `cell: T4-supported` ·
**`supported: peak 0.04715 x body weight, 50.2 N.m, 100% of window`** ·
`support_attestation: attested` · `arena_attestation: attested` ·
`method: scripted` · `excluded from comparison: False` ·
`unanswered channels: none` · `vacuous clauses: none` · contract checks
(`check_bundle` + `check_t4_evidence`) **clean, zero findings** ·
`termination: time_limit` — *the clock ran out while it was moving* ·
`distance_to_termination_m: 73.3048`.

| # | assertion | measured | threshold |
|---|---|---|---|
| **T4.1** | crossed ten metres in one continuous run | furthest from the start **73.3048 m**; same at the end **73.3048 m**; integrated path **90.0811 m**; largest single inter-sample displacement **0.0057 m** at a **0.01 s** sample interval; clock strictly increasing; the region allowed **145.123 m** of straight run against the task's stated **15.0 m** | ≥ 10.0 m · ≤ 0.50 m · increasing |
| **T4.2** | never fell | settled standing height **0.5987 m**; lowest base height **0.5877 m** against a floor of **0.3592 m**; max \|roll\| **0.0557 rad**, max \|pitch\| **0.1013 rad**; combined tilt off vertical **0.1121 rad**; **0** samples over any bound | ≥ 0.60 × standing · ≤ 0.8 rad · every sample |
| **T4.3** | walked; did not slide or drift | mean speed made good over the scored window **0.2348 m/s** (whole recording 0.2443); scored window **42.60 s**, ending where the 10 m bar was first reached at **10.001 m**; **183** make-and-break ground-contact transitions, **91** completed lift-and-place cycles, across **foot_l / foot_r**; 327 424 contacts observed, all naming two distinct bodies; the contact query ran **75 000** times. **Vertical bob 0.00183 m** — *measured, not graded* | ≥ 0.15 m/s · ≥ 8 transitions · *(bob: reported, see §8.2)* |
| **T4.4** | nothing but the ground touched it, and what was applied to it was measured | **0** contacts with any body that is not the ground inside the scored window (83 336 with it); peak applied force **5.5509 N = 0.04715 × body weight** against the unsupported cell's bound of **2.3544 N**; peak applied torque **50.1952 N·m** against **2.0**; live **100 %** of the window. **Per channel — and this is the whole reason the channel is per-axis:** `fx` **0.00 N / 0 %**, `fz` **0.00 N / 0 %**, `fy` 5.5509 N / 96.46 %; `tx` 25.5016 N·m / 96.46 %, `ty` 45.4579 N·m / 100 %, `tz` 25.8135 N·m / 96.46 % | 0 foreign contacts · the wrench **selects the cell and never fails the row** |
| **T4.5** | the run is real | exit **0**, **0** error-class lines (all seven `mjData.warning` physics counters zero), reached finalize on the **engine-authored** `mj_saveLastXML`, driver completed; attribution `mujoco 3.8.1` / solver `Newton` / integrator `implicitfast` / cone `pyramidal` / timestep 0.002 s; **the driver is attested to have loaded**, by path + sha256 `2f9101228d70c959…` and **150 750** actuator-command writes over 150 000 physics steps | 0 · true · attributed · **loaded** |

**⚠ The carrying channel is idle and the attitude channel is continuous.** That
is the same shape as the only real support rig this tree has ever measured per
channel (`docs/developer/g1-endurance-2026-08-01.md` §4: `fz` 0.00 N for 0 % of
a whole 10 m walk while the attitude channel ran at a 69.2 N·m peak for 100 %
of it). Here it is not a coincidence and not a discovery: `support_wrench()`
returns `fx = fz = 0` by construction, and
`test_the_rig_never_carries_any_weight` asserts it on the live model rather
than trusting the return statement. **It is still a rig.**

**`reuse_class: null`.** No grader can decide it (§9 Q3: the reviewer
arbitrates), and §8 forbids quoting a T4 cell without it. It is printed as null
by the tool that prints the cell, with the sentence saying such a cell is not
publishable.

### 2.2 `--rig none` — the same script with the wrench off — **T4-unsupported**

`T4_humanoid` **FAIL 2/5** — red on **T4.1, T4.2 and T4.3** ·
`cell: T4-unsupported` ·
**`unsupported: peak 0 x body weight, 0 N.m, 0% of window`** ·
`support_attestation: attested` · `method: scripted` ·
`unanswered channels: none` · `vacuous clauses: none` · contract checks clean ·
`termination: fell` — *the fall test was breached at t = 2.430 s* ·
`distance_to_termination_m: 0.6898`.

| # | assertion | measured |
|---|---|---|
| **T4.1** | ✗ | furthest from the start **0.6898 m**, at the end 0.5110 m |
| **T4.2** | ✗ | settled standing **0.5973 m**; lowest base **0.1389 m** against a floor of 0.3584 m; max \|roll\| **1.5927 rad**; **29 758** samples over the bound; first breach at **t = 2.43 s** |
| **T4.3** | ✗ | mean speed made good **0.0017 m/s**; 39 705 transitions (a fallen robot's legs still make and break contact — which is exactly why the speed clause is in the conjunction) |
| **T4.4** | ✓ | 0 foreign contacts; peak **0 N**, **0 N·m**, live **0 %** — every per-axis channel identically zero |
| **T4.5** | ✓ | exit 0, finalize attested, driver loaded |

**One recording, one switch, two cells, and the number that separates them is
printed in both.** A rig is what stands between this robot and the floor, and
the tier's own two-cell mechanism is what makes that visible instead of
arguable. What this pair does **not** say is that the unsupported cell is
unreachable: it is one open-loop gait on one column, and a feedback controller
is the obvious next thing to try and was not tried.

### 2.3 Reproducibility, cost and machine

Two independent cold runs of the whole pipeline (build → phase B → grade)
produced **bit-identical** base pose series — sha256 `d14f86c00dcf8a9d…` both
times — and identical verdicts, at **6.80 s / 6.77 s** wall for 90 s of
simulated time. The canonical 300 s run is **22.6 s** wall (phase B 15.2 s),
i.e. roughly **20× realtime on one CPU core**, including a per-step contact
scan, a 10 ms pose+orientation dump, a per-step applied-wrench read and a
per-sample constraint-reaction solve. Its run directory is **49 MB**.

**Machine.** `9722d23d12a3` — host `hc385771a14`, AMD64 Family 25 Model 80
(16 cores), RTX 3060 Laptop (**unused**: this is CPU `mj_step` throughout),
Windows 11 (10.0.26200), CPython 3.12.9, `mujoco` **3.8.1** (the official PyPI
wheel). Same box as the T1, T2 and T3 records, confirmed with
`python projects/policies/common/env_fingerprint.py`.

---

## 3. The pieces

| file | role |
|---|---|
| `t4_scene.py` | **the shipped URDF → one MJCF scene**, every edit a cited `BuildStep`, plus the ground/light/actuators/**armature** URDF cannot express — and, under `rig="weld"`, the constraint rig §6 exists to measure. Ends by loading the written file **back off disk** and re-checking the armature *and the equality count* there. |
| `t4_drive.py` | the **scripted gait and the rig** — the controller half of the deliverable. Zero imports from `ladder`; every length it uses is measured off the compiled model; it falsifies its own IK against the compiled forward kinematics over a whole gait cycle on every deployment. ⚠ Unlike every other driver in this column it writes `xfrc_applied`, on purpose: that is the channel T4 measures. |
| `runner_t4.py` | **phase B**: load the scene, import the driver *by path*, step the loop, write the artifacts. The only genuinely new code on this rung is its **support channel** — the applied wrench **plus the equality-constraint reaction on the base** (§6) — and its refusal to attest at all for a kinematic base. |
| `evidence.py` (+) | `t4_channels()` — the eight channels mapped into `ladder.graders.t4_evidence`, each with the citation naming the MuJoCo call behind it, and a `support_reading` switch that serves the same recording as the attested total **or** as an `xfrc_applied`-only column would have seen it; `t4_run_standalone()`. |
| `recording.py` (+) | learns `t4.json`, and surfaces `equality_reaction_live_samples` / `neq` in its summary, because *"was anything holding this robot that the wrench array could not see"* is the one question this rung adds. |
| `run_t4.py` | end to end in one command: build → phase B → both contract checks → grade through the **real** `ladder.graders.t4`. On a welded run it grades the same recording **twice** and prints both cells. |
| `test_mujoco_t4.py` | **32 tests**, **15** of which need no simulator at all. Four are tripwires on MuJoCo, three on the recorded recipe and the task file. |

A T4 deliverable on this column is two files — `scene.xml` and `drive.py` —
because MuJoCo cannot express a controller in a scene. The driver is imported
with `importlib.util.spec_from_file_location` and **never put on `sys.path`**
(asserted by a test).

---

## 4. Four things bite

### 4.1 ⚠ **URDF cannot express rotor inertia, and without it this robot cannot stand**

Unchanged from T3 and **not new here** — but the container's own
`PROVENANCE.txt` repeats it for a reason worth repeating again: *"a two-legged
robot has less margin than a four-legged one, not more, so an agent that reads
that first behaviour as 'the robot is badly modelled' or 'the physics is wrong'
will fix the wrong thing."* `<dynamics>` carries damping and friction and
nothing else, so `armature` defaults to **0** and a `kp = 400` servo on a 1.8 kg
shank at a 2 ms step is numerically unstable. This scene sets
`armature = 0.02 kg·m²` and `damping = 0.6 N·m·s/rad` on all twelve hinges, on
the MJCF side, and re-checks both **on the model reloaded from disk** —
because phase B re-runs the file, and "it compiled in memory" is not a
deliverable.

**Trap inside the trap, and it cost the first build:** `MjsJoint.armature` is a
scalar and **`MjsJoint.damping` is an `mjtNum[3]`** (ball and free joints have
three). `j.damping = 0.6` raises `TypeError`; `j.damping = [0.6] * 3` is
correct, and the compiler reads only the first slot for a hinge.

### 4.2 The root link has no joint to the world, so an importer welds it down

T1's finding, unchanged, and it applies to every URDF: `compiler/fusestatic`
defaults to **true for URDF**, which does not merely weld a jointless root but
*absorbs it into the world body*, and the robot then has no `base_link` at all.
T4 grades the base **by name**, so a scene built on the defaults is one the
grader must refuse. Fixed with `fusestatic="false"` plus URDF's own
`<joint type="floating">`. `test_the_urdf_defaults_absorb_the_base_into_the_world`
is the tripwire.

### 4.3 ⚠ **The floor's LENGTH is set by the task's own clock, and getting it wrong is a FALL, not an arena finding**

**New on this rung**, and it is the one scene parameter that cannot be copied
from T3. `phases.standalone.duration_s` is **300 s** — deliberately generous,
so that a walker slower than the speed floor is *measured as slow rather than
truncated*. This gait makes ~0.24 m/s, so it covers **73 m** in that window. A
T3-sized 45 m slab would therefore end the run with the robot **walking off the
edge of the world**, and the fall test would record that as a fall at
z ≈ −10³ m with `termination_cause: fell`. It is not an arena finding: the
arena rule reports a run *at* the edge, and a robot that has left the floor
entirely is past that.

`GROUND_X` is 150 m long for exactly that reason — `duration_s × the gait's own
nominal speed`, with room — and
`test_the_floor_is_long_enough_for_the_tasks_own_clock` asserts the derivation
rather than the number. Measured directly while attributing §7: with the
recipe's own hip-sign the robot walks in **−x** and does exactly this, reaching
z = −23 984 m on a 150 m slab that only extends 5 m behind it.

The floor is a **box** and not a plane, for T3's reason (a plane geom's world
AABB is ±1e10 m, so the arena channel could not print a readable number), and on
*this* tier that matters more: an unattested arena makes the cell **incomplete**
rather than merely under-annotated. `--ground plane` is kept so the two can be
compared; it changes the walk by **nothing measurable** (§7).

### 4.4 The ankle-pitch actuator saturates at the robot's own declared effort — and the walk does not care

The recorded recipe stated *"no force limit"*. Unlimited, this gait peaks at
**95.30 N·m** on an ankle-pitch joint whose own description declares **60**, and
63.88 N·m on the other — so the recipe's setting would have demonstrated that a
robot *stronger than the one the container ships* can cross ten metres. Clamped
to each joint's declared effort (the default here), both ankles **saturate at
exactly 60.00 N·m** and the same gait covers **21.6464 m against 21.6459 m** in
90 s. The clamp costs 0.5 mm in 21 m.

The peaks are measured and printed against the declared efforts on every run,
either way, so *"did the walk stay inside the robot's own limits"* is a
measurement rather than an assumption. Same finding as `BRINGUP_T3.md` §4.4,
different joint, and it lands on the ankle here because the level-foot
constraint `ankle = −(hip + knee)` makes the ankle carry the whole crouch angle.

---

## 5. Three things the recorded recipe and the task file got wrong

None of these were tuned away. All three are asserted against the compiled model
by a test, and their cost is reported in §7.

### 5.1 ⚠ **The recipe's leg IK walks this robot BACKWARDS**

The recorded recipe solves the hip pitch as `hip = alpha + beta`. On this robot
the hip pitch axis is `+y`, so rotating the thigh by `+θ` maps a point at
`(0, 0, −L)` to `(−L sin θ, 0, −L cos θ)` — **a positive hip pitch swings the
foot backwards**. The correct solution is `hip = beta − alpha`, and the recipe's
expression is exactly this one **mirrored**: its answer for a foot in front is
this one's answer for a foot behind.

The two agree *exactly* when `x = 0`. That is why it survived: the scratch
harness's self-check solved only the standing pose. Measured, on an infinite
plane so the slab could not confound it: the recipe's gait walks **−24.75 m**
where the corrected one walks **+21.64 m**.

`setup()` here therefore checks the IK at **seventeen** targets, sixteen of them
points on the trajectory the gait actually commands, and records every commanded
point the robot cannot reach by name rather than silently holding the previous
solution. Residual over a whole cycle: **1.11 × 10⁻¹⁶ m**, zero unreachable
targets. `test_the_recipes_hip_sign_walks_this_robot_backwards` states the whole
thing as arithmetic, with no simulator.

### 5.2 ⚠ **Ankle-to-sole is 0.06 m, not the recipe's 0.04 m**

The recipe's constant was `ANKLE_TO_SOLE = 0.04`, commented *"ankle_roll offset
0.02 + foot half height 0.02"* — which drops the foot box's own `−0.02 m` origin
offset. Measured off the compiled model it is **0.06 m**, and the container's
own `PROVENANCE.txt` agrees arithmetically: *"with every joint at zero … the
sole sits 0.725 m below the base origin"* = 0.105 + 0.28 + 0.28 + **0.06**, and
the model returns **0.7250 m** exactly.

Nothing here is typed: `measure()` reads it from the ankle anchor and the foot
geom's own half-height. Effect on the walk: **+1.53 m in 90 s** for the recipe's
value, because a 20 mm deeper effective crouch lengthens the stride.

### 5.3 ⚠ **`meta.json`'s reach band and crouch height carry the same 0.04 error**

`robot.standing_geometry` states two numbers that the shipped description does
not produce:

| claim in `meta.json` | measured on the compiled model |
|---|---|
| *"each leg reaches between **0.09 m** and **0.60 m** from the hip pitch axis to the sole"* | **0.2888 m … 0.6198 m** |
| *"with a crouch of hip pitch 0.30, knee −0.60, ankle pitch 0.30 the sole sits **0.680 m** below \[the base origin]"* | **0.7000 m** |
| *"with every joint at zero … the sole sits 0.725 m below the base origin"* | **0.7250 m** ✓ |

Both wrong numbers are exactly what the 0.04 ankle-to-sole would give
(0.2687…0.5998, and 0.680), and the one right number is what 0.06 gives — so the
file is internally inconsistent, and the *zero-pose* line is the one to trust.
The lower reach bound is not close under either: 0.09 m would need the two
0.28 m links folded almost flat, which a knee bounded to `−2.30` cannot do. The
claim's *substance* survives — the knee limit and not the reach is what binds —
and the numbers are corrected here, exactly as `BRINGUP_T3.md` §5.1 corrected
the equivalent band one rung down.
`test_the_reach_band_the_task_file_states_is_wrong_and_the_model_is_right` is
the tripwire; `meta.json` has been annotated rather than silently edited.

---

## 6. ⚠ The constraint rig: the tier's own open question, executed

`meta.json` → `container.authored_here.an_open_question_the_demonstration_exposed`
records the hole and says no grader can close it:

> *"A SUPPORT RIG IMPLEMENTED AS A CONSTRAINT REPORTS NO APPLIED WRENCH. …
> A rig implemented instead as a weld, an equality constraint, a mocap
> attachment or a kinematic base would hold the robot just as firmly and the
> wrench channel would read ZERO — and the run would be published in
> T4-unsupported, which is the cell the plan says must be 'numerically nothing'.
> Nothing in the tier's text closes this, and nothing in this grader can: it
> reads what the column attests."*

**It is true, and here it is.** `--rig weld` builds the scene with a mocap body
and a `<weld>` equality binding `base_link` to it; the driver advances the
anchor at the gait's nominal speed and keeps it upright, and the legs run the
identical gait. `mjData.xfrc_applied` is **identically zero for the whole run**
— asserted, not assumed. One recording, graded twice:

| the column attests | cell | published as | reds | vacuous | excluded |
|---|---|---|---|---|---|
| the wrench **+ the equality reaction** (this column) | `T4-supported` | **`supported: peak 2.188 x body weight, 94.09 N.m, 100% of window`** | none | none | no |
| `mjData.xfrc_applied` **alone** | `T4-unsupported` | **`unsupported: peak 0 x body weight, 0 N.m, 0% of window`** | none | none | no |

Both are **PASS 5/5** over the same 23.3608 m of travel, `termination:
time_limit`. The second row is the hole: a robot held rigidly upright by a
constraint, walking 23 m it could not walk unaided, published in the cell the
plan reserves for *numerically nothing* — with **no red assertion, no vacuous
clause, and not even an exclusion from comparison** to warn a reader. Nothing
downstream can tell it from an honest unsupported walk.

**The column-side attestation that closes it.** MuJoCo assembles constraint rows
in the fixed order *equality, friction loss, limit, contact* and publishes the
first two counts as `mjData.ne` / `mjData.nf`, so the equality rows are exactly
`efc_J[:ne]` and the generalized force they apply is
`efc_J[:ne].T @ efc_force[:ne]`. Restricted to the base's own free-joint DOFs
that is a wrench about the base's centre of mass — the translational part
already in the world frame, the rotational part in the **body** frame (measured
on 3.8.1, not assumed: a free joint's linear velocity is global and its angular
velocity is local), so the torque is rotated by `mjData.xmat` before it is
added. Contact rows are deliberately excluded: T4.4 excludes contact by its own
wording and counts it separately, and folding them in would publish the floor as
a support rig.

`test_a_welded_body_reports_its_whole_weight_as_a_constraint_reaction` is the
tripwire on that arithmetic: a 12 kg body welded to a mocap anchor applies
nothing through `xfrc_applied`, and its equality reaction totals **117.72 N** —
its own weight, to three decimals. If MuJoCo ever changes its row order or its
free-joint frame convention, that test says so.

**And the half a reaction cannot answer.** A **kinematic base** — one welded
structurally into the world's weld group — has no DOFs at all, so there is no
degree of freedom for a reaction to appear on and no honest number to report.
This column then **declines to attest**: `support_attestation` becomes
`unverified` and the run lands in `T4-support-unverified`, which is excluded
from comparison and never credited. `T4-unsupported` means *"nothing was
applied"*; `T4-support-unverified` means *"nobody knows"*; a held robot belongs
in the second and never in the first.
`test_a_kinematic_base_is_support_unverified_and_never_unsupported` asserts it.

**What is still open, and it is not fixable here.** This closes the hole **on
this column**. It does not close it on any other, and it cannot: the grader
reads what a column attests, and an omitted constraint reaction is
indistinguishable from a rig that was never there. `LADDER_REQUIRED_EVIDENCE_T4`
already states the requirement in as many words; what this adds is a *worked
demonstration that the requirement bites*, with both cells printed, so a
reviewer checking another column knows exactly what to ask for and what a
failure to supply it looks like. The task file has been updated with this
result.

---

## 7. The reproduction is 13 % short of the scratch figures, and here is why

Reported rather than tuned away. All measurements at the scratch harness's own
90 s window, this column's default configuration unless stated.

| | scratch harness | this oracle |
|---|---|---|
| travel in 90 s | 24.94 m | **21.6464 m** (−13.2 %) |
| direction | **−x** (see §5.1) | **+x** |
| speed made good to the 10 m bar | 0.2699 m/s over 37.06 s | **0.2348 m/s over 42.60 s** (−13.0 %) |
| footfalls in that window | 160 | **183** — i.e. **4.32/s vs 4.30/s**: the *cadence* reproduces exactly, only the distance per step differs |
| peak applied force | 5.14 N | **5.5509 N** (+8 %) |
| peak applied torque | 34.22 N·m | **50.1952 N·m** (+47 %) |
| bob (RMS about the trend) | 0.0031 m | **0.00183 m** |
| termination | `arena_geometry` at 24.94 m | `time_limit` (and `arena_geometry` was never reachable — see below) |

**The distance gap is entirely the two IK corrections.** Measured by putting the
recipe's own constant and its own sign back, one at a time, everything else held
(90 s, wrench rig):

| configuration | travel | direction | peak F | peak M |
|---|---|---|---|---|
| **as shipped** (corrected IK, ankle-to-sole 0.06) | 21.637 m | +x | 5.557 N | 50.483 N·m |
| the recipe's ankle-to-sole = 0.04 | 23.164 m | +x | 5.863 N | 49.700 N·m |
| the recipe's `hip = alpha + beta` (on a plane) | 24.754 m | **−x** | 5.074 N | 35.993 N·m |
| **both — i.e. the scratch script** (on a plane) | 25.838 m | **−x** | 5.199 N | 35.863 N·m |

The last row reproduces the scratch harness's own 24.94 m to within 3.6 %, which
is what makes the attribution a measurement rather than a story. Neither
correction was made to move a number: one is a sign error and the other
contradicts the container's own stated geometry.

**The torque gap is the direction reversal.** Walking backwards costs 36 N·m of
attitude authority and walking forwards costs 50 — on this robot those are not
mirror images, because the foot box extends **0.03 m ahead of the ankle**
(measured, and recorded in the run), so the contact patch sits under a different
part of the sole in the two directions.

**⚠ The force gap is not a gap at all — it is the DENOMINATOR, and it is an
open question for the tier.** The scratch harness attested the *whole robot's*
25.6 kg as the base's mass; this runner attests `mjModel.body_mass[base_link]` =
**12.0 kg**, the base body's own. The tier publishes the peak as *"a multiple of
body weight"* and sets the unsupported cell at `0.02 × m·g`, and the grader
takes that `m` from this channel — so on this robot the same unchanged run reads
**0.04715 × body weight against a 2.3544 N bound** (base body) or
**0.02210 × body weight against a 5.0228 N bound** (whole robot). A factor of
**2.13**, on the number the cell boundary is made of.

This column reports `body_mass`: it is the literal reading of *"the base's
mass"*, it is what the tier below already reports, and it is the **stricter** of
the two. It also records `subtree_mass_kg` beside it and writes the whole
ambiguity into the channel's own citation, so it travels into the verdict rather
than living in this file. **Which one the tier means is a question for its
owner**, and it is recorded in `meta.json` rather than decided here — deciding it
after seeing a measurement is the act §5a voids a pass for.

**Two divergences that turned out to cost nothing**, both measured rather than
argued: `--unlimited-actuators` (21.6459 m vs 21.6464 m, §4.4) and
`--ground plane` (21.6464 m, identical to four decimals). And one that is a
consequence of the arena rather than the gait: the scratch harness ran on an
**infinite plane** while telling the grader its arena was a 33 m box, so its
robot walked 24.94 m *outside the region it had declared* and the run was
recorded `arena_geometry` — the region ended, the floor did not. Here the floor
and the declared region are the same box, so the two cannot disagree.

---

## 8. The eight T4 channels, and the two readings of one of them

T4's evidence contract **is** T3's — `t4_evidence.T4Evidence` subclasses
`T3Evidence` and adds no field, because the tier replaces an *assertion*, not a
channel list. All eight are answered on this column; `unanswered_channels` on
the oracle's verdict is **absent**, which is what "nothing fell back" looks like.

| channel | MuJoCo source | note |
|---|---|---|
| `base_pose` | `mjData.xpos` **+ `mjData.xmat`**, 10 ms | `xmat` is a world-from-body 3×3 already. **Without the rotation half of T4.2 cannot be answered at all** |
| `standing_height` | the base's own world z between the settle window's last `mj_step` and the first recorded one | frozen **by construction**: the recorder owns the loop |
| `gait_contacts` | `mjData.contact` every physics step, emitted every 2 steps, **with the times the query ran** | 75 000 queries. ⚠ **Deduplicated to one record per (robot body, other body) pair per query** — a box foot flat on a plane makes four coplanar points naming the same pair, and the tier counts BODIES. A divergence from T3, stated in the channel's citation |
| `applied_support` | `mjData.xfrc_applied` on the base **+ the equality-constraint reaction** | §6. Recorded **twice**: the attested total and the `xfrc_applied`-only reading |
| `arena` | the union world AABB of the bodies the task names as the walking surface | exact, because the floor is a **box** (§4.3) |
| `base_mass` | `mjModel.body_mass` | ⚠ the base body's own mass, not the robot's — §7, and the citation says so |
| `gravity` | `mjModel.opt.gravity` of the model that ran | |
| `controller_load` | the driver's path, **sha256**, whether `setup()` ran, and the number of `control()` writes | a positive attestation, and **not an exit code** |

**Why the wrench total is complete rather than partial — the argument, not the
assertion.** In MuJoCo a wrench reaches a body through exactly four routes:
gravity (excluded by the tier), contact (excluded by the tier, counted
separately), an actuator transmission, or `xfrc_applied` — **plus** a constraint
reaction, which §6 is about. This scene's actuators are all `mjTRN_JOINT`
transmissions on **hinges**, none on the base's free joint; `mjModel.ntendon` is
0; and `mjModel.neq` is 0 on the default build and 1 on the weld build with its
reaction counted. All four counts are written into every run, so a reader can
check completeness rather than take it on trust, and
`support_attestation()` **refuses to attest** if any of them says the total
would be partial.

### 8.2 The retired bob clause, confirmed a second time on a second robot

T3 retired the `≥ 0.005 m` vertical-oscillation conjunct on 2026-08-02 after it
failed a robot that was plainly walking. This tier calls that row rather than
copying it, so it inherits the retirement automatically — and **this walk
confirms the defect independently on a two-legged robot**: 73 m of travel, 183
footfalls, never fell, and a bob of **0.00183 m**, red against a bar that is no
longer applied. The task file already predicted this from the scratch run
(0.0031 m); the rebuilt figure is lower still. What remains open is a *text*
divergence, not a behaviour: the plan still states the clause and the graders no
longer apply it.

---

## 9. Known gaps and caveats (state, don't bury)

1. **This is one column, and one gait.** Achievability is proven for *a*
   scripted gait, on level ground, on MuJoCo, in the **supported** cell. It says
   nothing whatever about OmniSim, about any other simulator, or about any
   agent. The **OmniSim** T4 channels are unimplemented; the same list
   `BRINGUP_T3.md` §7.2 gives applies unchanged, with **one addition that is
   specific to this rung**: whatever OmniSim's support-rig hook applies, its
   ladder-side channel must also count anything holding the base through a
   constraint or a kinematic parent, or it can publish a harnessed G1 as
   `T4-unsupported` — see §6.
2. **The unsupported cell is not shown to be unreachable.** One open-loop gait
   fell at 0.69 m. A feedback controller is the obvious next thing to try and
   was not tried, and the plan's own pre-registration for that cell
   (`not_achieved`) is untouched by this run either way.
3. **The oracle is a scripted control and its verdict is not a cell.** It is
   also not a *baseline* an agent must beat — it is an existence proof.
4. **The rig is a rig.** `fx = fz = 0` narrows what may be claimed; it does not
   make the walk free-standing. The trunk is held upright continuously by a
   400 N·m/rad attitude PD, and the honest sentence is the cell's own:
   *supported: peak 0.04715 × body weight, 50.2 N·m, 100 % of window*.
5. **The scored window is 42.6 s of a 300 s recording.** T4.3 is measured over
   the walk to the bar, as the tier requires; the robot then keeps walking for
   another 63 m, which is recorded in `distance_to_termination_m` and graded by
   nothing. `meta.json` → `T4_OPEN_QUESTION_a_run_that_walks_and_then_stops_is_not_penalised`
   is the other half of that and is untouched here.
6. **No clause reads the terrain**, inherited and not fixed: the tier says "flat
   ground" and not one assertion checks that it was. This scene builds a level
   slab, so the question is not exercised.
7. **Windows-native, CPU, one machine**, as the T1, T2 and T3 records.
8. **The `t4.json` artifact is this column's own shape**, not a cross-simulator
   contract — the contract is the *dataclasses* in `ladder.graders.t4_evidence`.
   It is ~40 MB for a 300 s run at the task's own `contact_stride: 2` even after
   the per-pair dedup, and the run directory is 49 MB. That is the task config
   honoured rather than a choice made here.
9. **The pose series is downsampled 5:1** (10 ms against a 2 ms step) — five
   times finer than `MAX_SAMPLE_DT_S = 0.05`, but not the one-sample-per-step
   series `REQUIRED_EVIDENCE` asks for.
10. **A measure-zero floating-point tie exists in the gait schedule.** At the
    exact phase-swap instant, `0.5 − 2⁻⁵⁴ + 0.5` rounds to `1.0` and both legs
    can read `stance` for one sample. It is named in a test rather than hidden;
    it costs nothing physically.

---

## 10. Effort ledger (plan §5b rule 6)

The plan makes this mandatory and consequential: *"any column whose scaffolding
effort is below half the OmniSim column's is labelled `under-invested` in the
grid and in every prose sentence that mentions it."* This is the **T4 line**;
[`BRINGUP.md`](BRINGUP.md) §6 carries T1 (≈ 6–7 h), [`BRINGUP_T2.md`](BRINGUP_T2.md)
§8 carries T2 (≈ 5–6 h) and [`BRINGUP_T3.md`](BRINGUP_T3.md) §8 carries T3
(≈ 5–6 h).

| item | this rung, this column |
|---|---|
| **engineer-hours-equivalent** | **≈ 5–6 h** in one session: ~1 h reading the T4 core, `t4_evidence`'s two-cell mechanism, the task file's recorded recipe and the T3 files this mirrors; ~1 h on `t4_scene` (the weld rig and the derived floor length are most of it); ~1.5 h on `t4_drive` (the exact two-link leg IK, the sign correction, the cycle-wide self-check, and the rig); ~1.5 h on `runner_t4` + `t4_channels` — **the constraint-reaction attestation in §6 is the single largest piece of new work on this rung and about half of that**; ~1 h on the 32 tests and this record. |
| **cumulative for the column** | **≈ 21–25 h** (T1 ≈ 6–7 h + T2 ≈ 5–6 h + T3 ≈ 5–6 h + T4 ≈ 5–6 h). |
| **debug iterations to the first graded cell** | **5**, named rather than smoothed away: (1) `MjsJoint.damping` is an `mjtNum[3]` and not a scalar, which `TypeError`s the first build while `armature` beside it accepts a float (§4.1); (2) the IK self-check's own probe point `(x = 0.05, z = 0.44)` is outside the **ankle's** ±0.90 rad limit, so the first run reported `ik_residual: inf` for a gait that was fine — the probe set was replaced with the trajectory the gait actually commands, which is the check that means something; (3) a `⚠` in a `print()` killed a whole weld run under Windows `cp1252` with `UnicodeEncodeError` *after* the verdict had been computed, when stdout was redirected to a file; (4) the single-support test failed at exactly one sample in 400 on a floating-point tie at the phase swap (§9.10) — three minutes were spent looking for a gait bug that was not there; (5) the 2.3× "divergence" in support force from the scratch figures was chased as physics for two runs before it turned out to be the **denominator** — base-body mass versus whole-robot mass (§7), which is now written into the channel's own citation. |
| **compute cost** | **$0.** Local CPU. The canonical run is 22.6 s of wall clock; the whole session's simulated time is about forty minutes. |
| **tokens / agent cells** | **zero.** No agent session was run; none was needed, and none was available. |
| **what is NOT included** | the ladder's T4 core, its channel dataclasses and its negative fixtures — a parallel workstream's, not this column's. This ledger counts only `ladder/adapters/mujoco/`. |

**How to read that honestly.** The same caveat the T1–T3 ledgers carry applies:
MuJoCo genuinely needs less scaffolding than an application-shaped column does —
there is no world file, no controller process, no injection, no port and no log
to parse, and the sampler *is* the stepping loop. The one place that stops being
true on this rung is §6: counting a constraint reaction is real work on any
column, it is work every column must do to be honest here, and it is not cheaper
elsewhere. If the OmniSim column's cumulative ledger comes in at more than
**~42–50 h**, this column is `under-invested` by the plan's own rule and must be
labelled so wherever it appears.
