# MuJoCo-column bring-up record — **T3 `quadruped`**

**2026-08-02.** The T3 half of the MuJoCo column: what it takes to bring the
shipped quadruped description into a scene, whether the walk is **achievable at
all**, and the eight evidence channels T3 needs that no shipped column had.
Companion to [`BRINGUP.md`](BRINGUP.md) (T1) and
[`BRINGUP_T2.md`](BRINGUP_T2.md) (T2), and to
[`docs/developer/capability-ladder-plan.md`](../../../../../docs/developer/capability-ladder-plan.md)
§2 T3.

> **Nothing here is a result.** Every number below comes from a **scripted**
> run — a human wrote the gait knowing the thresholds. A ladder cell is an
> autonomous agent given one sentence and no help (plan §2), and no figure in
> this file may be reported as one. What is proven here is that the *task* is
> achievable, that it is achievable **with no learning of any kind**, and that
> the *instrument* works on this column.

---

## 1. What this settles, and why it had to be settled twice

`tasks/T3_quadruped/meta.json` → `container.authored_here.before_the_freeze`
makes it a precondition of the freeze: *"Demonstrate the walk once, by hand, on
at least one column, and publish the recipe alongside the scaffolding. If it
cannot be demonstrated anywhere, the tier is unachievable-as-shipped and that
is a finding about this file, not about any simulator."*

It was demonstrated on 2026-08-02 — **from a scratch harness outside this
tree**. Under the standing *"no row, no result"* rule
([`agent-edge-validation-plan.md`](../../../../../docs/developer/agent-edge-validation-plan.md)
§0.2) that made the tier's achievability **demonstrated but not reproducible**,
which is the weaker of the two states and which the task file said out loud.
This file is that gap closed: the recipe rebuilt inside the tree, committed,
and run end to end.

```
python tests/benchmarks/ladder/adapters/mujoco/run_t3.py --out <dir>
```

≈ 5 s of wall clock. CPU only. No GPU, no network, no tokens.

Useful variants, each of which exists for a measurement in this document:

```
run_t3.py --out <dir> --skip-run              # re-grade the artifacts on disk
run_t3.py --out <dir> --speed 0.27            # the slow statically-stable walk
run_t3.py --out <dir> --ground plane          # the scratch recipe's floor
run_t3.py --out <dir> --unlimited-actuators   # the scratch recipe's actuators
```

---

## 2. The verdict, verbatim

`T3_quadruped` **PASS 5/5** · `support_attestation: attested` ·
`method: scripted` · `unanswered channels: none` · `vacuous clauses: none` ·
contract checks (`check_bundle` + `check_t3_evidence`) **clean, zero
findings** · `termination: time_limit` — *the clock ran out while it was
moving*.

| # | assertion | measured | threshold |
|---|---|---|---|
| **T3.1** | crossed ten metres in one continuous run | furthest from the start **31.5416 m**; same at the end **31.5416 m**; integrated path **32.7647 m**; largest single inter-sample displacement **0.0075 m** at a **0.01 s** sample interval; clock strictly increasing; the region allowed **45.402 m** of straight run | ≥ 10.0 m · ≤ 0.50 m · increasing |
| **T3.2** | never fell | settled standing height **0.2776 m**; lowest base height **0.2695 m** against a floor of **0.1666 m**; max \|roll\| **0.1091 rad**, max \|pitch\| **0.1213 rad**; combined tilt off vertical **0.1622 rad**; **0** samples over any bound | ≥ 0.60 × standing · ≤ 0.8 rad · every sample |
| **T3.3** | walked; did not slide or drift | mean speed made good over the scored window **0.4287 m/s** (whole recording 0.4506); scored window **23.33 s**; **745** make-and-break ground-contact transitions, **372** completed lift-and-place cycles, across **shank_fl / shank_fr / shank_rl / shank_rr**; 91 498 contacts observed, all naming two distinct bodies; the contact query ran **17 500** times. **Vertical bob 0.00208 m** — *measured, not graded* | ≥ 0.15 m/s · ≥ 8 transitions · *(bob: reported, see §5.2)* |
| **T3.4** | nothing held it up | **0** contacts with any body that is not the ground inside the scored window (45 738 with it); peak applied non-gravitational non-contact force **0.0 N** against a **1.1772 N** limit; peak applied torque **0.0 N·m**; the wrench was live for **0.0 %** of the window; body weight **58.86 N**; `neq = 0` and **0** actuators on the base's free joint | 0 · ≤ 0.02 × m·g · ≤ 2.0 N·m |
| **T3.5** | the run is real | exit **0**, **0** error-class lines (all seven `mjData.warning` physics counters zero), reached finalize on the **engine-authored** `mj_saveLastXML`, driver completed; attribution `mujoco 3.8.1` / solver `Newton` / integrator `implicitfast` / cone `pyramidal` / timestep 0.002 s; **the driver is attested to have loaded**, by path + sha256 + 35 500 actuator-command writes over 35 000 physics steps | 0 · true · attributed · **loaded** |

**`reuse_class: null`.** No grader can decide it (§9 Q3: the reviewer
arbitrates), and §8 forbids quoting a T3 cell without it. It is printed as null
by the tool that prints the cell, with the sentence saying such a cell is not
publishable.

**Reproducibility.** Two independent cold runs of the whole pipeline (build →
phase B → grade) produced **bit-identical** base pose series — sha256
`dfcf012d537dfc9a…` both times — and identical verdicts. Wall clock **4.4 s /
4.6 s** for **70.0 s** of simulated time, i.e. roughly **15× realtime on one
CPU core**, including a per-step contact scan, a 10 ms pose+orientation dump
and a per-step applied-wrench read.

**Machine.** Host `hc385771a14`, AMD64 Family 25 (16 cores), RTX 3060 Laptop
(**unused**: this is CPU `mj_step` throughout), Windows 11, CPython 3.12.9,
`mujoco` **3.8.1** (the official PyPI wheel). Same box as the T1 and T2
records.

---

## 3. The pieces

| file | role |
|---|---|
| `t3_scene.py` | **the shipped URDF → one MJCF scene**, every edit a cited `BuildStep`, plus the ground/light/actuators/**armature** URDF cannot express. Ends by loading the written file **back off disk** and re-checking the armature there, because phase B re-runs the file and "it compiled in memory" is not a deliverable. |
| `t3_drive.py` | the **scripted gait** — the controller half of the deliverable. Zero imports from `ladder`; every length it uses is measured off the compiled model; it verifies its own IK against the compiled forward kinematics on every deployment and records the residual. |
| `runner_t3.py` | **phase B**: load the deliverable's scene, import its driver *by path*, step the loop, write the artifacts. Plus `launch()` (a real subprocess, for a real exit code) and `run_standalone()`. |
| `evidence.py` (+) | `t3_channels()` — the eight channels, mapped into `ladder.graders.t3_evidence`, each with the citation naming the MuJoCo call behind it; `t3_run_standalone()` (see §7.1). |
| `run_t3.py` | end to end in one command: build → phase B → both contract checks → grade through the **real** `ladder.graders.t3`. |
| `test_mujoco_t3.py` | **25 tests**, **14** of which need no simulator at all. Three are tripwires on the findings in §4. |

A T3 deliverable on this column is two files — `scene.xml` and `drive.py` —
because MuJoCo cannot express a controller in a scene. The driver is imported
with `importlib.util.spec_from_file_location` and **never put on `sys.path`**
(asserted by a test): a driver that needed the benchmark's own package tree
would not be a deliverable that stands alone.

---

## 4. Three things bite, and the first one is the most likely blocker anywhere

### 4.1 ⚠ **URDF cannot express rotor inertia, and without it this robot cannot stand**

The single most likely first blocker on any column, and it is a *numerical*
failure that reads exactly like a physics or modelling defect.

A position servo at `kp = 250` on a shank whose own inertia is
`8.5 × 10⁻⁴ kg·m²` has an undamped period shorter than two 2 ms steps. With
`armature` at the importer's default of **0** and an explicit integrator, the
robot **bounces across the floor while being told to hold a fixed standing
pose**:

| configuration | told to STAND STILL for 20 s |
|---|---|
| `armature = 0`, explicit integrator | bounces at ≈ 1 m/s, **drifts 1.816 m**, **flips over inside 3 s**, peak joint tracking error **0.42 rad** (≈ 100 N·m of commanded torque to stand still) |
| `armature = 0.008 kg·m²`, `implicitfast` | base held at **z = 0.2768 m**, **zero drift**, **zero roll**, all four feet down **100 %** of the time |

URDF has **no field for it** — `<dynamics>` carries damping and friction and
nothing else — so no importer of any format could have supplied it, and nothing
warns either way. Lowering `kp` instead also works (`kp = 60` was stable at the
same step) and is the other legitimate fix. **An agent that reads the first
behaviour as *"the robot is badly modelled"* or *"the physics is wrong"* will
fix the wrong thing.** Tripwire:
`test_the_armature_survives_the_round_trip_to_disk`.

### 4.2 ⚠ **The recorded sway amplitude does not walk, and the fix is statics**

The scratch recipe recorded *"sinusoidal lateral body sway ± 0.06 m toward the
support triangle"* at a 2.2 Hz cycle. Re-implemented here, **that number does
not walk.**

At 2.2 Hz a 0.06 m sinusoid demands `A(2πf)² = 11.5 m/s²` of lateral
acceleration against a friction ceiling of `μ·g = 11.8 m/s²` — the feet are
asked for essentially the entire friction budget just to shake the body
sideways. Measured, everything else identical, over 20 s:

| lateral sway | forward travel in 20 s | mean feet on the ground |
|---|---|---|
| **0.060 m** (the recorded number) | **0.18 m** | **1.64** |
| 0.020 m | 8.60 m | 2.71 |
| **0.019 m** (**derived**, see below) | **8.46 m** | **2.67** |
| 0.000 m | 6.05 m | 2.27 |

against the **3** feet the 0.75 duty factor predicts.

**So the amplitude is derived rather than transcribed, and it is derived from
statics.** Lifting any one foot leaves a support triangle whose nearest edge is
the diagonal **through the body centre** — a level body standing on three feet
has a stability margin of exactly **zero**, which is the whole job of the sway.
`t3_drive.sway_amplitude` solves for the lateral shift that puts the body
centre `STABILITY_MARGIN_M` inside that diagonal, using the hip spacing it
**measured** off the model:

```
sway = margin × hypot(hip_x, lateral) / hip_x
     = 0.015 × hypot(0.18, 0.14) / 0.18
     = 0.019 m
```

The margin is stated as a **length in metres** so a reader can attack the
number rather than a fudge factor, and it is **not knife-edge** — which is the
point of publishing the sweep rather than the winner:

| stability margin | derived sway | 70 s result |
|---|---|---|
| 0.010 m | 0.0127 m | **27.90 m**, never fell |
| **0.015 m** (shipped) | **0.0190 m** | **31.53 m**, never fell |
| 0.020 m | 0.0253 m | **35.77 m**, never fell |
| 0.025 m | 0.0317 m | toppled at t = 38.0 s |

Note what that table means for the honesty of the headline number: 0.020 m of
margin gets **closer** to the scratch run's 34.73 m than the shipped 0.015 m
does. The margin was chosen before the distances were compared and was not
moved afterwards. Tests:
`test_the_sway_amplitude_is_derived_from_the_measured_hip_spacing`,
`test_the_recorded_recipes_sway_asks_for_the_whole_friction_budget`.

### 4.3 A plane floor cannot answer T3's arena channel

The scratch recipe's floor was an **infinite plane**. MuJoCo gives a plane geom
an AABB of **±1 × 10¹⁰ m**, so a column that derives the walking region from it
prints, in every row:

```
'longest straight run the region allowed (m)': 14142135623.734
```

True, unreadable, and useless for the one thing the arena channel exists for
(`g1-endurance-2026-08-01.md` §8: six clean runs of this tree's flagship were
ended by a wall and would have been published as a degrading gait). This column
therefore builds the floor as a **finite named box slab**, x ∈ [−5, 45] m,
y ∈ [−6, 6] m, top at z = 0, with the recipe's friction `[1.2, 0.005, 0.0001]`.
The arena then reads **45.402 m** of straight run against the task's stated
minimum of **15.0 m**.

**Measured to be physically irrelevant.** The same gait on the plane and on the
slab produced base positions **identical to five decimal places** (final xy
`31.53734, 0.03026` both times), so the divergence is about what the row can
*say*, not about what the robot did — and a bounded floor is strictly *harder*
than an unbounded one. Reproduce the recipe's choice with `--ground plane`.
Tripwire: `test_a_plane_floor_cannot_answer_the_arena_channel_with_a_readable_number`.

### 4.4 A smaller one: the recipe's unlimited actuators exceed the robot's own limits

The recorded recipe used *"no force limit"* on the twelve position servos.
Measured with the limits off, this gait peaks at **95.37 N·m** on `thigh_fl`
and **78.79 N·m** on `thigh_rr` — against the **30 N·m** those joints' own
`<limit effort>` declares. That would be a demonstration that a robot
*stronger than the one the container ships* can cross ten metres.

Clamped to each joint's declared effort, the same gait covers **31.5416 m
against 31.5231 m** and **every joint stays inside its limit** — the spikes are
transient. The stricter configuration is therefore the default here; the
recipe's is `--unlimited-actuators`. Either way the runner **measures** the
peak actuator force per joint against the declared efforts and `run_t3.py`
prints the comparison under the verdict, so *"did the walk stay inside the
robot's own limits"* is a measurement in the row rather than an assumption in
the build.

---

## 5. What the robot and the tier look like once you actually drive them

### 5.1 The knee limit keeps the leg off its own singularity — and the recorded band was wrong

`meta.json` recorded *"hip-to-foot distance is bounded to 0.040 .. 0.396 m by
the knee's −2.50 .. −0.20 rad range"*. Recomputed from the model's own limits,
the band is **0.126 .. 0.398 m**: `0.040 m` would need a knee of about
**−2.94 rad**, which the description does not allow. The task file has been
corrected and the band is now asserted by a test rather than remembered.

The claim's *substance* survives and is worth keeping: the **−0.20 rad** upper
bound costs only about **2 mm** of extension against the two links' own
0.400 m, so at full stretch the knee bound and the reach bound are practically
coincident — what the bound actually buys is that a solver never returns a
**fully extended** leg, which is numerically fine and physically rigid. At the
other end the knee bound is the only one that bites, and it bites hard: nothing
closer than 0.126 m. The commanded 0.24 m stance sits comfortably inside.

### 5.2 ⚠ **T3.3's vertical-bob conjunct failed this walking robot, and has been retired**

The measurement that got a tier clause changed.

The scratch demonstration already recorded it once: the same crawl driven to
0.27 m/s walks **16.5 m without falling** with **733** contact transitions and
**0.0018 m** of detrended vertical RMS — **red** against the tier's 0.005 m.
Rebuilding it here reproduced the finding and then went further than it. A
`--speed` sweep through the **committed** runner, every run graded through the
real T3 path:

| commanded speed | speed made good | distance in 70 s | transitions | **vertical bob** | old bob clause |
|---|---|---|---|---|---|
| 0.27 m/s | 0.2325 m/s | **16.52 m** | 734 | **0.00117 m** | ❌ red |
| 0.35 m/s | 0.3062 m/s | 22.02 m | 556 | **0.00120 m** | ❌ red |
| 0.50 m/s | 0.4287 m/s | **31.54 m** | 745 | **0.00208 m** | ❌ red |

**Every one of them is below the retired 0.005 m bar.** Under the pre-repair
predicate, this container's own achievability oracle would have been **RED at
every speed tested** — that is, the tier would have read as
*unachievable-as-shipped* on a robot that walked 31 m without falling, with 745
recorded footfall transitions, holding nothing.

A rigid body carried at a constant commanded height does not rise and fall much
however honestly it is walking; the clause was not measuring *whether it
walked*, it was measuring *how dynamic its gait was*. **Decided 2026-08-02**:
ground-contact make/breaks remain the gate for *"it stepped rather than slid"*
and the bob becomes a **reported measurement**.

> **No threshold moved.** `MIN_CONTACT_TRANSITIONS` is still 8,
> `MIN_MEAN_SPEED_MPS` is still 0.15, and `MIN_BOB_RMS_M` is still 0.005 —
> retained, still computed, and printed in every row **beside** the measured
> bob as *the bar this clause applied until 2026-08-02*, together with whether
> that bar would have been met. Any row that passes only because of the change
> says so in its own `detail`, and the row for the run in §2 does. The
> reasoning, the residual risk and both regression fixtures are in
> [`tasks/T3_quadruped/meta.json`](../../tasks/T3_quadruped/meta.json) →
> `grading_readings.T3.3_gait_liveness_is_the_transition_count` and in
> `ladder/graders/t3_core.py` reading 5b. What the repair does **not** close:
> the tier's own *text* still states the bob as a conjunct, so the grader and
> the plan's table diverge until the tier owner ratifies it — declared rather
> than hidden.

### 5.3 The gait is open loop, and that is the whole point of `method: scripted`

Nothing in `t3_drive.py` reads a sensor, a pose or a contact back out of the
simulator. It writes `data.ctrl` and nothing else — never `qpos`, never `qvel`,
and in particular never `xfrc_applied`, which is the array T3.4 measures. So
`support_attestation: attested` with a peak of **0.0 N** is an observation
about a run in which nothing *could* have been applied, and `method: scripted`
is an honest label rather than a modest one. §2 T3 permits training and does
not require it; this reaches the outcome with none.

---

## 6. The eight T3 channels, and where each comes from

`ladder.graders.t3_evidence` lists what T3 needs and the frozen AgentBench
bundle has no field for. All eight are answered on this column; the
`unanswered_channels` measurement on the oracle's verdict is **absent**, which
is what "nothing fell back" looks like.

| channel | MuJoCo source | note |
|---|---|---|
| `base_pose` | `mjData.xpos` **+ `mjData.xmat`**, 10 ms | `xmat` is a world-from-body 3×3 already, so no quaternion or Euler convention is involved. **Without the rotation half of T3.2 cannot be answered at all** — the frozen motion contract carries position and velocity and no rotation |
| `standing_height` | the base's own world z between the settle window's last `mj_step` and the first recorded one | frozen **by construction**: the recorder owns the loop, so nothing can advance the clock during the read |
| `gait_contacts` | `mjData.contact` every physics step, emitted every 2 steps, **with the times the query ran** | 17 500 queries over the run. The times are the channel's whole reason for existing: a transition is a change of state and a list of touches carries no evidence of a *not*-touch |
| `applied_support` | `mjData.xfrc_applied` on the base, per sample, force + torque | see below |
| `arena` | the union world AABB of the bodies the task names as the walking surface | exact, because the floor is a **box** (§4.3) |
| `base_mass` | `mjModel.body_mass` | kilograms, which the neutral inventory has no field for at all |
| `gravity` | `mjModel.opt.gravity` of the model that ran | |
| `controller_load` | the driver's path, **sha256**, whether `setup()` ran, and the number of `control()` writes | a positive attestation, and **not an exit code** — the trap T3.5 exists for is a deploy whose model runtime was missing, which runs a zero-residual baseline and exits 0 |

**Why `xfrc_applied` is the *whole* answer here rather than part of it — the
argument, not the assertion.** In MuJoCo a wrench reaches a body through
exactly four routes: gravity (excluded by the tier), contact (excluded by the
tier, counted separately), an actuator transmission, or `xfrc_applied`. This
scene's actuators are all `mjTRN_JOINT` transmissions on **hinges**, none on
the base's free joint, and `mjModel.neq` is **0**. Both counts are written into
every run, so if either becomes non-zero the attestation is incomplete and the
row can be read as such rather than trusted.

**`other_is_ground` is structural and its limit is stated.** The sampler
answers it as *"the other body is welded to the world and is not in the robot's
kinematic subtree"* — which is **true of a static wall too**. That is exactly
why the task's own name list is authoritative and why
`t3_evidence.classify_contacts` takes the stricter of the two readings on every
disagreement: an adapter free to decide what counts as the ground could label
the thing holding the robot up as the floor and pass T3.4 outright.

---

## 7. Known gaps and caveats (state, don't bury)

1. **⚠ `ladder.graders.t3.run_and_grade` reaches the WRONG sampler on this
   column.** It looks for a `run_standalone` hook, like its two siblings do,
   and on this column that name is **T2's**. So
   `python -m ladder.graders.t3 <deliverable> --sim mujoco` would run the T2
   sampler and record none of T3's channels. The column exposes
   `t3_run_standalone` so the fix is one line — *prefer it where a column has
   one* — but `ladder/graders/t3.py` is outside this work's scope, so the gap
   is named here, in `evidence.run_standalone`'s docstring, and in a test
   (`test_the_t3_phase_b_hook_is_a_separate_name_from_t2s_on_purpose`) that
   **skips itself** once `t3.py` is fixed. `run_t3.py` calls the T3 runner
   directly and is unaffected.
2. **This is one column.** The **OmniSim** T3 channels are unimplemented, so
   every T3 assertion there remains `scaffolding_defect_ours`. What it needs is
   the same eight channels out of the harness/supervisor surface plus a `.wbt`
   scene: `base_pose` needs a per-step pose **with orientation** for the base
   (the ladder recorder already records robots, but not their rotation), the
   standing height and the arena are `GET /scene/tree?bounds=1` reads,
   `gait_contacts` is `/sim/contacts` sampled across the walk *with the query
   times*, mass and gravity are supervisor field reads, and
   `applied_support` is the one genuinely new build — the deploy hook can print
   the wrench it is already applying. The plan's own prediction for the OmniSim
   T3 **cell** (`achieved, reuse_class: assembled`) is untouched by anything
   here and must not be read as confirmed *or* softened: this run says the task
   is achievable *on MuJoCo, by a human*, and says nothing whatever about
   OmniSim or about any agent.
3. **The oracle is a scripted control and its verdict is not a cell.** It is
   also not a *baseline* an agent must beat — it is an existence proof.
4. **The reproduction is 9 % short of the scratch run** (31.54 m against
   34.73 m; 0.4287 m/s made good against 0.4867). Reported rather than tuned
   away. The recipe was prose and three parts of it were under-determined
   (§4.2, §4.3, §4.4); the sway margin in particular was fixed on a static
   argument before the distances were compared, and a different defensible
   margin gets closer to the scratch number than the shipped one does. Every
   other clause reproduces, and the one measurement that did **not** —
   the vertical bob, 0.00208 m here against 0.00887 m there — makes the case
   for the T3.3 repair stronger rather than weaker (§5.2).
5. **One gait, one layout, one flat floor.** Achievability is proven for *a*
   gait on level ground. Nothing here says anything about terrain — which is
   also T3's own recorded open question (`meta.json` →
   `grading_readings.T3_OPEN_QUESTION_no_clause_reads_the_terrain`: the tier
   says "flat ground" and not one assertion checks that it was).
6. **Windows-native, CPU, one machine**, as the T1 and T2 records.
7. **The `t3.json` artifact is this column's own shape**, not a
   cross-simulator contract. The cross-simulator contract is the *dataclasses*
   in `ladder.graders.t3_evidence`; another column may write whatever it likes
   as long as its `t3_channels` produces those. `t3.json` is ~9 MB and
   `contacts.json` ~6 MB for a 70 s run at the task's own `contact_stride: 2`;
   that is the task config honoured rather than a choice made here.
8. **The pose series is downsampled 5:1** (10 ms against a 2 ms step). Five
   times finer than the tier's own `MAX_SAMPLE_DT_S = 0.05` witness bound, but
   it is not the one-sample-per-physics-step series `REQUIRED_EVIDENCE` asks
   for, and the shared T1 note about downsampling under-stating *path length*
   rides along in the row.

---

## 8. Effort ledger (plan §5b rule 6)

The plan makes this mandatory and consequential: *"any column whose scaffolding
effort is below half the OmniSim column's is labelled `under-invested` in the
grid and in every prose sentence that mentions it."* This is the **T3 line**;
[`BRINGUP.md`](BRINGUP.md) §6 carries T1 (≈ 6–7 h) and
[`BRINGUP_T2.md`](BRINGUP_T2.md) §8 carries T2 (≈ 5–6 h).

| item | this rung, this column |
|---|---|
| **engineer-hours-equivalent** | **≈ 5–6 h** in one session: ~1 h reading the T3 grader core, the eight-channel contract and the negative fixtures before writing anything; ~1 h on `t3_scene` (the armature finding is most of it); ~1.5 h on `t3_drive` (the exact 3-DOF leg IK including the thigh offset, the four-beat schedule, and the sway sweep after the recorded amplitude would not walk); ~1 h on `runner_t3` + `t3_channels` (the neutral mapping and the applied-wrench argument); ~1 h on the 25 tests and this record. |
| **cumulative for the column** | **≈ 16–19 h** (T1 ≈ 6–7 h + T2 ≈ 5–6 h + T3 ≈ 5–6 h). |
| **debug iterations to the first graded cell** | **5**, named rather than smoothed away: (1) the recorded ±0.06 m sway, which crabbed the robot off the side of the slab and free-fell it to z = −9360 m — three runs were spent blaming the gait phase before the mean contact count was printed and read **1.55** against the duty factor's 3; (2) the slab was ±4 m wide in y, so a lateral drift that a *wider* floor tolerates read as a fall — widened to ±6 m, because "it walked off the edge" and "it fell over" are different findings; (3) the plane floor's ±1e10 AABB, which made T3.1's row unreadable; (4) `describe()` reporting `ik_residual_m: null` because `_S.update(st)` copied the keys that existed at that instant and the self-check was written to `st` afterwards — a self-check that reached the caller and not the run record; (5) the recorded 0.040 m knee band, which no knee limit in the shipped description can produce. |
| **compute cost** | **$0.** Local CPU. Every run in this record is ≈ 5 s of wall clock; the whole session's simulated time is under fifteen minutes. |
| **tokens / agent cells** | **zero.** No agent session was run; none was needed, and none was available. |
| **what is NOT included** | the ladder's T3 core, its channel dataclasses and its negative fixtures — a parallel workstream's, not this column's. This ledger counts only `ladder/adapters/mujoco/`. |

**How to read that honestly.** The same caveat the T1 and T2 ledgers carry
applies here: MuJoCo genuinely needs less scaffolding than an
application-shaped column does — there is no world file, no controller process,
no injection, no port and no log to parse, and the sampler *is* the stepping
loop, so six of the eight channels are a single array read. If the OmniSim
column's cumulative ledger comes in at more than ~32–38 h, this column is
`under-invested` by the plan's own rule and must be labelled so wherever it
appears.
