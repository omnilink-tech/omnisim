# MuJoCo-column bring-up record — **T2 `transfer`**

**2026-08-02.** The T2 half of the MuJoCo column: what it takes to bring the
three shipped descriptions into a scene, whether the transfer is **achievable
at all**, and the eight evidence channels T2 needs that no shipped column had.
Companion to [`BRINGUP.md`](BRINGUP.md) (the T1 record, whose §5 gap 8 said
*"Scope. T1 only. Nothing here addresses T2"* — this is that gap closed) and to
[`docs/developer/capability-ladder-plan.md`](../../../../../docs/developer/capability-ladder-plan.md)
§2 T2.

> **Nothing here is a result.** Every number below comes from a **scripted**
> run — a human wrote the control law knowing the thresholds. A ladder cell is
> an autonomous agent given one sentence and no help (plan §2), and no figure in
> this file may be reported as one. What is proven here is that the *task* is
> achievable and that the *instrument* works on this column.

---

## 1. What this settles, and why it had to be settled first

[`tasks/T2_transfer/container/PROVENANCE.txt`](../../tasks/T2_transfer/container/PROVENANCE.txt),
shipped with the task, in its own words:

> The arm has never been demonstrated completing this task on any simulator. It
> is a description with plausible masses, inertias, limits and efforts, and the
> geometry has been checked for reach and for gripper travel against the block —
> but *"the numbers are sane"* is not *"the task is achievable"*, and if it
> turns out to be unachievable everywhere then the grid measured the asset and
> not the agents.

`meta.json` → `container.authored_here.before_the_freeze` makes settling it a
precondition of the freeze: *"Demonstrate the transfer once, by hand, on at
least one column, and publish the recipe alongside the scaffolding."*

**It is achievable.** A scripted driver picks the block off the table, carries
it, holds it through a 90° yaw, lowers it into the bin and lets go — and the
run grades **PASS 5/5 through the real T2 path**, with **no clause vacuous** and
**no channel unanswered**.

```
python tests/benchmarks/ladder/adapters/mujoco/run_t2.py --out <dir>
```

Equivalently, through the documented grader CLI on the deliverable it leaves
behind:

```
python -m ladder.graders.t2 <dir>/deliverable --sim mujoco --run-dir <dir2>
```

---

## 2. The verdict, verbatim

`T2_transfer` **PASS 5/5** · `hold_mechanism: friction` · `unanswered
channels: none` · `vacuous clauses: none` · contract checks (`check_bundle` +
`check_t2_evidence`) **clean, zero findings**.

| # | assertion | measured | threshold |
|---|---|---|---|
| **T2.1** | ended inside the container, at rest | centre `(-0.000, 0.4411, 0.0449)` m, **inside** the bin's AABB; lowest point **0.0199 m** against a **measured** rim of **0.160 m**; mean speed over the last 1.0 s **0.00000 m/s** (1.000 s of samples in the window) | inside · below the rim · < 0.02 m/s |
| **T2.2** | lifted, not dragged | longest continuous window clear by ≥ 0.05 m: **15.08 s** (t = 5.48 → 20.56); greatest clearance **0.5251 m**; surfaces beneath it at some point `[ground, table, bin, bin_wall_x_pos]`; **7** static bounded surfaces; **0** samples with nothing beneath the object | ≥ 0.05 m for ≥ 3.0 s |
| **T2.3** | the grip held it for ten seconds | a 10.0 s window exists; longest run of satisfying windows **19.42 s** (t = 2.95 → 22.37); **1851** windows tested; RMS **0.00481 m**, max excursion **0.03993 m**; clock skew between the two series **0.0 s** | ≥ 10.0 s · RMS ≤ 0.02 m · max ≤ 0.04 m · **in the end effector's frame** |
| **T2.4** | the object is a real body under gravity | mass **0.2 kg**, mass model attached, gravity **9.81 m/s²**, largest inter-sample displacement **0.0041 m** at a **0.01 s** sample interval | > 0 · dynamic · > 0 · ≤ 0.50 m |
| **T2.5** | the run is real | exit **0**, **0** error-class lines (all seven `mjData.warning` physics counters zero), reached finalize on the **engine-authored** `mj_saveLastXML`, driver completed, attribution `mujoco 3.8.1` / solver `Newton` / integrator `implicitfast` / cone `elliptic` | 0 · true · attributed |

**Read T2.3's `max 0.03993` correctly — it is not a near miss.** The core
reports the numbers for the **first** window of the longest satisfying run, and
the first window is marginal by construction (it straddles the descent onto the
block). The interior windows are two orders of magnitude better: measured
window by window, RMS runs **0.00098 – 0.00571 m** and max excursion
**0.00123 – 0.01586 m** from t = 4 s to t = 12 s. The grip does not slip; the
object's position in the tool's own frame is `z = 0.0979 m` at t = 8 s and
`z = 0.0979 m` at t = 18 s, against a commanded grasp offset of 0.098 m.

**Reproducibility.** Two independent cold runs of the whole pipeline (build →
phase B → grade) produced **bit-identical** object trajectories
(sha256 of the recorded series `dc4712113d12add0…` both times) and identical
verdicts. Wall clock **2.38 s / 2.45 s** for **28.5 s** of simulated time —
roughly **12× realtime on one CPU core**, including a per-step contact scan and
a 10 ms pose+orientation dump.

**Machine.** `9722d23d12a3` — host `hc385771a14`, AMD64 Family 25 (16 cores),
RTX 3060 Laptop (**unused**: this is CPU `mj_step` throughout), Windows 11,
CPython 3.12.9, `mujoco` 3.8.1. Same box as the T1 record.

---

## 3. The pieces

| file | role |
|---|---|
| `t2_scene.py` | **three shipped URDFs → one MJCF scene**, every edit a cited `BuildStep`, plus the ground/table/light/actuators URDF cannot express. Ends by loading the written file **back off disk**, because phase B re-runs the file and "it compiled in memory" is not a deliverable. |
| `t2_drive.py` | the **scripted driver** — the controller half of the deliverable. Zero imports from `ladder`; every length it uses is measured off the compiled model. |
| `runner_t2.py` | **phase B**: load the deliverable's scene, import its driver *by path*, step the loop, write the artifacts. Plus `launch()` (a real subprocess, for a real exit code) and `run_standalone()` (the hook `ladder.graders.t2.run_and_grade` calls). |
| `evidence.py` (+) | `t2_channels()` — the eight channels, mapped into `ladder.graders.ladder_evidence`, each with the citation naming the MuJoCo call behind it. `run_standalone()` re-exported so the column is reachable through the grader's own CLI. |
| `run_t2.py` | end to end in one command: build → phase B → both contract checks → grade through the **real** `ladder.graders.t2`. |
| `test_mujoco_t2.py` | **41 tests**, **31** of which need no simulator at all. Four are tripwires on the findings in §4. |

A T2 deliverable on this column is two files — `scene.xml` and `drive.py` —
because MuJoCo cannot express a controller in a scene. The driver is imported
with `importlib.util.spec_from_file_location` and **never put on `sys.path`**
(asserted by a test): a driver that needed the benchmark's own package tree
would not be a deliverable that stands alone.

---

## 4. Four things bite, and three of them are silent

### 4.1 With MuJoCo's URDF defaults, **none of the three names survives**

`compiler/fusestatic` defaults to **`true` for URDF**, so a link with no joint
is *absorbed into the world body*. Measured on 3.8.1:

| description | compiled with the defaults |
|---|---|
| `block.urdf` | **0 bodies** — `mj_name2id(…, "block")` returns −1, the cube is world geometry |
| `bin.urdf` | **0 bodies** — same, all five geoms become world geometry |
| `bench_arm.urdf` | 8 bodies, but **no `base`**: the pedestal is absorbed |

T2 resolves its three roles **by name** (`meta.json` → `roles`), so a scene
built on the defaults is one the grader must *refuse* — and refuse is the right
behaviour, since grading whatever body happened to be nearest is exactly what
`roles.why_not_in_the_container` forbids. `fusestatic="false"` is not optional
here. Tripwire: `test_the_urdf_defaults_lose_the_object_and_the_container`.

### 4.2 The object needs a joint to the world, or it is scenery

Same rule from the other side. `block.urdf` is a 0.2 kg cube with no joint, and
MuJoCo will happily weld it to the world for ever — T2.4 would read *"a mass
model is attached"* and the object would still be immovable. It gets URDF's own
`<joint type="floating">` from a dummy `world` link, the same in-format fix the
T1 builder uses on the Husky chassis.

### 4.3 ⚠ **Anchoring the arm's pedestal jams its first axis**

The loudest finding, and it is a *contact-filter* effect rather than a geometry
error.

The shipped arm's `shoulder` cylinder spans z ∈ [0.14, 0.26] and its `base`
pedestal spans z ∈ [0, 0.15]: **they overlap by 10 mm by design**, which is
ordinary URDF practice because a simulator filters contacts between a body and
its own parent. MuJoCo does too — *except* that its filter is written in terms
of **weld groups** and deliberately does not apply *"if the parent is the world
body"*. Weld the pedestal down and it joins the world's weld group, the parent
exemption fires, and the pedestal starts colliding with the shoulder bolted to
it.

Measured, commanding `joint_1` to 1.5 rad through a 3000 N·m/rad servo limited
to the URDF's declared 150 N·m:

| pedestal | `joint_1` after 3 s | actuator force |
|---|---|---|
| **welded** (no joint) | **0.079 rad** — 5 % of the command | **saturated at 150 N·m** |
| **floating** (40 kg on the ground) | **> 1.4 rad** | tracking |

In the full 28.5 s oracle run with a welded pedestal, `joint_1` moved
**0.02 rad** and the arm simply never turned toward the bin. **Nothing warns.**
The scene therefore gives the pedestal a floating joint and lets its 40 kg hold
it down — which is what `meta.json` → `container.no_fixed_base_convention` says
the pedestal is *for*. An `<exclude>` pair between `base` and `shoulder` is the
other legitimate fix and would also work. Tripwires:
`test_a_welded_pedestal_jams_the_arms_first_axis` and its passing twin.

### 4.4 A friction pinch needs the friction model configured

With MuJoCo's defaults (pyramidal cone, `impratio` 1, no noslip pass) the block
**creeps down through the jaws at ≈ 3.5 mm/s** — **42 mm over a 12 s carry**,
which is a T2.3 failure by a factor of two on a grip that is nowhere near
slipping physically (30 N of squeeze per pad against a 2 N object, μ = 1). It is
the tangential-softness artifact MuJoCo's own documentation points at, and its
own documented answers remove it completely:

| setting | creep over the same 12 s carry |
|---|---|
| defaults | **42 mm** (T2.3 red) |
| `cone="elliptic"`, `impratio=20`, `noslip_iterations=5` | **0.0 mm** (T2.3 green, 0.001 m RMS) |

These are **solver settings, not grasp tuning**: no contact parameter, geometry,
mass or commanded width was touched. They are also exactly the shape of
undiscoverable pin the plan predicts an agent will miss — §2 T2 says the same
thing about OmniSim's own `newtonSolver "mujoco"` requirement — so they are
recorded as `BuildStep`s in every `build.json` rather than buried in a default.

### 4.5 A fifth, smaller one: MuJoCo will not reload its own serialised scene

`MjSpec.attach` brings each child's unnamed root `<default>` along, and
`to_xml()` writes them out as nested `<default/>` elements with no `class`
attribute — which MuJoCo's **own parser then rejects** with `XML Error: empty
class name`. The scene compiles in memory and will not load from disk. An agent
that assembled a scene this way and handed over the file would ship a
deliverable that does not open, and nothing warns at write time. `t2_scene`
strips the empty elements (`sanitise_mjcf`) and then **loads the file back** to
prove the deliverable stands alone.

---

## 5. What the arm and the tier look like once you actually drive them

**Wrist range binds long before reach does.** The container advertises *"about
0.9 m of reach from a shoulder at z = 0.25"*. That is true and it is not the
constraint. Holding the gripper **vertical** requires `joint_2 + joint_3 +
joint_5 = π`, and `joint_5` is limited to ±2.0 rad, so a straight-down grasp
needs `joint_2 + joint_3 ≥ 1.14 rad` — the arm must be *folded*. At the
pre-grasp height this driver uses, radius **0.50 m has no solution** while
sitting comfortably inside the 0.73 m the two long links span; **0.44 m** has
0.07 rad of joint 5 in hand. The usable straight-down workspace is a good deal
smaller than the advertised reach. Not a defect — but an agent that places the
block at 0.6 m and reasons from "reach is 0.9 m" will fail for a reason it
cannot see. Test: `test_the_wrist_range_binds_before_reach_does`.

**The pads are longer than the object.** The fingers' inner faces span 60 mm and
the cube is 50 mm, so a centred grasp always puts 5 mm of pad below the object —
into whatever it is standing on. The driver therefore grasps 13 mm high
(`PAD_CLEARANCE_M`), giving 35 mm of pad-on-block contact and 10 mm of
fingertip clearance over the table. Anyone reproducing this will meet the same
constraint.

**⚠ T2.3 as written is satisfied by "everything stopped".** The tier asks
whether *some* continuous 10 s window exists in which the object's position in
the end effector's frame varies by ≤ 0.02 m RMS. Demonstrated on this run's own
final state — block resting on the bin floor, jaws open, arm parked 0.6 m away,
nothing in contact — a 12 s window scores **RMS 4.0 × 10⁻⁸ m, max excursion
3.8 × 10⁻¹⁵ m** and **satisfies T2.3 outright**. Nothing was held. The oracle
does not exploit this (its `stand` phase is 4.0 s, so the only 10 s window in
the run is the carry, t = 2.95 → 22.37), but a cell that ended with a long quiet
tail would pass the clause without ever gripping anything.

*No threshold was changed and none should be changed on this account.* Three
repairs were available and each is one line, and the choice belonged to whoever
owns the tier before the freeze: (a) require the window to overlap the
`≥ 0.05 m` clearance window T2.2 already computes — *held* implies *off the
surface*; (b) require the grip channel to report a contact naming the carrier
and the object inside the window — the channel is already recorded in every
cell, and this would be the first thing to *grade* it, which §2 T2 forbids for
the mechanism but not for its existence; (c) require the object to have moved
at all during the window. (a) is the cheapest and needs no new evidence.

> **DECIDED, same day (2026-08-02): (a) OR (c), and neither on its own.** A
> candidate window must now contain at least one sample at which the object was
> *being carried* — **clear of everything beneath it by ≥ `LIFT_CLEARANCE_M`**
> (gravity is acting, so an object off the surface is held up by *something*)
> **or moving at ≥ `AT_REST_SPEED_MPS`** (an object that moves while keeping a
> fixed offset to the carrier is moving *with* it). No threshold moved; both
> gate numbers are the tier's own. (a) alone was measured and rejected: it
> reddens T2.3 on any run that was never lifted, which makes a *drag* — an
> object genuinely gripped the whole way — fail the grip clause, leaves T2.2
> with no isolating negative fixture, and makes a 10 s continuous clearance
> imply T2.2's 3 s one. (c) alone fails the most obvious correct answer to the
> prompt: lift it and stand still. The reasoning, the residual limit and the
> regression fixture (`at_rest_never_held`) are in
> [`tasks/T2_transfer/meta.json`](../../tasks/T2_transfer/meta.json) →
> `grading_readings.T2.3_carry_gate` and in `ladder/graders/t2_core.py`
> reading 4b. **This run re-grades PASS 5/5 from the same artifacts** with no
> re-simulation: T2.3 unchanged at a **19.42 s** run of satisfying windows,
> RMS **0.00481 m**, max **0.03993 m**, **814 of the 1001 samples** in the
> reported window off the surface or moving, and the pre-repair reading
> (*"a window inside both bounds in which the object was at rest on a surface
> throughout"*) **false**.

**⚠ A note the T1 path emits into every T2 row.** The bundle's trajectory is
recorded at one sample per **5** physics steps (10 ms, five times finer than the
tier's own `MAX_SAMPLE_DT_S = 0.05` witness bound), and the shared evidence
builder attaches its T1 warning about downsampling under-stating *path length*.
Path length is a T1.3 quantity and no T2 assertion reads it, so the note is
harmless here — but it is in the row and a reader should know why.

---

## 6. The eight T2 channels, and where each comes from

`ladder.graders.ladder_evidence` gap 3 lists what T2 needs and the frozen
AgentBench bundle has no field for. All eight are answered on this column; the
`unanswered_channels` measurement on the oracle's verdict is **absent**, which
is what "nothing fell back" looks like.

| channel | MuJoCo source | note |
|---|---|---|
| `object_pose` | `mjData.xpos` for the named body, 10 ms | the object is a plain body; nothing that enumerates *robots* would have recorded it |
| `end_effector` | `mjData.xpos` **+ `mjData.xmat`** | `xmat` is already a world-from-body 3×3, so no quaternion/Euler convention is involved. Both series come from **one loop on one clock** — the core's skew guard reads **0.0 s** |
| `container_geometry` | union of the named body's **kinematic subtree** geom AABBs, + a **measured rim** | the bin's four walls are child bodies, so the body's own box would be its floor slab (top z = 0.02) and containment would fail on a correctly delivered object |
| `support_surfaces` | every body with `body_weldid == 0`, not in the arm's subtree, carrying geometry | **structural, not a name list**: a surface an agent named something unexpected is still found, and a body that can move is still excluded. Found 7 on the oracle scene |
| `object_mass` | `mjModel.body_mass` | kilograms, which the neutral inventory has no field for at all |
| `gravity` | `mjModel.opt.gravity` of the model that ran | |
| `object_aabb` | the same world-AABB walk at t = 0 | turns a centre into a lowest point |
| `grip` | contacts naming an arm body and the object, timed on the pose clock, **+ `mjModel.neq` and the equality table** | this is what makes `hold_mechanism: friction` an **observation**: `neq == 0`, so no constraint binds the object and the only thing holding it is contact |

**The rim rule, stated so it can be attacked.** `ContainerGeometry.rim_z` is the
one number T2.1 needs that no simulator has a field for. Here it is *measured*:
the rim is the highest point of any geom in the container's subtree **whose
horizontal footprint does not cover the subtree's own horizontal centre**. An
open box's walls stand clear of the centre line and its floor does not, so the
walls decide the rim; a lid or a raised handle sits *over* the centre and is
excluded, which is the whole reason the rim is not just the top of the bounding
box. Verified on three synthetic containers: an open box (rim = wall top 0.16,
= box top), a **lidded** box (box top **0.18**, rim **0.16** — the lid is
correctly ignored), and a solid slab (**no rim**, so the core falls back to the
permissive box top and marks the clause's witness absent). Limits stated in the
`rim_rule` string that is printed inside every row: a container whose wall
geometry is one revolved shell covering its own centre reports `None`, and the
footprints are axis-aligned world boxes, so a container tilted off the world
axes is over-estimated.

**One shim change was needed.** `ladder.adapters.build_t2_evidence` passed only
`phase_b` to a column's `t2_channels` hook, so grading an **existing run
directory** (a re-grade, a fixture, somebody else's run) handed the hook `None`
and reported every channel unanswered on a run that had all of them on disk. It
now falls back to `run_dir`, mirroring what `build_bundle` already does with its
own run-directory candidates. Additive; no other column has a `t2_channels` hook
today.

---

## 7. Known gaps and caveats (state, don't bury)

1. **This is one column.** The **OmniSim** T2 channels are still unimplemented,
   so every T2 assertion there remains `scaffolding_defect_ours`. What it needs
   is the same eight channels out of the harness/supervisor surface plus a `.wbt`
   scene: `object_pose`/`end_effector` need a per-step pose **with orientation**
   for a named non-robot body (the ladder recorder records *robots*), the
   container box and the static surfaces are `GET /scene/tree?bounds=1`, the
   rim needs the same geom rule over the container's subtree, mass and gravity
   are supervisor field reads, and `grip` is `/sim/grips` + `/sim/contacts`. The
   plan's own prediction for the OmniSim T2 **cell** (`not_achieved`) is
   untouched by anything here and must not be read as softened: this run says
   the task is achievable *on MuJoCo, by a human*, and says nothing whatever
   about OmniSim or about any agent.
2. **The oracle is a scripted control and its verdict is not a cell.** It is
   also not a *baseline* an agent must beat — it is an existence proof.
3. **The scene is one layout.** Everything at radius 0.44 m, bin 90° round from
   the block. Achievability is proven for *a* placement, not for every placement
   an agent might choose; §5's wrist-range finding says directly that some
   placements have no straight-down solution.
4. **`hold_mechanism` is `friction` here.** It is recorded and never graded
   (§2 T2), and this run's mechanism is an observation (`neq == 0`), not a
   declaration. A column that used an attachment would still pass, by design.
5. **Windows-native, CPU, one machine**, as the T1 record. `mujoco-mjx` is
   installed and unused; `jax.devices()` is CPU-only on this box and it does
   not matter for a single arm.
6. **The `t2.json` artifact is this column's own shape**, not a cross-simulator
   contract. The cross-simulator contract is the *dataclasses* in
   `ladder.graders.ladder_evidence`; another column may write whatever it likes
   as long as its `t2_channels` produces those.
7. **The pose series is downsampled 5:1** (see §5). Every T2 witness bound is
   satisfied with margin, but it is not the one-sample-per-physics-step series
   `REQUIRED_EVIDENCE` asks for, and the row says so.

---

## 8. Effort ledger (plan §5b rule 6)

The plan makes this mandatory and consequential: *"any column whose scaffolding
effort is below half the OmniSim column's is labelled `under-invested` in the
grid and in every prose sentence that mentions it."* This is the **T2 line**;
[`BRINGUP.md`](BRINGUP.md) §6 carries the T1 line (≈ 6–7 h).

| item | this rung, this column |
|---|---|
| **engineer-hours-equivalent** | **≈ 5–6 h** in one session: ~1 h reading the T2 grader core, the channel contract and the negative fixtures before writing anything; ~1.5 h on `t2_scene` (the four findings in §4 are most of it — each was a failing run before it was a paragraph); ~1 h on `t2_drive` (the closed-form IK, the measured geometry, the integral term); ~1 h on `runner_t2` + `t2_channels` (the neutral mapping); ~1 h on the 41 tests and this record. |
| **cumulative for the column** | **≈ 11–13 h** (T1 ≈ 6–7 h + T2 ≈ 5–6 h). |
| **debug iterations to the first graded cell** | **6**, named rather than smoothed away: (1) `fusestatic` swallowing the block and the bin, so the scene had no `block` body at all; (2) the welded pedestal jamming `joint_1` — three wasted runs blamed on the control law before the contact list was printed; (3) `MjSpec.attach`'s default name prefix producing `/block`, `/tool`, `/bin`, which the grader correctly refuses; (4) the empty-`<default/>` round-trip defect, which made phase B exit 7 on a scene that had just compiled; (5) friction creep — the first working carry lost 42 mm and failed T2.3; (6) the mass audit comparing the whole scene (3 724 kg, mostly a 6 m floor slab at MuJoCo's default geom density) against the three descriptions' 53.9 kg. |
| **compute cost** | **$0.** Local CPU. Every run in this record is 2.4 s of wall clock; the whole session's simulated time is under two minutes. |
| **tokens / agent cells** | **zero.** No Claude session was run; none was needed. |
| **what is NOT included** | the ladder's T2 core, its channel dataclasses and its negative fixtures — a parallel workstream's, not this column's. This ledger counts only `ladder/adapters/mujoco/`. |

**How to read that honestly.** The same caveat the T1 ledger carries applies
here and applies harder: MuJoCo genuinely needs less scaffolding for T2 than an
application-shaped column does — there is no world file, no controller process,
no injection, no port and no log to parse, and the sampler *is* the stepping
loop, so five of the eight channels are a single array read. If the OmniSim
column's T2 ledger comes in at more than ~22 h cumulative, this column is
`under-invested` by the plan's own rule and must be labelled so wherever it
appears.
