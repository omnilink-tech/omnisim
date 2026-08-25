# OmniSim column — bring-up record and effort ledger

The roster entry for `capability-ladder-plan.md` §4, and the column's half of
§5b rule 6 (the effort-parity ledger). The MuJoCo column keeps one file per
rung (`../mujoco/BRINGUP.md`, `_T2`, `_T3`, `_T4`); this column keeps one file
because T2, T3 and T4 were built together, out of one scan, in one session.

Nothing here is a cell. Everything here was measured with **scripted probe
scenes written by a human who knew the thresholds**, which is explicitly *not*
what `capability-ladder-plan.md` §2 means by a cell. What these runs establish
is that the **instrument** works: that a real agent's deliverable, re-run cold
and standalone, produces a verdict whose reds are the agent's.

---

## 1. What this column is

`omnisim` is the **product clone** column: the workspace is this repository,
staged by `ladder/cell/stage_ladder_workspace.py` with the ladder's own
quarantine on top of `cc_lane`'s. The deliverable convention is a `.wbt` scene
the session authored (`ladder/cell/run_ladder_cell.py` → `DELIVERABLE_RULES`).

Two files carry the column:

| file | what it is |
|---|---|
| [`evidence.py`](evidence.py) | the launcher and the hooks. Reuses `agentbench.adapters.omnisim.headless` for the process, `agentbench.common.paths.engine_launch` for the binary/env/newton-sidecar, and `agentbench.adapters.omnisim.evidence.build_bundle` for the neutral bundle. Nothing is forked. |
| [`channels.py`](channels.py) | **pure.** One tier document → the neutral channel dataclasses. No engine, no file, no socket, so the whole T2–T4 contract is exercised in a third of a second. |

and one controller:

| file | what it is |
|---|---|
| [`../../controllers/ladder_recorder/ladder_recorder.py`](../../controllers/ladder_recorder/ladder_recorder.py) | the grader-owned sampler. T1's pose + support-contact channels (pre-existing) plus, from this session, the tier-channel document every named body's pose **and orientation**, the frozen t=0 inventory with world AABBs and masses, the full contact record with both sides named, gravity, the per-robot controller attestation, and the structural support probe. |

**One document, three rungs.** T2, T3 and T4 read the same
`<out>.channels.json`. That is a property of this column, not a shortcut: a
Supervisor sees the whole scene generically, so *"every named body's pose"*
already contains T2's object, T2's end effector and T3/T4's base. The
cross-simulator contract is the dataclasses, never an artifact shape —
`../mujoco/BRINGUP_T2.md` §7.6 says so in as many words.

---

## 2. The eight T2 channels, and where each comes from

| channel | OmniSim source | note |
|---|---|---|
| `object_pose` | `wb_supervisor_node_get_position` for the named body, every ≤ 0.04 s | the object is a plain body; the T1 sampler enumerates *robots* and would never have recorded it |
| `end_effector` | the same **plus `wb_supervisor_node_get_orientation`** | `getOrientation` returns a world-from-body 3×3 directly, so no quaternion or Euler convention is involved. Both series come from **one loop on one clock** — measured skew on the T2 probe: **0.0 s** |
| `container_geometry` | `geometry.bounds_for_subtree` over the container's whole subtree, with the rim taken as the top of that union | the bin's four walls are child links, so the body's own box would be its floor slab. Measured rim on the probe: **0.2094 m** |
| `support_surfaces` | every body carrying **no `Physics` node** that is not itself a robot and has a world box | structural, not a name list: a surface the agent named something unexpected is still found, and a body that can move is still excluded |
| `object_mass` | the `Physics` node's own `mass` field, in kilograms, with the subtree sum beside it | measured **0.2 kg** for the shipped block. ⚠ a *density*-based Physics node reports `null` with the reason — see §5 |
| `gravity` | `WorldInfo.gravity` | read in **both** declared shapes (SFFloat here, SFVec3f upstream) — see §5 |
| `object_aabb` | the same t=0 world-AABB walk | turns a centre into a lowest point |
| `grip` | contacts naming the object and a body in the carrier's robot subtree, **plus** whether any `Connector` exists in a tracked subtree | this is what makes `hold_mechanism` an **observation**: a `Connector` is the only run-time constraint in this engine that can bind two bodies, so its absence plus holding contacts *is* friction |

## 3. The eight T3/T4 channels

| channel | OmniSim source |
|---|---|
| `base_pose` | position **and orientation** for the base, every ≤ 0.04 s on one clock |
| `standing_height` | the base's z at the first recorded sample — i.e. after the tier's own settle window, so it is the height it came to rest at |
| `gait_contacts` | the harness supervisor's paired contact query, every `contact_stride` recorded steps, with **both sides named**, the robot-side *part* resolved through the subtree index, and **the times the query ran** |
| `applied_support` | ⚠ **a structural route enumeration, not a wrench read-back — see §4** |
| `arena` | the union of the world AABBs of the bodies the task named as the walking surface, with the static bodies of ≥ 0.20 m vertical extent named as `boundary_bodies`. Measured on the T3 probe: **28.359 m** of longest straight run |
| `base_mass` | as `object_mass`, and reported as the base body's **own** mass with the subtree sum in the citation, matching the MuJoCo column so the two are like for like |
| `gravity` | as above |
| `controller_load` | the robot's declared `controller` field **plus a positive motion attestation**: how many of its joints moved by more than 1e-4 over the window. Deliberately not an exit code — the trap T3.5/T4.5 exist for is a deploy whose runtime was missing, which runs a zero-residual baseline and exits 0 |

---

## 4. `applied_support`: what this column can and cannot attest

**OmniSim has no wrench read-back.** `wb_supervisor_node_add_force` and
`add_torque` are write-only from a Supervisor controller and nothing in the
supervisor API reports what another controller applied; contact points carry no
force either. So this column **cannot total the wrench**, and the
`LADDER_REQUIRED_EVIDENCE_T4` clause that a rig may be a weld, a kinematic base
or an attachment applies here with a different set of names.

What it does instead is enumerate every route by which a non-gravitational,
non-contact wrench could reach a body, and attest a zero **only when none is
open**:

1. any `Robot` declaring `supervisor TRUE` other than the grader's own sampler
   — it can call `add_force` on any node;
2. a robot whose own body carries **no `Physics` node**, which the engine holds
   rigidly;
3. a robot **parented into another body's subtree**, whose pose is carried
   rather than simulated;
4. a `Connector` device anywhere in a tracked robot's subtree;
5. a physics plugin declared in `WorldInfo.physics`, which runs arbitrary force
   calls every step.

Open route ⇒ `attested=None` ⇒ the tier's **`unverified`** cell: not failed and
not credited, `excluded_from_comparison: true`, exactly as
`T3_quadruped/meta.json` → `support_attestation` requires. No route open ⇒
`attested=True` with a zero series **and the argument written into the
citation**, so a reader can see it is a structural proof rather than a
measurement.

⚠ **This is weaker than the MuJoCo column's channel and the difference must
travel.** MuJoCo streams `mjData.xfrc_applied` plus the equality-constraint
reaction, so it can publish *"peak 0.047 × body weight"* for a rig that is
actually running. This column can only say *"nothing could have been
applied"* or *"something could have been and I cannot see it"*. A scene where
an agent legitimately uses a harness — which T4 **permits** — will therefore
land in `T4-support-unverified` on this column and in `T4-supported` with
numbers on MuJoCo. That is a real capability gap in OmniSim's agent-facing
surface, it is named in `../../READINESS.md`, and it is the single most
valuable thing a future engine change could close here.

---

## 5. Six things that were measured wrong first

Named rather than smoothed away, in the order they were found.

1. **Contacts are invisible under Newton.** The *same* T1 probe scene returned
   **1008** support contacts on ODE and **0** on Newton — with
   `supported: true`, `error: null` and 126 sampled steps, i.e. the query ran
   cleanly and saw nothing. `WbSolid`'s contact-point list is fed from the ODE
   collision callback and the Newton backend never populates it
   (`src/omnisim/physics/WbPhysicsBackend.cpp` calls the `contact_points`
   smoke worlds *"ODE-specific"* in as many words). Every contact-dependent
   assertion on this column is therefore backend-conditional. This is a
   finding about the engine, not about the instrument; it is the first blocker
   in `../../READINESS.md`.
2. **`WorldInfo.physics` reads as the literal string `"<none>"`.** Treating
   that as a declared plugin opened support route (5) on an otherwise perfect
   T3 probe and turned `attested` into `unverified` for every scene ever
   authored. One measured scene, one line, and the whole support channel went
   from useless to correct.
3. **`WorldInfo.gravity` is an SFFloat in this engine**, not the SFVec3f the
   upstream lineage uses. Reading only the vector form raised
   `TypeError("'float' object is not iterable")` and reported gravity absent —
   which is T2.4's and T3.4's datum. Both shapes are now read and neither is
   assumed.
4. **A fixed record stride of 5 is too coarse.** On a 16 ms world it gives a
   0.08 s pose interval, and T3.1's continuity clause needs 0.05 s or finer, so
   the clause went **vacuous** while every other channel was fine.
   `--record-stride` is now a *ceiling* the sampler tightens against the
   world's own basic timestep (`MAX_RECORD_DT_S = 0.04`). A world coarser than
   50 ms still cannot satisfy the clause on any stride — that is the agent's
   scene, and the clause says so itself.
5. **`ladder.graders.t2` reaches the generic `run_standalone` and nothing
   else.** Its three siblings prefer a tier-specific hook; T2 does not. With
   the generic hook pinned to T1's cheap mode, `t2.run_and_grade(sim=
   "omnisim")` produced a verdict with **four channels "unanswered"** — i.e.
   blaming our scaffolding — on a run that could have answered every one of
   them. Fixed **without editing a grader**: the generic hook defaults to the
   full scan and `t1_run_standalone` pins the cheap mode for the one tier that
   does not want it.
6. **The URDF importer does not preserve the root link's name.** T3/T4 declare
   the base as `base_link`; OmniSim folds a description's root link into the
   `URDFRobot` node, which carries the name the `.wbt` gave it. On the T3
   probe the named bodies were `walker`, `hip_fl`, `thigh_fl`, `shank_fl`, …
   and **no `base_link` at all** — with the root's own mass (6.0 kg) and the
   subtree's (10.8 kg, exactly `robot.mass_kg_declared`) both correct on the
   `walker` node. The channel substitutes the single robot-class body, refuses
   on ambiguity and **discloses the substitution in every base channel's
   citation**; the T3 core then correctly refuses the substituted body because
   its name is not the declared one. See `../../READINESS.md` — this is the
   tiers' own `open_question_for_the_freeze` and it is the one blocker between
   this column and a publishable T3/T4 cell.

---

## 6. What was measured, and with what

All probes: local RTX 3060 laptop, machine id `9722d23d12a3`, ODE backend
pinned (`backend="ode"`), engine `msys64/mingw64/bin/omnisim-bin.exe`. Probe
scenes and controllers are scratch, not committed — they are scripted controls
a human wrote knowing the thresholds and are not evidence about any agent.

| probe | outcome | what it establishes |
|---|---|---|
| T1, husky + floor, naive drive | `FAIL 4/5`, T1.4 **green** with 1008 support contacts | the T1 path is unchanged by this session's work and still writes no tier document |
| T1, same scene, Newton default | `FAIL`, T1.4 red, **0 contacts of any kind** | finding 1 |
| T2, bench_arm + block + bin + floor, wiggle | `FAIL 2/5`, `unanswered_channels: null` | all eight T2 channels answered: mass 0.2 kg, gravity 9.81, rim 0.2094 m, one static surface found, **clock skew 0.0 s** between the object and end-effector series |
| T3, walker + ground + wall, wiggle, named `walker` | `FAIL 0/5`, every assertion vacuous | finding 6: the core refuses a base whose name is not the declared one |
| T3, same scene named `base_link` | `FAIL 2/5`, `unanswered_channels: null`, `support_attestation: attested` | all eight T3 channels answered: 216 make-and-break transitions, per-foot bodies named, arena 28.359 m, controller attested from 12/12 joints moving |
| T4, strider + ground + wall, wiggle, named `base_link` | `FAIL 2/5`, `unanswered_channels: null`, `cell: T4-unsupported`, `arena_attestation: attested` | all eight T4 channels answered plus the tier's own four required measurements |

In every green-instrument case the reds are the **robot's**: the wiggle
controller is not a gait, the strider falls at t = 0.0 s and the block never
moves. That is what "attributable to the agent" looks like.

---

## 7. Tests

`python -m pytest tests/benchmarks/ladder/adapters/omnisim -q` → **71 passed**
in ~0.3 s, with no engine, no GPU and no network.

| file | tests | what it holds |
|---|---|---|
| [`test_omnisim_channels.py`](test_omnisim_channels.py) | 51 | the hook contract (every name the four shims and the cell runner look up), the document schema and each of its seven defects, base resolution (declared name wins / single-robot substitution / refusal on two candidates / refusal with none), every channel's own reading, and three end-to-end passes through the **real** shims asserting `unanswered_channels == {}` at T2, T3 and T4 |
| [`test_ladder_recorder_tiers.py`](test_ladder_recorder_tiers.py) | 20 | the sampler's tier half against a fake scene graph and a stub `controller` module: mass (stated / absent / density-based / subtree), both gravity shapes, the `"<none>"` plugin, the named-body walk, and **each of the five support routes opening the probe** |

Every case in the second file is one that was measured wrong first (§5).

---

## 8. Effort ledger (plan §5b rule 6)

The plan makes this mandatory and consequential: *"any column whose scaffolding
effort is below half the OmniSim column's is labelled `under-invested` in the
grid and in every prose sentence that mentions it."* The MuJoCo column reads
**≈ 21–25 h cumulative** (T1 ≈ 6–7 h, T2 ≈ 5–6 h, T3 ≈ 5–6 h, T4 ≈ 5–6 h).

| item | this column |
|---|---|
| **engineer-hours-equivalent, T1** | **≈ 8–10 h, ESTIMATED FROM THE ARTIFACTS, not timed.** The T1 half (`evidence.py`'s launcher + the two mappings, `ladder_recorder`'s pose and support-contact channels, the sampler tamper check) predates this session and no ledger was kept for it. The estimate is stated as an estimate and should be replaced if the session that built it left a record. |
| **engineer-hours-equivalent, T2–T4** | **≈ 7–8 h** in one session: ~1 h reading the MuJoCo column's four hooks and the three channel-dataclass modules; ~2.5 h on the sampler's tier half (the inventory walk, the mass/gravity/plugin reads and the structural probe are most of it); ~1.5 h on `channels.py`; **~1.5 h on the six measured defects in §5** — the Newton contact A/B and the base-name finding are about half of that; ~1.5 h on the 71 tests and this record. |
| **cumulative for the column** | **≈ 15–18 h** (T1 ≈ 8–10 h estimated + T2–T4 ≈ 7–8 h measured). |
| **debug iterations to the first green instrument** | **6**, all in §5. |
| **compute cost** | **$0.** Local CPU + GPU already owned. Seven engine probes, ~20–25 s of wall clock each. |
| **tokens / agent cells** | **zero.** No Claude session was run; the campaign's quota is reserved for real cells. |
| **what is NOT included** | the ladder's cores, task registry, channel dataclasses, cell package and negative fixtures — a parallel workstream's, not this column's. This ledger counts only `ladder/adapters/omnisim/` and the tier half of `ladder/controllers/ladder_recorder/`. |

**How to read that honestly, and it does not flatter us.** At ≈ 15–18 h
cumulative, the MuJoCo column's ≈ 21–25 h is **not** below half — so nothing
here labels MuJoCo `under-invested`, and the plan's parity rule is satisfied in
the direction it was written to catch. But the comparison cuts the other way
too and should be said plainly: **this column has had more total investment
across the whole repository's history than any other and still reaches T3/T4
with a support channel that cannot total a wrench** (§4) and a base name that
does not match the task (§5.6). The MuJoCo column got there in a third of the
effort because a library column needs no process, no injection, no ports and
no log parsing — and because MuJoCo's own API answers questions ours does not.
Effort parity is not capability parity, and this ledger is evidence for the
second sentence rather than against it.
