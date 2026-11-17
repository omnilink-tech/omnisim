# MuJoCo-column bring-up record (capability ladder, T1)

**2026-08-02.** What is installed on this machine, with hashes; what MuJoCo's
own URDF path actually costs; how the column was proven end to end; and the
exact numbers the bring-up runs produced. Companion to
[`docs/developer/capability-ladder-plan.md`](../../../../../docs/developer/capability-ladder-plan.md)
§4.5 (the roster entry this column fills) and §5b rule 6 (the effort-parity
ledger, §6 below). Shaped after
[`agentbench/adapters/webots/BRINGUP.md`](../../../agentbench/adapters/webots/BRINGUP.md),
the other non-OmniSim column's record.

> **Nothing here is a result.** Every number below comes from a **scripted**
> run — a human wrote the control law knowing the thresholds. A ladder cell is
> an autonomous agent given one sentence and no help (plan §2), and no figure in
> this file may be reported as one. What is proven here is that the *instrument*
> works on this column.

---

## 1. The install (this machine)

**Nothing was installed for this work.** MuJoCo was already present in the
Windows system Python as an OmniBench dependency
([`tests/benchmarks/omnibench/lane1/run_mujoco.py`](../../../omnibench/lane1/run_mujoco.py),
[`lane2/run_throughput.py`](../../../omnibench/lane2/run_throughput.py)), so the
brief's "pip into a suitable interpreter" step turned out to be a no-op. The
engine's embedded interpreter and the bundled Newton runtime next to
`omnisim-bin.exe` were **not touched**, and no venv was created.

| item | value |
|---|---|
| machine | `9722d23d12a3` — host `hc385771a14`, AMD64 Family 25 (16 cores), **RTX 3060 Laptop**, Windows 11, per `python projects/policies/common/env_fingerprint.py` |
| interpreter | `C:\Users\<user>\AppData\Local\Programs\Python\Python312\python.exe` — CPython **3.12.9** (the Windows system Python; the same one the OmniSim harness is documented to use) |
| package | `mujoco` **3.8.1** (PyPI wheel), `mujoco.mj_versionString()` = `3.8.1`, `mj_version()` = `3008001` |
| module path | `…\Python312\Lib\site-packages\mujoco\` |
| native library | `mujoco.dll`, 4,752,896 bytes, sha256 `397e32061458de0ec5e2781294e774460b2b896dc543fe5530eecf056ffe8d71` |
| siblings present | `mujoco-mjx` 3.8.1, `mujoco-warp` 3.8.0.3, `jax`/`jaxlib` 0.10.0, `numpy` 2.4.4, `absl-py` 2.4.0, `glfw` 2.10.0, `etils` 1.14.0, `pyopengl` 3.1.10 |
| invocation | in-process: `import mujoco; mujoco.MjModel.from_xml_path(...)`, `mujoco.MjData`, `mujoco.mj_step`. There is **no simulator process, no CLI and no console script** — the `mujoco` wheel declares zero entry points (`mujoco-mjx` contributes `mjx-viewer`/`mjx-testspeed`, `mujoco-warp` contributes three `mjwarp-*`; none is a converter). |
| **MJX / GPU** | **installed but CPU-only here**: `jax.devices()` returns `[CpuDevice(id=0)]`, i.e. no CUDA jaxlib. **It does not matter for T1** — the whole tier is a single 56 kg wheeled robot on a plane, and the scripted run below does 24.3 s of simulated time in 1.83 s of wall clock on one CPU core, ~13× realtime, *including* a per-physics-step pose dump of all 17 bodies and a per-step contact scan. GPU batching is a T3/T4 question and this column does not touch it. |

**Version pinning matters more here than the table suggests.** Two of the
compiler defaults this column depends on have moved between MuJoCo releases —
`strippath` used to be hardcoded `true` for URDF and is now a plain
`false`-defaulting attribute — so every citation below is written against
**3.8.1** and a different version is a different measurement, not the same one.
The adapter carries this as `MUJOCO_RELEASE` and warns on a row whose run
reports something else.

---

## 2. The pieces (all new, all under this directory)

| file | role |
|---|---|
| `model_build.py` | **raw ROS URDF → a driveable MJCF scene**, by MuJoCo's own documented path, with every edit recorded as a cited `BuildStep`. Includes the mass audit (§3.6) and a strict XML re-parse of the edited URDF (§5 trap 6). |
| `runner.py` | one **scripted** T1 attempt: build (or re-use a scene), settle, drive to the commanded point, hold; writes the run directory. `launch()` spawns it as a **subprocess** so `ProcessFacts` gets a real exit code and a real captured error stream. Also carries the `--defect teleport` fault injector. |
| `recording.py` | reads one run directory off disk. No neutral schema; never raises; records what was missing and what failed to parse. |
| `evidence.py` | → `agentbench.graders.evidence.EvidenceBundle` **plus** the ladder's `SupportContactObservation`. Answers the identity predicate and the engine attribution, and publishes the rule for each. |
| `run_t1.py` | end to end in one command: launch → read → build evidence → run **both** contract checks → grade with the sim-neutral `ladder.graders.t1_core`. |
| `negatives.py` | four **deliberately wrong MuJoCo runs**, so the column's assertions can be observed failing through the real path (§4.2, the red-evidence rule). |
| `test_mujoco_adapter.py` | **43 tests**, 41 of which need no simulator at all. |

Artifact set produced per run (exactly what `recording.read_run` parses):
`build.json`, `roster.json`, `trajectory.json`, `contacts.json`,
`completion.json`, `model_saved.xml`, `process.json`, `stdout.log`,
`stderr.log`, plus the generated `workspace/` holding the edited URDF, the pure
URDF→MJCF conversion and the deliverable scene.

---

## 3. The URDF → MuJoCo reality

This is the honest heart of the column, and it is worth stating the headline
first: **MuJoCo's URDF support is real, first-party and documented — and the
robot the ladder ships does not load.** Both halves are true and neither is the
whole story.

### 3.1 What MuJoCo documents

MuJoCo's *Modeling* chapter, section "URDF extensions" (anchor `CURDF`, source
`doc/modeling.rst` in `google-deepmind/mujoco`), states the whole recipe. Quoted
rather than paraphrased:

> In addition to standard URDF files, MuJoCo can load files that have a custom
> (from the viewpoint of URDF) `mujoco` element as a child of the top-level
> element `robot`. This custom element can have sub-elements `compiler`,
> `option`, `size` with the same functionality as in MJCF, except that the
> default compiler settings are modified so as to accommodate the URDF modeling
> convention. … **This extension is also needed to specify mesh directories.**
> Also note that the compiler attributes `strippath`, `angle`, `fusestatic` and
> `discardvisual` have different default values for URDF and MJCF.
>
> Note that the while MJCF models are checked against a custom XML schema by the
> parser, **URDF models are not.** Even the MuJoCo-specific elements embedded in
> the URDF file are not checked. As a result, mis-typed attribute names are
> silently ignored…
>
> …If the user wants to build models taking full advantage of MuJoCo and at the
> same time maintain URDF compatibility, we recommend the following procedure.
> **Introduce extensions in the URDF as needed, load it and save it as MJCF.
> Then add information to the MJCF using `include` elements** whenever possible.

So the first-party path is: *(1)* a `<mujoco><compiler/></mujoco>` block inside
the URDF, *(2)* load and save as MJCF, *(3)* add the rest MJCF-side. The
save-as-MJCF step is `mj_saveLastXML()` / `MjSpec.to_xml()` in-process, or the
`compile` sample program in the C distribution — **not** in the pip wheel
(*unverified for the C distribution: it was not downloaded on this machine*).

### 3.2 What actually happens, stage by stage (measured, MuJoCo 3.8.1)

Against this checkout's `husky_description` — a real ROS package with
`package://` mesh URIs, COLLADA visuals, Gazebo plugin tags and four continuous
wheel joints.

| # | what an agent would try | result |
|---|---|---|
| 1 | `MjModel.from_xml_path("husky.urdf")` | ❌ `Error opening file 'package://husky_description/meshes/top_plate.stl'` |
| 2 | …with `cwd` set to the URDF's directory | ❌ identical |
| 3 | `<mujoco><compiler strippath="true" meshdir="../meshes/"/></mujoco>` | ✅ loads — **nq 4, nv 4, nu 0, nbody 5** |
| 4 | `strippath="true"` alone (no `meshdir`) | ❌ `Error opening file 'top_plate.stl'` |
| 5 | …plus `discardvisual="false"` | ❌ `no decoder found for mesh file '…base_link.dae'` |
| 6 | wrap the URDF in an MJCF `<include>` | ❌ `Schema violation: unrecognized element 'material'` — `<include>` is an MJCF-fragment merge, not a URDF importer |
| 7 | `mj_saveLastXML` on the loaded model | ✅ 1,861 bytes of MJCF, `meshdir` preserved, **no freejoint, no actuators** |
| 8 | `MjSpec.from_file(urdf).to_xml()` | ✅ byte-identical to stage 7 |
| 9 | stage 3 + `fusestatic="false"` | ✅ **nbody 16** — the chassis reappears as a body, still with **nq 4, nu 0** |
| 10 | `MjSpec` + `body("base_link").add_freejoint()` + 4 actuators | ✅ **nq 11, nv 10, nu 4** |
| 11 | URDF's **own** `<joint type="floating">` from a dummy `world` link | ✅ **nq 11, nv 10, njnt 5** — MuJoCo's URDF parser honours it |
| 12 | …and it falls under gravity (no ground: z 0.2 → −0.589 in 200 steps) | ✅ physics is live; URDF simply cannot express a floor |

### 3.3 The three things that bite, and why only one of them is loud

**`package://` (loud).** MuJoCo ships no ROS package resolver, and the failure
is immediate and legible. The fix is MuJoCo's own and documented:
`compiler/strippath` ("will remove any path information in file names specified
in the model", default **`false`**) plus `compiler/meshdir`. An agent that reads
the URDF-extensions section finds this in one hop.

**COLLADA (silent, and silently *survived*).** MuJoCo 3.8.1 decodes STL, OBJ and
MSH; it has no `.dae` decoder. **Six of the seven meshes the shipped Husky
references are `.dae`** — every visual one. It does not matter, because
`compiler/discardvisual` defaults to **`true` for URDF**, so the visual geoms
are dropped before their meshes are ever opened and the one collision mesh
(`top_plate.stl`) is the only file read. The trap is that this is a *default
saving you*, not a capability: an agent that sets `discardvisual="false"` to get
a nicer screenshot converts a working model into a hard failure, and an agent
that renders the result gets collision boxes rather than the robot. Recorded in
`build.json` as `meshes_undecodable`.

**The welded chassis (silent, and the real one).** A URDF root link has no joint
to the world, and in MuJoCo "if no joints are defined within a given body, that
body is welded to its parent". With `fusestatic` — default **`true` for URDF** —
it is not merely welded, it is *absorbed into the world body*: stage 3 compiles
the Husky to **five bodies (world + four wheels)**, the chassis geometry becomes
world geometry, and `nq = 4`. The model loads, steps, produces no warning, and
the wheels spin in place for ever. Nothing in MuJoCo's output says the robot
cannot move. **This is the failure mode most likely to eat a T1 cell on this
column**, and it is why `negatives.py` ships it as a fixture.

Two first-party fixes exist and **both were verified here**: URDF's own
`<joint type="floating">` from a dummy `world` link (stage 11), and MJCF's
`<freejoint>` added on the MJCF side (stage 10). This column uses the URDF one,
so the conversion stays a pure conversion.

### 3.4 What URDF cannot say, on any simulator

Actuators, a ground plane, a light, contact parameters, sensors, keyframes.
`<transmission>` names a `ros_control` interface, not a force. So *"MuJoCo loads
URDF"* never means *"an agent that has a URDF has a scene"* — an MJCF must be
authored either way, which is precisely what MuJoCo's own recommended procedure
says. This is not a MuJoCo gap and it must not be scored as one.

### 3.5 Where MuJoCo is better than either shipped column

Stated because an under-invested competitor column is our defect (plan §5b), and
the fair reading of this bring-up is that two evidence channels are *easier*
here than on our own product:

- **`t0` is frozen by construction.** MuJoCo is a library and the recorder owns
  the stepping loop, so a scan between `mj_forward` and the first `mj_step` is
  at t=0 with no synchronization flag to trust and no process to race. On the
  OmniSim arm a `GET /robots` on a 10-Husky world cost 22 s of simulated driving.
- **Contacts name both sides trivially.** `mjData.contact` carries `geom1`/
  `geom2`; `mjModel.geom_bodyid` resolves each to a body. The `(id, id)`
  self-pair bug that made `A1.3` unfalsifiable for weeks has no analogue here.
- **The pose series is genuinely per-physics-step**, not per polling round.

### 3.6 The mass trap, found by auditing rather than by symptom

`compiler/inertiafromgeom` defaults to **`"auto"`**: use an explicit
`<inertial>` where a link has one, otherwise **infer mass and inertia from the
link's geoms** at `geom/density` (default 1000 kg/m³). URDF says the opposite —
a link with no `<inertial>` is massless.

This Husky's `base_link` carries two collision boxes and no `<inertial>`
(the mass lives in a separate `inertial_link`, a common ROS idiom). Measured:

| setting | compiled total | `base_link` |
|---|---|---|
| `inertiafromgeom="auto"` (the default) | **175.838 kg** | 116.546 kg |
| `inertiafromgeom="false"` | **56.582 kg** | 0 kg |
| the URDF's own declared sum | **56.582 kg** | — |

A factor of **3.1**, silent, no warning either way. A column that shipped the
175 kg robot would be measuring a different vehicle from every other column, so
the build sets `inertiafromgeom="false"` and `build.json` publishes both numbers
plus the ratio; a >2 % discrepancy becomes a note on every row.

---

## 4. End-to-end proof (bring-up telemetry — NOT benchmark results)

### 4.1 The scripted T1 attempt

`python tests/benchmarks/ladder/adapters/mujoco/run_t1.py --out <dir>` —
container URDF in, graded verdict out, no agent, no network, no GPU.

- **build**: zero problems; compiled mass **56.582 kg** = declared 56.582 kg;
  wheel radius **0.1651 m** and half-track **0.2775 m** *measured off the
  compiled model*, not typed from a datasheet
- **process**: exit code **0**, `timed_out=false`, wall **1.83 s** for **24.268
  simulated seconds** (12,134 steps at 2 ms) — ≈ **13× realtime on one CPU
  core**, with every body's pose written every step
- **roster**: **17 bodies**, `frozen=true`; **7 carry a world-space AABB** (the
  other ten are URDF frame links with no geoms, which honestly have none)
- **contacts**: `supported=true`, **55,924** observed, **55,924** naming two
  distinct bodies, 2,930 emitted pair records, surfaces touched `["ground"]`
- **motion**: arrived at **t = 21.27 s**, final horizontal error **0.1003 m**,
  trailing window inside 0.35 m **4.99 s**, path **6.409 m** against a 5.000 m
  straight line (ratio **1.282**), largest inter-sample displacement **0.0023 m**,
  base z **0.1318–0.1333 m**
- artifacts: `trajectory.json` 7.7 MB, `contacts.json` 441 KB, everything else
  under 10 KB

Adapter confirmation against these **real** artifacts:

- `recording.read_run(run_dir)`: all seven artifacts found, `errors == {}`
- `evidence.build_bundle(...)` + `evidence.support_observation(...)`: build
  cleanly; frozen t=0 inventory, trajectory aligned to the roster by name,
  attribution `mujoco` / solver `Newton` / integrator `Euler` / cone
  `pyramidal` with its citation
- **`agentbench.adapters.check_bundle` → clean, zero findings**, and
  `ladder.graders.ladder_evidence.check_t1_evidence` → clean. (For contrast, the
  Webots arm's bring-up left one standing finding here: no world-space AABBs.)
- `ladder.graders.t1_core.grade` → **PASS 5/5**, every assertion measured

The verdict is **PASS 5/5 for a scripted control**, which says the instrument
works. It says nothing about whether an agent can do this.

### 4.2 The red-evidence fixtures (`negatives.py`)

Plan §5c rule 2: *no assertion enters a ladder cell until it has been observed
FAILING on a deliberately wrong artifact.* `ladder/graders/fixtures.py` does this
for the **core** with synthetic arrays; this does it for the **column**, with
real MuJoCo runs through the real builder, recorder and evidence builder.

| defect | what it is | red | green |
|---|---|---|---|
| `welded` | the `<freejoint>` stripped from the deliverable — i.e. **what a raw URDF load gives you** | T1.1 (never arrives, error 5.000 m), T1.3 (path **0.000 m**), T1.4 (pinned at spawn height, wheels 0.12 m clear, **zero** contacts) | T1.2, T1.5 |
| `no_ground` | the ground body stripped; it falls for ever | T1.1, T1.2 (**z −1178 m**), T1.3, T1.4 | T1.5 |
| `teleport` | the base's free-joint `qpos` written straight at the goal in 1 m jumps, all actuators at zero | **T1.3 only** — largest inter-sample displacement **1.000 m** | T1.1 (it *arrives*), T1.2, T1.4, T1.5 |
| `will_not_compile` | an invalid geom type spliced in; MuJoCo refuses the file | all five, including **T1.5** | — |

`mismatches: none`. **All five T1 assertions have been observed failing through
the real MuJoCo path**, and `teleport` is the one that matters most: it is the
only fixture that separates "arrived" from "drove there", and it passes T1.1
while failing T1.3, which is exactly what T1.3 exists for.

**The `welded` fixture paid for itself on its first execution.** It exposed an
identity rule in this adapter that keyed on `mjJNT_FREE`: a welded chassis
reported as *no robot in the scene at all*, which collapsed the whole verdict
(five nulls) instead of failing two assertions. That is the difference between
"the robot could not move" and "there was nothing to grade" — a physical finding
turned into an instrument failure. The rule is now structural (a direct child of
the world body whose subtree carries joints) and the regression is a test.

### 4.3 Tests

`pytest tests/benchmarks/ladder/adapters/mujoco -q` → **43 passed** in 0.71 s;
`pytest tests/benchmarks/ladder -q` → **101 passed** (mine plus the graders').
Two tests need the wheel; the other 41 guard the pure logic — the URDF reader,
the drive law, the artifact reader, and every mapping into the neutral schema —
on a machine with no simulator, because that mapping is where a column lies.

One of them is deliberately a **tripwire on this document**:
`test_a_raw_load_of_the_container_urdf_fails_on_package_uris` asserts that the
shipped description does *not* load as it stands. If a future MuJoCo gains a
`package://` resolver, that test fails, which is the correct way to learn that
§3's headline changed.

---

## 5. Known gaps, caveats and traps (state, don't bury)

1. **⚠️ The registry seam — this column is not yet reachable through the shared
   front door.** `ladder.adapters.build_t1_evidence` resolves a column's bundle
   builder through `agentbench.adapters.resolve()`, whose registry maps
   `"mujoco"` → `agentbench.adapters.mujoco.evidence` — **a module that does not
   exist in this tree.** This column lives entirely in the ladder namespace
   (`ladder.adapters.mujoco.evidence`), by the brief's constraint not to edit
   `tests/benchmarks/agentbench/`. So `run_t1.build_t1_evidence` composes the
   evidence itself today. Closing it is either a three-line forwarder module
   under `agentbench/adapters/mujoco/` or a ladder-first lookup in the shim;
   both are one-line decisions that belong to whoever owns those files.
   `LADDER_CHANNELS["mujoco"]` already points here and
   `support_observation(phase_b)` is implemented to that signature, so the
   ladder-channel half of the seam is done.
2. **The `teleport` fixture found a one-metre goal bug, and the fix is in the
   runner.** The commanded point is start-relative and "where it starts" means
   *the first recorded sample*. The recorder used to take its first sample after
   the first step, so the driver and the grader resolved goals one step of travel
   apart — sub-millimetre while driving, **exactly one metre** while teleporting.
   The first sample is now taken before the first step. Any other column with a
   start-relative goal has the same hazard.
3. **Error-class lines are not like-for-like across columns.** OmniSim and
   Webots count `ERROR:` lines from an engine log. MuJoCo has no log and no such
   line: hard errors go through `mju_error`, which **aborts the process**, and
   everything short of that is a `mjData.warning` counter. This adapter maps the
   seven physics counters to error-class lines, because counting console text
   alone would make T1.5 unfalsifiable here. The asymmetry is real and its
   direction is **stricter on MuJoCo** — the opposite of the direction a
   competitor comparison must never lean, but still an asymmetry, and it is
   published on every row (`WARNING_NOTE`).
4. **`reached_finalize` has no perfect analogue.** MuJoCo has no build/finalize
   phase: `mj_loadXML` returns a compiled model or raises. The strong witness is
   the **engine-authored** `mj_saveLastXML` output; the fallback is the driver's
   own record that `mjData.time` advanced, and the adapter labels that one
   "the WEAKER of the two witnesses" in the verdict text rather than letting a
   reader assume parity with the Webots arm's `--log-performance` flush.
5. **`no_ground` cannot be distinguished from a broken contact query by the
   vacuity counters alone.** With no floor there is genuinely nothing to touch,
   so `total_observed = 0` legitimately — the same reading a dead query gives.
   What separates them here is that the *same recorder* produced 55,924 contacts
   on the good run. That is a cross-run argument, not a within-row witness, and
   it is a limitation of the witness design rather than of this column.
6. **MuJoCo's URDF parser accepts XML that a strict reader rejects.** Its own
   docs say URDF is not schema-checked; measured here, it also compiled a file
   whose XML comment contained a double hyphen, which `xml.etree` refuses
   outright. `build_scene` therefore re-parses its own edited URDF strictly and
   reports a failure rather than trusting that "MuJoCo loaded it" means "the file
   is valid". (This caught a real defect in the builder's own inserted comment.)
7. **`mjOption.solver == "Newton"` is not NVIDIA Newton.** It is MuJoCo's own
   constraint solver. This repository's default physics backend is *also* called
   Newton and is a different thing entirely. Every attribution this adapter emits
   carries the disambiguation in its `source` string, because a cross-column
   table will contain both words.
8. **Scope.** T1 only. **T2 has since been brought up on this column and has
   its own record — [`BRINGUP_T2.md`](BRINGUP_T2.md)** (the three shipped
   descriptions assembled into a scene, a scripted transfer graded PASS 5/5
   through the real T2 path, and all eight of T2's evidence channels
   implemented). Nothing in either file addresses T3/T4 (locomotion, where
   MuJoCo Playground is the plan's specific named risk — §1 adverse outcome 3),
   and no claim about those tiers may be drawn from this package.
9. **Windows-native, CPU, one machine.** This arm ran on `9722d23d12a3` under
   native Windows; the OmniSim arm is also native Windows and the Webots arm is
   WSL2. Any published cross-column number must say so.
10. **The identity rule has a stated limit.** "A direct child of the world body"
    misses a scene that parents the robot under an intermediate frame body; the
    count would read 0 rather than 1. Published in `IdentityRule.scene_rule` on
    every row rather than left as a comment.

---

## 6. Effort ledger (plan §5b rule 6)

The plan makes this mandatory and makes it consequential: *"any column whose
scaffolding effort is below half the OmniSim column's is labelled
`under-invested` in the grid and in every prose sentence that mentions it."*

| item | this column |
|---|---|
| **engineer-hours-equivalent** | **≈ 6–7 h** of focused work, in one session: ~1 h establishing and verifying the URDF path against MuJoCo's primary docs and twelve measured probe stages, ~2.5 h on `model_build` + `runner`, ~1.5 h on `recording` + `evidence` (the neutral mapping), ~1 h on `negatives` + the 43 tests, ~1 h on this record. |
| **debug iterations to the first graded cell** | **6**, all named above rather than smoothed away: (1) `MjsLight.directional` is `type` in the spec API; (2) `geom_xmat`/`geom_xpos` are on `mjData`, not `mjModel`; (3) a `--` inside an inserted XML comment made the edited URDF invalid to strict readers while MuJoCo compiled it happily; (4) the wheel side/target assignment was inverted, so the robot span in place for 43 s — found by instrumenting the loop, not by staring; (5) the broken negative scenes were written away from their `meshdir` and failed to load for the wrong reason, producing three all-red verdicts that were really one missing mesh path; (6) the first recorded sample was one step late, which the `teleport` fixture surfaced as a one-metre goal offset. |
| **compute cost** | **$0.** Local CPU. Total simulated time across every run in this record is under three minutes of wall clock. |
| **tokens / agent cells** | **zero.** No Claude session was run; none was needed. |
| **what is NOT included** | the ladder's T1 core, task registry and `ladder_evidence` channels — a parallel workstream's, not this column's. This column's ledger counts only `ladder/adapters/mujoco/`. |

**How to read that number honestly.** Six to seven hours is small against the
OmniSim column's history, and if the OmniSim column's own ledger comes in at
more than ~14 h then this column is `under-invested` by the plan's own rule and
must be labelled so wherever it appears. Two things push the other way and
neither cancels it: MuJoCo genuinely needs less scaffolding (no process to
launch, no ports, no IPC, no log parsing, no hot reload — §3.5), and the column
already carries a fuller evidence set than the Webots arm did at the equivalent
point (world-space AABBs present, `check_bundle` clean, four red-evidence
fixtures). Neither of those is a licence to stop; the T2–T4 work on this column
is untouched and is where the asymmetry will actually bite.
