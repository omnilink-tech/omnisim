# MuJoCo arm — exclusions, and the surfaces they apply to

**Contract.** SPEC §6.2 binds every competitor's scaffolding: a bridge faithful
in every included verb but curated by omission is the abuse the fidelity rules
alone cannot catch, so **anything this arm does not wrap must appear here with
a structural justification, not a convenience one.**

The Webots arm's [`EXCLUSIONS.md`](../webots/EXCLUSIONS.md) enumerates omissions
from a *wrapped function surface*, because that arm's `shell+tools` condition
wraps upstream's published Supervisor/Robot API as JSON-RPC verbs. **This arm
has no such bridge** (§1), so the surface that needs auditing here is a
different one: the set of MuJoCo entry points the grader's **observer** wraps,
because a stepping path the recorder does not see is a capability the arm
silently cannot measure (§2).

---

## 1. The `shell+tools` bridge — **not built**, and the reason matters

There is no `runner/manifests/mujoco_shell_plus_tools.json` and no
`runner/tools/mujoco_bridge.py`. Only the **`shell`** condition is runnable on
this arm today, and [`sims.py`](../../sims.py) says so in the `surface` field
rather than implying a bridge exists.

That is a genuine gap in coverage — the A/B between `shell` and `shell+tools`
cannot be run here — but it is a *smaller* gap on this simulator than on any
other, and the reason is worth stating before someone reads the missing column
as a handicap:

- On Webots and OmniSim, the simulator is a **process** with an API you must
  reach across a boundary (an IPC channel, an HTTP harness). Wrapping that API
  as tools genuinely adds reach an agent does not otherwise have.
- On MuJoCo there is no boundary. **MJCF is inert data and every MuJoCo program
  is Python.** The `shell` condition's byte-identical `write_file` + POSIX
  shell already hands the agent the entire documented `mujoco` API, in the form
  its own documentation teaches it. A JSON-RPC wrapper over `mj_step` would be
  a *worse* surface than the one the agent already has, and standing one up to
  fill a column would make the comparison less faithful, not more.

So the honest statement is: **the `shell` cell measures MuJoCo's real
surface**, and what the missing bridge costs is the *ablation* (does a tool
surface help?), not this simulator's best path. If the campaign wants the
ablation, the thing to wrap is the viewer/introspection verbs (`mj_name2id`,
`MjModel` field reads, `mj_forward`, a screenshot via `mujoco.Renderer`) —
naming that here so the omission is a decision on record rather than an
oversight.

## 2. Stepping entry points the observer wraps — and the ones it does not

The grader's recorder is an in-process observer around MuJoCo's stepping
functions (see `recorder.py`). What it wraps **is** the measurement surface: a
completed integration step that does not pass through one of these produces no
sample, no contact scan and no pose.

| entry point | wrapped? | consequence |
|---|---|---|
| `mujoco.mj_step` | **yes** | the ordinary path; one sample per call |
| `mujoco.mj_step1` | **yes** (bind only) | the `step1` / set-ctrl / `step2` split: binding happens here so the t=0 scan is still taken before the first integration |
| `mujoco.mj_step2` | **yes** | the sample point of the split path |
| `mujoco.mj_forward` / `mj_inverse` | no | they compute derived quantities and integrate **nothing**; sampling them would emit duplicate poses at the same simulated time |
| `mujoco.mjx.*` (JAX/GPU) | **no** | a driver that steps through MJX is invisible: **zero samples**, reported as "no samples", never as "it did not move". A real limitation, not a judgement about MJX |
| `mujoco.rollout.rollout` | **no** | batch rollout does not call the Python `mj_step`; same consequence |
| a C/C++ extension calling `mj_step` | **no** | structurally unreachable from Python; same consequence |
| `mujoco.viewer.launch*` | not wrapped, not stubbed | the viewer's own loop calls `mj_step`, so a *passive* viewer run would still be observed — but the benchmark is headless for every arm and a driver that needs a window fails here. `MUJOCO_GL` is deliberately not pinned (see BRINGUP §1) |

**Why these are excluded rather than covered.** Each unwrapped path is a
*different implementation of the integrator* (MJX is JAX; `rollout` is C++
threads) rather than a different way of calling the same one, so observing it
would mean a second recorder, not a second decorator. Covering MJX in
particular would be worth doing if a MuJoCo user's idiomatic answer to these
tasks were an MJX program — for the single-scene, single-robot tasks in this
suite it is not, and that judgement is exactly the kind that should be
overturnable during the §6.2.3 correction window.

## 3. `MjModel` / `MjData` fields the inventory does not publish

The t=0 scan reads what the neutral schema has slots for. Everything below is
present in the model and deliberately not carried into the bundle, because the
bundle is a *physical-units* contract and a grader must not be able to branch
on a simulator's own structures (SPEC 6.2.6).

| not published | reason |
|---|---|
| `geom_friction`, `solref`, `solimp`, `margin`, `gap` | contact-model tuning; no neutral slot, and a grader that read them would be scoring a config rather than a behaviour. Recorded implicitly through the attribution's `cone` / `impratio` |
| `body_inertia`, `body_ipos`, `body_iquat` | mass distribution; `mass_kg` and `movable` are what the schema asks for |
| `jnt_range`, `jnt_stiffness`, `dof_damping` | joint limits and passive dynamics; the schema carries a joint **count**, and a limit-hit channel does not exist in `Body` |
| `sensordata` and the whole `<sensor>` block | **the biggest deliberate omission.** R1's sensing proof is behavioural on purpose (`r1_core`: "a read count would have been simulator-specific and gameable"), so a lidar-sample counter is not evidence the suite wants. Publishing it would tempt a future grader into exactly the non-neutral assertion R1 rejected |
| `<tendon>`, `<equality>`, `<flex>`, `<skin>` | no neutral counterpart. Tendons **are** followed for the robot predicate (an actuated tendon marks its subtrees); they are not otherwise reported |
| the free camera, `mjvCamera`, `mjvScene` | viewer state, not model state; it does not exist in a headless run and is not in `MjData` |

## 4. Contact detail collapsed by the participant rule

`data.contact` carries `dist`, `frame`, `friction`, `solref`, `includemargin`,
`efc_address` and `exclude` per contact point. The neutral `ContactPair` has
`a`, `b`, `point` and `step`, so the rest is dropped — with two exceptions kept
in `adapter_measurements` so the resolution rule cannot hide anything:
`a_detail` / `b_detail` (the precise geom and link behind a subtree-level name)
and `self_pair`.

Pairs are **deduped** by participant pair, keeping the first witness. The two
vacuity counters (`total_observed`, `distinct_named`) are totals over every
scanned step and are **not** deduped and **not** capped, so a truncated pair
list can never make an empty result look like a checked one.

## 5. Structural

Constructors, enum classes (`mjtObj`, `mjtGeom`, `mjtWarn`, ...), the
low-level `mju_*` math library, and the model/data **compilers and
serialisers** (`mj_saveModel`, `mj_saveLastXML`, `mj_printModel`) are not part
of any surface this arm wraps: none of them steps a simulation or answers a
question the evidence bundle asks. `MjModel.from_xml_path` **is** used — once
per run, by the grader, to answer "did the agent's file compile" independently
of the agent's own program.
