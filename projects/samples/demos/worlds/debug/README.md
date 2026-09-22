# The arm drops the box two times in five

**A robot arm picks a box off one table and places it on another. Two payloads
out of five never make it. An agent finds out why in 90 seconds, from the event
stream alone — no screenshot, no world source, no result file.**

This is OmniSim's debugging demo. It exists to make one claim checkable:
*robot software can be debugged with instruments instead of guesses.* The
positioning it is the proof for is [`docs/developer/positioning.md`](../../../../../docs/developer/positioning.md);
its §6 is the honesty gate every number below is written against.

Honest headline: **90 seconds to the diagnosis, ~2 minutes to the verified fix.**

---

## Run it

Three commands. Nothing here needs the GUI.

```bash
# 1. See the fault. (~15 s. Expect: carried=False.)
#    PICK_OUT is not optional -- without it the controller overwrites its own
#    tracked result file. PICK_AUTOQUIT ends the run at the verdict.
PICK_AUTOQUIT=1 PICK_OUT=drop_result.json \
python -m omnisim run-headless \
  projects/samples/demos/worlds/debug/omniarm6_drop_fault.omniworld --duration 90

# 2. Diagnose it the way an agent would -- one HTTP endpoint, start to finish.
python scripts/dev/diagnose_drop.py \
  projects/samples/demos/worlds/debug/omniarm6_drop_fault.omniworld --expect dropped

# 3. Prove the fault is load-dependent, and that the fix removes it. (~3 min.)
python scripts/dev/drop_payload_sweep.py --both     # 3/5 faulty, 5/5 fixed
```

---

## The bug, as reported

> *"The cell works. Most of the time. Every so often the part just isn't there
> when the arm gets to the place table. We can't reproduce it."*

That is the report you actually get. Here is what makes it hard:

- **The end state looks correct.** A dropped block routinely lands *inside* the
  0.10 m place tolerance — one of them lands on the destination table itself.
  Across the five payloads the controller's own `placed` test scores **5 of 5 as
  success**, including the two that were dropped. A screenshot is useless.
- **There is no exception, no error, no warning.** `run-headless` prints
  `PASS`. The log verdict is a log verdict, not a physics verdict.
- **It is not every part.** Three payloads are carried perfectly and two are
  not, in the same cell, with the same program.

## The bug, as an instrument sees it

Load the world with **full tracking** and poll **one** endpoint.

```bash
curl -sX POST localhost:6789/world/load \
  -d '{"path":"projects/samples/demos/worlds/debug/omniarm6_drop_fault.omniworld","light":false}'
curl -s "localhost:6789/sim/events?since=0&log_since=0"
```

⚠ `"light": false` is not optional. Light mode is the default and it suppresses
five of the ten event types — `contact.began`, `contact.ended`, `grip.acquired`,
`grip.released`, `joint.limit_hit` — which is exactly the set this diagnosis
runs on. `GET /capabilities` → `event_types_detail.suppressed` reports it per
session rather than leaving you to guess.

### The faulty signature

Verbatim from `diagnose_drop.py` on `omniarm6_drop_fault.omniworld`:

```
t_sim 8      contact.began   BLOCK <-> PICK_TABLE
t_sim 6224   grip.acquired   BLOCK
t_sim 6624   contact.ended   BLOCK <-> PICK_TABLE  <- the part leaves the table
t_sim 7376   grip.released   BLOCK   <- flicker, re-acquired 40 ms later (NOT a drop)
t_sim 7416   grip.acquired   BLOCK
t_sim 7424   grip.released   BLOCK   <- flicker, re-acquired 72 ms later (NOT a drop)
t_sim 7496   grip.acquired   BLOCK
t_sim 8128   contact.ended   #193 <-> BLOCK        <- both pads stop touching it
t_sim 8128   contact.ended   #205 <-> BLOCK
t_sim 8128   grip.released   BLOCK   <- GRIP LOST IN MID-AIR
t_sim 8264   contact.began   BLOCK <-> PLACE_TABLE <- it HITS the table 136 ms AFTER
```

There is **no** `contact.began BLOCK <-> PLACE_TABLE` anywhere *before* that
release. The part never reached its destination in the gripper — it arrived
there by falling.

### The healthy signature

Same tool, `omniarm6_drop_fault_fixed.omniworld`:

```
t_sim 8      contact.began   BLOCK <-> PICK_TABLE
t_sim 5344   grip.acquired   BLOCK
t_sim 5704   contact.ended   BLOCK <-> PICK_TABLE
t_sim 6520   grip.released   BLOCK   <- flicker, re-acquired 40 ms later (NOT a drop)
t_sim 6560   grip.acquired   BLOCK
t_sim 7184   grip.released   BLOCK   <- flicker, re-acquired 32 ms later (NOT a drop)
t_sim 7216   grip.acquired   BLOCK
t_sim 7840   contact.began   BLOCK <-> PLACE_TABLE <- ARRIVES, still gripped
t_sim 8464   contact.ended   #205 <-> BLOCK
t_sim 8464   contact.ended   #193 <-> BLOCK
t_sim 8464   grip.released   BLOCK                 <- the deliberate release
```

Put the two side by side and the difference is **one ordering**:
`contact.began BLOCK<->PLACE_TABLE` at 7840 *before* the release at 8464, versus
at 8264 *after* the release at 8128. Everything else looks the same, including
the flickers, including the final resting place of the block.

### ⚠ The trap, and it is the whole reason this demo has a test

**`grip.released` alone is not the signal.** A perfectly healthy carry emits
**two** release / re-acquire flickers while the pads re-seat under load. The
obvious detector — *"a `grip.released` while the part is airborne means it was
dropped"* — fires **three times on a successful carry**.

An instrument that reports a fault on a healthy run is worse than no instrument.
[`tests/test_drop_detector.py`](../../../../../tests/test_drop_detector.py) pins
that this one does not, and deliberately makes the naive rule go red on the same
fixture so the test cannot quietly become vacuous.

### The decision rule

> A **drop** is the **final** `grip.released` for the part — final meaning no
> `grip.acquired` for the same part follows within 200 ms — that is **not**
> preceded by a `contact.began` between the part and the destination surface,
> and **is** followed within 500 ms by a `contact.began` between the part and
> some named surface.

Three details that matter, all of them learned the hard way:

1. **Order, not presence.** On this world the dropped block lands *on the place
   table*. `contact.began BLOCK <-> PLACE_TABLE` is emitted on both runs. Only
   the **ordering** relative to the release separates them.
2. **The pads are not a surface.** Contacts name URDF links by opaque node id
   (`#193`, `#205`) because URDF links carry no DEF. The pads re-touch the
   falling part within milliseconds of the release; counting that as "it landed"
   would call every release a drop. `test_the_pads_are_not_mistaken_for_a_landing_surface`
   pins it.
3. **If the part never lands inside the window, the verdict is `inconclusive`,
   not `dropped`.** The stream may simply have ended in mid-fall. Refusing to
   certify on thin evidence is the same posture as `--fail-on-runaway`.

### The suspect parameter — off the same stream

The engine publishes its own contact and solver settings at world start, and the
harness forwards engine stdout as `controller.log` events. So the agent reads the
value of the parameter it suspects **without opening the world file**:

```
controller.log  INFO: [OmNewtonBackend] contact/solver params from WorldInfo:
                mu=1.5 ke=8000 kd=200 iters=150 ls_iters=50
```

`WorldInfo.newtonGroundMu` is **1.5**. This cell's recipe was recorded at **6**
(the flagship world's own header: *ladder T2, 2026-08-03 — 29.704 s hold,
4.242 mm max deviation, 1.214 m of tool travel*). Friction is **global** in this
world — one `ShapeConfig` serves every body — so the pad cannot be grippier than
that one number.

That is symptom → mechanism → parameter value, entirely from `GET /sim/events`.

---

## The fix

One field:

```diff
- newtonGroundMu 1.5
+ newtonGroundMu 6
```

`omniarm6_drop_fault_fixed.omniworld` is that change and nothing else. Check it
yourself — strip the comment headers and the two worlds differ in exactly two
lines, one of which is the `title` string:

```bash
cd projects/samples/demos/worlds/debug
diff <(grep -v '^#' omniarm6_drop_fault.omniworld) \
     <(grep -v '^#' omniarm6_drop_fault_fixed.omniworld)
# 10c10  title "... (FAULTY: newtonGroundMu 1.5)"  ->  "... (FIXED: newtonGroundMu 6)"
# 18c18  newtonGroundMu 1.5                        ->  newtonGroundMu 6
```

Same arm, same tables, same 0.30 kg block, same solver settings, same
controller. One number.

---

## The measurements

Every figure below was produced on the machine named at the bottom of this page.
Re-run any of them with the commands above.

### The payload sweep — `python scripts/dev/drop_payload_sweep.py --both`

**FAULTY — `newtonGroundMu 1.5`**

| payload | carried | pinched | placed | drift | wall |
|---|---|---|---|---|---|
| 0.15 kg | yes | yes | yes | 51.7 mm | 14.7 s |
| 0.20 kg | yes | yes | yes | 34.5 mm | 15.7 s |
| 0.25 kg | yes | yes | yes | 23.3 mm | 28.4 s |
| **0.30 kg** | **DROP** | no | yes | 225.3 mm | 15.3 s |
| **0.35 kg** | **DROP** | no | yes | 224.4 mm | 16.3 s |

**3 of 5 carried.** And the line the sweep prints for itself:

```
!! 0.30 kg, 0.35 kg landed inside the place tolerance after being DROPPED
   -- `placed` would have scored 5 of 5 as success.
```

**FIXED — `newtonGroundMu 6`**, one field changed, nothing else

| payload | carried | pinched | placed | drift | wall |
|---|---|---|---|---|---|
| 0.15 kg | yes | **no** | yes | 28.3 mm | 15.3 s |
| 0.20 kg | yes | yes | yes | 31.5 mm | 14.8 s |
| 0.25 kg | yes | yes | yes | 23.7 mm | 14.3 s |
| 0.30 kg | yes | yes | yes | 23.0 mm | 13.3 s |
| 0.35 kg | yes | yes | yes | 20.4 mm | 13.7 s |

**5 of 5 carried.** Both failures disappear. Total for the ten runs: **161.9 s**.

Read the `drift` column of that second table before celebrating — see
[Honesty](#honesty) below.

⚠ And read the `pinched` column sideways too. At 0.15 kg the **faulty** world
passes the pinch assertion and the **fixed** one does not, while drifting nearly
twice as far (51.7 mm vs 28.3 mm). `pinched` is a contact-census test — both pads
on the block's own grasp faces at *both* the lift and carry samples — not a
measure of how well the part was held. It is not a scoreboard. **`carried` is the
success test; `drift` is the quality measure.** Publishing a metric that can
reward the broken configuration, rather than quietly dropping it from the table,
is the same discipline this whole demo is arguing for.

### Determinism

The two faulty sweeps above were run in separate processes on separate
invocations and produced **identical** drift figures to 0.1 mm on every row.
Newton's CPU `mujoco` solver — `newtonSolver "mujoco"`, the default and what
this world declares — is bitwise deterministic here.

That is the point of the demo, not a footnote. **The intermittency does not come
from noise.** The physics repeats exactly; what varies is the *payload*. A
failure you can make happen again on demand is a failure you can fix.

⚠ **Scope.** This is the CPU `mujoco` solver only. GPU determinism on
`mujoco_warp` is **refuted**, not unmeasured: 0 of 24 contact-rich pairs were
bitwise. Never quote "OmniSim is deterministic" without naming the solver.

### Timings

| step | measured |
|---|---|
| one headless pick, `PICK_AUTOQUIT=1` | **13.3–28.4 s** wall (it is ~34 s shorter than a bare `--duration 45`) |
| cold harness → live | **1.6 s** |
| `POST /world/load` this world, full tracking | **4.4–5.0 s** warm |
| `GET /sim/events` | **28.6–37.6 ms** per call |
| **cold start → named root cause** | **30.0 s** (see below) |
| the decision rule itself, over a captured stream | < 0.2 s, no engine |
| the whole five-payload sweep, both worlds (10 runs) | **161.9 s** |

⚠ **Poll `/sim/events` and nothing else.** Every read pauses the engine for the
duration of its own walk. Polling five endpoints at 2 Hz stretched an 11 s run
to **104 s**. `/sim/contacts` alone costs 126 ms per call on this world.

⚠ **Do not build a timeline out of the timestamps.** Supervisor events carry
`t_sim_ms` and log events carry `t_wall` (Unix epoch); `seq` collides across the
two sides, and `t_sim_ms` under-reports against the engine's own step counter
(measured 2.7× on one run). **Event order is reliable. Absolute time is not.**
The decision rule uses order and intervals within the supervisor stream only.

### The live run — is "90 seconds" honest?

`python scripts/dev/diagnose_drop.py <world> --expect dropped`, cold, including
starting its own harness, loading the world, watching the whole pick, deciding,
printing the evidence chain, and reaping the harness and its engine:

| | faulty world | fixed world |
|---|---|---|
| harness live | 1.6 s | 1.6 s |
| `POST /world/load` (full tracking) | 5.0 s | 4.4 s |
| watching `/sim/events` to the verdict | 21.5 s | 19.8 s |
| `GET /sim/events` cost | 37.6 ms/call | 28.6 ms/call |
| events seen | 116 | 115 |
| **total wall** | **30.0 s** | **27.4 s** |
| verdict | `DROPPED` | `CARRIED` |

So the claim holds with room to spare: **30 seconds from cold start to a named
root cause**, against a 90-second budget. What does *not* fit in 90 seconds is
the verified fix — re-running the five payloads on the corrected world is
another ~75 s. Hence the honest phrasing: **90 seconds to the diagnosis,
~2 minutes to the verified fix.**

Both live runs also exercised the flicker trap for real. The faulty run emitted
two flickers (re-acquired 40 ms and 72 ms later) *and* a genuine drop; the fixed
run emitted two flickers (40 ms, 32 ms) and no drop. The detector separated them
correctly in both directions.

The captured streams are replayable with no engine at all:

```bash
python scripts/dev/diagnose_drop.py --offline captured_events.json
```


---

## Honesty

This demo is an argument that OmniSim's instruments do not lie. It would be a
poor argument if the demo itself shaded anything.

### The "fixed" arm is not nominal — it ships a known open defect

`omniarm6_drop_fault_fixed.omniworld` carries the block and places it, 5 of 5.
It is **not** a clean bill of health:

- **Every payload drifts 20.4–31.5 mm.** ⚠️ An earlier version of this section
  compared that against **4.242 mm** and called it "4.8× to 7.4× the reference".
  **That comparison was wrong and is corrected here rather than deleted.**
  4.242 mm is the *recipe's* provenance from a **different scene** — ladder T2,
  2026-08-03 — exactly as the world header states. This cell's own last recorded
  drift is **20.55 mm with `ok=FALSE`** (2026-08-22,
  `_real_pick_result.json`). It has never been under 20 mm. The honest
  statement is that HEAD widened an already-failing 20.55 mm to 28.34 mm.
- At the shipped 0.15 kg payload the controller's own `pinched` assertion
  **fails** and its last log line is `-> FAIL` — while `run-headless` prints
  `PASS` with 0 errors and 0 warnings.
- At that same payload the block slides **15.6 mm down through the pads** during
  the 0.47 m carry (block z 0.4426 → 0.4270 while the tool z is unchanged) and
  levers the pads 3.7 mm further apart — then lands standing anyway, so the
  end-state screenshot looks perfect.

This is a **real widening** of an already-failing number, found by the
instruments while this demo was being built. ⚠️ An earlier version of this
section called it "previously unreported". **That was also wrong**: the commit
that caused it reported it. See "Localised" below. It is stated here rather
than tuned away. Two things follow, and both are the point:

1. The A/B is still valid — the fixed world carries **5 of 5** and the faulty
   world **3 of 5** — but the word for the fixed arm is *"repaired"*, not
   *"healthy"*.
2. A green `run-headless` told us nothing about it. The instruments did.

#### Localised

**Cause: commit `17f003e7f` (2026-09-10), "newton: URDF inertia tensors and
Robot-root inertia now reach the solver."** Before it, declared `<inertia>`
tensors never reached the solver at all (`OMNISIM_URDF_USE_INERTIA` defaulted
OFF). The fix flipped that on, the eight arm links got their declared tensors,
and `link6` dropped 2–5×. A much lighter wrist responds faster to the block's
reaction torque over the 0.47 m carry.

**The commit measured this and wrote it in its own message** — *"its drift
widens to 0.0283 m"*. It was never unreported. The first search simply did not
look there.

One run each on the shipped flagship cell (`newtonGroundMu 6`, 0.15 kg),
`PICK_AUTOQUIT=1`:

| run | `drift` | inertia census | verdict |
|---|---|---|---|
| baseline, no hatch | **28.34 mm** | 8 declared, 1 from geometry | reproduces |
| `OMNISIM_URDF_USE_INERTIA=0` | **19.32 mm** | **0 declared, 9 from geometry** | **the lever, −32%** |
| `OMNISIM_NEWTON_INERTIA_COM=0` | 36.06 mm | 8 declared, 1 from geometry | worse, +27% |
| `OMNISIM_NEWTON_ROBOT_GEOM_INERTIA=0` | 28.34 mm | 8 declared, 1 from geometry | bitwise inert |

⛔ **Do not revert `17f003e7f`.** It is a correctness fix — the drift is a truth
it exposed, not damage it did. The stale artefact is **this demo's grasp
recipe**, tuned in August 2026 against wrong arm inertia: it commands ~5.0 N per
pad against a 0.123 N per-pad Coulomb bound. The fix is to re-tune the grip and
re-record the reference against *this* world.

**Residual not accounted for:** 19.32 mm is still not the 20.55 mm of
2026-08-22, and the failure even changes sign — with the hatch the block stays
up (1.1 mm) but slides sideways. A second change in the same window moved the
arm's reached pose; `SPAWN_AT_POSITION` likely owns ~1.2 mm of it.
`17f003e7f` owns ~9.0 mm.

##### The first attempt — a negative result, kept

Three commits were the original suspects. All three were wrong. Kept so nobody
re-runs them:

| run | `drift` | verdict |
|---|---|---|
| `OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO=1` | 28.34 mm | **no effect** |
| `OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO=0` | 28.34 mm | **no effect** |
| `OMNISIM_NEWTON_STATICS_ON_WORLD=0` | 28.34 mm | **no effect** |
| `OMNISIM_NEWTON_SPAWN_AT_POSITION=0` | **43.06 mm** | a real lever, but **worse** |

Bitwise-identical with a hatch flipped in *both* directions is a genuine
exoneration on a deterministic solver, not a weak signal. The lesson worth
keeping is about method, not physics: the search went straight to recent
*physics* commits and never checked whether the causing commit had already
documented its own effect. It had.

### The failure rate is **two in five**

Not one in five. The 0.15 / 0.20 / 0.25 / 0.30 / 0.35 kg set drops **two**. A
1-in-5 set would need another calibration sweep between 0.25 and 0.30 kg and has
not been done.

### Mass is not a smooth dial

Inside 0.15–0.35 kg the behaviour is ordered: lighter holds, heavier drops. It
does **not** extrapolate. Measured: **0.45 kg holds at the faulty friction and
drops at the healthy one.** This is a marginal contact state, not a Coulomb
margin, so do not narrate it as "heavier is harder". The demo stays inside the
band where the ordering is real, and says so.

### `placed` is not a success test

Use `carried` — was the part still airborne in the gripper at the carry sample.
`placed` scored 5 of 5 on a run where two were dropped.

### What OmniSim does **not** do, and this demo does not imply

Per [`positioning.md` §7](../../../../../docs/developer/positioning.md):

- **No pause over HTTP.** The engine free-runs between calls. The per-read guard
  is internal and per-read; you cannot hold it yourself.
- **No breakpoints, no watch conditions** — blocked by the same missing pause.
- **No record / replay / run-diff** as a product surface.
- **Snapshot is not a checkpoint.** `POST /sim/snapshot` saves poses and joint
  angles; velocity is never captured and solver state is never touched. It does
  not rewind the clock. Identical forward evolution is not achievable and must
  not be claimed.

This demo re-runs a *world*; it does not rewind a *run*. What makes that
sufficient here is the CPU-path determinism above — nothing more.

### One more limit this demo bumped into

There is **no endpoint that returns `WorldInfo` fields as JSON**. The engine's
`[OmNewtonBackend] contact/solver params` log line is the only wire-reachable
source for the value of `newtonGroundMu`, and it is emitted once at world start —
so an agent that begins polling late will miss it and has to restart the capture
from `log_since=0`. Worth adding a real endpoint.

---

## What is in this directory

| File | What |
|---|---|
| `omniarm6_drop_fault.omniworld` | the faulty cell — `newtonGroundMu 1.5`, 0.30 kg payload |
| `omniarm6_drop_fault_fixed.omniworld` | the same cell with `newtonGroundMu 6` |
| `README.md` | this page |

Elsewhere:

| File | What |
|---|---|
| [`scripts/dev/diagnose_drop.py`](../../../../../scripts/dev/diagnose_drop.py) | the decision rule (pure, importable) + the live harness driver. `--offline` replays a saved capture with no engine |
| [`scripts/dev/drop_payload_sweep.py`](../../../../../scripts/dev/drop_payload_sweep.py) | the five-payload batch, faulty vs fixed |
| [`tests/test_drop_detector.py`](../../../../../tests/test_drop_detector.py) | 14 engine-free tests, including the flicker trap and the naive-rule control |
| [`projects/samples/demos/controllers/omniarm6_real_pick_place/`](../../controllers/omniarm6_real_pick_place/) | the controller, **shared with the flagship demo and unmodified** |
| [`projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld`](../flagship/omniarm6_real_pick_place.omniworld) | the healthy cell these two are derived from |
| [`docs/guide/friction-grasp.md`](../../../../../docs/guide/friction-grasp.md) | why `basicTimeStep 8`, `condim 4`, the elliptic cone and `impratio` are load-bearing and must not be "fixed" |

### Why the grip cannot be faked here

Nothing in this cell is welded. Almost every "grasp" in the tree is a kinematic
weld — the bridge welds the nearest `DEF GRASP_*` node to the tool and teleports
it to the TCP every tick, which would hold with the fingers wide open, for ever.
The block here is deliberately **not** named `DEF GRASP_*`, so
`ArmBridge._attach_nearest` is *structurally* unable to weld it. The only thing
holding it is contact friction between two Robotiq 2F-85 pads.

That is what makes the fault real, and what makes the fix mean something.

### Operational notes if you re-run this

- `PICK_AUTOQUIT=1` ends each run at the controller's verdict (~12–28 s) instead
  of idling out the `--duration`.
- ⚠ **Always set `PICK_OUT`.** Without it the controller overwrites its own
  tracked `_real_pick_result.json` next to its source.
- ⚠ **Always set `OMNISIM_LOG_PATH`.** Without it every run writes the shared
  repo-root `omnisim_log.txt` and the last one wins.
- `run-headless` reports `ERROR: omnisim exited with code 1` on a dropped run
  under `PICK_AUTOQUIT`. That is `simulationQuit(1)` propagating the
  controller's own verdict, **not** a load failure — the result JSON is complete.
- `python -m omnisim harness` survives `Popen.terminate()` and keeps its port.
  Assert the port is free before starting, or you will silently drive a previous
  session's harness and read its stale event ring. `diagnose_drop.py` refuses to
  start if the port is held, and reaps the harness, its port owner and its engine
  on the way out — killing only engines that were not already running and whose
  command line names this world.

---

## Machine

Every number on this page was measured here, and nowhere else:

```
OmniSim 8.5.1, main @ 9bf6e006b       Windows 11, Python 3.12.9
AMD64 Family 25 Model 80, 16 cores    NVIDIA GeForce RTX 3060 Laptop (596.36)
newton 1.5.0  warp 1.16.0  mujoco 3.11.0  mujoco_warp 3.11.0
newtonSolver "mujoco"  (CPU mj_step)
machine id 9722d23d12a3
```

Regenerate with `python projects/policies/common/env_fingerprint.py` before
quoting any of it anywhere else.
