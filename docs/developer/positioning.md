# OmniSim positioning — the canonical statement

**This file is the single source of truth for what OmniSim says it is.** Every
tagline, package description, doc front door, splash string and pitch paragraph
in the tree should be traceable to this page. If a surface disagrees with this
page, the surface is wrong — unless the disagreement is a *measurement*, in
which case this page is wrong and should be corrected in the same change.

Rewritten 2026-09-17 by owner decision, on top of the 2026-09-14 capability
audit. §7 — the honesty gate — **is** that audit and is carried over intact:
one row was widened by that decision, and it says so in place, with its boundary
attached. Nothing on this page may be widened without a new measurement.

---

## OmniLink access policy — owner decision, 2026-09-22

OmniLink is the connected AI experience and requires an OmniKey on every plan. Do not advertise keyless command handling as an OmniLink agent. Missing credentials or model failures must report connection errors; no automatic local-command downgrade. OmniSim remains the free workshop with ordinary typed controls and Stop available independently. The setup guide supersedes historical keyless descriptions below.

## 1. The change

| | |
|---|---|
| **Was**, to 2026-09-14 | "The simulator you can talk to." |
| **Then**, 2026-09-14 → 2026-09-17 | "Where robot software gets debugged." |
| **Is** | **"An open-source robotics workshop for agents."** |

⚠️ **"Where robot software gets debugged" is retired.** It is struck from every
user-facing surface — README, AGENTS.md, the doc front doors, the CLI banner and
`--help`, the About box — and from the trademark assertion in
[TRADEMARKS.md](../../TRADEMARKS.md). Do not reintroduce it as a tagline, a
splash string, a CLI line or a section heading.

What that line got *right* is kept, and it is §6: **the instruments do not lie.**
That is still the deepest property in the tree and still the hardest thing for a
competitor to copy. It is now a pillar rather than the headline.

We do **not** stop calling OmniSim a simulator. It is one, it is how people
search for it, and denying it reads as evasion. The sentence is *"more than a
simulator"*, not *"not a simulator"*.

## 2. Why the debug line had to go

- **It presupposed a robot, a controller, and a bug.** Three things the person
  we most want to reach does not have. Someone learning robotics, someone with
  an idea and no hardware, and someone holding a robot they cannot program all
  heard nothing addressed to them.
- **It named a task, not a place.** A task is something you finish and leave. A
  workshop is somewhere you keep coming back to — and the tree is already a
  workshop: import, authoring, physics, training, capture, ROS 2, the bridges.
  The tagline described one bench in it.
- **"Debugging" is the last step, and we were selling only the last step.** The
  agent that debugs your controller is the same agent that wrote it, built the
  world it runs in, and imported the robot from your CAD. Naming only the end of
  that arc hid the rest of it.

## 3. Who it is for

> **If you're learning robotics, building a robot, or have an idea but don't
> have access to the hardware, OmniSim is built for you.**

That sentence leads. The barrier to robotics is not talent and it is not
curiosity — it is a robot on a desk and the skill to program it. OmniSim removes
the first and lends you an agent for the second.

## 4. The statement

> **OmniSim — an open-source robotics workshop for agents.**
>
> If you're learning robotics, building a robot, or have an idea but don't have
> access to the hardware, OmniSim is built for you.
>
> OmniSim is more than a simulator. It's an open-source robotics workshop
> designed for agentic development — a place where an agent has everything it
> needs to work on any robotic system.
>
> You can simulate complete robotic systems with high-fidelity physics, build
> digital twins, connect simulation to real robots, make the parts on
> twins of the CNC routers, lasers and 3D printers that build them, and give AI
> agents a workshop to program, test, and debug your system.
>
> You can do all of that simply by talking to it.
>
> **No robot?** Simulate one.
> **Already have a robot?** Build its digital twin.
> **Don't know how to program it?** Let an agent help build and debug it.
> **Designed one?** Make its parts on machine twins.
>
> The goal is simple: make robotics accessible to anyone with an idea.
> OmniSim is completely free and open source.

## 5. The pillars, and the boundary on each

Each pillar is a claim somebody will test on day one. The boundary is not a
disclaimer to bury — it is what makes the claim survive being tested, and §6 is
why we state it that way.

### Simulate complete robotic systems with high-fidelity physics

**Ships:** [Newton](https://github.com/newton-physics/newton) (MuJoCo solver) is
the only physics backend — rigid bodies, joints, friction, contacts, cloth, soft
bodies and CUDA granular media, with wgpu rendering, cameras, range finders,
lidar and the rest of the sensor battery. 52 demo worlds, procedural world
generation via [omniworld](omniworld-user-guide.md), multi-instance parallel runs.

⚠️ **Boundary.** "High-fidelity" describes the solver we run on; it is **not** a
claim to beat Isaac, MuJoCo, Genesis or Drake on fidelity or GPU scale, and we
do not invite that comparison as our headline — see
[simulator-comparison.md](simulator-comparison.md) for where we actually stand.
And **no physics runtime means no physics at all**, not a degraded mode:
`python -m omnisim doctor` exits non-zero on it for exactly this reason.

### Build digital twins

**Ships:** native URDF import (`URDFRobot`), CAD import through
[`step_to_urdf.py`](step-to-urdf.md), 20+ vendor robot packages under
[`projects/robots/`](../../projects/robots/), and the authoring loop in AGENTS.md
§5 — load, inspect the scene tree, edit, `POST /world/sync`, screenshot.
Since v10 also **machine twins** — a CNC router, a laser and an FDM printer built
from their controller configs — and the robots made with them
([OmniSim Manufacturing](#omnisim-manufacturing--what-may-be-claimed-v10-2026-11-04),
with its own boundary: simulation only, no twin validated against its machine).

### Make the parts on machine twins (v10)

**Show the machines, not only the robot.** A reader who sees only the finished
SparkRover assumes it was assembled from parts that came from somewhere else.
The claim is that OmniSim **runs the machines that make the parts**: the LongMill
CNC and laser twins answer GRBL 1.1 (a GRBL sender streams them a job over TCP),
the SV08 twin runs CuraEngine G-code planned as Klipper plans it, the tool's real
path removes or lays the material, and each part is measured against its
drawing. The CNC twin can also be **talked to** through its OmniLink bridge
(live run 2026-11-01: a spoken pocket milled, a too-deep one refused by
`cut_depth`; an OmniKey is required). Every Manufacturing surface shows the
machines at work beside the robot: the README's three-machine table
(`docs/images/manufacturing/machine_*.png` over `twin_*.png`) is the pattern.

⚠️ **Boundary.** Say "the twin cuts / prints"; never "OmniSim controls your
machine". The CNC bridge drives GRBL over a stream, so it is built to run a real
board, but **it has not been run on one**, and no twin is validated against its
real machine. The full boundary is in
[OmniSim Manufacturing](#omnisim-manufacturing--what-may-be-claimed-v10-2026-11-04).

⚠️ **Boundary.** A twin here is **a model built from your robot's own
description** — its URDF, its CAD, its joint limits and inertias — running in
Newton. It is not live-synced telemetry from the physical machine, and its
dynamics are not validated against that machine unless you validate them. Say
"build a model of your robot", never "a twin that matches your robot's
behaviour", until somebody measures the second thing.

### Connect simulation to real robots

**Ships:** one control surface over two backends. The same agent tool names,
argument schemas, return shapes and HTTP endpoints address a robot in OmniSim
and a robot on your bench — the swap is the dispatch implementation behind the
bridge, and [omnilink-sim-to-real.md](../guide/omnilink-sim-to-real.md) names
every file in the seam. Alongside it the
[ROS 2 sidecar](../../packages/omnisim-ros2/) ships and speaks the
`simulation_interfaces` standard (Tier 1 services, Tier 2 topics and sensors,
Tier 3 `ros2_control`, Nav2 planning), and MAVLink HIL exists for aircraft.

⚠️ **Boundary — state it wherever this pillar appears.** What is portable is
**the software above the driver.** The bridges in
[`agents/bridges/`](../../agents/bridges/) run against mock drivers and log the
commands they *would* send; wiring one to your hardware is your work. **No
policy trained in OmniSim has been validated on physical hardware**, and
sim-to-real transfer of learned control is unproven here. Claim the control
surface. Never claim physics transfer.

### Give AI agents a workshop to program, test and debug your system

**Additional measured task (2026-10-28):** the physical-wheel mobility assessment
ships as `python -m omnisim assess sweep`. The [guide](../guide/robot-assessments.md)
and [64-case Husky campaign](../../tests/assessment/evidence/husky-envelope-20261028/README.md)
document immediate-braking tests over 0–20° slopes, contact μ 0.10–0.80 and
0/10/20 kg rigidly mounted loads at a target 0.6 m/s, on CPU machine
`9722d23d12a3`. Four fresh confirmations reproduced the selected pass/fail
distances in these runs. Separate campaigns cover seven physical-wheeled model
configurations in 28 unloaded and seven mounted-payload cases. This is simulated model/test-controller behavior,
not vendor firmware or hardware validation, a global safety envelope, or a
general assessment adapter for arms, legs and drones. The inventory distinguishes
69 URDF variant assets from the seven available physical-wheel task adapters.

**Ships:** the agent-facing HTTP harness (AGENTS.md §5), 46 first-party
[MCP tools](../../packages/omnisim-mcp/) — 42 of them one call to the same
harness a human uses, and four that reach a robot's OmniLink bridge —
[AGENTS.md](../../AGENTS.md) itself as the machine-readable
entry point, and the debugging instrument this project is deepest in — eleven
code-verified event types on one cursor-paged stream, contact / joint
(`hit_limit`) / device / bounds inspection, a **held pause and break-on-event**
(v9), CPU-path bitwise determinism, and CI
verdicts that fail rather than pass on thin evidence. That is §6 and §7.

⚠️ **Boundary.** The agent now controls time in the debugger's sense — hold,
single-step, break on an event — but **not in the recorder's sense**: no record,
no replay, no run-diff, and `POST /sim/snapshot` is still not a checkpoint.
Break detection is **never sub-step**, and free-running it can miss a transient
entirely (§7). Light mode is the default and silences
5 of the 11 event types. Full detail in §7.

### …simply by talking to it

**Ships:** every URDF robot has a chat demo
(`projects/samples/demos/worlds/chat/omnilink_<robot>.omniworld`, ×21 as of
2026-09-21 — the Mavic joined when it was given a gated `/prompt`) — right
click → *Show Robot Window*, type `home` or `drive forward 1 m`. And the MCP
server means talking to Claude Code or Cursor *is* driving the simulator.

**What answers the sentence (v9).** Two jobs, split on purpose. A free
deterministic parser interprets first and a model is called only for what it
declines; then `gate.check(utterance, frames)` — a deterministic veto that sits
between a typed frame and a motor and **does not care who produced the frame** —
decides whether anything actuates. The gate's rules run on `(utterance, frames)`,
so they protect against the parser, against a model, and against whatever
interprets next.

⚠️ **This is a PILLAR of the workshop, not a new tagline.** *"The simulator you
can talk to"* remains **retired** (§1); v9 does not re-promote it. Talking is
still the interface, and the workshop is still the claim.

⚠️ **Boundary, and it is narrow.**
- The gate is a **sanity rail, not a world model**. It has no arena: a two-hour
  soak walked a Husky off a 12 m floor to (−6.86, 4.75) and nothing objected.
- **Not every actuation path is vetted.**
  [`GATE_COVERAGE.md`](../../packages/omnisim-bridges/GATE_COVERAGE.md) is the
  authoritative list of what is not — including the direct REST actuators in all
  four bridges.
- The measured figure is **22.0% → 2.4%** of "must not move" cases
  (langsoak, ungated vs gated, same corpus, same judge, **one robot, one
  surface**). It is not "zero false actuations", and it is an adversarial
  corpus, not real operator traffic.
- The four-class wiring landed 2026-09-21 and is unit-tested, and the drone's
  chat demo is verified live; **there is still no behavioural shift-demo
  measurement per robot class.** Until there is, do not say the gate is
  *measured* on arms, quadrupeds or drones — say it is *wired* to them.
- **There is no humanoid surface at all.** G1, H1 and DR02 have a policy and
  controller stack and no chat bridge.
- Nobody has measured `gate.check()`'s wall time. There is no latency claim.
- The OmniLink model tier needs `python -m omnisim key` or a BYOK provider key.
- Do not imply a plain-English sentence reaches the solver directly.

### …make robotics accessible to anyone with an idea

⚠️ **Boundary on the accessibility claim itself, because accessibility is the
claim.** Windows 10/11 has the only prebuilt package. Linux (Ubuntu 24.04 /
22.04) is a source build, 25–45 minutes, most of it compiling. **macOS is not
supported** — no package, no verified build, Newton unverified. State the
platform truth next to the accessibility promise every time; a newcomer who
discovers it after downloading is the exact person this positioning is for.

## 6. The differentiator: instruments that refuse to lie

This is the part no competitor has, and the part that makes the workshop worth
standing in. It is not a feature, it is a property of the whole surface, and it
is exactly what you want from a workshop you cannot see into — **a tool that
lies to you is worse than no tool.**

Evidence, all of it in-tree:

- `GET /sim/contacts` **never claims to be complete.** It returns what it
  walked, `completeness`, `empty_set_reasons[]`, and which bodies are
  physically inert. An empty contact list is never silently sold as "no
  contact". [`omnisim_harness.py:160-171`](../../scripts/harness/omnisim_harness.py#L160-L171)
- `--fail-on-runaway` **refuses to certify on thin evidence.** Zero tracked
  bodies, or fewer than `window+2` samples, is a FAIL — not a pass — because
  "an untracked world and a healthy world produce byte-identical evidence".
  [`headless_runner.py:648-690`](../../scripts/dev/headless_runner.py#L648-L690)
- The **event-type list is generated from the code, not the docs.** The
  simulator regex-scans its own emit call sites and reports drift as
  `undeclared` / `declared_not_emitted` on `GET /capabilities`.
  [`event_bus.py:109-122`](../../projects/default/controllers/harness_supervisor/event_bus.py#L109-L122)
- `GET /capabilities` **publishes what the simulator refuses to do**, with a
  reason and a workaround per gap.
- `GET /sim/grips` in light mode returns `tracking.enabled=false` with a
  machine-readable `reason` and `workaround`, and emits a `world.warning` —
  rather than an empty list that reads as "nothing gripped".
- [`determinism-scope.md`](../benchmarks/determinism-scope.md) **publishes its own
  refutation**: the false-positive that briefly inflated our determinism claim,
  and the three defects that produced it. Its standing rule — *"an assertion
  that has never gone red should be assumed broken until you make it go red on
  purpose."*
- `/debug/read_bench` measures read cost on **your** session, so a bug report
  quotes its own machine instead of ours.

## 7. What is actually true — the honesty gate

From the 2026-09-14 audit, carried over intact. **Do not widen any of this
without a measurement.**

### Strong — lead with these

| Capability | Status |
|---|---|
| Event stream, 11 types, two cursors, drop counters | REAL, code-verified against emit sites (`break.hit` is the eleventh, v9) |
| Contact / joint (`hit_limit`) / device / bounds inspection | REAL; each read internally holds the engine for the duration of its own walk, so a response is one consistent instant rather than a smear across steps. ⚠️ That guard is **per-read and internal** — you cannot hold it yourself; the verb that *you* hold is `POST /sim/pause`, below |
| **Pause held across calls** (`POST /sim/pause` / `/sim/resume`) | REAL as of v9, and **leased** — default 30 s, min 1 s, max 300 s, so a client that dies cannot freeze the engine. `POST /sim/step` while held is single-stepping. ⚠️ Measured 2026-09-22 (machine 9722d23d12a3): the step is exact on the **supervisor** clock (10 steps → exactly 80 ms) and **not** on the engine clock (80–112 ms over the same calls, 0–4 basic steps of overshoot), because lifting the pause, stepping and re-queuing the pause is a race the supervisor binding cannot close. The response **reports** `engine_advanced_ms` rather than asserting it. ⚠️ The pause shipped 2026-09-15 and **did not work** until 2026-09-22 — see the note under the table |
| **Break on event** (`POST /sim/break`, `GET /sim/breaks`, `DELETE /sim/break/<id>`) | REAL as of v9. An armed break takes the pause lease when a matching event fires; `break.hit` carries the match; `POST /sim/step` stops early on it and reports `steps_executed` + `stopped_on_break`, which makes it continue-to-breakpoint. ⚠️ **Two latency regimes, and neither is sub-step.** Held + `/sim/step`: 0 supervisor-ms, `hold_latency_steps` 0, at most one basic step of engine time. FREE-RUNNING: one **supervisor tick**, and a tick is not a basic step — measured from 8 ms to about 600 ms of engine time depending on load. On the three-body `break_drop` fixture the engine outran the controller so far that a whole one-second drop fitted inside one tick and no `contact.began` was emitted at all (`fired: false`). **The reliable workflow is the debugger one: pause, then step.** |
| **A break that could never fire is REFUSED** | REAL, and it is the headline honesty behaviour rather than a limitation. Light mode is the default and silences 5 of the 11 event types; a break armed on a silenced type comes back `400 BREAK_EVENT_TYPE_UNAVAILABLE` with an `event_type_silenced_in_light_mode` diagnostic naming the producer, the silenced set and the `{"light": false}` workaround. The refusal is **scoped**: `damage.*` survives light mode and is accepted in the same session. A refused break is not armed. |
| CPU-path determinism (`newtonSolver "mujoco"`, the default) | REAL — bitwise on contact-rich 10-robot scenes across cold launches, and across two machines running the **same binary** |
| CI verdicts (`--until-finalized`, `--fail-on-runaway`, `doctor`) | REAL, and rigorous — they fail rather than pass on thin evidence |
| 46 MCP tools — 42 one call each to the same harness a human uses, plus `robot_prompt` / `robot_tool` / `robot_state` / `robot_events` on a robot's OmniLink bridge | REAL as of v9. The command surface is reachable from an MCP client without curl; the four bridge tools go through the gated paths, never the ungated REST verbs. ⚠️ `robot_events` reads the **robot's** ring, not the harness's `/sim/events` — two rings, two cursors — and it dies with the bridge process |
| **The robot raises events of its own** (`GET /events` on a bridge, cursor-paged) | REAL as of v9, verified live on a Husky 2026-09-22 (machine 9722d23d12a3): the ring pages and fills, a gate refusal landed on it at sim t=4.688, and `GET /sim/state` carries the totals. Eight bridge-raised types — motion timeout, motion unsettled, joint limit, two fault kinds, gate refusal, contact — plus six forwarded from the harness when one is attached. Every detector was disabled in turn and its tests confirmed to fail, so none of them is an assertion that has never gone red. ⚠️ Contact events are **top-level scope only**: the supervisor's contact query cannot see a URDF robot's sub-links. ⚠️ The ring lives in the bridge process and dies with it — this is a live stream, **not a recording** |
| **An event can wake the agent, bounded** | REAL as of v9. A physical or safety event dispatches one turn through the same cascade and the same gate as an operator sentence. Measured: a burst of 100 qualifying events produces **1** wake and records 99 suppressed; disabling it produces none while the ring still fills. No OmniKey still means no model turn, and now also no wake. **Verified live end to end on 2026-09-23:** one timed-out drive woke the Husky, which answered *"It looks like my attempt to drive forward timed out after covering 1.2 meters of the requested 5.0 meters"* — the figures the event carried — and a burst of twenty inside the window produced no second turn (`tests/benchmarks/trackd_loop/wake_live.py`). ⚠️ The wake rate on an idle scene has **never been measured**, so the default policy is bounded by a limiter rather than by evidence |
| **Sim time on every measured result, and a stall distinguished from a timeout** | REAL as of v9, verified live: `/state` reports `sim_time` and `last_tick_at` in **sim seconds** where `last_tick_at` used to carry a wall-clock epoch, with the wall clock moved to its own field. Waits are counted in simulation steps. A frozen world can never expire a step budget, so a bridge reports `stalled` rather than `timed_out` — an agent told "timed out" reissues commands into a simulation that is not running |
| **One safety vocabulary across the simulator and the platform** | REAL as of v9, and the guard is now self-proving: the fixture is generated from `gate.py`, publishes its own registration table built by the same function that teaches the gate, and the generator refuses to write unless every case replays from that table and agrees under all 24 registration orders. ⚠️ Worth stating plainly because it is why the guard exists: the platform's copy **was found stale at 64 cases** while the source had moved to 78, so the parity test had been passing against an old table. ⚠️ A surface arriving over HTTP is **accepted and ignored** — the rail is the bridge's own, because a caller able to declare itself a drone would buy the 120 m altitude rail on a quadruped |
| **The action journal follows the agent** | REAL as of v9. ⚠️ Surviving a *restart* was never the new part, and an earlier draft of this row said it was: the local file has done that since 2026-07-22, shipped in the journal's own first commit precisely because an in-process-only record produced a worse fabrication — a robot citing an empty history as proof a drive never happened. What v9 adds is crossing a **machine**: the newest 50 entries (≤3 kB) ride in the memory write and are restored before the next session's first turn, on any machine holding the same key. ⚠️ The sync is a **30 s beat plus a flush on clean close, not a commit** — a hard kill between beats loses up to one beat, only the newest 50 entries travel, and no OmniKey means no relay and a local-only record. It does not require a completed model turn; that dependency was the shipped defect, found on the first live check and fixed 2026-09-22. This is the surface that removed a measured 26% fabrication rate |
| Structural fault injection into a named part | REAL (see limits below) |
| Native URDF / CAD import, 20+ vendor robot packages | REAL — this is what "build a digital twin" rests on; see the §5 boundary on what a twin is |
| One portable control surface, sim ↔ real, and a ROS 2 sidecar speaking `simulation_interfaces` | REAL **for the software above the driver only.** ⚠️ This row was widened on 2026-09-17 by owner decision, and this is the boundary that came with it: the shipped bridges run against mock drivers, and no policy trained here has been validated on hardware. Prior wording was "hardware is roadmap, not product" — the correction is that the *seam* ships and is documented; the *driver* and its validation do not. |

### The gap — state it, do not hide it

**Time control.** As of v9 OmniSim gives you the **debugger's** half of time
control — hold, single-step, break on an event. It still gives you none of the
**recorder's** half.

- **No watch conditions.** A watch is a polled predicate over pose or joint
  state ("break when `HUSKY.z < 0.1`"), which needs a per-step evaluator over
  state the trackers do not publish as events. Declared on `GET /capabilities`
  under `not_supported` as `sim.watch` / `WATCH_NOT_IMPLEMENTED`, with the
  workaround: break on the nearest event type, or hold the pause and poll the
  predicate yourself against a scene that is not moving underneath you.
  (Owner decision D7: out of scope for v9.)
- **No record / replay / run-diff** as a product surface.
- **Snapshot is not a checkpoint.** `POST /sim/snapshot` saves poses + joint
  angles only. Velocity is never captured — `OmSolid::saveHiddenFieldValues()`
  is an empty function
  ([`OmSolid.cpp:5448-5453`](../../src/omnisim/nodes/OmSolid.cpp#L5448-L5453)) —
  and solver state is never touched. Restore teleports bodies to saved poses
  while they keep their live Newton velocities, and zeroes only the *reported*
  velocity ([`OmSolid.cpp:4878-4894`](../../src/omnisim/nodes/OmSolid.cpp#L4878-L4894)).
  It does not rewind the clock. **Identical forward evolution is not
  achievable and must never be claimed.**

**The loop to OmniLink, and what is not yet true of it.** v9 makes the
simulator and the platform one loop rather than a tether, and three parts of
that must not be over-claimed.

- **The platform's half is written and not deployed.** The handoff that makes
  OmniLink give the operator's sentence to the robot — so the deterministic
  parser runs on the web door too, not only on the robot's own — ships in the
  OmniLink repository and needs a deploy. Until then the web door behaves
  exactly as it did before, and the two-door comparison is **unrun**. Do not
  describe the platform as parser-first in anything customer-facing until the
  sweep has measured it on both doors.
- **Lockstep is opt-in, covers three of four robot classes, and reproduces a run
  only for a client that waits on it.** Measured 2026-09-22 (machine 9722d23d12a3,
  CPU solver, four parser-answered commands, three runs each): free-running gave
  three different results, the robot finishing 0.99 m and 1.98 m from identical
  input; with the hold on and a client pacing on the bridge's `held` flag, all three
  runs were **identical**. With the hold on and a naive velocity-paced client they
  still diverged, because a held robot reads zero velocity while the world is frozen.
  So the claim is conditional and must be stated with its condition. It is
  reproducibility of a run, not replay, and nothing is claimed for commands a model
  chose. The drone declares lockstep unsupported rather than shipping a hold that
  would stop commanding its rotors. Rigs and evidence: `tests/benchmarks/trackd_loop/`.
- **There is no window-raise verb** in the robot-window protocol. A wake makes
  itself visible in the chat panel; nothing raises a window, and the platform's
  wake request must not be described as doing so.

**The unlock, and which releases it spans.** Two of the four gaps above were
closed on 2026-09-22 by exposing one primitive — a *leased* pause the caller
holds across calls — and building the breakpoint on it. Be precise about the
dates, because "the pause exists" and "the pause worked" are different claims:

- `0aafab096` (2026-09-15) shipped `PauseLease`, `pause_lifted` and
  `step_or_hold`, with no test, no documentation and no MCP tool. **It did not
  work.** Seven defects: it reported a frozen clock over a still-moving engine
  (`simulationSetMode` only *queues*, and a held pause has no round trip to
  carry it); a lease expiry could hang the supervisor permanently (the engine
  and the controller each waiting on the other); a held lease survived a
  `/world/load`; and break detection lagged up to 17 basic steps.
- 2026-09-22 fixed all seven and covered them with **43 engine-free tests plus
  four live cases** (`tests/harness/test_break_on_event.py`,
  `test_break_integration.py`). Only from here is "OmniSim has a working pause"
  a true sentence.

**What has not moved.** Record, replay and run-diff need the checkpoint gap
closed first, and it is not: velocity is never captured, so two runs cannot be
made to evolve identically.

### Other limits that must be stated where relevant

- **Light mode is the default and silences 5 of the 11 event types**
  (`contact.*`, `grip.*`, `joint.limit_hit`). `/sim/contacts` still answers.
  Load with `{"light": false}` for a debugging session — and note that arming a
  break on a silenced type is **refused**, not quietly accepted.
- **The two clocks on `GET /sim/state` are different rulers.** `sim_time_ms` is
  the injected supervisor's per-iteration counter, and it is what stamps every
  other harness response. `engine_time_ms` is the engine's own clock as of that
  controller's last step. In `--mode=fast` the engine runs far ahead — measured
  2026-09-22, about 700 engine-ms inside one supervisor tick on a three-body
  world. `engine_time_ms` does not rewind on `/sim/reset`. `paused` is `null`,
  not `false`, when nobody could be asked.
- **Controller logs and physics events cannot be put on one timeline.** Log
  events carry `t_wall` (Unix epoch), supervisor events carry `t_sim_ms` (sim
  ms), and `seq` collides across the two sides.
- **`/robot/<def>/sensor/<name>` is a deliberate 501.** Cameras, lidar and IMUs
  are not readable through the debug surface.
- **A green `run-headless` is a log verdict, not a physics verdict** — "a
  hologram floor lets a body reach z = −69 km and still PASS".
- **Fault injection is structural/impact damage only.** No sensor dropout, no
  encoder drift, no actuator degradation, no latency, no thermal derating. It
  binds to a robot named `husky` by default and idles silently otherwise.
  `WHEEL_TORQUE_SCALE` is written into `customData` for a cooperating
  controller to honour — **the motor is never touched.**
- **GPU determinism is refuted**, not unmeasured: 0 of 24 contact-rich
  `mujoco_warp` pairs were bitwise.
- **Hardware: the control surface ships, the driver does not.** See the widened
  row above and the §5 boundary. MAVLink HIL exists for aircraft; bus discovery
  and real-robot diagnostics are designed, not shipped. Our own
  [sim-to-real guide](../guide/omnilink-sim-to-real.md) states that no policy
  trained here has been validated on hardware, and that stays until measured.
  Say "before it reaches hardware" only with that boundary stated nearby.
- **macOS is not supported**, and the accessibility promise in §4 must never be
  quoted without the platform truth in §5 nearby.

## 8. Audience — widened at the front door, unchanged at the bench

The primary user is still an **AI coding agent, with a human directing it**.
That is genuinely ours and we keep it — [AGENTS.md](../../AGENTS.md) is still the
entry point, and the workshop is a workshop *for agents*.

What the repositioning changes is **who the human is allowed to be**. The debug
line addressed an engineer who already had a robot, a controller and a bug. The
workshop framing addresses the learner, the builder, and the person with an idea
and no hardware — and it tells them the agent is how they cross the gap.

- Was: an agent that talks to a simulator.
- Then: an agent that debugs robot software.
- Is: **an agent with a workshop — where it can build, program, test and debug a
  whole robotic system, for a person who could not do it alone.**

Keep "agent-native". **Do not claim "first".**

## 9. The proof — and where the numbers come from

**Sequencing is proof → docs → outreach.** Ship the demo first, then let the
docs describe something that visibly exists. Reversing the order turns a
repositioning into a claim we have to defend instead of demonstrate.

**The proof behind the program/test/debug pillar shipped:**
[`projects/samples/demos/worlds/debug/`](../../projects/samples/demos/worlds/debug/)
— two worlds differing in exactly two lines (a title and `newtonGroundMu`),
[`scripts/dev/diagnose_drop.py`](../../scripts/dev/diagnose_drop.py), and 14
engine-free tests in
[`tests/test_drop_detector.py`](../../tests/test_drop_detector.py).

Measured on `9722d23d12a3` (RTX 3060 laptop, Windows 11), `main @ 9bf6e006b`:
faulty **3/5** carried → fixed **5/5**; 30.0 s cold-start to root cause; two
sweeps in separate processes agreed to 0.1 mm on every row.

⚠️ **"two times in five" is a measurement, not a figure of speech.** Wherever
that phrase survives — the user guide, the comparison paper — it is there
because the demo drops 2 of 5 payloads. If the demo's numbers change, those
sentences change with it.

⚠️ **Three of the other four pillars do not yet have a demo of this calibre.**
Import, twin-building and the sim↔real seam are all real and all shipped, but
none of them has a two-worlds-one-variable artefact with a measured
before/after. Until they do, describe them from what the tree contains and the
§5 boundaries — never from a number nobody measured.

⚠️ **CORRECTED 2026-09-21: the talk-to-it path now HAS one, and this page said
it did not.** The sentence above used to name four pillars, the talk-to-it path
among them. The langsoak pair is exactly the artefact it said was missing —
the same held-out corpus, the same judge, the same robot, one variable (the gate
wired or not), measured before and after: **22.0% of "must not move" cases
actuated, down to 2.4%**
([`RESULTS_2026-09-20.md`](../../tests/benchmarks/langsoak/RESULTS_2026-09-20.md)).

Recording the correction rather than quietly editing the count, because the
failure mode this page exists to prevent runs in *both* directions. A page that
under-claims is wrong in a way that is comfortable to leave alone, and leaving
it alone is how it stops being read as a live document. **This does not widen
the claim** — the boundaries in §5 bind unchanged, and one measured artefact on
one robot is not a demo of the debug pillar's calibre.

**The debug demo also carries an open defect, on purpose.** At HEAD the repaired
arm shows **28.34 mm** of carry drift.

⚠️ **This page's first account of that defect was wrong in two ways**, and the
corrections are kept here rather than quietly edited away, because §6 has to
apply to this page too:

- **It is not measured against 4.242 mm.** That figure is the *recipe's*
  provenance from a **different scene** — ladder T2, 2026-08-03 — and the world
  header says so plainly. This world's own last recorded drift is **20.55 mm
  with `ok=FALSE`** (2026-08-22). It has never been under 20 mm, and the
  failing assertion is not new.
- **It was not "unreported".** Commit `17f003e7f` (2026-09-10, URDF inertia
  tensors reaching the solver) measured the widening to 0.0283 m and wrote it
  in its own commit message. The first account called it unreported because it
  had not looked there.

**Localised:** `OMNISIM_URDF_USE_INERTIA=0` returns 19.32 mm (−32%);
`NEWTON_INERTIA_COM=0` is worse (36.06 mm) and `ROBOT_GEOM_INERTIA=0` is
bitwise inert. ⛔ **Do not revert `17f003e7f`** — before it, declared inertia
tensors never reached the solver at all, so it is a correctness fix and the
drift is the truth it exposed. The stale artefact is the demo's **grasp
recipe**, tuned in August 2026 against wrong arm inertia: it commands ~5.0 N
per pad against a 0.123 N Coulomb bound. Re-tune the grip and re-record the
reference against *this* world.

## 10. Note: outreach

The startup-tier outreach pivot — *run their robot first, and the finding must
be behavioural* — is the program/test/debug pillar in the field. It stays
exactly as it is: that is an agent using the workshop on somebody's real project
and telling them what is wrong, which is the clearest demonstration of §4 we
have. What widens is who else we can now address — the learner and the person
with an idea, who were previously not in the sentence at all.


## Measured assessment extension — 2026-10-29

The reference-controller assessment now includes Go2 walk/stand/walk transitions
and G1 physical standing. [Protocol, source and evidence](../guide/legged-assessments.md).
Go2 support/contact checks combined with a shorter blend changed a fall at
14.272 s to a completed 32 s trial; G1 ankle trim changed a fall at 1.728 s
to standing through 32 s and held-out 48/60 s checks. Measured on machine
`9722d23d12a3`, Newton/MuJoCo CPU, 16 ms observations, eight Go2 / four G1 substeps.
These are bounded simulated reference-controller results, not manufacturer
firmware or hardware validation. G1 standing has no weight-bearing harness;
the existing authored walking demonstrations still use one. No unassisted
humanoid-walking claim is added. Windows has the only prebuilt package; Linux
requires a source build. Public v9.2.7 needs the separately published source
add-on for these assessment commands.

Go2 prospective-limit audit: after freezing the candidate, a new 9 s request passed and a 15 s request fell at 17.232 s. The 11/13 s timings were already used during development. Describe the shown 12 s improvement as bounded, with that remaining failure; never call it a general transition fix. The complete protocol has 18 valid episodes, 14 PASS / 4 FAIL.


## Go2 handover repair — 2026-10-29, assessment v1.1.0

The prior 15 s failure is repaired within the tested reference fixture: tightening
handover body tilt from 0.25 to 0.12 rad, while retaining phase/contact checks and
one-tick blending, changes a fall at 17.232 s to completion of both handovers
through 32 s. The model, learned policy, actuator gains and physics are unchanged;
paired compiled solver models match exactly. All 14 candidate trials passed,
including seven prior timings and four prespecified new timings
(6.25/9.45/12.35/14.75 s). The full protocol has 17 valid episodes, 14 PASS / 3 FAIL;
the FAIL cases reproduce the previous guard. The candidate also demonstrates
>=5 s actual standing, >=3 s return walking and >=0.5 m return progress.

Machine `9722d23d12a3`, Newton 1.5.0 / MuJoCo 3.11 CPU, 16 ms observations,
configured Go2 substeps 8, flat/contact μ 2.0, no added payload or disturbance.
The earlier study is retained as history. This establishes the listed simulation
results, not a universal handover solution or hardware validation. Windows has
the prebuilt engine; Linux requires a source build. G1's separate standing result
and its authored walking-demo harness boundary remain unchanged.

## Husky mixed rigid surfaces — 2026-10-29

The [mixed-surface module](../guide/husky-mixed-surfaces.md) measures braking
and 90-degree turning across full-width and split friction patches. Its frozen
76 episodes include 48 grid, eight uniform-floor controls, eight confirmations
and 12 prespecified new-input trials: 50 PASS / 26 FAIL, zero ERROR. In the
selected flat, unloaded μ 0.8→0.04 crossing, lowering approach/task speed from
0.6 to 0.35 m/s changes forward braking excursion from 0.458 to 0.156 m and
maximum path error from 0.963 to 0.109 m, within unchanged task requirements.
Paired compiled physical models match. Both task pairs repeat; five of six
capped prospective cases pass. Seven capped cases still fail, all split-surface
turning. Do not describe the speed cap as a general controller repair, free
performance improvement, continuous safe envelope or hardware result.

Machine `9722d23d12a3`, Newton 1.5.0 / MuJoCo 3.11 CPU, 16 ms observations,
eight configured substeps. Reference feedback driver, immediate braking, no
external force or body pose/velocity write. Rigid contact coefficients are
imposed assumptions; this study does not qualify wet flooring, water, mud,
deformable soil or other real materials. Public v9.2.7 needs the source add-on;
Windows has the prebuilt engine and Linux requires a source build.


### Split-surface repair qualification — 2026-10-29

The separate 93-episode repair addresses all seven v1 split-left-turn failures
without relaxing the original path/goal/heading/24-second requirements. It
adds a 0.30 m approach corridor for both variants. All seven matched endpoints
repeat twice, and all 27 unique previously passing capped inputs still pass.
New coefficients 0.025/0.065/0.14/0.30 crossed with loads 2.5/7.5/17.5 kg:
11/12 candidate inputs pass, all 12 old-driver inputs fail. The remaining
candidate input (mu 0.30, 17.5 kg) exceeds the approach upper speed bound:
0.420247 versus 0.420000 m/s. It remains FAIL. Overall 59 PASS / 34 FAIL,
zero ERROR; candidate 59/60 PASS. Source and plan were frozen before evaluation.

The repair combines known-map pre-alignment and route shaping, independent
wheel limits, a 0.18 m/s post-trigger target and stopped endpoint completion.
Both use the same nominal 0.35 m/s approach cap, but trigger pose/heading differ.
Do not present this as an isolated controller-gain fix or equal-throughput win.
On the selected low-side mu 0.04 unloaded split, maximum path error changes
1.095 to 0.254 m. Same machine/runtime/substeps as above. Physical models match
between each pair. Known map and localization assumed; no friction estimation,
production firmware or physical-robot validation. Rigid coefficients are not
calibrated real materials. The original 76-episode study remains unchanged.

## OmniSim Manufacturing — what may be claimed (v10, 2026-11-04)

Manufacturing is the v10 theme: machine twins that make parts, and robots made
with them. It sits under the "build digital twins" pillar (§5), and the same
boundary binds it: a twin here is **a model built from the machine's own
controller config and, where the vendor publishes it, its CAD**. Each line below
is a measurement with its source; quote it with its conditions or not at all.
All were measured on machine `9722d23d12a3` (Ryzen 7 5800H, RTX 3060 Laptop,
Windows 11), Newton 1.5.0 / MuJoCo CPU.

### Lead with these — measured, in the twin

| Claim | Measurement and conditions | Source |
|---|---|---|
| Three machine twins run the job file the real machine would run | CNC: a virtual GRBL 1.1 on TCP that a GRBL sender connects to (homing, a 12-line job, `ALARM:2`, reset, jog `error:15`, 2026-10-31). Laser: GRBL 1.1 laser mode (M3/M4, G0 dark, GRBL's PWM). Printer: G-code from an installed CuraEngine, planned as Klipper plans it | `projects/manufacturing/cnc/README.md`, `laser/README.md`, `printing/README.md` |
| Material is removed or laid where the tool really went | `MillingStock` carves by the cutter Solid's real pose every physics step; `PrintedPart` lays beads along the nozzle Solid's real path | `src/omnisim/nodes/OmMillingStock.*`, `OmPrintedPart.*` |
| The CNC twin cuts within microns of a perfect machine on the same program | star pocket: floor −0.002 … +0.0013 mm, walls 0.006 / 0.007 mm, path deviation max 0.026 mm; 70 mm mould within −0.012 … +0.024 mm of the program's ideal cut, volume −0.04 % (LongMill twin, 2026-11-01) | commits `1cb471380`, `0036355aa`; `cnc/README.md` |
| The laser twin is calibrated on the vendor's table | one row fitted (3 mm plywood, 3 passes); predicted 6 mm in 5 passes (Sienci: 5) and 1/4" in 14 (Sienci: 16); a 30 × 20 mm tag cut free at 29.9 × 19.9 mm, hole 6.06 mm (drawn 6.0) | commit `477d324eb`; `laser/README.md` |
| The printer twin prints a part and measures it | 20 mm cube: 2852.27 mm³ extruded, 2850.18 laid, 100 layers, outer surface +0.041 mm, height 20.05 mm; axis tracking 9 µm median (SV08 twin, 2026-11-02) | commit `710623c3a`; `printing/README.md` |
| An ideal (engine-free) print agrees with the twin | three parts, ideal vs twin: plastic laid identical to 0.1 mm³, same layers and height; the twin's part larger by 0.02–0.03 mm a side on the cube and plate, 0.003 mm on the shell's long walls; a 145 g shell in 21 s instead of ~1.9 h | `printing/README.md` "Ideal print (no engine)" |
| Two robots were designed, made on all three twins, assembled and driven on Newton, with a release pack each | desk robot: decks 90.1 × 80.1 mm (drawn 90 × 80), hubs OD 15.994 (16.0), 0.672 m/s straight vs 0.679 datasheet. SparkRover Mini: 325 solids with 0 interfering; 16 of 16 laser/CNC parts cut free; 27 printed parts on the corrected bead rule (2 on the full twin, 25 in ideal mode, their full-twin re-runs on the fixed engine to follow); 127 of 128 wires routed; 53 units assembled in 18 steps with 0 blocked; 0.6941 m/s straight vs 0.6964 predicted at the 8.184 V bus mean | `products/desk_robot/README.md`, `products/spark_rover/release/REPORT.md` |
| From AI concept images to a buildable design | the SparkRover's concept sheets were AI-generated; the design corrected their numbers and records each decision (no gripper, a plain RGB camera, a 2S2P battery) in `skeleton.py` `DECISIONS`; `release/` carries drawings, STLs, the machine files, the BOM and an assembly guide | `products/spark_rover/skeleton.py`, `docs/guide/manufacturing-concept-to-robot.md` |
| An operator talks to the CNC twin and it machines | live run 2026-11-01, LongMill twin, model through OmniLink: a spoken 50 mm × 2 mm circular pocket milled by `cut_pocket`; pause, a status question and resume mid-job; a 15 mm-deep request in 12 mm stock refused by `cut_depth` with the 11.5 mm limit explained; removed volume 6,201.3 mm³ (design about 6,164). Needs an OmniKey; `/tool` and Stop work without one | `projects/manufacturing/cnc/README.md` "Talk to it (OmniLink)", `cnc/cnc_bridge.py` |
| A load test found a real design fault | the clothes rack's concept 500 mm base tipped at 9.4 N empty on Newton; the 600 mm base raises it to 11.5 N (hand calculation 10.9) | commit `10cf4514a` |

### The boundary — state it wherever Manufacturing appears

- **Simulation only.** No twin has been validated against its real machine,
  and nothing has been made on a real machine. Say "the twin cuts / prints / measures";
  never "the machine will", and never quote a twin number as a real part's.
- **Twin and ideal are different results.** A twin print is full physics in the
  engine; an ideal print has no engine and no axis dynamics. Name the mode with
  every print number. On the corrected bead rule, 25 of the SparkRover's 27
  printed parts are ideal-mode results so far and 2 are full-twin results.
- **Calibration is one point.** The laser is fitted to one row of Sienci's table,
  and on full parts the twin needed **6 passes for 6 mm plywood, one more than
  Sienci's 5** (at 5 the parts stayed joined by a 0.16–0.20 mm web). Do not say the
  laser twin matches the vendor's table.
- **What is not simulated:** cutting load on acrylic (no published value: volume is
  removed with no load, melt or cracking model); the SparkRover's microphones
  (`Microphone` is retired), speaker, touch pads and status ring (visual meshes);
  an emissive screen; and the screws (placed at their seats, not simulated).
- **The bead model was corrected in v10, and twin numbers say which rule made them.**
  Before the fix, `PrintedPart` laid plastic held over a creeping nozzle as one bead up
  to ~7 mm wide, or as a dome the nozzle later "crashed" into. The fixed rule caps a bead
  at the nozzle face and lays the rest as a blob (CHANGELOG v10.0.0). A twin print
  measured before the fix is labelled so in its PRINT.md: never quote its p95,
  footprint or crash count as current.
- **Open design items stay visible:** the SparkRover's board hole positions on
  U5, U6, U11 and U12 are assumed; the front carrier needs about 64 g of support for a
  75 g part; the spin rate and stopping distance depend on the engine's
  wheel-armature default; the desk robot's assembly settle check is flaky.
- **Licences.** The LongMill CAD pack is CC BY-SA and the SV08 mesh pack GPL-3.0,
  each in its machine's `cad/` folder. Vendor STEP files for bought parts (Pololu,
  Seeed, ROBOTIS) are fetched and hash-checked at build time and never committed;
  the shipped models are ours, drawn from the vendors' dimension drawings.
- **No price or throughput comparison with a real machine,** which no twin has been run against, and no "first" or "only".
  "In seconds instead of hours" is true of ideal versus twin printing, measured
  above, and of nothing else.
