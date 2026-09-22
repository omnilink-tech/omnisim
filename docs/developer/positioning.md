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
> digital twins, connect simulation to real robots, and give AI agents a
> workshop to program, test, and debug your system.
>
> You can do all of that simply by talking to it.
>
> **No robot?** Simulate one.
> **Already have a robot?** Build its digital twin.
> **Don't know how to program it?** Let an agent help build and debug it.
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
