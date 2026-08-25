# The agent-native API: what we actually have, what we are missing, and what to build

**Status:** analysis + proposal. 2026-07-26.

> **⚠ 2026-08-08 — EVERY ODE NUMBER AND EVERY ODE ESCAPE HATCH IN THIS DOC IS HISTORICAL.**
> `bdc02139` deleted the vendored ODE library; Newton with `SolverMuJoCo` is the only physics
> backend and `OMNISIM_FORCE_ODE` is warned about and ignored. So doc-wide: every `ODE` column,
> every "under ODE it costs X", and the G1 mitigation *"it completes under ODE"* are **preserved
> verbatim as 2026-07 measurements and are UNREPRODUCIBLE** — "pick ODE" is no longer an option
> an agent, a benchmark or a reader can take. ⚠ Worse than unavailable: an explicit
> `physicsBackend "ode"` pin still *wins* and resolves to an inert stub, so such a world **still
> loads and still answers the harness while simulating nothing at all**, with no FATAL, no ERROR
> and no warning. The step-cost problem G1 describes is therefore **unmitigated by backend
> choice**; `{"light": true}` and `/capabilities` → `limits.step_cost` are the real levers.
> Record: [ode-retirement-campaign.md](ode-retirement-campaign.md).

> **UPDATE, same day — P1, P4 and most of P3 are now implemented and measured.**
> `GET /capabilities` (P1), `POST /scene/spawn` / `/scene/delete` /
> `/scene/set_pose` (P3), and `POST /sim/snapshot` / `/sim/restore` plus a
> **repaired `/sim/reset`** (P4, closing G2) ship in
> [`scripts/harness/omnisim_harness.py`](../../scripts/harness/omnisim_harness.py) +
> [`harness_supervisor.py`](../../projects/default/controllers/harness_supervisor/harness_supervisor.py).
> Contract: [PROTOCOL.md §7.28–§7.32](../../PROTOCOL.md); how-to and the
> hard-won rules: [scripts/harness/README.md](../../scripts/harness/README.md).
> G1's `--light` mitigation also became reachable over HTTP
> (`POST /world/load {"light": true}`, commit `06a0e23d`).
>
> Three things in the proposals below turned out to be **wrong**, and they are
> the interesting part:
>
> 1. **P3 cannot spawn a `URDFRobot` from a node string, and no amount of
>    harness code fixes that.** `URDFRobot { url ... }` is a *source*
>    expansion performed by `OmTokenizer::tokenizeFile`; the supervisor's
>    import path is `tokenizeString`, which never expands it, so
>    `OmParser::protoNodeList()` treats `URDFRobot` as a PROTO and
>    `OmNodeOperations::importNode` refuses it as not `IMPORTABLE`. The same
>    refusal hits all 261 in-tree PROTOs unless the loaded world declares
>    them. The working answer is `{"clone": "<DEF>"}`, which re-imports the
>    engine's own `Node.exportString()` output — already expanded, no second
>    URDF importer to drift from. So the flagship task needs **one** authored
>    robot in the container world, not zero.
> 2. **P4's stated implementation risk was the wrong risk.** `loadState`
>    *does* re-sync Newton body poses (verified: 0.0 m pose delta after a
>    2.83 m displacement). The real trap is that the map behind it
>    default-constructs a **zero vector** for an unsaved name, so an
>    unguarded restore teleports the scene to the origin — and that a
>    supervisor-taken "initial" snapshot is not the authored state at all,
>    because the engine free-runs before the controller's first step. The
>    authored state already exists engine-side as `"__init__"`.
> 3. **The flagship "before/after" table (§4) overstates the win as pure wall
>    time.** Measured on the same machine and session: the hand-authored
>    10-Husky world cold-loads in **9.2 s** (light mode), while building the
>    same scene from a one-robot container took **12 calls / 10.4 s** under
>    Newton and **4.7 s** under ODE. The real wins are that the agent writes
>    **zero** per-entity VRML (2.4 kB of the 3.7 kB file), that a bad entity
>    is a 0.3 s `422` carrying the rejected text instead of a whole-file
>    reload, and that *incremental* edits stop costing a load at all.
>
> P2 (step observability budget, `return: ["poses"]`, per-request
> `timeout_s`) and P5 (`/world/validate`, `/world/generate`) remain
> unimplemented; `/capabilities.not_supported` lists them with workarounds.

**Scope:** the World Harness surface (`scripts/harness/omnisim_harness.py`, `:6789`) and
its MCP projection ([`packages/omnisim-mcp/`](../../packages/omnisim-mcp/)), measured
against the [ROS 2 `simulation_interfaces`](https://github.com/ros-simulation/simulation_interfaces)
standard (Apache-2.0; implemented natively by Gazebo, Isaac Sim and O3DE).

The question this document answers: **which parts of our agent surface are genuinely
agent-native, and which are ordinary simulator CRUD that three other simulators already
ship behind a committee-blessed standard?** The answer is not flattering in every column,
and the parts that are not flattering are the useful parts.

> **This document is adversarial about our own surface on purpose.** Where a claim in
> [PROTOCOL.md](../../PROTOCOL.md) or
> [DRIVEABILITY.md](../../tests/benchmarks/omnibench/lane3/DRIVEABILITY.md) turned out to
> be wrong under measurement, it is corrected here and the correction is flagged.

---

## Contents

- [0. How this was measured](#0-how-this-was-measured)
- [1. The capability matrix](#1-the-capability-matrix)
- [2. The honest thesis — and where it breaks](#2-the-honest-thesis--and-where-it-breaks)
- [3. The gap list, prioritised by agent impact](#3-the-gap-list-prioritised-by-agent-impact)
- [4. Concrete API proposals](#4-concrete-api-proposals)
- [5. What we should NOT build](#5-what-we-should-not-build)
- [Appendix A. Measured latency tables](#appendix-a-measured-latency-tables)
- [Appendix B. Spec-vs-code drift found while writing this](#appendix-b-spec-vs-code-drift-found-while-writing-this)

---

## 0. How this was measured

Every latency and behaviour claim below was produced on **this machine, this session**, or
cited from a recorded OmniBench lane-3 row. Nothing is estimated.

| | |
|---|---|
| Machine | `9722d23d12a3` — RTX 3060 Laptop GPU (driver 596.36), AMD Ryzen 16-core, Windows 11, Python 3.12.9 |
| Engine build | `806b055c`, `msys64/mingw64/bin/omnisim-bin.exe` sha256 `f95073f976323787` |
| libController | `lib/controller/Controller.dll` sha256 `7e4650efd75970a4` |
| Harness | started on non-default ports: `python scripts/harness/omnisim_harness.py --port 6889 --supervisor-port 6890` |
| Backend attribution | via the `.newton.json` sidecar next to the harness's engine log. Newton runs: `{"backend":"newton","degraded":false,"finalised":true,"solver":"XPBD(iters=10)"}`. ODE runs: sidecar absent (started with `OMNISIM_FORCE_ODE=1`), which is the documented proof ODE drove it. ⚠ **2026-08-08: this attribution methodology is unrunnable, and its failure is silent.** `bdc02139` deleted ODE and `OMNISIM_FORCE_ODE` is now warned about and ignored, so the ODE arm of every A/B in this doc cannot be reproduced. An absent sidecar no longer means "ODE drove it" — it means either the run never reached world-finalize **or** the world ran on the inert no-physics stub (an explicit `"ode"` pin still wins; an *absent* Newton runtime is also silent — only an installed-but-broken one FATALs). Gate anything you intend to trust with `OMNISIM_REQUIRE_NEWTON=1` and assert the sidecar's presence. |
| Worlds | `tests/benchmarks/omnibench/lane3/worlds/lane3_drive.wbt` (3 bodies, 17 listed nodes) and a hand-authored 10-Husky scene (298 nodes) written to a scratch dir — **no repo file was created or modified.** |

The recorded lane-3 driveability rows referenced below are from `machine=9722d23d12a3` on the
*older* build `a74b7699`; where my numbers differ from theirs I say so.

---

## 1. The capability matrix

### 1.1 Their service → our endpoint

`simulation_interfaces` has 21 services. Here is where each lands on our surface.

| `simulation_interfaces` service | OmniSim harness | Verdict |
|---|---|---|
| `LoadWorld` | `POST /world/load` | ✅ **We are ahead.** Same verb, plus hot-reload of the same engine process and **33 structured diagnostic codes** (29 in the classifier's rule table + 4 the harness synthesizes: `PROTO_NAME_MISMATCH`, `WORLD_PARSE_SYNTAX_ERROR`, `LAUNCHER_DLL_NOT_FOUND`, …) an agent branches on instead of regex-matching stderr. The standard returns a single result enum. |
| `UnloadWorld` | — | ❌ **Missing.** No way to return the harness to an empty state without loading another world. |
| `GetAvailableWorlds` | — | ❌ **Missing.** The agent must `ls projects/samples/demos/worlds/` out-of-band. There are 7 omniworld recipes and **358** tracked `.wbt` files under `projects/` + `distribution/` (369 on disk, counting untracked WIP), and the API exposes none of them. |
| `GetCurrentWorld` | `GET /sim/state` → `world` | ✅ Equivalent (a path string). |
| `GetSimulationState` | `GET /sim/state` → `running` | ◐ **Partial and misleading.** `running` means "the engine subprocess is alive", not `STOPPED / PLAYING / PAUSED / QUITTING`. PROTOCOL.md §7.12 used to document `paused`, `sim_time` and `last_load` fields; **the endpoint returns none of the three** (verified). §7.12 was corrected on 2026-07-26 to list the real 13-field body. |
| `SetSimulationState` | — | ❌ **Missing.** We cannot pause, resume, or quit over HTTP. `Supervisor.simulationSetMode()` exists in the shipped controller binding and is not wired to anything. |
| `StepSimulation` | `POST /sim/step {steps}` | ◐ Present, **but see G1** — it is unusable at 10 robots. |
| `ResetSimulation` | `POST /sim/reset` | ⚠️ **Present but semantically broken.** It rewinds `sim_time` to 0 and **does not restore node state**. Independently re-verified this session on *both* backends (§3, G2). |
| `SpawnEntity` | — | ❌ **Missing.** The single largest gap. |
| `SpawnEntities` | — | ❌ Missing (batch form of the above). |
| `DeleteEntity` | — | ❌ Missing. `Node.remove()` exists in the binding; nothing calls it. |
| `GetEntities` | `GET /scene/tree`, `GET /robots` | ✅ **We are ahead.** We return the whole scene graph with parentage, not just spawned entities. |
| `GetEntitiesStates` | `GET /scene/tree` | ◐ Poses only. **No velocities** — `Node.getVelocity()` exists in the binding and is not exposed. Their `EntityState` carries a twist. |
| `GetEntityState` | `GET /scene/node/<def>` | ◐ Same caveat: pose yes, twist no. |
| `SetEntityState` | — | ❌ **Missing.** We can write exactly one field in the whole scene: `Viewpoint.position` / `.orientation`, via `/scene/look_at`. The generic `field.setSFVec3f(...)` machinery is right there in the supervisor and is used for nothing else. |
| `GetEntityInfo` | `GET /scene/node/<def>` + `/robot/<def>/devices` + `/robot/<def>/joints` | ✅ **We are ahead.** Full field dump, device inventory, and per-joint limits with a `hit_limit` flag. |
| `GetEntityBounds` | `GET /scene/tree?bounds=1`, `GET /scene/node/<def>?bounds=1` | ✅ **We are well ahead.** World-space AABB + centre + radius, an **`exact` flag** plus a `skipped` list naming any mesh the geometry walk could not parse, and an opt-in `?probe=1` oracle that recovers the engine's own bounding sphere by inverting `OmViewpoint::moveViewpointToObject`. Honest uncertainty is a feature no competitor ships. |
| `GetNamedPoses` | — | ❌ Missing. |
| `GetNamedPoseBounds` | — | ❌ Missing. |
| `GetSpawnables` | — | ❌ **Missing.** We ship **261** `.proto` files and a URDF importer, and there is no way to ask what can be placed. |
| `GetSimulatorFeatures` | — | ❌ **Missing, and this one is structural.** There is no capability-discovery endpoint of any kind. `GET /protocol` returns 404 on the harness (verified) despite PROTOCOL.md §4.1 specifying it. An agent must discover our surface by 404-probing it — which is literally what I did to write this table. |

**Score against the standard: 6 ahead, 5 partial, 10 missing.**

### 1.2 Our verbs with no counterpart in the standard — and what each buys an agent

This is the column that justifies the word "agent-native". For each, the test applied is:
*does this remove a round-trip, or remove a guess, from an LLM agent's loop?*

| Endpoint | What it buys an agent |
|---|---|
| **`POST /scene/frame`** ⭐ | Computes aim **and** distance from the subject's real geometric bounds, pushes it, and returns a `verification` block: `fits`, `headroom_h_deg`, `headroom_v_deg`, `subject_angular_radius_deg`. **The agent learns it aimed correctly without rendering an image.** The alternative loop everywhere else is guess-a-pose → screenshot → look at pixels with a vision model → guess again, at ~1–5 s and a few thousand image tokens per iteration. This is the single most differentiated verb we own — and it was **absent from PROTOCOL.md** until Appendix B caught it (now §7.21). |
| **`GET /scene/visible`** | Closed-loop aiming feedback: per node, frustum test, screen-space bbox in **pixels**, signed angular offset, and a natural-language hint (`"off-screen: 34 deg to the left, 12 deg up"`). Converts "the screenshot looks wrong" into a number the agent can act on. |
| **`POST /scene/orbit`** | The only *relative* camera verb. Everything else (ours and everyone's) is absolute, so "a bit more to the left" otherwise requires the agent to re-derive an absolute pose. |
| **`GET /scene/viewpoint`** | Reads the camera back — position, orientation, resolved horizontal *and* vertical FOV for the real viewport aspect, derived forward/up/right. Before this existed, every camera API wrote to a camera you could not read. |
| **`GET /world/render_stats`** | `mean_brightness`, `saturated_pct`, `black_pct` + warnings like `"blown out: 41% of pixels are saturated"`. **Catches a lighting bug as JSON**, so the agent does not need an image round-trip to discover the scene is black. |
| **`GET /sim/events`** | One cursor-paged stream merging supervisor-side events (`contact.began`, `joint.limit_hit`, `grip.acquired`, `damage.*`) with harness-side ones (`controller.log`, `world.warning`, `world.error`). Crucially it carries **`dropped_sup` / `dropped_log`** — the agent is told when it is polling too slowly instead of silently missing events. ROS's answer is "subscribe to five topics and correlate them yourself." |
| **`GET /world/diagnostics`** | Re-fetch structured load diagnostics without re-parsing. The codes are an **open enum** the client is told to degrade gracefully on. |
| **`GET /sim/contacts` / `GET /sim/grips`** | Global contact set and *inferred* grips (`gripper_def`, `held_def`, `since_t_ms`). "Is the robot holding the block" is a first-class question, not a physics-log excavation. |
| **`GET /robot/<def>/joints`** | Per-joint position, velocity, limits and **`hit_limit`** in one call, with no controller written. `hit_limit` is the difference between "the arm didn't reach" and "the arm hit a joint stop" — an agent's single most common manipulation bug. |
| **`GET /robot/damage/*`** | Structural damage state, impact events with impulse in joules, and a direct injection hook for testing. No standard has an equivalent. Also **was absent from PROTOCOL.md** (now §7.24–§7.27). |
| **`GET /healthz`** | Liveness without touching the simulator. Trivially useful; the standard has no non-blocking probe. |

### 1.3 Transport, honestly

`simulation_interfaces` is DDS. Ours is HTTP/JSON on loopback. The differentiator is **not**
that we have a programmatic surface and they do not — the 2026-07-10 edition of
[simulator-comparison.md](simulator-comparison.md) claimed that and §5.1 has since corrected
it. The differentiator is that driving Gazebo or Isaac through the standard requires a ROS 2
install and a DDS participant, and driving OmniSim requires `curl`. For a coding agent in a
sandbox, that is the whole game — but it is a *packaging* advantage, not a capability one,
and it evaporates the moment the agent's environment already has ROS.

---

## 2. The honest thesis — and where it breaks

### The thesis

> **`simulation_interfaces` is a control-plane CRUD standard: create, read, update, delete
> entities and worlds, plus run/pause/step. OmniSim's harness is weak on exactly that CRUD
> and strong on something the standard does not attempt: perception-and-verification verbs
> that let an agent close its own loop without a human, a GUI, or a vision model in the
> middle.**

The evidence is the shape of the two lists in §1. Every one of the ten services we lack is a
*mutation* or a *discovery* verb. Almost every verb we own that they lack answers a question
of the form **"did the thing I just did actually work, and by how much?"** — `fits` with a
headroom in degrees, `saturated_pct`, `hit_limit`, `dropped_sup`, `exact: false` with the
reason, a diagnostic `code` instead of a stderr line.

That is a real design position, and it is the right one for an LLM driver. An LLM cannot
tell that a camera is 12° off from a 512-token image, and it cannot tell that a load
"succeeded but the PROTO silently fell back" from free text. It can branch on a number.

### Where the thesis does not hold

Four places. All of them matter.

1. **Verification is worthless if the loop is too slow to run.** The strongest verb we own,
   `/scene/frame`, costs 4.8 s in the recorded lane-3 row and sits behind a `/sim/step` that
   costs **26.6 s for one 16 ms step** on a 10-robot scene (§3, G1). A closed loop you can
   run three times in a five-minute budget is not a closed loop. This is the thesis's biggest
   self-inflicted wound.

2. **We are not the perception surface for anything except the camera.**
   `GET /robot/<def>/sensor/<name>` returns **501 by design** — the supervisor cannot honestly
   read devices it does not own. That is architecturally correct and it is also a large hole:
   a Gazebo agent gets lidar, IMU, depth, joint states and camera as first-class ROS topics
   with `rosbag`, `rviz` and `ros2 topic echo` around them. Our answer is "write a controller
   and expose your own endpoint." **On non-camera perception we are behind, and the honest
   framing is that our verification verbs cover *scene geometry and load health*, not sensing.**

3. **"Perception verbs" is a thin moat.** `/scene/frame` is ~300 lines of framing math over
   `GetEntityBounds`. Any of Gazebo, Isaac or O3DE could ship it in a sprint on top of the
   bounds service they already have. What is defensible is the *composition* — bounds +
   framing + read-back + screen-space visibility + exposure stats + structured diagnostics as
   one contract — not any single verb.

4. **Some of our "verification" is not verified.** `/sim/reset` is documented as a reset and
   measurably is not one (G2). `/sim/state` reports `load_ok: true, load_state: "complete",
   running: true` while every scene endpoint returns 503 — those two fields are latched at
   load time and never re-evaluated (now documented in PROTOCOL.md §7.12; the G3
   cross-reference here was wrong, this is not a capability-discovery finding). PROTOCOL.md
   named four event types that the code does not emit, plus wrong payloads on three more
   (Appendix B). A surface that sells honest signals has to
   be held to a higher bar on this than one that does not.

---

## 3. The gap list, prioritised by agent impact

Ordered by how much each one blocks the flagship task: **"build a scene with 10 Huskies
moving randomly, verify they moved."**

### G1 — `/sim/step` is unusable at scale, and a slow step *destroys the session* 🔴 CRITICAL

Measured this session, same world, same supervisor, both backends:

| World | Backend | `/sim/step {1}` | marginal s/step | `/scene/tree` | `/robots` |
|---|---|---|---|---|---|
| lane3 (17 nodes) | **Newton** (default) | 0.86–1.22 s | 0.47 s | 1.02 s | 1.02 s |
| lane3 (17 nodes) | ODE | **0.018 s** | **0.008 s** | 0.14 s | 0.017 s |
| 10 Huskies (298 nodes) | **Newton** (default) | **26.6–27.1 s** | ~14 s | 23.0 s | 22.9 s |
| 10 Huskies (298 nodes) | ODE | 5.8–6.2 s | 2.28 s | 4.40 s | 4.27 s |

Three separate findings live in that table.

**(a) The recorded lane-3 attribution is incomplete.** DRIVEABILITY.md says the ~0.7–0.9 s
step cost is "the per-step damage/contact/grip polling in the injected supervisor, **not
physics**." On the identical world with the identical supervisor and identical polling, ODE
is **~60× faster**. So the polling is the *multiplier* (it issues O(nodes) supervisor IPC
round-trips per sim step) and the **Newton backend's per-round-trip cost is the base**. Both
are real; the doc names only one.

**(b) The polling is genuinely O(nodes) per step, and the mitigation that exists is never
applied.** In [`harness_supervisor.py`](../../projects/default/controllers/harness_supervisor/harness_supervisor.py)
the `step` command runs, *per inner 16 ms step*, `damage.poll()`, `contact_tracker.poll()`,
and `grip_tracker.poll(..., observe.build_robot_subtree_index(supervisor), ...)` — and
`build_robot_subtree_index` walks the **entire scene graph** through supervisor IPC on every
one of those steps. The supervisor already implements a `--light` flag that skips all three
producers, and `OMNISIM_DAMAGE_POLL_EVERY` to sub-sample damage. **The harness's
`SUPERVISOR_INJECT_STANZA` passes no `controllerArgs` at all**, so `--light` can never be
reached over HTTP, and the `damage_poll_every` sub-sampling is a local in `main()` that the
`step` command handler does not consult.

**(c) A step that exceeds 120 s permanently bricks the session.**
`SUPERVISOR_RPC_TIMEOUT_S = 120.0` is a module constant with no per-request override. On
timeout, `SupervisorClient.call()` calls `_drop_locked()` and closes the socket. There *is* a
reconnect path — but only for commands in `IDEMPOTENT_SUPERVISOR_COMMANDS` (which excludes
`step`), with a **5 s** deadline and a stability ping. On a heavy world one supervisor loop
iteration takes ~4–23 s, so the 5 s reconnect window **structurally cannot succeed there**.
Observed twice: `/sim/step {60}` → 503 after 120 s, and every subsequent read → 503
`"supervisor not connected"`, recoverable only by a full `/world/load` (28.5 s cold).

**Agent impact:** the flagship task is **not completable through `/sim/step` on the shipped
Newton default.** The largest step budget that fits under the timeout is ~8 steps (0.128 s of
sim time); seeing 10 Huskies displace 0.14 m needs ~30. That is 4+ sequential calls of ~100 s
each, every one a coin-flip against session death.

**It completes under ODE.** Full measured loop: `/world/load` 12.0 s → `/robots` 1.5 s →
`/sim/step {30}` **76.2 s** → `/robots` 5.1 s, client-side diff → **MOVED 10/10, 0.1418 m
each. ~95 s, 4 round-trips.** That is the honest current-state baseline for the benchmark.

### G2 — `/sim/reset` does not reset 🔴 CRITICAL

Re-verified independently this session on **both** backends. `BALL` is authored at `z = 1.0`;
after stepping it rests at `z ≈ 0.10` (Newton) / `z ≈ 0.149` (ODE); `POST /sim/reset` returns
`{"sim_time_ms": 0}` in 0.01–0.71 s and the ball **stays where it fell**.

So this is not a Newton bug — it is the surface's semantics. Engine-side,
`OmNewtonBackend::reset()` only calls `resetJointsToDefaults()` and delegates per-body pose to
"the Solid-side `syncNewtonPoseFromFields` signal cascade", which measurably does not land
through this path.

**Agent impact:** the only working state-reset primitive is a full world reload — 2.4 s on a
tiny world, **12–36 s on a 10-robot one**. Any experiment of the form "try N variations from
the same initial state" pays a reload per trial. `ResetSimulation` is one of the four verbs
every implementation of the standard gets right, and ours is a trap: it returns `ok` and does
nothing observable.

The fix is already in the shipped binding and unused: `Node.saveState(name)` /
`Node.loadState(name)` are real C entry points (`wb_supervisor_node_save_state` /
`_load_state` in `src/controller/c/supervisor.c`).

### G3 — No capability discovery; the surface is undiscoverable 🟠 HIGH

`GET /protocol` → **404**. `GET /capabilities` → **404**. There is no machine-readable
description of what this harness can do, what backend is running, what the timestep is, what
diagnostic codes exist, or what the per-step cost is likely to be.

Consequences an agent actually hits:
- It cannot tell Newton from ODE over HTTP — and that is a **60×** difference in loop cost
  (G1). Today the only honest answer is to read a `.newton.json` sidecar off the filesystem.
  ⚠ **2026-08-08: the Newton-vs-ODE ambiguity no longer exists** — `bdc02139` left Newton as
  the only backend. The **60×** stays on the record as a historical measurement of what the
  two backends cost, but it is no longer a *discovery* problem: what an agent still cannot
  learn over HTTP is the **per-step cost of the world in front of it** (which varies by
  solver, scene size and `--light`), and that is what `/capabilities` →
  `limits.step_cost` exists to answer.
- It cannot discover `/scene/frame` exists. AGENTS.md tells a *Claude Code* agent; an MCP
  client sees 18 tools; an arbitrary HTTP client sees nothing.
- It cannot size a `/sim/step` request against the 120 s timeout, because neither the timeout
  nor the per-step cost is published.
- PROTOCOL.md §16 already lists the missing `/protocol` as an open compliance gap. It has not
  moved.

`GetSimulatorFeatures` is the one service in the standard whose *absence* compounds every
other gap, because it is how a client learns which of the other gaps apply to it.

### G4 — No spawn / delete / set-state: the agent hand-writes `.wbt` text 🟠 HIGH

To put a robot in a scene an agent must author VRML by hand, write it to disk, and pay a full
world load. There is no `SpawnEntity`, no `DeleteEntity`, no `SetEntityState`.

This is **not** blocked by the engine. The shipped Supervisor binding already has
`Field.importMFNodeFromString(position, nodeString)`, `Node.remove()`,
`Node.setVelocity()`, `Node.addForce()`, `Node.addTorque()`, `Node.setJointPosition()`,
`Node.resetPhysics()`, `Node.restartController()`, `Node.exportString()` and
`Supervisor.worldSave()`. The harness supervisor's command table exposes **none** of them —
its only field write in the entire scene is `Viewpoint.position` / `.orientation`.

**Agent impact on the flagship task:** the 10-Husky scene was 3,864 bytes of hand-generated
VRML, and every mistake in it costs a 12–36 s load to discover. A `POST /scene/spawn` loop
would be 10 sub-second calls with per-entity error reporting, and `SpawnEntities` makes it one.

### G5 — omniworld is not exposed over HTTP, and its fleet primitive is Mars-only and capped at 8 🟠 HIGH

[`src/python/omniworld/`](../../src/python/omniworld/) generates deterministic worlds from 7
recipes (`flat_ground`, `indoor_apartment`, `mars`, `outdoor_desert`, `outdoor_forest`,
`urban_block`, `warehouse`) with `(recipe, seed, params) → byte-identical .wbt`. It is
reachable **only** as `python scripts/dev/omniworld.py` — it is not on the harness, not in the
MCP server, and not even a `python -m omnisim` subcommand.

Two specific findings:

- **Every recipe places at most one robot.** The common params are `spawn_urdf` (singular) +
  `spawn_controller` + `spawn_height`. There is no general "place N robots" parameter.
- **The only fleet primitive in the tree is Mars-specific and hard-capped at 8.**
  `mars` alone has `husky_count` / `husky_formation` (`circle` | `corners`) /
  `husky_spawn_radius` / `husky_corner_margin` / `husky_controller`, and
  `biomes/mars.py` enforces `_HUSKY_MAX = 8`. **The flagship "10 Huskies" task cannot be
  expressed by the one generator feature that comes closest to it**, and only on Mars.

  (Separately: [`newton_husky_swarm_drive.omniworld`](../../projects/samples/demos/worlds/physics/newton_husky_swarm_drive.omniworld)
  documents a *physics* reason for 8-not-10 under Newton XPBD. That is a different constraint
  from the generator cap, and it means the flagship task's robot count deserves an explicit
  decision rather than an accident.)

- **A free preflight is sitting unused.** `omniworld.validation.validate(path)` runs four
  static checks (`asset_locality`, `prop_overlap`, `spawn_reachability`, `viewpoint_framing`)
  on the `.wbt` text. Measured: **3.6 ms** on the 10-Husky world, **36 ms** on the lane-3
  world — against a **12–36 s** load. Nothing over HTTP calls it (G7).

### G6 — Every call is N round-trips; no batching, no transaction, no undo 🟡 MEDIUM

There is no compound endpoint. "Spawn 10 robots and confirm" is 10+ calls if spawn existed;
"snapshot poses, step, snapshot poses, diff" is 3 calls plus client-side math the agent
re-implements every time. On the flagship world each of those reads is 4.3 s (ODE) or 22.9 s
(Newton), so round-trip count converts directly into minutes.

There is also no undo and no transaction. An agent that spawns 9 robots and fails on the 10th
has no rollback; its only recovery is a full reload.

**Caveat, stated deliberately:** the standard does not have transactions either, and adding
them would make the surface stateful in a way §5 argues against. `SpawnEntities`-style
*batching* (one request, N items, per-item results, no cross-item atomicity) is the correct
scope.

### G7 — No dry-run / validate-before-load 🟡 MEDIUM

The only way to find out whether a `.wbt` is loadable is to load it. Measured costs of being
wrong:

- A **nonexistent path** returns `422` with `{"ok": false, "error": "world not found: ..."}` —
  free text, and **`diagnostics: []`**, even though `WORLD_FILE_NOT_FOUND` exists as a code in
  `diagnostic_codes.py`. The most common authoring failure produces no structured code.
- A **syntactically broken world** takes **243 s** to resolve (recorded lane-3 row): hot-reload
  attempt → cold engine relaunch retry → supervisor-bind stall detector. The synchronous
  response can be `ok: true, load_state: "in_progress"`, with the real codes only arriving via
  `GET /world/diagnostics`.

Four minutes to learn about a typo, when a 4 ms static check exists in-tree (G5).

### G8 — Harness errors do not use the protocol's own error envelope 🟡 MEDIUM

PROTOCOL.md §3.2 mandates `{"ok": false, "error": "<snake_case_code>", "message": ..., "details": ...}`.
Every harness error observed this session is `{"error": "<free text>"}`:

```
404 {"error": "not found: /protocol"}
400 {"error": "path is required"}
503 {"error": "supervisor RPC failed: timed out"}
503 {"error": "supervisor not connected (load a world with with_supervisor=true)"}
501 {"error": "live sensor reads not supported from the supervisor (…)", "robot": "...", "sensor": "..."}
```

§16 already admits this. The agent-facing consequence is that the *failure* path — the path an
agent spends most of its tokens on — is the one path with no machine-branchable code, in a
surface whose whole pitch is machine-branchable signals. The 501 is the sharpest example: it
is a deliberate, well-designed answer delivered as prose instead of `effector_unavailable`.

### G9 — No pause/resume; no velocities; no named poses 🟢 LOWER

`SetSimulationState` (pause/play/quit), twist in entity state, and `GetNamedPoses` are all
missing. Each is small on its own. Pause matters most: an agent that wants to inspect a scene
mid-motion has no way to freeze it, so every read races the free-running sim.

### G10 — Sensor reads are 501 by design 🟢 ACCEPT, DON'T FIX HERE

Correct architecture, real hole (§2, break #2). The fix is a per-robot bridge
([PROTOCOL.md §5](../../PROTOCOL.md)), not a harness endpoint — the supervisor genuinely
cannot read devices it does not own without lying. The actionable part is making the *answer*
machine-readable (G8) and pointing at the bridge in `/capabilities` (G3).

---

## 4. Concrete API proposals

Five endpoints. Ordered by unblock-per-line-of-code. Together, P1–P4 turn the flagship task
from "not completable on the default backend" into **three calls and about six seconds**.

### P1 — `GET /capabilities` (mirrors `GetSimulatorFeatures`, honestly)

The keystone: it is how an agent discovers every other fix. The design rule is that it must
publish our *weaknesses* too, because those are what an agent needs to plan around.

```http
GET /capabilities
```
```json
{
  "ok": true,
  "omnisim_wire": "1.1",
  "service": "world_harness",
  "sim_version": "5.1.0",
  "build": "806b055c",
  "machine": { "id": "9722d23d12a3", "gpu": "NVIDIA GeForce RTX 3060 Laptop GPU" },

  "physics": {
    "backend": "newton",
    "solver": "XPBD(iters=10)",
    "degraded": false,
    "source": "sidecar",
    "basic_time_step_ms": 16.0
  },

  "features": [
    "world.load", "world.validate", "world.screenshot", "world.render_stats",
    "scene.tree", "scene.bounds", "scene.frame", "scene.visible", "scene.orbit",
    "scene.spawn", "scene.delete", "scene.set_state",
    "sim.step", "sim.snapshot", "sim.restore", "sim.batch",
    "events.cursor", "robot.joints", "robot.devices", "robot.damage",
    "world.generate"
  ],
  "not_supported": [
    { "feature": "robot.sensor_read", "code": "effector_unavailable",
      "reason": "OmniSim restricts device APIs to the owning controller.",
      "workaround": "GET /robot/<def>/joints, or a Robot Bridge (PROTOCOL.md §5)." },
    { "feature": "sim.pause", "reason": "not implemented yet" },
    { "feature": "entity.velocity", "reason": "not implemented yet" }
  ],

  "limits": {
    "supervisor_rpc_timeout_s": 120.0,
    "max_steps_per_request": 4096,
    "scene_nodes": 298,
    "measured_step_cost_s": 14.2,
    "measured_step_cost_source": "rolling median, last 20 steps, this world",
    "recommended_max_steps_per_request": 8
  },

  "diagnostic_codes": ["WORLD_FILE_NOT_FOUND", "PROTO_NAME_MISMATCH", "..."],
  "event_types": ["contact.began", "contact.ended", "grip.acquired", "grip.released",
                  "joint.limit_hit", "damage.impact", "damage.state_transition",
                  "controller.log", "world.warning", "world.error"],
  "generators": { "recipes": ["flat_ground", "warehouse", "mars", "..."] }
}
```

**One call replaces:** 404-probing the route table; reading a `.newton.json` sidecar off disk
to learn the backend; guessing a step budget against an unpublished 120 s timeout; and
hard-coding the event-type list from a doc that is currently wrong (Appendix B). `event_types`
served **from the code** makes that class of drift impossible.

`limits.measured_step_cost_s` is the unusual field and the most valuable one: it lets an agent
*plan* — `steps = min(desired, floor(0.6 * timeout / cost))` — instead of discovering the
limit by bricking its session.

### P2 — `POST /sim/step` gains an observability budget (fixes G1)

Additive, backward compatible. Default behaviour changes only in that it stops being the
slowest thing on the surface.

```json
POST /sim/step
{ "steps": 30,
  "observe": "none",          // "none" | "contacts" | "full"  (default "contacts")
  "observe_every": 8,          // poll producers every Nth step (default 1 for "full")
  "timeout_s": 300,            // per-request override of the 120 s constant
  "return": ["sim_time", "poses"] }
```
```json
{ "ok": true, "sim_time_ms": 1136.0, "advanced_to_ms": 1136.0,
  "steps_executed": 30, "wall_ms": 640,
  "observe": { "mode": "none", "events_suppressed": true },
  "poses": { "HUSKY_0": [-6.86, -54.0, 0.31], "...": [] } }
```

Mechanism: pass `--light` (and an `--observe-every`) through the harness's
`SUPERVISOR_INJECT_STANZA` as `controllerArgs`, and thread the budget into the `step` command
handler so `build_robot_subtree_index()` stops running once per 16 ms of sim time. **The
supervisor already implements `--light`; only the plumbing is missing.**

Two hard requirements attached:

- **`timeout_s` must be per-request**, and a timeout must **not** drop the connection for a
  recoverable case. At minimum, extend the reconnect path to scale its deadline with the
  observed loop period instead of a fixed 5 s.
- **`return: ["poses"]`** collapses the near-universal step→read pattern into one call. On the
  flagship world that alone saves 4.3 s (ODE) / 22.9 s (Newton) per iteration.

Expected effect on the flagship task, from the measured ODE marginal cost (2.28 s/step, of
which the trackers are the dominant term) and the lane-3 world's ODE floor (0.008 s/step):
**76 s → low single-digit seconds** for the same 30 steps.

### P3 — `POST /scene/spawn` + `POST /scene/spawn_many` + `DELETE /scene/node/<def>` (fixes G4)

```json
POST /scene/spawn_many
{ "template": {
    "type": "URDFRobot",
    "url": "projects/robots/clearpath/husky_description/urdf/husky.urdf",
    "controller": "husky_random",
    "physicsBackend": "newton" },
  "pattern": { "kind": "grid", "count": 10, "spacing": [12.0, 12.0],
               "origin": [-7, -54, 0.3], "jitter": 0.0, "seed": 42 },
  "def_prefix": "HUSKY_",
  "verify": true }
```
```json
{ "ok": true, "spawned": 10, "failed": 0,
  "entities": [ { "def": "HUSKY_0", "position": [-7, -54, 0.3], "bounds": { "radius": 0.628 } } ],
  "verification": { "all_present": true, "overlaps": [], "settled": false },
  "wall_ms": 890 }
```

`pattern.kind` ∈ `grid` | `circle` | `line` | `corners` | `explicit` — deliberately the same
vocabulary `mars.husky_formation` already uses, promoted out of one biome and off the
`_HUSKY_MAX = 8` cap. `verify: true` runs the overlap check from
`omniworld.validation.overlap` against the *live* scene and returns which pairs collide.

Built on `Field.importMFNodeFromString()` + `Node.remove()`, both already in the binding. This
is the single largest reduction in agent effort on the surface: **it deletes VRML authoring
from the agent's job description.**

**Companion:** `POST /world/save {path}` (`Supervisor.worldSave()`) so a scene composed by
spawning can be persisted and re-loaded deterministically.

### P4 — `POST /sim/snapshot` + `POST /sim/restore` (fixes G2)

```json
POST /sim/snapshot   { "name": "t0", "scope": "world" }
→ { "ok": true, "name": "t0", "sim_time_ms": 0.0, "nodes": 298, "wall_ms": 120 }

POST /sim/restore    { "name": "t0" }
→ { "ok": true, "name": "t0", "sim_time_ms": 0.0, "nodes_restored": 298,
    "verification": { "max_pose_delta_m": 0.0, "exact": true } }
```

Built on `Node.saveState()` / `Node.loadState()`, walked over the scene root. The
`verification` block is the house style: it reports how far restoration actually landed rather
than asserting it worked — which matters because G2 exists precisely because a reset silently
did nothing.

`POST /sim/reset` should then either be **fixed to mean `restore("__authored__")`** or
**deprecated with a `410` pointing at `/sim/restore`**. Shipping a verb that returns `ok` and
does nothing is worse than not shipping it.

**This also goes beyond the standard.** `ResetSimulation` restores one state; *named*
snapshots let an agent do branch-and-compare — "try three grasp approaches from the same
initial condition" — which is the core loop of agentic experimentation and which currently
costs a 12–36 s world reload per branch.

⚠️ Must be validated against the Newton backend before shipping. `OmNewtonBackend::reset()`
today only re-FKs joints; whether `loadState` re-syncs Newton body poses is **unverified** and
is the main implementation risk in this proposal.

### P5 — `POST /world/validate` and `POST /world/generate` (fixes G5, G7)

```json
POST /world/validate
{ "path": "…/husky10.wbt" }              // or { "source": "<inline .wbt text>" }
```
```json
{ "ok": true, "loadable": true, "wall_ms": 4,
  "checks": [
    { "name": "asset_locality",     "status": "pass" },
    { "name": "prop_overlap",       "status": "pass" },
    { "name": "spawn_reachability", "status": "pass" },
    { "name": "viewpoint_framing",  "status": "pass",
      "note": "borderline: robot is centred but clipped by the frame edge" } ],
  "diagnostics": [] }
```

A thin wrapper over `omniworld.validation.validate()`, already in-tree. **Measured 3.6 ms
against a 12–36 s load** — a ~10³× cheaper preflight, and the natural place to also emit
`WORLD_FILE_NOT_FOUND` as a *code* instead of the prose 422 the loader returns today.

```json
POST /world/generate
{ "recipe": "warehouse", "seed": 42,
  "params": { "size_x": 40, "rack_rows": 4, "spawn_urdf": "…/husky.urdf" },
  "out": "generated/wh_42.wbt", "load": true, "validate": true }
```
```json
{ "ok": true, "path": "generated/wh_42.wbt", "sha256": "…",
  "manifest": "generated/wh_42.manifest.json",
  "validation": { "ok": true, "checks": [] },
  "load": { "ok": true, "load_ms": 6100, "supervisor": "connected" } }
```

Plus `GET /world/recipes` (→ `list_recipes()` + `describe`) and `GET /worlds` (→
`GetAvailableWorlds`, an index of the 367 in-tree `.wbt` files). Determinism is already
guaranteed by the library, so `(recipe, seed, params)` is a reproducible scene handle an agent
can put in a report.

### The flagship task, before and after

⚠ **Every `ODE` column in the tables below is HISTORICAL and UNREPRODUCIBLE** (`bdc02139`
deleted ODE; `OMNISIM_FORCE_ODE` selects nothing). The numbers are preserved verbatim as
2026-07 measurements. Read them as "what the deleted backend cost", never as an alternative an
agent or a benchmark can select today.

| | today (Newton default) | ⚠ historical (ODE) | with P1–P4 |
|---|---|---|---|
| Author the scene | hand-write 3,864 B of VRML | same | `POST /scene/spawn_many` — 1 call |
| Load | 28.5–36 s | 12.0 s | (spawn into a loaded empty world) |
| Snapshot poses | 22.9 s | 1.5 s | folded into the step call |
| Step 30 × 16 ms | **impossible** — 120 s timeout at ~8 steps, and the timeout kills the session | 76.2 s | `observe:"none"`, `return:["poses"]` — ~1 call |
| Verify | 22.9 s | 5.1 s | in the step response |
| **Total** | **not completable** | **~95 s, 4 round-trips** | **~6 s, 3 round-trips** |

---

## 5. What we should NOT build

⚠️ **This paragraph used to read "Do not build a ROS 2 bridge, and do not adopt DDS."** The
ROS 2 half was reversed on 2026-08-17 and the bridge exists:
[`packages/omnisim-ros2/`](../../packages/omnisim-ros2/). **The DDS half stands** — the engine
takes no ROS or DDS dependency, and the bridge is a sidecar over this same HTTP surface.

The corollary in the original text turned out to be exactly right, which is why the port was
cheap: the naming here deliberately *mirrors* `simulation_interfaces` semantics
(`GetSimulatorFeatures` → `/capabilities`, `SpawnEntity` → `/scene/spawn`), so the shim really
was a small job. Mirroring the vocabulary is free; taking the dependency **into the engine** is
not, and that is still the line. See [ros2-integration.md](ros2-integration.md).

**Do not put an LLM inside the simulator or the harness.** No `/prompt` on the harness, no
natural-language scene description, no "make this look nicer." The agent is the LLM. Our job
is to return numbers it can branch on. The moment the harness has an inference dependency it
stops being `curl`-able, which is the entire transport advantage from §1.3.

**Do not make the surface stateful in a way that breaks restartability.** Specifically:
- **No sessions, no login, no per-client server-side context.** Every endpoint must stay
  answerable from "the world that is loaded right now."
- **No undo stack.** P4's *named* snapshots are explicit, addressable, and client-driven;
  an implicit history is a mutable server-side timeline that two parallel agents would race on.
- **No transactions across endpoints.** `spawn_many` returns per-item results and no
  cross-item atomicity, deliberately. A failed 10th spawn is reported, not rolled back.
- **`limits.measured_step_cost_s` in P1 is telemetry, not state** — a rolling median with no
  semantics attached. It must never become an input to server-side behaviour.

**Do not proxy sensor reads through the supervisor.** The 501 is correct. Making it a lie to
be convenient would trade the one thing this surface is selling — honesty about what it knows
— for an endpoint that returns plausible-looking wrong numbers.

**Do not build streaming (WebSocket/SSE) yet.** PROTOCOL.md §15 already defers it and the
cursor-paged `/sim/events` with `dropped_*` counters covers every current need. Fix G1 first;
most of the perceived need for streaming is really the step-latency problem.

**Do not add a physics-parameter surface.** PID gains, solver iterations, friction tuning:
these belong in the world file, where they are reproducible and reviewable, not in a
mutable HTTP call that makes a run unrepeatable from its `.wbt`.

---

## Appendix A. Measured latency tables

All rows measured this session on machine `9722d23d12a3`, build `806b055c`, unless marked
*(lane-3)*, which are recorded rows from `results/driveability.jsonl` on build `a74b7699`.

### A.1 `lane3_drive.wbt` — 3 bodies, 17 listed nodes

| Call | Newton (XPBD, default) | ⚠ historical: ODE (`OMNISIM_FORCE_ODE=1`) | ratio |
|---|---|---|---|
| `POST /world/load` (cold) | 6.94 s | 2.40 s | 2.9× |
| `GET /scene/tree` | 1.02 s | 0.139 s | 7.3× |
| `GET /scene/tree?bounds=1` | 3.04 s | 0.069 s | 44× |
| `GET /robots` | 1.02 s | 0.017 s | 62× |
| `GET /sim/contacts` | 0.866 s | 0.026 s | 33× |
| `GET /sim/grips` | 0.687 s | 0.012 s | 59× |
| `GET /sim/state` | 0.0016 s | — | (never touches the sim) |
| `POST /sim/step {1}` | 0.86 / 1.22 s | 0.018 s | ~47–66× |
| `POST /sim/step {10}` | 5.30 s | — | |
| `POST /sim/step {50}` | 23.48 s (0.47 s/step) | 0.390 s (0.008 s/step) | 60× |
| `POST /sim/reset` | 0.710 s | 0.010 s | |

### A.2 10-Husky scene — 298 nodes

| Call | Newton (default) | ⚠ historical: ODE |
|---|---|---|
| `POST /world/load` (cold) | 28.5 / 36.1 s | 12.0 s |
| `POST /world/load` (hot) | — | 19.5 s |
| `GET /scene/tree` | 23.0 s | 4.40 s |
| `GET /robots` | 22.1 / 22.9 s | 1.5 / 4.06 / 4.27 s |
| `POST /sim/step {1}` | 26.63 / 27.09 s | 5.83 / 6.22 s |
| `POST /sim/step {5}` | 70.78 s (14.2 s/step) | 14.78 s (2.96 s/step) |
| `POST /sim/step {20}` | **timeout at 120 s → session dropped** | 45.51 s (2.28 s/step) |
| `POST /sim/step {30}` | — | 76.2 s |
| `POST /sim/step {60}` | **timeout at 120 s → session dropped** | **timeout at 120 s → session dropped** |

> **Addendum 2026-07-31 (post-`--light` re-measure — Phase R item 7).** The same scene
> class was re-measured on the same machine (`9722d23d12a3`, build `5dc31a84`, Newton
> `XPBD(iters=10)` non-degraded per sidecar) in both session modes, 3 trials per endpoint:
> `--light` **fixes** `/sim/step` (0.044 s), `/sim/events` (0.02 s), `/capabilities`
> (0.05 s), `/sim/state` (0.006 s) and `/robot/<def>/joints` (0.9 s), but the per-call
> scene-walking reads are **halved, not fixed** — `/scene/tree` 11.0 s (was 23.0 s),
> `/robots` 15.8 s, `/sim/contacts` 11.9 s, `?bounds=1` 19.5 s. Non-light sessions are now
> *worse* than the table above and **degrade with session age** (e.g. `/scene/tree`
> 17 → 51 s across three trials; even `/sim/state` costs 14–27 s while queued behind the
> saturated supervisor). Full trial table, verdicts, and the campaign implication:
> [`harness-latency-2026-07-31.md`](harness-latency-2026-07-31.md). The rows above are the
> historical pre-`--light` baseline and are kept unchanged.

### A.3 Recorded lane-3 driveability rows *(build `a74b7699`)*

| Probe | Pass | Latency |
|---|---|---|
| `load_valid_world` | ✅ | 8.49 s |
| `hot_reload_edited_world` | ✅ | 4.25 s |
| `scene_tree_poses` | ✅ | 1.56 s |
| `scene_tree_bounds` | ✅ | 3.04 s |
| `sim_step_deterministic` | ✅ | **85.06 s** (2 × [reload + 50 steps + read]) |
| `events_cursor_stream` | ✅ | 0.91 s |
| `robot_joints_state` | ✅ | 1.13 s |
| `scene_frame_verified` | ✅ | 4.80 s |
| `screenshot_png` | ✅ | 1.24 s |
| `broken_world_structured_diagnostic` | ✅ | **242.86 s** |

### A.4 Static validation vs. loading

| World | `omniworld.validation.validate()` | ⚠ historical: `POST /world/load` (ODE cold) |
|---|---|---|
| `lane3_drive.wbt` | **36.1 ms** | 2,400 ms |
| 10-Husky (298 nodes) | **3.6 ms** | 12,000 ms |

### A.5 Reproducing

```bash
# Windows: the harness needs the system Python (Pillow) and mingw64 on PATH.
OMNISIM_HOME=o:/omnisim \
OMNISIM_LOG_PATH=/tmp/h.log \
PATH="/o/omnisim/msys64/mingw64/bin:$PATH" \
  /c/Users/<you>/AppData/Local/Programs/Python/Python312/python.exe \
  scripts/harness/omnisim_harness.py --port 6889 --supervisor-port 6890

# ODE control run -- ⚠ NO LONGER RUNNABLE (bdc02139). It was:
#   prefix OMNISIM_FORCE_ODE=1 and confirm no /tmp/h.log.newton.json appears.
# ODE is deleted and OMNISIM_FORCE_ODE is now warned about and ignored. An absent
# sidecar no longer proves ODE drove anything -- it means the run never finalized, or
# it ran on the inert no-physics stub. THE ODE COLUMNS ABOVE CANNOT BE RE-DERIVED;
# only the Newton columns reproduce. Run the Newton arm under OMNISIM_REQUIRE_NEWTON=1
# and assert the sidecar, since an ABSENT Newton runtime degrades silently.
```

---

## Appendix B. Spec-vs-code drift found while writing this

[PROTOCOL.md](../../PROTOCOL.md) declares itself normative and says a disagreement with the
code "is treated as a wire-protocol bug, not a documentation bug." By that rule, the following
are bugs. They are listed here because an agent that trusts the spec gets silently wrong
answers — which is the exact failure mode this surface exists to prevent.

> **STATUS: all six items below were fixed in PROTOCOL.md on 2026-07-26** (§7 endpoints
> added, §10 event names corrected, real response bodies documented). This appendix is kept
> as the record of what drifted and why. **Three of the six were also under-reported here** —
> re-verifying against the code found more than the first pass did, which is itself the
> argument for P1: a hand-audit of a hand-maintained list finds most of the drift, not all of
> it. The under-reports are marked ⊕ below.
>
> Note also that the internal `codebase-audit-2026-07.md` (not in the public snapshot) §9.2 and §9.12 had
> already found the payload-level event drift and the `limit` default before this document
> was written; neither had reached PROTOCOL.md.

**1. Eight implemented harness endpoints are absent from §7** — including `/scene/frame`, the
most differentiated verb on the surface:

`GET /scene/viewpoint` · `POST /scene/frame` · `POST /scene/orbit` · `GET /scene/visible` ·
`GET /robot/damage` · `GET /robot/damage/events` · `POST /robot/damage/reset` ·
`POST /robot/damage/inject`

(All eight *are* in AGENTS.md §5 and most are in the MCP server, so AGENTS.md is currently
more accurate than the normative spec.)

**2. Four of twelve documented event type names do not exist.** §10 names them; the code emits
different strings. Because `GET /sim/events?types=` is an **exact-match allowlist**, an agent
filtering on the documented names gets an empty stream and no error:

| PROTOCOL.md §10 | Actually emitted |
|---|---|
| `grip.began` | `grip.acquired` |
| `damage.applied` | `damage.impact` |
| `damage.state_changed` | `damage.state_transition` |
| `damage.part_detached` | *(not emitted by the harness supervisor)* |

⊕ **Under-reported: the names were only half of it.** Re-verification found the *payloads* of
three surviving types were wrong too, and the §10 preamble's one universal requirement was
never implemented at all:

- **`t` does not exist.** §10 required every event to carry `t` (float sim-seconds). Supervisor
  events carry `t_sim_ms` (int ms); harness log events carry `t_wall` (float Unix epoch).
  Different clocks, neither named `t`. Differencing them silently yields nonsense.
- `joint.limit_hit` emits `{joint, side, position, lower, upper}` — **no `robot_def`** (the
  emitter explicitly skips the owning-robot lookup), and the band field is `side`, not
  `direction`. In a multi-robot scene the event cannot be attributed to a robot.
- `controller.log` emits `{stream, line}` — no `robot_def`, and the message field is `line`,
  not `text`. The harness reads the engine's merged stdout/stderr and *cannot* attribute a
  line to a robot.
- The damage payloads differ as much as the names: `part` not `target_def`, `impulse_J` not
  `impulse_j`, and a single post-transition `hp` rather than `hp_before`/`hp_after`.
- `contact.began` never carries the optional `normal_force` the old §10.1 (now §10.2) listed.
- `?limit=` defaults to **256** (clamped to `[1, 1024]`), not the 100 §7.19 documented.
- Ten types are emitted, not twelve: `fault.raised` / `fault.cleared` (§10.7) have **no
  producer anywhere in the tree** either, and were not marked reserved.

**3. `GET /sim/state` does not return the `paused` field §7.12 documents.** Verified: the body
is `world, running, exit_code, load_ok, load_ms, load_state, load_started_at,
load_completed_at, supervisor_connected, supervisor_connected_at, supervisor_bind, binary,
webots_home`.

⊕ **Under-reported: three of the five documented fields are absent, not one.** §7.12 specified
`{world, running, sim_time, paused, last_load}`; `sim_time` and `last_load` are missing too.
Only `world` and `running` were ever real — and `running` means "the engine subprocess is
alive", which is the §1.1 caveat.

**4. `/robot/<def>/sensor/<name>` returns 501 with a prose `error`, not the
`effector_unavailable` code §7.16 specifies.** Already listed in §16; unchanged.

**5. `POST /world/load` on a missing path returns `422` with free text and `diagnostics: []`**,
though `WORLD_FILE_NOT_FOUND` exists in `diagnostic_codes.py`.

**6. The capture service (`:6791`) serves `/world/robots`, `/world/subject` and `/shutdown`,
none of which appear in §8.**

⊕ **Under-reported: four routes, not three.** `GET /healthz` is undocumented in §8 as well
(and unlike the harness's, it returns an `ffmpeg` path-or-`null` — the field that predicts
whether a `/capture/sequence` will be able to encode). `/world/robots` is also registered on
**both** GET and POST with identical behaviour.

**7. Not in the original pass at all** — found on re-verification, and now documented in §7:

- **Response bodies routinely lack the `ok` field the §7 examples showed.** Only `/healthz`
  and `POST /world/load` return one. Every supervisor-backed endpoint returns the supervisor's
  raw dict verbatim, so `/sim/contacts`, `/sim/grips`, `/robots`, `/sim/events`,
  `/world/render_stats`, `/sim/step` and `/sim/reset` have no `ok` despite §7.6/§7.10/§7.17/
  §7.18/§7.19 all showing one. (Permitted by §3.1 — but the examples were still wrong.)
- **`/world/render_stats` is on the 0–255 scale, not 0–1.** §7.6's example showed
  `mean_brightness: 0.42` and `max_rgb: [1.0, 1.0, 1.0]`; the harness returns 8-bit channel
  statistics. A client thresholding `mean_brightness < 0.1` for "the scene is black" matches
  nothing. The example's `warnings` array was also impossible: it showed a warning firing at
  12.3% near-black, against a real threshold of 60%. Undocumented `width`/`height`/`pixels`
  are always present.
- **`/sim/step` returns `{sim_time_ms, advanced_to_ms}`**, not `{ok, sim_time}` — different
  key *and* different unit (ms, not s).
- **`/scene/tree` returns `{nodes, bounds_included}`**, not a bare array, and its `?bounds=1`
  parameter — the input to every framing decision — was undocumented, as was
  `/scene/node/<def>?bounds=1&probe=1`.
- **A concurrent `/world/load` returns a distinct `422` with `load_state: "busy"`**, and a slow
  load can return `{ok: true, load_state: "in_progress"}` before it has finished, with the real
  diagnostics arriving later via `/world/diagnostics`.

Recommendation: rather than hand-patching §7 and §10, serve the endpoint list, the diagnostic
codes and the event types **from the code** via `GET /capabilities` (P1), and generate the
spec tables from that. Drift of this kind cannot be fixed once; it has to be made structurally
impossible. **The 2026-07-26 pass is the hand-patch, and the ⊕ marks above are the evidence
that a hand-patch is not enough**: the first audit of this same surface, done carefully and
with the code open, still missed roughly a third of the drift.
