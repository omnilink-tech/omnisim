# scripts/harness/ — agent-facing validation harness

Long-running HTTP service on `127.0.0.1:6789` that wraps a headless OmniSim subprocess and injects a generic supervisor controller into whatever world it loads. Lets a coding agent author and iterate on `.wbt` files in a tight loop — load → screenshot → inspect scene tree → adjust camera → check exposure → hot-reload — without ever launching the desktop GUI.

| File | Purpose |
|---|---|
| [`omnisim_harness.py`](omnisim_harness.py) | The HTTP service. Run directly or via `python scripts/dev/omnisim_dev.py harness`. |
| [`diagnostic_codes.py`](diagnostic_codes.py) | Free-text-stderr → structured-code mapper used by `/world/load` and `/world/diagnostics`. Anchored in real `OmLog::error` / `OmLog::warning` call sites; unmatched lines pass through as `code: "UNKNOWN"`. |
| [`spatial.py`](spatial.py) | Camera framing / orbiting / screen-projection math behind `/scene/frame`, `/scene/orbit`, `/scene/visible`, `/scene/viewpoint`. Pure functions; loads `src/python/omniworld/viewpoint.py` **by path** so there is exactly one framing implementation in the tree. |

The supervisor controller that the harness injects lives at [`projects/default/controllers/harness_supervisor/`](../../projects/default/controllers/harness_supervisor/). Tests live at [`tests/harness/`](../../tests/harness/).

## Runtime notes

- **HTTP/1.1 keep-alive (2026-09-01).** The server sets `protocol_version = "HTTP/1.1"`; every response carries `Content-Length`, so pooled clients (the ROS harness client, the MCP wrapper, any `http.client` user) reuse one TCP connection. Measured on the flip (229 requests): connection reuse 0/229 -> 229/229, `TIME_WAIT` growth +221 -> +6, `GET /healthz` 5.09 -> 0.31 ms. Unhandled handler exceptions now come back as a coded `500 HARNESS_INTERNAL` (with the connection closed) instead of an empty reply, and every 4xx/5xx body carries a machine-branchable `code`.
- **Per-tracker toggles (2026-09-01, public issue #4).** `POST /world/load` accepts `{"tracking": {"contacts": false, "joint_limits": false, "grips": false}}` -- each `false` drops exactly that tracker, instead of the all-or-nothing `light`. `contacts: false` implies grips off (GripTracker consumes ContactTracker's pairs). The load response's `tracking.mode` reads `light`/`partial`/`full`, and `GET /capabilities` -> `event_types_detail.suppressed` names exactly the quiet types for the running combination. Measured: partial mode (contacts+grips off, joint-limits on) steps at light-mode cost (~10 ms vs ~600 ms full on the 309-node fleet arena).
- **`GET /debug/read_bench?n=N` (diagnostic).** Measures the cost of one supervisor read on THIS session, free-running vs paused, plus whether the pause engaged -- the number every inspection endpoint's cost is built from. After the engine's immediate-burst fast path (2026-09-01, `OMNISIM_IMMEDIATE_BURST`, `=0` reverts), a read costs ~0.6-0.9 ms on the fleet arena where it cost 3.4-7.4 ms before; `/scene/tree` 4.4 s -> 0.09 s, `/robots` 7.1 s -> 0.10 s, `/sim/contacts` 4.5 s -> 0.06 s (light session, same world, same machine).


- **Pillow** is required by `/world/render_stats` (503 without it; the harness prints a startup hint). Install it into whichever interpreter runs the harness: on Windows use the system Python (full path — a bare `python` resolves to the msys2 one once mingw64 `bin` is on `PATH`); on Linux `pip install Pillow` into the launching `python3` — Ubuntu 24.04+ (PEP 668) needs `--break-system-packages` or a venv. The harness itself runs fine from a venv; it is only the *engine's embedded interpreter* (for the Newton runtime) that must be the system `python3`, not a venv — don't conflate the two.
- **Cold-load times vary a lot with disk speed.** The "~1 s empty, ~6 s asset-heavy" figures quoted in AGENTS.md §5 are measured on a local NVMe Windows machine. On WSL2 / virtualized / network disks the same asset-heavy world (e.g. `warehouse_husky.omniworld`) has measured **46–79 s** on a cold load. A `/world/load` that takes a minute on such a setup is a slow disk, not a hang.
- **`wait_s` bounds how long `POST /world/load` blocks — not the supervisor bind window.** If the caller's `wait_s` expires while the engine is alive and still loading, the response is `{"ok": true, "load_state": "in_progress", "supervisor": "load_in_progress: ..."}` and a background waiter keeps polling for the supervisor: progress-aware (it extends while the engine log / stdout are still growing), with a 300 s hard ceiling from launch and a 30 s no-progress stall detector past the caller's window. Poll `GET /sim/state` until `supervisor_connected: true` (it also carries `load_state` and a `supervisor_bind` block), or re-POST the same path to block again — a repeat load of the *same* world joins the in-flight bind instead of killing the loading engine. A stalled or over-ceiling engine is terminated cleanly and reported as a `SUPERVISOR_BIND_STALLED` / `SUPERVISOR_BIND_CEILING` diagnostic, leaving the harness ready for a retry.
- **All responses are strict JSON (RFC 8259).** Non-finite floats coming out of the engine (uninitialised transforms can carry NaN) are sanitized to `null` at the HTTP boundary — no bare `NaN` / `Infinity` tokens, so non-Python clients can parse `/scene/tree`.
- **Read endpoints pause the engine for the duration of their walk** (`518a335e`, 2026-07-31): `/scene/tree`, `?bounds=1`, `/robots`, `/sim/contacts`, `/robot/<def>/joints|devices`, `/scene/node` and the camera reads take a paused **single-instant snapshot** and restore the prior mode on the way out. ⚠️ **The numbers this bullet used to quote as current are HISTORICAL (July 2026, pre-burst engine):** "a round-trip costs ~6 ms free-running vs ~0.15 ms paused (40×)", and the pause-attributed medians on the 298-node 10-Husky stress scene (`/scene/tree` 11.0 s → 0.11 s, `?bounds=1` 19.5 s → 0.19 s, `/robots` 15.8 s → 0.14 s, `/sim/contacts` 11.9 s → 0.13 s) — they measured the pause against an engine whose per-read cost was dominated by a Qt event-loop wakeup per packet. Since the engine's **immediate-burst fast path** (2026-09-01, `ff8477451`; `OMNISIM_IMMEDIATE_BURST=0` reverts), a supervisor read costs **~0.6–0.9 ms pause-independent** (fleet arena; 3.4–7.4 ms before), and the pause is kept because it buys a **consistent snapshot**, not speed. For live numbers on *your* session, ask `GET /debug/read_bench` (runtime note above). The sim still free-runs *between* calls. ⚠️ Still true: a **non-light** session on a multi-robot scene queues every RPC behind the per-step trackers' own free-run walks, so the "load `{"light": true}` on multi-robot scenes" rule (or the per-tracker toggles above) stands. History and mechanism: [docs/developer/harness-latency-2026-07-31.md](../../docs/developer/harness-latency-2026-07-31.md).
- **`OMNISIM_LOG_PATH` is honoured.** When set in the harness's environment, both the harness's diagnostics reader and the spawned engine use that file (the harness pins the child's `OMNISIM_LOG_PATH` to the file it watches), so parallel harnesses don't fight over the shared `omnisim_log.txt`.

## Capability discovery: `GET /capabilities`

One call, and the answer to "what am I talking to, what is driving the physics, what will a step cost, and what will it refuse to do". Start every session with it.

| Field | Why an agent needs it |
|---|---|
| `physics` | **Which backend drove the run, read from the engine's own verdict** — the `<engine-log>.newton.json` sidecar `OmNewtonBackend::finalizeWorld` writes (`source: "sidecar"`, with `solver`, `degraded`, `finalised`). Provenance labels: `engine_log`, `sidecar_stale` (the sidecar predates this load → backend *unverified*), `sidecar_absent` (a load that never reached finalize — a short run proves nothing), and `forced_by_env` (`OMNISIM_FORCE_ODE` / `OMNISIM_LEGACY`). ⚠️ **`forced_by_env` is now a WARNING, not a backend.** Both vars are RETIRED — src/ode was deleted (commit bdc02139) — so the response also carries `forced_ode_env_retired` spelling out that the var does not give you an ODE run, it gives you a world the engine never builds physics for (measured 2026-08-08: a forced run left a body bit-identical to its authored pose for 3000 ms). Unset it. The old ODE-vs-Newton step-cost contrast this row used to quote (1.15 s vs 0.0025 s on a 10-robot scene) is historical; for the live lever see `light` below. |
| `limits.step_cost` + `recommended_max_steps_per_request` | A rolling median of the **measured** per-step cost on the world that is loaded, cleared by every load, plus `floor(0.6 × supervisor_rpc_timeout_s / cost)`. This is how you size a step budget instead of discovering the 120 s RPC timeout by killing your session. `?probe_step=1` advances one step to measure it if nothing has been measured yet. Telemetry only — nothing server-side branches on it. |
| `event_types` + `event_types_detail` | The authoritative ten types, **served from the code**: the supervisor scans its own `emit()` call sites and cross-checks them against `event_bus.SUPERVISOR_EVENT_TYPES`, the harness does the same for its three log types, and `verified` / `undeclared` / `declared_not_emitted` come back in the response. `suppressed` names the types a `--light` session does **not** produce — `?types=` is an exact-match allowlist, so filtering on one of those returns an empty stream, not an error. |
| `endpoints` + `endpoints_verification` | The route table, cross-checked against the request handler's own source. A route added to `do_GET`/`do_POST` and not declared shows up as `undeclared_literals`. |
| `not_supported` | Every gap with a `reason` and a `workaround` — sensor reads (501 by design), pause, velocities, `world.validate` / `world.generate` / `world.save`, batch spawn, and the URDF-spawn constraint below. Session-specific entries appear too: a `--light` supervisor lists `sim.grips` + the suppressed contact/grip/joint **event** types. It does **not** list `/sim/contacts` — that endpoint walks the scene per call (`observe.collect_contacts`) and never reads a tracker, so it answers identically in light mode. Claiming otherwise sent agents into a ~790×-cost reload they did not need. |
| `supervisor.commands` | The live supervisor RPC vocabulary, scanned from `dispatch()`. |
| `diagnostic_codes` / `request_error_codes` | The load-diagnostic enum (from `diagnostic_codes.py` plus the four the harness synthesizes) and the machine-branchable `code` values on 4xx bodies. |

It never needs a supervisor: with no world loaded it answers with `supervisor.connected: false` and the three log event types.

⚠ **A `physics` block on a session with NO WORLD LOADED is not evidence of anything, and it used to read as if it were.** MEASURED 2026-08-12: `/capabilities` on a freshly started harness that had never loaded a world returned a full attestation — `backend: "newton"`, the solver named, a **50-body census** — with `source: "sidecar"` at `sidecar_age_s: 85.6`. The engine log and its `.newton.json` sidecar are files on disk left by whatever ran last (a `run-headless`, a previous harness, another lane), and the documented contract is that the sidecar's presence means *"Newton drove **this** run"* — so every provenance claim built on that response was void. Both the verdict and the census are now gated on this session having actually started a load: with none, `physics.source` is `sidecar_stale` (or `sidecar_absent`), `physics.backend` is `unverified`, and `physics.bodies.source` is `no_world_loaded`. The pre-existing "sidecar predates the current load" check now also drops `backend` to `unverified` instead of continuing to assert `newton` off a verdict it has just called stale.

## Mutating the live scene: spawn, delete, set_pose

These replace "hand-write `.wbt` text, then pay a full load to find out whether it parsed". All three are thin wrappers over controller-binding calls that have shipped all along (`Field.importMFNodeFromString`, `Node.remove`, `Field.setSFVec3f` / `setSFRotation`, `Node.resetPhysics`, `Node.exportString`).

> ⛔ **`spawn` AND `delete` ARE SCENE-GRAPH VERBS, NOT PHYSICS VERBS — THE SOLVER IGNORES BOTH UNTIL THE WORLD IS RELOADED. MEASURED 2026-08-17.**
> The MuJoCo model is frozen at `finalizeWorld()`, so the two failures are exact mirrors of each other: **a deleted node's collider stays in the model** (a deleted floor still holds a body up, a deleted wall still stops a robot and still blocks rays), and **a spawned node's collider never enters it** (a spawned dynamic body never falls; a spawned static body never collides).
>
> Spawn, measured on the CPU `mj_step` path with an in-session control — a world whose floor is topped at z = 0.50, elevated so the implicit ground plane cannot substitute:
>
> | body | authored/spawned | released at | after 2200 steps (~87 s sim time) |
> |---|---|---|---|
> | control box (0.2 m, `physics`) | **authored in the `.wbt`** | z = 1.5 | **z = 0.599892** — floor top + half box = 0.600 ✅ |
> | identical twin | **`POST /scene/spawn`** | z = 1.5 | **z = 1.5, unchanged** — not one float ULP ⛔ |
> | platform (static, topped z = 1.00) | **`POST /scene/spawn`** | — | control **fell straight through it** to 0.599892 ⛔ |
>
> The engine log is the mechanism: exactly **one** `[OmNewtonBackend] registered 1 dynamic + 1 static Newton bodies` pass in the whole run, emitted at load, with the two spawns adding **zero** — and every `[OmNewtonBackend] step` line out to step 61440 lists only `b0` (floor) and `b1` (control). Cause: `finalizeWorld()` sets `openForBuild=false` (`OmNewtonBackend.cpp:2384`), all ~25 `addBody`/`addShape*` verbs guard on it (`:886`, `:1013`, `:1033`, …), and `ensureWorldOpen()` refuses to reopen once running (`:807`).
>
> **The failure is completely silent engine-side.** `0 errors, 0 warnings`; the response returns `verification.node_resolved: true`; the node appears in `/scene/tree` at its authored pose forever and renders normally. Nothing engine-side reports it, which is why this stood as documented-working for so long — and why the unit tests in `tests/harness/test_mutation_verbs.py` never caught it: they are pure/unit with no engine and assert nothing physical. **The harness now makes it loud (the W1.7 honest interim):** every successful `/scene/spawn` and `/scene/delete` response carries a `physics_warning` block — `{"code": "RUNTIME_MUTATION_NOT_IN_SOLVER", "message": ...}` — on every input form (vrml / type+fields / clone), and the first use of each verb per world-load emits one `world.warning` with the same code into `/sim/events`. `GET /capabilities` carries the gap too, in `not_supported` (`scene.runtime_mutation_physics`) and in the two endpoints' summaries.
>
> **So:** in DEFAULT mode, use `spawn` for cameras, markers and visual props, and for staging a scene you then `/world/load` to make real. Do **not** use it for anything that must fall, collide or be picked up, and never treat a spawned floor or wall as a collision surface — unless you opt into the rebuild below. `set_pose` is unaffected — it moves a body the solver already knows about. Tracked internally as W1.7 — runtime scene mutation, one workstream covering both the spawn and the delete direction.
>
> ✅ **Opt-in fix since 2026-09-01 (W1.7 shipped, engine `88487d988`): a mid-run physics rebuild.** `POST /sim/rebuild_physics {"settle_steps"?}` — or `{"physics": "rebuild"}` directly on `/scene/spawn` / `/scene/delete` — re-registers the whole scene with the solver at its **current** poses, so runtime-spawned nodes gain physics and deleted ones lose their phantom colliders; measured **97–267 ms**. It answers `409 REBUILD_REFUSED` (with the engine's own reason) on Cloth / SoftBody / GranularBed worlds, and engaged `Connector`/`VacuumGripper` welds are **DROPPED** with a loud warning — the `physics_warning` path above remains the DEFAULT behaviour, never silently replaced (PROTOCOL.md §7.36).

```bash
# from a raw VRML node string
curl -X POST :6989/scene/spawn -d '{"def":"PROBE_BOX","translation":[2,1,0.6],
  "vrml":"Solid { name \"probe\" children [ Shape { geometry Box { size 0.4 0.4 0.4 } } ] }"}'

# from a type + fields spec (JSON in, VRML composed for you)
curl -X POST :6989/scene/spawn -d '{"def":"BOX","type":"Solid","translation":[2,1,0.6],
  "fields":{"name":"box"}}'

# clone an existing node -- the only way to spawn a URDF robot (see below)
curl -X POST :6989/scene/spawn -d '{"clone":"HUSKY_0","def":"HUSKY_1","name":"husky_1",
  "translation":[9.7082,7.0534,0.2],"rotation":[0,0,1,2.19911]}'

curl -X POST :6989/scene/set_pose -d '{"def":"HUSKY_0","translation":[12,0,0.2]}'
curl -X POST :6989/scene/delete   -d '{"defs":["MARKER","NO_SUCH_DEF"]}'
```

Measured: **0.27–0.44 s** per spawn (Newton, light) / **0.03–0.32 s** (ODE). A ten-robot scene built from a one-robot container took **12 HTTP calls and 10.4 s** (Newton) / **4.7 s** (ODE), all ten robots distinct, controlled and driving.

### Default edit iteration: `/world/sync`

After the initial `/world/load`, agents should send every authored edit to the
same endpoint; the harness decides whether it is safe to avoid a reload:

```bash
curl -X POST :6789/world/sync -d \
  '{"path":"worlds/scene.wbt","reset_physics":true,"settle_steps":120}'
```

The classifier compares the edited file with the exact authored-source snapshot
that produced the running world. It chooses one of three modes:

- `live_pose`: every semantic change is a numeric `translation` or `rotation`
  on an existing root-level DEF. The supervisor validates the entire batch
  before its first write, applies all Solid/Robot poses, resets moved bodies,
  settles once, and returns every measured position.
- `no_change`: only comments, whitespace, or equivalent numeric spelling
  changed; no runtime action is needed.
- `full_reload`: everything else. Geometry, collision, physics/mass, material,
  controller, nested-node, add/remove, malformed, light-mode, and ambiguous
  changes automatically use the ordinary engine parser and return its normal
  structured diagnostics.

This is deliberately one-sided: uncertainty costs a reload; it never causes an
unsafe live write. Callers should not pre-classify edits. The MCP `load_world`
tool also uses `/world/sync` by default; `force_reload=true` preserves deliberate
controller-restart/full-reparse semantics.

For a physics check, size `settle_steps` from the motion being tested rather
than copying a large constant. For free fall,
`ceil(sqrt(2 * distance / |gravity|) / basicTimeStep)` is the impact budget;
add a conservative contact-settling margin. The live response already contains
the final positions, so a separate `/sim/step` and node-read pair is unnecessary.

Measured 2026-08-12 on the 0.45 m, 1 kg Newton cube used for the agent-cycle
check (explicit static floor, 8 ms step, five trials, native curl, identical
0.224946 m final height): the full hot-reload -> 250 steps -> node-read loop
took **1,562.7 ms median**; the live pose/reset -> 120-step settled readback
took **115.3 ms median**. That is **13.6x faster / 92.6% less wall time**;
120 steps matched the converged 250-step answer in all five trials.

Generalized runtime validation on the committed fixtures
[`world_sync_solid.omniworld`](../../tests/harness/worlds/world_sync_solid.omniworld) and
[`world_sync_robot.omniworld`](../../tests/harness/worlds/world_sync_robot.omniworld): a
two-Solid pose batch plus 120 steps completed in **130.7 ms** and returned both
converged rest heights (**9.8x faster** than the measured reload); a Robot
translation+rotation plus 120 steps completed in **138.3 ms** (**9.2x faster**)
and read back `(0.600000, -0.400000, 0.099946)` with rotation angle `0.700000`.
A geometry-size edit on the same live session was correctly refused by the fast
classifier and automatically hot-reloaded in **1,275.5 ms** with a clean
`2 dynamic + 1 static` Newton census.

Do not emulate unsupported edits by deleting and re-importing a live Solid:
Newton currently has no body-removal path, so deleted MuJoCo geometry remains
physical even after it disappears from the scene tree
([`tests/test_newton_runtime_deletion.py`](../../tests/test_newton_runtime_deletion.py)).

### The four rules that cost the most to learn

1. **`URDFRobot` cannot be imported from a string — clone instead.** This is an engine constraint. `URDFRobot { url ... }` is a *source* expansion done by `OmTokenizer::tokenizeFile` ([`src/omnisim/vrml/OmTokenizer.cpp:412`](../../src/omnisim/vrml/OmTokenizer.cpp)); a supervisor import goes through `tokenizeString`, never expands it, and `OmParser::protoNodeList()` then classifies `URDFRobot` as a PROTO — which `OmNodeOperations::importNode` refuses unless the loaded world declares it `IMPORTABLE EXTERNPROTO`. The same applies to **every PROTO** the world does not declare (`CardboardBox`, etc.). You get a `422` `SPAWN_REJECTED` carrying the exact VRML that was rejected plus the engine's own parse error. `{"clone": "<DEF>"}` sidesteps it: the engine hands back the *already expanded* `Robot` via `Node.exportString()`, so there is no second URDF importer to drift from.
2. **A cloned robot's `name` must be right at import time**, so it is rewritten in the node text rather than set afterwards. The engine starts the imported robot's controller immediately and the controller's IPC channel is keyed by the robot's **name** — clones carrying the source's name collide with `refusing connection attempt from another extern controller`, the second controller exits 1, and that robot simply never moves. Measured before the fix: **8 of 9 clones silently dead, no error anywhere in the HTTP responses.** The rewrite is depth-aware (a robot subtree is full of nested `name` fields); `overrides_in_vrml` vs `overrides_by_field_write` tells you which route each override took.
3. **A supervisor field write lands on the engine's next step**, so `/scene/set_pose` defaults to `settle_steps: 1` and `reset_physics: true` (a teleported body otherwise keeps its velocity and drifts, which reads as "the pose did not stick"). A spawn needs no settle for pose, because the pose is spliced into the node text.
4. **Nothing checks interpenetration.** `set_pose` will happily place a dynamic body inside static geometry and let the physics resolve it: on lane3, `BALL` (rest height `z ≈ 0.149` under ODE) placed at `z = 0.1` tunnelled through the floor and read `z = -2251` moments later. `GET /scene/node/<def>?bounds=1` before placing is the check.

Failure bodies on these verbs carry a `code` (`DEF_NOT_FOUND`, `PARENT_DEF_NOT_FOUND`, `CLONE_DEF_NOT_FOUND`, `DEF_TAKEN`, `SPAWN_SPEC_INVALID`, `SPAWN_REJECTED`, `FIELD_NOT_ON_NODE`, `POSE_UNSPECIFIED`).

**A caller error is now a 4xx everywhere, not just on the mutation verbs (2026-08-12).** Every supervisor rejection used to arrive as a 503, including the caller's own bad argument: `POST /sim/reset {"restore": 7}` answered `503 {"error": "'restore' must be a snapshot name or null"}`, and `urllib`/`requests` **raise** on a 5xx — so the agent read a server outage and retried a request that could never succeed. `classify_supervisor_error()` now splits the three cases by the prose each layer is known to produce: `internal: ...` (the supervisor's own dispatch prefix) → **500 `SUPERVISOR_INTERNAL_ERROR`**; transport prose (`supervisor RPC failed`, `not connected`, `world load in progress`, …) → **503 `SUPERVISOR_UNAVAILABLE`**, because retrying is exactly right there; anything else is a `CommandError` from the supervisor's dispatch → the rule table's status (404 `DEF_NOT_FOUND`, 409 `DEF_TAKEN`, 422 `FIELD_NOT_ON_NODE`, …) or **400 `ARGUMENT_INVALID`**. Applied to every endpoint that forwards the caller's own arguments: `/sim/reset`, `/sim/snapshot`, `/sim/restore`, `/scene/node/<def>`, `/robot/<def>/joints`, `/robot/<def>/joints/set`, `/robot/<def>/ik`, `/robot/<def>/devices`, `/robot/damage/inject`.

Two verification flags that mean exactly what they say, and used not to:

- **`409 DEF_TAKEN`: a spawn onto an already-used DEF is refused, not merged.** The engine does not rename a duplicate DEF on import, and `Supervisor.getFromDef` answers with the **first** dictionary match — so the spawn would import the new node and then report the *pre-existing* one's `def`/`id`/`type`/`position`, with `verification.def_resolves` reading `true` and `pose_delta_m` measured against the wrong body. There is no honest resolution at that layer, so the DEF is checked before the import.
- **`/scene/delete`'s `verification.all_removed` requires that something was actually removed.** It was computed from the removed list alone, so a request naming only DEFs that do not exist (`{"def":"TYPO"}`) removed nothing, had nothing to re-resolve, and came back `200` with `all_removed: true` — a confirmation for a typo. It is now `removed AND nothing missing AND nothing still resolving`, with `all_removed_reason` naming which condition failed.

## Commanding a robot's joints: `POST /robot/<def>/joints/set`

The harness's first robot-**commanding** endpoint (internal parity plan, item W2.1): supervisor-driven joint position targets with settle-and-verify semantics. `{"joints": {"<name>": <rad_or_m>, ...}, "settle_steps"?: int}` (or parallel `names` + `positions` lists); joint names are the ones `GET /robot/<def>/joints` reports. Every joint comes back with **measured** `commanded` / `achieved` / `error` / `moved`, plus a `verification` block (`max_abs_error`, `settle_steps`, `sim_time_advanced_ms`) — never the argument echoed back.

- **NOT a teleport.** `Node.setJointPosition()` also re-pins the motor's PD target ([`OmJoint.cpp`](../../src/omnisim/nodes/OmJoint.cpp)), so under Newton the joint **converges over ticks** — hence the settle (default 16 basic steps) and the measured read-back. Measured on a passive rig (Newton, light): a limited hinge commanded to 0.6 rad reads `achieved 0.5999995`, `error -4.5e-07` after 16 steps / 256 ms of sim time.
- **Targets beyond the joint's hard stops are clamped and flagged** (`commanded` is the adopted value, `clamped: true`), mirroring the engine's own `OmJointParameters::clampPosition` — stops only, `minStop == maxStop == 0` means unconstrained. Measured: requested 2.5 on a ±1.0 joint → `commanded 1.0`, `achieved 0.9999994`.
- ⚠️ **The W1.4 trap is reported, never silenced: a motor with no position limits is a velocity wheel (`ke = 0`) whose position targets the physics silently ignores** ([`OmBasicJoint.cpp`](../../src/omnisim/nodes/OmBasicJoint.cpp); 1680 such joints in-tree). The endpoint pre-classifies every joint by the engine's own registration rule (motor `minPosition`/`maxPosition` when they differ, else joint stops; sliders always servos) and reports `position_controllable: false` with the mechanism in `note`. Measured: a limit-less wheel commanded to 2.0 rad → `achieved ≈ 0`, `moved: false`, `error: -2.0`, and the load log carries the engine's own VELOCITY-wheel warning.
- ⚠️ **A robot whose controller re-asserts its targets wins.** The OmniLink bridges' hold mode re-applies its setpoints every tick (`omnilink_arm_bridge.py`: *"Re-apply target each tick so motors don't drift"*), so on a bridge-driven robot `achieved` snaps back to `position_before` within the settle window — measured on the UR5e chat world, all three commanded joints returned to the hold pose (residual error 0.42–0.70 rad). For clean supervisor-side joint control use a robot with a passive/no-op controller, or command the robot through its own bridge (PROTOCOL.md §5).
- Unknown joint names refuse the **whole batch** before anything is written (`422 JOINT_NOT_FOUND`, naming the offenders and the addressable joints); duplicate names across joints are `409 JOINT_NAME_AMBIGUOUS`. Works in light **and** heavy supervisor mode (verified live in both). A write verb — never transparently retried.

## Previewing a reach: `POST /robot/<def>/ik`

Batched inverse kinematics against the **live Newton model** (`World.solve_ik`, internal parity plan, item W2.1). `{"effector": "<DEF of the end-effector Solid>", "targets": [[x,y,z], ...], "rotations"?: [[qx,qy,qz,qw], ...], "tool_offset"?: [x,y,z], "iterations"?: int}`. A **pure preview**: nothing moves — the response carries per-target joint angles keyed by the same names `GET /robot/<def>/joints` reports (clamped to the authored limits) plus `residual_m` per target, and the caller applies them with `POST /robot/<def>/joints/set`. The intended loop is **solve → branch on residual → apply → verify**.

- **`residual_m` is FK-measured on exactly the returned angles**, in metres — an unreachable target reports its real miss (never hidden), so reject a target on its residual instead of driving to it.
- **Only the effector robot's own Hinge/Slider joints are solved** (the `joint_dof_mask` — unmasked, newton's optimiser will "reach" a target by translating a floating base 0.923 m, a coordinate no controller can command; see `tests/test_newton_ik_slots.py`). Hinge2/Ball joints are multi-coordinate and excluded by design. A solved joint the robot walk cannot name comes back `node_<id>` / `appliable: false` with `verification.unmapped_node_ids` — usually a wrong `def` for the effector's robot.
- **`tool_offset` is how you target a massless tip.** A tip Solid without `Physics` is folded into its parent body, so the solve targets the parent body's frame; give the tip's offset in that frame (e.g. `[0, 0, 0.3]`) and the residual is measured at the true TCP.
- ⚠ **The first solve per world compiles a warp kernel** — measured (machine `9722d23d12a3`): 8.3 s truly cold on a 6R arm, 2.37 s first-call on a 2R rig with a warm on-disk warp cache, **106–116 ms warm** in-process (`solve_ms` in the response is the measured cost of that call). Budget the first request's timeout accordingly.
- **Closed-loop verified live** (2R rig, Newton CPU `mj_step`, light mode): three reachable targets solved to residuals 1.2e-07–3.1e-06 m; applying the returned angles via `joints/set` (settle 60) put the measured TIP **1.0e-05–2.2e-04 m** from the requested Cartesian targets; an unreachable target at 1.5 m reported residual **0.9000 m** — geometrically exact (1.5 m distance − 0.6 m reach), not hidden.
- Verified on the default CPU `"mujoco"` solver only; **unverified on `mujoco_warp`**. Light mode verified live; heavy mode runs the identical path (the verb reads no tracker). A pure read, transparently retryable. Errors: `404 DEF_NOT_FOUND` (robot or effector), `422 IK_NO_BODY` / `IK_NO_JOINTS`, `503 IK_UNAVAILABLE` (world not finalised — retry), `500 IK_SOLVER_FAILED`, `400 EFFECTOR_UNSPECIFIED` / `TARGETS_UNSPECIFIED`.

## State snapshots, and what `/sim/reset` now means

`POST /sim/snapshot {"name": "t0"}` / `POST /sim/restore {"name": "t0"}` / `GET /sim/snapshots`, built on `Node.saveState()` / `Node.loadState()` from the scene root — recursive over the whole scene engine-side (`OmGroup::save`). `/sim/restore` puts the bodies back **without** rewinding the clock; `/sim/reset` rewinds the clock **and** restores.

- **`/sim/reset` now actually resets.** It used to rewind `sim_time_ms` and leave the scene where it fell (measured on both backends, and documented as a trap in PROTOCOL.md §7.11). It now also loads `"__init__"`, and `{"restore": null}` gets the old behaviour back.
- ⚠ **A RESET RE-PINS EVERY MOTOR IN THE SCENE, AND NOTHING RESTARTS THE CONTROLLERS.** This is the single most expensive thing on this page. MEASURED 2026-08-12 (2 of 3 agent cells): fresh harness, single load, supervisor connected; `/sim/reset` reported "authored poses restored"; 1250 subsequent `/sim/step`s advanced **20.0 s of sim time at normal per-step cost**; all **10 robots read 0.00 m net and 0.00 m path**. A second cell read its wheel joints frozen at **980.14 rad**. The same world drives 57.9–89.3 m/robot under `run-headless`, so the world is fine. **Mechanism** (read from the engine source): `OmSimulationWorld::reset()` → `root()->reset("__init__")` walks the whole scene → [`OmMotor::reset`](../../src/omnisim/nodes/OmMotor.cpp) clears `mUserControl` and re-pins `mTargetPosition` to the joint's **current** position — so a wheel set up in velocity mode with `setPosition(inf)` becomes a **position hold at wherever it stopped**, which is exactly that 980.14 rad. The supervisor path passes `restartControllers = false`, so a controller that commanded its motors once at start-up never re-issues them. **The response now says so**: an `actuation` block carrying the mechanism, the measurement and two workarounds (re-issue the motor commands from the controller, or `POST /world/load` the same world — a load starts fresh controllers), plus a top-level `warning`. The fix itself belongs in the engine; the harness's job is to stop an agent concluding the physics is broken.
- **`/sim/reset` refuses to report success without a supervisor behind it.** If the reset RPC returns but the supervisor is no longer answering a real RPC, you get `503 SUPERVISOR_LOST` with `supervisor_connected: false` instead of a 200 that attests to a scene nobody can read.
- **`"__init__"` is the engine's own parse-time state, and it is free.** `OmNode`'s constructor sets `mCurrentStateId = "__init__"` ([`src/omnisim/vrml/OmNode.cpp:161`](../../src/omnisim/vrml/OmNode.cpp)) and `OmPose`'s constructor saves the authored translation/rotation under it. Nothing has to be snapshotted for a reset to mean "as authored".
- **A supervisor snapshot is not a substitute for it.** The engine free-runs (`--mode=fast`, `synchronization FALSE`), so by the supervisor's first step a dropped body has already fallen: on lane3, `BALL` is authored at `z = 1.0` and the supervisor's first read is `z = 0.1`.
- **Restoring a name that was never saved is refused, and that refusal matters.** `OmPose`'s saved-pose map is a `QMap` whose `[]` **default-constructs a zero vector** on a miss, so restoring an unknown state name would silently teleport the whole scene to the origin. The supervisor keeps a registry and returns `404 SNAPSHOT_NOT_FOUND` instead.
- **Names die with the world.** The registry lives in the supervisor process, which every world load restarts — a snapshot can never outlive the world it describes. Names starting with `__` are reserved.
- Verified both backends: after moving a body 2.83 m, `restore` reports `verification.vs_snapshot.max_pose_delta_m` = `0.0` (Newton) / `6.6e-05` (ODE, still settling). The `verification` block samples **top-level** poses only — one IPC read per node is what makes a whole-scene sample unaffordable — and a body that is still falling reports a non-zero delta legitimately.

## Answers that must never be more confident than what they measured

Four fixes from the 2026-08-12 agent diagnostic round (three agent-built cells, every place the *tool* lied recorded; frequencies are out of three). Regression tests: [`tests/harness/test_agent_trust_regressions.py`](../../tests/harness/test_agent_trust_regressions.py).

- **`/world/screenshot` never answers `200` without a picture (2/3).** It returned HTTP 200, `Content-Type: image/png`, and a **zero-byte body**. Not the rendering-disabled case — `--no-rendering` is only passed when `with_supervisor` is false, and the same scene rendered 603 KB through the capture service moments later; the supervisor reported success and nothing checked that a file had landed. It now validates the bytes are a PNG (IHDR present) and otherwise returns `502` `SCREENSHOT_EMPTY` / `SCREENSHOT_NOT_PNG` with the byte count, the render path, the read error and the supervisor's own reply. The path form returns `bytes` + `pixels` so a caller that never opens the file can still tell a picture from a placeholder. **This one was expensive out of all proportion to its size**: agents worked around it by reaching for the capture service, which renders 1920×1080 through WREN and took the owner's laptop GPU to **86 °C**, and the `.capture_*` sibling world it leaves behind got a correct 10-robot run graded FAIL on the wrong file. (This is the *second* silent failure of this endpoint — `note_render`'s stale-frame check is the first.)
- **`/robots` no longer counts the harness's own supervisor (2/3).** The injected `harness_supervisor` Robot appeared in the roster as `'#939' | 'harness_supervisor'`, so an agent asserting "exactly 10 robots" on a 10-robot world read 11. It is not in the user's `.wbt` — the harness put it in a sibling copy. It is now **excluded by default** and named in `harness_injected` (never removed silently), with `?include_harness=1` to list it flagged. `/scene/tree` takes the opposite remedy for the same problem: the node stays (a tree is a dump of what is *in* the scene) but carries `harness_injected: true`, and the response carries the same top-level `harness_injected` list.
- **A supervised load is not `complete` without a supervisor (2/3).** Measured: `load_state: "complete"` next to `supervisor_connected: false`, on a *fresh* `/world/load` that returned `ok` — the session was dead for the rest of the run. Every `/world/load` result now carries a boolean `supervisor_connected`, `_supervised_load_result` re-checks it before claiming `complete` (a supervised load with no supervisor is `ok: false`, `load_state: "bind_failed"`), and `/sim/state` reports `load_state: "supervisor_lost"` rather than `complete` for a supervised world whose supervisor is gone. A bare `with_supervisor=false` load is unaffected and still reads `complete`.
- **The hot reload proves its rebind with a real RPC, not a ping.** The *outgoing* supervisor keeps listening until the engine actually swaps worlds and answers `ping` the whole time it is dying, so the two-ping stability check could adopt a corpse and report `supervisor: "connected"` on a session that had none — that is the mechanism behind "the supervisor never rebinds on the second load". Adoption now requires a successful `sim_state` RPC; a candidate that only answers pings is closed and the harness re-binds until one answers or the deadline expires (`supervisor_rebind_rejections` counts them), then falls back to the cold launch.

## Spatial awareness: bounds, camera read-back, framing, orbit, visibility

The camera endpoints exist so an agent stops guessing a pose, screenshotting, and guessing again. The loop is: **`/scene/frame` → screenshot → `/scene/visible` → `/scene/orbit`**.

| Endpoint | What it answers |
|---|---|
| `GET /scene/tree?bounds=1` | *How big is everything and where is its middle?* Attaches `{center, radius, bbox_min, bbox_max, size, exact, sources, skipped}` in **world coordinates** to every node. Opt-in because it walks all geometry (and reads mesh files). |
| `GET /scene/node/<def>?bounds=1` | The same, for one node's whole subtree. |
| `GET /scene/viewpoint` | *Where is the camera actually looking?* Position, orientation, `fieldOfView`, near/far, follow settings, plus derived `forward`/`left`/`right`/`up` unit vectors, the real viewport size, and the resolved horizontal + vertical FOV. Every other camera call used to write to a camera you could not read. |
| `POST /scene/frame` | *Put this in frame.* Takes `{"def": "HUSKY"}`, `{"defs": [...]}`, or `{"target": [...], "radius": r}` plus `mode` / `margin` / `aspect` / `push`. Computes **both** aim and distance and returns a `verification` block (angular offset vs available half-FOV, subject screen bbox in pixels, `fits`). |
| `POST /scene/orbit` | *Nudge from here.* `azimuth_deg`, `elevation_deg`, `dolly` (multiplier), `pan` `[dx, dy]` in screen-space metres, around the current look-at point or an explicit `center` / `def`. |
| `GET /scene/visible` | *What is on screen right now?* Per node: frustum test, screen-space bbox + centroid in pixel coords, distance, signed angular offset, and a hint like `"off-screen: 34 deg to the left, 12 deg up"`. |

`mode` for `/scene/frame`: `hero` (default, the same 3/4 shot the world generators bake), `top_down`, and the subject-relative `front` / `back` / `left` / `right` / `top` / `bottom`. With a single `def` the directional modes are rotated into the **subject's** frame (`relative_to: "subject"` in the response); with several, they fall back to world axes.

### How the bounds are computed, and how exact they are

The bounds come from a **Python-side geometry walk** in [`projects/default/controllers/harness_supervisor/geometry.py`](../../projects/default/controllers/harness_supervisor/geometry.py), not from the engine's internal `OmBoundingSphere` (which is not exposed to controllers). It unions the **collision** geometry (`boundingObject`) and the **visual** geometry (`children` → `Shape` → `geometry`) of a node's whole subtree, in one post-order pass over the scene.

- Primitives (`Box`, `Sphere`, `Cylinder`, `Capsule`, `Cone`, `Plane`, `ElevationGrid`) and explicit coordinate sets (`IndexedFaceSet` / `PointSet` / `IndexedLineSet`) are **exact**; a rotated primitive's world AABB is the AABB of its rotated corners, so it is conservative — it always contains the geometry.
- `Mesh { url ... }` is read off disk (STL binary + ASCII, OBJ, PLY ASCII, Collada `.dae`) and cached by path+mtime. A mesh that cannot be parsed is **skipped and flagged** (`exact: false`, with the reason in `skipped`) rather than silently under-reported.
- `radius` is the half-diagonal of the AABB, so it is always ≥ the true bounding-sphere radius: framing on it never crops the subject.

**Cost — ask by DEF.** Every supervisor field read is a round-trip to the engine — **~0.6–0.9 ms** on the current engine (the immediate-burst fast path, 2026-09-01; pause-independent — measure *your* session with `GET /debug/read_bench`) — so a whole-scene bounds walk is hundreds of round-trips and stays opt-in. (The "~6 ms free-running vs ~0.15 ms paused" contrast previously quoted here is the July 2026 pre-burst world; the walk still runs against a paused engine, but for snapshot consistency, not speed — see the runtime note above.) Node *shape* — type, which fields exist, geometry extents, child handles — is cached per node id for the life of the world (the supervisor process is restarted by every world load, so the cache cannot outlive its world); only the world transforms are re-read, and an MFNode child list is re-fetched only when its `getCount()` changes. Measured post-fix on the 298-node 10-Husky stress scene (light): `?bounds=1` **0.46 s cold, 0.19 s warm** (the pre-fix figures for the same class of walk were tens of seconds — see the [latency ledger](../../docs/developer/harness-latency-2026-07-31.md)). **`/scene/frame` and `/scene/visible?defs=...` walk only the named subtrees**, so name what you care about instead of asking for the whole scene.

⚠️ **`Cylinder` / `Capsule` / `Cone` are Z-axis aligned** in OmniSim (post-R2022b). `OmCylinder::rescale` and `OmCone::scaledHeight` still multiply `height` by `scale.y()` — those are stale pre-R2022b leftovers; the authoritative code (`scaledHeight()` → `scale.z`, the ODE geoms, `computeFrictionDirection`, `recomputeBoundingSphere`) uses Z. Reading the stale path measured a Husky **0.885 m wide instead of 0.685 m**.

### Cross-checking against the engine (`?probe=1`)

`GET /scene/node/<def>?bounds=1&probe=1` additionally recovers the engine's **true** `OmBoundingSphere` by inverting `OmViewpoint::moveViewpointToObject`, which parks the camera at `centre − direction × 1.05·r / (sin(fov/2)·min(aspect, 1/aspect))`. Probing from two different orientations gives `distance = |p₁ − p₂| / |d₂ − d₁|`, then the centre and radius. The camera pose is saved and restored, so the probe is side-effect free — but it is **slow (~5–8 s)** and steps the sim, so it is opt-in and meant as a correctness oracle, not a data path. `residual` (how far the two probes' recovered centres disagree) should be ~0; anything else means the inversion did not hold for that scene.

Measured on `omnilink_husky.omniworld` (Clearpath Husky A200, real chassis 0.99 × 0.67 × 0.39 m):

| | centre | radius | size |
|---|---|---|---|
| geometry walk | `[0, 0, 0.184]` | 0.628 | `0.987 × 0.669 × 0.396` |
| engine probe | `[0.0003, 0.0116, 0.200]` | 0.721 | — |

Centres agree to 2 cm; the engine's sphere is ~15 % larger because it is a hierarchical merge of sub-spheres rather than a fit to the AABB.

> **Note.** Supervisor field writes are queued and applied on the *next* sim step. Anything that sets a field and then expects the engine to have acted on it (the probe sets `Viewpoint.orientation` before calling `moveViewpoint()`) must step in between — otherwise the engine acts on the previous value.

For the entry-level overview (startup, common loop, endpoint cheatsheet) see [`AGENTS.md` §5](../../AGENTS.md#5-iterating-on-worlds-with-the-validation-harness).
