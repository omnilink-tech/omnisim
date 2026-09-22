# Harness endpoint cheatsheet — full reference

> **Verbatim reference moved out of `AGENTS.md` on 2026-09-02.** Every passage below is the
> original text, word for word (markers, dates, commit hashes and self-correction history included);
> `AGENTS.md` now carries a short summary of each item and links here. Nothing was paraphrased.


This is the complete text of every row of the **Endpoint cheatsheet** table in `AGENTS.md` §5, one section per endpoint, in the table's order. The section heading is the row's first column (route + request shape); the body is the row's second column verbatim. The live contract is `GET /capabilities` and [PROTOCOL.md](../../PROTOCOL.md) §7.

## GET /capabilities

**Route:** `GET /capabilities?probe_step=1`

⭐ **Start every session here.** One call answers what you are talking to and what it will refuse: `physics` **read from the engine's own `.newton.json` verdict sidecar**, with `source` naming the provenance — `sidecar` / `engine_log` / `sidecar_stale` / `sidecar_unreadable` / `sidecar_absent` / `retired_selector_ignored`. ⚠️ **`backend` never says `"ode"` any more** (fixed 2026-08-08; it used to, which contradicted its own `detail` text on the field an agent actually branches on). The negative case is `backend: "unverified"` with `source: "sidecar_absent"`, and it means **"Newton did not finalize this world"** — most often a run too short to reach finalize (budget ≥15 s, ≥45 s on virtualised disks), otherwise a runtime that would not come up. It does **not** mean another engine drove the world; there is no other engine. A set `OMNISIM_FORCE_ODE` / `OMNISIM_LEGACY` / `OMNISIM_ALLOW_ODE_FALLBACK` reports `backend: "newton"` with `source: "retired_selector_ignored"` — the variable is warned about and ignored, so the run is Newton regardless. `limits.step_cost` + `recommended_max_steps_per_request` (a rolling median of the **measured** per-step cost on *this* world, so you size a step budget instead of discovering the 120 s RPC timeout by hitting it — `?probe_step=1` advances one step to measure it); `event_types` served **from the code** (the supervisor scans its own `emit()` call sites, so doc drift is impossible) with `suppressed` naming what a `--light` session will not produce; `endpoints` cross-checked against the request handler's own source; `not_supported`, every gap with a `reason` and a `workaround`; and the `diagnostic_codes` / `request_error_codes` enums. Needs no supervisor and no loaded world.

## POST /world/load

**Route:** `POST /world/load {path, wait_s?, with_supervisor?, light?, tracking?}`

Load a `.wbt`. Returns structured diagnostics with codes like `WORLD_PARSE_SYNTAX_ERROR`, `PROTO_NAME_MISMATCH`, `TEXTURE_READ_FAILED`. **The set is OPEN — read it from `GET /capabilities` → `diagnostic_codes`, never from a count written here** (a hard-coded count has now been stale twice, 33 then 54; PROTOCOL.md §7.3 makes open-enum the contract, so this line stopped carrying a numeral on 2026-09-01; it was 56 that day): **40** distinct codes across the classifier's 48-entry rule table ([`scripts/harness/diagnostic_codes.py`](../../scripts/harness/diagnostic_codes.py)), + **9** `CUDA_CODES`, + `NEWTON_ZERO_DYNAMIC_BODIES` (synthesized from a matched rule rather than owning one), + `UNKNOWN`, + **5** the harness synthesizes when the engine never got far enough to log (`LAUNCHER_DLL_NOT_FOUND`, `SIMULATOR_EXITED_NONZERO`, `SUPERVISOR_BIND_STALLED`, `SUPERVISOR_BIND_CEILING`, `WORLD_DIR_NOT_WRITABLE`). `with_supervisor` defaults to true.

## POST /world/sync

**Route:** `POST /world/sync {path?, settle_steps?, reset_physics?, wait_s?, light?}`

⭐ **Default after any authored edit.** Compares the file with the exact source snapshot that produced the running world. If and only if all semantic changes are numeric `translation`/`rotation` values on existing root-level DEF nodes, validates the whole batch, applies it live, resets moved bodies, settles once, and returns measured positions (`mode: "live_pose"`). Comments/format-only edits return `mode: "no_change"`. Geometry, collision, mass, material, controller, nested-node, add/remove, malformed, light-mode, or ambiguous edits automatically use the ordinary engine reload (`mode: "full_reload"`). Do not pre-classify the edit yourself. ⚠️ **Two more `mode` values exist and this row used to omit them — branch on all five:** `rejected` (HTTP **422** — no path and no loaded world, world not found, unreadable file, or a bad `settle_steps`) and `busy` (HTTP **409** — another load or sync already in flight; retry). Status is 200 when `ok`, else 409 for `busy`, else 422 (the `409 if result.get("mode") == "busy" else 422` branch in [`omnisim_harness.py`](../../scripts/harness/omnisim_harness.py)).

## POST /world/load light

**Route:** `POST /world/load {"light": true}`

⭐ **The step-cost lever — and it is NOT just a multi-robot lever, which is how this row used to read.** `light=true` injects the supervisor with `--light`, dropping the per-step contact / joint-limit / grip trackers that walk the whole scene graph every basic step. Measured on a 298-node 10-Husky world under Newton (machine `9722d23d12a3`): `/sim/step` **27.0 s → 0.034 s** (~790×), a 10-step advance **120.0 s → 0.19 s** (~630×). ⚠️ **Those are PRE-`3b952b61d` figures and public issue #4 quoted them back at us as current. Re-measured 2026-08-29 on the same world (`husky_fleet_arena`, 309 nodes, CPU `mj_step`, same machine): full `/sim/step 1` **573–606 ms** vs light **6–35 ms** (~17×); 10 steps **2855–3187 ms** vs **48–67 ms** (~47×); the load itself 12.1 s vs 4.1 s. Smaller, still an order of magnitude — the advice stands, and every supervised `/world/load` response now carries a `tracking` block naming the mode and this cost, so an agent that never read this row still learns it from the response.** ⚠️ **It matters just as much on a TINY scene if that scene steps slowly** — measured 2026-08-14 on `newton_cloth_drape.omniworld`, two static bodies and one 289-particle sheet: `world/load` reload **13414 → 3131 ms**, `sim/step 1` **4298 → 210 ms**, `sim/step 30` **66671 → 1496 ms** (44.6×), `sim/reset` **7660 → 1870 ms**. Those heavy-mode figures predate `3b952b61d`, which cached the per-step scene walk and took heavy-mode cloth stepping to 191 ms/step; light is still ~4× better again. See the harness rule above for the mechanism — the cost is round-trips × step time, not node count, so "small world, skip the flag" is exactly wrong. The trade: `/sim/grips` returns empty and the `contact.*` / `grip.*` / `joint.limit_hit` event types go quiet — `?types=` is an exact-match allowlist, so filtering on a suppressed type returns an empty stream, not an error (`GET /capabilities` → `event_types_detail.suppressed` names them). ⚠️ **`/sim/contacts` is NOT suppressed and this row used to say it was** — it is served by `observe.collect_contacts`, which walks the scene per call and never reads the `ContactTracker`. On the Husky world the tracker-fed surfaces returned empty anyway, so the trackers were pure cost. **Light is the DEFAULT since 2026-09-02** — a `POST /world/load` naming neither `light` nor `tracking` runs light and says so in the response's `tracking.default_applied`. Ask for the trackers with `{"light": false}` (all three) or a `tracking` object (per-tracker); `OMNISIM_HARNESS_LIGHT=0` makes full tracking the process-wide default again. On the current engine the fleet arena loads 4.65 s light vs 5.2 s full and steps 23 ms vs 54 ms — the 12.1 s / 17–47× figures above are the 2026-08-29 engine and are history. ✅ **Per-tracker toggles ship as of 2026-09-01 (public issue #4):** `{"tracking": {"contacts": false, "joint_limits": false, "grips": false}}` on `POST /world/load` drops exactly the named trackers instead of all three — e.g. keep `joint.limit_hit` while paying no contact walk (measured: partial mode steps at light-mode cost, ~10 ms vs ~600 ms full on the fleet arena). `GET /capabilities` reports the per-mode suppression honestly, and the load response's `tracking.mode` reads `light`/`partial`/`full`.

## GET /world/diagnostics

**Route:** `GET /world/diagnostics`

Re-fetch parsed diagnostics from the current load.

## POST /world/screenshot

**Route:** `POST /world/screenshot {path?, quality?}`

Render PNG. Returned as the response body (`image/png`) or written to a server-side `path`.

## GET /world/render_stats

**Route:** `GET /world/render_stats`

`{mean_brightness, mean_rgb, max_rgb, saturated_pct, black_pct, warnings[]}`. Warnings include `"blown out: NN% of pixels are saturated"` and `"underexposed: NN% near-black"`.

## GET /scene/tree

**Route:** `GET /scene/tree`

Flat node list with type, DEF, position, orientation.

## GET /scene/node

**Route:** `GET /scene/node/<def>`

Field dump + contact points for one node. The dump includes **`boundingObject`** and **`physics`** as `{field_exists, present, summary}` — the two fields that decide whether a node collides and whether it moves (a floor with no `boundingObject` is a hologram).

## POST /scene/look_at

**Route:** `POST /scene/look_at {position, target, push?}`

Computes axis-angle orientation from default forward (+X) to the target direction and pushes it to the live `Viewpoint` when `push=true` (the default). Returns the orientation so it can be persisted back to the `.wbt`.

## GET /scene/tree bounds

**Route:** `GET /scene/tree?bounds=1`

Same tree, plus each node's **world-space** `{center, radius, bbox_min, bbox_max, size, exact}`. This is the number every camera decision needs; opt-in because it walks all geometry.

## GET /scene/viewpoint

**Route:** `GET /scene/viewpoint`

**Read** the live camera: position, orientation, `fieldOfView`, near/far, follow, plus derived `forward`/`up`/`right` and the resolved horizontal + vertical FOV for the real viewport aspect.

## POST /scene/frame

**Route:** `POST /scene/frame {def|defs|target+radius, mode?, margin?, push?}`

⭐ **The camera verb to reach for first.** Computes BOTH aim and distance so the subject fills the frame, pushes it, and returns a `verification` block (angular offset vs half-FOV, subject screen bbox in pixels, `fits`). `mode`: `hero` (default) / `top_down` / subject-relative `front`/`back`/`left`/`right`/`top`.

## POST /scene/orbit

**Route:** `POST /scene/orbit {azimuth_deg?, elevation_deg?, dolly?, pan?, center?|def?}`

Incremental nudge **relative to the current view** — every other camera API is absolute.

## GET /scene/visible

**Route:** `GET /scene/visible?defs=A,B`

What is on screen right now: frustum test, screen-space bbox + centroid in pixels, distance, angular offset, and a hint like `"off-screen: 34 deg to the left, 12 deg up"`. The closed-loop feedback signal for aiming.

## POST /scene/spawn

**Route:** `POST /scene/spawn {vrml|type+fields|clone, def, name?, translation?, rotation?, parent?, index?, settle_steps?, reset_physics?}`

⛔ **BY DEFAULT A SPAWNED NODE HAS NO PHYSICS — IT RENDERS AND IT IS IN `/scene/tree`, BUT THE SOLVER NEVER SEES IT (measured 2026-08-17). ✅ SINCE 2026-09-01 THERE IS AN OPT-IN FIX: pass `{"physics": "rebuild"}` (or call `POST /sim/rebuild_physics` after the spawn) and the node IS simulated — W1.7 shipped in `88487d988`; details and caveats below.** The default is deliberately unchanged (a rebuild costs 97–267 ms and drops engaged welds, so it is never applied silently), and in that default spawn is only a working *scene-graph* primitive. Both directions fail: a spawned **dynamic** body never falls, and a spawned **static** body never collides. Measured on the CPU `mj_step` path against an in-session control (machine `9722d23d12a3`), floor topped at z=0.50 so the implicit ground plane cannot substitute: the *authored* control box settled at **z=0.599892** (floor top + half box = 0.600) while a spawned twin of that exact box, released at z=1.5, read **z=1.5 unchanged after 2200 explicit steps and ~87 s of simulated time** — not one float ULP; and a spawned static platform topped at z=1.00 was **fallen straight through**, the control landing back on the authored floor at 0.599892. The engine log is the mechanism: **exactly one** `registered 1 dynamic + 1 static Newton bodies` pass, emitted at load, the two spawns adding **zero**, and every `[OmNewtonBackend] step` line to step 61440 listing only `b0` (floor) and `b1` (control). **The failure is silent engine-side — 0 errors, 0 warnings, and the response returns `verification.node_resolved: true`** — but the HARNESS now tells you (2026-08-19 honest interim): every successful `/scene/spawn` and `/scene/delete` response carries a `physics_warning` block (`code: RUNTIME_MUTATION_NOT_IN_SOLVER`), the first use per verb per world-load emits one `world.warning` into `/sim/events`, and `GET /capabilities` lists the gap under `not_supported` (`scene.runtime_mutation_physics`). Cause: `finalizeWorld()` sets `openForBuild=false` ([`OmNewtonBackend.cpp`](../../src/omnisim/physics/OmNewtonBackend.cpp), `OmNewtonBackend::finalizeWorld`), every `addBody`/`addShape*` verb guards on it, and `OmNewtonBackend::ensureWorldOpen()` refuses to reopen mid-run. ⚠️ **This is the exact MIRROR of the runtime-delete defect above — same frozen MuJoCo model, opposite symptom: delete leaves phantoms IN, spawn leaves real nodes OUT.** So in DEFAULT mode use spawn for cameras/markers/visual props and for staging a scene you will then reload; do **not** use it for anything that must fall, collide, or be picked up, and never treat a spawned floor or wall as a collision surface — unless you opt in. ✅ **THE OPT-IN FIX SHIPPED 2026-09-01 (`88487d988`, W1.7 — runtime scene mutation, both directions at once): a mid-run PHYSICS REBUILD.** A new engine verb (opcode 105, `wb_supervisor_simulation_rebuild_physics`) tears down the live Newton world and re-registers the WHOLE scene at its **current** poses — live velocities are replayed and motor targets re-pushed automatically, so a running robot keeps driving. Harness surface: `POST /sim/rebuild_physics {settle_steps?}`, or `{"physics": "rebuild"}` directly on `/scene/spawn` / `/scene/delete`. Measured (machine `9722d23d12a3`, CPU `mj_step`): rebuild costs **97–267 ms** (in-process SolverMuJoCo reconstruction, skipping the module-load 98%); the spawned box from this row's own reproducer, frozen at z=1.5 for 120 steps, lands at **0.599892258644104** after rebuild — **bit-identical** to the authored control's rest height, with the control body unmoved; a deleted floor genuinely stops colliding (both bodies fell through); and an 8-Husky motorised world drove THROUGH a mid-run rebuild at unchanged speed (+1.749 vs +1.689 m per 2 s window). ⚠️ Three caveats before you lean on it: it is **REFUSED with `409 REBUILD_REFUSED` on Cloth / SoftBody / GranularBed worlds** (those re-register from *authored* state, so a rebuild would teleport them — reload instead); **engaged `Connector`/`VacuumGripper` welds are DROPPED**, with a loud warning naming the count — do not rebuild mid-grasp; and **bitwise step-for-step continuation across a rebuild is NOT claimed** (a fresh world is a fresh solver state). The DEFAULT spawn/delete behaviour is unchanged — `physics_warning` still attached unless the caller opts into the rebuild. Three input forms: a raw VRML node string, a `type` + `fields` spec (VRML composed for you), or `{"clone": "<DEF>"}`. Measured 0.27–0.44 s per spawn (Newton, light); the 0.03–0.32 s figure alongside it was the ODE path and is history — Newton's is the only cost you can hit now. ⚠️ **A `URDFRobot` CANNOT be spawned from a string — clone one instead.** `URDFRobot { url ... }` is a *source* expansion done by `OmTokenizer::tokenizeFile`; a supervisor import goes through `tokenizeString`, never expands it, and `OmParser::protoNodeList()` then classifies it as a PROTO that `importNode` refuses. The same refusal hits **every** PROTO the loaded world does not declare `IMPORTABLE EXTERNPROTO`. So the container world needs **one** authored robot, not zero. ⚠️ **A clone's `name` must be unique and must be right AT import time** — the engine starts the controller immediately and keys its IPC channel by the robot's name; a clone carrying the source's name collides (`refusing connection attempt from another extern controller`), that controller exits 1, and the robot simply never moves. Measured before the fix: **8 of 9 clones silently dead, no error anywhere in the HTTP responses.** Pass `name` and the harness rewrites it depth-aware in the node text. Failures are a `422` `SPAWN_REJECTED` carrying the rejected VRML + the engine's own parse error.

## POST /scene/delete

**Route:** `POST /scene/delete {def|defs, settle_steps?}`

Remove nodes by DEF. Unknown DEFs come back named rather than failing the batch.

## POST /scene/set_pose

**Route:** `POST /scene/set_pose {def, translation?, rotation?, reset_physics?, settle_steps?}`

Move an existing node. A supervisor field write lands on the engine's **next** step, so this defaults to `settle_steps: 1` and **`reset_physics: true`** (a teleported body otherwise keeps its velocity and drifts, which reads as "the pose did not stick"). ⚠️ **Nothing checks interpenetration** — it will happily place a dynamic body inside static geometry and let the solver resolve it: on lane3, `BALL` (rest z ≈ 0.149) placed at z = 0.1 tunnelled through the floor and read **z = −2251** moments later. `GET /scene/node/<def>?bounds=1` before placing is the check.

## POST /sim/rebuild_physics

**Route:** `POST /sim/rebuild_physics {settle_steps?}`

⭐ **Make runtime spawns/deletes reach the solver (2026-09-01, W1.7).** Tears down the live Newton world and re-registers the WHOLE scene at its current poses — velocities replayed, motor targets re-pushed, 97–267 ms measured. `409 REBUILD_REFUSED` on Cloth/SoftBody/GranularBed worlds (reload those); engaged welds are dropped with a loud warning; bitwise continuation across a rebuild is not claimed. Same effect inline via `{"physics": "rebuild"}` on `/scene/spawn`/`/scene/delete` — see that row.

## POST /sim/pause and POST /sim/resume

**Route:** `POST /sim/pause {lease_ms?}` / `POST /sim/resume`

⭐ **Hold the engine ACROSS calls (v9; the primitive landed 2026-09-15 and was made to work 2026-09-22).** Every read endpoint already pauses the engine *internally* for the duration of its own walk — that is why a response is one consistent instant — but the scene moves again the moment you get it (~88–112 ms of sim time per idle poll). This is the guard you hold yourself. **Leased:** `lease_ms` defaults to 30000 and is clamped silently to `[1000, 300000]`; a non-integer is `400 BAD_REQUEST`. The deadline is a **safety property**, not a convenience — a client that pauses and then dies would otherwise freeze the simulation with no way back but killing the engine, so the lease self-expires and the main loop resumes on its own. 200 body, all 5 fields: `{paused, lease_remaining_ms, paused_at_sim_ms, lease_ms, rpc_ms}` (`lease_ms` is the **effective**, clamped value). A second pause while held **EXTENDS** the lease rather than erroring and does not move `paused_at_sim_ms` (measured: 116233 ms remaining → `{"lease_ms": 200000}` → 200000, `paused_at_sim_ms` unchanged at 2016.0). `POST /sim/resume` takes an empty body and returns `{paused, was_paused, released_by, rpc_ms}`; it is **idempotent** — a second call returns `was_paused: false`, not an error. ⚠️ **A held lease does not cross a `POST /world/load`**: `worldLoad` usually terminates the supervisor, but measured 2026-09-22 a second load of the *same* world came back with the process still alive, so the supervisor now releases the lease, clears every armed break and drops `break_hit` explicitly — otherwise the session inherited a lease held over the NEW world (frozen at t = 8 ms with nothing to explain why) and a `break_hit` describing a break in the OLD one. PROTOCOL.md [§7.38](../../PROTOCOL.md#738-post-simpause-post-simresume).

## POST /sim/break, GET /sim/breaks, DELETE /sim/break/&lt;id&gt;

**Route:** `POST /sim/break {types, filter?, lease_ms?, once?}` / `GET /sim/breaks` / `DELETE /sim/break/<break_id>` (twin: `POST /sim/break/delete {break_id}`)

⭐ **BREAK ON EVENT (v9) — the breakpoint this surface was missing.** Arm a condition and the supervisor takes the pause lease the moment a matching event is emitted, so the scene you then read is the scene **at** the moment of interest rather than the scene ~90 ms of free-running later. `types` is required and non-empty; `filter` keys are `def`, `counterpart`, `joint` **and nothing else** (all present keys must match); `once` defaults `true`; `lease_ms` is the lease the *hit* takes, same bounds as `/sim/pause`, clamped with a `lease_ms_clamped` diagnostic. At most **32** armed at a time. Arming fast-forwards the scan cursor to the end of the bus, so a break never fires on history. 200 body, all 13 fields: `{break_id, types, filter, once, armed, hits, lease_ms, armed_at_sim_ms, last_hit_sim_ms, armed_types, diagnostics, rpc_ms, ok}`. `GET /sim/breaks` returns `{breaks[], breakable_types[], silenced_types[], break_hit, paused, rpc_ms, ok}` — **read it before arming**. `DELETE` returns `{break_id, removed, rpc_ms, ok}` or `404 BREAK_NOT_FOUND`; `POST /sim/break/delete {"break_id"}` is the **exact twin** (both shipped; same 200 and 404 bodies) for clients whose HTTP layer cannot route a bodyless `DELETE`.

⚠️ **A BREAK THAT COULD NEVER FIRE IS REFUSED, AND THAT IS THE HEADLINE BEHAVIOUR.** Light tracking is the default and silences 5 of the 11 event types (`contact.*`, `grip.*`, `joint.limit_hit`). A break armed on one comes back `400 BREAK_EVENT_TYPE_UNAVAILABLE` with an `event_type_silenced_in_light_mode` diagnostic naming the **producer** (`ContactTracker`), the full `silenced_types` set and the `{"light": false}` workaround — because accepting it would hand you a breakpoint that simply never trips, which is indistinguishable from "the bug did not happen". The refusal is **SCOPED**: `damage.impact` / `damage.state_transition` survive light mode and a break on one is **accepted in the same session** that refused the `contact.began` (verified live, 2026-09-22). **A refused break is not armed** — `armed_types` is `[]` and `GET /sim/breaks` does not list it. Other refusal codes: `BREAK_TYPES_REQUIRED`, `BREAK_FILTER_INVALID`, `BREAK_LEASE_INVALID`, `BREAK_LIMIT_REACHED`, `BREAK_REFUSED` (fallback), `BREAK_NOT_FOUND`. Two more `diagnostics[].code`s exist because a type can be *declared* and still unreachable: `event_type_not_on_supervisor_bus` (`controller.log` / `world.warning` / `world.error` come from the harness's log ring buffer reading the engine's stdout, never from the supervisor bus) and `event_type_not_breakable` (`break.hit` would re-trigger on its own record). A non-fatal `filter_key_not_matchable_for_armed_types` rides along on a 200 when a filter key no armed type carries would fail to **narrow** the break rather than fail to fire it — the opposite surprise, and just as bad.

⚠️ **TWO LATENCY REGIMES, AND NEITHER IS SUB-STEP.** The hold is taken on the supervisor **tick** that scans the bus, never inside the solver sub-step. Measured 2026-09-22 (machine `9722d23d12a3`, `basicTimeStep` 8 ms, three-body `break_drop` fixture): **held and driven by `/sim/step`**, detection is per basic step — `hold_latency_ms` 0.0, `hold_latency_steps` 0, `hold_latency_engine_ms_max` 8.0, i.e. at most one basic step of engine time. **Free-running**, detection is one supervisor tick, and a tick is *not* a basic step — `engine_tick_ms` measured 8, 16, 48 and 128 ms across the four live cases, and a loaded full-tracking world costs ~600 ms per step. **Free-running can miss the event entirely:** on that fixture the engine outran the controller so far that a whole one-second drop fitted inside one tick and no `contact.began` was emitted at all, so the live case records `fired: false`. **The reliable workflow is the debugger one: pause, then step.** Every hit carries its own `hold_latency_ms` (the supervisor-clock delta, ~0 by construction and flattering on its own) and `hold_latency_engine_ms_max` (= `engine_tick_ms`, the number that bounds the real thing). Read them; do not assume sub-step precision. PROTOCOL.md [§7.40](../../PROTOCOL.md#740-post-simbreak-get-simbreaks-delete-simbreakbreak_id).

## POST /sim/step

**Route:** `POST /sim/step {steps?}`

Advance the simulation N basic timesteps (default 1).

⭐ **Since v9 a batch STOPS EARLY on the step an armed break fires**, which makes it *continue-to-breakpoint*: `{"steps": 400}` came back `steps_executed: 118, stopped_on_break: "brk2"`. ⚠️ **`steps_executed` used to be the request echoed back** — it is now what the supervisor actually ran, and `steps_requested` is the echo. The response also carries `engine_time_ms`, `engine_advanced_ms`, `requested_advance_ms`, `paused` and `lease_remaining_ms`. ⚠️ **A step under a held pause is exact on the SUPERVISOR clock and NOT on the engine clock:** `{"steps": 10}` advanced the supervisor clock by exactly 80 ms on every call and the engine clock by 80, 88, 104 or 112 ms — 0 to 4 basic steps of overshoot — because lifting the pause, stepping and re-queuing the pause is a race the supervisor binding cannot close. That is why the response **reports** `engine_advanced_ms` rather than asserting it; a test that pins an exact engine advance will flake, and one did (`/sim/step 10 moved the ENGINE clock by 96.0 ms, outside [80.0, 88.0]`).

## POST /sim/reset

**Route:** `POST /sim/reset {restore?, verify?, settle_steps?}`

Rewind the clock to t=0 **and restore the scene**, without re-parsing. It used to only rewind the clock and leave a fallen body where it fell; it now also loads the engine's own parse-time state `"__init__"` (`OmNode`'s constructor sets it, `OmPose`'s saves the authored pose under it — nothing has to be snapshotted first). `{"restore": null}` gets the old clock-only behaviour back.

## POST /sim/snapshot and restore

**Route:** `POST /sim/snapshot {name}` / `POST /sim/restore {name, settle_steps?}` / `GET /sim/snapshots`

Named engine-side state snapshots over `Node.saveState()` / `loadState()` from the scene root — recursive over the whole scene. `restore` puts the bodies back **without** rewinding the clock, and reports `verification.vs_snapshot.max_pose_delta_m` (top-level poses only; a body still falling legitimately reports non-zero). ⚠️ **Restoring an unknown name is refused on purpose** — the engine's saved-pose `QMap` default-constructs a **zero vector** on a miss, so an unguarded restore would teleport the whole scene to the origin; you get `404 SNAPSHOT_NOT_FOUND`. Names die with the world (the registry lives in the supervisor, which every load restarts); `__`-prefixed names are reserved. A supervisor-taken "initial" snapshot is **not** the authored state — the engine free-runs before the controller's first step (lane3's `BALL` is authored at z=1.0 and first reads z=0.1); use `/sim/reset` for authored.

## GET /sim/state

**Route:** `GET /sim/state`

Current world, supervisor connection, last load result.

⭐ **Since v9 it also carries the pause and break state, and BOTH clocks**: `paused`, `lease_remaining_ms`, `break_hit` (the last break that froze the engine, in the `break.hit` payload shape plus an `event` object carrying the matched event verbatim — it **survives the resume**, so it is also the record of *why* it was frozen), `breaks_armed`, and `engine_time_ms`. ⚠️ **`paused` is `null`, not `false`, when nobody could be asked** (no supervisor, or a load in flight) — do not branch on it as a boolean. ⚠️ **`sim_time_ms` and `engine_time_ms` are DIFFERENT RULERS.** `sim_time_ms` is the injected supervisor's per-iteration counter, and it is what stamps every *other* harness response, so it is the clock those numbers are comparable to; `engine_time_ms` is the engine's own clock as of that controller's last step. In `--mode=fast` the engine free-runs independently of the controller loop and runs far ahead — measured 2026-09-22 on a three-body world, a `/sim/reset` returned with the boxes re-lifted at supervisor t = 8 ms and the very next tracker poll (one supervisor tick later, supervisor t = 16 ms) already saw both at rest on the floor: about **700 engine-ms inside one supervisor tick**. Quote `engine_time_ms` for "how much simulated time has passed"; ⚠️ it does **not** rewind on `/sim/reset`. A reader who sees `sim_time_ms` standing still needs `paused` from the *same* response, or the honest reading of a frozen clock is "the simulator hung". PROTOCOL.md [§7.39](../../PROTOCOL.md#739-get-simstate--the-pause-and-break-fields).

## GET /sim/contacts

**Route:** `GET /sim/contacts`

Global contact set: `[{a_def, b_def, point, paired}]` **plus a `tracking` block** (what was walked, which bodies are idle). ⚠️ **`?wake=1` is a TOTAL no-op — it advances nothing and costs nothing; drop it anyway.** It existed because ODE auto-disabled a body idle for `WorldInfo.physicsDisableTime` and a disabled body generated no contacts, so a crate demonstrably resting on a floor returned `[]`. Newton has no body sleep, and native contact readback has been on by default since 2026-08-07, so a resting body reports its contacts without help. The two settle steps it used to take were **deleted**, not merely re-worded, so the read is idempotent again and can rejoin the harness's transparent-retry set — measured, `tracking.woken` reports `applied: false, steps_advanced: 0`. ⚠️ Do not infer stepping from the clock: the wrapped engine **free-runs between HTTP calls** (measured 88–112 ms of sim time per idle `/sim/state` poll with no other call), so `sim_time_ms` moves across any pair of requests and a non-zero delta around a read proves nothing about that read.

## GET /sim/grips

**Route:** `GET /sim/grips`

Inferred grips: `[{gripper_def, held_def, since_t_ms}]`.

## GET /sim/events

**Route:** `GET /sim/events?since=&log_since=&limit=&types=`

Unified runtime event stream — supervisor-side (`contact.*`, `joint.limit_hit`, `grip.*`, `damage.*`, **`break.hit`**) and harness-side (`controller.log`, `world.warning`, `world.error`) merged. Two cursors (`since` for sup, `log_since` for log). **Eleven types since v9**; `break.hit` is the eleventh and is emitted through the same `emit()` call form the capability self-scan reads, so it is counted by code and the `declared_not_emitted` drift check stays green. It is a **record of a freeze**, not a physics event: `matched` carries the underlying event verbatim (minus its own `seq` and `type`) and `matched_seq` points back at it on the same cursor. ⚠️ Read `hold_latency_engine_ms_max`, not `hold_latency_ms` — see the `/sim/break` row.

## GET /robots

**Route:** `GET /robots`

Enumerate every Robot in the scene with pose and joint count.

## GET /robot/<def>/joints

**Route:** `GET /robot/<def>/joints`

Per-joint snapshot: position, velocity (differenced), limits, `hit_limit`.

## GET /robot/<def>/devices

**Route:** `GET /robot/<def>/devices`

List devices visible in the robot's subtree.

## POST /robot/<def>/joints/set

**Route:** `POST /robot/<def>/joints/set`

⭐ **The first robot-commanding endpoint (2026-08-19).** `{"joints": {"<name>": <rad>}, "settle_steps"?}` — supervisor joint targets with settle-and-verify: the write is a PD setpoint that converges over ticks (default settle 16 steps), so each joint returns `{commanded, achieved, error, moved, clamped, position_controllable, limits}` measured, never echoed. ⚠️ A limit-less motor (no `minPosition`/`maxPosition`) is a ke=0 velocity wheel whose `setPosition` is ignored — the verb pre-classifies and reports `position_controllable: false` instead of lying. ⚠️ An active bridge in hold mode re-asserts its own targets every tick and WINS (measured 0.42–0.70 rad residual on the UR5e) — command bridge-owned robots through their bridge; this verb owns passive/supervisor-only robots. PROTOCOL.md §7.33.

## POST /robot/<def>/ik

**Route:** `POST /robot/<def>/ik`

⭐ **Batched IK against the exact model the solver steps (2026-08-19)** — `{"effector": "<DEF>", "targets": [[x,y,z],...], "tool_offset"?, "iterations"?}`, pure PREVIEW (nothing moves; apply the returned angles via `/joints/set`, whose joint names the response maps to). Per-target `residual` in metres from the solver's own FK — an unreachable target reports its true residual, never "reached". Closed-loop verified: Cartesian error 1.0e-05–2.2e-04 m on a passive rig; ⚠️ first call in a fresh world compiles a warp kernel (~2.4 s warm disk cache, 8.3 s truly cold; ~110 ms after — `verification.warmup` discloses it). Hinge/Slider joints only (Ball/Hinge2 excluded, multi-coordinate); `mujoco_warp` unverified. PROTOCOL.md §7.34.

## GET /robot/<def>/sensor/<name>

**Route:** `GET /robot/<def>/sensor/<name>`

501 by design — supervisor can't read sensors it doesn't own; use `/joints` or a per-robot helper.

## GET /robot/damage

**Route:** `GET /robot/damage`

Damage state of the tracked robot (per-part HP / state).

## GET /robot/damage/events

**Route:** `GET /robot/damage/events?since=&limit=`

A filtered view of the `damage.*` events — same records the unified `/sim/events` stream carries, with their own cursor.

## POST /robot/damage/reset

**Route:** `POST /robot/damage/reset`

Heal every part **without** resetting the simulation.

## POST /robot/damage/inject

**Route:** `POST /robot/damage/inject {part, state?, hp_delta?}`

Set a part's damage state directly — the fault-injection verb, so a damage-response path can be tested without staging a collision.

## GET /healthz

**Route:** `GET /healthz`

Liveness — does not touch the simulator.
