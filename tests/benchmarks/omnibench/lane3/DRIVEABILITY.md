# OmniBench lane 3c — agent-driveability probe list

**What this measures.** Whether a coding agent can *drive* the simulator
programmatically — load and edit worlds, inspect state, verify camera framing,
recover structured errors — without a human or a GUI in the loop. It is a
property of the simulator's agent-facing surface, not of its physics.

**Scoring.** 10 machine-checkable probes, each pass/fail plus wall-clock
latency (the latency of *everything* the probe does, all HTTP round-trips
included). Score = probes passed / 10. Runner:
[`driveability.py`](driveability.py) (starts its own harness on non-default
ports, tears it down after). Results: one SPEC row per probe + a summary row in
`results/driveability.jsonl`.

**Hand-scoring another simulator.** Each probe below states its *capability
contract* in simulator-neutral terms. Score a competitor by asking: can a
program (not a person) do this against the sim's public API, and how would it
verify success *from the response alone*? Partial credit is not a thing — a
capability that requires a human to eyeball something is a FAIL for that probe.

The world used is deliberately tiny ([`worlds/lane3_drive.wbt`](worlds/lane3_drive.wbt):
a static box `TARGET`, a falling ball `BALL`, a 1-joint pendulum robot `BOT`)
so latencies measure the surface, not asset streaming. The runner copies it to
a temp dir and edits only the copy.

## The probes

| # | probe id | capability contract | pass criterion (machine-checked) |
|---|---|---|---|
| 1 | `load_valid_world` | Load a scene file via API and get a **structured** success signal. | `POST /world/load {path}` returns HTTP 200 JSON with `ok: true` (or a pollable in-progress state that resolves to connected). |
| 2 | `hot_reload_edited_world` | Edit the scene file on disk and reload **without restarting the simulator**, fast enough for an iteration loop. | Re-`POST /world/load` of the edited file returns `ok: true` in **< 30 s**, and the edit is *observable* in the reloaded scene (the moved object's new pose appears in the scene tree). |
| 3 | `scene_tree_poses` | Enumerate the scene programmatically: every node with its type and **world pose**. | `GET /scene/tree` returns a node list where nodes carry a 3-vector `position`. |
| 4 | `scene_tree_bounds` | Get **world-space geometric bounds** (AABB/center/radius) per node — the number every camera/placement decision needs. | `GET /scene/tree?bounds=1` returns nodes carrying `bounds.bbox_min` / `bbox_max`. |
| 5 | `sim_step_deterministic` | Advance the simulation by an exact API call and get **reproducible dynamics**: two trials from the same initial state end in the same state. | Twice: (reload world → `POST /sim/step {steps:50}` → read `BALL` pose). Ball must have *moved* from its authored pose (dynamics ran) and the two rest poses must agree to **< 1e-9 m**. |
| 6 | `events_cursor_stream` | Poll a **cursor-based runtime event stream** (contacts, joint limits, controller logs) with no events lost between polls. | `GET /sim/events?since=0&log_since=0` returns `events[]` plus `next_since` / `next_log_since` cursors, and a follow-up poll from those cursors succeeds. |
| 7 | `robot_joints_state` | Read **per-joint state** of a named robot (position at minimum) without writing a controller. | `GET /robot/BOT/joints` returns ≥ 1 joint, each with a `position` field. |
| 8 | `scene_frame_verified` | Aim the camera at a named object and receive a **numeric proof** it is in frame — not "trust me", but margins an agent can branch on. | `POST /scene/frame {def}` returns a `verification` block with `fits: true` and numeric `headroom_h_deg` / `headroom_v_deg` / `subject_angular_radius_deg`. |
| 9 | `screenshot_png` | Render the current view to a **decodable image** over the API (headless — no GUI window). | `POST /world/screenshot` returns bytes with a valid PNG signature that a decoder (Pillow) verifies. |
| 10 | `broken_world_structured_diagnostic` | A syntactically broken scene file fails with a **machine-branchable diagnostic code**, and the sim does not misreport the broken world as healthy. | Loading a deliberately-broken `.wbt` yields diagnostics whose `code` ≠ `UNKNOWN` (e.g. `WORLD_PARSE_SYNTAX_ERROR`), and the sim's state endpoint does not claim a completed, supervisor-connected load. |

## Measured findings on OmniSim (2026-07-24, machine 9722d23d12a3)

Recorded in the result rows' `deviations`; kept here because they are surface
facts a user of the harness should know:

- **Score 10/10.** Engine attribution: the Newton backend drove the probe
  world (verified via the `.newton.json` sidecar next to the harness's engine
  log).
  > **Reading the `engine` field on these rows (changed 2026-08-08).** The
  > sidecar is the *only* thing that decides the label. When it is missing or
  > reports `degraded`, the row is written `omnisim-unverified` and carries the
  > reason in `deviations` (prefix `engine=omnisim-unverified:`); `run_all.py`
  > then raises it as an `[attribution]` finding. It used to default to
  > **`omnisim-ode`**, which after the deletion of `src/ode` (commit `bdc02139`)
  > named an engine that does not exist — a sidecar-less row was published as an
  > ODE measurement and no consumer could tell it from a real one. A missing
  > sidecar means the load never reached world-finalize, not that another
  > backend ran.
- **`/sim/reset` used to rewind sim time WITHOUT restoring node state** (a
  fallen ball stayed fallen), so probe 5 was written around a world (re)load,
  which was then the only working state-reset primitive on this surface.
  **That is fixed** (`25fbc755`): `/sim/reset` now rewinds the clock **and**
  restores the engine's own parse-time state `"__init__"`, verified on both
  backends. The probe deliberately still uses the reload path — switching it
  to reset would silently change what the 10/10 score measures across the fix
  boundary, so the score stays comparable either side of it.
- **The sim free-runs between harness RPCs** (the injected supervisor steps the
  world while polling for commands), so "exactly N steps from state X" is not
  expressible over this surface; probe 5 compares settled rest states, which
  absorbs the timing noise. Strict trajectory determinism is lane 3a's job
  (controller-side recording; graded `bitwise` there).
- **`/sim/step` costs ~0.7–0.9 s per 16 ms step** in even a 17-node world —
  the per-step damage/contact/grip polling in the injected supervisor, not
  physics. Probe 5's ~70 s latency is that cost.
- **A broken-world load takes ~4 min to resolve** (hot-reload attempt, cold
  engine relaunch retry, then the supervisor-bind stall detector), and the
  synchronous response can be `ok: true, load_state: "in_progress"` with the
  structured codes arriving via `GET /world/diagnostics` — an agent must poll
  `/sim/state` rather than trust the first response. Probe 10 passes on the
  structured codes + honest final state, but the latency is the finding.
