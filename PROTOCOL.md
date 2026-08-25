# OmniSim Wire Protocol

This document specifies the **OmniSim Wire Protocol** — the over-the-wire
contract between an OmniSim simulator instance and any agent, controller,
or external system that drives or observes it. It is what is meant when
OmniSim "speaks OmniSim."

This file is normative. Where this document and the code disagree, the
code wins for the current release and this document is updated in the
same change — and the disagreement is treated as a wire-protocol bug,
not a documentation bug. Per-bridge READMEs may describe extensions
beyond what this document covers, but anything declared *required* here
must be honoured.

> **Reconciled against the code on 2026-07-26.** §7, §8 and §10 had
> drifted: eight implemented harness endpoints were undocumented, four
> event-type names did not exist, and several documented response fields
> were never returned. Everything below was re-derived from the route
> registrations and event emitters rather than from the previous edition;
> where behaviour is wrong but shipped, this document now describes the
> shipped behaviour and points at the proposed fix in
> [docs/developer/agent-native-api.md](docs/developer/agent-native-api.md)
> rather than describing the aspiration. Read §16 before depending on any
> harness endpoint. The structural fix — serving these lists from the
> code — is proposal P1 in that document.

| Field | Value |
|---|---|
| Protocol name | `omnisim_wire` |
| Version | `1.0` |
| Specification status | Stable for the three implemented surfaces (Robot Bridge, World Harness, Capture Service). §9 Twin Shadow is **reserved and unimplemented**. See §16 for the open compliance gaps. |
| Reference simulator versions | OmniSim ≥ 2.0.0 |
| Transport | HTTP/1.1 + JSON over loopback TCP |
| Encoding | UTF-8 |
| Source of truth | this file |

The OmniSim wire protocol is versioned independently of the simulator
binary. A given simulator release declares which protocol versions it
speaks (see [§13 Compatibility](#13-compatibility-and-version-negotiation)).
Wire-protocol semver follows the standard rules:

- **Patch** (`1.0` → `1.0`): no wire change; clarifying language only.
- **Minor** (`1.0` → `1.1`): additive only. New endpoints, new optional
  fields, new event types, new fault codes. Older clients keep working
  unmodified.
- **Major** (`1.0` → `2.0`): allowed to break older clients. Requires a
  new specification document; both old and new are supported in parallel
  for one full minor cycle before the old one is retired.

Tooling that depends on the OmniSim wire protocol should pin to a major
version and negotiate minor versions at runtime.

---

## Table of contents

1. [Surfaces](#1-surfaces)
2. [Transport and encoding](#2-transport-and-encoding)
3. [Common envelope and error model](#3-common-envelope-and-error-model)
4. [Version negotiation](#4-version-negotiation)
5. [Robot Bridge — required endpoints](#5-robot-bridge--required-endpoints)
6. [Robot Bridge — per-class endpoints](#6-robot-bridge--per-class-endpoints)
7. [World Harness](#7-world-harness)
8. [Capture Service](#8-capture-service)
9. [Twin Shadow](#9-twin-shadow-reserved--not-implemented) *(reserved — not implemented)*
10. [Event taxonomy](#10-event-taxonomy)
11. [Fault codes](#11-fault-codes)
12. [Multi-instance and port allocation](#12-multi-instance-and-port-allocation)
13. [Compatibility and version negotiation](#13-compatibility-and-version-negotiation)
14. [Stability commitment](#14-stability-commitment)
15. [Out of scope](#15-out-of-scope)
16. [Reference implementations](#16-reference-implementations)
17. [Hardware-in-the-loop (MAVLink)](#17-hardware-in-the-loop-mavlink)

---

## 1. Surfaces

The OmniSim wire protocol defines four surfaces, each independently
versioned under the same overall protocol version. A given simulator
instance MAY expose any subset of them.

| Surface | Purpose | Default port | Owner |
|---|---|---|---|
| **Robot Bridge** | Agent ↔ robot. The primary live-control surface. One bridge per controllable robot (or per scene of robots). | `8765` (`6060` legacy single-arm) | A robot controller process inside the simulator |
| **World Harness** | Agent ↔ scene authoring. World load, hot-reload, scene-tree inspection, screenshot, run-state event stream. | `6789` (supervisor IPC on `6790`) | `scripts/harness/omnisim_harness.py` |
| **Capture Service** | Agent ↔ cinematic output. High-resolution stills, deterministic camera-path sequences, movie encoding. | `6791` (supervisor IPC on `6792`) | `scripts/capture/omnisim_capture.py` |
| **Twin Shadow** ⚠️ *reserved — not implemented* | Real robot ↔ simulator. Would let the simulator hard-snap a robot's joint state from an external telemetry source for digital-twin and replay use cases. Layered on top of a Robot Bridge. | Same port as the Robot Bridge it extends | A bridge controller that implements shadow mode |

These are independent surfaces; a tool may speak any one of them in
isolation. The required-vs-optional split below makes "Robot Bridge
v1.0 compliant" testable independently of whether the same simulator
exposes a harness.

> ⚠️ **Twin Shadow (§9) is a reserved design, not a shipped feature.**
> **No bridge in this repository implements it**, and OmniSim makes **no claim
> of validated sim-to-real transfer**. §9 is published so that the endpoint
> names and payload shapes are pinned before anyone builds against them — treat
> it as a proposal, not as a surface you can call today. The other three
> surfaces are implemented; see §16 for their exact compliance gaps.

---

## 2. Transport and encoding

- **Protocol:** HTTP/1.1.
- **Default host:** `127.0.0.1` (loopback). Canonical robot bridges refuse a
  non-loopback bind unless `OMNISIM_BRIDGE_TOKEN` is configured. Deployments
  that leave the host MUST additionally provide TLS at a reverse proxy or
  equivalent trusted transport boundary.
- **Request bodies:** JSON, `Content-Type: application/json; charset=utf-8`.
  Empty bodies are permitted on requests that have no parameters.
- **Response bodies:** JSON unless explicitly noted (screenshots are
  `image/png`, sequence captures stream `application/octet-stream`).
- **Numeric precision:** All floats are IEEE 754 double. Angles are
  **radians** unless the field name explicitly ends in `_deg`.
  Positions are **metres**. Sim time is **seconds**.
- **Coordinate frame:** Right-handed ENU (`+X = east`, `+Y = north`,
  `+Z = up`) at the world level. Arm-local `xyz` fields are in the
  arm's base frame and are explicitly distinguished from world-frame
  `xyz` by per-endpoint naming (`tcp_world` vs `tcp_arm_local`).
- **Timestamps:** Two clocks are exposed and never conflated.
  - `sim_time` (seconds, float) — wall-time of the simulation, monotonic
    within a single run, resets to `0.0` on world load.
  - `wall_time` (seconds since Unix epoch, float, optional) — real-time
    of the simulator host. Optional; required only for shadow-mode
    timing math.
- **Idempotency:** Every `GET` endpoint is side-effect free. `POST`
  endpoints that semantically describe queries (`/get_robot_state`,
  `/read_joints`, `/list_robots`) are also side-effect free. Mutating
  `POST`s are NOT automatically retry-safe; callers that need at-most-
  once semantics MUST supply an `id` field (see §3) and use it to
  deduplicate on retry.

---

## 3. Common envelope and error model

### 3.1 Success envelope

Action endpoints respond with either:

- A body whose shape is specific to the action (see per-endpoint docs), OR
- The standard ok-envelope:

```json
{ "ok": true, "result": { "...": "..." } }
```

Both shapes are valid. The ok-envelope is preferred for new endpoints
because it makes "did the call succeed" trivially extractable; existing
endpoints that omit it remain valid v1.0.

### 3.2 Error envelope

Errors MUST use the standard envelope:

```json
{
  "ok": false,
  "error": "<error_code>",
  "message": "<human-readable message>",
  "details": { "...": "..." }
}
```

The `error` field is a stable, lowercase, snake_case **error code** from
§11. The `message` field is human-readable and MAY change between
releases without bumping the protocol version. The `details` object is
endpoint-specific and unstable.

HTTP status codes:

- `200 OK` for success.
- `400 Bad Request` for malformed JSON, missing required fields, or
  out-of-range values.
- `404 Not Found` for unknown endpoint or unknown robot id.
- `409 Conflict` for state errors (robot busy, world not loaded, mode
  conflict). The body MUST include the structured error envelope.
- `500 Internal Server Error` for unexpected failures.
- `501 Not Implemented` for "feature exists in the protocol but not in
  this bridge."
- `503 Service Unavailable` for "the simulator is alive but not ready
  yet" (world loading, controller not yet connected).

Note: `500` and `501` look similar but mean different things — `501`
is a structural "this implementation chose not to support that feature"
(used for example by the harness's `/robot/<def>/sensor/<name>`), while
`500` is an unexpected crash that callers should retry or report.

### 3.3 Request id and idempotency

Mutating `POST` requests MAY include an `"id"` field (string, ≤128
chars). When present, the receiver:

- MUST retain the request id for at least 5 seconds.
- MUST either return the original response or return `409 Conflict` with
  `error = "duplicate_request"` for a repeated id within that window.
- MUST NOT re-apply the underlying mutation.

This prevents an agent loop from commanding the robot twice after a lost
response. On `duplicate_request`, inspect robot state rather than sending a new
id until the prior action's outcome is known. Implementations that do not yet
track request ids MUST still accept the field (and ignore it) so callers can
send it uniformly across bridge versions.

---

## 4. Version negotiation

### 4.1 GET /protocol

> **Status:** implemented by the canonical package bridge and the arm, mobile,
> quadruped, Husky, and Mavic robot bridges. The world harness and capture
> service do not expose it yet; those two surfaces remain open compliance gaps.

Every compliant service exposes `GET /protocol`:

**Request:** no body.

**Response (200):**

```json
{
  "ok": true,
  "omnisim_wire": "1.0",
  "service": "robot_bridge",
  "service_versions": { "robot_bridge": "1.0" },
  "instance": {
    "name": "omnilink_mobile_bridge",
    "robot_id": "husky",
    "world": "projects/samples/demos/worlds/chat/omnilink_husky.omniworld",
    "sim_version": "2.0.0",
    "pid": 12345
  },
  "extensions": []
}
```

Fields:

- `omnisim_wire` — the highest protocol major.minor this service
  implements. SemVer string.
- `service` — one of `robot_bridge`, `world_harness`, `capture_service`,
  `twin_shadow`. The surface this endpoint represents.
- `service_versions` — map of service-name → spec version. A bridge
  that ALSO implements twin shadow declares both here.
- `instance` — diagnostic identity. `world` MAY be absent on the
  harness before a world is loaded.
- `extensions` — array of vendor extension strings (see §14.4). Stable
  cross-vendor extensions land in a future minor; vendor-specific
  experiments stay under a `x-<vendor>-` prefix.

### 4.2 Accept-Protocol-Version request header

Clients MAY send `Accept-Protocol-Version: <major>.<minor>` to indicate
the protocol version they expect. If the service cannot satisfy that
version it MUST respond `409 Conflict` with `error = "protocol_unsupported"`.
Services that do not yet honour the header MUST ignore it (the rest of
the response is unchanged); callers MUST treat absence of `409` as
"the version is acceptable."

### 4.3 Server response headers

> **Status:** emitted by the canonical package bridge and the main robot
> bridges. The harness and capture service do not emit them yet.

The target: every response from a v1.0-compliant service includes:

```
X-OmniSim-Wire: 1.0
X-OmniSim-Service: robot_bridge
```

These are advisory — clients SHOULD use them for logging but MUST NOT
fail closed if they are absent (compatibility with non-v1.0 deployments
during transition — which today is *all* of them).

---

## 5. Robot Bridge — required endpoints

A **Robot Bridge** is a service that exposes one or more robots for
live agent control. Required endpoints below MUST be implemented by
every v1.0-compliant bridge regardless of robot class.

### 5.1 GET /protocol

See §4.1. `service = "robot_bridge"`. The `instance.robot_id` MUST be
the robot id used everywhere else on this bridge; for multi-robot
bridges (e.g. assembly-line orchestrators) it MAY be omitted from
`/protocol` and discovered through `/list_robots`.

### 5.2 GET /capabilities — also POST /list_robots

**Request:** no body.

**Response (200) — single-robot bridge:**

```json
{
  "ok": true,
  "robot_id": "arm_1",
  "model": "arm6",
  "class": "arm",
  "dof": 6,
  "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
  "joint_limits": [[-3.14, 3.14], [-3.14, 3.14], ...],
  "home_pose": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
  "tick_period_s": 0.032,
  "actions": ["stop", "reset_home", "set_joint_positions", "set_tcp_target", "solve_ik"],
  "workspace": { "kind": "shell", "r_min": 0.1, "r_max": 0.85, "z_min": 0.0 },
  "ik": { "max_iters": 20, "tol": 1e-3, "damping": 0.08, "max_dq": 0.08 },
  "gripper": { "available": false }
}
```

**Response (200) — multi-robot bridge:**

```json
{
  "ok": true,
  "robots": [
    { "robot_id": "arm_a", "model": "arm6", "class": "arm", ... },
    { "robot_id": "arm_b", "model": "arm6", "class": "arm", ... },
    { "robot_id": "arm_c", "model": "arm6", "class": "arm", ... }
  ]
}
```

Required fields:

- `robot_id` — stable identifier; lowercase ASCII, snake_case, no spaces.
- `model` — robot model string (`husky`, `mavic_2_pro`, `go2`, …).
- `class` — one of `arm`, `mobile`, `flying`, `quadruped`, `humanoid`,
  `manipulator_on_mobile`, `parallel`. Drives which §6 endpoints apply.
- `dof` — degrees of freedom relevant to the class. For arms this is
  the joint count; for mobile bases it is 2 (linear + angular).
- `joint_names` and `joint_limits` for arms/manipulators (omit for
  pure-mobile robots).
- `actions` — array of action verbs this bridge will accept via
  `POST /action`. Clients SHOULD validate intended actions against
  this list before sending.
- `tick_period_s` — the simulator's basic timestep in seconds. Used
  by clients to size retry/poll loops sensibly.

Optional but standardized:

- `home_pose` for arms (radians) or `home_xyz_yaw` for mobile bases.
- `workspace` describing the reachable envelope (`kind ∈ {"shell", "box", "cylinder"}`).
- `ik` constants for arms.
- `gripper`, `camera`, `lidar` capability blocks; each declares
  `available: true|false` so clients can branch.

`POST /list_robots` MUST return the same body and is provided for
clients that prefer a POST-only verb surface (the Axis convention).

### 5.3 GET /state — also POST /get_robot_state

**Request:** no body.

**Response (200) — class-agnostic envelope:**

```json
{
  "ok": true,
  "robot_id": "arm_1",
  "sim_time": 12.480,
  "last_tick_at": 12.448,
  "mode": "idle",
  "fault": null,
  "state_source": "sim"
}
```

Additional fields by class — present iff the robot is of that class:

- **arm / manipulator:**
  ```json
  { "q": [6 floats], "commanded_q": [6 floats],
    "tcp_world": [x, y, z], "tcp_arm_local": [x, y, z],
    "target_xyz": [x, y, z] | null }
  ```
- **mobile:**
  ```json
  { "x": 1.20, "y": -0.35, "yaw": 1.5708,
    "v_linear": 0.0, "v_angular": 0.0 }
  ```
- **flying:**
  ```json
  { "x": ..., "y": ..., "z": ..., "roll": ..., "pitch": ..., "yaw": ...,
    "v_xy": ..., "v_z": ...,
    "target_altitude_m": ..., "gimbal_pitch_rad": ... }
  ```
- **quadruped, humanoid, parallel:** class-specific. The required
  fields are `mode`, `sim_time`, `last_tick_at`, and `state_source`;
  everything else is class-dependent.

The `mode` field is a free-form string describing the high-level
behaviour (`idle`, `interpolating`, `tcp_tracking`, `velocity`,
`drive`, `takeoff`, `hover`, `goto`, `shadowed`, …). Clients SHOULD
NOT branch on its exact value; it is for telemetry, not control.

The `fault` field is `null` when healthy or an object
`{ "code": "<error_code>", "message": "...", "since_t": 12.0 }` when
the bridge has declared the robot faulted. See §11.

`state_source` is one of `"sim"`, `"shadow"`, `"shadow_stale"`. Bridges
that do not implement twin shadow always return `"sim"`.

`POST /get_robot_state` MUST return the same body. For multi-robot
bridges, both verbs MAY accept a `{ "robot_id": "..." }` query body or
return the array form.

### 5.4 POST /action — the typed action dispatch

The canonical typed mutating endpoint for bridges that adopt it.

> **Implementation status.** `POST /action` is implemented today by the
> **mobile** bridge ([`husky_omnilink_bridge`](projects/samples/demos/controllers/husky_omnilink_bridge/))
> and the **flying** bridge ([`mavic_omnilink_bridge`](projects/samples/demos/controllers/mavic_omnilink_bridge/)).
> The **arm** bridge ([`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/))
> does **not** expose `/action` — it offers the Axis-style one-verb-per-
> endpoint surface (§6.1) only. v1.0 keeps `/action` as the target a bridge
> SHOULD offer (alongside, or in place of, the Axis verbs per §6), but a
> bridge that ships only the Axis surface is still usable today.

**Request:**

```json
{
  "id": "optional-idempotency-key",
  "robot_id": "arm_1",
  "action": "set_joint_positions",
  "q": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
}
```

- `action` is required, MUST appear in `/capabilities.actions`.
- `robot_id` is required on multi-robot bridges, ignored on single-robot.
- `id` is optional (see §3.3).
- Additional fields are action-specific.

**Response (200):** action-specific. SHOULD include an `accepted: bool`
field so callers can branch on "did the bridge actually start executing
this" without parsing free-text. Long-running actions MAY return
immediately on acceptance and report progress via subsequent `/state`
polls — but `accepted` alone is **not** a sufficient result for an action
an LLM agent will call. See §5.4.1.

**Response (400):** `error = "invalid_action"`, `"missing_field"`, or
`"value_out_of_range"`.

**Response (409):** `error = "joint_limit"`, `"unreachable_target"`,
`"ik_nonconvergent"`, etc. — the action was understood but cannot be
performed.

### 5.4.1 Action result contract — achieved, not commanded

**Normative, and the most-violated part of this spec.** An LLM agent has no
independent access to the world: every belief it holds about what the robot
did came from an action result. A result that echoes the **commanded** value
back does not merely fail to help — it installs a false belief that the agent
then reports confidently and, from its own point of view, correctly.

The measured consequence in this tree: `POST /drive_forward {"distance": 1.0}`
returned `{"accepted": true, "distance": 1.0, "eta_s": 1.84}` in **0.01 s**,
at which instant the robot had travelled **0.019 m**. The agent has been told
the number it asked for, with no way to learn otherwise in that call. The
repo's own anti-fabrication study found 26% of agent turns contained a
fabrication, every one concerning the agent's own past actions, and **not one
originating from a tool read** — the model reports faithfully what the bridge
tells it.

Therefore, for any action that changes the robot's physical state:

1. A bridge **MUST NOT** report a commanded quantity under a field name that
   reads as a measurement. `{"distance": <the argument>}` is non-conformant.
   Echo arguments under an explicit `commanded` key or not at all.
2. A completed action **SHOULD** return
   `{"commanded": …, "achieved": …, "error": …, "settled": bool}` in the
   action's own units, measured against bridge-side ground truth.
3. If the bridge cannot measure the outcome, `achieved` **MUST** be `null` —
   never a value it did not measure. (Same rule as the benchmark suites:
   unmeasured is `null`, never `0.0`.)
4. An action that returns before completing **MUST** say so in its
   `/capabilities` description, in words, and **SHOULD** return a monotonic
   `seq` the caller can wait on. A wait primitive gated only on a mode string
   such as `idle` is non-conformant: it is a TOCTOU race that returns true for
   a robot which has not started moving.
5. A mutating action arriving while another is in flight **SHOULD** be
   rejected with `409 {"error": "busy", …}` rather than silently replacing it.
   A single-slot motion register that clobbers turns `turn` followed by
   `drive` — dispatched back to back with no delay by a model emitting two
   tool calls in one turn — into a drive on a barely-rotated heading, with
   both calls returning `accepted: true`.
6. Where a primitive is known to be inaccurate, the magnitude **MUST** appear
   in its description until it is fixed. Shipping a −43% actuator behind a
   description implying exactness is a protocol defect, not a robot defect.

**Conformance status, stated honestly:**

| bridge | 5.4.1 conformance |
|---|---|
| `omnilink_mobile_bridge` | ✅ **Full.** `wait: true` on `drive_forward` / `turn` returns `{commanded, achieved, error, unit, settled, timed_out}`; `get_robot_state.last_command` carries the same record; `drive_to(x,y)` is always-blocking and returns `achieved_xy` / `error_m` / `arrived` — and now `settled` only when no leg timed out, having stopped the robot before reading the final pose. `drive_forward.achieved` is the displacement **projected onto the start heading**, so a robot pushed backwards reports a negative number (it used to `copysign` a magnitude onto the commanded value, which asserted the direction rather than measuring it). The `409 busy` check lives in `_begin_motion`, so it covers **every** entry point — `POST /tool`, the offline intent router and the idle loop included, not just the three path routes — and a waiter now matches its own `seq` **exactly**, returning `achieved: null, superseded: true` instead of the clobbering motion's measurement. `capabilities` publishes `actions`, `blocking_actions`, `waitable_actions`, `busy_rejecting_actions`, `busy_overriding_actions` and `site_bounds_m`. ⚠️ Two published verbs deliberately do **not** reject when busy: `stop_robot` and `set_velocity` are the escape hatches, so they *cancel* the running motion — whose `achieved` then reports `null` — and both say so in their descriptions (rule 6). |
| `husky_omnilink_bridge` | ◐ Partial — satisfies (1)–(3) on `drive_forward` / `turn`: both honour `wait` and return `{commanded, achieved, error, unit, settled}` measured against supervisor pose, `achieved` is `null` when not waited on, and `drive_to_waypoint?wait=true` returns `final_pose` + `distance_remaining_m` (commit `f57d910e`). **`turn` is fixed** for \|angle\| ≥ π: it tracks a signed **accumulated** residual instead of an absolute `wrap_pi` target, and reports `achieved` / `error` **unwrapped**. Previously the target wrapped onto the current yaw, so `turn(6.283185)` never moved the robot and answered `achieved: -0.000000, error: 0.000000, settled: true`, while `turn(3.5)` rotated **−2.783 rad — the short way, the opposite sign** — and reported a near-zero error, because a wrapped error cannot express an under-rotation past half a turn. A reply that cannot attribute a measurement to its own command `seq` now returns `achieved: null` with `measured_from: "unattributed"`. **Still missing:** (4) in part — the "NOT complete" warning is in the response body, but `/capabilities` publishes no `actions` list and therefore no per-action description; (5) busy-rejection entirely — a second motion command silently replaces the first; and (6) — the `drive_forward` accuracy figure below appears nowhere the model can read it. ⚠️ (1) is **violated by `drive_to_waypoint` without `wait`**, which answers `{"x": <target>, "y": <target>, "speed_m_s": …}` — the caller's own target under measurement-shaped keys, with no `commanded` key and no note that it returns before completing. ⚠️ `drive_forward` delivering **~65% of the commanded distance** (same early-stop-on-a-leading-pose class as `52f3f6ca`) is a **carried-over claim, not re-measured here** — it needs a live re-measurement before it is repeated or written into a description. |
| `mavic_omnilink_bridge` | ❌ **Non-conformant on its default path.** The row here used to claim `goto_waypoint` returns `final_pose`; it does not — the string `final_pose` does not occur anywhere in `mavic_omnilink_bridge.py`. `wait` defaults **false**, and the default reply is `{"x": tx, "y": ty, "altitude": target_alt}`: the caller's own target echoed under measurement-shaped keys, with no `commanded` key and nothing saying the flight has not happened yet — violating (1) and (4), the exact failure 5.4.1 exists to forbid. With `wait: true` it *is* mostly honest — `_wait_until_arrived` supplies measured `x` / `y` / `z` / `yaw` plus `distance_remaining_m`, and because `**res` is spread last those measured values shadow the echoed target — but `altitude` still holds the **commanded** value beside the measured `z`, so one payload mixes commanded and measured under similar keys. Registers **no LLM tools at all** (`Tool(` occurs 0 times), so today nothing reads any of it; that is what keeps the defect latent, not fixed. |
| `omnilink_arm_bridge` | ❌ Not yet. |
| `omnilink_quadruped_bridge` | ❌ Not yet. |

Rationale, measurements and the external literature:
[docs/developer/tool-design-for-agents.md](docs/developer/tool-design-for-agents.md).

### 5.5 POST /stop_robot

Idempotent emergency stop. The bridge MUST hold the robot at its
current pose, clear any in-flight action, and respond
`{ "ok": true, "halted_at": <sim_time> }`. MUST always succeed
(returning `200`) even if the robot was already stopped — `/stop_robot`
is the only verb that callers can rely on without preconditions.

> **Status note.** The reference arm bridge currently returns the
> *pre-v1.0* body `{ "halted_at": ..., "q": [...] }` (no `ok` field).
> Bringing it up to the envelope above is part of the open conformance
> work (§16).

### 5.6 POST /reset_to_home

Drive the robot back to its `home_pose` (arms) or `home_xyz_yaw`
(mobile) over a bridge-defined interpolation time. Returns
`{ "ok": true, "accepted": true, "eta_s": <float> }` on acceptance.
On bridges where homing is meaningless (e.g. a stand-pose-only
quadruped) this returns `{ "ok": true, "accepted": true, "pose": "stand" }`.

> **Status note.** The reference arm bridge currently returns the
> *pre-v1.0* body `{ "q": [...] }` (no `ok`/`accepted`/`eta_s` fields).
> Bringing it up to the envelope above is part of the open conformance
> work (§16).

### 5.7 POST /prompt — required iff `prompt` ∈ actions

The natural-language convenience endpoint. Bridges MAY route through
a local intent router, an LLM, or omit this endpoint entirely
(reporting it as absent from `/capabilities.actions`).

**Request:**

```json
{ "text": "drive forward one meter" }
```

**Response (200):**

```json
{
  "ok": true,
  "response": "Driving forward 1.00 m (~11.2s).",
  "actions": [
    { "tool": "drive_forward", "result": "ok", "summary": "distance=+1.00 m" }
  ]
}
```

The `actions` array describes which tools the bridge actually invoked
in response to the prompt. Each entry MUST have `tool`, `result` ∈
`{"ok", "error"}`, and `summary` (string).

---

## 6. Robot Bridge — per-class endpoints

Per-class endpoints are required iff `class` matches and the
corresponding capability bit is `available: true`. A bridge MAY also
expose the same operation via `POST /action` with the action verb — the
direct endpoint exists for clients that prefer one-verb-per-endpoint
(the Axis convention); the typed `/action` exists for clients that
prefer a single dispatch point (the original arm bridge convention).
Both forms MUST produce identical state effects and identical responses.

### 6.1 Arm / manipulator

- `POST /set_joint_positions { "q": [N floats] }`
- `POST /servo_joint_positions { "q": [N floats] }` (optional; the streaming setpoint lane — see below)
- `POST /set_tcp_target { "xyz": [x, y, z] }` (requires IK)
- `POST /solve_ik { "xyz": [x, y, z] }` (no motion, returns `{q, err_norm, iters}`)
- `POST /read_joints` → `{ "q": [N floats], "commanded_q": [N floats], "joint_names": [...] }`
- `POST /read_tcp_pose` → `{ "tcp_world": [x,y,z], "tcp_arm_local": [x,y,z], "arm_origin_world": [x,y,z] }`

#### POST /servo_joint_positions — the streaming setpoint lane

`{"q": [<rad> × n_joints]}` → `{"accepted": true, "verb": "servo_joint_positions", "mode": "servo", "seq": <int>, "target": [<rad>…], "clamped": <bool>, "superseded_previous": <bool>, "preempted": <verb|null>, "updates": <int>, "achieved": null, "error": null, "note": "…"}`

Non-blocking, **last-write-wins**: it returns at dispatch, and a servo command sent while a servo stream is live retargets the stream in place — same `seq`, never a 409. A servo command arriving while a **goal** verb (`set_joint_positions`, `set_tcp_*`, `pick`, …) is in flight **preempts** it: the goal's waiter is released with `achieved: null` and the reply names the cancelled verb in `preempted`. The goal verbs themselves are unchanged — a goal sent while another goal *or a live servo stream* is running still gets the ordinary 409 `busy` (rule 5). This is the verb a trajectory controller (MoveIt's `joint_trajectory_controller`, streaming teleop) points at; under the goal contract its stream lands in pieces (measured: a second `set_joint_positions` 50 ms after the first answered 409 and was not applied).

Semantics (also machine-readable in `capabilities.servo`): `target` is the **adopted** (limit-clamped) vector, never the request echoed back, and never a measurement — `achieved` is always `null` in the reply because nothing has moved when it answers (rule 1). Each tick the bridge drives the motors at the latest target; the motors' own velocity limits bound the tracking rate, so a far target is a rate-limited move, not a jump. When the stream goes quiet (0.5 wall s, converged within 0.02 rad — or 5 wall s regardless), the motion parks and the **measured** result appears in `get_robot_state.last_command` under the stream's `seq`, like any other motion. Out-of-limit values are clamped (`clamped: true`), not refused; a wrong joint count is refused. Takes no `wait` (`non_waitable_actions` says why). Sim-only: nothing is forwarded to a hardware backend — use `set_joint_positions` for hardware moves.

Reference implementation: [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) (verified on the UR5e: 24 setpoints streamed at ~18 Hz, 24/24 accepted, zero 409s, parked max error 0.011 rad).

### 6.2 Mobile (wheeled / tracked)

- `POST /set_velocity { "linear": v_m_s, "angular": w_rad_s }`
- `POST /drive_forward { "distance": d_m, "speed": v_m_s? }`
- `POST /turn { "angle": rad }`
- `POST /drive_to_waypoint { "x": ..., "y": ... }` (optional; declare in capabilities)

### 6.3 Flying

- `POST /takeoff { "altitude": z_m?, "wait": bool?, "timeout_s": float? }`
- `POST /land { "wait": bool?, "timeout_s": float? }`
- `POST /hover`
- `POST /goto_waypoint { "x": ..., "y": ..., "altitude": z?, "yaw_to": rad? }`
- `POST /set_gimbal_pitch { "pitch_rad": float }`
- `POST /set_yaw { "yaw_rad": float }`

### 6.4 Quadruped

- `POST /set_pose { "pose": "stand" | "sit" | "crouch" | "settle" }`
- `POST /wave` (optional; declare in capabilities)

### 6.5 Gripper / end-effector (any class with `gripper.available = true`)

- `POST /open_gripper { "id"?: str }` → `{ "state": "open" }`
- `POST /close_gripper { "id"?: str, "force"?: float }` → `{ "state": "closed", "grasped": bool }`

### 6.6 Sensor read-through (optional)

This is the ONLY honest source of device data. `GET /robot/<def>/sensor/<name>`
on the World Harness returns **501 by design** (§7): OmniSim restricts device
APIs to the controller that owns the device, so a supervisor cannot read
another robot's IMU or lidar. The robot's own controller can, and a bridge is
that controller.

- `POST /read_sensor { "sensor": "<name>" }` →
  `{ "available": false, "note": "..." }`
  on bridges that don't proxy sensors; this is the structurally correct
  answer and it is a **200, not an error**. The same shape answers a sensor
  the robot does not carry, and SHOULD then list the ones it does in
  `known_sensors`.
  Bridges that do proxy sensors return
  `{ "available": true, "value": <...>, "unit": "..." }`.

- `POST /list_sensors {}` / `GET /list_sensors` →
  `{ "sensors": [ ... ], "count": N, "mount_frame": "robot_base", "mounts_measured": bool }`.
  Discovery, so a client never has to probe for device names. The same list
  SHOULD appear under `capabilities.sensors` (§5.3). Each entry:
  `{ "name", "type", "readable", "unit"?, "shape"?, "note"?, "mount"? }`,
  where `type` is the OmniSim device class (`InertialUnit`, `Gyro`,
  `Accelerometer`, `GPS`, `Lidar`, `DistanceSensor`, `PositionSensor`, ...),
  `shape` is the length of `value` or `"n"` when device-configured, and
  `mount` is `{ "translation": [x,y,z], "rotation_matrix": [9] }` giving the
  device's pose **in the robot's own frame** — or `null` when the bridge could
  not measure it. ⚠ `null` and a zero offset are different claims: a consumer
  building a TF tree must not publish an unmeasured mount as identity.

**Per-type `value` shapes.** v1.0 specified only a scalar `value` + `unit`,
which is not enough for a vector or a scan. The concrete shapes:

| `type` | `value` | `unit` | extra keys |
|---|---|---|---|
| `InertialUnit` | `[x, y, z, w]` | `quaternion_xyzw` | `roll_pitch_yaw` |
| `Gyro` | `[wx, wy, wz]` | `rad/s` | |
| `Accelerometer` | `[ax, ay, az]` | `m/s^2` | |
| `Compass` | `[x, y, z]` | `unit_vector` | |
| `GPS` | `[x, y, z]` | `m` | `speed`, `coordinate_system` ∈ `{local, WGS84}` |
| `Lidar` | `[r0 … rn]` | `m` | `layout`, `no_return_encoding` |
| scalar sensors | `<float>` | `lookup_table` / `rad_or_m` | |

`Lidar.layout` is
`{ horizontal_resolution, number_of_layers, fov, vertical_fov, min_range, max_range }`,
and `value` is **all layers concatenated**, length
`horizontal_resolution × number_of_layers`.

⚠ **`coordinate_system` is mandatory on a GPS read.** `local` means metres in
the world frame; `WGS84` means degrees. A consumer that assumes the wrong one
mislocates the robot by the whole planet.

⚠ **A no-return lidar ray is `+inf`, which is not valid JSON.** Bridges MUST
sanitise non-finite floats to `null` and say so via
`"no_return_encoding": "null"`. `null` therefore means "no return past
`max_range`" — **never** a zero-range hit, which is how a consumer reads `0.0`.

**Warm-up.** A device yields no data until it has been enabled AND one
simulation step has completed. A bridge enabling lazily on first read MUST
answer `{ "available": true, "value": null, "warming_up": true }` for that
window rather than substituting a zero, which would read as "level and
stationary" — a measurement nobody made.

Implemented by [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/);
consumed by the ROS 2 sidecar's `sensor_node` (§ROS 2, `packages/omnisim-ros2/`).

### 6.7 Class-agnostic perception helpers (optional)

- `GET /scan` — class-appropriate scene perception summary. Format is
  bridge-defined; the canonical examples are the drone marker scan
  (`{ "markers": [...], "frame_summary": {...}, "pose": {...} }`) and
  the assembly-line camera scan (`{ "view": "topdown", "tiles": [...] }`).
- `GET /image` — base64 PNG snapshot from the robot's primary camera.
  Body: `{ "width": int, "height": int, "encoding": "image/png; base64", "image_base64": str, "pose": {...}, "sim_time": float }`.
- `GET /read_camera` — ⚠️ **reserved name; not implemented.** Intended as an
  alias for `/image`, but no bridge serves it today. The in-tree spellings are
  `/image` (mavic) and `/camera` (husky). Call `/image`.

### 6.8 Mission brief (optional, multi-stage worlds)

- `GET /read_mission_brief` → `{ "brief": "<text>" }`
- `GET /mission` →
  ```json
  { "world_title": "...", "brief": "...", "complete": false,
    "log": ["..."], "hint": "..." }
  ```
- `POST /action { "action": "complete_mission", "rationale": "...", "payload": {...} }`

Mission briefs are the contract that distinguishes "controlled by a
script" from "controlled by an agent reading a prompt"; a world that
needs an agent SHOULD ship a brief.

---

## 7. World Harness

Service: `scripts/harness/omnisim_harness.py`. Default port `6789`,
supervisor IPC on `6790`. The harness wraps a headless OmniSim
subprocess and exposes endpoints for authoring, hot-reloading, and
inspecting worlds.

> **The route table below is the complete implemented set as of
> 2026-07-26** — 19 `GET` routes and 14 `POST` routes, and no `DELETE`.
> **Ask the harness rather than trusting this section: `GET /capabilities`
> (§7.28) returns the route list, and it cross-checks that list against the
> request handler's own source and reports any mismatch in
> `endpoints_verification`.** There is still no `GET /protocol` (§16).
>
> Equivalent grep, if you have the tree but not a running harness:
>
> ```bash
> grep -nE 'path == "|path in \(|base == "|parsed_early\.path == "|\.path\.startswith\("|suffix == "' \
>   scripts/harness/omnisim_harness.py
> ```
>
> (The `suffix ==` and `startswith` alternatives matter: `/robot/<def>/joints`,
> `/robot/<def>/devices`, `/robot/<def>/sensor/<name>` and `/scene/node/<def>`
> are matched by path *segment*, not by a literal string, so a grep for
> `path == "` alone silently misses four routes.)
>
> **Success bodies mostly do NOT carry `ok: true`.** Only `/healthz` and
> `POST /world/load` do. Every endpoint backed by a supervisor RPC returns
> the supervisor's raw result dict verbatim (`_supervisor_call` →
> `self._json(200, result)`), so callers MUST branch on the HTTP status
> and on the presence of the payload key they asked for, not on `ok`.
> This is permitted by §3.1 but it is not what several examples below
> used to imply; the examples have been corrected to the real bodies.
>
> **Error bodies are free text, not the §3.2 envelope** — see §16.

### 7.1 GET /protocol (PLANNED)

See §4.1 (not yet implemented by the harness). `service = "world_harness"`.
When shipped, `instance.world` is the path of the most recently loaded
world or `null` if none has been loaded. In the meantime
**`GET /capabilities` (§7.28) is the discovery endpoint that exists** — it
carries `omnisim_wire`, `service`, `sim_version`, the route list and the
current world.

### 7.2 GET /healthz

Liveness probe; does not touch the simulator subprocess.

**Response:** `{ "ok": true, "uptime_s": float }`.

### 7.3 POST /world/load

**Request:**

```json
{ "path": "projects/samples/demos/worlds/showcase/warehouse_husky.omniworld",
  "wait_s": 30.0,
  "with_supervisor": true }
```

**Response (200):**

```json
{ "ok": true, "world": "...", "load_ms": 1040,
  "exit_code": null, "supervisor": "connected",
  "hot_reloaded": true, "diagnostics": [] }
```

**Response (422 on a load failure; 400 on a malformed request):**

```json
{ "ok": false, "world": "...", "load_ms": 210,
  "diagnostics": [ { "code": "PROTO_NAME_MISMATCH", "path": "...", "line": 12 } ] }
```

Structured diagnostic codes. The authoritative list is
[`scripts/harness/diagnostic_codes.py`](scripts/harness/diagnostic_codes.py) — a
client MUST treat this enum as open and fall back gracefully on an unrecognised
code, because new codes are added there without a protocol major bump. The set
emitted today:

| Group | Codes |
|---|---|
| World file | `WORLD_WRONG_EXTENSION`, `WORLD_FILE_NOT_FOUND`, `WORLD_FILE_EMPTY`, `WORLD_PARSE_INVALID_TOKENS`, `WORLD_PARSE_SYNTAX_ERROR` |
| Header | `HEADER_MISSING`, `HEADER_INVALID` |
| PROTO | `PROTO_RECURSIVE`, `PROTO_BASE_NAME_INVALID`, `PROTO_NAME_MISMATCH`, `PROTO_PARAM_ERROR`, `EXTERNPROTO_DOWNLOAD_FAILED` |
| Assets | `ASSET_DOWNLOAD_FAILED`, `TEXTURE_READ_FAILED`, `MESH_READ_FAILED`, `URDF_MESH_UNRESOLVED` |
| Controller | `CONTROLLER_CRASHED`, `CONTROLLER_EXITED_NONZERO` |
| Launch | `LAUNCHER_DLL_NOT_FOUND` (Windows), `SIMULATOR_EXITED_NONZERO` |
| CUDA | `CUDA_*` (9 codes — see the source) |
| Fallthrough | `PARSE_ERROR`, `UNKNOWN` |

Hot reload is the behaviour of repeated `/world/load` calls. Agent authoring
clients SHOULD use `/world/sync` (§7.3a) after edits so pose-only changes can
avoid even that reload; `/world/load` remains the explicit full-reparse and
controller-restart primitive.

**⚠️ Known gap — a missing world file produces NO diagnostic code.** The
most common authoring failure is the one case the structured-diagnostic
path never reaches. `load_world()` short-circuits on `world.exists()`
before the engine is ever launched, so there is no engine log to parse
and the response is prose:

```json
422
{ "ok": false, "error": "world not found: O:\\omnisim\\nope.wbt",
  "diagnostics": [], "load_ms": 0 }
```

`WORLD_FILE_NOT_FOUND` **does** exist in
[`diagnostic_codes.py`](scripts/harness/diagnostic_codes.py) but is only
emitted when the *engine* reports a missing file (a `.wbt` that exists but
references a missing world, a bad `--` argument), never by this
precondition check. A client that branches on
`diagnostics[].code == "WORLD_FILE_NOT_FOUND"` will silently mis-handle a
typo'd path; branch on `ok === false && diagnostics.length === 0` too.
Proposed fix: [docs/developer/agent-native-api.md](docs/developer/agent-native-api.md)
G7 / P5 (`POST /world/validate`).

Two other real responses this section did not previously describe:

- **A concurrent load** returns `422` with
  `{ "ok": false, "error": "another /world/load is already in flight; retry when it returns", "load_state": "busy", "diagnostics": [], "load_ms": 0 }`.
- **A slow load returns before it finishes.** `wait_s` bounds only how
  long the *call blocks* (default `30.0` supervised, `3.0` bare, clamped
  to `[0.1, 300.0]`); the supervisor bind continues on a background
  thread. A synchronous `{"ok": true, "load_state": "in_progress"}` is
  therefore possible, with the real diagnostics arriving later via
  `GET /world/diagnostics`. A recorded broken-world resolution took
  **242.9 s** end to end.

### 7.3a POST /world/sync

Default authored-world iteration primitive. `path` may be omitted to select the
currently loaded source.

```json
{ "path": "worlds/scene.wbt", "settle_steps": 1,
  "reset_physics": true, "wait_s": 30.0, "light": true }
```

The service compares the file with the exact source snapshot used for the
running world and returns one of:

- `mode: "live_pose"`: only numeric `translation`/`rotation` values on
  existing root-level DEF nodes changed. The entire Solid/Robot batch is
  validated before mutation, applied with one settling window, and returned
  with physical position readbacks.
- `mode: "no_change"`: no semantic edit (comments, whitespace, or equivalent
  numeric spelling only).
- `mode: "full_reload"`: every other edit automatically executes the ordinary
  `/world/load` path and carries its structured diagnostics.

The classifier is conservative. Nested-node, geometry, collision, physics,
mass, material, controller, add/remove, malformed, light-mode, different-world,
and ambiguous changes MUST fall back. Clients SHOULD NOT duplicate this
classification. A transport failure during a live batch also falls back to a
reload so disk and runtime converge.

### 7.4 GET /world/diagnostics

Re-fetch the structured diagnostics from the most recent load without
re-parsing.

### 7.5 POST /world/screenshot

**Request:** `{ "path": "shot.png", "quality": 90 }` — `path` is
optional. When absent the PNG is streamed back as the response body
(`Content-Type: image/png`). When supplied, the PNG is written to that
path (server-relative) and `{ "ok": true, "path": "..." }` is returned.

### 7.6 GET /world/render_stats

**Response:**

```json
{ "width": 1024, "height": 768, "pixels": 786432,
  "mean_brightness": 107.35, "mean_rgb": [101.2, 107.4, 113.5],
  "max_rgb": [255, 255, 255],
  "saturated_pct": 0.8, "black_pct": 12.3,
  "warnings": [] }
```

**⚠️ Brightness is on the 0–255 scale, not 0–1.** `mean_brightness`,
`mean_rgb` and `max_rgb` come straight from Pillow's 8-bit channel
statistics (`compute_render_stats`, `omnisim_harness.py`). An earlier
version of this section showed normalised `0.42` / `[1.0, 1.0, 1.0]`
values that the harness has never returned — a client thresholding at
`mean_brightness < 0.1` for "the scene is black" matches nothing.

There is no `ok` field. `width` / `height` / `pixels` are always present
(and are the only fields returned for a zero-pixel image).

`warnings` fires on exactly two conditions, both deliberately
conservative — an ordinary well-exposed scene returns `[]`:

| Condition | Message |
|---|---|
| `saturated_pct > 30` | `"blown out: N% of pixels are saturated; reduce DirectionalLight/PointLight intensities"` |
| `black_pct > 60` | `"underexposed: N% of pixels are near-black; increase light intensities or check camera framing"` |

"Saturated" means per-pixel `max(r,g,b) >= 250`; "near-black" means
`max(r,g,b) <= 5`.

`503` if Pillow is not installed in the interpreter running the harness.

### 7.7 GET /scene/tree

Flat node list. **Response:** `{ "nodes": [ ... ], "bounds_included": bool }`
— note the `nodes` wrapper; the array is not the top-level body. Each
entry has `def`, `type`, `position`, `orientation`, `parent_def`,
`is_robot`.

**Query parameters:**

- `bounds=1` — attach world-space geometric bounds to every node:
  `{center, radius, bbox_min, bbox_max, size, exact, sources, skipped}`.
  Opt-in because it walks every geometry node and reads mesh files off
  disk. `exact: false` plus a `skipped` list naming the meshes the walk
  could not parse is the honest-uncertainty signal — do not treat a
  bounds value as exact without checking it.

Cost scales with node count: measured **1.02 s** on a 17-node world and
**23.0 s** on a 298-node one. (The same runs measured 0.14 s / 4.40 s on
the ODE backend — historical, since ODE was deleted on 2026-08-08; the
Newton figures are the only ones you can reproduce.) See the `/sim/step`
warning in §7.10.

### 7.8 GET /scene/node/<def>

Field dump + contact points for one node, identified by DEF name.
`CommandError` → `503` if no node carries that DEF.

**Query parameters:**

- `bounds=1` — the same bounds block as §7.7, for this node's whole subtree.
- `probe=1` — attach `bounds_probe`, an exactness oracle that recovers the
  engine's own bounding sphere by inverting
  `OmViewpoint::moveViewpointToObject`. **Slow (seconds) and it steps the
  simulation**, hence opt-in. On failure the key is present with an
  `{"error": ...}` value rather than absent.

### 7.9 POST /scene/look_at

**Request:** `{ "position": [x,y,z], "target": [x,y,z], "push": true }`.

**Response:** computed axis-angle orientation;`push: true` (default)
applies it to the live `Viewpoint` immediately.

### 7.10 POST /sim/step

`{ "steps": int }` — advance N basic timesteps (default `1`; `< 1` is a
`400`).

**Response:** `{ "sim_time_ms": float, "advanced_to_ms": float }`. There
is no `ok` field and no `sim_time` field — the unit is **milliseconds**
and the key is `sim_time_ms`. Both values are equal on success;
`advanced_to_ms` is what the supervisor's main loop uses to resynchronise
its own clock.

**⚠️ A single step can take tens of seconds, and an over-long step kills
the session.** Budget before you call; this is the surface's sharpest
edge.

Measured on an RTX 3060 laptop (machine `9722d23d12a3`, build `806b055c`):

| World | Backend | `steps: 1` | marginal s/step |
|---|---|---|---|
| 17 nodes | Newton (the only backend) | 0.86–1.22 s | 0.47 s |
| **298 nodes** | **Newton** | **26.6–27.1 s** | **~14 s** |
| 17 nodes | ~~ODE~~ (historical) | 0.018 s | 0.008 s |
| 298 nodes | ~~ODE~~ (historical) | 5.8–6.2 s | 2.28 s |

⚠️ The ODE rows were measured while ODE still shipped; it was deleted on
2026-08-08 (commit `bdc02139`). They are kept because they are the reason
the two mitigations below exist — **not** because a cheaper backend is
available to switch to. It is not.

Two causes compound. The injected supervisor polls damage, contacts and
grips **once per inner 16 ms step**, and the grip poll calls
`observe.build_robot_subtree_index()`, which walks the entire scene graph
over supervisor IPC — so the observability cost is O(nodes × steps). On
top of that, a Newton supervisor round-trip measured ~60× an ODE one, so
the per-round-trip cost is now simply the cost.

**Both mitigations are now reachable over the wire.** The supervisor's
`--light` flag (it skips all three producers) is exposed as
`POST /world/load {"light": true}` — measured 27.0 s → 0.034 s per step on
that 298-node world — and `GET /capabilities` (§7.28) publishes the
**measured** `limits.step_cost` for the loaded world plus a derived
`recommended_max_steps_per_request`, so the budget below does not have to
be guessed.

`SUPERVISOR_RPC_TIMEOUT_S = 120.0` is still a module constant with no
per-request override, and `step` is deliberately **not** in
`IDEMPOTENT_SUPERVISOR_COMMANDS`, so a step that exceeds it drops the
socket and is not retried. Recovery from that state requires a full
`/world/load`. On the 298-node world, `steps: 20` under Newton and
`steps: 60` under either backend both hit it. Practical rule: read
`limits.recommended_max_steps_per_request` from `/capabilities`
(`floor(0.6 × timeout / measured cost)`), or measure `steps: 1` yourself
first on an unfamiliar world.

The response also carries `wall_ms` and `steps_executed`, and each call
feeds the rolling `limits.step_cost` median.

Proposed fix: [docs/developer/agent-native-api.md](docs/developer/agent-native-api.md)
G1 / P2 (`observe`, `observe_every`, per-request `timeout_s`, `return: ["poses"]`).

### 7.11 POST /sim/reset

Rewind the simulation clock **and** restore the world to its authored
state.

**Request (all optional):**

```json
{ "restore": "__init__", "verify": true, "settle_steps": 1 }
```

**Response:**

```json
{ "sim_time_ms": 0.0, "advanced_to_ms": 0.0,
  "restored": "__init__", "settle_steps": 1,
  "verification": {
    "moved_by_reset": { "sampled_nodes": 5, "max_pose_delta_m": 6.434663,
                        "max_pose_delta_node": "BALL", "exact": false },
    "poses_after": { "BALL": [1.0, 0.0, 0.736], "TARGET": [0, 0, 0.3] },
    "vs_snapshot": null,
    "vs_snapshot_note": "'__init__' is the engine's own parse-time state, ..." } }
```

**This changed on 2026-07-26.** `simulationReset()` alone rewinds the
clock and leaves the scene where it fell — verified on both backends, and
that was the documented behaviour here. `/sim/reset` now *also* loads a
named engine state, `"__init__"` by default:

- `"__init__"` is populated by the engine itself, for free, at parse time:
  `OmNode`'s constructor sets `mCurrentStateId = "__init__"`
  (`src/omnisim/vrml/OmNode.cpp:161`) and `OmPose`'s constructor saves the
  node's authored translation/rotation under it. Restoring it is therefore
  "the world as the `.wbt` wrote it".
- Pass `"restore": null` for the old clock-only behaviour, or any name
  from `POST /sim/snapshot` (§7.30) to reset the clock to a snapshot
  instead.

Measured (lane3, `BALL` authored at `z = 1.0`): moved to `(-4, -4, 0.1)`
with `/scene/set_pose`, then `/sim/reset` → `BALL` reads
`(1.0, 0.0, 0.736)` — back at its authored x/y and mid-fall from its
authored height. (Measured identically on ODE at the time, before it was
deleted; the behaviour is a scene-graph property, not a solver one.)

**A supervisor snapshot is NOT a substitute for `"__init__"`.** The engine
free-runs (`--mode=fast`, `synchronization FALSE`), so by the time the
injected supervisor's first step executes, a dropped body has already
fallen: on lane3 the supervisor's first read of `BALL` is `z = 0.1`, never
`z = 1.0`. Only the engine's parse-time state is the authored one.

**`verification` is a sample, not a proof.** `poses_after` and the pose
deltas cover the scene's **top-level** posed nodes (root children), which
is what a per-node IPC read can afford; the restore itself is recursive
over the whole scene (`OmGroup::save` / `OmSolid::reset`). And because the
engine keeps stepping between RPCs, a body that is still falling reports a
non-zero delta legitimately.

### 7.12 GET /sim/state

**Response — the complete body, all 13 fields:**

```json
{ "world": "O:\\omnisim\\...\\lane3_drive.wbt",
  "running": true,
  "exit_code": null,
  "load_ok": true,
  "load_ms": 6941,
  "load_state": "complete",
  "load_started_at": 1769472000.1,
  "load_completed_at": 1769472007.0,
  "supervisor_connected": true,
  "supervisor_connected_at": 1769472007.0,
  "supervisor_bind": { "status": "bound", "world": "...", "elapsed_s": 6.8, "detail": "" },
  "binary": "O:\\omnisim\\msys64\\mingw64\\bin\\omnisim-bin.exe",
  "webots_home": "O:\\omnisim" }
```

**⚠️ There is no `paused` field, no `sim_time` field, and no `last_load`
object.** Earlier text for this section listed all three; none has ever
been returned. `running` means only "the engine subprocess is alive" — it
is **not** a `STOPPED / PLAYING / PAUSED` run state, and the harness
exposes no way to pause or resume (§16). For the simulation clock, use
the `sim_time_ms` returned by `POST /sim/step` (§7.10).

`load_state` ∈ `idle` | `in_progress` | `complete` | `failed`.

**`load_ok` / `load_state` describe the last load, not current health.**
They are latched at load time and are not re-evaluated, so a session
whose supervisor has since dropped (e.g. after a `/sim/step` timeout,
§7.10) still reports `load_ok: true, load_state: "complete",
running: true` while every scene endpoint returns `503`.
`supervisor_connected` is the only field recomputed on each call — check
that one for liveness.

`/sim/state` reports metadata about the harness session — **not scene
state**. For scene state, use `/robots`, `/robot/<def>/joints`,
`/sim/contacts`. It is also the one endpoint that never touches the
simulator subprocess, so it stays fast (~1.6 ms) and answers even while
the supervisor is disconnected.

### 7.13 GET /robots

Enumerate every `Robot` in the current scene.

**Response:** `{ "robots": [ ... ] }` — note the wrapper; no `ok` field.
Each entry carries identity (`def`, `name`, `model`, `controller`), pose
(`position`, `orientation`) and a joint count.

### 7.14 GET /robot/<def>/joints

Per-joint snapshot: `name`, `type`, `position`, `velocity`, `lower`,
`upper`, `hit_limit`.

### 7.15 GET /robot/<def>/devices

List of devices visible in the robot's subtree (cameras, lidars, motors, …).

### 7.16 GET /robot/<def>/sensor/<name>

MUST return `501 Not Implemented` with `error = "effector_unavailable"`.
The supervisor that backs the harness cannot honestly read sensors it
does not own; clients needing sensor data MUST go through the robot's
own controller or via the Robot Bridge (§5).

**⚠️ Implemented differently:** the harness does return `501`, but the
body is prose plus context, not the fault code:

```json
501
{ "error": "live sensor reads not supported from the supervisor (OmniSim restricts device APIs to the controller that owns the device). Use /robot/<def>/joints for joint positions, or run a per-robot helper controller that exports its sensor data over its own endpoint.",
  "robot": "HUSKY", "sensor": "lidar" }
```

Branch on the status code, not on `error`. Tracked in §16. The `501`
itself is correct and deliberate — it is not a gap to be filled by
proxying reads through the supervisor.

### 7.17 GET /sim/contacts

`{ "contacts": [ { "a_def": "...", "b_def": "...", "point": [x,y,z] } ] }`
— no `ok` field.

### 7.18 GET /sim/grips

`{ "grips": [ { "gripper_def": "...", "held_def": "...", "since_t_ms": int } ] }`
— no `ok` field. Returns `{"grips": []}` when the supervisor is running
`--light` (grip tracking disabled) as well as when nothing is held; the
two are indistinguishable over the wire.

Grips are **inferred** from stable contact membership, not reported by
the engine: a candidate pair must persist for `STABLE_STEPS = 3`
consecutive polls before it is reported, which filters transient
multi-finger touches during approach but also means a grip appears ~3
steps after it physically forms.

### 7.19 GET /sim/events

The unified runtime event stream — supervisor-side and harness-side
events merged into a single cursor-paged response.

**Query parameters:**

- `since` — supervisor-side cursor (default `0`).
- `log_since` — harness-side log cursor (default `0`).
- `limit` — max events to return. **Default `256`**, silently clamped to
  `[1, 1024]`. (Applied to each side independently, so a single response
  can carry up to `2 × limit` events.)
- `types` — comma-separated allowlist; if absent, all event types are
  returned.

> ### ⚠️ `types` is an exact-match allowlist with NO validation
>
> Both filters are a literal `evt["type"] not in set(types)` test — the
> supervisor's `EventBus.since()` and the harness's `LogRingBuffer.since()`.
> There is no prefix matching, no globbing, and **no error for an
> unrecognised name**. `?types=grip.began` or `?types=damage.*` returns
> `{"events": [], ...}` with HTTP `200`, which is indistinguishable from
> "nothing happened".
>
> Copy the names from §10.1 exactly. If a filtered poll returns nothing,
> re-poll **without** `types` before concluding the scene is quiet.

**Response:**

```json
{
  "events": [
    { "seq": 41, "type": "contact.began", "t_sim_ms": 1408,
      "a_def": "HUSKY", "b_def": "WALL_03", "point": [1.0, 2.0, 0.05],
      "source": "sup" },
    { "seq": 17, "type": "controller.log", "t_wall": 1769472011.4,
      "stream": "stdout", "line": "husky_random: turning", "source": "log" }
  ],
  "next_since": 41,
  "next_log_since": 17,
  "dropped_sup": 0,
  "dropped_log": 0
}
```

**⚠️ There is no `ok` field and no `t` field.** §10's requirement that
every event carry `t` (float seconds) is **not met by this stream**.
Timestamps are:

| Origin | `source` | Timestamp field | Unit |
|---|---|---|---|
| Supervisor producers | `"sup"` | `t_sim_ms` | **int milliseconds of sim time** |
| Harness log tail | `"log"` | `t_wall` | float Unix epoch **wall** seconds |

The two are not comparable, and neither is `t`. `source` is stamped by
the harness as it merges the streams and is the only reliable way to tell
which clock an event is on.

`seq` is monotonic **per side and per supervisor process** — the two
cursors are independent counters, so a `sup` event and a `log` event can
share a `seq`. Advance `since` from `next_since` and `log_since` from
`next_log_since` separately; never cross them.

Supervisor-side events are best-effort: if the supervisor is not
connected the harness still returns the harness-side log events (with
`next_since` echoed back unchanged) rather than failing.

Event taxonomy: see §10. `dropped_sup`/`dropped_log` going non-zero
means the caller is polling slower than events arrive; raise `limit`
or poll more often. Both ring buffers hold 4096 events.

### 7.20 GET /scene/viewpoint

Read the live camera back. Every other camera verb writes; this is the
only one that reads, and framing math is unreliable without it because
`Viewpoint.fieldOfView` alone does not determine the on-screen angles.

**Response:** `position`, `orientation` (axis-angle), `field_of_view`,
`near`, `far`, `follow`, `follow_type`, `follow_smoothness`,
`projection_mode`, `exposure`; derived unit vectors `forward`, `left`,
`right`, `up`; `aspect` and `viewport {width, height, source}`; the
resolved `fov_h_deg`, `fov_v_deg`, `half_fov_h_deg`, `half_fov_v_deg`;
a `fov_semantics` string; and `raw`, the unprocessed `Viewpoint` field
dump.

`fieldOfView` is the VRML angle on the **larger** viewport dimension —
which is why the resolved per-axis angles are returned separately.
`viewport.source` names how the real 3D-view size was learned (the
harness infers it from the header of the last PNG it rendered), so a
`viewport` of `{null, null}` means no screenshot has been taken yet and
`aspect` is the default rather than measured.

`503` if the supervisor is not connected.

### 7.21 POST /scene/frame

Compute **both** aim and distance so a subject fills the frame, push the
result to the live `Viewpoint`, and return a numeric proof that it is in
frame. This is the verb to reach for before any screenshot — it replaces
the guess-a-pose → render → look-at-pixels → guess-again loop with one
call and a number.

**Request** — the subject is given in one of three ways:

```json
{ "def": "HUSKY" }
{ "defs": ["HUSKY", "CRATE_01"] }
{ "target": [x, y, z], "radius": 2.5 }
```

plus the optional `margin`, `aspect`, `fov`, `radius_override`,
`subject_relative` (default `true`), `push` (default `true`), and `mode`.

`mode` is one of **eight** values — `hero` (default), `front`, `back`,
`left`, `right`, `top`, `top_down`, `bottom` — plus the aliases
`topdown` and `overview` → `top_down`, `3/4` and `default` → `hero`,
`side` → `left`. Matching is case-insensitive and whitespace-trimmed.
With a single `def` and `subject_relative: true`, the directional modes
are rotated into the **subject's** frame (+X forward, +Y left, +Z up), so
`front` means the robot's front, not world +X; the response's
`relative_to` says which frame was used. With `defs` (several subjects)
they fall back to world axes.

**Response:** the computed `position` / `orientation` / `target`;
`pushed`; `previous_position` / `previous_orientation` (so the caller can
restore the camera without a second read); the resolved `subject`;
`framing` metadata; `relative_to` (`"subject"` or `"world"`); `camera`;
and:

```json
"verification": {
  "fits": true,
  "headroom_h_deg": 6.4, "headroom_v_deg": 2.1,
  "subject_angular_radius_deg": 15.8,
  "subject_screen_bbox": { "pixels": [ ... ], "ndc": [ ... ] }
}
```

`verification.fits` is the field to branch on. An unknown `mode` is a
`400` with `code: "BAD_VIEW_MODE"` — the one place on this surface where
an error carries a machine-readable code.

Framing is only as exact as the bounds it is derived from: check
`subject.exact` (§7.7) before trusting a tight margin.

### 7.22 POST /scene/orbit

The only **relative** camera verb — every other one, here and in the
`simulation_interfaces` standard, is absolute, so "a bit more to the
left" otherwise means re-deriving a whole pose. (OmniSim implements that
standard itself, as a ROS 2 sidecar over this protocol — see
[`packages/omnisim-ros2/`](packages/omnisim-ros2/).)

**Request:** `azimuth_deg`, `elevation_deg`, `dolly` (a multiplier on the
current distance, default `1.0`), `pan` (`[dx, dy]` in screen-space
metres), `push` (default `true`), and an orbit centre resolved in this
priority order:

1. `center: [x, y, z]` — explicit.
2. `def: "HUSKY"` — the node's bounds centre.
3. neither — a point `distance` metres (default `10.0`) straight ahead
   along the current view axis.

**Response:** `position`, `orientation`, `previous_position`,
`previous_orientation`, `pushed`, `camera`, and an `orbit` block whose
`center_source` says which of the three rules above was used — check it
when a nudge moves the camera somewhere unexpected.

### 7.23 GET /scene/visible

What is actually on screen right now, and by how much you are off. The
closed-loop feedback signal for aiming: it turns "the screenshot looks
wrong" into a number.

**Query parameters:** `defs` (comma-separated; omitted = the whole
scene), `all=1` (include DEF-less nodes, which are otherwise filtered as
noise), `limit` (default `200`, clamped to `[1, 2000]`).

**Response:** a `camera` block, a `counts` block
(`considered` / `on_screen` / `returned`), `pixel_basis`, and `nodes`
sorted on-screen-first then by distance. Per node: `def`, `type`,
`center`, `radius`, `distance`, `in_frame`, `on_screen`,
`behind_camera`, `centroid_ndc`, `centroid_pixel`, `screen_bbox_ndc`,
`screen_bbox_pixels`, `yaw_deg`, `pitch_deg`, `angle_off_axis_deg`,
`bounds_exact`, and a natural-language `hint` such as
`"off-screen: 34 deg to the left, 12 deg up"`.

`in_frame` tests the **centroid** against the frustum; `on_screen` is the
looser test that also counts a node whose screen bounding box overlaps
the viewport. A large partially-visible object is `on_screen: true,
in_frame: false` — use `on_screen` for "can the viewer see it".

### 7.24 GET /robot/damage

Structural damage state for the tracked robot: per-part HP and state.
No standard has an equivalent. Damage is an OmniSim addition, not
inherited from upstream.

### 7.25 GET /robot/damage/events

Damage-event log with its own cursor, independent of `/sim/events`.

**Query parameters:** `since` (int), `limit` (int); a non-integer value
for either is a `400`.

The same events are also fanned out into `/sim/events` as
`damage.impact` / `damage.state_transition` (§10.5), so most agents
should poll the unified stream instead and use this endpoint only when
they want damage alone with a separate cursor. Payloads here carry a
`step_id` that the `/sim/events` copies do not.

### 7.26 POST /robot/damage/reset

Restore all tracked parts to pristine. Takes no body (any body sent is
drained and ignored — a deliberate accommodation for Windows clients,
which otherwise see a `ConnectionReset` when their request body is still
in flight as the response arrives).

### 7.27 POST /robot/damage/inject

Directly set or perturb a part's damage state, for testing a reaction
without staging a real collision.

**Request:** `{ "part": "<name>", "hp_delta": float, "state": "<state>" }`
— `part` is required (`400` if absent or not a string); `hp_delta` and
`state` are optional, and a non-numeric `hp_delta` is a `400`.

A world can also run a schedule without any external client, via the
supervisor's own `customData`:
`{"damage_inject_schedule": [{"t_ms": 20000, "part": "...", "state": null, "hp_delta": -50.0}]}`.

### 7.28 GET /capabilities

Capability discovery. The one call that answers "what is this harness,
what is actually driving the physics, what will a step cost me, and what
does it refuse to do".

**Query parameters:**

- `probe_step=1` — if no `/sim/step` has been measured on this world yet,
  advance **one** basic step to measure it. It mutates the simulation by
  one step, hence opt-in.

**Response (abridged; every field below is real):**

```json
{ "ok": true, "omnisim_wire": "1.1", "service": "world_harness",
  "sim_version": "5.3.0",
  "build": { "commit": "6d27913f", "binary": "...omnisim-bin.exe",
             "harness_python": "3.12.9", "pillow": true },
  "machine": { "host": "...", "platform": "win32", "note": "canonical machine id ..." },

  "physics": { "backend": "newton", "solver": "MuJoCo (cpu/mj_step, WorldInfo.newtonSolver)",
               "degraded": false, "finalised": true, "source": "sidecar",
               "sidecar_path": "...omnisim_log.txt.newton.json",
               "sidecar_age_s": 0.6, "basic_time_step_ms": 32 },

  "supervisor": { "connected": true, "port": 6990, "light": true,
                  "commands": ["capabilities", "scene_spawn", "..."],
                  "commands_source": "scanned from dispatch() in harness_supervisor.py",
                  "snapshots": ["__init__", "t0"] },

  "world": { "path": "...", "load_ok": true, "load_ms": 7662, "load_state": "complete" },

  "features": ["world.load", "scene.spawn", "sim.snapshot", "..."],
  "not_supported": [ { "feature": "robot.sensor_read", "code": "effector_unavailable",
                       "http": 501, "reason": "...", "workaround": "..." } ],

  "limits": { "supervisor_rpc_timeout_s": 120.0,
              "max_steps_per_request": null,
              "step_cost": { "median_s_per_step": 1.150688, "samples": 1,
                             "source": "rolling median of the last /sim/step calls on this world" },
              "recommended_max_steps_per_request": 62,
              "recommended_max_steps_per_request_formula":
                "floor(0.6 * supervisor_rpc_timeout_s / step_cost.median_s_per_step)",
              "events_limit_default": 256, "events_limit_max": 1024 },

  "endpoints": [ { "method": "GET", "path": "/scene/tree", "summary": "...", "params": ["bounds"] } ],
  "endpoints_verification": { "declared": 33, "scanned_literals": 32,
                              "declared_not_found_in_source": [], "undeclared_literals": [],
                              "verified": true },
  "event_types": ["contact.began", "...", "world.error"],
  "event_types_detail": { "supervisor": { "verified": true, "undeclared": [],
                                          "declared_not_emitted": [],
                                          "active": [...], "suppressed": [...] },
                          "harness": { "verified": true } },
  "diagnostic_codes": ["ASSET_DOWNLOAD_FAILED", "..."],
  "request_error_codes": ["DEF_NOT_FOUND", "SPAWN_REJECTED", "..."],
  "step_probe": { "steps": 1, "wall_s": 1.1507 } }
```

Four things about it are load-bearing:

1. **`physics` is the engine's own verdict, not a guess.** It reads the
   `<engine-log>.newton.json` sidecar `OmNewtonBackend::finalizeWorld`
   writes (`source: "sidecar"`). `OmLog` deletes a stale copy when it
   truncates the log at startup, so the file's presence means Newton drove
   *this* run. Fallbacks are labelled honestly:
   `source: "engine_log"` (the finalise line), `"forced_by_env"`
   (`OMNISIM_FORCE_ODE` / `OMNISIM_LEGACY`, also echoed as
   `forced_ode_env`), `"sidecar_stale"` (the sidecar predates the current
   load — the backend is *unverified* for this world), or
   `"sidecar_absent"` with `backend: "ode"|"unknown"` and a `detail`
   saying a run that never reached finalize proves nothing.
   ⚠️ **Those `"ode"` labels and the `forced_by_env` source predate the
   ODE deletion (2026-08-08, commit `bdc02139`) and no longer name a
   working backend.** Read `backend: "ode"` as *"Newton did not finalize
   this world"* — there is nothing for it to have fallen back to — and
   treat `OMNISIM_FORCE_ODE` / `OMNISIM_LEGACY` as retired variables that
   a client should never set. A client must not branch on `"ode"` as a
   capability.
   Historical contrast measured on the same 10-robot scene while ODE
   shipped: **1.15 s** per `/sim/step` under Newton non-light vs
   **0.0025 s** under ODE light. The `light` half of that is still
   available and still the lever; the backend half is not.
2. **`limits.step_cost` is measured on the current world** and cleared by
   every load. `recommended_max_steps_per_request` is derived from it, so
   an agent can size a budget instead of discovering
   `supervisor_rpc_timeout_s` by killing its session (§7.10). It is
   telemetry: nothing server-side branches on it.
3. **`event_types` and the endpoint list are served from the code.** The
   supervisor scans its own `emit()` sites; the harness scans its own
   handler for path literals. Both report `verified` plus the mismatch
   lists, which is why §7 and §10.1 now tell you to ask this endpoint
   instead of trusting a table.
4. **`not_supported` is part of the contract.** Each entry carries a
   `reason` and a `workaround`, including the entries that only apply to
   *this* session (a `--light` supervisor lists its suppressed
   contact/grip/joint surface).

`GET /capabilities` answers with `supervisor.connected: false` and
`event_types` limited to the three log types when no world is loaded — it
never requires a supervisor.

### 7.29 POST /scene/spawn

⛔ **A SPAWNED NODE HAS NO PHYSICS UNTIL THE WORLD IS RELOADED — MEASURED 2026-08-17.**
This verb adds a node to the **scene graph**, not to the solver. A spawned **dynamic**
body never falls and a spawned **static** body never collides: the MuJoCo model is frozen
at `finalizeWorld()` (`openForBuild=false`, `OmNewtonBackend.cpp:2384`) and every
`addBody`/`addShape*` verb guards on it, so a mid-run spawn registers **zero** bodies.
Measured against an in-session control on the CPU `mj_step` path: an *authored* 0.2 m box
released at z = 1.5 settled at **z = 0.599892** on a floor topped at 0.50, while a spawned
twin read **z = 1.5 unchanged after 2200 steps and ~87 s of simulated time**, and a spawned
static platform topped at z = 1.00 was fallen straight through. **The failure is silent**:
0 errors, 0 warnings, the response below still returns `verification.node_resolved: true`,
and the node appears in `/scene/tree` and renders. This is the exact mirror of the runtime
**delete** defect (a deleted collider stays in the model). Use `spawn` for cameras, markers
and visual props, and for staging a scene you then reload; do not use it for anything that
must fall, collide or be picked up. `set_pose` is unaffected. Tracked internally as W1.7 — runtime scene
mutation (one workstream covering both directions); no public issue yet.
As the honest interim, every successful spawn response carries a `physics_warning`
block — `{"code": "RUNTIME_MUTATION_NOT_IN_SOLVER", "message": ...}` — on all input
forms, and the first spawn per world-load also emits one `world.warning` with the same
code into `/sim/events` (§7.19).

Add a node to the **live** scene. Four input shapes:

```json
{ "urdf": "projects/robots/clearpath/husky_description/urdf/husky.urdf",
  "def": "HUSKY_1", "translation": [12, 0, 0.2], "rotation": [0, 0, 1, 1.57],
  "fields": { "name": "husky_1", "controller": "husky_random" } }

{ "type": "Solid", "def": "BOX", "fields": { "name": "box" },
  "translation": [2, 1, 0.6] }

{ "vrml": "Solid { name \"probe\" children [ Shape { geometry Box { size 0.4 0.4 0.4 } } ] }",
  "def": "PROBE_BOX", "translation": [2, 1, 0.6], "rotation": [0, 0, 1, 0.3] }

{ "clone": "HUSKY_0", "def": "HUSKY_1", "name": "husky_1",
  "translation": [9.7082, 7.0534, 0.2], "rotation": [0, 0, 1, 2.19911] }
```

Also accepts `parent` (a DEF whose `children` field receives the node;
default the scene root), `index`, `settle_steps` and `reset_physics`.

**Response:**

```json
{ "def": "HUSKY_1", "id": 203, "type": "Robot",
  "position": [9.7082, 7.0534, 0.2], "orientation": [ ...9 floats... ],
  "index": 8, "parent": "root", "cloned_from": "HUSKY_0",
  "overrides_in_vrml": ["name", "translation", "rotation"],
  "overrides_by_field_write": {},
  "children_before": 8, "children_after": 9,
  "settle_steps": 1, "sim_time_ms": 576.0, "advanced_to_ms": 576.0,
  "verification": { "node_resolved": true, "def_resolves": true,
                    "children_delta": 1, "pose_delta_m": 0.0 },
  "physics_warning": { "code": "RUNTIME_MUTATION_NOT_IN_SOLVER",
                       "message": "the Newton/MuJoCo model is frozen at world finalize, ..." },
  "vrml": "DEF HUSKY_1 Robot { ... }" }
```

`vrml` is echoed for the composed forms so a rejection is debuggable
without reconstructing what was sent. Backed by
`Field.importMFNodeFromString`. Measured: 0.27–0.44 s per spawn (Newton,
light). The 0.03–0.32 s figure recorded alongside it was the ODE path and
is historical — Newton's is the only cost you can hit now.

**⚠️ `URDFRobot` and undeclared PROTOs cannot be imported — use `clone`.**
This is an engine constraint, not a harness gap. `URDFRobot { url ... }`
is a *source* expansion performed by `OmTokenizer::tokenizeFile`
(`src/omnisim/vrml/OmTokenizer.cpp:412`); a supervisor import goes through
`tokenizeString`, which never expands it, so
`OmParser::protoNodeList()` classifies `URDFRobot` as a PROTO and
`OmNodeOperations::importNode` refuses anything not in the world's
`IMPORTABLE EXTERNPROTO` list. The response is a `422`:

```json
{ "ok": false, "code": "SPAWN_REJECTED",
  "error": "import added no node: the engine rejected the VRML (children 8 -> 8). ...",
  "vrml": "DEF CRATE CardboardBox {\n  translation 2 2 0.3\n}",
  "engine_diagnostics": [ { "code": "UNKNOWN",
    "message": "In order to import the PROTO 'CardboardBox', first it must be declared in the IMPORTABLE EXTERNPROTO list." } ] }
```

`clone` sidesteps it: the supervisor asks the engine for the source node's
VRML via `Node.exportString()`, which returns the **already expanded**
`Robot`, and re-imports that. There is no second URDF importer, so nothing
can drift from the engine's.

**⚠️ For a cloned robot, `name` must be rewritten before the import, and
it is.** The engine starts the imported robot's controller immediately,
and the controller's IPC channel is keyed by the robot's **name** — so
clones that arrive carrying the source's name collide
(`refusing connection attempt from another extern controller`, the second
controller exits `1`, that robot never moves, nothing fails loudly).
Measured before the fix: 8 of 9 clones silently dead. The supervisor
therefore rewrites `name` / `translation` / `rotation` in the node text
(depth-aware, so nested `name` fields in the robot's subtree are
untouched) and reports which fields took that route in
`overrides_in_vrml` versus `overrides_by_field_write`.

Other error shapes: `400` + `SPAWN_SPEC_INVALID` (the body is not a usable
spec), `404` + `CLONE_DEF_NOT_FOUND` / `PARENT_DEF_NOT_FOUND`.

### 7.30 POST /scene/delete

⛔ **A DELETED NODE LEAVES ITS COLLIDERS IN THE SOLVER AS PHANTOMS UNTIL THE WORLD IS
RELOADED — MEASURED 2026-08-17.** The exact mirror of the spawn defect above (§7.29): the
frozen MuJoCo model has no remove path either, so a deleted wall still blocks robots and
rays, and a deleted floor still holds bodies up, silently (a 0.2 m box rested at z = 0.5999
for 61,440 steps on a floor `POST /scene/delete` had removed). The node is gone from the
scene graph and the render, which makes the phantom invisible. Tracked internally as
W1.7 — runtime scene mutation, one workstream covering both directions. As the honest interim,
every successful delete response carries the same `physics_warning` block
(`RUNTIME_MUTATION_NOT_IN_SOLVER`), and the first delete per world-load emits one
`world.warning` into `/sim/events` (§7.19). Reload the world after removing collidable nodes.

**Request:** `{ "def": "PROBE_BOX" }` or `{ "defs": ["A", "B"], "settle_steps": 0 }`.

**Response:**

```json
{ "removed": [ { "def": "MARKER", "id": 206, "type": "Solid" } ],
  "missing": ["NO_SUCH_DEF"],
  "settle_steps": 0, "sim_time_ms": 4096.0,
  "verification": { "all_removed": true, "still_resolves": [] },
  "physics_warning": { "code": "RUNTIME_MUTATION_NOT_IN_SOLVER",
                       "message": "the Newton/MuJoCo model is frozen at world finalize, ..." } }
```

Per-DEF results, no atomicity: a missing DEF is reported in `missing`, not
raised. `verification.still_resolves` re-queries each removed DEF, so the
caller does not have to trust `Node.remove()`.

### 7.31 POST /scene/set_pose

**Request:**

```json
{ "def": "HUSKY_0", "translation": [12, 0, 0.2], "rotation": [0, 0, 1, 1.5708],
  "reset_physics": true, "settle_steps": 1 }
```

**Response:**

```json
{ "def": "HUSKY_0", "type": "Robot",
  "requested": { "translation": [12, 0, 0.2], "rotation": [0, 0, 1, 1.5708] },
  "position_before": [11.6714, 1.0084, 0.1319],
  "position": [12.0, 0.0000062, 0.1941],
  "sim_time_ms": 1472.0,
  "verification": { "settled_steps": 1, "reset_physics": true,
                    "pose_delta_m": 0.00586,
                    "frame": "world position vs requested local translation; these differ when the node is not a root child" } }
```

- **`settle_steps` defaults to `1` on purpose.** A supervisor field write
  is applied by the engine on its next step, so a read-back with
  `settle_steps: 0` legitimately still shows the old pose.
- **`reset_physics` defaults to `true`** (`Node.resetPhysics()`), because a
  teleported body otherwise keeps its velocity and immediately drifts,
  which reads as "the pose did not stick".
- `pose_delta_m` compares the **world** position read back against the
  **local** translation requested; those differ by the parent transform
  for a non-root child, which is what `frame` says.
- **No interpenetration check.** Placing a dynamic body inside static
  geometry is accepted and then resolved by the physics: measured on
  lane3, `BALL` (rest height `z ≈ 0.149`, measured on ODE at the time)
  placed at `z = 0.1` tunnelled through the floor and read `z = -2251` a
  moment later.
  `GET /scene/node/<def>?bounds=1` (§7.8) before placing is the check.

Errors: `404` + `DEF_NOT_FOUND`, `422` + `FIELD_NOT_ON_NODE` (e.g. a light
or a `Viewpoint` has no `translation`), `400` + `POSE_UNSPECIFIED`.

### 7.32 POST /sim/snapshot, POST /sim/restore, GET /sim/snapshots

Named engine-side state, built on `Node.saveState()` / `Node.loadState()`
walked from the scene root — so the save/restore is recursive over the
whole scene (`OmGroup::save`), not just the sampled nodes.

```json
POST /sim/snapshot  { "name": "t0" }
-> { "name": "t0", "sim_time_ms": 224.0, "sampled_nodes": 6,
     "names": ["__init__", "t0"],
     "scope": "world (OmNode::save recurses the whole scene)",
     "sample_scope": "root children (used only to verify a later restore)" }

POST /sim/restore   { "name": "t0", "settle_steps": 1 }
-> { "name": "t0", "sim_time_ms": 288.0, "snapshot_sim_time_ms": 80.0,
     "engine_provided": false, "clock_rewound": false,
     "verification": {
       "vs_snapshot":      { "max_pose_delta_m": 0.0, "exact": true, "sampled_nodes": 5 },
       "moved_by_restore": { "max_pose_delta_m": 2.828427, "max_pose_delta_node": "BALL" },
       "poses_after": { "BALL": [1.0, 0.0, 0.1], "...": [] } } }

GET /sim/snapshots
-> { "snapshots": [ { "name": "__init__", "sim_time_ms": 0.0,
                      "sampled_nodes": null, "engine_provided": true,
                      "note": "the engine's parse-time state: ...", "age_s": 12.8 },
                    { "name": "t0", "sim_time_ms": 224.0, "sampled_nodes": 5,
                      "engine_provided": false, "age_s": 9.7 } ] }
```

- **`/sim/restore` does not rewind the clock** (`clock_rewound: false`): it
  puts the bodies back. Use `/sim/reset` (§7.11) for `t = 0`.
- **Restoring a name that was never saved is refused (`404`
  `SNAPSHOT_NOT_FOUND`), and that refusal is load-bearing.**
  `OmPose`'s saved-pose map is a `QMap` whose `[]` **default-constructs a
  zero vector** on a miss (`src/omnisim/nodes/OmPose.hpp`), so restoring an
  unknown state name would silently teleport the whole scene to the origin.
- **Snapshot names live in the supervisor process**, which is restarted by
  every world load — so a load clears them, and a snapshot never outlives
  the world it describes. `"__init__"` (§7.11) is always present and is the
  only one with `engine_provided: true`; names beginning `__` are reserved
  (`400` `SNAPSHOT_NAME_RESERVED`).
- Verified on Newton: after moving `BALL` 2.83 m and restoring,
  `vs_snapshot.max_pose_delta_m` is `0.0`. Cost: 0.72–0.85 s (Newton,
  light). It was verified on ODE too while ODE shipped (`6.6e-05`, the body
  still settling; 0.013–0.034 s) — historical, since ODE was deleted on
  2026-08-08.

### 7.33 POST /robot/<def>/joints/set

The harness's first robot-**commanding** endpoint: supervisor-driven joint
position targets with settle-and-verify semantics. Joint names are the same
ones `GET /robot/<def>/joints` (§7.14) reports.

**Request:**

```json
{ "joints": { "shoulder_lift_joint_motor": -1.4, "elbow_joint_motor": 1.0 },
  "settle_steps": 16 }
```

(Parallel `"names": [...]` + `"positions": [...]` lists are accepted as an
alternative to the `joints` object.)

**Response (trimmed):**

```json
{ "robot": "RIG",
  "joints": { "servo_limited": {
      "requested": 0.6, "commanded": 0.6, "clamped": false,
      "position_before": 0.0, "achieved": 0.5999995, "error": -4.53e-07,
      "moved": true, "position_controllable": true,
      "limits": { "lower": -1.0, "upper": 1.0,
                  "source": "motor minPosition/maxPosition" } } },
  "sim_time_ms": 1136.0,
  "verification": { "applied": 1, "settle_steps": 16,
                    "sim_time_advanced_ms": 256.0,
                    "max_abs_error": 4.53e-07,
                    "max_abs_error_joint": "servo_limited",
                    "semantics": "PD setpoint, not a teleport: ..." } }
```

- **NOT a teleport.** `Node.setJointPosition()` also re-pins the motor's PD
  target (`OmJoint.cpp`), so under Newton the joint **converges over
  ticks** — hence `settle_steps` (default 16) and the measured
  `achieved`/`error` per joint, never the argument echoed back. A large
  residual error usually means more `settle_steps` — or a controller
  fighting the write (below).
- **Targets beyond the joint's hard stops are clamped and flagged.**
  `commanded` is the adopted (clamped) value with `clamped: true`;
  `error` is measured against the adopted value. The clamp mirrors the
  engine's own (`OmJointParameters::clampPosition`): `minStop`/`maxStop`
  only, and `minStop == maxStop == 0` means unconstrained.
- **⚠️ A motor with no position limits is a velocity wheel, and position
  targets on it are silently ignored by the physics** (built with `ke = 0`
  — `OmBasicJoint.cpp`; 1680 such joints exist in-tree). The endpoint
  pre-classifies every joint by the same rule the engine uses (motor
  `minPosition`/`maxPosition` when they differ, else the joint's stops;
  sliders are always servos) and reports it as
  `position_controllable: false` with the mechanism in `note`, never bare
  success. Measured: a limit-less wheel commanded to 2.0 rad reads
  `achieved ≈ 0`, `moved: false`, `error: -2.0`.
- **⚠️ A robot whose controller re-asserts its own targets wins.** The
  OmniLink bridges' hold mode re-applies its setpoints every tick
  (`omnilink_arm_bridge.py`: "Re-apply target each tick so motors don't
  drift"), so on a bridge-driven robot the write is overwritten within the
  settle window and `achieved` snaps back to `position_before`. Measured on
  the UR5e chat world: all three commanded joints returned to their hold
  pose (error 0.42–0.70 rad). For clean supervisor-side joint control, use
  a robot with a passive/no-op controller — or command the robot through
  its own bridge (§5).
- Unknown joints refuse the **whole batch** (nothing is written), naming the
  offenders and the addressable joints. Works in light and heavy supervisor
  mode. A write verb: never transparently retried by the harness.

Errors: `404` + `DEF_NOT_FOUND`, `422` + `JOINT_NOT_FOUND`, `409` +
`JOINT_NAME_AMBIGUOUS` (two joints share a name), `400` +
`JOINTS_UNSPECIFIED` / `ARGUMENT_MISSING` / `ARGUMENT_INVALID`.

### 7.34 POST /robot/<def>/ik

Batched inverse-kinematics **preview** against the live Newton model
(`World.solve_ik`, internal parity plan, item W2.1). A **pure read**: nothing in the
scene moves — the endpoint returns joint angles and a per-target residual,
and the caller applies the angles (or not) with a separate
`POST /robot/<def>/joints/set` (§7.33), using the same joint names.

**Request:**

```json
{ "effector": "TIP",
  "targets": [[0.35, 0.0, 0.45], [0.5, 0.0, 0.2]],
  "rotations": [[0, 0, 0, 1], [0, 0, 0, 1]],
  "tool_offset": [0.0, 0.0, 0.3],
  "iterations": 64 }
```

`effector` (required) is the DEF of the end-effector Solid; `targets`
(required) are world-frame positions. `rotations` optionally pairs each
target with a `[qx, qy, qz, qw]` orientation goal; `tool_offset` is a TCP
offset in the effector's own frame (a gripper's grasp point rather than its
link origin — required whenever the effector DEF is a massless tip folded
into its parent body, since the solve then targets the parent body's frame).

**Response (trimmed):**

```json
{ "robot": "IKARM", "effector": "TIP",
  "solved_joints": [ {"name": "shoulder", "node_id": 101, "appliable": true},
                     {"name": "elbow", "node_id": 102, "appliable": true} ],
  "results": [ { "target": [0.35, 0.0, 0.45],
                 "residual_m": 3.1e-06,
                 "joints": {"shoulder": 0.9207, "elbow": -1.2622} } ],
  "solve_ms": 154.2,
  "verification": { "semantics": "PURE PREVIEW: nothing moved. ...",
                    "warmup": "the FIRST solve per world compiles a warp kernel ..." } }
```

- **The residual is measured, not asserted.** `residual_m` (metres) comes
  from forward kinematics on **exactly the returned angles** — after
  clamping to the authored joint limits — so an unreachable target reports
  its real miss instead of a flattering solver internal. Branch on it:
  reject a target rather than driving to it, and never report "reached"
  from this call alone.
- **Angles are keyed by joint name and ready to apply.** The engine solves
  every Hinge/Slider joint of the effector's robot that is registered with
  the physics backend (Hinge2/Ball joints are multi-coordinate and excluded
  by design), and answers joint *node ids*; the supervisor maps them onto
  the same names `GET /robot/<def>/joints` reports. A solved joint the
  robot walk cannot name (wrong `def` for the effector's robot, or an
  unnamed joint) comes back keyed `node_<id>` with `appliable: false` and
  is disclosed in `verification.unmapped_node_ids` — never silently
  dropped.
- **⚠ The first solve per world compiles a warp kernel.** Measured (machine
  `9722d23d12a3`): 8.3 s truly cold on a 6R arm; 2.37 s first-call on a 2R
  rig with a warm on-disk warp cache; **106–116 ms warm** in-process.
  `solve_ms` in the response is the measured cost of that call — budget the
  first request's timeout accordingly.
- **The mask is the point.** Only the effector robot's own 1-DoF joint
  slots are solved; unmasked, newton's optimiser "reaches" targets by
  translating a floating base or another robot's joints — coordinates the
  caller cannot command (measured 0.923 m of base translation;
  `tests/test_newton_ik_slots.py`).
- Seeds come from the **live** joint angles, so consecutive calls warm-start
  from the current pose. Verified on the default CPU `"mujoco"` solver
  only; **unverified on `mujoco_warp`**. Light supervisor mode verified
  live; heavy runs the identical path (no tracker involved). A pure read,
  transparently retryable.

Errors: `404` + `DEF_NOT_FOUND` (robot or effector), `422` + `IK_NO_BODY`
(the effector owns no Newton physics body) / `IK_NO_JOINTS` (no Hinge/Slider
joint registered), `503` + `IK_UNAVAILABLE` (no backend / world not
finalised — retry after finalize), `500` + `IK_SOLVER_FAILED`, `400` +
`EFFECTOR_UNSPECIFIED` / `TARGETS_UNSPECIFIED` / `ARGUMENT_INVALID`.

---

## 8. Capture Service

Service: `scripts/capture/omnisim_capture.py`. Default port `6791`,
supervisor IPC on `6792`. The capture service is the harness's sister
— same HTTP+supervisor-injection shape, but tuned for cinematic output
(high-resolution stills, deterministic camera-path sequences, movie
encoding) rather than tight authoring loops.

### 8.1 GET /protocol (PLANNED)

See §4.1 (not yet implemented by the capture service).
`service = "capture_service"`. When shipped, `instance` includes
`"ffmpeg": "/usr/bin/ffmpeg"` or `null`.

### 8.2 POST /world/load

```json
{ "path": "...",
  "width": 3840, "height": 2160,
  "fov": 0.785, "wait_s": 30.0 }
```

Same diagnostic codes as §7.3. The supervisor injected here carries a
`Camera` device sized to `(width, height)` so renders are independent
of any GUI viewport — 4K and 8K both work.

### 8.3 POST /capture/camera

```json
{ "position": [x,y,z], "target": [x,y,z],
  "orientation": [ax, ay, az, angle],
  "sync_viewpoint": true }
```

Position the capture camera. `orientation` is optional (computed from
`target` when absent). `sync_viewpoint: true` also moves the live
`Viewpoint` so the GUI matches if one is attached.

### 8.4 POST /capture/screenshot

```json
{ "path": "still.png", "quality": 100, "source": "capture_camera" }
```

`source = "capture_camera"` (default) renders from the dedicated camera;
`source = "viewpoint"` renders from the live viewpoint. Returns
`{ "ok": true, "path": "..." }` or streams the PNG.

### 8.5 POST /capture/movie/start | /capture/movie/stop | /capture/movie/status

`{ "path": "out.mp4", "codec": "h264", "quality": 100,
   "acceleration": 1, "caption": "...", "fps": 60 }`.

`/capture/movie/status` is GET; returns
`{ "encoding": bool, "frames_captured": int, "elapsed_s": float }`.

### 8.6 POST /capture/sequence

The single endpoint that walks a keyframe path frame-by-frame, captures
PNGs, and ffmpeg-encodes the result.

```json
{
  "path_keyframes": [
    { "t": 0.0, "position": [...], "target": [...] },
    { "t": 30.0, "position": [...], "target": [...] }
  ],
  "duration_s": 30.0, "fps": 60,
  "output": "social/youtube_videos/captures/orbit.mp4",
  "codec": "h264", "crf": 8, "ease": "smoothstep",
  "warmup_steps": 50, "settle_steps_per_frame": 1,
  "keep_frames": false,
  "playback_speed": 1.0
}
```

`codec` ∈ `{ "h264", "h265", "vp9", "prores" }`. `ease` ∈
`{ "linear", "smoothstep", "smootherstep", "ease_in", "ease_out", "ease_in_out" }`.
`playback_speed` < 1 = slow motion; `> 1` = time-lapse.

### 8.7 POST /sim/step, POST /sim/reset, GET /sim/state

Same shape as the harness equivalents (§7.10–7.12). The capture
supervisor runs `synchronization: TRUE` — every `/sim/step` is
guaranteed to advance exactly the requested number of basic timesteps.

⚠️ "Same shape" includes the caveats: `/sim/step` returns
`{sim_time_ms, advanced_to_ms}` (§7.10) and `/sim/reset` **does not
restore node state** (§7.11).

### 8.8 GET /healthz

Liveness probe. **Response:**
`{ "ok": true, "uptime_s": float, "ffmpeg": "<path>" | null }` — the
`ffmpeg` field is the capture-specific addition, and a `null` there is
why `/capture/sequence` and `/capture/movie/*` will fail later. Check it
before starting a long render.

### 8.9 GET /world/robots, POST /world/robots

Enumerate robot nodes in the loaded world. Registered on **both** verbs
with identical behaviour and no body; `POST` exists so a client that
posts to everything else on this service does not need a special case.

**Response:** `{ "robots": [ { "name": ..., "def": ..., "translation": [x,y,z] } ] }`.
Thinner than the harness's `/robots` (§7.13) — no orientation, no
controller, no joint count.

### 8.10 POST /world/subject

Subject-pose lookup for the cinema pipeline: it is what makes a camera
move subject-relative rather than world-coordinate-relative, so
`tracking_side(omniquad)` works wherever OmniQuad was spawned.

**Request:** `{ "name": "omniquad" }` or `{ "def": "OMNIQUAD" }` — at least one
is required. `def` is tried first via `getFromDef`, then `name` by
walking the scene for a matching robot.

**Response:** `{ "name": ..., "def": ..., "translation": [x,y,z], "rotation": [ax,ay,az,angle] }`.
A `503` (`CommandError`) if no robot matches.

### 8.11 POST /shutdown

Graceful exit: tears down the OmniSim subprocess in-process, replies
`{ "shutting_down": true }`, then exits the service ~0.5 s later.

This exists because on Windows `TerminateProcess` on the service skips
the signal handler that would otherwise clean up the engine child, so
`render.py --ad-hoc` would leave orphaned `omnisim-bin` processes. Prefer
it over killing the process. The harness (§7) has **no** equivalent.

---

## 9. Twin Shadow *(reserved — NOT IMPLEMENTED)*

> ⚠️ **Status: reserved design. No implementation exists in this repository.**
> Nothing in the tree serves `/shadow_state` or `/shadow/disable`, reports
> `state_source`, or reads external robot telemetry. This section pins the
> names and payload shapes ahead of an implementation; it is **not** a
> description of working behaviour, and it is **not** a claim that OmniSim
> does validated sim-to-real or digital-twin synchronisation today.
>
> The `MUST`s below are conditional: they bind *a bridge that chooses to
> implement Twin Shadow*. No such bridge ships. Do not write a client against
> this section expecting it to connect to anything.

Twin Shadow would extend a Robot Bridge with two endpoints that hard-snap
the simulated robot's pose from external telemetry. The shadow
endpoints live on the same port as the bridge they extend (the bridge
declares the extension in `/capabilities`).

A bridge that implements twin shadow MUST:

- Include `"twin_shadow": "1.0"` in `/protocol.service_versions`.
- Include `"shadow_state"` and `"shadow_disable"` in
  `/capabilities.actions` OR expose the dedicated endpoints below.
- Report `state_source ∈ {"sim", "shadow", "shadow_stale"}` in `/state`.

### 9.1 POST /shadow_state

**Request:**

```json
{
  "q": [6 floats],
  "base_pose": { "xyz": [x,y,z], "rpy": [r,p,y] },
  "t_real_s": 1715180000.123,
  "seq": 42
}
```

- `q` — required for jointed robots.
- `base_pose` — optional; when present, the bridge writes
  `translation` and `rotation` directly via Supervisor.
- `t_real_s` — optional Unix epoch float. Used by the bridge to detect
  stale telemetry.
- `seq` — optional monotonic sequence number. Out-of-order packets MUST
  be dropped silently and reported in the response.

**Response:**

```json
{ "ok": true, "accepted": true, "seq": 42, "ttl_s": 0.5 }
```

or, on out-of-order or rejected:

```json
{ "ok": false, "error": "shadow_out_of_order", "message": "...",
  "details": { "received_seq": 41, "last_seq": 42 } }
```

### 9.2 POST /shadow/disable

Force immediate exit from shadow mode. The bridge resumes driving from
the last `commanded_q` (kept in sync with the last shadow `q` so there
is no jolt at handover). Returns `{ "ok": true, "shadow": "disabled" }`.

### 9.3 Shadow timing

Bridges MUST implement a TTL on shadow telemetry. The recommended
default is **0.5 s** (any `/shadow_state` extends the window by this
much). When the window expires the bridge transitions
`shadow → shadow_stale → sim` over one bridge tick, MAY emit a
`fault.shadow_stale` (see §10), and resumes from `commanded_q`.

`/state.discrepancy_q` reports the max joint error between the most
recent shadow `q` and the actual simulated `q` over the last tick.
Clients use this to monitor twin health.

---

## 10. Event taxonomy

Events flow through `GET /sim/events` (§7.19) and through bridge-level
event streams (a v1.1 addition). Each event carries:

- `type` — namespaced dotted name from §10.1.
- `seq` — monotonic cursor, per side (§7.19).
- a timestamp — `t_sim_ms` (int ms of sim time) on supervisor-produced
  events, `t_wall` (float Unix epoch) on harness log events.

> **⚠️ There is no `t` field.** Earlier text for this section required
> every event to carry `t` as float sim-seconds. Nothing in the tree has
> ever emitted it. Read `t_sim_ms` and divide by 1000, and check `source`
> (§7.19) first — a `t_wall` value is a wall clock, not a sim clock, and
> silently differencing the two produces garbage.

### 10.1 The authoritative type list

**Exactly ten types are emitted.** `GET /sim/events?types=` is an
exact-match allowlist with no validation (§7.19), so a name that is not
in this table yields an empty `200` rather than an error. Copy these
strings literally.

| Type | Source | Emitter |
|---|---|---|
| `contact.began` | `sup` | `event_bus.py` · `ContactTracker` |
| `contact.ended` | `sup` | `event_bus.py` · `ContactTracker` |
| `joint.limit_hit` | `sup` | `event_bus.py` · `JointLimitTracker` |
| `grip.acquired` | `sup` | `event_bus.py` · `GripTracker` |
| `grip.released` | `sup` | `event_bus.py` · `GripTracker` |
| `damage.impact` | `sup` | `harness_supervisor.py` · `_emit_with_fanout` |
| `damage.state_transition` | `sup` | `harness_supervisor.py` · `_emit_transition_with_fanout` |
| `controller.log` | `log` | `omnisim_harness.py` · `LogRingBuffer.emit_controller_log` |
| `world.warning` | `log` | `omnisim_harness.py` · `LogRingBuffer.emit_world_diagnostic` |
| `world.error` | `log` | `omnisim_harness.py` · `LogRingBuffer.emit_world_diagnostic` |

**Ask the harness instead of trusting this table.** `GET /capabilities`
(§7.28) serves `event_types` from the code: the supervisor scans its own
`emit()` call sites and cross-checks them against
`event_bus.SUPERVISOR_EVENT_TYPES`, the harness does the same for its
three log types, and the response carries `event_types_detail` with
`undeclared` / `declared_not_emitted` / `verified` so drift is visible
rather than silent. It also reports which types the **running**
configuration actually produces: a `--light` supervisor suppresses the
five contact / grip / joint types, and they appear in
`event_types_detail.suppressed` with the reason.

Equivalent greps, if you have the tree but not a running harness:

```bash
grep -rhoE '(bus|_bus)\.emit\("[a-z_]+\.[a-z_]+"' \
    projects/default/controllers/harness_supervisor/ | sort -u
grep -nE '"(controller\.log|world\.(warning|error))"' \
    scripts/harness/omnisim_harness.py
```

**Four names in earlier editions of this section do not exist.** They are
listed here because a client written against them fails silently:

| Documented (wrong) | Reality |
|---|---|
| `grip.began` | renamed → **`grip.acquired`** |
| `damage.applied` | renamed → **`damage.impact`** (different payload too) |
| `damage.state_changed` | renamed → **`damage.state_transition`** (different payload too) |
| `damage.part_detached` | **never emitted** — no producer exists anywhere in the tree |

### 10.2 Contact

- `contact.began { a_def, b_def, point: [x,y,z] }`
- `contact.ended { a_def, b_def }`

`normal_force` was previously listed as an optional field on
`contact.began`; no producer emits it. `a_def` / `b_def` fall back to
`"#<node_id>"` for a node with no DEF. Pairs are order-normalised by node
id, so `(a, b)` and `(b, a)` are one event, and which node lands in
`a_def` is not stable across runs.

### 10.3 Grip (inferred gripper attachment)

- `grip.acquired { gripper_def, held_def }`
- `grip.released { gripper_def, held_def, held_for_ms }`

Inferred from stable contact membership, not reported by the engine —
see §7.18 for the 3-poll stability delay. `held_for_ms` on release is
undocumented in previous editions but always present.

### 10.4 Joints

- `joint.limit_hit { joint, side: "lower"|"upper", position, lower, upper }`

**No `robot_def`.** The emitter deliberately skips the owning-robot
lookup ("agents can correlate via joint name" — `event_bus.py`), so an
event cannot be attributed to a robot in a multi-robot scene without a
prior `/robot/<def>/joints` call to build a name map. The band field is
`side`, not `direction`; `position`, `lower` and `upper` are extra.

Only the **entering** transition fires — there is no "joint left the
stop" event — and hysteresis (hit at 1e-3, clear at 5e-3 from the stop)
suppresses chatter at the band edge. Joints with `minStop == maxStop == 0`
are treated as unconstrained and never emit.

### 10.5 Damage

- `damage.impact { part, impulse_J, point: [x,y,z], other }`
- `damage.state_transition { part, from_state, to_state, hp, trigger_impulse_J }`

Note `part` (a part name), not `target_def`; and `impulse_J` /
`trigger_impulse_J` with a capital J, not `impulse_j`. There is no
`hp_before` / `hp_after` pair — `state_transition` carries a single
post-transition `hp`.

These are a fan-out of the damage tracker's own log, which is also
readable with its own cursor at `GET /robot/damage/events` (§7.25); that
copy additionally carries `step_id`.

(See [§13.7 of docs/developer/engine-migration-plan.md](docs/developer/engine-migration-plan.md)
for the full damage event design, parts of which remain unimplemented.)

### 10.6 Controller / log

- `controller.log { stream: "stdout"|"stderr", line }`

**No `robot_def` and no `text`.** The message field is `line`, and the
harness reads the OmniSim subprocess's merged stdout/stderr, so it cannot
attribute a line to a robot at all. To tell controllers apart, have each
one prefix its own output.

### 10.7 World / harness

- `world.warning { code?, message, raw }`
- `world.error { code?, message, raw }`

Parsed from `omnisim_log.txt` deltas. `code` is a §7.3 diagnostic code.
`raw` is the original log line. Severity maps `fatal`/`error` →
`world.error`, `warning` → `world.warning`; **`info` and unrecognised
lines are dropped entirely** rather than passed through as a lower
severity, so this stream is not a complete log tail.

### 10.8 Faults (bridge-emitted) *(reserved — not emitted)*

- `fault.raised { robot_id, code, message }` — see §11.
- `fault.cleared { robot_id, code }`

⚠️ No service in this repository emits either type. Bridges surface
faults as the `fault` object inside `GET /state` (§5.3); there is no
bridge-level event stream yet (it is the v1.1 addition referenced above).
These names are reserved, not implemented.

### 10.9 Twin *(reserved — not emitted)*

- `shadow.stale { robot_id, last_t_real_s }`
- `shadow.resumed { robot_id }`

⚠️ Part of §9, which is an entirely unimplemented reserved design.

Future minor versions MAY add new types. Clients MUST ignore unknown
types, never fail on them.

---

## 11. Fault codes

The following error codes are reserved by v1.0 and MUST be used with
the meanings below whenever applicable. They appear in the `error`
field of the error envelope (§3.2), in `/state.fault.code`, and in
`fault.raised` events.

| Code | Meaning |
|---|---|
| `controller_lost` | The bridge cannot reach its controlled robot's controller process. |
| `joint_limit` | A requested joint position is outside the limits in `/capabilities.joint_limits`. |
| `ik_nonconvergent` | The IK solver ran to `max_iters` without satisfying `tol`. |
| `ik_singular` | The Jacobian became singular during IK. |
| `unreachable_target` | The requested TCP target is outside `workspace`. |
| `telemetry_stale` | A telemetry source the bridge depends on has not updated within its TTL. |
| `shadow_stale` | Twin telemetry has not arrived within `shadow.ttl_s`. |
| `shadow_out_of_order` | A `/shadow_state` request arrived with a `seq` lower than the last one. |
| `effector_unavailable` | A requested effector (gripper, sensor, …) is not present on this robot. |
| `world_not_loaded` | The harness was asked to do something that requires a loaded world. |
| `protocol_unsupported` | The requested wire-protocol version is not supported. |
| `invalid_action` | The `action` field of `POST /action` is not in `/capabilities.actions`. |
| `missing_field` | A required field is absent from the request body. |
| `malformed_json` | The request body is not valid finite UTF-8 JSON. |
| `invalid_body` | The JSON body has the wrong top-level shape. |
| `invalid_type` | A field has the wrong JSON type. |
| `invalid_request_id` | The optional request id is empty, too long, or not a string. |
| `duplicate_request` | The same mutating request id was already accepted and will not run again. |
| `value_out_of_range` | A field's value is outside its documented range. |
| `unauthorized` | A configured bridge token was missing or invalid. |
| `origin_not_allowed` | A browser Origin is outside the configured allowlist. |
| `request_too_large` | The JSON body exceeds the configured request-size limit. |
| `not_found` | The requested endpoint does not exist. |
| `not_supported` | The endpoint exists but this robot class does not implement the action. |
| `internal_error` | An unexpected implementation failure occurred; implementation details are not exposed. |
| `proto_not_found` | A referenced PROTO definition could not be located. |
| `world_parse_syntax_error` | The `.wbt` file failed to parse. |
| `world_load_timeout` | The world did not become ready within `wait_s`. |

Bridges MAY define additional codes; new codes MUST follow the same
snake_case convention and MUST be documented in `/capabilities.actions`
or in a bridge's own README. Reserved-but-not-yet-emitted codes are
safe; never-documented codes are not.

---

## 12. Multi-instance and port allocation

OmniSim is designed for N parallel `omnisim-bin` processes on one host
(legacy `omnisim-bin` name is an identical-content alias). The
wire-protocol implications:

### 12.1 Default ports are advisory

Every service in this document declares a default port. None of them
are mandatory — every service MUST accept a `--port <N>` command-line
flag (and, where applicable, a `--supervisor-port <N>` flag).

### 12.2 Auto-port range

Where an `omnisim-bin` process auto-allocates its own TCP port (for
extern-controllers and robot windows), it scans `[1234, 1244]` for a
free slot. Tools that talk to a specific `omnisim-bin` MUST read its
chosen port from the process's stdout/log (the simulator logs the
actual port when it falls back off the default).

### 12.3 Per-instance log path

Parallel-instance children MUST set `OMNISIM_LOG_PATH` to a unique
path; otherwise they all write the shared `OMNISIM_HOME/omnisim_log.txt`
and the last writer wins (no usable per-child log).

### 12.4 Per-instance tmp / IPC dir

`omnisim-bin` automatically salts its tmp path with the chosen TCP
port so controller IPC sockets do not collide between parallel
instances. No client action is required.

### 12.5 Harness and capture sharing one host

The harness (`6789`/`6790`) and capture service (`6791`/`6792`) can
coexist. Two harnesses cannot share defaults; the second instance MUST
take both `--port` and `--supervisor-port`. Failing to do so produces
`error = "harness_port_collision"` with copy-pasteable remediation in
`message`.

---

## 13. Compatibility and version negotiation

### 13.1 Simulator → protocol map

| OmniSim simulator | Wire protocol versions spoken |
|---|---|
| `1.0.0` – `1.0.10` | `1.0` (this document) |
| `2.0.0`            | `1.0` (this document) |

Future simulator releases extend this table.

### 13.2 Client → simulator handshake

> **Note:** this handshake is available on canonical robot bridges. The world
> harness and capture service still return `404` and require the best-effort
> path described at the end of this section.

A v1.0-compliant client SHOULD:

1. Connect to the service's port.
2. Issue `GET /protocol`.
3. Verify `omnisim_wire` major matches the version it was built
   against.
4. Cache `instance.sim_version` for logging.
5. Optionally send `Accept-Protocol-Version: 1.0` on subsequent
   requests for end-to-end version pinning.

If `GET /protocol` returns `404`, the service is pre-v1.0; clients MAY
fall back to a best-effort path or refuse to operate.

### 13.3 Adding a field is not a breaking change

Clients MUST ignore unknown response fields. Servers MUST ignore
unknown request fields except where explicitly documented as strict
(`POST /action.action` is strict; everything else is lenient).

### 13.4 Removing a field IS a breaking change

Even a field that "no one was using" is potentially a breaking change.
Removals wait for a major bump.

### 13.5 Renaming endpoints

Where a bridge documents *and implements* a dual name (`/state` and
`/get_robot_state`, `/list_robots` and `/capabilities`), both spellings MUST
keep working for the lifetime of major version 1. v2.0 MAY collapse them.

This applies only to names a bridge actually serves. It is **not** a guarantee
that every alias listed in this spec exists — `/read_camera` in particular is
reserved but unimplemented (§16). Probe `/capabilities` rather than assuming an
alias is live.

---

## 14. Stability commitment

### 14.1 What is stable

Within version `1.x`:

- The endpoint URLs in §5.1–§5.7, §6, §7, §8.
- Required request fields and their semantics.
- The error envelope shape (§3.2).
- The reserved fault codes (§11).
- The event type names and their required fields (§10).

Tools that depend on these can be written once against `1.x` and not
revisited until `2.0` is cut — **subject to the compliance gaps in §16**,
which list the places where the in-tree services do not yet meet the spec.
Check §16 before depending on an endpoint.

> **Note on the 2026-07-26 §10 correction.** Four event names in §10 were
> corrected (`grip.began` → `grip.acquired`, `damage.applied` →
> `damage.impact`, `damage.state_changed` → `damage.state_transition`,
> `damage.part_detached` withdrawn as never-implemented), along with
> several payload fields. **This is not a wire change and not a `1.x`
> break** — the corrected names are what the harness has emitted since
> the event stream shipped; the spec was wrong. No client could have been
> depending on the old names, because `?types=` filtering on them
> returned an empty stream (§7.19). §7 was corrected the same way and on
> the same principle: this document declares itself normative, but where
> it contradicted shipped behaviour the shipped behaviour is what clients
> actually met.

§9 (Twin Shadow) is **excluded** from this commitment: it is a reserved
design with no implementation, and its shapes may change before it ships.

The `GET /protocol` shape (§4.1) is stable for robot bridges. Harness and
capture implementations are still pending (see §16).

### 14.2 What is unstable

- The exact wording of `message` strings.
- The contents of `details` objects.
- The exact values of `mode` strings.
- The set of optional response fields. New optional fields may appear
  at any minor version.
- Bridge-defined extension actions (anything not in §6).

### 14.3 What is experimental

Endpoints in this document marked **(experimental)** — currently none —
may change without a minor bump. v1.0 ships no experimental endpoints;
the slot is reserved.

### 14.4 Vendor extensions

Vendors implementing custom bridges or harnesses MAY add their own
endpoints under an `x-<vendor>-` prefix:

```
GET  /x-omnilink-policy
POST /x-acme-grasp
```

Vendor extensions are not part of the wire protocol; clients targeting
multiple vendors MUST gate on `/protocol.extensions` rather than
assume them.

---

## 15. Out of scope

The following are explicitly NOT part of v1.0:

- **A universal authentication protocol.** Canonical robot bridges provide an
  optional bearer-token guard and require it for non-loopback binding, but key
  issuance, rotation, roles, and identity are outside v1.0. Internet-exposed
  deployments still require a TLS reverse proxy and deployment-specific auth.
- **Encryption.** Loopback HTTP, no TLS. Same rationale.
- **Streaming transports.** No WebSocket, SSE, or gRPC. Polling and
  cursor-paged event responses cover every current use case; a v1.1
  may add `WebSocket /sim/events/stream` once an existing client needs
  it.
- **Cross-host clustering.** Multi-instance is a single-host story;
  cross-host coordination is the caller's problem.
- **OpenAPI / JSON Schema.** Machine-readable schemas of every shape
  in this document are tracked as a v1.1 deliverable
  (`docs/protocol/schemas/`) — they will be additive, not authoritative.
- **Robot-specific control parameters.** PID gains, motor limits,
  trajectory profiles. These remain bridge-specific and live in
  bridge READMEs.

---

## 16. Reference implementations

Each surface has a canonical in-tree reference implementation. New
bridges and services SHOULD copy the closest reference rather than
reinvent the wire envelope.

| Surface | Reference | Notes |
|---|---|---|
| Robot Bridge — arm | [projects/samples/demos/controllers/omnilink_arm_bridge/](projects/samples/demos/controllers/omnilink_arm_bridge/) | Port `8765`. Generic 6-DOF arm bridge; the arm is selected with `--robot <id>` from the registry in `_arm_configs.py`. Axis-style one-verb-per-endpoint surface only — does **not** implement typed `/action` (§5.4); its `/stop_robot` / `/reset_to_home` responses are pre-v1.0 (no `ok`/`accepted` envelope). |
| Robot Bridge — mobile | [projects/samples/demos/controllers/omnilink_mobile_bridge/](projects/samples/demos/controllers/omnilink_mobile_bridge/) | Port `8765`. Husky / Jackal / Rosbot / TurtleBot3 — skid-steer kinematics. The `husky_omnilink_bridge` sibling is a **separate** controller on port `6070` (eye camera on `6071`); it implements typed `POST /action` but none of the §5 required endpoints. |
| Robot Bridge — flying | [projects/samples/demos/controllers/mavic_omnilink_bridge/](projects/samples/demos/controllers/mavic_omnilink_bridge/) | Port `6090`. Mavic 2 Pro with gimbal camera + marker perception. Implements typed `POST /action`. |
| Robot Bridge — quadruped | [projects/samples/demos/controllers/omnilink_quadruped_bridge/](projects/samples/demos/controllers/omnilink_quadruped_bridge/) | Port `8765`. OmniQuad. Motions: `stand`, `sit`, `crouch`, `settle`, `wave`, `walk`, `stop`, `home` — driven through `/tool` / `/prompt`, not dedicated routes. ⚠️ Its `walk` is a **wave-gait leg cycle with supervisor-driven body translation**, i.e. scripted, not physically-actuated locomotion. Learned OmniQuad locomotion lives in `projects/policies/`, not in this bridge. |
| World Harness | [scripts/harness/omnisim_harness.py](scripts/harness/omnisim_harness.py) | Port `6789` + supervisor `6790`. |
| Capture Service | [scripts/capture/omnisim_capture.py](scripts/capture/omnisim_capture.py) | Port `6791` + supervisor `6792`. |

Conformance: a bridge is **v1.0 compliant** when:

- `GET /protocol` returns `{"omnisim_wire": "1.0", ...}` with the
  correct `service` value.
- All §5 required endpoints are present and conform.
- All applicable §6 per-class endpoints are present.
- Errors use the §3.2 envelope and codes from §11.

**Open compliance gaps at time of writing** (the spec above states the
v1.0 target; these are the deltas the in-tree services still owe). This list
is the honest delta — read it before depending on any endpoint:

- **§9 Twin Shadow is entirely unimplemented.** No service serves
  `/shadow_state` or `/shadow/disable`, and no bridge reports `state_source`.
  The whole surface is a reserved design.
- **`GET /protocol` and the `X-OmniSim-Wire` / `X-OmniSim-Service`
  headers remain missing on the world harness and capture service.** Canonical
  robot bridges implement them (§4.1, §4.3). **`GET /capabilities` now
  exists on the harness** (§7.28, 2026-07-26) and closes the discovery
  gap — route list, physics backend from the engine's own verdict sidecar,
  measured step cost, RPC timeout, diagnostic codes, event types, and an
  explicit `not_supported` list, all served from the code and
  self-cross-checked. `GET /protocol` itself (the §4.1 shape) is still
  owed, as is the capture service's discovery surface.
- **The §3.2 error envelope is not implemented by the harness.** Harness
  errors are mostly `{"error": "<free text>"}` — no `ok: false`, no
  snake_case code, no `message`/`details` split. Bridges are closer but not
  uniform. The exceptions are `POST /scene/frame`'s `BAD_VIEW_MODE`
  (§7.21) and the 2026-07-26 mutation/state verbs, which do return
  `{ok: false, error, code}` with `DEF_NOT_FOUND`, `PARENT_DEF_NOT_FOUND`,
  `CLONE_DEF_NOT_FOUND`, `SNAPSHOT_NOT_FOUND`, `SNAPSHOT_NAME_RESERVED`,
  `FIELD_NOT_ON_NODE`, `POSE_UNSPECIFIED`, `SPAWN_SPEC_INVALID` and
  `SPAWN_REJECTED` (the live set is in `/capabilities.request_error_codes`).
  The older endpoints still answer with prose, so the failure path is only
  half converted.
- ~~**`POST /sim/reset` does not restore node state**~~ — **fixed
  2026-07-26** (§7.11): it now rewinds the clock *and* loads the engine's
  parse-time `"__init__"` state, with `POST /sim/snapshot` /
  `POST /sim/restore` (§7.32) for named states. Verified on both backends.
  Two caveats remain: the returned `verification` samples top-level poses
  only, and `"restore": null` still gives the old clock-only behaviour.
- **`POST /sim/step` can cost tens of seconds per 16 ms step, and a step
  over 120 s drops the supervisor connection unrecoverably** (§7.10).
  Neither the cost nor the timeout is discoverable over the wire.
  Proposed: same doc, G1 / P2.
- **A missing world path yields no diagnostic code** — prose plus
  `diagnostics: []`, despite `WORLD_FILE_NOT_FOUND` existing in
  `diagnostic_codes.py` (§7.3). Proposed: same doc, G7 / P5.
- **No `paused` field, and no way to pause, resume, or quit** the
  simulation over HTTP (§7.12). `Supervisor.simulationSetMode()` exists
  in the shipped controller binding and is wired to nothing.
- **No spawn / delete / set-entity-state.** An agent composing a scene
  must hand-author `.wbt` text and pay a full world load per mistake. The
  shipped Supervisor binding already has
  `Field.importMFNodeFromString()`, `Node.remove()`, `Node.setVelocity()`
  and `Supervisor.worldSave()`; the harness supervisor exposes none of
  them, and its only field write in the entire scene is
  `Viewpoint.position` / `.orientation`. Proposed: same doc, G4 / P3.
- **§10.8 (`fault.*`) and §10.9 (`shadow.*`) are reserved names with no
  producer anywhere in the tree.** Bridges report faults through the
  `fault` object in `GET /state` (§5.3) instead; there is no bridge-level
  event stream.
- **`/world/load` returns 422 on a load failure** (not the 400/409/500 in
  §7.3's original text; the §7.3 example is now correct).
- **Per-class endpoints (§6) are largely not exposed as endpoints.** The
  flying bridge (`mavic_omnilink_bridge`) implements `takeoff` / `land` /
  `hover` / `goto_waypoint` / `set_gimbal_pitch` / `set_yaw` as verbs inside
  `POST /action`, not as the dedicated routes §6 describes. The quadruped
  bridge likewise has no `/set_pose` or `/wave` route — poses go through
  `/tool` or `/prompt`.
- **`/read_camera` (§6) and `/read_mission_brief` (§6) do not exist.** The
  mavic bridge serves `/image`; the husky bridge serves `/camera` and
  `/mission`. §13.5's claim that `/image` *and* `/read_camera` both work is
  **false** — only `/image` does.
- **`/read_sensor` + `/list_sensors` (§6.6) exist on `omnilink_mobile_bridge`
  only** (added 2026-08-17). Every other bridge still has no sensor
  read-through, so a client MUST check `capabilities.sensors` rather than
  assume the verb is there.
- **`/list_robots`, `/get_robot_state`, `/stop_robot`, `/reset_to_home` and
  `/prompt` are missing on `husky_omnilink_bridge` and
  `mavic_omnilink_bridge`.** They are present on the arm, mobile
  (`omnilink_mobile_bridge`) and quadruped bridges. Those two older bridges
  route everything through `POST /action` instead.
- **`POST /action` is implemented by `husky_omnilink_bridge` and
  `mavic_omnilink_bridge`, but not by the arm bridge** (§5.4), which uses the
  Axis verb surface only.
- **The arm bridge's `/stop_robot` and `/reset_to_home` responses are
  pre-v1.0** — they omit the `ok`/`accepted`/`eta_s` envelope (§5.5/§5.6).
- **`/robot/<def>/sensor/<name>` returns 501 with a prose `error` string**,
  not the `effector_unavailable` fault code §7 specifies.
- **Most robot/physics-specific fault codes (§11) remain reserved rather than
  observed in every bridge.** The shared HTTP codes are emitted by canonical
  bridges; `unreachable_target` and `effector_unavailable` are the main
  robot-specific codes emitted by the arm bridge today.
- **`docs/protocol/schemas/` does not exist yet** (§15) — the machine-readable
  JSON Schemas are a v1.1 deliverable.

---

## 17. Hardware-in-the-loop (MAVLink)

**This is the one OmniSim surface that is not HTTP and not JSON**, which is why
it is a top-level section rather than a bridge capability under §6: §2's
transport and §3's envelope do not apply to any of it.

OmniSim speaks the **MAVLink v2 HIL** protocol over **UDP**, the same protocol
PX4 and ArduPilot expose for hardware-in-the-loop. It is deliberately not an
OmniSim invention: the value of this surface is that a real autopilot -- SITL,
or a flight controller on a bench -- can be substituted for the reference
implementation without either side changing.

### 17.1 Direction and roles

OmniSim is the **simulator**, never the autopilot. It sends state and sensors;
it receives actuator commands. An implementation that generates its own control
outputs is not using this surface.

| direction | message | rate |
|---|---|---|
| sim to autopilot | `HIL_SENSOR` (107) | every basic timestep |
| sim to autopilot | `HIL_STATE_QUATERNION` (115) | every basic timestep |
| sim to autopilot | `HIL_GPS` (113) | 10 Hz |
| sim to autopilot | `HEARTBEAT` (0) | 1 Hz |
| autopilot to sim | `HIL_ACTUATOR_CONTROLS` (93) | autopilot's own rate |

`HIL_ACTUATOR_CONTROLS.controls` is a 16-float array. The fixed-wing mapping is
`[0]` aileron, `[1]` elevator, `[2]` rudder in `[-1, 1]`, and `[3]` throttle in
`[0, 1]`. Indices 4-15 are unassigned; a receiver MUST ignore them rather than
reject the message.

### 17.2 Frames — the part that is easy to get wrong

OmniSim is **ENU / FLU**; MAVLink is **NED / FRD**. These are TWO different
conversions and they are not the same matrix:

* world, ENU to NED: swap east and north, negate up.
* body, FLU to FRD: keep forward, negate left and up.

Both are involutions, so an implementation that uses one for both passes a
round-trip test and still reports an aircraft flying east as heading north, with
forward acceleration appearing as lateral. Conversions MUST be tested against
named physical cases, not only against the involution property.

### 17.3 Timing

The sim clock is authoritative: `time_usec` on every message is SIMULATED time.

A hardware rig also needs wall-clock pacing, and that is a separate, measured
constraint rather than a protocol guarantee. Launch the engine with
`--mode=realtime`; `--batch` and `--no-rendering` do not affect pacing. Measured
on one Windows machine, real-time mode cannot pace a step below the ~15.6 ms OS
timer quantum, so `basicTimeStep 8` runs at 0.516x while `basicTimeStep 20`
holds 1.0005x. Keep `basicTimeStep` an **integer**: it is truncated into
`QTimer::start(int)`, so a fractional value paces faster than real time
permanently, silently, and at exit code 0.

Under `--mode=fast` the same link is software-in-the-loop rather than
hardware-in-the-loop: correct, much faster than real time, and not valid for
anything whose timing is under test.

### 17.4 Sensor honesty

A sensor value that has not been measured MUST NOT be sent as zero. A zeroed
`HIL_SENSOR` reads to an autopilot as a level, motionless aircraft -- a
measurement nobody made, and the most dangerous possible default. Withhold the
message until real data exists, or mark the field unavailable by the MAVLink
convention (`-1` in the corresponding covariance) where one exists.

### 17.5 Ports

`14560` is the default HIL port, chosen to sit clear of the conventional
`14550` ground-station port so a GCS and a HIL link can coexist. §12's
multi-instance rules apply: give each parallel simulator its own port.

### 17.6 Reference implementation

`packages/omnisim-hil/` -- the codec (`omnisim_hil/mavlink.py`, stdlib only),
the simulator-side controller (`controllers/hil_aircraft/`), and a reference
autopilot. See its README for what is measured and what is not.

---

*This document is part of the OmniSim public surface. Update it in the
same change as the underlying wire change.*
