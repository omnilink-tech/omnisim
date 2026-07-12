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
9. [Twin Shadow](#9-twin-shadow)
10. [Event taxonomy](#10-event-taxonomy)
11. [Fault codes](#11-fault-codes)
12. [Multi-instance and port allocation](#12-multi-instance-and-port-allocation)
13. [Compatibility and version negotiation](#13-compatibility-and-version-negotiation)
14. [Stability commitment](#14-stability-commitment)
15. [Out of scope](#15-out-of-scope)
16. [Reference implementations](#16-reference-implementations)

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
- **Default host:** `127.0.0.1` (loopback). Public binding is out of
  scope for v1.0 and any deployment doing it MUST add its own auth
  layer.
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

- MUST cache the request id for at least 5 seconds.
- MUST return the same response for a repeated id within that window.
- MUST NOT re-apply the underlying mutation.

This lets agent loops retry network errors without commanding the
robot twice. Implementations that do not yet cache request ids MUST
still accept the field (and ignore it) so callers can send it
uniformly across bridge versions.

---

## 4. Version negotiation

### 4.1 GET /protocol — PLANNED (will be required on every surface)

> **Status: not yet implemented.** No in-tree service exposes
> `GET /protocol` today (the harness, the capture service, and every
> reference bridge lack the route). It is the v1.0 *target* — once
> implemented it will be required on every surface — but it is currently
> the first open compliance gap (see [§16](#16-reference-implementations)).
> The shape below is normative for that target; the MUST applies once
> the endpoint ships.

The target: every service implementing this protocol exposes `GET /protocol`:

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
    "world": "projects/samples/demos/worlds/chat/omnilink_husky.wbt",
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

> **Status: not yet emitted.** No in-tree service emits these headers
> today (a grep over every bridge, the harness, and the capture service
> finds zero). They are the v1.0 *target*; the MUST below applies to a
> v1.0-compliant service once headers ship, not to the current in-tree
> implementations.

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

### 5.1 GET /protocol (PLANNED)

See §4.1 (not yet implemented by any bridge). `service = "robot_bridge"`.
When shipped, the `instance.robot_id` MUST be
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
this" without parsing free-text. Long-running actions return immediately
on acceptance and report progress via subsequent `/state` polls.

**Response (400):** `error = "invalid_action"`, `"missing_field"`, or
`"value_out_of_range"`.

**Response (409):** `error = "joint_limit"`, `"unreachable_target"`,
`"ik_nonconvergent"`, etc. — the action was understood but cannot be
performed.

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
- `POST /set_tcp_target { "xyz": [x, y, z] }` (requires IK)
- `POST /solve_ik { "xyz": [x, y, z] }` (no motion, returns `{q, err_norm, iters}`)
- `POST /read_joints` → `{ "q": [N floats], "commanded_q": [N floats], "joint_names": [...] }`
- `POST /read_tcp_pose` → `{ "tcp_world": [x,y,z], "tcp_arm_local": [x,y,z], "arm_origin_world": [x,y,z] }`

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

- `POST /read_sensor { "sensor": "<name>" }` →
  `{ "available": false, "note": "OmniSim restricts device APIs to the owning controller" }`
  on bridges that don't proxy sensors; this is the structurally correct
  v1.0 answer. Bridges that do proxy sensors return
  `{ "available": true, "value": <...>, "unit": "..." }`.

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

### 7.1 GET /protocol (PLANNED)

See §4.1 (not yet implemented by the harness). `service = "world_harness"`.
When shipped, `instance.world` is the path of the most recently loaded
world or `null` if none has been loaded.

### 7.2 GET /healthz

Liveness probe; does not touch the simulator subprocess.

**Response:** `{ "ok": true, "uptime_s": float }`.

### 7.3 POST /world/load

**Request:**

```json
{ "path": "projects/samples/demos/worlds/showcase/warehouse_husky.wbt",
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

Hot reload is the default behaviour of repeated `/world/load` calls —
the same supervisor is reused, ~600 ms turnaround.

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
{ "ok": true,
  "mean_brightness": 0.42, "mean_rgb": [0.4, 0.42, 0.45],
  "max_rgb": [1.0, 1.0, 1.0],
  "saturated_pct": 0.8, "black_pct": 12.3,
  "warnings": ["underexposed: 12.3% near-black"] }
```

### 7.7 GET /scene/tree

Flat node list; each entry has `def`, `type`, `position`,
`orientation`, `parent_def`, `is_robot`.

### 7.8 GET /scene/node/<def>

Field dump + contact points for one node, identified by DEF name.

### 7.9 POST /scene/look_at

**Request:** `{ "position": [x,y,z], "target": [x,y,z], "push": true }`.

**Response:** computed axis-angle orientation;`push: true` (default)
applies it to the live `Viewpoint` immediately.

### 7.10 POST /sim/step

`{ "steps": int }` — advance N basic timesteps. Returns
`{ "ok": true, "sim_time": float }`.

### 7.11 POST /sim/reset

Reset world to `t=0` without re-parsing.

### 7.12 GET /sim/state

`{ "world": "...", "running": bool, "sim_time": float, "paused": bool, "last_load": { ... } }`.

`/sim/state` reports metadata about the harness session — **not scene
state**. For scene state, use `/robots`, `/robot/<def>/joints`,
`/sim/contacts`.

### 7.13 GET /robots

Enumerate every `Robot` in the current scene with pose and joint count.

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

### 7.17 GET /sim/contacts

`{ "ok": true, "contacts": [ { "a_def": "...", "b_def": "...", "point": [x,y,z] } ] }`.

### 7.18 GET /sim/grips

`{ "ok": true, "grips": [ { "gripper_def": "...", "held_def": "...", "since_t_ms": int } ] }`.

### 7.19 GET /sim/events

The unified runtime event stream — supervisor-side and harness-side
events merged into a single cursor-paged response.

**Query parameters:**

- `since` — supervisor-side cursor (default `0`).
- `log_since` — harness-side log cursor (default `0`).
- `limit` — max events to return (default `100`).
- `types` — comma-separated allowlist; if absent, all event types are
  returned.

**Response:**

```json
{
  "ok": true,
  "events": [
    { "type": "contact.began", "t": 1.4, "a_def": "HUSKY", "b_def": "WALL_03",
      "point": [1.0, 2.0, 0.05] }
  ],
  "next_since": 42,
  "next_log_since": 17,
  "dropped_sup": 0,
  "dropped_log": 0
}
```

Event taxonomy: see §10. `dropped_sup`/`dropped_log` going non-zero
means the caller is polling slower than events arrive; raise `limit`
or poll more often.

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
event streams (a v1.1 addition). Each event MUST have:

- `type` — namespaced dotted name from the list below.
- `t` — `sim_time` at which the event occurred (float seconds).

Type families and the required fields per type are:

### 10.1 Contact

- `contact.began { a_def, b_def, point: [x,y,z], normal_force?: float }`
- `contact.ended { a_def, b_def }`

### 10.2 Grip (gripper attachment, inferred or explicit)

- `grip.began { gripper_def, held_def }`
- `grip.released { gripper_def, held_def }`

### 10.3 Joints

- `joint.limit_hit { robot_def, joint, direction: "lower"|"upper" }`

### 10.4 Damage

- `damage.applied { target_def, hp_before, hp_after, source_def?, impulse_j?: float }`
- `damage.state_changed { target_def, state: "ok"|"degraded"|"broken" }`
- `damage.part_detached { target_def, part_def }`

(See [§13.7 of docs/developer/engine-migration-plan.md](docs/developer/engine-migration-plan.md)
for the full damage event design.)

### 10.5 Controller / log

- `controller.log { robot_def, stream: "stdout"|"stderr", text }`

### 10.6 World / harness

- `world.warning { code?, message }`
- `world.error { code?, message }`

### 10.7 Faults (bridge-emitted)

- `fault.raised { robot_id, code, message }` — see §11.
- `fault.cleared { robot_id, code }`

### 10.8 Twin

- `shadow.stale { robot_id, last_t_real_s }`
- `shadow.resumed { robot_id }`

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
| `value_out_of_range` | A field's value is outside its documented range. |
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

> **Note:** this handshake depends on `GET /protocol`, which is not yet
> implemented by any in-tree service (§4.1). Until it ships, step 2
> returns `404` and clients fall through to the best-effort path
> described at the end of this section.

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

§9 (Twin Shadow) is **excluded** from this commitment: it is a reserved
design with no implementation, and its shapes may change before it ships.

The `GET /protocol` shape (§4.1) is **planned, not yet stable** — no
in-tree service implements it. When it ships its shape will be stable
under the same `1.x` rules; until then it cannot be relied on (see §16).

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

- **Authentication.** Loopback-only is the v1.0 trust boundary.
  Internet-exposed deployments MUST wrap services in a reverse proxy
  with their own auth.
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
| Robot Bridge — quadruped | [projects/samples/demos/controllers/omnilink_quadruped_bridge/](projects/samples/demos/controllers/omnilink_quadruped_bridge/) | Port `8765`. Spot. Motions: `stand`, `sit`, `crouch`, `settle`, `wave`, `walk`, `stop`, `home` — driven through `/tool` / `/prompt`, not dedicated routes. ⚠️ Its `walk` is a **wave-gait leg cycle with supervisor-driven body translation**, i.e. scripted, not physically-actuated locomotion. Learned Spot locomotion lives in `projects/policies/`, not in this bridge. |
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
- **`GET /protocol` is implemented by no service** — not the harness,
  not the capture service, not any bridge (§4.1).
- **No service emits the `X-OmniSim-Wire` / `X-OmniSim-Service`
  response headers** (§4.3).
- **The §3.2 error envelope is not implemented by the harness.** Harness
  errors are `{"error": "<free text>"}` — no `ok: false`, no snake_case code,
  no `message`/`details` split. Bridges are closer but not uniform.
- **`/world/load` returns 422 on a load failure** (not the 400/409/500 in
  §7.3's original text; the §7.3 example is now correct).
- **Per-class endpoints (§6) are largely not exposed as endpoints.** The
  flying bridge (`mavic_omnilink_bridge`) implements `takeoff` / `land` /
  `hover` / `goto_waypoint` / `set_gimbal_pitch` / `set_yaw` as verbs inside
  `POST /action`, not as the dedicated routes §6 describes. The quadruped
  bridge likewise has no `/set_pose` or `/wave` route — poses go through
  `/tool` or `/prompt`.
- **`/read_camera` (§6), `/read_sensor` (§5), and `/read_mission_brief` (§6)
  do not exist.** The mavic bridge serves `/image`; the husky bridge serves
  `/camera` and `/mission`. §13.5's claim that `/image` *and* `/read_camera`
  both work is **false** — only `/image` does.
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
- **Of the 17 reserved fault codes (§11), only two are emitted** in practice:
  `unreachable_target` and `effector_unavailable` (both from the arm bridge).
  Treat §11 as a reserved namespace, not as a set you can match on today.
- **`docs/protocol/schemas/` does not exist yet** (§15) — the machine-readable
  JSON Schemas are a v1.1 deliverable.

---

*This document is part of the OmniSim public surface. Update it in the
same change as the underlying wire change.*
