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
> docs/developer/agent-native-api.md (archived 2026-09-02, see [docs/ARCHIVE.md](docs/ARCHIVE.md))
> rather than describing the aspiration. Read §16 before depending on any
> harness endpoint. The structural fix — serving these lists from the
> code — is proposal P1 in that document.

> **Re-reconciled against the bridge code on 2026-09-21.** §5 had drifted
> the same way: `POST /tool` — the platform's tool callback, implemented by
> `bridge_base` and by all four in-tree controller bridges, and the path a
> model's command actually arrives on — was undocumented, appearing only
> incidentally in §5.4.1 and §16. It is now §5.8. The safety gate it runs
> was likewise invisible on the wire: `refused_by_gate` was an undocumented
> fault code (now §11 + §11.1), `/prompt`'s `actions[].result` had no value
> that could express a refusal (now §5.7.2, a **new** addition), and
> `/capabilities` had no way to declare whether a bridge gates anything at
> all (now §5.2.1, **new and unimplemented** — ⚠️ *implemented since
> 2026-09-22; see the next note*). ⚠️ The same pass records what
> is *not* gated, because a spec that documents the checks and not their
> boundary reads as a guarantee it cannot make: the direct REST actuators in
> §5.4, §5.5, §5.6 and §6 are ungated in every bridge and in `bridge_base`.
> The audit is
> [`packages/omnisim-bridges/GATE_COVERAGE.md`](packages/omnisim-bridges/GATE_COVERAGE.md).
> All of it is additive under `1.1`.

> **Re-reconciled against the bridge code on 2026-09-22 (v9).** Three of the
> notes above have stopped being true, and a spec that keeps an "unimplemented"
> banner on a shipped feature misleads exactly as badly as one that claims an
> unshipped one. **`safety_gate` (§5.2.1) is served by all five reference
> implementations**; `POST /prompt` **always** carries `via` on a 200 and
> **every** refusal carries `rule` (§5.7.1, §5.7.2); `last_tick_at` is SIM
> seconds on all five and the wall clock moved to `wall_time` (§5.3); measured
> results carry `sim_time_start` / `sim_time_end` / `steps`, and a **stall is
> reported as `stalled`, not as `timed_out`** (§5.4.1); the bridge serves its
> own cursor-paged event stream at **`GET /events`** (§5.9). ⚠️ That stream is
> **not** the harness's `/sim/events` (§7.19): two rings, two cursors, and the
> bridge's dies with the bridge process. What has **not** changed: the direct
> REST actuators are still ungated (§5.4–§5.6, §6, §16), event detection is
> still never sub-step, and the two break-latency regimes of §7.40 are
> untouched. All of it is additive under `1.1`.

| Field | Value |
|---|---|
| Protocol name | `omnisim_wire` |
| Version | `1.1` |
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

**Changelog:** `1.1` (2026-07-26, additive; harness commit `5f994bf99`) — world-harness capability discovery (`GET /capabilities`, §7.28), the scene-mutation verbs (`/scene/spawn`, `/scene/delete`, `/scene/set_pose`, §§7.29–7.31), named snapshots (`/sim/snapshot` / `/sim/restore` / `GET /sim/snapshots`, §7.32), and `/sim/reset` restoring the authored scene; later harness routes (§§7.33–7.36) are additive under `1.1`. The 2026-09-21 robot-bridge additions are also additive under `1.1`: `POST /tool` (§5.8, documenting a shipped endpoint), the `safety_gate` capability block (§5.2.1, new and unimplemented **on that date; implemented 2026-09-22**), `actions[].result: "refused"` with its `rule` field (§5.7.2, new), and the `refused_by_gate` / `tool_not_registered` / `tool_execution_failed` fault codes (§11). The 2026-09-22 (v9) robot-bridge additions are additive under `1.1` as well: `GET /events` (§5.9, new), the `events` and `lockstep` capability blocks (§5.2), `safety_gate` gaining `vertical_rail` and becoming **implemented** (§5.2.1), `wall_time` / `step` / `held` / `events` on `GET /state` (§5.3), `sim_time_start` / `sim_time_end` / `steps` / `stalled` on a measured result (§5.4.1), the guaranteed top-level `via` on `/prompt` (§5.7.1), `rule` on every refusal (§5.7.2), `details.rule` on a `400 refused_by_gate` body (§11.1), and `surface` as an ignored transport field on `/tool` (§5.8.1). `1.0` (2026-07-26) — the initial reconciled specification. The harness serves `"omnisim_wire": "1.1"`; the shipped robot bridges still implement `1.0` (see §4.1).

---

## Table of contents

1. [Surfaces](#1-surfaces)
2. [Transport and encoding](#2-transport-and-encoding)
3. [Common envelope and error model](#3-common-envelope-and-error-model)
4. [Version negotiation](#4-version-negotiation)
5. [Robot Bridge — required endpoints](#5-robot-bridge--required-endpoints)
   - 5.9 [GET /events — the bridge's own event stream](#59-get-events--the-bridges-own-event-stream) *(v9; **not** the harness's §7.19 stream)*
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
  implements. SemVer string. ⚠️ Per-service, so it may lag the spec
  version at the top of this document: the example above shows `"1.0"`
  because the shipped robot bridges genuinely implement `1.0`
  (`WIRE_VERSION` in
  `packages/omnisim-bridges/src/omnisim_bridges/http_security.py` —
  every `1.1` addition is a World-Harness route), while the world
  harness reports `"omnisim_wire": "1.1"` via `GET /capabilities`
  (§7.28), since it does not serve `GET /protocol` yet.
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
- `safety_gate` — gate discovery. **Implemented (v9): all five reference
  implementations publish it.** See §5.2.1.
- `events` — event-stream discovery: `{endpoint, state_field, types[]}`,
  so a client finds `GET /events` (§5.9) instead of 404-probing for it.
  Published by the four in-tree controller bridges. ⚠️ The `bridge_base`
  reference attaches `safety_gate` to every `/capabilities` response
  automatically but does **not** synthesise `events`; a bridge built on it
  declares that block itself or not at all, and its absence is not
  evidence that the endpoint is missing — ask `GET /events`.
- `lockstep` — `{supported: bool, why?}`. ⚠️ **Only the flying bridge
  publishes it, and it publishes `supported: false`** (a held drone is a
  drone whose rotors stop being commanded). The mobile, arm and quadruped
  bridges honour the `OMNISIM_BRIDGE_LOCKSTEP` hold and publish **no**
  `lockstep` block at all, so absence here means "not declared", never
  "not supported". The wire signal for a hold that is live *right now* is
  `GET /state.held` (§5.3).

`POST /list_robots` MUST return the same body and is provided for
clients that prefer a POST-only verb surface (the Axis convention).

### 5.2.1 `safety_gate` — declaring whether commands are vetted

> **Status: IMPLEMENTED (v9, 2026-09-22).** The banner this section carried
> for one day — "NEW, and unimplemented" — is retired. All five reference
> implementations now serve the block: `bridge_base` (which attaches it to
> every `/capabilities` response through one shared builder) and the mobile,
> arm, quadruped and flying controller bridges. It exists because there was
> **no way for an external client to tell a gated bridge from an ungated one
> over the wire**, and the two behave differently on the same request: a
> gated route answers `400 refused_by_gate` (§11) where an ungated one
> actuates. A client that cannot discover this cannot decide whether it is
> itself responsible for bounding a command.
>
> ⚠️ **Where to read it differs by bridge, because `/capabilities` itself
> does.** The mobile, arm and quadruped bridges and `bridge_base` answer the
> array form `[{id, model, capabilities: {…}}]`, so the block is at
> `[0].capabilities.safety_gate`; the flying bridge answers a flat object
> and puts it at the top level. Look in both places rather than concluding a
> bridge is ungated.

A bridge that vets commands before dispatch SHOULD publish a
`safety_gate` object in `/capabilities`:

```json
{
  "safety_gate": {
    "present": true,
    "implementation": "omnisim_bridges.gate",
    "surface": "mobile",
    "fails_closed": true,
    "gated_paths": ["/prompt", "/tool"],
    "ungated_paths": ["/drive_forward", "/turn", "/drive_to",
                      "/set_velocity", "/stop_robot", "/reset_to_home"],
    "rules": ["interrogative", "prohibition", "self_negating",
              "reported_speech", "retracted", "deferred", "contradiction",
              "invented_magnitude", "sign_conflict", "implausible",
              "unknown_arg", "missing_arg", "bad_type", "not_finite",
              "unresolved_referent", "needs_authorization"],
    "rails": {
      "max_distance_m": 50.0, "max_angle_rad": 25.132741,
      "max_speed_mps": 5.0, "max_yaw_rate_rps": 6.0,
      "max_altitude_m": 120.0, "max_body_shift_m": 1.0
    },
    "vertical_rail": "max_body_shift_m"
  }
}
```

Field rules:

- `present` (bool, REQUIRED when the block is present) — whether any
  vetting runs at all.
- `implementation` (string, OPTIONAL) — a stable identifier for the
  checker, so a client can recognise two bridges that share one. It is
  **not** a version guarantee.
- `surface` (string, OPTIONAL) — the robot class the bridge declares
  itself to be (`mobile`, `arm`, `quadruped`, `drone`). It selects the
  rail where two classes share a verb; see §5.8.3.
- `fails_closed` (bool, OPTIONAL) — `true` iff a *physical* command is
  refused when the checker itself is unavailable or raises. A bridge
  MUST NOT report `true` unless that is what it does.
- `gated_paths` / `ungated_paths` (arrays of route strings) — which of
  this bridge's own routes the gate is on and which it is not. A bridge
  publishing `present: true` **SHOULD** publish both. `ungated_paths` is the load-bearing half: a bridge
  that publishes `present: true` while omitting `ungated_paths` invites
  the reading "everything here is vetted", which is false of every bridge
  in this tree (§5.4, §5.5, §5.6, §6, and §16's compliance gaps).
  ⚠️ **These are ROUTES.** A bridge whose actuation surface is one route
  taking a verb in the body publishes the route, never the verb: the
  flying bridge published `["/reset", "/complete_mission"]` here for a
  day, and `POST /reset` on it has always been a 404, so neither string
  told a client anything about any path it could call.
  ⚠️ **A PARTIALLY vetted route appears in BOTH arrays**, with the split
  named in `safety_gate_note`. A route that gates some of its verbs and
  passes the rest through unchanged is not a gated route, and the
  protocol has no per-verb field: publishing it as gated only would tell
  a client it need not bound the verbs nobody checks, and dropping it
  from `gated_paths` would understate the refusals it does issue. Both
  halves is the fail-safe shape, because a client that reads only the
  load-bearing half is told to bound the route. ⛔ An **empty**
  `ungated_paths` on a bridge that has any unvetted actuation is the one
  answer that must never be published — it is indistinguishable from
  "everything here is vetted".
- `rules` (array of strings, OPTIONAL) — the rule names this gate can
  emit, from the enumeration in §5.8.3. It is an OPEN set; a client MUST
  tolerate an unknown name.
- `rails` (object, OPTIONAL) — the per-quantity magnitude rails in SI
  units. ⚠️ These are **rails, not bounds**: they express "nobody meant
  this", not "the robot can do this". Nothing in the stack limits a robot
  to the floor it is standing on, and a client MUST NOT read a rail as a
  workspace limit. The reachable envelope is `workspace` (§5.2), and the
  yaw envelope is `max_angular_rad_s` (§6.2) — both separate fields, both
  measured, both meaning something else.
- `vertical_rail` (string **or `null`**, OPTIONAL) — which key of `rails`
  this bridge's `move_body{vertical}` is judged against:
  `"max_altitude_m"` on a drone, `"max_body_shift_m"` otherwise. It is
  published so a client can check that the bridge picked the rail it
  expected. ⚠️ **`null` when the bridge declares no `surface`, and `null`
  is not "the strictest rail"** — see the selection rule below. A bridge
  that cannot say which rail it will get publishes `null` rather than a
  guess, because a guess here is a safety property nobody checked.
- `safety_gate_note` (string, OPTIONAL, **outside** the block) — prose for
  a caveat the fields cannot carry, and the **required** companion of a
  route published in both arrays. The flying bridge uses it to name, for
  `POST /action`, which verbs are vetted (`takeoff`, `land`, `hover`,
  `set_yaw`, `stop`, `reset` — mapped onto the gate's vocabulary) and
  which are not (`goto_waypoint`, `set_gimbal_pitch`, `complete_mission`
  — no mapping, passed through unchanged), so a client can discover that
  `goto_waypoint` flies the aircraft to a coordinate with no rail on it.
  ⚠️ This bullet said until 2026-09-22 that `/action` appears in
  `gated_paths` "not in both halves", and the bridge's own lists said
  something different again. A note MUST NOT be the only place an
  unvetted path is discoverable — it is prose, and a client branches on
  the arrays.

#### How the rail is selected — the caller's surface wins

This is the single easiest thing in the gate to get wrong, and getting it
wrong in the "safe" direction is a live false-refusal bug, not a
conservative default. The rule, verbatim from `gate.check`:

1. **The caller's `surface` always wins.** A bridge that knows its own
   class MUST pass it: the fallback is a guess and this argument is a fact.
2. With no caller surface, the gate falls back to the surface **recorded
   on the tool's spec** — but only when **exactly one** robot class ever
   registered that tool.
3. When **two or more** classes registered it — a genuinely shared tool —
   the recorded surface is discarded and the tool takes the **ground /
   body-shift** rail (1.0 m). Guessing air would raise a 1 m limit to
   120 m; guessing ground is wrong only in the direction that refuses a
   legitimate climb, which is visible and is fixed by passing `surface`.
4. `takeoff` / `land` / `hover` are air **by name**, whatever the caller
   claims.

⚠️ **"No surface means the strictest rail" is therefore FALSE as a blanket
rule.** It is true of a *shared* tool and false of every other. Measured on
the shipped tree, where only the flying bridge registers `move_body`:
`move_body{vertical: 119}` with **no** surface is **allowed**, on the 120 m
air rail. A client — or a port of this gate — that hard-codes "null → 1.0 m"
will pass the parity fixture and be wrong the day one process serves the
tool alone. The fixture that pins all of this is
[`tests/benchmarks/gate_parity/cases.json`](tests/benchmarks/gate_parity/cases.json);
its `registry` key publishes the registration table a port must replay, and
the generator refuses to write unless all 24 registration orders agree.

#### ⚠️ A `surface` sent over the wire is accepted and IGNORED

`POST /tool` (§5.8.1) accepts a top-level `surface` field, strips it from
the argument set, and **throws the value away**. The bridge's own surface
is authoritative.

Both halves are deliberate. It is **stripped** because the field is
otherwise an argument, and the gate would answer
`400 unknown_arg: surface is not a parameter of drive_forward` — refusing
every tool call against every shipped bridge. It is **dropped rather than
honoured** because the rail a shared verb is judged against is picked by
the surface: a caller who could declare `"drone"` would buy the 120 m
altitude rail for a quadruped's `move_body{vertical}`, turning a safety
property into a client-chosen setting — the party the rail protects against
choosing the rail. This is a security property of the endpoint, not an
oversight, and it MUST NOT be "fixed" by threading the caller's value
through. A client that wants a different rail declares a different
`surface` **on the bridge**, not in the request.

**Absence of the field means "not declared", never "not gated"**, and the
converse holds too: `present: true` with an empty or absent `ungated_paths`
must never be read as "everything here is vetted". Every bridge in this
tree has ungated routes, and they are listed per bridge below.

#### What each reference implementation publishes

Read as: these are the route tables the shipped bridges declare, and
`ungated_paths` is exact for each.

| bridge | `surface` | `gated_paths` | `ungated_paths` |
|---|---|---|---|
| `omnilink_mobile_bridge` | `mobile` | `/prompt`, `/tool` | `/set_velocity`, `/drive_forward`, `/turn`, `/drive_to`, `/drive_to_waypoint`, `/stop_robot`, `/reset_to_home`, `/attach_trolley`, `/grab_pallet`, `/detach_trolley`, `/release_pallet`, `/resume_autonomy`, `/resume_idle_loop`, `/intents` |
| `omnilink_arm_bridge` | `arm` | `/prompt`, `/tool` | `/set_joint_positions`, `/set_tcp_target`, `/solve_ik`, `/open_gripper`, `/close_gripper`, `/set_gripper_width`, `/grasp`, `/release`, `/stop_robot`, `/reset_to_home`, `/pick`, `/place` |
| `omnilink_quadruped_bridge` | `quadruped` | `/prompt`, `/tool` | `/stop_robot`, `/stand`, `/sit`, `/walk`, `/wave`, `/reset_to_home` |
| `mavic_omnilink_bridge` | `drone` | `/prompt`, `/tool`, `/action` | `/action` — ⚠️ **deliberately in both halves**: it is PARTIALLY vetted, and `safety_gate_note` names the split (vetted `takeoff`, `land`, `hover`, `set_yaw`, `stop`, `reset`; **unvetted** `goto_waypoint`, `set_gimbal_pitch`, `complete_mission`) |
| `bridge_base` reference handler | `null` unless the subclass sets one | `/prompt`, `/tool` | the twelve direct verbs it serves |

⚠️ The arm's **learned verbs** are promoted to `POST /<verb>` routes at
runtime and therefore cannot appear in a static `ungated_paths` list. They
are ungated. `GATE_COVERAGE.md` names them; this field does not.

⚠️ **The flying bridge's row was wrong in both directions until
2026-09-22**, and it is worth stating because the failure mode is the
generic one. Its `ungated_paths` read `["/reset", "/complete_mission"]`:
two `action` VERBS published as routes (neither is a path — `POST /reset`
is a 404); `/reset` described as unvetted although it maps to
`reset_to_home` and goes through the gate; and the verbs that genuinely
are unvetted — `goto_waypoint` above all — named nowhere at all. The lists
are now DERIVED from the router's own verb inventory and its
verb→gate-tool map, and
[`packages/omnisim-bridges/tests/test_mavic_gate_publication.py`](packages/omnisim-bridges/tests/test_mavic_gate_publication.py)
fails the moment a verb is added to the router without reaching the
publication. A hand-maintained safety claim beside the code it describes
is a safety claim that will drift.

### 5.3 GET /state — also POST /get_robot_state

**Request:** no body.

**Response (200) — class-agnostic envelope:**

```json
{
  "ok": true,
  "robot_id": "arm_1",
  "sim_time": 12.480,
  "last_tick_at": 12.448,
  "wall_time": 1769472011.4,
  "step": 390,
  "held": false,
  "events": { "total": 12, "last": { "...": "..." }, "next_since": 12, "dropped": 0 },
  "mode": "idle",
  "fault": null,
  "state_source": "sim"
}
```

#### ⚠️ The clocks: `last_tick_at` is SIM seconds, and it used not to be

`sim_time` and `last_tick_at` are both **sim seconds**, as the example
above has always shown. Until 2026-09-22 every shipped bridge sent
`time.time()` in `last_tick_at` — about 1.7e9 where this section's own
example reads `12.448` — so a client differencing the two fields got the
age of the Unix epoch and any wait built on that difference was wrong. All
five reference implementations now report sim seconds.

The wall clock did not disappear; it moved to its own field:

| field | clock | use |
|---|---|---|
| `sim_time` | SIM seconds | the physics clock; the ruler every measurement in §5.4.1 is quoted on |
| `last_tick_at` | SIM seconds | the sim time of the bridge's last tick |
| `wall_time` | Unix epoch seconds (float) | liveness only — "is the sim thread still stepping?" |
| `step` | int | basic steps this bridge has ticked, from 0 |

⚠️ **Never difference a sim field against `wall_time`.** They are different
rulers, exactly as `sim_time_ms` and `engine_time_ms` are on the harness
(§7.12). A wall-clock timeout silently truncates a motion whenever the sim
runs below realtime, which on a loaded world is most of the time; that is
why §5.4.1's waits are counted in steps.

Two more class-agnostic fields, both v9 and both optional:

- `held` (bool) — a lockstep hold (`OMNISIM_BRIDGE_LOCKSTEP=1`, default
  **off**) is freezing the world *right now*. The mobile, arm and quadruped
  bridges report the live value; the flying bridge reports a constant
  `false` and declares `lockstep: {supported: false}` in `/capabilities`
  (§5.2), because a held drone is a drone whose rotors stop being
  commanded. The hold is **leased** with the same bounds as the harness's
  pause (§7.38) — 30 s default, `[1 s, 300 s]`,
  `OMNISIM_BRIDGE_HOLD_LEASE_MS` — so a dead client cannot freeze a demo
  forever; expiry resumes the world and files `hold.expired` (§5.9).
  `stop_robot` always runs. ⚠️ While held, the loop pumps at most one step
  per second **if the robot window is open**, because the window's text
  channel needs a step to drain. Any such step is world motion during a
  nominally frozen run, and the count rides on the `hold.expired` event's
  detail as `leak_steps` — ⚠️ **no bridge publishes it on `/state`**, so a
  client that must know cannot learn it until the lease expires.
- `events` (object) — the `GET /events` summary, `{total, last, next_since,
  dropped}`, so a reader learns whether it has fallen behind without a
  second request. See §5.9.

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

⚠️ `state_source` remains **unreported by every in-tree bridge** (§16); it
is specified, not shipped. The v9 fields above are shipped.

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

**Response (400):** `error = "refused_by_gate"` — **only where a bridge
gates this route.** See the note below, and §11.

> ⚠️ **`/action` is not a vetted path, except on the Mavic.** Audited
> 2026-09-21 ([`packages/omnisim-bridges/GATE_COVERAGE.md`](packages/omnisim-bridges/GATE_COVERAGE.md)):
> the flying bridge maps `takeoff` / `land` / `hover` / `set_yaw` / `stop` /
> `reset` onto gate verbs and refuses with `400 refused_by_gate`
> (`details.action` carries the action name; since 2026-09-22 `details.tool`
> and `details.rule` ride beside it) — and its other three verbs,
> **`goto_waypoint`, `set_gimbal_pitch` and `complete_mission`, have no
> mapping and are passed through unchanged** rather than refused, because
> refusing what the bridge has always accepted would be a behaviour change
> rather than a safety fix. That is why `/action` appears in **both**
> halves of its `safety_gate` block (§5.2.1) — `goto_waypoint` flies the
> aircraft to a coordinate with nothing checking the coordinate.
> **`husky_omnilink_bridge`'s `/action` is
> ungated.** A client MUST NOT assume a `/action` dispatch is bounded by
> anything; the magnitude it sends is the magnitude the actuator gets.

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
   `seq` the caller can wait on. Because a process restart resets a local
   counter, a bridge that uses `seq` **SHOULD** also return an immutable
   process-incarnation identifier; callers correlate the tuple
   `(bridge_instance_id, seq)`, never `seq` alone. A wait primitive gated only on a mode string
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
   (That −43% is a historical worked example: it was measured on the Husky's
   open-loop `turn` before `69b4b024b` (2026-09-11) lifted the wheel
   stall-torque cap, and has not been re-measured through the bridge since.
   The rule is about the *description*, and is unaffected by which number is
   current.)
7. ⭐ **A measured result SHOULD carry the SIM WINDOW it was measured over**
   (v9): `sim_time_start`, `sim_time_end` (both sim seconds) and `steps`
   (int, basic steps elapsed). The wall clock says how loaded the machine
   was, not what the robot did, and two runs of the same command are only
   comparable on the physics clock. Either time field is `null` when the
   bridge did not stamp the start of the motion — never a number nobody
   measured. This is also the ruler the wait itself is decided on: a wait
   is a **step budget** (`timeout_s / dt` steps), never a wall-clock
   deadline, because a wall-clock deadline truncates every motion on a sim
   running below realtime.
8. ⭐ **A STALL IS NOT A TIMEOUT, and they MUST NOT share a field** (v9).
   A frozen world can never expire a step budget, so a wait that discovers
   the sim has stopped stepping returns `stalled: true` with
   `timed_out: false`, `achieved: null` and `settled: false`. An agent told
   "timed out" reissues the command into a simulation that is not running;
   an agent told "stalled" knows the instrument stopped, not the robot.
   Detecting the stall is the one place a wall clock is correct — "has the
   sim thread stopped?" is a wall-clock question — and it is reported
   separately for exactly that reason. The same condition also files
   `fault.telemetry_stale` on the bridge's event stream (§5.9).
   ⚠️ **`stalled` is the ONE field name for this**, on every bridge that can
   tell the condition apart. The two shipped implementations spelled it
   differently until 2026-09-22 — `stalled: true` on the mobile bridge,
   `measurable: false` on the arm — and two names for one concept on one
   wire contract is how a client comes to handle one robot class and
   silently not the other. Both now answer `stalled: true, timed_out: false`;
   the arm ALSO keeps `measurable: false` beside it, because that field
   already means something else on that bridge (see the conformance note
   below). A bridge that cannot distinguish a stall **MUST omit the field**
   rather than publish `stalled: false` — an absent field is honest, a
   verdict nobody measured is not.

**Conformance status, stated honestly:**

| bridge | 5.4.1 conformance |
|---|---|
| `omnilink_mobile_bridge` | ✅ **Full.** `wait: true` on `drive_forward` / `turn` returns `{commanded, achieved, error, unit, settled, timed_out}`; `get_robot_state.last_command` carries the same record; dispatch, completion, timeout, supersession and state all carry the same restart-safe `(bridge_instance_id, seq)` identity, with the UUID regenerated once per bridge process. `drive_to(x,y)` is always-blocking and returns `achieved_xy` / `error_m` / `arrived` — and now `settled` only when no leg timed out, having stopped the robot before reading the final pose. `drive_forward.achieved` is the displacement **projected onto the start heading**, so a robot pushed backwards reports a negative number (it used to `copysign` a magnitude onto the commanded value, which asserted the direction rather than measuring it). The `409 busy` check lives in `_begin_motion`, so it covers **every** entry point — `POST /tool`, the offline intent router and the idle loop included, not just the three path routes — and a waiter now matches its own `seq` **exactly**, returning `achieved: null, superseded: true` instead of the clobbering motion's measurement. `capabilities` publishes `actions`, `blocking_actions`, `waitable_actions`, `busy_rejecting_actions`, `busy_overriding_actions` and `site_bounds_m`. ⚠️ Two published verbs deliberately do **not** reject when busy: `stop_robot` and `set_velocity` are the escape hatches, so they *cancel* the running motion — whose `achieved` then reports `null` — and both say so in their descriptions (rule 6). |
| `husky_omnilink_bridge` | ◐ Partial — satisfies (1)–(3) on `drive_forward` / `turn`: both honour `wait` and return `{commanded, achieved, error, unit, settled}` measured against supervisor pose, `achieved` is `null` when not waited on, and `drive_to_waypoint?wait=true` returns `final_pose` + `distance_remaining_m` (commit `f57d910e`). **`turn` is fixed** for \|angle\| ≥ π: it tracks a signed **accumulated** residual instead of an absolute `wrap_pi` target, and reports `achieved` / `error` **unwrapped**. Previously the target wrapped onto the current yaw, so `turn(6.283185)` never moved the robot and answered `achieved: -0.000000, error: 0.000000, settled: true`, while `turn(3.5)` rotated **−2.783 rad — the short way, the opposite sign** — and reported a near-zero error, because a wrapped error cannot express an under-rotation past half a turn. A reply that cannot attribute a measurement to its own command `seq` now returns `achieved: null` with `measured_from: "unattributed"`. **Still missing:** (4) in part — the "NOT complete" warning is in the response body, but `/capabilities` publishes no `actions` list and therefore no per-action description; (5) busy-rejection entirely — a second motion command silently replaces the first; and (6) — the `drive_forward` accuracy figure below appears nowhere the model can read it. ⚠️ (1) is **violated by `drive_to_waypoint` without `wait`**, which answers `{"x": <target>, "y": <target>, "speed_m_s": …}` — the caller's own target under measurement-shaped keys, with no `commanded` key and no note that it returns before completing. ⚠️ `drive_forward` delivering **~65% of the commanded distance** (same early-stop-on-a-leading-pose class as `52f3f6ca`) is a **carried-over claim, not re-measured here** — it needs a live re-measurement before it is repeated or written into a description. |
| `mavic_omnilink_bridge` | ❌ **Non-conformant on its default path.** The row here used to claim `goto_waypoint` returns `final_pose`; it does not — the string `final_pose` does not occur anywhere in `mavic_omnilink_bridge.py`. `wait` defaults **false**, and the default reply is `{"x": tx, "y": ty, "altitude": target_alt}`: the caller's own target echoed under measurement-shaped keys, with no `commanded` key and nothing saying the flight has not happened yet — violating (1) and (4), the exact failure 5.4.1 exists to forbid. With `wait: true` it *is* mostly honest — `_wait_until_arrived` supplies measured `x` / `y` / `z` / `yaw` plus `distance_remaining_m`, and because `**res` is spread last those measured values shadow the echoed target — but `altitude` still holds the **commanded** value beside the measured `z`, so one payload mixes commanded and measured under similar keys. Registers **no relay `Tool` objects** (`Tool(` still occurs 0 times), which is what kept the defect latent rather than fixed. ⚠️ **That mitigation is now partly gone.** Since 2026-09-21 the bridge serves `POST /tool` (§5.8) over an in-process registry — `takeoff`, `land`, `hover`, `turn`, `move_body`, `stop_robot`, `reset_to_home`, `get_robot_state` — and `POST /prompt` (§5.7), so a model does now read some of this bridge's replies. `goto_waypoint` is not among the registered names and has no `route._ADAPTERS` entry either, so the specific payload described above is still reachable only through `POST /action`. The defect is unchanged; the reason nobody had hit it is one endpoint smaller. |
| `omnilink_arm_bridge` | ❌ Not yet. |
| `omnilink_quadruped_bridge` | ❌ Not yet. |

⚠️ **The five grades above were last audited on 2026-09-21 and were NOT
re-graded in the v9 pass.** What the v9 pass did change is narrow and is
stated per bridge rather than folded into a grade:

| new field (rules 7–8) | where it is shipped |
|---|---|
| `sim_time_start` / `sim_time_end` / `steps` on a completion record | `omnilink_mobile_bridge`, `omnilink_arm_bridge` |
| the same three on a wait that **did not** complete (timeout, stall) | `omnilink_mobile_bridge`, `omnilink_arm_bridge` |
| waits decided on a step budget rather than a wall clock | `omnilink_mobile_bridge`, `omnilink_arm_bridge` |
| the stall reported as its own outcome, as `stalled` | both, under **one** field name since 2026-09-22 |

The quadruped and flying bridges publish no measured completion record, so
they gain none of these. Do not read the table above as a change in any
row's grade; read it as the exact extent of the v9 work.

✅ **The two bridges that report a stall now spell it the same way.** Both
return `stalled: true, timed_out: false, achieved: null, settled: false`
plus the sim window, and both notes say the world stopped rather than the
motion failing; the arm's adds that the command *was* accepted and will
play out when the world resumes. Branch on `stalled === true`, or on
`timed_out === false && achieved === null` if you must also satisfy an
older bridge.

⚠️ **The arm ALSO emits `measurable: false`, and it is not a synonym.**
It was that bridge's only stall signal until 2026-09-22 — one condition
under two names across two robot classes, which is how a client comes to
handle one and silently not the other — and it is kept beside `stalled`
for the clients that already read it. But the arm emits the same field for
a **different** unmeasurable: a mirroring hardware link, where the world is
stepping perfectly well and the vendor controller owns the trajectory
(`/set_joint_positions` and friends answer `measurable: false` with no
`stalled`). So `measurable: false` means "no completion can be timed
here", for either reason; only `stalled: true` means the sim froze. A
client that treats them as one field will read a healthy real arm as a
dead simulator.

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

> ⚠️ **Ungated, and deliberately so.** `POST /stop_robot` passes no safety
> gate in any bridge, and it MUST NOT acquire one that can refuse it. A
> stop **reduces** what the robot is doing: a spurious stop costs a pause,
> a refused stop costs whatever the robot was about to hit. The gate
> itself encodes this — `stop` and `hold` are tier `safe` and are exempt
> from every intent rule (§5.8.3) — and the same asymmetry is why the REST
> route is left clear. This is the one ungated actuator on the bridge that
> is a design decision rather than a gap.

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

> ⚠️ **Ungated, and NOT deliberately.** `POST /reset_to_home` moves the
> robot through whatever lies between it and its home pose, and no bridge
> vets the REST route. The gate classes `reset_to_home` as physical and
> `guarded` — a *tool call* named `reset_to_home` arriving on `/tool`
> (§5.8) or through `/prompt` (§5.7) **is** vetted, and the same verb
> arriving on this route is not. That asymmetry is a known, unfixed gap
> (§16), not a property a client should rely on either way.

### 5.7 POST /prompt — required iff `prompt` ∈ actions

The natural-language endpoint: one operator sentence in, a reply and a
record of what was actually done out. A bridge MAY omit it entirely
(reporting it as absent from `/capabilities.actions`).

### 5.7.1 The interpretation cascade

v1.0's one line here — a bridge "MAY route through a local intent router, an
LLM, or omit this endpoint" — is **retired**, and its local intent routers were
deleted from the tree on 2026-09-22. It was never an accurate description of
the shipped bridges, and the difference matters to a caller: what
interprets a sentence decides which of `actions[]` it will see, and
whether a refusal is possible at all.

A conforming bridge SHOULD implement the cascade below. Each stage
**declines** rather than guesses, and the next stage gets the turn:

1. **Deterministic parser first.** `text` goes to a rule-based
   interpreter that returns typed frames — `{tool, args}` — plus an
   intent and a confidence. It handles the exact, unambiguous cases and
   abstains on everything else. It is the only stage whose decisions can
   be read back and re-run.
2. **Model second, on what the parser declined.** A bridge with a model
   relay attached sends the turn to the model, which emits tool calls of
   its own. Unset `OMNISIM_BRIDGE_PARSER_FIRST` means the parser **answers**
   its default intent set (confident `COMMAND` turns) and the relay gets
   everything else; `=0` opts back into *shadow*, where the parser still
   runs and records what it would have done but the relay answers every
   turn. ⭐ **A bridge MUST say which stage answered.** `via` is a
   top-level string on the 200 body, and since v9 it is **guaranteed
   present on every 200 from all five reference implementations** — the
   parser stamps `"parser"` when it short-circuits, the model relay stamps
   `"relay"` on every return including its own timeout envelope, and a
   shared helper fills `"relay"` as the default so the field cannot be
   best-effort. A bridge whose `/prompt` answers from neither stage — a
   canned reply, a queue, a second interpreter — MUST stamp its own value;
   nothing invents a third name for it.
   ⚠️ A client MUST NOT **infer** `via` from `actions[]`, from latency or
   from whether a key was configured. Read the field, or record that it was
   absent; inferring the column is fabricating the measurement.
3. **The gate vets whatever produced the frames.** The safety gate
   (§5.8.3) judges `(utterance, frames)` and does **not** look at which
   stage emitted them. This is the whole design: the same veto covers the
   parser, the model, and whatever interprets next. Both stages call it —
   the parser path before dispatching its frames, the relay path before
   dispatching each model tool call.

⛔ **There is no fourth stage, and there must never be one again.** Until
2026-09-22 all five reference bridges fell through to a legacy keyword
ladder (`IntentRouter`) that called `act_*` directly when no relay was
attached and the parser abstained. That ladder was **ungated** in four of
the five, and the measured cost of not screening the utterance before
entering it was 22% of "must not move" phrasings actuating the robot. All
five ladders and the shared `intent_router` module were **deleted** on
2026-09-22. A bridge with no relay attached now answers
`401 omnikey_required` on `/prompt` — the OmniKey access check runs before
the parser sees the sentence — and a bridge MUST NOT reintroduce a
keyword fallback under any name, including after a connection error.
Precisely: the four controller bridges answer `401` with
`error: "omnikey_required"` when no key is configured, and `503` with
`error: "omnilink_unavailable"` when a key is configured but the relay did
not attach. The `bridge_base` reference's **un-overridden** `act_prompt`
returns the same body through its generic action path, which reports it as
`501 not_supported`; a subclass that wires a relay never reaches it. In all
three cases nothing is interpreted and nothing moves.

A bridge MAY implement fewer stages. A bridge MUST NOT describe a stage
it does not run, and MUST NOT report a refusal it did not make.

**Request:**

```json
{ "text": "drive forward one meter" }
```

`text` is the operator's own sentence, and it is the only thing that lets
stages 1 and 3 judge *intent* rather than arguments. A caller that has an
operator sentence SHOULD send it here rather than decomposing it into
`/tool` calls client-side: `POST /tool` (§5.8) carries no sentence, so the
four intent rules cannot fire on that path.

**Response (200):**

```json
{
  "ok": true,
  "via": "parser",
  "response": "Driving forward 1.00 m (~11.2s).",
  "actions": [
    { "tool": "drive_forward", "result": "ok", "summary": "distance=+1.00 m" }
  ]
}
```

The `actions` array describes which tools the bridge actually invoked
in response to the prompt. Each entry MUST have `tool`, `result`, and
`summary` (string), plus `rule` on a refusal (§5.7.2).

⚠️ The shipped bridges answer `{via, response, actions, error?}` with **no
top-level `ok`** — permitted by §3.1, which allows either shape, but it
means a client MUST NOT use the presence of `ok` to decide whether the
prompt succeeded. Read `error` and `actions[].result` instead.

### 5.7.2 `result: "refused"` — a third value, and why it is NEW

> **This is a NEW addition to the wire contract.** v1.0 specified
> `result ∈ {"ok", "error"}`, and **neither value can express what the gate
> does.** A refusal is not a success: nothing moved. It is not an execution
> error either: nothing failed — the bridge understood the request perfectly
> and declined it on purpose. Reporting a refusal as `"error"` tells a caller
> to retry, which is the one thing it must not do; reporting it as `"ok"`
> tells a caller the robot moved, which is the fabrication §5.4.1 exists to
> forbid. The endpoint that *can* refuse had no way to say so.

`result` is therefore extended to the OPEN set:

| `result` | Meaning | Caller's correct response |
|---|---|---|
| `"ok"` | The tool ran and did what it reports. | Proceed. |
| `"error"` | The tool ran and failed, or could not run. | Retry, or surface the failure. |
| `"refused"` | ⭐ **NEW.** The bridge declined to dispatch the tool. Nothing was actuated. | Do **not** retry the same frames. Re-read `rule`, and ask the operator. |

An entry with `result: "refused"` **MUST** additionally carry:

- `rule` (string, REQUIRED on a refusal) — the machine-readable reason,
  from the enumeration in §5.8.3. This is the field a program branches on;
  `summary` is prose and MAY change between releases.

```json
{
  "ok": true,
  "response": "I won't do that: the utterance asks a question; a question may not move a robot.",
  "error": "drive_forward: interrogative",
  "actions": [
    { "tool": "drive_forward", "result": "refused", "rule": "interrogative",
      "summary": "the utterance asks a question; a question may not move a robot" }
  ]
}
```

A top-level `error` string MUST also be set whenever any entry's `result`
is `"error"` or `"refused"`. That rule is not new and it is not cosmetic:
a machine client reads fields, not sentences, and a two-hour endurance run
lost ~5% of its orders to refusals that were visible only in `response`
prose, logging zero errors from start to finish while the robot drove off
the floor.

**Clients MUST treat `result` as an open set** and MUST NOT assume an
unrecognised value means success. Values already emitted in this tree
beyond the three above: `"no_action"` (the bridge deliberately did
nothing — a declined question, an unrecognised sentence) and
`"unsupported"` (the verb exists in the vocabulary and not on this robot).
Both are closer to `"refused"` than to `"ok"`, and neither actuated
anything.

> **Conformance, stated honestly (rewritten 2026-09-22).** The note that
> stood here — "no shipped bridge emits `rule` yet, and the two producers
> disagree on the spelling of a refusal" — is **out of date**. Both
> producers now spell a refusal `result: "refused"` and carry `rule`:
>
> - **the parser path** emits `{result: "refused", rule: "<rule>", summary:
>   "<prose>"}`. It used to put the rule name in `summary` and drop the
>   prose entirely, so the only human-readable account of the refusal —
>   "distance=300.0 exceeds the 50 m rail" — existed nowhere in the
>   envelope, and one field carried two jobs badly;
> - **the model-relay path** emits the same shape. A gate refusal takes its
>   rule from `reject_toolcall`'s `"<rule>: <detail>"` string; the
>   fail-closed wrapper's "safety gate unavailable (…)" gets
>   `gate_unavailable` rather than borrowing a rule name it did not earn; a
>   bridge-level refusal gets the bridge's own rule or `bridge_refused`.
>
> A refusal that reaches the reply with no classification at all is filled
> in as `rule: "unclassified"` rather than left absent, so a client
> branching on the key never has to distinguish "no rule" from "no field".
> `unclassified` is a placeholder, not a rule name from the §5.8.3
> enumeration, and it is deliberately **not** used in the top-level `error`
> string — the prose summary is a strictly better error than the word
> meaning "we do not know".
>
> **What a client must STILL tolerate on the wire**, and why this section's
> "open set" rule has not relaxed:
>
> - `"err"` and `"error"` both remain in the wild for a *failure*. They are
>   not the same string, both are produced, and a client MUST accept
>   either. The shared reply builder treats both — plus `"refused"` — as
>   setting the top-level `error`.
> - An **external** bridge copied from a pre-2026-09-22 reference emits
>   neither `rule` nor the `"refused"` spelling. The OmniLink platform
>   therefore still recovers a rule from a `"<rule>: …"` or
>   `"refused: <rule>: …"` summary prefix when the field is absent, and
>   yields *undefined* rather than a guess when the prefix does not parse.
>   That recovery is a compatibility path for other people's bridges, not a
>   description of these five.
> - `"no_action"` and `"unsupported"` are unchanged and still mean nothing
>   was actuated.
>
> ⚠️ The top-level `error` on a refusal is built from `result`, **never**
> from a tool's numeric `error` residual: a 2 m drive landing at 1.9977 m
> carries `error: -0.0023` inside its result and sets no top-level error at
> all. That is §5.4.1's control error, a float, and reading it for
> truthiness has shipped as a bug three times.

### 5.8 POST /tool — the platform tool callback, required iff the bridge serves a tool surface

> **Status: universally implemented, undocumented until this edition.**
> `POST /tool` is served by `bridge_base` — the reference every external
> bridge copies — and by all four in-tree controller bridges (mobile, arm,
> quadruped, flying). It is the surface the OmniLink platform posts a
> model's tool calls to, and it is the path a model's command actually
> arrives on in the shipping configuration. It appeared in this document
> only incidentally, in §5.4.1 and §16. An undocumented, universally
> implemented, safety-critical endpoint is a wire-protocol bug by this
> document's own opening rule, and this section closes it.

Where `/prompt` (§5.7) takes a *sentence* and lets the bridge decide which
tools to run, `/tool` takes an *already-resolved tool call* and runs it.
The interpretation happened elsewhere — on the platform, in a model the
bridge never saw. A bridge that serves any tool surface to an external
agent SHOULD serve this endpoint, and MUST gate it (§5.8.3).

### 5.8.1 Request

```json
{ "tool": "drive_forward", "distance": 1.0, "speed": 0.4 }
```

- `tool` (string, REQUIRED) — the tool name, as registered on this bridge.
  MUST be non-empty; an empty or non-string value is `400 invalid_type`.
- `id` (string, OPTIONAL) — the §3.3 idempotency key. A bridge MUST strip
  it before the arguments reach the gate or the tool. ✅ **Fixed on the
  flying bridge, 2026-09-22.** It used to strip `id` only on
  `POST /action`, so an `id` sent to its `/tool` survived into the argument
  set and came back `unknown_arg` — a transport detail wearing a safety
  verdict's clothes. All five implementations now share one `/tool`
  handler, and that handler strips the transport fields before the gate
  sees them.
- **Every other top-level key is an argument to the tool.** There is no
  nested `args` object: the arguments are siblings of `tool`. This is
  deliberate (it is the shape the platform posts) and it is the reason
  `tool` and `id` are reserved names that a tool MUST NOT declare as
  parameters.
- `utterance` (string, OPTIONAL) — the operator sentence this tool call was
  derived from, when the caller has one. A bridge that accepts it MUST
  strip it before dispatch and MUST pass it to the gate, which cannot
  otherwise apply its four intent rules (§5.8.3) on this path. ✅ **All
  five implementations honour it since 2026-09-22** (it was the flying
  bridge only), because they share one handler. The OmniLink platform now
  sends the operator's **original** sentence on every round of a tool loop
  — not the most recent "user" turn, which on that loop is the hook's own
  tool-result feedback; feeding those back would have the gate judge the
  robot's output as though a human had said it.
- `surface` (string, OPTIONAL) — ⚠️ **accepted, stripped, and IGNORED.**
  It is stripped so it does not land in the argument set and refuse the
  call as `unknown_arg`; its value is discarded because the bridge's own
  surface is authoritative. See §5.2.1, "A `surface` sent over the wire is
  accepted and IGNORED", for why that is a security property rather than a
  gap. A bridge older than 2026-09-22 refuses the field as `unknown_arg`;
  a caller that must work against both probes once per endpoint and drops
  the field the refusal names.

### 5.8.2 Responses

**200 — dispatched:**

```json
{ "status": "ok", "tool": "drive_forward", "result": { "...": "..." } }
```

`result` is the tool's own return value, verbatim, and is subject to
§5.4.1: a physical tool's result MUST report what was achieved, not what
was commanded.

⚠️ **This is not the §3.2 ok-envelope.** `/tool` answers `status` ∈
`{"ok", "err"}` rather than `ok: true|false`, on both the success and the
`5xx` paths. It is shipped behaviour on five implementations and is
described here as it is; the refusal path below *does* use the §3.2
envelope, so a caller of this one endpoint must be ready for both shapes.

**400 `refused_by_gate` — the gate declined to dispatch:**

```json
{
  "ok": false,
  "error": "refused_by_gate",
  "message": "implausible: distance=300.0 exceeds the 50 m rail",
  "details": { "tool": "drive_forward", "rule": "implausible" }
}
```

The `message` is `"<rule>: <detail>"` — the rule name (§5.8.3) followed by
the human-readable reason. `details.tool` names the tool, and
`details.rule` (v9) carries the rule name as its own field so a caller
does not have to re-parse prose. **Nothing was actuated.** A caller MUST
NOT retry the same arguments; it MUST branch on the rule and, where there
is an operator, ask.

**503 `tool_not_registered` — nothing here can serve that name:**

```json
{ "status": "err", "tool": "drive_forward", "error": "tool_not_registered" }
```

Returned by the four controller bridges when the name is not in the
bridge's tool registry **or when no relay is attached at all** — those two
cases are not distinguished on the wire, and the second is the common one:
a bridge running without a platform key registers no tools, so *every*
`/tool` call answers 503. `503` rather than `404` is correct for that case
(the bridge is alive but not ready), and it is why an end-to-end gate test
that sees only 503s has proved nothing about the gate: five refusals for
the wrong reason look exactly like five refusals for the right one. A
client MUST include at least one call it expects to *succeed* before
concluding anything from a refusal.

The `bridge_base` reference answers an unknown name differently — `404`
with `{"status": "err", "tool": …, "result": {"error": "unknown tool: <name>",
"known": [...]}}`, because its registry is a static alias table rather than
a relay. Both are conformant; a client MUST handle `404` and `503` alike
as "this name is not dispatchable here" and SHOULD read `result.known`
when it is present.

**500 `tool_execution_failed` — the tool raised:**

```json
{ "status": "err", "tool": "pick", "error": "tool_execution_failed",
  "detail": "ValueError: no grasp target" }
```

`detail` is OPTIONAL and unstable. A bridge SHOULD include it: without it
a tool that raised is indistinguishable from a tool that raised something
else, from either end of the wire.

Other codes from §11 apply as usual — `400 invalid_arguments` when the
arguments do not bind to the tool's signature, `401 unauthorized`,
`409 busy` where the bridge enforces single-flight motion (§5.4.1 rule 5;
the mobile bridge's busy check sits in its motion entry point, so it covers
`/tool` as well as its REST routes).

### 5.8.3 The safety gate on `/tool`

A bridge serving `/tool` **MUST** vet the call before dispatch. The
in-tree gate is
[`packages/omnisim-bridges/src/omnisim_bridges/gate.py`](packages/omnisim-bridges/src/omnisim_bridges/gate.py),
entered through `check(utterance, frames, authorized=(), surface=None)`
for a whole turn or `reject_toolcall(tool_name, args, utterance="", surface=None)`
for one bare call. It judges `(utterance, frames)` and does not look at who
produced them.

**Rules it can emit.** The `rule` half of a refusal is drawn from this
enumeration; it is an OPEN set and a client MUST tolerate an unknown name.

| Rule | Fires when |
|---|---|
| `interrogative` | The utterance asks a question, and a question may not move a robot. |
| `prohibition` | The utterance forbids an action rather than ordering one. |
| `self_negating` | The utterance asks to act and not act at once. |
| `reported_speech` | The utterance reports an order rather than giving one. |
| `retracted` | The order was withdrawn and nothing replaced it. |
| `deferred` | The order waits on an event that has not happened. |
| `contradiction` | The utterance asks for two things that cannot both be done. |
| `invented_magnitude` | The frame carries a distance or angle the utterance never gave. |
| `sign_conflict` | The utterance says forward and the magnitude is negative. |
| `implausible` | A magnitude is past its rail (below). |
| `unknown_arg` | An argument is not a parameter of this tool. |
| `missing_arg` | A required argument is absent. |
| `bad_type` | An argument has the wrong JSON type. |
| `not_finite` | A numeric argument is `NaN` or infinite. |
| `unresolved_referent` | A slot is still a bare pronoun (`it`, `there`) — it was guessed, not resolved. |
| `needs_authorization` | A `confirm_required` tool was called without an authorization token. |
| `unknown_tool` | No schema declares this tool. ⚠️ **Deliberately dropped — see below.** |

The first nine rows — `interrogative` through `sign_conflict` — are
properties of the **sentence**, not of the frame, and the four *intent*
rules among them (`interrogative`, `prohibition`, `reported_speech`,
`retracted`) are what this endpoint most obviously loses. On `/tool` there
is usually no sentence, so all nine stay silent and are not expected to
fire. `invented_magnitude` in particular **MUST**
be vacuous without an utterance: "the sentence never gave a number" means
nothing when there is no sentence, and a version of this rule that fired
anyway refused every drive, turn and takeoff with a non-zero magnitude
posted to `/tool`.

What does fire is the half that needs no utterance — magnitude rails,
schema validation and unresolved deixis. That is what stops
`set_velocity {v: 40}` and `drive_forward {distance: 300}` arriving over
HTTP, and it is the practical reason a caller with an operator sentence
SHOULD send it as `utterance` (§5.8.1) or use `/prompt` instead.

De-escalating tools (tier `safe`: `stop`, `hold`) are exempt from
`prohibition`, `retracted`, `reported_speech` and `deferred`, because a
refused stop costs more than a spurious one — "stop moving" reads as a
prohibition and must still halt the robot. `interrogative` and
`self_negating` are **not** lifted for them.

**`unknown_tool` is dropped at every call site, and that is load-bearing.**
The gate does **not** police tool existence — the bridge's own registry
already resolved the name, and overruling it would refuse tools that are
named at runtime (the arm's learned verbs can never appear in a static
table). The consequence a client and a bridge author both need: **a tool
the gate has never heard of is not merely unknown, it is UNCHECKED.**
`check()` emits `unknown_tool` and skips every other rule for that frame,
and the call site then discards the one rejection it produced. A tool
therefore MUST be registered — `gate.register_tools(tools, surface=…)`,
which reads the tool's own JSON-schema parameters — to be checked at all.
Until 2026-09-21 most of the arm's surface was not, and `grasp`,
`set_tcp_target`, `set_joint_positions`, `set_gripper_width`,
`run_learned_skill`, `goto_waypoint` and `set_yaw` reached a motor with no
schema, magnitude or deixis check whatsoever.

**Rails are per-quantity and per-surface.** They are "nobody meant this"
limits, not bounds (§5.2.1):

| Quantity | Rail | Applies to |
|---|---|---|
| `distance` | `MAX_DISTANCE_M` = 50 m | ground distance |
| `angle_rad` | `MAX_ANGLE_RAD` = 8π rad | four full turns |
| `speed`, `v`, `linear` | `MAX_SPEED_MPS` = 5 m/s | linear velocity, however it is spelled |
| `w`, `angular` | `MAX_YAW_RATE_RPS` = 6 rad/s | yaw rate |
| `altitude` | `MAX_ALTITUDE_M` = 120 m | flight altitude |
| `forward`, `lateral`, `vertical` | `MAX_BODY_SHIFT_M` = 1.0 m | a quadruped body shift |

The last row is why `surface` is an **argument** and not a property of the
tool. `move_body` is served by both the quadruped and the drone with the
same three argument names meaning entirely different things: a drone's
`vertical` is a climb and rails on `MAX_ALTITUDE_M`, its horizontal
translation is a flight and rails on `MAX_DISTANCE_M`, and a quadruped's
three axes are body shifts railing on `MAX_BODY_SHIFT_M`. A bridge
**MUST** pass its own `surface` (`mobile`, `arm`, `quadruped`, `drone`)
rather than let the rail depend on which bridge registered the tool first
— an order-dependent safety property is worse than a surface-blind one,
because it looks correct in whichever order you happen to test.

**That the order no longer matters is now PROVEN, not assumed.** The parity
generator regenerates the post-registration verdicts under all 24
permutations of (drone, quadruped, arm, mobile) and refuses to write the
fixture if any permutation disagrees, naming the rows. The guard was run
red on purpose — reverted to the pre-`a85fca20f` behaviour, it fired on 7
rows and exited 1 without touching the fixture — and the output is stable
across `PYTHONHASHSEED`.

⚠️ **The exact fallback rule when the caller passes no surface is in
§5.2.1**, and it is *not* "always the strictest rail". Read it there before
porting this gate: the recorded surface **is** the rail when exactly one
class registered the tool, and only a tool registered by two or more falls
to the ground rail.

A refusal on this path is also filed as a `gate.refused` event on the
bridge's own stream (§5.9), because otherwise it is the one thing that
happens here which nobody sees: the caller gets a 400 and the robot's own
agent learns nothing.

**It fails closed on physical tools.** If the gate cannot be imported or
raises, a physical tool **MUST** be refused; the refusal reason names the
failure (`"safety gate unavailable (ImportError)"`). Read-only tools still
answer — refusing to report a position because a *motion* check is
unavailable would be a self-inflicted outage. A bridge author copying this
endpoint MUST copy the fail-closed wrapper with it: a safety check that
vanishes with its import is not a safety check.

⚠️ **Gating `/tool` gates `/tool`, and nothing else.** The audit behind
this section is
[`packages/omnisim-bridges/GATE_COVERAGE.md`](packages/omnisim-bridges/GATE_COVERAGE.md);
read it before assuming any other path is protected. Gating the
`bridge_base` reference did **not** cover the four controller bridges,
because each ships its own `/tool` handler — measured 2026-09-21, a gated
`bridge_base` still let `{"tool": "drive_forward", "distance": 300}`
through a controller bridge and hung the HTTP call for 90 s while the
robot drove it. ✅ **That divergence class is closed as of 2026-09-22**:
the **five** near-identical `/tool` handlers — `bridge_base`'s and one in
each controller bridge — were collapsed onto one implementation, and the
**six** copies of the fail-closed wrapper (those five plus the model
relay's dispatch) onto one. A fix now lands everywhere at once, and a
bridge cannot quietly ship a sixth spelling. ⚠️ **The
direct REST actuators (§5.4, §5.5, §5.6, §6) remain ungated in all four
bridges and in `bridge_base`**; that is unchanged and is the reason
`ungated_paths` (§5.2.1) is the load-bearing half of the capability block.

### 5.9 GET /events — the bridge's own event stream

> **Status: NEW in v9 (2026-09-22), and implemented.** Served by all five
> reference implementations. Additive under `1.1`. Until v9 the sim→agent
> channel was **pull only**: a joint pinned against its limit, a motion
> that quietly gave up, a refusal on an ungated path — none of them reached
> the agent unless a tool result happened to carry it, and the agent had no
> way to ask *what happened while I was thinking?* This endpoint is that
> channel.

⚠️ **This is NOT the World Harness's `GET /sim/events` (§7.19).** They
share an envelope shape on purpose, so one client loop can drain both, and
they are otherwise different streams:

| | bridge `GET /events` (§5.9) | harness `GET /sim/events` (§7.19) |
|---|---|---|
| port | the robot bridge (`8765`, Mavic `6090`) | the harness (`6789`) |
| cursor | one, `since` / `next_since` | **two**, `since` **and** `log_since` |
| clock on an event | `sim_time` (float sim seconds) + `step` | `t_sim_ms` (int ms) or `t_wall` (epoch) |
| lifetime | **in-process; dies with the bridge** | the supervisor's ring buffer |
| capacity | 512 events by default | 4096 per side |
| who measures | this one robot's bridge | the whole scene's supervisor |

**Never carry one stream's cursor into the other.** An empty ring after a
bridge restart means "nothing since boot", not "nothing happened" — the
ring is not persisted and is not restored.

**Request:** `GET /events?since=<cursor>&limit=<n>&types=<a,b,c>`

The four controller bridges serve it on `GET` only. The `bridge_base`
reference also answers `POST /events` (reading the query string, not a
body), for clients confined to a POST-only verb surface; a bridge MAY
serve either or both.

- `since` — return events with `seq` greater than this. Default `0`.
  ⚠️ A malformed cursor reads as `0` rather than `400`: an agent that has
  lost its place needs the stream more than it needs a lecture.
- `limit` — default `100`, clamped to `[1, 1000]`.
- `types` — comma-separated **exact-match** allowlist. Same trap as §7.19:
  there is no prefix matching, no globbing and **no error for an
  unrecognised name**, so `?types=damage.*` returns `{"events": [], …}`
  with HTTP `200`, indistinguishable from a quiet robot. Copy the names
  from the table below. `next_since` advances past filtered-out events, so
  a filtered poller does not re-scan the buffer forever.

**Response (200):**

```jsonc
{"events": [ /* … */ ],
 "next_since": 42,     // the cursor for the next poll
 "dropped": 0,         // cumulative EVICTIONS (the harness's dropped_sup semantics)
 "total": 42,          // every event ever filed, evicted ones included
 "missed": 0,          // evicted before THIS cursor read them: poll faster
 "buffered": 42, "capacity": 512,
 "robot": "husky", "sim_time": 12.48, "step": 390}
```

One event:

```jsonc
{"seq": 12,                 // monotonic from 1; never resets while the process lives
 "type": "joint.limit_hit",
 "sim_time": 4.096,         // SIM seconds, never the wall clock
 "step": 128,
 "robot": "husky",
 "source": "bridge",        // or "harness" for a forwarded one
 "detail": { /* per-type, see below */ }}
```

`dropped` and `missed` answer different questions and a client should read
both: `dropped` is how many events the ring has evicted since the process
started, `missed` is how many **this** cursor will never see. Either going
non-zero means poll faster or raise `limit`; it also means a report built
from this stream is incomplete and must say so.

**A bridge with no ring answers `200` with `"error": "this bridge serves
no event ring"` and an empty list**, and a build without the bridges
package answers `501 not_supported` with the same sentence. Neither is
evidence that nothing happened.

#### The types, and which detector emits each

`source: "bridge"` — measured by this bridge:

| type | detector | thread | shipped on | wakes? |
|---|---|---|---|---|
| `motion.timed_out` | motion-completion emitter | sim | mobile, arm | yes |
| `motion.unsettled` | motion-completion emitter | sim | mobile, arm | **no** — a sloppy stop is not an emergency |
| `joint.limit_hit` | `JointLimitDetector` (hysteresis; re-arms only after clearing the band) | sim | arm, quadruped | yes |
| `fault.controller_lost` | `FaultDetector.on_tick`, rising edge on `/state.fault` | sim | all four controller bridges | yes |
| `fault.telemetry_stale` | `FaultDetector.poll_stale` | ⚠️ **the READER's thread** | all four | yes |
| `gate.refused` | filed where the gate said no | HTTP thread (`/tool`, `origin: "tool"`) and the parser path (`/prompt`, `origin: "prompt"`) | all five | only when `detail.origin` is **not** the agent's own turn |
| `contact.began` | `ContactDetector` | sim | mobile | only for a non-floor body |
| `hold.expired` | the lockstep lease (`detail`: `note`, `leak_steps`) | sim | mobile, arm, quadruped | **no** — bookkeeping, not physics |

⚠️ **`gate.refused` has two producers, not three.** A refusal of a
**model's** tool call inside a relay turn is reported in that turn's
`actions[]` (§5.7.2) and is **not** filed on the ring — the agent is
already reading it. Both producers fill `detail.rule` with a real rule
name and both fall back to `gate_unavailable`, spelled identically, when
the reason has no parsable rule head; a consumer branching on the rule
cannot tell which of them wrote the event and must not have to (§11.1).

⚠️ **`fault.telemetry_stale` is polled by the reader, and that is the
point.** It means *the sim thread has stopped*, so a detector that only ran
in `tick()` could never fire for the one condition that means `tick()` is
not running. `GET /events` polls it on the way through. It is also the
event twin of §5.4.1 rule 8: a stall, not a timeout.

⚠️ **`contact.began` is TOP-LEVEL SCOPE ONLY**, and the event says so
itself — every one carries `scope: "top-level"` and a `blind_spot`
sentence. The supervisor's `getContactPoints()` reports contacts on
top-level Solids; **a URDF robot's sub-links are invisible to it**. The
counterpart body is named by joining on the shared contact *point* (never
on a contact point's node id, which carries the queried solid's own id and
made every pair read `(X, X)`), against a **bounded watchlist** — a
controller tick cannot afford to walk every Solid in the world the way the
harness can. A body outside that watchlist produces no event. Floor
contacts are filtered: a mobile robot touches the floor for the whole run.
**The absence of a `contact.began` is never evidence that nothing touched.**

`source: "harness"` — **forwarded, not measured**, when
`OMNISIM_HARNESS_URL` names a World Harness: `break.hit`, `damage.impact`,
`damage.state_transition`, `joint.limit_hit`, `contact.began`,
`contact.ended`. Forwarding is what lets one client loop see what the robot
measured *and* what the supervisor saw. Three properties a client needs:

- the harness's **log** types (`controller.log`, `world.warning`,
  `world.error`) are deliberately **not** forwarded — they are this
  bridge's own stdout coming back around and would loop;
- a forwarded event's `step` and `sim_time` are **when it ARRIVED**, not
  when it happened. The harness's own clock is preserved in the detail as
  `harness_t_sim_ms` (and `harness_seq`, `harness_robot`) rather than
  thrown away — two rulers, both reported, neither pretending to be the
  other;
- a `source: "harness"` type stops arriving the moment the harness goes
  away, and **that is not a statement about the robot**. `source` is how a
  client tells a type this bridge measures from one it merely relays.

⚠️ **The pass-through runs on a daemon thread, NOT the simulation tick.**
A blocking HTTP call on the tick is a blocking call on the thread the whole
simulation waits on: a harness that is slow, paused or gone would stall the
world for the socket timeout on every poll. Emitting is thread-safe
(the ring holds a lock and touches no device); reading the controller API
is not, and nothing in this path does.

#### What this stream does NOT change

- **Detection is still never sub-step**, and the two break-latency regimes
  of §7.40 are untouched: held plus `/sim/step`, at most one basic step of
  engine time; free-running, one supervisor tick — measured 8 ms to
  ~600 ms of engine time — inside which a whole one-second drop has fitted
  with no `contact.began` firing at all. A bridge event is *at best* as
  timely as the detector that produces it, and the sim-thread detectors
  run once per tick (the contact detector every 16 ticks). Nothing here
  makes a transient visible that was invisible before. The sim-thread
  detectors are also **sub-sampled**: the arm reads its joints every tick,
  the quadruped every 4 ticks, and the mobile bridge polls contacts every
  16 ticks, because a controller tick cannot afford what the supervisor
  can.
- **It is not a record, a replay or a run-diff.** It is a bounded ring of
  what was noticed, in one process, with eviction reported rather than
  hidden.
- **`/sim/contacts`-style honesty applies**: a stream that cannot answer
  says so (`error`, or `available: false` on the relay tool) instead of
  returning an empty list that reads as "all quiet".

#### Discovery, and the other two surfaces that read it

- `GET /capabilities.events` = `{endpoint, state_field, types[]}` (§5.2),
  published by the four controller bridges.
- `GET /state.events` = `{total, last, next_since, dropped}` (§5.3), so a
  reader learns whether it has fallen behind without a second request.
- The bridge's own model relay registers a `get_events` **tool**, so the
  robot's agent can read its own events; with no ring attached that tool
  answers `{"available": false, …}` with a reason — never `None` and never
  a bare empty list, because an empty list reads as "nothing happened" and
  that is a claim.
- The MCP server exposes `robot_events`, which calls **this** endpoint and
  passes the envelope through verbatim. ⚠️ It is not the relay's
  `get_events`: same ring, but two clients and two cursors, and a cursor
  from one must never be sent to the other.

#### Waking the agent on an event

A bridge with a model relay attached MAY dispatch **one** model turn in
response to a qualifying event, through the same cascade and the same gate
as an operator sentence (§5.7.1). This is bridge behaviour, not a wire
contract — nothing about `GET /events` changes — but a client should know
it exists, because it spends the operator's model budget:

- only physical or safety events qualify (the "wakes?" column above);
- `contact.began` wakes only for a **non-floor** body, and `gate.refused`
  only when the refusal did **not** come from the agent's own turn — a
  refusal on `/prompt` is already in the reply the model is reading, and
  waking it about that is a loop with a bill;
- the rate limit is **module constants, not environment variables**
  (one wake per 10 s, 30 per session), deliberately: an over-firing policy
  is narrowed, not a limit raised;
- the first poll only **primes** the cursor, so a restart with a full ring
  does not wake on history;
- `OMNILINK_EVENT_WAKE=0` disables every wake while the ring still fills
  and `get_events` still answers;
- no key means no relay, which means no wake and no error.

⚠️ **Idle wake frequency on a cluttered world has never been measured.**
The rate limit is the only guard, and the policy default is not defended
by a measurement yet.

#### Environment

| variable | effect |
|---|---|
| `OMNISIM_BRIDGE_EVENT_CAPACITY` | ring size (default `512`), clamped to `[16, 100000]`; evictions are reported as `dropped` |
| `OMNISIM_HARNESS_URL` | the World Harness base URL. Pre-existing and previously undocumented; the MCP server and the ROS 2 client read it too. Setting it turns the pass-through on |
| `OMNISIM_BRIDGE_HARNESS_EVENTS` | `=0` stops mirroring the harness's events onto this ring while leaving the harness attached |
| `OMNILINK_EVENT_WAKE` | `=0` disables the wake above; the ring still fills |

All are value-parsed (`=0` means off). The generated list of every
`OMNISIM_*` variable is
[docs/reference/environment-variables.md](docs/reference/environment-variables.md).

---

## 6. Robot Bridge — per-class endpoints

Per-class endpoints are required iff `class` matches and the
corresponding capability bit is `available: true`. A bridge MAY also
expose the same operation via `POST /action` with the action verb — the
direct endpoint exists for clients that prefer one-verb-per-endpoint
(the Axis convention); the typed `/action` exists for clients that
prefer a single dispatch point (the original arm bridge convention).
Both forms MUST produce identical state effects and identical responses.

> ⚠️ **None of the §6 routes is vetted by a safety gate.** Audited
> 2026-09-21 ([`packages/omnisim-bridges/GATE_COVERAGE.md`](packages/omnisim-bridges/GATE_COVERAGE.md)):
> in all four reference bridges and in `bridge_base`, the §6 routes that
> are actually served — `/drive_forward`, `/turn`, `/drive_to`,
> `/set_velocity`, `/set_joint_positions`, `/servo_joint_positions`,
> `/set_tcp_target`, `/open_gripper`, `/close_gripper`, `/grasp`,
> `/release` — reach an actuator with nothing in between, for any HTTP
> caller holding the token. The gate is on `POST /tool` (§5.8) and on the
> `/prompt` cascade (§5.7); calling the *same verb* through one of those
> paths is checked, and calling it here is not. The arm's learned verbs,
> which are promoted to `POST /<verb>` routes at runtime, are ungated for
> the same reason. The §6.3 flying and §6.4 quadruped verbs are not served
> as routes at all (§16) — they are `/action` verbs or tool calls, and on
> the flying bridge those *are* gated (§5.4).
>
> That is a stated gap, not a claim about what is safe. **A client driving
> these routes is the only thing bounding the magnitudes it sends**, and
> "the gate would have caught it" is not true here. Nothing in the stack
> limits a robot to the floor it is standing on: a two-hour soak walked a
> Husky off a 12 m floor and nothing objected.

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

- `POST /set_velocity { "linear": v_m_s, "angular": w_rad_s, "wait": bool? }`
- `POST /drive_forward { "distance": d_m, "speed": v_m_s? }`
- `POST /turn { "angle": rad }`
- `POST /drive_to_waypoint { "x": ..., "y": ... }` (optional; declare in capabilities)

#### The yaw envelope is MEASURED, not derived

A mobile bridge MUST publish `capabilities.max_angular_rad_s` as a yaw rate
the base can actually hold, and MUST clamp `set_velocity` against that
number rather than against `wheel_speed x radius / half_track`. The two are
not close on a skid-steer base, and the gap is not a constant you can copy
from this page.

⚠️ **The three numbers this section used to quote are withdrawn.** Measured
2026-09-10 under Newton/MuJoCo, a Clearpath Husky whose geometry implies
3.47 rad/s held **0.119**, a Husarion ROSbot XL implying 4.65 held **0.032**,
and a TurtleBot3 Burger implying 2.475 held **0.328**. Those are
**pre-`69b4b024b` (2026-09-11)** and will not reproduce: the solver was
bounding a pure-velocity motor's gain at `kv <= M_ii/dt`, so a wheel's stall
torque tracked its own rotational inertia instead of the effort the URDF
declares. It was never the differential the solver refused -- it was the
servo gain the solver starved, and a pivot is simply the first real LOAD a
wheeled base meets. Straight-line speed was unaffected throughout (96-100%
of command on all three, tracking 0.9991 before and after), which is exactly
why the defect hid for so long. Open-loop chassis yaw ratio before -> after:
**Husky 0.0069 -> 0.532, TB3 Burger 0.2383 -> 0.958, ROSbot XL
0.0069 -> 0.525**. The post-fix figures land where physics puts them rather
than at 1.0: the Burger is a true two-wheel differential drive and scrubs
nothing, while the two skid-steers must scrub all four tyres (a Coulomb
moment balance for the Husky's geometry predicts a 0.55 ceiling). No bridge
has re-published its `max_angular_rad_s` against the post-fix engine yet.

The rule is unchanged and is the whole point: publishing the geometric
figure means every angular command is accepted and none is delivered, and
publishing a *stale measured* figure is the same defect one release later.
Measure the base you ship, on the engine you ship. Where the two differ,
publish both, plus the ratio:
`max_angular_rad_s`, `max_angular_rad_s_kinematic`, `yaw_rate_gain`,
`can_rotate_in_place`, `can_strafe`.

#### set_velocity reports what the base DID (rule 1 applies to it too)

`set_velocity` has no completion to wait for, which is exactly why it used
to answer with the arguments it had been handed
(`{accepted, linear, angular}`). That is the bare echo rule 1 forbids: on
the Husky it said "angular 2.0 rad/s" while the base turned at 0.0115.
(That 0.0115 is a pre-`69b4b024b` measurement and will not reproduce; the
echo defect it illustrates is independent of how much yaw the base can
deliver.)

A conforming bridge MUST answer with
`{commanded: {linear, angular}, applied: {linear, angular}, achieved:
{linear, angular} | null, error: {linear, angular} | null, settled: bool}`.
`applied` is the request after clamping; `achieved` is measured over a short
hold (the reference bridge discards 1.2 sim-s of ramp and differences the
next 0.8 s of settled pose) and is `null` -- **never** the commanded value --
when the caller passed `wait: false` or no samples arrived. When the request
was clamped, the reply MUST also carry `clamped_from`, a machine-readable
`limited` reason and a `limit_note` naming the measured ceiling.

#### A rotation the base cannot finish is REFUSED, not timed out

Where a closed-loop `turn` would need more spinning time than the bridge is
willing to spend, it MUST refuse up front with
`{accepted: false, refused: "cannot_rotate_in_place", max_angular_rad_s,
max_angular_rad_s_kinematic, yaw_rate_gain, estimated_spin_s, message,
alternatives[]}`. A ROSbot XL asked for 90 degrees previously answered
`{settled: false, timed_out: true}` with 0.0875 rad achieved -- a timeout is
what a loop reports when something went wrong, and nothing had: the base
held 0.032 rad/s, so the rotation needed 49 s. An agent can act on the
refusal; it cannot act on the timeout.

⚠️ **That 0.032 rad/s and the 49 s it implies are pre-`69b4b024b`
(2026-09-11) and are withdrawn** -- the XL's open-loop yaw ratio went
0.0069 -> 0.525 when the wheel stall-torque cap was lifted. The requirement
above stands unchanged; the *constant* behind it does not. As shipped today
the XL's `cannot_rotate_in_place` refusal is a **false negative on a public
API**: it refuses a rotation the base can now perform. That is the second
failure mode of this rule and it is worth stating plainly -- a refusal
threshold **MUST** be derived from a ceiling measured on the engine in
front of you, never from a constant baked into a bridge or copied out of a
document. A bridge that hard-codes either number is wrong the moment the
solver changes.

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
> 2026-09-01** — 21 `GET` routes and 18 `POST` routes (39 total, counted
> from the harness's own `ROUTES` table, including the diagnostic
> `GET /debug/read_bench` (§7.35), the W1.7 mid-run rebuild
> `POST /sim/rebuild_physics` (§7.36) and the particle-state readback
> `GET /scene/node/<def>/particles` (§7.37)), and no `DELETE`.
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
  "hot_reloaded": true, "diagnostics": [],
  "tracking": { "light": true, "mode": "light", "default_applied": true,
                "hint": "light mode: /sim/grips is empty and contact.*/grip.*/joint.limit_hit events are not produced; /sim/contacts still answers.",
                "default": { "light": true, "mode": "light", "source": "built-in", "since": "2026-09-02",
                             "why": "...", "revert": "...", "explicit_wins": "..." },
                "default_note": "LIGHT tracking was applied BY DEFAULT (built-in; light is the default since 2026-09-02) because the request named neither `light` nor `tracking`: pass {\"light\": false} ..." } }
```

`tracking` (added 2026-08-29, every supervised load) names the tracking mode
this load runs in and what it costs, so a client learns the step tax from the
response rather than from a timeout: `mode` is `light` (the trackers are
dropped; `/sim/contacts` still answers), `partial` (a `tracking` object
disabled some of them) or `full` (the supervisor walks the scene every basic
step for `contact.*` / `grip.*` / `joint.limit_hit` events and `/sim/grips`).

**Light is the default since 2026-09-02.** A request that names **neither**
`light` **nor** `tracking` runs light, and the block says so:
`default_applied: true`, a `default` sub-block (the same one
`GET /capabilities` → `limits.tracking_default` serves, built by one
function), and `default_note` — one sentence naming how to get the trackers
back. The contract is **dual: an explicit `light` (either value) or any
`tracking` object always wins** over the default, and a `tracking` object with
no `light` is `partial` mode, never the default. `OMNISIM_HARNESS_LIGHT=0` on
the harness restores full tracking as the process-wide default (value-parsed:
unset or `1` → light, `0` → full); the harness names the armed default on its
startup banner, and `default_applied: true` with `mode: "full"` is what a
`=0` process reports. Why it flipped — measured on machine `9722d23d12a3`: on
the 10-Husky `husky_fleet_arena` world (309 nodes, CPU `mj_step`, 2026-08-29)
a full load costs 5.2 s vs 4.65 s (2026-09-02 engine; 12.1 s vs 4.1 s on 2026-08-29) light, `/sim/step 1` 573–606 ms vs 6–35 ms on the 2026-08-29 engine (54 ms vs 23 ms re-measured 2026-09-02 after the controller-probe cache)
(~17×), 10 steps 2855–3187 ms vs 48–67 ms (~47×); on the 10-node cloth world
(2026-08-14) a default reload cost 13.4 s vs 3.1 s light — with the trackers
on, the harness was slower than the `run-headless` it exists to replace, and
an agent that forgot the flag got that path. Before the flip the default was
`full` and the block read `"FULL tracking (the backward-compatible default)"`;
a client written against that MUST now pass `light: false` (all three
trackers) or a `tracking` object (exactly the ones it needs) if it reads
`/sim/grips` or the `contact.*` / `grip.*` / `joint.limit_hit` events. The
first `/sim/grips` read of a session whose GripTracker is not running emits
one `world.warning` (`TRACKER_NOT_RUNNING`, per load) on `/sim/events` naming
whether light was the default or requested. A client MAY ignore the block.

**Response (422 on a load failure; 400 on a malformed request):**

```json
{ "ok": false, "world": "...", "load_ms": 210,
  "diagnostics": [ { "code": "PROTO_NAME_MISMATCH", "path": "...", "line": 12 } ] }
```

Structured diagnostic codes. The authoritative list is
[`scripts/harness/diagnostic_codes.py`](scripts/harness/diagnostic_codes.py)
plus the handful the harness synthesizes itself, and **`GET /capabilities` →
`diagnostic_codes` serves the live enumeration** — trust that over any table
written here. A client MUST treat this enum as open and fall back gracefully on
an unrecognised code, because new codes are added there without a protocol
major bump. The set emitted today (56 codes, regenerated from the source
2026-09-01):

| Group | Codes |
|---|---|
| World file | `WORLD_WRONG_EXTENSION`, `WORLD_FILE_NOT_FOUND`, `WORLD_FILE_EMPTY`, `WORLD_PARSE_INVALID_TOKENS`, `WORLD_PARSE_SYNTAX_ERROR` |
| Header | `HEADER_MISSING`, `HEADER_INVALID` |
| PROTO | `PROTO_RECURSIVE`, `PROTO_BASE_NAME_INVALID`, `PROTO_NAME_MISMATCH`, `PROTO_PARAM_ERROR`, `EXTERNPROTO_DOWNLOAD_FAILED` |
| Fields / nodes | `UNKNOWN_FIELD_IN_NODE`, `WORLDINFO_PHYSICSBACKEND_MISNAMED` |
| Assets | `ASSET_DOWNLOAD_FAILED`, `TEXTURE_READ_FAILED`, `MESH_READ_FAILED`, `URDF_MESH_UNRESOLVED` |
| Newton / physics | `NEWTON_RUNTIME_ABSENT`, `NEWTON_RUNTIME_BROKEN`, `NEWTON_WORLD_NOT_BUILT`, `NEWTON_BODIES_REGISTERED`, `NEWTON_ZERO_DYNAMIC_BODIES`, `NEWTON_STATICS_NOT_REGISTERED`, `NEWTON_ENFORCE_REFUSED`, `NO_PHYSICS_BACKEND`, `NO_STATIC_COLLISION_SURFACE`, `KINEMATIC_ARTICULATION`, `INERTIA_FROM_BOUNDING_OBJECT_UNAVAILABLE`, `CONTACT_PROPERTIES_IGNORED`, `CONTACT_QUERIES_BLIND`, `SOLID_ODE_PIN_INERT`, `RETIRED_ODE_SELECTOR` |
| Joints | `JOINT_FEATURE_UNIMPLEMENTED`, `JOINT_REGISTRATION_FAILED` |
| Sensors | `SENSOR_NO_SOURCE`, `OCCLUSION_RAYS_UNANSWERED` |
| Controller | `CONTROLLER_CRASHED`, `CONTROLLER_EXITED_NONZERO` |
| Platform | `QT_PLATFORM_PLUGIN_FAILED` |
| Harness-synthesized (the engine never got far enough to log) | `LAUNCHER_DLL_NOT_FOUND` (Windows), `SIMULATOR_EXITED_NONZERO`, `SUPERVISOR_BIND_STALLED`, `SUPERVISOR_BIND_CEILING`, `WORLD_DIR_NOT_WRITABLE` |
| CUDA | `CUDA_NOT_AVAILABLE`, `CUDA_DRIVER_TOO_OLD`, `CUDA_COMPUTE_CAPABILITY_TOO_OLD`, `CUDA_DEVICE_INIT_FAILED`, `CUDA_OUT_OF_MEMORY`, `CUDA_KERNEL_LAUNCH_FAILED`, `CUDA_KERNEL_EXECUTION_ERROR`, `CUDA_MEMCPY_FAILED`, `CUDA_GL_INTEROP_NOT_IMPLEMENTED` |
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
Proposed fix: docs/developer/agent-native-api.md (archived 2026-09-02, see [docs/ARCHIVE.md](docs/ARCHIVE.md))
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
that 298-node world — **and is the default since 2026-09-02** (§7.3: a load
naming neither `light` nor `tracking` runs light; `OMNISIM_HARNESS_LIGHT=0`
restores full) — and `GET /capabilities` (§7.28) publishes the
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

**⚠️ As of v9 a step batch STOPS EARLY on the step an armed break fires** — and
`steps_executed` is what actually ran, not the request echoed back (it was the
echo until 2026-09-22). The response gains `steps_requested`,
`stopped_on_break`, `engine_time_ms`, `engine_advanced_ms`,
`requested_advance_ms`, `paused` and `lease_remaining_ms`. That is what makes
`/sim/step` a *continue-to-breakpoint*. Full shape and the two clocks: §7.40 and
§7.39. A step under a held pause (§7.38) is single-stepping, and is exact on the
supervisor clock only.

Proposed fix: docs/developer/agent-native-api.md (archived 2026-09-02, see [docs/ARCHIVE.md](docs/ARCHIVE.md))
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

**⚠️ This 13-field body is the pre-v9 shape, kept because the *reasons* below
still hold.** As of v9 the body also carries `sim_time_ms`,
`basic_time_step_ms`, `sim_time_source`, `engine_time_ms`, **`paused`**,
`lease_remaining_ms`, `break_hit` and `breaks_armed` — documented in **§7.39**,
which also states why `sim_time_ms` and `engine_time_ms` are different rulers.

**There is still no `sim_time` field and no `last_load` object.** Earlier text
for this section listed both; neither has ever been returned. `running` means
only "the engine subprocess is alive" — it is **not** a
`STOPPED / PLAYING / PAUSED` run state; read `paused` for that, and note it is
`null` rather than `false` when nobody could be asked. ⚠️ `paused` describes a
lease **you** hold (§7.38); every read endpoint also pauses the engine
internally for the duration of its own walk, and that guard is invisible here.

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

  "supervisor": { "connected": true, "port": 6990, "light": true, "light_default_applied": true,
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
              "events_limit_default": 256, "events_limit_max": 1024,
              "tracking_default": { "light": true, "mode": "light", "source": "built-in",
                                    "since": "2026-09-02", "why": "...", "revert": "...",
                                    "explicit_wins": "..." } },

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
mutation (one workstream covering both directions).
This is still the DEFAULT behaviour: every successful spawn response carries a `physics_warning`
block — `{"code": "RUNTIME_MUTATION_NOT_IN_SOLVER", "message": ...}` — on all input
forms, and the first spawn per world-load also emits one `world.warning` with the same
code into `/sim/events` (§7.19).
✅ **Since 2026-09-01 there is an OPT-IN fix (W1.7 shipped): pass `{"physics": "rebuild"}`
on this verb (optionally with `rebuild_settle_steps`), or call `POST /sim/rebuild_physics`
(§7.36) after the spawn, and the spawned node IS simulated** — the Newton world is rebuilt
at the scene's current poses in a measured 97–267 ms. The default is deliberately unchanged
(a rebuild drops engaged welds, so it is never applied silently); see §7.36 for the caveats.

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
W1.7 — runtime scene mutation, one workstream covering both directions. This is still the
DEFAULT behaviour: every successful delete response carries the same `physics_warning` block
(`RUNTIME_MUTATION_NOT_IN_SOLVER`), and the first delete per world-load emits one
`world.warning` into `/sim/events` (§7.19). In default mode, reload the world after removing
collidable nodes. ✅ **Since 2026-09-01 there is an OPT-IN fix (W1.7 shipped): pass
`{"physics": "rebuild"}` on this verb (optionally with `rebuild_settle_steps`), or call
`POST /sim/rebuild_physics` (§7.36) after the delete, and the phantom colliders are genuinely
gone** — measured, a deleted floor stops holding bodies up. See §7.36 for cost and caveats.

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

### 7.35 GET /debug/read_bench

Diagnostic: the measured cost of **one supervisor read on this live
session**, free-running vs paused — the number every inspection
endpoint's cost is built from, so an agent (or a bug report) quotes the
session it is actually on instead of a figure measured on another
machine or era.

**Request:** `GET /debug/read_bench?n=100`. `n` (default `50`, clamped
`1`–`1000`) is the number of `getPosition()` round-trips per arm.

**Response:**

```json
{ "n": 100,
  "free_running_ms_per_read": 0.71,
  "paused_ms_per_read": 0.68,
  "pause_taken": true,
  "sim_advance_during_paused_reads_s": 0.0 }
```

The bench picks the first root node whose `getPosition()` answers, times
`n` reads against the free-running engine, then `n` more inside the same
paused-reads guard the inspection endpoints use, and reports whether the
pause actually engaged plus how much sim time advanced while paused.
Results are measured, never echoed. Since the engine's immediate-burst
fast path (2026-09-01, `OMNISIM_IMMEDIATE_BURST`, `=0` reverts) the two
arms land in the same ~0.6–0.9 ms band on the 309-node fleet arena —
reads are no longer pause-dependent; the pause survives because it buys
a consistent single-instant snapshot, not speed.

### 7.36 POST /sim/rebuild_physics

**W1.7 — mid-run physics rebuild (2026-09-01, engine commit
`88487d988`).** Tears down the live Newton world and re-registers the
WHOLE scene at its **current** poses, so runtime-spawned nodes (§7.29)
gain physics and runtime-deleted ones (§7.30) lose their phantom
colliders — the opt-in fix for the frozen-model defect both banners
describe. Live velocities are replayed and motor targets re-pushed, so a
running robot keeps driving (measured: an 8-Husky motorised world drove
through a mid-run rebuild at unchanged speed). Also reachable as
`{"physics": "rebuild"}` directly on `/scene/spawn` / `/scene/delete`
(optionally with `rebuild_settle_steps`), which chains the rebuild onto
the mutation in one call and replaces the `physics_warning` block with a
`physics: {"mode": "rebuild", ...}` block in that response.

**Request:** `{ "settle_steps": 8 }` (default `8`, clamped `1`–`1024`).

**Response (200):**

```json
{ "ok": true, "requested": true, "settle_steps": 8,
  "advanced_to_ms": 1234.0 }
```

**Refusal (`409` + `REBUILD_REFUSED`):** on a Cloth / SoftBody /
GranularBed world the engine refuses the rebuild — those systems
re-register from *authored* state, so a rebuild would teleport them;
reload the world instead. The engine logs
`physics rebuild REFUSED: <reason>` as a warning
(`OmSupervisorUtilities.cpp`), and the harness watches the settle window
for that line and surfaces it, so the caller does not have to poll
`/sim/events`:

```json
{ "ok": false, "code": "REBUILD_REFUSED", "requested": true,
  "settle_steps": 8, "advanced_to_ms": 1234.0,
  "error": "... physics rebuild REFUSED: <the engine's reason>" }
```

Caveats, all deliberate:

- **Engaged `Connector` / `VacuumGripper` welds are DROPPED**, with a
  loud engine warning naming the count. Do not rebuild mid-grasp;
  re-lock from the controller afterwards.
- **Measured cost: 97–267 ms** (machine `9722d23d12a3`, CPU `mj_step`;
  an in-process `SolverMuJoCo` reconstruction), plus the settle steps.
- **Bitwise step-for-step continuation across a rebuild is NOT
  claimed** — a fresh solver is fresh state. What IS measured: a spawned
  box frozen at z = 1.5 landed at **0.599892258644104** after a rebuild,
  bit-identical to the authored control's rest height, and a deleted
  floor genuinely stopped colliding.
- The verb needs a current libController
  (`wb_supervisor_simulation_rebuild_physics` /
  `Supervisor.simulationRebuildPhysics`); an older build gets a coded
  error telling it to rebuild (`python -m omnisim doctor` checks the
  engine↔libController pair).

The DEFAULT `/scene/spawn` / `/scene/delete` behaviour is unchanged:
without the opt-in, their responses still carry `physics_warning`
(`RUNTIME_MUTATION_NOT_IN_SOLVER`).

### 7.37 GET /scene/node/&lt;def&gt;/particles

**Particle-state readback (2026-09-01,
`C_SUPERVISOR_NODE_PARTICLE_STATS`).** The deformable/granular systems
(`Cloth`, `SoftBody`, `GranularBed`, `GranularGroup`) were, until this
route, unobservable from the harness: their surfaces are engine-owned
particle arrays with no supervisor accessor, so a controller could drive
a gripper into a sheet and have no way to tell a grasp from a miss.
This route is **stats-first, sample-optional**: the default answer is an
80-byte aggregate computed engine-side (one FFI crossing), never the
whole cloud — a 100k-particle bed costs the same as a 441-particle
sheet. A **PURE READ** off the engine's per-step particle caches:
nothing in the scene moves, and the RPC is in the harness's
idempotent-retry set.

**Request:** `GET /scene/node/SHEET/particles?sample=25`. `sample`
(default `0` = stats only, clamped `0`–`4096`) returns every
`sample`-th particle's world xyz in addition to the stats.

**Response (200):**

```json
{ "def": "SHEET", "status": 0, "count": 441,
  "min": [-0.55, -0.52, 0.02], "max": [0.55, 0.53, 0.31],
  "centroid": [0.01, 0.00, 0.12],
  "non_finite": 0,
  "sample_stride": 25, "sampled": 18,
  "sample": [[-0.55, -0.52, 0.02], ...],
  "verification": { "semantics": "PURE READ ..." },
  "rpc_ms": 4.2 }
```

Semantics, all deliberate:

- `min` / `max` / `centroid` are world-frame aggregates over the
  **FINITE** particles only; `non_finite` counts particles carrying a
  NaN/Inf component (counted, never propagated — a diverging cloth
  reads as a **rising `non_finite`**, not a NaN centroid, which is the
  failure mode this stack actually exhibits). All three read `0.0` when
  no particle is finite.
- `sample` is raw (non-finite values included) so a caller can locate
  the divergence, and `sampled`/`sample_stride` are echoed measured,
  not assumed.
- The stats are served off the same per-step particle cache the render
  readback uses (one GPU→CPU transfer per tick, shared); the
  `GranularGroup` arm aggregates the CUDA demo's host buffer instead —
  that node's particles are not in the Newton arrays.

**Errors** (via the supervisor's coded classifier): `404` +
`DEF_NOT_FOUND`; `400` + `ARGUMENT_INVALID` for a non-particle node
(engine status `-2`), a node that never registered with the particle
solver or a missing Newton runtime (`-1`), an inert `GranularGroup`
with no CUDA state (`-5`, honest inert — there is no particle state to
read), or a bad `sample` stride; `503` when the supervisor link is
down. The verb needs a current libController
(`wb_supervisor_node_get_particle_stats` /
`Node.getParticleStats`); an older build gets a coded error telling it
to rebuild (`python -m omnisim doctor` checks the engine↔libController
pair).

### 7.38 POST /sim/pause, POST /sim/resume

**Hold the engine across calls.** Shipped 2026-09-15 (`0aafab096`) and
undocumented until v9; read §7.40's latency note before depending on it.

Every other read endpoint pauses the engine *internally*, for the duration of
its own walk, and unpauses before it answers — so each response is one
consistent instant, but the scene moves again the moment you get it (~88–112 ms
of sim time per idle poll). `POST /sim/pause` is the guard you hold yourself.

**Request (optional body):** `{ "lease_ms": int }`

| Field | Type | Default | Notes |
|---|---|---|---|
| `lease_ms` | int | `30000` | Clamped to **[1000, 300000]** silently. A non-integer is `400 BAD_REQUEST`. |

**⚠️ The lease is a deadline, and it is a safety property, not a convenience.**
A client that pauses and then dies — crashed agent, closed laptop, dropped
ssh — would otherwise freeze the simulation with no way back but killing the
engine. The lease self-expires and the main loop resumes on its own.

**Response (200) — the complete body, all 5 fields:**

```json
{ "paused": true,
  "lease_remaining_ms": 60000,
  "paused_at_sim_ms": 48.0,
  "lease_ms": 60000,
  "rpc_ms": 5.65 }
```

`lease_ms` is the **effective** (clamped) lease, not the requested one.

**A second `/sim/pause` while held EXTENDS the lease** rather than erroring, and
does not re-capture the pre-pause mode or move `paused_at_sim_ms`. Measured
2026-09-22 (machine `9722d23d12a3`): `lease_remaining_ms` 116233 →
`{"lease_ms": 200000}` → 200000, `paused_at_sim_ms` unchanged at 2016.0.

**`POST /sim/resume` — request:** empty body. **Response (200), all 4 fields:**

```json
{ "paused": false, "was_paused": true, "released_by": "resume", "rpc_ms": 15.57 }
```

Resume is **idempotent**: a second call returns `was_paused: false` and is not
an error. `released_by` is `"resume"` on this path; the internal expiry and
world-load paths use `"lease_expired"` and `"world_load"`, which you observe as
`paused` flipping to `false` on `/sim/state` rather than as a response body.

**`POST /sim/step` while held is the point of the feature** (§7.10): the step
handler lifts the pause, steps, and re-takes it inside its own RPC, then
re-holds. ⚠️ **The step is exact on the supervisor clock and NOT on the engine
clock.** Measured on the three-body `break_drop` fixture,
`basicTimeStep` 8 ms: `{"steps": 10}` advanced the supervisor clock by
**exactly 80 ms** on every call, and the engine clock by **80, 88, 104 or
112 ms** — 0 to 4 basic steps of overshoot — because lifting the pause,
stepping and re-queuing the pause is a race the supervisor binding cannot
close. That is why the response **reports** `engine_advanced_ms` rather than
asserting it. Do not build a test that asserts an exact engine advance.

**A held lease does not cross a `POST /world/load`.** `worldLoad` usually
terminates the supervisor, but measured 2026-09-22 a second load of the *same*
world came back with the process still alive; the supervisor therefore releases
the lease, clears every armed break and drops `break_hit` explicitly on a
`world_load`. Without that, a session inherited a lease held over the *new*
world (frozen at t = 8 ms with nothing in the new session to explain why) and a
`break_hit` describing a break in the *old* one.

**Errors:** `400 BAD_REQUEST` (`lease_ms must be an integer`);
`503` when the supervisor link is down. There is no "already paused" error.

### 7.39 GET /sim/state — the pause and break fields

§7.12's "there is no `paused` field" is **no longer true**. As of v9 the clock
block carries five more fields:

| Field | Type | Meaning |
|---|---|---|
| `paused` | bool \| **null** | `true` = a lease is held and `sim_time_ms` is legitimately standing still. `false` = measured free-running. **`null`, not `false`,** when nobody could be asked — no supervisor, or a load in flight. |
| `lease_remaining_ms` | int \| null | Milliseconds left on the held lease; `0` when not held. |
| `break_hit` | object \| null | The last break that froze the engine, in the §7.40 payload shape plus an `event` object carrying the matched event verbatim. **Survives the resume**, so it is also the record of *why* it was frozen. |
| `breaks_armed` | int \| null | How many breaks are armed right now. |
| `engine_time_ms` | float \| null | The **engine's own** clock as of that controller's last step. |

**⚠️ `sim_time_ms` and `engine_time_ms` are DIFFERENT RULERS, and the
difference is the whole honest story about break latency.** `sim_time_ms` is
the injected supervisor's per-iteration counter — it is what stamps every other
harness response, so it is the one those numbers are comparable to.
`engine_time_ms` is the engine's clock. In `--mode=fast` the engine free-runs
independently of the controller loop and runs far ahead of it: measured
2026-09-22 on a three-body world, a `/sim/reset` returned with the boxes
re-lifted at supervisor t = 8 ms and the very next tracker poll — **one
supervisor tick later**, supervisor t = 16 ms — already saw both boxes at rest
on the floor. That is **about 700 engine-ms inside one supervisor tick**.

Quote `engine_time_ms` for "how much simulated time has passed". ⚠️ It does
**not** rewind on `POST /sim/reset`.

A reader who sees `sim_time_ms` standing still needs `paused` in the *same*
response, or the honest reading of a frozen clock is "the simulator hung".

`POST /sim/step`'s response gains four fields for the same reason (§7.40):
`steps_executed`, `steps_requested`, `stopped_on_break`, `engine_advanced_ms`,
alongside `engine_time_ms` and `requested_advance_ms`.
⚠️ **`steps_executed` used to be the request echoed back.** It is now what the
supervisor actually ran.

### 7.40 POST /sim/break, GET /sim/breaks, DELETE /sim/break/&lt;break_id&gt;

**Break on event.** Arm a condition; the supervisor takes the pause lease the
moment a matching event is emitted, so the scene you then read is the scene *at*
the moment of interest rather than the scene ~90 ms of free-running later. It is
the breakpoint this surface was missing, built on §7.38.

#### Request

```json
{ "types": ["contact.began"],
  "filter": { "def": "BREAK_BOX", "counterpart": "BREAK_FLOOR" },
  "lease_ms": 60000,
  "once": true }
```

| Field | Type | Default | Notes |
|---|---|---|---|
| `types` | list[str] | **required** | Non-empty. Duplicates are collapsed. Every entry must be breakable **in this session** or the whole request is refused. |
| `filter` | object | `{}` | Keys: `def`, `counterpart`, `joint` — **and no others**. All present keys must match (AND). |
| `lease_ms` | int | `30000` | The lease the *hit* takes. Clamped to [1000, 300000] with a `lease_ms_clamped` diagnostic. |
| `once` | bool | `true` | `true` disarms the break after its first hit (`still_armed: false`); it stays listed with `hits: 1`. |

**Breakable types** are the supervisor's own, minus `break.hit` itself, minus
whatever this session silences: `contact.began`, `contact.ended`,
`joint.limit_hit`, `grip.acquired`, `grip.released`, `damage.impact`,
`damage.state_transition`. At most **32** breaks may be armed at once.

**Filter matching is per type.** Only `contact.*` (`a_def` / `b_def`) and
`grip.*` (`gripper_def` / `held_def`) carry a DEF identity, so `def` /
`counterpart` can only narrow those; `joint` narrows `joint.limit_hit` only.
A filter key no armed type carries does **not** refuse the break — it earns a
non-fatal `filter_key_not_matchable_for_armed_types` diagnostic and the break
fires on any event of the armed types, because failing to *narrow* is the
opposite surprise from failing to *fire*.

A break never fires on an event that happened before it was armed: arming
fast-forwards the scan cursor to the current end of the bus.

#### Response (200) — the complete body, all 13 fields

```json
{ "break_id": "brk1",
  "types": ["contact.began"],
  "filter": { "def": "BREAK_BOX", "counterpart": "BREAK_FLOOR" },
  "once": true,
  "armed": true,
  "hits": 0,
  "lease_ms": 60000,
  "armed_at_sim_ms": 8.0,
  "last_hit_sim_ms": null,
  "armed_types": ["contact.began"],
  "diagnostics": [],
  "rpc_ms": 0.09,
  "ok": true }
```

#### ⚠️ Refusals are the headline behaviour, not an inconvenience

**Light mode is the default** (§7.3) and silences 5 of the 11 event types. A
break armed on a silenced type could never fire, and accepting it would hand you
a breakpoint that simply never trips — **indistinguishable from "the bug did not
happen"**. So it is refused, with the producer named:

```json
{ "ok": false,
  "code": "BREAK_EVENT_TYPE_UNAVAILABLE",
  "error": "refused: a break on contact.began could never fire in this session (event_type_silenced_in_light_mode)",
  "diagnostics": [
    { "code": "event_type_silenced_in_light_mode",
      "event_type": "contact.began",
      "producer": "ContactTracker",
      "detail": "this session runs with ContactTracker, JointLimitTracker, GripTracker not constructed (--light, or a per-tracker flag), so no contact.began is ever emitted and a break armed on it could never fire",
      "workaround": "reload the world with light disabled -- POST /world/load with {\"light\": false} for all three trackers, or a `tracking` object naming just the ones you need -- then arm the break again",
      "silenced_types": ["contact.began", "contact.ended", "grip.acquired", "grip.released", "joint.limit_hit"] } ],
  "armed_types": [] }
```

**The refusal is SCOPED, not blanket.** `damage.impact` and
`damage.state_transition` survive light mode, and a break on one is **accepted
in the same session** that refused the `contact.began` above — verified live,
2026-09-22. **A refused break is not armed**: `armed_types` is `[]` and
`GET /sim/breaks` does not list it.

| Status | `code` | When |
|---|---|---|
| 400 | `BREAK_TYPES_REQUIRED` | `types` missing, not a list, empty, or an entry is not a non-empty string |
| 400 | `BREAK_EVENT_TYPE_UNAVAILABLE` | any entry could never fire here — see the four `diagnostics[].code`s below |
| 400 | `BREAK_FILTER_INVALID` | `filter` is not an object, an unknown key, or a non-string value |
| 400 | `BREAK_LEASE_INVALID` | `lease_ms` is not an integer |
| 409 | `BREAK_LIMIT_REACHED` | 32 breaks already armed |
| 400 | `BREAK_REFUSED` | fallback when the registry names no code |
| 400 | `BAD_JSON` / `BAD_REQUEST` | unparseable body, or a body that is not an object |
| 404 | `BREAK_NOT_FOUND` | `DELETE` / `POST /sim/break/delete` for an id that is not armed |
| 503 | — | supervisor link down |

`diagnostics[].code` is a **closed enum**, published on `GET /capabilities`
under `supervisor.breaks.diagnostic_codes`:
`types_missing`, `event_type_unknown`, `event_type_not_breakable`,
`event_type_not_on_supervisor_bus`, `event_type_silenced_in_light_mode`,
`filter_key_unknown`, `filter_key_not_matchable_for_armed_types`,
`break_limit_reached`, `lease_ms_clamped`. The first five are fatal; the last
four ride along on a 200 (except `break_limit_reached`).

Two of them exist because a type can be *declared* and still unreachable:
`controller.log` / `world.warning` / `world.error` come from the **harness's**
log ring buffer reading the engine's stdout, never from the supervisor bus, so
they are `event_type_not_on_supervisor_bus`; and `break.hit` is
`event_type_not_breakable` because arming a break on it would re-trigger on its
own record.

#### GET /sim/breaks

**Response (200), all 7 fields:**

```json
{ "breaks": [ { "break_id": "brk1", "types": ["contact.began"],
                "filter": { "def": "BREAK_BOX", "counterpart": "BREAK_FLOOR" },
                "once": true, "armed": false, "hits": 1, "lease_ms": 60000,
                "armed_at_sim_ms": 8.0, "last_hit_sim_ms": 904.0 } ],
  "breakable_types": ["contact.began", "contact.ended", "joint.limit_hit",
                      "grip.acquired", "grip.released", "damage.impact",
                      "damage.state_transition"],
  "silenced_types": [],
  "break_hit": { "...": "the §7.40 hit payload, plus `event`" },
  "paused": true,
  "rpc_ms": 0.06,
  "ok": true }
```

In a light session the same call reports `breakable_types` as
`["damage.impact", "damage.state_transition"]` and `silenced_types` as the
five silenced ones — **read it before arming**, rather than discovering the
refusal.

#### DELETE /sim/break/&lt;break_id&gt;

**Response (200):** `{ "break_id": "brk1", "removed": true, "rpc_ms": 0.02, "ok": true }`.
**404:** `{ "ok": false, "code": "BREAK_NOT_FOUND", "error": "no armed break with id 'brk1'", "break_id": "brk1", "removed": false }`.

**`POST /sim/break/delete {"break_id": "brk1"}` is an exact twin**, for clients
whose HTTP layer cannot route a bodyless `DELETE`. Both shipped; same 200 body,
same 404 body. It is not a fallback guess — pick whichever your client routes.

#### The hit: `break.hit` on /sim/events, and `/sim/step` stopping early

When a break fires, the supervisor takes the lease **first** — everything after
that runs against a frozen engine — then records the hit and emits `break.hit`
through the same `emit()` call form the capability self-scan reads, so it is the
**eleventh** code-verified event type (§10) and the `declared_not_emitted` drift
check stays green.

```json
{ "seq": 2, "type": "break.hit", "t_sim_ms": 904,
  "break_id": "brk1",
  "matched_type": "contact.began",
  "matched_seq": 1,
  "matched": { "t_sim_ms": 904, "a_def": "BREAK_BOX", "b_def": "BREAK_FLOOR",
               "point": [-0.5000000074505806, -0.10000000149011612, -0.02212010648846298] },
  "paused_at_sim_ms": 904.0,
  "hold_latency_ms": 0.0,
  "hold_latency_steps": 0,
  "engine_time_ms_at_hold": 1008.0,
  "engine_tick_ms": 8.0,
  "hold_latency_engine_ms_max": 8.0,
  "paused": true,
  "lease_ms": 60000,
  "lease_remaining_ms": 60000,
  "once": true,
  "still_armed": false,
  "hits": 1,
  "source": "sup" }
```

`matched` is the matched event minus its `seq` and `type`. A hold that failed
adds `lease_error`; `paused: false` on a hit means the freeze did not take.

**`POST /sim/step` stops early on the step a break fires**, which is what makes
it *continue-to-breakpoint*:

```json
{ "sim_time_ms": 904.0, "advanced_to_ms": 904.0,
  "steps_executed": 112, "steps_requested": 400,
  "stopped_on_break": "brk1",
  "engine_time_ms": 1008.0, "engine_advanced_ms": 992.0,
  "requested_advance_ms": 3200,
  "paused": true, "lease_remaining_ms": 60000,
  "rpc_ms": 5535.02, "wall_ms": 5535 }
```

`stopped_on_break` is the `break_id`, or `null` when the batch ran to
completion. ⚠️ Never read `steps_requested` as what happened.

#### ⚠️ HONEST LATENCY: two regimes, and neither is sub-step

The hold is taken on the supervisor **tick** that scans the bus, never inside
the solver sub-step. There are two regimes and they are two orders of magnitude
apart:

| Regime | Detection granularity | Measured (machine `9722d23d12a3`, `basicTimeStep` 8 ms, three-body `break_drop`) |
|---|---|---|
| **Held, driven by `/sim/step`** | per **basic step** | `hold_latency_ms` **0.0**, `hold_latency_steps` **0**, `hold_latency_engine_ms_max` **8.0** — at most one basic step of engine time |
| **Free-running** | one **supervisor tick** | a tick is **not** a basic step: `engine_tick_ms` measured at 8, 16, 48 and 128 ms across the live cases, and a loaded full-tracking step costs ~600 ms |

**And free-running it can miss the event entirely.** On the three-body fixture
the engine outran the controller so far that a whole **one-second drop fitted
inside one supervisor tick**: the live case armed a `contact.began` break, let
the world free-run for 20 s, and recorded `fired: false` — no `contact.began`
was emitted at all, because the tracker polls once per tick and the box was
already at rest before the first poll after the reset.

**The reliable workflow is the debugger one: pause, then step.**

```
POST /sim/pause  {"lease_ms": 60000}
POST /sim/reset  {"restore": "__init__"}
POST /sim/break  {"types": ["contact.began"], "filter": {"def": "BOX"}}
POST /sim/step   {"steps": 400}      -> steps_executed 118, stopped_on_break "brk2"
GET  /scene/tree                      # a STILL scene
POST /sim/step   {"steps": 10}        # single-step forward
POST /sim/resume
```

Every hit carries its **own measured** `hold_latency_ms`
(`paused_at_sim_ms − event.t_sim_ms`, on the supervisor clock, so ~0 by
construction and flattering on its own) and `hold_latency_engine_ms_max`
(`engine_tick_ms`, the engine-clock width of the tick detection happened on,
which is the number that bounds the real thing). **Read the numbers; do not
assume sub-step precision.**

#### History: which releases "the pause worked" spans

`0aafab096` (2026-09-15) shipped the primitive with no test, no documentation
and no MCP tool, and **it did not work**. Seven defects, all fixed 2026-09-22
and now covered by 43 engine-free tests plus four live cases
(`tests/harness/test_break_on_event.py`, `test_break_integration.py`):

1. `simulationSetMode` only **queues** — a held pause has no round trip to carry
   it, so the supervisor reported a frozen clock over a still-moving engine. A
   break fired at t = 168 ms and the engine ran on to ~400 ms, ~29 basic steps
   of scene motion after the "freeze". A witness body in free fall is the ruler
   that caught it. Fixed with an explicit flush.
2. The same queueing made `/sim/step {"steps": 1}` come back having moved the
   scene by two steps' worth of fall; `pause_lifted` now flushes its re-pause.
3. A lease expiry could hang the supervisor **permanently** — the un-pause was
   left for the loop's own `supervisor.step()`, which blocks against a paused
   engine, so engine and controller each waited on the other. Measured: a 1 s
   lease expired and the supervisor never answered again, turning the safety
   property into the thing that broke it.
4. A held lease **survived a `/world/load`** (see §7.38).
5. So did `break_hit` and the armed break ids.
6. Break detection lagged up to **17 basic steps** (`hold_latency_steps: 17`).
7. A single-stepped session stamped every hit with a stale engine clock — a hit
   carrying `engine_time_ms_at_hold: 196000` on a scene whose engine clock was
   1016 — because the main loop skips its bookkeeping while a lease is held.

"The pause exists" was true from 2026-09-15. "The pause worked" is true from
2026-09-22. Do not conflate them when reading an older release.

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

Events flow through **two** streams, and §10.1 below is the **harness's**
list. The bridge-level stream promised here as "a v1.1 addition" shipped in
v9 as `GET /events` (§5.9); its type list, its detectors and its blind
spots are in that section, and its events carry a different timestamp
(`sim_time`, float sim seconds, plus `step`) from the ones described here.
Do not mix the two cursors.

On `GET /sim/events` (§7.19) each event carries:

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

**Exactly eleven types are emitted on the harness stream** (ten until v9;
`break.hit` is the eleventh). The bridge stream (§5.9) has its own list of
eight measured types plus six it forwards from here; a name from one list
is not valid on the other. `GET /sim/events?types=` is an
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
| `break.hit` | `sup` | `event_bus.py` · `BreakRegistry._fire` (v9, §7.40) |
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

### 10.7a Break (v9)

- `break.hit { break_id, matched_type, matched_seq, matched, paused_at_sim_ms,
  hold_latency_ms, hold_latency_steps, engine_time_ms_at_hold, engine_tick_ms,
  hold_latency_engine_ms_max, paused, lease_ms, lease_remaining_ms, once,
  still_armed, hits }`

Emitted on `source: "sup"` when an armed break (§7.40) takes the pause lease.
It is a **record of a freeze**, not a physics event: `matched` carries the
underlying event verbatim (minus its own `seq` and `type`), and `matched_seq`
points back at it on the same cursor.

⚠️ **A break cannot be armed on `break.hit`** (`event_type_not_breakable`) — it
would re-trigger on its own record. ⚠️ **Read `hold_latency_engine_ms_max`, not
`hold_latency_ms`**: the latter is measured on the supervisor's own counter and
is ~0 by construction. Neither is sub-step. §7.40 has the two latency regimes.

### 10.8 Faults (bridge-emitted) *(these two names reserved — not emitted)*

- `fault.raised { robot_id, code, message }` — see §11.
- `fault.cleared { robot_id, code }`

⚠️ **No service in this repository emits either of these two names**, and
they stay reserved. What changed in v9 is the surface, not the names:
there **is** a bridge-level event stream now (§5.9), and it emits two
differently-spelled fault types of its own —
`fault.controller_lost { code, source_field }`, the rising edge on the
bridge's own `fault` field, and
`fault.telemetry_stale { age_s, budget_s, note }`, meaning the simulation
tick has stopped so every reading the bridge serves is at least that old.
Neither is `fault.raised`, there is **no** `cleared` counterpart (the
detector re-arms silently), and neither carries `robot_id` inside `detail`
— the envelope's own `robot` field carries it. Bridges continue to surface
current fault state as the `fault` object inside `GET /state` (§5.3); the
event is the edge, the state field is the level.

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
| `refused_by_gate` | The safety gate declined to dispatch a command. **Nothing was actuated.** See below. |
| `tool_not_registered` | `POST /tool` named a tool this bridge does not serve, or no tool registry is attached yet (§5.8.2). |
| `tool_execution_failed` | A `POST /tool` dispatch raised (§5.8.2). |
| `proto_not_found` | A referenced PROTO definition could not be located. |
| `world_parse_syntax_error` | The `.wbt` file failed to parse. |
| `world_load_timeout` | The world did not become ready within `wait_s`. |

### 11.1 `refused_by_gate` — the details shape

`refused_by_gate` is the one code in this table that means **"understood,
and deliberately not done"**. It is neither a failure nor a success, and a
caller MUST NOT treat it as either: retrying the same arguments will be
refused again, and reporting it as a completed action is the fabrication
§5.4.1 exists to forbid.

It uses the §3.2 envelope, with `400 Bad Request`:

```json
{
  "ok": false,
  "error": "refused_by_gate",
  "message": "<rule>: <detail>",
  "details": { "tool": "drive_forward", "rule": "implausible" }
}
```

- `message` — `"<rule>: <detail>"`. The part before the first colon is a
  rule name from the enumeration in §5.8.3 and is the machine-readable
  half; the detail after it is prose and MAY change between releases. A
  client SHOULD parse the rule out of `message` rather than matching the
  whole string, and MUST tolerate a `message` with **no** rule prefix: the
  fail-closed path (§5.8.3) reports `"safety gate unavailable (<Error>)"`
  or `"safety gate errored (<Error>)"`, which is a refusal with no rule
  behind it because the rules could not be reached.
- `details.rule` (string) — ⭐ **v9, additive; `message` is unchanged.**
  The rule name as its own field, so a caller does not have to re-parse
  prose — the same problem a reader of `gate.refused` (§5.9) has, solved
  the same way and by the same parser. It is **`gate_unavailable`**, never
  a guessed member of the enumeration, when the reason has no parsable
  rule head — which is exactly what the fail-closed path above produces.
  It is emitted on the `/tool` path **and**, since 2026-09-22, on the
  flying bridge's `POST /action`. Bridges older than that send no
  `details.rule` at all, so a client MUST tolerate its absence and fall
  back to parsing `message`.
- `details.tool` (string) — the tool that was refused. Emitted by
  `bridge_base` and by the mobile, arm, quadruped and flying `/tool`
  handlers, and by the flying bridge's `POST /action` (which reports the
  gate tool its action verb was mapped onto).
- `details.action` (string) — emitted **in addition to** `details.tool` by
  the flying bridge's `POST /action` (§5.4), because that path refuses on
  the action verb and the caller sent the verb, not the tool name. A
  client MUST read whichever of the two is present; older flying bridges
  send `details.action` alone.

⚠️ **Where this code does and does not appear.** It is raised on
`POST /tool` (§5.8) and on the flying bridge's `POST /action` (§5.4), and
nowhere else. `POST /prompt` is gated too, but a refusal there is **not**
an error envelope: the call succeeds with `200` and the refusal appears in
`actions[]` as `result: "refused"` (§5.7.2), because the bridge still has
a sentence to reply to. And the **absence** of this code says nothing
about whether a request was checked: the direct REST actuators (§5.4,
§5.5, §5.6, §6) never raise it because nothing checks them.

✅ **The flying bridge's `POST /action` used to carry neither** — the
envelope had `details.action` alone, no `details.rule` and **no
`gate.refused` event**, because it refuses before reaching the shared
`/tool` handler that files both. Closed 2026-09-22: it still refuses
early (the verb must be mapped onto a gate tool first), so it now calls
the same two shared producers itself and emits `gate.refused` with
`origin: "action"`. A client talking to an older flying bridge still sees
the refusal with neither.

A refusal on `POST /tool`, on the flying bridge's `POST /action`, and one
on the `/prompt` **parser** path, is
**also** filed on the bridge's own event stream as `gate.refused` (§5.9),
with `detail` carrying `tool`, `reason`, `origin` (the surface the call
arrived on), a truncated `utterance` and a `rule`. That exists because a
400 is seen only by the caller: the robot's own agent would otherwise
never learn that something was refused on its behalf.

The event's `detail.rule` is produced by the **same** parser as
`details.rule` above, with the same `gate_unavailable` fallback, spelled
identically in both producers — a consumer branching on the rule cannot
tell which of them wrote the record and must not have to.

Two precise properties of that event, both worth knowing before building
on it:

- **`detail.rule` is always a real rule name, never an empty string.**
  Both producers parse it out of the same `"<rule>: <detail>"` reason
  string and both fall back to **`gate_unavailable`** — spelled
  identically in each — when the reason carries no parsable rule head, as
  the fail-closed wrapper's `"safety gate unavailable (ImportError)"` does.
  That fallback is deliberate: the enumeration is open and a client must
  tolerate an unknown name, but it must never be told `interrogative` when
  the truth is that nothing was checked. ⚠️ An empty rule was the earlier
  behaviour on the `/tool` origin and was worse than a missing one — it
  looks populated to a schema check while being unusable for the automated
  reaction the field exists to enable. Measured on a live Husky: a
  correctly refused 300 m drive filed `rule: ""` beside
  `reason: "implausible: distance=300 …"`.
- **A refusal of a MODEL's tool call inside a relay turn is not filed on
  the ring at all.** It is reported in that turn's `actions[]` as
  `result: "refused"` with its `rule` (§5.7.2), which the agent is already
  reading; duplicating it onto the ring would buy a second model turn to
  tell the agent something it just said.

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
| `1.0.0` – `1.0.10` | `1.0` |
| `2.0.0`            | `1.0` – `1.1` (this document; the world harness speaks `1.1` since 2026-07-26, the robot bridges `1.0`) |

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

- The endpoint URLs in §5.1–§5.9, §6, §7, §8.
- Required request fields and their semantics.
- The error envelope shape (§3.2).
- The reserved fault codes (§11).
- The event type names and their required fields (§10 for the harness
  stream, §5.9 for the bridge stream). ⚠️ **Which bridge emits which type
  is NOT stable** and is not a promise: it follows the detectors a given
  robot class can honestly run, and the per-class table in §5.9 is a
  statement about the shipped tree on its date.

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
| Robot Bridge — flying | [projects/samples/demos/controllers/mavic_omnilink_bridge/](projects/samples/demos/controllers/mavic_omnilink_bridge/) | Port `6090`. Mavic 2 Pro with gimbal camera + marker perception. Implements typed `POST /action`, and since 2026-09-21 also `POST /prompt` (§5.7) and `POST /tool` (§5.8). **All three are gated** — `/prompt` through the shared parser with `surface="drone"`, `/tool` directly, and `/action` by mapping `takeoff` / `land` / `hover` / `set_yaw` / `stop` / `reset` onto gate verbs (an unmapped action is passed through, not refused). Its `/tool` registry is built in-process from the bridge's own `act_*` methods rather than from a relay, so the drone is gated without a platform key; it is also the only bridge that accepts an `utterance` alongside a tool call, which is what lets the four intent rules fire there. Still missing `/list_robots`, `/get_robot_state`, `/stop_robot`, `/reset_to_home`, and the §6.3 routes remain `/action` verbs. |
| Robot Bridge — quadruped | [projects/samples/demos/controllers/omnilink_quadruped_bridge/](projects/samples/demos/controllers/omnilink_quadruped_bridge/) | Port `8765`. OmniQuad. Motions: `stand`, `sit`, `crouch`, `settle`, `wave`, `walk`, `stop`, `home` — driven through `/tool` / `/prompt`, not dedicated routes. ⚠️ Its `walk` is a **wave-gait leg cycle with supervisor-driven body translation**, i.e. scripted, not physically-actuated locomotion. Learned OmniQuad locomotion lives in `projects/policies/`, not in this bridge. |
| World Harness | [scripts/harness/omnisim_harness.py](scripts/harness/omnisim_harness.py) | Port `6789` + supervisor `6790`. |
| Capture Service | [scripts/capture/omnisim_capture.py](scripts/capture/omnisim_capture.py) | Port `6791` + supervisor `6792`. |

**All four controller bridges, and `bridge_base`, additionally serve
`GET /events` (§5.9) and publish `safety_gate` (§5.2.1) as of 2026-09-22**,
and all five report `last_tick_at` in sim seconds with the wall clock in
`wall_time` (§5.3). The per-bridge extent of the other v9 fields is in
§5.4.1 and in the gap list below; it is deliberately not uniform.

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
- ~~**No `paused` field, and no way to pause, resume, or quit**~~ — **pause,
  resume, single-step and break-on-event shipped in v9** (§7.38–§7.40, live
  2026-09-22). `GET /sim/state` reports `paused`, `lease_remaining_ms`,
  `break_hit`, `breaks_armed` and `engine_time_ms`; `POST /sim/step` stops early
  on a break and reports `steps_executed` + `stopped_on_break`. ⚠️ Two caveats
  that are **not** gaps but must be quoted with the feature: break detection is
  **never sub-step** (held, one basic step of engine time; free-running, one
  supervisor tick, 8 ms to ~600 ms of engine time — and a transient can be
  missed entirely), and a break armed on a type this session silences is
  **refused**, not accepted. There is still no `quit`.
- **No watch conditions** (a polled predicate over pose or joint state).
  Declared on `GET /capabilities` under `not_supported` as `sim.watch` /
  `WATCH_NOT_IMPLEMENTED`, with the workaround: break on the nearest event type,
  or hold the pause and poll the predicate yourself against a scene that is not
  moving underneath you. Out of scope for v9 by owner decision.
- **No record, replay or run-diff**, and they stay blocked on the checkpoint
  gap: `POST /sim/snapshot` (§7.32) saves **poses and joint angles only**.
  Velocity is never captured and solver state is never touched, so a restore
  teleports bodies to saved poses while they keep their live Newton velocities,
  and it does **not** rewind the clock. Identical forward evolution is not
  achievable here and must never be claimed.
- **No spawn / delete / set-entity-state.** An agent composing a scene
  must hand-author `.wbt` text and pay a full world load per mistake. The
  shipped Supervisor binding already has
  `Field.importMFNodeFromString()`, `Node.remove()`, `Node.setVelocity()`
  and `Supervisor.worldSave()`; the harness supervisor exposes none of
  them, and its only field write in the entire scene is
  `Viewpoint.position` / `.orientation`. Proposed: same doc, G4 / P3.
- ~~**§10.8 (`fault.*`) and §10.9 (`shadow.*`) are reserved names with no
  producer, and there is no bridge-level event stream**~~ — **half fixed
  2026-09-22.** The bridge-level stream shipped as `GET /events` (§5.9) on
  all five reference implementations, and it emits
  `fault.controller_lost` / `fault.telemetry_stale`. ⚠️ The two §10.8
  names themselves (`fault.raised`, `fault.cleared`) still have **no
  producer**, there is no `cleared` counterpart to either shipped fault
  type, and §10.9 (`shadow.*`) remains entirely unimplemented with §9.
- **`/world/load` returns 422 on a load failure** (not the 400/409/500 in
  §7.3's original text; the §7.3 example is now correct).
- **Per-class endpoints (§6) are largely not exposed as endpoints.** The
  flying bridge (`mavic_omnilink_bridge`) implements `takeoff` / `land` /
  `hover` / `goto_waypoint` / `set_gimbal_pitch` / `set_yaw` as verbs inside
  `POST /action`, not as the dedicated routes §6 describes. The quadruped
  bridge likewise has no `/set_pose` or `/wave` route — poses go through
  `/tool` or `/prompt`.
- **The direct REST actuators are ungated in every bridge** (§6's opening
  note, and §5.4 / §5.5 / §5.6). `POST /tool` (§5.8) and the `/prompt`
  cascade (§5.7) are vetted; `/drive_forward`, `/turn`, `/drive_to`,
  `/set_velocity`, `/reset_to_home`, the grippers and the joint verbs are
  not, in all four reference bridges and in `bridge_base` — which is the
  file every external bridge copies. `/stop_robot` is ungated on purpose
  (§5.5); the rest are not. Audit:
  [`packages/omnisim-bridges/GATE_COVERAGE.md`](packages/omnisim-bridges/GATE_COVERAGE.md).
  One further hole it names: the arm's learned verbs are promoted to
  `POST /<verb>` routes at runtime with no gate on them at all.
  ⚠️ **The ungated offline keyword ladder is no longer one of them** — all
  five ladders and the shared `intent_router` module were deleted on
  2026-09-22 (§5.7.1), and a relay-less bridge answers `401 omnikey_required`
  instead of falling through to one.
- ~~**`safety_gate` (§5.2.1) is specified and unimplemented**~~ — **fixed
  2026-09-22.** All five reference implementations publish it, including
  `ungated_paths` exact for each route table, so a client can now tell a
  gated route from an ungated one over the wire. ⚠️ Three caveats ride
  with it: the block sits at `[0].capabilities.safety_gate` on four
  implementations and at the top level on the flying bridge, because
  `/capabilities` itself has two shapes; `vertical_rail` is `null` on a
  bridge that declares no `surface`, and that is **not** "the strictest
  rail" (§5.2.1); and the arm's runtime-named learned verbs can never
  appear in a static `ungated_paths` list, so that list is exact for the
  declared routes and silent about them.
- ~~**The flying bridge's `POST /tool` does not strip `id`**~~ — **fixed
  2026-09-22** (§5.8.1), when the five `/tool` handlers were collapsed onto
  one that strips every transport field before the gate sees it.
- ~~**`actions[].result` has no shipped `"refused"` producer**~~ — **fixed
  2026-09-22** (§5.7.2). Both producers now emit `result: "refused"` with
  `rule`, and an unclassified refusal is filled in as
  `rule: "unclassified"` rather than left absent. ⚠️ A client still has to
  accept **both** `"err"` and `"error"` for a *failure*, and an external
  bridge copied from a pre-v9 reference emits neither `rule` nor
  `"refused"` — the summary-prefix recovery described in §5.7.2 exists for
  those bridges, not for these five.
- **New in v9, and stated as limits rather than features:**
  - **The bridge event ring (§5.9) is in-process and is not persisted.**
    It dies with the bridge, so an empty ring after a restart means
    "nothing since boot". There is still no record, no replay and no
    run-diff anywhere in this protocol.
  - **`contact.began` on a bridge sees top-level Solids only, and only a
    bounded watchlist of them.** URDF sub-links are invisible to the
    supervisor's contact query, and a body outside the watchlist produces
    no event. The absence of a contact event is never evidence.
  - **The measured-window fields are not universal.**
    `sim_time_start` / `sim_time_end` / `steps` ship on the mobile and arm
    bridges; the quadruped and flying bridges publish no measured
    completion record at all, so they carry neither those fields nor a
    stall verdict. ✅ **The stall field no longer has two names** — it was
    `stalled: true` on the mobile bridge and `measurable: false` on the
    arm until 2026-09-22; both now answer `stalled: true`, and the arm
    keeps `measurable: false` beside it for the different unmeasurable it
    also marks (§5.4.1).
  - **`lockstep` is declared by one bridge and it declares `false`.** The
    flying bridge publishes `lockstep: {supported: false}`; the three that
    honour `OMNISIM_BRIDGE_LOCKSTEP` publish no `lockstep` block, so the
    hold is discoverable only through `/state.held` (§5.2, §5.3).
  - **A held run is not bit-identical by construction, and the leak is
    hard to observe.** While a hold is live the loop pumps at most one
    step per second **if the robot window is open**, to keep it from going
    mute. Any such step is world motion during a nominally frozen run, and
    the count (`leak_steps`) reaches the wire only in the `hold.expired`
    event's detail — **no bridge reports it on `/state`** (§5.3).
  - **The `events` capability block is published by the four controller
    bridges, not synthesised by `bridge_base`.** Its absence does not mean
    the endpoint is missing.
  - **The event-driven wake's cost on an idle cluttered world has never
    been measured.** The rate limit is the only guard (§5.9).
  - **There is no window-raise verb in the robot-window protocol.** When
    the platform asks for an operator's attention, the bridge makes the
    request **visible in the chat panel**; an OS-level raise would need a
    plugin verb that does not exist. Nothing in this protocol raises a
    window.
- **`/read_camera` (§6) and `/read_mission_brief` (§6) do not exist.** The
  mavic bridge serves `/image`; the husky bridge serves `/camera` and
  `/mission`. §13.5's claim that `/image` *and* `/read_camera` both work is
  **false** — only `/image` does.
- **`/read_sensor` + `/list_sensors` (§6.6) exist on `omnilink_mobile_bridge`
  only** (added 2026-08-17). Every other bridge still has no sensor
  read-through, so a client MUST check `capabilities.sensors` rather than
  assume the verb is there.
- **`/list_robots`, `/get_robot_state`, `/stop_robot` and `/reset_to_home`
  are missing on `husky_omnilink_bridge` and `mavic_omnilink_bridge`.**
  They are present on the arm, mobile (`omnilink_mobile_bridge`) and
  quadruped bridges. ⚠️ **`/prompt` is no longer missing on the Mavic** —
  it was added 2026-09-21 along with `/tool`, and both are gated, which is
  also what put the aircraft into the chat-demo sweep
  (`smoke_chat_demos.EXPECTED_UNSCRIPTED` is now empty). ⚠️
  **`husky_omnilink_bridge` still routes everything through `POST /action`,
  and its `/action` is ungated.** It is a standalone server that imports no
  part of the bridges package — no shared `/tool` handler, no fail-closed
  wrapper, no gate — and it serves neither `/prompt` nor `/tool`, so it has
  no vetted path at all. It is also not idle: the `husky_maze` production
  agent reads a model's tool calls and turns each one into a
  `POST /action` against it. None of the v9 bridge work (`safety_gate`,
  `GET /events`, the clocks, `via`, `rule`) reaches it. `/action` is *not*
  inherently ungated — the flying bridge's is vetted — so this is a
  per-bridge gap, not a property of the verb.
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
