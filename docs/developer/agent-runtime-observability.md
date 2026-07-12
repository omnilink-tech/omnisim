# Agent Runtime Observability

**Status:** implemented and live — all three phases are wired in the harness.

This document describes the runtime observability layer that the OmniSim
validation harness exposes to AI coding agents. It is the answer to "agents
drive OmniSim mostly blind" — they could load a world and step it, but had
no general way to see joint state, contacts, gripped objects, or runtime
events. This layer fills that gap.

It is distinct from
[observability-and-performance-telemetry.md](observability-and-performance-telemetry.md),
which covers engine-internal phase timings (loading / physics / rendering).
That layer is for OmniSim contributors profiling the engine. The layer
described here is for agents driving the engine.

---

## Design principles

1. **Two layers, mirrored on every observable subsystem:**
   - **Snapshot endpoints** answer "what is true right now." Cheap,
     idempotent, JSON.
   - **Event endpoints** answer "what changed since cursor N." Since-cursor
     pull, no server-side per-client state.
2. **Reuse the existing harness IPC** (length-prefixed JSON over a single
   TCP socket between the harness HTTP service and the
   `harness_supervisor` controller). No new processes, no new ports, no new
   IPC channel.
3. **Snapshots derive from the scene tree.** The supervisor cannot use
   `Motor` / `PositionSensor` device APIs on robots it does not own — those
   belong to the user's controller. Joint state is read from
   `JointParameters.position` (which Webots updates live), velocity is
   computed by differencing.
4. **Events flow through one bus per side.** The supervisor maintains an
   `EventBus` for sim-visible events (contacts, joint limits, grips,
   damage). The harness maintains a `LogRingBuffer` for controller stdout /
   stderr lines and `omnisim_log.txt` deltas. `/sim/events` composes both
   into a single response with a composite cursor `{"sup": N, "log": M}`.
5. **Bounded memory.** Both buffers use `collections.deque(maxlen=...)`. A
   `dropped` counter is exposed so an agent can detect cursor lag.

---

## HTTP surface

### Snapshots

| Endpoint | Returns |
|---|---|
| `GET /robots` | `{robots: [{def, name, model, controller, position, orientation, num_devices}]}` |
| `GET /robot/<def>/joints` | `{robot, joints: [{name, type, position, velocity, lower, upper, hit_limit}]}` |
| `GET /robot/<def>/devices` | `{robot, devices: [{name, type, sample_period_ms?}]}` |
| `GET /robot/<def>/sensor/<name>` | `501` — the supervisor cannot read sensors owned by user controllers; use `/joints` for kinematic state or have the user controller expose its own surface |
| `GET /sim/contacts` | `{contacts: [{a_def, b_def, point}]}` |
| `GET /sim/grips` | `{grips: [{gripper_def, held_def, since_t_ms}]}` |

Pre-existing endpoints (`/sim/state`, `/scene/tree`, `/scene/node/<def>`,
`/world/screenshot`, `/world/render_stats`, `/robot/damage`) keep their
shape.

### Events

```
GET /sim/events?since=<sup_cursor>&log_since=<log_cursor>&limit=<int>&types=<csv>
```

Returns:
```json
{
  "events":     [{"seq", "source": "sup"|"log", "t_sim_ms"?, "t_wall"?, "type", ...}],
  "next_since": <sup_cursor>,
  "next_log_since": <log_cursor>,
  "dropped_sup": <int>,
  "dropped_log": <int>
}
```

`source` distinguishes which buffer the event came from. `t_sim_ms` is set
on supervisor-side events; `t_wall` is set on controller-log / world-log
events (they have no sim time — they came off a stdout pipe).

The `types` filter accepts a CSV like `contact.began,joint.limit_hit`.

For back-compat, `/robot/damage/events?since=N` keeps its shape and is now
a filtered view onto the same supervisor bus (`source=sup`,
`type=damage.*`).

### Event taxonomy

**Supervisor-side** (sim time available):

- `contact.began` — `{a_def, b_def, point, force}`
- `contact.ended` — `{a_def, b_def}`
- `joint.limit_hit` — `{robot, joint, side: "lower"|"upper", position}`
- `grip.acquired` — `{gripper_def, object_def}`
- `grip.released` — `{gripper_def, object_def}`
- `damage.impact` — same payload as the existing damage_tracker impact
- `damage.state_transition` — same payload as the existing damage_tracker
  state-transition

**Harness-side** (wall time only):

- `controller.log` — `{stream: "stdout"|"stderr", line}` — one event per
  line read off the OmniSim subprocess's stdout/stderr.
- `world.warning` / `world.error` — derived from `omnisim_log.txt` deltas
  using the same classifier as `/world/diagnostics`.

---

## Wiring — controller stdout

The harness launches OmniSim with `--stdout --stderr`. Previously the
subprocess was started with `stdout=DEVNULL, stderr=DEVNULL`, throwing all
controller output away. The new wiring:

1. `subprocess.Popen(..., stdout=PIPE, stderr=PIPE)`
2. Two daemon reader threads (`_stdout_pump`, `_stderr_pump`) read line by
   line, push each line as a `controller.log` event into the harness
   `LogRingBuffer`, and also forward to the harness's own stdout/stderr so
   the operator running the harness still sees the log.
3. Buffer is bounded (default 4096 lines). When full, oldest lines are
   evicted; `dropped_log` increments.

This is the "supervisor/launcher level" wiring the design called for —
it captures all controllers' output, including the user's, without
touching the C++ launcher.

When the world is hot-reloaded, the OmniSim subprocess survives, so the
threads stay attached. When the world is cold-launched (subprocess
terminated and respawned), the threads exit cleanly on EOF and new ones
are started.

---

## Wiring — supervisor event producers

Each supervisor step polls three producers:

1. **Contact tracker.** Walk all `Solid` nodes once at startup, cache them
   by node id. Each step, call `getContactPoints(includeDescendants=False)`
   on each solid, build a set of `(self_def_or_id, other_node_id)` pairs.
   Diff against the previous step's set:
   - new pair → `contact.began`
   - dropped pair → `contact.ended`
2. **Joint-limit tracker.** Walk all `HingeJoint` / `SliderJoint` nodes,
   cache `(jointParameters, minStop, maxStop)`. Each step, read each
   joint's `position` and emit `joint.limit_hit` when crossing into a
   stop band (with hysteresis to avoid oscillating).
3. **Grip tracker.** A grip is heuristically "two-finger gripper holding
   object" — derived from the contact set: same object touching ≥2 child
   solids of the same gripper parent for ≥N consecutive steps. Emits
   `grip.acquired` / `grip.released`. Generic; no per-robot config.
4. **Damage tracker.** Already runs each step. Its emit calls now also
   push onto the EventBus as `damage.*` types. (Damage's own ring buffer
   is kept for back-compat — events are fanned out to both.)

---

## Phasing

- **Phase 1 (snapshots).** `/robots`, `/robot/<def>/joints`,
  `/sim/contacts`. No event bus needed.
- **Phase 2 (events).** EventBus + LogRingBuffer + `/sim/events` with
  contact + joint-limit + damage producers. Controller-stdout pipe.
- **Phase 3 (devices/sensors/grips).** `/robot/<def>/devices`,
  `/robot/<def>/sensor/<name>`, `/sim/grips`.

---

## Tests

Unit tests in `tests/harness/test_observability.py` cover:

- `EventBus` ring-buffer semantics (seq monotonic, drops, since-cursor
  filtering, type filter).
- `LogRingBuffer` line capture from a fake stream.
- `compose_events` merging the two buffers under the composite cursor.

End-to-end behavior (joint reads against a live OmniSim world) is exercised
by the existing harness smoke lane — adding a one-line `curl /robots`
check to the lane is enough to confirm the surface works.

---

## Compat

- Existing endpoints unchanged. Existing damage event consumers continue
  to work via `/robot/damage/events`.
- The supervisor wire protocol gains new commands (`robots_list`,
  `robot_joints`, `sim_contacts`, `events_drain`, `robot_devices`,
  `robot_sensor_read`, `sim_grips`) — additive only, no removals.
- The harness subprocess change (DEVNULL → PIPE) means the harness now
  also forwards controller log lines to its own stdout/stderr. Operators
  running the harness on top of `run-headless`'s log-only check should be
  unaffected — `run-headless` doesn't share state with the harness.
