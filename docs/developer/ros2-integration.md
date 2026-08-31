# ROS 2 and OmniSim

**2026-08-17.** OmniSim ships first-party ROS 2 support: the ROS 2
[`simulation_interfaces`](https://github.com/ros-simulation/simulation_interfaces)
standard plus a live robot surface (`/clock`, `/tf`, `JointState`, `cmd_vel`,
`/odom`), implemented as a **sidecar package** over OmniSim's existing HTTP
surface. The package is [`packages/omnisim-ros2/`](../../packages/omnisim-ros2/).

**This page used to say the opposite.** Until 2026-08-17, ROS 2 was a documented
non-goal. That history is kept below, because the reasoning that produced it is
still half-true and it explains the shape of what was built.

---

## The history, and what changed

### What the decision was (2026-07-10 → 2026-08-17)

> *OmniSim's agent interface is HTTP/JSON, not ROS.* The whole product thesis is
> agent-native: an LLM agent authors a world and drives a robot over a small,
> versioned HTTP surface it can reason about, without a ROS graph, a middleware,
> or a message-type compilation step. That is a different bet from the ROS
> ecosystem, made on purpose.

That reasoning was sound and **is still the reason the HTTP surface stays
first-class**. What it got wrong was treating "HTTP is our agent interface" and
"we should have a ROS bridge" as alternatives. They are not. An agent-native
simulator that a robotics lab cannot plug into its existing stack is a simulator
that lab will not evaluate at all — and the cost of supporting both turned out to
be far lower than the old page assumed, because the harness already implements
every verb the ROS standard asks for.

### What changed

1. **The project owner reversed the decision** and asked for full ROS 2 support.
2. **`simulation_interfaces` matured.** The old page predated the standard being
   a realistic target. It is now implemented by Gazebo, Isaac Sim and O3DE, which
   makes it the interoperability layer rather than a fourth bespoke bridge. It
   maps almost one-to-one onto verbs the harness already served.
3. **The old page's own escape hatch turned out to be the right architecture.**
   It sketched "a small ROS 2 node that polls the OmniSim bridge over HTTP and
   republishes as ROS messages" and called it *"a SKETCH of the pattern, not a
   shipped package"*. That sketch is now the shipped package. Nothing about the
   approach needed to change — only the decision to maintain and test it.

### What did NOT change

- **The engine stays ROS-free.** No `rclcpp` in `src/omnisim/`, no ROS in the
  engine Makefiles, no ROS build dependency. A non-ROS user's build is unaffected.
- **HTTP/JSON remains the first-class agent interface.** [PROTOCOL.md](../../PROTOCOL.md)
  is still the contract; ROS 2 is *additive*, layered on top of it, and every ROS
  verb is implemented by calling an HTTP endpoint that already existed.
- **We did not port `webots_ros2`.** The old page named that as the reversible
  path. It was the wrong one: `webots_ros2` targets the Webots controller
  protocol and node set, from which OmniSim has genuinely diverged (`URDFRobot`,
  the `omnisim://` scheme, `newton*` fields, deleted `Fluid`/physics-plugin API).
  Building on OmniSim's own HTTP surface is less code and cannot rot against
  upstream Webots.

---

## Architecture

```
   ROS 2 graph                    HTTP/JSON                 OmniSim
┌────────────────┐          ┌──────────────────┐      ┌────────────────┐
│ your nodes,    │  ROS 2   │ omnisim_ros2     │ HTTP │ World Harness  │
│ rviz2, rosbag  │◄────────►│ (sidecar package)│◄────►│    :6789       │
└────────────────┘ services └──────────────────┘      ├────────────────┤
                   topics            │           HTTP │ robot bridge   │
                                     └───────────────►│    :8765       │
                                                      ├────────────────┤
                                                      │  omnisim-bin   │
                                                      │  (no ROS deps) │
                                                      └────────────────┘
```

Two HTTP surfaces, for a structural reason:

- **The World Harness (`:6789`)** answers everything about the *scene* — entities,
  poses, stepping, spawning, world loading, joint positions.
- **The per-robot bridge (`:8765`)** answers everything about a *device*.
  `GET /robot/<def>/sensor/<name>` on the harness returns **501 by design**:
  OmniSim, like upstream Webots, restricts device APIs to the controller that
  owns the device, so the supervisor genuinely cannot read another robot's camera
  or drive another robot's motors. Anything device-shaped — sensors, `cmd_vel`,
  joint commands, odometry — therefore goes to the robot's own bridge.

That 501 is not a gap to route around; it is the reason the ROS surface is split
across two clients.

---

## What is implemented

### Tier 1 — `simulation_interfaces` ✅

15 services + the `SimulateSteps` action, against
[`simulation_interfaces` 2.1.0](https://github.com/ros-simulation/simulation_interfaces).
`GetSimulatorFeatures` advertises 21 features and `spawn_formats: [vrml, urdf]`.

| Service | Harness endpoint |
|---|---|
| `GetSimulatorFeatures` | *(static declaration)* |
| `GetSimulationState` / `SetSimulationState` | `GET /sim/state` · `POST /sim/reset` |
| `StepSimulation` · `SimulateSteps` (action) | `POST /sim/step` |
| `ResetSimulation` | `POST /sim/reset` + `POST /scene/delete` |
| `GetEntities` · `GetEntityState` · `GetEntitiesStates` | `GET /scene/tree` |
| `SetEntityState` | `POST /scene/set_pose` |
| `GetEntityInfo` · `GetEntityBounds` | `GET /scene/node/<def>` · `?bounds=1` |
| `SpawnEntity` · `DeleteEntity` | `POST /scene/spawn` · `POST /scene/delete` |
| `LoadWorld` · `GetCurrentWorld` | `POST /world/load` · `GET /sim/state` |

### Tier 2 — the live robot surface ✅

| Topic | Type | Source |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | `GET /sim/state` → `sim_time_ms` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `GET /scene/tree` |
| `<robot>/joint_states` | `sensor_msgs/JointState` | `GET /robot/<def>/joints` |
| `/odom` | `nav_msgs/Odometry` | bridge `POST /get_robot_state` |
| `/cmd_vel` *(sub)* | `geometry_msgs/Twist` | bridge `POST /set_velocity` |
| `/joint_command` *(sub)* | `sensor_msgs/JointState` | bridge `POST /set_joint_positions` |
| `/imu/data` | `sensor_msgs/Imu` | bridge `POST /read_sensor` |
| `/scan` | `sensor_msgs/LaserScan` | bridge `POST /read_sensor` |
| `/gps/local` *or* `/gps/fix` | `PointStamped` / `NavSatFix` | bridge `POST /read_sensor` |

`use_sim_time` is supported throughout, and TF is a **real tree** — each node is
published relative to its nearest DEF-bearing ancestor, not flattened to `world`.

**Sensor topics (added 2026-08-17).** The blocker was never the ROS side: the
harness's `GET /robot/<def>/sensor/<name>` is a deliberate 501, so data had to
come from the robot's own controller, and **no shipped bridge implemented the
optional `/read_sensor` verb**. `omnilink_mobile_bridge` now does, plus
`/list_sensors` for discovery, and `sensor_node` publishes from it.

Three things about that lane are load-bearing and are documented at the source
rather than discovered later:

- ⚠ **A URDF robot has no devices unless the world was loaded with
  `OMNISIM_URDF_USE_SENSORS=1`.** The importer parses `<gazebo>` sensor blocks
  always and drops them at emit time when the variable is unset. Measured on the
  shipped Husky: **0 devices without it, 5 with it**. The gate's own source
  comment claims the emission path "crashes OmniSim' device registration
  mid-startup"; that is **stale** — it loads cleanly and produces working
  devices, presumably fixed by the later rework that moved sensor carriers
  inline into their link's Solid.
- ⛔ **`Gyro` and `Accelerometer` yield nothing here — and the ROOT CAUSE is now known:
  the device is fine, its CARRIER is not a Newton body.** The gyro reads exactly
  `[0,0,0]` while the robot rotates; the accelerometer never yields a sample at
  all, not even gravity. Both sit in the *same* emitted carrier Solid as the
  `InertialUnit`, which works perfectly.

  The asymmetry is in how the two device families read their pose.
  `OmGyro`/`OmAccelerometer` go through **`upperSolid()->bodyHandle()`**;
  `InertialUnit` and GPS-position go through **`matrix()`**. A `matrix()` read
  works off the scene graph and needs no solver body; a `bodyHandle()` read
  needs one. **Declaring `physics` on a jointless nested `Solid` does not create
  a Newton body** — so the importer's IMU cluster structurally cannot serve
  Gyro or Accelerometer, while its `InertialUnit` is unaffected.

  Measured (OmniBench lane 4, 2026-08-17) in one run, one rotating arm, three
  gyros: a **real Newton body** reads `[0, 0, 2.0000]` against a
  supervisor-measured 2.0000 rad/s; a **physics-less nested Solid** reads
  `[0, 0, 0.0]`; and the URDF importer's *exact* carrier pattern
  ([`OmUrdfImporter.cpp:1103`](../../src/omnisim/vrml/OmUrdfImporter.cpp))
  also reads `[0, 0, 0.0]`, with the engine warning by name.

  ✅ **So this is a fixable URDF-emission bug, not a broken device** — the fix is
  to emit the IMU cluster onto something that becomes a body, and the lane-4
  probe (rebuilt on a driven turntable) proves the device itself works.
  Until then `sensor_msgs/Imu` ships a real orientation with
  `angular_velocity_covariance[0]` and `linear_acceleration_covariance[0]` set
  to `-1`, the ROS convention for "not available", rather than zeros that would
  read as real measurements. The genuine yaw rate is on `/odom`, differenced
  from pose.
- ⛔ **No camera exists in this tree**, so no `Image`/`CameraInfo` was shipped.

The Husky URDF gained a `<sensor type="ray">` block (a SICK LMS111 on the sensor
arch, 270° / 541 samples / 0.1–20 m) so the lidar path could be verified against
a real device. It is **inert** unless the flag above is set — verified by a
control run: the same world reports `devices: []` without it.

### Tier 3 — `ros2_control` ✅ velocity-commanded bases · ⛔ arms

[`packages/omnisim-ros2/src/omnisim_ros2_control/`](../../packages/omnisim-ros2/src/omnisim_ros2_control/)
ships `omnisim_ros2_control/OmniSimSystem`, a C++
`hardware_interface::SystemInterface`. `controller_manager` treats an OmniSim
robot as ordinary hardware, so stock controllers drive it unmodified.

| | |
|---|---|
| state | harness `GET /robot/<def>/joints` → `position` + `velocity` interfaces |
| commands | bridge `POST /set_velocity` (`diff_drive`) or `POST /set_joint_positions` |
| verified | `diff_drive_controller` + `joint_state_broadcaster` on the Husky |

Neither endpoint is new. The plugin adds no OmniSim-side surface, and the engine
is untouched.

```bash
ros2 launch omnisim_ros2_control husky_diff_drive.launch.py
```

**Measured, machine `9722d23d12a3`, 2026-08-17.** With
`omnilink_husky.omniworld` loaded, `ros2 control list_hardware_interfaces`
reports four claimed `velocity` command interfaces and eight `position`/
`velocity` state interfaces; both controllers report `active`; and a
`cmd_vel_unstamped` publish drove the Husky from `x = −0.00000` to
`x = +5.59211` — **5.5921 m over 195.22 s of simulated time**, with the pose and
the clock read out of the simulator rather than from the command.

⚠ **`update_rate` is not the actuation rate.** `read()` / `write()` do no I/O; a
background thread owns the HTTP at `comms_rate_hz`. The **ceiling is OmniSim's
supervisor round trip, not TCP**: `GET /robot/<def>/joints` costs 21.01 ms
against 4.48 ms for a bare `GET /healthz` on the same server, so one read plus
one write is ~22 ms and the hard ceiling is near **45 Hz**. The shipped default
is 25 Hz. Full table, the WSL-tunnel figures and the live diagnostics are in the
package README.

✅ **The arm-bridge blocker is FIXED (2026-08-19): `omnilink_arm_bridge` now has
the non-blocking, superseding servo verb — `POST /servo_joint_positions`
(PROTOCOL.md §6.1).** The old goal contract is unchanged and still measured true:
a second `set_joint_positions` 50 ms after the first comes back
`HTTP 409  accepted=False  error='busy'` with `"this set_joint_positions was NOT
applied"` — a trajectory controller pointing at *that* verb still lands in
pieces. The servo verb is the lane it points at instead: last-write-wins, never
a 409 for servo-on-servo, preempts an in-flight goal and says so
(`preempted: "set_joint_positions"`), and the measured result parks into
`get_robot_state.last_command` when the stream goes quiet. Verified on
`omnilink_ur5e.omniworld`: 24 setpoints at ~18 Hz, 24/24 accepted, zero 409s,
parked max error 0.011 rad. ✅ **The ROS-side wiring is ALSO done (2026-08-19,
same day):** both `joint_command` (Tier 2) and `command_mode: joint_positions`
(Tier 3) read the bridge's `/capabilities` and stream to
`/servo_joint_positions` when `capabilities.servo` is advertised, falling back
to the goal verb — with the 409-explainer demoted to that fallback lane — on
bridges without it. Verified live on the Tier 2 lane (machine `9722d23d12a3`,
WSL Humble → the Windows engine over the port tunnel: 60 `joint_command`
messages at 15 Hz through the real `command_node`, zero 409s, arm settled
0.018 rad from target, and a single stream `seq` per target proving
last-write-wins retargeting). ⚠️ Tier 3's servo path is compiled and
unit-tested under Humble but has **not** run under a live `controller_manager`,
and **MoveIt has still never been brought up** — say "unblocked", never
"working".

**Nav2 ✅ brought up end-to-end (2026-08-31).** The first Nav2 stack (ROS 2
**Jazzy**) ever run against OmniSim drove the Husky autonomously to a
`NavigateToPose` goal — `(0,0) → (1.3, 0)`, `SUCCEEDED` in ~10–12 s, pose read out
of the simulator. It ships as its own colcon package,
[`packages/omnisim-ros2/src/omnisim_ros2_nav2/`](../../packages/omnisim-ros2/src/omnisim_ros2_nav2/)
(runbook + beta-feedback report), layered on the Tier-2 sidecar with **no** new
OmniSim surface. Two paths were tried — a Windows-engine + WSL-ROS split (reached
M1–M5, then stalled on tunnels / `wslrelay` / firewall / ephemeral-port
exhaustion) and **all-in-WSL** (OmniSim built and run inside the *same* Ubuntu
24.04 as ROS 2, everything on `localhost`); the all-in-WSL path is the one that
closed M6 and is the documented, supported path going forward. Winning
configuration: CPU `mj_step` solver (`OMNISIM_NEWTON_MODEL_DEVICE=cpu`, needed for
the Lidar rays), a no-window headless sim (`OMNISIM_NO_WINDOW=1`), **CycloneDDS**
as the RMW on every node (FastDDS discovery is unreliable in WSL2), a static
identity `map→odom` (OmniSim `/odom` is ground truth, so no SLAM/AMCL), and a lean
five-server Nav2 launch. ⚠ **Scope it exactly: this demonstrates planning and goal
execution ONLY — not a full localization or obstacle-avoidance benchmark.** The
winning config uses a rolling costmap with no `static_layer`, and `lidar_layer:=0`
sees open space because this short-wall test world's walls are only 0.1 m tall; a
world with taller walls would exercise real obstacle avoidance. The package is
written to generalize to any well-defined differential-drive robot, not just the
Husky. OmniSim is also still absent from the `ros2_control` simulator registry.

---

## Honest limitations

Declared through the feature flags **and** repeated in
`GetSimulatorFeatures.custom_info`, so a caller learns them from the API:

1. **No pause.** OmniSim's engine free-runs between HTTP calls and the harness
   exposes no pause verb. `SIMULATION_STATE_PAUSE` is not advertised;
   `GetSimulationState` answers `STATE_PLAYING` whenever a world is loaded; and
   `StepSimulation` means "advance at least N basic steps", not "advance exactly
   N from a frozen state".
2. **`EntityState.twist` / `.acceleration` are not measured** and are returned as
   zeros — *unmeasured*, not observed-to-be-zero. The harness reports poses only.
   Real velocities are available on `/odom` (from the bridge) and in
   `JointState.velocity`.
3. **`JointState.velocity` is finite-differenced by the harness**, not read from
   the engine, and is `null` on a joint's first read. When any joint's velocity is
   unknown the whole array is published **empty** — the ROS idiom for "not
   provided" — rather than substituting a zero that reads as "stationary".
   `effort` is never published; OmniSim does not expose it.
4. **`ResetSimulation` re-pins every motor** to a position hold. That is the
   harness's documented reset behaviour; its warning is passed through verbatim.
5. **`DeleteEntity` removes a node from the scene graph, not from the physics
   model.** Deleted geometry keeps blocking rays and contacts until a reload.
6. **Sensor topics cover orientation, range and position only.** `Gyro` reads a
   constant zero and `Accelerometer` never samples, so both are declared absent
   via `covariance[0] = -1` instead of being published as zeros; there is no
   camera in the tree, so no `Image`/`CameraInfo`. Only
   `omnilink_mobile_bridge` implements `/read_sensor`
   ([PROTOCOL.md §6.6](../../PROTOCOL.md)) — other bridges still have no sensor
   surface, so check `capabilities.sensors` rather than assuming the verb.
   And on any URDF robot the devices only exist at all when the world was loaded
   with `OMNISIM_URDF_USE_SENSORS=1`.
7. **`WorldInfo.coordinateSystem` is not queryable over HTTP.** OmniSim defaults
   to Z-up/ENU, which is exactly REP-103, so no axis remapping is applied. A world
   that overrides it to `NUE` cannot be detected at runtime.
8. **Rates cost TCP connections — for harness traffic.** The **bridge** now sets
   `protocol_version = "HTTP/1.1"` and the client pools connections, measured at
   300 requests costing **+0** `TIME_WAIT` sockets at 908.8 req/s, against
   **+300** at 114.8 req/s before. The **World Harness still speaks HTTP/1.0**,
   so `clock_node`, `robot_state_node` and `simulation_interfaces_node` still
   burn one socket per request; the pool detects that and degrades automatically.
   Each closed socket holds an ephemeral port for 120 s against Windows' 16,384,
   which is how a ~50 Hz bringup previously reached 17,487 sockets in
   `TIME_WAIT` and `WinError 10048`. Giving the harness keep-alive is the
   obvious next win and has not been done.

---

## Quickstart

```bash
# 1. OmniSim side
python -m omnisim harness --auto-port

# 2. ROS 2 side (Humble or newer)
bash packages/omnisim-ros2/scripts/bootstrap.sh
source packages/omnisim-ros2/install/setup.bash
ros2 launch omnisim_ros2 omnisim_bringup.launch.py

# 3. use it
ros2 service call /omnisim_simulation_interfaces/get_entities \
  simulation_interfaces/srv/GetEntities "{}"
ros2 topic echo /clock --once
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

Worked examples with real captured output, the Windows+WSL2 transport, and the
full parameter list: [`packages/omnisim-ros2/README.md`](../../packages/omnisim-ros2/README.md).

---

## When to use something else

Still honest, with two clauses narrowed. **If your stack *is* ROS 2 end to end —
Nav2, MoveIt, the whole graph — [Gazebo](https://gazebosim.org) remains the
better-integrated choice today**, and upstream
[Webots](https://github.com/cyberbotics/webots) + `webots_ros2` is the supported
path if you want Webots' robot models with a mature bridge. What changed is that
`ros2_control` is no longer on that list for a velocity-commanded base (a
`SystemInterface` ships and `diff_drive_controller` is verified), and **Nav2 is no
longer either**: a full Nav2 stack (ROS 2 Jazzy) now drives the Husky to a goal —
but only for **planning + goal execution** on the all-in-WSL path, not as a
localization or obstacle-avoidance benchmark (see the Nav2 note above and
[`packages/omnisim-ros2/src/omnisim_ros2_nav2/`](../../packages/omnisim-ros2/src/omnisim_ros2_nav2/)).
What has not changed is the rest — **MoveIt is unblocked (the arm bridge grew its
servo verb, 2026-08-19) but has never been brought up here, and OmniSim is not in
the `ros2_control` simulator registry.**

What OmniSim offers instead is the agent-native surface — the harness, MCP,
structured load diagnostics, the capture/cinema pipeline — with ROS 2 now
available on top rather than instead.

## See also

- [`packages/omnisim-ros2/`](../../packages/omnisim-ros2/) — the package, its README and tests
- [`packages/omnisim-ros2/src/omnisim_ros2_nav2/`](../../packages/omnisim-ros2/src/omnisim_ros2_nav2/) — the Nav2 (Jazzy) Husky bring-up: runbook + beta-feedback report (M1–M6)
- [PROTOCOL.md](../../PROTOCOL.md) — the HTTP/JSON surface underneath
- [scripts/harness/README.md](../../scripts/harness/README.md) — the World Harness
- [simulator-comparison.md](simulator-comparison.md) — where OmniSim stands
- [packages/omnisim-mcp/](../../packages/omnisim-mcp/) — the MCP server over the same surface
