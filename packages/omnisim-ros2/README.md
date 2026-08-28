# omnisim-ros2 — ROS 2 support for OmniSim

**ROS 2 support for [OmniSim](../../README.md), built as a sidecar over the
existing HTTP surface. The engine stays ROS-free.**

Implements the ROS 2
[`simulation_interfaces`](https://github.com/ros-simulation/simulation_interfaces)
standard (v2.1.0) — the same standard Gazebo, Isaac Sim and O3DE implement — plus
a live robot surface (`/clock`, `/tf`, `JointState`, `/odom`, `cmd_vel`, sensors)
and a **`ros2_control` hardware interface**, so stock controllers drive an
OmniSim robot without knowing OmniSim exists.

> **This was a declared non-goal until 2026-08-17.** The reasoning, what changed,
> and what is still true about the HTTP-first design is recorded in
> [docs/developer/ros2-integration.md](../../docs/developer/ros2-integration.md).

---

## Architecture: why a sidecar

```
   ROS 2 graph                    HTTP/JSON                 OmniSim
┌────────────────┐          ┌──────────────────┐      ┌────────────────┐
│ your nodes,    │  ROS 2   │  omnisim_ros2    │ HTTP │ World Harness  │
│ rviz2, rosbag  │◄────────►│  (this package)  │◄────►│    :6789       │
└────────────────┘ services └──────────────────┘      ├────────────────┤
                   topics            │           HTTP │ robot bridge   │
                                     └───────────────►│    :8765       │
                                                      ├────────────────┤
                                                      │  omnisim-bin   │
                                                      │  (no ROS deps) │
                                                      └────────────────┘
```

**The OmniSim engine carries no ROS dependency** — no `rclcpp` in `src/omnisim/`,
no ROS in the engine Makefiles. A non-ROS user's build is unaffected. Every verb
maps onto a harness endpoint that already existed; nothing in the engine changed.

**Two HTTP surfaces, for a structural reason.** The harness answers everything
about the *scene*. The per-robot bridge answers everything about a *device*:
`GET /robot/<def>/sensor/<name>` on the harness is **501 by design** — OmniSim
restricts device APIs to the controller that owns the device, so the supervisor
genuinely cannot read another robot's camera or drive another robot's motors.
Sensors, `cmd_vel`, joint commands and odometry therefore go to the bridge.

---

## What is implemented

### Tier 1 — `simulation_interfaces` ✅ · 15 services + 1 action

| ROS 2 service | OmniSim harness endpoint |
|---|---|
| `GetSimulatorFeatures` | *(static declaration)* |
| `GetSimulationState` · `SetSimulationState` | `GET /sim/state` · `POST /sim/reset` |
| `StepSimulation` · `SimulateSteps` *(action)* | `POST /sim/step` |
| `ResetSimulation` | `POST /sim/reset` + `POST /scene/delete` |
| `GetEntities` · `GetEntityState` · `GetEntitiesStates` | `GET /scene/tree` |
| `SetEntityState` | `POST /scene/set_pose` |
| `GetEntityInfo` · `GetEntityBounds` | `GET /scene/node/<def>` · `?bounds=1` |
| `SpawnEntity` · `DeleteEntity` | `POST /scene/spawn` · `POST /scene/delete` |
| `LoadWorld` · `GetCurrentWorld` | `POST /world/load` · `GET /sim/state` |

`GetSimulatorFeatures` advertises 21 features and `spawn_formats: [vrml, urdf]`.

### Tier 2 — the live robot surface ✅

| Topic | Type | Node | Source |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | `clock_node` | `GET /sim/state` → `sim_time_ms` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_node` | `GET /scene/tree` |
| `<robot>/joint_states` | `sensor_msgs/JointState` | `robot_state_node` | `GET /robot/<def>/joints` |
| `/odom` | `nav_msgs/Odometry` | `odom_node` | bridge `POST /get_robot_state` |
| `/cmd_vel` *(sub)* | `geometry_msgs/Twist` | `command_node` | bridge `POST /set_velocity` |
| `/joint_command` *(sub)* | `sensor_msgs/JointState` | `command_node` | bridge `POST /servo_joint_positions` when advertised (`capabilities.servo`), else `POST /set_joint_positions` |
| `/imu/data` | `sensor_msgs/Imu` | `sensor_node` | bridge `POST /read_sensor` |
| `/scan` | `sensor_msgs/LaserScan` | `sensor_node` | bridge `POST /read_sensor` |
| `/gps/local` *or* `/gps/fix` | `PointStamped` / `NavSatFix` | `sensor_node` | bridge `POST /read_sensor` |

`use_sim_time` is supported throughout. **TF is a real tree** — each node is
published relative to its nearest DEF-bearing ancestor (`T_parent⁻¹·T_child`),
not flattened onto `world`.

#### Joint commands pick their bridge verb from `/capabilities`

`joint_command` is a *stream*, and the two arm-bridge verbs treat a stream very
differently (PROTOCOL.md §6.1): `POST /servo_joint_positions` is the streaming
lane — non-blocking, last-write-wins, preempts an in-flight goal, never a 409
for servo-on-servo — while `POST /set_joint_positions` is a **goal** that
answers 409 busy to any command arriving mid-interpolation, so a stream pointed
at it lands in pieces. The bridge self-describes, so `command_node` reads
`/capabilities` once (retrying while the bridge is unreachable) and takes the
servo verb when `capabilities.servo` is advertised, falling back to the goal
verb on older bridges and non-arm bridges. The chosen lane is logged at
startup; on the fallback lane a 409 warning explains the goal contract, on the
servo lane a 409 would mean the bridge is not honouring its own advertisement.

#### Sensor topics — read this before wiring anything to them

**⚠ A URDF robot has NO sensor devices unless the world was loaded with
`OMNISIM_URDF_USE_SENSORS=1`.** The importer parses `<gazebo>` sensor blocks
always but *drops* them at emit time when that variable is unset. Measured on
the shipped Husky: **0 devices without it, 5 with it**. This is the single most
likely reason `sensor_node` reports "no readable sensors".

```bash
OMNISIM_URDF_USE_SENSORS=1 python -m omnisim harness --auto-port
```

Sensors come from the robot's **own controller**, never the harness:
`GET /robot/<def>/sensor/<name>` is a deliberate 501, because a supervisor
cannot honestly read a device it does not own. `sensor_node` therefore talks to
the bridge's `/read_sensor` + `/list_sensors` (PROTOCOL.md §6.6).

**What is live and what is not.** Measured 2026-08-17 on the shipped Husky
(machine `9722d23d12a3`, CPU `mj_step`), driving and turning the robot between
reads:

| Device | Verdict | Evidence |
|---|---|---|
| `InertialUnit` | ✅ **live** | `/imu/data` yaw `+0.1300` vs the bridge's own `0.1300` after a turn |
| `GPS` | ✅ **live** | `/gps/local` x `0.0000 → +5.5918` over a drive |
| `Lidar` | ✅ **live** | 541 rays; finite returns `541 → 190`, min range `5.828 → 5.438` |
| `PositionSensor` | ✅ live | wheel angle reached 34.5 rad |
| `Gyro` | ⛔ **dead** | read exactly `[0,0,0]` while yaw travelled `0 → 0.136 rad` |
| `Accelerometer` | ⛔ **dead** | never produced a sample at all — not even gravity |

So `sensor_msgs/Imu` goes out with a **real orientation** and with
`angular_velocity_covariance[0]` and `linear_acceleration_covariance[0]` set to
**`-1`** — the ROS-wide convention for "this component is not available". That
is deliberate: a zero there would claim the robot is neither rotating nor
accelerating, which nothing measured. The genuine yaw *rate* is on `/odom`
(`twist.twist.angular.z`), differenced from pose by the bridge — it is not
copied into the Imu message, because an `Imu` blending two sources is exactly
the kind of quiet mix a later reader would over-trust.

⛔ **No camera, and no `Image`/`CameraInfo`.** No URDF in this tree declares a
`<sensor type="camera">`, so nothing was there to publish. `/read_sensor`
refuses image devices by design and points at `/image` (§6.7); inlining frames
would also blow the request budget below.

**Lidar layers.** `LaserScan` is single-layer, but OmniSim's `Lidar` node
defaults to **4** layers and the importer only writes `numberOfLayers` when the
URDF asks for >1 — so a URDF-declared *planar* scanner still arrives as 4-layer.
`lidar_layer:=-1` (the default) picks the layer with the most finite returns and
**logs which**; on the Husky demo world that is layer 3 (`[0, 0, 0, 541]`),
because the stage walls are only 0.1 m tall so the one down-angled layer sees
floor. Set it explicitly for a genuinely multi-layer device.

A no-return ray is `+inf`, which is not valid JSON, so the bridge sends `null`
and `sensor_node` maps it back to `inf` — never to `0.0`, which every consumer
reads as an obstacle touching the lens.

**Sensor frames are measured, not assumed.** The bridge resolves each device
through the supervisor and reports its mount pose in the robot's frame, which
`sensor_node` latches onto `/tf_static` (`base_link → base_laser` came back as
`0.2012, 0, 0.505`). A sensor whose mount cannot be measured gets **no**
transform rather than an identity one.

### Tier 3 — `ros2_control` ✅ for velocity-commanded bases · arms wired to the servo lane, not yet verified end-to-end

`omnisim_ros2_control/OmniSimSystem` is a `hardware_interface::SystemInterface`,
so `controller_manager` treats an OmniSim robot as ordinary hardware and stock
controllers drive it without knowing OmniSim exists.

| | |
|---|---|
| **State** | harness `GET /robot/<def>/joints` → `position` + `velocity` state interfaces |
| **Commands** | bridge `POST /set_velocity` (`command_mode: diff_drive`); `command_mode: joint_positions` streams to `POST /servo_joint_positions` when the bridge advertises `capabilities.servo`, else falls back to the goal verb `POST /set_joint_positions` |
| **Verified** | `diff_drive_controller` + `joint_state_broadcaster` on the Husky |
| **Unverified** | `joint_trajectory_controller`, and therefore **MoveIt** — unblocked by the servo verb, but no end-to-end bringup has been run; see below |

Neither endpoint is new; the plugin adds no OmniSim-side surface at all.

```bash
ros2 launch omnisim_ros2_control husky_diff_drive.launch.py
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel_unstamped \
    geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

**⚠ `update_rate` is not the actuation rate.** `read()` and `write()` do no I/O —
they swap values with a snapshot under a mutex — and one background thread owns
the HTTP at `comms_rate_hz`. A 50 Hz controller manager over a 25 Hz link is a
50 Hz controller reading a 25 Hz sensor. Measured ceiling below.

**✅ The arm-bridge blocker is fixed; the joint path now targets the servo
lane.** `omnilink_arm_bridge` gained `POST /servo_joint_positions`
(PROTOCOL.md §6.1): non-blocking, last-write-wins, preempts an in-flight goal,
never a 409 for servo-on-servo — verified bridge-side on
`omnilink_ur5e.omniworld` (24 setpoints at ~18 Hz, 24/24 accepted, zero 409s,
parked max error 0.011 rad). `command_mode: joint_positions` reads the bridge's
`/capabilities` once on its comms thread and streams to the servo verb when
`capabilities.servo` is advertised, logging the chosen lane.

The **goal verb's contract is unchanged** and still measured true — a second
`set_joint_positions` arriving while the previous interpolation is still
running is refused. Measured 2026-08-17 against `omnilink_ur5e.omniworld`,
three commands 50 ms apart:

```console
  #1 POST /set_joint_positions wait=false -> HTTP 200  accepted=True
  #2 POST /set_joint_positions 50 ms later -> HTTP 409  accepted=False  error='busy'
     message: a set_joint_positions is already in flight; this set_joint_positions
              was NOT applied and the arm is still executing the previous one.
  #3 POST /set_joint_positions 50 ms later -> HTTP 200  accepted=True  error=None
```

Note #3: the refusal is **intermittent**, which is worse than a hard failure — a
trajectory lands in pieces. That mechanism now applies to the **fallback lane
only** (an older bridge, or any bridge without `capabilities.servo`); the plugin
still names it on a 409 there. On the servo lane a 409 is *not* an expected
condition — it would mean the bridge is not honouring its own advertisement, and
the plugin says so instead of blaming the stream. ⚠️ **MoveIt itself has still
never been brought up against OmniSim** — the servo lane unblocks a
`joint_trajectory_controller`, but no end-to-end run exists; treat arms as
unblocked, never as working.

**Nav2**: `/odom`, `/cmd_vel`, `/tf`, `use_sim_time` and now `ros2_control` all
exist, which is everything a Nav2 bring-up consumes — but **no Nav2 stack has
been brought up against OmniSim**, so treat it as unblocked, not as working.
OmniSim is also still absent from the `ros2_control` simulator registry.

---

## Quickstart

Needs **ROS 2 Humble or newer** (verified on Humble) and a running harness.

```bash
# 1. OmniSim side
python -m omnisim harness
curl -s -X POST http://127.0.0.1:6789/world/load \
  -H "Content-Type: application/json" \
  -d '{"path":"projects/samples/demos/worlds/chat/omnilink_husky.omniworld","light":true}'

# 2. ROS 2 side — fetches simulation_interfaces (pinned 2.1.0) and builds
bash packages/omnisim-ros2/scripts/bootstrap.sh
source packages/omnisim-ros2/install/setup.bash

# 3. one command for the whole surface
ros2 launch omnisim_ros2 omnisim_bringup.launch.py
```

⚠ **Do not add `--auto-port` here.** That flag exists to move the harness *off*
`:6789` when the pair is already taken, and it prints the pair it actually chose
to **stderr** — so pairing it with a hard-coded `127.0.0.1:6789` gives you a
`curl` that talks to the other harness, or to nothing. Add it only when you are
deliberately running a second harness alongside an existing one; then read the
chosen port off stderr and carry it everywhere: `curl` it, and pass it to ROS as
`harness_url:=http://127.0.0.1:<port>` on the launch, or as `OMNISIM_HARNESS_URL`
in the environment (which `harness_client.py` reads).

Harness-only (no robot bridge): `... omnisim_bringup.launch.py bridge:=false`.
Tier 1 alone: `ros2 launch omnisim_ros2 simulation_interfaces.launch.py`.
**Tier 3** (`ros2_control` + `diff_drive_controller`) is its own bring-up:
`ros2 launch omnisim_ros2_control husky_diff_drive.launch.py`.

⚠ **Do not run the Tier 3 launch and `omnisim_bringup.launch.py` against the
same robot.** Three collisions, all silent: `diff_drive_controller` and
`odom_node` both publish `odom -> base_link`; `joint_state_broadcaster` and
`robot_state_node` both publish joint states *and* both poll
`GET /robot/<def>/joints`, whose velocity is finite-differenced between the
harness's own successive reads — so two independent pollers halve each other's
`dt` and each sees a velocity that is a function of the other's polling; and
`command_node` forwards `/cmd_vel` straight to the bridge, fighting the
controller for the same actuator. Run `simulation_interfaces.launch.py` alongside
Tier 3 if you want the standard's services — that node is service-only.

---

## Worked example — real captured output

All of the following was captured 2026-08-17 against
`projects/samples/demos/worlds/chat/omnilink_husky.omniworld`, ROS 2 Humble.

### Tier 1: spawn, read back, move, step

```console
$ ros2 service call /omnisim_simulation_interfaces/spawn_entity \
    simulation_interfaces/srv/SpawnEntity \
    "{name: 'ROS2_BOX', entity_resource: {resource_string: 'Solid { children [ Shape { appearance PBRAppearance { baseColor 1 0 0 } geometry Box { size 0.3 0.3 0.3 } } ] boundingObject Box { size 0.3 0.3 0.3 } physics Physics { } }'},
      initial_pose: {pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}}"
response:
SpawnEntity_Response(result=Result(result=1, error_message='spawned at [2.0, 1.0, 1.5]'),
                     entity_name='ROS2_BOX')

$ ros2 service call ... GetEntities "{}"
GetEntities_Response(result=Result(result=1, error_message=''),
                     entities=['SUN_MARKER', '#10', 'HUSKY', 'ROS2_BOX'])

$ ros2 service call ... SetEntityState \
    "{entity: 'ROS2_BOX', set_pose: true,
      state: {pose: {position: {x: -2.0, y: -1.0, z: 2.0},
                     orientation: {z: 0.7071068, w: 0.7071068}}}}"
SetEntityState_Response(result=Result(result=1,
                        error_message='placed at world position [-2.0, -1.0, 2.0]'))

$ ros2 service call ... GetEntityBounds "{entity: 'ROS2_BOX'}"     # a 0.3 m box
bounds=Bounds(type=1, points=[Vector3(x=-0.15, y=-0.15, z=-0.15),
                              Vector3(x=0.15, y=0.15, z=0.15)])

$ ros2 action send_goal -f ... SimulateSteps "{steps: 25}"
Feedback:  completed_steps: 10   remaining_steps: 15
Feedback:  completed_steps: 20   remaining_steps: 5
Feedback:  completed_steps: 25   remaining_steps: 0
Goal finished with status: SUCCEEDED
```

Errors are typed, not prose:

```console
$ ... GetEntityState "{entity: 'NO_SUCH_THING'}"
result=Result(result=2, error_message="no entity named 'NO_SUCH_THING'")   # NOT_FOUND

$ ... SetSimulationState "{state: {state: 2}}"                             # PAUSED
result=Result(result=0, error_message="OmniSim's engine free-runs and the harness
  exposes no pause verb; SIMULATION_STATE_PAUSE is not advertised in
  GetSimulatorFeatures")                                          # FEATURE_UNSUPPORTED
```

### Tier 2: topics

```console
$ ros2 topic list
/clock
/cmd_vel
/husky/joint_states
/joint_command
/odom
/parameter_events
/rosout
/tf
/tf_static

$ ros2 topic echo /clock --once
clock:
  sec: 557
  nanosec: 744000000

$ ros2 topic echo /husky/joint_states --once
header: {stamp: {sec: 573, nanosec: 696000000}, frame_id: HUSKY}
name: [front_left_wheel_motor, front_right_wheel_motor,
       rear_left_wheel_motor, rear_right_wheel_motor]
position: [4.84e-09, 7.28e-08, -8.57e-09, -7.19e-08]
velocity: [-1.46e-10, -1.53e-10, -1.46e-10, -1.53e-10]
effort: []

$ ros2 run tf2_ros tf2_echo world HUSKY
At time 621.696000000
- Translation: [0.000, -0.000, 0.132]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
```

`0.132` is the Husky's authored ride height, and matches `GET /robots` exactly.

### Tier 2: the robot actually moves

```console
$ ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
   x: +0.0000 -> +0.2846    moved +0.2846 m
   watchdog fires: 1        node crashes: 0
```

A second run at `linear.x=0.5` drove the Husky from `x=0.0000` to **`x=5.5918`**,
where it stopped against the arena wall.

⚠ **Do not read a wall-clock command duration as a distance.** The harness runs
worlds with `--mode=fast`: measured on this machine, simulation time advanced
**39.920 s in 3.015 s of wall time — 13.24× realtime**. A command published for
4 wall-seconds is therefore live for ~53 simulated seconds. That is why the
`0.5 m/s` run hit a wall 5.59 m away instead of covering 2 m, and why the honest
low-speed number above is quoted with its own window rather than as a
"commanded vs achieved" check.

### Tier 2: sensor topics, and they respond to motion

A topic publishing zeros forever is worse than no topic, so the bar here is
that values **change when the robot moves**. Captured 2026-08-17 with
`tools/sample_sensor_topics.py` on either side of a real motion:

```console
$ ros2 topic list -t
/gps/local [geometry_msgs/msg/PointStamped]
/imu/data [sensor_msgs/msg/Imu]
/scan [sensor_msgs/msg/LaserScan]
/tf_static [tf2_msgs/msg/TFMessage]

--- BEFORE ---
  imu/data     quat=(+0.0000,+0.0000,+0.0000,+1.0000) yaw=+0.0000 rad  ang_vel_cov[0]=-1 lin_acc_cov[0]=-1
  scan         n=541 finite=541 inf=0 min=5.828 max=6.509  fov=[-2.356,+2.356] range=[0.10,20.00] frame=base_laser
  gps/local    x=+0.0000 y=-0.0000 z=+0.1320 frame=world

>>> set_velocity linear=0.5
--- AFTER DRIVING FORWARD ---
  imu/data     quat=(+0.0000,+0.0000,+0.0000,+1.0000) yaw=+0.0000 rad  ang_vel_cov[0]=-1 lin_acc_cov[0]=-1
  scan         n=541 finite=190 inf=351 min=5.438 max=6.486  fov=[-2.356,+2.356] range=[0.10,20.00] frame=base_laser
  gps/local    x=+5.5918 y=-0.0000 z=+0.1320 frame=world

>>> turn 0.9        (skid-steer delivered 0.130 rad; bridge yaw=0.1300)
--- AFTER TURNING ---
  imu/data     quat=(-0.0000,-0.0000,+0.0650,+0.9979) yaw=+0.1300 rad  ang_vel_cov[0]=-1 lin_acc_cov[0]=-1
  scan         n=541 finite=192 inf=349 min=5.364 max=6.576  fov=[-2.356,+2.356] range=[0.10,20.00] frame=base_laser
  gps/local    x=+5.5508 y=-0.0028 z=+0.1320 frame=world
```

Reading that honestly: the IMU is **correct** to stay at identity through the
straight drive (`y` never left `0.0000`), and it tracks the turn to 4 decimals
against the bridge's own yaw — `+0.1300` vs `0.1300`, with `z=0.0650` giving
`2·asin(0.0650) = 0.1301 rad`. The lidar responds to both translation and
rotation. The `-1` covariances are the gyro and accelerometer being declared
absent, every tick, by design.

`/tf_static` carries the measured mounts:

```console
$ ros2 topic echo --once /tf_static
  frame_id: base_link   child_frame_id: base_laser
  translation: {x: 0.2012, y: 0.0, z: 0.505}
  frame_id: base_link   child_frame_id: imu_data
  translation: {x: 0.0, y: 0.0, z: 0.0}
```

**Control run, without `OMNISIM_URDF_USE_SENSORS=1`:** the same world reports
`{"robot": "HUSKY", "devices": []}`, and `/read_sensor` answers
`available: false` naming the four wheel `PositionSensor`s that do exist. That
is the whole difference the flag makes, and it is why the warning above is
first.

### Tier 3: `ros2_control` on the Husky

```console
$ ros2 launch omnisim_ros2_control husky_diff_drive.launch.py
[resource_manager]: Loading hardware 'OmniSimHusky'
[OmniSimSystem.OmniSimHusky]: OmniSim hardware 'OmniSimHusky': 4 joints, robot_def=HUSKY,
  state <- http://127.0.0.1:6789/robot/HUSKY/joints,
  commands -> http://127.0.0.1:8766 (mode=diff_drive), comms_rate=10.0 Hz
[OmniSimSystem.OmniSimHusky]: activated; joint state is live
[controller_manager]: update rate is 50 Hz
[spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[spawner_diff_drive_controller]:   Configured and activated diff_drive_controller

$ ros2 control list_hardware_interfaces
command interfaces
	front_left_wheel/velocity [available] [claimed]
	front_right_wheel/velocity [available] [claimed]
	rear_left_wheel/velocity [available] [claimed]
	rear_right_wheel/velocity [available] [claimed]
state interfaces
	front_left_wheel/position
	front_left_wheel/velocity
	front_right_wheel/position
	front_right_wheel/velocity
	rear_left_wheel/position
	rear_left_wheel/velocity
	rear_right_wheel/position
	rear_right_wheel/velocity

$ ros2 control list_controllers
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active
diff_drive_controller   diff_drive_controller/DiffDriveController      active

$ ros2 control list_hardware_components
Hardware Component 1
	name: OmniSimHusky
	type: system
	plugin name: omnisim_ros2_control/OmniSimSystem
	state: id=3 label=active
```

And the robot moves. Pose and simulation clock are read from the **simulator**
— one `POST /get_robot_state` on the robot's own controller, which reports the
supervisor's pose — never from the command that was sent:

```console
$ ros2 topic pub -r 20 /diff_drive_controller/cmd_vel_unstamped \
    geometry_msgs/msg/Twist "{linear: {x: 0.05}}"     # for 10 s of wall time

BEFORE  sim_t=16681.82s  x=-0.00000  y=+0.00000
AFTER   sim_t=16877.04s  x=+5.59211  y=-0.00000
MOVED   5.5921 m over 195.22 s of simulated time
```

`5.59211` is the arena wall, and it is the same stopping x the Tier 2 `cmd_vel`
run above reached — the robot drove into it from the origin.

**Is the ros2_control layer faithful?** Three A/B repetitions, alternating, each
starting from a recentred pose, `diff_drive_controller` deactivated for the
control arm so nothing else touched the bridge:

| arm | median steady speed, reps 1–3 | vs commanded 0.05 m/s |
|---|---|---|
| **A** `diff_drive_controller` → plugin → bridge | 0.0380 · 0.0374 · 0.0440 m/s | −24.0% · −25.2% · −12.1% |
| **B** control: `POST /set_velocity` direct, no ROS | 0.0428 · 0.0409 · 0.0402 m/s | −14.4% · −18.2% · −19.6% |

The two arms agree to within ~4%, so **the shortfall is the robot and its bridge,
not the ROS layer** — it is present with ROS removed entirely. The plugin's fold
is exact and was verified live at `DEBUG`: `left=0.302847 right=0.302847
r=0.165100 b=0.570800 -> linear=0.050000 angular=0.000000` for a commanded
0.05 m/s.

⚠ **Two traps this measurement walked into, both worth stealing.**

1. **Do not read the clock and the pose from two different harness calls.**
   `sim_time_ms` comes from `GET /sim/state` and the pose from `GET /robots` —
   two supervisor RPCs, each serviced at a different engine step boundary. Under
   `--mode=fast` the gap between them is *seconds of simulated time* and it moves
   with HTTP load, so the same 10-second drive read **+50.6%** and **−21.8%**
   depending only on how busy the harness was. Both figures were artefacts. Take
   the clock and the pose from one response.
2. **A wheel cannot exceed its velocity setpoint.** That is what exposed trap 1:
   an "achieved" wheel speed of 0.4565 rad/s against a 0.30285 rad/s setpoint is
   not a fast robot, it is a bad denominator. If a measured actuator beats its
   own command, suspect the instrument before the physics.

---

## Disclosed divergences from the standard

Declared through the feature flags **and** repeated in
`GetSimulatorFeatures.custom_info`, so a caller learns them from the API.

1. **There is no pause.** OmniSim's engine free-runs and the harness exposes no
   pause verb. `SIMULATION_STATE_PAUSE` is not advertised; `GetSimulationState`
   answers `STATE_PLAYING` whenever a world is loaded; `StepSimulation` means
   "advance at least N basic steps", not "exactly N from a frozen state".
2. **`EntityState.twist` / `.acceleration` are not measured** and are returned as
   zeros — *unmeasured*, not observed-to-be-zero. Real velocities are on `/odom`
   and in `JointState.velocity`.
3. **`JointState.velocity` is finite-differenced by the harness**, not read from
   the engine, and is `null` on a joint's first read. When any joint's velocity is
   unknown the array is published **empty** — the ROS idiom for "not provided" —
   rather than a zero that reads as "stationary". `effort` is never published.
4. **`ResetSimulation` re-pins every motor** to a position hold; the harness's
   warning is passed through verbatim in `result.error_message`.
5. **`DeleteEntity` removes a node from the scene graph, not from the physics
   model.** Deleted geometry keeps blocking rays and contacts until a reload.
6. **`GetEntityBounds` returns a world-axis-aligned box** expressed relative to
   the entity origin — exact for an axis-aligned entity, approximate for a rotated
   one. The result message says which.
7. **`/odom` is ground truth, not a sensor model.** The pose comes from the
   simulator, so it has no drift — ideal for bringing a nav stack up, wrong for
   evaluating a localisation algorithm. Covariances are left at zero, which in
   ROS convention means *unknown*, not *perfectly certain*.

---

## Known gaps

- **Tier 3 is verified for velocity-commanded bases only.** `diff_drive_controller`
  is verified on the Husky. The joint-position path now streams to the arm
  bridge's servo verb when advertised (Tier 3 above), which unblocks
  `joint_trajectory_controller` — but **no trajectory controller or MoveIt
  bringup has been run end-to-end**, so treat arms as unblocked, not working.
  A Nav2 bring-up has never been run against OmniSim, and OmniSim is not in the
  `ros2_control` simulator registry.
- **No effort interfaces anywhere.** OmniSim exposes no joint effort on any
  surface, so `effort` is neither a state nor a command interface, and the plugin
  rejects a URDF that asks for one instead of exporting a fabricated zero.
- **Sensor topics ship for orientation, range and position only** (see Tier 2
  above). Three device types are genuinely unavailable and are declared as such
  rather than published as zeros:
  - **`Gyro` reads a constant `[0,0,0]`** even while the robot rotates, so
    `Imu.angular_velocity` carries `covariance[0] = -1`.
  - **`Accelerometer` never produces a sample**, so `Imu.linear_acceleration`
    carries `covariance[0] = -1`.
  - **No camera anywhere in the tree**, so no `Image` / `CameraInfo`. Adding a
    `<sensor type="camera">` to a URDF is enough for the importer; the ROS side
    would then need an image-transport path, which `/read_sensor` deliberately
    does not carry.
  Only `omnilink_mobile_bridge` implements `/read_sensor`; other bridges have no
  sensor surface, so check `capabilities.sensors` rather than assuming.
- **Sensors need `OMNISIM_URDF_USE_SENSORS=1` at world load** on any URDF robot,
  or the device set is empty. This is an engine-side importer gate, not a ROS one.
- **Not advertised, because not implemented**: `NAMED_POSES`, `POSE_BOUNDS`,
  `ENTITY_BOUNDS_CONVEX`, `ENTITY_INFO_SETTING`, `SPAWNABLES`,
  `SPAWNING_ENTITIES`, `WORLD_UNLOADING`, `WORLD_TAGS`, `AVAILABLE_WORLDS`,
  `WORLD_RESOURCE_STRING`.
- **`WorldInfo.coordinateSystem` is not queryable over the harness.** OmniSim
  defaults to Z-up/ENU, which is exactly REP-103, so no axis remapping is applied.
  A world overriding it to `NUE` cannot be detected at runtime.
- **Entities are root-level posed nodes.** Child links are not entities.
- **One bridge per `command_node`/`odom_node`.** Multi-robot needs one instance
  per robot, in its own namespace.

### ⚠ Publish rates cost TCP connections — but the bridge no longer does

**Fixed for bridge traffic (2026-08-17).** `omnilink_mobile_bridge` now sets
`protocol_version = "HTTP/1.1"`, and the client pools one connection per node
instead of calling `urllib.urlopen` (which opens a new socket per request and
offers no way to reuse one). Measured A/B against the live bridge, 300 requests,
machine `9722d23d12a3`:

| mode | throughput | per request | `TIME_WAIT` sockets added |
|---|---|---|---|
| connection per request *(old)* | 114.8 req/s | 8.71 ms | **+300** |
| pooled keep-alive *(now)* | **908.8 req/s** | 1.10 ms | **+0** |

That is 7.9× throughput and, more importantly, it removes socket consumption
from bridge polling entirely — which is what made rates dangerous. Each closed
socket sits in `TIME_WAIT` for 120 s against Windows' 16,384-port ephemeral
range; a ~50 Hz bringup previously measured **17,487 sockets in `TIME_WAIT`**
before `connect()` began failing with `WinError 10048` (which reads like a bind
conflict but is exhaustion).

⚠ **The World Harness still speaks HTTP/1.0**, so `clock_node`,
`robot_state_node` and `simulation_interfaces_node` still pay one connection per
request. The pool degrades to the old behaviour automatically against such a
peer (`resp.will_close`), so nothing breaks — it just costs what it always did.
The bringup defaults (20 Hz clock + 5 Hz state + 10 Hz odom + 10 Hz sensors)
stay inside the budget; the sensor node is the heaviest client at **one request
per sensor per tick**, and it is the one that got cheap.

`OMNISIM_ROS2_KEEPALIVE=0` restores a fresh connection per request for an A/B.

### ⚠ The `ros2_control` ceiling is the STATE READ, not the transport

Measured 2026-08-17, machine `9722d23d12a3` (RTX 3060 laptop, CPU `mj_step`,
binary `13906cc6f12451eb`), `omnilink_husky.omniworld` loaded `light`, one
request at a time from Windows loopback:

| call | what it costs | mean | p50 | sustained |
|---|---|---|---|---|
| harness `GET /healthz` | pure transport, never touches the simulator | 4.48 ms | 0.94 ms | 221.8 req/s |
| harness `GET /robot/HUSKY/joints` | **supervisor RPC, serviced at an engine step boundary** | **21.01 ms** | 22.06 ms | **47.5 req/s** |
| bridge `POST /get_robot_state` | robot's own controller | 6.23 ms | 1.53 ms | 159.6 req/s |
| bridge `POST /set_velocity` | robot's own controller | 3.60 ms | 1.06 ms | 275.2 req/s |

The joint read is **4.7× the cost of a bare HTTP round trip on the same server**,
so what limits a control loop here is OmniSim's supervisor round trip, not TCP.
One read + one write is ~22 ms, i.e. a **hard ceiling near 45 Hz** — and that is
before the ROS side is on another host. Through `tools/wsl_harness_link.py` the
same two calls measured 18.48 ms and 7.46 ms (54.0 and 132.8 req/s), so the
tunnel is not the binding constraint either.

Live confirmation from the plugin's own diagnostics at `comms_rate_hz: 10`, over
a 15-minute session:

```console
[OmniSimSystem.OmniSimHusky]: comms 10.000 Hz (target 10.000) | state RTT 18.571 ms |
  cmd RTT 0.872 ms | 9255 requests over 6916 TCP connections
  (keep-alive: harness NO, bridge yes) | 2340 commands sent | failures state=0 cmd=0
```

Read that connection count: ~6800 state reads cost ~6800 sockets (the harness is
HTTP/1.0) while 2340 commands cost ~100 (the bridge grants keep-alive). At the
Windows budget of ~136 new connections/s that is a *second* ceiling at ~136 Hz —
comfortably above the 45 Hz the supervisor imposes, so **latency binds first**.

**Practical settings**: `comms_rate_hz: 25` (the shipped default) leaves ~45%
headroom; the launch file lowers it to 10 through the WSL tunnel. Raising
`update_rate` above `comms_rate_hz` costs nothing and buys nothing.

---

## Windows + WSL2

OmniSim's engine is Windows-primary; ROS 2 is Linux. The natural pairing is
"engine + harness on Windows, ROS 2 in WSL2", and it has one obstacle: **WSL2
cannot open a TCP connection to a Windows-host service** without an Administrator
firewall rule. The reverse direction works.

[`tools/wsl_harness_link.py`](tools/wsl_harness_link.py) inverts the direction so
no admin rights are needed: the WSL half listens, the Windows half dials in and
parks connections that become return paths. It is a byte pump, not an HTTP proxy,
so it never parses or rewrites the stream and every harness endpoint works.

```bash
# inside WSL — note the DIFFERENT local port, see the warning below
python3 tools/wsl_harness_link.py wsl --listen-port 6789 --tunnel-port 7799 &
python3 tools/wsl_harness_link.py wsl --listen-port 8766 --tunnel-port 7800 &

# on Windows (get the IP with: wsl -d Ubuntu-22.04 -- hostname -I)
python tools\wsl_harness_link.py windows --wsl-host 192.168.209.179 \
    --tunnel-port 7799 --harness-port 6889
python tools\wsl_harness_link.py windows --wsl-host 192.168.209.179 \
    --tunnel-port 7800 --harness-port 8765

ros2 launch omnisim_ros2 omnisim_bringup.launch.py \
    bridge_url:=http://127.0.0.1:8766
```

⚠ **Never give the WSL listener the same port number as the Windows service.**
WSL2 mirrors a WSL listener onto Windows `localhost`, so a WSL listener on `8765`
**shadows the real bridge on Windows `127.0.0.1:8765`** and the tunnel ends up
forwarding to itself. Measured: `POST /get_robot_state` on Windows returned
nothing until the WSL side was moved to `8766`. Pick distinct ports on each side.

⚠ **Halve your publish rates through the tunnel.** It costs *two* Windows-side
connections per request (the dial into WSL plus the loopback connect to the
service). Windows' default 16384-port range and 120 s `TcpTimedWaitDelay` cap the
host at ~136 new connections/sec; exceeding it fails with `WinError 10048`, which
reads like a bind conflict but is exhaustion, and shows up as nodes reporting a
perfectly healthy harness as unreachable.

**A cleaner alternative if it suits your setup:** WSL2's *mirrored* networking
mode (`networkingMode=mirrored` in `%USERPROFILE%\.wslconfig`, Windows 11 22H2+)
makes WSL share the Windows network namespace, so `127.0.0.1:6789` in WSL *is*
the Windows service and no tunnel is needed. It is a global WSL setting, so it is
not the default recommendation here.

**None of this applies on a same-host Linux deployment** — point the nodes
straight at the services with `harness_url:=` / `bridge_url:=`.

---

## Tests

```bash
cd packages/omnisim-ros2/src/omnisim_ros2 && python3 -m pytest test/ -q
```

**96 tests, no ROS node, no simulator, no network beyond loopback.** They cover:

- **Rotation conversions**, including the 180° branch where the naive trace
  formula suffers catastrophic cancellation, and the two encodings OmniSim uses
  under confusingly similar names (9-element matrix on reads, 4-element
  axis-angle on writes).
- **TF composition** — recomposing parent ⊗ relative must recover the child's
  world pose.
- **Entity selection** against a *captured real* `/scene/tree` body, including the
  measured quirk that the harness reports `harness_injected: []` even when its
  supervisor is present.
- **The HTTP clients against an in-process HTTP server**, so the "return 4xx
  bodies instead of raising" contract is exercised over real sockets.
- **Joint-command verb selection** (`bridge_servo_verb`, and its C++ twin
  `servo_verb_path` under the gtest suite) against the arm bridge's real
  `/capabilities` shape — both verbs accept the *first* command, so a wrong
  pick only surfaces as intermittent 409s under a stream.
- **Connection reuse**, by counting the distinct client ports the server saw —
  the property that matters is "one connection for many requests", not latency.
  Includes the case that would otherwise surface as a random dead node: a peer
  hanging up a pooled socket between ticks must be retried transparently, and a
  server that closes every response (the harness today) must still work.
- **Lidar layer selection and the null→`inf` mapping**, including that a
  no-return never becomes `0.0` and that an out-of-range `lidar_layer` clamps
  instead of raising `IndexError` inside a timer callback.

The end-to-end check against a live harness is the worked example above.

---

## Layout

```
packages/omnisim-ros2/
├── deps.repos                 # simulation_interfaces, pinned to 2.1.0
├── scripts/bootstrap.sh       # fetch deps + colcon build
├── tools/wsl_harness_link.py  # Windows<->WSL2 transport (no admin needed)
├── tools/sample_sensor_topics.py  # one-line summary per sensor topic
└── src/omnisim_ros2/
    ├── omnisim_ros2/
    │   ├── harness_client.py  # stdlib HTTP client + connection pool
    │   ├── bridge_client.py   # stdlib HTTP client for a per-robot bridge
    │   ├── conversions.py     # the rotation-encoding trap, contained
    │   ├── entities.py        # scene node <-> simulation_interfaces entity
    │   ├── node_support.py    # keeps a transport hiccup from killing a node
    │   ├── simulation_interfaces_node.py   # Tier 1
    │   ├── clock_node.py · robot_state_node.py
    │   ├── command_node.py · odom_node.py  # Tier 2
    │   └── sensor_node.py     # Tier 2 — Imu / LaserScan / GPS
    │   ├── launch/
    │   └── test/
    └── omnisim_ros2_control/       # Tier 3 (ament_cmake, C++)
        ├── src/omnisim_system.cpp  # the SystemInterface itself
        ├── src/http_client.cpp     # blocking HTTP/1.1, keep-alive if granted
        ├── src/json_lite.cpp       # ~200-line reader, no external dependency
        ├── description/husky_omnisim.urdf.xacro   # kinematics + <ros2_control>
        ├── config/husky_controllers.yaml
        ├── launch/husky_diff_drive.launch.py
        └── test/                   # gtest: JSON reader + diff-drive fold
```

`harness_client.py`, `bridge_client.py`, `conversions.py`, `entities.py` and
`node_support.py` deliberately **do not import ROS**, so they are unit-testable
and reusable without a ROS environment.

## See also

- [docs/developer/ros2-integration.md](../../docs/developer/ros2-integration.md) — the policy history and full picture
- [PROTOCOL.md](../../PROTOCOL.md) — the OmniSim Wire Protocol this sits on
- [scripts/harness/README.md](../../scripts/harness/README.md) — the harness itself
