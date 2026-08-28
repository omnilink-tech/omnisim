# RoboLife — modular robots that scavenge, charge, fabricate and fail

The flagship successor to `projects/alife/` (read its README: every engine
trap it documents applies here). Biology is out. Every individual is a **real
robot** — the Clearpath Husky, imported from its shipped URDF, with real
devices — living under real physics: batteries drain with mechanical work,
mass costs energy and momentum, parts are physical bodies that dock by a
native weld and come off on impact. The "game of life" is what emerges:
robots that manage charge, scavenge parts, fabricate successors, and die.

This file is the binding contract for three parallel implementers (A, B, C).
Where the code and this file disagree, fix one in the same change.

## Hard engine facts (measured; do not re-derive)

- `URDFRobot { url }` takes ONLY url/translation/rotation/name/controller/
  controllerArgs/supervisor/customData/staticBase/physicsBackend/window —
  **no extra children**. So the Husky is used EXPANDED: `python
  scripts/dev/urdf_import.py projects/robots/clearpath/husky_description/urdf/husky.urdf --to <f>`
  emits a plain `Robot { ... }` (46.03 kg `inertial_link`, four `HingeJoint`
  wheels with `RotationalMotor` `*_wheel_motor`, `InertialUnit imu_data`,
  `GPS navsat_fix`, `Gyro`, `Accelerometer`) whose children list we may
  extend. Its mesh URLs come out ABSOLUTE (`O:/omnisim/...`) — rewrite them to
  the repo-relative form the shipped worlds use (`omnisim://projects/...` or a
  relative path) before committing.
- Runtime spawn/delete have no physics → every robot and every module is
  authored at load and POOLED (parked on a far static slab; revive = plain
  teleport, **never `setVelocity`** — it freezes the body ~2 s).
- `Connector` welds are native MuJoCo equality welds, **default ON**
  (`OMNISIM_NEWTON_WELDS` unset = on, `OmConnector.cpp:53`). Locking is a
  robot DEVICE call (`connector.lock()` in the robot's own controller) when a
  compatible connector is within `distanceTolerance` / `axisTolerance` /
  `rotationTolerance`; `getPresence()` reports 1 when one is. `unlock()`
  releases. Unverified until A's probe: that the weld carries a module while
  the robot drives, and that unlock frees it.
- `WorldInfo.newtonRobotColliders TRUE` or the chassis collider becomes a
  1 mm sphere and robots phase through walls. Floor = Box. Canonical lighting.
  dt 8, `newtonSolver "mujoco"`, `newtonGroundMu 1.5`, ke 8000 / kd 200,
  elliptic, impratio 10 — identical to alife.
- Wheels are VELOCITY joints (no position limits) — the supervisor's batched
  field-write trick cannot drive them. Therefore **each robot runs its own
  controller process** (`robolife_robot`), exactly like a real robot. One
  no-op controller costs ~0.22 ms/tick of IPC; 8 robots ≈ 2 ms. Physics for
  8 Huskies measured 0.61 ms/tick. Realtime budget 8 ms/step holds.
- Supervisor ↔ robot communication is the Robot node's `customData` field:
  the supervisor writes it (batched field write, cheap), the robot reads it
  with `robot.getCustomData()`; the robot answers with `setCustomData()`,
  which the supervisor reads via the field. This is the modelled radio link.
- Ray sensors (`Lidar`, `DistanceSensor`) work on the CPU `mj_step` path.
- Tool-design rule: every reported motion/dock result carries MEASURED state
  (`{commanded, achieved, error}`), never the argument echoed back.

## Layout (owner in brackets)

```
projects/robolife/
  DESIGN.md                      this file
  robots/husky_base.txt      [A] expanded Husky Robot{} body, relative URLs,
                                 with the RoboLife additions (below)
  rl/worldgen.py             [A] fleet + module pool + scene -> .omniworld
  rl/modules.py              [A] module catalogue (type -> geometry, mass,
                                 effect); shared by B and C
  probe_dock.py + controllers/robolife_probe_dock/   [A]  the docking gate
  rl/brain.py                [B] PURE robot logic (state machine, docking
                                 geometry, diff-drive kinematics), tested
  controllers/robolife_robot/robolife_robot.py       [B]  the robot program
  rl/energy.py               [C] PURE energy/impact/fabrication rules, tested
  controllers/robolife_world/robolife_world.py       [C]  the supervisor
  rl/scene.py                [C] arena, pads, bay, crypt, lighting, viewpoint
  robolife.py                [C] epoch driver;  watch_robolife.py [C] launcher
  tests/                     [B, C]
```

## The robot (A authors it in `husky_base.txt`; B drives it)

Expanded Husky plus, as extra children of the Robot:
- `DEF <name>_SOCKET_F Connector { name "socket_front" type "active"
  translation 0.55 0 0.25 rotation 0 0 1 0 (normal +x) distanceTolerance 0.08
  axisTolerance 0.45 rotationTolerance 0.6 numberOfRotations 4
  boundingObject NULL physics NULL }` — a second socket `socket_rear` at
  `-0.55 0 0.25` facing −x (rotation 0 0 1 3.14159).
- `Lidar { name "lidar" translation 0 0 0.45 horizontalResolution 180
  fieldOfView 3.14159 numberOfLayers 1 minRange 0.2 maxRange 8 noise 0.01 }`
  — mounted above module height so a docked front module does not blind it.
- `customData ""`, `controller "robolife_robot"`, `controllerArgs [ "--slot" "<i>" ]`,
  `supervisor FALSE`. DEF `ROBOT_<i>`, name `robot_<i>`.
- Keep the URDF's own IMU/GPS device names; B navigates with GPS + IMU yaw.

Wheel convention (B measures rather than trusts): motors
`front_left_wheel_motor, front_right_wheel_motor, rear_left_wheel_motor,
rear_right_wheel_motor`, `setPosition(inf)` then `setVelocity(rad/s)`; Husky
wheel radius 0.1651 m, track 0.555 m. The shipped
`omnilink_mobile_bridge` had a 56.7% turn error before its control law was
rewritten — measure `{commanded, achieved}` yaw/distance in the probe.

## Modules (`rl/modules.py`)

Physical bodies, pooled: `DEF MODULE_<j> Solid` with `physics`, a Box or
Cylinder collider, a distinct colour per type, and a **passive** `Connector`
`name "plug"` on its −x face at height 0.25 facing −x (so a robot's front
socket facing +x mates when the robot drives up to that face). Same
tolerances as the sockets.

| type | geometry | mass | effect (C applies it) |
|---|---|---|---|
| `battery` | box 0.34×0.30×0.24, dark green | 6 kg | +50 % capacity, +0.5 W idle draw |
| `solar` | box 0.40×0.40×0.06 + post, blue | 2.5 kg | +6 W while sunlit (always, v1) |
| `mast` | cylinder r0.05 h0.6, orange | 1.5 kg | detection range 6 → 12 m |
| `armor` | box 0.36×0.30×0.20, grey | 5 kg | impact threshold ×2 |

Parked modules sit on the crypt slab; loose modules are teleported into the
arena by the supervisor (initially and when scattered by a death/impact).
A robot carries at most 2 (one per socket). Mass is real: a 46 kg Husky
towing 11 kg of modules accelerates, brakes and drains differently.

## Energy and lifecycle (`rl/energy.py`, applied by C)

- Battery: capacity `C = 200 Wh · (1 + 0.5·n_battery_modules)`; start 60 %.
  Drain per second = `2 W idle + 0.5 W·n_battery + 0.9 · m_total[kg] ·
  |v|[m/s] · 0.02 + 40 W · |ω_wheels_mean|/10` — i.e. rolling work scales with
  total mass and speed. Solar +6 W each. Reported in Wh; sim seconds × 3600.
  (Tune so an idle robot lasts ~25 min and a cruising bare robot ~4 min.)
- Charging pad: robot centre within 0.9 m of a pad → +60 W. 3 pads.
- Impact: supervisor differences each robot's GPS velocity per tick; `|Δv| >
  0.9 m/s in one tick` (×2 with armor) → order `release` of one module
  (rear first); the module is then loose. Robot-robot and robot-wall hits
  both count; it is just momentum.
- Death: charge ≤ 0 → order `stop`; after 20 s its modules are ordered
  released and the chassis is parked (pooled). Modules stay loose in the
  arena — scavengeable.
- Fabrication: charge ≥ 85 % of capacity AND within 1.5 m of the factory bay
  AND a free pooled robot exists → parent pays 45 % of capacity, the child
  revives at the bay with 40 % charge and `mutate(genome)`. This is the
  only birth path. Lineage recorded.
- Genome (behaviour, mutated at fabrication): `{cruise_speed 0.4-1.2 m/s,
  charge_at 0.15-0.5 (fraction that triggers charge-seeking), module_pref
  {battery, solar, mast, armor: weights}, greed 0-1 (chase modules vs
  conserve), caution 0.3-2.0 m (lidar stop distance), explore_radius}`.
- Epoch driver: 6 robot slots (4 alive at start), 14 modules (10 loose),
  3 pads, bay at the centre, arena 24 m; epoch 240 s; score per lineage =
  fabrications·10 + modules_docked + charge_collected/50; keep top half,
  refill with mutations. Persist under `_run/robolife/epoch_NN/`.

## Supervisor ↔ robot bus (`customData`, JSON, ≤ 1 KB)

Supervisor → robot every 5 ticks:
`{"t": s, "batt": 0-1, "cap_wh": C, "state_hint": "ok|low|dead",
  "pads": [[x,y],..], "bay": [x,y],
  "modules": [{"id": j, "type": t, "x": x, "y": y, "yaw": r, "loose": true}, ...]   (within detection range only),
  "orders": ["release_rear"|"release_front"|"stop"|"none"],
  "genome": {...}}`
Robot → supervisor (`setCustomData`, every 5 ticks):
`{"state": "explore|seek_module|dock|seek_charge|charging|stopped",
  "v": measured m/s, "w": measured rad/s, "docked": {"front": j|null, "rear": j|null},
  "target": j|null, "lidar_min": m}`

## Verification gates

- **A (engine, the ONLY agent that may run it; one process at a time):**
  probe world = 1 Husky (`robolife_probe_dock` controller) + 2 modules loose
  + walls. The controller drives to module 0, aligns to its plug, creeps,
  `lock()`s, reports `getPresence()` before/after, then drives 4 m: the
  module's position must track the socket (report max separation), then
  `unlock()` and drive on: the module must stay behind (report). Also report:
  engine ms/step for 1 robot + 2 modules; the achieved-vs-commanded yaw for a
  90° turn and distance for a 2 m drive (the wheel law B will inherit);
  whether the docked mass changes braking distance. PASS = weld carries and
  releases. If `lock()` never engages, try `type "symmetric"` on both sides,
  then report exact tolerances tried. Run `--duration ≤ 60`.
- **B:** `pytest projects/robolife/tests/test_brain.py` green — docking
  geometry (approach pose from a plug pose), diff-drive inverse kinematics,
  lidar stop logic, state transitions on battery thresholds, genome parsing.
- **C:** `pytest projects/robolife/tests/test_energy.py` green — drain with
  mass and speed, pad charging, impact threshold with armor, death ordering,
  fabrication preconditions and cost, epoch scoring; `robolife.py --dry-run`
  prints the plan without an engine.
