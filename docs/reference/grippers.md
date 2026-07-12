# Grippers

OmniSim arms drive different end effectors through one pluggable layer.
A gripper is **decoupled from the arm** and selected at runtime — the same
`--gripper <id>` picks the same physical gripper in simulation and on real
hardware, so OmniLink agents, chat, tools and the HTTP surface never change.

This mirrors how arms work: an arm is an entry in `ARM_CONFIGS`, a gripper
is an entry in `GRIPPER_CONFIGS`. Adding either is drop-in, no bridge code.

## Selecting a gripper

```
controller "omnilink_arm_bridge"
controllerArgs [ "--robot" "ur5e" "--gripper" "robotiq_2f85" "--port" "8765" ]
```

Resolution order in the bridge:

1. `--gripper <id>` (the registry, below)
2. the arm config's `default_gripper`
3. the arm's legacy inline `gripper_*` fields (back-compat shim)
4. none → the arm has no gripper

## Built-in grippers

| id | model | family | width control |
|---|---|---|---|
| `robotiq_2f85`  | Robotiq 2F-85   | parallel | yes (0–85 mm) |
| `robotiq_2f85_phys` | Robotiq 2F-85 (physics)    | parallel | yes (0–85 mm) |
| `robotiq_2f85_grip` | Robotiq 2F-85 (force-grip) | parallel | yes (0–85 mm) |
| `robotiq_2f140` | Robotiq 2F-140  | parallel | yes (0–140 mm) |
| `robotiq_3f`    | Robotiq 3F      | angular (3-finger, modes) | yes |
| `onrobot_rg2`   | OnRobot RG2     | parallel | yes |
| `onrobot_rg6`   | OnRobot RG6     | parallel | yes |
| `schunk_egk40`  | Schunk EGK 40   | parallel | yes |
| `vacuum`        | Suction         | vacuum   | no (on/off) |
| `magnetic`      | Magnetic coupler| magnetic | no (on/off) |

Sim configs: [`_gripper_configs.py`](../../projects/samples/demos/controllers/omnilink_arm_bridge/_gripper_configs.py).
Sim effectors: [`gripper_effectors.py`](../../projects/samples/demos/controllers/omnilink_arm_bridge/gripper_effectors.py).

## HTTP / tool surface

Every arm bridge with a gripper exposes (POST unless noted):

| endpoint | body | does |
|---|---|---|
| `/open_gripper`     | — | open to full width |
| `/close_gripper`    | — | close fully |
| `/set_gripper_width`| `{"width": <m>}` | move to an opening (metres) |
| `/grasp`            | `{"force"?, "width"?}` | close + hold; in sim, welds the nearest graspable object |
| `/release`          | — | open + drop |

`/capabilities` and `/state` report the gripper `kind`, `width`, `holding`
and `object_present`. The natural-language router understands "grab it",
"let go", "open to 4 cm", "halfway".

## Grasping in simulation (kinematic attach)

Sim grasping uses a **kinematic attach**, not physics-contact finger
friction (which is unstable). On `/grasp` the bridge finds the nearest
graspable object within `grasp_radius` of the TCP, welds it to the tool,
and teleports it to follow the TCP each tick; `/release` drops it.

Objects opt in by giving their scene-tree node a **DEF that starts with
`GRASP_`**:

```
DEF GRASP_CUBE_A Solid {
  translation 0.45 0.0 0.03
  children [ DEF S Shape { geometry Box { size 0.05 0.05 0.05 } } ]
  boundingObject USE S
  physics Physics { density -1 mass 0.1 }
}
```

The TCP world position is computed from the arm's base pose × the FK TCP
point, so the arm needs an IK chain (see `_arm_configs.py`). `grasp_radius`
defaults to 8 cm and can be set per gripper config.

## Physics grasp (real contact, not kinematic attach)

The kinematic attach above is the default (robust). For a *real* contact
grasp — fingers that close on the part and hold it by friction — use a
gripper whose fingers are real actuated joints in the robot:

- Merge the Robotiq 2F-85 into the arm's URDF on its **last link** as two
  prismatic finger joints (collision + 50 N effort → real linear motors).
  Attaching to the last link (not a `flange` fixed-joint child, which the
  importer orphans) is what keeps the gripper rigidly on the arm.
- The `robotiq_2f85_phys` gripper config sets `physics_grasp: true`, so
  `/grasp` closes the finger motors and **skips the kinematic weld** —
  contact friction holds the part.
- Grasping needs a correctly *oriented* approach. `POST /set_tcp_pose`
  `{xyz, tcp_offset_z}` runs **6-DOF IK** to put the tool at `xyz` pointing
  straight down, so the open fingers straddle the part top-down.
  (`/set_tcp_target` is position-only and can't orient the approach.)
- World friction matters: a `ContactProperties` with high `coulombFriction`
  (~5) and small `softCFM` keeps the grip stable.

Verified on a 6-DOF arm with this setup: cube lifted to z≈0.27 m and placed
within ~1 cm of the target, by friction alone.

## Real hardware

The real-robot bridge mirrors this exactly. See
[`agents/bridges/grippers/`](../../agents/bridges/grippers/): a
`RealGripperDriver` per family (Robotiq 2F/3F over Modbus URCap, OnRobot RG,
Schunk EGx, vacuum-over-IO, mock) selected with the same `--gripper` id.
Drivers default to a `DryTransport` (logs the frames they would send) so the
whole path runs hardware-free; inject a real Modbus/IO transport to deploy.

```bash
python agents/bridges/arm_bridge_stub.py --gripper robotiq_2f85
```

## Adding a new gripper

1. Add an entry to `GRIPPER_CONFIGS` in `_gripper_configs.py` (`kind`,
   `motors`/`device`, `open_q`/`close_q`, `max_width`, optional `modes`).
2. If it's a new *family*, add an effector subclass in `gripper_effectors.py`
   and register it in `_FAMILIES`.
3. For real hardware, add a `RealGripperDriver` in `agents/bridges/grippers/`
   and an entry in `GRIPPER_SPECS`.

The bridge picks all of this up with no further changes.
