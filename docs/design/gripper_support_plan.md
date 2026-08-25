# Plan: First-class gripper support for OmniSim arms

Status: **ALL PHASES DONE.**
- Phase 1: registry + effector abstraction + behavior-preserving bridge refactor.
- Phase 2: richer sim surface (set_gripper_width / grasp / release + tools + NL router).
- Phase 3: kinematic-attach grasp weld — nearest `GRASP_*` object within
  grasp_radius of the TCP welds to the tool and follows it each tick; release
  drops it. Verified deterministically (fake Supervisor) + world boots clean.
- Phase 4: real-robot driver layer (`agents/bridges/grippers/`) — RealGripperDriver
  + Robotiq 2F/3F (Modbus URCap), OnRobot RG, Schunk EGx, vacuum IO, mock;
  DryTransport default; `--gripper` on arm_bridge_stub; BridgeBase + tool aliases.
- Phase 5: arm pick-and-place demo world, `docs/reference/grippers.md`,
  DEMOS.md / WORLDS.md updates.

Outstanding (not blocking): real-hardware validation against a physical gripper;
OnRobot/Schunk register offsets marked `# TODO confirm`; flange-mount gripper
geometry assets (the bridge drives gripper motors if present, but bare URDF arms
have none — the kinematic weld is what performs the visible pick).
Owner: OmniLink
Created: 2026-05-21

## Goal

Let OmniSim arm robots drive **different real grippers** (Robotiq 2F-85/2F-140,
Robotiq 3F, vacuum/suction, OnRobot, Schunk, …) through one generic, plug-in
layer — selected at runtime, identical surface in sim and on real hardware.

## Design principle

Mirror what already works for arms. Today an arm is a dict in `ARM_CONFIGS`
(`_arm_configs.py`) and the bridge composes it generically. We do the same for
grippers: a `GRIPPER_CONFIGS` registry + a driver abstraction, so a gripper is
**decoupled from the arm** and selected with `--gripper <id>`. Sim and real
share one interface (the existing `BridgeBase` parity pattern), so OmniLink
agents / chat / tools never change.

### Current state (what we're replacing)

- Arms: fully config-driven, drop-in via `ARM_CONFIGS`.
- Grippers: **not** first-class. Three optional fields bolted onto the arm
  config (`gripper_motors`, `gripper_open_q`, `gripper_close_q`) and the bridge
  only does **binary open/close** by setting fixed motor positions
  (`omnilink_arm_bridge.py` ~L596-616).
- No gripper *type*, no width/force control, no feedback (holding?
  object_present?).
- An OmniSim `VacuumGripper` device exists but is not wired into the bridge.
- Real side (`agents/bridges/arm_bridge_stub.py`) mirrors the same binary
  open/close against a `MockArmDriver`; clean `BridgeBase` sim↔real parity.
- ROADMAP notes "gripper sim is shaky"; the early arm assembly demo used a
  Supervisor-teleported fake gripper because real Robotiq PROTOs hung the engine.

### Decisions (confirmed with user)

- Support **all popular grippers** via the generic layer.
- **Generic plug-in layer first**, then reference drivers.
- Sim grasping = **kinematic attach** (robust magnet/connector), not
  physics-contact (avoids the "shaky" path).

## Core abstraction

`GripperEffector` base class with a `kind` discriminator and one uniform
action surface, regardless of hardware:

```
open()              close()
set_width(meters)   grasp(force=…, width=…)   release()
state() -> {kind, width, holding, object_present, fault}
```

Concrete subclasses, one per gripper *family*:

- `ParallelFingerGripper` — 2-finger position-driven (Robotiq 2F-85/2F-140,
  Panda hand, OnRobot RG2/RG6, Schunk EGK). Width→motor-position map per config.
- `AngularGripper` — adaptive 3-finger (Robotiq 3F): coupled finger groups +
  open/pinch/wide modes.
- `VacuumEffector` — wraps the OmniSim `VacuumGripper` device
  (`turnOn`/`turnOff`/`getPresence`); on/off + presence feedback.
- `MagneticEffector` — Connector-node based (cleans up the assembly hack).

Each gripper config carries: `kind`, `motors`/`device`, named state map
(`open_q`/`close_q` or width-table), `stroke`/`max_width`, `flange_mount`
(offset to attach to any arm's TCP), and a `real_driver` reference.

## Phases

### Phase 1 — Gripper registry + sim abstraction
- New `_gripper_configs.py` next to `_arm_configs.py`: `GRIPPER_CONFIGS` keyed by
  id (`robotiq_2f85`, `robotiq_2f140`, `robotiq_3f`, `vacuum`, `panda_hand`,
  `onrobot_rg2`, `schunk_egk`, …).
- New `gripper_effectors.py`: base class + the 4 sim subclasses.
- Refactor `omnilink_arm_bridge.py`: resolve an `effector` from `--gripper`,
  replace inline gripper fields/`act_open_gripper`/`act_close_gripper`. Keep a
  back-compat shim so panda's arm-embedded gripper keeps running unchanged.

### Phase 2 — Richer bridge + tool surface
- New endpoints: `/set_gripper_width`, `/grasp`, `/release` (keep
  `/open_gripper`, `/close_gripper`).
- `/capabilities` + `/state` report gripper `kind`, width, `holding`,
  `object_present`.
- Update OmniLink tool defs + the NL intent router (~L677) so "grab it",
  "open halfway", "use suction" map onto the new actions.

### Phase 3 — Kinematic-attach grasp model (sim)
- Grasp = Supervisor-driven attach: on `grasp`, find nearest graspable Solid
  within reach of the TCP, weld it (Connector / field-write), `holding=true`;
  `release` detaches. Vacuum uses native device presence.
- One reusable attach/detach helper shared by every family.

### Phase 4 — Real-robot driver parity
- `RealGripperDriver` interface mirroring `GripperEffector` in `omnisim-bridges`
  + `arm_bridge_stub.py`.
- Reference real drivers (the literal "support different real grippers"
  deliverable):
  - Robotiq 2F-85/2F-140 over Modbus-RTU/TCP (URCap register protocol).
  - Robotiq 3F over Modbus.
  - Vacuum/suction over generic digital IO.
  - OnRobot / Schunk stubs with wire protocol sketched.
  - `MockGripperDriver` for offline runs.
- Selection via the same `--gripper` flag; agent/chat code byte-identical
  sim↔real.

### Phase 5 — Assets, demo world, docs
- Standardize a flange-mount that attaches a gripper Solid to any arm's TCP via
  Supervisor field-write (not a heavyweight PROTO teleported per tick — the
  cause of the `ur_arms.omniworld` hang).
- One demo world: arm + `--gripper`-selectable effector doing pick/place.
- Docs: `docs/reference/grippers.md` + "how to add a new gripper" guide
  mirroring the arm one; update DEMOS.md / WORLDS.md.

## Key files
- `projects/samples/demos/controllers/omnilink_arm_bridge/_arm_configs.py`
- new `_gripper_configs.py`, `gripper_effectors.py` (same dir)
- `omnilink_arm_bridge.py`
- `agents/bridges/arm_bridge_stub.py`, `packages/omnisim-bridges/...`
- new `agents/bridges/grippers/` (robotiq, vacuum, onrobot drivers)
- demo `.wbt` + `DEMOS.md` / `WORLDS.md` / `docs/reference/grippers.md`

## Risks / notes
- Back-compat: panda's gripper-in-arm-config keeps working via shim.
- Real drivers written against published specs (Robotiq URCap registers are
  documented) but untested against physical hardware here — Mock driver +
  register-level unit tests are the verification path until run on real grippers.
- Per-gripper width↔motor mapping needs calibration; seed defaults marked
  `# TODO calibrate`.

Phases 1+2 are the foundation and independently shippable.
