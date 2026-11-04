# A desk robot, made with three machine twins

A small two-wheeled robot whose parts come from every process OmniSim Manufacturing
has: the decks laser-cut from plywood, the motor clamps, wheel rims and electronics
tray printed in PLA, and the wheel hubs milled from aluminium, around bought motors,
caster, board and batteries. Each made part is run on its machine's twin and measured
against its drawing, the robot is assembled from those parts and driven on Newton,
and [`release/`](release/) holds everything someone needs to build one.

⚠️ **Simulation only.** No part here has been made on the real machines. Every
measured number is the twin's, with the conditions given next to it.

| | |
|---|---|
| Size | 90 × 80 mm decks, two 39.3 mm wheels on a 100.8 mm track |
| Mass | 219.0 g with four AA cells and the wiring, centre of mass 38 mm above the desk |
| Drive | two Pololu 100:1 micro metal gearmotors (#3065), 6 V |
| Brain | Seeed XIAO ESP32-S3 with a DRV8833 motor driver |
| No-load speed | 0.68 m/s (the motors' datasheet, through the wheel) |

## Who makes what

| Part | Qty | Process | Machine twin | Material |
|---|---|---|---|---|
| Bottom plate | 1 | laser cut | LongMill MK2 + Sienci LaserBeam 7 W | 3 mm plywood |
| Top plate, the "OmniSim" wordmark engraved | 1 | laser cut + engrave | same | 3 mm plywood |
| Motor clamp | 2 | FDM print | Sovol SV08 | PLA |
| Wheel rim | 2 | FDM print | same | PLA |
| Electronics tray | 1 | FDM print | same | PLA |
| Wheel hub | 2 | CNC mill | LongMill MK2 30×30 | 4 mm 6061 aluminium |
| Motors, caster, O-ring tyres, board, driver, cells, standoffs, screws | | bought | | [BOM](release/BOM.md) |

The design is in [`design.py`](design.py): every dimension and fit is derived there,
and `python design.py` prints the fit checks (wheel height against the caster, the
D-bore against the shaft, the tyre stretch). The plate underside sits 11.66 mm up on
the wheels and 11.69 mm up on the caster plus its spacer, so the robot stands level
to 0.03 mm.

## Real parts, real wiring

The bought parts are solids in the assembly, not blocks: [`bought_parts.py`](bought_parts.py)
puts each one in a local frame fixed from its manufacturer's own CAD (Pololu's gearmotor,
caster and DRV8833 carrier, Seeed's XIAO ESP32-S3), plus every screw and nut of every joint.

- **The vendors' CAD is fetched, never committed.** `python bought_parts.py fetch` downloads
  the four official STEP files, checks each against its SHA-256 and tessellates it into
  `meshes/vendor/` (git-ignored). Pololu states no licence for its models and Seeed's
  licence names its documents, not its models, so the repo carries our own models instead,
  drawn from the vendors' dimension drawings (`meshes/bought/`): the committed world and
  every test use those.
- **Every part is checked against every other, assembled** (`python bought_parts.py
  clearance`, FCL): a pair that meets is *touching* (a 0.05 mm nudge frees it), *threaded*
  (a screw in its nut or standoff), *pressed* (a designed squeeze) or *interference*. Both
  model sets give 41 solids and **no interference**.
- **What the vendors' CAD changed in the design**, each found by that check:
  - The caster's screw heads came up inside the tray: the caster moved back 5 mm (x -38).
  - Pololu's caster leaves 0.9 mm between each hole and its housing, so no nut fits on its
    side; the housing is notched 3.0 mm wide beside each hole for a round head, and an M2
    cap head (3.8 mm) does not fit it. It now takes **M1.6 x 8** screws from the ball side,
    heads in the notches, nuts on the plate.
  - With the clamp holes 18 mm apart an M2 nut overlapped the clamp's wall by up to 1 mm:
    they are 21 mm apart now.
  - The clamp channel was 10.1 mm over a 10.0 mm gearbox: the motor would have rattled.
    The roof now sits 0.1 mm into the gearbox, so the screws squeeze it (measured on
    Pololu's STEP: 0.1 mm).
  - The XIAO's USB-C overhangs its board by 1.53 mm, into the tray's wall: the wall is
    notched.
  - Every screw was too short for its stack (M2 x 6 through 3 mm of plate and a 2 mm tab
    leaves 1 mm for a 1.6 mm nut): the lengths are now computed joint by joint.
  - The wheel hubs' M2 cap heads (2.0 mm) face the gearbox, and the hub stood 1.0 mm off
    it: each wheel stopped 0.95 mm short of its seat. Found by the physical assembly below,
    then added to the static check (which now includes the wheels): the hub stands 2.4 mm
    off, 0.4 mm of clearance, and the track widens from 98 to 100.8 mm.
  - The right motor is mounted turned round, so its shaft's flat faces the other way: the
    right wheel goes on half a turn round, or its D-bore overlaps the shaft by 1.57 mm.
- **The electrical design** ([`electrical.py`](electrical.py), `release/electrical/`): a 5 V
  buck-boost regulator (the XIAO's 5V pin feeds a 5.5 V-max converter, so 6.4 V of fresh
  alkaline cannot go on it), a Schottky against USB back-feed, a MOSFET slide switch with
  reverse protection, a 3 A resettable fuse, a bulk capacitor and JST SM wire-to-wire
  connectors at the battery and each motor. Its checks pass (every net, pin, PWM pin, logic
  level and current against its rating); what it flags is in `POWER.md`: a motor stall runs
  the DRV8833 at 95 % of its continuous rating with nothing to trip it (the firmware needs a
  stall timeout), and a full-duty start on spent alkaline sags the bus below 2.7 V (the
  firmware must soft-start).
- **The pins are where the boards have them**: the DRV8833 carrier's pin order read off its
  silkscreen (Pololu's photo) and oriented by its STEP; the XIAO's pads measured on Seeed's
  STEP.
- **Every wire is routed in 3D** ([`wiring.py`](wiring.py)): 26 wires through the assembled
  chassis, insulation 0.3 mm off every part, laid along the surfaces, cut to the routed
  length plus 10 mm at each end. Smallest gap to any part 0.10 mm; to any other wire
  0.19 mm, measured beyond the 3.5 mm where wires fan out of a 2.54 mm pin row (there they
  touch and are bent by hand).

## Assembling it, on Newton

[`assemble.py`](assemble.py) builds the robot step by step in the simulator: each step's
parts come in 5 mm off their seat along their fastener axis and are brought on in 0.5 mm
increments (0.05 mm after a first contact), let go under gravity, then checked from the
engine: seat error, hole alignment against the holes' real clearances, penetration, and
that the sub-assembly stays still for 0.5 s. The wheels are spun in the air; the finished
robot stands on the bench and drives (`release/assembly/assembly_log.json`, a picture per
step).

- All seven steps seat within 0.1 mm, holes aligned within 0.03 mm (the clamps sit 0.08 mm
  high on their gearboxes: that is the designed 0.1 mm squeeze, which a rigid model cannot
  compress). The finished robot stands on both wheels and the caster, 0.17° off level.
- ⚠️ **The settle check is flaky.** In about one full run in two, one part creeps past the
  0.05 mm limit (the right motor clamp 0.057 mm, once the caster 0.129 mm), against 0.0004
  to 0.02 mm in every repeat of the same step. The engine is held and stepped, so it is not
  timing: it is a rigid part resting on a rigid part under the stiff contact these worlds
  use (50 kN/m; at the default 2.5 kN/m the parts sink half a millimetre). The clamp is
  screwed down at that point in the real build, so the check should weld a screwed part
  at its seat before measuring drift. Not done yet; the tolerance was not loosened.
- The assembly also hit a probable engine defect: a Solid with `density -1 mass m
  centerOfMass [c]` and no `inertiaMatrix`, whose origin is far from its material, tumbles
  off a flat seat. `assemble.py` puts every body's origin at its centre of mass, with an
  explicit inertia.

## Making it

```bash
python projects/manufacturing/products/desk_robot/make.py --out .tmp/desk_robot
```

runs all three twins headless, one after another, and writes `made.json`:

| Step | Machine time | Wall time (this laptop) | Result |
|---|---|---|---|
| Bottom plate | 661 s | 149 s | cut free, 90.1 × 80.1 mm (drawn 90 × 80), 13/13 holes and slugs out |
| Top plate | 715 s | 232 s | cut free, 90.1 × 80.1 mm, 7/7 holes; the "OmniSim" wordmark engraved 0.125 mm deep, letters 35.33 mm² (drawn 35.64) |
| Hubs | 1256 s | 422 s | both cut free; OD 15.994 mm (16.0), screw holes 2.39 mm (2.4), the shaft fits the D-bore |
| Print plate | 1606 s | about 1020 s | 58 layers, 11.65 mm tall (11.7), outer walls −0.05 / +0.07 mm (5th–95th percentile), 12.9 g PLA |

Conditions and what to read into them:

- **Laser holes come out 0.12–0.25 mm small at the bottom face.** The kerf is a V:
  wider at the top than the bottom. An M2 hole measures 2.27 mm and an M3 hole 3.28 mm,
  so the screws still pass. The sheet is resolved at 0.05 mm.
- **The hub program dives below the blank's top in rapid** (preflight reports
  `rapid_below_z0`). That is deliberate: each depth pass rapids to 0.1 mm above the
  floor the previous pass left, which is air. Feeding through that air took the
  first version 1.8 hours of machine time; this one takes 21 minutes. The stock is
  resolved at 0.025 mm.
- **The print's first two layers report millimetres of size error.** That is Cura's
  skirt line around the parts, not the parts. The twin also logged 20 nozzle crashes
  (the nozzle touching beads already laid); none stopped the print.
- `--only laser|cnc|print` reruns one step, and `--only remeasure-laser` /
  `remeasure-cnc` measure saved results again without a new run.

## Driving it

[`robot.py`](robot.py) builds the robot from the parts as made, with the motors'
datasheet model (stall torque falling to zero at no-load speed, and braking past it),
and writes
[`desk_robot_drive_test.omniworld`](../../../samples/demos/worlds/manufacturing/desk_robot_drive_test.omniworld).
Its controller drives straight for 1.2 s (0.8 m: the desk is 1.4 m long), stops, spins in place for 3 s and records
speed, effective wheel radius, yaw rate and heading drift
(`tests/test_desk_robot_drive.py` checks them against the datasheet).

Measured (2 ms step, MuJoCo CPU solver, 10 s of wall time for the whole test):

| | Predicted | Measured |
|---|---|---|
| Straight speed | 0.679 m/s (no load, 6 V) | 0.672 m/s |
| Effective wheel radius | 19.66 mm | 19.60 mm |
| Heading drift, straight | | 0.00° |
| Spin rate | 13.48 rad/s (no load) | 13.26 rad/s |
| Wheel rate in the spin | at most 34.56 rad/s | 33.95 rad/s (peak 34.58, a 2 ms finite difference) |
| Effective track | 100.8 mm | 100.3 mm |

**The motor model must brake above no-load speed** (fixed 2026-11-03). Until then
the controller set each motor's available torque to `max(0, stall × (1 − |ω| / no-load))`,
and the spin read 17.6 rad/s with the wheels at 45.8 rad/s, a third over the
motors' no-load speed. That was not the engine. `setAvailableTorque(0)` makes a
joint passive, so the first wheel past no-load speed (the left) went free while
the right one kept driving. The robot then swung round the free wheel and the
ground back-drove it to 58 rad/s. The solver's per-step record
(`OMNISIM_DEBUG_LAUNCH`) showed the free wheel carrying no actuator torque and
only the ground torque that rolls its 17.7 g·cm² of inertia. The robot's yaw
acceleration was the right wheel's traction: 0.43 N at 49 mm predicts
21.6 rad/s², and 22 was measured. With both wheels passive the yaw decayed.
A real DC motor on a battery does not go free past no-load speed: its back-EMF
exceeds the supply and it brakes, with the same `stall × (1 − ω / no-load)`, now
negative. The controller now sets the available torque to that line's magnitude,
and the velocity servo aims it at no-load speed from either side. The straight
run is unchanged to every digit. `tests/test_desk_robot_motor_model.py` pins
the model.

The speeds assume the motors' 6 V rating: the electrical design's budget puts the motors
at 4.6 V on mid-discharge NiMH, about 0.50 m/s (`release/electrical/POWER.md`).

## The release folder

```bash
python projects/manufacturing/products/desk_robot/render_steps.py
python projects/manufacturing/products/desk_robot/release.py --made .tmp/desk_robot --drive .tmp/desk_robot/drive.json
```

- `drawings/`: a dimensioned SVG drawing per made part (top view, section, hole table).
- `cam/`: the laser G-code for both plates, the hub mill program, the print plate's STL
  and G-code: exactly what the twins ran.
- `models/`: an STL per part.
- `BOM.md`, `ASSEMBLY.md` (seven steps, a picture each), `REPORT.md` (the measurements).
