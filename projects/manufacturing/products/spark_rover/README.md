# SparkRover Mini, made with three machine twins

A classroom robot about the size of a shoebox stood on end: four driven wheels, two waving
arms, a 5-inch face, a camera, a status ring and an accessory slot on its head, around a
Raspberry Pi 5. Its frame is laser-cut plywood and CNC-milled aluminium. Its shell, arms,
bumpers and electronics carriers are printed. Each made part is run on its machine's twin
and measured against its drawing. The robot is assembled from those parts on Newton and
driven there. [`release/`](release/) holds everything someone needs to build one.

**The story, start to finish:** [From concept images to a working robot](../../../../docs/guide/manufacturing-concept-to-robot.md)
tells how this robot went from AI concept sheets to this design, with every number's source.

⚠️ **Simulation only.** No part here has been made on a real machine, and no twin has
been validated against its real machine. Every measured number is the twin's, with
its conditions next to it.

⚠️ **The printing is not finished.** The printed parts are made on the SV08 twin plate
by plate. Until a part's plate has printed, it reads **pending** in the release (see the
table below). Nothing is predicted for it. `release.py` fills it in when it is run again.

| | |
|---|---|
| Envelope | 290 mm tall, 210 mm across the wheels, 185 mm bumper to bumper (`skeleton.py`) |
| Drive | skid steer: four Pololu #4844 25D 34:1 gearmotors with encoders, 65 mm wheels on a 170 mm track and a 110 mm wheelbase |
| Arms | two, each 2 DOF (shoulder and elbow pitch) on ROBOTIS XC330-M288-T servos, no gripper |
| Brain | Raspberry Pi 5 with an RP2040 IO board, two Pololu Motoron motor controllers |
| Face | 5-inch DSI display behind a CNC-cut acrylic cover, Camera Module 3, two I2S microphones, a speaker, capacitive touch pads |
| Power | 2S2P Li-ion pack (four Molicel P28A), USB-C charging, a 15 A fuse and a pushbutton power switch |
| Mass | held to 2.4 kg (`skeleton.MASS_TARGET_G`); the design's total is in the table below |

## The design

[`skeleton.py`](skeleton.py) is the contract. It fixes the envelope, the robot frame
(origin on the floor under the wheels' centre, +x forward, +z up) and a zone for each
subsystem. It also sets every interface between subsystems (the shell's mounts, the
shoulder bolts) and the decisions the concept sheets left open (`DECISIONS`). Each
subsystem is its own module, designed inside its zones and to those interfaces:

| Module | What it designs |
|---|---|
| [`chassis.py`](chassis.py) | the drive (motors, brackets, wheels), the two-layer plywood deck, the CNC spine foot, the glued plywood spine, the battery's volume |
| [`shell.py`](shell.py) | the printed PETG shell in four sections, TPU bumpers, diffusers, rear hatch, top socket, the acrylic face cover |
| [`arms.py`](arms.py) | the arms: CNC shoulder brackets, printed posts and links, the servos riding in the links |
| [`electrical.py`](electrical.py) | every board, the netlist, the power budget, the printed carriers the boards sit on, and its own release in [`release/electrical/`](release/electrical/) |
| [`harness.py`](harness.py) | routes every wire of `electrical.wire_list()` through the assembled robot in 3D |
| [`integrate.py`](integrate.py) | joins the four: zones, interference (FCL), interfaces, mass |

Every module provides `parts(r)`, `solids(r)`, `jobs(r)` (the machine files of its made
parts) and `checks(r)` (its own design rules). `python integrate.py` prints the
integration report and exits non-zero on any interference.

## Who makes what

| Process | Machine twin | Parts | Material |
|---|---|---|---|
| Laser cut | LongMill MK2 30×30 with the Sienci LaserBeam 7 W | two deck layers, four motor spacers, the spine's two side plates, front and rear plates and two rungs | 3 mm and 6 mm birch plywood |
| CNC mill | the same LongMill, router | the spine foot, both shoulder brackets (one blank) | 6061 aluminium, 4 mm and 8 mm |
| CNC route | the same | the face cover | 1.5 mm clear cast acrylic |
| FDM print | Sovol SV08 | the four shell sections, the hatch, the socket, the shoulder rings, the diffusers, the arms' posts, links and caps, the two electronics carriers and the cooler duct | PETG |
| FDM print | the same | the front and rear bumpers | TPU 95A |
| Bought | | motors, wheels, brackets, servos, boards, cells, screws, inserts, wire | [BOM](release/BOM.md) |

- **Laser**: [`make.py`](make.py) runs each subsystem's own programs on the twin
  headless, two engines at a time, and measures every part from the engine's height map.
  It checks that each part cut free, its outline, every hole and its thickness against the
  drawing. A sheet longer than 99 mm is cut as 0.05 mm windows and stitched. The engine's
  stock holds at most 1984 cells a side, and a coarser sheet keeps a skin under every
  kerf. The 6 mm plywood is cut at one pass more than Sienci's table gives
  (`chassis.LASER`): at Sienci's passes the 6 mm parts stayed joined by a thin web on
  their outlines (make.py's notes in REPORT.md).
- **CNC**: the spine foot and the brackets on the aluminium tool table. The brackets' tap
  holes are piloted 4.8 mm deep by the mill, then drilled through and tapped M3 by hand.
  The face cover's program names a 1/8 in O-flute that no LongMill tool table holds, so
  `make.py` supplies that tool.
- **Printing**: [`make_print.py`](make_print.py) checks every printed part for
  overhangs and bridges, slices it with CuraEngine, packs the parts into plates, prints
  each plate on the SV08 twin and measures every part against its STL.

## Measured

Generated by `release.py` from the result files. Do not edit this block by hand.

<!-- release.py: begin (generated; do not hand-edit) -->

| | Result (in simulation) | Conditions |
|---|---|---|
| Laser and CNC parts | 16 of 16 cut free, every hole cut | LongMill MK2 twin with the Sienci LaserBeam 7W and the router, height maps at 0.05-0.08 mm ([MAKE.md](release/make/MAKE.md)) |
| Printed parts | 27 of 27 printed: 2 on the full twin, 25 in ideal mode (no engine; their fixed-rule twin runs to come); 0 pending | Sovol SV08 twin, CuraEngine, Klipper's planner ([PRINT.md](release/make/print/PRINT.md)) |
| Integration | 325 solids, 0 interfering, 0 outside their zones | FCL on the design's solids (`integrate.py`) |
| Mass | 2397.8 g (ceiling 2400 g) | the design's masses, battery and harness included |
| Wiring | 127 of 128 wires routed, 17.94 m, 0 mm smallest gap to a part | `harness.py` on a 1 mm grid |
| Straight speed | 0.6941 m/s (no-load prediction 0.6964) | Newton, 2 ms, MuJoCo CPU, floor mu 1, 8.184 V mean on the bus (the battery model, from full charge) |
| Spin in place | 4.843 rad/s (6.816 if the tyres did not slip) | the same, the engine's default wheel armature: it depends on it (REPORT.md) |
| Stop from full speed | 47.669 mm | the same |
| Assembly | 53 units, 0 blocked, 0 interfering at a seat; stands on all four wheels: yes | `assemble.py` on Newton, the screws placed, not simulated |

The full tables are [release/REPORT.md](release/REPORT.md).

<!-- release.py: end -->

Conditions and what to read into them:

- **The laser's edges are a V.** Holes come out a little small at the bottom face, as on
  the desk robot, and the screws still pass. Some 6 mm parts report a "thickness
  minimum" of about 4.9 mm, and the 3 mm spacers less. That is the 1st percentile over every
  cell of the part, and on a narrow part the kerf's sloped flank, the cells within
  0.25 mm of an edge, makes up more than 1 % of them. From the saved height maps, every one
  of those parts is the full sheet thickness away from its edges. The window seams leave
  a second, smaller mark: a few interior cells along a seam read a fraction of a
  millimetre under the sheet. A real part has no seams. The numbers are in REPORT.md, "The thin readings".
- **The brackets' 3.2 mm minimum is the tap pilots' floor** (8 mm less 4.8 mm), by design.
- **The face cover's acrylic has no cutting-load model.** The twin removes the swept volume
  and models no load, melt or cracking.
- **The printed parts weigh less than designed.** The design gives a printed part's mass
  from its solid volume. The slicer lays walls and infill. The printed parts so far are
  compared in REPORT.md.
- **The spin and the stop depend on an engine constant.** The straight run matches the
  motors' datasheet. The spin's yaw rate and the stopping distance change with the
  engine's wheel-armature ratio (`OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO`), a rotor inertia
  the engine adds to every velocity-servoed wheel. REPORT.md has the runs at 10, 28 and the
  default. A real robot's spin is set by its motors and tyres, not by that constant.

## Assembling it, on Newton

[`assemble.py`](assemble.py) builds the robot in the design's order. The shoulder brackets
go onto the spine first, then the shell and the electronics, then each arm module last,
from outside. Each unit is swept along its insertion path against everything already
placed (FCL, the parts' own meshes), checked at its seat, then brought the last 5 mm on
Newton and let go. The electronics carriers are populated on the bench. The order and
each board's path are found by taking the populated carrier apart. The loaded carrier then
goes onto the spine as one unit. The finished robot is stood on the bench on its four
braked wheels. [`release/ASSEMBLY.md`](release/ASSEMBLY.md) is the build, step by step,
with the step pictures and the screws of every step. Screws are placed at their seats, not
simulated: a part held only by its screws moves when it is let go, and the guide says so.

## Driving it

[`robot.py`](robot.py) builds the robot from the design: every part's mass at its centre,
each part's inertia from its own solid, the wheels' hinges and the arms' four joints. It
writes [`spark_rover_drive_test.omniworld`](../../../samples/demos/worlds/manufacturing/spark_rover_drive_test.omniworld).
Its controller, [`spark_rover_drive`](../../../samples/demos/controllers/spark_rover_drive/spark_rover_drive.py),
drives 1 m straight, stops, spins in place for 4 s and waves both arms through every
limit on the design's motion profile. Each wheel motor is the Pololu #4844's datasheet
line at the pack's nominal voltage, braking above no-load speed. The results are in
[`release/drive/`](release/drive/), one file per floor friction and armature setting.

The head's camera and screen are working devices:

- **`camera`**, U9's Camera Module 3 (standard lens) at its lens, looking out of the face.
  It has the module's 66° horizontal field of view ([product brief](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf),
  `electrical.CAMERA_MOD`) at 640 × 360, the IMX708's 16:9 (the sensor is 4608 × 2592).
  That gives 40.1° vertical against the brief's 41°. The near plane is 1 cm, past the clear
  face cover 4-6 mm ahead of the lens. The board is mounted FPC edge down. The firmware sets
  the camera's rotation for that, and the simulated frame is the upright one it delivers.
  The controller runs it at 10 fps.
- **`display`**, U8's Waveshare 5" panel: 800 × 480 on its 108 × 64.8 mm active area, centred
  in the face window, 0.2 mm in front of the module and behind the cover. The controller
  draws a face that follows the test: eyes, a mouth, and "driving", "spinning" or "waving".
  The camera's latest frame sits in its corner.
- **The drive test is unchanged with them on.** Every number in `release/drive/drive_mu1.json`
  comes out the same. The run costs about 18 % more wall time, 0.95 → 1.12 ms a 2 ms step
  on the machine that measured it. A target board 1.7 m ahead gives the camera something to
  look at. It is visual only.
- **The engine needed a fix for the screen.** Before 2026-11-04 the wgpu renderer never
  uploaded a Display's texture, so the screen drew as a flat white quad. A binary built
  before that fix shows no face.

## Battery model

[`battery.py`](battery.py) models the 2S2P pack for the controller. It does not import
`electrical.py` and needs nothing but the standard library.

- **Source.** The [Molicel INR-18650-P28A data sheet](https://www.rosebatteries.com/hubfs/INR18650P28A-V2-80093.pdf),
  the copy `electrical.CELL` cites. Its discharge figure has curves at 0.56, 2.8, 10, 20 and
  30 A (23 C, to 2.5 V). The points are digitized from that figure: its curves are vector
  paths, read through the figure's axis ticks. Each point is good to ±10 mAh and ±20 mV,
  the width of the drawn line, and the curves are one typical cell's.
- **Modelled.** Each cell has an open-circuit voltage against state of charge, an ohmic
  resistance R0 (18.8 mΩ), and one RC branch (τ 8.5 s). The RC resistance is 6-14 mΩ down to
  12 % charge, then climbs to 190 mΩ by 4 %, the curves' knee. These come from
  the 0.56 A and 2.8 A curves, the two either side of this robot's 0.5-2 A a cell, and the
  model reproduces both curves within 3 mV at a steady current. Its 1-second resistance,
  19.8 mΩ, matches the data sheet's "DC (10A/1s) 20 mΩ". The current splits equally
  between the two parallel cells. The model counts coulombs, and its capacity is the data
  sheet's typical 2800 mAh (`capacity_mah=2700` gives the minimum cell). The protection
  board's 10 mΩ (ASSUMED, `electrical.BMS`) is in series. The cut-off latches at 6.4 V at
  P+/P-, optionally after a hold time. There is no Peukert term: to 2.5 V the 10-30 A curves
  give no less charge than the 2.8 A curve does. What does fall with current is the charge
  left above 3.2 V, and the sag gives that.
- **Not modelled.** Temperature: the data sheet's temperature curves are recorded in the
  module as data only, and a 3.2 V cut-off at -40 C ends a run within the first 6 % of the
  charge. Also not modelled: ageing, imbalance between the series cells, the protection
  board's own trips and balancing, and self-heating. Above about 5 A a cell the real cell
  sags less than the model says: 36-80 mV less at 10 A, up to 0.46 V less at 30 A. That
  makes the model conservative, and only the stall cases reach those currents.
- **The load.** `pack_load_a(motor_currents_a, electronics_a, duties=, servo_a=, v_bus=)`
  turns loads into pack current. Each motor contributes its current times its duty, through
  the Motoron's H-bridge, and the duty is 1.0 unless you pass one. The 5 V rail and the servo
  rail are constant-power loads through their regulators' 85 % (`electrical.REG5`,
  `REG6`). The function adds `electrical.operating_point`'s quiescent terms, and it gives
  that function's pack current for every case.

```python
from battery import Battery, pack_load_a
pack = Battery(soc=1.0)
i = pack_load_a(motor_currents, electronics_a=p5_w / 5.0, servo_a=servo_a, v_bus=pack.v_terminal)
v = pack.step(i, dt_s)          # also pack.soc, pack.v_ocv, pack.cut_off, pack.energy_wh_used
```

Runtimes, electrical's three profiles run to the cut-off (`python battery.py`). The
classroom mix is run as a one-minute cycle: 36 s idle, 15 s gesturing, 9 s driving. The
design column is `electrical.power_budget()` at the motor currents of 2026-11-04.

| Profile | Model (cut-off at P+/P-) | Model (cut-off read on the bus) | Design | Charge used |
|---|---|---|---|---|
| Idle | 4.60 h | 4.58 h | 4.79 h | 90.0 % |
| Classroom mix | 2.68 h | 2.62 h | 2.84 h | 88.5 % |
| Driving | 1.40 h | 1.37 h | 1.51 h | 88.1 % |

The model runs 4-7 % shorter than the design, for three reasons:

- **Usable energy.** The design counts 95 % (ASSUMED) of the four cells' 41.2 Wh as
  usable above 3.2 V a cell. At these currents the curve reaches 3.2 V with 10-12 % of its
  charge still in the cell, which leaves 37.0-37.7 Wh, or 90-92 %. This is most of the gap.
- **Driving's motor current.** The motors draw a fixed current, so on driving what runs out
  is charge. The design's energy, taken at 7.2 V, overstates that charge, because the
  cells sit near 7.4 V for most of the run.
- **The mix.** The mix cuts off in a driving burst, on that burst's sag.

If the firmware reads the bus instead, the Motoron's VIN after the fuse, the switch and
about 41 mΩ of wiring, the cut-off comes a little earlier: the middle column.

## Reproduce

These scripts import more than the engine ships: shapely, trimesh, python-fcl, OpenCASCADE
and others, pinned in [`../../requirements.txt`](../../requirements.txt). `print` also
needs UltiMaker Cura for slicing
([Before you start](../../../../docs/guide/manufacturing-make-your-own-product.md#before-you-start)).

```bash
python -m omnisim manufacturing setup      # from the repository root, once: shows what pip installs, asks first
python -m omnisim manufacturing doctor     # per stage: can it run here, and what is missing
cd projects/manufacturing/products/spark_rover
python skeleton.py                  # the contract's own checks
python integrate.py                 # zones, interference, interfaces, mass (a few minutes)
python harness.py                   # route every wire (release/electrical/wires_routed.json)
python make.py --process laser --process cnc     # the laser and CNC twins, headless (about an hour)
python make_print.py --run          # the SV08 twin, plate by plate (hours)
python robot.py --quit --result <path>.json      # the drive-test world, then:
python -m omnisim run-headless projects/samples/demos/worlds/manufacturing/spark_rover_drive_test.omniworld --duration 60
python assemble.py                  # the assembly on Newton (about half an hour)
python release.py                   # release/: drawings, models, cam, BOM, ASSEMBLY, REPORT, and the block above
```

`release.py` launches no engine. It reads the modules and the result files and
regenerates the whole folder. It caches the integration report and the printed parts'
solids in `.tmp/spark_rover_release/` by a hash of the design's sources. Pass
`--fresh-integration` to rerun the FCL check, and `--no-height-maps` to skip reading
make.py's saved height maps.

## The release folder

- `drawings/`: an SVG per made part. A laser or CNC part gets its top view with every hole
  and blind feature tabled. A printed part gets its top, front and side views as it lies on
  the bed, its mid-height section and its holes along Z.
- `models/`: an STL per made part, in mm. The flat parts are as cut and the printed parts
  as they lie on the bed.
- `cam/`: the laser G-code and CNC programs the twins ran, the face cover's tool table and,
  in `cam/print/`, the G-code of every plate that makes a printed part (gzip) with the SV08
  material profiles.
- `BOM.md`: made and bought parts with quantities, masses and sources, the electrical parts,
  the wire cut list and the stock. Its total is integrate.py's.
- `ASSEMBLY.md`, `REPORT.md`; `make/` (make.py's and make_print.py's own results),
  `electrical/`, `assembly/` and `drive/`, each written by its own script.

## Open items

- **Printing**: the parts not yet printed on the twin are pending in every document, with
  the plate each waits on.
- **The XC330 horn's dimensions are unverified.** The horn diameter, its screw circle and
  its axis height are scaled from ROBOTIS' e-manual figures (`arms.SERVO`), not measured.
- **The board hole positions on U5, U6, U12 and U11 are assumed** (`electrical.BOARD_HOLES`).
  Check each against the board bought before its carrier is printed.
- **The spin rate and the stopping distance depend on the engine's wheel-armature
  default** (above).
- **The acrylic has no cutting-load model** (above).
- **The 1984-cell stock cap is worked around with windows, not lifted.**
- **The mass margin to 2400 g is small**: the design's total and its margin are in the table
  above. They are on the design's masses, and the printed parts weigh less on the twin.
- **The 4.9 mm thickness readings** on the side plate and the mid rung come from the edge
  cells: the kerf's flank, plus a smaller dip along the window seams (above). make.py
  reports the 1st percentile over every cell. The interior's would be the better statistic.
- `shell.checks()` reports `skeleton_issues` not passing: the shell lists the things it
  would change in `skeleton.py`, and the skeleton records the ones accepted.
- **The camera and the screen are simulated; the rest of the face is not.** The two I2S
  microphones are not simulated: OmniSim retired its `Microphone` node, so there is no
  device to give them. The speaker, the touch pads and the status ring's LEDs are not simulated yet.
  They are visual meshes with mass.
- **Which way up the bare camera module's image comes out in this mount was not checked on
  a real module.** The simulated frame is the one the firmware delivers once its rotation
  is set. If a real module comes out inverted, the fix is that setting.
- **The screen is lit, not glowing.** The wgpu renderer has no emissive texture, so the face
  takes the scene's light like a printed sticker would.

REPORT.md's "Open items" is generated from the same sources and is the current list.
