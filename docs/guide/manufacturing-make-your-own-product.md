# Make your own product with OmniSim Manufacturing

This page is the how-to. It takes your own concept images to a product that OmniSim can
make on its machine twins, assemble, drive and release. A person and an agent can follow it
together. The template is the SparkRover Mini
([`projects/manufacturing/products/spark_rover/`](../../projects/manufacturing/products/spark_rover/README.md)).
The story of how that robot went from its concept sheets to a driving robot is
[From concept images to a working robot](manufacturing-concept-to-robot.md). Read that for
the why. This page covers the steps and the files.

> ⚠️ **Simulation only.** No twin in OmniSim Manufacturing has been validated against its
> real machine, and nothing on this page has been made on a real machine. Every number is
> the twin's or Newton's, quoted with its conditions. Printed parts come in two modes:
> **twin** (the printer twin in the engine, full physics) and **ideal**
> (`printing/ideal_print.py`, no engine). An ideal result is never a twin result.

## Before you start

The stages need three things beyond the repository. Install the first one, and check all
three, with two commands:

```bash
python -m omnisim manufacturing setup       # shows what pip will install, asks, then installs it
python -m omnisim manufacturing doctor      # per stage: can it run here, and what is missing, by name
```

- **Python packages.** The design scripts run in your own Python, the one that runs
  `python -m omnisim`, and import shapely, trimesh (with python-fcl and rtree), scipy,
  OpenCASCADE (`cadquery-ocp`), matplotlib, open3d and a few smaller packages. They are pinned in
  [`projects/manufacturing/requirements.txt`](../../projects/manufacturing/requirements.txt),
  with the stages that need each one. `setup` installs them into the Python that runs it, so
  run it inside a venv if you keep one per project. numba is optional. Without it the wire
  router and `ideal_print.py` run as plain Python, which is much slower.
- **CuraEngine**, for slicing: `print`, `print-ideal` until a plan exists, and
  `manufacturing printing slice`. It is not a Python package, and `setup` does not download
  it. Install UltiMaker Cura 5 (tested with 5.3.0) from
  [ultimaker.com](https://ultimaker.com/software/ultimaker-cura/). `printing/cura_slice.py`
  finds it under `C:/Program Files/UltiMaker Cura*`, `/usr/share/cura` or `/opt/cura*`, or in
  the folder `CURA_HOME` names.
- **The engine**, for `make`, `print`, `assemble` and `drive` (`python -m omnisim doctor`).
  `check`, `wire`, `print-ideal` and `release` run without it. The twins' controllers run on
  the engine's own Python and need nothing beyond it. The Windows installer is the only
  prebuilt package; on Linux, build from source first.

When a stage's package is missing, the router stops before the stage starts. It names each
missing package with its pip pin and prints the setup command, and the stage exits with code 3.

## The commands

One router runs each product's own scripts. Nothing is reimplemented: each stage runs the
product's script unchanged, from the repository root.

```bash
python -m omnisim manufacturing products                       # every product, its stages, its results
python -m omnisim manufacturing product spark_rover            # the stages and the scripts they run
python -m omnisim manufacturing product spark_rover check      # skeleton.py, then integrate.py (no engine)
python -m omnisim manufacturing product spark_rover make --plan            # the laser/CNC jobs (no engine)
python -m omnisim manufacturing product spark_rover print-ideal --only rev03   # one plate, ideal mode
python -m omnisim manufacturing product spark_rover all --dry-run          # every stage's command, in order
```

| Stage | Runs | Engine | What it does |
|---|---|---|---|
| `check` | `skeleton.py`, then `integrate.py` | no | the contract's own checks; zones, FCL interference, mounts, mass |
| `wire` | `harness.py` | no | routes every wire through the assembled robot |
| `make` | `make.py` | yes | the laser and CNC parts on their twins, measured against their drawings |
| `print` | `make_print.py --run` | yes | the printed parts on the SV08 twin, plate by plate |
| `print-ideal` | `make_print.py --ideal --only <plate>` | no | the same plates in ideal mode |
| `assemble` | `assemble.py` | yes | the build order: each unit swept, seated, then let go on Newton |
| `drive` | `robot.py`, then `run-headless` | yes | writes the drive-test world and runs it |
| `release` | `release.py` | no | regenerates `release/` from the result files |

Arguments after the stage go to the script. Every argparse script answers `--help`. A
script that takes no arguments, such as `harness.py`, has its docstring printed for
`--help` instead of being run. `print` with no arguments runs `--run`. Any argument replaces
that, so `print --plan` only plans. The router's own flags are `--dry-run` (print the
commands, run nothing), `--keep-going` (for `all`) and `--duration N` (for `drive`,
default 60 s). `all` runs `check`, `wire`, `make`, `print`, `assemble`, `drive` and `release`
in that order, stops at the first failure, and prints a summary. The engine stages take an
hour (`make`) to several hours (`print`). Run them one at a time while you work.

A product is any folder under `projects/manufacturing/products/` that has these scripts.
Give each script working defaults, so the router needs to pass nothing. SparkRover's
scripts do this. The desk robot predates the convention. The router runs it with the
commands from its README, and it has no `print` stage: its one print plate is printed by
`make --only print`.

## 1. Read the concept, then write the skeleton contract

Concept images give a look and a set of approximate numbers. Some of the numbers conflict.
SparkRover's sheets drew a 55-60 mm wheel on one sheet and 62-65 mm on another, a gripper,
and a depth camera. Decide each number once, in one file, before anything is designed:
[`skeleton.py`](../../projects/manufacturing/products/spark_rover/skeleton.py).

- **The envelope and the frame.** `HEIGHT`, `WIDTH`, `DEPTH`, the wheels (`WHEELS`, `TRACK`,
  `WHEELBASE`), and one robot frame for every module: origin on the floor under the wheels'
  centre, +x forward, +z up, in mm.
- **`DECISIONS`.** Record each question the sheets left open, and why you answered it. For
  SparkRover: no gripper, a plain RGB camera, and a 2S2P battery chosen for runtime over mass.
  An agent writes the sheet's number and its source next to the decision. A person makes
  the decision.
- **`MASS_TARGET_G`.** A ceiling that every module's masses are summed against.
- **`ZONES`.** A volume (`Box`) for each subsystem, which it may fill and must not leave.
  Zones of different owners may touch but must not overlap.
- **Interfaces.** Each fastened joint between two subsystems is a `Mount`, with its point,
  screw axis, size, the side holding the thread and the side the screw passes through.
  SparkRover's are `SHELL_MOUNTS`, `SHOULDER_BOLTS` and `ARM_POST_MOUNTS`.
- **The build order in words** (`ASSEMBLY_NOTE`) and the face features the shell must
  open (`FACE_WINDOW`, `CAMERA`, `TOP_SLOT`, ...).
- **`check_skeleton()`.** The contract checks itself: zones inside the envelope, owners not
  overlapping, wheels in their wells.

Only the integrator changes the contract, never one subsystem. A module that wants a change
lists it, as `shell.skeleton_issues()` does, and the skeleton records the changes it accepts.

## 2. One module per subsystem

Split the robot into subsystems that one designer (a person or an agent) can own. SparkRover
has four: [`chassis.py`](../../projects/manufacturing/products/spark_rover/chassis.py),
[`shell.py`](../../projects/manufacturing/products/spark_rover/shell.py),
[`arms.py`](../../projects/manufacturing/products/spark_rover/arms.py) and
[`electrical.py`](../../projects/manufacturing/products/spark_rover/electrical.py). Each one
imports the skeleton and designs inside its zones and to its interfaces. Each one provides
the same functions, which the integrator, the twins, the assembly and the release all read:

| Function | Returns | Read by |
|---|---|---|
| `parts(r)` | one row per part: `label`, `process` (laser, cnc, print, bought), `material`, `mass_g`, `centre_mm`, `source`, ... | `integrate.mass`, `robot.py`, `release.py` (BOM) |
| `solids(r)` | `{label: triangles}`, in the robot frame, mm | `integrate.zones` / `interference`, `harness.py`, `assemble.py`, `robot.py` |
| `mounts(r)` | the skeleton interfaces this module provides or uses | `integrate.mounts` |
| `jobs(r)` | `{part or job: machine file}`: laser G-code, a CNC program, or a print STL in its print orientation | `make.py`, `make_print.py`, `release.py` |
| `checks(r)` | `{name: (passed, why)}`: the module's own design rules (torque margins, servo supply, wheel positions) | `integrate.report`, `release.py` |

Optional: `INTERFACE_PARTS` and `interface_solids(r)` for parts that cross into another zone
by design, such as screws and the charger's USB-C receptacle. The zone check exempts them,
and the interference check still covers them. Put every source in the module: a datasheet
URL for a bought part, "ASSUMED" for a number no vendor gave (SparkRover's
`electrical.BOARD_HOLES` on four boards are assumed, and that stays an open item).

## 3. Integrate

```bash
python -m omnisim manufacturing product spark_rover check
```

[`integrate.py`](../../projects/manufacturing/products/spark_rover/integrate.py) joins the
modules. Every solid must stay inside its owner's zones (the shell inside the envelope). Every
pair is checked with FCL and labelled TOUCHING (a 0.05 mm nudge frees it), THREADED (a fastener
within a thread's depth) or INTERFERENCE. It reports any `Mount` that one side does not
provide or the other does not use, and checks the total mass against the target. It exits
non-zero on any interference or any solid outside its zone. On 2026-11-04 the router's `check` on SparkRover
took 2 min 46 s on the shared laptop. It reported 325 solids, 0 interfering, 0 outside their
zones, every mount provided and 2397.8 g against the 2400 g ceiling. One module check still
fails, `shell.skeleton_issues`, and the README lists it as an open item.

## 4. Run the twins

- **Laser and CNC:** [`make.py`](../../projects/manufacturing/products/spark_rover/make.py)
  collects every module's laser and CNC jobs, lays each out on its sheet, runs it headless on
  the LongMill twin and measures each part from the engine's height map: cut free, outline,
  holes, thickness. `--plan` lists the jobs and checks their layouts without an engine.
  `--only <job>` reruns one job, and `--remeasure` measures saved runs again. It respects the
  machine: `--parallel`, `--max-engines` and `--max-gpu-c`.
- **Printing:** [`make_print.py`](../../projects/manufacturing/products/spark_rover/make_print.py)
  checks every printed part for overhangs and bridges, slices it with CuraEngine, packs the
  parts into plates, prints each plate on the SV08 twin and measures each part against its STL.
  `--plan` stops before printing, though it still rewrites `release/make/print/` from the results
  on disk. `--run` prints on the twin. `--ideal --only <plate>` prints
  in ideal mode. Every part records the mode it was printed in (`printed.json`).
- The process guides cover the machines themselves:
  [laser](../../projects/manufacturing/laser/README.md),
  [CNC](../../projects/manufacturing/cnc/README.md),
  [printing](../../projects/manufacturing/printing/README.md).

## 5. Route the wires

[`harness.py`](../../projects/manufacturing/products/spark_rover/harness.py) routes every wire
of `electrical.wire_list()` through the assembled robot. It uses the desk robot's router
([`desk_robot/wiring.py`](../../projects/manufacturing/products/desk_robot/wiring.py)): the
solids voxelised and grown by each wire's radius plus a clearance, then A* with a discount for
running along surfaces, then string pulling, and each routed wire becomes solid for the next.
It writes `release/electrical/wires_routed.json`, which supplies the cut lengths for the BOM, and
`meshes/wires/`, which `robot.py` draws. Ribbon cables are routed along their centre line, and
their clearance is not claimed.

## 6. Assemble

[`assemble.py`](../../projects/manufacturing/products/spark_rover/assemble.py) builds the robot
in the design's order, one unit at a time. A unit can be a part or a sub-assembly made on the
bench, such as a populated electronics carrier. Each unit is first swept along its insertion
path against everything already placed (FCL, 0.5 mm steps), then checked at its seat, then
brought the last few millimetres on Newton and let go. The result is recorded as RESTS, SHIFTS or
FALLS. A part held only by its screws falls when it is let go, because screws are placed, not
simulated. The guide says so. `--no-physics` runs the exact (FCL) checks alone, without an
engine. `release.py` turns `release/assembly/assembly_log.json` into `ASSEMBLY.md`.

## 7. Simulate the robot

[`robot.py`](../../projects/manufacturing/products/spark_rover/robot.py) builds the robot from
the design. Every part's mass sits at its centre, with its inertia taken from its own solid.
The wheels get hinges, and the joints get their limits. The motors use their datasheet's DC
line, braking above no-load speed. It writes the drive-test world, and its controller writes a
result file.

```bash
python -m omnisim manufacturing product spark_rover drive --result projects/manufacturing/products/spark_rover/release/drive/drive_mu1.json
```

The router adds `--quit` when `robot.py` takes it, so the controller ends the run once it
has written its result. It then runs `python -m omnisim run-headless <world> --duration 60`.
`release.py` reads every file in `release/drive/`, so name each one after its conditions,
such as `drive_mu0.8.json` or `drive_mu1_wheel_armature_ratio10.json`.

## 8. Write the release

[`release.py`](../../projects/manufacturing/products/spark_rover/release.py) launches no engine.
It reads the modules and the result files and regenerates `release/`:

- `drawings/`, `models/` and `cam/`;
- `BOM.md`, `ASSEMBLY.md` and `REPORT.md`;
- the generated table in the product README.

Each result file is written by its own script:

- `make/made.json` and `make/print/printed.json`;
- `electrical/`, `assembly/assembly_log.json` and `drive/*.json`.

A part that has not been made reads **pending**, never predicted. Generate the open items from
the same sources, so they cannot drift from the code.

## Lessons from SparkRover, each with its source

- **Check the build order with exact sweeps, not only the finished pose.** A robot can pass the
  interference check assembled and still be impossible to build. An FCL sweep of the upper
  shell halves' path found the electronics wings standing in it: the hatch bosses hit them
  4.5-17 mm out (`electrical.py`, 2026-11-04). The first bay carrier's rear corners blocked the
  upper halves as they slid on (`printed.json`, `superseded`). The order itself was wrong in
  places: the LED strips, charger and touch pads had been scheduled after the shell part that
  carries each of them (the `electrical.STEPS` comment). Each board's insertion path is now
  found by taking the populated carrier apart (`ASSEMBLY.md`).
- **Carriers need holes.** The first electronics carriers were drawn as solid boxes, with no
  screw or insert holes. The 2026-11-03 front carrier was printed on the twin before this was
  caught. All three were then redrawn, and the old drawings are recorded as superseded
  (`release/REPORT.md`, "Superseded drawings"). Draw every printed part that holds something
  with its screw and insert holes before it goes to the printer. Check the board hole positions
  against the real board (`electrical.BOARD_HOLES` are assumed for U5, U6, U11 and U12).
- **A contact cap can hide overlaps.** One FCL collision manager over all the solids keeps at
  most 100,000 contacts, trimesh's request size. It silently drops the pairs past that limit. At
  259 solids it found 110,997 contacts and lost two real interferences (2026-11-03).
  `integrate.interference()` now checks pair by pair, for every pair whose boxes overlap. A
  clean result from a capped query proves nothing.
- **Compile the wire router.** The router's A* search and line test are compiled with numba in
  `desk_robot/wiring.py`. The pure-Python search stays as the reference, and the desk robot's 26
  routes came out identical. With the compiled router, SparkRover's 128 wires went from more
  than 2 h unfinished to about 90 s (commit `f7af89eda`). Without numba the router runs as plain
  Python, much slower.
- **Iterate in ideal print mode, and keep a twin print as the check.** `ideal_print.py` printed
  the 145 g shell section in 21 s (plus 116 s of inspection). The twin's runs of it took
  6,551-6,879 s of wall time.
  An ideal part reads about 0.02-0.03 mm smaller a side than the twin's, because the axis
  dynamics are left out ([printing README](../../projects/manufacturing/printing/README.md#ideal-print-no-engine)).
  Use ideal mode while a part's design is still changing. Then print a plate on the twin to check
  it, and report each part in the mode it was actually printed in. On SparkRover, 22 parts are
  printed on the twin and 5 in ideal mode only: the two TPU bumpers and the three revised carriers
  (`python -m omnisim manufacturing products`).
- **Calibrate laser passes on the twin before cutting the sheet.** Sienci's table gives 5
  passes for 6 mm plywood. On the twin, every 6 mm part stayed joined to its sheet by a 0.16-0.20
  mm web. Six passes cut the spine rear free, with 9 of 9 holes through, for 3.7 more minutes a
  plate (`chassis.LASER`, `release/make/MAKE.md`, 2026-11-03). Cut one part, read its height
  map, then set the passes. The twin's verdict is the twin's: no real sheet was cut.

## Keep the open items visible

A release lists what is not done next to what is. SparkRover's list is in its
[README](../../projects/manufacturing/products/spark_rover/README.md#open-items) and in
`release/REPORT.md`. It includes these items:

- the microphones are not simulated, because OmniSim retired its `Microphone` node;
- the bead model, corrected in v10: twin prints measured before the fix are labelled so;
- the front carrier's 64 g of support for a 75 g part;
- the assumed board holes.

Carry your own list the same way. Generate it from the sources, and never hide an item behind
a passing table.

## Start your own

1. Copy the layout of `spark_rover/`: `skeleton.py`, one module per subsystem, `integrate.py`,
   `harness.py`, `make.py`, `make_print.py`, `robot.py`, `assemble.py`, `release.py` and a
   README. Keep the file names: the router discovers stages by them. It also reads each
   stage's packages from your scripts' imports, so `doctor` and the stage check cover your
   product too. A package that is not in `requirements.txt` is yours to add, there and in
   `deps.py`'s `PACKAGES`, with the stage that needs it (`tests/test_manufacturing_setup.py`
   fails until both list it).
2. Keep your concept images with a note of where they came from. SparkRover's are copied
   to `docs/images/manufacturing/concept/`. Write `skeleton.py` from them, with every
   decision in `DECISIONS`.
3. Design one module at a time and run `product <name> check` after each change.
4. Run `make --plan` and `print --plan`, then the twins, then `wire`, `assemble` and `drive`.
5. Run `release`, and read every generated table before you share it.

`python -m omnisim manufacturing products` shows where each product stands, read from its own
result files.
