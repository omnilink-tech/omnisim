# OmniSim Manufacturing

Machine twins that make parts, and the products made with them. A machine here is
built from its own controller config (and, where the vendor publishes it, its CAD). It
runs the same job file the real machine would. The material it works is simulated
where the tool really went, and the part it makes is measured against the drawing.

**Status, 2026-11-02: simulation only.** No twin here has been compared against its
real machine. Every number below is the twin's, measured in the twin, with its
conditions in the process's README. None is a claim about the real machine.

```bash
python -m omnisim manufacturing                  # what is here
python -m omnisim manufacturing machines         # every machine, its process, its files
python -m omnisim manufacturing worlds           # every world that runs one
python -m omnisim manufacturing printing slice part.stl --out part.gcode
python -m omnisim manufacturing printing preflight part.gcode
python -m omnisim manufacturing cnc preflight job.nc --machine projects/manufacturing/machines/longmill_mk2_30x30/longmill_mk2_30x30.grbl
python -m omnisim manufacturing laser kerf          # passes to cut through, against the vendor's table
python -m omnisim manufacturing products            # every product, its stages and its results
python -m omnisim manufacturing product spark_rover check   # one stage of one product
```

Each tool is the process's own script, run unchanged, and each answers `--help`.

## Setup

The twins run in the engine, and their controllers need only what the engine's own Python
already has. The design and process scripts run in your Python, the one that runs
`python -m omnisim`, and they import more. That means shapely, trimesh (with python-fcl,
rtree and networkx), scipy, OpenCASCADE (`cadquery-ocp`), matplotlib, open3d and a few smaller
packages, pinned in [`requirements.txt`](requirements.txt) with the stages that need each.

```bash
python -m omnisim manufacturing setup      # shows what pip will install, asks, installs; ends with the table below
python -m omnisim manufacturing doctor     # per stage and tool: can it run here, and what is missing, by name
```

- `setup` installs into the Python that runs it, so run it inside a venv if you keep one.
  `--yes` skips the question, and `--dry-run` only shows the list. On Windows with CPython
  3.12 every pin installs from a wheel, python-fcl and cadquery-ocp included. Use 3.12:
  on 3.13, `open3d==0.19.0` has no wheel (0.20.0 has one, untested here). Measured
  2026-11-04 in an empty venv: 82 packages (16 pinned, the rest their dependencies; cadquery-ocp
  brings vtk, open3d brings dash and plotly), 1.4 GB on disk. It took 3 min 43 s from
  pip's cache, and 5 min 12 s in an earlier run that downloaded most of the wheels.
- **CuraEngine** is not a Python package, and `setup` does not download it. `print`,
  `print-ideal` (until `make_print.py` has written its plan) and `printing slice` need it.
  Install UltiMaker Cura 5 (tested with 5.3.0) from
  [ultimaker.com](https://ultimaker.com/software/ultimaker-cura/).
  [`printing/cura_slice.py`](printing/cura_slice.py) finds it under
  `C:/Program Files/UltiMaker Cura*`, `/usr/share/cura` or `/opt/cura*`, or in the folder
  `CURA_HOME` names.
- **The engine** is needed for `make`, `print`, `assemble`, `drive` and `laser run`
  (`python -m omnisim doctor`). `check`, `wire`, `print-ideal` and `release` need none.
- numba is optional. Without it the wire router and `printing/ideal_print.py` run as plain
  Python, which is much slower.
- A stage or tool whose package is missing stops before it starts. It names each package
  with its pip pin, prints the setup command, and exits with code 3, without a traceback.
  [`deps.py`](deps.py) holds the per-stage table. A product of your own has its needs read
  from its scripts' imports.

Guides: [Make your own product](../../docs/guide/manufacturing-make-your-own-product.md)
(the how-to, with SparkRover as the template) and
[From concept images to a working robot](../../docs/guide/manufacturing-concept-to-robot.md)
(the SparkRover story).

## Processes

| Process | Folder | The real machine's side | What the engine simulates | Guide |
|---|---|---|---|---|
| **Subtractive:** CNC milling | [`cnc/`](cnc/) | A virtual GRBL 1.1 controller on TCP: any GRBL sender connects. Preflight, tool changes, cutting load | `MillingStock`: a height map carved where the cutter really went (flat, ball, V-bit) | [cnc/README.md](cnc/README.md) |
| **Thermal:** laser cutting and engraving | [`laser/`](laser/) | The CNC's virtual GRBL in laser mode: M3/M4, G0 dark, PWM as GRBL computes it, integrated every 0.5 ms | `MillingStock` with `toolShape "laser"`: a Gaussian spot laid along the beam's real path, widening out of focus, ablating and charring | [laser/README.md](laser/README.md) |
| **Additive:** FDM printing | [`printing/`](printing/) | The job planned as Klipper plans it (junctions, input shaper, pressure advance), sliced by a real slicer, with a hotend flow model | `PrintedPart`: beads laid where the nozzle really went, on the bed or on earlier plastic | [printing/README.md](printing/README.md) |

## Machines

`machines/<name>/` holds everything about one machine: its controller config, its tool
tables or profile, and its CAD pack (`cad/`) when there is one.

| Machine | Process | Built from | CAD pack licence |
|---|---|---|---|
| `longmill_mk2_30x30` | cnc, laser | Sienci Labs' GRBL defaults, tool tables, and the Sienci LaserBeam 7W module (`sienci_laserbeam_7w.json`, discontinued, from Sienci's docs) | CC BY-SA (Sienci Labs' Onshape CAD); the laser head is schematic |
| `example_3018` | cnc | a GRBL `$$` dump with limit switches | – (schematic) |
| `sostaric_laser_100` | laser | Daniel Sostaric's 100 mm XY laser: the numbers of his FluidNC config (numbers only; his repository states no licence) and a LaserTree LT-40W-AA 5 W module from LaserTree's product page (`lasertree_lt40w_aa_5w.json`). A moving bed: the sheet rides the stages under a fixed head | – (schematic, sized from his published screenshots; see its `PROVENANCE.md`) |
| `sovol_sv08` | printing | Sovol's Klipper config, macros and OrcaSlicer profile (numbers only) | GPL-3.0 (Sovol's STEP) |

**Licences.** A CAD pack carries the vendor's licence, kept in its own `cad/` folder
with a `PROVENANCE.md`. Everything else in this sector, including the manifests and
tools that cut the CAD, is OmniSim's Apache-2.0. A vendor's config file is not copied,
only the numbers the twin uses, with the commit they came from.

## Products

`products/<name>/` is something made with the machines, from design to finished
article. Two are robots, taken through every stage below; each has a `release/` folder with
what someone would need to build it (drawings, models, machine files, BOM, assembly guide,
report).

- [`spark_rover`](products/spark_rover/README.md): the SparkRover Mini, a four-wheeled
  classroom robot with two arms and a Raspberry Pi 5, designed from AI concept images as four
  subsystems against one skeleton contract, made on all three twins, wired in 3D, assembled
  and driven on Newton. It is the template for a new product.
- [`desk_robot`](products/desk_robot/README.md): a two-wheeled desk robot made on all three
  twins (laser-cut plywood decks, printed PLA clamps, rims and tray, milled aluminium hubs),
  assembled and driven on Newton. It predates the stage convention below.
- [`clothes_rack`](products/clothes_rack/README.md): aluminium connectors milled on the
  LongMill twin and measured against the drawing, then the rack assembled step by step
  and load-tested on Newton. It has its own scripts and no stages.

```bash
python -m omnisim manufacturing products                    # each product's stages and results, from its files
python -m omnisim manufacturing product spark_rover         # its stages and the scripts they run
python -m omnisim manufacturing product spark_rover <stage> [args]
python -m omnisim manufacturing product spark_rover all --dry-run
```

[`product_cli.py`](product_cli.py) routes each stage to the product's own script, run
unchanged from the repository root, with the arguments given:

| Stage | Script | Engine |
|---|---|---|
| `check` | `skeleton.py`, then `integrate.py` (zones, FCL interference, mounts, mass) | no |
| `wire` | `harness.py` (every wire routed through the assembled robot) | no |
| `make` | `make.py` (laser and CNC parts on the twins; `--plan` without the engine) | yes |
| `print` | `make_print.py --run` (the printed parts on the SV08 twin; `print --plan` plans only) | yes |
| `print-ideal` | `make_print.py --ideal --only <plate>` (ideal mode, below) | no |
| `assemble` | `assemble.py` (FCL sweeps along each insertion path, then Newton) | yes |
| `drive` | `robot.py` (with `--quit` where it takes it), then `run-headless <world> --duration 60` | yes |
| `release` | `release.py` (regenerates `release/` from the result files) | no |

`all` runs `check`, `wire`, `make`, `print`, `assemble`, `drive` and `release` in order and
prints a summary; `--dry-run` prints any stage's commands without running them. A product
is found by its script names. The desk robot is run with the commands from its README
(`design.py` and `bought_parts.py clearance` for `check`, `wiring.py` for `wire`, `--out` /
`--made .tmp/desk_robot`) and has no `print` stage: its print plate is `make --only print`.

**Two print modes.** `print` runs the SV08 twin in the engine, with full physics.
[`printing/ideal_print.py`](printing/ideal_print.py) prints the same G-code with no engine:
the same Klipper plan, hotend and PrintedPart laying rule, with the nozzle exactly where the
steppers would put it. A 145 g shell section takes 21 s instead of the twin's 6,551-6,879 s.
An ideal part reads about 0.02-0.03 mm smaller a side than the twin's, because the axis dynamics
are left out ([printing/README.md](printing/README.md#ideal-print-no-engine)). Every printed
part records the mode it was made in, and an ideal result is never quoted as a twin result.

## The shared core

`core/` is what every machine twin uses. It started as copies inside each process.

| Module | What |
|---|---|
| [`drives.py`](core/drives.py) | Axis drives: OmniSim position servos fed forward to move like steppers. **The rule worth knowing:** feed forward the axis's *effective* inertia, `max(moved mass, dt × kd)`, not its mass. Newton's servo-armature bound raises any lighter joint to that. It explains both machines: the LongMill's hand-tuned 1e-4 s² is exactly its 20 kg effective, and the SV08's 0.65 kg toolhead moves like 10 kg. |
| [`cad_pack.py`](core/cad_pack.py) | A vendor's CAD to a mesh pack: the assembly cut into the bodies each axis moves, in one frame. Optional coarser tessellation per group. |
| [`worldkit.py`](core/worldkit.py) | The pieces every world generator writes: numbers to 0.1 µm, boxes, cylinders, a camera. |

## Adding a machine

1. **Config.** A machine folder with the controller's own config:
   - a GRBL `$$` dump or a FluidNC YAML for a CNC (`cnc/machine.py`);
   - a profile from a Klipper `printer.cfg` for a printer (`printing/printer.py extract`).
2. **World.** Generate it with the process's `world.py`. It starts schematic, sized from
   the travel.
3. **CAD (optional).** Write a `cad/manifest.json` and build it with
   `python -m omnisim manufacturing cad-pack`. Record the source, the licence and what was
   changed in `cad/PROVENANCE.md`.
4. **Measure.** Run a job and inspect the part. A number goes into the README with its
   conditions, or it does not go in at all.

## Worlds

`projects/samples/demos/worlds/manufacturing/` holds them all; `python -m omnisim
manufacturing worlds` lists them with their controllers, and WORLDS.md describes each.
