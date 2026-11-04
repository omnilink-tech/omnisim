# Laser twin: cutting and engraving with a GRBL laser

A diode laser on a CNC gantry, run through the same virtual GRBL 1.1 controller as the
CNC twin (any GRBL sender connects on TCP `:2323`). GRBL's laser mode decides what the
laser pin does, millisecond by millisecond. The engine burns the sheet where the beam
really went, at that power, and the part it cuts is measured against the drawing.

**Status, 2026-11-02: simulation only.** Nothing here has been compared with a real
laser. The burn model is calibrated on one row of the vendor's material table and checked
against two more (below). It is not checked against a measured cut.

```bash
python -m omnisim manufacturing laser kerf                       # the calibration against Sienci's table
python -m omnisim manufacturing laser job --out tag.gcode        # the test part's G-code
python -m omnisim manufacturing laser run \
    projects/samples/demos/worlds/manufacturing/laser_longmill_laserbeam.omniworld \
    projects/manufacturing/laser/jobs/tag_3mm_plywood.gcode --out .tmp/laser-run
python -m omnisim manufacturing laser inspect .tmp/laser-run/sheet.zmap --trace .tmp/laser-run/trace.jsonl
```

## The machine

**The Sienci LaserBeam 7W on the LongMill MK2 30x30.** This is the best-documented diode
laser found: its controller settings, diode, lenses and a material table with exact
settings are all published. It also fits the LongMill the CNC twin is already built from
(same CAD, same axes and drives).

- **Status:** discontinued by Sienci Labs. The twin is built from its documentation, and
  nobody can buy a new one.
- **Configuration:** `machines/longmill_mk2_30x30/sienci_laserbeam_7w.json`, every
  figure with its source page:
  - 7 W optical at 445 nm, through the G2 lens at 95 %, which focuses 28–35 mm below the
    head.
  - A 0.065 × 0.13 mm spot.
  - `$30=255 $31=0 $32=1`, the laser settings gSender switches the LongBoard to.
- **Assumed** (Sienci does not publish these):
  - The spot's long side lies along machine Y.
  - The lens focuses a cone of 0.12 rad half-angle, which corresponds to a beam about
    7 mm across at a 30 mm focal length.
- **Schematic:** the head itself (nozzle, heatsink, fan, bracket), mounted in front of the
  router. Sienci publishes no licensed CAD for it, so its shape and mounting point are not
  the real part's. Its optics are.

## What is simulated

**GRBL's laser pin** (`laser.py LaserOutput`, from GRBL 1.1's laser-mode docs and source):

- **G0 never lights.**
- **M3** holds its power whether the machine is moving or not. It stays lit after a G1
  when the buffer runs dry, and a bare `M3 S255` lights it at rest.
- **M4** scales S by current speed over programmed feed, and is off at rest.
- **S to PWM** follows `spindle_compute_pwm_value`: 8-bit, clamped at `$30`, at least the
  minimum PWM above 0, and scaled by the spindle override.
- **A completed feed hold** switches the laser off (`DISABLE_LASER_DURING_HOLD`).

The power is integrated over every planner sub-step (0.5 ms). Each physics step gets the
energy of the interval its motion belongs to.

**The burn** happens in the engine: `MillingStock` with `toolShape "laser"`.

- **Spot:** every physics step's energy is laid along the beam's real path as an
  elliptical Gaussian spot. Its 1/e² radius widens away from focus as √(w₀² + (tan θ · dz)²).
- **Removal:** absorbed fluence removes `F / e_v` of material and chars the surface toward
  `charColor`. As a kerf deepens it falls out of focus, so each pass cuts wider and
  shallower. Lowering Z between passes, as Sienci's table does, brings it back.
- **Through-cuts:** energy that reaches a cell already cut through passes on and is lost.
- **Energy only:** no heat conduction, flame, smoke or resin.

**The 1D kerf model** (`laser.py kerf`) is the same physics across one straight cut. It
sweeps each pass in slices, as the engine does by burning step by step. It is fast enough
to answer "how many passes?" before a job runs.

## Calibration: one row fitted, two predicted

`e_v`, the energy that removes a cubic millimetre, is **effective**. It lumps absorption,
conduction, the plume and wall losses into one fitted number, which is why it is several
times wood's bulk decomposition energy.

It is fitted on one row of [Sienci's material guide](https://resources.sienci.com/view/lb-material-guide/),
3 mm plywood at 100 %, 2.2 mm/s, 3 passes, Z down 1 mm a pass. Exactly 3 passes cut
through on both spot axes for `e_v` in 15–26 J/mm³, and the middle, **20 J/mm³**, is used.
With that value:

| Sienci row | Settings | Sienci's passes | Model's passes (both axes) |
|---|---|---|---|
| 3 mm plywood (fitted) | 100 %, 2.2 mm/s, Z step 1 mm | 3 | 3 |
| 6 mm plywood | 100 %, 2.2 mm/s, Z step 1.2 mm | 5 | 5 |
| 1/4" plywood | 100 %, 5 mm/s, Z step 0.4 mm | 16 | 14 |

Two passes do not cut 3 mm (pinned in `tests/test_laser.py`).

`char_j_mm2` is a **look**, not a measurement. Sienci's engrave setting (100 %, 25.4 mm/s,
at a 0.1 mm line interval) lays 2.6 J/mm², and it is set so that setting reads as dark
brown.

## Measured: the tag

`jobs/tag_3mm_plywood.gcode` (`job.py tag`) is a rounded 30 × 20 mm tag with a 6 mm hole,
cut from 3 mm plywood with Sienci's settings. A 10 × 4 mm block is engraved beside the
hole first. The G-code is shaped like LightBurn's: M4, inner shape first, no kerf offset,
`G10 L20 P1 X0 Y0 Z0` with the focus on the sheet's top centre.

The run, measured 2026-11-02:
- **Machine:** RTX 3060 Laptop, AMD 16 cores, MuJoCo CPU solver, engine `0f821658d` with
  this change.
- **Setup:** an 80 × 80 × 3 mm sheet at 0.05 mm, streamed over TCP by `run_job.py`.

| What | Result |
|---|---|
| Job | 135 lines, 196 s of machine time, 33 s on the wall clock (headless) |
| Energy | the laser emitted 1139 J (the job's lengths and speeds give 1136 J); the sheet absorbed 1094 J, 96.0 %. The rest went through the finished cut. |
| The tag | **cut free**, and so did the hole's slug. 29.9 × 19.9 mm at the bottom of the cut (drawn 30 × 20) |
| The hole | 6.06 mm (drawn 6.0) |
| Kerf | 0.26 mm at the top (deeper than 0.1 mm); about 0.05 mm at the bottom, which is one grid cell, the sheet's resolution |
| Engraving | 0.131 mm deep, char 0.89; every raster line starts at x = 0.00 and ends at 10.00 in both directions (no timing comb) |
| M3 against M4 | the same engraving in M3 burns the line ends **74 % deeper** (0.229 against 0.131 mm), where the head slows down; in M4 the ends match the middle (0.132 mm) |

The last row is why LightBurn defaults to M4, and it is the same physics, not a special
case.

## Limits

- **One material.** Plywood, calibrated on one vendor row. Acrylic and cardboard have
  vendor rows but no fit yet.
- **Assumed optics.** The cone angle and the spot's orientation are assumptions (above).
  Cutting along X and along Y differs because the spot does.
- **Energy only.** No heat-affected zone, flame-up, resin or smoke. Air assist is not
  modelled.
- **Resolution.** The sheet's resolution limits the kerf's bottom width. At 0.05 mm an
  80 mm sheet is 1601 × 1601 points, and the engine caps a side at 1984 points.
- **Geometry only.** A part that cuts free is identified by the inspector; it does not
  drop out as a body.

## Files

| File | What |
|---|---|
| `laser.py` | the laser module, materials, GRBL's laser output, the 1D kerf model and `kerf_check` |
| `job.py` | the test part's G-code, in LightBurn's shape |
| `world.py` | the world: the CNC's CAD pack with the schematic head and a laser `MillingStock` sheet |
| `run_job.py` | run a job headless over TCP and keep the sheet dump and trace |
| `inspect_cut.py` | measure the sheet: pieces that fell out, part size, hole, kerf, engraving, energy |
| `jobs/` | the tag's G-code |

The controller is the CNC twin's `grbl_twin` with `--laser <module json>`.
