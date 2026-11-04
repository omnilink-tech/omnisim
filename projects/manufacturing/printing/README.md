# Printer twin: an FDM 3D printer, from G-code to a measured part

A 3D printer in OmniSim, built from the machine's own firmware config, that runs the
G-code a real slicer writes for it and lays plastic where its nozzle really goes. It is
the additive half of the fabrication twins; the [CNC twin](../cnc/README.md) is the
subtractive half, and the two share their drive model.

Status, 2026-11-02: simulation only. No physical printer has been connected or
compared. The twin models the firmware's motion planning, the hotend's flow, and the
geometry of what is laid down. It does not model heating, cooling, warping, layer
adhesion, stringing, ooze or part strength.

## The machine: Sovol SV08

The first printer is the [Sovol SV08](https://github.com/Sovol3d/SV08), a 350 × 350 × 345 mm
flying-gantry CoreXY machine on Klipper, derived from the Voron 2.4. It was chosen
because everything a twin needs is public: the full STEP assembly, the factory
`printer.cfg` and macros, and an official OrcaSlicer profile.

- **Licence.** Sovol publishes the SV08's config and CAD under **GPL-3.0**. Nothing of
  Sovol's is copied into this repository. `../machines/sovol_sv08/sovol_sv08.json` keeps only the
  numbers the twin uses, extracted with `printer.py extract`, our own transcription of
  what the homing override and the start/end macros do, and the commit each came from.
- **Two worlds.**
  - `sv08_printer_twin.omniworld` is schematic, sized from the travel and bed.
  - `sv08_printer_twin_cad.omniworld` is the machine from **Sovol's own CAD**. The mesh
    pack is in `projects/manufacturing/machines/sovol_sv08/cad/` (`world.py --pack`): the full
    assembly cut into frame, gantry, X beam and toolhead. It is 0.79 M triangles: the toolhead
    and beam are at full detail, the frame and gantry coarser, and the parts sealed in the base
    are left out. The nozzle tip and the bed
    surface are taken from the CAD, so plastic lands on the CAD's own PEI sheet.
  - The pack's meshes are derived from GPL-3.0 CAD and say so in their
    [provenance](../machines/sovol_sv08/cad/PROVENANCE.md). The CAD's placeholder
    colours are drawn as dark plastic; those colours are ours.
- **The moving masses are estimates** (Sovol publishes none). They hardly matter: each
  axis moves like at least 10 kg anyway (see "Axis drives").

## How a print runs

| Piece | File | What it does |
|---|---|---|
| Printer | `printer.py`, `../machines/sovol_sv08/sovol_sv08.json` | Reads a Klipper config: kinematics, limits, square corner velocity, steps/mm, extruder, pressure advance, input shaper |
| Slicer | `cura_slice.py` | Slices an STL with the installed CuraEngine, set from the vendor's OrcaSlicer profile; evaluates Cura's setting formulas itself |
| Firmware | `klipper.py` | Plans the job as Klipper does: junctions, look-ahead, Z and extrude-only limits, the errors Klipper stops a print on, then the input shaper and pressure advance at step generation |
| Hotend | `hotend.py` | Turns the extruder motor's motion into the plastic that leaves the nozzle: pressure lag, melt limit, retraction |
| Engine | `PrintedPart` node | Lays that plastic as beads along the nozzle's real path, on the bed or on what is already printed |
| Controller | `fdm_twin` | Drives the three axes as fed-forward servos and feeds the hotend's output to `PrintedPart` |
| Checks | `preflight.py`, `inspect_print.py` | Before: time, filament, flow against the hotend, refusals. After: the printed part against the STL |

```bash
# Slice (the part is centred on the bed; --at x,y places it).
python projects/manufacturing/printing/cura_slice.py part.stl --out part.gcode

# Preflight without the engine: time, filament, flow, what Klipper would refuse.
python projects/manufacturing/printing/preflight.py part.gcode

# A world that prints it.
python projects/manufacturing/printing/world.py --gcode <repo-relative part.gcode> --out my_print.omniworld
python -m omnisim run-world my_print.omniworld

# Measure what was printed (the engine writes its beads when asked).
OMNISIM_PRINTED_PART_DUMP=part.beads python -m omnisim run-headless my_print.omniworld --duration 300
python projects/manufacturing/printing/inspect_print.py part.beads part.stl
```

## What each model assumes

**Firmware (`klipper.py`).** This follows `klippy/toolhead.py` and
`klippy/kinematics/extruder.py`:

- a 90° corner runs at `square_corner_velocity`;
- the extruder limits a junction where the extrusion ratio changes (instantaneous corner
  velocity);
- `max_accel_to_decel` (or `minimum_cruise_ratio`) smooths the look-ahead;
- the Z axis and extrude-only moves have their own limits;
- a job stops at the first line Klipper would refuse.

Then, as Klipper's step generation does, X and Y go through the configured input
shaper (MZV at 35 Hz on the SV08), and the extruder gets pressure advance with its
triangular smoothing. Not modelled:

- **heater warm-up:** a wait for temperature takes no time;
- **probing:** `QUAD_GANTRY_LEVEL` and `BED_MESH_CALIBRATE` are no-ops, and the bed is
  flat and level;
- **steps and resonance:** no step timing, no stepper resonance.

Each is reported by `preflight.py` when a job uses it.

**Hotend (`hotend.py`).** The melt behind the nozzle is a spring: flow out lags the
filament pushed in by a time constant. That is the model Klipper's pressure advance is
built on, and the twin's constant is the factory `pressure_advance` (0.025 s), the
value Sovol tuned to cancel it. The hotend melts at most 21 mm³/s (the vendor's PLA
profile limit); beyond that the gear skips and the line is thin.

**Laying plastic (`PrintedPart`).** Every physics step, the plastic that left the nozzle
during the step is laid along the nozzle's real path:

- **Height:** the gap between the nozzle tip and what is under it, the bed or earlier
  plastic.
- **Width:** whatever that volume needs, as a flattened stadium, but **never wider than
  the nozzle's flat face** (2.5 nozzle diameters, 1 mm on a 0.4 mm nozzle). Plastic is
  flattened into a line only under the face.
- **Bridges:** with nothing close underneath, a round strand hangs from the nozzle (also
  at most the face across).
- **Blobs:** what a step pushes out beyond the face, and the plastic from a nozzle
  standing still (under 0.01 nozzle diameters a step), stays where it left the nozzle as
  a blob. Under the face it is pressed into a disc as tall as the gap, its top at the
  tip; only past what the face holds does it well up around the nozzle, as a puck at
  least face wide. Plastic left while the nozzle is still over its last blob grows that
  blob where it is. A nozzle standing in its own blob does not count as a crash.
- **Ooze:** too little to form a strand of a quarter-nozzle diameter is dropped (ooze is
  not modelled).

**The bead fix, 2026-11-04.** Until then a nozzle moving less than 0.1 nozzle diameters a
step (20 mm/s at 2 ms) counted as standing still. A nozzle slowing into a corner, or
creeping where the melt outran it (pressure advance is smoothed over 40 ms, a corner
decelerates in about 20), held its plastic, and the next faster step laid it on its
0.04–0.05 mm chord as one bead up to 7 mm wide. Past 0.05 mm³ the held plastic became a
half-sphere dome standing on the layer below, 0.09–0.21 mm above the tip, and the nozzle
counted a crash when it next ran over it. Traced step by step in ideal mode on the
SparkRover plates (152, 116, 96 and 11 crashes on plate06, plate01, plate02 and rev03),
**every crash was a dome of that kind and none a wide bead**; on plate06 every dome hit was
laid while the nozzle crept 0.13–0.38 mm, not while it stood, and it was hit a median 18
and at most 3,848 steps later (11 of the 152 by the very next step). Now the plastic is laid along the path the
nozzle really took, and what is left over is a blob at the spot. The measured sections
below give the numbers before and after.

Physics runs at 2 ms. At 300 mm/s the nozzle moves 0.6 mm per step and each step's bead
is straight; at sharp corners Klipper slows to 5 mm/s, so steps there are microns apart.

## Axis drives

The drives are the CNC twin's: OmniSim position servos at 20 kN, with the target fed
forward (velocity and acceleration) and centred on the step it holds for, and an
integral trim at rest for the gantry's weight on Z. See the CNC twin's
["Axis drives"](../cnc/README.md#axis-drives). What differs on a printer:

- **The acceleration feed-forward is the axis's effective inertia over the stiffness,
  and the effective inertia is not the mass.** Newton's servo-armature bound gives a
  light position servo armature up to `dt × kd` (AGENTS.md hard-won rules). At 1 ms
  substeps and kd = 10 kN·s/m, every printer axis moves like 10 kg, even the 0.65 kg
  toolhead. `world.py` computes it and passes it to the controller.
- **No lead trim.** The CNC trims its velocity lead by half its 2 ms substep. At the
  printer's 1 ms substeps the trim measured no better than none.

Measured on the 20 mm cube, 2026-11-02. Error is the measured minus the planned nozzle
position, sampled every 50 ms. The first two rows are from short runs, over job time
70–206 s (the first layers run at 30 mm/s and are left out); the last two rows are from
the full print.

| Feed-forward (s², x / y / z) | Window | X p50 / p99 / max (mm) | Y p50 / p99 / max (mm) |
|---|---|---|---|
| CNC's 1e-4 | 70–206 s | 0.021 / 0.260 / 0.313 | 0.022 / 0.206 / 0.342 |
| Bare masses 3.3e-6 / 6.3e-6 / 2.1e-5 | 70–206 s | 0.010 / 0.229 / 0.288 | 0.010 / 0.175 / 0.201 |
| **Effective inertia 5e-5** | 70–115 s (bottom layers, walls) | **0.009 / 0.016 / 0.049** | **0.008 / 0.017 / 0.057** |
| **Effective inertia 5e-5** | 115–206 s (infill at 233 mm/s) | **0.021 / 0.080 / 0.087** | **0.015 / 0.080 / 0.097** |

What is left on the fast layers is not yet understood; Z stays within 0.010 mm
throughout.

## Measured: the 20 mm calibration cube

`jobs/cube20.stl`, sliced by `cura_slice.py` with the SV08 profile:

- 0.2 mm layers on a 0.25 mm first layer;
- two walls, 10 % grid infill, a 5 mm brim;
- PLA at 220 °C.

Printed headless on `sv08_printer_twin.omniworld`, 2026-11-02, Newton/MuJoCo CPU at
2 ms, RTX 3060 Laptop machine. The print ran about 2× faster than real time.

| | Twin |
|---|---|
| Print time (Klipper's plan) | 375.3 s; Cura estimated 325 s for the same layers |
| Filament | 1161.5 mm, 3.46 g |
| Peak flow asked of the hotend | 16.5 mm³/s (limit 21) |
| Plastic out of the nozzle / laid | 2852.3 / 2850.2 mm³ (2.1 mm³ of ooze dropped) |
| Layers | 100 |
| Height | 20.05 mm (the slicer's layer stack: 0.25 + 99 × 0.2) |
| Outer surface vs the STL, mid-height layers | +0.041 mm (the outline sits that far outside the drawing) |
| Footprint vs the STL, mid-height layers | +0.16 mm in X and Y (20.16 mm) |
| Outer surface, 5th–95th percentile, all layers | +0.015 to +0.070 mm |
| Nozzle crashes into the part | 0 |

Two things make the part oversize:

- **Bead shape.** A slicer meters plastic as a rectangle, line width × layer height,
  but a bead is rounded, so the same volume spreads 0.035 mm wider (0.017 mm a side).
- **The drives.** The axes' remaining error and the step-to-step width variation
  account for the rest.

Real printers also print slightly oversize, but **no real SV08 has been measured to
compare.** These are the twin's own numbers, not a claim about the machine.

The first layer is not counted: it is fused to the brim, as on a real print. Before the
bead fix (2026-11-04) the `inspect_print.py` layer table showed the first layer +3.2 mm
wide, and that was blamed on the brim. It was not: a brim loop's centre lies outside the
drawing and is never counted. It was the wide beads where the first layer's slow lines
started. After the fix the twin's first layer reads +0.17 / +0.13 mm, like the layers
above it.

**After the bead fix**, re-printed on the twin 2026-11-04 (scratch engine
`omnisim-bin-v10beads`, the same world and machine, other sessions' engines running
alongside, 532 s wall): plastic laid 2850.27 mm³ (2.0 mm³ dropped), 0 crashes, 27,128 beads
(was 24,488) of which 721 are blobs. The mid-height layers are unchanged (outer surface
+0.041 mm, footprint +0.16 / +0.15 mm), the 5th–95th percentile is +0.014 to +0.070 mm,
and the footprint over all layers is +0.164 / +0.154 mm (was +0.201 / +0.191: some
layers read up to 0.69 mm wide, the widest outer point was 1.68 mm out, now 0.10).

## Measured: a 145 g shell section

What a big part costs, measured 2026-11-03 on one section of a 210 × 185 × 290 mm
rounded-box robot shell (25 mm edge radii, 2.5 mm walls): its upper half on one side,
185 × 140 mm on the bed and 105 mm tall, printed closed face down with the open cuts up.
The 25 mm fillets that rise from the bed are undercut and need support. The test part
is not in the repository (it was generated for the measurement). Sliced by
`cura_slice.py` with `--set support_enable=true --set support_type=buildplate` and a
PETG copy of the SV08 profile (OrcaSlicer's "Generic PETG @Sovol SV08": 235/255 °C,
bed 75 °C, fan 30–70 %, 17 mm³/s). Headless, schematic world, RTX 3060 Laptop /
Ryzen 7 5800H machine, other sessions' engines running alongside.

| | Twin |
|---|---|
| Print time (Klipper's plan) | 11,758 s (3.3 h); the G-code's `;TIME:6666` is a placeholder CuraEngine leaves for its GUI to fill, not an estimate (its `;TIME_ELAPSED:` lines run -24 % to +5 % of preflight) |
| Moves, extrusion moves, layers | 434,842, 257,903, 525 (29 with support) |
| Filament | 47.6 m, 145.5 g of PETG (`preflight.py --material PETG`) |
| Plastic laid | 114,417 mm³ (99.9 % of what left the nozzle) |
| Beads, bead dump | 1,018,371, 35 MB |
| Wall time, one engine | 6,551 s (1.8× faster than real time) |
| Peak RAM | engine 770 MB; controller 1,548 MB, 612 MB since the windowed sampler |
| `inspect_print.py` | 74–90 s, 387 MB |
| Layers, height | 525, 105.05 mm |
| Outer surface, 5th–95th percentile, all layers | −0.042 to +0.059 mm |
| Footprint vs the STL, mean | +0.06 mm in X, +0.12 mm in Y |
| Nozzle crashes into the part | 0 |

These are the twin's numbers before the bead fix (2026-11-04); the shell's twin has not
been re-run since. In ideal mode the fix moved its footprint from +0.05 / +0.05 to +0.04 /
+0.04 mm and left its outer-surface percentiles where they were (see "Ideal print").

What it took (all fixed, each with a test):

- **The engine kept only 523,776 beads** (1023 chunks of 512, the renderer's mesh slots
  per part). Past that, plastic was counted but no bead was kept, so the dump held 52 %
  of the plastic and the inspector measured the bottom 32 mm of a 105 mm part without
  saying so. Beads past the drawn chunks are now recorded and dumped, only not drawn
  (`tests/test_printed_part_bead_cap.py`, engine). The inspector also reports
  `recorded_mm3` and `unrecorded_fraction` and warns when the beads hold less than the
  part laid.
- **The bead dump is written in one write**, at most once a second and no more often
  than 50 times its own write time, and within a second of the last change.
- **The controller sampled the whole job up front**: 5.9 M steps, 1.55 GB at the peak,
  growing with the print. `klipper.StepSampler` samples a window at a time with exactly
  the same values (`tests/test_printer_twin.py`).
- **A bead laid while Z still rises into a layer** (the servo trails a layer change by
  ~20 µm) made a "layer" of one bead, measured as −170 mm of footprint. The inspector
  folds clusters with under 2 mm of path into the nearest layer.

Support is left out of the measurement by where it is (the slicer keeps it 0.7 mm
clear of the part), not by a tag: the dump does not say which feature a bead was.

## Ideal print (no engine)

`ideal_print.py` prints a job without the engine, in seconds to minutes instead of
hours. It runs the twin's chain and leaves out only the axes:

- the same Klipper plan (`klipper.py`), with the input shaper and pressure advance;
- the commanded nozzle position sampled at the twin's 2 ms step times, the same floats
  as `klipper.StepSampler`;
- `hotend.py`'s extrusion, stepped as `fdm_twin` steps it (compiled, bit for bit the
  same);
- PrintedPart's laying rule step by step: height from the map under the nozzle, width
  from the volume up to the nozzle's face, straight runs merged, blobs, bridges and
  dropped ooze, with the engine's constants and float32 beads.

It writes a dump in PrintedPart's format, so `inspect_print.py` reads it unchanged, plus
the inspect report.

```bash
python projects/manufacturing/printing/ideal_print.py part.gcode part.stl --printer <profile.json>
```

**What it leaves out.** The nozzle is exactly where Klipper's steppers would put it. On
the twin it is where the servo-driven axes carried it, so the axis dynamics are missing:
following error, overshoot, the servo's step-to-step jitter, and anything else the
engine adds. Everything the twin does not model is missing here too (see "What each
model assumes").

**Parallelism.** The trajectory is sampled in threads, 8 of them. Windows' spawned
processes took about 14 s just to start. The inspection runs in processes (`--jobs`).
The hotend and the laying are sequential, because each step depends on the last, but
they are compiled (numba; without it they run as plain Python, much slower).

**Validated against the twin's own dumps, 2026-11-04.** Each part was printed both ways
and measured by the same `inspect_print.py`:

- **Cube:** `jobs/cube20.stl`, PLA. Twin re-run 2026-11-04 on the schematic world.
- **Plate:** the desk robot's print plate, PLA. Twin run 2026-11-03 on the CAD world.
- **Shell:** the 145 g PETG shell section above, the twin run that kept all 1,018,371
  beads.

Machine: Ryzen 7 5800H / RTX 3060 Laptop, with other sessions' engines running alongside.

**After the bead fix** (2026-11-04, the cube twin on the scratch engine
`omnisim-bin-v10beads`; the plate and the shell in ideal mode only, their twins not
re-run; blobs counted in the outline):

| | Cube, ideal / twin | Plate, ideal (before the fix) | Shell, ideal (before the fix) |
|---|---|---|---|
| Plastic laid (mm³) | 2850.30 / 2850.27 | 10311.86 (10311.66) | 114417.98 (114417.34) |
| Layers, height (mm) | 100, 20.05 / same | 57, 11.45 (same) | 525, 105.05 (same) |
| Beads, of them blobs | 27,694 / 27,128 (+2.1 %), 721 on the twin | 232,391, 1,367 (218,716) | 1,090,598, 2,075 (1,042,811) |
| Nozzle crashes | 0 / 0 | 0 (0) | 0 (0) |
| Outer surface, mean (mm) | +0.013 / +0.041 | −0.018 (+0.001) | +0.009 (+0.010) |
| Outer surface, p05 / p95 (mm) | −0.005, +0.019 / +0.014, +0.070 | −0.065, +0.027 (−0.064, +0.033) | −0.069, +0.084 (same) |
| Outer surface, max (mm) | +0.09 / +0.10 | +0.19 (+1.76) | +0.15 |
| Footprint vs the STL, X / Y (mm) | +0.06, +0.05 / +0.16, +0.15 | +0.01, +0.06 (+0.18, +0.26) | +0.04, +0.04 (+0.05, +0.05) |
| Wall time | 1.9 s + 17 s inspect / 532 s | 6 s + 56 s inspect | 34 s + 163 s inspect |

On the cube the ideal still stays within the twin by what the table before the fix
shows: −0.027 mm on the mean outer surface, −0.019 / −0.051 on p05 / p95, −0.10 / −0.11 on
the footprint, the same height, layers and volume. The ideal cube's outer surface is now
+0.013 mm, under the 0.017 mm of bead-shape oversize: before the fix its wide beads added
0.005 mm.

**Before the bead fix** (the dumps of 2026-11-03 and 2026-11-04):

| | Cube, ideal / twin | Plate, ideal / twin | Shell, ideal / twin |
|---|---|---|---|
| Plastic laid (mm³) | 2850.22 / 2850.18 | 10311.66 / 10311.61 | 114417.34 / 114417.42 |
| Layers, height (mm) | 100, 20.05 / same | 57, 11.45 / same | 525, 105.05 / same |
| Beads | 25,023 / 24,488 (+2.2 %) | 218,716 / 212,361 (+3.0 %) | 1,042,811 / 1,018,371 (+2.4 %) |
| Outer surface, mean (mm) | +0.018 / +0.045 | +0.001 / +0.022 | +0.010 / +0.013 |
| Outer surface, p05 / p95 (mm) | −0.006, +0.019 / +0.015, +0.070 | −0.064, +0.033 / −0.048, +0.071 | −0.069, +0.084 / −0.042, +0.059 |
| Footprint vs the STL, X / Y (mm) | +0.11, +0.09 / +0.20, +0.19 | +0.18, +0.26 / +0.24, +0.33 | +0.05, +0.05 / +0.06, +0.12 |
| Per-layer outer mean, ideal − twin, p50 / p95 (mm) | −0.027 / −0.029 | −0.022 / −0.025 | 0.003 / 0.011 in magnitude |
| Bead centre lines, ideal to nearest twin, p50 / p99 / max (µm) | 11 / 63 / 174 | 12 / 66 / 236 | 8 / 82 / 204 |
| Bead width, ideal − nearest twin, p50 / p95 of the magnitude (µm) | 1.5 / 12 | 2.2 / 37 | 0.2 / 11 |
| Wall time | 1.2 s + 4 s inspect / 418 s | 3.9 s + 51 s / 1,110 s | **21 s + 116 s / 6,551–6,879 s** |

The extruded and skipped volumes are identical (the hotend gets the same commands).
The twin's part is the larger one on every print. Its servo error and overshoot, about
12–40 µm median and up to 0.3 mm, carry the nozzle slightly out:

- by 0.02–0.03 mm a side on the cube and the plate;
- by 0.003 mm on the shell's long walls, though the shell is 0.06 mm longer in Y. That
  is the axis with the twin's largest error on this print: 40 µm median, against 12 µm
  in X.

The cube's ideal outer surface, +0.018 mm, is the bead-shape oversize alone (0.017 mm a
side, see the cube above). So the twin's further +0.027 mm is its drives.

Where they differ most: at the starts of long straight lines. A merged bead is a straight
chord from its first step to its last, so the nozzle's position at that first step tilts
the whole line. At a corner that position is off the line by the shaper's smoothing on
the ideal and by servo error on the twin. On the shell that tilt makes the ideal's p05 /
p95 wider than the twin's. In the per-layer tables the twin's footprint is larger on
every layer. The biggest per-layer footprint gaps:

- 0.13–0.15 mm on the cube's top layers;
- 0.25 mm in Y on the shell's fillet layers;
- 0.50 mm on two of the plate's lower layers (z 3.05 and 3.65).

The mean footprint gaps are those in the table.

**When to use which.**

- **Ideal:** part geometry, volume, layer structure, flow and slicer checks, a long
  print, or many variants. It stays within the twin by:
  - ≤ 0.03 mm on the mean outer surface;
  - ≤ 0.06 mm on the outer-surface p05 / p95;
  - ≤ 0.11 mm on the footprint;
  - the same height, layers and volume.
- **Twin:** anything that depends on how the machine moves, such as corner overshoot,
  axis tuning, or the drives' share of the oversize. Use it also to watch or film the
  print.

## A slicer bug the twin caught

The first cube came out with solid layers half empty: thin round strands where 0.4 mm
lines should be. The engine had laid exactly what the G-code asked for, and the G-code
asked for 38 % of the plastic. CuraEngine keeps a per-extruder copy of every extruder
setting, defaulted from `fdmextruder.def.json` rather than inherited, so the 1.75 mm
filament set globally never reached the extruder, and every E value was computed for
2.85 mm filament. A real SV08 would have under-extruded the same way.
`cura_slice.py` now passes those settings to extruder 0 as well, and
`tests/test_printer_twin.py` checks the cube job's flow.

## Tests

- `tests/test_printer_twin.py`, engine-free:
  - the config parser and the SV08 profile;
  - Klipper's corner speed and trapezoid timing;
  - the refusals and the homing sequence;
  - the shaper, and the hotend with and without pressure advance and at its melt limit;
  - preflight of the cube job;
  - the filament the job was sliced for;
  - the slicer itself, skipped without Cura;
  - the world's fields against `PrintedPart.wrl`;
  - the windowed step sampler against sampling the whole job;
  - the inspector against hand-made beads, a dump missing plastic, and a bead laid
    while Z rises.
- `tests/test_printed_part_bead_cap.py`, engine (about two minutes): beads past the
  renderer's 523,776 are recorded and dumped.
- `tests/test_printed_part_bead_width.py`: the bead fix. Engine-free, the rule on a
  scripted path (a fast line, a slow creep the melt outruns, two dwells, a second layer):
  no line wider than the face, every bit of plastic laid, the dwell blobs as a disc and a
  puck, no crash. With the engine (a few seconds), the engine's dump of the same path
  equals `ideal_print`'s to float32 rounding.
- `tests/test_ideal_print.py`, engine-free, about 12 s (plus one numba compile):
  - the step sampling against `StepSampler`, and the hotend against `hotend.py`, bit
    for bit;
  - the laying rule on a line, a second layer, a blob, a bridge and ooze;
  - a nozzle at 10 mm/s lays one line (before the bead fix it counted as standing
    still), a creep with four times the plastic is capped at the face with the rest in
    blobs, and plastic held at a stop is a disc where the nozzle stood;
  - the compiled laying against its Python;
  - the dump round-trip through `inspect_print.py`;
  - the ideal cube against the twin's within the bounds measured above.
