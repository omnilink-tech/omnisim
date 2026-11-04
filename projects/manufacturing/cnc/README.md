# CNC twin: a virtual GRBL controller on a simulated gantry

A 3-axis CNC machine in OmniSim, built from the machine's own controller
settings, behind a virtual GRBL 1.1 controller that any GRBL sender can
connect to over TCP. It is the first piece of the CNC service: rehearse a job
on the twin, see what the real machine would do, before anything moves.

Status, 2026-11-01: simulation only. No physical machine has been connected
or compared. The twin models motion, limits, job time, tool changes and
**material removal** (2.5D pockets and 3D surfaces, measured against the
drawing). It does not model cutting forces, spindle load or tool deflection.

## What it does

- **Machine from config.** `machine.py` reads a GRBL `$$` dump or a FluidNC
  `config.yaml` (x, y, z only): travel, max rates, acceleration, soft/hard
  limits, homing direction and pull-off.
- **Virtual GRBL 1.1** (`grbl.py`, `server.py`). It implements the line
  protocol (`ok`/`error:N`), real-time commands (`?`, `!`, `~`, Ctrl-X, jog
  cancel, overrides), `$$`, `$x=val`, `$#`, `$G`, `$I`, `$N`, `$C`, `$X`,
  `$H`, `$J=`, `$SLP` and `$RST=`. It raises soft limits (ALARM:2, jog
  error:15), hard limits (ALARM:1), reset in motion (ALARM:3) and the boot
  alarm when homing is enabled. Sync commands (dwell, M3/M5, M0, M2/M30)
  reply `ok` only once the buffer reaches them.
- **Belief vs reality** (`twin.py`). GRBL only knows its step count. The twin
  also keeps where each carriage physically is, so it can show:
  - a never-homed machine believing MPos 0 at mid-rail;
  - a hard limit tripping inside the soft envelope when the rail is shorter
    than `$13x` (`--rails`);
  - lost steps when a carriage is driven into its end stop with hard limits
    off.

  `$TWIN` (an extension, not GRBL) prints both as `[TWIN:{json}]`.
- **Preflight** (`preflight.py`). It runs a whole job through the same
  interpreter and reports:
  - errors per line;
  - envelope violations (needs the work offset);
  - bounds;
  - feeds the machine will silently clamp;
  - feed moves with the spindle off;
  - rapids below Z0;
  - an estimated run time.
- **World generator** (`world.py`). It builds a moving-gantry `.omniworld`
  sized from the rail lengths. Geometry is schematic, not CAD.

## Run it

```bash
# In OmniSim: the twin listens on 127.0.0.1:2323 once the world is running.
python -m omnisim run-world projects/samples/demos/worlds/manufacturing/cnc_3018_twin.omniworld

# Without the engine: same controller, ideal plant, wall clock.
python projects/manufacturing/cnc/server.py --machine projects/manufacturing/machines/example_3018/example_3018_with_switches.grbl

# Preflight a job (work offset G54 in machine mm).
python projects/manufacturing/cnc/preflight.py job.nc --machine <cfg> --wco -200,-120,-40

# A world for your machine (optionally with the as-built rail lengths).
python projects/manufacturing/cnc/world.py --machine my_machine.grbl --out my_cnc.omniworld
```

Point a sender at TCP `127.0.0.1:2323` (FluidNC-style telnet). A sender that
only opens serial ports needs a TCP-to-virtual-COM bridge, which is not
provided. The in-engine controller writes a trace to `.tmp/cnc-twin/trace.jsonl`
(override with `OMNISIM_CNC_TWIN_TRACE`).

## Measured (2026-10-31, this machine, `example_3018_with_switches.grbl`)

Headless OmniSim, Newton/MuJoCo CPU, 8 ms step, driven over TCP:

- **Protocol:** boot alarm, `$H`, a 12-line job (rapids, linear, G3 arc,
  M3/M5, M30) all answered `ok`. `G53 G0 X5` raised ALARM:2, reset relocked,
  and `$J=G91 X-400` returned error:15.
- **Tracking:** the simulated carriages followed GRBL's belief within
  0.061 mm (X), 0.028 mm (Y) and 0.006 mm (Z) during the job, after the
  2026-11-01 drive fix (see "Axis drives" below). Before the fix it was
  0.43 / 0.25 / 0.29 mm.
- **Load check:** `run-headless --until-finalized --fail-on-warning` PASS.
- **Job time:** preflight's estimate matched the engine-free controller's
  executed time within the test tolerance (`tests/test_cnc_twin.py`).

## A real machine: Sienci LongMill MK2 30x30

`projects/samples/demos/worlds/manufacturing/cnc_longmill_mk2_twin.omniworld` is
the LongMill built from Sienci Labs' own CAD (public Onshape document,
CC BY-SA; see `projects/manufacturing/machines/longmill_mk2_30x30/cad/PROVENANCE.md`). The
meshes are **not** Apache-licensed. It runs Sienci's GRBL defaults from gSender
(`../machines/longmill_mk2_30x30/longmill_mk2_30x30.grbl`): 810 x 855 x 120 mm, with homing and
limits off as shipped, so GRBL believes MPos 0 wherever the gantry sits at
power-up.

From CAD to twin:

```bash
# 1. Export the assembly as STEP (Onshape: right-click the assembly tab > Export > STEP).
# 2. Cut it into frame / gantry / carriage / z by assembly path (manifest.json).
python projects/manufacturing/core/cad_pack.py projects/manufacturing/machines/longmill_mk2_30x30/cad/manifest.json \
    --cad-dir .tmp/cnc-cad --out projects/manufacturing/machines/longmill_mk2_30x30/cad
# 3. Generate the world: X/Y power up where the CAD has them, Z raised 76 mm, a stock block under the bit.
python projects/manufacturing/cnc/world.py --machine projects/manufacturing/machines/longmill_mk2_30x30/longmill_mk2_30x30.grbl \
    --pack projects/manufacturing/machines/longmill_mk2_30x30/cad --s0 394,441,100 --stock 180,180,12 --tool-d 6.35 \
    --out projects/samples/demos/worlds/manufacturing/cnc_longmill_mk2_twin.omniworld
```

Measured 2026-10-31 through the harness, driven over TCP:

- **Full travel:** both corners reached with no visible interpenetration
  (X 0 to 810, Y 0 to 855, Z 120 to 0 mm). At rest, measured equals commanded.
- **Tracking:** before the drive fix, the carriages strayed up to 3.9 mm (X)
  and 4.4 mm (Y) during the star job. After it, the real tool stayed within
  0.076 mm of GRBL's path. With the centred feed-forward it stayed within
  0.026 mm, and with the velocity lead trimmed it stays within **0.006 mm**
  (p99 0.003 mm; 2026-11-01).
- **Load check:** `run-headless --until-finalized --fail-on-warning` PASS.

## Axis drives

The axes are OmniSim position servos. The engine builds one with stiffness
`ke = 10 × maxForce` and damping `kd = 0.5 × maxForce`, so the ratio
`kd / ke` is a fixed 0.05 s whatever the force. A servo like that differs from
a stepper on a lead screw in two ways:

- **It trails a moving command by 0.05 s × speed.** That is 3.3 mm at
  4000 mm/min.
- **It lags under acceleration** by (effective mass ÷ stiffness) ×
  acceleration.

Three measures make the twin move like a stepper drive (`grbl_twin`):

1. **The servo target is fed forward and centred on the step it holds for.**
   A target set at one physics step holds until the next one, so the planner
   runs two steps (16 ms) ahead of the physics. Each target is then the
   command's midpoint over the coming step, plus `0.05 s ×` its velocity over
   that step, plus a per-axis coefficient × its acceleration at the middle of
   the step. Before this, the target came from the command at the start of
   the step and the velocity and acceleration of the step before. That was
   half a step to a step late, and every plunge stopped 14–35 µm too deep.
   Fed forward exactly, the 0.05 s lead left every axis **ahead** of its
   command by 0.95 ms × speed (48 µm at 50 mm/s in Z, about half the engine's
   2 ms substep), so the velocity lead is shorter by that much
   (`VELOCITY_LEAD_TRIM_S`). The acceleration coefficient was then measured on
   a 3D finishing staircase: Z at 2500 mm/min down and up, 90-degree turns into
   X rows, and 600 mm/min plunges, sampled every step. Worst error over the run:

   | Acceleration coefficient (s², every axis) | 0.9e-4 | **1e-4** | 1.1e-4 |
   |---|---|---|---|
   | Z (µm) | −5.1 / +6.1 | **−4.7 / +4.5** | −8.6 / +7.6 |
   | X (µm) | −5.5 / +5.1 | **−5.3 / +5.3** | −9.4 / +9.7 |

   Before the lead trim the best values (Z 2e-4, X/Y 1.25e-4 s²) left 60–70 µm
   at those corners, and a ball nose finishing against a wall cut 0.28 mm into
   its fillet.
2. **The drives are 20 kN** (`world.py SERVO_FORCE_N`). A lead-screw drive is
   effectively rigid; at 600 N the 12.5 kg gantry overshot every reversal.
3. **The integral trim** that removes static sag (the router's weight on Z)
   **integrates only at rest.** Integrating during motion wound it up and
   pushed the tool past each reversal.

**The CAD pose is estimated.** It comes from each carriage's offset from its
rail centre, not from homing switches, so a few mm of the travel's position
along each rail is uncertain. Colliders are 2 cm tokens: nothing checks for
crashes against the machine, clamps or stock yet.

## Milling a part (material removal)

The LongMill world carries a 180 x 180 x 12 mm MDF block centred under the bit
at power-up (`world.py --stock 180,180,12 --tool-d 6.35`).

- **What carves it:** the engine node `MillingStock`
  (`resources/nodes/MillingStock.wrl`, `src/omnisim/nodes/OmMillingStock.*`).
  - It keeps a 0.2 mm height map (a Z-map) and carves it every physics step,
    sweeping the cutter along the router Solid's **real** path.
  - It supports flat, ball-nose and V-bit cutters.
  - It is drawn in 64-cell tiles through the renderer's dynamic-draw path, so
    only tiles the cutter touched are re-uploaded and nothing re-walks the
    scene.
  - A flat tile (untouched, or a flat pocket floor) is one quad.
- **The controller's part:** it keeps a coarse 1 mm model of its own, only to
  judge dry cuts (material entered with the spindle stopped), because only it
  knows the spindle state. `$TWIN` reports both its own figures and the
  engine's.
- **Fallback:** `world.py --stock-renderer grid` uses supervisor-edited
  ElevationGrids instead, for an engine without the node.
- **What it doesn't model:** this is geometry only. There are no cutting
  forces, chip load or deflection.
- **Making cuts visible:** the top is a painted skin and the core is light
  MDF. A flat-colour pocket is nearly invisible under sky light.

Why a node, measured 2026-10-31 on a 180 mm block:

- **Edits from the controller are cheap,** 0.05–0.15 ms per step.
- **The engine rebuilt the whole ElevationGrid and re-walked the scene on
  every edit,** and leaked a GPU mesh each time:
  - at 1 mm the window ran at 0.51× real time;
  - at 0.5 mm it ran out of GPU memory;
  - at 0.25 mm it ran at 0.07×.

GPU load over the whole star job, RTX 3060 Laptop, same camera, measured
2026-11-01:

| Run | GPU util mean | Power |
|---|---|---|
| Native node, all tiles at full detail | 55% | 68.7 W |
| With flat tiles collapsed | 37% | 62.6 W |
| The same scene with no stock | 37% | 50.6 W |

The rest of that GPU time is spent shading the machine itself. Per-pass GPU timers
(`OMNISIM_WGPU_GPU_TIMING=1`) put the main scene pass at 74% of a 6.5 ms frame;
sun shadows are only 5%. So the generated worlds cap the window at
`world.py RENDER_FPS` = 30 (`WorldInfo.FPS`). Physics still steps every 8 ms, and
when the machine is idle the unchanged frame is already held. The same star job,
run back to back, measured 2026-11-01:

| Window | GPU util mean | Power | GPU temp mean / max | Job |
|---|---|---|---|---|
| 60 fps | 40% | 57.9 W | 83 / 91 °C | 125.4 s |
| 30 fps | 31% | 31.4 W | 68 / 72 °C | 120.5 s |

The part is identical either way: +0.11% volume, 3.034 mm deep (before the
drive-timing fix below). Utilization drops less than
power because the GPU clocks down (median 1537 to 922 MHz). For smoother motion,
regenerate with a higher `RENDER_FPS`.

`toolpath.py` is a minimal 2.5D CAM. It does zig-zag clearing of a shapely
region shrunk by the tool radius, plus a finishing contour along every wall
and island.

The star job, measured 2026-11-01 (a star standing in a 140 mm round pocket,
3 mm deep, 670 lines, streamed over TCP):

- **Time:** the job runs in the window in 114.8 s, against a preflight
  estimate of 107.5 s.
- **Material removed (engine):** 39,856.2 mm³. The drawing is 39,881.6 mm³,
  and what a 6.35 mm end mill can reach of it is 39,868.7 mm³ (−0.031%).
- **Deepest point:** 3.001 mm for a 3 mm pocket.
- **Dry cuts:** none.
- **Before the drive fix:** the deepest point was 3.36 mm and the walls showed
  1 mm bumps at every reversal.

### How accurate is the part

`inspect_part.py` measures the part the engine actually carved. Set
`OMNISIM_MILLING_STOCK_DUMP=<file>` and the MillingStock node writes its height
map. The script compares that map with the drawing, and with the drawing opened
by the cutter radius. The second is what a perfect machine can cut with this end
mill: a round tool cannot make a sharp inside corner, on any machine.

```bash
python projects/manufacturing/cnc/inspect_part.py star.zmap --part star_in_circle \
    --tool-d 6.35 --depth 3 --work-origin-world 84.227,-61.818
```

The star job, measured 2026-11-01 from the engine's height map (901 × 901 points
at 0.2 mm):

| | First measurement | Drive timing and precision fixed | Velocity lead trimmed |
|---|---|---|---|
| Walls, outside / inside the ideal edge | 0.031 / 0.023 mm | 0.006 / 0.007 mm | **0 / 0.0003 mm** |
| Grid points that disagree with the ideal part | 68 | 19 | 1 |
| Floor depth error (3 mm target) | 0 to +0.034 mm | −0.002 to +0.001 mm | **0 to +0.0008 mm** |
| Volume against the ideal part | +0.15% | −0.025% | −0.031% |
| Stock outside the pocket | untouched | untouched | untouched |

The first measurement found two faults in the twin, not in the geometry:

- **Every plunge went 10–35 µm too deep.** The drive's feed-forward was half a
  step to a step late (see "Axis drives").
- **The whole floor sat 4 µm deep.** The generated world wrote metres to 10 µm,
  so the engine's cutter tip landed 4 µm lower than the controller's. The world
  is now written to 0.1 µm.

The star's five inside corners keep 4.31 mm² of material at the cutter's
3.175 mm radius, as they would on the real machine.

A height map locates a wall only at its grid points, so it does not show detail
finer than 0.2 mm. What the twin's part leaves out is listed under "Not modelled"
(cutter deflection, backlash, tram, runout).

## Tool changes

A machine's cutters are a **tool table** (`tool_table.py`), one JSON per machine:
`../machines/longmill_mk2_30x30/longmill_mk2_tools.json` holds a 1/4" flat end mill (T1), a 1/8" ball
nose (T2) and a 60-degree V-bit (T3). Each tool has a shape, a diameter and a
`stickout_mm`: how much lower its tip sits than T1's. Work Z is zeroed with T1.

GRBL 1.1 has no M6: it answers `error:20`, and the twin does the same. As
gSender does, the **sender** (`grbl_link.GrblLink.stream`) performs the change:

1. It sends the rest of the line (`T2`) and waits for motion to stop (`G4 P0`).
2. It stops the spindle (`M5`). The program has to start it again, as CAM posts do.
3. It loads the tool with `$TWIN TOOL=2`, a twin extension that stands for the
   operator's hands. The engine's cutter becomes the ball nose: shape, diameter,
   and the tip moved down by the stickout.
4. It reads the tool's length from `$TWIN`, standing in for a tool setter, and
   applies `G43.1 Z<stickout>`, so the same work Z puts the new tip at the same height.

A real board answers `error:3` to `$TWIN`, so on hardware the job **stops at
the M6** and says why, rather than cutting on with the wrong bit. Probing (G38)
is not modelled yet, so the tool-setter step is not available on hardware.

Preflight models the same change: the length offset (machine Z bounds move by
the stickout), a T number missing from the table (`error:38`), and cuts after
the change with the spindle stopped. The OmniLink bridge has a `change_tool`
action and reports the tool in the spindle. `cut_pocket` refuses unless a flat
end mill is in, and preflight starts from the controller's real offset (`$#`).

### What you see

The CAD's own bit is left out of the mesh pack (`cad_pack.py` `tool_part`: the
lowest router part, found by position because STEP part names carry random
suffixes). The pack keeps the tool tip where it was and records the collet. The
world draws the collet and one fluted bit per tool in the table, at its length
and shape (`world.py` `_tool_nodes`). What cuts is still `MillingStock`'s cutter;
these are pictures of it.

- **Turning:** while the spindle runs and the machine moves, the bit turns at
  4 turns per second. At the real 18,000 rpm it would turn 600 times per
  displayed frame and look frozen or random. So the drawn turning means
  "spindle on", not its speed.
- **Spin blur:** while the spindle runs, a translucent sleeve over the flutes
  shows the blur a spinning cutter makes. When the machine stands still, the
  bit stops turning and the sleeve alone says "spinning", so the window can
  hold its frames. Measured 2026-11-01 (RTX 3060 Laptop, 30 fps window, machine
  still, spindle on): turning the bit every frame cost 34.0% GPU at 32.3 W; with
  the sleeve it is 8.8% at 17.6 W, the same power as spindle off (17.4 W).
  Moving, it is about 34% either way, because a moving machine is redrawn
  every frame anyway.
- **Tool change:** the old bit retracts up into the router (0.4 s) and the new
  one comes down (0.4 s). Bits not in use wait inside the router body.

## Streaming

`GrblLink.stream` uses **character counting**, as UGS and gSender do: it keeps
GRBL's 128-byte receive buffer full instead of waiting for each `ok`. A 3D
finishing pass is tens of thousands of sub-millimetre moves; one round trip per
line starves the planner. On the first error it stops sending and feed-holds,
because lines already in GRBL's buffer would still run.

A line longer than 127 bytes, comments included, cannot fit GRBL's buffer at
all: the sender refuses it, and `preflight.py` fails it as `line_too_long`.
The CAM keeps its header comments short for this reason. A 150-character
header stopped the clothes-rack connector's first run on line 1.

## 3D parts

`toolpath3d.py` is a small 3D CAM for a 3-axis router, from an STL:

1. **Surface:** the STL's top surface, rasterised onto a grid (0.1 mm here).
2. **Drop-cutter:** at each grid point, the lowest the tool tip can go without
   the cutter entering the part, for flat, ball and V-bit cutters (a greyscale
   dilation, done by scipy).
3. **Roughing:** a flat end mill clears Z levels and leaves an allowance.
4. **Finishing:** a ball nose rides the surface in a zig-zag along X, plus
   rows along Y over faces steeper than 60 degrees across the rows.

Moves between grid points are made safe:
- **Staircase:** a steep step rises before it moves across, or moves across
  before it descends. A straight move would cut the corner off the climb.
- **Link moves** take the highest of the four grid points around them.

`job()` writes both passes into one program with the M6 between them.
`parts.py` holds test parts with an exact formula, so the result can be
measured against the truth. `mould` is a 70 mm cavity, 10 mm deep, holding a
spherical dome, a ring of round section, and four bosses with 45-degree
chamfers.

```bash
python projects/manufacturing/cnc/parts.py mould --out mould.stl
```

`inspect_part.py --stl` measures a 3D part three ways:
- **Twin vs ideal cut:** the ideal cut is the program's exact path swept into the
  same grid by `cut_program`, what a perfect machine leaves. The difference is
  the twin's own error.
- **Ideal cut vs STL:** the program's own result. That covers scallops,
  material the tool cannot reach, and any gouge.
- **Along the surface normal:** on a steep face, a sideways error of a few µm
  is millimetres vertically, so each comparison is also taken along the normal.

The mould, measured 2026-11-01. World `cnc_longmill_mk2_3d.omniworld`: a
100 × 100 × 20 mm block at 0.1 mm. Program: rough with T1 (6.35 mm flat, 2.5 mm
levels, 0.4 mm left), finish with T2 (3.175 mm ball, 0.6 mm stepover), 70,443
lines, two tool changes. It took 23 min of machine time and 3.6 min of wall time
headless in fast mode.

| | Result |
|---|---|
| Twin vs ideal cut, along the normal | −0.012 to +0.024 mm (p01 −0.004, p99 +0.014) |
| Points cut into the STL by more than 0.01 mm | **0** (deepest 0.006 mm) |
| Program vs what the ball can reach (scallops) | p95 0.013 mm, p99 0.028 mm along the normal |
| Material no 3.175 mm ball can reach (fillets, corners) | p99 0.84 mm along the normal; any machine leaves it |
| Volume, engine vs ideal cut | 38,801 vs 38,815 mm³ (−0.04%) |

Before the CAM's staircase fix, the ideal cut gouged the pocket walls by
3.6 mm. Before the drive's lead trim, the twin cut up to 0.28 mm into the
ball's fillets. Both were found by this measurement.

## Cutting load

`cutting.py` turns the material the engine actually removes each step into the
load the cut puts on the machine. Each figure is averaged over 0.1 s and
reported in `$TWIN` (`cutting`), in the trace, and in the bridge's state:

- **Removal rate** (MRR, mm³/s).
- **Cutting power** = specific cutting energy u × MRR, with the range the
  material's u spans.
- **Spindle torque** at the programmed RPM, and **mean tangential force** at
  the cutter's rim.
- **Chip load per tooth** = feed ÷ (flutes × RPM).
- **Spindle load** against the rating in the tool table (the LongMill's Makita
  RT0701C: 700 W, 10,000–30,000 rpm, per Sienci). Going over it raises a
  `spindle_overload` event.

Specific cutting energy, each value with its source in `cutting.MATERIALS`:

| Material | u (J/mm³) | Source |
|---|---|---|
| Aluminium alloy | 0.7–0.8 | MSU ME478 notes (0.25 mm chip) |
| Brass | 0.63–1.15 | Mitsubishi Materials Kc, 630–1150 MPa |
| Cast iron | 1.1–1.6 | MSU ME478 notes |
| Carbon steel | 1.6–2.8 | MSU ME478 notes |
| Stainless steel | 2.8 | MSU ME478 notes |
| Particleboard | 0.032–0.038 | Peripheral up-milling study (Materials, 2021) |
| MDF | 0.032–0.038 | **Estimated**: particleboard's; no MDF figure found |
| Acrylic | unknown | No published figure found; load reported as unknown |

Limits, stated so nobody reads more into it:
- **Thin chips:** u grows as chips get thinner, so a fine finishing cut takes
  more energy per mm³ than the table says.
- **Mean force only:** the force is a mean over a revolution, not its peaks.
- **Not this machine:** the values come from handbooks and papers. One
  calibration cut on the real machine (spindle power against removal rate)
  replaces them.

## Talk to it (OmniLink)

`cnc_bridge.py` is an OmniLink bridge for any GRBL machine. It drives the
machine through GRBL over a stream (`grbl_link.py`), so the same bridge will
run a real board. The LongMill world starts it on `:8765` and zeroes the work
on the stock's top centre. Talk to it at `http://127.0.0.1:8765/chat` or
`POST /prompt {"text": ...}`. An OmniKey is required (`python -m omnisim key`);
without one, `/prompt` answers 401, while `/tool` and Stop still work.

- **Pipeline:** OmniLink's own pipeline, unchanged.
  - The package parser handles "stop", "pause", "resume" and "go home"
    without the model. Pause is a feed hold (position and job kept). Stop is a
    feed hold, then a reset at rest: the job is cancelled, the spindle is off
    and the position is kept. Both report the rest they measured.
  - Everything else, including status questions, goes to the model. Its tool
    calls pass the safety gate, so questions, negations and quotations never
    move the machine.
- **Tools:**
  - reading the machine: `get_robot_state`;
  - moving: `jog`, `move_to`, `set_work_zero`;
  - cutting: `cut_pocket` (circle, rectangle, or a star inside a circle,
    through `toolpath.py`) and `run_gcode`;
  - interrupting: `pause_job`, `resume_job`, `stop_robot`.
- **Machine checks, in the bridge,** each refusing with a named rule:
  - `preflight`: the job would error on the controller;
  - `spindle_off`: it cuts with the spindle stopped;
  - `travel`: against the carriages' real rails;
  - `cut_depth`: deeper than the stock minus 0.5 mm;
  - `plunge`: a rapid below Z0;
  - `busy`: a job is already running.
- **Units:** every argument is named in mm, because the gate's own rails are
  metric robot rails.

Live run, 2026-11-01, model reached through OmniLink, LongMill in the window:

| Operator said | What happened |
|---|---|
| "What's your status, and what tool do you have loaded?" | Answered from `get_robot_state`. |
| "Cut a circular pocket 50 mm wide and 2 mm deep, centred 45 mm to the left of the origin." | `cut_pocket` at X−45 Y0, milled. |
| "pause" … "What are you doing right now?" … "resume" | Feed hold at line 20 of 292, described accurately; resumed and finished. |
| "Now a rectangular pocket, 50 by 30 mm and 1.5 mm deep, centred 45 mm to the right." | Milled. |
| "Cut a 60 mm circle 15 mm deep in the middle." | Refused (`cut_depth`), and the model explained the 11.5 mm limit. |
| "Don't cut anything else. How much material have you removed so far?" | No motion; 6,201.3 mm³ (the design is about 6,164). |

## A product: the clothes rack

[`products/clothes_rack/`](../products/clothes_rack/README.md) takes a concept
sketch to a finished product with the twin. Its joint is designed for this
machine: an aluminium two-piece tube clamp, eight identical halves. Each half
is milled on
[`cnc_longmill_mk2_aluminium.omniworld`](../../samples/demos/worlds/manufacturing/cnc_longmill_mk2_aluminium.omniworld)
and measured against its CAD. The rack is then assembled step by step and
load-tested on Newton. Helical holes (`toolpath3d.bore`) and the material look
of the stock (`world.py STOCK_LOOK`) came from it.

## Not modelled

- Probing (G38.x)
- The safety door and parking
- Coolant and spindle pins
- Step generation and stepper resonance
- GRBL's 128-byte RX limit (input is never dropped)
- FluidNC-only `$` commands
- Rotary axes
- Machines whose kinematics are not three orthogonal linear axes
- Cutting forces acting on the machine: cutter and frame deflection, lost
  steps, chatter, tool runout and wear (the load is computed, see "Cutting
  load"; nothing pushes back yet)
- Tool changes on real hardware: they need probing (G38) for the tool length
- Backlash, tram (spindle not square to the bed) and spoilboard flatness
- Material: chips, burrs, tear-out and grain

The preflight time estimate plans the whole job at once, so it can be
optimistic on dense short segments.

## Next

In this order:

1. **Forces into the machine.** Lost steps when the cutting force passes what
   an axis can hold, and cutter deflection showing as dimensional error. The
   LongMill's stepper torque and screw lead are needed first; Sienci's spec page
   was not readable (HTTP 403), so they will come from a datasheet or a
   measurement.
2. **A real machine.** A USB serial transport for `grbl_link`, mirroring a
   real board from its status reports, probing (G38) so tool changes and work
   zero work on hardware, and a calibration loop: one test part cut, measured,
   and compared with the twin's prediction.
