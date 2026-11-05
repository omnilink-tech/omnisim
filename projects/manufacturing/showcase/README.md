# Showcase stills: how docs/images/manufacturing/ is made

Every simulator render in [`docs/images/manufacturing/`](../../../docs/images/manufacturing/)
comes from a script here, and each one runs again with one command. What the images are,
and what they are not, is in that folder's [PROVENANCE.md](../../../docs/images/manufacturing/PROVENANCE.md).

**Simulation only.** No part shown was made on a real machine, and no twin is validated
against its real machine.

## How a still is taken

- **The engine.** Each run is one headless `omnisim-bin` started the way the repo's render
  tests start one: `OMNISIM_NO_WINDOW=1` (no window, so nothing appears on the desktop),
  `--batch --mode=fast`, its own `OMNISIM_LOG_PATH`. Before a launch it waits while six or
  more engines run on the machine or the GPU is at 75 C or above (`showcase_lib.wait_for_room`).
- **The world.** The scene is written to `worlds/.gen_<name>.omniworld` (hidden, gitignored,
  deleted after the run), with every relative URL re-pointed (`showcase_lib.read_world`).
  `OMNISIM_EXTRA_PROJECT_PATH` lets it find this folder's controllers and the demo
  controllers. Scenes follow the canonical lighting recipe (`OmniSimSky`, `DEF SUN
  OmniSimSun`, `DEF SUN_MARKER OmniSimSunMarker`), plus one unshadowed fill light. Every
  PROTO is local.
- **The photographer.** [`controllers/showcase_camera`](controllers/showcase_camera/showcase_camera.py)
  is a kinematic supervisor carrying one Camera. For each shot it waits for a simulation
  time (`at_s`) and, for the twins, a trigger file. It then moves itself to the shot's
  position, aims at its target (+x forward, +z up, no roll), and saves the frame. Its Camera
  is enabled only around a shot: a 3200 x 2000 camera rendering every step ran the laser
  twin at 0.17x real time, against 2.4x with it off between shots. Its `far` plane also sizes the camera's shadow map (OmCamera.cpp fits the
  map to `0.75 far`), so close shots use `far` 2.5-3.
- **The frame.** The frame is rendered at 3200 x 2000 and downsampled to 1600 x 1000 (Lanczos).
  Nothing is painted onto it.

Raw frames, logs and alternate angles go to `.tmp_showcase/` (gitignored). `--review` sends
every frame there and publishes nothing.

## The images

| Image | Command | World and subject | Camera (m) |
|---|---|---|---|
| `sparkrover_hero.png` | `make_images.py hero` | The SparkRover as `robot.py` writes it for `spark_rover_drive_test.omniworld`, on the oak desk before a dark warm-grey backdrop, with three look-only changes: the design's sRGB colours converted to the linear values the renderer reads (`design_colours`: `robot.py` writes the hex values straight in, so the #5A5F66 lower shell and TPU bumpers rendered light grey), the two diffusers drawn emissive teal (`lit_diffusers`: the status ring's LEDs are visual meshes in the design, not simulated devices), and clear acrylic at 0.92. Warm key from the front left, 35° up (`OmniSimSun` colour 1 0.86 0.70, intensity 3.2), a weak cool fill from the front (0.35), so the robot shades its right side and the desk. Right arm at shoulder 0.9, elbow 1.0 rad on its servos (the shoulder stops at horizontal, 1.571 rad, so the concept's raised-hand wave is out of its range), face on the Display, shot at t = 2.5 s. Camera `far` 2.6 m. | (0.74, -0.46, 0.17) at (0, 0, 0.136), fov 0.62, the robot's chest height |
| `face_camera.png` | same run | The screen close up. The face is drawn on the 800 x 480 Display by the controller (the drive controller's `Face`, in the accent colours, without the phase word). The inset is the head Camera's live frame at 1/4 scale, looking at a target board 1.3 m ahead. Shot at t = 2.7 s. | (0.34, -0.15, 0.245) at the display's centre |
| `concept_vs_built.png` | `make_images.py concept` | Left: `concept/sparkrover_1.png` cropped to its hero and callouts. Right: the same robot, both forearms forward (pose `concept`), from the sheet's angle (front right, about 15° above the face). | (0.80, -0.43, 0.40) at (0, -0.005, 0.135), fov 0.37 |
| `twin_laser.png` | `twins.py laser --at 0.97` | `make.py`'s world for `deck_upper` (LongMill MK2 twin, Sienci LaserBeam 7 W, 3 mm plywood), streamed `make.py`'s program over GRBL, shot when the twin had acknowledged 97 % of the lines, the job still running, so the passes have cut the holes and the outline through. `twins.look` changes the look only: the pack's wasteboard (`frame_24`) drawn dark so the plywood and the cut-through kerfs read against it, and the sun marker moved from overhead (0 0 8) to the camera's side. The twin's own laser glow marker is drawn as it is. Camera `far` 2 m. | `views/laser.json` `twin_laser`, from the sheet's centre |
| `twin_cnc.png` | `twins.py cnc --at 0.5` | `make.py`'s world for `shoulder_brackets` (the LongMill router twin, 8 mm 6061 blank, both brackets from one blank), shot at 50 % of the lines, cutting. The same `twins.look` changes, plus the blank's skin and core drawn lighter and glossier (skin .60 .63 .68 and machined core .84 .86 .89 at roughness .2, from .58 .60 .63, .86 .87 .89 at .35), so the pockets read as machined. Camera `far` 2 m. | `views/cnc.json` `twin_cnc` |
| `twin_printer.png` | `twins.py printer` | `printing/world.py` with the SV08's CAD pack (the full-physics twin, not the ideal mode), printing `shell_lower_front` alone (`release/models`, sliced by `cura_slice.py`), its G-code cut after 209 of its 522 layers (40 %). Shot at 2332 s of machine time, 97 % of the 2404 s Klipper plans to the cut, with the nozzle laying plastic. The beads are PrintedPart's default colour, not the PETG's. The published still leaves out the right fifth of the frame, where the printer's front upright stands in front of the bed. | `views/printer.json` `twin_printer`, from the part's centre on the bed |
| `machine_printer.png` | same run as `twin_printer.png` | The same printer, part and instant (2332 s of machine time), from far enough back to show the SV08's gantry and frame. The front upright stays in this frame. | `views/printer.json` `wide` |
| `machine_cnc.png` | `twins.py cnc --at 0.5` (it shoots every view in `views/cnc.json`) | The router twin on `shoulder_brackets` at 50 % of the lines, cutting, from far enough back to show the LongMill's gantry and router. A separate run from `twin_cnc.png`, on the same job and look. | `views/cnc.json` `machine` |
| `machine_laser.png` | `twins.py laser --at 0.97` (it shoots every view in `views/laser.json`) | The laser twin on `deck_upper` at 97 % of the lines, from far enough back to show the LongMill's gantry and the laser module. A separate run from `twin_laser.png`, on the same job and look. | `views/laser.json` `machine` |
| `printed_parts.png` | `make_images.py printed` | The 27 printed parts' **models** (`release/models`, in their print orientation, the superseded carrier drawings left out), laid flat on the desk in the design's colours. It is not the twin's beads: PRINT.md has what the twin laid, in which mode. | (1.32, -0.24, 1.02) at (0.06, 0, 0) |
| `exploded.png` | `make_images.py exploded` | The design's solids (`robot.part_table`) by subsystem. The chassis stays; the shell is lifted 0.25 m, the electrical subsystem moved 0.30 m forward, each arm 0.18 m out. | (1.30, -1.05, 0.80) at (0.10, 0, 0.24) |
| `wiring.png` | `make_images.py wiring` | The 127 routed wires (`meshes/wires/W*.stl`, in `wires_routed.json`'s colours) with the chassis, electrical and arm parts, the shell left out. Seen from the rear right, into the bay carrier. | (-0.58, -0.50, 0.35) at (0, 0, 0.145) |
| `assembly_steps.png` | `make_images.py assembly` | Steps 1, 3, 4, 6, 8, 10, 14 and 18 of `assemble.py`'s 18 (`release/assembly/assembly_log.json`), cumulative, from one camera. A part no step names (screws, nuts) appears with the nearest part that one does. | (0.56, -0.47, 0.40) at (0, 0, 0.125) |

`pipeline.svg` is written by hand (the OmniSim palette, `resources/branding/omnisim/BRAND.md`;
it switches to its dark colours under `prefers-color-scheme: dark`).

## Files

| File | What |
|---|---|
| `make_images.py` | the robot, concept, exploded, wiring, assembly and printed-parts stills |
| `twins.py` | the three machine-twin stills: the job's world, the job streamed (laser, CNC) or cut (printer), the trigger |
| `parts.py` | one STL per design part and the wire meshes, and the static scenery built from them |
| `showcase_lib.py` | world rewriting, the photographer node, the engine launch, the machine's limits, publishing |
| `controllers/showcase_camera/` | the photographer |
| `controllers/spark_rover_pose/` | the robot's pose and face for the stills |
| `views/*.json` | the twins' camera views, `(name, dx, dy, dz, tx, ty, tz)` from the stock's (or part's) centre; the `twin_*` view is the published one |

## Known limits of these renders

- **The camera's shadows are coarse and lift off small parts.** A Camera device fits one 2048²
  shadow map to `max(2, 0.75 far)` m, so a `far` under 2.7 m buys nothing more; the map's depth
  runs `halfExtent × 6 + 20` m and its bias is 0.0006 of that (OmCamera.cpp), about 2 cm, so a
  shadow starts about 2 cm from what casts it and anything that small casts little. The stills
  place the key light so the shadows that show are the large ones. The main view's cascades are
  not available headless.
- **No path-traced still.** `OMNISIM_PHOTO_OUTPUT` (OmView3D.cpp) renders from the main view's
  frame loop, which needs the GUI's 3D view: `OMNISIM_NO_WINDOW=1` builds no view at all, and
  any other headless route opens a window. It was not used.
- **The screen is lit, not glowing.** The wgpu renderer has no emissive texture, so the
  face takes the scene's light (the SparkRover README says the same).
- **The arms are the design's.** Each forearm is a round servo housing, not the concept's
  tube; `concept_vs_built.png` shows the difference.
