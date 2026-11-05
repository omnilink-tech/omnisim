# Provenance: docs/images/manufacturing/

Two kinds of picture live here. Keep them apart in every caption.

## Concept art (`concept/`)

AI-generated concept art, made for OmniLink with an AI image tool on 2026-11-03. It is the
starting point of the SparkRover Mini design, not a picture of anything that exists or was
simulated. The design then corrected the sheets' numbers and features; `skeleton.py`
`DECISIONS` records what changed (among them: no gripper, a normal camera instead of a depth
camera, a 2S2P battery).

| File | What it is |
|---|---|
| `concept/edubuddy_1.png` … `edubuddy_4.png` | the first concept, "EduBuddy", with grippers and a depth camera |
| `concept/sparkrover_1.png` … `sparkrover_3.png` | the simplified SparkRover Mini concept sheets |

The copies are resized to at most 1600 px wide (the originals are 1448 px, so they are
unchanged in size) and saved as PNG. Nothing in them was edited.

## Simulator renders (every other PNG)

Rendered by OmniSim's wgpu renderer through a Camera device, in headless engines
(`OMNISIM_NO_WINDOW=1`), by the scripts in
[`projects/manufacturing/showcase/`](../../../projects/manufacturing/showcase/README.md), which
list each image's world, camera and job progress. Each frame is rendered at twice its
published size and downsampled. Nothing is painted onto a render afterwards. The captions and
the two-panel layout of `concept_vs_built.png` and `assembly_steps.png` are added by those
scripts, and the left panel of `concept_vs_built.png` is a crop of `concept/sparkrover_1.png`.

**Simulation only.** No part shown was made on a real machine, and no machine twin is
validated against its real machine. What each render shows:

| File | Source |
|---|---|
| `sparkrover_hero.png`, `face_camera.png` | the drive-test robot (`robot.py`), posed by `spark_rover_pose`; the face is drawn on its Display device and the inset is its Camera device's live frame |
| `concept_vs_built.png` | `concept/sparkrover_1.png` beside the same robot at that sheet's angle |
| `twin_laser.png` | the LongMill laser twin part-way through the deck's G-code (`make.py`'s job) |
| `twin_cnc.png` | the LongMill router twin part-way through the shoulder brackets' program |
| `twin_printer.png` | the SV08 printer twin part-way through a shell section's G-code |
| `machine_cnc.png`, `machine_laser.png`, `machine_printer.png` | the same three twins on the same jobs, from far enough back to show the whole machine at work: the router twin cutting the shoulder brackets, the laser twin cutting the deck, the printer twin at the same instant as `twin_printer.png` |
| `printed_parts.png` | the printed parts' **models** in their print orientation, not the twin's beads |
| `exploded.png`, `wiring.png`, `assembly_steps.png` | the design's own solids and `harness.py`'s routed wires, as static scenery |
| `pipeline.svg` | a hand-written diagram in the OmniSim palette |

Some rendering choices differ from the worlds the product's scripts write. They change the
look only, never a part, a pose, a job or a result:

- the clear acrylic parts are drawn at transparency 0.92 (cast acrylic passes about 92 %)
  instead of 0.75;
- the scenes add an unshadowed fill light beside the canonical `OmniSimSky` /
  `OmniSimSun` / `OmniSimSunMarker` trio, and place the sun marker;
- `sparkrover_hero.png` converts the design's sRGB colours to the linear values the
  renderer reads (`robot.py` writes the hex values straight into `baseColor`, so every part
  renders lighter than its design colour), and draws the status ring's two diffusers glowing
  teal. The LEDs behind them are visual meshes in the design, not simulated devices;
- `twin_laser.png` and `twin_cnc.png` draw the LongMill's wasteboard dark, and
  `twin_cnc.png` draws the aluminium blank's skin lighter and glossier.

The renders and the diagram are OmniLink's. The machine geometry the twin renders show keeps
its own licence: the LongMill's is Sienci Labs' CAD (CC BY-SA) and the SV08's is Sovol's CAD
(GPL-3.0), as their packs' `PROVENANCE.md` files in `projects/manufacturing/machines/` record.
