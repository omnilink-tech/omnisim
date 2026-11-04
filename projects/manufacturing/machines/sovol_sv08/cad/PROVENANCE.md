# Sovol SV08 mesh pack: provenance

The meshes in `meshes/` and `pack.json` are generated from Sovol's CAD of the SV08:

- **Source:** [github.com/Sovol3d/SV08, `STEP/`](https://github.com/Sovol3d/SV08/tree/main/STEP),
  the pre-assembled frame (`0JXHSV08-00000 … Frame 3d printer pre-assembled.zip`, 37.6 MB,
  one 154 MB STEP). Downloaded 2026-11-02.
- **Licence:** Sovol publishes the SV08 under **GPL-3.0** (the repository's `LICENSE`, copied
  here as [`LICENSE`](LICENSE)). `meshes/` and `pack.json` are derived from that CAD and are
  distributed under GPL-3.0, **not** OmniSim's Apache-2.0. They are data files kept apart in
  this folder; nothing else in OmniSim is derived from them.
- **What was changed:**
  - the assembly is cut into the four bodies the twin moves;
  - fasteners under 9 mm across are left out;
  - the parts sealed inside the base (the control board, power supply and wiring, 36
    parts) are left out;
  - the two flexible parts that follow the toolhead (the umbilical and the filament
    path) are left out;
  - the toolhead and X beam are tessellated at 0.6 mm / 0.5 rad, the frame and gantry
    coarser at 1.0 mm / 1.0 rad.

  The twin's world draws the CAD's placeholder colours (SolidWorks defaults) as dark
  plastic. Those colours are ours, not Sovol's.

`manifest.json` and `make_manifest.py` are OmniSim's own (Apache-2.0).

## The bodies

The names in Sovol's assembly are anonymous (`NAUO1`…), so `make_manifest.py` cuts the
bodies by where the parts are:

| Body | Moves along | What it is |
|---|---|---|
| frame | – | base, the four Z columns, top, heated bed, spool holder, screen, camera |
| gantry | Z | the AB (XY) gantry: corner motors, side rails, belts |
| beam | Y (on the gantry) | the X beam: extrusion, rail, end blocks, idlers, Y carriage blocks |
| toolhead | X (on the beam) | the hotend assembly; its lowest point is the nozzle |

The CAD pose is measured from the CAD: the nozzle against the PEI sheet under it puts
the drawing at machine X 156.4, Y 92.3, Z −2.0 mm. The CAD has the gantry at its
lowest, a little below Z0, and the SV08's Z travel starts at −5 mm. **Assumed:** the
350 mm printable area is centred on the sheet (Sovol publishes no bed origin).

## Rebuilding

```bash
# 1. Download and unzip the pre-assembled STEP into .tmp/sv08/cad/ as frame.step.
# 2. The manifest, from the assembly (the tessellation is cached in the .glb).
python projects/manufacturing/machines/sovol_sv08/cad/make_manifest.py .tmp/sv08/cad/frame.step --glb .tmp/sv08/cad/frame.glb
# 3. The pack (about 3 minutes; the CAD is tessellated twice; 0.79 M triangles, 39 MB of STL).
python projects/manufacturing/core/cad_pack.py projects/manufacturing/machines/sovol_sv08/cad/manifest.json \
    --cad-dir .tmp/sv08/cad --out projects/manufacturing/machines/sovol_sv08/cad
# 4. The world.
python projects/manufacturing/printing/world.py --gcode projects/manufacturing/printing/jobs/cube20_sv08.gcode \
    --pack projects/manufacturing/machines/sovol_sv08/cad \
    --out projects/samples/demos/worlds/manufacturing/sv08_printer_twin_cad.omniworld
```

The pack is over `cad_pack.py`'s 400 k-triangle budget. Options tried on 2026-11-02:

- **The whole assembly at 0.6 mm** was 1.37 M triangles. It held a laptop RTX 3060's 3D
  engine at 38–68 % while printing (53 W, 85 °C).
- **Decimation** was dropped. It tore the frame into shards, and even with the vertices
  welded first it faceted the toolhead visibly.
- **What is used:** the hidden base interior left out (145 k triangles, mostly the
  board), and the frame and gantry tessellated coarser. Side by side with the 0.6 mm pack,
  the same views showed the same detail.
