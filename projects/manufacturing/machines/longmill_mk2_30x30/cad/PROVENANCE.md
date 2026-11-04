# Provenance: LongMill MK2 30x30 meshes

**These files are NOT under the repository's Apache-2.0 licence.** They are
derived from Sienci Labs' CAD and keep its licence.

| | |
|---|---|
| Design | LongMill MK2 30x30 benchtop CNC, by Sienci Labs |
| Source | Public Onshape document "LongMill Benchtop CNC MK2", assembly "LongMill MK2 30x30": https://cad.onshape.com/documents/59335d21bb649cd2b941a693/w/290c65d1d584435649b87e46/e/99a028de0f32d790196bbe8a |
| Exported | 2026-10-31, STEP AP242, by OmniLink from the public document (export permitted by the document's sharing settings) |
| Licence | Creative Commons Attribution-ShareAlike (CC BY-SA), as stated by Sienci Labs for the LongMill's mechanical design files. **The version (4.0 is likely) is not yet confirmed** against Sienci's own page; confirm it before these files are published. |
| Attribution | "LongMill MK2 30x30 by Sienci Labs, CC BY-SA. Meshes derived by OmniLink." |
| Share-alike | Anything derived from these meshes must carry the same licence. |

## Files covered

`meshes/*.stl` and `pack.json` (generated). `manifest.json` (how they are cut)
is OmniLink's own work, under the repository licence.

## Changes from the original CAD

Made by `projects/manufacturing/core/cad_pack.py` with `manifest.json`:

- Tessellated at 1.0 mm linear / 0.5 rad angular tolerance; converted to
  metres, Z-up; shifted so the MDF base sits on z = 0, centred in x/y.
- Grouped into frame / gantry / carriage / z by assembly path, and merged by
  CAD colour into meshes of at most 21,000 triangles.
- The four ACME lead screws are replaced by their bounding cylinders: their
  threads are ~57,000 triangles each.
- Removed: wood screws, and the electronics inside the control box (Arduino,
  drivers, terminal blocks, power supply).

The STEP file itself is not in the repository (it lives in `.tmp/cnc-cad/`).
Regenerate with:

```bash
python projects/manufacturing/core/cad_pack.py projects/manufacturing/machines/longmill_mk2_30x30/cad/manifest.json \
    --cad-dir .tmp/cnc-cad --out projects/manufacturing/machines/longmill_mk2_30x30/cad
```
