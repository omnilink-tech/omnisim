# 140 mm parallel gripper — provenance

**Original work of this repository**, Apache-2.0 (© OmniLink), like the rest of
the tree. This file exists because `publish_snapshot.sh` publishes a squashed
single commit, so provenance must live beside the geometry rather than in git
history.

## Geometry

`2f140/urdf/robotiq_2f140.urdf` is defined **entirely by URDF primitive
solids** — boxes and cylinders declared inline. There are no binary geometry
assets in this package: no `.stl`, `.dae`, `.obj` or texture files. The shells
are emitted by
[`scripts/dev/gen_omnisim_robot_visuals.py`](../../../scripts/dev/gen_omnisim_robot_visuals.py)
(`--robot gripper140`) and re-checkable with `--check`.

**No mesh, texture or CAD file from any third party is used, and none of this
geometry is imported from, tessellated from, or derived from any manufacturer's
product model.**

## Naming

"Robotiq" and "2F-140" are trademarks of Robotiq Inc. They appear here
**nominatively** — to identify the class of hardware this model represents and
to keep the identifiers stable for worlds, bridges and gripper configs that
already reference them. This package is not affiliated with, sponsored by, or
endorsed by Robotiq, and it is not a Robotiq release.

### User-facing prose — corrected 2026-09-11

Nominative use covers identifiers. It does **not** cover a catalogue entry that
hands our own work a manufacturer's product name as its title. `DEMOS.md`, the
launcher catalogue and three world titles read "Robotiq 2F-140" as the name of
the thing on screen; they now read **"140 mm two-finger gripper"**. The
`2f140` in paths, DEF names, link names, device names and `--gripper` ids is
unchanged — it is a size/class identifier, and renaming it is a separate
decision that would touch worlds, configs and tests (see the rename note the
2026-09-11 doc-accuracy pass left for the owner).

Corrected in the same pass: two flagship worlds and their two controllers still
claimed every `<visual>` was "the customer's own CAD
(3d_models/2F-140_Assy_Open_20191022.STEP, tessellated by
scripts/dev/step_to_urdf.py)". That contradicted this file outright and claimed
third-party CAD lineage for geometry that is entirely ours. The arm-mounted
URDF had already been corrected on 2026-08-24; the four copies downstream of it
were missed.

## Dimensions

Mounting interface, ~144 mm coupling-face-to-pivot height, and the fingers'
prismatic travel (−0.030 … 0.070 m along ±Y) describe the 2F-140 class and are
carried over unchanged from the model this package replaces, so existing
worlds, gripper configs and demos remain valid. The jaw **collision** boxes on
the arm-mounted variant were hand-authored in this repository from the start
and are unchanged.
