# Clearpath Robotics packages — provenance

Covers `husky_description/` and `jackal_description/`.

This file exists because `scripts/release/publish_snapshot.sh` publishes a
**squashed single commit** — git history does not travel to the public
repository, so a provenance conclusion has to live in a file next to the
geometry it is about. Method and vocabulary: [`docs/developer/asset-provenance.md`](../../../docs/developer/asset-provenance.md).

---

## 1. The grant

| package | upstream | licensor | licence text |
|---|---|---|---|
| `husky_description/` | [husky/husky](https://github.com/husky/husky) | Clearpath Robotics Inc. | [`husky_description/LICENSE.upstream`](husky_description/LICENSE.upstream) |
| `jackal_description/` | [jackal/jackal](https://github.com/jackal/jackal) | Clearpath Robotics Inc. | [`jackal_description/LICENSE.upstream`](jackal_description/LICENSE.upstream) |

Both are BSD 3-Clause. Clause 1 requires the copyright notice, the conditions
and the disclaimer to travel with a source redistribution; the two
`LICENSE.upstream` files discharge that and **must not be removed** while the
packages are in use.

## 2. The test this pass applied

BSD-3 from Clearpath covers what **Clearpath** owns. A licensor can only
license what it owns, so for every binary geometry asset the question is not
"is the package licensed?" but:

> **Is the licensor the design owner of the shape in this file?**

Clearpath designs and builds the Husky and the Jackal, so that test passes for
the chassis, wheels, fenders, bumpers, top plates, risers and every
Clearpath-designed **mount or bracket** — including the ones that are *named
after* a third-party sensor, because a bracket that holds a SICK LMS1xx is
Clearpath's design, not SICK's.

It does **not** pass for a model of another manufacturer's product that
happens to sit inside a Clearpath package. Three such files were found and
removed (§3).

## 3. Removed, 2026-08-24 — three Hokuyo lidar models

| path | bytes | STL header | bbox (m) |
|---|---:|---|---|
| `husky_description/meshes/accessories/hokuyo_ust10.stl` | 105,884 | `Exported from Blender-2.78 (sub 0)` | 0.0643 × 0.0500 × 0.0700 |
| `jackal_description/meshes/hokuyo_ust10.stl` | 105,884 | `Exported from Blender-2.78 (sub 0)` | 0.0643 × 0.0500 × 0.0700 |
| `jackal_description/meshes/hokuyo_utm30.stl` | 442,284 | **`solid utm-30lx`** | 0.0800 × 0.0600 × 0.0870 |

**Why they went.** All three depict scanning lidars designed and manufactured
by **Hokuyo Automatic Co., Ltd.** — a company that is not Clearpath and has
granted nothing here. The `utm30` file names the product in its own STL header
(`solid utm-30lx`), and the bounding boxes match Hokuyo's published enclosure
dimensions (UST-10LX 50 × 50 × 70 mm; UTM-30LX 60 × 60 × 87 mm) to within the
connector boss. This is the same defect that removed Boston Dynamics' Spot CAD
from `boston_dynamics/spot`, and Orbbec's Astra, Intel's RealSense R200 and
Hitachi-LG's LDS-01 from `robotis/turtlebot3_description`: a sound BSD-3 grant from a vendor who is not
the design owner of the part.

**Proof they were unreferenced** (whole tracked tree plus untracked
non-ignored files):

* `git grep -i hokuyo` — the only live hits are
  `projects/devices/hokuyo/urdf/*.urdf.xacro` and `projects/devices/README.md`,
  which are OmniSim's **own** primitive-geometry device macros
  (see [`projects/devices/PROVENANCE.md`](../../devices/PROVENANCE.md)) and
  reference no mesh file at all.
* Basename greps for `hokuyo_ust10.stl` / `hokuyo_utm30.stl` — **zero hits**
  anywhere, including worlds, PROTOs, controllers, scripts, docs and the
  `scripts/packaging/*.txt` manifests.
* The code that would consume them is the URDF importer, and it only ever sees
  a mesh through a `filename="package://…"` attribute. `husky.urdf` references
  7 meshes and `jackal.urdf` references 3; none is one of these.

**Verification after deletion.** Every `filename="package://…"` in both
packages, and in the TurtleBot3 package cleaned in the same pass, was
re-resolved against the files on disk: **27 references, 0 dangling.**

The two `hokuyo_ust10.stl` copies were byte-identical
(md5 `9bca7367193dc8f61bed328e2222575a`).

## 4. Kept, deliberately — the mounts named after other vendors

`velodyne_tower.stl` and `kinect_mount.stl` are equally unreferenced, and an
earlier audit listed them alongside the Hokuyos as orphans. **They were kept.**

| file | STL header | bbox | what it is |
|---|---|---|---|
| `jackal_description/meshes/velodyne_tower.stl` | `solid velodyne_tower` | 0.1046 × 0.1411 × 0.0814 m | the **tower** that raises a Velodyne above the Jackal — not a Velodyne |
| `jackal_description/meshes/kinect_mount.stl` | `MESHMIXER-STL-BINARY-FORMAT…` | 135 × 135 × 24.7 mm | the **mount plate** a Kinect bolts to — not a Kinect |
| `jackal_description/meshes/sick-lms1xx-upright-bracket.stl` | `solid lidarbracket` | 0.135 × 0.138 × 0.127 m | bracket |
| `jackal_description/meshes/sick-lms1xx-inverted-bracket.stl` | *(Meshmixer colour header)* | 0.100 × 0.108 × 0.118 m | bracket |
| `jackal_description/meshes/camera-bracket.stl` | **`solid Jackal Camera Brackets.SLDASM`** | 163 × 16 × 180 mm | bracket — the header names Clearpath's own SolidWorks assembly |
| `husky_description/meshes/accessories/lms1xx_mount.stl` | `solid lms1xx_mount` | 0.100 × 0.108 × 0.118 m | mount |
| `husky_description/meshes/accessories/lidar_mount.stl` | `Exported from blender` | 0.105 × 0.240 × 0.147 m | mount |

**Reasoning, stated because the decision could reasonably have gone the other
way.** The design-owner test *passes* for all of these — a mount is
Clearpath's design however it is named, and `camera-bracket.stl` says so in
its own header. That leaves orphan-hood as the only argument for deleting
them, and orphan-hood here is a **partial-import artefact, not abandonment**:
`jackal.urdf:361` still carries

```xml
<!-- <xacro:include filename="$(find jackal_description)/urdf/accessories.urdf.xacro" /> -->
```

commented out, and that xacro — the file that would consume every one of these
meshes — was simply never imported. Removing the assets of a dormant
subsystem buys nothing legally and makes the package harder to complete later.

This pass is a **licence** pass. It removes what cannot be licensed; it does
not tidy. Deleting these would be housekeeping wearing a licence pass's
clothes, and this repository has already recorded once (`simple_skin.fbx`,
`docs/developer/asset-provenance.md` §1) that the two should not be confused.

## 5. Unresolved — the NovAtel GNSS antennas

Found during this pass, **not** acted on, and flagged rather than papered over:

| file | bytes | tris | STL header | bbox (m) |
|---|---:|---:|---|---|
| `jackal_description/meshes/novatel-smart6.stl` | 11,284 | 224 | `solid basic_smart6` | 0.1545 × 0.1550 × 0.0800 |
| `jackal_description/meshes/novatel-smart7.stl` | 63,084 | 1,260 | `solid smart7_simple` | 0.1933 × 0.0600 × 0.2177 |

**For treating them like the Hokuyos:** SMART6 and SMART7 are GNSS antennas
designed and sold by **NovAtel Inc.** (Hexagon), not by Clearpath, and the
filenames say so.

**Against:** unlike `utm-30lx`, neither is plausibly a vendor CAD import. At
**224** and **1,260** triangles they are crude blocks, and their own headers
call them `basic_` and `_simple` — i.e. Clearpath-authored stand-ins for a
sensor envelope, which is expression Clearpath does own. A dimension is a
fact, not a design.

They are unreferenced, so removing them costs nothing functionally. The call
belongs to whoever owns the register, not to this pass, and no licence is
asserted for them here either way.

`ark_enclosure.stl` (`Exported from Blender-2.78`, 0.405 × 0.310 × 0.068 m)
was looked at for the same reason and raised no question: "ARK" is a Clearpath
enclosure, and nothing in the file suggests otherwise.

## 6. What is used and what is dormant

| package | mesh files | referenced by its URDF |
|---|---:|---:|
| `husky_description/` | 32 | 7 — `base_link.dae`, `bumper.dae`, `top_chassis.dae`, `top_plate.dae`, `top_plate.stl`, `user_rail.dae`, `wheel.dae` |
| `jackal_description/` | 18 | 3 — `jackal-base.stl`, `jackal-fender.stl`, `jackal-wheel.stl` |

Three of the husky meshes are additionally referenced by
`projects/default/controllers/harness_supervisor/damage_profiles.py`
(`wheel.dae`, `bumper.dae`, `top_plate.dae`).

## 7. Adding a mesh to either package

Ask the question in §2 before you add it. If the shape is another
manufacturer's product, Clearpath's BSD-3 does not reach it, and neither does
anybody else's — author a primitive instead, as
[`projects/devices/`](../../devices/PROVENANCE.md) and
[`projects/robots/omnisim/`](../omnisim/PROVENANCE.md) do.
