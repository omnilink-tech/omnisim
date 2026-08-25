# TurtleBot3 (`turtlebot3_description`) — provenance

This file exists because `scripts/release/publish_snapshot.sh` publishes a
**squashed single commit** — git history does not travel to the public
repository, so a provenance conclusion has to live in a file next to the
geometry it is about. Method and vocabulary:
[`docs/developer/asset-provenance.md`](../../../../docs/developer/asset-provenance.md).

---

## 1. The grant

Upstream is [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3),
package `turtlebot3_description` **v2.3.6** (`package.xml`), Apache License 2.0.
The licence text is retained at [`LICENSE.upstream`](LICENSE.upstream).

⚠️ **`LICENSE.upstream` is the unfilled Apache-2.0 appendix boilerplate** — its
closing block still reads `Copyright [yyyy] [name of copyright owner]`, so the
file names no holder and no year. `package.xml` declares
`<license>Apache 2.0</license>` and names ROBOTIS people as the authors and
maintainer (`thlim@robotis.com`, `pyo@robotis.com`, `willson@robotis.com`),
which is where the holder is actually established. No holder or year is
invented here. This is the same shape as the Universal Robots entry recorded in
`THIRD_PARTY_NOTICES.md`.

## 2. The test this pass applied

Apache-2.0 from ROBOTIS covers what **ROBOTIS** owns. A licensor can only
license what it owns, so for every binary geometry asset the question is not
"is the package licensed?" but:

> **Is the licensor the design owner of the shape in this file?**

ROBOTIS unambiguously designs, manufactures and sells the TurtleBot3, so the
test passes for the chassis plates and the tyres. It does not pass for a model
of a **different** company's product that ROBOTIS bolted onto its robot and
shipped in the same package — and this package turned out to contain two of
those, not one.

## 3. Removed, 2026-08-24

| path | bytes | what it was |
|---|---:|---|
| `meshes/sensors/r200.dae` | 208,224 | model of an **Intel RealSense R200** depth camera (§3.1) |
| `meshes/sensors/lds.stl` | 728,384 | model of the **LDS-01 / HLS-LFCD2** lidar, a **Hitachi-LG Data Storage** product (§3.2) |
| `meshes/sensors/r200.jpg` | 2,508 | the R200's 256 × 256 texture — **unreferenced, including by the `.dae` itself** |
| `meshes/sensors/astra.jpg` | 65,411 | texture of the **Orbbec Astra** whose `.dae` was already deleted (§3.4) |

The `meshes/sensors/` directory is now empty and gone. Both meshes were
replaced by primitives authored here (§3.3); no link, joint, collider or
inertial changed.

### 3.1 The Intel RealSense R200 (`r200.dae`)

Read out of the file's own bytes:

```xml
<asset>
    <contributor>
        <authoring_tool>Google SketchUp 8.0.4811</authoring_tool>
    </contributor>
    <created>2015-05-31T08:41:34Z</created>
    <modified>2015-05-31T08:41:34Z</modified>
    <unit meter="0.002539999969303608" name="inch" /><!-- Changed! -->
    <up_axis>Z_UP</up_axis>
</asset>
```

* **No `<author>`, no `<copyright>`, no `<comments>`, no `<source_data>`.** The
  only contributor element in the whole 208 KB file is the authoring tool.
* **Authored in Google SketchUp 8** and **dimensioned in inches** — while every
  other mesh in this package is a binary STL in millimetres. Nothing about it
  belongs to the same CAD project as the robot.
* **Its bounding box is the R200's published enclosure**: 0.1300 × 0.0200 ×
  0.0078 m over 1,597 vertices, against Intel's stated 130 × 20 × 7 mm. It is a
  model of a specific Intel product, not a generic sensor block.
* ROBOTIS' own `CHANGELOG.rst` (v0.1.3, 2017-04-24) names it as such:
  `* added Intel RealSense R200`.

The R200 is an Intel product. ROBOTIS' Apache-2.0 grant is sound for what
ROBOTIS authored and cannot reach Intel's industrial design.

### 3.2 The LDS-01 lidar (`lds.stl`) — the harder call, and how it was decided

This one was investigated at length and the first draft of this file **kept**
it. That draft was wrong, and the reason it was wrong is worth recording.

**The suspicious signal was a red herring.** `lds.stl`'s 80-byte STL header
reads `dilos LDS`, and DILOS is not ROBOTIS. But the package's six meshes all
use one header convention, `<authoring identity> <PART>`:

| file | header | tris | bbox (mm) |
|---|---|---:|---|
| `bases/burger_base.stl` | **`dilos`** `PR30_TB3_BASIC` | 96,524 | 137.55 × 148.35 × 172.59 |
| `bases/waffle_base.stl` | **`dilos`** `PR30_TB3_PREMIUM` | 325,838 | 270.57 × 278.80 × 123.52 |
| `bases/waffle_pi_base.stl` | **`robot`** `PR30_TB3_WAFFLE_PI` | 157,576 | 274.05 × 278.80 × 118.92 |
| `wheels/left_tire.stl` | **`robot`** `PR30_ISW_TIRE_L` | 21,672 | 66.00 × 18.20 × 66.00 |
| `wheels/right_tire.stl` | **`robot`** `PR30_ISW_TIRE_R` | 21,672 | 66.00 × 18.20 × 66.00 |
| ~~`sensors/lds.stl`~~ | **`dilos`** `LDS` | 14,566 | 93.85 × 68.88 × 39.20 |

The `PR30_` part-number family spans *both* identities, and `dilos` sits on
`PR30_TB3_BASIC` and `PR30_TB3_PREMIUM` — the Burger and Waffle **chassis**,
ROBOTIS' own product. A third-party design owner cannot plausibly be stamped on
ROBOTIS' own chassis, so the string is an authoring identity (a CAD seat), not
a vendor attribution. That was checked externally too: `dilos.co.kr` does not
resolve in DNS, and the only Korean entity of that name that could be found is
a real-estate development SME with no robotics or CAD business. **A
third-party authoring tool or CAD seat is not a third-party design owner**, and
if that had been the only question the file would have stayed.

**What actually decides it is the product, and ROBOTIS answers it against
itself.** ROBOTIS' own e-manual page for the part
([`appendix_lds_01`](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/))
says:

> "ROS Hector SLAM demo using only a 360 Laser Distance Sensor LDS-01 made by
> **HLDS (Hitachi-LG Data Storage)**."

> "The `hls_lfcd_lds_driver` package provides a driver for **HLS(Hitachi-LG
> Sensor) LFCD LDS(Laser Distance Sensor)**."

and the specification sheet ROBOTIS hosts for it
(`assets/docs/LDS_Basic_Specification.pdf`) is **HLDS' own drawing-control
document**, its title block reading

```
MODEL : HLS-LFCD2   LDS 1.5   SPECIFICATIONS   17-02-14   ID NUMBER   PREPARATION
```

with a Korean approval block and no ROBOTIS branding. The LDS-01 is a
**rebadged HLDS HLS-LFCD2**: ROBOTIS sells it under its own name and SKU and
credits HLDS explicitly and repeatedly. (For contrast, the successor LDS-02
page names no external manufacturer at all, so the LDS-01 attribution is
deliberate rather than boilerplate.) Our mesh's bounding box, 93.85 × 68.88 ×
39.20 mm, matches HLDS' published enclosure of 69.5(W) × 95.5(D) × 39.5(H) mm.

So the answer to §2's question is **no**: ROBOTIS is not the design owner of
the shape in this file, and ROBOTIS says so itself. That is the same structure
as the R200 — a third-party sensor module bolted onto ROBOTIS' robot — and
"we resell it under our own brand" is precisely the reasoning §2 exists to
refuse. It went.

**Stated plainly, because it could have gone the other way.** ROBOTIS ships
this file under Apache-2.0 in its own repository (our copy is byte-identical
to `ROBOTIS-GIT/turtlebot3` on both `master` and `humble`, sha256
`98f0bd7046159f83…`), and unlike the Spot meshes — which carried Boston
Dynamics' own CAD part number and release annotation — nothing *inside*
`lds.stl` proves a foreign CAD import: no HLDS part number, no foreign
toolchain, no foreign unit system, and it sits in the same CAD project as
ROBOTIS' own chassis. It is entirely possible that ROBOTIS modelled the
housing themselves. **But that is "possible", not "established", and the
standard this ledger applies is that an indication is never upgraded to an
establishment.** What is established is that the depicted product is another
company's. No statement by ROBOTIS on who owns the LDS-01's industrial design
could be found, and ROBOTIS publishes no CAD for the part at all — the
e-manual page links a spec PDF and two Molex connector datasheets, and no
STEP, STL, IGES or drawing file.

### 3.3 What replaced them, and what did not change

Both removals follow the pattern used for the Robotiq 2F-140 and the OmniTug
500: the mesh visual becomes a primitive **authored here**, and specifically
**the link's own already-declared collision primitive**, reused verbatim. No
new dimension is introduced anywhere; visual and collider now coincide exactly,
which is a real improvement on both links.

`camera_link` (waffle only), replacing `r200.dae`:

```xml
<visual>
  <origin xyz="0.003 0.065 0.007" rpy="0 0 0"/>
  <geometry>
    <box size="0.012 0.132 0.020"/>
  </geometry>
  <material name="dark"/>
</visual>
```

`base_scan` (all three variants), replacing `lds.stl`:

```xml
<visual>
  <origin xyz="0.015 0 -0.0065" rpy="0 0 0"/>
  <geometry>
    <cylinder length="0.0315" radius="0.055"/>
  </geometry>
  <material name="dark"/>
</visual>
```

Those `<box size="0.012 0.132 0.020"/>` and `<cylinder length="0.0315"
radius="0.055"/>` declarations are ROBOTIS' own, present inline in these URDFs
long before this pass, and are plain primitive text rather than CAD.

**How closely they follow what they replaced.** For the camera, the mesh's
`rpy="1.57 0 1.57"` maps mesh-local → link as `X←z, Y←x, Z←y`, putting the
shell at X −0.0006…0.0072, Y 0…0.130, Z 0…0.020; the substituted box is at most
**2 mm** larger on X and Y and sits **3 mm** lower on Z. For the lidar the
substitution is coarser and deliberately so: a Ø110 × 31.5 mm disc against a
93.9 × 68.9 × 39.2 mm housing — the same envelope the solver already used, and
a round puck remains a fair generic representation of a spinning planar
scanner. `<material name="dark"/>` was kept on `base_scan` and **added** to
`camera_link`, which previously had none (the importer was falling back to a
hash-derived palette colour there).

**Nothing else changed.** Old and new were parsed and diffed structurally:

| file | joints identical | collisions + inertials identical | visuals changed |
|---|---|---|---|
| `turtlebot3_burger.urdf` | **yes** (6) | **yes** (7 links) | 1 — `base_scan` |
| `turtlebot3_waffle.urdf` | **yes** (12) | **yes** (13 links) | 2 — `base_scan`, `camera_link` |
| `turtlebot3_waffle_pi.urdf` | **yes** (10) | **yes** (11 links) | 1 — `base_scan` |

Every joint origin, axis, limit, parent and child; every `<collision>` block;
and every `<inertial>` (including `camera_link`'s, which upstream had already
commented out as unreliable) is untouched. `common_properties.urdf` was not
edited at all.

### 3.4 The two orphaned textures

Neither `r200.jpg` nor `astra.jpg` had a single reference anywhere in the tree,
and `r200.dae` contained **no `<library_images>` and no texture filename at
all**, so it never used its own namesake either.

`astra.jpg` is a 600 × 600 Photoshop-processed image (created 2012-11-19 at
UTC+09:00 in Photoshop CS5, converted from PNG, last saved 2016-03-09 at
UTC+08:00 in Photoshop CS6) carrying **no author, no rights and no copyright**
in its EXIF, XMP or IPTC blocks. It is the surface map of the Orbbec Astra CAD
that was already removed on design-owner grounds; leaving an unattributed image
of a third-party product behind after removing its mesh would have preserved
exactly the defect the removal was for. Both were deleted.

### 3.5 Verification

* Every `filename="package://…"` in all three URDFs was re-resolved against the
  files on disk: **27 references across this and the Clearpath packages, 0
  dangling.**
* All four XML files in `urdf/` parse
  (`python -c "import xml.etree.ElementTree as E; E.parse(...)"`), and no XML
  comment contains a `--` sequence.
* Tree-wide greps for `r200.dae`, `lds.stl`, `r200.jpg` and `astra.jpg` return
  only prose in `docs/` and world/source **comments** — no loader path. Those
  comments are listed as follow-ups in the pass report; they are outside this
  package.

### 3.6 Reversing this

Both removals are one command each
(`git checkout HEAD -- projects/robots/robotis/turtlebot3_description/`), and
the LDS decision in particular is a judgement on incomplete evidence, not a
finding of infringement. If ROBOTIS' authorship of the LDS-01 housing model is
ever established, restoring `lds.stl` costs nothing but the file.

## 4. Inventory

```
turtlebot3_description/
  LICENSE.upstream    Apache-2.0 boilerplate, holder/year fields UNFILLED (see §1)
  PROVENANCE.md       this file
  package.xml         v2.3.6; declares the licence and names the ROBOTIS authors
  CHANGELOG.rst       upstream history; the R200 / LDS entries cited in §3.1
  CMakeLists.txt      upstream ament build file (unused here)
  meshes/
    bases/burger_base.stl  waffle_base.stl  waffle_pi_base.stl     ROBOTIS
    wheels/left_tire.stl   right_tire.stl                          ROBOTIS
  rviz/model.rviz     upstream RViz config (unused here)
  urdf/               common_properties.urdf + the three robot URDFs
```

**Five binary meshes remain, and every one of them is a TurtleBot3 part that
ROBOTIS designs and builds.** No third-party product model is left in this
package.

## 5. Adding a mesh to this package

Ask the question in §2 before you add it. If the shape is another
manufacturer's product, ROBOTIS' Apache-2.0 grant does not reach it — author a
primitive instead, as [`projects/devices/`](../../../devices/PROVENANCE.md) and
[`projects/robots/omnisim/`](../../omnisim/PROVENANCE.md) do.
