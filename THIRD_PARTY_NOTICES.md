# Third-Party Notices

OmniSim is distributed under the Apache License, Version 2.0 (see [`LICENSE`](LICENSE)).
It also includes, links against, and redistributes third-party material that remains
subject to its own terms.

This file records what that material is, where it came from, and which licence applies.
It is the companion to [`NOTICE`](NOTICE), which is the short attribution file required
by Apache-2.0 §4(d).

> **Read this before redistributing.** Not every file in this tree is Apache-2.0.
> Some inherited models, textures, fonts, icons and reference data carry narrower terms.
> The [Carve-Out](#carve-out--material-not-covered-by-the-apache-20-grant) section at the
> end names them. When in doubt, check the individual file's header — many declare their
> own licence on a `# license:` line.

---

## 1. Upstream — Webots (Cyberbotics Ltd.)

OmniSim is a derivative work of [Webots](https://github.com/cyberbotics/webots), the
open-source robot simulator developed by Cyberbotics Ltd. and released under the
Apache License, Version 2.0 in December 2018.

    Copyright 1996-2024 Cyberbotics Ltd.

Substantial portions of the source tree — under `src/`, `projects/`, `resources/`,
`include/`, `lib/`, `docs/` and `Contents/` — originate from Webots. Files that retain
their original Cyberbotics copyright headers remain copyright of Cyberbotics Ltd.;
those headers are preserved in place, as Apache-2.0 §4 requires. Modifications to those
files, and all newly added files, are copyright of OmniLink and contributors and are
likewise licensed under Apache-2.0.

This included **WREN**, the *Webots RENderer* — Cyberbotics' own OpenGL renderer,
Apache-2.0, inherited with the derivation above. Its source was **deleted on
2026-08-23**, when wgpu-native became OmniSim's only renderer. What still ships is
`resources/wren/`: ten Cyberbotics-authored gizmo meshes and overlay textures that
the current wgpu GUI still reads, under `resources/wren/LICENSE`. WREN is unrelated
to the `wren-lang/wren` scripting language, which has never been part of this
project.

OmniSim is not affiliated with, endorsed by, or sponsored by Cyberbotics Ltd.

---

## 2. Robot models

OmniSim redistributes robot descriptions (URDF/MJCF) and 3D meshes originating from
third parties. Each remains subject to its upstream licence and to the rights of the
manufacturer whose product it depicts. Redistribution here implies no endorsement by,
or affiliation with, those manufacturers, and the Apache-2.0 grant over OmniSim's own
source code does not extend to third-party robot geometry.

| Robot | Manufacturer | Upstream source | Licence | Licence text in this tree |
|---|---|---|---|---|
| Husky, Jackal | Clearpath Robotics | `husky_description`, `jackal_description` | BSD 3-Clause — © 2015, 2021 Clearpath Robotics, Inc. ⚠️ **Three Hokuyo lidar meshes were REMOVED 2026-08-24** (`husky_description/meshes/accessories/hokuyo_ust10.stl`, `jackal_description/meshes/hokuyo_ust10.stl`, `jackal_description/meshes/hokuyo_utm30.stl`). Clearpath's BSD-3 is sound for what Clearpath designed, but **Hokuyo Automatic Co., Ltd. designs those sensors and granted nothing here** — `hokuyo_utm30.stl` names the product in its own STL header (`solid utm-30lx`) and both bounding boxes match Hokuyo's published enclosures on two axes each. All three were unreferenced. Clearpath-designed **mounts and brackets** named after third-party sensors (`velodyne_tower.stl`, `kinect_mount.stl`, the SICK LMS1xx brackets, `camera-bracket.stl` — whose header reads `solid Jackal Camera Brackets.SLDASM`) were deliberately KEPT: a bracket is Clearpath's design however it is named. | `projects/robots/clearpath/*/LICENSE.upstream`, `projects/robots/clearpath/PROVENANCE.md` |
| ROSbot, ROSbot XL | Husarion | [husarion/rosbot_ros](https://github.com/husarion/rosbot_ros) (`rosbot_description`) | Apache-2.0 | `projects/robots/husarion/rosbot_description/LICENSE.upstream` |
| TurtleBot3 (Burger, Waffle, Waffle Pi) | ROBOTIS | [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) (`turtlebot3_description`) | Apache-2.0. ⚠️ **Two third-party sensor models were REMOVED 2026-08-24** and replaced by primitives authored in this repository: `meshes/sensors/r200.dae` (an **Intel RealSense R200** — SketchUp 8, units in inches, 2015, no author or copyright element anywhere in 208 KB, bounding box = Intel's published 130 × 20 × 7 mm; ROBOTIS' own `CHANGELOG.rst:193` reads *"added Intel RealSense R200"*) and `meshes/sensors/lds.stl` (the **LDS-01**, which ROBOTIS' own e-manual documents as *"made by HLDS (Hitachi-LG Data Storage)"*, model HLS-LFCD2, and for which ROBOTIS hosts HLDS' own drawing-control spec sheet). ROBOTIS' Apache-2.0 is sound for what ROBOTIS designed and cannot reach either. Two orphaned, unattributed textures (`r200.jpg`, `astra.jpg`) went with them. **The five remaining meshes are all TurtleBot3 parts ROBOTIS designs and builds.** No joint, collider or inertial changed — verified by parsing all three URDFs before and after. | `projects/robots/robotis/turtlebot3_description/LICENSE.upstream`, `projects/robots/robotis/turtlebot3_description/PROVENANCE.md` |
| G1, H1, Go2, B2 | Unitree Robotics | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) (`g1_`/`h1_`/`go2_`/`b2_description`) | BSD 3-Clause — © 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") | `projects/robots/unitree/LICENSE.upstream` |
| OmniQuad (kinematic skeleton only) | — geometry is OmniSim's own; skeleton from Clearpath's `spot_description` | [clearpathrobotics/spot_ros](https://github.com/clearpathrobotics/spot_ros) (`spot_description`) | BSD 3-Clause — © 2021 Clearpath Robotics Inc.; © 2022 Oxford Robotics Institute. ⚠️ **All third-party geometry was DELETED 2026-08-22** — see the note below the table. | `projects/robots/omnisim/omniquad/LICENSE.upstream` |
| ~~Valkyrie (R5)~~ | NASA JSC | *(removed)* | ✅ **REMOVED FROM THE TREE 2026-08-22.** NOSA v1.3 is not Apache-2.0 compatible, and the `LICENSE.upstream` carried here was an **unfilled template** — every blank, including the agency name, left as `_____`. Nothing is redistributed, so no obligation attaches. | — |
| UR3e, UR5e, UR10e | Universal Robots | [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (`ur_description`) | BSD 3-Clause, per upstream's `package.xml` (`<license file="LICENSE">BSD-3-Clause</license>`). ⚠️ **No copyright holder or year is asserted, and none is known** — upstream's LICENSE carries the BSD-3 body with **no `Copyright (c) …` line above it**, so clause 1 has no notice to retain. `package.xml` names Universal Robots A/S only as a *maintainer* (beside 16 author entries from several organisations); the xacro-generated URDFs carry no header; the `.dae` meshes record only Blender's default `Blender User`. A **Provenance note at the top of `LICENSE.upstream`** records the gap and the evidence rather than inventing a holder. ⚠️ Upstream places **only** the UR8LONG / UR15 / UR18 / UR20 / UR30 meshes under Universal Robots A/S' restrictive "Terms and Conditions for Use of Graphical Documentation". **OmniSim ships none of those families** — only ur3e / ur5e / ur10e, which are BSD-3. Do not add a restricted family without re-checking the terms. | `projects/robots/universal_robots/ur_description/LICENSE.upstream` |
| Panda (+ Panda Hand) | Franka Emika / Franka Robotics | [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros) (`franka_description`), reached via the Webots `Panda.proto`, which declares Apache-2.0 and cites the same upstream | Apache-2.0 — © 2023 Franka Robotics GmbH. The licence covers the whole package including the meshes; upstream declares no mesh-specific or product-scoped carve-out. **Not included in the public snapshot** — the package is held from public distribution, so this entry applies to the development tree and to distributions that carry the model. | `projects/robots/franka_emika/LICENSE.upstream` (+ upstream's attribution notice, `NOTICE.upstream`) |
| Mavic 2 Pro | DJI | Kinematic URDF authored for OmniSim from primitive geometry | Apache-2.0 (OmniSim-authored). **No DJI CAD or mesh data is redistributed** — the package contains zero mesh files. "DJI" and "Mavic" are trademarks of SZ DJI Technology Co., Ltd., used only to identify the modelled aircraft. | — |

#### OmniQuad — why the geometry was removed, and what the BSD-3 still covers

This package used to ship 13 `.dae` visual meshes, 9 `.stl` colliders and a texture
inherited from Clearpath's `spot_description`. **All of it was deleted on 2026-08-22.**
The package now contains **zero binary asset files**; every shape is a primitive solid
authored in this repository and emitted by `scripts/dev/gen_omnisim_robot_visuals.py`.

**The evidence that forced it.** An earlier revision of this section asserted the geometry
was *"modelled and exported from Blender, not emitted by a mechanical-CAD package"*. The
files refuted it. Every one of the 13 named its geometry object:

```
02-042137-001-A00 TOP LEVEL DEFEATURED - NOT FOR PRODUCTION.<nnn>
```

a manufacturer part number with a CAD release annotation, carrying Blender's *duplicate*
suffix (`.008`, `.020`, …). Thirteen files, one imported assembly. Blender was the
**exporter**, not the author of the shape; `Blender User` is Blender's default author
string and names nobody. Boston Dynamics' own SDK documentation points payload developers
at a support article titled "Defeatured Spot CAD Models" — an exact terminology match, and
that CAD is login-gated.

**Why the BSD-3 notice is RETAINED anyway.** A licensor can only license what it owns, so
the grant could not convey rights in a shape Clearpath did not author — but it validly
covers what they *did* author and what this package still uses: the **kinematic skeleton**
(link topology, joint origins, axes, limits), carried over unchanged so existing worlds and
trained policies stay valid. Clause 1 requires the notice to travel with it, so it does, at
`projects/robots/omnisim/omniquad/LICENSE.upstream`.

**The substitution was measured, not assumed.** Holding the policy, world, controller and
engine constant and swapping only the URDF, the velocity-conditioned walk is **bitwise
identical** over ~11,000 steps — `max |Δpos| = 0.00000000 m`, 24.601 m travelled in both
arms — as is the bare trot model. One scenario did change and is reported rather than
buried: the get-up policy, whose contact set *is* the substituted body and thigh colliders,
now recovers where it previously flipped. Full evidence, including the voxel fits for each
collider (volume ratios 0.977–1.013): `docs/developer/spot-provenance-research.md` (archived 2026-09-02, see [docs/ARCHIVE.md](docs/ARCHIVE.md)).

**Trademark.** "Spot" and "Boston Dynamics" are trademarks of Boston Dynamics, Inc. This
package no longer models that machine and no longer uses those marks.

## 3. Vendored and bundled code

⚠️ This section carried no `## 3.` heading until 2026-08-24 — the file jumped from §2 to
§4 and its vendored-code subsections hung under nothing, which is also why the in-tree
native components below had no rows at all while `NOTICE` listed them. Both are fixed.

### Vendored native source (checked into this tree)

| Path | Component | Upstream | Licence | Licence text |
|---|---|---|---|---|
| `src/glad/glad.c`, `include/glad/glad.h` | glad OpenGL loader, generated v0.1.28 (2019-01-24, per the in-file banner) | [Dav1dde/glad](https://github.com/Dav1dde/glad) | **Generator: MIT — © 2013-2018 David Herberth** (glad's LICENSE at tag `v0.1.28`, which is MIT only and mentions neither Apache nor Khronos). **Generated output: glad's author elects Public Domain / WTFPL / CC0**, while the Khronos OpenGL XML registry it derives from is **Apache-2.0 — © 2013-2018 The Khronos Group Inc.** ⚠️ Whether the latter reaches through into generated code is **not settled**: glad's author writes "not a lawyer" and the Khronos registry maintainer's only statement is expressly "not speaking as a Khronos legal representative". OmniSim honours both notices. Generated with `--omit-khrplatform`, so no Khronos `khrplatform.h` ships and its separate notice does not arise (verified: no `#include <KHR/khrplatform.h>`, no `khronos_*` typedef, and no `KHR/` directory under `include/`). | `src/glad/LICENSE` (full record + verbatim MIT text); `include/glad/LICENSE` points at it |
| `src/glm` *(git submodule)* | GLM (OpenGL Mathematics) | [g-truc/glm](https://github.com/g-truc/glm) | The Happy Bunny License or MIT | `src/glm/copying.txt` — run `git submodule update --init` |
| `src/stb` *(git submodule)* | stb single-file libraries, `omichel/stb` branch `patch-1` | [nothings/stb](https://github.com/nothings/stb) | MIT / public domain (dual) | `src/stb/LICENSE` |
| `src/omnisim/external/siphash/` | SipHash, taken from google/highwayhash master as of 2017-04-24 | [google/highwayhash](https://github.com/google/highwayhash) | Apache-2.0 | `src/omnisim/external/siphash/LICENSE` + its own `NOTICE` and `Readme.md` |
| `src/omnisim/external/compilation_timestamp/` | compile-time date/time constants | — | **OmniLink's own**, Apache-2.0 | Clean-room reimplementation, 2026-08-22; the previous version was assembled from Stack Overflow answers (CC BY-SA) with no licence and was replaced, proven identical over 1,463 date cases |

### Fetched at build time / bundled only in binary releases

Not present in this git tree. Listed because a **binary** distribution of OmniSim carries
them, and a redistributor of that binary inherits their terms.

| Component | Version | Licence | Linkage |
|---|---|---|---|
| Qt 6 | 6.5.3 | **LGPL-3.0** | Dynamically linked; official builds bundle the unmodified shared libraries. Relinkable against a modified Qt 6 — which is what keeps the LGPL obligation satisfiable. |
| OpenAL Soft | — | **LGPL-2.1** | Dynamically linked, same posture as Qt 6. |
| Assimp | 5.2.3 | BSD-3-Clause | Fetched into `dependencies/` at build time |
| OpenVR | 1.0.7 | BSD-3-Clause | Fetched into `dependencies/` |
| OIS | 1.4 | zlib/libpng | Fetched into `dependencies/` |
| FreeType | — | FreeType Licence (FTL) / GPLv2 dual; OmniSim relies on the FTL | Bundled with the Qt runtime |
| OpenSSL | 3.0.14 | Apache-2.0 | Bundled in binary releases |
| wgpu-native | v29 | MIT **or** Apache-2.0 (dual) | The renderer, since WREN's deletion on 2026-08-23 |

### Web-viewer dependencies (`resources/web/wwi/dependencies/`)

Six vendored browser-side files. **Three of them — `ansi_up.js`, `quaternion.min.js` and
the assimpjs pair — also ship a separate licence text in that directory**
(`LICENSE-ansi_up.txt`, `LICENSE-quaternion.txt`, `LICENSE-assimpjs.txt`). For the rest,
each row's attribution is read out of the file's own header — the header text is the
evidence, and where there is no header that is stated as such.

| File | Upstream | Licence, as declared in the file itself |
|---|---|---|
| `ansi_up.js` | [drudru/ansi_up](https://github.com/drudru/ansi_up) | MIT. Header: `author : Dru Nelson / license : MIT`. **No copyright line** in the header; see upstream. |
| `glm-js.min.js` | [humbletim.github.io/glm-js](http://humbletim.github.io/glm-js) | MIT — © 2015-2016 humbletim. The full MIT text is embedded in the file's header comment. |
| `quaternion.min.js` | Quaternion.js v1.2.1 | MIT — © 2021 Robert Eisele (robert@xarg.org). |
| `libtess.min.js` | libtess.js (port of the GLU tessellator) | Permissive MIT-style with an **SGI FreeB attribution requirement** — © 2000 Silicon Graphics, Inc.; © 2015 Google Inc. The notice must be reproduced "including the dates of first publication and either this permission notice or a reference to `http://oss.sgi.com/projects/FreeB/`". Derived from the OpenGL Sample Implementation v1.2.1 (26 Jan 2000), © 1991-2000 Silicon Graphics, Inc. Verbatim text is embedded in the file's header. |
| `assimpjs.js` | **[cyberbotics/assimpjs](https://github.com/cyberbotics/assimpjs)** — a **fork** of [kovacsv/assimpjs](https://github.com/kovacsv/assimpjs) | ✅ **MIT — © 2021 Viktor Kovacs.** ⚠️ The upstream is the **fork**, not kovacsv: our bytes match `cyberbotics/assimpjs@main:dist/` exactly (sha256 `d63a8621f33f7f93…`), and **no kovacsv release matches the size** — all ten ship a 3.9–4.2 MB `.wasm` against our 2.44 MB, because the fork disables most importers. The tell is `MeshLoader.js:67`, which calls `ConvertFileList(…, 'assjson', true)` with a third argument upstream has never had (`const bool isMesh`, added by fork commit `6e9725f56e`, 2022-05-06). The fork's `LICENSE.md` is upstream's unchanged, so Kovacs is the sole holder. The file carries no header, so the notice is supplied by `LICENSE-assimpjs.txt` beside it — which also covers the **Emscripten runtime** compiled into it (MIT / U. Illinois-NCSA dual, © 2010-2014 Emscripten authors). |
| `assimpjs.wasm` | compiled [Assimp](https://github.com/assimp/assimp) **v5.2.3** + the assimpjs wrapper | Assimp is BSD 3-Clause. ✅ **The version is now established, not inferred** — the fork pins its `assimp` submodule at `19f2a624a9d69aa…`, the exact object `v5.2.3` resolves to, which is the same version the build fetches into `dependencies/` (see `Makefile.windows` there; the archive itself is fetched, not tracked). The binary is symbol-stripped and carries no embedded copyright; BSD-3 clause 2 requires one to travel with a binary redistribution — **discharged by `LICENSE-assimpjs.txt`**, which reproduces the © 2006-2022 assimp team notice, conditions and disclaimer verbatim, alongside the wrapper's MIT and Emscripten's dual notice. |
| `../wrenjs.js`, `../wrenjs.wasm`, `../wrenjs.data` | prebuilt from [cyberbotics/webots](https://github.com/cyberbotics/webots), fetched from `https://cyberbotics.com/wwi/R2025a/` | Three layers, **previously named in no attribution document at all**: WREN (Apache-2.0, © 1996-2024 Cyberbotics Ltd.; 325 `_wr_*` exports prove the identity), the 84 WREN GLSL shaders bundled inside `wrenjs.data` (same licence — ⚠️ these no longer exist as source in this tree, deleted with WREN in `976b9449d`), and **Emscripten's own runtime library** (MIT / U. Illinois-NCSA dual, © 2010-2014 Emscripten authors). None of the three files carries an embedded notice; `wrenjs.wasm` is fully stripped. Licence text: `resources/web/wwi/LICENSE-wrenjs.txt`. |

### Newton physics runtime

Release builds bundle the Newton physics runtime by default (`make release` sets
`BUNDLE_NEWTON=1`). Exact versions are pinned in
`scripts/packaging/newton_runtime_pins.py`.

| Component | Upstream | Licence |
|---|---|---|
| Newton | [newton-physics/newton](https://github.com/newton-physics/newton) (Linux Foundation / NVIDIA / Google DeepMind / Disney Research) | Apache-2.0 |
| NVIDIA Warp | [NVIDIA/warp](https://github.com/NVIDIA/warp) | Apache-2.0 |
| MuJoCo Warp / MuJoCo | [google-deepmind/mujoco_warp](https://github.com/google-deepmind/mujoco_warp) | Apache-2.0 |
| OpenUSD (`usd-core`) | [PixarAnimationStudios/OpenUSD](https://github.com/PixarAnimationStudios/OpenUSD) | Apache-2.0 (modified; see the upstream `LICENSE.txt`) |
| CPython (embedded interpreter) | [python.org](https://www.python.org/) | Python Software Foundation License 2.0 |

### Platform runtime

The Windows distribution additionally bundles a subset of the MSYS2 / MinGW-w64 runtime
under `msys64/`. Those binaries remain subject to their respective upstream licences
(MIT, BSD, LGPL, GPL with runtime-library exceptions, and others); see the upstream
MSYS2 packages for individual terms.

### Historical — removed dependencies

**Open Dynamics Engine (ODE), and the `libccd` and `OPCODE` components bundled inside
it, were REMOVED from this tree on 2026-08-08** (commit `bdc02139` deleted `src/ode/`
and `include/ode/`; Newton is now the only physics backend). Nothing here links or ships
them. Their attribution rows and the LGPL-3 `testsuites/cu` carve-out item are gone from
this file and from `NOTICE` for the same reason: **the paths those entries pointed at —
`src/ode/COPYING`, `src/ode/LICENSE.TXT`, `src/ode/LICENSE-BSD.TXT`,
`src/ode/libccd/BSD-LICENSE`, `src/ode/OPCODE/COPYING`, `src/ode/Makefile` and
`src/ode/libccd/src/testsuites/cu/` — no longer exist**, and an attribution file must
not point a redistributor at files they cannot follow. Verify with
`git ls-files src/ode include/ode` (empty).

Releases up to and including 2026-08-08 *did* ship ODE, under its BSD election. A
redistributor holding one of those releases should rely on the `NOTICE` and
`THIRD_PARTY_NOTICES.md` that accompanied it, not on this revision.

---

## 4. Simulation assets (PROTO models, textures)

Assets inherited from Webots are **not uniformly Apache-2.0**. Every Webots PROTO
declares its own licence on a `# license:` header line.

> **Measured 2026-08-22. Re-derive, don't trust the table.** Every number below comes
> from reading the *first* `# license:` line of every tracked `*.proto` file:
> `git ls-files '*.proto'`, then match `^\s*#\s*license\s*:\s*(.*)$` in the first 60
> lines of each. Values are grouped by their **verbatim** declared string, which is why
> `Apache License 2.0` and `Apache-2.0` are separate rows — they are different strings in
> the files, not a transcription slip.
>
> The previous revision of this table said "397 PROTO files under `projects/`" and its
> own rows summed to 262. Both were wrong; the measured figures are below.

Recounted 2026-08-24: the tree holds **452 tracked `*.proto` files** — **259** under
`projects/`, **1** under `resources/`, and **192** test fixtures under `tests/`.

**The 259 under `projects/` — the shipped asset library:**

| Licence declared (verbatim `# license:` string) | Count |
|---|---|
| `Copyright 2026 OmniLink. Apache 2.0.` — authored for OmniSim | 106 |
| `Apache License 2.0` | 77 |
| `Creative Commons Attribution 4.0 International License.` | 66 |
| `Apache-2.0` — the construction-site / stage demo models under `projects/samples/demos/protos/`, authored for OmniSim | 8 |
| `Creative Commons Attribution 3.0 United States License (original model by Andrew Kator & Jennifer Legaz).` — `projects/objects/traffic/protos/StreetLight.proto`, `ControlledStreetLight.proto` | 2 |
| **No `# license:` header** | 0 |
| *total* | **259** |

The 66 CC BY 4.0 PROTOs sit under `projects/objects/buildings/` (25),
`street_furniture/` (21), `garden/` (12), `trees/` (6) and `traffic/` (2). ⚠️ **None of
them carries an explicit copyright line of its own** — checked, 0 of 66. They are
attributed to Cyberbotics Ltd. on the strength of the Webots derivation in §1, not on the
strength of a per-file notice; the `# license:` line declares only the licence.

Every shipped PROTO under `projects/` now declares a `# license:` header — recounted
2026-08-22, 0 without one. The four sky/sun PROTOs and the two rendering samples that
previously lacked one are OmniSim-authored and now carry the OmniLink Apache-2.0 line;
`resources/templates/protos/template.proto` carries it too, so newly generated PROTOs
inherit it rather than repeating the defect.

**Outside `projects/`:** `resources/templates/protos/template.proto` (the new-PROTO
skeleton) **does** carry the OmniLink Apache-2.0 header, at line 2 — which is what makes
newly generated PROTOs inherit it. (An earlier revision of this section said it carried
none, four lines after saying it did; the file is the authority and it has the header.) Of the **192 test fixtures under `tests/`**, 2 declare
`Copyright 2026 OmniLink. Apache 2.0.` and **190 carry no header**; these are
OmniSim- and Webots-authored parser/renderer scaffolding, not redistributed third-party
assets, and they are the reason a repo-wide "no header" count (197) looks alarming next to
the shipped-product count (6).

✅ **Verified 2026-08-22: ZERO PROTOs in this tree declare "Licensed for use only with
Webots"**, and zero name Webots on a `# license:` line at all. Every PROTO that carried
the former Cyberbotics assets-licence text has been removed or re-authored. Re-run:
`git ls-files '*.proto' | xargs grep -l '# license:.*only with Webots'` → no output.
⚠️ Scope that grep to the `# license:` line as written. The unscoped form returns
`projects/samples/rendering/protos/PingPongBallScaled.proto`, whose hit is a PROSE COMMENT
explaining why a decal was REMOVED — not a licence declaration. The substantive claim
holds; only the command needed narrowing.

Image assets (`.png` / `.jpg` / `.hdr`) under `projects/objects/` and
`projects/appearances/` back these PROTOs and follow the licence of the PROTO that
uses them.

### Fonts

Four directories ship font files. Where a font declares its own licence in its
OpenType `name` table, that declaration is the evidence for the row — these are
not assumptions from the filename.

#### Engine fonts (`resources/fonts/`, mirrored as `.woff2` in `resources/web/wwi/fonts/`)

The web-viewer directory carries its own copies of the same licence texts.

| Font | Licence |
|---|---|
| Liberation Sans / Serif / Mono | SIL Open Font License 1.1. Text: `resources/fonts/LICENSE-Liberation-OFL-1.1.txt` |
| DejaVu Sans / Serif / Sans Mono | DejaVu Fonts License (Bitstream Vera derivative, permissive). Text: `resources/fonts/LICENSE-DejaVu.txt` |
| Raleway Light | SIL Open Font License 1.1. © 2010 Matt McInerney, © 2011 Pablo Impallari, © 2011 Rodrigo Fuenzalida, with Reserved Font Name Raleway. Text: `resources/fonts/SIL-Open-Font-License.txt` |
| Code2000 / Code2001 / Code2002 | © 1998-2008 James Kass (1998-2005 for Code2002). **No licence text accompanies these files in any form.** Their own metadata declares: Code2000 — *"Code2000 is shareware. Users are required to register the font after a reasonable evaluation period by sending $5.00 (US) or equivalent to: James Kass …"*; Code2001 / Code2002 — *"may be freely distributed. All rights reserved."* Shareware-with-registration is not an Apache-2.0 grant. **Excluded from the public snapshot** — all six files (3 `.ttf` + 3 `.woff2`). See the [Carve-Out](#carve-out--material-not-covered-by-the-apache-20-grant). |

#### Documentation webfonts (`docs/css/fonts/`, loaded by `docs/css/omnisim-doc.css`)

| Font | Licence |
|---|---|
| Roboto, Roboto Mono | **Apache-2.0**, as declared in the fonts' own metadata (name ID 13 `Licensed under the Apache License, Version 2.0`, name ID 14 the Apache URL). © 2011 Google Inc. (© 2015 for Roboto Mono). All Rights Reserved. "Roboto" is a trademark of Google. Text: `docs/css/fonts/LICENSE-Roboto-Apache-2.0.txt`. ⚠️ Google has shipped Roboto under more than one licence over time — the Apache election is what *these* files (v2.137/2017, v2.000985/2015) declare; re-read the name table before assuming it for a newer drop. |
| Raleway Thin | SIL Open Font License 1.1. © 2010-2013 Matt McInerney, Pablo Impallari, Rodrigo Fuenzalida, with Reserved Font Name "Raleway". Text: `docs/css/fonts/LICENSE-Raleway-OFL-1.1.txt` |

#### Brand fonts (`resources/branding/omnilink/fonts/`)

| Font | Licence |
|---|---|
| Montserrat (Light / Regular / Medium / SemiBold / Bold) | SIL Open Font License 1.1, as declared in the fonts' own metadata. © 2011 The Montserrat Project Authors ([JulietaUla/Montserrat](https://github.com/JulietaUla/Montserrat)), with Reserved Font Name "Montserrat"; designed by Julieta Ulanovsky. Text: `resources/branding/omnilink/fonts/LICENSE-Montserrat-OFL-1.1.txt` |
| The OmniLink display face (3 `.otf`) | ⚠️ **No licence grant of any kind.** The files carry no copyright, licence, licence-URL or vendor record in their own OpenType metadata, and no terms are filed with them. **Excluded from the public snapshot** — not redistributed. `resources/branding/omnilink/BRAND.md` names the OFL-licensed substitute. See the [Carve-Out](#carve-out--material-not-covered-by-the-apache-20-grant). |

### Icons (`resources/icons/`)

The GUI icon set is generated by `scripts/dev/gen_gui_icons.py` from two
Apache-2.0 sources: the toolbar and menu glyphs are rendered from
[Google Material Symbols](https://github.com/google/material-design-icons)
(Apache License 2.0), and the six viewpoint glyphs, the small UI primitives and
the splitter grips are OmniSim original artwork. Per-directory detail is in the
`license.txt` shipped in each icon directory.

### Reference-motion data (`projects/policies/`)

The G1 and H1 "ghost" reference trajectories under
`projects/policies/ghosts/g1/` have mixed provenance. Each LUT records its
own origin in its `source` field — check it before reuse:

- **Synthesized** (`ghost_*_synth_lut.json` and others) — generated by OmniSim's own
  `ghost_synth.py` tooling from first principles. Apache-2.0.
- **Recorded in-engine** (`ghost_official_*`, `ghost_h1_lut.json`) — trajectories
  recorded by running Unitree's official pretrained walk policy inside OmniSim. The
  recorded trajectory is OmniSim's own measurement. ⚠️ **The upstream policy weights
  ARE redistributed** — earlier revisions of this file said they were not, which was
  wrong; see §5 below.

### Benchmark reference data (`tests/benchmarks/`)

| Dataset | Upstream | Licence | Licence text in this tree |
|---|---|---|---|
| **DAIR cube-toss dataset** — `tests/benchmarks/omnibench/lane1r/data/` : **550** `.pt` trajectories (measured 2026-08-22 by `git ls-files 'tests/benchmarks/omnibench/lane1r/data/*.pt' \| wc -l`; an earlier count of 552 was wrong) plus `contactnets_cube.urdf` | [DAIRLab/dair_pll](https://github.com/DAIRLab/dair_pll) | **BSD 3-Clause** — © 2022, Dynamic Autonomy and Intelligent Robotics Lab. All rights reserved. | `tests/benchmarks/omnibench/lane1r/data/LICENSE` |

Third-party **measured data**, redistributed unmodified and **not** covered by the
Apache-2.0 grant in `LICENSE`. BSD-3 clause 1 requires the copyright notice, the
conditions and the disclaimer to be retained in a source redistribution — the `LICENSE`
file vendored beside the data, plus this row, are that material. This lane is **not**
excluded from the public snapshot (`scripts/release/publish_deny.txt` names nothing under
`lane1r/`), so the data ships.

Each trajectory is 550 recorded tosses of an acrylic cube onto a wooden table,
AprilTag/TagSLAM-tracked at 148 Hz, replayed by OmniBench lane 1R to score the engine
against *real* motion rather than a closed form. The dataset accompanies Acosta, Yang &
Posa, *"Validating Robotics Simulators on Real-World Impacts"*, RA-L 2022
([arXiv:2110.00541](https://arxiv.org/abs/2110.00541)), whose published per-simulator
baselines the lane compares against; provenance and method are recorded in
`tests/benchmarks/omnibench/lane1r/README.md`.

---

## 5. Pretrained policy weights

| Artifact | Upstream | Licence | Licence text in this tree |
|---|---|---|---|
| Unitree G1 and H1 pretrained walk policies — `projects/policies/research/controllers/{g1,h1}_unitree_deploy/motion.pt` | [unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) (`pre_train/g1/motion.pt`, `pre_train/h1/motion.pt`) | **BSD 3-Clause** — © HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics"). All rights reserved. | `projects/policies/research/controllers/g1_unitree_deploy/LICENSE.upstream`, `.../h1_unitree_deploy/LICENSE.upstream` |

These are third-party **binary** artifacts, redistributed unmodified. **BSD-3 clause 2
is why the licence files exist**: a binary redistribution must reproduce the copyright
notice, the conditions and the disclaimer in the materials accompanying it. They are not
OmniLink's work and are **not** covered by the Apache-2.0 grant in `LICENSE`.

In-tree evidence for the identification: the deploy scripts' own docstrings — *"OmniSim
deploy of UNITREE'S OFFICIAL G1 walk policy (unitree_rl_gym) / Runs Unitree's pre-trained
`pre_train/g1/motion.pt` with their EXACT control stack"* — plus the control constants
those scripts reproduce from Unitree's `deploy_mujoco.py` / `g1.yaml` / `h1.yaml`.

**No copyright year is asserted.** Nothing in this tree establishes the year range that
`unitree_rl_gym`'s own LICENSE declares; the 2016-2022 range shown for the Unitree URDFs
in §2 comes from a *different* upstream repository (`unitree_ros`).

⚠️ **Open item — do not treat as settled.** `unitree_rl_gym`'s training code descends from
`legged_gym` (ETH Zürich, Robotic Systems Lab) and `rsl_rl`, whose BSD-3 notices are
reported to carry additional **ETH Zürich** and **NVIDIA CORPORATION** copyright lines.
Whether those notices attach to a trained-weights artifact — as opposed to the training
source, none of which is redistributed here — has **not** been determined, and could not
be determined from this tree. No such copyright line or year is asserted anywhere in this
distribution because none could be sourced from it. If they do apply, the two
`LICENSE.upstream` files are incomplete and need those lines added verbatim from
upstream's own LICENSE files.

Also unresolved: `g1_bc_walk.pt` / `h1_bc_walk.pt` sit beside the two `motion.pt` files,
within ~16 bytes of the same size, and load as drop-in policies of the same shape. If they
are re-traced or behaviour-cloned copies of Unitree's weights they are derivative works
travelling under the same notice. How they were produced is not recorded in-tree.

---

## Carve-Out — material not covered by the Apache-2.0 grant

**Notice to redistributors and downstream users.**

The Apache License 2.0 in [`LICENSE`](LICENSE) applies to OmniSim's own source code and
to the Webots-derived source code it builds on. It does not, and cannot, grant rights in
third-party material that its owners licensed on other terms. The following components
are present in this tree under terms narrower than Apache-2.0:

1. **Code2000 / Code2001 / Code2002 glyph-fallback fonts — six files, two formats.**
   `resources/fonts/Code200{0,1,2}.ttf` and `resources/web/wwi/fonts/Code200{0,1,2}.woff2`.
   No licence text accompanies them in any form. Their own OpenType metadata declares
   Code2000 to be **shareware requiring per-user registration** (*"…sending $5.00 (US) or
   equivalent to: James Kass …"*), and Code2001 / Code2002 *"may be freely distributed.
   All rights reserved."* — a distribution permission with no permission to modify, which
   the `.woff2` format conversions arguably need. © 1998-2008 James Kass (1998-2005 for
   Code2002). **Excluded from the public snapshot**, so this entry applies to the
   development tree and to any distribution that does carry them.

2. **The OmniLink display typeface.** `resources/branding/omnilink/fonts/`, three `.otf`
   files. **No licence grant of any kind** accompanies them: no copyright, licence,
   licence-URL or vendor record in their own OpenType metadata, and no EULA or written
   permission on file. Retail display faces are typically licensed for use, not for
   redistribution. **Excluded from the public snapshot** — this entry applies to the
   development tree only. Do not add them, or any other retail typeface, back into a
   redistributed tree without a written redistribution grant.

3. ~~**`assimpjs.js`** (the JavaScript wrapper only).~~
   ✅ **RESOLVED 2026-08-24 — WITHDRAWN FROM THE CARVE-OUT.** The upstream was identified
   by sha256 as **cyberbotics/assimpjs**, a fork of `kovacsv/assimpjs` — which is why
   matching against upstream releases kept failing: every kovacsv release ships a
   3.9–4.2 MB `.wasm` against our 2.44 MB. The fork carries upstream's `LICENSE.md`
   unchanged: **MIT, © 2021 Viktor Kovacs**. The bundled Assimp was pinned to **v5.2.3**
   by submodule SHA, and the **Emscripten runtime** layer inside both files was named for
   the first time. All three notices now ship at `LICENSE-assimpjs.txt` beside the code.
   Nothing about these two files is unlicensed any more.
   *The lesson worth keeping: "the upstream is unidentifiable" was never true — it was a
   fork, and the fork was findable from a call signature in our own tree
   (`MeshLoader.js:67` passes a third argument upstream has never had).*

4. ~~**NASA Valkyrie (R5) geometry — NASA Open Source Agreement v1.3.**~~
   ✅ **WITHDRAWN 2026-08-22 — the package was REMOVED from the tree.** Nothing is
   redistributed, so no NOSA obligation attaches to any distribution of OmniSim. It was
   deleted rather than merely held because NOSA v1.3 is not Apache-2.0 compatible
   (GPL-incompatible, non-relicensable, and §3 requires the agreement accompany every
   redistribution), and because the licence file carried here was an **unfilled
   template** — every identifying blank, including the name of the government agency,
   was left as `_____`, so it did not satisfy its own §3 either. Agility Robotics'
   Digit went at the same time and for a starker reason: its upstream carried no
   LICENSE on any branch and Agility has published no redistribution grant, so there
   was no permission to rely on.

5. **`resources/branding/` — the OmniSim and OmniLink BRAND ARTWORK.** Copyright 2026
   OmniLink, all rights reserved. Full terms: [`resources/branding/LICENSE`](resources/branding/LICENSE).
   Unlike items 1–3 this is not a third party's restriction; it is **ours, asserted
   deliberately**, and it is the one carve-out a reader is most likely to trip over
   precisely because everything around it is permissively licensed.

   The distinction that matters: this is a **copyright** reservation in the image files,
   which is separate from the **trademark** reservation in the marks those images depict
   ([`TRADEMARKS.md`](TRADEMARKS.md), and Apache-2.0 §6). Apache-2.0 §6 withholds
   trademark rights but says nothing about copying a `.png`; on its own it would leave a
   recipient free to alter and republish this artwork so long as they did not use it *as*
   a mark. Reserving only the trademark would therefore have left the artwork inside the
   code grant, which was the state of this tree until 2026-08-24.

   ✅ **Reproducing the assets UNMODIFIED to refer factually to OmniSim or OmniLink needs
   no permission** — articles, talks, courses, compatibility badges, screenshots, docs for
   something you built on OmniSim. `TRADEMARKS.md` is deliberately generous here.
   ⛔ What is reserved is altering them, folding them into another project's identity, or
   shipping them as the branding of a modified distribution. A fork that redistributes
   modified OmniSim code must replace this directory with its own artwork.

   Two files inside the directory are governed by their own terms instead: **Montserrat**
   (SIL OFL 1.1, text alongside) and the **Hogira** display faces, which are item 2.


## Trademarks

"Webots" is a trademark of Cyberbotics Ltd. "OmniSim" and "OmniLink", the OmniLink
dot-sphere orb mark, and the associated brand palette and trade dress are trademarks of
OmniLink. All other trademarks are the property of their respective owners. Use of any
trademark in this file does not imply endorsement.

Every manufacturer and product name used in this project is used **nominatively**: to
identify, factually, the machine or component that a model depicts or that OmniSim
supports. No such use indicates endorsement, sponsorship, affiliation or approval by the
trademark owner. Both BSD 3-Clause (clause 3) and Apache-2.0 (§6) withhold trademark
rights, and no licence in this tree grants any.

**Robots:** "Husky", "Jackal" (Clearpath Robotics); "Go2", "B2", "G1", "H1" (Unitree
Robotics); "ROSbot" (Husarion); "TurtleBot" (a registered mark of the Open Source Robotics
Foundation, not of ROBOTIS); "UR3e", "UR5e", "UR10e" (Universal Robots A/S); "Panda"
(Franka Robotics); "Mavic", "DJI" (SZ DJI Technology Co., Ltd.).

**Components and sensors** modelled in `projects/devices/` — "Velodyne", "SICK", "Hokuyo",
"Robotiq", "Kinect" (Microsoft), "MultiSense" (Carnegie Robotics), "Jetson Nano" (NVIDIA),
"RPLIDAR" (SlamTec), "smartmicro", "Delphi", "ibeo", "Sharp", "TDK". **None of those
packages contains any mesh, texture or CAD file** — every one is modelled from primitive
solids, so no manufacturer geometry is redistributed. The same is true of the DJI Mavic.

**No longer used.** "Spot" and "Boston Dynamics": the quadruped package that used to carry
those names now ships OmniSim's own geometry under the name **OmniQuad** and does not use
the marks. "Valkyrie" (NASA JSC) and "Digit" (Agility Robotics): both packages were removed
from the tree entirely on 2026-08-22.

Note in particular that **"TurtleBot" is a registered trademark of the Open Source
Robotics Foundation (OSRF)**, not of ROBOTIS — ROBOTIS is the manufacturer of the
TurtleBot3 hardware and the copyright holder of the `turtlebot3_description` package,
which is a distinct thing from ownership of the mark.

The Apache License 2.0 covers the source code only; it grants no rights to use the
OmniLink or OmniSim trademarks (Apache-2.0 §6). See [`TRADEMARKS.md`](TRADEMARKS.md) for
the project's trademark policy.
