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

This includes **WREN**, the *Webots RENderer* (`src/wren/`, `resources/wren/`) —
Cyberbotics' own OpenGL renderer, Apache-2.0, inherited with the derivation above.
It is unrelated to the `wren-lang/wren` scripting language, which is not part of
this project.

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
| Husky, Jackal | Clearpath Robotics | `husky_description`, `jackal_description` | BSD 3-Clause — © 2015, 2021 Clearpath Robotics, Inc. | `projects/robots/clearpath/*/LICENSE.upstream` |
| ROSbot, ROSbot XL | Husarion | [husarion/rosbot_ros](https://github.com/husarion/rosbot_ros) (`rosbot_description`) | Apache-2.0 | `projects/robots/husarion/rosbot_description/LICENSE.upstream` |
| TurtleBot3 (Burger, Waffle, Waffle Pi) | ROBOTIS | [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) (`turtlebot3_description`) | Apache-2.0 | `projects/robots/robotis/turtlebot3_description/LICENSE.upstream` |
| G1, H1, Go2, B2 | Unitree Robotics | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) (`g1_`/`h1_`/`go2_`/`b2_description`) | BSD 3-Clause — © 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") | `projects/robots/unitree/LICENSE.upstream` |
| Spot | Boston Dynamics (robot); geometry authored by Clearpath Robotics | [clearpathrobotics/spot_ros](https://github.com/clearpathrobotics/spot_ros) (`spot_description`) | BSD 3-Clause — © 2021 Clearpath Robotics Inc.; © 2022 Oxford Robotics Institute. The visual `.dae` meshes and the `spot_mat.png` texture were **authored by Clearpath in Blender** (the COLLADA headers record `<author>Blender User</author>`, Blender 2.82.7, 2020-04-13); they are not Boston Dynamics CAD, which is why Clearpath licenses them under BSD-3 and why DeepMind's `mujoco_menagerie` redistributes the same geometry on the same terms. | `projects/robots/boston_dynamics/spot/LICENSE.upstream` |
| Atlas (v5) | Boston Dynamics (robot); model maintained by the Robot Locomotion Group | [RobotLocomotion/models](https://github.com/RobotLocomotion/models) (`atlas/`), the Drake model set | BSD 3-Clause — © 2012-2022 Robot Locomotion Group @ CSAIL. Upstream's `LICENSE.TXT` sits beside `meshes/` in the `atlas/` package, so it covers the geometry. | `projects/robots/boston_dynamics/atlas/LICENSE.upstream` |
| Valkyrie (R5) | NASA Johnson Space Center | [openhumanoids/val_description](https://github.com/openhumanoids/val_description) | **NASA Open Source Agreement v1.3 (NASA-1.3)** — declared by upstream's `package.xml` (`<license>NASA-1.3</license>`). NOSA is OSI-approved but is **not** Apache-2.0, is GPL-incompatible, and is not relicensable. NOSA §3 requires that a copy of the agreement accompany every redistribution — hence the text below. Upstream ships no licence *file*; the verbatim agreement is reproduced here from the SPDX licence list. See the [Carve-Out](#carve-out--material-not-covered-by-the-apache-20-grant). | `projects/robots/nasa/valkyrie/LICENSE.upstream` |
| UR3e, UR5e, UR10e | Universal Robots | [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (`ur_description`) | BSD 3-Clause. ⚠️ Upstream places **only** the UR8LONG / UR15 / UR18 / UR20 / UR30 meshes under Universal Robots A/S' restrictive "Terms and Conditions for Use of Graphical Documentation". **OmniSim ships none of those families** — only ur3e / ur5e / ur10e, which are BSD-3. Do not add a restricted family without re-checking the terms. | `projects/robots/universal_robots/ur_description/LICENSE.upstream` |
| Panda (+ Panda Hand) | Franka Emika / Franka Robotics | [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros) (`franka_description`), reached via the Webots `Panda.proto`, which declares Apache-2.0 and cites the same upstream | Apache-2.0 — © 2023 Franka Robotics GmbH. The licence covers the whole package including the meshes; upstream declares no mesh-specific or product-scoped carve-out. | `projects/robots/franka_emika/LICENSE.upstream` (+ upstream's attribution notice, `NOTICE.upstream`) |
| Mavic 2 Pro | DJI | Kinematic URDF authored for OmniSim from primitive geometry | Apache-2.0 (OmniSim-authored). **No DJI CAD or mesh data is redistributed** — the package contains zero mesh files. "DJI" and "Mavic" are trademarks of SZ DJI Technology Co., Ltd., used only to identify the modelled aircraft. | — |

All robot and product names, and the trade dress of the robots depicted, are the
trademarks of their respective owners. The manufacturer names above are used
descriptively, to identify the machine each model depicts; they do not indicate
any endorsement, sponsorship or approval by those manufacturers.

---

## 3. Vendored and bundled code

Each component below remains subject to its own licence. Where a licence text ships
in-tree, its path is given; otherwise see the upstream source.

| Component | Upstream | Licence |
|---|---|---|
| Open Dynamics Engine (ODE) | [ode.org](https://www.ode.org/) | BSD 3-Clause **or** LGPL (dual); used here under the BSD option. Text: `src/ode/LICENSE.TXT` |
| GLM (OpenGL Mathematics) | [g-truc/glm](https://github.com/g-truc/glm) | The Happy Bunny License **or** MIT. Text: `src/glm/copying.txt` (git submodule — run `git submodule update --init` to fetch it) |
| stb (single-file libraries) | [nothings/stb](https://github.com/nothings/stb) | Public domain **or** MIT, at the user's option. Text: `src/stb/LICENSE` (git submodule) |
| GLAD (OpenGL loader) | [Dav1dde/glad](https://github.com/Dav1dde/glad) | Public domain / MIT |
| Assimp (Open Asset Import Library) | [assimp/assimp](https://github.com/assimp/assimp) | BSD 3-Clause |
| OIS (Object Oriented Input System) | [wgois/OIS](https://github.com/wgois/OIS) | zlib/libpng |
| SVOX Pico (text-to-speech) | SVOX Pico | Apache-2.0 |
| OpenVR | [ValveSoftware/openvr](https://github.com/ValveSoftware/openvr) | BSD 3-Clause. Fetched into `dependencies/` at build time; see the upstream `LICENSE`. |
| **Qt 6** | [qt.io](https://www.qt.io/) | **LGPL-3.0.** OmniSim links Qt 6 **dynamically**; official binary distributions bundle the unmodified Qt 6 shared libraries. Qt 6 remains copyright of The Qt Company Ltd. and contributors. Users may relink OmniSim against a modified Qt 6 by replacing those shared libraries, as LGPL-3.0 requires. |
| FreeType | [freetype.org](https://freetype.org/) | The FreeType License (BSD-style) **or** GPLv2 (dual); used here under the FreeType License. |
| OpenAL Soft | [openal-soft.org](https://openal-soft.org/) | **LGPL-2.1.** Linked dynamically; the unmodified shared library is bundled and replaceable. |
| wgpu-native | [gfx-rs/wgpu-native](https://github.com/gfx-rs/wgpu-native) | MIT **or** Apache-2.0 (dual). Shipped in official binaries; compiled to an inert stub in a plain source build. |

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

---

## 4. Simulation assets (PROTO models, textures)

Assets inherited from Webots are **not uniformly Apache-2.0**. Every Webots PROTO
declares its own licence on a `# license:` header line. Across the 397 PROTO files
under `projects/`:

| Licence declared | Count |
|---|---|
| Apache-2.0, © 2026 OmniLink — authored for OmniSim | 102 |
| Apache License 2.0 | 82 |
| Creative Commons Attribution 4.0 International (CC BY 4.0) — © Cyberbotics Ltd. | 66 |
| Creative Commons Attribution 3.0 United States — original models by Andrew Kator & Jennifer Legaz (`projects/objects/traffic/protos/StreetLight.proto`, `ControlledStreetLight.proto`) | 2 |
| MIT — including `projects/samples/rendering/protos/Helmet.proto`, from the Khronos Group glTF-WebGL-PBR reference implementation | 2 |
| Apache-2.0, © 2026 OmniLink (authored for OmniSim) | 2 |
| No `# license:` header | 6 |

Image assets (`.png` / `.jpg` / `.hdr`) under `projects/objects/` and
`projects/appearances/` back these PROTOs and follow the licence of the PROTO that
uses them.

### Fonts (`resources/fonts/`)

| Font | Licence |
|---|---|
| Liberation Sans / Serif / Mono | SIL Open Font License 1.1. Text: `resources/fonts/LICENSE-Liberation-OFL-1.1.txt` |
| DejaVu Sans / Serif / Sans Mono | DejaVu Fonts License (Bitstream Vera derivative, permissive). Text: `resources/fonts/LICENSE-DejaVu.txt` |
| Raleway | SIL Open Font License 1.1. © Matt McInerney, Pablo Impallari, Rodrigo Fuenzalida. Text: `resources/fonts/SIL-Open-Font-License.txt` |
| Code2000 / Code2001 / Code2002 | James Kass. No licence text accompanies these files — unverified; see upstream. |

### Icons (`resources/icons/`)

The GUI icon set is generated by `scripts/dev/gen_gui_icons.py` from two
Apache-2.0 sources: the toolbar and menu glyphs are rendered from
[Google Material Symbols](https://github.com/google/material-design-icons)
(Apache License 2.0), and the six viewpoint glyphs, the small UI primitives and
the splitter grips are OmniSim original artwork. Per-directory detail is in the
`license.txt` shipped in each icon directory.

### Reference-motion data (`projects/policies/`)

The G1 and H1 "ghost" reference trajectories under
`projects/policies/controllers/g1_ghost/` have mixed provenance. Each LUT records its
own origin in its `source` field — check it before reuse:

- **Synthesized** (`ghost_*_synth_lut.json` and others) — generated by OmniSim's own
  `ghost_synth.py` tooling from first principles. Apache-2.0.
- **Recorded in-engine** (`ghost_official_*`, `ghost_h1_lut.json`) — trajectories
  recorded by running Unitree's official pretrained walk policy inside OmniSim. The
  recorded trajectory is redistributed here; the upstream policy weights are not.

---

## Carve-Out — material not covered by the Apache-2.0 grant

**Notice to redistributors and downstream users.**

The Apache License 2.0 in [`LICENSE`](LICENSE) applies to OmniSim's own source code and
to the Webots-derived source code it builds on. It does not, and cannot, grant rights in
third-party material that its owners licensed on other terms. The following components
are present in this tree under terms narrower than Apache-2.0:

1. **Code2000 / Code2001 / Code2002 fonts.** `resources/fonts/` — no licence text
   accompanies these files; their terms are unverified.

2. **NASA Valkyrie (R5) geometry — NASA Open Source Agreement v1.3.**
   `projects/robots/nasa/valkyrie/`. NOSA v1.3 is an OSI-approved open-source licence,
   but it is **not** Apache-2.0: it is **incompatible with the GPL**, it cannot be
   relicensed, and **§3 requires that a copy of the agreement accompany every
   redistribution** — see `projects/robots/nasa/valkyrie/LICENSE.upstream`. It also
   obliges each redistributor to add a notice of their own modifications and to make
   source available. If you redistribute the Valkyrie model, you take on those
   obligations. The Apache-2.0 grant over OmniSim's source code does not extend to it.

Every other third-party robot model in this tree is redistributed under a permissive
licence whose verbatim text ships beside the geometry (BSD 3-Clause or Apache-2.0) —
see the table in §2 for the per-robot licence and the path to its licence file.

Do not assume Apache-2.0 rights in any of the above. If you redistribute OmniSim — in
whole or in part, in source or binary form — satisfy yourself that you have the rights
you need in each component you ship, and check the individual file headers.

---

## Trademarks

"Webots" is a trademark of Cyberbotics Ltd. "OmniSim" and "OmniLink", the OmniLink
dot-sphere orb mark, and the associated brand palette and trade dress are trademarks of
OmniLink. All other trademarks are the property of their respective owners. Use of any
trademark in this file does not imply endorsement.

Every manufacturer and product name used in this project — "Husky", "Jackal", "Spot",
"Atlas", "Go2", "B2", "G1", "H1", "ROSbot", "TurtleBot", "UR3e"/"UR5e"/"UR10e", "Panda",
"Mavic", and the names of the companies that make them — is used **nominatively**: to
identify, factually, the machine that a model depicts or that OmniSim supports. No such
use indicates endorsement, sponsorship, affiliation or approval by the trademark owner.
Both BSD 3-Clause (clause 3) and Apache-2.0 (§6) withhold trademark rights, and no
licence in this tree grants any.

Note in particular that **"TurtleBot" is a registered trademark of the Open Source
Robotics Foundation (OSRF)**, not of ROBOTIS — ROBOTIS is the manufacturer of the
TurtleBot3 hardware and the copyright holder of the `turtlebot3_description` package,
which is a distinct thing from ownership of the mark.

The Apache License 2.0 covers the source code only; it grants no rights to use the
OmniLink or OmniSim trademarks (Apache-2.0 §6). See [`TRADEMARKS.md`](TRADEMARKS.md) for
the project's trademark policy.
