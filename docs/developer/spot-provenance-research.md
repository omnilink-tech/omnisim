# Spot geometry — provenance research and costed options

> ## ✅ RESOLVED 2026-08-22 — option (c) was executed
>
> The owner chose **(c) replace the geometry with own-authored primitives**,
> and it has shipped. `projects/robots/boston_dynamics/spot/` is gone: the
> package is now `projects/robots/omnisim/omniquad/` ("OmniQuad", id
> `omniquad`, DEF `OMNIQUAD`), all 13 `.dae`, all 13 `.stl` and `spot_mat.png`
> are deleted, and every visual *and* collision solid is an inline URDF
> primitive emitted by `scripts/dev/gen_omnisim_robot_visuals.py`. The BSD-3
> `LICENSE.upstream` is retained because the kinematic skeleton is still a
> Clearpath derivative — see
> [`projects/robots/omnisim/omniquad/PROVENANCE.md`](../../projects/robots/omnisim/omniquad/PROVENANCE.md),
> which also carries the measured behaviour-preservation A/B (§7.4's protocol,
> executed).
>
> **Everything below is preserved as the evidence record it was.** Its paths,
> filenames and the name "Spot" are deliberately NOT rewritten: this document
> is about Boston Dynamics' Spot and renaming it would destroy its meaning.
> Read the paths below as historical.
>
> ### ⚠ Correction, 2026-08-24 — "deny-listed" is stale for two rows
>
> The §5.4 register and §5.5 both describe **NASA Valkyrie** and **Agility
> Digit** as *"❌ deny-listed"* / *"already publish-denied"*. That was true when
> written and is not true now: both packages were **REMOVED FROM THE TREE
> ENTIRELY** in `be41986f8` (2026-08-22), together with their dependent worlds
> and stand specs. `git ls-files | grep -icE 'nasa|valkyrie'` returns **0**, and
> the deny-list entries that named them were deleted with them — deliberately,
> because *"a deny-list entry depends on that file staying correct forever;
> deleting the material does not."*
>
> The distinction matters and is not pedantic. **Deny-listed** means the
> material is in the private tree, is being carried, and is withheld at publish
> time — a standing obligation and a standing risk. **Deleted** means there is
> nothing left to withhold. §5.5's analysis of *why* each grant failed is
> untouched by this and is still the reason the deletion happened; only the
> disposition changed. `franka_emika` in the same table **is** still held
> (physics, not licence) and its row is current.

**Status:** research + dependency map only. **No file was deleted, moved or modified to produce
this document.** The decision is a human/lead call; this file is the evidence and the costed
options it should be made on.

**Date:** 2026-08-22 · **Machine:** `9722d23d12a3` · **Tree:** `main`, HEAD `3b30359e0`

---

## 0. Executive summary

The repo's two ledgers disagree about `projects/robots/boston_dynamics/spot/`. The private
register ([`third-party-licenses.md:64`](third-party-licenses.md), blocker #9 at line 301) marks it
🔴 **DO-NOT-SHIP**; the public [`THIRD_PARTY_NOTICES.md:57`](../../THIRD_PARTY_NOTICES.md) and
[`NOTICE:260`](../../NOTICE) treat it as settled BSD-3. Only the permissive one gates the
release, so **Spot ships today in full** (`omnisim.yaml: publish: true`, zero entries in
`scripts/release/publish_deny.txt`).

Six findings, in order of how much they move the decision:

1. **The geometry is Boston Dynamics' CAD, and it says so inside the files.** Every one of the
   13 `.dae` meshes names its object
   **`02-042137-001-A00 TOP LEVEL DEFEATURED - NOT FOR PRODUCTION`** — a manufacturer part
   number plus a CAD release annotation. Boston Dynamics distributes exactly this to customers
   under the title **"Defeatured Spot CAD Models"**. Clearpath did **not** independently model Spot.
2. **The public `NOTICE` states the opposite, and it is wrong.** It asserts the meshes are
   *"a Blender-modelled and Blender-exported asset set, NOT an export from a mechanical CAD
   package"*. That inference is unsound — `<authoring_tool>` records the last tool to touch the
   file, not the origin of the shape — and the object name refutes it directly. **This is a
   factual error in a public legal notice and should be corrected regardless of which option
   is chosen.**
3. **BD themselves publish no open Spot geometry.** Their GitHub org carries `spot-sdk`,
   `spot-cpp-sdk`, `spot-rl-example` and a `mjlab` fork — **none contains a mesh or a URDF**.
   The defeatured CAD sits behind a Support Center login. The cheapest possible resolution —
   "BD license this themselves" — **does not exist.**
4. **But the identical geometry is published under MIT by the RAI Institute** (formerly the
   Boston Dynamics AI Institute), founded and led by Boston Dynamics' founder Marc Raibert and
   funded by Hyundai, which also owns BD. Proven identical here by three independent
   fingerprints, not by resemblance.
5. **And Google DeepMind redistributes the same geometry under the same BSD-3/Clearpath grant**
   in MuJoCo Menagerie. Our exact position is the mainstream practice of the field.
6. **Our copies are byte-identical to upstream** (a `diff` of `front_left_hip.dae` against
   `heuristicus/spot_ros` is **empty**; the 114-byte size delta is CRLF line endings). OmniSim
   is a pure redistributor. It has added no expression to the geometry and has nothing of its
   own to defend.

**Verdict (§4):** the BSD-3 chain of title is **UNSOUND as to the shape and SOUND as to
everything Clearpath actually authored**. Confidence **high** on the factual chain, **moderate**
on how much the shape-copyright deficiency practically matters.

**Recommendation (§8):** option **(a) keep + correct the attribution** as the immediate,
same-day action — because the current notice contains a false statement of fact, and that is a
worse exposure than the licence question itself. Option **(c) own-authored geometry** is the only
path that meets the owner's *"no one can say anything otherwise"* bar, and §7 shows it is **far
cheaper than it looks** — the contact-critical geometry is already own-authored.

**Spot is not the only licensor≠owner case in the tree (§5.4), but it is the only one that
currently publishes.** Agility Digit is worse (no grant exists at all) and NASA Valkyrie is
weaker than recorded — both are already publish-denied.

---

## 1. What ships today

`projects/robots/boston_dynamics/spot/` — 20 MB.

| Item | Count | Detail |
|---|---|---|
| `meshes/*.dae` (visual) | 13 | 1.16–5.28 MB each; `body.dae` = 5,277,950 B |
| `meshes/*_collision.stl` | 13 | **all exactly 16,084 B** = 320 facets each |
| `meshes/spot_mat.png` | 1 | 1,528 B, 64×64 RGB8 |
| `LICENSE.upstream` | 1 | BSD-3, © 2021 Clearpath Robotics Inc.; © 2022 Oxford Robotics Institute |
| `omnisim.yaml` | 1 | `publish: true` |
| `urdf/` | 5 URDFs + 1 generator | §6.1 |

Adding commit `4598fa51` (2026-05-13) says plainly:

> Replaces the PROTO Spot … with the URDF version from **Clearpath's fork of spot_description** …
> `projects/robots/boston_dynamics/spot/meshes/` — **26 mesh files downloaded from the upstream repo**

---

## 2. Question A — does Boston Dynamics publish Spot geometry, and under what licence?

### 2.1 BD's own GitHub org ships no geometry

`GET https://api.github.com/orgs/boston-dynamics/repos` returns five repositories:

| repo | licence | contains meshes/URDF? |
|---|---|---|
| `spot-sdk` | NOASSERTION | no |
| `spot-cpp-sdk` | NOASSERTION | no |
| `spot-rl-example` | MIT | **no** — full recursive tree is 30 entries, zero `.obj`/`.stl`/`.dae`/`.urdf`/`.xml` |
| `bosdyn-hospital-bot` | none | no |
| `mjlab` | Apache-2.0 (fork) | n/a |

`spot-rl-example` was the most promising candidate — MIT, published by BD, described as
*"Public-facing code originally developed by the AI Institute"* — and it contains **no geometry
at all**.

**Answer: NO.** Boston Dynamics, Inc. publishes no open-licensed Spot description package.

### 2.2 BD *does* publish the CAD — behind a login, with no open grant

BD's own SDK documentation, at
[`docs/payload/mechanical_interfaces.md`](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/master/docs/payload/mechanical_interfaces.md),
states verbatim:

> "Detailed CAD is available on the Support Center at
> https://support.bostondynamics.com/s/article/Defeatured-Spot-CAD-Models."

That article is **login-gated** (unauthenticated fetch → HTTP 404). BD's site
[Terms of Use](https://bostondynamics.com/terms/) reserve rights broadly:

> "All text, graphics, user interfaces, visual interfaces, photographs, trademarks, logos,
> sounds, music, artwork and computer code (collectively, 'Content') … is owned, controlled or
> licensed by or to Boston Dynamics."
>
> Materials may not be "copied, reproduced, republished, uploaded, posted, publicly displayed,
> encoded, translated, transmitted or distributed in any way … without Boston Dynamics'
> express prior written consent."

**No open redistribution grant for Spot CAD exists.** Note the title of BD's own article —
**"Defeatured Spot CAD Models"** — and hold it against §3.1.

### 2.3 The nearest thing to a grant: RAI Institute publishes the identical meshes under MIT

[`rai-opensource/spot_description`](https://github.com/rai-opensource/spot_description)
(formerly `bdaiinstitute/spot_description`) ships `spot_description/meshes/` and declares in
`package.xml`:

```xml
<!-- Copyright (c) 2025 Robotics and AI Institute LLC dba RAI Institute. -->
<maintainer email="engineering@theaiinstitute.com">AI Institute</maintainer>
<license>MIT</license>
```

Its `LICENSE` is a dual file: **MIT** (© 2023-2024 Robotics and AI Institute LLC dba RAI
Institute) followed by the **same BSD-3, © 2021 Clearpath Robotics Inc.** text we ship.

**Who RAI is, precisely.** The Robotics and AI Institute launched in 2022 as the *Boston
Dynamics AI Institute*, funded with $400M by Hyundai Motor Group (which also owns Boston
Dynamics), led by **Marc Raibert, Boston Dynamics' founder**; it renamed to the AI Institute and
then RAI Institute. Boston Dynamics and RAI announced a formal partnership in February 2025
([bostondynamics.com/news](https://bostondynamics.com/news/boston-dynamics-and-the-robotics-ai-institute-partner/),
[rai-inst.com](https://rai-inst.com/resources/press-release/hyundai-launches-boston-dynamics-ai-institute/)).

⚠️ **RAI Institute is a legally separate entity from Boston Dynamics, Inc.** An MIT grant from
RAI is *not* a grant from BD. It is, however, a grant from the organisation BD's founder runs,
over BD's own product geometry, standing publicly and unchallenged.

### 2.4 The RAI meshes are provably the *same* meshes

Not "similar" — the same asset. Three independent fingerprints:

| fingerprint | OmniSim (`.dae`) | RAI `spot_description` (`.obj`) | match |
|---|---|---|---|
| `body` object name | `…NOT FOR PRODUCTION.020` | `_2-042137-001-A00_TOP_LEVEL_DEFEATURED_-_NOT_FOR_PRODUCTION_020` | ✅ |
| `body` triangle count | **33,207** | **33,207** | ✅ |
| `front_left_hip` object name | `…NOT FOR PRODUCTION.008` | `…_008` | ✅ |
| `front_left_hip` triangle count | **7,938** | **7,938** | ✅ |
| `spot_mat.png` decoded pixels (MD5) | `cde550a488b2e582f014c71d1793cfa1` | `cde550a488b2e582f014c71d1793cfa1` | ✅ |

The texture files differ in container metadata only (ours carries extra `bKGD`/`tIME` chunks
from a re-save); the 64×64 RGB8 pixel payload is bit-identical.

### 2.5 Google DeepMind redistributes the same geometry under the same grant

[`google-deepmind/mujoco_menagerie/boston_dynamics_spot`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/boston_dynamics_spot)
ships a Spot model whose `LICENSE` is **the identical BSD-3, © 2021 Clearpath Robotics Inc.**
text in our `LICENSE.upstream`. Its README states:

> "It is derived from the [publicly available URDF description](https://github.com/bdaiinstitute/spot_ros2)."

Its `assets/body_collision.obj` is **14,532 bytes** — byte-for-byte the same size as RAI's
`meshes/base/collision/body_collision.obj`. Menagerie is a Google-published, heavily-scrutinised
asset collection with an explicit provenance policy. **A well-resourced organisation reached the
same conclusion we did and published on it.**

---

## 3. Question B — where did Clearpath's meshes actually come from?

### 3.1 The object name is a Boston Dynamics CAD part number

Every one of the 13 `.dae` files contains exactly one geometry, named:

```
02-042137-001-A00 TOP LEVEL DEFEATURED - NOT FOR PRODUCTION
```

with Blender's duplicate suffixes — `.001`, `.002`, `.003`, `.004`, `.007`, `.008`, `.009`,
`.010`, `.012`, `.015`, `.020`, one bare, and one reading `- 0`. **All 13 are duplicates of a
single imported object**, i.e. one CAD assembly imported into Blender and split into per-link
files. The suffix running to `.020` shows the original import had at least 21 objects.

Three things establish this is Boston Dynamics' CAD:

- **The part-number format is BD's.** A web search for the exact string surfaces sibling BD part
  numbers in the same format — `02-044200-001` and `02-040236-001`, both attributed to Boston
  Dynamics equipment ([device.report](https://device.report/boston-dynamics/02-044200-001),
  [manual-hub.com](https://manual-hub.com/manuals/boston-dynamics-spot-02-040236-001-01-pdf-manual/)).
- **"DEFEATURED" is BD's own word for their customer CAD release.** Their support article is
  literally titled *"Defeatured Spot CAD Models"* (§2.2). "Defeaturing" is the CAD operation of
  stripping internal detail from a production assembly before releasing it — exactly what a
  manufacturer does for payload developers.
- **"NOT FOR PRODUCTION" is a CAD drawing release state**, not something a 3D artist writes.

### 3.2 The tessellation density is CAD, not hand-modelling

| mesh | triangles | vertices |
|---|---|---|
| `body` | 33,207 | 92,334 (OBJ) / 16,530 positions (DAE) |
| `front_left_hip` | 7,938 | 22,450 (OBJ) |

A hand-modelled robot leg is hundreds to low thousands of triangles. 33k triangles for a body
shell is a tessellated CAD surface.

A further tell: RAI's `front_left_hip.obj` carries **3 UV coordinates** for a 7,938-triangle
mesh — a degenerate flat mapping, not an unwrap. Only `body.obj` has real UVs (34,069). The
human work in Blender was **import, assign two materials (`BlackAbs`, `wrap`), split by link,
export** — minutes of work, not modelling.

### 3.3 Who did the export, and when — to the minute

The COLLADA headers, verified on all 13 files:

```xml
<author>Blender User</author>
<authoring_tool>Blender 2.82.7 commit date:2020-03-12, commit time:05:06, hash:375c7dc4caf4</authoring_tool>
<created>2020-04-13T15:53:56</created>   <!-- body.dae, earliest -->
<created>2020-04-13T15:54:07</created>   <!-- rear_right_lower_leg.dae, latest -->
```

One 11-second export session. `Blender User` is Blender's default author string and names
nobody — the public `NOTICE` is right about that much.

**The commit closes the gap.** The root commit that introduced these files upstream:

```
repo    : heuristicus/spot_ros
sha     : 50b6c68089ed859c8101b06823ef8757b61da4bf   (no parents — initial commit)
author  : Dave Niewinski <dniewinski@clearpathrobotics.com>
date    : 2020-04-13T20:03:04Z
message : Initial commit
files   : 36 changed, 27 of them meshes, and NO LICENSE file
```

Clearpath Robotics is headquartered in Kitchener, Ontario (EDT = UTC−4 on that date). The
Blender exports are stamped **15:53:56–15:54:07 local = 19:53:56–19:54:07 UTC**; the commit is
**20:03:04 UTC**. **Dave Niewinski of Clearpath Robotics exported the meshes and committed them
nine minutes later.**

### 3.4 The licence arrived three years after the geometry

`heuristicus/spot_ros`'s root `LICENSE` has exactly one commit in its history:

```
8ad630d3ad  2023-04-01  Michal Staniaszek
"update license to include ORI, bump packages to v1 since the wrapper is a significant change"
```

So the BSD-3 that OmniSim ships as `LICENSE.upstream` was applied to the repository **on
2023-04-01, almost three years after the meshes were committed on 2020-04-13**, and by the ORI
maintainer rather than by Clearpath. The mesh provenance was plainly never separately assessed.
A GitHub issue search across `heuristicus/spot_ros` and `bdaiinstitute/spot_ros2` for mesh
licensing returns **zero results** — nobody upstream has ever raised it.

### 3.5 Our copies are unmodified

```
$ diff <(tr -d '\r' < upstream/front_left_hip.dae) <(tr -d '\r' < ours/front_left_hip.dae)
$ echo $?
0
```

Zero differing lines. The 114-byte size delta (1,233,763 → 1,233,877) is exactly the CRLF
conversion of a Windows checkout. **OmniSim redistributes upstream verbatim and has added no
expression of its own to the geometry.**

### 3.6 The chain of title, assembled

```
Boston Dynamics, Inc.
  designs Spot; owns the CAD assembly 02-042137-001-A00
  releases a DEFEATURED variant to customers via the Support Center (login-gated, rights reserved)
        │
        ▼   [no open licence at this step — this is the break]
Dave Niewinski, Clearpath Robotics
  2020-04-13 ~15:53 EDT: imports the defeatured CAD into Blender 2.82.7,
  assigns 2 materials, splits into 13 per-link meshes, exports .dae + 320-facet .stl hulls
  2020-04-13 20:03 UTC: commits them, WITH NO LICENCE FILE
        │
        ▼
heuristicus/spot_ros  (Michal Staniaszek, Oxford Robotics Institute)
  2023-04-01: adds a root BSD-3 LICENSE naming Clearpath (2021) + ORI (2022)
        │
        ├──▶ clearpathrobotics/spot_ros  (a FORK of heuristicus, created 2024-11-20)
        ├──▶ rai-opensource/spot_description  — re-exported to .obj, relicensed MIT + BSD-3
        ├──▶ google-deepmind/mujoco_menagerie — BSD-3, © Clearpath
        └──▶ OmniSim (4598fa51) — verbatim .dae/.stl, BSD-3 LICENSE.upstream
```

⚠️ Note the commit message in `4598fa51` calls it *"Clearpath's fork of spot_description"*. That
is backwards: **`clearpathrobotics/spot_ros` is a fork of `heuristicus/spot_ros`**, created
2024-11-20, four years after the meshes. The lineage is Clearpath-authored → ORI-maintained →
re-forked by Clearpath.

---

## 4. Question C — what does the BSD-3 grant actually cover? The verdict

A licensor can license only what they own. Split the asset in two:

### What Clearpath **does** own, and validly licensed under BSD-3

- The **URDF**: link/joint topology, axes, limits, inertial values, the `package://` layout.
- The **derivative arrangement**: the decision to split one CAD assembly into 13 per-link
  meshes, the naming scheme, the material assignment (`BlackAbs`, `wrap`), the `spot_mat.png`
  texture, the 320-facet decimated collision hulls.
- The **ROS driver code** the licence was originally written for.

For all of that the BSD-3 is **valid and effective**, and our `LICENSE.upstream` satisfies
clause 1 (the notice travels with the redistribution).

### What Clearpath does **not** own

- **The shape of Spot.** It is Boston Dynamics' industrial design, expressed in BD's CAD
  assembly `02-042137-001-A00`, obtained by Clearpath as a *defeatured* release under BD's
  reserved-rights terms. Clearpath's Blender work is a **format conversion and subdivision** of
  that CAD — the paradigm case of a derivative work with thin original authorship. The §3.2
  evidence (33k CAD-tessellated triangles, 3 placeholder UVs on the hip, one 11-second export)
  shows how thin.

### Verdict

> **The BSD-3 chain of title is UNSOUND as to the underlying shape, and SOUND as to everything
> Clearpath actually authored on top of it.**

**Confidence: HIGH** that the geometry originates in Boston Dynamics CAD rather than independent
modelling. The part number, BD's matching "defeatured" terminology, the single-object Blender
import split 13 ways, the tessellation density, and the placeholder UVs are five independent
indicators that all point one way, and none points the other. The task brief asked whether the
evidence supports "independently modelled from photos/dimensions" — **it does not, and it
positively refutes it.**

**Confidence: MODERATE** on practical consequence, and the reasons cut both ways:

*Toward "this is fine":*
- BD released this CAD deliberately, to customers, **for integration work** — simulating a robot
  you are integrating with is squarely the released purpose.
- BD has left it standing publicly for **six years** (2020-04-13 → today) across many
  high-visibility redistributions.
- **RAI Institute** — BD's founder's institute, BD's announced partner — publishes the identical
  geometry under **MIT** and maintains it as the reference ROS 2 description.
- **Google DeepMind** redistributes it under the same BSD-3 in Menagerie.
- Nobody upstream has ever raised the question (zero issues found).

*Toward "this is not clean":*
- None of the above is a licence. Acquiescence, affiliation and industry practice are not a
  grant, and **BD's own terms expressly reserve the rights**.
- The BSD-3 was applied to the repo **three years after** the meshes landed, by a party (ORI)
  that did not create them, with no mesh-provenance assessment on record.
- The originating commit carried **no licence at all**.

**What cannot be established from available evidence:** the terms attached to the Support Center
CAD download itself (login-gated), and whether Clearpath had any separate written agreement with
Boston Dynamics. Either could settle this in either direction and **neither is publicly
discoverable.** If certainty is required, it has to come from BD in writing.

---

## 5. Question D — beyond copyright, and the same test on every other family

### 5.1 Three distinct rights *(factual summary; NOT legal advice)*

| right | what it protects | what would infringe | does our asset implicate it? |
|---|---|---|---|
| **(i) Copyright** | the *expression* — the mesh file, the CAD model, the arrangement | copying the model without a licence | **Yes — this is §4's question.** The mesh is a copy of BD's CAD expression. |
| **(ii) Trademark** | the *name* and source-identifying marks | using "Spot"/"Boston Dynamics" so as to suggest endorsement or confuse origin | **Yes, but manageable.** Nominative fair use — naming a product to identify it factually — is generally permitted. Our `NOTICE` already carries the correct disclaimer. |
| **(iii) Design patent / trade dress** | the *ornamental appearance* of the product itself | making, using, selling or importing an article embodying the design | **Weakly, if at all.** A design patent covers an *article of manufacture*. A rendered image in a simulator is not ordinarily an article; a 3D-printed physical replica would be a different question. |

### 5.2 Trademark, verified

"SPOT" is a registered Boston Dynamics, Inc. trademark (USPTO serial **88184751**), first use
2018-06-08, covering *"quadruped robots with artificial intelligence for use in security, safety
and inspection applications"* among others ([uspto.report/TM/88184751](https://uspto.report/TM/88184751)).
"BOSTON DYNAMICS" is registered separately (reg. 5811310). BD's Terms of Use state: *"Neither the
name of Boston Dynamics nor any of Boston Dynamics' other trademarks … may be used in any way …
without Boston Dynamics' prior written permission."*

Note that BSD-3 **clause 3** and Apache-2.0 **§6** both expressly withhold trademark rights, so
neither licence helps here — the defence is nominative use, and it is the one we already assert.

### 5.3 Design patents, verified

Boston Dynamics, Inc. holds granted US design patents titled *"Robotic device"*, claiming *"The
ornamental design for a robotic device, as shown and described"* — e.g. **USD1013003S1** (filed
2022-08-10, granted 2024-01-30, assignee Boston Dynamics Inc), and more recently **D1,103,236**
(2025-11-25) and **D1,114,013** (2026-02-17)
([patents.google.com](https://patents.google.com/patent/USD1013003S1/en),
[patents.justia.com](https://patents.justia.com/patent/D1103236)).

⚠️ **Do not overstate this.** A design patent's claim is its *drawings*, and I could not confirm
from the public claim text that any specific one of these reads on the Spot body we ship. The
honest statement is: **BD actively maintains a design-patent portfolio over robot appearance**,
which is a reason to be careful about *physical* replicas and product-lookalike commercial use,
and a much weaker concern for a simulation mesh.

**Proportionate reading:** of the three, **(i) copyright is the real question** and the one §4
answers. **(ii) trademark** is already handled correctly by our nominative-use disclaimer.
**(iii) design patent** is worth knowing about and is not, on these facts, the operative risk.

### 5.4 Table E — "is the licensor the design owner?" across every robot family

Third-party mesh total excluding Spot: **~296 MB across 418 files.**

| family | robot(s) | upstream repo | licensor in our LICENSE | licensor == design owner? | verdict | published? |
|---|---|---|---|---|---|---|
| **boston_dynamics** | Spot | `heuristicus/spot_ros` → Clearpath fork | Clearpath Robotics Inc. (2021) + Oxford Robotics Institute (2022), BSD-3 | 🚩 **NO** — the shape is Boston Dynamics' CAD | 🚩 **UNSOUND as to shape** (§4) | ✅ **yes** |
| **unitree** | G1, H1, Go2, B2 | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) | HangZhou YuShu TECHNOLOGY ("Unitree Robotics"), BSD-3 | ✅ **YES** — Unitree's own org, own copyright, own robots, and they ship the meshes | **SOUND** | ✅ yes |
| **clearpath** | Husky, Jackal | [husky/husky](https://github.com/husky/husky), [jackal/jackal](https://github.com/jackal/jackal) | Clearpath Robotics Inc., BSD-3 | ✅ **YES** — Clearpath designs and builds both | **SOUND** | ✅ yes |
| **husarion** | ROSbot | [husarion/rosbot_ros](https://github.com/husarion/rosbot_ros) | Apache-2.0, **no © line**; maintainer `support@husarion.com` | ✅ **YES** — Husarion's own org and product | **SOUND** (minor: no © line to retain) | ✅ yes |
| **robotis** | TurtleBot3 | [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) | Apache-2.0, **no © line**; all authors `@robotis.com` | ⚠️ **PARTIAL** — YES for the robot; **NO for 2 of 8 meshes** (§5.5) | **SOUND**, one sub-component **QUESTIONABLE** | ✅ yes |
| **universal_robots** | UR3e, UR5e, UR10e | [UniversalRobots/…_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | BSD-3 — ⛔ **no holder, no year** (verified on `master`, `ros2`, `humble`) | ✅ **YES in substance** — UR A/S's own GitHub org, UR A/S a declared maintainer | **SOUND-BUT-UNNAMED** | ✅ yes |
| **franka_emika** | Panda | [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros) | Franka Robotics GmbH, Apache-2.0 | ✅ **YES** — Franka's own org, own NOTICE, own robot | **SOUND** | ❌ held (physics, **not** licence) |
| **nasa** | Valkyrie R5 | [openhumanoids/val_description](https://github.com/openhumanoids/val_description) | ⛔ **blank NOSA 1.3 template** | 🚩 **PARTIAL** — the hosting org is Edinburgh/MIT DRC consortium, **not NASA** | 🚩 **QUESTIONABLE** | ❌ deny-listed |
| **agility** | Digit | [adubredu/DigitRobot.jl](https://github.com/adubredu/DigitRobot.jl) — an **individual academic's** repo | ⛔ **nothing** | 🚩 **NO** | 🚩 **NO GRANT AT ALL** | ❌ deny-listed |
| **dji** | Mavic 2 Pro | — (OmniSim-authored primitives) | Apache-2.0 (OmniLink) | n/a — **zero DJI geometry** (3 boxes, 8 cylinders) | **SOUND** | ✅ yes |
| **robotiq** | 2F-140 | — (OmniSim-authored primitives) | Apache-2.0 (OmniLink) | n/a | **SOUND** | ✅ yes |
| **omnisim** | OmniArm6/7, OmniTug500 | — (original work) | Apache-2.0 (OmniLink) | n/a | **SOUND** | ✅ yes |

### 5.5 The other flagged cases, in detail

**🚩 `agility/digit` — NO GRANT. Structurally worse than Spot.**
Upstream is `adubredu/DigitRobot.jl`, an **individual academic researcher's** repository, not
Agility Robotics. **There is no LICENSE file on any branch of either mirror** (`main` and
`master` on both `adubredu/` and `alphonsusadubredu/` — all four 404). **Agility publishes no
Digit description at all** — their [public org](https://github.com/orgs/agilityrobotics/repositories)
has 8 repos, none robot-model-shaped. Our tree carries 20 `.obj` product meshes (4.67 MB) with
**no licence, no provenance file, no `omnisim.yaml`, and no row in the public notices**. Spot at
least has a *contestable* BSD-3; Digit has **no chain of title whatsoever** — there is no grant to
question and none available to obtain. Correctly deny-listed; the only clean resolutions are
deletion or a written grant from Agility. ⚠️ Register blocker #11 should **not** be closed by
adding a `LICENSE.upstream` — there is no text to add.

**🚩 `nasa/valkyrie` — weaker than the register records.**
Upstream `openhumanoids/val_description` is the **MIT / University of Edinburgh DARPA Robotics
Challenge consortium**, not NASA. The **only** licence assertion anywhere upstream is
`<license>NASA-1.3</license>` in `package.xml`; there is **no LICENSE file upstream**. (What
strengthens it: the same `package.xml` names maintainer `jordan.t.lack@nasa.gov`, a NASA JSC
address, and NOSA is NASA's own licence that nobody else would plausibly elect.) ⛔ **Our
`LICENSE.upstream` is the unexecuted SPDX template with the identifying fields blank** —
`Government Agency: _____`, `Original Software Title: _____` — so it never names NASA, while
NOSA §3 requires a copy of the agreement to accompany every redistribution. NOSA 1.3 is
OSI-approved but not FSF-approved and not Apache-relicensable
([SPDX](https://spdx.org/licenses/NASA-1.3.html)). Correctly deny-listed.

**✅ `robotis/turtlebot3_description` — the same defect as Spot, two orders of magnitude smaller. RESOLVED 2026-08-24.**
ROBOTIS unambiguously owns TurtleBot3, but two of its eight shipped meshes depicted **other
companies' products**: `meshes/sensors/r200.dae` (209 KB, SketchUp 8, 2015 — an **Intel RealSense
R200**, referenced by `turtlebot3_waffle.urdf:244`) and `meshes/sensors/astra.dae` (**9.23 MB**,
FBX COLLADA exporter, 2016 — an **Orbbec Astra**, unreferenced). Both were genuinely in ROBOTIS's
own upstream under Apache-2.0, but ROBOTIS does not own Intel's or Orbbec's CAD.
`astra.dae` went on 2026-08-22; `r200.dae` went on 2026-08-24, replaced by a primitive box
authored here that reuses the link's own collision envelope, and its orphaned texture `r200.jpg`
and the Astra's `astra.jpg` went with them.

⚠️ **This section MISSED a third one, and the miss is the interesting part.** It counted the two
meshes that *look* foreign — a SketchUp file in inches, an FBX export — and cleared the rest. But
`meshes/sensors/lds.stl` was also not ROBOTIS's to grant: the **LDS-01** is a rebadged **HLDS
(Hitachi-LG Data Storage) HLS-LFCD2**, which ROBOTIS' own e-manual states outright and for which
ROBOTIS hosts *HLDS' own drawing-control spec sheet*. Nothing inside the file gives it away — same
CAD project as ROBOTIS' own chassis, no foreign part number, no foreign units, and its `dilos`
header string is a red herring (the same string sits on the Burger and Waffle **chassis**, which are
indisputably ROBOTIS' own, so it is a CAD seat and not a design owner).
**The lesson: file forensics finds imported CAD, but it cannot find a REBADGED PRODUCT.** For that
the question has to be asked about the product — who designed the thing this depicts? — and answered
from the vendor's own documentation. `lds.stl` was removed on 2026-08-24 on that ground; it is one
`git checkout` to reverse if a ROBOTIS design-ownership statement ever surfaces.

**The five remaining meshes are all TurtleBot3 parts ROBOTIS designs and builds.**

**⚠️ `universal_robots` — sound in substance, unsatisfiable notice clause.**
Upstream's `LICENSE` **begins at "Redistribution and use…" with no `Copyright (c)` line** on any
branch, so BSD-3 clause 1 has no notice to retain. The in-tree provenance note records this
accurately and correctly refuses to invent a holder. Separately: the **UR20/UR30/UR15/UR18/UR8LONG**
meshes are under UR A/S's restrictive *"Terms and Conditions for Use of Graphical Documentation"*,
which upstream's own README concedes *"do not fully comply with OSI's definition of Open Source"*.
**We ship none of those families** — only ur3e/ur5e/ur10e. That constraint must survive any future
family addition.

### 5.6 Register hygiene found along the way (separate from the Spot decision)

1. ⚠️ **Universal Robots and Franka Emika have NO ROW AT ALL** in the private register
   `docs/developer/third-party-licenses.md` — the strings `franka`, `panda`, `ur_description`,
   `universal_robot` appear nowhere in it, while UR ships 29.4 MB publicly. A register missing two
   shipping families cannot be the evidence base for the public notices.
2. **Register blocker #10 (Unitree, "no BSD-3 text anywhere") is stale** —
   `projects/robots/unitree/LICENSE.upstream` now exists and byte-matches upstream.
3. **Blocker #11 conflates two different problems** — Valkyrie (weak chain) and Digit (no chain).
   They need separate dispositions.
4. **Digit appears nowhere in the public notices**, so the public tree carries no record of it.
5. **`omnisim.yaml` is decorative.** `publish_snapshot.sh` reads only `publish_deny.txt`; nothing
   in `scripts/` consumes `omnisim.yaml` except `step_to_urdf.py`, which *writes* it. So
   `franka_emika/panda/omnisim.yaml: publish: true` is inert but reads as a contradiction.
   **Consequence for Spot: setting `publish: false` on Spot would do nothing.** Only
   `publish_deny.txt` gates.

---

## 6. Dependency map — what a change to Spot would cost

Verified against `main` @ `3b30359e0`.

### 6.1 URDF variants

All five: **18 links / 17 joints** (12 revolute + 5 fixed).

| variant | collision blocks | **mesh** | **primitive** | visual `.dae` | inertials | git |
|---|---|---|---|---|---|---|
| `spot.urdf` | 13 | **9 STL** | **4 × `<sphere radius=0.035>`** | 13 | 13 | tracked |
| `spot.classic.urdf` | 13 | **9 STL** | **4 × sphere** | 13 | 13 | tracked |
| `spot_bigfoot.urdf` | 13 | **9 STL** | **4 × `<box 0.09 0.15 0.026>`** | 13 | 13 | tracked |
| `spot.rewritten.urdf` | 13 | 9 STL | 4 × sphere | 13 | 13 | **gitignored, zero consumers** |
| `spot_ghost.urdf` | **0** | **0** | **0** | 13 | **0** | tracked |

The brief's "9 mesh + 4 primitive" is **correct for four variants and wrong for
`spot_ghost.urdf`**, which is collision- and inertial-free by construction (`make_ghost_urdf.py`
strips them so the importer emits a kinematic visual-only body).

- **`spot.urdf`** — canonical; `spot.classic` + widened hip ROM for self-righting + `<rest/>` tags.
- **`spot.classic.urdf`** — the pre-widen model, deliberately preserved because the Newton
  residual-RL runs were trained against it.
- **`spot_bigfoot.urdf`** — differs from `spot.urdf` by **8 lines only** (4 sphere→box, 4 origin z).
- **`spot_ghost.urdf`** — depends on **the 13 `.dae` only**, no `.stl` at all.

**Per-link split, identical across the 4 physics variants:**
- **9 mesh-collision links:** `body`, and `{front,rear}_{left,right}_{hip,upper_leg}`
- **4 primitive-collision links:** `{front,rear}_{left,right}_lower_leg` — **the feet**
- **5 geometry-free frames:** `base_link`, `front_rail`, `real_front_rail`, `rear_rail`, `real_rear_rail`

⚠️ **Consequence:** the 4 `*_lower_leg_collision.stl` files are **referenced by nothing in the
repository**. Every variant already replaces the foot with a sphere or box. **Foot–ground contact
is already 100 % own-authored primitive geometry.** This is the single most important fact for §7.

### 6.2 Worlds — 39 in the live tree

- **`projects/policies/research/worlds/`** — 35 files: 28 use `spot.urdf`; 4 use
  `spot.classic.urdf` (`spot_residual_{deploy,train,train_perturb}_newton.omniworld`); 1 uses
  `spot_bigfoot.urdf` (`spot_terrain_mpc_bigfoot.omniworld`); 2 are ghost-only
  (`spot_hill_ghost_preview`, `spot_hill_shot`); 4 load a real body **and** a ghost together
  (`spot_{getup,jump,walk}_ghost_demo`, `spot_hill_deploy`).
- **Elsewhere:** `projects/robots/boston_dynamics/spot/worlds/spot.omniworld`,
  `projects/samples/demos/worlds/chat/omnilink_spot.omniworld`,
  `tests/engine/joint_limits/worlds/stress.omniworld` (→ `spot.classic.urdf`).
- Plus 21 in the `lane-l6` worktree and 11 frozen AgentBench snapshots.

⚠️ **`spot.omniworld` is cited in `AGENTS.md:75`, `ARCHITECTURE.md:135` and
`docs/WORLD_RECIPE.md` as the canonical lighting-recipe reference world.** Removing it breaks
three normative doc pointers.

### 6.3 Trained artifacts — **all tracked, all publishing**

**Deploy checkpoints** (`projects/policies/research/inference/policies/`): `gpu_spot_walk_main`
(champion, 374,633 B `.pt` + ONNX), `gpu_spot_walk_vc_main` (velocity-conditioned, 47.8 m / 0
falls), `gpu_spot_rough_main` (**`.pt` only, no ONNX** — 18 cm rubble), `gpu_spot_getup_main`,
`gpu_spot_jump_main`, `gpu_spot_residual_v8`.

**Archived** (`projects/policies/research/policies/`): `spot_omnisim_main`, `spot_walk_main`,
`spot_walk_mjx_main`, `spot_ppo_main`, `spot_residual_main` (the canonical 5.55 m/30 s residual
walker), `spot_residual_newton_main`, `spot_residual_perturb_main`, `spot_cpg_zero`.

**Ghosts** (`projects/policies/research/shadowing/ghosts/`): `spot_jump_ghost.npz` (47,766 B),
`spot_getup_ghost.npz`, `spot_crouch_ghost.npz`, `spot_hill{,8,12}_ghost.npz` (754,728 B each).
**Ghost LUTs** (`projects/policies/ghosts/spot/`): `spot_shadow_ghost_lut.json`,
`spot_stand_ghost_lut.json`.

**MJCF training models:** `spot_newton_fixed2{,_rough}.xml` + 6 `quad_morph/models/spot_*.xml`.

**Manifests:** `projects/policies/skills/quadruped/spot_walk/skill.json` (`method: residual-rl`,
`status: verified`, empty `ghost.lut`, carries the 2026-07-12 note *"Spot is NOT YET a Shadowing
walk"*); `skills/registry.json`; `motions/catalog.json:76`; `benchmarks/matrix.json:63`
(`spot_walk_durability`, candidate).

⚠️ **`projects/policies/common/robot_registry.py:70`** hardcodes
`"spot" → projects/robots/boston_dynamics/spot/urdf/spot.urdf`. **This is the single hardest path
binding outside the worlds.** Sibling: `projects/policies/research/backends/spot_robot_spec.py`
(whose `JOINT_LIMITS` still encode the *pre-widen* `spot.classic` ROM — a pre-existing divergence
unrelated to licensing).

**There is no Spot BATON *sequence manifest*** (all 6 in `skills/sequences/` are G1 or Go2), but a
full Spot BATON deploy path exists: `spot_baton_deploy.py` + world + `run_spot_baton_deploy.sh`.

### 6.4 Controllers, tests, docs

**Controllers:** `spot_simple_pose` (19,882 B, drives `spot.omniworld`);
`omnilink_quadruped_bridge` (**Spot is its hardcoded `--robot` default**); 15 research controllers
under `projects/policies/research/controllers/spot_*`; 10 control libraries under
`projects/policies/control/spot_*.py` + `gait/spot_{trot,crawl}_gait.py`.

**Tests:** `tests/engine/joint_limits/` (controller **hardcodes** the `spot.classic.urdf` path at
line 89); `tests/python/omniworld/test_viewpoint_check.py`;
`tests/test_robot_general_policies.py:209`; `tests/test_g1_lane_divergence_pins.py:633`;
`tests/smoke_omnilink_demos.sh:40` and `..._live.sh:57` (**both smoke lists include Spot**);
`tests/sources/test_textures.py` (a **known pre-existing failure** on `spot_mat.png`'s ICC profile).

**Benchmarks:** `omnibench/lane2/run_throughput.py:135` anchors *all quadruped* throughput mapping
on the Spot-named `BatchedSpotWalkEnv` class.

**Catalogues/docs:** `DEMOS.md` (9 refs), `demos.json:115` (`"id": "spot"`),
`omnisim/agent/cli.py:47`, `WORLDS.md`, 21 `scripts/dev/run_spot_*` runners,
`scripts/dev/viewpoint_targets.json` (**24 entries**), `docs/developer/spot-residual-rl.md`
(**59,745 B**) + ~40 other docs.

### 6.5 Publish gating — Spot is **not** gated

`scripts/release/publish_deny.txt`: `grep -i "spot\|boston"` → **zero matches**.
`omnisim.yaml` → `publish: true` (and per §5.6.5 that file is inert anyway). Every Spot checkpoint
lives outside the deny-listed `research/training/runs/**`, so **all of them publish**. Agility,
NASA and Franka packages *are* deny-listed; Spot is not.

### 6.6 Loose ends found (unrelated to licensing, worth a separate ticket)

1. **4 orphan STLs** (`*_lower_leg_collision.stl`, 64 KB) — referenced by nothing.
2. `spot.rewritten.urdf` — gitignored absolute-path scratch, zero consumers.
3. `spot_moving_demo.exe` — orphan binary; its only reference names a `.c` that does not exist.
4. `run_spot_baton_deploy.sh:77` → `gpu_spot_shadow_main/policy.onnx`, **directory absent** (falls
   back at line 79).
5. **6 stale paths** in the cinema/capture pipeline point at
   `projects/policies/worlds/spot_rl_deploy.omniworld` (correct: `.../research/worlds/`).
6. `spot_robot_spec.py` joint limits are stale w.r.t. the widened `spot.urdf`.
7. `WORLDS.md` says 34 Spot research worlds; there are 35.
8. `gpu_spot_rough_main` ships `.pt` only — nothing ONNX-based can deploy it.
9. Dangling legacy `Spot` PROTO at `tests/cache/worlds/backwards_compatibility.wbt:67`; no
   `Spot.proto` exists in the tree.

---

## 7. Would replacing the collision geometry break the trained policies?

This is the question that decides whether option (c) is affordable, and the answer is **much more
favourable than it first appears.** Three measured facts, then the caveat.

### 7.1 The contact that matters is already own-authored

Foot–ground contact is the dominant contact in every locomotion policy in §6.3. **All four feet
already use primitives** — `<sphere radius="0.035">` (or a box in bigfoot) — in every variant, and
the four foot collision meshes are dead files. **Replacing the remaining 9 collision meshes does
not touch foot–ground contact at all.**

### 7.2 MuJoCo already convexifies the 9 that remain — and they are nearly convex anyway

`AGENTS.md` records that every triangle-mesh collider is silently convexified: `solver_mujoco.py`
sends both `GeoType.MESH` and `GeoType.CONVEX_MESH` to `mjGEOM_MESH`, and MuJoCo compiles every
mesh geom through its convex-hull path. So the solver **never sees** the authored concave detail.

Measured here (all 9 in-use collision meshes, 320 facets each, `scipy.spatial.ConvexHull`):

| collision mesh | mesh volume (m³) | hull volume (m³) | **vol / hull** | bbox (m) |
|---|---|---|---|---|
| `body` | 0.032355 | 0.033918 | **0.954** | 0.892 × 0.256 × 0.214 |
| `front_left_hip` | 0.001457 | 0.001491 | **0.978** | 0.161 × 0.120 × 0.113 |
| `front_right_hip` | 0.001459 | 0.001491 | **0.979** | 0.161 × 0.120 × 0.114 |
| `rear_left_hip` | 0.001465 | 0.001493 | **0.981** | 0.161 × 0.120 × 0.113 |
| `rear_right_hip` | 0.001466 | 0.001494 | **0.981** | 0.161 × 0.120 × 0.113 |
| `front_left_upper_leg` | 0.004166 | 0.004863 | **0.857** | 0.121 × 0.151 × 0.438 |
| `front_right_upper_leg` | 0.004167 | 0.004870 | **0.856** | 0.121 × 0.151 × 0.438 |
| `rear_left_upper_leg` | 0.004163 | 0.004858 | **0.857** | 0.121 × 0.151 × 0.438 |
| `rear_right_upper_leg` | 0.004166 | 0.004866 | **0.856** | 0.121 × 0.151 × 0.438 |

Every one is **85.6–98.1 % convex**. The hulls the solver actually uses are within 2–14 % volume of
the authored meshes, and the hips are within 2 %. A hand-authored box or capsule fitted to the same
bounding volume is a **far closer substitute than intuition suggests** — for the hips it is almost
the identity.

### 7.3 Inertia is declared, not derived

All four physics variants carry **13 explicit `<inertial>` blocks**. Mass and inertia therefore do
**not** change when collision geometry changes — one of the two usual ways a collider swap
perturbs a policy is already off the table.

### 7.4 …but this must be MEASURED, not assumed

The above says the change is *plausibly* behaviour-preserving. It does not prove it, and a policy
is a closed loop that can be destabilised by small perturbations. **Do not ship a geometry change
on the strength of §7.1–7.3.** The protocol:

1. **Fix the venue.** `python projects/policies/common/env_fingerprint.py`; pin `OMNISIM_LINKER`;
   assert the `.newton.json` sidecar so Newton provably drove every run.
2. **Establish the baseline first**, on the unmodified tree: run each of the `gpu_spot_walk_main`,
   `gpu_spot_walk_vc_main`, `gpu_spot_getup_main` deploy worlds `n ≥ 5` seeds, recording **final x
   displacement, never-fell %, mean body height, foot-contact count per gait cycle, and per-joint
   torque saturation %**. `spot_walk/skill.json` already records that the current gait saturates
   210/750 steps (28 %) — a metric that will move if contact changes.
3. **A/B on one binary, interleaved.** Same process, same seeds, alternating arms; never compare
   across two build sessions.
4. **Accept on distribution overlap, not on a single number.** The pass bar is that the modified
   arm's displacement and never-fell distributions overlap the baseline's — not that one run
   "looks fine".
5. **Geometric control:** re-measure the §7.2 hull volumes for the substitutes and require each
   within ~10 % of the original hull, plus a rest-height check (drop the body, assert settle height
   matches baseline to <1 mm).
6. **Also run the joint-limit stress test** (`tests/engine/joint_limits/`) — the one test that
   *deliberately* pelts the collision geometry with impacts.

**If any policy degrades, retraining is available and cheap for this robot** — the residual recipe
in `docs/developer/spot-residual-rl.md` reports 20k steps / **52 s wall clock**. That is the fact
that makes option (c) tractable.

---

## 8. Costed options

Ordered most-preferred first. The bar is the owner's: **"completely legal and open source, so no
one can say anything otherwise."**

### (a) Keep + strengthen attribution — **do this immediately, whatever else is decided**

**What it costs:** hours. Documentation only. Zero code, zero geometry, zero policy risk.

**What it buys:**
- **Corrects a false statement in a public legal notice.** `NOTICE:270-273` currently tells the
  world the meshes are *"NOT an export from a mechanical CAD package"*. §3 refutes that. **A wrong
  assertion in a licence notice is a worse position than an honest uncertainty** — it reads as a
  claim made without checking, and it is checkable in ninety seconds by anyone who opens the file.
- Replaces it with what the files actually show: the `02-042137-001-A00 TOP LEVEL DEFEATURED - NOT
  FOR PRODUCTION` object name, the Clearpath-employee root commit, and the honest statement that
  the shape originates in BD CAD.
- Lets us cite the **RAI Institute MIT publication** and the **DeepMind Menagerie** precedent —
  genuinely strong context we are currently not using at all.
- **Resolves the ledger split** by making both files say the same, true thing.

**What it breaks:** nothing.

**Meets the bar?** **NO.** It makes our position honest and defensible, and it is strictly better
than today. It does not make the shape-copyright question go away, because §4 shows the grant does
not reach the shape. Someone *can* still say something.

### (b) Keep in-tree but publish-deny

**What it costs:** entries in `scripts/release/publish_deny.txt` — note per §5.6.5 that
`omnisim.yaml` is inert, so **the deny list is the only lever**. Following the Agility/NASA/Franka
pattern, it needs the robot dir plus dependent worlds, docs and the 15 checkpoints that live
outside the already-denied `runs/**`. Development continues unchanged.

**What it buys:** removes Spot from public distribution, which is where redistribution liability
actually lives. Aligns the gate with the private register's existing 🔴 DO-NOT-SHIP row.

**What it breaks:** a **lot** of public surface. `DEMOS.md` and `demos.json` lose the Spot chat
demo; `omnilink_spot.omniworld` and the quadruped bridge's default robot go; both smoke lists
(`tests/smoke_omnilink_demos{,_live}.sh`) need editing; the RL showcase table in `DEMOS.md` (vc
walk 47.8 m / 0 falls, get-up, crouch-recover) loses its subject; `docs/developer/
spot-residual-rl.md` (59,745 B) becomes a doc about a robot the public cannot run; and
`spot.omniworld`'s role as the canonical lighting-recipe reference in `AGENTS.md:75` /
`ARCHITECTURE.md:135` / `docs/WORLD_RECIPE.md` must be reassigned.

**Meets the bar?** **Partially.** Nothing questionable is redistributed, so the public artifact is
clean. But it is a retreat: the private tree still holds the geometry, and the flagship quadruped
disappears from the public product.

### (c) Replace the geometry with own-authored primitives

**What it costs.** Two separable halves — and this separation is the key insight:

- **Collision (9 meshes — cheap).** Replace each of the 9 `*_collision.stl` references with a
  `<box>`/`<cylinder>` fitted to the measured hulls in §7.2. Feet are already primitives; inertia
  is already declared. **Then run the §7.4 protocol.** Given 85.6–98.1 % convexity and a solver
  that already convexifies, the physics delta should be small — **but it must be measured, and the
  honest budget is a day of A/B runs plus up to 52 s per policy retrain if a metric moves.**
- **Visual (13 `.dae` — the real cost).** This is 20 MB of BD-CAD-derived shape and it is what a
  viewer actually sees. Options: (i) emit primitive visuals from
  `scripts/dev/gen_omnisim_robot_visuals.py`, exactly as was done for OmniArm 6/7, OmniTug 500 and
  the Robotiq 2F-140 on 2026-08-22 — **this precedent exists in-tree and worked**; or (ii) drop
  visuals and let collision geometry render. Either way Spot stops *looking* like Spot, which is a
  product decision, not a technical one.

**What it buys:** the geometry becomes OmniLink's own Apache-2.0 work. The copyright question
disappears entirely. Trademark reduces to nominative naming, already handled — and if the robot no
longer looks like Spot, the trade-dress question evaporates too.

**What it breaks:** the visual identity of the demos, the marketing stills, the cinema storyboards
and the YouTube material. Possibly some policy metrics until re-verified. Renaming the robot would
additionally touch ~40 docs, 21 runner scripts, 24 `viewpoint_targets.json` entries and
`robot_registry.py`.

**Meets the bar?** **YES — this is the only option that fully does.** It is exactly the remedy
already applied four times in this tree on 2026-08-22, so the pattern, the generator and the
`PROVENANCE.md` convention all exist.

### (d) Remove entirely

**What it costs:** the largest blast radius of the four. 39 live worlds, 15 checkpoints, 6 ghosts,
2 LUTs, 9 MJCF models, 17 research controllers, 10 control libraries, a skill manifest, a benchmark
entry, 5 tests (2 of them smoke lists), 21 runner scripts, `robot_registry.py`, the quadruped
bridge's default robot, and ~40 docs including a 59,745-byte canonical recipe.

Compare the **Atlas** precedent (blocker #9's other half): Atlas was removed cleanly on 2026-07-17
precisely because it was *"a confirmed negative result with no shipped dependents"*. **Spot is the
opposite** — a working, shipping, front-page demo with the deepest dependency graph of any robot in
the tree.

**What it buys:** total certainty, same as (c).

**Meets the bar?** **YES**, but at a cost (c) does not require. **Removal destroys the kinematics,
the tuned inertials, the trained policies and the demos — all of which are either ours or validly
BSD-3 licensed.** The problem is the *shape*, and (c) fixes the shape while keeping everything
else. **Do not choose (d) over (c) without a specific reason.**

### Recommended sequence

1. **Today — (a).** Correct `NOTICE` and `THIRD_PARTY_NOTICES.md`; reconcile the private register.
   The false CAD claim is the most urgent single item in this document.
2. **This week — decide between (c) and status quo.** (b) is available as a reversible holding
   position if a snapshot is imminent and the decision is not ready.
3. **If (c):** do the **collision half first** — cheap, measurable, and independently valuable (it
   also retires the 4 orphan STLs). Treat the **visual half** as a separate product decision, since
   it is the half that changes what the robot looks like.
4. **Independently of Spot:** §5.5–5.6 list four cheap wins that do not need this decision —
   delete the unreferenced 9.2 MB `astra.dae`, add the missing UR and Franka register rows, close
   the stale Unitree blocker #10, and split Digit from Valkyrie in blocker #11.

---

## 9. Reproducing this research

```bash
# The object name — the finding everything else rests on
grep -oE '<geometry id="[^"]*" name="[^"]*"' projects/robots/boston_dynamics/spot/meshes/body.dae

# COLLADA provenance headers, all 13
for f in projects/robots/boston_dynamics/spot/meshes/*.dae; do
  grep -oE '<(author|authoring_tool|created)>[^<]*' "$f" | head -3; done

# The upstream root commit (Clearpath employee, 2020-04-13)
curl -s "https://api.github.com/repos/heuristicus/spot_ros/commits?path=spot_description/meshes/body.dae"

# Identity vs the RAI Institute's MIT-licensed copy
curl -sL -o /tmp/rai_body.obj \
  https://raw.githubusercontent.com/rai-opensource/spot_description/main/spot_description/meshes/base/visual/body.obj
awk '/^f /{f++} END{print "faces="f}' /tmp/rai_body.obj      # -> 33207, == our body.dae

# We redistribute upstream verbatim
curl -sL -o /tmp/up.dae \
  https://raw.githubusercontent.com/heuristicus/spot_ros/master/spot_description/meshes/front_left_hip.dae
diff <(tr -d '\r' < /tmp/up.dae) \
     <(tr -d '\r' < projects/robots/boston_dynamics/spot/meshes/front_left_hip.dae)   # -> empty

# Convexity of the 9 in-use collision meshes (needs scipy)
#   -> 0.856-0.981 vol/hull; see the table in section 7.2
```

### Sources fetched

| claim | URL |
|---|---|
| BD's own SDK points at defeatured CAD | https://raw.githubusercontent.com/boston-dynamics/spot-sdk/master/docs/payload/mechanical_interfaces.md |
| BD's defeatured CAD article (login-gated; 404 unauthenticated) | https://support.bostondynamics.com/s/article/Defeatured-Spot-CAD-Models |
| BD Terms of Use — rights reserved, trademark restriction | https://bostondynamics.com/terms/ |
| BD GitHub org — no geometry in any repo | https://api.github.com/orgs/boston-dynamics/repos |
| RAI `spot_description` — MIT + BSD-3, ships meshes | https://github.com/rai-opensource/spot_description |
| RAI mesh identity (33,207 / 7,938 tris, same object names) | https://raw.githubusercontent.com/rai-opensource/spot_description/main/spot_description/meshes/base/visual/body.obj |
| RAI = ex-Boston Dynamics AI Institute, Hyundai-funded, Raibert-led | https://rai-inst.com/resources/press-release/hyundai-launches-boston-dynamics-ai-institute/ |
| BD × RAI partnership (Feb 2025) | https://bostondynamics.com/news/boston-dynamics-and-the-robotics-ai-institute-partner/ |
| DeepMind Menagerie ships Spot under the same BSD-3 | https://github.com/google-deepmind/mujoco_menagerie/tree/main/boston_dynamics_spot |
| Upstream root commit | https://api.github.com/repos/heuristicus/spot_ros/commits/50b6c68089 |
| BD part-number format corroboration | https://device.report/boston-dynamics/02-044200-001 |
| "SPOT" trademark, BD Inc., serial 88184751 | https://uspto.report/TM/88184751 |
| BD design patent "Robotic device" USD1013003S1 | https://patents.google.com/patent/USD1013003S1/en |
| BD design patent D1,103,236 | https://patents.justia.com/patent/D1103236 |
| Unitree — own org, own copyright, BSD-3 | https://github.com/unitreerobotics/unitree_ros |
| Clearpath Husky / Jackal — own products | https://github.com/husky/husky · https://github.com/jackal/jackal |
| Husarion ROSbot — own org | https://github.com/husarion/rosbot_ros |
| ROBOTIS TurtleBot3 — own org, third-party sensor meshes | https://github.com/ROBOTIS-GIT/turtlebot3 |
| UR ROS2 description — no copyright line in LICENSE | https://github.com/UniversalRobots/Universal_Robots_ROS2_Description |
| Franka — own org, own NOTICE | https://github.com/frankaemika/franka_ros |
| Valkyrie upstream — Edinburgh/MIT consortium, not NASA | https://github.com/openhumanoids/val_description |
| NOSA 1.3 — OSI-approved, not FSF-approved | https://spdx.org/licenses/NASA-1.3.html |
| Digit upstream — individual's repo, no LICENSE on any branch | https://github.com/adubredu/DigitRobot.jl |
| Agility Robotics org — publishes no robot description | https://github.com/orgs/agilityrobotics/repositories |
