# Texture provenance — `projects/appearances/` and `projects/objects/`

**Status:** audited 2026-08-22. **Verdict: the tree is redistributable, with 9 files to remove
and one cross-directory duplicate to reconcile.** See [What is still open](#what-is-still-open).

This file exists because `scripts/release/publish_snapshot.sh` publishes a **squashed single
commit**. Git history does not travel to the public repository, so every provenance finding
below has to live in a file that ships. The two directory-level records that ship next to the
assets themselves are [`projects/appearances/protos/textures/LICENSE.txt`](../../projects/appearances/protos/textures/LICENSE.txt)
and [`projects/objects/PROVENANCE.md`](../../projects/objects/PROVENANCE.md); this document is
the evidence they rest on.

---

## 1. The question that was asked

An earlier audit flagged the PBR texture library as the **highest-risk unresolved entry** in the
tree, on this reasoning:

> material texture sets of this kind commonly originate from third-party texture sites whose
> terms vary per set, and none of that is recorded.

That is a **suspicion about a category of asset**, not evidence about these assets. It is a fair
suspicion — `textures.com` material, for instance, is licensed to a downloading account and may
not be redistributed as a texture file, which would be fatal to an Apache-2.0 distribution — so
it was tested rather than dismissed.

**The suspicion does not survive the evidence for the bulk of the library, and the real defect
turned out to be somewhere else entirely.** The genuine finding is 9 unreferenced files inherited
from a *product-scoped Cyberbotics licence*, which is a different problem with a different cause.

---

## 2. What the upstream actually says

OmniSim is a fork of [Cyberbotics' Webots](https://github.com/cyberbotics/webots).

### 2.1 The repository licence

`cyberbotics/webots`'s root [`LICENSE`](https://raw.githubusercontent.com/cyberbotics/webots/master/LICENSE)
is the **Apache License, Version 2.0**, verbatim and unmodified (fetched 2026-08-22).

The repository root has **no `NOTICE`, no `THIRD_PARTY`, no `CREDITS`, no `COPYRIGHT` and no
`AUTHORS` file** — verified against
[`api.github.com/repos/cyberbotics/webots/contents/`](https://api.github.com/repos/cyberbotics/webots/contents/),
whose complete root listing is `.eslintrc.json .flake8 .github .gitignore .gitmodules
.nautilus-metafile.xml CODE_OF_CONDUCT.md CONTRIBUTING.md Contents LICENSE Makefile README.md
bin dependencies distribution docs include lib projects resources scripts src tests`.

So **upstream carries no per-asset credit file of any kind.** Anything we want to know about a
texture's origin has to come from the PROTO header, the binary itself, or the commit history.

### 2.2 The PROTO header convention, and what Cyberbotics says it means

Upstream's own [`docs/reference/proto-design-guidelines.md`](https://raw.githubusercontent.com/cyberbotics/webots/master/docs/reference/proto-design-guidelines.md)
states, verbatim:

> If a PROTO is meant to be distributed, it is important to specify the license under which it
> can be used. For that purpose, the name of the license should be specified in the `license:`
> comment and the URL to the license file should be given in the `license url:` comment.

Two things follow, and they matter for the rest of this document:

1. The `# license:` line is Cyberbotics stating **the terms on which the distributed PROTO may
   be used**. A Webots PROTO's textures live in a `textures/` directory beside it and are
   referenced by relative URL — they are constituent parts of the distributed work, not
   independent files that happen to share a folder. The header is therefore intended to cover
   them.
2. The guidelines say **nothing** about what licence an asset must be under, and impose **no
   assertion language** on a contributor. The header records a conclusion; it does not document
   how the conclusion was reached.

### 2.3 Cyberbotics does NOT apply a blanket header — this is the decisive fact

Upstream discriminates per PROTO, and the discrimination is deliberate:

| upstream file | `# license:` line, verbatim |
|---|---|
| [`projects/appearances/protos/Asphalt.proto`](https://raw.githubusercontent.com/cyberbotics/webots/master/projects/appearances/protos/Asphalt.proto) | `# license: Apache License 2.0` |
| [`projects/objects/buildings/protos/Church.proto`](https://raw.githubusercontent.com/cyberbotics/webots/master/projects/objects/buildings/protos/Church.proto) | `# license: Creative Commons Attribution 4.0 International License.` |
| [`projects/objects/school_furniture/protos/Book.proto`](https://raw.githubusercontent.com/cyberbotics/webots/master/projects/objects/school_furniture/protos/Book.proto) | `# license: Copyright Cyberbotics Ltd. Licensed for use only with Webots.` |

That third row is a **product-scoped, non-open licence**. It is not Apache-2.0, it is not a
Creative Commons licence, and its stated `license url:`,
`https://cyberbotics.com/webots_assets_license`, returns **HTTP 404** on both
`cyberbotics.com` and `www.cyberbotics.com` (checked 2026-08-22) — so the terms of the grant are
not even readable.

A vendor that puts its *whole appearance library* on Apache-2.0 while carving specific object
PROTOs out under a product-scoped grant is a vendor that has looked at its own assets and sorted
them. **That is the strongest single piece of evidence in this audit**, and it cuts in the
library's favour: the appearance textures were not swept under a blanket header, they were
placed on the permissive side of a line Cyberbotics themselves drew.

At the moment of OmniSim's import that sort produced, across the two directories audited here:

| upstream `# license:` at import | PROTOs |
|---|---|
| `Copyright Cyberbotics Ltd. Licensed for use only with Webots.` | 238 |
| `Apache License 2.0` | 70 (68 of them the entire `projects/appearances/` library) |
| `Creative Commons Attribution 4.0 International License.` | 66 |
| `Creative Commons Attribution 3.0 United States License (original model by Andrew Kator & Jennifer Legaz).` | 2 |
| `Attribution-NonCommercial 4.0 International (original model by 3DHaupt)` | 1 |
| `MIT` | 1 |
| **total** | **378** |

*(Measured against commit `0db6a18a74ba16fa2c10f744423405d153b87c7a`, "Initial commit: OmniSim
robotics simulator", 2026-04-11 — the squashed upstream import.)*

### 2.4 Did Cyberbotics say where the textures came from?

**No — and the absence is documented rather than assumed.** Every commit touching
`projects/appearances/protos/textures` was read via
[`api.github.com/repos/cyberbotics/webots/commits?path=projects/appearances/protos/textures`](https://api.github.com/repos/cyberbotics/webots/commits?path=projects/appearances/protos/textures&per_page=100).
The messages are `"Appearances"`, `"New apperances."`, `"Cleanup textures."`, `"Add new
appearances (#4174)"`, `"Enhancement reduce assets size (#4643)"` and similar. **Not one names a
website, an artist, a marketplace or a licence.**

One commit deserves specific handling because it looks incriminating and is not:
[`a30519cad5e06de8348f3dbb826446f0ce0a1b70`](https://github.com/cyberbotics/webots/commit/a30519cad5e06de8348f3dbb826446f0ce0a1b70),
**"clean problematic exif data."** (DavidMansolino, 2020-04-24). Read cold, "someone stripped the
EXIF" is exactly what a provenance scrub looks like. It is not one:

* it touched **45 files**, not the library — 0 additions, 0 deletions, binary-only;
* the files span *robot* textures (Spot, Robotino3, TiaGo) as well as appearance ones, which a
  provenance scrub of a texture library would not;
* upstream issue [#1520](https://github.com/cyberbotics/webots/issues/1520), "JPEG textures not
  displayed, generate warning about *Unsupported image format*", records EXIF actually breaking
  texture **loading**. The commit is a rendering fix;
* decisively, **331 of our 333 appearance JPEGs still carry an APP1/Exif segment today.** Had
  this been a scrub, it failed at it.

---

## 3. What the binaries themselves say

Every JPEG marker segment and every PNG ancillary chunk was walked across **1,104 image files**
in `projects/appearances/` and `projects/objects/` — APP0–APP15, COM, and PNG `tEXt` / `iTXt` /
`zTXt` (inflated) / `eXIf` / `iCCP` — and Exif IFD0/ExifIFD plus the Photoshop IRB **IPTC**
block were parsed for the fields that carry credit: `Artist`, `Copyright`, `ImageDescription`,
`UserComment`, `XPAuthor`, `XPComment`, and IPTC `By-line`, `Credit`, `Source`,
`CopyrightNotice`, `Writer`, `OrigTransRef`.

### 3.1 Everything that was found

| value found | files | where |
|---|---|---|
| `Software = GIMP 2.9.9` | 138 | across the appearance library |
| `Software = GIMP 2.10.8` | 17 | across the appearance library |
| `Software = GIMP 2.10.4` | 3 | `marble`, `rough_pine`, `shiny_leather` |
| `Software = Adobe Photoshop CC 2015 (Windows)` | 3 | `grass/grass_artificial_*` |
| XMP `<xmp:CreatorTool>Substance Designer</xmp:CreatorTool>` | 4 | `metal_pipe_paint/*` |
| JPEG `COM = Created by fCoder Graphic Processor` | 2 | `rough_pine_occlusion`, `rough_pine_roughness` |
| `DateTime` / IPTC `DateCreated` | 331 | clustering **2019-02-20 → 2020-01-10** |
| `Copyright (c) 1998 Hewlett-Packard Company` | 49 | **ICC profile boilerplate** (sRGB IEC61966-2.1), not an image copyright |
| `Copyright (c) 2004 Microsoft Corporation` | 31 | **ICC profile boilerplate** (sRGB), not an image copyright |
| `<plus:ImageCreator>` / `<plus:CopyrightOwner>` | 3 | present as **empty XMP tags** with no value |

### 3.2 What was NOT found

**Zero** occurrences, across all 1,104 files, of:

* any populated `Artist`, `Copyright`, `ImageDescription`, `UserComment` or `XPAuthor` Exif tag;
* any populated IPTC `By-line`, `Credit`, `Source` or `CopyrightNotice` field;
* any `dc:rights`, `dc:creator` or populated `plus:` value in XMP;
* **any string matching** `textures.com`, `ambientCG`, `cc0textures`, `Poly Haven`, `polyhaven`,
  `hdrihaven`, `texturehaven`, `sharetextures`, `freepbr`, `3dtextures`, `opengameart`,
  `creativecommons`, `CC0`, or any other marketplace, stock or licence marker.

### 3.3 How much weight this carries — stated honestly

**Absence of a credit field is weak evidence of authorship and must not be reported as strong.**
Re-exporting a downloaded image through GIMP strips the original metadata just as effectively as
authoring it there does, and 138 files carry exactly that GIMP signature. On its own, §3.2 is
consistent with both "Cyberbotics made these" and "Cyberbotics downloaded these and re-exported
them".

What §3 *does* establish, and it is worth stating precisely:

1. **Nothing in any binary contradicts the declared licence.** There is no embedded copyright
   line naming a third party, which is the artefact that most often exposes this class of
   problem — and it is the artefact that condemned two other assets in this repository
   (see `projects/samples/rendering/protos/PROVENANCE.md`).
2. **Nothing traces to a restrictive source.** A `textures.com` origin, the outcome that would
   actually be fatal, has no positive support anywhere in the tree.
3. **The `Substance Designer` tag is a genuine positive signal, though only for 4 files.**
   Substance Designer is a procedural *material-authoring* tool; a file it exported was made,
   not photographed. It is a small sample and is reported as such.
4. **The date cluster corroborates the upstream history.** 2019-02 → 2020-01 matches Webots'
   R2019b/R2020a PBR appearance-library development window exactly, i.e. these were produced as
   part of that work rather than accumulated from elsewhere.

---

## 4. Per-directory table

Attribution below was computed by **resolving each PROTO's relative texture URL against the
PROTO's own directory** (`projects/objects/garden/protos/X.proto` + `"textures/foo.jpg"` →
`projects/objects/garden/protos/textures/foo.jpg`), for both the import-time tree and the
current tree. An earlier pass matched on bare basenames and got two buckets wrong, because
several categories reuse file names; the numbers here are from the resolving pass.

### 4.1 `projects/appearances/protos/textures/`

| path | files | declared | creator | evidence | risk |
|---|---|---|---|---|---|
| `projects/appearances/protos/textures/**` (63 material sets) | **333** (327 jpg, 6 png) | **Apache-2.0** | Cyberbotics Ltd. | 68/68 owning PROTOs declared `Apache License 2.0` at import **and** upstream today (`Asphalt.proto` re-verified live). Byte-unchanged from the import commit — 333/333. No third-party marker in any binary (§3). | **LOW** |

Every file in this directory is owned by a PROTO in `projects/appearances/protos/`, all 68 of
which are Apache-2.0. There has never been a Cyberbotics-restricted asset in this directory, at
any point in this repository's history. **This is the ~401-file bucket the original audit was
most worried about, and it is the cleanest part of the tree.**

### 4.2 `projects/objects/**`

| path | files | declared | creator | evidence | risk |
|---|---|---|---|---|---|
| `buildings/protos/textures/` | 351 | **CC BY 4.0** | Cyberbotics Ltd. | Owned at import *and today* by CC BY 4.0 PROTOs (25 of the 36 building PROTOs; the other 11 are re-authored OmniLink ones that bundle no texture). Byte-unchanged. | **LOW** — needs attribution, see §5 |
| `street_furniture/protos/textures/` | 130 | **CC BY 4.0** | Cyberbotics Ltd. | Same, via the 21 CC BY 4.0 street-furniture PROTOs. Byte-unchanged. | **LOW** — needs attribution |
| `garden/protos/textures/` | 62 | **CC BY 4.0** | Cyberbotics Ltd. | Same, via the 12 CC BY 4.0 garden PROTOs. Byte-unchanged. | **LOW** — needs attribution |
| `trees/protos/textures/` (16 of 18) | 16 | **CC BY 4.0** | Cyberbotics Ltd. | Same, via 6 CC BY 4.0 tree PROTOs. Byte-unchanged. | **LOW** — needs attribution |
| `traffic/protos/textures/` (14 of 17) | 14 | **CC BY 4.0** | Cyberbotics Ltd. | Same, via `WorkBarrier` + `ParkingMeter`. Byte-unchanged. | **LOW** — needs attribution |
| `backgrounds/textures/night_sky/` | 6 | Apache-2.0, © OmniLink | OmniLink | **Added after the import** (`f7bcc9b18`, 2026-04-26); the blobs appear nowhere in the import tree, so they are not renamed Cyberbotics cubemaps. No metadata of any kind. | **LOW**, one caveat in §6 |
| 16 files replaced during `b3038e3ae` (`bedroom`, `drinks`, `kitchen/breakfast`, `plants`, `school_furniture`, `traffic`, `trees`) | 16 | Apache-2.0, © OmniLink | OmniLink | Path existed at import under a Cyberbotics-restricted PROTO; **the blob was replaced**, so today's bytes are not upstream's. | **LOW** |
| `street_furniture/protos/textures/advertising.jpg` | 1 | Apache-2.0, © OmniLink | OmniLink | Blob **absent from the import tree at every path**. Replaces upstream's `cocacola_advertising.jpg`, deleted as third-party brand advertising. | **LOW** |
| `road/protos/textures/road_line_{dashed,triangle}.png` | 2 | Apache-2.0, © OmniLink | OmniLink | Replaced blobs. Not referenced by a PROTO body, but **documented in `Road.proto`'s `startLine`/`endLine` field comment as values a world may pass** — a live API surface. Keep. | **LOW** |
| `factory/conveyors/protos/textures/corrugated_plates_*.jpg` | **5** | *(none — orphan)* | **Cyberbotics Ltd., product-scoped** | Byte-unchanged from import. Sit in a directory owned by `ConveyorBelt.proto`, which upstream licensed `Licensed for use only with Webots.` **Distinct blobs** from the Apache-2.0 appearance set of the same name (md5s differ, all five). **Referenced by nothing.** | **⛔ REMOVE** |
| `apartment_structure/protos/textures/door_{base_color,normal}.jpg` | **2** | *(none — orphan)* | **Cyberbotics Ltd., product-scoped** | Byte-unchanged. Owned at import by `GenericDoorAppearance.proto`, `Licensed for use only with Webots.` **Referenced by nothing** — an earlier substring match to `picket_fence_with_door_base_color.jpg` was a false positive. | **⛔ REMOVE** |
| `balls/protos/textures/pingpong_logo.jpg` | **1** | *(none — orphan)* | **Cyberbotics Ltd., product-scoped** | Byte-unchanged. Owned at import by `PingPongBall.proto`, `Licensed for use only with Webots.` That PROTO was deleted in `b3038e3ae`; its texture was left behind. **Referenced by nothing here.** ⚠ A byte-identical copy (blob `f407344b`) is live at `projects/samples/rendering/` — see §6. | **⛔ REMOVE** |
| `factory/forklift/protos/meshes/forklift.mtl` | **1** | *(none — orphan)* | **Cyberbotics Ltd., product-scoped** | Byte-unchanged. Material sidecar for a deleted `.obj`; `Forklift.proto` references no mesh. **Referenced by nothing.** | **⛔ REMOVE** |
| **total texture images under `projects/objects/`** | **606** | | | (573 CC BY 4.0 + 25 OmniLink + 8 orphans to remove) | |

### 4.3 The rest of `projects/objects/` — icons and PROTO bodies

**115** assets whose import-time owner was a Cyberbotics-restricted PROTO survive the import, and
**all 115 are byte-modified since it** — 99 `protos/icons/*.png` preview renders (exactly the 99
PROTOs the sweep re-authored) plus the 16 textures already listed above. Not one was carried over
unchanged under a new header, so no shipped icon is a render of Cyberbotics' geometry. The icons
were not overlooked.

The only assets in this tree that *are* byte-unchanged under a restricted owner are the 9 orphans
in §6.1.

`projects/objects/**.proto` today: 104 × `Copyright 2026 OmniLink. Apache 2.0.`, 66 × CC BY 4.0,
4 × `Apache License 2.0` (2 inherited unchanged from upstream — `FifaSoccerBall`, `Rectangle`;
2 authored here — `RockHD`, `TerrainRock`), 2 × CC BY 3.0 US. **Zero** declare the Cyberbotics
product-scoped licence, zero declare CC BY-NC, zero declare MIT.

---

## 5. The licence chain, and what our obligation actually is

### 5.1 Apache-2.0 material (333 appearance textures + 2 object PROTOs)

Upstream licensed it Apache-2.0. Apache-2.0 is irrevocable and permits redistribution and
modification. **Our obligation is §4(a)–(d): ship the licence, state changes, keep the notices.**
That is discharged by the repository-root `LICENSE`/`NOTICE` and by the per-file headers.

There is no residual question here. The declaration is Cyberbotics' to make; the evidence in §2.3
and §3 supports rather than undermines it; and we carry it forward unchanged.

### 5.2 CC BY 4.0 material (573 textures under 5 categories, 66 PROTOs)

Upstream licensed it CC BY 4.0. CC BY 4.0 is compatible with redistribution inside an
Apache-2.0 project **provided §3(a) attribution travels with it.** Per the
[legal code](https://creativecommons.org/licenses/by/4.0/legalcode), a redistributor must retain

> identification of the creator(s) of the Licensed Material and any others designated to receive
> attribution, in any reasonable manner […] a copyright notice […] a notice that refers to this
> Public License […] a notice that refers to the disclaimer of warranties […] a URI or hyperlink
> to the Licensed Material to the extent reasonably practicable

and must

> indicate if You modified the Licensed Material and retain an indication of any previous
> modifications.

**Today `THIRD_PARTY_NOTICES.md` records only a count.** A count is not attribution. The block in
§7 is what CC BY 4.0 asks for; it needs to be pasted into that file by its owner.

None of the 66 CC BY 4.0 PROTOs names an individual creator — checked with a
`sponsor|model by|courtesy|thanks|designed by|created by|original` sweep across every PROTO in
both directories, which returned only the two CC BY 3.0 headers and one sponsorship line. The
creator of record is therefore **Cyberbotics Ltd.**, the licensor that made the grant.

### 5.3 CC BY 3.0 US material (2 PROTOs, no bundled textures)

`traffic/protos/StreetLight.proto` and `traffic/protos/ControlledStreetLight.proto` declare

    # license: Creative Commons Attribution 3.0 United States License (original model by Andrew Kator & Jennifer Legaz).

Andrew Kator & Jennifer Legaz published a well-known set of ~90 free 3D models at
`katorlegaz.com/3d_models/`. The canonical attribution string used by redistributors, quoted
verbatim from [a third-party project that carries it](https://raw.githubusercontent.com/mchamberlain/Cel-Shader/master/README.md):

> The model used in the third scene was obtained from: http://www.katorlegaz.com/3d_models/ and
> is licensed under a Creative Commons Attribution 3.0 United States License and is Copyright ©
> 2003-2012 Andrew Kator & Jennifer Legaz.

⚠ **`katorlegaz.com` no longer hosts the model repository.** The domain now serves unrelated
content and carries only `Copyright © 2026 Katorlegaz`; `web.archive.org` is unreachable from
this environment, so the original licence page could not be read first-hand. The wording above is
corroborated but **second-hand** — see §6.

Neither PROTO bundles a texture: both build from primitives and reference the Apache-2.0
`MattePaint` / `OldSteel` / `BrushedAluminium` appearance PROTOs. `StreetLight.proto` carries one
further credit line that must be preserved:

    # This model was sponsored by the CTI project RO2IVSim (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles).

### 5.4 Cyberbotics product-scoped material — the actual defect

238 object PROTOs arrived declaring `Copyright Cyberbotics Ltd. Licensed for use only with
Webots.` **OmniSim is not Webots**, and the harder the fork insists on that, the less it can
claim to be using those assets "with Webots". They cannot travel in an Apache-2.0 distribution.

This was recognised and remediated in `b3038e3ae` (2026-07-11): 136 unused PROTOs deleted, 99
used ones **re-authored clean-room from primitives** with their interfaces frozen, and 1,023
texture images plus their icons and meshes removed. Today **no file in this tree declares that
licence.**

The remediation is sound. What it left behind is **9 orphans** — files whose owning PROTO was
deleted or re-authored, but which were not swept up with it. They are byte-identical to
Cyberbotics' originals, they are referenced by nothing, and they still ship.

---

## 6. What is still open

### 6.1 ⛔ Nine files must be removed (blocking)

All nine are byte-unchanged from the upstream import, all nine were owned upstream by a PROTO
declaring `Copyright Cyberbotics Ltd. Licensed for use only with Webots.`, and all nine are
referenced by **nothing** — verified by repository-wide search for each basename and for each
containing directory path, including `omnisim://` absolute URLs.

```
git rm projects/objects/apartment_structure/protos/textures/door_base_color.jpg \
       projects/objects/apartment_structure/protos/textures/door_normal.jpg \
       projects/objects/balls/protos/textures/pingpong_logo.jpg \
       projects/objects/factory/conveyors/protos/textures/corrugated_plates_base_color.jpg \
       projects/objects/factory/conveyors/protos/textures/corrugated_plates_metalness.jpg \
       projects/objects/factory/conveyors/protos/textures/corrugated_plates_normal.jpg \
       projects/objects/factory/conveyors/protos/textures/corrugated_plates_occlusion.jpg \
       projects/objects/factory/conveyors/protos/textures/corrugated_plates_roughness.jpg \
       projects/objects/factory/forklift/protos/meshes/forklift.mtl
```

Removing them breaks nothing: each leaves an empty directory and no dangling reference.

### 6.2 ⛔ `pingpong_logo.jpg` also ships from `projects/samples/rendering/` — and that copy is live

`projects/samples/rendering/protos/textures/pingpong_logo.jpg` is **blob `f407344b`, byte-identical**
to the orphan above, and it *is* referenced, by `PingPongBallScaled.proto`. Deleting the
`projects/objects/` copy does not remove this asset from the distribution.

That directory's `PROVENANCE.md` currently elects Apache-2.0 for it on this reasoning:

> upstream Webots is Apache-2.0 and so is this repository, so both readings land on Apache-2.0

**That reasoning does not survive §2.3.** Upstream is *not* uniformly Apache-2.0 — it carves
specific assets out under a product-scoped grant, and this exact blob is one of them at its other
path: `projects/objects/balls/protos/PingPongBall.proto` declared, at import,
`# license: Copyright Cyberbotics Ltd. Licensed for use only with Webots.`
`PingPongBallScaled.proto` itself declared **no licence header at all** upstream, so it supplies
no independent grant.

This is outside the ownership of this audit and is reported, not changed. The options are to
replace the texture with own artwork (it is one logo decal on a `Sphere`), to drop the decal, or
to obtain written confirmation from Cyberbotics. **Leaving the current reasoning in place is not
one of them.**

### 6.3 The CC BY attribution is not yet discharged

573 texture files and 66 PROTOs are CC BY 4.0, and `THIRD_PARTY_NOTICES.md` records a count where
§3(a) requires an attribution. The block in §7 fixes it; it needs an owner to paste it in.

### 6.4 Things that are unresolved but not blocking

* **The upstream texture-authoring record.** Cyberbotics never wrote down where the appearance
  textures came from. §3 is consistent with in-house authorship and shows no trace of any
  restrictive source, but it does not *prove* authorship, and no amount of further forensics on
  these files will. **What would settle it:** written confirmation from Cyberbotics Ltd. that the
  `projects/appearances/` texture set is their own work, licensed Apache-2.0 as declared. That is
  a one-email question, and it is the only thing that would close this line completely.
* **The `webots_assets_license` terms are unreadable.** The URL 404s. It does not matter for the
  9 files above — the header text alone (`Licensed for use only with Webots.`) is enough to
  remove them — but it means the exact scope of that grant cannot be quoted.
* **The Kator & Legaz licence page is gone.** The attribution wording in §5.3 is corroborated by
  a third party but not read first-hand, and `web.archive.org` is unreachable from here. The
  claim being made is modest (we repeat upstream's own declaration and add the standard
  attribution), and CC BY 3.0 US is a permissive licence, so the risk is low. **What would settle
  it:** an archive.org snapshot of `katorlegaz.com/3d_models/` from a network that can reach it.
* **`night_sky/*` has thin positive provenance.** Six cubemap faces added post-import by the
  project owner's own account in a commit messaged `demo`, with no embedded metadata and no
  recorded generator script. The blobs are absent from the import tree, so they are certainly not
  renamed Cyberbotics cubemaps, and the surrounding commit `b3038e3ae` deleted the upstream
  cubemaps precisely because their provenance was unverifiable. **What would settle it:**
  committing (or naming) the script that generated them.
* **A stale test allowlist.** `tests/sources/test_textures.py` lists `'pingpong_logo.jpg'` in
  `duplicatedTextures`, an allowlist of basenames permitted to appear twice. After §6.1 the
  duplicate is gone and the entry is dead. It is permissive, so the test still passes; it is
  simply no longer true. `tests/**` is outside this audit's ownership.

---

## 7. CC BY attribution block — for `THIRD_PARTY_NOTICES.md`

To be pasted by the owner of that file. It is written to satisfy CC BY 4.0 §3(a)(1)(A)(i)–(iv)
and 3(a)(1)(B): creator, copyright notice, licence notice, warranty-disclaimer notice, URI, and
a modification statement.

```
-------------------------------------------------------------------------------
Creative Commons Attribution 4.0 International — Webots object library
-------------------------------------------------------------------------------

Work:      The Webots object library — 66 PROTO models and their 573 bundled
           texture images, comprising:
             * projects/objects/buildings/       25 PROTOs, 351 textures
             * projects/objects/street_furniture/ 21 PROTOs, 130 textures
             * projects/objects/garden/           12 PROTOs,  62 textures
             * projects/objects/trees/             6 PROTOs,  16 textures
             * projects/objects/traffic/           2 PROTOs,  14 textures
               (WorkBarrier, ParkingMeter)
Creator:   Cyberbotics Ltd.
Copyright: Copyright 1996-2024 Cyberbotics Ltd.
Source:    https://github.com/cyberbotics/webots
           (each PROTO's upstream original is at the identical path under
            .../blob/released/, e.g.
            https://github.com/cyberbotics/webots/blob/released/projects/objects/buildings/protos/Church.proto)
License:   Creative Commons Attribution 4.0 International Public License
           https://creativecommons.org/licenses/by/4.0/legalcode
Warranty:  The Licensed Material is provided as-is and as-available, and
           Cyberbotics Ltd. makes no representations or warranties of any kind
           concerning it. See sections 5 and 6 of the Public License linked
           above for the full disclaimer of warranties and limitation of
           liability.
Changes:   Modified by OmniLink. The PROTO header lines were rewritten during
           the OmniSim rebrand: the file-format token "#VRML_SIM" became
           "#OMNISIM", and the "documentation url:" line was repointed from
           webots.cloud to github.com/omnilink-tech/omnisim. The
           "license:" and "license url:" lines are unchanged. The PROTO bodies
           and every bundled texture image are byte-for-byte as received from
           upstream. No prior modification is indicated by upstream.

-------------------------------------------------------------------------------
Creative Commons Attribution 3.0 United States — street light models
-------------------------------------------------------------------------------

Work:      projects/objects/traffic/protos/StreetLight.proto
           projects/objects/traffic/protos/ControlledStreetLight.proto
           (models only; neither bundles a texture — both build from
            primitives and the Apache-2.0 appearance library)
Creator:   Andrew Kator & Jennifer Legaz
Copyright: Copyright (c) 2003-2012 Andrew Kator & Jennifer Legaz
Source:    http://www.katorlegaz.com/3d_models/
           (the original host no longer serves this collection; the URI is
            retained because CC BY 3.0 s.4(b) requires it "to the extent
            reasonably practicable")
License:   Creative Commons Attribution 3.0 United States
           https://creativecommons.org/licenses/by/3.0/legalcode
Via:       Adapted into Webots PROTO form by Cyberbotics Ltd.
           (https://github.com/cyberbotics/webots), from whom OmniSim received
           them. StreetLight.proto records that the Webots adaptation "was
           sponsored by the CTI project RO2IVSim",
           http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles
Changes:   Modified by OmniLink. Header lines only, as described above; the
           model bodies are as received.
```

---

## 8. Verdict

**Does this tree meet the "no one can say anything otherwise" bar?**

**For `projects/appearances/protos/textures/` — yes, and it always did.** 333 files, one
declared licence, Apache-2.0, made by the upstream that also carved *other* assets out under a
restrictive grant and deliberately did not carve these. Zero third-party markers in 1,104 scanned
binaries. The original audit's suspicion was reasonable and is not borne out.

**For `projects/objects/` — yes once the nine files in §6.1 are removed, the attribution in §7 is
pasted, and §6.2 is resolved.** The hard work was already done in `b3038e3ae`; what remains is
sweeping up after it and writing down the attribution CC BY has been owed all along.

**The framing that should be used externally** is the one the evidence actually supports: the
licences here are *upstream's declarations to make*, they are recorded per file, they discriminate
sensibly, and our obligation is to carry them forward accurately and attribute what needs
attributing. That is a defensible position. It is not the same as claiming we have proven where
every JPEG came from, and this document does not claim that.

---

## 9. How to re-derive any number in this document

Everything above is reproducible from the repository plus the upstream URLs cited. The method:

1. **Upstream licence sort:** `git show 0db6a18a74ba16fa2c10f744423405d153b87c7a:<path>` for any
   PROTO gives its header as received. Tally with a `^# license:` grep over the import tree.
2. **Byte provenance:** compare `git ls-tree -r 0db6a18a74ba16fa2c10f744423405d153b87c7a` with
   `git ls-tree -r HEAD` and classify each texture path as unchanged / modified / added by blob
   sha. Do **not** iterate `git rev-parse` per file — that is ~940 process spawns.
3. **Texture → PROTO attribution:** resolve each PROTO's relative texture URL against the PROTO's
   own directory. **Do not match on bare basenames** — `corrugated_plates_base_color.jpg` exists
   twice with different bytes, and `door_base_color.jpg` is a substring of
   `picket_fence_with_door_base_color.jpg`. Both produced wrong answers on the first pass.
   Remember that `omnisim://`-prefixed URLs are absolute and resolve from the repository root.
4. **Binary metadata:** walk JPEG marker segments (skip APP0/JFIF) and PNG ancillary chunks
   (inflate `zTXt`), then parse the Exif IFD and the Photoshop IRB 0x0404 IPTC block. Pillow's
   `_getexif()` is not sufficient — it misses IPTC entirely, which is where 48+ of the date
   fields live.
