# `projects/samples/rendering/protos/` — provenance

This file exists because `scripts/release/publish_snapshot.sh` publishes a
**squashed single commit** — git history does not travel to the public
repository, so provenance has to live in a file next to the assets.

## What ships here

| Asset | Origin | Licence |
|---|---|---|
| [`PbrMaterialSpecimen.proto`](PbrMaterialSpecimen.proto) | **Original work of this repository** (© OmniLink), authored 2026-08-22. Primitive geometry only — one `Cylinder` plus the caller's geometry node. Bundles no texture. | Apache-2.0 |
| [`SphereGrid.proto`](SphereGrid.proto) | Predates separable history (see below). Procedurally generated `Sphere` primitives; bundles no mesh and no texture. | Apache-2.0 |
| [`PingPongBallScaled.proto`](PingPongBallScaled.proto) | Predates separable history (see below). One `Sphere` primitive; bundles no mesh and, since 2026-08-22, **no texture**. | Apache-2.0 |
| ~~`textures/pingpong_logo.jpg`~~ | ⚠️ **DELETED 2026-08-22.** Byte-identical (`6185f74b…`) to an upstream Webots asset that shipped under *"Copyright Cyberbotics Ltd. Licensed for use only with Webots."* — a product-scoped grant that cannot ship in an Apache-2.0 tree. An earlier revision of this file reasoned that "upstream Webots is Apache-2.0 … both readings land on Apache-2.0"; **that was wrong**, because Cyberbotics does not apply a blanket header — it carves restricted assets out per file. The PROTO now uses a plain white `baseColor`. | n/a — removed |
| [`icons/SphereGrid.png`](icons/), [`icons/PingPongBallScaled.png`](icons/) | Predates separable history. PROTO preview renders. | Apache-2.0 |

No mesh, texture or model file from any third party remains in this directory.

**On "predates separable history".** Everything except `PbrMaterialSpecimen.proto`
entered the tree in the squashed initial import
(`0db6a18a74ba16fa2c10f744423405d153b87c7a`, 2026-04-11), which contains both
OmniSim's own work and the upstream Webots derivation. Git therefore cannot
distinguish the two for these files, and none of them carries an in-file
copyright line. `NOTICE` records them as OmniSim-authored. That question does
not affect the licence: upstream Webots is Apache-2.0 and so is this
repository, so both readings land on Apache-2.0 — and, decisively for this
audit, neither PROTO carries a third-party **mesh** or a non-permissive claim.
That is exactly what `Helmet.proto` and `Telephone.proto` did carry, which is
why they are treated differently below and these are not.

Both files were missing the `# license:` header tag that `NOTICE` flags as
"a defect to fix, not a licensing gap"; the tag was added 2026-08-22 so the
files now state their own terms.

## Removed: `Helmet.proto` and `Telephone.proto` (2026-08-22)

Two imported PROTOs were **deleted from this repository** — `Helmet.proto` and
its assets from this directory, and `projects/objects/telephone/` in its
entirety. Both were inherited in the initial upstream-Webots import
(`0db6a18a74ba16fa2c10f744423405d153b87c7a`, 2026-04-11).

### The defect

Both PROTOs declared:

    # license: MIT
    # license url: https://opensource.org/licenses/MIT

and both cited the same source in their description line:

    sourced from the GLTF PBR reference implementation, found at
    https://github.com/KhronosGroup/glTF-WebGL-PBR

**MIT is the licence of that repository's reference-implementation CODE. It is
not the licence of the sample models that implementation displays.** The
headers tracked the repository, not the asset. (Its successor,
`KhronosGroup/glTF-Sample-Viewer`, is now Apache-2.0 — the code licence has
moved and the models' have not, because they were never the same grant.)

### `Helmet.proto` — upstream states CC BY-NC

The model is Khronos' "DamagedHelmet" sample. The cited repository's README
states, verbatim:

    "Battle Damaged Sci-fi Helmet - PBR by theblueturtle_, published under a
     Creative Commons Attribution-NonCommercial license"

Khronos' current `glTF-Sample-Assets` records the lineage as two layers with
two different grants:

    (c) 2016, theblueturtle_  — CC BY-NC 4.0  — earlier version of the model
    (c) 2018, ctxwing         — CC BY 4.0     — rebuild and conversion to glTF

**CC BY-NC forbids commercial use. It cannot be sublicensed under Apache-2.0
and cannot travel inside an Apache-2.0 distribution.** This is the same defect
that removed `projects/objects/factory/fire_extinguisher/protos/FireExtinguisher.proto`
(CC BY-NC, original model by 3DHaupt) in `65d566cb8`.

Which of the two layers our copy derived from was **not determinable from this
tree**: the assets were an OBJ plus six JPEG maps carrying no metadata of any
kind — verified by walking every JPEG marker segment and PNG ancillary chunk,
which found nothing but bare `JFIF` APP0 headers (no EXIF, no XMP, no comment
block, no text chunk, no author, no date, no source URL). The OBJ's only header
was `o Helmet_0`. The history is a squashed import, so git could not corroborate
anything earlier. The citation the PROTO itself carried named the **older,
CC BY-NC** layer.

There was no reading of the evidence under which the shipped state was correct:
on the favourable reading (the ctxwing CC BY 4.0 rebuild) the tree was still
non-compliant, because CC BY 4.0 §3(a) requires retaining the creator's
attribution and a licence link, and neither appeared anywhere in this
repository.

### `Telephone.proto` — same defect, licence undeterminable

Investigated to the same standard and found in the same state:

* the **identical** `# license: MIT` header and OSI MIT URL;
* the **identical** citation to `KhronosGroup/glTF-WebGL-PBR`, so the same
  code-licence-vs-asset-licence error applies to it verbatim;
* geometry inlined as an `IndexedFaceSet` in the PROTO — an import artefact,
  carrying no author, no source and no date;
* five JPEG maps and one PNG icon, each checked the same way, each carrying
  **no metadata at all** — bare `JFIF` APP0 headers and no PNG text chunks;
* no `LICENSE`, `NOTICE`, `PROVENANCE` or attribution file anywhere in
  `projects/objects/telephone/`.

Its actual licence is therefore **undeterminable from this tree**, and the one
claim it did make — MIT — is unsupported by its own citation. An asset whose
licence cannot be established cannot be redistributed under Apache-2.0, so it
was removed alongside `Helmet.proto` rather than left to ship on an unverified
header.

Note that a prior sweep had already deleted the neighbouring
`OfficeTelephone.proto` from that same directory as Webots-licensed
(`b3038e3ae`); `Telephone.proto` survived only because its header claimed MIT.

### What replaced them

Their sole consumer was
[`../worlds/physically_based_rendering.omniworld`](../worlds/physically_based_rendering.omniworld),
where they were the only objects demonstrating a **complete** PBR texture map
set — the four `SphereGrid` instances sweep scalar metalness/roughness with
every map explicitly `NULL`.

That demonstration is preserved by `PbrMaterialSpecimen.proto` (above), which
puts a full map set — base colour, roughness, metalness, normal, occlusion — on
primitive geometry, using appearance PROTOs from `projects/appearances/protos/`
that are already Apache-2.0 in this tree. The world now instantiates two: a
`HammeredCopper` sphere for curved-surface response and a `RustyMetal` box for
flat faces, where normal and occlusion maps read most clearly.

Nothing else in the tree referenced either PROTO. In particular,
`projects/samples/demos/protos/ConstructionSiteEnvironment.proto` contains a
`DEF HELMET` appearance, but that is an own-authored yellow hard-hat built from
a `Cylinder` primitive and is **unrelated** — it declares no EXTERNPROTO for
`Helmet.proto` and never did.

This record supersedes `LICENSE-Helmet.txt`, which described the defect while it
was still unresolved and was removed with the asset it covered.
