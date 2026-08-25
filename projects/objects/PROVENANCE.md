# `projects/objects/` — provenance

**The terms in this tree are NOT uniform.** Three different licences apply, split by PROTO, and
one of them (CC BY) carries an obligation that travels with redistribution. Read the table before
copying anything out of here.

This file exists because `scripts/release/publish_snapshot.sh` publishes a **squashed single
commit** — git history does not travel to the public repository, so provenance has to live in a
file next to the assets. The full audit, with the upstream URLs and the binary-metadata forensics
behind every claim here, is [`docs/developer/texture-provenance.md`](../../docs/developer/texture-provenance.md).

Sibling record for the material library these PROTOs draw on:
[`projects/appearances/protos/textures/LICENSE.txt`](../appearances/protos/textures/LICENSE.txt).

---

## What ships here

176 PROTOs across 43 category directories, 606 bundled texture images, and 165 PROTO preview
icons.

| Group | PROTOs | Bundled textures | Licence | Copyright holder |
|---|---|---|---|---|
| `buildings/` (25), `street_furniture/` (21), `garden/` (12), `trees/` (6), `traffic/` (2: `WorkBarrier`, `ParkingMeter`) | **66** | **573** | **CC BY 4.0** — attribution required, see below | Cyberbotics Ltd. |
| `traffic/StreetLight.proto`, `traffic/ControlledStreetLight.proto` | **2** | 0 (primitives + the Apache-2.0 appearance library) | **CC BY 3.0 US** — attribution required | Andrew Kator & Jennifer Legaz; Webots adaptation by Cyberbotics Ltd. |
| Re-authored and original OmniSim PROTOs, across every other category | **104** | 19 | **Apache-2.0** | OmniLink |
| `balls/FifaSoccerBall.proto`, `geometries/Rectangle.proto` | **2** | 0 | **Apache-2.0** | Cyberbotics Ltd. (header inherited unchanged) |
| `rocks/RockHD.proto`, `rocks/TerrainRock.proto` | **2** | 0 | **Apache-2.0** | OmniLink (authored here; not present at the upstream import) |
| `backgrounds/textures/night_sky/` | — | 6 | **Apache-2.0** | OmniLink (added post-import) |
| ⛔ orphans awaiting removal, see below | — | 8 | *(none — no owning PROTO)* | Cyberbotics Ltd., product-scoped |
| **total** | **176** | **606** | | |

**Zero** PROTOs in this tree declare `Copyright Cyberbotics Ltd. Licensed for use only with
Webots.`, `Attribution-NonCommercial`, or `MIT`. All three were present at the upstream import
and all three were removed. That history is the point of the next section.

---

## The licence that had to be removed, and why

238 of the 310 object PROTOs in the upstream import arrived declaring:

    # license: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
    # license url: https://cyberbotics.com/webots_assets_license

That is a **product-scoped, non-open grant**, and it gets legally weaker the more successfully
OmniSim establishes that it is *not* Webots. It cannot travel inside an Apache-2.0 distribution.
(The cited licence URL now returns **HTTP 404** on both `cyberbotics.com` and
`www.cyberbotics.com`, checked 2026-08-22, so the exact scope of the grant is not even readable.)

Commit `b3038e3ae` (2026-07-11) resolved it: 136 unused PROTOs deleted, the 99 that worlds
actually used **re-authored clean-room from primitives** with their interfaces frozen — same
PROTO name, fields, types, defaults and order, so ~460 worlds kept working with zero edits — and
1,023 texture images plus the associated icons and meshes removed. A licence line changed over
someone else's geometry would have been dishonest and would have changed nothing legally; the
bodies were rebuilt.

Two related removals happened in the same sweep and are worth recording: `coca-cola_billboard.jpg`
and `fanta_billboard.jpg` (third-party **brand advertising** shipping inside an asset library),
and the `TexturedBackground` cubemaps (attributed upstream to Ogre Media / X3Dom with **no stated
licence anywhere** — unverifiable provenance being the exact risk being removed).

**Verification that the sweep held.** Every non-PROTO asset in this tree whose import-time owner
was one of those 238 PROTOs was checked blob-by-blob against the import commit. **All 115 come
back byte-modified** — 99 preview icons (exactly the 99 PROTOs that were re-authored) plus 16
textures. They were re-rendered and redrawn from the new geometry, not carried over as renders of
Cyberbotics' geometry under a new header. **Nine files did not get swept up**, and they are the
only assets here still byte-identical to a restricted upstream original. They are listed below.

---

## ⛔ Nine orphans still to remove

Byte-identical to the upstream originals, owned upstream by a PROTO declaring the product-scoped
licence above, and **referenced by nothing** — verified by repository-wide search on each
basename and each containing directory path, including `omnisim://` absolute URLs.

| file | upstream owner | why it survived |
|---|---|---|
| `apartment_structure/protos/textures/door_base_color.jpg` | `GenericDoorAppearance.proto` | PROTO re-authored; texture left behind |
| `apartment_structure/protos/textures/door_normal.jpg` | `GenericDoorAppearance.proto` | same |
| `balls/protos/textures/pingpong_logo.jpg` | `PingPongBall.proto` | PROTO deleted; texture left behind |
| `factory/conveyors/protos/textures/corrugated_plates_base_color.jpg` | `ConveyorBelt.proto`'s directory | PROTO re-authored to use the Apache-2.0 `CorrugatedPlates` appearance instead |
| `factory/conveyors/protos/textures/corrugated_plates_metalness.jpg` | same | same |
| `factory/conveyors/protos/textures/corrugated_plates_normal.jpg` | same | same |
| `factory/conveyors/protos/textures/corrugated_plates_occlusion.jpg` | same | same |
| `factory/conveyors/protos/textures/corrugated_plates_roughness.jpg` | same | same |
| `factory/forklift/protos/meshes/forklift.mtl` | `Forklift.proto` | material sidecar for a `.obj` that was deleted |

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

Nothing breaks: each removal leaves an empty directory and no dangling reference.

⚠ **Two traps that made these hard to find, and will make them hard to re-find.**

1. The five `corrugated_plates_*.jpg` here are **NOT** the Apache-2.0 appearance textures of the
   same name — all five md5s differ from
   `projects/appearances/protos/textures/corrugated_plates/`. Matching textures to PROTOs by bare
   basename attributes them to the wrong licence. Resolve the relative url against the PROTO's
   own directory instead.
2. `door_base_color.jpg` is a **substring** of `picket_fence_with_door_base_color.jpg` and of
   `small_residential_tower_door_base_color.jpg`, both of which are live CC BY 4.0 textures in
   other categories. A naive grep reports the orphan as referenced. It is not.

⚠ **A byte-identical copy of `pingpong_logo.jpg` (blob `f407344b`) also ships from
`projects/samples/rendering/protos/textures/`, and that copy IS live** — `PingPongBallScaled.proto`
loads it. Removing the copy here does not remove the asset from the distribution. See
`docs/developer/texture-provenance.md` §6.2; that directory is outside this record's scope.

---

## Attribution — this is an obligation, not a courtesy

CC BY 4.0 §3(a) and CC BY 3.0 §4(b) require that attribution travel with redistribution. The
full, paste-ready notice blocks — creator, copyright notice, licence notice, warranty disclaimer,
source URI and a statement of what was modified — are in
[`docs/developer/texture-provenance.md`](../../docs/developer/texture-provenance.md) §7, for
`THIRD_PARTY_NOTICES.md`.

Short form, for anyone reading only this file:

* **573 textures and 66 PROTOs** under `buildings/`, `street_furniture/`, `garden/`, `trees/` and
  two `traffic/` PROTOs are **© 1996-2024 Cyberbotics Ltd., CC BY 4.0**
  (<https://creativecommons.org/licenses/by/4.0/legalcode>), from
  <https://github.com/cyberbotics/webots>. Modified by OmniLink: **header lines only** — the
  `#VRML_SIM` token became `#OMNISIM` and the `documentation url:` was repointed. The `license:`
  lines, the PROTO bodies and every bundled texture are byte-for-byte as received.
* **`StreetLight.proto` and `ControlledStreetLight.proto`** are **© 2003-2012 Andrew Kator &
  Jennifer Legaz, CC BY 3.0 US** (<https://creativecommons.org/licenses/by/3.0/legalcode>),
  originally from <http://www.katorlegaz.com/3d_models/>, adapted into PROTO form by Cyberbotics
  Ltd. `StreetLight.proto` additionally records that the adaptation "was sponsored by the CTI
  project RO2IVSim" — **keep that line**; it is part of the credit.

---

## Editing rules for this directory

* **Never rewrite a `# license:` header to make a file look permissive.** That is what makes an
  audit unsurvivable. If an asset's terms are wrong for this distribution, re-author the asset or
  delete it.
* **A re-authored PROTO must take its textures and its icon with it.** All nine orphans above
  exist because a PROTO was rebuilt or removed and its assets were not. Re-render the icon.
* **New PROTOs get `# license: Copyright 2026 OmniLink. Apache 2.0.`** and should source their
  materials from `projects/appearances/protos/` rather than bundling their own textures.
* **Before deleting anything, resolve references properly** — relative urls against the PROTO
  directory, plus a search for `omnisim://projects/objects/...` absolute urls, plus the
  template-constructed `%<= … >%` filename forms. All three are in use here.
