# Provenance — binary assets under `resources/web/`

**Status: RESOLVED.** 14 tracked binary assets: 10 inherited from upstream
Webots, 3 produced in this repository, 1 obtained from elsewhere.

⚠️ **Corrected 2026-08-24.** This file previously read "4 produced in this
repository" and listed `wrenjs.wasm` among them, describing it as carrying
"this repository's modifications" and a `Copyright 2026 OmniLink` line. **Both
halves were false.** The file was never built here: `resources/web/wwi/.gitignore`
records the fetch recipe (`curl -o wrenjs.wasm https://cyberbotics.com/wwi/R<rel>/…`)
and commit `2ff452a45` vendored the prebuilt R2025a artifacts as they came. It has
moved to section 3, and its two siblings — `wrenjs.js` and `wrenjs.data`, which
this record omitted entirely — are recorded with it.

A provenance record that over-claims authorship is worse than one that admits a
gap: it asserts a copyright we do not hold and hides an attribution we owe.

Origins were read out of git per file (`git log --diff-filter=A --follow`), and
for the rebranded files the *current bytes* were compared by blob hash, because
a rename plus a content replacement reads as neither on its own.

## 1. Inherited from upstream Webots — 10 files

Apache License, Version 2.0 — Copyright 1996-2024 Cyberbotics Ltd.
<https://www.apache.org/licenses/LICENSE-2.0>

Upstream is <https://github.com/cyberbotics/webots>, whose `LICENSE` reads
verbatim (fetched 2026-08-22 from
<https://raw.githubusercontent.com/cyberbotics/webots/master/LICENSE>):

```
Apache License
Version 2.0, January 2004
http://www.apache.org/licenses/
```

The derivation of this fork is recorded in the repository-root `NOTICE`.

| count | directory | what |
|---:|---|---|
| 9 | `resources/web/wwi/images/documentation/` | web-viewer UI documentation images |
| 1 | `resources/web/wwi/images/loading/` | loading spinner |

## 2. Produced in this repository — 3 files

| file | terms | note |
|---|---|---|
| `resources/web/streaming_viewer/omnisim_icon.png` | Copyright 2026 OmniLink | The **OmniLink dot-sphere orb** brand mark. Added at import as `webots_icon.png` (2,174 bytes) and replaced with the orb by `a0859d4d7` *"branding: OmniLink dot-sphere orb as canonical OmniSim identity"* (26,533 bytes), then renamed by `f1fbf4890`. Its bytes are identical to `resources/images/omnisim.png`, `resources/icons/core/omnisim_doc.png` and both `scripts/packaging/omnisim*.png`. ⚠ **Trademark-reserved:** the Apache grant covers the code, not the marks — see `TRADEMARKS.md`. |
| `resources/web/wwi/images/missing_proto_icon.png` | Apache-2.0, Copyright 2026 OmniLink | added by `b3038e3ae` |
| `resources/web/wwi/images/missing_texture.png` | Apache-2.0, Copyright 2026 OmniLink | added by `b3038e3ae` |

## Adding a new asset under `resources/web/`

This file confers licence coverage on everything beneath it, so the provenance
gate will not prompt you — honour the rule anyway. Anything obtained from
elsewhere must be recorded in section 3 with its source URL, fetch date and
licence, or not added.

## 3. Third-party assets obtained from elsewhere — 1 tracked asset, 3 files

Only `wrenjs.wasm` counts toward the 14 above, because `.js` and `.data` are not
in the provenance gate's `ASSET_EXTENSIONS`. All three are recorded anyway: they
are one package, they carry the same notices, and an attribution that stops at
the file the gate happens to look at is not an attribution.

| file | source | terms |
|---|---|---|
| `resources/web/wwi/wrenjs.js` | fetched from `https://cyberbotics.com/wwi/R2025a/`, vendored by `2ff452a45` (2026-05-10) | **Three separately-licensed layers.** (1) **WREN**, Apache-2.0, © 1996-2024 Cyberbotics Ltd. — the identity is evidenced by 325 distinct `_wr_*` exports (`_wr_camera_new`, `_wr_scene_*`, `_wr_material_*`); licence text at `resources/wren/LICENSE`. (2) **Emscripten's own runtime library**, compiled in and dual-licensed **MIT / University of Illinois-NCSA**, © 2010-2014 Emscripten authors. Full record: `resources/web/wwi/LICENSE-wrenjs.txt`. |
| `resources/web/wwi/wrenjs.wasm` | same | Same two layers. The binary is **fully stripped** — a byte scan for "copyright", "licence/license", "Cyberbotics", "Webots" and "emscripten" returns zero hits — so every notice it needs is supplied by the sidecar. |
| `resources/web/wwi/wrenjs.data` | same | Same, **plus 84 of Cyberbotics' WREN GLSL shaders** carried uncompressed inside the blob (52 `.frag`, 32 `.vert`, all `/resources/wren/shaders/*`, verified by 84 `#version 330 core` occurrences). ⚠️ Those shaders **no longer exist as source in this tree** — they were deleted with WREN in `976b9449d` on 2026-08-23 — so this package and its sidecar are now the only in-tree record of what they are. |

⚠️ **These three are why the `--stream` browser viewer permanently diverges in look
from the desktop view.** The engine renders wgpu; the browser viewer still renders
through this vendored WREN build. That divergence is recorded and accepted, not a
bug awaiting a fix.

---
See `docs/developer/asset-provenance.md` for the tree-wide media ledger.
