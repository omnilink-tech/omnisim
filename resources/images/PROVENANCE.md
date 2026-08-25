# Provenance — binary assets under `resources/images/`

**Status: RESOLVED.** 6 tracked binary assets: 4 inherited from upstream Webots,
2 produced in this repository.

Origins were read out of git per file (`git log --diff-filter=A --follow`), and
for the rebranded file the *current bytes* were compared by blob hash — a rename
plus a content replacement reads as neither on its own.

## 1. Inherited from upstream Webots — 4 files

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

* `missing_texture.png` — the placeholder drawn when a texture fails to load
* `missing_proto_icon.png` — the placeholder drawn for a PROTO with no icon
* `themes/dusk.png` — theme preview
* `themes/night.png` — theme preview

## 2. Produced in this repository — 2 files

| file | terms | note |
|---|---|---|
| `omnisim.png` | Copyright 2026 OmniLink | The **OmniLink dot-sphere orb** brand mark, used as the application image. Added at import as `webots.png` (65,896 bytes) and replaced with the orb by `a0859d4d7` *"branding: OmniLink dot-sphere orb as canonical OmniSim identity"* (26,533 bytes), then renamed by `88f98e169`. Byte-identical to `resources/icons/core/omnisim_doc.png`, `resources/web/streaming_viewer/omnisim_icon.png` and both `scripts/packaging/omnisim*.png`. ⚠ **Trademark-reserved:** the Apache grant covers the code, not the marks — see `TRADEMARKS.md`. |
| `themes/classic.png` | Apache-2.0, Copyright 2026 OmniLink | theme preview re-rendered in this repository, added by `0c93de28f` |

## Adding a new asset under `resources/images/`

This file confers licence coverage on everything beneath it, so the provenance
gate will not prompt you — honour the rule anyway. Anything obtained from
elsewhere must be recorded in section 3 with its source URL, fetch date and
licence, or not added.

## 3. Third-party assets obtained from elsewhere

None.

---
See `docs/developer/asset-provenance.md` for the tree-wide media ledger.
