# Provenance — binary assets under `scripts/packaging/`

**Status: RESOLVED.** 3 tracked binary assets: 1 inherited from upstream Webots,
2 OmniLink brand marks.

Origins were read out of git per file (`git log --diff-filter=A --follow`), and
for the rebranded files the *current bytes* were compared by blob hash — a
rename plus a content replacement reads as neither on its own.

## 1. Inherited from upstream Webots — 1 file

Apache License, Version 2.0 — Copyright 1996-2024 Cyberbotics Ltd.
<https://www.apache.org/licenses/LICENSE-2.0>

* `MacOSXBackground.png` — the macOS disk-image installer backdrop, arriving in
  the squashed import commit `0db6a18a74ba16fa2c10f744423405d153b87c7a` and
  unmodified since.

Upstream is <https://github.com/cyberbotics/webots>, whose `LICENSE` reads
verbatim (fetched 2026-08-22 from
<https://raw.githubusercontent.com/cyberbotics/webots/master/LICENSE>):

```
Apache License
Version 2.0, January 2004
http://www.apache.org/licenses/
```

The derivation of this fork is recorded in the repository-root `NOTICE`.

⚠ It still carries Webots' installer artwork and name. That is a **branding**
question for the macOS packaging lane, not a licensing one — the Apache grant
covers redistributing and modifying it. Flagged here because a reader auditing
brand consistency will want to know.

## 2. OmniLink brand marks — 2 files

Copyright 2026 OmniLink.

* `omnisim.png` — application icon
* `omnisim_doc.png` — document icon

Both are the **OmniLink dot-sphere orb**. They were added at import as
`webots.png` (7,662 bytes) and `webots_doc.png` (9,991 bytes), replaced with the
orb by `a0859d4d7` *"branding: OmniLink dot-sphere orb as canonical OmniSim
identity"* (26,533 bytes each), and renamed by `88f98e169`. Their bytes are
identical to `resources/images/omnisim.png`,
`resources/icons/core/omnisim_doc.png` and
`resources/web/streaming_viewer/omnisim_icon.png`.

⚠ **Trademark-reserved.** The Apache-2.0 grant covers the code, not the marks.
See `TRADEMARKS.md`. This is a trademark reservation, not a missing licence.

## Adding a new asset under `scripts/packaging/`

This file confers licence coverage on everything beneath it, so the provenance
gate will not prompt you — honour the rule anyway. Anything obtained from
elsewhere must be recorded in section 3 with its source URL, fetch date and
licence, or not added.

## 3. Third-party assets obtained from elsewhere

None.

---
See `docs/developer/asset-provenance.md` for the tree-wide media ledger.
