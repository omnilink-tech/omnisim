# Provenance — binary assets under `docs/`

**Status: RESOLVED.** Every tracked binary asset under `docs/` has an
established origin and terms. There are two populations and they are separated
below by measurement, not by assumption.

Scope: the 508 tracked files under `docs/` with a redistributable binary
extension (`.png`, `.jpg`, `.jpeg`, `.ttf`, …) as enumerated by
`tests/sources/test_asset_provenance.py`. Prose, Markdown and CSS are covered by
the repository-root `LICENSE`.

## How the split was measured

Not by eye. For each asset, the commit that **added** it was read out of git:

```bash
git log --diff-filter=A --follow --format='%H' -- <path>
```

The repository begins with one squashed import commit,
`0db6a18a74ba16fa2c10f744423405d153b87c7a` — *"Initial commit: OmniSim robotics
simulator"*, 2026-04-11 — which carries the whole upstream Webots tree. An asset
added by that commit is inherited; an asset added by any later commit was
authored in this repository. `--follow` is required: four rebrand commits
renamed files, and without it a rename reads as an addition.

| population | count |
|---|---:|
| added by the squashed Webots import | **480** |
| added by this repository's own commits | **28** |
| **total** | **508** |

## 1. Inherited from upstream Webots — 480 files

Apache License, Version 2.0 — Copyright 1996-2024 Cyberbotics Ltd.
<https://www.apache.org/licenses/LICENSE-2.0>

Upstream is <https://github.com/cyberbotics/webots>, whose `LICENSE` file reads
verbatim (fetched 2026-08-22 from
<https://raw.githubusercontent.com/cyberbotics/webots/master/LICENSE>):

```
Apache License
Version 2.0, January 2004
http://www.apache.org/licenses/

TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION
```

These are the figures and screenshots of the Webots documentation — node
diagrams, sample-world stills, sensor and actuator illustrations, and the CSS
image set. The derivation of this fork from that repository is recorded in the
repository-root `NOTICE`, and the Apache-2.0 grant covers redistribution and
modification of the documentation assets along with the rest of the work.

Distribution by directory:

| count | directory |
|---:|---|
| 224 | `docs/guide/images/samples/` |
| 111 | `docs/guide/images/` |
| 92 | `docs/reference/images/` |
| 17 | `docs/guide/images/sensors/` |
| 12 | `docs/guide/images/actuators/` |
| 10 | `docs/css/images/` |
| 6 | `docs/guide/images/humans/` |
| 2 | `docs/guide/images/` (renamed in the rebrand: `pycharm_omnisim.png`, `pycharm_omnisim.thumbnail.jpg` — added at import as their `pycharm_webots*` names, content unchanged) |
| 6 | `docs/css/fonts/` — **not covered by the above; see below** |

### `docs/css/fonts/` — separately and correctly licensed already

The six webfonts are **not** Cyberbotics' work and are not covered by the
Apache grant above. They carry their own licence files in that directory and
those govern:

* `Raleway-Thin.ttf` — SIL Open Font License 1.1, `LICENSE-Raleway-OFL-1.1.txt`
* `Roboto-{Regular,Bold,Italic,BoldItalic,Mono}.ttf` — Apache License 2.0,
  `LICENSE-Roboto-Apache-2.0.txt`

Those files predate this record and are left exactly as they are. This entry
exists so that a future reader does not fold the fonts into the Cyberbotics
grant, which would be wrong for Raleway.

## 2. Authored in this repository — 32 files

Copyright 2026 OmniLink. Apache License, Version 2.0, i.e. the repository's own
`LICENSE`. These are renders, plots and screenshots produced by this project's
own simulator, benchmarks and analysis scripts.

| count | directory | what they are | added by |
|---:|---|---|---|
| 13 | `docs/developer/shadowing_paper/figs/` | Shadowing-method paper figures: architecture, generality, learnability, throughput plots, and G1/Go2/OmniQuad/B2 stills rendered in OmniSim | `e1f23c40f`, `b3038e3ae` |
| 9 | `docs/media/screenshots/` | OmniSim screenshots of the Husky maze and warehouse demo worlds | `47234849f` |
| 4 | `docs/media/videos/` | `cuda_showcase.{gif,mp4}` and `omniarm6_real_pick.{gif,mp4}` — captures rendered by OmniSim from worlds and controllers in this repository | `4228ce0b0`, public-beta launch |
| 2 | `docs/paper/figs/` | `fig_learnability.png`, `fig_scatter.png` | `22998d566` |
| 2 | `docs/guide/images/samples/` | `omnisim_box.png`, `omnisim_box.thumbnail.jpg` — an OmniSim render of this repo's own sample box world, sitting among the inherited sample stills | `28a0cd99b` |
| 1 | `docs/developer/baton_paper/figs/` | `fig_arch.png` | `7356cd9d9` |
| 1 | `docs/developer/img/` | `vc_milestone_profile.png` | `15a661038` |

## Adding a new image to `docs/`

This file confers licence coverage on everything beneath `docs/`, so the
provenance gate will not prompt you. That makes the rule a convention rather
than an enforced check, so honour it:

* A screenshot or plot **you** produced from this repository needs nothing —
  it is already described by section 2, and the git history records it.
* Anything obtained from elsewhere — a vendor render, a photograph, a figure
  from a paper, a diagram from another project — **must not** be dropped in
  silently. Add it to section 3 of this file with its source URL, the date you
  fetched it, and its licence, or do not add it at all.

## 3. Third-party images obtained from elsewhere

None. No image under `docs/` is known to originate from any source other than
the two populations above.

---
See `docs/developer/asset-provenance.md` for the tree-wide media ledger and
`tests/sources/test_asset_provenance.py` for the gate that enumerates coverage.
