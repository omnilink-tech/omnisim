# Provenance — binary assets under `tests/`

**Status: RESOLVED. Re-measured 2026-08-24: 627 tracked binary assets**, in
five lineages — **two of which are BSD-3-Clause third-party sets that this
record did not previously mention at all**:

| count | lineage | licence |
|---:|---|---|
| **550** | DAIR contact-dynamics datasets under `benchmarks/omnibench/lane1r/data/` | **BSD-3-Clause**, © 2022 Dynamic Autonomy and Intelligent Robotics Lab |
| **50** | inherited from upstream Webots as test fixtures | Apache-2.0, © 1996-2024 Cyberbotics Ltd. |
| **18** | produced by this repository's own benchmark runs | Apache-2.0, © 2026 OmniLink |
| **7** | Clearpath Husky description meshes under `benchmarks/ladder/tasks/T1_arrive/` | **BSD-3-Clause**, © 2021 Clearpath Robotics Inc. |
| **2** | frozen WREN renderer reference images | Apache-2.0, © 2026 OmniLink |

> ⚠ **This record said "68 … 50 inherited … 18 own" until 2026-08-24, and the
> total was wrong by 559 files.** The 68 was the *Apache-2.0 half* — it counted
> the two lineages that were investigated in the 2026-08-22 pass and silently
> omitted everything that arrived by a different route, including **both of the
> tree's BSD-3-Clause third-party asset sets**. Nothing was mis-licensed: all
> 559 missing files sit under a licence file of their own (`lane1r/data/LICENSE`,
> `T1_arrive/container/husky_description/LICENSE.upstream`) or under this one, so
> the provenance gate was green throughout and the terms were on disk the whole
> time. But a ledger whose headline number omits the only two BSD-3 attribution
> obligations in `tests/` is a **stale claim about authorship**, which this
> tree's own rule says is worse than no claim. Corrected by re-running the
> measurement rather than by patching the number.

These are test payloads, not product assets — but they are tracked, and the
repository is published, so they are redistributed and need a record.

## How the split was measured

The counted set is exactly the one the provenance gate counts — the extensions
in `ASSET_EXTENSIONS` in
[`sources/test_asset_provenance.py`](sources/test_asset_provenance.py) — so
these numbers mean the same thing the gate means. Re-derive the total and the
per-directory breakdown with:

```bash
python - <<'EOF'
import os, sys, collections
sys.path.insert(0, os.path.join('tests', 'sources'))
import test_asset_provenance as G
assets = [p for p in G._tracked_files()
          if p.startswith('tests/')
          and os.path.splitext(p)[1].lower() in G.ASSET_EXTENSIONS]
print(len(assets), 'assets')
for d, n in collections.Counter(os.path.dirname(p) for p in assets).most_common():
    print('%6d  %s/' % (n, d))
EOF
```

For each asset the commit that **added** it was then read out of git
(`git log --diff-filter=A --follow`). The repository begins with one squashed
import commit, `0db6a18a74ba16fa2c10f744423405d153b87c7a` — *"Initial commit:
OmniSim robotics simulator"*, 2026-04-11 — carrying the whole upstream Webots
tree.

⚠ **An add-commit later than the import means "not from the Webots import" — it
does NOT mean "authored here."** Two of the five lineages below arrived after
the import and are third-party: the DAIR datasets (`d6ad45597`, 2026-08-09) and
the Clearpath meshes (`6a342fc2e`, 2026-04-24). Git tells you the boundary; the
licence file beside the asset tells you the authorship.

## 1. Inherited Webots test fixtures — 50 files

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
| 15 | `tests/api/worlds/textures/` | API-test world textures |
| 8 | `tests/rendering/colors/` | rendering-comparison reference targets |
| 7 | `tests/cache/protos/textures/` | asset-cache PROTO fixtures |
| 4 | `tests/rendering/meshes/` | rendering-test meshes |
| 3 | `tests/cache/worlds/textures/` | asset-cache world fixtures |
| 3 | `tests/rendering/worlds/meshes/` | rendering-test world meshes |
| 2 | `tests/cache/worlds/meshes/` | asset-cache world meshes |
| 2 | `tests/protos/worlds/textures/` | PROTO-test world textures |
| 1 each | `tests/api/controllers/display/`, `tests/api/controllers/robot_time_consecutive_packets/`, `tests/api/controllers/speaker/`, `tests/physics/worlds/meshes/`, `tests/physics/worlds/textures/`, `tests/rendering/worlds/textures/` | per-controller device fixtures (Display image, Speaker sound) and physics/rendering world payloads |

## 2. DAIR contact-dynamics datasets — 550 files

**BSD 3-Clause License — Copyright (c) 2022, Dynamic Autonomy and Intelligent
Robotics Lab.** Verbatim licence text:
[`benchmarks/omnibench/lane1r/data/LICENSE`](benchmarks/omnibench/lane1r/data/LICENSE).

550 `.pt` trajectory tensors under
[`tests/benchmarks/omnibench/lane1r/data/`](benchmarks/omnibench/lane1r/data/),
added by `d6ad45597` (2026-08-09) as the real-world ground truth OmniBench
lane 1r scores the solver against.

⚠ **BSD-3 clause 2 is a live obligation on this repository.** Redistribution in
binary form must reproduce the copyright notice and the disclaimer *"in the
documentation and/or other materials provided with the distribution"*, and
clause 3 forbids using the lab's name to endorse. The in-tree `LICENSE` beside
the data discharges the first; do not move, rename or summarise it, and do not
attribute lane 1r results to DAIR.

⚠ **This is the largest single asset lineage in the repository — 550 of the 627
files under `tests/` — and it was absent from this record until 2026-08-24.**
It is also the reason the old "68" looked plausible: 68 is the count you get if
you only count what the Apache-2.0 investigation touched.

## 3. Clearpath Husky description meshes — 7 files

**BSD 3-Clause — Copyright 2021 Clearpath Robotics Inc.** Verbatim licence text:
[`benchmarks/ladder/tasks/T1_arrive/container/husky_description/LICENSE.upstream`](benchmarks/ladder/tasks/T1_arrive/container/husky_description/LICENSE.upstream).

Six `.dae` and one `.stl` under
`tests/benchmarks/ladder/tasks/T1_arrive/container/husky_description/meshes/`
(`base_link`, `bumper`, `top_chassis`, `top_plate` ×2, `user_rail`, `wheel`),
added by `6a342fc2e` (2026-04-24). They are the robot the ladder's T1 task hands
an agent, staged inside the task container so the cell is self-contained.

⚠ These have a **later add-commit than the Webots import and are still not our
work** — the case the warning above exists for.

## 4. Produced by this repository's own benchmark runs — 18 files

Copyright 2026 OmniLink. Apache License, Version 2.0.

Screenshots and plots emitted by OmniBench ladder runs — i.e. OmniSim rendering
OmniSim's own worlds, and matplotlib plotting this repository's own run logs.
They are committed as the evidence trail for a benchmark deliverable.

| count | directory | added by |
|---:|---|---|
| 14 | `tests/benchmarks/ladder/results/ladder_cell/20260804_092003_mujoco_T2_transfer/deliverable/solution/` | `39b746779` |
| 4 | `tests/benchmarks/ladder/results/ladder_cell/20260803_172036_mujoco_T2_transfer/deliverable/20260803_172041_mujoco_T2_r1/` | `1ae8f40ae` |

⚠ **PUBLISH-DENIED as of 2026-08-24 — these 18 files exist in the private tree
and are NOT in the public snapshot.** `scripts/release/publish_deny.txt` excludes
all of `tests/benchmarks/ladder/results/` (286 files, 77.4 MB of agent-cell run
residue) because five of its thirteen run directories carry transcripts and a
captured working-tree snapshot naming a robot whose publication permission is
void. The deny is by directory rather than by file so the exclusion cannot rot;
these 18 are collateral, and they are our own screenshots, so nothing is lost
but the pictures. **A reader of the public repository will not find this
directory — that is expected, not a packaging bug.** The ladder *instrument*
(`benchmarks/ladder/{adapters,cell,graders,tasks}`) ships in full, section 3
above included.

## 5. Frozen WREN renderer reference images — 2 files

Copyright 2026 OmniLink. Apache License, Version 2.0.

`tests/rendering/wren_reference/beauty_bench_wren_1280x720.png` and
`warehouse_industrial_wren_1280x720.png`, added by `f21b68c32` (2026-08-23):
OmniSim rendering OmniSim's own worlds through the WREN renderer, frozen at the
moment WREN was deleted so the wgpu look has a before-picture to diff against.
Their own [`README.md`](rendering/wren_reference/README.md) states what they are
and how they were captured.

## Note — removed 2026-08-22

`tests/api/worlds/skins/` (`simple_skin.fbx` and its `LICENSE.txt`) was deleted.
The FBX was a synthetic Skin-node fixture — Blender's parametric Cylinder
primitive plus a two-bone armature, all authorship fields empty — and nothing in
the tracked tree loaded it: there was no `Skin {` node in any `.wbt` or
`.omniworld`, and no world, controller, test, doc or script mentioned it.

It carried **no licence risk** (the Apache-2.0 chain above covered it), so this
was housekeeping, not a licensing fix. Note that the `Skin` node itself is still
live in the engine (`src/omnisim/nodes/OmSkin.cpp`, `resources/nodes/Skin.wrl`)
and this was the tree's only `.fbx`, so a future Skin test will need a new
fixture; restore it with
`git checkout <commit>^ -- tests/api/worlds/skins/` if that is easier than
authoring one.

## Adding a new asset under `tests/`

This file confers licence coverage on everything beneath `tests/`, so the
provenance gate will not prompt you — honour the rule anyway, and note that
**that blanket coverage is exactly how 557 third-party files stayed out of this
record for months**: the gate went green on this file's mere existence and never
looked at what was underneath. Fixtures you generate from this repository are
described by section 4. Anything obtained from elsewhere must be recorded in
section 6 with its source URL, fetch date and licence — **and gets its own
`LICENSE` file in its own directory**, so its terms do not depend on anyone
reading this one.

**`tests/benchmarks/` additionally pins byte-level freezes.** `agentbench`'s
`freeze_manifest.json` records SHA256 by path and a red freeze test is a release
gate, so do not rewrite an asset there without re-freezing.

## 6. Third-party assets obtained from elsewhere

**Two sets, 557 files — sections 2 and 3 above.** This heading read *"None"*
until 2026-08-24, which was the sharpest form of the same error: both BSD-3 sets
were already sitting in the tree with their licence files beside them, and this
line said they did not exist. Anything new obtained from elsewhere is recorded
here with its source URL, fetch date and licence, or it is not added.

---
See `docs/developer/asset-provenance.md` for the tree-wide media ledger.
