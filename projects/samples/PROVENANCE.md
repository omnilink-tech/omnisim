# Provenance — binary assets under `projects/samples/`

**Status: RESOLVED. Re-measured 2026-08-24: 74 tracked binary assets**, split
by measurement into **37 inherited** from upstream Webots and **37 authored** in
this repository.

> ⚠ **This record said 68 / 31 / 37 until 2026-08-24 and the inherited half was
> wrong by six files.** Four directories were missing from section 1 entirely
> (`devices/controllers/speaker/sounds/`, `devices/controllers/
> speaker_text_to_speech/sounds/`, `rendering/protos/icons/`, and the inherited
> half of `demos/worlds/rendering/textures/`), and one directory was listed in
> section 2 as holding one file when it holds two of *different* lineage. Nothing
> was mis-licensed by the gap — all six are Webots-inherited Apache-2.0 like the
> rest of section 1 — but a count that does not reconcile with the directory is
> a stale claim, and a stale claim about provenance is worse than no claim.
> Corrected by re-running the measurement below rather than by patching the
> table.

## How the split was measured

For each asset the commit that **added** it was read out of git
(`git log --diff-filter=A --follow`). The repository begins with one squashed
import commit, `0db6a18a74ba16fa2c10f744423405d153b87c7a` — *"Initial commit:
OmniSim robotics simulator"*, 2026-04-11 — carrying the whole upstream Webots
tree. Added by that commit means inherited; added by any later commit means
authored here.

The counted set is exactly the one the provenance gate counts — the extensions
in `ASSET_EXTENSIONS` in
[`tests/sources/test_asset_provenance.py`](../../tests/sources/test_asset_provenance.py)
— so these numbers mean the same thing the gate means. Re-derive with:

```bash
python - <<'EOF'
import os, subprocess, sys, collections
sys.path.insert(0, os.path.join('tests', 'sources'))
import test_asset_provenance as G
IMPORT = '0db6a18a74ba16fa2c10f744423405d153b87c7a'
assets = sorted(p for p in G._tracked_files()
                if p.startswith('projects/samples/')
                and os.path.splitext(p)[1].lower() in G.ASSET_EXTENSIONS)
for p in assets:
    log = subprocess.run(['git', 'log', '--diff-filter=A', '--follow', '--format=%H', '--', p],
                         capture_output=True, text=True).stdout.split()
    print('inherited' if log[-1] == IMPORT else 'authored ', p)
EOF
```

## 1. Inherited from upstream Webots — 37 files

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
| 13 | `projects/samples/devices/worlds/textures/` | device sample-world textures |
| 5 | `projects/samples/demos/worlds/textures/gantry/` | gantry demo textures |
| 4 | `projects/samples/demos/worlds/textures/soccer/` | soccer demo textures |
| 3 | `projects/samples/devices/worlds/meshes/` | device sample meshes |
| 2 | `projects/samples/demos/worlds/textures/moon/` | moon demo textures |
| 2 | `projects/samples/geometries/worlds/textures/` | geometry sample textures (`compact_mapping.jpg`, `cube_mapping.jpg`) |
| 2 | `projects/samples/devices/controllers/speaker/sounds/` | ⚠ `robot_bip.wav`, `robot_sound.wav` — **PARTIALLY EVIDENCED**, see below |
| 2 | `projects/samples/rendering/protos/icons/` | PROTO preview icons (`PingPongBallScaled.png`, `SphereGrid.png`) |
| 1 | `projects/samples/devices/controllers/speaker_text_to_speech/sounds/` | ⚠ `cow.wav` — **UNRESOLVED**, see below |
| 1 | `projects/samples/demos/worlds/rendering/textures/` | `color_checker_chart.png` |
| 1 | `projects/samples/devices/controllers/display/` | Display-device test image |
| 1 | `projects/samples/geometries/controllers/water_flow_animation/` | animation frame source |

### ⚠ Three of these 37 are weaker than the rest, and say so in-place

The four `.wav` files above do **not** rest on the blanket Apache-2.0 grant the
way the images do, and each of the two directories carries its own `LICENSE.txt`
recording exactly how far the evidence goes and what would settle it:

* [`devices/controllers/speaker/sounds/LICENSE.txt`](devices/controllers/speaker/sounds/LICENSE.txt)
  — **PARTIALLY EVIDENCED.** `robot_bip.wav`'s RIFF `LIST/INFO` names only tools
  (`ITCH` "LAME in FL Studio 9", `ISFT` FFmpeg, `ICRD` 2012); `robot_sound.wav`
  has an empty ID3v2 `TXXX` Software frame. **Neither names an author**, so
  Cyberbotics authorship is *inferred* from the upstream grant plus the absence
  of any competing claim, not established.
* [`devices/controllers/speaker_text_to_speech/sounds/LICENSE.txt`](devices/controllers/speaker_text_to_speech/sounds/LICENSE.txt)
  — **UNRESOLVED.** `cow.wav` carries only `fmt ` and `data` chunks: no metadata
  of any kind. It is the weakest audio provenance in the tree.

Those files govern; this table only counts. Do not upgrade "inferred" to
"established" by reading the count here.

## 2. Authored in this repository — 37 files

Copyright 2026 OmniLink. Apache License, Version 2.0 — the repository's own
`LICENSE`.

| count | directory | what |
|---:|---|---|
| 14 | `projects/samples/demos/worlds/physics/textures/` | textures authored for the Newton physics demo worlds |
| 9 | `projects/samples/demos/worlds/physics/meshes/` | meshes authored for the Newton physics demo worlds |
| 8 | `projects/samples/demos/protos/icons/` | PROTO preview icons rendered by this repo's icon studio (`scripts/icon_studio/`) from this repo's own PROTOs |
| 2 | `projects/samples/demos/controllers/smart_house_bridge/docs/` | smart-house bridge documentation images |
| 2 | `projects/samples/protogen/protos/icons/` | PROTO preview icons, same pipeline |
| 1 | `projects/samples/demos/worlds/rendering/textures/` | `grass_clump.png`, added `e1ea5623c` (2026-08-21) |
| 1 | `projects/samples/geometries/worlds/textures/` | `omnisim_box.png`, added `28a0cd99b` (2026-05-25) |

⚠ **Two directories are SPLIT between sections 1 and 2 — read them together.**
`demos/worlds/rendering/textures/` holds 2 files (`color_checker_chart.png`
inherited, `grass_clump.png` ours) and `geometries/worlds/textures/` holds 3
(two inherited `.jpg` mappings, one `omnisim_box.png` of ours). A per-directory
count in either table alone is therefore not the directory's file count. This
is what the old record got wrong: it listed each of those directories in one
section only.

## Note — `projects/default/worlds/meshes/torus_knot.obj`

The geometry sample world
`projects/samples/geometries/worlds/geometric_primitives.omniworld` loads its
`Mesh` from `projects/default/worlds/meshes/`, which carries its own
`LICENSE.txt`. That mesh is generated by a committed script
(`scripts/dev/gen_mesh_fixtures.py`) and is this project's own work. It replaced
a verbatim export of Blender's GPL-sourced "Suzanne" primitive on 2026-08-22;
the full reasoning is in that directory's `LICENSE.txt`. Do not re-introduce a
Blender built-in primitive here.

## Adding a new asset under `projects/samples/`

This file confers licence coverage on everything beneath it, so the provenance
gate will not prompt you — honour the rule anyway. Assets you produce from this
repository are already described by section 2. Anything obtained from elsewhere
must be recorded in section 3 with its source URL, fetch date and licence, or
not added.

## 3. Third-party assets obtained from elsewhere

None.

---
See `docs/developer/asset-provenance.md` for the tree-wide media ledger.
