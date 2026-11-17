# Beauty Bench rendering comparisons

`projects/samples/demos/worlds/rendering/beauty_bench.omniworld` is the fixed
house reference. Keep its assets, light rig, exposure and cameras constant when
judging renderer changes. `beauty_bench_realism.omniworld` is the optional content
study: rounded mailbox/table edges, clearer alpha-blended glazing, and a robot
model that can be moved through the scene. These are content changes; they do
not constitute a path-traced or photorealistic rendering mode.

The separate **File → Render Photo…** option now provides a native path-traced
still capture for either world. See [photo rendering](../../docs/guide/photo-rendering.md)
for controls, automation and limitations.

## Repeatable captures

The three poses in `beauty_bench_views.json` cover the exterior, interior/window,
and patio glass/metal. They were read from the harness after explicit camera
framing. `render_ab.py` makes temporary `.omniworld` files next to the source so
relative texture paths keep resolving; it removes them after capture. Source
worlds are untouched. Both arms use the same pose and expose binary and world
SHA-256 hashes in the report.

```sh
python projects/policies/common/env_fingerprint.py
python scripts/dev/render_ab.py \
  --world projects/samples/demos/worlds/rendering/beauty_bench.omniworld \
  --views tests/rendering/beauty_bench_views.json \
  --arm-a OMNISIM_BINARY=/absolute/path/to/before/omnisim-bin \
  --arm-b OMNISIM_BINARY=/absolute/path/to/after/omnisim-bin \
  --perf --out-dir outputs/beauty-bench --json outputs/beauty-bench/results.json
```

Use the actual platform binary paths (Windows: `.exe`). For a single baseline,
omit both arms and add `--no-diff`. `--noise-floor` compares arm A with itself.
`DIFFERS` exits 1: it reports a visual change, not whether that change looks better.
Check the images, errors, frame dimensions and noise floor before interpreting a
pixel difference. A missing frame or engine error also fails a baseline-only run.
Rendering time is a sample from the engine's report, not total simulation speed.

For a strict pixel comparison, disable temporal/display variation in **both**
arms with `OMNISIM_WGPU_TAA=0`, `OMNISIM_WGPU_CAMFX=0`, and
`OMNISIM_WGPU_AUTOEXP=0`. For appearance review, keep the shipped defaults and
let the bake and temporal history settle. More rays or camera post-effects alone
are not a definition of photorealism.

## Motion checks in the content study

Load `beauty_bench_realism.omniworld` with the light harness. After edits, use
`POST /world/sync`. The canonical sun marker is hidden by default, with a fixed
position matching the reference sun direction.

`MOTION_ROBOT` is deliberately **kinematic**: it has no collision hull, controller,
or Physics node. Moving it checks shadows/reflections/temporal artifacts and says
nothing about traction, balance or navigation. For example:

```json
{"def":"MOTION_ROBOT","translation":[2.8,-6.3,0.28]}
```

Send that to `POST /scene/set_pose`; read `/scene/node/MOTION_ROBOT` to verify the
achieved pose. Restore `[1.8,-6.3,0.28]` for matching stills. A full animated light
and camera sequence remains a separate extension of this testbed.

## Material regressions

```sh
python -m unittest discover -s tests -p test_wgpu_roughness_map.py -v
python -m unittest discover -s tests -p test_render_ab_views.py -v
g++ -std=c++17 -O2 -pthread -I src/omnisim/render \
  tests/rendering/omnilight_cube_test.cpp src/omnisim/render/OmniLight.cpp \
  -o /tmp/omnilight_cube_test
/tmp/omnilight_cube_test
```

The camera regression compares a white roughness map with its scalar reference,
then checks that an untextured smooth material differs. It fails on the original
engine, passes on the corrected one, and disables sensor draw caching while
editing the material so cache invalidation is not the test's oracle. Select a
candidate with `OMNISIM_BINARY`. The CPU cubemap regression checks that all texels
for every GPU level from 64 through 1 are present, including multiple local captures.

`python -m unittest discover -s tests -p test_local_light_shadows.py -v` verifies
that enabling local shadows darkens the real main view and moving the lamp moves
the shadow. It toggles the flag during one run, exercising atlas reallocation.
See [Photo rendering](../../docs/guide/photo-rendering.md) for the optical,
sampling and denoising checks.
