# Local shadow reuse and culling

Local point/spot shadows use the shared wgpu renderer. Each enabled light has six
256-pixel cube faces in a shared atlas. The renderer now skips the local shadow
pass when its inputs have not changed, and culls casters against each light face
when a redraw is necessary. This applies to any world using local shadows.

Reuse compares exact input bytes: light projections and atlas layout, caster
membership, geometry revisions, transforms and bounds. Mesh uploads and in-place
vertex updates receive unique revisions, including across mesh-cache instances.
Unknown revisions disable reuse. Camera movement alone does not invalidate local
shadows. Light/caster motion, geometry edits, opacity or cast-shadow changes,
insertion/removal, and atlas reallocation do. Cache state is published only after
command submission. Culling transforms frustum planes into local space, which
preserves conservative bounds under shear, mirrored and non-uniform scales.

The draw caches in the main view and camera sensors also observe geometry and
opacity edits. They refresh after an imported subtree finishes initialization;
observing only the initial child insertion can capture an incomplete subtree.
Conveyor texture scrolling does not trigger a full scene walk through these
hooks.

This is whole-atlas reuse: one moving caster invalidates all local shadow faces.
Per-light updates and separate static/dynamic layers remain future work. Sun
shadows are unchanged.

## Measured stationary scenes, 2026-09-12

[Recorded results](../benchmarks/data/local-shadow-2026-09-12.json) include both
repeats, source/binary/image hashes, timing percentiles and repeat-image checks.
Machine `9722d23d12a3`: Windows 11, 16 logical CPU cores, RTX 3060 Laptop GPU,
driver 596.36. Both arms use candidate SHA-256
`85f9f1920022787e69056ee5cbe48c9d99c679f85f35689b7e0fb7c362b7e139`.
Captures are 1896×1113 at frame 90, with the 5 FPS cap and controls below.
The results separately identify the final validation binary, whose notification
cleanup uses a Qt-free interface; its shadow algorithm is unchanged.

| Scene | Local-shadow draws/frame, before → after | CPU shadow work, median µs before → after | CPU render phase, median ms before → after |
|---|---:|---:|---:|
| House | 4,860 → 0 | 165–198 → 9–11 | 3.88–4.00 → 2.89–2.96 |
| Warehouse stress variant | 76,650 → 0 | 3,365–4,441 → 69–93 | 14.45–17.49 → 4.67–5.18 |
| City, no local shadows | 0 → 0 | 0 → 0 | 8.11–8.70 → 8.97–10.06 |

Ranges are the two run medians, not confidence intervals. All six A/B image
pairs and all six repeated-arm image pairs are byte-identical in decoded RGB.
The house and warehouse reuse their local shadow atlas on every measured warm
frame. This demonstrates savings for stationary local shadows, not a universal
frame-rate gain. The city does no local shadow work in either arm. Follow-up
controls measured 7.63/7.70 ms with both features disabled and 8.33/6.51 ms with
both enabled, with identical images. The original increase did not persist;
variation between unchanged enabled runs exceeded the original A/B difference.
These short, capped runs do not establish a city speedup or precise performance
equivalence. GPU timings and longer frame-pacing measurements remain separate work.

## Reproduce a comparison

```sh
python projects/policies/common/env_fingerprint.py
python scripts/dev/render_shadow_bench.py --binary /absolute/path/to/omnisim-bin \
  --out outputs/shadow-bench --repeats 2 --frame 120
```

Use `.exe` on Windows. The benchmark runs sequentially, alternates arm order,
discards the first 60 frames, and saves original PNGs, logs, binary/world hashes,
per-frame reports and JSON summaries. A nonzero exit indicates a missing capture,
engine error, insufficient timing samples, or an image difference exceeding a
summed-RGB tolerance of three levels per pixel. Review images and a repeated-arm
noise floor before attributing a difference to the optimization.

The scene suite holds assets and cameras fixed within each comparison:

- House: authored local lights in `beauty_bench_realism.omniworld`.
- Warehouse: a temporary copy enables shadows on the eight existing ceiling
  lights and stops the conveyor's texture animation. The animation follows
  simulation time, which is not tied to the screenshot frame counter. Geometry
  stays unchanged. This is a stress variant, not the shipped default.
- City: local-shadow-free control; its traffic controller is disabled in a
  temporary copy to hold geometry still.

Both arms use the same binary; `OMNISIM_WGPU_LOCAL_SHADOW_CACHE=0` and
`OMNISIM_WGPU_LOCAL_SHADOW_CULL=0` restore the original shadow work independently.
Both default to enabled. The harness records identical temporal/exposure and
lighting settings for both arms. It disables the GI bake to isolate the local
shadow path and caps drawing at 5 FPS (`--fps` changes the cap). This limits duty
while measuring per-frame work; it is not a maximum-throughput benchmark. No
authored world is modified.

## Read the measurements correctly

`OMNISIM_WGPU_REPORT_EVERY=1` records every frame through the existing
`OMNISIM_WGPU_REPORT` file; the default interval remains 100 frames.

- `collectUs`: CPU time collecting/refreshing scene draws.
- `renderUs`: CPU time in the main render phase, including setup, submission and
  any requested readback waits. It is not a GPU timestamp or an FPS measurement.
- `localShadowUs`: CPU cache validation and local-shadow command encoding.
- `localCandidates`: eligible casters times active cube faces, before optimization.
- `localDraws` / `localFaces`: local-shadow draws/faces actually encoded this frame.
- `localReused`: one when the previous atlas was reused.

GPU pass timings, frame pacing, and total simulation throughput require separate
measurements; do not translate a draw-count or CPU-time reduction into an FPS gain.

## Regression checks

```sh
g++ -std=c++17 -O2 -I src/omnisim/render tests/rendering/local_shadow_cache_test.cpp -o /tmp/shadow-test
/tmp/shadow-test
python -m pytest tests/test_local_shadow_cache.py tests/test_local_light_shadows.py tests/test_render_ab_views.py
```

The native test checks cache transitions and frustum conservatism against sampled
points under affine transforms. The engine test compares thirteen edit states
with culling and reuse independently enabled, in both the main view and a camera
sensor. It verifies visible responses to edits, not just agreement between two
potentially stale images. Failed engine runs preserve diagnostics under the
ignored `.local-runs/local-shadow-failure` directory.
Main-view export is queued to the GUI: the probe waits for the PNG's completion
before applying the next edit, so a delayed export cannot photograph a later state.
