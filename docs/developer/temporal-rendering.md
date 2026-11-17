# Temporal rendering and GPU diagnostics

The live HDR viewport now validates temporal history against the previous
frame's depth. Each of the four history-filter taps must describe the expected
surface; uncovered backgrounds no longer inherit an unrelated foreground's color.
The resolve uses the same jittered projection that produced its depth buffer,
checks the nearest MSAA depth sample at silhouettes, and reduces history weight
when color changes. Invalid history, sky without finite depth, and off-screen
reprojection use the current image. Resize and failed resolves invalidate history.

Object motion now supplements camera/depth reprojection. Shape and CadShape draws
retain their previous model transform; Cloth, SoftBody, and Muscle draws retain
previous vertex positions as well. Each render target owns its history. Stable
node-lifetime serials distinguish instances even when a native address, engine node
ID, or draw-list position is reused. A CadShape's submeshes have separate entries.
Removed, hidden, newly created, and unexpectedly changed geometry reject history.
Deformation requires matching vertex counts and the full index correspondence;
topology changes reject history instead of treating unrelated vertices as motion.
Robot camera sensors retain their existing unjittered path; the change is not
applied to measurement pixels.

Only moving or history-invalid draws need the extra geometry pass. Static opaque
surfaces keep camera reprojection; when there are no such draws, a tiny dummy
texture avoids a full-screen motion clear. The pass uses the lit depth buffer
read-only, the same transform operation order, and the same albedo alpha test.
Both vertex outputs declare [WGSL position invariance](https://www.w3.org/TR/WGSL/#invariant)
so independent shader optimization cannot break the equal-depth comparison.
This invariant lit-vertex variant is restricted to the motion-enabled HDR
viewport; sensors and TAA-off/legacy arms keep their original vertex program.
Motion remains multisampled: the temporal resolve selects the velocity belonging
to the nearest depth sample rather than averaging foreground and background.
Each valid sample carries previous image coordinates and the expected previous
surface depth, so an object moving toward the camera can still accumulate detail.

Live glass has no single surface correspondence. Its visible coverage rejects
history, as do particle/Track draws without persistent per-instance identity.
Animated textures, changing reflections, and illumination still rely on color
rejection rather than dedicated velocities. Screen-space effects mixed into the
image can therefore retain residual artifacts; motion tracking is not a complete
temporal reconstruction/upscaling system.

Scene-buffer growth also explicitly invalidates dependent GPU bindings. Native
handle addresses can be recycled after release; comparing addresses alone could
keep an older, smaller storage buffer bound and silently hide newly inserted
objects. Repeated insertion checks cover this in both the viewport and sensors.

`OMNISIM_WGPU_TAA_VALIDATION=0` selects the former unchecked history blend and
unjittered reprojection for comparison. Validation defaults on. This comparison
arm still allocates/writes the new depth histories, so it isolates the resolve's
behavior rather than reproducing the old memory cost.
`OMNISIM_WGPU_TAA=0` disables the entire temporal path as before. The extra history
storage is two R32Float images (8 bytes per viewport pixel, about 16 MiB at 1080p),
allocated only when temporal anti-aliasing runs.

`OMNISIM_WGPU_MOTION_VECTORS=0` keeps validated camera-only reprojection for
comparison. Object motion defaults on and requires both TAA and validation.
The RGBA16Float, four-sample motion attachment costs 32 bytes per viewport pixel
(about 63 MiB at 1080p), allocated on the first frame that needs it and retained
until target destruction. Deforming surfaces also retain CPU positions/indices
and upload previous positions when they change. Rigid motion needs matrices only.
No motion pass, attachment, or snapshots are used by the TAA-off/sensor paths.

## GPU timing

Set `OMNISIM_WGPU_GPU_TIMING` to an absolute output-file path before launching
OmniSim. Unset it or set it to `0` to disable profiling. The adapter's timestamp
feature is requested only when profiling is enabled and supported; an unsupported
adapter keeps rendering and emits `status=unavailable` rather than zero timings.

Each render target owns a three-slot query/readback ring. Results are mapped
asynchronously; a full ring skips a sample instead of waiting for the GPU.
Callback state owns its readback lifetime independently of the render target.
Timestamp ticks are converted with the queue's timestamp period, as required by
[wgpu's timestamp API](https://docs.rs/wgpu/latest/wgpu/enum.QueryType.html).

Rows identify `target`, target-local `frame`, `width`, and `height`. They contain:

- `gpuSpanUs`: earliest measured render-pass start to latest measured pass end.
  It excludes presentation, readback transfer, physics, and CPU work. Sky-LUT
  updates and work in other encoders are not included. It is not a complete frame
  time or an FPS estimate.
- `sceneUs`, `sunShadowUs`, `localShadowUs`, `ssrUs`, `volumeUs`, `exposureUs`,
  `tonemapUs`, `aoDepthUs`, `aoUs`, `bloomUs`, `motionUs`, and `taaUs` when those passes run.
  Repeated passes in a group are summed; groups should not be summed to infer FPS.
- `cpuIntervalUs`: wall-clock interval between consecutive render starts for that
  target, recorded independently from GPU timestamps. This reflects requested
  drawing rate and caller scheduling, not actual display presentation intervals.

Results arrive on later frames but retain the measured frame ID. Different
targets, including synchronous camera-sensor renders, must not be pooled.
`scripts.dev.render_ab.summarize_gpu_profile` reports p50/p95 per target, omits
warm-up, deduplicates frame IDs, and rejects invalid measurements. Use a fresh
file per run; target IDs restart with each process. Existing `renderUs` telemetry
remains a CPU submission measurement.

## Reproduction and evidence

`scripts/dev/render_temporal_bench.py --binary <candidate> --baseline <previous>
--out <directory>` runs bounded, sequential house, warehouse, and city comparisons.
It captures object motion versus validated camera-only and legacy temporal resolve,
TAA off, timing off, and an
optional previous-binary image. Source worlds are unchanged; temporary variants
reuse the stationary shadow benchmark's existing lights and camera views.
The default is 96 render frames at a requested 5 FPS, with 60 warm-up frames.
These stationary captures are visual controls, not a moving-scene quality score.

`python -m pytest tests/rendering/test_taa_validation_gpu.py -q` executes the live
WGSL shader on controlled current/history images and depth, covering disocclusion,
coplanar color changes, retained stable accumulation, initial history, sky, and
off-screen camera cuts, rigid motion, previous surface depth, and selection of the
correct MSAA motion sample. `tests/rendering/test_object_motion_gpu.py` executes
the production motion raster shader for rigid and deforming triangles, cutout
textures, occlusion, and transparent/new surfaces. This GPU lane requires `wgpu`,
NumPy, and a working adapter. `tests/rendering/motion_history_test.cpp` is an
engine-free C++17 executable covering object identity, draw reordering, topology,
history resets, and owned deformation snapshots (`-Isrc/omnisim/render`).

`tests/test_object_motion_scene.py` runs bounded translating/rotating object,
camera, insertion/removal, and geometry edits through the actual engine. It verifies
motion-pass timestamps when available and exact sensor-image parity between the
comparison arms. Its moving captures are visual evidence; asynchronous viewport
capture does not supply a frame-synchronized quality score.
The same test module runs the existing cloth/sensor world with the viewport
matched to the sensor's proven pose, checking live deformation and motion timings.
`tests/test_local_shadow_cache.py` also verifies profiling with main-view and
sensor readback while editing geometry and lights; profiling must not change pixels.

The [September 12 evidence](../benchmarks/data/temporal-rendering-2026-09-12.json)
records the candidate hash, machine, images' comparison statistics, shader checks,
and timing scope. Other simulator jobs were active during the scene comparisons;
the GPU timings are diagnostic evidence, not a before/after speedup claim.

The [object-motion evidence](../benchmarks/data/object-motion-2026-09-12.json)
records the subsequent motion phase, final binary hashes, exact TAA-off controls
for all three scenes, 12 unchanged sensor comparisons, live cloth checks, and
GPU timings. In the controlled one-pixel moving-pattern fixture, mean channel
error against correctly aligned history falls from 33.5 to 0 on the 0–255 scale.
This is a history-alignment test, not a photorealism score or an FPS improvement.
