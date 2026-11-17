# Moving detail: before and after

Installed commit: `affa49d62`.

OmniSim now follows moving objects and bending surfaces when it blends frames for anti-aliasing. This applies across the rendering pipeline, including robots and imported CAD shapes, cloth, soft bodies, and muscles.

| Before: camera movement only | After: object movement too |
|---|---|
| ![Before](O:/omnisim/.local-runs/motion-pipeline/before.png) | ![After](O:/omnisim/.local-runs/motion-pipeline/after.png) |

These are unscaled crops of real viewport captures at the same scripted pose. Capture timing can differ in the anti-aliasing cycle, so this is a visual comparison rather than a numerical quality benchmark.

In the controlled moving-pattern GPU test, the average error against correctly aligned frame history fell from **33.5 to 0.0** on the 0â€“255 color scale. This measures that specific test, not every scene.

- The house, warehouse, and city match the previous rendering exactly with temporal anti-aliasing disabled.
- All 12 robot-camera comparisons match exactly. Live cloth motion is also verified.
- 1,290 unit tests, 13 GPU shader tests, 8 renderer/engine checks, the C++ history checks, and the documentation/license checks pass.

Static opaque surfaces skip the extra drawing pass. Moving surfaces add work, and the motion buffer uses about 63 MiB at 1080p. GPU pass timings are recorded in the [evidence](O:/omnisim/docs/benchmarks/data/object-motion-2026-09-12.json); these capped runs do not establish an FPS improvement.

The feature is enabled by default alongside temporal anti-aliasing. `OMNISIM_WGPU_MOTION_VECTORS=0` restores the previous camera-only tracking. Glass, animated textures, and changing reflections remain areas for further work.
