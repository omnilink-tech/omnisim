# Render a photo

Choose **File → Render Photo…** to make a still image of the current view with
OmniSim's optional CPU path tracer. Choose the image size, detail and maximum
render time, then save a PNG. The simulation pauses during the capture and resumes
afterwards. **Cancel** stops the render without replacing an existing output file.

Normal interactive rendering and robot camera output continue to use wgpu. Photo
rendering is an offline still-image option; it does not switch simulation sensors
or claim real-time performance comparable to Unreal.

## What changes in the image

The photo renderer follows rays through the actual scene triangles. It supports
textured materials, roughness and metalness maps, normal maps, vegetation cutouts,
reflections of objects outside the camera view, indirect diffuse light, emissive
surfaces, soft sun shadows, and occlusion of point and spot lights. The camera and
lighting come from the current world. Output uses an AgX display transform.

The sampler explicitly targets bright sky regions and emissive triangles, combines
these samples with material samples using multiple importance sampling, and stops
sampling quiet pixels after at least 32 samples. **Reduce grain** uses Intel Open
Image Denoise on the CPU, guided by sampled surface color and normals. Raw linear
pixels remain intact internally. If the optional runtime is missing or cannot
finish within the remaining budget, an edge-aware filter is used and the capture
report names the fallback. Turn grain reduction off for unfiltered output.

Existing transparent materials retain thin-sheet transmission. To enable solid
glass, use closed geometry with outward-facing triangles and a material such as:

```vrml
PBRAppearance {
  metalness 0
  transparency 1
  refraction TRUE
  indexOfRefraction 1.5
  attenuationColor 0.8 0.95 0.9
  attenuationDistance 1
}
```

Photo rays follow Snell refraction, Fresnel reflection and total internal
reflection. Absorption depends on the actual distance travelled through the
solid: `attenuationColor` is the transmitted fraction after
`attenuationDistance` metres. White gives clear glass. Properly nested closed
solids can form multiple media. The dielectric interface is smooth; rough glass,
dispersion and focused caustics are not implemented. Direct-light visibility
through glass remains an approximation. The live view retains alpha-blended
glass. The photo renderer also does not include
motion blur, depth of field, participating media, the procedural
cloud layer, Pen paint overlays or specialized cloth sheen. More detail reduces
sampling grain; it cannot compensate for missing geometric or material detail.

The renderer uses one CPU worker and a finite time budget. When the budget expires,
it saves the completed samples, if any, and reports that the time limit was reached.
If it cannot complete even one sample, it reports failure without saving an image.
Each capture also writes a `.png.json` report recording minimum and average sample
counts, duration, denoiser, texture count and triangle count. With adaptive
sampling, pixels may have different sample counts. A scene snapshot owns its geometry and
pixels, so rendering does not access live simulation nodes from the worker.

## Automated still capture

The same render path can run once at startup, then exit. Set
`OMNISIM_PHOTO_OUTPUT` to an absolute PNG path in an existing directory and launch
the world with `--mode=realtime` and rendering enabled. Capture starts after the
first simulation step so startup controllers can finish setting the scene.
Optional settings are `OMNISIM_PHOTO_WIDTH`
(960), `OMNISIM_PHOTO_HEIGHT` (540), `OMNISIM_PHOTO_SAMPLES` (64), and
`OMNISIM_PHOTO_SECONDS` (60). `OMNISIM_PHOTO_DENOISE=0` disables grain reduction.
`OMNISIM_PHOTO_LIGHT_SAMPLING=0` and `OMNISIM_PHOTO_ADAPTIVE=0` disable the
respective sampling improvements for controlled comparisons.
The parentheses show defaults. Invalid settings fail
the capture. Do not use `--no-rendering`: the renderer needs the loaded visual
assets and current viewpoint.

Use the [Beauty Bench](../../tests/rendering/BEAUTY_BENCH.md) views for comparisons.
Compare the same camera, scene and dimensions, and report the actual samples from
the sidecar. Pixel differences between this path tracer and the interactive
renderer are expected; judge lighting, geometry and material behavior in the image.

## Live reflections and lights

`Background.reflectionProbePositions` (also exposed by `OmniSimSky`) accepts up to
three local capture positions, in world metres, in addition to the scene-wide
capture. Place captures in open space inside rooms or beside reflective outdoor
objects. Captures blend with spatial bounds and box parallax correction. Their
full 64-to-1 mip chains use GGX prefiltering and a split-sum specular response.
They update through the existing asynchronous OmniLight bake when the light rig
or static scene changes. Moving robots do not trigger reflection rebakes.
`OMNILIGHT_THREADS` limits
the baker's CPU workers; zero retains automatic sizing.

Point and spot lights with `castShadows TRUE` now cast live shadows, including
moving lights and off-screen occluders. Up to eight local lights share a 256-pixel
per-face shadow atlas. Transparent objects do not cast these raster shadows.
Sun shadows retain their separate cascades. More shadowed lights increase render
cost; the authored flag remains opt-in.

`OmniSimAreaLight` is a rectangular emitter facing local -Z. Its `size`, `color`,
`intensity`, `translation` and `rotation` control its shape and radiance. It uses
four shadowed spot samples in the live view and the actual emitting rectangle in
photo mode, producing continuous soft shadows there. It consumes four of the
eight live local-light slots; the live penumbra is a four-sample approximation.
Its raster proxies have `rayTracing FALSE` so the photo renderer does not count
them again. `PBRAppearance.emissiveTwoSided FALSE` restricts emission to the front
face of the rectangle. Both new flags preserve prior behavior by default.

The [realism house](../../projects/samples/demos/worlds/rendering/beauty_bench_realism.omniworld)
includes local captures, solid glazing, shadowed lamps and a ceiling panel.

## Optional denoiser runtime

The Windows development bundle includes the CPU runtime from
[Open Image Denoise 2.5.1](https://github.com/RenderKit/oidn/releases/tag/v2.5.1).
To reproduce it, download `oidn-2.5.1.x64.windows.zip` from that release and verify
SHA256 `f11f91bc072a5e3a564515724cb72ab8fcfbc445c84a84c197ff9b16cc01396f`
before extracting. Create `photo-denoise` beside `omnisim-bin.exe`, then copy
`OpenImageDenoise.dll`, `OpenImageDenoise_core.dll`,
`OpenImageDenoise_device_cpu.dll`, `tbb12.dll` and the `tbbbind*.dll` files from
the archive's `bin` directory. Include its `LICENSE.txt` and third-party notices.
GPU device plugins are unnecessary. The Windows packager includes this folder
recursively and reports missing runtime files.

Linux and macOS can supply the corresponding CPU distribution beside the engine
under `photo-denoise`, with `libOpenImageDenoise.so.2` or
`libOpenImageDenoise.dylib` and its dependencies. These platforms have not been
runtime-validated for this integration. Loading is optional and uses an absolute
path; the engine still starts when the denoiser is absent.

## Validation

`tests/rendering/photo_test.cpp` checks deterministic output, cancellation, time
limits, material energy under a constant environment, texture overrides, alpha
cutouts, off-screen reflections and local-light shadowing. Compile it with
`src/omnisim/render/OmPhoto.cpp` and the `src/omnisim/render` include directory.
`tests/rendering/photo_quality_test.cpp`, also linked with `OmPhotoDenoise.cpp`,
checks emissive sampling error, Snell bending, total internal reflection,
thickness absorption, adaptive sample counts and all six local-shadow projections.
Pass the absolute denoiser library path to also exercise real OIDN filtering.
`tests/rendering/omnilight_cube_test.cpp` also guards the shared ray-intersection
code used by the existing light baker.

The reflection model and transport follow the equations described in
[PBRT's microfacet chapter](https://pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory)
and [path-tracing chapter](https://pbr-book.org/4ed/Light_Transport_I_Surface_Reflection/A_Better_Path_Tracer).
