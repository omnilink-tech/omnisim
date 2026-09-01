# OmniSim Rendering and Visual Quality Guide

This guide is for developers working on the renderer, sensor rendering, and overall visual quality of the simulator.

> ✅ **WREN was DELETED on 2026-08-23** (commit `976b9449d`: `src/wren` + `include/wren` + `src/omnisim/wren`). wgpu-native (Vulkan / D3D12 / Metal) is the only renderer, compiled into the engine from `src/omnisim/render/`. WREN-era problem descriptions below are kept only where the underlying issue (CPU-side overlay work, instrumentation gaps) still exists on the wgpu path. (Banner added 2026-09-01.)

## Rendering Stack

The rendering path is split across two layers:

### `src/omnisim/render`

The wgpu backend:

- surface/context management (`OmWgpuSurface`, `OmVulkanBackend`)
- render targets and framebuffers (`OmWgpuRenderTarget`)
- shaders (`OmWgpuShaders`), mesh and texture caches (`OmWgpuMeshCache`, `OmWgpuTextureCache`)
- adapters mapping simulator assets to GPU resources (`OmWgpuMeshAdapter`, `OmWgpuImageAdapter`)

### `src/omnisim/gui` and sensor nodes

Product and device layer:

- main 3D view
- user-visible renderer configuration
- camera, display, range finder, and other sensor-specific rendering paths

## High-Value Current Problems

### 1. CPU-side background and irradiance rotation

Background texture rotation and irradiance rotation are still called out in code as work that should be performed in OpenGL or in shaders.

Why it matters:

- avoidable CPU work during environment setup
- more complicated texture preparation path
- harder to reason about asset-load cost vs render cost

### 2. Coarse visual-quality fallback

The main view currently reduces quality using broad heuristics such as:

- disable shadows
- disable anti-aliasing
- disable GTAO
- reduce texture quality

This is a practical compatibility path, but it is not a modern quality system.

Why it matters:

- user-visible quality can change in large jumps
- diagnostics are weak when quality is degraded automatically
- benchmarking visual regressions becomes harder

### 3. Sensor overlays still do too much CPU-side work

The current sensor and overlay paths still include:

- CPU buffer allocation
- per-frame conversion work
- repeated texture upload calls

Why it matters:

- sensor-heavy worlds pay extra CPU and GPU-transfer cost
- camera-recognition overlays and depth-display paths can dominate frame cost

### 4. Performance instrumentation is incomplete

The performance log still has an average FPS field, but the renderer-side hook that should feed it is disabled (a WREN-era gap that survived the wgpu migration; `OMNISIM_RENDERER_TIMINGS=1` is the current signal).

Why it matters:

- developers are missing a direct visual throughput signal
- before/after render comparisons are harder than they should be

### 5. Debugging support is still uneven

Some older debugging surfaces still have renderer-migration gaps (originally WREN-era; not all were closed by the wgpu move).

Why it matters:

- render and physics visualization issues are harder to inspect
- developers spend more time inferring renderer state from symptoms

## Visual Quality Priorities

### Priority A: Make the renderer observable

Add or restore:

- trustworthy frame statistics
- pass-level timings where practical
- clear logs when quality fallbacks are activated
- simple before/after benchmark commands developers can run repeatedly

### Priority B: Move obvious texture work to the GPU

Good candidates:

- background rotation
- irradiance rotation
- overlay compositing that is currently CPU-prepared

### Priority C: Separate desktop visuals from sensor visuals

The simulator needs two related but different rendering goals:

- high-quality desktop visualization
- predictable and efficient sensor/image generation

Treating them as the same pipeline creates unnecessary coupling.

### Priority D: Improve quality presets

Replace one-off degradation rules with explicit presets such as:

- compatibility
- balanced
- quality
- sensor-accuracy-first

This makes developer behavior, user behavior, and benchmark behavior easier to reason about.

### Priority E: Reduce GPU transfer pressure

When `gpuMemoryTransfer` moves, inspect:

- camera overlays
- display updates
- depth-texture conversion
- any path that reads mesh or texture data back through the CPU

## Recommended Validation For Renderer Work

### Narrow visual change

```bash
python scripts/dev/omnisim_dev.py build renderer
python scripts/dev/omnisim_dev.py build gui
python scripts/dev/omnisim_dev.py test-world tests/rendering/worlds/normals.omniworld --nomake
```

### Performance-sensitive visual change

```bash
python scripts/dev/omnisim_dev.py build core
python scripts/dev/omnisim_dev.py profile-world tests/rendering/worlds/normals.omniworld
python scripts/dev/omnisim_dev.py benchmarks --nomake
```

## Edit Map

If the task is about:

### shaders, textures, mesh upload, framebuffers, GPU resources

Start in:

- `src/omnisim/render`

### simulator-specific rendering integration, offscreen device rendering

Start in:

- `src/omnisim/nodes/OmWgpuSceneRenderer.*`

### main viewport behavior, feature toggles, renderer capability checks

Start in:

- `src/omnisim/gui/OmView3D.*`
- `src/omnisim/gui/OmWgpuView.*`

### camera, display, and sensor rendering

Start in:

- `src/omnisim/nodes/OmAbstractCamera.*`
- `src/omnisim/nodes/OmCamera.*`
- `src/omnisim/nodes/OmDisplay.*`
- related sensor node files under `src/omnisim/nodes`

## What To Improve Later

Strong phase-two and later targets:

- first-frame render timing
- better scene and pass diagnostics
- explicit quality presets
- cleaner sensor-vs-desktop rendering separation
- GPU-side environment texture transforms
- reduced CPU staging for overlays and sensor textures
