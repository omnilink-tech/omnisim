# OmniSim Sensor and Device Performance

This guide focuses on cameras, distance sensors, overlays, recognition, and other device-side rendering or ray-based costs.

Sensor-heavy worlds are often where simulator performance feels worst, because they combine:

- simulation stepping
- per-device update scheduling
- extra rendering passes
- CPU-side post-processing
- GPU memory transfer

## Current Sensor Path

### Camera and rendering devices

The runtime calls `robot->renderCameras()` during the simulation step for started controllers.

Each rendering device then decides whether it needs to refresh and may trigger a render pass or texture update work.

### Abstract camera behavior

`WbAbstractCamera`:

- checks whether the sensor needs refresh
- computes device value
- toggles viewpoint and invisible-node visibility
- renders through a WREN camera
- records device-rendering time in the performance log

Implication:

- each sensor render is not just a texture copy
- it can temporarily alter visibility state and run a full render path

### Camera recognition

The camera recognition code currently:

- loops over all recognition objects in the world
- discards some by simple distance checks
- constructs recognized-object state
- projects corners into image space
- updates overlay textures and controller-visible data

Implication:

- recognition cost scales with world content
- there is no obvious broadphase or spatial index for candidate selection here

### Distance sensors

Distance sensors can reduce work when multiple rays are specified with zero aperture by collapsing to one ray, which is already a useful optimization.

Implication:

- ray count is a major cost driver
- sensor configuration matters as much as engine implementation

### Overlay and texture upload path

The overlay path and recognition rendering still perform CPU-side processing and call texture-upload functions in performance-sensitive code.

Implication:

- `gpuMemoryTransfer` is a key sensor-performance metric
- camera-heavy and display-heavy worlds can bottleneck outside pure rendering time

## Main Performance Problems

### 1. Too much per-device scene-state work

If every sensor render has to:

- enable hidden nodes
- hide specific nodes
- render
- restore visibility

then multi-sensor worlds pay substantial scene-state management overhead.

### 2. Recognition is world-size sensitive

The current recognition path still does object-by-object candidate work and per-object image projection calculations.

Better candidate filtering would help worlds with many detectable objects.

### 3. Overlay work is still CPU-heavy

Recognition frames, depth conversion, and related paths still spend time in CPU-side processing and repeated texture uploads.

### 4. Sensor configuration can explode cost

High ray counts, high resolution, high update frequency, and many active devices combine multiplicatively.

That makes authoring rules essential.

## Improvement Strategy

### Priority A: Separate desktop rendering policy from sensor policy

Sensors need:

- predictable output
- bounded cost
- clear performance knobs

The main viewport needs:

- visual quality
- interactive feel

These are related but not identical goals.

### Priority B: Add better candidate filtering for recognition

The recognition system should get cheaper before projection and overlay work begin.

Promising directions:

- spatial indexing
- view-dependent candidate sets
- cheaper bounding tests before per-object corner projection

### Priority C: Reduce CPU-side overlay and texture work

Good targets:

- camera recognition overlays
- depth conversion paths
- repeated partial texture updates

### Priority D: Benchmark sensor-heavy worlds explicitly

Keep at least one benchmark that is clearly sensor-driven rather than physics-driven.

## Authoring Rules For Sensor Worlds

- keep camera resolution no higher than the scenario actually needs
- keep update periods realistic
- keep distance-sensor ray counts justified by the use case
- avoid turning every available sensor on by default in sample worlds
- treat remote-mode overlays and debug overlays as optional cost, not free cost

## Developer Checklist For Sensor Changes

When touching:

- `src/omnisim/nodes/WbAbstractCamera.*`
- `src/omnisim/nodes/WbCamera.*`
- `src/omnisim/nodes/WbDistanceSensor.*`
- `src/omnisim/nodes/WbDisplay.*`
- `src/omnisim/wren/WbWrenTextureOverlay.*`
- `src/omnisim/engine/WbSimulationWorld.*`

Do all of the following:

- compare `deviceRendering` and `gpuMemoryTransfer` before and after
- validate at least one sensor-heavy world
- note whether the change affects headless automation, desktop rendering, or both
- avoid increasing default sensor cost without documenting the tradeoff

## Better Sensor Performance Later

Later, a better sensor subsystem should have:

- clearer sensor-vs-desktop rendering separation
- cheaper candidate selection for recognition
- more GPU-side overlay and conversion work
- benchmark-grade sensor scenarios
- contributor guidance that treats sensor configuration as a first-class performance concern
