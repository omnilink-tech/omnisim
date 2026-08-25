# OmniLight — baked global illumination

*2026-08-21. Status: shipped (v1), default ON on the HDR path. Kill switch `OMNILIGHT=0`
(value-parsed, verified pixel-identical). Source: [`src/omnisim/render/OmniLight.cpp`](../../src/omnisim/render/OmniLight.cpp).*

## The principle

Lumen-class engines trace light every frame and pay for it every frame. OmniLight takes the
other trade — the same one the scattered sky's LUT took: **trace the light COMPLETELY and
PHYSICALLY, but only when the light rig or the scene changes.** Per frame, the lit shader pays
one trilinear 3D-texture sample. The runtime cost is a constant, independent of scene
complexity; the tracing cost is seconds, paid off-frame.

## What it does

A CPU path tracer (multithreaded, deterministic) runs over a BVH built from the **actual render
triangles** (retained CPU-side by the mesh cache) and bakes an **irradiance probe volume**
(up to 40×40×16 probes, L1 spherical harmonics per probe):

- **sky radiance** on miss, from a table baked over the *same scattered atmosphere* the dome
  renders — GI and sky can never disagree;
- **transmitted-sun direct lighting** at every hit (shadow ray), so bounce light carries the
  sun's real colour and occlusion;
- **surface albedo** = material baseColor × the albedo texture's linear mean (computed once at
  texture upload — diffuse GI needs the average, not the detail);
- **emissive surfaces as light sources** — at night the door strip and lamp heads are the
  scene's lights, and their bounce wraps the facade;
- **two bounces** (probe → surface → surface → sky/sun).

Probes buried inside geometry are detected (backface-hit fraction) and written with weight 0;
the shader renormalises the trilinear blend around them. The sample point is biased a full
probe cell along the surface normal — the cure for interior light leaking through thin walls
(found on the bench's plaster wall, first bake).

The result replaces the lit shader's flat hemisphere **ambient** term. Direct sun stays
real-time (PCSS shadow maps); specular stays SSR + analytic IBL.

## Measured (machine `9722d23d12a3`, RTX 3060 laptop, 8-thread bake)

| scene | triangles | probes | BVH | trace | renderMs after |
|---|---|---|---|---|---|
| Beauty Bench | 32,440 | 25,600 (25,401 valid) | 8 ms | **1.6 s** | unchanged |
| city_traffic | 619,747 | 25,600 (25,240 valid) | 210 ms | **2.9–3.4 s** | **7–8** (unchanged) |

Bakes run on a worker thread over a main-thread snapshot of world-space triangles (no live
scene pointers — reload-safe; the snapshot copy is the only main-thread cost). `maxGapMs`
stayed at its baseline through bakes — no frame hitch.

## Rebake policy

The trigger key = quantized sun direction (~1.4° steps, so a cycling day-night marker rebakes
in steps, not every frame) + sun energy + a scene fingerprint (draw count + index sum) + the
atmosphere preset. **Moving robots do not retrigger** — this is static-scene GI by design; a
robot reads the field as it moves through it, but does not write into it.

## Knobs

`OMNILIGHT=0` off (bit-exact revert) · `OMNILIGHT_RAYS` rays/probe (default 256) ·
`OMNILIGHT_SCALE` output multiplier (default 0.85).

## Honest limits

- **Diffuse only.** Specular GI stays SSR + analytic IBL.
- **Static-scene assumption**: a moved object leaves its lighting behind until the next
  rebake trigger; dynamic emissives (headlights) don't inject light.
- **Point/spot lights are not baked** (only emissive surfaces, sun, sky) — they remain
  real-time unshadowed direct lights. Baking them is a natural v2.
- L1 SH is low-frequency — no sharp indirect shadows; probe spacing (~0.5–0.7 m) bounds the
  smallest lighting feature.
- HDR path only; the LDR arm keeps the legacy hemisphere bit-exactly.
- **No RTX required, deliberately**: tracing once per light change puts the bake at seconds on
  a CPU, which meets the design goal. A GPU/hardware-RT bake is a drop-in upgrade if bake
  latency ever matters (e.g. continuous time-of-day), not a prerequisite.

## OmniLight 2 + the realism-push finale, 2026-08-21

Four additions in one campaign (each committed + gated; full-stack city perf: renderMs **8**):

1. **Baked local lights** (`OMNILIGHT_LOCAL_SCALE`, default 3.0): point/spot lights trace into
   the field with a shadow ray per light per hit; their unshadowed real-time records crossfade
   out with the volume fade-in. No more light through walls at night.
2. **Traced specular probe**: a 64² 3-mip cubemap path-traced from the scene's airy centre,
   parallax-corrected (scene AABB) — metals and glass mirror actual surroundings; replaces the
   analytic env palette whenever a volume is live.
3. **Volumetric shafts** (`OMNISIM_WGPU_VOLUMETRIC`, `_VOL_DENSITY` default 0.012): sun
   visibility bakes into slab-1 alpha of the probe volume; a 16-step additive march gives god
   rays and occlusion-aware haze. (First cut overcounted by 4π — the HG phase is already
   normalised; caught on the golden-hour A/B.)
4. **TAA** (`OMNISIM_WGPU_TAA`) — Halton-jittered lit projection + post-tonemap
   reproject/clamp/blend resolve (`kTaaMvResolve`; the `Mv` avoids the pre-existing offscreen
   T1.4 `accumulateTaa` names) — and the **camera pass** (`OMNISIM_WGPU_AUTOEXP`, clamp
   [0.55, 1.7] so night stays moody; `OMNISIM_WGPU_CAMFX` vignette + grain).

Content: the bench's v4 pass adds PicketFence/Sassafras/Wheelbarrow/WateringCan + 42
alpha-cutout grass-card clumps (generated `textures/grass_clump.png`). A world copied OUTSIDE
`worlds/rendering/` must rewrite that relative texture URL (the launcher generator does).
