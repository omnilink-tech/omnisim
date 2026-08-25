# The wgpu "shadow aura" — what it actually was

**Status: FIXED (2026-08-24).** Reported 2026-08-22 from a user observation on the
OmniArm 6 universal-pick demo, then reported again on 2026-08-24 after a first fix that
turned out to be aimed at the wrong subsystem. Both passes are recorded here, because the
misattribution is the more useful half of the story.

## The symptom

Under the wgpu main view (the default since 2026-08-19) every object sits in a broad, soft
dark field on the floor — the robot, the bins, the small parts alike. It reaches well past
each object's own footprint, it has no penumbra structure, and it does not track the sun
direction. It is not a shadow.

## The cause: GTAO's falloff window stopped matching the disc it samples

[`OmWgpuShaders.cpp`](../../src/omnisim/render/OmWgpuShaders.cpp), `kSsaoGtao`:

```wgsl
let projPix = clamp(u.p0.x * u.p2.z / pos.z, 2.0, u.p2.w);   // u.p2.w = 64 px
...
let falloffStart2 = 0.16;                                     // 0.4 m, WORLD units
let falloffEnd2 = max(u.p0.x * u.p0.x, falloffStart2);        // radius^2, WORLD units
```

WREN's `gtao.frag` used its projected radius **unclamped**:

```glsl
float projectedRadius = (radius * clipInfo.z) / viewSpacePosition.z;   // no clamp
#define FALLOFF_END2 max(radius * radius, FALLOFF_START2)
```

so its world-space falloff window always described the disc it actually searched: a sample
at the edge of the disc really was `radius` metres away and really did get attenuated to
zero. The wgpu port added the pixel clamp as a **perf guard** — capping the kernel at 64 px
keeps the texture-cache cost bounded — and left the falloff window derived from the
*authored* radius.

Once that clamp binds, the two disagree. It binds for anything nearer than
`radius * projScale / maxPix`, which at the default `Viewpoint.ambientOcclusionRadius 2`
and a 1113 px viewport is about **42 m** — i.e. essentially always. Back-projected, the
64 px disc covers roughly 0.1 m of world in a tabletop scene, so every sample lands far
inside a 0.4 m `FALLOFF_START2` and is taken at **full weight**. The kernel degenerates
into a flat, unattenuated 64-pixel disc: a fixed-size dark ring glued to every silhouette,
the same width whether the object is a 4 m robot or a 3 cm pin, because it is measured in
pixels and nothing in it is measured in metres any more.

That is the aura.

### The fix

Derive the window from the radius the disc *actually* searches:

```wgsl
let effR = projPix * pos.z / u.p2.z;     // back-project the CLAMPED radius
let falloffEnd2 = max(effR * effR, 1e-6);
let falloffStart2 = falloffEnd2 * 0.04;  // WREN's 0.16/2^2 ratio, scaled to the real disc
```

Where the clamp does not bind, `effR == u.p0.x` and this is WREN's formula exactly — which
is why the city is unaffected (below). Where it does bind, the outer samples taper to zero
as they always should have, and AO collapses back to contact and crevice darkening.

## Measured

Instrument: `scripts/dev/render_ab.py` against a **static** copy of the demo world (both
controllers set to `"<none>"`, so the run-to-run noise floor is 0 px over threshold — on the
driven world it is not, and that is what made the first pass misread). Machine
`9722d23d12a3`, RTX 3060 laptop, frame 300, threshold 30.

Isolating the subsystem, each arm against the same baseline:

| arm | max | mean | px > 30 | shape of the difference |
|---|---|---|---|---|
| baseline repeated (noise floor) | 4 | 0.000 | 0 | — |
| `OMNISIM_WGPU_NO_SSAO=1` | 90 | 5.188 | 17064 | **a glow ring around every silhouette** |
| `OMNILIGHT=0` | 73 | 4.858 | 72701 | a uniform ambient wash, no ring |
| `OMNISIM_WGPU_SSR=0` | 23 | 0.064 | 0 | nothing |
| `OMNISIM_WGPU_PCSS=0` | 14 | 0.006 | 0 | nothing |

`OMNILIGHT=0` moves the **most** pixels and is **not** the aura: it lifts ambient light
everywhere by roughly the same amount. Only the SSAO arm is shaped like the complaint. Read
the diff image, not the pixel count.

Before/after the shader fix, both against the AO-off arm:

| | max | mean | px > 30 |
|---|---|---|---|
| old AO vs no AO | 90 | 5.188 | 17064 |
| **new AO vs no AO** | **49** | **4.424** | **620** |

AO is still doing its job (the mean barely moves — most of the ambient darkening is real
crevice occlusion) while the halo pixels drop **27×**. The residual is a tight contact seam
where objects meet the floor, which is what AO is for.

### No regression on the look benchmarks

| world | max | mean | px > 30 |
|---|---|---|---|
| `city_traffic` | 38 | 0.013 | **1 (0.00%)** |
| `beauty_bench` | 93 | 2.146 | 17065 (0.81%) |

The city is untouched, as the derivation predicts: at city camera distances the pixel clamp
does not bind, so `effR == radius` and the arithmetic is identical. `beauty_bench` is the
close-up hero scene, where the clamp binds and the change is the intended one — the grey
wash over the grass and the smear down the brickwork are gone, the doorway and hedge-base
occlusion are unchanged.

## The wrong turn (2026-08-22), kept on purpose

The first pass attributed the aura to OmniLight's irradiance volume being too coarse —
`OmniLightParams::minSpacing = 0.45f`, giving 864 probes over a ~5 m scene — and fixed it by
tightening the default to 0.22 m (`OMNILIGHT_SPACING`, exact-revert `=0.45`). That change is
real and worth keeping: denser probes genuinely reduce GI over-occlusion, and it made the
aura *less* obvious. It was not the cause.

Three things produced the wrong answer, all of them avoidable:

1. **One arm, no alternatives.** `OMNILIGHT=0` was the only variable tested. It changed the
   image a lot, so it got the blame. Nothing was tested against SSAO, SSR or PCSS.
2. **No noise floor.** The A/B ran on the world with its controllers live, so the arm poses
   differed between runs. `render_ab.py --noise-floor` exists for exactly this and was not
   used. Trap 6 in that script's own header says so.
3. **A pixel count read as a diagnosis.** `OMNILIGHT=0` still moves the most pixels of any
   arm here. It is a scene-wide brightness shift, and no scalar summary can tell that apart
   from a localised halo. The 4×-amplified difference image answers it in one look.

## Known still broken

`OMNISIM_WGPU_GTAO=0` renders no frame at all — the documented revert hatch for the GTAO
path does not work, which is why AO could not be A/B'd through it and
`OMNISIM_WGPU_NO_SSAO=1` (the `OmView3D` strength kill-switch) had to be used instead. Not
fixed here.

## Reproducing

```bash
# a static probe: no controllers, so the noise floor is exactly 0
sed 's/^  controller ".*"/  controller "<none>"/' \
  projects/samples/demos/worlds/flagship/omniarm6_universal_pick.omniworld \
  > projects/samples/demos/worlds/flagship/.aoprobe_static.omniworld

python scripts/dev/render_ab.py --world projects/samples/demos/worlds/flagship/.aoprobe_static.omniworld \
  --no-diff --out-dir /tmp/base
python scripts/dev/render_ab.py --world projects/samples/demos/worlds/flagship/.aoprobe_static.omniworld \
  --no-diff --arm-a OMNISIM_WGPU_NO_SSAO=1 --out-dir /tmp/noao
# then diff /tmp/base vs /tmp/noao AND LOOK AT THE IMAGE, amplified 4x
```
