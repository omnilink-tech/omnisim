# Synthetic data generation

*2026-08-21. Status: shipped (v1). Owner surface: `OMNISIM_WGPU_SYNTH_DUMP` (engine) +
[`scripts/dev/synth_data.py`](../../scripts/dev/synth_data.py) (driver).*

OmniSim can generate aligned RGB / depth / instance-segmentation samples with per-sample
domain randomization, rendered through the full wgpu stack (scattered sky, clouds, PCSS
shadows, SSR, GTAO, HDR+AgX). One command produces a dataset:

```bash
python scripts/dev/synth_data.py \
  --world projects/samples/demos/worlds/rendering/beauty_bench.omniworld \
  --out .local-runs/synth_demo --samples 12 --seed 7
```

Measured (machine `9722d23d12a3`, RTX 3060 laptop): **~11 s per sample**, 12/12 samples with
zero retries on the Beauty Bench. Deterministic poses/light rigs from `(world, seed, samples)`.

## The engine dump (`OMNISIM_WGPU_SYNTH_DUMP=<dir>`)

When set, the wgpu main view writes one ground-truth sample at the dump frame
(`OMNISIM_WGPU_MAINVIEW_DUMP_FRAME`, default 200; `OMNISIM_WGPU_SYNTH_EVERY=N` repeats every N
frames after it):

| file | contents |
|---|---|
| `rgb_NNNNNN.png` | the tonemapped frame — the full wgpu render stack, exactly what the main view shows |
| `depth_NNNNNN.png` | **uint16 millimetres** from the camera plane (0 = no hit, range 65.535 m), via the metric F32 depth pass |
| `inst_NNNNNN.png` | **per-SOLID instance ids** — `id = R + G·256 + B·65536`, 0 = background — via the flat pick shader with sRGB encode off (exact byte round-trip) |
| `meta_NNNNNN.json` | camera intrinsics (`fx/fy/cx/cy`, fov, near/far) + extrinsics (world position, row-major rotation; camera axes +X forward / +Z up in world ENU), the light rig (sun direction/energy, day factor, cloud cover), encodings, and the id → node name/DEF/world-position mapping |

The ground-truth passes render with a standard (non-reversed) view-projection through
pre-existing render-target paths (`clearAndDrawSceneDepthF32`, `clearAndDrawScene` pick mode), so
they are pixel-aligned with the RGB by construction. The main render is forced to a synchronous
readback on dump frames (the pipelined async readback would leave the shared readback buffer
mid-map when the ground-truth passes run — a wgpu validation abort, found the hard way).

## The driver (`scripts/dev/synth_data.py`)

Per sample, from one seed: sun azimuth uniform / elevation U[12°, 65°] (edits the world's
`OmniSimSun.direction`), cloud cover U[0.10, 0.60] (`OMNISIM_WGPU_CLOUD_COVER`), camera on an
orbit shell around `--target` (radius U[7, 15] m, elevation U[8°, 40°], always looking at the
target). All ranges are flags. It writes a per-sample world variant, runs one engine instance,
polls for the meta file (no fixed sleeps), retries once on the known cold-launch flake class,
and writes a `dataset.json` manifest.

## Limits — read before promising anyone a dataset

- **A desktop session is required.** The wgpu main view never repaints under
  `--batch`/`--minimize`, so the driver runs the engine windowed (`--mode=realtime`). Headless
  offscreen dumping is future work.
- **This pipeline renders the MAIN VIEW, and only the main view** — the dump lives in
  `OmView3D.cpp`, not in the Camera device. That means arbitrary camera poses yes (including
  robot-mounted ones via Viewpoint placement), one aligned sample per engine run, and no batched
  rendering: **vision-in-the-loop RL is NOT covered. This is an offline dataset generator.**
  ⚠️ The reason is no longer the renderer. WREN was deleted on 2026-08-23 (`976b9449d`) and the
  Camera device now renders through wgpu offscreen — including the real materials-and-lighting
  layer (`OMNISIM_WGPU_CAMERA_SCENE`, default ON: albedo/roughness/metalness/normal maps, the
  world's Background, the first `DirectionalLight` as a shadow-casting sun, Fog, cast shadows),
  so `wb_camera_get_image` sees the current renderer. What the Camera device does **not** emit is
  this generator's ground-truth passes (uint16-mm depth, per-solid instance ids, `meta.json`).
- **Alpha-cutout foliage is not trimmed in depth/instance maps.** The leaf shapes come from the
  mesh's leaf-card triangles; the alpha test (RGB pass) trims their edges, the GT passes do not —
  so foliage masks run slightly fat at leaf borders.
- **Glass occludes in the instance map** (translucent draws render solid there, deliberately —
  an id map cannot alpha-blend). The RGB shows through glass; `inst` labels the pane itself.
- **Instance = OmSolid.** A multi-shape solid is one instance; a URDF robot is its solid tree
  (per-link ids). Class labels are derived downstream from the `meta.json` names/DEFs.
- **Throughput** is one engine run per sample (~11 s warm). For scale, run K drivers in parallel
  with per-child `OMNISIM_LOG_PATH` (the multi-instance pattern, AGENTS.md §3e) — unmeasured.
- Depth is planar (camera-plane), not radial; 16-bit mm caps at 65.535 m.

## What this is honestly for

Offline perception datasets (detection, segmentation, depth estimation, pose estimation from
`meta.json`) with lighting/viewpoint domain randomization. It is NOT (yet): vision-in-the-loop
policy training (needs the ground-truth passes on the Camera-device path + batched rendering),
video sequences (use
`OMNISIM_WGPU_SYNTH_EVERY` for crude clips), or physics-labelled data (contacts/forces — the
harness serves those separately).
