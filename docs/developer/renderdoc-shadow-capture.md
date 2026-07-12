# RenderDoc capture guide — the textured-shadowed render non-determinism

## ✅ ROOT CAUSE FOUND (2026-06-06, captured + analysed headlessly — see "Findings" at the bottom)

**The floor draw's per-draw transform uniform is intermittently read by the GPU as UNINITIALISED GARBAGE,
which projects its vertices to astronomical clip coordinates → the primitive is DEPTH-CLIPPED → the whole
floor drops** (≈50 % of runs). Proven via RenderDoc PixelHistory (`EID 1239 passed=False FAILS: depthClipped`
on every floor pixel) + post-VS clip positions (`[16206086, 28322148, …]`, `[-958632361984, …]`, z/w out of
range). The floor is garbage in BOTH passes (shadow map + colour), so it is the **per-draw MODEL slot**, not
viewProj. It is NOT a GPU sync / command-ordering issue — **five fixes were ruled out by measurement** (pass-1
fence, render→copy fence, dedicated depth texture, write→submit empty flush, write→real-cmd-buffer flush). The
remaining fix work is to find why that specific uniform slot reads garbage (next: dump the raw `mScnUniformBuffer`
slot for the floor draw). Everything below is the (now-validated) capture method.

**Why this doc exists:** the wgpu **cast-shadow** render (`WbWgpuRenderTarget::clearAndDrawSceneTexturedShadowed`)
is non-deterministic — across runs its output coverage swings ~31–100% (the large ground/floor draw
intermittently drops), while the single-pass `clearAndDrawScene` is rock-stable. This is the **last real bug**
blocking lighting parity (and therefore the 3c-B main-view flip and Phase ζ).

See the full diagnosis in [r4-completion-checklist.md](r4-completion-checklist.md) (the
"Lighting/shadow convergence to WREN" block).

## What we already know (so you can skip re-deriving it)

- **All 222 draws are submitted every run** (`[texshadow-pass2] drawn=222 skipNoBuf=0 skipNoBg=0`) — nothing is
  skipped on the CPU side. So the floor *draw call* is issued; it just sometimes doesn't end up in the output.
- **No wgpu validation error fires** (an uncaptured-error callback is installed; set `OMNISIM_WGPU_ERRLOG=<file>`
  to capture any). So it's a *silent* state/scheduling issue, not a rejected command.
- **The value 1 071 725 (≈50% coverage) recurs exactly** across runs → a *deterministic partial* render gated on
  some timing-quantized state, not random garbage.
- **Ruled out by measurement (3 fixes, each reverted):** the pass-1→pass-2 shadow-map barrier (split + fenced —
  still flaky); the render→copy barrier (split + fenced — still flaky); and a **dedicated per-call depth texture**
  for pass-2 (vs sharing `mScnDepthView` — still flaky). So it is **not** command-buffer ordering between those
  stages, and **not** a shared-depth carryover. The leading remaining suspect is the **222 per-draw 9-binding
  bind-groups** created per call (unique to this heavy path; the stable single-pass path uses far lighter groups)
  — a possible wgpu-native allocator/lifetime hazard. RenderDoc's resource/bind-group view should show it.
- **Plain-lit `clearAndDrawScene` (single pass) is 100% stable** at 2 112 060 px every run — only the heavy
  *two-pass* path is affected.

## Build a wgpu-ON binary

```bash
make -C src/omnisim release \
  OMNISIM_WITH_CUDA=OFF OMNISIM_WITH_NEWTON=ON OMNISIM_WITH_VULKAN=ON \
  WGPU_NATIVE_HOME=$PWD/_scratch/wgpu-native
```
The link step auto-ships `wgpu_native.dll` next to `msys64/mingw64/bin/omnisim-bin.exe`
(see [wgpu-native-setup.md](wgpu-native-setup.md)).

## Backend is already Vulkan — no forcing needed (CONFIRMED)

RenderDoc only hooks Vulkan / D3D / GL, not WREN's GL pane swap — so the wgpu work must run on a
Vulkan (or D3D12) backend. **Verified on the dev box: wgpu-native already picks Vulkan by default**
(`adapter backend=Vulkan (RenderDoc-capturable)  device=NVIDIA GeForce RTX 3060 Laptop GPU`). So just set:

```
OMNISIM_VIEW3D_WGPU=1           # render the live pane through wgpu
OMNISIM_WGPU_INITLOG=<file>     # (optional) re-confirms the backend/adapter in <file>
```
The init log line `adapter backend=Vulkan (RenderDoc-capturable)` confirms RenderDoc can hook it. (If a
machine ever defaults to a non-capturable backend, the init log will say so.)

## Capture steps

1. Open **RenderDoc** → *Launch Application*.
2. **Executable**: `msys64\mingw64\bin\omnisim-bin.exe`
   **Working dir**: `msys64\mingw64\bin`
   **Command-line args**: `"projects\robots\boston_dynamics\spot\worlds\spot.wbt" --mode=pause`
   (full path to any shadowed world). `--mode=pause` keeps the scene static so the capture is reproducible.
3. **Environment** (RenderDoc's env editor): set `OMNISIM_VIEW3D_WGPU=1` (the backend is already Vulkan —
   no `WGPUNATIVE_BACKEND` needed).
4. Capture options: tick **"Capture all sub-resources"** and **"Save all initial resource states"** (so the
   shadow map + depth textures are inspectable). Leave API validation ON.
5. *Launch*. When the OmniSim window shows the 3D pane, press **F12** (or *Capture Frame(s) Now*) a few times to
   grab several frames — we specifically want a frame where the **floor is missing** vs one where it's present, so
   capture ~5 frames.
6. Close OmniSim; open the captures.

## What to inspect (the actual question)

In each capture's **Event Browser**, find the two-pass shadowed render. It is two `BeginRenderPass` blocks back to
back, then a `CopyTextureToBuffer`:
- **Pass 1** → renders depth into the shadow map (color target is the R32Float shadow texture).
- **Pass 2** → renders the lit scene (color target is the offscreen RGBA8; it samples the shadow map).

For a **"floor missing"** capture, answer these (this is what will pinpoint the bug):

1. **Is the floor's `DrawIndexed` present in Pass 2's event list?** (It's the draw with the largest index count —
   `maxIdxCount` ≈ 38 100.) If it's there but produces no output → it's a depth/raster issue. If the event is
   absent → a CPU-side skip we missed.
2. **Select the floor draw → Pipeline State → Depth/Stencil.** Is the depth attachment the expected full-size
   texture? Is `depthCompare = Less`, depth-write on? Compare to a "floor present" capture — does anything differ?
3. **Mesh Output / VS Output for the floor draw:** are the floor's clip-space vertices sane (inside the frustum),
   or are they NaN / behind the near plane in the bad capture?
4. **Texture viewer → the offscreen color target after Pass 2:** is the floor region the sky clear-colour
   (= didn't write) or a shaded-but-wrong colour (= wrote but failed later)?
5. **The shadow map (Pass 1 output):** is it fully rendered in both the good and bad captures, or partially
   written in the bad one?
6. **Resource initial states:** does the offscreen depth texture (`mScnDepthView`) or the shadow map carry
   **stale/uninitialised** contents at the start of Pass 2 in the bad capture?

## What to send back

- The `.rdc` capture files (one "floor missing", one "floor present" if you can catch both), **or**
- screenshots of: Pass 2's event list with the floor draw selected, its Depth/Stencil pipeline state, and the
  color-target texture-viewer — for both a good and a bad frame.

That comparison (good vs bad frame, same draw) will almost certainly show the divergence — most likely a
depth-attachment / initial-state hazard or a VS-output anomaly on the large floor primitive — and then the fix is
a targeted one (dedicated depth texture, explicit clear, or a per-draw state correction).

## Quick repro of the flakiness without RenderDoc (to confirm a fix later)

The GUI self-check writes a `render-coverage:` line. Run it PAUSED several times; a real fix makes
`litShadow`/`litShadowFlat` == `lit` (2 112 060) on **every** run:

```
OMNISIM_VIEW3D_WGPU=1 OMNISIM_VIEW3D_WGPU_SELFCHECK=<file> \
  omnisim-bin.exe <world>.wbt --mode=pause   # force the window visible; read <file>
```

---

## Findings — how the capture + analysis was done HEADLESSLY (2026-06-06)

The whole capture + analysis ran with **no GUI clicks**, from the agent environment:

1. **Tooling (no install/admin):** downloaded the RenderDoc 1.44 portable ZIP into `_scratch/renderdoc/`.
   `renderdoccmd vulkanlayer --register --user` is NOT supported, but the Vulkan loader honours
   `VK_LAYER_PATH=<renderdoc dir>` + `VK_INSTANCE_LAYERS=VK_LAYER_RENDERDOC_Capture` to load the capture layer
   into wgpu-native's Vulkan instance with **no registration**. (wgpu-native already picks the Vulkan backend —
   the `OMNISIM_WGPU_INITLOG` line `adapter backend=Vulkan` confirms it.)
2. **In-app capture harness** (in `WbWgpuView::runSelfCheck`, gated by `OMNISIM_RDC_CAPTURE`, compiled only when
   `renderdoc_app.h` is present): resolves `RENDERDOC_GetAPI` via `QLibrary("renderdoc")` (the layer has loaded
   `renderdoc.dll` into the process), then brackets each `clearAndDrawSceneTexturedShadowed` with
   `StartFrameCapture(NULL,NULL)` / `EndFrameCapture(NULL,NULL)` in a 16× loop — **no swapchain present needed** —
   tagging each `.rdc` with its measured coverage so a known-BAD and known-GOOD capture are produced in one run.
3. **Headless analysis:** `qrenderdoc.exe --python <script.py>` runs a script with the `renderdoc` module
   available (embedded Python 3.6). Open the `.rdc` (`rd.OpenCaptureFile().OpenFile` + `.OpenCapture`), then:
   - **PixelHistory** on a floor pixel of the RGBA8 colour target (`controller.PixelHistory(rid, x, y, sub, ...)`)
     → reports per-fragment fate. Floor pixels showed `passed=False FAILS: depthClipped`.
   - **Post-VS** (`controller.GetPostVSData(0,0,rd.MeshDataStage.VSOut)` + `GetBufferData`) on the floor draw →
     clip positions were astronomical garbage, z/w out of `[0,1]` → confirms depth-clip is from a garbage transform.
   The probe/PixelHistory/floor-draw scripts live in `_scratch/rdc_*.py`.

**Conclusion:** the floor draw's per-draw model-matrix uniform slot is read as uninitialised garbage on ~50 % of
runs. Next fix step: read the raw `mScnUniformBuffer` slot for the floor draw (via the Vulkan descriptor-set API /
`GetBufferData`) to see whether the slot itself is garbage in the buffer (write never landed / wrong slot) vs the
correct buffer being read at a wrong dynamic offset — that distinguishes an upload bug from a bind-offset bug.
