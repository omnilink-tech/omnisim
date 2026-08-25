# R4 step-3c — replace WREN as the main viewport (plan + coupling map)

> **Update 2026-06-08:** this doc's *strategy* (additive-first, flip-last) is now
> largely executed — 3c-A parity (picking, selection, manipulator handles,
> overlays) has landed in the wgpu pane, and 3c-B (the offscreen→GL-blit main
> view, `renderMainFrameViaWgpu`) was **un-gated 2026-06-07** for any
> `renderBackend "wgpu"` Viewpoint (~75% lighting parity). The Phase-ζ *default*
> flip is still gated. Live task state:
> [r4-completion-checklist.md](r4-completion-checklist.md); canonical status:
> [engine-migration-plan.md §8.1](engine-migration-plan.md). The coupling map
> below remains the accurate reference for what 3c had to re-home.

**Status: PLANNING (2026-06-04).** Steps R4-1 → R4-3b+polish shipped a wgpu viewport
that renders the live world from the live camera, embedded as a pane and framed like
WREN, all behind `OMNISIM_VIEW3D_WGPU` (see [rendering-arm-checklist.md](rendering-arm-checklist.md)).
Step-3c is the endgame: make wgpu *the* main viewport with feature parity, then flip the
default. The migration plan budgets this (Phase ε) at **3–6 months**. This doc answers
the load-bearing question — **"does 3c break the working simulator?"** — and lays out a
sequencing that keeps the answer **no** until the very last, gated, reversible step.

---

## 1 — Core finding: the main view is 100% WREN, with no seam

`OmView3D : OmWrenWindow : QWindow` is hardcoded to WREN/OpenGL. The `OmRenderBackend`
abstraction (`src/omnisim/render/`) exists but is **consulted only on the per-Camera /
per-Viewpoint sensor path** — never in the main-view render loop. So 3c either (a) builds
parity in the **additive** wgpu pane first, or (b) introduces a seam into `OmWrenWindow`,
which edits code every user runs.

The HWND surface type is fixed in `OmWrenWindow`'s ctor: `setSurfaceType(QWindow::OpenGLSurface)`
+ `OmWrenOpenGlContext::init(...)`. On Windows an HWND's pixel format is set once — **flipping
that to a wgpu surface is the single most dangerous change**, because it's what the default
WREN view depends on.

## 2 — Coupling map (what 3c must re-home, and how risky each is)

| Subsystem | Key files | WREN/GL coupling | Class | Effort | If mishandled |
|---|---|---|---|---|---|
| Per-frame submission | `OmWrenWindow::renderNow` (`wr_scene_render` → blit → `swapBuffers`) | total | **SHARED** | L | breaks the view for **all** users, every world |
| Picking / selection | `OmWrenPicker` (`wr_scene_render_to_viewports` "picking" pass → `wr_frame_buffer_copy_pixel` GL readback); `OmSelection` | total | **SHARED** | M | click-to-select dead |
| Manipulator handles | `OmWrenAbstractManipulator`, resize manips, drag-event classes (WREN renderables + handle-ID picking) | total | **SHARED** | M | drag-to-move/rotate dead |
| Overlays | `OmWrenFullScreenOverlay`, `OmWrenTextureOverlay` (camera/display device output), `wr_viewport_attach_overlay` | total | **SHARED** | M | device overlays + status messages gone |
| Video / screenshots | `OmWrenWindow::grabWindowBufferNow` (`readPixels` GL), PBO path (`wr_scene_init_frame_capture`/`map_pixel_buffer`) | total | **SHARED** | M | recording/screenshots broken |
| Optional renderings | `OmWrenRenderingContext` visibility-mask + per-node renderables (bounding objs, contact pts, joint axes, COM, rays, normals…) | mask is **decoupled** | **ADDITIVE** | S | (none — read the mask, render in parallel) |
| Backend-dispatch seam | none exists; `OmWrenWindow` ctor + `renderNow` | — | **SHARED** | L | architecture-defining |

Only the optional-renderings **visibility mask** is genuinely decoupled (a wgpu renderer can
read `OmWrenRenderingContext::optionalRenderingsMask()` and filter its own draws). Everything
else couples tightly to WREN.

## 3 — The safe strategy: additive-first, flip-last

**Almost all of 3c can be built without touching WREN's default path**, because the wgpu
*pane* (`OmWgpuView`/`OmWgpuSurface`) is already a self-contained, opt-in render surface.
Build feature parity **inside that additive pane** until it's a fully usable view, and defer
the one truly shared, dangerous change — making wgpu *the* main view — to the very end.

- **Phase 3c-A — additive parity (breaks nothing).** All new code lives in the wgpu path,
  gated on `OMNISIM_VIEW3D_WGPU`; WREN's `OmView3D`/`OmWrenWindow` are not edited.
  - Wire mouse input on `OmWgpuView` → a **wgpu picking pass** (render IDs to an offscreen
    target, read back the clicked texel) → update the shared `OmSelection`.
  - Read the optional-renderings mask and draw those aids in wgpu.
  - Render **manipulator gizmos** in wgpu + hit-test via the wgpu picking pass.
  - Composite **overlays** (device output textures, messages) in the wgpu pane.
  - **Screenshot** the wgpu pane via the existing `presentScene(rgbaOut)` readback.
  - Cost: some logic is duplicated WREN-side vs wgpu-side until the flip. That duplication
    is the price of zero regression risk.
- **Phase 3c-B — the flip (gated + reversible; the only risky step).** Introduce a
  `WbMainViewport` seam (or extend `OmRenderBackend`) so `OmWrenWindow::initialize()` /
  `renderNow()` / `grabWindowBufferNow()` dispatch to a backend, and the window's surface
  type is chosen per-backend. **WREN stays the default**; wgpu is selected only by the flag
  until the final Phase ζ default-flip. Each edit to the shared path is guarded so the
  WREN branch is byte-identical.

## 4 — Direct answer: "are we breaking anything?"

- **Phase 3c-A: no.** Same risk profile as steps 1→3b — purely additive, flag-gated, WREN
  untouched. Worst case, the opt-in pane misbehaves.
- **Phase 3c-B: possible, and this is where the risk concentrates.** Editing
  `OmWrenWindow`'s ctor (surface type) and `renderNow` is code every user runs. The danger
  is a bug in the WREN branch of a now-conditional path, or the HWND pixel-format flip
  leaking into the default. Mitigated by: WREN remaining the default, every change guarded
  so the WREN branch is unchanged, the test net below, and revertibility.

## 5 — Non-negotiable safeguards (from the migration plan + repo memory)

- WREN remains the **default** main-view backend until the deliberate Phase ζ flip; existing
  worlds stay byte-equivalent.
- Everything behind `OMNISIM_WITH_VULKAN` (build) + `OMNISIM_VIEW3D_WGPU` (runtime) until ζ.
- **No file renamed** (engine-migration non-negotiable #1).
- The **determinism smoke worlds** (`accelerometer`, `contact_points`, `template_deterministic`)
  and **golden-image** checks must stay green with the flag **off** before every 3c commit.
- The wgpu surface self-check (`OMNISIM_VIEW3D_WGPU_SELFCHECK`) extends to each new 3c
  capability (picking hit, gizmo present, overlay composite) for numerical verification.

## 6 — Recommended first 3c increment

**3c-A.1: wgpu picking in the pane.** Wire `OmWgpuView` mouse-press → an offscreen wgpu
ID-render pass → readback → set `OmSelection`. It's additive (zero WREN-path edits),
numerically verifiable (self-check the clicked-ID readback), and unlocks manipulators next.
Defer the 3c-B flip until the additive pane is a genuinely usable view.
