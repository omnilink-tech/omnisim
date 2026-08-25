# D1.3 / C8 — the `OmGlWindow` extraction: mechanical implementation brief

Produced 2026-08-23 by a read-only design pass at HEAD `08a2f10a5`, for Lane E5 of
[wren-deletion-runbook.md](wren-deletion-runbook.md). This document IS the implementation
brief: the lane types it in rather than re-discovering it. It also corrects the runbook's C8
scoping in nine places (§6) — including two stale line citations, three missing consumers, a
missing seventh responsibility, and one instruction (the headless window) that would have
silently broken `--no-window` camera rendering if followed.

## 0. The rule that decides every line

> **Everything that survives D1.4 goes to `OmGlWindow`. Everything that dies stays in
> `OmWrenWindow`.**

Applied mechanically this resolves every ambiguous member, and it makes D1.4 a pure file
deletion plus consumer trims rather than a second refactor.

## 1. `gui/OmGlWindow` — what moves

New class `OmGlWindow : public QWindow`, `Q_OBJECT`. ~95 lines hpp + ~150 cpp. Members moved
from `OmWrenWindow` (hpp is **131** lines, not the runbook's 127):

- `instance()` + `cInstance` (retyped) — ⚠️ or leave the singleton on `OmWrenWindow`: its ONLY
  repo-wide consumer is `OmWgpuView.cpp:2342`, the parity self-check that dies with C9. If
  moved, that site becomes `dynamic_cast<OmWrenWindow *>(OmGlWindow::instance())`.
- static `flipAndScaleDownImageBuffer` (hpp:38-39, cpp:432-450) — must move because
  `OmVideoRecorder.cpp:67` reaches it as an inherited static through `OmView3D::`.
- ctor split (cpp:61-111): `mUpdatePending`, `mVideoStreamingServer`, the `cInstance` assert,
  `setSurfaceType(OpenGLSurface)` (cpp:76), the format request (78-93),
  `OmWrenOpenGlContext::init` (95) → `OmGlWindow`. The `mSnapshot*`/`mVideoPBO*`/four
  `mWren*FrameBuffer*` inits stay in `OmWrenWindow`. The two GL fatals (97-110) → §4.
- dtor split (cpp:113-147): WREN teardown (114-137) + `delete[] mSnapshotBuffer` (146) stay;
  `#ifndef _WIN32 destroy()` (139-141) and `doneWren(); OmWrenOpenGlContext::destroy();`
  (143-144) move. ⚠️ `cInstance` is never nulled in the dtor — copy the omission verbatim
  (behaviour parity); a second construction would trip the assert, unexercised today.
- `initializeForHeadless()` (hpp:44-48), `minimumSize`/`sizeHint` (cpp:613-619),
  `setVideoStreamingServer` (cpp:621-624) + protected accessor (hpp:98-100),
  `feedMultimediaStreamer` (cpp:626-628), `renderLater()` slot (cpp:200-205),
  `event()` (cpp:392-408), signal `resized()`.
- Base bodies for the split virtuals:
  `initialize() { create(); }` (was cpp:159) · `renderNow(bool,bool) {}` ·
  `resizeWren(int,int) { renderLater(); emit resized(); }` (was cpp:427+429) ·
  `grabWindowBufferNow() { return QImage(); }`.
  `OmWrenWindow`'s overrides call up: `initialize()` keeps its `wr_gl_state_is_initialized()`
  early-return (cpp:155-156) AHEAD of the up-call; `resizeWren` keeps cpp:411-425 then calls up.
  Byte-identical behaviour.
- Include hygiene (§6.9): `OmGlWindow.cpp` includes `OmVersion.hpp` DIRECTLY (today it arrives
  transitively via `OmPreferences.hpp`). Do not migrate the five dead includes in
  `OmWrenWindow.cpp` (`OmDragSolidEvent`, `OmLensFlare`, `OmMessageBox`, `OmWrenBloom`,
  `OmWrenPicker`) nor the unused `<QtCore/QMutex>`.

## 2. What stays in `OmWrenWindow : public OmGlWindow`

`grabSceneOffscreen` (100% WREN) · `initVideoPBO`/`completeVideoPBOProcessing` (virtuals) ·
`requestGrabWindowBuffer` · `processVideoPBO` · `updateFrameBuffer`/`recreateMainFrameBuffer` ·
`readPixels` · signal `videoImageReady` (emitted only at cpp:509) · all `mSnapshot*`,
`mVideoPBO*`, `mWren*FrameBuffer*` members · the six `<wren/*.h>` includes ·
`QOpenGLFunctions_3_3_Core`.

**The two lines that break on a naive split** — both inside `OmWrenWindow.cpp` itself:
`:378` and `:380` touch `mVideoStreamingServer`, which becomes a base **private**. Fix: use
the protected `videoStreamingServer()` accessor (as `OmView3D.cpp:3561` already does). A clean
compile of these two lines is evidence the split is otherwise complete — everything else the
WREN render path touches is WREN, `QWindow`, or a still-virtual.

## 3. Consumers — complete list with decisions

- `OmView3D` base class + its `OmWrenWindow::…` qualified calls: ALL stay at D1.3; re-point at
  D1.4. The `mousePressEvent`/`mouseReleaseEvent` qualified calls are at **`:3804`/`:4294`**
  (runbook's `:3683`/`:4173` are stale) and are `QWindow` virtuals reached through the base
  name — they survive any re-point unchanged, verified: `OmWrenWindow` declares neither.
- `renderLater` connects (`OmSimulationView.cpp:849-922`, `OmMainWindow.cpp:1402`) and
  `resized()` connects (`OmSimulationView.cpp:779/783`, `OmMainWindow.cpp:1085/1105`):
  **zero edits** — base→derived PMF conversion in `connect` handles the move. ⚠️ But this is a
  RUNTIME binding: verify by resizing the pane and watching device pop-outs reposition, not by
  the build log.
- `OmVideoRecorder.cpp` — actual surface (runbook's `:264-278`/`:366` are stale):
  `:67` (the static), `:171/:227/:292/:305` (`videoImageReady` connects), `:293/:306` (PBO
  virtuals), `:390-403` (E2's wgpu arm + the WREN `requestGrabWindowBuffer` arm). E2 built a
  PARALLEL wgpu arm and left the WREN arm untouched by design — so the PBO triple +
  `videoImageReady` all stay in `OmWrenWindow` at D1.3, and D1.4 deletes the WREN-arm call
  sites.
- ⚠️ **Missing from the runbook, structural**: `OmSimulationView.cpp:1099` calls
  `mView3D->resizeWren(...)` — an EXTERNAL caller, which is why `OmView3D.hpp:107` widens it
  to public. Keep the name and the public widening. Also missing:
  `OmSimulationView.cpp:1118` (`updateWrenViewportDimensions`, §6.3) and
  `OmVideoRecorder.cpp:67` (the static).
- `OmMultimediaStreamingServer.cpp:59`: no change. `OmWgpuView.cpp:2342-2353`: the
  `instance()` decision above; dies with C9 either way.
- ⛔ **`OmGuiApplication.cpp:528/:568` (the headless window): NO D1.3 WORK — the runbook's E5
  row is WRONG here.** `OmGlWindow::initialize()` is `create()` and nothing else; re-pointing
  the headless window at it leaves `wr_gl_state` uninitialised and every node's
  `createWrenObjects()` executing `wr_*` against dead WREN — `--no-window` camera rendering
  silently produces nothing (the runbook's own silent-failure entry). The REAL replacement is
  the already-shipped `OMNISIM_NO_GL` + wgpu wrenless path (`OmAbstractCamera.cpp:314-362`,
  `OmLidar` Lane-E1 wrenless setup): at D1.4, `setupNoWindow()`'s two branches collapse into
  the `noGl` branch and the headless window + its `QOffscreenSurface` + `initializeOpenGlInfo`
  triplet are deleted. `OmWrenRenderingContext::setWrenRenderingContext(160,120)` survives
  (D0b: not WREN).

## 4. The GL-3.3 fatal → degrade (behind `OMNISIM_GL_OPTIONAL`, default OFF)

Who genuinely needs live GL: WREN itself (`wr_gl_state::init` holds the tree's ONLY
`gladLoadGL()` — `src/wren/GlState.cpp:115`), the GL present blit, `swapBuffers`, device
pop-out windows (`OmRenderingDeviceWindow.cpp:118` asserts the stored context),
`OmSysInfo::initializeOpenGlInfo` (glad fn-ptrs — a NULL CALL, not garbage, if glad never
loaded), `checkRendererCapabilities`, `GpuPassTimer`. Who does NOT: the wgpu Vulkan child
surface present (`OmView3D.cpp:2151-2182, 3543`) and headless/presentation-free frames.

Design: in the `OmGlWindow` ctor, replace the fatals with — hatch OFF ⇒ fatal verbatim
(byte-identical shipped behaviour); hatch ON ⇒ `OmWrenOpenGlContext::destroy()` + one loud
warning. Expressing the degrade as "no GL context exists" reuses the ALREADY-SHIPPED
`OMNISIM_NO_GL` contract that six node files branch on, instead of inventing a second path.

> 🔴 **BLOCKING PRE-REQ (real bug, one line):** `OmWrenOpenGlContext::destroy()`
> (`OmWrenOpenGlContext.cpp:30-33`) deletes `mWrenContext` and never nulls it, so
> `isInitialized()` returns true on a dangling pointer. Add `mWrenContext = NULL;`.

Nine call sites must gate on `isInitialized()` for the degrade to hold (each otherwise a
segfault or garbage): `OmGuiApplication.cpp:484-486/:565-567`, `OmWrenWindow::initialize`
head, `OmView3D.cpp:1412`, `:1420`, `:1450` (⚠️ re-enters `initialize()` EVERY FRAME when GL
absent — latch it), `:3544` (the blit), `:3602` (returns an uninitialised snapshot buffer),
`GpuPassTimer.cpp:31`. `OmWrenWindow.cpp:208/:411` are already correct.
Unanswerable without a GL-less host (R8): whether `setSurfaceType(OpenGLSurface)` still
yields a usable platform window there. Leave it in D1.3 with a TODO naming
`OmView3D.cpp:2153-2157` as the surface that actually presents.

## 5. Ordered diff plan (every step compiles)

0. The `destroy()` null fix (independent, do first).
1. Delete `renderImmediately()` + `blitMainFrameBufferToScreen()` — **both have zero callers
   repo-wide**; `renderImmediately`'s own comment documents a caller that no longer exists
   (`OmWgpuView.cpp:2343-2346` says "do NOT force a render here").
2. Makefile: add `OmGlWindow.cpp` to `QT_SOURCES` (moc derives automatically; vpath/`-Igui`
   already present).
3. **The atomic step**: new `OmGlWindow.{hpp,cpp}` + the `OmWrenWindow` split + the
   `:378/:380` accessor fix + (if moving the singleton) `OmWgpuView.cpp:2342`. Clean-build
   `OmWrenWindow.moc.o` + `OmView3D.moc.o` — a stale `.moc.o` is the documented trap.
4. The §4 degrade + nine gates, behind the hatch (separable, default-off).
5. Optional prep: split `updateWrenViewportDimensions` (§6.3).

Riskiest: step 3's moc boundary — broken `connect`s are RUNTIME stderr warnings, not compile
errors, and both `resized()` consumers look fine in a screenshot when broken.

**E5 exit evidence:** build green with `OmWrenWindow` still present; a movie recorded on both
arms; a `--stream` frame served; `OMNISIM_NO_WINDOW=1` camera world produces non-empty images.

## 6. Corrections to the runbook's C8 scoping (all with evidence)

1. Stale line numbers: `OmView3D.cpp:3683/:4173` → `:3804/:4294`; `OmVideoRecorder.cpp:264-278/:366`
   → `:67/:171/:227/:292-306/:390-403`; hpp is 131 lines not 127.
2. Three missing consumers, one structural (`OmSimulationView.cpp:1099` external `resizeWren`).
3. A missing SEVENTH responsibility: `updateWrenViewportDimensions` (cpp:190-194) — one line
   dies with WREN, but `OmVideoRecorder::setScreenPixelRatio` SURVIVES; unsplit, D1.4 silently
   drops the recorder's DPR path.
4. Two scoped members are already dead (`renderImmediately`, `blitMainFrameBufferToScreen`).
5. E5's "incl. the headless second window" is wrong for D1.3 (see §3) — its replacement is a
   shipped mode, not new code.
6. `resized()` fires TWICE per resize today (base emit + `OmView3D.cpp:1446`), arity depending
   on GL init state; masked by one-shot connects. Record, do NOT fix in D1.3 (behaviour change
   = own commit).
7. The runbook's "six responsibilities, three WREN" miscounts: honest split is three pure-Qt,
   three pure-WREN, **three SPLIT** (`initialize`, `resizeWren`, `updateWrenViewportDimensions`)
   plus two dead. The split ones are where the work is.
8. Verbatim-copy hazards: `cpp:467` passes `1.0` to an int param (`:581` passes `1`); moving
   the fatal strings changes their `tr()` context and orphans `.ts` entries.
