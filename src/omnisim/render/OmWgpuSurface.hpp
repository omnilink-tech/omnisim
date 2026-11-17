// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OM_WGPU_SURFACE_HPP
#define OM_WGPU_SURFACE_HPP

//
// OmWgpuSurface — R4 step 1 + 2: the on-screen wgpu target.
//
// Twin of OmWgpuRenderTarget (the offscreen RTT used by the Camera
// path), but instead of rendering to a texture we copy back, this owns
// a wgpu *surface* (swapchain) bound to a native window and PRESENTS to
// the screen. This is the piece R4 was missing: until now wgpu could
// render a pixel but could not put one on the display.
//
//   - presentClear (step 1): acquire → clear → present. Proved the
//     surface/configure/acquire/present chain works on real hardware.
//   - presentScene (step 2): acquire → depth-tested lit scene draws →
//     present. Reuses the kSolidLit shader + the 256-byte dynamic-offset
//     Scene-uniform layout proven by OmWgpuRenderTarget::clearAndDrawScene,
//     but renders into the surface's acquired backbuffer (no readback) with
//     a pipeline keyed to the SURFACE'S format (the offscreen pipeline is
//     RGBA8Unorm; a surface is typically BGRA8Unorm, and a render
//     pipeline's color-target format must match its attachment).
//
// The window handles are passed in as raw native handles, in the TAGGED
// OmWgpuNativeWindow struct below (HWND+HINSTANCE on Windows, Display*+XID
// on X11, wl_display*+wl_surface* on Wayland), so this render-layer class
// has no Qt dependency. Everything is gated on OMNISIM_WITH_VULKAN +
// WB_WGPU_NATIVE_AVAILABLE; the OFF build compiles to inert stubs.
//

#include <cstddef>
#include <cstdint>

class OmVulkanBackend;
struct OmWgpuSolidDraw;
class QWindow;

//
// OmWgpuNativeWindow — the platform-neutral native window handle.
//
// wgpu needs a DIFFERENT handle pair per windowing system, so the original
// (void *hwnd, void *hinstance) constructor only ever described Windows: on
// X11 the companion handle is a Display* and the window is an INTEGER XID,
// not a pointer; on Wayland both handles are pointers but neither is an HWND.
// Two bare void*s whose meaning silently changes per platform is exactly how
// this kind of code rots, so the handles travel as one TAGGED struct: `kind`
// selects which fields are meaningful and every other field stays null/zero.
//
// Build one with the named factories — never aggregate-initialise it, so a
// future platform cannot be bolted on by re-purposing an existing field.
//
// Nothing here needs a platform header: every wgpu surface-source struct takes
// void* / uint64_t, so this compiles identically on every host. Which SOURCE
// wgpu is handed is chosen at RUNTIME from `kind`, not at compile time — one
// Linux binary has to serve both an xcb session and a wayland session.
//
struct OmWgpuNativeWindow {
  enum class Kind : uint32_t {
    None = 0,        // no handle — the caller must fall back (GL blit)
    WindowsHwnd,     // -> WGPUSurfaceSourceWindowsHWND
    XlibWindow,      // -> WGPUSurfaceSourceXlibWindow
    XcbWindow,       // -> WGPUSurfaceSourceXCBWindow
    WaylandSurface,  // -> WGPUSurfaceSourceWaylandSurface
  };

  Kind kind = Kind::None;
  void *hwnd = nullptr;      // WindowsHwnd: HWND
  void *instance = nullptr;  // WindowsHwnd: HINSTANCE
  void *display = nullptr;   // Xlib: Display* | Xcb: xcb_connection_t* | Wayland: wl_display*
  void *surface = nullptr;   // Wayland: wl_surface*
  uint64_t windowId = 0;     // Xlib: Window (XID) | Xcb: xcb_window_t (32-bit, widened)

  static OmWgpuNativeWindow windowsHwnd(void *hwnd, void *hinstance) {
    OmWgpuNativeWindow n;
    n.kind = Kind::WindowsHwnd;
    n.hwnd = hwnd;
    n.instance = hinstance;
    return n;
  }
  static OmWgpuNativeWindow xlibWindow(void *display, uint64_t window) {
    OmWgpuNativeWindow n;
    n.kind = Kind::XlibWindow;
    n.display = display;
    n.windowId = window;
    return n;
  }
  static OmWgpuNativeWindow xcbWindow(void *connection, uint32_t window) {
    OmWgpuNativeWindow n;
    n.kind = Kind::XcbWindow;
    n.display = connection;
    n.windowId = window;
    return n;
  }
  static OmWgpuNativeWindow waylandSurface(void *display, void *surface) {
    OmWgpuNativeWindow n;
    n.kind = Kind::WaylandSurface;
    n.display = display;
    n.surface = surface;
    return n;
  }

  // True only when every field the `kind` actually needs is populated. A false
  // here is the caller's cue to skip wgpu presentation entirely and keep the
  // legacy readback+GL-blit path — never a reason to show a black window.
  bool isValid() const {
    switch (kind) {
      case Kind::WindowsHwnd:
        return hwnd != nullptr;
      case Kind::XlibWindow:
      case Kind::XcbWindow:
        return display != nullptr && windowId != 0;
      case Kind::WaylandSurface:
        return display != nullptr && surface != nullptr;
      case Kind::None:
        break;
    }
    return false;
  }

  const char *kindName() const {
    switch (kind) {
      case Kind::WindowsHwnd:
        return "windows-hwnd";
      case Kind::XlibWindow:
        return "xlib-window";
      case Kind::XcbWindow:
        return "xcb-window";
      case Kind::WaylandSurface:
        return "wayland-surface";
      case Kind::None:
        break;
    }
    return "none";
  }
};

// Resolve the running platform's native window handles from a live QWindow.
//
// DECLARED here but IMPLEMENTED in the GUI layer (gui/OmWgpuView.cpp), because
// it needs Qt and this render-layer class deliberately does not. It lives in
// one place so BOTH construction sites (OmWgpuView + OmView3D's present child
// window) share one implementation instead of each growing its own copy.
//
// Returns a Kind::None handle (isValid() == false) whenever the platform is
// unknown, the Qt native interface is unavailable, or the window has no native
// handle yet — the caller must then fall back rather than fail.
OmWgpuNativeWindow OmWgpuNativeWindowFromQtWindow(QWindow *window);

class OmWgpuSurface {
public:
  // nativeWindow carries the platform window handles (see OmWgpuNativeWindow).
  // On a non-wgpu build, on a platform with no wgpu surface source, or if
  // surface creation/configure fails, isUsable() stays false and present is a
  // no-op — the caller keeps its fallback path.
  OmWgpuSurface(OmVulkanBackend *backend, const OmWgpuNativeWindow &nativeWindow, uint32_t width,
                uint32_t height);
  ~OmWgpuSurface();

  OmWgpuSurface(const OmWgpuSurface &) = delete;
  OmWgpuSurface &operator=(const OmWgpuSurface &) = delete;

  bool isUsable() const { return mUsable; }
  uint32_t width() const { return mWidth; }
  uint32_t height() const { return mHeight; }

  // Reconfigure the swapchain for a new window size. No-op if the size
  // is unchanged or the surface is not usable.
  void resize(uint32_t width, uint32_t height);

  // Step 1: acquire the current backbuffer, clear it to (r,g,b,1), present.
  bool presentClear(float r, float g, float b);

  // Step 2: acquire the current backbuffer, render the given lit scene
  // draws (depth-tested) over a clear, present. viewProj16 / lightDirAmbient4
  // are column-major / (dir.xyz, ambient.w). Returns false if unusable or
  // the frame was skipped (surface Outdated/Lost — retry next frame).
  //
  // If rgbaOut is non-null AND the surface supports CopySrc (canReadback()),
  // the rendered backbuffer is copied back into rgbaOut (width*height*4 bytes,
  // in the surface's native channel order) BEFORE present — the numerical
  // self-check path that makes R4 verifiable without a human watching.
  // lineVerts/lineVertCount/lineColor4 (R4 step-3c-A, optional): after the lit scene
  // pass, draw `lineVertCount` line-segment vertices (stride-32 pos3+pad, two per
  // segment) in flat `lineColor4`, depth-tested against the scene — so overlays
  // (bounding objects, etc.) appear in the LIVE pane. Null/0 → no overlay (the
  // existing callers stay byte-identical).
  // Two optional line overlays: the depth-tested batch (lineVerts*, for wireframes
  // that read as 3D, e.g. bounding objects) and the always-on-top batch (topVerts*,
  // depthCompare=Always, for markers that sit inside geometry — joint axes, COM,
  // contact — so they stay visible). Both null/0 → no overlay (byte-identical).
  bool presentScene(const float *viewProj16, const float *lightDirAmbient4,
                    const OmWgpuSolidDraw *draws, uint32_t numDraws,
                    float clearR, float clearG, float clearB, void *rgbaOut = nullptr,
                    const float *lineVerts = nullptr, uint32_t lineVertCount = 0,
                    const float *lineColor4 = nullptr, const float *topVerts = nullptr,
                    uint32_t topVertCount = 0, const float *topColor4 = nullptr,
                    const float *cameraWorldPos3 = nullptr);

  // Window-swap present: acquire the backbuffer and draw the given offscreen texture view
  // (kTexturedQuad, full rect) into it — the GPU-to-GPU presentation that replaces the
  // readback/GL-upload/blit path for the wgpu main view.
  bool presentTexture(void *srcTextureView);

  // True if the configured surface supports CopySrc, so presentScene's
  // rgbaOut readback is available. Channel order matches the surface format
  // (formatIsBgra() distinguishes BGRA8 vs RGBA8).
  bool canReadback() const { return mCanReadback; }
  bool formatIsBgra() const;

private:
  void configure();          // (re)configure the surface at mWidth x mHeight
  bool ensureScenePipeline(); // build kSolidLit pipeline keyed to mFormat (once)
  bool ensureSceneTexPipeline();            // kSolidLitTextured (albedo), keyed to mFormat
  bool ensureLineOverlayPipeline();         // kSolidPick + LineList, depth-tested
  bool ensureLineNoDepthOverlayPipeline();  // kSolidPick + LineList, always-on-top
  bool ensureSceneDepth();    // build/resize the Depth24Plus attachment to mWidth x mHeight
  bool ensurePresentTexPipeline();  // kTexturedQuad keyed to mFormat (window-swap present)
  void releaseAll();

  // Window-swap present resources (kTexturedQuad): pipeline keyed to mFormat + full-rect uniform +
  // sampler; the bind group is rebuilt when the source texture view changes (target resize).
  void *mPresentPipeline = nullptr;
  void *mPresentShaderModule = nullptr;
  void *mPresentBgl = nullptr;
  void *mPresentUb = nullptr;
  void *mPresentSampler = nullptr;
  void *mPresentBg = nullptr;
  void *mPresentSrcView = nullptr;

  OmVulkanBackend *mBackend = nullptr;  // non-owning
  void *mSurface = nullptr;             // WGPUSurface (opaque)
  uint32_t mWidth = 0;
  uint32_t mHeight = 0;
  uint32_t mFormat = 0;                 // WGPUTextureFormat (opaque enum value)
  bool mConfigured = false;
  bool mUsable = false;

  // Scene pipeline (step 2) — mirrors OmWgpuRenderTarget's, but its
  // color target is mFormat (the surface format) not RGBA8Unorm.
  void *mScnShaderModule = nullptr;
  void *mScnPipeline = nullptr;
  void *mScnBindGroupLayout = nullptr;
  void *mScnUniformBuffer = nullptr;
  size_t mScnUniformBufferSize = 0;
  // R4 material fidelity: textured-lit pipeline (kSolidLitTextured) for the LIVE pane —
  // 6-entry layout (uniform@0 + albedo@1 + roughness@2 + metalness@3 + normal@4 + sampler@5).
  // Used per-draw for draws carrying a textureView; flat draws stay on mScnPipeline.
  void *mScnTexShaderModule = nullptr;
  void *mScnTexPipeline = nullptr;
  void *mScnTexBindGroupLayout = nullptr;
  void *mScnTexSampler = nullptr;
  // 1×1 default textures for the map slots a draw may lack (byte-identical for albedo-only):
  // white roughness, black metalness (dielectric), flat normal (128,128,255 → no perturb).
  void *mScnDefaultWhiteTex = nullptr;
  void *mScnDefaultWhiteView = nullptr;
  void *mScnDefaultBlackTex = nullptr;
  void *mScnDefaultBlackView = nullptr;
  void *mScnDefaultNormalTex = nullptr;
  void *mScnDefaultNormalView = nullptr;
  void *mScnDepthTexture = nullptr;
  void *mScnDepthView = nullptr;
  uint32_t mScnDepthW = 0;
  uint32_t mScnDepthH = 0;

  // R4 step-3c-A: line-overlay pipeline (kSolidPick shader + LineList, mFormat
  // target). Reuses mScnBindGroupLayout; mLineUniformBuffer holds one slot
  // (viewProj + identity model + flat line colour).
  void *mLineShaderModule = nullptr;
  void *mLinePipeline = nullptr;          // depth-tested
  void *mLineNoDepthPipeline = nullptr;   // always-on-top
  void *mLineUniformBuffer = nullptr;     // 2 slots: [0]=depth batch colour, [1]=top batch colour

  // Self-check readback (presentScene rgbaOut). CopySrc must be in the
  // surface's supported usages; the readback buffer is row-stride aligned.
  uint32_t mUsage = 0;
  bool mCanReadback = false;
  void *mReadbackBuffer = nullptr;
  size_t mReadbackSize = 0;
};

#endif
