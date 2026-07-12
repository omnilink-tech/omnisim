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

#ifndef WB_WGPU_VIEW_HPP
#define WB_WGPU_VIEW_HPP

//
// WbWgpuView — R4 step 1/2: a standalone on-screen window that wgpu
// presents into. GUI half of the R4 increments (render half is
// WbWgpuSurface).
//
//   step 1: clear-and-present (cycling colour) — proved the chain.
//   step 2: a lit, depth-tested rotating box rendered through the scene
//           pipeline into the surface — proves the surface does real 3D.
//
// A QWindow with surfaceType=VulkanSurface (NOT OpenGLSurface) so Qt does
// not set an OpenGL pixel format on the HWND — that lets wgpu own a
// D3D12/Vulkan swapchain on it without colliding with WREN's GL context on
// the (separate) main view.
//
// Deliberately NOT a Q_OBJECT: it adds no signals/slots (the animation
// timer uses a lambda), so it needs no moc and lives in OTHER_SOURCES.
//
// Created by WbView3D only when OMNISIM_VIEW3D_WGPU=1, and only meaningful
// on a wgpu-ON build. Otherwise the surface stays unusable and the window
// just logs once and idles.
//

#include <QtGui/QWindow>

class WbWgpuSurface;
class WbWgpuMeshCache;
class WbWgpuTextureCache;
class WbWgpuRenderTarget;
class WbVulkanBackend;
class WbMatrix4;
class WbSolid;
class QTimer;
class QMouseEvent;

class WbWgpuView : public QWindow {
public:
  explicit WbWgpuView(QWindow *parent = nullptr);
  ~WbWgpuView() override;

protected:
  void exposeEvent(QExposeEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;    // click -> handle-grab or wgpu pick
  void mouseMoveEvent(QMouseEvent *event) override;     // drag a grabbed manipulator handle
  void mouseReleaseEvent(QMouseEvent *event) override;  // end a handle drag

private:
  void ensureBackend();   // fetch the wgpu backend + mesh cache — NO window/exposure needed
  void ensureSurface();   // lazy-build the on-screen wgpu surface once exposed
  void renderTick();      // one animated frame (lit rotating box, or clear fallback)
  void runSelfCheck();    // capture frame(s), write pixel stats to a file (no eyes needed)
  bool runDeviceInsetCheck();  // device-output inset w/ a live camera feed; true once verdict written
  bool drawBox(double phase, void *rgbaOut);  // render the box at `phase`; rgbaOut optional readback
  // Build a +X-forward (Webots camera convention) matrix from the live main
  // Viewpoint. Returns false if no world/viewpoint is loaded. horizFov out in rad.
  bool buildViewpointCamera(WbMatrix4 &cam, double &horizFov) const;
  // Render the live world from the live Viewpoint into the surface (step 3:
  // match the live camera). rgbaOut optional readback; outDraws optional count.
  bool drawWorld(void *rgbaOut, int *outDraws = nullptr);
  // Column-major view-proj for the live world from the live Viewpoint (camera +
  // aspect-corrected FOV). False if no world/viewpoint. Shared by draw + pick so
  // a click maps to exactly what's displayed.
  bool worldViewProj(float out16[16]) const;
  // R4 step-3c-A.1: pick — render the world flat with encoded per-draw IDs to an
  // offscreen target, read back the texel at (px,py) in pane pixels, decode, and map
  // the draw index to its top-level WbSolid. Returns the picked solid, or nullptr on
  // miss/failure.
  WbSolid *pickSolidAt(int px, int py);
  // R4 step-3c-A: which translate-handle axis (0=X,1=Y,2=Z) of `sel` the pane pixel
  // (px,py) is on, projected through `vp16` at (w,h); -1 if none within the threshold.
  int pickHandleAxis(int px, int py, WbSolid *sel, const float *vp16, int w, int h, double len) const;
  // R4 step-3c-A: which rotate-gizmo ring (0=X,1=Y,2=Z) of `sel` the pixel is on (screen
  // distance to the projected ring circle of radius `len`), -1 if none.
  int pickRotateRing(int px, int py, WbSolid *sel, const float *vp16, int w, int h, double len) const;
  int pixelWidth() const;
  int pixelHeight() const;

  WbVulkanBackend *mBackend = nullptr;   // non-owning
  WbWgpuSurface *mSurface = nullptr;
  WbWgpuMeshCache *mMeshCache = nullptr;
  WbWgpuTextureCache *mTextureCache = nullptr;  // R4 material fidelity: albedo uploads
  WbWgpuRenderTarget *mPickTarget = nullptr;  // offscreen RGBA8 for ID picking
  QTimer *mTimer = nullptr;
  double mPhase = 0.0;
  bool mInitAttempted = false;
  bool mSelfCheckDone = false;
  bool mSurfaceLineCheckDone = false;  // one-shot live-pane bounding-overlay verification
  bool mDeviceInsetDone = false;       // one-shot device-output-inset check (retries until camera enabled)
  // R4 step-3c-A: active translate-manipulator drag (the gizmo handle the user grabbed).
  int mGrabAxis = -1;                 // 0=X,1=Y,2=Z; -1 = not translate-dragging
  WbSolid *mGrabSolid = nullptr;      // solid being translated
  double mGrabStartT[3] = {0, 0, 0};  // its translation at grab time
  int mGrabStartPx = 0, mGrabStartPy = 0;  // pane pixel at grab time
  // Rotate-drag state: which ring axis is grabbed (0/1/2; -1 none), the solid's rotation
  // at grab time (axis x,y,z + angle), and the mouse angle around the gizmo centre then.
  int mGrabRotateAxis = -1;
  WbSolid *mGrabRotateSolid = nullptr;
  double mGrabStartRot[4] = {0, 0, 1, 0};
  double mGrabStartAngle = 0.0;
  // Fixed rotation axis in the PARENT frame (= R_localStart · unitAxis), captured at grab.
  // Parent-frame (not world) so composing deltaM·startRotation is correct even when the
  // solid sits under a rotated/nested parent; for a top-level solid it equals the world axis.
  double mGrabRotateParentAxis[3] = {0, 0, 1};
  static constexpr double kHandleLen = 0.25;
  int mLastWorldDraws = 0;  // shapes harvested last drawWorld — gates the self-check
};

#endif
