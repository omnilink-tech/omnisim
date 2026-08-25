// Copyright 1996-2024 Cyberbotics Ltd.
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
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#ifndef OM_VIEW_3D_HPP
#define OM_VIEW_3D_HPP

//
// Description: 3D window for displaying the scene and for switching the display mode
//

#include "OmAction.hpp"
#include "OmBaseNode.hpp"
#include "OmGlWindow.hpp"

#include <QtCore/QMap>


#include <QtCore/QElapsedTimer>
#include <QtCore/QPoint>
#include <QtCore/QSet>

#include <array>
#include <vector>
#include <atomic>
#include <memory>
#include <thread>

#include "OmWgpuRenderTarget.hpp"    // OmWgpuSolidDraw (cached main-view draw list)
#include "OmWgpuSceneRenderer.hpp"   // OmWgpuDrawRefresh

class QTimer;

class OmAbstractPose;
class OmCamera;
class OmDragKinematicsEvent;
class OmDragForceEvent;
class OmDragTorqueEvent;
class OmDragOverlayEvent;
class OmDragResizeHandleEvent;
class OmDragRotateAroundWorldVerticalAxisEvent;
class OmDragRotateAroundAxisEvent;
class OmDragTranslateAlongAxisEvent;
class OmMatter;
class OmWrenRenderingContext;
class OmRobot;
class OmSolid;
class OmSimulationWorld;
class OmTouchSensor;
class OmWheelEvent;
class OmScenePicker;
// R4 3c-B: wgpu main-view backend (forward decls — pointers only; no wgpu headers leak into the hpp).
class OmVulkanBackend;
class OmWgpuMeshCache;
class OmWgpuTextureCache;
class OmWgpuRenderTarget;

class OmView3D : public OmGlWindow {
  Q_OBJECT;

public:
  explicit OmView3D();
  virtual ~OmView3D() override;

  void setParentWidget(QWidget *widget) { mParentWidget = widget; }

  // accessor
  OmWrenRenderingContext *wrenRenderingContext() const { return mWrenRenderingContext; }
  // rendering
  void showBlackRenderingOverlay();
  void hideBlackRenderingOverlay();
  // in case the context menu show is triggered, return the selected OmMatter node
  const OmMatter *remoteMouseEvent(QMouseEvent *event);
  void remoteWheelEvent(QWheelEvent *event);

  void prepareWorldLoading();
  void setWorld(OmSimulationWorld *w);
  // Suspends the wgpu main-view render path while a world (re)load is in flight, so a paint event
  // firing mid-teardown can't drive wgpu against a half-freed world/scene (the reload crash); the view
  // falls back to WREN, which reloads safely. Set true at the very start of (re)load, cleared once the
  // new world is ready.
  void setWgpuMainViewSuspended(bool suspended) { mWgpuMainViewSuspended = suspended; }
  void requestScreenshot() { mScreenshotRequested = true; }
  void resetScreenshotRequest() { mScreenshotRequested = false; }
  void cleanupEvents();
  void cleanupOptionalRendering();
  void cleanupFullScreenOverlay();
  void restoreOptionalRendering(const QStringList &enabledCenterOfMassNodeNames,
                                const QStringList &enabledCenterOfBuoyancyNodeNames,
                                const QStringList &enabledSupportPolygonNodeNames) const;
  void setUserInteractionDisabled(OmAction::OmActionKind action, bool disabled);

  void enableResizeManipulator(bool enabled);
  void resizeWren(int width, int height) override;

  // Screenshot/stream grab that matches what the pane actually shows: re-renders one synchronous
  // wgpu frame WITH readback and returns those pixels whenever wgpu owns the main view; otherwise
  // defers to the WREN GL-buffer grab. Fixes stale-GL screenshots, thumbnails and --stream frames
  // under a wgpu main view.
  // W2 (WREN retirement): this used to gate on mWgpuPresentedLastFrame, which is false in every
  // session with no exposed window to present to (--minimize / --no-window, i.e. the harness) and
  // also right after the offScreen render the screenshot path performs on the way here -- so the
  // grab silently returned WREN pixels. It now ASKS renderMainFrameViaWgpu instead of trusting a
  // flag (that call returns false cheaply, before any allocation, whenever wgpu does not own the
  // main view), and renders the grab presentation-free when nothing was presented.
  QImage grabWindowBufferNow() override;


  void logWrenStatistics();
  void handleModifierKey(QKeyEvent *event, bool pressed);

  void disableOptionalRenderingAndOverLays();
  void restoreOptionalRenderingAndOverLays();

public slots:
  void refresh();
  void setShowRenderingDevice(bool checked);
  void unleashAndClean();

protected slots:
  // cppcheck-suppress virtualCallInConstructor
  void renderNow(bool culling = true, bool offScreen = false) override;

protected:
  void initialize() override;

  virtual void leaveEvent(QEvent *event);
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  // replaces QWidget::mouseDoubleClickEvent() which is never called under Windows (Qt bug ?)
  virtual void mouseDoubleClick(QMouseEvent *event);
  void keyReleaseEvent(QKeyEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void focusInEvent(QFocusEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;

signals:
  void mainRenderingStarted(bool fromPhysics);
  void mainRenderingEnded(bool fromPhysics);
  void mouseDoubleClicked(QMouseEvent *event);
  void screenshotReady();
  void applicationActionsUpdateRequested();
  void contextMenuRequested(const QPoint &pos, QWidget *parentWidget);

private:
  // R4 3c-B: backend-dispatch seam. Renders the current main frame through the wgpu backend when the
  // active Viewpoint selects it (renderBackend "wgpu"/"vulkan") AND it is available; returns true if it
  // presented the frame. Returns false for the WREN default (the caller then runs the byte-identical WREN
  // path) and whenever the wgpu render is unavailable — so a wgpu failure degrades safely to WREN.
  bool renderMainFrameViaWgpu(bool culling, bool offScreen);

  // P10: cheap mirror of renderMainFrameViaWgpu's early gates — "would a main-view frame render
  // through wgpu right now?" — used only to pick the recording capture path at recording start.
  bool wgpuMainViewCurrentlyActive() const;
  // P10: true while a recording started under a wgpu main view is in flight (set/cleared by the
  // initVideoPBO / completeVideoPBOProcessing overrides).

  // R4 3c-B: wgpu main-view render resources (lazy — created only when a Viewpoint first selects wgpu).
  // The GL blit (RGBA→window) owns its own GL texture/FBO in OmWgpuGlBlit.cpp.
  OmVulkanBackend *mWgpuBackend = nullptr;
  OmWgpuMeshCache *mWgpuMeshCache = nullptr;
  OmWgpuTextureCache *mWgpuTextureCache = nullptr;
  OmWgpuRenderTarget *mWgpuRenderTarget = nullptr;  // cached offscreen target (recreated on resize)
  int mWgpuRtWidth = 0;
  int mWgpuRtHeight = 0;
  bool mWgpuMainViewUnavailable = false;   // sticky: a wgpu failure pins the main view back to WREN
  bool mWgpuMainViewSuspended = false;     // true during a world (re)load → wgpu main view falls back to WREN
  // Cached main-view draw list: the scene walk (collectWorldDraws) costs ~40 ms/frame on a 3.5k-draw
  // city while the structure barely changes — so cache the list + refresh only the model matrices
  // per frame. Full rebuild when: marked dirty (a referenced node was destroyed — see the destroyed()
  // connections), every 30 frames (textures/appearance staleness bound), or on world (re)load.
  std::vector<OmWgpuSolidDraw> mWgpuDrawList;
  std::vector<std::array<float, 16>> mWgpuModelList;
  std::vector<OmWgpuSceneRenderer::OmWgpuDrawRefresh> mWgpuRefreshList;
  // Synthetic-data ground truth: the owning OmSolid of each cached draw (parallel to
  // mWgpuDrawList), so the synth dump can label instance IDs with node names/DEFs.
  std::vector<OmSolid *> mWgpuDrawSolids;
  // OmniLight bake manager: one worker thread traces the probe volume over a SNAPSHOT of
  // world-space triangles (no live-scene pointers); the main thread polls and uploads.
  std::thread mOmniBakeThread;
  std::atomic<bool> mOmniBakeDone{false};
  bool mOmniBakeRunning = false;
  std::atomic<int> mOmniProgressDone{0};
  std::atomic<int> mOmniProgressTotal{0};
  float mOmniBlend = 1.0f;
  bool mOmniEverApplied = false;
  bool mOmniLocalsBaked = false;
  std::shared_ptr<struct OmniLightVolume> mOmniResult;
  uint64_t mOmniKey = 0;
  // W4a (WREN retirement): selection-highlight bookkeeping. The wgpu main view has no WREN
  // selection outline, so the selected top solid's draws are tinted toward mimosa in place.
  // The draw list is CACHED across frames (only model matrices are refreshed), so the tint
  // must be undone explicitly when the selection changes — otherwise it sticks to whatever
  // was selected first. Indices are only meaningful for the CURRENT list, so
  // invalidateWgpuDrawList() clears all three (and nulls the top, forcing a re-tint).
  const OmSolid *mWgpuSelTintTop = nullptr;
  std::vector<uint32_t> mWgpuSelTintIdx;
  std::vector<std::array<float, 3>> mWgpuSelTintRgb;
  std::vector<QMetaObject::Connection> mWgpuDrawListConns;  // destroyed() hooks of referenced nodes
  bool mWgpuDrawListDirty = true;
  int mWgpuDrawListAge = 0;
  // Bounded retry counter for incomplete collects (shapes whose WREN mesh is not created yet).
  int mWgpuCollectRetries = 0;
  // DYNAMIC CONTENT (Cloth / SoftBody / GranularGroup) is NOT part of the cached draw list --
  // its vertices or transforms change every simulation step -- so it is re-appended to the END
  // of mWgpuDrawList + mWgpuModelList on every frame. mWgpuDynamicDraws is how many entries last
  // frame appended IN TOTAL, i.e. exactly how many to trim off both before the cache-validity
  // check, which requires modelStorage.size() == mWgpuRefreshList.size(). Zeroed by
  // invalidateWgpuDrawList(), which clears both vectors.
  size_t mWgpuDynamicDraws = 0;
  // The per-kind breakdown of that same tail, for the OMNISIM_WGPU_REPORT line only. The tail is
  // laid out [deformables][granular][track][muscle], so each kind's ANIMATION telemetry -- the
  // measurement that the surface is moving rather than frozen at its first upload -- can scan
  // exactly its own sub-range and cannot be widened by a neighbour.
  size_t mWgpuDeformableDraws = 0;
  size_t mWgpuGranularDraws = 0;
  size_t mWgpuTrackDraws = 0;
  size_t mWgpuMuscleDraws = 0;
  std::vector<uint8_t> mWgpuRgba;          // persistent frame buffer (avoid an 11 MB alloc+zero per frame)
  // Window-swap presentation: an input-transparent Vulkan-surface CHILD window over this view's
  // client area; frames present GPU→GPU (OmWgpuSurface::presentTexture samples the offscreen
  // texture) — no readback, no GL upload, no blit. Falls back to the blit path if unavailable.
  QWindow *mWgpuPresentWindow = nullptr;
  class OmWgpuSurface *mWgpuPresentSurface = nullptr;
  // Which OmBackground the render target's image cubemap was uploaded for (raw pointer used only
  // as an identity token, never dereferenced) — re-uploads when the Background instance changes.
  void *mWgpuSkyCubeUploadedFor = nullptr;
  bool mWgpuPresentedLastFrame = false;  // the last main-view frame came from the wgpu path
  bool mWgpuGrabRequested = false;       // one-shot: force synchronous readback for a pixel grab
  // W2 (WREN retirement): one-shot request for a PRESENTATION-FREE wgpu frame -- no Vulkan-surface
  // child window, no surface acquire, no GL blit, no swapBuffers; the offscreen target is rendered
  // and read back synchronously and that is all. Set by grabWindowBufferNow() when nothing was
  // presented, so a screenshot works in a session that has no exposed window. Named for what it
  // does: such a frame is NOT presented, and mWgpuPresentedLastFrame is deliberately left false
  // for it rather than being faked (a flag that lies about presentation misleads the next reader).
  bool mWgpuOffscreenOnly = false;
  void invalidateWgpuDrawList();           // disconnect + clear + mark dirty

  QWidget *mParentWidget;
  QElapsedTimer mRenderPacingClock;   // monotonic since construction; drives the fixed-rate render throttle in refresh()
  double mNextRenderDueMs = 0.0;      // absolute due time (ms on mRenderPacingClock) of the next throttled render
  QElapsedTimer mFpsAccumulationTimer;
  qint64 mRenderedFrameCount = 0;
  static int cView3DNumber;
  int mProjectionMode;  // OmWrenRenderingContext::PM_*
  int mRenderingMode;  // OmWrenRenderingContext::RM_*
  QElapsedTimer *mMousePressTimer;
  QPoint mMousePressPosition;
  QMap<OmAction::OmActionKind, bool> mDisabledUserInteractionsMap;
  double mAspectRatio;
  bool mDisabledRenderingVisible;  // D1.4: the WREN "No Rendering" overlay is gone; state only

  OmWrenRenderingContext *mWrenRenderingContext;

  // Store options before creating thumbnail
  int mOptionalRenderingsMask;

  // Cleanup
  void cleanupDrags();
  void cleanupPhysicsDrags();
  void cleanupWheel();
  void cleanupCameraRecognizedObjectsOverlayIfNeeded();
  void cleanupPickers();

  // setters
  void setProjectionMode(int mode, bool updatePerspective, bool updateAction);
  void setRenderingMode(int mode, bool updatePerspective);

  // Others
  int stringToRenderingMode(const QString &s);
  int stringToProjectionMode(const QString &s);
  void rescaleFastModePanel();
  void enableOptionalRenderingFromPerspective();
  OmRobot *getCurrentRobot() const;
  static bool isContextMenuShortcut(const QMouseEvent *event);
  void selectNode(const QMouseEvent *event);

  bool mPhysicsRefresh;
  bool mScreenshotRequested;

  OmSimulationWorld *mWorld;
  OmTouchSensor *mTouchSensor;                     // touch sensor pressed by the mouse pointer if any
  OmCamera *mCameraUsingRecognizedObjectsOverlay;  // camera using the recognized object overlay if any

  // Drags
  OmDragForceEvent *mDragForce;
  OmDragTorqueEvent *mDragTorque;
  OmDragKinematicsEvent *mDragKinematics;
  OmDragOverlayEvent *mDragOverlay;
  OmDragResizeHandleEvent *mDragResize;
  OmDragTranslateAlongAxisEvent *mDragTranslate;
  OmDragRotateAroundWorldVerticalAxisEvent *mDragVerticalAxisRotate;
  OmDragRotateAroundAxisEvent *mDragRotate;
  bool mResizeHandlesDisabled;

  // Pickers
  OmScenePicker *mPicker;
  OmScenePicker *mControllerPicker;
  OmMatter *mPickedMatter;
  OmWheelEvent *mWheel;

  bool mMouseEventInitialized;
  QCursor mLastMouseCursor;
  Qt::MouseButtons mLastButtonState;
  bool mIsRemoteMouseEvent;
  OmMatter *mRemoteContextMenuMatter;

  // WASD free-fly camera
  QSet<int> mFlyKeys;
  QTimer *mFlyTimer;
  qint64 mFlyLastTickMs;
  bool mFlyMouseLook;
  QPoint mFlyMouseAnchor;
  QCursor mFlyPrevCursor;

  static bool isFlyKey(int key);
  void stopFly();
  void enterFlyMouseLook();
  void exitFlyMouseLook();

  // On selection changed
  void setCheckedShowCenterOfMassAction(OmSolid *selectedSolid);
  void setCheckedShowCenterOfBuoyancyAction(OmSolid *selectedSolid);
  void setCheckedShowSupportPolygonAction(OmSolid *selectedSolid);
  void setCheckedFollowObjectAction(OmSolid *selectedSolid);

private slots:
  void updateFlyCamera();
  void abortPhysicsDrag();
  void abortResizeDrag();
  void abortOverlayDrag();
  void followNone(bool checked);
  void followTracking(bool checked);
  void followMounted(bool checked);
  void followPanAndTilt(bool checked);
  void showCenterOfMass(bool checked);
  void showCenterOfBuoyancy(bool checked);
  void showSupportPolygon(bool checked);
  void notifyFollowObjectAction(int type);
  void restoreViewpoint();
  void setPerspectiveProjection();
  void setOrthographicProjection();
  void setPlain();
  void setWireframe();
  void setShowCoordinateSystem(bool show);
  void setShowBoundingObjects(bool show);
  void setShowContactPoints(bool show);
  void setShowConnectorAxes(bool show);
  void setShowJointAxes(bool show);
  void setShowCameraFrustums(bool show);
  void setShowRangeFinderFrustums(bool show);
  void setShowRadarFrustums(bool show);
  void setShowLidarRaysPaths(bool show);
  void setShowLidarPointClouds(bool show);
  void setHideAllCameraOverlays(bool hidden);
  void setHideAllRangeFinderOverlays(bool hidden);
  void setHideAllDisplayOverlays(bool hidden);
  void setShowDistanceSensorRays(bool show);
  void setShowLightSensorRays(bool show);
  void setShowLightsPositions(bool show);
  void setShowPenPaintingRays(bool show);
  void setShowSkeletonAction(bool show);
  void setShowNormals(bool show);
  void setShowPhysicsClustersAction(bool show);
  void setShowBoundingSphereAction(bool show);
  void setViewPointLocked(bool locked) { setUserInteractionDisabled(OmAction::LOCK_VIEWPOINT, locked); }
  void setSelectionDisabled(bool disabled) { setUserInteractionDisabled(OmAction::DISABLE_SELECTION, disabled); }
  void setContextMenuDisabled(bool disabled) { setUserInteractionDisabled(OmAction::DISABLE_3D_VIEW_CONTEXT_MENU, disabled); }
  void disableObjectMove(bool disabled);
  void disableApplyForceAndTorque(bool disabled) { setUserInteractionDisabled(OmAction::DISABLE_FORCE_AND_TORQUE, disabled); }
  void updateMousesPosition(bool fromMouseClick = false, bool fromMouseMove = false);

  void cleanWorld() { mWorld = NULL; }
  void updateViewport();
  void updateShadowState();
  void unleashPhysicsDrags();
  void onSelectionChanged(OmAbstractPose *selectedPose);
  void handleWorldModificationFromSupervisor();
};

#endif
