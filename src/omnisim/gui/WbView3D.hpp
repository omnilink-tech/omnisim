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

#ifndef WB_VIEW_3D_HPP
#define WB_VIEW_3D_HPP

//
// Description: 3D window for displaying the scene and for switching the display mode
//

#include "WbAction.hpp"
#include "WbBaseNode.hpp"
#include "WbWrenWindow.hpp"

#include <wren/camera.h>
#include <wren/viewport.h>

#include <QtCore/QElapsedTimer>
#include <QtCore/QPoint>
#include <QtCore/QSet>

#include <array>
#include <vector>

#include "WbWgpuRenderTarget.hpp"    // WbWgpuSolidDraw (cached main-view draw list)
#include "WbWgpuSceneRenderer.hpp"   // WbWgpuDrawRefresh

class QTimer;

class WbAbstractPose;
class WbCamera;
class WbDragKinematicsEvent;
class WbDragForceEvent;
class WbDragTorqueEvent;
class WbDragOverlayEvent;
class WbDragResizeHandleEvent;
class WbDragRotateAroundWorldVerticalAxisEvent;
class WbDragRotateAroundAxisEvent;
class WbDragTranslateAlongAxisEvent;
class WbMatter;
class WbWrenRenderingContext;
class WbContactPointsRepresentation;
class WbRobot;
class WbSolid;
class WbSimulationWorld;
class WbTouchSensor;
class WbWheelEvent;
class WbWrenPicker;
class WbWrenFullScreenOverlay;
// R4 3c-B: wgpu main-view backend (forward decls — pointers only; no wgpu headers leak into the hpp).
class WbVulkanBackend;
class WbWgpuMeshCache;
class WbWgpuTextureCache;
class WbWgpuRenderTarget;

class WbView3D : public WbWrenWindow {
  Q_OBJECT;

public:
  explicit WbView3D();
  virtual ~WbView3D() override;

  void setParentWidget(QWidget *widget) { mParentWidget = widget; }

  // accessor
  WbWrenRenderingContext *wrenRenderingContext() const { return mWrenRenderingContext; }
  // rendering
  void showBlackRenderingOverlay();
  void hideBlackRenderingOverlay();
  // in case the context menu show is triggered, return the selected WbMatter node
  const WbMatter *remoteMouseEvent(QMouseEvent *event);
  void remoteWheelEvent(QWheelEvent *event);

  void prepareWorldLoading();
  void setWorld(WbSimulationWorld *w);
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
  void updateVirtualRealityHeadsetOverlay();
  void restoreOptionalRendering(const QStringList &enabledCenterOfMassNodeNames,
                                const QStringList &enabledCenterOfBuoyancyNodeNames,
                                const QStringList &enabledSupportPolygonNodeNames) const;
  void setUserInteractionDisabled(WbAction::WbActionKind action, bool disabled);

  void enableResizeManipulator(bool enabled);
  void resizeWren(int width, int height) override;

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

  // R4 3c-B: wgpu main-view render resources (lazy — created only when a Viewpoint first selects wgpu).
  // The GL blit (RGBA→window) owns its own GL texture/FBO in WbWgpuGlBlit.cpp.
  WbVulkanBackend *mWgpuBackend = nullptr;
  WbWgpuMeshCache *mWgpuMeshCache = nullptr;
  WbWgpuTextureCache *mWgpuTextureCache = nullptr;
  WbWgpuRenderTarget *mWgpuRenderTarget = nullptr;  // cached offscreen target (recreated on resize)
  int mWgpuRtWidth = 0;
  int mWgpuRtHeight = 0;
  bool mWgpuMainViewUnavailable = false;   // sticky: a wgpu failure pins the main view back to WREN
  bool mWgpuMainViewSuspended = false;     // true during a world (re)load → wgpu main view falls back to WREN
  // Cached main-view draw list: the scene walk (collectWorldDraws) costs ~40 ms/frame on a 3.5k-draw
  // city while the structure barely changes — so cache the list + refresh only the model matrices
  // per frame. Full rebuild when: marked dirty (a referenced node was destroyed — see the destroyed()
  // connections), every 30 frames (textures/appearance staleness bound), or on world (re)load.
  std::vector<WbWgpuSolidDraw> mWgpuDrawList;
  std::vector<std::array<float, 16>> mWgpuModelList;
  std::vector<WbWgpuSceneRenderer::WbWgpuDrawRefresh> mWgpuRefreshList;
  std::vector<QMetaObject::Connection> mWgpuDrawListConns;  // destroyed() hooks of referenced nodes
  bool mWgpuDrawListDirty = true;
  int mWgpuDrawListAge = 0;
  std::vector<uint8_t> mWgpuRgba;          // persistent frame buffer (avoid an 11 MB alloc+zero per frame)
  // Window-swap presentation: an input-transparent Vulkan-surface CHILD window over this view's
  // client area; frames present GPU→GPU (WbWgpuSurface::presentTexture samples the offscreen
  // texture) — no readback, no GL upload, no blit. Falls back to the blit path if unavailable.
  QWindow *mWgpuPresentWindow = nullptr;
  class WbWgpuSurface *mWgpuPresentSurface = nullptr;
  void invalidateWgpuDrawList();           // disconnect + clear + mark dirty

  QWidget *mParentWidget;
  QElapsedTimer mLastRefreshTimer;
  QElapsedTimer mFpsAccumulationTimer;
  qint64 mRenderedFrameCount = 0;
  static int cView3DNumber;
  WrCameraProjectionMode mProjectionMode;
  WrViewportPolygonMode mRenderingMode;
  QElapsedTimer *mMousePressTimer;
  QPoint mMousePressPosition;
  QMap<WbAction::WbActionKind, bool> mDisabledUserInteractionsMap;
  double mAspectRatio;
  WbWrenFullScreenOverlay *mDisabledRenderingOverlay;
  WbWrenFullScreenOverlay *mLoadingWorldOverlay;
  WbWrenFullScreenOverlay *mVirtualRealityHeadsetOverlay;

  WbContactPointsRepresentation *mContactPointsRepresentation;
  WbWrenRenderingContext *mWrenRenderingContext;

  // Store options before creating thumbnail
  int mOptionalRenderingsMask;

  // Cleanup
  void cleanupDrags();
  void cleanupPhysicsDrags();
  void cleanupWheel();
  void cleanupCameraRecognizedObjectsOverlayIfNeeded();
  void cleanupPickers();

  // setters
  void setProjectionMode(WrCameraProjectionMode mode, bool updatePerspective, bool updateAction);
  void setRenderingMode(WrViewportPolygonMode mode, bool updatePerspective);

  // Others
  WrViewportPolygonMode stringToRenderingMode(const QString &s);
  WrCameraProjectionMode stringToProjectionMode(const QString &s);
  void rescaleFastModePanel();
  void enableOptionalRenderingFromPerspective();
  WbRobot *getCurrentRobot() const;
  void checkRendererCapabilities();
  static bool isContextMenuShortcut(const QMouseEvent *event);
  void selectNode(const QMouseEvent *event);

  bool mPhysicsRefresh;
  bool mScreenshotRequested;

  WbSimulationWorld *mWorld;
  WbTouchSensor *mTouchSensor;                     // touch sensor pressed by the mouse pointer if any
  WbCamera *mCameraUsingRecognizedObjectsOverlay;  // camera using the recognized object overlay if any

  // Drags
  WbDragForceEvent *mDragForce;
  WbDragTorqueEvent *mDragTorque;
  WbDragKinematicsEvent *mDragKinematics;
  WbDragOverlayEvent *mDragOverlay;
  WbDragResizeHandleEvent *mDragResize;
  WbDragTranslateAlongAxisEvent *mDragTranslate;
  WbDragRotateAroundWorldVerticalAxisEvent *mDragVerticalAxisRotate;
  WbDragRotateAroundAxisEvent *mDragRotate;
  bool mResizeHandlesDisabled;

  // Pickers
  WbWrenPicker *mPicker;
  WbWrenPicker *mControllerPicker;
  WbMatter *mPickedMatter;
  WbWheelEvent *mWheel;

  bool mMouseEventInitialized;
  QCursor mLastMouseCursor;
  Qt::MouseButtons mLastButtonState;
  bool mIsRemoteMouseEvent;
  WbMatter *mRemoteContextMenuMatter;

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
  void setCheckedShowCenterOfMassAction(WbSolid *selectedSolid);
  void setCheckedShowCenterOfBuoyancyAction(WbSolid *selectedSolid);
  void setCheckedShowSupportPolygonAction(WbSolid *selectedSolid);
  void setCheckedFollowObjectAction(WbSolid *selectedSolid);

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
  void setVirtualRealityHeadset(bool enable);
  void setVirtualRealityHeadsetPositionTracking(bool enable);
  void setVirtualRealityHeadsetOrientationTracking(bool enable);
  void setVirtualRealityHeadsetLeftEyeView(bool enable);
  void setVirtualRealityHeadsetRightEyeView(bool enable);
  void setVirtualRealityHeadsetNoEyeView(bool enable);
  void setVirtualRealityHeadsetAntiAliasing(bool enable);
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
  void setViewPointLocked(bool locked) { setUserInteractionDisabled(WbAction::LOCK_VIEWPOINT, locked); }
  void setSelectionDisabled(bool disabled) { setUserInteractionDisabled(WbAction::DISABLE_SELECTION, disabled); }
  void setContextMenuDisabled(bool disabled) { setUserInteractionDisabled(WbAction::DISABLE_3D_VIEW_CONTEXT_MENU, disabled); }
  void disableObjectMove(bool disabled);
  void disableApplyForceAndTorque(bool disabled) { setUserInteractionDisabled(WbAction::DISABLE_FORCE_AND_TORQUE, disabled); }
  void updateMousesPosition(bool fromMouseClick = false, bool fromMouseMove = false);

  void cleanWorld() { mWorld = NULL; }
  void updateViewport();
  void updateShadowState();
  void unleashPhysicsDrags();
  void onSelectionChanged(WbAbstractPose *selectedPose);
  void handleWorldModificationFromSupervisor();
};

#endif
