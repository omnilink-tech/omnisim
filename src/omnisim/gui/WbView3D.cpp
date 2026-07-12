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

#include "WbView3D.hpp"

#include "WbAbstractDragEvent.hpp"
#include "WbAbstractPose.hpp"
#include "WbActionManager.hpp"
#include "WbBaseNode.hpp"
#include "WbBox.hpp"
#include "WbCamera.hpp"
#include "WbCapsule.hpp"
#include "WbCone.hpp"
#include "WbContactPointsRepresentation.hpp"
#include "WbContextMenuGenerator.hpp"
#include "WbCylinder.hpp"
#include "WbBackground.hpp"
#include "WbDirectionalLight.hpp"
#include "WbFog.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbRgb.hpp"
#include "WbDragOverlayEvent.hpp"
#include "WbDragPoseEvent.hpp"
#include "WbDragResizeEvent.hpp"
#include "WbDragScaleEvent.hpp"
#include "WbDragSolidEvent.hpp"
#include "WbDragViewpointEvent.hpp"
#include "WbElevationGrid.hpp"
#include "WbGroup.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbLog.hpp"
#include "WbMatter.hpp"
#include "WbMessageBox.hpp"
#include "WbMouse.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeDebugger.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPerspective.hpp"
#include "WbPlane.hpp"
#include "WbPose.hpp"
#include "WbPreferences.hpp"
#include "WbRenderingDevice.hpp"
#include "WbRenderingDeviceWindowFactory.hpp"
#include "WbRobot.hpp"
#include "WbSelection.hpp"
#include "WbSimulationState.hpp"
#include "WbSimulationWorld.hpp"
#include "WbSkin.hpp"
#include "WbSolid.hpp"
#include "WbSphere.hpp"
#include "WbStandardPaths.hpp"
#include "WbSupervisorUtilities.hpp"
#include "WbRenderBackend.hpp"  // R4 3c-B: per-Viewpoint backend dispatch for the main view
// R4 3c-B: wgpu main-view render path (offscreen wgpu render → GL blit into this WREN window).
#include "WbMatrix4.hpp"
#include "WbRotation.hpp"
#include "WbSFRotation.hpp"
#include "WbSFVector3.hpp"
#include "WbVector3.hpp"
#include "WbVulkanBackend.hpp"
#include "WbWgpuGlBlit.hpp"
#include "WbWgpuMeshCache.hpp"
#include "WbWgpuRenderTarget.hpp"
#include "WbWgpuSurface.hpp"
#ifdef _WIN32
#  include <windows.h>  // GetModuleHandleW for the wgpu present-surface HINSTANCE
#endif
#include "WbWgpuSceneRenderer.hpp"
#include "WbWgpuTextureCache.hpp"
#include "WbWorld.hpp"
#include "WbSysInfo.hpp"
#include "WbTouchSensor.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbVersion.hpp"
#include "WbVideoRecorder.hpp"
#include "WbViewpoint.hpp"
#include "WbVisualBoundingSphere.hpp"
#include "WbWheelEvent.hpp"
#include "WbWorldInfo.hpp"
#include "WbWrenFullScreenOverlay.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPicker.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenTextureOverlay.hpp"

#ifdef _WIN32
#include "WbVirtualRealityHeadset.hpp"
#endif

#include <QtCore/QDateTime>
#include <QtCore/QFile>
#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtGui/QAction>

#include <cstdio>
#include <QtGui/QImage>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QScreen>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMenu>

#include <wren/camera.h>
#include <wren/config.h>
#include <wren/gl_state.h>
#include <wren/scene.h>
#include <wren/transform.h>
#include <wren/viewport.h>

int WbView3D::cView3DNumber = 0;

WbView3D::WbView3D() :
  WbWrenWindow(),
  mParentWidget(NULL),
  mMousePressTimer(NULL),
  mAspectRatio(1.0),
  mDisabledRenderingOverlay(NULL),
  mLoadingWorldOverlay(NULL),
  mVirtualRealityHeadsetOverlay(NULL),
  mContactPointsRepresentation(NULL),
  mWrenRenderingContext(NULL),
  mPhysicsRefresh(false),
  mScreenshotRequested(false),
  mWorld(NULL),
  mTouchSensor(NULL),
  mCameraUsingRecognizedObjectsOverlay(NULL),
  mDragForce(NULL),
  mDragTorque(NULL),
  mDragKinematics(NULL),
  mDragOverlay(NULL),
  mDragResize(NULL),
  mDragTranslate(NULL),
  mDragVerticalAxisRotate(NULL),
  mDragRotate(NULL),
  mResizeHandlesDisabled(false),
  mPicker(NULL),
  mControllerPicker(NULL),
  mPickedMatter(NULL),
  mWheel(NULL),
  mMouseEventInitialized(false),
  mLastButtonState(Qt::NoButton),
  mIsRemoteMouseEvent(false),
  mRemoteContextMenuMatter(NULL),
  mFlyTimer(NULL),
  mFlyLastTickMs(0),
  mFlyMouseLook(false) {
  QDir::addSearchPath("gl", WbStandardPaths::resourcesPath() + "wren");

  mLastRefreshTimer.start();
  setObjectName("View3D");

  WbWrenRenderingContext::setWrenRenderingContext(width(), height());
  mWrenRenderingContext = WbWrenRenderingContext::instance();

  WbActionManager *actionManager = WbActionManager::instance();
  // render after each simulation step and when simulation mode changed
  connect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this, &WbView3D::refresh,
          Qt::UniqueConnection);
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbView3D::refresh, Qt::UniqueConnection);
  connect(WbSimulationState::instance(), &WbSimulationState::renderingStateChanged, this, &WbView3D::refresh,
          Qt::UniqueConnection);
  // clean up pending drag-force / drag-torque when simulation restarts
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbView3D::unleashPhysicsDrags);
  // update mouses if required
  connect(WbSimulationState::instance(), SIGNAL(physicsStepStarted()), this, SLOT(updateMousesPosition()));
  // viewpoint
  connect(actionManager->action(WbAction::FOLLOW_NONE), &QAction::triggered, this, &WbView3D::followNone);
  connect(actionManager->action(WbAction::FOLLOW_TRACKING), &QAction::triggered, this, &WbView3D::followTracking);
  connect(actionManager->action(WbAction::FOLLOW_MOUNTED), &QAction::triggered, this, &WbView3D::followMounted);
  connect(actionManager->action(WbAction::FOLLOW_PAN_AND_TILT), &QAction::triggered, this, &WbView3D::followPanAndTilt);
  connect(actionManager->action(WbAction::RESTORE_VIEWPOINT), &QAction::triggered, this, &WbView3D::restoreViewpoint);
  // signal the simulation state about a rendering
  connect(actionManager->action(WbAction::ORTHOGRAPHIC_PROJECTION), &QAction::triggered, this,
          &WbView3D::setOrthographicProjection);
  connect(actionManager->action(WbAction::PERSPECTIVE_PROJECTION), &QAction::triggered, this,
          &WbView3D::setPerspectiveProjection);
  connect(actionManager->action(WbAction::PLAIN_RENDERING), &QAction::triggered, this, &WbView3D::setPlain);
  connect(actionManager->action(WbAction::WIREFRAME_RENDERING), &QAction::triggered, this, &WbView3D::setWireframe);
  connect(actionManager->action(WbAction::LOCK_VIEWPOINT), &QAction::triggered, this, &WbView3D::setViewPointLocked);
  connect(actionManager->action(WbAction::DISABLE_SELECTION), &QAction::triggered, this, &WbView3D::setSelectionDisabled);
  connect(actionManager->action(WbAction::DISABLE_3D_VIEW_CONTEXT_MENU), &QAction::triggered, this,
          &WbView3D::setContextMenuDisabled);
  connect(actionManager->action(WbAction::DISABLE_OBJECT_MOVE), &QAction::triggered, this, &WbView3D::disableObjectMove);
  connect(actionManager->action(WbAction::DISABLE_FORCE_AND_TORQUE), &QAction::triggered, this,
          &WbView3D::disableApplyForceAndTorque);
  // optional renderings
  connect(actionManager->action(WbAction::COORDINATE_SYSTEM), &QAction::toggled, this, &WbView3D::setShowCoordinateSystem);
  connect(actionManager->action(WbAction::BOUNDING_OBJECT), &QAction::toggled, this, &WbView3D::setShowBoundingObjects);
  connect(actionManager->action(WbAction::NORMALS), &QAction::triggered, this, &WbView3D::setShowNormals);
  connect(actionManager->action(WbAction::CONTACT_POINTS), &QAction::toggled, this, &WbView3D::setShowContactPoints);
  connect(actionManager->action(WbAction::CONNECTOR_AXES), &QAction::toggled, this, &WbView3D::setShowConnectorAxes);
  connect(actionManager->action(WbAction::JOINT_AXES), &QAction::toggled, this, &WbView3D::setShowJointAxes);
  connect(actionManager->action(WbAction::RANGE_FINDER_FRUSTUMS), &QAction::toggled, this,
          &WbView3D::setShowRangeFinderFrustums);
  connect(actionManager->action(WbAction::LIDAR_RAYS_PATH), &QAction::toggled, this, &WbView3D::setShowLidarRaysPaths);
  connect(actionManager->action(WbAction::LIDAR_POINT_CLOUD), &QAction::toggled, this, &WbView3D::setShowLidarPointClouds);
  connect(actionManager->action(WbAction::CAMERA_FRUSTUM), &QAction::toggled, this, &WbView3D::setShowCameraFrustums);
  connect(actionManager->action(WbAction::DISTANCE_SENSOR_RAYS), &QAction::toggled, this, &WbView3D::setShowDistanceSensorRays);
  connect(actionManager->action(WbAction::LIGHT_SENSOR_RAYS), &QAction::toggled, this, &WbView3D::setShowLightSensorRays);
  connect(actionManager->action(WbAction::LIGHT_POSITIONS), &QAction::toggled, this, &WbView3D::setShowLightsPositions);
  connect(actionManager->action(WbAction::CENTER_OF_BUOYANCY), &QAction::triggered, this, &WbView3D::showCenterOfBuoyancy);
  connect(actionManager->action(WbAction::PEN_PAINTING_RAYS), &QAction::toggled, this, &WbView3D::setShowPenPaintingRays);
  connect(actionManager->action(WbAction::CENTER_OF_MASS), &QAction::triggered, this, &WbView3D::showCenterOfMass);
  connect(actionManager->action(WbAction::SUPPORT_POLYGON), &QAction::triggered, this, &WbView3D::showSupportPolygon);
  connect(actionManager->action(WbAction::SKIN_SKELETON), &QAction::triggered, this, &WbView3D::setShowSkeletonAction);
  connect(actionManager->action(WbAction::RADAR_FRUSTUMS), &QAction::toggled, this, &WbView3D::setShowRadarFrustums);
  connect(actionManager->action(WbAction::PHYSICS_CLUSTERS), &QAction::triggered, this,
          &WbView3D::setShowPhysicsClustersAction);
  connect(actionManager->action(WbAction::BOUNDING_SPHERE), &QAction::triggered, this, &WbView3D::setShowBoundingSphereAction);
  // virtual reality headset
  const WbPreferences *const prefs = WbPreferences::instance();
  connect(actionManager->action(WbAction::VIRTUAL_REALITY_HEADSET_ENABLE), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadset);
  setVirtualRealityHeadset(WbPreferences::instance()->value("VirtualRealityHeadset/enable").toBool());
  connect(actionManager->action(WbAction::VIRTUAL_REALITY_HEADSET_POSITION), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetPositionTracking);
  setVirtualRealityHeadsetPositionTracking(WbPreferences::instance()->value("VirtualRealityHeadset/trackPosition").toBool());
  connect(actionManager->action(WbAction::VIRTUAL_REALITY_HEADSET_ORIENTATION), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetOrientationTracking);
  setVirtualRealityHeadsetOrientationTracking(
    WbPreferences::instance()->value("VirtualRealityHeadset/trackOrientation").toBool());
  connect(actionManager->action(WbAction::VIRTUAL_REALITY_HEADSET_LEFT_EYE), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetLeftEyeView);
  setVirtualRealityHeadsetLeftEyeView(WbPreferences::instance()->value("VirtualRealityHeadset/visibleEye").toString() ==
                                      "left");
  connect(actionManager->action(WbAction::VIRTUAL_REALITY_HEADSET_RIGHT_EYE), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetRightEyeView);
  setVirtualRealityHeadsetRightEyeView(WbPreferences::instance()->value("VirtualRealityHeadset/visibleEye").toString() ==
                                       "right");
  connect(actionManager->action(WbAction::VIRTUAL_REALITY_HEADSET_NO_EYE), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetNoEyeView);
  setVirtualRealityHeadsetNoEyeView(WbPreferences::instance()->value("VirtualRealityHeadset/visibleEye").toString() == "none");
  connect(actionManager->action(WbAction::VIRTUAL_REALITY_HEADSET_ANTI_ALIASING), &QAction::triggered, this,
          &WbView3D::setVirtualRealityHeadsetAntiAliasing);
  setVirtualRealityHeadsetAntiAliasing(WbPreferences::instance()->value("VirtualRealityHeadset/antiAliasing").toBool());
  actionManager->action(WbAction::HIDE_ALL_CAMERA_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllCameraOverlays", false).toBool());
  connect(actionManager->action(WbAction::HIDE_ALL_CAMERA_OVERLAYS), &QAction::toggled, this,
          &WbView3D::setHideAllCameraOverlays);
  actionManager->action(WbAction::HIDE_ALL_RANGE_FINDER_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllRangeFinderOverlays", false).toBool());
  connect(actionManager->action(WbAction::HIDE_ALL_RANGE_FINDER_OVERLAYS), &QAction::toggled, this,
          &WbView3D::setHideAllRangeFinderOverlays);
  actionManager->action(WbAction::HIDE_ALL_DISPLAY_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllDisplayOverlays", false).toBool());
  connect(actionManager->action(WbAction::HIDE_ALL_DISPLAY_OVERLAYS), &QAction::toggled, this,
          &WbView3D::setHideAllDisplayOverlays);
  // enable/disable shadows when preferences change
  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbView3D::updateShadowState);

  // WASD free-fly camera: 60 Hz tick that advances the viewpoint while any fly key is held
  mFlyTimer = new QTimer(this);
  mFlyTimer->setInterval(16);
  connect(mFlyTimer, &QTimer::timeout, this, &WbView3D::updateFlyCamera);
}

void WbView3D::setPerspectiveProjection() {
  setProjectionMode(WR_CAMERA_PROJECTION_MODE_PERSPECTIVE, true, true);
}

void WbView3D::setOrthographicProjection() {
  setProjectionMode(WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC, true, true);
}

void WbView3D::setPlain() {
  setRenderingMode(WR_VIEWPORT_POLYGON_MODE_FILL, true);
}

void WbView3D::setWireframe() {
  setRenderingMode(WR_VIEWPORT_POLYGON_MODE_LINE, true);
}

void WbView3D::onSelectionChanged(WbAbstractPose *selectedPose) {
  assert(mWorld);

  if (mWorld->isCleaning())
    return;

  WbSolid *const selectedSolid = dynamic_cast<WbSolid *>(selectedPose);
  const WbViewpoint *const viewpoint = mWorld->viewpoint();

  if (selectedSolid) {
    setCheckedShowSupportPolygonAction(selectedSolid);
    setCheckedShowCenterOfMassAction(selectedSolid);
    setCheckedShowCenterOfBuoyancyAction(selectedSolid);
    setCheckedFollowObjectAction(selectedSolid);
    selectedSolid->updateTranslateRotateHandlesSize();
    WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setEnabled(true);
  } else {
    WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setEnabled(viewpoint->followType() != WbViewpoint::FOLLOW_NONE);
    WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT)->setChecked(false);
    WbActionManager::instance()->action(WbAction::SUPPORT_POLYGON)->setChecked(false);
    WbActionManager::instance()->action(WbAction::CENTER_OF_MASS)->setChecked(false);
    WbActionManager::instance()->action(WbAction::CENTER_OF_BUOYANCY)->setChecked(false);
  }

  bool enable = selectedSolid != NULL;
  WbActionManager::instance()->action(WbAction::CENTER_OF_BUOYANCY)->setEnabled(enable);
  WbActionManager::instance()->action(WbAction::CENTER_OF_MASS)->setEnabled(enable);
  WbActionManager::instance()->action(WbAction::SUPPORT_POLYGON)->setEnabled(enable);
  WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setEnabled(enable);
  WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED)->setEnabled(enable);
  WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT)->setEnabled(enable);
  enable = enable && selectedSolid == viewpoint->followedSolid();
  WbActionManager::instance()
    ->action(WbAction::FOLLOW_NONE)
    ->setChecked(enable && viewpoint->followType() == WbViewpoint::FOLLOW_NONE);
  WbActionManager::instance()
    ->action(WbAction::FOLLOW_TRACKING)
    ->setChecked(enable && viewpoint->followType() == WbViewpoint::FOLLOW_TRACKING);
  WbActionManager::instance()
    ->action(WbAction::FOLLOW_MOUNTED)
    ->setChecked(enable && viewpoint->followType() == WbViewpoint::FOLLOW_MOUNTED);
  WbActionManager::instance()
    ->action(WbAction::FOLLOW_PAN_AND_TILT)
    ->setChecked(enable && viewpoint->followType() == WbViewpoint::FOLLOW_PAN_AND_TILT);

  cleanupEvents();
}

WbView3D::~WbView3D() {
  cleanupFullScreenOverlay();
  cleanupPickers();
  cleanupOptionalRendering();
  WbWrenRenderingContext::cleanup();
  delete mMousePressTimer;
  // R4 3c-B: free the lazily-created wgpu main-view resources (owned by this view; null if never used).
  delete mWgpuRenderTarget;
  delete mWgpuMeshCache;
  delete mWgpuTextureCache;

  WbWrenLabelOverlay::cleanup();
#ifdef _WIN32
  WbVirtualRealityHeadset::cleanup();
#endif
}

void WbView3D::focusInEvent(QFocusEvent *event) {
  WbActionManager::instance()->enableTextEditActions(false, true);
  WbActionManager::instance()->setFocusObject(this);
  emit applicationActionsUpdateRequested();
}

void WbView3D::focusOutEvent(QFocusEvent *event) {
  if (WbActionManager::instance()->focusObject() == this)
    WbActionManager::instance()->setFocusObject(NULL);
  // Stop flying when the 3D view loses keyboard focus, otherwise a held key gets stuck and
  // the camera keeps drifting after the user clicks into the scene tree or another panel.
  stopFly();
}

// main refresh function (update from the simulation engine)
// for refresh coming from the GUI, use renderLater() instead
void WbView3D::refresh() {
  if (!mWorld || !WbSimulationState::instance()->isRendering()) {
    // render black screen
    renderLater();
    return;
  }

  const WbSimulationState *const sim = WbSimulationState::instance();
  mPhysicsRefresh = true;
  if (mScreenshotRequested)
    renderNow(true, true);
  else if (sim->isPaused())
    renderLater();
  else if (WbVideoRecorder::instance()->isRecording()) {
    const double time = WbSimulationState::instance()->time();
    static double lastRefreshTime = time;
    if (time - lastRefreshTime >= WbVideoRecorder::displayRefresh() || time < lastRefreshTime) {
      // render main window immediately even if it is not exposed
      lastRefreshTime = time;
      renderNow();
    }
  } else {
    const qint64 lastRefreshDelta = mLastRefreshTimer.elapsed();
    const double maxFrameDuration = 1000.0 / mWorld->worldInfo()->fps();  // ms
    if (lastRefreshDelta > maxFrameDuration)
      renderNow();
  }
  mPhysicsRefresh = false;
}

// Initializes or terminates solid's camera follow up according to the status of the WbActionManager actions
void WbView3D::followNone(bool checked) {
  if (!checked)
    return;

  mWorld->setModified();
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();
  viewpoint->setFollowType(WbViewpoint::FOLLOW_NONE);
}

void WbView3D::followTracking(bool checked) {
  if (!checked)
    return;

  mWorld->setModified();
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);
  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();
  viewpoint->setFollowType(WbViewpoint::FOLLOW_TRACKING);
  viewpoint->startFollowUp(selectedSolid, true);
}

void WbView3D::followMounted(bool checked) {
  if (!checked)
    return;

  mWorld->setModified();
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);
  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();
  viewpoint->setFollowType(WbViewpoint::FOLLOW_MOUNTED);
  viewpoint->startFollowUp(selectedSolid, true);
}

void WbView3D::followPanAndTilt(bool checked) {
  if (!checked)
    return;

  mWorld->setModified();
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);
  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();
  viewpoint->setFollowType(WbViewpoint::FOLLOW_PAN_AND_TILT);
  viewpoint->startFollowUp(selectedSolid, true);
}

void WbView3D::setCheckedFollowObjectAction(WbSolid *selectedSolid) {
  if (selectedSolid) {
    const WbViewpoint *const viewpoint = mWorld->viewpoint();
    if (viewpoint->followType() == WbViewpoint::FOLLOW_NONE)
      WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_TRACKING)
      WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_MOUNTED)
      WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_PAN_AND_TILT)
      WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT)->setChecked(true);
  }
}

// Notifies a change in the follow object action (checked/unchecked) from mViewpoint
void WbView3D::notifyFollowObjectAction(int type) {
  const WbViewpoint *const viewpoint = mWorld->viewpoint();
  if (viewpoint->followType() == WbViewpoint::FOLLOW_NONE)
    WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setChecked(true);
  else if (viewpoint->followType() == WbViewpoint::FOLLOW_TRACKING)
    WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setChecked(true);
  else if (viewpoint->followType() == WbViewpoint::FOLLOW_MOUNTED)
    WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED)->setChecked(true);
  else if (viewpoint->followType() == WbViewpoint::FOLLOW_PAN_AND_TILT)
    WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT)->setChecked(true);
}

// Shows the center of mass and the support polygon of a dynamic top WbSolid
void WbView3D::showSupportPolygon(bool checked) {
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (!selectedSolid->showSupportPolygonRepresentation(checked))
    WbActionManager::instance()->action(WbAction::SUPPORT_POLYGON)->setChecked(false);

  renderLater();
}

// Shows the center of mass of a dynamic WbSolid
void WbView3D::showCenterOfMass(bool checked) {
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (selectedSolid->showGlobalCenterOfMassRepresentation(checked) == false)
    WbActionManager::instance()->action(WbAction::CENTER_OF_MASS)->setChecked(false);

  renderLater();
}

void WbView3D::setCheckedShowCenterOfMassAction(WbSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->globalCenterOfMassRepresentationEnabled();
  WbActionManager::instance()->action(WbAction::CENTER_OF_MASS)->setChecked(enabled);
  if (enabled)
    renderLater();
}

// Shows the center of buoyancy of a dynamic WbSolid
void WbView3D::showCenterOfBuoyancy(bool checked) {
  WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (selectedSolid->showCenterOfBuoyancyRepresentation(checked) == false)
    WbActionManager::instance()->action(WbAction::CENTER_OF_BUOYANCY)->setChecked(false);

  renderLater();
}

void WbView3D::setCheckedShowCenterOfBuoyancyAction(WbSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->centerOfBuoyancyRepresentationEnabled();
  WbActionManager::instance()->action(WbAction::CENTER_OF_BUOYANCY)->setChecked(enabled);
  if (enabled)
    renderLater();
}

void WbView3D::setCheckedShowSupportPolygonAction(WbSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->supportPolygonRepresentationEnabled();
  WbActionManager::instance()
    ->action(WbAction::SUPPORT_POLYGON)
    ->setChecked(selectedSolid->supportPolygonRepresentationEnabled());
  if (enabled)
    renderLater();
}

void WbView3D::restoreViewpoint() {
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  viewpoint->restore();
  renderLater();
}

WrViewportPolygonMode WbView3D::stringToRenderingMode(const QString &s) {
  if (s == "WIREFRAME")
    return WR_VIEWPORT_POLYGON_MODE_LINE;
  return WR_VIEWPORT_POLYGON_MODE_FILL;  // default value
}

WrCameraProjectionMode WbView3D::stringToProjectionMode(const QString &s) {
  if (s == "ORTHOGRAPHIC")
    return WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC;

  return WR_CAMERA_PROJECTION_MODE_PERSPECTIVE;
}

void WbView3D::setRenderingMode(WrViewportPolygonMode mode, bool updatePerspective) {
  switch (mode) {
    case WR_VIEWPORT_POLYGON_MODE_FILL:
      if (updatePerspective && mWorld)
        mWorld->perspective()->setRenderingMode("PLAIN");
      WbActionManager::instance()->action(WbAction::PLAIN_RENDERING)->setChecked(true);
      break;
    case WR_VIEWPORT_POLYGON_MODE_LINE:
      if (updatePerspective && mWorld)
        mWorld->perspective()->setRenderingMode("WIREFRAME");
      WbActionManager::instance()->action(WbAction::WIREFRAME_RENDERING)->setChecked(true);
      break;
    default:
      assert(false);
  }

  mRenderingMode = mode;

  if (wr_gl_state_is_initialized()) {
    if (mRenderingMode == WR_VIEWPORT_POLYGON_MODE_FILL)
      WbWrenRenderingContext::instance()->setRenderingMode(WbWrenRenderingContext::RM_PLAIN, true);
    else
      WbWrenRenderingContext::instance()->setRenderingMode(WbWrenRenderingContext::RM_WIREFRAME, true);
  }

  renderLater();
}

void WbView3D::setVirtualRealityHeadset(bool enable) {
  if (mWorld) {
    bool sucess = mWorld->viewpoint()->enableVirtualRealityHeadset(enable);
    if (sucess)
      renderLater();
    else
      enable = !enable;
  }

  WbPreferences::instance()->setValue("VirtualRealityHeadset/enable", enable);
  WbActionManager::instance()->action(WbAction::VIRTUAL_REALITY_HEADSET_ENABLE)->setChecked(enable);

  if (enable) {
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_CAMERA, false);
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, false);
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, false);
  } else {
    if (!WbPreferences::instance()->value("View3d/hideAllCameraOverlays").toBool())
      WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_CAMERA, true);
    if (!WbPreferences::instance()->value("View3d/hideAllRangeFinderOverlays").toBool())
      WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, true);
    if (!WbPreferences::instance()->value("View3d/hideAllDisplayOverlays").toBool())
      WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, true);
  }

  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setVirtualRealityHeadsetPositionTracking(bool enable) {
  WbPreferences::instance()->setValue("VirtualRealityHeadset/trackPosition", enable);
  WbActionManager::instance()->action(WbAction::VIRTUAL_REALITY_HEADSET_POSITION)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse()) {
    WbVirtualRealityHeadset::instance()->enablePositionTracking(enable);
    renderLater();
  }
#endif
}

void WbView3D::setVirtualRealityHeadsetOrientationTracking(bool enable) {
  WbPreferences::instance()->setValue("VirtualRealityHeadset/trackOrientation", enable);
  WbActionManager::instance()->action(WbAction::VIRTUAL_REALITY_HEADSET_ORIENTATION)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse()) {
    WbVirtualRealityHeadset::instance()->enableOrientationTracking(enable);
    renderLater();
  }
#endif
}

void WbView3D::setVirtualRealityHeadsetLeftEyeView(bool enable) {
  if (enable)
    WbPreferences::instance()->setValue("VirtualRealityHeadset/visibleEye", "left");
  WbActionManager::instance()->action(WbAction::VIRTUAL_REALITY_HEADSET_LEFT_EYE)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && enable) {
    WbVirtualRealityHeadset::instance()->setEyeView(WbVirtualRealityHeadset::LEFT);
    renderLater();
  }
#endif
  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setVirtualRealityHeadsetRightEyeView(bool enable) {
  if (enable)
    WbPreferences::instance()->setValue("VirtualRealityHeadset/visibleEye", "right");
  WbActionManager::instance()->action(WbAction::VIRTUAL_REALITY_HEADSET_RIGHT_EYE)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && enable) {
    WbVirtualRealityHeadset::instance()->setEyeView(WbVirtualRealityHeadset::RIGHT);
    renderLater();
  }
#endif
  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setVirtualRealityHeadsetNoEyeView(bool enable) {
  if (enable)
    WbPreferences::instance()->setValue("VirtualRealityHeadset/visibleEye", "none");
  WbActionManager::instance()->action(WbAction::VIRTUAL_REALITY_HEADSET_NO_EYE)->setChecked(enable);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && enable) {
    WbVirtualRealityHeadset::instance()->setEyeView(WbVirtualRealityHeadset::NONE);
    renderLater();
  }
#endif
  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setVirtualRealityHeadsetAntiAliasing(bool enable) {
  WbPreferences::instance()->setValue("VirtualRealityHeadset/antiAliasing", enable);
  WbActionManager::instance()->action(WbAction::VIRTUAL_REALITY_HEADSET_ANTI_ALIASING)->setChecked(enable);
  if (mWorld) {
    mWorld->viewpoint()->setVirtualRealityHeadsetAntiAliasing(enable);
    renderLater();
  }
  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::setProjectionMode(WrCameraProjectionMode mode, bool updatePerspective, bool updateAction) {
  mProjectionMode = mode;
  if (mWorld)
    mWorld->viewpoint()->setProjectionMode(mode);

  switch (mode) {
    case WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC:
      if (updateAction)
        WbActionManager::instance()->action(WbAction::ORTHOGRAPHIC_PROJECTION)->setChecked(true);
      if (mWorld) {
        mWorld->viewpoint()->updateOrthographicViewHeight();
        wr_config_enable_shadows(false);  // No shadows in orthographic mode
        if (updatePerspective)
          mWorld->perspective()->setProjectionMode("ORTHOGRAPHIC");
      }
      break;
    default:
      updateShadowState();
      if (updatePerspective && mWorld)
        mWorld->perspective()->setProjectionMode("PERSPECTIVE");
      if (updateAction)
        WbActionManager::instance()->action(WbAction::PERSPECTIVE_PROJECTION)->setChecked(true);
      break;
  }

  if (wr_gl_state_is_initialized())
    wr_camera_set_projection_mode(wr_viewport_get_camera(wr_scene_get_viewport(wr_scene_get_instance())), mProjectionMode);

  renderLater();
}

void WbView3D::setShowCoordinateSystem(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("CoordinateSystem", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_COORDINATE_SYSTEM, show);
  // Like other optional rendering features, enabling the coordinate system
  // triggers a redraw on the screen. However until the user interacts with
  // webots the coordinate system will not be rendered onto the scene.
  // We force the coordinate system to be rendered here so that it appears
  // immediately, without needing user interaction.
  renderNow();
}

void WbView3D::setShowBoundingObjects(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("AllBoundingObjects", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS, show);
}

void WbView3D::setShowContactPoints(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("ContactPoints", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CONTACT_POINTS, show);
}

void WbView3D::setShowConnectorAxes(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("ConnectorAxes", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CONNECTOR_AXES, show);
}

void WbView3D::setShowJointAxes(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("JointAxes", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_JOINT_AXES, show);
}

void WbView3D::setShowCameraFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("CameraFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CAMERA_FRUSTUMS, show);
}

void WbView3D::setShowRangeFinderFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("RangeFinderFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS, show);
}

void WbView3D::setShowRadarFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("RadarFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_RADAR_FRUSTUMS, show);
}

void WbView3D::setShowLidarRaysPaths(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LidarRaysPaths", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIDAR_RAYS_PATHS, show);
}

void WbView3D::setShowLidarPointClouds(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LidarPointClouds", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIDAR_POINT_CLOUD, show);
}

void WbView3D::setShowRenderingDevice(bool checked) {
  WbRenderingDevice *device = static_cast<WbRenderingDevice *>(sender()->property("renderingDevice").value<void *>());
  device->toggleOverlayVisibility(checked);
  renderLater();
}

void WbView3D::setHideAllCameraOverlays(bool hidden) {
  WbPreferences::instance()->setValue("View3d/hideAllCameraOverlays", hidden);

#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && hidden)
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_CAMERA, false);
  else if (!WbVirtualRealityHeadset::isInUse())
#endif
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_CAMERA, !hidden);

  renderLater();
}

void WbView3D::setHideAllRangeFinderOverlays(bool hidden) {
  WbPreferences::instance()->setValue("View3d/hideAllRangeFinderOverlays", hidden);

#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && hidden)
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, false);
  else if (!WbVirtualRealityHeadset::isInUse())
#endif
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, !hidden);

  renderLater();
}

void WbView3D::setHideAllDisplayOverlays(bool hidden) {
  WbPreferences::instance()->setValue("View3d/hideAllDisplayOverlays", hidden);

#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse() && hidden)
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, false);
  else if (!WbVirtualRealityHeadset::isInUse())
#endif
    WbWrenTextureOverlay::setElementsVisible(WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, !hidden);

  renderLater();
}

void WbView3D::setShowDistanceSensorRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("DistanceSensorRays", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_DISTANCE_SENSORS_RAYS, show);
}

void WbView3D::setShowLightSensorRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LightSensorRays", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIGHT_SENSORS_RAYS, show);
}

void WbView3D::setShowLightsPositions(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LightPositions", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIGHTS_POSITIONS, show);
}

void WbView3D::setShowPenPaintingRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("PenPaintingRays", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_PEN_RAYS, show);
}

void WbView3D::setShowSkeletonAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("Skeleton", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_SKIN_SKELETON, show);
}

void WbView3D::setShowNormals(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("Normals", show);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_NORMALS, show);
}

void WbView3D::setShowPhysicsClustersAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("PhysicsClusters", show);
  WbOdeDebugger::instance()->toggleDebugging(show);
}

void WbView3D::setShowBoundingSphereAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("BoundingSphere", show);
  WbVisualBoundingSphere::enable(show, WbSelection::instance()->selectedNode());
  renderLater();
}

void WbView3D::setUserInteractionDisabled(WbAction::WbActionKind action, bool disabled) {
  mDisabledUserInteractionsMap[action] = disabled;
  if (mWorld)
    mWorld->perspective()->setUserInteractionDisabled(action, disabled);
}

void WbView3D::disableObjectMove(bool disabled) {
  setUserInteractionDisabled(WbAction::DISABLE_OBJECT_MOVE, disabled);
  if (disabled)
    WbSelection::instance()->disableActiveManipulator();
  else
    WbSelection::instance()->restoreActiveManipulator();
  renderLater();
}

void WbView3D::updateMousesPosition(bool fromMouseClick, bool fromMouseMove) {
  const QList<WbMouse *> mouses = WbMouse::mouses();
  if (mouses.size() == 0)
    return;

  QList<WbMouse *> mousesRequiringRefresh;
  bool shouldUsePicker = false;
  for (int i = 0; i < WbMouse::mouses().size(); ++i) {
    WbMouse *mouse = WbMouse::mouses().at(i);
    if (mouse->needToRefresh()) {
      mousesRequiringRefresh.append(mouse);
      if (!shouldUsePicker && mouse->is3dPositionEnabled())
        shouldUsePicker = true;
    }
    if (!mouse->isTracked()) {
      // In the non-tracked case, update the buttons in any cases to avoid loosing a press event
      // in case the press and release events happen in the same step
      mouse->setLeft(mouse->left() | (mLastButtonState & Qt::LeftButton));
      mouse->setMiddle(mouse->middle() | (mLastButtonState & Qt::MiddleButton));
      mouse->setRight(mouse->right() | (mLastButtonState & Qt::RightButton));
    }
  }

  if (mousesRequiringRefresh.size() == 0)
    return;

  const QPoint position = mapFromGlobal(QCursor::pos());
  if (position.x() < 0 || position.y() < 0 || position.x() >= width() || position.y() >= height())
    return;

  if (!mControllerPicker)
    mControllerPicker = new WbWrenPicker();

  const bool picked = shouldUsePicker ? mControllerPicker->pick(position.x(), position.y()) : false;

  foreach (WbMouse *mouse, mousesRequiringRefresh) {
    if (picked && mouse->is3dPositionEnabled()) {
      WbVector3 screenPosition = mControllerPicker->screenCoordinates();
      screenPosition[0] = (screenPosition[0] / width()) * 2 - 1;
      screenPosition[1] = (screenPosition[1] / height()) * 2 - 1;
      WbVector3 worldPosition;
      mWorld->viewpoint()->toWorld(screenPosition, worldPosition);
      mouse->setPosition(worldPosition.x(), worldPosition.y(), worldPosition.z());
    }

    mouse->setScreenPosition((double)position.x() / width(), (double)position.y() / height());

    if (mouse->isTracked()) {
      mouse->setLeft(mLastButtonState & Qt::LeftButton);
      mouse->setMiddle(mLastButtonState & Qt::MiddleButton);
      mouse->setRight(mLastButtonState & Qt::RightButton);
    }
    mouse->setHasMoved(fromMouseMove);
    mouse->setHasClicked(fromMouseClick);
    mouse->refreshSensorIfNeeded();
    emit mouse->changed();
  }
}

void WbView3D::logWrenStatistics() {
  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (!log)
    return;
  if (mRenderedFrameCount <= 0 || !mFpsAccumulationTimer.isValid())
    return;
  const double elapsedSeconds = mFpsAccumulationTimer.elapsed() / 1000.0;
  if (elapsedSeconds <= 0.0)
    return;
  log->setAvgFPS(static_cast<double>(mRenderedFrameCount) / elapsedSeconds);
  mRenderedFrameCount = 0;
  mFpsAccumulationTimer.invalidate();
}

void WbView3D::prepareWorldLoading() {
  WbWrenOpenGlContext::makeWrenCurrent();

  // reset text labels
  WbWrenLabelOverlay::removeAllLabels();

  if (!wr_gl_state_is_initialized())  // may occur at least on Windows when launched with the minimized option
    initialize();

  if (!mLoadingWorldOverlay) {
    mLoadingWorldOverlay = new WbWrenFullScreenOverlay(tr("Loading world"), 80, true);
    mLoadingWorldOverlay->attachToViewport(wr_scene_get_viewport(wr_scene_get_instance()));
#ifdef _WIN32
    WbVirtualRealityHeadset::setLoadingTexture(mLoadingWorldOverlay->overlayTexture());
#endif
  }
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse())
    WbVirtualRealityHeadset::instance()->setTextureOverlayVisible(true);
#endif
  hideBlackRenderingOverlay();
  mLoadingWorldOverlay->setVisible(true);
  WbWrenWindow::renderNow();

  // Resets the background if no Background node exists
  const float clearColor[] = {1.0f, 1.0f, 1.0f};
  wr_viewport_set_clear_color_rgb(wr_scene_get_viewport(wr_scene_get_instance()), clearColor);

  // Cleanup the drags events that were possibly used in the previous world
  cleanupEvents();

  // signals that update the menu's ticks according to the status of the selection
  disconnect(WbSelection::instance(), &WbSelection::selectionChangedFromView3D, this, &WbView3D::onSelectionChanged);
  disconnect(WbSelection::instance(), &WbSelection::selectionChangedFromSceneTree, this, &WbView3D::onSelectionChanged);

  cleanupOptionalRendering();

  // wgpu main-view: the mesh/texture caches key on this world's WREN resources, which are freed when
  // the old world is torn down. Carrying them into the reloaded world makes collectWorldDraws read
  // stale/reused pointers → a crash on reload (Ctrl+Shift+R). Drop the owned GPU resources and re-arm
  // the lazy init (mWgpuBackend is registry-owned, not deleted — nulling it re-fetches + rebuilds the
  // caches for the new world). No-op when the wgpu main view was never used.
  delete mWgpuRenderTarget;
  mWgpuRenderTarget = nullptr;
  delete mWgpuMeshCache;
  mWgpuMeshCache = nullptr;
  delete mWgpuTextureCache;
  mWgpuTextureCache = nullptr;
  mWgpuBackend = nullptr;
  mWgpuMainViewUnavailable = false;
  invalidateWgpuDrawList();  // draws alias the mesh cache + scene nodes of the dying world

  WbWrenOpenGlContext::doneWren();
}

// Drop the cached main-view draw list (and its destroyed() hooks) and mark it for rebuild. Called
// when a referenced node is destroyed, on world (re)load, and periodically for appearance staleness.
void WbView3D::invalidateWgpuDrawList() {
  for (const QMetaObject::Connection &c : mWgpuDrawListConns)
    QObject::disconnect(c);
  mWgpuDrawListConns.clear();
  mWgpuDrawList.clear();
  mWgpuModelList.clear();
  mWgpuRefreshList.clear();
  mWgpuDrawListDirty = true;
  mWgpuDrawListAge = 0;
}

void WbView3D::updateViewport() {
  // Sets the solid follow up according to viewpoint's follow field
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  connect(viewpoint, &WbViewpoint::followTypeChanged, this, &WbView3D::notifyFollowObjectAction);
  connect(viewpoint, SIGNAL(virtualRealityHeadsetRequiresRender()), this, SLOT(renderNow()));
  if (viewpoint->followedSolid()) {
    if (viewpoint->followType() == WbViewpoint::FOLLOW_NONE)
      WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_TRACKING)
      WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_MOUNTED)
      WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_PAN_AND_TILT)
      WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT)->setChecked(true);
  } else {
    WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT)->setChecked(false);
  }

  cleanupPickers();
  mPicker = new WbWrenPicker();

  viewpoint->updateAspectRatio(mAspectRatio);

  // Re-initialize matter handles size
  WbWrenAbstractManipulator::setViewport(viewpoint->viewportWren());
  WbSelection::instance()->updateHandlesScale();

#ifdef _WIN32
  updateVirtualRealityHeadsetOverlay();
#endif

  // update handles size when viewpoint changes
  connect(viewpoint, &WbViewpoint::cameraParametersChanged, WbSelection::instance(), &WbSelection::updateHandlesScale);
}

void WbView3D::updateShadowState() {
  if (WbPreferences::instance()->value("OpenGL/disableShadows").toBool() == wr_config_are_shadows_enabled() &&
      mWorld->viewpoint()->projectionMode() != WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC) {
    wr_config_enable_shadows(!WbPreferences::instance()->value("OpenGL/disableShadows").toBool());
    renderLater();
  }
}

void WbView3D::setWorld(WbSimulationWorld *w) {
  WbWrenOpenGlContext::makeWrenCurrent();

  mLoadingWorldOverlay->setVisible(false);
#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse())
    WbVirtualRealityHeadset::instance()->setTextureOverlayVisible(false);
#endif
  mWorld = w;  // world is loaded!

  // apply optional rendering
  if (WbPreferences::instance()->value("View3d/hideAllCameraOverlays").toBool())
    setHideAllCameraOverlays(true);
  if (WbPreferences::instance()->value("View3d/hideAllRangeFinderOverlays").toBool())
    setHideAllRangeFinderOverlays(true);
  if (WbPreferences::instance()->value("View3d/hideAllDisplayOverlays").toBool())
    setHideAllDisplayOverlays(true);

  const WbPerspective *perspective = mWorld->perspective();
  setProjectionMode(stringToProjectionMode(perspective->projectionMode()), false, true);
  setRenderingMode(stringToRenderingMode(perspective->renderingMode()), false);
  mDisabledUserInteractionsMap = perspective->disabledUserInteractionsMap();

  enableOptionalRenderingFromPerspective();

  connect(mWorld, &WbSimulationWorld::destroyed, this, &WbView3D::cleanWorld);
  connect(mWorld, &WbWorld::viewpointChanged, this, &WbView3D::updateViewport);

  // Sets the solid follow up according to viewpoint's follow field
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  connect(viewpoint, &WbViewpoint::followTypeChanged, this, &WbView3D::notifyFollowObjectAction);
  connect(viewpoint, SIGNAL(virtualRealityHeadsetRequiresRender()), this, SLOT(renderNow()));
  viewpoint->startFollowUpFromField();
  if (viewpoint->followedSolid()) {
    if (viewpoint->followType() == WbViewpoint::FOLLOW_NONE)
      WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_TRACKING)
      WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_MOUNTED)
      WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED)->setChecked(true);
    else if (viewpoint->followType() == WbViewpoint::FOLLOW_PAN_AND_TILT)
      WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT)->setChecked(true);
  } else {
    WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED)->setChecked(false);
    WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT)->setChecked(false);
  }

  // Prepares the contact point rendering (Note: WbControlledSimulation::instance() is valid after the call to
  // mMainWindow->loadWorld(mWorldName) in WbGuiApplication.cpp)
  // The constructor connects an update slot to the signal WbSimulationWorld::physicsStepEnded()
  mContactPointsRepresentation = new WbContactPointsRepresentation(mWrenRenderingContext);

  // Connects GUI-defined mode and rendering options to update methods for material of bounding objects
  const WbSimulationState *const simulationState = WbSimulationState::instance();
  connect(mWrenRenderingContext, &WbWrenRenderingContext::optionalRenderingChanged, mWorld,
          &WbSimulationWorld::checkNeedForBoundingMaterialUpdate, Qt::UniqueConnection);
  connect(simulationState, &WbSimulationState::renderingStateChanged, mWorld,
          &WbSimulationWorld::checkNeedForBoundingMaterialUpdate, Qt::UniqueConnection);
  mWorld->checkNeedForBoundingMaterialUpdate();

  // Prepares the shape picker
  delete mPicker;
  delete mControllerPicker;
  mControllerPicker = NULL;
  mPicker = new WbWrenPicker();

  // Creates the fast mode overlay
  if (!mDisabledRenderingOverlay) {
    mDisabledRenderingOverlay = new WbWrenFullScreenOverlay("No Rendering", 128, true);
    mDisabledRenderingOverlay->attachToViewport(wr_scene_get_viewport(wr_scene_get_instance()));
  }
  if (WbSimulationState::instance()->isRendering())
    hideBlackRenderingOverlay();
  else
    showBlackRenderingOverlay();

#ifdef _WIN32
  // Creates the virtual reality headset overlay
  if (!mVirtualRealityHeadsetOverlay) {
    mVirtualRealityHeadsetOverlay = new WbWrenFullScreenOverlay("Headset preview disabled", 64, false);
    mVirtualRealityHeadsetOverlay->attachToViewport(wr_scene_get_viewport(wr_scene_get_instance()));
  }
  updateVirtualRealityHeadsetOverlay();
#endif

  // connect supervisor scene tree modifications to graphical updates
  const QList<WbRobot *> &robots = mWorld->robots();
  foreach (const WbRobot *const robot, robots) {
    if (robot->supervisor())
      connect(robot->supervisorUtilities(), &WbSupervisorUtilities::worldModified, this,
              &WbView3D::handleWorldModificationFromSupervisor);
  }

  // initialize matter handles size
  WbWrenAbstractManipulator::setViewport(wr_scene_get_viewport(wr_scene_get_instance()));
  WbSelection::instance()->updateHandlesScale();
  // update handles size when viewpoint changes
  connect(viewpoint, &WbViewpoint::cameraParametersChanged, WbSelection::instance(), &WbSelection::updateHandlesScale);
  connect(viewpoint, &WbViewpoint::refreshRequired, this, &WbView3D::renderLater);

  // signals that update the menu's ticks according to the status of the selection
  connect(WbSelection::instance(), &WbSelection::selectionChangedFromView3D, this, &WbView3D::onSelectionChanged);
  connect(WbSelection::instance(), &WbSelection::selectionChangedFromSceneTree, this, &WbView3D::onSelectionChanged);

  mAspectRatio = ((double)width()) / height();
  viewpoint->updateAspectRatio(mAspectRatio);
  updateWrenViewportDimensions();
  onSelectionChanged(WbSelection::instance()->selectedAbstractPose());

  WbWrenOpenGlContext::doneWren();

  // first rendering is offscreen without culling to make sure every meshes/textures are actually
  // loaded on the GPU
  renderNow(false, true);
}

void WbView3D::restoreOptionalRendering(const QStringList &enabledCenterOfMassNodeNames,
                                        const QStringList &enabledCenterOfBuoyancyNodeNames,
                                        const QStringList &enabledSupportPolygonNodeNames) const {
  // restore node specific optional rendering from world properties
  WbSolid *solid = NULL;
  for (int i = 0; i < enabledCenterOfMassNodeNames.size(); ++i) {
    solid = WbSolid::findSolidFromUniqueName(enabledCenterOfMassNodeNames[i]);
    if (solid)
      solid->showGlobalCenterOfMassRepresentation(true);
  }

  for (int i = 0; i < enabledCenterOfBuoyancyNodeNames.size(); ++i) {
    solid = WbSolid::findSolidFromUniqueName(enabledCenterOfBuoyancyNodeNames[i]);
    if (solid)
      solid->showCenterOfBuoyancyRepresentation(true);
  }

  for (int i = 0; i < enabledSupportPolygonNodeNames.size(); ++i) {
    solid = WbSolid::findSolidFromUniqueName(enabledSupportPolygonNodeNames[i]);
    if (solid)
      solid->showSupportPolygonRepresentation(true);
  }
}

void WbView3D::enableOptionalRenderingFromPerspective() {
  // Enables optional rendering from preferences
  assert(mWorld);
  const WbPerspective *perspective = mWorld->perspective();
  WbActionManager *actionManager = WbActionManager::instance();
  actionManager->action(WbAction::COORDINATE_SYSTEM)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("CoordinateSystem"));
  actionManager->action(WbAction::BOUNDING_OBJECT)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("AllBoundingObjects"));
  actionManager->action(WbAction::NORMALS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("Normals"));
  actionManager->action(WbAction::CONTACT_POINTS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("ContactPoints"));
  actionManager->action(WbAction::CONNECTOR_AXES)->setChecked(perspective->isGlobalOptionalRenderingEnabled("ConnectorAxes"));
  actionManager->action(WbAction::JOINT_AXES)->setChecked(perspective->isGlobalOptionalRenderingEnabled("JointAxes"));
  actionManager->action(WbAction::RANGE_FINDER_FRUSTUMS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("RangeFinderFrustums"));
  actionManager->action(WbAction::LIDAR_RAYS_PATH)->setChecked(perspective->isGlobalOptionalRenderingEnabled("LidarRaysPaths"));
  actionManager->action(WbAction::LIDAR_POINT_CLOUD)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("LidarPointClouds"));
  actionManager->action(WbAction::CAMERA_FRUSTUM)->setChecked(perspective->isGlobalOptionalRenderingEnabled("CameraFrustums"));
  actionManager->action(WbAction::DISTANCE_SENSOR_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("DistanceSensorRays"));
  actionManager->action(WbAction::LIGHT_SENSOR_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("LightSensorRays"));
  actionManager->action(WbAction::LIGHT_POSITIONS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("LightPositions"));
  actionManager->action(WbAction::CENTER_OF_BUOYANCY)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("CenterOfBuoyancy"));
  actionManager->action(WbAction::PEN_PAINTING_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("PenPaintingRays"));
  actionManager->action(WbAction::CENTER_OF_MASS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("CenterOfMass"));
  actionManager->action(WbAction::SUPPORT_POLYGON)->setChecked(perspective->isGlobalOptionalRenderingEnabled("SupportPolygon"));
  actionManager->action(WbAction::SKIN_SKELETON)->setChecked(perspective->isGlobalOptionalRenderingEnabled("Skeleton"));
  actionManager->action(WbAction::RADAR_FRUSTUMS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("RadarFrustums"));
  actionManager->action(WbAction::PHYSICS_CLUSTERS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("PhysicsClusters"));
  actionManager->action(WbAction::BOUNDING_SPHERE)->setChecked(perspective->isGlobalOptionalRenderingEnabled("BoundingSphere"));
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_COORDINATE_SYSTEM,
                                                 perspective->isGlobalOptionalRenderingEnabled("CoordinateSystem"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS,
                                                 perspective->isGlobalOptionalRenderingEnabled("AllBoundingObjects"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_NORMALS,
                                                 perspective->isGlobalOptionalRenderingEnabled("Normals"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CONTACT_POINTS,
                                                 perspective->isGlobalOptionalRenderingEnabled("ContactPoints"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CONNECTOR_AXES,
                                                 perspective->isGlobalOptionalRenderingEnabled("ConnectorAxes"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_JOINT_AXES,
                                                 perspective->isGlobalOptionalRenderingEnabled("JointAxes"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("RangeFinderFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIDAR_RAYS_PATHS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LidarRaysPaths"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIDAR_POINT_CLOUD,
                                                 perspective->isGlobalOptionalRenderingEnabled("LidarPointClouds"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_CAMERA_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("CameraFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_RADAR_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("RadarFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_DISTANCE_SENSORS_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("DistanceSensorRays"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIGHT_SENSORS_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LightSensorRays"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_LIGHTS_POSITIONS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LightPositions"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_PEN_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("PenPaintingRays"), false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VF_SKIN_SKELETON,
                                                 perspective->isGlobalOptionalRenderingEnabled("Skeleton"), false);
  WbOdeDebugger::instance()->toggleDebugging(perspective->isGlobalOptionalRenderingEnabled("PhysicsClusters"));
}

void WbView3D::disableOptionalRenderingAndOverLays() {
  // Save optional renderings before saving thumbnail
  mOptionalRenderingsMask = mWrenRenderingContext->optionalRenderingsMask();

  // Temporary hide optional renderings (without notifying the nodes and removing them from the scene)
  // unset optional renderings flags in mask and set VM_REGULAR (no special rendering) bits only
  mWrenRenderingContext->blockSignals(true);
  mWrenRenderingContext->enableOptionalRendering(~WbWrenRenderingContext::VM_REGULAR, false, false);
  mWrenRenderingContext->enableOptionalRendering(WbWrenRenderingContext::VM_REGULAR, true, false);
  mWrenRenderingContext->blockSignals(false);
  mWorld->viewpoint()->updateOptionalRendering(WbWrenRenderingContext::VM_REGULAR);

  // Hide overlays for thumbnail
  setHideAllCameraOverlays(true);
  setHideAllRangeFinderOverlays(true);
  setHideAllDisplayOverlays(true);

  // Switch to perspective projection if necessary
  if (mWorld->viewpoint()->projectionMode() == WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC)
    setProjectionMode(WR_CAMERA_PROJECTION_MODE_PERSPECTIVE, true, false);
}

void WbView3D::restoreOptionalRenderingAndOverLays() {
  // Restore optional renderings (without notifying all the nodes)
  mWrenRenderingContext->blockSignals(true);
  mWrenRenderingContext->enableOptionalRendering(mOptionalRenderingsMask, true, false);
  mWrenRenderingContext->blockSignals(false);
  mWorld->viewpoint()->updateOptionalRendering(WbWrenRenderingContext::VM_REGULAR);

  // Restore overlays after saving thumbnail
  WbActionManager *actionManager = WbActionManager::instance();
  setHideAllCameraOverlays(actionManager->action(WbAction::HIDE_ALL_CAMERA_OVERLAYS)->isChecked());
  setHideAllRangeFinderOverlays(actionManager->action(WbAction::HIDE_ALL_RANGE_FINDER_OVERLAYS)->isChecked());
  setHideAllDisplayOverlays(actionManager->action(WbAction::HIDE_ALL_DISPLAY_OVERLAYS)->isChecked());

  // Switch back to orthographic projection if necessary
  if (WbActionManager::instance()->action(WbAction::ORTHOGRAPHIC_PROJECTION)->isChecked())
    setProjectionMode(WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC, true, false);
}

void WbView3D::checkRendererCapabilities() {
  QString message;  // The displayed message to forge

  // 1. parameters which can be reduced
  bool disableShadows = false;
  bool disableAntiAliasing = false;
  bool disableGTAO = false;
  int reduceTextureQuality = 0;

  // 2. determine what has to be reduced
  if (!mWrenRenderingContext->isNvidiaRenderer() && !mWrenRenderingContext->isAmdRenderer() &&
      !mWrenRenderingContext->isIntelRenderer()) {
    message += tr("OmniSim has detected that your GPU vendor is '%1'. "
                  "A recent NVIDIA or AMD graphics adapter is highly recommended to run OmniSim smoothly. ")
                 .arg(wr_gl_state_get_vendor());

    if (mWrenRenderingContext->isMesaRenderer() || mWrenRenderingContext->isMicrosoftRenderer()) {
      message += tr("OmniSim has detected that your computer uses a slow 3D software rendering system. "
                    "It is strongly recommended to install the latest graphics drivers provided by your GPU manufacturer. "
                    "OmniSim will run much faster after the installation of the correct driver.");
    }

    message += '\n';

    disableShadows = true;
    disableAntiAliasing = true;
    disableGTAO = true;
    reduceTextureQuality = 1;
  }

#ifdef _WIN32
  if (mWrenRenderingContext->isIntelRenderer()) {
    int gpuGeneration = WbSysInfo::intelGPUGeneration(WbWrenOpenGlContext::instance()->functions());
    if (gpuGeneration < 5) {
      message += tr("OmniSim has detected that your system features an old unsupported Intel GPU. "
                    "A recent NVIDIA or AMD graphics adapter is highly recommended to run OmniSim smoothly. ");
      message += '\n';
      disableShadows = true;
      disableAntiAliasing = true;
    }
  } else if (WbSysInfo::isAmdLowEndGpu(WbWrenOpenGlContext::instance()->functions())) {
    message += tr("OmniSim has detected that you are using an old unsupported AMD GPU. "
                  "A recent NVIDIA or AMD graphics adapter is highly recommended to run OmniSim smoothly. ");
    disableAntiAliasing = true;
    disableGTAO = true;
    reduceTextureQuality = 1;
  }
#else
  if (WbSysInfo::isLowEndGpu()) {
    message += tr("OmniSim has detected that your system features an old unsupported GPU. "
                  "A recent NVIDIA or AMD graphics adapter is highly recommended to run OmniSim smoothly. ");
    message += '\n';
    disableAntiAliasing = true;
    disableGTAO = true;
    disableShadows = true;
    reduceTextureQuality = 1;
  }
#endif

  int maxTextureFiltering = 1;
  int maxHardwareAfLevel = wr_gl_state_max_texture_anisotropy();
  // Find integer log2 of maxHardwareAfLevel to transcribe to user filtering level
  while (maxHardwareAfLevel >>= 1)
    ++maxTextureFiltering;

  // check GPU memory on NVIDIA GPU
  // (not for Intel GPU, because the texture size has no impact on the rendring speed)
  // (not for AMD GPU, because the GPU memory cannot be retrieved accurately)
  if (mWrenRenderingContext->isNvidiaRenderer()) {
    if (wr_gl_state_get_gpu_memory() == 2097152)
      WbPreferences::instance()->setValue("OpenGL/limitBakingResolution", true);
    else if (wr_gl_state_get_gpu_memory() < 2097152) {  // Less than 2 GB of GPU memory
      if (message.isEmpty()) {
        message += tr("OmniSim has detected that your GPU has less than 2 GB of memory. "
                      "A minimum of 2 GB of memory is recommended to use high-resolution textures. ");
        message += '\n';
      }
      if (wr_gl_state_get_gpu_memory() < 1048576)  // Less than 1 GB of GPU memory
        reduceTextureQuality = 3;
      else
        reduceTextureQuality = 1;
    }
  }

  // 3. apply the parameter reducing
  if (disableShadows) {
    message += "\n - ";
    message += tr("Shadows have been deactivated.");
    WbPreferences::instance()->setValue("OpenGL/disableShadows", true);
  }

  if (disableAntiAliasing) {
    message += "\n - ";
    message += tr("Anti-aliasing has been deactivated.");
    WbPreferences::instance()->setValue("OpenGL/disableAntiAliasing", true);
  }

  if (disableGTAO) {
    message += "\n - ";
    message += tr("Main 3D view global ambient occlusion has been de-activated.");
    WbPreferences::instance()->setValue("OpenGL/GTAO", 0);
  }

  if (reduceTextureQuality != 0) {
    message += "\n - ";
    message += tr("Texture quality has been reduced.");
    WbPreferences::instance()->setValue("OpenGL/textureQuality", 4 - reduceTextureQuality);
  }

  if (maxTextureFiltering < WbPreferences::instance()->value("OpenGL/textureFiltering").toInt()) {
    message += "\n - ";
    message += tr("Texture maximum filtering has been reduced due to GPU limitations.");
    WbPreferences::instance()->setValue("OpenGL/textureFiltering", maxTextureFiltering);
  }

  // 4. check OpenGL capabilities.
  if (!wr_gl_state_is_anisotropic_texture_filtering_supported()) {
    message += "\n - ";
    message += tr("Anisotropic texture filtering is not supported by the GPU.");
  }

  // 5. complete and display the message
  if (!message.isEmpty()) {
    message += "\n\n";
    if (disableShadows || disableAntiAliasing || disableGTAO || reduceTextureQuality)
      message += tr("You can try to re-activate some OpenGL features from the OmniSim preferences.");
    else
      message +=
        tr("If there are some 3D rendering issues, you can try to reduce some OpenGL features from the OmniSim preferences.");

    WbLog::warning(tr("System below the minimal requirements.") + "\n\n" + message, true);
  }
}

void WbView3D::initialize() {
  // prepare WREN rendering context
  WbWrenRenderingContext::setWrenRenderingContext(width(), height());
  mWrenRenderingContext = WbWrenRenderingContext::instance();

  // propagate main window refresh signals
  connect(this, &WbView3D::mainRenderingStarted, mWrenRenderingContext, &WbWrenRenderingContext::mainRenderingStarted);
  connect(this, &WbView3D::mainRenderingEnded, mWrenRenderingContext, &WbWrenRenderingContext::mainRenderingEnded);

  // refresh for example when the user change an optional rendering option or
  // the rendering device external window is closed
  connect(mWrenRenderingContext, &WbWrenRenderingContext::view3dRefreshRequired, this, &WbView3D::renderLater);

  if (wr_gl_state_is_initialized())
    return;

  WbWrenWindow::initialize();

  if (WbPreferences::instance()->value("Internal/firstLaunch").toBool())
    checkRendererCapabilities();

  wr_config_enable_shadows(!WbPreferences::instance()->value("OpenGL/disableShadows").toBool());

  // reset timer
  mLastRefreshTimer.start();

  WbRenderingDeviceWindowFactory::storeOpenGLContext(WbWrenOpenGlContext::instance());
}

void WbView3D::resizeWren(int width, int height) {
  if (!mWorld)
    return;

  if (mWrenRenderingContext)
    mWrenRenderingContext->setDimension(width, height);

  if (mDisabledRenderingOverlay && mDisabledRenderingOverlay->isVisible())
    rescaleFastModePanel();

  if (mLoadingWorldOverlay && mLoadingWorldOverlay->isVisible())
    mLoadingWorldOverlay->adjustSize();

  if (mVirtualRealityHeadsetOverlay && mVirtualRealityHeadsetOverlay->isVisible())
    mVirtualRealityHeadsetOverlay->adjustSize();

  if (!wr_gl_state_is_initialized())
    return;

  if (mWorld) {
    mAspectRatio = (double)width / height;
    mWorld->viewpoint()->updateAspectRatio(mAspectRatio);
  }

  WbWrenWindow::resizeWren(width, height);

  emit resized();
}

void WbView3D::renderNow(bool culling, bool offScreen) {
  if (!wr_gl_state_is_initialized())
    initialize();

  if (mWorld) {
    emit mainRenderingStarted(mPhysicsRefresh);
#ifdef _WIN32
    if (WbVirtualRealityHeadset::isInUse()) {
      WbVirtualRealityHeadset::instance()->updateOrientationAndPosition();
      WbWrenOpenGlContext::makeWrenCurrent();
      if (mVirtualRealityHeadsetOverlay) {
        // on quit it might be possible that 'cleanupFullScreenOverlay' is called before the world actual destruction
        mVirtualRealityHeadsetOverlay->render();
      }
      wr_viewport_render_overlays(wr_scene_get_viewport(wr_scene_get_instance()));
      WbWrenWindow::blitMainFrameBufferToScreen();
      WbWrenOpenGlContext::instance()->swapBuffers(this);
      WbWrenOpenGlContext::doneWren();
    } else
#endif
      // R4 3c-B backend dispatch: try the wgpu main-view path when the Viewpoint selects it; otherwise
      // (default, or wgpu unavailable/failed) fall through to the byte-identical WREN path.
      if (!renderMainFrameViaWgpu(culling, offScreen))
        WbWrenWindow::renderNow(culling, offScreen);
    mLastRefreshTimer.start();
    if (!mFpsAccumulationTimer.isValid())
      mFpsAccumulationTimer.start();
    ++mRenderedFrameCount;
    emit mainRenderingEnded(mPhysicsRefresh);

    // take screenshot if needed
    if (mScreenshotRequested) {
      mScreenshotRequested = false;
      emit screenshotReady();
    }
  }
}

// First WbDirectionalLight anywhere in the world tree (e.g. the OmniSimSun PROTO's light), so the
// wgpu main view lights + casts shadows from WREN's actual sun direction. Mirrors the same-named
// helper in WbWgpuView.cpp's parity self-check; kept file-local to avoid a cross-TU dependency.
static WbDirectionalLight *findFirstDirectionalLightV3D(WbBaseNode *root) {
  if (!root)
    return nullptr;
  if (WbDirectionalLight *dl = dynamic_cast<WbDirectionalLight *>(root))
    return dl;
  if (WbGroup *g = dynamic_cast<WbGroup *>(root)) {
    const int n = g->childCount();
    for (int i = 0; i < n; ++i)
      if (WbDirectionalLight *d = findFirstDirectionalLightV3D(g->child(i)))
        return d;
  }
  return nullptr;
}

// First WbBackground in the world tree — its skyColor is the world's sky tint, fed as the
// hemisphere-IBL ambient (sky from above, a darkened bounce from below) so shadowed regions take a
// world-appropriate fill instead of crushing to black. This is the world-general replacement for the
// old panda-tuned "ambient 0" — the fix that makes the wgpu main view look right across the corpus,
// not just on panda.
static WbBackground *findFirstBackgroundV3D(WbBaseNode *root) {
  if (!root)
    return nullptr;
  if (WbBackground *b = dynamic_cast<WbBackground *>(root))
    return b;
  if (WbGroup *g = dynamic_cast<WbGroup *>(root)) {
    const int n = g->childCount();
    for (int i = 0; i < n; ++i)
      if (WbBackground *b = findFirstBackgroundV3D(g->child(i)))
        return b;
  }
  return nullptr;
}

bool WbView3D::renderMainFrameViaWgpu(bool culling, bool offScreen) {
  // R4 3c-B (increment 1b: the real wgpu main-view render). When the active Viewpoint selects wgpu AND the
  // backend is available, render the live world OFFSCREEN via wgpu → RGBA, then blit that into this WREN
  // window's GL framebuffer (offscreen render → GL blit; the HWND keeps its OpenGL pixel format, so there
  // is no surface-type conflict — safe + reversible). WREN stays the default; any failure pins the view
  // back to WREN (sticky) so the worst case degrades to a normal WREN frame.
  (void)culling;
  if (offScreen || mWgpuMainViewUnavailable || mWgpuMainViewSuspended)
    return false;  // offscreen/sensor/screenshot or mid-(re)load → stay on WREN; a prior failure → WREN
  // Stale-world guard: during a reload the old world is freed before setWorld() updates mWorld, so a
  // paint event firing in that window would deref a dangling mWorld → crash (WREN dodges this by using
  // WbWorld::instance()). Skip the wgpu path whenever mWorld isn't the current global world; it resumes
  // once setWorld() re-syncs. (Pointer compare only — never dereferences the stale mWorld.)
  if (!mWorld || static_cast<const WbWorld *>(mWorld) != WbWorld::instance())
    return false;
  if (!mWorld->viewpoint())
    return false;
  WbViewpoint *const vp = mWorld->viewpoint();
  WbRenderBackend *const backend = vp->renderBackend();
  // OMNISIM_WGPU_MAINVIEW_FORCE (test lever): drive the main view through wgpu on ANY world, bypassing
  // the Viewpoint's renderBackend field — for soak-testing the real main-view path on a stock world
  // without authoring a wgpu-Viewpoint world. With it OFF (the default), a world must select
  // `renderBackend "wgpu"` on its Viewpoint to use wgpu, so the default/WREN path stays byte-identical.
  // The wgpu backend availability is still required (checked lazily below).
  const bool forceWgpu = qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_FORCE");
  if (!forceWgpu &&
      (!backend || backend->kind() != WbRenderBackendKind::Vulkan || !backend->isAvailable()))
    return false;  // WREN default (or wgpu unavailable) → byte-identical WREN path

  // 3c-B UN-GATED (2026-06-07): a Viewpoint that selects `renderBackend "wgpu"` now renders the main
  // view through wgpu directly — no experimental flag (the former OMNISIM_WGPU_MAINVIEW gate) required.
  // The sustained-use VRAM OOM that formerly gated this (the ~30 s "0xC0000409 after ~2000 frames"
  // fault) was an APP-LEVEL texture-cache key bug — shared-file textures re-uploaded once per
  // PROTO-instance — fixed by path-keying the cache (a4fec74b, WbWgpuSceneRenderer::stableTexId), and
  // verified by a 6-world sustained soak (75 s+, texture count plateaus, 0 wgpu errors). Default worlds
  // keep `renderBackend "wren"` (the Viewpoint.wrl default), so this branch is unreachable for them and
  // the WREN path stays byte-identical. The wgpu render is functional + leak-free but not yet at full
  // lighting parity (~75%); flipping the DEFAULT to wgpu (Phase ζ) stays gated on §5.2 parity +
  // cross-platform surfaces. OMNISIM_WGPU_MAINVIEW_FORCE (above) still forces any world for testing.

  // Lazily create the wgpu render resources the first time a Viewpoint selects wgpu.
  if (!mWgpuBackend) {
    mWgpuBackend = static_cast<WbVulkanBackend *>(WbRenderBackendRegistry::vulkanBackend());
    if (!mWgpuBackend || !mWgpuBackend->isAvailable()) {
      mWgpuMainViewUnavailable = true;
      // Loud, always-on signal (the WbVulkanBackend ctor already explains WHY on
      // stderr->omnisim_log.txt). Without this, a requested-but-unavailable wgpu
      // main view fell back to WREN with zero trace -- the "why is it still 0.4x?"
      // black box. stderr is captured into omnisim_log.txt even on the GUI binary.
      fprintf(stderr, "[WbView3D] wgpu main view requested (%s) but the wgpu backend is "
                      "unavailable -- using WREN. (See the [WbWgpuBackend] line above for why.)\n",
              forceWgpu ? "OMNISIM_WGPU_MAINVIEW_FORCE" : "Viewpoint renderBackend \"wgpu\"");
      fflush(stderr);
      return false;
    }
    mWgpuMeshCache = new WbWgpuMeshCache(mWgpuBackend);
    mWgpuTextureCache = new WbWgpuTextureCache(mWgpuBackend);
    WbLog::info(tr("[WbView3D] main view now rendering through the wgpu backend (renderBackend \"wgpu\")."));
  }

  // Camera from the live Viewpoint (same convention as WbWgpuView::buildViewpointCamera).
  if (!vp->position() || !vp->orientation())
    return false;
  const WbVector3 eye = vp->position()->value();
  const WbRotation rot = vp->orientation()->value();
  const WbVector3 fwd = rot.direction().normalized();
  const WbVector3 rgt = fwd.cross(rot.up()).normalized();
  const WbVector3 up = rgt.cross(fwd);
  const WbMatrix4 cam(fwd.x(), -rgt.x(), up.x(), eye.x(), fwd.y(), -rgt.y(), up.y(), eye.y(), fwd.z(),
                      -rgt.z(), up.z(), eye.z(), 0, 0, 0, 1);
  const double horizFov = vp->fieldOfView() ? vp->fieldOfView()->value() : 0.785;

  const int W = std::max(1, static_cast<int>(width() * devicePixelRatio()));
  const int H = std::max(1, static_cast<int>(height() * devicePixelRatio()));
  // Cache the offscreen target (recreate only on resize) — creating one PER FRAME leaks/exhausts GPU
  // resources and faulted after ~1k frames.
  if (!mWgpuRenderTarget || mWgpuRtWidth != W || mWgpuRtHeight != H) {
    delete mWgpuRenderTarget;
    mWgpuRenderTarget =
      new WbWgpuRenderTarget(mWgpuBackend, static_cast<uint32_t>(W), static_cast<uint32_t>(H));
    mWgpuRtWidth = W;
    mWgpuRtHeight = H;
  }
  if (!mWgpuRenderTarget || !mWgpuRenderTarget->isUsable()) {
    mWgpuMainViewUnavailable = true;
    return false;
  }
  const double aspect = static_cast<double>(W) / static_cast<double>(H);
  const double hf = aspect < 1.0 ? 2.0 * std::atan(std::tan(0.5 * horizFov) * aspect) : horizFov;
  float vpm[16];
  // Authored near plane: depth precision is NEAR-dominated — the hardcoded 0.05 vs the city's
  // authored 0.3 cost 6x precision and z-fought the road decals at orbit distance (user-visible
  // flicker). Floor 0.05 preserves old behaviour for worlds that author nothing.
  double zNear = 0.05;
  if (vp->nearField() && vp->nearField()->value() > 0.05)
    zNear = vp->nearField()->value();
  const bool revZ = !qEnvironmentVariableIsSet("OMNISIM_WGPU_NO_REVZ");
  WbWgpuSceneRenderer::buildViewProj(cam, hf, aspect, zNear, 1000.0, vpm, revZ);
  std::vector<WbWgpuSolidDraw> &draws = mWgpuDrawList;
  std::vector<std::array<float, 16>> &modelStorage = mWgpuModelList;  // draws alias into this
  // The WREN GL context MUST be current before collecting draws: collectWorldDraws → acquireFromWren →
  // wr_static_mesh_read_data falls back to glGetBufferSubData for non-primitive geometry (IndexedFaceSet,
  // CadShape, …). With no current GL context that readback returns garbage, so complex meshes (the
  // floor, etc.) get cached with bad vertices on the FIRST frame and stay invisible forever (the cache
  // keys on the WrStaticMesh* and never re-reads). renderNow() does not make the context current before
  // this path (only the VR-headset branch did), so do it here — primitives use CPU builders and are
  // unaffected, but the readback path needs it. (This is why a fresh cache collected after a prior
  // render+blit rendered correctly while the persistent first-frame cache did not.)
  // Per-phase frame timing, reported through OMNISIM_WGPU_REPORT (inert otherwise) — the measure
  // that directs the perf ladder: collect (scene walk) vs render+readback (GPU+map-wait) vs blit.
  QElapsedTimer phaseTimer;
  phaseTimer.start();
  qint64 tCollect = 0, tRender = 0;
  static qint64 sPrevBlitMs = -1;  // last frame's blit cost (this frame's blit runs after the report)
  // Motion-smoothness diagnostics: the WORST inter-frame gap and the worst collect (rebuild hitch)
  // in each 100-frame report window — judder shows up here, not in the average FPS.
  static QElapsedTimer sFrameGapTimer;
  static qint64 sMaxGapMs = 0, sMaxCollectMs = 0;
  if (sFrameGapTimer.isValid()) {
    const qint64 gap = sFrameGapTimer.elapsed();
    if (gap > sMaxGapMs)
      sMaxGapMs = gap;
  }
  sFrameGapTimer.restart();
  // Draw-list cache: full scene walk only when dirty (a referenced node died, the root's children
  // changed, a robot was added/removed, world load) or every 600 frames as a slow fallback for
  // changes no hook catches (deep node insertion, appearance edits); otherwise refresh just the
  // model matrices. The walk costs ~50 ms on the 3.5k-draw city, the refresh ~1 ms — at the old
  // 30-frame cadence that was a visible motion HITCH every ~2 s (measured maxGapMs 96–130 vs the
  // ~58 ms norm: "the cars stutter"). Signal-driven rebuilds keep correctness without the rhythm.
  if (mWgpuDrawListDirty || mWgpuRefreshList.empty() || mWgpuDrawListAge >= 600 ||
      !WbWgpuSceneRenderer::refreshWorldDraws(modelStorage, mWgpuRefreshList)) {
    invalidateWgpuDrawList();
    WbWrenOpenGlContext::makeWrenCurrent();
    WbWgpuSceneRenderer::collectWorldDraws(*mWgpuMeshCache, draws, modelStorage, nullptr, mWgpuTextureCache,
                                           &mWgpuRefreshList);
    // Balance the makeWrenCurrent() above. makeWrenCurrent/doneWren PUSH/POP a context-state stack, and
    // the blit below does its own make/done pair — without this done, every wgpu frame net-pushes one
    // entry, growing the stack unbounded and corrupting WREN's context-active state, which then faults
    // when the world is torn down on reload (the wgpu-specific reload crash). The GL context is not
    // needed between here and the blit (the wgpu render is Vulkan; the sky/light harvest is CPU).
    WbWrenOpenGlContext::doneWren();
    // A destroyed scene node would dangle the cached geom/WrTransform pointers — hook every
    // referenced node's destroyed() to invalidate the cache BEFORE the next frame can touch them.
    QSet<QObject *> hooked;
    for (const WbWgpuSceneRenderer::WbWgpuDrawRefresh &r : mWgpuRefreshList)
      if (r.node && !hooked.contains(r.node)) {
        hooked.insert(r.node);
        mWgpuDrawListConns.push_back(
          connect(r.node, &QObject::destroyed, this, [this]() { invalidateWgpuDrawList(); }));
      }
    // Structural-change hooks (replace the old 30-frame timer): top-level node additions (the
    // world root's children) and robot add/remove rebuild the list the moment they happen.
    // Qt::UniqueConnection dedupes across rebuilds.
    if (WbWorld::instance()) {
      if (WbGroup *root = WbWorld::instance()->root())
        connect(root, &WbGroup::childrenChanged, this, &WbView3D::invalidateWgpuDrawList,
                Qt::UniqueConnection);
      connect(WbWorld::instance(), &WbWorld::robotAdded, this, &WbView3D::invalidateWgpuDrawList,
              Qt::UniqueConnection);
      connect(WbWorld::instance(), &WbWorld::robotRemoved, this, &WbView3D::invalidateWgpuDrawList,
              Qt::UniqueConnection);
    }
    mWgpuDrawListDirty = false;
  } else
    ++mWgpuDrawListAge;
  tCollect = phaseTimer.elapsed();
  if (tCollect > sMaxCollectMs)
    sMaxCollectMs = tCollect;  // worst collect in the window — the 30-frame rebuild hitch shows here
  if (mWgpuRgba.size() != static_cast<size_t>(W) * H * 4)
    mWgpuRgba.resize(static_cast<size_t>(W) * H * 4);  // overwritten in full by the readback
  std::vector<uint8_t> &rgba = mWgpuRgba;
  // Window-swap presentation: lazily create an input-transparent Vulkan-surface CHILD window over
  // this view; when usable, frames present GPU→GPU (presentTexture samples the offscreen texture)
  // and the readback below is SKIPPED entirely (rgba=nullptr). Mouse/keyboard pass through to this
  // view, so the full editor interaction is preserved. Any failure → the legacy blit path.
  if (!mWgpuPresentWindow) {
    mWgpuPresentWindow = new QWindow(this);
    mWgpuPresentWindow->setSurfaceType(QWindow::VulkanSurface);
    mWgpuPresentWindow->setFlags(mWgpuPresentWindow->flags() | Qt::WindowTransparentForInput);
    mWgpuPresentWindow->setGeometry(0, 0, width(), height());
    mWgpuPresentWindow->show();
  }
  if (mWgpuPresentWindow->width() != width() || mWgpuPresentWindow->height() != height())
    mWgpuPresentWindow->setGeometry(0, 0, width(), height());
  if (!mWgpuPresentSurface && mWgpuPresentWindow->handle()) {
    void *hwnd = reinterpret_cast<void *>(mWgpuPresentWindow->winId());
    void *hinst = nullptr;
#ifdef _WIN32
    hinst = reinterpret_cast<void *>(GetModuleHandleW(nullptr));
#endif
    mWgpuPresentSurface = new WbWgpuSurface(mWgpuBackend, hwnd, hinst, static_cast<uint32_t>(W),
                                            static_cast<uint32_t>(H));
    if (!mWgpuPresentSurface->isUsable()) {
      delete mWgpuPresentSurface;
      mWgpuPresentSurface = nullptr;  // blit fallback (no retry churn: window handle was valid)
    }
  }
  // OMNISIM_WGPU_NO_SWAP=1: kill-switch back to the readback+blit path (visual safety valve).
  const bool present = mWgpuPresentSurface != nullptr && !qEnvironmentVariableIsSet("OMNISIM_WGPU_NO_SWAP");
  if (present)
    mWgpuPresentSurface->resize(static_cast<uint32_t>(W), static_cast<uint32_t>(H));
  const bool wantDump = qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_DUMP");
  void *rgbaOut = (present && !wantDump) ? nullptr : static_cast<void *>(rgba.data());
  // World-general sky tint: harvest the scene's Background skyColor (the sky the viewer sees), used
  // BOTH as the clear colour AND as the hemisphere-IBL ambient. Falls back to a neutral sky-blue when
  // there's no Background, or it's near-black (TexturedBackground / NightSky) — so shadows still get a
  // small fill rather than crushing. This replaces the panda-tuned hardcoded clear + the "ambient 0"
  // that made every non-panda world render with crushed-black shadows.
  float skyC[3] = {0.45f, 0.62f, 0.85f};
  if (mWorld && mWorld->root())
    if (WbBackground *bg = findFirstBackgroundV3D(mWorld->root())) {
      const WbRgb sc = bg->skyColor();
      const float r = static_cast<float>(sc.red()), g = static_cast<float>(sc.green()),
                  b = static_cast<float>(sc.blue());
      if (r + g + b > 0.05f) {  // keep the default for an unset/black sky
        skyC[0] = r;
        skyC[1] = g;
        skyC[2] = b;
      }
    }
  WbWgpuClearColor sky;
  sky.r = skyC[0];
  sky.g = skyC[1];
  sky.b = skyC[2];
  // Hemisphere-IBL ambient from the sky tint: sky colour from above, a dim warm bounce from below,
  // blended by the surface normal's up-component. The shader adds this as UN-shadowed fill, so shadowed
  // regions take a soft sky/ground colour instead of going black — the world-general fix for the
  // crushed-black regression. Intensity 0.4 keeps shadows clearly darker than lit areas, not washed out.
  // Floor the AMBIENT sky colour so shadowed regions never crush to black on worlds with a dim/dark
  // Background (the clear colour above still uses the real sky). A minimum neutral fill keeps robots +
  // floors readable regardless of the world's sky luminance — the "dark scene" fix.
  const float ambSky[3] = {skyC[0] > 0.40f ? skyC[0] : 0.40f, skyC[1] > 0.42f ? skyC[1] : 0.42f,
                           skyC[2] > 0.45f ? skyC[2] : 0.45f};
  const float hemiSky4[4] = {ambSky[0], ambSky[1], ambSky[2], 0.45f};
  const float hemiGround4[4] = {ambSky[0] * 0.30f, ambSky[1] * 0.29f, ambSky[2] * 0.27f, 0.0f};
  float worldUp3[3] = {0.0f, 0.0f, 1.0f};
  if (mWorld && mWorld->worldInfo()) {
    const WbVector3 up = -mWorld->worldInfo()->gravityUnitVector();  // up = opposite gravity
    if (up.length() > 1e-6) {
      worldUp3[0] = static_cast<float>(up.x());
      worldUp3[1] = static_cast<float>(up.y());
      worldUp3[2] = static_cast<float>(up.z());
    }
  }
  // R4 3c-B: light the wgpu main view with the REAL scene sun (the OmniSimSun PROTO's
  // WbDirectionalLight) + cast shadows, so the live wgpu view is shadow-dominated like WREN instead of
  // flat-lit. Mirrors the parity self-check's sun harvest + light-frustum + shadowed render. Ambient is
  // kept small (WREN renders this scene shadow-dominated); the finer sun-shaft shadow-placement parity
  // is a follow-up. Falls back to a hardcoded direction if no directional light is found.
  float lit4[4] = {0.3f, 0.4f, -0.85f, 0.05f};
  float sunColor3[3] = {1.0f, 1.0f, 1.0f};
  bool haveSun = false;
  if (mWorld && mWorld->root())
    if (WbDirectionalLight *sun = findFirstDirectionalLightV3D(mWorld->root())) {
      const WbVector3 sd = sun->direction().normalized();
      lit4[0] = static_cast<float>(sd.x());
      lit4[1] = static_cast<float>(sd.y());
      lit4[2] = static_cast<float>(sd.z());
      const WbRgb sc = sun->color();
      sunColor3[0] = static_cast<float>(sc.red());
      sunColor3[1] = static_cast<float>(sc.green());
      sunColor3[2] = static_cast<float>(sc.blue());
      haveSun = true;
    }
  // HDR + AgX filmic tonemapping: always on for the wgpu main view (exposure from the Viewpoint's
  // exposure field, default 1.0) — the contrast/colour response that makes WREN's output read
  // "graded" while plain linear→gamma reads flat.
  // OPT-IN (OMNISIM_WGPU_AGX=<exposure>), default OFF: A/B on the city showed default AgX reads
  // milky at exposure 1.0 and pastel-blown at 2.5 — our shading is display-tuned, so the filmic
  // curve double-compresses; WREN's perceptual edge on this scene is AO + shadow crispness, not
  // the tone curve. The full HDR pipeline (RGBA16F + tonemap pass) stays available behind this
  // flag for side-by-side judging once SSAO/CSM land.
  float agxExposure = 0.0f;
  if (qEnvironmentVariableIsSet("OMNISIM_WGPU_AGX"))
    agxExposure = qEnvironmentVariable("OMNISIM_WGPU_AGX").toFloat();
  // SSAO: enabled when the Viewpoint authors a positive ambientOcclusionRadius (the city sets 2).
  // Contact darkening at building bases / under cars — the depth/weight cue WREN's GTAO provides.
  float ssaoStrength = 0.0f;
  if (vp->ambientOcclusionRadiusField() && vp->ambientOcclusionRadiusField()->value() > 0.0 &&
      !qEnvironmentVariableIsSet("OMNISIM_WGPU_NO_SSAO"))  // diagnostic kill-switch
    ssaoStrength = 1.0f;
  // Bloom: enabled when the Viewpoint authors a non-negative bloomThreshold (the city sets 6;
  // -1 disables). The HDR threshold maps to an LDR composite strength — lower thresholds bloom
  // stronger, anchored so the city's 6 → 0.55 and the Webots default 21 → subtle.
  float bloomStrength = 0.0f;
  if (vp->bloomThresholdField() && vp->bloomThresholdField()->value() >= 0.0) {
    const double bt = vp->bloomThresholdField()->value();
    bloomStrength = static_cast<float>(std::min(0.8, 0.55 * (6.0 / (bt > 1.0 ? bt : 1.0))));
  }
  // Fog: the world's Fog node (the city ships one — exponential, 420 m, pale blue), shaded
  // per-pixel in the wgpu scene shader. Density 3/visibilityRange ≈ 95% fogged at the authored
  // range, approximating WREN's GL exponential fog. Absent node → density 0 → off.
  float fogParams4[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  if (WbFog *fog = WbFog::fogInstance()) {
    if (fog->fogColor() && fog->fogVisibilityRange() && fog->fogVisibilityRange()->value() > 0.1) {
      const WbRgb fc = fog->fogColor()->value();
      fogParams4[0] = static_cast<float>(fc.red());
      fogParams4[1] = static_cast<float>(fc.green());
      fogParams4[2] = static_cast<float>(fc.blue());
      fogParams4[3] = static_cast<float>(1.0 / fog->fogVisibilityRange()->value());  // WREN-exact: density=1/range (the shader applies exp2 + the pow-2.2 blend)
    }
  }
  const float camPos[3] = {static_cast<float>(eye.x()), static_cast<float>(eye.y()),
                           static_cast<float>(eye.z())};
  // Light frustum: an orthographic light camera looking along the sun direction, positioned back
  // along it, with a scene-covering ortho extent (the factory floor/walls far exceed a small extent).
  const WbVector3 sdir = WbVector3(lit4[0], lit4[1], lit4[2]).normalized();
  WbVector3 lightAxis = WbVector3(1.0, 0.0, 0.0).cross(sdir);
  const double laLen = lightAxis.length();
  const double laAng = std::acos(std::max(-1.0, std::min(1.0, WbVector3(1.0, 0.0, 0.0).dot(sdir))));
  lightAxis = laLen > 1e-6 ? lightAxis / laLen : WbVector3(0.0, 1.0, 0.0);
  // Camera-following fitted light frustum (the "cascade 0" the crispness complaint needs): a 45 m
  // half-extent anchored ahead of the camera gives 4× the shadow texel density of the old fixed
  // origin-centred 90 m box, exactly where the viewer looks. The anchor snaps to the shadow-map
  // texel grid so shadow edges don't shimmer as the camera pans; outside the box the shader's
  // out-of-frustum guard renders unshadowed (distant areas keep AO + fog as depth cues — WREN's
  // CSM also fades far shadows).
  const double kShadowHalfExtent = 45.0;
  // Anchor where the viewer LOOKS: the view ray's ground intersection (an aerial camera looks
  // hundreds of meters ahead — "30 m in front of the eye" left its focus outside the box). A
  // horizontal/upward ray falls back to a point ahead of the eye.
  WbVector3 anchor;
  if (fwd.z() < -1e-3) {
    const double t = std::min((eye.z() - 0.4) / -fwd.z(), 400.0);
    anchor = eye + fwd * t;
  } else {
    WbVector3 fwdH(fwd.x(), fwd.y(), 0.0);
    fwdH = fwdH.length() > 1e-3 ? fwdH.normalized() : WbVector3(1.0, 0.0, 0.0);
    anchor = eye + fwdH * (kShadowHalfExtent * 0.6);
  }
  const double texelWorld = 2.0 * kShadowHalfExtent / std::max(1, std::min(W, H));
  anchor = WbVector3(std::floor(anchor.x() / texelWorld) * texelWorld,
                     std::floor(anchor.y() / texelWorld) * texelWorld, 0.4);
  const WbVector3 lightPos = anchor - sdir * 130.0;
  const WbMatrix4 lightWorld(lightPos.x(), lightPos.y(), lightPos.z(), lightAxis.x(), lightAxis.y(),
                             lightAxis.z(), laAng);
  float lightVP[16] = {0};
  WbWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, kShadowHalfExtent, 0.05, 260.0, lightVP);
  // R4 3c-B un-gate leak-hunt: instrument the main-view path. Trace draw count + W/H + the render
  // result + the wgpu resource-registry report every 100 frames, BEFORE the failure-latch, so a
  // sustained soak shows whether the registry climbs toward the historical VRAM OOM. Gated by
  // OMNISIM_WGPU_REPORT=<file>; inert otherwise (WREN/default path byte-identical). Diagnostic only.
  static long sWgpuMainViewFrame = 0;
  const long f = sWgpuMainViewFrame++;
  // R4 3c-B (L3↔L2 wiring): OMNISIM_WGPU_MAINVIEW_CSM (default-off) routes the main view through the
  // FULL-MATERIAL multi-cascade shadow path (WbWgpuRenderTarget::clearAndDrawSceneTexturedCsm) — the
  // same material path (albedo/roughness/metalness/normal + GGX) as the default textured-shadow render,
  // but with N per-camera-frustum cascades (buildCascadeLightViewProjs over the [0.05, 40] shadow range)
  // instead of the single fixed-extent ortho light frustum, so near shadows are tighter. This is the
  // candidate for the eventual main-view default; default-off keeps the single-cascade textured-shadow
  // render — and the panda parity golden — byte-identical until the flip is human-gated.
  bool cdOk;
  if (qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_CSM")) {
    const int NC = 3;
    float lvps[WbWgpuSceneRenderer::kMaxCascades * 16] = {0};
    float splits[WbWgpuSceneRenderer::kMaxCascades + 1] = {0};
    WbWgpuSceneRenderer::buildCascadeLightViewProjs(cam, hf, aspect, 0.05, 40.0, lightWorld, NC, 0.6,
                                                    lvps, splits);
    float splitsFar4[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // cascade far view-depth boundaries
    for (int ci = 0; ci < NC && ci < 4; ++ci)
      splitsFar4[ci] = splits[ci + 1];
    cdOk = mWgpuRenderTarget->clearAndDrawSceneTexturedCsm(
      sky, vpm, lvps, splitsFar4, static_cast<uint32_t>(NC), lit4, draws.data(),
      static_cast<uint32_t>(draws.size()), 0.8f /*shadowStrength: WREN outdoor shadows keep partial direct — 1.0 read too dark*/, 0.0006f /*depthBias: NDC over the 260m light range = ~0.16m; the old 0.003 became ~0.8m after the frustum widening -> visibly detached shadows*/, camPos,
      hemiSky4, hemiGround4, worldUp3, rgba.data());
  } else {
    // Atmospheric sky + day-night (the wgpu counterpart of Background.atmosphericSky + the
    // sun_marker system): SkyU = camera basis scaled by the half-FOV tangents (per-pixel ray
    // reconstruction), TOWARD-sun, the light's colour, world up. The day factor — sun elevation
    // smoothed through twilight — also dims the scene's DIRECT term, so dragging the sun marker
    // below the horizon darkens geometry in step with the dome. Full day → directScale 1.0 →
    // the gate-passing render is unchanged.
    const double halfW = std::tan(0.5 * hf);
    const double halfH = halfW / aspect;
    const WbVector3 towardSun(-lit4[0], -lit4[1], -lit4[2]);
    const WbVector3 upWorld(worldUp3[0], worldUp3[1], worldUp3[2]);
    const double sunElev = towardSun.normalized().dot(upWorld.normalized());
    const float dayF = static_cast<float>(std::max(0.05, std::min(1.0, sunElev * 5.0 + 0.5)));
    const float skyU[24] = {
      static_cast<float>(rgt.x() * halfW), static_cast<float>(rgt.y() * halfW),
      static_cast<float>(rgt.z() * halfW), 0.0f,
      static_cast<float>(up.x() * halfH), static_cast<float>(up.y() * halfH),
      static_cast<float>(up.z() * halfH), 0.0f,
      static_cast<float>(fwd.x()), static_cast<float>(fwd.y()), static_cast<float>(fwd.z()), 0.0f,
      static_cast<float>(towardSun.x()), static_cast<float>(towardSun.y()),
      static_cast<float>(towardSun.z()), 0.0f,
      sunColor3[0], sunColor3[1], sunColor3[2], 0.0f,
      worldUp3[0], worldUp3[1], worldUp3[2], 0.0f};
    cdOk = mWgpuRenderTarget->clearAndDrawSceneTexturedShadowed(
      sky, vpm, lightVP, lit4, draws.data(), static_cast<uint32_t>(draws.size()),
      qEnvironmentVariableIsSet("OMNISIM_WGPU_NO_SHADOW") ? 0.0f
        : 0.8f /*shadowStrength: WREN outdoor shadows keep partial direct (env = diagnostic kill)*/,
      qEnvironmentVariableIsSet("OMNISIM_WGPU_SHADOW_DEBUG") ? -0.0006f : 0.0006f /*depthBias; negative = error-field debug*/,
      camPos, hemiSky4, hemiGround4, worldUp3, rgbaOut,
      /*asyncReadback=*/true,  // pipeline the readback: blit shows frame N-1, CPU never waits on the GPU
      haveSun ? skyU : nullptr, dayF, fogParams4[3] > 0.0f ? fogParams4 : nullptr, bloomStrength,
      agxExposure, ssaoStrength, revZ);
  }
  tRender = phaseTimer.elapsed();
  if (qEnvironmentVariableIsSet("OMNISIM_WGPU_REPORT") && (f % 100 == 0 || (!cdOk && f < 5))) {
    const QString rpath = qEnvironmentVariable("OMNISIM_WGPU_REPORT");
    QFile tf(rpath);
    if (tf.open(QIODevice::Append | QIODevice::Text)) {
      tf.write(QString("frame=%1 calls draws=%2 W=%3 H=%4 cdOk=%5 collectMs=%6 renderMs=%7 prevBlitMs=%8 maxGapMs=%9 maxCollectMs=%10\n")
                 .arg(f).arg(static_cast<qulonglong>(draws.size())).arg(W).arg(H).arg(cdOk ? 1 : 0)
                 .arg(tCollect).arg(tRender - tCollect).arg(sPrevBlitMs).arg(sMaxGapMs).arg(sMaxCollectMs)
                 .toUtf8());
      sMaxGapMs = 0;
      sMaxCollectMs = 0;
      tf.close();
    }
    mWgpuRenderTarget->appendResourceReport(rpath.toLocal8Bit().constData(), f);
  }
  if (!cdOk) {
    mWgpuMainViewUnavailable = true;
    return false;
  }

  // R4 3c-B: one-shot screenshot of the rendered wgpu main-view frame (after the scene settles at
  // f==200) for visual verification on any world. Gated by OMNISIM_WGPU_MAINVIEW_DUMP; inert otherwise.
  // OMNISIM_WGPU_MAINVIEW_DUMP_FRAME overrides the frame so short-lived worlds (whose controller quits
  // before frame 200) can still be captured early.
  const long dumpFrame = qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_DUMP_FRAME")
                           ? static_cast<long>(qEnvironmentVariableIntValue("OMNISIM_WGPU_MAINVIEW_DUMP_FRAME"))
                           : 200;
  if (f == dumpFrame && rgbaOut && qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_DUMP")) {
    QImage img(rgba.data(), W, H, W * 4, QImage::Format_RGBA8888);
    img.copy().save(qEnvironmentVariable("OMNISIM_WGPU_MAINVIEW_DUMP"), "PNG");
  }

  // Present: blit the wgpu RGBA into this window's GL framebuffer (raw GL is isolated in WbWgpuGlBlit.cpp).
  if (present) {
    // Native window-swap: GPU→GPU, no CPU pixels, no GL. (A failed acquire skips one frame.)
    mWgpuPresentSurface->presentTexture(mWgpuRenderTarget->sceneTextureView());
  } else {
    WbWrenOpenGlContext::makeWrenCurrent();
    WbWgpuGlBlitRgbaToScreen(rgba.data(), W, H);
    WbWrenOpenGlContext::instance()->swapBuffers(this);
    WbWrenOpenGlContext::doneWren();
  }
  sPrevBlitMs = phaseTimer.elapsed() - tRender;
  return true;
}

const WbMatter *WbView3D::remoteMouseEvent(QMouseEvent *event) {
  mRemoteContextMenuMatter = NULL;
  mIsRemoteMouseEvent = true;
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      mousePressEvent(event);
      break;
    case QEvent::MouseButtonRelease:
      mouseReleaseEvent(event);
      break;
    case QEvent::MouseMove:
      mouseMoveEvent(event);
      break;
    default:
      break;
  }
  mIsRemoteMouseEvent = false;
  return mRemoteContextMenuMatter;
}

void WbView3D::remoteWheelEvent(QWheelEvent *event) {
  wheelEvent(event);
}

void WbView3D::selectNode(const QMouseEvent *event) {
  if (mDisabledUserInteractionsMap.value(WbAction::DISABLE_SELECTION, false))
    return;

  // Object selection:
  // - at first click select the top Matter node
  // - at second click on the same geometry select the picked Matter node
  // - further clicks on the same geometry will toggle between picked and top Matter nodes
  // exception in case of context menu shortcut where the selected Matter node is always used
  WbSelection *const selection = WbSelection::instance();
  if (!mPickedMatter) {
    selection->selectPoseFromView3D(
      NULL, mDisabledUserInteractionsMap.value(WbAction::DISABLE_OBJECT_MOVE, false));  // sending NULL allows to unselect
    if (isContextMenuShortcut(event) && event->type() == QEvent::MouseButtonRelease) {
      if (mIsRemoteMouseEvent || mDisabledUserInteractionsMap.value(WbAction::DISABLE_3D_VIEW_CONTEXT_MENU, false))
        mRemoteContextMenuMatter = mPickedMatter;
      else
        emit contextMenuRequested(event->globalPosition().toPoint(), mParentWidget);
    }
    return;
  }

  const WbAbstractPose *const selectedAbstractPose = selection->selectedAbstractPose();
  WbMatter *visiblePickedMatter = WbNodeUtilities::findUpperVisibleMatter(mPickedMatter);
  WbMatter *selectedMatter = NULL;
  if (isContextMenuShortcut(event))
    selectedMatter = visiblePickedMatter;
  else {
    const WbMatter *const previousTopMatter =
      selectedAbstractPose != NULL ? WbNodeUtilities::findUppermostMatter(selectedAbstractPose->baseNode()) : NULL;
    WbMatter *topMatter = WbNodeUtilities::findUppermostMatter(visiblePickedMatter);
    if (topMatter == NULL)
      topMatter = visiblePickedMatter;
    const int alt = event->modifiers() & Qt::AltModifier;
    if (visiblePickedMatter == selectedAbstractPose) {
      if (alt)
        // do not change selection when starting force or torque drag
        return;
      if (topMatter == visiblePickedMatter) {
        // do not change selection if the picked node is already selected and it doesn't have any Matter ancestor
        selection->confirmSelectedAbstractPoseFromView3D();
        return;
      }
      selectedMatter = topMatter;
    } else if ((topMatter == previousTopMatter) || alt)
      selectedMatter = visiblePickedMatter;
    else
      selectedMatter = topMatter;
  }

  selection->selectPoseFromView3D(selectedMatter, mDisabledUserInteractionsMap.value(WbAction::DISABLE_OBJECT_MOVE, false));

  if (WbSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty())
    WbVisualBoundingSphere::instance()->show(selectedMatter);

  if (isContextMenuShortcut(event) && event->type() == QEvent::MouseButtonRelease) {
    if (mIsRemoteMouseEvent || mDisabledUserInteractionsMap.value(WbAction::DISABLE_3D_VIEW_CONTEXT_MENU, false))
      mRemoteContextMenuMatter = selectedMatter;
    else
      emit contextMenuRequested(event->globalPosition().toPoint(), mParentWidget);
  }
}

void WbView3D::mousePressEvent(QMouseEvent *event) {
  mLastButtonState = event->buttons();

  // Ignore if dragging handles
  // e.g. received on macOS when mouse moved outside application window area
  if (mDragTranslate || mDragRotate || (mDragTorque && !mDragTorque->isLocked()) || (mDragForce && !mDragForce->isLocked()) ||
      mDragVerticalAxisRotate)
    return;

#ifdef __APPLE__
  // Fix an issue on macOS where the context menu was not closed by a click.
  delete mParentWidget->findChild<QMenu *>("ContextMenu");
#endif

  mMouseEventInitialized = true;
  updateMousesPosition(true, false);

  const QPoint &position = event->pos();

  // Overlays come first - special case for overlay resize and close (resize has priority)
  bool displayOverlayClicked = false;
  WbWrenTextureOverlay *overlay = NULL;
  if (!mDragOverlay) {
    WbRenderingDevice *renderingDevice = WbRenderingDevice::fromMousePosition(position.x(), position.y());
    if (renderingDevice) {
      overlay = renderingDevice->overlay();
      if (overlay) {
        displayOverlayClicked = true;

        if (overlay->isInsideResizeArea(position.x(), position.y())) {
          // reset double click timer for resize area
          delete mMousePressTimer;
          mMousePressTimer = NULL;

          mLastMouseCursor = cursor();
          setCursor(QCursor(Qt::SizeFDiagCursor));

          overlay->putOnTop();
          mDragOverlay = new WbDragResizeOverlayEvent(position, renderingDevice);
          connect(renderingDevice, &QObject::destroyed, this, &WbView3D::abortOverlayDrag);

          return;
        } else if (overlay->isInsideCloseButton(position.x(), position.y())) {
          renderingDevice->toggleOverlayVisibility(false, true);

          // reset double click timer on close area
          delete mMousePressTimer;
          mMousePressTimer = NULL;
          return;
        } else {
          mLastMouseCursor = cursor();
          setCursor(QCursor(Qt::ClosedHandCursor));

          mDragOverlay = new WbDragTranslateOverlayEvent(position, QPoint(width(), height()), renderingDevice);
          connect(renderingDevice, &QObject::destroyed, this, &WbView3D::abortOverlayDrag);
        }
      }
    }
  }

  // if we didn't close an overlay perform double-click check as normal
  if ((event->buttons() == Qt::LeftButton) && mMousePressTimer) {
    int delay = mMousePressTimer->elapsed();
    if (delay < QApplication::doubleClickInterval()) {
      delete mMousePressTimer;
      mMousePressTimer = NULL;
      mouseDoubleClick(event);
      return;
    }
  }
  delete mMousePressTimer;
  mMousePressTimer = new QElapsedTimer();
  mMousePressTimer->start();
  mMousePressPosition = position;
  WbWrenWindow::mousePressEvent(event);

  if (!mWorld)
    return;

  cleanupWheel();

  // if we didn't close an overlay but still clicked on one (without this being
  // a double click), then handle this double click as normal and exit
  if (displayOverlayClicked) {
    overlay->putOnTop();
    return;
  }

  // Overlays come first
  if (!mDragOverlay) {
    WbRenderingDevice *renderingDevice = WbRenderingDevice::fromMousePosition(position.x(), position.y());
    if (renderingDevice) {
      overlay = renderingDevice->overlay();
      if (overlay) {
        if (overlay->isInsideCloseButton(position.x(), position.y()))
          renderingDevice->toggleOverlayVisibility(false, true);
        else
          overlay->putOnTop();
        renderLater();
        return;
      }
    }
  }

  // clear picked matter, this will be set again later once the picked matter (if any) has been deduced
  mPickedMatter = NULL;

  // Picks the WbNode and retrieves the corresponding WbGeometry
  mWorld->viewpoint()->storePickedCoordinates(WbVector3(0, 0, 0));

  bool picked = mPicker->pick(event->pos().x(), event->pos().y());
  if (picked) {
    const int id = mPicker->selectedId();

    // Check if a transformation handle was picked
    if (id == -1)
      return;

    WbVector3 screenCoords = mPicker->screenCoordinates();
    screenCoords[0] = (screenCoords[0] / width()) * 2 - 1;
    screenCoords[1] = (screenCoords[1] / height()) * 2 - 1;
    WbVector3 center;
    mWorld->viewpoint()->toWorld(screenCoords, center);
    mWorld->viewpoint()->storePickedCoordinates(center);

    mPickedMatter = WbNodeUtilities::findUpperMatter(WbNode::findNode(id));
  } else
    mWorld->viewpoint()->storePickedCoordinates(mWorld->viewpoint()->position()->value());

  if (isContextMenuShortcut(event))
    return;

  // Handle bumpers
  WbTouchSensor *const touchSensor = dynamic_cast<WbTouchSensor *>(mPickedMatter);
  if (touchSensor && touchSensor->deviceType() == WbTouchSensor::BUMPER) {
    touchSensor->setGuiTouch(true);
    mTouchSensor = touchSensor;
    selectNode(event);
  }
}

void WbView3D::leaveEvent(QEvent *event) {
  setCursor(QCursor(Qt::ArrowCursor));
  if (mWheel)
    cleanupWheel();
  cleanupCameraRecognizedObjectsOverlayIfNeeded();
}

void WbView3D::mouseMoveEvent(QMouseEvent *event) {
  if (!mWorld)
    return;

  updateMousesPosition(false, true);

  const QPoint &position = event->pos();

  // Unreal-style fly-mode mouselook: holding left mouse button while any WASD/QE key is down
  // turns the left-mouse drag from "orbit" into "free look" around the camera position. Stays in
  // mouselook for as long as LMB is held — releasing all WASD keys lets the user just look around.
  // The 5-pixel movement threshold prevents a stationary click + jiggle from accidentally entering
  // mouselook, so plain left-click selection still works while a fly key is held.
  if (!mFlyMouseLook && (event->buttons() & Qt::LeftButton) && !mFlyKeys.isEmpty() &&
      (position - mMousePressPosition).manhattanLength() > 5)
    enterFlyMouseLook();
  if (mFlyMouseLook) {
    if (!(event->buttons() & Qt::LeftButton)) {
      exitFlyMouseLook();
    } else {
      const QPoint delta = position - mFlyMouseAnchor;
      if (!delta.isNull()) {
        WbViewpoint *const viewpoint = mWorld->viewpoint();
        if (viewpoint && !viewpoint->isLocked() &&
            !mDisabledUserInteractionsMap.value(WbAction::LOCK_VIEWPOINT, false)) {
          const WbVector3 worldUp = WbWorld::instance()->worldInfo()->upVector();
          // Rotate around the camera's own position (not a picked rotation centre) so the camera
          // looks around in place — the FPS feel — and pass objectPicked=true to use the full
          // sensitivity rather than the 1/8 scaled-down "background drag" speed.
          WbRotateViewpointEvent::applyToViewpoint(delta, viewpoint->position()->value(), worldUp, true, viewpoint);
          QCursor::setPos(mapToGlobal(mFlyMouseAnchor));
          renderLater();
        }
      }
      return;
    }
  }

  // do not change cursor shape while dragging an overlay
  if (mDragOverlay) {
    mDragOverlay->apply(position);
    renderLater();
    return;
  }

  // Overlay management comes first
  if (event->buttons() == Qt::NoButton) {
    // no mouse button is pressed
    WbRenderingDevice *const renderingDevice = WbRenderingDevice::fromMousePosition(position.x(), position.y());
    if (renderingDevice && renderingDevice->overlay()) {
      bool resizeArea = false;
      int u, v;
      renderingDevice->overlay()->convertMousePositionToIndex(position.x(), position.y(), u, v, resizeArea);
      if (WbSimulationState::instance()->isPaused()) {
        WbLog::status(renderingDevice->name() + ": " + renderingDevice->pixelInfo(u, v));
        WbCamera *camera = dynamic_cast<WbCamera *>(renderingDevice);
        if (camera) {
          if (mCameraUsingRecognizedObjectsOverlay != camera)
            cleanupCameraRecognizedObjectsOverlayIfNeeded();
          mCameraUsingRecognizedObjectsOverlay = camera;
          camera->updateRecognizedObjectsOverlay((double)position.x() / width(), (double)position.y() / height(), u, v);
          refresh();
        } else
          cleanupCameraRecognizedObjectsOverlayIfNeeded();
      } else
        cleanupCameraRecognizedObjectsOverlayIfNeeded();
      if (resizeArea)
        setCursor(QCursor(Qt::SizeFDiagCursor));
      else if (renderingDevice->overlay()->isInsideCloseButton(position.x(), position.y()))
        setCursor(QCursor(Qt::ArrowCursor));
      else
        setCursor(QCursor(Qt::CrossCursor));
    } else {
      cleanupCameraRecognizedObjectsOverlayIfNeeded();
      setCursor(QCursor(Qt::ArrowCursor));
      if (mDragForce == NULL && mDragTorque == NULL)
        WbLog::status("");
    }

    mLastMouseCursor = cursor();
    return;
  }

  if (!mMouseEventInitialized)
    // return if mouse pressed event was not executed
    return;

  // At least one mouse button is pressed, so a drag event is ongoing or has to be created

  // Checks whether there is an ongoing drag event and update it in this case
  if (mDragResize) {
    mDragResize->apply(position);
    renderLater();
    return;
  }

  if (mDragTranslate) {
    mDragTranslate->apply(position);
    renderLater();
    return;
  }

  if (mDragVerticalAxisRotate) {
    mDragVerticalAxisRotate->apply(position);
    renderLater();
    return;
  }

  if (mDragRotate) {
    mDragRotate->apply(position);
    renderLater();
    return;
  }

  if (mDragKinematics) {
    mDragKinematics->apply(position);
    renderLater();
    return;
  }

  if (mDragForce && !mDragForce->isLocked()) {
    mDragForce->apply(position);
    renderLater();
    return;
  } else if (mDragTorque && !mDragTorque->isLocked()) {
    mDragTorque->apply(position);
    renderLater();
    return;
  }

  // Overlays come first
  // Drag overlay even if modifier keys are pressed
  WbRenderingDevice *const renderingDevice = WbRenderingDevice::fromMousePosition(position.x(), position.y());
  if (renderingDevice) {
    WbWrenTextureOverlay *const overlay = renderingDevice->overlay();
    if (overlay) {
      overlay->putOnTop();
      if (overlay->isInsideResizeArea(position.x(), position.y()))
        mDragOverlay = new WbDragResizeOverlayEvent(position, renderingDevice);
      else
        mDragOverlay = new WbDragTranslateOverlayEvent(position, QPoint(width(), height()), renderingDevice);
      connect(renderingDevice, &QObject::destroyed, this, &WbView3D::abortOverlayDrag);
      return;
    }
  }

  WbViewpoint *const viewpoint = mWorld->viewpoint();

  // Translate, rotate, resize events come right after overlays
  const int translateHandle = mPicker->pickedTranslateHandle(), rotateHandle = mPicker->pickedRotateHandle(),
            resizeHandle = mPicker->pickedResizeHandle();

  // Creates a new drag event according to keys (SHIFT, ALT) and buttons (LEFT, MIDDLE, RIGHT)
  const int shift = event->modifiers() & Qt::ShiftModifier;
  const int alt = event->modifiers() & Qt::AltModifier;

  int selective = !shift;
  bool resizeActive =
    WbSelection::instance()->resizeManipulatorEnabledFromSceneTree() || (event->modifiers() & Qt::ControlModifier);
  if (resizeHandle && resizeActive) {
    cleanupPhysicsDrags();

    WbBaseNode *pickedNode = WbSelection::instance()->selectedNode();
    WbGeometry *const pickedGeometry = dynamic_cast<WbGeometry *>(pickedNode);

    assert(pickedGeometry);
    if (!pickedGeometry)
      return;

    const int handleNumber = resizeHandle - 1;
    const int geometryType = pickedGeometry->nodeType();
    switch (geometryType) {
      case WB_NODE_SPHERE:
        mDragResize = new WbResizeSphereEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CYLINDER:
        if (selective)
          mDragResize = new WbResizeCylinderEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleCylinderEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CAPSULE:
        if (selective)
          mDragResize = new WbResizeCapsuleEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleCapsuleEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_BOX:
        if (selective)
          mDragResize = new WbResizeBoxEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleBoxEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_PLANE:
        if (selective)
          mDragResize = new WbResizePlaneEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescalePlaneEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_INDEXED_FACE_SET:
        if (selective)
          mDragResize = new WbResizeIndexedFaceSetEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleIndexedFaceSetEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CONE:
        if (selective)
          mDragResize = new WbResizeConeEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleConeEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_ELEVATION_GRID:
        if (selective)
          mDragResize = new WbResizeElevationGridEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new WbRescaleElevationGridEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
    }
    connect(mDragResize, &WbDragResizeHandleEvent::aborted, this, &WbView3D::abortResizeDrag);
    return;
  }

  if (translateHandle) {
    cleanupPhysicsDrags();
    int handleNumber = translateHandle - 1;
    WbBaseNode *pickedNode = WbSelection::instance()->selectedNode();
    WbSolid *const pickedSolid = dynamic_cast<WbSolid *>(pickedNode);
    if (pickedSolid)
      mDragTranslate = new WbDragTranslateAlongAxisSolidEvent(position, size(), viewpoint, handleNumber, pickedSolid);
    else {
      WbAbstractPose *pickedPose = dynamic_cast<WbAbstractPose *>(pickedNode);
      assert(pickedPose);
      mDragTranslate = new WbDragTranslateAlongAxisEvent(position, size(), viewpoint, handleNumber, pickedPose);
    }
    return;
  } else if (rotateHandle) {
    cleanupPhysicsDrags();
    const int handleNumber = rotateHandle - 1;
    WbBaseNode *pickedNode = WbSelection::instance()->selectedNode();
    WbSolid *const pickedSolid = dynamic_cast<WbSolid *>(pickedNode);
    if (pickedSolid)
      mDragRotate = new WbDragRotateAroundAxisSolidEvent(position, size(), viewpoint, handleNumber, pickedSolid);
    else {
      WbAbstractPose *pickedPose = dynamic_cast<WbAbstractPose *>(pickedNode);
      assert(pickedPose);
      mDragRotate = new WbDragRotateAroundAxisEvent(position, size(), viewpoint, handleNumber, pickedPose);
    }
    return;
  }

  // Cases 1 SHIFT + CLICK
  // - LEFT CLICK  -> move the selected solid along horizontal plane
  // - RIGHT CLICK -> rotate the selected solid around world vertical axis
  // - MID CLICK   -> lift the selected solid
  if (shift) {
    if (mDisabledUserInteractionsMap.value(WbAction::DISABLE_OBJECT_MOVE, false))
      // user interaction disabled
      return;
    selectNode(event);
    const WbSelection *const selection = WbSelection::instance();
    if (!selection->isObjectMotionAllowed())
      return;

    WbBaseNode *const selectedNode = dynamic_cast<WbBaseNode *>(selection->selectedAbstractPose());
    WbPose *const uppermostPose = WbNodeUtilities::findUppermostPose(selectedNode);
    WbSolid *const uppermostSolid = WbNodeUtilities::findUppermostSolid(selectedNode);
    Qt::MouseButtons buttons = event->buttons();
    if (buttons == Qt::MiddleButton || buttons == (Qt::LeftButton | Qt::RightButton)) {
      if (uppermostSolid) {
        if (uppermostSolid->canBeTranslated())
          mDragKinematics = new WbDragVerticalSolidEvent(position, viewpoint, uppermostSolid);
      } else if (uppermostPose->canBeTranslated())
        mDragKinematics = new WbDragVerticalEvent(position, viewpoint, uppermostPose);
    } else if (buttons == Qt::LeftButton) {
      if (uppermostSolid) {
        if (uppermostSolid->canBeTranslated())
          mDragKinematics = new WbDragHorizontalSolidEvent(position, viewpoint, uppermostSolid);
      } else if (uppermostPose->canBeTranslated())
        mDragKinematics = new WbDragHorizontalEvent(position, viewpoint, uppermostPose);
    } else if (buttons == Qt::RightButton) {
      if (uppermostSolid) {
        if (uppermostSolid->canBeRotated())
          mDragVerticalAxisRotate = new WbDragRotateAroundWorldVerticalAxisSolidEvent(position, viewpoint, uppermostSolid);
      } else if (uppermostPose->canBeRotated())
        mDragVerticalAxisRotate = new WbDragRotateAroundWorldVerticalAxisEvent(position, viewpoint, uppermostPose);
    }
  } else if (alt) {
    // Case 2: ALT and CLICK -> add a force / torque to the selected solid
    if (mDisabledUserInteractionsMap.value(WbAction::DISABLE_FORCE_AND_TORQUE, false))
      // user interaction disabled
      return;

    WbNode *node = dynamic_cast<WbNode *>(mPickedMatter);
    if (!node)
      return;
    WbSolid *selectedSolid;
    while (1) {
      selectedSolid = dynamic_cast<WbSolid *>(node);
      if (selectedSolid && selectedSolid->bodyMerger() != NULL)
        break;

      node = node->parentNode();
      if (!node || node->level() < 1)  // abort the search at the top of this node chain
        return;
    }

    Qt::MouseButtons buttons = event->buttons();
    bool forceButtonPressed = buttons == Qt::LeftButton;
#ifdef __APPLE__
    bool torqueButtonPressed =
      buttons == Qt::RightButton || (buttons == Qt::LeftButton && event->modifiers() & Qt::MetaModifier);
#else
    bool torqueButtonPressed = buttons == Qt::RightButton;
#endif
    if (torqueButtonPressed || forceButtonPressed) {
      if (mDragTorque) {
        if (mDragTorque->isLocked()) {
          delete mDragTorque;
          mDragTorque = NULL;
        }
      }

      if (mDragForce) {
        if (mDragForce->isLocked()) {
          delete mDragForce;
          mDragForce = NULL;
        }
      }

      if (!mDragTorque && torqueButtonPressed) {
        WbSelection::instance()->disableActiveManipulator();
        mDragTorque = new WbDragTorqueEvent(size(), viewpoint, selectedSolid);
        connect(mDragTorque, &WbDragTorqueEvent::aborted, this, &WbView3D::abortPhysicsDrag);
        connect(mDragTorque, &WbDragTorqueEvent::destroyed, WbSelection::instance(), &WbSelection::restoreActiveManipulator);
      } else if (!mDragForce && forceButtonPressed) {
        WbSelection::instance()->disableActiveManipulator();
        mDragForce = new WbDragForceEvent(size(), viewpoint, selectedSolid);
        connect(mDragForce, &WbDragForceEvent::aborted, this, &WbView3D::abortPhysicsDrag);
        connect(mDragForce, &WbDragForceEvent::destroyed, WbSelection::instance(), &WbSelection::restoreActiveManipulator);
      }
    }
  } else if (!mDisabledUserInteractionsMap.value(WbAction::LOCK_VIEWPOINT, false)) {
    // Case 3: CLICK only -> move the camera
    Qt::MouseButtons buttons = event->buttons();

    // For zoom and translation, we need the distance to the clicked object, if any.
    double distanceToPickPosition;
    if (mPicker->selectedId() != -1)
      distanceToPickPosition = (viewpoint->position()->value() - viewpoint->rotationCenter()).length();
    else
      distanceToPickPosition = viewpoint->position()->value().length();

    if (distanceToPickPosition < 0.001)
      distanceToPickPosition = 0.001;

    double scale = distanceToPickPosition * 2 * tan(viewpoint->fieldOfView()->value() / 2) / std::max(width(), height());

#ifdef __APPLE__
    if (buttons == Qt::RightButton || (buttons == Qt::LeftButton && event->modifiers() & Qt::MetaModifier))
#else
    if (buttons == Qt::RightButton)
#endif
      mDragKinematics = new WbTranslateViewpointEvent(position, viewpoint, scale);
    else if (buttons == Qt::MiddleButton || buttons == (Qt::LeftButton | Qt::RightButton))
      mDragKinematics = new WbZoomAndRotateViewpointEvent(position, viewpoint, 5 * scale);
    else if (buttons == Qt::LeftButton)
      mDragKinematics = new WbRotateViewpointEvent(position, viewpoint, mPicker->selectedId() != -1);
  }
}

void WbView3D::mouseDoubleClick(QMouseEvent *event) {
  if (!mWorld)
    return;

  const QPoint &mousePosition = event->pos();

  // Overlays come first
  // open external window
  WbRenderingDevice *const renderingDevice = WbRenderingDevice::fromMousePosition(mousePosition.x(), mousePosition.y());
  if (renderingDevice) {
    WbRenderingDeviceWindowFactory::instance()->showWindowForDevice(renderingDevice);
    return;
  }

  if (mDisabledUserInteractionsMap.value(WbAction::DISABLE_SELECTION, false))
    return;

  const bool picked = mPicker->pick(mousePosition.x(), mousePosition.y());
  if (picked) {
    const int id = mPicker->selectedId();
    if (id == -1)
      return;

    emit mouseDoubleClicked(event);

    WbNode *node = WbNode::findNode(id);
    WbRobot *pickedRobot = dynamic_cast<WbRobot *>(node);
    if (pickedRobot == NULL && node != NULL)
      pickedRobot = WbNodeUtilities::findRobotAncestor(node);
    if (pickedRobot)
      mPickedMatter = pickedRobot;
    else
      mPickedMatter = WbNodeUtilities::findUpperMatter(node);
  }
}

bool WbView3D::isContextMenuShortcut(const QMouseEvent *event) {
#ifdef __APPLE__
  return (event->button() == Qt::RightButton && event->modifiers() == Qt::NoModifier) ||
         (event->button() == Qt::LeftButton && event->modifiers() & Qt::MetaModifier);
#else
  return event->button() == Qt::RightButton && (event->modifiers() == Qt::NoModifier);
#endif
}

void WbView3D::mouseReleaseEvent(QMouseEvent *event) {
  WbWrenWindow::mouseReleaseEvent(event);

  mLastButtonState = event->buttons();

  if (mFlyMouseLook && event->button() == Qt::LeftButton)
    exitFlyMouseLook();

  if (!mMouseEventInitialized)
    // mouse press event handled by another widget
    return;

  updateMousesPosition(true, false);

  setCursor(mLastMouseCursor);

  const bool wasNotInAnEvent = !mDragOverlay && !mDragKinematics && !mDragResize && !mDragTranslate &&
                               !mDragVerticalAxisRotate && !mDragRotate && !mDragForce && !mDragTorque && !mTouchSensor;

  delete mDragOverlay;
  mDragOverlay = NULL;

  delete mDragKinematics;
  mDragKinematics = NULL;

  if (mDragResize) {
    mDragResize->addActionInUndoStack();
    delete mDragResize;
    mDragResize = NULL;
    if (mResizeHandlesDisabled)
      WbSelection::instance()->showResizeManipulatorFromView3D(false);
  }

  if (mDragTranslate) {
    delete mDragTranslate;
    mDragTranslate = NULL;
  }

  if (mDragVerticalAxisRotate) {
    delete mDragVerticalAxisRotate;
    mDragVerticalAxisRotate = NULL;
  }

  if (mDragRotate) {
    delete mDragRotate;
    mDragRotate = NULL;
  }

  const WbSimulationState *const sim = WbSimulationState::instance();
  if (sim->isPaused()) {
    if (mDragForce && !mDragForce->isLocked())
      mDragForce->lock();
    if (mDragTorque && !mDragTorque->isLocked())
      mDragTorque->lock();
  } else {
    delete mDragForce;
    mDragForce = NULL;
    delete mDragTorque;
    mDragTorque = NULL;
  }
  if (mTouchSensor) {
    mTouchSensor->setGuiTouch(false);
    mTouchSensor = NULL;
  }

  renderLater();

  if (wasNotInAnEvent)
    selectNode(event);
  else if (mMousePressTimer) {  // test if we did a quick button press and release, possibly moving only slightly the mouse
    const int delay = mMousePressTimer->elapsed();
    if (delay < QApplication::doubleClickInterval()) {  // the mouse button was released quickly after being pressed
      const QPoint diff = mMousePressPosition - event->pos();
      if (diff.manhattanLength() < 20)  // the mouse was moved by less than 20 pixels (determined empirically)
        selectNode(event);
    }
  }

  mPickedMatter = NULL;
  mMouseEventInitialized = false;
}

void WbView3D::handleModifierKey(QKeyEvent *event, bool pressed) {
  if (event->key() == Qt::Key_Control)
    enableResizeManipulator(pressed);
  else if (event->key() == Qt::Key_Shift)
    WbSelection::instance()->setUniformConstraintForResizeHandles(pressed);
}

void WbView3D::keyPressEvent(QKeyEvent *event) {
  // handle event in parent class
  if (event->key() == Qt::Key_Escape ||
      (event->modifiers() == Qt::CTRL && event->key() >= Qt::Key_0 && event->key() <= Qt::Key_4)) {
    QWindow::keyPressEvent(event);
    return;
  }

  // Numpad view-snap shortcuts (Blender convention so users coming from a DCC tool feel at home):
  //   Numpad 7 = top, Numpad 1 = north, Numpad 3 = east; hold Ctrl to flip to the opposite face.
  // The TOP_VIEW/etc. actions are already wired through WbActionManager -> WbViewpoint, so we just
  // trigger them. Swallow the event afterwards so it isn't forwarded to robot controllers.
  if (mWorld && (event->modifiers() & Qt::KeypadModifier) && !event->isAutoRepeat()) {
    const bool ctrl = event->modifiers() & Qt::ControlModifier;
    QAction *snap = NULL;
    WbActionManager *const am = WbActionManager::instance();
    switch (event->key()) {
      case Qt::Key_7:
        snap = am->action(ctrl ? WbAction::BOTTOM_VIEW : WbAction::TOP_VIEW);
        break;
      case Qt::Key_1:
        snap = am->action(ctrl ? WbAction::SOUTH_VIEW : WbAction::NORTH_VIEW);
        break;
      case Qt::Key_3:
        snap = am->action(ctrl ? WbAction::WEST_VIEW : WbAction::EAST_VIEW);
        break;
      default:
        break;
    }
    if (snap) {
      snap->trigger();
      QWindow::keyPressEvent(event);
      return;
    }
  }

  // F = toggle "follow selected object" (tracking shot, preserves the camera's current offset
  // from the target). Lets the user click a robot, press F, and have the camera ride along.
  // Press F again to stop. Existing F5 menu shortcut is left intact.
  if (mWorld && event->key() == Qt::Key_F && !event->isAutoRepeat() &&
      (event->modifiers() & ~Qt::KeypadModifier) == Qt::NoModifier) {
    WbViewpoint *const viewpoint = mWorld->viewpoint();
    if (viewpoint && viewpoint->followedSolid()) {
      followNone(true);
      WbActionManager::instance()->action(WbAction::FOLLOW_NONE)->setChecked(true);
      WbLog::status(tr("Camera follow stopped."));
    } else if (viewpoint) {
      WbSolid *const selectedSolid = WbSelection::instance()->selectedSolid();
      if (selectedSolid) {
        followTracking(true);
        WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING)->setChecked(true);
        WbLog::status(tr("Camera now tracking: %1").arg(selectedSolid->name()));
      } else {
        WbLog::status(tr("Click on an object to select it, then press F to follow."));
      }
    }
    QWindow::keyPressEvent(event);
    return;
  }

  // WASD/QE free-fly camera. Triggers only on bare keys or Shift+key (sprint), so Ctrl/Alt/Meta
  // shortcuts (Ctrl+S etc.) are unaffected. Robot key forwarding below is still done so that
  // controllers using WASD keep working — users who don't want camera drift can lock the viewpoint.
  if (mWorld && !event->isAutoRepeat() && isFlyKey(event->key())) {
    const Qt::KeyboardModifiers nonShift = event->modifiers() & ~Qt::ShiftModifier;
    const WbViewpoint *const viewpoint = mWorld->viewpoint();
    if (nonShift == Qt::NoModifier && viewpoint && !viewpoint->isLocked() &&
        !mDisabledUserInteractionsMap.value(WbAction::LOCK_VIEWPOINT, false)) {
      mFlyKeys.insert(event->key());
      if (!mFlyTimer->isActive()) {
        mFlyLastTickMs = QDateTime::currentMSecsSinceEpoch();
        mFlyTimer->start();
      }
      // If the user is already holding the left mouse button, switch to mouselook immediately
      // so they don't have to release-and-redrag to start looking around.
      if (QApplication::mouseButtons() & Qt::LeftButton)
        enterFlyMouseLook();
    }
  }

  // pass key event to robots if appropriate
  const int modifiers = (((event->modifiers() & Qt::SHIFT) == 0) ? 0 : WbRobot::mapSpecialKey(Qt::SHIFT)) +
#ifdef __APPLE__
                        (((event->modifiers() & Qt::META) == 0) ? 0 : WbRobot::mapSpecialKey(Qt::CTRL)) +
#else
                        (((event->modifiers() & Qt::CTRL) == 0) ? 0 : WbRobot::mapSpecialKey(Qt::CTRL)) +
#endif
                        (((event->modifiers() & Qt::ALT) == 0) ? 0 : WbRobot::mapSpecialKey(Qt::ALT));

  // cppcheck-suppress constVariablePointer
  WbRobot *const currentRobot = getCurrentRobot();
  QList<WbRobot *> robotList;
  if (currentRobot)
    robotList.append(currentRobot);
  else
    robotList = mWorld->robots();

  const int key = event->key();
  if (key != Qt::Key_Control && key != Qt::Key_Meta && key != Qt::Key_Shift && key != Qt::Key_Alt) {
    foreach (WbRobot *robot, robotList)
      robot->keyPressed(key, modifiers);
  }
  handleModifierKey(event, true);
  QWindow::keyPressEvent(event);
}

void WbView3D::keyReleaseEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Shift)
    cleanupWheel();

  if (!event->isAutoRepeat() && isFlyKey(event->key())) {
    mFlyKeys.remove(event->key());
    if (mFlyKeys.isEmpty() && mFlyTimer->isActive())
      mFlyTimer->stop();
  }

  // pass key event to robots
  if (mWorld) {
    // cppcheck-suppress constVariablePointer
    WbRobot *const currentRobot = getCurrentRobot();
    QList<WbRobot *> robotList;
    if (currentRobot)
      robotList.append(currentRobot);
    else
      robotList = mWorld->robots();

    const int key = event->key();
    if (key != Qt::Key_Control && key != Qt::Key_Meta && key != Qt::Key_Shift && key != Qt::Key_Alt) {
      foreach (WbRobot *const robot, robotList)
        robot->keyReleased(key);
    }
  }
  handleModifierKey(event, false);
  QWindow::keyReleaseEvent(event);
}

bool WbView3D::isFlyKey(int key) {
  return key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_Q ||
         key == Qt::Key_E;
}

void WbView3D::stopFly() {
  if (!mFlyKeys.isEmpty())
    mFlyKeys.clear();
  if (mFlyTimer && mFlyTimer->isActive())
    mFlyTimer->stop();
  exitFlyMouseLook();
}

void WbView3D::enterFlyMouseLook() {
  if (mFlyMouseLook || !mWorld)
    return;
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  if (!viewpoint || viewpoint->isLocked() ||
      mDisabledUserInteractionsMap.value(WbAction::LOCK_VIEWPOINT, false))
    return;
  // Cancel any pan/orbit drag the right-mouse press might have started, otherwise the next
  // mouseMoveEvent would still apply the pan before we get a chance to look around.
  delete mDragKinematics;
  mDragKinematics = NULL;
  mFlyMouseLook = true;
  mFlyPrevCursor = cursor();
  setCursor(QCursor(Qt::BlankCursor));
  // Anchor the cursor at the window centre and keep warping it back there each move so the user
  // can pan the view indefinitely in any direction without the pointer hitting the window edge.
  mFlyMouseAnchor = QPoint(width() / 2, height() / 2);
  QCursor::setPos(mapToGlobal(mFlyMouseAnchor));
}

void WbView3D::exitFlyMouseLook() {
  if (!mFlyMouseLook)
    return;
  mFlyMouseLook = false;
  setCursor(mFlyPrevCursor);
}

void WbView3D::updateFlyCamera() {
  if (!mWorld) {
    stopFly();
    return;
  }
  WbViewpoint *const viewpoint = mWorld->viewpoint();
  if (!viewpoint || viewpoint->isLocked() ||
      mDisabledUserInteractionsMap.value(WbAction::LOCK_VIEWPOINT, false)) {
    stopFly();
    return;
  }

  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  double dt = (now - mFlyLastTickMs) / 1000.0;
  mFlyLastTickMs = now;
  // Clamp dt: a paused/sleeping window can produce huge gaps that would teleport the camera.
  if (dt <= 0.0)
    return;
  if (dt > 0.1)
    dt = 0.1;

  // Build a movement direction in world space from the active keys.
  const WbRotation &orientation = viewpoint->orientation()->value();
  const WbVector3 forward = orientation.direction();  // camera look direction
  const WbVector3 right = orientation.right();
  const WbVector3 worldUp = WbWorld::instance()->worldInfo()->upVector();
  WbVector3 move(0.0, 0.0, 0.0);
  if (mFlyKeys.contains(Qt::Key_W))
    move += forward;
  if (mFlyKeys.contains(Qt::Key_S))
    move -= forward;
  if (mFlyKeys.contains(Qt::Key_D))
    move -= right;
  if (mFlyKeys.contains(Qt::Key_A))
    move += right;
  if (mFlyKeys.contains(Qt::Key_E))
    move += worldUp;
  if (mFlyKeys.contains(Qt::Key_Q))
    move -= worldUp;
  if (move.isNull())
    return;
  move.normalize();

  // Adaptive base speed: scales with how far the camera sits from its rotation center, so the
  // same key press feels right whether you're inspecting a 10 cm gripper or an outdoor terrain.
  // Floored so motion still works when the camera is sitting on its rotation center.
  double distance = (viewpoint->position()->value() - viewpoint->rotationCenter()).length();
  if (distance < 0.5)
    distance = 0.5;
  if (distance > 200.0)
    distance = 200.0;
  double speed = 0.5 * distance;  // m/s at 1x
  const Qt::KeyboardModifiers mods = QApplication::queryKeyboardModifiers();
  if (mods & Qt::ShiftModifier)
    speed *= 3.0;

  WbSFVector3 *const position = viewpoint->position();
  position->setValue(position->value() + move * (speed * dt));
  mWorld->setModified();
  renderLater();
}

void WbView3D::enableResizeManipulator(bool enabled) {
  if (enabled && WbSelection::instance()->showResizeManipulatorFromView3D(true))
    mResizeHandlesDisabled = false;
  else {
    if (mDragResize)
      mResizeHandlesDisabled = true;
    else
      WbSelection::instance()->showResizeManipulatorFromView3D(false);
  }
}

WbRobot *WbView3D::getCurrentRobot() const {
  if (!WbSelection::instance() || !WbSelection::instance()->selectedSolid())
    return NULL;

  WbRobot *const robot = WbSelection::instance()->selectedSolid()->robot();
  if (robot)
    return robot;

  const QList<WbRobot *> &robotList = mWorld->robots();
  if (robotList.size() == 1)
    return robotList.first();

  return NULL;
}

void WbView3D::wheelEvent(QWheelEvent *event) {
  if (!mWorld)
    return;

#ifndef __APPLE__  // bug in qt on Mac: -> QWheelEvent->orientation() is wrong when SHIFT + MOUSE_WHEEL_VERTICAL_SCROLL
  // Some mouse wheels can be scrolled horizontally
  if (event->angleDelta().x() != 0)
    return;
#endif

  WbViewpoint *const viewpoint = mWorld->viewpoint();
  if (event->modifiers() & Qt::ShiftModifier) {
    if (mDisabledUserInteractionsMap.value(WbAction::DISABLE_OBJECT_MOVE, false))
      return;
    if (mWheel) {
      mWheel->apply(event->angleDelta().y());
      renderLater();
      return;
    }
    // SHIFT and WHEEL MOUSE -> lift the selected solid in the 3D View
    WbBaseNode *const selectedNode = dynamic_cast<WbBaseNode *>(WbSelection::instance()->selectedAbstractPose());
    WbSolid *const uppermostSolid = WbNodeUtilities::findUppermostSolid(selectedNode);
    if (!uppermostSolid || uppermostSolid->isLocked() || !uppermostSolid->canBeTranslated())
      return;
    mWheel = new WbWheelLiftSolidEvent(viewpoint, uppermostSolid);
    mWheel->apply(event->angleDelta().y());
    renderLater();
  } else if (!mDisabledUserInteractionsMap.value(WbAction::LOCK_VIEWPOINT, false)) {
    // WHEEL MOUSE only -> zoom
    if (mProjectionMode == WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC) {
      if (event->angleDelta().y() > 0)
        viewpoint->decOrthographicViewHeight();
      else
        viewpoint->incOrthographicViewHeight();
    }

    double distanceToPickPosition;
    const QPoint mousePosition = mapFromGlobal(QCursor::pos());
    if (mousePosition.x() < 0 || mousePosition.y() < 0 || mousePosition.x() >= width() || mousePosition.y() >= height())
      distanceToPickPosition = viewpoint->position()->value().length();
    else {
      if (mPicker->selectedId() != -1)
        distanceToPickPosition = (viewpoint->position()->value() - viewpoint->rotationCenter()).length();
      else
        distanceToPickPosition = viewpoint->position()->value().length();
      if (distanceToPickPosition < 0.001)
        distanceToPickPosition = 0.001;
    }

    const double scaleFactor = 0.1 * (event->angleDelta().y() < 0.0 ? -1 : 1) * distanceToPickPosition;
    const WbVector3 zDisplacement(scaleFactor * viewpoint->orientation()->value().direction());
    WbSFVector3 *const position = viewpoint->position();
    position->setValue(position->value() + zDisplacement);
    if (!zDisplacement.isNull())
      mWorld->setModified();
    renderLater();
  }
}

// Cleanup methods

void WbView3D::cleanupEvents() {
  cleanupWheel();
  cleanupDrags();
  stopFly();
}

void WbView3D::cleanupOptionalRendering() {
  delete mContactPointsRepresentation;
  mContactPointsRepresentation = NULL;
}

void WbView3D::cleanupWheel() {
  delete mWheel;
  mWheel = NULL;
}

void WbView3D::cleanupCameraRecognizedObjectsOverlayIfNeeded() {
  if (mCameraUsingRecognizedObjectsOverlay) {
    mCameraUsingRecognizedObjectsOverlay->clearRecognizedObjectsOverlay();
    mCameraUsingRecognizedObjectsOverlay = NULL;
    refresh();
  }
}

void WbView3D::cleanupDrags() {
  delete mDragOverlay;
  mDragOverlay = NULL;

  delete mDragKinematics;
  mDragKinematics = NULL;

  delete mDragResize;
  mDragResize = NULL;

  delete mDragTranslate;
  mDragTranslate = NULL;

  delete mDragVerticalAxisRotate;
  mDragVerticalAxisRotate = NULL;

  delete mDragRotate;
  mDragRotate = NULL;

  cleanupPhysicsDrags();
}

void WbView3D::abortPhysicsDrag() {
  cleanupPhysicsDrags();
  WbSelection::instance()->selectPoseFromView3D(NULL);
  WbLog::warning(tr("Solid out of world numeric bounds, mouse drag aborted"));
}

void WbView3D::abortResizeDrag() {
  delete mDragResize;
  mDragResize = NULL;
  WbSelection::instance()->selectPoseFromView3D(NULL);
  WbLog::warning(tr("The dimensions of the resized object exceeds world numeric bounds, mouse drag aborted"));
  if (mResizeHandlesDisabled)
    WbSelection::instance()->showResizeManipulatorFromView3D(false);
}

void WbView3D::abortOverlayDrag() {
  delete mDragOverlay;
  mDragOverlay = NULL;
}

void WbView3D::cleanupPhysicsDrags() {
  delete mDragForce;
  mDragForce = NULL;

  delete mDragTorque;
  mDragTorque = NULL;
}

void WbView3D::cleanupPickers() {
  delete mPicker;
  delete mControllerPicker;
  mPicker = NULL;
  mControllerPicker = NULL;
  mPickedMatter = NULL;
}

void WbView3D::unleashAndClean() {
  if (mDragForce) {
    mDragForce->applyToOde();
    delete mDragForce;
    mDragForce = NULL;
  }

  if (mDragTorque) {
    mDragTorque->applyToOde();
    delete mDragTorque;
    mDragTorque = NULL;
  }

  if (mDragForce || mDragTorque)
    renderLater();
}

void WbView3D::unleashPhysicsDrags() {
  const WbSimulationState *const sim = WbSimulationState::instance();
  if (sim->isPaused())
    return;

  unleashAndClean();
}
// Fast mode related methods

void WbView3D::rescaleFastModePanel() {
  mDisabledRenderingOverlay->adjustSize();
}

void WbView3D::showBlackRenderingOverlay() {
  if (!mWorld || mDisabledRenderingOverlay->isVisible())
    return;

  disconnect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this, &WbView3D::refresh);

  rescaleFastModePanel();
  mDisabledRenderingOverlay->setVisible(true);

  mParentWidget->setEnabled(false);
  renderLater();

  WbRenderingDeviceWindowFactory::instance()->setWindowsEnabled(false);

  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::hideBlackRenderingOverlay() {
  if (!mWorld || !mDisabledRenderingOverlay->isVisible())
    return;

  connect(WbSimulationState::instance(), &WbSimulationState::controllerReadRequestsCompleted, this, &WbView3D::refresh,
          Qt::UniqueConnection);

  mDisabledRenderingOverlay->setVisible(false);

  mParentWidget->setEnabled(true);
  renderLater();

  WbRenderingDeviceWindowFactory::instance()->setWindowsEnabled(true);

  updateVirtualRealityHeadsetOverlay();
}

void WbView3D::cleanupFullScreenOverlay() {
  delete mDisabledRenderingOverlay;
  mDisabledRenderingOverlay = NULL;
  delete mVirtualRealityHeadsetOverlay;
  mVirtualRealityHeadsetOverlay = NULL;
  delete mLoadingWorldOverlay;
  mLoadingWorldOverlay = NULL;
}

void WbView3D::updateVirtualRealityHeadsetOverlay() {
  if (!mWorld || !mVirtualRealityHeadsetOverlay)
    return;

  if (mDisabledRenderingOverlay->isVisible()) {
    mVirtualRealityHeadsetOverlay->setVisible(false);
    return;
  }

#ifdef _WIN32
  if (WbVirtualRealityHeadset::isInUse()) {
    mVirtualRealityHeadsetOverlay->setVisible(true);
    mVirtualRealityHeadsetOverlay->setExternalTexture(WbVirtualRealityHeadset::instance()->visibleTexture());
    mParentWidget->setEnabled(false);
  } else {
#endif
    mVirtualRealityHeadsetOverlay->setVisible(false);
    mParentWidget->setEnabled(true);
#ifdef _WIN32
  }
#endif

  renderLater();
}

void WbView3D::handleWorldModificationFromSupervisor() {
  // even if the simulation is running in no-rendering mode the pending updates need to be executed in order to process
  // supervisor deletions, or Webots might run out of memory
  if (!WbSimulationState::instance()->isRendering()) {
    WbWrenOpenGlContext::makeWrenCurrent();
    wr_scene_apply_pending_updates(wr_scene_get_instance());
    WbWrenOpenGlContext::doneWren();
  }

  const WbSimulationState *const sim = WbSimulationState::instance();
  // refresh only if simulation is paused or stepped
  if (sim->isPaused())
    refresh();
}
