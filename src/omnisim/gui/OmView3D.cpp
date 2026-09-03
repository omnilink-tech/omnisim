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

#include "OmView3D.hpp"

#include "OmAbstractDragEvent.hpp"
#include "OmAbstractPose.hpp"
#include "OmActionManager.hpp"
#include "OmBaseNode.hpp"
#include "OmBox.hpp"
#include "OmCamera.hpp"
#include "OmCapsule.hpp"
#include "OmCone.hpp"
#include "OmContextMenuGenerator.hpp"
#include "OmCylinder.hpp"
#include "OmBackground.hpp"
#include "OmDirectionalLight.hpp"
#include "OmPointLight.hpp"
#include "OmSpotLight.hpp"
#include "OmFog.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmRgb.hpp"
#include "OmDragOverlayEvent.hpp"
#include "OmDragPoseEvent.hpp"
#include "OmDragResizeEvent.hpp"
#include "OmDragScaleEvent.hpp"
#include "OmDragSolidEvent.hpp"
#include "OmDragViewpointEvent.hpp"
#include "OmElevationGrid.hpp"
#include "OmGroup.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmLog.hpp"
#include "OmMultimediaStreamingServer.hpp"
#include "OmMatter.hpp"
#include "OmMessageBox.hpp"
#include "OmMouse.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPerformanceLog.hpp"
#include "OmPerspective.hpp"
#include "OmPlane.hpp"
#include "OmPose.hpp"
#include "OmPreferences.hpp"
#include "OmRenderingDevice.hpp"
#include "OmRenderingDeviceWindowFactory.hpp"
#include "OmRobot.hpp"
#include "OmSelection.hpp"
#include "OmSimulationState.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "../render/OmniLight.hpp"
#include "OmSphere.hpp"
#include "OmStandardPaths.hpp"
#include "OmSupervisorUtilities.hpp"
#include "OmRenderBackend.hpp"  // R4 3c-B: per-Viewpoint backend dispatch for the main view
// R4 3c-B: wgpu main-view render path (offscreen wgpu render → GL blit into this WREN window).
#include "OmMatrix4.hpp"
#include "OmRotation.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector3.hpp"
#include "OmVector3.hpp"
#include "OmVulkanBackend.hpp"
#include "OmWgpuGlBlit.hpp"
#include "OmWgpuMeshCache.hpp"
#include "OmWgpuRenderTarget.hpp"
#include "OmWgpuSurface.hpp"
#ifdef _WIN32
#  include <windows.h>  // GetModuleHandleW for the wgpu present-surface HINSTANCE
#endif
#include "OmWgpuSceneRenderer.hpp"
#include "OmWgpuTextureCache.hpp"
// W4a (WREN retirement): the geometry-overlay collectors live with the wgpu pane; the main
// view calls the same static entry points so there is exactly one implementation.
#include "OmDragArrowLines.hpp"
#include "OmGizmoLines.hpp"
#include "OmHudOverlay.hpp"
#include "OmWgpuView.hpp"
#include "OmWorld.hpp"
#include "OmSysInfo.hpp"
#include "OmTrack.hpp"  // P2 belt-advance telemetry (wgpuFirstBeltProbe)
#include "OmTouchSensor.hpp"
#include "OmTranslateRotateManipulator.hpp"
#include "OmVersion.hpp"
#include "OmVideoRecorder.hpp"
#include "OmViewpoint.hpp"
#include "OmVisualBoundingSphere.hpp"
#include "OmWheelEvent.hpp"
#include "OmWorldInfo.hpp"
#include "OmWrenLabelOverlay.hpp"
#include "OmWrenOpenGlContext.hpp"
#include "OmScenePicker.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmWrenTextureOverlay.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <map>
#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtGui/QAction>

#include <algorithm>  // std::min - the per-frame deformable draw-tail trim
#include <cmath>
#include <cstdio>
#include <QtGui/QImage>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QScreen>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMenu>


int OmView3D::cView3DNumber = 0;

OmView3D::OmView3D() :
  OmGlWindow(),
  mParentWidget(NULL),
  mMousePressTimer(NULL),
  mAspectRatio(1.0),
  mDisabledRenderingVisible(false),
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
  QDir::addSearchPath("gl", OmStandardPaths::resourcesPath() + "wren");

  mRenderPacingClock.start();
  setObjectName("View3D");

  OmWrenRenderingContext::setWrenRenderingContext(width(), height());
  mWrenRenderingContext = OmWrenRenderingContext::instance();

  OmActionManager *actionManager = OmActionManager::instance();
  // render after each simulation step and when simulation mode changed
  connect(OmSimulationState::instance(), &OmSimulationState::controllerReadRequestsCompleted, this, &OmView3D::refresh,
          Qt::UniqueConnection);
  connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this, &OmView3D::refresh, Qt::UniqueConnection);
  connect(OmSimulationState::instance(), &OmSimulationState::renderingStateChanged, this, &OmView3D::refresh,
          Qt::UniqueConnection);
  // clean up pending drag-force / drag-torque when simulation restarts
  connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this, &OmView3D::unleashPhysicsDrags);
  // update mouses if required
  connect(OmSimulationState::instance(), SIGNAL(physicsStepStarted()), this, SLOT(updateMousesPosition()));
  // viewpoint
  connect(actionManager->action(OmAction::FOLLOW_NONE), &QAction::triggered, this, &OmView3D::followNone);
  connect(actionManager->action(OmAction::FOLLOW_TRACKING), &QAction::triggered, this, &OmView3D::followTracking);
  connect(actionManager->action(OmAction::FOLLOW_MOUNTED), &QAction::triggered, this, &OmView3D::followMounted);
  connect(actionManager->action(OmAction::FOLLOW_PAN_AND_TILT), &QAction::triggered, this, &OmView3D::followPanAndTilt);
  connect(actionManager->action(OmAction::RESTORE_VIEWPOINT), &QAction::triggered, this, &OmView3D::restoreViewpoint);
  // signal the simulation state about a rendering
  connect(actionManager->action(OmAction::ORTHOGRAPHIC_PROJECTION), &QAction::triggered, this,
          &OmView3D::setOrthographicProjection);
  connect(actionManager->action(OmAction::PERSPECTIVE_PROJECTION), &QAction::triggered, this,
          &OmView3D::setPerspectiveProjection);
  connect(actionManager->action(OmAction::PLAIN_RENDERING), &QAction::triggered, this, &OmView3D::setPlain);
  connect(actionManager->action(OmAction::WIREFRAME_RENDERING), &QAction::triggered, this, &OmView3D::setWireframe);
  connect(actionManager->action(OmAction::LOCK_VIEWPOINT), &QAction::triggered, this, &OmView3D::setViewPointLocked);
  connect(actionManager->action(OmAction::DISABLE_SELECTION), &QAction::triggered, this, &OmView3D::setSelectionDisabled);
  connect(actionManager->action(OmAction::DISABLE_3D_VIEW_CONTEXT_MENU), &QAction::triggered, this,
          &OmView3D::setContextMenuDisabled);
  connect(actionManager->action(OmAction::DISABLE_OBJECT_MOVE), &QAction::triggered, this, &OmView3D::disableObjectMove);
  connect(actionManager->action(OmAction::DISABLE_FORCE_AND_TORQUE), &QAction::triggered, this,
          &OmView3D::disableApplyForceAndTorque);
  // optional renderings
  connect(actionManager->action(OmAction::COORDINATE_SYSTEM), &QAction::toggled, this, &OmView3D::setShowCoordinateSystem);
  connect(actionManager->action(OmAction::BOUNDING_OBJECT), &QAction::toggled, this, &OmView3D::setShowBoundingObjects);
  connect(actionManager->action(OmAction::NORMALS), &QAction::triggered, this, &OmView3D::setShowNormals);
  connect(actionManager->action(OmAction::CONTACT_POINTS), &QAction::toggled, this, &OmView3D::setShowContactPoints);
  connect(actionManager->action(OmAction::CONNECTOR_AXES), &QAction::toggled, this, &OmView3D::setShowConnectorAxes);
  connect(actionManager->action(OmAction::JOINT_AXES), &QAction::toggled, this, &OmView3D::setShowJointAxes);
  connect(actionManager->action(OmAction::RANGE_FINDER_FRUSTUMS), &QAction::toggled, this,
          &OmView3D::setShowRangeFinderFrustums);
  connect(actionManager->action(OmAction::LIDAR_RAYS_PATH), &QAction::toggled, this, &OmView3D::setShowLidarRaysPaths);
  connect(actionManager->action(OmAction::LIDAR_POINT_CLOUD), &QAction::toggled, this, &OmView3D::setShowLidarPointClouds);
  connect(actionManager->action(OmAction::CAMERA_FRUSTUM), &QAction::toggled, this, &OmView3D::setShowCameraFrustums);
  connect(actionManager->action(OmAction::DISTANCE_SENSOR_RAYS), &QAction::toggled, this, &OmView3D::setShowDistanceSensorRays);
  connect(actionManager->action(OmAction::LIGHT_SENSOR_RAYS), &QAction::toggled, this, &OmView3D::setShowLightSensorRays);
  connect(actionManager->action(OmAction::LIGHT_POSITIONS), &QAction::toggled, this, &OmView3D::setShowLightsPositions);
  connect(actionManager->action(OmAction::CENTER_OF_BUOYANCY), &QAction::triggered, this, &OmView3D::showCenterOfBuoyancy);
  connect(actionManager->action(OmAction::PEN_PAINTING_RAYS), &QAction::toggled, this, &OmView3D::setShowPenPaintingRays);
  connect(actionManager->action(OmAction::CENTER_OF_MASS), &QAction::triggered, this, &OmView3D::showCenterOfMass);
  connect(actionManager->action(OmAction::SUPPORT_POLYGON), &QAction::triggered, this, &OmView3D::showSupportPolygon);
  connect(actionManager->action(OmAction::SKIN_SKELETON), &QAction::triggered, this, &OmView3D::setShowSkeletonAction);
  connect(actionManager->action(OmAction::RADAR_FRUSTUMS), &QAction::toggled, this, &OmView3D::setShowRadarFrustums);
  connect(actionManager->action(OmAction::PHYSICS_CLUSTERS), &QAction::triggered, this,
          &OmView3D::setShowPhysicsClustersAction);
  connect(actionManager->action(OmAction::BOUNDING_SPHERE), &QAction::triggered, this, &OmView3D::setShowBoundingSphereAction);
  const OmPreferences *const prefs = OmPreferences::instance();
  actionManager->action(OmAction::HIDE_ALL_CAMERA_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllCameraOverlays", false).toBool());
  connect(actionManager->action(OmAction::HIDE_ALL_CAMERA_OVERLAYS), &QAction::toggled, this,
          &OmView3D::setHideAllCameraOverlays);
  actionManager->action(OmAction::HIDE_ALL_RANGE_FINDER_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllRangeFinderOverlays", false).toBool());
  connect(actionManager->action(OmAction::HIDE_ALL_RANGE_FINDER_OVERLAYS), &QAction::toggled, this,
          &OmView3D::setHideAllRangeFinderOverlays);
  actionManager->action(OmAction::HIDE_ALL_DISPLAY_OVERLAYS)
    ->setChecked(prefs->value("View3d/hideAllDisplayOverlays", false).toBool());
  connect(actionManager->action(OmAction::HIDE_ALL_DISPLAY_OVERLAYS), &QAction::toggled, this,
          &OmView3D::setHideAllDisplayOverlays);
  // enable/disable shadows when preferences change
  connect(OmPreferences::instance(), &OmPreferences::changedByUser, this, &OmView3D::updateShadowState);

  // WASD free-fly camera: 60 Hz tick that advances the viewpoint while any fly key is held
  mFlyTimer = new QTimer(this);
  mFlyTimer->setInterval(16);
  connect(mFlyTimer, &QTimer::timeout, this, &OmView3D::updateFlyCamera);
}

void OmView3D::setPerspectiveProjection() {
  setProjectionMode(OmWrenRenderingContext::PM_PERSPECTIVE, true, true);
}

void OmView3D::setOrthographicProjection() {
  setProjectionMode(OmWrenRenderingContext::PM_ORTHOGRAPHIC, true, true);
}

void OmView3D::setPlain() {
  setRenderingMode(OmWrenRenderingContext::RM_PLAIN, true);
}

void OmView3D::setWireframe() {
  setRenderingMode(OmWrenRenderingContext::RM_WIREFRAME, true);
}

void OmView3D::onSelectionChanged(OmAbstractPose *selectedPose) {
  assert(mWorld);

  if (mWorld->isCleaning())
    return;

  OmSolid *const selectedSolid = dynamic_cast<OmSolid *>(selectedPose);
  const OmViewpoint *const viewpoint = mWorld->viewpoint();

  if (selectedSolid) {
    setCheckedShowSupportPolygonAction(selectedSolid);
    setCheckedShowCenterOfMassAction(selectedSolid);
    setCheckedShowCenterOfBuoyancyAction(selectedSolid);
    setCheckedFollowObjectAction(selectedSolid);
    selectedSolid->updateTranslateRotateHandlesSize();
    OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setEnabled(true);
  } else {
    OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setEnabled(viewpoint->followType() != OmViewpoint::FOLLOW_NONE);
    OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT)->setChecked(false);
    OmActionManager::instance()->action(OmAction::SUPPORT_POLYGON)->setChecked(false);
    OmActionManager::instance()->action(OmAction::CENTER_OF_MASS)->setChecked(false);
    OmActionManager::instance()->action(OmAction::CENTER_OF_BUOYANCY)->setChecked(false);
  }

  bool enable = selectedSolid != NULL;
  OmActionManager::instance()->action(OmAction::CENTER_OF_BUOYANCY)->setEnabled(enable);
  OmActionManager::instance()->action(OmAction::CENTER_OF_MASS)->setEnabled(enable);
  OmActionManager::instance()->action(OmAction::SUPPORT_POLYGON)->setEnabled(enable);
  OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setEnabled(enable);
  OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED)->setEnabled(enable);
  OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT)->setEnabled(enable);
  enable = enable && selectedSolid == viewpoint->followedSolid();
  OmActionManager::instance()
    ->action(OmAction::FOLLOW_NONE)
    ->setChecked(enable && viewpoint->followType() == OmViewpoint::FOLLOW_NONE);
  OmActionManager::instance()
    ->action(OmAction::FOLLOW_TRACKING)
    ->setChecked(enable && viewpoint->followType() == OmViewpoint::FOLLOW_TRACKING);
  OmActionManager::instance()
    ->action(OmAction::FOLLOW_MOUNTED)
    ->setChecked(enable && viewpoint->followType() == OmViewpoint::FOLLOW_MOUNTED);
  OmActionManager::instance()
    ->action(OmAction::FOLLOW_PAN_AND_TILT)
    ->setChecked(enable && viewpoint->followType() == OmViewpoint::FOLLOW_PAN_AND_TILT);

  cleanupEvents();
}

OmView3D::~OmView3D() {
  // OmniLight: the bake worker owns only snapshots, but it must not outlive the widget.
  if (mOmniBakeThread.joinable())
    mOmniBakeThread.join();
  cleanupFullScreenOverlay();
  cleanupPickers();
  cleanupOptionalRendering();
  OmWrenRenderingContext::cleanup();
  delete mMousePressTimer;
  // R4 3c-B: free the lazily-created wgpu main-view resources (owned by this view; null if never used).
  delete mWgpuRenderTarget;
  delete mWgpuMeshCache;
  delete mWgpuTextureCache;

  OmWrenLabelOverlay::cleanup();
}

void OmView3D::focusInEvent(QFocusEvent *event) {
  OmActionManager::instance()->enableTextEditActions(false, true);
  OmActionManager::instance()->setFocusObject(this);
  emit applicationActionsUpdateRequested();
}

void OmView3D::focusOutEvent(QFocusEvent *event) {
  if (OmActionManager::instance()->focusObject() == this)
    OmActionManager::instance()->setFocusObject(NULL);
  // Stop flying when the 3D view loses keyboard focus, otherwise a held key gets stuck and
  // the camera keeps drifting after the user clicks into the scene tree or another panel.
  stopFly();
}

// main refresh function (update from the simulation engine)
// for refresh coming from the GUI, use renderLater() instead
void OmView3D::refresh() {
  if (!mWorld || !OmSimulationState::instance()->isRendering()) {
    // render black screen
    renderLater();
    return;
  }

  const OmSimulationState *const sim = OmSimulationState::instance();
  mPhysicsRefresh = true;
  if (mScreenshotRequested)
    renderNow(true, true);
  else if (sim->isPaused())
    renderLater();
  else if (OmVideoRecorder::instance()->isRecording()) {
    const double time = OmSimulationState::instance()->time();
    static double lastRefreshTime = time;
    if (time - lastRefreshTime >= OmVideoRecorder::displayRefresh() || time < lastRefreshTime) {
      // render main window immediately even if it is not exposed
      lastRefreshTime = time;
      renderNow();
    }
  } else {
    // Fixed-rate pacing against an absolute due time. Render opportunities arrive quantized to
    // step completions, so the old "elapsed since last render > budget" gate had two structural
    // losses: (a) the render's own CPU time was charged ON TOP of the FPS budget (period =
    // renderTime + budget), and (b) a step quantum just under the budget systematically missed
    // the first eligible boundary -- e.g. 32 ms steps vs a 33.3 ms budget rendered every SECOND
    // step, halving the authored FPS for both renderers. Scheduling against a due time that
    // advances by exactly one budget per rendered frame keeps the average rate at WorldInfo.fps
    // regardless of the step quantum; after a stall (pause, long hitch) it resyncs instead of
    // burst-rendering to catch up.
    const double maxFrameDuration = 1000.0 / mWorld->worldInfo()->fps();  // ms
    const double now = static_cast<double>(mRenderPacingClock.elapsed());
    if (now >= mNextRenderDueMs) {
      mNextRenderDueMs += maxFrameDuration;
      if (now >= mNextRenderDueMs)  // more than one budget behind: resync
        mNextRenderDueMs = now + maxFrameDuration;
      renderNow();
    }
  }
  mPhysicsRefresh = false;
}

// Initializes or terminates solid's camera follow up according to the status of the OmActionManager actions
void OmView3D::followNone(bool checked) {
  if (!checked)
    return;

  mWorld->setModified();
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();
  viewpoint->setFollowType(OmViewpoint::FOLLOW_NONE);
}

void OmView3D::followTracking(bool checked) {
  if (!checked)
    return;

  mWorld->setModified();
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  OmSolid *const selectedSolid = OmSelection::instance()->selectedSolid();
  assert(selectedSolid);
  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();
  viewpoint->setFollowType(OmViewpoint::FOLLOW_TRACKING);
  viewpoint->startFollowUp(selectedSolid, true);
}

void OmView3D::followMounted(bool checked) {
  if (!checked)
    return;

  mWorld->setModified();
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  OmSolid *const selectedSolid = OmSelection::instance()->selectedSolid();
  assert(selectedSolid);
  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();
  viewpoint->setFollowType(OmViewpoint::FOLLOW_MOUNTED);
  viewpoint->startFollowUp(selectedSolid, true);
}

void OmView3D::followPanAndTilt(bool checked) {
  if (!checked)
    return;

  mWorld->setModified();
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  OmSolid *const selectedSolid = OmSelection::instance()->selectedSolid();
  assert(selectedSolid);
  if (viewpoint->followedSolid())
    viewpoint->terminateFollowUp();
  viewpoint->setFollowType(OmViewpoint::FOLLOW_PAN_AND_TILT);
  viewpoint->startFollowUp(selectedSolid, true);
}

void OmView3D::setCheckedFollowObjectAction(OmSolid *selectedSolid) {
  if (selectedSolid) {
    const OmViewpoint *const viewpoint = mWorld->viewpoint();
    if (viewpoint->followType() == OmViewpoint::FOLLOW_NONE)
      OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_TRACKING)
      OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_MOUNTED)
      OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_PAN_AND_TILT)
      OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT)->setChecked(true);
  }
}

// Notifies a change in the follow object action (checked/unchecked) from mViewpoint
void OmView3D::notifyFollowObjectAction(int type) {
  const OmViewpoint *const viewpoint = mWorld->viewpoint();
  if (viewpoint->followType() == OmViewpoint::FOLLOW_NONE)
    OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setChecked(true);
  else if (viewpoint->followType() == OmViewpoint::FOLLOW_TRACKING)
    OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setChecked(true);
  else if (viewpoint->followType() == OmViewpoint::FOLLOW_MOUNTED)
    OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED)->setChecked(true);
  else if (viewpoint->followType() == OmViewpoint::FOLLOW_PAN_AND_TILT)
    OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT)->setChecked(true);
}

// Shows the center of mass and the support polygon of a dynamic top OmSolid
void OmView3D::showSupportPolygon(bool checked) {
  OmSolid *const selectedSolid = OmSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (!selectedSolid->showSupportPolygonRepresentation(checked))
    OmActionManager::instance()->action(OmAction::SUPPORT_POLYGON)->setChecked(false);

  renderLater();
}

// Shows the center of mass of a dynamic OmSolid
void OmView3D::showCenterOfMass(bool checked) {
  OmSolid *const selectedSolid = OmSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (selectedSolid->showGlobalCenterOfMassRepresentation(checked) == false)
    OmActionManager::instance()->action(OmAction::CENTER_OF_MASS)->setChecked(false);

  renderLater();
}

void OmView3D::setCheckedShowCenterOfMassAction(OmSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->globalCenterOfMassRepresentationEnabled();
  OmActionManager::instance()->action(OmAction::CENTER_OF_MASS)->setChecked(enabled);
  if (enabled)
    renderLater();
}

// Shows the center of buoyancy of a dynamic OmSolid
void OmView3D::showCenterOfBuoyancy(bool checked) {
  OmSolid *const selectedSolid = OmSelection::instance()->selectedSolid();
  assert(selectedSolid);

  if (selectedSolid->showCenterOfBuoyancyRepresentation(checked) == false)
    OmActionManager::instance()->action(OmAction::CENTER_OF_BUOYANCY)->setChecked(false);

  renderLater();
}

void OmView3D::setCheckedShowCenterOfBuoyancyAction(OmSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->centerOfBuoyancyRepresentationEnabled();
  OmActionManager::instance()->action(OmAction::CENTER_OF_BUOYANCY)->setChecked(enabled);
  if (enabled)
    renderLater();
}

void OmView3D::setCheckedShowSupportPolygonAction(OmSolid *selectedSolid) {
  assert(selectedSolid);
  const bool enabled = selectedSolid->supportPolygonRepresentationEnabled();
  OmActionManager::instance()
    ->action(OmAction::SUPPORT_POLYGON)
    ->setChecked(selectedSolid->supportPolygonRepresentationEnabled());
  if (enabled)
    renderLater();
}

void OmView3D::restoreViewpoint() {
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  viewpoint->restore();
  renderLater();
}

int OmView3D::stringToRenderingMode(const QString &s) {
  if (s == "WIREFRAME")
    return OmWrenRenderingContext::RM_WIREFRAME;
  return OmWrenRenderingContext::RM_PLAIN;  // default value
}

int OmView3D::stringToProjectionMode(const QString &s) {
  if (s == "ORTHOGRAPHIC")
    return OmWrenRenderingContext::PM_ORTHOGRAPHIC;

  return OmWrenRenderingContext::PM_PERSPECTIVE;
}

void OmView3D::setRenderingMode(int mode, bool updatePerspective) {
  switch (mode) {
    case OmWrenRenderingContext::RM_PLAIN:
      if (updatePerspective && mWorld)
        mWorld->perspective()->setRenderingMode("PLAIN");
      OmActionManager::instance()->action(OmAction::PLAIN_RENDERING)->setChecked(true);
      break;
    case OmWrenRenderingContext::RM_WIREFRAME:
      if (updatePerspective && mWorld)
        mWorld->perspective()->setRenderingMode("WIREFRAME");
      OmActionManager::instance()->action(OmAction::WIREFRAME_RENDERING)->setChecked(true);
      break;
    default:
      assert(false);
  }

  mRenderingMode = mode;

  OmWrenRenderingContext::instance()->setRenderingMode(
    mRenderingMode == OmWrenRenderingContext::RM_PLAIN ? OmWrenRenderingContext::RM_PLAIN :
                                                         OmWrenRenderingContext::RM_WIREFRAME,
    true);

  renderLater();
}

void OmView3D::setProjectionMode(int mode, bool updatePerspective, bool updateAction) {
  mProjectionMode = mode;
  if (mWorld)
    mWorld->viewpoint()->setProjectionMode(mode);

  switch (mode) {
    case OmWrenRenderingContext::PM_ORTHOGRAPHIC:
      if (updateAction)
        OmActionManager::instance()->action(OmAction::ORTHOGRAPHIC_PROJECTION)->setChecked(true);
      if (mWorld) {
        mWorld->viewpoint()->updateOrthographicViewHeight();
        if (updatePerspective)
          mWorld->perspective()->setProjectionMode("ORTHOGRAPHIC");
      }
      break;
    default:
      updateShadowState();
      if (updatePerspective && mWorld)
        mWorld->perspective()->setProjectionMode("PERSPECTIVE");
      if (updateAction)
        OmActionManager::instance()->action(OmAction::PERSPECTIVE_PROJECTION)->setChecked(true);
      break;
  }

  renderLater();
}

void OmView3D::setShowCoordinateSystem(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("CoordinateSystem", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_COORDINATE_SYSTEM, show);
  // Like other optional rendering features, enabling the coordinate system
  // triggers a redraw on the screen. However until the user interacts with
  // webots the coordinate system will not be rendered onto the scene.
  // We force the coordinate system to be rendered here so that it appears
  // immediately, without needing user interaction.
  renderNow();
}

void OmView3D::setShowBoundingObjects(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("AllBoundingObjects", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS, show);
}

void OmView3D::setShowContactPoints(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("ContactPoints", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_CONTACT_POINTS, show);
}

void OmView3D::setShowConnectorAxes(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("ConnectorAxes", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_CONNECTOR_AXES, show);
}

void OmView3D::setShowJointAxes(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("JointAxes", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_JOINT_AXES, show);
}

void OmView3D::setShowCameraFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("CameraFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_CAMERA_FRUSTUMS, show);
}

void OmView3D::setShowRangeFinderFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("RangeFinderFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS, show);
}

void OmView3D::setShowRadarFrustums(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("RadarFrustums", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_RADAR_FRUSTUMS, show);
}

void OmView3D::setShowLidarRaysPaths(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LidarRaysPaths", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_LIDAR_RAYS_PATHS, show);
}

void OmView3D::setShowLidarPointClouds(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LidarPointClouds", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_LIDAR_POINT_CLOUD, show);
}

void OmView3D::setShowRenderingDevice(bool checked) {
  OmRenderingDevice *device = static_cast<OmRenderingDevice *>(sender()->property("renderingDevice").value<void *>());
  device->toggleOverlayVisibility(checked);
  renderLater();
}

void OmView3D::setHideAllCameraOverlays(bool hidden) {
  OmPreferences::instance()->setValue("View3d/hideAllCameraOverlays", hidden);

  OmWrenTextureOverlay::setElementsVisible(OmWrenTextureOverlay::OVERLAY_TYPE_CAMERA, !hidden);

  renderLater();
}

void OmView3D::setHideAllRangeFinderOverlays(bool hidden) {
  OmPreferences::instance()->setValue("View3d/hideAllRangeFinderOverlays", hidden);

  OmWrenTextureOverlay::setElementsVisible(OmWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER, !hidden);

  renderLater();
}

void OmView3D::setHideAllDisplayOverlays(bool hidden) {
  OmPreferences::instance()->setValue("View3d/hideAllDisplayOverlays", hidden);

  OmWrenTextureOverlay::setElementsVisible(OmWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, !hidden);

  renderLater();
}

void OmView3D::setShowDistanceSensorRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("DistanceSensorRays", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_DISTANCE_SENSORS_RAYS, show);
}

void OmView3D::setShowLightSensorRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LightSensorRays", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_LIGHT_SENSORS_RAYS, show);
}

void OmView3D::setShowLightsPositions(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("LightPositions", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_LIGHTS_POSITIONS, show);
}

void OmView3D::setShowPenPaintingRays(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("PenPaintingRays", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_PEN_RAYS, show);
}

void OmView3D::setShowSkeletonAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("Skeleton", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_SKIN_SKELETON, show);
}

void OmView3D::setShowNormals(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("Normals", show);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_NORMALS, show);
}

void OmView3D::setShowPhysicsClustersAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("PhysicsClusters", show);
}

void OmView3D::setShowBoundingSphereAction(bool show) {
  if (mWorld)
    mWorld->perspective()->enableGlobalOptionalRendering("BoundingSphere", show);
  OmVisualBoundingSphere::enable(show, OmSelection::instance()->selectedNode());
  renderLater();
}

void OmView3D::setUserInteractionDisabled(OmAction::OmActionKind action, bool disabled) {
  mDisabledUserInteractionsMap[action] = disabled;
  if (mWorld)
    mWorld->perspective()->setUserInteractionDisabled(action, disabled);
}

void OmView3D::disableObjectMove(bool disabled) {
  setUserInteractionDisabled(OmAction::DISABLE_OBJECT_MOVE, disabled);
  if (disabled)
    OmSelection::instance()->disableActiveManipulator();
  else
    OmSelection::instance()->restoreActiveManipulator();
  renderLater();
}

void OmView3D::updateMousesPosition(bool fromMouseClick, bool fromMouseMove) {
  const QList<OmMouse *> mouses = OmMouse::mouses();
  if (mouses.size() == 0)
    return;

  QList<OmMouse *> mousesRequiringRefresh;
  bool shouldUsePicker = false;
  for (int i = 0; i < OmMouse::mouses().size(); ++i) {
    OmMouse *mouse = OmMouse::mouses().at(i);
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
    mControllerPicker = new OmScenePicker();

  const bool picked = shouldUsePicker ? mControllerPicker->pick(position.x(), position.y()) : false;

  foreach (OmMouse *mouse, mousesRequiringRefresh) {
    if (picked && mouse->is3dPositionEnabled()) {
      const OmVector3 &worldPosition = mControllerPicker->worldCoordinates();
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

void OmView3D::logWrenStatistics() {
  OmPerformanceLog *log = OmPerformanceLog::instance();
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

void OmView3D::prepareWorldLoading() {
  // reset text labels
  OmWrenLabelOverlay::removeAllLabels();

  hideBlackRenderingOverlay();

  // Cleanup the drags events that were possibly used in the previous world
  cleanupEvents();

  // signals that update the menu's ticks according to the status of the selection
  disconnect(OmSelection::instance(), &OmSelection::selectionChangedFromView3D, this, &OmView3D::onSelectionChanged);
  disconnect(OmSelection::instance(), &OmSelection::selectionChangedFromSceneTree, this, &OmView3D::onSelectionChanged);

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
}

// Drop the cached main-view draw list (and its destroyed() hooks) and mark it for rebuild. Called
// when a referenced node is destroyed, on world (re)load, and periodically for appearance staleness.
void OmView3D::invalidateWgpuDrawList() {
  for (const QMetaObject::Connection &c : mWgpuDrawListConns)
    QObject::disconnect(c);
  mWgpuDrawListConns.clear();
  mWgpuDrawList.clear();
  mWgpuModelList.clear();
  mWgpuRefreshList.clear();
  // Both vectors just lost their dynamic tail along with everything else, so there is
  // nothing left to trim. Missing this is how the trim below would eat real scene draws.
  mWgpuDynamicDraws = 0;
  mWgpuDeformableDraws = 0;
  mWgpuGranularDraws = 0;
  mWgpuTrackDraws = 0;
  mWgpuMuscleDraws = 0;
  mWgpuDrawListDirty = true;
  mWgpuDrawListAge = 0;
  // W4a: the saved selection-tint indices name draws in the list we just dropped. Clear them
  // AND null the tinted-solid token, so the next frame re-tints the rebuilt list instead of
  // believing the highlight is already applied (which is how a selection silently loses its
  // highlight the moment a node is added or destroyed).
  mWgpuSelTintIdx.clear();
  mWgpuSelTintRgb.clear();
  mWgpuSelTintTop = nullptr;
}

void OmView3D::updateViewport() {
  // Sets the solid follow up according to viewpoint's follow field
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  connect(viewpoint, &OmViewpoint::followTypeChanged, this, &OmView3D::notifyFollowObjectAction);
  if (viewpoint->followedSolid()) {
    if (viewpoint->followType() == OmViewpoint::FOLLOW_NONE)
      OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_TRACKING)
      OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_MOUNTED)
      OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_PAN_AND_TILT)
      OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT)->setChecked(true);
  } else {
    OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT)->setChecked(false);
  }

  cleanupPickers();
  mPicker = new OmScenePicker();
  mPicker->setSharedMeshCache(mWgpuMeshCache);

  viewpoint->updateAspectRatio(mAspectRatio);

  // Re-initialize matter handles size
  OmSelection::instance()->updateHandlesScale();

  // update handles size when viewpoint changes
  connect(viewpoint, &OmViewpoint::cameraParametersChanged, OmSelection::instance(), &OmSelection::updateHandlesScale);
}

void OmView3D::updateShadowState() {
  // D1.4: WREN's shadow toggle died with WREN (wgpu owns its own shadow pipeline).
}

void OmView3D::setWorld(OmSimulationWorld *w) {
  mWorld = w;  // world is loaded!

  // apply optional rendering
  if (OmPreferences::instance()->value("View3d/hideAllCameraOverlays").toBool())
    setHideAllCameraOverlays(true);
  if (OmPreferences::instance()->value("View3d/hideAllRangeFinderOverlays").toBool())
    setHideAllRangeFinderOverlays(true);
  if (OmPreferences::instance()->value("View3d/hideAllDisplayOverlays").toBool())
    setHideAllDisplayOverlays(true);

  const OmPerspective *perspective = mWorld->perspective();
  setProjectionMode(stringToProjectionMode(perspective->projectionMode()), false, true);
  setRenderingMode(stringToRenderingMode(perspective->renderingMode()), false);
  mDisabledUserInteractionsMap = perspective->disabledUserInteractionsMap();

  enableOptionalRenderingFromPerspective();

  connect(mWorld, &OmSimulationWorld::destroyed, this, &OmView3D::cleanWorld);
  connect(mWorld, &OmWorld::viewpointChanged, this, &OmView3D::updateViewport);

  // Sets the solid follow up according to viewpoint's follow field
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  connect(viewpoint, &OmViewpoint::followTypeChanged, this, &OmView3D::notifyFollowObjectAction);
  viewpoint->startFollowUpFromField();
  if (viewpoint->followedSolid()) {
    if (viewpoint->followType() == OmViewpoint::FOLLOW_NONE)
      OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_TRACKING)
      OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_MOUNTED)
      OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED)->setChecked(true);
    else if (viewpoint->followType() == OmViewpoint::FOLLOW_PAN_AND_TILT)
      OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT)->setChecked(true);
  } else {
    OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED)->setChecked(false);
    OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT)->setChecked(false);
  }

  // Prepares the contact point rendering (Note: OmControlledWorld::instance() is valid after the call to
  // mMainWindow->loadWorld(mWorldName) in OmGuiApplication.cpp)
  // The constructor connects an update slot to the signal OmSimulationWorld::physicsStepEnded()

  // Connects GUI-defined mode and rendering options to update methods for material of bounding objects
  const OmSimulationState *const simulationState = OmSimulationState::instance();
  connect(mWrenRenderingContext, &OmWrenRenderingContext::optionalRenderingChanged, mWorld,
          &OmSimulationWorld::checkNeedForBoundingMaterialUpdate, Qt::UniqueConnection);
  connect(simulationState, &OmSimulationState::renderingStateChanged, mWorld,
          &OmSimulationWorld::checkNeedForBoundingMaterialUpdate, Qt::UniqueConnection);
  mWorld->checkNeedForBoundingMaterialUpdate();

  // Prepares the shape picker
  delete mPicker;
  delete mControllerPicker;
  mControllerPicker = NULL;
  mPicker = new OmScenePicker();
  mPicker->setSharedMeshCache(mWgpuMeshCache);

  if (OmSimulationState::instance()->isRendering())
    hideBlackRenderingOverlay();
  else
    showBlackRenderingOverlay();

  // connect supervisor scene tree modifications to graphical updates
  const QList<OmRobot *> &robots = mWorld->robots();
  foreach (const OmRobot *const robot, robots) {
    if (robot->supervisor())
      connect(robot->supervisorUtilities(), &OmSupervisorUtilities::worldModified, this,
              &OmView3D::handleWorldModificationFromSupervisor);
  }

  // initialize matter handles size
  OmSelection::instance()->updateHandlesScale();
  // update handles size when viewpoint changes
  connect(viewpoint, &OmViewpoint::cameraParametersChanged, OmSelection::instance(), &OmSelection::updateHandlesScale);
  connect(viewpoint, &OmViewpoint::refreshRequired, this, &OmView3D::renderLater);

  // signals that update the menu's ticks according to the status of the selection
  connect(OmSelection::instance(), &OmSelection::selectionChangedFromView3D, this, &OmView3D::onSelectionChanged);
  connect(OmSelection::instance(), &OmSelection::selectionChangedFromSceneTree, this, &OmView3D::onSelectionChanged);

  mAspectRatio = ((double)width()) / height();
  viewpoint->updateAspectRatio(mAspectRatio);
  updateScreenPixelRatio();
  onSelectionChanged(OmSelection::instance()->selectedAbstractPose());
}

void OmView3D::restoreOptionalRendering(const QStringList &enabledCenterOfMassNodeNames,
                                        const QStringList &enabledCenterOfBuoyancyNodeNames,
                                        const QStringList &enabledSupportPolygonNodeNames) const {
  // restore node specific optional rendering from world properties
  OmSolid *solid = NULL;
  for (int i = 0; i < enabledCenterOfMassNodeNames.size(); ++i) {
    solid = OmSolid::findSolidFromUniqueName(enabledCenterOfMassNodeNames[i]);
    if (solid)
      solid->showGlobalCenterOfMassRepresentation(true);
  }

  for (int i = 0; i < enabledCenterOfBuoyancyNodeNames.size(); ++i) {
    solid = OmSolid::findSolidFromUniqueName(enabledCenterOfBuoyancyNodeNames[i]);
    if (solid)
      solid->showCenterOfBuoyancyRepresentation(true);
  }

  for (int i = 0; i < enabledSupportPolygonNodeNames.size(); ++i) {
    solid = OmSolid::findSolidFromUniqueName(enabledSupportPolygonNodeNames[i]);
    if (solid)
      solid->showSupportPolygonRepresentation(true);
  }
}

void OmView3D::enableOptionalRenderingFromPerspective() {
  // Enables optional rendering from preferences
  assert(mWorld);
  const OmPerspective *perspective = mWorld->perspective();
  OmActionManager *actionManager = OmActionManager::instance();
  actionManager->action(OmAction::COORDINATE_SYSTEM)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("CoordinateSystem"));
  actionManager->action(OmAction::BOUNDING_OBJECT)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("AllBoundingObjects"));
  actionManager->action(OmAction::NORMALS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("Normals"));
  actionManager->action(OmAction::CONTACT_POINTS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("ContactPoints"));
  actionManager->action(OmAction::CONNECTOR_AXES)->setChecked(perspective->isGlobalOptionalRenderingEnabled("ConnectorAxes"));
  actionManager->action(OmAction::JOINT_AXES)->setChecked(perspective->isGlobalOptionalRenderingEnabled("JointAxes"));
  actionManager->action(OmAction::RANGE_FINDER_FRUSTUMS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("RangeFinderFrustums"));
  actionManager->action(OmAction::LIDAR_RAYS_PATH)->setChecked(perspective->isGlobalOptionalRenderingEnabled("LidarRaysPaths"));
  actionManager->action(OmAction::LIDAR_POINT_CLOUD)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("LidarPointClouds"));
  actionManager->action(OmAction::CAMERA_FRUSTUM)->setChecked(perspective->isGlobalOptionalRenderingEnabled("CameraFrustums"));
  actionManager->action(OmAction::DISTANCE_SENSOR_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("DistanceSensorRays"));
  actionManager->action(OmAction::LIGHT_SENSOR_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("LightSensorRays"));
  actionManager->action(OmAction::LIGHT_POSITIONS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("LightPositions"));
  actionManager->action(OmAction::CENTER_OF_BUOYANCY)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("CenterOfBuoyancy"));
  actionManager->action(OmAction::PEN_PAINTING_RAYS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("PenPaintingRays"));
  actionManager->action(OmAction::CENTER_OF_MASS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("CenterOfMass"));
  actionManager->action(OmAction::SUPPORT_POLYGON)->setChecked(perspective->isGlobalOptionalRenderingEnabled("SupportPolygon"));
  actionManager->action(OmAction::SKIN_SKELETON)->setChecked(perspective->isGlobalOptionalRenderingEnabled("Skeleton"));
  actionManager->action(OmAction::RADAR_FRUSTUMS)->setChecked(perspective->isGlobalOptionalRenderingEnabled("RadarFrustums"));
  actionManager->action(OmAction::PHYSICS_CLUSTERS)
    ->setChecked(perspective->isGlobalOptionalRenderingEnabled("PhysicsClusters"));
  actionManager->action(OmAction::BOUNDING_SPHERE)->setChecked(perspective->isGlobalOptionalRenderingEnabled("BoundingSphere"));
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_COORDINATE_SYSTEM,
                                                 perspective->isGlobalOptionalRenderingEnabled("CoordinateSystem"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS,
                                                 perspective->isGlobalOptionalRenderingEnabled("AllBoundingObjects"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_NORMALS,
                                                 perspective->isGlobalOptionalRenderingEnabled("Normals"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_CONTACT_POINTS,
                                                 perspective->isGlobalOptionalRenderingEnabled("ContactPoints"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_CONNECTOR_AXES,
                                                 perspective->isGlobalOptionalRenderingEnabled("ConnectorAxes"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_JOINT_AXES,
                                                 perspective->isGlobalOptionalRenderingEnabled("JointAxes"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("RangeFinderFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_LIDAR_RAYS_PATHS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LidarRaysPaths"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_LIDAR_POINT_CLOUD,
                                                 perspective->isGlobalOptionalRenderingEnabled("LidarPointClouds"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_CAMERA_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("CameraFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_RADAR_FRUSTUMS,
                                                 perspective->isGlobalOptionalRenderingEnabled("RadarFrustums"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_DISTANCE_SENSORS_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("DistanceSensorRays"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_LIGHT_SENSORS_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LightSensorRays"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_LIGHTS_POSITIONS,
                                                 perspective->isGlobalOptionalRenderingEnabled("LightPositions"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_PEN_RAYS,
                                                 perspective->isGlobalOptionalRenderingEnabled("PenPaintingRays"), false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VF_SKIN_SKELETON,
                                                 perspective->isGlobalOptionalRenderingEnabled("Skeleton"), false);

  // OMNISIM_OPTIONAL_RENDERING=<comma-separated names> force-enables optional renderings at load,
  // using the SAME names the perspective file uses ("AllBoundingObjects", "JointAxes",
  // "CameraFrustums", ...). Unset changes nothing.
  //
  // ⚠ WHY THIS EXISTS. Until now the ONLY way to turn an optional rendering on was a human
  // clicking the View menu, which meant the whole class of overlay/gizmo/HUD pixels had NO
  // automated verification path at all -- and that is precisely how the wgpu default flip on
  // 2026-08-19 left selection feedback, all 21 Optional Rendering items and the device HUDs dark
  // for days without anyone noticing. Four separate attempts to enable one from a script failed
  // (a hand-written .wbproj, both the .omniperspective and legacy .wbproj extensions, and a
  // renderingMode probe), each for a different reason. Phase W4 of
  // docs/developer/wren-retirement-plan.md cannot be finished safely without this lever, because
  // "the collector is wired in" is not a claim anyone can currently check.
  {
    const QString wanted = qEnvironmentVariable("OMNISIM_OPTIONAL_RENDERING").trimmed();
    if (!wanted.isEmpty()) {
      static const struct { const char *name; int flag; } cMap[] = {
        {"CoordinateSystem", OmWrenRenderingContext::VF_COORDINATE_SYSTEM},
        {"AllBoundingObjects", OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS},
        {"Normals", OmWrenRenderingContext::VF_NORMALS},
        {"ContactPoints", OmWrenRenderingContext::VF_CONTACT_POINTS},
        {"ConnectorAxes", OmWrenRenderingContext::VF_CONNECTOR_AXES},
        {"JointAxes", OmWrenRenderingContext::VF_JOINT_AXES},
        {"RangeFinderFrustums", OmWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS},
        {"LidarRaysPaths", OmWrenRenderingContext::VF_LIDAR_RAYS_PATHS},
        {"LidarPointClouds", OmWrenRenderingContext::VF_LIDAR_POINT_CLOUD},
        {"CameraFrustums", OmWrenRenderingContext::VF_CAMERA_FRUSTUMS},
        {"RadarFrustums", OmWrenRenderingContext::VF_RADAR_FRUSTUMS},
        {"DistanceSensorRays", OmWrenRenderingContext::VF_DISTANCE_SENSORS_RAYS},
        {"LightSensorRays", OmWrenRenderingContext::VF_LIGHT_SENSORS_RAYS},
        {"PenPaintingRays", OmWrenRenderingContext::VF_PEN_RAYS},
        {"Skeleton", OmWrenRenderingContext::VF_SKIN_SKELETON},
      };
      const QStringList names = wanted.split(',', Qt::SkipEmptyParts);
      QStringList applied, unknown;
      for (const QString &raw : names) {
        const QString n = raw.trimmed();
        if (n.isEmpty())
          continue;
        bool found = false;
        for (const auto &e : cMap)
          if (n.compare(QString::fromLatin1(e.name), Qt::CaseInsensitive) == 0) {
            mWrenRenderingContext->enableOptionalRendering(e.flag, true, true);
            applied << QString::fromLatin1(e.name);
            found = true;
            break;
          }
        if (!found)
          unknown << n;
      }
      // Name every rejected entry: a silently ignored typo would look exactly like a renderer
      // that failed to draw, which is the confusion this whole lever exists to remove.
      OmLog::info(QString("[OmView3D] OMNISIM_OPTIONAL_RENDERING enabled: %1%2")
                    .arg(applied.isEmpty() ? QStringLiteral("(none)") : applied.join(", "))
                    .arg(unknown.isEmpty() ? QString() :
                                             QString("  -- UNKNOWN, ignored: %1").arg(unknown.join(", "))));
    }
  }
}

void OmView3D::disableOptionalRenderingAndOverLays() {
  // Save optional renderings before saving thumbnail
  mOptionalRenderingsMask = mWrenRenderingContext->optionalRenderingsMask();

  // Temporary hide optional renderings (without notifying the nodes and removing them from the scene)
  // unset optional renderings flags in mask and set VM_REGULAR (no special rendering) bits only
  mWrenRenderingContext->blockSignals(true);
  mWrenRenderingContext->enableOptionalRendering(~OmWrenRenderingContext::VM_REGULAR, false, false);
  mWrenRenderingContext->enableOptionalRendering(OmWrenRenderingContext::VM_REGULAR, true, false);
  mWrenRenderingContext->blockSignals(false);

  // Hide overlays for thumbnail
  setHideAllCameraOverlays(true);
  setHideAllRangeFinderOverlays(true);
  setHideAllDisplayOverlays(true);

  // Switch to perspective projection if necessary
  if (mWorld->viewpoint()->projectionMode() == OmWrenRenderingContext::PM_ORTHOGRAPHIC)
    setProjectionMode(OmWrenRenderingContext::PM_PERSPECTIVE, true, false);
}

void OmView3D::restoreOptionalRenderingAndOverLays() {
  // Restore optional renderings (without notifying all the nodes)
  mWrenRenderingContext->blockSignals(true);
  mWrenRenderingContext->enableOptionalRendering(mOptionalRenderingsMask, true, false);
  mWrenRenderingContext->blockSignals(false);

  // Restore overlays after saving thumbnail
  OmActionManager *actionManager = OmActionManager::instance();
  setHideAllCameraOverlays(actionManager->action(OmAction::HIDE_ALL_CAMERA_OVERLAYS)->isChecked());
  setHideAllRangeFinderOverlays(actionManager->action(OmAction::HIDE_ALL_RANGE_FINDER_OVERLAYS)->isChecked());
  setHideAllDisplayOverlays(actionManager->action(OmAction::HIDE_ALL_DISPLAY_OVERLAYS)->isChecked());

  // Switch back to orthographic projection if necessary
  if (OmActionManager::instance()->action(OmAction::ORTHOGRAPHIC_PROJECTION)->isChecked())
    setProjectionMode(OmWrenRenderingContext::PM_ORTHOGRAPHIC, true, false);
}

void OmView3D::initialize() {
  // prepare WREN rendering context
  OmWrenRenderingContext::setWrenRenderingContext(width(), height());
  mWrenRenderingContext = OmWrenRenderingContext::instance();

  // propagate main window refresh signals
  connect(this, &OmView3D::mainRenderingStarted, mWrenRenderingContext, &OmWrenRenderingContext::mainRenderingStarted);
  connect(this, &OmView3D::mainRenderingEnded, mWrenRenderingContext, &OmWrenRenderingContext::mainRenderingEnded);

  // refresh for example when the user change an optional rendering option or
  // the rendering device external window is closed
  connect(mWrenRenderingContext, &OmWrenRenderingContext::view3dRefreshRequired, this, &OmView3D::renderLater);

  // reset the render pacing so the first frame after (re)initialization renders immediately
  mNextRenderDueMs = 0.0;

  // GL-less degrade (shipped since D1.5): the GL bring-up + the device pop-out factory's stored context
  // need a real context. Inert when GL is present (the default).
  if (!OmWrenOpenGlContext::isInitialized())
    return;

  OmGlWindow::initialize();

  OmRenderingDeviceWindowFactory::storeOpenGLContext(OmWrenOpenGlContext::instance());
}

void OmView3D::resizeWren(int width, int height) {
  if (!mWorld)
    return;

  if (mWrenRenderingContext)
    mWrenRenderingContext->setDimension(width, height);

  if (mWorld) {
    mAspectRatio = (double)width / height;
    mWorld->viewpoint()->updateAspectRatio(mAspectRatio);
  }

  OmGlWindow::resizeWren(width, height);
}

void OmView3D::renderNow(bool culling, bool offScreen) {
  if (mWorld) {
    emit mainRenderingStarted(mPhysicsRefresh);
    // D1.4: wgpu is the only renderer. A declined frame (offscreen request, mid-reload,
    // backend unavailable) simply produces no pixels this call.
    if (!renderMainFrameViaWgpu(culling, offScreen))
      mWgpuPresentedLastFrame = false;
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

// First OmDirectionalLight anywhere in the world tree (e.g. the OmniSimSun PROTO's light), so the
// wgpu main view lights + casts shadows from WREN's actual sun direction. Mirrors the same-named
// helper in OmWgpuView.cpp's parity self-check; kept file-local to avoid a cross-TU dependency.
static OmDirectionalLight *findFirstDirectionalLightV3D(OmBaseNode *root) {
  if (!root)
    return nullptr;
  if (OmDirectionalLight *dl = dynamic_cast<OmDirectionalLight *>(root))
    return dl;
  if (OmGroup *g = dynamic_cast<OmGroup *>(root)) {
    const int n = g->childCount();
    for (int i = 0; i < n; ++i)
      if (OmDirectionalLight *d = findFirstDirectionalLightV3D(g->child(i)))
        return d;
  }
  return nullptr;
}

// First OmBackground in the world tree — its skyColor is the world's sky tint, fed as the
// hemisphere-IBL ambient (sky from above, a darkened bounce from below) so shadowed regions take a
// world-appropriate fill instead of crushing to black. This is the world-general replacement for the
// old panda-tuned "ambient 0" — the fix that makes the wgpu main view look right across the corpus,
// not just on panda.
static OmBackground *findFirstBackgroundV3D(OmBaseNode *root) {
  if (!root)
    return nullptr;
  if (OmBackground *b = dynamic_cast<OmBackground *>(root))
    return b;
  if (OmGroup *g = dynamic_cast<OmGroup *>(root)) {
    const int n = g->childCount();
    for (int i = 0; i < n; ++i)
      if (OmBackground *b = findFirstBackgroundV3D(g->child(i)))
        return b;
  }
  return nullptr;
}

// Multi-light: every ADDITIONAL scene light beyond the shadowed sun — PointLight, SpotLight, and any
// DirectionalLight other than `sun` — packed 16 floats per light in the WGSL ExtraLight layout
// {posType, colorRad, atten, spotDir}. Off / zero-intensity lights are skipped; capped at 8 (the
// shader's array size). These render as UNSHADOWED fills: only the sun owns the shadow map.
// ---- CPU port of the kSkyScatterLut single-scatter march (16 view / 4 sun steps) ----
// Evaluates a handful of sky directions on the CPU so the hemisphere ambient + the analytic-IBL
// env palette derive from the SAME atmosphere the dome renders. Called only when the sun or the
// preset changed (cached by the caller); cost is microseconds.
namespace {
  struct SkyMarchP {
    float rayR, rayG, rayB, rayExp;   // per-metre Rayleigh + density exp scale (/km)
    float mieS, mieExp;               // per-metre Mie scattering (grey) + exp scale
    float mieA, phaseG;               // per-metre Mie absorption (grey) + phase g
    float ozR, ozG, ozB, camHkm;      // per-metre ozone + camera height (km)
    float botKm, topKm, sinSunElev, albedo;
    float illR, illG, illB;
  };

  static double sciExit(const double o[3], const double d[3], double r) {
    const double b = o[0] * d[0] + o[1] * d[1] + o[2] * d[2];
    const double c = o[0] * o[0] + o[1] * o[1] + o[2] * o[2] - r * r;
    const double disc = b * b - c;
    return disc < 0.0 ? -1.0 : -b + std::sqrt(disc);
  }
  static double sciEnter(const double o[3], const double d[3], double r) {
    const double b = o[0] * d[0] + o[1] * d[1] + o[2] * d[2];
    const double c = o[0] * o[0] + o[1] * o[1] + o[2] * o[2] - r * r;
    const double disc = b * b - c;
    if (disc < 0.0)
      return -1.0;
    const double t = -b - std::sqrt(disc);
    return t < 0.0 ? -1.0 : t;
  }
  static void sciOdepth(const SkyMarchP &p, const double o[3], const double d[3], double out[3]) {
    out[0] = out[1] = out[2] = 0.0;
    const double tExit = sciExit(o, d, p.topKm);
    if (tExit < 0.0)
      return;
    if (sciEnter(o, d, p.botKm) > 0.0) {
      out[0] = out[1] = out[2] = 1e10;
      return;
    }
    const double dt = tExit / 4.0;
    for (int i = 0; i < 4; ++i) {
      const double t = dt * (i + 0.5);
      const double px = o[0] + d[0] * t, py = o[1] + d[1] * t, pz = o[2] + d[2] * t;
      const double h = std::max(0.0, std::sqrt(px * px + py * py + pz * pz) - p.botKm);
      const double rd = std::exp(h * p.rayExp), md = std::exp(h * p.mieExp);
      out[0] += (p.rayR * rd + (p.mieS + p.mieA) * md + p.ozR * rd) * dt * 1000.0;
      out[1] += (p.rayG * rd + (p.mieS + p.mieA) * md + p.ozG * rd) * dt * 1000.0;
      out[2] += (p.rayB * rd + (p.mieS + p.mieA) * md + p.ozB * rd) * dt * 1000.0;
    }
  }
  // Marches one view direction (LUT frame: Y up, sun in the XZ=0 half-plane); out = linear radiance.
  static void sciSampleSky(const SkyMarchP &p, double elev, double azDelta, float out[3]) {
    const double cE = std::cos(elev);
    const double view[3] = {std::cos(azDelta) * cE, std::sin(elev), std::sin(azDelta) * cE};
    const double sE = std::max(-1.0, std::min(1.0, static_cast<double>(p.sinSunElev)));
    const double sun[3] = {std::sqrt(std::max(0.0, 1.0 - sE * sE)), sE, 0.0};
    const double origin[3] = {0.0, p.botKm + p.camHkm, 0.0};
    out[0] = out[1] = out[2] = 0.0f;
    const double tExit = sciExit(origin, view, p.topKm);
    if (tExit <= 0.0)
      return;
    const double tGround = sciEnter(origin, view, p.botKm);
    const double tEnd = tGround > 0.0 ? std::min(tExit, tGround) : tExit;
    const double dt = tEnd / 16.0;
    double sum[3] = {0, 0, 0}, trans[3] = {1, 1, 1};
    const double cosSV = view[0] * sun[0] + view[1] * sun[1] + view[2] * sun[2];
    const double pR = (3.0 / (16.0 * M_PI)) * (1.0 + cosSV * cosSV);
    const double g = p.phaseG, g2 = g * g;
    const double pM = (3.0 / (8.0 * M_PI)) * ((1.0 - g2) * (1.0 + cosSV * cosSV)) /
                      ((2.0 + g2) * std::pow(std::max(1e-6, 1.0 + g2 - 2.0 * g * cosSV), 1.5));
    const double rayS[3] = {p.rayR, p.rayG, p.rayB};
    const double ozS[3] = {p.ozR, p.ozG, p.ozB};
    for (int i = 0; i < 16; ++i) {
      const double t = dt * (i + 0.5);
      const double px = origin[0] + view[0] * t, py = origin[1] + view[1] * t, pz = origin[2] + view[2] * t;
      const double h = std::max(0.0, std::sqrt(px * px + py * py + pz * pz) - p.botKm);
      const double rd = std::exp(h * p.rayExp), md = std::exp(h * p.mieExp);
      const double pp[3] = {px, py, pz};
      double sunOD[3];
      sciOdepth(p, pp, sun, sunOD);
      for (int k = 0; k < 3; ++k) {
        const double ext = rayS[k] * rd + (p.mieS + p.mieA) * md + ozS[k] * rd;
        const double inscatter = (rayS[k] * rd * pR + p.mieS * md * pM) * std::exp(-sunOD[k]) * dt * 1000.0;
        sum[k] += trans[k] * inscatter;
        trans[k] *= std::exp(-ext * dt * 1000.0);
      }
    }
    if (tGround > 0.0) {
      const double gx = origin[0] + view[0] * tGround, gy = origin[1] + view[1] * tGround,
                   gz = origin[2] + view[2] * tGround;
      const double gp[3] = {gx, gy, gz};
      double sunOD[3];
      sciOdepth(p, gp, sun, sunOD);
      const double nDotL = std::max(0.0, sE);
      for (int k = 0; k < 3; ++k)
        sum[k] += trans[k] * (p.albedo / M_PI) * std::exp(-sunOD[k]) * nDotL;
    }
    const double ill[3] = {p.illR, p.illG, p.illB};
    for (int k = 0; k < 3; ++k)
      out[k] = static_cast<float>(sum[k] * ill[k]);
  }
}  // namespace

static void collectExtraLightsV3D(OmBaseNode *root, const OmDirectionalLight *sun, std::vector<float> &out) {
  if (!root || out.size() >= 8u * 16u)
    return;
  if (OmLight *l = dynamic_cast<OmLight *>(root)) {
    if (!l->isOn())
      return;
    const double inten = l->intensity();
    if (inten <= 0.0)
      return;
    float rec[16] = {0};
    const OmRgb c = l->color();
    rec[4] = static_cast<float>(c.red() * inten);
    rec[5] = static_cast<float>(c.green() * inten);
    rec[6] = static_cast<float>(c.blue() * inten);
    if (OmDirectionalLight *dl = dynamic_cast<OmDirectionalLight *>(l)) {
      if (dl == sun)
        return;  // the sun renders through the shadowed direct path, not as an extra
      const OmVector3 d = dl->direction().normalized();
      rec[0] = static_cast<float>(d.x());
      rec[1] = static_cast<float>(d.y());
      rec[2] = static_cast<float>(d.z());
      rec[3] = 0.0f;  // type: directional
    } else if (OmPointLight *pl = dynamic_cast<OmPointLight *>(l)) {
      const OmVector3 p = pl->computeAbsoluteLocation();
      rec[0] = static_cast<float>(p.x());
      rec[1] = static_cast<float>(p.y());
      rec[2] = static_cast<float>(p.z());
      rec[3] = 1.0f;  // type: point
      rec[7] = static_cast<float>(pl->radius());
      const OmVector3 &a = pl->attenuation();
      rec[8] = static_cast<float>(a.x());
      rec[9] = static_cast<float>(a.y());
      rec[10] = static_cast<float>(a.z());
    } else if (OmSpotLight *sl = dynamic_cast<OmSpotLight *>(l)) {
      const OmVector3 p = sl->computeAbsoluteLocation();
      rec[0] = static_cast<float>(p.x());
      rec[1] = static_cast<float>(p.y());
      rec[2] = static_cast<float>(p.z());
      rec[3] = 2.0f;  // type: spot
      rec[7] = static_cast<float>(sl->radius());
      const OmVector3 &a = sl->attenuation();
      rec[8] = static_cast<float>(a.x());
      rec[9] = static_cast<float>(a.y());
      rec[10] = static_cast<float>(a.z());
      rec[11] = static_cast<float>(std::cos(sl->cutOffAngle()));
      const OmVector3 d = sl->computeAbsoluteDirection().normalized();
      rec[12] = static_cast<float>(d.x());
      rec[13] = static_cast<float>(d.y());
      rec[14] = static_cast<float>(d.z());
      rec[15] = static_cast<float>(std::cos(sl->beamWidth()));
    } else
      return;  // some other OmLight subclass — not supported as an extra
    out.insert(out.end(), rec, rec + 16);
    return;  // lights carry no child scene nodes of interest
  }
  if (OmGroup *g = dynamic_cast<OmGroup *>(root)) {
    const int n = g->childCount();
    for (int i = 0; i < n && out.size() < 8u * 16u; ++i)
      collectExtraLightsV3D(g->child(i), sun, out);
  }
}

// W4a (WREN retirement): the SELECTION OUTLINE — a world-space AABB wireframe around
// everything the selected TOP solid draws.
//
// Derived from the cached draw list's OWN bounds (localCenter/localRadius + the model matrix
// the frame is about to render with), so it costs no scene walk, is automatically in sync with
// what is on screen, and — unlike a boundingObject-derived outline — also works for a purely
// visual Solid that declares no boundingObject at all. Conservative by construction (each
// draw contributes a bounding SPHERE), which is the right way for a selection box to be wrong.
// Appends 12 edges to `out`; appends nothing when the solid has no renderable draw.
static void collectWgpuSelectionBoxV3D(const std::vector<OmWgpuSolidDraw> &draws,
                                       const std::vector<OmSolid *> &owners, const OmSolid *selTop,
                                       std::vector<float> &out) {
  if (!selTop)
    return;
  double mn[3] = {0.0, 0.0, 0.0}, mx[3] = {0.0, 0.0, 0.0};
  bool any = false;
  const size_t n = std::min(draws.size(), owners.size());
  for (size_t i = 0; i < n; ++i) {
    if (owners[i] != selTop || !draws[i].modelMatrix16)
      continue;
    const float *m = draws[i].modelMatrix16;  // column-major
    const double lx = draws[i].localCenter[0], ly = draws[i].localCenter[1],
                 lz = draws[i].localCenter[2];
    const double c[3] = {m[0] * lx + m[4] * ly + m[8] * lz + m[12],
                         m[1] * lx + m[5] * ly + m[9] * lz + m[13],
                         m[2] * lx + m[6] * ly + m[10] * lz + m[14]};
    if (!std::isfinite(c[0]) || !std::isfinite(c[1]) || !std::isfinite(c[2]))
      continue;  // the degenerate-transform sentinels the self-check found on some robots
    // Largest column length of the upper 3x3 — the worst-case scale the local radius sees.
    double s = 0.0;
    for (int col = 0; col < 3; ++col) {
      const double a = m[col * 4 + 0], b = m[col * 4 + 1], d = m[col * 4 + 2];
      const double l = std::sqrt(a * a + b * b + d * d);
      if (l > s)
        s = l;
    }
    // localRadius < 0 means "bounds unknown" (the mesh cache could not supply them) — treat
    // that draw as a point rather than inventing a size for it.
    const double r = draws[i].localRadius > 0.0f ? static_cast<double>(draws[i].localRadius) * s : 0.0;
    for (int k = 0; k < 3; ++k) {
      const double lo = c[k] - r, hi = c[k] + r;
      if (!any) {
        mn[k] = lo;
        mx[k] = hi;
      } else {
        if (lo < mn[k])
          mn[k] = lo;
        if (hi > mx[k])
          mx[k] = hi;
      }
    }
    any = true;
  }
  if (!any)
    return;
  // A zero-extent box draws nothing at all; give a degenerate selection a small cube so the
  // click still produces visible feedback rather than looking like a dead menu.
  for (int k = 0; k < 3; ++k)
    if (mx[k] - mn[k] < 1e-4) {
      const double mid = 0.5 * (mn[k] + mx[k]);
      mn[k] = mid - 0.05;
      mx[k] = mid + 0.05;
    }
  OmWgpuView::appendOverlayBox(out, mn, mx);
}

// W4a overlay telemetry. The geometry overlays are drawn by a SEPARATE pass
// (drawOverlayLines) that never touches the `draws` list, so draws= is BLIND to them --
// measured 2026-08-22, an overlays-on vs overlays-off A/B reported draws=1052 in BOTH arms
// and that identity meant nothing at all. These four fields are the actual instrument:
// ovCalled is whether the draw site fired (2 = reached it with zero geometry, which is a
// different failure from never reaching it), ovBatches/ovVerts is how much geometry was
// handed over, and ovOk is drawOverlayLines' own return (-1 = not called). They are written
// LATER in this same function than this report line, so each line carries the PREVIOUS
// frame's overlay state; on a settled world that is the same state.
static int sOvCalled = 0;
static qulonglong sOvBatches = 0, sOvVerts = 0;
static int sOvOk = -1;
// P7 HUD telemetry, same shape and same reason as the four above: the HUD quads are drawn by
// their OWN pass (drawOverlayQuads), which likewise never touches the `draws` list. hudCalled 2
// means the draw site was reached with zero quads -- a different failure from never reaching it.
static int sHudCalled = 0;
static qulonglong sHudQuads = 0;
static int sHudOk = -1;

// F1 (Phase D hazard #9): OMNISIM_WGPU_MAINVIEW_FORCE is RETIRED AND IGNORED. It used to drive
// the main view through wgpu on ANY world, bypassing the Viewpoint's renderBackend field -- a
// lever whose entire value was that the default resolution landed on WREN. Post-F1 every
// resolution lands on wgpu whenever wgpu-native is available, and when it is not the force never
// worked anyway (the lazy backend init fails either way), so the variable has NOTHING left to
// select on either arm -- which is why it is a warned no-op rather than value-parsed. One
// warning per process, mirroring warnRetiredWrenSelectors() in OmRenderBackend.cpp.
static void warnRetiredMainviewForce() {
  static const bool sWarned = []() {
    if (qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_FORCE"))
      OmLog::warning(QString("[OmView3D] OMNISIM_WGPU_MAINVIEW_FORCE is set but RETIRED and IGNORED: it used to "
                             "force the main view through wgpu regardless of the Viewpoint's renderBackend field. "
                             "WREN is retired and every resolution already lands on wgpu when wgpu-native is "
                             "available, so there is nothing left to force. Unset the variable."));
    return true;
  }();
  (void)sWarned;
}

bool OmView3D::renderMainFrameViaWgpu(bool culling, bool offScreen) {
  // R4 3c-B (increment 1b: the real wgpu main-view render), updated post-WREN-deletion (976b9449d,
  // 2026-08-23): render the live world OFFSCREEN via wgpu → RGBA, then blit that into this window's GL
  // framebuffer (offscreen render → GL blit; the HWND keeps its OpenGL pixel format, so there is no
  // surface-type conflict). This is now the ONLY main-view render path — WREN is deleted, so every
  // `return false` below means NO main-view frame is drawn this paint, not a fallback: a sticky failure
  // (mWgpuMainViewUnavailable) leaves the view permanently blank, and the transient guards (mid-reload,
  // suspended, offscreen/sensor/screenshot) just skip this paint.
  (void)culling;
  if (offScreen || mWgpuMainViewUnavailable || mWgpuMainViewSuspended)
    return false;  // offscreen/sensor/screenshot or mid-(re)load → stay on WREN; a prior failure → WREN
  // --no-rendering means exactly that. Until 2026-09-02 a `--batch --no-rendering --minimize` load
  // check still drew a full main-view frame on its first paint: it lazily initialised wgpu-native
  // (~300 ms on the reference machine) and kicked off the OmniLight CPU probe bake (11,200 probes,
  // ~450 ms of trace on husky_fleet_arena) -- for a run that never shows a pixel. Gate on the CLI
  // intent AND the live flag together, so a --no-rendering session that later turns rendering back
  // on (View > Rendering) renders normally; a screenshot request goes through the offscreen path
  // above and is unaffected. Camera/RangeFinder/Lidar DEVICES have their own wgpu path and still
  // render (they do not come through here).
  // OMNISIM_RENDERER_PROBE=1 (value-parsed) keeps the old behaviour -- a first frame is drawn even
  // under --no-rendering -- because that frame's lazy wgpu-native init is the only line a headless
  // smoke can grep to prove the BUILD has a renderer ("[OmWgpuBackend] wgpu-native init OK").
  // linux_bootstrap.sh's smoke sets it; the Linux CI went red the day the gate above shipped
  // without it (2026-09-02: "wgpu-native did not initialise -- this build has NO renderer").
  {
    static const bool rendererProbe = []() {
      const QByteArray v = qgetenv("OMNISIM_RENDERER_PROBE").trimmed().toLower();
      return !v.isEmpty() && v != "0" && v != "false" && v != "off";
    }();
    const OmSimulationState *const sim = OmSimulationState::instance();
    if (!rendererProbe && sim && sim->startedWithoutRendering() && !sim->isRendering())
      return false;
  }
  // Stale-world guard: during a reload the old world is freed before setWorld() updates mWorld, so a
  // paint event firing in that window would deref a dangling mWorld → crash (WREN dodges this by using
  // OmWorld::instance()). Skip the wgpu path whenever mWorld isn't the current global world; it resumes
  // once setWorld() re-syncs. (Pointer compare only — never dereferences the stale mWorld.)
  if (!mWorld || static_cast<const OmWorld *>(mWorld) != OmWorld::instance())
    return false;
  if (!mWorld->viewpoint())
    return false;
  OmViewpoint *const vp = mWorld->viewpoint();
  OmRenderBackend *const backend = vp->renderBackend();
  // F1 (Phase D hazard #9): OMNISIM_WGPU_MAINVIEW_FORCE is a RETIRED, warned no-op. It existed to
  // bypass the Viewpoint's renderBackend field on a stock (then WREN-default) world; post-F1 every
  // resolution already lands on wgpu whenever wgpu-native is available, and when it is NOT
  // available forcing wgpu was never possible (the lazy init below just failed) -- so the variable
  // selects nothing on either arm. Warned rather than silent so a soak harness exporting it learns
  // the lever is gone.
  warnRetiredMainviewForce();
  if (!backend || backend->kind() != OmRenderBackendKind::Vulkan || !backend->isAvailable())
    return false;  // wgpu unavailable → WREN last-resort path (deleted at D1.4)

  // 3c-B UN-GATED (2026-06-07): a Viewpoint that selects `renderBackend "wgpu"` now renders the main
  // view through wgpu directly — no experimental flag (the former OMNISIM_WGPU_MAINVIEW gate) required.
  // The sustained-use VRAM OOM that formerly gated this (the ~30 s "0xC0000409 after ~2000 frames"
  // fault) was an APP-LEVEL texture-cache key bug — shared-file textures re-uploaded once per
  // PROTO-instance — fixed by path-keying the cache (a4fec74b, OmWgpuSceneRenderer::stableTexId), and
  // verified by a 6-world sustained soak (75 s+, texture count plateaus, 0 wgpu errors).
  // HISTORICAL, corrected 2026-09-01: the rest of this comment described the pre-flip world --
  // Viewpoint.wrl now defaults `renderBackend` to "wgpu" (the 2026-08-19 default flip), WREN was
  // deleted on 2026-08-23 (976b9449d), and this branch is the ONLY main-view render path.
  // OMNISIM_WGPU_MAINVIEW_FORCE (above) is a RETIRED, warned no-op -- it forces nothing.

  // Lazily create the wgpu render resources the first time a Viewpoint selects wgpu.
  if (!mWgpuBackend) {
    mWgpuBackend = static_cast<OmVulkanBackend *>(OmRenderBackendRegistry::vulkanBackend());
    if (!mWgpuBackend || !mWgpuBackend->isAvailable()) {
      mWgpuMainViewUnavailable = true;
      // Loud, always-on signal (the OmVulkanBackend ctor already explains WHY on
      // stderr->omnisim_log.txt). Without this, a requested-but-unavailable wgpu
      // main view fell back to WREN with zero trace -- the "why is it still 0.4x?"
      // black box. stderr is captured into omnisim_log.txt even on the GUI binary.
      fprintf(stderr, "[OmView3D] wgpu main view requested (Viewpoint renderBackend resolution) but the wgpu "
                      "backend is unavailable -- using WREN. (See the [OmWgpuBackend] line above for why.)\n");
      fflush(stderr);
      return false;
    }
    mWgpuMeshCache = new OmWgpuMeshCache(mWgpuBackend);
    mWgpuTextureCache = new OmWgpuTextureCache(mWgpuBackend);
    OmLog::info(tr("[OmView3D] main view now rendering through the wgpu backend (renderBackend \"wgpu\")."));
  }

  // Camera from the live Viewpoint (same convention as OmWgpuView::buildViewpointCamera).
  if (!vp->position() || !vp->orientation())
    return false;
  const OmVector3 eye = vp->position()->value();
  const OmRotation rot = vp->orientation()->value();
  const OmVector3 fwd = rot.direction().normalized();
  const OmVector3 rgt = fwd.cross(rot.up()).normalized();
  const OmVector3 up = rgt.cross(fwd);
  const OmMatrix4 cam(fwd.x(), -rgt.x(), up.x(), eye.x(), fwd.y(), -rgt.y(), up.y(), eye.y(), fwd.z(),
                      -rgt.z(), up.z(), eye.z(), 0, 0, 0, 1);
  const double horizFov = vp->fieldOfView() ? vp->fieldOfView()->value() : 0.785;

  const int W = std::max(1, static_cast<int>(width() * devicePixelRatio()));
  const int H = std::max(1, static_cast<int>(height() * devicePixelRatio()));
  // Cache the offscreen target (recreate only on resize) — creating one PER FRAME leaks/exhausts GPU
  // resources and faulted after ~1k frames.
  if (!mWgpuRenderTarget || mWgpuRtWidth != W || mWgpuRtHeight != H) {
    delete mWgpuRenderTarget;
    mWgpuRenderTarget =
      new OmWgpuRenderTarget(mWgpuBackend, static_cast<uint32_t>(W), static_cast<uint32_t>(H));
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
  OmWgpuSceneRenderer::buildViewProj(cam, hf, aspect, zNear, 1000.0, vpm, revZ);
  std::vector<OmWgpuSolidDraw> &draws = mWgpuDrawList;
  std::vector<std::array<float, 16>> &modelStorage = mWgpuModelList;  // draws alias into this
  // ---- DYNAMIC CONTENT, part 1 of 2: drop LAST frame's appended tail. ----
  // Cloth / SoftBody / GranularGroup draws are re-derived every frame (their vertices or their
  // transforms move every step), so they are appended AFTER the cached scene draws and trimmed
  // here before anything reads the cache.
  // The trim must come before the validity check below, which compares modelStorage.size()
  // against mWgpuRefreshList.size() -- a leftover tail would make that mismatch and force a full
  // scene walk every single frame.
  if (mWgpuDynamicDraws > 0) {
    const size_t nTrim = std::min(mWgpuDynamicDraws, draws.size());
    draws.resize(draws.size() - nTrim);
    modelStorage.resize(modelStorage.size() - std::min(mWgpuDynamicDraws, modelStorage.size()));
    mWgpuDynamicDraws = 0;
    mWgpuDeformableDraws = 0;
    mWgpuGranularDraws = 0;
    mWgpuTrackDraws = 0;
    mWgpuMuscleDraws = 0;
  }
  // WREN-retirement W1c: the makeWrenCurrent()/doneWren() bracket that used to wrap the whole
  // collect is GONE. It existed because collectWorldDraws → acquireFromWren →
  // wr_static_mesh_read_data falls back to glGetBufferSubData, which returns garbage with no
  // current GL context — one bad read gets cached forever, so complex meshes stayed invisible.
  // That consumer still exists, but it is now the only one and it is reached only on a cache
  // MISS, so the collect arms the context itself (lazily, once, RAII-released on every path;
  // see OmWgpuSceneRenderer's glArmCount()). The arms-per-window number in the report below is
  // the measurement: once the mesh cache is warm it reads 0, which is what makes "the collect
  // no longer needs GL" a number rather than a claim.
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
  int wgpuCollectSkipped = 0;
  if (mWgpuDrawListDirty || mWgpuRefreshList.empty() || mWgpuDrawListAge >= 600 ||
      !OmWgpuSceneRenderer::refreshWorldDraws(modelStorage, mWgpuRefreshList, &draws)) {
    invalidateWgpuDrawList();
    // Rebuild point: the texture cache may release textures nothing used since the last rebuild;
    // the view-keyed bind groups must go with them (see OmWgpuTextureCache::evictStale).
    if (mWgpuTextureCache && mWgpuTextureCache->evictStale() && mWgpuRenderTarget)
      mWgpuRenderTarget->forgetTextureBindGroups();
    // No GL bracket here any more (W1c). collectWorldDraws arms WREN's context itself, only
    // around the readback that needs it, and releases it in a destructor — which is what the
    // old bracket's own warning demanded: makeWrenCurrent/doneWren PUSH/POP a context-state
    // stack, and an arm that is not released net-pushes one entry per frame, grows the stack
    // unbounded, corrupts WREN's context-active state and faults on world teardown (the
    // wgpu-specific reload crash). Nothing between here and the blit needs GL either — the
    // wgpu render is Vulkan and the sky/light harvest is CPU — and the blit brackets itself.
    OmWgpuSceneRenderer::collectWorldDraws(*mWgpuMeshCache, draws, modelStorage, &mWgpuDrawSolids,
                                           mWgpuTextureCache, &mWgpuRefreshList, &wgpuCollectSkipped);
    // A destroyed scene node would dangle the cached geom/WrTransform pointers — hook every
    // referenced node's destroyed() to invalidate the cache BEFORE the next frame can touch them.
    QSet<QObject *> hooked;
    for (const OmWgpuSceneRenderer::OmWgpuDrawRefresh &r : mWgpuRefreshList)
      if (r.node && !hooked.contains(r.node)) {
        hooked.insert(r.node);
        mWgpuDrawListConns.push_back(
          connect(r.node, &QObject::destroyed, this, [this]() { invalidateWgpuDrawList(); }));
      }
    // Structural-change hooks (replace the old 30-frame timer): top-level node additions (the
    // world root's children) and robot add/remove rebuild the list the moment they happen.
    // Qt::UniqueConnection dedupes across rebuilds.
    if (OmWorld::instance()) {
      if (OmGroup *root = OmWorld::instance()->root())
        connect(root, &OmGroup::childrenChanged, this, &OmView3D::invalidateWgpuDrawList,
                Qt::UniqueConnection);
      connect(OmWorld::instance(), &OmWorld::robotAdded, this, &OmView3D::invalidateWgpuDrawList,
              Qt::UniqueConnection);
      connect(OmWorld::instance(), &OmWorld::robotRemoved, this, &OmView3D::invalidateWgpuDrawList,
              Qt::UniqueConnection);
    }
    mWgpuDrawListDirty = false;
    // Self-healing collect: a Shape whose WREN mesh does not exist yet is silently dropped
    // above, and nothing structural fires when the mesh appears a few frames later — measured
    // on the launcher's patrol rover (a Robot's Pose-wrapped shapes), whose entire body was
    // collected as ZERO draws on frame 0 and stayed invisible for the whole session while the
    // engine drove it around the scene (draws=154 frozen, collectMs=0 forever). While a collect
    // reports skipped shapes, keep re-collecting (bounded, so a shape that legitimately never
    // gets a mesh cannot pin the walk cost forever — after the cap the 600-frame fallback owns it).
    if (wgpuCollectSkipped > 0 && mWgpuCollectRetries < 300) {
      ++mWgpuCollectRetries;
      mWgpuDrawListDirty = true;
      if (mWgpuCollectRetries == 1)
        OmLog::info(QString("[OmView3D] wgpu draw collect incomplete (%1 shape(s) lack WREN meshes) "
                            "-- re-collecting until the scene settles")
                      .arg(wgpuCollectSkipped));
    } else if (wgpuCollectSkipped == 0 && mWgpuCollectRetries > 0) {
      OmLog::info(QString("[OmView3D] wgpu draw collect settled after %1 retries (%2 draws)")
                    .arg(mWgpuCollectRetries)
                    .arg(static_cast<qulonglong>(draws.size())));
      mWgpuCollectRetries = 0;
    }
  } else
    ++mWgpuDrawListAge;
  // ---- DYNAMIC CONTENT, part 2 of 2: pump the readbacks and append this frame's draws. ----
  // Cloth / SoftBody / GranularGroup -- the node types collectWorldDraws structurally cannot
  // reach (none of them is an OmSolid) and whose geometry changes every step. ONE call, shared
  // with every sensor device (OmAbstractCamera::collectWgpuDraws), so a new dynamic node type is
  // wired once rather than at nine call sites -- see collectDynamicDraws' header note for the
  // regression that convention exists to prevent.
  // Cost on a world with none of them: two static bools plus an empty-test each; nothing walks
  // the scene to find out there is nothing to find. It ALSO owns the per-frame animateMesh() and
  // particle-readback pumps, which are normally driven from inside wr_scene_render -- a call this
  // path never makes.
  OmWgpuSceneRenderer::OmWgpuDynamicCounts dynCounts;
  mWgpuDynamicDraws = OmWgpuSceneRenderer::collectDynamicDraws(*mWgpuMeshCache, draws, modelStorage,
                                                               mWgpuTextureCache, nullptr, &dynCounts);
  mWgpuDeformableDraws = dynCounts.deformable;
  mWgpuGranularDraws = dynCounts.granular;
  mWgpuTrackDraws = dynCounts.track;
  mWgpuMuscleDraws = dynCounts.muscle;
  tCollect = phaseTimer.elapsed();
  if (tCollect > sMaxCollectMs)
    sMaxCollectMs = tCollect;  // worst collect in the window — the 30-frame rebuild hitch shows here

  // ===== W4a (WREN retirement): SELECTION FEEDBACK, part 1 of 2 — the tint. =====
  // Clicking a Solid produced NO visible change on the wgpu main view, because the selection
  // outline is a WREN scene node. The cheapest honest fix is not a new pass: lerp the selected
  // top solid's draws toward OmniSim mimosa in the cached draw list, exactly as the (offscreen-
  // verified) wgpu pane does. Part 2 — the outline box — rides the overlay pass further down.
  //
  // Cost model: this loop runs ONLY on the frame the selection actually changes; a frame where
  // it did not is one pointer compare. With nothing selected it is one pointer compare against
  // null, forever, and the draw list is byte-identical to the pre-W4a list.
  //
  // Hatch: OMNISIM_WGPU_OVERLAYS=0 disables every W4a overlay (tint, outline, and the optional-
  // rendering line pass) and restores the pre-W4a frame exactly. VALUE-parsed, default ON —
  // "set to anything" gating is the trap that makes FOO=0 mean ON.
  static const bool sOverlaysOn = !qEnvironmentVariableIsSet("OMNISIM_WGPU_OVERLAYS") ||
                                  qEnvironmentVariableIntValue("OMNISIM_WGPU_OVERLAYS") != 0;
  OmSolid *selTop = nullptr;
  if (sOverlaysOn && OmSelection::instance()) {
    selTop = OmSelection::instance()->selectedSolid();
    // collectWorldDraws tags every draw with its TOP solid, so highlight by top solid — that
    // is also what a bare click resolves to (see the pick handler's topMatter branch).
    while (selTop && selTop->upperSolid())
      selTop = selTop->upperSolid();
  }
  if (selTop != mWgpuSelTintTop) {
    // Undo the previous tint FIRST: the draw list outlives the selection.
    const size_t nSaved = std::min(mWgpuSelTintIdx.size(), mWgpuSelTintRgb.size());
    for (size_t i = 0; i < nSaved; ++i) {
      const uint32_t di = mWgpuSelTintIdx[i];
      if (di < draws.size()) {
        draws[di].baseColorR = mWgpuSelTintRgb[i][0];
        draws[di].baseColorG = mWgpuSelTintRgb[i][1];
        draws[di].baseColorB = mWgpuSelTintRgb[i][2];
      }
    }
    mWgpuSelTintIdx.clear();
    mWgpuSelTintRgb.clear();
    if (selTop) {
      const float hr = 0.96f, hg = 0.91f, hb = 0.02f, k = 0.55f;  // OmniSim mimosa, lerp weight
      const size_t nTag = std::min(draws.size(), mWgpuDrawSolids.size());
      for (size_t i = 0; i < nTag; ++i) {
        if (mWgpuDrawSolids[i] != selTop)
          continue;
        mWgpuSelTintIdx.push_back(static_cast<uint32_t>(i));
        mWgpuSelTintRgb.push_back(std::array<float, 3>{
          {draws[i].baseColorR, draws[i].baseColorG, draws[i].baseColorB}});
        draws[i].baseColorR = draws[i].baseColorR * (1.0f - k) + hr * k;
        draws[i].baseColorG = draws[i].baseColorG * (1.0f - k) + hg * k;
        draws[i].baseColorB = draws[i].baseColorB * (1.0f - k) + hb * k;
      }
    }
    mWgpuSelTintTop = selTop;
  }
  // Diagnostic (OMNISIM_WGPU_DRAW_DIAG=<index>): log one draw's CPU-side state every 100 frames
  // (model translation, local bounds, castShadows/translucent) — the first instrument to reach for
  // when a body renders wrong or not at all. Env-gated, zero cost when unset.
  if (qEnvironmentVariableIsSet("OMNISIM_WGPU_DRAW_DIAG")) {
    const int di = qEnvironmentVariableIntValue("OMNISIM_WGPU_DRAW_DIAG");
    static long sDiagF = 0;
    if ((sDiagF++ % 100) == 0 && di >= 0 && di < static_cast<int>(draws.size()) && draws[di].modelMatrix16) {
      const float *m = draws[di].modelMatrix16;
      OmLog::info(QString("[OmView3D] DIAG draw %1 model t=(%2, %3, %4) localC=(%5, %6, %7) localR=%8 "
                          "castShadows=%9 translucent=%10 alpha=%11")
                    .arg(di).arg(m[12]).arg(m[13]).arg(m[14])
                    .arg(draws[di].localCenter[0]).arg(draws[di].localCenter[1])
                    .arg(draws[di].localCenter[2]).arg(draws[di].localRadius)
                    .arg(draws[di].castShadows).arg(draws[di].translucent)
                    .arg(draws[di].baseColorA));
    }
  }
  if (mWgpuRgba.size() != static_cast<size_t>(W) * H * 4)
    mWgpuRgba.resize(static_cast<size_t>(W) * H * 4);  // overwritten in full by the readback
  std::vector<uint8_t> &rgba = mWgpuRgba;

  // ===== W2 (WREN retirement): PRESENTATION-FREE FRAME. =====
  // This function already renders into an OFFSCREEN target and only THEN presents, so the render
  // half was always headless-capable. What forced a desktop session was the presentation half:
  // a Vulkan-surface CHILD QWindow, or the readback + GL blit + swapBuffers fallback -- both of
  // which need a window that is actually on screen (Qt itself calls swapBuffers on a non-exposed
  // window "undefined behaviour"). When there is no exposed window there is nothing to present TO,
  // so skip both halves: no child window is created or shown, no surface is acquired, no GL
  // context is made current, no buffers are swapped. That is what lets --minimize / --no-window
  // sessions produce wgpu pixels at all -- the harness POST /world/screenshot, the
  // OMNISIM_WGPU_SYNTH_DUMP ground-truth pipeline, and any CI pixel gate.
  //
  // isExposed() is the same predicate WREN uses to decide it cannot draw (OmWrenWindow::renderNow
  // returns immediately on !isExposed() && !offScreen), so it is exactly "there is a real desktop
  // window" -- read from Qt rather than guessed from the command line.
  //
  // HONESTY: a frame rendered this way did NOT present, and mWgpuPresentedLastFrame is left FALSE
  // for it at the tail. The two conditions are deliberately separate variables so that no later
  // reader is told a frame reached the screen when it did not.
  //
  // COST: with nothing asking for pixels (no dump, no grab) the readback is skipped here exactly
  // as it is in present mode, so an unexposed frame costs the render and nothing else -- it does
  // not silently add a full-frame GPU->CPU copy per step to a headless run.
  //
  // Hatch: OMNISIM_WGPU_HEADLESS=1 OPTS IN. VALUE-parsed; DEFAULT OFF, and that default is a
  // MEASURED decision, not caution.
  //
  // ⚠ WHY THIS IS NOT ON BY DEFAULT. `!isExposed()` is not a reliable proxy for "there is no
  // desktop window": Qt also reports a real, realised window as unexposed when it is not the
  // foreground window -- which is the normal case for any engine launched from a script. When
  // that happens this branch skips presentation, and with nothing presenting Qt stops scheduling
  // paint events, so THE FRAME COUNTER STOPS ADVANCING and the view renders one frame and stalls.
  // Measured 2026-08-22 (machine 9722d23d12a3) on construction_site_dev.omniworld, windowed
  // --mode=realtime, 50 s per arm: default-on reached frame 0 only, while OMNISIM_WGPU_HEADLESS=0
  // reached frame 1000+ at renderMs 11-12. Every scripted render (render_ab.py, the MAINVIEW
  // dump, the synth dump) launches exactly that kind of non-foreground window, so shipping this
  // on would have broken all of them.
  //
  // The W2 GOAL is still right and the rest of the machinery below is kept: a frame that does not
  // present must still render, so headless screenshots become possible. What is missing is an
  // honest trigger -- "this process has no display at all", not "this window is not on top" --
  // plus a repaint driver that does not depend on presentation. Finish those before flipping.
  static const bool sWgpuHeadlessOk = qEnvironmentVariableIsSet("OMNISIM_WGPU_HEADLESS") &&
                                      qEnvironmentVariableIntValue("OMNISIM_WGPU_HEADLESS") != 0;
  const bool headlessFrame = mWgpuOffscreenOnly || (sWgpuHeadlessOk && !isExposed());
  if (headlessFrame && (width() <= 1 || height() <= 1)) {
    // A window that was never realised has no size, so W/H clamp to 1 and every headless frame
    // (and every screenshot taken from one) is a 1x1 image. Say so once rather than shipping a
    // one-pixel PNG silently. INFO, not WARNING, so it cannot flip a --fail-on-warning run.
    static bool sWarnedDegenerateSize = false;
    if (!sWarnedDegenerateSize) {
      sWarnedDegenerateSize = true;
      OmLog::info(tr("[OmView3D] the wgpu main view is rendering with no exposed window and the 3D view "
                     "has no usable size (%1x%2), so headless frames and screenshots will be degenerate. "
                     "Use --minimize (which realises the window) rather than --no-window when pixels matter.")
                    .arg(width())
                    .arg(height()));
    }
  }

  // Window-swap presentation: lazily create an input-transparent Vulkan-surface CHILD window over
  // this view; when usable, frames present GPU→GPU (presentTexture samples the offscreen texture)
  // and the readback below is SKIPPED entirely (rgba=nullptr). Mouse/keyboard pass through to this
  // view, so the full editor interaction is preserved. Any failure → the legacy blit path.
  // Skipped entirely on a presentation-free frame (above): creating and show()ing a child QWindow
  // is precisely the desktop-session requirement W2 removes.
  if (!headlessFrame) {
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
      // Native handles for whatever windowing system is actually running: HWND on
      // Windows, Xlib/XCB on an X11 session, wl_surface on Wayland. An invalid
      // handle means this platform has no wgpu surface source — keep the readback
      // + GL-blit path below. The resolver (gui/OmWgpuView.cpp) logs the reason
      // once per process, so re-attempting on later frames costs no log spam.
      const OmWgpuNativeWindow nativeWindow = OmWgpuNativeWindowFromQtWindow(mWgpuPresentWindow);
      if (nativeWindow.isValid()) {
        mWgpuPresentSurface = new OmWgpuSurface(mWgpuBackend, nativeWindow, static_cast<uint32_t>(W),
                                                static_cast<uint32_t>(H));
        if (!mWgpuPresentSurface->isUsable()) {
          delete mWgpuPresentSurface;
          mWgpuPresentSurface = nullptr;  // blit fallback (no retry churn: window handle was valid)
        }
      }
    }
  }
  // OMNISIM_WGPU_NO_SWAP=1: kill-switch back to the readback+blit path (visual safety valve).
  const bool present = !headlessFrame && mWgpuPresentSurface != nullptr &&
                       !qEnvironmentVariableIsSet("OMNISIM_WGPU_NO_SWAP");
  if (present)
    mWgpuPresentSurface->resize(static_cast<uint32_t>(W), static_cast<uint32_t>(H));
  const bool wantDump = qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_DUMP") ||
                        qEnvironmentVariableIsSet("OMNISIM_WGPU_SYNTH_DUMP");
  // A pixel grab (screenshot/thumbnail/stream frame) forces the readback even in present mode,
  // synchronously (asyncReadback would deliver frame N-1, which may not exist yet).
  // A presentation-free frame is read back on the same terms: nobody needs the pixels unless a
  // dump or a grab asked for them, and when one did they must be THIS frame's (see asyncReadback
  // at the render call). Without this, an unexposed run would pay a full-frame readback per step
  // purely because there is no surface to present to.
  void *rgbaOut =
    ((present || headlessFrame) && !wantDump && !mWgpuGrabRequested) ? nullptr : static_cast<void *>(rgba.data());
  // World-general sky tint: harvest the scene's Background skyColor (the sky the viewer sees), used
  // BOTH as the clear colour AND as the hemisphere-IBL ambient. Falls back to a neutral sky-blue when
  // there's no Background, or it's near-black (TexturedBackground / NightSky) — so shadows still get a
  // small fill rather than crushing. This replaces the panda-tuned hardcoded clear + the "ambient 0"
  // that made every non-panda world render with crushed-black shadows.
  float skyC[3] = {0.45f, 0.62f, 0.85f};
  OmBackground *bgNode = nullptr;
  if (mWorld && mWorld->root())
    if (OmBackground *bg = findFirstBackgroundV3D(mWorld->root())) {
      bgNode = bg;
      const OmRgb sc = bg->skyColor();
      const float r = static_cast<float>(sc.red()), g = static_cast<float>(sc.green()),
                  b = static_cast<float>(sc.blue());
      if (r + g + b > 0.05f) {  // keep the default for an unset/black sky
        skyC[0] = r;
        skyC[1] = g;
        skyC[2] = b;
      }
    }
  // Sky-backdrop selection, mirroring OmBackground::applySkyBoxToWren's priority: recognised
  // atmosphericSky preset > complete image cubemap (NightSky-class worlds) > flat skyColor clear.
  // The dome used to key on "a DirectionalLight exists", which was wrong in both directions: a
  // cubemap-only world with a sun got the procedural dome, and an atmosphericSky world with no
  // sun got a flat clear.
  const bool skyAtmo = bgNode && !bgNode->atmosphericSkyPreset().isEmpty();
  bool skyCube = !skyAtmo && bgNode && bgNode->hasCompleteCubemap();
  if (skyCube && mWgpuRenderTarget) {
    // Upload once per (render target, background node): the target is recreated on resize/world
    // load, and a different Background instance means different faces.
    if (mWgpuSkyCubeUploadedFor != static_cast<void *>(bgNode) || !mWgpuRenderTarget->hasSkyCubemap()) {
      const QImage *faces[6];
      for (int i = 0; i < 6; ++i)
        faces[i] = bgNode->cubemapTexture(i);
      if (mWgpuRenderTarget->setSkyCubemap(faces))
        mWgpuSkyCubeUploadedFor = bgNode;
      else
        skyCube = false;  // upload failed → fall back to the dome/flat-clear path
    }
  }
  OmWgpuClearColor sky;
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
    const OmVector3 up = -mWorld->worldInfo()->gravityUnitVector();  // up = opposite gravity
    if (up.length() > 1e-6) {
      worldUp3[0] = static_cast<float>(up.x());
      worldUp3[1] = static_cast<float>(up.y());
      worldUp3[2] = static_cast<float>(up.z());
    }
  }
  // R4 3c-B: light the wgpu main view with the REAL scene sun (the OmniSimSun PROTO's
  // OmDirectionalLight) + cast shadows, so the live wgpu view is shadow-dominated like WREN instead of
  // flat-lit. Mirrors the parity self-check's sun harvest + light-frustum + shadowed render. Ambient is
  // kept small (WREN renders this scene shadow-dominated); the finer sun-shaft shadow-placement parity
  // is a follow-up. Falls back to a hardcoded direction if no directional light is found.
  float lit4[4] = {0.3f, 0.4f, -0.85f, 0.05f};
  float sunColor3[3] = {1.0f, 1.0f, 1.0f};
  // Real sun energy (colour x intensity, zeroed when the sun is off) for the HDR linear-light arm.
  // The LDR arm never reads it (shader legacy path). Fallback 2.5 = OmniSimSun's default intensity,
  // so sun-less worlds get a plausible key light level under HDR.
  float sunEnergy3[3] = {2.5f, 2.5f, 2.5f};
  bool haveSun = false;
  OmDirectionalLight *sunNode = nullptr;
  if (mWorld && mWorld->root())
    if (OmDirectionalLight *sun = findFirstDirectionalLightV3D(mWorld->root())) {
      const OmVector3 sd = sun->direction().normalized();
      lit4[0] = static_cast<float>(sd.x());
      lit4[1] = static_cast<float>(sd.y());
      lit4[2] = static_cast<float>(sd.z());
      const OmRgb sc = sun->color();
      sunColor3[0] = static_cast<float>(sc.red());
      sunColor3[1] = static_cast<float>(sc.green());
      sunColor3[2] = static_cast<float>(sc.blue());
      const float e = sun->isOn() ? static_cast<float>(sun->intensity()) : 0.0f;
      sunEnergy3[0] = sunColor3[0] * e;
      sunEnergy3[1] = sunColor3[1] * e;
      sunEnergy3[2] = sunColor3[2] * e;
      haveSun = true;
      sunNode = sun;
    }
  // Multi-light: harvest every additional light (point/spot/extra directionals) as unshadowed
  // fills for the shader's ExtraLight array. Empty (the single-sun common case) leaves the shader
  // loop dead and the output unchanged.
  std::vector<float> extraLights;
  if (mWorld && mWorld->root())
    collectExtraLightsV3D(mWorld->root(), sunNode, extraLights);
  // HDR + AgX filmic tonemapping: ON BY DEFAULT (exposure from the Viewpoint's exposure field,
  // default 1.0). The historical "AgX reads milky/blown" verdict was diagnosed 2026-08-19: the
  // shading underneath was DISPLAY-referred (raw albedo, unit-energy sun, pow(1/2.2) inside the
  // scene shader), so the filmic transform re-compressed an already-final image. The scene shader
  // now carries a linear-light arm (hdrMode via LightU groundColor.w): sRGB-decoded albedo, real
  // sun energy (colour x intensity via extraMeta.yzw), linear ambient/emissive, no in-shader
  // encode — the tonemap pass owns the display transform (and the dither).
  // OMNISIM_WGPU_AGX=<exposure> is now the OVERRIDE + kill-switch: value-parsed, so =0 selects the
  // untouched LDR path bit-exactly (never presence-gate this — the OMNISIM_REQUIRE_NEWTON trap).
  float agxExposure = 1.0f;
  if (vp->exposure() && vp->exposure()->value() > 0.0)
    agxExposure = static_cast<float>(vp->exposure()->value());
  if (qEnvironmentVariableIsSet("OMNISIM_WGPU_AGX"))
    agxExposure = qEnvironmentVariable("OMNISIM_WGPU_AGX").toFloat();
  static bool sHdrModeLogged = false;
  if (!sHdrModeLogged) {
    sHdrModeLogged = true;
    OmLog::info(QString("[OmView3D] wgpu main view tonemap: %1 (exposure %2)")
                  .arg(agxExposure > 0.0f ? "HDR linear-light + AgX" : "legacy LDR (OMNISIM_WGPU_AGX=0)")
                  .arg(agxExposure));
  }
  // SSAO: enabled when the Viewpoint authors a positive ambientOcclusionRadius (the city sets 2).
  // Contact darkening at building bases / under cars — the depth/weight cue WREN's GTAO provides.
  float ssaoStrength = 0.0f;
  float ssaoRadiusScale = 1.0f;
  if (vp->ambientOcclusionRadiusField() && vp->ambientOcclusionRadiusField()->value() > 0.0 &&
      !qEnvironmentVariableIsSet("OMNISIM_WGPU_NO_SSAO")) {  // diagnostic kill-switch
    ssaoStrength = 1.0f;
    // Honor the authored radius (not just as an on/off gate): normalized so the default radius 2
    // keeps the tuned kernel exactly, clamped so extreme authored values stay stable.
    const double r = vp->ambientOcclusionRadiusField()->value();
    ssaoRadiusScale = static_cast<float>(std::min(4.0, std::max(0.25, r / 2.0)));
  }
  // Bloom: enabled when the Viewpoint authors a non-negative bloomThreshold (the city sets 6;
  // -1 disables). The HDR threshold maps to an LDR composite strength — lower thresholds bloom
  // stronger, anchored so the city's 6 → 0.55 and the OmniSim default 21 → subtle.
  float bloomStrength = 0.0f;
  if (vp->bloomThresholdField() && vp->bloomThresholdField()->value() >= 0.0) {
    const double bt = vp->bloomThresholdField()->value();
    bloomStrength = static_cast<float>(std::min(0.8, 0.55 * (6.0 / (bt > 1.0 ? bt : 1.0))));
  }
  // Fog: the world's Fog node (the city ships one — exponential, 420 m, pale blue), shaded
  // per-pixel in the wgpu scene shader. Density 3/visibilityRange ≈ 95% fogged at the authored
  // range, approximating WREN's GL exponential fog. Absent node → density 0 → off.
  float fogParams4[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  if (OmFog *fog = OmFog::fogInstance()) {
    if (fog->fogColor() && fog->fogVisibilityRange() && fog->fogVisibilityRange()->value() > 0.1) {
      const OmRgb fc = fog->fogColor()->value();
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
  const OmVector3 sdir = OmVector3(lit4[0], lit4[1], lit4[2]).normalized();
  OmVector3 lightAxis = OmVector3(1.0, 0.0, 0.0).cross(sdir);
  const double laLen = lightAxis.length();
  const double laAng = std::acos(std::max(-1.0, std::min(1.0, OmVector3(1.0, 0.0, 0.0).dot(sdir))));
  lightAxis = laLen > 1e-6 ? lightAxis / laLen : OmVector3(0.0, 1.0, 0.0);
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
  OmVector3 anchor;
  if (fwd.z() < -1e-3) {
    const double t = std::min((eye.z() - 0.4) / -fwd.z(), 400.0);
    anchor = eye + fwd * t;
  } else {
    OmVector3 fwdH(fwd.x(), fwd.y(), 0.0);
    fwdH = fwdH.length() > 1e-3 ? fwdH.normalized() : OmVector3(1.0, 0.0, 0.0);
    anchor = eye + fwdH * (kShadowHalfExtent * 0.6);
  }
  const double texelWorld = 2.0 * kShadowHalfExtent / std::max(1, std::min(W, H));
  anchor = OmVector3(std::floor(anchor.x() / texelWorld) * texelWorld,
                     std::floor(anchor.y() / texelWorld) * texelWorld, 0.4);
  const OmVector3 lightPos = anchor - sdir * 130.0;
  const OmMatrix4 lightWorld(lightPos.x(), lightPos.y(), lightPos.z(), lightAxis.x(), lightAxis.y(),
                             lightAxis.z(), laAng);
  float lightVP[16] = {0};
  OmWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, kShadowHalfExtent, 0.05, 260.0, lightVP);
  // R4 3c-B un-gate leak-hunt: instrument the main-view path. Trace draw count + W/H + the render
  // result + the wgpu resource-registry report every 100 frames, BEFORE the failure-latch, so a
  // sustained soak shows whether the registry climbs toward the historical VRAM OOM. Gated by
  // OMNISIM_WGPU_REPORT=<file>; inert otherwise (WREN/default path byte-identical). Diagnostic only.
  static long sWgpuMainViewFrame = 0;
  const long f = sWgpuMainViewFrame++;
  // R4 3c-B (L3↔L2 wiring): OMNISIM_WGPU_MAINVIEW_CSM (default-off) routes the main view through the
  // FULL-MATERIAL multi-cascade shadow path (OmWgpuRenderTarget::clearAndDrawSceneTexturedCsm) — the
  // same material path (albedo/roughness/metalness/normal + GGX) as the default textured-shadow render,
  // but with N per-camera-frustum cascades (buildCascadeLightViewProjs over the [0.05, 40] shadow range)
  // instead of the single fixed-extent ortho light frustum, so near shadows are tighter. This is the
  // candidate for the eventual main-view default; default-off keeps the single-cascade textured-shadow
  // render — and the panda parity golden — byte-identical until the flip is human-gated.
  bool cdOk;
  // Cascaded shadow maps are the DEFAULT (merged into the one textured-shadow render — HDR, sky,
  // transparency, GTAO all included). NC=3 fitted cascades over [zNear, 60 m], lambda 0.6.
  // OMNISIM_WGPU_NO_CSM=1 (value-parsed) reverts to the legacy single 45 m fitted map.
  // OMNISIM_WGPU_MAINVIEW_CSM (the old dead-end LDR path's lever) is retired.
  uint32_t csmCount = 0;
  float csmVps[OmWgpuSceneRenderer::kMaxCascades * 16] = {0};
  float csmSplits4[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  if (qEnvironmentVariableIntValue("OMNISIM_WGPU_NO_CSM") != 1) {
    const int NC = 3;
    float splits[OmWgpuSceneRenderer::kMaxCascades + 1] = {0};
    OmWgpuSceneRenderer::buildCascadeLightViewProjs(cam, hf, aspect, zNear, 60.0, lightWorld, NC, 0.6,
                                                    csmVps, splits);
    for (int ci = 0; ci < NC && ci < 4; ++ci)
      csmSplits4[ci] = splits[ci + 1];
    csmCount = static_cast<uint32_t>(NC);
  }
  {
    static bool sCsmEnvWarned = false;
    if (!sCsmEnvWarned && qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_CSM")) {
      sCsmEnvWarned = true;
      OmLog::info("[OmView3D] OMNISIM_WGPU_MAINVIEW_CSM is retired: cascades are the default now "
                  "(OMNISIM_WGPU_NO_CSM=1 reverts to the single-map path)");
    }
  }
  // Synthetic-data dump: decide NOW whether this frame dumps, because the main render below must
  // then read back SYNCHRONOUSLY — the pipelined async readback leaves the shared readback buffer
  // mid-map when the ground-truth passes run right after the frame (wgpu validation abort).
  const bool wantSynthEnv = qEnvironmentVariableIsSet("OMNISIM_WGPU_SYNTH_DUMP");
  const long synthDumpFrame = qEnvironmentVariableIsSet("OMNISIM_WGPU_MAINVIEW_DUMP_FRAME")
                                ? static_cast<long>(qEnvironmentVariableIntValue("OMNISIM_WGPU_MAINVIEW_DUMP_FRAME"))
                                : 200;
  const long synthEveryN = qEnvironmentVariableIsSet("OMNISIM_WGPU_SYNTH_EVERY")
                             ? static_cast<long>(qEnvironmentVariableIntValue("OMNISIM_WGPU_SYNTH_EVERY"))
                             : 0;
  const bool synthThisFrame =
    wantSynthEnv && (f == synthDumpFrame ||
                     (synthEveryN > 0 && f > synthDumpFrame && (f - synthDumpFrame) % synthEveryN == 0));
  // Mirrors of sky-block locals for the synthetic-data dump (declared at function scope).
  float synthDayF = 1.0f;
  bool synthScatter = false;
  float synthCloud = 0.0f;
  {
    // Atmospheric sky + day-night (the wgpu counterpart of Background.atmosphericSky + the
    // sun_marker system): SkyU = camera basis scaled by the half-FOV tangents (per-pixel ray
    // reconstruction), TOWARD-sun, the light's colour, world up. The day factor — sun elevation
    // smoothed through twilight — also dims the scene's DIRECT term, so dragging the sun marker
    // below the horizon darkens geometry in step with the dome. Full day → directScale 1.0 →
    // the gate-passing render is unchanged.
    const double halfW = std::tan(0.5 * hf);
    const double halfH = halfW / aspect;
    const OmVector3 towardSun(-lit4[0], -lit4[1], -lit4[2]);
    const OmVector3 upWorld(worldUp3[0], worldUp3[1], worldUp3[2]);
    const double sunElev = towardSun.normalized().dot(upWorld.normalized());
    const float dayF = static_cast<float>(std::max(0.05, std::min(1.0, sunElev * 5.0 + 0.5)));
    // PHYSICALLY-SCATTERED SKY (the realism lever, 2026-08-20): when the world declares an
    // atmosphericSky preset and the HDR pipeline is on, the dome samples a Hillaire-class
    // single-scatter sky-view LUT instead of the hand-tuned palette. The LUT (128x64) is
    // re-marched ONLY when the sun moves — steady-state per-frame cost is a memcmp — so the
    // renderMs budget is untouched. OMNISIM_WGPU_SKY_SCATTER=0 (value-parsed) reverts to the
    // palette dome; OMNISIM_WGPU_SKY_ILLUM recalibrates the sun illuminance (A/B knob).
    static const bool sSkyScatterOn = !qEnvironmentVariableIsSet("OMNISIM_WGPU_SKY_SCATTER") ||
                                      qEnvironmentVariableIntValue("OMNISIM_WGPU_SKY_SCATTER") != 0;
    const bool skyScatter = skyAtmo && !skyCube && agxExposure > 0.0f && sSkyScatterOn;
    synthScatter = skyScatter;
    // Atmosphere presets: constants mirror OmWrenAtmosphericSky::AtmosphereParameters (the WREN
    // Hillaire path's source of truth) — Bruneton/Hillaire published values, per-METRE coefficients.
    const bool skyMars = skyAtmo && bgNode->atmosphericSkyPreset() == QStringLiteral("mars");
    static const float sSkyIllumScale = []() {
      bool ok = false;
      const float v = qEnvironmentVariable("OMNISIM_WGPU_SKY_ILLUM").toFloat(&ok);
      return ok && v > 0.0f ? v : 22.0f;
    }();
    // Cloud layer: OMNISIM_WGPU_CLOUDS=0 disables; OMNISIM_WGPU_CLOUD_COVER sets coverage
    // (default 0.4). Rides SkyU fwd.w; scatter mode only (the LDR palette dome stays cloudless).
    static const float sCloudCover = []() {
      if (qEnvironmentVariableIsSet("OMNISIM_WGPU_CLOUDS") &&
          qEnvironmentVariableIntValue("OMNISIM_WGPU_CLOUDS") == 0)
        return 0.0f;
      bool ok = false;
      const float v = qEnvironmentVariable("OMNISIM_WGPU_CLOUD_COVER").toFloat(&ok);
      return ok && v >= 0.0f && v <= 1.0f ? v : 0.4f;
    }();
    const float illumR = skyMars ? sSkyIllumScale * (10.0f / 22.0f) : sSkyIllumScale;
    const float illumG = skyMars ? sSkyIllumScale * (8.5f / 22.0f) : sSkyIllumScale;
    const float illumB = skyMars ? sSkyIllumScale * (7.0f / 22.0f) : sSkyIllumScale;
    synthDayF = dayF;
    synthCloud = skyScatter ? sCloudCover : 0.0f;
    const float skyU[28] = {
      static_cast<float>(rgt.x() * halfW), static_cast<float>(rgt.y() * halfW),
      static_cast<float>(rgt.z() * halfW), skyScatter ? 1.0f : 0.0f,  // right.w = scatter mode
      static_cast<float>(up.x() * halfH), static_cast<float>(up.y() * halfH),
      static_cast<float>(up.z() * halfH), 0.0f,
      static_cast<float>(fwd.x()), static_cast<float>(fwd.y()), static_cast<float>(fwd.z()),
      skyScatter ? sCloudCover : 0.0f,  // fwd.w = cloud coverage
      static_cast<float>(towardSun.x()), static_cast<float>(towardSun.y()),
      static_cast<float>(towardSun.z()), 0.0f,
      sunColor3[0], sunColor3[1], sunColor3[2],
      agxExposure > 0.0f ? 1.0f : 0.0f,  // sunColor.w = HDR linear-radiance sky arm
      worldUp3[0], worldUp3[1], worldUp3[2], skyCube ? 1.0f : 0.0f,  // worldUp.w = sky mode (1 = image cubemap)
      illumR, illumG, illumB, 0.99995f};  // sunIll + sun disc cos radius (WREN's discCos)
    float scat24[24] = {0};
    if (skyScatter) {
      const float sinSunElev = static_cast<float>(sunElev);
      if (skyMars) {
        const float mars[24] = {19.918e-7f, 13.57e-7f, 5.75e-7f, -1.0f / 11.1f,
                                2.0e-5f, 1.7e-5f, 1.4e-5f, -1.0f / 11.0f,
                                1.0e-5f, 1.2e-5f, 1.5e-5f, 0.76f,
                                0.15e-6f, 0.3e-6f, 0.85e-6f, 0.1f,
                                3389.5f, 3489.5f, sinSunElev, 0.15f,
                                illumR, illumG, illumB, 0.0f};
        std::memcpy(scat24, mars, sizeof(mars));
      } else {
        const float earth[24] = {5.802e-6f, 13.558e-6f, 33.1e-6f, -1.0f / 8.0f,
                                 3.996e-6f, 3.996e-6f, 3.996e-6f, -1.0f / 1.2f,
                                 4.4e-6f, 4.4e-6f, 4.4e-6f, 0.8f,
                                 0.65e-6f, 1.881e-6f, 0.085e-6f, 0.1f,
                                 6360.0f, 6460.0f, sinSunElev, 0.30f,
                                 illumR, illumG, illumB, 0.0f};
        std::memcpy(scat24, earth, sizeof(earth));
      }
    }
    // DERIVED LIGHTING (phase 2 of the scattered sky): the hemisphere ambient + the analytic-IBL
    // env palette come from a CPU march of the SAME atmosphere (16/4 steps, 5 directions),
    // re-evaluated only when the sun/preset/illum changed. The scene's fill light and the metals'
    // reflections now track the sky — warm at sunset, blue at noon — instead of fixed palettes.
    // OMNISIM_WGPU_SKY_DERIVED=0 (value-parsed) keeps the sky but reverts ambient/IBL.
    static const bool sSkyDerivedOn = !qEnvironmentVariableIsSet("OMNISIM_WGPU_SKY_DERIVED") ||
                                      qEnvironmentVariableIntValue("OMNISIM_WGPU_SKY_DERIVED") != 0;
    const bool skyDerived = skyScatter && sSkyDerivedOn;
    float iblSky8[8] = {0};
    float hemiSkyDrv4[4] = {0, 0, 0, 0.45f};
    float hemiGroundDrv4[4] = {0, 0, 0, 0.0f};
    if (skyDerived) {
      static float sDrvKey[24] = {0};
      static float sDrvZen[3], sDrvHor[3], sDrvHemi[3], sDrvGround[3];
      if (std::memcmp(sDrvKey, scat24, sizeof(sDrvKey)) != 0) {
        SkyMarchP mp;
        mp.rayR = scat24[0]; mp.rayG = scat24[1]; mp.rayB = scat24[2]; mp.rayExp = scat24[3];
        mp.mieS = scat24[4]; mp.mieExp = scat24[7];
        mp.mieA = scat24[8]; mp.phaseG = scat24[11];
        mp.ozR = scat24[12]; mp.ozG = scat24[13]; mp.ozB = scat24[14]; mp.camHkm = scat24[15];
        mp.botKm = scat24[16]; mp.topKm = scat24[17]; mp.sinSunElev = scat24[18];
        mp.albedo = scat24[19];
        mp.illR = scat24[20]; mp.illG = scat24[21]; mp.illB = scat24[22];
        float zen[3], hor0[3], hor90[3], hor180[3];
        sciSampleSky(mp, 1.35, 0.0, zen);              // near-zenith
        sciSampleSky(mp, 0.30, 0.0, hor0);             // mid-sky toward the sun
        sciSampleSky(mp, 0.30, M_PI / 2.0, hor90);     // mid-sky, side
        sciSampleSky(mp, 0.30, M_PI, hor180);          // mid-sky, away
        for (int k = 0; k < 3; ++k) {
          sDrvZen[k] = zen[k];
          sDrvHor[k] = 0.25f * (hor0[k] + 2.0f * hor90[k] + hor180[k]);
          sDrvHemi[k] = 0.35f * zen[k] + 0.65f * sDrvHor[k];
          // Ground bounce: transmitted sun on a Lambertian ground, dimmed toward the ambient level.
          const float sunUp = std::max(0.0f, scat24[18]);
          sDrvGround[k] = (mp.albedo / static_cast<float>(M_PI)) * scat24[20 + k] * sunUp * 0.5f;
        }
        std::memcpy(sDrvKey, scat24, sizeof(sDrvKey));
      }
      // kSkyAmbScale maps sky RADIANCE to the calibrated ambient level (the old display floors sat
      // near 0.1 linear); the shader's hdr arm decodes pow(2.2) and applies its 0.7 constant.
      const float kSkyAmbScale = 0.25f;
      for (int k = 0; k < 3; ++k) {
        hemiSkyDrv4[k] = std::pow(std::max(0.0f, sDrvHemi[k] * kSkyAmbScale), 1.0f / 2.2f);
        hemiGroundDrv4[k] = std::pow(std::max(0.0f, sDrvGround[k] * kSkyAmbScale), 1.0f / 2.2f);
        iblSky8[k] = sDrvZen[k];
        iblSky8[4 + k] = sDrvHor[k];
      }
      iblSky8[3] = 1.0f;  // valid flag
    }
    // Real-time extra lights, crossfaded: once an OmniLight volume with baked locals is live,
    // the point/spot records fade out by the same blend the traced field fades in with — their
    // light now arrives through the volume, occluded and bounced. Extra DIRECTIONALS (rare)
    // always stay real-time.
    std::vector<float> omniExtras = extraLights;
    {
      const float rtScale = (mOmniEverApplied && mOmniLocalsBaked) ? (1.0f - mOmniBlend) : 1.0f;
      if (rtScale < 1.0f)
        for (size_t li = 0; li + 15 < omniExtras.size(); li += 16)
          if (omniExtras[li + 3] > 0.5f)  // point/spot only
            for (int k = 4; k < 7; ++k)
              omniExtras[li + k] *= rtScale;
    }
    // ===== OMNILIGHT: baked global illumination (the sky-LUT philosophy at full scale) =====
    // Trace the light COMPLETELY (CPU path tracer over the real scene triangles, real sky
    // radiance, sun shadow rays, emissive bounce) but only when the LIGHT RIG or the SCENE
    // changes; per frame the shader pays one trilinear volume sample. OMNILIGHT=0 (value-parsed)
    // disables; OMNILIGHT_RAYS tunes quality (default 256/probe).
    static const bool sOmniOn = !qEnvironmentVariableIsSet("OMNILIGHT") ||
                                qEnvironmentVariableIntValue("OMNILIGHT") != 0;
    // apply a finished bake
    if (mOmniBakeRunning && mOmniBakeDone.load(std::memory_order_acquire)) {
      if (mOmniBakeThread.joinable())
        mOmniBakeThread.join();
      mOmniBakeRunning = false;
      if (mOmniResult && mOmniResult->valid) {
        const OmniLightVolume &v = *mOmniResult;
        float org[3], invExt[3];
        for (int k = 0; k < 3; ++k) {
          org[k] = v.origin[k] - 0.5f * v.spacing[k];
          invExt[k] = 1.0f / (v.dims[k] * v.spacing[k]);
        }
        mWgpuRenderTarget->setOmniLightVolume(v.texels.data(), v.dims[0], v.dims[1], v.dims[2],
                                              org, invExt);
        if (!v.cubeTexels.empty())
          mWgpuRenderTarget->setOmniLightCube(v.cubeTexels.data(), v.cubeSize, v.cubeCenter,
                                              v.aabbMin, v.aabbMax);
        // First landing fades in over ~0.6 s (no pop); later rebakes swap directly (both sides
        // are lit states, the delta is small).
        if (!mOmniEverApplied) {
          mOmniBlend = 0.0f;
          mOmniEverApplied = true;
        }
        OmLog::info(QString("[OmniLight] probe volume live: %1x%2x%3 (%4 probes, %5 valid), "
                            "%6 tris, BVH %7 ms, trace %8 ms")
                      .arg(v.dims[0]).arg(v.dims[1]).arg(v.dims[2])
                      .arg(v.probeCount).arg(v.validProbes)
                      .arg(static_cast<qulonglong>(v.triangleCount))
                      .arg(v.bvhSeconds * 1000.0, 0, 'f', 0)
                      .arg(v.bakeSeconds * 1000.0, 0, 'f', 0));
      }
      mOmniResult.reset();
    }
    // trigger: light-rig + scene-fingerprint key (quantized sun so day-night cycling rebakes
    // in steps, not every frame; moving robots do NOT retrigger — static-scene GI by design)
    if (sOmniOn && skyScatter && !mOmniBakeRunning && !draws.empty()) {
      uint64_t key = 1469598103934665603ULL;
      auto mix = [&key](int64_t v) { key = (key ^ static_cast<uint64_t>(v)) * 1099511628211ULL; };
      for (int k = 0; k < 3; ++k)
        mix(static_cast<int64_t>(std::lround(towardSun[k] * 40.0)));
      for (int k = 0; k < 3; ++k)
        mix(static_cast<int64_t>(std::lround(sunEnergy3[k] * 32.0)));
      mix(static_cast<int64_t>(draws.size()));
      uint64_t idxSum = 0;
      for (const auto &d : draws)
        idxSum += d.indexCount;
      mix(static_cast<int64_t>(idxSum));
      mix(skyMars ? 7 : 3);
      {
        const size_t nEx = extraLights.size() / 16;
        for (size_t li = 0; li < nEx; ++li) {
          const float *e = extraLights.data() + li * 16;
          if (e[3] < 0.5f)
            continue;  // extra directionals stay real-time
          for (int k = 0; k < 3; ++k)
            mix(static_cast<int64_t>(std::lround(e[k] * 8.0f)));
          for (int k = 4; k < 7; ++k)
            mix(static_cast<int64_t>(std::lround(e[k] * 16.0f)));
        }
      }
      if (key != mOmniKey) {
        mOmniKey = key;
        // main-thread snapshot: world-space triangles + materials (reload-safe; the worker
        // touches nothing live)
        auto tris = std::make_shared<std::vector<OmniLightTriangle>>();
        auto mats = std::make_shared<std::vector<OmniLightMaterial>>();
        for (const auto &d : draws) {
          // castShadows FALSE draws are excluded on principle (an object that opts out of
          // shadowing must not write light or occlusion into the GI field) — and one of them
          // is load-bearing: the SUN MARKER's emissiveIntensity-35 ball. Its slow-spawning
          // python controller parks it at z=100000 AFTER the first bake key stabilises, so
          // the launcher's first bake captured a radiance-~2500 area light hovering over the
          // lawn — a warm ambient flood that drowned every prop shadow until the second bake
          // (~7 s in) healed it. Measured: bake #1 42544 tris (ball in), bake #2 41392.
          if (!d.cpuPositions || !d.cpuIndices || d.translucent || !d.modelMatrix16 ||
              !d.castShadows)
            continue;
          OmniLightMaterial m;
          for (int k = 0; k < 3; ++k) {
            const float bc = k == 0 ? d.baseColorR : (k == 1 ? d.baseColorG : d.baseColorB);
            const float em = k == 0 ? d.emissiveR : (k == 1 ? d.emissiveG : d.emissiveB);
            m.albedoLin[k] = std::min(0.95f, bc * d.texMeanLin[k]);
            m.emissiveLin[k] = std::pow(std::max(0.0f, em), 2.2f);
          }
          const uint32_t mi = static_cast<uint32_t>(mats->size());
          mats->push_back(m);
          const std::vector<float> &P = *d.cpuPositions;
          const std::vector<uint32_t> &I = *d.cpuIndices;
          const float *M = d.modelMatrix16;  // column-major
          auto xf = [&](uint32_t vi, float *o) {
            const float x = P[vi * 3 + 0], y = P[vi * 3 + 1], z = P[vi * 3 + 2];
            o[0] = M[0] * x + M[4] * y + M[8] * z + M[12];
            o[1] = M[1] * x + M[5] * y + M[9] * z + M[13];
            o[2] = M[2] * x + M[6] * y + M[10] * z + M[14];
          };
          for (size_t t = 0; t + 2 < I.size(); t += 3) {
            const size_t nv = P.size() / 3;
            if (I[t] >= nv || I[t + 1] >= nv || I[t + 2] >= nv)
              continue;
            OmniLightTriangle tr;
            xf(I[t], tr.v0);
            xf(I[t + 1], tr.v1);
            xf(I[t + 2], tr.v2);
            tr.material = mi;
            tris->push_back(tr);
          }
        }
        if (!tris->empty()) {
          // sky table (64x32 lat-long over the CURRENT atmosphere) for thread-safe miss lookups
          auto skyTab = std::make_shared<std::vector<float>>(64 * 32 * 3, 0.0f);
          {
            SkyMarchP mp;
            mp.rayR = scat24[0]; mp.rayG = scat24[1]; mp.rayB = scat24[2]; mp.rayExp = scat24[3];
            mp.mieS = scat24[4]; mp.mieExp = scat24[7];
            mp.mieA = scat24[8]; mp.phaseG = scat24[11];
            mp.ozR = scat24[12]; mp.ozG = scat24[13]; mp.ozB = scat24[14]; mp.camHkm = scat24[15];
            mp.botKm = scat24[16]; mp.topKm = scat24[17]; mp.sinSunElev = scat24[18];
            mp.albedo = scat24[19];
            mp.illR = scat24[20]; mp.illG = scat24[21]; mp.illB = scat24[22];
            for (int iy = 0; iy < 32; ++iy) {
              const double elev = (((iy + 0.5) / 32.0) * 2.0 - 1.0) * 1.5707963;
              for (int ix = 0; ix < 64; ++ix) {
                const double azd = ((ix + 0.5) / 64.0) * 3.14159265;  // symmetric about the sun
                float col[3];
                sciSampleSky(mp, elev, azd, col);
                float *o = skyTab->data() + (iy * 64 + ix) * 3;
                o[0] = col[0]; o[1] = col[1]; o[2] = col[2];
              }
            }
          }
          const float sunAz = std::atan2(towardSun.y(), towardSun.x());
          OmniLightParams bp;
          for (int k = 0; k < 3; ++k)
            bp.sunEnergy[k] = sunEnergy3[k];
          bp.sunDirTo[0] = static_cast<float>(towardSun.x());
          bp.sunDirTo[1] = static_cast<float>(towardSun.y());
          bp.sunDirTo[2] = static_cast<float>(towardSun.z());
          static const int sOmniRays = []() {
            const int v = qEnvironmentVariableIntValue("OMNILIGHT_RAYS");
            return v >= 16 ? v : 256;
          }();
          bp.raysPerProbe = sOmniRays;
          // PROBE DENSITY. The default was OmniLightParams' own 0.45 m, which put probes
          // more than twice as far apart as a robot base is wide (OmniArm 6's is ~0.20 m),
          // so occlusion belonging to a 20 cm object was trilinearly smeared across ~0.45 m
          // of floor in every direction. Tightening it measurably reduces GI over-occlusion
          // on tabletop-scale worlds, and the shader's normal bias is one probe cell, so it
          // tightens with the lattice rather than needing its own tune.
          // ⚠ This is NOT the fix for the user-reported "shadow aura", though it was first
          // committed as if it were. That aura is GTAO's -- its falloff window stopped
          // matching the disc it samples once a pixel clamp was added to the port. See
          // docs/developer/wgpu-shadow-aura.md, which also records why a single-variable
          // A/B against OMNILIGHT=0 pointed here: turning GI off moves MORE pixels than
          // turning AO off, and a pixel count cannot tell a scene-wide brightness shift
          // apart from a halo. Read the difference IMAGE.
          // ⚠ This only binds on SMALL worlds: above maxDims x minSpacing (40 x 0.45 = 18 m)
          // the per-axis dim cap already decides spacing, so city-scale worlds are
          // untouched and pay nothing extra.
          static const float sOmniSpacing = []() {
            bool ok = false;
            const float v = qEnvironmentVariable("OMNILIGHT_SPACING").toFloat(&ok);
            return ok && v > 0.0f ? v : 0.22f;   // OMNILIGHT_SPACING=0.45 = exact revert
          }();
          bp.minSpacing = sOmniSpacing;
          static const float sOmniScale = []() {
            bool ok = false;
            const float v = qEnvironmentVariable("OMNILIGHT_SCALE").toFloat(&ok);
            return ok && v > 0.0f ? v : 0.85f;
          }();
          bp.outputScale = sOmniScale;
          bp.progressDone = &mOmniProgressDone;
          bp.progressTotal = &mOmniProgressTotal;
          // Static local lights (point/spot) bake INTO the field with occlusion; their
          // unshadowed real-time versions crossfade out as the volume fades in.
          {
            const size_t nEx = extraLights.size() / 16;
            for (size_t li = 0; li < nEx; ++li) {
              const float *e = extraLights.data() + li * 16;
              if (e[3] < 0.5f)
                continue;  // keep extra directionals real-time
              OmniLightLocal gl;
              gl.pos[0] = e[0]; gl.pos[1] = e[1]; gl.pos[2] = e[2];
              gl.type = e[3] > 1.5f ? 2 : 1;
              gl.colorLin[0] = e[4]; gl.colorLin[1] = e[5]; gl.colorLin[2] = e[6];
              gl.radius = e[7];
              gl.atten[0] = e[8]; gl.atten[1] = e[9]; gl.atten[2] = e[10];
              gl.cosCut = e[11];
              gl.spotDir[0] = e[12]; gl.spotDir[1] = e[13]; gl.spotDir[2] = e[14];
              gl.cosBeam = e[15];
              bp.locals.push_back(gl);
            }
            mOmniLocalsBaked = !bp.locals.empty();
            // Brightness calibration vs the retired real-time term: the L1-SH field softens
            // peaks, so pleasing pools need a boost over the raw physical value.
            static const float sLocalScale = []() {
              bool ok = false;
              const float v = qEnvironmentVariable("OMNILIGHT_LOCAL_SCALE").toFloat(&ok);
              return ok && v > 0.0f ? v : 3.0f;
            }();
            bp.localScale = sLocalScale;
          }
          bp.skySample = [skyTab, sunAz](const float dir[3], float out[3]) {
            const float el = std::asin(std::max(-1.0f, std::min(1.0f, dir[2])));
            float az = std::atan2(dir[1], dir[0]) - sunAz;
            while (az > 3.14159265f) az -= 6.2831853f;
            while (az < -3.14159265f) az += 6.2831853f;
            const float azAbs = std::abs(az);  // sky is symmetric about the sun azimuth
            int ix = static_cast<int>(azAbs / 3.14159265f * 64.0f);
            int iy = static_cast<int>((el / 1.5707963f * 0.5f + 0.5f) * 32.0f);
            ix = std::max(0, std::min(63, ix));
            iy = std::max(0, std::min(31, iy));
            const float *c = skyTab->data() + (iy * 64 + ix) * 3;
            out[0] = c[0]; out[1] = c[1]; out[2] = c[2];
          };
          auto result = std::make_shared<OmniLightVolume>();
          mOmniResult = result;
          mOmniBakeDone.store(false, std::memory_order_release);
          mOmniBakeRunning = true;
          std::atomic<bool> *done = &mOmniBakeDone;
          mOmniBakeThread = std::thread([tris, mats, bp, result, done]() {
            omniLightBake(*tris, *mats, bp, *result);
            done->store(true, std::memory_order_release);
          });
        }
      }
    }
    cdOk = mWgpuRenderTarget->clearAndDrawSceneTexturedShadowed(
      sky, vpm, lightVP, lit4, draws.data(), static_cast<uint32_t>(draws.size()),
      qEnvironmentVariableIsSet("OMNISIM_WGPU_NO_SHADOW") ? 0.0f
        : 0.8f /*shadowStrength: WREN outdoor shadows keep partial direct (env = diagnostic kill)*/,
      qEnvironmentVariableIsSet("OMNISIM_WGPU_SHADOW_DEBUG") ? -0.0006f : 0.0006f /*depthBias; negative = error-field debug*/,
      camPos, skyDerived ? hemiSkyDrv4 : hemiSky4, skyDerived ? hemiGroundDrv4 : hemiGround4,
      worldUp3, rgbaOut,
      // pipelined normally; a grab OR a synth dump needs THIS frame, synchronously (and the
      // ground-truth passes reuse the readback buffer, which must not be mid-map).
      // W2: a presentation-free frame is synchronous too -- the async ping-pong exists to stop the
      // CPU waiting on the GPU it just fed for a frame that is about to be DISPLAYED, and delivers
      // frame N-1. With nothing displayed there is no latency to hide, and a headless dump that
      // silently wrote frame N-1 would be a quietly wrong ground truth.
      /*asyncReadback=*/!mWgpuGrabRequested && !synthThisFrame && !headlessFrame,
      (skyAtmo || skyCube) ? skyU : nullptr, dayF, fogParams4[3] > 0.0f ? fogParams4 : nullptr, bloomStrength,
      agxExposure, ssaoStrength, revZ,
      extraLights.empty() ? nullptr : omniExtras.data(),
      static_cast<uint32_t>(extraLights.size() / 16), ssaoRadiusScale, sunEnergy3,
      static_cast<float>(zNear),
      csmCount > 0 ? csmVps : nullptr, csmCount > 0 ? csmSplits4 : nullptr, csmCount,
      /*bloomHdrThreshold=*/(agxExposure > 0.0f && vp->bloomThresholdField() &&
                             vp->bloomThresholdField()->value() > 0.0)
        ? static_cast<float>(vp->bloomThresholdField()->value())
        : 0.0f,
      skyScatter ? scat24 : nullptr, skyDerived ? iblSky8 : nullptr);
  }
  tRender = phaseTimer.elapsed();
  if (qEnvironmentVariableIsSet("OMNISIM_WGPU_REPORT") && (f % 100 == 0 || (!cdOk && f < 5))) {
    const QString rpath = qEnvironmentVariable("OMNISIM_WGPU_REPORT");
    QFile tf(rpath);
    // W1c: glArms is the CUMULATIVE number of times the draw collect has had to make WREN's
    // GL context current (a mesh-cache miss falling back to wr_static_mesh_read_data);
    // glArmsWindow is how many of those happened since the previous report line. The
    // WREN-retirement claim "the collect no longer needs GL" is glArmsWindow == 0 on a
    // settled world, per world, measured — not asserted. A non-zero window on a static scene
    // means something is still missing the cache every rebuild; chase THAT, not the average.
    const qulonglong glArms = static_cast<qulonglong>(OmWgpuSceneRenderer::glArmCount());
    static qulonglong sPrevGlArms = 0;
    const qulonglong glArmsWindow = glArms - sPrevGlArms;
    // DEFORMABLES: how many Cloth/SoftBody draws this frame appended, and the world-space Z
    // extent of their CURRENT geometry (their model matrix is identity, so the mesh cache's local
    // bounding sphere IS the world one). Both are read off the buffers that were uploaded to the
    // GPU this frame, so `deformZ` MOVING over a run is the measurement that a deformable is
    // ANIMATING rather than frozen at its rest pose -- the half of the "deformables do not render"
    // defect a single screenshot cannot tell apart. `deform=0` in a world that has a Cloth means
    // the draw path declined it; -1.0000/-1.0000 means there is no deformable at all.
    // The appended tail is [deformables][granular]; scan only the deformable half so a world
    // with particles cannot widen deformZ with sand.
    float dzMin = -1.0f, dzMax = -1.0f;
    if (mWgpuDeformableDraws > 0 && draws.size() >= mWgpuDynamicDraws) {
      const size_t dBegin = draws.size() - mWgpuDynamicDraws;
      const size_t dEnd = dBegin + mWgpuDeformableDraws;
      bool first = true;
      for (size_t di = dBegin; di < dEnd; ++di) {
        if (draws[di].localRadius < 0.0f)
          continue;  // unknown bounds (non-finite geometry) -- do not fold into a range
        const float lo = draws[di].localCenter[2] - draws[di].localRadius;
        const float hi = draws[di].localCenter[2] + draws[di].localRadius;
        if (first || lo < dzMin)
          dzMin = lo;
        if (first || hi > dzMax)
          dzMax = hi;
        first = false;
      }
    }
    // P2 ANIMATION TELEMETRY, and it is deliberately a POSITION, not a count. `track=N`
    // says the belt was collected; `trackT` -- the world translation of the FIRST belt
    // element's model matrix -- says it MOVED. A belt frozen at element 0 (the pump never
    // ran) reports the same N and a constant trackT, which is exactly the half of the
    // defect a single screenshot cannot see. `muscleR` is the first muscle draw's local
    // bounding radius: the spheroid keeps constant VOLUME, so its radius tracks its
    // contraction, and a muscle frozen at its first upload holds that number still.
    // -1 in either means "there is none", never "it is not moving".
    float trkX = -1.0f, trkY = -1.0f, trkZ = -1.0f, musR = -1.0f;
    if (draws.size() >= mWgpuDynamicDraws) {
      const size_t tBegin =
        draws.size() - mWgpuDynamicDraws + mWgpuDeformableDraws + mWgpuGranularDraws;
      if (mWgpuTrackDraws > 0 && tBegin < draws.size() && draws[tBegin].modelMatrix16) {
        trkX = draws[tBegin].modelMatrix16[12];
        trkY = draws[tBegin].modelMatrix16[13];
        trkZ = draws[tBegin].modelMatrix16[14];
      }
      const size_t mBegin = tBegin + mWgpuTrackDraws;
      if (mWgpuMuscleDraws > 0 && mBegin < draws.size())
        musR = draws[mBegin].localRadius;
    }
    // ⚠ trackT ALONE CANNOT SAY "the belt advanced", and on the shipped track.omniworld it
    // measurably does not: its robot descends, so the world-space number drifts without
    // bound while saying nothing about the belt. trackP is the same element in the TRACK's
    // OWN path coordinates -- bounded by the closed path, and moving if and only if the belt
    // did. Read the two together: trackT proves the DRAW's matrix is live, trackP proves the
    // BELT is.
    double bpx = -1.0, bpy = -1.0, bpr = -1.0;
    OmTrack::wgpuFirstBeltProbe(bpx, bpy, bpr);
    // The OTHER half of a Track, and the one ConveyorBelt.proto (and therefore the shipped
    // warehouse_industrial demo) actually uses: its belt animates by SCROLLING ITS TEXTURE,
    // i.e. by a per-step TextureTransform translation. On a CACHED draw list only the model
    // matrices used to be refreshed, so that scroll was frozen on the main view while every
    // sensor -- which rebuilds its list per frame -- saw it move. uvOff is the largest |uvB|
    // over the cached prefix: it moves if and only if some draw's UV offset is being re-read.
    float uvOff = 0.0f;
    const size_t cachedPrefix = draws.size() >= mWgpuDynamicDraws ? draws.size() - mWgpuDynamicDraws : 0;
    for (size_t k = 0; k < cachedPrefix; ++k) {
      const float m = std::max(std::fabs(draws[k].uvB[0]), std::fabs(draws[k].uvB[1]));
      if (m > uvOff)
        uvOff = m;
    }
    if (tf.open(QIODevice::Append | QIODevice::Text)) {
      tf.write(QString("frame=%1 calls draws=%2 W=%3 H=%4 cdOk=%5 collectMs=%6 renderMs=%7 prevBlitMs=%8 maxGapMs=%9 "
                       "maxCollectMs=%10 glArms=%11 glArmsWindow=%12 ovCalled=%13 ovBatches=%14 ovVerts=%15 ovOk=%16 deform=%17 deformZ=%18/%19 hudCalled=%20 hudQuads=%21 hudOk=%22 granular=%23 track=%24 trackT=%25/%26/%27 muscle=%28 muscleR=%29 trackP=%30/%31/%32 uvOff=%33\n")
                 .arg(f).arg(static_cast<qulonglong>(draws.size())).arg(W).arg(H).arg(cdOk ? 1 : 0)
                 .arg(tCollect).arg(tRender - tCollect).arg(sPrevBlitMs).arg(sMaxGapMs).arg(sMaxCollectMs)
                 .arg(glArms).arg(glArmsWindow).arg(sOvCalled).arg(sOvBatches).arg(sOvVerts).arg(sOvOk)
                 .arg(static_cast<qulonglong>(mWgpuDeformableDraws))
                 .arg(dzMin, 0, 'f', 4).arg(dzMax, 0, 'f', 4)
                 .arg(sHudCalled).arg(sHudQuads).arg(sHudOk)
                 .arg(static_cast<qulonglong>(mWgpuGranularDraws))
                 .arg(static_cast<qulonglong>(mWgpuTrackDraws))
                 .arg(trkX, 0, 'f', 4).arg(trkY, 0, 'f', 4).arg(trkZ, 0, 'f', 4)
                 .arg(static_cast<qulonglong>(mWgpuMuscleDraws))
                 .arg(musR, 0, 'f', 5)
                 .arg(bpx, 0, 'f', 5).arg(bpy, 0, 'f', 5).arg(bpr, 0, 'f', 5)
                 .arg(uvOff, 0, 'f', 5)
                 .toUtf8());
      sMaxGapMs = 0;
      sMaxCollectMs = 0;
      sPrevGlArms = glArms;
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

  // SYNTHETIC-DATA DUMP (OMNISIM_WGPU_SYNTH_DUMP=<dir>): one aligned ground-truth sample per
  // trigger frame — rgb (this frame, the full wgpu stack), metric depth (16-bit millimetre PNG via
  // the F32 depth pass), per-SOLID instance IDs (flat pick-shader colours, exact round-trip), and
  // a meta.json with camera intrinsics/extrinsics, the light rig, and the id -> node mapping.
  // OMNISIM_WGPU_SYNTH_EVERY=N (default 0 = one-shot at the dump frame) repeats every N frames.
  if (rgbaOut && synthThisFrame) {
    if (!draws.empty()) {
      const QString dir = qEnvironmentVariable("OMNISIM_WGPU_SYNTH_DUMP");
      QDir().mkpath(dir);
      const QString tag = QStringLiteral("%1").arg(f, 6, 10, QLatin1Char('0'));
      // RGB — the frame just rendered (tonemapped display output).
      QImage(rgba.data(), W, H, W * 4, QImage::Format_RGBA8888)
        .copy()
        .save(dir + QStringLiteral("/rgb_") + tag + QStringLiteral(".png"), "PNG");
      // Ground-truth passes use a STANDARD (non-reversed) view-proj — their pipelines carry the
      // classic Less depth test.
      float vpStd[16];
      OmWgpuSceneRenderer::buildViewProj(cam, hf, aspect, zNear, 1000.0, vpStd, false);
      // Metric depth: metres from the camera plane; encoded as uint16 MILLIMETRES (0 = no hit,
      // range 65.535 m — the synthetic-data convention). farClear 0 marks empty pixels.
      {
        std::vector<float> meters(static_cast<size_t>(W) * H, 0.0f);
        if (mWgpuRenderTarget->clearAndDrawSceneDepthF32(0.0f, vpStd, draws.data(),
                                                         static_cast<uint32_t>(draws.size()),
                                                         meters.data())) {
          QImage d16(W, H, QImage::Format_Grayscale16);
          for (int y = 0; y < H; ++y) {
            quint16 *row = reinterpret_cast<quint16 *>(d16.scanLine(y));
            const float *src = meters.data() + static_cast<size_t>(y) * W;
            for (int x = 0; x < W; ++x) {
              const float mm = src[x] * 1000.0f;
              row[x] = static_cast<quint16>(std::max(0.0f, std::min(65535.0f, mm)));
            }
          }
          d16.save(dir + QStringLiteral("/depth_") + tag + QStringLiteral(".png"), "PNG");
        }
      }
      // Instance IDs: one id PER OWNING SOLID (not per draw), painted flat through the pick
      // shader with sRGB encode OFF so the readback returns the exact authored bytes.
      // id 0 = background; id -> node mapping goes into meta.json.
      {
        std::vector<OmWgpuSolidDraw> idDraws = draws;
        std::map<OmSolid *, int> solidIds;
        std::vector<int> drawSolidId(idDraws.size(), 0);
        for (size_t i = 0; i < idDraws.size(); ++i) {
          OmSolid *owner = i < mWgpuDrawSolids.size() ? mWgpuDrawSolids[i] : nullptr;
          int id;
          if (owner) {
            auto it = solidIds.find(owner);
            id = it != solidIds.end() ? it->second
                                      : (solidIds[owner] = static_cast<int>(solidIds.size()) + 1);
          } else
            id = static_cast<int>(i) + 1;  // unowned draw: fall back to a per-draw id
          drawSolidId[i] = id;
          idDraws[i].baseColorR = static_cast<float>(id & 255) / 255.0f;
          idDraws[i].baseColorG = static_cast<float>((id >> 8) & 255) / 255.0f;
          idDraws[i].baseColorB = static_cast<float>((id >> 16) & 255) / 255.0f;
          idDraws[i].baseColorA = 1.0f;
          idDraws[i].translucent = false;  // glass occludes in the ID map (disclosed in the docs)
          idDraws[i].textureView = nullptr;
        }
        std::vector<uint8_t> idRgba(static_cast<size_t>(W) * H * 4, 0);
        const OmWgpuClearColor idClear = {0.0f, 0.0f, 0.0f, 1.0f};
        const float idLight[4] = {0.0f, 0.0f, -1.0f, 1.0f};  // unused by the pick shader
        if (mWgpuRenderTarget->clearAndDrawScene(idClear, vpStd, idLight, idDraws.data(),
                                                 static_cast<uint32_t>(idDraws.size()),
                                                 idRgba.data(), false, 1.0f, false, nullptr,
                                                 /*pickMode=*/true, /*srgbEncode=*/false)) {
          QImage(idRgba.data(), W, H, W * 4, QImage::Format_RGBA8888)
            .copy()
            .save(dir + QStringLiteral("/inst_") + tag + QStringLiteral(".png"), "PNG");
        }
        // meta.json: everything a training pipeline needs to consume the three images.
        QJsonObject meta;
        {
          QJsonObject camj;
          QJsonArray pos;
          pos << cam(0, 3) << cam(1, 3) << cam(2, 3);
          camj.insert(QStringLiteral("position"), pos);
          QJsonArray rot;  // world-from-camera rotation, row-major 3x3 (camera +X fwd, +Z up)
          for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
              rot << cam(r, c);
          camj.insert(QStringLiteral("rotation_row_major"), rot);
          camj.insert(QStringLiteral("width"), W);
          camj.insert(QStringLiteral("height"), H);
          camj.insert(QStringLiteral("fov_x_rad"), hf);
          const double fx = W / (2.0 * std::tan(0.5 * hf));
          camj.insert(QStringLiteral("fx"), fx);
          camj.insert(QStringLiteral("fy"), fx);  // square pixels
          camj.insert(QStringLiteral("cx"), W / 2.0);
          camj.insert(QStringLiteral("cy"), H / 2.0);
          camj.insert(QStringLiteral("near"), zNear);
          camj.insert(QStringLiteral("far"), 1000.0);
          camj.insert(QStringLiteral("axes"), QStringLiteral("camera +X forward, +Z up (world ENU)"));
          meta.insert(QStringLiteral("camera"), camj);
        }
        {
          QJsonObject lightj;
          QJsonArray sd;
          sd << lit4[0] << lit4[1] << lit4[2];
          lightj.insert(QStringLiteral("sun_direction"), sd);  // direction light TRAVELS
          QJsonArray se;
          se << sunEnergy3[0] << sunEnergy3[1] << sunEnergy3[2];
          lightj.insert(QStringLiteral("sun_energy_linear"), se);
          lightj.insert(QStringLiteral("day_factor"), synthDayF);
          lightj.insert(QStringLiteral("sky_scatter"), synthScatter);
          lightj.insert(QStringLiteral("cloud_cover"), synthCloud);
          meta.insert(QStringLiteral("light"), lightj);
        }
        {
          QJsonObject conv;
          conv.insert(QStringLiteral("depth"), QStringLiteral("uint16 millimetres, 0 = no hit"));
          conv.insert(QStringLiteral("instance"),
                      QStringLiteral("id = R + G*256 + B*65536 of inst png, 0 = background"));
          meta.insert(QStringLiteral("encoding"), conv);
        }
        {
          QJsonArray inst;
          for (const auto &kv : solidIds) {
            QJsonObject o;
            o.insert(QStringLiteral("id"), kv.second);
            o.insert(QStringLiteral("name"), kv.first->name());
            if (!kv.first->defName().isEmpty())
              o.insert(QStringLiteral("def"), kv.first->defName());
            // world position from the solid's first draw's model matrix (column-major, col 3)
            for (size_t i = 0; i < idDraws.size(); ++i)
              if (i < mWgpuDrawSolids.size() && mWgpuDrawSolids[i] == kv.first &&
                  draws[i].modelMatrix16) {
                QJsonArray p;
                p << draws[i].modelMatrix16[12] << draws[i].modelMatrix16[13]
                  << draws[i].modelMatrix16[14];
                o.insert(QStringLiteral("position"), p);
                break;
              }
            inst.append(o);
          }
          meta.insert(QStringLiteral("instances"), inst);
        }
        meta.insert(QStringLiteral("frame"), static_cast<qint64>(f));
        QFile mf(dir + QStringLiteral("/meta_") + tag + QStringLiteral(".json"));
        if (mf.open(QIODevice::WriteOnly | QIODevice::Truncate))
          mf.write(QJsonDocument(meta).toJson(QJsonDocument::Indented));
      }
    }
  }

  // ===== W4a (WREN retirement): GEOMETRY OVERLAY PASS. =====
  // The five GUI surfaces that went dark when wgpu became the default renderer exist only as
  // WREN scene nodes. This wires back the GEOMETRY half: the selection outline and every
  // View > Optional Rendering item whose collector already existed (bounding objects, joint
  // axes, camera/range-finder frustums, surface normals, lidar ray paths, contact points).
  //
  // WHICH PASS, AND WHY: dead last — after opaque, sky, SSR, volumetrics, transparent, TAA and
  // the tonemap, i.e. the same slot drawOmniProgress uses. Everything upstream of that is a
  // radiance pipeline; a debug wireframe is not radiance, and running one through AgX, bloom
  // and a temporal reprojection would change its colour and smear it across frames. The cost
  // is that the lines are always-on-top rather than depth-tested (the scene depth for this
  // frame is a 4x-MSAA reversed-Z buffer a 1x post-tonemap pass cannot attach) — see
  // OmWgpuRenderTarget::drawOverlayLines for the full disclosure.
  //
  // COST WHEN NOTHING IS SHOWN: one bitmask read and one pointer compare. No collect, no
  // buffer, no encoder, no pass — drawOverlayLines is not even called.
  //
  // PRESENT-PATH ONLY, like drawOmniProgress: on the readback/blit fallback the CPU pixels were
  // already copied out INSIDE the scene render, so painting the texture afterwards would change
  // nothing visible. Same reason screenshots and the MAINVIEW/SYNTH dumps come out clean. It is
  // deliberately NOT skipped while mWgpuGrabRequested: that re-entrant render presents too, and
  // skipping it would make overlays flicker off on every web-stream frame.
  if (sOverlaysOn && present) {
    const bool anyOptional = OmWgpuView::anyOptionalRenderingEnabled();
    // P8: the manipulator gizmo. Hatch OMNISIM_WGPU_GIZMO=0 (VALUE-parsed, default ON).
    static const bool sGizmoOn = !qEnvironmentVariableIsSet("OMNISIM_WGPU_GIZMO") ||
                                 qEnvironmentVariableIntValue("OMNISIM_WGPU_GIZMO") != 0;
    const bool anyGizmo = sGizmoOn && OmGizmoLines::anyVisible();
    // Lane E4 (Phase D orphaned ports): the force/torque DRAG ARROW and the BoundingSphere
    // optional rendering, both WREN-only visuals until now. Hatches value-parsed, default ON,
    // matching the P8 gizmo pattern.
    static const bool sDragArrowsOn = !qEnvironmentVariableIsSet("OMNISIM_WGPU_DRAG_ARROWS") ||
                                      qEnvironmentVariableIntValue("OMNISIM_WGPU_DRAG_ARROWS") != 0;
    // The arrow follows the ACTIVE drag event -- the exact state the WREN representation
    // reads -- and, like WREN, keeps showing a LOCKED drag (paused sim) until it is deleted.
    const OmDragPhysicsEvent *dragPhys =
      mDragForce ? static_cast<const OmDragPhysicsEvent *>(mDragForce) : static_cast<const OmDragPhysicsEvent *>(mDragTorque);
    const bool anyDragArrow = sDragArrowsOn && dragPhys;
    const bool sphereOn = OmVisualBoundingSphere::overlayEnabled();
    if (anyOptional || selTop || anyGizmo || anyDragArrow || sphereOn) {
      OmWgpuView::OverlayLines ov;
      if (anyOptional)
        OmWgpuView::collectOptionalRenderingLines(ov);
      std::vector<float> selBox;
      if (selTop)
        collectWgpuSelectionBoxV3D(draws, mWgpuDrawSolids, selTop, selBox);
      // P8 (WREN retirement): THE MANIPULATOR GIZMO -- the main view's OWN gizmo, not the pane's.
      //
      // OmWgpuView already has translate/rotate handle collectors with a verified hit test, and
      // they are deliberately not used here. The pane's gizmo is a SECOND gizmo: fixed 0.25 m
      // handles with a screen-distance hit test in OmWgpuView's own mouse handlers. The MAIN
      // view's gizmo is OmTranslateRotateManipulator, whose hit test is OmWrenPicker -- a GL
      // picking render of the actual handle meshes through resources/wren/shaders/handles.vert,
      // which still works perfectly well under a wgpu main view because it renders into its own
      // framebuffer. Drawing the pane's handles here would put visible arrows at 0.25 m while the
      // draggable regions sit wherever that shader put them. So OmGizmoLines reproduces THAT
      // shader's transform over THAT manipulator's own WREN transforms -- see its header.
      //
      // Verified, not asserted: OMNISIM_GIZMO_HITTEST_CHECK=<path> rasterises the same triangles
      // the picker rasterises and compares them against OmWrenPicker pick-by-pick.
      std::vector<float> gizX, gizY, gizZ;
      if (anyGizmo) {
        float view16[16];
        OmWgpuSceneRenderer::buildView(cam, view16);
        const double vertFov = 2.0 * std::atan(std::tan(hf * 0.5) / aspect);
        const double p11 = 1.0 / std::tan(vertFov * 0.5);
        const double projMin = std::min(p11 / aspect, p11);  // handles.vert's own divisor
        const float eye3[3] = {static_cast<float>(eye.x()), static_cast<float>(eye.y()),
                               static_cast<float>(eye.z())};
        OmGizmoLines::collect(view16, projMin, eye3, gizX, gizY, gizZ, nullptr);
      }
      // The drag arrow: same geometry the WREN representation computes, over the same inputs
      // (drag origin/end, viewpoint orientation, the drag's view-distance scaling, pane px).
      std::vector<float> dragArrow, dragCoil;
      if (anyDragArrow)
        OmDragArrowLines::collect(dragPhys, vp->orientation()->value(), W, H, dragArrow, dragCoil);
      // The BoundingSphere optional rendering: three great circles of the selected node's
      // live culling sphere (recomputed per frame, so it follows a moving node).
      std::vector<float> sphereVerts;
      if (sphereOn)
        OmVisualBoundingSphere::collectOverlayCircles(sphereVerts);
      // One flat-coloured batch per overlay type -- WREN colours them apart, so do we.
      OmWgpuLineBatch batches[16];
      uint32_t nb = 0;
      auto addOverlayBatch = [&batches, &nb](const std::vector<float> &v, float r, float g, float b) {
        if (nb >= 16 || v.size() < 16)  // 2 vertices x 8 floats = one segment, the minimum
          return;
        batches[nb].verts = v.data();
        batches[nb].vertexCount = static_cast<uint32_t>(v.size() / 8);
        batches[nb].color[0] = r;
        batches[nb].color[1] = g;
        batches[nb].color[2] = b;
        batches[nb].color[3] = 1.0f;
        ++nb;
      };
      addOverlayBatch(ov.bounding, 0.10f, 1.00f, 0.10f);   // green
      addOverlayBatch(ov.jointAxes, 0.00f, 0.95f, 0.95f);  // cyan
      addOverlayBatch(ov.frustums, 1.00f, 0.55f, 0.00f);   // orange
      addOverlayBatch(ov.normals, 1.00f, 1.00f, 0.00f);    // yellow
      addOverlayBatch(ov.lidarRays, 0.40f, 0.70f, 1.00f);  // sky blue
      addOverlayBatch(ov.contacts, 0.00f, 0.90f, 0.90f);   // cyan
      addOverlayBatch(ov.radar, 0.20f, 0.20f, 1.00f);      // blue, matching OmRadar's WREN colour
      addOverlayBatch(ov.lightRays, 1.00f, 1.00f, 0.00f);  // yellow, matching OmLightSensor's
      addOverlayBatch(selBox, 0.96f, 0.91f, 0.02f);        // OmniSim mimosa — the selection box
      // The gizmo, in WREN's own per-axis colours (OmTranslateRotateManipulator axesColor).
      addOverlayBatch(gizX, 1.00f, 0.00f, 0.00f);          // X red
      addOverlayBatch(gizY, 0.00f, 1.00f, 0.00f);          // Y green
      addOverlayBatch(gizZ, 0.00f, 0.00f, 1.00f);          // Z blue
      // The drag arrow, in the WREN materials' own colours (force orange / torque dark
      // yellow, coil pure yellow -- OmPhysicsVectorRepresentation's constants).
      if (dragPhys && dragPhys->isTorqueDrag())
        addOverlayBatch(dragArrow, 1.00f, 0.85f, 0.00f);
      else
        addOverlayBatch(dragArrow, 1.00f, 0.50f, 0.00f);
      addOverlayBatch(dragCoil, 1.00f, 1.00f, 0.00f);
      addOverlayBatch(sphereVerts, 1.00f, 1.00f, 1.00f);   // BoundingSphere: white wireframe
      // The vectors above must outlive this call: they do, drawOverlayLines stages every batch
      // into GPU buffers before it returns.
      if (nb > 0) {
        sOvCalled = 1;
        sOvBatches = nb;
        sOvVerts = 0;
        for (uint32_t i = 0; i < nb; ++i)
          sOvVerts += batches[i].vertexCount;
        sOvOk = mWgpuRenderTarget->drawOverlayLines(vpm, batches, nb) ? 1 : 0;
      } else {
        sOvCalled = 2;
        sOvBatches = 0;
        sOvVerts = 0;
        sOvOk = -1;
      }
    }
  }

  // ===== P8 GATE INSTRUMENT: does the DRAWN handle coincide with the DRAGGABLE region? =====
  //
  // This is the whole point of P8, so it is measured rather than eyeballed. Ground truth is
  // OmWrenPicker -- the same object mousePressEvent asks, rendering the same handle meshes
  // through the same picking shader. The comparison set is the projection of the triangles
  // OmGizmoLines just drew the silhouette of. For a grid of pixels over each handle's screen
  // bbox we ask both: "is this pixel inside the drawn handle?" and "does the picker report this
  // handle here?", and report the agreement as an intersection-over-union.
  //
  // A perfect score is not expected and would be suspicious: the picker rasterises at WREN's
  // viewport resolution with its own depth test (a handle can be occluded by another handle),
  // while the drawn set is every triangle including back faces. Both are reported so the
  // difference is visible instead of averaged away.
  //
  // OMNISIM_GIZMO_HITTEST_CHECK=<path>. One shot. Auto-selects the first top-level solid when
  // nothing is selected, so the check is runnable without a human clicking.
  static const QString gizCheckPath = qEnvironmentVariable("OMNISIM_GIZMO_HITTEST_CHECK");
  static bool gizCheckDone = false;
  if (!gizCheckDone && !gizCheckPath.isEmpty() && present && f > 60) {
    if (!OmGizmoLines::anyVisible()) {
      // Nothing selected yet -- select something and retry on a later frame. Pick the solid
      // NEAREST THE SCREEN CENTRE and in front of the camera, not merely the first in the draw
      // list: the first one here was the sun marker, whose gizmo origin sits behind the camera,
      // so handles.vert's scalar goes negative and there is nothing to compare. That is a real
      // property of the shader, not a bug -- but it makes a useless check.
      if (OmSelection::instance() && !mWgpuDrawSolids.empty()) {
        OmSolid *best = nullptr;
        double bestScore = 1e18;
        for (size_t i = 0; i < mWgpuDrawSolids.size() && i < draws.size(); ++i) {
          OmSolid *s = mWgpuDrawSolids[i];
          if (!s || !draws[i].modelMatrix16)
            continue;
          const float *mm = draws[i].modelMatrix16;
          double c[4];
          for (int r = 0; r < 4; ++r)
            c[r] = static_cast<double>(vpm[0 * 4 + r]) * mm[12] +
                   static_cast<double>(vpm[1 * 4 + r]) * mm[13] +
                   static_cast<double>(vpm[2 * 4 + r]) * mm[14] +
                   static_cast<double>(vpm[3 * 4 + r]);
          if (!(c[3] > 1e-6))
            continue;  // behind the camera
          const double nx = c[0] / c[3], ny = c[1] / c[3];
          const double score = nx * nx + ny * ny;
          if (score < bestScore) {
            bestScore = score;
            while (s && s->upperSolid())
              s = s->upperSolid();
            best = s;
          }
        }
        if (best)
          OmSelection::instance()->selectPoseFromView3D(best);
      }
    } else {
      gizCheckDone = true;
      float view16[16];
      OmWgpuSceneRenderer::buildView(cam, view16);
      const double vertFov = 2.0 * std::atan(std::tan(hf * 0.5) / aspect);
      const double p11 = 1.0 / std::tan(vertFov * 0.5);
      const double projMin = std::min(p11 / aspect, p11);
      const float eye3[3] = {static_cast<float>(eye.x()), static_cast<float>(eye.y()),
                             static_cast<float>(eye.z())};
      std::vector<float> gx, gy, gz;
      std::vector<OmGizmoLines::Handle> handles;
      OmGizmoLines::collect(view16, projMin, eye3, gx, gy, gz, &handles);
      const double dpr = width() > 0 ? static_cast<double>(W) / width() : 1.0;
      const int lw = width(), lh = height();
      QString report;
      report += QString("handles=%1 view=%2x%3 target=%4x%5 dpr=%6\n")
                  .arg(static_cast<int>(handles.size())).arg(lw).arg(lh).arg(W).arg(H)
                  .arg(dpr, 0, 'f', 3);
      {
        // Say WHICH node the gizmo belongs to and what the manipulator actually offers --
        // "handles=0" is otherwise unattributable between "nothing selected", "this node
        // cannot be translated" and "the mesh would not load".
        OmAbstractPose *const gp = OmSelection::instance() ? OmSelection::instance()->selectedAbstractPose() : nullptr;
        OmTranslateRotateManipulator *const gm = gp ? gp->translateRotateManipulator() : nullptr;
        report += QString("selected=%1 attached=%2 hasTranslate=%3 hasRotate=%4 screenScale=%5\n")
                    .arg(gp && gp->baseNode() ? gp->baseNode()->usefulName() : QStringLiteral("<none>"))
                    .arg(gm && gm->isAttached() ? 1 : 0)
                    .arg(gm && gm->hasTranslationHandles() ? 1 : 0)
                    .arg(gm && gm->hasRotationHandles() ? 1 : 0)
                    .arg(gm ? gm->handleScreenScale() : -1.0f, 0, 'f', 6);
        report += QString("handleScale_w=%1 m\n")
                    .arg(OmGizmoLines::debugHandleScale(view16, projMin), 0, 'f', 6);
      }
      // world -> LOGICAL view pixels (the frame OmWrenPicker::pick and the mouse handlers use)
      auto project = [&](const float *p, double &ox, double &oy) -> bool {
        double c[4];
        for (int r = 0; r < 4; ++r)
          c[r] = static_cast<double>(vpm[0 * 4 + r]) * p[0] + static_cast<double>(vpm[1 * 4 + r]) * p[1] +
                 static_cast<double>(vpm[2 * 4 + r]) * p[2] + static_cast<double>(vpm[3 * 4 + r]);
        if (!(c[3] > 1e-6))
          return false;
        ox = ((c[0] / c[3]) * 0.5 + 0.5) * W / dpr;
        oy = (1.0 - ((c[1] / c[3]) * 0.5 + 0.5)) * H / dpr;
        return true;
      };
      for (size_t hi = 0; hi < handles.size(); ++hi) {
        const OmGizmoLines::Handle &h = handles[hi];
        const size_t nTri = h.tris.size() / 9;
        std::vector<double> pts(nTri * 6, 0.0);
        std::vector<unsigned char> ok(nTri, 0);
        double minX = 1e18, minY = 1e18, maxX = -1e18, maxY = -1e18;
        for (size_t t = 0; t < nTri; ++t) {
          bool good = true;
          for (int k = 0; k < 3 && good; ++k) {
            double px = 0.0, py = 0.0;
            good = project(&h.tris[t * 9 + k * 3], px, py);
            pts[t * 6 + k * 2 + 0] = px;
            pts[t * 6 + k * 2 + 1] = py;
            if (good) {
              if (px < minX) minX = px;
              if (px > maxX) maxX = px;
              if (py < minY) minY = py;
              if (py > maxY) maxY = py;
            }
          }
          ok[t] = good ? 1 : 0;
        }
        if (maxX < minX) {
          report += QString("handle axis=%1 kind=%2 OFFSCREEN\n").arg(h.axis).arg(h.rotate ? "rotate" : "translate");
          continue;
        }
        const int x0 = std::max(0, static_cast<int>(std::floor(minX)) - 2);
        const int y0 = std::max(0, static_cast<int>(std::floor(minY)) - 2);
        const int x1 = std::min(lw - 1, static_cast<int>(std::ceil(maxX)) + 2);
        const int y1 = std::min(lh - 1, static_cast<int>(std::ceil(maxY)) + 2);
        const int stepX = std::max(1, (x1 - x0) / 10);
        const int stepY = std::max(1, (y1 - y0) / 10);
        int both = 0, drawnOnly = 0, pickedOnly = 0, neither = 0;
        // Attribution for the drawnOnly bucket: a pixel inside the drawn handle that the picker
        // gives to a DIFFERENT handle is occlusion (the picker depth-tests; two handles overlap
        // on screen), which is expected and harmless. A pixel inside the drawn handle that the
        // picker gives to NOTHING would be a real miss -- a visible handle you cannot grab.
        int drawnPickedOther = 0, drawnPickedNothing = 0;
        double dcx = 0.0, dcy = 0.0, pcx = 0.0, pcy = 0.0;
        int dn = 0, pn = 0;
        for (int py = y0; py <= y1; py += stepY) {
          for (int px = x0; px <= x1; px += stepX) {
            // inside the DRAWN triangle set?
            bool inDrawn = false;
            const double fx = px + 0.5, fy = py + 0.5;
            for (size_t t = 0; t < nTri && !inDrawn; ++t) {
              if (!ok[t])
                continue;
              const double ax = pts[t * 6 + 0], ay = pts[t * 6 + 1];
              const double bx = pts[t * 6 + 2], by = pts[t * 6 + 3];
              const double cx = pts[t * 6 + 4], cy = pts[t * 6 + 5];
              const double d1 = (fx - bx) * (ay - by) - (ax - bx) * (fy - by);
              const double d2 = (fx - cx) * (by - cy) - (bx - cx) * (fy - cy);
              const double d3 = (fx - ax) * (cy - ay) - (cx - ax) * (fy - ay);
              const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
              const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
              inDrawn = !(neg && pos);
            }
            // what does the LIVE hit test say here?
            mPicker->pick(px, py);
            const int gotT = mPicker->pickedTranslateHandle();
            const int gotR = mPicker->pickedRotateHandle();
            const int got = h.rotate ? gotR : gotT;
            const bool inPick = (got == h.axis + 1);
            const bool anyHandleHere = (gotT != 0) || (gotR != 0);
            if (inDrawn) { dcx += fx; dcy += fy; ++dn; }
            if (inPick) { pcx += fx; pcy += fy; ++pn; }
            if (inDrawn && inPick) ++both;
            else if (inDrawn) {
              ++drawnOnly;
              if (anyHandleHere) ++drawnPickedOther; else ++drawnPickedNothing;
            }
            else if (inPick) ++pickedOnly;
            else ++neither;
          }
        }
        const int uni = both + drawnOnly + pickedOnly;
        const double iou = uni > 0 ? static_cast<double>(both) / uni : -1.0;
        const double cdx = (dn && pn) ? (dcx / dn - pcx / pn) : 0.0;
        const double cdy = (dn && pn) ? (dcy / dn - pcy / pn) : 0.0;
        report += QString("handle axis=%1 kind=%2 tris=%3 bbox=%4,%5,%6,%7 step=%8/%9 "
                          "both=%10 drawnOnly=%11 pickedOnly=%12 neither=%13 IoU=%14 "
                          "centroidDelta=%15,%16 px drawnPickedOther=%17 drawnPickedNothing=%18\n")
                    .arg(h.axis).arg(h.rotate ? "rotate" : "translate")
                    .arg(static_cast<int>(nTri)).arg(x0).arg(y0).arg(x1).arg(y1)
                    .arg(stepX).arg(stepY)
                    .arg(both).arg(drawnOnly).arg(pickedOnly).arg(neither)
                    .arg(iou, 0, 'f', 4).arg(cdx, 0, 'f', 2).arg(cdy, 0, 'f', 2)
                    .arg(drawnPickedOther).arg(drawnPickedNothing);
      }
      QFile gf(gizCheckPath);
      if (gf.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        gf.write(report.toUtf8());
        gf.close();
      }
    }
  }

  // ===== P7 (WREN retirement): DEVICE HUD INSETS, FRAMES AND SUPERVISOR LABELS. =====
  // The other half of the five GUI surfaces the 2026-08-19 main-view flip put out: the
  // Camera / RangeFinder / Display output insets and every wb_supervisor_set_label().
  //
  // WHICH PASS, AND WHY: the same dead-last, post-tonemap slot as the overlay LINES above, and
  // for a sharper reason here -- a device inset is supposed to show the bytes the CONTROLLER
  // read. Running it through AgX, bloom and a temporal reprojection would show an AgX-graded,
  // smeared version of them, i.e. a picture that disagrees with getImage(). The quad pass loads
  // (never clears) the same private overlay output, so lines and HUD compose into one frame and
  // one present.
  //
  // COST WHEN NOTHING IS SHOWN: OmHudOverlay::anyVisible() walks the rendering-device list and
  // the label list -- a handful of pointers, no scene graph, no allocation -- and returns false.
  //
  // Hatch: OMNISIM_WGPU_HUD=0 restores the pre-P7 frame exactly. VALUE-parsed, default ON.
  static const bool sHudOn = !qEnvironmentVariableIsSet("OMNISIM_WGPU_HUD") ||
                             qEnvironmentVariableIntValue("OMNISIM_WGPU_HUD") != 0;
  if (sHudOn && present && OmHudOverlay::anyVisible()) {
    static std::vector<OmHudOverlay::Quad> hud;
    OmHudOverlay::collect(width(), height(), hud);
    OmWgpuHudQuad rq[64];
    const double dpr = width() > 0 ? static_cast<double>(W) / width() : 1.0;
    const unsigned int nq = OmHudOverlay::toRenderQuads(hud, dpr, rq, 64);
    if (nq > 0) {
      sHudCalled = 1;
      sHudQuads = nq;
      sHudOk = mWgpuRenderTarget->drawOverlayQuads(rq, nq) ? 1 : 0;
    } else {
      sHudCalled = 2;  // reached the draw site with nothing to draw -- a different failure
      sHudQuads = 0;
      sHudOk = -1;
    }
    // P7 GATE INSTRUMENT. OMNISIM_WGPU_HUD_CHECK=<path> writes, ONCE, a verdict comparing the
    // PRESENTED overlay output against the device image it was built from. This exists because
    // render_ab.py is structurally blind here: its MAINVIEW dump reads back inside the scene
    // render, upstream of this pass, so a MATCH from it says nothing about HUD pixels. Every
    // other number in this file would too -- draws= counts the SCENE list and never sees an
    // overlay pass. So the check reads the overlay texture itself.
    static const QString hudCheckPath = qEnvironmentVariable("OMNISIM_WGPU_HUD_CHECK");
    static bool hudCheckDone = false;
    if (!hudCheckDone && !hudCheckPath.isEmpty() && sHudOk == 1 && f > 30) {
      hudCheckDone = true;
      std::vector<unsigned char> rgba(static_cast<size_t>(W) * H * 4u, 0);
      const bool rb = mWgpuRenderTarget->readbackOverlayOutput(rgba.data());
      QString verdict;
      // Find the first TEXTURED quad (the frames are flat fills) and compare its interior.
      int qi = -1;
      for (unsigned int i = 0; i < nq; ++i)
        if (rq[i].pixels && rq[i].srcW > 1 && rq[i].srcH > 1) {
          qi = static_cast<int>(i);
          break;
        }
      if (!rb)
        verdict = QStringLiteral("readback=FAIL\n");
      else if (qi < 0)
        verdict = QStringLiteral("no_textured_quad\n");
      else {
        const OmWgpuHudQuad &q = rq[qi];
        // Sample a grid strictly INSIDE the inset, map each sample back to its source texel, and
        // compare. Nearest sampling + an integer-ish rect means an exact byte match is the
        // expected result, so any tolerance at all is generous.
        int samples = 0, matched = 0, maxErr = 0;
        long long sumAbs = 0;
        for (int sy = 1; sy < 8; ++sy) {
          for (int sx = 1; sx < 8; ++sx) {
            const float u = sx / 8.0f, v = sy / 8.0f;
            const int px = static_cast<int>(q.x + u * q.w);
            const int py = static_cast<int>(q.y + v * q.h);
            if (px < 0 || py < 0 || px >= W || py >= H)
              continue;
            const int tu = std::min<int>(q.srcW - 1, static_cast<int>(u * q.srcW));
            const int tv = std::min<int>(q.srcH - 1, static_cast<int>(v * q.srcH));
            const unsigned char *src = q.pixels + (static_cast<size_t>(tv) * q.srcW + tu) * 4u;
            const unsigned char *dst = rgba.data() + (static_cast<size_t>(py) * W + px) * 4u;
            // Source is BGRA, the readback is RGBA.
            const int dr = std::abs(int(dst[0]) - int(src[2]));
            const int dg = std::abs(int(dst[1]) - int(src[1]));
            const int db = std::abs(int(dst[2]) - int(src[0]));
            const int e = std::max(dr, std::max(dg, db));
            sumAbs += dr + dg + db;
            if (e > maxErr)
              maxErr = e;
            if (e <= 2)
              ++matched;
            ++samples;
          }
        }
        // RED ARM. The same samples read from the frame ONE PASS EARLIER -- exactly what the
        // pane would show with the HUD off. Reporting ctlMatched next to matched is what makes
        // the green arm mean something: an instrument that matches in both arms is measuring
        // nothing. (It is a LOWER bound on red-capability: a camera pointed at a uniform patch
        // of scene could legitimately agree with the frame behind it.)
        std::vector<unsigned char> ctl(static_cast<size_t>(W) * H * 4u, 0);
        int ctlMatched = -1;
        if (mWgpuRenderTarget->readbackOverlayOutput(ctl.data(), /*preOverlay=*/true)) {
          ctlMatched = 0;
          for (int sy = 1; sy < 8; ++sy) {
            for (int sx = 1; sx < 8; ++sx) {
              const float u = sx / 8.0f, v = sy / 8.0f;
              const int px = static_cast<int>(q.x + u * q.w);
              const int py = static_cast<int>(q.y + v * q.h);
              if (px < 0 || py < 0 || px >= W || py >= H)
                continue;
              const int tu = std::min<int>(q.srcW - 1, static_cast<int>(u * q.srcW));
              const int tv = std::min<int>(q.srcH - 1, static_cast<int>(v * q.srcH));
              const unsigned char *src = q.pixels + (static_cast<size_t>(tv) * q.srcW + tu) * 4u;
              const unsigned char *dst = ctl.data() + (static_cast<size_t>(py) * W + px) * 4u;
              const int e = std::max(std::abs(int(dst[0]) - int(src[2])),
                                     std::max(std::abs(int(dst[1]) - int(src[1])),
                                              std::abs(int(dst[2]) - int(src[0]))));
              if (e <= 2)
                ++ctlMatched;
            }
          }
        }
        verdict = QString("quad=%1 rect=%2,%3,%4,%5 src=%6x%7 samples=%8 matched=%9 maxErr=%10 "
                          "meanAbs=%11 ctlMatched=%12\n")
                    .arg(qi).arg(q.x, 0, 'f', 1).arg(q.y, 0, 'f', 1).arg(q.w, 0, 'f', 1)
                    .arg(q.h, 0, 'f', 1).arg(q.srcW).arg(q.srcH).arg(samples).arg(matched)
                    .arg(maxErr)
                    .arg(samples ? double(sumAbs) / (3.0 * samples) : 0.0, 0, 'f', 3)
                    .arg(ctlMatched);
      }
      QFile hf(hudCheckPath);
      if (hf.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        hf.write(QString("frame=%1 W=%2 H=%3 quads=%4 ok=%5\n").arg(f).arg(W).arg(H).arg(nq)
                   .arg(sHudOk).toUtf8());
        hf.write(verdict.toUtf8());
        hf.close();
      }
    }
  }

  // OmniLight loading UX: advance the fade-in, and draw the progress overlay while a bake is
  // cooking (present path only — screenshots/dumps read back before this and stay clean).
  if (mOmniBlend < 1.0f) {
    mOmniBlend = std::min(1.0f, mOmniBlend + 0.04f);
    mWgpuRenderTarget->setOmniLightBlend(mOmniBlend);
  }
  // OMNILIGHT_BAR_TEST=1 forces the overlay with a sweeping fill (visual diagnostic).
  static const bool sBarTest = qEnvironmentVariableIntValue("OMNILIGHT_BAR_TEST") == 1;
  if (present && (mOmniBakeRunning || sBarTest)) {
    const int total = mOmniProgressTotal.load(std::memory_order_relaxed);
    const int done = mOmniProgressDone.load(std::memory_order_relaxed);
    float prog = total > 0 ? std::min(1.0f, static_cast<float>(done) / total) : 0.0f;
    if (sBarTest && !mOmniBakeRunning)
      prog = static_cast<float>(f % 150) / 150.0f;
    const bool barOk = mWgpuRenderTarget->drawOmniProgress(prog, static_cast<float>(f % 3600) * 0.033f);
    static bool sBarLogged = false;
    if (!sBarLogged) {
      sBarLogged = true;
      OmLog::info(QString("[OmniLight] progress overlay %1").arg(barOk ? "active" : "UNAVAILABLE (pipeline failed)"));
    }
  }
  // Present: blit the wgpu RGBA into this window's GL framebuffer (raw GL is isolated in OmWgpuGlBlit.cpp).
  if (present) {
    // Native window-swap: GPU→GPU, no CPU pixels, no GL. (A failed acquire skips one frame.)
    mWgpuPresentSurface->presentTexture(mWgpuRenderTarget->sceneTextureView());
  } else if (!headlessFrame && OmWrenOpenGlContext::isInitialized()) {
    // (the isInitialized() guard is the GL-less degrade (shipped since D1.5): no GL context means no GL
    // blit and no swapBuffers -- the frame stays in the offscreen target, same as headlessFrame)
    OmWrenOpenGlContext::makeWrenCurrent();
    OmWgpuGlBlitRgbaToScreen(rgba.data(), W, H);
    OmWrenOpenGlContext::instance()->swapBuffers(this);
    OmWrenOpenGlContext::doneWren();
  }
  // else: W2 presentation-free frame -- the offscreen target holds the frame (and mWgpuRgba holds
  // its pixels when someone asked for them). Deliberately NOTHING here: no GL context, no swap.
  sPrevBlitMs = phaseTimer.elapsed() - tRender;
  // Only a frame that actually reached the screen may claim it did. A presentation-free frame
  // leaves this false, which is what routes the next grab through the offscreen path instead of
  // the WREN GL buffer -- and what keeps the flag readable as "the pane is live".
  mWgpuPresentedLastFrame = !headlessFrame;
  // Web stream (--stream mjpeg): the base-class feed lives inside OmWrenWindow::renderNow, which this
  // path replaces — feed it here instead, through the wgpu-aware grab. Skipped during a grab-triggered
  // re-render (grabWindowBufferNow → this function → here) to avoid recursion.
  if (!mWgpuGrabRequested) {
    OmMultimediaStreamingServer *const stream = videoStreamingServer();
    if (stream && stream->isNewFrameNeeded())
      stream->sendImage(grabWindowBufferNow());
  }
  return true;
}

QImage OmView3D::grabWindowBufferNow() {
  // Under a wgpu main view the WREN GL buffer is stale (usually empty, or holding whatever the
  // screenshot path's offScreen WREN render just drew), so grab from the wgpu render target
  // instead: one synchronous re-render with readback, so the pixels match the pane exactly (same
  // draw list, same camera, same frame).
  //
  // W2 (WREN retirement): this used to be gated on mWgpuPresentedLastFrame, and that flag is false
  // in two situations that matter -- (1) any session with no exposed window to present to
  // (--minimize / --no-window: the harness), and (2) immediately after OmView3D::renderNow's
  // offScreen branch, which the screenshot path always runs on the way here and which clears the
  // flag because renderMainFrameViaWgpu declines offScreen renders. Either way the grab silently
  // returned WREN pixels, which is why an agent's POST /world/screenshot differed from the user's
  // pane on 98.6% of pixels. So do not trust a flag: ASK. renderMainFrameViaWgpu returns false
  // cheaply -- before any allocation, before the lazy backend init -- whenever wgpu does not own
  // the main view (a WREN Viewpoint, an unavailable backend, a world reload in flight, a movie
  // recording), and we then fall through to exactly the WREN grab that ran before.
  if (mWorld && !mWgpuGrabRequested) {
    mWgpuGrabRequested = true;
    // Nothing was presented -> there is nowhere to present this grab's frame either, so render it
    // presentation-free (no child window, no surface, no GL blit, no swap). When the pane IS
    // presenting, keep the pre-W2 presenting re-render so a live windowed session is unchanged.
    mWgpuOffscreenOnly = !mWgpuPresentedLastFrame;
    const bool ok = renderMainFrameViaWgpu(true, false);
    mWgpuOffscreenOnly = false;
    mWgpuGrabRequested = false;
    const int W = std::max(1, static_cast<int>(width() * devicePixelRatio()));
    const int H = std::max(1, static_cast<int>(height() * devicePixelRatio()));
    if (ok && mWgpuRgba.size() == static_cast<size_t>(W) * H * 4) {
      const QImage img(mWgpuRgba.data(), W, H, W * 4, QImage::Format_RGBA8888);
      return img.copy().convertToFormat(QImage::Format_RGB32);
    }
    // fall through: wgpu does not own this view (or just failed and degraded it to WREN), whose
    // buffer the base grab reads
  }
  // D1.4: no WREN GL buffer to fall through to -- an empty image is the honest answer.
  return OmGlWindow::grabWindowBufferNow();
}

// P10 (WREN retirement): cheap mirror of renderMainFrameViaWgpu's early gates. True iff a
// main-view frame would render through wgpu right now. Kept in lockstep with the gate sequence at
// the top of that function (minus the recording bail-out, which is what this predicate decides).
bool OmView3D::wgpuMainViewCurrentlyActive() const {
  if (mWgpuMainViewUnavailable || mWgpuMainViewSuspended)
    return false;
  if (!mWorld || static_cast<const OmWorld *>(mWorld) != OmWorld::instance() || !mWorld->viewpoint())
    return false;
  // F1: no OMNISIM_WGPU_MAINVIEW_FORCE short-circuit any more (retired, warned no-op at the
  // render gate above) -- the backend resolution below already answers wgpu whenever it is
  // available, which is everything the force lever used to add.
  OmRenderBackend *const backend = mWorld->viewpoint()->renderBackend();
  return backend && backend->kind() == OmRenderBackendKind::Vulkan && backend->isAvailable();
}

const OmMatter *OmView3D::remoteMouseEvent(QMouseEvent *event) {
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

void OmView3D::remoteWheelEvent(QWheelEvent *event) {
  wheelEvent(event);
}

void OmView3D::selectNode(const QMouseEvent *event) {
  if (mDisabledUserInteractionsMap.value(OmAction::DISABLE_SELECTION, false))
    return;

  // Object selection:
  // - at first click select the top Matter node
  // - at second click on the same geometry select the picked Matter node
  // - further clicks on the same geometry will toggle between picked and top Matter nodes
  // exception in case of context menu shortcut where the selected Matter node is always used
  OmSelection *const selection = OmSelection::instance();
  if (!mPickedMatter) {
    selection->selectPoseFromView3D(
      NULL, mDisabledUserInteractionsMap.value(OmAction::DISABLE_OBJECT_MOVE, false));  // sending NULL allows to unselect
    if (isContextMenuShortcut(event) && event->type() == QEvent::MouseButtonRelease) {
      if (mIsRemoteMouseEvent || mDisabledUserInteractionsMap.value(OmAction::DISABLE_3D_VIEW_CONTEXT_MENU, false))
        mRemoteContextMenuMatter = mPickedMatter;
      else
        emit contextMenuRequested(event->globalPosition().toPoint(), mParentWidget);
    }
    return;
  }

  const OmAbstractPose *const selectedAbstractPose = selection->selectedAbstractPose();
  OmMatter *visiblePickedMatter = OmNodeUtilities::findUpperVisibleMatter(mPickedMatter);
  OmMatter *selectedMatter = NULL;
  if (isContextMenuShortcut(event))
    selectedMatter = visiblePickedMatter;
  else {
    const OmMatter *const previousTopMatter =
      selectedAbstractPose != NULL ? OmNodeUtilities::findUppermostMatter(selectedAbstractPose->baseNode()) : NULL;
    OmMatter *topMatter = OmNodeUtilities::findUppermostMatter(visiblePickedMatter);
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

  selection->selectPoseFromView3D(selectedMatter, mDisabledUserInteractionsMap.value(OmAction::DISABLE_OBJECT_MOVE, false));

  // OMNISIM_DEBUG is preferred; WEBOTS_DEBUG is the legacy alias.
  if (OmSysInfo::environmentVariable("OMNISIM_DEBUG").isEmpty() && OmSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty())
    OmVisualBoundingSphere::instance()->show(selectedMatter);

  if (isContextMenuShortcut(event) && event->type() == QEvent::MouseButtonRelease) {
    if (mIsRemoteMouseEvent || mDisabledUserInteractionsMap.value(OmAction::DISABLE_3D_VIEW_CONTEXT_MENU, false))
      mRemoteContextMenuMatter = selectedMatter;
    else
      emit contextMenuRequested(event->globalPosition().toPoint(), mParentWidget);
  }
}

void OmView3D::mousePressEvent(QMouseEvent *event) {
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
  OmWrenTextureOverlay *overlay = NULL;
  if (!mDragOverlay) {
    OmRenderingDevice *renderingDevice = OmRenderingDevice::fromMousePosition(position.x(), position.y());
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
          mDragOverlay = new OmDragResizeOverlayEvent(position, renderingDevice);
          connect(renderingDevice, &QObject::destroyed, this, &OmView3D::abortOverlayDrag);

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

          mDragOverlay = new OmDragTranslateOverlayEvent(position, QPoint(width(), height()), renderingDevice);
          connect(renderingDevice, &QObject::destroyed, this, &OmView3D::abortOverlayDrag);
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
  OmGlWindow::mousePressEvent(event);

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
    OmRenderingDevice *renderingDevice = OmRenderingDevice::fromMousePosition(position.x(), position.y());
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

  // Picks the OmNode and retrieves the corresponding OmGeometry
  mWorld->viewpoint()->storePickedCoordinates(OmVector3(0, 0, 0));

  bool picked = mPicker->pick(event->pos().x(), event->pos().y());
  if (picked) {
    const int id = mPicker->selectedId();

    // Check if a transformation handle was picked
    if (id == -1)
      return;

    mWorld->viewpoint()->storePickedCoordinates(mPicker->worldCoordinates());

    mPickedMatter = OmNodeUtilities::findUpperMatter(OmNode::findNode(id));
  } else
    mWorld->viewpoint()->storePickedCoordinates(mWorld->viewpoint()->position()->value());

  if (isContextMenuShortcut(event))
    return;

  // Handle bumpers
  OmTouchSensor *const touchSensor = dynamic_cast<OmTouchSensor *>(mPickedMatter);
  if (touchSensor && touchSensor->deviceType() == OmTouchSensor::BUMPER) {
    touchSensor->setGuiTouch(true);
    mTouchSensor = touchSensor;
    selectNode(event);
  }
}

void OmView3D::leaveEvent(QEvent *event) {
  setCursor(QCursor(Qt::ArrowCursor));
  if (mWheel)
    cleanupWheel();
  cleanupCameraRecognizedObjectsOverlayIfNeeded();
}

void OmView3D::mouseMoveEvent(QMouseEvent *event) {
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
        OmViewpoint *const viewpoint = mWorld->viewpoint();
        if (viewpoint && !viewpoint->isLocked() &&
            !mDisabledUserInteractionsMap.value(OmAction::LOCK_VIEWPOINT, false)) {
          const OmVector3 worldUp = OmWorld::instance()->worldInfo()->upVector();
          // Rotate around the camera's own position (not a picked rotation centre) so the camera
          // looks around in place — the FPS feel — and pass objectPicked=true to use the full
          // sensitivity rather than the 1/8 scaled-down "background drag" speed.
          OmRotateViewpointEvent::applyToViewpoint(delta, viewpoint->position()->value(), worldUp, true, viewpoint);
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
    OmRenderingDevice *const renderingDevice = OmRenderingDevice::fromMousePosition(position.x(), position.y());
    if (renderingDevice && renderingDevice->overlay()) {
      bool resizeArea = false;
      int u, v;
      renderingDevice->overlay()->convertMousePositionToIndex(position.x(), position.y(), u, v, resizeArea);
      if (OmSimulationState::instance()->isPaused()) {
        OmLog::status(renderingDevice->name() + ": " + renderingDevice->pixelInfo(u, v));
        OmCamera *camera = dynamic_cast<OmCamera *>(renderingDevice);
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
        OmLog::status("");
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
  OmRenderingDevice *const renderingDevice = OmRenderingDevice::fromMousePosition(position.x(), position.y());
  if (renderingDevice) {
    OmWrenTextureOverlay *const overlay = renderingDevice->overlay();
    if (overlay) {
      overlay->putOnTop();
      if (overlay->isInsideResizeArea(position.x(), position.y()))
        mDragOverlay = new OmDragResizeOverlayEvent(position, renderingDevice);
      else
        mDragOverlay = new OmDragTranslateOverlayEvent(position, QPoint(width(), height()), renderingDevice);
      connect(renderingDevice, &QObject::destroyed, this, &OmView3D::abortOverlayDrag);
      return;
    }
  }

  OmViewpoint *const viewpoint = mWorld->viewpoint();

  // Translate, rotate, resize events come right after overlays
  const int translateHandle = mPicker->pickedTranslateHandle(), rotateHandle = mPicker->pickedRotateHandle(),
            resizeHandle = mPicker->pickedResizeHandle();

  // Creates a new drag event according to keys (SHIFT, ALT) and buttons (LEFT, MIDDLE, RIGHT)
  const int shift = event->modifiers() & Qt::ShiftModifier;
  const int alt = event->modifiers() & Qt::AltModifier;

  int selective = !shift;
  bool resizeActive =
    OmSelection::instance()->resizeManipulatorEnabledFromSceneTree() || (event->modifiers() & Qt::ControlModifier);
  if (resizeHandle && resizeActive) {
    cleanupPhysicsDrags();

    OmBaseNode *pickedNode = OmSelection::instance()->selectedNode();
    OmGeometry *const pickedGeometry = dynamic_cast<OmGeometry *>(pickedNode);

    assert(pickedGeometry);
    if (!pickedGeometry)
      return;

    const int handleNumber = resizeHandle - 1;
    const int geometryType = pickedGeometry->nodeType();
    switch (geometryType) {
      case WB_NODE_SPHERE:
        mDragResize = new OmResizeSphereEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CYLINDER:
        if (selective)
          mDragResize = new OmResizeCylinderEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new OmRescaleCylinderEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CAPSULE:
        if (selective)
          mDragResize = new OmResizeCapsuleEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new OmRescaleCapsuleEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_BOX:
        if (selective)
          mDragResize = new OmResizeBoxEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new OmRescaleBoxEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_PLANE:
        if (selective)
          mDragResize = new OmResizePlaneEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new OmRescalePlaneEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_INDEXED_FACE_SET:
        if (selective)
          mDragResize = new OmResizeIndexedFaceSetEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new OmRescaleIndexedFaceSetEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_CONE:
        if (selective)
          mDragResize = new OmResizeConeEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new OmRescaleConeEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
      case WB_NODE_ELEVATION_GRID:
        if (selective)
          mDragResize = new OmResizeElevationGridEvent(position, viewpoint, handleNumber, pickedGeometry);
        else
          mDragResize = new OmRescaleElevationGridEvent(position, viewpoint, handleNumber, pickedGeometry);
        break;
    }
    connect(mDragResize, &OmDragResizeHandleEvent::aborted, this, &OmView3D::abortResizeDrag);
    return;
  }

  if (translateHandle) {
    cleanupPhysicsDrags();
    int handleNumber = translateHandle - 1;
    OmBaseNode *pickedNode = OmSelection::instance()->selectedNode();
    OmSolid *const pickedSolid = dynamic_cast<OmSolid *>(pickedNode);
    if (pickedSolid)
      mDragTranslate = new OmDragTranslateAlongAxisSolidEvent(position, size(), viewpoint, handleNumber, pickedSolid);
    else {
      OmAbstractPose *pickedPose = dynamic_cast<OmAbstractPose *>(pickedNode);
      assert(pickedPose);
      mDragTranslate = new OmDragTranslateAlongAxisEvent(position, size(), viewpoint, handleNumber, pickedPose);
    }
    return;
  } else if (rotateHandle) {
    cleanupPhysicsDrags();
    const int handleNumber = rotateHandle - 1;
    OmBaseNode *pickedNode = OmSelection::instance()->selectedNode();
    OmSolid *const pickedSolid = dynamic_cast<OmSolid *>(pickedNode);
    if (pickedSolid)
      mDragRotate = new OmDragRotateAroundAxisSolidEvent(position, size(), viewpoint, handleNumber, pickedSolid);
    else {
      OmAbstractPose *pickedPose = dynamic_cast<OmAbstractPose *>(pickedNode);
      assert(pickedPose);
      mDragRotate = new OmDragRotateAroundAxisEvent(position, size(), viewpoint, handleNumber, pickedPose);
    }
    return;
  }

  // Cases 1 SHIFT + CLICK
  // - LEFT CLICK  -> move the selected solid along horizontal plane
  // - RIGHT CLICK -> rotate the selected solid around world vertical axis
  // - MID CLICK   -> lift the selected solid
  if (shift) {
    if (mDisabledUserInteractionsMap.value(OmAction::DISABLE_OBJECT_MOVE, false))
      // user interaction disabled
      return;
    selectNode(event);
    const OmSelection *const selection = OmSelection::instance();
    if (!selection->isObjectMotionAllowed())
      return;

    OmBaseNode *const selectedNode = dynamic_cast<OmBaseNode *>(selection->selectedAbstractPose());
    OmPose *const uppermostPose = OmNodeUtilities::findUppermostPose(selectedNode);
    OmSolid *const uppermostSolid = OmNodeUtilities::findUppermostSolid(selectedNode);
    Qt::MouseButtons buttons = event->buttons();
    if (buttons == Qt::MiddleButton || buttons == (Qt::LeftButton | Qt::RightButton)) {
      if (uppermostSolid) {
        if (uppermostSolid->canBeTranslated())
          mDragKinematics = new OmDragVerticalSolidEvent(position, viewpoint, uppermostSolid);
      } else if (uppermostPose->canBeTranslated())
        mDragKinematics = new OmDragVerticalEvent(position, viewpoint, uppermostPose);
    } else if (buttons == Qt::LeftButton) {
      if (uppermostSolid) {
        if (uppermostSolid->canBeTranslated())
          mDragKinematics = new OmDragHorizontalSolidEvent(position, viewpoint, uppermostSolid);
      } else if (uppermostPose->canBeTranslated())
        mDragKinematics = new OmDragHorizontalEvent(position, viewpoint, uppermostPose);
    } else if (buttons == Qt::RightButton) {
      if (uppermostSolid) {
        if (uppermostSolid->canBeRotated())
          mDragVerticalAxisRotate = new OmDragRotateAroundWorldVerticalAxisSolidEvent(position, viewpoint, uppermostSolid);
      } else if (uppermostPose->canBeRotated())
        mDragVerticalAxisRotate = new OmDragRotateAroundWorldVerticalAxisEvent(position, viewpoint, uppermostPose);
    }
  } else if (alt) {
    // Case 2: ALT and CLICK -> add a force / torque to the selected solid
    if (mDisabledUserInteractionsMap.value(OmAction::DISABLE_FORCE_AND_TORQUE, false))
      // user interaction disabled
      return;

    OmNode *node = dynamic_cast<OmNode *>(mPickedMatter);
    if (!node)
      return;
    OmSolid *selectedSolid;
    while (1) {
      selectedSolid = dynamic_cast<OmSolid *>(node);
      if (selectedSolid && selectedSolid->effectiveNewtonBodyIndex() >= 0)
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
        OmSelection::instance()->disableActiveManipulator();
        mDragTorque = new OmDragTorqueEvent(size(), viewpoint, selectedSolid);
        connect(mDragTorque, &OmDragTorqueEvent::aborted, this, &OmView3D::abortPhysicsDrag);
        connect(mDragTorque, &OmDragTorqueEvent::destroyed, OmSelection::instance(), &OmSelection::restoreActiveManipulator);
      } else if (!mDragForce && forceButtonPressed) {
        OmSelection::instance()->disableActiveManipulator();
        mDragForce = new OmDragForceEvent(size(), viewpoint, selectedSolid);
        connect(mDragForce, &OmDragForceEvent::aborted, this, &OmView3D::abortPhysicsDrag);
        connect(mDragForce, &OmDragForceEvent::destroyed, OmSelection::instance(), &OmSelection::restoreActiveManipulator);
      }
    }
  } else if (!mDisabledUserInteractionsMap.value(OmAction::LOCK_VIEWPOINT, false)) {
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
      mDragKinematics = new OmTranslateViewpointEvent(position, viewpoint, scale);
    else if (buttons == Qt::MiddleButton || buttons == (Qt::LeftButton | Qt::RightButton))
      mDragKinematics = new OmZoomAndRotateViewpointEvent(position, viewpoint, 5 * scale);
    else if (buttons == Qt::LeftButton)
      mDragKinematics = new OmRotateViewpointEvent(position, viewpoint, mPicker->selectedId() != -1);
  }
}

void OmView3D::mouseDoubleClick(QMouseEvent *event) {
  if (!mWorld)
    return;

  const QPoint &mousePosition = event->pos();

  // Overlays come first
  // open external window
  OmRenderingDevice *const renderingDevice = OmRenderingDevice::fromMousePosition(mousePosition.x(), mousePosition.y());
  if (renderingDevice) {
    OmRenderingDeviceWindowFactory::instance()->showWindowForDevice(renderingDevice);
    return;
  }

  if (mDisabledUserInteractionsMap.value(OmAction::DISABLE_SELECTION, false))
    return;

  const bool picked = mPicker->pick(mousePosition.x(), mousePosition.y());
  if (picked) {
    const int id = mPicker->selectedId();
    if (id == -1)
      return;

    emit mouseDoubleClicked(event);

    OmNode *node = OmNode::findNode(id);
    OmRobot *pickedRobot = dynamic_cast<OmRobot *>(node);
    if (pickedRobot == NULL && node != NULL)
      pickedRobot = OmNodeUtilities::findRobotAncestor(node);
    if (pickedRobot)
      mPickedMatter = pickedRobot;
    else
      mPickedMatter = OmNodeUtilities::findUpperMatter(node);
  }
}

bool OmView3D::isContextMenuShortcut(const QMouseEvent *event) {
#ifdef __APPLE__
  return (event->button() == Qt::RightButton && event->modifiers() == Qt::NoModifier) ||
         (event->button() == Qt::LeftButton && event->modifiers() & Qt::MetaModifier);
#else
  return event->button() == Qt::RightButton && (event->modifiers() == Qt::NoModifier);
#endif
}

void OmView3D::mouseReleaseEvent(QMouseEvent *event) {
  OmGlWindow::mouseReleaseEvent(event);

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
      OmSelection::instance()->showResizeManipulatorFromView3D(false);
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

  const OmSimulationState *const sim = OmSimulationState::instance();
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

void OmView3D::handleModifierKey(QKeyEvent *event, bool pressed) {
  if (event->key() == Qt::Key_Control)
    enableResizeManipulator(pressed);
  else if (event->key() == Qt::Key_Shift)
    OmSelection::instance()->setUniformConstraintForResizeHandles(pressed);
}

void OmView3D::keyPressEvent(QKeyEvent *event) {
  // handle event in parent class
  if (event->key() == Qt::Key_Escape ||
      (event->modifiers() == Qt::CTRL && event->key() >= Qt::Key_0 && event->key() <= Qt::Key_4)) {
    QWindow::keyPressEvent(event);
    return;
  }

  // Numpad view-snap shortcuts (Blender convention so users coming from a DCC tool feel at home):
  //   Numpad 7 = top, Numpad 1 = north, Numpad 3 = east; hold Ctrl to flip to the opposite face.
  // The TOP_VIEW/etc. actions are already wired through OmActionManager -> OmViewpoint, so we just
  // trigger them. Swallow the event afterwards so it isn't forwarded to robot controllers.
  if (mWorld && (event->modifiers() & Qt::KeypadModifier) && !event->isAutoRepeat()) {
    const bool ctrl = event->modifiers() & Qt::ControlModifier;
    QAction *snap = NULL;
    OmActionManager *const am = OmActionManager::instance();
    switch (event->key()) {
      case Qt::Key_7:
        snap = am->action(ctrl ? OmAction::BOTTOM_VIEW : OmAction::TOP_VIEW);
        break;
      case Qt::Key_1:
        snap = am->action(ctrl ? OmAction::SOUTH_VIEW : OmAction::NORTH_VIEW);
        break;
      case Qt::Key_3:
        snap = am->action(ctrl ? OmAction::WEST_VIEW : OmAction::EAST_VIEW);
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
    OmViewpoint *const viewpoint = mWorld->viewpoint();
    if (viewpoint && viewpoint->followedSolid()) {
      followNone(true);
      OmActionManager::instance()->action(OmAction::FOLLOW_NONE)->setChecked(true);
      OmLog::status(tr("Camera follow stopped."));
    } else if (viewpoint) {
      OmSolid *const selectedSolid = OmSelection::instance()->selectedSolid();
      if (selectedSolid) {
        followTracking(true);
        OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING)->setChecked(true);
        OmLog::status(tr("Camera now tracking: %1").arg(selectedSolid->name()));
      } else {
        OmLog::status(tr("Click on an object to select it, then press F to follow."));
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
    const OmViewpoint *const viewpoint = mWorld->viewpoint();
    if (nonShift == Qt::NoModifier && viewpoint && !viewpoint->isLocked() &&
        !mDisabledUserInteractionsMap.value(OmAction::LOCK_VIEWPOINT, false)) {
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
  const int modifiers = (((event->modifiers() & Qt::SHIFT) == 0) ? 0 : OmRobot::mapSpecialKey(Qt::SHIFT)) +
#ifdef __APPLE__
                        (((event->modifiers() & Qt::META) == 0) ? 0 : OmRobot::mapSpecialKey(Qt::CTRL)) +
#else
                        (((event->modifiers() & Qt::CTRL) == 0) ? 0 : OmRobot::mapSpecialKey(Qt::CTRL)) +
#endif
                        (((event->modifiers() & Qt::ALT) == 0) ? 0 : OmRobot::mapSpecialKey(Qt::ALT));

  // cppcheck-suppress constVariablePointer
  OmRobot *const currentRobot = getCurrentRobot();
  QList<OmRobot *> robotList;
  if (currentRobot)
    robotList.append(currentRobot);
  else
    robotList = mWorld->robots();

  const int key = event->key();
  if (key != Qt::Key_Control && key != Qt::Key_Meta && key != Qt::Key_Shift && key != Qt::Key_Alt) {
    foreach (OmRobot *robot, robotList)
      robot->keyPressed(key, modifiers);
  }
  handleModifierKey(event, true);
  QWindow::keyPressEvent(event);
}

void OmView3D::keyReleaseEvent(QKeyEvent *event) {
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
    OmRobot *const currentRobot = getCurrentRobot();
    QList<OmRobot *> robotList;
    if (currentRobot)
      robotList.append(currentRobot);
    else
      robotList = mWorld->robots();

    const int key = event->key();
    if (key != Qt::Key_Control && key != Qt::Key_Meta && key != Qt::Key_Shift && key != Qt::Key_Alt) {
      foreach (OmRobot *const robot, robotList)
        robot->keyReleased(key);
    }
  }
  handleModifierKey(event, false);
  QWindow::keyReleaseEvent(event);
}

bool OmView3D::isFlyKey(int key) {
  return key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_Q ||
         key == Qt::Key_E;
}

void OmView3D::stopFly() {
  if (!mFlyKeys.isEmpty())
    mFlyKeys.clear();
  if (mFlyTimer && mFlyTimer->isActive())
    mFlyTimer->stop();
  exitFlyMouseLook();
}

void OmView3D::enterFlyMouseLook() {
  if (mFlyMouseLook || !mWorld)
    return;
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  if (!viewpoint || viewpoint->isLocked() ||
      mDisabledUserInteractionsMap.value(OmAction::LOCK_VIEWPOINT, false))
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

void OmView3D::exitFlyMouseLook() {
  if (!mFlyMouseLook)
    return;
  mFlyMouseLook = false;
  setCursor(mFlyPrevCursor);
}

void OmView3D::updateFlyCamera() {
  if (!mWorld) {
    stopFly();
    return;
  }
  OmViewpoint *const viewpoint = mWorld->viewpoint();
  if (!viewpoint || viewpoint->isLocked() ||
      mDisabledUserInteractionsMap.value(OmAction::LOCK_VIEWPOINT, false)) {
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
  const OmRotation &orientation = viewpoint->orientation()->value();
  const OmVector3 forward = orientation.direction();  // camera look direction
  const OmVector3 right = orientation.right();
  const OmVector3 worldUp = OmWorld::instance()->worldInfo()->upVector();
  OmVector3 move(0.0, 0.0, 0.0);
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

  OmSFVector3 *const position = viewpoint->position();
  position->setValue(position->value() + move * (speed * dt));
  mWorld->setModified();
  renderLater();
}

void OmView3D::enableResizeManipulator(bool enabled) {
  if (enabled && OmSelection::instance()->showResizeManipulatorFromView3D(true))
    mResizeHandlesDisabled = false;
  else {
    if (mDragResize)
      mResizeHandlesDisabled = true;
    else
      OmSelection::instance()->showResizeManipulatorFromView3D(false);
  }
}

OmRobot *OmView3D::getCurrentRobot() const {
  if (!OmSelection::instance() || !OmSelection::instance()->selectedSolid())
    return NULL;

  OmRobot *const robot = OmSelection::instance()->selectedSolid()->robot();
  if (robot)
    return robot;

  const QList<OmRobot *> &robotList = mWorld->robots();
  if (robotList.size() == 1)
    return robotList.first();

  return NULL;
}

void OmView3D::wheelEvent(QWheelEvent *event) {
  if (!mWorld)
    return;

#ifndef __APPLE__  // bug in qt on Mac: -> QWheelEvent->orientation() is wrong when SHIFT + MOUSE_WHEEL_VERTICAL_SCROLL
  // Some mouse wheels can be scrolled horizontally
  if (event->angleDelta().x() != 0)
    return;
#endif

  OmViewpoint *const viewpoint = mWorld->viewpoint();
  if (event->modifiers() & Qt::ShiftModifier) {
    if (mDisabledUserInteractionsMap.value(OmAction::DISABLE_OBJECT_MOVE, false))
      return;
    if (mWheel) {
      mWheel->apply(event->angleDelta().y());
      renderLater();
      return;
    }
    // SHIFT and WHEEL MOUSE -> lift the selected solid in the 3D View
    OmBaseNode *const selectedNode = dynamic_cast<OmBaseNode *>(OmSelection::instance()->selectedAbstractPose());
    OmSolid *const uppermostSolid = OmNodeUtilities::findUppermostSolid(selectedNode);
    if (!uppermostSolid || uppermostSolid->isLocked() || !uppermostSolid->canBeTranslated())
      return;
    mWheel = new OmWheelLiftSolidEvent(viewpoint, uppermostSolid);
    mWheel->apply(event->angleDelta().y());
    renderLater();
  } else if (!mDisabledUserInteractionsMap.value(OmAction::LOCK_VIEWPOINT, false)) {
    // WHEEL MOUSE only -> zoom
    if (mProjectionMode == OmWrenRenderingContext::PM_ORTHOGRAPHIC) {
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
    const OmVector3 zDisplacement(scaleFactor * viewpoint->orientation()->value().direction());
    OmSFVector3 *const position = viewpoint->position();
    position->setValue(position->value() + zDisplacement);
    if (!zDisplacement.isNull())
      mWorld->setModified();
    renderLater();
  }
}

// Cleanup methods

void OmView3D::cleanupEvents() {
  cleanupWheel();
  cleanupDrags();
  stopFly();
}

void OmView3D::cleanupOptionalRendering() {
}

void OmView3D::cleanupWheel() {
  delete mWheel;
  mWheel = NULL;
}

void OmView3D::cleanupCameraRecognizedObjectsOverlayIfNeeded() {
  if (mCameraUsingRecognizedObjectsOverlay) {
    mCameraUsingRecognizedObjectsOverlay->clearRecognizedObjectsOverlay();
    mCameraUsingRecognizedObjectsOverlay = NULL;
    refresh();
  }
}

void OmView3D::cleanupDrags() {
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

void OmView3D::abortPhysicsDrag() {
  cleanupPhysicsDrags();
  OmSelection::instance()->selectPoseFromView3D(NULL);
  OmLog::warning(tr("Solid out of world numeric bounds, mouse drag aborted"));
}

void OmView3D::abortResizeDrag() {
  delete mDragResize;
  mDragResize = NULL;
  OmSelection::instance()->selectPoseFromView3D(NULL);
  OmLog::warning(tr("The dimensions of the resized object exceeds world numeric bounds, mouse drag aborted"));
  if (mResizeHandlesDisabled)
    OmSelection::instance()->showResizeManipulatorFromView3D(false);
}

void OmView3D::abortOverlayDrag() {
  delete mDragOverlay;
  mDragOverlay = NULL;
}

void OmView3D::cleanupPhysicsDrags() {
  delete mDragForce;
  mDragForce = NULL;

  delete mDragTorque;
  mDragTorque = NULL;
}

void OmView3D::cleanupPickers() {
  delete mPicker;
  delete mControllerPicker;
  mPicker = NULL;
  mControllerPicker = NULL;
  mPickedMatter = NULL;
}

void OmView3D::unleashAndClean() {
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

void OmView3D::unleashPhysicsDrags() {
  const OmSimulationState *const sim = OmSimulationState::instance();
  if (sim->isPaused())
    return;

  unleashAndClean();
}
// Fast mode related methods

void OmView3D::rescaleFastModePanel() {
  // D1.4: the WREN "No Rendering" full-screen overlay died with WREN; only the state and the
  // widget/device-window enable/disable side effects survive.
}

void OmView3D::showBlackRenderingOverlay() {
  if (!mWorld || mDisabledRenderingVisible)
    return;

  disconnect(OmSimulationState::instance(), &OmSimulationState::controllerReadRequestsCompleted, this, &OmView3D::refresh);

  mDisabledRenderingVisible = true;

  mParentWidget->setEnabled(false);
  renderLater();

  OmRenderingDeviceWindowFactory::instance()->setWindowsEnabled(false);
}

void OmView3D::hideBlackRenderingOverlay() {
  if (!mWorld || !mDisabledRenderingVisible)
    return;

  connect(OmSimulationState::instance(), &OmSimulationState::controllerReadRequestsCompleted, this, &OmView3D::refresh,
          Qt::UniqueConnection);

  mDisabledRenderingVisible = false;

  mParentWidget->setEnabled(true);
  renderLater();

  OmRenderingDeviceWindowFactory::instance()->setWindowsEnabled(true);
}

void OmView3D::cleanupFullScreenOverlay() {
  mDisabledRenderingVisible = false;
}

void OmView3D::handleWorldModificationFromSupervisor() {
  const OmSimulationState *const sim = OmSimulationState::instance();
  // refresh only if simulation is paused or stepped
  if (sim->isPaused())
    refresh();
}
