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

#include "WbSimulationWorld.hpp"

#include "WbBoundingSphere.hpp"
#include "WbDownloadManager.hpp"
#include "WbLog.hpp"
#include "WbMassChecker.hpp"
#include "WbNodeOperations.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeContact.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeDebugger.hpp"
#include "WbOdeGeomData.hpp"
#include "WbBasicJoint.hpp"
#include "WbNewtonBackend.hpp"
#include "WbPaintTexture.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPhysicsBackend.hpp"
#include "WbPhysicsPlugin.hpp"
#include "WbSolid.hpp"
#include "WbPreferences.hpp"
#include "WbRadio.hpp"
#include "WbRandom.hpp"
#include "WbRobot.hpp"
#include "WbSimulationCluster.hpp"
#include "WbSimulationState.hpp"
#include "WbSoundEngine.hpp"
#include "WbTemplateManager.hpp"
#include "WbTokenizer.hpp"
#include "WbViewpoint.hpp"
#include "WbWrenRenderingContext.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QTimer>
#include <QtCore/QtGlobal>

#include <cassert>

WbSimulationWorld *WbSimulationWorld::instance() {
  return static_cast<WbSimulationWorld *>(WbWorld::instance());
}

WbSimulationWorld::WbSimulationWorld(WbTokenizer *tokenizer) :
  WbWorld(tokenizer),
  mCluster(NULL),
  mOdeContext(new WbOdeContext()),  // create ODE worlds and spaces
  mPhysicsPlugin(NULL),
  mTimer(new QTimer(this)),
  mSimulationHasRunAfterSave(false) {
  if (mWorldLoadingCanceled)
    return;

  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::LOADING_DOWNLOAD);

  emit worldLoadingStatusHasChanged(tr("Downloading assets"));
  emit worldLoadingHasProgressed(0);
  WbDownloadManager::instance()->reset();
  root()->downloadAssets();
  int progress = WbDownloadManager::instance()->progress();
  while (progress < 100) {
    QCoreApplication::processEvents(QEventLoop::WaitForMoreEvents);
    int newProgress = WbDownloadManager::instance()->progress();
    if (newProgress != progress) {
      progress = newProgress;
      emit worldLoadingHasProgressed(progress);
    }
  }

  if (log) {
    log->stopMeasure(WbPerformanceLog::LOADING_DOWNLOAD);
    log->setTimeStep(basicTimeStep());
    log->startMeasure(WbPerformanceLog::LOADING_FINALIZE);
  }

  WbSimulationState::instance()->resetTime();
  // Reset random seed to ensure reproducible simulations.
  updateRandomSeed();
  updateNumberOfThreads();
  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbSimulationWorld::updateNumberOfThreads);

  // create clusters and start threads
  mCluster = new WbSimulationCluster(mOdeContext);

  emit worldLoadingStatusHasChanged(tr("Loading plugins (if any)"));
  // see if physics plugin is required
  const QString &physics = worldInfo()->physics();
  if (!physics.isEmpty()) {
    // try to load physics plugin and resolve functions
    mPhysicsPlugin = new WbPhysicsPlugin(physics);
    if (!mPhysicsPlugin->load()) {
      // load has failed: destroy the plugin
      delete mPhysicsPlugin;
      mPhysicsPlugin = NULL;
    }
  }

  setIsLoading(true);
  root()->finalize();
  finalize();
  setIsLoading(false);

  if (mWorldLoadingCanceled)
    return;

  WbMassChecker::instance()->checkMasses();

  emit worldLoadingStatusHasChanged(tr("Initializing plugins (if any)"));

  if (mPhysicsPlugin) {
    mPhysicsPlugin->connectReceiver(worldInfo()->physicsReceiver());
    mPhysicsPlugin->init();
  }

  mCluster->handleInitialCollisions();

  WbPaintTexture::init();
  WbRadio::createAndSetupPluginObjects();

  WbSoundEngine::setWorld(this);

  if (log) {
    log->stopMeasure(WbPerformanceLog::LOADING_FINALIZE);
    log->stopMeasure(WbPerformanceLog::LOADING);
  }

  mTimer->setTimerType(Qt::PreciseTimer);
  connect(mTimer, &QTimer::timeout, this, &WbSimulationWorld::triggerStepFromTimer);
  const WbSimulationState *const s = WbSimulationState::instance();
  connect(s, &WbSimulationState::rayTracingEnabled, this, &WbSimulationWorld::rayTracingEnabled);
  connect(s, &WbSimulationState::modeChanged, this, &WbSimulationWorld::modeChanged);
  modeChanged();
  connect(this, &WbSimulationWorld::physicsStepStarted, s, &WbSimulationState::physicsStepStarted);
  connect(this, &WbSimulationWorld::physicsStepEnded, s, &WbSimulationState::physicsStepEnded);
  connect(this, &WbSimulationWorld::cameraRenderingStarted, s, &WbSimulationState::cameraRenderingStarted);
  connect(worldInfo(), &WbWorldInfo::optimalThreadCountChanged, this, &WbSimulationWorld::updateNumberOfThreads);
  connect(worldInfo(), &WbWorldInfo::randomSeedChanged, this, &WbSimulationWorld::updateRandomSeed);

  if (WbTokenizer::worldFileVersion() < WbVersion(2021, 1, 1))
    WbLog::info(tr("You are using a world from an old version of OmniSim. The backwards compability algorithm will try to "
                   "convert it."));

  WbNodeUtilities::fixBackwardCompatibility(WbWorld::instance()->root());
}

WbSimulationWorld::~WbSimulationWorld() {
  setIsCleaning(true);

  // Tear down the Newton backend's per-world state FIRST. The backend is a
  // process singleton: without this, a world RELOAD left its `running` flag
  // true, so the next world's ensureWorldOpen() no-opped, every
  // addBody/addShape registration was silently rejected (robots fell back to
  // ODE and exploded within seconds) and the ORPHANED old Newton world kept
  // stepping invisibly.
  WbNewtonBackend *const newtonToTearDown = static_cast<WbNewtonBackend *>(WbPhysicsBackendRegistry::newtonBackend());
  if (newtonToTearDown != nullptr)
    newtonToTearDown->teardownWorld();

  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->worldClosed(fileName(), logWorldMetrics());

  delete mTimer;

  WbPaintTexture::cleanup();
  WbSoundEngine::setWorld(NULL);

  // this must be done before deleting mCluster, because ODE
  // objects must be still existing during webots_physics_cleanup()
  delete mPhysicsPlugin;

  // this will destroy all the geoms and bodies
  // this must be done before deleting the space and world (right below)
  root()->deleteAllChildren();

  // this must be called after the nodes have been destroyed
  delete mCluster;

  // delete (spaces, worlds) etc.
  delete mOdeContext;

  mAddedNode.clear();

  setIsCleaning(false);
}

void WbSimulationWorld::step() {
  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->stepChanged();

  const double timeStep = basicTimeStep();

  // Update the timer's timestep if it's been changed
  if (timeStep != mOldTimeStep && WbSimulationState::instance()->isRealTime()) {
    mOldTimeStep = timeStep;
    mTimer->start(timeStep);
  }

  emit physicsStepStarted();

  if (log)
    log->startMeasure(WbPerformanceLog::PRE_PHYSICS_STEP);
  mOdeContacts.clear();
  const int size = mImmersionGeoms.size();
  for (int i = 0; i < size; ++i)
    dImmersionOutlineDestroy(mImmersionGeoms.at(i).outline);
  mImmersionGeoms.clear();

  foreach (WbRobot *const robot, robots()) {
    if (robots().contains(robot))  // the 'processImmediateMessages' of another robot may have removed/regenerated this robot
      robot->processImmediateMessages();
  }

  // TODO: this should be removed using the WbSimulationState::signals
  const QList<WbSolid *> &l = topSolids();
  foreach (WbSolid *const solid, l)
    solid->prePhysicsStep(timeStep);

  WbPaintTexture::prePhysicsStep(timeStep);

  if (mPhysicsPlugin)
    mPhysicsPlugin->step();

  WbRadio::runPlugin(timeStep);
  if (log) {
    log->stopMeasure(WbPerformanceLog::PRE_PHYSICS_STEP);
    log->startMeasure(WbPerformanceLog::PHYSICS_STEP);
  }
  // P3.2 of cuda-newton-physics-plan.md: drive the Newton solver
  // alongside ODE. Lazy-finalize the Newton model on the first tick
  // (postFinalize already added the bodies); subsequent ticks just
  // call step(dt). On NEWTON=OFF builds or worlds with no Newton
  // solids, isWorldOpenForBuild()/isWorldRunning() both return false
  // and these branches are no-ops -- pure-ODE worlds carry zero per-
  // tick overhead from this dispatcher.
  WbNewtonBackend *const newton = static_cast<WbNewtonBackend *>(WbPhysicsBackendRegistry::newtonBackend());
  if (newton != nullptr) {
    // P3.7: solid registration must precede joint registration. Both
    // are deferred from postFinalize so matrix() returns world-correct
    // poses (parent transform chain is fully wired by step time).
    WbSolid::flushPendingNewtonRegistrations();
    if (newton->isWorldOpenForBuild()) {
      WbBasicJoint::flushPendingNewtonRegistrations();
      newton->finalizeWorld();
    }
    if (newton->isWorldRunning()) {
      // P3.8.b: push controller-side motor velocity commands through
      // to Newton just before the solver step. No-op if no motorized
      // hinges are Newton-backed.
      WbBasicJoint::pushNewtonMotorTargets();
      newton->step(timeStep / 1000.0);  // basicTimeStep is ms; Newton solver wants seconds
    }
  }

  mCluster->step();
  if (log) {
    log->stopMeasure(WbPerformanceLog::PHYSICS_STEP);
    log->startMeasure(WbPerformanceLog::POST_PHYSICS_STEP);
  }

  WbSoundEngine::updateAfterPhysicsStep();

  WbOdeDebugger::instance()->step();

  // give a chance to read the dJointFeedback structs
  if (mPhysicsPlugin)
    mPhysicsPlugin->stepEnd();

  // call postPhysicsStep on all Solids to assign new coordinates
  foreach (WbSolid *const solid, l)
    solid->postPhysicsStep();

  emit physicsStepEnded();

  if (mResetRequested) {
    WbWorld::instance()->reset(false);
    emit resetRequested(false);
  }

  WbSimulationState::instance()->increaseTime(timeStep);
  viewpoint()->updateFollowUp();

  if (log)
    log->stopMeasure(WbPerformanceLog::POST_PHYSICS_STEP);

  // update camera textures before main rendering
  const QList<WbRobot *> robotList = robots();
  for (int i = 0; i < robots().size(); ++i) {
    WbRobot *robot = robotList[i];
    if (robot->isControllerStarted())
      robot->renderCameras();
  }

  if (!mSimulationHasRunAfterSave) {
    mSimulationHasRunAfterSave = true;
    emit simulationStartedAfterSave(true);
  }
}

void WbSimulationWorld::pauseStepTimer() {
  mTimer->stop();
}

void WbSimulationWorld::restartStepTimer() {
  const WbSimulationState::Mode mode = WbSimulationState::instance()->mode();
  if (!mTimer->isActive() && (mode == WbSimulationState::REALTIME || mode == WbSimulationState::FAST))
    mTimer->start();
}

void WbSimulationWorld::modeChanged() {
  WbPerformanceLog *log = WbPerformanceLog::instance();
  const double timeStep = basicTimeStep();

  const WbSimulationState::Mode mode = WbSimulationState::instance()->mode();
  switch (mode) {
    case WbSimulationState::PAUSE:
      mTimer->stop();
      WbSoundEngine::setPause(true);
      WbSoundEngine::setMute(WbPreferences::instance()->value("Sound/mute").toBool());
      if (log)
        log->stopMeasure(WbPerformanceLog::SPEED_FACTOR);
      break;
    case WbSimulationState::STEP:
      WbSoundEngine::setMute(WbPreferences::instance()->value("Sound/mute").toBool());
      break;
    case WbSimulationState::REALTIME:
      WbSoundEngine::setPause(false);
      WbSoundEngine::setMute(WbPreferences::instance()->value("Sound/mute").toBool());
      mOldTimeStep = timeStep;
      mTimer->start(timeStep);
      break;
    case WbSimulationState::FAST:
      WbSoundEngine::setPause(false);
      WbSoundEngine::setMute(true);
      mTimer->start(0);  // the 0 argument here is important, don't remove it
      break;
    case WbSimulationState::NONE:
      assert(false);
      break;
  }
}

void WbSimulationWorld::propagateBoundingObjectMaterialUpdate(bool onMenuAction) {
  const QList<WbSolid *> &l = topSolids();
  foreach (WbSolid *const solid, l)
    solid->propagateBoundingObjectMaterialUpdate(onMenuAction);
}

void WbSimulationWorld::checkNeedForBoundingMaterialUpdate() {
  const WbWrenRenderingContext *const context = WbWrenRenderingContext::instance();
  const int showAllBoundingObjects = context->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS);

  if (!showAllBoundingObjects && !WbSimulationState::instance()->isRendering())
    return;

  if (showAllBoundingObjects && WbSimulationState::instance()->isRendering()) {
    connect(this, &WbSimulationWorld::physicsStepEnded, this, &WbSimulationWorld::propagateBoundingObjectUpdate,
            Qt::UniqueConnection);
    propagateBoundingObjectMaterialUpdate(true);
  } else
    disconnect(this, &WbSimulationWorld::physicsStepEnded, this, &WbSimulationWorld::propagateBoundingObjectUpdate);
}

void WbSimulationWorld::rayTracingEnabled() {
  WbBoundingSphere::enableUpdates(true, root()->boundingSphere());
}

void WbSimulationWorld::updateNumberOfThreads() {
  int numberOfthreads =
    qMin(WbPreferences::instance()->value("General/numberOfThreads", 1).toInt(), WbWorld::instance()->optimalThreadCount());
  mOdeContext->setNumberOfThreads(numberOfthreads);
}

void WbSimulationWorld::updateRandomSeed() {
  assert(worldInfo());
  int seed = worldInfo()->randomSeed();
  if (seed < 0)
    seed = QDateTime::currentMSecsSinceEpoch() + QCoreApplication::applicationPid();
  WbRandom::setSeed(seed);  // Webots random seed
  dRandSetSeed(seed);       // ODE random seed
}

void WbSimulationWorld::storeLastSaveTime() {
  mSimulationHasRunAfterSave = false;
}

bool WbSimulationWorld::simulationHasRunAfterSave() {
  return mSimulationHasRunAfterSave;
}

void WbSimulationWorld::reset(bool restartControllers) {
  WbWorld::reset(restartControllers);
  WbSimulationState::instance()->pauseSimulation();
  WbSimulationState::instance()->resetTime();
  WbTemplateManager::instance()->blockRegeneration(true);
  if (mPhysicsPlugin && mPhysicsPlugin->isLoaded())
    mPhysicsPlugin->cleanup();
  foreach (WbNode *node, mAddedNode) {
    if (mAddedNode.contains(node))  // the node may already have been remove if it is a children of another previously removed
      WbNodeOperations::instance()->deleteNode(node, true);
  }
  mAddedNode.clear();
  root()->reset("__init__");
  WbTemplateManager::instance()->blockRegeneration(false);
  mImmersionGeoms.clear();
  mCluster->handleInitialCollisions();
  dImmersionLinkGroupEmpty(mCluster->immersionLinkGroup());

  // P7 reset hook: give each registered physics backend a chance to
  // flush world-scope state that the per-Solid reset cascade missed.
  // ODE inherits the default no-op; Newton re-runs eval_fk through
  // resetJointsToDefaults() so any fixed-base solids' joint_q gets
  // back to the model's initial values even when their chassis
  // doesn't fire positionChangedArtificially.
  WbPhysicsBackendRegistry::odeBackend()->reset();
  WbPhysicsBackendRegistry::newtonBackend()->reset();
  WbSoundEngine::stopAllSources();
  if (restartControllers) {
    foreach (WbRobot *const robot, robots()) {
      if (robot->isControllerStarted())
        robot->restartController();
    }
  }
  updateRandomSeed();
  if (WbDownloadManager::instance()->progress() == 100)
    WbSimulationState::instance()->resumeSimulation();
  if (mPhysicsPlugin)
    mPhysicsPlugin->init();
  storeLastSaveTime();
  setModified(false);
}

void WbSimulationWorld::storeAddedNodeIfNeeded(WbNode *node) {
  mAddedNode << node;
  connect(node, &QObject::destroyed, this, &WbSimulationWorld::removeNodeFromAddedNodeList);
}

void WbSimulationWorld::removeNodeFromAddedNodeList(const QObject *node) {
  const WbNode *n = static_cast<const WbNode *>(node);
  mAddedNode.removeAll(n);
}

bool WbSimulationWorld::saveAs(const QString &fileName) {
  mAddedNode.clear();
  return WbWorld::saveAs(fileName);
}
