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

#include "OmSimulationWorld.hpp"

#include "OmBaseNode.hpp"
#include "OmBoundingSphere.hpp"
#include "OmDownloadManager.hpp"
#include "OmLog.hpp"
#include "OmMassChecker.hpp"
#include "OmMotor.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeUtilities.hpp"
#include "OmBasicJoint.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPaintTexture.hpp"
#include "OmPerformanceLog.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmSolid.hpp"
#include "OmPreferences.hpp"
#include "OmRandom.hpp"
#include "OmRobot.hpp"
#include "OmSFString.hpp"
#include "OmSimulationState.hpp"
#include "OmSoundEngine.hpp"
#include "OmTemplateManager.hpp"
#include "OmTokenizer.hpp"
#include "OmViewpoint.hpp"
#include "OmWrenRenderingContext.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QTimer>
#include <QtCore/QtGlobal>

#include <cassert>
#include <chrono>

OmSimulationWorld *OmSimulationWorld::instance() {
  return static_cast<OmSimulationWorld *>(OmWorld::instance());
}

OmSimulationWorld::OmSimulationWorld(OmTokenizer *tokenizer) :
  OmWorld(tokenizer),
  mTimer(new QTimer(this)),
  mSimulationHasRunAfterSave(false) {
  if (mWorldLoadingCanceled)
    return;

  OmPerformanceLog *log = OmPerformanceLog::instance();
  if (log)
    log->startMeasure(OmPerformanceLog::LOADING_DOWNLOAD);

  emit worldLoadingStatusHasChanged(tr("Downloading assets"));
  emit worldLoadingHasProgressed(0);
  OmDownloadManager::instance()->reset();
  root()->downloadAssets();
  int progress = OmDownloadManager::instance()->progress();
  while (progress < 100) {
    QCoreApplication::processEvents(QEventLoop::WaitForMoreEvents);
    int newProgress = OmDownloadManager::instance()->progress();
    if (newProgress != progress) {
      progress = newProgress;
      emit worldLoadingHasProgressed(progress);
    }
  }

  if (log) {
    log->stopMeasure(OmPerformanceLog::LOADING_DOWNLOAD);
    log->setTimeStep(basicTimeStep());
    log->startMeasure(OmPerformanceLog::LOADING_FINALIZE);
  }

  OmSimulationState::instance()->resetTime();
  // Reset random seed to ensure reproducible simulations.
  updateRandomSeed();


  emit worldLoadingStatusHasChanged(tr("Loading plugins (if any)"));

  setIsLoading(true);
  // Headless texture-decode gate: decide BEFORE finalize whether any node in
  // this world needs decoded texture PIXELS for its simulation output --
  // Camera devices (offscreen wgpu render), infra-red DistanceSensor (reads
  // the red channel of the hit surface, see OmDistanceSensor::updateRaySetup
  // / computeValue), and Pen (paints into the texture). Everything else only
  // needs textures to DRAW, which a --no-rendering / --no-window process never
  // does; OmImageTexture::updateUrl consults OmWorld::needsTextures() through
  // this flag and skips the decode during loading. One O(nodes) pass, the same
  // subNodes(true) walk OmNodeUtilities::hasDescendantNodesOfType uses. The
  // DistanceSensor `type` field is an ordinary SFString parsed at
  // construction, so it is resolvable here, pre-finalize; the string->type
  // mapping below mirrors OmDistanceSensor::updateRaySetup exactly.
  {
    bool needsTextures = false;
    const QList<OmNode *> allNodes = root()->subNodes(true);
    foreach (const OmNode *n, allNodes) {
      const OmBaseNode *baseNode = dynamic_cast<const OmBaseNode *>(n);
      if (baseNode == NULL)
        continue;
      const int type = baseNode->nodeType();
      if (type == WB_NODE_CAMERA || type == WB_NODE_PEN) {
        needsTextures = true;
        break;
      }
      if (type == WB_NODE_DISTANCE_SENSOR) {
        const OmSFString *const typeField = n->findSFString("type");
        const QString typeValue = typeField ? typeField->value() : QString();
        if (typeValue.compare("infra-red", Qt::CaseInsensitive) == 0 ||
            typeValue.compare("infrared", Qt::CaseInsensitive) == 0) {
          needsTextures = true;
          break;
        }
      }
    }
    setNeedsTextures(needsTextures);
    // 2026-09-02: value-parsed. This read was presence-gated, so `OMNISIM_EAGER_TEXTURE_DECODE=0`
    // ARMED the revert (the OMNISIM_REQUIRE_NEWTON trap); unset/empty/"0"/"false"/"off" now keep the
    // gate and anything else reverts. Same parse as OmImageTexture.cpp's eagerTextureDecodeForced().
    const QString eager = QString::fromUtf8(qgetenv("OMNISIM_EAGER_TEXTURE_DECODE")).trimmed().toLower();
    const bool eagerDecodeForced = !(eager.isEmpty() || eager == "0" || eager == "false" || eager == "off");
    if (!needsTextures && OmSimulationState::instance()->startedWithoutRendering() && !eagerDecodeForced)
      OmLog::info(tr("headless load: eager texture decode skipped (no Camera/IR-sensor/Pen in world). "
                     "OMNISIM_EAGER_TEXTURE_DECODE=1 reverts."));
  }
  root()->finalize();
  finalize();
  setIsLoading(false);

  if (mWorldLoadingCanceled)
    return;

  OmMassChecker::instance()->checkMasses();

  emit worldLoadingStatusHasChanged(tr("Initializing plugins (if any)"));


  OmPaintTexture::init();

  OmSoundEngine::setWorld(this);

  if (log) {
    log->stopMeasure(OmPerformanceLog::LOADING_FINALIZE);
    log->stopMeasure(OmPerformanceLog::LOADING);
  }

  mTimer->setTimerType(Qt::PreciseTimer);
  connect(mTimer, &QTimer::timeout, this, &OmSimulationWorld::triggerStepFromTimer);
  const OmSimulationState *const s = OmSimulationState::instance();
  connect(s, &OmSimulationState::rayTracingEnabled, this, &OmSimulationWorld::rayTracingEnabled);
  connect(s, &OmSimulationState::modeChanged, this, &OmSimulationWorld::modeChanged);
  modeChanged();
  connect(this, &OmSimulationWorld::physicsStepStarted, s, &OmSimulationState::physicsStepStarted);
  connect(this, &OmSimulationWorld::physicsStepEnded, s, &OmSimulationState::physicsStepEnded);
  connect(this, &OmSimulationWorld::cameraRenderingStarted, s, &OmSimulationState::cameraRenderingStarted);
  connect(worldInfo(), &OmWorldInfo::randomSeedChanged, this, &OmSimulationWorld::updateRandomSeed);

  if (OmTokenizer::worldFileVersion() < OmVersion(2021, 1, 1))
    OmLog::info(tr("You are using a world from an old version of OmniSim. The backwards compability algorithm will try to "
                   "convert it."));

  OmNodeUtilities::fixBackwardCompatibility(OmWorld::instance()->root());
}

OmSimulationWorld::~OmSimulationWorld() {
  setIsCleaning(true);

  // Tear down the Newton backend's per-world state FIRST. The backend is a
  // process singleton: without this, a world RELOAD left its `running` flag
  // true, so the next world's ensureWorldOpen() no-opped, every
  // addBody/addShape registration was silently rejected (robots fell back to
  // ODE and exploded within seconds) and the ORPHANED old Newton world kept
  // stepping invisibly.
  OmNewtonBackend *const newtonToTearDown = static_cast<OmNewtonBackend *>(OmPhysicsBackendRegistry::newtonBackend());
  if (newtonToTearDown != nullptr)
    newtonToTearDown->teardownWorld();
  // The next world must re-run the registration flush from scratch even if its
  // Solids happen to reuse this one's generation number.
  OmSolid::bumpNewtonFlushGeneration();

  OmPerformanceLog *log = OmPerformanceLog::instance();
  if (log)
    log->worldClosed(fileName(), logWorldMetrics());

  delete mTimer;

  OmPaintTexture::cleanup();
  OmSoundEngine::setWorld(NULL);

  // this will destroy all the geoms and bodies
  // this must be done before deleting the space and world (right below)
  root()->deleteAllChildren();


  mAddedNode.clear();

  setIsCleaning(false);
}

void OmSimulationWorld::step() {
  OmPerformanceLog *log = OmPerformanceLog::instance();
  if (log)
    log->stepChanged();

  const double timeStep = basicTimeStep();

  // Update the timer's timestep if it's been changed
  if (timeStep != mOldTimeStep && OmSimulationState::instance()->isRealTime()) {
    mOldTimeStep = timeStep;
    mTimer->start(timeStep);
  }

  // W1.7 mid-run physics rebuild -- consumed BEFORE physicsStepStarted so
  // any producer that registers from that signal would land in the fresh
  // world (v1 refuses particle worlds at request time regardless). The
  // rebuild completes and the world steps within this same call: teardown
  // here, re-registration in the flush below, finalize right after it.
  if (mNewtonRebuildRequested) {
    mNewtonRebuildRequested = false;
    OmNewtonBackend *const nb =
      static_cast<OmNewtonBackend *>(OmPhysicsBackendRegistry::newtonBackend());
    if (nb != nullptr && nb->isAvailable()) {
      OmLog::info(QObject::tr("[OmNewtonBackend] mid-run physics rebuild: tearing down the live "
                              "Newton world and re-registering the scene at its CURRENT poses."));
      OmSolid::captureNewtonVelocitiesForRebuild();
      nb->teardownWorld();
      OmSolid::resetNewtonRegistrationsForRebuild();
      OmBasicJoint::requeueAllNewtonJointsForRebuild();
      OmSolid::bumpNewtonFlushGeneration();
    }
  }

  emit physicsStepStarted();

  if (log)
    log->startMeasure(OmPerformanceLog::PRE_PHYSICS_STEP);

  foreach (OmRobot *const robot, robots()) {
    if (robots().contains(robot))  // the 'processImmediateMessages' of another robot may have removed/regenerated this robot
      robot->processImmediateMessages();
  }

  // TODO: this should be removed using the OmSimulationState::signals
  const QList<OmSolid *> &l = topSolids();
  foreach (OmSolid *const solid, l)
    solid->prePhysicsStep(timeStep);

  OmPaintTexture::prePhysicsStep(timeStep);

  if (log) {
    log->stopMeasure(OmPerformanceLog::PRE_PHYSICS_STEP);
    log->startMeasure(OmPerformanceLog::PHYSICS_STEP);
  }
  // Start of the PHYSICS_STEP bracket on the sub-profiler's clock (see the
  // profBracket block below). Cheap enough to take unconditionally.
  const std::chrono::steady_clock::time_point mPhysBracketStart = std::chrono::steady_clock::now();
  // P3.2 of cuda-newton-physics-plan.md: drive the Newton solver. Lazy-finalize
  // the Newton model on the first tick (postFinalize already added the bodies);
  // subsequent ticks just call step(dt). On NEWTON=OFF builds or worlds with no
  // Newton solids, isWorldOpenForBuild()/isWorldRunning() both return false and
  // these branches are no-ops.
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(OmPhysicsBackendRegistry::newtonBackend());

  // ⚠ RETIRING THE SILENT ODE FALL-BACK. When the Newton runtime is INSTALLED
  // but will not come up, `newtonBackend()` hands back nullptr, every branch
  // below is skipped, and the world runs to completion under a log that says
  // Newton was asked for -- historically on ODE, and since ODE's deletion on
  // nothing at all. Measured 2026-08-05 on the cold-launch defect:
  // 5 of 10 launches of a `defaultPhysicsBackend "newton"` world came back that
  // way and nothing downstream could tell them apart from Newton runs. A world
  // simulated by a backend nobody chose is a wrong result, not a degraded one.
  //
  // This is the only site that runs in that case -- the checks inside the
  // registration flushes all sit behind a non-null backend, i.e. behind the
  // very failure they were meant to catch, and the backend's own constructor
  // runs before the world is parsed (refusing there rejected explicit-ODE
  // worlds, 4/4 measured). Here the world IS parsed and no backend is needed.
  //
  // Inert unless the runtime is installed-and-broken.
  OmNewtonBackend::refuseIfNewtonBroken();
  if (newton != nullptr) {
    // Physics-bracket sub-profile (OMNISIM_NEWTON_STEP_PROFILE=1, same gate as
    // the Python-side split). Exists because differencing the Python profile
    // against the engine bucket left ~0.26 ms/step at 5 bodies unattributed on
    // the C++ side, and "unattributed" is exactly how the first optimisation
    // campaign mis-aimed: it is not knowledge until something times it.
    // Accumulators are ms; reported every 500 ticks as a per-tick mean.
    static const bool profBracket = !qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_STEP_PROFILE");
    static double accFlush = 0.0, accMotor = 0.0, accStep = 0.0, accWhole = 0.0;
    static long long profTicks = 0;
    using profClock = std::chrono::steady_clock;
    const auto t0 = profBracket ? profClock::now() : profClock::time_point();

    // P3.7: solid registration must precede joint registration. Both
    // are deferred from postFinalize so matrix() returns world-correct
    // poses (parent transform chain is fully wired by step time).
    OmSolid::flushPendingNewtonRegistrations();
    if (newton->isWorldOpenForBuild()) {
      OmBasicJoint::flushPendingNewtonRegistrations();
      newton->finalizeWorld();
    }
    const auto t1 = profBracket ? profClock::now() : profClock::time_point();
    if (newton->isWorldRunning()) {
      // P3.8.b: push controller-side motor velocity commands through
      // to Newton just before the solver step. No-op if no motorized
      // hinges are Newton-backed.
      OmBasicJoint::pushNewtonMotorTargets();
      const auto t2 = profBracket ? profClock::now() : profClock::time_point();
      newton->step(timeStep / 1000.0);  // basicTimeStep is ms; Newton solver wants seconds
      if (profBracket) {
        const auto t3 = profClock::now();
        accFlush += std::chrono::duration<double, std::milli>(t1 - t0).count();
        accMotor += std::chrono::duration<double, std::milli>(t2 - t1).count();
        accStep += std::chrono::duration<double, std::milli>(t3 - t2).count();
        // Whole bracket on the SAME clock, so "sum of parts vs whole" is a
        // like-for-like comparison and cannot be an artefact of the engine
        // bucket's own units (OmPerformanceLog accumulates microseconds).
        accWhole += std::chrono::duration<double, std::milli>(t3 - mPhysBracketStart).count();
        if ((++profTicks % 500) == 0)
          OmLog::info(QString("[OmNewtonBackend] bracket profile over %1 ticks: "
                              "flushRegistrations %2 + motorTargets %3 + newton->step %4 "
                              "= %5 accounted, whole bracket %6 ms/tick")
                        .arg(profTicks)
                        .arg(accFlush / profTicks, 0, 'f', 4)
                        .arg(accMotor / profTicks, 0, 'f', 4)
                        .arg(accStep / profTicks, 0, 'f', 4)
                        .arg((accFlush + accMotor + accStep) / profTicks, 0, 'f', 4)
                        .arg(accWhole / profTicks, 0, 'f', 4));
      }
    }
  }

  if (log) {
    log->stopMeasure(OmPerformanceLog::PHYSICS_STEP);
    log->startMeasure(OmPerformanceLog::POST_PHYSICS_STEP);
  }

  OmSoundEngine::updateAfterPhysicsStep();

  // call postPhysicsStep on all Solids to assign new coordinates
  foreach (OmSolid *const solid, l)
    solid->postPhysicsStep();

  emit physicsStepEnded();

  if (mResetRequested) {
    OmWorld::instance()->reset(false);
    emit resetRequested(false);
  }

  OmSimulationState::instance()->increaseTime(timeStep);
  viewpoint()->updateFollowUp();

  if (log)
    log->stopMeasure(OmPerformanceLog::POST_PHYSICS_STEP);

  // update camera textures before main rendering
  const QList<OmRobot *> robotList = robots();
  for (int i = 0; i < robots().size(); ++i) {
    OmRobot *robot = robotList[i];
    if (robot->isControllerStarted())
      robot->renderCameras();
  }

  if (!mSimulationHasRunAfterSave) {
    mSimulationHasRunAfterSave = true;
    emit simulationStartedAfterSave(true);
  }
}

int OmSimulationWorld::requestNewtonRebuild(QString *whyNot) {
  OmNewtonBackend *const nb =
    static_cast<OmNewtonBackend *>(OmPhysicsBackendRegistry::newtonBackend());
  const auto refuse = [whyNot](const QString &why) {
    if (whyNot != nullptr)
      *whyNot = why;
    return -1;
  };
  if (nb == nullptr || !nb->isAvailable())
    return refuse(QObject::tr("no Newton backend is available in this process"));
  if (!nb->isWorldRunning())
    return refuse(QObject::tr("the Newton world is not running (still building, or it never "
                              "finalised -- a world with no physics cannot be rebuilt)"));
  if (nb->particleCount() > 0)
    return refuse(QObject::tr("this world simulates Cloth/SoftBody particles, which re-register "
                              "from their AUTHORED rest state -- a mid-run rebuild would snap "
                              "them back. Reload the world instead."));
  if (nb->granularBedCount() > 0)
    return refuse(QObject::tr("this world simulates a GranularBed, which re-registers from its "
                              "AUTHORED state -- a mid-run rebuild would reset it. Reload the "
                              "world instead."));
  mNewtonRebuildRequested = true;
  return 0;
}

void OmSimulationWorld::pauseStepTimer() {
  mTimer->stop();
}

void OmSimulationWorld::restartStepTimer() {
  const OmSimulationState::Mode mode = OmSimulationState::instance()->mode();
  if (!mTimer->isActive() && (mode == OmSimulationState::REALTIME || mode == OmSimulationState::FAST))
    mTimer->start();
}

void OmSimulationWorld::modeChanged() {
  OmPerformanceLog *log = OmPerformanceLog::instance();
  const double timeStep = basicTimeStep();

  const OmSimulationState::Mode mode = OmSimulationState::instance()->mode();
  switch (mode) {
    case OmSimulationState::PAUSE:
      mTimer->stop();
      OmSoundEngine::setPause(true);
      OmSoundEngine::setMute(OmPreferences::instance()->value("Sound/mute").toBool());
      if (log)
        log->stopMeasure(OmPerformanceLog::SPEED_FACTOR);
      break;
    case OmSimulationState::STEP:
      OmSoundEngine::setMute(OmPreferences::instance()->value("Sound/mute").toBool());
      break;
    case OmSimulationState::REALTIME:
      OmSoundEngine::setPause(false);
      OmSoundEngine::setMute(OmPreferences::instance()->value("Sound/mute").toBool());
      mOldTimeStep = timeStep;
      mTimer->start(timeStep);
      break;
    case OmSimulationState::FAST:
      OmSoundEngine::setPause(false);
      OmSoundEngine::setMute(true);
      mTimer->start(0);  // the 0 argument here is important, don't remove it
      break;
    case OmSimulationState::NONE:
      assert(false);
      break;
  }
}

void OmSimulationWorld::propagateBoundingObjectMaterialUpdate(bool onMenuAction) {
  const QList<OmSolid *> &l = topSolids();
  foreach (OmSolid *const solid, l)
    solid->propagateBoundingObjectMaterialUpdate(onMenuAction);
}

void OmSimulationWorld::checkNeedForBoundingMaterialUpdate() {
  const OmWrenRenderingContext *const context = OmWrenRenderingContext::instance();
  const int showAllBoundingObjects = context->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS);

  if (!showAllBoundingObjects && !OmSimulationState::instance()->isRendering())
    return;

  if (showAllBoundingObjects && OmSimulationState::instance()->isRendering()) {
    connect(this, &OmSimulationWorld::physicsStepEnded, this, &OmSimulationWorld::propagateBoundingObjectUpdate,
            Qt::UniqueConnection);
    propagateBoundingObjectMaterialUpdate(true);
  } else
    disconnect(this, &OmSimulationWorld::physicsStepEnded, this, &OmSimulationWorld::propagateBoundingObjectUpdate);
}

void OmSimulationWorld::rayTracingEnabled() {
  OmBoundingSphere::enableUpdates(true, root()->boundingSphere());
}

void OmSimulationWorld::updateRandomSeed() {
  assert(worldInfo());
  int seed = worldInfo()->randomSeed();
  if (seed < 0)
    seed = QDateTime::currentMSecsSinceEpoch() + QCoreApplication::applicationPid();
  OmRandom::setSeed(seed);  // Webots random seed
}

void OmSimulationWorld::storeLastSaveTime() {
  mSimulationHasRunAfterSave = false;
}

bool OmSimulationWorld::simulationHasRunAfterSave() {
  return mSimulationHasRunAfterSave;
}

void OmSimulationWorld::reset(bool restartControllers) {
  OmWorld::reset(restartControllers);
  OmSimulationState::instance()->pauseSimulation();
  OmSimulationState::instance()->resetTime();
  OmTemplateManager::instance()->blockRegeneration(true);
  foreach (OmNode *node, mAddedNode) {
    if (mAddedNode.contains(node))  // the node may already have been remove if it is a children of another previously removed
      OmNodeOperations::instance()->deleteNode(node, true);
  }
  mAddedNode.clear();
  {
    // Whether the cascade may overwrite what each controller commanded its
    // motors is exactly the question of whether those controllers are about to
    // be restarted. They are for the GUI Reset button (restartControllers ==
    // true, below); they are NOT for wb_supervisor_simulation_reset(), which
    // reaches here via the mResetRequested branch in step() with
    // restartControllers == false. See OmMotor::resetMayOverwriteMotorCommand.
    const OmMotor::ResetPolicy motorPolicy(restartControllers);
    root()->reset("__init__");
  }
  OmTemplateManager::instance()->blockRegeneration(false);

  // P7 reset hook: give the physics backend a chance to flush world-scope
  // state that the per-Solid reset cascade missed. Newton re-runs eval_fk
  // through resetJointsToDefaults() so any fixed-base solids' joint_q gets
  // back to the model's initial values even when their chassis doesn't fire
  // positionChangedArtificially.
  OmPhysicsBackendRegistry::newtonBackend()->reset();
  OmSoundEngine::stopAllSources();
  if (restartControllers) {
    foreach (OmRobot *const robot, robots()) {
      if (robot->isControllerStarted())
        robot->restartController();
    }
  }
  updateRandomSeed();
  if (OmDownloadManager::instance()->progress() == 100)
    OmSimulationState::instance()->resumeSimulation();
  storeLastSaveTime();
  setModified(false);
}

void OmSimulationWorld::storeAddedNodeIfNeeded(OmNode *node) {
  mAddedNode << node;
  connect(node, &QObject::destroyed, this, &OmSimulationWorld::removeNodeFromAddedNodeList);
}

void OmSimulationWorld::removeNodeFromAddedNodeList(const QObject *node) {
  const OmNode *n = static_cast<const OmNode *>(node);
  mAddedNode.removeAll(n);
}

bool OmSimulationWorld::saveAs(const QString &fileName) {
  mAddedNode.clear();
  return OmWorld::saveAs(fileName);
}
