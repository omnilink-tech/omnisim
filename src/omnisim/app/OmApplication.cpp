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

#include "OmApplication.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmAnimationRecorder.hpp"
#include "OmApplicationInfo.hpp"
#include "OmBoundingSphere.hpp"
#include "OmControlledWorld.hpp"
#include "OmCudaContext.hpp"
#include "OmCudaSmoke.hpp"
#include "OmDownloadManager.hpp"
#include "OmLog.hpp"
#include "OmNodeOperations.hpp"
#include "OmParser.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmTemplateEngine.hpp"
#include "OmSimulationState.hpp"
#include "OmSolid.hpp"
#include "OmStandardPaths.hpp"
#include "OmSysInfo.hpp"
#include "OmTokenizer.hpp"
#include "OmVersion.hpp"
#include "OmWorld.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QElapsedTimer>

#include <cassert>

OmApplication *OmApplication::cInstance = NULL;
static QString gProjectLibsInPath;

// Kept as a narrow declaration here so the application layer doesn't inherit
// the triangle cache's hashing implementation headers.
namespace OmTriangleMeshCache {
  void clear();
}

bool updateParsingProgress(int progress) {
  OmApplication::instance()->setWorldLoadingProgress(progress);
  return !OmApplication::instance()->wasWorldLoadingCanceled();
}

void updateDownloadingProgress(int progress) {
  if (OmDownloadManager::instance()->isCompleted())
    emit OmApplication::instance()->deleteWorldLoadingProgressDialog();
  else {
    OmApplication::instance()->setWorldLoadingStatus(OmApplication::instance()->tr("Downloading assets"));
    OmApplication::instance()->setWorldLoadingProgress(progress);
  }
}

OmApplication::OmApplication() {
  assert(cInstance == NULL);
  cInstance = this;

  mWorld = NULL;
  mWorldLoadingCanceled = false;
  mWorldLoadingProgressDialogCreated = false;
  OmDownloadManager::instance()->setProgressUpdateCallback(&updateDownloadingProgress);

  OmPreferences::createInstance("OmniLink", "OmniSim", OmApplicationInfo::version());

  mStartupPath = QDir::currentPath();

  // compute WEBOTS_HOME without trailing "/"
  QString WEBOTS_HOME(QDir::toNativeSeparators(OmStandardPaths::omniSimHomePath()));
  WEBOTS_HOME.chop(1);

  // OmniSim must execute in its home directory
  QDir::setCurrent(WEBOTS_HOME);

  // needed for OmniSim controllers (the legacy WEBOTS_HOME twin is also set so
  // pre-rename controllers / robot-window libraries keep resolving the install root)
  qputenv("OMNISIM_HOME", WEBOTS_HOME.toUtf8());
  qputenv("WEBOTS_HOME", WEBOTS_HOME.toUtf8());

#ifdef _WIN32
  // On Windows, if OmniSim is started from a DOS console or from the
  // Windows desktop, we need to remove the path to msys\1.0\bin (if any)
  // to prevent the Makefile to use the mkdir.exe and rmdir.exe provided
  // by MSYS which conflict with the corresponding DOS commands (same
  // name but different syntax) and cause the Makefile to fail.
  // If OmniSim is started from MSYS, we shouldn't remove this path.
  QString MSYSTEM(qgetenv("MSYSTEM"));
  QString TERM(qgetenv("TERM"));
  if (MSYSTEM != "MINGW32" && TERM != "cygwin") {  // we are in the DOS or Windows Desktop case, not MSYS
    QString path(qgetenv("Path"));
    QString newPath(path);
    while (1) {
      const int i = newPath.indexOf("\\msys\\1.0\\bin", 0, Qt::CaseInsensitive);
      if (i == -1)
        break;
      int j = 0;
      for (j = i; j > 0; j--) {
        if (newPath[j] == ':')
          break;  // Volume separator
      }
      j--;  // points to volume name (e.g., "C")
      newPath = newPath.mid(0, j) + newPath.mid(i + 14);
    }
    qputenv("Path", QByteArray(newPath.toUtf8()));
  }
#endif
}

OmApplication::~OmApplication() {
  OmPhysicsBackendRegistry::waitForNewtonRuntimePreload();
  delete mWorld;
  // World teardown deliberately leaves immutable tessellation results in the
  // process cache for hot reloads.  Release them when the application itself
  // exits, after every scene user has gone away.
  OmTriangleMeshCache::clear();
  OmPreferences::cleanup();
  OmNodeOperations::cleanup();
  cInstance = NULL;

  // remove temporary folder
  QDir tmpDir(OmStandardPaths::webotsTmpPath());
  tmpDir.removeRecursively();
}

void OmApplication::setup() {
  OmNodeOperations *nodeOperations = OmNodeOperations::instance();

  // The CUDA compute layer (OmCudaContext) is LAZY: it initialises on the first
  // consumer (GranularGroup / the CUDA smoke) and logs "CUDA initialized: ..."
  // then. Until 2026-09-02 it was probed eagerly here, so every world -- the
  // overwhelming majority of which never touch OmCudaContext (Newton/warp has
  // its own CUDA init) -- paid a cudaSetDevice + cudaGetDeviceProperties +
  // cudaStreamCreate on the main thread before the world file was even opened
  // (~250 ms on the reference RTX 3060 laptop). OMNISIM_CUDA_EAGER_INIT=1
  // (value-parsed) restores the eager probe for anyone who wants the device
  // line at the top of every log.
  {
    const QByteArray eager = qgetenv("OMNISIM_CUDA_EAGER_INIT").trimmed().toLower();
    if (!eager.isEmpty() && eager != "0" && eager != "false" && eager != "off")
      OmCudaContext::instance();
  }
  // Run the in-process buffer round-trip if OMNISIM_CUDA_SMOKE=1. No-op
  // otherwise; cost in normal runs is one env-var lookup.
  OmCudaSmoke::runIfRequested();

  // create and connect OmAnimationRecorder
  OmAnimationRecorder *recorder = OmAnimationRecorder::instance();
  connect(recorder, &OmAnimationRecorder::animationStartStatusChanged, this, &OmApplication::animationStartStatusChanged);
  connect(recorder, &OmAnimationRecorder::animationStopStatusChanged, this, &OmApplication::animationStopStatusChanged);
  connect(this, &OmApplication::animationCaptureStarted, recorder, &OmAnimationRecorder::start);
  connect(this, &OmApplication::animationCaptureStopped, recorder, &OmAnimationRecorder::stop);
  connect(nodeOperations, &OmNodeOperations::nodeAdded, recorder, &OmAnimationRecorder::propagateNodeAddition);
  connect(this, &OmApplication::deleteWorldLoadingProgressDialog, this,
          &OmApplication::setWorldLoadingProgressDialogCreatedtoFalse);
}

void OmApplication::setWorldLoadingProgress(const int progress) {
  static int previousProgress = 0;
  if (progress == previousProgress)
    return;
  previousProgress = progress;
  if (!mWorldLoadingProgressDialogCreated) {
    // more than 2 seconds that world is loading
    emit createWorldLoadingProgressDialog();
    mWorldLoadingProgressDialogCreated = true;
  }
  emit worldLoadingHasProgressed(progress);
}

void OmApplication::setWorldLoadingStatus(const QString &status) {
  if (!mWorldLoadingProgressDialogCreated) {
    // more than 2 seconds that world is loading
    emit createWorldLoadingProgressDialog();
    mWorldLoadingProgressDialogCreated = true;
  }
  emit worldLoadingStatusHasChanged(status);
}

void OmApplication::setWorldLoadingCanceled() {
  mWorldLoadingCanceled = true;
  OmDownloadManager::instance()->abort();
  emit worldLoadingWasCanceled();
}

void OmApplication::setWorldLoadingProgressDialogCreatedtoFalse() {
  mWorldLoadingProgressDialogCreated = false;
}

bool OmApplication::wasWorldLoadingCanceled() const {
  return mWorldLoadingCanceled;
}

void OmApplication::cancelWorldLoading(bool loadEmpty, bool deleteWorld) {
  emit deleteWorldLoadingProgressDialog();

  if (deleteWorld) {
    delete mWorld;
    mWorld = NULL;
  }

  OmLog::setConsoleLogsPostponed(false);
  OmLog::showPendingConsoleMessages();

  if (loadEmpty)
    loadWorld(OmProject::newWorldPath(), false);
}

bool OmApplication::isValidWorldFileName(const QString &worldName) {
  QFileInfo worldNameInfo(worldName);
  if (!worldNameInfo.exists() || !worldNameInfo.isFile() || !worldNameInfo.isReadable()) {
    OmLog::diagnostic("WORLD_FILE_NOT_FOUND", tr("Could not open file: '%1'.").arg(worldName), worldName);
    OmLog::error(tr("Could not open file: '%1'.").arg(worldName));
    return false;
  }
  // Dual-read: both the current '.omniworld' and the legacy '.wbt' are accepted, indefinitely.
  if (!OmWorldFileFormat::isWorldSuffix(worldNameInfo.suffix())) {
    OmLog::diagnostic(
      "WORLD_WRONG_EXTENSION",
      tr("Could not open file: '%1'. The world file extension must be '.omniworld' (or the legacy '.wbt').").arg(worldName),
      worldName);
    OmLog::error(
      tr("Could not open file: '%1'. The world file extension must be '.omniworld' (or the legacy '.wbt').").arg(worldName));
    return false;
  }
  return true;
}

void OmApplication::loadWorld(QString worldName, bool reloading, bool isLoadingAfterDownload) {
  // Newton's cold CPython + warp/newton import is several seconds on the
  // reference Windows machine. Start it before external PROTO retrieval so it
  // overlaps downloads, tokenization, and parsing instead of serializing with
  // world construction. Subsequent reloads hit the process cache immediately.
  OmPhysicsBackendRegistry::startNewtonRuntimePreload();
  disconnect(OmProtoManager::instance(), &OmProtoManager::worldLoadCompleted, this, &OmApplication::loadWorld);
  bool isValidProject = true;
  const QString newProjectPath = OmProject::projectPathFromWorldFile(worldName, isValidProject);
  OmProject::setCurrent(new OmProject(newProjectPath));

  // decisive load signal should come from OmProtoManager (to ensure all assets are available)
  if (!isLoadingAfterDownload) {
    connect(OmProtoManager::instance(), &OmProtoManager::worldLoadCompleted, this, &OmApplication::loadWorld);
    OmProtoManager::instance()->retrieveExternProto(worldName, reloading);
    return;
  }

  mWorldLoadingCanceled = false;
  mWorldLoadingProgressDialogCreated = false;

  OmNodeOperations::instance()->enableSolidNameClashCheckOnNodeRegeneration(false);

  worldName = QDir::cleanPath(worldName);
  const bool profileReload = !qEnvironmentVariableIsEmpty("OMNISIM_RELOAD_PROFILE");
  QElapsedTimer reloadTimer;
  reloadTimer.start();

  setWorldLoadingStatus(tr("Reading world file "));
  if (wasWorldLoadingCanceled()) {
    cancelWorldLoading(true);
    return;
  }

  OmTokenizer tokenizer;
  const int errors = tokenizer.tokenize(worldName);
  const qint64 tokenizeMs = reloadTimer.restart();
  if (errors > 0) {
    OmLog::diagnostic("WORLD_PARSE_INVALID_TOKENS", tr("'%1': Failed to load due to invalid token(s).").arg(worldName),
                      worldName);
    OmLog::error(tr("'%1': Failed to load due to invalid token(s).").arg(worldName));
    cancelWorldLoading(false);
    return;
  }

  setWorldLoadingStatus(tr("Parsing world"));
  if (wasWorldLoadingCanceled()) {
    cancelWorldLoading(true);
    return;
  }

  OmParser parser(&tokenizer);
  if (!parser.parseWorld(worldName, &updateParsingProgress)) {
    OmLog::diagnostic("WORLD_PARSE_SYNTAX_ERROR", tr("'%1': Failed to load due to syntax error(s).").arg(worldName),
                      worldName);
    OmLog::error(tr("'%1': Failed to load due to syntax error(s).").arg(worldName));
    cancelWorldLoading(true);
    return;
  }
  const qint64 parseMs = reloadTimer.restart();

  emit preWorldLoaded(reloading);
  // create a file in tmp path for ipc extern controllers
  QFile loading_file(OmStandardPaths::webotsTmpPath() + "loading");
  if (!loading_file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    OmLog::warning(tr("Could not create loading signal file: '%1'.").arg(loading_file.fileName()));

  bool isFirstLoad = (mWorld == NULL);
  delete mWorld;
  const qint64 teardownMs = reloadTimer.restart();

  if (wasWorldLoadingCanceled()) {
    cancelWorldLoading(true, true);
    return;
  }

  OmBoundingSphere::enableUpdates(false);

  mWorld = new OmControlledWorld(&tokenizer);
  const qint64 constructMs = reloadTimer.restart();
  if (mWorld->wasWorldLoadingCanceled()) {
    cancelWorldLoading(true, true);
    return;
  }

  OmSimulationState::instance()->setEnabled(true);

  OmNodeOperations::instance()->updateDictionary(true, mWorld->root());

  emit postWorldLoaded(reloading, isFirstLoad);
  loading_file.remove();

  emit deleteWorldLoadingProgressDialog();

  OmNodeOperations::instance()->enableSolidNameClashCheckOnNodeRegeneration(true);
  OmBoundingSphere::enableUpdates(OmSimulationState::instance()->isRayTracingEnabled(), mWorld->root()->boundingSphere());

  emit worldLoadCompleted();
  if (profileReload)
    OmLog::info(QString("[runtime-cycle] world load tokenize=%1 ms parse=%2 ms teardown=%3 ms construct=%4 ms finish=%5 ms total=%6 ms")
                  .arg(tokenizeMs).arg(parseMs).arg(teardownMs).arg(constructMs).arg(reloadTimer.elapsed())
                  .arg(tokenizeMs + parseMs + teardownMs + constructMs + reloadTimer.elapsed()));
    if (profileReload) {
      OmLog::info(QString("[runtime-cycle] ") + OmProtoLoadProfile::instance().report());
      {
        const OmTemplateEngineProfile &tp = OmTemplateEngineProfile::instance();
        OmLog::info(QString("[runtime-cycle] template engine: %1 calls: translate %2 ms + write %3 ms + engine ctor %4 ms + "
                            "importModule %5 ms + call %6 ms; %7 KB of filled templates; result hash %8")
                      .arg(tp.calls).arg(tp.ns[0] / 1000000).arg(tp.ns[1] / 1000000).arg(tp.ns[2] / 1000000)
                      .arg(tp.ns[3] / 1000000).arg(tp.ns[4] / 1000000).arg(tp.bytes / 1024)
                      .arg(QString::number(static_cast<qulonglong>(tp.resultHash), 16)));
      }
    }
}

void OmApplication::takeScreenshot(const QString &fileName, int quality) {
  emit requestScreenshot(fileName, quality);
}

void OmApplication::simulationQuit(int exitStatus) {
  emit simulationQuitRequested(exitStatus);
}

void OmApplication::worldReload() {
  emit worldReloadRequested();
}

void OmApplication::simulationReset(bool restartControllers) {
  OmWorld::instance()->reset(restartControllers);
  emit simulationResetRequested(restartControllers);
}

void OmApplication::startVideoCapture(const QString &fileName, int type, int width, int height, int quality, int acceleration,
                                      bool showCaption) {
  emit videoCaptureStarted(fileName, type, width, height, quality, acceleration, showCaption);
}

void OmApplication::stopVideoCapture() {
  emit videoCaptureStopped(false);
}

void OmApplication::startAnimationCapture(const QString &fileName) {
  emit animationCaptureStarted(fileName);
}

void OmApplication::stopAnimationCapture() {
  emit animationCaptureStopped();
}

void OmApplication::resetPhysics() {
  foreach (OmSolid *solid, OmWorld::instance()->topSolids())
    solid->resetPhysics();
}
