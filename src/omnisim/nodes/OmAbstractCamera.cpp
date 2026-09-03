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

#include "OmAbstractCamera.hpp"

#include "OmBackground.hpp"
#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmLens.hpp"
#include "OmLight.hpp"
#include "OmLog.hpp"
#include "OmPerformanceLog.hpp"
#include "OmPreferences.hpp"
#include "OmProtoModel.hpp"
#include "OmRenderBackend.hpp"
#include "OmRgb.hpp"
#include "OmRobot.hpp"
#include "OmSFNode.hpp"
#include "OmSimulationState.hpp"
#include "OmStandardPaths.hpp"
#include "OmViewpoint.hpp"
#include "OmWgpuRenderTarget.hpp"  // OmWgpuSolidDraw (complete type for the cached draw list)
#include "OmWgpuSceneRenderer.hpp"  // P1/P9: the shared sensor collect (Solids + dynamic content)
#include "OmWgpuTextureCache.hpp"   // evictStale() at the draw-list rebuild point
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmWrenTextureOverlay.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtCore/QDir>
#include <QtCore/QFile>
#ifndef _WIN32
#include "OmPosixMemoryMappedFile.hpp"

#else
#include <QtCore/QSharedMemory>
#endif
#include <QtCore/QUrl>

#include <QtCore/QElapsedTimer>
#include <QtCore/QSet>

struct OmAbstractCamera::WgpuDrawCache {
  std::vector<OmWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> models;  // draws' modelMatrix16 alias into this, 1:1
  std::vector<OmWgpuSceneRenderer::OmWgpuDrawRefresh> refresh;
  std::vector<QMetaObject::Connection> conns;
  const OmWgpuMeshCache *meshCache = nullptr;
  const OmWgpuTextureCache *texCache = nullptr;
  bool dirty = true;
  int age = 0;
  int retries = 0;
  size_t dynamicTail = 0;
  // telemetry (OMNISIM_WGPU_REPORT): cumulative collect time and how many frames rebuilt the list
  qint64 collectNs = 0;
  int frames = 0;
  int rebuilds = 0;
};

namespace {
  bool sensorDrawCacheEnabled() {
    static const bool on = []() {
      const QByteArray v = qgetenv("OMNISIM_WGPU_SENSOR_DRAW_CACHE").trimmed().toLower();
      return v.isEmpty() || (v != "0" && v != "false" && v != "off");
    }();
    return on;
  }
  bool sensorDrawCacheReport() {
    static const bool on = qEnvironmentVariableIsSet("OMNISIM_WGPU_REPORT");
    return on;
  }
}  // namespace
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

#include <cmath>
#include <limits>


int OmAbstractCamera::cCameraNumber = 0;
int OmAbstractCamera::cCameraCounter = 0;

void OmAbstractCamera::init() {
  mWgpuDrawCache = NULL;
  mImageMemoryMappedFile = NULL;
  mImageData = NULL;
  mSensor = NULL;
  mRefreshRate = 0;
  mNeedToConfigure = false;
  mSendMemoryMappedFile = false;
  mHasExternControllerChanged = false;
  mIsRemoteExternController = false;
  mNeedToCheckShaderErrors = false;
  mMemoryMappedFileReset = false;
  mExternalWindowEnabled = false;
  mCharType = 0;
  mWrenlessSetup = false;
  mWrenlessNoImageWarned = false;

  // Fields initialization
  mFieldOfView = findSFDouble("fieldOfView");
  mProjection = findSFString("projection");
  mNear = findSFDouble("near");
  mMotionBlur = findSFDouble("motionBlur");
  mNoise = findSFDouble("noise");
  mLens = findSFNode("lens");
  mImageChanged = false;
}

OmAbstractCamera::OmAbstractCamera(const QString &modelName, OmTokenizer *tokenizer) : OmRenderingDevice(modelName, tokenizer) {
  init();
}

OmAbstractCamera::OmAbstractCamera(const OmAbstractCamera &other) : OmRenderingDevice(other) {
  init();
}

OmAbstractCamera::OmAbstractCamera(const OmNode &other) : OmRenderingDevice(other) {
  init();
}

OmAbstractCamera::~OmAbstractCamera() {
  invalidateWgpuDrawCache();
  delete mWgpuDrawCache;
  mWgpuDrawCache = NULL;
  delete mImageMemoryMappedFile;
  delete mSensor;

  // D1.4: deleteWren() is the cCameraCounter release (the WREN objects are gone).
  if (areWrenObjectsInitialized() || mWrenlessSetup)
    deleteWren();
}

void OmAbstractCamera::preFinalize() {
  OmRenderingDevice::preFinalize();

  if (lens())
    lens()->preFinalize();

  mSensor = new OmSensor();

  updateFieldOfView();
  updateProjection();
  updateNoise();
}

void OmAbstractCamera::postFinalize() {
  OmRenderingDevice::postFinalize();

  if (lens()) {
    lens()->postFinalize();
    updateLens();
  }

  connect(mFieldOfView, &OmSFDouble::changed, this, &OmAbstractCamera::updateFieldOfView);
  connect(mProjection, &OmSFString::changed, this, &OmAbstractCamera::updateProjection);
  connect(mNoise, &OmSFDouble::changed, this, &OmAbstractCamera::updateNoise);
  if (mLens)
    connect(mLens, &OmSFNode::changed, this, &OmAbstractCamera::updateLens);
  if (mMotionBlur)
    connect(mMotionBlur, &OmSFDouble::changed, this, &OmAbstractCamera::updateMotionBlur);
}

OmLens *OmAbstractCamera::lens() const {
  if (mLens)
    return dynamic_cast<OmLens *>(mLens->value());
  else
    return NULL;
}

void OmAbstractCamera::deleteWren() {
  cCameraCounter--;
  if (cCameraCounter == 0)
    resetStaticCounters();
}

void OmAbstractCamera::initializeImageMemoryMappedFile() {
  mSendMemoryMappedFile = true;
  delete mImageMemoryMappedFile;
  mImageMemoryMappedFile = initializeMemoryMappedFile();
  if (mImageMemoryMappedFile)
    mImageData = static_cast<unsigned char *>(mImageMemoryMappedFile->data());
}

OmMemoryMappedFile *OmAbstractCamera::initializeMemoryMappedFile(const QString &id) {
  // On Linux, we need to use memory mapped files named snap.webots.* to be compliant with the strict confinement policy of snap
  // applications.
  const QString folder = OmStandardPaths::webotsTmpPath() + "ipc/" + QUrl::toPercentEncoding(robot()->name());
  const QString memoryMappedFileName = folder + "/snap.webots." + QUrl::toPercentEncoding(name()) + id;
  QDir().mkdir(folder);
  OmMemoryMappedFile *imageMemoryMappedFile = new OmMemoryMappedFile(memoryMappedFileName);
  // A controller of the previous simulation may have not released cleanly the memory mapped file (e.g. when the controller
  // crashes). This can be detected by trying to attach, and the memory mapped file may be cleaned by detaching.
  if (imageMemoryMappedFile->attach())
    imageMemoryMappedFile->detach();
  if (!imageMemoryMappedFile->create(size())) {
    const QString message =
      tr("Cannot allocate memory mapped file for camera image.\nCaused by: %1.").arg(imageMemoryMappedFile->errorString());
    warn(message);
    delete imageMemoryMappedFile;
    return NULL;
  }
  return imageMemoryMappedFile;
}

namespace {
  // OMNISIM_WGPU_CAMERA_NO_GL -- exact-revert hatch for the wrenless setup path below.
  // VALUE-parsed, never presence-gated: unset (or a non-numeric value) means ON, "=0" means
  // OFF and restores the unconditional FATAL. Read once through a function-local static
  // because the environment is immutable mid-process.
  bool wgpuCameraNoGlEnabled() {
    static const bool kEnabled = []() {
      if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_CAMERA_NO_GL"))
        return true;
      bool ok = false;
      const int value = qEnvironmentVariableIntValue("OMNISIM_WGPU_CAMERA_NO_GL", &ok);
      return ok ? (value != 0) : true;
    }();
    return kEnabled;
  }
}  // namespace

bool OmAbstractCamera::resolvesToWgpuBackend() const {
  // Lane E1 (P5/P6 prerequisite): every camera-family node now DECLARES `renderBackend` --
  // Lidar.wrl and RangeFinder.wrl gained the field on 2026-08-23, precisely because this
  // NULL branch used to swallow them: a Lidar could neither take the wgpu envelope, nor
  // warn about an unported post-process, nor escape the OMNISIM_NO_GL fatal in setup().
  // The branch is kept as a safety net for any future field-less subclass; it answers
  // "wren", mirroring OmCamera::renderBackendName().
  const OmSFString *const field = findSFString("renderBackend");
  if (field == NULL)
    return false;

  QString backendName = field->value();
  // World-level default (default-flip-plan.md 3.2): an empty or "auto" value defers to
  // WorldInfo.defaultRenderBackend; an explicit per-node value wins. Same order as
  // OmCamera::renderBackend().
  bool fromWorldInfo = false;
  if (backendName.isEmpty() || backendName == QStringLiteral("auto")) {
    const OmWorldInfo *const worldInfo = OmWorld::instance() ? OmWorld::instance()->worldInfo() : NULL;
    if (worldInfo != NULL && !worldInfo->defaultRenderBackend().isEmpty()) {
      backendName = worldInfo->defaultRenderBackend();
      fromWorldInfo = true;
    }
  }
  warnIfRetiredWrenBackend(backendName, fromWorldInfo);

  const QByteArray name = backendName.toUtf8();
  // resolve() already returns the WREN backend when wgpu-native is unavailable (the F1
  // last-resort arm), so testing the RESOLVED kind -- not the authored string -- is what
  // makes this answer "wgpu will really drive this device".
  const OmRenderBackend *const backend = OmRenderBackendRegistry::resolve(OmRenderBackendKindFromString(name.constData()));
  return backend != NULL && backend->kind() == OmRenderBackendKind::Vulkan;
}

// F1 (wren-deletion-runbook.md, Phase F): "wren" keeps PARSING -- an undeclared field or a
// refused value would be an ERROR that takes a headless run's exit code to 1, the
// Solid.immersionProperties precedent -- but it no longer selects a renderer. Warn once per
// node, by name (warn() prefixes the node), so a legacy world says exactly which devices
// changed renderer instead of silently rendering differently.
void OmAbstractCamera::warnIfRetiredWrenBackend(const QString &value, bool fromWorldInfo) const {
  if (mWarnedRetiredWrenBackend || value != QStringLiteral("wren"))
    return;
  mWarnedRetiredWrenBackend = true;
  if (fromWorldInfo)
    warn(tr("WorldInfo.defaultRenderBackend \"wren\" is RETIRED: WREN is no longer selectable, so this device now "
            "renders through wgpu. Remove the WorldInfo value (wgpu is the default)."));
  else
    warn(tr("renderBackend \"wren\" is RETIRED: WREN is no longer selectable, so this device now renders through "
            "wgpu. Remove the field (wgpu is the default)."));
}

void OmAbstractCamera::warnWrenlessNoImage() {
  if (mWrenlessNoImageWarned)
    return;
  mWrenlessNoImageWarned = true;
  warn(tr("'%1': set up on the wgpu render backend with no OpenGL context, but no image was produced for this step. The "
          "controller is reading a frozen buffer -- this is not a rendering artefact, nothing wrote pixels at all.")
         .arg(deviceName()));
}

// P1/P9: the ONE sensor-side scene collect. See the long note on the declaration for why it
// lives on the shared base rather than being spelled out at each of the seven call sites.
void OmAbstractCamera::collectWgpuDraws(OmWgpuMeshCache &cache, std::vector<OmWgpuSolidDraw> &out,
                                        std::vector<std::array<float, 16>> &modelStorage,
                                        OmWgpuTextureCache *texCache) {
  OmWgpuSceneRenderer::collectWorldDraws(cache, out, modelStorage, /*outNodes=*/NULL, texCache);
  if (!OmWgpuSceneRenderer::sensorDynamicEnabled())
    return;  // exact-revert arm: Solids only, i.e. the pre-P1 sensor image
  // Cloth / SoftBody / GranularGroup. Every sensor's draw vectors are FUNCTION-LOCAL and
  // rebuilt per call, so unlike the main view there is no cached prefix and nothing to trim
  // -- the returned count is deliberately dropped.
  OmWgpuSceneRenderer::collectDynamicDraws(cache, out, modelStorage, texCache);
}

void OmAbstractCamera::invalidateWgpuDrawCache() {
  if (!mWgpuDrawCache)
    return;
  WgpuDrawCache &c = *mWgpuDrawCache;
  for (const QMetaObject::Connection &conn : c.conns)
    QObject::disconnect(conn);
  c.conns.clear();
  c.draws.clear();
  c.models.clear();
  c.refresh.clear();
  c.dynamicTail = 0;
  c.dirty = true;
}

void OmAbstractCamera::collectWgpuDrawsCached(OmWgpuMeshCache &cache, OmWgpuTextureCache *texCache,
                                              std::vector<OmWgpuSolidDraw> *&outDraws,
                                              std::vector<std::array<float, 16>> *&outModels,
                                              bool *texturesEvicted) {
  if (texturesEvicted)
    *texturesEvicted = false;
  if (!mWgpuDrawCache)
    mWgpuDrawCache = new WgpuDrawCache();
  WgpuDrawCache &c = *mWgpuDrawCache;
  outDraws = &c.draws;
  outModels = &c.models;
  QElapsedTimer timer;
  timer.start();
  if (!sensorDrawCacheEnabled()) {
    // exact-revert arm: the pre-cache behaviour, a fresh walk into fresh vectors every frame
    c.draws.clear();
    c.models.clear();
    c.draws.reserve(64);
    c.models.reserve(64);
    if (texCache && texCache->evictStale() && texturesEvicted)
      *texturesEvicted = true;
    collectWgpuDraws(cache, c.draws, c.models, texCache);
    c.collectNs += timer.nsecsElapsed();
    ++c.frames;
    ++c.rebuilds;
    if (sensorDrawCacheReport() && (c.frames % 100) == 0)
      OmLog::info(QString("[OmAbstractCamera] '%1' draw collect (cache OFF): %2 frames, %3 ms/frame mean, %4 draws")
                    .arg(deviceName())
                    .arg(c.frames)
                    .arg(c.collectNs / 1.0e6 / c.frames, 0, 'f', 3)
                    .arg(static_cast<qulonglong>(c.draws.size())));
    return;
  }
  // The records alias GPU resources owned by these two caches; a different cache is a different list.
  if (c.meshCache != &cache || c.texCache != texCache) {
    invalidateWgpuDrawCache();
    c.meshCache = &cache;
    c.texCache = texCache;
  }
  // Drop last frame's dynamic tail before the size check below, exactly as the main view does.
  if (c.dynamicTail > 0) {
    c.draws.resize(c.draws.size() - std::min(c.dynamicTail, c.draws.size()));
    c.models.resize(c.models.size() - std::min(c.dynamicTail, c.models.size()));
    c.dynamicTail = 0;
  }
  ++c.age;
  if (c.dirty || c.refresh.empty() || c.age >= 600 ||
      !OmWgpuSceneRenderer::refreshWorldDraws(c.models, c.refresh, &c.draws)) {
    invalidateWgpuDrawCache();
    c.dirty = false;
    c.age = 0;
    // Rebuild point for the texture cache (see OmWgpuTextureCache::evictStale).
    if (texCache && texCache->evictStale() && texturesEvicted)
      *texturesEvicted = true;
    ++c.rebuilds;
    int skipped = 0;
    c.draws.reserve(64);
    c.models.reserve(64);
    // Same call as collectWgpuDraws makes (no per-viewer hidden set is passed there either), plus
    // the refresh records and the incomplete-collect counter the cache needs.
    OmWgpuSceneRenderer::collectWorldDraws(cache, c.draws, c.models, /*outNodes=*/nullptr, texCache, &c.refresh,
                                           &skipped);
    // A destroyed scene node would dangle the cached records: hook every referenced node.
    QSet<QObject *> hooked;
    for (const OmWgpuSceneRenderer::OmWgpuDrawRefresh &r : c.refresh)
      if (r.node && !hooked.contains(r.node)) {
        hooked.insert(r.node);
        c.conns.push_back(connect(r.node, &QObject::destroyed, this, [this]() { invalidateWgpuDrawCache(); }));
      }
    if (OmWorld::instance()) {
      if (OmGroup *root = OmWorld::instance()->root())
        c.conns.push_back(connect(root, &OmGroup::childrenChanged, this, [this]() { invalidateWgpuDrawCache(); }));
      c.conns.push_back(connect(OmWorld::instance(), &OmWorld::robotAdded, this, [this]() { invalidateWgpuDrawCache(); }));
      c.conns.push_back(
        connect(OmWorld::instance(), &OmWorld::robotRemoved, this, [this]() { invalidateWgpuDrawCache(); }));
    }
    // Self-healing collect (the main view's rule): shapes whose geometry is not finalized yet are
    // dropped silently; keep re-collecting, bounded, until the scene settles.
    if (skipped > 0 && c.retries < 300) {
      ++c.retries;
      c.dirty = true;
    } else if (skipped == 0)
      c.retries = 0;
  }
  // Dynamic content is appended after the cached prefix every frame (collectDynamicDraws re-points
  // the WHOLE list by index at its end, so the prefix survives any reallocation it causes).
  if (OmWgpuSceneRenderer::sensorDynamicEnabled())
    c.dynamicTail = OmWgpuSceneRenderer::collectDynamicDraws(cache, c.draws, c.models, texCache);
  c.collectNs += timer.nsecsElapsed();
  ++c.frames;
  if (sensorDrawCacheReport() && (c.frames % 100) == 0)
    OmLog::info(QString("[OmAbstractCamera] '%1' draw collect: %2 frames, %3 rebuilds, %4 ms/frame mean, %5 draws")
                  .arg(deviceName())
                  .arg(c.frames)
                  .arg(c.rebuilds)
                  .arg(c.collectNs / 1.0e6 / c.frames, 0, 'f', 3)
                  .arg(static_cast<qulonglong>(c.draws.size())));
}

void OmAbstractCamera::setup() {
  // D1.4 (WREN deletion): EVERY camera-family device sets up WRENLESS -- the wgpu arm
  // produces the image (E1/E5's shipped path); there is no OmWrenCamera, no WREN frustum
  // and no per-device GL requirement any more. resolvesToWgpuBackend() still runs for its
  // side effects (the retired-"wren" warning) and to detect a host with NO renderer.
  const bool wgpu = resolvesToWgpuBackend();
  if (!wgpu)
    warn(tr("'%1': the wgpu render backend is unavailable on this host and the legacy WREN renderer was deleted -- "
            "this device will produce NO image data (the controller reads a frozen buffer).")
           .arg(deviceName()));

  // Authored image features with no wgpu implementation: warn by name (E1's retirement
  // contract -- fields keep parsing, the device renders, the effect is absent).
  const QString degraded = wgpuUnsupportedFeature(/*includeSilentPostFx=*/true);
  if (!degraded.isEmpty())
    warn(tr("'%1': the wgpu renderer does not implement %2, so it is ABSENT from this device's "
            "image. The rest of the image is unaffected. The WREN renderer that implemented it is "
            "RETIRED (there is no per-%3 opt-out); if this effect matters to you, open an issue.")
           .arg(deviceName())
           .arg(degraded)
           .arg(nodeModelName()));

  mWrenlessSetup = true;
  // Subclass hook (Lane E1, P6): OmLidar latches its mActual* copies of the authored fields
  // here (createWrenCamera() used to do it and is gone).
  setupWrenless();

  cCameraNumber++;
  cCameraCounter++;

  initializeImageMemoryMappedFile();
  if (!mImageMemoryMappedFile)
    return;

  OmRenderingDevice::setup();

  // The device-inset overlay (P7's HUD draws it from this state + the CPU image buffer).
  createWrenOverlay();
}

bool OmAbstractCamera::needToRender() const {
  return mSensor->isEnabled() && mSensor->needToRefresh();
}

void OmAbstractCamera::updateCameraTexture() {
  if (isPowerOn() && needToRender()) {
    // update camera overlay before main rendering
    computeValue();
    if (OmAbstractCamera::needToRender()) {
      mSensor->updateTimer();
      mImageChanged = true;
    }
  } else if (mOverlay && mSensor->isEnabled() && mSensor->isRemoteModeEnabled())
    // keep updating the overlay in remote mode
    mOverlay->requestUpdateTexture();
}

// Generally, computeValue() acquires the data from the RTT
// handle and copies the resulting value in the memory mapped file
void OmAbstractCamera::computeValue() {
  if (!hasBeenSetup())
    return;

  OmPerformanceLog *log = OmPerformanceLog::instance();
  if (log)
    log->startMeasure(OmPerformanceLog::DEVICE_RENDERING, deviceName());

  // enable viewpoint's invisible nodes
  // they will be re-disabled after the controller step
  OmViewpoint *viewpoint = OmWorld::instance()->viewpoint();
  viewpoint->enableNodeVisibility(true);

  // D1.4: render() is the subclass's wgpu render; the per-viewer invisible-node set is
  // applied CPU-side (collectWorldDraws' hiddenNodes), not through a scene-graph toggle.
  render();

  if (log)
    log->stopMeasure(OmPerformanceLog::DEVICE_RENDERING, deviceName());
}

void OmAbstractCamera::copyImageToMemoryMappedFile(unsigned char *data) {
  // D1.4: reaching the base copy means the subclass's wgpu render did not produce pixels for
  // this step and the controller is about to read a frozen buffer -- the silent-wrong-data
  // failure this warning exists to name.
  (void)data;
  warnWrenlessNoImage();
}

void OmAbstractCamera::editChunkMetadata(OmDataStream &stream, int newImageSize) {
  int chunkSize = stream.length() - stream.mSizePtr;
  int chunkDataSize = chunkSize - sizeof(int) - sizeof(unsigned char);

  if (chunkDataSize) {  // data chunk between images
    // increase first char by 2 (data + new image)
    stream.increaseNbChunks(2);

    // edit size and type information for the data chunk
    OmDataStream newDataMeta;
    unsigned char newDataType = TCP_DATA_TYPE;
    newDataMeta << chunkDataSize << newDataType;
    stream.replace(stream.mSizePtr, sizeof(int) + sizeof(unsigned char), newDataMeta);
    stream.mDataSize += chunkDataSize;

    // add size and type information for the new image chunk
    unsigned char newImageType = TCP_IMAGE_TYPE;
    stream << newImageSize << newImageType;

  } else {  // two consecutive images
    // increase first char by 1 (new image)
    stream.increaseNbChunks(1);

    // add size and type information for the new image chunk
    OmDataStream newImageMeta;
    unsigned char newImageType = TCP_IMAGE_TYPE;
    newImageMeta << newImageSize << newImageType;
    stream.replace(stream.mSizePtr, sizeof(int) + sizeof(unsigned char), newImageMeta);
  }
}

void OmAbstractCamera::reset(const QString &id) {
  OmRenderingDevice::reset(id);

  if (mLens) {
    OmNode *const l = mLens->value();
    if (l)
      l->reset(id);
  }

  for (int i = 0; i < mInvisibleNodes.size(); ++i)
    disconnect(mInvisibleNodes.at(i), &QObject::destroyed, this, &OmAbstractCamera::removeInvisibleNodeFromList);
  mInvisibleNodes.clear();
}

void OmAbstractCamera::resetMemoryMappedFile() {
  if (hasBeenSetup()) {
    // the previous memory mapped file will be released by the new controller start
    cCameraNumber++;
    initializeImageMemoryMappedFile();
    mMemoryMappedFileReset = true;
  }
}

void OmAbstractCamera::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigureToStream(stream);
}

void OmAbstractCamera::writeAnswer(OmDataStream &stream) {
  if (mImageChanged) {
    if (mIsRemoteExternController) {
      editChunkMetadata(stream, size());

      // copy image to stream
      stream << (short unsigned int)tag();
      stream << (unsigned char)C_ABSTRACT_CAMERA_SERIAL_IMAGE;
      int streamLength = stream.length();
      stream.resize(size() + streamLength);
      // D1.4: the remote-extern-controller path streamed straight out of the WREN camera and
      // has no wgpu equivalent yet, so the resized stream region is left untouched. Say so
      // rather than ship the caller whatever was in that buffer.
      warnWrenlessNoImage();

      // prepare next chunk
      stream.mSizePtr = stream.length();
      stream << (int)0;            // next chunk size
      stream << (unsigned char)0;  // next chunk type
    } else
      copyImageToMemoryMappedFile(image());
    mSensor->resetPendingValue();
    mImageChanged = false;
  }

  if (mNeedToConfigure)
    addConfigureToStream(stream, true);

  if (mSendMemoryMappedFile && !mIsRemoteExternController) {
    stream << (short unsigned int)tag();
    stream << (unsigned char)C_CAMERA_MEMORY_MAPPED_FILE;
    if (mImageMemoryMappedFile) {
      stream << (int)(mImageMemoryMappedFile->size());
      QByteArray n = QFile::encodeName(mImageMemoryMappedFile->nativeKey());
      stream.writeRawData(n.constData(), n.size() + 1);
    } else
      stream << (int)(0);
    mSendMemoryMappedFile = false;
  }
}

void OmAbstractCamera::addConfigureToStream(OmDataStream &stream, bool reconfigure) {
  stream << (short unsigned int)tag();
  if (reconfigure)
    stream << (unsigned char)C_CAMERA_RECONFIGURE;
  else {
    stream << (unsigned char)C_CONFIGURE;
    stream << (unsigned int)uniqueId();
    stream << (unsigned short)width();
    stream << (unsigned short)height();
  }
  stream << (double)fieldOfView();
  stream << (double)minRange();
  stream << (unsigned char)isPlanarProjection();

  mNeedToConfigure = false;
  mMemoryMappedFileReset = false;
}

bool OmAbstractCamera::handleCommand(QDataStream &stream, unsigned char command) {
  bool commandHandled = true;
  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> mRefreshRate;
      mSensor->setRefreshRate(mRefreshRate);

      emit enabled(this, isEnabled());

      if (!hasBeenSetup()) {
        setup();
        mSendMemoryMappedFile = true;
      } else if (mHasExternControllerChanged) {
        mSendMemoryMappedFile = true;
        mHasExternControllerChanged = false;
      }
      break;
    default:
      commandHandled = false;
  }
  return commandHandled;
}

void OmAbstractCamera::setNodesVisibility(QList<const OmBaseNode *> nodes, bool visible) {
  QListIterator<const OmBaseNode *> it(nodes);
  while (it.hasNext()) {
    const OmBaseNode *node = it.next();
    if (visible)
      mInvisibleNodes.removeAll(node);
    else if (!mInvisibleNodes.contains(node)) {
      mInvisibleNodes.append(node);
      connect(node, &QObject::destroyed, this, &OmAbstractCamera::removeInvisibleNodeFromList, Qt::UniqueConnection);
    }
  }
}

void OmAbstractCamera::removeInvisibleNodeFromList(QObject *node) {
  const OmBaseNode *const baseNode = static_cast<OmBaseNode *>(node);
  mInvisibleNodes.removeAll(baseNode);
}

void OmAbstractCamera::createWrenObjects() {
  OmRenderingDevice::createWrenObjects();
  // D1.4: the WREN frustum renderables died with WREN; the wgpu frustum overlay
  // (VF_CAMERA_FRUSTUMS / VF_RANGE_FINDER_FRUSTUMS in OmWgpuView) draws from engine state.
}

// WREN-retirement: the authored image features a wgpu-backed Camera cannot reproduce.
//
// TWO CALLERS, TWO SEVERITIES, and the difference is deliberate.
//  * includeSilentPostFx=false is the WRENLESS ENVELOPE (OMNISIM_NO_GL). Reaching it means there
//    is no OmWrenCamera to fall back to, so any entry here is fatal. This set is UNCHANGED from
//    before the 2026-08-22 Camera default flip -- widening it would turn compute-only runs that
//    work today into hard failures.
//  * includeSilentPostFx=true adds the post-processes that degrade QUIETLY in a normal session:
//    with a GL context the device still builds a WREN camera, but the IMAGE now comes from the
//    wgpu arm, so a WREN post-process is simply absent from the bytes the controller reads. The
//    flip made that reachable by default, and an authored effect that vanishes without a word is
//    exactly the failure mode this codebase keeps having to apologise for. Warn, do not fail:
//    the image is still a correct image, just without the effect.
QString OmAbstractCamera::wgpuUnsupportedFeature(bool includeSilentPostFx) const {
  const OmSFNode *const recognitionField = findSFNode("recognition");
  const OmSFNode *const focusField = findSFNode("focus");
  const OmSFNode *const lensFlareField = findSFNode("lensFlare");
  const OmSFNode *const lensField = findSFNode("lens");
  const OmSFString *const noiseMaskField = findSFString("noiseMaskUrl");
  if (!isPlanarProjection())
    return tr("projection \"%1\" (the wgpu sensor projection is planar-only)").arg(mProjection->value());
  if (recognitionField != NULL && recognitionField->value() != NULL && findSFString("renderBackend") == NULL)
    return tr("a 'recognition' node (recognition and segmentation render through WREN)");
  // Lane E1 (P5) RETIREMENTS: the four entries below are deliberately NOT ported to wgpu.
  // The fields keep parsing (an undeclared field is an ERROR -> headless exit 1, the
  // Solid.immersionProperties precedent), the device still renders, and the effect is
  // reported by name. WREN behaviour is unchanged.
  if (focusField != NULL && focusField->value() != NULL)
    return tr("a 'focus' node (the depth-of-field BLUR is retired on wgpu and will not be rendered; the controller-facing "
              "focus API -- wb_camera_set_focal_distance and friends -- is independent of the blur and keeps working)");
  if (lensFlareField != NULL && lensFlareField->value() != NULL)
    return tr("a 'lensFlare' node (the lens-flare effect is retired on wgpu)");
  if (lensField != NULL && lensField->value() != NULL)
    return tr("a 'lens' node (a camera-CALIBRATION model, not an effect: lens distortion is retired on wgpu, so this device "
              "produces an UNDISTORTED image -- pinhole geometry, as if no Lens node were declared)");
  if (noiseMaskField != NULL && !noiseMaskField->value().isEmpty())
    return tr("a 'noiseMaskUrl' (the noise-mask overlay is retired on wgpu; use the gaussian 'noise' field instead, which IS "
              "applied on wgpu)");
  if (includeSilentPostFx && !wgpuSensorPostFxEnabled()) {
    // Lane E1 (P5): `noise` and `motionBlur` are PORTED (CPU post-FX on the wgpu readback,
    // see applyWgpuRangePostFx / applyWgpuColorPostFx), so by default they are no longer
    // degraded and must not be warned about. The warning comes back only under the exact
    // -revert hatch OMNISIM_WGPU_SENSOR_POSTFX=0, where the port stands down and the
    // pre-port behaviour -- authored effect silently absent -- would otherwise return
    // silently too. mNoise / mMotionBlur are the class's own cached field pointers (set in
    // init), so this cannot drift from what the device actually reads.
    if (mNoise != NULL && mNoise->value() > 0.0)
      return tr("noise %1 (its CPU port is disabled by OMNISIM_WGPU_SENSOR_POSTFX=0)").arg(mNoise->value());
    if (mMotionBlur != NULL && mMotionBlur->value() > 0.0)
      return tr("motionBlur %1 (its CPU port is disabled by OMNISIM_WGPU_SENSOR_POSTFX=0)").arg(mMotionBlur->value());
  }
  return QString();
}

// Lane E1 (P5+P6): CPU ports of the WREN post-FX shader chain for the wgpu sensor arms.
//
// OMNISIM_WGPU_SENSOR_POSTFX -- exact-revert hatch. VALUE-parsed, never presence-gated:
// unset (or non-numeric) means ON, "=0" means OFF and restores the pre-port wgpu output
// (effects dropped, with the silent-post-FX warning in wgpuUnsupportedFeature restored).
bool OmAbstractCamera::wgpuSensorPostFxEnabled() {
  static const bool kEnabled = []() {
    if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_SENSOR_POSTFX"))
      return true;
    bool ok = false;
    const int value = qEnvironmentVariableIntValue("OMNISIM_WGPU_SENSOR_POSTFX", &ok);
    return ok ? (value != 0) : true;
  }();
  return kEnabled;
}

namespace {
  // CPU mirror of the canonical one-liner GLSL hash both noise shaders use
  // (range_noise.frag:19-27 / color_noise.frag:18-26):
  //   rand(seed) = fract(sin(mod(dot(seed, (12.9898, 78.233)), 3.14)) * 43758.5453)
  // Computed in double then truncated, so the STRUCTURE (stateless, per-pixel decorrelated,
  // per-frame reseeded) matches the shader exactly while the bits do not -- bit-exactness is
  // unattainable anyway (float32 sin() differs across GPU drivers) and is not the contract.
  inline float wgpuPostFxRand(float x, float y) {
    const double dt = static_cast<double>(x) * 12.9898 + static_cast<double>(y) * 78.233;
    const double sn = std::fmod(dt, 3.14);
    const double v = std::sin(sn) * 43758.5453;
    return static_cast<float>(v - std::floor(v));
  }

  // Box-Muller cosine branch, exactly as the shaders' gaussian(uv, seed)
  // (range_noise.frag:30-44): standard normal, mean 0, sigma 1.
  inline float wgpuPostFxGaussian(float ux, float uy, float sx, float sy) {
    float U = wgpuPostFxRand(ux + sx, uy + sx);
    const float V = wgpuPostFxRand(ux + sy, uy + sy);
    if (U <= 0.0000001f)
      U = 0.0000001f;
    return std::sqrt(-2.0f * std::log(U)) * std::cos(2.0f * static_cast<float>(M_PI) * V);
  }

  // Per-frame seed chain from the simulation clock in ms, mirroring color_noise.vert:27-32
  // (three chained vec2 seeds for the three colour channels; the range shaders use the
  // first). Stateless: a pure function of OmSimulationState::time().
  inline void wgpuPostFxSeeds(float timeMs, float outSeeds[3][2]) {
    float prev = timeMs;
    for (int k = 0; k < 3; ++k) {
      const float r = wgpuPostFxRand(prev, prev);
      outSeeds[k][0] = r;
      outSeeds[k][1] = wgpuPostFxRand(r, r);
      prev = outSeeds[k][1];
    }
  }
}  // namespace

void OmAbstractCamera::applyWgpuRangePostFx(float *img, int w, int h, double resolution) const {
  if (img == NULL || w <= 0 || h <= 0 || !wgpuSensorPostFxEnabled())
    return;
  const float noise = mNoise != NULL ? static_cast<float>(mNoise->value()) : 0.0f;
  const float res = static_cast<float>(resolution);
  const bool doNoise = noise > 0.0f;
  const bool doQuant = res > 0.0f;
  if (!doNoise && !doQuant)
    return;
  const float lo = static_cast<float>(minRange());
  const float hi = static_cast<float>(maxRange());
  // DECISION (Lane E1, stated explicitly because no test pinned either behaviour before):
  // with noise > 0 the port reproduces WREN's range CLIP -- range_noise.frag:47-51 rewrites
  // anything outside the OPEN interval (minRange, maxRange), including the background at
  // exactly maxRange, to +inf. WREN is the reference the authored worlds were tuned
  // against, and the controller API documents out-of-range as infinity. The wgpu arms'
  // NO-noise behaviour (clamp into [minRange, maxRange], OmLidar.cpp / OmRangeFinder.cpp)
  // is a pre-existing, independent divergence and is deliberately NOT changed here.
  const float kInf = std::numeric_limits<float>::infinity();
  float seeds[3][2];
  wgpuPostFxSeeds(static_cast<float>(OmSimulationState::instance()->time()), seeds);
  const float du = w > 1 ? 1.0f / static_cast<float>(w - 1) : 0.0f;
  const float dv = h > 1 ? 1.0f / static_cast<float>(h - 1) : 0.0f;
  for (int y = 0; y < h; ++y) {
    const float uvY = static_cast<float>(y) * dv;
    float *const row = img + static_cast<size_t>(y) * static_cast<size_t>(w);
    for (int x = 0; x < w; ++x) {
      float v = row[x];
      if (doNoise) {
        if (v < hi && v > lo)
          // sigma = the authored `noise` value in ABSOLUTE METRES: OmAbstractCamera passes
          // mNoise->value() through untouched (there is NO maxRange scaling anywhere -- the
          // reference docs used to claim otherwise and were corrected 2026-08-22).
          v += noise * wgpuPostFxGaussian(static_cast<float>(x) * du, uvY, seeds[0][0], seeds[0][1]);
        else
          v = kInf;
      }
      if (doQuant)
        // depth_resolution.frag:16-19 -- round-half-up in metres, AFTER noise. floor() and
        // multiplication both map +inf to +inf, so clipped samples stay +inf.
        v = std::floor(v / res + 0.5f) * res;
      row[x] = v;
    }
  }
}

void OmAbstractCamera::applyWgpuColorPostFx(unsigned char *bgra, int w, int h, std::vector<unsigned char> &blurHistory) const {
  if (bgra == NULL || w <= 0 || h <= 0 || !wgpuSensorPostFxEnabled())
    return;
  const size_t bytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 4u;

  // 1. motionBlur -- WREN pass order runs MOTION_BLUR before COLOR_NOISE
  // (OmWrenRenderingContext.hpp PP_* enum, sorted ascending), so the history holds the
  // BLURRED-but-not-noised frame, exactly like the WREN pass's feedback texture.
  // blend = pow(0.005, refreshRate_ms / motionBlur_ms) = the weight of the OLD frame
  // (OmAbstractCamera::applyMotionBlurToWren). motionBlur == 0 would divide by zero -- the
  // > 0.0 gate below is the guard. DEVIATION, documented: WREN blends HDR linear PRE-
  // tonemap; this blends the final LDR bytes (post-tonemap, post-swizzle). Same time
  // constant, slightly different ghost falloff on bright content.
  if (mMotionBlur != NULL && mMotionBlur->value() > 0.0 && mRefreshRate > 0) {
    const double blend = pow(0.005, static_cast<double>(mRefreshRate) / mMotionBlur->value());
    if (blurHistory.size() != bytes) {
      // first render (or resize): motion_blur.frag's firstRender branch -- output = scene.
      blurHistory.assign(bgra, bgra + bytes);
    } else {
      const float fb = static_cast<float>(blend);
      const float fs = 1.0f - fb;
      for (size_t i = 0; i < bytes; ++i) {
        const float mixed = fs * static_cast<float>(bgra[i]) + fb * static_cast<float>(blurHistory[i]);
        const unsigned char out = static_cast<unsigned char>(mixed + 0.5f);
        bgra[i] = out;
        blurHistory[i] = out;  // feedback: history = this pass's own output
      }
    }
  } else if (!blurHistory.empty())
    blurHistory.clear();  // field cleared at runtime: drop stale history

  // 2. noise -- color_noise.frag:45-47: per-channel independent additive Gaussian (three
  // chained seeds, color_noise.vert:27-32), alpha untouched, then clamp -- the clamp is
  // load-bearing (asymmetric near black/white). sigma = `noise` in normalised channel
  // units, i.e. noise * 255 in byte units. Runs on the final LDR bytes, which is where the
  // WREN pass sits too (COLOR_NOISE is after HDR/tonemap in the drawingIndex order).
  if (mNoise != NULL && mNoise->value() > 0.0) {
    const float sigmaBytes = static_cast<float>(mNoise->value()) * 255.0f;
    float seeds[3][2];
    wgpuPostFxSeeds(static_cast<float>(OmSimulationState::instance()->time()), seeds);
    const float du = w > 1 ? 1.0f / static_cast<float>(w - 1) : 0.0f;
    const float dv = h > 1 ? 1.0f / static_cast<float>(h - 1) : 0.0f;
    for (int y = 0; y < h; ++y) {
      const float uvY = static_cast<float>(y) * dv;
      unsigned char *const row = bgra + static_cast<size_t>(y) * static_cast<size_t>(w) * 4u;
      for (int x = 0; x < w; ++x) {
        const float uvX = static_cast<float>(x) * du;
        unsigned char *const px = row + static_cast<size_t>(x) * 4u;
        for (int c = 0; c < 3; ++c) {
          const float noised =
            static_cast<float>(px[c]) + sigmaBytes * wgpuPostFxGaussian(uvX, uvY, seeds[c][0], seeds[c][1]);
          px[c] = noised <= 0.0f ? 0 : (noised >= 255.0f ? 255 : static_cast<unsigned char>(noised + 0.5f));
        }
      }
    }
  }
}

void OmAbstractCamera::createWrenOverlay() {
  // D1.4: the overlay is a state-only model now (OmHudOverlay draws it from the CPU image
  // buffer). Built unconditionally: under a windowless session it is inert state.
  QStringList previousSettings;
  if (mOverlay)
    previousSettings = mOverlay->perspective();

  delete mOverlay;
  mOverlay = NULL;

  if (mCharType == 'r')
    mOverlay = new OmWrenTextureOverlay(image(), width(), height(), OmWrenTextureOverlay::TEXTURE_TYPE_DEPTH,
                                        OmWrenTextureOverlay::OVERLAY_TYPE_RANGE_FINDER);
  else if (mCharType == 'c')
    mOverlay = new OmWrenTextureOverlay(image(), width(), height(), OmWrenTextureOverlay::TEXTURE_TYPE_BGRA,
                                        OmWrenTextureOverlay::OVERLAY_TYPE_CAMERA, 1.0, false);
  if (!mOverlay)
    return;
  if (mCharType == 'r')
    mOverlay->setMaxRange(maxRange());

  if (!previousSettings.isEmpty())
    mOverlay->restorePerspective(previousSettings, areOverlaysEnabled());
  else
    mOverlay->setVisible(true, areOverlaysEnabled());
}

/////////////////////
//  Update methods //
/////////////////////

void OmAbstractCamera::updateWidth() {
  OmRenderingDevice::updateWidth();

  if (areWrenObjectsInitialized() && !hasBeenSetup())
    createWrenOverlay();
}

void OmAbstractCamera::updateHeight() {
  OmRenderingDevice::updateHeight();

  if (areWrenObjectsInitialized() && !hasBeenSetup())
    createWrenOverlay();
}

void OmAbstractCamera::updateLens() {
  // D1.4: lens distortion is a retired wgpu feature (E1) -- the field state survives, the
  // WREN application is gone.
}

void OmAbstractCamera::updateFieldOfView() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mFieldOfView, 0.7854))
    return;
  if (isPlanarProjection() && fieldOfView() > M_PI) {
    parsingWarn(tr(
      "Invalid 'fieldOfView' changed to 0.7854. The field of view is limited to pi if the 'projection' field is \"planar\"."));
    mFieldOfView->setValue(0.7854);
    return;
  }

  mNeedToConfigure = true;
  // D1.4: the wgpu render reads fieldOfView() live; the WREN frustum visuals are gone.
}

void OmAbstractCamera::updateProjection() {
  const QString &projection = mProjection->value();
  if (projection != "planar" && projection != "spherical" && projection != "cylindrical") {
    parsingWarn(tr("Unknown 'projection' value \"%1\": field value reset to \"planar\".").arg(projection));
    mProjection->setValue("planar");
    return;
  }

  mNeedToConfigure = true;

  if (hasBeenSetup())
    setup();

  updateFieldOfView();
}

void OmAbstractCamera::updateAntiAliasing() {
  if (hasBeenSetup())
    setup();
}

void OmAbstractCamera::updateMotionBlur() {
  if (!mMotionBlur || OmFieldChecker::resetDoubleIfNegative(this, mMotionBlur, 0.0))
    return;
  // D1.4: the wgpu motion-blur port (P5) reads the field live per frame.
}

void OmAbstractCamera::updateNoise() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mNoise, 0.0))
    return;
  // D1.4: the wgpu noise port (P5) reads the field live per frame.
}

void OmAbstractCamera::enableExternalWindowForAttachedCamera(bool enabled) {
  // D1.4: the WREN texture-update notifications died with the WREN camera; the pop-out
  // window repaints from the CPU image buffer on its own cadence.
  (void)enabled;
}

void OmAbstractCamera::enableExternalWindow(bool enabled) {
  OmRenderingDevice::enableExternalWindow(enabled);
  mExternalWindowEnabled = enabled;
}
