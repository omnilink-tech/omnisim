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

#include "OmCamera.hpp"

#include "OmAffinePlane.hpp"
#include "OmBasicJoint.hpp"
#include "OmBoundingSphere.hpp"
#include "OmBox.hpp"
#include "OmCapsule.hpp"
#include "OmCylinder.hpp"
#include "OmPlane.hpp"
#include "OmSphere.hpp"
#include "OmDataStream.hpp"
#include "OmDownloadManager.hpp"
#include "OmDownloader.hpp"
#include "OmFieldChecker.hpp"
#include "OmFocus.hpp"
#include "OmLens.hpp"
#include "OmLensFlare.hpp"
#include "OmLight.hpp"
#include "OmNetwork.hpp"
#include "OmObjectDetection.hpp"
#include "OmPerformanceLog.hpp"
#include "OmPreferences.hpp"
#include "OmProtoModel.hpp"
#include "OmRecognition.hpp"
#include "OmLog.hpp"
#include "OmMatrix4.hpp"
#include "OmPbrAppearance.hpp"
#include "OmRenderBackend.hpp"
#include "OmRgb.hpp"
#include "OmShape.hpp"
#include "OmVulkanBackend.hpp"
#include "OmWgpuMeshAdapter.hpp"
#include "OmWgpuMeshCache.hpp"
#include "OmWgpuRenderTarget.hpp"
#include "OmWgpuSceneRenderer.hpp"
#include "OmWgpuTextureCache.hpp"
// W3: the sensor scene render harvests the SAME world state the wgpu main view does — the
// Background's sky tint (clear colour + hemisphere ambient), the first DirectionalLight as the
// shadow-casting sun, every other light as an unshadowed fill, and the Fog node.
#include "OmBackground.hpp"
#include "OmDirectionalLight.hpp"
#include "OmFog.hpp"
#include "OmPointLight.hpp"
#include "OmSFColor.hpp"  // OmFog::fogColor()->value() needs the complete type
#include "OmSpotLight.hpp"
// R3.7/δ Newton-interop: snapshot live body translations into the wgpu
// instanced storage-buffer path (runtime-gated by OMNISIM_PROBE_DELTA; the
// newton dynamic_cast no-ops on ODE/NEWTON-OFF builds).
#include "OmNewtonBackend.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include "OmGroup.hpp"
#include "OmRgb.hpp"
#include "OmRobot.hpp"
#include "OmSFNode.hpp"
#include "OmSolid.hpp"
#include "OmSolidUtilities.hpp"
#include "OmSensor.hpp"
#include "OmSimulationState.hpp"
#include "OmStandardPaths.hpp"
#include "OmUrl.hpp"
#include "OmWorld.hpp"
#include "OmWrenLabelOverlay.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmZoom.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QtGlobal>

#ifndef _WIN32
#include "OmPosixMemoryMappedFile.hpp"
#else
#include <QtCore/QSharedMemory>
#endif

// Newton raycast service gate (kernel blocker #1, ode-retirement-campaign.md).
// Same value-parsed read as OmDistanceSensor::refreshRaysFromNewton.
static bool isNewtonRaycastEnabled() {
  static int sGate = -1;
  if (sGate < 0) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_RAYCAST")).trimmed().toLower();
    // DEFAULT ON since 2026-08-07: every ray consumer answers from the
    // Newton service (mj_ray on the live mjModel), incl. the LASER
    // transparency re-cast and the joint-descending exclusion walk.
    // =0 turns the service off; with ODE deleted there is no other ray
    // provider, so occlusion cannot be evaluated at all in that case.
    sGate = (v == "0" || v == "false" || v == "off" || v == "no") ? 0 : 1;
  }
  return sGate == 1;
}

// W3 defect 1 gate: correct the R/B channel order of the wgpu SENSOR readback.
//
// The wgpu render target is RGBA8Unorm; the controller image contract is BGRA
// (wb_camera_image_get_red == byte 2, wb_camera_image_get_blue == byte 0, see
// include/controller/c/omnisim/camera.h). Without the swizzle a
// `renderBackend "wgpu"` Camera hands controllers red and blue swapped.
//
// VALUE-parsed, DEFAULT ON (= corrected), same predicate as the raycast gate
// above: OMNISIM_WGPU_CAMERA_BGRA=0 / false / off / no reproduces the pre-fix
// swapped byte order exactly, so the frozen "known good" regression
// expectations still have an arm to run under.
static bool isWgpuCameraBgraEnabled() {
  static int sGate = -1;
  if (sGate < 0) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_WGPU_CAMERA_BGRA")).trimmed().toLower();
    sGate = (v == "0" || v == "false" || v == "off" || v == "no") ? 0 : 1;
  }
  return sGate == 1;
}

// W3 defect 3 gate: render the wgpu SENSOR through the world's REAL materials and lighting.
//
// The measured defect: point the capture service at a real world with
// OMNISIM_CAPTURE_BACKEND=wgpu (it renders through a Camera device) and beauty_bench comes back
// MONOCHROME -- R = G = B = 66.8, std 29.5 -- untextured, unlit, byte-for-byte identical before
// and after 120 settle steps. The cause was not one bug but a whole missing layer: this file
// collected the scene with texCache == nullptr (so every baseColorMap was dropped and every
// material collapsed onto its usually-white baseColor), then rendered it with a hardcoded 0.18
// grey clear, a hardcoded light direction, a hardcoded ambient, no sky and no shadows. The MAIN
// VIEW harvests all of that from the world; the sensor path never got any of it.
//
// ON restores the missing layer: albedo/roughness/metalness/normal maps, the world's Background
// (sky tint as both clear colour and hemisphere ambient, plus the atmospheric dome where the
// world declares one), the first DirectionalLight as a shadow-casting sun with its real colour x
// intensity, every other light as an unshadowed fill, the Fog node, and a cast-shadow pass.
//
// VALUE-parsed, DEFAULT ON, same predicate as the two gates above. =0 / false / off / no
// restores the flat, untextured, hardcoded-light render EXACTLY -- the collect goes back to
// texCache == nullptr and the frame back to clearAndDrawScene -- so the change is A/B-able
// against every frozen expectation on the old output.
//
// It is deliberately ONE gate for the whole layer. Splitting it (textures here, sky there) would
// let a run land in a combination nothing has ever been measured in.
static bool isWgpuCameraSceneEnabled() {
  static int sGate = -1;
  if (sGate < 0) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_WGPU_CAMERA_SCENE")).trimmed().toLower();
    sGate = (v == "0" || v == "false" || v == "off" || v == "no") ? 0 : 1;
  }
  return sGate == 1;
}

// ---- W3: world-state harvest for the sensor scene render ----
//
// These three mirror the file-local helpers of the same shape in OmView3D.cpp
// (findFirstDirectionalLightV3D / findFirstBackgroundV3D / collectExtraLightsV3D). They are
// COPIED rather than shared on purpose: those are `static` in a gui/ translation unit, and a
// nodes/ file must not gain an up-dependency on gui/ to reach them. Keep the two in sync -- the
// semantics (first light wins, the sun is excluded from the extras, off/zero-intensity lights are
// skipped, at most 8 extras) are the shader's contract, not a local choice.

// First OmDirectionalLight anywhere in the world tree (e.g. the OmniSimSun PROTO's light).
static OmDirectionalLight *findFirstDirectionalLightCam(OmBaseNode *root) {
  if (!root)
    return NULL;
  if (OmDirectionalLight *dl = dynamic_cast<OmDirectionalLight *>(root))
    return dl;
  if (OmGroup *g = dynamic_cast<OmGroup *>(root)) {
    const int n = g->childCount();
    for (int i = 0; i < n; ++i)
      if (OmDirectionalLight *d = findFirstDirectionalLightCam(g->child(i)))
        return d;
  }
  return NULL;
}

// First OmBackground in the world tree — its skyColor is the world's sky tint.
static OmBackground *findFirstBackgroundCam(OmBaseNode *root) {
  if (!root)
    return NULL;
  if (OmBackground *b = dynamic_cast<OmBackground *>(root))
    return b;
  if (OmGroup *g = dynamic_cast<OmGroup *>(root)) {
    const int n = g->childCount();
    for (int i = 0; i < n; ++i)
      if (OmBackground *b = findFirstBackgroundCam(g->child(i)))
        return b;
  }
  return NULL;
}

// Every ADDITIONAL scene light beyond the shadowed sun — PointLight, SpotLight, and any
// DirectionalLight other than `sun` — packed 16 floats per light in the WGSL ExtraLight layout
// {posType, colorRad, atten, spotDir}. Rendered UNSHADOWED: only the sun owns the shadow map.
static void collectExtraLightsCam(OmBaseNode *root, const OmDirectionalLight *sun,
                                  std::vector<float> &out) {
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
      collectExtraLightsCam(g->child(i), sun, out);
  }
}

// One-shot, loud-by-design: Recognition.occlusion was asked for but no ray could
// be cast this tick. Silence here is exactly the defect class that made every
// recognized object read as unoccluded after the ODE ray carrier was removed, so
// the camera keeps its PREVIOUS occlusion verdicts rather than pretending every
// line of sight is clear -- and says so once.
static void warnRecognitionOcclusionUnavailableOnce(const char *why) {
  static bool warned = false;
  if (warned)
    return;
  warned = true;
  OmLog::warning(QString::fromUtf8("Camera Recognition 'occlusion' is enabled but the Newton raycast service could not "
                                   "answer (") +
                 QString::fromUtf8(why) +
                 QString::fromUtf8("). The previous occlusion verdicts are kept -- recognized objects are NOT re-tested "
                                   "this tick, and none is silently promoted to \"visible\"."));
}

// Collects the Newton body indices of every Solid under `robot` (same walk as
// OmDistanceSensor builds its mNewtonExcludeBodies).
static void collectRobotNewtonBodies(const OmRobot *robot, QVector<int> &out) {
  // shared walk: descends joint/slot endPoints too (the original per-file
  // OmGroup-only walk missed Solids behind joints, so an articulated robot
  // could see its own arm where ODE's same-robot rule excluded it)
  OmSolidUtilities::collectNewtonBodies(const_cast<OmRobot *>(robot), out);
}

// ---- D1.4 (WREN deletion): local ports of the deleted OmWrenCamera FOV math. ----
// Verified against the pre-deletion source (OmWrenCamera.cpp): computeFieldOfViewY was
// `2 * atan(tan(fovX * 0.5) / aspectRatio)` (aspectRatio = width/height), and
// setupSphericalSubCameras derived the spherical/cylindrical vertical FOV as
// `fovX * height / width`, multiplied by a 1.4 correction coefficient while a SINGLE
// vertical sub-camera covered it (rawFovY <= 2*asin(1/sqrt(3)) = 1.230959417), else 1.0.
// Only the recognition frustum/projection math consumes these now.
static double cameraFieldOfViewY(double fovX, double aspectRatio) {
  return 2.0 * atan(tan(fovX * 0.5) / aspectRatio);
}

static double cameraSphericalFovYCorrectionCoefficient(double fovX, int w, int h) {
  const double rawFovY = fovX * static_cast<double>(h) / static_cast<double>(w);
  return rawFovY <= 1.230959417 ? 1.4 : 1.0;  // 2*asin(1/sqrt(3))
}

static double cameraSphericalFieldOfViewY(double fovX, int w, int h) {
  const double rawFovY = fovX * static_cast<double>(h) / static_cast<double>(w);
  return rawFovY * cameraSphericalFovYCorrectionCoefficient(fovX, w, h);
}

class OmRecognizedObject : public OmObjectDetection {
public:
  OmRecognizedObject(OmCamera *camera, OmSolid *object, const int occlusion, const double maxRange) :
    OmObjectDetection(camera, object, occlusion, maxRange, camera->fieldOfView()) {
    mId = object->uniqueId();
    mModel = "";
    mRelativeOrientation = OmRotation(0.0, 1.0, 0.0, 0.0);
    mPositionOnImage = OmVector2(0, 0);
    mPixelSize = OmVector2(0, 0);
    mColors.clear();
  };

  virtual ~OmRecognizedObject() {}

  int id() const { return mId; }
  const QString &model() const { return mModel; }
  const OmRotation &relativeOrientation() const { return mRelativeOrientation; }
  const OmVector2 positionOnImage() const { return mPositionOnImage; }
  const OmVector2 pixelSize() const { return mPixelSize; }
  const QList<OmRgb> &colors() const { return mColors; }

  void setModel(const QString &model) { mModel = model; }
  void setRelativeOrientation(const OmRotation &relativeOrientation) { mRelativeOrientation = relativeOrientation; }
  void setPositionOnImage(const OmVector2 &positionOnImage) { mPositionOnImage = positionOnImage; }
  void setPixelSize(const OmVector2 &pixelSize) { mPixelSize = pixelSize; }
  void addColor(const OmRgb &colors) { mColors.append(colors); }
  void clearColors() { mColors.clear(); }

protected:
  double distance() override { return fabs(objectRelativePosition().x()); }

  int mId;
  QString mModel;
  OmRotation mRelativeOrientation;
  OmVector2 mPositionOnImage;
  OmVector2 mPixelSize;
  QList<OmRgb> mColors;
};

void OmCamera::init() {
  mCharType = 'c';
  // Fields initialization
  mFocus = findSFNode("focus");
  mZoom = findSFNode("zoom");
  mRecognition = findSFNode("recognition");
  mNoiseMaskUrl = findSFString("noiseMaskUrl");
  mAntiAliasing = findSFBool("antiAliasing");
  mAmbientOcclusionRadius = findSFDouble("ambientOcclusionRadius");
  mBloomThreshold = findSFDouble("bloomThreshold");
  mLensFlare = findSFNode("lensFlare");
  mFar = findSFDouble("far");
  mExposure = findSFDouble("exposure");
  mRenderBackend = findSFString("renderBackend");

  // R3.3b: wgpu render target stays null until first writeAnswer
  // with the wgpu backend active. Width/height tracked so we know
  // when to rebuild after a Camera resize.
  mWgpuTarget = NULL;
  mWgpuTargetWidth = 0;
  mWgpuTargetHeight = 0;
  // R3.4-step-4: per-Camera mesh cache, ditto lazy.
  mWgpuMeshCache = NULL;
  // W3 defect 2: one-shot guard for the `far 0` -> finite-far substitution INFO.
  mWgpuFarSubstitutionLogged = false;
  // F2.3: one-shot guard for the non-planar-projection decline warning.
  mWgpuNonPlanarDeclineWarned = false;
  // W3 defect 3: per-Camera albedo texture cache + its one-shot log, both lazy.
  mWgpuTextureCache = NULL;
  mWgpuScenePathLogged = false;

  // backward compatibility
  OmSFBool *sphericalField = findSFBool("spherical");
  if (sphericalField->value()) {  // Deprecated in Webots R2023
    parsingWarn("Deprecated 'spherical' field, please use the 'projection' field instead.");
    if (isPlanarProjection())
      mProjection->setValue("cylindrical");
    sphericalField->setValue(false);
  }

  mLabelOverlay = NULL;
  mNeedToDeleteRecognizedObjectsRays = false;
  mRecognitionSensor = NULL;
  mRecognitionRefreshRate = 0;
  mRecognizedObjects.clear();
  mIsSubscribedToRayTracing = false;
  mSegmentationChanged = false;
  mSegmentationEnabled = false;
  mSegmentationWrenWarned = false;
  mSegmentationMemoryMappedFile = NULL;
  mSegmentationImageChanged = false;
  mHasSegmentationMemoryMappedFileChanged = false;
  mInvalidRecognizedObjects = QList<OmRecognizedObject *>();
  mDownloader = NULL;
}

OmCamera::OmCamera(OmTokenizer *tokenizer) : OmAbstractCamera("Camera", tokenizer) {
  init();
}

OmCamera::OmCamera(const OmCamera &other) : OmAbstractCamera(other) {
  init();
}

OmCamera::OmCamera(const OmNode &other) : OmAbstractCamera(other) {
  init();
}

OmCamera::~OmCamera() {
  delete mRecognitionSensor;
  qDeleteAll(mRecognizedObjects);
  mRecognizedObjects.clear();

  delete mSegmentationMemoryMappedFile;

  // R3.3b + R3.4-step-4: wgpu render target + mesh cache. Order
  // matters: mesh cache references the same WGPUDevice as the
  // target, so destroy cache (which releases its WGPUBuffers)
  // before the target (whose dtor doesn't release the device
  // itself, but does free the depth+color textures that share the
  // device's allocator).
  delete mWgpuMeshCache;
  mWgpuMeshCache = NULL;
  // W3: same ordering rule as the mesh cache — the texture cache releases WGPUTextures owned by
  // the same device the target's attachments come from, so it must die BEFORE the target.
  delete mWgpuTextureCache;
  mWgpuTextureCache = NULL;
  delete mWgpuTarget;
  mWgpuTarget = NULL;

  if (mIsSubscribedToRayTracing)
    OmSimulationState::instance()->unsubscribeToRayTracing();
}

void OmCamera::downloadAssets() {
  OmAbstractCamera::downloadAssets();

  const QString &noiseMaskUrl = mNoiseMaskUrl->value();
  if (!noiseMaskUrl.isEmpty()) {  // noise mask not mandatory, URL can be empty
    const QString completeUrl = OmUrl::computePath(this, "noiseMaskUrl", noiseMaskUrl);
    if (!OmUrl::isWeb(completeUrl) || OmNetwork::instance()->isCachedWithMapUpdate(completeUrl))
      return;

    delete mDownloader;
    mDownloader = OmDownloadManager::instance()->createDownloader(QUrl(completeUrl), this);
    if (!OmWorld::instance()->isLoading())  // URL changed from the scene tree or supervisor
      connect(mDownloader, &OmDownloader::complete, this, &OmCamera::updateNoiseMaskUrl);
    mDownloader->download();
  }
}

// R2 of engine-migration-plan.md: parser-only renderBackend accessors.
// No call site consults these yet; they exist so R3+ can route this
// Camera's render-to-texture through OmRenderBackendRegistry::resolve.
// Mirrors OmSolid::physicsBackendName / physicsBackend exactly.
const QString &OmCamera::renderBackendName() const {
  // F1: the field-missing fallback answers the DEFAULT renderer, which is wgpu (Camera.wrl
  // flipped its default at F1; "wren" is retired and only survives as an authored legacy value).
  static const QString kWgpu = QStringLiteral("wgpu");
  if (!mRenderBackend)
    return kWgpu;
  return mRenderBackend->value();
}

OmRenderBackend *OmCamera::renderBackend() const {
  QString nameStr = renderBackendName();
  // World-level default (default-flip-plan.md §3.2): an empty or "auto" renderBackend defers to
  // WorldInfo.defaultRenderBackend; an explicit per-node value is used as-is.
  bool fromWorldInfo = false;
  if (nameStr.isEmpty() || nameStr == QStringLiteral("auto")) {
    const OmWorldInfo *const wi = OmWorld::instance() ? OmWorld::instance()->worldInfo() : nullptr;
    if (wi != nullptr && !wi->defaultRenderBackend().isEmpty()) {
      nameStr = wi->defaultRenderBackend();
      fromWorldInfo = true;
    }
  }
  // F1: an authored (or WorldInfo-supplied) "wren" is retired -- warn once, naming this node
  // (latch shared with resolvesToWgpuBackend(), so the node warns exactly once either way).
  warnIfRetiredWrenBackend(nameStr, fromWorldInfo);
  const QByteArray name = nameStr.toUtf8();
  return OmRenderBackendRegistry::resolve(OmRenderBackendKindFromString(name.constData()));
}

// R3.3b: lazy-build / resize the wgpu render target.
//
// Allocates a OmWgpuRenderTarget at this camera's current
// width()/height() the first time the wgpu path is taken. If the
// camera is later resized, the old target is destroyed and a new
// one allocated. Returns true iff a usable target now lives in
// `mWgpuTarget`. False means: not a wgpu-backed camera, OR wgpu
// init failed at startup, OR target construction failed (OOM,
// device lost). Callers must fall through to WREN when this
// returns false.
bool OmCamera::ensureWgpuTarget() {
  // R5c: delegate to the shared sensor scene-renderer. Behaviorally identical
  // to the former inline body (same Vulkan-kind/availability + size + resize
  // checks); keeps Camera + RangeFinder on a single render code path.
  return OmWgpuSceneRenderer::ensureTarget(renderBackend(), width(), height(),
                                           mWgpuTarget, mWgpuMeshCache, mWgpuTargetWidth,
                                           mWgpuTargetHeight);
}

// W3 defect 1: the wgpu readback is RGBA8, the controller image contract is BGRA.
// Swap byte 0 and byte 2 of every texel so wb_camera_image_get_red (byte 2) and
// wb_camera_image_get_blue (byte 0) name the channels they claim to.
//
// SENSOR PATH ONLY. The main view samples the very same RGBA8Unorm target format
// and is correct on screen today, so the fix must not touch the target format or
// any shared shader -- it belongs exactly here, where the sensor copies pixels
// into the device's memory-mapped buffer.
//
// The buffer is mWgpuTargetWidth * mWgpuTargetHeight texels of 4 bytes, which is
// this Camera's width()*height()*4 == size() whenever ensureWgpuTarget() returned
// true (ensureTarget stores the very width()/height() it was handed). Green and
// alpha are untouched, so a grayscale depth readback (r == g == b) is unchanged
// byte for byte.
void OmCamera::swizzleWgpuRgbaToBgra(unsigned char *data) const {
  if (data == NULL || !isWgpuCameraBgraEnabled())
    return;
  if (mWgpuTargetWidth <= 0 || mWgpuTargetHeight <= 0)
    return;
  const size_t texels =
    static_cast<size_t>(mWgpuTargetWidth) * static_cast<size_t>(mWgpuTargetHeight);
  for (size_t i = 0; i < texels; ++i) {
    unsigned char *const p = data + i * 4;
    const unsigned char r = p[0];
    p[0] = p[2];
    p[2] = r;
  }
}

void OmCamera::copyImageToMemoryMappedFile(unsigned char *data) {
  // D1.4 (WREN deletion): the wgpu render IS the main-image path — there is no WREN copy
  // arm and no OmWrenCamera parameter any more. The segmentation image is produced nowhere
  // (see writeAnswer) and never reaches this copy.
  // F2.3 (wren-deletion-runbook): the wgpu sensor arm renders a PLANAR PINHOLE only --
  // buildViewProj derives the projection from tan(fieldOfView/2), so a cylindrical or
  // spherical device (fov up to 2*pi) degenerates (tan(pi) ~ 0) and produces a
  // confidently-wrong image (camera_color_spherical measured sky-only (3,3,3) under it).
  // Non-planar projections are RETIRED-unported on wgpu: DECLINE here, once and by name,
  // and fall through to the base copy, i.e. an honestly-empty image + warnWrenlessNoImage.
  // An empty image is honest; wrong pixels in metres/colour are the failure mode this
  // codebase keeps apologising for.
  if (data && !isPlanarProjection()) {
    if (!mWgpuNonPlanarDeclineWarned) {
      mWgpuNonPlanarDeclineWarned = true;
      warn(tr("'%1': projection \"%2\" is not implemented by the wgpu sensor path (planar "
              "pinhole only) -- the wgpu render is DECLINED for this device and its image "
              "stays empty (the WREN renderer that implemented it is deleted).")
             .arg(deviceName())
             .arg(mProjection->value()));
    }
  } else if (data && ensureWgpuTarget()) {
    // R3.4-step-4 production scene pass.
    //
    // Walk the world's Solids, harvest one draw call per visible
    // Shape (geometry uploaded through OmWgpuMeshAdapter,
    // transform from OmGeometry::matrix(),
    // baseColor from the Shape's PBRAppearance), build a viewProj
    // from this Camera's pose + FOV + near/far + aspect, fire
    // clearAndDrawScene.
    //
    // ⚠ THE LIST OF LIMITATIONS THAT USED TO SIT HERE IS NOW THE `OMNISIM_WGPU_CAMERA_SCENE=0`
    // ARM, NOT THE DEFAULT. Flat Lambertian shading, dropped textures, a hardcoded light and a
    // hardcoded 0.18 grey clear are what the gated-OFF path still does — see the W3 SENSOR SCENE
    // RENDER block further down, which is the default and harvests the world's real materials,
    // Background, lights, Fog and shadows. The remaining gaps are recorded there.
    //
    // Still true on BOTH arms: `clear` below is the flat 0.18 grey (the scene render computes its
    // own clear from the Background), and geometry coverage is whatever
    // OmWgpuSceneRenderer::collectWorldDraws produces.
    // Sensor MODE selection, hoisted above the collect because it decides whether the collect
    // uploads textures at all. Each of these is a diagnostic that owns the whole frame and must
    // keep reaching the pipeline it was written for, so the W3 scene render stands down for all
    // of them:
    //   OMNISIM_CAMERA_DEPTH  — grayscale / metric-depth / radial-range proofs (no colour at all)
    //   OMNISIM_CAMERA_AGX    — the T1.1 filmic A/B on the SIMPLE lit pipeline
    //   OMNISIM_R34_IDENTITY_VP — bypasses the view-projection to isolate pipeline from matrix
    //                             bugs; a shadow pass fitted to the REAL camera would be
    //                             inconsistent with an identity-VP scene and prove nothing.
    const int depthLevel = qEnvironmentVariableIntValue("OMNISIM_CAMERA_DEPTH");
    const bool agxMode = qEnvironmentVariableIntValue("OMNISIM_CAMERA_AGX") >= 1;
    const bool identityVp = qEnvironmentVariableIntValue("OMNISIM_R34_IDENTITY_VP") != 0;
    const bool wantSceneRender =
      isWgpuCameraSceneEnabled() && depthLevel <= 0 && !agxMode && !identityVp;

    // Per-device cached draw list (see OmAbstractCamera::collectWgpuDrawsCached): the vectors
    // belong to the device and are refreshed, not rebuilt, on a frame with no structural change.
    std::vector<OmWgpuSolidDraw> *drawsPtr = nullptr;
    std::vector<std::array<float, 16>> *modelPtr = nullptr;
    // W3: the albedo/roughness/metalness/normal maps are uploaded by the collect itself, but ONLY
    // when it is handed a texture cache. Creating it here (rather than in ensureWgpuTarget) keeps
    // it off every gated-off and every depth-mode frame, so OMNISIM_WGPU_CAMERA_SCENE=0 does not
    // merely skip the render — it never allocates a texture at all.
    if (wantSceneRender && !mWgpuTextureCache) {
      OmRenderBackend *const back = renderBackend();
      // ensureWgpuTarget() already returned true, so the backend IS a usable Vulkan/wgpu one;
      // re-check anyway rather than blind-casting.
      if (back && back->kind() == OmRenderBackendKind::Vulkan && back->isAvailable())
        mWgpuTextureCache = new OmWgpuTextureCache(static_cast<OmVulkanBackend *>(back));
    }
    // R5c: scene walk via the shared sensor scene-renderer (same coverage as
    // the former inline collectShapeDraws — Box/Plane/Sphere/Cylinder/Capsule
    // primitives + WREN-readback fallback).
    bool texturesEvicted = false;
    collectWgpuDrawsCached(*mWgpuMeshCache, wantSceneRender ? mWgpuTextureCache : nullptr, drawsPtr, modelPtr,
                           &texturesEvicted);
    if (texturesEvicted && mWgpuTarget)
      mWgpuTarget->forgetTextureBindGroups();  // the cache released views; the cached bind groups are dead
    std::vector<OmWgpuSolidDraw> &draws = *drawsPtr;
    std::vector<std::array<float, 16>> &modelStorage = *modelPtr;
    (void)modelStorage;

    // viewProj from this Camera's pose + FOV + near/far + aspect, via the
    // shared scene-renderer (encodes the Webots +X-fwd/+Z-up → wgpu -Z-fwd/
    // +Y-up basis swap; identical math to the former inline block).
    const double aspect =
      mWgpuTargetHeight > 0 ? static_cast<double>(mWgpuTargetWidth) / mWgpuTargetHeight : 1.0;
    // W3 defect 2: Camera.wrl defaults `far 0`, which means INFINITE, not zero.
    // WREN substitutes a finite 10 km plane for it inside wren::Camera::setFar
    // (src/wren/Camera.hpp:51-53), so the WREN sensor path has always rendered
    // with a usable far plane. Feeding the raw 0 to perspective() produces
    // m22 = zFar/(zNear-zFar) = 0 and m32 = 0, i.e. clip.z == 0 for EVERY
    // fragment -- the depth test degenerates and visibility becomes draw-order
    // dependent. sensorFarPlane applies WREN's exact predicate and constant, so
    // any world declaring a positive `far` (all nine wgpu probe worlds declare
    // `far 10.0`) is bit-identical to before.
    const double authoredFar = mFar->value();
    const double effectiveFar = OmWgpuSceneRenderer::sensorFarPlane(authoredFar);
    if (effectiveFar != authoredFar && !mWgpuFarSubstitutionLogged) {
      mWgpuFarSubstitutionLogged = true;
      OmLog::info(QString("[OmCamera] '%1' declares far=%2 (means infinite); the wgpu sensor "
                          "projection substitutes %3 m, matching WREN's wren::Camera::setFar")
                    .arg(deviceName())
                    .arg(authoredFar, 0, 'f', 3)
                    .arg(effectiveFar, 0, 'f', 1));
    }
    float viewProj[16] = {0};
    OmWgpuSceneRenderer::buildViewProj(matrix(), fieldOfView(), aspect, mNear->value(),
                                       effectiveFar, viewProj);

    const float lightDirAmbient[4] = {0.3f, 0.4f, -0.85f, 0.25f};
    OmWgpuClearColor clear = {0.18f, 0.18f, 0.18f, 1.0f};
    // R3.4-step-4 diagnostic: OMNISIM_R34_IDENTITY_VP=1 bypasses
    // the viewProj math (mesh local coords land directly in NDC).
    // A unit-box at the origin fills most of the screen; useful for
    // isolating viewProj bugs from pipeline bugs.
    float vpIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const float *vpToUse = identityVp ? vpIdentity : viewProj;

    // R3.7 / Phase δ — Newton → wgpu interop demo (the marquee: physics-GPU
    // body positions feed the render-GPU vertex shader). When OMNISIM_PROBE_DELTA
    // is set, draw the LIVE Newton body translations as instances through the
    // storage-buffer instanced path (R3.7) using THIS camera's viewProj, instead
    // of the per-Solid scene walk. Opt-in; default render path unchanged. The
    // newton dynamic_cast returns null on ODE / NEWTON-OFF builds (→ skip).
    if (qEnvironmentVariableIntValue("OMNISIM_PROBE_DELTA") >= 1) {
      OmNewtonBackend *nb =
          dynamic_cast<OmNewtonBackend *>(OmPhysicsBackendRegistry::newtonBackend());
      static float sBodyBuf[4 * 256];
      const int nbod = nb ? nb->snapshotBodyTranslations(256, sBodyBuf) : -1;
      if (nbod > 0) {
        // Small +Z-normal triangle (pos3/norm3/uv2, stride 32 B), one instance
        // per body; each instance offset by bodies[iid].xyz in the shader.
        static const float kDeltaVerts[24] = {
          -0.15f, -0.15f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
           0.15f, -0.15f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
           0.0f,   0.15f, 0.0f, 0.0f, 0.0f, 1.0f, 0.5f, 1.0f,
        };
        static const unsigned int kDeltaIdx[3] = {0, 1, 2};
        OmWgpuMeshHandle dm = mWgpuMeshCache->acquire(
            0xDE17Aull, kDeltaVerts, sizeof(kDeltaVerts), kDeltaIdx, sizeof(kDeltaIdx), 3, 32);
        if (dm.vertexBuffer &&
            mWgpuTarget->clearAndDrawInstanced(clear, vpToUse, sBodyBuf,
                                               static_cast<uint32_t>(nbod), dm.vertexBuffer,
                                               dm.indexBuffer, dm.indexCount, data)) {
          // W3 defect 1: RGBA8 readback -> BGRA controller buffer.
          swizzleWgpuRgbaToBgra(data);
          static thread_local int sDeltaLog = 0;
          if (sDeltaLog < 8) {
            OmLog::info(QString("[OmCamera δ] %1 LIVE Newton bodies → wgpu instanced "
                                "storage-buffer draw via '%2'; body0=(%3,%4,%5)")
                          .arg(nbod)
                          .arg(deviceName())
                          .arg(sBodyBuf[0], 0, 'f', 3)
                          .arg(sBodyBuf[1], 0, 'f', 3)
                          .arg(sBodyBuf[2], 0, 'f', 3));
            ++sDeltaLog;
          }
          return;  // δ probe owns this frame
        }
      }
    }

    // T1.2 CSM sub-step 2b: OMNISIM_CAMERA_LIGHTDEPTH=1 renders the scene DEPTH
    // from a LIGHT's point of view (the shadow-map half of CSM) instead of the
    // camera's, using the orthographic light-space viewProj
    // (buildOrthoLightViewProj) + the clip.z depth pipeline
    // (clearAndDrawSceneDepthF32 clipDepth=true). VERIFIED 2026-05-31: center
    // reads 0.4690 ≈ predicted (4.7-0.05)/9.95 = 0.467 (the earlier center=1.0
    // was a pipeline-selection bug — the clipDepth path rendered the clip.w F32
    // pipeline, ≡1 under ortho; fixed by selecting mScnClipDepthPipeline). The
    // light is a FIXED DIAGNOSTIC
    // overhead pose so the readback is hand-checkable; a real CSM derives the
    // light frame from the scene light direction + cascade bounds. Default off →
    // byte-identical (the lit/depth/range paths are untouched). The normalized
    // grayscale is just so the 8-bit image isn't blank.
    // T1.2 CSM sub-step 3b probe: OMNISIM_CAMERA_SHADOWPROBE=1 force-builds the
    // shadow-receiving lit pipeline (naga-validates kSolidLitShadow + its 3-entry
    // bind-group layout) and logs PASS/FAIL, without rendering anything. A
    // headless check that the 3a shader is valid before the two-pass wiring (3c).
    // Default off → no effect; the normal frame proceeds below.
    if (qEnvironmentVariableIntValue("OMNISIM_CAMERA_SHADOWPROBE") >= 1) {
      const bool okShadow = mWgpuTarget->probeShadowPipeline();
      static thread_local int sShadowProbeLog = 0;
      if (sShadowProbeLog < 4) {
        OmLog::info(QString("[OmCamera] '%1' [T1.2 CSM shadow-pipeline probe: %2]")
                      .arg(deviceName())
                      .arg(okShadow ? "PASS (kSolidLitShadow built/validated)"
                                    : "FAIL (pipeline build returned false)"));
        ++sShadowProbeLog;
      }
    }
    // T1.2 CSM sub-step 3c: OMNISIM_CAMERA_SHADOW two-pass cast-shadow render
    // (light clip-depth → sampleable map → lit pass samples it). =1 strength 0
    // (plumbing isolation), >=2 strength 0.6 (shadowing). Fixed diagnostic
    // overhead light. Gated, default-off → byte-identical.
    if (qEnvironmentVariableIntValue("OMNISIM_CAMERA_SHADOW") >= 1) {
      const OmMatrix4 lightWorld(1.0, 0.0, 5.0, 0.0, 1.0, 0.0, M_PI_2);
      float lightVP[16] = {0};
      OmWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, 2.0, 0.05, 10.0, lightVP);
      const float strength =
        qEnvironmentVariableIntValue("OMNISIM_CAMERA_SHADOW") >= 2 ? 0.6f : 0.0f;
      if (mWgpuTarget->clearAndDrawSceneShadowed(clear, viewProj, lightVP, lightDirAmbient,
                                                 draws.empty() ? nullptr : draws.data(),
                                                 static_cast<uint32_t>(draws.size()),
                                                 strength, 0.002f, data)) {
        // W3 defect 1: RGBA8 readback -> BGRA controller buffer. Done BEFORE the
        // log below, whose "center BGRA=" line then reports genuine BGRA.
        swizzleWgpuRgbaToBgra(data);
        static thread_local int sShadowLog = 0;
        if (sShadowLog < 30) {
          const size_t cx = static_cast<size_t>(mWgpuTargetWidth) / 2;
          const size_t cy = static_cast<size_t>(mWgpuTargetHeight) / 2;
          const size_t ci = (cy * static_cast<size_t>(mWgpuTargetWidth) + cx) * 4;
          OmLog::info(QString("[OmCamera] '%1' [T1.2 CSM two-pass shadow strength %2: "
                              "center BGRA=(%3,%4,%5)]")
                        .arg(deviceName()).arg(strength, 0, 'f', 1)
                        .arg(data[ci + 0]).arg(data[ci + 1]).arg(data[ci + 2]));
          // T1.2 CSM 3d: dump a GR x GC grid of the G channel so an A/B
          // (strength 0 vs 0.6) reveals WHERE darkening falls — a contiguous
          // sub-region darkening proves a localized cross-object cast shadow
          // (vs a global darken / self-shadow). Gated on _GRID, off by default.
          if (qEnvironmentVariableIntValue("OMNISIM_CAMERA_SHADOW_GRID") >= 1) {
            const int GR = 8, GC = 10;
            QString grid;
            for (int r = 0; r < GR; ++r) {
              const size_t gy =
                static_cast<size_t>((r + 1) * mWgpuTargetHeight / (GR + 1));
              for (int c = 0; c < GC; ++c) {
                const size_t gx =
                  static_cast<size_t>((c + 1) * mWgpuTargetWidth / (GC + 1));
                const size_t gi =
                  (gy * static_cast<size_t>(mWgpuTargetWidth) + gx) * 4;
                grid += QString("%1").arg(data[gi + 1], 4);  // G channel
              }
              grid += " /";
            }
            OmLog::info(QString("[OmCamera] '%1' [T1.2 CSM 3d grid G strength %2 frame %4:%3]")
                          .arg(deviceName()).arg(strength, 0, 'f', 1).arg(grid).arg(sShadowLog));
          }
          ++sShadowLog;
        }
        return;
      }
    }
    if (qEnvironmentVariableIntValue("OMNISIM_CAMERA_LIGHTDEPTH") >= 1) {
      // Light at (1,0,5) looking straight down: local +X (buildView's forward)
      // maps to world -Z via an axis-(0,1,0) angle +π/2 rotation. Over the
      // camera_wgpu_scene_smoke box (centre (1,0,0), size 0.6 → top face z=0.3),
      // the centre ray's light-space NDC depth (ortho near 0.05, far 10) is
      // (dist-near)/(far-near) = (4.7-0.05)/9.95 ≈ 0.4673.
      const OmMatrix4 lightWorld(1.0, 0.0, 5.0, 0.0, 1.0, 0.0, M_PI_2);
      float lightVP[16] = {0};
      OmWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, 1.0, 0.05, 10.0, lightVP);
      std::vector<float> ndc(
        static_cast<size_t>(mWgpuTargetWidth) * static_cast<size_t>(mWgpuTargetHeight), 1.0f);
      if (mWgpuTarget->clearAndDrawSceneDepthF32(1.0f, lightVP,
                                                 draws.empty() ? nullptr : draws.data(),
                                                 static_cast<uint32_t>(draws.size()),
                                                 ndc.data(), /*clipDepth=*/true)) {
        const size_t n = ndc.size();
        for (size_t i = 0; i < n; ++i) {
          float g = ndc[i] < 0.0f ? 0.0f : (ndc[i] > 1.0f ? 1.0f : ndc[i]);
          const unsigned char v = static_cast<unsigned char>(g * 255.0f + 0.5f);
          data[i * 4 + 0] = v;
          data[i * 4 + 1] = v;
          data[i * 4 + 2] = v;
          data[i * 4 + 3] = 255;
        }
        static thread_local int sLightDepthLog = 0;
        if (sLightDepthLog < 4) {
          const size_t cx = static_cast<size_t>(mWgpuTargetWidth) / 2;
          const size_t cy = static_cast<size_t>(mWgpuTargetHeight) / 2;
          OmLog::info(QString("[OmCamera] '%1' rendered through wgpu (%2x%3, %4 draws) "
                              "[T1.2 CSM light-space clip-depth: center=%5 (expect ~0.467)]")
                        .arg(deviceName())
                        .arg(mWgpuTargetWidth)
                        .arg(mWgpuTargetHeight)
                        .arg(static_cast<int>(draws.size()))
                        .arg(ndc[cy * static_cast<size_t>(mWgpuTargetWidth) + cx], 0, 'f', 4));
          ++sLightDepthLog;
        }
        return;
      }
      // light-depth path failed → fall through to the normal camera paths.
    }

    // R5: OMNISIM_CAMERA_DEPTH renders view-space distance instead of lit color
    // through the depth scene pipeline (a wgpu RangeFinder proof). Default off
    // (0) → the lit scene-walk path is byte-unchanged.
    //   =1   grayscale RGBA8 (distance/far, 8-bit) — the original R5 proof.
    //   =2   R5b: REAL metric (planar) depth via the R32Float path, logged at
    //        full float precision, then normalized to grayscale for the 8-bit
    //        camera image. The actual RangeFinder-device output shape.
    //   >=3  R5d: RADIAL range (euclidean distance from the camera) via the
    //        Lidar range path — off-axis pixels read larger than the planar
    //        value. The Lidar-device output shape.
    // (`depthLevel` is read once, above the collect — the W3 scene render has to know the mode
    //  before it decides whether to upload textures.)
    if (depthLevel >= 3) {
      const float farVal = static_cast<float>(effectiveFar);
      float viewMat[16] = {0};
      OmWgpuSceneRenderer::buildView(matrix(), viewMat);
      std::vector<float> meters(
        static_cast<size_t>(mWgpuTargetWidth) * static_cast<size_t>(mWgpuTargetHeight), farVal);
      // Use the real viewProj (not vpToUse) — the range shader's view matrix
      // must be consistent with it; the IDENTITY_VP diagnostic is depth-only.
      if (mWgpuTarget->clearAndDrawSceneRangeF32(farVal, viewProj, viewMat,
                                                 draws.empty() ? nullptr : draws.data(),
                                                 static_cast<uint32_t>(draws.size()),
                                                 meters.data())) {
        const size_t n = meters.size();
        for (size_t i = 0; i < n; ++i) {
          float g = farVal > 1e-3f ? meters[i] / farVal : 0.0f;
          g = g < 0.0f ? 0.0f : (g > 1.0f ? 1.0f : g);
          const unsigned char v = static_cast<unsigned char>(g * 255.0f + 0.5f);
          data[i * 4 + 0] = v;
          data[i * 4 + 1] = v;
          data[i * 4 + 2] = v;
          data[i * 4 + 3] = 255;
        }
        static thread_local int sRangeLog = 0;
        if (sRangeLog < 4) {
          const int cx = mWgpuTargetWidth / 2;
          const int cy = mWgpuTargetHeight / 2;
          const float centerR = meters[static_cast<size_t>(cy) * mWgpuTargetWidth + cx];
          const float cornerR = meters[0];  // top-left = max off-axis angle
          OmLog::info(QString("[OmCamera] '%1' rendered through wgpu (%2x%3, %4 draws) "
                              "[R5d Lidar radial-range: center=%5 m corner=%6 m]")
                        .arg(deviceName())
                        .arg(mWgpuTargetWidth)
                        .arg(mWgpuTargetHeight)
                        .arg(static_cast<int>(draws.size()))
                        .arg(centerR, 0, 'f', 4)
                        .arg(cornerR, 0, 'f', 4));
          ++sRangeLog;
        }
        return;
      }
      // range path failed → fall through to planar/grayscale/lit below.
    }
    if (depthLevel >= 2) {
      const float farVal = static_cast<float>(effectiveFar);
      std::vector<float> meters(
        static_cast<size_t>(mWgpuTargetWidth) * static_cast<size_t>(mWgpuTargetHeight), farVal);
      if (mWgpuTarget->clearAndDrawSceneDepthF32(farVal, vpToUse,
                                                 draws.empty() ? nullptr : draws.data(),
                                                 static_cast<uint32_t>(draws.size()),
                                                 meters.data())) {
        const size_t n = meters.size();
        for (size_t i = 0; i < n; ++i) {
          float g = farVal > 1e-3f ? meters[i] / farVal : 0.0f;
          g = g < 0.0f ? 0.0f : (g > 1.0f ? 1.0f : g);
          const unsigned char v = static_cast<unsigned char>(g * 255.0f + 0.5f);
          data[i * 4 + 0] = v;
          data[i * 4 + 1] = v;
          data[i * 4 + 2] = v;
          data[i * 4 + 3] = 255;
        }
        static thread_local int sF32Log = 0;
        if (sF32Log < 4) {
          const size_t cx = static_cast<size_t>(mWgpuTargetWidth) / 2;
          const size_t cy = static_cast<size_t>(mWgpuTargetHeight) / 2;
          OmLog::info(QString("[OmCamera] '%1' rendered through wgpu (%2x%3, %4 draws) "
                              "[R5b depth-F32 real-meters: center=%5 m]")
                        .arg(deviceName())
                        .arg(mWgpuTargetWidth)
                        .arg(mWgpuTargetHeight)
                        .arg(static_cast<int>(draws.size()))
                        .arg(meters[cy * static_cast<size_t>(mWgpuTargetWidth) + cx], 0, 'f', 4));
          ++sF32Log;
        }
        return;
      }
      // F32 path failed → fall through to the grayscale/lit path below.
    }
    const bool depthMode = depthLevel >= 1;
    // T1.1: OMNISIM_CAMERA_AGX=1 runs the lit colour through the AgX filmic
    // tonemap (kSolidLitAgX pipeline). Default off → plain Lambertian, byte-
    // identical (golden-image-gated). Ignored under depthMode.
    // (`agxMode` is read once, above the collect — same reason as `depthLevel`.)
    // The clearAndDrawScene `farPlane` arg doubles as AgX exposure when agxMode
    // (the two modes share pad0.x and are mutually exclusive). Exposure = 2^EV
    // from OMNISIM_CAMERA_AGX_EV (integer stops; default 0 → 1.0 → byte-identical
    // to the no-exposure AgX path). Depth/lit path passes the real far plane.
    float farOrExposure = static_cast<float>(effectiveFar);
    if (agxMode && !depthMode) {
      const int ev = qEnvironmentVariableIntValue("OMNISIM_CAMERA_AGX_EV");
      farOrExposure = static_cast<float>(std::pow(2.0, static_cast<double>(ev)));
    }
    // T1.1 specular: hand the camera world position to the AgX path so the
    // gated Blinn-Phong highlight can form its view vector. Only the (gated)
    // specular branch reads it; null/zero on every non-AgX or rough-surface
    // frame leaves output byte-identical.
    const OmVector3 camWorld = matrix().translation();
    const float camPos3[3] = {static_cast<float>(camWorld.x()), static_cast<float>(camWorld.y()),
                              static_cast<float>(camWorld.z())};

    // ===================== W3: THE SENSOR SCENE RENDER =====================
    //
    // Everything below the gate is the layer the wgpu sensor never had: the world's own
    // materials (albedo/roughness/metalness/normal maps, uploaded by the collect above through
    // mWgpuTextureCache), its own Background, its own lights, its own Fog, and a cast-shadow
    // pass — harvested from exactly the same nodes the wgpu MAIN VIEW harvests them from, so a
    // Camera device and the viewport are looking at one lighting model rather than two.
    //
    // OUTPUT CONTRACT — the one thing here that is NOT the main view's:
    // OM_WGPU_XFER_WREN_SENSOR. A Camera device is not a display surface; WREN's colour Camera
    // installs OmWrenHdr, whose hdr_resolve.frag is `1 - exp(-c * exposure)` then `pow(., 1/2.2)`,
    // and tests/api/controllers/camera_checker.c pins those ENCODED bytes to +-2. So this render
    // must reproduce WREN's CURVE — it must NOT emit linear light, and it must NOT take the main
    // view's AgX or its camera pass (vignette is a spatial gain, grain is time-varying, and
    // auto-exposure would make this frame's bytes a function of the previous frame). See the
    // OmWgpuOutputTransfer comment in OmWgpuRenderTarget.hpp.
    //
    // NOT touched, deliberately: the SEGMENTATION image (D1.4: produced nowhere any more —
    // it never reaches this copy; when it existed it had to bypass this render because any
    // tonemap or sRGB step would corrupt the recognitionColors ids, and that constraint
    // binds again if segmentation is ever re-implemented on wgpu), the DEPTH/RANGE float
    // buffers (their modes stand this path down via `wantSceneRender`), and the MAIN VIEW
    // (which keeps the default OM_WGPU_XFER_VIEW everywhere).
    //
    // Cost: this is a genuinely bigger frame than the flat path — a light-depth pass, a 2048^2
    // shadow map, 4x MSAA, an RGBA16F scene target and a tonemap pass. On a 64x48 probe camera
    // that is noise; on a 1280x720 Camera stepping every basic timestep it is not.
    // OMNISIM_WGPU_CAMERA_SCENE=0 is the way back to the cheap flat render.
    if (wantSceneRender) {
      OmWorld *const world = OmWorld::instance();
      OmGroup *const worldRoot = world ? world->root() : NULL;

      // ---- Sky: the world's Background, used as BOTH the clear colour and the hemisphere
      // ambient. The fallback sky-blue (and the ambient floor below it) are the main view's, and
      // they exist for the same reason: a world with no Background, or a near-black one
      // (TexturedBackground / NightSky), otherwise crushes every shadowed surface to black.
      float skyC[3] = {0.45f, 0.62f, 0.85f};
      OmBackground *const bgNode = worldRoot ? findFirstBackgroundCam(worldRoot) : NULL;
      if (bgNode) {
        const OmRgb sc = bgNode->skyColor();
        const float sr = static_cast<float>(sc.red());
        const float sg = static_cast<float>(sc.green());
        const float sb = static_cast<float>(sc.blue());
        if (sr + sg + sb > 0.05f) {  // keep the default for an unset/black sky
          skyC[0] = sr;
          skyC[1] = sg;
          skyC[2] = sb;
        }
      }
      OmWgpuClearColor sky;
      sky.r = skyC[0];
      sky.g = skyC[1];
      sky.b = skyC[2];
      sky.a = 1.0f;
      const float ambSky[3] = {skyC[0] > 0.40f ? skyC[0] : 0.40f, skyC[1] > 0.42f ? skyC[1] : 0.42f,
                               skyC[2] > 0.45f ? skyC[2] : 0.45f};
      const float hemiSky4[4] = {ambSky[0], ambSky[1], ambSky[2], 0.45f};
      const float hemiGround4[4] = {ambSky[0] * 0.30f, ambSky[1] * 0.29f, ambSky[2] * 0.27f, 0.0f};
      float worldUp3[3] = {0.0f, 0.0f, 1.0f};
      if (world && world->worldInfo()) {
        const OmVector3 up = -world->worldInfo()->gravityUnitVector();  // up = opposite gravity
        if (up.length() > 1e-6) {
          worldUp3[0] = static_cast<float>(up.x());
          worldUp3[1] = static_cast<float>(up.y());
          worldUp3[2] = static_cast<float>(up.z());
        }
      }

      // ---- Sun: the first DirectionalLight in the tree (the OmniSimSun PROTO's light on every
      // world that follows docs/WORLD_RECIPE.md), with its REAL direction, colour and intensity.
      // The fallbacks are the main view's, and the 2.5 is not arbitrary — it is OmniSimSun's own
      // default intensity, so a sun-less world still gets a plausible key light on the HDR arm
      // (where sunEnergy3 IS the direct term: pass zeros and nothing is lit at all).
      float lit4[4] = {0.3f, 0.4f, -0.85f, 0.05f};
      float sunColor3[3] = {1.0f, 1.0f, 1.0f};
      float sunEnergy3[3] = {2.5f, 2.5f, 2.5f};
      OmDirectionalLight *const sunNode =
        worldRoot ? findFirstDirectionalLightCam(worldRoot) : NULL;
      if (sunNode) {
        const OmVector3 sd = sunNode->direction().normalized();
        lit4[0] = static_cast<float>(sd.x());
        lit4[1] = static_cast<float>(sd.y());
        lit4[2] = static_cast<float>(sd.z());
        const OmRgb sc = sunNode->color();
        sunColor3[0] = static_cast<float>(sc.red());
        sunColor3[1] = static_cast<float>(sc.green());
        sunColor3[2] = static_cast<float>(sc.blue());
        const float e = sunNode->isOn() ? static_cast<float>(sunNode->intensity()) : 0.0f;
        sunEnergy3[0] = sunColor3[0] * e;
        sunEnergy3[1] = sunColor3[1] * e;
        sunEnergy3[2] = sunColor3[2] * e;
      }
      // Every other light (point / spot / additional directionals) as unshadowed fills. Empty on
      // the single-sun common case, which leaves the shader's extra-light loop dead.
      std::vector<float> extraLights;
      if (worldRoot)
        collectExtraLightsCam(worldRoot, sunNode, extraLights);

      // Day-night: the same sun-elevation dimmer the main view applies to its DIRECT term, so a
      // sensor darkens in step with the sky dome instead of staying noon-lit under a night sky
      // (beauty_bench's sun_marker moves the sun every step, so this is a live case, not a
      // hypothetical). Full day clamps to 1.0, which the shader treats identically to "off" —
      // so on a static daylight world this changes nothing.
      const OmVector3 towardSun(-lit4[0], -lit4[1], -lit4[2]);
      const OmVector3 upWorld(worldUp3[0], worldUp3[1], worldUp3[2]);
      const double sunElev = towardSun.normalized().dot(upWorld.normalized());
      const float dayF = static_cast<float>(std::max(0.05, std::min(1.0, sunElev * 5.0 + 0.5)));

      // ---- Fog: WREN renders the world's Fog node into a camera image, so this path must too,
      // or every far-field pixel of a foggy world disagrees with WREN by the haze amount.
      float fogParams4[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      if (OmFog *const fog = OmFog::fogInstance()) {
        if (fog->fogColor() && fog->fogVisibilityRange() &&
            fog->fogVisibilityRange()->value() > 0.1) {
          const OmRgb fc = fog->fogColor()->value();
          fogParams4[0] = static_cast<float>(fc.red());
          fogParams4[1] = static_cast<float>(fc.green());
          fogParams4[2] = static_cast<float>(fc.blue());
          // WREN-exact: density = 1 / visibilityRange (the shader applies exp2 + the pow-2.2 blend).
          fogParams4[3] = static_cast<float>(1.0 / fog->fogVisibilityRange()->value());
        }
      }

      // ---- Shadow light frustum: one orthographic light camera looking along the sun. Same
      // construction as the main view's fitted single map, with two sensor-specific choices:
      // the half-extent is fitted to what THIS sensor can see (a `far 10` Camera would waste
      // ~99% of a 45 m box, i.e. throw away shadow-map resolution it needs), and there is no
      // texel-grid snapping (that exists to stop shadow edges shimmering as a viewer PANS; a
      // sensor is nailed to its robot and the frustum follows it rigidly).
      const OmVector3 sdir = OmVector3(lit4[0], lit4[1], lit4[2]).normalized();
      OmVector3 lightAxis = OmVector3(1.0, 0.0, 0.0).cross(sdir);
      const double laLen = lightAxis.length();
      const double laAng =
        std::acos(std::max(-1.0, std::min(1.0, OmVector3(1.0, 0.0, 0.0).dot(sdir))));
      lightAxis = laLen > 1e-6 ? lightAxis / laLen : OmVector3(0.0, 1.0, 0.0);
      const OmVector3 camFwd = matrix().xAxis().normalized();  // camera-local +X is forward
      // A `far 0` Camera resolves to the 10 km substitute, so both of these are CLAMPED rather
      // than proportional: the box is capped at 60 m (the main view's is a fixed 45 m) and the
      // anchor at 0.6 of it, which keeps the camera itself inside the box so geometry BEHIND it
      // still casts into the frame.
      const double halfExtent = std::max(2.0, std::min(60.0, effectiveFar * 0.75));
      const OmVector3 anchor =
        camWorld + camFwd * std::min(effectiveFar * 0.5, halfExtent * 0.6);
      const OmVector3 lightPos = anchor - sdir * (halfExtent * 3.0 + 10.0);
      const OmMatrix4 lightWorld(lightPos.x(), lightPos.y(), lightPos.z(), lightAxis.x(),
                                 lightAxis.y(), lightAxis.z(), laAng);
      float lightVP[16] = {0};
      OmWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, halfExtent, 0.05,
                                                   halfExtent * 6.0 + 20.0, lightVP);

      // ---- Exposure: the Camera's OWN `exposure` field (Camera.wrl default 1.0), which is
      // exactly what WREN feeds OmWrenHdr::setExposure. It must be > 0 or the render target's
      // HDR arm does not engage at all and the frame falls back to the scene shader's own
      // display encode (still a real render — just exposure-free).
      const float exposureVal =
        (mExposure && mExposure->value() > 0.0) ? static_cast<float>(mExposure->value()) : 1.0f;

      // ---- Atmospheric sky dome: only for a world that actually declares an atmosphericSky
      // preset. A world whose Background carries an image CUBEMAP is deliberately NOT given the
      // dome — there is no cubemap upload on this path, and painting the procedural DAY dome
      // over a NightSky world would be worse than the flat clear it falls back to.
      const bool skyAtmo = bgNode && !bgNode->atmosphericSkyPreset().isEmpty();
      const bool skyMars = skyAtmo && bgNode->atmosphericSkyPreset() == QStringLiteral("mars");
      const float illumBase = 22.0f;  // main-view default (OMNISIM_WGPU_SKY_ILLUM's fallback)
      const OmVector3 camUp = matrix().zAxis().normalized();     // camera-local +Z is up
      const OmVector3 camRgt = -matrix().yAxis().normalized();   // camera-local +Y is LEFT
      const double halfW = std::tan(0.5 * fieldOfView());
      const double halfH = aspect > 1e-9 ? halfW / aspect : halfW;
      const float skyU[28] = {
        static_cast<float>(camRgt.x() * halfW), static_cast<float>(camRgt.y() * halfW),
        static_cast<float>(camRgt.z() * halfW), 0.0f,  // right.w = scatter mode (LUT: main view only)
        static_cast<float>(camUp.x() * halfH), static_cast<float>(camUp.y() * halfH),
        static_cast<float>(camUp.z() * halfH), 0.0f,   // up.w — overwritten with the far-plane z
        static_cast<float>(camFwd.x()), static_cast<float>(camFwd.y()),
        static_cast<float>(camFwd.z()), 0.0f,          // fwd.w = cloud coverage (scatter only)
        -lit4[0], -lit4[1], -lit4[2], 0.0f,            // sunDir = TOWARD the sun
        sunColor3[0], sunColor3[1], sunColor3[2],
        exposureVal > 0.0f ? 1.0f : 0.0f,              // sunColor.w = HDR linear-radiance arm
        worldUp3[0], worldUp3[1], worldUp3[2], 0.0f,   // worldUp.w = sky mode (1 = image cubemap)
        skyMars ? illumBase * (10.0f / 22.0f) : illumBase,
        skyMars ? illumBase * (8.5f / 22.0f) : illumBase,
        skyMars ? illumBase * (7.0f / 22.0f) : illumBase,
        0.99995f};                                     // sun disc cos radius (WREN's discCos)

      if (mWgpuTarget->clearAndDrawSceneTexturedShadowed(
            sky, viewProj, lightVP, lit4, draws.empty() ? nullptr : draws.data(),
            static_cast<uint32_t>(draws.size()),
            /*shadowStrength=*/0.8f, /*depthBias=*/0.0006f, camPos3, hemiSky4, hemiGround4,
            worldUp3, data,
            /*asyncReadback=*/false,  // a sensor must deliver THIS frame's pixels, never last frame's
            skyAtmo ? skyU : nullptr,
            /*directScale=*/dayF,     // sun-elevation dimmer; 1.0 in full day = the undimmed term
            fogParams4[3] > 0.0f ? fogParams4 : nullptr,
            /*bloomStrength=*/0.0f,   // bloom is a lens artefact, not a measurement
            /*agxExposure=*/exposureVal, /*ssaoStrength=*/0.0f,
            /*reversedZ=*/false,      // also keeps TAA jitter and SSR off — both are sensor noise
            extraLights.empty() ? nullptr : extraLights.data(),
            static_cast<uint32_t>(extraLights.size() / 16), /*ssaoRadiusScale=*/1.0f, sunEnergy3,
            /*ssaoNearZ=*/static_cast<float>(mNear->value()),
            /*cascadeLightViewProjs=*/nullptr, /*cascadeSplitsFar4=*/nullptr, /*cascadeCount=*/0,
            /*bloomHdrThreshold=*/0.0f, /*skyScatter24=*/nullptr, /*iblSky8=*/nullptr,
            OM_WGPU_XFER_WREN_SENSOR)) {
        // W3 defect 1: RGBA8 readback -> BGRA controller buffer, exactly as on every other
        // successful wgpu sensor branch, and BEFORE the log so its bytes are genuine BGRA.
        swizzleWgpuRgbaToBgra(data);
        // Lane E1 (P5): CPU port of the WREN post-FX (motionBlur -> noise, the PP_* pass
        // order) on the final BGRA bytes. Production frame only -- the diagnostic modes
        // below keep their instrument output clean.
        applyWgpuColorPostFx(data, mWgpuTargetWidth, mWgpuTargetHeight, mWgpuBlurHistory);
        if (!mWgpuScenePathLogged) {
          mWgpuScenePathLogged = true;
          const size_t cx = static_cast<size_t>(mWgpuTargetWidth) / 2;
          const size_t cy = static_cast<size_t>(mWgpuTargetHeight) / 2;
          const size_t ci = (cy * static_cast<size_t>(mWgpuTargetWidth) + cx) * 4;
          OmLog::info(
            QString("[OmCamera] '%1' rendered through wgpu (%2x%3, %4 draws) [W3 scene: sun=%5 "
                    "extraLights=%6 sky=%7 fog=%8 exposure=%9 center BGRA=(%10,%11,%12)]")
              .arg(deviceName())
              .arg(mWgpuTargetWidth)
              .arg(mWgpuTargetHeight)
              .arg(static_cast<int>(draws.size()))
              .arg(sunNode ? "world" : "fallback")
              .arg(static_cast<int>(extraLights.size() / 16))
              .arg(skyAtmo ? "atmospheric dome" : (bgNode ? "Background skyColor" : "default"))
              .arg(fogParams4[3] > 0.0f ? "on" : "off")
              .arg(exposureVal, 0, 'f', 2)
              .arg(data[ci + 0])
              .arg(data[ci + 1])
              .arg(data[ci + 2]));
        }
        return;
      }
      // Scene render failed (pipeline build, MSAA/HDR targets, readback) → fall through to the
      // flat lit path below rather than hand the controller nothing.
    }

    // srgbEncode=false: the camera is a SENSOR, not a display surface — keep its plain-lit
    // (kSolidLit) output in the R5-landed LINEAR space its controllers/ML consumers expect, NOT
    // the display-sRGB the main pane uses. Only the plain-lit path reads this (depth/AgX modes
    // write farPlane/exposure to the same pad0.x slot and are unaffected).
    if (mWgpuTarget->clearAndDrawScene(clear, vpToUse, lightDirAmbient,
                                       draws.empty() ? nullptr : draws.data(),
                                       static_cast<uint32_t>(draws.size()), data,
                                       depthMode, farOrExposure, agxMode, camPos3,
                                       /*pickMode=*/false, /*srgbEncode=*/false)) {
      // W3 defect 1: RGBA8 readback -> BGRA controller buffer. Unconditional on
      // this branch: under depthMode the shader writes a grayscale image
      // (r == g == b), so the swap is a byte-for-byte no-op there and there is no
      // mode-dependent branch to get wrong.
      swizzleWgpuRgbaToBgra(data);
      // Lane E1 (P5): post-FX on the production lit frame only. The OMNISIM_CAMERA_DEPTH /
      // _AGX / _R34_IDENTITY_VP diagnostic modes own their frame and stay byte-clean --
      // noising a depth proof or an A/B instrument would corrupt the measurement.
      if (depthLevel <= 0 && !agxMode && !identityVp)
        applyWgpuColorPostFx(data, mWgpuTargetWidth, mWgpuTargetHeight, mWgpuBlurHistory);
      // One-time per Camera log so test harnesses + smoke runs can
      // verify the wgpu path actually fired (vs falling through to
      // WREN silently). The mWgpuTargetWidth check is harmless +
      // makes this fire on every resize as well.
      static thread_local int sLoggedCount = 0;
      if (sLoggedCount < 4) {
        OmLog::info(QString("[OmCamera] '%1' rendered through wgpu (%2x%3, %4 draw%5)%6")
                      .arg(deviceName())
                      .arg(mWgpuTargetWidth)
                      .arg(mWgpuTargetHeight)
                      .arg(static_cast<int>(draws.size()))
                      .arg(draws.size() == 1 ? "" : "s")
                      .arg(depthMode ? " [R5 depth/RangeFinder mode]" : ""));
        ++sLoggedCount;
      }
      return;
    }
    // Fall through to the base if the readback failed — it warns once that nothing
    // wrote pixels, which beats handing the controller a corrupt frame.
  }
  OmAbstractCamera::copyImageToMemoryMappedFile(data);
}

void OmCamera::preFinalize() {
  OmAbstractCamera::preFinalize();

  if (zoom())
    zoom()->preFinalize();

  if (recognition())
    recognition()->preFinalize();

  if (focus())
    focus()->preFinalize();

  if (lensFlare())
    lensFlare()->preFinalize();

  mRecognitionSensor = new OmSensor();

  updateNear();
  updateFar();
  updateExposure();
  updateBloomThreshold();
  updateAmbientOcclusionRadius();
}

void OmCamera::postFinalize() {
  OmAbstractCamera::postFinalize();

  if (zoom())
    zoom()->postFinalize();

  if (recognition()) {
    recognition()->postFinalize();
    updateRecognition();
  }

  if (focus()) {
    focus()->postFinalize();
    updateFocus();
  }

  connect(mFocus, &OmSFNode::changed, this, &OmCamera::updateFocus);
  connect(mRecognition, &OmSFNode::changed, this, &OmCamera::updateRecognition);
  connect(mNear, &OmSFDouble::changed, this, &OmCamera::updateNear);
  connect(mFar, &OmSFDouble::changed, this, &OmCamera::updateFar);
  connect(mExposure, &OmSFDouble::changed, this, &OmCamera::updateExposure);
  connect(mAmbientOcclusionRadius, &OmSFDouble::changed, this, &OmCamera::updateAmbientOcclusionRadius);
  connect(mBloomThreshold, &OmSFDouble::changed, this, &OmCamera::updateBloomThreshold);
  connect(mAntiAliasing, &OmSFBool::changed, this, &OmAbstractCamera::updateAntiAliasing);

  if (lensFlare())
    lensFlare()->postFinalize();
}

OmFocus *OmCamera::focus() const {
  return dynamic_cast<OmFocus *>(mFocus->value());
}

OmZoom *OmCamera::zoom() const {
  return dynamic_cast<OmZoom *>(mZoom->value());
}

OmRecognition *OmCamera::recognition() const {
  return dynamic_cast<OmRecognition *>(mRecognition->value());
}

OmLensFlare *OmCamera::lensFlare() const {
  return dynamic_cast<OmLensFlare *>(mLensFlare->value());
}

void OmCamera::initializeImageMemoryMappedFile() {
  OmAbstractCamera::initializeImageMemoryMappedFile();
  if (mImageMemoryMappedFile) {
    // initialize the memory mapped file with a black image
    int *im = reinterpret_cast<int *>(image());
    const int cameraSize = width() * height();
    for (int i = 0; i < cameraSize; i++)
      im[i] = 0xFF000000;
  }
}

void OmCamera::initializeSegmentationMemoryMappedFile() {
  cCameraNumber++;
  delete mSegmentationMemoryMappedFile;
  mSegmentationMemoryMappedFile = initializeMemoryMappedFile("segmentation");
  mHasSegmentationMemoryMappedFileChanged = true;
  if (mSegmentationMemoryMappedFile) {
    unsigned char *data = static_cast<unsigned char *>(mSegmentationMemoryMappedFile->data());
    // initialize the memory mapped file with a black image
    int *im = reinterpret_cast<int *>(data);
    const int cameraSize = width() * height();
    for (int i = 0; i < cameraSize; i++)
      im[i] = 0xFF000000;
  }
}

QString OmCamera::pixelInfo(int x, int y) const {
  // D1.4: the WREN texture readback is gone -- answer from the device's own BGRA buffer
  // (the very bytes the controller reads), which the wgpu render fills.
  int red = 0, green = 0, blue = 0;
  if (hasBeenSetup() && constImage() != NULL && x >= 0 && x < width() && y >= 0 && y < height()) {
    const unsigned char *const p = constImage() + 4 * (static_cast<size_t>(y) * width() + x);
    blue = p[0];
    green = p[1];
    red = p[2];
  }
  return QString::asprintf("pixel(%d,%d)=#%02X%02X%02X", x, y, red, green, blue);
}

void OmCamera::updateRecognizedObjectsOverlay(double screenX, double screenY, double overlayX, double overlayY) {
  if (!recognition() || !mSensor->isEnabled()) {
    clearRecognizedObjectsOverlay();
    return;
  }

  // check if mouse is over an object
  int objectIndex = -1;
  for (int i = 0; i < mRecognizedObjects.size(); ++i) {
    const OmVector2 positionOnImage = mRecognizedObjects.at(i)->positionOnImage();
    const OmVector2 pixelsSize = mRecognizedObjects.at(i)->pixelSize();
    if (overlayX < positionOnImage.x() - pixelsSize.x() * 0.5)
      continue;
    if (overlayX > positionOnImage.x() + pixelsSize.x() * 0.5)
      continue;
    if (overlayY < positionOnImage.y() - pixelsSize.y() * 0.5)
      continue;
    if (overlayY > positionOnImage.y() + pixelsSize.y() * 0.5)
      continue;
    objectIndex = i;
    break;
  }
  // display information of the object (if any) in an overlay
  if (objectIndex >= 0) {
    const OmRecognizedObject *object = mRecognizedObjects.at(objectIndex);
    QString text(object->model());
    text += tr("\nId: %1").arg(object->id());
    text += tr("\nRelative position: %1 %2 %3")
              .arg(OmPrecision::doubleToString(object->objectRelativePosition().x(), OmPrecision::GUI_LOW))
              .arg(OmPrecision::doubleToString(object->objectRelativePosition().y(), OmPrecision::GUI_LOW))
              .arg(OmPrecision::doubleToString(object->objectRelativePosition().z(), OmPrecision::GUI_LOW));
    text += tr("\nRelative orientation: %1 %2 %3 %4")
              .arg(OmPrecision::doubleToString(object->relativeOrientation().x(), OmPrecision::GUI_LOW))
              .arg(OmPrecision::doubleToString(object->relativeOrientation().y(), OmPrecision::GUI_LOW))
              .arg(OmPrecision::doubleToString(object->relativeOrientation().z(), OmPrecision::GUI_LOW))
              .arg(OmPrecision::doubleToString(object->relativeOrientation().angle(), OmPrecision::GUI_LOW));
    text += tr("\nSize: %1 %2")
              .arg(OmPrecision::doubleToString(object->objectSize().y(), OmPrecision::GUI_LOW))
              .arg(OmPrecision::doubleToString(object->objectSize().z(), OmPrecision::GUI_LOW));
    text += tr("\nPosition on the image: %1 %2").arg(object->positionOnImage().x()).arg(object->positionOnImage().y());
    text += tr("\nSize on the image: %1 %2").arg(object->pixelSize().x()).arg(object->pixelSize().y());
    for (int i = 0; i < object->colors().size(); ++i)
      text += tr("\nColor %1: %2 %3 %4")
                .arg(i)
                .arg(OmPrecision::doubleToString(object->colors().at(i).red(), OmPrecision::GUI_LOW))
                .arg(OmPrecision::doubleToString(object->colors().at(i).green(), OmPrecision::GUI_LOW))
                .arg(OmPrecision::doubleToString(object->colors().at(i).blue(), OmPrecision::GUI_LOW));
    if (mLabelOverlay == NULL) {
      mLabelOverlay = OmWrenLabelOverlay::createOrRetrieve(OmWrenLabelOverlay::cameraCaptionOverlayId(),
                                                           OmStandardPaths::fontsPath() + "LiberationSans-Regular.ttf");
      mLabelOverlay->setText(text);
      mLabelOverlay->setPosition(screenX, screenY);
      mLabelOverlay->setSize(0.06);
      mLabelOverlay->setColor(0xFF0000);
      mLabelOverlay->applyChangesToWren();
    } else {
      mLabelOverlay->moveToPosition(screenX, screenY);
      mLabelOverlay->updateText(text);
    }
  } else
    clearRecognizedObjectsOverlay();
}

void OmCamera::clearRecognizedObjectsOverlay() {
  if (mLabelOverlay) {
    OmWrenLabelOverlay::removeLabel(OmWrenLabelOverlay::cameraCaptionOverlayId());
    mLabelOverlay = NULL;
  }
}

void OmCamera::prePhysicsStep(double ms) {
  OmAbstractCamera::prePhysicsStep(ms);

  if (isPowerOn() && mRecognitionSensor->isEnabled() && mRecognitionSensor->needToRefreshInMs(ms)) {
    // delete them only when sensor will be updated (we need to keep them for the overlay)
    qDeleteAll(mRecognizedObjects);
    mRecognizedObjects.clear();
  } else if (mNeedToDeleteRecognizedObjectsRays) {
    // we can destroy the ray because we don't need it anymore
    foreach (OmRecognizedObject *recognizedObject, mRecognizedObjects)
      recognizedObject->deleteRays();
    mNeedToDeleteRecognizedObjectsRays = false;
  }

  if (isPowerOn() && mRecognitionSensor->isEnabled() && mRecognitionSensor->needToRefreshInMs(ms) && recognition() &&
      recognition()->occlusion() > 0) {
    // create the candidates (and their occlusion rays). They are re-aimed later,
    // in refreshRecognitionSensorIfNeeded(): ODE used to re-aim them mid-step
    // through OmSolidDevice's static dirty-sensor list (since removed: nothing drained it) and collide them in the same
    // pass, but the Newton raycast service answers AFTER the step, so there is
    // nothing to subscribe to any more.
    computeRecognizedObjects();
    mNeedToDeleteRecognizedObjectsRays = true;
  }
}

void OmCamera::postPhysicsStep() {
  OmAbstractCamera::postPhysicsStep();

  // delete the OmRecognizedObjects that dropped out of the frustum
  // it is preferable to not delete them during the physics step to avoid
  // possible inconsistencies in other clusters
  qDeleteAll(mInvalidRecognizedObjects);
  mInvalidRecognizedObjects.clear();

  if (!isControllerRunning() && recognition())
    // update recognized objects info displayed from the overlay in case occlusion is FALSE
    // (otherwise updated in the OmCamera::writeAnswer method)
    refreshRecognitionSensorIfNeeded();
}

void OmCamera::reset(const QString &id) {
  OmAbstractCamera::reset(id);

  OmNode *const focusNode = mFocus->value();
  if (focusNode)
    focusNode->reset(id);
  OmNode *const zoomNode = mZoom->value();
  if (zoomNode)
    zoomNode->reset(id);
  OmNode *const recognitionNode = mRecognition->value();
  if (recognitionNode)
    recognitionNode->reset(id);
  OmNode *const lensFlareNode = mLensFlare->value();
  if (lensFlareNode)
    lensFlareNode->reset(id);
}

// Aims every recognition candidate's occlusion rays at the object's CURRENT pose
// and re-runs the frustum + property pass, dropping the candidates that no longer
// qualify. ODE called this from the middle of the physics step (through
// OmSolidDevice's static dirty-sensor list (since removed: nothing drained it)) because its collision pass ran there;
// refreshRecognitionSensorIfNeeded() calls it directly now, immediately before
// the Newton raycast, so the rays that get cast match the poses the raycast
// service sees.
void OmCamera::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();

  // compute the camera frustum planes
  // D1.4: OmWrenCamera's FOV math is inlined above (cameraFieldOfViewY /
  // cameraSphericalFieldOfViewY) -- same formulas, no WREN camera.
  const double horizontalFieldOfView = fieldOfView();
  const double verticalFieldOfView =
    (isPlanarProjection() || horizontalFieldOfView > M_PI) ?
      cameraFieldOfViewY(horizontalFieldOfView, (double)width() / (double)height()) :
      cameraSphericalFieldOfViewY(horizontalFieldOfView, width(), height());
  const OmAffinePlane *frustumPlanes = OmObjectDetection::computeFrustumPlanes(this, verticalFieldOfView, horizontalFieldOfView,
                                                                               recognition()->maxRange(), isPlanarProjection());

  // update list of recognized objects
  foreach (OmRecognizedObject *recognizedObject, mRecognizedObjects) {
    recognizedObject->object()->updateTransformForPhysicsStep();
    if (!recognizedObject->recomputeRayDirection(frustumPlanes) || !setRecognizedObjectProperties(recognizedObject)) {
      mRecognizedObjects.removeAll(recognizedObject);
      mInvalidRecognizedObjects.append(recognizedObject);
    }
  }

  delete[] frustumPlanes;
}

void OmCamera::addConfigureToStream(OmDataStream &stream, bool reconfigure) {
  OmAbstractCamera::addConfigureToStream(stream, reconfigure);
  if (zoom()) {
    stream << (double)zoom()->minFieldOfView();
    stream << (double)zoom()->maxFieldOfView();
  } else {
    stream << (double)mFieldOfView->value();
    stream << (double)mFieldOfView->value();
  }
  stream << (unsigned char)(recognition() ? 1 : 0);
  stream << (unsigned char)(recognition() && recognition()->segmentation() ? 1 : 0);
  stream << (double)mExposure->value();
  if (focus()) {
    stream << (double)focus()->focalLength();
    stream << (double)focus()->focalDistance();
    stream << (double)focus()->minFocalDistance();
    stream << (double)focus()->maxFocalDistance();
  } else {
    stream << (double)0.0;
    stream << (double)0.0;
    stream << (double)0.0;
    stream << (double)0.0;
  }
}

void OmCamera::resetMemoryMappedFile() {
  OmAbstractCamera::resetMemoryMappedFile();
  if (hasBeenSetup() && (mSegmentationMemoryMappedFile || (recognition() && recognition()->segmentation())))
    // the previous memory mapped file will be released by the new controller start
    initializeSegmentationMemoryMappedFile();
}

void OmCamera::writeConfigure(OmDataStream &stream) {
  OmAbstractCamera::writeConfigure(stream);

  mRecognitionSensor->connectToRobotSignal(robot());
}

void OmCamera::writeAnswer(OmDataStream &stream) {
  OmAbstractCamera::writeAnswer(stream);

  if (mSegmentationImageChanged) {
    // D1.4 (WREN deletion): the segmentation image was rendered by a WREN camera and is
    // now produced NOWHERE -- degrade LOUDLY, not silently. The stream framing and the
    // memory-mapped file survive so the controller-side protocol is unbroken; the bytes
    // stay at their black-initialized value (the resized remote chunk region is left
    // untouched, exactly like the base's remote main-image path).
    if (mIsRemoteExternController) {
      editChunkMetadata(stream, size());

      // copy image to stream
      stream << (short unsigned int)tag();
      stream << (unsigned char)C_CAMERA_SERIAL_SEGMENTATION_IMAGE;
      int streamLength = stream.length();
      stream.resize(size() + streamLength);
      warnSegmentationImageEmpty();

      // prepare next chunk
      stream.mSizePtr = stream.length();
      stream << (int)0;
      stream << (unsigned char)0;
    } else
      warnSegmentationImageEmpty();
    mSegmentationImageChanged = false;
  }

  if (recognition()) {
    if (refreshRecognitionSensorIfNeeded() || mRecognitionSensor->hasPendingValue()) {
      stream << tag();
      stream << (unsigned char)C_CAMERA_OBJECTS;
      const int numberOfObjects = mRecognizedObjects.size();
      stream << (int)numberOfObjects;
      for (int i = 0; i < numberOfObjects; ++i) {
        const OmRecognizedObject *recognizedObject = mRecognizedObjects.at(i);
        // id
        stream << (int)recognizedObject->id();
        // relative position
        stream << (double)recognizedObject->objectRelativePosition().x();
        stream << (double)recognizedObject->objectRelativePosition().y();
        stream << (double)recognizedObject->objectRelativePosition().z();
        // relative orientation
        stream << (double)recognizedObject->relativeOrientation().x();
        stream << (double)recognizedObject->relativeOrientation().y();
        stream << (double)recognizedObject->relativeOrientation().z();
        stream << (double)recognizedObject->relativeOrientation().angle();
        // size
        stream << (double)recognizedObject->objectSize().x();
        stream << (double)recognizedObject->objectSize().y();
        // position on the image
        stream << (int)recognizedObject->positionOnImage().x();
        stream << (int)recognizedObject->positionOnImage().y();
        // size on the image
        stream << (int)recognizedObject->pixelSize().x();
        stream << (int)recognizedObject->pixelSize().y();
        // colors
        const int numberOfColors = recognizedObject->colors().size();
        stream << (int)numberOfColors;
        for (int j = 0; j < numberOfColors; ++j) {
          stream << (double)recognizedObject->colors().at(j).red();
          stream << (double)recognizedObject->colors().at(j).green();
          stream << (double)recognizedObject->colors().at(j).blue();
        }
        // model
        const QByteArray model = recognizedObject->model().toUtf8();
        stream.writeRawData(model.constData(), model.size() + 1);
      }

      mRecognitionSensor->resetPendingValue();
      // D1.4: the recognition-overlay frame drawing (boxes on the camera image through
      // WREN overlay textures) died with WREN; the recognition DATA above is unaffected.
    }

    if (mSegmentationChanged) {
      stream << (short unsigned int)tag();
      stream << (unsigned char)C_CAMERA_SET_SEGMENTATION;
      stream << (unsigned char)recognition()->segmentation();
      mSegmentationChanged = false;
    }

    if (recognition()->segmentation() && mHasSegmentationMemoryMappedFileChanged && !mIsRemoteExternController) {
      stream << (short unsigned int)tag();
      stream << (unsigned char)C_CAMERA_SEGMENTATION_MEMORY_MAPPED_FILE;
      if (mSegmentationMemoryMappedFile) {
        stream << (int)(mSegmentationMemoryMappedFile->size());
        const QByteArray n = QFile::encodeName(mSegmentationMemoryMappedFile->nativeKey());
        stream.writeRawData(n.constData(), n.size() + 1);
      } else
        stream << (int)(0);
      mHasSegmentationMemoryMappedFileChanged = false;
    }
  }
}

void OmCamera::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  if (OmAbstractCamera::handleCommand(stream, command))
    return;

  switch (command) {
    case C_CAMERA_SET_FOV: {
      double fov;
      stream >> fov;

      const OmZoom *z = zoom();
      if (z) {
        if (fov >= z->minFieldOfView() && fov <= z->maxFieldOfView())
          mFieldOfView->setValue(fov);
        else
          warn(
            tr("wb_camera_set_fov(%1) out of zoom range [%2, %3].").arg(fov).arg(z->minFieldOfView()).arg(z->maxFieldOfView()));
      } else
        warn(tr("wb_camera_set_fov() cannot be applied to this camera: missing 'zoom'."));
      break;
    }
    case C_CAMERA_SET_EXPOSURE: {
      double exposure;
      stream >> exposure;
      mExposure->setValue(exposure);
      break;
    }
    case C_CAMERA_SET_FOCAL: {
      double focalDistance;
      stream >> focalDistance;

      OmFocus *f = focus();
      if (f) {
        if ((focalDistance >= f->minFocalDistance()) && (focalDistance <= f->maxFocalDistance()))
          f->setFocalDistance(focalDistance);
        else
          warn(tr("wb_camera_set_focal_distance(%1) out of focus range [%2, %3].")
                 .arg(focalDistance)
                 .arg(f->minFocalDistance())
                 .arg(f->maxFocalDistance()));
      } else
        warn(tr("wb_camera_set_focal_distance() cannot be applied to this camera: missing 'focus'."));
      break;
    }
    case C_CAMERA_SET_RECOGNITION_SAMPLING_PERIOD: {
      stream >> mRecognitionRefreshRate;
      mRecognitionSensor->setRefreshRate(mRecognitionRefreshRate);
      break;
    }
    case C_CAMERA_ENABLE_SEGMENTATION:
      unsigned char segmentationEnabled;
      stream >> segmentationEnabled;
      mSegmentationEnabled = segmentationEnabled;
      // D1.4: the overlay mask-texture refresh died with WREN; the enable protocol is kept.
      if (!hasBeenSetup())
        setup();
      emit enabled(this, isEnabled());
      break;
    default:
      assert(0);
  }
}

void OmCamera::computeRecognizedObjects() {
  // compute the camera referential
  const OmVector3 cameraPosition = position();
  const double horizontalFieldOfView = fieldOfView();
  // D1.4: OmWrenCamera's FOV math is inlined above (cameraFieldOfViewY /
  // cameraSphericalFieldOfViewY) -- same formulas, no WREN camera.
  const double verticalFieldOfView =
    (isPlanarProjection() || horizontalFieldOfView > M_PI) ?
      cameraFieldOfViewY(horizontalFieldOfView, (double)width() / (double)height()) :
      cameraSphericalFieldOfViewY(horizontalFieldOfView, width(), height());
  const OmAffinePlane *frustumPlanes = OmObjectDetection::computeFrustumPlanes(this, verticalFieldOfView, horizontalFieldOfView,
                                                                               recognition()->maxRange(), isPlanarProjection());

  // loop for each possible target to check if it is visible
  const QList<OmSolid *> objects = OmWorld::instance()->cameraRecognitionObjects();
  for (int i = 0; i < objects.size(); i++) {
    OmSolid *object = objects.at(i);
    if (object == this || robot() == object || robot()->solidChildren().contains(object))
      continue;
    // We should discard targets as soon as possible to improve performance.
    if ((cameraPosition - object->position() - object->boundingSphere()->center()).length() >
        (recognition()->maxRange() + object->boundingSphere()->scaledRadius()))
      continue;
    // create target
    OmRecognizedObject *generatedObject =
      new OmRecognizedObject(this, object, recognition()->occlusion(), recognition()->maxRange());
    if (!generatedObject->isContainedInFrustum(frustumPlanes) || !setRecognizedObjectProperties(generatedObject)) {
      delete generatedObject;
      continue;
    }
    mRecognizedObjects.append(generatedObject);
  }

  delete[] frustumPlanes;
}

OmVector2 OmCamera::applyCameraDistortionToImageCoordinate(const OmVector2 &uv) {
  if (!lens())
    return uv;
  OmVector2 distortedUv(uv);
  const OmVector2 &rc = lens()->radialCoefficients();
  const OmVector2 &tc = lens()->tangentialCoefficients();
  const bool hasRadialDistortion = rc.x() != 0 || rc.y() != 0;
  const bool hasTangentialDistortion = tc.x() != 0 || tc.y() != 0;
  const OmVector2 relativeUv = uv - lens()->center();
  const float r2 = relativeUv.length2();
  if (hasRadialDistortion)
    distortedUv += (rc.x() * r2 + rc.y() * r2 * r2) * relativeUv;
  if (hasTangentialDistortion) {
    distortedUv +=
      OmVector2(2 * tc.x() * relativeUv.x() * relativeUv.y() + tc.y() * (r2 + 2 * relativeUv.x() * relativeUv.x()),
                tc.x() * (r2 + 2 * relativeUv.y() * relativeUv.y() + 2 * tc.y() * relativeUv.x() * relativeUv.y()));
  }
  distortedUv.setX(qMax(0.0, qMin(distortedUv.x(), 1.0)));
  distortedUv.setY(qMax(0.0, qMin(distortedUv.y(), 1.0)));
  return distortedUv;
}

OmVector2 OmCamera::projectOnImage(const OmVector3 &position) {
  const double fovX = fieldOfView();
  OmVector2 uv;  // uv coordinates in range [-0.5, 0.5]
  if (mProjection->value() == "planar") {
    if (position.x() < 0)
      uv.setXy(position.y() > 0 ? -0.5 : 0.5, position.z() > 0 ? -0.5 : 0.5);
    else {
      const double fovY = cameraFieldOfViewY(fovX, (double)width() / (double)height());
      const double theta1 = -atan2(position.y(), fabs(position.x()));
      const double theta2 = atan2(position.z(), fabs(position.x()));
      uv.setX(0.5 * tan(theta1) / tan(0.5 * fovX));
      uv.setY(-0.5 * tan(theta2) / tan(0.5 * fovY));
    }
  } else if (mProjection->value() == "spherical") {
    const OmVector3 p = position.normalized();
    const double b = p.z() / p.y();
    uv.setY(acos(p.x()) / (fovX * sqrt(1 + 1 / (b * b))));
    if (p.z() > 0.0)
      uv.setY(-uv.y());
    uv.setX(uv.y() / b);
    if (fovX < M_PI_2)
      uv.setX(uv.x() * M_PI_2 / fovX);
  } else {
    assert(mProjection->value() == "cylindrical");
    // D1.4: same spherical-FOV values the WREN camera computed, inlined (see the helpers
    // at the top of this file).
    const double fovY = cameraSphericalFieldOfViewY(fovX, width(), height());
    const double fovYCorrectionCoefficient = cameraSphericalFovYCorrectionCoefficient(fovX, width(), height());
    const OmVector3 normP = position.normalized();
    const double theta = acos(normP.z());
    uv.setX(-acos(normP.x() / sin(theta)) / fovX);
    uv.setY((theta - M_PI_2) * fovYCorrectionCoefficient / fovY);
    if (normP.y() < 0.0)
      uv.setX(-uv.x());
    if (fovX < M_PI_2)
      uv.setX(uv.x() * M_PI_2 / fovX);
  }

  // convert uv to range [0, 1]
  uv.setX(qMax(0.0, qMin(0.5 + uv.x(), 1.0)));
  uv.setY(qMax(0.0, qMin(0.5 + uv.y(), 1.0)));
  uv = applyCameraDistortionToImageCoordinate(uv);

  // return uv coordinates in range [0, width/height]
  return OmVector2((int)((width() - 1) * uv.x()), (int)((height() - 1) * uv.y()));
}

bool OmCamera::setRecognizedObjectProperties(OmRecognizedObject *recognizedObject) {
  assert(recognizedObject);

  // compute object relative orientation
  OmRotation relativeRotation;
  relativeRotation.fromMatrix3(recognizedObject->device()->rotationMatrix().transposed() *
                               recognizedObject->object()->rotationMatrix());
  relativeRotation.normalize();
  recognizedObject->setRelativeOrientation(relativeRotation);

  // set the objects colors
  recognizedObject->clearColors();
  for (int j = 0; j < recognizedObject->object()->recognitionColorSize(); ++j)
    recognizedObject->addColor(recognizedObject->object()->recognitionColor(j));

  // set the object model
  recognizedObject->setModel(recognizedObject->object()->model());

  // compute position and size in the camera image
  double minU = width();
  double minV = height();
  double maxU = 0;
  double maxV = 0;
  QList<OmVector3> corners = recognizedObject->computeCorners();
  if (!isPlanarProjection())
    corners << OmVector3(0, 0, 0);  // add center to better localize deformed objects
  QListIterator<OmVector3> cornerIt(corners);
  while (cornerIt.hasNext()) {  // project each of the corners in the camera image
    const OmVector2 &positionOnImage = projectOnImage(recognizedObject->objectRelativePosition() + cornerIt.next());
    if (positionOnImage.x() < minU)
      minU = positionOnImage.x();
    if (positionOnImage.y() < minV)
      minV = positionOnImage.y();
    if (positionOnImage.x() > maxU)
      maxU = positionOnImage.x();
    if (positionOnImage.y() > maxV)
      maxV = positionOnImage.y();
  }
  recognizedObject->setPositionOnImage(OmVector2(round((maxU + minU) / 2.0), round((maxV + minV) / 2.0)));
  recognizedObject->setPixelSize(OmVector2(ceil(maxU - minU), ceil(maxV - minV)));
  if (recognizedObject->pixelSize().x() < 1 || recognizedObject->pixelSize().y() < 1)
    return false;

  return true;
}

bool OmCamera::refreshRecognizedObjectsOcclusionFromNewton() {
  // Kernel blocker #1 (ode-retirement-campaign.md): answer every recognition
  // candidate's occlusion rays from the Newton raycast service (mj_ray over the
  // live mjModel). This is the ONLY ray provider left, so a gate-off or an
  // unavailable runtime means the occlusion question is UNANSWERED -- returning
  // false makes the caller keep its previous verdicts instead of treating
  // "no answer" as "clear line of sight".
  if (mRecognizedObjects.isEmpty())
    return true;  // nothing to test
  if (!isNewtonRaycastEnabled()) {
    warnRecognitionOcclusionUnavailableOnce("OMNISIM_NEWTON_RAYCAST is disabled");
    return false;
  }
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable()) {
    warnRecognitionOcclusionUnavailableOnce("the Newton runtime is not available");
    return false;
  }
  const OmNewtonBackend *const newton = static_cast<const OmNewtonBackend *>(raw);

  // Own-robot exclusion, replicating odeNearCallback's same-robot rule
  // (waived by selfCollision); cached once like OmDistanceSensor.
  if (mNewtonExcludeBodies.isEmpty()) {
    const OmRobot *const r = robot();
    if (r != nullptr && !r->selfCollision())
      collectRobotNewtonBodies(r, mNewtonExcludeBodies);
    if (mNewtonExcludeBodies.isEmpty())
      mNewtonExcludeBodies.append(-999);  // sentinel: "built, nothing to exclude"
  }
  const QVector<int> deviceExclude = (mNewtonExcludeBodies.size() == 1 && mNewtonExcludeBodies[0] == -999)
                                       ? QVector<int>()
                                       : mNewtonExcludeBodies;
  // one raycastBatch per candidate: the exclude list carries the candidate's
  // own bodies, so candidates cannot share a batch
  bool answered = true;
  foreach (OmRecognizedObject *recognizedObject, mRecognizedObjects) {
    if (!recognizedObject->refreshCollisionDepthsFromNewton(newton, deviceExclude))
      answered = false;
  }
  if (!answered)
    warnRecognitionOcclusionUnavailableOnce("the raycast service returned no answer");
  return answered;
}

bool OmCamera::refreshRecognitionSensorIfNeeded() {
  assert(recognition());

  if (!isPowerOn() || !mRecognitionSensor->needToRefresh())
    return false;

  if (recognition()->occlusion() == 0)
    // no ray casting needed: objects can be created here, at the end of the
    // step, when all the body positions are up-to-date
    computeRecognizedObjects();
  else {
    // The candidates were created in prePhysicsStep(); re-aim their rays at the
    // post-step poses and recompute their image properties -- ODE did this
    // mid-step.
    updateRaysSetupIfNeeded();
    // Then cast the rays through the Newton service and drop what it says is
    // blocked. If the service could not answer, the object list is reported
    // unfiltered rather than being silently declared fully visible: the
    // one-shot warning above names the reason.
    if (refreshRecognizedObjectsOcclusionFromNewton())
      removeOccludedRecognizedObjects();
  }

  // check the number of objects and keep only the biggest (in pixel size)
  if (recognition()->maxObjects() > 0 && mRecognizedObjects.size() > recognition()->maxObjects()) {
    QMultiMap<int, OmRecognizedObject *> objectsMap;
    for (int i = 0; i < mRecognizedObjects.size(); ++i) {
      const int pixelSurface = mRecognizedObjects.at(i)->pixelSize().x() * mRecognizedObjects.at(i)->pixelSize().y();
      objectsMap.insert(pixelSurface, mRecognizedObjects.at(i));
    }
    mRecognizedObjects.clear();
    QMultiMapIterator<int, OmRecognizedObject *> i(objectsMap);
    i.toBack();
    while (i.hasPrevious() && mRecognizedObjects.size() < recognition()->maxObjects()) {
      i.previous();
      mRecognizedObjects.append(i.value());
    }
    // delete the remaining ones
    while (i.hasPrevious()) {
      i.previous();
      delete i.value();
    }
  }

  mRecognitionSensor->updateTimer();
  return true;
}

void OmCamera::removeOccludedRecognizedObjects() {
  for (int i = mRecognizedObjects.size() - 1; i >= 0; --i) {
    if (mRecognizedObjects.at(i)->hasCollided()) {
      delete mRecognizedObjects[i];
      mRecognizedObjects.removeAt(i);
    }
  }
}

// D1.4 (WREN deletion): once per device -- the segmentation IMAGE was a WREN render and is
// produced nowhere any more. The protocol (enable/disable, memory-mapped file, stream
// chunks) survives; the bytes stay black. Loud, so a controller reading an empty
// segmentation image finds the reason in the log instead of debugging its own code.
void OmCamera::warnSegmentationImageEmpty() {
  if (mSegmentationWrenWarned)
    return;
  mSegmentationWrenWarned = true;
  warn(tr("'%1': segmentation rendered through WREN, which is deleted; the segmentation image is empty. The recognition "
          "object list (wb_camera_recognition_get_objects) is unaffected -- it is computed from Newton rays, not from the "
          "segmentation render.")
         .arg(deviceName()));
}

void OmCamera::setup() {
  OmAbstractCamera::setup();
  createSegmentationCamera();
  if (mSegmentationMemoryMappedFile || (recognition() && recognition()->segmentation()))
    initializeSegmentationMemoryMappedFile();

  if (!isPlanarProjection())
    return;

  updateNoiseMaskUrl();
  updateExposure();
  updateAmbientOcclusionRadius();
  updateBloomThreshold();
  connect(mNoiseMaskUrl, &OmSFString::changed, this, &OmCamera::updateNoiseMaskUrl);

  // make sure bounding sphere is updated when object's size and position changes
  if (recognition()) {
    mIsSubscribedToRayTracing = true;
    OmSimulationState::instance()->subscribeToRayTracing();
  }
}

bool OmCamera::isEnabled() const {
  return (mSegmentationEnabled && recognition() && recognition()->segmentation()) || OmAbstractCamera::isEnabled();
}

bool OmCamera::needToRender() const {
  return OmAbstractCamera::needToRender() ||
         (mSegmentationEnabled && mRecognitionSensor->isEnabled() && mRecognitionSensor->needToRefresh());
}

void OmCamera::render() {
  // D1.4: the main image is rendered by copyImageToMemoryMappedFile (wgpu) at answer time.
  // The WREN segmentation render is gone; keep the protocol cadence (the flag drives the
  // stream chunk + the once-per-device empty-image warning in writeAnswer).
  if (recognition() && recognition()->segmentation() && mSegmentationEnabled && isPowerOn() &&
      mRecognitionSensor->isEnabled() && mRecognitionSensor->needToRefresh())
    mSegmentationImageChanged = true;
}

OmVector3 OmCamera::urdfRotation(const OmMatrix3 &rotationMatrix) const {
  OmVector3 eulerRotation = rotationMatrix.toEulerAnglesZYX();
  // Webots defines the camera frame as FLU but ROS desfines it as RDF (Right-Down-Forward)
  eulerRotation[0] -= M_PI / 2;
  eulerRotation[2] -= M_PI / 2;
  return eulerRotation;
}

void OmCamera::exportNodeFields(OmWriter &writer) const {
  OmAbstractCamera::exportNodeFields(writer);

  exportSFResourceField("noiseMaskUrl", mNoiseMaskUrl, writer.relativeMeshesPath(), writer);
}

QStringList OmCamera::customExportedFields() const {
  QStringList fields;
  fields << "noiseMaskUrl";
  return fields;
}

/////////////////////
//  Update methods //
/////////////////////

void OmCamera::updateFocus() {
  if (focus())
    connect(focus(), &OmFocus::focusSettingsChanged, this, &OmCamera::applyFocalSettingsToWren);
  else if (hasBeenSetup())
    mNeedToConfigure = true;
  if (hasBeenSetup())
    applyFocalSettingsToWren();
}

void OmCamera::updateRecognition() {
  if (!hasBeenSetup())
    return;

  const OmRecognition *recognitionNode = recognition();
  // D1.4: the recognition-overlay foreground texture died with WREN (the pop-out window
  // signal contract keeps textureIdUpdated, but there is no id to send any more).

  // make sure bounding sphere is updated when object's size and position changes
  const bool recognitionEnabled = recognitionNode != NULL;
  if (recognitionEnabled != mIsSubscribedToRayTracing) {
    mIsSubscribedToRayTracing = recognitionEnabled;
    if (recognitionNode)
      OmSimulationState::instance()->subscribeToRayTracing();
    else
      OmSimulationState::instance()->unsubscribeToRayTracing();
  }

  // clear mRecognizedObjects if Recognition node changed but not if `Recognition.segmentation` changed
  qDeleteAll(mRecognizedObjects);
  mRecognizedObjects.clear();

  createSegmentationCamera();

  mNeedToConfigure = true;
}

void OmCamera::updateSegmentation() {
  mSegmentationChanged = true;
  if (mSegmentationEnabled && (!recognition() || !recognition()->segmentation()))
    mSegmentationEnabled = false;
  createSegmentationCamera();
}

// D1.4 (WREN deletion): the OmWrenCamera this used to create is gone. The name is kept for
// its callers; what remains is the segmentation PROTOCOL wiring -- the segmentationChanged
// connection and the memory-mapped file -- so the controller API keeps its framing while
// the image itself stays empty (warned once in writeAnswer).
void OmCamera::createSegmentationCamera() {
  const OmRecognition *recognitionNode = recognition();
  if (recognitionNode)
    connect(recognitionNode, &OmRecognition::segmentationChanged, this, &OmCamera::updateSegmentation, Qt::UniqueConnection);

  if (recognitionNode && recognitionNode->segmentation()) {
    if (!mSegmentationMemoryMappedFile)
      initializeSegmentationMemoryMappedFile();
  }
}

void OmCamera::updateNear() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.01))
    return;

  mNeedToConfigure = true;

  if (mFar->value() > 0.0 and mFar->value() < mNear->value()) {
    mNear->setValue(mFar->value());
    parsingWarn(tr("'near' is greater than 'far'. Setting 'near' to %1.").arg(mNear->value()));
  }
  // D1.4: the wgpu render reads mNear live; the WREN frustum visuals are gone.
}

void OmCamera::updateFar() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mFar, 0.0))
    return;

  if (mFar->value() > 0.0 and mFar->value() < mNear->value()) {
    mFar->setValue(mNear->value() + 1.0);
    parsingWarn(tr("'far' is less than 'near'. Setting 'far' to %1.").arg(mFar->value()));
    return;
  }
  // D1.4: the wgpu render reads mFar live (sensorFarPlane substitution included).
}

void OmCamera::updateExposure() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mExposure, 1.0))
    return;
  // D1.4: the wgpu scene render reads mExposure live per frame.
}

void OmCamera::updateAmbientOcclusionRadius() {
  OmFieldChecker::resetDoubleIfNegative(this, mAmbientOcclusionRadius, 2.0);
  // D1.4: the WREN GTAO consumer is gone (the wgpu sensor render keeps SSAO off on
  // purpose -- a sensor measures the scene, not a stylized image); the field check
  // survives.
}

void OmCamera::updateBloomThreshold() {
  OmFieldChecker::resetDoubleIfNegativeAndNotDisabled(this, mBloomThreshold, 21.0, -1.0);
  // D1.4: the WREN bloom consumer is gone (bloom is a lens artefact the wgpu sensor
  // render deliberately keeps at 0); the field check survives.
}

void OmCamera::updateNoiseMaskUrl() {
  if (!hasBeenSetup() || mNoiseMaskUrl->value().isEmpty())
    return;

  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  QString url = mNoiseMaskUrl->value();
  mNoiseMaskUrl->blockSignals(true);
  mNoiseMaskUrl->setValue(url.replace("\\", "/"));
  mNoiseMaskUrl->blockSignals(false);

  const QString &completeUrl = OmUrl::computePath(this, "noiseMaskUrl", mNoiseMaskUrl->value(), true);
  if (OmUrl::isWeb(completeUrl)) {
    if (mDownloader && !mDownloader->error().isEmpty()) {
      warn(mDownloader->error());  // failure downloading or file does not exist (404)
      delete mDownloader;
      mDownloader = NULL;
      return;
    }

    if (!OmNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
      downloadAssets();  // URL was changed from the scene tree or supervisor
      return;
    }
  }

  if (!(completeUrl == OmUrl::missingTexture() || completeUrl.isEmpty())) {
    // D1.4: the WREN noise-mask application is gone -- noiseMaskUrl is a RETIRED effect on
    // wgpu (setup() warns it by name). The URL normalization + download plumbing above is
    // kept so the field's asset behaviour (and the download error report) is unchanged.
  }
}

bool OmCamera::isFrustumEnabled() const {
  return OmWrenRenderingContext::instance()->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_CAMERA_FRUSTUMS);
}

/////////////////////
//  Apply methods  //
/////////////////////

// D1.4: WREN's depth-of-field blur is gone (retired on wgpu), but this slot stays wired to
// OmFocus::focusSettingsChanged -- the controller-facing focus API
// (wb_camera_set_focal_distance and friends) still needs the reconfigure it triggers.
void OmCamera::applyFocalSettingsToWren() {
  if (hasBeenSetup())
    mNeedToConfigure = true;
}
