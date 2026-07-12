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

#include "WbCamera.hpp"

#include "WbAffinePlane.hpp"
#include "WbBasicJoint.hpp"
#include "WbBoundingSphere.hpp"
#include "WbBox.hpp"
#include "WbCapsule.hpp"
#include "WbCylinder.hpp"
#include "WbPlane.hpp"
#include "WbSphere.hpp"
#include "WbDataStream.hpp"
#include "WbDownloadManager.hpp"
#include "WbDownloader.hpp"
#include "WbFieldChecker.hpp"
#include "WbFocus.hpp"
#include "WbLens.hpp"
#include "WbLensFlare.hpp"
#include "WbLight.hpp"
#include "WbNetwork.hpp"
#include "WbObjectDetection.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPreferences.hpp"
#include "WbProtoModel.hpp"
#include "WbRecognition.hpp"
#include "WbLog.hpp"
#include "WbMatrix4.hpp"
#include "WbPbrAppearance.hpp"
#include "WbRenderBackend.hpp"
#include "WbRgb.hpp"
#include "WbShape.hpp"
#include "WbVulkanBackend.hpp"
#include "WbWgpuMeshAdapter.hpp"
#include "WbWgpuMeshCache.hpp"
#include "WbWgpuRenderTarget.hpp"
#include "WbWgpuSceneRenderer.hpp"
// R3.7/δ Newton-interop: snapshot live body translations into the wgpu
// instanced storage-buffer path (runtime-gated by OMNISIM_PROBE_DELTA; the
// newton dynamic_cast no-ops on ODE/NEWTON-OFF builds).
#include "WbNewtonBackend.hpp"
#include "WbPhysicsBackend.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"

#include <array>
#include <cmath>
#include <vector>
#include "WbRgb.hpp"
#include "WbRobot.hpp"
#include "WbSFNode.hpp"
#include "WbSensor.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"
#include "WbWorld.hpp"
#include "WbWrenCamera.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenTextureOverlay.hpp"
#include "WbZoom.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QtGlobal>

#ifndef _WIN32
#include "WbPosixMemoryMappedFile.hpp"
#else
#include <QtCore/QSharedMemory>
#endif

class WbRecognizedObject : public WbObjectDetection {
public:
  WbRecognizedObject(WbCamera *camera, WbSolid *object, const int occlusion, const double maxRange) :
    WbObjectDetection(camera, object, occlusion, maxRange, camera->fieldOfView()) {
    mId = object->uniqueId();
    mModel = "";
    mRelativeOrientation = WbRotation(0.0, 1.0, 0.0, 0.0);
    mPositionOnImage = WbVector2(0, 0);
    mPixelSize = WbVector2(0, 0);
    mColors.clear();
  };

  virtual ~WbRecognizedObject() {}

  int id() const { return mId; }
  const QString &model() const { return mModel; }
  const WbRotation &relativeOrientation() const { return mRelativeOrientation; }
  const WbVector2 positionOnImage() const { return mPositionOnImage; }
  const WbVector2 pixelSize() const { return mPixelSize; }
  const QList<WbRgb> &colors() const { return mColors; }

  void setModel(const QString &model) { mModel = model; }
  void setRelativeOrientation(const WbRotation &relativeOrientation) { mRelativeOrientation = relativeOrientation; }
  void setPositionOnImage(const WbVector2 &positionOnImage) { mPositionOnImage = positionOnImage; }
  void setPixelSize(const WbVector2 &pixelSize) { mPixelSize = pixelSize; }
  void addColor(const WbRgb &colors) { mColors.append(colors); }
  void clearColors() { mColors.clear(); }

protected:
  double distance() override { return fabs(objectRelativePosition().x()); }

  int mId;
  QString mModel;
  WbRotation mRelativeOrientation;
  WbVector2 mPositionOnImage;
  WbVector2 mPixelSize;
  QList<WbRgb> mColors;
};

void WbCamera::init() {
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

  // backward compatibility
  WbSFBool *sphericalField = findSFBool("spherical");
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
  mRecognizedObjectsTexture = NULL;
  mIsSubscribedToRayTracing = false;
  mSegmentationChanged = false;
  mSegmentationCamera = NULL;
  mSegmentationEnabled = false;
  mSegmentationMemoryMappedFile = NULL;
  mSegmentationImageChanged = false;
  mHasSegmentationMemoryMappedFileChanged = false;
  mInvalidRecognizedObjects = QList<WbRecognizedObject *>();
  mDownloader = NULL;
}

WbCamera::WbCamera(WbTokenizer *tokenizer) : WbAbstractCamera("Camera", tokenizer) {
  init();
}

WbCamera::WbCamera(const WbCamera &other) : WbAbstractCamera(other) {
  init();
}

WbCamera::WbCamera(const WbNode &other) : WbAbstractCamera(other) {
  init();
}

WbCamera::~WbCamera() {
  delete mRecognitionSensor;
  qDeleteAll(mRecognizedObjects);
  mRecognizedObjects.clear();

  delete mSegmentationCamera;
  delete mSegmentationMemoryMappedFile;

  // R3.3b + R3.4-step-4: wgpu render target + mesh cache. Order
  // matters: mesh cache references the same WGPUDevice as the
  // target, so destroy cache (which releases its WGPUBuffers)
  // before the target (whose dtor doesn't release the device
  // itself, but does free the depth+color textures that share the
  // device's allocator).
  delete mWgpuMeshCache;
  mWgpuMeshCache = NULL;
  delete mWgpuTarget;
  mWgpuTarget = NULL;

  if (mIsSubscribedToRayTracing)
    WbSimulationState::instance()->unsubscribeToRayTracing();
}

void WbCamera::downloadAssets() {
  WbAbstractCamera::downloadAssets();

  const QString &noiseMaskUrl = mNoiseMaskUrl->value();
  if (!noiseMaskUrl.isEmpty()) {  // noise mask not mandatory, URL can be empty
    const QString completeUrl = WbUrl::computePath(this, "noiseMaskUrl", noiseMaskUrl);
    if (!WbUrl::isWeb(completeUrl) || WbNetwork::instance()->isCachedWithMapUpdate(completeUrl))
      return;

    delete mDownloader;
    mDownloader = WbDownloadManager::instance()->createDownloader(QUrl(completeUrl), this);
    if (!WbWorld::instance()->isLoading())  // URL changed from the scene tree or supervisor
      connect(mDownloader, &WbDownloader::complete, this, &WbCamera::updateNoiseMaskUrl);
    mDownloader->download();
  }
}

// R2 of engine-migration-plan.md: parser-only renderBackend accessors.
// No call site consults these yet; they exist so R3+ can route this
// Camera's render-to-texture through WbRenderBackendRegistry::resolve.
// Mirrors WbSolid::physicsBackendName / physicsBackend exactly.
const QString &WbCamera::renderBackendName() const {
  static const QString kWren = QStringLiteral("wren");
  if (!mRenderBackend)
    return kWren;
  return mRenderBackend->value();
}

WbRenderBackend *WbCamera::renderBackend() const {
  QString nameStr = renderBackendName();
  // World-level default (default-flip-plan.md §3.2): an empty or "auto" renderBackend defers to
  // WorldInfo.defaultRenderBackend; an explicit per-node value is used as-is. Inert today (node
  // default is the concrete "wren").
  if (nameStr.isEmpty() || nameStr == QStringLiteral("auto")) {
    const WbWorldInfo *const wi = WbWorld::instance() ? WbWorld::instance()->worldInfo() : nullptr;
    if (wi != nullptr && !wi->defaultRenderBackend().isEmpty())
      nameStr = wi->defaultRenderBackend();
  }
  const QByteArray name = nameStr.toUtf8();
  return WbRenderBackendRegistry::resolve(WbRenderBackendKindFromString(name.constData()));
}

// R3.3b: lazy-build / resize the wgpu render target.
//
// Allocates a WbWgpuRenderTarget at this camera's current
// width()/height() the first time the wgpu path is taken. If the
// camera is later resized, the old target is destroyed and a new
// one allocated. Returns true iff a usable target now lives in
// `mWgpuTarget`. False means: not a wgpu-backed camera, OR wgpu
// init failed at startup, OR target construction failed (OOM,
// device lost). Callers must fall through to WREN when this
// returns false.
bool WbCamera::ensureWgpuTarget() {
  // R5c: delegate to the shared sensor scene-renderer. Behaviorally identical
  // to the former inline body (same Vulkan-kind/availability + size + resize
  // checks); keeps Camera + RangeFinder on a single render code path.
  return WbWgpuSceneRenderer::ensureTarget(renderBackend(), width(), height(),
                                           mWgpuTarget, mWgpuMeshCache, mWgpuTargetWidth,
                                           mWgpuTargetHeight);
}

void WbCamera::copyImageToMemoryMappedFile(WbWrenCamera *camera, unsigned char *data) {
  // R3.3b: route the MAIN camera image through wgpu when the camera
  // opted into `renderBackend "wgpu"` AND we have a working backend.
  // The segmentation camera (a separate WbWrenCamera owned by this
  // node) always stays on WREN — segmentation has no wgpu equivalent
  // yet, and the kind() check on this camera's renderBackend would
  // incorrectly redirect segmentation traffic too if we didn't gate
  // on the camera pointer here.
  if (camera != mSegmentationCamera && data && ensureWgpuTarget()) {
    // R3.4-step-4 production scene pass.
    //
    // Walk the world's Solids, harvest one draw call per visible
    // Shape (geometry's WrStaticMesh translated through
    // WbWgpuMeshAdapter, transform from WbGeometry::matrix(),
    // baseColor from the Shape's PBRAppearance), build a viewProj
    // from this Camera's pose + FOV + near/far + aspect, fire
    // clearAndDrawScene.
    //
    // Things this layer DOES NOT do yet (Phase ζ / R3.4-step-5+):
    //   - Specular / metallic / roughness — the WGSL is Lambertian
    //     only with the PBRAppearance's baseColor as albedo.
    //   - Textures — baseColorMap, normalMap, occlusionMap all
    //     ignored. R3.5b's adapter is wired up; R3.4-step-5 starts
    //     binding them.
    //   - Frustum culling — every Shape produces a draw regardless
    //     of viewport position. R3.4-step-6 wires Wb-side culling.
    //   - Cylinder / Sphere / IndexedFaceSet / capsule / IndexedLineSet
    //     etc.: anything whose WbGeometry::wrenMesh() is null gets
    //     skipped. Most primitives DO populate wrenMesh(), so the
    //     coverage on common worlds is reasonable.
    //   - Background / Sky / lights other than a single hard-coded
    //     directional light pointing down (0, 0, -1) with 25%
    //     ambient.
    //
    // The clear color comes from the Viewpoint's background — for
    // now hardcoded to a mid-gray (0.18) so unrendered pixels are
    // visibly distinct from baseColor-rendered pixels.
    std::vector<WbWgpuSolidDraw> draws;
    std::vector<std::array<float, 16>> modelStorage;
    modelStorage.reserve(64);  // grow without invalidating pointers in `draws`
    draws.reserve(64);
    // R5c: scene walk via the shared sensor scene-renderer (same coverage as
    // the former inline collectShapeDraws — Box/Plane/Sphere/Cylinder/Capsule
    // primitives + WREN-readback fallback).
    WbWgpuSceneRenderer::collectWorldDraws(*mWgpuMeshCache, draws, modelStorage);

    // viewProj from this Camera's pose + FOV + near/far + aspect, via the
    // shared scene-renderer (encodes the Webots +X-fwd/+Z-up → wgpu -Z-fwd/
    // +Y-up basis swap; identical math to the former inline block).
    const double aspect =
      mWgpuTargetHeight > 0 ? static_cast<double>(mWgpuTargetWidth) / mWgpuTargetHeight : 1.0;
    float viewProj[16] = {0};
    WbWgpuSceneRenderer::buildViewProj(matrix(), fieldOfView(), aspect, mNear->value(),
                                       mFar->value(), viewProj);

    const float lightDirAmbient[4] = {0.3f, 0.4f, -0.85f, 0.25f};
    WbWgpuClearColor clear = {0.18f, 0.18f, 0.18f, 1.0f};
    // R3.4-step-4 diagnostic: OMNISIM_R34_IDENTITY_VP=1 bypasses
    // the viewProj math (mesh local coords land directly in NDC).
    // A unit-box at the origin fills most of the screen; useful for
    // isolating viewProj bugs from pipeline bugs.
    float vpIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const float *vpToUse =
      qEnvironmentVariableIntValue("OMNISIM_R34_IDENTITY_VP") ? vpIdentity : viewProj;

    // R3.7 / Phase δ — Newton → wgpu interop demo (the marquee: physics-GPU
    // body positions feed the render-GPU vertex shader). When OMNISIM_PROBE_DELTA
    // is set, draw the LIVE Newton body translations as instances through the
    // storage-buffer instanced path (R3.7) using THIS camera's viewProj, instead
    // of the per-Solid scene walk. Opt-in; default render path unchanged. The
    // newton dynamic_cast returns null on ODE / NEWTON-OFF builds (→ skip).
    if (qEnvironmentVariableIntValue("OMNISIM_PROBE_DELTA") >= 1) {
      WbNewtonBackend *nb =
          dynamic_cast<WbNewtonBackend *>(WbPhysicsBackendRegistry::newtonBackend());
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
        WbWgpuMeshHandle dm = mWgpuMeshCache->acquire(
            0xDE17Aull, kDeltaVerts, sizeof(kDeltaVerts), kDeltaIdx, sizeof(kDeltaIdx), 3, 32);
        if (dm.vertexBuffer &&
            mWgpuTarget->clearAndDrawInstanced(clear, vpToUse, sBodyBuf,
                                               static_cast<uint32_t>(nbod), dm.vertexBuffer,
                                               dm.indexBuffer, dm.indexCount, data)) {
          static thread_local int sDeltaLog = 0;
          if (sDeltaLog < 8) {
            WbLog::info(QString("[WbCamera δ] %1 LIVE Newton bodies → wgpu instanced "
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
        WbLog::info(QString("[WbCamera] '%1' [T1.2 CSM shadow-pipeline probe: %2]")
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
      const WbMatrix4 lightWorld(1.0, 0.0, 5.0, 0.0, 1.0, 0.0, M_PI_2);
      float lightVP[16] = {0};
      WbWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, 2.0, 0.05, 10.0, lightVP);
      const float strength =
        qEnvironmentVariableIntValue("OMNISIM_CAMERA_SHADOW") >= 2 ? 0.6f : 0.0f;
      if (mWgpuTarget->clearAndDrawSceneShadowed(clear, viewProj, lightVP, lightDirAmbient,
                                                 draws.empty() ? nullptr : draws.data(),
                                                 static_cast<uint32_t>(draws.size()),
                                                 strength, 0.002f, data)) {
        static thread_local int sShadowLog = 0;
        if (sShadowLog < 30) {
          const size_t cx = static_cast<size_t>(mWgpuTargetWidth) / 2;
          const size_t cy = static_cast<size_t>(mWgpuTargetHeight) / 2;
          const size_t ci = (cy * static_cast<size_t>(mWgpuTargetWidth) + cx) * 4;
          WbLog::info(QString("[WbCamera] '%1' [T1.2 CSM two-pass shadow strength %2: "
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
            WbLog::info(QString("[WbCamera] '%1' [T1.2 CSM 3d grid G strength %2 frame %4:%3]")
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
      const WbMatrix4 lightWorld(1.0, 0.0, 5.0, 0.0, 1.0, 0.0, M_PI_2);
      float lightVP[16] = {0};
      WbWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, 1.0, 0.05, 10.0, lightVP);
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
          WbLog::info(QString("[WbCamera] '%1' rendered through wgpu (%2x%3, %4 draws) "
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
    const int depthLevel = qEnvironmentVariableIntValue("OMNISIM_CAMERA_DEPTH");
    if (depthLevel >= 3) {
      const float farVal = static_cast<float>(mFar->value());
      float viewMat[16] = {0};
      WbWgpuSceneRenderer::buildView(matrix(), viewMat);
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
          WbLog::info(QString("[WbCamera] '%1' rendered through wgpu (%2x%3, %4 draws) "
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
      const float farVal = static_cast<float>(mFar->value());
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
          WbLog::info(QString("[WbCamera] '%1' rendered through wgpu (%2x%3, %4 draws) "
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
    const bool agxMode = qEnvironmentVariableIntValue("OMNISIM_CAMERA_AGX") >= 1;
    // The clearAndDrawScene `farPlane` arg doubles as AgX exposure when agxMode
    // (the two modes share pad0.x and are mutually exclusive). Exposure = 2^EV
    // from OMNISIM_CAMERA_AGX_EV (integer stops; default 0 → 1.0 → byte-identical
    // to the no-exposure AgX path). Depth/lit path passes the real far plane.
    float farOrExposure = static_cast<float>(mFar->value());
    if (agxMode && !depthMode) {
      const int ev = qEnvironmentVariableIntValue("OMNISIM_CAMERA_AGX_EV");
      farOrExposure = static_cast<float>(std::pow(2.0, static_cast<double>(ev)));
    }
    // T1.1 specular: hand the camera world position to the AgX path so the
    // gated Blinn-Phong highlight can form its view vector. Only the (gated)
    // specular branch reads it; null/zero on every non-AgX or rough-surface
    // frame leaves output byte-identical.
    const WbVector3 camWorld = matrix().translation();
    const float camPos3[3] = {static_cast<float>(camWorld.x()), static_cast<float>(camWorld.y()),
                              static_cast<float>(camWorld.z())};
    // srgbEncode=false: the camera is a SENSOR, not a display surface — keep its plain-lit
    // (kSolidLit) output in the R5-landed LINEAR space its controllers/ML consumers expect, NOT
    // the display-sRGB the main pane uses. Only the plain-lit path reads this (depth/AgX modes
    // write farPlane/exposure to the same pad0.x slot and are unaffected).
    if (mWgpuTarget->clearAndDrawScene(clear, vpToUse, lightDirAmbient,
                                       draws.empty() ? nullptr : draws.data(),
                                       static_cast<uint32_t>(draws.size()), data,
                                       depthMode, farOrExposure, agxMode, camPos3,
                                       /*pickMode=*/false, /*srgbEncode=*/false)) {
      // One-time per Camera log so test harnesses + smoke runs can
      // verify the wgpu path actually fired (vs falling through to
      // WREN silently). The mWgpuTargetWidth check is harmless +
      // makes this fire on every resize as well.
      static thread_local int sLoggedCount = 0;
      if (sLoggedCount < 4) {
        WbLog::info(QString("[WbCamera] '%1' rendered through wgpu (%2x%3, %4 draw%5)%6")
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
    // Fall through to WREN if the readback failed — better a real
    // frame than a corrupt one.
  }
  WbAbstractCamera::copyImageToMemoryMappedFile(camera, data);
}

void WbCamera::preFinalize() {
  WbAbstractCamera::preFinalize();

  if (zoom())
    zoom()->preFinalize();

  if (recognition())
    recognition()->preFinalize();

  if (focus())
    focus()->preFinalize();

  if (lensFlare())
    lensFlare()->preFinalize();

  mRecognitionSensor = new WbSensor();

  updateNear();
  updateFar();
  updateExposure();
  updateBloomThreshold();
  updateAmbientOcclusionRadius();
}

void WbCamera::postFinalize() {
  WbAbstractCamera::postFinalize();

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

  connect(mFocus, &WbSFNode::changed, this, &WbCamera::updateFocus);
  connect(mRecognition, &WbSFNode::changed, this, &WbCamera::updateRecognition);
  connect(mNear, &WbSFDouble::changed, this, &WbCamera::updateNear);
  connect(mFar, &WbSFDouble::changed, this, &WbCamera::updateFar);
  connect(mExposure, &WbSFDouble::changed, this, &WbCamera::updateExposure);
  connect(mAmbientOcclusionRadius, &WbSFDouble::changed, this, &WbCamera::updateAmbientOcclusionRadius);
  connect(mBloomThreshold, &WbSFDouble::changed, this, &WbCamera::updateBloomThreshold);
  connect(mAntiAliasing, &WbSFBool::changed, this, &WbAbstractCamera::updateAntiAliasing);

  if (lensFlare())
    lensFlare()->postFinalize();
}

WbFocus *WbCamera::focus() const {
  return dynamic_cast<WbFocus *>(mFocus->value());
}

WbZoom *WbCamera::zoom() const {
  return dynamic_cast<WbZoom *>(mZoom->value());
}

WbRecognition *WbCamera::recognition() const {
  return dynamic_cast<WbRecognition *>(mRecognition->value());
}

WbLensFlare *WbCamera::lensFlare() const {
  return dynamic_cast<WbLensFlare *>(mLensFlare->value());
}

void WbCamera::initializeImageMemoryMappedFile() {
  WbAbstractCamera::initializeImageMemoryMappedFile();
  if (mImageMemoryMappedFile) {
    // initialize the memory mapped file with a black image
    int *im = reinterpret_cast<int *>(image());
    const int cameraSize = width() * height();
    for (int i = 0; i < cameraSize; i++)
      im[i] = 0xFF000000;
  }
}

void WbCamera::initializeSegmentationMemoryMappedFile() {
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

QString WbCamera::pixelInfo(int x, int y) const {
  WbRgb color;
  if (hasBeenSetup())
    color = mWrenCamera->copyPixelColourValue(x, y);

  const int red = color.red() * 255;
  const int green = color.green() * 255;
  const int blue = color.blue() * 255;

  return QString::asprintf("pixel(%d,%d)=#%02X%02X%02X", x, y, red, green, blue);
}

void WbCamera::updateRecognizedObjectsOverlay(double screenX, double screenY, double overlayX, double overlayY) {
  if (!recognition() || !mSensor->isEnabled()) {
    clearRecognizedObjectsOverlay();
    return;
  }

  // check if mouse is over an object
  int objectIndex = -1;
  for (int i = 0; i < mRecognizedObjects.size(); ++i) {
    const WbVector2 positionOnImage = mRecognizedObjects.at(i)->positionOnImage();
    const WbVector2 pixelsSize = mRecognizedObjects.at(i)->pixelSize();
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
    const WbRecognizedObject *object = mRecognizedObjects.at(objectIndex);
    QString text(object->model());
    text += tr("\nId: %1").arg(object->id());
    text += tr("\nRelative position: %1 %2 %3")
              .arg(WbPrecision::doubleToString(object->objectRelativePosition().x(), WbPrecision::GUI_LOW))
              .arg(WbPrecision::doubleToString(object->objectRelativePosition().y(), WbPrecision::GUI_LOW))
              .arg(WbPrecision::doubleToString(object->objectRelativePosition().z(), WbPrecision::GUI_LOW));
    text += tr("\nRelative orientation: %1 %2 %3 %4")
              .arg(WbPrecision::doubleToString(object->relativeOrientation().x(), WbPrecision::GUI_LOW))
              .arg(WbPrecision::doubleToString(object->relativeOrientation().y(), WbPrecision::GUI_LOW))
              .arg(WbPrecision::doubleToString(object->relativeOrientation().z(), WbPrecision::GUI_LOW))
              .arg(WbPrecision::doubleToString(object->relativeOrientation().angle(), WbPrecision::GUI_LOW));
    text += tr("\nSize: %1 %2")
              .arg(WbPrecision::doubleToString(object->objectSize().y(), WbPrecision::GUI_LOW))
              .arg(WbPrecision::doubleToString(object->objectSize().z(), WbPrecision::GUI_LOW));
    text += tr("\nPosition on the image: %1 %2").arg(object->positionOnImage().x()).arg(object->positionOnImage().y());
    text += tr("\nSize on the image: %1 %2").arg(object->pixelSize().x()).arg(object->pixelSize().y());
    for (int i = 0; i < object->colors().size(); ++i)
      text += tr("\nColor %1: %2 %3 %4")
                .arg(i)
                .arg(WbPrecision::doubleToString(object->colors().at(i).red(), WbPrecision::GUI_LOW))
                .arg(WbPrecision::doubleToString(object->colors().at(i).green(), WbPrecision::GUI_LOW))
                .arg(WbPrecision::doubleToString(object->colors().at(i).blue(), WbPrecision::GUI_LOW));
    if (mLabelOverlay == NULL) {
      mLabelOverlay = WbWrenLabelOverlay::createOrRetrieve(WbWrenLabelOverlay::cameraCaptionOverlayId(),
                                                           WbStandardPaths::fontsPath() + "LiberationSans-Regular.ttf");
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

void WbCamera::clearRecognizedObjectsOverlay() {
  if (mLabelOverlay) {
    WbWrenLabelOverlay::removeLabel(WbWrenLabelOverlay::cameraCaptionOverlayId());
    mLabelOverlay = NULL;
  }
}

WrTexture *WbCamera::getWrenTexture() {
  if (!hasBeenSetup())
    setup();
  if (mWrenCamera)
    return mWrenCamera->getWrenTexture();
  return NULL;
}

void WbCamera::displayRecognizedObjectsInOverlay() {
  assert(recognition());

  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::GPU_MEMORY_TRANSFER);

  const int w = width();
  const int h = height();
  const int cameraSize = w * h;
  const int frameThickness = recognition()->frameThickness();

  if (frameThickness > 0) {
    const int color = 0xFF000000 + (recognition()->frameColor().redByte() << 16) +
                      (recognition()->frameColor().greenByte() << 8) + recognition()->frameColor().blueByte();

    int *data = new int[cameraSize];
    int *clearData = new int[cameraSize];
    for (int i = 0; i < cameraSize; ++i) {
      data[i] = color;
      clearData[i] = 0;
    }

    WbWrenOpenGlContext::makeWrenCurrent();

    // Clear texture
    wr_texture_change_data(mRecognizedObjectsTexture, clearData, 0, 0, w, h);

    for (int i = 0; i < mRecognizedObjects.size(); ++i) {
      const WbRecognizedObject *object = mRecognizedObjects.at(i);
      const int x1 = qMax(0, (int)(object->positionOnImage().x() - object->pixelSize().x() / 2 - 1));
      const int x2 = qMin(w - 1, (int)(object->positionOnImage().x() + object->pixelSize().x() / 2 + 1));
      const int y1 = qMax(0, (int)(object->positionOnImage().y() - object->pixelSize().y() / 2 - 1));
      const int y2 = qMin(h - 1, (int)(object->positionOnImage().y() + object->pixelSize().y() / 2 + 1));

      wr_texture_change_data(mRecognizedObjectsTexture, data, x1, y1, frameThickness, y2 - y1);
      wr_texture_change_data(mRecognizedObjectsTexture, data, x2 - frameThickness, y1, frameThickness, y2 - y1);
      wr_texture_change_data(mRecognizedObjectsTexture, data, x1, y1, x2 - x1, frameThickness);
      wr_texture_change_data(mRecognizedObjectsTexture, data, x1, y2 - frameThickness, x2 - x1, frameThickness);
    }
    WbWrenOpenGlContext::doneWren();

    delete[] data;
    delete[] clearData;
  }

  if (log)
    log->stopMeasure(WbPerformanceLog::GPU_MEMORY_TRANSFER);
}

void WbCamera::prePhysicsStep(double ms) {
  WbAbstractCamera::prePhysicsStep(ms);

  if (isPowerOn() && mRecognitionSensor->isEnabled() && mRecognitionSensor->needToRefreshInMs(ms)) {
    // delete them only when sensor will be updated (we need to keep them for the overlay)
    qDeleteAll(mRecognizedObjects);
    mRecognizedObjects.clear();
  } else if (mNeedToDeleteRecognizedObjectsRays) {
    // we can destroy the ray because we don't need it anymore
    foreach (WbRecognizedObject *recognizedObject, mRecognizedObjects)
      recognizedObject->deleteRays();
    mNeedToDeleteRecognizedObjectsRays = false;
  }

  if (isPowerOn() && mRecognitionSensor->isEnabled() && mRecognitionSensor->needToRefreshInMs(ms) && recognition() &&
      recognition()->occlusion() > 0) {
    // create rays
    computeRecognizedObjects();
    mNeedToDeleteRecognizedObjectsRays = true;

    if (!mRecognizedObjects.isEmpty()) {
      const QList<dGeomID> &rays = mRecognizedObjects[0]->geoms();
      if (!rays.isEmpty())
        subscribeToRaysUpdate(rays.first());
    }
  }
}

void WbCamera::postPhysicsStep() {
  WbAbstractCamera::postPhysicsStep();

  // delete invalid WbRecognizedObject and ODE rays
  // it is preferable to not delete them during the physics step to avoid
  // possible inconsistencies in other clusters
  qDeleteAll(mInvalidRecognizedObjects);
  mInvalidRecognizedObjects.clear();

  if (!isControllerRunning() && recognition())
    // update recognized objects info displayed from the overlay in case occlusion is FALSE
    // (otherwise updated in the WbCamera::writeAnswer method)
    refreshRecognitionSensorIfNeeded();
}

void WbCamera::reset(const QString &id) {
  WbAbstractCamera::reset(id);

  WbNode *const focusNode = mFocus->value();
  if (focusNode)
    focusNode->reset(id);
  WbNode *const zoomNode = mZoom->value();
  if (zoomNode)
    zoomNode->reset(id);
  WbNode *const recognitionNode = mRecognition->value();
  if (recognitionNode)
    recognitionNode->reset(id);
  WbNode *const lensFlareNode = mLensFlare->value();
  if (lensFlareNode)
    lensFlareNode->reset(id);
}

void WbCamera::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();

  // compute the camera frustum planes
  const double horizontalFieldOfView = fieldOfView();
  const double verticalFieldOfView =
    (isPlanarProjection() || horizontalFieldOfView > M_PI) ?
      WbWrenCamera::computeFieldOfViewY(horizontalFieldOfView, (double)width() / (double)height()) :
      mWrenCamera->sphericalFieldOfViewY();
  const WbAffinePlane *frustumPlanes = WbObjectDetection::computeFrustumPlanes(this, verticalFieldOfView, horizontalFieldOfView,
                                                                               recognition()->maxRange(), isPlanarProjection());

  // update list of recognized objects
  foreach (WbRecognizedObject *recognizedObject, mRecognizedObjects) {
    recognizedObject->object()->updateTransformForPhysicsStep();
    if (!recognizedObject->recomputeRayDirection(frustumPlanes) || !setRecognizedObjectProperties(recognizedObject)) {
      mRecognizedObjects.removeAll(recognizedObject);
      mInvalidRecognizedObjects.append(recognizedObject);
    }
  }

  delete[] frustumPlanes;
}

void WbCamera::rayCollisionCallback(dGeomID geom, WbSolid *collidingSolid, double depth) {
  foreach (WbRecognizedObject *recognizedObject, mRecognizedObjects) {
    // check if this object is the one that collides
    if (recognizedObject->contains(geom)) {
      // make sure the colliding solid is not the target itself (or a sub-part)
      if (recognizedObject->object() != collidingSolid && !recognizedObject->object()->solidChildren().contains(collidingSolid))
        recognizedObject->setCollided(geom, depth);
      return;
    }
  }
}

void WbCamera::addConfigureToStream(WbDataStream &stream, bool reconfigure) {
  WbAbstractCamera::addConfigureToStream(stream, reconfigure);
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

void WbCamera::resetMemoryMappedFile() {
  WbAbstractCamera::resetMemoryMappedFile();
  if (hasBeenSetup() && (mSegmentationMemoryMappedFile || (recognition() && recognition()->segmentation())))
    // the previous memory mapped file will be released by the new controller start
    initializeSegmentationMemoryMappedFile();
}

void WbCamera::writeConfigure(WbDataStream &stream) {
  WbAbstractCamera::writeConfigure(stream);

  mRecognitionSensor->connectToRobotSignal(robot());
}

void WbCamera::writeAnswer(WbDataStream &stream) {
  WbAbstractCamera::writeAnswer(stream);

  if (mSegmentationImageChanged) {
    if (mIsRemoteExternController) {
      editChunkMetadata(stream, size());

      // copy image to stream
      stream << (short unsigned int)tag();
      stream << (unsigned char)C_CAMERA_SERIAL_SEGMENTATION_IMAGE;
      int streamLength = stream.length();
      stream.resize(size() + streamLength);
      if (mSegmentationCamera) {
        mSegmentationCamera->enableCopying(true);
        mSegmentationCamera->copyContentsToMemory(stream.data() + streamLength);
      }

      // prepare next chunk
      stream.mSizePtr = stream.length();
      stream << (int)0;
      stream << (unsigned char)0;
    } else
      copyImageToMemoryMappedFile(mSegmentationCamera, static_cast<unsigned char *>(mSegmentationMemoryMappedFile->data()));
    mSegmentationImageChanged = false;
  }

  if (recognition()) {
    if (refreshRecognitionSensorIfNeeded() || mRecognitionSensor->hasPendingValue()) {
      stream << tag();
      stream << (unsigned char)C_CAMERA_OBJECTS;
      const int numberOfObjects = mRecognizedObjects.size();
      stream << (int)numberOfObjects;
      for (int i = 0; i < numberOfObjects; ++i) {
        const WbRecognizedObject *recognizedObject = mRecognizedObjects.at(i);
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
      if (mSensor->isEnabled())
        displayRecognizedObjectsInOverlay();
    }

    if (mSegmentationChanged) {
      stream << (short unsigned int)tag();
      stream << (unsigned char)C_CAMERA_SET_SEGMENTATION;
      stream << (unsigned char)recognition()->segmentation();
      mSegmentationChanged = false;
    }

    if (mSegmentationCamera && mHasSegmentationMemoryMappedFileChanged && !mIsRemoteExternController) {
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

void WbCamera::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  if (WbAbstractCamera::handleCommand(stream, command))
    return;

  switch (command) {
    case C_CAMERA_SET_FOV: {
      double fov;
      stream >> fov;

      const WbZoom *z = zoom();
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

      WbFocus *f = focus();
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
      if (!hasBeenSetup())
        setup();
      else
        updateOverlayMaskTexture();
      emit enabled(this, isEnabled());
      break;
    default:
      assert(0);
  }
}

void WbCamera::computeRecognizedObjects() {
  // compute the camera referential
  const WbVector3 cameraPosition = position();
  const double horizontalFieldOfView = fieldOfView();
  const double verticalFieldOfView =
    (isPlanarProjection() || horizontalFieldOfView > M_PI) ?
      WbWrenCamera::computeFieldOfViewY(horizontalFieldOfView, (double)width() / (double)height()) :
      mWrenCamera->sphericalFieldOfViewY();
  const WbAffinePlane *frustumPlanes = WbObjectDetection::computeFrustumPlanes(this, verticalFieldOfView, horizontalFieldOfView,
                                                                               recognition()->maxRange(), isPlanarProjection());

  // loop for each possible target to check if it is visible
  const QList<WbSolid *> objects = WbWorld::instance()->cameraRecognitionObjects();
  for (int i = 0; i < objects.size(); i++) {
    WbSolid *object = objects.at(i);
    if (object == this || robot() == object || robot()->solidChildren().contains(object))
      continue;
    // We should discard targets as soon as possible to improve performance.
    if ((cameraPosition - object->position() - object->boundingSphere()->center()).length() >
        (recognition()->maxRange() + object->boundingSphere()->scaledRadius()))
      continue;
    // create target
    WbRecognizedObject *generatedObject =
      new WbRecognizedObject(this, object, recognition()->occlusion(), recognition()->maxRange());
    if (!generatedObject->isContainedInFrustum(frustumPlanes) || !setRecognizedObjectProperties(generatedObject)) {
      delete generatedObject;
      continue;
    }
    mRecognizedObjects.append(generatedObject);
  }

  delete[] frustumPlanes;
}

WbVector2 WbCamera::applyCameraDistortionToImageCoordinate(const WbVector2 &uv) {
  if (!lens())
    return uv;
  WbVector2 distortedUv(uv);
  const WbVector2 &rc = lens()->radialCoefficients();
  const WbVector2 &tc = lens()->tangentialCoefficients();
  const bool hasRadialDistortion = rc.x() != 0 || rc.y() != 0;
  const bool hasTangentialDistortion = tc.x() != 0 || tc.y() != 0;
  const WbVector2 relativeUv = uv - lens()->center();
  const float r2 = relativeUv.length2();
  if (hasRadialDistortion)
    distortedUv += (rc.x() * r2 + rc.y() * r2 * r2) * relativeUv;
  if (hasTangentialDistortion) {
    distortedUv +=
      WbVector2(2 * tc.x() * relativeUv.x() * relativeUv.y() + tc.y() * (r2 + 2 * relativeUv.x() * relativeUv.x()),
                tc.x() * (r2 + 2 * relativeUv.y() * relativeUv.y() + 2 * tc.y() * relativeUv.x() * relativeUv.y()));
  }
  distortedUv.setX(qMax(0.0, qMin(distortedUv.x(), 1.0)));
  distortedUv.setY(qMax(0.0, qMin(distortedUv.y(), 1.0)));
  return distortedUv;
}

WbVector2 WbCamera::projectOnImage(const WbVector3 &position) {
  const double fovX = fieldOfView();
  WbVector2 uv;  // uv coordinates in range [-0.5, 0.5]
  if (mProjection->value() == "planar") {
    if (position.x() < 0)
      uv.setXy(position.y() > 0 ? -0.5 : 0.5, position.z() > 0 ? -0.5 : 0.5);
    else {
      const double fovY = WbWrenCamera::computeFieldOfViewY(fovX, (double)width() / (double)height());
      const double theta1 = -atan2(position.y(), fabs(position.x()));
      const double theta2 = atan2(position.z(), fabs(position.x()));
      uv.setX(0.5 * tan(theta1) / tan(0.5 * fovX));
      uv.setY(-0.5 * tan(theta2) / tan(0.5 * fovY));
    }
  } else if (mProjection->value() == "spherical") {
    const WbVector3 p = position.normalized();
    const double b = p.z() / p.y();
    uv.setY(acos(p.x()) / (fovX * sqrt(1 + 1 / (b * b))));
    if (p.z() > 0.0)
      uv.setY(-uv.y());
    uv.setX(uv.y() / b);
    if (fovX < M_PI_2)
      uv.setX(uv.x() * M_PI_2 / fovX);
  } else {
    assert(mProjection->value() == "cylindrical");
    const double fovY = mWrenCamera->sphericalFieldOfViewY();
    const double fovYCorrectionCoefficient = mWrenCamera->sphericalFovYCorrectionCoefficient();
    const WbVector3 normP = position.normalized();
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
  return WbVector2((int)((width() - 1) * uv.x()), (int)((height() - 1) * uv.y()));
}

bool WbCamera::setRecognizedObjectProperties(WbRecognizedObject *recognizedObject) {
  assert(recognizedObject);

  // compute object relative orientation
  WbRotation relativeRotation;
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
  QList<WbVector3> corners = recognizedObject->computeCorners();
  if (!isPlanarProjection())
    corners << WbVector3(0, 0, 0);  // add center to better localize deformed objects
  QListIterator<WbVector3> cornerIt(corners);
  while (cornerIt.hasNext()) {  // project each of the corners in the camera image
    const WbVector2 &positionOnImage = projectOnImage(recognizedObject->objectRelativePosition() + cornerIt.next());
    if (positionOnImage.x() < minU)
      minU = positionOnImage.x();
    if (positionOnImage.y() < minV)
      minV = positionOnImage.y();
    if (positionOnImage.x() > maxU)
      maxU = positionOnImage.x();
    if (positionOnImage.y() > maxV)
      maxV = positionOnImage.y();
  }
  recognizedObject->setPositionOnImage(WbVector2(round((maxU + minU) / 2.0), round((maxV + minV) / 2.0)));
  recognizedObject->setPixelSize(WbVector2(ceil(maxU - minU), ceil(maxV - minV)));
  if (recognizedObject->pixelSize().x() < 1 || recognizedObject->pixelSize().y() < 1)
    return false;

  return true;
}

bool WbCamera::refreshRecognitionSensorIfNeeded() {
  assert(recognition());

  if (!isPowerOn() || !mRecognitionSensor->needToRefresh())
    return false;

  if (recognition()->occlusion() == 0)
    // no need of ODE ray collision detection
    // rays can be created at the end of the step when all the body positions are up-to-date
    computeRecognizedObjects();
  else
    // post process objects
    removeOccludedRecognizedObjects();

  // check the number of objects and keep only the biggest (in pixel size)
  if (recognition()->maxObjects() > 0 && mRecognizedObjects.size() > recognition()->maxObjects()) {
    QMultiMap<int, WbRecognizedObject *> objectsMap;
    for (int i = 0; i < mRecognizedObjects.size(); ++i) {
      const int pixelSurface = mRecognizedObjects.at(i)->pixelSize().x() * mRecognizedObjects.at(i)->pixelSize().y();
      objectsMap.insert(pixelSurface, mRecognizedObjects.at(i));
    }
    mRecognizedObjects.clear();
    QMultiMapIterator<int, WbRecognizedObject *> i(objectsMap);
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

void WbCamera::removeOccludedRecognizedObjects() {
  for (int i = mRecognizedObjects.size() - 1; i >= 0; --i) {
    if (mRecognizedObjects.at(i)->hasCollided()) {
      delete mRecognizedObjects[i];
      mRecognizedObjects.removeAt(i);
    }
  }
}

void WbCamera::createWrenCamera() {
  // Safely detach lens flare postProcessingEffect from camera viewport (before it is deleted)
  if (lensFlare())
    lensFlare()->detachFromViewport();

  WbAbstractCamera::createWrenCamera();

  if (mSegmentationCamera)
    delete mSegmentationCamera;
  if (recognition() && recognition()->segmentation()) {
    mSegmentationCamera = new WbWrenCamera(wrenNode(), width(), height(), nearValue(), minRange(), recognition()->maxRange(),
                                           fieldOfView(), 's', false, mProjection->value());
    connect(mSensor, &WbSensor::stateChanged, this, &WbCamera::updateOverlayMaskTexture);
  } else {
    mSegmentationCamera = NULL;
    disconnect(mSensor, &WbSensor::stateChanged, this, &WbCamera::updateOverlayMaskTexture);
  }

  applyCameraSettings();
  applyFocalSettingsToWren();
  applyFarToWren();
  updateExposure();
  updateBloomThreshold();
  updateAmbientOcclusionRadius();

  updateLensFlare();
  updateCameraOrientation();
  connect(mWrenCamera, &WbWrenCamera::cameraInitialized, this, &WbCamera::updateLensFlare);
  connect(mWrenCamera, &WbWrenCamera::cameraInitialized, this, &WbCamera::updateCameraOrientation);

  if (mSegmentationCamera) {
    updateSegmentationCameraOrientation();
    connect(mSegmentationCamera, &WbWrenCamera::cameraInitialized, this, &WbCamera::updateSegmentationCameraOrientation);
  }
}

void WbCamera::createWrenOverlay() {
  WbAbstractCamera::createWrenOverlay();

  // mRecognizedObjectsTexture deleted when creating the WREN overlay
  assert(recognition() || !mRecognizedObjectsTexture);
  if (mWrenCamera && recognition()) {
    mRecognizedObjectsTexture = WR_TEXTURE(mOverlay->createForegroundTexture());
    emit textureIdUpdated(mOverlay->foregroundTextureGLId(), FOREGROUND_TEXTURE);
    updateOverlayMaskTexture();
  }
}

void WbCamera::updateOverlayMaskTexture() {
  if (!mOverlay)
    return;

  if (mWrenCamera && mSensor->isEnabled() && recognition() && mSegmentationEnabled && mSegmentationCamera)
    mOverlay->setMaskTexture(mSegmentationCamera->getWrenTexture());
  else
    mOverlay->unsetMaskTexture();
  emit textureIdUpdated(mOverlay->maskTextureGLId(), MASK_TEXTURE);
}

void WbCamera::updateTextureUpdateNotifications(bool enabled) {
  WbAbstractCamera::updateTextureUpdateNotifications(enabled);
  if (!mWrenCamera || !mSegmentationCamera)
    return;
  if (enabled && mExternalWindowEnabled)
    connect(mSegmentationCamera, &WbWrenCamera::textureUpdated, this, &WbRenderingDevice::textureUpdated, Qt::UniqueConnection);
  else
    disconnect(mSegmentationCamera, &WbWrenCamera::textureUpdated, this, &WbRenderingDevice::textureUpdated);
  mSegmentationCamera->enableTextureUpdateNotifications(mExternalWindowEnabled);
}

void WbCamera::setup() {
  WbAbstractCamera::setup();
  createSegmentationCamera();
  if (mSegmentationMemoryMappedFile || (recognition() && recognition()->segmentation()))
    initializeSegmentationMemoryMappedFile();

  if (!isPlanarProjection())
    return;

  updateNoiseMaskUrl();
  updateExposure();
  updateAmbientOcclusionRadius();
  updateBloomThreshold();
  connect(mNoiseMaskUrl, &WbSFString::changed, this, &WbCamera::updateNoiseMaskUrl);

  // make sure bounding sphere is updated when object's size and position changes
  if (recognition()) {
    mIsSubscribedToRayTracing = true;
    WbSimulationState::instance()->subscribeToRayTracing();
  }
}

bool WbCamera::isEnabled() const {
  return (mSegmentationEnabled && recognition() && recognition()->segmentation()) || WbAbstractCamera::isEnabled();
}

bool WbCamera::needToRender() const {
  return WbAbstractCamera::needToRender() ||
         (mSegmentationEnabled && mRecognitionSensor->isEnabled() && mRecognitionSensor->needToRefresh());
}

void WbCamera::render() {
  if (mSegmentationCamera && mSegmentationEnabled && isPowerOn() && mRecognitionSensor->isEnabled() &&
      mRecognitionSensor->needToRefresh()) {
    mSegmentationCamera->render();
    mSegmentationImageChanged = true;
  }
}

WbVector3 WbCamera::urdfRotation(const WbMatrix3 &rotationMatrix) const {
  WbVector3 eulerRotation = rotationMatrix.toEulerAnglesZYX();
  // Webots defines the camera frame as FLU but ROS desfines it as RDF (Right-Down-Forward)
  eulerRotation[0] -= M_PI / 2;
  eulerRotation[2] -= M_PI / 2;
  return eulerRotation;
}

void WbCamera::exportNodeFields(WbWriter &writer) const {
  WbAbstractCamera::exportNodeFields(writer);

  exportSFResourceField("noiseMaskUrl", mNoiseMaskUrl, writer.relativeMeshesPath(), writer);
}

QStringList WbCamera::customExportedFields() const {
  QStringList fields;
  fields << "noiseMaskUrl";
  return fields;
}

/////////////////////
//  Update methods //
/////////////////////

void WbCamera::updateFocus() {
  if (focus())
    connect(focus(), &WbFocus::focusSettingsChanged, this, &WbCamera::applyFocalSettingsToWren);
  else if (hasBeenSetup())
    mNeedToConfigure = true;
  if (hasBeenSetup())
    applyFocalSettingsToWren();
}

void WbCamera::updateRecognition() {
  if (!hasBeenSetup())
    return;

  const WbRecognition *recognitionNode = recognition();
  if (recognitionNode && !mOverlay->foregroundTexture()) {
    mRecognizedObjectsTexture = WR_TEXTURE(mOverlay->createForegroundTexture());
    emit textureIdUpdated(mOverlay->foregroundTextureGLId(), FOREGROUND_TEXTURE);
  } else if (mOverlay->foregroundTexture()) {
    mOverlay->deleteForegroundTexture(true);
    emit textureIdUpdated(mOverlay->foregroundTextureGLId(), FOREGROUND_TEXTURE);
    mRecognizedObjectsTexture = NULL;
  }

  // make sure bounding sphere is updated when object's size and position changes
  const bool recognitionEnabled = recognitionNode != NULL;
  if (recognitionEnabled != mIsSubscribedToRayTracing) {
    mIsSubscribedToRayTracing = recognitionEnabled;
    if (recognitionNode)
      WbSimulationState::instance()->subscribeToRayTracing();
    else
      WbSimulationState::instance()->unsubscribeToRayTracing();
  }

  // clear mRecognizedObjects if Recognition node changed but not if `Recognition.segmentation` changed
  qDeleteAll(mRecognizedObjects);
  mRecognizedObjects.clear();

  createSegmentationCamera();

  mNeedToConfigure = true;
}

void WbCamera::updateSegmentation() {
  mSegmentationChanged = true;
  if (mSegmentationEnabled && (!recognition() || !recognition()->segmentation())) {
    mSegmentationEnabled = false;
    updateOverlayMaskTexture();
  }
  createSegmentationCamera();
}

void WbCamera::createSegmentationCamera() {
  const WbRecognition *recognitionNode = recognition();
  if (recognitionNode)
    connect(recognitionNode, &WbRecognition::segmentationChanged, this, &WbCamera::updateSegmentation, Qt::UniqueConnection);

  delete mSegmentationCamera;

  if (recognitionNode && recognitionNode->segmentation()) {
    mSegmentationCamera = new WbWrenCamera(wrenNode(), width(), height(), nearValue(), minRange(), recognition()->maxRange(),
                                           fieldOfView(), 's', false, mProjection->value());
    connect(mSensor, &WbSensor::stateChanged, this, &WbCamera::updateOverlayMaskTexture);
    if (!mSegmentationMemoryMappedFile)
      initializeSegmentationMemoryMappedFile();
  } else {
    mSegmentationCamera = NULL;
    disconnect(mSensor, &WbSensor::stateChanged, this, &WbCamera::updateOverlayMaskTexture);
  }
  updateOverlayMaskTexture();
  if (mExternalWindowEnabled)
    updateTextureUpdateNotifications(mExternalWindowEnabled);

  if (mSegmentationCamera) {
    updateSegmentationCameraOrientation();
    connect(mSegmentationCamera, &WbWrenCamera::cameraInitialized, this, &WbCamera::updateSegmentationCameraOrientation);
  }
}

void WbCamera::updateLensFlare() {
  if (hasBeenSetup() && lensFlare()) {
    if (!isPlanarProjection()) {
      parsingWarn(tr("Lens flare can only be applied to planar cameras."));
      return;
    }
    WrViewport *viewport = mWrenCamera->getSubViewport(WbWrenCamera::CAMERA_ORIENTATION_FRONT);
    lensFlare()->setup(viewport);
  }
}

void WbCamera::updateCameraOrientation() {
  if (hasBeenSetup()) {
    // FLU axis orientation
    mWrenCamera->rotateRoll(M_PI_2);
    mWrenCamera->rotateYaw(-M_PI_2);
  }
}

void WbCamera::updateSegmentationCameraOrientation() {
  // FLU axis orientation
  mSegmentationCamera->rotateRoll(M_PI_2);
  mSegmentationCamera->rotateYaw(-M_PI_2);
}

void WbCamera::updateNear() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.01))
    return;

  mNeedToConfigure = true;

  if (mFar->value() > 0.0 and mFar->value() < mNear->value()) {
    mNear->setValue(mFar->value());
    parsingWarn(tr("'near' is greater than 'far'. Setting 'near' to %1.").arg(mNear->value()));
  }

  if (hasBeenSetup())
    applyNearToWren();

  if (areWrenObjectsInitialized()) {
    applyFrustumToWren();
    if (isPlanarProjection() && hasBeenSetup())
      updateFrustumDisplay();
  }
}

void WbCamera::updateFar() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mFar, 0.0))
    return;

  if (mFar->value() > 0.0 and mFar->value() < mNear->value()) {
    mFar->setValue(mNear->value() + 1.0);
    parsingWarn(tr("'far' is less than 'near'. Setting 'far' to %1.").arg(mFar->value()));
    return;
  }

  if (hasBeenSetup())
    applyFarToWren();

  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbCamera::updateExposure() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mExposure, 1.0))
    return;

  if (mWrenCamera)
    mWrenCamera->setExposure(mExposure->value());
}

void WbCamera::updateAmbientOcclusionRadius() {
  WbFieldChecker::resetDoubleIfNegative(this, mAmbientOcclusionRadius, 2.0);

  if (mWrenCamera)
    mWrenCamera->setAmbientOcclusionRadius(mAmbientOcclusionRadius->value());
}

void WbCamera::updateBloomThreshold() {
  WbFieldChecker::resetDoubleIfNegativeAndNotDisabled(this, mBloomThreshold, 21.0, -1.0);

  if (mWrenCamera)
    mWrenCamera->setBloomThreshold(mBloomThreshold->value());
}

void WbCamera::updateNoiseMaskUrl() {
  if (!hasBeenSetup() || mNoiseMaskUrl->value().isEmpty())
    return;

  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  QString url = mNoiseMaskUrl->value();
  mNoiseMaskUrl->blockSignals(true);
  mNoiseMaskUrl->setValue(url.replace("\\", "/"));
  mNoiseMaskUrl->blockSignals(false);

  const QString &completeUrl = WbUrl::computePath(this, "noiseMaskUrl", mNoiseMaskUrl->value(), true);
  if (WbUrl::isWeb(completeUrl)) {
    if (mDownloader && !mDownloader->error().isEmpty()) {
      warn(mDownloader->error());  // failure downloading or file does not exist (404)
      delete mDownloader;
      mDownloader = NULL;
      return;
    }

    if (!WbNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
      downloadAssets();  // URL was changed from the scene tree or supervisor
      return;
    }
  }

  if (!(completeUrl == WbUrl::missingTexture() || completeUrl.isEmpty())) {
    const QString error = mWrenCamera->setNoiseMask(completeUrl);
    if (!error.isEmpty())
      parsingWarn(error);
  }
}

bool WbCamera::isFrustumEnabled() const {
  return WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_CAMERA_FRUSTUMS);
}

void WbCamera::updateFrustumDisplayIfNeeded(int optionalRendering) {
  if (optionalRendering == WbWrenRenderingContext::VF_CAMERA_FRUSTUMS)
    updateFrustumDisplay();
}

/////////////////////
//  Apply methods  //
/////////////////////

void WbCamera::applyFocalSettingsToWren() {
  if (hasBeenSetup()) {
    if (focus())
      mWrenCamera->setFocus(focus()->focalDistance(), focus()->focalLength());
    else
      mWrenCamera->setFocus(0.0, 0.0);
    mNeedToConfigure = true;
  }
}

void WbCamera::applyFarToWren() {
  mWrenCamera->setFar(mFar->value());
  if (mSegmentationCamera) {
    mSegmentationCamera->setFar(mFar->value());
    updateOverlayMaskTexture();
  }
}

void WbCamera::applyCameraSettingsToWren() {
  WbAbstractCamera::applyCameraSettingsToWren();
  applyFocalSettingsToWren();
}

void WbCamera::applyNearToWren() {
  WbAbstractCamera::applyNearToWren();
  if (mSegmentationCamera) {
    mSegmentationCamera->setNear(nearValue());
    updateOverlayMaskTexture();
  }
}

void WbCamera::applyFieldOfViewToWren() {
  WbAbstractCamera::applyFieldOfViewToWren();
  if (mSegmentationCamera) {
    mSegmentationCamera->setFieldOfView(mFieldOfView->value());
    updateOverlayMaskTexture();
  }
}

void WbCamera::applyLensToWren() {
  WbAbstractCamera::applyLensToWren();
  if (mSegmentationCamera && hasBeenSetup()) {
    const WbLens *l = lens();
    if (l) {
      mSegmentationCamera->enableLensDistortion();
      mSegmentationCamera->setLensDistortionCenter(l->center());
      mSegmentationCamera->setRadialLensDistortionCoefficients(l->radialCoefficients());
      mSegmentationCamera->setTangentialLensDistortionCoefficients(l->tangentialCoefficients());
    } else
      mSegmentationCamera->disableLensDistortion();
    updateOverlayMaskTexture();
  }
}
