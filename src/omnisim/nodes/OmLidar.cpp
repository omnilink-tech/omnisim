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

#include "OmLidar.hpp"

#include "OmBoundingSphere.hpp"
#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmRgb.hpp"
#include "OmSensor.hpp"
#include "OmSimulationState.hpp"
#include "OmWorld.hpp"

// R5e: wgpu range path for the Lidar device node.
#include "OmLog.hpp"
#include "OmMatrix4.hpp"
#include "OmRenderBackend.hpp"
#include "OmWgpuMeshCache.hpp"
#include "OmWgpuRenderTarget.hpp"
#include "OmWgpuSceneRenderer.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QtGlobal>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

void OmLidar::init() {
  mCharType = 'l';
  mIsPointCloudEnabled = false;
  mCurrentRotatingAngle = 0;
  mPreviousRotatingAngle = 0;
  mCurrentTiltAngle = 0;
  mTemporaryImage = NULL;

  mTiltAngle = findSFDouble("tiltAngle");
  mHorizontalResolution = findSFInt("horizontalResolution");
  mVerticalFieldOfView = findSFDouble("verticalFieldOfView");
  mNumberOfLayers = findSFInt("numberOfLayers");
  mMinRange = findSFDouble("minRange");
  mMaxRange = findSFDouble("maxRange");
  mResolution = findSFDouble("resolution");
  mDefaultFrequency = findSFDouble("defaultFrequency");
  mMinFrequency = findSFDouble("minFrequency");
  mMaxFrequency = findSFDouble("maxFrequency");
  mType = findSFString("type");
  mRotatingHead = findSFNode("rotatingHead");

  mActualNumberOfLayers = mNumberOfLayers->value();
  mActualHorizontalResolution = mHorizontalResolution->value();
  mActualVerticalFieldOfView = mVerticalFieldOfView->value();
  mActualFieldOfView = mFieldOfView->value();
  mIsActuallyRotating = mType->value().startsWith('r', Qt::CaseInsensitive);

  mTcpImage = NULL;
  mTcpCloudPoints = NULL;

  // R5e: wgpu range target stays null until the first copyAllLayersToMemory-
  // MappedFile with a usable Vulkan backend (D1.4: unconditional, WREN is deleted).
  mWgpuTarget = NULL;
  mWgpuMeshCache = NULL;
  mWgpuTargetWidth = 0;
  mWgpuTargetHeight = 0;

  // backward compatibility
  OmSFBool *sphericalField = findSFBool("spherical");
  if (!sphericalField->value()) {  // Deprecated in Webots R2023
    parsingWarn("Deprecated 'spherical' field, please use the 'projection' field instead.");
    if (mProjection->value() == "cylindrical")
      mProjection->setValue("planar");
    sphericalField->setValue(true);
  }
}

OmLidar::OmLidar(OmTokenizer *tokenizer) : OmAbstractCamera("Lidar", tokenizer) {
  init();
}

OmLidar::OmLidar(const OmLidar &other) : OmAbstractCamera(other) {
  init();
}

OmLidar::OmLidar(const OmNode &other) : OmAbstractCamera(other) {
  init();
}

OmLidar::~OmLidar() {
  delete mTemporaryImage;
  if (mIsRemoteExternController) {
    if (mIsPointCloudEnabled)
      delete mTcpCloudPoints;
    delete mTcpImage;
  }
  // R5e: same teardown order as the Camera/RangeFinder wgpu paths.
  delete mWgpuMeshCache;
  mWgpuMeshCache = NULL;
  delete mWgpuTarget;
  mWgpuTarget = NULL;
}

// Lane E1 (P5+P6): the wgpu envelope for a LIDAR, which is not the Camera/RangeFinder one.
// The wgpu Lidar pipeline is CYLINDRICAL by construction -- every branch of
// renderRangeViaWgpu angular-resamples perspective frustums into the uniform-angle layout
// updatePointCloud expects -- so the base class's "planar-only" projection test is exactly
// inverted here: a `projection "planar"` Lidar is the one thing that would silently get
// cylindrical columns (P6 finding 3). `noise` and `resolution` are PORTED
// (applyWgpuRangePostFx) and therefore absent from this list; Lidar declares no lens/
// lensFlare/focus/noiseMaskUrl/motionBlur fields, so the base checks have nothing to find.
QString OmLidar::wgpuUnsupportedFeature(bool includeSilentPostFx) const {
  Q_UNUSED(includeSilentPostFx);
  if (isPlanarProjection())
    return tr("projection \"planar\" (the wgpu Lidar pipeline is cylindrical-only; it would silently produce "
              "cylindrical columns)");
  if (actualFieldOfView() > 6.3)
    return tr("fieldOfView %1 (beyond the wgpu Lidar multi-frustum ceiling of 6.3 rad)").arg(actualFieldOfView());
  return QString();
}

// Lane E1 (P6), now the ONLY setup path (D1.4: every device sets up wrenless). The deleted
// createWrenCamera() used to be the latch site for these copies; without this the Lidar
// would report 0 layers / 0 columns.
void OmLidar::setupWrenless() {
  mActualNumberOfLayers = mNumberOfLayers->value();
  mActualHorizontalResolution = mHorizontalResolution->value();
  mActualVerticalFieldOfView = mVerticalFieldOfView->value();
  mActualFieldOfView = mFieldOfView->value();
  mIsActuallyRotating = mType->value().startsWith('r', Qt::CaseInsensitive);
  // D1.4: applyTiltAngleToWren() (deleted) used to latch the tilt at setup; the wgpu
  // render paths read mCurrentTiltAngle, so latch it here.
  mCurrentTiltAngle = mTiltAngle->value();
}

// F2.4 (wren-deletion-runbook): out-of-range is +INFINITY, per the reference contract
// (docs/reference/rangefinder.md:65/:69/:390 -- "infinity is returned" below minRange and
// above maxRange; the Lidar shares the RangeFinder's range-image semantics and its WREN
// pipeline wrote +inf, which lidar_rotating.c tests with isinf()). The wgpu resample sites
// below used to CLAMP into [lo, hi]; every one now maps anything outside the OPEN interval
// (minRange, maxRange) -- including the background seeded at exactly maxRange -- to +inf,
// mirroring the E1 noise path's range CLIP (applyWgpuRangePostFx), which is unchanged.
static inline float lidarRangeOrInf(float r, float lo, float hi) {
  return (r > lo && r < hi) ? r : std::numeric_limits<float>::infinity();
}

// F2 (wren-deletion-runbook): DISCONTINUITY-AWARE angular resample. A plain bilinear mix
// across a silhouette edge INVENTS ranges that exist on no surface -- measured on
// lidar_vs_range_finder (128-column cylindrical sweep vs the WREN oracle): 125/128 columns
// agreed, and the 3 that disagreed were all object-edge columns, the worst reporting
// 4.6458 m (a hit/background blend) where WREN reports a MISS (+inf). A lidar must return
// a range on a surface or no range at all; when the contributing samples straddle a depth
// edge (spread > 5% of maxRange), snap to the angularly-nearest sample instead of blending.
// Smooth regions (spread below the threshold) keep the sub-column bilinear precision.
static inline float lidarResample2(float r0, float r1, double t, float edge) {
  if (std::fabs(r0 - r1) > edge)
    return t < 0.5 ? r0 : r1;
  return static_cast<float>(r0 * (1.0 - t) + r1 * t);
}

static inline float lidarResample4(float r00, float r01, float r10, float r11, double tx, double ty, float edge) {
  const float mn = std::min(std::min(r00, r01), std::min(r10, r11));
  const float mx = std::max(std::max(r00, r01), std::max(r10, r11));
  if (mx - mn > edge) {
    if (ty < 0.5)
      return tx < 0.5 ? r00 : r01;
    return tx < 0.5 ? r10 : r11;
  }
  const double top = r00 * (1.0 - tx) + r01 * tx;
  const double bot = r10 * (1.0 - tx) + r11 * tx;
  return static_cast<float>(top * (1.0 - ty) + bot * ty);
}

// R5e: render the single-layer, non-rotating, narrow-FOV Lidar range scan via
// the wgpu radial-range pipeline, angular-resampling the perspective image into
// the uniform-angle layout updatePointCloud expects. Returns false (caller keeps
// WREN) for any config outside that subset, or on any wgpu failure. The default
// WREN path is untouched. See engine-migration-plan.md R5 OmLidar spec.
bool OmLidar::renderRangeViaWgpu(float *data) {
  const int layers = actualNumberOfLayers();
  const int res = actualHorizontalResolution();
  const double fov = actualFieldOfView();
  // Gate: non-rotating, no tilt, FOV ≤ 2π. Narrow FOV (≤ 1.4 rad) uses a single
  // perspective frustum (single- or multi-layer); wider single-layer FOV uses
  // the multi-frustum branch below.
  if (mIsActuallyRotating || layers < 1 || res <= 0 || fov <= 0.0 || fov > 6.3)
    return false;

  OmRenderBackend *back = OmRenderBackendRegistry::resolve(OmRenderBackendKind::Vulkan);
  if (!back || !back->isAvailable())
    return false;

  // Sensor tilt (R5i): pitch the render camera up by the tilt angle — rotate
  // forward +X toward up +Z, i.e. rotate about +Y by −tilt. The per-layer φ
  // sampling is UNCHANGED: it's measured relative to the (now tilted) forward,
  // so tilt cancels and lives entirely in the camera orientation. tilt==0 gives
  // the identity, so the camera stays byte-identical to matrix() and R5e/f/g/h
  // are unaffected.
  const bool hasTilt = mCurrentTiltAngle != 0.0;
  const OmMatrix4 tiltRot = OmMatrix4(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -mCurrentTiltAngle);

  // Wide-FOV (R5g/R5h): multi-frustum stitch. A single perspective frustum can't
  // span a wide azimuth, so split into N = ceil(fov/1.2) sub-frustums, render
  // each rotated about the sensor up-axis (camK = worldMatrix · Rz(θ_centre_k)),
  // and stitch by azimuth — for output column i, sample the frustum whose slice
  // contains θ_i at the local angle (θ_i − θ_centre_k). With L>1 layers each
  // frustum also covers the vertical FOV and is 2D-resampled per layer, using
  // the same φ convention + ndc_y→row orientation as the narrow multi-layer
  // path. Draws are view-independent → collected once.
  if (fov > 1.4) {
    const double kFrustumFov = 1.2;
    const int nFrustums = static_cast<int>(std::ceil(fov / kFrustumFov));
    const double fovPer = fov / nFrustums;
    const double tanHalfPer = std::tan(fovPer * 0.5);
    const bool multi = layers > 1;
    const double vfov = multi ? verticalFieldOfView() : fovPer;  // L==1: square render
    if (vfov <= 1e-6)
      return false;
    const double tanHalfV = std::tan(vfov * 0.5);
    const double renderAspect = tanHalfPer / tanHalfV;  // → per-frustum vertFov == vfov
    const int wfW = (res / nFrustums) + 8 > 32 ? (res / nFrustums) + 8 : 32;
    const int wfH = multi ? (layers * 8 > 64 ? layers * 8 : 64) : wfW;
    if (!OmWgpuSceneRenderer::ensureTarget(back, wfW, wfH, mWgpuTarget, mWgpuMeshCache,
                                           mWgpuTargetWidth, mWgpuTargetHeight))
      return false;
    std::vector<OmWgpuSolidDraw> draws;
    std::vector<std::array<float, 16>> modelStorage;
    modelStorage.reserve(64);
    draws.reserve(64);
    collectWgpuDraws(*mWgpuMeshCache, draws, modelStorage);  // once
    const double farVal = mMaxRange->value();
    const double dtheta = -fov / static_cast<double>(res);
    const double theta0 = fov / 2.0 + dtheta / 2.0;
    const double dphi = multi ? (-vfov / static_cast<double>(layers - 1)) : 0.0;
    const double phi0 = multi ? (vfov / 2.0) : 0.0;
    const float lo = static_cast<float>(mMinRange->value());
    const float hi = static_cast<float>(farVal);
    for (int k = 0; k < nFrustums; ++k) {
      const double thetaC = fov / 2.0 - fovPer * (k + 0.5);
      OmMatrix4 camK = matrix() * OmMatrix4(0, 0, 0, 0, 0, 1, thetaC);  // azimuth: rotate about up
      if (hasTilt) camK = camK * tiltRot;                               // then pitch by tilt
      float vp[16] = {0}, vm[16] = {0};
      OmWgpuSceneRenderer::buildViewProj(camK, fovPer, renderAspect, mNear->value(), farVal, vp);
      OmWgpuSceneRenderer::buildView(camK, vm);
      std::vector<float> persp(static_cast<size_t>(wfW) * static_cast<size_t>(wfH),
                               static_cast<float>(farVal));
      if (!mWgpuTarget->clearAndDrawSceneRangeF32(static_cast<float>(farVal), vp, vm,
                                                  draws.empty() ? nullptr : draws.data(),
                                                  static_cast<uint32_t>(draws.size()), persp.data()))
        return false;
      for (int j = 0; j < layers; ++j) {
        const double phi = phi0 + j * dphi;
        double ndcy = multi ? (std::tan(phi) / tanHalfV) : 0.0;
        if (ndcy < -1.0) ndcy = -1.0; else if (ndcy > 1.0) ndcy = 1.0;
        double fy = (0.5 - 0.5 * ndcy) * (wfH - 1);  // ndc_y +1 → row 0 (top)
        if (fy < 0.0) fy = 0.0; else if (fy > wfH - 1) fy = wfH - 1;
        int y0 = static_cast<int>(std::floor(fy));
        if (y0 > wfH - 1) y0 = wfH - 1;
        int y1 = y0 + 1 > wfH - 1 ? wfH - 1 : y0 + 1;
        const double ty = fy - y0;
        for (int i = 0; i < res; ++i) {
          const double theta = theta0 + i * dtheta;
          int fk = static_cast<int>(std::floor((fov / 2.0 - theta) / fovPer));
          if (fk < 0) fk = 0; else if (fk > nFrustums - 1) fk = nFrustums - 1;
          if (fk != k)
            continue;
          // lidar +θ = left (+y) → camera-left → NDC x < 0 (see OmWgpuSceneRenderer
          // basis swap); hence the leading minus. Local azimuth within frustum k.
          double ndcx = -std::tan(theta - thetaC) / tanHalfPer;
          if (ndcx < -1.0) ndcx = -1.0; else if (ndcx > 1.0) ndcx = 1.0;
          double fx = (0.5 + 0.5 * ndcx) * (wfW - 1);
          int x0 = static_cast<int>(std::floor(fx));
          if (x0 < 0) x0 = 0; else if (x0 > wfW - 1) x0 = wfW - 1;
          int x1 = x0 + 1 > wfW - 1 ? wfW - 1 : x0 + 1;
          const double tx = fx - x0;
          const float r00 = persp[static_cast<size_t>(y0) * wfW + x0];
          const float r01 = persp[static_cast<size_t>(y0) * wfW + x1];
          const float r10 = persp[static_cast<size_t>(y1) * wfW + x0];
          const float r11 = persp[static_cast<size_t>(y1) * wfW + x1];
          // F2: edge-aware resample -- no blending across depth discontinuities (see lidarResample4)
          float r = lidarResample4(r00, r01, r10, r11, tx, ty, 0.05f * hi);
          r = lidarRangeOrInf(r, lo, hi);  // F2.4: +inf outside (minRange, maxRange)
          data[j * res + i] = r;
        }
      }
    }
    static thread_local int sWfLog = 0;
    if (sWfLog < 4) {
      const int cx = res / 2;
      const int cl = layers / 2;
      OmLog::info(QString("[OmLidar] '%1' range via wgpu (%2 layers x %3, fov %4 rad, %5 frustums) "
                          "center=%6 m")
                    .arg(deviceName())
                    .arg(layers)
                    .arg(res)
                    .arg(fov, 0, 'f', 3)
                    .arg(nFrustums)
                    .arg(data[cl * res + cx], 0, 'f', 4));
      ++sWfLog;
    }
    return true;
  }

  // Multi-layer (L>1): render a perspective whose vertFov == the lidar's
  // verticalFieldOfView() (the method updatePointCloud derives φ from), then
  // 2D-bilinear-resample uniform-angle (θ_i, φ_j) into data[layer*res+col].
  // Orientation: ndc_y=+1 → texture row 0 (top), and φ_0 = +vfov/2 is layer 0,
  // so layer 0 = top — matching updatePointCloud. Single-layer (L==1) falls
  // through to the simpler centre-row path below (kept byte-identical).
  if (layers > 1) {
    if (fov > 1.4)
      return false;  // multi-layer + wide-FOV (multi-frustum) deferred → WREN
    const double vfov = verticalFieldOfView();  // == fov * height()/width()
    if (vfov <= 1e-6)
      return false;
    const double tanHalfH = std::tan(fov * 0.5);
    const double tanHalfV = std::tan(vfov * 0.5);
    const double renderAspect = tanHalfH / tanHalfV;  // → buildViewProj vertFov == vfov
    const int mlRenderW = res;
    const int mlRenderH = layers * 8 > 64 ? layers * 8 : 64;  // ≥ layers for vertical density
    if (!OmWgpuSceneRenderer::ensureTarget(back, mlRenderW, mlRenderH, mWgpuTarget, mWgpuMeshCache,
                                           mWgpuTargetWidth, mWgpuTargetHeight))
      return false;
    std::vector<OmWgpuSolidDraw> mlDraws;
    std::vector<std::array<float, 16>> mlModelStorage;
    mlModelStorage.reserve(64);
    mlDraws.reserve(64);
    collectWgpuDraws(*mWgpuMeshCache, mlDraws, mlModelStorage);
    const double mlFar = mMaxRange->value();
    float mlVP[16] = {0}, mlView[16] = {0};
    const OmMatrix4 mlCam = hasTilt ? (matrix() * tiltRot) : matrix();
    OmWgpuSceneRenderer::buildViewProj(mlCam, fov, renderAspect, mNear->value(), mlFar, mlVP);
    OmWgpuSceneRenderer::buildView(mlCam, mlView);
    std::vector<float> persp(static_cast<size_t>(mlRenderW) * static_cast<size_t>(mlRenderH),
                             static_cast<float>(mlFar));
    if (!mWgpuTarget->clearAndDrawSceneRangeF32(static_cast<float>(mlFar), mlVP, mlView,
                                                mlDraws.empty() ? nullptr : mlDraws.data(),
                                                static_cast<uint32_t>(mlDraws.size()), persp.data()))
      return false;
    const double dtheta = -fov / static_cast<double>(res);
    const double theta0 = fov / 2.0 + dtheta / 2.0;
    const double dphi = -vfov / static_cast<double>(layers - 1);
    const double phi0 = vfov / 2.0;  // layer 0 = +vfov/2 (top); tilt == 0 (gated)
    const float lo = static_cast<float>(mMinRange->value());
    const float hi = static_cast<float>(mlFar);
    for (int j = 0; j < layers; ++j) {
      const double phi = phi0 + j * dphi;
      double ndcy = std::tan(phi) / tanHalfV;
      if (ndcy < -1.0) ndcy = -1.0; else if (ndcy > 1.0) ndcy = 1.0;
      double fy = (0.5 - 0.5 * ndcy) * (mlRenderH - 1);  // ndc_y +1 → row 0 (top)
      if (fy < 0.0) fy = 0.0; else if (fy > mlRenderH - 1) fy = mlRenderH - 1;
      int y0 = static_cast<int>(std::floor(fy));
      if (y0 > mlRenderH - 1) y0 = mlRenderH - 1;
      int y1 = y0 + 1 > mlRenderH - 1 ? mlRenderH - 1 : y0 + 1;
      const double ty = fy - y0;
      for (int i = 0; i < res; ++i) {
        const double theta = theta0 + i * dtheta;
        double ndcx = -std::tan(theta) / tanHalfH;  // lidar +θ = left → NDC x < 0
        if (ndcx < -1.0) ndcx = -1.0; else if (ndcx > 1.0) ndcx = 1.0;
        double fx = (0.5 + 0.5 * ndcx) * (mlRenderW - 1);
        int x0 = static_cast<int>(std::floor(fx));
        if (x0 < 0) x0 = 0; else if (x0 > mlRenderW - 1) x0 = mlRenderW - 1;
        int x1 = x0 + 1 > mlRenderW - 1 ? mlRenderW - 1 : x0 + 1;
        const double tx = fx - x0;
        const float r00 = persp[static_cast<size_t>(y0) * mlRenderW + x0];
        const float r01 = persp[static_cast<size_t>(y0) * mlRenderW + x1];
        const float r10 = persp[static_cast<size_t>(y1) * mlRenderW + x0];
        const float r11 = persp[static_cast<size_t>(y1) * mlRenderW + x1];
        // F2: edge-aware resample -- no blending across depth discontinuities (see lidarResample4)
        float r = lidarResample4(r00, r01, r10, r11, tx, ty, 0.05f * hi);
        r = lidarRangeOrInf(r, lo, hi);  // F2.4: +inf outside (minRange, maxRange)
        data[j * res + i] = r;
      }
    }
    static thread_local int sMlLog = 0;
    if (sMlLog < 4) {
      const int cx = res / 2;
      const int cl = layers / 2;
      OmLog::info(QString("[OmLidar] '%1' range via wgpu (%2 layers x %3, fov %4 vfov %5, "
                          "%6 draws) center=%7 m")
                    .arg(deviceName())
                    .arg(layers)
                    .arg(res)
                    .arg(fov, 0, 'f', 3)
                    .arg(vfov, 0, 'f', 3)
                    .arg(static_cast<int>(mlDraws.size()))
                    .arg(data[cl * res + cx], 0, 'f', 4));
      ++sMlLog;
    }
    return true;
  }

  // Render a square-ish perspective covering the horizontal FOV (aspect 1 →
  // vertFov == fov), so the centre row (elevation φ = tilt ≈ 0) sees the scene.
  const int renderW = res;
  const int renderH = res > 8 ? res / 4 : res;  // a horizontal band; centre row sampled
  if (!OmWgpuSceneRenderer::ensureTarget(back, renderW, renderH, mWgpuTarget, mWgpuMeshCache,
                                         mWgpuTargetWidth, mWgpuTargetHeight))
    return false;

  std::vector<OmWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  modelStorage.reserve(64);
  draws.reserve(64);
  collectWgpuDraws(*mWgpuMeshCache, draws, modelStorage);

  const double aspect = static_cast<double>(renderW) / static_cast<double>(renderH);
  const double farVal = mMaxRange->value();
  float viewProj[16] = {0}, viewMat[16] = {0};
  const OmMatrix4 slCam = hasTilt ? (matrix() * tiltRot) : matrix();
  OmWgpuSceneRenderer::buildViewProj(slCam, fov, aspect, mNear->value(), farVal, viewProj);
  OmWgpuSceneRenderer::buildView(slCam, viewMat);

  std::vector<float> persp(static_cast<size_t>(renderW) * static_cast<size_t>(renderH),
                           static_cast<float>(farVal));
  if (!mWgpuTarget->clearAndDrawSceneRangeF32(static_cast<float>(farVal), viewProj, viewMat,
                                              draws.empty() ? nullptr : draws.data(),
                                              static_cast<uint32_t>(draws.size()), persp.data()))
    return false;

  // Angular resample: uniform-angle θ_i (matching updatePointCloud) → uniform-tan
  // perspective column. Sample the centre row (φ ≈ 0), bilinear in x. The
  // perspective vertFov == fov here, so ndc_x = tan(θ)/tan(fov/2).
  const double dtheta = -fov / static_cast<double>(res);
  const double theta0 = fov / 2.0 + dtheta / 2.0;  // updatePointCloud convention
  const double tanHalf = std::tan(fov * 0.5);
  const int row = renderH / 2;  // centre row ≈ elevation 0
  const float lo = static_cast<float>(mMinRange->value());
  const float hi = static_cast<float>(farVal);
  for (int i = 0; i < res; ++i) {
    const double theta = theta0 + i * dtheta;
    double ndcx = -std::tan(theta) / tanHalf;  // lidar +θ = left → NDC x < 0
    if (ndcx < -1.0) ndcx = -1.0; else if (ndcx > 1.0) ndcx = 1.0;
    const double fx = (ndcx * 0.5 + 0.5) * (renderW - 1);  // pixel coord
    int x0 = static_cast<int>(std::floor(fx));
    if (x0 < 0) x0 = 0; else if (x0 > renderW - 1) x0 = renderW - 1;
    int x1 = x0 + 1 > renderW - 1 ? renderW - 1 : x0 + 1;
    const double t = fx - x0;
    const float r0 = persp[static_cast<size_t>(row) * renderW + x0];
    const float r1 = persp[static_cast<size_t>(row) * renderW + x1];
    float r = lidarResample2(r0, r1, t, 0.05f * hi);  // F2: edge-aware resample
    r = lidarRangeOrInf(r, lo, hi);  // F2.4: +inf outside (minRange, maxRange)
    data[i] = r;
  }

  static thread_local int sLidarLog = 0;
  if (sLidarLog < 4) {
    const int cx = res / 2;
    OmLog::info(QString("[OmLidar] '%1' range via wgpu (1 layer x %2, fov %3 rad, %4 draws) "
                        "center=%5 m")
                  .arg(deviceName())
                  .arg(res)
                  .arg(fov, 0, 'f', 3)
                  .arg(static_cast<int>(draws.size()))
                  .arg(data[cx], 0, 'f', 4));
    ++sLidarLog;
  }
  return true;
}

// R5j/R5k: render the rotating-head instantaneous fov window into `tempImage`
// (height()×width()), at the sensor pose yawed by mPreviousRotatingAngle
// (+ tilt). This is the wgpu replacement for mWrenCamera->copyContentsToMemory
// in the rotating branch; the downstream shared band-copy + widthOffset +
// updatePointCloud consume tempImage with the SAME per-(row,column) convention
// as the non-rotating window, so the yaw lives entirely in the render camera and
// nothing downstream changes. R5j: single-layer (centre row). R5k: multi-layer
// (2D elevation resample of height() rows). Narrow FOV only (single frustum) —
// returns false (→ WREN fallback) for wide-FOV or any wgpu miss, so the default
// path is byte-untouched.
bool OmLidar::renderRotatingWindowViaWgpu(float *tempImage) {
  if (!tempImage || !hasBeenSetup())
    return false;
  const int layers = actualNumberOfLayers();
  const double fov = actualFieldOfView();
  const int winW = width();  // instantaneous window = ceil(hres * fov / 2pi)
  // Single-layer uses a centre-row sample; multi-layer (layers > 1) 2D-resamples
  // height() camera rows by elevation (the R5f composition). Wide-FOV rotating
  // (multi-frustum) is still deferred to WREN.
  // Narrow-FOV only this increment (a single perspective frustum spans the
  // window). Wide-FOV (multi-frustum) rotating is a later pass -> WREN fallback.
  // Multi-layer (layers > 1) is handled by the 2D-resample branch below.
  if (winW < 1 || fov <= 0.0)
    return false;
  // Wide FOV (> 1.4 rad): a single perspective frustum can't span the window, so
  // dispatch to the multi-frustum rotating path (R5l). The narrow single-frustum
  // path below (R5j/R5k, WREN-parity verified) stays byte-untouched.
  if (fov > 1.4)
    return renderRotatingWideFovWindowViaWgpu(tempImage);

  OmRenderBackend *back = OmRenderBackendRegistry::resolve(OmRenderBackendKind::Vulkan);
  if (!back || !back->isAvailable())
    return false;

  // Render camera = sensor world pose YAWED about the sensor up-axis (+z) by the
  // PREVIOUS rotating angle (then pitched by tilt; identity when tilt==0). Why
  // prevAngle and not currAngle: the shared band-copy below files the freshly-
  // swept window centered at the global column for mPreviousRotatingAngle
  // (widthOffset is computed from prevAngle), so the rendered window content must
  // be centered on that same heading or it lands ~one-step (≈w/2 columns) off —
  // the exact bug the partial-sweep exact-column test caught (measured -12 cols
  // ≈ -angle·res/2π). Rendering at prevAngle removes that offset.
  const bool hasTilt = mCurrentTiltAngle != 0.0;
  const OmMatrix4 yawRot = OmMatrix4(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, mPreviousRotatingAngle);
  const OmMatrix4 tiltRot = OmMatrix4(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -mCurrentTiltAngle);
  OmMatrix4 cam = matrix() * yawRot;
  if (hasTilt)
    cam = cam * tiltRot;

  // Perspective covering the window FOV. Single-layer: square-ish, centre row
  // sampled (R5j). Multi-layer (R5k): vertFov == verticalFieldOfView() via
  // aspect = tanHalfH/tanHalfV, then 2D-resample height() camera rows by
  // elevation — the R5f composition, written into the rotating window's
  // height()×winW layout the shared band-copy consumes (it reads camera row
  // (int)(i·skip) for layer i, so row r holds elevation φ(r)).
  const bool multi = layers > 1;
  const double tanHalf = std::tan(fov * 0.5);
  const double vfov = multi ? verticalFieldOfView() : 0.0;
  if (multi && vfov <= 1e-6)
    return false;
  const double tanHalfV = multi ? std::tan(vfov * 0.5) : 0.0;
  const int H = height();  // rows in tempImage (1 for single-layer)
  const int renderW = winW > 8 ? winW : 8;
  const int renderH = multi ? (H * 8 > 64 ? H * 8 : 64)
                            : (renderW > 8 ? renderW / 4 : renderW);
  if (!OmWgpuSceneRenderer::ensureTarget(back, renderW, renderH, mWgpuTarget, mWgpuMeshCache,
                                         mWgpuTargetWidth, mWgpuTargetHeight))
    return false;

  std::vector<OmWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  modelStorage.reserve(64);
  draws.reserve(64);
  collectWgpuDraws(*mWgpuMeshCache, draws, modelStorage);

  // Multi-layer: aspect makes the render's vertFov == vfov (so ndc_y maps to
  // elevation). Single-layer: square-ish aspect, vertical doesn't matter (centre
  // row only). Both keep the yaw + tilt entirely in `cam`.
  const double aspect = multi ? (tanHalf / tanHalfV)
                              : (static_cast<double>(renderW) / static_cast<double>(renderH));
  const double farVal = mMaxRange->value();
  float viewProj[16] = {0}, viewMat[16] = {0};
  OmWgpuSceneRenderer::buildViewProj(cam, fov, aspect, mNear->value(), farVal, viewProj);
  OmWgpuSceneRenderer::buildView(cam, viewMat);

  std::vector<float> persp(static_cast<size_t>(renderW) * static_cast<size_t>(renderH),
                           static_cast<float>(farVal));
  if (!mWgpuTarget->clearAndDrawSceneRangeF32(static_cast<float>(farVal), viewProj, viewMat,
                                              draws.empty() ? nullptr : draws.data(),
                                              static_cast<uint32_t>(draws.size()), persp.data()))
    return false;

  // Window column → local azimuth, IDENTICAL convention to the non-rotating
  // window (col 0 = +fov/2 = left = +theta; col winW-1 = −fov/2 = right):
  // ndc_x = −tan(theta)/tan(fov/2). The yaw is entirely in the camera, so this
  // is unchanged from the static path. Multi-layer adds the per-row elevation
  // φ(r) = vfov/2 − r·vfov/(H−1) (top-down, layer 0 = top), matching the static
  // multi-layer path + updatePointCloud; single-layer takes the centre row.
  const double dtheta = -fov / static_cast<double>(winW);
  const double theta0 = fov / 2.0 + dtheta / 2.0;
  const float lo = static_cast<float>(mMinRange->value());
  const float hi = static_cast<float>(farVal);
  for (int r = 0; r < H; ++r) {
    double fy;
    if (multi) {
      const double phi = vfov / 2.0 - r * vfov / static_cast<double>(H - 1);
      double ndcy = std::tan(phi) / tanHalfV;
      if (ndcy < -1.0) ndcy = -1.0; else if (ndcy > 1.0) ndcy = 1.0;
      fy = (0.5 - 0.5 * ndcy) * (renderH - 1);  // ndc_y +1 → row 0 (top)
    } else {
      fy = renderH / 2.0;  // centre row ≈ elevation 0 (+ tilt in camera)
    }
    if (fy < 0.0) fy = 0.0; else if (fy > renderH - 1) fy = renderH - 1;
    int y0 = static_cast<int>(std::floor(fy));
    if (y0 > renderH - 1) y0 = renderH - 1;
    int y1 = y0 + 1 > renderH - 1 ? renderH - 1 : y0 + 1;
    const double ty = fy - y0;
    for (int col = 0; col < winW; ++col) {
      const double theta = theta0 + col * dtheta;
      double ndcx = -std::tan(theta) / tanHalf;  // lidar +θ = left → NDC x < 0
      if (ndcx < -1.0) ndcx = -1.0; else if (ndcx > 1.0) ndcx = 1.0;
      const double fx = (ndcx * 0.5 + 0.5) * (renderW - 1);
      int x0 = static_cast<int>(std::floor(fx));
      if (x0 < 0) x0 = 0; else if (x0 > renderW - 1) x0 = renderW - 1;
      int x1 = x0 + 1 > renderW - 1 ? renderW - 1 : x0 + 1;
      const double tx = fx - x0;
      const float r00 = persp[static_cast<size_t>(y0) * renderW + x0];
      const float r01 = persp[static_cast<size_t>(y0) * renderW + x1];
      const float r10 = persp[static_cast<size_t>(y1) * renderW + x0];
      const float r11 = persp[static_cast<size_t>(y1) * renderW + x1];
      // F2: edge-aware resample -- no blending across depth discontinuities (see lidarResample4)
      float rng = lidarResample4(r00, r01, r10, r11, tx, ty, 0.05f * hi);
      rng = lidarRangeOrInf(rng, lo, hi);  // F2.4: +inf outside (minRange, maxRange)
      tempImage[static_cast<size_t>(r) * winW + col] = rng;
    }
  }

  static thread_local int sRotLog = 0;
  if (sRotLog < 6) {
    const size_t midRow = static_cast<size_t>(H / 2);
    OmLog::info(QString("[OmLidar] '%1' ROTATING window via wgpu (%2 layers x winW=%3, fov %4 rad, "
                        "yawPrev=%5 rad, %6 draws) midLayerColMid=%7")
                  .arg(deviceName())
                  .arg(layers)
                  .arg(winW)
                  .arg(fov, 0, 'f', 3)
                  .arg(mPreviousRotatingAngle, 0, 'f', 3)
                  .arg(static_cast<int>(draws.size()))
                  .arg(tempImage[midRow * winW + winW / 2], 0, 'f', 4));
    ++sRotLog;
  }
  return true;
}

// R5l: wide-FOV rotating-head window. Composes the R5g multi-frustum azimuth
// stitch with the R5j/R5k rotating window: split the window's `fov` azimuth span
// into N = ceil(fov/1.2) sub-frustums, render each at the sensor pose yawed by
// mPreviousRotatingAngle THEN rotated by the per-frustum centre θ_k (both about
// +z, so they compose to Rz(prevAngle + θ_k)), + tilt, and stitch by azimuth
// into tempImage's height()×winW layout (per-row elevation φ(r) as in R5k). The
// downstream shared band-copy then consumes it byte-unchanged. Narrow FOV is
// handled by renderRotatingWindowViaWgpu's single-frustum path; this is only
// reached for fov > 1.4.
bool OmLidar::renderRotatingWideFovWindowViaWgpu(float *tempImage) {
  if (!tempImage || !hasBeenSetup())
    return false;
  const int layers = actualNumberOfLayers();
  const double fov = actualFieldOfView();
  const int winW = width();
  if (winW < 1 || fov <= 1.4)
    return false;

  OmRenderBackend *back = OmRenderBackendRegistry::resolve(OmRenderBackendKind::Vulkan);
  if (!back || !back->isAvailable())
    return false;

  const bool multi = layers > 1;
  const double vfov = multi ? verticalFieldOfView() : 0.0;
  if (multi && vfov <= 1e-6)
    return false;
  const double tanHalfV = multi ? std::tan(vfov * 0.5) : 0.0;
  const int H = height();  // rows in tempImage (1 for single-layer)

  // Multi-frustum split of the window azimuth span (same as static R5g).
  const double kFrustumFov = 1.2;
  const int nFrustums = static_cast<int>(std::ceil(fov / kFrustumFov));
  const double fovPer = fov / nFrustums;
  const double tanHalfPer = std::tan(fovPer * 0.5);
  // Per-frustum vertFov: multi-layer → vfov (elevation maps to ndc_y); single →
  // square-ish (fovPer) since only the centre row is sampled.
  const double perVfov = multi ? vfov : fovPer;
  const double tanHalfPerV = std::tan(perVfov * 0.5);
  const double renderAspect = tanHalfPer / tanHalfPerV;
  const int renderW = (winW / nFrustums) + 8 > 32 ? (winW / nFrustums) + 8 : 32;
  const int renderH = multi ? (H * 8 > 64 ? H * 8 : 64) : renderW;
  if (!OmWgpuSceneRenderer::ensureTarget(back, renderW, renderH, mWgpuTarget, mWgpuMeshCache,
                                         mWgpuTargetWidth, mWgpuTargetHeight))
    return false;

  std::vector<OmWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  modelStorage.reserve(64);
  draws.reserve(64);
  collectWgpuDraws(*mWgpuMeshCache, draws, modelStorage);

  const bool hasTilt = mCurrentTiltAngle != 0.0;
  const OmMatrix4 tiltRot = OmMatrix4(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -mCurrentTiltAngle);
  const double farVal = mMaxRange->value();
  // Window column → azimuth, same convention as the narrow rotating window
  // (col 0 = +fov/2 = left; col winW-1 = −fov/2). The yaw lives in the camera.
  const double dtheta = -fov / static_cast<double>(winW);
  const double theta0 = fov / 2.0 + dtheta / 2.0;
  const double dphi = multi ? (-vfov / static_cast<double>(H - 1)) : 0.0;
  const double phi0 = multi ? (vfov / 2.0) : 0.0;
  const float lo = static_cast<float>(mMinRange->value());
  const float hi = static_cast<float>(farVal);

  for (int k = 0; k < nFrustums; ++k) {
    const double thetaC = fov / 2.0 - fovPer * (k + 0.5);
    // Yaw by prevAngle AND the frustum centre θ_k, both about +z → compose.
    OmMatrix4 camK = matrix() * OmMatrix4(0, 0, 0, 0, 0, 1, mPreviousRotatingAngle + thetaC);
    if (hasTilt)
      camK = camK * tiltRot;
    float vp[16] = {0}, vm[16] = {0};
    OmWgpuSceneRenderer::buildViewProj(camK, fovPer, renderAspect, mNear->value(), farVal, vp);
    OmWgpuSceneRenderer::buildView(camK, vm);
    std::vector<float> persp(static_cast<size_t>(renderW) * static_cast<size_t>(renderH),
                             static_cast<float>(farVal));
    if (!mWgpuTarget->clearAndDrawSceneRangeF32(static_cast<float>(farVal), vp, vm,
                                                draws.empty() ? nullptr : draws.data(),
                                                static_cast<uint32_t>(draws.size()), persp.data()))
      return false;
    for (int r = 0; r < H; ++r) {
      double fy;
      if (multi) {
        const double phi = phi0 + r * dphi;
        double ndcy = std::tan(phi) / tanHalfPerV;
        if (ndcy < -1.0) ndcy = -1.0; else if (ndcy > 1.0) ndcy = 1.0;
        fy = (0.5 - 0.5 * ndcy) * (renderH - 1);
      } else {
        fy = renderH / 2.0;
      }
      if (fy < 0.0) fy = 0.0; else if (fy > renderH - 1) fy = renderH - 1;
      int y0 = static_cast<int>(std::floor(fy));
      if (y0 > renderH - 1) y0 = renderH - 1;
      int y1 = y0 + 1 > renderH - 1 ? renderH - 1 : y0 + 1;
      const double ty = fy - y0;
      for (int col = 0; col < winW; ++col) {
        const double theta = theta0 + col * dtheta;
        int fk = static_cast<int>(std::floor((fov / 2.0 - theta) / fovPer));
        if (fk < 0) fk = 0; else if (fk > nFrustums - 1) fk = nFrustums - 1;
        if (fk != k)
          continue;
        double ndcx = -std::tan(theta - thetaC) / tanHalfPer;  // +θ = left → ndc_x < 0
        if (ndcx < -1.0) ndcx = -1.0; else if (ndcx > 1.0) ndcx = 1.0;
        const double fx = (0.5 + 0.5 * ndcx) * (renderW - 1);
        int x0 = static_cast<int>(std::floor(fx));
        if (x0 < 0) x0 = 0; else if (x0 > renderW - 1) x0 = renderW - 1;
        int x1 = x0 + 1 > renderW - 1 ? renderW - 1 : x0 + 1;
        const double tx = fx - x0;
        const float r00 = persp[static_cast<size_t>(y0) * renderW + x0];
        const float r01 = persp[static_cast<size_t>(y0) * renderW + x1];
        const float r10 = persp[static_cast<size_t>(y1) * renderW + x0];
        const float r11 = persp[static_cast<size_t>(y1) * renderW + x1];
        // F2: edge-aware resample -- no blending across depth discontinuities (see lidarResample4)
        float rng = lidarResample4(r00, r01, r10, r11, tx, ty, 0.05f * hi);
        rng = lidarRangeOrInf(rng, lo, hi);  // F2.4: +inf outside (minRange, maxRange)
        tempImage[static_cast<size_t>(r) * winW + col] = rng;
      }
    }
  }

  static thread_local int sWideRotLog = 0;
  if (sWideRotLog < 6) {
    const size_t midRow = static_cast<size_t>(H / 2);
    OmLog::info(QString("[OmLidar] '%1' WIDE ROTATING window via wgpu (%2 layers x winW=%3, "
                        "fov %4 rad, %5 frustums, yawPrev=%6) midLayerColMid=%7")
                  .arg(deviceName())
                  .arg(layers)
                  .arg(winW)
                  .arg(fov, 0, 'f', 3)
                  .arg(nFrustums)
                  .arg(mPreviousRotatingAngle, 0, 'f', 3)
                  .arg(tempImage[midRow * winW + winW / 2], 0, 'f', 4));
    ++sWideRotLog;
  }
  return true;
}

void OmLidar::preFinalize() {
  OmAbstractCamera::preFinalize();

  OmBaseNode *const e = dynamic_cast<OmBaseNode *>(mRotatingHead->value());
  if (e && !e->isPreFinalizedCalled())
    e->preFinalize();

  updateNear();
  updateMinRange();
  updateMaxRange();
  updateResolution();
  updateType();
  updateMinFrequency();
  updateMaxFrequency();
  updateDefaultFrequency();
  updateHorizontalResolution();
  updateVerticalFieldOfView();
  updateNumberOfLayers();
}

void OmLidar::postFinalize() {
  OmAbstractCamera::postFinalize();

  OmBaseNode *const e = dynamic_cast<OmBaseNode *>(mRotatingHead->value());
  if (e && !e->isPostFinalizedCalled())
    e->postFinalize();

  connect(mNear, &OmSFDouble::changed, this, &OmLidar::updateNear);
  connect(mMinRange, &OmSFDouble::changed, this, &OmLidar::updateMinRange);
  connect(mMaxRange, &OmSFDouble::changed, this, &OmLidar::updateMaxRange);
  connect(mResolution, &OmSFDouble::changed, this, &OmLidar::updateResolution);
  connect(mTiltAngle, &OmSFDouble::changed, this, &OmLidar::updateTiltAngle);
  connect(mType, &OmSFString::changed, this, &OmLidar::updateType);
  connect(mMinFrequency, &OmSFDouble::changed, this, &OmLidar::updateMinFrequency);
  connect(mMaxFrequency, &OmSFDouble::changed, this, &OmLidar::updateMaxFrequency);
  connect(mDefaultFrequency, &OmSFDouble::changed, this, &OmLidar::updateDefaultFrequency);
  connect(mHorizontalResolution, &OmSFDouble::changed, this, &OmLidar::updateHorizontalResolution);
  connect(mVerticalFieldOfView, &OmSFDouble::changed, this, &OmLidar::updateVerticalFieldOfView);
  connect(mNumberOfLayers, &OmSFInt::changed, this, &OmLidar::updateNumberOfLayers);
  connect(mRotatingHead, &OmSFNode::changed, this, &OmLidar::updateRotatingHead);
}

void OmLidar::reset(const QString &id) {
  OmAbstractCamera::reset(id);

  OmNode *const r = mRotatingHead->value();
  if (r)
    r->reset(id);

  mIsPointCloudEnabled = false;
  mCurrentRotatingAngle = 0;
  mPreviousRotatingAngle = 0;
  if (mTemporaryImage)
    memset(mTemporaryImage, 0, actualHorizontalResolution() * height() * sizeof(float));
}

void OmLidar::initializeImageMemoryMappedFile() {
  OmAbstractCamera::initializeImageMemoryMappedFile();
  if (mImageMemoryMappedFile) {
    // initialize the memory mapped file with a black image
    float *im = lidarImage();
    const int s = actualHorizontalResolution() * actualNumberOfLayers();
    for (int i = 0; i < s; i++)
      im[i] = 0.0f;
  }
  mTemporaryImage = new float[actualHorizontalResolution() * height()];
}

QString OmLidar::pixelInfo(int x, int y) const {
  // D1.4: the WREN window-texture readback is gone. Answer from the instantaneous fov
  // window buffer the render paths fill (mTemporaryImage, width() x height() floats) --
  // the same content the WREN texture held. The non-rotating wgpu path writes the lidar
  // image directly and leaves this buffer at 0, matching the previous wrenless behaviour
  // (which always answered 0).
  float depth = 0.0f;
  if (hasBeenSetup() && mTemporaryImage != NULL && x >= 0 && x < width() && y >= 0 && y < height())
    depth = mTemporaryImage[static_cast<size_t>(y) * width() + x];
  return QString::asprintf("depth(%d,%d)=%f", x, y, depth);
}

void OmLidar::prePhysicsStep(double ms) {
  OmSolid::prePhysicsStep(ms);
  OmSolid *s = solidEndPoint();
  if (mIsActuallyRotating && mSensor->isEnabled()) {
    double angle = -(ms * 2 * M_PI * mDefaultFrequency->value()) / 1000;
    if (s)
      s->rotate(OmVector3(0.0, 0.0, angle));
    if (hasBeenSetup()) {
      // D1.4: the wgpu window render derives its own yawed pose from these angles.
      mPreviousRotatingAngle = mCurrentRotatingAngle;
      mCurrentRotatingAngle += angle;
    }
  }
  if (s)
    s->prePhysicsStep(ms);
}

void OmLidar::postPhysicsStep() {
  OmSolid::postPhysicsStep();
  if (mIsActuallyRotating && mSensor->isEnabled())
    copyAllLayersToMemoryMappedFile();
}

void OmLidar::write(OmWriter &writer) const {
  if (writer.isOmniSim() || writer.isUrdf())
    OmBaseNode::write(writer);
  else
    writeExport(writer);
}

void OmLidar::exportNodeSubNodes(OmWriter &writer) const {
  OmAbstractCamera::exportNodeSubNodes(writer);
  if (writer.isOmniSim() || writer.isUrdf())
    return;

  const OmSolid *s = solidEndPoint();
  if (s)
    s->write(writer);
}

void OmLidar::addConfigureToStream(OmDataStream &stream, bool reconfigure) {
  OmAbstractCamera::addConfigureToStream(stream, reconfigure);
  stream << (double)mMaxRange->value();
  stream << (short)mNumberOfLayers->value();
  stream << (double)mDefaultFrequency->value();
  stream << (double)mMinFrequency->value();
  stream << (double)mMaxFrequency->value();
  stream << (double)mVerticalFieldOfView->value();
  stream << (double)actualHorizontalResolution();
}

void OmLidar::writeAnswer(OmDataStream &stream) {
  if (mImageChanged) {
    mImageChanged = false;  // prevent AbstractCamera from copying the whole content of the camera in the memory mapped file
    OmAbstractCamera::writeAnswer(stream);
    mSensor->resetPendingValue();
    if (!mIsActuallyRotating && mSensor->isEnabled())  // in case of rotating lidar, the copy is done during the step
      copyAllLayersToMemoryMappedFile();  // for non-rotating lidar, copy the layers needed in the memory mapped file
    if (mIsRemoteExternController) {
      const int lidarDataSize = actualHorizontalResolution() * actualNumberOfLayers();
      editChunkMetadata(stream, mIsPointCloudEnabled ? size() : sizeof(float) * lidarDataSize);

      // copy image to stream
      stream << (short unsigned int)tag();
      stream << (unsigned char)C_ABSTRACT_CAMERA_SERIAL_IMAGE;
      int streamLength = stream.length();
      stream.resize(lidarDataSize * sizeof(float) + streamLength);
      memcpy(stream.data() + streamLength, mTcpImage, lidarDataSize * sizeof(float));
      if (mIsPointCloudEnabled) {
        streamLength = stream.length();
        stream.resize(lidarDataSize * sizeof(WbLidarPoint) + streamLength);
        memcpy(stream.data() + streamLength, mTcpCloudPoints, lidarDataSize * sizeof(WbLidarPoint));
      }

      // prepare next chunk
      stream.mSizePtr = stream.length();
      stream << (int)0;
      stream << (unsigned char)0;
    }
  } else
    OmAbstractCamera::writeAnswer(stream);
}

void OmLidar::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;
  if (command == C_SET_SAMPLING_PERIOD) {
    stream >> mRefreshRate;
    if (mIsActuallyRotating)
      mRefreshRate = OmWorld::instance()->basicTimeStep();

    mSensor->setRefreshRate(mRefreshRate);

    emit enabled(this, mSensor->isEnabled());

    if (!hasBeenSetup()) {
      setup();
      mSendMemoryMappedFile = true;
    } else if (mHasExternControllerChanged) {
      mSendMemoryMappedFile = true;
      mHasExternControllerChanged = false;
    }

    return;
  } else if (command == C_LIDAR_ENABLE_POINT_CLOUD) {
    mIsPointCloudEnabled = true;
    mTcpCloudPoints =
      mIsRemoteExternController ? new WbLidarPoint[actualHorizontalResolution() * actualNumberOfLayers()] : NULL;
    return;
  } else if (command == C_LIDAR_DISABLE_POINT_CLOUD) {
    mIsPointCloudEnabled = false;
    if (mIsRemoteExternController)
      delete mTcpCloudPoints;
    return;
  } else if (command == C_LIDAR_SET_FREQUENCY) {
    double frequency;
    stream >> frequency;
    mDefaultFrequency->setValue(frequency);
    return;
  } else if (OmAbstractCamera::handleCommand(stream, command))
    return;

  assert(0);
}

void OmLidar::copyAllLayersToMemoryMappedFile() {
  if (!hasBeenSetup() || !mImageMemoryMappedFile)
    return;

  // R5e: wgpu range path (the whole R5e-R5l family), UNCONDITIONAL since D1.4 -- WREN is
  // deleted, so this is the only producer. Fills the lidar image with uniform-angle radial
  // ranges, then runs the standard point-cloud derivation. renderRangeViaWgpu returns
  // false for the rotating case (handled below) or on any wgpu miss.
  if (!mIsRemoteExternController && renderRangeViaWgpu(lidarImage())) {
    // Lane E1 (P5+P6): the WREN pipeline runs range_noise + depth_resolution between the
    // render and copyContentsToMemory, so `noise` and `resolution` were applied on WREN and
    // silently DROPPED here. Apply the CPU port on the same floats the controller reads.
    applyWgpuRangePostFx(lidarImage(), actualHorizontalResolution(), actualNumberOfLayers(), mResolution->value());
    if (mIsPointCloudEnabled)
      // D1.4: the WREN point-cloud/ray VISUALS died with WREN (the wgpu overlay
      // collectors draw them from this very point array); the DATA is unchanged.
      updatePointCloud(0, actualHorizontalResolution());
    return;
  }

  delete mTcpImage;
  mTcpImage = mIsRemoteExternController ? new float[actualHorizontalResolution() * actualNumberOfLayers()] : NULL;

  float *data = mIsRemoteExternController ? mTcpImage : lidarImage();
  double skip = 1.0;
  if (height() != actualNumberOfLayers() && actualNumberOfLayers() != 1)
    skip = (double)(height() - 1) / (double)(actualNumberOfLayers() - 1);
  double w = width();
  int resolution = actualHorizontalResolution();
  int minWidth = 0;
  int maxWidth = w;
  int widthOffset = 0;

  // R5j: rotating-head wgpu path. Fill mTemporaryImage (the instantaneous fov
  // window) via the wgpu radial-range render at the YAWED camera, then let the
  // SHARED band-copy + widthOffset + updatePointCloud below run byte-unchanged.
  // Rendering at mPreviousRotatingAngle (not current) matches the basis the
  // band-copy's widthOffset is computed from — the heading WREN's lagged GL
  // readback also reflects — so the window lands at the SAME global column.
  //
  // WREN-parity verified before the WREN deletion (lidar_wgpu_rotating_smoke
  // partial-sweep exact-column test + WREN-oracle column diff): off-axis box lands
  // in the exact predicted band, identical closest column to WREN (187==187), clean
  // front-face columns agree to <0.001 m (flank cols differ only by the same
  // sub-column resample tolerance as the static paths). D1.4: unconditional -- a
  // wgpu miss now means an honestly-empty image, warned below.
  bool windowFilledByWgpu = false;
  if (mIsActuallyRotating && !mIsRemoteExternController)
    windowFilledByWgpu = renderRotatingWindowViaWgpu(mTemporaryImage);
  if (windowFilledByWgpu) {
    // Lane E1 (P5+P6): same CPU port as the static path, applied to the instantaneous fov
    // window BEFORE the shared band-copy -- the same place in the chain where WREN's
    // range_noise/depth_resolution passes sat.
    applyWgpuRangePostFx(mTemporaryImage, width(), height(), mResolution->value());
  } else {
    // D1.4: the wgpu window render declined or failed and the WREN camera it used to fall
    // back to is deleted -- nothing produced pixels for this step. Say so (once) instead
    // of letting the controller read a frozen buffer.
    warnWrenlessNoImage();
    return;
  }
  // if rotating compute which part of the image should be updated
  if (mIsActuallyRotating) {
    double deltaAngle = fabs(mCurrentRotatingAngle - mPreviousRotatingAngle);
    double ratio = deltaAngle / actualFieldOfView();
    if (ratio > 1.0)
      ratio = 1.0;
    minWidth = ((double)w / 2.0) * (1.0 - ratio);
    maxWidth = ((double)w / 2.0) * (1.0 + ratio);

    double tmpAngle =
      (fabs((mPreviousRotatingAngle - M_PI) / (2.0 * M_PI)) - floor(fabs((mPreviousRotatingAngle - M_PI) / (2.0 * M_PI)))) *
      (2.0 * M_PI);
    if (tmpAngle < 0)
      tmpAngle += 2.0 * M_PI;
    widthOffset = resolution * (tmpAngle / (2.0 * M_PI));
    widthOffset -= w / 2;
  }

  for (int i = 0; i < actualNumberOfLayers(); ++i) {
    if ((maxWidth + widthOffset) <= resolution && (minWidth + widthOffset) >= 0)
      memcpy(data + i * resolution + minWidth + widthOffset, mTemporaryImage + width() * (int)(i * skip) + minWidth,
             sizeof(float) * (maxWidth - minWidth));
    else {  // we need two split into two because the current image is 'across' the lidar image (avoid overflow)
      if ((maxWidth + widthOffset) > resolution) {
        memcpy(data + i * resolution + minWidth + widthOffset, mTemporaryImage + width() * (int)(i * skip) + minWidth,
               sizeof(float) * (resolution - minWidth - widthOffset));
        memcpy(data + i * resolution, mTemporaryImage + width() * (int)(i * skip) + resolution - widthOffset,
               sizeof(float) * (maxWidth + widthOffset - resolution));
      } else {  // (minWidth + widthOffset) < 0
        memcpy(data + (i + 1) * resolution + minWidth + widthOffset, mTemporaryImage + width() * (int)(i * skip) + minWidth,
               sizeof(float) * abs(minWidth + widthOffset));
        memcpy(data + i * resolution, mTemporaryImage + width() * (int)(i * skip) - widthOffset,
               sizeof(float) * abs(maxWidth + widthOffset));
      }
    }
  }

  if (mIsPointCloudEnabled) {
    // D1.4: the WREN point-cloud/ray visuals died with WREN; the point DATA is unchanged.
    if ((maxWidth + widthOffset) <= resolution && (minWidth + widthOffset) >= 0)
      updatePointCloud(minWidth + widthOffset, widthOffset + maxWidth);
    else {  // we need two split into two because the current image is 'across' the lidar image (avoid overflow)
      if ((maxWidth + widthOffset) > resolution) {
        updatePointCloud(minWidth + widthOffset, resolution);
        updatePointCloud(0, maxWidth + widthOffset - resolution);
      } else {  // (minWidth + widthOffset) < 0
        updatePointCloud(resolution + minWidth + widthOffset,
                         resolution + minWidth + widthOffset + abs(minWidth + widthOffset));
        updatePointCloud(0, maxWidth + widthOffset);
      }
    }
  }
}

void OmLidar::updatePointCloud(int minWidth, int maxWidth) {
  WbLidarPoint *lidarPoints = pointArray();
  const float *image = mIsRemoteExternController ? mTcpImage : lidarImage();
  const int resolution = actualHorizontalResolution();
  const int numberOfLayers = actualNumberOfLayers();
  const double w = width();

  const double dt = -((double)mRefreshRate / 1000.0) / w;
  const double t0 = OmSimulationState::instance()->time() / 1000.0 + minWidth * dt;

  const double dphi = (numberOfLayers > 1) ? (-verticalFieldOfView() / (numberOfLayers - 1)) : 0.0;
  const double cosdPhi = cos(dphi);
  const double sindPhi = sin(dphi);
  const double phi0 = ((numberOfLayers > 1) ? (verticalFieldOfView() / 2) : 0.0) + mCurrentTiltAngle;
  const double cosPhi0 = cos(phi0);
  const double sinPhi0 = sin(phi0);

  const double dtheta = mIsActuallyRotating ? (-2 * M_PI / (double)resolution) : (-actualFieldOfView() / w);
  const double cosdTheta = cos(dtheta);
  const double sindTheta = sin(dtheta);
  const double theta0 =
    mIsActuallyRotating ? minWidth * dtheta - M_PI : actualFieldOfView() / 2 + minWidth * dtheta + dtheta / 2;
  const double cosTheta0 = cos(theta0);
  const double sinTheta0 = sin(theta0);

  // We use addition law on cos and sin to recursively compute them, avoiding the costly computation.
  // cos(x+dx) = cos(x)cos(dx)-sin(x)sin(dx)
  // sin(x+dx) = sin(x)cos(dx)+cos(x)sin(dx)

  double cosPhi = cosPhi0;
  double sinPhi = sinPhi0;
  for (int i = 0; i < numberOfLayers; ++i) {
    double t = t0;
    double cosTheta = cosTheta0;
    double sinTheta = sinTheta0;
    const int indexStart = resolution * i + minWidth;
    const int indexEnd = resolution * i + maxWidth;
    for (int index = indexStart; index < indexEnd; ++index) {
      const double r = image[index];
      lidarPoints[index].x = r * cosTheta * cosPhi;
      lidarPoints[index].y = r * sinTheta * cosPhi;
      lidarPoints[index].z = r * sinPhi;
      lidarPoints[index].time = t;
      lidarPoints[index].layer_id = i;
      t += dt;

      double cosTheta_tmp = cosTheta * cosdTheta - sinTheta * sindTheta;
      double sinTheta_tmp = sinTheta * cosdTheta + cosTheta * sindTheta;
      cosTheta = cosTheta_tmp;
      sinTheta = sinTheta_tmp;
    }
    double cosPhi_tmp = cosPhi * cosdPhi - sinPhi * sindPhi;
    double sinPhi_tmp = sinPhi * cosdPhi + cosPhi * sindPhi;
    cosPhi = cosPhi_tmp;
    sinPhi = sinPhi_tmp;
  }
}

float *OmLidar::lidarImage() const {
  return reinterpret_cast<float *>(image());
}

int OmLidar::height() const {
  if (actualNumberOfLayers() == 1)
    return 1;
  // as we want the center of the upper/lower pixel line to be aligned with the upper/lower layer we add 'actualFieldOfView() /
  // width()' to verticalFieldOfView
  return ceil((actualVerticalFieldOfView() + actualFieldOfView() / width()) * (width() / actualFieldOfView()));
}

int OmLidar::width() const {
  if (mIsActuallyRotating)
    return ceil(actualHorizontalResolution() * (actualFieldOfView() / (2.0 * M_PI)));
  return actualHorizontalResolution();
}

OmSolid *OmLidar::solidEndPoint() const {
  OmSolid *solid = dynamic_cast<OmSolid *>(mRotatingHead->value());
  if (solid)
    return solid;
  return NULL;
}

int OmLidar::actualNumberOfLayers() const {
  if (hasBeenSetup())
    return mActualNumberOfLayers;
  return mNumberOfLayers->value();
}

int OmLidar::actualHorizontalResolution() const {
  if (hasBeenSetup())
    return mActualHorizontalResolution;
  return mHorizontalResolution->value();
}

double OmLidar::actualVerticalFieldOfView() const {
  if (hasBeenSetup())
    return mActualVerticalFieldOfView;
  return mVerticalFieldOfView->value();
}

double OmLidar::actualFieldOfView() const {
  if (hasBeenSetup())
    return mActualFieldOfView;
  return mFieldOfView->value();
}

/////////////////////
//  Update methods //
/////////////////////

void OmLidar::updateNear() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.01))
    return;

  if (mNear->value() > mMinRange->value()) {
    parsingWarn(tr("'near' is greater than to 'minRange'. Setting 'near' to %1.").arg(mMinRange->value()));
    mNear->setValue(mMinRange->value());
    return;
  }
  // D1.4: the wgpu range paths read mNear live.
}

void OmLidar::updateMinRange() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mMinRange, 0.01))
    return;

  if (mMinRange->value() < mNear->value()) {
    parsingWarn(tr("'minRange' is less than 'near'. Setting 'minRange' to %1.").arg(mNear->value()));
    mMinRange->setValue(mNear->value());
    return;
  }

  if (mMinRange->value() >= mMaxRange->value()) {
    parsingWarn(tr("'minRange' is greater or equal to 'maxRange'. Setting 'maxRange' to %1.").arg(mMinRange->value() + 1.0));
    mMaxRange->setValue(mMinRange->value() + 1.0);
    return;
  }

  mNeedToConfigure = true;
  // D1.4: the WREN frustum visuals are gone (the wgpu overlay draws from engine state).
}

void OmLidar::updateMaxRange() {
  if (mMaxRange->value() <= mMinRange->value()) {
    double newMaxRange = mMinRange->value() + 1.0;
    parsingWarn(tr("'maxRange' is less or equal to 'minRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
    mMaxRange->setValue(newMaxRange);
    return;
  }

  mNeedToConfigure = true;
  // D1.4: the wgpu range paths read mMaxRange live; the WREN frustum visuals are gone.
}

void OmLidar::updateFieldOfView() {
  OmAbstractCamera::updateFieldOfView();

  // warn in case of width modification after the setup
  if (hasBeenSetup())
    warn(tr(
      "'fieldOfView' has been modified. This modification will be taken into account after saving and reloading the world."));
}

void OmLidar::updateResolution() {
  if (OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0))
    return;
  // D1.4: the wgpu paths read mResolution live in applyWgpuRangePostFx.
}

void OmLidar::updateTiltAngle() {
  // D1.4: the wgpu render paths apply the tilt themselves; latch the value they read
  // (formerly applyTiltAngleToWren's wrenless arm).
  if (hasBeenSetup())
    mCurrentTiltAngle = mTiltAngle->value();
}

void OmLidar::updateType() {
  if (mType->value().compare("fixed", Qt::CaseInsensitive) != 0 &&
      mType->value().compare("rotating", Qt::CaseInsensitive) != 0) {
    parsingWarn(tr("'type' should either be 'fixed' or 'rotating', reset to 'fixed'"));
    mType->setValue("fixed");
  }
  if (hasBeenSetup())
    warn(tr("'type' has been modified. This modification will be taken into account after saving and reloading the world."));
}

void OmLidar::updateMinFrequency() {
  OmFieldChecker::resetDoubleIfNonPositive(this, mMinFrequency, 0.01);
  if (mMinFrequency->value() > mMaxFrequency->value()) {
    parsingWarn(tr("'minFrequency' should be smaller or equal to 'maxFrequency'."));
    mMinFrequency->setValue(mMaxFrequency->value());
  }
  if (hasBeenSetup())
    mNeedToConfigure = true;
}

void OmLidar::updateMaxFrequency() {
  OmFieldChecker::resetDoubleIfNonPositive(this, mMaxFrequency, mMinFrequency->value());
  if (mMaxFrequency->value() < mMinFrequency->value()) {
    parsingWarn(tr("'maxFrequency' should be bigger or equal to 'minFrequency'."));
    mMaxFrequency->setValue(mMinFrequency->value());
  }
  if (hasBeenSetup())
    mNeedToConfigure = true;
}

void OmLidar::updateDefaultFrequency() {
  OmFieldChecker::resetDoubleIfNonPositive(this, mDefaultFrequency, mMinFrequency->value());
  if (mDefaultFrequency->value() < mMinFrequency->value()) {
    parsingWarn(tr("'defaultFrequency' should be bigger or equal to 'minFrequency'."));
    mDefaultFrequency->setValue(mMinFrequency->value());
  } else if (mDefaultFrequency->value() > mMaxFrequency->value()) {
    parsingWarn(tr("'defaultFrequency' should be bigger or equal to 'maxFrequency'."));
    mDefaultFrequency->setValue(mMaxFrequency->value());
  }
  if (hasBeenSetup())
    mNeedToConfigure = true;
}

void OmLidar::updateHorizontalResolution() {
  OmFieldChecker::resetIntIfNonPositive(this, mHorizontalResolution, 1);

  // make sure we have at least 1 pixel height per layer
  if (height() < actualNumberOfLayers()) {
    int requiredResolution = ceil((actualNumberOfLayers() * actualFieldOfView()) / verticalFieldOfView());
    if (mIsActuallyRotating) {
      requiredResolution *= 2.0 * M_PI / actualFieldOfView();
      parsingWarn(
        tr("Impossible to have a so small 'horizontalResolution' using this 'numberOfLayers' and 'verticalFieldOfView'. "
           "'horizontalResolution' should be bigger or equal to 2.0 * M_PI * numberOfLayers  / verticalFieldOfView. "
           "'horizontalResolution' set to %1.")
          .arg(requiredResolution));
    } else
      parsingWarn(
        tr("Impossible to have a so small 'horizontalResolution' using this 'fieldOfView', 'numberOfLayers' and "
           "'verticalFieldOfView'. 'horizontalResolution' should be bigger or equal to numberOfLayers * fieldOfView / "
           "verticalFieldOfView. 'horizontalResolution' set to %1.")
          .arg(requiredResolution));
    mHorizontalResolution->setValue(requiredResolution);
  }

  // warn in case of width modification after the setup
  if (hasBeenSetup())
    warn(tr("'horizontalResolution' has been modified. This modification will be taken into account after saving and reloading "
            "the world."));
}

void OmLidar::updateVerticalFieldOfView() {
  OmFieldChecker::resetDoubleIfNonPositive(this, mVerticalFieldOfView, 0.1);
  OmFieldChecker::resetDoubleIfGreater(this, mVerticalFieldOfView, M_PI, M_PI);

  // make sure we have at least 1 pixel height per layer
  if (height() < actualNumberOfLayers()) {
    double requiredVerticalFieldOfView = (actualNumberOfLayers() * actualFieldOfView()) / width();
    if (mIsActuallyRotating)
      parsingWarn(
        tr("Impossible to have a so small 'verticalFieldOfView' using this 'numberOfLayers' and 'horizontalResolution'. "
           "'verticalFieldOfView' should be bigger or equal to 2.0 * M_PI * numberOfLayers / horizontalResolution. "
           "'verticalFieldOfView' set to %1.")
          .arg(requiredVerticalFieldOfView));
    else
      parsingWarn(
        tr("Impossible to have a so small 'verticalFieldOfView' using this 'fieldOfView', 'numberOfLayers' and "
           "'horizontalResolution'. 'verticalFieldOfView' should be bigger or equal to numberOfLayers * fieldOfView / "
           "horizontalResolution. 'verticalFieldOfView' set to %1.")
          .arg(requiredVerticalFieldOfView));
    mVerticalFieldOfView->setValue(requiredVerticalFieldOfView);
  }

  // warn in case of width modification after the setup
  if (hasBeenSetup())
    warn(tr("'verticalFieldOfView' has been modified. This modification will be taken into account after saving and reloading "
            "the world."));
}

void OmLidar::updateNumberOfLayers() {
  OmFieldChecker::resetIntIfNonPositive(this, mNumberOfLayers, 1);

  // make sure we have at least 1 pixel height per layer
  if (height() < actualNumberOfLayers()) {
    int requiredNumberOfLayers = height();
    if (mIsActuallyRotating)
      parsingWarn(
        tr("Impossible to have a so big 'numberOfLayers' using this 'verticalFieldOfView' and 'horizontalResolution'. "
           "'numberOfLayers' should be smaller or equal to verticalFieldOfView * actualHorizontalResolution() / (2.0 * "
           "M_PI). 'numberOfLayers' set to %1.")
          .arg(requiredNumberOfLayers));
    else
      parsingWarn(tr("Impossible to have a so big 'numberOfLayers' using this 'fieldOfView', 'verticalFieldOfView' and "
                     "'horizontalResolution'. 'numberOfLayers' should be smaller or equal to verticalFieldOfView * "
                     "horizontalResolution / fieldOfView. 'numberOfLayers' set to %1.")
                    .arg(requiredNumberOfLayers));
    mNumberOfLayers->setValue(requiredNumberOfLayers);
  }

  // warn in case of width modification after the setup
  if (hasBeenSetup())
    warn(tr("'numberOfLayers' has been modified. This modification will be taken into account after saving and reloading the "
            "world."));
}

void OmLidar::updateRotatingHead() {
  const OmSolid *head = solidEndPoint();
  if (head && isPostFinalizedCalled()) {
    if (head->isPostFinalizedCalled())
      mBoundingSphere->addSubBoundingSphere(head->boundingSphere());
    else
      connect(head, &OmBaseNode::finalizationCompleted, this, &OmLidar::updateBoundingSphere);
  }
}

void OmLidar::updateBoundingSphere(OmBaseNode *subNode) {
  disconnect(subNode, &OmBaseNode::finalizationCompleted, this, &OmLidar::updateBoundingSphere);
  mBoundingSphere->addSubBoundingSphere(subNode->boundingSphere());
}

void OmLidar::createOdeObjects() {
  OmAbstractCamera::createOdeObjects();
  OmSolid *s = solidEndPoint();
  if (s)
    s->createOdeObjects();
}

void OmLidar::propagateSelection(bool selected) {
  OmAbstractCamera::propagateSelection(selected);
  OmSolid *solid = solidEndPoint();
  if (solid)
    solid->propagateSelection(selected);
}

void OmLidar::setMatrixNeedUpdate() {
  OmAbstractCamera::setMatrixNeedUpdate();
  OmSolid *s = solidEndPoint();
  if (s)
    s->setMatrixNeedUpdate();
}

void OmLidar::updateCollisionMaterial(bool triggerChange, bool onSelection) {
  OmAbstractCamera::updateCollisionMaterial(triggerChange, onSelection);
  OmSolid *s = solidEndPoint();
  if (s)
    s->updateCollisionMaterial(triggerChange, onSelection);
}

void OmLidar::setSleepMaterial() {
  OmAbstractCamera::setSleepMaterial();
  OmSolid *s = solidEndPoint();
  if (s)
    s->setSleepMaterial();
}

void OmLidar::setScaleNeedUpdate() {
  OmAbstractCamera::setScaleNeedUpdate();
  OmSolid *s = solidEndPoint();
  if (s)
    s->setScaleNeedUpdate();
}

void OmLidar::attachResizeManipulator() {
  OmAbstractCamera::attachResizeManipulator();
  OmSolid *s = solidEndPoint();
  if (s)
    s->attachResizeManipulator();
}

void OmLidar::detachResizeManipulator() const {
  OmAbstractCamera::detachResizeManipulator();
  const OmSolid *s = solidEndPoint();
  if (s)
    s->detachResizeManipulator();
}

void OmLidar::createWrenObjects() {
  // D1.4: the WREN frustum / point-cloud / ray visuals died with WREN (the wgpu overlay
  // collectors draw the lidar optional renderings from the point array + engine state).
  // Only the base call and the rotating-head child recursion remain.
  OmAbstractCamera::createWrenObjects();

  OmSolid *const s = solidEndPoint();
  if (s)
    s->createWrenObjects();
}
