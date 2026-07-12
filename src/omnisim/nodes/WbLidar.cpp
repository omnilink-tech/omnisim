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

#include "WbLidar.hpp"

#include "WbBoundingSphere.hpp"
#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbPerspective.hpp"
#include "WbRgb.hpp"
#include "WbSensor.hpp"
#include "WbSimulationState.hpp"
#include "WbWorld.hpp"
#include "WbWrenCamera.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

// R5e: wgpu range path for the Lidar device node.
#include "WbLog.hpp"
#include "WbMatrix4.hpp"
#include "WbRenderBackend.hpp"
#include "WbWgpuMeshCache.hpp"
#include "WbWgpuRenderTarget.hpp"
#include "WbWgpuSceneRenderer.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QtGlobal>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <wren/config.h>
#include <wren/dynamic_mesh.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#define POINT_CLOUD_RAY_REPRESENTATION_THRESHOLD 2500

void WbLidar::init() {
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

  mFrustumMesh = NULL;
  mFrustumRenderable = NULL;
  mFrustumMaterial = NULL;

  mLidarPointsRenderable = NULL;
  mLidarPointsMesh = NULL;
  mLidarPointsMaterial = NULL;

  mLidarRaysRenderable = NULL;
  mLidarRaysMesh = NULL;
  mLidarRaysMaterial = NULL;

  mActualNumberOfLayers = mNumberOfLayers->value();
  mActualHorizontalResolution = mHorizontalResolution->value();
  mActualVerticalFieldOfView = mVerticalFieldOfView->value();
  mActualFieldOfView = mFieldOfView->value();
  mIsActuallyRotating = mType->value().startsWith('r', Qt::CaseInsensitive);

  mTcpImage = NULL;
  mTcpCloudPoints = NULL;

  // R5e: wgpu range target stays null until the first copyAllLayersToMemory-
  // MappedFile with OMNISIM_LIDAR_WGPU=1 and a usable Vulkan backend.
  mWgpuTarget = NULL;
  mWgpuMeshCache = NULL;
  mWgpuTargetWidth = 0;
  mWgpuTargetHeight = 0;

  // backward compatibility
  WbSFBool *sphericalField = findSFBool("spherical");
  if (!sphericalField->value()) {  // Deprecated in Webots R2023
    parsingWarn("Deprecated 'spherical' field, please use the 'projection' field instead.");
    if (mProjection->value() == "cylindrical")
      mProjection->setValue("planar");
    sphericalField->setValue(true);
  }
}

WbLidar::WbLidar(WbTokenizer *tokenizer) : WbAbstractCamera("Lidar", tokenizer) {
  init();
}

WbLidar::WbLidar(const WbLidar &other) : WbAbstractCamera(other) {
  init();
}

WbLidar::WbLidar(const WbNode &other) : WbAbstractCamera(other) {
  init();
}

WbLidar::~WbLidar() {
  delete mTemporaryImage;
  if (mIsRemoteExternController) {
    if (mIsPointCloudEnabled)
      delete mTcpCloudPoints;
    delete mTcpImage;
  }
  if (areWrenObjectsInitialized())
    deleteWren();
  // R5e: same teardown order as the Camera/RangeFinder wgpu paths.
  delete mWgpuMeshCache;
  mWgpuMeshCache = NULL;
  delete mWgpuTarget;
  mWgpuTarget = NULL;
}

// R5e: render the single-layer, non-rotating, narrow-FOV Lidar range scan via
// the wgpu radial-range pipeline, angular-resampling the perspective image into
// the uniform-angle layout updatePointCloud expects. Returns false (caller keeps
// WREN) for any config outside that subset, or on any wgpu failure. The default
// WREN path is untouched. See engine-migration-plan.md R5 WbLidar spec.
bool WbLidar::renderRangeViaWgpu(float *data) {
  const int layers = actualNumberOfLayers();
  const int res = actualHorizontalResolution();
  const double fov = actualFieldOfView();
  // Gate: non-rotating, no tilt, FOV ≤ 2π. Narrow FOV (≤ 1.4 rad) uses a single
  // perspective frustum (single- or multi-layer); wider single-layer FOV uses
  // the multi-frustum branch below.
  if (mIsActuallyRotating || layers < 1 || res <= 0 || fov <= 0.0 || fov > 6.3)
    return false;

  WbRenderBackend *back = WbRenderBackendRegistry::resolve(WbRenderBackendKind::Vulkan);
  if (!back || !back->isAvailable())
    return false;

  // Sensor tilt (R5i): pitch the render camera up by the tilt angle — rotate
  // forward +X toward up +Z, i.e. rotate about +Y by −tilt. The per-layer φ
  // sampling is UNCHANGED: it's measured relative to the (now tilted) forward,
  // so tilt cancels and lives entirely in the camera orientation. tilt==0 gives
  // the identity, so the camera stays byte-identical to matrix() and R5e/f/g/h
  // are unaffected.
  const bool hasTilt = mCurrentTiltAngle != 0.0;
  const WbMatrix4 tiltRot = WbMatrix4(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -mCurrentTiltAngle);

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
    if (!WbWgpuSceneRenderer::ensureTarget(back, wfW, wfH, mWgpuTarget, mWgpuMeshCache,
                                           mWgpuTargetWidth, mWgpuTargetHeight))
      return false;
    std::vector<WbWgpuSolidDraw> draws;
    std::vector<std::array<float, 16>> modelStorage;
    modelStorage.reserve(64);
    draws.reserve(64);
    WbWgpuSceneRenderer::collectWorldDraws(*mWgpuMeshCache, draws, modelStorage);  // once
    const double farVal = mMaxRange->value();
    const double dtheta = -fov / static_cast<double>(res);
    const double theta0 = fov / 2.0 + dtheta / 2.0;
    const double dphi = multi ? (-vfov / static_cast<double>(layers - 1)) : 0.0;
    const double phi0 = multi ? (vfov / 2.0) : 0.0;
    const float lo = static_cast<float>(mMinRange->value());
    const float hi = static_cast<float>(farVal);
    for (int k = 0; k < nFrustums; ++k) {
      const double thetaC = fov / 2.0 - fovPer * (k + 0.5);
      WbMatrix4 camK = matrix() * WbMatrix4(0, 0, 0, 0, 0, 1, thetaC);  // azimuth: rotate about up
      if (hasTilt) camK = camK * tiltRot;                               // then pitch by tilt
      float vp[16] = {0}, vm[16] = {0};
      WbWgpuSceneRenderer::buildViewProj(camK, fovPer, renderAspect, mNear->value(), farVal, vp);
      WbWgpuSceneRenderer::buildView(camK, vm);
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
          // lidar +θ = left (+y) → camera-left → NDC x < 0 (see WbWgpuSceneRenderer
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
          const double top = r00 * (1.0 - tx) + r01 * tx;
          const double bot = r10 * (1.0 - tx) + r11 * tx;
          float r = static_cast<float>(top * (1.0 - ty) + bot * ty);
          if (r < lo) r = lo; else if (r > hi) r = hi;
          data[j * res + i] = r;
        }
      }
    }
    static thread_local int sWfLog = 0;
    if (sWfLog < 4) {
      const int cx = res / 2;
      const int cl = layers / 2;
      WbLog::info(QString("[WbLidar] '%1' range via wgpu (%2 layers x %3, fov %4 rad, %5 frustums) "
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
    if (!WbWgpuSceneRenderer::ensureTarget(back, mlRenderW, mlRenderH, mWgpuTarget, mWgpuMeshCache,
                                           mWgpuTargetWidth, mWgpuTargetHeight))
      return false;
    std::vector<WbWgpuSolidDraw> mlDraws;
    std::vector<std::array<float, 16>> mlModelStorage;
    mlModelStorage.reserve(64);
    mlDraws.reserve(64);
    WbWgpuSceneRenderer::collectWorldDraws(*mWgpuMeshCache, mlDraws, mlModelStorage);
    const double mlFar = mMaxRange->value();
    float mlVP[16] = {0}, mlView[16] = {0};
    const WbMatrix4 mlCam = hasTilt ? (matrix() * tiltRot) : matrix();
    WbWgpuSceneRenderer::buildViewProj(mlCam, fov, renderAspect, mNear->value(), mlFar, mlVP);
    WbWgpuSceneRenderer::buildView(mlCam, mlView);
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
        const double top = r00 * (1.0 - tx) + r01 * tx;
        const double bot = r10 * (1.0 - tx) + r11 * tx;
        float r = static_cast<float>(top * (1.0 - ty) + bot * ty);
        if (r < lo) r = lo; else if (r > hi) r = hi;
        data[j * res + i] = r;
      }
    }
    static thread_local int sMlLog = 0;
    if (sMlLog < 4) {
      const int cx = res / 2;
      const int cl = layers / 2;
      WbLog::info(QString("[WbLidar] '%1' range via wgpu (%2 layers x %3, fov %4 vfov %5, "
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
  if (!WbWgpuSceneRenderer::ensureTarget(back, renderW, renderH, mWgpuTarget, mWgpuMeshCache,
                                         mWgpuTargetWidth, mWgpuTargetHeight))
    return false;

  std::vector<WbWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  modelStorage.reserve(64);
  draws.reserve(64);
  WbWgpuSceneRenderer::collectWorldDraws(*mWgpuMeshCache, draws, modelStorage);

  const double aspect = static_cast<double>(renderW) / static_cast<double>(renderH);
  const double farVal = mMaxRange->value();
  float viewProj[16] = {0}, viewMat[16] = {0};
  const WbMatrix4 slCam = hasTilt ? (matrix() * tiltRot) : matrix();
  WbWgpuSceneRenderer::buildViewProj(slCam, fov, aspect, mNear->value(), farVal, viewProj);
  WbWgpuSceneRenderer::buildView(slCam, viewMat);

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
    float r = static_cast<float>(r0 * (1.0 - t) + r1 * t);
    if (r < lo) r = lo; else if (r > hi) r = hi;
    data[i] = r;
  }

  static thread_local int sLidarLog = 0;
  if (sLidarLog < 4) {
    const int cx = res / 2;
    WbLog::info(QString("[WbLidar] '%1' range via wgpu (1 layer x %2, fov %3 rad, %4 draws) "
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
bool WbLidar::renderRotatingWindowViaWgpu(float *tempImage) {
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

  WbRenderBackend *back = WbRenderBackendRegistry::resolve(WbRenderBackendKind::Vulkan);
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
  const WbMatrix4 yawRot = WbMatrix4(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, mPreviousRotatingAngle);
  const WbMatrix4 tiltRot = WbMatrix4(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -mCurrentTiltAngle);
  WbMatrix4 cam = matrix() * yawRot;
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
  if (!WbWgpuSceneRenderer::ensureTarget(back, renderW, renderH, mWgpuTarget, mWgpuMeshCache,
                                         mWgpuTargetWidth, mWgpuTargetHeight))
    return false;

  std::vector<WbWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  modelStorage.reserve(64);
  draws.reserve(64);
  WbWgpuSceneRenderer::collectWorldDraws(*mWgpuMeshCache, draws, modelStorage);

  // Multi-layer: aspect makes the render's vertFov == vfov (so ndc_y maps to
  // elevation). Single-layer: square-ish aspect, vertical doesn't matter (centre
  // row only). Both keep the yaw + tilt entirely in `cam`.
  const double aspect = multi ? (tanHalf / tanHalfV)
                              : (static_cast<double>(renderW) / static_cast<double>(renderH));
  const double farVal = mMaxRange->value();
  float viewProj[16] = {0}, viewMat[16] = {0};
  WbWgpuSceneRenderer::buildViewProj(cam, fov, aspect, mNear->value(), farVal, viewProj);
  WbWgpuSceneRenderer::buildView(cam, viewMat);

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
      const double topv = r00 * (1.0 - tx) + r01 * tx;
      const double botv = r10 * (1.0 - tx) + r11 * tx;
      float rng = static_cast<float>(topv * (1.0 - ty) + botv * ty);
      if (rng < lo) rng = lo; else if (rng > hi) rng = hi;
      tempImage[static_cast<size_t>(r) * winW + col] = rng;
    }
  }

  static thread_local int sRotLog = 0;
  if (sRotLog < 6) {
    const size_t midRow = static_cast<size_t>(H / 2);
    WbLog::info(QString("[WbLidar] '%1' ROTATING window via wgpu (%2 layers x winW=%3, fov %4 rad, "
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
bool WbLidar::renderRotatingWideFovWindowViaWgpu(float *tempImage) {
  if (!tempImage || !hasBeenSetup())
    return false;
  const int layers = actualNumberOfLayers();
  const double fov = actualFieldOfView();
  const int winW = width();
  if (winW < 1 || fov <= 1.4)
    return false;

  WbRenderBackend *back = WbRenderBackendRegistry::resolve(WbRenderBackendKind::Vulkan);
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
  if (!WbWgpuSceneRenderer::ensureTarget(back, renderW, renderH, mWgpuTarget, mWgpuMeshCache,
                                         mWgpuTargetWidth, mWgpuTargetHeight))
    return false;

  std::vector<WbWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  modelStorage.reserve(64);
  draws.reserve(64);
  WbWgpuSceneRenderer::collectWorldDraws(*mWgpuMeshCache, draws, modelStorage);

  const bool hasTilt = mCurrentTiltAngle != 0.0;
  const WbMatrix4 tiltRot = WbMatrix4(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -mCurrentTiltAngle);
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
    WbMatrix4 camK = matrix() * WbMatrix4(0, 0, 0, 0, 0, 1, mPreviousRotatingAngle + thetaC);
    if (hasTilt)
      camK = camK * tiltRot;
    float vp[16] = {0}, vm[16] = {0};
    WbWgpuSceneRenderer::buildViewProj(camK, fovPer, renderAspect, mNear->value(), farVal, vp);
    WbWgpuSceneRenderer::buildView(camK, vm);
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
        const double topv = r00 * (1.0 - tx) + r01 * tx;
        const double botv = r10 * (1.0 - tx) + r11 * tx;
        float rng = static_cast<float>(topv * (1.0 - ty) + botv * ty);
        if (rng < lo) rng = lo; else if (rng > hi) rng = hi;
        tempImage[static_cast<size_t>(r) * winW + col] = rng;
      }
    }
  }

  static thread_local int sWideRotLog = 0;
  if (sWideRotLog < 6) {
    const size_t midRow = static_cast<size_t>(H / 2);
    WbLog::info(QString("[WbLidar] '%1' WIDE ROTATING window via wgpu (%2 layers x winW=%3, "
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

void WbLidar::preFinalize() {
  WbAbstractCamera::preFinalize();

  WbBaseNode *const e = dynamic_cast<WbBaseNode *>(mRotatingHead->value());
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

void WbLidar::postFinalize() {
  WbAbstractCamera::postFinalize();

  WbBaseNode *const e = dynamic_cast<WbBaseNode *>(mRotatingHead->value());
  if (e && !e->isPostFinalizedCalled())
    e->postFinalize();

  connect(mNear, &WbSFDouble::changed, this, &WbLidar::updateNear);
  connect(mMinRange, &WbSFDouble::changed, this, &WbLidar::updateMinRange);
  connect(mMaxRange, &WbSFDouble::changed, this, &WbLidar::updateMaxRange);
  connect(mResolution, &WbSFDouble::changed, this, &WbLidar::updateResolution);
  connect(mTiltAngle, &WbSFDouble::changed, this, &WbLidar::updateTiltAngle);
  connect(mType, &WbSFString::changed, this, &WbLidar::updateType);
  connect(mMinFrequency, &WbSFDouble::changed, this, &WbLidar::updateMinFrequency);
  connect(mMaxFrequency, &WbSFDouble::changed, this, &WbLidar::updateMaxFrequency);
  connect(mDefaultFrequency, &WbSFDouble::changed, this, &WbLidar::updateDefaultFrequency);
  connect(mHorizontalResolution, &WbSFDouble::changed, this, &WbLidar::updateHorizontalResolution);
  connect(mVerticalFieldOfView, &WbSFDouble::changed, this, &WbLidar::updateVerticalFieldOfView);
  connect(mNumberOfLayers, &WbSFInt::changed, this, &WbLidar::updateNumberOfLayers);
  connect(mRotatingHead, &WbSFNode::changed, this, &WbLidar::updateRotatingHead);
}

void WbLidar::reset(const QString &id) {
  WbAbstractCamera::reset(id);

  WbNode *const r = mRotatingHead->value();
  if (r)
    r->reset(id);

  if (mWrenCamera)
    mWrenCamera->rotateYaw(-mCurrentRotatingAngle);

  mIsPointCloudEnabled = false;
  mCurrentRotatingAngle = 0;
  mPreviousRotatingAngle = 0;
  if (mTemporaryImage)
    memset(mTemporaryImage, 0, actualHorizontalResolution() * height() * sizeof(float));
  hidePointCloud();
}

void WbLidar::updateOptionalRendering(int option) {
  if (areWrenObjectsInitialized()) {
    if (option == WbWrenRenderingContext::VF_LIDAR_POINT_CLOUD) {
      if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(option) && mIsPointCloudEnabled)
        displayPointCloud();
      else
        hidePointCloud();
    } else if (option == WbWrenRenderingContext::VF_LIDAR_RAYS_PATHS)
      applyFrustumToWren();
  }
}

void WbLidar::initializeImageMemoryMappedFile() {
  WbAbstractCamera::initializeImageMemoryMappedFile();
  if (mImageMemoryMappedFile) {
    // initialize the memory mapped file with a black image
    float *im = lidarImage();
    const int s = actualHorizontalResolution() * actualNumberOfLayers();
    for (int i = 0; i < s; i++)
      im[i] = 0.0f;
  }
  mTemporaryImage = new float[actualHorizontalResolution() * height()];
}

QString WbLidar::pixelInfo(int x, int y) const {
  WbRgb color;
  if (hasBeenSetup())
    color = mWrenCamera->copyPixelColourValue(x, y);

  return QString::asprintf("depth(%d,%d)=%f", x, y, color.red());
}

void WbLidar::prePhysicsStep(double ms) {
  WbSolid::prePhysicsStep(ms);
  WbSolid *s = solidEndPoint();
  if (mIsActuallyRotating && mSensor->isEnabled()) {
    double angle = -(ms * 2 * M_PI * mDefaultFrequency->value()) / 1000;
    if (s)
      s->rotate(WbVector3(0.0, 0.0, angle));
    if (hasBeenSetup()) {
      mWrenCamera->rotateYaw(angle);
      mPreviousRotatingAngle = mCurrentRotatingAngle;
      mCurrentRotatingAngle += angle;
    }
  }
  if (s)
    s->prePhysicsStep(ms);
}

void WbLidar::postPhysicsStep() {
  WbSolid::postPhysicsStep();
  if (mIsActuallyRotating && mSensor->isEnabled())
    copyAllLayersToMemoryMappedFile();
}

void WbLidar::write(WbWriter &writer) const {
  if (writer.isWebots() || writer.isUrdf())
    WbBaseNode::write(writer);
  else
    writeExport(writer);
}

void WbLidar::exportNodeSubNodes(WbWriter &writer) const {
  WbAbstractCamera::exportNodeSubNodes(writer);
  if (writer.isWebots() || writer.isUrdf())
    return;

  const WbSolid *s = solidEndPoint();
  if (s)
    s->write(writer);
}

void WbLidar::addConfigureToStream(WbDataStream &stream, bool reconfigure) {
  WbAbstractCamera::addConfigureToStream(stream, reconfigure);
  stream << (double)mMaxRange->value();
  stream << (short)mNumberOfLayers->value();
  stream << (double)mDefaultFrequency->value();
  stream << (double)mMinFrequency->value();
  stream << (double)mMaxFrequency->value();
  stream << (double)mVerticalFieldOfView->value();
  stream << (double)actualHorizontalResolution();
}

void WbLidar::writeAnswer(WbDataStream &stream) {
  if (mImageChanged) {
    mImageChanged = false;  // prevent AbstractCamera from copying the whole content of the camera in the memory mapped file
    WbAbstractCamera::writeAnswer(stream);
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
    WbAbstractCamera::writeAnswer(stream);
}

void WbLidar::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;
  if (command == C_SET_SAMPLING_PERIOD) {
    stream >> mRefreshRate;
    if (mIsActuallyRotating)
      mRefreshRate = WbWorld::instance()->basicTimeStep();

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
    hidePointCloud();
    return;
  } else if (command == C_LIDAR_SET_FREQUENCY) {
    double frequency;
    stream >> frequency;
    mDefaultFrequency->setValue(frequency);
    return;
  } else if (WbAbstractCamera::handleCommand(stream, command))
    return;

  assert(0);
}

void WbLidar::copyAllLayersToMemoryMappedFile() {
  if (!hasBeenSetup() || !mImageMemoryMappedFile)
    return;

  // R5e: opt-in wgpu range path (single-layer, non-rotating, narrow FOV). Fills
  // the lidar image with uniform-angle radial ranges, then runs the standard
  // point-cloud derivation. renderRangeViaWgpu returns false for any other
  // config, so the default WREN path below is reached + byte-untouched.
  if (!mIsRemoteExternController && qEnvironmentVariableIntValue("OMNISIM_LIDAR_WGPU") >= 1 &&
      renderRangeViaWgpu(lidarImage())) {
    if (mIsPointCloudEnabled) {
      if (WbWorld::instance()->perspective()->isGlobalOptionalRenderingEnabled("LidarPointClouds"))
        displayPointCloud();
      updatePointCloud(0, actualHorizontalResolution());
    }
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
  // WREN-parity verified (lidar_wgpu_rotating_smoke partial-sweep exact-column
  // test + WREN-oracle column diff): off-axis box lands in the exact predicted
  // band, identical closest column to WREN (187==187), clean front-face columns
  // agree to <0.001 m (flank cols differ only by the same sub-column resample
  // tolerance as the static paths). So this rides the SAME opt-in flag as the
  // rest of the family (OMNISIM_LIDAR_WGPU=1). Single-layer narrow-FOV this
  // increment; multi-layer / wide-FOV rotating + any wgpu miss → WREN fallback,
  // so the default path stays byte-untouched.
  bool windowFilledByWgpu = false;
  if (mIsActuallyRotating && !mIsRemoteExternController &&
      qEnvironmentVariableIntValue("OMNISIM_LIDAR_WGPU") >= 1)
    windowFilledByWgpu = renderRotatingWindowViaWgpu(mTemporaryImage);
  if (!windowFilledByWgpu) {
    mWrenCamera->enableCopying(true);
    mWrenCamera->copyContentsToMemory(mTemporaryImage);
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
    if (WbWorld::instance()->perspective()->isGlobalOptionalRenderingEnabled("LidarPointClouds"))
      displayPointCloud();
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

void WbLidar::updatePointCloud(int minWidth, int maxWidth) {
  WbLidarPoint *lidarPoints = pointArray();
  const float *image = mIsRemoteExternController ? mTcpImage : lidarImage();
  const int resolution = actualHorizontalResolution();
  const int numberOfLayers = actualNumberOfLayers();
  const double w = width();

  const double dt = -((double)mRefreshRate / 1000.0) / w;
  const double t0 = WbSimulationState::instance()->time() / 1000.0 + minWidth * dt;

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

float *WbLidar::lidarImage() const {
  return reinterpret_cast<float *>(image());
}

void WbLidar::createWrenCamera() {
  mActualNumberOfLayers = mNumberOfLayers->value();
  mActualHorizontalResolution = mHorizontalResolution->value();
  mActualVerticalFieldOfView = mVerticalFieldOfView->value();
  mActualFieldOfView = mFieldOfView->value();
  mIsActuallyRotating = mType->value().startsWith('r', Qt::CaseInsensitive);

  WbAbstractCamera::createWrenCamera();
  applyCameraSettings();
  applyMaxRangeToWren();
  applyResolutionToWren();
  applyTiltAngleToWren();
  updateOrientation();
  connect(mWrenCamera, &WbWrenCamera::cameraInitialized, this, &WbLidar::updateOrientation);
}

void WbLidar::updateOrientation() {
  if (hasBeenSetup()) {
    // FLU axis orientation
    mWrenCamera->rotateRoll(M_PI_2);
    mWrenCamera->rotateYaw(-M_PI_2);
  }
}

void WbLidar::deleteWren() {
  if (areWrenObjectsInitialized()) {
    wr_node_delete(WR_NODE(mFrustumRenderable));
    wr_material_delete(mFrustumMaterial);
    wr_static_mesh_delete(mFrustumMesh);

    mFrustumRenderable = NULL;
    mFrustumMaterial = NULL;
    mFrustumMesh = NULL;

    wr_node_delete(WR_NODE(mLidarRaysRenderable));
    wr_node_delete(WR_NODE(mLidarPointsRenderable));
    wr_material_delete(mLidarRaysMaterial);
    wr_material_delete(mLidarPointsMaterial);
    wr_dynamic_mesh_delete(mLidarPointsMesh);
    wr_dynamic_mesh_delete(mLidarRaysMesh);

    mLidarPointsRenderable = NULL;
    mLidarPointsMesh = NULL;
    mLidarPointsMaterial = NULL;

    mLidarRaysRenderable = NULL;
    mLidarRaysMesh = NULL;
    mLidarRaysMaterial = NULL;
  }
}

void WbLidar::displayPointCloud() {
  if (hasBeenSetup() && mImageMemoryMappedFile) {
    const float layersNumber = actualNumberOfLayers();
    const int resolution = actualHorizontalResolution();
    const bool showRays = layersNumber * resolution < POINT_CLOUD_RAY_REPRESENTATION_THRESHOLD;

    wr_node_set_visible(WR_NODE(mLidarPointsRenderable), true);
    wr_node_set_visible(WR_NODE(mLidarRaysRenderable), showRays);

    wr_dynamic_mesh_clear(mLidarRaysMesh);
    wr_dynamic_mesh_clear(mLidarPointsMesh);

    const float origin[3] = {0.0f, 0.0f, 0.0f};
    float color[3] = {0.0f, 0.0f, 1.0f};
    unsigned int pointsIndex = 0;
    unsigned int raysIndex = 0;
    for (int k = 0; k < layersNumber; ++k) {
      if (layersNumber > 1) {  // to avoid division by zero
        color[0] = k / (layersNumber - 1.0f);
        color[2] = 1.0f - color[0];
      }
      for (int l = 0; l < resolution; ++l) {
        const float *vertex_x = &pointArray()[k * resolution + l].x;
        const float *vertex_y = &pointArray()[k * resolution + l].y;
        const float *vertex_z = &pointArray()[k * resolution + l].z;

        if (isinf(*vertex_x) || isinf(*vertex_y) || isinf(*vertex_z))
          continue;

        wr_dynamic_mesh_add_vertex(mLidarPointsMesh, vertex_x);
        wr_dynamic_mesh_add_color(mLidarPointsMesh, color);
        wr_dynamic_mesh_add_index(mLidarPointsMesh, pointsIndex++);
        // Ray
        if (showRays) {
          wr_dynamic_mesh_add_vertex(mLidarRaysMesh, origin);
          wr_dynamic_mesh_add_color(mLidarRaysMesh, color);
          wr_dynamic_mesh_add_index(mLidarRaysMesh, raysIndex++);

          wr_dynamic_mesh_add_vertex(mLidarRaysMesh, vertex_x);
          wr_dynamic_mesh_add_color(mLidarRaysMesh, color);
          wr_dynamic_mesh_add_index(mLidarRaysMesh, raysIndex++);
        }
      }
    }
  }
}

void WbLidar::hidePointCloud() {
  if (hasBeenSetup()) {
    wr_node_set_visible(WR_NODE(mLidarPointsRenderable), false);
    wr_node_set_visible(WR_NODE(mLidarRaysRenderable), false);
  }
}

static void pushVertex(float *vertices, unsigned int index, double x, double y, double z) {
  vertices[3 * index] = static_cast<float>(x);
  vertices[3 * index + 1] = static_cast<float>(y);
  vertices[3 * index + 2] = static_cast<float>(z);
}

// (Re)creates the cyan or gray (lidar disabled) ray if needed
void WbLidar::applyFrustumToWren() {
  wr_node_set_visible(WR_NODE(mFrustumRenderable), false);
  wr_static_mesh_delete(mFrustumMesh);
  mFrustumMesh = NULL;

  if (!WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_LIDAR_RAYS_PATHS))
    return;

  WbRgb frustumColorRgb;
  if (mSensor->isEnabled() && mSensor->isFirstValueReady())
    frustumColorRgb = enabledCameraFrustrumColor();
  else
    frustumColorRgb = disabledCameraFrustrumColor();

  const float frustumColor[3] = {static_cast<float>(frustumColorRgb.red()), static_cast<float>(frustumColorRgb.green()),
                                 static_cast<float>(frustumColorRgb.blue())};
  wr_phong_material_set_color(mFrustumMaterial, frustumColor);

  int i = 0;
  const double n = minRange();
  const double f = maxRange();
  const double fovV = verticalFieldOfView();
  double fovH = fieldOfView();
  if (mIsActuallyRotating)
    fovH = 2 * M_PI;

  const int intermediatePointsNumber = floor(fovH / 0.2);
  const int vertexCount = 4 * actualNumberOfLayers() * (intermediatePointsNumber + 3);
  float vertices[3 * vertexCount];

  for (int layer = 0; layer < actualNumberOfLayers(); ++layer) {
    double vAngle = 0;
    if (actualNumberOfLayers() > 1)
      vAngle = fovV / 2.0 - ((int)layer / ((int)actualNumberOfLayers() - 1.0)) * fovV + mTiltAngle->value();
    const double cosV = cos(vAngle);
    const double sinV = sin(vAngle);
    pushVertex(vertices, i++, 0, 0, 0);
    // min range
    for (int j = 0; j < intermediatePointsNumber + 2; ++j) {
      const double tmpHAngle = fovH / 2.0 - fovH * j / (intermediatePointsNumber + 1);
      const double x = n * cos(tmpHAngle) * cosV;
      const double y = n * sin(tmpHAngle) * cosV;
      const double z = n * sinV;
      pushVertex(vertices, i++, x, y, z);
      pushVertex(vertices, i++, x, y, z);
    }

    pushVertex(vertices, i++, 0, 0, 0);
    pushVertex(vertices, i++, 0, 0, 0);

    // max range
    for (int j = 0; j < intermediatePointsNumber + 2; ++j) {
      const double tmpHAngle = fovH / 2.0 - fovH * j / (intermediatePointsNumber + 1);
      const double x = f * cos(tmpHAngle) * cosV;
      const double y = f * sin(tmpHAngle) * cosV;
      const double z = f * sinV;
      pushVertex(vertices, i++, x, y, z);
      pushVertex(vertices, i++, x, y, z);
    }
    pushVertex(vertices, i++, 0, 0, 0);
  }

  mFrustumMesh = wr_static_mesh_line_set_new(vertexCount, vertices, NULL);
  wr_renderable_set_mesh(mFrustumRenderable, WR_MESH(mFrustumMesh));
  wr_node_set_visible(WR_NODE(mFrustumRenderable), true);
}

int WbLidar::height() const {
  if (actualNumberOfLayers() == 1)
    return 1;
  // as we want the center of the upper/lower pixel line to be aligned with the upper/lower layer we add 'actualFieldOfView() /
  // width()' to verticalFieldOfView
  return ceil((actualVerticalFieldOfView() + actualFieldOfView() / width()) * (width() / actualFieldOfView()));
}

int WbLidar::width() const {
  if (mIsActuallyRotating)
    return ceil(actualHorizontalResolution() * (actualFieldOfView() / (2.0 * M_PI)));
  return actualHorizontalResolution();
}

WbSolid *WbLidar::solidEndPoint() const {
  WbSolid *solid = dynamic_cast<WbSolid *>(mRotatingHead->value());
  if (solid)
    return solid;
  return NULL;
}

int WbLidar::actualNumberOfLayers() const {
  if (hasBeenSetup())
    return mActualNumberOfLayers;
  return mNumberOfLayers->value();
}

int WbLidar::actualHorizontalResolution() const {
  if (hasBeenSetup())
    return mActualHorizontalResolution;
  return mHorizontalResolution->value();
}

double WbLidar::actualVerticalFieldOfView() const {
  if (hasBeenSetup())
    return mActualVerticalFieldOfView;
  return mVerticalFieldOfView->value();
}

double WbLidar::actualFieldOfView() const {
  if (hasBeenSetup())
    return mActualFieldOfView;
  return mFieldOfView->value();
}

/////////////////////
//  Update methods //
/////////////////////

void WbLidar::updateNear() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.01))
    return;

  if (mNear->value() > mMinRange->value()) {
    parsingWarn(tr("'near' is greater than to 'minRange'. Setting 'near' to %1.").arg(mMinRange->value()));
    mNear->setValue(mMinRange->value());
    return;
  }

  if (hasBeenSetup())
    applyNearToWren();
}

void WbLidar::updateMinRange() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mMinRange, 0.01))
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

  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbLidar::updateMaxRange() {
  if (mMaxRange->value() <= mMinRange->value()) {
    double newMaxRange = mMinRange->value() + 1.0;
    parsingWarn(tr("'maxRange' is less or equal to 'minRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
    mMaxRange->setValue(newMaxRange);
    return;
  }

  mNeedToConfigure = true;

  if (hasBeenSetup())
    applyMaxRangeToWren();

  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbLidar::updateFieldOfView() {
  WbAbstractCamera::updateFieldOfView();

  // warn in case of width modification after the setup
  if (hasBeenSetup())
    warn(tr(
      "'fieldOfView' has been modified. This modification will be taken into account after saving and reloading the world."));
}

void WbLidar::updateResolution() {
  if (WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0))
    return;

  if (hasBeenSetup())
    applyResolutionToWren();
}

void WbLidar::updateTiltAngle() {
  if (hasBeenSetup())
    applyTiltAngleToWren();

  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbLidar::updateType() {
  if (mType->value().compare("fixed", Qt::CaseInsensitive) != 0 &&
      mType->value().compare("rotating", Qt::CaseInsensitive) != 0) {
    parsingWarn(tr("'type' should either be 'fixed' or 'rotating', reset to 'fixed'"));
    mType->setValue("fixed");
  }
  if (hasBeenSetup())
    warn(tr("'type' has been modified. This modification will be taken into account after saving and reloading the world."));
  else if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbLidar::updateMinFrequency() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mMinFrequency, 0.01);
  if (mMinFrequency->value() > mMaxFrequency->value()) {
    parsingWarn(tr("'minFrequency' should be smaller or equal to 'maxFrequency'."));
    mMinFrequency->setValue(mMaxFrequency->value());
  }
  if (hasBeenSetup())
    mNeedToConfigure = true;
}

void WbLidar::updateMaxFrequency() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mMaxFrequency, mMinFrequency->value());
  if (mMaxFrequency->value() < mMinFrequency->value()) {
    parsingWarn(tr("'maxFrequency' should be bigger or equal to 'minFrequency'."));
    mMaxFrequency->setValue(mMinFrequency->value());
  }
  if (hasBeenSetup())
    mNeedToConfigure = true;
}

void WbLidar::updateDefaultFrequency() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mDefaultFrequency, mMinFrequency->value());
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

void WbLidar::updateHorizontalResolution() {
  WbFieldChecker::resetIntIfNonPositive(this, mHorizontalResolution, 1);

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

void WbLidar::updateVerticalFieldOfView() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mVerticalFieldOfView, 0.1);
  WbFieldChecker::resetDoubleIfGreater(this, mVerticalFieldOfView, M_PI, M_PI);

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
  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbLidar::updateNumberOfLayers() {
  WbFieldChecker::resetIntIfNonPositive(this, mNumberOfLayers, 1);

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
  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbLidar::updateRotatingHead() {
  const WbSolid *head = solidEndPoint();
  if (head && isPostFinalizedCalled()) {
    if (head->isPostFinalizedCalled())
      mBoundingSphere->addSubBoundingSphere(head->boundingSphere());
    else
      connect(head, &WbBaseNode::finalizationCompleted, this, &WbLidar::updateBoundingSphere);
  }
}

void WbLidar::updateBoundingSphere(WbBaseNode *subNode) {
  disconnect(subNode, &WbBaseNode::finalizationCompleted, this, &WbLidar::updateBoundingSphere);
  mBoundingSphere->addSubBoundingSphere(subNode->boundingSphere());
}

/////////////////////
//  Apply methods  //
/////////////////////

void WbLidar::applyResolutionToWren() {
  mWrenCamera->setRangeResolution(mResolution->value());
}

void WbLidar::applyMaxRangeToWren() {
  mWrenCamera->setMaxRange(mMaxRange->value());
}

void WbLidar::applyCameraSettingsToWren() {
  WbAbstractCamera::applyCameraSettingsToWren();
  applyResolutionToWren();
  applyMaxRangeToWren();
}

int WbLidar::textureGLId() const {
  if (mWrenCamera)
    return mWrenCamera->textureGLId();
  return WbRenderingDevice::textureGLId();
}

void WbLidar::applyTiltAngleToWren() {
  mWrenCamera->rotatePitch(mTiltAngle->value() - mCurrentTiltAngle);
  mCurrentTiltAngle = mTiltAngle->value();
}

void WbLidar::createOdeObjects() {
  WbAbstractCamera::createOdeObjects();
  WbSolid *s = solidEndPoint();
  if (s)
    s->createOdeObjects();
}

void WbLidar::propagateSelection(bool selected) {
  WbAbstractCamera::propagateSelection(selected);
  WbSolid *solid = solidEndPoint();
  if (solid)
    solid->propagateSelection(selected);
}

void WbLidar::setMatrixNeedUpdate() {
  WbAbstractCamera::setMatrixNeedUpdate();
  WbSolid *s = solidEndPoint();
  if (s)
    s->setMatrixNeedUpdate();
}

//////////
// WREN //
//////////

void WbLidar::updateCollisionMaterial(bool triggerChange, bool onSelection) {
  WbAbstractCamera::updateCollisionMaterial(triggerChange, onSelection);
  WbSolid *s = solidEndPoint();
  if (s)
    s->updateCollisionMaterial(triggerChange, onSelection);
}

void WbLidar::setSleepMaterial() {
  WbAbstractCamera::setSleepMaterial();
  WbSolid *s = solidEndPoint();
  if (s)
    s->setSleepMaterial();
}

void WbLidar::setScaleNeedUpdate() {
  WbAbstractCamera::setScaleNeedUpdate();
  WbSolid *s = solidEndPoint();
  if (s)
    s->setScaleNeedUpdate();
}

void WbLidar::attachResizeManipulator() {
  WbAbstractCamera::attachResizeManipulator();
  WbSolid *s = solidEndPoint();
  if (s)
    s->attachResizeManipulator();
}

void WbLidar::detachResizeManipulator() const {
  WbAbstractCamera::detachResizeManipulator();
  const WbSolid *s = solidEndPoint();
  if (s)
    s->detachResizeManipulator();
}

void WbLidar::createWrenObjects() {
  // Required to draw lidar points
  wr_config_enable_point_size(true);

  // Lidar frustum
  mFrustumMaterial = wr_phong_material_new();
  wr_material_set_default_program(mFrustumMaterial, WbWrenShaders::lineSetShader());

  mFrustumRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mFrustumRenderable, false);
  wr_renderable_set_receive_shadows(mFrustumRenderable, false);
  wr_renderable_set_visibility_flags(mFrustumRenderable, WbWrenRenderingContext::VF_LIDAR_RAYS_PATHS);
  wr_renderable_set_material(mFrustumRenderable, mFrustumMaterial, NULL);
  wr_renderable_set_drawing_mode(mFrustumRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_node_set_visible(WR_NODE(mFrustumRenderable), false);

  // Lidar point cloud
  mLidarPointsMaterial = wr_phong_material_new();
  wr_phong_material_set_color_per_vertex(mLidarPointsMaterial, true);
  wr_material_set_default_program(mLidarPointsMaterial, WbWrenShaders::pointSetShader());
  wr_phong_material_set_transparency(mLidarPointsMaterial, 0.3f);

  mLidarRaysMaterial = wr_phong_material_new();
  wr_phong_material_set_color_per_vertex(mLidarRaysMaterial, true);
  wr_material_set_default_program(mLidarRaysMaterial, WbWrenShaders::lineSetShader());
  wr_phong_material_set_transparency(mLidarRaysMaterial, 0.7f);

  mLidarPointsMesh = wr_dynamic_mesh_new(false, false, true);
  mLidarRaysMesh = wr_dynamic_mesh_new(false, false, true);

  mLidarPointsRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mLidarPointsRenderable, false);
  wr_renderable_set_receive_shadows(mLidarPointsRenderable, false);
  wr_renderable_set_visibility_flags(mLidarPointsRenderable, WbWrenRenderingContext::VF_LIDAR_POINT_CLOUD);
  wr_renderable_set_material(mLidarPointsRenderable, mLidarPointsMaterial, NULL);
  wr_renderable_set_drawing_mode(mLidarPointsRenderable, WR_RENDERABLE_DRAWING_MODE_POINTS);
  wr_renderable_set_mesh(mLidarPointsRenderable, WR_MESH(mLidarPointsMesh));
  wr_renderable_set_drawing_order(mLidarPointsRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);
  wr_renderable_set_point_size(mLidarPointsRenderable, 3.0f);

  mLidarRaysRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mLidarRaysRenderable, false);
  wr_renderable_set_receive_shadows(mLidarRaysRenderable, false);
  wr_renderable_set_visibility_flags(mLidarRaysRenderable, WbWrenRenderingContext::VF_LIDAR_POINT_CLOUD);
  wr_renderable_set_material(mLidarRaysRenderable, mLidarRaysMaterial, NULL);
  wr_renderable_set_drawing_mode(mLidarRaysRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_mesh(mLidarRaysRenderable, WR_MESH(mLidarRaysMesh));
  wr_renderable_set_drawing_order(mLidarRaysRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_0);

  WbAbstractCamera::createWrenObjects();

  wr_transform_attach_child(wrenNode(), WR_NODE(mFrustumRenderable));
  wr_transform_attach_child(wrenNode(), WR_NODE(mLidarRaysRenderable));
  wr_transform_attach_child(wrenNode(), WR_NODE(mLidarPointsRenderable));

  WbSolid *const s = solidEndPoint();
  if (s)
    s->createWrenObjects();
}
