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

#include "OmRangeFinder.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmPreferences.hpp"
#include "OmRgb.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmWrenTextureOverlay.hpp"

// R5c: wgpu depth path for the RangeFinder device node.
#include "OmLog.hpp"
#include "OmMatrix4.hpp"
#include "OmRenderBackend.hpp"
#include "OmWgpuMeshCache.hpp"
#include "OmWgpuRenderTarget.hpp"
#include "OmWgpuSceneRenderer.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QtGlobal>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

void OmRangeFinder::init() {
  mCharType = 'r';

  // Fields initialization
  mMinRange = findSFDouble("minRange");
  mMaxRange = findSFDouble("maxRange");
  mResolution = findSFDouble("resolution");

  // R5c: wgpu depth target stays null until the first copyImageToMemoryMappedFile
  // with OMNISIM_RANGEFINDER_WGPU=1 and a usable Vulkan backend.
  mWgpuTarget = NULL;
  mWgpuMeshCache = NULL;
  mWgpuTargetWidth = 0;
  mWgpuTargetHeight = 0;
  // F2.3: one-shot guard for the non-planar-projection decline warning.
  mWgpuNonPlanarDeclineWarned = false;

  // backward compatibility
  OmSFBool *sphericalField = findSFBool("spherical");
  if (sphericalField->value()) {  // Deprecated in Webots R2023
    parsingWarn("Deprecated 'spherical' field, please use the 'projection' field instead.");
    if (isPlanarProjection())
      mProjection->setValue("cylindrical");
    sphericalField->setValue(false);
  }
}

OmRangeFinder::OmRangeFinder(OmTokenizer *tokenizer) : OmAbstractCamera("RangeFinder", tokenizer) {
  init();
}

OmRangeFinder::OmRangeFinder(const OmRangeFinder &other) : OmAbstractCamera(other) {
  init();
}

OmRangeFinder::OmRangeFinder(const OmNode &other) : OmAbstractCamera(other) {
  init();
}

OmRangeFinder::~OmRangeFinder() {
  // R5c: same teardown order as OmCamera — the mesh cache references the
  // target's WGPUDevice, so release it first.
  delete mWgpuMeshCache;
  mWgpuMeshCache = NULL;
  delete mWgpuTarget;
  mWgpuTarget = NULL;
}

// R5c (unconditional since D1.4 — WREN is deleted, so this is the ONLY producer): route the
// RangeFinder's depth image through the wgpu R32Float pipeline. The RangeFinder's output
// buffer is already float metres (rangeFinderImage()), which is exactly what
// clearAndDrawSceneDepthF32 produces — so the readback lands straight in the device buffer
// with no conversion. Falls through to the base (which warns the honest no-image case) when
// wgpu is unavailable or on any failure.
void OmRangeFinder::copyImageToMemoryMappedFile(unsigned char *data) {
  // F2.3 (wren-deletion-runbook): the wgpu depth arm is a PLANAR PINHOLE -- buildViewProj
  // derives the projection from tan(fieldOfView/2), so a cylindrical/spherical device
  // (fov up to 2*pi) degenerates and returns confidently-wrong METRES (the
  // range_finder_spherical family measured metres off against expectations). Non-planar
  // projections are RETIRED-unported on wgpu: DECLINE, once and by name. Post-D1.4 there
  // is no WREN fallback, so the image stays HONESTLY EMPTY (the base copy names the frozen
  // buffer once). Wrong distances are worse than no distances.
  if (data && !isPlanarProjection()) {
    if (!mWgpuNonPlanarDeclineWarned) {
      mWgpuNonPlanarDeclineWarned = true;
      warn(tr("'%1': projection \"%2\" is not implemented by the wgpu sensor path (planar "
              "pinhole only) -- the wgpu render is DECLINED for this device and its image "
              "stays empty (the WREN renderer that implemented it is deleted).")
             .arg(deviceName())
             .arg(mProjection->value()));
    }
  } else if (data) {
    OmRenderBackend *back = OmRenderBackendRegistry::resolve(OmRenderBackendKind::Vulkan);
    if (back && back->isAvailable() &&
        OmWgpuSceneRenderer::ensureTarget(back, width(), height(), mWgpuTarget, mWgpuMeshCache,
                                          mWgpuTargetWidth, mWgpuTargetHeight)) {
      std::vector<OmWgpuSolidDraw> draws;
      std::vector<std::array<float, 16>> modelStorage;
      modelStorage.reserve(64);  // stable backing for draws' modelMatrix16 ptrs
      draws.reserve(64);
      collectWgpuDraws(*mWgpuMeshCache, draws, modelStorage);

      const double aspect =
        mWgpuTargetHeight > 0 ? static_cast<double>(mWgpuTargetWidth) / mWgpuTargetHeight : 1.0;
      const float farVal = static_cast<float>(mMaxRange->value());
      float viewProj[16] = {0};
      OmWgpuSceneRenderer::buildViewProj(matrix(), fieldOfView(), aspect, mNear->value(),
                                         mMaxRange->value(), viewProj);

      // The device buffer IS float depth — write metres straight into it.
      // farVal seeds pixels no geometry covers (RangeFinder "nothing hit").
      float *meters = reinterpret_cast<float *>(data);
      if (mWgpuTarget->clearAndDrawSceneDepthF32(farVal, viewProj,
                                                 draws.empty() ? nullptr : draws.data(),
                                                 static_cast<uint32_t>(draws.size()), meters)) {
        // Lane E1 (P5): motionBlur, WREN pass order (PP_MOTION_BLUR runs BEFORE the range
        // noise/quantization passes), so it blends the clean depth. Feedback history is the
        // pass's own previous output, as in motion_blur.frag; a non-finite sample on either
        // side passes the scene value through (the shader's FLT_MAX branch).
        applyWgpuDepthMotionBlur(meters, mWgpuTargetWidth * mWgpuTargetHeight);
        const bool noiseActive =
          wgpuSensorPostFxEnabled() && mNoise != NULL && mNoise->value() > 0.0;
        if (!noiseActive) {
          // F2.4 (wren-deletion-runbook): out-of-range is +INFINITY, matching the reference
          // contract -- docs/reference/rangefinder.md:65 "If the depth is smaller than the
          // minRange value then infinity is returned", :69 "If the depth is bigger than the
          // maxRange value then infinity is returned" (restated at :390) -- and matching
          // the E1 noise path, which already reproduces WREN's range CLIP (outside the OPEN
          // interval (minRange, maxRange), including the background seeded at exactly
          // maxRange, becomes +inf). Controllers test isinf(); the previous no-noise CLAMP
          // into [minRange, maxRange] returned maxRange where WREN wrote +inf, which is
          // exactly what pinned the `range_finder` api test red at R5.
          const float lo = static_cast<float>(mMinRange->value());
          const float hi = farVal;
          const float kInf = std::numeric_limits<float>::infinity();
          const int n = mWgpuTargetWidth * mWgpuTargetHeight;
          for (int i = 0; i < n; ++i) {
            if (!(meters[i] > lo && meters[i] < hi))
              meters[i] = kInf;
          }
        }
        // Lane E1 (P5): the CPU port of range_noise.frag + depth_resolution.frag. With
        // noise > 0 it also reproduces WREN's range CLIP (outside the open interval ->
        // +inf, replacing the clamp above); with noise == 0 it applies quantization only.
        applyWgpuRangePostFx(meters, mWgpuTargetWidth, mWgpuTargetHeight, mResolution->value());
        static thread_local int sLog = 0;
        if (sLog < 4) {
          const int cx = mWgpuTargetWidth / 2;
          const int cy = mWgpuTargetHeight / 2;
          OmLog::info(QString("[OmRangeFinder] '%1' depth via wgpu (%2x%3, %4 draw%5) "
                              "center=%6 m [maxRange %7]")
                        .arg(deviceName())
                        .arg(mWgpuTargetWidth)
                        .arg(mWgpuTargetHeight)
                        .arg(static_cast<int>(draws.size()))
                        .arg(draws.size() == 1 ? "" : "s")
                        .arg(meters[cy * mWgpuTargetWidth + cx], 0, 'f', 4)
                        .arg(farVal, 0, 'f', 2));
          ++sLog;
        }
        return;
      }
    }
    // Fall through to the base on any failure — it warns once that nothing wrote pixels.
  }
  OmAbstractCamera::copyImageToMemoryMappedFile(data);
}

// Lane E1 (P5): CPU port of motion_blur.frag for the FLOAT depth image.
// out = mix(scene, history, blend), blend = pow(0.005, refreshRate_ms / motionBlur_ms) --
// the weight of the OLD frame (OmAbstractCamera::applyMotionBlurToWren, verified
// numerically in the runbook). History is this pass's own previous OUTPUT (feedback), as in
// the shader; firstRender (empty/mis-sized history) passes the scene through; a non-finite
// sample on either side passes the scene value through (the shader's FLT_MAX branch).
// motionBlur == 0 would divide by zero -- the > 0.0 gate is the guard. Gated on
// OMNISIM_WGPU_SENSOR_POSTFX like every Lane E1 port.
void OmRangeFinder::applyWgpuDepthMotionBlur(float *meters, int count) {
  if (meters == NULL || count <= 0 || !wgpuSensorPostFxEnabled())
    return;
  if (mMotionBlur == NULL || mMotionBlur->value() <= 0.0 || mRefreshRate <= 0) {
    mWgpuBlurHistory.clear();
    return;
  }
  const float blend = static_cast<float>(pow(0.005, static_cast<double>(mRefreshRate) / mMotionBlur->value()));
  if (mWgpuBlurHistory.size() != static_cast<size_t>(count)) {
    mWgpuBlurHistory.assign(meters, meters + count);
    return;
  }
  const float fs = 1.0f - blend;
  for (int i = 0; i < count; ++i) {
    const float scene = meters[i];
    const float last = mWgpuBlurHistory[i];
    const float out = (std::isfinite(scene) && std::isfinite(last)) ? fs * scene + blend * last : scene;
    meters[i] = out;
    mWgpuBlurHistory[i] = out;
  }
}

void OmRangeFinder::preFinalize() {
  OmAbstractCamera::preFinalize();

  updateNear();
  updateMinRange();
  updateMaxRange();
  updateResolution();
}

void OmRangeFinder::postFinalize() {
  OmAbstractCamera::postFinalize();

  connect(mNear, &OmSFDouble::changed, this, &OmRangeFinder::updateNear);
  connect(mMinRange, &OmSFDouble::changed, this, &OmRangeFinder::updateMinRange);
  connect(mMaxRange, &OmSFDouble::changed, this, &OmRangeFinder::updateMaxRange);
  connect(mResolution, &OmSFDouble::changed, this, &OmRangeFinder::updateResolution);
}

void OmRangeFinder::initializeImageMemoryMappedFile() {
  OmAbstractCamera::initializeImageMemoryMappedFile();
  if (mImageMemoryMappedFile) {
    // initialize the memory mapped file with a black image
    float *im = rangeFinderImage();
    const int s = width() * height();
    for (int i = 0; i < s; i++)
      im[i] = 0.0f;
  }
}

QString OmRangeFinder::pixelInfo(int x, int y) const {
  // D1.4: the WREN texture readback is gone -- answer from the device's own float buffer
  // (the very metres the controller reads), which the wgpu depth path fills.
  float depth = 0.0f;
  if (hasBeenSetup() && image() != NULL && x >= 0 && x < width() && y >= 0 && y < height())
    depth = rangeFinderImage()[y * width() + x];
  return QString::asprintf("depth(%d,%d)=%f", x, y, depth);
}

void OmRangeFinder::addConfigureToStream(OmDataStream &stream, bool reconfigure) {
  OmAbstractCamera::addConfigureToStream(stream, reconfigure);
  stream << (double)mMaxRange->value();
}

void OmRangeFinder::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  if (OmAbstractCamera::handleCommand(stream, command))
    return;

  assert(0);
}

float *OmRangeFinder::rangeFinderImage() const {
  return reinterpret_cast<float *>(image());
}

/////////////////////
//  Update methods //
/////////////////////

void OmRangeFinder::updateNear() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.01))
    return;

  if (mNear->value() > mMinRange->value()) {
    parsingWarn(tr("'near' is greater than to 'minRange'. Setting 'near' to %1.").arg(mMinRange->value()));
    mNear->setValueNoSignal(mMinRange->value());
    return;
  }
  // D1.4: the wgpu depth path reads mNear live per frame.
}

void OmRangeFinder::updateMinRange() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mMinRange, 0.01))
    return;

  if (mMinRange->value() < mNear->value()) {
    parsingWarn(tr("'minRange' is less than 'near'. Setting 'minRange' to %1.").arg(mNear->value()));
    mMinRange->setValueNoSignal(mNear->value());
  }
  if (mMaxRange->value() <= mMinRange->value()) {
    if (mMaxRange->value() == 0.0) {
      double newMaxRange = mMinRange->value() + 1.0;
      parsingWarn(tr("'minRange' is greater or equal to 'maxRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
      mMaxRange->setValueNoSignal(newMaxRange);
    } else {
      double newMinRange = mMaxRange->value() - 1.0;
      if (newMinRange < 0.0)
        newMinRange = 0.0;
      parsingWarn(tr("'minRange' is greater or equal to 'maxRange'. Setting 'minRange' to %1.").arg(newMinRange));
      mMinRange->setValueNoSignal(newMinRange);
    }
  }

  mNeedToConfigure = true;
  // D1.4: the wgpu depth path reads mMinRange live; the WREN frustum visuals are gone
  // (the wgpu frustum overlay draws from engine state).
}

void OmRangeFinder::updateMaxRange() {
  if (mMaxRange->value() <= mMinRange->value()) {
    double newMaxRange = mMinRange->value() + 1.0;
    parsingWarn(tr("'maxRange' is less or equal to 'minRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
    mMaxRange->setValueNoSignal(newMaxRange);
  }

  mNeedToConfigure = true;

  // D1.4: the wgpu depth path reads mMaxRange live; only the overlay's depth-display
  // normalization still needs the value pushed (formerly applyMaxRangeToWren's non-WREN
  // side effect).
  if (hasBeenSetup() && mOverlay)
    mOverlay->setMaxRange(mMaxRange->value());
}

void OmRangeFinder::updateResolution() {
  if (OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0))
    return;
  // D1.4: the wgpu path reads mResolution live in applyWgpuRangePostFx.
}

bool OmRangeFinder::isFrustumEnabled() const {
  return OmWrenRenderingContext::instance()->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS);
}
