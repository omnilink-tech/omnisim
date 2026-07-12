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

#ifndef WB_RANGE_FINDER_HPP
#define WB_RANGE_FINDER_HPP

#include "WbAbstractCamera.hpp"

class WbRenderBackend;
class WbWgpuMeshCache;
class WbWgpuRenderTarget;

class WbRangeFinder : public WbAbstractCamera {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbRangeFinder(WbTokenizer *tokenizer = NULL);
  WbRangeFinder(const WbRangeFinder &other);
  explicit WbRangeFinder(const WbNode &other);
  virtual ~WbRangeFinder() override;

  // reimplemented public functions
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  int nodeType() const override { return WB_NODE_RANGE_FINDER; }
  QString pixelInfo(int x, int y) const override;
  WbRgb enabledCameraFrustrumColor() const override { return WbRgb(1.0f, 1.0f, 0.0f); }

  bool isRangeFinder() override { return true; }
  double maxRange() const override { return mMaxRange->value(); }

  int textureGLId() const override;

private:
  // user accessible fields
  WbSFDouble *mMinRange;
  WbSFDouble *mMaxRange;
  WbSFDouble *mResolution;

  // private functions
  void addConfigureToStream(WbDataStream &stream, bool reconfigure = false) override;

  float *rangeFinderImage() const;

  WbRangeFinder &operator=(const WbRangeFinder &);  // non copyable
  WbNode *clone() const override { return new WbRangeFinder(*this); }
  void init();
  void initializeImageMemoryMappedFile() override;

  // R5c: off-screen wgpu depth path for the RangeFinder DEVICE node. Opt-in
  // via OMNISIM_RANGEFINDER_WGPU=1; renders linear view-space distance into an
  // R32Float target (WbWgpuRenderTarget::clearAndDrawSceneDepthF32) and writes
  // the metric depth straight into this device's float image buffer — the
  // RangeFinder's native output. Falls through to WREN when disabled or wgpu
  // is unavailable. Shares WbWgpuSceneRenderer with the (eventual) Camera path.
  // Owned + destroyed here; null when on the legacy WREN path.
  WbWgpuRenderTarget *mWgpuTarget;
  WbWgpuMeshCache *mWgpuMeshCache;
  int mWgpuTargetWidth;
  int mWgpuTargetHeight;
  void copyImageToMemoryMappedFile(WbWrenCamera *camera, unsigned char *data) override;

  int size() const override { return sizeof(float) * width() * height(); }
  double minRange() const override { return mMinRange->value(); }
  bool isFrustumEnabled() const override;

  // WREN
  void createWrenCamera() override;
  void applyMinRangeToWren();
  void applyMaxRangeToWren();
  void applyResolutionToWren();

private slots:
  void updateNear();
  void updateMinRange();
  void updateMaxRange();
  void updateResolution();
  void updateOrientation();
  void applyCameraSettingsToWren() override;
  void updateFrustumDisplayIfNeeded(int optionalRendering) override;
};

#endif  // WB_RANGE_FINDER_HPP
