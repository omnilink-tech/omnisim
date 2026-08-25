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

#ifndef OM_RANGE_FINDER_HPP
#define OM_RANGE_FINDER_HPP

#include "OmAbstractCamera.hpp"

class OmRenderBackend;
class OmWgpuMeshCache;
class OmWgpuRenderTarget;

class OmRangeFinder : public OmAbstractCamera {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmRangeFinder(OmTokenizer *tokenizer = NULL);
  OmRangeFinder(const OmRangeFinder &other);
  explicit OmRangeFinder(const OmNode &other);
  virtual ~OmRangeFinder() override;

  // reimplemented public functions
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  int nodeType() const override { return WB_NODE_RANGE_FINDER; }
  QString pixelInfo(int x, int y) const override;
  OmRgb enabledCameraFrustrumColor() const override { return OmRgb(1.0f, 1.0f, 0.0f); }

  bool isRangeFinder() override { return true; }
  double maxRange() const override { return mMaxRange->value(); }

private:
  // user accessible fields
  OmSFDouble *mMinRange;
  OmSFDouble *mMaxRange;
  OmSFDouble *mResolution;

  // private functions
  void addConfigureToStream(OmDataStream &stream, bool reconfigure = false) override;

  float *rangeFinderImage() const;

  OmRangeFinder &operator=(const OmRangeFinder &);  // non copyable
  OmNode *clone() const override { return new OmRangeFinder(*this); }
  void init();
  void initializeImageMemoryMappedFile() override;

  // R5c (unconditional since D1.4 — WREN is deleted, so this is the ONLY producer):
  // off-screen wgpu depth path for the RangeFinder DEVICE node. Renders linear
  // view-space distance into an R32Float target
  // (OmWgpuRenderTarget::clearAndDrawSceneDepthF32) and writes the metric depth
  // straight into this device's float image buffer — the RangeFinder's native output.
  // Shares OmWgpuSceneRenderer with the Camera path. Owned + destroyed here.
  OmWgpuRenderTarget *mWgpuTarget;
  OmWgpuMeshCache *mWgpuMeshCache;
  int mWgpuTargetWidth;
  int mWgpuTargetHeight;
  void copyImageToMemoryMappedFile(unsigned char *data) override;

  // Lane E1 (P5): CPU port of motion_blur.frag for the FLOAT depth image on the wgpu arm
  // (WREN pass order: blur runs before the range noise/quantization). `mWgpuBlurHistory` is
  // the pass's own previous output (the shader's feedback texture); cleared whenever blur is
  // inactive so a re-enable starts from firstRender semantics.
  void applyWgpuDepthMotionBlur(float *meters, int count);
  std::vector<float> mWgpuBlurHistory;

  // F2.3 (wren-deletion-runbook): one-shot guard for the non-planar-projection DECLINE
  // warning -- the wgpu sensor path is a planar pinhole (buildViewProj = tan(fov/2)), so
  // cylindrical/spherical devices stand down to the WREN/base path instead of rendering
  // confidently-wrong metres. Per device, reset by init().
  bool mWgpuNonPlanarDeclineWarned;

  int size() const override { return sizeof(float) * width() * height(); }
  double minRange() const override { return mMinRange->value(); }
  bool isFrustumEnabled() const override;

private slots:
  void updateNear();
  void updateMinRange();
  void updateMaxRange();
  void updateResolution();
};

#endif  // OM_RANGE_FINDER_HPP
