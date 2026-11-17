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

#ifndef OM_LIDAR_HPP
#define OM_LIDAR_HPP

#include "OmAbstractCamera.hpp"
#include "OmSFInt.hpp"

#include "../../../include/controller/c/omnisim/lidar_point.h"

class OmRenderBackend;
class OmWgpuMeshCache;
class OmWgpuRenderTarget;

class OmLidar : public OmAbstractCamera {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmLidar(OmTokenizer *tokenizer = NULL);
  OmLidar(const OmLidar &other);
  explicit OmLidar(const OmNode &other);
  virtual ~OmLidar() override;

  // reimplemented public functions
  void createOdeObjects() override;
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void writeAnswer(OmDataStream &stream) override;
  void reset(const QString &id) override;
  void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setScaleNeedUpdate() override;
  void attachResizeManipulator() override;
  void detachResizeManipulator() const override;
  void handleMessage(QDataStream &) override;
  int nodeType() const override { return WB_NODE_LIDAR; }
  QString pixelInfo(int x, int y) const override;
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void write(OmWriter &writer) const override;
  void exportNodeSubNodes(OmWriter &writer) const override;
  OmRgb enabledCameraFrustrumColor() const override { return OmRgb(0.0f, 1.0f, 1.0f); }

  double maxRange() const override { return mMaxRange->value(); }

  // These functions return the value actually used by the lidar (that was initially loaded from the world file or changed
  // before the start of the simulation). It may be different from the current value of the field if it was changed after the
  // start of the simulation. Once the simulation starts, such changes cannot be applied directly and are applied only after a
  // save and reload. This is explained to the user in a warning message.
  int actualNumberOfLayers() const;
  int actualHorizontalResolution() const;
  double actualVerticalFieldOfView() const;
  double actualFieldOfView() const;

  int width() const override;
  int height() const override;
  double fieldOfView() const override { return actualFieldOfView(); }

  OmSolid *solidEndPoint() const;

  // selection
  void propagateSelection(bool selected) override;

  // lazy matrix multiplication system
  void setMatrixNeedUpdate() override;

private:
  // user accessible fields
  OmSFDouble *mTiltAngle;
  OmSFInt *mHorizontalResolution;
  OmSFDouble *mVerticalFieldOfView;
  OmSFInt *mNumberOfLayers;
  OmSFDouble *mMinRange;
  OmSFDouble *mMaxRange;
  OmSFDouble *mResolution;
  OmSFDouble *mDefaultFrequency;
  OmSFDouble *mMinFrequency;
  OmSFDouble *mMaxFrequency;
  OmSFString *mType;
  OmSFNode *mRotatingHead;

  bool mIsPointCloudEnabled;
  double mCurrentRotatingAngle;
  double mPreviousRotatingAngle;
  double mCurrentTiltAngle;
  float *mTemporaryImage;
  float *mTcpImage;
  WbLidarPoint *mTcpCloudPoints;

  int mActualNumberOfLayers;
  int mActualHorizontalResolution;
  double mActualVerticalFieldOfView;
  double mActualFieldOfView;
  bool mIsActuallyRotating;

  // private functions
  void addConfigureToStream(OmDataStream &stream, bool reconfigure = false) override;

  // Lane E1 (P5+P6): the wgpu Lidar pipeline is CYLINDRICAL by construction (the whole
  // point of the angular resample), so the base class's planar-only envelope test is
  // inverted here; noise + resolution are ported (applyWgpuRangePostFx) and so absent.
  QString wgpuUnsupportedFeature(bool includeSilentPostFx) const override;
  // Latches mActual* / mIsActuallyRotating / mCurrentTiltAngle at setup (D1.4: every
  // setup is wrenless -- the deleted createWrenCamera() used to be the latch site).
  void setupWrenless() override;

  void copyAllLayersToMemoryMappedFile();
  void updatePointCloud(int minWidth, int maxWidth);
  float *lidarImage() const;

  // R5e (unconditional since D1.4 -- WREN is deleted, so this family is the ONLY
  // producer): off-screen wgpu range path for the Lidar device node. Renders radial range
  // (clearAndDrawSceneRangeF32) and angular-resamples it into `data` (uniform-angle, as
  // updatePointCloud expects). Handles non-rotating single/multi-layer, narrow and wide
  // FOV; returns false for the rest (the caller then reports the honest no-image case).
  // See engine-migration-plan.md R5 OmLidar implementation spec. Owned + destroyed here.
  bool renderRangeViaWgpu(float *data);
  // R5j: rotating-head wgpu path. Fills `tempImage` (the instantaneous fov
  // window, width()xheight()) with uniform-angle radial range, rendered at the
  // sensor pose YAWED by mPreviousRotatingAngle (+tilt) — the wgpu replacement
  // for mWrenCamera->copyContentsToMemory in the rotating branch of
  // copyAllLayersToMemoryMappedFile. prevAngle (not current) matches the basis
  // the shared band-copy's widthOffset uses (and the heading WREN's lagged GL
  // readback reflects), so the window lands at the SAME global column; the
  // band-copy + updatePointCloud downstream then run byte-unchanged. WREN-parity
  // verified (lidar_wgpu_rotating_smoke partial-sweep exact-column test +
  // WREN-oracle diff, measured before the WREN deletion). Returns false on any
  // wgpu miss (the caller then reports the honest no-image case).
  bool renderRotatingWindowViaWgpu(float *tempImage);
  // R5l: wide-FOV variant of the above. When the rotating window's fov exceeds a
  // single frustum (> 1.4 rad), renderRotatingWindowViaWgpu dispatches here: the
  // window azimuth span is split into N = ceil(fov/1.2) sub-frustums (the R5g
  // multi-frustum stitch), each rendered at the sensor pose yawed by
  // mPreviousRotatingAngle + the per-frustum centre θ_k (+tilt), then stitched by
  // azimuth into tempImage's height()×width() layout (per-row elevation as R5k).
  // Returns false on any wgpu miss (honest no-image case; WREN is deleted).
  bool renderRotatingWideFovWindowViaWgpu(float *tempImage);
  OmWgpuRenderTarget *mWgpuTarget;
  OmWgpuMeshCache *mWgpuMeshCache;
  int mWgpuTargetWidth;
  int mWgpuTargetHeight;

  OmLidar &operator=(const OmLidar &);  // non copyable
  OmNode *clone() const override { return new OmLidar(*this); }
  void init();
  void initializeImageMemoryMappedFile() override;

  int size() const override {
    return (sizeof(float) + sizeof(WbLidarPoint)) * actualHorizontalResolution() * actualNumberOfLayers();
  }
  double minRange() const override { return mMinRange->value(); }
  double verticalFieldOfView() const { return actualFieldOfView() * ((double)height() / (double)width()); }

  WbLidarPoint *pointArray() {
    return mIsRemoteExternController ?
             mTcpCloudPoints :
             reinterpret_cast<WbLidarPoint *>(lidarImage() + actualHorizontalResolution() * actualNumberOfLayers());
  }

private slots:
  void updateNear();
  void updateMinRange();
  void updateMaxRange();
  void updateResolution();
  void updateTiltAngle();
  void updateType();
  void updateMinFrequency();
  void updateMaxFrequency();
  void updateDefaultFrequency();
  void updateHorizontalResolution();
  void updateVerticalFieldOfView();
  void updateNumberOfLayers();
  void updateRotatingHead();
  void updateBoundingSphere(OmBaseNode *subNode);
  void updateFieldOfView() override;
};

#endif  // OM_LIDAR_HPP
