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

#ifndef WB_LIDAR_HPP
#define WB_LIDAR_HPP

#include "WbAbstractCamera.hpp"
#include "WbSFInt.hpp"

#include "../../../include/controller/c/webots/lidar_point.h"

struct WrRenderable;
struct WrDynamicMesh;
struct WrStaticMesh;
struct WrMaterial;

class WbRenderBackend;
class WbWgpuMeshCache;
class WbWgpuRenderTarget;

class WbLidar : public WbAbstractCamera {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbLidar(WbTokenizer *tokenizer = NULL);
  WbLidar(const WbLidar &other);
  explicit WbLidar(const WbNode &other);
  virtual ~WbLidar() override;

  // reimplemented public functions
  void createOdeObjects() override;
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void writeAnswer(WbDataStream &stream) override;
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
  void write(WbWriter &writer) const override;
  void exportNodeSubNodes(WbWriter &writer) const override;
  WbRgb enabledCameraFrustrumColor() const override { return WbRgb(0.0f, 1.0f, 1.0f); }

  double maxRange() const override { return mMaxRange->value(); }

  // These functions return the value actually used by the lidar (that was initially loaded from the world file or changed
  // before the start of the simulation). It may be different from the current value of the field if it was changed after the
  // start of the simulation. Once the simulation starts, such changes cannot be applied directly and are applied only after a
  // save and reload. This is explained to the user in a warning message.
  int actualNumberOfLayers() const;
  int actualHorizontalResolution() const;
  double actualVerticalFieldOfView() const;
  double actualFieldOfView() const;

  int textureGLId() const override;
  int width() const override;
  int height() const override;
  double fieldOfView() const override { return actualFieldOfView(); }

  WbSolid *solidEndPoint() const;

  // selection
  void propagateSelection(bool selected) override;

  // lazy matrix multiplication system
  void setMatrixNeedUpdate() override;

private:
  // user accessible fields
  WbSFDouble *mTiltAngle;
  WbSFInt *mHorizontalResolution;
  WbSFDouble *mVerticalFieldOfView;
  WbSFInt *mNumberOfLayers;
  WbSFDouble *mMinRange;
  WbSFDouble *mMaxRange;
  WbSFDouble *mResolution;
  WbSFDouble *mDefaultFrequency;
  WbSFDouble *mMinFrequency;
  WbSFDouble *mMaxFrequency;
  WbSFString *mType;
  WbSFNode *mRotatingHead;

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

  WrRenderable *mFrustumRenderable;
  WrMaterial *mFrustumMaterial;
  WrStaticMesh *mFrustumMesh;

  WrRenderable *mLidarPointsRenderable;
  WrDynamicMesh *mLidarPointsMesh;
  WrMaterial *mLidarPointsMaterial;

  WrRenderable *mLidarRaysRenderable;
  WrDynamicMesh *mLidarRaysMesh;
  WrMaterial *mLidarRaysMaterial;

  // private functions
  void addConfigureToStream(WbDataStream &stream, bool reconfigure = false) override;

  void copyAllLayersToMemoryMappedFile();
  void updatePointCloud(int minWidth, int maxWidth);
  float *lidarImage() const;

  // R5e: off-screen wgpu range path for the Lidar device node. Opt-in via
  // OMNISIM_LIDAR_WGPU=1; renders radial range (clearAndDrawSceneRangeF32) and
  // angular-resamples it into `data` (uniform-angle, as updatePointCloud
  // expects). Gated to non-rotating, single-layer, narrow FOV (a single
  // perspective frustum); returns false otherwise so the caller keeps the WREN
  // path. See engine-migration-plan.md R5 WbLidar implementation spec. Owned +
  // destroyed here; null on the legacy WREN path.
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
  // WREN-oracle diff). Opt-in via OMNISIM_LIDAR_WGPU=1; gated to single-layer
  // narrow-FOV this increment, returns false otherwise (or on any wgpu miss) so
  // the WREN copy is kept.
  bool renderRotatingWindowViaWgpu(float *tempImage);
  // R5l: wide-FOV variant of the above. When the rotating window's fov exceeds a
  // single frustum (> 1.4 rad), renderRotatingWindowViaWgpu dispatches here: the
  // window azimuth span is split into N = ceil(fov/1.2) sub-frustums (the R5g
  // multi-frustum stitch), each rendered at the sensor pose yawed by
  // mPreviousRotatingAngle + the per-frustum centre θ_k (+tilt), then stitched by
  // azimuth into tempImage's height()×width() layout (per-row elevation as R5k).
  // Returns false on any wgpu miss → WREN fallback.
  bool renderRotatingWideFovWindowViaWgpu(float *tempImage);
  WbWgpuRenderTarget *mWgpuTarget;
  WbWgpuMeshCache *mWgpuMeshCache;
  int mWgpuTargetWidth;
  int mWgpuTargetHeight;

  WbLidar &operator=(const WbLidar &);  // non copyable
  WbNode *clone() const override { return new WbLidar(*this); }
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

  // WREN methods
  void createWrenCamera() override;
  void deleteWren();
  void displayPointCloud();
  void hidePointCloud();
  void applyMaxRangeToWren();
  void applyResolutionToWren();
  void applyTiltAngleToWren();

private slots:
  void updateOrientation();
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
  void updateBoundingSphere(WbBaseNode *subNode);
  void applyCameraSettingsToWren() override;
  void applyFrustumToWren() override;
  void updateOptionalRendering(int option) override;
  void updateFieldOfView() override;
};

#endif  // WB_LIDAR_HPP
