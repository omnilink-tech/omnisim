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

#ifndef OM_CAMERA_HPP
#define OM_CAMERA_HPP

#include "OmAbstractCamera.hpp"

class OmAffinePlane;
class OmDownloader;
class OmFocus;
class OmLensFlare;
class OmWrenLabelOverlay;
class OmRecognition;
class OmRecognizedObject;
class OmRenderBackend;
class OmWgpuMeshCache;
class OmWgpuRenderTarget;
class OmWgpuTextureCache;
class OmZoom;

class OmCamera : public OmAbstractCamera {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmCamera(OmTokenizer *tokenizer = NULL);
  OmCamera(const OmCamera &other);
  explicit OmCamera(const OmNode &other);
  virtual ~OmCamera() override;

  // reimplemented public functions
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  void handleMessage(QDataStream &) override;
  int nodeType() const override { return WB_NODE_CAMERA; }
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void reset(const QString &id) override;
  void resetMemoryMappedFile() override;
  bool isEnabled() const override;

  // specific functions
  QString pixelInfo(int x, int y) const override;
  void updateRecognizedObjectsOverlay(double screenX, double screenY, double overlayX, double overlayY);
  void clearRecognizedObjectsOverlay();
  OmRgb enabledCameraFrustrumColor() const override { return OmRgb(1.0f, 0.0f, 1.0f); }

  // renderBackend: resolves this Camera's RTT renderer through OmRenderBackendRegistry.
  // F1 (wren-deletion-runbook.md): always the wgpu backend when wgpu-native is available --
  // "wren" is retired, warns once per node, and resolves to wgpu; the WREN backend is
  // returned only as the wgpu-native-unavailable last resort (deleted at D1.4). The string
  // accessor returns the raw field value for diagnostic logging.
  const QString &renderBackendName() const;
  OmRenderBackend *renderBackend() const;

protected:
  OmVector3 urdfRotation(const OmMatrix3 &rotationMatrix) const override;
  void setup() override;
  void render() override;
  bool needToRender() const override;
  void exportNodeFields(OmWriter &writer) const override;
  QStringList customExportedFields() const override;

private:
  OmSFNode *mFocus;
  OmSFNode *mZoom;
  OmSFNode *mRecognition;
  OmSFString *mNoiseMaskUrl;
  OmSFBool *mAntiAliasing;
  OmSFDouble *mAmbientOcclusionRadius;
  OmSFDouble *mBloomThreshold;
  OmSFNode *mLensFlare;
  OmSFDouble *mFar;
  OmSFDouble *mExposure;
  OmSFString *mRenderBackend;

  // R3.3b: lazy-built off-screen wgpu render target. Allocated on
  // first writeAnswer after a Camera with `renderBackend "wgpu"` is
  // resolved through the dispatcher. Rebuilt if width/height
  // change. Owned + destroyed by this Camera; nullptr when the
  // Camera uses the legacy WREN path or when wgpu isn't available.
  OmWgpuRenderTarget *mWgpuTarget;
  int mWgpuTargetWidth;
  int mWgpuTargetHeight;

  // R3.4-step-4: per-Camera mesh cache for the wgpu scene-walk
  // pipeline. Keyed by a stable mesh id through OmWgpuMeshAdapter
  // (D1.4: the WrStaticMesh* keying died with WREN) so the same Box
  // geometry shared across N Solids only uploads its vertex/index
  // buffers once. Same lifetime as mWgpuTarget — both die when the
  // Camera resizes or is destroyed.
  OmWgpuMeshCache *mWgpuMeshCache;

  // W3: per-Camera ALBEDO texture cache for the sensor scene render.
  //
  // Without one, OmWgpuSceneRenderer::collectWorldDraws is called with texCache == nullptr and
  // every draw is flat-shaded from its PBRAppearance baseColor alone. On a real world that is
  // the measured MONOCHROME sensor frame: most production materials leave baseColor at its
  // (1,1,1) default and carry all their colour in baseColorMap, so dropping the maps collapses
  // the whole scene to shades of one grey (beauty_bench through the capture service:
  // R = G = B = 66.8, std 29.5, unchanged after 120 settle steps).
  //
  // Keyed by OmWgpuSceneRenderer's stable texture id, so a texture shared by N Solids uploads
  // once. Lifetime: created lazily on the first scene render, destroyed with the Camera. It is
  // NOT dropped on resize (unlike mWgpuTarget/mWgpuMeshCache): its entries live on the
  // process-global OmVulkanBackend device, which a resize does not replace, and re-uploading
  // every texture because a Camera changed size would be pure waste.
  OmWgpuTextureCache *mWgpuTextureCache;

  // Lane E1 (P5): motion-blur feedback history for the CPU post-FX port
  // (applyWgpuColorPostFx) -- the previous blurred-but-not-noised BGRA frame, i.e. the WREN
  // motion-blur pass's feedback texture. Self-(re)sizing on resolution change inside
  // applyWgpuColorPostFx; empty = firstRender semantics.
  std::vector<unsigned char> mWgpuBlurHistory;

  // One-shot per Camera: the scene-render path fired and reported what it harvested (draw count,
  // sun, sky). Per Camera rather than per process so a world with several wgpu Cameras shows one
  // line each instead of the first four cameras' worth.
  bool mWgpuScenePathLogged;

  // Returns true if this camera should route its image through wgpu
  // and the wgpu backend is currently usable. Side effect: lazily
  // (re)builds mWgpuTarget at the current width()/height().
  bool ensureWgpuTarget();

  // W3 defect 1 -- R/B channel swap on the wgpu SENSOR path.
  //
  // Every wgpu render target is WGPUTextureFormat_RGBA8Unorm (fixed, so the
  // readback can memcpy 4 bytes per pixel -- see OmWgpuRenderTarget's ctor
  // contract), but the CONTROLLER image contract is BGRA:
  // wb_camera_image_get_red reads byte 2 and wb_camera_image_get_blue reads
  // byte 0 (include/controller/c/omnisim/camera.h:87-89). So an RGBA8 readback
  // memcpy'd straight into the device buffer hands every controller red and
  // blue swapped -- measured: the cyan box (r=0, g=1, b=1) of
  // camera_wgpu_scene_smoke.omniworld read back as bytes (0,141,141), i.e. a
  // controller computed red=141 from a box with no red in it at all.
  //
  // Fixed HERE, in the sensor copy, and DELIBERATELY nowhere else: the MAIN
  // VIEW shares the same RGBA8 target format and renders correctly on screen
  // today, so neither the target format nor any shared shader may change.
  //
  // Value-parsed hatch OMNISIM_WGPU_CAMERA_BGRA, default ON (= corrected);
  // =0 / false / off / no restores the pre-fix swapped bytes exactly, so the
  // old regression expectations still have an arm to run under.
  //
  // Operates on mWgpuTargetWidth * mWgpuTargetHeight RGBA8 texels, which is
  // exactly this Camera's width()*height() whenever ensureWgpuTarget()
  // succeeded. A no-op in effect on a grayscale (depth) readback, where
  // r == g == b. Never called on the float RangeFinder/Lidar buffers.
  void swizzleWgpuRgbaToBgra(unsigned char *data) const;

  // One-shot guard for the `far 0` -> finite-far substitution INFO (W3 defect
  // 2), so the substitution is observable in the log without spamming it every
  // frame. Per Camera, reset by init().
  bool mWgpuFarSubstitutionLogged;

  // F2.3 (wren-deletion-runbook): one-shot guard for the non-planar-projection DECLINE
  // warning. The wgpu sensor path is a planar pinhole (buildViewProj = tan(fov/2), so a
  // cylindrical/spherical fov >= pi degenerates); rather than render confidently-wrong
  // pixels the arm stands down for non-planar devices. Per Camera, reset by init().
  bool mWgpuNonPlanarDeclineWarned;

  // private functions
  void addConfigureToStream(OmDataStream &stream, bool reconfigure = false) override;

  OmFocus *focus() const;
  OmZoom *zoom() const;
  OmRecognition *recognition() const;
  OmLensFlare *lensFlare() const;

  OmCamera &operator=(const OmCamera &);  // non copyable
  OmNode *clone() const override { return new OmCamera(*this); }
  void init();
  void initializeImageMemoryMappedFile() override;

  // D1.4 (WREN deletion): the wgpu render IS the main-image path — there is no WREN copy
  // arm any more. Non-planar projections decline (honestly-empty image, warned once); the
  // segmentation image is produced nowhere (see writeAnswer) and never reaches this copy.
  void copyImageToMemoryMappedFile(unsigned char *data) override;

  int size() const override { return 4 * width() * height(); }
  double minRange() const override { return mNear->value(); }
  double maxRange() const override { return mFar->value(); }
  bool antiAliasing() const override { return mAntiAliasing->value(); }
  bool isFrustumEnabled() const override;

  // smart camera
  bool refreshRecognitionSensorIfNeeded();
  // Newton raycast service consumer: answer every recognition candidate's
  // occlusion rays from the live mjModel via
  // OmObjectDetection::refreshCollisionDepthsFromNewton, behind the value-parsed
  // OMNISIM_NEWTON_RAYCAST gate (default ON). Returns false when the occlusion
  // question could NOT be answered (gate off, Newton runtime unavailable, or the
  // service declined) -- the caller must then leave the recognized-object list
  // alone rather than declaring every object visible.
  bool refreshRecognizedObjectsOcclusionFromNewton();
  void removeOccludedRecognizedObjects();
  OmVector2 projectOnImage(const OmVector3 &position);
  OmVector2 applyCameraDistortionToImageCoordinate(const OmVector2 &uv);  // uv coordinates expected in range [0, 1]
  void computeRecognizedObjects();
  bool setRecognizedObjectProperties(OmRecognizedObject *recognizedObject);
  void updateRaysSetupIfNeeded() override;
  short mRecognitionRefreshRate;
  bool mNeedToDeleteRecognizedObjectsRays;
  OmSensor *mRecognitionSensor;
  OmWrenLabelOverlay *mLabelOverlay;
  QList<OmRecognizedObject *> mRecognizedObjects;
  QList<OmRecognizedObject *> mInvalidRecognizedObjects;
  QVector<int> mNewtonExcludeBodies;  // cached own-robot Newton bodies ([-999] = built, empty)
  bool mIsSubscribedToRayTracing;
  // smart camera segmentation
  // D1.4 (WREN deletion): the segmentation IMAGE was rendered by a WREN camera and is now
  // produced nowhere. The enable/disable protocol, the memory-mapped file and the stream
  // chunks all survive (so wb_camera_recognition_enable_segmentation keeps its framing);
  // the image bytes stay black and mSegmentationWrenWarned latches the once-per-device
  // warning that says so. The recognition OBJECT LIST is unaffected (it rides Newton rays).
  void initializeSegmentationMemoryMappedFile();
  void createSegmentationCamera();
  void warnSegmentationImageEmpty();
  bool mSegmentationEnabled;
  bool mSegmentationChanged;
  bool mSegmentationWrenWarned;
  OmMemoryMappedFile *mSegmentationMemoryMappedFile;
  bool mHasSegmentationMemoryMappedFileChanged;
  bool mSegmentationImageChanged;
  // URL downloader
  OmDownloader *mDownloader;

private slots:
  void updateFocus();
  void updateRecognition();
  void updateSegmentation();
  void updateNear();
  void updateFar();
  void updateExposure();
  void updateAmbientOcclusionRadius();
  void updateBloomThreshold();
  void updateNoiseMaskUrl();
  // D1.4: kept (name and all) as the focus-changed reconfigure hook — the WREN
  // depth-of-field application is gone, but wb_camera_set_focal_distance still needs the
  // controller reconfigure this slot triggers.
  void applyFocalSettingsToWren();
};

#endif  // OM_CAMERA_HPP
