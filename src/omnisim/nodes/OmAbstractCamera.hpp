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

#ifndef OM_ABSTRACT_CAMERA_HPP
#define OM_ABSTRACT_CAMERA_HPP

#include "OmRenderingDevice.hpp"
#include "OmSensor.hpp"

#include <array>
#include <vector>

class OmWgpuMeshCache;
class OmWgpuTextureCache;
struct OmWgpuSolidDraw;


class OmLens;

#ifdef _WIN32
class QSharedMemory;
#define OmMemoryMappedFile QSharedMemory
#else
class OmPosixMemoryMappedFile;
#define OmMemoryMappedFile OmPosixMemoryMappedFile
#endif

class QDataStream;

class OmAbstractCamera : public OmRenderingDevice {
  Q_OBJECT

public:
  // constructors and destructor
  OmAbstractCamera(const QString &modelName, OmTokenizer *tokenizer = NULL);
  OmAbstractCamera(const OmAbstractCamera &other);
  OmAbstractCamera(const OmNode &other);
  virtual ~OmAbstractCamera() override;

  // reimplemented public functions
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  void reset(const QString &id) override;

  virtual void updateCameraTexture();

  void externControllerChanged() { mHasExternControllerChanged = true; }
  void newRemoteExternController() { mIsRemoteExternController = true; }
  void removeRemoteExternController() { mIsRemoteExternController = false; }
  void enableExternalWindowForAttachedCamera(bool enabled);

  void setNodesVisibility(QList<const OmBaseNode *> nodes, bool visible);

  virtual bool isEnabled() const { return mSensor ? mSensor->isEnabled() : false; }

  // external window
  void enableExternalWindow(bool enabled) override;
  virtual bool isRangeFinder() { return false; }
  bool isPlanarProjection() const { return mProjection->value() == "planar"; }
  virtual double minRange() const = 0;
  virtual double maxRange() const { return 1.0; }
  virtual double nearValue() const { return mNear->value(); }  // near is a reserved keyword on Windows
  virtual double fieldOfView() const { return mFieldOfView->value(); }

  virtual void resetMemoryMappedFile();

  // static functions
  static int cCameraNumber;
  static int cCameraCounter;
  static void resetStaticCounters() { cCameraNumber = 0; }

  const unsigned char *constImage() const { return image(); }

signals:
  void enabled(OmAbstractCamera *camera, bool isActive);

protected:
  void setup() override;
  virtual void render(){};
  virtual bool needToRender() const;

  // WREN-retirement: true when THIS node's own `renderBackend` field resolves to an
  // AVAILABLE wgpu backend. Read generically off the field so this base class stays free of
  // any subclass dependency: since Lane E1 Camera, RangeFinder AND Lidar all declare
  // `renderBackend` (a hypothetical field-less subclass answers false). Mirrors
  // OmCamera::renderBackend() exactly -- same WorldInfo.defaultRenderBackend deferral for an
  // empty/"auto" value, and post-F1 the same registry behaviour: wgpu for every value (a
  // retired "wren" warns once per node), WREN only as the wgpu-native-unavailable last resort.
  bool resolvesToWgpuBackend() const;
  // One-shot per device: nothing produced pixels for this frame while set up wrenless, so
  // the controller is about to read a frozen buffer. Named, not silent.
  void warnWrenlessNoImage();
  // F1 (WREN unselectable): warn once per node -- naming the node, via warn() -- when the
  // renderBackend value in play is the retired "wren", saying the device renders wgpu.
  // `fromWorldInfo` selects the wording for a value supplied by WorldInfo.defaultRenderBackend
  // rather than authored on this node. Const because the resolution paths that detect the
  // value are const (the latch is mutable).
  void warnIfRetiredWrenBackend(const QString &value, bool fromWorldInfo) const;

  // ---- THE SENSOR SCENE COLLECT (P1/P9) -----------------------------------
  //
  // ⚠ EVERY wgpu SENSOR RENDER MUST GO THROUGH THIS, NOT collectWorldDraws DIRECTLY.
  //
  // collectWorldDraws walks Solids, and three node types that a WREN-rendered sensor
  // has always SEEN are not Solids: Cloth, SoftBody and GranularGroup (all three set
  // VM_REGULAR, which every sensor visibility mask intersects). Each therefore needs
  // its own collect appended after the Solid walk. Before this existed, the sensor
  // sites called collectWorldDraws alone and the deformable collect had exactly one
  // caller in the tree — the main view — so a cloth rendered on screen and was
  // invisible in every Camera / RangeFinder / Lidar image.
  //
  // The point of putting it HERE, on the shared base of all three sensor devices, is
  // that the failure was structural: eight call sites, one of which knew about the
  // dynamic content. Now the sensor sites are one function, and the dynamic content
  // is one function (OmWgpuSceneRenderer::collectDynamicDraws) — so a new node type
  // reaches every sensor by construction, and a new sensor gets every node type the
  // same way.
  //
  // `texCache` is forwarded verbatim (null on the depth/range paths, which want no
  // albedo). Hatch: OMNISIM_WGPU_SENSOR_DYNAMIC=0 collects Solids only, reproducing
  // the pre-P1 sensor image exactly while leaving the main view alone.
  void collectWgpuDraws(OmWgpuMeshCache &cache, std::vector<OmWgpuSolidDraw> &out,
                        std::vector<std::array<float, 16>> &modelStorage,
                        OmWgpuTextureCache *texCache = NULL);

  // user accessible fields
  OmSFDouble *mFieldOfView;
  OmSFString *mProjection;
  OmSFDouble *mNear;
  OmSFDouble *mMotionBlur;
  OmSFDouble *mNoise;
  OmSFNode *mLens;

  // private functions
  virtual void addConfigureToStream(OmDataStream &stream, bool reconfigure = false);
  bool handleCommand(QDataStream &stream, unsigned char command);

  unsigned char *image() const { return mImageData; }
  OmLens *lens() const;

  virtual OmRgb enabledCameraFrustrumColor() const = 0;
  virtual OmRgb disabledCameraFrustrumColor() const { return OmRgb(0.5f, 0.5f, 0.5f); }

  void init();
  virtual void initializeImageMemoryMappedFile();
  OmMemoryMappedFile *initializeMemoryMappedFile(const QString &id = "");
  virtual void computeValue();
  // Virtual so subclasses (OmCamera) can intercept the main-image copy path and route
  // through the wgpu backend. The base impl warns: reaching it means nothing wrote pixels.
  virtual void copyImageToMemoryMappedFile(unsigned char *data);
  void editChunkMetadata(OmDataStream &stream, int newImageSize);

  virtual bool antiAliasing() const { return false; }

  virtual int size() const = 0;


  // Wren methods
  // See the .cpp: false = the fatal wrenless envelope (unchanged), true also reports the
  // post-FX that degrade SILENTLY once a wgpu Camera owns the image.
  // Virtual (Lane E1 / P5): OmLidar overrides it -- the wgpu Lidar pipeline is
  // cylindrical-by-construction, so the base class's planar-only projection test is exactly
  // backwards for it.
  virtual QString wgpuUnsupportedFeature(bool includeSilentPostFx) const;
  // Called by setup() at the point it commits to a WRENLESS (OMNISIM_NO_GL + wgpu) device,
  // INSTEAD of createWrenCamera(). Subclasses that latch state inside createWrenCamera()
  // (OmLidar's mActual* copies) must latch it here too. Default: nothing.
  virtual void setupWrenless() {}

  // Lane E1 (P5+P6): CPU ports of the WREN post-FX shaders for images produced by the wgpu
  // sensor arms. Formulas measured from resources/wren/shaders/ (range_noise.frag,
  // depth_resolution.frag, color_noise.frag, motion_blur.frag) -- see the .cpp. All of them
  // are gated on OMNISIM_WGPU_SENSOR_POSTFX (value-parsed; unset/1 = ON, =0 = exact revert
  // to the pre-port wgpu output, with the silent-post-FX warning restored).
  //
  // NOT bit-exact with WREN and not meant to be: the shader's rand() runs through a
  // driver-specific float32 sin(), so two GPUs already disagree bitwise. The port preserves
  // the STATISTICS (additive zero-mean Gaussian, sigma exactly as authored, stateless
  // per-frame seed from the simulation clock) and the range semantics.
  static bool wgpuSensorPostFxEnabled();
  // Range image post-FX (Lidar + RangeFinder), applied in WREN's pass order:
  // 1. noise > 0 (range_noise.frag): value in the OPEN interval (minRange, maxRange) ->
  //    value + noise * N(0,1) [sigma in ABSOLUTE METRES]; anything outside the interval --
  //    including the background at exactly maxRange -- becomes +inf. This deliberately
  //    matches WREN over the wgpu arm's no-noise clamp (see the .cpp note).
  // 2. resolution > 0 (depth_resolution.frag): floor(v / resolution + 0.5) * resolution.
  // `w`/`h` only decorrelate the per-sample hash (uv), `count` = w*h samples in `img`.
  void applyWgpuRangePostFx(float *img, int w, int h, double resolution) const;
  // Color image post-FX for the BGRA controller buffer (already swizzled):
  // 1. motionBlur > 0 (motion_blur.frag): out = mix(scene, history, blend) with
  //    blend = pow(0.005, refreshRate_ms / motionBlur_ms); history = this pass's own
  //    previous OUTPUT (feedback), held in `history` (resized/reset here on size change).
  //    DEVIATION from WREN, documented: WREN blends HDR linear PRE-tonemap; this blends the
  //    final post-tonemap LDR bytes. Same time constant, slightly different ghost falloff.
  // 2. noise > 0 (color_noise.frag): per-channel independent additive Gaussian
  //    (sigma = noise in normalised channel units), alpha untouched, then clamp to [0,255]
  //    -- the clamp is load-bearing (asymmetric near black/white), exactly as the shader's
  //    clamp(0,1).
  void applyWgpuColorPostFx(unsigned char *bgra, int w, int h, std::vector<unsigned char> &blurHistory) const;
  void createWrenOverlay() override;
  void deleteWren();
  virtual bool isFrustumEnabled() const { return false; }



  QList<const OmBaseNode *> mInvisibleNodes;

  // other stuff
  OmSensor *mSensor;
  short mRefreshRate;
  OmMemoryMappedFile *mImageMemoryMappedFile;
  unsigned char *mImageData;
  char mCharType;
  bool mNeedToConfigure;
  bool mSendMemoryMappedFile;
  bool mHasExternControllerChanged;
  bool mIsRemoteExternController;
  bool mImageChanged;

  // WREN-retirement: set by setup() when it completed with NO OpenGL context, which it only
  // does for a device whose resolved render backend is wgpu. In that state every WREN handle
  // on this node stays NULL -- no OmWrenCamera, no OmWrenTextureOverlay, no frustum mesh --
  // so each wr_* / mWrenCamera call site below is gated on it. It is ALWAYS false whenever a
  // GL context exists, which is what keeps the WREN path byte-identical.
  bool mWrenlessSetup;
  bool mWrenlessNoImageWarned;
  // F1 (WREN unselectable): once-per-node latch for the "renderBackend \"wren\" is retired,
  // this device renders wgpu" warning. Mutable because the renderBackend accessors that
  // detect the retired value are const. Shared by resolvesToWgpuBackend() and
  // OmCamera::renderBackend(), so one node warns once no matter which path reads it first.
  mutable bool mWarnedRetiredWrenBackend = false;

  bool mNeedToCheckShaderErrors;

  bool mMemoryMappedFileReset;

  bool mExternalWindowEnabled;

public slots:
  void updateAntiAliasing();

protected slots:
  // update methods
  void updateWidth() override;
  void updateHeight() override;
  virtual void updateFieldOfView();
  void updateProjection();
  void updateMotionBlur();
  void updateNoise();
  void updateLens();
  void removeInvisibleNodeFromList(QObject *node);

private:
  OmAbstractCamera &operator=(const OmAbstractCamera &);  // non copyable
};

#endif  // OM_ABSTRACT_CAMERA_HPP
