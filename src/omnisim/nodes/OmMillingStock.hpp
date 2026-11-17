// Copyright 2026 OmniLink
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

#ifndef OM_MILLING_STOCK_HPP
#define OM_MILLING_STOCK_HPP

// MillingStock: a block of material carved by a CNC cutter (resources/nodes/MillingStock.wrl).
//
// WHY A NODE AND NOT A SUPERVISOR-EDITED ElevationGrid. Measured 2026-10-31 on a
// 180 mm block with a 6.35 mm cutter moving at 50 mm/s: pushing the edits costs the
// controller nothing (0.05-0.15 ms/step), but every ElevationGrid height edit makes
// the engine re-hash and rebuild the WHOLE grid and re-walk the scene, and each
// edited grid becomes a new content-keyed GPU mesh that is never evicted. At 1 mm
// the window ran at 0.51x real time, at 0.5 mm 0.31x and out of GPU memory, at
// 0.25 mm 0.07x. Cost has to scale with the area the cutter touched, not with the
// block. So: the height map lives here, is carved here every physics step from the
// cutter Solid's real pose, and is drawn as TILES through the renderer's dynamic-
// draw path (OmWgpuSceneRenderer::collectDynamicDraws) -- only a tile the cutter
// changed is re-uploaded, into the buffer it already has, and nothing re-collects.

#include "OmBaseNode.hpp"
#include "OmVector3.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

class OmSFBool;
class OmSFColor;
class OmSFDouble;
class OmSFString;
class OmSFVector2;
class OmSFVector3;
class OmSolid;

class OmMillingStock : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmMillingStock(OmTokenizer *tokenizer = NULL);
  OmMillingStock(const OmMillingStock &other);
  explicit OmMillingStock(const OmNode &other);
  virtual ~OmMillingStock();

  int nodeType() const override { return WB_NODE_MILLING_STOCK; }
  void preFinalize() override;
  void postFinalize() override;

  // ---- the registry the renderer reads (same pattern as OmGranularBed) -----------
  static bool anyMillingStocks();
  static const std::vector<OmMillingStock *> &liveStocks();

  // ---- renderer contract: the block is drawn as tiles of at most kTile x kTile cells.
  // Each tile is one streamed mesh with a FIXED index buffer; `tileRevision(t)` moves
  // only when the cutter changed that tile, so the renderer re-uploads only those.
  // Vertices are WORLD space, stride 32: pos3 + norm3 + uv2. uv.x selects the colour
  // from the texture `wgpuColorTexels`: 2x1 (0.25 = skin, 0.75 = core) for a cutter, a
  // kCharTexels x 1 ramp from skin to char for a laser (uv.x = how charred the vertex is).
  //
  // A FLAT tile -- every height in it and in the one-cell ring around it equal, which is
  // an untouched tile or one wholly inside a flat pocket floor -- is drawn as ONE quad
  // (4 vertices, 2 triangles) instead of kTile^2 x 2 triangles: those carry no shape.
  // Measured 2026-11-01 on a 180 mm block at 0.2 mm: before this, every tile was drawn at
  // full resolution (~2.1 M triangles, ~4x that with the cascaded shadow passes) and a
  // star-pocket job held the RTX 3060 Laptop GPU at 55 % mean, 68.7 W.
  static const int kTile = 64;
  int tileCount() const { return mTilesX * mTilesY; }
  double tileRevision(int tile) const { return static_cast<double>(mTileRevision[tile]); }
  const std::vector<unsigned int> &tileIndices(int tile) const;
  bool tileVertexStream(int tile, std::vector<unsigned char> &out) const;
  int fullTiles() const;  // tiles currently drawn at full resolution (telemetry, tests)
  // The four walls and the bottom, as one more streamed mesh (they follow the edge
  // heights, so a cut through the side of the block shows on the side).
  double sidesRevision() const { return static_cast<double>(mSidesRevision); }
  const std::vector<unsigned int> &sidesIndices() const { return mSidesIndices; }
  bool sidesVertexStream(std::vector<unsigned char> &out) const;
  static const int kCharTexels = 16;
  bool isLaser() const;
  int colorTexelCount() const { return isLaser() ? kCharTexels : 2; }
  // RGBA8 texels (colorTexelCount() of them) and their mean in linear light.
  void wgpuColorTexels(std::vector<unsigned char> &rgba, float meanLinear[3]) const;
  uint64_t colorRevision() const { return mColorRevision; }
  float roughness() const;
  bool castShadows() const;
  bool isReady() const { return !mHeights.empty(); }

  // ---- state a supervisor can read through the hidden fields as well ----------------
  double removedVolume() const { return mRemovedVolume; }
  double depositedEnergy() const { return mDeposited; }

private slots:
  void onPhysicsStepEnded();
  void rebuild();
  void resolveTool();
  void bumpColorRevision() { ++mColorRevision; }

private:
  OmMillingStock &operator=(const OmMillingStock &);  // non copyable
  OmNode *clone() const override { return new OmMillingStock(*this); }
  void init();

  // Carve along the segment a -> b (stock-local metres, z from the block's bottom).
  void sweep(const OmVector3 &a, const OmVector3 &b);
  void stamp(double cx, double cy, double tipZ);
  double toolSurface(double d, double tipZ) const;  // cutter bottom at radial distance d
  bool isCut(int i, int j) const;
  // Laser: deposit `energy` joules as the elliptical Gaussian spot centred at (cx, cy) with
  // its focal point at height focusZ; the spot widens with the surface's distance from focus.
  bool burn(double cx, double cy, double focusZ, double energy);
  void burnSweep(const OmVector3 &a, const OmVector3 &b, double energy);
  void touchTiles(int i0, int i1, int j0, int j1);
  float charLevel(int i, int j) const;  // 0 untouched .. 1 fully charred
  // OMNISIM_MILLING_STOCK_DUMP=<file>: the height map, rewritten at most once a second
  // while the cutter is removing material, so a test can measure the part it made.
  void dumpHeights();
  OmVector3 normalAt(int i, int j) const;

  OmSFVector3 *mTranslation;
  OmSFVector3 *mSize;
  OmSFDouble *mResolution;
  OmSFString *mToolSolid;
  OmSFVector3 *mToolTip;
  OmSFString *mToolShape;
  OmSFDouble *mToolDiameter;
  OmSFDouble *mToolAngle;
  OmSFVector2 *mLaserSpot;
  OmSFDouble *mBeamHalfAngle;
  OmSFDouble *mLaserPower;
  OmSFDouble *mAblationEnergy;
  OmSFDouble *mCharFluence;
  OmSFColor *mCharColor;
  OmSFDouble *mDepositedEnergyField;
  OmSFBool *mCutting;
  OmSFColor *mCoreColor;
  OmSFColor *mSkinColor;
  OmSFDouble *mSkinDepth;
  OmSFDouble *mRoughness;
  OmSFBool *mCastShadows;
  OmSFDouble *mRemovedVolumeField;
  OmSFDouble *mDeepestCutField;

  OmSolid *mTool;   // cleared by the Solid's destroyed() signal (resolveTool)
  bool mWarnedNoTool;

  // Height map: (mNx x mNy) vertex heights above the block's bottom, metres.
  int mNx, mNy;
  double mRes, mTop;
  OmVector3 mOrigin;
  std::vector<float> mHeights;
  // Laser only: absorbed fluence per vertex (J/m^2), the char it has built up.
  std::vector<float> mFluence;
  std::vector<double> mSpotWeights;   // burn() scratch
  int mTilesX, mTilesY;
  std::vector<uint64_t> mTileRevision;
  std::vector<std::vector<unsigned int>> mTileIndices;
  // Flatness is re-judged lazily, once per tile revision, when the renderer asks.
  bool tileIsFlat(int tile) const;
  mutable std::vector<uint64_t> mFlatJudgedAt;
  mutable std::vector<char> mTileFlat;
  std::vector<unsigned int> mQuadIndices;
  uint64_t mSidesRevision;
  std::vector<unsigned int> mSidesIndices;
  uint64_t mColorRevision;

  bool mHaveLastTip;
  OmVector3 mLastTip;
  double mRemovedVolume;
  double mDeepest;
  double mDeposited;

  std::string mDumpPath;
  bool mDumpDirty;
  bool mDumpedOnce;
  std::chrono::steady_clock::time_point mDumpedAt;
};

#endif
