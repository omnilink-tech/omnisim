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

#include "OmMillingStock.hpp"

#include "OmLog.hpp"
#include "OmMatrix4.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFString.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace {
  const double kDefaultResolution = 0.0002;
  const double kMinResolution = 0.00005;
  // Cap on cells per axis: keeps the tile count (kTile cells per tile edge) under the
  // 1023 tile slots the renderer's mesh-key space reserves for one stock.
  const int kMaxCellsPerAxis = OmMillingStock::kTile * 31;
  // A cutter that moves farther than this in ONE physics step was teleported (a world
  // reset, a supervisor pose edit), not driven: carve where it lands, not along the jump.
  const double kTeleportDistance = 0.05;
  const int kMaxStampsPerStep = 4000;

  void putFloat(unsigned char *&p, float v) {
    std::memcpy(p, &v, sizeof(float));
    p += sizeof(float);
  }
}  // namespace

static std::vector<OmMillingStock *> gLiveMillingStocks;

bool OmMillingStock::anyMillingStocks() {
  return !gLiveMillingStocks.empty();
}

const std::vector<OmMillingStock *> &OmMillingStock::liveStocks() {
  return gLiveMillingStocks;
}

void OmMillingStock::init() {
  mTranslation = findSFVector3("translation");
  mSize = findSFVector3("size");
  mResolution = findSFDouble("resolution");
  mToolSolid = findSFString("toolSolid");
  mToolTip = findSFVector3("toolTip");
  mToolShape = findSFString("toolShape");
  mToolDiameter = findSFDouble("toolDiameter");
  mToolAngle = findSFDouble("toolAngle");
  mLaserSpot = findSFVector2("laserSpot");
  mBeamHalfAngle = findSFDouble("beamHalfAngle");
  mLaserPower = findSFDouble("laserPower");
  mAblationEnergy = findSFDouble("ablationEnergy");
  mCharFluence = findSFDouble("charFluence");
  mCharColor = findSFColor("charColor");
  mDepositedEnergyField = findSFDouble("depositedEnergy");
  mCutting = findSFBool("cutting");
  mCoreColor = findSFColor("coreColor");
  mSkinColor = findSFColor("skinColor");
  mSkinDepth = findSFDouble("skinDepth");
  mRoughness = findSFDouble("roughness");
  mCastShadows = findSFBool("castShadows");
  mRemovedVolumeField = findSFDouble("removedVolume");
  mDeepestCutField = findSFDouble("deepestCut");

  mWarnedNoTool = false;
  mNx = mNy = 0;
  mRes = kDefaultResolution;
  mTop = 0.0;
  mTilesX = mTilesY = 0;
  mSidesRevision = 1;
  mColorRevision = 1;
  mHaveLastTip = false;
  mRemovedVolume = 0.0;
  mDeepest = 0.0;
  mDeposited = 0.0;
  const char *dump = std::getenv("OMNISIM_MILLING_STOCK_DUMP");
  mDumpPath = dump ? dump : "";
  mDumpDirty = false;
  mDumpedOnce = false;
  mTool = NULL;
}

OmMillingStock::OmMillingStock(OmTokenizer *tokenizer) : OmBaseNode("MillingStock", tokenizer) {
  init();
  gLiveMillingStocks.push_back(this);
}

OmMillingStock::OmMillingStock(const OmMillingStock &other) : OmBaseNode(other) {
  init();
  gLiveMillingStocks.push_back(this);
}

OmMillingStock::OmMillingStock(const OmNode &other) : OmBaseNode(other) {
  init();
  gLiveMillingStocks.push_back(this);
}

OmMillingStock::~OmMillingStock() {
  gLiveMillingStocks.erase(std::remove(gLiveMillingStocks.begin(), gLiveMillingStocks.end(), this),
                           gLiveMillingStocks.end());
}

void OmMillingStock::preFinalize() {
  OmBaseNode::preFinalize();
}

void OmMillingStock::postFinalize() {
  OmBaseNode::postFinalize();
  rebuild();
  connect(mSize, &OmSFVector3::changed, this, &OmMillingStock::rebuild);
  connect(mTranslation, &OmSFVector3::changed, this, &OmMillingStock::rebuild);
  connect(mResolution, &OmSFDouble::changed, this, &OmMillingStock::rebuild);
  connect(mToolSolid, &OmSFString::changed, this, &OmMillingStock::resolveTool);
  connect(mCoreColor, &OmSFColor::changed, this, &OmMillingStock::bumpColorRevision);
  connect(mSkinColor, &OmSFColor::changed, this, &OmMillingStock::bumpColorRevision);
  connect(mCharColor, &OmSFColor::changed, this, &OmMillingStock::bumpColorRevision);
  // A tool change between a cutter and the laser changes the colour texture and what each
  // vertex's uv means: re-stream everything (and give the laser its fluence map).
  const auto restyle = [this]() {
    if (isLaser() && mFluence.size() != mHeights.size())
      mFluence.assign(mHeights.size(), 0.0f);
    bumpColorRevision();
    for (uint64_t &r : mTileRevision)
      ++r;
    ++mSidesRevision;
  };
  connect(mToolShape, &OmSFString::changed, this, restyle);
  connect(mCharFluence, &OmSFDouble::changed, this, restyle);
  // skinDepth reclassifies every vertex: every tile has to be re-streamed.
  connect(mSkinDepth, &OmSFDouble::changed, this, [this]() {
    for (uint64_t &r : mTileRevision)
      ++r;
  });
  // physicsStepENDED, not Started: the cutter Solid's pose is then THIS step's Newton
  // result (OmSolid::postPhysicsStep has written it back), so a cut lands where the
  // tool is now, not one step behind.
  OmSimulationWorld *const world = OmSimulationWorld::instance();
  if (world != NULL)
    connect(world, &OmSimulationWorld::physicsStepEnded, this, &OmMillingStock::onPhysicsStepEnded);
}

void OmMillingStock::resolveTool() {
  if (mTool)
    disconnect(mTool, &QObject::destroyed, this, nullptr);
  mTool = NULL;
  mHaveLastTip = false;
  const QString name = mToolSolid->value();
  if (name.isEmpty() || OmWorld::instance() == NULL)
    return;
  mTool = OmWorld::instance()->findSolid(name);
  if (mTool)   // a deleted cutter must not leave a dangling pointer behind
    connect(mTool, &QObject::destroyed, this, [this]() { mTool = NULL; });
}

void OmMillingStock::rebuild() {
  mHeights.clear();
  mTileIndices.clear();
  mTileRevision.clear();
  mSidesIndices.clear();
  mHaveLastTip = false;
  mRemovedVolume = 0.0;
  mDeepest = 0.0;
  mDeposited = 0.0;
  mFluence.clear();
  const OmVector3 size = mSize->value();
  if (!(size.x() > 0.0 && size.y() > 0.0 && size.z() > 0.0)) {
    OmLog::warning(tr("MillingStock: 'size' must be strictly positive on all three axes; the block is empty."));
    return;
  }
  mRes = mResolution->value() > 0.0 ? std::max(mResolution->value(), kMinResolution) : kDefaultResolution;
  const double longest = std::max(size.x(), size.y());
  if (longest / mRes > kMaxCellsPerAxis) {
    const double coarser = longest / kMaxCellsPerAxis;
    OmLog::warning(tr("MillingStock: resolution %1 m gives more than %2 cells per side; using %3 m.")
                     .arg(mRes)
                     .arg(kMaxCellsPerAxis)
                     .arg(coarser));
    mRes = coarser;
  }
  mNx = static_cast<int>(std::lround(size.x() / mRes)) + 1;
  mNy = static_cast<int>(std::lround(size.y() / mRes)) + 1;
  mTop = size.z();
  mOrigin = mTranslation->value();
  mHeights.assign(static_cast<size_t>(mNx) * mNy, static_cast<float>(mTop));
  if (isLaser())
    mFluence.assign(mHeights.size(), 0.0f);

  mTilesX = (mNx - 1 + kTile - 1) / kTile;
  mTilesY = (mNy - 1 + kTile - 1) / kTile;
  mTileRevision.assign(static_cast<size_t>(mTilesX) * mTilesY, 1);
  mFlatJudgedAt.assign(mTileRevision.size(), 0);  // 0 never equals a revision: judge on first use
  mTileFlat.assign(mTileRevision.size(), 1);
  // A flat tile's quad: corners 0 (i0,j0), 1 (i1,j0), 2 (i0,j1), 3 (i1,j1), CCW from +Z.
  mQuadIndices = {0, 1, 3, 0, 3, 2};
  mTileIndices.resize(mTileRevision.size());
  for (int ty = 0; ty < mTilesY; ++ty)
    for (int tx = 0; tx < mTilesX; ++tx) {
      const int w = std::min(kTile, mNx - 1 - tx * kTile);  // cells
      const int h = std::min(kTile, mNy - 1 - ty * kTile);
      std::vector<unsigned int> &idx = mTileIndices[ty * mTilesX + tx];
      idx.reserve(static_cast<size_t>(w) * h * 6);
      for (int cj = 0; cj < h; ++cj)
        for (int ci = 0; ci < w; ++ci) {
          const unsigned int v00 = static_cast<unsigned int>(cj * (w + 1) + ci);
          const unsigned int v10 = v00 + 1, v01 = v00 + static_cast<unsigned int>(w + 1), v11 = v01 + 1;
          // Counter-clockwise seen from +Z (the renderer's front face).
          idx.insert(idx.end(), {v00, v10, v11, v00, v11, v01});
        }
    }

  // Sides: south, east, north, west, each a strip of (top, bottom) pairs walked
  // counter-clockwise around the block, then the bottom quad.
  const int sideLength[4] = {mNx, mNy, mNx, mNy};
  unsigned int base = 0;
  for (int s = 0; s < 4; ++s) {
    for (int k = 0; k + 1 < sideLength[s]; ++k) {
      const unsigned int t0 = base + 2 * k, b0 = t0 + 1, t1 = t0 + 2, b1 = t0 + 3;
      mSidesIndices.insert(mSidesIndices.end(), {b0, b1, t1, b0, t1, t0});
    }
    base += 2 * sideLength[s];
  }
  mSidesIndices.insert(mSidesIndices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
  ++mSidesRevision;
  mRemovedVolumeField->setValueFromOde(0.0);
  mDeepestCutField->setValueFromOde(0.0);
  mDepositedEnergyField->setValueFromOde(0.0);
  resolveTool();
}

// ---- carving ------------------------------------------------------------------------

double OmMillingStock::toolSurface(double d, double tipZ) const {
  const double radius = 0.5 * mToolDiameter->value();
  const QString &shape = mToolShape->value();
  if (shape == "ball")
    return tipZ + radius - std::sqrt(std::max(0.0, radius * radius - d * d));
  if (shape == "vbit") {
    const double half = 0.5 * std::min(std::max(mToolAngle->value(), 0.05), 3.0);
    return tipZ + d / std::tan(half);
  }
  return tipZ;  // flat end mill
}

void OmMillingStock::stamp(double cx, double cy, double tipZ) {
  if (tipZ >= mTop)
    return;  // every profile rises from its tip: a tip above the top cuts nothing
  const double radius = 0.5 * mToolDiameter->value();
  if (radius <= 0.0)
    return;
  const int i0 = std::max(0, static_cast<int>(std::floor((cx - radius) / mRes)));
  const int i1 = std::min(mNx - 1, static_cast<int>(std::ceil((cx + radius) / mRes)));
  const int j0 = std::max(0, static_cast<int>(std::floor((cy - radius) / mRes)));
  const int j1 = std::min(mNy - 1, static_cast<int>(std::ceil((cy + radius) / mRes)));
  if (i0 > i1 || j0 > j1)
    return;
  const double r2 = radius * radius;
  const double cellArea = mRes * mRes;
  bool changed = false;
  for (int j = j0; j <= j1; ++j) {
    const double dy = j * mRes - cy;
    for (int i = i0; i <= i1; ++i) {
      const double dx = i * mRes - cx;
      const double d2 = dx * dx + dy * dy;
      if (d2 > r2)
        continue;
      float &h = mHeights[static_cast<size_t>(j) * mNx + i];
      const double z = std::max(0.0, toolSurface(std::sqrt(d2), tipZ));
      if (z < h) {
        mRemovedVolume += (h - z) * cellArea;
        h = static_cast<float>(z);
        mDeepest = std::max(mDeepest, mTop - z);
        changed = true;
      }
    }
  }
  if (changed)
    touchTiles(i0, i1, j0, j1);
}

void OmMillingStock::touchTiles(int i0, int i1, int j0, int j1) {
  // A height feeds its own vertex and, through normals and the cut-colour test, its
  // four neighbours: re-stream every tile that holds any vertex of the grown window.
  const int vi0 = std::max(0, i0 - 1), vi1 = std::min(mNx - 1, i1 + 1);
  const int vj0 = std::max(0, j0 - 1), vj1 = std::min(mNy - 1, j1 + 1);
  const int tx0 = std::max(0, (vi0 - 1) / kTile), tx1 = std::min(mTilesX - 1, vi1 / kTile);
  const int ty0 = std::max(0, (vj0 - 1) / kTile), ty1 = std::min(mTilesY - 1, vj1 / kTile);
  for (int ty = ty0; ty <= ty1; ++ty)
    for (int tx = tx0; tx <= tx1; ++tx)
      ++mTileRevision[static_cast<size_t>(ty) * mTilesX + tx];
  if (vi0 == 0 || vj0 == 0 || vi1 == mNx - 1 || vj1 == mNy - 1)
    ++mSidesRevision;
}

void OmMillingStock::sweep(const OmVector3 &a, const OmVector3 &b) {
  const OmVector3 d = b - a;
  const double length = d.length();
  if (length > kTeleportDistance) {
    stamp(b.x(), b.y(), b.z());
    return;
  }
  // Half a cell per stamp: consecutive stamps overlap, so a fast move leaves a
  // continuous slot rather than a row of holes.
  const int n = std::min(kMaxStampsPerStep, std::max(1, static_cast<int>(std::ceil(length / (0.5 * mRes)))));
  for (int k = 1; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    stamp(a.x() + d.x() * t, a.y() + d.y() * t, a.z() + d.z() * t);
  }
}

bool OmMillingStock::isLaser() const {
  return mToolShape->value() == "laser";
}

bool OmMillingStock::burn(double cx, double cy, double focusZ, double energy) {
  if (energy <= 0.0)
    return false;
  // The spot is sized by the surface under its centre: as a kerf deepens it falls out of
  // focus, so each pass cuts wider and shallower than the one before.
  const int ic = static_cast<int>(std::lround(cx / mRes)), jc = static_cast<int>(std::lround(cy / mRes));
  const bool centreOn = ic >= 0 && ic < mNx && jc >= 0 && jc < mNy;
  const double surface = centreOn ? mHeights[static_cast<size_t>(jc) * mNx + ic] : mTop;
  // The lens focuses a cone of half-angle theta: away from focus each 1/e^2 radius grows
  // as sqrt(w0^2 + (theta dz)^2), the waist of a beam whose far field is that cone.
  const double spread = std::tan(std::min(std::max(mBeamHalfAngle->value(), 0.0), 1.2)) * (surface - focusZ);
  const OmVector2 spot = mLaserSpot->value();
  const double w0x = 0.5 * std::max(spot.x(), 1e-6), w0y = 0.5 * std::max(spot.y(), 1e-6);
  const double wx = std::sqrt(w0x * w0x + spread * spread), wy = std::sqrt(w0y * w0y + spread * spread);  // 1/e^2 radii
  // Two radii hold all but exp(-8) = 0.03 % of the beam.
  const int ri = static_cast<int>(std::ceil(2.0 * wx / mRes)), rj = static_cast<int>(std::ceil(2.0 * wy / mRes));
  const int ui0 = static_cast<int>(std::floor(cx / mRes)) - ri, ui1 = static_cast<int>(std::ceil(cx / mRes)) + ri;
  const int uj0 = static_cast<int>(std::floor(cy / mRes)) - rj, uj1 = static_cast<int>(std::ceil(cy / mRes)) + rj;
  if (ui1 < 0 || uj1 < 0 || ui0 >= mNx || uj0 >= mNy)
    return false;  // the beam misses the block: its energy is lost
  // Weights over the WHOLE window, on the block or not, normalised to the energy: the
  // block receives exactly what of the beam falls on it, at any spot-to-cell ratio.
  const int wi = ui1 - ui0 + 1, wj = uj1 - uj0 + 1;
  mSpotWeights.assign(static_cast<size_t>(wi) * wj, 0.0);
  double total = 0.0;
  for (int j = 0; j < wj; ++j) {
    const double dy = (uj0 + j) * mRes - cy;
    for (int i = 0; i < wi; ++i) {
      const double dx = (ui0 + i) * mRes - cx;
      const double w = std::exp(-2.0 * (dx * dx / (wx * wx) + dy * dy / (wy * wy)));
      mSpotWeights[static_cast<size_t>(j) * wi + i] = w;
      total += w;
    }
  }
  if (!(total > 0.0))
    return false;
  const double cellArea = mRes * mRes;
  const double perWeight = energy / (total * cellArea);  // J/m^2 per unit weight
  const double ablation = std::max(mAblationEnergy->value(), 1.0);
  const int i0 = std::max(0, ui0), i1 = std::min(mNx - 1, ui1);
  const int j0 = std::max(0, uj0), j1 = std::min(mNy - 1, uj1);
  bool changed = false;
  for (int j = j0; j <= j1; ++j)
    for (int i = i0; i <= i1; ++i) {
      const size_t k = static_cast<size_t>(j) * mNx + i;
      float &h = mHeights[k];
      if (h <= 0.0f)
        continue;  // cut through: the beam passes on below the block
      const double fluence = perWeight * mSpotWeights[static_cast<size_t>(j - uj0) * wi + (i - ui0)];
      if (fluence <= 0.0)
        continue;
      const double z = std::max(0.0, h - fluence / ablation);
      mRemovedVolume += (h - z) * cellArea;
      mDeposited += fluence * cellArea;
      mDeepest = std::max(mDeepest, mTop - z);
      h = static_cast<float>(z);
      mFluence[k] += static_cast<float>(fluence);
      changed = true;
    }
  if (changed)
    touchTiles(i0, i1, j0, j1);
  return changed;
}

void OmMillingStock::burnSweep(const OmVector3 &a, const OmVector3 &b, double energy) {
  const OmVector3 d = b - a;
  const double length = d.length();
  if (length > kTeleportDistance)
    return;  // not a driven path: nothing was burned along the jump
  // Samples a quarter of the smaller spot radius (or half a cell) apart, at the middle of
  // equal slices of the step, each carrying its slice of the step's energy.
  const OmVector2 spot = mLaserSpot->value();
  const double pitch = std::max(std::min(0.125 * std::min(spot.x(), spot.y()), 0.5 * mRes), 1e-7);
  const int n = std::min(kMaxStampsPerStep, std::max(1, static_cast<int>(std::ceil(length / pitch))));
  for (int k = 0; k < n; ++k) {
    const double t = (k + 0.5) / n;
    burn(a.x() + d.x() * t, a.y() + d.y() * t, a.z() + d.z() * t, energy / n);
  }
}

float OmMillingStock::charLevel(int i, int j) const {
  if (mFluence.empty())
    return 0.0f;
  const double f = mFluence[static_cast<size_t>(j) * mNx + i];
  return static_cast<float>(1.0 - std::exp(-f / std::max(mCharFluence->value(), 1e-9)));
}

void OmMillingStock::onPhysicsStepEnded() {
  if (mHeights.empty())
    return;
  if (!mTool) {
    resolveTool();
    if (!mTool) {
      if (!mWarnedNoTool && !mToolSolid->value().isEmpty()) {
        mWarnedNoTool = true;
        OmLog::warning(tr("MillingStock: no Solid named '%1'; nothing will be cut.").arg(mToolSolid->value()));
      }
      return;
    }
  }
  const OmVector3 tipWorld = mTool->matrix() * mToolTip->value();
  const OmVector3 tip = tipWorld - mOrigin;
  if (!mCutting->value()) {
    mLastTip = tip;
    mHaveLastTip = true;
    return;
  }
  const double before = mRemovedVolume, depositedBefore = mDeposited;
  if (isLaser()) {
    // The power the controller set for THIS step, delivered along the path the beam
    // really took during it (a lagging axis burns where it really is).
    const double power = mLaserPower->value();
    if (power > 0.0 && OmWorld::instance() && OmWorld::instance()->worldInfo()) {
      if (mFluence.size() != mHeights.size())
        mFluence.assign(mHeights.size(), 0.0f);
      const double dt = OmWorld::instance()->worldInfo()->basicTimeStep() / 1000.0;
      burnSweep(mHaveLastTip ? mLastTip : tip, tip, power * dt);
    }
  } else if (mHaveLastTip)
    sweep(mLastTip, tip);
  else
    stamp(tip.x(), tip.y(), tip.z());
  mLastTip = tip;
  mHaveLastTip = true;
  if (mRemovedVolume != before || mDeposited != depositedBefore) {
    mRemovedVolumeField->setValueFromOde(mRemovedVolume);
    mDeepestCutField->setValueFromOde(mDeepest);
    mDepositedEnergyField->setValueFromOde(mDeposited);
    mDumpDirty = !mDumpPath.empty();
  }
  if (mDumpDirty && (!mDumpedOnce || std::chrono::steady_clock::now() - mDumpedAt >= std::chrono::seconds(1)))
    dumpHeights();
}

// Layout (little-endian): "OMZMAP1\0", int32 nx, int32 ny, float64 resolution, float64 top,
// float64 origin x/y/z (world, m), float64 removed volume (m^3), then nx*ny float32 heights
// above the block's bottom (m), row j (along Y) after row j-1, i along X within a row.
// A laser block writes "OMZMAP2\0": the same, then float64 deposited energy (J) and
// float64 charFluence (J/m^2) after the removed volume, and nx*ny float32 absorbed
// fluences (J/m^2) after the heights.
void OmMillingStock::dumpHeights() {
  mDumpDirty = false;
  mDumpedOnce = true;
  mDumpedAt = std::chrono::steady_clock::now();
  // Written beside the target, then renamed over it, so a reader never sees half a map.
  const std::filesystem::path target = std::filesystem::u8path(mDumpPath);
  std::filesystem::path partial = target;
  partial += ".partial";
  bool ok;
  {
    std::ofstream file(partial, std::ios::binary | std::ios::trunc);
    const int32_t n[2] = {mNx, mNy};
    const double d[6] = {mRes, mTop, mOrigin.x(), mOrigin.y(), mOrigin.z(), mRemovedVolume};
    const bool laser = isLaser() && mFluence.size() == mHeights.size();
    if (laser)
      file.write("OMZMAP2", 8);
    else
      file.write("OMZMAP1", 8);
    file.write(reinterpret_cast<const char *>(n), sizeof(n));
    file.write(reinterpret_cast<const char *>(d), sizeof(d));
    if (laser) {
      const double e[2] = {mDeposited, mCharFluence->value()};
      file.write(reinterpret_cast<const char *>(e), sizeof(e));
    }
    file.write(reinterpret_cast<const char *>(mHeights.data()), static_cast<std::streamsize>(mHeights.size() * sizeof(float)));
    if (laser)
      file.write(reinterpret_cast<const char *>(mFluence.data()), static_cast<std::streamsize>(mFluence.size() * sizeof(float)));
    ok = static_cast<bool>(file);
  }
  std::error_code ec;
  if (ok) {
    std::filesystem::rename(partial, target, ec);
    if (ec) {   // a platform that will not rename over an existing file
      std::filesystem::remove(target, ec);
      std::filesystem::rename(partial, target, ec);
    }
  }
  if (!ok || ec) {
    OmLog::warning(tr("MillingStock: cannot write the height map to '%1'.").arg(QString::fromStdString(mDumpPath)));
    std::filesystem::remove(partial, ec);
    mDumpPath.clear();
  }
}

// ---- rendering ----------------------------------------------------------------------

bool OmMillingStock::isCut(int i, int j) const {
  return mTop - mHeights[static_cast<size_t>(j) * mNx + i] > mSkinDepth->value();
}

OmVector3 OmMillingStock::normalAt(int i, int j) const {
  const auto h = [this](int x, int y) { return static_cast<double>(mHeights[static_cast<size_t>(y) * mNx + x]); };
  const int il = std::max(0, i - 1), ir = std::min(mNx - 1, i + 1);
  const int jd = std::max(0, j - 1), ju = std::min(mNy - 1, j + 1);
  const double dzdx = (h(ir, j) - h(il, j)) / ((ir - il) * mRes);
  const double dzdy = (h(i, ju) - h(i, jd)) / ((ju - jd) * mRes);
  OmVector3 n(-dzdx, -dzdy, 1.0);
  n.normalize();
  return n;
}

bool OmMillingStock::tileIsFlat(int tile) const {
  if (mFlatJudgedAt[tile] == mTileRevision[tile])
    return mTileFlat[tile] != 0;
  const int tx = tile % mTilesX, ty = tile / mTilesX;
  // The ring matters: a neighbour's height shapes this tile's border normals and colours.
  const int i0 = std::max(0, tx * kTile - 1), i1 = std::min(mNx - 1, (tx + 1) * kTile + 1);
  const int j0 = std::max(0, ty * kTile - 1), j1 = std::min(mNy - 1, (ty + 1) * kTile + 1);
  const float h0 = mHeights[static_cast<size_t>(j0) * mNx + i0];
  const bool charred = !mFluence.empty() && isLaser();
  const float f0 = charred ? mFluence[static_cast<size_t>(j0) * mNx + i0] : 0.0f;
  bool flat = true;
  for (int j = j0; j <= j1 && flat; ++j) {
    const float *row = &mHeights[static_cast<size_t>(j) * mNx];
    const float *frow = charred ? &mFluence[static_cast<size_t>(j) * mNx] : nullptr;
    for (int i = i0; i <= i1; ++i)
      if (row[i] != h0 || (frow && frow[i] != f0)) {
        flat = false;
        break;
      }
  }
  mTileFlat[tile] = flat ? 1 : 0;
  mFlatJudgedAt[tile] = mTileRevision[tile];
  return flat;
}

const std::vector<unsigned int> &OmMillingStock::tileIndices(int tile) const {
  return tileIsFlat(tile) ? mQuadIndices : mTileIndices[tile];
}

int OmMillingStock::fullTiles() const {
  int n = 0;
  for (int t = 0; t < tileCount(); ++t)
    n += tileIsFlat(t) ? 0 : 1;
  return n;
}

bool OmMillingStock::tileVertexStream(int tile, std::vector<unsigned char> &out) const {
  if (mHeights.empty() || tile < 0 || tile >= tileCount())
    return false;
  const int tx = tile % mTilesX, ty = tile / mTilesX;
  const int i0 = tx * kTile, j0 = ty * kTile;
  const int i1 = std::min(mNx - 1, i0 + kTile), j1 = std::min(mNy - 1, j0 + kTile);
  const bool laser = isLaser() && !mFluence.empty();
  // uv.x on the laser's skin-to-char ramp: the centre of the texel nearest the char level.
  const auto charU = [](float c) { return (0.5f + std::min(1.0f, std::max(0.0f, c)) * (kCharTexels - 1)) / kCharTexels; };
  if (tileIsFlat(tile)) {
    // One quad: every height (and its ring) is equal, so the normal is +Z and the colour
    // is uniform -- the same pixels the full grid would draw, from 2 triangles.
    out.resize(4u * 32u);
    unsigned char *p = out.data();
    const double z = mHeights[static_cast<size_t>(j0) * mNx + i0];
    const float u = laser ? charU(charLevel(i0, j0)) : (isCut(i0, j0) ? 0.75f : 0.25f);
    const int ci[4] = {i0, i1, i0, i1}, cj[4] = {j0, j0, j1, j1};
    for (int k = 0; k < 4; ++k) {
      putFloat(p, static_cast<float>(mOrigin.x() + ci[k] * mRes));
      putFloat(p, static_cast<float>(mOrigin.y() + cj[k] * mRes));
      putFloat(p, static_cast<float>(mOrigin.z() + z));
      putFloat(p, 0.0f);
      putFloat(p, 0.0f);
      putFloat(p, 1.0f);
      putFloat(p, u);
      putFloat(p, 0.5f);
    }
    return true;
  }
  out.resize(static_cast<size_t>(i1 - i0 + 1) * (j1 - j0 + 1) * 32u);
  unsigned char *p = out.data();
  for (int j = j0; j <= j1; ++j)
    for (int i = i0; i <= i1; ++i) {
      const double z = mHeights[static_cast<size_t>(j) * mNx + i];
      const OmVector3 n = normalAt(i, j);
      // A vertex next to a cut takes the core colour too, so a wall is core-coloured
      // top to bottom instead of fading from skin to core across its height.
      const bool core = !laser && (isCut(i, j) || (i > 0 && isCut(i - 1, j)) || (i + 1 < mNx && isCut(i + 1, j)) ||
                                   (j > 0 && isCut(i, j - 1)) || (j + 1 < mNy && isCut(i, j + 1)));
      putFloat(p, static_cast<float>(mOrigin.x() + i * mRes));
      putFloat(p, static_cast<float>(mOrigin.y() + j * mRes));
      putFloat(p, static_cast<float>(mOrigin.z() + z));
      putFloat(p, static_cast<float>(n.x()));
      putFloat(p, static_cast<float>(n.y()));
      putFloat(p, static_cast<float>(n.z()));
      putFloat(p, laser ? charU(charLevel(i, j)) : (core ? 0.75f : 0.25f));
      putFloat(p, 0.5f);
    }
  return true;
}

bool OmMillingStock::sidesVertexStream(std::vector<unsigned char> &out) const {
  if (mHeights.empty())
    return false;
  const size_t count = 2u * (2u * mNx + 2u * mNy) + 4u;
  out.resize(count * 32u);
  unsigned char *p = out.data();
  const float sideU = isLaser() ? 0.5f / kCharTexels : 0.75f;
  auto vertex = [&](int i, int j, double z, float nx, float ny, float nz) {
    putFloat(p, static_cast<float>(mOrigin.x() + i * mRes));
    putFloat(p, static_cast<float>(mOrigin.y() + j * mRes));
    putFloat(p, static_cast<float>(mOrigin.z() + z));
    putFloat(p, nx);
    putFloat(p, ny);
    putFloat(p, nz);
    putFloat(p, sideU);
    putFloat(p, 0.5f);
  };
  auto pair = [&](int i, int j, float nx, float ny) {
    vertex(i, j, mHeights[static_cast<size_t>(j) * mNx + i], nx, ny, 0.0f);
    vertex(i, j, 0.0, nx, ny, 0.0f);
  };
  for (int i = 0; i < mNx; ++i)
    pair(i, 0, 0.0f, -1.0f);  // south, walking +X
  for (int j = 0; j < mNy; ++j)
    pair(mNx - 1, j, 1.0f, 0.0f);  // east, walking +Y
  for (int i = mNx - 1; i >= 0; --i)
    pair(i, mNy - 1, 0.0f, 1.0f);  // north, walking -X
  for (int j = mNy - 1; j >= 0; --j)
    pair(0, j, -1.0f, 0.0f);  // west, walking -Y
  // Bottom, facing -Z (clockwise seen from above).
  vertex(0, 0, 0.0, 0.0f, 0.0f, -1.0f);
  vertex(0, mNy - 1, 0.0, 0.0f, 0.0f, -1.0f);
  vertex(mNx - 1, mNy - 1, 0.0, 0.0f, 0.0f, -1.0f);
  vertex(mNx - 1, 0, 0.0, 0.0f, 0.0f, -1.0f);
  return true;
}

void OmMillingStock::wgpuColorTexels(std::vector<unsigned char> &rgba, float meanLinear[3]) const {
  const auto byte = [](double c) { return static_cast<unsigned char>(std::lround(std::min(1.0, std::max(0.0, c)) * 255.0)); };
  const auto put = [&](double r, double g, double b) { rgba.insert(rgba.end(), {byte(r), byte(g), byte(b), 255}); };
  rgba.clear();
  const OmRgb skin = mSkinColor->value();
  if (isLaser()) {
    // Skin to char, texel k at char level k / (kCharTexels - 1). The mean is the bare
    // material's: most of a sheet stays unburned, and the mean feeds its ambient bounce.
    const OmRgb burnt = mCharColor->value();
    for (int k = 0; k < kCharTexels; ++k) {
      const double c = static_cast<double>(k) / (kCharTexels - 1);
      put(skin.red() + (burnt.red() - skin.red()) * c, skin.green() + (burnt.green() - skin.green()) * c,
          skin.blue() + (burnt.blue() - skin.blue()) * c);
    }
    for (int c = 0; c < 3; ++c)
      meanLinear[c] = std::pow(rgba[c] / 255.0f, 2.2f);
    return;
  }
  const OmRgb core = mCoreColor->value();
  put(skin.red(), skin.green(), skin.blue());
  put(core.red(), core.green(), core.blue());
  for (int c = 0; c < 3; ++c)
    meanLinear[c] = 0.5f * (std::pow(rgba[c] / 255.0f, 2.2f) + std::pow(rgba[4 + c] / 255.0f, 2.2f));
}

float OmMillingStock::roughness() const {
  return static_cast<float>(std::min(1.0, std::max(0.0, mRoughness->value())));
}

bool OmMillingStock::castShadows() const {
  return mCastShadows->value();
}
