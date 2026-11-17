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

#include "OmPrintedPart.hpp"

#include "OmLog.hpp"
#include "OmMatrix4.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFString.hpp"
#include "OmSFVector3.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "OmWorld.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace {
  const double kDefaultResolution = 0.0002;
  const double kMinResolution = 0.00005;
  const int kMaxCellsPerAxis = 8192;
  // A bead is 36 bytes. Past kMaxDrawnChunks chunks beads are still RECORDED (they are the
  // part: the dump, inspect_print and the map under the nozzle need them) but not drawn;
  // past this many nothing more is recorded (1.8 GB of beads).
  const size_t kMaxRecordedBeads = 50000000;
  // While beads keep changing, the dump is rewritten no more often than once a second and
  // no more often than this many times its own write time, so the write (on the engine's
  // thread) costs at most ~2 % of the run however big the part. Once nothing has changed
  // for kDumpQuiet it is written at once, so the finished part is on disk within a second.
  const double kDumpCostFactor = 50.0;
  const std::chrono::milliseconds kDumpQuiet(500);
  // A nozzle that moves farther than this in ONE physics step was teleported (a world
  // reset, a supervisor pose edit), not driven: nothing is laid along the jump.
  const double kTeleportDistance = 0.05;
  // Plastic that left a nozzle standing still becomes a blob once there is this much;
  // less waits for the next move (an unretract's residue lands at the line's start).
  const double kBlobMinVolume = 5e-11;  // 0.05 mm^3
  // A bead is never flatter than this: a nozzle pressed onto what is under it still
  // leaves a smear, and the step is counted as a crash.
  const double kMinBeadHeight = 0.00002;
  // Steps along one line extend one bead while direction, width and height hold. The
  // width of a step's bead is its volume over the nozzle's REAL path that step, which the
  // servo's few-micron jitter moves by ~8 % step to step (MEASURED 2026-11-02, the SV08
  // cube); at 5 % a straight 20 mm wall was ~100 beads.
  const double kMergeCos = 0.9994;  // about 2 degrees
  const double kMergeRelative = 0.12;
  // Plastic within this of the nozzle's height is the layer being printed, not support:
  // the Z servo settles a layer a micron or two off its neighbours, and a bead that found
  // its own layer's fresh plastic "under" it came out 1 um tall and very wide.
  const double kSameLayerBand = 0.00002;
  // A step that would lay a strand thinner than a quarter of the nozzle is the hotend's
  // residual pressure leaking on a lift or a travel (ooze), which is not modelled: it is
  // dropped, so laid volume falls short of extruded volume by exactly that much.
  const double kMinStrandDiameterRatio = 0.25;
  const int kMaxSupportSamples = 4000;
  // A nozzle that moves less than this share of its diameter in one physics step stands
  // still. Until 2026-11-04 it was 0.1: on the SV08's 2 ms step that is 20 mm/s, so a
  // nozzle slowing into a corner or creeping on a first layer was "still", its plastic was
  // held, and it was laid on the next faster step's 0.04-0.05 mm chord as one bead up to
  // 7 mm wide (or, past kBlobMinVolume, as a dome standing above the tip, which the next
  // step counted as a crash).
  const double kStillRatio = 0.01;
  // The nozzle's flat face, in nozzle diameters (a 0.4 mm brass nozzle's face is about
  // 1 mm across). Plastic is flattened into a bead only under the face, so no bead is laid
  // wider than this; what a step pushes out beyond that wells up at the nozzle as a blob.
  const double kFaceRatio = 2.5;

  void putFloat(unsigned char *&p, float v) {
    std::memcpy(p, &v, sizeof(float));
    p += sizeof(float);
  }

  // The bead's section, six points counter-clockwise in (side, up) around its centre,
  // with smooth-shading normals: a flat-topped stadium, or a hexagonal round strand.
  void section(double width, double height, double pts[6][2], double nrm[6][2]) {
    const double r = 0.5 * height;
    const double straight = 0.5 * width - r;
    if (straight >= 0.5 * r) {
      const double p[6][2] = {{0.5 * width, 0.0}, {straight, r},   {-straight, r},
                              {-0.5 * width, 0.0}, {-straight, -r}, {straight, -r}};
      const double n[6][2] = {{1.0, 0.0}, {0.38, 0.92}, {-0.38, 0.92}, {-1.0, 0.0}, {-0.38, -0.92}, {0.38, -0.92}};
      std::memcpy(pts, p, sizeof(p));
      std::memcpy(nrm, n, sizeof(n));
      return;
    }
    const double radius = 0.5 * std::max(width, height);
    for (int k = 0; k < 6; ++k) {
      const double ang = k * M_PI / 3.0;
      nrm[k][0] = std::cos(ang);
      nrm[k][1] = std::sin(ang);
      pts[k][0] = radius * nrm[k][0];
      pts[k][1] = radius * nrm[k][1];
    }
  }
}  // namespace

static std::vector<OmPrintedPart *> gLivePrintedParts;

bool OmPrintedPart::anyPrintedParts() {
  return !gLivePrintedParts.empty();
}

const std::vector<OmPrintedPart *> &OmPrintedPart::liveParts() {
  return gLivePrintedParts;
}

void OmPrintedPart::init() {
  mTranslation = findSFVector3("translation");
  mSize = findSFVector3("size");
  mResolution = findSFDouble("resolution");
  mNozzleSolid = findSFString("nozzleSolid");
  mNozzleTip = findSFVector3("nozzleTip");
  mNozzleDiameter = findSFDouble("nozzleDiameter");
  mExtrudedVolume = findSFDouble("extrudedVolume");
  mColor = findSFColor("color");
  mRoughness = findSFDouble("roughness");
  mCastShadows = findSFBool("castShadows");
  mDepositedField = findSFDouble("depositedVolume");
  mBeadCountField = findSFDouble("beadCount");
  mCrashesField = findSFDouble("nozzleCrashes");

  mNozzle = NULL;
  mWarnedNoNozzle = false;
  mWarnedFull = false;
  mWarnedRecordFull = false;
  mNx = mNy = 0;
  mRes = kDefaultResolution;
  mLastBeadOpen = false;
  mHaveLastTip = false;
  mLastVolume = 0.0;
  mPending = 0.0;
  mLastSupport = 0.0;
  mDeposited = 0.0;
  mCrashes = 0;
  mLastBlob = kNoBlob;
  mBlobVolume = 0.0;
  mBlobHalfWidth = 0.0;
  const char *dump = std::getenv("OMNISIM_PRINTED_PART_DUMP");
  mDumpPath = dump ? dump : "";
  mDumpDirty = false;
  mDumpCost = std::chrono::steady_clock::duration::zero();

  // One chunk's index buffer, for all kChunk beads. Bead k's vertices: ring A (the start)
  // 0-5 and ring B (the end) 6-11 for the sides, then cap A 12-17 and cap B 18-23 with
  // their own flat normals. Counter-clockwise seen from outside.
  mChunkIndices.clear();
  mChunkIndices.reserve(static_cast<size_t>(kChunk) * 60);
  for (int bead = 0; bead < kChunk; ++bead) {
    const unsigned int o = static_cast<unsigned int>(bead * kVerticesPerBead);
    for (unsigned int k = 0; k < 6; ++k) {
      const unsigned int k1 = (k + 1) % 6;
      mChunkIndices.insert(mChunkIndices.end(), {o + k, o + 6 + k1, o + 6 + k, o + k, o + k1, o + 6 + k1});
    }
    for (unsigned int k = 1; k < 5; ++k) {
      mChunkIndices.insert(mChunkIndices.end(), {o + 12, o + 12 + k + 1, o + 12 + k});
      mChunkIndices.insert(mChunkIndices.end(), {o + 18, o + 18 + k, o + 18 + k + 1});
    }
  }
}

OmPrintedPart::OmPrintedPart(OmTokenizer *tokenizer) : OmBaseNode("PrintedPart", tokenizer) {
  init();
  gLivePrintedParts.push_back(this);
}

OmPrintedPart::OmPrintedPart(const OmPrintedPart &other) : OmBaseNode(other) {
  init();
  gLivePrintedParts.push_back(this);
}

OmPrintedPart::OmPrintedPart(const OmNode &other) : OmBaseNode(other) {
  init();
  gLivePrintedParts.push_back(this);
}

OmPrintedPart::~OmPrintedPart() {
  gLivePrintedParts.erase(std::remove(gLivePrintedParts.begin(), gLivePrintedParts.end(), this),
                          gLivePrintedParts.end());
}

void OmPrintedPart::postFinalize() {
  OmBaseNode::postFinalize();
  rebuild();
  connect(mSize, &OmSFVector3::changed, this, &OmPrintedPart::rebuild);
  connect(mTranslation, &OmSFVector3::changed, this, &OmPrintedPart::rebuild);
  connect(mResolution, &OmSFDouble::changed, this, &OmPrintedPart::rebuild);
  connect(mNozzleSolid, &OmSFString::changed, this, &OmPrintedPart::resolveNozzle);
  // Colour and roughness are read per frame (OmWgpuSceneRenderer::collectPrintedDraws).
  // physicsStepENDED, as MillingStock: the nozzle Solid's pose is this step's result.
  OmSimulationWorld *const world = OmSimulationWorld::instance();
  if (world != NULL)
    connect(world, &OmSimulationWorld::physicsStepEnded, this, &OmPrintedPart::onPhysicsStepEnded);
}

void OmPrintedPart::resolveNozzle() {
  if (mNozzle)
    disconnect(mNozzle, &QObject::destroyed, this, nullptr);
  mNozzle = NULL;
  mHaveLastTip = false;
  const QString name = mNozzleSolid->value();
  if (name.isEmpty() || OmWorld::instance() == NULL)
    return;
  mNozzle = OmWorld::instance()->findSolid(name);
  if (mNozzle)  // a deleted print head must not leave a dangling pointer behind
    connect(mNozzle, &QObject::destroyed, this, [this]() { mNozzle = NULL; });
}

void OmPrintedPart::rebuild() {
  mTop.clear();
  mBeads.clear();
  mChunkRevision.clear();
  mLastBeadOpen = false;
  mHaveLastTip = false;
  mLastVolume = mExtrudedVolume->value();
  mPending = 0.0;
  mLastSupport = 0.0;
  mDeposited = 0.0;
  mCrashes = 0;
  mLastBlob = kNoBlob;
  mBlobVolume = 0.0;
  mBlobHalfWidth = 0.0;
  const OmVector3 size = mSize->value();
  if (!(size.x() > 0.0 && size.y() > 0.0 && size.z() > 0.0)) {
    OmLog::warning(tr("PrintedPart: 'size' must be strictly positive on all three axes; nothing will be printed."));
    return;
  }
  mRes = mResolution->value() > 0.0 ? std::max(mResolution->value(), kMinResolution) : kDefaultResolution;
  const double longest = std::max(size.x(), size.y());
  if (longest / mRes > kMaxCellsPerAxis) {
    const double coarser = longest / kMaxCellsPerAxis;
    OmLog::warning(tr("PrintedPart: resolution %1 m gives more than %2 cells per side; using %3 m.")
                     .arg(mRes)
                     .arg(kMaxCellsPerAxis)
                     .arg(coarser));
    mRes = coarser;
  }
  mNx = static_cast<int>(std::ceil(size.x() / mRes)) + 1;
  mNy = static_cast<int>(std::ceil(size.y() / mRes)) + 1;
  mOrigin = mTranslation->value();
  mExtent = size;
  mTop.assign(static_cast<size_t>(mNx) * mNy, 0.0f);
  mDepositedField->setValueFromOde(0.0);
  mBeadCountField->setValueFromOde(0.0);
  mCrashesField->setValueFromOde(0.0);
  resolveNozzle();
}

// ---- laying plastic -----------------------------------------------------------------

double OmPrintedPart::supportUnder(const OmVector3 &a, const OmVector3 &b, double belowZ, bool &crashed) const {
  // The highest plastic under the centre line that is BELOW the nozzle. Plastic at the
  // nozzle's own height is the line being printed (or its neighbour in this layer), not
  // support; plastic clearly ABOVE the tip means the nozzle is ploughing through the part.
  crashed = false;
  // Plastic of the blob the nozzle is standing in (its footprint, this layer) is plastic
  // welling up around this nozzle, not a part it runs into (a map cell away, for the cells).
  const double nozzle = std::max(1e-5, mNozzleDiameter->value());
  const bool inBlob = mLastBlob != kNoBlob && std::abs(mBlobCentre.z() - belowZ) <= 0.25 * nozzle;
  const double length = std::hypot(b.x() - a.x(), b.y() - a.y());
  const int n = std::min(kMaxSupportSamples, std::max(2, static_cast<int>(std::ceil(length / mRes)) + 1));
  double best = -1.0;
  for (int k = 0; k < n; ++k) {
    const double t = static_cast<double>(k) / (n - 1);
    const double px = a.x() + (b.x() - a.x()) * t, py = a.y() + (b.y() - a.y()) * t;
    const int i = static_cast<int>(std::lround(px / mRes));
    const int j = static_cast<int>(std::lround(py / mRes));
    if (i < 0 || j < 0 || i >= mNx || j >= mNy)
      continue;
    const double h = mTop[static_cast<size_t>(j) * mNx + i];
    if (h > belowZ + kMinBeadHeight * 2.5 &&
        !(inBlob && std::hypot(px - mBlobCentre.x(), py - mBlobCentre.y()) < mBlobHalfWidth + mRes))
      crashed = true;
    if (h < belowZ - kSameLayerBand)
      best = std::max(best, h);
  }
  return best;
}

void OmPrintedPart::raise(const OmVector3 &a, const OmVector3 &b, double halfWidth, double top) {
  const double x0 = std::min(a.x(), b.x()) - halfWidth, x1 = std::max(a.x(), b.x()) + halfWidth;
  const double y0 = std::min(a.y(), b.y()) - halfWidth, y1 = std::max(a.y(), b.y()) + halfWidth;
  const int i0 = std::max(0, static_cast<int>(std::floor(x0 / mRes))), i1 = std::min(mNx - 1, static_cast<int>(std::ceil(x1 / mRes)));
  const int j0 = std::max(0, static_cast<int>(std::floor(y0 / mRes))), j1 = std::min(mNy - 1, static_cast<int>(std::ceil(y1 / mRes)));
  const double dx = b.x() - a.x(), dy = b.y() - a.y();
  const double len2 = dx * dx + dy * dy;
  const float t32 = static_cast<float>(top);
  for (int j = j0; j <= j1; ++j)
    for (int i = i0; i <= i1; ++i) {
      const double px = i * mRes - a.x(), py = j * mRes - a.y();
      const double t = len2 > 0.0 ? std::min(1.0, std::max(0.0, (px * dx + py * dy) / len2)) : 0.0;
      const double ex = px - t * dx, ey = py - t * dy;
      if (ex * ex + ey * ey > halfWidth * halfWidth)
        continue;
      float &h = mTop[static_cast<size_t>(j) * mNx + i];
      h = std::max(h, t32);
    }
}

void OmPrintedPart::touchChunk(size_t bead) {
  const size_t chunk = bead / kChunk;
  if (chunk >= static_cast<size_t>(kMaxDrawnChunks))
    return;  // recorded, not drawn
  while (mChunkRevision.size() <= chunk)
    mChunkRevision.push_back(1);
  ++mChunkRevision[chunk];
}

void OmPrintedPart::addBead(const OmVector3 &a, const OmVector3 &b, double width, double height, bool mayMerge) {
  if (mayMerge && mLastBeadOpen && !mBeads.empty()) {
    Bead &last = mBeads.back();
    const OmVector3 la(last.a[0], last.a[1], last.a[2]), lb(last.b[0], last.b[1], last.b[2]);
    const OmVector3 d0 = lb - la, d1 = b - a;
    const double l0 = d0.length(), l1 = d1.length();
    if (last.blob == 0.0f && l0 > 0.0 && l1 > 0.0 && (lb - a).length() < 1e-6 && std::abs(a.z() - la.z()) < 1e-6 &&
        std::abs(b.z() - a.z()) < 1e-6 && d0.dot(d1) / (l0 * l1) > kMergeCos &&
        std::abs(width - last.width) < kMergeRelative * last.width && std::abs(height - last.height) < kMergeRelative * last.height) {
      last.width = static_cast<float>((last.width * l0 + width * l1) / (l0 + l1));
      last.height = static_cast<float>((last.height * l0 + height * l1) / (l0 + l1));
      last.b[0] = static_cast<float>(b.x());
      last.b[1] = static_cast<float>(b.y());
      last.b[2] = static_cast<float>(b.z());
      touchChunk(mBeads.size() - 1);
      return;
    }
  }
  if (mBeads.size() >= kMaxRecordedBeads) {
    if (!mWarnedRecordFull) {
      mWarnedRecordFull = true;
      OmLog::warning(tr("PrintedPart: %1 beads recorded; later plastic is counted but not recorded.").arg(mBeads.size()));
    }
    mLastBeadOpen = false;
    return;
  }
  if (!mWarnedFull && mBeads.size() >= static_cast<size_t>(kChunk) * kMaxDrawnChunks) {
    mWarnedFull = true;
    OmLog::info(tr("PrintedPart: %1 beads drawn; later beads are recorded and dumped but not drawn.").arg(mBeads.size()));
  }
  Bead bead;
  bead.a[0] = static_cast<float>(a.x());
  bead.a[1] = static_cast<float>(a.y());
  bead.a[2] = static_cast<float>(a.z());
  bead.b[0] = static_cast<float>(b.x());
  bead.b[1] = static_cast<float>(b.y());
  bead.b[2] = static_cast<float>(b.z());
  bead.width = static_cast<float>(width);
  bead.height = static_cast<float>(height);
  bead.blob = 0.0f;
  mBeads.push_back(bead);
  touchChunk(mBeads.size() - 1);
  mLastBeadOpen = true;
}

void OmPrintedPart::lay(const OmVector3 &a, const OmVector3 &b, double volume) {
  const double nozzle = std::max(1e-5, mNozzleDiameter->value());
  const double length = (b - a).length();
  const double top = 0.5 * (a.z() + b.z());
  bool crashed = false;
  double support = supportUnder(a, b, top, crashed);
  if (support < 0.0) {
    // Every sample was this layer's own plastic: the layer below is where the last bead
    // found it, if that is close, or else assume a half-nozzle layer.
    const bool lastIsClose = top - mLastSupport > 0.0 && top - mLastSupport < 2.0 * nozzle;
    support = lastIsClose ? mLastSupport : top - 0.5 * nozzle;
  }
  mLastSupport = support;
  if (crashed)
    ++mCrashes;
  const double area = volume / length;
  const double minStrand = kMinStrandDiameterRatio * nozzle;
  if (area < 0.25 * M_PI * minStrand * minStrand) {
    mLastBeadOpen = false;
    return;
  }
  double height = std::max(kMinBeadHeight, top - support);
  double width;
  bool roundStrand = false;
  if (height > 2.0 * nozzle) {
    // Nothing close underneath: a round strand hanging from the nozzle (a bridge).
    width = height = std::sqrt(4.0 * area / M_PI);
    roundStrand = true;
  } else {
    // A flattened stadium of this height: area = (w - h) h + pi h^2 / 4.
    width = (area - 0.25 * M_PI * height * height) / height + height;
    if (width < height) {
      width = height = std::sqrt(4.0 * area / M_PI);  // too little plastic to reach down: a thin strand
      roundStrand = true;
    }
  }
  // No bead is wider than the nozzle's face: the line holds what a face-wide bead of this
  // height holds, and the rest is a blob where the step began (at a corner the nozzle
  // slows faster than the melt's pressure falls; a held residue lands at a line's start).
  double laid = volume;
  const double face = kFaceRatio * nozzle;
  if (width > face) {
    if (roundStrand) {
      width = height = face;
      laid = 0.25 * M_PI * face * face * length;
    } else {
      width = face;
      laid = ((face - height) * height + 0.25 * M_PI * height * height) * length;
    }
  }
  addBead(a, b, width, height, true);
  raise(a, b, 0.5 * width, top);
  mDeposited += laid;
  if (laid < volume)
    blob(a, volume - laid);
}

bool OmPrintedPart::nearBlob(const OmVector3 &p) const {
  const double nozzle = std::max(1e-5, mNozzleDiameter->value());
  return std::hypot(mBlobCentre.x() - p.x(), mBlobCentre.y() - p.y()) < std::max(nozzle, mBlobHalfWidth) &&
         std::abs(mBlobCentre.z() - p.z()) <= 0.25 * nozzle;
}

void OmPrintedPart::blobShape(double volume, double gap, double nozzle, double &width, double &height) {
  // Under the nozzle's face the plastic is pressed into a disc as tall as the gap. Once the
  // disc would be wider than the face it wells up around the nozzle: at least face wide,
  // and never narrower than a half sphere of its volume. With nothing close underneath (a
  // gap over 2 nozzles) it is a free drop, as wide as that half sphere. Past the disc, a
  // puck of width w and height 4 V / (pi w^2) holds the volume.
  const double face = kFaceRatio * nozzle;
  if (gap <= 2.0 * nozzle) {
    if (volume <= 0.25 * M_PI * face * face * gap) {
      width = std::sqrt(4.0 * volume / (M_PI * gap));
      height = gap;
      return;
    }
    width = std::max(face, 2.0 * std::cbrt(3.0 * volume / (2.0 * M_PI)));
  } else
    width = 2.0 * std::cbrt(3.0 * volume / (2.0 * M_PI));
  height = 4.0 * volume / (M_PI * width * width);
}

void OmPrintedPart::blob(const OmVector3 &at, double volume) {
  // Plastic left while the nozzle is still over the blob it last made (same layer) joins it where
  // that blob is: plastic stays where it left the nozzle.
  const double nozzle = std::max(1e-5, mNozzleDiameter->value());
  double total = volume;
  OmVector3 centre = at;
  const bool grow = mLastBlob != kNoBlob && nearBlob(at);
  if (grow) {
    total += mBlobVolume;
    centre = mBlobCentre;
  }
  bool crashed = false;
  double support = supportUnder(centre, centre + OmVector3(1e-9, 0.0, 0.0), at.z(), crashed);
  if (support < 0.0) {  // only this layer's plastic under it: fall back as lay() does
    const bool lastIsClose = at.z() - mLastSupport > 0.0 && at.z() - mLastSupport < 2.0 * nozzle;
    support = lastIsClose ? mLastSupport : at.z() - 0.5 * nozzle;
  }
  const double gap = std::max(kMinBeadHeight, at.z() - support);
  double width, height;
  blobShape(total, gap, nozzle, width, height);
  const double top = support + height;
  const OmVector3 a(centre.x() - 0.25 * width, centre.y(), top), b(centre.x() + 0.25 * width, centre.y(), top);
  if (grow) {
    Bead &bead = mBeads[mLastBlob];
    bead.a[0] = static_cast<float>(a.x());
    bead.a[1] = static_cast<float>(a.y());
    bead.a[2] = static_cast<float>(a.z());
    bead.b[0] = static_cast<float>(b.x());
    bead.b[1] = static_cast<float>(b.y());
    bead.b[2] = static_cast<float>(b.z());
    bead.width = static_cast<float>(width);
    bead.height = static_cast<float>(height);
    bead.blob = static_cast<float>(total);
    touchChunk(mLastBlob);
    mBlobVolume = total;
    mBlobHalfWidth = 0.5 * width;
  } else {
    mLastBeadOpen = false;
    const size_t before = mBeads.size();
    addBead(a, b, width, height, false);
    mLastBeadOpen = false;
    if (mBeads.size() > before) {  // not past the recorded-bead cap
      mBeads.back().blob = static_cast<float>(total);
      mLastBlob = mBeads.size() - 1;
      mBlobCentre = centre;
      mBlobVolume = total;
      mBlobHalfWidth = 0.5 * width;
    } else
      mLastBlob = kNoBlob;
  }
  raise(centre, centre, 0.5 * width, top);
  mDeposited += volume;
}

void OmPrintedPart::onPhysicsStepEnded() {
  if (mTop.empty())
    return;
  if (!mNozzle) {
    resolveNozzle();
    if (!mNozzle) {
      if (!mWarnedNoNozzle && !mNozzleSolid->value().isEmpty()) {
        mWarnedNoNozzle = true;
        OmLog::warning(tr("PrintedPart: no Solid named '%1'; nothing will be printed.").arg(mNozzleSolid->value()));
      }
      return;
    }
  }
  const OmVector3 tip = mNozzle->matrix() * mNozzleTip->value() - mOrigin;
  const double volume = mExtrudedVolume->value();
  const double added = volume - mLastVolume;
  mLastVolume = volume;
  if (added > 0.0)
    mPending += added;
  const size_t beadsBefore = mBeads.size();
  const double depositedBefore = mDeposited;
  const int crashesBefore = mCrashes;
  if (mHaveLastTip) {
    const OmVector3 move = tip - mLastTip;
    const double horizontal = std::hypot(move.x(), move.y());
    const double nozzle = std::max(1e-5, mNozzleDiameter->value());
    if (move.length() > kTeleportDistance) {
      mPending = 0.0;
      mLastBeadOpen = false;
      mLastBlob = kNoBlob;
    } else if (mPending <= 0.0) {
      mLastBeadOpen = false;
    } else if (horizontal < kStillRatio * nozzle) {
      if (mPending >= kBlobMinVolume) {
        blob(tip, mPending);
        mPending = 0.0;
      }
    } else if (added <= 0.0 && mPending < kBlobMinVolume) {
      mPending = 0.0;  // a residue on a travel move: dropped (ooze is not modelled)
      mLastBeadOpen = false;
    } else {
      lay(mLastTip, tip, mPending);
      mPending = 0.0;
    }
  }
  if (mLastBlob != kNoBlob && !nearBlob(tip))
    mLastBlob = kNoBlob;  // the nozzle has left it: the next blob is a new one
  mLastTip = tip;
  mHaveLastTip = true;
  const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  if (mDeposited != depositedBefore || mBeads.size() != beadsBefore) {
    mDepositedField->setValueFromOde(mDeposited);
    mBeadCountField->setValueFromOde(static_cast<double>(mBeads.size()));
    mDumpDirty = !mDumpPath.empty();
    mChangedAt = now;
  }
  if (mCrashes != crashesBefore)
    mCrashesField->setValueFromOde(static_cast<double>(mCrashes));
  if (mDumpDirty) {
    const std::chrono::steady_clock::duration sinceDump = now - mDumpedAt;
    const std::chrono::steady_clock::duration busyInterval =
      std::max<std::chrono::steady_clock::duration>(std::chrono::seconds(1),
                                                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(mDumpCost * kDumpCostFactor));
    const bool quiet = now - mChangedAt >= kDumpQuiet;
    if (sinceDump >= busyInterval || (quiet && sinceDump >= std::chrono::seconds(1)))
      dumpBeads();
  }
}

// Layout (little-endian): "OMBEADS1", int32 count, float64 origin x/y/z (world, m),
// float64 deposited volume (m^3), float64 resolution, then per bead 9 float32: a x/y/z,
// b x/y/z (the bead's top centre line, metres above the bed origin), width, height,
// blob volume (m^3, 0 for a line).
void OmPrintedPart::dumpBeads() {
  mDumpDirty = false;
  const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
  mDumpedAt = started;
  const std::filesystem::path target = std::filesystem::u8path(mDumpPath);
  std::filesystem::path partial = target;
  partial += ".partial";
  bool ok;
  {
    std::ofstream file(partial, std::ios::binary | std::ios::trunc);
    const int32_t count = static_cast<int32_t>(mBeads.size());
    const double d[5] = {mOrigin.x(), mOrigin.y(), mOrigin.z(), mDeposited, mRes};
    file.write("OMBEADS1", 8);
    file.write(reinterpret_cast<const char *>(&count), sizeof(count));
    file.write(reinterpret_cast<const char *>(d), sizeof(d));
    // A Bead is exactly the format's 9 float32, in its order: the beads go out in one write.
    static_assert(sizeof(Bead) == 9 * sizeof(float), "Bead must be the dump's 9 float32");
    if (!mBeads.empty())
      file.write(reinterpret_cast<const char *>(mBeads.data()), static_cast<std::streamsize>(mBeads.size() * sizeof(Bead)));
    ok = static_cast<bool>(file);
  }
  std::error_code ec;
  if (ok) {
    std::filesystem::rename(partial, target, ec);
    if (ec) {  // a platform that will not rename over an existing file
      std::filesystem::remove(target, ec);
      std::filesystem::rename(partial, target, ec);
    }
  }
  if (!ok || ec) {
    OmLog::warning(tr("PrintedPart: cannot write the beads to '%1'.").arg(QString::fromStdString(mDumpPath)));
    std::filesystem::remove(partial, ec);
    mDumpPath.clear();
  }
  mDumpCost = std::chrono::steady_clock::now() - started;
}

// ---- rendering ----------------------------------------------------------------------

bool OmPrintedPart::chunkVertexStream(int chunk, std::vector<unsigned char> &out) const {
  const size_t first = static_cast<size_t>(chunk) * kChunk;
  if (chunk < 0 || first >= mBeads.size())
    return false;
  out.resize(static_cast<size_t>(kChunk) * kVerticesPerBead * 32u);
  unsigned char *p = out.data();
  auto vertex = [&](const OmVector3 &v, const OmVector3 &n) {
    putFloat(p, static_cast<float>(mOrigin.x() + v.x()));
    putFloat(p, static_cast<float>(mOrigin.y() + v.y()));
    putFloat(p, static_cast<float>(mOrigin.z() + v.z()));
    putFloat(p, static_cast<float>(n.x()));
    putFloat(p, static_cast<float>(n.y()));
    putFloat(p, static_cast<float>(n.z()));
    putFloat(p, 0.0f);
    putFloat(p, 0.0f);
  };
  const size_t last = std::min(mBeads.size(), first + kChunk);
  OmVector3 parked;
  for (size_t k = first; k < last; ++k) {
    const Bead &bead = mBeads[k];
    const OmVector3 a(bead.a[0], bead.a[1], bead.a[2]), b(bead.b[0], bead.b[1], bead.b[2]);
    OmVector3 d = b - a;
    const double length = d.length();
    d = length > 0.0 ? d / length : OmVector3(1.0, 0.0, 0.0);
    OmVector3 side = OmVector3(0.0, 0.0, 1.0).cross(d);
    side = side.length() > 1e-9 ? side.normalized() : OmVector3(0.0, 1.0, 0.0);
    const OmVector3 up = d.cross(side);
    const OmVector3 drop = up * (0.5 * bead.height);  // the section is centred half a bead below the tip
    double pts[6][2], nrm[6][2];
    section(bead.width, bead.height, pts, nrm);
    OmVector3 ringA[6], ringB[6], normal[6];
    for (int i = 0; i < 6; ++i) {
      const OmVector3 offset = side * pts[i][0] + up * pts[i][1];
      ringA[i] = a - drop + offset;
      ringB[i] = b - drop + offset;
      normal[i] = side * nrm[i][0] + up * nrm[i][1];
    }
    if (k == first)
      parked = ringA[0];
    for (int i = 0; i < 6; ++i)
      vertex(ringA[i], normal[i]);
    for (int i = 0; i < 6; ++i)
      vertex(ringB[i], normal[i]);
    for (int i = 0; i < 6; ++i)
      vertex(ringA[i], -d);
    for (int i = 0; i < 6; ++i)
      vertex(ringB[i], d);
  }
  // The rest of the chunk's capacity: degenerate, all at one real point of this chunk so
  // the mesh's bounding sphere stays on the part.
  for (size_t k = last; k < first + kChunk; ++k)
    for (int i = 0; i < kVerticesPerBead; ++i)
      vertex(parked, OmVector3(0.0, 0.0, 1.0));
  return true;
}

void OmPrintedPart::color(float rgb[3]) const {
  const OmRgb c = mColor->value();
  rgb[0] = static_cast<float>(c.red());
  rgb[1] = static_cast<float>(c.green());
  rgb[2] = static_cast<float>(c.blue());
}

float OmPrintedPart::roughness() const {
  return static_cast<float>(std::min(1.0, std::max(0.0, mRoughness->value())));
}

bool OmPrintedPart::castShadows() const {
  return mCastShadows->value();
}
