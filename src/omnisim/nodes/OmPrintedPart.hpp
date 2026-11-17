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

#ifndef OM_PRINTED_PART_HPP
#define OM_PRINTED_PART_HPP

// PrintedPart: plastic laid down by an FDM nozzle (resources/nodes/PrintedPart.wrl).
//
// The additive twin of MillingStock, and a node for the same reason: a print is tens of
// thousands of beads, laid every physics step, and a supervisor rebuilding a mesh would
// re-hash and re-upload the whole part each time. Here the beads live in fixed-capacity
// CHUNKS drawn through the renderer's dynamic-draw path (OmWgpuSceneRenderer::
// collectDynamicDraws). A chunk's index buffer covers its whole capacity from the start
// (unused beads are degenerate), so filling it in is a vertex re-upload into the buffer it
// already has, and only the chunk being printed into changes; every full chunk is static.
//
// A bead is a straight run of constant width and height, a six-sided stadium in section,
// smooth-shaded. Consecutive steps along one straight line extend the same bead, so a
// 20 mm wall is one bead, not 40 steps of half a millimetre.

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
class OmSFVector3;
class OmSolid;

class OmPrintedPart : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmPrintedPart(OmTokenizer *tokenizer = NULL);
  OmPrintedPart(const OmPrintedPart &other);
  explicit OmPrintedPart(const OmNode &other);
  virtual ~OmPrintedPart();

  int nodeType() const override { return WB_NODE_PRINTED_PART; }
  void postFinalize() override;

  // ---- the registry the renderer reads (same pattern as OmMillingStock) ---------------
  static bool anyPrintedParts();
  static const std::vector<OmPrintedPart *> &liveParts();

  // ---- renderer contract. Chunk c holds beads [c*kChunk, (c+1)*kChunk). Each is one
  // streamed mesh with a FIXED index buffer for all kChunk beads; `chunkRevision(c)` moves
  // only when a bead in it was added or extended. Vertices are WORLD space, stride 32:
  // pos3 + norm3 + uv2 (uv unused).
  static const int kChunk = 512;
  static const int kVerticesPerBead = 24;
  // The renderer reserves 1023 mesh slots per part (OmWgpuSceneRenderer printedMeshKey):
  // beads past kChunk * kMaxDrawnChunks are recorded and dumped but not drawn.
  static const int kMaxDrawnChunks = 1023;
  int chunkCount() const { return static_cast<int>(mChunkRevision.size()); }
  double chunkRevision(int chunk) const { return static_cast<double>(mChunkRevision[chunk]); }
  const std::vector<unsigned int> &chunkIndices() const { return mChunkIndices; }
  bool chunkVertexStream(int chunk, std::vector<unsigned char> &out) const;
  void color(float rgb[3]) const;
  float roughness() const;
  bool castShadows() const;

  // ---- state a supervisor can read through the hidden fields as well ----------------
  double depositedVolume() const { return mDeposited; }
  int beadCount() const { return static_cast<int>(mBeads.size()); }

private slots:
  void onPhysicsStepEnded();
  void rebuild();
  void resolveNozzle();

private:
  OmPrintedPart &operator=(const OmPrintedPart &);  // non copyable
  OmNode *clone() const override { return new OmPrintedPart(*this); }
  void init();

  struct Bead {
    float a[3], b[3];  // centre line at the TOP of the bead (where the nozzle tip ran), local metres
    float width, height;
    float blob;        // > 0: a blob of this volume (m^3) centred between a and b, not a line
  };
  // Lay `volume` (m^3) along a -> b (local metres, z above the bed).
  void lay(const OmVector3 &a, const OmVector3 &b, double volume);
  // A blob: plastic that left the nozzle at `at` and stays there (a nozzle standing still,
  // or what a step pushed out beyond the nozzle's face).
  void blob(const OmVector3 &at, double volume);
  // Over the blob being grown (within its footprint, and never less than a nozzle of its
  // centre), in the same layer.
  bool nearBlob(const OmVector3 &p) const;
  // A blob's width and height for its volume and the gap under the nozzle tip.
  static void blobShape(double volume, double gap, double nozzle, double &width, double &height);
  void addBead(const OmVector3 &a, const OmVector3 &b, double width, double height, bool mayMerge);
  // The highest plastic (or the bed, 0) under the segment's centre line, below `belowZ`;
  // `crashed` when plastic stands above the tip there (not counting the blob it is in).
  double supportUnder(const OmVector3 &a, const OmVector3 &b, double belowZ, bool &crashed) const;
  void raise(const OmVector3 &a, const OmVector3 &b, double halfWidth, double top);
  void touchChunk(size_t bead);
  // OMNISIM_PRINTED_PART_DUMP=<file>: every bead, so a test (inspect_print.py) can measure
  // the part. Rewritten at most once a second while printing (less often once a write
  // takes more than 20 ms), and within about a second of the last change.
  void dumpBeads();

  OmSFVector3 *mTranslation;
  OmSFVector3 *mSize;
  OmSFDouble *mResolution;
  OmSFString *mNozzleSolid;
  OmSFVector3 *mNozzleTip;
  OmSFDouble *mNozzleDiameter;
  OmSFDouble *mExtrudedVolume;
  OmSFColor *mColor;
  OmSFDouble *mRoughness;
  OmSFBool *mCastShadows;
  OmSFDouble *mDepositedField;
  OmSFDouble *mBeadCountField;
  OmSFDouble *mCrashesField;

  OmSolid *mNozzle;  // cleared by the Solid's destroyed() signal (resolveNozzle)
  bool mWarnedNoNozzle;

  // What is under the nozzle: the top of the plastic over each cell, metres above the bed.
  int mNx, mNy;
  double mRes;
  OmVector3 mOrigin, mExtent;
  std::vector<float> mTop;

  std::vector<Bead> mBeads;
  bool mLastBeadOpen;          // the last bead may still be extended by the next step
  std::vector<uint64_t> mChunkRevision;
  std::vector<unsigned int> mChunkIndices;

  bool mHaveLastTip;
  OmVector3 mLastTip;
  double mLastVolume;          // extrudedVolume at the end of the previous step
  double mPending;             // volume that left the nozzle and is not laid yet
  double mLastSupport;         // the last height found under the nozzle (a fallback)
  static const size_t kNoBlob = static_cast<size_t>(-1);
  size_t mLastBlob;            // the blob the nozzle is still in (grown in place), or kNoBlob
  OmVector3 mBlobCentre;       // its centre, at the nozzle's height when it was started
  double mBlobVolume;          // its volume, m^3
  double mBlobHalfWidth;       // half its width: the nozzle is in it while within this (or a nozzle)
  bool mWarnedFull;        // past the drawn chunks
  bool mWarnedRecordFull;  // past the recorded-bead cap
  double mDeposited;
  int mCrashes;

  std::string mDumpPath;
  bool mDumpDirty;
  std::chrono::steady_clock::time_point mDumpedAt;
  std::chrono::steady_clock::time_point mChangedAt;  // when the beads last changed
  std::chrono::steady_clock::duration mDumpCost;     // the last dump's write time
};

#endif
