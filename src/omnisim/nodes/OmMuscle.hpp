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

#ifndef OM_MUSCLE_HPP
#define OM_MUSCLE_HPP

//
// Description: Graphical representation of an artificial muscle.
//              This node is a child node of LinearMotor and RotationalMotor
//              nodes. The mesh has a spheroid shape with constant volume that
//              is computed based on the the maximum radius field and the
//              minimal distance between the parent transform and endPoint solid
//              center positions.
//

#include "OmBaseNode.hpp"
#include "OmMatrix4.hpp"

#include <vector>

class QImage;

class OmMuscle : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmMuscle(OmTokenizer *tokenizer = NULL);
  OmMuscle(const OmMuscle &other);
  explicit OmMuscle(const OmNode &other);
  virtual ~OmMuscle() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_MUSCLE; }
  void postFinalize() override;
  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override {
    return QList<const OmBaseNode *>() << this;
  };

  void animateMesh();

  // ---- P2: the wgpu DRAW SOURCE (docs/developer/wren-deletion-runbook.md) ------------
  //
  // A Muscle is PROCEDURAL and, unlike a Cloth or a SoftBody, owns NO retained CPU
  // geometry: the spheroid is synthesised on demand from three INPUTS -- mMatrix,
  // mHeight, mRadius -- computed in computeStretchedDimensions(). wgpuVertexStream()
  // is that synthesis loop, emitting into a byte buffer.
  //
  // ⚠ The surface is redrawn from those inputs EVERY step (the muscle bulges as the joint
  // loads), so the caller must re-upload through OmWgpuMeshCache's per-(cache, mesh) epoch,
  // never a process-global "the clock advanced" flag -- see the long note on
  // OmWgpuMeshCache::vertexEpochIs for the freeze that produces.
  //
  // THE REGISTRY. OmDeformableFrameListener's muscle list is a PENDING-REBUILD queue,
  // not a liveness registry: frameStarted() CLEARS it after running it, because a muscle
  // re-subscribes from its own field-change paths. So anyDeformablesSubscribed() cannot
  // answer "does this world have a Muscle?" and this pair does. File-static, ctor/dtor
  // maintained, so a world with no Muscle pays one empty-test on a vector that already
  // exists -- nothing is constructed and no scene is walked.
  static bool anyMuscles();
  static const std::vector<OmMuscle *> &liveMuscles();

  // The CURRENT spheroid in the wgpu vertex layout (pos3 + normal3 + uv2, stride 32),
  // expressed in the PARENT POSE's frame -- exactly what WREN rendered (the node's
  // transform was an identity child of the upper pose's). Pair it with
  // wgpuWorldMatrix() as the model matrix. False when the node is not renderable yet.
  bool wgpuVertexStream(std::vector<unsigned char> &out) const;

  // The triangle list addressing those vertices. FIXED for the lifetime of the process
  // (SUBDIVISION is a compile-time constant), so it is built once and shared by every
  // Muscle and every mesh cache.
  static const std::vector<unsigned int> &wgpuIndices();

  // The parent pose's world matrix -- the model matrix for the vertices above. False when
  // there is no upper pose (a Muscle outside a Joint, which cannot happen in a valid world
  // but must not crash a renderer).
  bool wgpuWorldMatrix(OmMatrix4 &out) const;

  // The muscle's diffuse RIGHT NOW: the `color` field blended by the contraction
  // status, exactly as the legacy WREN material computed it. Default red when
  // `color` is empty.
  void wgpuDiffuse(float rgb3[3]) const;

  // The muscle texture (gl:textures/muscle.png), loaded lazily and owned by the node. WREN
  // multiplies the whole lit term by it (phong.frag:214), which is what the wgpu draw's
  // albedo texture does. Null when the file cannot be read -> flat diffuse.
  const QImage *wgpuTextureImage() const;

  bool wgpuCastShadows() const;
  bool wgpuVisible() const;

private slots:
  void updateVolume();
  void computeStretchedDimensions();

private:
  OmMuscle &operator=(const OmMuscle &);  // non copyable
  OmNode *clone() const override { return new OmMuscle(*this); }
  void init();

  OmSFDouble *mVolume;
  OmSFVector3 *mStartOffset;
  OmSFVector3 *mEndOffset;
  OmMFColor *mColors;
  OmSFBool *mCastShadows;
  OmSFBool *mVisible;
  // deprecated
  OmSFDouble *mMaxRadius;

  // P2: the wgpu albedo, loaded lazily by wgpuTextureImage().
  mutable QImage *mWgpuImage;
  mutable bool mWgpuImageTried;

  const OmPose *mParentPose;
  const OmSolid *mEndPoint;
  OmMatrix4 mMatrix;
  double mHeight;
  double mPreviousHeight;
  double mRadius;
  bool mDirectionInverted;

  double mStatus;  // idle = 0, contracting < 0, relaxing > 0
  double mMaterialStatus;

private slots:
  void updateEndPoint(OmBaseNode *node);
  void updateEndPointPosition();
  void updateVisible();
  void updateStretchForce(double forcePercentage, bool immediateUpdate, int motorIndex);
  void stretch();
};

#endif
