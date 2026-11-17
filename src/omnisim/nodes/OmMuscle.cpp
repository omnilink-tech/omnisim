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

#include "OmMuscle.hpp"

#include "OmAppearance.hpp"
#include "OmFieldChecker.hpp"
#include "OmHingeJoint.hpp"
#include "OmJoint.hpp"
#include "OmMFColor.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMotor.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"  // mParentPose->matrix() -- the wgpu model matrix (P2)
#include "OmPreferences.hpp"
#include "OmSliderJoint.hpp"
#include "OmSolid.hpp"
#include "OmDeformableFrameListener.hpp"

#include <cmath>
#include <cstring>

#include <QtCore/QFileInfo>
#include <QtGui/QImage>

const int SUBDIVISION = 16;            // ellipsoid subdivision
OmVector2 *gCircleCoordinates = NULL;  // based on SUBDIVISION value

void OmMuscle::init() {
  mVolume = findSFDouble("volume");
  mStartOffset = findSFVector3("startOffset");
  mEndOffset = findSFVector3("endOffset");
  mColors = findMFColor("color");
  mCastShadows = findSFBool("castShadows");
  mVisible = findSFBool("visible");

  // deprecated field
  mMaxRadius = findSFDouble("maxRadius");
  if (mVolume->value() == 0.01 && mMaxRadius->value() != 0.0)
    parsingWarn(tr("'maxRadius' field is deprecated, please use the 'volume' field instead."));

  mEndPoint = NULL;
  mHeight = 0.1;
  mPreviousHeight = 0.1;
  mRadius = 0.1;
  mStatus = 0.0;
  mMaterialStatus = mStatus;
  mDirectionInverted = false;
  mParentPose = NULL;
  mMatrix = OmMatrix4();

  // wgpu (P2)
  mWgpuImage = NULL;
  mWgpuImageTried = false;

  // precompute circle coordinates
  if (gCircleCoordinates == NULL) {
    gCircleCoordinates = new OmVector2[SUBDIVISION + 1];
    const double da = 2 * M_PI / SUBDIVISION;
    double angle = 0.0;
    for (int i = 0; i < SUBDIVISION; ++i, angle += da)
      gCircleCoordinates[i] = OmVector2(cos(angle), sin(angle));
    gCircleCoordinates[SUBDIVISION] = gCircleCoordinates[0];
  }
}

// THE MUSCLE REGISTRY (P2). File-static rather than a class member so nothing has to be
// constructed to ask "does this world have a Muscle?" -- anyMuscles() is an empty-test on a
// vector that already exists. Membership is ctor/dtor maintained, so it cannot drift from
// the set of live nodes.
//
// ⚠ THIS EXISTS BECAUSE OmDeformableFrameListener'S MUSCLE LIST IS NOT A REGISTRY.
// frameStarted() clears that list after running it (the subscriptions are event-driven
// re-render requests), so it is empty most of the time and answers a different question.
// anyDeformablesSubscribed() covers Cloth and SoftBody only, deliberately.
static std::vector<OmMuscle *> gLiveMuscles;

bool OmMuscle::anyMuscles() {
  return !gLiveMuscles.empty();
}

const std::vector<OmMuscle *> &OmMuscle::liveMuscles() {
  return gLiveMuscles;
}

OmMuscle::OmMuscle(OmTokenizer *tokenizer) : OmBaseNode("Muscle", tokenizer) {
  init();
  gLiveMuscles.push_back(this);
}

OmMuscle::OmMuscle(const OmMuscle &other) : OmBaseNode(other) {
  init();
  gLiveMuscles.push_back(this);
}

OmMuscle::OmMuscle(const OmNode &other) : OmBaseNode(other) {
  init();
  gLiveMuscles.push_back(this);
}

OmMuscle::~OmMuscle() {
  for (std::size_t i = 0; i < gLiveMuscles.size(); ++i)
    if (gLiveMuscles[i] == this) {
      gLiveMuscles.erase(gLiveMuscles.begin() + i);
      break;
    }
  delete mWgpuImage;
  mWgpuImage = NULL;
  if (isPostFinalizedCalled())
    OmDeformableFrameListener::instance()->unsubscribeMuscle(this);
}

void OmMuscle::postFinalize() {
  OmBaseNode::postFinalize();

  mParentPose = OmNodeUtilities::findUpperPose(this);
  const OmJoint *joint = dynamic_cast<OmJoint *>(parentNode()->parentNode());
  updateEndPoint(joint->solidEndPoint());
  OmDeformableFrameListener::instance()->subscribeMuscle(this);

  connect(joint, &OmJoint::endPointChanged, this, &OmMuscle::updateEndPoint);
  connect(mVolume, &OmSFDouble::changed, this, &OmMuscle::updateVolume);
  connect(mStartOffset, &OmSFVector3::changed, this, &OmMuscle::computeStretchedDimensions);
  connect(mEndOffset, &OmSFVector3::changed, this, &OmMuscle::computeStretchedDimensions);
  // D1.4: `color` and `castShadows` need no slots any more -- wgpuDiffuse() and
  // wgpuCastShadows() read the fields live at collect time.
  connect(mVisible, &OmSFBool::changed, this, &OmMuscle::updateVisible);
}

void OmMuscle::updateVolume() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mVolume, 0.2))
    return;

  // field check
  if (areWrenObjectsInitialized())
    computeStretchedDimensions();
}

void OmMuscle::updateVisible() {
  // D1.4: no WREN node visibility to flip -- wgpuVisible() reads the field
  // live. Re-run the geometry so a muscle turned back ON has fresh dimensions
  // and re-subscribes for its redraw.
  if (areWrenObjectsInitialized())
    computeStretchedDimensions();
}

void OmMuscle::updateEndPointPosition() {
  updateEndPoint(const_cast<OmSolid *>(mEndPoint));
}

void OmMuscle::updateEndPoint(OmBaseNode *node) {
  const OmSolid *solid = dynamic_cast<OmSolid *>(node);
  const OmJoint *joint = dynamic_cast<OmJoint *>(parentNode()->parentNode());
  if (mEndPoint != solid) {
    if (mEndPoint) {
      disconnect(mEndPoint->translationFieldValue(), &OmSFVector3::changedByOde, this, &OmMuscle::stretch);
      disconnect(joint, &OmJoint::updateMuscleStretch, this, &OmMuscle::stretch);
      disconnect(mEndPoint, &OmSolid::translationOrRotationChangedByUser, this, &OmMuscle::updateEndPointPosition);
    }
    mEndPoint = solid;
    if (mEndPoint) {
      // listen to solid endPoint position change
      connect(mEndPoint->translationFieldValue(), &OmSFVector3::changedByOde, this, &OmMuscle::stretch, Qt::UniqueConnection);
      // listen to joint force percentage
      connect(joint, &OmJoint::updateMuscleStretch, this, &OmMuscle::updateStretchForce, Qt::UniqueConnection);
      // listen to solid endPoint artificial translation change
      connect(mEndPoint, &OmSolid::translationOrRotationChangedByUser, this, &OmMuscle::updateEndPointPosition,
              Qt::UniqueConnection);
    }
  }

  computeStretchedDimensions();
}

void OmMuscle::updateStretchForce(double forcePercentage, bool immediateUpdate, int motorIndex) {
  const OmMotor *motor = dynamic_cast<OmMotor *>(parentNode());
  if (motor->positionIndex() != motorIndex)
    return;
  mStatus = forcePercentage;
  if (mStatus > 1.0)
    mStatus = 1.0;
  if (mStatus < -1.0)
    mStatus = -1.0;
  if (mStatus > 0.0)  // relaxing
    mDirectionInverted = mHeight > mPreviousHeight;
  else if (mStatus < 0.0)  // contracting
    mDirectionInverted = mHeight < mPreviousHeight;

  if (immediateUpdate)
    computeStretchedDimensions();
}

void OmMuscle::stretch() {
  computeStretchedDimensions();
}

void OmMuscle::computeStretchedDimensions() {
  if (!mEndPoint || !mVisible->value())
    return;

  const OmVector3 t =
    mEndPoint->rotation().toMatrix3() * mEndOffset->value() + mEndPoint->translation() - mStartOffset->value();
  mPreviousHeight = mHeight;
  mHeight = t.length();
  // keep spheroid volume constant: V = 4 / 3 * M_PI * height * radius^2
  mRadius = sqrt((3.0 * mVolume->value()) / (4.0 * M_PI * mHeight));

  // compute mesh matrix
  mMatrix.setIdentity();
  mMatrix.setTranslation(mStartOffset->value());
  const OmVector3 y(0, 1, 0);
  double dotProduct = y.dot(t) / mHeight;
  if (fabs(1 - dotProduct) > 1e-06) {
    OmVector3 w = y.cross(t);
    w.normalize();
    double angle = OmMathsUtilities::clampedAcos(dotProduct);
    assert(!std::isnan(angle));
    mMatrix.setRotation(w.x(), w.y(), w.z(), angle);
  }

  if (areWrenObjectsInitialized())
    OmDeformableFrameListener::instance()->subscribeMuscle(this);
}

// ============================ P2: the wgpu draw source ============================
//
// The legacy WREN per-step re-synthesis (updateMeshCoordinates), re-emitted into an
// interleaved byte buffer. Term for term, in the same order, from the same three inputs
// (mMatrix, mHeight, mRadius). The vertex COUNT is a compile-time constant, which is
// what lets the cache re-upload in place every step instead of releasing and
// re-acquiring.
//
//   top    SUBDIVISION                       vertices, all at the pole,    normal (0,-1,0)
//   body   (SUBDIVISION-1) x (SUBDIVISION+1) vertices, the spheroid wall
//   bottom SUBDIVISION                       vertices, all at (0,h,0),     normal (0,+1,0)
static const int kMuscleVertexCount = SUBDIVISION + (SUBDIVISION - 1) * (SUBDIVISION + 1) + SUBDIVISION;

// One interleaved vertex record: pos3 + normal3 + uv2, stride 32.
//
// ⚠ THE NORMALS ARE ANALYTIC AND NOT UNIT LENGTH AS WREN WRITES THEM -- the body term is
// (cx, y * h2 / radius, cz), whose length depends on the spheroid's aspect ratio -- and
// WREN's phong.frag normalizes in the fragment shader. The wgpu scene shader normalizes
// its interpolated normal too, but the OmniLight bake and the shadow normal-offset read
// the attribute directly, so normalise HERE and the two agree everywhere.
static void muscleEmitVertex(unsigned char *dst, std::size_t i, const OmVector3 &pos, const OmVector3 &nrm,
                             float u, float v) {
  float rec[8];
  rec[0] = static_cast<float>(pos.x());
  rec[1] = static_cast<float>(pos.y());
  rec[2] = static_cast<float>(pos.z());
  double nx = nrm.x(), ny = nrm.y(), nz = nrm.z();
  const double len = sqrt(nx * nx + ny * ny + nz * nz);
  if (len > 1e-12) {
    nx /= len;
    ny /= len;
    nz /= len;
  } else {
    nx = 0.0;
    ny = 1.0;
    nz = 0.0;
  }
  rec[3] = static_cast<float>(nx);
  rec[4] = static_cast<float>(ny);
  rec[5] = static_cast<float>(nz);
  rec[6] = u;
  rec[7] = v;
  memcpy(dst + i * 32u, rec, 32);
}

bool OmMuscle::wgpuVertexStream(std::vector<unsigned char> &out) const {
  // The same preconditions computeStretchedDimensions() enforces before it writes mMatrix:
  // an unresolved endPoint or an invisible muscle has no surface to draw.
  if (!mEndPoint || !mVisible->value())
    return false;
  if (!(mHeight > 0.0) || !(mRadius > 0.0) || !std::isfinite(mHeight) || !std::isfinite(mRadius))
    return false;
  if (gCircleCoordinates == NULL)
    return false;

  out.resize(static_cast<std::size_t>(kMuscleVertexCount) * 32u);
  unsigned char *const dst = out.data();
  std::size_t v = 0;

  const OmMatrix3 rm = mMatrix.extracted3x3Matrix();
  const float invSub = 1.0f / SUBDIVISION;

  // top vertices -- the pole, with the UV row the legacy mesh build wrote for it
  {
    const OmVector3 p = (mMatrix * OmVector4(0, 0, 0, 1)).toVector3();
    const OmVector3 n = rm * OmVector3(0, -1, 0);
    float u = 0.5f * invSub;
    for (int i = 0; i < SUBDIVISION; ++i, u += invSub, ++v)
      muscleEmitVertex(dst, v, p, n, u, 0.0f);
  }
  // body
  {
    const double h2 = mHeight * 0.5;
    const double dy = mHeight / SUBDIVISION;
    const double normalFactor = h2 / mRadius;
    double y = dy - h2;
    float tv = invSub;
    for (int j = 1; j < SUBDIVISION; ++j, y += dy, tv += invSub) {
      const double d = y / h2;
      const double r = mRadius * sqrt(1 - d * d);
      float u = 0.0f;
      for (int i = 0; i <= SUBDIVISION; ++i, u += invSub, ++v) {
        const OmVector4 coord(r * gCircleCoordinates[i].x(), y + h2, r * gCircleCoordinates[i].y(), 1.0);
        const OmVector3 p = (mMatrix * coord).toVector3();
        const OmVector3 n =
          rm * OmVector3(gCircleCoordinates[i].x(), coord.y() * normalFactor, gCircleCoordinates[i].y());
        muscleEmitVertex(dst, v, p, n, u, tv);
      }
    }
  }
  // bottom vertices
  {
    const OmVector3 p = (mMatrix * OmVector4(0, mHeight, 0, 1)).toVector3();
    const OmVector3 n = rm * OmVector3(0, 1, 0);
    float u = 0.5f * invSub;
    for (int i = 0; i < SUBDIVISION; ++i, u += invSub, ++v)
      muscleEmitVertex(dst, v, p, n, u, 1.0f);
  }
  return v == static_cast<std::size_t>(kMuscleVertexCount);
}

const std::vector<unsigned int> &OmMuscle::wgpuIndices() {
  // The legacy WREN mesh build's index emission, verbatim. Built once for the process: the
  // topology depends only on SUBDIVISION, so every Muscle and every mesh cache shares it.
  static std::vector<unsigned int> sIndices;
  if (!sIndices.empty())
    return sIndices;
  sIndices.reserve(static_cast<std::size_t>(SUBDIVISION) * 6u +
                   static_cast<std::size_t>(SUBDIVISION - 2) * SUBDIVISION * 6u);
  // first triangles row
  for (int i = 0; i < SUBDIVISION; ++i) {
    sIndices.push_back(i);
    sIndices.push_back(i + SUBDIVISION);
    sIndices.push_back(i + SUBDIVISION + 1);
  }
  // quads (2 triangles) body
  int vIndex = SUBDIVISION;
  const int bodyRowCount = SUBDIVISION - 2;
  for (int j = 0; j < bodyRowCount; ++j) {
    int thisIndex = vIndex;
    for (int i = 0; i < SUBDIVISION; ++i, ++thisIndex) {
      const int nextRowIndex = thisIndex + SUBDIVISION + 1;
      sIndices.push_back(thisIndex);
      sIndices.push_back(nextRowIndex);
      sIndices.push_back(thisIndex + 1);
      sIndices.push_back(thisIndex + 1);
      sIndices.push_back(nextRowIndex);
      sIndices.push_back(nextRowIndex + 1);
    }
    vIndex += SUBDIVISION + 1;
  }
  // last triangles row
  for (int i = 0; i < SUBDIVISION; ++i, ++vIndex) {
    sIndices.push_back(vIndex + SUBDIVISION + 1);
    sIndices.push_back(vIndex + 1);
    sIndices.push_back(vIndex);
  }
  return sIndices;
}

bool OmMuscle::wgpuWorldMatrix(OmMatrix4 &out) const {
  // The vertices above are in the PARENT POSE's frame: WREN attached this node's
  // transform, unrotated and untranslated, under the nearest ancestor that owned
  // one, i.e. the upper pose (a Joint was transparent in that chain), and the
  // wgpu draw reproduces that by taking the upper pose's matrix as the model.
  if (!mParentPose)
    return false;
  out = mParentPose->matrix();
  return true;
}

void OmMuscle::wgpuDiffuse(float rgb3[3]) const {
  // The legacy WREN material's hardcoded default, then its status blend -- the
  // same two branches in the same order, so a contracting muscle keeps the
  // colour the WREN renderer showed.
  rgb3[0] = 1.0f;
  rgb3[1] = 0.0f;
  rgb3[2] = 0.0f;
  if (!mColors || mColors->isEmpty())
    return;
  OmRgb rgb(mColors->item(0));
  OmVector3 c(rgb.red(), rgb.green(), rgb.blue());
  int index = (0.0 == mStatus) ? 0 : (mStatus < 0.0 ? 1 : 2);
  if (mDirectionInverted && index > 0)
    index = 2 - index + 1;
  if (index >= mColors->size())
    index = mColors->size() - 1;
  if (index != 0) {
    rgb = mColors->item(index);
    const double percent = fabs(mStatus);
    const double oneMinusPercent = 1.0 - percent;
    c[0] = c[0] * oneMinusPercent + rgb.red() * percent;
    c[1] = c[1] * oneMinusPercent + rgb.green() * percent;
    c[2] = c[2] * oneMinusPercent + rgb.blue() * percent;
  }
  rgb3[0] = static_cast<float>(c[0]);
  rgb3[1] = static_cast<float>(c[1]);
  rgb3[2] = static_cast<float>(c[2]);
}

const QImage *OmMuscle::wgpuTextureImage() const {
  // Loaded once per node, on first use, from the same gl:textures path the WREN
  // material used to read. A plain CPU QImage load -- no renderer involved.
  if (!mWgpuImageTried) {
    mWgpuImageTried = true;
    const QString imagePath = QFileInfo("gl:textures/muscle.png").absoluteFilePath();
    QImage *img = new QImage(imagePath);
    if (img->isNull()) {
      delete img;
      img = NULL;
    }
    mWgpuImage = img;
  }
  return mWgpuImage;
}

bool OmMuscle::wgpuCastShadows() const {
  return mCastShadows ? mCastShadows->value() : true;
}

bool OmMuscle::wgpuVisible() const {
  return mVisible ? mVisible->value() : true;
}

void OmMuscle::animateMesh() {
  // D1.4: WREN's per-step re-synthesis into a dynamic mesh is gone -- the wgpu
  // path re-emits the spheroid from (mMatrix, mHeight, mRadius) via
  // wgpuVertexStream() every frame, and wgpuDiffuse() blends the status colour
  // live. Only the status latch remains.
  mMaterialStatus = mStatus;
}
