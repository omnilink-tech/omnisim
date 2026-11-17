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

#include "OmIndexedFaceSet.hpp"

#include "OmBoundingSphere.hpp"
#include "OmCoordinate.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmMFInt.hpp"
#include "OmNodeUtilities.hpp"
#include "OmNormal.hpp"
#include "OmResizeManipulator.hpp"
#include "OmSFBool.hpp"
#include "OmSFDouble.hpp"
#include "OmSFNode.hpp"
#include "OmTextureCoordinate.hpp"
#include "OmTriangleMesh.hpp"
#include "OmVrmlNodeUtilities.hpp"

void OmIndexedFaceSet::init() {
  mCoord = findSFNode("coord");
  mNormal = findSFNode("normal");
  mTexCoord = findSFNode("texCoord");
  mCcw = findSFBool("ccw");
  mNormalPerVertex = findSFBool("normalPerVertex");
  mCoordIndex = findMFInt("coordIndex");
  mNormalIndex = findMFInt("normalIndex");
  mTexCoordIndex = findMFInt("texCoordIndex");
  mCreaseAngle = findSFDouble("creaseAngle");
  setCcw(mCcw->value());
}

OmIndexedFaceSet::OmIndexedFaceSet(OmTokenizer *tokenizer) : OmTriangleMeshGeometry("IndexedFaceSet", tokenizer) {
  init();
}

OmIndexedFaceSet::OmIndexedFaceSet(const OmIndexedFaceSet &other) : OmTriangleMeshGeometry(other) {
  init();
}

OmIndexedFaceSet::OmIndexedFaceSet(const OmNode &other) : OmTriangleMeshGeometry(other) {
  init();
}

OmIndexedFaceSet::~OmIndexedFaceSet() {
}

void OmIndexedFaceSet::preFinalize() {
  if (isPreFinalizedCalled())
    return;

  OmTriangleMeshGeometry::preFinalize();

  OmFieldChecker::resetDoubleIfNegative(this, mCreaseAngle, 0.0);
}

void OmIndexedFaceSet::postFinalize() {
  OmTriangleMeshGeometry::postFinalize();

  connect(mCoord, &OmSFNode::changed, this, &OmIndexedFaceSet::updateCoord);
  connect(mNormal, &OmSFNode::changed, this, &OmIndexedFaceSet::updateNormal);
  connect(mTexCoord, &OmSFNode::changed, this, &OmIndexedFaceSet::updateTexCoord);
  connect(mCcw, &OmSFBool::changed, this, &OmIndexedFaceSet::updateCcw);
  connect(mNormalPerVertex, &OmSFBool::changed, this, &OmIndexedFaceSet::updateNormalPerVertex);
  connect(mCoordIndex, &OmMFInt::changed, this, &OmIndexedFaceSet::updateCoordIndex);
  connect(mNormalIndex, &OmMFInt::changed, this, &OmIndexedFaceSet::updateNormalIndex);
  connect(mTexCoordIndex, &OmMFInt::changed, this, &OmIndexedFaceSet::updateTexCoordIndex);
  connect(mCreaseAngle, &OmSFDouble::changed, this, &OmIndexedFaceSet::updateCreaseAngle);

  if (coord())
    connect(coord(), &OmCoordinate::fieldChanged, this, &OmIndexedFaceSet::updateCoord, Qt::UniqueConnection);

  if (normal())
    connect(normal(), &OmNormal::fieldChanged, this, &OmIndexedFaceSet::updateNormal, Qt::UniqueConnection);

  if (texCoord())
    connect(texCoord(), &OmTextureCoordinate::fieldChanged, this, &OmIndexedFaceSet::updateTexCoord, Qt::UniqueConnection);
}

void OmIndexedFaceSet::reset(const QString &id) {
  OmTriangleMeshGeometry::reset(id);

  OmNode *const coordNode = mCoord->value();
  if (coordNode)
    coordNode->reset(id);
  OmNode *const normalNode = mNormal->value();
  if (normalNode)
    normalNode->reset(id);
  OmNode *const texCoordNode = mTexCoord->value();
  if (texCoordNode)
    texCoordNode->reset(id);
}

void OmIndexedFaceSet::updateTriangleMesh(bool issueWarnings) {
  mTriangleMeshError = mTriangleMesh->init(
    coord() ? &(coord()->point()) : NULL, mCoordIndex, normal() ? &(normal()->vector()) : NULL, mNormalIndex,
    texCoord() ? &(texCoord()->point()) : NULL, mTexCoordIndex, mCreaseAngle->value(), mNormalPerVertex->value());

  if (issueWarnings) {
    foreach (const QString &warning, mTriangleMesh->warnings())
      parsingWarn(warning);

    if (!mTriangleMeshError.isEmpty())
      parsingWarn(tr("Cannot create IndexedFaceSet because: \"%1\".").arg(mTriangleMeshError));
  }
}

uint64_t OmIndexedFaceSet::computeHash() const {
  uint64_t hash = 0;

  if (coord() && coord()->pointSize()) {
    const double *startCoord = coord()->point().item(0).ptr();
    int size = 3 * coord()->pointSize();
    hash ^= OmTriangleMeshCache::sipHash13x(startCoord, size);
  }

  if (normal() && normal()->vectorSize()) {
    const double *startNormal = normal()->vector().item(0).ptr();
    int size = 2 * normal()->vectorSize();
    hash ^= OmTriangleMeshCache::sipHash13x(startNormal, size);
  }

  if (texCoord() && texCoord()->pointSize()) {
    const double *startTexCoord = texCoord()->point().item(0).ptr();
    int size = 2 * texCoord()->pointSize();
    hash ^= OmTriangleMeshCache::sipHash13x(startTexCoord, size);
  }

  if (coordIndex() && coordIndex()->size()) {
    const int *startCoordIndex = &(coordIndex()->item(0));
    hash ^= OmTriangleMeshCache::sipHash13x(startCoordIndex, coordIndex()->size());
  }

  if (normalIndex() && normalIndex()->size()) {
    const int *startNormalIndex = &(normalIndex()->item(0));
    hash ^= OmTriangleMeshCache::sipHash13x(startNormalIndex, normalIndex()->size());
  }

  if (texCoordIndex() && texCoordIndex()->size()) {
    const int *startTexCoordIndex = &(texCoordIndex()->item(0));
    hash ^= OmTriangleMeshCache::sipHash13x(startTexCoordIndex, texCoordIndex()->size());
  }

  hash ^= OmTriangleMeshCache::sipHash13x(creaseAngle()->valuePointer(), 1);
  hash ^= OmTriangleMeshCache::sipHash13x(ccw()->valuePointer(), 1);
  hash ^= OmTriangleMeshCache::sipHash13x(normalPerVertex()->valuePointer(), 1);

  return hash;
}

OmCoordinate *OmIndexedFaceSet::coord() const {
  return static_cast<OmCoordinate *>(mCoord->value());
}

OmNormal *OmIndexedFaceSet::normal() const {
  return static_cast<OmNormal *>(mNormal->value());
}

OmTextureCoordinate *OmIndexedFaceSet::texCoord() const {
  return static_cast<OmTextureCoordinate *>(mTexCoord->value());
}

void OmIndexedFaceSet::createResizeManipulator() {
  mResizeManipulator =
    new OmRegularResizeManipulator(uniqueId(), OmWrenAbstractResizeManipulator::ResizeConstraint::NO_CONSTRAINT);
}

bool OmIndexedFaceSet::areSizeFieldsVisibleAndNotRegenerator() const {
  const OmField *const coordinates = findField("coord", true);
  return OmVrmlNodeUtilities::isVisible(coordinates) && !OmNodeUtilities::isTemplateRegeneratorField(coordinates);
}

void OmIndexedFaceSet::attachResizeManipulator() {
  if (coord())
    OmTriangleMeshGeometry::attachResizeManipulator();
}

void OmIndexedFaceSet::updateCoord() {
  if (coord())
    connect(coord(), &OmCoordinate::fieldChanged, this, &OmIndexedFaceSet::updateCoord, Qt::UniqueConnection);

  buildWrenMesh(true);

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();  // Must be called after updateTriangleMesh()

  emit changed();
}

void OmIndexedFaceSet::updateNormal() {
  if (normal()) {
    connect(normal(), &OmNormal::fieldChanged, this, &OmIndexedFaceSet::updateNormal, Qt::UniqueConnection);
    for (int i = 0; i < normal()->vector().size(); ++i) {
      if (normal()->vector(i).isNull()) {
        normal()->setVector(i, OmVector3(0.0, 1.0, 0.0));
        parsingWarn(tr("Normal values can't be null."));
      }
    }
  }

  buildWrenMesh(true);

  emit changed();
}

void OmIndexedFaceSet::updateTexCoord() {
  if (texCoord())
    connect(texCoord(), &OmTextureCoordinate::fieldChanged, this, &OmIndexedFaceSet::updateTexCoord, Qt::UniqueConnection);

  buildWrenMesh(true);

  emit changed();
}

void OmIndexedFaceSet::updateCcw() {
  setCcw(mCcw->value());

  buildWrenMesh(true);

  emit changed();
}

void OmIndexedFaceSet::updateNormalPerVertex() {
  buildWrenMesh(true);

  emit changed();
}

void OmIndexedFaceSet::updateCoordIndex() {
  buildWrenMesh(true);

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();  // Must be called after updateTriangleMesh()

  emit changed();
}

void OmIndexedFaceSet::updateNormalIndex() {
  buildWrenMesh(true);

  emit changed();
}

void OmIndexedFaceSet::updateTexCoordIndex() {
  buildWrenMesh(true);

  emit changed();
}

void OmIndexedFaceSet::updateCreaseAngle() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mCreaseAngle, 0.0))
    return;

  buildWrenMesh(true);

  emit changed();
}

QStringList OmIndexedFaceSet::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "ccw"
         << "normalPerVertex"
         << "coordIndex"
         << "normalIndex"
         << "texCoordIndex"
         << "creaseAngle";
  return fields;
}

/////////////////////////////////////////////////////////////
//  WREN related methods for resizing by pulling handles   //
/////////////////////////////////////////////////////////////
void OmIndexedFaceSet::rescale(const OmVector3 &v) {
  coord()->rescale(v);
}

void OmIndexedFaceSet::rescaleAndTranslate(int coordinate, double scale, double translation) {
  coord()->rescaleAndTranslate(coordinate, scale, translation);
}

void OmIndexedFaceSet::rescaleAndTranslate(double factor, const OmVector3 &t) {
  coord()->rescaleAndTranslate(OmVector3(factor, factor, factor), t);
}

void OmIndexedFaceSet::rescaleAndTranslate(const OmVector3 &scale, const OmVector3 &translation) {
  coord()->rescaleAndTranslate(scale, translation);
}

void OmIndexedFaceSet::translate(const OmVector3 &v) {
  coord()->translate(v);
}

bool OmIndexedFaceSet::exportNodeHeader(OmWriter &writer) const {
  if (!writer.isW3d())
    return OmGeometry::exportNodeHeader(writer);

  // reduce the number of exported TriangleMeshGeometrys by automatically
  // using a def-use based on the mesh hash
  writer << "<" << w3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
  if (writer.indexedFaceSetDefMap().contains(mMeshKey.mHash)) {
    writer << " USE=\'" + writer.indexedFaceSetDefMap().value(mMeshKey.mHash) + "\'></" + w3dName() + ">";
    return true;
  }

  if (cTriangleMeshMap.at(mMeshKey).mNumUsers > 1)
    writer.indexedFaceSetDefMap().insert(mMeshKey.mHash, QString::number(uniqueId()));
  return false;
}
