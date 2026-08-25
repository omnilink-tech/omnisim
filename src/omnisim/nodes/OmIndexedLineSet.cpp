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

#include "OmIndexedLineSet.hpp"

#include "OmBoundingSphere.hpp"
#include "OmCoordinate.hpp"
#include "OmField.hpp"
#include "OmMFInt.hpp"
#include "OmMFVector3.hpp"
#include "OmNodeUtilities.hpp"
#include "OmSFNode.hpp"

void OmIndexedLineSet::init() {
  mCoord = findSFNode("coord");
  mCoordIndex = findMFInt("coordIndex");
}

OmIndexedLineSet::OmIndexedLineSet(OmTokenizer *tokenizer) : OmGeometry("IndexedLineSet", tokenizer) {
  init();
}

OmIndexedLineSet::OmIndexedLineSet(const OmIndexedLineSet &other) : OmGeometry(other) {
  init();
}

OmIndexedLineSet::OmIndexedLineSet(const OmNode &other) : OmGeometry(other) {
  init();
}

OmIndexedLineSet::~OmIndexedLineSet() {
}

void OmIndexedLineSet::postFinalize() {
  OmGeometry::postFinalize();

  connect(mCoord, &OmSFNode::changed, this, &OmIndexedLineSet::updateCoord);
  connect(mCoordIndex, &OmMFInt::changed, this, &OmIndexedLineSet::updateCoordIndex);

  if (coord())
    connect(coord(), &OmCoordinate::fieldChanged, this, &OmIndexedLineSet::updateCoord, Qt::UniqueConnection);
}

OmCoordinate *OmIndexedLineSet::coord() const {
  return static_cast<OmCoordinate *>(mCoord->value());
}

void OmIndexedLineSet::createWrenObjects() {
  OmGeometry::createWrenObjects();
  updateCoord();
  emit wrenObjectsCreated();
}

bool OmIndexedLineSet::sanitizeFields() {
  if (!coord() || coord()->point().isEmpty()) {
    parsingWarn(tr("A 'Coordinate' node should be present in the 'coord' field with at least two items."));
    return false;
  }

  if (mCoordIndex->isEmpty() || estimateIndexCount() < 2) {
    parsingWarn(tr("The 'coordIndex' field should have at least two items."));
    return false;
  }

  return true;
}

void OmIndexedLineSet::reset(const QString &id) {
  OmGeometry::reset(id);

  OmNode *const c = mCoord->value();
  if (c)
    c->reset(id);
}

void OmIndexedLineSet::updateCoord() {
  if (!sanitizeFields())
    return;

  if (coord())
    connect(coord(), &OmCoordinate::fieldChanged, this, &OmIndexedLineSet::updateCoord, Qt::UniqueConnection);

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  emit changed();
}

void OmIndexedLineSet::updateCoordIndex() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  emit changed();
}

int OmIndexedLineSet::estimateIndexCount(bool isOutlineMesh) const {
  int ni = 0;
  int s1 = coord()->point().size();
  OmMFInt::Iterator it(*mCoordIndex);
  int i = it.next();
  while (it.hasNext()) {
    int j = it.next();
    if (i != -1 && j != -1 && i < s1 && j < s1)
      ni += 2;
    i = j;
  }
  return ni;
}

void OmIndexedLineSet::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();

  if (!coord() || mCoordIndex->isEmpty())
    return;

  const OmMFVector3 &points = coord()->point();
  if (points.size() == 0)
    return;

  // Ritter's bounding sphere approximation
  // (see description in OmIndexedFaceSet::recomputeBoundingSphere)
  OmMFInt::Iterator it(*mCoordIndex);
  OmVector3 p2(points.item(it.next()));
  OmVector3 p1;
  double maxDistance;  // squared distance
  for (int i = 0; i < 2; ++i) {
    maxDistance = 0.0;
    p1 = p2;
    while (it.hasNext()) {
      const int index = it.next();
      if (index >= 0 && index < points.size()) {  // skip '-1' or other invalid indices.
        const OmVector3 &point = points.item(index);
        const double d = p1.distance2(point);
        if (d > maxDistance) {
          maxDistance = d;
          p2 = point;
        }
      }
    }
    it.toFront();
  }
  mBoundingSphere->set((p2 + p1) * 0.5, sqrt(maxDistance) * 0.5);

  while (it.hasNext()) {
    const int index = it.next();
    if (index >= 0 && index < points.size())  // skip '-1' or other invalid indices.
      mBoundingSphere->enclose(points.item(index));
  }
}

QStringList OmIndexedLineSet::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "coordIndex";
  return fields;
}

////////////////////////
// Friction Direction //
////////////////////////

OmVector3 OmIndexedLineSet::computeFrictionDirection(const OmVector3 &normal) const {
  parsingWarn(tr("A IndexedLineSet is used in a Bounding object using an asymmetric friction. IndexedLineSet does not support "
                 "asymmetric friction"));
  return OmVector3(0, 0, 0);
}
