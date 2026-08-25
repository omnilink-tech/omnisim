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

#include "OmPointSet.hpp"

#include "OmBoundingSphere.hpp"
#include "OmColor.hpp"
#include "OmCoordinate.hpp"
#include "OmField.hpp"
#include "OmMFColor.hpp"
#include "OmMFVector3.hpp"
#include "OmNodeUtilities.hpp"
#include "OmSFNode.hpp"

void OmPointSet::init() {
  mColor = findSFNode("color");
  mCoord = findSFNode("coord");
}

OmPointSet::OmPointSet(OmTokenizer *tokenizer) : OmGeometry("PointSet", tokenizer) {
  init();
}

OmPointSet::OmPointSet(const OmPointSet &other) : OmGeometry(other) {
  init();
}

OmPointSet::OmPointSet(const OmNode &other) : OmGeometry(other) {
  init();
}

OmPointSet::~OmPointSet() {
}

void OmPointSet::postFinalize() {
  OmGeometry::postFinalize();

  connect(mCoord, &OmSFNode::changed, this, &OmPointSet::updateCoord);
  connect(mColor, &OmSFNode::changed, this, &OmPointSet::updateColor);

  if (coord())
    connect(coord(), &OmCoordinate::fieldChanged, this, &OmPointSet::updateCoord, Qt::UniqueConnection);

  if (color())
    connect(color(), &OmColor::fieldChanged, this, &OmPointSet::updateColor, Qt::UniqueConnection);
}

OmCoordinate *OmPointSet::coord() const {
  return static_cast<OmCoordinate *>(mCoord->value());
}

OmColor *OmPointSet::color() const {
  return static_cast<OmColor *>(mColor->value());
}

void OmPointSet::createWrenObjects() {
  OmGeometry::createWrenObjects();
  updateCoord();
  emit wrenObjectsCreated();
}

bool OmPointSet::sanitizeFields() {
  if (!coord() || coord()->point().isEmpty()) {
    parsingWarn(tr("A non-empty 'Coordinate' node should be present in the 'coord' field."));
    return false;
  }

  if (color() && color()->color().size() != coord()->pointSize()) {
    parsingWarn(tr("If a 'Color' node is present in the 'color' field, it should have the same number of component as the "
                   "'Coordinate' node in the 'coord' field."));
    if (color()->color().isEmpty())
      return false;
    else
      parsingWarn(tr("Only the %1 first points will be drawn.").arg(qMin(color()->color().size(), coord()->point().size())));
  }

  return true;
}

void OmPointSet::updateCoord() {
  if (coord())
    connect(coord(), &OmCoordinate::fieldChanged, this, &OmPointSet::updateCoord, Qt::UniqueConnection);

  if (!sanitizeFields())
    return;

  if (mBoundingSphere)
    mBoundingSphere->setOwnerSizeChanged();

  emit changed();
}

void OmPointSet::updateColor() {
  if (color())
    connect(color(), &OmCoordinate::fieldChanged, this, &OmPointSet::updateColor, Qt::UniqueConnection);

  if (!sanitizeFields())
    return;

  emit changed();
}

void OmPointSet::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();

  if (!coord())
    return;

  const OmMFVector3 &points = coord()->point();
  if (points.size() == 0)
    return;

  // Ritter's bounding sphere approximation
  OmMFVector3::Iterator it(points);
  OmVector3 p2 = it.next();
  OmVector3 p1;
  double maxDistance;  // squared distance
  for (int i = 0; i < 2; ++i) {
    maxDistance = 0.0;
    p1 = p2;
    while (it.hasNext()) {
      const OmVector3 &point = it.next();
      const double d = p1.distance2(point);
      if (d > maxDistance) {
        maxDistance = d;
        p2 = point;
      }
    }
    it.toFront();
  }
  mBoundingSphere->set((p2 + p1) * 0.5, sqrt(maxDistance) * 0.5);

  while (it.hasNext())
    mBoundingSphere->enclose(it.next());
}
