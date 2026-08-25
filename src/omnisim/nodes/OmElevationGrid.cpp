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

#include "OmElevationGrid.hpp"

#include "OmAffinePlane.hpp"
#include "OmBoundingSphere.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmLog.hpp"
#include "OmMFDouble.hpp"
#include "OmMatter.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmRay.hpp"
#include "OmResizeManipulator.hpp"
#include "OmRgb.hpp"
#include "OmSFBool.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSimulationState.hpp"
#include "OmTransform.hpp"
#include "OmVrmlNodeUtilities.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only

void OmElevationGrid::init() {
  mData = NULL;
  mMinHeight = 0;
  mMaxHeight = 0;
  mIs90DegreesRotated = true;

  mHeight = findMFDouble("height");
  mXDimension = findSFInt("xDimension");
  mXSpacing = findSFDouble("xSpacing");
  mYDimension = findSFInt("yDimension");
  mYSpacing = findSFDouble("ySpacing");
  mThickness = findSFDouble("thickness");
}

OmElevationGrid::OmElevationGrid(OmTokenizer *tokenizer) : OmGeometry("ElevationGrid", tokenizer) {
  init();

  if (tokenizer == NULL) {
    mXDimension->setValueNoSignal(2);
    mYDimension->setValueNoSignal(2);
  }
}

OmElevationGrid::OmElevationGrid(const OmElevationGrid &other) : OmGeometry(other) {
  init();
}

OmElevationGrid::OmElevationGrid(const OmNode &other) : OmGeometry(other) {
  init();
}

OmElevationGrid::~OmElevationGrid() {
  delete[] mData;
}

void OmElevationGrid::preFinalize() {
  OmGeometry::preFinalize();

  sanitizeFields();

  if (isInBoundingObject()) {
    if (OmNodeUtilities::findUpperMatter(this)->nodeType() == WB_NODE_FLUID) {
      parsingWarn("The ElevationGrid geometry cannot be used as a Fluid boundingObject. Immersions will have not effect.\n");
      // TODO: enable dHeightField for immersion detection in src/ode/ode/fluid_dynamics
    }
    isSuitableForInsertionInBoundingObject(true);  // boundingObject specific warnings
  }
}

void OmElevationGrid::postFinalize() {
  OmGeometry::postFinalize();
  connect(mHeight, &OmMFDouble::changed, this, &OmElevationGrid::updateHeight);
  connect(mXDimension, &OmSFInt::changed, this, &OmElevationGrid::updateXDimension);
  connect(mXSpacing, &OmSFDouble::changed, this, &OmElevationGrid::updateXSpacing);
  connect(mYDimension, &OmSFInt::changed, this, &OmElevationGrid::updateYDimension);
  connect(mYSpacing, &OmSFDouble::changed, this, &OmElevationGrid::updateYSpacing);
  connect(mThickness, &OmSFDouble::changed, this, &OmElevationGrid::updateThickness);
}

void OmElevationGrid::createWrenObjects() {
  OmGeometry::createWrenObjects();

  emit wrenObjectsCreated();
}

void OmElevationGrid::rescale(const OmVector3 &scale) {
  if (scale.x() != 1.0) {
    // rescale x spacing
    setXspacing(xSpacing() * scale.x());
  }

  if (scale.y() != 0.0) {
    // rescale y spacing
    setYspacing(ySpacing() * scale.y());
  }

  if (scale.z() != 0.0) {
    // rescale height
    setHeightScaleFactor(scale.z());
  }
}

bool OmElevationGrid::sanitizeFields() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mThickness, 0.0))
    return false;

  if (OmFieldChecker::resetIntIfNegative(this, mXDimension, 0))
    return false;

  if (OmFieldChecker::resetDoubleIfNonPositive(this, mXSpacing, 1.0))
    return false;

  if (OmFieldChecker::resetIntIfNegative(this, mYDimension, 0))
    return false;

  if (OmFieldChecker::resetDoubleIfNonPositive(this, mYSpacing, 1.0))
    return false;

  checkHeight();

  return true;
}

void OmElevationGrid::checkHeight() {
  const int xd = mXDimension->value();
  const int yd = mYDimension->value();
  const int xdyd = xd * yd;

  const int extra = mHeight->size() - xdyd;
  if (extra > 0)
    parsingWarn(tr("'height' contains %1 ignored extra value(s).").arg(extra));

  // find min/max height
  mMinHeight = 0;
  mMaxHeight = 0;
  mHeight->findMinMax(&mMinHeight, &mMaxHeight);

  // adjust min/max height if height field is not complete
  if (mHeight->size() < xdyd) {
    if (mMinHeight > 0.0)
      mMinHeight = 0.0;
    if (mMaxHeight < 0.0)
      mMaxHeight = 0.0;
  }
}

void OmElevationGrid::updateHeight() {
  if (!sanitizeFields())
    return;

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();
  emit changed();
}

void OmElevationGrid::updateThickness() {
  if (!sanitizeFields())
    return;

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmElevationGrid::updateXDimension() {
  if (!sanitizeFields())
    return;

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmElevationGrid::updateXSpacing() {
  if (!sanitizeFields())
    return;

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmElevationGrid::updateYDimension() {
  if (!sanitizeFields())
    return;

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmElevationGrid::updateYSpacing() {
  if (!sanitizeFields())
    return;

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmElevationGrid::createResizeManipulator() {
  mResizeManipulator =
    new OmRegularResizeManipulator(uniqueId(), OmWrenAbstractResizeManipulator::ResizeConstraint::NO_CONSTRAINT);
}

void OmElevationGrid::setResizeManipulatorDimensions() {
  OmVector3 scale(xSpacing(), ySpacing(), 1.0f);

  const OmTransform *const up = upperTransform();
  if (up)
    scale *= up->absoluteScale();

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

bool OmElevationGrid::areSizeFieldsVisibleAndNotRegenerator() const {
  const OmField *const xSpacingField = findField("xSpacing", true);
  const OmField *const ySpacingField = findField("ySpacing", true);
  return OmVrmlNodeUtilities::isVisible(xSpacingField) && OmVrmlNodeUtilities::isVisible(ySpacingField) &&
         !OmNodeUtilities::isTemplateRegeneratorField(xSpacingField) &&
         !OmNodeUtilities::isTemplateRegeneratorField(ySpacingField);
}

QStringList OmElevationGrid::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "height"
         << "xDimension"
         << "xSpacing"
         << "yDimension"
         << "ySpacing"
         << "thickness";
  return fields;
}

/////////////////
// ODE objects //
/////////////////

dGeomID OmElevationGrid::createOdeGeom(dSpaceID space) {
  if (setOdeHeightfieldData() == false)
    return NULL;

  // Creates a height field with without dSpace.
  // We need to translate the height field because in VRML the coordinate center is located in
  // one corner of the grid, while in ODE, it is located in the middle of the grid.
  mLocalOdeGeomOffsetPosition = OmVector3(scaledWidth() / 2.0, scaledDepth() / 2.0, 0.0);
  (void)space;
  return NULL;  // ODE is gone: the Newton heightfield is a separate registration path
}

bool OmElevationGrid::setOdeHeightfieldData() {
  if (isAValidBoundingObject(false, false) == false)
    return false;

  const int xd = mXDimension->value();
  const int yd = mYDimension->value();
  const int xdyd = xd * yd;
  // Creates height field data
  delete[] mData;
  mData = new double[xdyd];
  memset(mData, 0, xdyd * sizeof(double));
  mHeight->copyItemsTo(mData, xdyd);

  // Inverse mData lines for ODE
  for (int i = 0; i < yd / 2; i++) {  // integer division
    for (int j = 0; j < xd; j++) {
      double temp = mData[i * xd + j];
      mData[i * xd + j] = mData[(yd - 1 - i) * xd + j];
      mData[(yd - 1 - i) * xd + j] = temp;
    }
  }

  return true;
}

void OmElevationGrid::applyToOdeData(bool correctSolidMass) {
  if (setOdeHeightfieldData() == false)
    return;

  if (mOdeGeom == NULL) {
    if (areOdeObjectsCreated())
      emit validElevationGridInserted();
    return;
  }
}

double OmElevationGrid::scaledWidth() const {
  return fabs(absoluteScale().x() * width());
}

double OmElevationGrid::scaledDepth() const {
  return fabs(absoluteScale().y() * depth());
}

bool OmElevationGrid::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidDimensions = mXDimension->value() < 2 || mYDimension->value() < 2;
  const bool invalidSpacings = mXSpacing->value() <= 0.0 || mYSpacing->value() < 0.0;
  const bool invalid = invalidDimensions || invalidSpacings;

  if (warning) {
    if (mXDimension->value() < 2)
      parsingWarn(tr("Invalid 'xDimension' (should be greater than 1) for use in boundingObject."));

    if (mYDimension->value() < 2)
      parsingWarn(tr("Invalid 'yDimension' (should be greater than 1) for use in boundingObject."));

    if (invalidSpacings)
      parsingWarn(tr("'height' must be positive when used in a 'boundingObject'."));

    if (invalid)
      parsingWarn(tr("Cannot create the associated physics object."));
  }

  return !invalid;
}

bool OmElevationGrid::isAValidBoundingObject(bool checkOde, bool warning) const {
  return isSuitableForInsertionInBoundingObject(warning && isInBoundingObject()) &&
         OmGeometry::isAValidBoundingObject(checkOde, warning);
}

/////////////////
// Ray Tracing //
/////////////////

bool OmElevationGrid::pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet) const {
  OmVector3 localCollisionPoint;
  const double collisionDistance = computeLocalCollisionPoint(ray, localCollisionPoint);
  if (collisionDistance < 0)
    return false;

  const double sizeX = scaledWidth();
  const double sizeY = scaledDepth();

  const double u = (double)localCollisionPoint.x() / sizeX;
  const double v = 1 - (double)localCollisionPoint.y() / sizeY;

  // result
  uv.setXy(u, v);
  return true;
}

double OmElevationGrid::computeDistance(const OmRay &ray) const {
  OmVector3 localCollisionPoint;
  return computeLocalCollisionPoint(ray, localCollisionPoint);
}

double OmElevationGrid::computeLocalCollisionPoint(const OmRay &ray, OmVector3 &localCollisionPoint) const {
  double dx = mXSpacing->value() * absoluteScale().x();
  double dy = mYSpacing->value() * absoluteScale().y();
  int numX = mXDimension->value();
  int numY = mYDimension->value();
  double minDistance = std::numeric_limits<double>::infinity();

  int size = numX * numY;
  double *data = new double[size];
  memset(data, 0, size * sizeof(double));
  mHeight->copyItemsTo(data, size);

  OmRay localRay(ray);
  const OmPose *const up = upperPose();
  if (up) {
    localRay.setDirection(ray.direction() * up->matrix());
    OmVector3 origin = up->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
    localRay.setOrigin(origin);
    localRay.normalize();
  }

  for (int y = 0; y < (numY - 1); y++) {
    for (int x = 0; x < (numX - 1); x++) {
      OmVector3 vertexA(x * dx, (y + 1) * dy, data[(y + 1) * numX + x] * absoluteScale().z());
      OmVector3 vertexB(x * dx, y * dy, data[y * numX + x] * absoluteScale().z());
      OmVector3 vertexC((x + 1) * dx, (y + 1) * dy, data[(y + 1) * numX + x + 1] * absoluteScale().z());
      OmVector3 vertexD((x + 1) * dx, y * dy, data[y * numX + x + 1] * absoluteScale().z());

      // first triangle: ABC
      OmAffinePlane plane(vertexA, vertexB, vertexC);
      std::pair<bool, double> result = localRay.intersects(plane, true);
      if (result.first && result.second > 0 && result.second < minDistance) {
        // check finite plane bounds
        OmVector3 p = localRay.origin() + result.second * localRay.direction();
        if (p.x() >= vertexA.x() && p.x() <= vertexC.x() && p.y() >= vertexB.y() && p.y() <= vertexA.y()) {
          minDistance = result.second;
          localCollisionPoint = p;
        }
      }

      // second triangle: BCD
      plane = OmAffinePlane(vertexD, vertexC, vertexB);
      result = localRay.intersects(plane, true);

      if (result.first && result.second > 0 && result.second < minDistance) {
        // check finite plane bounds
        OmVector3 p = localRay.origin() + result.second * localRay.direction();
        if (p.x() >= vertexB.x() && p.x() <= vertexC.x() && p.y() >= vertexD.y() && p.y() <= vertexC.y()) {
          minDistance = result.second;
          localCollisionPoint = p;
        }
      }
    }
  }

  delete[] data;

  if (minDistance == std::numeric_limits<double>::infinity())
    return -1;

  return minDistance;
}

void OmElevationGrid::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();

  // create list of vertices
  const int xd = mXDimension->value();
  const int yd = mYDimension->value();
  const double xs = mXSpacing->value();
  const double ys = mYSpacing->value();
  const int size = yd * xd;
  double *h = new double[size];
  memset(h, 0, size * sizeof(double));
  mHeight->copyItemsTo(h, size);
  OmVector3 *vertices = new OmVector3[size];
  int index = 0;
  double posY = 0.0;
  for (int y = 0; y < yd; y++, posY += ys) {
    double posX = 0.0;
    for (int x = 0; x < xd; x++, posX += xs) {
      vertices[index] = OmVector3(posX, posY, h[index]);
      ++index;
    }
  }
  delete[] h;

  // Ritter's bounding sphere approximation
  // (see description in OmIndexedFaceSet::recomputeBoundingSphere)
  OmVector3 p2(vertices[0]);
  OmVector3 p1;
  double maxDistance;  // squared distance
  for (int i = 0; i < 2; ++i) {
    maxDistance = 0.0;
    p1 = p2;
    for (int j = 0; j < size; ++j) {
      const double d = p1.distance2(vertices[j]);
      if (d > maxDistance) {
        maxDistance = d;
        p2 = vertices[j];
      }
    }
  }
  mBoundingSphere->set((p2 + p1) * 0.5, sqrt(maxDistance) * 0.5);

  for (int j = 0; j < size; ++j)
    mBoundingSphere->enclose(vertices[j]);

  delete[] vertices;
}

////////////////////////
// Friction Direction //
////////////////////////

OmVector3 OmElevationGrid::computeFrictionDirection(const OmVector3 &normal) const {
  parsingWarn(tr("A ElevationGrid is used in a Bounding object using an asymmetric friction. ElevationGrid does not support "
                 "asymmetric friction"));
  return OmVector3(0, 0, 0);
}
