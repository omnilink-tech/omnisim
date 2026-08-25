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

#include "OmPhysics.hpp"

#include "OmDamping.hpp"
#include "OmFieldChecker.hpp"
#include "OmMFVector3.hpp"
#include "OmSFNode.hpp"

#include <cassert>

void OmPhysics::init() {
  mDensity = findSFDouble("density");
  mMass = findSFDouble("mass");
  mCenterOfMass = findMFVector3("centerOfMass");
  mInertiaMatrix = findMFVector3("inertiaMatrix");
  mDamping = findSFNode("damping");

  mSkipUpdate = false;
  mHasAvalidInertiaMatrix = true;
  mMode = INVALID;
}

OmPhysics::OmPhysics(OmTokenizer *tokenizer) : OmBaseNode("Physics", tokenizer) {
  init();
}

OmPhysics::OmPhysics(const OmPhysics &other) : OmBaseNode(other) {
  init();
}

OmPhysics::OmPhysics(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmPhysics::~OmPhysics() {
}

void OmPhysics::preFinalize() {
  OmBaseNode::preFinalize();

  if (damping())
    damping()->preFinalize();

  checkDensity();
  checkMass();
  checkMassAndDensity();

  int size = mInertiaMatrix->size();
  if (size == 1) {
    mInertiaMatrix->insertDefaultItem(1);
    parsingWarn(tr("'inertiaMatrix' must have exactly 0 or 2 lines. Second line inserted."));
  } else if (size > 2) {
    do {
      mInertiaMatrix->removeItem(2);
    } while (mInertiaMatrix->size() > 2);
    parsingWarn(tr("'inertiaMatrix' must have exactly 0 or 2 lines. Extra lines skipped."));
  }

  size = mCenterOfMass->size();
  if (size > 1) {
    do {
      mCenterOfMass->removeItem(1);
    } while (mCenterOfMass->size() > 1);
    parsingWarn(tr("'centerOfMass' must have exactly 0 or 1 line. Extra lines skipped."));
  }

  checkInertiaMatrix(true);
  updateMode();
  updateDamping();
}

void OmPhysics::postFinalize() {
  OmBaseNode::postFinalize();

  if (damping())
    damping()->postFinalize();

  connect(mDensity, &OmSFDouble::changed, this, &OmPhysics::updateDensity);
  connect(mMass, &OmSFDouble::changed, this, &OmPhysics::updateMass);
  connect(mCenterOfMass, &OmMFVector3::changed, this, &OmPhysics::updateCenterOfMass);
  connect(mInertiaMatrix, &OmMFVector3::changed, this, &OmPhysics::updateInertiaMatrix);
  connect(mDamping, &OmSFNode::changed, this, &OmPhysics::updateDamping);
}

void OmPhysics::reset(const QString &id) {
  OmBaseNode::reset(id);

  OmNode *const d = mDamping->value();
  if (d)
    d->reset(id);
}

void OmPhysics::checkMass() {
  if (mDensity->value() <= 0.0)
    OmFieldChecker::resetDoubleIfNonPositive(this, mMass, 1);
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mMass, -1, -1);
}

void OmPhysics::checkDensity() {
  if (mMass->value() <= 0.0)
    OmFieldChecker::resetDoubleIfNonPositive(this, mDensity, 1000);
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mDensity, 1000, -1);
}

void OmPhysics::checkMassAndDensity() const {
  const double m = mMass->value();
  const double d = mDensity->value();
  if (m == -1.0 && d == -1.0) {
    parsingWarn(tr("Either the 'mass' or the 'density' must be specified."));
    return;
  }

  if (m > 0.0 && d > 0.0) {
    parsingWarn(tr("Both 'density' and 'mass' specified: the 'density' will be ignored."));
    return;
  }
}

void OmPhysics::checkCenterOfMass() const {
  const bool validCom = mCenterOfMass->size() == 1;

  const int size = mInertiaMatrix->size();
  if (!validCom && size == 2)
    parsingInfo(tr("The center of mass must also be specified when using a custom inertia matrix."));
  // else if (validCom && size == 0)
  //  parsingInfo(tr("The inertia matrix must also be specified when using a center of mass."));
}

void OmPhysics::checkInertiaMatrix(bool showInfo) {
  mHasAvalidInertiaMatrix = true;
  const int size = mInertiaMatrix->size();

  if (size != 2) {
    mHasAvalidInertiaMatrix = false;
    // if (showInfo && size == 0 && mCenterOfMass->size() == 1)
    //  parsingWarn(tr("Remove 'centerOfMass' to use bounding object based inertial properties, or insert a new
    //  'inertiaMatrix'."));
    return;
  }

  if (showInfo && mCenterOfMass->size() == 0)
    parsingInfo(tr("You must also specify the 'centerOfMass' when specifying the 'inertiaMatrix'"));

  if (showInfo && mMass->value() <= 0.0)
    parsingInfo(tr("You must also set the 'mass' to a positive value when specifying the 'inertiaMatrix'."));

  // The principal moment of inertia must be positive
  // to avoid assertion failure with debug version of ODE
  const OmVector3 &v0 = mInertiaMatrix->item(0);
  if (v0[0] <= 0.0 || v0[1] <= 0.0 || v0[2] <= 0.0) {
    mHasAvalidInertiaMatrix = false;
    if (showInfo)
      parsingWarn(tr("The first line of 'inertiaMatrix' (principal moments of inertia) must have only positive values."));
  }
  const OmVector3 &v1 = mInertiaMatrix->item(1);
  // ODE is gone: Sylvester's criterion on the symmetric 3x3 -- exactly
  // dIsPositiveDefinite's verdict (Cholesky succeeds iff all leading principal
  // minors are positive). This validation still gates CUSTOM_INERTIA_MATRIX
  // mode, which the Newton registration consumes.
  const double a = v0[0], b = v0[1], c = v0[2];   // diagonal
  const double d = v1[0], e = v1[1], f = v1[2];   // ixy, ixz, iyz
  const double minor1 = a;
  const double minor2 = a * b - d * d;
  const double minor3 = a * (b * c - f * f) - d * (d * c - f * e) + e * (d * f - b * e);
  if (!(minor1 > 0.0 && minor2 > 0.0 && minor3 > 0.0)) {
    mHasAvalidInertiaMatrix = false;
    if (showInfo)
      parsingWarn(tr("'inertiaMatrix' must be positive definite."));
  } else if (mCenterOfMass->size() == 0 && showInfo)
    parsingWarn(tr("'centerOfmass' must also be specified when using an inertia matrix."));
  else if (mMass->value() < 0.0 && showInfo)
    parsingWarn(tr("'mass' must be positive when using an inertia matrix."));
}

OmDamping *OmPhysics::damping() const {
  return static_cast<OmDamping *>(mDamping->value());
}

void OmPhysics::updateMode() {
  const bool massIsPositive = mMass->value() > 0.0;
  const int comSize = mCenterOfMass->size();

  checkInertiaMatrix(false);

  if (massIsPositive && comSize == 1 && mHasAvalidInertiaMatrix)
    mMode = CUSTOM_INERTIA_MATRIX;
  else if ((massIsPositive || mDensity->value() > 0.0) && mInertiaMatrix->size() == 0)
    mMode = BOUNDING_OBJECT_BASED;
  else
    mMode = INVALID;
}

void OmPhysics::updateDensity() {
  if (mSkipUpdate)
    return;

  checkDensity();
  updateMode();

  if (mMode == BOUNDING_OBJECT_BASED)
    emit massOrDensityChanged();
}

void OmPhysics::updateMass() {
  if (mSkipUpdate)
    return;

  checkMass();
  updateMode();

  if (mMode == CUSTOM_INERTIA_MATRIX)
    emit inertialPropertiesChanged();
  else if (mMode == BOUNDING_OBJECT_BASED)
    emit massOrDensityChanged();
}

void OmPhysics::updateCenterOfMass() {
  assert(mCenterOfMass->size() <= 1);

  if (mSkipUpdate)
    return;

  checkCenterOfMass();

  updateMode();

  if (mMode == CUSTOM_INERTIA_MATRIX)
    emit inertialPropertiesChanged();
  else if (mMode == BOUNDING_OBJECT_BASED)
    emit centerOfMassChanged();
}

void OmPhysics::updateInertiaMatrix() {
  if (mSkipUpdate)
    return;

  checkInertiaMatrix(true);

  updateMode();

  if (mMode == CUSTOM_INERTIA_MATRIX)
    emit inertialPropertiesChanged();
  else if (mMode == BOUNDING_OBJECT_BASED)
    emit modeSwitched();
}

void OmPhysics::updateDamping() {
  if (mSkipUpdate)
    return;

  if (damping())
    connect(damping(), &OmDamping::changed, this, &OmPhysics::dampingChanged, Qt::UniqueConnection);

  emit dampingChanged();
}

void OmPhysics::setMass(double mass, bool skipUpdate) {
  mSkipUpdate = skipUpdate;
  mMass->setValue(mass);
  mSkipUpdate = false;
}

void OmPhysics::setDensity(double density, bool skipUpdate) {
  mSkipUpdate = skipUpdate;
  mDensity->setValue(density);
  mSkipUpdate = false;
}

void OmPhysics::setCenterOfMass(double x, double y, double z, bool skipUpdate) {
  assert(mCenterOfMass->size() <= 1);
  mSkipUpdate = skipUpdate;
  const OmVector3 v(x, y, z);
  if (mCenterOfMass->size() == 0)
    mCenterOfMass->insertItem(0, v);
  else
    mCenterOfMass->setItem(0, v);
  mSkipUpdate = false;
}

void OmPhysics::setInertiaMatrix(double v0x, double v0y, double v0z, double v1x, double v1y, double v1z, bool skipUpdate) {
  assert(mInertiaMatrix->size() == 0);
  mSkipUpdate = skipUpdate;
  mInertiaMatrix->insertItem(0, OmVector3(v0x, v0y, v0z));
  mInertiaMatrix->insertItem(1, OmVector3(v1x, v1y, v1z));
  mSkipUpdate = false;
}

bool OmPhysics::exportNodeHeader(OmWriter &writer) const {
  if (writer.isUrdf())
    return true;
  return OmNode::exportNodeHeader(writer);
}
