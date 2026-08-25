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

#include "OmHiddenKinematicParameters.hpp"

#include "OmField.hpp"
#include "OmRotation.hpp"
#include "OmSFDouble.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector3.hpp"
#include "OmVector3.hpp"

#include <QtCore/QRegularExpression>

#include <assert.h>

using HiddenParameters = OmHiddenKinematicParameters::HiddenKinematicParameters;

HiddenParameters::HiddenKinematicParameters() :
  mTranslation(NULL),
  mRotation(NULL),
  mPositions(NULL),
  mLinearVelocity(NULL),
  mAngularVelocity(NULL),
  mTranslationIsCreated(false),
  mRotationIsCreated(false) {
}

HiddenParameters::HiddenKinematicParameters(const OmVector3 *t, const OmRotation *r, PositionMap *p, const OmVector3 *l,
                                            const OmVector3 *a) :
  mTranslation(t),
  mRotation(r),
  mPositions(p),
  mLinearVelocity(l),
  mAngularVelocity(a),
  mTranslationIsCreated(false),
  mRotationIsCreated(false) {
}

HiddenParameters::~HiddenKinematicParameters() {
  if (mPositions)
    qDeleteAll(*mPositions);
  delete mPositions;
  delete mAngularVelocity;
  delete mLinearVelocity;
  if (mTranslationIsCreated)
    delete mTranslation;
  if (mRotationIsCreated)
    delete mRotation;
}

const OmVector3 *HiddenParameters::translation() const { return mTranslation; }
const OmRotation *HiddenParameters::rotation() const { return mRotation; }
const OmHiddenKinematicParameters::PositionMap *HiddenParameters::positions() const { return mPositions; }
const OmVector3 *HiddenParameters::linearVelocity() const { return mLinearVelocity; }
const OmVector3 *HiddenParameters::angularVelocity() const { return mAngularVelocity; }

void HiddenParameters::createTranslation(double x, double y, double z) {
  delete mTranslation;
  mTranslation = new OmVector3(x, y, z);
  mTranslationIsCreated = true;
}

void HiddenParameters::createRotation(double x, double y, double z, double angle) {
  delete mRotation;
  mRotation = new OmRotation(x, y, z, angle);
  mRotationIsCreated = true;
}

void HiddenParameters::createLinearVelocity(double x, double y, double z) {
  delete mLinearVelocity;
  mLinearVelocity = new OmVector3(x, y, z);
}

void HiddenParameters::createAngularVelocity(double x, double y, double z) {
  delete mAngularVelocity;
  mAngularVelocity = new OmVector3(x, y, z);
}

void HiddenParameters::insertPositions(int index, OmVector3 *positions) {
  if (mPositions == NULL)
    mPositions = new PositionMap;
  mPositions->insert(index, positions);
}

OmVector3 *HiddenParameters::positions(int index) {
  return mPositions ? mPositions->value(index) : NULL;
}

void OmHiddenKinematicParameters::createHiddenKinematicParameter(
  const OmField *field, OmHiddenKinematicParameters::HiddenKinematicParametersMap &map) {
  // Extract solid and joint indices
  static const QRegularExpression rx1("(_\\d+)+$");  // looks for a substring of the form _7 or _13_1 at the end of the
                                                     // parameter name, e.g. as in rotation_7, position2_13_1
  const QString parameterName(field->name());
  const QString str1(rx1.match(parameterName).captured());
  const QStringList indices = str1.split('_', Qt::SkipEmptyParts);
  assert(indices.size() > 0);
  const int solidIndex = indices[0].toInt();
  HiddenKinematicParameters *const hkp = map.value(solidIndex, NULL);
  HiddenKinematicParameters *const data = (hkp == NULL) ? new HiddenKinematicParameters() : hkp;

  const OmSFVector3 *const sfvec3f = dynamic_cast<OmSFVector3 *>(field->value());
  if (sfvec3f) {
    const double x = sfvec3f->x();
    const double y = sfvec3f->y();
    const double z = sfvec3f->z();

    if (parameterName.startsWith("translation"))
      data->createTranslation(x, y, z);
    else if (parameterName.startsWith("linearVelocity"))
      data->createLinearVelocity(x, y, z);
    else if (parameterName.startsWith("angularVelocity"))
      data->createAngularVelocity(x, y, z);

  } else if (parameterName.startsWith("rotation")) {
    const OmSFRotation *const sfrotation = static_cast<OmSFRotation *>(field->value());
    data->createRotation(sfrotation->x(), sfrotation->y(), sfrotation->z(), sfrotation->angle());
  } else if (parameterName.startsWith("position")) {
    assert(indices.size() > 1);
    const int jointIndex = indices[1].toInt();
    const int j = (parameterName.at(8) == QChar('3')) ? 2 : (parameterName.at(8) == QChar('2')) ? 1 : 0;
    OmVector3 *v = data->positions(jointIndex);
    if (v == NULL)
      v = new OmVector3(NAN, NAN, NAN);
    const OmSFDouble *const sfdouble = static_cast<OmSFDouble *>(field->value());
    (*v)[j] = sfdouble->value();
    data->insertPositions(jointIndex, v);
  }

  if (hkp == NULL)
    map.insert(solidIndex, data);
}
