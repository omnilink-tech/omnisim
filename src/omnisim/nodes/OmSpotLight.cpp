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

#include "OmSpotLight.hpp"

#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmMFColor.hpp"
#include "OmPose.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFVector3.hpp"

#include <cmath>

static const double gVrmlHalfLog = 0.5 * log(0.5);

void OmSpotLight::init() {
  mAttenuation = findSFVector3("attenuation");
  mLocation = findSFVector3("location");
  mRadius = findSFDouble("radius");
  mDirection = findSFVector3("direction");
  mCutOffAngle = findSFDouble("cutOffAngle");
  mBeamWidth = findSFDouble("beamWidth");
}

OmSpotLight::OmSpotLight(OmTokenizer *tokenizer) : OmLight("SpotLight", tokenizer) {
  init();
  if (tokenizer == NULL) {
    mDirection->setValueNoSignal(0, 1, -1);
    mAttenuation->setValueNoSignal(0.0, 0.0, 1.0);
    mBeamWidth->setValueNoSignal(0.7);
  }
}

OmSpotLight::OmSpotLight(const OmSpotLight &other) : OmLight(other) {
  init();
}

OmSpotLight::OmSpotLight(const OmNode &other) : OmLight(other) {
  init();
}

void OmSpotLight::preFinalize() {
  OmLight::preFinalize();

  updateAttenuation();
  updateLocation();
  updateRadius();
  updateDirection();
  updateCutOffAngle();
  updateBeamWidth();
}

void OmSpotLight::postFinalize() {
  OmLight::postFinalize();

  connect(mAttenuation, &OmSFVector3::changed, this, &OmSpotLight::updateAttenuation);
  connect(mLocation, &OmSFVector3::changed, this, &OmSpotLight::updateLocation);
  connect(mRadius, &OmSFDouble::changed, this, &OmSpotLight::updateRadius);
  connect(mDirection, &OmSFVector3::changed, this, &OmSpotLight::updateDirection);
  connect(mCutOffAngle, &OmSFDouble::changed, this, &OmSpotLight::updateCutOffAngle);
  connect(mBeamWidth, &OmSFDouble::changed, this, &OmSpotLight::updateBeamWidth);
}

OmSpotLight::~OmSpotLight() {
  // D1.4: nothing to release since the WREN light object and its representation were excised.
}

OmVector3 OmSpotLight::computeAbsoluteLocation() const {
  OmVector3 location = mLocation->value();
  const OmPose *const up = upperPose();
  if (up)
    location = up->matrix() * location;
  return location;
}

OmVector3 OmSpotLight::computeAbsoluteDirection() const {
  OmVector3 dir = mDirection->value();
  const OmPose *const up = upperPose();
  if (up)
    dir = up->matrix().sub3x3MatrixDot(dir);
  return dir;
}

const OmVector3 &OmSpotLight::attenuation() const {
  return mAttenuation->value();
}

double OmSpotLight::radius() const {
  return mRadius->value();
}

void OmSpotLight::updateAttenuation() {
  if (OmFieldChecker::resetVector3IfNegative(this, mAttenuation, OmVector3()))
    return;

  if (mAttenuation->value().x() > 0.0 || mAttenuation->value().y() > 0.0)
    parsingWarn(tr("A quadratic 'attenuation' should be preferred to have a realistic simulation of light. "
                   "Only the third component of the 'attenuation' field should be greater than 0."));

  checkAmbientAndAttenuationExclusivity();
}

void OmSpotLight::updateLocation() {
  emit locationChanged();
}

void OmSpotLight::updateRadius() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mRadius, 0.0))
    return;
}

void OmSpotLight::updateAmbientIntensity() {
  checkAmbientAndAttenuationExclusivity();

  OmLight::updateAmbientIntensity();
}

void OmSpotLight::updateIntensity() {
  OmLight::updateIntensity();
}

void OmSpotLight::updateColor() {
  OmLight::updateColor();
}

void OmSpotLight::updateDirection() {
  emit directionChanged();
}

void OmSpotLight::updateCutOffAngle() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mCutOffAngle, 0.0, M_PI_2, M_PI_2))
    return;

  if (mCutOffAngle->value() < mBeamWidth->value())
    mBeamWidth->setValueNoSignal(mCutOffAngle->value());
}

void OmSpotLight::updateBeamWidth() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mBeamWidth, 0.0))
    return;
  else if (mBeamWidth->value() > mCutOffAngle->value()) {
    parsingWarn(tr("Invalid 'beamWidth' changed to %1. The value should be less than or equal to 'cutOffAngle'.")
                  .arg(mCutOffAngle->value()));
    mBeamWidth->setValue(mCutOffAngle->value());
    return;
  }
}

void OmSpotLight::checkAmbientAndAttenuationExclusivity() {
  if (mAttenuation->value() != OmVector3(1.0, 0.0, 0.0) && ambientIntensity() != 0.0) {
    parsingWarn(
      tr("'ambientIntensity' and 'attenuation' cannot differ from their default values at the same time. 'ambientIntensity' "
         "was changed to 0."));
    setAmbientIntensity(0.0);
  }
}

double OmSpotLight::exponent() const {
  return gVrmlHalfLog / log(cos(mBeamWidth->value()));
}

const OmVector3 &OmSpotLight::direction() const {
  return mDirection->value();
}

double OmSpotLight::cutOffAngle() const {
  return mCutOffAngle->value();
}

double OmSpotLight::beamWidth() const {
  return mBeamWidth->value();
}

double OmSpotLight::computeAttenuation(double distance) const {
  return 1.0 / (mAttenuation->x() + mAttenuation->y() * distance + mAttenuation->z() * distance * distance);
}

QStringList OmSpotLight::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "attenuation"
         << "beamWidth"
         << "cutOffAngle"
         << "direction"
         << "location"
         << "radius" << OmLight::fieldsToSynchronizeWithW3d();
  return fields;
}
