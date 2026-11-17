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

#include "OmPointLight.hpp"

#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmMFColor.hpp"
#include "OmPose.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFVector3.hpp"

void OmPointLight::init() {
  mAttenuation = findSFVector3("attenuation");
  mLocation = findSFVector3("location");
  mRadius = findSFDouble("radius");

  mSavedLocation[stateId()] = mLocation->value();
}

OmPointLight::OmPointLight(OmTokenizer *tokenizer) : OmLight("PointLight", tokenizer) {
  init();
  if (tokenizer == NULL) {
    mLocation->setYnoSignal(0.3);
    mAttenuation->setValueNoSignal(0.0, 0.0, 1.0);
  }
}

OmPointLight::OmPointLight(const OmPointLight &other) : OmLight(other) {
  init();
}

OmPointLight::OmPointLight(const OmNode &other) : OmLight(other) {
  init();
}

OmPointLight::~OmPointLight() {
  // D1.4: nothing to release since the WREN light object and its representation were excised.
}

void OmPointLight::reset(const QString &id) {
  OmLight::reset(id);
  mLocation->setValue(mSavedLocation[id]);
}

void OmPointLight::save(const QString &id) {
  OmLight::save(id);
  mSavedLocation[id] = mLocation->value();
}

void OmPointLight::preFinalize() {
  OmLight::preFinalize();

  updateAttenuation();
  updateLocation();
  updateRadius();
}

void OmPointLight::postFinalize() {
  OmLight::postFinalize();

  connect(mAttenuation, &OmSFVector3::changed, this, &OmPointLight::updateAttenuation);
  connect(mLocation, &OmSFVector3::changed, this, &OmPointLight::updateLocation);
  connect(mRadius, &OmSFDouble::changed, this, &OmPointLight::updateRadius);
}

OmVector3 OmPointLight::computeAbsoluteLocation() const {
  OmVector3 location = mLocation->value();
  const OmPose *const up = upperPose();
  if (up)
    location = up->matrix() * location;
  return location;
}

const OmVector3 &OmPointLight::attenuation() const {
  return mAttenuation->value();
}

double OmPointLight::radius() const {
  return mRadius->value();
}

void OmPointLight::updateAttenuation() {
  if (OmFieldChecker::resetVector3IfNegative(this, mAttenuation, OmVector3()))
    return;

  if (mAttenuation->value().x() > 0.0 || mAttenuation->value().y() > 0.0)
    parsingWarn(tr("A quadratic 'attenuation' should be preferred to have a realistic simulation of light. "
                   "Only the third component of the 'attenuation' field should be greater than 0."));

  checkAmbientAndAttenuationExclusivity();
}

void OmPointLight::updateLocation() {
  emit locationChanged();
}

void OmPointLight::updateRadius() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mRadius, 0.0))
    return;
}

void OmPointLight::updateAmbientIntensity() {
  checkAmbientAndAttenuationExclusivity();

  OmLight::updateAmbientIntensity();
}

void OmPointLight::updateIntensity() {
  OmLight::updateIntensity();
}

void OmPointLight::checkAmbientAndAttenuationExclusivity() {
  if (mAttenuation->value() != OmVector3(1.0, 0.0, 0.0) && ambientIntensity() != 0.0) {
    parsingWarn(
      tr("'ambientIntensity' and 'attenuation' cannot differ from their default values at the same time. 'ambientIntensity' "
         "was changed to 0."));
    setAmbientIntensity(0.0);
  }
}

double OmPointLight::computeAttenuation(double distance) const {
  return 1.0 / (mAttenuation->x() + mAttenuation->y() * distance + mAttenuation->z() * distance * distance);
}

QStringList OmPointLight::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "attenuation"
         << "location"
         << "radius" << OmLight::fieldsToSynchronizeWithW3d();
  return fields;
}
