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

#include "OmMaterial.hpp"

#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmWorld.hpp"

void OmMaterial::init() {
  mInitialEmissiveColor = OmRgb();

  mAmbientIntensity = findSFDouble("ambientIntensity");
  mDiffuseColor = findSFColor("diffuseColor");
  mEmissiveColor = findSFColor("emissiveColor");
  mShininess = findSFDouble("shininess");
  mSpecularColor = findSFColor("specularColor");
  mTransparency = findSFDouble("transparency");
}

OmMaterial::OmMaterial(OmTokenizer *tokenizer) : OmBaseNode("Material", tokenizer) {
  init();
}

OmMaterial::OmMaterial(const OmMaterial &other) : OmBaseNode(other) {
  init();
}

OmMaterial::OmMaterial(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmMaterial::~OmMaterial() {
}

void OmMaterial::preFinalize() {
  OmBaseNode::preFinalize();

  updateAmbientIntensity();
  updateDiffuseColor();
  updateEmissiveColor();
  updateShininess();
  updateSpecularColor();
  updateTransparency();

  mInitialEmissiveColor = mEmissiveColor->value();
}

void OmMaterial::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mAmbientIntensity, &OmSFDouble::changed, this, &OmMaterial::updateAmbientIntensity);
  connect(mDiffuseColor, &OmSFColor::changed, this, &OmMaterial::updateDiffuseColor);
  connect(mEmissiveColor, &OmSFColor::changed, this, &OmMaterial::updateEmissiveColor);
  connect(mShininess, &OmSFDouble::changed, this, &OmMaterial::updateShininess);
  connect(mSpecularColor, &OmSFColor::changed, this, &OmMaterial::updateSpecularColor);
  connect(mTransparency, &OmSFDouble::changed, this, &OmMaterial::updateTransparency);

  if (!OmWorld::instance()->isLoading())
    emit changed();
}

float OmMaterial::transparency() const {
  return mTransparency->value();
}

const OmRgb &OmMaterial::emissiveColor() const {
  return mEmissiveColor->value();
}

void OmMaterial::setEmissiveColor(const OmRgb &color) {
  mEmissiveColor->setValue(color);
}

const OmRgb &OmMaterial::diffuseColor() const {
  return mDiffuseColor->value();
}

double OmMaterial::ambientIntensity() const {
  return mAmbientIntensity->value();
}

void OmMaterial::updateAmbientIntensity() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mAmbientIntensity, 0.0, 1.0, 0.5))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmMaterial::updateDiffuseColor() {
  if (OmFieldChecker::resetColorIfInvalid(this, mDiffuseColor))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmMaterial::updateEmissiveColor() {
  if (OmFieldChecker::resetColorIfInvalid(this, mEmissiveColor))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmMaterial::updateShininess() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mShininess, 0.0, 1.0, 0.5))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmMaterial::updateSpecularColor() {
  if (OmFieldChecker::resetColorIfInvalid(this, mSpecularColor))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmMaterial::updateTransparency() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransparency, 0.0, 1.0, 0.5))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

QStringList OmMaterial::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "ambientIntensity"
         << "shininess"
         << "specularColor"
         << "transparency"
         << "emissiveColor"
         << "diffuseColor";
  return fields;
}
