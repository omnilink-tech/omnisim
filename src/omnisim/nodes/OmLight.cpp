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

#include "OmLight.hpp"

#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmMFColor.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPerspective.hpp"
#include "OmPose.hpp"
#include "OmPreferences.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmWorld.hpp"
#include "OmWrenRenderingContext.hpp"

#include <QtCore/QList>

QList<const OmLight *> OmLight::cLights;

void OmLight::init() {
  mInitialColor = OmRgb();

  mAmbientIntensity = findSFDouble("ambientIntensity");
  mColor = findSFColor("color");
  mIntensity = findSFDouble("intensity");
  mOn = findSFBool("on");
  mCastShadows = findSFBool("castShadows");
  mCastLensFlares = findSFBool("castLensFlares");
}

OmLight::OmLight(const OmLight &other) : OmBaseNode(other) {
  init();
}

OmLight::OmLight(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmLight::OmLight(const QString &modelName, OmTokenizer *tokenizer) : OmBaseNode(modelName, tokenizer) {
  init();
}

void OmLight::preFinalize() {
  OmBaseNode::preFinalize();

  cLights << this;

  updateAmbientIntensity();
  updateColor();
  updateIntensity();
  updateOn();
  updateCastShadows();

  mInitialColor = mColor->value();
}

void OmLight::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mAmbientIntensity, &OmSFDouble::changed, this, &OmLight::updateAmbientIntensity);
  connect(mColor, &OmSFColor::changed, this, &OmLight::updateColor);
  connect(mIntensity, &OmSFDouble::changed, this, &OmLight::updateIntensity);
  connect(mOn, &OmSFBool::changed, this, &OmLight::updateOn);
  connect(mCastShadows, &OmSFBool::changed, this, &OmLight::updateCastShadows);

  if (!OmWorld::instance()->isLoading())
    emit OmWrenRenderingContext::instance()->numberOfOnLightsChanged();
}

OmLight::~OmLight() {
  cLights.removeOne(this);

  if (areWrenObjectsInitialized()) {
    if (!OmWorld::instance()->isCleaning())
      emit OmWrenRenderingContext::instance()->numberOfOnLightsChanged();
  }
}

bool OmLight::isOn() const {
  return mOn->value();
}

bool OmLight::castShadows() const {
  return mCastShadows->value();
}

bool OmLight::castLensFlares() const {
  return mCastLensFlares->value();
}

double OmLight::intensity() const {
  return mIntensity->value();
}

double OmLight::ambientIntensity() const {
  return mAmbientIntensity->value();
}

void OmLight::setAmbientIntensity(double value) {
  mAmbientIntensity->setValue(value);
}

const OmRgb &OmLight::color() const {
  return mColor->value();
}

void OmLight::setColor(const OmRgb &color) {
  mColor->setValue(color);
}

void OmLight::toggleOn(bool on) {
  mOn->setValue(on);
}

int OmLight::numberOfOnLights() {
  int n, i;
  for (i = 0, n = 0; i < cLights.count(); ++i) {
    if (cLights[i]->isOn())
      ++n;
  }

  return n;
}

void OmLight::createWrenObjects() {
  // D1.4: WREN light objects are gone; this hook now only latches non-render state.
  OmBaseNode::createWrenObjects();

  connect(OmPreferences::instance(), &OmPreferences::changedByUser, this, &OmLight::updateCastShadows);
}

void OmLight::updateAmbientIntensity() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mAmbientIntensity, 0.0, 1.0,
                                                                mAmbientIntensity->value() > 1.0 ? 1.0 : 0.0))
    return;
}

void OmLight::updateIntensity() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mIntensity, 1.0))
    return;

  emit intensityChanged();
}

void OmLight::updateOn() {
  if (areWrenObjectsInitialized()) {
    if (!OmWorld::instance()->isLoading())
      emit OmWrenRenderingContext::instance()->numberOfOnLightsChanged();
  }

  emit isOnChanged();
}

void OmLight::updateColor() {
  if (OmFieldChecker::resetColorIfInvalid(this, mColor))
    return;

  emit colorChanged();
}

void OmLight::updateCastShadows() {
  if (!OmWorld::instance()->isLoading() && numberOfLightsCastingShadows() < 2)
    emit OmWrenRenderingContext::instance()->shadowsStateChanged();
}

void OmLight::sceneAmbientLight(float rgb[3]) {
  rgb[0] = 0.0f;
  rgb[1] = 0.0f;
  rgb[2] = 0.0f;

  foreach (const OmLight *light, cLights) {
    if (light->isOn()) {
      rgb[0] += static_cast<float>(light->ambientIntensity() * light->color().red());
      rgb[1] += static_cast<float>(light->ambientIntensity() * light->color().green());
      rgb[2] += static_cast<float>(light->ambientIntensity() * light->color().blue());
    }
  }
}

int OmLight::numberOfLightsCastingShadows() {
  int counter = 0;
  foreach (const OmLight *light, cLights)
    if (light->isOn() && light->castShadows())
      counter++;
  return counter;
}

QStringList OmLight::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "color"
         << "on"
         << "intensity"
         << "ambientIntensity"
         << "castShadows";
  return fields;
}
