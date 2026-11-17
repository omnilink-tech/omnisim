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

#include "OmLed.hpp"

#include "OmAppearance.hpp"
#include "OmField.hpp"
#include "OmGroup.hpp"
#include "OmLight.hpp"
#include "OmMFNode.hpp"
#include "OmMaterial.hpp"
#include "OmPbrAppearance.hpp"
#include "OmShape.hpp"

#include "../../controller/c/messages.h"  // contains the definitions for the macros C_SET_LED

#include <QtCore/QDataStream>
#include <cassert>

void OmLed::init() {
  mColor = findMFColor("color");
  mGradual = findSFBool("gradual");
  mValue = 0;
}

OmLed::OmLed(OmTokenizer *tokenizer) : OmSolidDevice("LED", tokenizer) {
  init();
}

OmLed::OmLed(const OmLed &other) : OmSolidDevice(other) {
  init();
}

OmLed::OmLed(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmLed::~OmLed() {
  clearMaterialsAndLights();
}

void OmLed::postFinalize() {
  OmSolidDevice::postFinalize();

  updateChildren();

  // disable WREN lights if not on
  bool on = mValue != 0;
  foreach (OmLight *light, mLights)
    light->toggleOn(on);
}

void OmLed::reset(const QString &id) {
  OmSolidDevice::reset(id);
  mValue = 0;
  foreach (OmLight *light, mLights)
    light->toggleOn(false);
  foreach (OmMaterial *material, mMaterials)
    material->setEmissiveColor(OmRgb(0.0, 0.0, 0.0));
  foreach (OmPbrAppearance *pbrAppearance, mPbrAppearances)
    pbrAppearance->setEmissiveColor(OmRgb(0.0, 0.0, 0.0));
}

void OmLed::updateIfNeeded(OmField *field) {
  if (field->name() == "material" || field->name() == "appearance")
    updateChildren();
}

void OmLed::updateChildren() {
  OmSolid::updateChildren();

  if (!isPostFinalizedCalled())
    return;

  findMaterialsAndLights(this);
  if (mGradual->isTrue() && colorsCount() > 1)
    parsingWarn(tr("Too many colors defined for a gradual LED."));

  // update color of lights and materials
  setMaterialsAndLightsColor();
}

void OmLed::findMaterialsAndLights(const OmGroup *group) {
  int size = group->children().size();
  if (size < 1) {
    clearMaterialsAndLights();
    return;
  }

  if (group == this) {
    clearMaterialsAndLights();
    size = 1;  // we look only into the first child of the OmLed node
  }

  for (int i = 0; i < size; ++i) {
    OmBaseNode *const n = group->child(i);
    // cppcheck-suppress constVariablePointer
    OmLight *lightChild = dynamic_cast<OmLight *>(n);
    OmGroup *groupChild = dynamic_cast<OmGroup *>(n);

    if (n->nodeType() == WB_NODE_SHAPE) {
      const OmAppearance *appearance = dynamic_cast<OmShape *>(n)->appearance();
      if (appearance) {
        // cppcheck-suppress constVariablePointer
        OmMaterial *material = appearance->material();
        if (material)
          mMaterials.append(material);

        connect(appearance, &OmAppearance::fieldChanged, this, &OmLed::updateIfNeeded, Qt::UniqueConnection);
        connect(appearance->parentNode(), &OmShape::fieldChanged, this, &OmLed::updateIfNeeded, Qt::UniqueConnection);
      } else {
        OmPbrAppearance *pbrAppearance = dynamic_cast<OmShape *>(n)->pbrAppearance();
        if (pbrAppearance) {
          mPbrAppearances.append(pbrAppearance);

          connect(pbrAppearance, &OmPbrAppearance::fieldChanged, this, &OmLed::updateIfNeeded, Qt::UniqueConnection);
          connect(pbrAppearance->parentNode(), &OmShape::fieldChanged, this, &OmLed::updateIfNeeded, Qt::UniqueConnection);
        }
      }
    } else if (lightChild)
      mLights.append(lightChild);
    else if (groupChild) {
      findMaterialsAndLights(groupChild);
      connect(groupChild, &OmGroup::childrenChanged, this, &OmLed::updateChildren, Qt::UniqueConnection);
    }
  }

  if (group == this && !isAnyMaterialOrLightFound())
    parsingWarn(tr("No PBRAppearance, Material and no Light found. "
                   "The first child of a LED should be either a Shape, a Light "
                   "or a Group containing Shape and Light nodes."));
}

bool OmLed::isAnyMaterialOrLightFound() {
  return (mMaterials.size() > 0 || mLights.size() > 0 || mPbrAppearances.size() > 0);
}

void OmLed::handleMessage(QDataStream &stream) {
  unsigned char byte;
  int v;
  stream >> byte;

  switch (byte) {
    case C_LED_SET:
      stream >> v;
      if (isAnyMaterialOrLightFound())
        setValue(v);
      break;
    default:
      assert(0);
  }
}

bool OmLed::isGradual() const {
  return mGradual->value();
}

void OmLed::setValue(int value) {
  if (mValue == value)
    return;
  mValue = value;

  setMaterialsAndLightsColor();
}

void OmLed::setMaterialsAndLightsColor() {
  // compute the new color
  float r, g, b;
  if (mGradual->isTrue()) {
    if (mColor->isEmpty()) {  // RGB case
      r = (float)((mValue >> 16) & 0xff) / 255.0f;
      g = (float)((mValue >> 8) & 0xff) / 255.0f;
      b = (float)(mValue & 0xff) / 255.0f;
    } else {  // monochrome case
      float i = (float)mValue / 255.0f;
      const OmRgb &c = mColor->item(0);
      r = i * c.red();
      g = i * c.green();
      b = i * c.blue();
    }
  } else {
    if (mValue > 0) {
      if (mValue - 1 < mColor->size()) {
        const OmRgb &c = mColor->item(mValue - 1);
        r = c.red();
        g = c.green();
        b = c.blue();
      } else {
        r = 0.0f;
        g = 0.0f;
        b = 0.0f;
      }
    } else {
      r = 0.0f;
      g = 0.0f;
      b = 0.0f;
    }
  }

  assert(r >= 0 && r <= 1 && g >= 0 && g <= 1 && b >= 0 && b <= 1);
  OmRgb lightColor(r, g, b);

  // update every material
  foreach (OmMaterial *material, mMaterials)
    material->setEmissiveColor(lightColor);

  // same for PbrAppearances
  foreach (OmPbrAppearance *pbrAppearance, mPbrAppearances)
    pbrAppearance->setEmissiveColor(lightColor);

  // update every lights
  const bool on = mValue != 0;
  foreach (OmLight *light, mLights) {
    light->setColor(lightColor);
    // disable WREN lights if not on
    light->toggleOn(on);
  }
}

void OmLed::powerOn(bool e) {  // turn off
  OmDevice::powerOn(e);
  if (!e)
    setValue(0);
}
