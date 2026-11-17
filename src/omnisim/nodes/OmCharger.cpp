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

#include "OmCharger.hpp"

#include "OmAppearance.hpp"
#include "OmField.hpp"
#include "OmLight.hpp"
#include "OmLog.hpp"
#include "OmMFDouble.hpp"
#include "OmMFNode.hpp"
#include "OmMaterial.hpp"
#include "OmPbrAppearance.hpp"
#include "OmRobot.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmShape.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"

struct VisualElement {
  VisualElement(OmBaseNode *n, double r, double g, double b) {
    node = n;
    initialRed = r;
    initialGreen = g;
    initialBlue = b;
  }
  OmBaseNode *node;
  double initialRed;
  double initialGreen;
  double initialBlue;
};

void OmCharger::init() {
  mBattery = findMFDouble("battery");
  mRadius = findSFDouble("radius");
  mEmissiveColor = findSFColor("emissiveColor");
  mGradual = findSFBool("gradual");
  mParentRobot = NULL;
  mRobot = NULL;
  mElementsUpdateRequired = true;
  if (mBattery->size() > CURRENT_ENERGY)
    mSavedEnergies[stateId()] = mBattery->item(CURRENT_ENERGY);
}

OmCharger::OmCharger(OmTokenizer *tokenizer) : OmSolid("Charger", tokenizer) {
  init();
}

OmCharger::OmCharger(const OmCharger &other) : OmSolid(other) {
  init();
}

OmCharger::OmCharger(const OmNode &other) : OmSolid(other) {
  init();
}

OmCharger::~OmCharger() {
}

void OmCharger::postFinalize() {
  OmSolid::postFinalize();

  const OmNode *topNode = OmVrmlNodeUtilities::findTopNode(this);
  mParentRobot = dynamic_cast<const OmRobot *>(topNode);
}

void OmCharger::clearMaterialsAndLights() {
  foreach (VisualElement *visualElement, mVisualElements)
    delete visualElement;
  mVisualElements.clear();
}

void OmCharger::updateMaterialsAndLights(double batteryRatio) {
  foreach (VisualElement *const visualElement, mVisualElements) {
    // compute the color of the indicators
    float cr, cg, cb;
    if (batteryRatio == 1.0) {
      cr = mEmissiveColor->red();
      cg = mEmissiveColor->green();
      cb = mEmissiveColor->blue();
    } else if (mGradual->value()) {
      cr = (mEmissiveColor->red() - visualElement->initialRed) * batteryRatio + visualElement->initialRed;
      cg = (mEmissiveColor->green() - visualElement->initialGreen) * batteryRatio + visualElement->initialGreen;
      cb = (mEmissiveColor->blue() - visualElement->initialBlue) * batteryRatio + visualElement->initialBlue;
    } else {
      cr = visualElement->initialRed;
      cg = visualElement->initialGreen;
      cb = visualElement->initialBlue;
    }
    OmMaterial *material = dynamic_cast<OmMaterial *>(visualElement->node);
    OmPbrAppearance *appearance = dynamic_cast<OmPbrAppearance *>(visualElement->node);
    OmLight *light = dynamic_cast<OmLight *>(visualElement->node);
    const OmRgb color(cr, cg, cb);
#ifndef NDEBUG
    const bool clampNeeded = OmRgb(cr, cg, cb).clampValuesIfNeeded();
    assert(!clampNeeded);
#endif
    if (material)
      material->setEmissiveColor(color);
    else if (appearance)
      appearance->setEmissiveColor(color);
    else if (light)
      light->setColor(color);
  }
}

bool OmCharger::isAnyMaterialOrLightFound() const {
  return mVisualElements.size() > 0;
}

void OmCharger::findMaterialsAndLights(const OmGroup *const g) {
  int size = g->children().size();
  if (size < 1) {
    clearMaterialsAndLights();
    return;
  }

  if (g == this) {
    clearMaterialsAndLights();
    size = 1;  // we look only into the first child of the OmCharger node
  }

  for (int i = 0; i < size; ++i) {
    OmBaseNode *const n = g->child(i);
    const OmShape *const shape = dynamic_cast<OmShape *>(n);
    OmLight *const light = dynamic_cast<OmLight *>(n);
    const OmGroup *const group = dynamic_cast<OmGroup *>(n);
    if (shape) {
      const OmAppearance *const appearance = shape->appearance();
      OmPbrAppearance *const pbrAppearance = shape->pbrAppearance();
      if (appearance) {
        connect(appearance, &OmAppearance::destroyed, this, &OmCharger::updateElementsWhenRequired, Qt::UniqueConnection);
        connect(appearance, &OmAppearance::changed, this, &OmCharger::updateElementsWhenRequired, Qt::UniqueConnection);
        OmMaterial *const material = appearance->material();
        if (material) {
          connect(material, &OmMaterial::destroyed, this, &OmCharger::updateElementsWhenRequired, Qt::UniqueConnection);
          mVisualElements.append(new VisualElement(material, material->initialEmissiveColor().red(),
                                                   material->initialEmissiveColor().green(),
                                                   material->initialEmissiveColor().blue()));
        }
      } else if (pbrAppearance) {
        connect(pbrAppearance, &OmMaterial::destroyed, this, &OmCharger::updateElementsWhenRequired, Qt::UniqueConnection);
        mVisualElements.append(new VisualElement(pbrAppearance, pbrAppearance->initialEmissiveColor().red(),
                                                 pbrAppearance->initialEmissiveColor().green(),
                                                 pbrAppearance->initialEmissiveColor().blue()));
      }
    } else if (light) {
      connect(light, &OmLight::destroyed, this, &OmCharger::updateElementsWhenRequired, Qt::UniqueConnection);
      mVisualElements.append(
        new VisualElement(light, light->initialColor().red(), light->initialColor().green(), light->initialColor().blue()));
    } else if (group) {
      connect(group, &OmGroup::childAdded, this, &OmCharger::updateElementsWhenRequired, Qt::UniqueConnection);
      connect(group, &OmGroup::destroyed, this, &OmCharger::updateElementsWhenRequired, Qt::UniqueConnection);
      findMaterialsAndLights(group);
    }
  }
  if (g == this && !isAnyMaterialOrLightFound()) {
    parsingWarn(tr("No Material and no Light found. "
                   "The first child of a Charger should be either a Shape, a Light "
                   "or a Group containing Shape and Light nodes."));
  }
}

void OmCharger::prePhysicsStep(double ms) {
  OmSolid::prePhysicsStep(ms);

  if (mBattery->size() < 3)
    return;

  if (mElementsUpdateRequired)
    findMaterialsAndLights(this);

  const QList<OmRobot *> &robots = OmWorld::instance()->robots();
  foreach (OmRobot *const robot, robots)
    checkContact(robot);

  const double currentEnergy = mBattery->item(CURRENT_ENERGY);
  const double maxEnergy = mBattery->item(MAX_ENERGY);
  const double energyUploadSpeed = mBattery->item(ENERGY_UPLOAD_SPEED);
  double newEnergy = currentEnergy;

  // The Charger collects energy from the Nature (Sun, Earth, Water, etc.)
  if (mRobot == NULL || mRobot->battery().size() < 3) {
    newEnergy += energyUploadSpeed * ms / 1000;
    if (newEnergy > maxEnergy)
      newEnergy = maxEnergy;
  } else {  // exchange of energy from the Charger to the Robot
    double e = (mRobot->energyUploadSpeed() * ms) / 1000.0;
    if (e > currentEnergy) {  // robot cannot take more than available
      e = currentEnergy;
      newEnergy = 0.0;
    } else
      newEnergy = currentEnergy - e;  // transfer

    double robotCurrentEnergy = mRobot->currentEnergy();
    // special case:
    //   if the current energy of the robot is already bigger that its max energy
    //   the robot battery cannot be filled
    if (robotCurrentEnergy >= mRobot->maxEnergy())
      newEnergy = currentEnergy;  // no energy is transferred
    else {
      robotCurrentEnergy += e;
      if (robotCurrentEnergy > mRobot->maxEnergy())
        robotCurrentEnergy = mRobot->maxEnergy();
    }
    mRobot->setCurrentEnergy(robotCurrentEnergy);
  }

  // store value in battery
  mBattery->setItem(CURRENT_ENERGY, newEnergy);

  // energy level
  const double r = currentEnergy / maxEnergy;
  updateMaterialsAndLights(r);
}

void OmCharger::checkContact(OmRobot *const r) {
  if (mParentRobot && mParentRobot == r)
    // do not charge itself
    return;

  if (mRobot && mRobot != r)
    return;  // Charger is already busy

  const OmVector3 &dist = matrix().translation() - r->translation();
  const double norm2 = dist.length2();
  double r2 = mRadius->value();
  if (mRobot)
    r2 *= 1.1;  // tolerance to maintain contact
  r2 *= r2;
  // printf("range^2: %g <= %g\n", norm2, r2);
  if (norm2 > r2)
    // current robot is leaving
    mRobot = NULL;
  else if (mRobot == NULL) {
    mRobot = r;
    // now Charger is busy with that robot...
    // printf("found one robot %p\n", (void *)robot);
  }
}

void OmCharger::reset(const QString &id) {
  OmSolid::reset(id);
  mRobot = NULL;
  if (mBattery->size() > CURRENT_ENERGY)
    mBattery->setItem(CURRENT_ENERGY, mSavedEnergies[id]);
  if (mBattery->size() > MAX_ENERGY)
    updateMaterialsAndLights(mBattery->item(CURRENT_ENERGY) / mBattery->item(MAX_ENERGY));
}

void OmCharger::save(const QString &id) {
  OmSolid::save(id);
  if (mBattery->size() > CURRENT_ENERGY)
    mSavedEnergies[id] = mBattery->item(CURRENT_ENERGY);
}
