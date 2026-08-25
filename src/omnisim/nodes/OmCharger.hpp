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

#ifndef OM_CHARGER_HPP
#define OM_CHARGER_HPP

#include <QtCore/QList>
#include "OmSolid.hpp"

class OmGroup;
class OmMFDouble;
class OmRobot;
class OmSFBool;
class OmSFColor;
class OmSFDouble;

struct VisualElement;

class OmCharger : public OmSolid {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmCharger(OmTokenizer *tokenizer = NULL);
  OmCharger(const OmCharger &other);
  explicit OmCharger(const OmNode &other);
  virtual ~OmCharger() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CHARGER; }
  void postFinalize() override;
  void prePhysicsStep(double) override;
  void checkContact(OmRobot *const r);
  void reset(const QString &id) override;
  void save(const QString &id) override;
  enum { CURRENT_ENERGY = 0, MAX_ENERGY = 1, ENERGY_UPLOAD_SPEED = 2 };

private:
  // user accessible fields
  OmMFDouble *mBattery;
  OmSFDouble *mRadius;
  OmSFColor *mEmissiveColor;
  OmSFBool *mGradual;

  // private fields
  const OmRobot *mParentRobot;
  OmRobot *mRobot;  // robot currently connected to the Charger
  bool mElementsUpdateRequired;
  QList<VisualElement *> mVisualElements;
  QMap<QString, double> mSavedEnergies;

  OmCharger &operator=(const OmCharger &);  // non copyable
  OmNode *clone() const override { return new OmCharger(*this); }
  void init();
  bool isAnyMaterialOrLightFound() const;
  void findMaterialsAndLights(const OmGroup *const g);
  void clearMaterialsAndLights();
  void updateMaterialsAndLights(double batteryRatio);

private slots:
  void updateElementsWhenRequired() { mElementsUpdateRequired = true; }
};

#endif
