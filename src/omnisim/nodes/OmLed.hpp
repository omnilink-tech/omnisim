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

#ifndef OM_LED_HPP
#define OM_LED_HPP

#include "OmMFColor.hpp"
#include "OmSolidDevice.hpp"

class OmSFBool;
class OmRgb;
class OmLight;
class OmMaterial;
class OmPbrAppearance;
class OmGroup;

class QDataStream;

class OmLed : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmLed(OmTokenizer *tokenizer = NULL);
  OmLed(const OmLed &other);
  explicit OmLed(const OmNode &other);
  virtual ~OmLed() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_LED; }
  void postFinalize() override;
  void reset(const QString &id) override;

  // field accessors
  int colorsCount() const { return mColor->size(); }
  const OmRgb &color(int index) const { return mColor->item(index); }
  int value() const { return mValue; }
  const QList<OmPbrAppearance *> &pbrAppearances() const { return mPbrAppearances; }
  bool isGradual() const;

  void setValue(int value);  // value=0: off; value>=1: on (select color[value-1])

  void powerOn(bool) override;
  void handleMessage(QDataStream &) override;

protected slots:
  void updateChildren() override;
  virtual void updateIfNeeded(OmField *);

private:
  // user accessible fields
  OmMFColor *mColor;
  OmSFBool *mGradual;

  // other fields
  int mValue;
  QList<OmMaterial *> mMaterials;
  QList<OmPbrAppearance *> mPbrAppearances;
  QList<OmLight *> mLights;

  OmLed &operator=(const OmLed &);  // non copyable
  OmNode *clone() const override { return new OmLed(*this); }
  void init();

  void findMaterialsAndLights(const OmGroup *const group);
  void clearMaterialsAndLights() {
    mMaterials.clear();
    mLights.clear();
    mPbrAppearances.clear();
  }
  bool isAnyMaterialOrLightFound();
  void setMaterialsAndLightsColor();
};

#endif
