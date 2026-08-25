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

#ifndef OM_COMPASS_HPP
#define OM_COMPASS_HPP

#include "OmSolidDevice.hpp"

class OmSensor;
class OmLookupTable;

class OmCompass : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmCompass(OmTokenizer *tokenizer = NULL);
  explicit OmCompass(const OmCompass &other);
  explicit OmCompass(const OmNode &other);
  virtual ~OmCompass() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_COMPASS; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  bool refreshSensorIfNeeded() override;

private:
  // user accessible fields
  OmMFVector3 *mLookupTable;
  OmSFBool *mXAxis, *mYAxis, *mZAxis;
  OmSFDouble *mResolution;

  // other stuff
  OmSensor *mSensor;
  OmLookupTable *mLut;
  double mValues[3];  // current sensor values according to lookup table
  bool mNeedToReconfigure;

  // private functions
  OmCompass &operator=(const OmCompass &);  // non copyable
  OmNode *clone() const override { return new OmCompass(*this); }
  void init();
  void computeValue();
  void addConfigure(OmDataStream &);

private slots:
  void updateLookupTable();
  void updateResolution();
};

#endif
