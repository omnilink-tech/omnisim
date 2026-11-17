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

#ifndef OM_INERTIAL_UNIT_HPP
#define OM_INERTIAL_UNIT_HPP

#include "OmSolidDevice.hpp"

class OmSensor;
class OmMFVector3;
class OmSFBool;

class OmInertialUnit : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmInertialUnit(OmTokenizer *tokenizer = NULL);
  OmInertialUnit(const OmInertialUnit &other);
  explicit OmInertialUnit(const OmNode &other);
  virtual ~OmInertialUnit() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_INERTIAL_UNIT; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  bool refreshSensorIfNeeded() override;

private:
  // user accessible fields
  OmSFBool *mXAxis, *mYAxis, *mZAxis;
  OmSFDouble *mResolution;
  OmSFDouble *mNoise;

  // other stuff
  OmSensor *mSensor;
  OmQuaternion mQuaternion;
  bool mNeedToReconfigure;

  // private functions
  OmInertialUnit &operator=(const OmInertialUnit &);  // non copyable
  OmNode *clone() const override { return new OmInertialUnit(*this); }
  void init();
  void computeValue();
  void addConfigure(OmDataStream &);

private slots:
  void updateResolution();
  void updateNoise();
};

#endif
