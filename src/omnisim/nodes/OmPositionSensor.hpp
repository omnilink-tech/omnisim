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

//
//  OmPositionSensor.hpp
//

// Concrete class representing position sensors placed in mechanical joints

#ifndef OM_POSITION_SENSOR_HPP
#define OM_POSITION_SENSOR_HPP

#include "OmJointDevice.hpp"
#include "OmSensor.hpp"

class OmPositionSensor : public OmJointDevice {
  Q_OBJECT

public:
  virtual ~OmPositionSensor() override { delete mSensor; }
  explicit OmPositionSensor(const QString &modelName, OmTokenizer *tokenizer = NULL);
  explicit OmPositionSensor(OmTokenizer *tokenizer = NULL);
  OmPositionSensor(const OmPositionSensor &other);
  explicit OmPositionSensor(const OmNode &other);
  int nodeType() const override { return WB_NODE_POSITION_SENSOR; }

  // inherited from OmBaseNode
  void postFinalize() override;

  // inherited from OmDevice
  void writeConfigure(OmDataStream &stream) override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(OmDataStream &stream) override;
  bool refreshSensorIfNeeded() override;

  // inherited from OmJointDevice
  double position() const;

private:
  // user accessible field
  OmSFDouble *mResolution;
  OmSFDouble *mNoise;

  OmSensor *mSensor;
  double mValue;
  OmPositionSensor &operator=(const OmPositionSensor &);  // non copyable
  OmNode *clone() const override { return new OmPositionSensor(*this); }
  void init();

  WbDeviceTag *mRequestedDeviceTag;

private slots:
  void updateResolution();
  void updateNoise();
};

#endif
