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

// Concrete class representing a brake placed in mechanical joints

#ifndef OM_BRAKE_HPP
#define OM_BRAKE_HPP

#include "OmJointDevice.hpp"

class OmBrake : public OmJointDevice {
  Q_OBJECT

public:
  virtual ~OmBrake() override {}
  explicit OmBrake(const QString &modelName, OmTokenizer *tokenizer = NULL);
  explicit OmBrake(OmTokenizer *tokenizer = NULL);
  OmBrake(const OmBrake &other);
  explicit OmBrake(const OmNode &other);
  int nodeType() const override { return WB_NODE_BRAKE; }

  double getBrakingDampingConstant() const { return mBrakingDampingConstant; }

  // inherited from OmBaseNode
  void reset(const QString &id) override;

  // inherited from OmDevice
  void writeConfigure(OmDataStream &stream) override;
  void writeAnswer(OmDataStream &stream) override;
  void handleMessage(QDataStream &stream) override;

signals:
  // emitted when received command from controller
  void brakingChanged();

private:
  OmBrake &operator=(const OmBrake &);  // non copyable
  OmNode *clone() const override { return new OmBrake(*this); }
  void init();
  double mBrakingDampingConstant;
  WbDeviceTag *mRequestedDeviceTag;
};

#endif
