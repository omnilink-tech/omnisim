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

#ifndef OM_LINEAR_MOTOR_HPP
#define OM_LINEAR_MOTOR_HPP

#include "OmMotor.hpp"

class OmLinearMotor : public OmMotor {
  Q_OBJECT

public:
  explicit OmLinearMotor(OmTokenizer *tokenizer = NULL);
  OmLinearMotor(const OmLinearMotor &other);
  explicit OmLinearMotor(const OmNode &other);
  virtual ~OmLinearMotor() override;
  int nodeType() const override { return WB_NODE_LINEAR_MOTOR; }
  double force() const { return mMotorForceOrTorque; }
  double computeFeedback() const override;

protected:
  void turnOffMotor() override;

private:
  OmNode *clone() const override { return new OmLinearMotor(*this); }

  void init();
};

#endif
