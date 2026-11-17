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
//  WbballJointParameters.hpp
//

#ifndef OM_BALL_JOINT_PARAMETERS_HPP
#define OM_BALL_JOINT_PARAMETERS_HPP

// Alias class for instantiation OmBallJoint's anchor parameter

#include "OmJointParameters.hpp"
#include "OmSFDouble.hpp"

class OmBallJointParameters : public OmJointParameters {
  Q_OBJECT

public:
  virtual ~OmBallJointParameters() override;
  OmBallJointParameters(const QString &modelName, OmTokenizer *tokenizer);
  explicit OmBallJointParameters(OmTokenizer *tokenizer = NULL);
  OmBallJointParameters(const OmBallJointParameters &other);
  explicit OmBallJointParameters(const OmNode &other);

  int nodeType() const override { return WB_NODE_BALL_JOINT_PARAMETERS; }
  void postFinalize() override;

  virtual const OmVector3 &anchor() const { return mAnchor->value(); }

signals:
  void anchorChanged();

private:
  OmBallJointParameters &operator=(const OmBallJointParameters &);  // non copyable
  OmNode *clone() const override { return new OmBallJointParameters(*this); }
  void init();

  // fields
  OmSFVector3 *mAnchor;
};

#endif
