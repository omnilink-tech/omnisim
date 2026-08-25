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

#ifndef OM_HINGE_JOINT_PARAMETERS_HPP
#define OM_HINGE_JOINT_PARAMETERS_HPP

#include "OmJointParameters.hpp"
#include "OmSFVector3.hpp"

class OmHingeJointParameters : public OmJointParameters {
  Q_OBJECT

public:
  explicit OmHingeJointParameters(const QString &modelName, OmTokenizer *tokenizer = NULL);
  explicit OmHingeJointParameters(OmTokenizer *tokenizer = NULL, bool fromDeprecatedHinge2JointParameters = false);
  OmHingeJointParameters(const OmHingeJointParameters &other);
  explicit OmHingeJointParameters(const OmNode &other, bool fromDeprecatedHinge2JointParameters = false);
  virtual ~OmHingeJointParameters() override;

  int nodeType() const override { return WB_NODE_HINGE_JOINT_PARAMETERS; }
  void postFinalize() override;

  double suspensionSpringConstant() const { return mSuspensionSpringConstant->value(); }
  double suspensionDampingConstant() const { return mSuspensionDampingConstant->value(); }
  const OmVector3 &suspensionAxis() const { return mSuspensionAxis->value(); }

  virtual const OmVector3 &anchor() const { return mAnchor->value(); }

  double stopErp() const { return mStopErp->value(); }
  double stopCfm() const { return mStopCfm->value(); }

signals:
  void anchorChanged();
  void suspensionChanged();
  void stopErpChanged();
  void stopCfmChanged();

private:
  OmHingeJointParameters &operator=(const OmHingeJointParameters &);  // non copyable
  OmNode *clone() const override { return new OmHingeJointParameters(*this); }
  void init(bool fromDeprecatedHinge2JointParameters = false);

  // fields
  OmSFVector3 *mAnchor;
  OmSFDouble *mSuspensionSpringConstant;
  OmSFDouble *mSuspensionDampingConstant;
  OmSFVector3 *mSuspensionAxis;
  OmSFDouble *mStopErp;
  OmSFDouble *mStopCfm;

private slots:
  void updateSuspension();
  void updateAxis() override;
  void updateStopErp();
  void updateStopCfm();
};

#endif
