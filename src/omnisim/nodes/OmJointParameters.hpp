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

#ifndef OM_JOINT_PARAMETERS_HPP
#define OM_JOINT_PARAMETERS_HPP

#include "OmBaseNode.hpp"
#include "OmSFDouble.hpp"
#include "OmSFVector3.hpp"

class OmJointParameters : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmJointParameters(const QString &modelName, OmTokenizer *tokenizer = NULL);
  explicit OmJointParameters(OmTokenizer *tokenizer = NULL);
  OmJointParameters(const OmJointParameters &other);
  explicit OmJointParameters(const OmNode &other);
  virtual ~OmJointParameters() override;

  int nodeType() const override { return WB_NODE_JOINT_PARAMETERS; }
  void preFinalize() override;
  void postFinalize() override;

  double position() const { return mPosition->value(); }
  double maxStop() const { return mMaxStop->value(); }
  double minStop() const { return mMinStop->value(); }
  double springConstant() const { return mSpringConstant->value(); }
  double dampingConstant() const { return mDampingConstant->value(); }
  double staticFriction() const { return mStaticFriction->value(); }
  const OmVector3 axis() const { return mAxis ? mAxis->value() : OmVector3(); }

  void setPosition(double p) { mPosition->setValue(p); }
  void setPositionFromOde(double p) { mPosition->setValueFromOde(p); }

  bool clampPosition(double &p) const;

signals:
  void positionChanged();
  void minAndMaxStopChanged(double min, double max);
  void springAndDampingConstantsChanged();
  void axisChanged();

protected:
  OmSFVector3 *mAxis;  // axis default value redefined in a derived classes
  bool exportNodeHeader(OmWriter &writer) const override;

private:
  OmJointParameters &operator=(const OmJointParameters &);  // non copyable
  OmNode *clone() const override { return new OmJointParameters(*this); }
  void init();

  // fields
  OmSFDouble *mPosition;
  OmSFDouble *mMinStop;
  OmSFDouble *mMaxStop;
  OmSFDouble *mSpringConstant;
  OmSFDouble *mDampingConstant;
  OmSFDouble *mStaticFriction;

private slots:
  void updateMinAndMaxStop();
  void updateSpringConstant();
  void updateDampingConstant();
  void updateStaticFriction();
  virtual void updateAxis();
};

#endif
