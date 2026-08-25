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

#include "OmHingeJointParameters.hpp"

void OmHingeJointParameters::init(bool fromDeprecatedHinge2JointParameters) {
  mAnchor = findSFVector3("anchor");
  mSuspensionSpringConstant = findSFDouble("suspensionSpringConstant");
  mSuspensionDampingConstant = findSFDouble("suspensionDampingConstant");
  mSuspensionAxis = findSFVector3("suspensionAxis");
  mStopErp = findSFDouble("stopERP");
  mStopCfm = findSFDouble("stopCFM");

  // DEPRECATED, only for backward compatibility
  if (fromDeprecatedHinge2JointParameters) {
    if (mAxis) {  // if axis is set the suspension axis must be equal to it
      if (mSuspensionAxis)
        mSuspensionAxis->setValue(mAxis->value());
      else
        mSuspensionAxis = new OmSFVector3(mAxis->value());
    }
    parsingWarn(tr("'Hinge2JointParameters' is deprecated, please use 'HingeJointParameters' instead."));
  }
}

// Constructors

OmHingeJointParameters::OmHingeJointParameters(const QString &modelName, OmTokenizer *tokenizer) :
  OmJointParameters(modelName, tokenizer) {
  init();
}

OmHingeJointParameters::OmHingeJointParameters(OmTokenizer *tokenizer, bool fromDeprecatedHinge2JointParameters) :
  OmJointParameters("HingeJointParameters", tokenizer) {
  init(fromDeprecatedHinge2JointParameters);
}

OmHingeJointParameters::OmHingeJointParameters(const OmHingeJointParameters &other) : OmJointParameters(other) {
  init();
}

OmHingeJointParameters::OmHingeJointParameters(const OmNode &other, bool fromDeprecatedHinge2JointParameters) :
  OmJointParameters(other) {
  init(fromDeprecatedHinge2JointParameters);
}

OmHingeJointParameters::~OmHingeJointParameters() {
}

void OmHingeJointParameters::postFinalize() {
  OmJointParameters::postFinalize();

  connect(mAnchor, &OmSFVector3::changed, this, &OmHingeJointParameters::anchorChanged);
  connect(mSuspensionSpringConstant, &OmSFDouble::changed, this, &OmHingeJointParameters::updateSuspension);
  connect(mSuspensionDampingConstant, &OmSFDouble::changed, this, &OmHingeJointParameters::updateSuspension);
  connect(mSuspensionAxis, &OmSFVector3::changed, this, &OmHingeJointParameters::updateSuspension);
  connect(mStopErp, &OmSFDouble::changed, this, &OmHingeJointParameters::updateStopErp);
  connect(mStopCfm, &OmSFDouble::changed, this, &OmHingeJointParameters::updateStopCfm);
  updateStopErp();
  updateStopCfm();
}

void OmHingeJointParameters::updateAxis() {
  const OmVector3 &a = mAxis->value();
  if (a.isNull()) {
    parsingWarn(tr("'axis' must be non zero."));
    mAxis->setValue(1.0, 0.0, 0.0);
    return;
  }

  emit axisChanged();
}

void OmHingeJointParameters::updateSuspension() {
  const OmVector3 &a = mSuspensionAxis->value();
  if (a.isNull()) {
    parsingWarn(tr("'SuspensionAxis' must be non zero."));
    mSuspensionAxis->setValue(1.0, 0.0, 0.0);
    return;
  }

  emit suspensionChanged();
}

void OmHingeJointParameters::updateStopErp() {
  if (mStopErp->value() < 0.0 && mStopErp->value() != -1) {
    mStopErp->setValue(-1);
    parsingWarn(tr("'stopERP' must be greater or equal to zero or -1. Reverting to -1 (use global ERP)."));
    return;
  }

  emit stopErpChanged();
}

void OmHingeJointParameters::updateStopCfm() {
  if (mStopCfm->value() <= 0.0 && mStopCfm->value() != -1) {
    mStopCfm->setValue(-1);
    parsingWarn(tr("'stopCFM' must be greater than zero or -1. Reverting to -1 (use global CFM)."));
    return;
  }

  emit stopCfmChanged();
}
