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
//  OmRotationalMotor.cpp
//

#include "OmRotationalMotor.hpp"

#include "OmJoint.hpp"
#include "OmJointParameters.hpp"
#include "OmMathsUtilities.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmPropeller.hpp"
#include "OmSolid.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only

#include <cassert>

void OmRotationalMotor::init() {
  mMaxForceOrTorque = findSFDouble("maxTorque");
}

OmRotationalMotor::OmRotationalMotor(OmTokenizer *tokenizer) : OmMotor("RotationalMotor", tokenizer) {
  init();
}

OmRotationalMotor::OmRotationalMotor(const OmRotationalMotor &other) : OmMotor(other) {
  init();
}

OmRotationalMotor::OmRotationalMotor(const OmNode &other) : OmMotor(other) {
  init();
}

OmRotationalMotor::~OmRotationalMotor() {
}

void OmRotationalMotor::turnOffMotor() {
  // ⚠ TURNING A RotationalMotor OFF IS UNIMPLEMENTED. ODE did it by zeroing the
  // joint's motor velocity and dropping FMax to the parameters' staticFriction
  // (per joint type: hinge / hinge2 axis / ball AMotor axis). All of that went
  // with ODE, and it was already unreachable -- `joint()->jointID()` is
  // permanently NULL. Consequence: a motor switched off keeps whatever target
  // OmBasicJoint's per-tick Newton push last wrote. Not routed to Newton here on
  // purpose: that is a behaviour change, not a deletion.
}

double OmRotationalMotor::computeFeedback() const {
  const OmPropeller *propeller = dynamic_cast<OmPropeller *>(parentNode());
  if (propeller)
    // RotationalMotor usually returns only torque feedback,
    // but the propeller case is different since the motor produces both a torque and a force (thrust).
    // We therefore estimate the feedback by the sum of the force and torque,
    // so that the consumption computation takes both into account.
    return fabs(propeller->currentThrust()) + fabs(propeller->currentTorque());

  const OmJoint *j = joint();
  if (j == NULL) {  // function available for motorized joints only
    warn(tr("Feedback is available for motorized joints only"));
    return 0.0;
  }
  const dJointID jID = j->jointID();
  if (!jID)  // we need physics enabled to compute torque feedback
    return 0.0;

  return 0.0;  // unreachable (jID is always null now); the dJointFeedback math this replaced went with ODE
}
