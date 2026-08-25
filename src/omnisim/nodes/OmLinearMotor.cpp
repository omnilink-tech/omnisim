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

#include "OmLinearMotor.hpp"

#include "OmJoint.hpp"
#include "OmJointParameters.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmSolid.hpp"
#include "OmTrack.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only
#include <cassert>

void OmLinearMotor::init() {
  mMaxForceOrTorque = findSFDouble("maxForce");
}

OmLinearMotor::OmLinearMotor(OmTokenizer *tokenizer) : OmMotor("LinearMotor", tokenizer) {
  init();
}

OmLinearMotor::OmLinearMotor(const OmLinearMotor &other) : OmMotor(other) {
  init();
}

OmLinearMotor::OmLinearMotor(const OmNode &other) : OmMotor(other) {
  init();
}

OmLinearMotor::~OmLinearMotor() {
}

void OmLinearMotor::turnOffMotor() {
  // ⚠ TURNING A LinearMotor OFF IS UNIMPLEMENTED. ODE zeroed the slider's motor
  // velocity and dropped FMax to the parameters' staticFriction; that went with
  // ODE, and it was already unreachable -- `joint()->jointID()` is permanently
  // NULL. Consequence: a motor switched off keeps whatever target OmBasicJoint's
  // per-tick Newton push last wrote. See OmRotationalMotor::turnOffMotor.
}

double OmLinearMotor::computeFeedback() const {
  if (dynamic_cast<OmTrack *>(parentNode())) {
    warn(tr("Force feedback is not available for a LinearMotor node inside a Track node."));
    return 0.0;
  }

  const OmJoint *j = joint();
  if (j == NULL) {  // function available for motorized joints only
    warn(tr("Force feedback is available for motorized joints only"));
    return 0.0;
  }
  const dJointID jID = j->jointID();
  if (!jID)  // we need physics enabled to compute force feedback
    return 0.0;
  return 0.0;  // unreachable (jID is always null now); the dJointFeedback math this replaced went with ODE
}
