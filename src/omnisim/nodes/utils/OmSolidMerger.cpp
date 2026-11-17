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

#include "OmSolidMerger.hpp"

// The merge LEADER bookkeeping survives ODE: OmSolid::effectiveNewtonBodyIndex
// and the Newton flush use solid()/isSet() to fold joint-free children into
// their leader body. The mass-merge verbs are empty (OmInertia is the native
// mass path) and are kept only because their call sites in OmSolid still
// express the fold order; they are a deletion candidate.

#include "OmSolid.hpp"

OmSolidMerger::OmSolidMerger(OmSolid *solid) :
  mSolid(solid),
  mCenterOfMass(0.0, 0.0, 0.0),
  mBodyArtificiallyDisabled(false) {
}

OmSolidMerger::~OmSolidMerger() {
}

void OmSolidMerger::appendSolid(OmSolid *) {
}

void OmSolidMerger::removeSolid(OmSolid *) {
}

void OmSolidMerger::removeExtraSpace() {
}

void OmSolidMerger::mergeMass(OmSolid *const, bool) {
}

void OmSolidMerger::updateMasses() {
}

void OmSolidMerger::setGeomAndBodyPositions(bool, bool) {
}

void OmSolidMerger::setupOdeBody() {
}

void OmSolidMerger::setOdeDamping() {
}

void OmSolidMerger::setOdeAutoDisable() {
}

bool OmSolidMerger::isSet() const {
  return mSolid->mergerIsSet();
}

void OmSolidMerger::setBodyArtificiallyDisabled(bool disabled) {
  mBodyArtificiallyDisabled = disabled;
}
