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

// ODE HAS BEEN DELETED, so there is no body/geom/space to merge. Newton-side
// code queries bodyMerger()/body() on OmSolid, which stay null; every merge verb
// is a no-op. dMass is an INCOMPLETE type now (OmInertia is the native mass
// replacement), so nothing here may allocate or delete one -- that is why the
// constructor assigns mOdeMass = NULL rather than `new dMass`. The whole class
// is a deletion candidate once its callers stop asking for a merger.

#include "OmSolid.hpp"

OmSolidMerger::OmSolidMerger(OmSolid *solid) :
  mSolid(solid),
  mSpace(NULL),
  mCenterOfMass(0.0, 0.0, 0.0),
  mBodyArtificiallyDisabled(false) {
  mOdeMass = NULL;
  mBody = NULL;
}

OmSolidMerger::~OmSolidMerger() {
  mMergedSolids.clear();
}

const QMap<OmSolid *, dMass *> &OmSolidMerger::mergedSolids() const {
  return mMergedSolids;
}

void OmSolidMerger::appendSolid(OmSolid *) {
}

void OmSolidMerger::removeSolid(OmSolid *) {
}

dSpaceID OmSolidMerger::reservedSpace() {
  return NULL;
}

void OmSolidMerger::removeExtraSpace() {
}

void OmSolidMerger::addGeomToSpace(dGeomID) {
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

void OmSolidMerger::attachGeomsToBody(dGeomID) {
}

bool OmSolidMerger::isSet() const {
  return mSolid->mergerIsSet();
}

void OmSolidMerger::setBodyArtificiallyDisabled(bool disabled) {
  mBodyArtificiallyDisabled = disabled;
}
