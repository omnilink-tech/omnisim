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

#include "OmOdeContext.hpp"

// ODE HAS BEEN DELETED: there is no world or space behind this context.
// OmSimulationWorld no longer constructs one (mOdeContext stays null); these
// stubs only keep the linker happy for the residual unconditional callers. All
// handles are null, all verbs are no-ops. OmSimulationWorld still constructs one
// unconditionally, so instance() is non-null and the ~13 remaining
// OmOdeContext::instance()->... call sites are safe no-ops rather than crashes --
// but they are all dead work. Deletion candidate once they go.

OmOdeContext *OmOdeContext::cOdeContext = NULL;

OmOdeContext::OmOdeContext() : QObject() {
  mWorld = NULL;
  mSpace = NULL;
  mBroadphaseType = "simple";
  mJointGroupCreationMutex = new QMutex();
  mImmersionLinkGroup1 = NULL;
  mImmersionLinkGroup2 = NULL;
  mBodyContactJointGroupList1.clear();
  mBodyContactJointGroupList2.clear();
  mNumberOfThreads = -1;
  cOdeContext = this;
}

OmOdeContext::~OmOdeContext() {
  delete mJointGroupCreationMutex;
  mJointGroupCreationMutex = NULL;
  cOdeContext = NULL;
}

void OmOdeContext::setNumberOfThreads(int n) {
  mNumberOfThreads = n;
}

void OmOdeContext::setGravity(double, double, double) {
}

void OmOdeContext::setErp(double) {
}

void OmOdeContext::setCfm(double) {
}

void OmOdeContext::setDamping(double, double) {
  emit worldDefaultDampingChanged();
}

void OmOdeContext::setPhysicsDisableTime(double) {
}

void OmOdeContext::gatherSleepingStats(int *totalBodies, int *enabled, int *staticColliders) const {
  if (totalBodies)
    *totalBodies = 0;
  if (enabled)
    *enabled = 0;
  if (staticColliders)
    *staticColliders = 0;
}

void OmOdeContext::setBroadphaseType(const char *) {
}

void OmOdeContext::emptyEnabledBodyContactJointGroups1() {
}

void OmOdeContext::emptyEnabledBodyContactJointGroups2() {
}

dJointGroupID OmOdeContext::bodyContactJointGroup1(dBodyID) {
  return NULL;
}

dJointGroupID OmOdeContext::bodyContactJointGroup2(dBodyID) {
  return NULL;
}

void OmOdeContext::removeBodyContactJointGroup(dBodyID) {
}
