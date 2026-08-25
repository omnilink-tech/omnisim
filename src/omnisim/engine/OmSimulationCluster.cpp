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

#include "OmSimulationCluster.hpp"

// ODE HAS BEEN DELETED, so there is no world, space or contact pipeline behind
// this class. Every verb is a no-op stub, kept so OmSimulationWorld's step loop
// links and runs unchanged -- stepping happens in OmNewtonBackend, not here.
// Kinematic collision, contact sounds and the contact-point GUI are simply
// absent until their Newton-native replacements land. The whole class is a
// deletion candidate once those land and its last caller goes.

#include <QtCore/QMutex>

const long long int OmSimulationCluster::WEBOTS_MAGIC_NUMBER = 0x7765626F7473LL;

QMutex *OmSimulationCluster::cJointCreationMutex = NULL;

OmSimulationCluster::OmSimulationCluster(OmOdeContext *context) : mContext(context), mSwapJointContactBuffer(false) {
}

OmSimulationCluster::~OmSimulationCluster() {
}

void OmSimulationCluster::handleInitialCollisions() {
}

void OmSimulationCluster::swapBuffer() {
}

void OmSimulationCluster::step() {
}

dSpaceID OmSimulationCluster::space() const {
  return NULL;
}

dWorldID OmSimulationCluster::world() const {
  return NULL;
}

dImmersionLinkGroupID OmSimulationCluster::immersionLinkGroup() const {
  return NULL;
}

dJointGroupID OmSimulationCluster::bodyContactJointGroup(dBodyID) {
  return NULL;
}

void OmSimulationCluster::handleKinematicsCollisions() {
}

void OmSimulationCluster::appendCollisionedRobot(OmKinematicDifferentialWheels *) {
}

void OmSimulationCluster::handleCollisionIfSpace(void *, dGeomID, dGeomID) {
}

const OmContactProperties *OmSimulationCluster::findContactProperties(const OmSolid *, const OmSolid *) {
  return NULL;
}

void OmSimulationCluster::odeNearCallback(void *, dGeomID, dGeomID) {
}

void OmSimulationCluster::odeSensorRaysUpdate(int) {
}

void OmSimulationCluster::warnMoreContactPointsThanContactJoints(const QString &, const QString &, int, int) {
}
