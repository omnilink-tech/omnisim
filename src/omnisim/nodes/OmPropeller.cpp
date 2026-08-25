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

#include "OmPropeller.hpp"

#include "OmPhysicsBackend.hpp"
#include "OmRobot.hpp"
#include "OmRotationalMotor.hpp"
#include "OmSFNode.hpp"
#include "OmSimulationState.hpp"
#include "OmSolid.hpp"
#include "OmSolidMerger.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only

void OmPropeller::init() {
  mShaftAxis = findSFVector3("shaftAxis");
  mCenterOfThrust = findSFVector3("centerOfThrust");
  mThrustConstants = findSFVector2("thrustConstants");
  mTorqueConstants = findSFVector2("torqueConstants");
  mFastHelixThreshold = findSFDouble("fastHelixThreshold");
  mDevice = findSFNode("device");
  mFastHelix = findSFNode("fastHelix");
  mSlowHelix = findSFNode("slowHelix");

  mPosition = 0.0;
  mHelixType = SLOW_HELIX;
  mHelix = NULL;

  mCurrentThrust = 0.0;
  mCurrentTorque = 0.0;
}

// Constructors

OmPropeller::OmPropeller(OmTokenizer *tokenizer) : OmBaseNode("Propeller", tokenizer) {
  init();
}

OmPropeller::OmPropeller(const OmPropeller &other) : OmBaseNode(other) {
  init();
}

OmPropeller::OmPropeller(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmPropeller::~OmPropeller() {
  // D1.4: WREN shaft-axis visuals gone; nothing to clean up.
}

void OmPropeller::downloadAssets() {
  OmBaseNode::downloadAssets();
  OmSolid *const fastHelix = helix(FAST_HELIX);
  OmSolid *const slowHelix = helix(SLOW_HELIX);
  if (fastHelix)
    fastHelix->downloadAssets();
  if (slowHelix)
    slowHelix->downloadAssets();
}

void OmPropeller::preFinalize() {
  OmBaseNode::preFinalize();

  OmLogicalDevice *const d = device();
  if (d && !d->isPreFinalizedCalled())
    d->preFinalize();

  OmBaseNode *h = static_cast<OmBaseNode *>(mFastHelix->value());
  if (h && !h->isPreFinalizedCalled())
    h->preFinalize();

  h = static_cast<OmBaseNode *>(mSlowHelix->value());
  if (h && !h->isPreFinalizedCalled())
    h->preFinalize();

  updateShaftAxis();
  updateDevice();
}

void OmPropeller::postFinalize() {
  OmBaseNode::postFinalize();

  OmLogicalDevice *const d = device();
  if (d && !d->isPostFinalizedCalled())
    d->postFinalize();

  OmBaseNode *h = static_cast<OmBaseNode *>(mFastHelix->value());
  if (h && !h->isPostFinalizedCalled())
    h->postFinalize();

  h = static_cast<OmBaseNode *>(mSlowHelix->value());
  if (h && !h->isPostFinalizedCalled())
    h->postFinalize();

  updateHelix(0.0);

  connect(mShaftAxis, &OmSFVector3::changed, this, &OmPropeller::updateShaftAxis);
  connect(mCenterOfThrust, &OmSFVector3::changed, this, &OmPropeller::updateShaftAxis);
  connect(mDevice, &OmSFNode::changed, this, &OmPropeller::updateDevice);
  connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this, &OmPropeller::updateHelixRepresentation);
}

void OmPropeller::createOdeObjects() {
  OmSolid *const fastHelix = helix(FAST_HELIX);
  OmSolid *const slowHelix = helix(SLOW_HELIX);

  if (fastHelix)
    fastHelix->createOdeObjects();
  if (slowHelix)
    slowHelix->createOdeObjects();
}

OmRotationalMotor *OmPropeller::motor() const {
  return dynamic_cast<OmRotationalMotor *>(mDevice->value());
}

OmLogicalDevice *OmPropeller::device() const {
  return dynamic_cast<OmLogicalDevice *>(mDevice->value());
}

// Update methods: they check validity and correct if necessary
void OmPropeller::updateHelix(double angularSpeed, bool ode) {
  const bool fast = fabs(angularSpeed) > mFastHelixThreshold->value();
  mHelixType = fast ? FAST_HELIX : SLOW_HELIX;
  OmSolid *const fastHelix = helix(FAST_HELIX);
  OmSolid *const slowHelix = helix(SLOW_HELIX);
  mHelix = fast ? fastHelix : slowHelix;
  OmSolid *const hiddenHelix = fast ? slowHelix : fastHelix;
  if (mHelix)
    mHelix->enable(true, ode);

  const OmSimulationState *const state = OmSimulationState::instance();
  const bool pause = state->isPaused();
  if (hiddenHelix)
    hiddenHelix->enable(pause && hiddenHelix->isSelected(), ode);
}

void OmPropeller::updateHelixRepresentation() {
  const OmRotationalMotor *const m = motor();
  const double speed = m ? m->currentVelocity() : 0.0;
  updateHelix(speed, false);
}

void OmPropeller::updateDevice() {
  if (mDevice->value()) {
    const OmSolid *const s = upperSolid();
    if (s) {
      OmRobot *const r = s->robot();
      assert(r);
      r->descendantNodeInserted(NULL);
    }
  }
}

void OmPropeller::updateShaftAxis() {
  static const OmVector3 DEFAULT_AXIS(0.0, 1.0, 0.0);
  if (mShaftAxis->value().isNull()) {
    parsingWarn(
      tr("'shaftAxis'cannot be zero. Defaults to %1 %2 %3").arg(DEFAULT_AXIS.x()).arg(DEFAULT_AXIS.y()).arg(DEFAULT_AXIS.z()));
    mShaftAxis->setValue(DEFAULT_AXIS);
  }

  mNormalizedAxis = mShaftAxis->value().normalized();
}

void OmPropeller::prePhysicsStep(double ms) {
  mCurrentThrust = 0.0;
  mCurrentTorque = 0.0;

  OmRotationalMotor *const m = motor();
  if (m == NULL)
    return;

  const bool run = m->runKinematicControl(ms, mPosition);
  if (!run)
    return;

  const double velocity = m->currentVelocity();
  const double absoluteVelocity = fabs(velocity);
  if (absoluteVelocity > 0.0) {
    const OmSolid *const us = upperSolid();
    // The gate was the ODE merger body (OmSolidMerger::mBody), which the ODE
    // deletion left permanently NULL -- set in the constructor, assigned
    // nowhere. So this returned early for EVERY propeller on every world, and
    // the warning it printed blamed a missing Physics node on airframes that
    // declared one. Nothing below ever ran: no thrust, no reaction torque, and
    // no mCurrentThrust, which is also why RotationalMotor::getTorqueFeedback
    // read zero on the one motor in the tree where it is real.
    // Gate on the NEWTON body instead, the same handle applyExternalForceNewton
    // resolves a few lines down.
    if (us->bodyHandle() == NULL) {
      parsingWarn(tr("Adds a Physics node to Solid ancestors to enable thrust and torque effect."));
      return;
    }

    // Computes thrust and torque
    const OmPose *const up = upperPose();
    const OmVector3 &cot = up->matrix() * mCenterOfThrust->value();
    // ODE removed: the body point-velocity read at the thrust centre is gone, so
    // the axial inflow speed V is now pinned to zero and the INFLOW TERM of the
    // thrust/torque model (thrustConstants[1], torqueConstants[1]) is
    // UNIMPLEMENTED -- only the omega^2 term survives. Zero is what the inert
    // stub already produced, so this preserves the current numbers exactly.
    const double V = 0.0;

    const OmVector2 &tcs = mTorqueConstants->value();
    mCurrentTorque = tcs.x() * velocity * absoluteVelocity - tcs.y() * absoluteVelocity * V;
    const double mt = m->maxForceOrTorque();
    if (fabs(mCurrentTorque) > mt)
      mCurrentTorque = mCurrentTorque > 0.0 ? mt : -mt;

    const OmVector2 &fcs = mThrustConstants->value();
    mCurrentThrust = fcs.x() * velocity * absoluteVelocity - fcs.y() * absoluteVelocity * V;

    // Applies thrust and torque
    const OmMatrix3 &m3 = up->rotationMatrix();
    const OmVector3 &axisVector = m3 * mNormalizedAxis;
    const OmVector3 &thrustVector = mCurrentThrust * axisVector;
    const OmVector3 &torqueVector = -mCurrentTorque * axisVector;
    // W3.1 (newton-ode-replacement-plan.md): deliver the wrench to NEWTON --
    // applyExternalForceNewton routes thrust@cot + reaction torque to Newton's
    // body_f. ODE removed: the dBodyEnable and the addBodyForceAtPos/addBodyTorque
    // fallback that used to sit around this call are gone, so if the Solid has no
    // Newton body it now gets NO THRUST AT ALL -- Propeller thrust off the Newton
    // path is UNIMPLEMENTED (it was already a no-op against the inert stub).
    us->applyExternalForceNewton(thrustVector, cot, torqueVector);

    updateHelix(absoluteVelocity);
    if (mHelix == NULL)
      return;

    // Moves the slow helix
    const OmQuaternion q(mNormalizedAxis, mPosition);
    const OmQuaternion iq(mHelix->rotationFromFile(stateId()).toQuaternion());
    OmQuaternion qp(q * iq);
    if (qp.w() != 1.0)
      qp.normalize();
    OmRotation r(qp);
    if (r.angle() == 0.0)
      r = OmRotation(mNormalizedAxis.x(), mNormalizedAxis.y(), mNormalizedAxis.z(), 0.0);
    const OmVector3 &c = mCenterOfThrust->value();
    mHelix->setTranslationAndRotation(q * (mHelix->translationFromFile(stateId()) - c) + c, r);
  }
}

void OmPropeller::createWrenObjects() {
  // D1.4: the WREN shaft-axis line renderable is gone (wgpu overlay path owns
  // optional-rendering visuals); only the helix recursion survives.
  OmBaseNode::createWrenObjects();

  OmSolid *const fastHelix = helix(FAST_HELIX);
  OmSolid *const slowHelix = helix(SLOW_HELIX);

  if (fastHelix)
    fastHelix->createWrenObjects();
  if (slowHelix)
    slowHelix->createWrenObjects();
}

void OmPropeller::setMatrixNeedUpdate() {
  OmSolid *const fastHelix = helix(FAST_HELIX);
  OmSolid *const slowHelix = helix(SLOW_HELIX);

  if (fastHelix)
    fastHelix->setMatrixNeedUpdate();
  if (slowHelix)
    slowHelix->setMatrixNeedUpdate();
}
OmSolid *OmPropeller::helix(HelixType type) const {
  return type == FAST_HELIX ? static_cast<OmSolid *>(mFastHelix->value()) : static_cast<OmSolid *>(mSlowHelix->value());
}

void OmPropeller::propagateSelection(bool selected) {
  const OmSimulationState *const state = OmSimulationState::instance();
  const bool pause = state->isPaused();
  OmSolid *const fastHelix = helix(FAST_HELIX);
  OmSolid *const slowHelix = helix(SLOW_HELIX);
  const bool fast = mHelixType == FAST_HELIX;

  if (fastHelix && (fast || pause))
    fastHelix->propagateSelection(selected);

  const bool slow = !fast;
  if (slowHelix && (slow || pause))
    slowHelix->propagateSelection(selected);
}

void OmPropeller::write(OmWriter &writer) const {
  if (writer.isOmniSim())
    OmBaseNode::write(writer);
  else {
    const OmSolid *const fastHelix = helix(FAST_HELIX);
    const OmSolid *const slowHelix = helix(SLOW_HELIX);
    if (writer.isW3d())
      writer << "<Propeller>";
    else {
      writer << "Group {\n";
      writer.increaseIndent();
      writer.indent();
      writer << "children ";
    }
    writer.writeMFStart();
    if (fastHelix) {
      writer.writeMFSeparator(true, false);
      fastHelix->write(writer);
    }
    if (slowHelix) {
      writer.writeMFSeparator(!fastHelix, false);
      slowHelix->write(writer);
    }
    writer.writeMFEnd(!fastHelix && !slowHelix);
    if (writer.isW3d())
      writer << "</Propeller>";
    else {
      writer << "\n";
      writer.decreaseIndent();
      writer.indent();
      writer << "}";
    }
  }
}

void OmPropeller::reset(const QString &id) {
  OmBaseNode::reset(id);

  OmNode *const d = mDevice->value();
  if (d)
    d->reset(id);
  OmNode *const fastHelix = mFastHelix->value();
  if (fastHelix)
    fastHelix->reset(id);
  OmNode *const slowHelix = mSlowHelix->value();
  if (slowHelix)
    slowHelix->reset(id);

  updateHelix(0.0);
}

QList<const OmBaseNode *> OmPropeller::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const OmBaseNode *> list;
  const OmBaseNode *const fastHelix = helix(FAST_HELIX);
  if (fastHelix)
    list << fastHelix->findClosestDescendantNodesWithDedicatedWrenNode();
  const OmBaseNode *const slowHelix = helix(SLOW_HELIX);
  if (slowHelix)
    list << slowHelix->findClosestDescendantNodesWithDedicatedWrenNode();
  return list;
}
