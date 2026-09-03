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

#include "OmJoint.hpp"

#include "OmBrake.hpp"
#include "OmJointParameters.hpp"
#include "OmMatrix4.hpp"
#include "OmMotor.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmPositionSensor.hpp"
#include "OmRobot.hpp"
#include "OmSolidReference.hpp"
#include "OmVector3.hpp"


void OmJoint::init() {
  mDevice = findMFNode("device");

  // hidden field
  mPosition = findSFDouble("position")->value();
  mOdePositionOffset = mPosition;
  mTimeStep = 0.0;
  mSavedPositions[stateId()] = mPosition;
}

// Constructors

OmJoint::OmJoint(const QString &modelName, OmTokenizer *tokenizer) : OmBasicJoint(modelName, tokenizer) {
  init();
}

OmJoint::OmJoint(const OmJoint &other) : OmBasicJoint(other) {
  init();
}

OmJoint::OmJoint(const OmNode &other) : OmBasicJoint(other) {
  init();
}

OmJoint::~OmJoint() {
}

void OmJoint::downloadAssets() {
  OmBasicJoint::downloadAssets();
  OmMotor *m = motor();
  if (m)
    m->downloadAssets();
  m = motor2();
  if (m)
    m->downloadAssets();
  m = motor3();
  if (m)
    m->downloadAssets();
}

void OmJoint::preFinalize() {
  OmBasicJoint::preFinalize();

  mSavedPositions[stateId()] = mPosition;

  for (int i = 0; i < devicesNumber(); ++i) {
    if (device(i) && !device(i)->isPreFinalizedCalled())
      device(i)->preFinalize();
  }
}

void OmJoint::postFinalize() {
  OmBasicJoint::postFinalize();

  for (int i = 0; i < devicesNumber(); ++i) {
    if (device(i) && !device(i)->isPostFinalizedCalled())
      device(i)->postFinalize();
  }

  connect(mDevice, &OmMFNode::itemChanged, this, &OmJoint::addDevice);
  connect(mDevice, &OmMFNode::itemInserted, this, &OmJoint::addDevice);
  if (brake())
    connect(brake(), &OmBrake::brakingChanged, this, &OmJoint::updateSpringAndDampingConstants, Qt::UniqueConnection);
}

void OmJoint::reset(const QString &id) {
  OmBasicJoint::reset(id);

  for (int i = 0; i < mDevice->size(); ++i)
    mDevice->item(i)->reset(id);

  setPosition(mSavedPositions[id]);
}

void OmJoint::resetPhysics() {
  updatePosition();

  OmMotor *const m = motor();
  if (m)
    m->resetPhysics();
}

void OmJoint::save(const QString &id) {
  OmBasicJoint::save(id);

  for (int i = 0; i < mDevice->size(); ++i)
    mDevice->item(i)->save(id);

  mSavedPositions[id] = mPosition;
}

void OmJoint::setPosition(double position, int index) {
  if (index != 1)
    return;

  mPosition = position;
  mOdePositionOffset = position;
  OmJointParameters *const p = parameters();
  if (p)
    p->setPosition(mPosition);

  // THE SURVIVING WRITER IN THE RESET CASCADE. reset() below ends with
  // setPosition(mSavedPositions[id]), so this line -- not OmMotor::reset -- is
  // what actually lands on a motor when a scene is restored, and a finite
  // target here is what demotes a velocity-mode wheel to a position hold and
  // brakes it. Outside a reset (a field edit, a supervisor pose write, load
  // time) the guard is always true and this behaves as it always has.
  OmMotor *const m = motor();
  if (m && OmMotor::resetMayOverwriteMotorCommand())
    m->setTargetPosition(position);
}

bool OmJoint::resetJointPositions() {
  mOdePositionOffset = 0.0;
  return OmBasicJoint::resetJointPositions();
}

// Update methods: they check validity and correct if necessary

void OmJoint::addDevice(int index) {
  const OmSolid *const s = upperSolid();
  if (s) {
    OmRobot *const r = s->robot();
    assert(r);
    OmBaseNode *decendant = dynamic_cast<OmBaseNode *>(mDevice->item(index));
    r->descendantNodeInserted(decendant);
  }
  OmBrake *b = dynamic_cast<OmBrake *>(mDevice->item(index));
  if (b)
    connect(b, &OmBrake::brakingChanged, this, &OmJoint::updateSpringAndDampingConstants, Qt::UniqueConnection);
}

void OmJoint::updateParameters() {
  const OmJointParameters *const p = parameters();
  if (p) {
    mOdePositionOffset = p->position();
    mPosition = mOdePositionOffset;
    connect(p, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
    connect(p, &OmJointParameters::minAndMaxStopChanged, this, &OmJoint::updateMinAndMaxStop, Qt::UniqueConnection);
    connect(p, &OmJointParameters::springAndDampingConstantsChanged, this, &OmJoint::updateSpringAndDampingConstants,
            Qt::UniqueConnection);
    connect(p, &OmJointParameters::axisChanged, this, &OmJoint::updateAxis, Qt::UniqueConnection);
  }
}

// Utility functions

OmJointParameters *OmJoint::parameters() const {
  return dynamic_cast<OmJointParameters *>(mParameters->value());
}

bool OmJoint::axisRepresentation(OmVector3 &worldAnchor, OmVector3 &worldAxis) const {
  const OmPose *const p = upperPose();
  if (!p)
    return false;
  const OmMatrix4 &m4 = p->matrix();
  worldAnchor = m4 * anchor();                          // local anchor -> world point
  worldAxis = m4.sub3x3MatrixDot(axis()).normalized();  // local axis -> world direction
  return true;
}

OmJointDevice *OmJoint::device(int index) const {
  if (index >= 0 && mDevice->size() > index)
    return dynamic_cast<OmJointDevice *>(mDevice->item(index));
  else
    return NULL;
}

int OmJoint::devicesNumber() const {
  return mDevice->size();
}

QVector<OmLogicalDevice *> OmJoint::devices() const {
  QVector<OmLogicalDevice *> devices;
  for (int i = 0; i < devicesNumber(); ++i)
    devices.append(device(i));

  return devices;
}

OmMotor *OmJoint::motor() const {
  OmMotor *motor = NULL;
  for (int i = 0; i < mDevice->size(); ++i) {
    motor = dynamic_cast<OmMotor *>(mDevice->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

OmPositionSensor *OmJoint::positionSensor() const {
  OmPositionSensor *sensor = NULL;
  for (int i = 0; i < mDevice->size(); ++i) {
    sensor = dynamic_cast<OmPositionSensor *>(mDevice->item(i));
    if (sensor)
      return sensor;
  }

  return NULL;
}

OmBrake *OmJoint::brake() const {
  OmBrake *brake = NULL;
  for (int i = 0; i < mDevice->size(); ++i) {
    brake = dynamic_cast<OmBrake *>(mDevice->item(i));
    if (brake)
      return brake;
  }

  return NULL;
}

void OmJoint::updateAxis() {
  // update the current endPoint pose based on the new axis value
  // but do not modify the initial endPoint pose
  updatePosition();
}

void OmJoint::updateMinAndMaxStop(double, double) {
  // Hard stops reach Newton through the joint registration; the ODE
  // dParamLoStop/HiStop update this used to do is gone.
}

OmVector3 OmJoint::axis() const {
  static const OmVector3 DEFAULT_AXIS(0.0, 0.0, 1.0);
  const OmJointParameters *p = parameters();
  return p ? p->axis() : DEFAULT_AXIS;
}

void OmJoint::setOdeJoint() {
  OmBasicJoint::setOdeJoint();

  // compute and set the orientation of rotation axis
  applyToOdeAxis();

  // place hard stops if defined
  applyToOdeMinAndMaxStop();
}

void OmJoint::updateOdePositionOffset() {
  double newValue = position();
  if (mOdePositionOffset == newValue)
    return;

  mOdePositionOffset = newValue;
  applyToOdeMinAndMaxStop();
}

//////////
// WREN //
//////////

void OmJoint::createWrenObjects() {
  // D1.4: the WREN joint-axis line renderable is gone (wgpu overlay draws joint
  // axes via axisRepresentation()); only the device recursion survives.
  OmBasicJoint::createWrenObjects();

  // create objects for Muscle devices
  for (int i = 0; i < devicesNumber(); ++i) {
    if (device(i))
      device(i)->createWrenObjects();
  }
}

const QString OmJoint::urdfName() const {
  if (motor())
    return getUrdfPrefix() + motor()->deviceName();
  else if (positionSensor())
    return getUrdfPrefix() + positionSensor()->deviceName();
  return OmBaseNode::urdfName();
}

void OmJoint::writeExport(OmWriter &writer) const {
  if (writer.isUrdf() && solidEndPoint()) {
    if (dynamic_cast<OmSolidReference *>(mEndPoint->value())) {
      this->warn("Exporting a Joint node with a SolidReference endpoint to URDF is not supported.");
      return;
    }

    const OmNode *const parentRoot = findUrdfLinkRoot();
    const OmVector3 currentOffset = solidEndPoint()->translation() - anchor();
    const OmVector3 translation = solidEndPoint()->translationFrom(parentRoot) - currentOffset + writer.jointOffset();
    writer.setJointOffset(solidEndPoint()->rotationMatrixFrom(parentRoot).transposed() * currentOffset);
    const OmVector3 eulerRotation = solidEndPoint()->rotationMatrixFrom(parentRoot).toEulerAnglesZYX();
    const OmVector3 rotationAxis = axis() * solidEndPoint()->rotationMatrixFrom(OmNodeUtilities::findUpperPose(this));

    writer.increaseIndent();
    writer.indent();
    const OmMotor *m = motor();
    if (m && (m->minPosition() != 0.0 || m->maxPosition() != 0.0))
      writer << QString("<joint name=\"%1\" type=\"revolute\">\n").arg(urdfName());
    else
      writer << QString("<joint name=\"%1\" type=\"continuous\">\n").arg(urdfName());

    writer.increaseIndent();
    writer.indent();
    writer << QString("<parent link=\"%1\"/>\n").arg(parentRoot->urdfName());
    writer.indent();
    writer << QString("<child link=\"%1\"/>\n").arg(solidEndPoint()->urdfName());
    writer.indent();
    writer << QString("<axis xyz=\"%1\"/>\n").arg(rotationAxis.toString(OmPrecision::FLOAT_ROUND_6));
    writer.indent();

    if (m) {
      if (m->minPosition() != 0.0 || m->maxPosition() != 0.0)
        writer << QString("<limit effort=\"%1\" lower=\"%2\" upper=\"%3\" velocity=\"%4\"/>\n")
                    .arg(m->maxForceOrTorque())
                    .arg(m->minPosition())
                    .arg(m->maxPosition())
                    .arg(m->maxVelocity());
      else
        writer << QString("<limit effort=\"%1\" velocity=\"%2\"/>\n").arg(m->maxForceOrTorque()).arg(m->maxVelocity());
      writer.indent();
    }
    writer << QString("<origin xyz=\"%1\" rpy=\"%2\"/>\n")
                .arg(translation.toString(OmPrecision::FLOAT_ROUND_6))
                .arg(eulerRotation.toString(OmPrecision::FLOAT_ROUND_6));
    writer.decreaseIndent();

    writer.indent();
    writer << QString("</joint>\n");
    writer.decreaseIndent();

    OmNode::exportNodeSubNodes(writer);
    return;
  }

  OmBasicJoint::writeExport(writer);
}
