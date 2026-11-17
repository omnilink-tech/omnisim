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

#include "OmKinematicDifferentialWheels.hpp"

#include "OmCylinder.hpp"
#include "OmHingeJoint.hpp"
#include "OmLog.hpp"
#include "OmMotor.hpp"
#include "OmRobot.hpp"
#include "OmSolid.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

// Webots' kinematic mode moves a Robot WITHOUT Physics by its wheel speeds. A
// Robot without Physics whose wheel Solids DO carry Physics is not that robot:
// under docs/reference/physics.md it is a static base (its physics-bearing
// children attach to the static environment), and it is exactly what
// `URDFRobot { staticBase TRUE }` produces, since the importer strips only the
// root link's Physics. Until 2026-10-20 the kinematic model still attached to
// it: a staticBase NORI A3 whose wheels were spun at 4 rad/s slid 0.61 m in 2 s
// (exactly omega * r, z frozen) while its log reported the base welded. Value-
// parsed hatch, default OFF: OMNISIM_KINEMATIC_WHEELS_WITH_PHYSICS=1 restores it.
static std::string trimmedLower(const char *raw) {
  std::string v = raw ? raw : "";
  v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c); }), v.end());
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return std::tolower(c); });
  return v;
}

static bool kinematicWheelsWithPhysicsEnabled() {
  static const bool on = [] {
    const std::string v = trimmedLower(std::getenv("OMNISIM_KINEMATIC_WHEELS_WITH_PHYSICS"));
    return !v.empty() && !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

OmKinematicDifferentialWheels::OmKinematicDifferentialWheels(OmRobot *robot, double wheelsRadius, double axleLength,
                                                             OmHingeJoint *leftJoint, OmHingeJoint *rightJoint) :
  mWheelsRadius(wheelsRadius),
  mAxleLength(axleLength),
  mRobot(robot) {
  mWheelJoints[0] = leftJoint;
  mWheelJoints[1] = rightJoint;
  mKinematicDisplacementNumber = 0;
}

void OmKinematicDifferentialWheels::applyKinematicMotion(double ms) {
  mKinematicDisplacementNumber = 0;
  mKinematicDisplacement.setXy(0, 0);

  const double leftVelocity = mWheelJoints[0]->motor()->currentVelocity();
  const double rightVelocity = mWheelJoints[1]->motor()->currentVelocity();

  const double t = ms * 0.001;
  // rotation angle
  const double deltaDirection = (rightVelocity - leftVelocity) * t * mWheelsRadius / (mAxleLength * 2.0);
  const double a = mRobot->rotation().angle() + deltaDirection;            // [rad/s]
  const double v0 = 0.5 * (leftVelocity + rightVelocity) * mWheelsRadius;  // [m/s]

  const double v = v0 * t;
  mRobot->setTranslation(mRobot->translation() + OmVector3(v * cos(a), v * sin(a), 0.0));
  if (deltaDirection != 0.0)
    mRobot->setRotationAngle(mRobot->rotation().angle() + 2.0 * deltaDirection);

  mRobot->printKinematicWarningIfNeeded();
}

void OmKinematicDifferentialWheels::applyKinematicDisplacement() {
  // update position
  OmVector3 position = mRobot->translation();
  mRobot->setTranslation(position[0] - mKinematicDisplacement[0] / mKinematicDisplacementNumber,
                         position[1] - mKinematicDisplacement[1] / mKinematicDisplacementNumber, position[2]);
}

OmCylinder *OmKinematicDifferentialWheels::getRecursivelyBigestCylinder(OmBaseNode *node) {
  OmCylinder *wheelCylinder = NULL;
  if (node) {
    OmCylinder *cylinder = dynamic_cast<OmCylinder *>(node);
    if (cylinder)
      return cylinder;
    const OmGroup *group = dynamic_cast<OmGroup *>(node);
    if (group) {
      for (int i = 0; i < group->childCount(); ++i) {
        cylinder = getRecursivelyBigestCylinder(group->child(i));
        if (cylinder) {
          if (wheelCylinder && cylinder->radius() > wheelCylinder->radius())
            wheelCylinder = cylinder;
          else if (!wheelCylinder)
            wheelCylinder = cylinder;
        }
      }
    }
    const OmShape *shape = dynamic_cast<OmShape *>(node);
    if (shape) {
      cylinder = dynamic_cast<OmCylinder *>(shape->geometry());
      if (cylinder)
        return cylinder;
    }
  }
  return wheelCylinder;
}

OmKinematicDifferentialWheels *OmKinematicDifferentialWheels::createKinematicDifferentialWheelsIfNeeded(OmRobot *robot) {
  if (robot->isDynamic())
    return NULL;
  // check if the required joints and motors exist
  OmHingeJoint *leftJoint = NULL;
  OmHingeJoint *rightJoint = NULL;
  const QVector<OmBasicJoint *> joints = robot->jointChildren();
  QVector<OmHingeJoint *> motorizedJoints;
  for (int i = 0; i < joints.size(); ++i) {
    // cppcheck-suppress constVariablePointer
    OmHingeJoint *joint = dynamic_cast<OmHingeJoint *>(joints.at(i));
    if (!joint || !joint->motor() || !joint->solidEndPoint())
      continue;
    motorizedJoints << joint;
  }
  if (motorizedJoints.size() < 2)
    return NULL;
  // check all the possible pairs of joints
  for (int i = 0; i < motorizedJoints.size(); ++i) {
    leftJoint = motorizedJoints.at(i);
    const OmCylinder *leftWheelCylinder = getRecursivelyBigestCylinder(leftJoint->solidEndPoint()->boundingObject());
    // make sure this joint has a cylinder bounding object
    if (!leftWheelCylinder || leftWheelCylinder->radius() <= 0.0)
      continue;
    double leftWheelRadius = leftWheelCylinder->radius();
    double leftWheelDistance = (robot->position() - leftWheelCylinder->upperPose()->position()).length();
    for (int j = i + 1; j < motorizedJoints.size(); ++j) {
      rightJoint = motorizedJoints.at(j);
      // make sure this joint has a cylinder bounding object
      const OmCylinder *rightWheelCylinder = getRecursivelyBigestCylinder(rightJoint->solidEndPoint()->boundingObject());
      if (!rightWheelCylinder || rightWheelCylinder->radius() <= 0.0)
        continue;
      // make sure both cylinders have the same size
      double rightWheelRadius = rightWheelCylinder->radius();
      if (leftWheelRadius != rightWheelRadius)
        continue;
      double rightWheelDistance = (robot->position() - rightWheelCylinder->upperPose()->position()).length();
      // make sure the wheels are equally centered
      if (abs(leftWheelDistance - rightWheelDistance) > 1e-5)
        continue;
      // make sure the joint axes are parallel
      if (!leftJoint->axis().cross(rightJoint->axis()).isNull())
        continue;
      // make sure the axis between the 2 anchors is parallel to the joints axes
      OmVector3 anchorAxis = rightJoint->anchor() - leftJoint->anchor();
      if (!leftJoint->axis().cross(anchorAxis).isNull())
        continue;
      OmVector3 globalLeftAnchor =
        robot->rotationMatrix().transposed() * (robot->position() - leftWheelCylinder->upperPose()->position());
      OmVector3 globalRightAnchor =
        robot->rotationMatrix().transposed() * (robot->position() - rightWheelCylinder->upperPose()->position());
      // a static base, not a kinematic robot (see kinematicWheelsWithPhysicsEnabled)
      if (!kinematicWheelsWithPhysicsEnabled() &&
          (leftJoint->solidEndPoint()->physics() != nullptr || rightJoint->solidEndPoint()->physics() != nullptr)) {
        OmLog::info(QString("Robot '%1' has no Physics but its wheels '%2' and '%3' do, so it is a static base "
                            "(staticBase TRUE, docs/reference/physics.md): the kinematic differential-wheels mode is "
                            "not applied and the base stays where it was placed; "
                            "OMNISIM_KINEMATIC_WHEELS_WITH_PHYSICS=1 reverts")
                      .arg(robot->name(), leftJoint->solidEndPoint()->name(), rightJoint->solidEndPoint()->name()));
        return NULL;
      }
      if (globalLeftAnchor.y() < globalRightAnchor.y())  // make sure the joint are not inverted
        return new OmKinematicDifferentialWheels(robot, leftWheelRadius, leftWheelDistance + rightWheelDistance, leftJoint,
                                                 rightJoint);
      else
        return new OmKinematicDifferentialWheels(robot, leftWheelRadius, leftWheelDistance + rightWheelDistance, rightJoint,
                                                 leftJoint);
    }
  }
  return NULL;
}
