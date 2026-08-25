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

#ifndef OM_KINEMATIC_DIFFERENTIAL_WHEELS_HPP
#define OM_KINEMATIC_DIFFERENTIAL_WHEELS_HPP

#include "OmVector2.hpp"

class OmBaseNode;
class OmCylinder;
class OmHingeJoint;
class OmRobot;

class OmKinematicDifferentialWheels {
public:
  virtual ~OmKinematicDifferentialWheels() {}
  static OmKinematicDifferentialWheels *createKinematicDifferentialWheelsIfNeeded(OmRobot *robot);

  // kinematic motion model
  void applyKinematicMotion(double ms);

  // kinematic collision model
  void applyKinematicDisplacement();
  void addKinematicDisplacement(OmVector2 displacement) {
    mKinematicDisplacement += displacement;
    mKinematicDisplacementNumber++;
  }

private:
  OmKinematicDifferentialWheels(OmRobot *robot, double wheelsRadius, double axleLength, OmHingeJoint *leftJoint,
                                OmHingeJoint *rightJoint);
  static OmCylinder *getRecursivelyBigestCylinder(OmBaseNode *node);
  // kinematic displacement (kinematic collision model)
  OmVector2 mKinematicDisplacement;
  int mKinematicDisplacementNumber;
  double mWheelsRadius;
  double mAxleLength;
  OmHingeJoint *mWheelJoints[2];
  OmRobot *mRobot;
};

#endif  // OM_KINEMATIC_DIFFERENTIAL_WHEELS_HPP
