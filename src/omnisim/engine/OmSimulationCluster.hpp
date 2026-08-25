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

#ifndef OM_SIMULATION_CLUSTER_HPP
#define OM_SIMULATION_CLUSTER_HPP

//
// Description: single-threaded simulation
//

// The opaque handle typedefs keep this class's public API shape now that the
// real ODE headers are gone. (The members that took ODE VALUE types -- dContact,
// dImmersionSurfaceParameters -- were behind the build guard and went with it.)
#include "OmOdeTypes.hpp"
#include <QtCore/QList>
#include <QtCore/QMutex>

class OmContactProperties;
class OmOdeContext;
class OmKinematicDifferentialWheels;
class OmSolid;
class OmGeometry;
class QMutex;

class OmSimulationCluster {
public:
  // create/destroy cluster thread and ODE objects
  explicit OmSimulationCluster(OmOdeContext *context);
  virtual ~OmSimulationCluster();

  // collision detection and simulation steps
  // for this cluster's world and space
  void step();

  // ODE objects
  dWorldID world() const;
  dSpaceID space() const;
  dJointGroupID bodyContactJointGroup(dBodyID b);
  dImmersionLinkGroupID immersionLinkGroup() const;

  void handleInitialCollisions();  // used to synchronize contact point representations and current positions

private:
  OmOdeContext *mContext;
  static QMutex *cJointCreationMutex;

  QMutex mCollisionedRobotsMutex;
  QList<OmKinematicDifferentialWheels *> mCollisionedRobots;
  void appendCollisionedRobot(OmKinematicDifferentialWheels *robot);
  void handleKinematicsCollisions();
  void swapBuffer();
  static void handleCollisionIfSpace(void *data, dGeomID o1, dGeomID o2);
  static const OmContactProperties *findContactProperties(const OmSolid *s1, const OmSolid *s2);
  static void odeNearCallback(void *data, dGeomID o1, dGeomID o2);
  static void odeSensorRaysUpdate(int threadID);
  static const long long int WEBOTS_MAGIC_NUMBER;
  bool mSwapJointContactBuffer;
  static void warnMoreContactPointsThanContactJoints(const QString &material1, const QString &material2, int max, int n);
};

#endif
