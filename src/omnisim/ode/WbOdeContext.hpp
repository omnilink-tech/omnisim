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

#ifndef WB_ODE_CONTEXT_HPP
#define WB_ODE_CONTEXT_HPP

#include <ode/fluid_dynamics/common_fluid_dynamics.h>
#include <ode/ode.h>

#include <QtCore/QHash>
#include <QtCore/QMutex>
#include <QtCore/QObject>

class WbOdeContext : public QObject {
  Q_OBJECT

public:
  // create ODE objects
  WbOdeContext();

  // destroy ODE objects
  ~WbOdeContext();

  static WbOdeContext *instance() { return cOdeContext; }

  // apply to the world
  void setGravity(double x, double y, double z);
  void setCfm(double cfm);
  void setErp(double erp);
  void setDamping(double linear, double angular);
  void setPhysicsDisableTime(double time);
  void setNumberOfThreads(int n);

  // ODE broadphase selection (§8.4 of rendering-roadmap.md; landed).
  // Accepted values: "simple" (the default, brute-force O(N^2)), "sap"
  // (sweep-and-prune), "quadtree" (good for outdoor scenes), "auto"
  // (defensive fallback → "sap"; callers should pre-resolve "auto"
  // themselves via WbWorldInfo::resolveBroadphaseChoice when geometry-
  // aware selection is wanted).
  //
  // Safe to call only when the current space has zero geoms (typical
  // case: from WbWorldInfo::createOdeObjects at world-load time, before
  // any Solid has populated the space). If the space already has geoms
  // the requested switch is skipped with a single WbLog::warning and
  // the existing space stays in place -- the world still loads, the
  // perf win just doesn't materialize for this run.
  //
  // The parameter is a C-string so this header doesn't have to pull in
  // <QtCore/QString>; callers convert via .toUtf8().constData().
  void setBroadphaseType(const char *type);

  // Sleeping-island telemetry (item 6, second half: sleeping verification).
  // Walks the current collision space once, counting:
  //   totalBodies     -- distinct dBodyIDs found across all geoms
  //   enabled         -- subset of those that are dBodyIsEnabled (awake)
  //   staticColliders -- geoms with no body attached (immovable scenery,
  //                      arena walls etc.)
  // The "sleeping island fraction" the plan wants is
  // (totalBodies - enabled) / totalBodies. Any out-param may be null;
  // safe to call from anywhere, including outside the physics step.
  // O(numGeoms) per call -- explicit only, no per-step automatic fire,
  // so worlds that don't ask pay nothing.
  void gatherSleepingStats(int *totalBodies, int *enabled, int *staticColliders) const;

  // getters
  dWorldID world() const { return mWorld; }
  dSpaceID space() const { return mSpace; }
  int numberOfThreads() const { return mNumberOfThreads; }

  QMutex *jointGroupCreationMutex() { return mJointGroupCreationMutex; }
  dImmersionLinkGroupID immersionLinkGroup1() const { return mImmersionLinkGroup1; }
  dImmersionLinkGroupID immersionLinkGroup2() const { return mImmersionLinkGroup2; }
  dJointGroupID physicsPluginContactJointGroup1() const { return mPhysicsPluginContactJointGroup1; }
  dJointGroupID physicsPluginContactJointGroup2() const { return mPhysicsPluginContactJointGroup2; }

  void emptyEnabledBodyContactJointGroups1();
  void emptyEnabledBodyContactJointGroups2();
  dJointGroupID bodyContactJointGroup1(dBodyID b);
  dJointGroupID bodyContactJointGroup2(dBodyID b);
  void removeBodyContactJointGroup(dBodyID b);

signals:
  void worldDefaultDampingChanged();

private:
  dWorldID mWorld;
  dSpaceID mSpace;
  // Tracks which broadphase the current mSpace is. Defaults to "simple"
  // to match the dSimpleSpaceCreate call in the constructor. setBroadphaseType
  // compares against this to short-circuit no-op switches.
  const char *mBroadphaseType;

  QMutex *mJointGroupCreationMutex;
  QHash<dBodyID, dJointGroupID> mBodyContactJointGroupList1, mBodyContactJointGroupList2;
  dImmersionLinkGroupID mImmersionLinkGroup1, mImmersionLinkGroup2;
  dJointGroupID mPhysicsPluginContactJointGroup1, mPhysicsPluginContactJointGroup2;

  int mNumberOfThreads;
  static WbOdeContext *cOdeContext;
};

#endif
