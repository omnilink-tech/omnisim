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

#include "WbOdeContext.hpp"

#include "WbLog.hpp"

#include <ode/fluid_dynamics/objects_fluid_dynamics.h>
#include <cassert>
#include <cstring>

WbOdeContext *WbOdeContext::cOdeContext = NULL;

WbOdeContext::WbOdeContext() : QObject() {
  dInitODE();
  mWorld = dWorldCreate();
  dWorldSetLinearDampingThreshold(mWorld, 0.0);
  dWorldSetAngularDampingThreshold(mWorld, 0.0);

  mSpace = dSimpleSpaceCreate(NULL);
  mBroadphaseType = "simple";

  mJointGroupCreationMutex = new QMutex();
  mImmersionLinkGroup1 = dImmersionLinkGroupCreate();
  mImmersionLinkGroup2 = dImmersionLinkGroupCreate();
  mPhysicsPluginContactJointGroup1 = dJointGroupCreate(0);
  mPhysicsPluginContactJointGroup2 = dJointGroupCreate(0);
  mBodyContactJointGroupList1.clear();
  mBodyContactJointGroupList2.clear();

  mNumberOfThreads = -1;
  cOdeContext = this;
}

WbOdeContext::~WbOdeContext() {
  QList<dJointGroupID> groups(mBodyContactJointGroupList1.values());
  for (int i = 0; i < groups.size(); ++i) {
    dJointGroupDestroy(groups[i]);
  }
  groups = mBodyContactJointGroupList2.values();
  for (int i = 0; i < groups.size(); ++i) {
    dJointGroupDestroy(groups[i]);
  }
  mBodyContactJointGroupList1.clear();
  mBodyContactJointGroupList2.clear();

  dJointGroupDestroy(mPhysicsPluginContactJointGroup1);
  dJointGroupDestroy(mPhysicsPluginContactJointGroup2);
  dSpaceDestroy(mSpace);
  dWorldDestroy(mWorld);

  delete mJointGroupCreationMutex;
  mJointGroupCreationMutex = NULL;

  dCloseODE();
  cOdeContext = NULL;
}

void WbOdeContext::setNumberOfThreads(int n) {
  if (mNumberOfThreads == n)
    return;
  mNumberOfThreads = n;
  // dToggleODE_MT(0) will cause ODE to work non-threaded
  // dToggleODE_MT(1) will cause ODE to work on multi-thread with a single thread, which is inefficient
  // This is why when we set the number of threads to 1, we want to revert to the non-threaded mode
  if (mNumberOfThreads == 1)
    dToggleODE_MT(0);
  else
    dToggleODE_MT(n);
}

void WbOdeContext::setGravity(double x, double y, double z) {
  dWorldSetGravity(mWorld, x, y, z);
}

void WbOdeContext::setErp(double erp) {
  dWorldSetERP(mWorld, erp);
}

void WbOdeContext::setCfm(double cfm) {
  dWorldSetCFM(mWorld, cfm);
}

void WbOdeContext::setDamping(double linear, double angular) {
  dWorldSetDamping(mWorld, linear, angular);

  emit worldDefaultDampingChanged();
}

void WbOdeContext::setPhysicsDisableTime(double time) {
  dWorldSetAutoDisableTime(mWorld, time);
  dWorldSetAutoDisableSteps(mWorld, 0);  // focus is on time left (ms) before sleep
}

void WbOdeContext::gatherSleepingStats(int *totalBodies, int *enabled, int *staticColliders) const {
  // Walk the collision space once. Each geom either:
  //   (a) has a dBodyID attached -- it's part of a dynamic Solid; we
  //       count the body once (dedupe by dBodyID since one body can
  //       carry many geoms) and check dBodyIsEnabled to bucket awake
  //       vs sleeping.
  //   (b) has no body (dGeomGetBody returns NULL) -- it's a static
  //       collider (arena floor, wall, immovable scenery). Counted
  //       separately because the "sleeping island" question doesn't
  //       apply to them; they're permanently inactive by construction.
  //
  // QHash for dedup so a chassis with 4 wheel geoms doesn't get
  // counted as 4 bodies. The per-call cost is O(numGeoms) plus the
  // hash lookups -- fine for telemetry, since this is explicit-call
  // not per-step.
  QHash<dBodyID, bool> seen;  // value = isEnabled (for the sleeping bucket)
  int statics = 0;

  const int n = dSpaceGetNumGeoms(mSpace);
  for (int i = 0; i < n; ++i) {
    const dGeomID g = dSpaceGetGeom(mSpace, i);
    const dBodyID b = dGeomGetBody(g);
    if (b == NULL) {
      ++statics;
      continue;
    }
    if (!seen.contains(b))
      seen.insert(b, dBodyIsEnabled(b) != 0);
  }

  if (totalBodies)
    *totalBodies = seen.size();
  if (enabled) {
    int e = 0;
    for (auto it = seen.constBegin(); it != seen.constEnd(); ++it)
      if (it.value())
        ++e;
    *enabled = e;
  }
  if (staticColliders)
    *staticColliders = statics;
}

void WbOdeContext::setBroadphaseType(const char *type) {
  // Canonicalize: "auto" maps to "sap" today. The full AABB-aware
  // selection (quadtree when world > 500 m horizontally, else sap)
  // requires walking all Solids' bounds, which haven't been created
  // yet at createOdeObjects time. That walk is a follow-up; for now
  // "auto" is just a synonym for the safer modern default.
  const char *target = type;
  if (target == NULL || *target == '\0' || std::strcmp(target, "auto") == 0)
    target = "sap";

  // Already on the requested type? Short-circuit. This is the common
  // case for worlds that don't set the field (default "simple") or
  // worlds that reload without changing their broadphase choice.
  if (std::strcmp(target, mBroadphaseType) == 0)
    return;

  // Safety net: if the current space already has geoms, we can't
  // tear it down without re-registering each one. That's complex
  // (the geoms' dGeomID's hold a back-pointer to their space), so
  // the conservative move is to skip the switch and warn. Hitting
  // this means somebody called setBroadphaseType after world-load
  // creation order broke -- the common world-load path runs this
  // from WbWorldInfo::createOdeObjects, which fires before any
  // Solid's createOdeObjects, so mSpace should be empty.
  const int numGeoms = dSpaceGetNumGeoms(mSpace);
  if (numGeoms != 0) {
    WbLog::warning(
      QString("Cannot switch ODE broadphase from '%1' to '%2': current space already has %3 geom(s). "
              "Broadphase changes only take effect when no Solids have been created yet.")
        .arg(mBroadphaseType, target, QString::number(numGeoms)));
    return;
  }

  // Switch: destroy the old space, create the new one. The dWorld
  // is untouched; only the collision space changes type.
  dSpaceDestroy(mSpace);
  if (std::strcmp(target, "sap") == 0) {
    // axisorder=0 picks the default XYZ ordering. SAP is good for
    // worlds where dynamic bodies cluster (warehouses, indoor
    // scenes); each pair test gets cheap broadphase rejection.
    mSpace = dSweepAndPruneSpaceCreate(NULL, 0);
    mBroadphaseType = "sap";
  } else if (std::strcmp(target, "quadtree") == 0) {
    // 2 km square arena centered at origin, depth 4 (so leaves are
    // 125 m on a side -- finer than that bloats the tree; coarser
    // than that misses the benefit on a 500 m world). The plan's
    // 500 m threshold for "auto" was picked so worlds bigger than
    // that get the finer leaf. Authors with very large outdoor
    // scenes should override these defaults via a follow-up SFVec3f
    // field; for now we ship a conservative one-size-fits-most.
    const dVector3 center = {0, 0, 0};
    const dVector3 extents = {1000, 1000, 1000};
    mSpace = dQuadTreeSpaceCreate(NULL, center, extents, 4);
    mBroadphaseType = "quadtree";
  } else if (std::strcmp(target, "simple") == 0) {
    mSpace = dSimpleSpaceCreate(NULL);
    mBroadphaseType = "simple";
  } else {
    // Unknown value -- fall back to the canonical default rather than
    // leave mSpace dangling. Same logging convention as the registry's
    // fall-through path.
    WbLog::warning(QString("Unknown broadphase type '%1'; falling back to 'simple'.").arg(target));
    mSpace = dSimpleSpaceCreate(NULL);
    mBroadphaseType = "simple";
  }
}

void WbOdeContext::emptyEnabledBodyContactJointGroups1() {
  // mutex is locked in WbSimulationCluster before calling this function
  QHashIterator<dBodyID, dJointGroupID> it(mBodyContactJointGroupList1);
  while (it.hasNext()) {
    it.next();
    if (dBodyIsEnabled(it.key()))
      dJointGroupEmpty(it.value());
  }
}

void WbOdeContext::emptyEnabledBodyContactJointGroups2() {
  // mutex is locked in WbSimulationCluster before calling this function
  QHashIterator<dBodyID, dJointGroupID> it(mBodyContactJointGroupList2);
  while (it.hasNext()) {
    it.next();
    if (dBodyIsEnabled(it.key()))
      dJointGroupEmpty(it.value());
  }
}

dJointGroupID WbOdeContext::bodyContactJointGroup1(dBodyID b) {
  // mutex is locked in WbSimulationCluster before calling this function
  QHash<dBodyID, dJointGroupID>::const_iterator it = mBodyContactJointGroupList1.find(b);
  if (it == mBodyContactJointGroupList1.end()) {
    // body not found
    dJointGroupID group = dJointGroupCreate(0);
    mBodyContactJointGroupList1.insert(b, group);
    return group;
  }

  return it.value();
}

dJointGroupID WbOdeContext::bodyContactJointGroup2(dBodyID b) {
  // mutex is locked in WbSimulationCluster before calling this function
  QHash<dBodyID, dJointGroupID>::const_iterator it = mBodyContactJointGroupList2.find(b);
  if (it == mBodyContactJointGroupList2.end()) {
    // body not found
    dJointGroupID group = dJointGroupCreate(0);
    mBodyContactJointGroupList2.insert(b, group);
    return group;
  }

  return it.value();
}

void WbOdeContext::removeBodyContactJointGroup(dBodyID b) {
  mJointGroupCreationMutex->lock();
  if (mBodyContactJointGroupList1.contains(b)) {
    dJointGroupDestroy(mBodyContactJointGroupList1.value(b));
    mBodyContactJointGroupList1.remove(b);
  }
  if (mBodyContactJointGroupList2.contains(b)) {
    dJointGroupDestroy(mBodyContactJointGroupList2.value(b));
    mBodyContactJointGroupList2.remove(b);
  }
  mJointGroupCreationMutex->unlock();
}
