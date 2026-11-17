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

#include "OmVacuumGripper.hpp"

#include "OmBasicJoint.hpp"
#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmLog.hpp"
#include "OmNewtonBackend.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmSFDouble.hpp"
#include "OmSensor.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QList>
#include <QtCore/QMap>


#include <cassert>
#include <vector>

static const double MAX_STRENGTH = DBL_MAX / 2.0;

// OMNISIM_NEWTON_WELDS (value-parsed, DEFAULT OFF): route the suction weld to
// the Newton backend's MuJoCo equality-weld slots (_scratch/design_weld_touch
// .md) -- the ODE dJointCreateFixed below constrains artificially-disabled
// proxy bodies on Newton-backed Solids and holds NOTHING, and the ODE
// odeNearCallback attach trigger never fires for them either (measured
// ODE_pts=0 vs native_pts=8). "0"/"false"/"off"/"no" = off; unset = off.
static bool newtonWeldsEnabled() {
  static const bool on = []() {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_WELDS")).trimmed().toLower();
    // DEFAULT ON since 2026-08-08: ODE is being deleted, so there is no
    // fallback left to degrade to -- an unset gate must mean the native
    // path, not silence. "0"/"false"/"off"/"no" still opt out.
    if (v.isEmpty())
      return true;
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

// The available Newton backend, or NULL (runtime absent). The forced-ODE
// early-out this used to start with is gone with the OMNISIM_FORCE_ODE switch.
static OmNewtonBackend *availableNewtonBackend() {
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return nullptr;
  return static_cast<OmNewtonBackend *>(raw);
}

void OmVacuumGripper::init() {
  // init member variables
  mSolid = NULL;
  mSensor = NULL;
  mNeedToReconfigure = false;
  mNewtonWeldSlot = -1;
  mNewtonWeldActive = false;

  // init fields
  mIsOn = findSFBool("isOn");
  mTensileStrength = findSFDouble("tensileStrength");
  mShearStrength = findSFDouble("shearStrength");
  mContactPoints = findSFInt("contactPoints");

  mIsInitiallyOn[stateId()] = mIsOn->value();
}

OmVacuumGripper::OmVacuumGripper(OmTokenizer *tokenizer) : OmSolidDevice("VacuumGripper", tokenizer) {
  init();
}

OmVacuumGripper::OmVacuumGripper(const OmVacuumGripper &other) : OmSolidDevice(other) {
  init();
}

OmVacuumGripper::OmVacuumGripper(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmVacuumGripper::~OmVacuumGripper() {
  // remove bonds (fixed joints)
  if (mSolid)
    detachFromSolid();
}

void OmVacuumGripper::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();

  updateIsOn();
  updateTensileStrength();
  updateShearStrength();
  updateContactPoints();
}

void OmVacuumGripper::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mIsOn, &OmSFBool::changed, this, &OmVacuumGripper::updateIsOn);
  connect(mTensileStrength, &OmSFDouble::changed, this, &OmVacuumGripper::updateTensileStrength);
  connect(mShearStrength, &OmSFDouble::changed, this, &OmVacuumGripper::updateShearStrength);
}

void OmVacuumGripper::updateIsOn() {
  if (mIsOn->isTrue())
    turnOn();
  else
    turnOff();

  mNeedToReconfigure = true;
}

void OmVacuumGripper::updateTensileStrength() {
  if (mTensileStrength->value() < 0.0 && mTensileStrength->value() != -1.0) {
    parsingWarn(tr("'tensileStrength' must be positive or -1 (infinite)."));
    mTensileStrength->setValue(-1.0);
  }
}

void OmVacuumGripper::updateShearStrength() {
  if (mShearStrength->value() < 0.0 && mShearStrength->value() != -1.0) {
    parsingWarn(tr("'shearStrength' must be positive or -1 (infinite)."));
    mShearStrength->setValue(-1.0);
  }
}

void OmVacuumGripper::updateContactPoints() {
  OmFieldChecker::resetIntIfNonPositive(this, mContactPoints, 3);
}

void OmVacuumGripper::createFixedJoint(OmSolid *other) {
  // Newton-native weld (OMNISIM_NEWTON_WELDS). When the gripper's merge
  // leader owns a Newton body the ODE path below is physically inert (its
  // proxy bodies are artificially disabled) -- engage the pre-allocated
  // equality-weld slot at the pair's CURRENT relative pose instead; with the
  // gate off, keep the exact legacy behaviour but SAY SO once.
  {
    const int nb1 = nearestNewtonBodyIndex();
    const int nb2 = other != nullptr ? other->nearestNewtonBodyIndex() : -1;
    if (nb1 >= 0 || nb2 >= 0) {
      if (newtonWeldsEnabled()) {
        OmNewtonBackend *const newton = availableNewtonBackend();
        if (newton == nullptr)
          return;
        if (mNewtonWeldSlot < 0 || nb1 < 0) {
          static bool sWarnedNoSlot = false;
          if (!sWarnedNoSlot) {
            sWarnedNoSlot = true;
            warn(tr("VacuumGripper '%1': no Newton weld slot is available (the gripper's body is not Newton-owned or "
                    "it registered after world finalize), so turning it on will NOT hold the object. Give the gripper's Solid a Physics node and author it in the world file; a gripper spawned at runtime needs POST /sim/rebuild_physics before it can grip -- see docs/reference/vacuumgripper.md.")
                   .arg(name()));
          }
          return;
        }
        // The gripper's body is always obj1 (nb1 >= 0 guaranteed above);
        // nb2 < 0 welds it to the WORLD -- legitimate when the thing gripped
        // really is a bodiless static (suck onto a wall and the arm should
        // stop), but indistinguishable from it when the partner simply failed
        // to resolve. That silence cost a diagnosis: a gripper whose collider
        // had been dropped by the fold pinned the arm in mid-air with nothing
        // logged anywhere. SAY which one happened.
        if (nb2 < 0) {
          static bool sWarnedWorldWeld = false;
          if (!sWarnedWorldWeld) {
            sWarnedWorldWeld = true;
            warn(tr("VacuumGripper '%1' welded to the WORLD, not to an object: the solid it gripped has no Newton "
                    "body. If that solid is a static prop this is intended and the gripper is now anchored; "
                    "otherwise the gripper is pinned in mid-air and will not move until turnOff().")
                   .arg(name()));
          }
        }
        // Name the pair once. "the vacuum turned on and the arm stopped
        // moving" is not diagnosable without knowing WHAT it grabbed.
        static int sWeldLogged = 0;
        if (sWeldLogged < 5) {
          ++sWeldLogged;
          OmLog::info(tr("VacuumGripper '%1': weld engage slot=%2 body1=%3 body2=%4 (gripping '%5')")
                        .arg(name())
                        .arg(mNewtonWeldSlot)
                        .arg(nb1)
                        .arg(nb2)
                        .arg(other != nullptr ? other->name() : QStringLiteral("<none>")),
                      false, OmLog::ODE);
        }
        if (newton->weldEngage(mNewtonWeldSlot, nb1, nb2) != 0)
          return;  // weldEngage warned (e.g. mujoco_warp)
        mNewtonWeldActive = true;
        mSolid = other;
        connect(mSolid, &OmSolid::destroyed, this, &OmVacuumGripper::destroyFixedJoint);
        return;
      }
      static bool sWarnedInert = false;
      if (!sWarnedInert) {
        sWarnedInert = true;
        warn(tr("VacuumGripper '%1' is physically INERT under the Newton backend: the ODE fixed joint it creates "
                "constrains disabled proxy bodies, so the object will not be held. Set OMNISIM_NEWTON_WELDS=1 to "
                "enable the native Newton weld path.")
               .arg(name()));
      }
    }
  }

  // No Newton weld engaged: with ODE gone there is no fixed joint to fall back on.
  warn(tr(
    "VacuumGripper could not be attached because neither the VacuumGripper node nor the solid object have Physics nodes."));
}

void OmVacuumGripper::destroyFixedJoint() {
  // Newton weld branch: release the equality slot; there is no ODE joint or
  // feedback to free.
  if (mNewtonWeldActive) {
    mNewtonWeldActive = false;
    OmNewtonBackend *const newton = availableNewtonBackend();
    if (newton != nullptr)
      newton->weldRelease(mNewtonWeldSlot);
    mSolid = NULL;
    return;
  }
  // destroy ODE fixed joint and remove feedback structure
  mSolid = NULL;
}

void OmVacuumGripper::detachFromSolid() {
  assert(mSolid);
  OmSolid *attachedSolid = mSolid;

  disconnect(mSolid, &OmSolid::destroyed, this, &OmVacuumGripper::destroyFixedJoint);
  destroyFixedJoint();

  // detaching may cause some motion that wasn't possible when they were attached to each other
  // therefore we need to explicitely awake both of them in case they were idle
  // so that the physics engine can generate their motion accordingly
  awake();
  attachedSolid->awake();
}

double OmVacuumGripper::getEffectiveTensileStrength() const {
  if (mIsOn->isFalse())
    return 0.0;
  else if (mTensileStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mTensileStrength->value();
}

double OmVacuumGripper::getEffectiveShearStrength() const {
  if (mIsOn->isFalse())
    return 0.0;
  else if (mShearStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mShearStrength->value();
}

// check if the force exterted on the weld (mFixedJoint or the Newton slot)
// exceeds the limit, if it does, detach the dBodies
void OmVacuumGripper::detachIfForceExceedStrength() {
  assert(mSolid && mNewtonWeldActive);

  OmVector3 f1;
  if (mNewtonWeldActive) {
    // Rupture is wanted only when a strength field is finite -- the exact
    // condition under which the ODE path allocates joint feedback.
    if (mTensileStrength->value() == -1.0 && mShearStrength->value() == -1.0)
      return;
    OmNewtonBackend *const newton = availableNewtonBackend();
    double w[6];
    if (newton == nullptr || newton->weldForce(mNewtonWeldSlot, w) != 0)
      return;
    // w[0..2] = world-frame force the weld applies ON obj1 = the gripper's
    // body -- the same convention as ODE's f1 on node[0] (the gripper side is
    // always obj1 here, so no inversion flag is needed).
    f1 = OmVector3(w[0], w[1], w[2]);
  } else {
    return;  // no ODE fixed joint can exist any more
  }

  // the tensile direction corresponds to the positive x-axis
  // compute how much of the measured force is aligned with the x-axis
  const double xforce = xAxis().dot(f1);

  // check for tensile rupture
  // we are interested only in the positive x-direction
  const double tension = xforce < 0.0 ? 0.0 : xforce;
  if (tension > getEffectiveTensileStrength()) {
    detachFromSolid();
    return;
  }

  // check for shear rupture
  const double maxShear = getEffectiveShearStrength();
  if (maxShear < MAX_STRENGTH) {
    // find shear force (using Pythagoras theorem)
    double magnitude = f1.length();
    double shearing = sqrt(magnitude * magnitude - xforce * xforce);
    if (shearing > maxShear) {
      detachFromSolid();
      return;
    }
  }
}

void OmVacuumGripper::prePhysicsStep(double ms) {
  mCollidedSolidList.clear();

  if (mNewtonWeldActive)
    detachIfForceExceedStrength();

  // call base class
  OmSolidDevice::prePhysicsStep(ms);
}

void OmVacuumGripper::postPhysicsStep() {
  OmSolidDevice::postPhysicsStep();

  // The Newton arm has no odeNearCallback feed (mCollidedSolidList stays
  // empty for Newton bodies), so while waiting with a usable weld slot the
  // attach attempt runs every tick off the native contact snapshot instead.
  if (isWaitingForConnection() &&
      (!mCollidedSolidList.isEmpty() || (newtonWeldsEnabled() && mNewtonWeldSlot >= 0)))
    attachToSolid();
}

bool OmVacuumGripper::isWaitingForConnection() {
  return mIsOn->isTrue() && !mSolid;
}

void OmVacuumGripper::addCollidedSolid(OmSolid *solid, const double depth) {
  assert(solid);
  if (mSolid)
    return;  // ignore if already connected to another object
  mCollidedSolidList << std::pair<OmSolid *, const double>(solid, depth);
}

void OmVacuumGripper::attachToSolid() {
  // search for solid to connect to
  double maxDepth = 0;
  OmSolid *solid = NULL;
  QListIterator<std::pair<OmSolid *, const double>> it(mCollidedSolidList);
  while (it.hasNext()) {
    std::pair<OmSolid *, const double> item = it.next();
    if (item.second > maxDepth) {
      maxDepth = item.second;
      solid = item.first;
    }
  }

  // Newton arm: the ODE list is structurally empty for Newton bodies -- pick
  // the candidate off the native contact snapshot instead.
  if (!solid)
    solid = newtonDeepestCollidedSolid();

  if (solid)
    createFixedJoint(solid);
}

void OmVacuumGripper::ensureNewtonWeldSlot(OmNewtonBackend *newton) {
  if (mNewtonWeldSlot >= 0 || newton == nullptr || !newtonWeldsEnabled())
    return;
  const int body = nearestNewtonBodyIndex();
  if (body < 0)
    return;
  mNewtonWeldSlot = newton->addWeldSlot(body);
}

OmSolid *OmVacuumGripper::newtonDeepestCollidedSolid() const {
  // The Newton replacement for the odeNearCallback -> addCollidedSolid feed
  // (dead for Newton bodies: their disabled ODE proxies produce zero
  // contacts). Reads the backend's native contact snapshot and applies the
  // same admission rule -- at least `contactPoints` contacts with one solid.
  //
  // Phase-1 approximations, both recorded in _scratch/design_weld_touch.md:
  //   - contacts attribute to the MERGE LEADER's body, so any contact of the
  //     merged link counts as gripper contact (ODE attributed per-geom);
  //   - the native snapshot reports depth 0 (positional solve), so "deepest"
  //     degrades to "most contacts, then largest force sum".
  if (!newtonWeldsEnabled() || mNewtonWeldSlot < 0)
    return nullptr;
  OmNewtonBackend *const newton = availableNewtonBackend();
  if (newton == nullptr)
    return nullptr;
  const int myBody = nearestNewtonBodyIndex();
  if (myBody < 0)
    return nullptr;
  std::vector<OmNewtonContact> contacts;
  if (newton->getContacts(contacts) < 0)
    return nullptr;
  QMap<int, int> countByBody;
  QMap<int, double> forceByBody;
  for (const OmNewtonContact &c : contacts) {
    int other = -1;
    if (c.bodyA == myBody)
      other = c.bodyB;
    else if (c.bodyB == myBody)
      other = c.bodyA;
    if (other < 0)
      continue;  // not ours, or static world geometry with no Solid
    ++countByBody[other];
    forceByBody[other] += c.forceMag;
  }
  int best = -1, bestCount = 0;
  double bestForce = 0.0;
  for (auto itc = countByBody.constBegin(); itc != countByBody.constEnd(); ++itc) {
    if (itc.value() < contactPoints())
      continue;  // same admission rule as the ODE trigger (n >= contactPoints)
    const double f = forceByBody.value(itc.key());
    if (itc.value() > bestCount || (itc.value() == bestCount && f > bestForce)) {
      best = itc.key();
      bestCount = itc.value();
      bestForce = f;
    }
  }
  if (best < 0)
    return nullptr;
  return OmSolid::findSolidByNewtonBodyIndex(best);
}

void OmVacuumGripper::turnOn() {
  mIsOn->setTrue();
  // W0 (design_weld_touch.md): with the weld gate OFF, a Newton-bodied
  // gripper never even reaches createFixedJoint -- its ODE collision feed is
  // structurally empty, so turnOn() silently does nothing for ever. Say so
  // once instead.
  if (!newtonWeldsEnabled() && nearestNewtonBodyIndex() >= 0) {
    static bool sWarnedInertOn = false;
    if (!sWarnedInertOn) {
      sWarnedInertOn = true;
      warn(tr("VacuumGripper '%1' is physically INERT under the Newton backend: its ODE contact trigger never fires "
              "for Newton bodies, so turning it on will never attach anything. Set OMNISIM_NEWTON_WELDS=1 to enable "
              "the native Newton weld path.")
             .arg(name()));
    }
  }
  if (!mSolid)
    attachToSolid();
}

void OmVacuumGripper::turnOff() {
  mIsOn->setValue(false);
  if (mSolid)
    detachFromSolid();
}

void OmVacuumGripper::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_VACUUM_GRIPPER_GET_PRESENCE:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    case C_VACUUM_GRIPPER_TURN_ON:
      turnOn();
      return;
    case C_VACUUM_GRIPPER_TURN_OFF:
      turnOff();
      return;
    default:
      assert(0);
  }
}

void OmVacuumGripper::computeValue() {
  mValue = mSolid != NULL;
}

bool OmVacuumGripper::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void OmVacuumGripper::reset(const QString &id) {
  OmSolidDevice::reset(id);
  mIsOn->setValue(mIsInitiallyOn[id]);
  if (mSolid)
    detachFromSolid();
  mNeedToReconfigure = true;
}

void OmVacuumGripper::save(const QString &id) {
  OmSolidDevice::save(id);
  mIsInitiallyOn[id] = mIsOn->value();
}

void OmVacuumGripper::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    computeValue();
    stream << (unsigned short int)tag();
    stream << (unsigned char)C_VACUUM_GRIPPER_GET_PRESENCE;  // return if an object is connected
    stream << (unsigned char)(mValue ? 1 : 0);
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void OmVacuumGripper::writeConfigure(OmDataStream &) {
  if (robot())
    mSensor->connectToRobotSignal(robot());
}

void OmVacuumGripper::addConfigure(OmDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (unsigned char)(mIsOn->value() ? 1 : 0);
  mNeedToReconfigure = false;
}
