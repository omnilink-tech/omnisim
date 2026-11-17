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

#include "OmConnector.hpp"

#include "OmDataStream.hpp"
#include "OmMFNode.hpp"
#include "OmMFVector3.hpp"
#include "OmNewtonBackend.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPhysics.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmRobot.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSensor.hpp"
#include "OmSimulationState.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QList>
#include <QtCore/QSet>


#include <cassert>

static const double MAX_STRENGTH = DBL_MAX / 2.0;

static QList<OmConnector *> gConnectorList;

// OMNISIM_NEWTON_WELDS (value-parsed, DEFAULT OFF): route Connector /
// VacuumGripper lock welds to the Newton backend's MuJoCo equality-weld slots
// (_scratch/design_weld_touch.md) instead of the ODE dJointCreateFixed path,
// which on Newton-backed Solids constrains two artificially-disabled proxy
// bodies -- i.e. holds NOTHING (and its dJointFeedback reads eternal zeros, so
// rupture never fires). "0"/"false"/"off"/"no" = off; unset = off.
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

// next time that mouse motion is allowed
// static long nextMotionTime = 0;

void OmConnector::init() {
  // add myself to the list of connector
  gConnectorList.append(this);

  // init member variables
  mFaceType = UNKNOWN;
  mMinDist2 = -1.0;
  mPeer = NULL;
  mStartup = true;
  mSensor = NULL;
  mIsJointInversed = false;
  mNeedToReconfigure = false;
  mNewtonWeldSlot = -1;
  mNewtonWeldActive = false;

  // init fields
  mType = findSFString("type");
  mIsLocked = findSFBool("isLocked");
  mAutoLock = findSFBool("autoLock");
  mUnilateralLock = findSFBool("unilateralLock");
  mUnilateralUnlock = findSFBool("unilateralUnlock");
  mDistanceTolerance = findSFDouble("distanceTolerance");
  mAxisTolerance = findSFDouble("axisTolerance");
  mRotationTolerance = findSFDouble("rotationTolerance");
  mNumberOfRotations = findSFInt("numberOfRotations");
  mSnap = findSFBool("snap");
  mTensileStrength = findSFDouble("tensileStrength");
  mShearStrength = findSFDouble("shearStrength");

  mIsInitiallyLocked[stateId()] = mIsLocked->value();
}

OmConnector::OmConnector(OmTokenizer *tokenizer) : OmSolidDevice("Connector", tokenizer) {
  init();
}

OmConnector::OmConnector(const OmConnector &other) : OmSolidDevice(other) {
  init();
}

OmConnector::OmConnector(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmConnector::~OmConnector() {
  // remove bonds (fixed joints)
  if (mPeer)
    detachFromPeer();

  gConnectorList.removeOne(this);
}

void OmConnector::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();

  updateType();
  updateIsLocked();
  updateDistanceTolerance();
  updateAxisTolerance();
  updateRotationTolerance();
  updateNumberOfRotations();
  updateTensileStrength();
  updateShearStrength();
}

void OmConnector::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mType, &OmSFString::changed, this, &OmConnector::updateType);
  connect(mIsLocked, &OmSFBool::changed, this, &OmConnector::updateIsLocked);
  connect(mDistanceTolerance, &OmSFBool::changed, this, &OmConnector::updateDistanceTolerance);
  connect(mAxisTolerance, &OmSFDouble::changed, this, &OmConnector::updateAxisTolerance);
  connect(mRotationTolerance, &OmSFDouble::changed, this, &OmConnector::updateRotationTolerance);
  connect(mNumberOfRotations, &OmSFInt::changed, this, &OmConnector::updateNumberOfRotations);
  connect(mTensileStrength, &OmSFDouble::changed, this, &OmConnector::updateTensileStrength);
  connect(mShearStrength, &OmSFDouble::changed, this, &OmConnector::updateShearStrength);
}

void OmConnector::updateType() {
  const QString &type = mType->value();
  if (type == "symmetric")
    mFaceType = SYMMETRIC;
  else if (type == "active")
    mFaceType = ACTIVE;
  else if (type == "passive")
    mFaceType = PASSIVE;
  else {
    parsingWarn(tr("Unknown 'type' \"%1\": locking disabled.").arg(type));
    mFaceType = UNKNOWN;
  }
}

void OmConnector::updateIsLocked() {
  if (mFaceType == PASSIVE) {
    if (mIsLocked->isTrue()) {
      parsingWarn(tr("Passive connectors cannot be locked."));
      mIsLocked->setFalse();
    }
    return;
  }

  if (mIsLocked->isTrue())
    lock();
  else
    unlock();

  mNeedToReconfigure = true;
}

void OmConnector::updateNumberOfRotations() {
  if (mNumberOfRotations->value() < 0) {
    parsingWarn(tr("'numberOfRotations' must be positive or zero."));
    mNumberOfRotations->setValue(0);
  }
}

void OmConnector::updateDistanceTolerance() {
  if (mDistanceTolerance->value() < 0.0) {
    parsingWarn(tr("'distanceTolerance' must be positive or zero."));
    mDistanceTolerance->setValue(0.0);
  }
  mMinDist2 = mDistanceTolerance->value() * mDistanceTolerance->value();
}

void OmConnector::updateAxisTolerance() {
  if (mAxisTolerance->clip(0.0, M_PI))
    parsingWarn(tr("'axisTolerance' must be between 0 and pi."));
}

void OmConnector::updateRotationTolerance() {
  if (mRotationTolerance->clip(0.0, M_PI))
    parsingWarn(tr("'rotationTolerance' must between 0 and pi."));
}

void OmConnector::updateTensileStrength() {
  if (mTensileStrength->value() < 0.0 && mTensileStrength->value() != -1.0) {
    parsingWarn(tr("'tensileStrength' must be positive or -1 (infinite)."));
    mTensileStrength->setValue(-1.0);
  }
}

void OmConnector::updateShearStrength() {
  if (mShearStrength->value() < 0.0 && mShearStrength->value() != -1.0) {
    parsingWarn(tr("'shearStrength' must be positive or -1 (infinite)."));
    mShearStrength->setValue(-1.0);
  }
}

// return the angle (in the range [0 pi]) between two approximately normalized vectors
// this function should be used when it is known that v1 and v2 have approx lentgh = 1.0
// In this case it is faster than: acos((v1.v2)/|v1|.|v2|)
static inline double unitVectorsAngle(const OmVector3 &v1, const OmVector3 &v2) {
  double cos = v1.dot(v2);
  if (cos >= 1.0)
    return 0.0;
  else if (cos <= -1.0)
    return M_PI;
  else
    return acos(cos);
}

// search for possible rotational alignment matching alpha angle
// (thanks to problem symmetry we need to look only in 180°)
// input: alpha angle (angle between z-vectors of connectors)
// returns: -1.0 if no matching z-alignment was found
double OmConnector::findClosestRotationalAlignment(double alpha) const {
  int n = mNumberOfRotations->value();
  double angleStep = 2.0 * M_PI / n;
  double t = mRotationTolerance->value();
  double beta = 0.0;
  for (int i = 0; i < n / 2 + 1; i++) {
    if (alpha > beta - t && alpha < beta + t)
      return beta;
    else
      beta += angleStep;
  }

  return -1.0;
}

// this function must be called once the connectors are aligned
void OmConnector::createFixedJoint(OmConnector *other) {
  // Newton-native weld (OMNISIM_NEWTON_WELDS). When either side's merge
  // leader owns a Newton body, the ODE path below is physically inert: the
  // proxy bodies are artificially disabled at Newton registration, so a
  // dJointCreateFixed between them holds nothing and its feedback reads
  // zeros. Route to the backend's pre-allocated equality-weld slot instead --
  // or, with the gate off, keep the exact legacy behaviour but SAY SO once.
  {
    const int nb1 = nearestNewtonBodyIndex();
    const int nb2 = other->nearestNewtonBodyIndex();
    if (nb1 >= 0 || nb2 >= 0) {
      if (newtonWeldsEnabled()) {
        createNewtonWeld(other, nb1, nb2);
        return;
      }
      static QSet<QString> sWarnedInert;
      if (!sWarnedInert.contains(name())) {
        sWarnedInert.insert(name());
        warn(tr("Connector '%1' locks are physically INERT under the Newton backend: the ODE fixed joint it creates "
                "constrains disabled proxy bodies, so the pair will not hold and rupture never fires. Set "
                "OMNISIM_NEWTON_WELDS=1 to enable the native Newton weld path.")
               .arg(name()));
      }
    }
  }
  // No Newton weld engaged: with ODE gone there is no fixed joint to fall back on.
  warn(tr("Connectors could not be attached because neither of them (nor their parent nodes) has a Physics node."));
}

void OmConnector::attachTo(OmConnector *other) {
  assert(!mPeer);

  // if other connector is already attached: give up
  if (other->mPeer)
    return;

  // either mUnilateralLock or the other's side agreement is required to lock
  if (!(mUnilateralLock->isTrue() || other->mIsLocked->isTrue()))
    return;

  // A Newton-backed pair attaches through createFixedJoint, whose Newton-weld
  // branch handles it. ⚠ Connector 'snap' alignment is UNIMPLEMENTED: the ODE
  // snap (rotate / shift both bodies half-way before the fixed joint) went
  // with ODE and no Newton equivalent is wired, so the weld holds the pair at
  // its current relative pose.
  if (nearestNewtonBodyIndex() >= 0 || other->nearestNewtonBodyIndex() >= 0) {
    createFixedJoint(other);
    return;
  }
  warn(tr("Connectors could not be attached because neither of them (nor their parent nodes) has a Physics node."));
}

void OmConnector::ensureNewtonWeldSlot(OmNewtonBackend *newton) {
  if (mNewtonWeldSlot >= 0 || newton == nullptr || !newtonWeldsEnabled())
    return;
  // Anchor the placeholder on this connector's merge-leader body. A bodiless
  // connector (no Physics anywhere up its joint-free chain) gets no slot --
  // its connections use the PEER's slot, mirroring ODE's one-joint-per-
  // connection ownership. PASSIVE connectors DO get a slot for exactly that
  // reason (a bodiless active side welds the passive side's body to the
  // world).
  const int body = nearestNewtonBodyIndex();
  if (body < 0)
    return;
  mNewtonWeldSlot = newton->addWeldSlot(body);
}

// Newton counterpart of createFixedJoint: engage the connection's weld slot at
// the pair's CURRENT relative pose. `snap TRUE` intentionally does NOT get
// ODE's move-bodies/attach/restore dance -- presence detection already
// guarantees alignment within the authored tolerances, so the engage-at-
// current-pose weld is at most one tolerance off the snapped pose (recorded
// deviation from the design doc's analytic-snap suggestion; revisit if a demo
// needs exact snap).
void OmConnector::createNewtonWeld(OmConnector *other, int newtonBody1, int newtonBody2) {
  OmNewtonBackend *const newton = availableNewtonBackend();
  if (newton == nullptr)
    return;
  // The connection's single weld lives on whichever side has a slot --
  // preferring this (locking) side, like ODE's joint ownership.
  OmConnector *owner = mNewtonWeldSlot >= 0 ? this : (other->mNewtonWeldSlot >= 0 ? other : nullptr);
  if (owner == nullptr) {
    static QSet<QString> sWarnedNoSlot;
    if (!sWarnedNoSlot.contains(name())) {
      sWarnedNoSlot.insert(name());
      warn(tr("Connector '%1': no Newton weld slot was reserved for this connection (the devices registered after the "
              "world was finalized, or neither side's body is Newton-owned), so the lock will NOT hold. Author both Connectors in the world file with a Physics node on each side's Solid; a node spawned or re-parented at runtime needs POST /sim/rebuild_physics before it can lock -- see docs/reference/connector.md.")
             .arg(name()));
    }
    return;
  }
  // Mirror the ODE bodiless branch: when this side has no body, the weld's
  // obj1 becomes the peer's body (welded to the world), exactly like
  // dJointAttach(joint, NULL, b2) -- ODE swaps a NULL body1 internally too,
  // which is why f1 belongs to the real body in both engines.
  int bodyA = newtonBody1, bodyB = newtonBody2;
  if (bodyA < 0) {
    bodyA = newtonBody2;
    bodyB = -1;
  }
  if (newton->weldEngage(owner->mNewtonWeldSlot, bodyA, bodyB) != 0)
    return;  // weldEngage warned (e.g. mujoco_warp) -- do not marry
  owner->mNewtonWeldActive = true;
  // Rupture runs on the OWNER's prePhysicsStep with the OWNER's x-axis, and
  // weld_force reports the force ON obj1 -- so the ODE sign convention
  // ("tension = pull along +x, f1 on node[0]") holds VERBATIM when obj1 is
  // the owner's own side's body, and flips (mIsJointInversed) when obj1
  // belongs to the owner's peer. obj1 is this side's body iff newtonBody1
  // was valid (the swap above), hence:
  const bool obj1IsThisSide = newtonBody1 >= 0;
  owner->mIsJointInversed = (owner == this) ? !obj1IsThisSide : obj1IsThisSide;

  // marriage (same bookkeeping as createFixedJoint's ODE tail)
  mPeer = other;
  other->mPeer = this;
}

void OmConnector::releaseNewtonWeld() {
  if (!mNewtonWeldActive)
    return;
  mNewtonWeldActive = false;
  OmNewtonBackend *const newton = availableNewtonBackend();
  if (newton != nullptr)
    newton->weldRelease(mNewtonWeldSlot);
}

void OmConnector::detachFromPeer() {
  assert(mPeer);

  // only one weld holds the two bodies together -- find which side of the
  // connection owns it (the Newton weld slot) and undo that one
  if (mNewtonWeldActive)
    releaseNewtonWeld();
  else if (mPeer->mNewtonWeldActive)
    mPeer->releaseNewtonWeld();

  // detaching connectors may cause some motion that wasn't possible when they were attached to each other
  // therefore we need to explicitely awake both of them in case they were idle
  // so that the physics engine can generate their motion accordingly
  awake();
  mPeer->awake();

  // divorce
  mPeer->mPeer = NULL;
  mPeer = NULL;
}

double OmConnector::getEffectiveTensileStrength() const {
  if (mIsLocked->isFalse())
    return 0.0;
  else if (mTensileStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mTensileStrength->value();
}

double OmConnector::getEffectiveShearStrength() const {
  if (mIsLocked->isFalse())
    return 0.0;
  else if (mShearStrength->value() == -1.0)
    return MAX_STRENGTH;
  else
    return mShearStrength->value();
}

// check if the force exterted on the weld (mFixedJoint or the Newton slot)
// exceeds the limit, if it does, detach the connectors
// Note that this function is called for just one of the two connectors that build up each connection
void OmConnector::detachIfForceExceedStrength() {
  assert(mPeer && mNewtonWeldActive);

  OmVector3 f1;
  if (mNewtonWeldActive) {
    // Rupture is wanted only when some strength field is finite -- the exact
    // condition under which the ODE path allocates joint feedback.
    if (mTensileStrength->value() == -1.0 && mShearStrength->value() == -1.0 &&
        mPeer->mTensileStrength->value() == -1.0 && mPeer->mShearStrength->value() == -1.0)
      return;
    OmNewtonBackend *const newton = availableNewtonBackend();
    double w[6];
    if (newton == nullptr || newton->weldForce(mNewtonWeldSlot, w) != 0)
      return;
    // w[0..2] = world-frame force the weld applies ON obj1 (probed on the
    // bundled mujoco: a hanging 1 kg welded to the world reads +9.81 z --
    // the constraint HOLDS obj1). ODE's f1 is the same convention: force on
    // node[0], which dJointAttach's NULL-body swap makes the real body, just
    // as weld_engage's swap makes obj1 the real body -- so mIsJointInversed
    // carries over verbatim below.
    f1 = OmVector3(w[0], w[1], w[2]);
  } else {
    return;  // no ODE fixed joint can exist any more
  }

  // the tensile direction corresponds to the positive x-axis
  // compute how much of the measured force is aligned with the x-axis
  const double xforce = mIsJointInversed ? -xAxis().dot(f1) : xAxis().dot(f1);

  // check for tensile rupture
  double maxTension = getEffectiveTensileStrength() + mPeer->getEffectiveTensileStrength();

  // we are interested only in the positive x-direction
  double tension = xforce < 0.0 ? 0.0 : xforce;
  if (tension > maxTension) {
    detachFromPeer();
    return;
  }

  // check for shear rupture
  double maxShear = getEffectiveShearStrength() + mPeer->getEffectiveShearStrength();
  if (maxShear < MAX_STRENGTH) {
    // find shear force (using Pythagoras theorem)
    double magnitude = f1.length();
    double shearing = sqrt(magnitude * magnitude - xforce * xforce);
    if (shearing > maxShear) {
      detachFromPeer();
      return;
    }
  }
}

void OmConnector::prePhysicsStep(double ms) {
  bool skipAttach = mPeer != NULL;
  if (mNewtonWeldActive)
    detachIfForceExceedStrength();

  // handle pre-locked mStartup case (must be done once only)
  if (mStartup) {
    if (mIsLocked->isTrue())
      lock();
    mStartup = false;
  }

  // call baseclass
  OmSolidDevice::prePhysicsStep(ms);

  // autolocking (if not just detached)
  if (!skipAttach && !mPeer && mAutoLock->isTrue() && mIsLocked->isTrue()) {
    OmConnector *presence = detectPresence();
    if (presence)
      attachTo(presence);
  }
}

// locking required by controller
void OmConnector::lock() {
  if (mFaceType == PASSIVE) {
    parsingWarn(tr("Passive connectors cannot lock."));
    return;
  }

  mIsLocked->setTrue();

  if (!mPeer) {
    OmConnector *presence = detectPresence();
    if (presence)
      attachTo(presence);
  }
}

// unlocking required by controller
void OmConnector::unlock() {
  if (mFaceType == PASSIVE) {
    parsingWarn(tr("Passive connectors cannot lock."));
    return;
  }

  mIsLocked->setValue(false);

  if (mPeer && (mUnilateralUnlock->isTrue() || mPeer->mIsLocked->isFalse()))
    detachFromPeer();
}

// search in the whole list a connector that meets the
// compatibility and alignment criteria to attach
OmConnector *OmConnector::detectPresence() const {
  double min2 = 99999.9;
  OmConnector *result = NULL;
  foreach (OmConnector *c, gConnectorList) {
    if (c != this && isReadyToAttachTo(c)) {
      double dist2 = getDistance2(c);
      if (dist2 <= mMinDist2) {  // is near enough ?
        if (dist2 < min2) {
          min2 = dist2;
          result = c;
        }
      }
    }
  }

  return result;
}

bool OmConnector::isCompatibleWith(const OmConnector *other) const {
  int a = faceType();
  int b = other->faceType();

  // test face mType compatibility
  if (!((a == SYMMETRIC && b == SYMMETRIC) || (a == ACTIVE && b == PASSIVE) || (a == PASSIVE && b == ACTIVE)))
    return false;

  // test model compatibility
  return model() == other->model();
}

// returns true if this connector and the other connectors x-axes are parallel (but with opposite directions)
// In other words, the angle between them must be 180° with some tolerance
bool OmConnector::isXAlignedWith(const OmConnector *other) const {
  // the vector [ matrix[8], matrix[9], matrix[10] ] represents a connectors x-axis
  // orientation in global coordinate system, its length is approximately 1.0
  return unitVectorsAngle(xAxis(), other->xAxis()) > M_PI - mAxisTolerance->value();
}

// returns true if this connector and the other connector's z-axes are
// rotationally aligned according to mNumberOfRotations and mRotationTolerance
bool OmConnector::isZAlignedWith(const OmConnector *other) const {
  // if n == 0 any rotational alignment is fine
  if (mNumberOfRotations->isZero())
    return true;

  // compare the connectors z-axis orientation in global coordinate system
  // (the vector [ matrix[4], matrix[5], matrix[6] ] represents a connectors z-axis
  // orientation in global coordinate system, its length is approximately 1.0)
  double alpha = unitVectorsAngle(zAxis(), other->zAxis());

  // search for matching alignment
  return findClosestRotationalAlignment(alpha) != -1.0;
}

double OmConnector::getDistance2(const OmConnector *other) const {
  return (matrix().translation() - other->matrix().translation()).length2();
}

bool OmConnector::isAlignedWith(const OmConnector *other) const {
  return isXAlignedWith(other) && isZAlignedWith(other);
}

bool OmConnector::isReadyToAttachTo(const OmConnector *other) const {
  return isCompatibleWith(other) && isAlignedWith(other);
}

void OmConnector::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_CONNECTOR_GET_PRESENCE:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    case C_CONNECTOR_LOCK:
      lock();
      return;
    case C_CONNECTOR_UNLOCK:
      unlock();
      return;
    default:
      assert(0);
  }
}

void OmConnector::computeValue() {
  if (faceType() == PASSIVE)
    mValue = -1;
  else if (mPeer)
    mValue = 1;
  else
    mValue = detectPresence() ? 1 : 0;
}

bool OmConnector::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void OmConnector::reset(const QString &id) {
  OmSolidDevice::reset(id);
  mIsLocked->setValue(mIsInitiallyLocked[id]);
  if (mPeer)
    detachFromPeer();
  mStartup = true;
  mNeedToReconfigure = true;
}

void OmConnector::save(const QString &id) {
  OmSolidDevice::save(id);
  mIsInitiallyLocked[id] = mIsLocked->value();
}

void OmConnector::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    computeValue();
    stream << (unsigned short int)tag();
    stream << (unsigned char)C_CONNECTOR_GET_PRESENCE;
    stream << (unsigned short int)mValue;
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void OmConnector::writeConfigure(OmDataStream &) {
  if (robot())
    mSensor->connectToRobotSignal(robot());
}

void OmConnector::addConfigure(OmDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (unsigned char)(mIsLocked->value() ? 1 : 0);
  mNeedToReconfigure = false;
}

// converts a rotation from quaternion to euler axis/angle representation
// input: normalized quaternion 'q' in ODE compatible format:  [ w x y z ]
// output: euler axis and angle 'aa' (VRML-like) format: [ x y z alpha ]
static inline void quaternionToAxesAndAngle(const double q[4], double aa[4]) {
#ifndef NDEBUG  // ensure that the quaternion is normalized as it should be
  double nn = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  assert(nn > 0.9999999999 && nn < 1.0000000001);
#endif
  // if q[0] is slightly greater than 1 or slightly lower than -1, acos(q[0]) will return nan
  // unfortunately, due to floating point rounding, that happens even if the quaternion was just normalized
  if (q[0] >= 1.0)
    aa[3] = 0.0;
  else if (q[0] <= -1.0)
    aa[3] = 2.0 * M_PI;
  else
    aa[3] = 2.0 * acos(q[0]);

  if (aa[3] < 0.0001) {  // if aa[3] is close to zero, then the direction of the axis is not important
    aa[0] = 0.0;
    aa[1] = 1.0;
    aa[2] = 0.0;
    aa[3] = 0.0;
  } else {  // normalise axes
    double n = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    aa[0] = q[1] / n;
    aa[1] = q[2] / n;
    aa[2] = q[3] / n;
  }
}

// ---------- below: auto-assembly mechanism ------------------------------------------------------------

void OmConnector::assembleAxes(OmConnector *other) {
  // we need to apply the rotation to the whole solid not only the Connector
  OmSolid *solid = topSolid();


  // x-axes of both connectors
  OmVector3 x1 = xAxis();
  OmVector3 x2 = -other->xAxis();

  // find rotation axis w using cross product of x-axes
  OmVector3 w = x1.cross(x2);

  // if x1 and x2 are collinear we are already x-aligned
  (void)w;

  // if n == 0 we don't need to rotate
  int n = mNumberOfRotations->value();
  if (n) {
    // z-axes of connector 1 and 2
    // z1 and z2 have unit length
    OmVector3 z1 = zAxis();
    OmVector3 z2 = other->zAxis();

    // find required angle of rotation from z1 to z2
    double alpha = unitVectorsAngle(z1, z2);

    // if the y vectors are parallel we don't need to rotate
    if (alpha) {
      // find w, the rotation axis from z1 to z2
      w = z1.cross(z2);

      // special case: if z1 and z2 are anti-parallel:
      // rotate of 180° around x-axis
      if (w.isNull()) {
        w[0] = 0.0;
        w[1] = 0.0;
        w[2] = 1.0;
        alpha = M_PI;
      }

      // search for possible rotational alignment
      // we should always find a rotational alignment
      double beta = findClosestRotationalAlignment(alpha);
      assert(beta != -1.0);

      // set quaternion (r) to represent the required rotation
      (void)beta;  // rotational auto-assembly is compiled out (see above)
    }
  }

  // position of both connectors
  OmVector3 p1 = matrix().translation();
  OmVector3 p2 = other->matrix().translation();

  // translation from connector 1 to connector
  OmVector3 t = p2 - p1;

  // if necessary translate whole solid
  if (!t.isNull())
    solid->translate(t[0], t[1], t[2]);

  // update ODE bodies
  // solid->setBody();
}

void OmConnector::assembleWith(OmConnector *other) {
  assembleAxes(other);

  if (mAutoLock->isTrue())
    mIsLocked->setValue(true);

  if (mIsLocked->isTrue())
    createFixedJoint(other);
}

void OmConnector::hasMoved() {
  // see who is close now
  OmConnector *presence = detectPresence();

  if (!presence && mPeer)
    detachFromPeer();
  else if (presence && !mPeer) {
    assembleWith(presence);
    // block object motion for 800 milliseconds
    // nextMotionTime = wxGetLocalTimeMillis() + 800;
  }
}

// look recursively through Solid and notify each Connector
void OmConnector::solidHasMoved(OmSolid *solid) {
  // when the simulation is running we don's allow changes
  if (OmSimulationState::instance()->isFast())
    return;

  OmConnector *connector = dynamic_cast<OmConnector *>(solid);
  if (connector)
    connector->hasMoved();
  else {
    foreach (OmSolid *s, solid->solidChildren())
      solidHasMoved(s);
  }
}

/*
bool OmConnector::isAllowingMouseMotion() {
  return wxGetLocalTimeMillis() > nextMotionTime;
}*/

// D1.4: the WREN connector-axes / rotation-alignment visuals (VF_CONNECTOR_AXES)
// died with the WREN renderer; the wgpu overlay path owns optional-rendering
// visuals. All connector/weld logic above is untouched.
