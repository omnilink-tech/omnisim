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

#include "OmReceiver.hpp"

#include "OmDataPacket.hpp"
#include "OmDataStream.hpp"
#include "OmEmitter.hpp"
#include "OmFieldChecker.hpp"
#include "OmGroup.hpp"
#include "OmOdeContext.hpp"
#include "OmRandom.hpp"
#include "OmRobot.hpp"
#include "OmSensor.hpp"
#include "OmSolid.hpp"
#include "OmSolidUtilities.hpp"
#include "OmWorld.hpp"
#include "../physics/OmNewtonBackend.hpp"
#include "../physics/OmPhysicsBackend.hpp"

#include <omnisim/receiver.h>  // for WB_CHANNEL_BROADCAST
#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QHash>
#include <cassert>

static QList<OmReceiver *> gReceiverList;

// Newton raycast service gate (kernel blocker #1, ode-retirement-campaign.md).
// Same value-parsed read as OmDistanceSensor::refreshRaysFromNewton: DEFAULT
// OFF until the parity suite covers every ray consumer; =1 opts a run in,
// =0/unset keeps the ODE ray path.
static bool isNewtonRaycastEnabled() {
  static int sGate = -1;
  if (sGate < 0) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_RAYCAST")).trimmed().toLower();
    // DEFAULT ON since 2026-08-07: every ray consumer answers from the
    // Newton service (mj_ray on the live mjModel), incl. the LASER
    // transparency re-cast and the joint-descending exclusion walk.
    // =0 reverts to the ODE ray verdicts while src/ode still ships.
    sGate = (v == "0" || v == "false" || v == "off" || v == "no") ? 0 : 1;
  }
  return sGate == 1;
}

// Collects the Newton body indices of every Solid under `robot` (same walk as
// OmDistanceSensor builds its mNewtonExcludeBodies).
static void collectRobotNewtonBodies(const OmRobot *robot, QVector<int> &out) {
  // shared walk: descends joint/slot endPoints too (the original per-file
  // OmGroup-only walk missed Solids behind joints, so an articulated robot
  // could see its own arm where ODE's same-robot rule excluded it)
  OmSolidUtilities::collectNewtonBodies(const_cast<OmRobot *>(robot), out);
}

class Transmission {
public:
  Transmission(OmDataPacket *packet, OmReceiver *r, dSpaceID spaceId) : mPacket(packet), mCollided(false) {
    assert(spaceId && packet && r);
    OmEmitter *e = packet->emitter();
    assert(e);
    mEmittingRobot = e->robot();
    assert(mEmittingRobot);

    // compute ray direction and length
    const OmVector3 &te = e->matrix().translation();
    OmVector3 dir = r->matrix().translation() - te;

    // setup ray geom for ODE collision detection
    // ODE is gone: no ray geom exists; carry the segment in plain
    // members so the Newton raycast service still answers the occlusion.
    mGeom = NULL;
    mStart = te;
    mSegmentDir = dir;
    mSegmentLength = dir.length();
  };

  ~Transmission() {
  }

  OmDataPacket *packet() { return mPacket; }
  bool hasCollided() const { return mCollided; }
  void setCollided() { mCollided = true; }
  // The Newton raycast service owns the answer under OMNISIM_NEWTON_RAYCAST:
  // it may both set and CLEAR the flag (the ODE collision pass already ran).
  void setNewtonVerdict(bool collided) { mCollided = collided; }
  const OmRobot *emittingRobot() const { return mEmittingRobot; }
  dGeomID geom() const { return mGeom; }
  // Native segment accessors (the ODE ray geom used to be the carrier)
  const OmVector3 &segmentStart() const { return mStart; }
  const OmVector3 &segmentDirection() const { return mSegmentDir; }
  double segmentLength() const { return mSegmentLength; }

  void recomputeRayDirection(const OmVector3 &receiverTranslation) {
    // compute ray direction and length
    OmEmitter *e = mPacket->emitter();
    e->updateTransformForPhysicsStep();
    const OmVector3 &te = e->matrix().translation();
    OmVector3 dir = receiverTranslation - te;
    mStart = te;
    mSegmentDir = dir;
    mSegmentLength = dir.length();
  }

private:
  OmDataPacket *mPacket;    // packet being transmitted
  OmRobot *mEmittingRobot;  // robot that emitted the packet
  dGeomID mGeom;            // geom that checks collision of this packet
  bool mCollided;           // the geom has collided yet
  OmVector3 mStart;       // native segment carrier (see ctor)
  OmVector3 mSegmentDir;
  double mSegmentLength;
};

void OmReceiver::init() {
  mNeedToConfigure = false;
  mSensor = NULL;
  mLastWaitingPacketIndex = 0;
  mMediumType = OmDataPacket::UNKNOWN;
  mType = findSFString("type");
  mAperture = findSFDouble("aperture");
  mChannel = findSFInt("channel");
  mBaudRate = findSFInt("baudRate");
  mByteSize = findSFInt("byteSize");
  mBufferSize = findSFInt("bufferSize");
  mSignalStrengthNoise = findSFDouble("signalStrengthNoise");
  mDirectionNoise = findSFDouble("directionNoise");
  mAllowedChannels = findMFInt("allowedChannels");
}

OmReceiver::OmReceiver(OmTokenizer *tokenizer) : OmSolidDevice("Receiver", tokenizer) {
  init();
}

OmReceiver::OmReceiver(const OmReceiver &other) : OmSolidDevice(other) {
  init();
}

OmReceiver::OmReceiver(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmReceiver::~OmReceiver() {
  delete mSensor;

  // remove collision detection pending packets
  qDeleteAll(mTransmissionList);

  // remove unsent packets
  qDeleteAll(mReadyQueue);
  qDeleteAll(mWaitingQueue);
  mLastWaitingPacketIndex = 0;

  // remove myself
  gReceiverList.removeAll(this);
}

void OmReceiver::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();
  updateTransmissionSetup();
  updateAllowedChannels();

  gReceiverList.append(this);  // add myself
}

void OmReceiver::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mChannel, &OmSFInt::changed, this, &OmReceiver::updateTransmissionSetup);
  connect(mType, &OmSFString::changed, this, &OmReceiver::updateTransmissionSetup);
  connect(mAperture, &OmSFDouble::changed, this, &OmReceiver::updateTransmissionSetup);
  connect(mBufferSize, &OmSFInt::changed, this, &OmReceiver::updateTransmissionSetup);
  connect(mBaudRate, &OmSFInt::changed, this, &OmReceiver::updateTransmissionSetup);
  connect(mByteSize, &OmSFInt::changed, this, &OmReceiver::updateTransmissionSetup);
  connect(mSignalStrengthNoise, &OmSFDouble::changed, this, &OmReceiver::updateTransmissionSetup);
  connect(mDirectionNoise, &OmSFDouble::changed, this, &OmReceiver::updateTransmissionSetup);
  connect(mAllowedChannels, &OmMFInt::changed, this, &OmReceiver::updateAllowedChannels);
}

void OmReceiver::updateTransmissionSetup() {
  mMediumType = OmDataPacket::decodeMediumType(mType->value());
  if (mMediumType == OmDataPacket::UNKNOWN) {
    parsingWarn(tr("Unknown 'type': \"%1\".").arg(mType->value()));
    mMediumType = OmDataPacket::RADIO;
  }

  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBoundsAndNotDisabled(this, mAperture, 0, 2 * M_PI, -1, -1);
  OmFieldChecker::resetDoubleIfNegative(this, mSignalStrengthNoise, 0);
  OmFieldChecker::resetDoubleIfNegative(this, mDirectionNoise, 0);
  OmFieldChecker::resetIntIfNonPositiveAndNotDisabled(this, mBufferSize, -1, -1);
  OmFieldChecker::resetIntIfNonPositiveAndNotDisabled(this, mBaudRate, -1, -1);
  OmFieldChecker::resetIntIfLess(this, mByteSize, 8, 8);

  if (!isChannelAllowed()) {
    parsingWarn(tr("'channel' is not included in 'allowedChannels'. Setting 'channel' to %1").arg(mAllowedChannels->item(0)));
    mChannel->setValue(mAllowedChannels->item(0));
  }

  mNeedToConfigure = true;
}

bool OmReceiver::isChannelAllowed() {
  const int allowedChannelsSize = mAllowedChannels->size();
  if (allowedChannelsSize > 0) {
    const int currentChannel = (int)mChannel->value();
    for (int i = 0; i < allowedChannelsSize; i++) {
      if (currentChannel == mAllowedChannels->item(i))
        return true;
    }
    return false;
  }
  return true;
}

void OmReceiver::updateAllowedChannels() {
  if (!isChannelAllowed()) {
    parsingWarn(
      tr("'allowedChannels' does not contain current 'channel'. Setting 'channel' to %1.").arg(mAllowedChannels->item(0)));
    mChannel->setValue(mAllowedChannels->item(0));
  }

  mNeedToConfigure = true;
}

void OmReceiver::writeConfigure(OmDataStream &stream) {
  // TODO disable in remote or not ?
  mSensor->connectToRobotSignal(robot(), false);

  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mBufferSize->value();
  stream << (int)mChannel->value();
  stream << (int)mAllowedChannels->size();
  for (int i = 0; i < mAllowedChannels->size(); i++)
    stream << (int)mAllowedChannels->item(i);
  mNeedToConfigure = false;
}

void OmReceiver::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    for (int i = 0; i < mReadyQueue.size(); i++) {
      OmDataPacket *packet = mReadyQueue[i];
      stream << tag();
      stream << (unsigned char)C_RECEIVER_RECEIVE;
      stream << (int)mChannel->value();
      const OmVector3 emitterDir = packet->emitterDir();
      stream << (double)emitterDir[0];
      stream << (double)emitterDir[1];
      stream << (double)emitterDir[2];
      stream << (double)packet->signalStrength();
      stream << (int)packet->dataSize();
      stream.writeRawData(static_cast<const char *>(packet->data()), packet->dataSize());
      delete packet;
    }
    mReadyQueue.clear();
    mSensor->resetPendingValue();
  }

  if (mNeedToConfigure)
    writeConfigure(stream);
}

void OmReceiver::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD: {
      short rate;
      stream >> rate;
      mSensor->setRefreshRate(rate);
      return;
    }
    case C_RECEIVER_SET_CHANNEL: {
      int receiverChannel;
      stream >> receiverChannel;
      mChannel->setValue(receiverChannel);
      return;
    }
    default:
      assert(0);
  }
}

void OmReceiver::prePhysicsStep(double ms) {
  // Nothing to do before the step: the transmission rays are answered by the
  // Newton raycast service afterwards, against the poses the solver settled on.
  // (This used to subscribe the transmission ray for a mid-step re-aim by ODE's
  // collision pass -- with that pass gone the subscription only grew a list
  // nobody drained.)
  OmSolidDevice::prePhysicsStep(ms);
}

void OmReceiver::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();
  const OmVector3 position = matrix().translation();
  // update receiver position in pending packets
  foreach (Transmission *t, mTransmissionList)
    t->recomputeRayDirection(position);
}

void OmReceiver::refreshTransmissionCollisionsFromNewton() {
  // Kernel blocker #1 (ode-retirement-campaign.md): answer the infra-red
  // emitter->receiver line-of-sight query from the Newton raycast service
  // (mj_ray over the live mjModel) instead of the ODE collision pass. Same
  // contract as OmDistanceSensor::refreshRaysFromNewton: gate off (the
  // default for now) or service unavailable leaves the ODE verdicts intact.
  if (!isNewtonRaycastEnabled() || mTransmissionList.isEmpty())
    return;
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  const OmNewtonBackend *const newton = static_cast<const OmNewtonBackend *>(raw);

  // Hits on the RECEIVING robot never reach rayCollisionCallback on the ODE
  // path (odeNearCallback's same-robot rule, waived by selfCollision) -- cache
  // its exclusion once, like OmDistanceSensor.
  if (mNewtonExcludeBodies.isEmpty()) {
    const OmRobot *const r = robot();
    if (r != nullptr && !r->selfCollision())
      collectRobotNewtonBodies(r, mNewtonExcludeBodies);
    if (mNewtonExcludeBodies.isEmpty())
      mNewtonExcludeBodies.append(-999);  // sentinel: "built, nothing to exclude"
  }
  const bool hasReceiverExcludes = !(mNewtonExcludeBodies.size() == 1 && mNewtonExcludeBodies[0] == -999);

  // Hits on the EMITTING robot are ignored by rayCollisionCallback()
  // unconditionally (regardless of its selfCollision), so the exclude list
  // differs per emitter: one raycastBatch per emitting robot, all of that
  // emitter's pending packets in the same batch.
  QHash<const OmRobot *, QList<Transmission *>> byEmitter;
  foreach (Transmission *t, mTransmissionList)
    byEmitter[t->emittingRobot()].append(t);

  for (auto it = byEmitter.constBegin(); it != byEmitter.constEnd(); ++it) {
    QVector<int> exclude;
    if (hasReceiverExcludes)
      exclude = mNewtonExcludeBodies;
    collectRobotNewtonBodies(it.key(), exclude);

    const QList<Transmission *> &group = it.value();
    const int n = group.size();
    QVector<double> rays;
    rays.reserve(n * 7);
    QVector<double> lengths(n);
    for (int i = 0; i < n; ++i) {
      // ODE is gone: the Transmission carries the segment itself
      const OmVector3 &start = group[i]->segmentStart();
      const OmVector3 &dir = group[i]->segmentDirection();
      lengths[i] = group[i]->segmentLength();
      rays << start.x() << start.y() << start.z() << dir.x() << dir.y() << dir.z() << lengths[i];
    }
    QVector<OmNewtonRayHit> hits(n);
    if (newton->raycastBatch(n, rays.constData(), hits.data(), exclude.isEmpty() ? nullptr : exclude.constData(),
                             exclude.size()) != n)
      return;  // service unavailable this tick -- keep the ODE verdicts
    for (int i = 0; i < n; ++i)
      // blocked iff something sits strictly between emitter and receiver (the
      // epsilon keeps a surface flush against the receiver from counting)
      group[i]->setNewtonVerdict(hits[i].dist >= 0.0 && hits[i].dist < lengths[i] - 1e-6);
  }
}

// This function searches and destroys the information packets that have collided with an obstacle.
// It is called when the collision detection is over, that means
// after the last call to the function: OmReceiver::rayCollisionCallback()
// All the packets that did not collide with an obstacle are appended to the Receiver's buffer.
void OmReceiver::postPhysicsStep() {
  OmSolidDevice::postPhysicsStep();

  if (mMediumType == OmDataPacket::INFRA_RED) {
    // Newton raycast service: overrides the ODE occlusion verdicts under the
    // OMNISIM_NEWTON_RAYCAST gate; no-op otherwise
    refreshTransmissionCollisionsFromNewton();
    // check aperture, range and if communication is blocked by an obstacles
    // forward and delete all pending packets
    mLastWaitingPacketIndex = mWaitingQueue.size();
    OmDataPacket *packet = NULL;
    foreach (Transmission *t, mTransmissionList) {
      packet = t->packet();
      if (packet->emitter() == NULL ||  // emitter-less packet
          (checkApertureAndRange(packet->emitter(), this) && !t->hasCollided()))
        // push to waiting queue: ready for sending to controller
        mWaitingQueue.append(packet);
      else
        delete packet;

      delete t;
    }
    mTransmissionList.clear();

  } else {
    // check aperture and range
    for (int i = (mWaitingQueue.size() - 1); i >= mLastWaitingPacketIndex; --i) {
      OmDataPacket *packet = mWaitingQueue[i];
      if (packet->emitter() != NULL && !checkApertureAndRange(packet->emitter(), this, true)) {
        // remove and delete packet
        mWaitingQueue.removeAll(packet);
        delete packet;
      }
    }
  }

  if (mLastWaitingPacketIndex < mWaitingQueue.size()) {
    const OmMatrix4 &m = matrix();

    // compute emitter direction and signal strength
    for (int i = mLastWaitingPacketIndex; i < mWaitingQueue.size(); ++i) {
      OmDataPacket *packet = mWaitingQueue.at(i);
      if (packet->emitter() == NULL)
        // sent from supervisor
        continue;

      const OmVector3 &t = packet->emitter()->matrix().translation();
      const OmVector3 &emitterPos = m.pseudoInversed(t);

      double dist = emitterPos.length();  // Compute distance from emitter
      // compute the emitter direction vector
      if (mDirectionNoise->value() == 0)  // No noise on direction
        packet->setEmitterDir(emitterPos / dist);
      else {
        OmVector3 emitterNoisedPos(emitterPos[0], emitterPos[1], emitterPos[2]);  // Extraction of the real emitter position
        emitterNoisedPos[0] += dist * mDirectionNoise->value() * OmRandom::nextGaussian();  // Add noise on each components
        emitterNoisedPos[1] += dist * mDirectionNoise->value() * OmRandom::nextGaussian();
        emitterNoisedPos[2] += dist * mDirectionNoise->value() * OmRandom::nextGaussian();
        // Recompute distance from new position in order to have a correct normalization of the vector
        double newDist = emitterNoisedPos.length();
        packet->setEmitterDir(emitterNoisedPos / newDist);  // Send noisy position normalized
      }

      // simulate signal strength from known distance
      if (mSignalStrengthNoise->value() == 0)  // No noise on the signal strength
        packet->setSignalStrength(1.0 / (dist * dist));
      else {
        double signalStrength = 1.0 / (dist * dist);
        signalStrength *= (1 + mSignalStrengthNoise->value() * OmRandom::nextGaussian());  // add noise
        if (signalStrength < 0)
          signalStrength = 0;
        packet->setSignalStrength(signalStrength);  // Send noisy signal strength
      }
    }
  }
}

// if the packet is blocked by an obstacle we can discard it
void OmReceiver::rayCollisionCallback(dGeomID geom, OmSolid *obstacle) {
  foreach (Transmission *t, mTransmissionList) {
    if (t->geom() == geom) {
      // we want to ignore collision of ray with the emitting robot
      // (collision with receiver's robot is already filtered out by odeNearCallback())
      if (!t->hasCollided() && obstacle->robot() != t->emittingRobot())
        t->setCollided();

      return;  // we have found the right packet: no need to look further in the list
    }
  }

  assert(0);  // should never be reached
}

bool OmReceiver::refreshSensorIfNeeded() {
  if (!isPowerOn() || !mSensor->needToRefresh())
    return false;

  mReadyQueue.append(mWaitingQueue);
  mWaitingQueue.clear();
  mLastWaitingPacketIndex = 0;
  mSensor->updateTimer();
  return true;
}

void OmReceiver::reset(const QString &id) {
  OmSolidDevice::reset(id);
  qDeleteAll(mTransmissionList);
  mTransmissionList.clear();
  qDeleteAll(mReadyQueue);
  mReadyQueue.clear();
  qDeleteAll(mWaitingQueue);
  mWaitingQueue.clear();
  mLastWaitingPacketIndex = 0;
}

// return true is a transmission respects the emitter's and receiver's aperture
bool OmReceiver::checkApertureAndRange(const OmEmitter *emitter, const OmReceiver *receiver, bool checkRangeOnly) const {
  OmVector3 eTranslation = emitter->matrix().translation();
  OmVector3 rTranslation = receiver->matrix().translation();

  // out of range ?
  if (emitter->range() != -1.0) {
    double range2 = emitter->range() * emitter->range();
    double distance = (rTranslation - eTranslation).length2();
    if (distance > range2)
      return false;
  }

  if (checkRangeOnly)
    return true;

  // emission: check that receiver is within emitter's cone
  if (emitter->aperture() > 0.0) {
    const OmVector3 e2r = rTranslation - eTranslation;
    const OmVector4 eAxisX4 = emitter->matrix().column(0);
    const OmVector3 eAxisX3(eAxisX4[0], eAxisX4[1], eAxisX4[2]);
    if (eAxisX3.angle(e2r) > emitter->aperture() / 2.0)
      return false;
  }

  // reception: check that emitter is within receiver's cone
  if (receiver->aperture() > 0.0) {
    const OmVector3 r2e = eTranslation - rTranslation;
    const OmVector4 rAxisX4 = receiver->matrix().column(0);
    const OmVector3 rAxisX3(rAxisX4[0], rAxisX4[1], rAxisX4[2]);
    if (rAxisX3.angle(r2e) > receiver->aperture() / 2.0)
      return false;
  }

  return true;
}

void OmReceiver::transmitPacket(OmDataPacket *packet) {
  const OmEmitter *e = packet->emitter();
  // look for emitting robot
  const OmRobot *eRobot = e->robot();
  assert(eRobot);

  // loop through emitters
  for (int i = 0; i < gReceiverList.size(); i++) {
    OmReceiver *r = gReceiverList.at(i);

    // was the receiver enabled ?
    if (r->mSensor->isEnabled()) {
      // media types must match
      if (e->mediumType() == r->mediumType()) {
        // channel numbers must match
        if ((packet->channel() == r->channel()) || (packet->channel() == WB_CHANNEL_BROADCAST) ||
            (r->channel() == WB_CHANNEL_BROADCAST)) {
          // look for receiving robot
          const OmRobot *rRobot = r->robot();
          assert(rRobot);

          // robot cannot send message to self
          if (eRobot != rRobot) {
            // aperture will be checked in postPhysicsStep
            // we check collision only for infra-red
            if (r->mediumType() != OmDataPacket::INFRA_RED)
              // push to waiting queue: ready for sending to controller
              // range will be checked in postPhysicsStep
              r->mWaitingQueue.append(new OmDataPacket(*packet));
            else
              // register for collision detection (append)
              r->mTransmissionList.append(new Transmission(new OmDataPacket(*packet), r, OmOdeContext::instance()->space()));
          }
        }
      }
    }
  }

  // we are done with this packet: let's destroy it !
  delete packet;
}
