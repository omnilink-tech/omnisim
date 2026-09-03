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

#ifndef OM_RECEIVER_HPP
#define OM_RECEIVER_HPP

#include "OmMFInt.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSolidDevice.hpp"

#include <QtCore/QQueue>

class OmDataPacket;
class OmEmitter;
class OmSensor;
class Transmission;

class OmReceiver : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmReceiver(OmTokenizer *tokenizer = NULL);
  OmReceiver(const OmReceiver &other);
  explicit OmReceiver(const OmNode &other);
  virtual ~OmReceiver() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_RECEIVER; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;

  static void transmitPacket(OmDataPacket *packet);


  // field accessors
  double aperture() const { return mAperture->value(); }
  int channel() const { return mChannel->value(); }
  int mediumType() const { return mMediumType; }

private:
  int mMediumType;                          // UNKNOWN, RADIO, SERIAL or INFRA_RED
  QList<Transmission *> mTransmissionList;  // packets waiting for potential collision detection callback
  QList<OmDataPacket *> mWaitingQueue;      // queue of packets received since last sensor update
  QList<OmDataPacket *> mReadyQueue;        // queue of packets received before last sensor update
  OmSensor *mSensor;
  int mLastWaitingPacketIndex;

  // user accessible fields
  OmSFString *mType;
  OmSFDouble *mAperture;
  OmSFInt *mChannel;
  OmSFInt *mBaudRate;
  OmSFInt *mByteSize;
  OmSFInt *mBufferSize;
  OmSFDouble *mSignalStrengthNoise;
  OmSFDouble *mDirectionNoise;
  OmMFInt *mAllowedChannels;

  bool mNeedToConfigure;

  // private functions
  OmReceiver &operator=(const OmReceiver &);  // non copyable
  OmNode *clone() const override { return new OmReceiver(*this); }
  void init();
  // Newton raycast service consumer: re-answer every pending infra-red
  // transmission's emitter->receiver occlusion ray from the live mjModel via
  // OmNewtonBackend::raycastBatch when OMNISIM_NEWTON_RAYCAST is enabled
  // (value-parsed; default off until the parity suite covers all consumers).
  // No-op when the gate is off or the Newton runtime is not running.
  void refreshTransmissionCollisionsFromNewton();
  QVector<int> mNewtonExcludeBodies;  // cached own-robot Newton bodies ([-999] = built, empty)
  void receivePacketIfPossible(OmDataPacket *packet);
  bool checkApertureAndRange(const OmEmitter *emitter, const OmReceiver *receiver, bool checkRangeOnly = false) const;
  void updateRaysSetupIfNeeded() override;
  bool isChannelAllowed();

private slots:
  void updateTransmissionSetup();
  void updateAllowedChannels();
};

#endif  // OM_RECEIVER_HPP
