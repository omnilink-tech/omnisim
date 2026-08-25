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

#ifndef OM_EMITTER_HPP
#define OM_EMITTER_HPP

#include "OmMFInt.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSolidDevice.hpp"

#include <QtCore/QQueue>

class OmDataPacket;

class OmEmitter : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmEmitter(OmTokenizer *tokenizer = NULL);
  OmEmitter(const OmEmitter &other);
  explicit OmEmitter(const OmNode &other);
  virtual ~OmEmitter() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_EMITTER; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void prePhysicsStep(double ms) override;
  void reset(const QString &id) override;

  // field accessors
  int channel() const { return mChannel->value(); }
  double range() const { return mRange->value(); }
  double aperture() const { return mAperture->value(); }
  int mediumType() const { return mMediumType; }

private:
  int mMediumType;                // UNKNOWN, RADIO, SERIAL or INFRA_RED
  double mByteRate;               // expressed in bytes per millisecond;
  QQueue<OmDataPacket *> mQueue;  // emission queue
  bool mNeedToSetRange;
  bool mNeedToSetChannel;
  bool mNeedToSetBufferSize;
  bool mNeedToSetAllowedChannels;

  // user accessible fields
  OmSFString *mType;
  OmSFDouble *mRange;
  OmSFDouble *mMaxRange;
  OmSFDouble *mAperture;
  OmSFInt *mChannel;
  OmSFInt *mBaudRate;
  OmSFInt *mByteSize;
  OmSFInt *mBufferSize;
  OmMFInt *mAllowedChannels;

  // private functions
  OmEmitter &operator=(const OmEmitter &);  // non copyable
  OmNode *clone() const override { return new OmEmitter(*this); }
  void init();
  bool isChannelAllowed();

private slots:
  void updateTransmissionSetup();
  void updateBufferSize();
  void updateRange();
  void updateChannel();
  void updateAllowedChannels();
};

#endif  // OM_EMITTER_HPP
