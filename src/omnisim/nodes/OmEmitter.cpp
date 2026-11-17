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

#include "OmEmitter.hpp"

#include "OmDataPacket.hpp"
#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmReceiver.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>

#include <cassert>

void OmEmitter::init() {
  mType = findSFString("type");
  mRange = findSFDouble("range");
  mMaxRange = findSFDouble("maxRange");
  mAperture = findSFDouble("aperture");
  mChannel = findSFInt("channel");
  mBaudRate = findSFInt("baudRate");
  mByteSize = findSFInt("byteSize");
  mBufferSize = findSFInt("bufferSize");
  mAllowedChannels = findMFInt("allowedChannels");
  mNeedToSetRange = false;
  mNeedToSetChannel = false;
  mNeedToSetBufferSize = false;
  mNeedToSetAllowedChannels = false;
  mMediumType = OmDataPacket::UNKNOWN;
  mByteRate = -1.0;
}

OmEmitter::OmEmitter(OmTokenizer *tokenizer) : OmSolidDevice("Emitter", tokenizer) {
  init();
}

OmEmitter::OmEmitter(const OmEmitter &other) : OmSolidDevice(other) {
  init();
}

OmEmitter::OmEmitter(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmEmitter::~OmEmitter() {
  qDeleteAll(mQueue);
}

void OmEmitter::preFinalize() {
  OmSolidDevice::preFinalize();

  updateTransmissionSetup();
  updateAllowedChannels();
}

void OmEmitter::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mType, &OmSFString::changed, this, &OmEmitter::updateTransmissionSetup);
  connect(mRange, &OmSFDouble::changed, this, &OmEmitter::updateRange);
  connect(mMaxRange, &OmSFDouble::changed, this, &OmEmitter::updateTransmissionSetup);
  connect(mAperture, &OmSFDouble::changed, this, &OmEmitter::updateTransmissionSetup);
  connect(mChannel, &OmSFInt::changed, this, &OmEmitter::updateChannel);
  connect(mBaudRate, &OmSFInt::changed, this, &OmEmitter::updateTransmissionSetup);
  connect(mByteSize, &OmSFInt::changed, this, &OmEmitter::updateTransmissionSetup);
  connect(mBufferSize, &OmSFInt::changed, this, &OmEmitter::updateBufferSize);
  connect(mAllowedChannels, &OmMFInt::changed, this, &OmEmitter::updateAllowedChannels);
}

void OmEmitter::updateTransmissionSetup() {
  if (mBaudRate->value() < 0)
    mByteRate = -1.0;
  else
    mByteRate = (double)mBaudRate->value() / (double)mByteSize->value() / 1000.0;

  mMediumType = OmDataPacket::decodeMediumType(mType->value());
  if (mMediumType == OmDataPacket::UNKNOWN) {
    parsingWarn(tr("Unknown 'type': \"%1\".").arg(mType->value()));
    mMediumType = OmDataPacket::RADIO;
  }

  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBoundsAndNotDisabled(this, mAperture, 0, 2 * M_PI, -1, -1);
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mMaxRange, -1, -1);
  OmFieldChecker::resetIntIfLess(this, mByteSize, 8, 8);
}

void OmEmitter::updateBufferSize() {
  OmFieldChecker::resetIntIfNonPositiveAndNotDisabled(this, mBufferSize, -1, -1);
  mNeedToSetBufferSize = true;
}
void OmEmitter::updateRange() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mRange, -1, -1);
  if (mMaxRange->value() != -1.0 && (mRange->value() > mMaxRange->value() || mRange->value() == -1.0)) {
    parsingWarn(tr("'range' must be less than or equal to 'maxRange'."));
    mRange->setValue(mMaxRange->value());
  }
  mNeedToSetRange = true;
}

bool OmEmitter::isChannelAllowed() {
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

void OmEmitter::updateAllowedChannels() {
  if (!isChannelAllowed()) {
    parsingWarn(
      tr("'allowedChannels' does not contain current 'channel'. Setting 'channel' to %1.").arg(mAllowedChannels->item(0)));
    mChannel->setValue(mAllowedChannels->item(0));
  }

  mNeedToSetAllowedChannels = true;
}

void OmEmitter::updateChannel() {
  if (!isChannelAllowed()) {
    parsingWarn(tr("'channel' is not included in 'allowedChannels'. Setting 'channel' to %1").arg(mAllowedChannels->item(0)));
    mChannel->setValue(mAllowedChannels->item(0));
  }

  mNeedToSetChannel = true;
}

void OmEmitter::writeConfigure(OmDataStream &stream) {
  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mBufferSize->value();
  stream << (int)mChannel->value();
  stream << (double)mByteRate;
  stream << (double)mRange->value();
  stream << (double)mMaxRange->value();
  stream << (int)mAllowedChannels->size();
  for (int i = 0; i < mAllowedChannels->size(); i++)
    stream << (int)mAllowedChannels->item(i);

  mNeedToSetRange = false;
  mNeedToSetChannel = false;
  mNeedToSetBufferSize = false;
  mNeedToSetAllowedChannels = false;
}

void OmEmitter::writeAnswer(OmDataStream &stream) {
  if (mNeedToSetRange) {
    stream << tag();
    stream << (unsigned char)C_EMITTER_SET_RANGE;
    stream << (double)mRange->value();
    mNeedToSetRange = false;
  }
  if (mNeedToSetChannel) {
    stream << tag();
    stream << (unsigned char)C_EMITTER_SET_CHANNEL;
    stream << (int)mChannel->value();
    mNeedToSetChannel = false;
  }
  if (mNeedToSetBufferSize) {
    stream << tag();
    stream << (unsigned char)C_EMITTER_SET_BUFFER_SIZE;
    stream << (int)mBufferSize->value();
    mNeedToSetBufferSize = false;
  }
  if (mNeedToSetAllowedChannels) {
    stream << tag();
    stream << (unsigned char)C_EMITTER_SET_ALLOWED_CHANNELS;
    stream << (int)mAllowedChannels->size();
    for (int i = 0; i < mAllowedChannels->size(); i++)
      stream << (int)mAllowedChannels->item(i);
  }
}

void OmEmitter::handleMessage(QDataStream &stream) {
  unsigned char command;
  unsigned int size;
  int newChannel;
  double newRange;
  char *data;

  stream >> command;
  switch (command) {
    case C_EMITTER_SEND:
      stream >> newChannel;
      mChannel->setValue(newChannel);
      stream >> newRange;
      mRange->setValue(newRange);
      stream >> size;
      data = new char[size];
      stream.readRawData(data, size);
      mQueue.enqueue(new OmDataPacket(this, mChannel->value(), data, size));
      delete[] data;
      return;

    case C_EMITTER_SET_CHANNEL:
      stream >> newChannel;
      mChannel->setValue(newChannel);
      return;

    case C_EMITTER_SET_RANGE:
      stream >> newRange;
      mRange->setValue(newRange);
      return;

    default:
      assert(0);
  }
}

void OmEmitter::prePhysicsStep(double ms) {
  OmSolidDevice::prePhysicsStep(ms);

  // forward queued packets to receivers
  while (!mQueue.isEmpty()) {
    OmDataPacket *packet = mQueue.takeFirst();
    OmReceiver::transmitPacket(packet);
  }
}

void OmEmitter::reset(const QString &id) {
  OmSolidDevice::reset(id);
  qDeleteAll(mQueue);
  mQueue.clear();
}
