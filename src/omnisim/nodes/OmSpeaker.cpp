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

#include "OmSpeaker.hpp"

#include "OmDataStream.hpp"
#include "OmNodeUtilities.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmRobot.hpp"
#include "OmSimulationState.hpp"
#include "OmSoundClip.hpp"
#include "OmSoundEngine.hpp"
#include "OmSoundSource.hpp"
#include "OmStandardPaths.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QBuffer>
#include <QtCore/QDataStream>
#include <QtCore/QDir>

#include <cassert>

#define TEXT_TO_SPEECH_KEY "WB_TEXT_TO_SPEECH"

void OmSpeaker::init() {
  mControllerDir = "";
  mSoundSourcesMap.clear();
  mPlayingSoundSourcesMap.clear();
  mEngine = QString("microsoft");
  mLanguage = QString("en-US");
}

OmSpeaker::OmSpeaker(OmTokenizer *tokenizer) : OmSolidDevice("Speaker", tokenizer) {
  init();
}

OmSpeaker::OmSpeaker(const OmSpeaker &other) : OmSolidDevice(other) {
  init();
}

OmSpeaker::OmSpeaker(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmSpeaker::~OmSpeaker() {
  foreach (OmSoundSource *source, mSoundSourcesMap) {
    if (source)
      OmSoundEngine::deleteSource(source);
  }
  mSoundSourcesMap.clear();
  mPlayingSoundSourcesMap.clear();

  foreach (QByteArray *buffer, mStreamedSoundDataMap)
    delete buffer;
  mStreamedSoundDataMap.clear();
}

void OmSpeaker::postFinalize() {
  OmSolidDevice::postFinalize();
  OmRobot *robot = OmNodeUtilities::findRobotAncestor(this);
  if (robot)
    mControllerDir = robot->controllerDir();
}

void OmSpeaker::handleMessage(QDataStream &stream) {
  unsigned char command;

  stream >> command;
  switch (command) {
    case C_SPEAKER_PLAY_SOUND: {
      int numberOfSound = 0;
      stream >> numberOfSound;
      for (int i = 0; i < numberOfSound; ++i) {
        SoundPlayData playData(stream);
        playSound(playData);
      }
      return;
    }
    case C_SPEAKER_STOP: {
      short numberOfSound = 0;
      stream >> numberOfSound;
      // cppcheck-suppress knownConditionTrueFalse
      if (numberOfSound == 0)
        stopAll();
      else {
        for (int i = 0; i < numberOfSound; ++i) {
          short size;
          stream >> size;
          char sound[size];
          stream.readRawData(sound, size);
          stop(sound);
        }
      }
      return;
    }
    case C_SPEAKER_SET_ENGINE: {
      short size;
      stream >> size;
      char engine[size];
      stream.readRawData(engine, size);
      mEngine = QString(engine);
      return;
    }
    case C_SPEAKER_SET_LANGUAGE: {
      short size;
      stream >> size;
      char language[size];
      stream.readRawData(language, size);
      mLanguage = QString(language);
      return;
    }
    case C_SPEAKER_SPEAK: {
      short size;
      double volume;
      stream >> size;
      char text[size];
      stream.readRawData(text, size);
      stream >> volume;
      playText(text, volume);
      return;
    }
    default:
      assert(0);
  }
}

void OmSpeaker::writeAnswer(OmDataStream &stream) {
  foreach (const OmSoundSource *source, mPlayingSoundSourcesMap) {
    if (!source->isPlaying()) {
      if (mPlayingSoundSourcesMap.key(source) == TEXT_TO_SPEECH_KEY) {
        stream << (short unsigned int)tag();
        stream << (unsigned char)C_SPEAKER_SPEAK_OVER;
      } else {
        stream << (short unsigned int)tag();
        stream << (unsigned char)C_SPEAKER_SOUND_OVER;
        const QByteArray name = mPlayingSoundSourcesMap.key(source).toUtf8();
        stream.writeRawData(name.constData(), name.size() + 1);
      }
      mPlayingSoundSourcesMap.remove(mPlayingSoundSourcesMap.key(source));
    }
  }
}

void OmSpeaker::postPhysicsStep() {
  OmSolidDevice::postPhysicsStep();
  foreach (OmSoundSource *source, mSoundSourcesMap) {
    if (source)
      updateSoundSource(source);
  }
}

void OmSpeaker::playText(const char *text, double volume) {
  if (!mSoundSourcesMap.contains(TEXT_TO_SPEECH_KEY)) {  // text-to-speech was never used
    OmSoundSource *source = OmSoundEngine::createSource();
    updateSoundSource(source);
    mSoundSourcesMap[TEXT_TO_SPEECH_KEY] = source;
  }
  OmSoundSource *source = mSoundSourcesMap.value(TEXT_TO_SPEECH_KEY, NULL);
  if (source) {
    source->stop();
    // cd to the controller directory (because some tags in the text may refeer to file relatively to the controller)
    QDir initialDir = QDir::current();
    if (!QDir::setCurrent(mControllerDir))
      this->warn(tr("Cannot change directory to: '%1'").arg(mControllerDir));
    const OmSoundClip *soundClip = OmSoundEngine::soundFromText(text, mEngine, mLanguage);
    QDir::setCurrent(initialDir.path());
    if (soundClip) {
      source->setSoundClip(soundClip);
      source->play();
      source->setGain(volume);
      mPlayingSoundSourcesMap[TEXT_TO_SPEECH_KEY] = source;
    }
  }
}

void OmSpeaker::playSound(SoundPlayData &playData) {
  const QString filename(playData.file());
  QString key(filename);
  if (playData.side() == -1)
    key += "_left";
  else if (playData.side() == 1)
    key += "_right";

  // Controller streamed sound data available ?
  if (playData.rawLength() && !mStreamedSoundDataMap.contains(filename))
    mStreamedSoundDataMap[filename] = new QByteArray(playData.rawData());
  QByteArray *cachedSoundData = NULL;
  if (mStreamedSoundDataMap.contains(key))
    cachedSoundData = mStreamedSoundDataMap[key];

  if (!mSoundSourcesMap.contains(key)) {  // this sound was never played
    QString path;
    // check if sound data was streamed from controller
    if (cachedSoundData)
      path = filename;
    // check if the path is absolute
    if (path.isEmpty() && QDir::isAbsolutePath(filename) && QFile::exists(filename))
      path = filename;
    // check if the path is relative to the controller
    if (path.isEmpty() && QFile::exists(mControllerDir + filename))
      path = mControllerDir + filename;
    // check default location for vehicle sounds
    if (path.isEmpty() && QFile::exists(OmStandardPaths::vehicleLibraryPath() + filename))
      path = OmStandardPaths::vehicleLibraryPath() + filename;
    if (path.isEmpty())
      this->warn(
        tr("Sound file '%1' not found. The sound file should be defined relatively to the controller, or absolutely.\n")
          .arg(filename));

    OmSoundSource *source = OmSoundEngine::createSource();
    updateSoundSource(source);
    const QString extension = filename.mid(filename.lastIndexOf('.') + 1).toLower();

    // Use controller streamed sound data if available.
    QBuffer *device = NULL;
    if (cachedSoundData) {
      device = new QBuffer(cachedSoundData);
      device->open(QBuffer::ReadOnly);
    }

    const OmSoundClip *soundClip = OmSoundEngine::sound(path, extension, device, playData.balance(), playData.side());
    delete device;
    device = NULL;

    if (!soundClip) {
      this->warn(tr("Impossible to play '%1'. Make sure the file format is supported (8 or 16 bits, mono or stereo wave).\n")
                   .arg(filename));
      return;
    }
    source->setSoundClip(soundClip);
    source->play();
    mSoundSourcesMap[key] = source;
  }

  OmSoundSource *source = mSoundSourcesMap.value(key, NULL);
  if (source) {
    mPlayingSoundSourcesMap[key] = source;
    if (!source->isPlaying())  // this sound was already played but is over
      source->play();
    source->setLooping(playData.loop());
    source->setGain(playData.volume());
    source->setPitch(playData.pitch());
    if (OmSimulationState::instance()->isPaused() || OmSimulationState::instance()->isStep())
      source->pause();
  }
}

void OmSpeaker::stop(const char *sound) {
  QString key = QString(sound);
  OmSoundSource *source = mSoundSourcesMap.value(key, NULL);
  if (!source) {
    key = QString(sound) + "_left";
    source = mSoundSourcesMap.value(key, NULL);
  }
  if (!source) {
    key = QString(sound) + "_right";
    source = mSoundSourcesMap.value(key, NULL);
  }
  if (source) {
    source->stop();
    OmSoundEngine::deleteSource(source);
    mSoundSourcesMap.remove(key);
  }
  mPlayingSoundSourcesMap.remove(key);
}

void OmSpeaker::stopAll() {
  foreach (OmSoundSource *source, mSoundSourcesMap) {
    if (source) {
      source->stop();
      OmSoundEngine::deleteSource(source);
    }
  }
  mSoundSourcesMap.clear();
  mPlayingSoundSourcesMap.clear();
}

void OmSpeaker::updateSoundSource(OmSoundSource *source) {
  source->setPosition(position());
  source->setVelocity(linearVelocity());
  source->setDirection(rotationMatrix() * OmVector3(0, 1, 0));
}

OmSpeaker::SoundPlayData::SoundPlayData(QDataStream &stream) : mFile(), mRawData() {
  short size;
  unsigned char loopByte;
  stream >> size;
  char soundFile[size];
  stream.readRawData(soundFile, size);
  mFile = QString(soundFile);

  stream >> mVolume;
  stream >> mPitch;
  stream >> mBalance;
  stream >> mSide;
  stream >> loopByte;
  mLoop = (bool)loopByte;
  stream >> mRawLength;
  if (mRawLength) {
    mRawData.resize(mRawLength);
    stream.readRawData(mRawData.data(), mRawLength);
  }
}
