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

#ifndef OM_SPEAKER_HPP
#define OM_SPEAKER_HPP

#include "OmSolidDevice.hpp"

#include <QtCore/QMap>

class OmSoundSource;
class QDataStream;

class OmSpeaker : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmSpeaker(OmTokenizer *tokenizer = NULL);
  OmSpeaker(const OmSpeaker &other);
  explicit OmSpeaker(const OmNode &other);
  virtual ~OmSpeaker() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SPEAKER; }
  void postFinalize() override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(OmDataStream &) override;
  void postPhysicsStep() override;

private:
  // private data types

  // Data from wb_speaker_play_sound() API
  class SoundPlayData {
  public:
    explicit SoundPlayData(QDataStream &);

    const QString &file() const { return mFile; }
    double volume() const { return mVolume; }
    double pitch() const { return mPitch; }
    double balance() const { return mBalance; }
    bool loop() const { return mLoop; }
    short side() const { return mSide; }
    int rawLength() const { return mRawLength; }
    QByteArray &rawData() { return mRawData; }

  private:
    QString mFile;
    double mVolume;
    double mPitch;
    double mBalance;
    bool mLoop;
    short mSide;
    int mRawLength;
    QByteArray mRawData;
  };

  // private functions
  OmSpeaker &operator=(const OmSpeaker &);  // non copyable
  OmNode *clone() const override { return new OmSpeaker(*this); }
  void init();

  void playSound(SoundPlayData &playData);
  void playText(const char *text, double volume);
  void stop(const char *sound);
  void stopAll();

  void updateSoundSource(OmSoundSource *source);

  QMap<QString, OmSoundSource *> mSoundSourcesMap;
  QMap<QString, const OmSoundSource *> mPlayingSoundSourcesMap;
  QMap<QString, QByteArray *> mStreamedSoundDataMap;

  QString mControllerDir;
  QString mEngine;
  QString mLanguage;
};

#endif
