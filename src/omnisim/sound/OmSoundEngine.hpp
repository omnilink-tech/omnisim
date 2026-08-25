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

#ifndef OM_SOUND_ENGINE_HPP
#define OM_SOUND_ENGINE_HPP

//
// Description: manage sounds
//

class QString;
class QIODevice;

class OmWorld;
class OmSoundClip;
class OmSoundSource;

namespace OmSoundEngine {
  bool openAL();
  const QString &device();
  void setWorld(OmWorld *world);
  void setMute(bool mute);
  void setVolume(int volume);
  void setPause(bool pause);
  void updateAfterPhysicsStep();
  void updateListener();
  OmSoundSource *createSource();
  void deleteSource(OmSoundSource *);
  void stopAllSources();
  // 0: both sides, -1: left only, 1: right only
  OmSoundClip *sound(const QString &url, const QString &extension, QIODevice *device = 0, double balance = 0.0, int side = 0);
  OmSoundClip *soundFromText(const QString &text, const QString &engine, const QString &language);
  void clearAllMotorSoundSources();
  void clearAllContactSoundSources();
  void updateViewpointConnection();

};  // namespace OmSoundEngine

#endif
