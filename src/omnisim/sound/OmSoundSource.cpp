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

#include "OmSoundSource.hpp"

#include "OmSoundClip.hpp"
#include "OmSoundEngine.hpp"
#include "OmVector3.hpp"

#ifdef __APPLE__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <OpenAL/al.h>
#else
#include <AL/al.h>
#endif

OmSoundSource::OmSoundSource() {
  if (!OmSoundEngine::openAL())
    return;
  alGenSources(1, &mSource);
  alSourcei(mSource, AL_LOOPING, AL_FALSE);
}

OmSoundSource::~OmSoundSource() {
  if (!OmSoundEngine::openAL())
    return;
  alSourceStop(mSource);
  alDeleteSources(1, &mSource);
}

bool OmSoundSource::isPlaying() const {
  if (!OmSoundEngine::openAL())
    return false;
  ALenum source_state;
  alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
  return (source_state == AL_PLAYING);
}

bool OmSoundSource::isStopped() const {
  if (!OmSoundEngine::openAL())
    return false;
  ALenum source_state;
  alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
  return (source_state == AL_STOPPED);
}

bool OmSoundSource::isPaused() const {
  if (!OmSoundEngine::openAL())
    return false;
  ALenum source_state;
  alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
  return (source_state == AL_PAUSED);
}

void OmSoundSource::play() {
  if (!OmSoundEngine::openAL())
    return;
  alSourcePlay(mSource);
}

void OmSoundSource::stop() {
  if (!OmSoundEngine::openAL())
    return;
  alSourceStop(mSource);
}

void OmSoundSource::pause() {
  if (!OmSoundEngine::openAL())
    return;
  alSourcePause(mSource);
}

void OmSoundSource::setLooping(bool loop) {
  if (!OmSoundEngine::openAL())
    return;
  if (loop)
    alSourcei(mSource, AL_LOOPING, AL_TRUE);
  else
    alSourcei(mSource, AL_LOOPING, AL_FALSE);
}

void OmSoundSource::setSoundClip(const OmSoundClip *clip) {
  if (!OmSoundEngine::openAL())
    return;
  alSourcei(mSource, AL_BUFFER, clip->openALBuffer());
}

void OmSoundSource::setPitch(double pitch) {
  if (!OmSoundEngine::openAL())
    return;
  alSourcef(mSource, AL_PITCH, pitch);
}

void OmSoundSource::setGain(double gain) {
  if (!OmSoundEngine::openAL())
    return;
  alSourcef(mSource, AL_GAIN, gain);
}

void OmSoundSource::setPosition(const OmVector3 &pos) {
  if (!OmSoundEngine::openAL())
    return;
  alSource3f(mSource, AL_POSITION, pos.x(), pos.y(), pos.z());
}

void OmSoundSource::setVelocity(const OmVector3 &v) {
  if (!OmSoundEngine::openAL())
    return;
  alSource3f(mSource, AL_VELOCITY, v.x(), v.y(), v.z());
}

void OmSoundSource::setDirection(const OmVector3 &dir) {
  if (!OmSoundEngine::openAL())
    return;
  alSource3f(mSource, AL_DIRECTION, dir.x(), dir.y(), dir.z());
}

#ifdef __APPLE__
#pragma GCC diagnostic pop
#endif
