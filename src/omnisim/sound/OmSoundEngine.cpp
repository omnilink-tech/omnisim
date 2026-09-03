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

#include "OmSoundEngine.hpp"
#include "../physics/OmPhysicsBackend.hpp"

#include "OmLog.hpp"
#include "OmPreferences.hpp"
#include "OmSFRotation.hpp"
#include "OmTextToSpeech.hpp"
#ifdef _WIN32
#include "OmMicrosoftTextToSpeech.hpp"
#endif
#include "OmMotorSoundManager.hpp"
#include "OmSoundClip.hpp"
#include "OmSoundSource.hpp"
#include "OmStandardPaths.hpp"
#include "OmSysInfo.hpp"
#include "OmViewpoint.hpp"
#include "OmWaveFile.hpp"
#include "OmWorld.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QFileInfo>
#include <QtCore/QObject>
#include <QtCore/QTemporaryFile>

#ifdef __APPLE__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#else
#include <AL/al.h>
#include <AL/alc.h>
#endif

#include <cassert>
#include <cstring>

static OmWorld *gWorld = NULL;
static bool gOpenAL = false;
static bool gMute = true;
static int gVolume = 80;
static QString gDevice;
static ALCdevice *gDefaultDevice = NULL;
static ALCcontext *gContext = NULL;
static QList<OmSoundClip *> gSounds;
static QList<OmSoundSource *> gSources;
static OmTextToSpeech *gTextToSpeech = NULL;

static void clearSounds() {
  qDeleteAll(gSounds);
  gSounds.clear();
}

static void clearSources() {
  qDeleteAll(gSources);
  gSources.clear();
}

static void cleanup() {
  clearSounds();
  clearSources();
  alcMakeContextCurrent(NULL);
  if (gContext)
    alcDestroyContext(gContext);
  if (gDefaultDevice)
    alcCloseDevice(gDefaultDevice);
  delete gTextToSpeech;
  gTextToSpeech = NULL;
}

static void init() {
  static bool initialized = false;
  if (initialized)  // init was already done
    return;
  initialized = true;
  gMute = OmPreferences::instance()->value("Sound/mute", true).toBool();
  gVolume = OmPreferences::instance()->value("Sound/volume", 80).toInt();
  OmLog::toggle(stderr);  // we want to disable stderr to avoid warnings in the console
  try {
    const ALCchar *defaultDeviceName = alcGetString(NULL, ALC_DEFAULT_DEVICE_SPECIFIER);
    if (defaultDeviceName == NULL || strcmp(defaultDeviceName, "Null Audio Device") == 0)
      throw QObject::tr("Cannot find OpenAL default device");
    gDefaultDevice = alcOpenDevice(defaultDeviceName);
    if (gDefaultDevice == NULL)
      throw QObject::tr("Cannot initialize OpenAL default device '%1'").arg(defaultDeviceName);
    gContext = alcCreateContext(gDefaultDevice, NULL);
    if (gContext == NULL)
      throw QObject::tr("Cannot create OpenAL context");
    if (alcMakeContextCurrent(gContext) == ALC_FALSE)
      throw QObject::tr("Cannot make OpenAL current context");
    gDevice = QString(defaultDeviceName);
  } catch (const QString &e) {
    OmLog::toggle(stderr);
    if (OmSysInfo::environmentVariable("CI").isEmpty())
      OmLog::warning(QObject::tr("Cannot initialize the sound engine: %1").arg(e));
    return;
  }
  OmLog::toggle(stderr);
  gOpenAL = true;
  qAddPostRoutine(cleanup);
  OmSoundEngine::updateListener();
}

const QString &OmSoundEngine::device() {
  init();
  return (const QString &)gDevice;
}

bool OmSoundEngine::openAL() {
  init();
  return gOpenAL;
}

void OmSoundEngine::setWorld(OmWorld *world) {
  if (world) {
    gWorld = world;
    QObject::connect(gWorld, &OmWorld::viewpointChanged, &OmSoundEngine::updateViewpointConnection);

    updateListener();
  } else {
    gWorld = NULL;
    OmMotorSoundManager::clearAllMotorSoundSources();
    clearSources();
    clearSounds();
  }
}

void OmSoundEngine::updateViewpointConnection() {
  if (gWorld && gWorld->viewpoint())
    QObject::connect(gWorld->viewpoint(), &OmViewpoint::cameraParametersChanged, &OmSoundEngine::updateListener);
}

void OmSoundEngine::setMute(bool mute) {
  gMute = mute;
  updateListener();
}

void OmSoundEngine::setVolume(int volume) {
  gVolume = volume;
  updateListener();
}

void OmSoundEngine::setPause(bool pause) {
  if (gDefaultDevice) {
    foreach (OmSoundSource *source, gSources) {
      if (pause) {
        if (source->isPlaying())
          source->pause();
      } else {
        if (source->isPaused())
          source->play();
      }
    }
  }
}

void OmSoundEngine::updateListener() {
#ifdef __APPLE__  // macOS bug described at https://developer.apple.com/forums/thread/104309
  // It affects only Apple OpenAL, not OpenAL soft:
  // alListenerf(AL_GAIN, 0) doesn't work, it should be replaced with alListenerf(AL_GAIN, 0.0001f)
  alListenerf(AL_GAIN, (gMute || gVolume == 0) ? 0.0001f : 0.01f * gVolume);
#else
  alListenerf(AL_GAIN, gMute ? 0.0f : 0.01f * gVolume);
#endif
  if (gMute || gVolume == 0)
    return;

  // else update listener position and orientation
  ALfloat orientation[6] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f};
  ALfloat position[3] = {0.0f, 0.0f, 0.0f};

  if (gWorld && gWorld->viewpoint()) {
    const OmVector3 &translation = gWorld->viewpoint()->position()->value();
    const OmRotation &rotation = gWorld->viewpoint()->orientation()->value();
    OmVector3 rotationAt = rotation.direction();
    OmVector3 rotationUp = rotation.up();

    position[0] = translation.x();
    position[1] = translation.y();
    position[2] = translation.z();

    orientation[0] = rotationAt.x();
    orientation[1] = rotationAt.y();
    orientation[2] = rotationAt.z();
    orientation[3] = rotationUp.x();
    orientation[4] = rotationUp.y();
    orientation[5] = rotationUp.z();
  }

  // qDebug() << "OpenAL listener position:" << position[0] << position[1] << position[2];
  // qDebug() << "OpenAL listener orientation at:" << orientation[0] << orientation[1] << orientation[2];
  // qDebug() << "OpenAL listener orientation up:" << orientation[3] << orientation[4] << orientation[5];

  alListener3f(AL_POSITION, position[0], position[1], position[2]);
  alListenerfv(AL_ORIENTATION, orientation);

  // viewpoint velocity is not taken into account because not smooth
  // it may however be interesting to set it when following a robot.
  alListener3f(AL_VELOCITY, 0.0f, 0.0f, 0.0f);
}

void OmSoundEngine::updateAfterPhysicsStep() {
  if (gMute || gVolume == 0)
    return;
  assert(gWorld);

  // CONTACT SOUND IS NOT IMPLEMENTED ON NEWTON, and has not been since Newton
  // became the default. The old subsystem (OmContactSound / OmContactSoundManager)
  // was keyed on ODE geom ids and fed by the ODE near-callback; it was deleted
  // with the ODE stub layer on 2026-09-02. Restoring it means keying contact
  // sounds on Newton body pairs and deriving energy from OmNewtonContact::
  // forceMag -- feature work, tracked separately. Say so once rather than
  // leaving users to wonder why collisions are mute.
  {
    static bool warned = false;
    if (!warned && OmPhysicsBackendRegistry::newtonBackend() != NULL &&
        OmPhysicsBackendRegistry::newtonBackend()->isAvailable()) {
      warned = true;
      OmLog::info(QObject::tr("Contact sounds are not produced on the Newton physics backend (the ODE-keyed contact "
                              "sound subsystem was removed). Motor sounds are unaffected."));
    }
  }
  OmMotorSoundManager::update();
}

OmSoundClip *OmSoundEngine::sound(const QString &url, const QString &extension, QIODevice *device, double balance, int side) {
  if (url.isEmpty())
    return NULL;

  if (extension != "mp3" && extension != "wav") {
    OmLog::warning(QObject::tr("Invalid URL '%1'. Sounds must be in '.mp3' or '.wav' format.").arg(url));
    return NULL;
  }

  init();
  foreach (OmSoundClip *s, gSounds) {
    if (s->filename() == url && s->side() == side && s->balance() == balance)
      return s;
  }
  OmSoundClip *soundClip = new OmSoundClip;
  try {
    soundClip->load(url, extension, device, balance, side);
    gSounds << soundClip;
    return soundClip;
  } catch (const QString &e) {
    OmLog::warning(QObject::tr("Could not open '%1' sound file: %2").arg(url).arg(e));
    delete soundClip;
    return NULL;
  }
}

OmSoundClip *OmSoundEngine::soundFromText(const QString &text, const QString &engine, const QString &language) {
#ifdef _WIN32
  Q_UNUSED(engine);
  int size = 0;
  if (!gTextToSpeech)
    gTextToSpeech = new OmMicrosoftTextToSpeech();
  if (!gTextToSpeech->error().isEmpty()) {
    OmLog::warning(QObject::tr("TextToSpeech error: %1").arg(gTextToSpeech->error()));
    return NULL;
  }
  qint16 *buffer = gTextToSpeech->generateBufferFromText(text, &size, language);
  if (buffer == NULL) {
    OmLog::warning(QObject::tr("Could not open generated sound from '%1': %2").arg(text).arg(gTextToSpeech->error()));
    return NULL;
  }
  OmWaveFile wave(buffer, size, gTextToSpeech->generatedChannelNumber(), gTextToSpeech->generatedBitsPerSample(),
                  gTextToSpeech->generatedRate());
  OmSoundClip *soundClip = new OmSoundClip();
  try {
    soundClip->load(&wave);
    gSounds << soundClip;
    return soundClip;
  } catch (const QString &e) {
    OmLog::warning(QObject::tr("Could not open generated sound from '%1'.").arg(text));
    delete soundClip;
    return NULL;
  }
#else
  Q_UNUSED(text);
  Q_UNUSED(engine);
  Q_UNUSED(language);
  static bool warned = false;
  if (!warned) {
    OmLog::warning(QObject::tr("Speaker.speak() is not supported on this platform (Microsoft TTS is Windows-only)."));
    warned = true;
  }
  return NULL;
#endif
}

void OmSoundEngine::deleteSource(OmSoundSource *source) {
  if (gSources.contains(source)) {  // at reload/close the source might already have been deleted by cleanup()
    gSources.removeAll(source);
    delete source;
  }
}

void OmSoundEngine::stopAllSources() {
  foreach (OmSoundSource *source, gSources)
    source->stop();
}

OmSoundSource *OmSoundEngine::createSource() {
  init();
  OmSoundSource *source = new OmSoundSource;
  gSources << source;
  return source;
}

void OmSoundEngine::clearAllMotorSoundSources() {
  OmMotorSoundManager::clearAllMotorSoundSources();
}

void OmSoundEngine::clearAllContactSoundSources() {
}

#ifdef __APPLE__
#pragma GCC diagnostic pop
#endif
