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

#include "OmJoystickInterface.hpp"
#include "OmJoystickListener.hpp"

#include <math.h>

#include <OIS/OISEffect.h>
#include <OIS/OISException.h>
#include <OIS/OISForceFeedback.h>
#include <OIS/OISInputManager.h>

#define SIGMOID_LAMBDA 0.0001

static OIS::InputManager *gInputManager = NULL;
static int gCurrentNumberOfInterface = 0;
static WId gWindowHandle = 0;

void OmJoystickInterface::setWindowHandle(WId windowHandle) {
  gWindowHandle = windowHandle;
}

void OmJoystickInterface::init() {
  mCorrectlyInitialized = true;
  mHasForceFeedback = false;
  mForceFeedbackUsed = false;
  mSupportConstantForceFeedbackEffect = false;
  mAddedForce = 0.0;
  mAutoCenteringForce = 0.0;
  mResistantForce = 0.0;
  mAutoCenteringGain = 0.0;
  mResistanceGain = 0.0;
  mConstantForceDuration = 1.0;
  mConstantForceTimer = 0.0;
  mPreviousResistanceAngle = 0;
  mPreviousResistanceAngleInitialized = false;
  mForceAxis = 0;

  mListener = NULL;

  mJoystick = NULL;
  mForceFeedback = NULL;
  mEffect = NULL;

  mError.clear();

  // initialize the input manager
  gCurrentNumberOfInterface++;
  if (!gInputManager) {
    OIS::ParamList pl;
    pl.insert(std::make_pair(std::string("WINDOW"), QString::number(gWindowHandle).toStdString()));
    gInputManager = OIS::InputManager::createInputSystem(pl);
  }

  if (gInputManager->getNumberOfDevices(OIS::OISJoyStick) < gCurrentNumberOfInterface) {
    mError = tr("No free joystick found.");
    mCorrectlyInitialized = false;
    return;
  }

  // get the joystick
  mJoystick = static_cast<OIS::JoyStick *>(gInputManager->createInputObject(OIS::OISJoyStick, true));

  if (!mJoystick) {
    mError = tr("Joystick not accessible.");
    mCorrectlyInitialized = false;
    return;
  }

  int nbAxes = mJoystick->getNumberOfComponents(OIS::OIS_Axis);
  int nbPovs = mJoystick->getNumberOfComponents(OIS::OIS_POV);
  int nbButtons = mJoystick->getNumberOfComponents(OIS::OIS_Button);
  mListener = new OmJoystickListener(nbAxes, nbButtons, nbPovs);
  connect(mListener, &OmJoystickListener::changed, this, &OmJoystickInterface::changed);
  mJoystick->setEventCallback(mListener);

  // get the force feedback if any
  mForceFeedback = static_cast<OIS::ForceFeedback *>(mJoystick->queryInterface(OIS::Interface::ForceFeedback));
  if (mForceFeedback) {
    mHasForceFeedback = true;
    // check supported effects
    const OIS::ForceFeedback::SupportedEffectList forceFeedbackEffects = mForceFeedback->getSupportedEffects();
    if (forceFeedbackEffects.size() > 0) {
      OIS::ForceFeedback::SupportedEffectList::const_iterator itforceFeedbackEffects;
      for (itforceFeedbackEffects = forceFeedbackEffects.begin(); itforceFeedbackEffects != forceFeedbackEffects.end();
           ++itforceFeedbackEffects) {
        if (itforceFeedbackEffects->second == OIS::Effect::Constant)
          mSupportConstantForceFeedbackEffect = true;
      }
    }
  } else
    mHasForceFeedback = false;

  // if force feedback and constant effect available, create the effect
  if (mHasForceFeedback && mSupportConstantForceFeedbackEffect) {
    mEffect = new OIS::Effect(OIS::Effect::ConstantForce, OIS::Effect::Constant);
    mEffect->direction = OIS::Effect::East;
    mEffect->trigger_button = 0;
    mEffect->trigger_interval = 0;
    mEffect->replay_length = OIS::Effect::OIS_INFINITE;
    mEffect->replay_delay = 0;
    mEffect->setNumAxes(1);
  }
}

OmJoystickInterface::OmJoystickInterface() {
  init();
}

OmJoystickInterface::~OmJoystickInterface() {
  gCurrentNumberOfInterface--;
  delete mListener;
  if (mHasForceFeedback && mSupportConstantForceFeedbackEffect)
    mForceFeedback->remove(mEffect);
  delete mEffect;
  try {
    if (gInputManager && mJoystick)
      gInputManager->destroyInputObject(mJoystick);
  } catch (const OIS::Exception &e) {
  }
  if (gInputManager && gCurrentNumberOfInterface == 0) {
    OIS::InputManager::destroyInputSystem(gInputManager);
    gInputManager = NULL;
  }
}

void OmJoystickInterface::step() {
  capture();
  computeAutoCentering();
  computeLowSpeedResistance();
  setForceFeedback();
  if (mAddedForce != 0.0) {
    mConstantForceTimer -= 0.001 * updateRate();
    if (mConstantForceTimer <= 0)
      mAddedForce = 0.0;
  }
}

void OmJoystickInterface::setForceFeedback() {
  if (!mHasForceFeedback || !mSupportConstantForceFeedbackEffect ||
      (!mForceFeedbackUsed && (mAddedForce + mAutoCenteringForce + mResistantForce) == 0))
    return;

  if (gInputManager->getNumberOfDevices(OIS::OISJoyStick) == 0)
    return;  // avoid crash of Webots when removing the joystick at runtime

  mEffect->replay_length = OIS::Effect::OIS_INFINITE;
  OIS::ConstantEffect *constantEffect = dynamic_cast<OIS::ConstantEffect *>(mEffect->getForceEffect());
  constantEffect->level = mAddedForce + mAutoCenteringForce + mResistantForce;
  constantEffect->envelope.attackLength = 0;
  constantEffect->envelope.attackLevel = (unsigned short)constantEffect->level;
  constantEffect->envelope.fadeLength = 0;
  constantEffect->envelope.fadeLevel = (unsigned short)constantEffect->level;

  // upload the effect to the force feedback
  try {
    if (mForceFeedbackUsed)
      mForceFeedback->modify(mEffect);
    else
      mForceFeedback->upload(mEffect);
  } catch (const OIS::Exception &e) {
    mHasForceFeedback = false;
  }

  mForceFeedbackUsed = true;
}

void OmJoystickInterface::capture() {
  if (mJoystick)
    mJoystick->capture();
}

void OmJoystickInterface::setForceAxis(int axis) {
  if (mListener && mListener->numberOfAxes())
    mForceAxis = axis;
}

void OmJoystickInterface::computeAutoCentering() {
  double angle = mListener->axisValue(mForceAxis);
  double force = 1.0 / (1.0 + exp(SIGMOID_LAMBDA * angle));
  force = mAutoCenteringGain * ((force - 0.5) * 2);  // convert sigmoïd from [0;1] to [-mAutoCenteringGain;mAutoCenteringGain]
#ifdef _WIN32
  force = -force;
#endif
  mAutoCenteringForce = force;
}

void OmJoystickInterface::computeLowSpeedResistance() {
  double currentAngle = mListener->axisValue(mForceAxis);
  // as long as we receive 0 it means we haven't received any event for this axis
  if (!mPreviousResistanceAngleInitialized && currentAngle != 0) {
    mPreviousResistanceAngleInitialized = true;
    mPreviousResistanceAngle = currentAngle;
  }
  double angle = 0.2 * currentAngle + 0.8 * mPreviousResistanceAngle;  // compute a kind of 'average' of the steering speed
  double diff = angle - mPreviousResistanceAngle;
  double force = -mResistanceGain * diff;
#ifdef _WIN32
  force = -force;
#endif
  mResistantForce = force;
  mPreviousResistanceAngle = angle;
}

bool OmJoystickInterface::isButtonPressed(int button) const {
  return mListener->isButtonPressed(button);
}

int OmJoystickInterface::axisValue(int axis) const {
  return mListener->axisValue(axis);
}

int OmJoystickInterface::povValue(int pov) const {
  return mListener->povValue(pov);
}

int OmJoystickInterface::numberOfAxes() const {
  return mListener->numberOfAxes();
}

int OmJoystickInterface::numberOfPovs() const {
  return mListener->numberOfPovs();
}

int OmJoystickInterface::numberOfButtons() const {
  return mListener->numberOfButtons();
}

int OmJoystickInterface::numberOfPressedButtons() const {
  return mListener->numberOfPressedButtons();
}

void OmJoystickInterface::releaseButtons() {
  mListener->releaseButtons();
}

const QString OmJoystickInterface::model() const {
  if (mJoystick)
    return QString(mJoystick->vendor().c_str());
  return QString();
}

void OmJoystickInterface::addForce(int level) {
  mAddedForce = level;
  mConstantForceTimer = mConstantForceDuration;
}

bool OmJoystickInterface::buttonHasChanged() {
  return mListener->buttonHasChanged();
}

bool OmJoystickInterface::povHasChanged() {
  return mListener->povHasChanged();
}

bool OmJoystickInterface::axisHasChanged() {
  return mListener->axisHasChanged();
}

void OmJoystickInterface::resetButtonHasChanged() {
  mListener->resetButtonHasChanged();
}

void OmJoystickInterface::resetPovHasChanged() {
  mListener->resetPovHasChanged();
}

void OmJoystickInterface::resetAxisHasChanged() {
  mListener->resetAxisHasChanged();
}
