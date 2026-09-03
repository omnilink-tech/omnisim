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

//
//  OmMotor.cpp
//

#include "OmMotor.hpp"

#include "OmDataStream.hpp"
#include "OmDownloadManager.hpp"
#include "OmDownloader.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmJoint.hpp"
#include "OmJointParameters.hpp"
#include "OmMuscle.hpp"
#include "OmNetwork.hpp"
#include "OmPropeller.hpp"
#include "OmRobot.hpp"
#include "OmSensor.hpp"
#include "OmSolid.hpp"
#include "OmSoundEngine.hpp"
#include "OmTrack.hpp"
#include "OmUrl.hpp"


#include <QtCore/QDataStream>
#include <QtCore/QUrl>

#include <cassert>
#include <cmath>

#include "../../controller/c/messages.h"  // contains the definitions for the macros C_SET_SAMPLING_PERIOD, C_MOTOR_SET_POSITION, C_MOTOR_SET_VELOCITY ...

QList<const OmMotor *> OmMotor::cMotors;
bool OmMotor::cResetMayOverwriteCommand = true;

void OmMotor::init() {
  mHasAllocatedJointFeedback = false;
  mForceOrTorqueSensor = NULL;
  mNeedToConfigure = false;
  mSoundClip = NULL;
  mForceOrTorqueLastValue = 0.0;
  mKinematicVelocitySign = 0;
  mRequestedDeviceTag = NULL;
  mCoupledMotors.clear();

  mMotorForceOrTorque = 0.0;
  mTargetVelocity = 0.0;
  mTargetPosition = 0.0;
  mCurrentVelocity = 0.0;
  mRawInput = 0.0;
  mUserControl = false;
  mControllerCommandedFinitePosition = false;
  mErrorIntegral = 0.0;
  mPreviousError = 0.0;

  mMaxForceOrTorque = NULL;
  mAcceleration = findSFDouble("acceleration");
  mConsumptionFactor = findSFDouble("consumptionFactor");
  mControlPID = findSFVector3("controlPID");
  mMinPosition = findSFDouble("minPosition");
  mMaxPosition = findSFDouble("maxPosition");
  mMaxVelocity = findSFDouble("maxVelocity");
  mMultiplier = findSFDouble("multiplier");
  mSound = findSFString("sound");
  mMuscles = findMFNode("muscles");
  mDownloader = NULL;
}

OmMotor::OmMotor(const QString &modelName, OmTokenizer *tokenizer) : OmJointDevice(modelName, tokenizer) {
  init();
}

OmMotor::OmMotor(const OmMotor &other) : OmJointDevice(other) {
  init();
}

OmMotor::OmMotor(const OmNode &other) : OmJointDevice(other) {
  init();
}

OmMotor::~OmMotor() {
  delete mForceOrTorqueSensor;
  // notify siblings about removal
  for (int i = 0; i < mCoupledMotors.size(); ++i)
    mCoupledMotors[i]->removeFromCoupledMotors(this);
  cMotors.removeAll(this);
}

void OmMotor::downloadAssets() {
  const QString &soundString = mSound->value();
  if (soundString.isEmpty())
    return;

  const QString &completeUrl = OmUrl::computePath(this, "sound", soundString);
  if (!OmUrl::isWeb(completeUrl) || OmNetwork::instance()->isCachedWithMapUpdate(completeUrl))
    return;

  delete mDownloader;
  mDownloader = OmDownloadManager::instance()->createDownloader(QUrl(completeUrl), this);
  if (isPostFinalizedCalled())
    connect(mDownloader, &OmDownloader::complete, this, &OmMotor::updateSound);
  mDownloader->download();
}

void OmMotor::preFinalize() {
  OmJointDevice::preFinalize();

  cMotors << this;

  // note: only parameter validity check has to occur in preFinalize, validity across couplings must be done in postFinalize
  updateMaxVelocity();
  updateMaxAcceleration();
  updateControlPID();
  updateMinAndMaxPosition();
  updateMultiplier();
  updateSound();

  mForceOrTorqueSensor = new OmSensor();

  const OmJoint *const j = joint();
  mTargetPosition = j ? position() : 0.0;
  mMotorForceOrTorque = mMaxForceOrTorque->value();
  updateMaxForceOrTorque();
}

void OmMotor::postFinalize() {
  OmJointDevice::postFinalize();
  assert(robot());
  if (!mMuscles->isEmpty() || robot()->currentEnergy() >= 0)
    setupJointFeedback();

  inferMotorCouplings();  // it also checks consistency across couplings

  // must be done in the postFinalize step as the coupled motor consistency check could change the values
  mTargetVelocity = mMaxVelocity->value();
  mNeedToConfigure = true;  // notify libController about the changes done by the check

  OmMFIterator<OmMFNode, OmNode *> it(mMuscles);
  while (it.hasNext())
    dynamic_cast<OmMuscle *>(it.next())->postFinalize();

  connect(mMultiplier, &OmSFDouble::changed, this, &OmMotor::updateMultiplier);
  connect(mMaxVelocity, &OmSFDouble::changed, this, &OmMotor::updateMaxVelocity);
  connect(mAcceleration, &OmSFDouble::changed, this, &OmMotor::updateMaxAcceleration);
  connect(mControlPID, &OmSFVector3::changed, this, &OmMotor::updateControlPID);
  connect(mMinPosition, &OmSFDouble::changed, this, &OmMotor::updateMinAndMaxPosition);
  connect(mMaxPosition, &OmSFDouble::changed, this, &OmMotor::updateMinAndMaxPosition);
  connect(mMinPosition, &OmSFDouble::changed, this, &OmMotor::minPositionChanged);
  connect(mMaxPosition, &OmSFDouble::changed, this, &OmMotor::maxPositionChanged);
  connect(mSound, &OmSFString::changed, this, &OmMotor::updateSound);
  connect(mMuscles, &OmSFNode::changed, this, &OmMotor::updateMuscles);
  connect(mMaxForceOrTorque, &OmSFDouble::changed, this, &OmMotor::updateMaxForceOrTorque);
  connect(mDeviceName, &OmSFString::changed, this, &OmMotor::inferMotorCouplings);
}

void OmMotor::createWrenObjects() {
  OmJointDevice::createWrenObjects();
  OmMFIterator<OmMFNode, OmNode *> it(mMuscles);
  while (it.hasNext())
    dynamic_cast<OmMuscle *>(it.next())->createWrenObjects();
}

void OmMotor::setupJointFeedback() {
  // Force/torque feedback was read from the ODE joint feedback; there is no
  // Newton force-feedback service yet, so there is nothing to set up (readings
  // stay 0, see the one-shot warning in the refresh-rate handler).
}

double OmMotor::energyConsumption() const {
  if (dynamic_cast<OmTrack *>(parentNode()))
    return 0.0;

  return fabs(computeFeedback()) * mConsumptionFactor->value();
}

void OmMotor::inferMotorCouplings() {
  // remove the reference to this motor from any of the siblings (necessary in the event of name changes from the interface)
  for (int i = 0; i < mCoupledMotors.size(); ++i)
    mCoupledMotors[i]->removeFromCoupledMotors(this);
  // clear references to my siblings
  mCoupledMotors.clear();
  // rebuild the sibling list
  const QStringList nameChunks = deviceName().split("::");
  if (nameChunks.size() != 2)  // name structure: "motor name::specifier name". Coupling is determined by the motor name part
    return;

  for (int i = 0; i < cMotors.size(); ++i) {
    QStringList otherNameChunks = cMotors[i]->deviceName().split("::");
    if (otherNameChunks.size() != 2)
      continue;

    if (robot() == cMotors[i]->robot() && cMotors[i]->tag() != tag() && nameChunks[0] == otherNameChunks[0] &&
        !mCoupledMotors.contains(const_cast<OmMotor *>(cMotors[i]))) {
      mCoupledMotors.append(const_cast<OmMotor *>(cMotors[i]));
      mCoupledMotors.last()->addToCoupledMotors(this);  // add myself to the newly added sibling
    }
  }

  if (deviceName().contains("::") && mCoupledMotors.size() == 0)
    parsingWarn(tr("Motor '%1' uses the coupled motor name structure but does not have any siblings.").arg(deviceName()));

  checkMultiplierAcrossCoupledMotors();  // it calls all the other checks implicitly
}

/////////////
// Updates //
/////////////

void OmMotor::updateMinAndMaxPosition() {
  enforceMotorLimitsInsideJointLimits();

  if (isPreFinalizedCalled())
    checkMinAndMaxPositionAcrossCoupledMotors();

  mNeedToConfigure = true;
}

void OmMotor::updateMaxForceOrTorque() {
  OmFieldChecker::resetDoubleIfNegative(this, mMaxForceOrTorque, 10.0);
  mNeedToConfigure = true;
}

void OmMotor::updateMaxVelocity() {
  OmFieldChecker::resetDoubleIfNegative(this, mMaxVelocity, -mMaxVelocity->value());

  if (mCoupledMotors.size() > 0 && isPreFinalizedCalled())
    checkMaxVelocityAcrossCoupledMotors();

  mNeedToConfigure = true;
}

void OmMotor::updateMultiplier() {
  if (multiplier() == 0.0) {
    parsingWarn(tr("The value of 'multiplier' cannot be 0. Value reverted to 1."));
    mMultiplier->setValue(1.0);
  }

  if (isPreFinalizedCalled()) {
    checkMultiplierAcrossCoupledMotors();
  }

  mNeedToConfigure = true;
}

void OmMotor::updateControlPID() {
  const OmVector3 &pid = mControlPID->value();
  const double p = pid.x();
  if (p <= 0.0)
    parsingWarn(tr("'controlP' (currently %1) must be positive.").arg(p));

  const double i = pid.y();
  if (i < 0.0)
    parsingWarn(tr("'controlI' (currently %1) must be non-negative.").arg(i));

  const double d = pid.z();
  if (d < 0.0)
    parsingWarn(tr("'controlD' (currently %1) must be non-negative.").arg(d));

  mErrorIntegral = 0.0;
  mPreviousError = 0.0;

  mNeedToConfigure = true;
}

void OmMotor::updateSound() {
  const QString &soundString = mSound->value();
  if (soundString.isEmpty()) {
    mSoundClip = NULL;
  } else {
    const QString &completeUrl = OmUrl::computePath(this, "sound", mSound->value(), true);
    if (OmUrl::isWeb(completeUrl)) {
      if (mDownloader && !mDownloader->error().isEmpty()) {
        warn(mDownloader->error());  // failure downloading or file does not exist (404)
        mSoundClip = NULL;
        // downloader needs to be deleted in case the URL is switched back to something valid
        delete mDownloader;
        mDownloader = NULL;
        return;
      }
      if (!OmNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
        downloadAssets();
        return;
      }
    }

    // at this point the sound must be available (locally or in the cache).
    // determine extension from URL since for remotely defined assets the cached version does not retain this information
    const QString extension = completeUrl.mid(completeUrl.lastIndexOf('.') + 1).toLower();
    if (OmUrl::isWeb(completeUrl))
      mSoundClip = OmSoundEngine::sound(OmNetwork::instance()->get(completeUrl), extension);
    // completeUrl can contain missing_texture.png if the user inputs an invalid .png or .jpg file in the field. In this case,
    // mSoundClip should be set to NULL
    else if (!(completeUrl.isEmpty() || completeUrl == OmUrl::missingTexture()))
      mSoundClip = OmSoundEngine::sound(completeUrl, extension);
    else
      mSoundClip = NULL;
  }
  OmSoundEngine::clearAllMotorSoundSources();
}

void OmMotor::updateMuscles() {
  setupJointFeedback();
}

void OmMotor::updateMaxAcceleration() {
  OmFieldChecker::resetDoubleIfNegativeAndNotDisabled(this, mAcceleration, -1, -1);
  mNeedToConfigure = true;
}

///////////////////
// Plain setters //
///////////////////

void OmMotor::setMaxVelocity(double v) {
  mMaxVelocity->setValue(v);
  awake();
}

void OmMotor::setMaxAcceleration(double acc) {
  mAcceleration->setValue(acc);
  awake();
}

/////////////////
// API setters //
/////////////////

void OmMotor::setTargetPosition(double position) {
  const double maxp = mMaxPosition->value();
  const double minp = mMinPosition->value();
  const bool velocityControl = std::isinf(position);

  // negative multipliers could turn the +inf into a -inf
  mTargetPosition = velocityControl ? position : position * multiplier();

  if (maxp != minp && !velocityControl) {
    if (mTargetPosition > maxp) {
      mTargetPosition = maxp;
      warn(QString("too big requested position: %1 > %2 -- the target is clamped to 'maxPosition'. Command within [minPosition, maxPosition], or raise 'maxPosition' on the motor (and 'maxStop' on its joint) in the world; see docs/reference/motor.md.").arg(position).arg(maxp));
    } else if (mTargetPosition < minp) {
      mTargetPosition = minp;
      warn(QString("too low requested position: %1 < %2 -- the target is clamped to 'minPosition'. Command within [minPosition, maxPosition], or lower 'minPosition' on the motor (and 'minStop' on its joint) in the world; see docs/reference/motor.md.").arg(position).arg(minp));
    }
  }

  mUserControl = false;
  awake();
}

void OmMotor::setVelocity(double velocity) {
  mTargetVelocity = velocity * multiplier();

  const double m = mMaxVelocity->value();
  if (fabs(mTargetVelocity) > m) {
    warn(tr("The requested velocity %1 exceeds 'maxVelocity' = %2. It is clamped to the limit, so the motor runs slower than commanded; raise 'maxVelocity' on the motor in the world or command within it -- see docs/reference/motor.md.").arg(mTargetVelocity).arg(m));
    mTargetVelocity = mTargetVelocity >= 0.0 ? m : -m;
  }

  awake();
}

void OmMotor::setAcceleration(double acceleration) {
  // note: a check is performed on libController side for negative values
  mAcceleration->setValue(acceleration);  // mAcceleration specifies maximal acceleration, hence it isn't multiplied
  awake();
}

void OmMotor::setForceOrTorque(double forceOrTorque) {
  if (!mUserControl)  // we were previously using motor force
    turnOffMotor();
  mUserControl = true;

  mRawInput = forceOrTorque * multiplier();
  if (fabs(mRawInput) > mMotorForceOrTorque) {
    if (mCoupledMotors.size() == 0) {  // silence warning for coupled motors
      if (nodeType() == WB_NODE_ROTATIONAL_MOTOR)
        warn(tr("The requested motor torque %1 exceeds 'maxTorque' = %2. It is clamped to the limit; raise 'maxTorque' on the RotationalMotor in the world or command within it -- see docs/reference/rotationalmotor.md.").arg(mRawInput).arg(mMotorForceOrTorque));
      else
        warn(tr("The requested motor force %1 exceeds 'maxForce' = %2. It is clamped to the limit; raise 'maxForce' on the LinearMotor in the world or command within it -- see docs/reference/linearmotor.md.").arg(mRawInput).arg(mMotorForceOrTorque));
    }

    mRawInput = mRawInput >= 0.0 ? mMotorForceOrTorque : -mMotorForceOrTorque;
  }

  awake();
}

void OmMotor::setAvailableForceOrTorque(double availableForceOrTorque) {
  // note: a check is performed on libController side for negative values
  mMotorForceOrTorque = availableForceOrTorque;

  const double m = mMaxForceOrTorque->value();
  if (mMotorForceOrTorque > m) {
    if (mCoupledMotors.size() == 0) {  // silence warning for coupled motors
      if (nodeType() == WB_NODE_ROTATIONAL_MOTOR)
        warn(tr("The requested available motor torque %1 exceeds 'maxTorque' = %2. It is clamped to the limit; raise 'maxTorque' on the RotationalMotor in the world or request within it -- see docs/reference/rotationalmotor.md.").arg(mMotorForceOrTorque).arg(m));
      else
        warn(tr("The requested available motor force %1 exceeds 'maxForce' = %2. It is clamped to the limit; raise 'maxForce' on the LinearMotor in the world or request within it -- see docs/reference/linearmotor.md.").arg(mMotorForceOrTorque).arg(m));
    }
    mMotorForceOrTorque = m;
  }
  awake();
}

double OmMotor::computeCurrentDynamicVelocity(double ms, double position) {
  if (mMotorForceOrTorque == 0.0) {  // no control
    mCurrentVelocity = 0.0;
    return mCurrentVelocity;
  }

  // compute required velocity
  double velocity = -mTargetVelocity;
  if (!std::isinf(mTargetPosition)) {  // PID-position control
    const double error = mTargetPosition - position;
    const double ts = ms * 0.001;
    mErrorIntegral += error * ts;
    const double errorDerivative = (error - mPreviousError) / ts;
    const double outputValue =
      mControlPID->value().x() * error + mControlPID->value().y() * mErrorIntegral + mControlPID->value().z() * errorDerivative;
    mPreviousError = error;

    if (fabs(outputValue) < fabs(mTargetVelocity))
      velocity = -outputValue;
    else
      velocity = outputValue > 0.0 ? -fabs(mTargetVelocity) : fabs(mTargetVelocity);
  }

  // try to get closer to velocity
  double acc = mAcceleration->value();
  if (acc == -1.0)
    mCurrentVelocity = velocity;  // use maximum force/torque
  else {
    const double ts = ms * 0.001;
    double d = fabs(velocity - mCurrentVelocity) / ts;  // requested acceleration
    if (d < acc)
      acc = d;
    if (velocity > mCurrentVelocity)
      mCurrentVelocity += acc * ts;
    else
      mCurrentVelocity -= acc * ts;
  }

  return mCurrentVelocity;
}

// run control without physics simulation
bool OmMotor::runKinematicControl(double ms, double &position) {
  static const double TARGET_POSITION_THRESHOLD = 1e-6;
  bool doNothing = false;
  if (std::isinf(mTargetPosition)) {  // velocity control
    const double maxp = mMaxPosition->value();
    const double minp = mMinPosition->value();
    doNothing = maxp != minp && ((position >= maxp && mTargetVelocity >= 0.0) || (position <= minp && mTargetVelocity <= 0.0));
  } else
    doNothing = fabs(mTargetPosition - position) < TARGET_POSITION_THRESHOLD;

  if (doNothing) {
    mCurrentVelocity = 0.0;
    mKinematicVelocitySign = 0;
    return false;
  }

  const double ts = ms * 0.001;
  double acc = mAcceleration->value();
  if (acc == -1.0 || acc > mMotorForceOrTorque)
    acc = mMotorForceOrTorque;
  static const double TARGET_VELOCITY_THRESHOLD = 1e-6;
  if (fabs(mTargetVelocity - mCurrentVelocity) > TARGET_VELOCITY_THRESHOLD) {
    // didn't reach the target velocity yet
    if (mTargetVelocity > mCurrentVelocity) {
      mCurrentVelocity += acc * ts;
      if (mCurrentVelocity > mTargetVelocity)
        mCurrentVelocity = mTargetVelocity;
    } else {
      mCurrentVelocity -= acc * ts;
      if (mCurrentVelocity < mTargetVelocity)
        mCurrentVelocity = mTargetVelocity;
    }
  }

  if (mTargetPosition > position) {
    mKinematicVelocitySign = 1;
    position += mCurrentVelocity * ts;
    if (position > mTargetPosition)
      position = mTargetPosition;
  } else {
    mKinematicVelocitySign = -1;
    position -= mCurrentVelocity * ts;
    if (position < mTargetPosition)
      position = mTargetPosition;
  }

  return true;
}

void OmMotor::powerOn(bool e) {  // called when running out of energy with e=false
  OmDevice::powerOn(e);
  if (!e)
    mMotorForceOrTorque = 0.0;
  else
    mMotorForceOrTorque = mMaxForceOrTorque->value();
}

bool OmMotor::isConfigureDone() const {
  return robot()->isConfigureDone();
}

void OmMotor::checkMinAndMaxPositionAcrossCoupledMotors() {
  // if the position is unlimited for this motor, the coupled ones must also be
  for (int i = 0; i < mCoupledMotors.size(); ++i) {
    if (isPositionUnlimited() && !mCoupledMotors[i]->isPositionUnlimited()) {
      parsingWarn(
        tr("For coupled motors, if one has unlimited position, its siblings must have unlimited position as well. Adjusting "
           "'minPosition' and 'maxPosition' to 0 for motor '%1'.")
          .arg(deviceName()));
      mCoupledMotors[i]->setMinPosition(0);
      mCoupledMotors[i]->setMaxPosition(0);
    }
  }

  // if the position is limited for this motor, adjust the limits of the siblings accordingly
  if (!isPositionUnlimited()) {
    for (int i = 0; i < mCoupledMotors.size(); ++i) {
      double potentialMinimalPosition = minPosition() * mCoupledMotors[i]->multiplier() / multiplier();
      double potentialMaximalPosition = maxPosition() * mCoupledMotors[i]->multiplier() / multiplier();

      if (potentialMaximalPosition < potentialMinimalPosition) {  // swap values for consistency
        const double tmp = potentialMaximalPosition;
        potentialMaximalPosition = potentialMinimalPosition;
        potentialMinimalPosition = tmp;
      }

      if (mCoupledMotors[i]->minPosition() != potentialMinimalPosition) {
        parsingWarn(tr("When using coupled motors, 'minPosition' must be consistent across devices. Adjusting 'minPosition' "
                       "from %1 to %2 for motor '%3'.")
                      .arg(mCoupledMotors[i]->minPosition())
                      .arg(potentialMinimalPosition)
                      .arg(deviceName()));
        mCoupledMotors[i]->setMinPosition(potentialMinimalPosition);
      }

      if (mCoupledMotors[i]->maxPosition() != potentialMaximalPosition) {
        parsingWarn(tr("When using coupled motors, 'maxPosition' must be consistent across devices. Adjusting 'maxPosition' "
                       "from %1 to %2 for motor '%3'.")
                      .arg(mCoupledMotors[i]->maxPosition())
                      .arg(potentialMaximalPosition)
                      .arg(deviceName()));
        mCoupledMotors[i]->setMaxPosition(potentialMaximalPosition);
      }
    }
  }
}

void OmMotor::enforceMotorLimitsInsideJointLimits() {
  if (isPositionUnlimited())
    return;

  const OmJoint *parentJoint = dynamic_cast<OmJoint *>(parentNode());
  double p = 0.0;
  if (parentJoint) {
    if (positionIndex() == 1 && parentJoint->parameters())
      p = parentJoint->parameters()->position();
    if (positionIndex() == 2 && parentJoint->parameters2())
      p = parentJoint->parameters2()->position();
    if (positionIndex() == 3 && parentJoint->parameters3())
      p = parentJoint->parameters3()->position();
  }

  // current joint position should lie between min and max position
  OmFieldChecker::resetDoubleIfLess(this, mMaxPosition, p, p);
  OmFieldChecker::resetDoubleIfGreater(this, mMinPosition, p, p);
}

void OmMotor::checkMaxVelocityAcrossCoupledMotors() {
  // assume this motor is pushed to its limit, ensure the sibling limits aren't broken
  for (int i = 0; i < mCoupledMotors.size(); ++i) {
    const double potentialMaximalVelocity = maxVelocity() * fabs(mCoupledMotors[i]->multiplier()) / fabs(multiplier());

    if (mCoupledMotors[i]->maxVelocity() != potentialMaximalVelocity) {
      parsingWarn(tr("When using coupled motors, velocity limits must be consistent across devices. Adjusted 'maxVelocity' "
                     "from %1 to %2 for motor '%3'.")
                    .arg(mCoupledMotors[i]->maxVelocity())
                    .arg(potentialMaximalVelocity)
                    .arg(deviceName()));
      mCoupledMotors[i]->setMaxVelocity(potentialMaximalVelocity);
    }
  }
}

void OmMotor::checkMultiplierAcrossCoupledMotors() {
  if (mCoupledMotors.size() == 0)
    return;

  checkMinAndMaxPositionAcrossCoupledMotors();
  checkMaxVelocityAcrossCoupledMotors();
}

/////////////
// Control //
/////////////

void OmMotor::addConfigureToStream(OmDataStream &stream) {
  stream << (unsigned short)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)type();
  stream << (double)mMinPosition->value();
  stream << (double)mMaxPosition->value();
  stream << (double)mMaxVelocity->value();
  stream << (double)mAcceleration->value();
  stream << (double)mMaxForceOrTorque->value();
  stream << (double)mControlPID->value().x();
  stream << (double)mControlPID->value().y();
  stream << (double)mControlPID->value().z();
  stream << (double)mTargetPosition;
  stream << (double)mTargetVelocity;
  stream << (double)mMultiplier->value();
  stream << (int)mCoupledMotors.size();
  for (int i = 0; i < mCoupledMotors.size(); ++i)
    stream << (WbDeviceTag)mCoupledMotors[i]->tag();
  mNeedToConfigure = false;
}

void OmMotor::writeConfigure(OmDataStream &stream) {
  if (mForceOrTorqueSensor)
    mForceOrTorqueSensor->connectToRobotSignal(robot());
  addConfigureToStream(stream);
}

void OmMotor::enableMotorFeedback(int rate) {
  const OmJoint *const j = joint();

  if (j == NULL) {
    warn(tr("Feedback is available for motorized joints only: this motor is not the 'device' of a Joint (a Propeller or a bare Motor has no joint to measure), so the reading stays 0. Place the motor in a HingeJoint/SliderJoint 'device' field to measure it -- see docs/reference/motor.md."));
    return;
  }

  const OmSolid *const s = j->solidEndPoint();

  if (s == NULL) {
    warn(nodeType() == WB_NODE_ROTATIONAL_MOTOR ?
           tr("wb_motor_enable_torque_feedback(): cannot be invoked for a Joint with no end point. The feedback stays 0; set the Joint's 'endPoint' to a Solid that has a Physics node.") :
           tr("wb_motor_enable_force_feedback(): cannot be invoked for a Joint with no end point. The feedback stays 0; set the Joint's 'endPoint' to a Solid that has a Physics node."));
    return;
  }

  if (s->physics() == NULL) {
    warn(nodeType() == WB_NODE_ROTATIONAL_MOTOR ?
           tr("wb_motor_enable_torque_feedback(): cannot be invoked for a Joint whose end point has no Physics node. The feedback stays 0; add a Physics node to the Joint's endPoint Solid.") :
           tr("wb_motor_enable_force_feedback(): cannot be invoked for a Joint whose end point has no Physics node. The feedback stays 0; add a Physics node to the Joint's endPoint Solid."));
    return;
  }

  const OmSolid *const us = j->upperSolid();
  if (us->physics() == NULL) {
    warn(nodeType() == WB_NODE_ROTATIONAL_MOTOR ?
           tr("wb_motor_enable_torque_feedback(): cannot be invoked because the parent Solid has no Physics node.") :
           tr("wb_motor_enable_force_feedback(): cannot be invoked because the parent Solid has no Physics node."));
    return;
  }

  // The measurement source was ODE's joint feedback, which has been removed,
  // and there is no Newton force-feedback service yet -- the sensor stays
  // enabled but reads its no-data default (0). One-shot warning so the zero is
  // never mistaken for a measured torque/force.
  if (rate) {
    static bool cNoFeedbackWarned = false;
    if (!cNoFeedbackWarned) {
      cNoFeedbackWarned = true;
      warn(tr("motor force/torque feedback is not measured: it was read from ODE's joint feedback, and ODE "
              "has been removed. Readings stay 0 until the Newton force-feedback service lands."));
    }
  }

  mForceOrTorqueSensor->setRefreshRate(rate);
}

void OmMotor::handleMessage(QDataStream &stream) {
  short command;
  stream >> command;

  switch (command) {
    case C_MOTOR_SET_POSITION: {
      double p;
      stream >> p;
      setTargetPosition(p);
      // W1.4 servo promotion latch: the WIRE is the only place controller
      // intent is unambiguous (setTargetPosition also runs on reset and on
      // engine-internal pre-configure moves, neither of which should promote
      // a limit-less wheel to a servo).
      mControllerCommandedFinitePosition = !std::isinf(p);
      // relay target position to coupled motors, if any
      for (int i = 0; i < mCoupledMotors.size(); ++i) {
        mCoupledMotors[i]->setTargetPosition(p);
        mCoupledMotors[i]->mControllerCommandedFinitePosition = !std::isinf(p);
      }
      break;
    }
    case C_MOTOR_SET_VELOCITY: {
      double v;
      stream >> v;
      setVelocity(v);
      // relay target velocity to coupled motors, if any
      for (int i = 0; i < mCoupledMotors.size(); ++i)
        mCoupledMotors[i]->setVelocity(v);
      break;
    }
    case C_MOTOR_SET_ACCELERATION: {
      double a;
      stream >> a;
      setAcceleration(a);
      break;
    }
    case C_MOTOR_SET_FORCE: {
      double forceOrTorque;
      stream >> forceOrTorque;
      setForceOrTorque(forceOrTorque);
      for (int i = 0; i < mCoupledMotors.size(); ++i)
        mCoupledMotors[i]->setForceOrTorque(forceOrTorque);
      break;
    }
    case C_MOTOR_SET_AVAILABLE_FORCE: {
      double availableForceOrTorque;
      stream >> availableForceOrTorque;
      setAvailableForceOrTorque(availableForceOrTorque);
      break;
    }
    case C_MOTOR_SET_CONTROL_PID: {
      double controlP, controlI, controlD;
      stream >> controlP;
      stream >> controlI;
      stream >> controlD;
      mControlPID->setValue(controlP, controlI, controlD);
      awake();
      break;
    }
    case C_MOTOR_FEEDBACK: {
      short rate;
      stream >> rate;
      enableMotorFeedback(rate);
      break;
    }
    case C_MOTOR_GET_ASSOCIATED_DEVICE: {
      short deviceType;
      stream >> deviceType;
      assert(mRequestedDeviceTag == NULL);
      mRequestedDeviceTag = new WbDeviceTag[1];
      const OmLogicalDevice *device = getSiblingDeviceByType(deviceType);
      mRequestedDeviceTag[0] = device ? device->tag() : 0;
      break;
    }
    default:
      assert(0);  // Invalid command.
  }
}

void OmMotor::writeAnswer(OmDataStream &stream) {
  if (mForceOrTorqueSensor && (refreshSensorIfNeeded() || mForceOrTorqueSensor->hasPendingValue())) {
    stream << tag();
    stream << (unsigned char)C_MOTOR_FEEDBACK;
    stream << (double)mForceOrTorqueLastValue;
    mForceOrTorqueSensor->resetPendingValue();
  }
  if (mRequestedDeviceTag != NULL) {
    stream << tag();
    stream << (unsigned char)C_MOTOR_GET_ASSOCIATED_DEVICE;
    stream << (unsigned short)mRequestedDeviceTag[0];
    delete[] mRequestedDeviceTag;
    mRequestedDeviceTag = NULL;
  }
  if (mNeedToConfigure)
    addConfigureToStream(stream);
}

bool OmMotor::refreshSensorIfNeeded() {
  if (isPowerOn() && mForceOrTorqueSensor && mForceOrTorqueSensor->needToRefresh()) {
    mForceOrTorqueLastValue = computeFeedback();
    mForceOrTorqueSensor->updateTimer();
    return true;
  }
  return false;
}

void OmMotor::reset(const QString &id) {
  OmJointDevice::reset(id);

  turnOffMotor();

  // ALWAYS reset the servo's own accumulated state. mCurrentVelocity is where
  // the acceleration ramp got to and the PID pair is an integral/derivative
  // history: they describe how far the simulation ran, not what anybody asked
  // for, and carrying a stale integral across a reset kicks the joint on the
  // first step after it. Available force/torque is likewise restored from the
  // (just re-restored) authored field, exactly as before -- notably it is what
  // re-arms a motor that powerOn(false) zeroed when the robot's battery ran
  // flat, since OmRobot::reset refills the battery without re-emitting powerOn.
  mCurrentVelocity = 0.0;
  mErrorIntegral = 0.0;
  mPreviousError = 0.0;
  mMotorForceOrTorque = mMaxForceOrTorque->value();

  // THE CONTROLLER'S COMMAND IS NOT PART OF THE AUTHORED SCENE. When the reset
  // leaves controllers running, overwriting it is not a restore -- it replaces
  // a live command with one that will never be re-issued, and because velocity
  // control is encoded as an INFINITE target position, the overwrite silently
  // demotes every velocity-mode wheel to a position hold. See
  // OmMotor::resetMayOverwriteMotorCommand() for the measurement.
  if (!cResetMayOverwriteCommand)
    return;

  mTargetVelocity = mMaxVelocity->value();
  mUserControl = false;
  mRawInput = 0.0;
  // NOTE (verified 2026-08-12, correcting an earlier root-cause note): reading
  // the LIVE position here rather than the saved one is not observable. This
  // runs from OmJoint::reset, which finishes by calling
  // setPosition(mSavedPositions[id]) -> OmMotor::setTargetPosition(authored
  // angle), so for any joint-mounted motor the authored value overwrites this
  // one a few lines later. `j` is only NULL for a Track or Propeller motor,
  // where 0.0 is the authored target.
  const OmJoint *const j = joint();
  mTargetPosition = j ? position() : 0.0;
}

void OmMotor::awake() const {
  const OmJoint *const j = joint();
  if (j) {
    OmSolid *const s = j->solidEndPoint();
    if (s)
      s->awake();
    return;
  }

  const OmPropeller *const p = propeller();
  const OmTrack *const t = dynamic_cast<OmTrack *>(parentNode());
  if (p || t) {
    OmSolid *const s = upperSolid();
    assert(s);
    s->awake();
  }
}

void OmMotor::addToCoupledMotors(OmMotor *motor) {
  if (!mCoupledMotors.contains(motor))
    mCoupledMotors.append(motor);
}

void OmMotor::resetPhysics() {
  mCurrentVelocity = 0;
}

QList<const OmBaseNode *> OmMotor::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const OmBaseNode *> list;
  OmMFIterator<OmMFNode, OmNode *> it(mMuscles);
  while (it.hasNext())
    list << static_cast<OmBaseNode *>(it.next());
  return list;
}

void OmMotor::exportNodeFields(OmWriter &writer) const {
  OmJointDevice::exportNodeFields(writer);

  exportSFResourceField("sound", mSound, writer.relativeSoundsPath(), writer);
}

QStringList OmMotor::customExportedFields() const {
  QStringList fields;
  fields << "sound";
  return fields;
}
