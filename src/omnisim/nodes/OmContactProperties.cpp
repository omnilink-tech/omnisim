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

#include "OmContactProperties.hpp"

#include "OmDownloadManager.hpp"
#include "OmDownloader.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmNetwork.hpp"
#include "OmSoundEngine.hpp"
#include "OmUrl.hpp"
#include "OmWorld.hpp"

static const QString gUrlNames[3] = {"bumpSound", "rollSound", "slideSound"};

// internal parity plan, item W1.5 -- see the header. "Authored" == the field is
// present in the file with a value other than the .wrl default, which is what
// OmField::isDefault() tests. A missing field and a field written at its own
// default are indistinguishable here and both count as NOT authored: that is
// the conservative direction, because the point of the disclosure is to catch
// an author who asked for restitution, not to shout at every world that has
// ever carried a ContactProperties node.
bool OmContactProperties::bounceIsAuthored() const {
  const OmField *const f = findField("bounce");
  return f != NULL && !f->isDefault();
}

bool OmContactProperties::bounceVelocityIsAuthored() const {
  const OmField *const f = findField("bounceVelocity");
  return f != NULL && !f->isDefault();
}

void OmContactProperties::init() {
  mMaterial1 = findSFString("material1");
  mMaterial2 = findSFString("material2");
  mCoulombFriction = findMFDouble("coulombFriction");
  mFrictionRotation = findSFVector2("frictionRotation");
  mRollingFriction = findSFVector3("rollingFriction");
  mBounce = findSFDouble("bounce");
  mBounceVelocity = findSFDouble("bounceVelocity");
  mForceDependentSlip = findMFDouble("forceDependentSlip");
  mSoftErp = findSFDouble("softERP");
  mSoftCfm = findSFDouble("softCFM");
  mBumpSound = findSFString("bumpSound");
  mRollSound = findSFString("rollSound");
  mSlideSound = findSFString("slideSound");
  mMaxContactJoints = findSFInt("maxContactJoints");

  mBumpSoundClip = NULL;
  mRollSoundClip = NULL;
  mSlideSoundClip = NULL;
  for (int i = 0; i < 3; ++i)
    mDownloader[i] = NULL;
}

OmContactProperties::OmContactProperties(OmTokenizer *tokenizer) : OmBaseNode("ContactProperties", tokenizer) {
  init();
}

OmContactProperties::OmContactProperties(const OmContactProperties &other) : OmBaseNode(other) {
  init();
}

OmContactProperties::OmContactProperties(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmContactProperties::~OmContactProperties() {
}

void OmContactProperties::downloadAsset(const QString &url, int index) {
  if (url.isEmpty())
    return;

  const QString &completeUrl = OmUrl::computePath(this, gUrlNames[index], url);
  if (!OmUrl::isWeb(completeUrl) || OmNetwork::instance()->isCachedWithMapUpdate(completeUrl))
    return;

  delete mDownloader[index];
  mDownloader[index] = OmDownloadManager::instance()->createDownloader(QUrl(completeUrl), this);
  if (isPostFinalizedCalled()) {
    void (OmContactProperties::*callback)(void);
    callback = index == 0 ? &OmContactProperties::updateBumpSound :
                            (index == 1 ? &OmContactProperties::updateRollSound : &OmContactProperties::updateSlideSound);
    connect(mDownloader[index], &OmDownloader::complete, this, callback);
  }
  mDownloader[index]->download();
}

void OmContactProperties::downloadAssets() {
  downloadAsset(mBumpSound->value(), 0);
  downloadAsset(mRollSound->value(), 1);
  downloadAsset(mSlideSound->value(), 2);
}

void OmContactProperties::preFinalize() {
  OmBaseNode::preFinalize();

  updateCoulombFriction();
  updateBounce();
  updateBounceVelocity();
  updateSoftCfm();
  updateSoftErp();
  updateBumpSound();
  updateRollSound();
  updateSlideSound();
  updateMaxContactJoints();
}

void OmContactProperties::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mMaterial1, &OmSFString::changed, this, &OmContactProperties::valuesChanged);
  connect(mMaterial2, &OmSFString::changed, this, &OmContactProperties::valuesChanged);
  connect(mCoulombFriction, &OmSFDouble::changed, this, &OmContactProperties::updateCoulombFriction);
  connect(mFrictionRotation, &OmSFVector2::changed, this, &OmContactProperties::updateFrictionRotation);
  connect(mRollingFriction, &OmSFVector3::changed, this, &OmContactProperties::updateRollingFriction);
  connect(mBounce, &OmSFDouble::changed, this, &OmContactProperties::updateBounce);
  connect(mBounceVelocity, &OmSFDouble::changed, this, &OmContactProperties::updateBounceVelocity);
  connect(mForceDependentSlip, &OmSFDouble::changed, this, &OmContactProperties::updateForceDependentSlip);
  connect(mSoftErp, &OmSFDouble::changed, this, &OmContactProperties::updateSoftErp);
  connect(mSoftCfm, &OmSFDouble::changed, this, &OmContactProperties::updateSoftCfm);
  connect(mBumpSound, &OmSFString::changed, this, &OmContactProperties::updateBumpSound);
  connect(mRollSound, &OmSFString::changed, this, &OmContactProperties::updateRollSound);
  connect(mSlideSound, &OmSFString::changed, this, &OmContactProperties::updateSlideSound);
  connect(this, &OmContactProperties::needToEnableBodies, this, &OmContactProperties::enableBodies);
  connect(mMaxContactJoints, &OmSFInt::changed, this, &OmContactProperties::updateMaxContactJoints);
}

void OmContactProperties::exportNodeFields(OmWriter &writer) const {
  OmBaseNode::exportNodeFields(writer);

  exportSFResourceField(gUrlNames[0], mBumpSound, writer.relativeSoundsPath(), writer);
  exportSFResourceField(gUrlNames[1], mRollSound, writer.relativeSoundsPath(), writer);
  exportSFResourceField(gUrlNames[2], mSlideSound, writer.relativeSoundsPath(), writer);
}

QStringList OmContactProperties::customExportedFields() const {
  QStringList fields;
  fields << "url";
  return fields;
}

void OmContactProperties::updateCoulombFriction() {
  const int nbElements = mCoulombFriction->size();
  if (nbElements < 1 || nbElements > 4) {
    parsingWarn(tr("'coulombFriction' must have between one and four elements"));
    return;
  }

  for (int c = 0; c < nbElements; ++c) {
    const double cf = mCoulombFriction->item(c);
    if (cf != -1.0 && cf < 0.0) {
      parsingWarn(tr("If not set to -1 (infinity), 'coulombFriction' must be non-negative. Field value reset to 1"));
      mCoulombFriction->setItem(c, 1.0);
      return;
    }
  }

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void OmContactProperties::updateFrictionRotation() {
  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void OmContactProperties::updateRollingFriction() {
  const OmVector3 &rf = mRollingFriction->value();
  if ((rf[0] != -1.0 && rf[0] < 0.0) || (rf[1] != -1.0 && rf[1] < 0.0) || (rf[2] != -1.0 && rf[2] < 0.0)) {
    parsingWarn(tr("'rollingFriction' values must be positive or -1.0. Field value reset to 0 0 0."));
    mRollingFriction->setValue(0, 0, 0);
    return;
  }

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void OmContactProperties::updateBounce() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mBounce, 0.0, 1.0, 0.5))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void OmContactProperties::updateBounceVelocity() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mBounceVelocity, 0.01))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void OmContactProperties::updateForceDependentSlip() {
  const int nbElements = mForceDependentSlip->size();
  if (nbElements < 1 || nbElements > 4) {
    parsingWarn(tr("'forceDependentSlip' must have between one and four elements"));
    return;
  }

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void OmContactProperties::updateSoftCfm() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mSoftCfm, 0.001))
    return;
  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void OmContactProperties::updateSoftErp() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mSoftErp, 0.0, 1.0, 0.2))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void OmContactProperties::updateMaxContactJoints() {
  OmFieldChecker::resetIntIfNonPositive(this, mMaxContactJoints, 10);
}

void OmContactProperties::loadSound(int index, const QString &sound, const QString &name, const OmSoundClip **clip) {
  if (sound.isEmpty()) {
    *clip = NULL;
    return;
  }

  const QString completeUrl = OmUrl::computePath(this, gUrlNames[index], sound, true);
  if (OmUrl::isWeb(completeUrl)) {
    if (mDownloader[index] && !mDownloader[index]->error().isEmpty()) {
      warn(mDownloader[index]->error());  // failure downloading or file does not exist (404)
      *clip = NULL;
      // downloader needs to be deleted in case the URL is switched back to something valid
      delete mDownloader[index];
      mDownloader[index] = NULL;
      return;
    }
    if (!OmNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
      downloadAsset(completeUrl, index);  // changed by supervisor
      return;
    }
  }

  OmSoundEngine::clearAllContactSoundSources();
  // determine extension from URL since for remotely defined assets the cached version doesn't retain this information
  const QString extension = sound.mid(sound.lastIndexOf('.') + 1).toLower();

  if (OmUrl::isWeb(completeUrl)) {
    assert(OmNetwork::instance()->isCachedNoMapUpdate(completeUrl));  // by this point, the asset should be cached
    *clip = OmSoundEngine::sound(OmNetwork::instance()->get(completeUrl), extension);
  }
  // completeUrl can contain missing_texture.png if the user inputs an invalid .png or .jpg file in the field. In this case,
  // clip should be set to NULL
  else if (!(completeUrl.isEmpty() || completeUrl == OmUrl::missingTexture()))
    *clip = OmSoundEngine::sound(OmUrl::computePath(this, name, completeUrl, true), extension);
  else
    *clip = NULL;
}

void OmContactProperties::updateBumpSound() {
  loadSound(0, mBumpSound->value(), "bumpSound", &mBumpSoundClip);
}

void OmContactProperties::updateRollSound() {
  loadSound(1, mRollSound->value(), "rollSound", &mRollSoundClip);
}

void OmContactProperties::updateSlideSound() {
  loadSound(2, mSlideSound->value(), "slideSound", &mSlideSoundClip);
}

void OmContactProperties::enableBodies() {
  // when a value is changed, we need to re-enable the bodies,
  // otherwise a sleeping body would keep its old contact parameters
  OmWorld::instance()->awake();
}
