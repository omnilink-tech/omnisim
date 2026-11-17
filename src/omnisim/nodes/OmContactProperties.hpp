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

#ifndef OM_CONTACT_PROPERTIES_HPP
#define OM_CONTACT_PROPERTIES_HPP

#include "OmBaseNode.hpp"
#include "OmMFDouble.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFString.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"

class OmSoundClip;
class OmDownloader;

class OmContactProperties : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmContactProperties(OmTokenizer *tokenizer = NULL);
  OmContactProperties(const OmContactProperties &other);
  explicit OmContactProperties(const OmNode &other);
  virtual ~OmContactProperties() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CONTACT_PROPERTIES; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  const QString &material1() const { return mMaterial1->value(); }
  const QString &material2() const { return mMaterial2->value(); }
  int coulombFrictionSize() const { return mCoulombFriction->size(); }
  double coulombFriction(int index) const { return mCoulombFriction->item(index); }
  OmVector2 frictionRotation() const { return mFrictionRotation->value(); }
  OmVector3 rollingFriction() const { return mRollingFriction->value(); }
  double bounce() const { return mBounce->value(); }
  double bounceVelocity() const { return mBounceVelocity->value(); }
  // internal parity plan, item W1.5. Both accessors above have ZERO callers and
  // structurally cannot gain one: restitution is an ODE-path concept and MuJoCo
  // has no coefficient of restitution at all -- our contacts map to the stock,
  // critically-damped solref (0.02, 1.0). So a world declaring a bounce is
  // declaring something the engine will never read. These say whether the
  // author actually asked for it (field written in the file) as opposed to
  // inheriting the .wrl default of 0.5, so the disclosure can be precise
  // instead of firing on all 322 worlds that carry a ContactProperties node.
  bool bounceIsAuthored() const;
  bool bounceVelocityIsAuthored() const;
  int forceDependentSlipSize() const { return mForceDependentSlip->size(); }
  double forceDependentSlip(int index) const { return mForceDependentSlip->item(index); }
  double softERP() const { return mSoftErp->value(); }
  double softCFM() const { return mSoftCfm->value(); }
  const OmSoundClip *bumpSoundClip() const { return mBumpSoundClip; }
  const OmSoundClip *rollSoundClip() const { return mRollSoundClip; }
  const OmSoundClip *slideSoundClip() const { return mSlideSoundClip; }
  int maxContactJoints() const { return mMaxContactJoints->value(); }

signals:
  void valuesChanged();
  void needToEnableBodies();

protected:
  void exportNodeFields(OmWriter &writer) const override;
  QStringList customExportedFields() const override;

private:
  // user accessible fields
  OmSFString *mMaterial1;
  OmSFString *mMaterial2;
  OmMFDouble *mCoulombFriction;
  OmSFVector2 *mFrictionRotation;
  OmSFVector3 *mRollingFriction;
  OmSFDouble *mBounce;
  OmSFDouble *mBounceVelocity;
  OmMFDouble *mForceDependentSlip;
  OmSFDouble *mSoftCfm;
  OmSFDouble *mSoftErp;
  OmSFString *mBumpSound;
  OmSFString *mRollSound;
  OmSFString *mSlideSound;
  OmSFInt *mMaxContactJoints;
  const OmSoundClip *mBumpSoundClip;
  const OmSoundClip *mRollSoundClip;
  const OmSoundClip *mSlideSoundClip;
  OmDownloader *mDownloader[3];
  OmContactProperties &operator=(const OmContactProperties &);  // non copyable
  OmNode *clone() const override { return new OmContactProperties(*this); }
  void init();
  void downloadAsset(const QString &url, int index);
  void loadSound(int index, const QString &sound, const QString &name, const OmSoundClip **clip);

private slots:
  void updateCoulombFriction();
  void updateFrictionRotation();
  void updateRollingFriction();
  void updateBounce();
  void updateBounceVelocity();
  void updateSoftCfm();
  void updateSoftErp();
  void updateBumpSound();
  void updateRollSound();
  void updateSlideSound();
  void updateForceDependentSlip();
  void enableBodies();
  void updateMaxContactJoints();
};

#endif
