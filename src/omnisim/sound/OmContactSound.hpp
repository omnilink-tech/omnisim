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

#ifndef OM_CONTACT_SOUND_HPP
#define OM_CONTACT_SOUND_HPP

//
// Description: manage a sound contact
//

#include "OmOdeTypes.hpp"  // opaque handles + the dContactGeom value mirror

#include <OmVector3.hpp>

class OmContactProperties;
class OmSoundClip;
class OmSoundSource;

class OmContactSound {
public:
  OmContactSound(const dGeomID &geom1, const dGeomID &geom2, const OmContactProperties *contactProperties);
  virtual ~OmContactSound();

  bool doesGeomsMatch(const dGeomID &geom1, const dGeomID &geom2) const;

  void newOdeContact(const dContactGeom &c);
  double lastContactTime() const { return mContactTime; }
  double lastUpdateTime() const { return mUpdateTime; }

  void finalizeContactUpdate();
  void updateSource();

private:
  void updateVelocities();

  enum Type { BUMP, ROLL, SLIDE, NONE };
  Type mType;

  dGeomID mGeom1;
  dGeomID mGeom2;
  dBodyID mBody1;
  dBodyID mBody2;

  double mContactTime;  // in seconds
  double mUpdateTime;   // in seconds

  double mEnergy;
  double mDerivativeEnergy;
  double mDoubleDerivativeEnergy;

  double mMass1;
  double mMass2;

  OmVector3 mContactPosition;
  OmVector3 mContactVelocity;

  OmVector3 mLinearVelocity1;
  OmVector3 mLinearVelocity2;
  OmVector3 mAngularVelocity1;
  OmVector3 mAngularVelocity2;

  OmSoundSource *mSource;

  const OmSoundClip *mBumpSoundClip;
  const OmSoundClip *mRollSoundClip;
  const OmSoundClip *mSlideSoundClip;
};

#endif
