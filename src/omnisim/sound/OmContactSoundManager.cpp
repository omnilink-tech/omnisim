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

#include "OmContactSoundManager.hpp"

#include "OmContactSound.hpp"
#include "OmOdeContact.hpp"
#include "OmSimulationState.hpp"

#include <QtCore/QCoreApplication>

QList<OmContactSound *> gContactSounds;

static OmContactSound *findContactFromGeoms(const dGeomID &geom1, const dGeomID &geom2) {
  foreach (OmContactSound *contactSound, gContactSounds) {
    if (contactSound->doesGeomsMatch(geom1, geom2))
      return contactSound;
  }
  return NULL;
}

static void newOdeContact(const OmOdeContact &odeContact) {
  OmContactSound *contactSound = findContactFromGeoms(odeContact.contactGeom().g1, odeContact.contactGeom().g2);

  if (contactSound == NULL) {
    contactSound = new OmContactSound(odeContact.contactGeom().g1, odeContact.contactGeom().g2, odeContact.contactProperties());
    gContactSounds << contactSound;
  }

  contactSound->newOdeContact(odeContact.contactGeom());
}

static void removeObsoleteContacts() {
  double currentTime = 0.001 * OmSimulationState::instance()->time();

  QMutableListIterator<OmContactSound *> it(gContactSounds);
  while (it.hasNext()) {
    OmContactSound *contactSound = it.next();
    if (contactSound->lastContactTime() + 0.05 < currentTime) {
      it.remove();
      delete contactSound;
    }
  }
}

void OmContactSoundManager::clearAllContactSoundSources() {
  qDeleteAll(gContactSounds);
  gContactSounds.clear();
}

void OmContactSoundManager::update(const QList<OmOdeContact> &odeContacts) {
  foreach (const OmOdeContact &odeContact, odeContacts)
    newOdeContact(odeContact);

  foreach (OmContactSound *contactSound, gContactSounds)
    contactSound->finalizeContactUpdate();

  removeObsoleteContacts();

  foreach (OmContactSound *contactSound, gContactSounds)
    contactSound->updateSource();
}
