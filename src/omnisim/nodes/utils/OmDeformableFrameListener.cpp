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

#include "OmDeformableFrameListener.hpp"

#include "OmCloth.hpp"
#include "OmMuscle.hpp"
#include "OmSimulationState.hpp"
#include "OmSoftBody.hpp"
#include "OmTrack.hpp"


OmDeformableFrameListener *OmDeformableFrameListener::cInstance = NULL;
static double lastUpdateTime = -1.0;

void OmDeformableFrameListener::resetLastUpdateTime() {
  lastUpdateTime = -1.0;
}

OmDeformableFrameListener *OmDeformableFrameListener::instance() {
  if (!cInstance)
    cInstance = new OmDeformableFrameListener();

  return cInstance;
}

void OmDeformableFrameListener::clear() {
  delete cInstance;
  cInstance = NULL;
}

OmDeformableFrameListener::OmDeformableFrameListener() : mListening(false) {
}

OmDeformableFrameListener::~OmDeformableFrameListener() {
  mTrackList.clear();
  mMuscleList.clear();
  mClothList.clear();
  mSoftBodyList.clear();
  mListening = false;
}

// cppcheck-suppress constParameterPointer
void OmDeformableFrameListener::subscribeTrack(OmTrack *track) {
  if (mTrackList.contains(track))
    return;
  mTrackList.append(track);
  updateListening();
}

// cppcheck-suppress constParameterPointer
void OmDeformableFrameListener::subscribeMuscle(OmMuscle *muscle) {
  if (mMuscleList.contains(muscle))
    return;
  mMuscleList.append(muscle);
  updateListening();
}

// cppcheck-suppress constParameterPointer
void OmDeformableFrameListener::subscribeCloth(OmCloth *cloth) {
  if (mClothList.contains(cloth))
    return;
  mClothList.append(cloth);
  updateListening();
}

// cppcheck-suppress constParameterPointer
void OmDeformableFrameListener::unsubscribeCloth(OmCloth *cloth) {
  mClothList.removeAll(cloth);
  updateListening();
}

// cppcheck-suppress constParameterPointer
void OmDeformableFrameListener::subscribeSoftBody(OmSoftBody *softBody) {
  if (mSoftBodyList.contains(softBody))
    return;
  mSoftBodyList.append(softBody);
  updateListening();
}

// cppcheck-suppress constParameterPointer
void OmDeformableFrameListener::unsubscribeSoftBody(OmSoftBody *softBody) {
  mSoftBodyList.removeAll(softBody);
  updateListening();
}

// cppcheck-suppress constParameterPointer
void OmDeformableFrameListener::unsubscribeTrack(OmTrack *track) {
  mTrackList.removeAll(track);
  updateListening();
}

// cppcheck-suppress constParameterPointer
void OmDeformableFrameListener::unsubscribeMuscle(OmMuscle *muscle) {
  mMuscleList.removeAll(muscle);
  updateListening();
}

void OmDeformableFrameListener::frameStarted() {
  for (int i = 0; i < mTrackList.size(); ++i)
    mTrackList[i]->animateMesh();
  mTrackList.clear();
  for (int i = 0; i < mMuscleList.size(); ++i)
    mMuscleList[i]->animateMesh();
  mMuscleList.clear();
  // ⚠ NOT cleared, unlike the two above. A cloth's vertices move on every step,
  // so it is subscribed once at createWrenObjects() and stays subscribed until
  // it is destroyed -- there is no field-change event that would re-subscribe
  // it, and clearing here would stream exactly one frame and then freeze the
  // patch in place.
  for (int i = 0; i < mClothList.size(); ++i)
    mClothList[i]->animateMesh();
  // Same reasoning, same non-clearing loop: a soft body's particles move every
  // step, and it subscribes once, from the point at which its mesh exists.
  for (int i = 0; i < mSoftBodyList.size(); ++i)
    mSoftBodyList[i]->animateMesh();
}

bool OmDeformableFrameListener::anyDeformablesSubscribed() {
  // No instance() call: a world without deformables must not pay a singleton
  // allocation for asking, and must not create a listener that then has to be torn
  // down. cInstance is main-thread-only state, same as every other member here.
  return cInstance != NULL && (!cInstance->mClothList.isEmpty() || !cInstance->mSoftBodyList.isEmpty());
}

void OmDeformableFrameListener::animateTracksAndMuscles() {
  // frameStarted()'s first two loops, and ONLY those two -- see the header for why the
  // clearing is right here and wrong for the deformables.
  for (int i = 0; i < mTrackList.size(); ++i)
    mTrackList[i]->animateMesh();
  mTrackList.clear();
  for (int i = 0; i < mMuscleList.size(); ++i)
    mMuscleList[i]->animateMesh();
  mMuscleList.clear();
}

void OmDeformableFrameListener::animateDeformables() {
  for (int i = 0; i < mClothList.size(); ++i)
    mClothList[i]->animateMesh();
  for (int i = 0; i < mSoftBodyList.size(); ++i)
    mSoftBodyList[i]->animateMesh();
}

void OmDeformableFrameListener::updateListening() {
  bool need =
    mMuscleList.size() > 0 || mTrackList.size() > 0 || mClothList.size() > 0 || mSoftBodyList.size() > 0;
  // D1.4: the wr_scene frame-listener registration is gone with WREN. The wgpu pumps
  // (animateTracksAndMuscles / the collectDynamicDraws deformable pump) are the only
  // drivers now; mListening just mirrors whether any subscriber exists.
  mListening = need;
}
