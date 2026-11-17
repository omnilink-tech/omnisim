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

#ifndef OM_DEFORMABLE_FRAME_LISTENER
#define OM_DEFORMABLE_FRAME_LISTENER

//
// OmDeformableFrameListener (until D1.1 of the WREN-deletion runbook:
// OmWrenVertexArrayFrameListener) -- the ONLY registry of live Cloth / SoftBody / Muscle /
// Track nodes, which BOTH renderers pump every frame. It was renamed rather than deleted
// because deleting it re-breaks d5897ff7d: cloth freezes at its rest pose, compiles clean,
// and looks right in one screenshot. The three wr_scene_* lines in the .cpp are the last of
// its WREN and go with D1.4; the registry itself must survive the deletion.
//

#include <QtCore/QList>

class OmCloth;
class OmMuscle;
class OmSoftBody;
class OmTrack;

class OmDeformableFrameListener {
public:
  static OmDeformableFrameListener *instance();
  static void clear();
  static void resetLastUpdateTime();

  void subscribeTrack(OmTrack *track);
  void unsubscribeTrack(OmTrack *track);

  void subscribeMuscle(OmMuscle *muscle);
  void unsubscribeMuscle(OmMuscle *muscle);

  // ⚠ DEFORMABLE SUBSCRIPTIONS ARE PERSISTENT; track and muscle subscriptions
  // are NOT. frameStarted() CLEARS the track and muscle lists after running
  // them, because those two are event-driven -- they re-subscribe from their own
  // field-change / update paths whenever their geometry actually needs
  // rebuilding. A cloth's or a soft body's geometry changes on every single step
  // by definition, so each stays subscribed until it is destroyed and never
  // re-subscribes.
  void subscribeCloth(OmCloth *cloth);
  void unsubscribeCloth(OmCloth *cloth);

  // A parallel list rather than a shared base: OmCloth and OmSoftBody are
  // siblings under OmBaseNode with no common deformable base to hold
  // animateMesh(), and inventing one to save a QList here would be a wider change
  // than the thing it saves. Same persistent, never-cleared semantics as cloth.
  void subscribeSoftBody(OmSoftBody *softBody);
  void unsubscribeSoftBody(OmSoftBody *softBody);

  void frameStarted();

  // ---- THE DEFORMABLE REGISTRY (wgpu main view) ----------------------------
  //
  // ⚠ THIS CLASS IS THE ONLY REGISTRY OF LIVE Cloth / SoftBody NODES, and the
  // wgpu main view needs one for two reasons at once. (1) It must DRAW them, and
  // a per-frame scene walk to find them would cost every world that has none.
  // (2) It must ANIMATE them: frameStarted() is a wr_scene frame listener, so it
  // fires from inside wr_scene_render -- which the wgpu main view never calls, so
  // under wgpu the deformables were frozen at their rest pose even before the
  // draw path existed. Both subscriptions are made from createWrenObjects(),
  // which runs on every renderer, so the lists are correct either way.
  //
  // The name says WREN because the class does; the LIST does not. Renaming the
  // class is WREN-retirement work and is deliberately not done here.
  //
  // anyDeformablesSubscribed() is a STATIC that does NOT construct the singleton:
  // a world with no Cloth and no SoftBody must pay one null test per frame and
  // nothing else, and instance() would allocate on the first call.
  static bool anyDeformablesSubscribed();
  const QList<OmCloth *> &clothList() const { return mClothList; }
  const QList<OmSoftBody *> &softBodyList() const { return mSoftBodyList; }

  // Run animateMesh() on every subscribed deformable and NOTHING else -- no
  // tracks, no muscles, and no list clearing. The wgpu main view's substitute for
  // the wr_scene frame callback it never receives. Deliberately NOT frameStarted():
  // that one also drains the track + muscle lists, whose re-subscription is
  // event-driven, so calling it from a second driver would consume their pending
  // rebuilds on a frame that never rendered them.
  void animateDeformables();

  // ---- P2: the TRACK / MUSCLE pump for a renderer that never calls wr_scene_render ----
  //
  // Same problem animateDeformables() solves, different list semantics. frameStarted() is a
  // wr_scene frame listener, so under a wgpu main view it never fires and a Track's belt
  // never advances (OmTrack::animateMesh is also what DRAINS mAnimationStepSize, so the
  // accumulator would grow without bound) and a Muscle's mesh is never rebuilt.
  //
  // ⚠ THIS ONE CLEARS THE LISTS, AND animateDeformables() DELIBERATELY DOES NOT. A track /
  // muscle subscription is a PENDING-REBUILD request that its own event paths re-issue when
  // the geometry actually needs redoing, so consuming it is the contract; a cloth or a soft
  // body subscribes once for its lifetime, so clearing that would stream one frame and
  // freeze. The two drivers (this and WREN's processEvent) each own their own "has the
  // clock advanced" edge, and whichever runs first in a step legitimately consumes the
  // pending work -- both write the same WREN transforms and the same engine-side state, so
  // a session with BOTH a wgpu main view and a WREN camera device stays correct.
  void animateTracksAndMuscles();

private:
  static OmDeformableFrameListener *cInstance;

  OmDeformableFrameListener();
  ~OmDeformableFrameListener();

  void updateListening();

  QList<OmTrack *> mTrackList;
  QList<OmMuscle *> mMuscleList;
  QList<OmCloth *> mClothList;
  QList<OmSoftBody *> mSoftBodyList;

  bool mListening;
};

#endif
