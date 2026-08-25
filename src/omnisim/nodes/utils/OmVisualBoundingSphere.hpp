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

#ifndef OM_VISUAL_BOUNDING_SPHERE_HPP
#define OM_VISUAL_BOUNDING_SPHERE_HPP

//
// Description: Utility class to debug bounding spheres by graphically
//              displaying the sphere in the 3D scene.
//

#include "QtCore/QObject"
#include "OmVector3.hpp"

#include <vector>

class OmBaseNode;

class OmVisualBoundingSphere : public QObject {
  Q_OBJECT

public:
  static OmVisualBoundingSphere *instance();
  static void deleteInstance();

  static void enable(bool enabled, const OmBaseNode *node = NULL);

  // W4a overlay port (lane E4): the wgpu main view has no WREN scene, so the sphere is
  // ALSO emitted as overlay line segments (three world-space great circles) through the
  // same enable/show state machine the WREN visual uses. overlayEnabled() is the cheap
  // per-frame gate; collect recomputes center/radius from the tracked node's live
  // bounding sphere every call, so the visual follows a moving node.
  // Layout: drawOverlayLines' stride 32 (pos3 + 5 ignored floats), two vertices/segment.
  static bool overlayEnabled();
  static void collectOverlayCircles(std::vector<float> &out);

public slots:
  void show(const OmBaseNode *node);

private:
  static OmVisualBoundingSphere *cInstance;

  OmVisualBoundingSphere();
  ~OmVisualBoundingSphere();

  // the node whose sphere is currently shown (NULL = nothing); tracked for the wgpu
  // overlay collector, cleared automatically when the node is destroyed
  const OmBaseNode *mNode;
};

#endif  // OM_VISUAL_BOUNDING_SPHERE_HPP
