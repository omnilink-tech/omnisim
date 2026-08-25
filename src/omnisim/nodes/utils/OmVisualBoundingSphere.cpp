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

#include "OmVisualBoundingSphere.hpp"

#include "OmBaseNode.hpp"
#include "OmBoundingSphere.hpp"
#include "OmSimulationState.hpp"

#include <cmath>

static bool gEnabled = false;

OmVisualBoundingSphere *OmVisualBoundingSphere::cInstance = NULL;

OmVisualBoundingSphere *OmVisualBoundingSphere::instance() {
  if (!cInstance)
    cInstance = new OmVisualBoundingSphere();
  return cInstance;
}

void OmVisualBoundingSphere::deleteInstance() {
  if (!cInstance)
    return;
  delete cInstance;
  cInstance = NULL;
}

void OmVisualBoundingSphere::enable(bool enabled, const OmBaseNode *node) {
  gEnabled = enabled;
  if (node || cInstance)
    cInstance->show(node);
}

OmVisualBoundingSphere::OmVisualBoundingSphere() :
  QObject(),
  mNode(NULL) {
  // make sure the bounding spheres are updates when node's position and size changes
  OmSimulationState::instance()->subscribeToRayTracing();
}

OmVisualBoundingSphere::~OmVisualBoundingSphere() {
  OmSimulationState::instance()->unsubscribeToRayTracing();
}

void OmVisualBoundingSphere::show(const OmBaseNode *node) {
  // Track the node for the wgpu overlay collector (collectOverlayCircles reads it per frame).
  // The overlay path holds the pointer across frames, hence the destroyed() hook.
  if (node != mNode) {
    if (mNode)
      disconnect(mNode, &QObject::destroyed, this, nullptr);
    mNode = node;
    if (mNode)
      connect(mNode, &QObject::destroyed, this, [this]() { mNode = NULL; });
  }
  // D1.4: the WREN sphere renderable is gone; the wgpu overlay
  // (collectOverlayCircles, gated by gEnabled through overlayEnabled())
  // is the drawer now.
}

bool OmVisualBoundingSphere::overlayEnabled() {
  return gEnabled && cInstance && cInstance->mNode;
}

void OmVisualBoundingSphere::collectOverlayCircles(std::vector<float> &out) {
  if (!overlayEnabled())
    return;
  const OmBoundingSphere *boundingSphere = cInstance->mNode->boundingSphere();
  if (!boundingSphere)
    return;
  OmVector3 center;
  double radius;
  boundingSphere->computeSphereInGlobalCoordinates(center, radius);
  if (!(radius > 0.0))
    return;
  // Three world-axis great circles. WREN drew a 16-subdivision wireframe unit sphere; the
  // overlay-line equivalent keeps the same silhouette information at 48 segments per circle.
  const int steps = 48;
  auto pushVertex = [&out](const OmVector3 &p) {
    out.push_back(static_cast<float>(p.x()));
    out.push_back(static_cast<float>(p.y()));
    out.push_back(static_cast<float>(p.z()));
    for (int k = 0; k < 5; ++k)
      out.push_back(0.0f);  // drawOverlayLines' stride-32 padding
  };
  for (int axis = 0; axis < 3; ++axis) {
    // unit vectors spanning the plane orthogonal to `axis`
    const OmVector3 u(axis == 0 ? 0.0 : 1.0, axis == 0 ? 1.0 : 0.0, 0.0);
    const OmVector3 v(0.0, axis == 2 ? 1.0 : 0.0, axis == 2 ? 0.0 : 1.0);
    OmVector3 prev = center + u * radius;
    for (int i = 1; i <= steps; ++i) {
      const double a = (2.0 * M_PI * i) / steps;
      const OmVector3 p = center + u * (radius * cos(a)) + v * (radius * sin(a));
      pushVertex(prev);
      pushVertex(p);
      prev = p;
    }
  }
}
