// Copyright 2026 OmniLink
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

#include "OmDragArrowLines.hpp"

#include "OmDragSolidEvent.hpp"
#include "OmRotation.hpp"
#include "OmVector3.hpp"

#include <cmath>

namespace {

  // drawOverlayLines' vertex layout: pos3 + 5 ignored floats, two vertices per segment
  // (the same pushSeg OmWgpuView's collectors use).
  void pushSeg(std::vector<float> &v, const OmVector3 &a, const OmVector3 &b) {
    const OmVector3 e[2] = {a, b};
    for (const OmVector3 &p : e) {
      v.push_back(static_cast<float>(p.x()));
      v.push_back(static_cast<float>(p.y()));
      v.push_back(static_cast<float>(p.z()));
      for (int k = 0; k < 5; ++k)
        v.push_back(0.0f);
    }
  }

  // The arrowhead kite of OmPhysicsVectorRepresentation::initializeTailAndArrow, verbatim:
  // tip at the local origin, pointing +Y, barbs behind. Perimeter order tip -> left barb ->
  // notch -> right barb -> tip is the silhouette of exactly the two triangles WREN rasterises.
  const double kHead[4][2] = {{0.0, -0.9}, {-0.5, -1.0}, {0.0, 0.0}, {0.5, -1.0}};
  const int kHeadPerimeter[4][2] = {{2, 1}, {1, 0}, {0, 3}, {3, 2}};

}  // namespace

void OmDragArrowLines::collect(const OmDragPhysicsEvent *event, const OmRotation &orientation,
                               int viewportWidthPx, int viewportHeightPx,
                               std::vector<float> &outArrow, std::vector<float> &outCoil) {
  if (!event)
    return;

  const OmVector3 begin = event->dragOrigin();
  const OmVector3 end = event->dragEnd();
  const OmVector3 d = end - begin;
  const double len = d.length();
  if (len < 1e-12)
    return;  // drag just started: WREN drew a degenerate zero-length arrow, we draw nothing

  // updatePosition's exact basis construction: +Y along the vector, the other two axes chosen
  // against the camera so the flat arrowhead faces the viewer.
  OmVector3 baseX, baseY, baseZ;
  baseY = d.normalized();
  if (std::fabs(baseY.dot(orientation.direction())) < 1e-6)
    baseX = baseY.cross(orientation.direction()).normalized();
  else {
    baseZ = orientation.up().cross(baseY).normalized();
    baseX = baseY.cross(baseZ).normalized();
  }
  baseZ = baseX.cross(baseY).normalized();

  // local -> world, in a frame at `origin` with uniform scale `s` over the drag basis
  auto toWorld = [&](double lx, double ly, double lz, const OmVector3 &origin, double s) {
    return origin + baseX * (lx * s) + baseY * (ly * s) + baseZ * (lz * s);
  };

  // Tail: updatePosition's unit line under the tail transform == one world-space segment.
  pushSeg(outArrow, begin, end);

  // Head: setScale's screen-constant factor over the same inputs -- viewDistanceScaling is the
  // value the drag computed at start (viewDistanceUnscaling(solid) * 50 px), the divisor is the
  // live viewport's larger dimension, exactly as wr_viewport_get_width/height supplied it.
  const double maxDimension = viewportWidthPx > viewportHeightPx ? viewportWidthPx : viewportHeightPx;
  if (maxDimension <= 0.0)
    return;
  const double headScale = event->viewDistanceScaling() * 2.0 / maxDimension;
  for (int i = 0; i < 4; ++i) {
    const int a = kHeadPerimeter[i][0], b = kHeadPerimeter[i][1];
    pushSeg(outArrow, toWorld(kHead[a][0], kHead[a][1], 0.0, end, headScale),
            toWorld(kHead[b][0], kHead[b][1], 0.0, end, headScale));
  }

  if (!event->isTorqueDrag())
    return;

  // Torque spin symbol: initializeSpinSymbol's helix, verbatim constants. It hangs off the
  // TAIL transform in WREN, i.e. frame at `begin`, uniformly scaled by |end - begin|.
  const int steps = 32;
  const double coilHeight = 0.1;
  const double coilRadius = 0.1;
  const double revolutions = 1.25 * (2.0 * M_PI);
  const double coilStartHeight = 0.4;
  for (int i = 0; i < steps; ++i) {
    const double a0 = -i * (revolutions / steps);
    const double a1 = -(i + 1) * (revolutions / steps);
    const OmVector3 p0 = toWorld(-coilRadius * std::sin(a0), coilStartHeight + coilHeight - (coilHeight / steps) * i,
                                 coilRadius * std::cos(a0), begin, len);
    const OmVector3 p1 = toWorld(-coilRadius * std::sin(a1),
                                 coilStartHeight + coilHeight - (coilHeight / steps) * (i + 1),
                                 coilRadius * std::cos(a1), begin, len);
    pushSeg(outCoil, p0, p1);
  }

  // The coil's own arrowhead: WREN parks it at the helix top (first coil vertex, x overridden
  // to -(0.02 * 0.9)), rotated 90 deg about local Z (so (x, y) -> (-y, x)), scale 0.02 -- all
  // inside the tail frame, so the world scale is 0.02 * len.
  const double arrowFactor = 0.02;
  const double ax = -(arrowFactor * 0.9);
  const double ay = coilStartHeight + coilHeight;   // helix start vertex y
  const double az = coilRadius;                     // helix start vertex z (cos(0) * radius)
  for (int i = 0; i < 4; ++i) {
    const int a = kHeadPerimeter[i][0], b = kHeadPerimeter[i][1];
    // local kite vertex, rotated by Rz(pi/2): (x, y, 0) -> (-y, x, 0), then offset + scale
    const double lax = ax + arrowFactor * (-kHead[a][1]);
    const double lay = ay + arrowFactor * (kHead[a][0]);
    const double lbx = ax + arrowFactor * (-kHead[b][1]);
    const double lby = ay + arrowFactor * (kHead[b][0]);
    pushSeg(outCoil, toWorld(lax, lay, az, begin, len), toWorld(lbx, lby, az, begin, len));
  }
}
