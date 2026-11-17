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

#ifndef OM_DRAG_ARROW_LINES_HPP
#define OM_DRAG_ARROW_LINES_HPP

//
// OmDragArrowLines -- Phase D (WREN-deletion runbook, lane E4): the force/torque DRAG ARROW,
// drawn on the wgpu main view through the W4a overlay-line path.
//
// WHAT THIS IS. Alt-dragging a Solid applies a force (or, with the torque mouse button, a
// torque), and the user aims it by an arrow from the application point to the mouse ray. That
// arrow is OmPhysicsVectorRepresentation -- a WREN scene object (76 wr_* sites), so under a wgpu
// main view it went dark: the drag WORKS but the user aims blind. This module redraws the same
// visual as overlay line segments, the mechanism OmGizmoLines (P8) established.
//
// GEOMETRY CONTRACT: this is a REPRODUCTION of the WREN representation, term for term, over the
// SAME inputs the WREN arrows read (OmDragPhysicsEvent's mOrigin / mEnd, the viewpoint
// orientation, the drag's view-distance scaling, the viewport size):
//   * tail            OmPhysicsVectorRepresentation::updatePosition's unit line, i.e. one
//                     segment origin -> end;
//   * head            the 4-vertex arrowhead kite of initializeTailAndArrow, drawn as its
//                     perimeter, positioned at `end`, oriented by updatePosition's exact
//                     basis construction, scaled by setScale's screen-constant factor
//                     (viewDistanceScaling * 2 / max(viewportW, viewportH));
//   * torque coil     initializeSpinSymbol's 32-step helix + its small arrowhead, both in the
//                     tail frame (origin at `begin`, scaled by |end - begin|), constants
//                     (0.1 radius, 0.1 height, 1.25 revolutions, 0.4 start height, 0.02 arrow
//                     scale, x = -(0.02 * 0.9) arrow offset) copied verbatim.
// The WREN implementation itself is byte-untouched and stays until D1.4, like every other
// WREN twin of a working wgpu path.
//
// The one deliberate deviation: at |end - begin| < 1e-12 (the instant the drag starts) nothing
// is emitted. WREN drew a zero-scaled, invisibly degenerate arrow there; emitting nothing is
// the same pixel result without the NaN basis.
//
// Deliberately not a Q_OBJECT -> OTHER_SOURCES, no moc.
//

#include <vector>

class OmDragPhysicsEvent;
class OmRotation;

namespace OmDragArrowLines {

  // Collect the live drag arrow into drawOverlayLines' vertex layout (stride 32: pos3 + 5
  // ignored floats, TWO vertices per segment).
  //   event            the active force/torque drag (caller passes mDragForce or mDragTorque)
  //   orientation      the live viewpoint orientation -- updatePosition's own aiming input
  //   viewportWidthPx / viewportHeightPx
  //                    the rendered pane size in px -- setScale's own scale input
  //   outArrow         tail + head silhouette (colour: the caller draws force orange /
  //                    torque dark-yellow, matching the WREN materials)
  //   outCoil          torque only: the spin helix + its arrowhead (WREN colours it pure
  //                    yellow); untouched for a force drag
  void collect(const OmDragPhysicsEvent *event, const OmRotation &orientation,
               int viewportWidthPx, int viewportHeightPx,
               std::vector<float> &outArrow, std::vector<float> &outCoil);

}  // namespace OmDragArrowLines

#endif
