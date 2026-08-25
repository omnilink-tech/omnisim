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

#ifndef OM_GIZMO_LINES_HPP
#define OM_GIZMO_LINES_HPP

//
// OmGizmoLines -- P8 of the WREN-deletion runbook: the MAIN VIEW's translate/rotate manipulator,
// drawn where its hit regions actually are.
//
// WHY THIS AND NOT THE PANE'S COLLECTORS. OmWgpuView (the opt-in wgpu pane) already carries
// translate/rotate handle collectors with a verified hit test -- pickHandleAxis() /
// pickRotateRing(), a fixed kHandleLen = 0.25 world metres and a screen-distance test. They were
// deliberately never drawn in the MAIN view, and that was the right call: the main view's gizmo
// is a DIFFERENT gizmo. It is OmTranslateRotateManipulator, a WREN scene object whose hit test is
// OmWrenPicker -- a GL picking render of the actual handle meshes through
// resources/wren/shaders/handles.vert. Drawing the pane's gizmo in the main view would put
// visible handles at 0.25 m from the origin while the draggable regions sit wherever that shader
// put them: two gizmos, one visible, the other clickable.
//
// So this module draws the MAIN VIEW'S OWN gizmo. It reads:
//   * wr_transform_get_matrix() on the very handle transforms the picking renderables hang off --
//     which returns exactly the `modelTransform` uniform the picking shader receives, pose chain,
//     manipulator unscale, per-axis rotation and sort offset included; and
//   * OmWrenAbstractManipulator::handleScreenScale(), the `screenScale` uniform that same shader
//     multiplies every vertex by,
// and then reproduces handles.vert's scalar exactly:
//     w = screenScale * (camera-space depth of modelTransform[3]) / min(proj[0][0], proj[1][1])
// Drawn position and draggable position are therefore the same computation over the same inputs,
// not two formulas that agree today.
//
// WHAT IS DRAWN: the SILHOUETTE of each handle mesh (arrow.obj for translate, circular_arrow.obj
// for rotate) -- the outline of exactly the triangle set the picking pass rasterises -- plus the
// three unit axis lines WREN draws as decoration. The axis lines are NOT pickable in WREN either
// and are not claimed to be here.
//
// Deliberately not a Q_OBJECT -> OTHER_SOURCES, no moc.
//

#include <vector>

namespace OmGizmoLines {

  // One handle's world-space geometry, for the hit-test verification instrument. `tris` is
  // 9 floats per triangle (3 vertices x xyz), already scaled and transformed -- i.e. the exact
  // triangles the picking pass rasterises for this handle.
  struct Handle {
    int axis = 0;          // 0 = X, 1 = Y, 2 = Z
    bool rotate = false;   // false = translate arrow, true = rotation ring
    std::vector<float> tris;
  };

  // Is a translate/rotate manipulator attached to the current selection? One pointer walk.
  bool anyVisible();

  // Diagnostic: handles.vert's screen-constant scalar `w` for the live gizmo, i.e. how big one
  // object-space unit of handle is in world metres this frame. Negative means the gizmo origin is
  // BEHIND the camera, which is the only legitimate reason collect() emits nothing while
  // anyVisible() is true -- so the verification report can attribute an empty result instead of
  // leaving it unexplained.
  double debugHandleScale(const float view16[16], double projMin);

  // Collect the live gizmo.
  //   view16   column-major view matrix (the same one the drawing viewProj was built from)
  //   projMin  min(proj[0][0], proj[1][1]) of that projection -- handles.vert's own divisor
  //   eye3     camera world position, for the silhouette facing test
  //   outX/outY/outZ  per-axis line vertices in drawOverlayLines' layout (stride 32: pos3 + 5
  //                   ignored floats, TWO vertices per segment)
  //   handlesOut  when non-null, also emit the per-handle triangle sets (verification only;
  //               skipped otherwise, so the per-frame path never pays for it)
  void collect(const float view16[16], double projMin, const float eye3[3],
               std::vector<float> &outX, std::vector<float> &outY, std::vector<float> &outZ,
               std::vector<Handle> *handlesOut);

}  // namespace OmGizmoLines

#endif
