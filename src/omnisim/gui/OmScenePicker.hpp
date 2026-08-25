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

#ifndef OM_SCENE_PICKER_HPP
#define OM_SCENE_PICKER_HPP

//
// OmScenePicker -- D1.4's replacement for OmWrenPicker: the main view's mouse hit test.
//
// WHAT IT REPRODUCES. OmWrenPicker rendered the WREN scene with an ID-encoding "picking"
// material into its own framebuffer and decoded the texel under the cursor. This class does
// the same through wgpu: OmWgpuSceneRenderer::collectWorldDraws() (with the per-draw Solid
// map), draw-index IDs encoded into baseColor, rendered flat through kSolidPick
// (OmWgpuRenderTarget::clearAndDrawScene pickMode) into an offscreen target sized to the 3D
// view, one readback, one texel decode. The manipulator HANDLES are picked through the SAME
// depth-tested render: OmGizmoLines::collect() emits the exact world triangles it draws, and
// those triangles are appended as transient ID draws -- so "drawn where it grabs" is one
// triangle set by construction, and handle-vs-scene occlusion resolves in the depth buffer
// exactly as WREN's picking pass (which also depth-tested handles against the scene) did.
//
// KNOWN DEVIATIONS from OmWrenPicker, deliberate and recorded:
//  * Selection granularity is the SOLID the collector records per draw, not the leaf
//    geometry: OmView3D's findUpperMatter() then lands on that Solid. Clicking a nested
//    sub-device (e.g. a bumper pad inside a robot) selects/reports the recorded solid.
//  * Resize/scale handles are neither drawn (P8's documented leftover) nor pickable --
//    under WREN they were invisible-but-draggable on the wgpu main view, which P8 called
//    the regression class. Resize via the scene-tree fields is unaffected.
//  * pickedScaleHandle()/pickedResizeHandle() therefore always report 0.
//
// COORDINATES. Instead of reading the pick depth buffer back (OmWrenPicker's
// copy_depth_pixel), the world-space hit point is computed by a CPU ray-triangle
// intersection against the picked solid's own draws (cpuPositions/cpuIndices retained in
// the mesh cache), with the ray derived from inverse(viewProj) -- convention-free and
// exact on the mesh the GPU rasterised.
//

#include "OmVector3.hpp"

#include <cstdint>
#include <vector>

class OmVulkanBackend;
class OmWgpuMeshCache;
class OmWgpuRenderTarget;

class OmScenePicker {
public:
  // Reserved unique IDs used by the handles -- OmWrenPicker's own values, kept because the
  // drag events and view code branch on the same constants.
  static const int HANDLES_X_AXIS = 0x7FFFFFF1, HANDLES_Y_AXIS = 0x7FFFFFF2, HANDLES_Z_AXIS = 0x7FFFFFF3,
                   HANDLES_TRANSLATE = 0x7FFFFFF0, HANDLES_ROTATE = 0x7FFFFFF4, HANDLES_SCALE = 0x7FFFFFF8,
                   HANDLES_RESIZE = 0x7FFFFFFC;

  OmScenePicker();
  ~OmScenePicker();

  // Share the main view's mesh cache so the pick render reuses the already-uploaded scene
  // buffers instead of duplicating every mesh in VRAM. Optional: without it the picker lazily
  // owns a private cache. The pointer is non-owning; pass NULL when the cache dies.
  void setSharedMeshCache(OmWgpuMeshCache *cache) { mSharedCache = cache; }

  // x/y in 3D-view logical pixels, origin top-left (the mouse handlers' frame).
  bool pick(int x, int y);

  // World-space coordinates of the picked point (valid when the last pick() returned true
  // with a scene hit, i.e. selectedId() != -1).
  const OmVector3 &worldCoordinates() const { return mWorldCoordinates; }

  const int &selectedId() const { return mSelectedId; }

  int pickedTranslateHandle() const { return mPickedTranslation; }
  int pickedRotateHandle() const { return mPickedRotation; }
  int pickedScaleHandle() const { return mPickedScale; }
  int pickedResizeHandle() const { return mPickedResize; }

private:
  OmScenePicker(const OmScenePicker &);
  OmScenePicker &operator=(const OmScenePicker &);

  OmWgpuMeshCache *ensureCache();

  OmVulkanBackend *mBackend;      // non-owning, resolved lazily
  OmWgpuMeshCache *mSharedCache;  // non-owning
  OmWgpuMeshCache *mOwnCache;     // owned fallback
  OmWgpuRenderTarget *mTarget;    // owned, recreated on view resize
  int mTargetWidth;
  int mTargetHeight;
  std::vector<uint8_t> mReadback;
  // Transient handle-triangle uploads: reserved keys + last vertex byte length per slot, so
  // updateVertices() can be used when the size is unchanged and release+acquire when not.
  std::vector<size_t> mHandleSlotBytes;

  OmVector3 mWorldCoordinates;
  int mSelectedId;

  int mPickedTranslation;
  int mPickedRotation;
  int mPickedScale;
  int mPickedResize;
};

#endif  // OM_SCENE_PICKER_HPP
