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

#ifndef WB_WGPU_MESH_ADAPTER_HPP
#define WB_WGPU_MESH_ADAPTER_HPP

//
// WbWgpuMeshAdapter — R3.2b of engine-migration-plan.md §14.3.
//
// Glue layer between WREN's WrStaticMesh API (which exposes
// coordinates + normals + UVs + indices as four separate float/uint
// arrays through wr_static_mesh_read_data) and WbWgpuMeshCache's
// byte-stream `acquire()` API (which wants one interleaved
// pos3+norm3+uv2=32-byte buffer + one uint32 index buffer).
//
// The adapter is the only place that knows the WREN-side layout
// AND the wgpu vertex layout simultaneously. R3.4-step-4's PBR
// port would change the wgpu side by adding tangents / vertex
// colors / etc; the adapter would grow a wider stride accordingly,
// but the upstream WREN-mesh API stays unchanged.
//
// MeshId convention: reinterpret the WrStaticMesh pointer as
// uint64_t. The pointer is stable per-process (StaticMesh entries
// are cache::createOrRetrieveFromCache-managed), so the cache hits
// on the second + Nth resolve of the same mesh.
//
// R3.4-step-5: primitive codegen path (`acquirePrimitive`) bypasses
// wr_static_mesh_read_data for the WREN-cached unit primitives (Box,
// Sphere, Cylinder, Plane). In OmniSim's headless --no-rendering
// mode the WREN-readback falls back to glGetBufferSubData (see
// wren::StaticMesh::prepareGl clearing mCoords after GL upload),
// which returns garbage without an active GL context. The CPU-side
// codegen avoids the readback for the common case and the caller
// composes the geometry's local scale (size for Box/Plane, radius
// for Sphere, (radius, radius, height) for Cylinder) into the
// model matrix.
//

#include <cstdint>

struct WrStaticMesh;
class WbWgpuMeshCache;
struct WbWgpuMeshHandle;

namespace WbWgpuMeshAdapter {

  // The unit primitives the codegen path knows about. Pass to
  // `acquirePrimitive` to select the matching CPU-side mesh
  // generator. Each kind lands a vertex stream in WREN-local space
  // (Box / Plane in [-0.5, 0.5], Sphere at radius=1, Cylinder at
  // radius=1 / z in [-0.5, 0.5]); the caller scales via the model
  // matrix.
  enum class PrimitiveKind {
    Box,        // 24 verts, 36 idx — matches wren::createUnitBox
    Plane,      // 4 verts, 6 idx — matches wren::createUnitRectangle
    UVSphere,   // latitude/longitude UV sphere at subdivision=24
    Cylinder,   // closed cylinder at subdivision=24, side + 2 caps
  };

  // Translate a WrStaticMesh into the cache's byte-stream form and
  // upload (or return the cached handle). Returns the cache's
  // {nullptr, nullptr, 0} sentinel on any failure (invalid mesh,
  // cache backend unavailable, zero vertices/indices, or the
  // WREN-readback path that hits the --no-rendering glGetBufferSubData
  // fallback). Safe to call from any thread the cache itself is
  // safe on.
  WbWgpuMeshHandle acquireFromWren(WbWgpuMeshCache &cache, WrStaticMesh *wrenMesh);

  // CPU-side codegen path. The mesh is built directly from a unit
  // primitive template — `meshId` is just the cache key (the
  // caller will typically reuse the WrStaticMesh* bit pattern so
  // hits on the primitive path collide with the WREN path's keys
  // when the same Solid is re-rendered through both backends).
  WbWgpuMeshHandle acquirePrimitive(WbWgpuMeshCache &cache, uint64_t meshId, PrimitiveKind kind);

  // Capsule codegen. Unlike the other primitives, the Capsule's
  // radius + height are baked into the vertices (matching WREN's
  // `wr_static_mesh_capsule_new(subdivision, radius, height, ...)`
  // which doesn't use `wr_transform_set_scale` — the hemispherical
  // caps make a single (sx, sy, sz) scale impossible to express).
  // The caller's model matrix should NOT compose a local scale on
  // top of this — the geometry's pose alone is what places the
  // capsule in world space.
  //
  // `subdivision` is the segment count around the axis (≥4); the
  // hemisphere caps use `subdivision/4` latitude rings each, so
  // pass a value divisible by 4 for the cleanest topology. The
  // wrench cap-ring resolution scales with subdivision so a
  // user-set value of e.g. 24 lands a 6-ring hemisphere.
  WbWgpuMeshHandle acquireCapsule(WbWgpuMeshCache &cache, uint64_t meshId, float radius, float height, int subdivision);

}  // namespace WbWgpuMeshAdapter

#endif
