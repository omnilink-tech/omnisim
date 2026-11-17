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

#ifndef OM_WGPU_MESH_ADAPTER_HPP
#define OM_WGPU_MESH_ADAPTER_HPP

//
// OmWgpuMeshAdapter — R3.2b of engine-migration-plan.md §14.3.
//
// Glue layer between WREN's WrStaticMesh API (which exposes
// coordinates + normals + UVs + indices as four separate float/uint
// arrays through wr_static_mesh_read_data) and OmWgpuMeshCache's
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
// WREN-retirement W1c: the codegen primitives are now keyed on their
// CONTENT (see primitiveContentKey / capsuleContentKey below) rather
// than on the WrStaticMesh pointer the caller happened to have. That
// removes the last reason the primitive path needed a WREN mesh to
// exist at all, and it de-duplicates the GPU buffers: the unit meshes
// depend on nothing but the kind, so every Box in a world now shares
// ONE vertex/index buffer pair instead of uploading an identical copy
// per node.
//

#include <cstdint>

class OmWgpuMeshCache;
struct OmWgpuMeshHandle;

namespace OmWgpuMeshAdapter {

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

  // Content keys for the codegen primitives (W1c). Box / Plane /
  // UVSphere / Cylinder are UNIT meshes that the caller scales via the
  // model matrix, and the builders below take no parameters other than
  // the kind — so their vertices are a function of the kind alone and
  // one key serves every instance in the world. The Capsule is the
  // exception: `wr_static_mesh_capsule_new` has no single-(sx,sy,sz)
  // scale equivalent (the hemispherical caps would shear), so radius +
  // height + subdivision are baked into the vertices and therefore into
  // the key. The Cone and ElevationGrid (post-D1.4) are unit meshes
  // again — Cone scaled (radius, radius, height), ElevationGrid
  // (xSpacing, ySpacing, 1), exactly WREN's wr_transform_set_scale
  // conventions — but their vertex CONTENT is parameterised
  // (subdivision + face flags; dimensions + every height sample), so
  // they key on a hash of it like the Capsule does.
  //
  // Key space: bit 62 set, bit 63 clear. The cache's other callers pass
  // heap pointers (canonical 47-bit, so bits 62-63 clear) and
  // OmCadShape's submesh content hashes (bit 63 set), so a primitive key
  // can never alias either — structurally, not probabilistically.
  uint64_t primitiveContentKey(PrimitiveKind kind);
  uint64_t capsuleContentKey(float radius, float height, int subdivision);
  uint64_t coneContentKey(int subdivision, bool hasSide, bool hasBottom);
  uint64_t elevationGridContentKey(int dimX, int dimY, const float *heights);

  // CPU-side codegen path, content-keyed (W1c). Prefer these: they need
  // no WrStaticMesh and they share one GPU buffer pair per shape.
  OmWgpuMeshHandle acquirePrimitive(OmWgpuMeshCache &cache, PrimitiveKind kind);
  OmWgpuMeshHandle acquireCapsule(OmWgpuMeshCache &cache, float radius, float height, int subdivision);

  // Unit cone: base circle radius=1 at z=-0.5, apex at z=+0.5 — WREN's
  // createUnitCone layout, smooth side normals, optional base disc. The
  // caller's model matrix applies (bottomRadius, bottomRadius, height),
  // matching OmCone::updateScale()'s wr_transform_set_scale.
  OmWgpuMeshHandle acquireCone(OmWgpuMeshCache &cache, int subdivision, bool hasSide, bool hasBottom);

  // Unit-spacing elevation grid: vertex (xi, yi) of a dimX×dimY grid at
  // x = xi, y = yi, z = height — WREN's createUnitElevationGrid render
  // (non-outline) layout, including its row flip (mesh row yi reads
  // height row dimY-1-yi) and 4-neighbour averaged normals. `heights`
  // must hold exactly dimX*dimY floats (caller pads with zeros, as the
  // node's buildWrenMesh always did). The caller's model matrix applies
  // (xSpacing, ySpacing, 1), matching OmElevationGrid::updateScale().
  // The `thickness` field only ever shaped the OUTLINE (bounding-object
  // wireframe) mesh, never the render mesh, so it is not a parameter.
  OmWgpuMeshHandle acquireElevationGrid(OmWgpuMeshCache &cache, int dimX, int dimY, const float *heights);

  // CPU-side codegen path with a CALLER-CHOSEN cache key. The mesh is
  // built directly from a unit primitive template — `meshId` is just the
  // cache key. Kept for the diagnostic/probe call sites that want their
  // own key space; ordinary scene collection uses the content-keyed
  // overloads above. Both skip the codegen entirely on a cache hit.
  OmWgpuMeshHandle acquirePrimitive(OmWgpuMeshCache &cache, uint64_t meshId, PrimitiveKind kind);

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
  OmWgpuMeshHandle acquireCapsule(OmWgpuMeshCache &cache, uint64_t meshId, float radius, float height, int subdivision);

}  // namespace OmWgpuMeshAdapter

#endif
