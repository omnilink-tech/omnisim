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

#include "WbWgpuMeshAdapter.hpp"

#include "WbWgpuMeshCache.hpp"

#include <wren/static_mesh.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {
  constexpr size_t kStride = 32;  // pos3 (12) + norm3 (12) + uv2 (8)

  inline void packVertex(std::vector<uint8_t> &out, float px, float py, float pz, float nx, float ny, float nz, float u,
                         float v) {
    const size_t i0 = out.size();
    out.resize(i0 + kStride);
    uint8_t *dst = out.data() + i0;
    const float pos[3] = {px, py, pz};
    const float nrm[3] = {nx, ny, nz};
    const float uv[2] = {u, v};
    std::memcpy(dst + 0, pos, 12);
    std::memcpy(dst + 12, nrm, 12);
    std::memcpy(dst + 24, uv, 8);
  }

  // ------------------------------------------------------------------
  // R3.4-step-5 primitive codegen.
  //
  // Each helper emits the same vertex layout the WREN-cached unit
  // primitive would, but built entirely on the CPU so the wgpu Camera
  // path doesn't depend on `wren::StaticMesh::readData` (which falls
  // back to glGetBufferSubData in OmniSim's headless `--no-rendering`
  // mode and returns garbage).
  //
  // Vertices land in WREN-local space: Box / Plane in [-0.5, 0.5],
  // Sphere at radius=1, Cylinder at radius=1 / z in [-0.5, 0.5]. The
  // model matrix in WbCamera applies the geometry's local scale
  // (size for Box/Plane, radius for Sphere, (radius, radius, height)
  // for Cylinder).
  // ------------------------------------------------------------------

  void buildUnitBox(std::vector<uint8_t> &packed, std::vector<uint32_t> &indices) {
    // Matches wren::StaticMesh::createUnitBox(outline=false): 24 verts
    // (6 faces * 4 corners), 36 indices (6 faces * 2 triangles).
    // Order: left (+Y), back (-X), bottom (-Z), right (-Y), front
    // (+X), top (+Z). Webots conventions: +X forward, +Y left, +Z up.
    packed.clear();
    packed.reserve(24 * kStride);
    indices.clear();
    indices.reserve(36);

    struct Face {
      float n[3];
      float v[4][3];
      float uv[4][2];
    };
    const Face faces[6] = {
      // left (+Y)
      {{0, 1, 0},
       {{-0.5f, 0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}, {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f}},
       {{1, 0}, {0, 0}, {0, 1}, {1, 1}}},
      // back (-X)
      {{-1, 0, 0},
       {{-0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, 0.5f}},
       {{0, 0}, {0, 1}, {1, 1}, {1, 0}}},
      // bottom (-Z)
      {{0, 0, -1},
       {{0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}},
       {{1, 0}, {0, 0}, {0, 1}, {1, 1}}},
      // right (-Y)
      {{0, -1, 0},
       {{0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}},
       {{1, 0}, {0, 0}, {0, 1}, {1, 1}}},
      // front (+X)
      {{1, 0, 0},
       {{0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, -0.5f}},
       {{1, 1}, {1, 0}, {0, 0}, {0, 1}}},
      // top (+Z)
      {{0, 0, 1},
       {{0.5f, -0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f}},
       {{1, 1}, {1, 0}, {0, 0}, {0, 1}}},
    };

    for (int f = 0; f < 6; ++f) {
      const uint32_t base = static_cast<uint32_t>(f) * 4u;
      for (int c = 0; c < 4; ++c)
        packVertex(packed, faces[f].v[c][0], faces[f].v[c][1], faces[f].v[c][2], faces[f].n[0], faces[f].n[1], faces[f].n[2],
                   faces[f].uv[c][0], faces[f].uv[c][1]);
      indices.push_back(base + 0);
      indices.push_back(base + 1);
      indices.push_back(base + 2);
      indices.push_back(base + 0);
      indices.push_back(base + 2);
      indices.push_back(base + 3);
    }
  }

  void buildUnitRectangle(std::vector<uint8_t> &packed, std::vector<uint32_t> &indices) {
    // Matches wren::StaticMesh::createUnitRectangle(outline=false): 4
    // verts at z=0 (normal +Z), two triangles.
    packed.clear();
    packed.reserve(4 * kStride);
    indices.clear();
    indices.reserve(6);
    packVertex(packed, -0.5f, -0.5f, 0.0f, 0, 0, 1, 0, 1);
    packVertex(packed, 0.5f, -0.5f, 0.0f, 0, 0, 1, 1, 1);
    packVertex(packed, 0.5f, 0.5f, 0.0f, 0, 0, 1, 1, 0);
    packVertex(packed, -0.5f, 0.5f, 0.0f, 0, 0, 1, 0, 0);
    indices.insert(indices.end(), {0, 1, 2, 0, 2, 3});
  }

  void buildUnitUVSphere(int subdivision, std::vector<uint8_t> &packed, std::vector<uint32_t> &indices) {
    // Latitude/longitude UV sphere at radius=1. `subdivision` (≥3) is
    // both the longitude (full circle) and latitude (pole-to-pole)
    // segment count.
    if (subdivision < 3)
      subdivision = 3;
    const int rings = subdivision;
    const int segments = subdivision;
    const float pi = 3.14159265358979323846f;
    packed.clear();
    indices.clear();
    packed.reserve(static_cast<size_t>((rings + 1) * (segments + 1)) * kStride);
    indices.reserve(static_cast<size_t>(rings * segments) * 6u);
    for (int r = 0; r <= rings; ++r) {
      const float v = static_cast<float>(r) / static_cast<float>(rings);
      const float theta = pi * v;
      const float sinT = std::sin(theta);
      const float cosT = std::cos(theta);
      for (int s = 0; s <= segments; ++s) {
        const float u = static_cast<float>(s) / static_cast<float>(segments);
        const float phi = 2.0f * pi * u;
        const float sinP = std::sin(phi);
        const float cosP = std::cos(phi);
        // Webots Sphere: +Z is up; place the poles on Z.
        const float x = sinT * cosP;
        const float y = sinT * sinP;
        const float z = cosT;
        packVertex(packed, x, y, z, x, y, z, u, 1.0f - v);
      }
    }
    const int stride = segments + 1;
    for (int r = 0; r < rings; ++r) {
      for (int s = 0; s < segments; ++s) {
        const uint32_t a = static_cast<uint32_t>(r * stride + s);
        const uint32_t b = static_cast<uint32_t>((r + 1) * stride + s);
        const uint32_t c = static_cast<uint32_t>((r + 1) * stride + s + 1);
        const uint32_t d = static_cast<uint32_t>(r * stride + s + 1);
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
        indices.push_back(a);
        indices.push_back(c);
        indices.push_back(d);
      }
    }
  }

  void buildCapsule(int subdivision, float radius, float height, std::vector<uint8_t> &packed, std::vector<uint32_t> &indices) {
    // Mirrors `wren::StaticMesh::createCapsule(... outline=false ...)`
    // for the always-on (hasSide=hasTop=hasBottom=true) case. Unlike
    // the unit primitives, radius + height are baked into the
    // vertices — the caller's model matrix must NOT also scale.
    if (subdivision < 4)
      subdivision = 4;
    const int sub1 = subdivision + 1;
    const int sub4 = subdivision / 4 > 0 ? subdivision / 4 : 1;
    const int sub5 = sub4 + 1;
    const float pi = 3.14159265358979323846f;
    const float halfH = 0.5f * height;
    packed.clear();
    indices.clear();
    const size_t vertEst = static_cast<size_t>(2 * sub1) + 2u * static_cast<size_t>(sub1) * static_cast<size_t>(sub5);
    packed.reserve(vertEst * kStride);
    indices.reserve(static_cast<size_t>(subdivision) * 12u + 2u * static_cast<size_t>(subdivision) * static_cast<size_t>(sub4) * 6u);

    // -------------------- side wall --------------------
    // Two parallel rings of `sub1` verts at z=±halfH. Outward-pointing
    // normals (x, y, 0) so the side doesn't share normals with the caps.
    // Vertices laid out as pairs: (i*2) is bottom, (i*2+1) is top.
    const uint32_t sideBase = 0;
    for (int i = 0; i < sub1; ++i) {
      const float alpha = 2.0f * pi * static_cast<float>(i) / static_cast<float>(subdivision);
      const float xs = std::sin(alpha);
      const float ys = std::cos(alpha);
      const float u = (static_cast<float>(subdivision - i)) / static_cast<float>(subdivision);
      packVertex(packed, radius * xs, radius * ys, -halfH, xs, ys, 0.0f, u, 2.0f / 3.0f);
      packVertex(packed, radius * xs, radius * ys, halfH, xs, ys, 0.0f, u, 1.0f / 3.0f);
    }
    for (int i = 0; i < subdivision; ++i) {
      const uint32_t start = sideBase + static_cast<uint32_t>(i * 2);
      indices.push_back(start);
      indices.push_back(start + 1);
      indices.push_back(start + 3);
      indices.push_back(start);
      indices.push_back(start + 3);
      indices.push_back(start + 2);
    }

    // -------------------- caps --------------------
    // Quarter-circle profile shared by the top + bottom hemispheres.
    // ay[j] is the z height (top half); ar[j] is the radius at that z.
    // For the bottom we flip z and negate the normal's z component;
    // winding gets reversed so the outside still faces out.
    std::vector<float> ay(sub5), ar(sub5);
    const float factor4 = 0.5f * pi / static_cast<float>(sub4);
    for (int j = 0; j < sub5; ++j) {
      const float a = factor4 * static_cast<float>(j);
      ay[j] = halfH + radius * std::sin(a);
      ar[j] = -radius * std::cos(a);
    }
    const float invSub = 1.0f / static_cast<float>(subdivision);

    auto buildCap = [&](bool top) {
      const uint32_t base = static_cast<uint32_t>(packed.size() / kStride);
      const float zSign = top ? 1.0f : -1.0f;
      for (int i = 0; i < sub1; ++i) {
        const float beta = 2.0f * pi * static_cast<float>(i) / static_cast<float>(subdivision);
        const float sb = std::sin(beta);
        const float cb = std::cos(beta);
        const float d1 = invSub * static_cast<float>(i);
        for (int j = 0; j < sub5; ++j) {
          const float cx = ar[j] * sb;
          const float cy = -ar[j] * cb;
          const float cz = zSign * ay[j];
          // Normal points from the hemisphere center (0, 0, ±halfH) to the
          // vertex, normalised — for a hemisphere of radius=radius, this
          // is just (cx, cy, cz - zSign*halfH) / radius.
          const float dx = cx;
          const float dy = cy;
          const float dz = cz - zSign * halfH;
          const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
          const float invLen = len > 1e-6f ? (1.0f / len) : 0.0f;
          const float v = top ? ((sub4 - j) / static_cast<float>(sub4 * 3)) : (1.0f - (sub4 - j) / static_cast<float>(sub4 * 3));
          packVertex(packed, cx, cy, cz, dx * invLen, dy * invLen, dz * invLen, d1, v);
        }
      }
      auto vidx = [&](int i, int j) { return base + static_cast<uint32_t>(i * sub5 + j); };
      for (int i = 0; i < subdivision; ++i) {
        for (int j = 0; j < sub4; ++j) {
          if (j < sub4 - 1) {
            if (top) {
              indices.push_back(vidx(i, j));
              indices.push_back(vidx(i + 1, j));
              indices.push_back(vidx(i + 1, j + 1));
              indices.push_back(vidx(i, j));
              indices.push_back(vidx(i + 1, j + 1));
              indices.push_back(vidx(i, j + 1));
            } else {
              // Bottom: reverse winding so the outside faces -Z.
              indices.push_back(vidx(i, j + 1));
              indices.push_back(vidx(i + 1, j + 1));
              indices.push_back(vidx(i + 1, j));
              indices.push_back(vidx(i, j + 1));
              indices.push_back(vidx(i + 1, j));
              indices.push_back(vidx(i, j));
            }
          } else {
            // Pole row: one tri per segment.
            if (top) {
              indices.push_back(vidx(i, j));
              indices.push_back(vidx(i + 1, j));
              indices.push_back(vidx(i, j + 1));
            } else {
              indices.push_back(vidx(i, j + 1));
              indices.push_back(vidx(i + 1, j));
              indices.push_back(vidx(i, j));
            }
          }
        }
      }
    };
    buildCap(true);
    buildCap(false);
  }

  void buildUnitCylinder(int subdivision, std::vector<uint8_t> &packed, std::vector<uint32_t> &indices) {
    // Closed cylinder at radius=1, total height=1 (z in [-0.5, 0.5]).
    // Matches the WREN side's `wr_transform_set_scale(radius, radius,
    // height)` convention. Always emits side + top + bottom — the
    // user's hasSide/hasTop/hasBottom flags would let us elide caps
    // for perf later, but the cost is negligible at typical
    // subdivisions and primitive-aware culling belongs in R3.4-step-6.
    if (subdivision < 3)
      subdivision = 3;
    packed.clear();
    indices.clear();
    const float pi = 3.14159265358979323846f;
    const float h = 0.5f;
    const int seg = subdivision;
    packed.reserve(static_cast<size_t>((seg + 1) * 2 + 2 * (seg + 2)) * kStride);
    indices.reserve(static_cast<size_t>(seg * 12));

    // Side wall: two parallel rings of (seg+1) verts with side-pointing
    // normals (so the cap and the side don't share normals).
    const uint32_t sideBase = 0;
    for (int i = 0; i <= seg; ++i) {
      const float alpha = 2.0f * pi * static_cast<float>(i) / static_cast<float>(seg);
      const float x = std::sin(alpha);
      const float y = std::cos(alpha);
      const float uu = static_cast<float>(i) / static_cast<float>(seg);
      packVertex(packed, x, y, -h, x, y, 0.0f, uu, 1.0f);
      packVertex(packed, x, y, h, x, y, 0.0f, uu, 0.0f);
    }
    for (int i = 0; i < seg; ++i) {
      const uint32_t a = sideBase + static_cast<uint32_t>(i * 2);
      const uint32_t b = sideBase + static_cast<uint32_t>(i * 2 + 1);
      const uint32_t c = sideBase + static_cast<uint32_t>(i * 2 + 3);
      const uint32_t d = sideBase + static_cast<uint32_t>(i * 2 + 2);
      indices.push_back(a);
      indices.push_back(b);
      indices.push_back(c);
      indices.push_back(a);
      indices.push_back(c);
      indices.push_back(d);
    }

    // Top cap (z = +h, normal +Z), fan: center + ring of seg+1 verts.
    const uint32_t topBase = static_cast<uint32_t>(packed.size() / kStride);
    packVertex(packed, 0.0f, 0.0f, h, 0, 0, 1, 0.5f, 0.5f);
    for (int i = 0; i <= seg; ++i) {
      const float alpha = 2.0f * pi * static_cast<float>(i) / static_cast<float>(seg);
      const float x = std::sin(alpha);
      const float y = std::cos(alpha);
      packVertex(packed, x, y, h, 0, 0, 1, 0.5f + 0.5f * x, 0.5f - 0.5f * y);
    }
    for (int i = 0; i < seg; ++i) {
      indices.push_back(topBase);
      indices.push_back(topBase + 1u + static_cast<uint32_t>(i));
      indices.push_back(topBase + 1u + static_cast<uint32_t>(i + 1));
    }

    // Bottom cap (z = -h, normal -Z), reverse winding so the outside
    // faces -Z.
    const uint32_t botBase = static_cast<uint32_t>(packed.size() / kStride);
    packVertex(packed, 0.0f, 0.0f, -h, 0, 0, -1, 0.5f, 0.5f);
    for (int i = 0; i <= seg; ++i) {
      const float alpha = 2.0f * pi * static_cast<float>(i) / static_cast<float>(seg);
      const float x = std::sin(alpha);
      const float y = std::cos(alpha);
      packVertex(packed, x, y, -h, 0, 0, -1, 0.5f + 0.5f * x, 0.5f + 0.5f * y);
    }
    for (int i = 0; i < seg; ++i) {
      indices.push_back(botBase);
      indices.push_back(botBase + 1u + static_cast<uint32_t>(i + 1));
      indices.push_back(botBase + 1u + static_cast<uint32_t>(i));
    }
  }
}  // namespace

namespace WbWgpuMeshAdapter {

  WbWgpuMeshHandle acquireFromWren(WbWgpuMeshCache &cache, WrStaticMesh *wrenMesh) {
    if (!wrenMesh)
      return {nullptr, nullptr, 0};

    const int vertexCount = wr_static_mesh_get_vertex_count(wrenMesh);
    const int indexCount = wr_static_mesh_get_index_count(wrenMesh);
    if (vertexCount <= 0 || indexCount <= 0)
      return {nullptr, nullptr, 0};

    // Cache hit fast-path: if the mesh is already in the cache for
    // this WrStaticMesh*, acquire() returns the existing handles
    // without re-reading the WREN side. We still allocate the
    // temporary arrays here because acquire() needs the
    // bytes-pointer args to be non-null — but they're memcpy
    // sources only, not retained.

    std::vector<float> coords(static_cast<size_t>(vertexCount) * 3u);
    std::vector<float> normals(static_cast<size_t>(vertexCount) * 3u);
    std::vector<float> uvs(static_cast<size_t>(vertexCount) * 2u);
    std::vector<unsigned int> indices(static_cast<size_t>(indexCount));
    wr_static_mesh_read_data(wrenMesh, coords.data(), normals.data(), uvs.data(), indices.data());

    // Validate the readback BEFORE caching. wren::StaticMesh::readData falls back to glGetBufferSubData,
    // which returns GARBAGE when the mesh's GL buffer hasn't been uploaded yet (the first render frame
    // right after a world loads) or there is no current GL context. The cache keys on the WrStaticMesh*
    // and NEVER re-reads, so caching ONE garbage read renders the mesh as noise FOREVER — exactly the
    // failure on dense URDF convex hulls (imported arms et al; primitives dodge it via CPU codegen). Detect
    // non-finite / astronomical coords and SKIP (return empty WITHOUT caching) so the NEXT frame retries
    // once WREN has actually uploaded the buffer — turning "garbage noise forever" into "missing for a
    // frame or two, then correct" (the buffer stays uploaded after the first valid read, so no flicker).
    bool readbackValid = true;
    for (size_t i = 0; i < coords.size(); ++i) {
      if (!std::isfinite(coords[i]) || std::fabs(coords[i]) > 1.0e6f) {
        readbackValid = false;
        break;
      }
    }
    if (!readbackValid)
      return {nullptr, nullptr, 0};

    // Interleave into the 32-byte stride layout the wgpu pipelines
    // expect: pos3 @ offset 0, norm3 @ offset 12, uv2 @ offset 24.
    std::vector<uint8_t> packed(static_cast<size_t>(vertexCount) * kStride);
    for (int v = 0; v < vertexCount; ++v) {
      uint8_t *dst = packed.data() + static_cast<size_t>(v) * kStride;
      std::memcpy(dst + 0, &coords[3 * v], 12);
      std::memcpy(dst + 12, &normals[3 * v], 12);
      std::memcpy(dst + 24, &uvs[2 * v], 8);
    }

    const uint64_t meshId = reinterpret_cast<uint64_t>(wrenMesh);
    return cache.acquire(meshId, packed.data(), packed.size(), indices.data(),
                         indices.size() * sizeof(unsigned int),
                         static_cast<uint32_t>(indexCount), static_cast<uint32_t>(kStride));
  }

  WbWgpuMeshHandle acquireCapsule(WbWgpuMeshCache &cache, uint64_t meshId, float radius, float height, int subdivision) {
    std::vector<uint8_t> packed;
    std::vector<uint32_t> indices;
    buildCapsule(subdivision, radius, height, packed, indices);
    if (packed.empty() || indices.empty())
      return {nullptr, nullptr, 0};
    return cache.acquire(meshId, packed.data(), packed.size(), indices.data(),
                         indices.size() * sizeof(uint32_t),
                         static_cast<uint32_t>(indices.size()),
                         static_cast<uint32_t>(kStride));
  }

  WbWgpuMeshHandle acquirePrimitive(WbWgpuMeshCache &cache, uint64_t meshId, PrimitiveKind kind) {
    std::vector<uint8_t> packed;
    std::vector<uint32_t> indices;
    switch (kind) {
      case PrimitiveKind::Box:
        buildUnitBox(packed, indices);
        break;
      case PrimitiveKind::Plane:
        buildUnitRectangle(packed, indices);
        break;
      case PrimitiveKind::UVSphere:
        buildUnitUVSphere(24, packed, indices);
        break;
      case PrimitiveKind::Cylinder:
        buildUnitCylinder(24, packed, indices);
        break;
    }
    if (packed.empty() || indices.empty())
      return {nullptr, nullptr, 0};
    return cache.acquire(meshId, packed.data(), packed.size(), indices.data(),
                         indices.size() * sizeof(uint32_t),
                         static_cast<uint32_t>(indices.size()),
                         static_cast<uint32_t>(kStride));
  }

}  // namespace WbWgpuMeshAdapter
