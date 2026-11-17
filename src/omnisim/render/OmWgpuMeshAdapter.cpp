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

#include "OmWgpuMeshAdapter.hpp"

#include "OmWgpuMeshCache.hpp"


#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {
  constexpr size_t kStride = 32;  // pos3 (12) + norm3 (12) + uv2 (8)

  // ------------------------------------------------------------------
  // W1c content keys.
  //
  // Bit 62 tags the whole primitive key space; bit 63 stays CLEAR. The
  // cache's other callers pass canonical-47-bit heap pointers (WrStaticMesh*,
  // OmTriangleMesh*) and OmCadShape's bit-63-tagged FNV content hashes, so
  // neither can ever alias a key produced here.
  //
  // Inside the space, bit 61 separates the two families: CLEAR = one of the
  // four parameterless unit meshes (a small ordinal), SET = a hashed-content
  // mesh. Two disjoint regions, so a content hash can never collide with a
  // unit-mesh ordinal either. Within the hashed family, bits 59-60 name the
  // MEMBER (0 = capsule, 1 = cone, 2 = elevation grid) and the hash itself is
  // truncated to bits 0-58 — so two different node types with coincidentally
  // equal parameter hashes still get distinct keys, structurally.
  // ------------------------------------------------------------------
  constexpr uint64_t kPrimitiveKeyTag = static_cast<uint64_t>(1) << 62;
  constexpr uint64_t kHashedKeyBit = static_cast<uint64_t>(1) << 61;
  constexpr int kHashedMemberShift = 59;
  constexpr uint64_t kHashedHashMask = (static_cast<uint64_t>(1) << kHashedMemberShift) - 1u;
  enum HashedMember : uint64_t { kMemberCapsule = 0, kMemberCone = 1, kMemberElevationGrid = 2 };

  uint64_t hashedContentKey(uint64_t member, uint64_t hash) {
    return kPrimitiveKeyTag | kHashedKeyBit | (member << kHashedMemberShift) | (hash & kHashedHashMask);
  }
  constexpr uint64_t kFnv1a64Offset = 1469598103934665603ULL;
  constexpr uint64_t kFnv1a64Prime = 1099511628211ULL;

  // 64-bit FNV-1a — same construction OmCadShape uses for its submesh keys.
  uint64_t fnv1a64(const void *data, size_t length, uint64_t hash) {
    const unsigned char *p = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < length; ++i) {
      hash ^= static_cast<uint64_t>(p[i]);
      hash *= kFnv1a64Prime;
    }
    return hash;
  }

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
  // model matrix in OmCamera applies the geometry's local scale
  // (size for Box/Plane, radius for Sphere, (radius, radius, height)
  // for Cylinder).
  // ------------------------------------------------------------------

  void buildUnitBox(std::vector<uint8_t> &packed, std::vector<uint32_t> &indices) {
    // Matches wren::StaticMesh::createUnitBox(outline=false): 24 verts
    // (6 faces * 4 corners), 36 indices (6 faces * 2 triangles).
    // Order: left (+Y), back (-X), bottom (-Z), right (-Y), front
    // (+X), top (+Z). OmniSim conventions: +X forward, +Y left, +Z up.
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
        // OmniSim Sphere: +Z is up; place the poles on Z.
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

  void buildUnitCone(int subdivision, bool hasSide, bool hasBottom, std::vector<uint8_t> &packed,
                     std::vector<uint32_t> &indices) {
    // Mirrors `wren::StaticMesh::createUnitCone(subdivision, hasSide,
    // hasBottom)` verbatim (src/wren/StaticMesh.cpp:252 pre-D1.4): base
    // circle radius=1 at z=-0.5, apex at z=+0.5, one apex VERTEX per
    // segment carrying the segment's mid-angle smooth normal, optional
    // -Z base disc. The caller's model matrix applies
    // (bottomRadius, bottomRadius, height) — OmCone::updateScale()'s
    // wr_transform_set_scale convention — so nothing here is scaled.
    if (subdivision < 3)
      subdivision = 3;
    packed.clear();
    indices.clear();
    const int sub1 = subdivision + 1;
    const float pi = 3.14159265358979323846f;
    const float h = 0.5f;
    const float invSub = 1.0f / static_cast<float>(subdivision);
    const float k = invSub * 2.0f * pi;
    packed.reserve(static_cast<size_t>((hasSide ? 2 * sub1 : 0) + (hasBottom ? sub1 + 1 : 0)) * kStride);
    indices.reserve(static_cast<size_t>((hasSide ? subdivision * 3 : 0) + (hasBottom ? subdivision * 3 : 0)));

    if (hasSide) {
      // Side normal: for a unit cone (slope 1 over height 1), the outward
      // normal at angle a is (sin a, cos a, hh) / rr with hh=0.5, rr=|(1, hh)|.
      float hh = 0.5f;
      const float rr = std::sqrt(1.0f + hh * hh);
      hh /= rr;
      const float invRr = 1.0f / rr;
      for (int i = 0; i < sub1; ++i) {
        const float alpha = k * static_cast<float>(i);
        const float beta = k * (0.5f + static_cast<float>(i));
        const float x = std::sin(alpha);
        const float y = std::cos(alpha);
        const float xM = std::sin(beta);
        const float yM = std::cos(beta);
        const float d1 = static_cast<float>(subdivision - i - 1) * invSub;
        const float d2 = static_cast<float>(subdivision - i) * invSub;
        const float d = d1 + d2;
        // Pairs: (i*2) base-rim vertex, (i*2+1) apex vertex for this segment.
        packVertex(packed, x, y, -h, x * invRr, y * invRr, hh, d1, 1.0f);
        packVertex(packed, 0.0f, 0.0f, h, xM * invRr, yM * invRr, hh, d * 0.5f, 0.0f);
      }
      for (int i = 0, start = 0; i < subdivision; ++i, start += 2) {
        indices.push_back(static_cast<uint32_t>(start));
        indices.push_back(static_cast<uint32_t>(start + 1));
        indices.push_back(static_cast<uint32_t>(start + 2));
      }
    }

    if (hasBottom) {
      const uint32_t center = static_cast<uint32_t>(packed.size() / kStride);
      packVertex(packed, 0.0f, 0.0f, -h, 0.0f, 0.0f, -1.0f, 0.5f, 0.5f);
      for (int i = 0; i < sub1; ++i) {
        const float alpha = k * static_cast<float>(i);
        const float x = std::sin(alpha);
        const float y = -std::cos(alpha);
        packVertex(packed, x, -y, -h, 0.0f, 0.0f, -1.0f, 0.5f * x + 0.5f, -0.5f * y + 0.5f);
      }
      for (uint32_t i = 0, start = center + 1u; i < static_cast<uint32_t>(subdivision); ++i, ++start) {
        indices.push_back(start);
        indices.push_back(start + 1u);
        indices.push_back(center);
      }
    }
  }

  void buildElevationGrid(int dimX, int dimY, const float *heightData, std::vector<uint8_t> &packed,
                          std::vector<uint32_t> &indices) {
    // Mirrors the RENDER (non-outline) arm of
    // `wren::StaticMesh::createUnitElevationGrid` verbatim
    // (src/wren/StaticMesh.cpp:535 pre-D1.4). Axis convention, matched to
    // that source rather than guessed: the grid lies in the X-Y plane with
    // heights along +Z (ENU/FLU — post axis-migration OmElevationGrid is
    // x/y-named: xDimension/yDimension/xSpacing/ySpacing), at UNIT spacing
    // (the caller's model matrix applies (xSpacing, ySpacing, 1)), and
    // WREN builds mesh row yi from HEIGHT row (dimY-1-yi) — a Y flip that
    // keeps the first height row at the far-Y edge while UV v still runs
    // 0→1 across mesh rows. Normals are the normalised sum of the cross
    // products over the 4 neighbouring cells, cells outside the grid
    // ignored; two triangles per cell with WREN's winding.
    packed.clear();
    indices.clear();
    if (dimX < 2 || dimY < 2)
      return;
    const int stepsX = dimX - 1;
    const int stepsY = dimY - 1;
    const float du = 1.0f / static_cast<float>(stepsX);
    const float dv = 1.0f / static_cast<float>(stepsY);
    const size_t vcount = static_cast<size_t>(dimX) * static_cast<size_t>(dimY);

    // Positions first (normals need the neighbours).
    std::vector<float> px(vcount), py(vcount), pz(vcount);
    for (int yi = 0; yi < dimY; ++yi)
      for (int xi = 0; xi < dimX; ++xi) {
        const size_t at = static_cast<size_t>(dimX) * yi + xi;
        px[at] = static_cast<float>(xi);
        py[at] = static_cast<float>(dimY - 1 - yi);
        pz[at] = heightData[dimX * (dimY - 1 - yi) + xi];
      }

    packed.reserve(vcount * kStride);
    for (int yi = 0; yi < dimY; ++yi)
      for (int xi = 0; xi < dimX; ++xi) {
        const size_t at = static_cast<size_t>(dimX) * yi + xi;
        float nx = 0.0f, ny = 0.0f, nz = 0.0f;
        auto accumulate = [&](size_t ia, size_t ib) {
          // += cross(coords[ia] - v0, coords[ib] - v0)
          const float ax = px[ia] - px[at], ay = py[ia] - py[at], az = pz[ia] - pz[at];
          const float bx = px[ib] - px[at], by = py[ib] - py[at], bz = pz[ib] - pz[at];
          nx += ay * bz - az * by;
          ny += az * bx - ax * bz;
          nz += ax * by - ay * bx;
        };
        const size_t xm = at - 1, xp = at + 1;
        const size_t ym = at - static_cast<size_t>(dimX), yp = at + static_cast<size_t>(dimX);
        if (yi > 0 && xi > 0)
          accumulate(ym, xm);
        if (yi > 0 && xi < stepsX)
          accumulate(xp, ym);
        if (yi < stepsY && xi > 0)
          accumulate(xm, yp);
        if (yi < stepsY && xi < stepsX)
          accumulate(yp, xp);
        const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-12f) {
          nx /= len;
          ny /= len;
          nz /= len;
        } else {
          nx = ny = 0.0f;
          nz = 1.0f;  // fully degenerate neighbourhood — face up
        }
        packVertex(packed, px[at], py[at], pz[at], nx, ny, nz, du * static_cast<float>(xi),
                   dv * static_cast<float>(yi));
      }

    indices.reserve(static_cast<size_t>(stepsX) * static_cast<size_t>(stepsY) * 6u);
    for (int yi = 0; yi < stepsY; ++yi)
      for (int xi = 0; xi < stepsX; ++xi) {
        indices.push_back(static_cast<uint32_t>(dimX * yi + xi));
        indices.push_back(static_cast<uint32_t>(dimX * (yi + 1) + xi));
        indices.push_back(static_cast<uint32_t>(dimX * yi + (xi + 1)));
        indices.push_back(static_cast<uint32_t>(dimX * yi + (xi + 1)));
        indices.push_back(static_cast<uint32_t>(dimX * (yi + 1) + xi));
        indices.push_back(static_cast<uint32_t>(dimX * (yi + 1) + (xi + 1)));
      }
  }
}  // namespace

namespace OmWgpuMeshAdapter {

  uint64_t primitiveContentKey(PrimitiveKind kind) {
    uint64_t ordinal = 0;
    switch (kind) {
      case PrimitiveKind::Box:
        ordinal = 1;
        break;
      case PrimitiveKind::Plane:
        ordinal = 2;
        break;
      case PrimitiveKind::UVSphere:
        ordinal = 3;
        break;
      case PrimitiveKind::Cylinder:
        ordinal = 4;
        break;
    }
    // bit 61 CLEAR — the unit-mesh half of the primitive key space.
    return kPrimitiveKeyTag | ordinal;
  }

  uint64_t capsuleContentKey(float radius, float height, int subdivision) {
    // Hash the EXACT three values buildCapsule() consumes, by bit pattern, in
    // a padding-free payload (a struct would fold uninitialised padding into
    // the hash and make the key non-deterministic). buildCapsule clamps
    // subdivision to >= 4 internally; hashing the raw value is still correct —
    // a 3 and a 4 would just get two keys for identical geometry, which costs
    // a duplicate upload and never a wrong mesh.
    uint32_t bits[3] = {0, 0, 0};
    std::memcpy(&bits[0], &radius, 4);
    std::memcpy(&bits[1], &height, 4);
    const int32_t sub = static_cast<int32_t>(subdivision);
    std::memcpy(&bits[2], &sub, 4);
    const uint64_t hash = fnv1a64(bits, sizeof(bits), kFnv1a64Offset);
    // bit 61 SET + member 0 — the capsule slot of the hashed-content space.
    return hashedContentKey(kMemberCapsule, hash);
  }

  uint64_t coneContentKey(int subdivision, bool hasSide, bool hasBottom) {
    // Same padding-free-payload discipline as capsuleContentKey: hash the
    // exact values buildUnitCone() consumes, widened to int32 so the layout
    // is deterministic.
    int32_t bits[3] = {static_cast<int32_t>(subdivision), hasSide ? 1 : 0, hasBottom ? 1 : 0};
    const uint64_t hash = fnv1a64(bits, sizeof(bits), kFnv1a64Offset);
    return hashedContentKey(kMemberCone, hash);
  }

  uint64_t elevationGridContentKey(int dimX, int dimY, const float *heights) {
    // Dimensions first, then every height sample by bit pattern — the same
    // shape as WREN's own sipHash13c(params) ^ sipHash13c(heightData) key,
    // so a supervisor height edit (or setHeightScaleFactor) lands a NEW key
    // and therefore a fresh upload on the next collect. `thickness` is
    // deliberately absent: it only ever shaped the outline mesh.
    int32_t dims[2] = {static_cast<int32_t>(dimX), static_cast<int32_t>(dimY)};
    uint64_t hash = fnv1a64(dims, sizeof(dims), kFnv1a64Offset);
    hash = fnv1a64(heights, sizeof(float) * static_cast<size_t>(dimX) * static_cast<size_t>(dimY), hash);
    return hashedContentKey(kMemberElevationGrid, hash);
  }

  OmWgpuMeshHandle acquireCapsule(OmWgpuMeshCache &cache, uint64_t meshId, float radius, float height, int subdivision) {
    // Skip the codegen on a hit — buildCapsule allocates and trigonometrically
    // generates a few thousand vertices, and the scene walk runs it per Capsule
    // node per rebuild.
    OmWgpuMeshHandle cached;
    if (cache.tryGet(meshId, cached))
      return cached;
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

  OmWgpuMeshHandle acquireCapsule(OmWgpuMeshCache &cache, float radius, float height, int subdivision) {
    return acquireCapsule(cache, capsuleContentKey(radius, height, subdivision), radius, height, subdivision);
  }

  OmWgpuMeshHandle acquireCone(OmWgpuMeshCache &cache, int subdivision, bool hasSide, bool hasBottom) {
    OmWgpuMeshHandle cached;
    const uint64_t meshId = coneContentKey(subdivision, hasSide, hasBottom);
    if (cache.tryGet(meshId, cached))
      return cached;
    std::vector<uint8_t> packed;
    std::vector<uint32_t> indices;
    buildUnitCone(subdivision, hasSide, hasBottom, packed, indices);
    if (packed.empty() || indices.empty())
      return {nullptr, nullptr, 0};
    return cache.acquire(meshId, packed.data(), packed.size(), indices.data(),
                         indices.size() * sizeof(uint32_t),
                         static_cast<uint32_t>(indices.size()),
                         static_cast<uint32_t>(kStride));
  }

  OmWgpuMeshHandle acquireElevationGrid(OmWgpuMeshCache &cache, int dimX, int dimY, const float *heights) {
    if (dimX < 2 || dimY < 2 || !heights)
      return {nullptr, nullptr, 0};
    OmWgpuMeshHandle cached;
    const uint64_t meshId = elevationGridContentKey(dimX, dimY, heights);
    if (cache.tryGet(meshId, cached))
      return cached;
    std::vector<uint8_t> packed;
    std::vector<uint32_t> indices;
    buildElevationGrid(dimX, dimY, heights, packed, indices);
    if (packed.empty() || indices.empty())
      return {nullptr, nullptr, 0};
    return cache.acquire(meshId, packed.data(), packed.size(), indices.data(),
                         indices.size() * sizeof(uint32_t),
                         static_cast<uint32_t>(indices.size()),
                         static_cast<uint32_t>(kStride));
  }

  OmWgpuMeshHandle acquirePrimitive(OmWgpuMeshCache &cache, PrimitiveKind kind) {
    return acquirePrimitive(cache, primitiveContentKey(kind), kind);
  }

  OmWgpuMeshHandle acquirePrimitive(OmWgpuMeshCache &cache, uint64_t meshId, PrimitiveKind kind) {
    OmWgpuMeshHandle cached;
    if (cache.tryGet(meshId, cached))
      return cached;  // same reason as acquireCapsule: don't re-run the builder
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

}  // namespace OmWgpuMeshAdapter
