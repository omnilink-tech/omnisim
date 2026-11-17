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
#ifndef OM_TRACE_HPP
#define OM_TRACE_HPP
#include "OmniLight.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
namespace OmTrace {
struct V3 {
  float x = 0, y = 0, z = 0;
};
static inline V3 v3(const float *p) { return {p[0], p[1], p[2]}; }
static inline V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline V3 mul(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(V3 a, V3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static inline V3 norm(V3 a) {
  const float l = std::sqrt(dot(a, a));
  return l > 1e-12f ? mul(a, 1.0f / l) : V3{0, 0, 1};
}

// ---- BVH (median split on the longest axis, leaf <= 4 tris, iterative stack traversal) ----

struct BvhNode {
  float bmin[3], bmax[3];
  int32_t left = -1;    // internal: left child (right = left + 1 is NOT used; explicit right)
  int32_t right = -1;
  int32_t first = 0;    // leaf: first tri index (into order)
  int32_t count = 0;    // leaf: tri count (0 = internal)
};

struct Bvh {
  std::vector<BvhNode> nodes;
  std::vector<uint32_t> order;  // triangle indices, leaf-contiguous
  const std::vector<OmniLightTriangle> *tris = nullptr;
};

static void triBounds(const OmniLightTriangle &t, float mn[3], float mx[3]) {
  for (int k = 0; k < 3; ++k) {
    mn[k] = std::min({t.v0[k], t.v1[k], t.v2[k]});
    mx[k] = std::max({t.v0[k], t.v1[k], t.v2[k]});
  }
}

static int buildNode(Bvh &b, std::vector<float> &cent, int first, int count) {
  BvhNode node;
  node.bmin[0] = node.bmin[1] = node.bmin[2] = 3.4e38f;
  node.bmax[0] = node.bmax[1] = node.bmax[2] = -3.4e38f;
  for (int i = first; i < first + count; ++i) {
    float mn[3], mx[3];
    triBounds((*b.tris)[b.order[i]], mn, mx);
    for (int k = 0; k < 3; ++k) {
      node.bmin[k] = std::min(node.bmin[k], mn[k]);
      node.bmax[k] = std::max(node.bmax[k], mx[k]);
    }
  }
  const int idx = static_cast<int>(b.nodes.size());
  b.nodes.push_back(node);
  if (count <= 4) {
    b.nodes[idx].first = first;
    b.nodes[idx].count = count;
    return idx;
  }
  int axis = 0;
  float ext[3] = {node.bmax[0] - node.bmin[0], node.bmax[1] - node.bmin[1],
                  node.bmax[2] - node.bmin[2]};
  if (ext[1] > ext[axis])
    axis = 1;
  if (ext[2] > ext[axis])
    axis = 2;
  const int mid = first + count / 2;
  std::nth_element(b.order.begin() + first, b.order.begin() + mid, b.order.begin() + first + count,
                   [&](uint32_t a, uint32_t c) { return cent[a * 3 + axis] < cent[c * 3 + axis]; });
  const int l = buildNode(b, cent, first, mid - first);
  const int r = buildNode(b, cent, mid, first + count - mid);
  b.nodes[idx].left = l;
  b.nodes[idx].right = r;
  b.nodes[idx].count = 0;
  return idx;
}

static void buildBvh(Bvh &b, const std::vector<OmniLightTriangle> &tris) {
  b.tris = &tris;
  const size_t n = tris.size();
  b.order.resize(n);
  std::vector<float> cent(n * 3);
  for (size_t i = 0; i < n; ++i) {
    b.order[i] = static_cast<uint32_t>(i);
    for (int k = 0; k < 3; ++k)
      cent[i * 3 + k] = (tris[i].v0[k] + tris[i].v1[k] + tris[i].v2[k]) / 3.0f;
  }
  b.nodes.reserve(n / 2 + 8);
  if (n)
    buildNode(b, cent, 0, static_cast<int>(n));
}

struct Hit {
  float t = 3.4e38f;
  uint32_t tri = 0;
  float u = 0, v = 0;
  bool backface = false;
  bool ok = false;
};

static inline bool aabbHit(const BvhNode &nd, V3 o, V3 invD, float tMax) {
  float t0 = 0.0f, t1 = tMax;
  const float *ov = &o.x;
  const float *iv = &invD.x;
  for (int k = 0; k < 3; ++k) {
    const float ta = (nd.bmin[k] - ov[k]) * iv[k];
    const float tb = (nd.bmax[k] - ov[k]) * iv[k];
    t0 = std::max(t0, std::min(ta, tb));
    t1 = std::min(t1, std::max(ta, tb));
  }
  return t0 <= t1;
}

static Hit trace(const Bvh &b, V3 o, V3 d, float tMax, bool anyHit) {
  Hit h;
  if (b.nodes.empty())
    return h;
  const V3 invD = {1.0f / (std::abs(d.x) > 1e-12f ? d.x : copysignf(1e-12f, d.x)),
                   1.0f / (std::abs(d.y) > 1e-12f ? d.y : copysignf(1e-12f, d.y)),
                   1.0f / (std::abs(d.z) > 1e-12f ? d.z : copysignf(1e-12f, d.z))};
  int stack[64];
  int sp = 0;
  stack[sp++] = 0;
  while (sp) {
    const BvhNode &nd = b.nodes[stack[--sp]];
    if (!aabbHit(nd, o, invD, std::min(tMax, h.t)))
      continue;
    if (nd.count) {
      for (int i = nd.first; i < nd.first + nd.count; ++i) {
        const OmniLightTriangle &tr = (*b.tris)[b.order[i]];
        // Moller-Trumbore
        const V3 e1 = sub(v3(tr.v1), v3(tr.v0));
        const V3 e2 = sub(v3(tr.v2), v3(tr.v0));
        const V3 pv = cross(d, e2);
        const float det = dot(e1, pv);
        if (std::abs(det) < 1e-9f)
          continue;
        const float inv = 1.0f / det;
        const V3 tv = sub(o, v3(tr.v0));
        const float u = dot(tv, pv) * inv;
        if (u < 0.0f || u > 1.0f)
          continue;
        const V3 qv = cross(tv, e1);
        const float vv = dot(d, qv) * inv;
        if (vv < 0.0f || u + vv > 1.0f)
          continue;
        const float t = dot(e2, qv) * inv;
        if (t > 1e-4f && t < std::min(tMax, h.t)) {
          h.t = t;
          h.u = u;
          h.v = vv;
          h.tri = b.order[i];
          h.backface = det < 0.0f;
          h.ok = true;
          if (anyHit)
            return h;
        }
      }
    } else if (sp < 62) {
      stack[sp++] = nd.left;
      stack[sp++] = nd.right;
    }
  }
  return h;
}

// ---- deterministic per-probe RNG (PCG32-flavoured) ----
struct Rng {
  uint64_t state;
  explicit Rng(uint64_t seed) : state(seed * 6364136223846793005ULL + 1442695040888963407ULL) {}
  float next() {  // [0,1)
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    const uint32_t xorshifted = static_cast<uint32_t>(((state >> 18u) ^ state) >> 27u);
    const uint32_t rot = static_cast<uint32_t>(state >> 59u);
    const uint32_t r = (xorshifted >> rot) | (xorshifted << ((0u - rot) & 31u));
    return (r >> 8) * (1.0f / 16777216.0f);
  }
};

static inline V3 sphereDir(int i, int n, float j1, float j2) {
  // Fibonacci sphere with per-probe jitter — stratified, deterministic.
  const float golden = 2.39996323f;
  const float z = 1.0f - 2.0f * ((i + j1) / n);
  const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
  const float phi = i * golden + j2 * 6.2831853f;
  return {r * std::cos(phi), r * std::sin(phi), z};
}

static V3 cosineDir(V3 n, float u1, float u2) {
  const float r = std::sqrt(u1);
  const float phi = 6.2831853f * u2;
  V3 t = std::abs(n.z) < 0.9f ? V3{0, 0, 1} : V3{1, 0, 0};
  const V3 b1 = norm(cross(t, n));
  const V3 b2 = cross(n, b1);
  const V3 d = add(add(mul(b1, r * std::cos(phi)), mul(b2, r * std::sin(phi))),
                   mul(n, std::sqrt(std::max(0.0f, 1.0f - u1))));
  return norm(d);
}


}  // namespace OmTrace
#endif
