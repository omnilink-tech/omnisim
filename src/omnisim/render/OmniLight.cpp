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
// OmniLight bake — BVH + deterministic path tracer + SH probe volume. See OmniLight.hpp for
// the design contract. Pure std; no Qt, no GPU — runs entirely on a worker thread.

#include "OmniLight.hpp"
#include "OmTrace.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace {

using namespace OmTrace;

static uint16_t toHalf(float f) {
  // round-to-nearest float -> half, clamped to half range (no NaN/inf inputs expected)
  f = std::max(-65504.0f, std::min(65504.0f, f));
  uint32_t x;
  std::memcpy(&x, &f, 4);
  const uint32_t sign = (x >> 16) & 0x8000u;
  int32_t exp = static_cast<int32_t>((x >> 23) & 0xFF) - 127 + 15;
  uint32_t man = x & 0x7FFFFFu;
  if (exp <= 0)
    return static_cast<uint16_t>(sign);  // flush subnormals to zero
  if (exp >= 31)
    return static_cast<uint16_t>(sign | 0x7BFFu);
  return static_cast<uint16_t>(sign | (static_cast<uint32_t>(exp) << 10) | (man >> 13));
}

struct TraceCtx {
  const Bvh *bvh;
  const std::vector<OmniLightTriangle> *tris;
  const std::vector<OmniLightMaterial> *mats;
  const OmniLightParams *p;
  V3 sunTo;
};

// Direct contribution of the baked local lights at a surface point (albedo-modulated, with a
// shadow ray per light — the occlusion the real-time term never had). The /pi-less convention
// deliberately matches the real-time extraPart it replaces, so brightness carries over.
static V3 localLightsAt(const TraceCtx &c, V3 hp, V3 n, const OmniLightMaterial &m) {
  V3 L = {0, 0, 0};
  for (const OmniLightLocal &gl : c.p->locals) {
    const V3 to = sub(v3(gl.pos), hp);
    const float dist = std::sqrt(dot(to, to));
    if (dist < 1e-4f)
      continue;
    if (gl.radius > 0.0f && dist > gl.radius)
      continue;
    const V3 dir = mul(to, 1.0f / dist);
    const float ndl = dot(n, dir);
    if (ndl <= 0.0f)
      continue;
    float att = 1.0f / std::max(gl.atten[0] + gl.atten[1] * dist + gl.atten[2] * dist * dist, 1e-3f);
    if (gl.type == 2) {
      const float cosA = -dot(dir, norm(v3(gl.spotDir)));
      if (cosA < gl.cosCut)
        continue;
      att *= std::min(1.0f, std::max(0.0f, (cosA - gl.cosCut) /
                                             std::max(gl.cosBeam - gl.cosCut, 1e-4f)));
    }
    const Hit sh = trace(*c.bvh, hp, dir, dist - 1e-3f, true);
    if (sh.ok)
      continue;  // occluded — the whole point
    const float f = ndl * att * (gl.physical?1.0f/3.14159265f:c.p->localScale);
    L = add(L, {m.albedoLin[0] * gl.colorLin[0] * f, m.albedoLin[1] * gl.colorLin[1] * f,
                m.albedoLin[2] * gl.colorLin[2] * f});
  }
  return L;
}

static inline V3 skyAt(const TraceCtx &c, V3 d) {
  float out[3] = {0, 0, 0};
  const float dir[3] = {d.x, d.y, d.z};
  if (c.p->skySample)
    c.p->skySample(dir, out);
  return {out[0], out[1], out[2]};
}

// Radiance arriving at `o` from direction `d`, with `bounces` remaining surface interactions.
static V3 radiance(const TraceCtx &c, V3 o, V3 d, int bounces, Rng &rng) {
  const Hit h = trace(*c.bvh, o, d, 3.4e38f, false);
  if (!h.ok)
    return skyAt(c, d);
  const OmniLightTriangle &tr = (*c.tris)[h.tri];
  const OmniLightMaterial &m = (*c.mats)[tr.material];
  V3 n = norm(cross(sub(v3(tr.v1), v3(tr.v0)), sub(v3(tr.v2), v3(tr.v0))));
  const bool emissionVisible=m.emissiveTwoSided || dot(n,d)<0;
  if (dot(n, d) > 0.0f)
    n = mul(n, -1.0f);  // face the ray
  const V3 hp = add(o, mul(d, h.t));
  const V3 hpo = add(hp, mul(n, 1e-3f));
  V3 L = {m.emissiveLin[0], m.emissiveLin[1], m.emissiveLin[2]};
  if (!emissionVisible) L={0,0,0};
  // direct sun at the hit (one shadow ray)
  const float ndl = dot(n, c.sunTo);
  if (ndl > 0.0f) {
    const Hit sh = trace(*c.bvh, hpo, c.sunTo, 3.4e38f, true);
    if (!sh.ok) {
      const float f = ndl * (1.0f / 3.14159265f);
      L = add(L, {m.albedoLin[0] * c.p->sunEnergy[0] * f, m.albedoLin[1] * c.p->sunEnergy[1] * f,
                  m.albedoLin[2] * c.p->sunEnergy[2] * f});
    }
  }
  // direct BAKED local lights at the hit (shadow ray each — occluded, unlike the real-time term)
  if (!c.p->locals.empty())
    L = add(L, localLightsAt(c, hpo, n, m));
  // one cosine-sampled continuation (sky ambient at the hit, or another bounce)
  if (bounces > 0) {
    const V3 nd = cosineDir(n, rng.next(), rng.next());
    const V3 Li = radiance(c, hpo, nd, bounces - 1, rng);
    // cosine-sampled estimator of the diffuse integral: albedo * Li
    L = add(L, {m.albedoLin[0] * Li.x, m.albedoLin[1] * Li.y, m.albedoLin[2] * Li.z});
  }
  return L;
}

}  // namespace

bool omniLightBake(const std::vector<OmniLightTriangle> &tris,
                   const std::vector<OmniLightMaterial> &materials, const OmniLightParams &p,
                   OmniLightVolume &out) {
  using clock = std::chrono::steady_clock;
  out = OmniLightVolume();
  out.triangleCount = tris.size();
  if (tris.empty() || materials.empty() || !p.skySample)
    return false;

  // scene AABB -> probe grid
  float mn[3] = {3.4e38f, 3.4e38f, 3.4e38f}, mx[3] = {-3.4e38f, -3.4e38f, -3.4e38f};
  for (const auto &t : tris) {
    float a[3], b[3];
    triBounds(t, a, b);
    for (int k = 0; k < 3; ++k) {
      mn[k] = std::min(mn[k], a[k]);
      mx[k] = std::max(mx[k], b[k]);
    }
  }
  for (int k = 0; k < 3; ++k) {
    mn[k] -= p.boundsPad;
    mx[k] += p.boundsPad;
    const float extent = std::max(0.1f, mx[k] - mn[k]);
    int d = static_cast<int>(std::floor(extent / p.minSpacing)) + 1;
    d = std::max(2, std::min(p.maxDims[k], d));
    out.dims[k] = d;
    out.spacing[k] = extent / (d - 1);
    out.origin[k] = mn[k];
  }

  const auto t0 = clock::now();
  Bvh bvh;
  buildBvh(bvh, tris);
  out.bvhSeconds = std::chrono::duration<double>(clock::now() - t0).count();

  const int nx = out.dims[0], ny = out.dims[1], nz = out.dims[2];
  const int probeCount = nx * ny * nz;
  out.probeCount = probeCount;
  if (p.progressTotal)
    p.progressTotal->store(probeCount, std::memory_order_relaxed);
  if (p.progressDone)
    p.progressDone->store(0, std::memory_order_relaxed);
  // per-probe: 4 texels {L00+w, L1x, L1y, L1z}, laid out as 4 z-slabs of the 3D texture
  out.texels.assign(static_cast<size_t>(nx) * ny * nz * 4 * 4, 0);

  TraceCtx ctx;
  ctx.bvh = &bvh;
  ctx.tris = &tris;
  ctx.mats = &materials;
  ctx.p = &p;
  ctx.sunTo = norm(v3(p.sunDirTo));

  const int nThreads =
    p.threads > 0 ? p.threads : std::max(1u, std::thread::hardware_concurrency() - 1u);
  std::atomic<int> nextProbe(0);
  std::atomic<int> validCount(0);
  const auto t1 = clock::now();

  auto worker = [&]() {
    const int rays = std::max(16, p.raysPerProbe);
    for (;;) {
      if (p.cancel && p.cancel->load(std::memory_order_relaxed)) return;
      const int pi = nextProbe.fetch_add(1);
      if (pi >= probeCount)
        return;
      if (p.progressDone && (pi & 63) == 0)
        p.progressDone->store(pi, std::memory_order_relaxed);
      const int px = pi % nx, py = (pi / nx) % ny, pz = pi / (nx * ny);
      const V3 pos = {out.origin[0] + px * out.spacing[0], out.origin[1] + py * out.spacing[1],
                      out.origin[2] + pz * out.spacing[2]};
      Rng rng(0x9E3779B97F4A7C15ULL ^ (static_cast<uint64_t>(pi) << 20));
      const float j1 = rng.next(), j2 = rng.next();
      float c0[3] = {0, 0, 0}, cx[3] = {0, 0, 0}, cy[3] = {0, 0, 0}, cz[3] = {0, 0, 0};
      int backfaces = 0;
      for (int r = 0; r < rays; ++r) {
        const V3 d = sphereDir(r, rays, j1, j2);
        // validity probe: an immediate backface = we are inside geometry
        const Hit h0 = trace(bvh, pos, d, 3.4e38f, false);
        if (h0.ok && h0.backface && h0.t < 0.6f)
          ++backfaces;
        const V3 L = radiance(ctx, pos, d, 2, rng);
        // radiance-SH projection (L1): Y00 = 0.282095, Y1m = 0.488603 * dir
        const float w = 12.566371f / rays;  // 4*pi/N solid-angle weight
        const float y0 = 0.282095f * w, y1 = 0.488603f * w;
        c0[0] += L.x * y0;   c0[1] += L.y * y0;   c0[2] += L.z * y0;
        cx[0] += L.x * y1 * d.x; cx[1] += L.y * y1 * d.x; cx[2] += L.z * y1 * d.x;
        cy[0] += L.x * y1 * d.y; cy[1] += L.y * y1 * d.y; cy[2] += L.z * y1 * d.y;
        cz[0] += L.x * y1 * d.z; cz[1] += L.y * y1 * d.z; cz[2] += L.z * y1 * d.z;
      }
      const bool inside = backfaces > rays / 4;
      const float weight = inside ? 0.0f : 1.0f;
      if (!inside)
        validCount.fetch_add(1);
      // Sun visibility for the volumetric shafts: 6 slightly-jittered shadow rays toward the
      // sun. Stored in slab 1's spare alpha — zero extra textures.
      float sunVis = 0.0f;
      if (!inside) {
        int visN = 0;
        for (int sv = 0; sv < 6; ++sv) {
          V3 jd = {ctx.sunTo.x + (rng.next() - 0.5f) * 0.04f,
                   ctx.sunTo.y + (rng.next() - 0.5f) * 0.04f,
                   ctx.sunTo.z + (rng.next() - 0.5f) * 0.04f};
          jd = norm(jd);
          const Hit sh = trace(bvh, pos, jd, 3.4e38f, true);
          if (!sh.ok)
            ++visN;
        }
        sunVis = visN / 6.0f;
      }
      // shader-side irradiance/pi reconstruction: E(n)/pi = c0*Y00 + (2/3)*Y1*(cx*nx+cy*ny+cz*nz)
      // -> premultiply the constants here so the shader does a plain dot.
      const float k0 = 0.282095f * p.outputScale, k1 = 0.488603f * (2.0f / 3.0f) * p.outputScale;
      auto put = [&](int slab, const float *v, float alpha) {
        const size_t base =
          ((static_cast<size_t>(slab) * nz + pz) * ny + py) * nx + px;  // slab-major z
        uint16_t *t = out.texels.data() + base * 4;
        t[0] = toHalf(v[0] * weight);
        t[1] = toHalf(v[1] * weight);
        t[2] = toHalf(v[2] * weight);
        t[3] = toHalf(alpha);
      };
      const float s0[3] = {c0[0] * k0, c0[1] * k0, c0[2] * k0};
      const float sx[3] = {cx[0] * k1, cx[1] * k1, cx[2] * k1};
      const float sy[3] = {cy[0] * k1, cy[1] * k1, cy[2] * k1};
      const float sz[3] = {cz[0] * k1, cz[1] * k1, cz[2] * k1};
      put(0, s0, weight);
      put(1, sx, sunVis);
      put(2, sy, 0.0f);
      put(3, sz, 0.0f);
    }
  };
  std::vector<std::thread> pool;
  for (int i = 0; i < nThreads; ++i)
    pool.emplace_back(worker);
  for (auto &th : pool)
    th.join();
  if (p.cancel && p.cancel->load(std::memory_order_relaxed)) return false;
  out.bakeSeconds = std::chrono::duration<double>(clock::now() - t1).count();
  out.validProbes = validCount.load();

  // ---- specular captures: one broad probe plus up to three local probes ----
  {
    // AABB (unpadded scene bounds recovered from the padded grid extents)
    for (int k = 0; k < 3; ++k) {
      out.aabbMin[k] = out.origin[k] + p.boundsPad * 0.0f;  // padded min (parallax hull)
      out.aabbMax[k] = out.origin[k] + out.spacing[k] * (out.dims[k] - 1);
      out.cubeCenter[k] = 0.5f * (out.aabbMin[k] + out.aabbMax[k]);
    }
    out.cubeCenter[2] = out.aabbMin[2] + 0.35f * (out.aabbMax[2] - out.aabbMin[2]);
    const int S = 64;
    out.cubeSize = S;
    std::vector<V3> centers{v3(out.cubeCenter)};
    for (const auto &position:p.reflectionPositions) {
      if (centers.size()==4) break;
      const V3 point=v3(position.data());
      bool duplicate=false;
      for (V3 other:centers) if (dot(sub(point,other),sub(point,other))<0.25f) duplicate=true;
      if (!duplicate) centers.push_back(point);
    }
    out.cubeCount=static_cast<int>(centers.size());
    auto cubeDirection=[](int face,float u,float v) {
      switch(face) {
        case 0:return norm(V3{1,-v,-u}); case 1:return norm(V3{-1,-v,u});
        case 2:return norm(V3{u,1,v}); case 3:return norm(V3{u,-1,-v});
        case 4:return norm(V3{u,-v,1}); default:return norm(V3{-u,-v,-1});
      }
    };
    for (size_t probe=0;probe<centers.size();++probe) {
    const V3 cc=centers[probe];
    for (int k=0;k<3;++k) {
      out.cubeCenters.push_back((&cc.x)[k]);
      float lo=std::min(out.aabbMin[k],(&cc.x)[k]-0.1f), hi=std::max(out.aabbMax[k],(&cc.x)[k]+0.1f);
      if (probe>0) {
        V3 direction{}; (&direction.x)[k]=1;
        const Hit positive=trace(bvh,cc,direction,hi-(&cc.x)[k],false);
        const Hit negative=trace(bvh,cc,mul(direction,-1),(&cc.x)[k]-lo,false);
        if (positive.ok && positive.t>0.1f) hi=(&cc.x)[k]+positive.t;
        if (negative.ok && negative.t>0.1f) lo=(&cc.x)[k]-negative.t;
      }
      out.cubeBoundsMin.push_back(lo); out.cubeBoundsMax.push_back(hi);
    }
    std::vector<float> face0(static_cast<size_t>(S) * S * 6 * 3);
    std::atomic<int> nextT2(0);
    TraceCtx cctx = ctx;
    auto cubeTrace = [&]() {
      for (;;) {
        if (p.cancel && p.cancel->load(std::memory_order_relaxed)) return;
        const int ti = nextT2.fetch_add(1);
        if (ti >= S * S * 6)
          return;
        const int face = ti / (S * S);
        const int py = (ti / S) % S;
        const int px = ti % S;
        const float u = (px + 0.5f) / S * 2.0f - 1.0f;
        const float v = (py + 0.5f) / S * 2.0f - 1.0f;
        V3 d;
        switch (face) {
          case 0: d = {1, -v, -u}; break;
          case 1: d = {-1, -v, u}; break;
          case 2: d = {u, 1, v}; break;
          case 3: d = {u, -1, -v}; break;
          case 4: d = {u, -v, 1}; break;
          default: d = {-u, -v, -1}; break;
        }
        d = norm(d);
        Rng rng(0xA076159AULL ^ (static_cast<uint64_t>(ti) << 18));
        const V3 L = radiance(cctx, cc, d, 1, rng);
        float *o = face0.data() + static_cast<size_t>(ti) * 3;
        o[0] = L.x; o[1] = L.y; o[2] = L.z;
      }
    };
    std::vector<std::thread> cpool;
    for (int i = 0; i < nThreads; ++i)
      cpool.emplace_back(cubeTrace);
    for (auto &th : cpool)
      th.join();
    if (p.cancel && p.cancel->load(std::memory_order_relaxed)) return false;
    auto sampleCube=[&](V3 d) {
      const float ax=std::abs(d.x),ay=std::abs(d.y),az=std::abs(d.z);
      float u,v,major; int face;
      if (ax>=ay && ax>=az) { face=d.x>=0?0:1; major=ax; u=d.x>=0?-d.z:d.z; v=-d.y; }
      else if (ay>=az) { face=d.y>=0?2:3; major=ay; u=d.x; v=d.y>=0?d.z:-d.z; }
      else { face=d.z>=0?4:5; major=az; u=d.z>=0?d.x:-d.x; v=-d.y; }
      const int x=std::clamp(static_cast<int>((u/major*0.5f+0.5f)*S),0,S-1);
      const int y=std::clamp(static_cast<int>((v/major*0.5f+0.5f)*S),0,S-1);
      return v3(&face0[((face*S+y)*S+x)*3]);
    };
    auto packHalf = [&](const std::vector<float> &src, int ss) {
      for (size_t i = 0; i < static_cast<size_t>(ss) * ss * 6; ++i) {
        out.cubeTexels.push_back(toHalf(src[i * 3]));
        out.cubeTexels.push_back(toHalf(src[i * 3 + 1]));
        out.cubeTexels.push_back(toHalf(src[i * 3 + 2]));
        out.cubeTexels.push_back(toHalf(1.0f));
      }
    };
    packHalf(face0, S);
    // GGX importance prefiltering, with roughness=mip/6. A full 64..1 chain
    // represents the complete roughness range instead of box-blurred mirrors.
    for (int mip=1,ms=S/2;ms>=1;++mip,ms/=2) {
      std::vector<float> filtered(ms*ms*6*3);
      const float roughness=mip/6.0f, alpha=roughness*roughness;
      for (int face=0;face<6;++face) for (int y=0;y<ms;++y) for (int x=0;x<ms;++x) {
        if (p.cancel && p.cancel->load()) return false;
        const V3 n=cubeDirection(face,(x+0.5f)/ms*2-1,(y+0.5f)/ms*2-1);
        const V3 tangent=norm(cross(std::abs(n.z)<0.9f?V3{0,0,1}:V3{1,0,0},n));
        V3 sum; float weight=0;
        Rng rng(0xB076159AULL ^ (face*ms*ms+y*ms+x));
        for (int sample=0;sample<64;++sample) {
          const float u=(sample+0.5f)/64, phi=6.2831853f*rng.next();
          const float c=std::sqrt((1-u)/(1+(alpha*alpha-1)*u)), r=std::sqrt(std::max(0.0f,1-c*c));
          const V3 h=norm(add(add(mul(tangent,r*std::cos(phi)),mul(cross(n,tangent),r*std::sin(phi))),mul(n,c)));
          const V3 l=sub(mul(h,2*dot(n,h)),n); const float nl=std::max(0.0f,dot(n,l));
          sum=add(sum,mul(sampleCube(l),nl)); weight+=nl;
        }
        const V3 value=mul(sum,1/std::max(weight,1e-8f));
        const int i=((face*ms+y)*ms+x)*3;
        filtered[i]=value.x; filtered[i+1]=value.y; filtered[i+2]=value.z;
      }
      packHalf(filtered,ms);
    }
    }
  }

  out.valid = out.validProbes > 0;
  return out.valid;
}
