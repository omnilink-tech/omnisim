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
// CPU still-image path tracer. GGX + Lambert reflection, thin transmission,
// next-event estimation for sun/point/spot lights, and stochastic indirect light.
// Equations: https://pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory
// Transport: https://pbr-book.org/4ed/Light_Transport_I_Surface_Reflection/A_Better_Path_Tracer
#include "OmPhoto.hpp"
#include "OmTrace.hpp"

#include <chrono>
#include <cmath>
#include <limits>

namespace {
using namespace OmTrace;
constexpr float PI = 3.14159265358979323846f;
constexpr float EPS = 0.0002f;
V3 product(V3 a, V3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
V3 mix(V3 a, V3 b, float t) { return add(mul(a, 1 - t), mul(b, t)); }
float maximum(V3 v) { return std::max({v.x, v.y, v.z}); }
float luminance(V3 v) { return 0.2126f*v.x + 0.7152f*v.y + 0.0722f*v.z; }
float powerWeight(float a, float b) { return a*a / std::max(a*a+b*b, 1e-30f); }
float clamp(float x, float a = 0, float b = 1) { return std::max(a, std::min(b, x)); }
float srgb(float x) { return x <= 0.0031308f ? 12.92f * x : 1.055f * std::pow(x, 1 / 2.4f) - 0.055f; }

std::array<float, 4> texture(const OmPhotoScene &s, int id, float u, float v, bool color) {
  static const std::array<float, 256> linear = []() {
    std::array<float, 256> table;
    for (int i = 0; i < 256; ++i) {
      const float x = i / 255.0f;
      table[i] = x <= 0.04045f ? x / 12.92f : std::pow((x + 0.055f) / 1.055f, 2.4f);
    }
    return table;
  }();
  if (id < 0 || static_cast<size_t>(id) >= s.textures.size())
    return {1, 1, 1, 1};
  const auto &t = s.textures[id];
  if (t.width < 1 || t.height < 1 || t.rgba.size() != static_cast<size_t>(t.width) * t.height * 4)
    return {1, 1, 1, 1};
  // Repeat UVs, bilinear filtering; Qt image rows and the raster texture agree.
  const float x = (u - std::floor(u)) * t.width - 0.5f;
  const float y = (v - std::floor(v)) * t.height - 0.5f;
  const int ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
  const float fx = x - ix, fy = y - iy;
  std::array<float, 4> value = {};
  for (int j = 0; j < 2; ++j)
    for (int i = 0; i < 2; ++i) {
      const int px = (ix + i + t.width) % t.width, py = (iy + j + t.height) % t.height;
      const float weight = (i ? fx : 1 - fx) * (j ? fy : 1 - fy);
      const uint8_t *pixel = &t.rgba[(static_cast<size_t>(py) * t.width + px) * 4];
      for (int k = 0; k < 4; ++k) {
        float f = pixel[k] / 255.0f;
        if (color && k < 3) f = linear[pixel[k]];
        value[k] += f * weight;
      }
    }
  return value;
}

struct Surface {
  V3 point, normal, geometric, color, emission;
  float roughness, metalness, opacity;
  bool cutout, casts, entering, dielectric;
};
Surface surface(const OmPhotoScene &s, const Hit &h, V3 origin, V3 direction) {
  const auto &t = s.triangles[h.tri];
  const auto &a = s.attributes[h.tri];
  const auto &m = s.materials[t.material];
  const float w = 1 - h.u - h.v;
  const float u = a.uv[0] * w + a.uv[2] * h.u + a.uv[4] * h.v;
  const float v = a.uv[1] * w + a.uv[3] * h.u + a.uv[5] * h.v;
  const auto tex = texture(s, m.albedoMap, u, v, true);
  Surface p;
  p.point = add(origin, mul(direction, h.t));
  p.geometric = norm(cross(sub(v3(t.v1), v3(t.v0)), sub(v3(t.v2), v3(t.v0))));
  p.entering = dot(p.geometric, direction) < 0;
  p.dielectric = m.refraction;
  p.normal = add(add(mul(v3(a.normals), w), mul(v3(a.normals + 3), h.u)), mul(v3(a.normals + 6), h.v));
  if (dot(p.normal, p.normal) < 1e-10f) p.normal = p.geometric;
  p.normal = norm(p.normal);
  if (dot(p.normal, p.geometric) < 0) p.normal = mul(p.normal, -1);
  if (m.normalMap >= 0) {
    const float du1 = a.uv[2] - a.uv[0], dv1 = a.uv[3] - a.uv[1];
    const float du2 = a.uv[4] - a.uv[0], dv2 = a.uv[5] - a.uv[1];
    const float det = du1 * dv2 - du2 * dv1;
    if (std::abs(det) > 1e-8f) {
      const V3 e1 = sub(v3(t.v1), v3(t.v0)), e2 = sub(v3(t.v2), v3(t.v0));
      V3 tangent = mul(sub(mul(e1, dv2), mul(e2, dv1)), 1 / det);
      tangent = norm(sub(tangent, mul(p.normal, dot(tangent, p.normal))));
      V3 bitangent = mul(cross(p.normal, tangent), det < 0 ? -1.0f : 1.0f);
      const auto bump = texture(s, m.normalMap, u, v, false);
      p.normal = norm(add(add(mul(tangent, (2 * bump[0] - 1) * m.normalStrength),
                              mul(bitangent, (2 * bump[1] - 1) * m.normalStrength)),
                         mul(p.normal, 2 * bump[2] - 1)));
    }
  }
  if (dot(p.geometric, direction) > 0) p.geometric = mul(p.geometric, -1);
  if (dot(p.normal, p.geometric) < 0) p.normal = mul(p.normal, -1);
  if (dot(p.normal, direction) >= -0.001f) p.normal = p.geometric;
  p.color = product(v3(m.color), {tex[0], tex[1], tex[2]});
  p.emission = m.emissiveTwoSided || p.entering ? v3(m.emission) : V3{0,0,0};
  p.roughness = clamp(m.roughnessMap >= 0 ? texture(s, m.roughnessMap, u, v, false)[0] : m.roughness, 0.025f, 1);
  p.metalness = clamp(m.metalnessMap >= 0 ? texture(s, m.metalnessMap, u, v, false)[0] : m.metalness);
  p.opacity = clamp(m.opacity);
  p.cutout = tex[3] < 0.5f || (p.opacity <= 0 && !m.refraction);
  p.casts = m.castShadows;
  return p;
}

V3 environment(const OmPhotoScene &s, V3 d) {
  if (s.skyWidth < 1 || s.skyHeight < 1 || s.sky.empty()) return {};
  float u = std::atan2(d.y, d.x) / (2 * PI) + 0.5f;
  float v = std::acos(clamp(d.z, -1, 1)) / PI;
  int x = std::min(s.skyWidth - 1, static_cast<int>(u * s.skyWidth));
  int y = std::min(s.skyHeight - 1, static_cast<int>(v * s.skyHeight));
  return v3(&s.sky[(static_cast<size_t>(y) * s.skyWidth + x) * 3]);
}

// Independent light and BSDF samples are combined with the power heuristic.
// Lat-long cells are sampled uniformly in solid angle, including at the poles.
struct LightDistribution {
  std::vector<double> skyCdf, emitterCdf;
  std::vector<uint32_t> emitters;
  std::vector<float> emitterPmf, triangleArea;
  double skyTotal = 0, emitterTotal = 0;
  explicit LightDistribution(const OmPhotoScene &s, bool enabled) {
    if (!enabled) return;
    for (int y=0; y<s.skyHeight; ++y) {
      const double solid = (2*PI/s.skyWidth) * (std::cos(PI*y/s.skyHeight)-std::cos(PI*(y+1)/s.skyHeight));
      for (int x=0; x<s.skyWidth; ++x) {
        skyTotal += std::max(0.0f,luminance(v3(&s.sky[(y*s.skyWidth+x)*3]))) * solid;
        skyCdf.push_back(skyTotal);
      }
    }
    triangleArea.assign(s.triangles.size(),0);
    emitterPmf.assign(s.triangles.size(),0);
    for (uint32_t i=0; i<s.triangles.size(); ++i) {
      const auto &t=s.triangles[i];
      const V3 n=cross(sub(v3(t.v1),v3(t.v0)),sub(v3(t.v2),v3(t.v0)));
      const float area=0.5f*std::sqrt(dot(n,n));
      const float energy=std::max(0.0f,luminance(v3(s.materials[t.material].emission)));
      if (area<=0 || energy<=0) continue;
      triangleArea[i]=area;
      emitterPmf[i]=area*energy;
      emitterTotal+=emitterPmf[i]; emitters.push_back(i); emitterCdf.push_back(emitterTotal);
    }
    if (emitterTotal>0) for (float &p:emitterPmf) p/=emitterTotal;
  }
  float skyPdf(const OmPhotoScene &s, V3 d) const {
    if (skyTotal<=0) return 0;
    return std::max(0.0f,luminance(environment(s,d)))/skyTotal;
  }
  V3 sampleSky(const OmPhotoScene &s, Rng &rng) const {
    const size_t i=std::min(skyCdf.size()-1,static_cast<size_t>(std::upper_bound(skyCdf.begin(),skyCdf.end(),rng.next()*skyTotal)-skyCdf.begin()));
    const int x=i%s.skyWidth, y=i/s.skyWidth;
    // One random variable interpolates the two cell-boundary cosines.
    const float u=rng.next(), c=(1-u)*std::cos(PI*y/s.skyHeight)+u*std::cos(PI*(y+1)/s.skyHeight);
    const float phi=2*PI*((x+rng.next())/s.skyWidth-0.5f), r=std::sqrt(std::max(0.0f,1-c*c));
    return {r*std::cos(phi),r*std::sin(phi),c};
  }
  float emitterPdf(const OmPhotoScene &s, uint32_t id, V3 origin, V3 point) const {
    if (id>=emitterPmf.size() || emitterPmf[id]<=0) return 0;
    const auto &t=s.triangles[id];
    const V3 delta=sub(point,origin), n=norm(cross(sub(v3(t.v1),v3(t.v0)),sub(v3(t.v2),v3(t.v0))));
    const float cosine=std::abs(dot(n,norm(delta)));
    return emitterPmf[id]*dot(delta,delta)/std::max(triangleArea[id]*cosine,1e-20f);
  }
};

float visibility(const OmPhotoScene &s, const Bvh &b, V3 origin, V3 direction, float distance) {
  float transmittance = 1;
  for (int layer = 0; layer < 48; ++layer) {
    const Hit hit = trace(b, origin, direction, distance, false);
    if (!hit.ok) return transmittance;
    // Shadow rays only need alpha/opacity. Decoding normal, colour, roughness
    // and metalness here used to repeat all material work for every local light.
    const auto &triangle = s.triangles[hit.tri];
    const auto &material = s.materials[triangle.material];
    const auto &attr = s.attributes[hit.tri];
    const float w = 1 - hit.u - hit.v;
    const float u = attr.uv[0]*w + attr.uv[2]*hit.u + attr.uv[4]*hit.v;
    const float v = attr.uv[1]*w + attr.uv[3]*hit.u + attr.uv[5]*hit.v;
    const float alpha = texture(s, material.albedoMap, u, v, false)[3];
    if (alpha >= 0.5f && material.opacity > 0 && material.castShadows) {
      transmittance *= (1 - clamp(material.opacity)) * 0.96f;
      if (transmittance < 0.001f) return 0;
    }
    origin = add(origin, mul(direction, hit.t + EPS));
    distance -= hit.t + EPS;
    if (distance <= EPS) return transmittance;
  }
  return 0;  // bounded traversal through pathological overlapping geometry
}

float ggxD(float nh, float alpha) {
  const float a2 = alpha * alpha;
  const float den = nh * nh * (a2 - 1) + 1;
  return a2 / (PI * den * den);
}
float smith(float nx, float alpha) {
  return 2 * nx / (nx + std::sqrt(alpha * alpha + (1 - alpha * alpha) * nx * nx));
}
V3 fresnel(V3 f0, float cosine) { return mix(f0, {1, 1, 1}, std::pow(1 - clamp(cosine), 5)); }
V3 brdf(const Surface &p, V3 wo, V3 wi, float &pdf) {
  const float nv = dot(p.normal, wo), nl = dot(p.normal, wi);
  pdf = 0;
  if (nv <= 0 || nl <= 0 || dot(p.geometric, wi) <= 0) return {};
  if (p.dielectric) { pdf=nl/PI*p.opacity; return mul(p.color,p.opacity/PI); }
  const V3 half = norm(add(wo, wi));
  const float nh = clamp(dot(p.normal, half)), vh = clamp(dot(wo, half));
  const float alpha = p.roughness * p.roughness;
  const float d = ggxD(nh, alpha);
  const V3 f = fresnel(mix({0.04f, 0.04f, 0.04f}, p.color, p.metalness), vh);
  const float specProbability = 0.25f + 0.5f * p.metalness;
  pdf = (1 - specProbability) * nl / PI + specProbability * d * nh / std::max(4 * vh, 1e-8f);
  const V3 diffuse = mul(product(sub({1, 1, 1}, f), p.color), (1 - p.metalness) * p.opacity / PI);
  return add(diffuse, mul(f, d * smith(nv, alpha) * smith(nl, alpha) / (4 * nv * nl)));
}
V3 sampleReflection(const Surface &p, V3 wo, Rng &rng) {
  if (rng.next() >= 0.25f + 0.5f * p.metalness)
    return cosineDir(p.normal, rng.next(), rng.next());
  const float alpha = p.roughness * p.roughness, u = rng.next();
  const float c = std::sqrt((1 - u) / (1 + (alpha * alpha - 1) * u));
  const float r = std::sqrt(std::max(0.0f, 1 - c * c)), phi = 2 * PI * rng.next();
  const V3 tangent = norm(cross(std::abs(p.normal.z) < 0.9f ? V3{0, 0, 1} : V3{1, 0, 0}, p.normal));
  const V3 h = norm(add(add(mul(tangent, r * std::cos(phi)), mul(cross(p.normal, tangent), r * std::sin(phi))), mul(p.normal, c)));
  return sub(mul(h, 2 * dot(wo, h)), wo);
}
V3 directLight(const OmPhotoScene &s, const Bvh &b, const LightDistribution &lights,
               const Surface &p, V3 wo, Rng &rng, bool continuation) {
  V3 result;
  auto illuminate = [&](V3 wi, V3 energy, float distance) {
    float pdf;
    const V3 f = brdf(p, wo, wi, pdf);
    if (maximum(f) <= 0) return;
    const float vis = visibility(s, b, add(p.point, mul(p.geometric, EPS)), wi, distance);
    result = add(result, mul(product(f, energy), vis * std::max(0.0f, dot(p.normal, wi))));
  };
  if (maximum(v3(s.sunEnergy)) > 0) {
    const V3 sun = norm(v3(s.sunTo));
    const V3 t = norm(cross(std::abs(sun.z) < 0.9f ? V3{0, 0, 1} : V3{1, 0, 0}, sun));
    const float r = std::sqrt(rng.next()) * s.sunAngularRadius, phi = 2 * PI * rng.next();
    const V3 wi = norm(add(sun, add(mul(t, r * std::cos(phi)), mul(cross(sun, t), r * std::sin(phi)))));
    illuminate(wi, v3(s.sunEnergy), 1e30f);
  }
  for (const auto &l : s.lights) {
    if (l.type == 0) { illuminate(norm(mul(v3(l.pos), -1)), v3(l.colorLin), 1e30f); continue; }
    const V3 delta = sub(v3(l.pos), p.point);
    const float dist = std::sqrt(dot(delta, delta));
    if (dist < EPS || (l.radius > 0 && dist > l.radius)) continue;
    const V3 wi = mul(delta, 1 / dist);
    float scale = 1 / std::max(0.0001f, l.atten[0] + dist * l.atten[1] + dist * dist * l.atten[2]);
    if (l.type == 2) {
      const float angle = dot(norm(v3(l.spotDir)), mul(wi, -1));
      if (angle <= l.cosCut) continue;
      scale *= clamp((angle - l.cosCut) / std::max(0.0001f, l.cosBeam - l.cosCut));
    }
    illuminate(wi, mul(v3(l.colorLin), scale), dist - EPS);
  }
  auto sampled = [&](V3 wi, V3 energy, float distance, float lightPdf) {
    if (lightPdf<=0) return;
    float bsdfPdf;
    const V3 f=brdf(p,wo,wi,bsdfPdf);
    if (maximum(f)<=0) return;
    if (!p.dielectric) {
      const float fr=0.04f+0.96f*std::pow(1-clamp(dot(p.normal,wo)),5);
      bsdfPdf*=1-(1-p.opacity)*(1-p.metalness)*(1-fr);
    }
    const float weight=continuation?powerWeight(lightPdf,bsdfPdf):1;
    const float vis=visibility(s,b,add(p.point,mul(p.geometric,EPS)),wi,distance);
    result=add(result,mul(product(f,energy),vis*weight*std::max(0.0f,dot(p.normal,wi))/lightPdf));
  };
  if (lights.skyTotal>0) {
    const V3 wi=lights.sampleSky(s,rng);
    sampled(wi,environment(s,wi),1e30f,lights.skyPdf(s,wi));
  }
  if (lights.emitterTotal>0) {
    const size_t k=std::min(lights.emitters.size()-1,static_cast<size_t>(std::upper_bound(lights.emitterCdf.begin(),lights.emitterCdf.end(),rng.next()*lights.emitterTotal)-lights.emitterCdf.begin()));
    const uint32_t id=lights.emitters[k]; const auto &t=s.triangles[id];
    const float root=std::sqrt(rng.next()), u=root*(1-rng.next()), v=root-u;
    const V3 point=add(add(mul(v3(t.v0),1-root),mul(v3(t.v1),u)),mul(v3(t.v2),v));
    const V3 delta=sub(point,p.point); const float distance=std::sqrt(dot(delta,delta));
    if (distance>2*EPS) {
      Hit hit; hit.tri=id; hit.u=u; hit.v=v; hit.t=distance;
      const Surface emitter=surface(s,hit,p.point,mul(delta,1/distance));
      if (!emitter.cutout) sampled(mul(delta,1/distance),emitter.emission,distance-2*EPS,lights.emitterPdf(s,id,p.point,point));
    }
  }
  return result;
}

struct Guide { V3 albedo{1,1,1}, normal{}; float depth=0; bool found=false; };
V3 path(const OmPhotoScene &s, const Bvh &b, const LightDistribution &lights,
         V3 origin, V3 direction, int bounces, Rng &rng, Guide *guide=nullptr) {
  V3 radiance, weight{1, 1, 1};
  V3 previous=origin;
  float previousPdf=0;
  bool deltaBounce=true;
  uint32_t media[16]={}; int mediumCount=0;
  float totalDistance=0;
  int transparentLayers = 0;
  for (int bounce = 0; bounce < bounces;) {
    const Hit h = trace(b, origin, direction, 1e30f, false);
    if (!h.ok) {
      const float mis=deltaBounce?1:powerWeight(previousPdf,lights.skyPdf(s,direction));
      radiance=add(radiance,mul(product(weight,environment(s,direction)),mis)); break;
    }
    if (mediumCount>0) {
      const auto &medium=s.materials[media[mediumCount-1]];
      for (int k=0;k<3;++k)
        (&weight.x)[k]*=std::pow(clamp(medium.attenuationColor[k],0.0001f,1),h.t/std::max(medium.attenuationDistance,0.0001f));
    }
    const Surface p = surface(s, h, origin, direction);
    totalDistance+=h.t;
    if (p.cutout) {
      if (++transparentLayers >= 48) break;
      origin = add(p.point, mul(direction, EPS));
      continue;
    }
    transparentLayers = 0;
    if (guide && !guide->found && !p.dielectric) {
      guide->albedo=p.color; guide->normal=p.normal; guide->depth=totalDistance; guide->found=true;
    }
    const V3 wo = mul(direction, -1);
    const float mis=deltaBounce?1:powerWeight(previousPdf,lights.emitterPdf(s,h.tri,previous,p.point));
    radiance=add(radiance,product(weight,add(mul(p.emission,mis),directLight(s,b,lights,p,wo,rng,bounce+1<bounces))));
    if (bounce+1>=bounces) break;
    previous=p.point;
    if (p.dielectric) {
      const uint32_t materialId=s.triangles[h.tri].material;
      const auto &material=s.materials[materialId];
      if (rng.next()<p.opacity) {
        direction=cosineDir(p.normal,rng.next(),rng.next());
        weight=product(weight,p.color);
        previousPdf=p.opacity*std::max(0.0f,dot(p.normal,direction))/PI;
        deltaBounce=false;
      } else {
        const float currentIor=mediumCount?s.materials[media[mediumCount-1]].indexOfRefraction:1;
        const float etaI=p.entering?currentIor:(mediumCount?currentIor:material.indexOfRefraction);
        const float etaT=p.entering?material.indexOfRefraction:(mediumCount>1?s.materials[media[mediumCount-2]].indexOfRefraction:1);
        const float eta=etaI/etaT, cosine=clamp(dot(p.geometric,wo));
        const float sin2=eta*eta*(1-cosine*cosine);
        const float ct=std::sqrt(std::max(0.0f,1-sin2));
        const float rs=(etaI*cosine-etaT*ct)/std::max(etaI*cosine+etaT*ct,1e-8f);
        const float rp=(etaT*cosine-etaI*ct)/std::max(etaT*cosine+etaI*ct,1e-8f);
        const float fresnel=sin2>=1?1:0.5f*(rs*rs+rp*rp);
        if (rng.next()<fresnel) direction=sub(mul(p.geometric,2*cosine),wo);
        else {
          direction=norm(add(mul(direction,eta),mul(p.geometric,eta*cosine-ct)));
          weight=mul(weight,eta*eta); // radiance transport across the dielectric interface
          if (p.entering) { if (mediumCount==16) break; media[mediumCount++]=materialId; }
          else if (mediumCount) --mediumCount;
        }
        deltaBounce=true; previousPdf=0;
      }
      origin=add(p.point,mul(direction,EPS));
      ++bounce;
      continue;
    }
    // Authored alpha-blended materials are thin sheets: preserve straight-through
    // transmission and trace their reflection. No invented glass volume or IOR.
    const float f = 0.04f + 0.96f * std::pow(1 - clamp(dot(p.normal, wo)), 5);
    const float transmission = (1 - p.opacity) * (1 - p.metalness) * (1 - f);
    if (rng.next() < transmission) {
      weight = product(weight, mix({1, 1, 1}, p.color, 0.15f));
      origin = add(p.point, mul(direction, EPS));
      deltaBounce=true; previousPdf=0;
    } else {
      const V3 wi = sampleReflection(p, wo, rng);
      float pdf;
      const V3 fvalue = brdf(p, wo, wi, pdf);
      pdf *= 1 - transmission;
      if (pdf <= 1e-10f) break;
      weight = product(weight, mul(fvalue, std::max(0.0f, dot(p.normal, wi)) / pdf));
      direction = wi;
      previousPdf=pdf; deltaBounce=false;
      origin = add(p.point, mul(p.geometric, EPS));
    }
    ++bounce;
    if (bounce >= 3) {
      const float survive = clamp(maximum(weight), 0.05f, 0.95f);
      if (rng.next() >= survive) break;
      weight = mul(weight, 1 / survive);
    }
  }
  return radiance;
}
}  // namespace

OmPhotoResult omRenderPhoto(const OmPhotoScene &s, const OmPhotoSettings &p,
                            const std::atomic<bool> &cancel, std::atomic<int> *progress) {
  OmPhotoResult result;
  if (p.width < 1 || p.height < 1 || p.width > 8192 || p.height > 8192 ||
      static_cast<uint64_t>(p.width) * p.height > 16000000 || p.samples < 1 || p.samples > 4096 ||
      p.bounces < 1 || p.bounces > 32 || !std::isfinite(p.timeLimitSeconds) ||
      p.timeLimitSeconds <= 0 || p.timeLimitSeconds > 3600 ||
      !std::isfinite(p.relativeError) || p.relativeError<=0 ||
      !std::isfinite(s.horizontalFov) || s.horizontalFov <= 0 || s.horizontalFov >= PI ||
      s.attributes.size() != s.triangles.size() || s.skyWidth < 0 || s.skyHeight < 0 ||
      s.sky.size() != static_cast<size_t>(s.skyWidth) * s.skyHeight * 3) {
    result.error = "Invalid photo dimensions, sampling limits, camera or scene attributes.";
    return result;
  }
  for (const auto &m:s.materials)
    if (m.refraction && (!std::isfinite(m.indexOfRefraction) || m.indexOfRefraction<1 ||
                         !std::isfinite(m.attenuationDistance) || m.attenuationDistance<=0)) {
      result.error="Invalid dielectric material."; return result;
    }
  for (const auto &t : s.triangles)
    if (t.material >= s.materials.size()) { result.error = "Invalid triangle material."; return result; }
  const auto start = std::chrono::steady_clock::now();
  auto elapsed = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(); };
  auto stopped = [&]() {
    result.cancelled = cancel.load(std::memory_order_relaxed);
    result.timeLimited = elapsed() >= p.timeLimitSeconds;
    return result.cancelled || result.timeLimited;
  };
  if (stopped()) return result;
  Bvh b;
  buildBvh(b, s.triangles);
  LightDistribution lights(s,p.lightSampling);
  const size_t count = static_cast<size_t>(p.width) * p.height * 3;
  result.linearRgb.assign(count, 0);
  std::vector<float> sample(count, 0);
  std::vector<float> squares(count / 3, 0);
  result.sampleCounts.assign(count/3,0);
  std::vector<uint8_t> converged(count/3,0);
  std::vector<Guide> sampleGuides(p.denoise?count/3:0);
  if (p.denoise) {
    result.albedoGuide.assign(count, 0);
    result.normalGuide.assign(count, 0);
    result.depthGuide.assign(count / 3, 0);
  }
  const float halfW = std::tan(0.5f * s.horizontalFov), halfH = halfW * p.height / p.width;
  for (int n = 0; n < p.samples && !stopped(); ++n) {
    bool interrupted = false;
    for (int y = 0; y < p.height && !interrupted; ++y)
      for (int x = 0; x < p.width; ++x) {
        if ((x & 31) == 0 && stopped()) { interrupted = true; break; }
        const size_t pixel = static_cast<size_t>(y) * p.width + x;
        if (converged[pixel]) continue;
        Rng rng(p.seed ^ (pixel * 0x9e3779b97f4a7c15ULL) ^ (static_cast<uint64_t>(n + 1) * 0xd1b54a32d192ed03ULL));
        const float sx = (2 * (x + rng.next()) / p.width - 1) * halfW;
        const float sy = (1 - 2 * (y + rng.next()) / p.height) * halfH;
        const V3 direction = norm(add(add(v3(s.forward), mul(v3(s.right), sx)), mul(v3(s.up), sy)));
        Guide guide;
        const V3 color = path(s, b, lights, v3(s.eye), direction, p.bounces, rng, p.denoise?&guide:nullptr);
        if (p.denoise) sampleGuides[pixel]=guide;
        for (int k = 0; k < 3; ++k) {
          const float v = (&color.x)[k];
          sample[pixel * 3 + k] = std::isfinite(v) ? std::max(0.0f, v) : 0;
        }
      }
    if (interrupted) break;
    size_t active=0;
    for (size_t i = 0; i < count / 3; ++i) {
      if (converged[i]) continue;
      const uint32_t spp=++result.sampleCounts[i];
      for (int k=0;k<3;++k) result.linearRgb[i*3+k]+=sample[i*3+k];
      if (p.denoise) {
        for (int k=0;k<3;++k) {
          result.albedoGuide[i*3+k]+=(&sampleGuides[i].albedo.x)[k];
          result.normalGuide[i*3+k]+=(&sampleGuides[i].normal.x)[k];
        }
        result.depthGuide[i]+=sampleGuides[i].depth;
      }
      const float luminance = 0.2126f*sample[i*3]+0.7152f*sample[i*3+1]+0.0722f*sample[i*3+2];
      squares[i] += luminance*luminance;
      const float mean=(0.2126f*result.linearRgb[i*3]+0.7152f*result.linearRgb[i*3+1]+0.0722f*result.linearRgb[i*3+2])/spp;
      const float variance=std::max(0.0f,squares[i]/spp-mean*mean);
      if (p.adaptiveSampling && spp>=32 && 1.96f*std::sqrt(variance/(spp-1))<p.relativeError*(0.05f+mean))
        converged[i]=1;
      else ++active;
    }
    result.completedSamples = n + 1;
    if (progress) progress->store(n + 1, std::memory_order_relaxed);
    if (!active) break;
  }
  result.completedSamples=*std::min_element(result.sampleCounts.begin(),result.sampleCounts.end());
  result.variance.assign(count/3,0);
  for (size_t i=0;i<count/3;++i) {
    const uint32_t spp=result.sampleCounts[i];
    result.averageSamples+=spp;
    if (!spp) continue;
    for (int k=0;k<3;++k) {
      result.linearRgb[i*3+k]/=spp;
      if (p.denoise) { result.albedoGuide[i*3+k]/=spp; result.normalGuide[i*3+k]/=spp; }
    }
    if (p.denoise) result.depthGuide[i]/=spp;
    const float mean=luminance(v3(&result.linearRgb[i*3]));
    if (spp>1) result.variance[i]=std::max(0.0f,squares[i]/spp-mean*mean)/(spp-1);
  }
  result.averageSamples/=count/3;
  result.seconds = elapsed();
  return result;
}

std::vector<float> omPhotoReduceGrain(const OmPhotoResult &r, int width, int height) {
  const size_t count = r.linearRgb.size() / 3;
  if (width < 1 || height < 1 || static_cast<size_t>(width)*height != count ||
      r.albedoGuide.size() != count*3 || r.normalGuide.size() != count*3 ||
      r.depthGuide.size() != count || r.variance.size() != count)
    return r.linearRgb;
  // Joint edge-aware a-trous filtering, guided by the first visible surface.
  // Work on demodulated radiance so brick/wood albedo texture detail survives.
  std::vector<float> current(count*3), next(count*3);
  for (size_t i=0;i<count*3;++i) current[i] = r.linearRgb[i] / (0.1f+r.albedoGuide[i]);
  for (int step : {1,2,4}) {
    for (int y=0;y<height;++y)
      for (int x=0;x<width;++x) {
        const size_t i=static_cast<size_t>(y)*width+x;
        V3 sum; float total=0;
        const float centerLum=0.2126f*r.linearRgb[i*3]+0.7152f*r.linearRgb[i*3+1]+0.0722f*r.linearRgb[i*3+2];
        for (int dy=-1;dy<=1;++dy)
          for (int dx=-1;dx<=1;++dx) {
            const int xx=x+dx*step, yy=y+dy*step;
            if (xx<0 || yy<0 || xx>=width || yy>=height) continue;
            const size_t j=static_cast<size_t>(yy)*width+xx;
            if ((r.depthGuide[i]==0)!=(r.depthGuide[j]==0)) continue;
            const V3 ni=v3(&r.normalGuide[i*3]), nj=v3(&r.normalGuide[j*3]);
            if (r.depthGuide[i]>0 && dot(ni,nj)<0.85f) continue;
            const V3 delta=sub(v3(&r.albedoGuide[i*3]),v3(&r.albedoGuide[j*3]));
            const float depth=std::abs(r.depthGuide[i]-r.depthGuide[j]) / (0.01f+0.015f*r.depthGuide[i]*step);
            const float lum=0.2126f*r.linearRgb[j*3]+0.7152f*r.linearRgb[j*3+1]+0.0722f*r.linearRgb[j*3+2];
            const float sigma=0.015f+4*std::sqrt(r.variance[i]+r.variance[j]);
            const float weight=(dx==0?2.0f:1.0f)*(dy==0?2.0f:1.0f)*
              std::exp(-dot(delta,delta)/0.02f-depth-std::abs(lum-centerLum)/sigma);
            sum=add(sum,mul(v3(&current[j*3]),weight)); total+=weight;
          }
        for (int k=0;k<3;++k) next[i*3+k]=(&sum.x)[k]/std::max(total,1e-10f);
      }
    current.swap(next);
  }
  for (size_t i=0;i<count*3;++i) current[i] *= 0.1f+r.albedoGuide[i];
  return current;
}

std::vector<uint8_t> omPhotoDisplay(const OmPhotoResult &result, float exposure, int width, int height) {
  const auto radiance = result.denoisedRgb.size()==result.linearRgb.size() && !result.denoisedRgb.empty()?
    result.denoisedRgb:omPhotoReduceGrain(result, width, height);
  std::vector<uint8_t> pixels(radiance.size());
  for (size_t i = 0; i + 2 < pixels.size(); i += 3) {
    // AgX inset, log encoding and sigmoid, then outset to linear display RGB.
    const float r = radiance[i] * exposure, g = radiance[i + 1] * exposure, b = radiance[i + 2] * exposure;
    float c[3] = {0.842479f*r + 0.0784336f*g + 0.0792237f*b,
                  0.0423282f*r + 0.8784686f*g + 0.0791661f*b,
                  0.0423757f*r + 0.0784336f*g + 0.87914297f*b};
    for (float &x : c) {
      x = clamp((std::log2(std::max(x, 1e-10f)) + 12.47393f) / 16.5f);
      const float x2 = x*x, x4 = x2*x2;
      x = std::pow(clamp(15.5f*x4*x2 - 40.14f*x4*x + 31.96f*x4 - 6.868f*x2*x + 0.4298f*x2 + 0.1191f*x - 0.00232f), 2.2f);
    }
    const float out[3] = {1.196879f*c[0] - 0.0980209f*c[1] - 0.0990297f*c[2],
                         -0.0528969f*c[0] + 1.151903f*c[1] - 0.0989612f*c[2],
                         -0.0529716f*c[0] - 0.0980435f*c[1] + 1.1510737f*c[2]};
    for (int k = 0; k < 3; ++k) pixels[i + k] = static_cast<uint8_t>(std::lround(clamp(srgb(std::max(0.0f, out[k]))) * 255));
  }
  return pixels;
}
