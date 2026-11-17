// Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
#include "OmPhoto.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <thread>
#include <chrono>

static void plane(OmPhotoScene &s, float x, uint32_t material) {
  OmniLightTriangle t;
  const float points[9] = {x, -20, -20, x, 20, -20, x, 0, 20};
  for (int k = 0; k < 3; ++k) { t.v0[k] = points[k]; t.v1[k] = points[3+k]; t.v2[k] = points[6+k]; }
  t.material = material;
  s.triangles.push_back(t);
  s.attributes.push_back({});
}
int main() {
  std::atomic<bool> cancel{false};
  OmPhotoSettings p;
  p.adaptiveSampling = false;
  p.width = 4; p.height = 4; p.samples = 128; p.timeLimitSeconds = 10;
  OmPhotoScene s;
  s.skyWidth = s.skyHeight = 1; s.sky = {1, 0.5f, 0.25f};
  auto sky = omRenderPhoto(s, p, cancel);
  assert(sky.error.empty() && sky.completedSamples == p.samples);
  assert(sky.linearRgb[0] == 1 && sky.linearRgb[1] == 0.5f && sky.linearRgb[2] == 0.25f);
  assert(sky.linearRgb == omRenderPhoto(s, p, cancel).linearRgb);
  // Cancel a render that has actually started, retaining whole-image samples.
  OmPhotoSettings longRun = p;
  longRun.width = longRun.height = 64; longRun.samples = 4096; longRun.timeLimitSeconds = 5;
  std::atomic<int> progress{0};
  std::atomic<bool> finished{false};
  OmPhotoResult interrupted;
  std::thread worker([&] {
    interrupted = omRenderPhoto(s, longRun, cancel, &progress);
    finished = true;
  });
  while (progress == 0 && !finished) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  cancel = true;
  worker.join();
  assert(interrupted.cancelled && interrupted.completedSamples > 0 && interrupted.completedSamples < longRun.samples);
  for (size_t i = 0; i < interrupted.linearRgb.size(); i += 3)
    assert(interrupted.linearRgb[i] == 1 && interrupted.linearRgb[i + 1] == 0.5f && interrupted.linearRgb[i + 2] == 0.25f);
  cancel = true;
  const auto cancelled = omRenderPhoto(s, p, cancel);
  assert(cancelled.cancelled && cancelled.completedSamples == 0);
  cancel = false;
  p.timeLimitSeconds = 1e-9;
  assert(omRenderPhoto(s, p, cancel).timeLimited);
  p.timeLimitSeconds = 10;
  p.width = -1;
  assert(!omRenderPhoto(s, p, cancel).error.empty());
  p.width = 4;
  s.sky = {1, 1, 1};
  s.materials.push_back({});
  plane(s, 2, 0);
  p.samples = 1024;
  const auto matte = omRenderPhoto(s, p, cancel);
  float mean = 0;
  for (float v : matte.linearRgb) mean += v / matte.linearRgb.size();
  assert(mean > 0.45f && mean < 0.65f); // white-furnace diffuse+dielectric energy
  s.materials[0].opacity = 0;
  const auto invisible = omRenderPhoto(s, p, cancel);
  assert(invisible.linearRgb[0] == 1);
  s.materials[0].opacity = 1;
  s.materials[0].metalness = 1;
  s.materials[0].roughness = 0.025f;
  for (float &c : s.materials[0].color) c = 1;
  s.sky = {0, 0, 0};
  OmPhotoMaterial emitter;
  emitter.emission[0] = 2;
  s.materials.push_back(emitter);
  plane(s, -2, 1); // behind the camera: reflection cannot come from screen pixels
  const auto reflected = omRenderPhoto(s, p, cancel);
  assert(reflected.linearRgb[0] > 1 && reflected.linearRgb[1] == 0);
  assert(omPhotoDisplay(reflected, 1).size() == reflected.linearRgb.size());
  // A white roughness map overrides an authored smooth scalar.
  OmPhotoTexture white;
  white.width = white.height = 1; white.rgba = {255,255,255,255};
  s.textures.push_back(white);
  s.materials[0].roughnessMap = 0;
  const auto mapped = omRenderPhoto(s, p, cancel);
  s.materials[0].roughnessMap = -1; s.materials[0].roughness = 1;
  assert(mapped.linearRgb == omRenderPhoto(s, p, cancel).linearRgb);
  // A cutout texture must reveal the background, not occlude it as a rectangle.
  s.textures[0].rgba[3] = 0; s.materials[0].albedoMap = 0;
  s.triangles.resize(1); s.attributes.resize(1); s.sky = {0.25f,0.5f,1};
  assert(omRenderPhoto(s, p, cancel).linearRgb[2] == 1);
  // Shadowed local lights: an off-camera blocker must remove their contribution.
  s.materials[0] = OmPhotoMaterial(); s.sky = {0,0,0};
  OmniLightLocal light;
  light.pos[0] = 0; light.pos[1] = 4;
  light.colorLin[0] = light.colorLin[1] = light.colorLin[2] = 10;
  s.lights.push_back(light);
  const auto lit = omRenderPhoto(s, p, cancel);
  OmniLightTriangle blocker;
  const float points[9] = {1, 0.5f,-5, 1,5,-5, 1,2.75f,5};
  for (int k=0;k<3;++k) { blocker.v0[k]=points[k]; blocker.v1[k]=points[3+k]; blocker.v2[k]=points[6+k]; }
  s.triangles.push_back(blocker); s.attributes.push_back({});
  p.bounces = 1; // isolate direct lighting, not the blocker's indirect bounce
  const auto blocked = omRenderPhoto(s,p,cancel);
  float bright=0,dark=0;
  for (size_t i=0;i<lit.linearRgb.size();++i) { bright += lit.linearRgb[i]; dark += blocked.linearRgb[i]; }
  assert(bright > 1 && dark < bright * 0.1f);
  // The filter reduces sampling noise while respecting a material boundary.
  OmPhotoResult noisy;
  noisy.linearRgb.resize(8*8*3); noisy.albedoGuide.resize(8*8*3);
  noisy.normalGuide.assign(8*8*3,0); noisy.depthGuide.assign(8*8,1); noisy.variance.assign(8*8,0.04f);
  for (int y=0;y<8;++y) for (int x=0;x<8;++x) {
    const int i=y*8+x;
    const float level=x<4?0.3f:0.9f;
    for (int k=0;k<3;++k) { noisy.linearRgb[i*3+k]=level+((x+y)%2?0.1f:-0.1f); noisy.albedoGuide[i*3+k]=level; }
    noisy.normalGuide[i*3+2]=1;
  }
  const auto filtered=omPhotoReduceGrain(noisy,8,8);
  float errorBefore=0,errorAfter=0;
  for (int y=0;y<8;++y) for (int x=0;x<8;++x) {
    const int i=(y*8+x)*3; const float level=x<4?0.3f:0.9f;
    errorBefore+=std::abs(noisy.linearRgb[i]-level); errorAfter+=std::abs(filtered[i]-level);
  }
  assert(errorAfter<errorBefore*0.4f);
  assert(filtered[3*3]<0.4f && filtered[4*3]>0.8f);
  std::puts("PASS: photo determinism, live cancellation, deadline, validation, furnace, transmission, reflection, maps, cutouts, shadows, grain filter");
}
