// Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
#include "OmPhoto.hpp"
#include "OmPhotoDenoise.hpp"
#include "OmLocalShadow.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

static void triangle(OmPhotoScene &s,float x,float y,float z,float size,int material,bool reverse=false) {
  OmniLightTriangle t;
  const float a[3]={x,y-size,z-size}, b[3]={x,y+size,z-size}, c[3]={x,y,z+size};
  std::copy(a,a+3,t.v0); std::copy(b,b+3,t.v1); std::copy(c,c+3,t.v2);
  if (reverse) for (int k=0;k<3;++k) std::swap(t.v1[k],t.v2[k]);
  t.material=material; s.triangles.push_back(t); s.attributes.push_back({});
}
static float red(const OmPhotoResult &r) { assert(r.error.empty()); return r.linearRgb[0]; }
int main(int argc,char **argv) {
  std::atomic<bool> cancel{false};
  OmPhotoSettings p; p.width=p.height=1; p.samples=4096; p.bounces=4;
  p.denoise=false; p.adaptiveSampling=false; p.timeLimitSeconds=10;
  OmPhotoScene s; s.materials.resize(2); s.materials[1].emission[0]=50;
  s.skyWidth=s.skyHeight=1; s.sky={0,0,0};
  triangle(s,2,0,0,20,0); triangle(s,0,2,0,0.15f,1);
  const float reference=red(omRenderPhoto(s,p,cancel));
  float smartError=0, oldError=0;
  p.samples=32;
  for (int i=0;i<32;++i) {
    p.seed=i+1; p.lightSampling=true;
    const float smart=red(omRenderPhoto(s,p,cancel));
    p.lightSampling=false;
    const float old=red(omRenderPhoto(s,p,cancel));
    smartError+=(smart-reference)*(smart-reference); oldError+=(old-reference)*(old-reference);
  }
  std::printf("Emitter MSE: sampled=%g, unsampled=%g, reference=%g\n",smartError/32,oldError/32,reference);
  assert(reference>0 && smartError<oldError*0.25f);
  // A closed slab: Snell refraction exits parallel, eta^2 cancels at both faces,
  // and Beer-Lambert attenuation depends on the actual travelled thickness.
  s.triangles.clear(); s.attributes.clear(); s.sky={1,1,1}; s.materials.resize(1);
  auto &glass=s.materials[0]; glass.refraction=true; glass.opacity=0;
  glass.attenuationColor[1]=0.5f; glass.attenuationColor[2]=0.25f;
  triangle(s,1,0,0,100,0,true); triangle(s,2,0,0,100,0);
  p.samples=2048; p.bounces=16; p.lightSampling=true;
  s.horizontalFov=0.01f;
  auto one=omRenderPhoto(s,p,cancel);
  for (auto *v:{s.triangles[1].v0,s.triangles[1].v1,s.triangles[1].v2}) v[0]=3;
  auto two=omRenderPhoto(s,p,cancel);
  assert(one.linearRgb[0]>0.95f && one.linearRgb[0]<1.05f);
  assert(one.linearRgb[1]>0.45f && one.linearRgb[1]<0.6f);
  assert(two.linearRgb[1]<one.linearRgb[1]*0.65f && two.linearRgb[2]<one.linearRgb[2]*0.5f);
  // Resolve a narrow angular target after a single interface. Snell's law bends
  // 30 degrees in air to 19.47 degrees in glass; straight alpha cannot pass.
  s.materials[0].attenuationColor[1]=s.materials[0].attenuationColor[2]=1;
  s.triangles.resize(1); s.attributes.resize(1);
  s.horizontalFov=.0001f; s.forward[0]=std::sqrt(.75f); s.forward[1]=.5f;
  s.right[0]=-.5f; s.right[1]=std::sqrt(.75f);
  s.skyWidth=720; s.skyHeight=1; s.sky.assign(720*3,0);
  for (int x=0;x<720;++x) {
    const float degrees=(x+.5f)/2-180;
    if (degrees>18 && degrees<21) s.sky[x*3+1]=1;
    if (degrees>28 && degrees<32) s.sky[x*3]=1;
  }
  auto bent=omRenderPhoto(s,p,cancel);
  assert(bent.linearRgb[1]>.4f && bent.linearRgb[0]<.001f);
  s.materials[0].indexOfRefraction=1;
  auto straight=omRenderPhoto(s,p,cancel);
  assert(straight.linearRgb[0]>.99f && straight.linearRgb[1]<.001f);
  // Starting inside a dielectric above its critical angle must totally reflect.
  s.materials[0].indexOfRefraction=1.5f;
  for (int k=0;k<3;++k) std::swap(s.triangles[0].v1[k],s.triangles[0].v2[k]);
  s.forward[0]=.5f; s.forward[1]=std::sqrt(.75f);
  s.right[0]=-std::sqrt(.75f); s.right[1]=.5f;
  s.sky.assign(720*3,0);
  for (int x=0;x<720;++x) if ((x+.5f)/2-180>118 && (x+.5f)/2-180<122) s.sky[x*3]=1;
  assert(red(omRenderPhoto(s,p,cancel))>.99f);
  // Every cube face projects its axis to the centre; moving the light preserves
  // the projection in relative coordinates, with forward depth in [0,1].
  const float eye[3]={3,-2,5};
  for (unsigned face=0;face<6;++face) {
    float matrix[16]; omLocalShadowMatrix(eye,face,20,matrix);
    float point[4]={eye[0],eye[1],eye[2],1}, clip[4]={};
    point[face/2]+=(face%2?-1:1)*2;
    for (int row=0;row<4;++row) for (int col=0;col<4;++col) clip[row]+=matrix[col*4+row]*point[col];
    assert(std::abs(clip[0])<1e-5 && std::abs(clip[1])<1e-5 && clip[3]>0 && clip[2]/clip[3]>0 && clip[2]/clip[3]<1);
  }
  // Quiet pixels stop, with correct means and actual per-pixel sample counts.
  s.triangles.clear(); s.attributes.clear();
  s.skyWidth=s.skyHeight=1; s.sky={1,1,1};
  p.width=p.height=16; p.samples=128; p.adaptiveSampling=true; p.denoise=true;
  auto quiet=omRenderPhoto(s,p,cancel);
  assert(quiet.completedSamples==32 && quiet.averageSamples==32 && quiet.linearRgb[0]==1);
  // Real OIDN integration; noisy flat patches become smoother, raw HDR is retained.
  if (argc>1) {
    const int w=64,h=64; OmPhotoResult noisy;
    noisy.linearRgb.resize(w*h*3); noisy.albedoGuide.resize(w*h*3); noisy.normalGuide.assign(w*h*3,0);
    double before=0;
    for (int i=0;i<w*h;++i) {
      noisy.normalGuide[i*3+2]=1;
      for (int k=0;k<3;++k) { noisy.albedoGuide[i*3+k]=0.5f; noisy.linearRgb[i*3+k]=0.5f+((i*73%101)/100.0f-0.5f)*0.3f; }
      before+=std::pow(noisy.linearRgb[i*3]-0.5f,2);
    }
    const auto raw=noisy.linearRgb; std::string message;
    assert(omPhotoDenoise(noisy,w,h,argv[1],cancel,20,message));
    assert(noisy.linearRgb==raw && noisy.denoiser=="Open Image Denoise 2");
    double after=0; for (int i=0;i<w*h;++i) after+=std::pow(noisy.denoisedRgb[i*3]-0.5f,2);
    std::printf("OIDN MSE: before=%g, after=%g\n",before/(w*h),after/(w*h));
    assert(after<before*0.2);
    cancel=true; assert(!omPhotoDenoise(noisy,w,h,argv[1],cancel,20,message));
  }
  std::puts("PASS: emissive MIS, solid glass absorption, adaptive sampling, OIDN");
}
