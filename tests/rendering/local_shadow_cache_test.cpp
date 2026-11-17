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
#include "OmLocalShadow.hpp"
#include <cassert>
#include <cstdio>
#include <limits>
#include <random>

int main() {
  float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  float eye[3]={0,0,0}, center[3]={0,0,0}, matrices[48*16]={}, meta[32]={};
  for (unsigned face=0; face<6; ++face) omLocalShadowMatrix(eye,face,10,matrices+face*16);
  float model[16]; std::copy(identity,identity+16,model);
  model[12]=3;
  assert(!omOutsideLocalShadow(model,center,.1f,matrices));
  assert(omOutsideLocalShadow(model,center,.1f,matrices+16));
  model[12]=11;
  assert(omOutsideLocalShadow(model,center,.1f,matrices));
  assert(!omOutsideLocalShadow(model,center,-1,matrices));
  assert(!omOutsideLocalShadow(model,center,std::numeric_limits<float>::quiet_NaN(),matrices));
  // No false rejection: whenever any sampled point in a transformed sphere is
  // inside a frustum, the sphere must survive. Include shear, mirrored scales,
  // near/far intersections and cubemap seams, with a fixed reproducible seed.
  std::mt19937 rng(9137);
  std::uniform_real_distribution<float> rnd(-1,1);
  unsigned witnesses=0;
  for (unsigned trial=0; trial<10000; ++trial) {
    std::copy(identity,identity+16,model);
    for (unsigned c=0;c<3;++c) for(unsigned r=0;r<3;++r) model[c*4+r]=rnd(rng)*3;
    for (unsigned c=0;c<3;++c) model[12+c]=rnd(rng)*12;
    for (unsigned sample=0;sample<40;++sample) {
      float p[4]={rnd(rng),rnd(rng),rnd(rng),1}, w[4]={};
      if (p[0]*p[0]+p[1]*p[1]+p[2]*p[2]>1) continue;
      for(unsigned r=0;r<4;++r) for(unsigned c=0;c<4;++c) w[r]+=model[c*4+r]*p[c];
      for(unsigned face=0;face<6;++face) {
        const float *vp=matrices+face*16;
        float clip[4]={};
        for(unsigned r=0;r<4;++r) for(unsigned c=0;c<4;++c) clip[r]+=vp[c*4+r]*w[c];
        if (clip[3]>0 && std::abs(clip[0])<=clip[3] && std::abs(clip[1])<=clip[3] && clip[2]>=0 && clip[2]<=clip[3]) {
          ++witnesses;
          assert(!omOutsideLocalShadow(model,center,1,vp));
        }
      }
    }
  }
  assert(witnesses>1000);
  OmLocalShadowCache cache;
  std::copy(identity,identity+16,model);
  auto input=[&](uint64_t revision=1, bool include=true) {
    cache.begin(matrices,meta,true);
    if (include) cache.caster(revision,identity,eye,36,model,center,1);
  };
  input(); assert(cache.needsRender()); // first frame, or an unsubmitted encoder
  input(); assert(cache.needsRender());
  cache.submitted(); input(); assert(!cache.needsRender());
  model[12]=1; input(); assert(cache.needsRender()); // moving caster
  cache.submitted(); input(); assert(!cache.needsRender());
  input(2); assert(cache.needsRender()); // in-place deformation, same GPU handles
  cache.submitted(); input(2); assert(!cache.needsRender());
  matrices[12]+=1; input(2); assert(cache.needsRender()); // moving light
  cache.submitted(); input(2,false); assert(cache.needsRender()); // removal / transparency / castShadows
  cache.submitted(); input(2); assert(cache.needsRender());
  cache.submitted(); meta[0]=1; input(2); assert(cache.needsRender()); // enable / atlas layer
  cache.submitted(); input(0); assert(cache.needsRender()); // unversioned external buffers
  cache.submitted(); input(0); assert(cache.needsRender());
  input(); cache.submitted(); cache.invalidate(); input(); assert(cache.needsRender());
  std::printf("Local shadow cache transitions and %u visible-point witnesses passed.\n",witnesses);
}
