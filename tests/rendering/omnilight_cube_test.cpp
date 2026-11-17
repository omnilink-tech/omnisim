// Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
// CPU-side regression for the cubemap upload contract; no graphics context required.
#include "OmniLight.hpp"

#include <cassert>
#include <cstdio>
#include <vector>

int main() {
  OmniLightTriangle triangle;
  triangle.v0[0] = -1; triangle.v0[1] = -1; triangle.v0[2] = 0;
  triangle.v1[0] = 1; triangle.v1[1] = -1; triangle.v1[2] = 0;
  triangle.v2[0] = 0; triangle.v2[1] = 1; triangle.v2[2] = 0;
  OmniLightParams params;
  params.raysPerProbe = 16;
  params.threads = 1;
  for (int axis = 0; axis < 3; ++axis)
    params.maxDims[axis] = 2;
  params.skySample = [](const float *, float *out) {
    out[0] = 0.25f; out[1] = 0.5f; out[2] = 0.75f;
  };
  OmniLightVolume volume;
  assert(omniLightBake({triangle}, {OmniLightMaterial()}, params, volume));
  assert(volume.cubeSize == 64);
  // Complete GGX roughness chain on all six faces, from 64x64 down to 1x1.
  size_t expected=0;
  for (int size=64;size>=1;size/=2) expected+=size*size*6*4;
  assert(volume.cubeTexels.size() == expected);
  for (size_t i = 3; i < expected; i += 4)
    assert(volume.cubeTexels[i] == 0x3c00);  // every texel has float16 alpha=1
  params.reflectionPositions.push_back({0,0,1});
  OmniLightVolume local;
  assert(omniLightBake({triangle},{OmniLightMaterial()},params,local));
  assert(local.cubeCount==2 && local.cubeTexels.size()==expected*2);
  assert(local.cubeCenters.size()==6 && local.cubeBoundsMin.size()==6);
  // A constant radiance field stays constant at every roughness level.
  OmniLightMaterial furnace;
  for (int k=0;k<3;++k) { furnace.albedoLin[k]=0; furnace.emissiveLin[k]=0.5f; }
  params.skySample=[](const float *,float *out) { out[0]=out[1]=out[2]=0.5f; };
  OmniLightVolume constant;
  assert(omniLightBake({triangle},{furnace},params,constant));
  for (size_t i=0;i<constant.cubeTexels.size();++i)
    if (i%4!=3) assert(constant.cubeTexels[i]>=0x37ff && constant.cubeTexels[i]<=0x3801);
  std::puts("PASS: complete GGX cubemap levels and multiple local captures");
}
