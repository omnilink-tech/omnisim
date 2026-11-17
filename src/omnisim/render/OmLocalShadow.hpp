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
#ifndef OM_LOCAL_SHADOW_HPP
#define OM_LOCAL_SHADOW_HPP
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
// Six 90-degree, forward-Z projections. Coordinates agree with WebGPU's Y-flipped UVs.
inline void omLocalShadowMatrix(const float *eye, unsigned face, float farPlane, float *matrix) {
  static const float forward[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
  static const float right[6][3]={{0,0,-1},{0,0,1},{1,0,0},{1,0,0},{1,0,0},{-1,0,0}};
  static const float up[6][3]={{0,1,0},{0,1,0},{0,0,-1},{0,0,1},{0,1,0},{0,1,0}};
  const float nearPlane=.03f, q=std::max(farPlane,.06f)/(std::max(farPlane,.06f)-nearPlane);
  std::fill(matrix,matrix+16,0.0f);
  for (unsigned j=0;j<3;++j) {
    matrix[j*4]=right[face][j]; matrix[j*4+1]=up[face][j];
    matrix[j*4+2]=q*forward[face][j]; matrix[j*4+3]=forward[face][j];
    matrix[12]-=right[face][j]*eye[j]; matrix[13]-=up[face][j]*eye[j];
    matrix[14]-=q*forward[face][j]*eye[j]; matrix[15]-=forward[face][j]*eye[j];
  }
  matrix[14]-=q*nearPlane;
}

// Test the local sphere against the six light-frustum planes in LOCAL space.
// Transforming planes rather than scaling the radius also handles shear and
// non-uniform scale. Unknown or non-finite bounds fail open.
inline bool omOutsideLocalShadow(const float *model, const float *center, float radius, const float *vp) {
  if (!model || !(radius >= 0) || !std::isfinite(radius)) return false;
  for (unsigned plane=0; plane<6; ++plane) {
    float world[4], local[4] = {};
    for (unsigned j=0; j<4; ++j) {
      const float w=vp[j*4+3];
      world[j] = plane<4 ? w + (plane%2 ? -1.0f : 1.0f)*vp[j*4+plane/2] :
                 (plane==4 ? vp[j*4+2] : w-vp[j*4+2]);
    }
    for (unsigned j=0; j<4; ++j)
      for (unsigned k=0; k<4; ++k) local[j] += model[j*4+k]*world[k];
    const float extent=radius*std::sqrt(local[0]*local[0]+local[1]*local[1]+local[2]*local[2]);
    const float distance=local[0]*center[0]+local[1]*center[1]+local[2]*center[2]+local[3];
    if (std::isfinite(distance) && std::isfinite(extent) && distance+extent < -1e-4f*(1+extent)) return true;
  }
  return false;
}

// Exact shadow inputs, without a hash collision or a dependency on camera pose.
// Unknown mesh revisions disable reuse. Publish only after queue submission;
// an abandoned encoder must never leave a supposedly valid atlas behind.
class OmLocalShadowCache {
public:
  void invalidate() { mValid=false; }
  void begin(const float *matrices, const float *meta, bool cull) {
    mPending.clear(); mCacheable=true;
    append(matrices,48*16*sizeof(float)); append(meta,8*4*sizeof(float)); append(&cull,sizeof(cull));
  }
  void caster(uint64_t revision, const void *vb, const void *ib, uint32_t count,
              const float *model, const float *center, float radius) {
    if (!revision || !model) mCacheable=false;
    append(&revision,sizeof(revision)); append(&vb,sizeof(vb)); append(&ib,sizeof(ib)); append(&count,sizeof(count));
    if (model) append(model,16*sizeof(float));
    append(center,3*sizeof(float)); append(&radius,sizeof(radius));
  }
  bool needsRender() const { return !mValid || !mCacheable || mPending!=mStored; }
  void submitted() { mStored.swap(mPending); mValid=mCacheable; }
private:
  void append(const void *data, size_t size) {
    const auto *bytes=static_cast<const uint8_t *>(data);
    mPending.insert(mPending.end(),bytes,bytes+size);
  }
  std::vector<uint8_t> mStored, mPending;
  bool mValid=false, mCacheable=false;
};
#endif
