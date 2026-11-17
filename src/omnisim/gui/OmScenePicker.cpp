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

#include "OmScenePicker.hpp"

#include "OmAbstractPose.hpp"
#include "OmGizmoLines.hpp"
#include "OmMatrix4.hpp"
#include "OmRenderBackend.hpp"
#include "OmRotation.hpp"
#include "OmSolid.hpp"
#include "OmVector3.hpp"
#include "OmViewpoint.hpp"
#include "OmVulkanBackend.hpp"
#include "OmWgpuMeshCache.hpp"
#include "OmWgpuRenderTarget.hpp"
#include "OmWgpuSceneRenderer.hpp"
#include "OmWorld.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmSFVector3.hpp"
#include "OmSFRotation.hpp"
#include "OmSFDouble.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace {

  // Reserved mesh-cache keys for the transient handle-triangle uploads. Bits 62+63 set: the
  // cache's other key families are heap pointers (bits 62-63 clear), CadShape content hashes
  // (bit 63 set, and their low bits are a hash of real mesh bytes), and primitive keys
  // (bit 62 set, bit 63 clear) -- so a small ordinal with both top bits set cannot collide.
  inline uint64_t handleSlotKey(size_t slot) {
    return (3ULL << 62) | (0x5C1CEULL << 8) | static_cast<uint64_t>(slot & 0xFF);
  }

  // Invert a column-major 4x4 (Gauss-Jordan). Returns false on a singular matrix.
  bool invert4(const float m[16], double out[16]) {
    double a[4][8];
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) {
        a[r][c] = m[c * 4 + r];  // column-major -> row-major working copy
        a[r][c + 4] = (r == c) ? 1.0 : 0.0;
      }
    for (int col = 0; col < 4; ++col) {
      int piv = col;
      for (int r = col + 1; r < 4; ++r)
        if (std::fabs(a[r][col]) > std::fabs(a[piv][col]))
          piv = r;
      if (std::fabs(a[piv][col]) < 1e-12)
        return false;
      if (piv != col)
        for (int c = 0; c < 8; ++c)
          std::swap(a[piv][c], a[col][c]);
      const double d = a[col][col];
      for (int c = 0; c < 8; ++c)
        a[col][c] /= d;
      for (int r = 0; r < 4; ++r) {
        if (r == col)
          continue;
        const double f = a[r][col];
        for (int c = 0; c < 8; ++c)
          a[r][c] -= f * a[col][c];
      }
    }
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        out[c * 4 + r] = a[r][c + 4];  // back to column-major
    return true;
  }

  // out = M * v (column-major, homogeneous). Returns w.
  double xformH(const double m[16], double x, double y, double z, double out3[3]) {
    const double ox = m[0] * x + m[4] * y + m[8] * z + m[12];
    const double oy = m[1] * x + m[5] * y + m[9] * z + m[13];
    const double oz = m[2] * x + m[6] * y + m[10] * z + m[14];
    const double ow = m[3] * x + m[7] * y + m[11] * z + m[15];
    const double w = (std::fabs(ow) > 1e-12) ? ow : 1.0;
    out3[0] = ox / w;
    out3[1] = oy / w;
    out3[2] = oz / w;
    return ow;
  }

  // Moeller-Trumbore. Returns t > 0 on a hit, -1 otherwise.
  double rayTriangle(const double o[3], const double d[3], const float *v0, const float *v1, const float *v2) {
    const double e1[3] = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
    const double e2[3] = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
    const double p[3] = {d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0]};
    const double det = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];
    if (std::fabs(det) < 1e-12)
      return -1.0;
    const double inv = 1.0 / det;
    const double tv[3] = {o[0] - v0[0], o[1] - v0[1], o[2] - v0[2]};
    const double u = (tv[0] * p[0] + tv[1] * p[1] + tv[2] * p[2]) * inv;
    if (u < -1e-6 || u > 1.0 + 1e-6)
      return -1.0;
    const double q[3] = {tv[1] * e1[2] - tv[2] * e1[1], tv[2] * e1[0] - tv[0] * e1[2], tv[0] * e1[1] - tv[1] * e1[0]};
    const double v = (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]) * inv;
    if (v < -1e-6 || u + v > 1.0 + 1e-6)
      return -1.0;
    const double t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * inv;
    return t > 1e-9 ? t : -1.0;
  }

}  // namespace

OmScenePicker::OmScenePicker() :
  mBackend(NULL),
  mSharedCache(NULL),
  mOwnCache(NULL),
  mTarget(NULL),
  mTargetWidth(0),
  mTargetHeight(0),
  mSelectedId(-1),
  mPickedTranslation(0),
  mPickedRotation(0),
  mPickedScale(0),
  mPickedResize(0) {
}

OmScenePicker::~OmScenePicker() {
  delete mTarget;
  delete mOwnCache;
}

OmWgpuMeshCache *OmScenePicker::ensureCache() {
  if (mSharedCache)
    return mSharedCache;
  if (!mOwnCache)
    mOwnCache = new OmWgpuMeshCache(mBackend);
  return mOwnCache;
}

bool OmScenePicker::pick(int x, int y) {
  mWorldCoordinates.setXyz(0.0, 0.0, 0.0);
  mSelectedId = -1;
  mPickedTranslation = 0;
  mPickedRotation = 0;
  mPickedScale = 0;
  mPickedResize = 0;

  OmWorld *const world = OmWorld::instance();
  if (!world)
    return false;
  OmViewpoint *const vp = world->viewpoint();
  if (!vp || !vp->position() || !vp->orientation())
    return false;

  if (!mBackend) {
    mBackend = static_cast<OmVulkanBackend *>(OmRenderBackendRegistry::vulkanBackend());
    if (!mBackend || !mBackend->isAvailable()) {
      mBackend = NULL;
      return false;
    }
  }

  const OmWrenRenderingContext *const ctx = OmWrenRenderingContext::instance();
  const int W = ctx && ctx->width() > 1 ? ctx->width() : 0;
  const int H = ctx && ctx->height() > 1 ? ctx->height() : 0;
  if (W <= 0 || H <= 0 || x < 0 || y < 0 || x >= W || y >= H)
    return false;

  // Camera from the live Viewpoint (same convention as OmView3D::renderMainFrameViaWgpu).
  const OmVector3 eye = vp->position()->value();
  const OmRotation rot = vp->orientation()->value();
  const OmVector3 fwd = rot.direction().normalized();
  const OmVector3 rgt = fwd.cross(rot.up()).normalized();
  const OmVector3 up = rgt.cross(fwd);
  const OmMatrix4 cam(fwd.x(), -rgt.x(), up.x(), eye.x(), fwd.y(), -rgt.y(), up.y(), eye.y(), fwd.z(), -rgt.z(),
                      up.z(), eye.z(), 0, 0, 0, 1);
  const double horizFov = vp->fieldOfView() ? vp->fieldOfView()->value() : 0.785;
  const double aspect = static_cast<double>(W) / static_cast<double>(H);
  const double hf = aspect < 1.0 ? 2.0 * std::atan(std::tan(0.5 * horizFov) * aspect) : horizFov;
  double zNear = 0.05;
  if (vp->nearField() && vp->nearField()->value() > 0.05)
    zNear = vp->nearField()->value();
  float vpm[16];
  // No reversed-Z here: the pick pipeline uses the standard depth convention (same as the
  // sensor and OmWgpuView pick-probe paths).
  OmWgpuSceneRenderer::buildViewProj(cam, hf, aspect, zNear, 1000.0, vpm, false);

  // ---- collect the scene, with the per-draw Solid map ----
  OmWgpuMeshCache *const cache = ensureCache();
  if (!cache)
    return false;
  std::vector<OmWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  std::vector<OmSolid *> nodes;
  OmWgpuSceneRenderer::collectWorldDraws(*cache, draws, modelStorage, &nodes, nullptr, nullptr, nullptr,
                                         OmWgpuSceneRenderer::mainViewHiddenNodes());
  if (draws.size() != nodes.size())
    nodes.resize(draws.size(), NULL);

  // Encode draw index+1 into baseColor (linear RGBA8 -> exact byte round-trip), opaque.
  const size_t sceneDrawCount = draws.size();
  for (size_t i = 0; i < sceneDrawCount; ++i) {
    const uint32_t id = static_cast<uint32_t>(i) + 1u;
    draws[i].baseColorR = static_cast<float>(id & 0xFFu) / 255.0f;
    draws[i].baseColorG = static_cast<float>((id >> 8) & 0xFFu) / 255.0f;
    draws[i].baseColorB = static_cast<float>((id >> 16) & 0xFFu) / 255.0f;
    draws[i].baseColorA = 1.0f;
    draws[i].translucent = false;
  }

  // ---- append the manipulator handles as transient ID draws ----
  // OmGizmoLines emits the exact world triangles it draws; rendering them into the same
  // depth-tested pick pass reproduces WREN's picking semantics (handles depth-tested against
  // the scene) and keeps "drawn == draggable" a single triangle set.
  static std::array<float, 16> sIdentity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::vector<int> handleCodes;  // parallel to the appended draws
  std::vector<std::vector<float>> handleBytesKeepAlive;
  if (OmGizmoLines::anyVisible()) {
    float view16[16];
    OmWgpuSceneRenderer::buildView(cam, view16);
    const double vertFov = 2.0 * std::atan(std::tan(hf * 0.5) / aspect);
    const double p11 = 1.0 / std::tan(vertFov * 0.5);
    const double projMin = std::min(p11 / aspect, p11);
    const float eye3[3] = {static_cast<float>(eye.x()), static_cast<float>(eye.y()), static_cast<float>(eye.z())};
    std::vector<float> gx, gy, gz;
    std::vector<OmGizmoLines::Handle> handles;
    OmGizmoLines::collect(view16, projMin, eye3, gx, gy, gz, &handles);
    if (mHandleSlotBytes.size() < handles.size())
      mHandleSlotBytes.resize(handles.size(), 0);
    for (size_t hi = 0; hi < handles.size(); ++hi) {
      const OmGizmoLines::Handle &h = handles[hi];
      const size_t triCount = h.tris.size() / 9;
      if (triCount == 0)
        continue;
      // Interleave into the standard pos3+norm3+uv2 / stride-32 stream.
      handleBytesKeepAlive.emplace_back();
      std::vector<float> &vb = handleBytesKeepAlive.back();
      const size_t vcount = triCount * 3;
      vb.resize(vcount * 8, 0.0f);
      std::vector<uint32_t> idx(vcount);
      for (size_t vtx = 0; vtx < vcount; ++vtx) {
        vb[vtx * 8 + 0] = h.tris[vtx * 3 + 0];
        vb[vtx * 8 + 1] = h.tris[vtx * 3 + 1];
        vb[vtx * 8 + 2] = h.tris[vtx * 3 + 2];
        idx[vtx] = static_cast<uint32_t>(vtx);
      }
      const uint64_t key = handleSlotKey(hi);
      const size_t bytesLen = vb.size() * sizeof(float);
      OmWgpuMeshHandle mh;
      // The vertices change every pick (world space); the slot is re-written in place when
      // the size matches and rebuilt when it does not.
      if (mHandleSlotBytes[hi] == bytesLen && cache->tryGet(key, mh) && mh.vertexBuffer) {
        cache->updateVertices(key, vb.data(), bytesLen, 32u);
      } else {
        cache->release(key);
        mh = cache->acquire(key, vb.data(), bytesLen, idx.data(), idx.size() * 4u,
                            static_cast<uint32_t>(idx.size()), 32u);
        mHandleSlotBytes[hi] = bytesLen;
      }
      if (!mh.vertexBuffer || !mh.indexBuffer)
        continue;
      // The reserved WREN handle code: (HANDLES_X_AXIS + axis) | TRANSLATE/ROTATE.
      const int code = (HANDLES_X_AXIS + h.axis) | (h.rotate ? HANDLES_ROTATE : HANDLES_TRANSLATE);
      const uint32_t id = static_cast<uint32_t>(draws.size()) + 1u;
      OmWgpuSolidDraw d;
      d.modelMatrix16 = sIdentity.data();
      d.baseColorR = static_cast<float>(id & 0xFFu) / 255.0f;
      d.baseColorG = static_cast<float>((id >> 8) & 0xFFu) / 255.0f;
      d.baseColorB = static_cast<float>((id >> 16) & 0xFFu) / 255.0f;
      d.baseColorA = 1.0f;
      d.vertexBuffer = mh.vertexBuffer;
      d.indexBuffer = mh.indexBuffer;
      d.indexCount = mh.indexCount;
      d.castShadows = false;
      draws.push_back(d);
      handleCodes.push_back(code);
    }
  }

  if (draws.empty())
    return false;

  // ---- render + decode ----
  if (!mTarget || mTargetWidth != W || mTargetHeight != H) {
    delete mTarget;
    mTarget = new OmWgpuRenderTarget(mBackend, static_cast<uint32_t>(W), static_cast<uint32_t>(H));
    mTargetWidth = W;
    mTargetHeight = H;
  }
  if (!mTarget || !mTarget->isUsable())
    return false;
  mReadback.assign(static_cast<size_t>(W) * H * 4, 0);
  OmWgpuClearColor black;  // (0,0,0,1) => background decodes to ID 0 = miss
  const float light[4] = {0.0f, 0.0f, -1.0f, 1.0f};  // ignored by kSolidPick
  if (!mTarget->clearAndDrawScene(black, vpm, light, draws.data(), static_cast<uint32_t>(draws.size()),
                                  mReadback.data(), false, 1.0f, false, nullptr, /*pickMode=*/true))
    return false;

  const size_t px = (static_cast<size_t>(y) * W + x) * 4;  // readback rows are top-down
  const uint32_t id = static_cast<uint32_t>(mReadback[px]) | (static_cast<uint32_t>(mReadback[px + 1]) << 8) |
                      (static_cast<uint32_t>(mReadback[px + 2]) << 16);
  if (id == 0)
    return false;
  const size_t drawIdx = static_cast<size_t>(id) - 1;
  if (drawIdx >= draws.size())
    return false;

  // Handle hit?
  if (drawIdx >= sceneDrawCount) {
    const int code = handleCodes[drawIdx - sceneDrawCount];
    const int axis = code & 0x00000003;
    if ((code & HANDLES_RESIZE) == HANDLES_ROTATE)
      mPickedRotation = axis;
    else
      mPickedTranslation = axis;
    return true;
  }

  // Scene hit: the picked solid's uniqueId, plus a CPU ray hit for the world coordinates.
  OmSolid *const solid = nodes[drawIdx];
  if (!solid)
    return false;
  mSelectedId = solid->uniqueId();

  // Ray through the pixel, from inverse(viewProj) -- convention-free.
  double inv[16];
  bool haveHit = false;
  if (invert4(vpm, inv)) {
    const double ndcX = 2.0 * (x + 0.5) / W - 1.0;
    const double ndcY = 1.0 - 2.0 * (y + 0.5) / H;
    double p0[3], p1[3];
    xformH(inv, ndcX, ndcY, 0.05, p0);
    xformH(inv, ndcX, ndcY, 0.95, p1);
    double dir[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
    const double len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (len > 1e-12) {
      dir[0] /= len;
      dir[1] /= len;
      dir[2] /= len;
      const double origin[3] = {eye.x(), eye.y(), eye.z()};
      double bestT = -1.0;
      for (size_t i = 0; i < sceneDrawCount; ++i) {
        if (nodes[i] != solid)
          continue;
        const OmWgpuSolidDraw &d = draws[i];
        if (!d.cpuPositions || !d.cpuIndices || !d.modelMatrix16)
          continue;
        const std::vector<float> &pos = *d.cpuPositions;
        const std::vector<uint32_t> &ind = *d.cpuIndices;
        // world-transform the vertices once per draw
        std::vector<float> wpos(pos.size());
        const float *mm = d.modelMatrix16;
        for (size_t v = 0; v + 2 < pos.size(); v += 3) {
          wpos[v + 0] = mm[0] * pos[v] + mm[4] * pos[v + 1] + mm[8] * pos[v + 2] + mm[12];
          wpos[v + 1] = mm[1] * pos[v] + mm[5] * pos[v + 1] + mm[9] * pos[v + 2] + mm[13];
          wpos[v + 2] = mm[2] * pos[v] + mm[6] * pos[v + 1] + mm[10] * pos[v + 2] + mm[14];
        }
        for (size_t t = 0; t + 2 < ind.size(); t += 3) {
          const size_t i0 = ind[t] * 3, i1 = ind[t + 1] * 3, i2 = ind[t + 2] * 3;
          if (i2 + 2 >= wpos.size() || i1 + 2 >= wpos.size() || i0 + 2 >= wpos.size())
            continue;
          const double t2 = rayTriangle(origin, dir, &wpos[i0], &wpos[i1], &wpos[i2]);
          if (t2 > 0.0 && (bestT < 0.0 || t2 < bestT))
            bestT = t2;
        }
      }
      if (bestT > 0.0) {
        mWorldCoordinates.setXyz(origin[0] + bestT * dir[0], origin[1] + bestT * dir[1], origin[2] + bestT * dir[2]);
        haveHit = true;
      } else {
        // Fallback: the ray point nearest the draw's bounding centre -- keeps drags anchored
        // sanely when the CPU copy is unavailable.
        const OmWgpuSolidDraw &d = draws[drawIdx];
        double c[3] = {0.0, 0.0, 0.0};
        if (d.modelMatrix16) {
          const float *mm = d.modelMatrix16;
          c[0] = mm[0] * d.localCenter[0] + mm[4] * d.localCenter[1] + mm[8] * d.localCenter[2] + mm[12];
          c[1] = mm[1] * d.localCenter[0] + mm[5] * d.localCenter[1] + mm[9] * d.localCenter[2] + mm[13];
          c[2] = mm[2] * d.localCenter[0] + mm[6] * d.localCenter[1] + mm[10] * d.localCenter[2] + mm[14];
        }
        const double t3 = std::max(zNear, (c[0] - origin[0]) * dir[0] + (c[1] - origin[1]) * dir[1] +
                                            (c[2] - origin[2]) * dir[2]);
        mWorldCoordinates.setXyz(origin[0] + t3 * dir[0], origin[1] + t3 * dir[1], origin[2] + t3 * dir[2]);
        haveHit = true;
      }
    }
  }
  (void)haveHit;
  return true;
}
