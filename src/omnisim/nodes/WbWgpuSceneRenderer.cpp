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

#include "WbWgpuSceneRenderer.hpp"

#include "WbBox.hpp"
#include "WbBasicJoint.hpp"
#include "WbCadShape.hpp"
#include "WbCapsule.hpp"
#include "WbCylinder.hpp"
#include "WbGroup.hpp"
#include "WbImageTexture.hpp"
#include "WbLog.hpp"
#include "WbMatrix4.hpp"
#include "WbAppearance.hpp"
#include "WbMaterial.hpp"
#include "WbPbrAppearance.hpp"
#include "WbPlane.hpp"
#include "WbRenderBackend.hpp"
#include "WbRgb.hpp"
#include "WbShape.hpp"
#include "WbTextureTransform.hpp"
#include "WbSolid.hpp"
#include "WbSphere.hpp"
#include "WbVulkanBackend.hpp"
#include "WbWgpuImageAdapter.hpp"
#include "WbWgpuMeshAdapter.hpp"
#include "WbWgpuMeshCache.hpp"
#include "WbWgpuRenderTarget.hpp"
#include "WbWgpuTextureCache.hpp"
#include "WbWorld.hpp"

#include <wren/transform.h>  // wr_transform_get_matrix — CadShape submesh world matrices

#include <QtGui/QImage>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {

  // Column-major (WGSL / OpenGL) perspective from a vertical FOV (radians),
  // aspect, near/far. Right-handed, depth → [0, 1] (wgpu NDC). Verbatim from
  // WbCamera.cpp's wgpuPerspective.
  void perspective(double fovY, double aspect, double zNear, double zFar, float out16[16]) {
    const double f = 1.0 / std::tan(fovY * 0.5);
    for (int i = 0; i < 16; ++i)
      out16[i] = 0.0f;
    out16[0] = static_cast<float>(f / aspect);                       // m00
    out16[5] = static_cast<float>(f);                                // m11
    out16[10] = static_cast<float>(zFar / (zNear - zFar));           // m22
    out16[11] = -1.0f;                                               // m23 (clip.w = -view.z)
    out16[14] = static_cast<float>((zFar * zNear) / (zNear - zFar)); // m32
  }

  // Column-major orthographic projection: light frustum [-h,+h] in X/Y,
  // [zNear,zFar] along -view.z, depth → [0,1] (wgpu NDC). Right-handed like
  // perspective() above (looks down -Z), so it composes with buildView's basis
  // swap identically. m22/m32 map z∈[-zNear,-zFar] (view space) → [0,1] clip.
  void ortho(double h, double zNear, double zFar, float out16[16]) {
    for (int i = 0; i < 16; ++i)
      out16[i] = 0.0f;
    out16[0] = static_cast<float>(1.0 / h);                  // m00: x/h
    out16[5] = static_cast<float>(1.0 / h);                  // m11: y/h
    out16[10] = static_cast<float>(1.0 / (zNear - zFar));    // m22
    out16[14] = static_cast<float>(zNear / (zNear - zFar));  // m32 (z=-zNear→0, z=-zFar→1)
    out16[15] = 1.0f;                                        // m33: affine (w=1)
  }

  // Off-center column-major orthographic projection: x∈[l,r]→[-1,1], y∈[b,t]→[-1,1],
  // view-space depth z∈[-n,-f]→[0,1] (wgpu NDC, looks down -Z). The general form of
  // ortho() above — the symmetric case l=-h,r=h,b=-h,t=h,n=zNear,f=zFar reproduces it
  // bit-for-bit. A fitted CSM cascade is rarely centered on the light axis, so the
  // tight per-cascade frustum needs the x/y offset terms (cols 3 rows 0/1).
  void orthoOffCenter(double l, double r, double b, double t, double n, double f,
                      float out16[16]) {
    for (int i = 0; i < 16; ++i)
      out16[i] = 0.0f;
    out16[0] = static_cast<float>(2.0 / (r - l));      // m00
    out16[5] = static_cast<float>(2.0 / (t - b));      // m11
    out16[10] = static_cast<float>(1.0 / (n - f));     // m22 (matches ortho)
    out16[12] = static_cast<float>(-(r + l) / (r - l));// m30 x-offset
    out16[13] = static_cast<float>(-(t + b) / (t - b));// m31 y-offset
    out16[14] = static_cast<float>(n / (n - f));       // m32 (matches ortho)
    out16[15] = 1.0f;                                  // m33: affine (w=1)
  }

  // out = A * B (both column-major 4x4, element [col*4+row]).
  void mul4(const float A[16], const float B[16], float out16[16]) {
    for (int col = 0; col < 4; ++col)
      for (int row = 0; row < 4; ++row) {
        float s = 0.0f;
        for (int k = 0; k < 4; ++k)
          s += A[k * 4 + row] * B[col * 4 + k];
        out16[col * 4 + row] = s;
      }
  }

  // out = M * v (column-major M, vec4 v).
  void mulVec4(const float M[16], const float v[4], float out4[4]) {
    for (int row = 0; row < 4; ++row) {
      float s = 0.0f;
      for (int k = 0; k < 4; ++k)
        s += M[k * 4 + row] * v[k];
      out4[row] = s;
    }
  }

  // General column-major 4x4 inverse (cofactor/adjugate, the MESA gluInvertMatrix
  // layout — already column-major, so our [col*4+row] flat array maps verbatim).
  // Returns false if singular. Needed to recover a camera sub-frustum's world-space
  // corners (invert its viewProj, transform the 8 NDC cube corners).
  bool invert4(const float m[16], float invOut[16]) {
    float inv[16];
    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
             m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] -
             m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
             m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] -
              m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] -
             m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
             m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
             m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
              m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
             m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
             m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
              m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
              m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
             m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
              m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
    float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    if (det == 0.0f)
      return false;
    det = 1.0f / det;
    for (int i = 0; i < 16; ++i)
      invOut[i] = inv[i] * det;
    return true;
  }

  // T1.2 CSM split scheme (PSSM / Zhang et al. "practical split scheme"): blend a
  // logarithmic and a uniform partition of [camNear,camFar] by splitLambda∈[0,1]
  // (1 = pure log → more resolution near the camera; 0 = pure uniform). Fills
  // outSplits[0..numCascades] with the boundary distances (outSplits[0]=camNear,
  // outSplits[numCascades]=camFar), the per-cascade depth ranges the fitting loop
  // and (later) the shader's cascade-selection use.
  void buildCascadeSplits(double camNear, double camFar, int numCascades,
                          double splitLambda, float *outSplits) {
    outSplits[0] = static_cast<float>(camNear);
    outSplits[numCascades] = static_cast<float>(camFar);
    const double ratio = camFar / camNear;
    for (int i = 1; i < numCascades; ++i) {
      const double p = static_cast<double>(i) / static_cast<double>(numCascades);
      const double cLog = camNear * std::pow(ratio, p);
      const double cUni = camNear + (camFar - camNear) * p;
      outSplits[i] = static_cast<float>(splitLambda * cLog + (1.0 - splitLambda) * cUni);
    }
  }

  // Row-major WbMatrix4 → column-major float[16] for WGSL upload. WbMatrix4's
  // toOpenGlMatrix is documented as the transpose to column-major. Verbatim
  // from WbCamera.cpp's wbMatrixToColumnMajorFloat.
  void wbMatrixToColumnMajorFloat(const WbMatrix4 &src, float out16[16]) {
    double tmp[16];
    src.toOpenGlMatrix(tmp);
    for (int i = 0; i < 16; ++i)
      out16[i] = static_cast<float>(tmp[i]);
  }

  // R4 3c-B un-gate FIX: stable texture-cache key. A scene PROTO instantiates a SEPARATE
  // WbImageTexture node per use, so dozens of shapes sharing one texture FILE (e.g. one Plaster.jpg
  // across many walls/doors) carry distinct WbImageTexture pointers. Keying the wgpu texture cache on
  // the pointer therefore re-uploaded the SAME file once per instance — the panda factory's ~30 unique
  // files became 500+ GPU uploads (mostly 2048²/1024²) totalling multiple GB → wgpu VRAM OOM, the
  // sustained main-view crash that blocked the 3c-B un-gate. Keying on the source file path collapses
  // shared files to ONE upload. Path-less (procedural / inline-data) textures keep the per-instance
  // pointer (there are few, and nothing to dedupe). Tagged in the high bits so a path hash can never
  // collide with a real pointer (which is ~0x00007ff7........).
  uint64_t stableTexId(WbImageTexture *map) {
    const QString p = map->path();
    if (!p.isEmpty())
      return (static_cast<uint64_t>(0x7e7e7e7eu) << 32) | static_cast<uint64_t>(qHash(p));
    return reinterpret_cast<uint64_t>(map);
  }

// TextureTransform -> 2D affine for the draw (uv' = M*uv + b). Derived by probing
// transformUVCoordinate at (0,0)/(1,0)/(0,1) so scale+rotation+center+translation all
// come out exactly as WREN computes them, with no field-math reimplementation. Grass
// {scale 40 40} on a bare Plane was sampled untransformed -> one tile smeared across
// the whole 200 m ground (the omni_quest white-streak bug).
static void fillUvTransform(WbAbstractAppearance *appearance, WbWgpuSolidDraw &draw) {
  WbTextureTransform *tt = appearance ? appearance->textureTransform() : nullptr;
  if (!tt)
    return;
  const WbVector2 b = tt->transformUVCoordinate(WbVector2(0.0, 0.0));
  const WbVector2 cu = tt->transformUVCoordinate(WbVector2(1.0, 0.0));
  const WbVector2 cv = tt->transformUVCoordinate(WbVector2(0.0, 1.0));
  draw.uvA[0] = static_cast<float>(cu.x() - b.x());
  draw.uvA[1] = static_cast<float>(cv.x() - b.x());
  draw.uvA[2] = static_cast<float>(cu.y() - b.y());
  draw.uvA[3] = static_cast<float>(cv.y() - b.y());
  draw.uvB[0] = static_cast<float>(b.x());
  draw.uvB[1] = static_cast<float>(b.y());
}

  // Recursively collect Shape draws under `root`. Verbatim from WbCamera.cpp's
  // collectShapeDraws (minus the one-shot first-draw diagnostic log, which is
  // Camera-specific). Keep in sync with WbCamera until it migrates here.
  void collectShapeDraws(WbBaseNode *root, WbWgpuMeshCache &cache,
                         std::vector<WbWgpuSolidDraw> &out,
                         std::vector<std::array<float, 16>> &modelStorage,
                         WbWgpuTextureCache *texCache,
                         std::vector<WbWgpuSceneRenderer::WbWgpuDrawRefresh> *outRefresh) {
    if (!root)
      return;
    // R4: URDF-imported robots (a mesh-visual arm) ship CadShape visuals (.glb/.dae imports → N submeshes),
    // NOT WbShape — collectShapeDraws only handled WbShape, so the whole robot was skipped (the "draws=3
    // / dark scene" bug). Collect each submesh: its WREN mesh + world transform (parents included, via
    // wr_transform_get_matrix) + its PBRAppearance.
    if (WbCadShape *cad = dynamic_cast<WbCadShape *>(root)) {
      const int nm = cad->wgpuMeshCount();
      for (int i = 0; i < nm; ++i) {
        WrStaticMesh *wm = cad->wgpuMesh(i);
        WrTransform *wt = cad->wgpuTransform(i);
        if (!wm || !wt)
          continue;
        WbWgpuMeshHandle h = WbWgpuMeshAdapter::acquireFromWren(cache, wm);
        if (!h.vertexBuffer || !h.indexBuffer)
          continue;  // garbage/not-yet-uploaded readback → skip, retried next frame
        const float *wm16 = wr_transform_get_matrix(wt);  // column-major world matrix
        if (!wm16)
          continue;
        std::array<float, 16> model;
        bool degenerate = false;
        for (int k = 0; k < 16; ++k) {
          model[k] = wm16[k];
          if (!std::isfinite(wm16[k]))
            degenerate = true;
        }
        if (degenerate || std::fabs(model[12]) > 1e4 || std::fabs(model[13]) > 1e4 ||
            std::fabs(model[14]) > 1e4)
          continue;
        modelStorage.push_back(model);
        WbWgpuSolidDraw draw;
        draw.modelMatrix16 = modelStorage.back().data();
        WbPbrAppearance *cpbr = cad->wgpuAppearance(i);
        const WbRgb color = cpbr ? cpbr->baseColor() : WbRgb(0.7, 0.7, 0.7);
        draw.baseColorR = static_cast<float>(color.red());
        draw.baseColorG = static_cast<float>(color.green());
        draw.baseColorB = static_cast<float>(color.blue());
        draw.baseColorA = 1.0f;
        if (cpbr) {
          const double sm = 1.0 - cpbr->roughness();
          draw.specularStrength = static_cast<float>(sm < 0.0 ? 0.0 : (sm > 1.0 ? 1.0 : sm));
        }
        draw.vertexBuffer = h.vertexBuffer;
        draw.indexBuffer = h.indexBuffer;
        draw.indexCount = h.indexCount;
        draw.castShadows = cad->wgpuCastShadows();
        fillUvTransform(cpbr, draw);
        if (texCache && cpbr) {
          if (WbImageTexture *map = cpbr->baseColorMap()) {
            const QImage *img = map->image();
            if (img) {
              const uint64_t texId = stableTexId(map);
              WbWgpuTextureHandle th = WbWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
              draw.textureView = th.view;
            }
          }
        }
        out.push_back(draw);
        if (outRefresh) {
          WbWgpuSceneRenderer::WbWgpuDrawRefresh r;
          r.wrenT = wt;
          r.node = cad;
          outRefresh->push_back(r);
        }
      }
      return;  // a CadShape is a leaf — nothing else under it
    }
    WbShape *shape = dynamic_cast<WbShape *>(root);
    if (shape) {
      WbGeometry *geom = shape->geometry();
      WbPbrAppearance *pbr = shape->pbrAppearance();
      // URDF-imported robots ship a plain Appearance (Material/diffuseColor), not a
      // PBRAppearance — fall back to it when pbr is null, else every such shape is skipped and the
      // robot never renders (the "draws=3 / dark scene" bug).
      WbAppearance *app = pbr ? nullptr : shape->appearance();
      if (geom && (pbr || app)) {
        WrStaticMesh *wmesh = geom->wrenMesh();
        if (wmesh) {
          using PrimitiveKind = WbWgpuMeshAdapter::PrimitiveKind;
          const uint64_t meshId = reinterpret_cast<uint64_t>(wmesh);
          WbWgpuMeshHandle h;
          double localScale[3] = {1.0, 1.0, 1.0};
          bool hasLocalScale = false;
          switch (geom->nodeType()) {
            case WB_NODE_BOX: {
              h = WbWgpuMeshAdapter::acquirePrimitive(cache, meshId, PrimitiveKind::Box);
              const WbVector3 &s = static_cast<WbBox *>(geom)->size();
              localScale[0] = s.x();
              localScale[1] = s.y();
              localScale[2] = s.z();
              hasLocalScale = true;
              break;
            }
            case WB_NODE_PLANE: {
              h = WbWgpuMeshAdapter::acquirePrimitive(cache, meshId, PrimitiveKind::Plane);
              const WbVector2 &s = static_cast<WbPlane *>(geom)->size();
              localScale[0] = s.x();
              localScale[1] = s.y();
              localScale[2] = 1.0;
              hasLocalScale = true;
              break;
            }
            case WB_NODE_SPHERE: {
              h = WbWgpuMeshAdapter::acquirePrimitive(cache, meshId, PrimitiveKind::UVSphere);
              const double r = static_cast<WbSphere *>(geom)->radius();
              localScale[0] = r;
              localScale[1] = r;
              localScale[2] = r;
              hasLocalScale = true;
              break;
            }
            case WB_NODE_CYLINDER: {
              h = WbWgpuMeshAdapter::acquirePrimitive(cache, meshId, PrimitiveKind::Cylinder);
              WbCylinder *c = static_cast<WbCylinder *>(geom);
              localScale[0] = c->radius();
              localScale[1] = c->radius();
              localScale[2] = c->height();
              hasLocalScale = true;
              break;
            }
            case WB_NODE_CAPSULE: {
              WbCapsule *c = static_cast<WbCapsule *>(geom);
              h = WbWgpuMeshAdapter::acquireCapsule(
                cache, meshId, static_cast<float>(c->radius()), static_cast<float>(c->height()), c->subdivision());
              break;
            }
            default:
              h = WbWgpuMeshAdapter::acquireFromWren(cache, wmesh);
              break;
          }
          if (h.vertexBuffer && h.indexBuffer) {
            WbWgpuSolidDraw draw;
            std::array<float, 16> model;
            WbMatrix4 modelMat = geom->matrix();
            if (hasLocalScale)
              modelMat.scale(localScale[0], localScale[1], localScale[2]);
            wbMatrixToColumnMajorFloat(modelMat, model.data());
            // R4 robustness: drop degenerate draws — non-finite or absurd-magnitude
            // transforms. Webots parks hidden/sentinel shapes ~1e5 m away (1 such draw
            // seen on panda); they render off-screen and only waste a draw call + a
            // pick-ID slot, so skipping them keeps the ID space tight and bounds sane.
            bool degenerate = false;
            for (int k = 0; k < 16; ++k)
              if (!std::isfinite(model[k]))
                degenerate = true;
            if (std::fabs(model[12]) > 1e4 || std::fabs(model[13]) > 1e4 ||
                std::fabs(model[14]) > 1e4)
              degenerate = true;
            if (degenerate)
              return;  // a Shape is a leaf; nothing else to collect under it
            modelStorage.push_back(model);
            draw.modelMatrix16 = modelStorage.back().data();
            // Base colour + material params from the PBRAppearance, else from a plain Appearance's
            // Material (diffuseColor) for URDF robots. emissive (T1.1 HDR for AgX) + smoothness
            // (1 - roughness) come from PBR; the plain-Appearance fallback uses a flat diffuse (no
            // emissive/spec) — enough to make the robot visible and correctly coloured.
            WbRgb color(0.8, 0.8, 0.8);
            double smooth = 0.0, emR = 0.0, emG = 0.0, emB = 0.0;
            if (pbr) {
              color = pbr->baseColor();
              const WbRgb emissive = pbr->emissiveColor();
              const double ei = pbr->emissiveIntensity();
              emR = emissive.red() * ei;
              emG = emissive.green() * ei;
              emB = emissive.blue() * ei;
              smooth = 1.0 - pbr->roughness();
            } else if (app) {
              if (WbMaterial *mat = app->material())
                color = mat->diffuseColor();
            }
            smooth = smooth < 0.0 ? 0.0 : (smooth > 1.0 ? 1.0 : smooth);
            draw.baseColorR = static_cast<float>(color.red());
            draw.baseColorG = static_cast<float>(color.green());
            draw.baseColorB = static_cast<float>(color.blue());
            draw.baseColorA = 1.0f;
            draw.emissiveR = static_cast<float>(emR);
            draw.emissiveG = static_cast<float>(emG);
            draw.emissiveB = static_cast<float>(emB);
            draw.specularStrength = static_cast<float>(smooth);
            draw.vertexBuffer = h.vertexBuffer;
            draw.indexBuffer = h.indexBuffer;
            draw.indexCount = h.indexCount;
            draw.castShadows = shape->isCastShadowsEnabled();
            fillUvTransform(pbr ? static_cast<WbAbstractAppearance *>(pbr)
                                : static_cast<WbAbstractAppearance *>(app),
                            draw);
            // R4 material fidelity: upload the PBRAppearance's baseColorMap (albedo) and
            // hand the texture view to the draw. Keyed on the WbImageTexture pointer so
            // repeat frames skip the re-upload. Null texCache / no map / plain Appearance → flat baseColor.
            if (texCache && pbr) {
              if (WbImageTexture *map = pbr->baseColorMap()) {
                const QImage *img = map->image();
                if (img) {
                  const uint64_t texId = stableTexId(map);
                  WbWgpuTextureHandle th =
                    WbWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
                  draw.textureView = th.view;
                }
              }
              // Per-pixel roughnessMap (modulates the specular highlight in the textured-lit
              // path). Only meaningful alongside an albedo map (the flat path ignores it);
              // null → the render target binds a default-white texture (no-op).
              if (WbImageTexture *rmap = pbr->roughnessMap()) {
                const QImage *img = rmap->image();
                if (img) {
                  const uint64_t texId = stableTexId(rmap);
                  WbWgpuTextureHandle th =
                    WbWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
                  draw.roughnessView = th.view;
                }
              }
              // metalnessMap (.r) — metals lose diffuse + tint specular by albedo. Null →
              // default-black (dielectric, no-op).
              if (WbImageTexture *mmap = pbr->metalnessMap()) {
                const QImage *img = mmap->image();
                if (img) {
                  const uint64_t texId = stableTexId(mmap);
                  WbWgpuTextureHandle th =
                    WbWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
                  draw.metalnessView = th.view;
                }
              }
              // normalMap (tangent-space) — perturbs the shading normal. Null → default-flat
              // (no perturbation, no-op).
              if (WbImageTexture *nmap = pbr->normalMap()) {
                const QImage *img = nmap->image();
                if (img) {
                  const uint64_t texId = stableTexId(nmap);
                  WbWgpuTextureHandle th =
                    WbWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
                  draw.normalView = th.view;
                }
              }
            }
            out.push_back(draw);
            if (outRefresh) {
              WbWgpuSceneRenderer::WbWgpuDrawRefresh r;
              r.geom = geom;
              r.node = shape;
              r.localScale[0] = static_cast<float>(localScale[0]);
              r.localScale[1] = static_cast<float>(localScale[1]);
              r.localScale[2] = static_cast<float>(localScale[2]);
              r.hasLocalScale = hasLocalScale;
              outRefresh->push_back(r);
            }
          }
        }
      }
    }
    WbGroup *group = dynamic_cast<WbGroup *>(root);
    if (group) {
      const int n = group->childCount();
      for (int i = 0; i < n; ++i)
        collectShapeDraws(group->child(i), cache, out, modelStorage, texCache, outRefresh);
    }
    // Articulated robots chain their links through JOINTS, not the children field: each subsequent link
    // is a WbBasicJoint's solidEndPoint(). The group recursion reaches the joint (it's a child of the
    // parent link) but a joint is not a WbGroup, so its endpoint was never descended — only the base
    // link rendered (the "draws=3" bug). Descend the endpoint here so the whole arm is collected.
    if (WbBasicJoint *joint = dynamic_cast<WbBasicJoint *>(root)) {
      if (WbSolid *ep = joint->solidEndPoint())
        collectShapeDraws(ep, cache, out, modelStorage, texCache, outRefresh);
    }
  }

}  // namespace

namespace WbWgpuSceneRenderer {

  bool ensureTarget(WbRenderBackend *back, int w, int h,
                    WbWgpuRenderTarget *&target, WbWgpuMeshCache *&cache,
                    int &targetW, int &targetH) {
    if (!back || back->kind() != WbRenderBackendKind::Vulkan || !back->isAvailable())
      return false;
    if (w <= 0 || h <= 0)
      return false;

    if (target && (targetW != w || targetH != h)) {
      delete cache;
      cache = nullptr;
      delete target;
      target = nullptr;
    }
    if (!target) {
      WbVulkanBackend *vb = static_cast<WbVulkanBackend *>(back);
      target = new WbWgpuRenderTarget(vb, static_cast<uint32_t>(w), static_cast<uint32_t>(h));
      targetW = w;
      targetH = h;
      if (!target->isUsable()) {
        delete target;
        target = nullptr;
        return false;
      }
      cache = new WbWgpuMeshCache(vb);
    }
    return true;
  }

  void collectWorldDraws(WbWgpuMeshCache &cache,
                         std::vector<WbWgpuSolidDraw> &out,
                         std::vector<std::array<float, 16>> &modelStorage,
                         std::vector<WbSolid *> *outNodes, WbWgpuTextureCache *texCache,
                         std::vector<WbWgpuDrawRefresh> *outRefresh) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return;
    const QList<WbSolid *> &tops = world->topSolids();
    for (WbSolid *s : tops) {
      const size_t before = out.size();
      collectShapeDraws(s, cache, out, modelStorage, texCache, outRefresh);
      if (outNodes)
        for (size_t i = before; i < out.size(); ++i)
          outNodes->push_back(s);  // every draw under top solid s maps back to s
    }
    // ROOT-CAUSE FIX (use-after-realloc): collectShapeDraws sets each `draw.modelMatrix16` to alias a
    // `modelStorage` entry, but `modelStorage` is a std::vector — every push_back that reallocated it
    // DANGLED all earlier pointers, so a draw's transform was read from freed/reused memory =
    // intermittent garbage. RenderDoc traced this exactly: the floor draw's clip coords came out
    // astronomical → the primitive was DEPTH-CLIPPED → the whole floor dropped on ~50% of runs under the
    // heavy two-pass cast-shadow render; the lighter single-pass path mostly survived because the freed
    // memory had not been reused yet. `modelStorage` is now final-sized (no more push_back), so re-point
    // every draw to its STABLE slot. Draws and models are pushed 1:1, so the indices line up.
    if (out.size() == modelStorage.size())
      for (size_t k = 0; k < out.size(); ++k)
        out[k].modelMatrix16 = modelStorage[k].data();
  }

  bool refreshWorldDraws(std::vector<std::array<float, 16>> &modelStorage,
                         const std::vector<WbWgpuDrawRefresh> &refresh) {
    if (modelStorage.size() != refresh.size())
      return false;
    for (size_t k = 0; k < refresh.size(); ++k) {
      const WbWgpuDrawRefresh &r = refresh[k];
      std::array<float, 16> model;
      if (r.geom) {
        WbMatrix4 m = r.geom->matrix();
        if (r.hasLocalScale)
          m.scale(r.localScale[0], r.localScale[1], r.localScale[2]);
        wbMatrixToColumnMajorFloat(m, model.data());
      } else if (r.wrenT) {
        const float *wm16 = wr_transform_get_matrix(static_cast<WrTransform *>(r.wrenT));
        if (!wm16)
          continue;  // keep the previous matrix
        for (int i = 0; i < 16; ++i)
          model[i] = wm16[i];
      } else
        continue;
      // Same degenerate guard as collection: a non-finite/parked transform keeps its old matrix
      // rather than poisoning the draw (the sentinel shapes Webots parks at ~1e5 m).
      bool degenerate = false;
      for (int i = 0; i < 16; ++i)
        if (!std::isfinite(model[i]))
          degenerate = true;
      if (degenerate || std::fabs(model[12]) > 1e4 || std::fabs(model[13]) > 1e4 ||
          std::fabs(model[14]) > 1e4)
        continue;
      modelStorage[k] = model;
    }
    return true;
  }

  void buildView(const WbMatrix4 &cameraWorld, float outView16[16]) {
    // kBasisSwap rotates the Webots camera-local frame (+X forward, +Y left,
    // +Z up) into the convention the wgpu/OpenGL projection expects (-Z
    // forward, +Y up). Column-major. Identical to WbCamera's inline matrix.
    static const float kBasisSwap[16] = {
       0,  0, -1, 0,  // col 0 (image of camera.x)
      -1,  0,  0, 0,  // col 1 (image of camera.y)
       0,  1,  0, 0,  // col 2 (image of camera.z)
       0,  0,  0, 1,  // col 3 (translation)
    };
    WbMatrix4 camWorldInv = cameraWorld;
    if (!camWorldInv.inverse())
      camWorldInv = WbMatrix4();
    float camInv[16] = {0};
    wbMatrixToColumnMajorFloat(camWorldInv, camInv);
    // view = kBasisSwap * camInv (both column-major).
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        float s = 0.0f;
        for (int k = 0; k < 4; ++k)
          s += kBasisSwap[k * 4 + row] * camInv[col * 4 + k];
        outView16[col * 4 + row] = s;
      }
    }
  }

  void buildViewProj(const WbMatrix4 &cameraWorld, double horizFov, double aspect,
                     double zNear, double zFar, float outViewProj16[16], bool reversedZ) {
    float view[16] = {0}, proj[16] = {0};
    buildView(cameraWorld, view);
    // Horizontal FOV → vertical via aspect (WbWrenCamera::computeFieldOfViewY).
    const double vertFov = 2.0 * std::atan(std::tan(horizFov * 0.5) / aspect);
    perspective(vertFov, aspect, zNear, zFar, proj);
    // viewProj = proj * view (column-major).
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        float s = 0.0f;
        for (int k = 0; k < 4; ++k)
          s += proj[k * 4 + row] * view[col * 4 + k];
        outViewProj16[col * 4 + row] = s;
      }
    }
    if (reversedZ)
      for (int c = 0; c < 4; ++c)
        outViewProj16[c * 4 + 2] = outViewProj16[c * 4 + 3] - outViewProj16[c * 4 + 2];

  }

  void buildOrthoLightViewProj(const WbMatrix4 &lightWorld, double halfExtent,
                               double zNear, double zFar, float outViewProj16[16]) {
    float view[16] = {0}, proj[16] = {0};
    buildView(lightWorld, view);  // same basis swap as the camera path
    ortho(halfExtent, zNear, zFar, proj);
    // viewProj = proj * view (column-major), identical composition to buildViewProj.
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        float s = 0.0f;
        for (int k = 0; k < 4; ++k)
          s += proj[k * 4 + row] * view[col * 4 + k];
        outViewProj16[col * 4 + row] = s;
      }
    }
  }

  void buildCascadeLightViewProjs(const WbMatrix4 &cameraWorld, double horizFov,
                                  double aspect, double camNear, double camFar,
                                  const WbMatrix4 &lightWorld, int numCascades,
                                  double splitLambda, float *outLightViewProjs,
                                  float *outSplits) {
    if (numCascades < 1)
      numCascades = 1;
    if (numCascades > kMaxCascades)
      numCascades = kMaxCascades;
    buildCascadeSplits(camNear, camFar, numCascades, splitLambda, outSplits);

    float lightView[16];
    buildView(lightWorld, lightView);  // basis-swapped light view (looks down -Z)
    float camView[16];
    buildView(cameraWorld, camView);
    const double vertFov = 2.0 * std::atan(std::tan(horizFov * 0.5) / aspect);

    for (int c = 0; c < numCascades; ++c) {
      const double nI = outSplits[c];
      const double fI = outSplits[c + 1];

      // The camera SUB-frustum [nI,fI]: its viewProj, inverted, maps the 8 NDC cube
      // corners back to world space — the exact slice of the camera frustum this
      // cascade must cover. Reuses perspective()+buildView() so the corners are
      // guaranteed consistent with the live camera path.
      float camProj[16], camVP[16], camVPinv[16];
      perspective(vertFov, aspect, nI, fI, camProj);
      mul4(camProj, camView, camVP);
      if (!invert4(camVP, camVPinv)) {
        // Degenerate viewProj — emit a valid (loose) symmetric fallback so the
        // cascade entry is never garbage.
        buildOrthoLightViewProj(lightWorld, std::max(fI, 1.0), 1e-3, fI * 2.0 + 1.0,
                                &outLightViewProjs[c * 16]);
        continue;
      }

      // Transform the 8 corners (wgpu NDC: x,y∈[-1,1], z∈[0,1]) world→light-view and
      // take the AABB in light-view space — the tight bounds of this cascade.
      double lxMin = 1e30, lxMax = -1e30, lyMin = 1e30, lyMax = -1e30, lzMin = 1e30, lzMax = -1e30;
      for (int ci = 0; ci < 8; ++ci) {
        const float ndc[4] = {
          (ci & 1) ? 1.0f : -1.0f,
          (ci & 2) ? 1.0f : -1.0f,
          (ci & 4) ? 1.0f : 0.0f,
          1.0f,
        };
        float world[4];
        mulVec4(camVPinv, ndc, world);
        const float invW = (world[3] != 0.0f) ? (1.0f / world[3]) : 1.0f;
        const float wpt[4] = {world[0] * invW, world[1] * invW, world[2] * invW, 1.0f};
        float lv[4];
        mulVec4(lightView, wpt, lv);
        lxMin = std::min(lxMin, static_cast<double>(lv[0]));
        lxMax = std::max(lxMax, static_cast<double>(lv[0]));
        lyMin = std::min(lyMin, static_cast<double>(lv[1]));
        lyMax = std::max(lyMax, static_cast<double>(lv[1]));
        lzMin = std::min(lzMin, static_cast<double>(lv[2]));
        lzMax = std::max(lzMax, static_cast<double>(lv[2]));
      }

      // Square the cascade about its AABB centre so a texel covers a constant
      // world area regardless of light angle (the standard CSM choice; texel-grid
      // snapping for shimmer-free edges is a later refinement).
      const double cx = 0.5 * (lxMin + lxMax);
      const double cy = 0.5 * (lyMin + lyMax);
      double h = 0.5 * std::max(lxMax - lxMin, lyMax - lyMin);
      if (h < 1e-4)
        h = 1e-4;

      // Light looks down -Z, so light-view z is negative in front: the nearest corner
      // is lzMax (least-negative), the farthest lzMin. ortho's (zNear,zFar) are signed
      // view-space planes (-zNear→NDC 0, -zFar→NDC 1). Pull the near plane toward — and
      // one slice-depth PAST — the light so casters standing in front of the cascade
      // still render into its depth map. zNear is deliberately NOT clamped positive: a
      // near plane behind the light origin is a valid ortho clip plane, and clamping it
      // collapses the depth range whenever a corner sits at/behind the light (z >= 0).
      const double depth = lzMax - lzMin;
      double zNear = -lzMax - depth;
      double zFar = -lzMin + 1e-3;
      if (zFar <= zNear)
        zFar = zNear + 1e-3;

      float lightProj[16];
      orthoOffCenter(cx - h, cx + h, cy - h, cy + h, zNear, zFar, lightProj);
      mul4(lightProj, lightView, &outLightViewProjs[c * 16]);
    }
  }

  // T1.2 CSM (multi-cascade) headless self-test orchestrator. See the header. Builds the
  // GPU-proven prototype camera/light frames (csm_render_prototype.py: cam (0,-18,14)→(0,4,0),
  // light (6,6,20)→origin) + the N=3 cascade fit, then drives the render-layer
  // WbWgpuRenderTarget::selfTestCsm with the raw matrices.
  bool csmSelfTest(WbWgpuRenderTarget &rt, unsigned char shadowedOut[3], unsigned char litSideOut[3],
                   unsigned char shadowOffOut[3], int *cascadeSelected) {
    // lookat (Webots convention: local +X forward, +Z up). Row-major WbMatrix4 so the engine's
    // buildView (kBasisSwap * columnMajor(inverse(world))) reproduces the prototype's frames.
    auto lookat = [](const double eye[3], const double tgt[3]) -> WbMatrix4 {
      double x[3] = {tgt[0] - eye[0], tgt[1] - eye[1], tgt[2] - eye[2]};
      const double lx = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
      for (int k = 0; k < 3; ++k)
        x[k] /= lx;
      const double up[3] = {0, 0, 1};
      double y[3] = {up[1] * x[2] - up[2] * x[1], up[2] * x[0] - up[0] * x[2],
                     up[0] * x[1] - up[1] * x[0]};  // up × x
      const double ly = std::sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
      for (int k = 0; k < 3; ++k)
        y[k] /= ly;
      const double z[3] = {x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2],
                           x[0] * y[1] - x[1] * y[0]};  // x × y
      return WbMatrix4(x[0], y[0], z[0], eye[0], x[1], y[1], z[1], eye[1], x[2], y[2], z[2], eye[2],
                       0, 0, 0, 1);
    };
    const double camEye[3] = {0, -18, 14}, camTgt[3] = {0, 4, 0};
    const double litEye[3] = {6, 6, 20}, litTgt[3] = {0, 0, 0};
    const WbMatrix4 camWorld = lookat(camEye, camTgt);
    const WbMatrix4 lightWorld = lookat(litEye, litTgt);

    const int NC = 3;
    const double HFOV = 1.0, ASPECT = 1.0, CN = 0.5, CF = 60.0, LAMBDA = 0.6;  // square RT → aspect 1
    float camVP[16] = {0};
    buildViewProj(camWorld, HFOV, ASPECT, CN, CF, camVP);
    float lvps[kMaxCascades * 16] = {0};
    float splits[kMaxCascades + 1] = {0};
    buildCascadeLightViewProjs(camWorld, HFOV, ASPECT, CN, CF, lightWorld, NC, LAMBDA, lvps, splits);
    float splitsFar4[4] = {0, 0, 0, 0};  // far view-depth boundaries of cascades 0..NC-1
    for (int c = 0; c < NC && c < 4; ++c)
      splitsFar4[c] = splits[c + 1];

    // Light dir = light local +X (toward-target axis); the shader normalizes. w = ambient 0.25.
    const float lightDirAmbient[4] = {static_cast<float>(litTgt[0] - litEye[0]),
                                      static_cast<float>(litTgt[1] - litEye[1]),
                                      static_cast<float>(litTgt[2] - litEye[2]), 0.25f};

    return rt.selfTestCsm(camVP, lvps, splitsFar4, static_cast<uint32_t>(NC), lightDirAmbient,
                          shadowedOut, litSideOut, shadowOffOut, cascadeSelected);
  }

  // T1.4 TAA sub-pixel jitter. See the header. Halton(2,3) radical-inverse, 8-frame sequence.
  void haltonJitter(int frameIndex, double amplitudePx, float outOffsetPx2[2]) {
    auto halton = [](int i, int base) -> double {
      double f = 1.0, r = 0.0;
      while (i > 0) {
        f /= base;
        r += f * (i % base);
        i /= base;
      }
      return r;  // radical inverse in [0,1)
    };
    const int ji = (frameIndex % 8) + 1;  // 1-based index into the 8-frame Halton(2,3) sequence
    outOffsetPx2[0] = static_cast<float>((halton(ji, 2) - 0.5) * 2.0 * amplitudePx);
    outOffsetPx2[1] = static_cast<float>((halton(ji, 3) - 0.5) * 2.0 * amplitudePx);
  }

  // Apply a pixel jitter to a column-major view-projection as a depth-independent clip shift: add
  // (2*dx/width)*row3 to row0 and (-2*dy/height)*row3 to row1, so after the perspective divide every
  // fragment shifts by exactly (dx, dy) pixels regardless of depth (+y px = down → NDC.y up negated).
  // Column-major element index = col*4 + row.
  void jitterViewProj(float viewProj16[16], const float offsetPx2[2], double width, double height) {
    const double clipDX = 2.0 * static_cast<double>(offsetPx2[0]) / width;
    const double clipDY = -2.0 * static_cast<double>(offsetPx2[1]) / height;
    for (int col = 0; col < 4; ++col) {
      const float w = viewProj16[col * 4 + 3];                    // row3 (clip.w contribution)
      viewProj16[col * 4 + 0] += static_cast<float>(clipDX) * w;  // row0 (clip.x)
      viewProj16[col * 4 + 1] += static_cast<float>(clipDY) * w;  // row1 (clip.y)
    }
  }

}  // namespace WbWgpuSceneRenderer
