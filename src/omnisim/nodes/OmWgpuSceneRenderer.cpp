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

#include "OmWgpuSceneRenderer.hpp"

#include "OmBox.hpp"
#include "OmBackground.hpp"
#include "OmBasicJoint.hpp"
#include "OmCadShape.hpp"
#include "OmCloth.hpp"
#include "OmCapsule.hpp"
#include "OmCone.hpp"  // P3 residue: the Pen-atlas deviation warning names Box/Cylinder/Cone
#include "OmCylinder.hpp"
#include "OmElevationGrid.hpp"  // W1d: native ElevationGrid mesh (post-D1.4 there is no readback to fall to)
#include "OmMFDouble.hpp"       // W1d: the height field's true size (heights pad with zeros)
#include "OmSFInt.hpp"          // W1d: Cone subdivision via findSFInt (no public accessor)
#include "OmGranularGroup.hpp"  // P9: the particle registry + host position buffer
#include "OmGroup.hpp"
#include "OmImageTexture.hpp"
#include "OmLight.hpp"
#include "OmLog.hpp"
#include "OmMatrix4.hpp"
#include "OmMesh.hpp"  // F2.5: file-loaded Mesh joined the Pen-atlas deviation class
#include "OmMuscle.hpp"  // P2: the muscle registry + its procedural vertex stream
#include "OmAppearance.hpp"
#include "OmMaterial.hpp"
#include "OmPaintTexture.hpp"
#include "OmPbrAppearance.hpp"
#include "OmPlane.hpp"
#include "OmRenderBackend.hpp"
#include "OmRgb.hpp"
#include "OmSFBool.hpp"  // W1c: side/top/bottom read for primitiveHasNoSurface()
#include "OmShape.hpp"
#include "OmTextureTransform.hpp"
#include "OmTrack.hpp"  // P2: the belt registry + this step's belt placements
#include "OmTriangleMesh.hpp"
#include "OmTriangleMeshGeometry.hpp"
#include "OmSoftBody.hpp"
#include "OmSolid.hpp"
#include "OmSphere.hpp"
#include "OmViewpoint.hpp"  // getInvisibleNodes() — main-view hidden set
#include "OmVulkanBackend.hpp"
#include "OmWgpuImageAdapter.hpp"
#include "OmWgpuMeshAdapter.hpp"
#include "OmWgpuMeshCache.hpp"
#include "OmWgpuRenderTarget.hpp"
#include "OmWgpuTextureCache.hpp"
#include "OmWorld.hpp"
// W1c: the collect owns its own GL arming now (see OmWrenGlArm below) instead of
// relying on a caller-side makeWrenCurrent()/doneWren() bracket.
#include "OmDeformableFrameListener.hpp"  // the Cloth/SoftBody subscription registry
#include "OmSimulationState.hpp"  // sim clock — the "has the surface moved?" test

// wr_transform_get_matrix — CadShape submesh world matrices, hatch-off fallback only since W1b
// (OMNISIM_WGPU_NATIVE_CADSHAPE=0). The default path takes the matrix from OmCadShape's own pose.

#include <QtCore/QFileInfo>  // P2: the Muscle texture path -> one stable cache key
#include <QtGui/QImage>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {

  // Cumulative GL arms taken by the collect; published by glArmCount().
  unsigned long long gGlArms = 0;

  // Column-major (WGSL / OpenGL) perspective from a vertical FOV (radians),
  // aspect, near/far. Right-handed, depth → [0, 1] (wgpu NDC). Verbatim from
  // OmCamera.cpp's wgpuPerspective.
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

  // Row-major OmMatrix4 → column-major float[16] for WGSL upload. OmMatrix4's
  // toOpenGlMatrix is documented as the transpose to column-major. Verbatim
  // from OmCamera.cpp's wbMatrixToColumnMajorFloat.
  void wbMatrixToColumnMajorFloat(const OmMatrix4 &src, float out16[16]) {
    double tmp[16];
    src.toOpenGlMatrix(tmp);
    for (int i = 0; i < 16; ++i)
      out16[i] = static_cast<float>(tmp[i]);
  }

  // R4 3c-B un-gate FIX: stable texture-cache key. A scene PROTO instantiates a SEPARATE
  // OmImageTexture node per use, so dozens of shapes sharing one texture FILE (e.g. one Plaster.jpg
  // across many walls/doors) carry distinct OmImageTexture pointers. Keying the wgpu texture cache on
  // the pointer therefore re-uploaded the SAME file once per instance — the panda factory's ~30 unique
  // files became 500+ GPU uploads (mostly 2048²/1024²) totalling multiple GB → wgpu VRAM OOM, the
  // sustained main-view crash that blocked the 3c-B un-gate. Keying on the source file path collapses
  // shared files to ONE upload. Path-less (procedural / inline-data) textures keep the per-instance
  // pointer (there are few, and nothing to dedupe). Tagged in the high bits so a path hash can never
  // collide with a real pointer (which is ~0x00007ff7........).
  uint64_t stableTexId(OmImageTexture *map) {
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
static void fillUvTransform(OmAbstractAppearance *appearance, OmWgpuSolidDraw &draw) {
  OmTextureTransform *tt = appearance ? appearance->textureTransform() : nullptr;
  if (!tt)
    return;
  const OmVector2 b = tt->transformUVCoordinate(OmVector2(0.0, 0.0));
  const OmVector2 cu = tt->transformUVCoordinate(OmVector2(1.0, 0.0));
  const OmVector2 cv = tt->transformUVCoordinate(OmVector2(0.0, 1.0));
  draw.uvA[0] = static_cast<float>(cu.x() - b.x());
  draw.uvA[1] = static_cast<float>(cv.x() - b.x());
  draw.uvA[2] = static_cast<float>(cu.y() - b.y());
  draw.uvA[3] = static_cast<float>(cv.y() - b.y());
  draw.uvB[0] = static_cast<float>(b.x());
  draw.uvB[1] = static_cast<float>(b.y());
}

// ===================== P4: WREN's ambient model, ported =====================
//
// WREN has TWO ambient models and selects between them BY APPEARANCE TYPE. wgpu had one
// analytic hemisphere for every surface, which is the single common cause of three separate
// symptoms: tests/api/worlds/pen.omniworld's white board reading sky-blue (85,141,187) where
// WREN reads a neutral (207,207,207), and `Background.luminosity` (7 worlds) +
// `PBRAppearance.IBLStrength` (81 files) being completely inert.
//
//   legacy Appearance + Material -> phong.frag:82,181,214
//       ambient = Lights.ambientLight x material.ambient
//       Lights.ambientLight = SUM over ON lights of (ambientIntensity x color)   [OmLight.cpp]
//       material.ambient    = (ai,ai,ai)          when the Appearance has a texture
//                           = ai x diffuseColor   otherwise                      [OmMaterial.cpp]
//       ...and the result is multiplied by the TEXTURE (not by diffuseColor, which WREN forces
//       to white on a textured material). It NEVER reads Background.skyColor.
//
//   PBRAppearance -> pbr.frag:296-321
//       ambient = skyColor x (diffuseColor + specularEnvBRDF) x (luminosity x IBLStrength)
//       with skyColor the FLAT Background.skyColor whenever there is no irradiance cubemap
//       (pbr.frag:300 -- which is every world in this tree, see P4). It never reads the lights'
//       ambientIntensity.
//
// So `luminosity` and `IBLStrength` are ONE premultiplied float in WREN
// (OmPbrAppearance.cpp:466 -> material.backgroundColorAndIblStrength.w) applied to BOTH the
// diffuse and the specular ambient, and they get identical semantics here by construction.
//
// DELIBERATE DEVIATION, and the reason is measured rather than aesthetic: the phong arm is taken
// only when the world actually declares a non-zero light ambientIntensity. Every light type
// defaults it to 0, so WREN's phong ambient is EXACTLY ZERO on a modern world -- a plain
// `Appearance` (which is what every URDF-imported robot ships) would go pure black on its
// shadowed side in the main view. Where WREN has nothing to reproduce, wgpu keeps its own
// hemisphere fill; where WREN has a value, wgpu reproduces it exactly.
//
// Hatch: OMNISIM_WGPU_WREN_AMBIENT=0 (value-parsed) leaves every draw at mode 0 / scale 1, which
// is byte-identical to the pre-P4 renderer. Read ONCE, in ONE place, by the collector both the
// main view and the Camera device share -- so the two call sites cannot drift.
static bool wgpuWrenAmbientEnabled() {
  static const bool sEnabled = !qEnvironmentVariableIsSet("OMNISIM_WGPU_WREN_AMBIENT") ||
                               qEnvironmentVariableIntValue("OMNISIM_WGPU_WREN_AMBIENT") != 0;
  return sEnabled;
}

// Per-collect snapshot of the two scene-global inputs (refreshed at the top of
// collectWorldDraws, once, not once per draw).
struct OmWgpuAmbientEnv {
  float sceneAmbient[3] = {0.0f, 0.0f, 0.0f};  // WREN's Lights.ambientLight
  bool hasSceneAmbient = false;
  double luminosity = 1.0;  // Background.luminosity
};
static OmWgpuAmbientEnv sAmbientEnv;

static void refreshWrenAmbientEnv() {
  sAmbientEnv = OmWgpuAmbientEnv();
  if (!wgpuWrenAmbientEnabled())
    return;
  OmLight::sceneAmbientLight(sAmbientEnv.sceneAmbient);
  sAmbientEnv.hasSceneAmbient = sAmbientEnv.sceneAmbient[0] > 0.0f ||
                                sAmbientEnv.sceneAmbient[1] > 0.0f ||
                                sAmbientEnv.sceneAmbient[2] > 0.0f;
  if (OmBackground *const bg = OmBackground::firstInstance())
    sAmbientEnv.luminosity = bg->luminosity();
}

// ============ P11: a legacy `Appearance`'s TEXTURE, on the wgpu path ============
//
// Every texture-upload block in this file used to sit inside an `if (pbr)` guard, and a
// legacy `Appearance` has no baseColorMap -- so its image never reached the GPU and the
// surface rendered as a flat, LIT `Material.diffuseColor` where WREN shows the picture.
// No new pipeline is needed: there is ONE scene pipeline, `textureView` is bind-group
// entry @1 with a white default, so such a draw was already ON the textured pipeline and
// merely sampling white.
//
// TWO halves, and shipping only the first renders `track` brown-tinted and
// `elevation_grid_rotation` pink-tinted:
//   1. upload `app->texture()->image()` into draw.textureView, and
//   2. FORCE baseColor to white, because WREN forces a textured Material's diffuseColor to
//      white (OmMaterial.cpp:151-152) and zeroes its specular and shininess with it. The
//      wgpu shader computes `albedo = texture * baseColor`, so without the forcing the
//      authored diffuseColor tints the whole picture -- twice, in fact, since P4 already
//      folded it into the CPU-side ambient product.
//
// Hatch: OMNISIM_WGPU_LEGACY_TEXTURE=0 (VALUE-PARSED -- unset or anything but
// "0"/"false"/"off"/"no" is ON) restores the pre-P11 renderer EXACTLY, including the
// ambient predicate below. Read once per process.
bool wgpuLegacyTextureEnabled() {
  static const bool sEnabled = []() {
    if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_LEGACY_TEXTURE"))
      return true;
    const QString v = qEnvironmentVariable("OMNISIM_WGPU_LEGACY_TEXTURE").trimmed().toLower();
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return sEnabled;
}

// THE ONE PREDICATE, and it exists because there were two.
//
// ⚠ A LIVE LATENT BUG FIXED HERE. fillWrenAmbient branched on `app->texture() != NULL` --
// a texture NODE being DECLARED -- while WREN branches on `texture()->wrenTexture()`, a
// texture actually being LOADED. The tree carries 9 empty `ImageTexture { }` nodes (Display
// targets), which are declared and never load, and for those the old predicate gave them
// WREN's TEXTURED ambient `(ai,ai,ai)` against an UNTEXTURED `diffuseColor` albedo -- a
// combination neither renderer produces. One predicate now drives the white-forcing, the
// upload and the ambient together, so they cannot disagree again.
//
// It is deliberately WREN-free: `image()` is the CPU pixel buffer OmImageTexture loads
// *because wgpu needs it* (OmImageTexture.cpp:332-334) and is set and cleared in lockstep
// with mWrenTexture, so it means the same thing without asking WREN anything.
const QImage *legacyAppearanceImage(OmAppearance *app) {
  if (!app)
    return nullptr;
  OmImageTexture *const tex = app->texture();
  if (!tex)
    return nullptr;
  const QImage *const img = tex->image();
  return (img && !img->isNull()) ? img : nullptr;
}

// Upload a legacy Appearance's texture onto `draw` and apply WREN's textured-Material
// rules. No-op for a PBRAppearance draw (the caller passes null), for an untextured or
// unloaded Appearance, and when the hatch is off.
//
// ⚠ ONE DELIBERATE DEVIATION, for the 3 tracked nodes that carry a `texture` and NO
// `Material`. WREN gives those `default.frag`, which is UNLIT -- it outputs the raw texel
// and never touches a light. wgpu has no unlit arm and adding one for three test nodes
// would be a new pipeline variant with its own parity surface, so those draws are LIT
// here. The image is right and the shading is not; that is a smaller and more visible
// error than the flat grey they render today, and it is written down rather than silent.
void applyLegacyAppearanceTexture(OmAppearance *app, OmWgpuTextureCache *texCache,
                                  OmWgpuSolidDraw &draw) {
  if (!texCache || !wgpuLegacyTextureEnabled())
    return;
  const QImage *const img = legacyAppearanceImage(app);
  if (!img)
    return;
  OmWgpuTextureHandle th =
    OmWgpuImageAdapter::acquireFromQImage(*texCache, stableTexId(app->texture()), *img);
  if (!th.view)
    return;
  draw.textureView = th.view;
  draw.texMeanLin[0] = th.meanLin[0];
  draw.texMeanLin[1] = th.meanLin[1];
  draw.texMeanLin[2] = th.meanLin[2];
  // WREN: diffuse = white, specular = black, shininess = 0 on a textured Material.
  // baseColorA (opacity, from `transparency`) is NOT touched -- WREN keeps it too.
  draw.baseColorR = 1.0f;
  draw.baseColorG = 1.0f;
  draw.baseColorB = 1.0f;
  draw.specularStrength = 0.0f;
}

static void fillWrenAmbient(OmPbrAppearance *pbr, OmAppearance *app, OmWgpuSolidDraw &draw) {
  if (!wgpuWrenAmbientEnabled())
    return;
  if (pbr) {
    // pbr.frag scales BOTH ambient terms by backgroundLuminosity * IBLStrength.
    draw.iblScale = static_cast<float>(sAmbientEnv.luminosity * pbr->iblStrength());
    return;
  }
  if (!app || !sAmbientEnv.hasSceneAmbient)
    return;  // nothing for WREN's phong arm to say -> keep the analytic hemisphere
  OmMaterial *const mat = app->material();
  if (!mat)
    return;
  const double ai = mat->ambientIntensity();
  // OmMaterial::modifyWrenMaterial's own `textured` test. WREN drops diffuseColor from BOTH
  // the ambient and the diffuse term the moment a texture is LOADED, so this must be the
  // SAME predicate that decides the albedo upload and the white-forcing over in
  // applyLegacyAppearanceTexture -- see legacyAppearanceImage() for the bug that being two
  // different predicates produced. Under OMNISIM_WGPU_LEGACY_TEXTURE=0 it falls back to the
  // pre-P11 "a texture node is DECLARED" test, so the hatch is an exact revert of the whole
  // change rather than half of it.
  const bool textured = wgpuLegacyTextureEnabled() ? (legacyAppearanceImage(app) != nullptr) :
                                                     (app->texture() != NULL);
  const OmRgb d = mat->diffuseColor();
  const double mr = textured ? ai : ai * d.red();
  const double mg = textured ? ai : ai * d.green();
  const double mb = textured ? ai : ai * d.blue();
  draw.ambientR = static_cast<float>(sAmbientEnv.sceneAmbient[0] * mr);
  draw.ambientG = static_cast<float>(sAmbientEnv.sceneAmbient[1] * mg);
  draw.ambientB = static_cast<float>(sAmbientEnv.sceneAmbient[2] * mb);
  draw.ambientMode = 1.0f;
}

  // OMNISIM_WGPU_PEN — the exact-revert hatch for W3/P3 (the Pen paint layer on the wgpu path).
  // VALUE-PARSED like its neighbours: unset or any non-zero value is ON, only "0" turns it off, so
  // `=0` can never accidentally mean ON. OFF leaves every draw's penView null, which binds the 1x1
  // transparent default and makes the frame byte-identical to the pre-P3 build.
  bool wgpuPenEnabled() {
    static const bool sEnabled = !qEnvironmentVariableIsSet("OMNISIM_WGPU_PEN") ||
                                 qEnvironmentVariableIntValue("OMNISIM_WGPU_PEN") != 0;
    return sEnabled;
  }

  // Resolve this Shape's Pen paint layer (if any) onto the draw.
  //
  // Upload-once + update-in-place, the texture-side twin of the Cloth/SoftBody vertex path: a
  // cache MISS acquires the layer at ONE mip level (a mutable texture must not carry a chain the
  // update would leave stale), and every later frame is a single updateRgba8, which itself no-ops
  // unless the revision moved. So an idle Pen costs one map lookup per draw per frame and zero
  // GPU traffic, and a world with NO Pen never reaches this function at all (hasAny() is false).
  void resolvePenTexture(const OmShape *shape, OmWgpuTextureCache *texCache, OmWgpuSolidDraw &draw) {
    OmPaintTexture *pt = OmPaintTexture::findPaintTexture(shape);
    if (!pt)
      return;
    // P3 residue — ACCEPTED DEVIATION, degraded LOUDLY: on Box / Cylinder / Cone the Pen paints
    // into the geometry's SECOND texture-coordinate set (the cross/sub-rect atlas that
    // pickUVCoordinate(uv, ray, 1) returns — see OmBox::computeTextureCoordinate's nonRecursive
    // arm), but the wgpu vertex stream carries only the FIRST UV set (pos3+norm3+uv2, stride 32),
    // so the ink layer is sampled with the wrong texels and paint appears displaced/scaled on
    // those geometries. Carrying the second set means a stride change through every mesh builder,
    // every pipeline's vertex layout and the WREN mesh readback — deferred until after the WREN
    // deletion (documented in docs/developer/wren-deletion-runbook.md, P3). Plane / Sphere /
    // Capsule / ElevationGrid paint correctly (their two sets coincide).
    // F2.5 (2026-08-23): file-loaded `Mesh` geometry is IN the deviation class too, refuting
    // this comment's previous "meshes paint exactly" claim — a Mesh carries its OWN authored
    // uv set-0, which need not coincide with the paint atlas computeTextureCoordinate derives,
    // and tests/api pen_mesh measured the result: ink flooded 3373 px where WREN painted <60.
    if (dynamic_cast<const OmBox *>(shape->geometry()) || dynamic_cast<const OmCylinder *>(shape->geometry()) ||
        dynamic_cast<const OmCone *>(shape->geometry()) || dynamic_cast<const OmMesh *>(shape->geometry())) {
      static bool sWarnedBoxAtlas = false;
      if (!sWarnedBoxAtlas) {
        sWarnedBoxAtlas = true;
        OmLog::warning(
          QString("[OmWgpu] a Pen painted onto a %1 under the wgpu renderer: this geometry's rendered UV set can "
                  "differ from the atlas the Pen paints into, and the wgpu vertex stream carries only one UV set, "
                  "so the ink can appear displaced, scaled or flooded (known deviation, see "
                  "wren-deletion-runbook.md P3). Paint onto a Plane, Sphere, Capsule or ElevationGrid for exact "
                  "placement.")
            .arg(shape->geometry()->nodeModelName()));
      }
    }
    const uint32_t w = static_cast<uint32_t>(pt->wgpuWidth());
    const uint32_t h = static_cast<uint32_t>(pt->wgpuHeight());
    if (w == 0 || h == 0)
      return;
    const uint64_t id = pt->wgpuTextureId();
    OmWgpuTextureHandle th;
    if (!texCache->tryGet(id, th)) {
      const std::vector<unsigned char> &px = pt->wgpuRgba8();
      if (px.size() < static_cast<size_t>(w) * h * 4u)
        return;
      th = texCache->acquire(id, w, h, px.data(), px.size(), /*mipLevels=*/1);
    } else {
      const std::vector<unsigned char> &px = pt->wgpuRgba8();
      if (px.size() >= static_cast<size_t>(w) * h * 4u)
        texCache->updateRgba8(id, w, h, px.data(), px.size(), pt->wgpuRevision());
    }
    draw.penView = th.view;
  }

  // OMNISIM_WGPU_NODE_VISIBILITY — the exact-revert hatch for wb_supervisor_node_set_visibility
  // on the wgpu path. VALUE-PARSED like the one above: unset or any non-zero value is ON, only
  // "0" turns it off, so `=0` can never accidentally mean ON. OFF restores the pre-change
  // behaviour exactly — every hidden set is ignored and every node is collected — even for a
  // caller that passes a non-null set. Read once per process.
  bool wgpuNodeVisibilityEnabled() {
    static const bool sEnabled = !qEnvironmentVariableIsSet("OMNISIM_WGPU_NODE_VISIBILITY") ||
                                 qEnvironmentVariableIntValue("OMNISIM_WGPU_NODE_VISIBILITY") != 0;
    return sEnabled;
  }

  // Is `node` hidden for the viewer whose set this is? Explicit linear scan rather than
  // QList::contains so the const-pointer comparison is unambiguous and the O(n) cost is
  // visible at the call site. The sets are tiny (a supervisor hides a handful of nodes at
  // most) and `hidden` is already null whenever the set is empty or the hatch is off, so the
  // walk's hot path is one null test per node.
  bool isHiddenForViewer(const QList<const OmBaseNode *> *hidden, const OmBaseNode *node) {
    if (!hidden || !node)
      return false;
    for (const OmBaseNode *n : *hidden)
      if (n == node)
        return true;
    return false;
  }

  // The SECOND thing the old `geom->wrenMesh()` gate was silently testing (W1c).
  //
  // OmCylinder::buildWrenMesh and OmCapsule::buildWrenMesh RETURN EARLY, leaving
  // wrenMesh() null, when side, top and bottom are all FALSE — a body with no surface
  // at all. The gate therefore dropped those Shapes. The wgpu codegen builders ignore
  // the face flags and always emit a CLOSED body, so without this test an author's
  // deliberately invisible cylinder would start rendering the moment the gate came out.
  //
  // Scoped to exactly the face-flagged primitives: Box, Plane and Sphere build their
  // mesh unconditionally, and tessellated geometry still reaches `default:`, which
  // skips a null triangle mesh itself. OmCone joined the list with W1d — it used to be
  // covered by the default branch's null-mesh read, but it now has its own case below,
  // so its side=FALSE bottom=FALSE early-out (OmCone::buildWrenMesh) must be tested
  // here or a deliberately invisible cone would start rendering. (A cone has no "top".)
  //
  // Read straight off the fields — none of these nodes exposes a side/top/bottom
  // accessor, and a const field read here is a far smaller blast radius than adding them.
  bool primitiveHasNoSurface(const OmGeometry *geom) {
    const int type = geom->nodeType();
    if (type == WB_NODE_CONE) {
      const OmSFBool *side = geom->findSFBool("side");
      const OmSFBool *bottom = geom->findSFBool("bottom");
      return side && bottom && side->isFalse() && bottom->isFalse();
    }
    if (type != WB_NODE_CYLINDER && type != WB_NODE_CAPSULE)
      return false;
    const OmSFBool *side = geom->findSFBool("side");
    const OmSFBool *top = geom->findSFBool("top");
    const OmSFBool *bottom = geom->findSFBool("bottom");
    // A field we cannot find is not evidence of invisibility — render, as before.
    return side && top && bottom && side->isFalse() && top->isFalse() && bottom->isFalse();
  }

  // ---- ONE geometry -> ONE wgpu mesh handle. Extracted from collectShapeDraws (P2). ----
  //
  // Verbatim, including every cache key, every hatch and every fallback -- a Shape's mesh
  // and a Track BELT ELEMENT's mesh must come from the same place, or the belt renders a
  // different tessellation of the same node than the scene does. That is the whole reason
  // this is an extraction and not a second implementation.
  //
  // `localScale` / `hasLocalScale` carry the primitive's authored size, which the caller
  // folds into its model matrix -- WREN carries it as the geometry's own scale WrTransform,
  // and a Track's innermost `wr_transform_copy(geom->wrenNode())` copies exactly that.
  //
  // Returns false when the geometry is not ready (bumping outSkipped, so the caller's
  // re-collect loop still sees an incomplete walk) or when the handle came back empty.
  bool acquireGeometryMesh(OmGeometry *geom, OmWgpuMeshCache &cache,
                           OmWgpuMeshHandle &h, double localScale[3], bool &hasLocalScale,
                           int *outSkipped) {
    localScale[0] = localScale[1] = localScale[2] = 1.0;
    hasLocalScale = false;
    if (!geom)
      return false;
    using PrimitiveKind = OmWgpuMeshAdapter::PrimitiveKind;
    const bool geomReady = geom->isPostFinalizedCalled() && !primitiveHasNoSurface(geom);
    if (!geomReady) {
      if (outSkipped)
        ++*outSkipped;  // not ready yet (Robot subtrees lag the first frame) -- incomplete
      return false;
    }
    auto acquireUnitPrimitive = [&](PrimitiveKind kind) {
      return OmWgpuMeshAdapter::acquirePrimitive(cache, kind);
    };
    switch (geom->nodeType()) {
      case WB_NODE_BOX: {
        h = acquireUnitPrimitive(PrimitiveKind::Box);
        const OmVector3 &s = static_cast<OmBox *>(geom)->size();
        localScale[0] = s.x();
        localScale[1] = s.y();
        localScale[2] = s.z();
        hasLocalScale = true;
        break;
      }
      case WB_NODE_PLANE: {
        h = acquireUnitPrimitive(PrimitiveKind::Plane);
        const OmVector2 &s = static_cast<OmPlane *>(geom)->size();
        localScale[0] = s.x();
        localScale[1] = s.y();
        localScale[2] = 1.0;
        hasLocalScale = true;
        break;
      }
      case WB_NODE_SPHERE: {
        h = acquireUnitPrimitive(PrimitiveKind::UVSphere);
        const double r = static_cast<OmSphere *>(geom)->radius();
        localScale[0] = r;
        localScale[1] = r;
        localScale[2] = r;
        hasLocalScale = true;
        break;
      }
      case WB_NODE_CYLINDER: {
        h = acquireUnitPrimitive(PrimitiveKind::Cylinder);
        OmCylinder *c = static_cast<OmCylinder *>(geom);
        localScale[0] = c->radius();
        localScale[1] = c->radius();
        localScale[2] = c->height();
        hasLocalScale = true;
        break;
      }
      case WB_NODE_CAPSULE: {
        // Content-keyed on (radius, height, subdivision) — the three values
        // buildCapsule bakes into the vertices. Two capsules with the same
        // dimensions now share one buffer pair instead of one per node.
        OmCapsule *c = static_cast<OmCapsule *>(geom);
        const float cr = static_cast<float>(c->radius());
        const float ch = static_cast<float>(c->height());
        const int cs = c->subdivision();
        h = OmWgpuMeshAdapter::acquireCapsule(cache, cr, ch, cs);
        break;
      }
      case WB_NODE_CONE: {
        // W1d (post-D1.4 fix): OmCone is an OmGeometry, NOT an
        // OmTriangleMeshGeometry, so the default branch's dynamic_cast
        // returned null and — with the WREN readback deleted — every visual
        // Cone was silently SKIPPED from every wgpu render, main view and
        // sensors alike. Unit cone content-keyed on
        // (subdivision, side, bottom); (r, r, height) rides the model
        // matrix, exactly OmCone::updateScale()'s convention.
        OmCone *cone = static_cast<OmCone *>(geom);
        const OmSFBool *side = geom->findSFBool("side");
        const OmSFBool *bottom = geom->findSFBool("bottom");
        const OmSFInt *subdiv = geom->findSFInt("subdivision");
        h = OmWgpuMeshAdapter::acquireCone(cache, subdiv ? subdiv->value() : 12,
                                           side ? side->isTrue() : true,
                                           bottom ? bottom->isTrue() : true);
        localScale[0] = cone->bottomRadius();
        localScale[1] = cone->bottomRadius();
        localScale[2] = cone->height();
        hasLocalScale = true;
        break;
      }
      case WB_NODE_ELEVATION_GRID: {
        // W1d (post-D1.4 fix): same class of miss as the Cone — OmElevationGrid
        // is an OmGeometry, so post-D1.4 every terrain was invisible to wgpu
        // (measured: tests/api/worlds/range_finder read inf where its
        // ElevationGrid obstacle stands at 2.0 m). Unit-spacing grid,
        // content-keyed on (dims + every height); (xSpacing, ySpacing, 1)
        // rides the model matrix — OmElevationGrid::updateScale()'s
        // convention — so heights stay unscaled in the vertices, as in WREN.
        OmElevationGrid *eg = static_cast<OmElevationGrid *>(geom);
        const int dimX = eg->xDimension();
        const int dimY = eg->yDimension();
        // Degenerate-grid early-outs, verbatim from the node's old buildWrenMesh.
        if (dimX < 2 || dimY < 2 || eg->xSpacing() == 0.0 || eg->ySpacing() == 0.0)
          break;
        // Pad missing height entries with zeros, as buildWrenMesh always did.
        const size_t n = static_cast<size_t>(dimX) * static_cast<size_t>(dimY);
        std::vector<float> heights(n, 0.0f);
        const OmMFDouble *hf = geom->findMFDouble("height");
        const size_t avail = hf ? std::min(n, static_cast<size_t>(hf->size())) : 0u;
        for (size_t i = 0; i < avail; ++i)
          heights[i] = static_cast<float>(eg->height(static_cast<int>(i)));
        h = OmWgpuMeshAdapter::acquireElevationGrid(cache, dimX, dimY, heights.data());
        localScale[0] = eg->xSpacing();
        localScale[1] = eg->ySpacing();
        localScale[2] = 1.0;
        hasLocalScale = true;
        break;
      }
      default: {
        // WREN-retirement W1: source tessellated geometry (IndexedFaceSet, Mesh,
        // PointSet-adjacent tessellations, ...) DIRECTLY from the engine's CPU-side
        // triangle mesh — i.e. OmTriangleMeshGeometry subclasses ONLY. Anything that
        // is a bare OmGeometry does NOT land here usefully: the dynamic_cast below
        // returns null and the geometry is skipped. ElevationGrid and Cone are bare
        // OmGeometry and have their own cases above (W1d) for exactly that reason —
        // this comment used to claim they were served here, and post-D1.4 that lie
        // meant they were silently invisible. The old WREN arm read the static mesh's
        // GL buffers back
        // (wr_static_mesh_read_data -> glGetBufferSubData), which needed a live GL
        // context and could cache one garbage readback forever. Byte-equivalent by
        // construction: WREN's own buffers are built corner-expanded from these
        // same accessors with an identity transform and raw texture coordinates
        // (OmTriangleMeshGeometry::buildGeomIntoBuffers). Keyed on the
        // OmTriangleMesh* — a re-tessellated geometry gets a new key.
        // D1.4: the OMNISIM_WGPU_NATIVE_MESH hatch is retired with the WREN readback arm.
        const OmTriangleMeshGeometry *tmg = dynamic_cast<const OmTriangleMeshGeometry *>(geom);
        const OmTriangleMesh *tmesh =
          (tmg && tmg->triangleMesh() && tmg->triangleMesh()->isValid() &&
           tmg->triangleMesh()->numberOfTriangles() > 0)
            ? tmg->triangleMesh()
            : nullptr;
        // W1c cache-hit fast path: the interleave below rebuilds the whole
        // vertex stream (a dense hull is tens of thousands of corners) on
        // every rebuild of the draw list, even though the upload it feeds is
        // discarded on a hit. Ask the cache first.
        if (tmesh && cache.tryGet(reinterpret_cast<uint64_t>(tmesh), h))
          break;
        if (tmesh) {
          const int nTri = tmesh->numberOfTriangles();
          const size_t vcount = static_cast<size_t>(nTri) * 3u;
          std::vector<uint8_t> vbytes(vcount * 32u);
          std::vector<uint32_t> tmIndices(vcount);
          uint8_t *dst = vbytes.data();
          for (int t = 0; t < nTri; ++t)
            for (int v = 0; v < 3; ++v) {
              const size_t corner = static_cast<size_t>(t) * 3u + static_cast<size_t>(v);
              tmIndices[corner] = static_cast<uint32_t>(corner);
              float data[8];
              for (int comp = 0; comp < 3; ++comp)
                data[comp] = static_cast<float>(tmesh->vertex(t, v, comp));
              for (int comp = 0; comp < 3; ++comp)
                data[3 + comp] = static_cast<float>(tmesh->normal(t, v, comp));
              data[6] = static_cast<float>(tmesh->textureCoordinate(t, v, 0));
              data[7] = static_cast<float>(tmesh->textureCoordinate(t, v, 1));
              std::memcpy(dst + corner * 32u, data, 32);
            }
          h = cache.acquire(reinterpret_cast<uint64_t>(tmesh), vbytes.data(), vbytes.size(),
                            tmIndices.data(), tmIndices.size() * 4u,
                            static_cast<uint32_t>(tmIndices.size()), 32u);
        } else {
          // D1.4: the WREN static-mesh readback that used to serve geometries with no CPU
          // triangle mesh is deleted with WREN. A geometry whose tessellation is merely not
          // ready yet keeps feeding the caller's re-collect loop; one that will NEVER carry a
          // CPU mesh (a point/line set) is skipped WITHOUT bumping outSkipped, so it cannot
          // pin the main view's cached scene walk open for ever.
          if (!geom->isPostFinalizedCalled() && outSkipped)
            ++*outSkipped;
        }
        break;
      }
    }
    return h.vertexBuffer != nullptr && h.indexBuffer != nullptr;
  }

  // Recursively collect Shape draws under `root`. Verbatim from OmCamera.cpp's
  // collectShapeDraws (minus the one-shot first-draw diagnostic log, which is
  // Camera-specific). Keep in sync with OmCamera until it migrates here.
  void collectShapeDraws(OmBaseNode *root, OmWgpuMeshCache &cache,
                         std::vector<OmWgpuSolidDraw> &out,
                         std::vector<std::array<float, 16>> &modelStorage,
                         OmWgpuTextureCache *texCache,
                         std::vector<OmWgpuSceneRenderer::OmWgpuDrawRefresh> *outRefresh,
                         int *outSkipped, const QList<const OmBaseNode *> *hidden) {
    if (!root)
      return;
    // wb_supervisor_node_set_visibility(root, <this viewer>, false): WREN hides the node's
    // TRANSFORM, which hides its whole subtree, so prune here rather than skipping one draw.
    // Deliberately NOT counted in outSkipped — a hidden node is absent on purpose, not "not
    // ready yet", and counting it would make a caller re-collect for ever. `hidden` is null
    // unless the caller named a viewer AND the hatch is on.
    if (isHiddenForViewer(hidden, root))
      return;
    // R4: URDF-imported robots (a mesh-visual arm) ship CadShape visuals (.glb/.dae imports → N submeshes),
    // NOT OmShape — collectShapeDraws only handled OmShape, so the whole robot was skipped (the "draws=3
    // / dark scene" bug). Collect each submesh: its geometry + world matrix (parents included) + its
    // PBRAppearance. Since W1b both the geometry and the matrix come from the engine side; the WREN
    // mesh + wr_transform_get_matrix pair below is the hatch-off fallback.
    if (OmCadShape *cad = dynamic_cast<OmCadShape *>(root)) {
      // Permanent diagnostic, inert unless OMNISIM_WGPU_CADSHAPE_AUDIT is set: logs once per
      // CadShape the max |delta| between the pose matrix used below and WREN's own
      // wr_transform_get_matrix, so the matrix substitution is verifiable numerically.
      cad->wgpuAuditMatrices();
      const int nm = cad->wgpuMeshCount();
      // WREN-retirement W1b: prefer the CPU-side submesh geometry OmCadShape retained at import,
      // and the node's own pose matrix, over WREN's GL readback (wr_static_mesh_read_data) and
      // wr_transform_get_matrix. `native` is decided ONCE per submesh from OmCadShape's single
      // hatch read (OMNISIM_WGPU_NATIVE_CADSHAPE, value-parsed, default ON) and drives BOTH the
      // vertex source and the matrix source, so the two can never come from different places.
      // The cache key is a CONTENT hash, matching WREN's own content dedup — two CadShapes on one
      // .dae still share a single GPU upload (a per-node key would multiply mesh VRAM).
      OmMatrix4 cadPoseMatrix;
      const bool cadPoseOk = cad->wgpuWorldMatrix(cadPoseMatrix);
      std::array<float, 16> cadPose16;
      cadPose16.fill(0.0f);
      if (cadPoseOk)
        wbMatrixToColumnMajorFloat(cadPoseMatrix, cadPose16.data());
      for (int i = 0; i < nm; ++i) {
        OmCadShape::WgpuSubmesh sub;
        const bool native = cadPoseOk && cad->wgpuNativeSubmesh(i, sub);
        if (!native) {
          if (outSkipped)
            ++*outSkipped;  // submesh not retained yet — this collect is incomplete
          continue;
        }
        OmWgpuMeshHandle h = cache.acquire(sub.contentKey, sub.vertexBytes, sub.vertexBytesLen, sub.indices,
                                           sub.indexBytesLen, sub.indexCount, 32u);
        if (!h.vertexBuffer || !h.indexBuffer) {
          if (outSkipped)
            ++*outSkipped;  // upload failed — retried while incomplete
          continue;
        }
        std::array<float, 16> model = cadPose16;
        bool degenerate = false;
        for (int k = 0; k < 16; ++k)
          if (!std::isfinite(model[k]))
            degenerate = true;
        if (degenerate || std::fabs(model[12]) > 1e4 || std::fabs(model[13]) > 1e4 ||
            std::fabs(model[14]) > 1e4)
          continue;
        modelStorage.push_back(model);
        OmWgpuSolidDraw draw;
        draw.modelMatrix16 = modelStorage.back().data();
        OmPbrAppearance *cpbr = cad->wgpuAppearance(i);
        const OmRgb color = cpbr ? cpbr->baseColor() : OmRgb(0.7, 0.7, 0.7);
        draw.baseColorR = static_cast<float>(color.red());
        draw.baseColorG = static_cast<float>(color.green());
        draw.baseColorB = static_cast<float>(color.blue());
        draw.baseColorA = 1.0f;
        if (cpbr) {
          const double sm = 1.0 - cpbr->roughness();
          draw.specularStrength = static_cast<float>(sm < 0.0 ? 0.0 : (sm > 1.0 ? 1.0 : sm));
          const double tr = cpbr->transparency();
          if (tr > 0.0) {
            draw.baseColorA = static_cast<float>(tr >= 1.0 ? 0.0 : 1.0 - tr);
            draw.translucent = true;
          }
        }
        draw.vertexBuffer = h.vertexBuffer;
        draw.indexBuffer = h.indexBuffer;
        draw.indexCount = h.indexCount;
        draw.localCenter[0] = h.localCenter[0];
        draw.localCenter[1] = h.localCenter[1];
        draw.localCenter[2] = h.localCenter[2];
        draw.localRadius = h.localRadius;
        draw.cpuPositions = h.cpuPositions;
        draw.cpuIndices = h.cpuIndices;
        draw.castShadows = cad->wgpuCastShadows();
        fillUvTransform(cpbr, draw);
        fillWrenAmbient(cpbr, nullptr, draw);
        if (texCache && cpbr) {
          if (OmImageTexture *map = cpbr->baseColorMap()) {
            const QImage *img = map->image();
            if (img) {
              const uint64_t texId = stableTexId(map);
              OmWgpuTextureHandle th = OmWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
              draw.textureView = th.view;
              draw.texMeanLin[0] = th.meanLin[0];
              draw.texMeanLin[1] = th.meanLin[1];
              draw.texMeanLin[2] = th.meanLin[2];
            }
          }
        }
        out.push_back(draw);
        if (outRefresh) {
          OmWgpuSceneRenderer::OmWgpuDrawRefresh r;
          r.cad = cad;
          r.node = cad;
          outRefresh->push_back(r);
        }
      }
      return;  // a CadShape is a leaf — nothing else under it
    }
    OmShape *shape = dynamic_cast<OmShape *>(root);
    if (shape) {
      OmGeometry *geom = shape->geometry();
      OmPbrAppearance *pbr = shape->pbrAppearance();
      // URDF-imported robots ship a plain Appearance (Material/diffuseColor), not a
      // PBRAppearance — fall back to it when pbr is null, else every such shape is skipped and the
      // robot never renders (the "draws=3 / dark scene" bug).
      OmAppearance *app = pbr ? nullptr : shape->appearance();
      // OmShape::findClosestDescendantNodesWithDedicatedWrenNode() returns the GEOMETRY, so a
      // Shape hidden through the supervisor API appears in the set as `geom`, never as `shape`
      // — the subtree prune above cannot see it. Fold it into the existing gate instead of
      // returning, so no other branch's control flow moves.
      const bool geomHidden = isHiddenForViewer(hidden, geom);
      // F2.4 (wren-deletion-runbook): the gate here used to also require `(pbr || app)`,
      // which silently DROPPED every Shape authored with no appearance at all -- legal
      // VRML that WREN renders flat white and, more importantly, that a DEPTH sensor must
      // still see. Measured on tests/api/worlds/range_finder.omniworld: of its four
      // authored obstacles the wgpu RangeFinder rendered ONLY the one Shape that declares
      // an Appearance; the three bare `Shape { geometry ... }` obstacles were invisible
      // (background +inf) while WREN reported 2.0 m hits. The branch body is null-safe for
      // both pointers (fallback 0.8 grey, no emissive, fillUvTransform/fillWrenAmbient/
      // applyLegacyAppearanceTexture all guard), so the appearance test was pure loss.
      if (geom && !geomHidden) {
        // WREN-retirement W1c: this used to be gated on `geom->wrenMesh()` being non-null
        // BEFORE the shape was considered at all. That single test was doing two unrelated
        // jobs.
        //
        //  (1) It supplied the cache key and the readback source. Only the `default:`
        //      branch's fallback still consumes a WrStaticMesh (W1a took tessellated
        //      geometry native, W1b took CadShape native, and the primitives below are now
        //      content-keyed), so the read moved down into that one branch.
        //
        //  (2) It silently doubled as TWO liveness tests, and dropping it outright would
        //      have lost both without a symptom:
        //
        //      (2a) "this geometry is finalized". OmBaseNode::finalize() is what calls
        //           createWrenObjects(), and the primitives sanitize their size fields
        //           inside it (OmBox/OmPlane/OmSphere/OmCylinder/OmCapsule all call
        //           sanitizeFields() there) — so a pre-finalize geometry can carry an
        //           unsanitized size and an unresolved upperPose(), i.e. it draws garbage.
        //
        //      (2b) "this geometry has a surface at all" — see primitiveHasNoSurface().
        //
        // (2a) is now the explicit engine-side flag. Ordering is what makes the swap safe:
        // OmShape::createWrenObjects() calls geometry()->createWrenObjects() and
        // OmShape::postFinalize() calls geometry()->postFinalize(), and in
        // OmBaseNode::finalize() the WREN call comes FIRST — so isPostFinalizedCalled()
        // flips strictly after wrenMesh() did, never before. The new gate is therefore
        // never looser in time than the old one, and unlike wrenMesh() it stays meaningful
        // in a build with no WREN to ask.
        //
        // OMNISIM_WGPU_NATIVE_PRIMITIVES=0 puts the old gate back verbatim, key and all.
        OmWgpuMeshHandle h;
        double localScale[3] = {1.0, 1.0, 1.0};
        bool hasLocalScale = false;
        if (acquireGeometryMesh(geom, cache, h, localScale, hasLocalScale, outSkipped)) {
          OmWgpuSolidDraw draw;
          std::array<float, 16> model;
          OmMatrix4 modelMat = geom->matrix();
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
          OmRgb color(0.8, 0.8, 0.8);
          double smooth = 0.0, emR = 0.0, emG = 0.0, emB = 0.0, transparency = 0.0;
          if (pbr) {
            color = pbr->baseColor();
            const OmRgb emissive = pbr->emissiveColor();
            const double ei = pbr->emissiveIntensity();
            emR = emissive.red() * ei;
            emG = emissive.green() * ei;
            emB = emissive.blue() * ei;
            smooth = 1.0 - pbr->roughness();
            transparency = pbr->transparency();
          } else if (app) {
            if (OmMaterial *mat = app->material()) {
              color = mat->diffuseColor();
              transparency = mat->transparency();
              // F2.2 (wren-deletion-runbook): a plain Material's emissiveColor is a live
              // runtime channel -- OmLed drives its on-colour through
              // Material::setEmissiveColor (OmLed.cpp, setMaterialsAndLightsColor) -- so
              // dropping it here made every phong LED invisible to a wgpu camera
              // (tests/api/led: "phong material should be bright red"). WREN's phong
              // shader ADDS emissive unscaled; there is no intensity factor on the
              // plain-Material path (that is a PBRAppearance-only field).
              const OmRgb emissive = mat->emissiveColor();
              emR = emissive.red();
              emG = emissive.green();
              emB = emissive.blue();
            }
          }
          smooth = smooth < 0.0 ? 0.0 : (smooth > 1.0 ? 1.0 : smooth);
          draw.baseColorR = static_cast<float>(color.red());
          draw.baseColorG = static_cast<float>(color.green());
          draw.baseColorB = static_cast<float>(color.blue());
          draw.baseColorA = 1.0f;
          if (transparency > 0.0) {
            draw.baseColorA = static_cast<float>(transparency >= 1.0 ? 0.0 : 1.0 - transparency);
            draw.translucent = true;
          }
          draw.emissiveR = static_cast<float>(emR);
          draw.emissiveG = static_cast<float>(emG);
          draw.emissiveB = static_cast<float>(emB);
          draw.specularStrength = static_cast<float>(smooth);
          draw.vertexBuffer = h.vertexBuffer;
          draw.indexBuffer = h.indexBuffer;
          draw.indexCount = h.indexCount;
          draw.localCenter[0] = h.localCenter[0];
          draw.localCenter[1] = h.localCenter[1];
          draw.localCenter[2] = h.localCenter[2];
          draw.localRadius = h.localRadius;
          draw.cpuPositions = h.cpuPositions;
          draw.cpuIndices = h.cpuIndices;
          draw.castShadows = shape->isCastShadowsEnabled();
          fillUvTransform(pbr ? static_cast<OmAbstractAppearance *>(pbr)
                              : static_cast<OmAbstractAppearance *>(app),
                          draw);
          fillWrenAmbient(pbr, app, draw);
          // P11: a legacy `Appearance`'s texture. Deliberately OUTSIDE the `if (texCache &&
          // pbr)` block below -- that guard is exactly what kept these images off the GPU.
          // No-op when `app` is null (a PBRAppearance draw) or its texture is not loaded.
          applyLegacyAppearanceTexture(app, texCache, draw);
          // R4 material fidelity: upload the PBRAppearance's baseColorMap (albedo) and
          // hand the texture view to the draw. Keyed on the OmImageTexture pointer so
          // repeat frames skip the re-upload. Null texCache / no map / plain Appearance → flat baseColor.
          if (texCache && pbr) {
            if (OmImageTexture *map = pbr->baseColorMap()) {
              const QImage *img = map->image();
              if (img) {
                const uint64_t texId = stableTexId(map);
                OmWgpuTextureHandle th =
                  OmWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
                draw.textureView = th.view;
                draw.texMeanLin[0] = th.meanLin[0];
                draw.texMeanLin[1] = th.meanLin[1];
                draw.texMeanLin[2] = th.meanLin[2];
              }
            }
            // Per-pixel roughnessMap (modulates the specular highlight in the textured-lit
            // path). Only meaningful alongside an albedo map (the flat path ignores it);
            // null → the render target binds a default-white texture (no-op).
            if (OmImageTexture *rmap = pbr->roughnessMap()) {
              const QImage *img = rmap->image();
              if (img) {
                const uint64_t texId = stableTexId(rmap);
                OmWgpuTextureHandle th =
                  OmWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
                draw.roughnessView = th.view;
              }
            }
            // metalnessMap (.r) — metals lose diffuse + tint specular by albedo. Null →
            // default-black (dielectric, no-op).
            if (OmImageTexture *mmap = pbr->metalnessMap()) {
              const QImage *img = mmap->image();
              if (img) {
                const uint64_t texId = stableTexId(mmap);
                OmWgpuTextureHandle th =
                  OmWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
                draw.metalnessView = th.view;
              }
            }
            // normalMap (tangent-space) — perturbs the shading normal. Null → default-flat
            // (no perturbation, no-op).
            if (OmImageTexture *nmap = pbr->normalMap()) {
              const QImage *img = nmap->image();
              if (img) {
                const uint64_t texId = stableTexId(nmap);
                OmWgpuTextureHandle th =
                  OmWgpuImageAdapter::acquireFromQImage(*texCache, texId, *img);
                draw.normalView = th.view;
              }
            }
          }
          // W3/P3 Pen. Deliberately OUTSIDE the `pbr` guard above: a Pen paints on any Shape
          // with a geometry (OmPaintTexture::isPaintable), including the plain-Appearance
          // Shapes the material-map block skips -- tests/api/worlds/pen.omniworld's BOARD is
          // exactly that. hasAny() is false for every world with no Pen node, so this is one
          // static bool test on the hot path there.
          if (texCache && OmPaintTexture::hasAny() && wgpuPenEnabled())
            resolvePenTexture(shape, texCache, draw);
          out.push_back(draw);
          if (outRefresh) {
            OmWgpuSceneRenderer::OmWgpuDrawRefresh r;
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
    OmGroup *group = dynamic_cast<OmGroup *>(root);
    if (group) {
      const int n = group->childCount();
      for (int i = 0; i < n; ++i)
        collectShapeDraws(group->child(i), cache, out, modelStorage, texCache, outRefresh, outSkipped, hidden);
    }
    // Articulated robots chain their links through JOINTS, not the children field: each subsequent link
    // is a OmBasicJoint's solidEndPoint(). The group recursion reaches the joint (it's a child of the
    // parent link) but a joint is not a OmGroup, so its endpoint was never descended — only the base
    // link rendered (the "draws=3" bug). Descend the endpoint here so the whole arm is collected.
    if (OmBasicJoint *joint = dynamic_cast<OmBasicJoint *>(root)) {
      if (OmSolid *ep = joint->solidEndPoint())
        collectShapeDraws(ep, cache, out, modelStorage, texCache, outRefresh, outSkipped, hidden);
    }
  }


  // ------------------------------------------------------------------
  // DEFORMABLES (Cloth / SoftBody) - see the long note in the header.
  // ------------------------------------------------------------------

  // OMNISIM_WGPU_DEFORMABLES - VALUE-PARSED, and parsed EXPLICITLY rather than through
  // qEnvironmentVariableIntValue(). That helper returns 0 for any non-numeric string, so
  // `=true` and `=on` would both read as OFF; here only "0" / "false" / "off" / "no"
  // (case-insensitive) disable, unset or anything else enables. `=0` can never mean ON -
  // the presence-gated trap this tree has shipped before. Read once per process.
  bool wgpuDeformablesEnabled() {
    static const bool sEnabled = []() {
      if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_DEFORMABLES"))
        return true;
      const QString v = qEnvironmentVariable("OMNISIM_WGPU_DEFORMABLES").trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return sEnabled;
  }

  // OMNISIM_WGPU_DEFORMABLE_EPOCH - the exact-revert hatch for the PER-CACHE UPLOAD decision.
  //
  // OFF restores the pre-P1 rule: the vertex re-upload is gated on the process-global "the
  // simulation clock advanced since the last pump" edge instead of on THIS cache's own epoch.
  // That is the bug, kept runnable on purpose, because it is otherwise unprovable: with only
  // one collector in the process (any headless run) the two rules are indistinguishable, and
  // it takes a MAIN VIEW and a Camera collecting in the same step to separate them. With this
  // arm the loser of that race uploads once and then renders a FROZEN surface for ever, while
  // still compiling clean, still drawing the right number of triangles, and still passing any
  // single-screenshot test. Value-parsed, default ON; read once per process.
  bool wgpuDeformableEpochEnabled() {
    static const bool sEnabled = []() {
      if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_DEFORMABLE_EPOCH"))
        return true;
      const QString v = qEnvironmentVariable("OMNISIM_WGPU_DEFORMABLE_EPOCH").trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return sEnabled;
  }

  // OMNISIM_WGPU_GRANULAR - the exact-revert hatch for the P9 particle path, parsed the
  // same explicit way as the one above (unset / anything but "0"/"false"/"off"/"no" is ON).
  // OFF restores the pre-P9 renderer exactly: no particle draws and no host-position pump,
  // i.e. GranularGroup renders through WREN only, as it did before. Read once per process.
  bool wgpuGranularEnabled() {
    static const bool sEnabled = []() {
      if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_GRANULAR"))
        return true;
      const QString v = qEnvironmentVariable("OMNISIM_WGPU_GRANULAR").trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return sEnabled;
  }

  // OMNISIM_WGPU_SENSOR_DYNAMIC - the exact-revert hatch for the half of P1/P9 that this
  // step is actually about: whether a SENSOR device (Camera / RangeFinder / Lidar) collects
  // dynamic content at all. OFF reproduces the pre-P1 sensor image byte-for-byte (deformables
  // and particles invisible to every sensor) while leaving the MAIN VIEW alone, which is what
  // makes a red/green A/B possible on one binary without also blanking the screen. Same
  // explicit value parse; read once per process.
  bool wgpuSensorDynamicEnabled() {
    static const bool sEnabled = []() {
      if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_SENSOR_DYNAMIC"))
        return true;
      const QString v = qEnvironmentVariable("OMNISIM_WGPU_SENSOR_DYNAMIC").trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return sEnabled;
  }

  // OMNISIM_WGPU_TRACK / OMNISIM_WGPU_MUSCLE - the exact-revert hatches for P2. Same
  // explicit value parse as their neighbours (unset or anything but "0"/"false"/"off"/"no"
  // is ON, so "=0" can never mean ON), read once per process. OFF removes that node type's
  // draws AND, for a world with no other dynamic content, its wgpu-side animateMesh() pump -
  // i.e. exactly the pre-P2 renderer.
  bool wgpuTrackEnabled() {
    static const bool sEnabled = []() {
      if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_TRACK"))
        return true;
      const QString v = qEnvironmentVariable("OMNISIM_WGPU_TRACK").trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return sEnabled;
  }

  bool wgpuMuscleEnabled() {
    static const bool sEnabled = []() {
      if (!qEnvironmentVariableIsSet("OMNISIM_WGPU_MUSCLE"))
        return true;
      const QString v = qEnvironmentVariable("OMNISIM_WGPU_MUSCLE").trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return sEnabled;
  }

  // ---- THE TRACK / MUSCLE PUMP (P2) --------------------------------------------------
  //
  // Both node types rebuild their geometry from a wr_scene FRAME LISTENER, which fires from
  // inside wr_scene_render - a call no wgpu render path makes. Without this, a Track's belt
  // never advances (and OmTrack::animateMesh is also what DRAINS mAnimationStepSize, so the
  // accumulator grows without bound) and a Muscle's spheroid is never rebuilt from its new
  // height and radius. Same shape as the deformables' pump: process-global, keyed on the
  // simulation clock in its OWN variable so the two drivers cannot swallow each other's
  // edge, and idempotent within a step so N renderers pay for it once.
  //
  // ⚠ Unlike animateDeformables(), this CONSUMES the subscriptions - see
  // OmDeformableFrameListener::animateTracksAndMuscles for why that is right here and
  // wrong there.
  void pumpTrackAndMuscleAnimations() {
    static double sLastPumpTime = -1.0;
    const double now = OmSimulationState::instance() ? OmSimulationState::instance()->time() : 0.0;
    if (now == sLastPumpTime)
      return;
    sLastPumpTime = now;
    OmDeformableFrameListener::instance()->animateTracksAndMuscles();
  }

  // Mesh-cache key for one deformable node.
  //
  // KEY-SPACE PROOF (the map is documented in OmWgpuMeshAdapter.cpp): raw-pointer keys
  // (WrStaticMesh*, OmTriangleMesh*) are canonical 47-bit, so bits 63 and 62 are CLEAR;
  // OmCadShape's content hashes always set bit 63; the codegen primitives set bit 62 and
  // use bit 61 to split "unit-mesh ordinal" (bit 61 clear, value 0..3) from "capsule
  // hash" (bit 61 set). Deformables therefore take bit 62 SET + bit 61 CLEAR + bit 60
  // SET, a region no ordinal in 0..3 and nothing else in the cache can reach, with the
  // node pointer in the low 48 bits.
  //
  // A node POINTER (not a content hash) is the right key precisely because the content
  // changes every frame: the entry has to survive the change so it can be written into.
  // The cache is destroyed on world teardown (OmView3D), so a key cannot outlive its
  // world; within a world, an address reused by a different node is caught by the
  // index-count check at the call site.
  constexpr uint64_t kDeformableKeyTag =
    (static_cast<uint64_t>(1) << 62) | (static_cast<uint64_t>(1) << 60);
  uint64_t deformableMeshKey(const void *node) {
    return kDeformableKeyTag |
           (reinterpret_cast<uint64_t>(node) & ((static_cast<uint64_t>(1) << 48) - 1u));
  }

  // Mesh-cache key for one Muscle (P2). Same key SPACE as the deformables' (bit 62 set,
  // bit 61 clear, bit 60 set, node pointer in the low 48), separated by bit 59 -- which a
  // deformable key always leaves CLEAR, because its payload is a 48-bit pointer. So the two
  // families cannot collide, and neither can reach an ordinal, a capsule hash, a CadShape
  // content hash or a raw WrStaticMesh/OmTriangleMesh pointer.
  constexpr uint64_t kMuscleKeyTag = (static_cast<uint64_t>(1) << 62) |
                                     (static_cast<uint64_t>(1) << 60) |
                                     (static_cast<uint64_t>(1) << 59);
  uint64_t muscleMeshKey(const void *node) {
    return kMuscleKeyTag |
           (reinterpret_cast<uint64_t>(node) & ((static_cast<uint64_t>(1) << 48) - 1u));
  }

  // ---- THE PER-FRAME VERTEX RE-UPLOAD, for every node type that streams one ----------
  //
  // Extracted from appendDeformableDraw so a Cloth, a SoftBody and a Muscle share ONE
  // implementation of the decision that has already gone wrong once here.
  //
  // ⚠ THAT DECISION IS PER-(CACHE, MESH), NOT PROCESS-GLOBAL, AND THE DIFFERENCE IS THE
  // WHOLE POINT. Each device owns its own OmWgpuMeshCache. When the "is my copy stale?"
  // test was a function-local static keyed on the simulation clock, whichever renderer
  // reached a step FIRST consumed the edge and every later one took the first-upload branch
  // once and then never updated -- a surface that animates on screen and is FROZEN at its
  // first pose in every Camera / RangeFinder / Lidar image, with the right draw count, the
  // right triangles and a passing single-screenshot test. `epoch` is the simulation time
  // and it is compared against THIS cache's stamp (OmWgpuMeshCache::vertexEpochIs).
  // `globalAdvanced` is used ONLY by OMNISIM_WGPU_DEFORMABLE_EPOCH=0, which restores the
  // broken global rule so the failure stays reproducible on demand.
  //
  // FillFn is `bool(std::vector<unsigned char> &)`: interleave the CURRENT surface as
  // pos3 + norm3 + uv2, stride 32, and return false when there is nothing to draw.
  //
  // The index buffer is uploaded once and never touched again; the vertex buffer is
  // re-written in place (no release + re-acquire, so no buffer churn per frame).
  template <class FillFn>
  bool acquireStreamedMesh(uint64_t key, const std::vector<unsigned int> &indices, double epoch,
                           bool globalAdvanced, OmWgpuMeshCache &cache,
                           std::vector<unsigned char> &scratch, FillFn fill,
                           OmWgpuMeshHandle &h) {
    if (indices.empty())
      return false;
    const uint32_t indexCount = static_cast<uint32_t>(indices.size());
    const bool cached = cache.tryGet(key, h);
    // A cached entry whose index count disagrees names DIFFERENT geometry than this node
    // has now - a creaseAngle re-split, or (in principle) a freed node's address reused.
    // Drop it and re-upload rather than draw someone else's triangles.
    const bool stale = cached && (h.indexCount != indexCount || !h.vertexBuffer);
    if (stale) {
      cache.release(key);
      h = OmWgpuMeshHandle();
    }
    if (!cached || stale) {
      if (!fill(scratch))
        return false;
      h = cache.acquire(key, scratch.data(), scratch.size(), indices.data(),
                        indices.size() * sizeof(unsigned int), indexCount, 32u);
      cache.setVertexEpoch(key, epoch);
    } else if (wgpuDeformableEpochEnabled() ? !cache.vertexEpochIs(key, epoch) : globalAdvanced) {
      // THE PER-FRAME PATH. One CPU pass to interleave + one wgpuQueueWriteBuffer into the
      // buffer that is already there. No buffer create/destroy, no index re-upload.
      // Gated on THIS cache's epoch, so N renderers in one step do N uploads (one each) and
      // a second render of the SAME step through the same cache does none.
      if (fill(scratch)) {
        if (!cache.updateVertices(key, scratch.data(), scratch.size(), 32u)) {
          // Byte length changed under us (only reachable through a topology edit): rebuild.
          cache.release(key);
          h = cache.acquire(key, scratch.data(), scratch.size(), indices.data(),
                            indices.size() * sizeof(unsigned int), indexCount, 32u);
        } else {
          cache.tryGet(key, h);  // re-read the refreshed bounding sphere + cpuPos pointer
        }
        cache.setVertexEpoch(key, epoch);
      }
    }
    return h.vertexBuffer != nullptr && h.indexBuffer != nullptr && h.indexCount != 0;
  }

  // Material for a draw whose appearance comes from a node OUTSIDE the Shape walk: a
  // Cloth / SoftBody (P1) and a Track belt element (P2). Deliberately a SEPARATE function
  // from the OmShape branch's inline block rather than a refactor of it: that change must
  // not be able to move a single pixel on a world with none of these, and the Shape path is
  // the most-exercised code in the renderer. The extraction rules are the same ones -
  // PBRAppearance first (baseColor, emissiveColor x emissiveIntensity, 1-roughness,
  // transparency, the four material maps, the TextureTransform), a plain Appearance's
  // Material.diffuseColor next, and the node's own legacy `diffuseColor` last.
  void fillAppearanceMaterial(OmPbrAppearance *pbr, OmAppearance *app, const float fallbackRgb[3],
                              OmWgpuTextureCache *texCache, OmWgpuSolidDraw &draw) {
    OmRgb color(fallbackRgb[0], fallbackRgb[1], fallbackRgb[2]);
    double smooth = 0.0, emR = 0.0, emG = 0.0, emB = 0.0, transparency = 0.0;
    if (pbr) {
      color = pbr->baseColor();
      const OmRgb emissive = pbr->emissiveColor();
      const double ei = pbr->emissiveIntensity();
      emR = emissive.red() * ei;
      emG = emissive.green() * ei;
      emB = emissive.blue() * ei;
      smooth = 1.0 - pbr->roughness();
      transparency = pbr->transparency();
    } else if (app) {
      if (OmMaterial *mat = app->material()) {
        color = mat->diffuseColor();
        transparency = mat->transparency();
        // F2.2: same emissive read as the OmShape branch's inline block (kept as a
        // deliberate duplicate under this function's own no-shared-refactor rule above).
        // OmLed writes a plain Material's on-colour into emissiveColor at runtime.
        const OmRgb emissive = mat->emissiveColor();
        emR = emissive.red();
        emG = emissive.green();
        emB = emissive.blue();
      }
    }
    smooth = smooth < 0.0 ? 0.0 : (smooth > 1.0 ? 1.0 : smooth);
    draw.baseColorR = static_cast<float>(color.red());
    draw.baseColorG = static_cast<float>(color.green());
    draw.baseColorB = static_cast<float>(color.blue());
    draw.baseColorA = 1.0f;
    if (transparency > 0.0) {
      draw.baseColorA = static_cast<float>(transparency >= 1.0 ? 0.0 : 1.0 - transparency);
      draw.translucent = true;
    }
    draw.emissiveR = static_cast<float>(emR);
    draw.emissiveG = static_cast<float>(emG);
    draw.emissiveB = static_cast<float>(emB);
    draw.specularStrength = static_cast<float>(smooth);
    fillUvTransform(pbr ? static_cast<OmAbstractAppearance *>(pbr) :
                          static_cast<OmAbstractAppearance *>(app),
                    draw);
    fillWrenAmbient(pbr, app, draw);
    // P11, same call and same reason as the Shape branch: the `!pbr` early-return below is
    // the guard that kept a legacy Appearance's texture off the GPU.
    applyLegacyAppearanceTexture(app, texCache, draw);
    if (!texCache || !pbr)
      return;
    struct MapSlot {
      OmImageTexture *map;
      void **view;
      bool albedo;
    };
    // NOT named `slots`: Qt's moc keyword macro expands `slots` to nothing, so
    // `const MapSlot slots[4]` compiles as `const MapSlot [4]`.
    const MapSlot mapSlots[4] = {{pbr->baseColorMap(), &draw.textureView, true},
                                 {pbr->roughnessMap(), &draw.roughnessView, false},
                                 {pbr->metalnessMap(), &draw.metalnessView, false},
                                 {pbr->normalMap(), &draw.normalView, false}};
    for (const MapSlot &sl : mapSlots) {
      if (!sl.map)
        continue;
      const QImage *img = sl.map->image();
      if (!img)
        continue;
      OmWgpuTextureHandle th =
        OmWgpuImageAdapter::acquireFromQImage(*texCache, stableTexId(sl.map), *img);
      *sl.view = th.view;
      if (sl.albedo) {
        draw.texMeanLin[0] = th.meanLin[0];
        draw.texMeanLin[1] = th.meanLin[1];
        draw.texMeanLin[2] = th.meanLin[2];
      }
    }
  }

  // One deformable -> at most one appended draw.
  //
  // ⚠ `epoch` IS THE SIMULATION TIME, AND IT IS COMPARED AGAINST *THIS CACHE'S* COPY - not
  // against a process-global "did the clock advance since the last pump" bool. That bool is
  // what this function used to take, and it was wrong the moment a second renderer existed:
  // each device owns its own OmWgpuMeshCache, the first collector in a step consumed the
  // clock edge, and every later one skipped its re-upload while still taking the
  // first-upload branch - i.e. a cloth that animates on screen and is FROZEN at its first
  // pose in every Camera / RangeFinder / Lidar image. See OmWgpuMeshCache::vertexEpochIs.
  //
  // Returns true iff a draw was appended (and therefore exactly one model matrix too -
  // the two stay 1:1, which the caller's trim relies on).
  template <class NodeT>
  bool appendDeformableDraw(NodeT *node, double epoch, bool globalAdvanced, OmWgpuMeshCache &cache,
                            OmWgpuTextureCache *texCache, std::vector<OmWgpuSolidDraw> &out,
                            std::vector<std::array<float, 16>> &modelStorage,
                            std::vector<unsigned char> &scratch) {
    if (!node)
      return false;
    OmWgpuMeshHandle h;
    if (!acquireStreamedMesh(deformableMeshKey(node), node->wgpuIndices(), epoch, globalAdvanced,
                             cache, scratch,
                             [node](std::vector<unsigned char> &s) { return node->wgpuVertexStream(s); },
                             h))
      return false;

    // IDENTITY. Both nodes hand back WORLD-space vertices (that is what Newton's particle
    // readback contains, and why their WREN transforms are parented to the scene root at
    // identity too) - a model matrix here would apply their placement twice.
    const std::array<float, 16> model = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                         0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    modelStorage.push_back(model);

    OmWgpuSolidDraw draw;
    draw.modelMatrix16 = modelStorage.back().data();  // re-pointed by the caller after the loop
    float fallback[3] = {0.8f, 0.8f, 0.8f};
    node->wgpuFallbackDiffuse(fallback);
    OmPbrAppearance *pbr = node->pbrAppearance();
    OmAppearance *app = pbr ? nullptr : node->appearance();
    fillAppearanceMaterial(pbr, app, fallback, texCache, draw);
    draw.vertexBuffer = h.vertexBuffer;
    draw.indexBuffer = h.indexBuffer;
    draw.indexCount = h.indexCount;
    draw.localCenter[0] = h.localCenter[0];
    draw.localCenter[1] = h.localCenter[1];
    draw.localCenter[2] = h.localCenter[2];
    draw.localRadius = h.localRadius;
    draw.cpuPositions = h.cpuPositions;
    draw.cpuIndices = h.cpuIndices;
    draw.castShadows = node->wgpuCastShadows();
    out.push_back(draw);
    return true;
  }

}  // namespace

namespace OmWgpuSceneRenderer {

  bool ensureTarget(OmRenderBackend *back, int w, int h,
                    OmWgpuRenderTarget *&target, OmWgpuMeshCache *&cache,
                    int &targetW, int &targetH) {
    if (!back || back->kind() != OmRenderBackendKind::Vulkan || !back->isAvailable())
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
      OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(back);
      target = new OmWgpuRenderTarget(vb, static_cast<uint32_t>(w), static_cast<uint32_t>(h));
      targetW = w;
      targetH = h;
      if (!target->isUsable()) {
        delete target;
        target = nullptr;
        return false;
      }
      cache = new OmWgpuMeshCache(vb);
    }
    return true;
  }

  void collectWorldDraws(OmWgpuMeshCache &cache,
                         std::vector<OmWgpuSolidDraw> &out,
                         std::vector<std::array<float, 16>> &modelStorage,
                         std::vector<OmSolid *> *outNodes, OmWgpuTextureCache *texCache,
                         std::vector<OmWgpuDrawRefresh> *outRefresh, int *outSkipped,
                         const QList<const OmBaseNode *> *hiddenNodes) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return;
    // P4: snapshot the two scene-global ambient inputs (WREN's Lights.ambientLight and
    // Background.luminosity) ONCE per collect, not once per draw.
    refreshWrenAmbientEnv();
    // Per-viewer visibility (see the header). Collapse "no viewer named", "empty set" and
    // "hatch off" into ONE null pointer here, so the walk pays a single null test per node and
    // the hatch is enforced at one place for every caller.
    const QList<const OmBaseNode *> *hidden = nullptr;
    if (hiddenNodes && !hiddenNodes->isEmpty() && wgpuNodeVisibilityEnabled())
      hidden = hiddenNodes;
    const QList<OmSolid *> &tops = world->topSolids();
    for (OmSolid *s : tops) {
      const size_t before = out.size();
      collectShapeDraws(s, cache, out, modelStorage, texCache, outRefresh, outSkipped, hidden);
      if (outNodes)
        for (size_t i = before; i < out.size(); ++i)
          outNodes->push_back(s);  // every draw under top solid s maps back to s
    }
    // Worlds authored WITHOUT Solid wrappers (samples/geometries/geometric_primitives.omniworld:
    // every Shape sits under a bare Pose/Transform root node) have no top solids at all, so this
    // function collected ZERO draws and the world rendered empty under wgpu. Walk the remaining
    // root-level children too — Solids were already collected above, and light/viewpoint/background
    // nodes fall through collectShapeDraws untouched. Restricted to the outNodes == nullptr callers
    // (the main view + the sensor path): the pick/overlay consumers require a 1:1 draw→Solid
    // mapping, which a bare Pose cannot provide.
    if (!outNodes) {
      if (OmGroup *rootG = dynamic_cast<OmGroup *>(world->root())) {
        const int n = rootG->childCount();
        for (int i = 0; i < n; ++i) {
          OmBaseNode *child = rootG->child(i);
          if (!child || dynamic_cast<OmSolid *>(child))
            continue;  // solids (incl. robots) were collected via topSolids above
          collectShapeDraws(child, cache, out, modelStorage, texCache, outRefresh, outSkipped, hidden);
        }
      }
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

  bool deformablesEnabled() {
    return wgpuDeformablesEnabled();
  }

  size_t collectDeformableDraws(OmWgpuMeshCache &cache, std::vector<OmWgpuSolidDraw> &out,
                                std::vector<std::array<float, 16>> &modelStorage,
                                OmWgpuTextureCache *texCache,
                                const QList<const OmBaseNode *> *hiddenNodes) {
    // THE ZERO-COST GATE. A static bool and a static-pointer null test; no world lookup, no
    // scene walk, no singleton construction. A world with no Cloth and no SoftBody pays this
    // and nothing else, for ever.
    if (!wgpuDeformablesEnabled() || !OmDeformableFrameListener::anyDeformablesSubscribed())
      return 0;
    OmDeformableFrameListener *const reg = OmDeformableFrameListener::instance();

    // ---- DECISION 1 of 2: PUMP. Process-global, once per simulation step. -----------------
    // "Has the surface moved since the last pump?" - the SIM clock, not the frame counter.
    // Same predicate the WREN frame listener's processEvent() uses, in its own variable so the
    // two drivers cannot swallow each other's edge. This one is CORRECTLY global: it drives
    // animateDeformables(), which re-reads the solver into the nodes' CPU arrays and is
    // idempotent within a step, so exactly one caller per step should pay for it and the rest
    // should ride it. On a paused simulation nobody pumps.
    //
    // ⚠ DECISION 2 - "is my GPU copy stale?" - USED TO SHARE THIS VARIABLE, and that is the
    // bug this split fixes. It is per (mesh cache, node), it lives on the cache entry, and it
    // is read inside appendDeformableDraw. Never re-merge them: `advanced` is false for every
    // collector after the first in a step, and using it to gate an UPLOAD freezes the surface
    // in every cache but one.
    static double sLastPumpTime = -1.0;
    const double now = OmSimulationState::instance() ? OmSimulationState::instance()->time() : 0.0;
    const bool globalAdvanced = (now != sLastPumpTime);
    if (globalAdvanced) {
      // The readback the wgpu render paths never get from wr_scene_render. It must run BEFORE
      // the vertex streams are built, or this frame would upload the previous step's surface.
      reg->animateDeformables();
      sLastPumpTime = now;
    }

    // One scratch buffer for the whole pass, reused across nodes and across frames: the
    // interleave is a resize + fill, so after the first frame this allocates nothing.
    static std::vector<unsigned char> sScratch;

    // Same hidden-set convention as collectWorldDraws: null unless the caller named a viewer,
    // the set is non-empty AND the hatch is on. A Cloth / SoftBody is parented at the scene
    // root with no subtree of its own, so plain membership IS the subtree test.
    const QList<const OmBaseNode *> *hidden = nullptr;
    if (hiddenNodes && !hiddenNodes->isEmpty() && wgpuNodeVisibilityEnabled())
      hidden = hiddenNodes;

    const size_t before = out.size();
    for (OmCloth *c : reg->clothList())
      if (!isHiddenForViewer(hidden, c))
        appendDeformableDraw(c, now, globalAdvanced, cache, texCache, out, modelStorage, sScratch);
    for (OmSoftBody *b : reg->softBodyList())
      if (!isHiddenForViewer(hidden, b))
        appendDeformableDraw(b, now, globalAdvanced, cache, texCache, out, modelStorage, sScratch);
    const size_t appended = out.size() - before;

    // USE-AFTER-REALLOC, the same hazard collectWorldDraws documents at its own tail: every
    // push_back above may have reallocated `modelStorage` and dangled EVERY earlier draw's
    // modelMatrix16 - including the cached scene draws this pass appended to. Draws and models
    // are pushed 1:1 on both paths, so re-point the whole list by index.
    if (out.size() == modelStorage.size())
      for (size_t k = 0; k < out.size(); ++k)
        out[k].modelMatrix16 = modelStorage[k].data();
    return appended;
  }

  bool granularEnabled() {
    return wgpuGranularEnabled();
  }

  bool sensorDynamicEnabled() {
    return wgpuSensorDynamicEnabled();
  }

  size_t collectGranularDraws(OmWgpuMeshCache &cache, std::vector<OmWgpuSolidDraw> &out,
                              std::vector<std::array<float, 16>> &modelStorage,
                              const QList<const OmBaseNode *> *hiddenNodes) {
    // THE ZERO-COST GATE, same shape as the deformables': one static bool and an empty-test
    // on a vector that already exists. A world with no GranularGroup pays this and nothing
    // else; nothing is constructed and no scene is walked to discover the absence.
    if (!wgpuGranularEnabled() || !OmGranularGroup::anyGranularGroups())
      return 0;

    // ---- DECISION 1 of 2: PUMP. Process-global, once per simulation step. -----------------
    // The GPU->host particle copy. OmGranularGroup::onPhysicsStepStarted already does this on
    // every step, so in a running simulation the buffer is fresh and this is belt-and-braces;
    // it EARNS its place at t=0 and on a PAUSED simulation, where no physics step has fired
    // and the host buffer is still the zeros createWrenObjects() assigned -- which would draw
    // every particle at the world origin. Keyed on the sim clock in its own variable, exactly
    // like the deformables' pump.
    //
    // ⚠ THERE IS NO DECISION 2 HERE, AND THAT IS THE POINT. A particle's per-frame datum is
    // its MODEL MATRIX, which is written into the CALLER'S OWN `modelStorage` -- so it is
    // per-collector by construction and there is no shared GPU copy whose freshness a global
    // flag could get wrong. The mesh is one immutable unit sphere. That is why the deformable
    // path needed a per-cache epoch and this one does not: the state that could go stale
    // simply does not exist. (Copying the deformable pattern verbatim, global flag included,
    // is what would have made particles frozen-or-invisible in every sensor.)
    static double sLastGranularPumpTime = -1.0;
    const double now = OmSimulationState::instance() ? OmSimulationState::instance()->time() : 0.0;
    const bool pump = (now != sLastGranularPumpTime);
    if (pump)
      sLastGranularPumpTime = now;

    // ONE unit sphere for every particle in the world, content-keyed, uploaded once per cache
    // and shared thereafter -- the same mesh a `Sphere` geometry gets, so a particle is lit,
    // shadowed and tone-mapped like any other surface. (Deliberately NOT the instanced pass:
    // clearAndDrawInstanced has no lighting, materials or shadows, so it would draw flat discs
    // in a lit scene.)
    const OmWgpuMeshHandle sphere =
      OmWgpuMeshAdapter::acquirePrimitive(cache, OmWgpuMeshAdapter::PrimitiveKind::UVSphere);
    if (!sphere.vertexBuffer || !sphere.indexBuffer || sphere.indexCount == 0)
      return 0;

    const QList<const OmBaseNode *> *hidden = nullptr;
    if (hiddenNodes && !hiddenNodes->isEmpty() && wgpuNodeVisibilityEnabled())
      hidden = hiddenNodes;

    const size_t before = out.size();
    for (OmGranularGroup *g : OmGranularGroup::liveGroups()) {
      if (!g || isHiddenForViewer(hidden, g))
        continue;
      if (pump)
        g->refreshHostPositions();
      const float *xyzr = nullptr;
      int n = 0;
      if (!g->wgpuParticles(xyzr, n) || !xyzr || n <= 0)
        continue;  // no live GPU state -> draw nothing (see OmGranularGroup::wgpuParticles)
      out.reserve(out.size() + static_cast<size_t>(n));
      modelStorage.reserve(modelStorage.size() + static_cast<size_t>(n));
      for (int i = 0; i < n; ++i) {
        const float px = xyzr[i * 4 + 0], py = xyzr[i * 4 + 1], pz = xyzr[i * 4 + 2];
        // The kernel writes a per-particle radius in .w; fall back to the node's authored
        // `radius` if it is not populated. A non-finite position is a dead particle, not a
        // draw -- the same degenerate guard the Shape path applies to its matrices.
        float r = xyzr[i * 4 + 3];
        if (!(r > 0.0f) || !std::isfinite(r))
          r = static_cast<float>(g->radius());
        if (r <= 0.0f || !std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
          continue;
        // Column-major TRS with a uniform scale: the unit sphere is radius 1, so scale IS the
        // particle radius and there is no rotation to carry.
        const std::array<float, 16> model = {r,  0,  0,  0,  0,  r,  0,  0,
                                             0,  0,  r,  0,  px, py, pz, 1};
        modelStorage.push_back(model);
        OmWgpuSolidDraw draw;
        draw.modelMatrix16 = modelStorage.back().data();  // re-pointed below
        // WREN's phong material for this node, term for term (OmGranularGroup::createWrenObjects):
        // sand-yellow diffuse, a low specular, no emission, shadows RECEIVED but not CAST.
        draw.baseColorR = 0.85f;
        draw.baseColorG = 0.70f;
        draw.baseColorB = 0.40f;
        draw.baseColorA = 1.0f;
        draw.specularStrength = 0.30f;
        draw.castShadows = false;
        draw.vertexBuffer = sphere.vertexBuffer;
        draw.indexBuffer = sphere.indexBuffer;
        draw.indexCount = sphere.indexCount;
        draw.localCenter[0] = sphere.localCenter[0];
        draw.localCenter[1] = sphere.localCenter[1];
        draw.localCenter[2] = sphere.localCenter[2];
        draw.localRadius = sphere.localRadius;
        draw.cpuPositions = sphere.cpuPositions;
        draw.cpuIndices = sphere.cpuIndices;
        out.push_back(draw);
      }
    }
    const size_t appended = out.size() - before;

    // Same use-after-realloc re-point the other two collects end with.
    if (out.size() == modelStorage.size())
      for (size_t k = 0; k < out.size(); ++k)
        out[k].modelMatrix16 = modelStorage[k].data();
    return appended;
  }

  bool trackEnabled() {
    return wgpuTrackEnabled();
  }

  bool muscleEnabled() {
    return wgpuMuscleEnabled();
  }

  size_t collectTrackDraws(OmWgpuMeshCache &cache, std::vector<OmWgpuSolidDraw> &out,
                           std::vector<std::array<float, 16>> &modelStorage,
                           OmWgpuTextureCache *texCache,
                           const QList<const OmBaseNode *> *hiddenNodes) {
    // THE ZERO-COST GATE, same shape as the other three: one static bool and an empty-test
    // on a vector that already exists. A world with no Track pays this and nothing else.
    if (!wgpuTrackEnabled() || !OmTrack::anyTracks())
      return 0;
    // The belt has to be advanced before it is read. Idempotent within a step, so the main
    // view and every sensor in the same step share one advance.
    pumpTrackAndMuscleAnimations();

    const QList<const OmBaseNode *> *hidden = nullptr;
    if (hiddenNodes && !hiddenNodes->isEmpty() && wgpuNodeVisibilityEnabled())
      hidden = hiddenNodes;

    // Reused across tracks and across frames: after the first frame this allocates nothing.
    static std::vector<OmTrack::WgpuBeltDraw> sBelt;

    const size_t before = out.size();
    for (OmTrack *tr : OmTrack::liveTracks()) {
      if (!tr || isHiddenForViewer(hidden, tr))
        continue;
      sBelt.clear();
      tr->wgpuBeltDraws(sBelt);
      for (const OmTrack::WgpuBeltDraw &b : sBelt) {
        OmWgpuMeshHandle h;
        double localScale[3] = {1.0, 1.0, 1.0};
        bool hasLocalScale = false;
        // outSkipped is deliberately null: a belt is rebuilt from scratch every frame, so a
        // geometry that is not ready yet simply reappears next frame. Feeding the main
        // view's re-collect counter from here would pin its cached SCENE walk open for a
        // reason that has nothing to do with the scene.
        if (!acquireGeometryMesh(b.geometry, cache, h, localScale, hasLocalScale, nullptr))
          continue;
        OmMatrix4 world = b.world;
        if (hasLocalScale)
          world.scale(localScale[0], localScale[1], localScale[2]);
        std::array<float, 16> model;
        wbMatrixToColumnMajorFloat(world, model.data());
        bool degenerate = false;
        for (int k = 0; k < 16; ++k)
          if (!std::isfinite(model[k]))
            degenerate = true;
        if (degenerate)
          continue;
        modelStorage.push_back(model);
        OmWgpuSolidDraw draw;
        draw.modelMatrix16 = modelStorage.back().data();  // re-pointed below
        OmPbrAppearance *pbr = b.shape ? b.shape->pbrAppearance() : nullptr;
        OmAppearance *app = (pbr || !b.shape) ? nullptr : b.shape->appearance();
        const float fallback[3] = {0.8f, 0.8f, 0.8f};
        fillAppearanceMaterial(pbr, app, fallback, texCache, draw);
        draw.castShadows = b.castShadows;
        draw.vertexBuffer = h.vertexBuffer;
        draw.indexBuffer = h.indexBuffer;
        draw.indexCount = h.indexCount;
        draw.localCenter[0] = h.localCenter[0];
        draw.localCenter[1] = h.localCenter[1];
        draw.localCenter[2] = h.localCenter[2];
        draw.localRadius = h.localRadius;
        draw.cpuPositions = h.cpuPositions;
        draw.cpuIndices = h.cpuIndices;
        out.push_back(draw);
      }
    }
    const size_t appended = out.size() - before;

    // Same use-after-realloc re-point the other collects end with.
    if (out.size() == modelStorage.size())
      for (size_t k = 0; k < out.size(); ++k)
        out[k].modelMatrix16 = modelStorage[k].data();
    return appended;
  }

  size_t collectMuscleDraws(OmWgpuMeshCache &cache, std::vector<OmWgpuSolidDraw> &out,
                            std::vector<std::array<float, 16>> &modelStorage,
                            OmWgpuTextureCache *texCache,
                            const QList<const OmBaseNode *> *hiddenNodes) {
    if (!wgpuMuscleEnabled() || !OmMuscle::anyMuscles())
      return 0;
    pumpTrackAndMuscleAnimations();

    // The upload epoch. Same two-decision split the deformables document: this `advanced`
    // bool drives NOTHING but the OMNISIM_WGPU_DEFORMABLE_EPOCH=0 revert arm; the real
    // staleness test lives on the cache entry (see acquireStreamedMesh).
    static double sLastEpoch = -1.0;
    const double now = OmSimulationState::instance() ? OmSimulationState::instance()->time() : 0.0;
    const bool globalAdvanced = (now != sLastEpoch);
    if (globalAdvanced)
      sLastEpoch = now;

    static std::vector<unsigned char> sScratch;

    const QList<const OmBaseNode *> *hidden = nullptr;
    if (hiddenNodes && !hiddenNodes->isEmpty() && wgpuNodeVisibilityEnabled())
      hidden = hiddenNodes;

    const size_t before = out.size();
    for (OmMuscle *m : OmMuscle::liveMuscles()) {
      if (!m || !m->wgpuVisible() || isHiddenForViewer(hidden, m))
        continue;  // `visible FALSE` hides the WREN transform; do the same, not a grey blob
      OmMatrix4 world;
      if (!m->wgpuWorldMatrix(world))
        continue;
      OmWgpuMeshHandle h;
      if (!acquireStreamedMesh(muscleMeshKey(m), OmMuscle::wgpuIndices(), now, globalAdvanced,
                               cache, sScratch,
                               [m](std::vector<unsigned char> &s) { return m->wgpuVertexStream(s); },
                               h))
        continue;
      std::array<float, 16> model;
      wbMatrixToColumnMajorFloat(world, model.data());
      bool degenerate = false;
      for (int k = 0; k < 16; ++k)
        if (!std::isfinite(model[k]))
          degenerate = true;
      if (degenerate)
        continue;
      modelStorage.push_back(model);
      OmWgpuSolidDraw draw;
      draw.modelMatrix16 = modelStorage.back().data();  // re-pointed below
      // ⚠ A Muscle IS NOT APPEARANCE-DRIVEN: it has no Appearance node at all. WREN builds a
      // hardcoded phong material -- diffuse from the `color` field blended by the contraction
      // status, the gl:textures/muscle.png main texture, and NO specular, because phong.frag
      // zeroes specularTotal on any textured material (phong.frag:187). Reproduce that
      // literally rather than reaching for fillAppearanceMaterial, which has nothing to read.
      float rgb[3] = {1.0f, 0.0f, 0.0f};
      m->wgpuDiffuse(rgb);
      draw.baseColorR = rgb[0];
      draw.baseColorG = rgb[1];
      draw.baseColorB = rgb[2];
      draw.baseColorA = 1.0f;
      draw.specularStrength = 0.0f;
      draw.castShadows = m->wgpuCastShadows();
      if (texCache) {
        if (const QImage *img = m->wgpuTextureImage()) {
          // One key for the whole process: every Muscle reads the same file, so they share a
          // single upload. Same tagged path-hash form stableTexId() produces, so a world that
          // ALSO loads muscle.png through an ImageTexture dedupes against it instead of
          // colliding with it.
          static const uint64_t sMuscleTexId =
            (static_cast<uint64_t>(0x7e7e7e7eu) << 32) |
            static_cast<uint64_t>(qHash(QFileInfo("gl:textures/muscle.png").absoluteFilePath()));
          OmWgpuTextureHandle th = OmWgpuImageAdapter::acquireFromQImage(*texCache, sMuscleTexId, *img);
          draw.textureView = th.view;
          draw.texMeanLin[0] = th.meanLin[0];
          draw.texMeanLin[1] = th.meanLin[1];
          draw.texMeanLin[2] = th.meanLin[2];
        }
      }
      draw.vertexBuffer = h.vertexBuffer;
      draw.indexBuffer = h.indexBuffer;
      draw.indexCount = h.indexCount;
      draw.localCenter[0] = h.localCenter[0];
      draw.localCenter[1] = h.localCenter[1];
      draw.localCenter[2] = h.localCenter[2];
      draw.localRadius = h.localRadius;
      draw.cpuPositions = h.cpuPositions;
      draw.cpuIndices = h.cpuIndices;
      out.push_back(draw);
    }
    const size_t appended = out.size() - before;

    if (out.size() == modelStorage.size())
      for (size_t k = 0; k < out.size(); ++k)
        out[k].modelMatrix16 = modelStorage[k].data();
    return appended;
  }

  size_t collectDynamicDraws(OmWgpuMeshCache &cache, std::vector<OmWgpuSolidDraw> &out,
                             std::vector<std::array<float, 16>> &modelStorage,
                             OmWgpuTextureCache *texCache,
                             const QList<const OmBaseNode *> *hiddenNodes,
                             OmWgpuDynamicCounts *outCounts) {
    const size_t nDeform = collectDeformableDraws(cache, out, modelStorage, texCache, hiddenNodes);
    const size_t nGranular = collectGranularDraws(cache, out, modelStorage, hiddenNodes);
    const size_t nTrack = collectTrackDraws(cache, out, modelStorage, texCache, hiddenNodes);
    const size_t nMuscle = collectMuscleDraws(cache, out, modelStorage, texCache, hiddenNodes);
    if (outCounts) {
      outCounts->deformable = nDeform;
      outCounts->granular = nGranular;
      outCounts->track = nTrack;
      outCounts->muscle = nMuscle;
    }
    return nDeform + nGranular + nTrack + nMuscle;
  }

  unsigned long long glArmCount() {
    return gGlArms;
  }

  bool nodeVisibilityEnabled() {
    return wgpuNodeVisibilityEnabled();
  }

  const QList<const OmBaseNode *> *mainViewHiddenNodes() {
    if (!wgpuNodeVisibilityEnabled())
      return nullptr;
    OmWorld *world = OmWorld::instance();
    OmViewpoint *viewpoint = world ? world->viewpoint() : nullptr;
    if (!viewpoint)
      return nullptr;
    const QList<const OmBaseNode *> &nodes = viewpoint->getInvisibleNodes();
    return nodes.isEmpty() ? nullptr : &nodes;
  }

  bool refreshWorldDraws(std::vector<std::array<float, 16>> &modelStorage,
                         const std::vector<OmWgpuDrawRefresh> &refresh,
                         std::vector<OmWgpuSolidDraw> *outDraws) {
    if (modelStorage.size() != refresh.size())
      return false;
    // ---- P2, the OTHER half of a Track, and the one a SHIPPED demo actually uses -------
    //
    // ConveyorBelt.proto animates its belt with `textureAnimation`, not `animatedGeometry`:
    // OmTrack::prePhysicsStep translates the TextureTransform's `translation` field every
    // step and the belt SCROLLS. On the wgpu SENSOR path that already worked, because every
    // sensor rebuilds its draw list per frame and fillUvTransform re-reads the field. On the
    // MAIN VIEW it did not: the draw list is cached and this function refreshes only the
    // MODEL MATRICES, so uvA/uvB stayed at whatever the last full collect saw and the belt
    // was frozen while the world around it moved. A full re-collect per frame is not the
    // answer -- it is the ~50 ms scene walk this cache exists to avoid.
    //
    // Gated on a Track actually declaring a texture animation, so every other world pays one
    // empty-vector test; and the appearance is re-derived from the refresh record's own node
    // (already hooked to destroyed()) rather than stored, so nothing new can dangle. It shares
    // OMNISIM_WGPU_TRACK with the belt collect, because both are "the pre-P2 Track renderer"
    // and splitting them would let an A/B land in a combination that never shipped.
    if (outDraws && outDraws->size() == refresh.size() && wgpuTrackEnabled() &&
        OmTrack::anyTextureAnimation())
      for (size_t k = 0; k < refresh.size(); ++k)
        if (OmShape *sh = dynamic_cast<OmShape *>(refresh[k].node))
          fillUvTransform(sh->pbrAppearance() ? static_cast<OmAbstractAppearance *>(sh->pbrAppearance()) :
                                                static_cast<OmAbstractAppearance *>(sh->appearance()),
                          (*outDraws)[k]);
    for (size_t k = 0; k < refresh.size(); ++k) {
      const OmWgpuDrawRefresh &r = refresh[k];
      std::array<float, 16> model;
      if (r.geom) {
        OmMatrix4 m = r.geom->matrix();
        if (r.hasLocalScale)
          m.scale(r.localScale[0], r.localScale[1], r.localScale[2]);
        wbMatrixToColumnMajorFloat(m, model.data());
      } else if (r.cad) {
        // W1b: CadShape world matrix from the node's own pose, not wr_transform_get_matrix.
        OmMatrix4 m;
        if (!r.cad->wgpuWorldMatrix(m))
          continue;  // keep the previous matrix
        wbMatrixToColumnMajorFloat(m, model.data());
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

  void buildView(const OmMatrix4 &cameraWorld, float outView16[16]) {
    // kBasisSwap rotates the Webots camera-local frame (+X forward, +Y left,
    // +Z up) into the convention the wgpu/OpenGL projection expects (-Z
    // forward, +Y up). Column-major. Identical to OmCamera's inline matrix.
    static const float kBasisSwap[16] = {
       0,  0, -1, 0,  // col 0 (image of camera.x)
      -1,  0,  0, 0,  // col 1 (image of camera.y)
       0,  1,  0, 0,  // col 2 (image of camera.z)
       0,  0,  0, 1,  // col 3 (translation)
    };
    OmMatrix4 camWorldInv = cameraWorld;
    if (!camWorldInv.inverse())
      camWorldInv = OmMatrix4();
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

  // W3 defect 2: `far 0` means "infinite" in the .wbt/.omniworld schema, and WREN
  // has always turned that into a finite 10 km plane inside wren::Camera::setFar
  // (src/wren/Camera.hpp:51-53, `farDistance > 0.0f ? farDistance : 10000.0f`).
  // The wgpu sensor path had no equivalent, so perspective() received zFar == 0 and
  // produced a degenerate projection (m22 == m32 == 0 -> clip.z == 0 everywhere).
  // Same predicate, same constant: a positive authored far is returned unchanged.
  double sensorFarPlane(double authoredFar) {
    return authoredFar > 0.0 ? authoredFar : kInfiniteFarSubstitute;
  }

  void buildViewProj(const OmMatrix4 &cameraWorld, double horizFov, double aspect,
                     double zNear, double zFar, float outViewProj16[16], bool reversedZ) {
    float view[16] = {0}, proj[16] = {0};
    buildView(cameraWorld, view);
    // Horizontal FOV → vertical via aspect (OmWrenCamera::computeFieldOfViewY).
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

  void buildOrthoLightViewProj(const OmMatrix4 &lightWorld, double halfExtent,
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

  void buildCascadeLightViewProjs(const OmMatrix4 &cameraWorld, double horizFov,
                                  double aspect, double camNear, double camFar,
                                  const OmMatrix4 &lightWorld, int numCascades,
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
      // world area regardless of light angle (the standard CSM choice), then STABILISE:
      // round the half-extent up to a 0.5 m grid (so the texel size itself doesn't
      // fluctuate frame to frame) and snap the centre to the cascade's texel grid at the
      // 2048 shadow resolution — shimmer-free shadow edges under camera motion.
      double cx = 0.5 * (lxMin + lxMax);
      double cy = 0.5 * (lyMin + lyMax);
      double h = 0.5 * std::max(lxMax - lxMin, lyMax - lyMin);
      if (h < 1e-4)
        h = 1e-4;
      h = std::ceil(h / 0.5) * 0.5;
      const double snapTexel = 2.0 * h / 2048.0;
      cx = std::floor(cx / snapTexel) * snapTexel;
      cy = std::floor(cy / snapTexel) * snapTexel;

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
      // Diagnostic (OMNISIM_WGPU_CSM_DIAG): one-shot per-cascade fit report + full matrices,
      // the companion to OMNISIM_WGPU_SHADOWMAP_DUMP for off-GPU shadow numerics.
      if (qEnvironmentVariableIsSet("OMNISIM_WGPU_CSM_DIAG")) {
        static int sCsmDiagN = 0;
        if (sCsmDiagN < 6) {
          ++sCsmDiagN;
          OmLog::info(QString("[CSM DIAG] c=%1 slice=[%2, %3] h=%4 zNear=%5 zFar=%6 depthSpan=%7")
                        .arg(c).arg(nI).arg(fI).arg(h).arg(zNear).arg(zFar).arg(zFar - zNear));
          QString mstr;
          for (int k = 0; k < 16; ++k)
            mstr += QString::number(outLightViewProjs[c * 16 + k], 'g', 9) + " ";
          OmLog::info(QString("[CSM DIAG] c=%1 vp16(col-major)= %2").arg(c).arg(mstr));
        }
      }
    }
  }

  // T1.2 CSM (multi-cascade) headless self-test orchestrator. See the header. Builds the
  // GPU-proven prototype camera/light frames (csm_render_prototype.py: cam (0,-18,14)→(0,4,0),
  // light (6,6,20)→origin) + the N=3 cascade fit, then drives the render-layer
  // OmWgpuRenderTarget::selfTestCsm with the raw matrices.
  bool csmSelfTest(OmWgpuRenderTarget &rt, unsigned char shadowedOut[3], unsigned char litSideOut[3],
                   unsigned char shadowOffOut[3], int *cascadeSelected) {
    // lookat (Webots convention: local +X forward, +Z up). Row-major OmMatrix4 so the engine's
    // buildView (kBasisSwap * columnMajor(inverse(world))) reproduces the prototype's frames.
    auto lookat = [](const double eye[3], const double tgt[3]) -> OmMatrix4 {
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
      return OmMatrix4(x[0], y[0], z[0], eye[0], x[1], y[1], z[1], eye[1], x[2], y[2], z[2], eye[2],
                       0, 0, 0, 1);
    };
    const double camEye[3] = {0, -18, 14}, camTgt[3] = {0, 4, 0};
    const double litEye[3] = {6, 6, 20}, litTgt[3] = {0, 0, 0};
    const OmMatrix4 camWorld = lookat(camEye, camTgt);
    const OmMatrix4 lightWorld = lookat(litEye, litTgt);

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

}  // namespace OmWgpuSceneRenderer
