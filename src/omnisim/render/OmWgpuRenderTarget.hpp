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

#ifndef OM_WGPU_RENDER_TARGET_HPP
#define OM_WGPU_RENDER_TARGET_HPP

//
// OmWgpuRenderTarget — R3.3 of engine-migration-plan.md §14.3.
//
// "Single-Camera RTT with renderBackend 'wgpu'. Solid-color shader,
//  no geometry yet. 1.5 weeks."
//
// This class owns the wgpu objects needed to render a single frame
// to an off-screen RGBA8 texture: render-target texture + view, a
// trivial WGSL render pipeline that emits a solid color (no vertex
// buffers, no bind groups), a depth attachment (kept null at R3.3
// since solid-color clear has no depth needs), and a readback
// buffer for parity tests.
//
// At R3.3 the only public verb is `clearAndRead(rgba8)` which:
//   1. Acquires a command encoder
//   2. Begins a render pass into the target with a clear color
//      driven by the constructor arg
//   3. Ends the pass + submits
//   4. Copies the texture to a CPU-mapped buffer
//   5. Blocks on the map future + memcpys into `rgba8`
//
// This is the "hello-wgpu" test that proves the device, pipeline,
// command encoder + submission, and texture readback all work
// end-to-end. R3.3b will wire this into Camera's `renderBackend
// "wgpu"` code path; R3.4 adds the WGSL shaders that draw actual
// geometry; R3.6 uses clearAndRead() as the harness for golden-
// image diffs against WREN-rendered reference frames.
//
// Lifetime: the target owns the texture, view, pipeline, and
// readback buffer. Pass in a backend pointer; the backend's
// device + queue handles must outlive the target.
//
// Failure model: every wgpu call is null-checked. If any step
// fails the target stays in an `isUsable()==false` state and
// clearAndRead returns false without writing to the buffer. No
// throws.
//

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

class OmVulkanBackend;
class QImage;

// One scene draw call (one Solid worth of geometry) for
// `OmWgpuRenderTarget::clearAndDrawScene`. The matrix is 16 floats
// column-major; baseColor is rgba in [0..1]; vb/ib are
// `WGPUBuffer` handles (typically from `OmWgpuMeshCache::acquire`
// or `OmWgpuMeshAdapter::acquireFromWren`).
struct OmWgpuSolidDraw {
  const float *modelMatrix16 = nullptr;
  float baseColorR = 1.0f;
  float baseColorG = 1.0f;
  float baseColorB = 1.0f;
  float baseColorA = 1.0f;
  // T1.1 HDR source: PBRAppearance emissiveColor × emissiveIntensity. Added to
  // the lit colour in the AgX path (kSolidLitAgX) so a >1.0 emissive gives AgX
  // genuine HDR to compress. Default 0 → no emission → lit/AgX paths unchanged.
  float emissiveR = 0.0f;
  float emissiveG = 0.0f;
  float emissiveB = 0.0f;
  // T1.1 specular: 1 - roughness from the PBRAppearance, in [0,1]. A small
  // Blinn-Phong specular highlight in kSolidLitAgX scales by this, so a smooth
  // (low-roughness) surface gets a bright >1 highlight for AgX to compress.
  // Default 0 (and roughness=1 → 0) → no specular → lit/AgX byte-identical.
  float specularStrength = 0.0f;
  void *vertexBuffer = nullptr;
  void *indexBuffer = nullptr;
  uint32_t indexCount = 0;
  // Shape/CadShape castShadows field. FALSE draws are skipped by the light-depth pass (pass 1) but
  // still render in pass 2 — matching WREN. The spot.omniworld "concentric rings" bug was the floating
  // SUN_MARKER sphere (castShadows FALSE) writing a giant occluder disc into the shadow map.
  bool castShadows = true;
  // Local bounding sphere of the mesh (from OmWgpuMeshHandle): drives shadow-frustum and camera-
  // frustum culling. radius < 0 = unknown -> the draw is NEVER culled (fail-open).
  float localCenter[3] = {0.0f, 0.0f, 0.0f};
  float localRadius = -1.0f;
  // PBRAppearance/Material transparency > 0. Translucent draws: excluded from the shadow map and
  // the SSAO depth prepass (glass neither occludes the sun nor darkens AO — matching WREN), and
  // pass 2 renders them AFTER all opaque draws, CPU-sorted back-to-front, through the src-over
  // blend + depth-write-off pipeline variants (WREN parity: sorted alpha blending, not OIT).
  bool translucent = false;
  // TextureTransform affine (row-major 2x2 + offset): uv' = M*uv + b. Identity default.
  // Applied by the textured-shadowed vertex shader to all material-map UVs.
  float uvA[4] = {1.0f, 0.0f, 0.0f, 1.0f};
  float uvB[2] = {0.0f, 0.0f};
  // OmniLight bake inputs: CPU triangle data (stable pointers into the never-evicting mesh
  // cache) + the albedo texture's LINEAR mean colour (bounce albedo; detail is irrelevant to
  // diffuse GI). Null/identity when unavailable -> the bake skips / uses baseColor alone.
  const std::vector<float> *cpuPositions = nullptr;
  const std::vector<uint32_t> *cpuIndices = nullptr;
  float texMeanLin[3] = {1.0f, 1.0f, 1.0f};
  // R4 material fidelity: optional albedo texture (WGPUTextureView from
  // OmWgpuTextureCache, via the PBRAppearance's baseColorMap). When null the draw is
  // flat-shaded from baseColor (existing behaviour); when set, the textured-lit path
  // samples it × baseColor. Sampled at the mesh UVs (vertex attribute @location 2).
  void *textureView = nullptr;
  // R4 material fidelity: optional roughnessMap (WGPUTextureView). Sampled per-pixel by
  // the textured-lit path to modulate the specular highlight (glTF semantic:
  // effectiveRoughness = roughnessFactor × map.r). Null → the draw uses a default-white
  // 1×1 texture, so a textured-but-no-roughnessMap draw stays byte-identical to before.
  void *roughnessView = nullptr;
  // Optional metalnessMap (.r in [0,1]): metals take no diffuse and tint their specular by
  // the albedo. Null → a default-BLACK 1×1 (metalness 0 = dielectric) → byte-identical.
  void *metalnessView = nullptr;
  // Optional normalMap (tangent-space, RGB-encoded): perturbs the shading normal via a
  // derivative-reconstructed tangent frame (no mesh tangents needed). Null → a default
  // FLAT-normal 1×1 (128,128,255 = +Z, no perturbation) → byte-identical.
  void *normalView = nullptr;
  // W3/P3 Pen paint layer (OmPaintTexture, via OmWgpuTextureCache). Mixed OVER the albedo by its
  // own alpha -- the same `mix(base, pen.rgb, pen.a)` WREN does at pbr.frag:522-525 /
  // phong.frag:206-208 -- and sampled at the RAW mesh UV (no TextureTransform), matching WREN's
  // `penTexUv = vUnwrappedTexCoord`. Null -> a default 1x1 TRANSPARENT texel -> alpha 0 -> the mix
  // is the identity -> byte-identical to a build with no pen support at all.
  void *penView = nullptr;
  // ---- WREN ambient parity (P4). WREN has TWO ambient models and picks by APPEARANCE TYPE;
  // wgpu had one analytic hemisphere for everything, which is why a white board under a
  // `Material { ambientIntensity 1 }` lamp read sky-blue instead of neutral.
  //
  //   * legacy `Appearance` + `Material` -> phong.frag: ambient = Lights.ambientLight x
  //     material.ambient, where Lights.ambientLight = sum(light.ambientIntensity x light.color)
  //     and material.ambient = ambientIntensity (textured) or ambientIntensity x diffuseColor
  //     (untextured). It NEVER reads Background.skyColor.
  //   * `PBRAppearance` -> pbr.frag: ambient = skyColor x (Background.luminosity x
  //     PBRAppearance.IBLStrength), on the diffuse AND the specular term. It never reads the
  //     lights' ambientIntensity.
  //
  // ambientMode selects which one this draw takes; ambientRGB carries the phong product
  // (already multiplied out on the CPU) and iblScale the pbr premultiply. Defaults
  // (mode 0, scale 1) reproduce the pre-P4 output byte-for-byte.
  float ambientR = 0.0f;
  float ambientG = 0.0f;
  float ambientB = 0.0f;
  float ambientMode = 0.0f;  // 0 = analytic sky/hemisphere ambient, 1 = WREN phong light-ambient
  float iblScale = 1.0f;     // Background.luminosity x PBRAppearance.IBLStrength (WREN's premultiply)
};

struct OmWgpuClearColor {
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  float a = 1.0f;
};

// W3 (WREN retirement): which DISPLAY TRANSFER clearAndDrawSceneTexturedShadowed's finishing pass
// applies to the linear HDR frame. It exists because ONE render function now serves two consumers
// whose output contracts genuinely differ, and neither may be given the other's:
//
//   * the MAIN VIEW is a display surface. It wants AgX (filmic), the OMNISIM_WGPU_TONEMAP_CURVE
//     A/B arm, and the camera pass (vignette + grain + auto-exposure) that makes a render read as
//     a photograph.
//   * a Camera DEVICE is a SENSOR whose bytes a controller reads. Its encoding is pinned by
//     WREN's OmWrenHdr (resources/wren/shaders/hdr_resolve.frag): mapped = 1 - exp(-c * exposure),
//     then pow(mapped, 1/2.2) -- and tests/api/controllers/camera_checker.c asserts those ENCODED
//     values to +-2, so the sensor must reproduce the CURVE, not emit linear light. It must also
//     be free of the camera pass: vignette darkens the frame corners, grain is time-varying,
//     auto-exposure makes the byte a controller reads depend on the previous frame's histogram,
//     and the display dither is a fixed +-1-code spatial pattern WREN's hdr_resolve does not
//     have. All four are display flourishes and all four are sensor NOISE.
//
// Default is the main view's behaviour, so every existing call site is bit-identical.
enum OmWgpuOutputTransfer {
  OM_WGPU_XFER_VIEW = 0,        // AgX (or the env A/B curve) + the camera pass -- the main view
  OM_WGPU_XFER_WREN_SENSOR = 1  // WREN hdr_resolve.frag exactly; no vignette/grain/auto-exposure
};

// W4a (WREN retirement): one flat-coloured batch of world-space line segments for
// OmWgpuRenderTarget::drawOverlayLines — the GUI geometry overlays (selection outline,
// bounding objects, joint axes, frustums, normals, lidar rays, contact points).
// `verts` is `vertexCount` vertices in the standard pos3+normal3+uv2 layout (stride 32;
// the normal/uv slots are ignored — pad with zeros), TWO consecutive vertices per segment.
// `color` is straight RGBA: the overlay pass runs AFTER the tonemap, so what you write is
// what the viewer sees. A batch with fewer than 2 vertices (or a null `verts`) is skipped,
// which is what makes "nothing enabled" cost nothing.
struct OmWgpuLineBatch {
  const float *verts = nullptr;
  uint32_t vertexCount = 0;
  float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

// P7 (WREN retirement): ONE screen-space HUD quad for OmWgpuRenderTarget::drawOverlayQuads --
// the device-output insets (Camera / RangeFinder / Display), their frames, and the supervisor
// labels that `wb_supervisor_set_label` puts on screen. Everything WREN drew through
// OmWrenTextureOverlay / OmWrenLabelOverlay and that went dark when the main view flipped to
// wgpu on 2026-08-19.
//
// `x`/`y`/`w`/`h` are TARGET pixels with the origin at the TOP-LEFT, i.e. exactly the frame the
// GUI hit tests in (OmWrenTextureOverlay::isInside, OmRenderingDevice::fromMousePosition). That
// is the whole point: the drawn rect and the clickable rect are the same numbers, so they cannot
// drift apart.
//
// `pixels` is `srcW * srcH` BGRA8 texels (the layout every OmniSim rendering device already keeps
// its image in -- no CPU swizzle) and may be null, in which case the quad is a flat `color` fill.
// `color` multiplies the sampled texel and is straight (non-premultiplied) RGBA; the pass runs
// after the tonemap, so what you write is what the viewer sees.
//
// `key` + `revision` drive the GPU texture cache: a quad whose (key, revision, size) repeats is
// drawn from the cached texture with NO upload. key 0 means "do not cache" (upload every frame).
// A label whose text has not changed therefore costs one draw call and zero bytes of traffic.
struct OmWgpuHudQuad {
  float x = 0.0f, y = 0.0f, w = 0.0f, h = 0.0f;
  const unsigned char *pixels = nullptr;  // BGRA8, srcW*srcH*4; null -> flat colour fill
  uint32_t srcW = 0, srcH = 0;
  unsigned long long key = 0;       // texture-cache identity (0 = uncached)
  unsigned long long revision = 0;  // content revision for `key`
  float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  bool flipV = false;  // source's first row is the BOTTOM one
};

// R4 3c-B un-gate leak-hunt (free function): dump wgpu-native's GLOBAL resource registry
// (wgpuGenerateReport) so a per-frame hook that only has the WGPUInstance — e.g. OmView3D::renderNow,
// which runs every frame regardless of which render backend is active — can monitor texture/buffer
// growth from ANY wgpu path (scene cache, render targets, sensors). `instance` is an opaque
// WGPUInstance; appends one line tagged with `frame` to `path` (append mode). Inert on non-wgpu
// builds or when instance/path is null. Diagnostic only.
void wbWgpuAppendInstanceReport(void *instance, const char *path, long frame);

class OmWgpuRenderTarget {
public:
  // `width` and `height` are in pixels and must be > 0. Format is
  // fixed at RGBA8Unorm so the readback path can memcpy 4 bytes per
  // pixel into the caller's buffer without conversion.
  OmWgpuRenderTarget(OmVulkanBackend *backend, uint32_t width, uint32_t height);
  ~OmWgpuRenderTarget();

  bool isUsable() const { return mUsable; }
  // The 1x scene texture view (the frame every post-pass ends in) — sampled by the window-swap
  // present path (OmWgpuSurface::presentTexture).
  // W4a: when drawOverlayLines ran THIS frame it published its own output here instead. That
  // output is a private copy, deliberately NOT one of the TAA ping-pong buffers: TAA's history
  // for frame N+1 is exactly the texture this used to return, so painting overlay lines into it
  // would feed them back through the temporal resolve and smear them. mOverlayActiveView is
  // reset to null at the top of every scene render, so a frame that draws no overlay returns
  // byte-identically to what it returned before W4a.
  void *sceneTextureView() const {
    return mOverlayActiveView ? mOverlayActiveView : (mTaaMvActiveView ? mTaaMvActiveView : mView);
  }
  uint32_t width() const { return mWidth; }
  uint32_t height() const { return mHeight; }

  // T1.2 CSM sub-step 3b probe: force-build the shadow-receiving lit pipeline so
  // the driver naga-COMPILES kSolidLitShadow (WGSL is only validated at pipeline
  // creation). Returns true iff the pipeline built. A headless way to verify the
  // 3a shader + 3-entry bind-group layout are valid before the full two-pass
  // shadow wiring (3c) exists. No render, no state change beyond caching the
  // pipeline.
  bool probeShadowPipeline() { return ensureSceneShadowPipeline(); }
  // R4 lighting rung 1: build the textured+shadowed pipeline (naga-validates kSolidLitTexturedShadow
  // + its 9-entry layout). No render — just the pipeline build, for OMNISIM_PROBE_TEXSHADOW.
  bool probeTexturedShadowPipeline() { return ensureTexturedShadowPipeline(); }

  // Clear the target to the given color and read the result back
  // into `rgba8` (must point to at least width*height*4 bytes).
  // Returns true on success, false on any wgpu failure or if the
  // target isn't usable. The readback uses BufferUsage_MapRead +
  // wgpuBufferMapAsync; we synchronously poll via the device queue.
  bool clearAndRead(const OmWgpuClearColor &color, void *rgba8);

  // R3.4-step-1: clear + draw the three-vertex OmWgpuShaders::
  // kTriangleClipSpace, then read back. The render pipeline +
  // shader module are lazy-built on first call and cached on the
  // target instance.
  //
  // Returns true if every wgpu call succeeded. Pixels outside the
  // triangle hold `clearColor`; pixels inside hold the vertex-
  // interpolated barycentric color.
  bool clearAndDrawTriangle(const OmWgpuClearColor &clearColor, void *rgba8);

  // R3.4-step-2: clear + draw a vertex-buffer-fed triangle through
  // the production pos3+norm3+uv2 (32 byte stride) WGSL pipeline.
  // The caller supplies the GPU-side vertex buffer (obtained from
  // OmWgpuMeshCache::acquire) + the matching index buffer; this
  // call exercises the cache + vertex-buffer binding end-to-end.
  bool clearAndDrawMesh(const OmWgpuClearColor &clearColor,
                        void *vertexBuffer, void *indexBuffer, uint32_t indexCount,
                        void *rgba8);

  // R3.4-step-3: same as clearAndDrawMesh but the WGSL transforms
  // each vertex by a 4x4 view-projection matrix taken from a
  // uniform buffer bound at @group(0) @binding(0). `viewProj16` is
  // 16 floats (row-major or column-major — see WGSL spec note in
  // the .cpp).  Adds the bind-group + uniform-buffer infrastructure
  // R3.5 (texture bridge) + the real Phong/PBR port will use.
  bool clearAndDrawMeshMVP(const OmWgpuClearColor &clearColor,
                           const float *viewProj16,
                           void *vertexBuffer, void *indexBuffer, uint32_t indexCount,
                           void *rgba8);

  // R3.5: same as clearAndDrawMeshMVP but the fragment samples a
  // texture (`WGPUTextureView`, typically from OmWgpuTextureCache)
  // via a default linear-clamp sampler the RT owns. Bind group has
  // 3 entries: uniform (0), texture (1), sampler (2).
  bool clearAndDrawMeshTextured(const OmWgpuClearColor &clearColor,
                                const float *viewProj16,
                                void *textureView,
                                void *vertexBuffer, void *indexBuffer, uint32_t indexCount,
                                void *rgba8);

  // R3.7: instanced draw reading body transforms from a wgpu
  // storage buffer. Each body is a vec4<f32> (.xyz = translation;
  // .w padding to keep the 16-byte alignment storage-buffer
  // layouts demand). `bodyOffsetsXyz0` is `bodyCount` consecutive
  // vec4s = 16*bodyCount bytes; uploaded to a USAGE_STORAGE buffer
  // and bound at @group(0) @binding(1). Draws `bodyCount`
  // instances of the supplied 3-vertex mesh, each at its body's
  // translation. This is the storage-buffer half of the Newton-
  // interop wire shape — the Phase D + R3.7 production path
  // replaces this upload with a zero-copy mapping of Newton's
  // own body-state ring buffer.
  bool clearAndDrawInstanced(const OmWgpuClearColor &clearColor,
                             const float *viewProj16,
                             const float *bodyOffsetsXyz0, uint32_t bodyCount,
                             void *vertexBuffer, void *indexBuffer, uint32_t indexCount,
                             void *rgba8);

  // R3.4-step-4: scene pass. One draw call per Solid through the
  // `kSolidLit` WGSL pipeline (Lambertian shading from
  // baseColor + a directional light).
  //
  // `viewProj16`: 16 floats, column-major mat4 (WGSL convention).
  // `lightDirAmbient4`: 4 floats — light direction xyz + ambient
  //   scalar [0..1] in w. Pass {0, 0, -1, 0.2} for a downward
  //   light with 20% ambient.
  //
  // `draws[]` is `numDraws` consecutive OmWgpuSolidDraw structs.
  // Each draw uses its own model matrix + baseColor uniform; the
  // uniform buffer is grown on demand to fit `numDraws *
  // alignedUniformStride` bytes (the wgpu spec requires per-draw
  // uniform-buffer-offset alignment of 256 bytes; the actual
  // useful payload per draw is 192 B with the rest as padding).
  // depthMode=true (R5 RangeFinder): render with the distance shader so the
  // readback is LINEAR view-space distance (grayscale, normalized by farPlane)
  // rather than lit color. Default false keeps the lit path byte-identical.
  // agxMode=true (T1.1): tonemap the lit colour through AgX (kSolidLitAgX)
  // instead of plain Lambertian. Ignored when depthMode is set (depth wins).
  // Default false → byte-identical lit path.
  // cameraWorldPos3 (optional, T1.1 specular foundation): the camera's world
  // position (3 floats). When non-null it is packed into the Scene uniform's
  // pad0.yzw slack so a future specular term in kSolidLitAgX can compute the
  // view vector. No shader reads it yet, and null (the default) writes nothing —
  // so every current caller + the golden image stay byte-identical.
  // pickMode (R4 step-3c-A.1): render flat via kSolidPick (no lighting) so each
  // draw's baseColor — set by the caller to an encoded integer ID — round-trips
  // through the RGBA8 readback unchanged. The caller decodes the texel under the
  // cursor to identify the picked draw. Additive: default false keeps every
  // existing caller (the Camera path) byte-identical.
  bool clearAndDrawScene(const OmWgpuClearColor &clearColor,
                         const float *viewProj16, const float *lightDirAmbient4,
                         const struct OmWgpuSolidDraw *draws, uint32_t numDraws,
                         void *rgba8, bool depthMode = false, float farPlane = 1.0f,
                         bool agxMode = false, const float *cameraWorldPos3 = nullptr,
                         bool pickMode = false, bool srgbEncode = true);

  // R4 step-3c-A: render colored line segments (LineList) into the target. The
  // foundation for the optional renderings (bounding objects, joint axes, normals,
  // contact points, frustums). `lineVerts` is `vertexCount` vertices in the standard
  // pos3+normal3+uv2 layout (stride 32; normal/uv ignored — pad with zeros), two
  // consecutive vertices per segment. `color4` is the flat RGBA line colour, applied
  // via the per-draw uniform. When `loadExisting` is true the colour+depth of the
  // target are PRESERVED (loadOp=Load) so the lines overlay a scene already rendered
  // by clearAndDrawScene; otherwise the target is cleared to `clearColor` first.
  // Reads the RGBA8 result back into `rgba8`. Depth-tested against the scene so
  // overlay lines are occluded by geometry. Additive — touches no existing path.
  // depthTest=false draws the lines ALWAYS-ON-TOP (depthCompare Always, no depth write)
  // — for markers (joint axes, COM, contact) that sit inside geometry and must stay
  // visible. depthTest=true (default) occludes them against the scene (for wireframes
  // that should read as 3D, e.g. bounding objects).
  bool clearAndDrawLines(const OmWgpuClearColor &clearColor, const float *viewProj16,
                         const float *color4, const float *lineVerts, uint32_t vertexCount,
                         void *rgba8, bool loadExisting = false, bool depthTest = true);

  // R4 step-3c-A: composite a flat RGBA full-screen overlay (alpha-blended) over the
  // target's current contents — the loading/black/status overlays. `color4` is premul-
  // free straight RGBA; alpha blends src over dst. Reads the result back into `rgba8`.
  bool drawFullScreenOverlay(const float *color4, void *rgba8);

  // R4 step-3c-A: composite `textureView` into the NDC sub-rect ndcRect={x0,y0,x1,y1} over the
  // target's current contents (loadOp=Load), then read back into rgba8. The device-output inset
  // primitive (camera/range-finder/display image as a corner HUD). Unlit, no depth. Additive.
  bool drawTexturedInset(void *textureView, const float ndcRect[4], void *rgba8);

  // R4 material fidelity self-test (headless, for OMNISIM_PROBE_TEX): build the textured-
  // lit pipeline (naga-validates the 6-binding WGSL), render a known full-screen quad with
  // a solid albedo texture, and read back the centre pixel — once with the default-white
  // roughness slot and once with a black roughness texture. Writes the two centre pixels
  // into albedoOut[3] (white-roughness render) and specOut[3] (black-roughness render).
  // A black roughnessMap forces effectiveRoughness→0 → max specular, so a head-on highlight
  // makes specOut strictly brighter than albedoOut — proving per-pixel roughness modulates.
  // metalOut[3] gets a third render: metalness=1 (+ a smooth black-roughness highlight). A
  // metal takes NO diffuse and tints its specular by the albedo, so metalOut comes out
  // albedo-coloured (blue-dominant here), NOT the white of the dielectric specOut render.
  // normalOut[3] gets a fourth render: the same smooth dielectric highlight but with a TILTED
  // tangent-space normal map — the perturbed normal no longer faces the half-vector head-on,
  // so the sharp highlight collapses → normalOut is markedly dimmer than specOut. Proves the
  // normalMap perturbs the shading normal. Returns false if the pipeline can't be built.
  bool selfTestTextured(unsigned char albedoOut[3], unsigned char specOut[3], unsigned char metalOut[3],
                        unsigned char normalOut[3]);

  // R4 device-inset self-test (headless, for OMNISIM_PROBE_INSET): clear the target to navy,
  // composite a known RED 1×1 texture into a top-right NDC sub-rect via drawTexturedInset, then
  // read back one pixel INSIDE the inset (→ red) and one OUTSIDE (→ navy). Proves the screen-
  // space textured-quad compositing primitive (the device-output inset mechanism) end-to-end.
  bool selfTestInset(unsigned char insideOut[3], unsigned char outsideOut[3]);

  // R4 3c-B un-gate leak-hunt: append wgpu-native's global resource-registry counts
  // (wgpuGenerateReport) to `path` (append mode; one line tagged with `frame`, each resource
  // type as alloc/kept/released). Used by the OMNISIM_PROBE_SOAK sustained-render probe to reveal
  // which resource TYPE accumulates before the per-frame-render VRAM OOM (the main-view un-gate
  // blocker) — or to prove the registry stays flat (leak is below the registry). Diagnostic only:
  // no render, no state change; inert on non-wgpu builds and when `path` is null.
  void appendResourceReport(const char *path, long frame) const;

  // T1.2 CSM sub-step 3c: two-pass shadowed render. Pass 1 renders the scene
  // depth (clip.z) from the light's POV (`lightViewProj16`, the ortho light-space
  // matrix from buildOrthoLightViewProj) into the GPU-resident sampleable shadow
  // map; pass 2 renders the lit scene into the RGBA8 color target with
  // kSolidLitShadow, sampling that map to darken occluded fragments by
  // `shadowStrength` [0,1]. `depthBias` offsets the comparison to fight acne.
  // Reads back the lit RGBA8 into `rgba8`. Separate from clearAndDrawScene (which
  // stays byte-identical); driven by OmCamera OMNISIM_CAMERA_SHADOW=1.
  bool clearAndDrawSceneShadowed(const OmWgpuClearColor &clearColor,
                                 const float *viewProj16, const float *lightViewProj16,
                                 const float *lightDirAmbient4,
                                 const struct OmWgpuSolidDraw *draws, uint32_t numDraws,
                                 float shadowStrength, float depthBias, void *rgba8);

  // T1.2 CSM (multi-cascade): the N-cascade generalisation of clearAndDrawSceneShadowed.
  // Renders `cascadeCount` light clip-depth layers into an R32Float texture_2d_array (one
  // light view-projection per cascade, as produced by OmWgpuSceneRenderer::
  // buildCascadeLightViewProjs) then ONE lit pass with kSolidLitCsm, which selects a cascade
  // per-fragment by linear view depth and 3x3-PCFs that array layer. `cascadeLightViewProjs`
  // is `cascadeCount*16` column-major floats (cascade-major); `cascadeSplitsFar4` is the 4 far
  // view-depth boundaries (cascades 0..3; zero-fill the unused tail); `cascadeCount` ∈ [1,4].
  // strength 0 → byte-identical to kSolidLit. Reads back the lit RGBA8 into `rgba8`. Self-
  // contained (clearAndDrawScene stays byte-identical). The on-GPU half of T1.2 CSM — the
  // engine translation of the GPU-proven docs/developer/csm_render_prototype.py design.
  bool clearAndDrawSceneCsm(const OmWgpuClearColor &clearColor, const float *viewProj16,
                            const float *cascadeLightViewProjs, const float *cascadeSplitsFar4,
                            uint32_t cascadeCount, const float *lightDirAmbient4,
                            const struct OmWgpuSolidDraw *draws, uint32_t numDraws,
                            float shadowStrength, float depthBias, void *rgba8);

  // T1.2 CSM render-layer self-test: build the floor+caster geometry, render it through
  // clearAndDrawSceneCsm at strength 0 then 0.8, and report the floor pixel under the caster
  // (`shadowedOut`, expected to darken), a floor pixel to the side (`litSideOut`, stays lit), and
  // the strength-0 reference for the shadow point (`shadowOffOut`). The camera view-proj, the
  // per-cascade light view-projs, the far split boundaries, the cascade count, and the light
  // dir+ambient are supplied by the caller (OmWgpuSceneRenderer::csmSelfTest — the nodes layer
  // owns OmMatrix4 + the cascade fit), so this stays render-self-contained. `*cascadeSelected`
  // (optional, may be null) returns the cascade the shadow point's view depth routes to (≥1).
  // Building the pipeline naga-validates kSolidLitCsm + its 3-entry layout in-engine. GUI-free.
  bool selfTestCsm(const float *camViewProj16, const float *cascadeLightViewProjs,
                   const float *cascadeSplitsFar4, uint32_t cascadeCount,
                   const float *lightDirAmbient4, unsigned char shadowedOut[3],
                   unsigned char litSideOut[3], unsigned char shadowOffOut[3], int *cascadeSelected);

  // T1.4 TAA temporal-resolve post pass: blend a current-frame color view (`curView`) with a
  // previous resolved-frame view (`histView`) into the RGBA8 target, then read back into `rgba8`.
  // `motionPx2` is the screen-space motion (px) used to reproject history; `feedback` is the
  // history weight (~0.90); `taaEnabled` false → `curView` passthrough (no-op); `clampEnabled`
  // toggles the 3x3 neighborhood-AABB ghost suppressor. The fullscreen resolve from taa-preview.html.
  bool resolveTaa(void *curView, void *histView, const float motionPx2[2], float feedback,
                  bool taaEnabled, bool clampEnabled, void *rgba8);

  // T1.4 TAA headless self-test (drives OMNISIM_PROBE_TAA). Runs the resolve over known uniform
  // current/history textures to exercise each branch: the feedback blend (`blendOut`), the
  // neighborhood clamp suppressing an out-of-range history (`clampOnOut` vs `clampOffOut`), the
  // TAA-off passthrough (`taaOffOut`), and off-screen history rejection (`offscreenOut`). Each
  // output is a center-pixel RGB. Building the pipeline naga-validates kTaaResolve in-engine.
  bool selfTestTaa(unsigned char blendOut[3], unsigned char clampOnOut[3],
                   unsigned char clampOffOut[3], unsigned char taaOffOut[3],
                   unsigned char offscreenOut[3]);

  // T1.4 TAA ping-pong history accumulator. Resolves `curView` against the running GPU-resident
  // history into the OTHER history buffer, then swaps — so consecutive calls accumulate a temporal
  // exponential moving average (the supersampling that makes TAA converge). `reset` true seeds the
  // history from `curView` (taa-off path) for the first frame of a sequence. `feedback` is the
  // history weight; `motionPx2` reprojects history; `clampEnabled` toggles the ghost suppressor.
  // `rgba8OrNull` (optional): if non-null, the just-written history buffer is read back into it;
  // pass null to accumulate without a CPU round-trip. The GPU-resident half of T1.4's history buffer.
  bool accumulateTaa(void *curView, bool reset, float feedback, const float motionPx2[2],
                     bool clampEnabled, void *rgba8OrNull);

  // T1.4 TAA history convergence self-test (drives OMNISIM_PROBE_TAA's accumulate checks). Seeds the
  // history black, then accumulates a white frame repeatedly with feedback 0.9 + no motion: the
  // exponential moving average must rise monotonically from ~black toward ~white. Returns the value
  // right after seeding (`afterResetOut`, ~0), after a few frames (`afterFewOut`, mid), and after
  // many (`afterManyOut`, ~converged-to-white) — each a center-pixel RGB. GUI-free.
  bool selfTestTaaAccum(unsigned char afterResetOut[3], unsigned char afterFewOut[3],
                        unsigned char afterManyOut[3]);

  // T1.3 fog resolve post pass: blend a scene-colour view (`sceneView`) toward `fogColor3` by the
  // metric view distance in `depthView` (R32Float, view-space distance in metres) via an exponential
  // transmittance 1-exp(-density*dist). Writes the fogged frame to the RGBA8 target + reads it back
  // into `rgba8`. `enabled` false (or `density` 0) → scene passthrough (a safe no-op). The analytic
  // foundation of T1.3 (full volumetric froxel scattering is a later tier).
  bool resolveFog(void *sceneView, void *depthView, const float fogColor3[3], float density,
                  bool enabled, void *rgba8);

  // T1.3 fog headless self-test (drives OMNISIM_PROBE_FOG). Renders a white scene at a NEAR vs a FAR
  // uniform depth and confirms the far one is heavily fog-coloured while the near one stays ~scene,
  // plus the fog-off passthrough (`nearOut`/`farOut`/`offOut`, each a center-pixel RGB). Building the
  // pipeline naga-validates kFogResolve in-engine. GUI-free.
  bool selfTestFog(unsigned char nearOut[3], unsigned char farOut[3], unsigned char offOut[3]);

  // R4 lighting rung 1: TWO-PASS textured + shadowed render (light clip-depth → kSolidLitTexturedShadow
  // lit pass). The full material path (albedo/roughness/metalness/normal + GGX + sRGB) modulated by a
  // PCF cast-shadow term. cameraWorldPos3 feeds the GGX view vector. Reads back into rgba8. Additive.
  // hemiSky4/hemiGround4 (rgb + intensity in sky.w) + worldUp3 enable a hemisphere-IBL ambient
  // (matches OmniSimSky's directional sky fill); pass nullptr for all three → flat-ambient fallback
  // (byte-identical to the pre-hemisphere path).
  // asyncReadback (live main view): pipeline the GPU→CPU readback across frames — submit this
  // frame's render+copy, then deliver the PREVIOUS frame's pixels (ping-pong readback buffers), so
  // the CPU never waits for the GPU it just fed. One frame of display latency; rgba8 is left
  // untouched on the very first call (no previous frame yet — still returns true). The default
  // false keeps the synchronous same-frame contract for screenshots/probes/parity gates.
  // skyUniform96 (optional, 24 floats = SkyU): when non-null and the MSAA path is active, the
  // analytic atmospheric dome (kSkyAtmosphere) is drawn behind the scene instead of the flat
  // clear colour — the wgpu counterpart of Background.atmosphericSky, incl. day-night.
  // directScale (0 = legacy off): scales the DIRECT light term only; the day-night dimmer.
  // extraLights (optional): unshadowed additional scene lights (PointLight/SpotLight/extra
  // DirectionalLights), 16 floats per light matching the WGSL ExtraLight layout
  // {posType, colorRad, atten, spotDir}; at most 8 are consumed. Null/0 = single-sun (prior output).
  // outputTransfer (OmWgpuOutputTransfer): the display encode the HDR finishing pass applies.
  // OM_WGPU_XFER_VIEW (the default) is the main view's AgX + camera pass, unchanged.
  // OM_WGPU_XFER_WREN_SENSOR selects WREN's hdr_resolve curve and suppresses vignette/grain/
  // auto-exposure -- read the enum's comment before using it; it is the Camera-DEVICE contract.
  // It only reaches the shader on the HDR arm (agxExposure > 0 AND the MSAA/RGBA16F pipelines
  // built); without that the scene shader's own LDR pow(1/2.2) encode stands, exposure-free.
  bool clearAndDrawSceneTexturedShadowed(const OmWgpuClearColor &clearColor, const float *viewProj16,
                                         const float *lightViewProj16, const float *lightDirAmbient4,
                                         const struct OmWgpuSolidDraw *draws, uint32_t numDraws,
                                         float shadowStrength, float depthBias,
                                         const float *cameraWorldPos3, const float *hemiSky4,
                                         const float *hemiGround4, const float *worldUp3, void *rgba8,
                                         bool asyncReadback = false, const float *skyUniform96 = nullptr,
                                         float directScale = 0.0f, const float *fogParams4 = nullptr,
                                         float bloomStrength = 0.0f, float agxExposure = 0.0f,
                                         float ssaoStrength = 0.0f, bool reversedZ = false,
                                         const float *extraLights = nullptr, uint32_t extraLightCount = 0,
                                         float ssaoRadiusScale = 1.0f, const float *sunEnergy3 = nullptr,
                                         float ssaoNearZ = 0.05f,
                                         const float *cascadeLightViewProjs = nullptr,
                                         const float *cascadeSplitsFar4 = nullptr,
                                         uint32_t cascadeCount = 0,
                                         float bloomHdrThreshold = 0.0f,
                                         const float *skyScatter24 = nullptr,
                                         const float *iblSky8 = nullptr,
                                         int outputTransfer = OM_WGPU_XFER_VIEW);

  // R4 lighting convergence: the MULTI-CASCADE generalisation of clearAndDrawSceneTexturedShadowed —
  // the full material path (albedo/roughness/metalness/normal + GGX + hemisphere ambient) with N-cascade
  // shadows. Renders `cascadeCount` light clip-depth layers into the R32Float texture_2d_array, then one
  // kSolidLitTexturedCsm lit pass that selects a cascade per-fragment by view depth. `cascadeLightViewProjs`
  // is `cascadeCount*16` column-major floats; `cascadeSplitsFar4` the 4 far view-depth boundaries. Same
  // material/hemisphere inputs as clearAndDrawSceneTexturedShadowed. The render the main-view default wants.
  bool clearAndDrawSceneTexturedCsm(const OmWgpuClearColor &clearColor, const float *viewProj16,
                                    const float *cascadeLightViewProjs, const float *cascadeSplitsFar4,
                                    uint32_t cascadeCount, const float *lightDirAmbient4,
                                    const struct OmWgpuSolidDraw *draws, uint32_t numDraws,
                                    float shadowStrength, float depthBias, const float *cameraWorldPos3,
                                    const float *hemiSky4, const float *hemiGround4,
                                    const float *worldUp3, void *rgba8);

  // R5b: scene depth pass writing REAL view-space distance in meters to an
  // R32Float color target, read back as `width*height` consecutive float32
  // values into `outMeters` (must hold at least width*height floats).
  //
  // Unlike clearAndDrawScene(depthMode=true) — which normalizes distance to an
  // 8-bit grayscale proof — this returns metric depth at full float precision,
  // the shape a real RangeFinder device node consumes. `farClear` is written to
  // pixels no geometry covers (the RangeFinder's "nothing hit" / maxRange
  // value). Reuses the lit path's bind-group layout + depth attachment + the
  // RGBA8 readback buffer (R32Float is 4 B/pixel, identical row stride). The
  // lit + grayscale paths are byte-untouched.
  // clipDepth=true (T1.2 CSM): write NDC depth clip.z (kSolidClipDepthF32) into
  // the R32Float target instead of clip.w view distance — correct under an
  // orthographic light projection for a shadow map. Default false keeps the
  // RangeFinder/F32 metric-depth behaviour byte-identical.
  bool clearAndDrawSceneDepthF32(float farClear, const float *viewProj16,
                                 const struct OmWgpuSolidDraw *draws, uint32_t numDraws,
                                 float *outMeters, bool clipDepth = false);

  // OmniLight: upload a baked probe volume (RGBA16F texels, dims nx x ny x nz*4 z-slabs).
  // Passing nullptr reverts to the black default (flag off -> the shader keeps the hemisphere
  // ambient bit-exactly). origin3/invExtent3 feed the shader's world -> volume mapping.
  bool setOmniLightVolume(const uint16_t *texels, int nx, int ny, int nz, const float *origin3,
                          const float *invExtent3);
  // Fade-in blend for a freshly landed bake (0 = hemisphere, 1 = full OmniLight).
  void setOmniLightBlend(float b) { mOmniMisc4[0] = b; }
  // Upload the traced specular cubemap (RGBA16F, 3 mips: size, size/4, size/16).
  bool setOmniLightCube(const uint16_t *texels, int size, const float *center3,
                        const float *aabbMin3, const float *aabbMax3);
  // Draw the OmniLight loading overlay (sun dot + progress bar) over the tonemapped frame in
  // mView. Present-path only; screenshots/dumps stay clean (their readback happens earlier).
  bool drawOmniProgress(float progress01, float pulseT);

  // W4a (WREN retirement): composite world-space line overlays over the FINISHED frame.
  //
  // Pass placement: LAST — after opaque/sky/SSR/volumetrics/transparent/TAA/tonemap, i.e. the
  // same slot drawOmniProgress uses, and for the same reason: everything upstream is a
  // radiance pipeline (HDR, AgX, bloom, temporal reprojection) and a debug wireframe is not
  // radiance. Drawing here means an overlay colour lands in the frame exactly as authored
  // rather than being tonemapped, bloomed and smeared.
  //
  // Consequence, disclosed rather than hidden: the lines are ALWAYS-ON-TOP (depthCompare
  // Always, no depth write). The scene's depth for this frame lives in a 4x-MSAA,
  // reversed-Z Depth32Float attachment; a 1x post-tonemap pass cannot attach it (sample
  // counts must match), and resolving/copying it would cost a full-frame pass on every frame
  // an overlay is on. So bounding-object wireframes read through geometry instead of being
  // occluded by it. Depth-correct occlusion needs the lines drawn inside pass 2 (MSAA + HDR +
  // reversed-Z pipeline variants) and is deliberately left to a follow-up.
  //
  // The frame is first COPIED into a private RGBA8 output (see sceneTextureView()) so overlay
  // pixels never re-enter the TAA history. Returns false — leaving sceneTextureView()
  // untouched, so the caller still presents a correct frame — if the target is unusable, the
  // line pipeline will not build, or every batch is empty.
  bool drawOverlayLines(const float *viewProj16, const OmWgpuLineBatch *batches,
                        uint32_t batchCount);

  // P7 (WREN retirement): composite screen-space HUD quads over the FINISHED frame -- device
  // insets, their frames, and supervisor labels.
  //
  // Same dead-last slot, same private-copy discipline and the same reasons as drawOverlayLines
  // above: after tonemap so a device inset shows the bytes the controller read rather than an
  // AgX-graded version of them, and into a private RGBA8 output so HUD pixels never re-enter the
  // TAA history and smear. When drawOverlayLines already ran THIS frame the copy is skipped and
  // the quads composite on top of the lines, so the two passes share one output and one present.
  //
  // Returns false -- leaving sceneTextureView() exactly as it found it -- when the target is
  // unusable, the pipeline will not build, or every quad is degenerate. At most kMaxHudQuads
  // (64) quads are drawn per frame; the rest are ignored rather than growing an unbounded buffer.
  bool drawOverlayQuads(const OmWgpuHudQuad *quads, uint32_t quadCount);

  // P7/P8: read back the OVERLAY output -- the texture drawOverlayLines / drawOverlayQuads wrote
  // and the present path shows. Every other readback in this class happens INSIDE the scene
  // render, upstream of both overlay passes, so this is the only way to measure an overlay pixel.
  // width()*height()*4 bytes, RGBA8. False when no overlay ran this frame, so it can never hand
  // back a pre-overlay frame that would read as evidence.
  // `preOverlay` reads the frame ONE PASS EARLIER -- the same pixels as they would be with the
  // overlay passes switched off. That is the control arm: a HUD/gizmo measurement that reports
  // both arms proves the instrument can go red rather than asserting it.
  bool readbackOverlayOutput(void *rgba8, bool preOverlay = false);

  // R5d: Lidar RADIAL-range pass. Like clearAndDrawSceneDepthF32 but outputs
  // euclidean distance from the camera origin (length of the view-space
  // position) instead of planar depth — what a Lidar ray actually measures, so
  // off-axis pixels read larger than the planar value. Needs the view matrix
  // (`view16`, column-major) on top of viewProj. Renders into the same R32Float
  // target; reads back width*height float metres. `farClear` seeds pixels no
  // geometry covers. Additive — the depth/lit/grayscale paths are untouched.
  bool clearAndDrawSceneRangeF32(float farClear, const float *viewProj16, const float *view16,
                                 const struct OmWgpuSolidDraw *draws, uint32_t numDraws,
                                 float *outMeters);

private:
  OmVulkanBackend *mBackend;  // non-owning
  uint32_t mWidth;
  uint32_t mHeight;
  bool mUsable = false;

  // Opaque wgpu handles.
  void *mTexture = nullptr;       // WGPUTexture
  void *mView = nullptr;          // WGPUTextureView
  void *mReadbackBuffer = nullptr; // WGPUBuffer
  size_t mReadbackBufferSize = 0;
  // Async-readback pipelining (asyncReadback=true): second buffer ping-ponged with mReadbackBuffer
  // so frame N's copy lands in one while frame N-1's map completes on the other.
  void *mReadbackBufferB = nullptr;  // WGPUBuffer, lazily created
  bool mAsyncPending = false;        // a mapAsync is outstanding on buffers[1 - mAsyncWhich]
  int mAsyncWhich = 0;               // which buffer the NEXT copy targets
  struct { bool done = false; bool ok = false; } mAsyncCap;  // callback state for the pending map

  // R3.4-step-1: triangle pipeline. Lazy-built on first call to
  // clearAndDrawTriangle, cached for subsequent calls.
  void *mTriangleShaderModule = nullptr;  // WGPUShaderModule
  void *mTrianglePipeline = nullptr;      // WGPURenderPipeline

  // R3.4-step-2: mesh pipeline. Same lazy + cached pattern.
  void *mMeshShaderModule = nullptr;  // WGPUShaderModule
  void *mMeshPipeline = nullptr;      // WGPURenderPipeline

  // R3.4-step-3: mesh-with-MVP-uniform pipeline + uniform-buffer +
  // bind group. Lazy-built same as the others.
  void *mMvpShaderModule = nullptr;       // WGPUShaderModule
  void *mMvpPipeline = nullptr;           // WGPURenderPipeline
  void *mMvpUniformBuffer = nullptr;      // WGPUBuffer (16 floats)
  void *mMvpBindGroupLayout = nullptr;    // WGPUBindGroupLayout
  void *mMvpBindGroup = nullptr;          // WGPUBindGroup

  // R3.5: textured-mesh pipeline. Uniform buffer is reused from the
  // MVP path (same shape, same content); bind-group layout is
  // wider (3 entries: uniform, texture, sampler) so it needs its
  // own pipeline. Bind group is rebuilt per textureView so we
  // don't cache it; the sampler is process-lifetime.
  void *mTexShaderModule = nullptr;     // WGPUShaderModule
  void *mTexPipeline = nullptr;         // WGPURenderPipeline
  void *mTexBindGroupLayout = nullptr;  // WGPUBindGroupLayout
  void *mTexSampler = nullptr;          // WGPUSampler

  // R3.7: instanced-from-storage-buffer pipeline. Reuses the MVP
  // uniform buffer. Storage buffer is grown to fit
  // `bodyCount` if needed; bind group rebuilt per call.
  void *mInstShaderModule = nullptr;     // WGPUShaderModule
  void *mInstPipeline = nullptr;         // WGPURenderPipeline
  void *mInstBindGroupLayout = nullptr;  // WGPUBindGroupLayout
  void *mInstStorageBuffer = nullptr;    // WGPUBuffer (grown on demand)
  size_t mInstStorageBufferSize = 0;

  // R3.4-step-4: scene-pass pipeline. One big uniform buffer holds
  // `numDraws` consecutive Scene blocks at 256-byte-aligned offsets.
  // Bind group is rebuilt with a dynamic offset per draw — single
  // pipeline + single layout regardless of solid count.
  // Also: a depth attachment so multiple solids occlude correctly.
  void *mScnShaderModule = nullptr;     // WGPUShaderModule
  void *mScnPipeline = nullptr;         // WGPURenderPipeline
  void *mScnBindGroupLayout = nullptr;  // WGPUBindGroupLayout
  void *mScnUniformBuffer = nullptr;    // WGPUBuffer (grown on demand)
  size_t mScnUniformBufferSize = 0;
  void *mScnDepthTexture = nullptr;     // WGPUTexture (Depth24Plus)
  void *mScnDepthView = nullptr;        // WGPUTextureView
  // R5: depth/RangeFinder scene pipeline. Reuses mScnBindGroupLayout +
  // mScnDepthView (built by ensureScenePipeline); only the distance shader +
  // pipeline differ.
  void *mScnDepthShaderModule = nullptr;  // WGPUShaderModule
  void *mScnDepthPipeline = nullptr;      // WGPURenderPipeline (kSolidDistance)
  // R5b: F32 real-meters depth pipeline. Renders into its OWN R32Float color
  // texture (mDepthF32Texture); reuses mScnBindGroupLayout + mScnDepthView (the
  // Depth24Plus occlusion attachment) + mReadbackBuffer. Only the color target
  // format + fragment shader differ from the grayscale depth pipeline.
  void *mDepthF32Texture = nullptr;          // WGPUTexture (R32Float)
  void *mDepthF32View = nullptr;             // WGPUTextureView
  void *mScnDepthF32ShaderModule = nullptr;  // WGPUShaderModule
  void *mScnDepthF32Pipeline = nullptr;      // WGPURenderPipeline (kSolidDistanceF32)
  // T1.2 CSM: clip.z (NDC depth) pipeline for shadow-map depth. Reuses the
  // R32Float target + bind-group layout + depth attachment from the F32 depth
  // pipeline (ensureSceneDepthF32Pipeline runs first); only the fragment shader
  // differs (kSolidClipDepthF32 writes clip.z, correct under ortho light proj).
  void *mScnClipDepthShaderModule = nullptr;  // WGPUShaderModule
  void *mScnClipDepthPipeline = nullptr;      // WGPURenderPipeline (kSolidClipDepthF32)
  // R5d: Lidar radial-range pipeline. Reuses mScnBindGroupLayout + mScnDepthView
  // + the R32Float target (mDepthF32Texture, via ensureSceneDepthF32Pipeline);
  // only the kSolidRangeF32 shader + pipeline differ.
  void *mScnRangeF32ShaderModule = nullptr;  // WGPUShaderModule
  void *mScnRangeF32Pipeline = nullptr;      // WGPURenderPipeline (kSolidRangeF32)
  // T1.1: AgX-tonemapped scene pipeline. Identical to the lit pipeline (RGBA8 +
  // Depth24Plus, same vertex + bind-group layout); only the fragment shader
  // differs (kSolidLitAgX). Reuses mScnBindGroupLayout + mScnDepthView built by
  // ensureScenePipeline.
  void *mScnAgxShaderModule = nullptr;  // WGPUShaderModule
  void *mScnAgxPipeline = nullptr;      // WGPURenderPipeline (kSolidLitAgX)
  // R4 material fidelity: textured-lit pipeline (kSolidLitTextured). Its OWN 6-entry
  // bind-group layout (uniform@0 + albedo@1 + roughness@2 + metalness@3 + normal@4 + sampler@5)
  // — distinct from the 1-entry mScnBindGroupLayout. Used per-draw when a draw carries albedo.
  void *mScnTexShaderModule = nullptr;     // WGPUShaderModule
  void *mScnTexPipeline = nullptr;         // WGPURenderPipeline
  void *mScnTexBindGroupLayout = nullptr;  // WGPUBindGroupLayout (6: uniform+albedo+rough+metal+normal+samp)
  void *mScnTexSampler = nullptr;          // WGPUSampler (linear, repeat)
  // R4 material fidelity: 1×1 default textures bound when a draw carries an albedo map but
  // not the others, so the layout always has valid views and those draws stay byte-identical:
  // white roughness (factor unchanged), black metalness (dielectric), flat normal (no perturb).
  void *mScnDefaultWhiteTex = nullptr;     // WGPUTexture  (roughness 1.0)
  void *mScnDefaultWhiteView = nullptr;    // WGPUTextureView
  void *mScnDefaultBlackTex = nullptr;     // WGPUTexture  (metalness 0.0)
  void *mScnDefaultBlackView = nullptr;    // WGPUTextureView
  void *mScnDefaultNormalTex = nullptr;    // WGPUTexture  (128,128,255 → tangent +Z)
  void *mScnDefaultNormalView = nullptr;   // WGPUTextureView
  void *mScnDefaultPenTex = nullptr;       // WGPUTexture  (0,0,0,0 -> pen alpha 0 -> mix is a no-op)
  void *mScnDefaultPenView = nullptr;      // WGPUTextureView
  // R4 step-3c-A.1: flat picking pipeline (kSolidPick). Reuses mScnBindGroupLayout
  // + mScnDepthView + the RGBA8 target + mReadbackBuffer (ensureScenePipeline runs
  // first); only the fragment shader differs (emits baseColor unshaded).
  void *mPickShaderModule = nullptr;  // WGPUShaderModule
  void *mPickPipeline = nullptr;      // WGPURenderPipeline (kSolidPick)
  // R4 step-3c-A: line/wireframe pipeline. Reuses the kSolidPick shader (transform
  // by viewProj*model, flat baseColor out) + mScnBindGroupLayout, but with LineList
  // topology — the foundation for the optional renderings (bounding objects, joint
  // axes, normals, contact points, frustums) that draw as colored line segments.
  void *mLineShaderModule = nullptr;  // WGPUShaderModule (kSolidPick, reused)
  void *mLinePipeline = nullptr;      // WGPURenderPipeline (LineList, depth-tested)
  void *mLineNoDepthPipeline = nullptr;  // WGPURenderPipeline (LineList, always-on-top)
  // R4 step-3c-A: full-screen overlay pipeline (kFullScreenOverlay) — alpha-blended
  // flat-colour full-screen triangle, no depth. For loading/black/status overlays.
  void *mOverlayShaderModule = nullptr;
  void *mOverlayPipeline = nullptr;
  void *mOverlayBindGroupLayout = nullptr;
  void *mOverlayUniformBuffer = nullptr;
  // R4 step-3c-A: screen-space textured-quad pipeline (kTexturedQuad) — a texture drawn into
  // an NDC sub-rect, the compositing primitive for device-output insets. 3-entry layout
  // (rect uniform@0 + texture@1 + sampler@2), no depth, alpha-blended over current contents.
  void *mTexQuadShaderModule = nullptr;
  void *mTexQuadPipeline = nullptr;
  void *mTexQuadBindGroupLayout = nullptr;
  void *mTexQuadSampler = nullptr;
  void *mTexQuadUniformBuffer = nullptr;
  // T1.2 CSM sub-step 3b: shadow-receiving lit pipeline (kSolidLitShadow). Has
  // its OWN 3-entry bind-group layout (ShadowScene uniform @0 + shadow texture
  // @1 + sampler @2) — distinct from the 1-entry mScnBindGroupLayout the other
  // scene pipelines share. Built lazily by ensureSceneShadowPipeline; the actual
  // two-pass wiring (light-depth → sampleable texture → this pipeline) is 3c.
  void *mScnShadowShaderModule = nullptr;   // WGPUShaderModule
  void *mScnShadowPipeline = nullptr;       // WGPURenderPipeline (kSolidLitShadow)
  void *mScnShadowBindGroupLayout = nullptr;  // WGPUBindGroupLayout (3-entry)
  void *mScnShadowSampler = nullptr;        // WGPUSampler
  // T1.2 CSM sub-step 3c: GPU-resident SAMPLEABLE shadow map. Distinct from
  // mDepthF32Texture (which is RenderAttachment|CopySrc for readback): this one
  // is RenderAttachment|TextureBinding so the light-depth pass renders into it
  // and the lit pass samples it — no CPU round-trip. Lazily built by
  // ensureShadowMapTexture(); the two-pass orchestration that uses it is 3c-ii.
  void *mShadowMapTexture = nullptr;  // WGPUTexture (R32Float, sampleable)
  void *mShadowMapView = nullptr;     // WGPUTextureView
  void *mShadowMapDepthTexture = nullptr;  // WGPUTexture (Depth24Plus, light-pass occlusion)
  void *mShadowMapDepthView = nullptr;     // WGPUTextureView
  // T1.2 CSM 3c: pass-2 ShadowScene uniform buffer (240 B payload, 256 B stride
  // per draw) — separate from mScnUniformBuffer so the two passes don't alias.
  void *mShadowUniformBuffer = nullptr;    // WGPUBuffer (grown on demand)
  // Texshadow-owned Scene slot buffer, DELTA-staged: a persistent host mirror is composed per
  // draw and memcmp'd; only changed slots upload (coalesced ranges). Deliberately not
  // mScnUniformBuffer — other paths stage that one with their own layouts, which would silently
  // desync the mirror.
  void *mTexShadowScnBuffer = nullptr;
  size_t mTexShadowScnBufferSize = 0;
  std::vector<uint8_t> mTexShadowScnStaging;
  bool mTexShadowScnStagingValid = false;
  void *mCsmVpBuffer = nullptr;            // pass-1 per-cascade light VPs (4 x 256-B slots)
  // Physically-scattered sky: LUT textures (always created with the sky pipeline so the dome
  // bind group is stable), the bake pipelines, and the last-baked parameter block.
  void *mSkyLutTex = nullptr;
  void *mSkyLutView = nullptr;
  void *mSkyTransTex = nullptr;
  void *mSkyTransView = nullptr;
  void *mSkyLutSampler = nullptr;
  void *mSkyLutPipeline = nullptr;
  void *mSkyTransPipeline = nullptr;
  void *mSkyLutShaderModule = nullptr;
  void *mSkyLutBgl = nullptr;
  void *mSkyLutBg = nullptr;
  void *mSkyLutUb = nullptr;
  float mSkyScatterKey[24] = {0};
  bool mSkyScatterBaked = false;
  // Screen-space reflections: full-res RGBA16F combine target (scene + reflections, pre-tonemap),
  // fed to the AgX pass instead of mHdrView when the SSR pass ran this frame.
  void *mSsrTex = nullptr;
  void *mSsrView = nullptr;
  uint32_t mSsrW = 0, mSsrH = 0;
  void *mSsrPipeline = nullptr;
  void *mSsrShaderModule = nullptr;
  void *mSsrBgl = nullptr;
  void *mSsrBg = nullptr;
  void *mSsrBgSceneKey = nullptr;   // mHdrView the bind group was built against
  void *mSsrBgDepthKey = nullptr;   // depth view ditto
  void *mSsrUb = nullptr;
  void *mAgxBgSsr = nullptr;        // AgX input bind group over mSsrView
  // OmniLight probe volume (3D RGBA16F, 4 z-slabs of SH coefficients). A 1x1x4 black default is
  // created with the texshadow pipeline so bind groups are always valid; setOmniLightVolume
  // swaps in a baked volume and invalidates the bind-group cache.
  void *mOmniTex = nullptr;
  void *mOmniView = nullptr;
  void *mOmniSampler = nullptr;
  float mOmniOrigin4[4] = {0, 0, 0, 0};
  float mOmniParams4[4] = {0, 0, 0, 1};
  float mOmniMisc4[4] = {1, 0, 0, 0};       // x = fade-in blend
  void *mOmniCubeTex = nullptr;
  void *mOmniCubeView = nullptr;
  float mOmniCubeCenter4[4] = {0, 0, 0, 0};
  float mOmniAabbMin4[4] = {0, 0, 0, 0};
  float mOmniAabbMax4[4] = {0, 0, 0, 0};
  void *mOmniBarPipeline = nullptr;
  void *mOmniBarShaderModule = nullptr;
  void *mOmniBarBgl = nullptr;
  void *mOmniBarBg = nullptr;
  void *mOmniBarUb = nullptr;
  // Camera pass: auto-exposure adaptation ping-pong (1x1 RGBA16F) + pipeline.
  void *mAdaptTexA = nullptr;
  void *mAdaptTexB = nullptr;
  void *mAdaptViewA = nullptr;
  void *mAdaptViewB = nullptr;
  void *mAdaptPipeline = nullptr;
  void *mAdaptShaderModule = nullptr;
  void *mAdaptBgl = nullptr;
  bool mAdaptFlip = false;
  uint64_t mFrameCounter = 0;
  // TAA: ping-pong resolved-output textures (this frame's output = next frame's history),
  // the previous frame's UNJITTERED view-proj, and the active output view (null = mView).
  void *mTaaMvTex[2] = {nullptr, nullptr};
  void *mTaaMvView[2] = {nullptr, nullptr};
  uint32_t mTaaMvW = 0, mTaaMvH = 0;
  int mTaaMvCur = 0;
  bool mTaaMvHistValid = false;
  float mTaaMvPrevVP[16] = {0};
  void *mTaaMvPipeline = nullptr;
  void *mTaaMvShaderModule = nullptr;
  void *mTaaMvBgl = nullptr;
  void *mTaaMvUb = nullptr;
  void *mTaaMvActiveView = nullptr;
  // W4a overlay lines: a private RGBA8 output the finished frame is copied into before the
  // lines are drawn, so overlay pixels never land in a TAA history buffer. Created on first
  // use and only then — a session that never enables an overlay allocates nothing.
  void *mOverlayTex = nullptr;         // WGPUTexture (RGBA8Unorm; CopyDst|RenderAttachment|TextureBinding)
  void *mOverlayView = nullptr;        // WGPUTextureView
  uint32_t mOverlayW = 0, mOverlayH = 0;
  void *mOverlayActiveView = nullptr;  // this frame's overlay output; null = no overlay drawn
  // Own Scene-slot uniform buffer (one 256 B slot per batch). Deliberately NOT
  // mScnUniformBuffer: the SSAO prepass bind group is keyed on that buffer's identity, and
  // reallocating it under the scene passes is how you silently desync a cached bind group.
  void *mOverlayLineUb = nullptr;
  size_t mOverlayLineUbSize = 0;
  void *mOverlayLineBg = nullptr;      // bind group over mOverlayLineUb (dynamic offset per batch)
  void *mOverlayLineBgBuf = nullptr;   // buffer identity mOverlayLineBg was built against
  void *mOverlayVb = nullptr;          // persistent line vertex buffer, grown on demand
  size_t mOverlayVbSize = 0;
  bool ensureOverlayLineTarget();
  void *mOverlayReadback = nullptr;  // MapRead buffer for readbackOverlayOutput (never shared)
  size_t mOverlayReadbackSize = 0;
  // P7 HUD quads. The uniform buffer is allocated ONCE at its full size (kMaxHudQuads slots)
  // rather than grown, because every cached per-texture bind group is built against that exact
  // buffer -- growing it would silently invalidate all of them, which is the bug class the
  // comment on mOverlayLineUb above already warns about.
  static const uint32_t kMaxHudQuads = 64;
  void *mHudShaderModule = nullptr;
  void *mHudPipeline = nullptr;
  void *mHudBgl = nullptr;
  void *mHudUb = nullptr;      // kMaxHudQuads * 256 B, dynamic offset per quad
  void *mHudSampler = nullptr;  // NON-filtering: a device inset is device pixels
  void *mHudWhiteTex = nullptr;  // 1x1 opaque white, for flat-colour quads
  void *mHudWhiteView = nullptr;
  void *mHudCache = nullptr;   // std::map<key, HudTex> -- opaque so webgpu types stay in the .cpp
  bool ensureHudQuadPipeline();
  void releaseHudResources();
  // Volumetric shafts pass (additive One/One onto the HDR scene, baked sun-vis from the probes).
  void *mVolPipeline = nullptr;
  void *mVolShaderModule = nullptr;
  void *mVolBgl = nullptr;
  void *mVolUb = nullptr;
  bool ensureVolPipeline();
  void *mClipDepthSharedPipeline = nullptr;  // pass-1 pipeline reading model from the Scene slots
  void *mClipDepthSharedShaderModule = nullptr;
  void *mClipDepthSharedBgl = nullptr;
  void *mClipDepthSharedBg = nullptr;        // rebuilt when mTexShadowScnBuffer reallocates
  void *mClipDepthSharedBgScnBuf = nullptr;
  size_t mShadowUniformBufferSize = 0;
  // R4 lighting rung 1: textured+shadowed pipeline (kSolidLitTexturedShadow). 9-entry layout:
  // Scene uniform @0 (dynamic, 192 B) + albedo/rough/metal/normal @1–4 + filtering sampler @5 +
  // unfilterable shadow map @6 + non-filtering shadow sampler @7 + shared LightU uniform @8
  // (lightViewProj + shadowParams, 80 B). Reuses mShadowMapView + the scn-tex defaults.
  void *mTexShadowShaderModule = nullptr;   // WGPUShaderModule
  void *mTexShadowPipeline = nullptr;       // WGPURenderPipeline
  void *mTexShadowBindGroupLayout = nullptr;
  void *mTexShadowSampler = nullptr;        // WGPUSampler (non-filtering, for the R32Float map)
  void *mLightUniformBuffer = nullptr;      // WGPUBuffer (80 B: lightViewProj + shadowParams)
  // Per-draw bind-group CACHE for the textured-shadowed pass. Bind groups only vary by the 4
  // resolved texture views (the uniform is bound via dynamic offset; samplers/shadow map/light
  // uniform are per-target constants), so a big scene has few unique groups (city: 3523 draws,
  // ~57 textures) — creating + releasing one per draw per frame dominated CPU (~5 FPS). Keyed on
  // the resolved (albedo, rough, metal, normal) views; invalidated when a referenced buffer is
  // reallocated (the pointers are baked into the group). Entries released on invalidate/destroy.
  // Analytic sky dome (kSkyAtmosphere): pipeline (MSAA 4x, matches the pass it draws in) + its
  // 96 B SkyU uniform + bind group/layout. Lazily built; null → flat clear colour as before.
  void *mSkyShaderModule = nullptr;   // WGPUShaderModule
  void *mSkyPipeline = nullptr;       // WGPURenderPipeline (multisample.count = 4)
  void *mSkyBindGroupLayout = nullptr;
  void *mSkyUniformBuffer = nullptr;  // WGPUBuffer (96 B SkyU)
  void *mSkyBindGroup = nullptr;
  bool ensureSkyPipeline();
  // Image-cubemap skybox (kSkyCubemap): Background's six *Url faces as a 6-layer cube texture,
  // drawn behind the scene instead of the analytic dome when skyUniform96's mode flag
  // (worldUp.w == 1) selects it. Same pass placement + variants as the procedural dome.
  void *mSkyCubeShaderModule = nullptr;
  void *mSkyCubePipeline = nullptr;        // count=4, RGBA8, D24
  void *mSkyCubePipelineHdr = nullptr;     // count=4, RGBA16F, D24
  void *mSkyCubePipelineRev = nullptr;     // count=4, RGBA8, D32F
  void *mSkyCubePipelineHdrRev = nullptr;  // count=4, RGBA16F, D32F
  void *mSkyCubeBindGroupLayout = nullptr;
  void *mSkyCubeBindGroup = nullptr;       // SkyU + cube view + sampler; rebuilt on setSkyCubemap
  void *mSkyCubeTexture = nullptr;         // WGPUTexture (6-layer RGBA8)
  void *mSkyCubeView = nullptr;            // WGPUTextureView (dimension Cube)
  void *mSkyCubeSampler = nullptr;
  bool ensureSkyCubePipeline();

public:
  // Upload/replace the Background image cubemap: six equally-sized decoded faces in WREN
  // cube-face order (+X,-X,+Y,-Y,+Z,-Z — OmBackground::cubemapTexture order). Returns false
  // (and leaves any previous cube in place) when unavailable. Passing a complete set twice is
  // the caller's concern — guard with your own "uploaded for this background" state.
  bool setSkyCubemap(const QImage *const faces[6]);
  bool hasSkyCubemap() const { return mSkyCubeView != nullptr; }

private:
  // SSAO (kSsaoEstimate): camera clip-depth prepass (reuses mScnClipDepthPipeline + the pass-2
  // scene uniform) → half-res AO estimate → shared bloom blur → multiply onto the scene. The
  // half-res ping-pong textures are the BLOOM ones (AO finishes before bloom starts).
  void *mAoDepthTex = nullptr;          // R32Float full-res camera clip depth
  void *mAoDepthView = nullptr;
  void *mSsaoShaderModule = nullptr;
  void *mSsaoEstimateBgl = nullptr;     // {uniform, UNFILTERABLE float tex, NON-filtering sampler}
  void *mSsaoPipeline = nullptr;        // estimate (layout = mSsaoEstimateBgl — R32F isn't filterable)
  void *mSsaoApplyPipeline = nullptr;   // kBloomPost fs_composite + MULTIPLY blend (Dst, Zero)
  void *mSsaoUb = nullptr;              // params: x = radius UV, y = intensity (written per exec)
  void *mSsaoApplyUb = nullptr;         // fs_composite strength = 1.0
  void *mSsaoBgEstimate = nullptr;      // {mSsaoUb, mAoDepthView, sampler} (+ normal view in GTAO mode)
  void *mSsaoBgApply = nullptr;         // {mSsaoApplyUb, mBloomViewA (legacy) / mBloomViewB (GTAO), sampler}
  // GTAO mode (default; OMNISIM_WGPU_GTAO=0 reverts to the legacy curvature kernel above):
  // MRT prepass (depth + world normal), horizon-based estimate, depth-aware denoise.
  bool mGtaoMode = false;               // resolved once in ensureSsaoResources
  void *mSsaoDenoisePipeline = nullptr;      // fs_denoise (4x4 depth-aware)
  void *mSsaoBgDenoise = nullptr;            // {mSsaoUb, scene MSAA depth, mBloomViewA(=AO)}
  void *mSsaoBgDepthKey = nullptr;           // which scene depth view the exec bind groups hold
  void *mSsaoSceneBg = nullptr;         // 1-entry scn-layout group over mScnUniformBuffer (prepass)
  void *mSsaoSceneBgBuf = nullptr;      // buffer identity the group was built against
  bool ensureSsaoResources();
  // HDR + AgX (kAgxTonemapPost): pass 2 renders into an RGBA16F target (4x MSAA + 1x resolve),
  // then the tonemap pass writes display LDR into mView — bloom/readback unchanged downstream.
  // Built lazily; any failure → the legacy LDR path.
  void *mHdrTex = nullptr;                 // RGBA16F 1x (resolve target, sampled by the tonemap)
  void *mHdrView = nullptr;
  void *mHdrMsaaTex = nullptr;             // RGBA16F 4x (pass-2 color attachment)
  void *mHdrMsaaView = nullptr;
  void *mTexShadowPipelineHdr = nullptr;   // textured-shadow pipeline, RGBA16F target, count=4
  void *mSkyPipelineHdr = nullptr;         // sky dome pipeline, RGBA16F target, count=4
  void *mAgxShaderModule = nullptr;
  void *mAgxBgl = nullptr;
  void *mAgxPipeline = nullptr;            // RGBA16F → AgX → RGBA8 (mView)
  void *mAgxUb = nullptr;                  // 16 B: exposure
  void *mAgxBg = nullptr;
  bool ensureHdrPipelines();
  // Bloom post-process chain (kBloomPost): bright-pass extract → half-res separable blur (ping-
  // pong A/B) → additive composite back onto mTexture. Built lazily; any failure → no bloom.
  void *mBloomShaderModule = nullptr;
  void *mBloomBgl = nullptr;                 // {uniform, texture, sampler} layout (all 3 pipelines)
  void *mBloomExtractPipeline = nullptr;
  void *mBloomBlurPipeline = nullptr;
  void *mBloomCompositePipeline = nullptr;   // additive One/One into mView
  void *mBloomTexA = nullptr;
  void *mBloomViewA = nullptr;               // half-res ping
  void *mBloomTexB = nullptr;
  void *mBloomViewB = nullptr;               // half-res pong
  void *mBloomUbExtract = nullptr;           // 16 B each, written once (per-target constants)
  void *mBloomUbBlurH = nullptr;
  void *mBloomUbBlurV = nullptr;
  void *mBloomUbComposite = nullptr;
  void *mBloomBgExtract = nullptr;
  void *mBloomBgExtractHdr = nullptr;  // extract sampling the RGBA16F scene (HDR bloom)           // scene → A
  void *mBloomBgBlurH = nullptr;             // A → B
  void *mBloomBgBlurV = nullptr;             // B → A
  void *mBloomBgComposite = nullptr;         // A → scene (additive)
  bool ensureBloomPipelines(float strength);
  // MSAA 4x: pass 2 renders into these and resolves to mTexture (readback unchanged). Null when
  // creation failed → the 1x path runs instead.
  void *mTexShadowPipelineMsaa = nullptr;  // WGPURenderPipeline (multisample.count = 4)
  void *mScnMsaaColorTex = nullptr;        // WGPUTexture (4x RGBA8, RenderAttachment)
  void *mScnMsaaColorView = nullptr;
  void *mScnMsaaDepthTex = nullptr;        // WGPUTexture (4x Depth24Plus)
  void *mScnMsaaDepthView = nullptr;
  bool ensureMsaaTargets();
  // Reversed-Z (near-uniform depth precision; kills far-field decal z-fighting): FLOAT depth
  // targets + Greater-compare pipeline variants, used when clearAndDrawSceneTexturedShadowed is
  // called with reversedZ=true (the live main view). Standard-Z callers untouched.
  void *mScnMsaaDepthTexF32 = nullptr;   // Depth32Float, 4x (reversed pass-2)
  void *mScnMsaaDepthViewF32 = nullptr;
  void *mScnDepthTexF32 = nullptr;       // Depth32Float, 1x (reversed SSAO prepass)
  void *mScnDepthViewF32 = nullptr;
  void *mTexShadowPipelineMsaaRev = nullptr;  // Greater + D32F, RGBA8, count=4
  void *mTexShadowPipelineHdrRev = nullptr;   // Greater + D32F, RGBA16F, count=4
  // Translucent (src-over blend, depth-write-off, RGB-only writeMask) variants of the five
  // pass-2 pipelines above; bound for the back-to-front translucent draw span. Null (creation
  // failure) degrades that span to the opaque pipeline — same output as before transparency.
  void *mTexShadowPipelineBlend = nullptr;         // 1x, RGBA8, Less + D24
  void *mTexShadowPipelineMsaaBlend = nullptr;     // count=4, RGBA8, Less + D24
  void *mTexShadowPipelineHdrBlend = nullptr;      // count=4, RGBA16F, Less + D24
  void *mTexShadowPipelineMsaaRevBlend = nullptr;  // count=4, RGBA8, Greater + D32F
  void *mTexShadowPipelineHdrRevBlend = nullptr;   // count=4, RGBA16F, Greater + D32F
  void *mSkyPipelineRev = nullptr;            // sky: Always compare, D32F format, RGBA8
  void *mSkyPipelineHdrRev = nullptr;         // sky: Always compare, D32F format, RGBA16F
  void *mScnClipDepthPipelineRev = nullptr;   // camera depth prepass: Greater + D32F
  bool ensureReversedDepthTargets();
  std::map<std::array<const void *, 5>, void *> mTexShadowBgCache;
  void *mTexShadowBgCacheScnBuf = nullptr;    // mScnUniformBuffer the cache was built against
  void *mTexShadowBgCacheLightBuf = nullptr;  // mLightUniformBuffer the cache was built against
  void *mTexShadowP1Bg = nullptr;             // cached pass-1 (clip-depth) bind group
  void *mTexShadowP1BgBuf = nullptr;          // mShadowUniformBuffer mTexShadowP1Bg was built against
  void releaseTexShadowBgCache();
  // T1.2 CSM (multi-cascade): kSolidLitCsm lit pipeline + its own 3-entry layout
  // (CsmScene uniform @0, 448 B dynamic + texture_2d_array @1 + non-filtering sampler @2),
  // an N-LAYER R32Float sampleable shadow map (one layer per cascade) with per-layer render
  // views + a shared Depth24Plus light-pass occlusion attachment, and a dedicated dynamic
  // CsmScene uniform buffer. Built lazily by ensureSceneCsmPipeline / ensureShadowMapArray.
  void *mCsmShaderModule = nullptr;          // WGPUShaderModule (kSolidLitCsm)
  void *mCsmPipeline = nullptr;              // WGPURenderPipeline
  void *mCsmBindGroupLayout = nullptr;       // WGPUBindGroupLayout (3-entry)
  void *mCsmSampler = nullptr;               // WGPUSampler (non-filtering)
  void *mCsmShadowArrayTexture = nullptr;    // WGPUTexture (R32Float, N array layers)
  void *mCsmShadowArrayView = nullptr;       // WGPUTextureView (2D-array, all layers — sampled)
  std::vector<void *> mCsmShadowLayerViews;  // WGPUTextureView per layer (2D — render target)
  void *mCsmShadowDepthTexture = nullptr;    // WGPUTexture (Depth24Plus, shared light-pass occlusion)
  void *mCsmShadowDepthView = nullptr;       // WGPUTextureView
  uint32_t mCsmShadowRes = 0;                // current array layer resolution
  uint32_t mCsmCascadeCount = 0;             // current array layer count
  void *mCsmDepthUniformBuffer = nullptr;    // WGPUBuffer (light clip-depth pass, N*draws slots)
  size_t mCsmDepthUniformBufferSize = 0;
  void *mCsmUniformBuffer = nullptr;         // WGPUBuffer (CsmScene, 512 B stride per draw)
  size_t mCsmUniformBufferSize = 0;
  // T1.4 TAA: temporal-resolve post pass (kTaaResolve). Fullscreen (no vertex buffer); samples a
  // current + history color texture + a filtering sampler, blends with neighborhood clamp. 4-entry
  // layout (TaaParams uniform @0 + curTex @1 + histTex @2 + sampler @3).
  void *mTaaShaderModule = nullptr;          // WGPUShaderModule (kTaaResolve)
  void *mTaaPipeline = nullptr;              // WGPURenderPipeline
  void *mTaaBindGroupLayout = nullptr;       // WGPUBindGroupLayout (4-entry)
  void *mTaaSampler = nullptr;               // WGPUSampler (filtering, for reprojection)
  void *mTaaUniformBuffer = nullptr;         // WGPUBuffer (TaaParams, 32 B)
  // T1.4 TAA ping-pong history: two RGBA8 color buffers (RenderAttachment|TextureBinding|CopySrc)
  // swapped each accumulateTaa() call — the GPU-resident accumulator the temporal resolve feeds back.
  void *mTaaHistoryTexture[2] = {nullptr, nullptr};
  void *mTaaHistoryView[2] = {nullptr, nullptr};
  uint32_t mTaaHistoryPing = 0;              // index written by the NEXT accumulateTaa()
  uint32_t mTaaHistoryW = 0;
  uint32_t mTaaHistoryH = 0;
  // T1.3 fog: analytic distance-fog resolve post pass (kFogResolve). Fullscreen; samples a scene
  // colour + a metric view-distance texture, blends toward the fog colour. 4-entry layout (FogParams
  // uniform @0 + sceneTex @1 + depthTex @2 unfilterable + non-filtering sampler @3).
  void *mFogShaderModule = nullptr;          // WGPUShaderModule (kFogResolve)
  void *mFogPipeline = nullptr;              // WGPURenderPipeline
  void *mFogBindGroupLayout = nullptr;       // WGPUBindGroupLayout (4-entry)
  void *mFogSampler = nullptr;               // WGPUSampler (non-filtering; depth is unfilterable)
  void *mFogUniformBuffer = nullptr;         // WGPUBuffer (FogParams, 32 B)
  // R4 lighting convergence: kSolidLitTexturedShadow × CSM (full material + multi-cascade shadows).
  // 9-entry layout (Scene @0 + albedo/rough/metal/normal @1-4 + material sampler @5 + cascade array @6
  // + non-filtering sampler @7 + CsmLightU @8). Reuses ensureSceneTexPipeline's material sampler +
  // default 1×1 maps, ensureShadowMapArray's array, and the clip-depth pipeline for the depth passes.
  void *mTexCsmShaderModule = nullptr;       // WGPUShaderModule (kSolidLitTexturedCsm)
  void *mTexCsmPipeline = nullptr;           // WGPURenderPipeline
  void *mTexCsmBindGroupLayout = nullptr;    // WGPUBindGroupLayout (9-entry)
  void *mTexCsmSampler = nullptr;            // WGPUSampler (non-filtering; the R32Float cascade array)
  void *mCsmLightUniformBuffer = nullptr;    // WGPUBuffer (CsmLightU: 4 light VPs + params/splits + hemi, 336 B)

  bool ensureTrianglePipeline();
  bool ensureMeshPipeline();
  bool ensureMvpPipeline();
  bool ensureTexPipeline();
  bool ensureInstPipeline();
  bool ensureScenePipeline();
  bool ensureSceneDepthPipeline();
  bool ensureSceneDepthF32Pipeline();
  bool ensureSceneClipDepthF32Pipeline();
  bool ensureClipDepthSharedPipeline();
  bool ensureAdaptPipeline();
  bool ensureTaaMvPipeline();
  static uint16_t toHalfPublic(float f);
  // Re-marches the 128x64 sky-view LUT + 128x1 transmittance strip when the sun/preset changed
  // (memcmp-cached); the dome shader then samples them. scat24 = the ScatU payload (24 floats).
  bool updateSkyScatterLut(const float *scat24);
  bool ensureSsrPipeline();
  bool ensureSceneRangeF32Pipeline();
  bool ensureSceneAgxPipeline();
  bool ensurePickPipeline();
  bool ensureSceneTexPipeline();
  bool ensureTexturedShadowPipeline();  // kSolidLitTexturedShadow (9-entry: material + shadow + light)
  bool ensureOverlayPipeline();
  bool ensureTexturedQuadPipeline();
  bool ensureLinePipeline();
  bool ensureLineNoDepthPipeline();
  bool ensureSceneShadowPipeline();
  bool ensureShadowMapTexture();
  bool ensureSceneCsmPipeline();                            // kSolidLitCsm (3-entry: CsmScene + array + sampler)
  bool ensureShadowMapArray(uint32_t res, uint32_t cascades);  // N-layer R32Float shadow array + per-layer views
  bool ensureTaaResolvePipeline();                          // kTaaResolve (4-entry: TaaParams + cur + hist + sampler)
  bool ensureTaaHistory();                                  // two RGBA8 ping-pong history color buffers
  bool ensureFogResolvePipeline();                          // kFogResolve (4-entry: FogParams + scene + depth + sampler)
  bool ensureTexturedCsmPipeline();                         // kSolidLitTexturedCsm (9-entry: material + cascade array + LightU)
};

#endif
