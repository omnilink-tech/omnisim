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

#ifndef WB_WGPU_SHADERS_HPP
#define WB_WGPU_SHADERS_HPP

//
// WbWgpuShaders — R3.4 of engine-migration-plan.md §14.3.
//
// "Path-3 shader port: hand-port core PBR/Phong/default to WGSL;
//  naga toolchain for long tail. 2 weeks."
//
// At R3.4-step-1 ships just the `kTriangleClipSpace` WGSL string —
// a vertex+fragment pair that emits a fixed-position triangle with
// vertex-shaded RGB. No vertex buffers, no bind groups, no uniforms.
// This is the minimum shader that proves the
// WbWgpuRenderTarget::clearAndDrawTriangle path works end-to-end:
//   draw call -> rasterizer -> fragment shader -> color attachment ->
//   readback -> pixel matches expected color.
//
// R3.4-step-2 adds a triangle-with-vertex-buffer pair that consumes
// the layout WbWgpuMeshCache uploads (pos3 + norm3 + uv2 = 32 bytes
// stride). That's the production WGSL contract that locks the
// vertex format the R3.2b WREN-mesh-adapter has to translate INTO.
//
// R3.4-step-3 is the actual Phong/PBR port. That layer adds:
//   - per-frame uniform (view-proj matrix)
//   - per-draw uniform (model matrix + material params)
//   - sampler + texture bind group (paired with R3.5 texture bridge)
//
// Each step is independently runtime-verifiable via the
// OMNISIM_PROBE_WGPU env var.
//

namespace WbWgpuShaders {

// WGSL source: clip-space triangle with vertex-shaded RGB.
// Vertex IDs 0/1/2 -> three hard-coded NDC positions covering the
// upper-right triangle of the viewport. Fragment color is just the
// per-vertex barycentric so we can sanity-check by sampling specific
// pixels in the readback.
extern const char *kTriangleClipSpace;

// WGSL source: clip-space triangle that consumes a vertex buffer in
// the production OmniSim layout — pos3 (12 bytes) + norm3 (12 bytes)
// + uv2 (8 bytes) = 32 bytes/vertex. This is the wire format
// WbWgpuMeshCache uploads and WREN's mesh-adapter (R3.2b) will
// translate INTO. The fragment shader colors by the (positive
// half of the) normal vector so we can verify the per-vertex
// attribute fetch end-to-end.
//
// R3.4-step-3 swaps this for a Phong/PBR pair with bind groups +
// view-proj uniform; the vertex-buffer layout stays unchanged so
// the upgrade is pipeline-only.
extern const char *kTriangleVertexBuffer;

// WGSL source: same vertex layout as kTriangleVertexBuffer, plus a
// `@group(0) @binding(0)` uniform holding a 4x4 view-proj matrix.
// Adds the bind-group infrastructure R3.5 (texture bridge) and the
// real Phong/PBR port will hang additional bindings off of. The
// vertex shader transforms each vertex by viewProj, the fragment
// shader is the same normal-as-color visualization.
extern const char *kTriangleMVP;

// WGSL source: same vertex layout, same MVP uniform at binding 0,
// plus a `@group(0) @binding(1)` texture_2d<f32> and
// `@group(0) @binding(2)` sampler. Fragment samples the texture
// at the interpolated uv. This is the R3.5 wire shape — the Phong
// + PBR ports add more bindings on top of the same 3-entry layout.
extern const char *kTriangleTextured;

// WGSL source: R3.7 storage-buffer-driven instancing. Same vertex
// layout, MVP uniform at binding 0, plus
// `@group(0) @binding(1) var<storage, read> bodies : array<vec4<f32>>`
// — one vec4 per instance, .xyz = body translation (we keep it
// vec4-aligned for storage-buffer layout). Vertex shader picks the
// per-instance offset via @builtin(instance_index) and adds it to
// the local vertex position, so a single 3-vertex VB renders N
// triangles at N body positions. The fragment shader colors by
// normal so the readback distinguishes instances by where their
// triangles land — exactly the Newton-interop shape the eventual
// production path will use, just with body-state floats instead of
// a debug position-only struct.
extern const char *kTriangleInstanced;

// WGSL source: R3.4-step-4 production scene-pass shader. Same
// vertex layout (pos3+norm3+uv2 stride 32) as the rest of R3.4.
// Bind group has one entry — a 192-byte uniform structured as:
//   viewProj : mat4x4<f32>   // 64 B
//   model    : mat4x4<f32>   // 64 B
//   baseColor: vec4<f32>     // 16 B  (.rgb + alpha)
//   light    : vec4<f32>     // 16 B  (.xyz = directional light
//                            //         direction in world space,
//                            //         .w = ambient term [0..1])
//   _pad     : vec4<f32>x2   // 32 B  (uniform-block min align)
// Vertex stage applies viewProj * model to the position and uses
// the upper-3x3 of model to rotate the normal into world space.
// Fragment stage computes Lambertian:
//   color = baseColor.rgb * (ambient + max(dot(N, -L), 0))
// — a placeholder for the real Phong/PBR port. Keeps the bind-
// group layout small (one uniform), making per-Solid rebinding
// cheap. R3.4-step-5 (PBR proper) extends this struct with the
// remaining PBRAppearance fields (metallic, roughness, IOR,
// normalMap binding, etc.).
extern const char *kSolidLit;

// R4 material fidelity — kSolidLit + an albedo texture sampled at the mesh UVs.
// Adds binding 1 (texture_2d) + binding 2 (sampler) to the Scene bind group; the
// fragment multiplies the sampled albedo by baseColor × Lambertian intensity. Used
// only for draws that carry a baseColorMap; flat draws stay on kSolidLit.
extern const char *kSolidLitTextured;

// Analytic atmospheric sky dome (wgpu counterpart of WREN's Background.atmosphericSky):
// fullscreen triangle, per-pixel camera ray, day/dusk/night palettes blended by sun
// elevation + a sun disk/halo tinted by the light colour. Drawn first in the scene pass
// with depth write off; the day-night system follows the sun direction automatically.
extern const char *kSkyAtmosphere;

// SSAO estimate: depth-only screen-space AO from the camera clip-depth prepass; the factor is
// blurred (shared bloom blur) and multiplied onto the scene.
extern const char *kSsaoEstimate;

// AgX filmic tonemap post-pass: HDR (RGBA16F) scene → exposure → AgX curve → display LDR.
extern const char *kAgxTonemapPost;

// Bloom post-process (Viewpoint.bloomThreshold): bright-pass extract + separable gaussian blur +
// additive composite, three fragment entry points over one fullscreen-triangle vertex shader.
extern const char *kBloomPost;

// R4 step-3c-A — full-screen overlay. A vertex-id full-screen triangle (no vertex
// buffer) + a flat RGBA colour from a 1-entry uniform, alpha-blended over the scene.
// The loading / black / status full-screen overlays composited in the pane.
extern const char *kFullScreenOverlay;

// R4 step-3c-A — screen-space textured quad: a texture drawn into an NDC sub-rect (quad
// from vertex-id + a rect uniform), unlit. The compositing primitive for device-output
// insets (camera/range-finder/display HUD) and 2D overlays.
extern const char *kTexturedQuad;

// T1.4 TAA — temporal-resolve post pass (WGSL port of taa-preview.html). Fullscreen (quad from
// vertex-id, no vertex buffer); reprojects history by a motion vector, 3x3 neighborhood-AABB
// clamps it, rejects off-screen history, feedback-blends current⊕history. Bindings: TaaParams
// uniform @0 + curTex @1 + histTex @2 + filtering sampler @3. ctrl.y<0.5 → curTex passthrough.
extern const char *kTaaResolve;

// T1.3 fog — analytic distance-fog resolve post pass (foundation toward volumetric). Fullscreen
// (quad from vertex-id); blends a scene-colour texture toward a fog colour by 1-exp(-density*dist),
// reading a metric view-distance texture. Bindings: FogParams uniform @0 + sceneTex @1 + depthTex @2
// (unfilterable) + non-filtering sampler @3. params.w<0.5 → scene passthrough.
extern const char *kFogResolve;

// R4 lighting-convergence rung 1 — kSolidLitTextured + cast shadows: the full material path
// (albedo/roughness/metalness/normal + GGX + sRGB, bindings 0–5) plus a PCF shadow term from a
// light-space depth map (bindings 6–7) via a shared light-VP uniform (binding 8). Lighting split
// into unshadowed ambient + shadowed direct. The textured-shadowed pass toward WREN parity.
extern const char *kSolidLitTexturedShadow;

// R4 lighting convergence — kSolidLitTexturedShadow × CSM: the full material path (albedo/roughness/
// metalness/normal + GGX + sRGB + hemisphere-IBL ambient) with MULTI-CASCADE shadows. @6 is a
// texture_2d_array; LightU carries array<mat4,4> light VPs + a cascadeSplits vec4 (shadowParams.w =
// cascade count); the shadow term selects a cascade by the fragment's linear view depth + 3x3-PCFs that
// layer. The shader the main-view default needs (materials AND tight multi-cascade shadows).
extern const char *kSolidLitTexturedCsm;

// R4 step-3c-A.1 — picking. Same Scene uniform + vertex stage as kSolidLit, but
// the fragment outputs `baseColor` FLAT (no lighting), so the caller can encode a
// per-draw integer ID into baseColor (rgb = id&0xFF, (id>>8)&0xFF, (id>>16)&0xFF
// over an RGBA8 target), render the scene offscreen, read back the texel under the
// cursor, and decode the picked draw. The wgpu analog of WbWrenPicker's "picking"
// material pass — built additively (offscreen RTT in the wgpu pane), so WREN's
// default picking path is untouched.
extern const char *kSolidPick;

// T1.1 — kSolidLit + AgX filmic tonemap (Sobotka/Wrensch minimal fit), the first
// real Tier-1 fidelity shader in the engine (ported verbatim from the converged
// WebGL2 spec preview docs/developer/agx-tonemap-preview.html). Same vertex stage
// + Lambertian shade as kSolidLit; the fragment runs the lit colour through AgX
// (3x3 inset → log2 encode → 6th-order contrast → 3x3 outset → clamp) before
// output. Selected by a default-false pipeline flag so the lit path is byte-
// identical (golden-image-gated); opt-in via WbCamera OMNISIM_CAMERA_AGX=1.
extern const char *kSolidLitAgX;

// T1.2 CSM sub-step 3 — shadow-receiving lit shader. kSolidLit + a depth-compare
// sample of the light-space shadow map, using its own ShadowScene uniform (adds
// lightViewProj + shadowParams) and a 3-entry bind group (uniform, shadow
// texture, sampler). shadowParams.x strength 0 → byte-identical to kSolidLit, so
// an unbound/zeroed shadow setup is a no-op. Unreferenced until the sub-step-3
// pipeline + bind group land. See .cpp.
extern const char *kSolidLitShadow;

// T1.2 CSM (multi-cascade) — N-cascade generalisation of kSolidLitShadow, pairing with
// WbWgpuSceneRenderer::buildCascadeLightViewProjs. Carries an array<mat4x4,4> of per-cascade
// light VPs + a cascadeSplits vec4 (far view-depths) in a 448 B CsmScene uniform @0, samples a
// texture_2d_array shadow map @1 (one layer per cascade) via a non-filtering sampler @2, and
// selects the cascade by the fragment's linear view depth (camera clip.w). shadowParams.z =
// cascade count; strength 0 → byte-identical to kSolidLit. UNREFERENCED until the multi-cascade
// depth pass + texture-array pipeline land; naga + render-pipeline validated standalone (wgpu-py
// on the live device) against the intended bind-group/vertex layout. See .cpp.
extern const char *kSolidLitCsm;

// R5 (sensor pipeline) — depth/RangeFinder shader. Same vertex transform +
// Scene uniform as kSolidLit, but the fragment outputs LINEAR view-space
// distance (grayscale) instead of Lambertian color. The key trick: a
// perspective projection's clip-space .w component IS the linear view-space
// depth (distance along the camera axis), so no depth-texture readback is
// needed (sidesteps wgpu's Depth24Plus-not-copyable constraint). The far plane
// is passed in the Scene uniform's pad0.x slot.
extern const char *kSolidDistance;

// R5b — F32 variant of kSolidDistance. Same vertex transform, but the fragment
// writes the RAW view-space distance (meters) into an R32Float target instead
// of a normalized grayscale RGBA8 value. This is the real RangeFinder-device
// output shape: metric depth at full float precision, no 8-bit quantization,
// no far-plane normalization. clip.w → color.r, read back as float32.
extern const char *kSolidDistanceF32;

// T1.2 CSM shadow-map depth: writes NDC depth clip.z/clip.w (@builtin(position).z,
// ∈[0,1]) instead of clip.w, so it is correct under an ORTHOGRAPHIC light
// projection (where clip.w≡1). The depth a cascaded-shadow-map pass stores. See
// .cpp. Renders into the same R32Float target as the other F32 depth passes.
extern const char *kSolidClipDepthF32;

// R5d — Lidar RADIAL-range shader. Outputs euclidean distance from the camera
// to the surface point (length of the view-space position) into an R32Float
// target, not planar depth. Uses a {viewProj, view, model} uniform (192 B, so
// it reuses the scene bind-group layout) because recovering view-space position
// needs the view matrix on top of viewProj. This is what a Lidar ray measures.
extern const char *kSolidRangeF32;

}  // namespace WbWgpuShaders

#endif
