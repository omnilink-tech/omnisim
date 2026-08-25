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

#include "OmWgpuShaders.hpp"

namespace OmWgpuShaders {

  // R3.4-step-1 shader: full-viewport-quad-ish triangle, vertex-shaded
  // RGB. Three hard-coded NDC vertices fill the upper half + diagonal
  // of the framebuffer; left untouched pixels keep the clear color.
  // This is enough to verify the entire render pipeline (encode ->
  // submit -> rasterize -> fragment shader -> color attachment ->
  // readback) works on this GPU, without needing vertex buffer
  // plumbing yet.
  //
  // The fragment colors are pure R/G/B at the three vertices, which
  // means the centroid pixel of the triangle should read back roughly
  // (255/3, 255/3, 255/3, 255) — that's the smoke test target.
  const char *kTriangleClipSpace = R"WGSL(
struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) color : vec3<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vid : u32) -> VertexOut {
  // NDC coords: a triangle covering the top-right of the framebuffer.
  // We pick coordinates so that pixel (4,4) of an 8x8 target lands
  // squarely inside the triangle.
  var positions = array<vec2<f32>, 3>(
    vec2<f32>(-0.8, -0.8),
    vec2<f32>( 0.8, -0.8),
    vec2<f32>( 0.0,  0.8),
  );
  var colors = array<vec3<f32>, 3>(
    vec3<f32>(1.0, 0.0, 0.0),
    vec3<f32>(0.0, 1.0, 0.0),
    vec3<f32>(0.0, 0.0, 1.0),
  );
  var out : VertexOut;
  out.position = vec4<f32>(positions[vid], 0.0, 1.0);
  out.color = colors[vid];
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  return vec4<f32>(in.color, 1.0);
}
)WGSL";

  const char *kTriangleMVP = R"WGSL(
struct Uniforms {
  viewProj : mat4x4<f32>,
};

@group(0) @binding(0) var<uniform> u : Uniforms;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) normal : vec3<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  var out : VertexOut;
  out.position = u.viewProj * vec4<f32>(in.position, 1.0);
  out.normal = in.normal;
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let c = max(in.normal, vec3<f32>(0.0));
  return vec4<f32>(c, 1.0);
}
)WGSL";

  const char *kTriangleInstanced = R"WGSL(
struct Uniforms {
  viewProj : mat4x4<f32>,
};

@group(0) @binding(0) var<uniform> u : Uniforms;
@group(0) @binding(1) var<storage, read> bodies : array<vec4<f32>>;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) normal : vec3<f32>,
};

@vertex
fn vs_main(in : VertexIn, @builtin(instance_index) iid : u32) -> VertexOut {
  let bodyOffset : vec3<f32> = bodies[iid].xyz;
  let worldPos = vec4<f32>(in.position + bodyOffset, 1.0);
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.normal = in.normal;
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let c = max(in.normal, vec3<f32>(0.0));
  return vec4<f32>(c, 1.0);
}
)WGSL";

  const char *kTriangleTextured = R"WGSL(
struct Uniforms {
  viewProj : mat4x4<f32>,
};

@group(0) @binding(0) var<uniform> u : Uniforms;
@group(0) @binding(1) var tex : texture_2d<f32>;
@group(0) @binding(2) var samp : sampler;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  var out : VertexOut;
  out.position = u.viewProj * vec4<f32>(in.position, 1.0);
  out.uv = in.uv;
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  return textureSample(tex, samp, in.uv);
}
)WGSL";

  const char *kSolidLit = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
};

@group(0) @binding(0) var<uniform> u : Scene;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) worldNormal : vec3<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  // For correctness on non-uniform scales we'd want the inverse-
  // transpose of model; for uniform-scale Solids (the common case)
  // the upper-3x3 of model is enough. R3.4-step-5 uploads the
  // inverse-transpose as a second mat4 when shapes have non-uniform
  // scale.
  let n3 = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.worldNormal = normalize(n3);
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let L = normalize(u.light.xyz);
  let ambient = u.light.w;
  // Place an absolute clamp at 1.0 so a near-grazing surface
  // doesn't go negative when ambient + diffuse round-trip the
  // floating-point pipeline.
  let diff = max(dot(in.worldNormal, -L), 0.0);
  let intensity = clamp(ambient + diff, 0.0, 1.0);
  // R4 sRGB: lighting is computed in linear space (baseColor is a linear glTF factor). Encode the
  // final colour to sRGB for DISPLAY surfaces (the main pane + screenshots) — but NOT for sensor
  // RTT (the camera device), which must stay in the R5-landed linear space its controllers/ML
  // consumers expect. pad0.x > 0.5 selects linear output (set by the camera path); default 0 → sRGB.
  let lin = max(u.baseColor.rgb * intensity, vec3<f32>(0.0, 0.0, 0.0));
  if (u.pad0.x > 0.5) {
    return vec4<f32>(lin, u.baseColor.a);
  }
  return vec4<f32>(pow(lin, vec3<f32>(1.0 / 2.2)), u.baseColor.a);
}
)WGSL";

  // R4 material fidelity — kSolidLit + an albedo texture (baseColorMap) sampled at
  // the mesh UVs. Same Scene uniform @0; adds texture @1 + sampler @2.
  const char *kSolidLitTextured = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
};

@group(0) @binding(0) var<uniform> u : Scene;
@group(0) @binding(1) var albedoTex : texture_2d<f32>;
@group(0) @binding(2) var roughTex : texture_2d<f32>;
@group(0) @binding(3) var metalTex : texture_2d<f32>;
@group(0) @binding(4) var normalTex : texture_2d<f32>;
@group(0) @binding(5) var albedoSamp : sampler;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) worldNormal : vec3<f32>,
  @location(1) uv : vec2<f32>,
  @location(2) worldPos : vec3<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  let n3 = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.worldNormal = normalize(n3);
  out.uv = in.uv;
  out.worldPos = worldPos.xyz;
  return out;
}

// Tangent-space normal mapping WITHOUT precomputed mesh tangents (Schüler's method):
// reconstruct the tangent frame from screen-space derivatives of worldPos + uv. A flat
// normal map (0,0,1) returns Ngeo unchanged → byte-identical for draws with no normalMap.
fn perturbNormal(Ngeo : vec3<f32>, worldPos : vec3<f32>, uv : vec2<f32>, mapN : vec3<f32>) -> vec3<f32> {
  let dp1 = dpdx(worldPos);
  let dp2 = dpdy(worldPos);
  let duv1 = dpdx(uv);
  let duv2 = dpdy(uv);
  let dp2perp = cross(dp2, Ngeo);
  let dp1perp = cross(Ngeo, dp1);
  let T = dp2perp * duv1.x + dp1perp * duv2.x;
  let B = dp2perp * duv1.y + dp1perp * duv2.y;
  // Degenerate/constant UVs (an untextured CAD mesh has no UV gradient) → T,B collapse to ~0,
  // so inverseSqrt(0) = +inf and T*invmax = 0*inf = NaN, poisoning the normal → garbage lighting
  // (the black speckle all over the robot in the shadowed material shader). Fall back to the
  // geometric normal when there's no valid tangent frame; real textured meshes keep normal mapping.
  let tbMax = max(dot(T, T), dot(B, B));
  if (tbMax < 1e-12) { return Ngeo; }
  let invmax = inverseSqrt(tbMax);
  let M = mat3x3<f32>(T * invmax, B * invmax, Ngeo);
  return normalize(M * mapN);
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let L = normalize(u.light.xyz);
  let ambient = u.light.w;
  // R4 PBR: perturb the geometric normal by the tangent-space normalMap. A flat map
  // (128,128,255 → (0,0,1)) leaves N = Ngeo, so no-normalMap draws are byte-identical.
  let Ngeo = normalize(in.worldNormal);
  let mapN = normalize(textureSample(normalTex, albedoSamp, in.uv).xyz * 2.0 - vec3<f32>(1.0));
  let N = perturbNormal(Ngeo, in.worldPos, in.uv, mapN);
  let diff = max(dot(N, -L), 0.0);
  let intensity = clamp(ambient + diff, 0.0, 1.0);
  // R4 sRGB: the baseColorMap is sRGB-encoded → linearize before lighting (~pow 2.2). baseColor
  // is a linear glTF factor (not linearized). The final colour is sRGB-encoded on output below.
  let albedoTexLin = pow(textureSample(albedoTex, albedoSamp, in.uv).rgb, vec3<f32>(2.2));
  let albedo = albedoTexLin * u.baseColor.rgb;
  // R4 PBR: metalness — metals take NO diffuse and tint their specular by the albedo; a
  // dielectric keeps full diffuse + a white highlight. metalTex.r in [0,1]; default-BLACK
  // (0) → dielectric → byte-identical to the pre-metalness path.
  let metal = clamp(textureSample(metalTex, albedoSamp, in.uv).r, 0.0, 1.0);
  let diffuseCol = albedo * (1.0 - metal);  // metals take no diffuse
  var col = diffuseCol * intensity;
  // R4 PBR: Cook-Torrance GGX microfacet specular (D·G·F / 4·NdotV·NdotL), replacing the
  // ad-hoc Blinn-Phong. pad1.w = 1-roughnessFactor; the per-pixel roughnessMap modulates it
  // (effRough = roughnessFactor × map.r). F0 = 0.04 for dielectrics, albedo for metals — so
  // the Fresnel term carries the metallic tint (no separate specColor). Camera pos in pad0.yzw.
  // Default-white roughness leaves roughFactor unchanged. GGX D, Smith G (Schlick-GGX,
  // direct-lighting k), Schlick Fresnel F.
  let roughFactor = clamp(1.0 - u.pad1.w, 0.0, 1.0);
  let texRough = textureSample(roughTex, albedoSamp, in.uv).r;
  let effRough = clamp(roughFactor * texRough, 0.045, 1.0);  // floor avoids the D singularity
  let V = normalize(u.pad0.yzw - in.worldPos);
  let Ld = -L;
  let H = normalize(V + Ld);
  let NdotL = max(dot(N, Ld), 0.0);
  let NdotV = max(dot(N, V), 1e-4);
  let NdotH = max(dot(N, H), 0.0);
  let VdotH = max(dot(V, H), 0.0);
  let alpha = effRough * effRough;
  let a2 = alpha * alpha;
  let dn = NdotH * NdotH * (a2 - 1.0) + 1.0;
  let D = a2 / (3.14159265 * dn * dn);
  let kg = (effRough + 1.0) * (effRough + 1.0) / 8.0;
  let gv = NdotV / (NdotV * (1.0 - kg) + kg);
  let gl = NdotL / (NdotL * (1.0 - kg) + kg);
  let G = gv * gl;
  let F0 = mix(vec3<f32>(0.04, 0.04, 0.04), albedo, metal);
  let F = F0 + (vec3<f32>(1.0, 1.0, 1.0) - F0) * pow(1.0 - VdotH, 5.0);
  let specBRDF = (D * G * F) / max(4.0 * NdotV * NdotL, 1e-4);
  col = col + specBRDF * NdotL;
  // R4 sRGB: encode the linear-lit colour to sRGB for display (matches kSolidLit's output).
  return vec4<f32>(pow(max(col, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(1.0 / 2.2)), u.baseColor.a);
}
)WGSL";

  // R4 lighting-convergence rung 1: kSolidLitTextured + cast shadows. The full material path
  // (albedo/roughness/metalness/normalMap + GGX + sRGB, bindings 0–5) PLUS a shadow term sampled
  // from a light-space depth map (bindings 6–7) via a SHARED light-VP uniform (binding 8 — kept
  // separate so the per-draw Scene uniform @0 stays the existing 192 B layout). Lighting is split
  // into UNSHADOWED ambient + SHADOWED direct (diffuse + GGX specular), so shadows darken only the
  // direct contribution. strength 0 → byte-identical to kSolidLitTextured. Pairs with a two-pass
  // render (light depth map → this lit pass) to converge the wgpu pane toward WREN's shadowing.
  const char *kSolidLitTexturedShadow = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  // P4 (WREN ambient parity) re-uses two slots the per-frame LightU hoist left dead:
  //   light.xyz = WREN's phong ambient product (Lights.ambientLight x material.ambient),
  //   light.w   = ambient MODE (0 = analytic sky/hemisphere, 1 = WREN phong light-ambient),
  //   pad0.x    = Background.luminosity x PBRAppearance.IBLStrength (WREN's premultiply).
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
  // TextureTransform as a 2D affine: uv' = (dot(uvA.xy,uv), dot(uvA.zw,uv)) + uvB.xy.
  // Identity (1,0,0,1)/(0,0) when the appearance has none. Grass{scale 40 40} on a bare
  // Plane was sampled untransformed -> one tile smeared across 200 m (white-streak bug).
  uvA       : vec4<f32>,
  uvB       : vec4<f32>,
  // Pads Scene to exactly 256 B = the CPU slot stride, so array<Scene> in the storage buffer
  // indexes the packed slots directly (element stride == struct size).
  padStrideA : vec4<f32>,
  padStrideB : vec4<f32>,
};
// Extra scene light (beyond the shadowed sun): PointLight / SpotLight / additional
// DirectionalLights, unshadowed. Layout is 4 vec4s = 64 B, mirrored by the CPU pack
// in clearAndDrawSceneTexturedShadowed.
struct ExtraLight {
  posType  : vec4<f32>,   // xyz = world position (point/spot) or direction (dir), w = type: 0 dir, 1 point, 2 spot
  colorRad : vec4<f32>,   // rgb = colour x intensity, w = cutoff radius (<=0: no cutoff; point/spot only)
  atten    : vec4<f32>,   // xyz = attenuation (constant, linear, quadratic), w = cos(cutOffAngle) for spot
  spotDir  : vec4<f32>,   // xyz = world spot direction, w = cos(beamWidth)
};
struct LightU {
  lightViewProj : mat4x4<f32>,
  shadowParams  : vec4<f32>,   // x = strength [0,1], y = depth bias, z = hemisphere-IBL enabled (>0.5), w = day-night direct scale (0 = off)
  skyColor      : vec4<f32>,   // rgb = sky/zenith ambient colour, w = ambient intensity scale
  groundColor   : vec4<f32>,   // rgb = ground ambient colour (xyz), w unused
  upDir         : vec4<f32>,   // xyz = world up direction (for the hemisphere blend), w unused
  fogParams     : vec4<f32>,   // rgb = display-space fog colour, w = exp density (0 = no fog)
  extraMeta     : vec4<f32>,   // x = number of extra lights [0..8], yzw = sun linear energy
  extras        : array<ExtraLight, 8>,
  // Cascaded shadow maps (extraMeta.y > 0): per-cascade fitted light view-projs, the far
  // view-depth boundary of each cascade, and each cascade's world-space texel size (drives the
  // per-cascade normal offset). All zero in the legacy single-map configuration.
  csmVP         : array<mat4x4<f32>, 4>,
  cascadeSplits : vec4<f32>,
  cascadeTexel  : vec4<f32>,   // xyz = per-cascade world texel size, w = cascade count (0 = legacy)
  // Per-frame SHARED values, hoisted out of the per-draw Scene slots so a static draw's slot only
  // changes when its model matrix does (the delta-staging win): camera view-proj (was
  // Scene.viewProj), sun dir + ambient scalar (was Scene.light), camera world pos (was
  // Scene.pad0.yzw). The Scene slot bytes they replace are dead, not removed — layout unchanged.
  sceneViewProj : mat4x4<f32>,
  sunDirAmbient : vec4<f32>,
  camPos        : vec4<f32>,
  // Derived-sky IBL palette (physically-scattered sky mode): the analytic-IBL env colours,
  // marched from the SAME atmosphere the dome renders, so metals reflect the actual sky.
  // iblZenith.w > 0.5 = valid; zero keeps the hand-tuned palette constants (bit-compat).
  iblZenith     : vec4<f32>,
  iblHorizon    : vec4<f32>,
  // PCSS contact-hardening: x = softness (world penumbra metres per metre of blocker gap,
  // ~tan of the effective sun angular radius), y = min PCF spread (texels), z = max spread,
  // w = enabled. cascadeDepthSpan = each cascade's light-frustum depth range in metres
  // (recovered from the ortho m22 like the texel size is from m00) — converts a stored-depth
  // difference into a world-space blocker gap.
  pcssParams    : vec4<f32>,
  cascadeDepthSpan : vec4<f32>,
  // OmniLight probe volume: origin.xyz + valid flag; invExtent.xyz (1/(dims*spacing)) + z dim.
  // Valid -> the ambient term is the path-traced irradiance field instead of the flat hemisphere.
  omniOrigin    : vec4<f32>,
  omniParams    : vec4<f32>,
  omniMisc      : vec4<f32>,   // x = fade-in blend [0,1] (a fresh bake settles in, no pop)
  // Specular probe: parallax-corrected traced cubemap. cubeCenter.w = valid flag.
  omniCubeCenter : vec4<f32>,
  omniAabbMin    : vec4<f32>,
  omniAabbMax    : vec4<f32>,
};

@group(0) @binding(0) var<storage, read> slots : array<Scene>;
@group(0) @binding(1) var albedoTex : texture_2d<f32>;
@group(0) @binding(2) var roughTex : texture_2d<f32>;
@group(0) @binding(3) var metalTex : texture_2d<f32>;
@group(0) @binding(4) var normalTex : texture_2d<f32>;
@group(0) @binding(5) var albedoSamp : sampler;
@group(0) @binding(6) var shadowTex : texture_2d_array<f32>;
@group(0) @binding(7) var shadowSamp : sampler;
@group(0) @binding(8) var<uniform> lu : LightU;
@group(0) @binding(9) var omniTex : texture_3d<f32>;   // 4 z-slabs of SH {L00+w, L1x, L1y, L1z}
@group(0) @binding(10) var omniSamp : sampler;
@group(0) @binding(11) var omniCube : texture_cube<f32>;  // traced specular probe (3 mips)
// W3/P3: the Pen paint layer (OmPaintTexture). A draw with no Pen binds a 1x1 TRANSPARENT texel,
// so the mix below is the identity and the frame is byte-identical to the pre-P3 build.
@group(0) @binding(12) var penTex : texture_2d<f32>;

struct VertexIn {
  @builtin(instance_index) slot : u32,   // firstInstance = the draw's Scene-slot index
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};
struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) worldNormal : vec3<f32>,
  @location(1) uv : vec2<f32>,
  @location(2) worldPos : vec3<f32>,
  @location(3) viewDepth : f32,
  @location(4) @interpolate(flat) slot : u32,
  // RAW mesh UV, i.e. `uv` WITHOUT the TextureTransform. WREN feeds the pen layer
  // `penTexUv = vUnwrappedTexCoord` (pbr.vert:36) while the material maps get the transformed
  // `texUv`, so the pen must not inherit a Grass{scale 40 40}-style tiling transform.
  @location(5) penUv : vec2<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let u = slots[in.slot];
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  let n3 = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
  var out : VertexOut;
  out.position = lu.sceneViewProj * worldPos;
  out.worldNormal = normalize(n3);
  out.uv = vec2<f32>(dot(u.uvA.xy, in.uv), dot(u.uvA.zw, in.uv)) + u.uvB.xy;
  out.penUv = in.uv;               // W3/P3: pen samples the untransformed mesh UV
  out.worldPos = worldPos.xyz;
  out.viewDepth = out.position.w;  // camera view depth — the cascade selector
  out.slot = in.slot;
  return out;
}

fn perturbNormal(Ngeo : vec3<f32>, worldPos : vec3<f32>, uv : vec2<f32>, mapN : vec3<f32>) -> vec3<f32> {
  let dp1 = dpdx(worldPos);
  let dp2 = dpdy(worldPos);
  let duv1 = dpdx(uv);
  let duv2 = dpdy(uv);
  let dp2perp = cross(dp2, Ngeo);
  let dp1perp = cross(Ngeo, dp1);
  let T = dp2perp * duv1.x + dp1perp * duv2.x;
  let B = dp2perp * duv1.y + dp1perp * duv2.y;
  // Degenerate/constant UVs (an untextured CAD mesh has no UV gradient) → T,B collapse to ~0,
  // so inverseSqrt(0) = +inf and T*invmax = 0*inf = NaN, poisoning the normal → garbage lighting
  // (the black speckle all over the robot in the shadowed material shader). Fall back to the
  // geometric normal when there's no valid tangent frame; real textured meshes keep normal mapping.
  let tbMax = max(dot(T, T), dot(B, B));
  if (tbMax < 1e-12) { return Ngeo; }
  let invmax = inverseSqrt(tbMax);
  let M = mat3x3<f32>(T * invmax, B * invmax, Ngeo);
  return normalize(M * mapN);
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let u = slots[in.slot];
  let L = normalize(lu.sunDirAmbient.xyz);
  let ambient = lu.sunDirAmbient.w;
  let Ngeo = normalize(in.worldNormal);
  let mapN = normalize(textureSample(normalTex, albedoSamp, in.uv).xyz * 2.0 - vec3<f32>(1.0));
  let N = perturbNormal(Ngeo, in.worldPos, in.uv, mapN);
  let Ld = -L;
  let NdotL = max(dot(N, Ld), 0.0);
  // hdrMode (LightU groundColor.w > 0.5): the LINEAR-LIGHT arm — sRGB-decode the albedo, light
  // with real sun energy, keep ambient/emissive linear, skip the display encode at the end (the
  // tonemap pass owns the display transform + dither). Flag 0 = legacy display-referred math,
  // bit-exact.
  let hdrMode = lu.groundColor.w > 0.5;
  // LDR arm keeps the WREN-parity RAW albedo (no sRGB decode — the legacy WREN texture path lights
  // the raw texel and gamma-encodes the output; decoding here made wgpu's textured city
  // dark+saturated vs WREN's near-white, the dominant failing pixels in the parity gate).
  let albedoTexS = textureSample(albedoTex, albedoSamp, in.uv);
  // ALPHA-TEST CUTOUT (foliage leaf cards, fence cutouts — the mesh-tree library's oak/pine leaf
  // textures): textured alpha below 0.5 discards the fragment. Opaque textures decode alpha 1
  // everywhere, so ordinary materials never take this branch.
  if (albedoTexS.a < 0.5) {
    discard;
  }
  var albedoTexLin = albedoTexS.rgb;
  if (hdrMode) {
    albedoTexLin = pow(albedoTexLin, vec3<f32>(2.2, 2.2, 2.2));  // WREN SRGBtoLINEAR (2.2 approx)
  }
  var albedo = albedoTexLin * u.baseColor.rgb;
  // P4: the TEXTURE-only albedo, i.e. WITHOUT baseColor. WREN's phong path forces
  // Material.diffuseColor to white the moment an ImageTexture is present and multiplies the
  // ambient by the texture alone (phong.frag:214, OmMaterial.cpp:148-149), and the
  // diffuseColor of an UNTEXTURED material is already folded into the CPU-side ambient product.
  // Using `albedo` here would apply diffuseColor twice on one arm and zero times on the other.
  var albedoTexOnly = albedoTexLin;
  // W3/P3 Pen: mix the ink over the base colour by its own alpha -- the same operation, in the
  // same place, as WREN (pbr.frag:522-525 mixes into baseColor BEFORE the BRDF, so ink is LIT,
  // not emissive). Decoded to linear alongside the albedo under hdrMode so the two are mixed in
  // ONE space. A draw with no Pen binds the 1x1 transparent default -> penS.a == 0 -> no branch.
  let penS = textureSample(penTex, albedoSamp, in.penUv);
  if (penS.a > 0.0) {
    var penLin = penS.rgb;
    if (hdrMode) {
      penLin = pow(penLin, vec3<f32>(2.2, 2.2, 2.2));
    }
    albedo = mix(albedo, penLin, penS.a);
    // The ink tints the ambient too -- WREN mixes it into texColor, which multiplies the whole
    // (emissive + ambient + diffuse + specular) sum (phong.frag:206-208, 214).
    albedoTexOnly = mix(albedoTexOnly, penLin, penS.a);
  }
  // P4 ambient model selector, read once (see the Scene struct above).
  let wrenAmbMode = u.light.w > 0.5;
  let iblScale = u.pad0.x;
  let metal = clamp(textureSample(metalTex, albedoSamp, in.uv).r, 0.0, 1.0);

  // Shadow term (PCF) from the cascaded light-space depth maps. strength 0 → fully lit.
  // extraMeta.y = cascade count: 0 keeps the legacy single fitted map (layer 0, lightViewProj,
  // the calibrated 0.12 m normal offset); N > 0 selects the cascade by camera view depth and
  // scales the normal offset by that cascade's own world texel size.
  var shadow = 1.0;
  let strength = lu.shadowParams.x;
  if (strength > 0.0) {
    let nc = i32(lu.cascadeTexel.w + 0.5);
    var ci = 0;
    var vpS = lu.lightViewProj;
    var nOff = 0.12;
    if (nc > 0) {
      if (in.viewDepth >= lu.cascadeSplits.y) { ci = 2; }
      else if (in.viewDepth >= lu.cascadeSplits.x) { ci = 1; }
      ci = min(ci, nc - 1);
      vpS = lu.csmVP[ci];
      nOff = 1.5 * lu.cascadeTexel[ci];
    }
    // NORMAL-OFFSET shadows: shift the receiver point ~1.5 shadow texels (≈12 cm on the 45 m
    // fitted frustum) along the geometric normal before projecting into light space. The PCF taps
    // compare against texels up to 3 away; on a slanted receiver (flat floor + low sun) those
    // legitimately differ by ~0.5 m of depth, which bias scaling cannot cover without metre-scale
    // peter-panning — the texel-grain false-occlusion BLOCKS seen on spot.omniworld. The offset moves
    // the receiver off its own surface so every tap clears it.
    let shadowPos = in.worldPos + Ngeo * nOff;
    let lc = vpS * vec4<f32>(shadowPos, 1.0);
    if (lc.w > 0.0) {
      let ndc = lc.xyz / lc.w;
      let suv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
      if (suv.x >= 0.0 && suv.x <= 1.0 && suv.y >= 0.0 && suv.y <= 1.0 && ndc.z <= 1.0) {
        // DEBUG (bias passed NEGATIVE): visualize the raw depth-compare error field instead of
        // shading — red = fragment deeper than stored (would occlude), green = shallower, scaled
        // x20. The definitive view of WHY a comparison misbehaves (sign/shape/magnitude).
        if (lu.shadowParams.y < 0.0) {
          let st = textureSampleLevel(shadowTex, shadowSamp, suv, ci, 0.0).r;
          let err = (ndc.z - st) * 20.0;
          return vec4<f32>(clamp(err, 0.0, 1.0), clamp(-err, 0.0, 1.0), select(0.0, 1.0, st > 0.999), 1.0);
        }
        let bias = max(lu.shadowParams.y, 1e-4);
        let texelUV = 1.0 / vec2<f32>(textureDimensions(shadowTex));
        // RECEIVER-PLANE DEPTH BIAS: each PCF tap compares against a texel stored up to ~3 texels
        // away; on a receiver slanted in LIGHT space (flat floor + low sun: spot's 23-degree sun
        // slopes ~19 cm of depth PER TEXEL) the plane itself crosses the bias budget, painting
        // texel-grain false-occlusion blocks no constant/normal offset can cure. Compute the
        // plane's ndc.z gradient over the map UV from the geometric normal pushed through the
        // light projection (direction transform), and bias EVERY tap by its own expected depth.
        // u = 0.5+0.5x, v = 0.5-0.5y => dz/duv = (-2 nx/nz, +2 ny/nz). Clamped against silhouette
        // blow-up at extreme grazing.
        let nClip = vpS * vec4<f32>(Ngeo, 0.0);
        let nzSafe = sign(nClip.z) * max(abs(nClip.z), 1e-4);
        let dzduv = clamp(vec2<f32>(-2.0 * nClip.x / nzSafe, 2.0 * nClip.y / nzSafe),
                          vec2<f32>(-10.0, -10.0), vec2<f32>(10.0, 10.0));
        // PCSS contact hardening (pcssParams.w > 0.5, cascades only): a 16-tap blocker search
        // over the max-penumbra window estimates the average blocker distance; the penumbra
        // width — (receiver - blocker) gap in metres x softness — then sets the PCF spread, so
        // shadows are razor-sharp at contact and soften with distance from the caster. No
        // blockers in the window -> fully lit, and the 25-tap PCF is skipped entirely.
        // pcssParams.w <= 0.5 (or the legacy single map) keeps the fixed 1.5-texel spread
        // bit-exactly.
        var spread = 1.5;
        var doPcf = true;
        if (lu.pcssParams.w > 0.5 && nc > 0) {
          let searchR = lu.pcssParams.z * 2.0 * texelUV;
          var bSum = 0.0;
          var bCnt = 0.0;
          for (var by = -2; by <= 1; by = by + 1) {
            for (var bx = -2; bx <= 1; bx = bx + 1) {
              let boff = (vec2<f32>(f32(bx), f32(by)) + vec2<f32>(0.5, 0.5)) * 0.5 * searchR;
              let zb = ndc.z + dzduv.x * boff.x + dzduv.y * boff.y;
              let sb = textureSampleLevel(shadowTex, shadowSamp, suv + boff, ci, 0.0).r;
              if (zb > sb + bias) {
                bSum = bSum + sb;
                bCnt = bCnt + 1.0;
              }
            }
          }
          if (bCnt < 0.5) {
            doPcf = false;  // nothing occludes inside the max penumbra — fully lit
          } else {
            let gapWorld = max(ndc.z - bSum / bCnt, 0.0) * lu.cascadeDepthSpan[ci];
            let penumbraTexels = gapWorld * lu.pcssParams.x / max(lu.cascadeTexel[ci], 1e-6);
            spread = clamp(penumbraTexels * 0.5, lu.pcssParams.y, lu.pcssParams.z);
          }
        }
        var occ = 0.0;
        if (doPcf) {
          for (var dy = -2; dy <= 2; dy = dy + 1) {
            for (var dx = -2; dx <= 2; dx = dx + 1) {
              let off = vec2<f32>(f32(dx), f32(dy)) * spread * texelUV;
              let zExp = ndc.z + dzduv.x * off.x + dzduv.y * off.y;
              let stored = textureSampleLevel(shadowTex, shadowSamp, suv + off, ci, 0.0).r;
              if (zExp > stored + bias) {
                occ = occ + 1.0;
              }
            }
          }
          shadow = 1.0 - strength * (occ / 25.0);
        }
      }
    }
  }

  // GGX direct specular (Cook-Torrance), same as kSolidLitTextured.
  let roughFactor = clamp(1.0 - u.pad1.w, 0.0, 1.0);
  let texRough = textureSample(roughTex, albedoSamp, in.uv).r;
  let effRough = clamp(roughFactor * texRough, 0.045, 1.0);
  let V = normalize(lu.camPos.xyz - in.worldPos);
  let Hh = normalize(V + Ld);
  let NdotV = max(dot(N, V), 1e-4);
  let NdotH = max(dot(N, Hh), 0.0);
  let VdotH = max(dot(V, Hh), 0.0);
  let a = effRough * effRough;
  let a2 = a * a;
  let dn = NdotH * NdotH * (a2 - 1.0) + 1.0;
  let D = a2 / (3.14159265 * dn * dn);
  let kg = (effRough + 1.0) * (effRough + 1.0) / 8.0;
  let G = (NdotV / (NdotV * (1.0 - kg) + kg)) * (NdotL / (NdotL * (1.0 - kg) + kg));
  let F0 = mix(vec3<f32>(0.04, 0.04, 0.04), albedo, metal);
  let F = F0 + (vec3<f32>(1.0, 1.0, 1.0) - F0) * pow(1.0 - VdotH, 5.0);
  let specBRDF = (D * G * F) / max(4.0 * NdotV * NdotL, 1e-4);

  // Unshadowed ambient + shadowed direct (diffuse + specular).
  // R4 lighting convergence: hemisphere-IBL ambient — matches OmniSimSky's directional sky fill
  // (sky colour from above, ground colour from below, blended by the surface normal's up-component),
  // so shadowed regions take a sky tint instead of a flat grey. shadowParams.z <= 0.5 → flat scalar
  // ambient (byte-identical to the pre-hemisphere path; the default when no sky params are supplied).
  let diffuse = albedo * (1.0 - metal);
  var ambTerm = vec3<f32>(ambient, ambient, ambient);
  var omniHit = false;
  var omniAmb = vec3<f32>(0.0, 0.0, 0.0);
  // OMNILIGHT (hdrMode + a baked volume): the PATH-TRACED irradiance field — sky visibility,
  // wall-colour bleed, interior darkening, emissive bounce — sampled with one trilinear read per
  // SH slab. Coefficients are premultiplied by probe validity; dividing by the interpolated
  // weight renormalises around probes buried inside geometry. The result MIXES over the
  // hemisphere ambient by omniMisc.x, so a fresh bake fades in instead of popping.
  if (hdrMode && lu.omniOrigin.w > 0.001 && lu.omniMisc.x > 0.001) {
    // The normal bias is a FULL probe cell (omniOrigin.w carries it in metres): pushing the
    // sample a cell off the surface makes wrong-side probes lose their trilinear weight — the
    // cure for interior light leaking through thin walls.
    let samplePos = in.worldPos + Ngeo * lu.omniOrigin.w;
    let uvw = (samplePos - lu.omniOrigin.xyz) * lu.omniParams.xyz;
    if (all(uvw >= vec3<f32>(0.0, 0.0, 0.0)) && all(uvw <= vec3<f32>(1.0, 1.0, 1.0))) {
      let dz = lu.omniParams.w;
      let zSub = clamp(uvw.z, 0.5 / dz, 1.0 - 0.5 / dz) * 0.25;
      let s0 = textureSampleLevel(omniTex, omniSamp, vec3<f32>(uvw.x, uvw.y, zSub), 0.0);
      if (s0.a > 0.05) {
        let s1 = textureSampleLevel(omniTex, omniSamp, vec3<f32>(uvw.x, uvw.y, 0.25 + zSub), 0.0).rgb;
        let s2 = textureSampleLevel(omniTex, omniSamp, vec3<f32>(uvw.x, uvw.y, 0.50 + zSub), 0.0).rgb;
        let s3 = textureSampleLevel(omniTex, omniSamp, vec3<f32>(uvw.x, uvw.y, 0.75 + zSub), 0.0).rgb;
        let inv = 1.0 / s0.a;
        omniAmb = max((s0.rgb + s1 * N.x + s2 * N.y + s3 * N.z) * inv, vec3<f32>(0.0, 0.0, 0.0));
        omniHit = true;
      }
    }
  }
  if (lu.shadowParams.z > 0.5) {
    let upd = normalize(lu.upDir.xyz);
    let t = clamp(dot(N, upd) * 0.5 + 0.5, 0.0, 1.0);
    let hemiCol = mix(lu.groundColor.rgb, lu.skyColor.rgb, t);
    ambTerm = hemiCol * lu.skyColor.w;
    if (hdrMode) {
      // Linear-light ambient: decode the display-tuned hemisphere colours; 0.7 is the linear
      // ambient level (the A/B knob) that puts shadow floors where WREN's GTAO-lit shadows sit.
      ambTerm = pow(max(hemiCol, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(2.2, 2.2, 2.2)) * 0.7;
    }
  } else if (hdrMode) {
    ambTerm = vec3<f32>(ambient * ambient, ambient * ambient, ambient * ambient);  // ~2.2 decode of the flat scalar
  }
  if (omniHit) {
    ambTerm = mix(ambTerm, omniAmb, clamp(lu.omniMisc.x, 0.0, 1.0));
  }
  // P4: `Background.luminosity x PBRAppearance.IBLStrength` -- WREN's ONE premultiplied ambient
  // scalar (pbr.frag:316-318), applied to the diffuse ambient here and to the specular ambient
  // below, exactly as WREN applies it to both. 1.0 (the field defaults) = unchanged.
  var ambientPart = diffuse * (ambTerm * iblScale);
  if (wrenAmbMode) {
    // WREN's phong arm: the ambient is a LIGHT product, not a sky product. The whole
    // Lights.ambientLight x material.ambient term arrives premultiplied from the CPU, so this is
    // literally phong.frag's `texColor * ambientTotal` -- no hemisphere, no probe, no sky tint.
    ambientPart = albedoTexOnly * u.light.xyz;
  }
  // Day-night: shadowParams.w (0 = legacy off → byte-identical) scales the DIRECT term only —
  // 1.0 in full day (unchanged), → 0 as the sun sets, so geometry darkens with the sky dome.
  var directScale = 1.0;
  if (lu.shadowParams.w > 0.001) {
    directScale = lu.shadowParams.w;
  }
  // hdrMode: real sun energy (colour x intensity from extraMeta.yzw); LDR: the legacy unit-white sun.
  let sunE = select(vec3<f32>(1.0, 1.0, 1.0), lu.extraMeta.yzw, hdrMode);
  let directPart = (diffuse * NdotL + specBRDF * NdotL) * sunE * shadow * directScale;
  // Extra lights (multi-light support): PointLight / SpotLight / additional DirectionalLights,
  // UNSHADOWED (only the sun owns the shadow map). Diffuse + a GGX-D specular lobe per light,
  // with the node's own attenuation coefficients and Webots radius-cutoff semantics. extraMeta.x
  // == 0 (the default when no extras are harvested) keeps this loop dead → prior output unchanged.
  var extraPart = vec3<f32>(0.0, 0.0, 0.0);
  let extraCount = i32(lu.extraMeta.x + 0.5);
  for (var li = 0; li < 8; li = li + 1) {
    if (li >= extraCount) { break; }
    let el = lu.extras[li];
    var Le = vec3<f32>(0.0, 0.0, 1.0);
    var att = 1.0;
    if (el.posType.w < 0.5) {  // additional directional light: posType.xyz is the direction
      Le = normalize(-el.posType.xyz);
    } else {                   // point / spot: posType.xyz is the world position
      let toL = el.posType.xyz - in.worldPos;
      let d = length(toL);
      if (el.colorRad.w > 0.0 && d > el.colorRad.w) { continue; }  // beyond the node's radius
      Le = toL / max(d, 1e-4);
      att = 1.0 / max(el.atten.x + el.atten.y * d + el.atten.z * d * d, 1e-3);
      if (el.posType.w > 1.5) {  // spot cone: fade from beamWidth to cutOffAngle
        let cosA = dot(-Le, normalize(el.spotDir.xyz));
        let cosCut = el.atten.w;
        if (cosA < cosCut) { continue; }
        let cosBeam = el.spotDir.w;
        att = att * clamp((cosA - cosCut) / max(cosBeam - cosCut, 1e-4), 0.0, 1.0);
      }
    }
    let NdotLe = max(dot(N, Le), 0.0);
    if (NdotLe <= 0.0) { continue; }
    let He = normalize(V + Le);
    let NdotHe = max(dot(N, He), 0.0);
    let dne = NdotHe * NdotHe * (a2 - 1.0) + 1.0;
    let De = a2 / (3.14159265 * dne * dne);
    let specE = De * 0.25 * F0;
    extraPart = extraPart + (diffuse + specE) * NdotLe * att * el.colorRad.rgb;
  }
  // ANALYTIC SPECULAR IBL (hdrMode only): the sky dome is analytic, so instead of baking an
  // environment cube (WREN's approach) we evaluate the dome's own linear palette in the per-pixel
  // REFLECTION direction — sky above, ground bounce below, plus a roughness-sharpened sun glint
  // with real sun energy. Fresnel-weighted: metals get their tinted environment (their diffuse is
  // zero, so this IS their ambient — the "copper looks copper" fix); dielectrics get a small
  // grazing sheen. Zero new uniforms; day-night follows the dome's own dimmer.
  var iblSpec = vec3<f32>(0.0, 0.0, 0.0);
  if (hdrMode) {
    let R = reflect(-V, N);
    let upd2 = normalize(lu.upDir.xyz);
    let tR = clamp(dot(R, upd2), -1.0, 1.0);
    let dayIbl = select(1.0, lu.shadowParams.w, lu.shadowParams.w > 0.001);
    // Env palette: derived from the scattered sky when valid (metals reflect the ACTUAL sky,
    // warm at sunset, blue at noon); else kSkyAtmosphere's day palette decoded to linear.
    var zenithLin = vec3<f32>(0.047, 0.157, 0.516);
    var horizonLin = vec3<f32>(0.349, 0.457, 0.648);
    if (lu.iblZenith.w > 0.5) {
      zenithLin = lu.iblZenith.xyz;
      horizonLin = lu.iblHorizon.xyz;
    }
    let nightLin = vec3<f32>(0.0005, 0.0012, 0.0035);
    var envUp = mix(horizonLin, zenithLin, pow(clamp(tR, 0.0, 1.0), 0.55));
    envUp = mix(nightLin, envUp, dayIbl);
    let groundLin = pow(max(lu.groundColor.rgb, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(2.2, 2.2, 2.2));
    var env = mix(groundLin * 0.6, envUp, smoothstep(-0.15, 0.15, tR));
    // TRACED SPECULAR PROBE: when OmniLight baked a cubemap, reflections come from the actual
    // scene (parallax-corrected against the scene AABB; mip by roughness) instead of the sky
    // palette — metals and glass mirror real surroundings, on- OR off-screen. SSR still wins
    // where it hits (added later in screen space); this replaces only the env fallback.
    if (lu.omniCubeCenter.w > 0.5) {
      let invR = 1.0 / select(R, vec3<f32>(1e-5, 1e-5, 1e-5), abs(R) < vec3<f32>(1e-5, 1e-5, 1e-5));
      let tA = (lu.omniAabbMin.xyz - in.worldPos) * invR;
      let tB = (lu.omniAabbMax.xyz - in.worldPos) * invR;
      let tMax3 = max(tA, tB);
      let tFar = max(min(min(tMax3.x, tMax3.y), tMax3.z), 0.05);
      let hitP = in.worldPos + R * tFar;
      let cdir = normalize(hitP - lu.omniCubeCenter.xyz);
      let mip = clamp(effRough * 5.0, 0.0, 2.0);
      env = textureSampleLevel(omniCube, omniSamp, cdir, mip).rgb;
    }
    // sun glint: tight mirror streak on smooth surfaces, broad wash on rough ones
    let sunTo = normalize(-lu.sunDirAmbient.xyz);
    let glintPow = mix(512.0, 8.0, effRough);
    let glint = pow(max(dot(R, sunTo), 0.0), glintPow) * dayIbl;
    let envR = env + lu.extraMeta.yzw * glint;
    let Fibl = F0 + (vec3<f32>(1.0, 1.0, 1.0) - F0) * pow(1.0 - NdotV, 5.0);
    let specScale = (1.0 - effRough) * (1.0 - effRough) * 0.9 + 0.05;
    iblSpec = envR * Fibl * specScale * iblScale;
  }
  // WREN's phong shader has NO image-based specular at all (specularTotal comes from the light
  // loops only, and is forced to zero on a textured material), so the phong arm drops it.
  if (wrenAmbMode) {
    iblSpec = vec3<f32>(0.0, 0.0, 0.0);
  }
  // Self-emission (pad1.xyz = emissiveColor × intensity): independent of sun/shadows/day-night —
  // shop strips, traffic lights and headlights stay lit at night. hdrMode decodes it to linear
  // (an intensity >1 survives the pow and reaches the AgX white point — real bloom sources).
  let emis = select(u.pad1.xyz, pow(max(u.pad1.xyz, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(2.2, 2.2, 2.2)), hdrMode);
  let col = ambientPart + directPart + extraPart + iblSpec + emis;
  // WREN-exact exponential fog (fog.frag): factor = exp2(-density·d), blend = pow(1-factor, 2.2).
  // Mixed in LINEAR space BEFORE the display encode — WREN fogs inside its HDR pipeline, so the
  // fade composes dark/moody; mixing the raw pale fog colour in display space made distant
  // streets/grass vanish toward WHITE (the user-visible zoom-out fade).
  var colOut = col;
  if (lu.fogParams.w > 0.0) {
    let dist = length(lu.camPos.xyz - in.worldPos);
    let fogF = pow(clamp(1.0 - exp2(-lu.fogParams.w * dist), 0.0, 1.0), 2.2);
    let fogLin = pow(max(lu.fogParams.xyz, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(2.2));
    colOut = mix(colOut, fogLin, fogF);
  }
  // hdrMode writes LINEAR radiance into the RGBA16F target — no encode, no dither here; the AgX
  // tonemap pass owns both. ALPHA on OPAQUE pixels carries REFLECTIVITY for the SSR pass (no
  // blend equation reads it there); translucent draws keep real alpha — src-over needs it.
  if (hdrMode) {
    var outA = u.baseColor.a;
    if (u.baseColor.a >= 0.999) {
      outA = clamp((1.0 - effRough) * (1.0 - effRough) * (0.15 + 0.85 * metal), 0.0, 1.0);
    }
    return vec4<f32>(max(colOut, vec3<f32>(0.0, 0.0, 0.0)), outA);
  }
  var outRgb = pow(max(colOut, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(1.0 / 2.2));
  // ±½-LSB hash dither before the 8-bit write: faint wide gradients (a broad rough-surface GGX
  // sheen, the sky dome) otherwise quantize into visible banding contours — WREN avoids this via
  // its HDR pipeline + dithered tonemap; the spot.omniworld flat floor showed ours as giant dark rings.
  let dith = fract(sin(dot(in.position.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453);
  outRgb = outRgb + vec3<f32>((dith - 0.5) / 255.0);
  return vec4<f32>(outRgb, u.baseColor.a);
}
)WGSL";

  // R4 lighting convergence — kSolidLitTexturedShadow × CSM: the full material path (albedo/roughness/
  // metalness/normal + GGX + sRGB + hemisphere-IBL ambient) with MULTI-CASCADE shadows. Identical to
  // kSolidLitTexturedShadow except the shadow term selects one of N cascades by the fragment's linear
  // view depth (the kSolidLitCsm trick) and 3x3-PCFs that layer of a texture_2d_array shadow map. This
  // is the shader the main-view default needs (materials AND tight multi-cascade shadows). @6 is now a
  // texture_2d_array; LightU carries an array<mat4,4> of light VPs + a cascadeSplits vec4; shadowParams.w
  // = cascade count. strength 0 → byte-identical material render (no shadow term).
  const char *kSolidLitTexturedCsm = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
};
struct LightU {
  lightViewProj : array<mat4x4<f32>, 4>,  // per-cascade light view-projections
  shadowParams  : vec4<f32>,   // x = strength, y = bias, z = hemisphere-IBL enabled, w = cascade count
  cascadeSplits : vec4<f32>,   // far view-depth of cascades 0,1,2,3
  skyColor      : vec4<f32>,   // rgb = sky ambient colour, w = ambient intensity scale
  groundColor   : vec4<f32>,   // rgb = ground ambient colour
  upDir         : vec4<f32>,   // xyz = world up direction
};

@group(0) @binding(0) var<uniform> u : Scene;
@group(0) @binding(1) var albedoTex : texture_2d<f32>;
@group(0) @binding(2) var roughTex : texture_2d<f32>;
@group(0) @binding(3) var metalTex : texture_2d<f32>;
@group(0) @binding(4) var normalTex : texture_2d<f32>;
@group(0) @binding(5) var albedoSamp : sampler;
@group(0) @binding(6) var shadowTexArray : texture_2d_array<f32>;
@group(0) @binding(7) var shadowSamp : sampler;
@group(0) @binding(8) var<uniform> lu : LightU;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};
struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) worldNormal : vec3<f32>,
  @location(1) uv : vec2<f32>,
  @location(2) worldPos : vec3<f32>,
  @location(3) viewDepth : f32,        // camera clip.w = linear view distance (cascade select)
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  let n3 = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.worldNormal = normalize(n3);
  out.uv = in.uv;
  out.worldPos = worldPos.xyz;
  out.viewDepth = out.position.w;
  return out;
}

fn perturbNormal(Ngeo : vec3<f32>, worldPos : vec3<f32>, uv : vec2<f32>, mapN : vec3<f32>) -> vec3<f32> {
  let dp1 = dpdx(worldPos);
  let dp2 = dpdy(worldPos);
  let duv1 = dpdx(uv);
  let duv2 = dpdy(uv);
  let dp2perp = cross(dp2, Ngeo);
  let dp1perp = cross(Ngeo, dp1);
  let T = dp2perp * duv1.x + dp1perp * duv2.x;
  let B = dp2perp * duv1.y + dp1perp * duv2.y;
  // Degenerate/constant UVs (an untextured CAD mesh has no UV gradient) → T,B collapse to ~0,
  // so inverseSqrt(0) = +inf and T*invmax = 0*inf = NaN, poisoning the normal → garbage lighting
  // (the black speckle all over the robot in the shadowed material shader). Fall back to the
  // geometric normal when there's no valid tangent frame; real textured meshes keep normal mapping.
  let tbMax = max(dot(T, T), dot(B, B));
  if (tbMax < 1e-12) { return Ngeo; }
  let invmax = inverseSqrt(tbMax);
  let M = mat3x3<f32>(T * invmax, B * invmax, Ngeo);
  return normalize(M * mapN);
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let L = normalize(u.light.xyz);
  let ambient = u.light.w;
  let Ngeo = normalize(in.worldNormal);
  let mapN = normalize(textureSample(normalTex, albedoSamp, in.uv).xyz * 2.0 - vec3<f32>(1.0));
  let N = perturbNormal(Ngeo, in.worldPos, in.uv, mapN);
  let Ld = -L;
  let NdotL = max(dot(N, Ld), 0.0);
  let albedoTexLin = pow(textureSample(albedoTex, albedoSamp, in.uv).rgb, vec3<f32>(2.2));
  let albedo = albedoTexLin * u.baseColor.rgb;
  let metal = clamp(textureSample(metalTex, albedoSamp, in.uv).r, 0.0, 1.0);

  // Multi-cascade shadow term (PCF 3x3). Select the cascade by linear view depth, then sample its
  // light VP + array layer. strength 0 → fully lit.
  var shadow = 1.0;
  let strength = lu.shadowParams.x;
  if (strength > 0.0) {
    var ci : i32 = 0;
    if (in.viewDepth > lu.cascadeSplits.x) { ci = 1; }
    if (in.viewDepth > lu.cascadeSplits.y) { ci = 2; }
    if (in.viewDepth > lu.cascadeSplits.z) { ci = 3; }
    let nc = i32(lu.shadowParams.w);
    ci = clamp(ci, 0, max(nc - 1, 0));
    let lc = lu.lightViewProj[ci] * vec4<f32>(in.worldPos, 1.0);
    if (lc.w > 0.0) {
      let ndc = lc.xyz / lc.w;
      let suv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
      if (suv.x >= 0.0 && suv.x <= 1.0 && suv.y >= 0.0 && suv.y <= 1.0 && ndc.z <= 1.0) {
        let bias = max(lu.shadowParams.y, 1e-4);
        let texelUV = 1.0 / vec2<f32>(textureDimensions(shadowTexArray));
        var occ = 0.0;
        for (var dy = -1; dy <= 1; dy = dy + 1) {
          for (var dx = -1; dx <= 1; dx = dx + 1) {
            let off = vec2<f32>(f32(dx), f32(dy)) * texelUV;
            let stored = textureSampleLevel(shadowTexArray, shadowSamp, suv + off, ci, 0.0).r;
            if (ndc.z > stored + bias) {
              occ = occ + 1.0;
            }
          }
        }
        shadow = 1.0 - strength * (occ / 9.0);
      }
    }
  }

  // GGX direct specular (Cook-Torrance), identical to kSolidLitTexturedShadow.
  let roughFactor = clamp(1.0 - u.pad1.w, 0.0, 1.0);
  let texRough = textureSample(roughTex, albedoSamp, in.uv).r;
  let effRough = clamp(roughFactor * texRough, 0.045, 1.0);
  let V = normalize(u.pad0.yzw - in.worldPos);
  let Hh = normalize(V + Ld);
  let NdotV = max(dot(N, V), 1e-4);
  let NdotH = max(dot(N, Hh), 0.0);
  let VdotH = max(dot(V, Hh), 0.0);
  let a = effRough * effRough;
  let a2 = a * a;
  let dn = NdotH * NdotH * (a2 - 1.0) + 1.0;
  let D = a2 / (3.14159265 * dn * dn);
  let kg = (effRough + 1.0) * (effRough + 1.0) / 8.0;
  let G = (NdotV / (NdotV * (1.0 - kg) + kg)) * (NdotL / (NdotL * (1.0 - kg) + kg));
  let F0 = mix(vec3<f32>(0.04, 0.04, 0.04), albedo, metal);
  let F = F0 + (vec3<f32>(1.0, 1.0, 1.0) - F0) * pow(1.0 - VdotH, 5.0);
  let specBRDF = (D * G * F) / max(4.0 * NdotV * NdotL, 1e-4);

  let diffuse = albedo * (1.0 - metal);
  var ambTerm = vec3<f32>(ambient, ambient, ambient);
  if (lu.shadowParams.z > 0.5) {
    let upd = normalize(lu.upDir.xyz);
    let t = clamp(dot(N, upd) * 0.5 + 0.5, 0.0, 1.0);
    ambTerm = mix(lu.groundColor.rgb, lu.skyColor.rgb, t) * lu.skyColor.w;
  }
  let ambientPart = diffuse * ambTerm;
  let directPart = (diffuse * NdotL + specBRDF * NdotL) * shadow;
  let col = ambientPart + directPart;
  return vec4<f32>(pow(max(col, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(1.0 / 2.2)), u.baseColor.a);
}
)WGSL";

  // R4 step-3c-A — full-screen overlay shader. A full-screen triangle generated from
  // @builtin(vertex_index) (no vertex buffer) + a flat RGBA colour; alpha-blended by
  // the pipeline over whatever the pane already holds.
  // wgpu counterpart of WREN's Background.atmosphericSky (the Hillaire-2020 pipeline), as a cheap
  // ANALYTIC dome: fullscreen triangle drawn first in the scene pass (depth write off), ray per
  // pixel from the camera basis, sky = day/dusk/night palettes blended purely by SUN ELEVATION +
  // a sun disk/halo tinted by the light colour. Same single input as WREN (the sun_marker
  // supervisor only writes the DirectionalLight.direction), so day-night follows automatically.
  const char *kSkyAtmosphere = R"WGSL(
struct SkyU {
  right    : vec4<f32>,  // camera right * tan(fovX/2); .w = scatter mode (1 = sample the LUT)
  up       : vec4<f32>,  // camera up * tan(fovY/2)
  fwd      : vec4<f32>,  // camera forward; .w = cloud coverage (0 = no cloud layer)
  sunDir   : vec4<f32>,  // TOWARD the sun, normalized
  sunColor : vec4<f32>,  // DirectionalLight colour; .w = HDR linear-radiance arm
  worldUp  : vec4<f32>,  // world up (opposite gravity); .w = sky mode (1 = image cubemap)
  sunIll   : vec4<f32>,  // rgb = linear sun illuminance (scatter mode), .w = sun disc cos radius
};
@group(0) @binding(0) var<uniform> u : SkyU;
@group(0) @binding(1) var lutTex : texture_2d<f32>;    // 128x64 sky-view inscatter (linear HDR)
@group(0) @binding(2) var transTex : texture_2d<f32>;  // 128x1 view transmittance vs elevation
@group(0) @binding(3) var lutSamp : sampler;

// Stable 3D hash + trilinear value noise + 3-octave fBm — WREN sky_apply.frag ports, used by
// the night decorations only (whole-warp skipped in daylight).
fn hash3s(p : vec3<f32>) -> f32 {
  return fract(sin(dot(p, vec3<f32>(127.1, 311.7, 74.7))) * 43758.5453);
}
fn vnoise3s(p : vec3<f32>) -> f32 {
  let i = floor(p);
  var f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  let n000 = hash3s(i + vec3<f32>(0.0, 0.0, 0.0));
  let n100 = hash3s(i + vec3<f32>(1.0, 0.0, 0.0));
  let n010 = hash3s(i + vec3<f32>(0.0, 1.0, 0.0));
  let n110 = hash3s(i + vec3<f32>(1.0, 1.0, 0.0));
  let n001 = hash3s(i + vec3<f32>(0.0, 0.0, 1.0));
  let n101 = hash3s(i + vec3<f32>(1.0, 0.0, 1.0));
  let n011 = hash3s(i + vec3<f32>(0.0, 1.0, 1.0));
  let n111 = hash3s(i + vec3<f32>(1.0, 1.0, 1.0));
  return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
             mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y), f.z);
}
// Sine-less hash (Dave Hoskins class) + quintic value noise for the CLOUD layer: the sin-based
// hash above loses f32 precision at large lattice coordinates (a 30 km cloud ray), which sliced
// into hard parallelogram blocks under the coverage threshold. Kept separate so the night kit's
// star field stays bit-stable.
fn hashNS(pIn : vec3<f32>) -> f32 {
  var p = fract(pIn * 0.1031);
  p = p + dot(p, p.yzx + vec3<f32>(33.33, 33.33, 33.33));
  return fract((p.x + p.y) * p.z);
}
fn vnoiseC(p : vec3<f32>) -> f32 {
  let i = floor(p);
  var f = fract(p);
  f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
  let n000 = hashNS(i + vec3<f32>(0.0, 0.0, 0.0));
  let n100 = hashNS(i + vec3<f32>(1.0, 0.0, 0.0));
  let n010 = hashNS(i + vec3<f32>(0.0, 1.0, 0.0));
  let n110 = hashNS(i + vec3<f32>(1.0, 1.0, 0.0));
  let n001 = hashNS(i + vec3<f32>(0.0, 0.0, 1.0));
  let n101 = hashNS(i + vec3<f32>(1.0, 0.0, 1.0));
  let n011 = hashNS(i + vec3<f32>(0.0, 1.0, 1.0));
  let n111 = hashNS(i + vec3<f32>(1.0, 1.0, 1.0));
  return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
             mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y), f.z);
}
fn fbmC(pIn : vec3<f32>) -> f32 {
  var p = pIn;
  var v = 0.0;
  var amp = 0.5;
  for (var i = 0; i < 5; i = i + 1) {
    v = v + amp * vnoiseC(p);
    p = p * 2.17;
    amp = amp * 0.5;
  }
  return v;
}
fn fbm3s(pIn : vec3<f32>) -> f32 {
  var p = pIn;
  var v = 0.0;
  var a = 0.5;
  for (var i = 0; i < 3; i = i + 1) {
    v = v + a * vnoise3s(p);
    p = p * 2.13;
    a = a * 0.5;
  }
  return v;
}

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) ndc : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  // The dome now draws AFTER the opaque span, depth-tested at the far plane (up.w carries the
  // clear-depth value: 1 standard, 0 reversed-Z), so every sky evaluation — LUT sample, night
  // kit, clouds — runs on ACTUAL sky pixels only instead of the whole frame.
  o.pos = vec4<f32>(p[vi], u.up.w, 1.0);
  o.ndc = p[vi];
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  let ray = normalize(u.fwd.xyz + in.ndc.x * u.right.xyz + in.ndc.y * u.up.xyz);
  let upw = normalize(u.worldUp.xyz);
  let sun = normalize(u.sunDir.xyz);
  let elevR = dot(ray, upw);            // ray elevation in [-1, 1]
  let sunE = dot(sun, upw);             // sun elevation: >0 day, ~0 horizon, <0 night
  // PHYSICALLY-SCATTERED SKY (right.w > 0.5, HDR path only): sample the per-sun-position
  // sky-view LUT (kSkyScatterLut marches Rayleigh+Mie+ozone into it only when the sun moves)
  // instead of the hand-tuned palette. The dome pays one texture sample per pixel — cheaper
  // than the palette math it replaces; the march cost lives in an 8k-pixel offline pass.
  if (u.right.w > 0.5) {
    // LUT parameterization: u = azimuth delta from the sun over [0, pi]; v = elevation with a
    // squared horizon-focused mapping, eA = sign(2v-1) * (2v-1)^2 * pi/2.
    let eA = asin(clamp(elevR, -1.0, 1.0));
    let vv = 0.5 + 0.5 * sign(eA) * sqrt(abs(eA) / 1.5707963);
    let rayH = ray - upw * elevR;
    let sunH = sun - upw * sunE;
    let hl = length(rayH) * length(sunH);
    var azCos = 1.0;
    if (hl > 1e-6) { azCos = clamp(dot(rayH, sunH) / hl, -1.0, 1.0); }
    let uu = acos(azCos) / 3.14159265;
    var skyLin = textureSampleLevel(lutTex, lutSamp, vec2<f32>(uu, vv), 0.0).rgb;
    // Sun disc: transmitted (naturally reddened at the horizon) direct illuminance.
    let cosSV = dot(ray, sun);
    if (cosSV > u.sunIll.w) {
      let tr = textureSampleLevel(transTex, lutSamp, vec2<f32>(vv, 0.5), 0.0).rgb;
      skyLin = skyLin + tr * u.sunIll.xyz;
    }
    // Night decorations (WREN sky_apply.frag port, world-frame): stars + Milky Way + moon,
    // added in linear HDR so AgX rolls them off. The branch is sun-elevation-uniform, so the
    // whole-warp skip keeps the day sky at one texture sample.
    let nightStrength = smoothstep(0.08, -0.05, sunE);
    if (nightStrength > 0.001) {
      let galacticPole = normalize(vec3<f32>(0.92, 0.15, 0.36));
      let bandDist = abs(dot(ray, galacticPole));
      let bandCore = smoothstep(0.10, 0.0, bandDist);
      let bandWide = smoothstep(0.30, 0.05, bandDist);
      let bandAbove = smoothstep(-0.10, 0.15, elevR);
      let dust = fbmC(ray * 8.0);
      let lanes = smoothstep(0.35, 0.65, fbmC(ray * 18.0));
      let mwTint = mix(vec3<f32>(0.65, 0.75, 1.00), vec3<f32>(0.92, 0.78, 0.90),
                       smoothstep(0.4, 0.7, dust));
      skyLin = skyLin + mwTint * (bandCore * 0.22 + bandWide * 0.06) * (0.30 + 0.70 * dust) *
                          (1.0 - 0.80 * lanes) * bandAbove * nightStrength;
      let starHorizon = smoothstep(-0.02, 0.12, elevR);
      let cell = floor(ray * 600.0);
      let h = hashNS(cell);
      let starOn = step(0.9970, h);
      let starMag = pow(fract(h * 31.13), 5.0);
      let hue = fract(h * 7.731);
      var starColor = vec3<f32>(0.95, 0.97, 1.05);
      if (hue >= 0.60) { starColor = vec3<f32>(1.00, 0.92, 0.78); }
      if (hue >= 0.88) { starColor = vec3<f32>(1.05, 0.78, 0.60); }
      skyLin = skyLin + starColor * 16.0 * starOn * (0.20 + 1.10 * starMag) * starHorizon * nightStrength;
      let cellB = floor(ray * 220.0);
      let hb = hashNS(cellB + vec3<f32>(13.7, 91.3, 47.1));
      skyLin = skyLin + vec3<f32>(1.0, 0.95, 0.85) * 70.0 * step(0.99955, hb) * starHorizon * nightStrength;
      let moonDir = -sun;
      let moonElev = dot(moonDir, upw);
      if (moonElev > 0.0) {
        let cosMoon = dot(ray, moonDir);
        let moonDisc = smoothstep(0.99910, 0.99960, cosMoon);
        let moonHalo = pow(smoothstep(0.9955, 0.99910, cosMoon), 1.6);
        let mUp = normalize(cross(moonDir, upw));
        let mRight = normalize(cross(moonDir, mUp));
        let moonUv = vec2<f32>(dot(ray, mRight), dot(ray, mUp));
        let maria = fbmC(vec3<f32>(moonUv * 160.0, 1.7));
        let mariaMask = smoothstep(0.48, 0.62, maria);
        let craters = smoothstep(0.55, 0.72, vnoiseC(vec3<f32>(moonUv * 900.0, 7.3)));
        let r = sqrt(max(0.0, 1.0 - cosMoon) * 2000.0);
        let limb = clamp(1.0 - r * 0.35, 0.6, 1.0);
        var moonColor = mix(vec3<f32>(0.97, 0.94, 0.88), vec3<f32>(0.45, 0.42, 0.40), mariaMask);
        moonColor = moonColor * (1.0 - 0.30 * craters) * limb;
        skyLin = skyLin + moonColor * 18.0 * moonDisc * nightStrength;
        skyLin = skyLin + vec3<f32>(0.78, 0.82, 0.92) * 0.14 * moonHalo * nightStrength;
      }
    }
    // PROCEDURAL CLOUD LAYER (fwd.w = coverage, 0 = off): a 2-scale fBm sheet at ~1.5 km,
    // ray-intersected per sky pixel. Lit by the TRANSMITTED sun sampled from the transmittance
    // strip at the SUN's own elevation — clouds go amber at sunset and dark at night by physics,
    // no palette. Ambient from the LUT's own upper sky. Composited OVER the disc/stars/moon so
    // weather occludes them. Static by design (the bench pins its light); wind is a follow-up.
    let cover = u.fwd.w;
    if (cover > 0.001 && elevR > 0.005) {
      let tC = 1500.0 / elevR;
      let cpos = ray * tC;
      let e1 = normalize(cross(upw, vec3<f32>(0.31, 0.75, 0.58)));
      let e2 = cross(upw, e1);
      let cuv = vec2<f32>(dot(cpos, e1), dot(cpos, e2)) * (1.0 / 9000.0);
      let dBase = fbmC(vec3<f32>(cuv.x, cuv.y, 3.1));
      let dDetail = fbmC(vec3<f32>(cuv.x * 3.7, cuv.y * 3.7, 7.9));
      let d = dBase * 0.72 + dDetail * 0.28;
      // fbmC's mean sits near 0.48 — the threshold must sit ABOVE it or the layer reads as
      // full overcast. cover 0.4 -> th ~0.68: scattered cumulus with open blue between.
      let th = 0.45 + clamp(1.0 - cover, 0.0, 1.0) * 0.38;
      let dens = smoothstep(th, th + 0.16, d);
      if (dens > 0.002) {
        let eS = asin(clamp(sunE, -1.0, 1.0));
        let sunVv = 0.5 + 0.5 * sign(eS) * sqrt(abs(eS) / 1.5707963);
        let sunTrans = textureSampleLevel(transTex, lutSamp, vec2<f32>(sunVv, 0.5), 0.0).rgb;
        let dayC = clamp(sunE * 4.0 + 0.1, 0.0, 1.0);
        let lit = u.sunIll.xyz * sunTrans * 0.085 * dayC;
        let ambC = textureSampleLevel(lutTex, lutSamp, vec2<f32>(0.5, 0.85), 0.0).rgb * 0.7;
        let shade = mix(1.0, 0.32, smoothstep(0.15, 0.95, dens));
        var cCol = lit * shade + ambC;
        cCol = cCol + u.sunIll.xyz * sunTrans * 0.05 * dayC * pow(max(dot(ray, sun), 0.0), 6.0) * (1.0 - dens);
        let aC = dens * exp(-tC / 60000.0);
        skyLin = mix(skyLin, cCol, aC);
      }
    }
    return vec4<f32>(max(skyLin, vec3<f32>(0.0, 0.0, 0.0)), 1.0);
  }
  let day = clamp(sunE * 5.0 + 0.5, 0.0, 1.0);
  let dusk = clamp(1.0 - abs(sunE) * 3.0, 0.0, 1.0);  // peaks when the sun sits on the horizon
  // Earth-preset palette: pale desaturated horizon, deeper blue zenith (matched to WREN's bake).
  let zenithDay = vec3<f32>(0.25, 0.43, 0.74);
  let horizonDay = vec3<f32>(0.62, 0.70, 0.82);
  let night = vec3<f32>(0.015, 0.025, 0.05);
  let t = clamp(elevR, 0.0, 1.0);
  var sky = mix(horizonDay, zenithDay, pow(t, 0.55));
  // Sunset/sunrise: warm wash hugging the horizon, focused toward the sun's azimuth.
  let toSun = max(dot(ray, sun), 0.0);
  let warm = vec3<f32>(1.0, 0.45, 0.20);
  sky = mix(sky, warm, dusk * pow(1.0 - t, 3.0) * (0.25 + 0.75 * pow(toSun, 4.0)));
  sky = mix(night, sky, day);
  // Below the horizon: BRIGHT aerial haze, like WREN's Hillaire output — from a high camera the
  // empty region under the horizon line reads as pale atmosphere, not dark ground. Slightly
  // dimmer + desaturated relative to the horizon band, day-scaled.
  let below = clamp(-elevR * 4.0, 0.0, 1.0);
  let haze = mix(horizonDay * 0.97, vec3<f32>(0.62, 0.66, 0.70), clamp(-elevR * 1.5, 0.0, 1.0));
  sky = mix(sky, haze * (0.08 + 0.92 * day), below);
  // Sun disk + halo, tinted by the light colour, hidden at night.
  let disk = smoothstep(0.9995, 0.99985, toSun);
  let halo = pow(toSun, 350.0) * 0.5 + pow(toSun, 32.0) * 0.08;
  // sunColor.w > 0.5 = HDR linear-radiance arm: decode the display-tuned palette, then add the
  // sun disk/halo with real energy — disk 24 clears the AgX ~16.3 white point, so the sun core
  // tonemaps to true white and becomes a genuine bloom source. No dither (the tonemap pass owns it).
  if (u.sunColor.w > 0.5) {
    var skyLin = pow(max(sky, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(2.2, 2.2, 2.2));
    let sunLin = pow(max(u.sunColor.xyz, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(2.2, 2.2, 2.2));
    skyLin = skyLin + (disk * 24.0 + halo * 4.0) * sunLin * day;
    return vec4<f32>(skyLin, 1.0);
  }
  sky = sky + (disk * 2.0 + halo) * u.sunColor.xyz * day;
  let dith = fract(sin(dot(in.pos.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453);
  return vec4<f32>(sky + vec3<f32>((dith - 0.5) / 255.0), 1.0);
}
)WGSL";

  // Sky-view LUT bake (Hillaire-class single scatter, WREN sky_apply.frag math): marches
  // Rayleigh + Mie + ozone into a 128x64 inscatter LUT (u = azimuth delta from sun over [0, pi],
  // v = squared horizon-focused elevation) and a 128x1 view-transmittance strip. Runs ONLY when
  // the sun/preset changes (~8k pixels x 32 view steps x 8 sun steps — sub-0.1 ms), so the
  // steady-state per-frame cost of the physical sky is ONE texture sample in the dome shader.
  // Coefficients are PER-METRE (the march multiplies by dt_km * 1000, WREN convention).
  const char *kSkyScatterLut = R"WGSL(
struct ScatU {
  rayleigh : vec4<f32>,  // rgb scattering /m, w = density exp scale (/km)
  mie      : vec4<f32>,  // rgb scattering /m, w = density exp scale (/km)
  mieAbs   : vec4<f32>,  // rgb absorption /m, w = Mie phase g
  ozone    : vec4<f32>,  // rgb extinction /m, w = camera height (km)
  radii    : vec4<f32>,  // x = bottom radius km, y = top radius km, z = sin(sun elevation), w = ground albedo
  sunIll   : vec4<f32>,  // rgb = sun illuminance (linear)
};
@group(0) @binding(0) var<uniform> u : ScatU;

struct VOut { @builtin(position) pos : vec4<f32> };

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  return o;
}

fn raySphereExit(o : vec3<f32>, d : vec3<f32>, r : f32) -> f32 {
  let b = dot(o, d);
  let c = dot(o, o) - r * r;
  let disc = b * b - c;
  if (disc < 0.0) { return -1.0; }
  return -b + sqrt(disc);
}
fn raySphereEnter(o : vec3<f32>, d : vec3<f32>, r : f32) -> f32 {
  let b = dot(o, d);
  let c = dot(o, o) - r * r;
  let disc = b * b - c;
  if (disc < 0.0) { return -1.0; }
  let t = -b - sqrt(disc);
  if (t < 0.0) { return -1.0; }
  return t;
}

fn odepth(origin : vec3<f32>, dir : vec3<f32>) -> vec3<f32> {
  let tExit = raySphereExit(origin, dir, u.radii.y);
  if (tExit < 0.0) { return vec3<f32>(0.0, 0.0, 0.0); }
  if (raySphereEnter(origin, dir, u.radii.x) > 0.0) { return vec3<f32>(1e10, 1e10, 1e10); }
  let dt = tExit / 8.0;
  var sum = vec3<f32>(0.0, 0.0, 0.0);
  for (var i = 0; i < 8; i = i + 1) {
    let p = origin + dir * (dt * (f32(i) + 0.5));
    let h = max(0.0, length(p) - u.radii.x);
    let rd = exp(h * u.rayleigh.w);
    let md = exp(h * u.mie.w);
    let ext = u.rayleigh.xyz * rd + (u.mie.xyz + u.mieAbs.xyz) * md + u.ozone.xyz * rd;
    sum = sum + ext * dt * 1000.0;
  }
  return sum;
}

fn viewFromElev(eA : f32, azD : f32) -> vec3<f32> {
  let cE = cos(eA);
  return vec3<f32>(cos(azD) * cE, sin(eA), sin(azD) * cE);
}
fn elevFromTexel(t : f32) -> f32 {
  let se = t * 2.0 - 1.0;
  return sign(se) * se * se * 1.5707963;
}

@fragment
fn fs_lut(in : VOut) -> @location(0) vec4<f32> {
  let azD = (in.pos.x / 128.0) * 3.14159265;
  let eA = elevFromTexel(in.pos.y / 64.0);
  let view = viewFromElev(eA, azD);
  let sE = clamp(u.radii.z, -1.0, 1.0);
  let sun = vec3<f32>(sqrt(max(0.0, 1.0 - sE * sE)), sE, 0.0);
  let origin = vec3<f32>(0.0, u.radii.x + u.ozone.w, 0.0);

  let tExit = raySphereExit(origin, view, u.radii.y);
  if (tExit <= 0.0) { return vec4<f32>(0.0, 0.0, 0.0, 1.0); }
  let tGround = raySphereEnter(origin, view, u.radii.x);
  var tEnd = tExit;
  if (tGround > 0.0) { tEnd = min(tExit, tGround); }

  let dt = tEnd / 32.0;
  var sum = vec3<f32>(0.0, 0.0, 0.0);
  var transmittance = vec3<f32>(1.0, 1.0, 1.0);
  let cosSV = dot(view, sun);
  let pR = (3.0 / (16.0 * 3.14159265)) * (1.0 + cosSV * cosSV);
  let g = u.mieAbs.w;
  let g2 = g * g;
  let pM = (3.0 / (8.0 * 3.14159265)) * ((1.0 - g2) * (1.0 + cosSV * cosSV)) /
           ((2.0 + g2) * pow(max(1e-6, 1.0 + g2 - 2.0 * g * cosSV), 1.5));
  for (var i = 0; i < 32; i = i + 1) {
    let p = origin + view * (dt * (f32(i) + 0.5));
    let h = max(0.0, length(p) - u.radii.x);
    let rd = exp(h * u.rayleigh.w);
    let md = exp(h * u.mie.w);
    let ext = u.rayleigh.xyz * rd + (u.mie.xyz + u.mieAbs.xyz) * md + u.ozone.xyz * rd;
    let segT = exp(-ext * dt * 1000.0);
    let sunT = exp(-odepth(p, sun));
    let inscatter = (u.rayleigh.xyz * rd * pR + u.mie.xyz * md * pM) * sunT * dt * 1000.0;
    sum = sum + transmittance * inscatter;
    transmittance = transmittance * segT;
  }
  // Ground-hitting rays (Hillaire's ground-albedo term): the planet surface reflects transmitted
  // sunlight Lambertian-ly. Without this, below-horizon sky reads as dark void on scenes whose
  // authored ground is smaller than the view (the bench's small lot).
  if (tGround > 0.0) {
    let pG = origin + view * tGround;
    let sunTG = exp(-odepth(pG, sun));
    let nDotL = max(u.radii.z, 0.0);
    sum = sum + transmittance * (u.radii.w / 3.14159265) * sunTG * nDotL;
  }
  return vec4<f32>(sum * u.sunIll.xyz, 1.0);
}

@fragment
fn fs_trans(in : VOut) -> @location(0) vec4<f32> {
  let eA = elevFromTexel(in.pos.x / 128.0);
  let view = viewFromElev(eA, 0.0);
  let origin = vec3<f32>(0.0, u.radii.x + u.ozone.w, 0.0);
  let od = odepth(origin, view);
  return vec4<f32>(exp(-od), 1.0);
}
)WGSL";

  // Screen-space reflections, single combine pass on the LINEAR HDR scene (pre-tonemap, WREN-side
  // ordering): reconstructs position + normal from the scene's own MSAA depth (sample 0 — the
  // GTAO trick, no G-buffer), marches the reflection ray with jitter + 4-step binary refinement,
  // and adds the Fresnel-weighted hit colour where the scene's alpha carries reflectivity
  // (written by the lit shader on opaque pixels). Pixels with refl < 0.03 or sky depth early-out,
  // so grass/brick/plaster pay one textureLoad. A miss keeps the analytic-IBL result — SSR only
  // ADDS what the screen can prove.
  const char *kSsrCombine = R"WGSL(
struct SsrU {
  invVP  : mat4x4<f32>,
  vp     : mat4x4<f32>,
  camPos : vec4<f32>,   // xyz = camera world pos, w = max march distance (m)
  params : vec4<f32>,   // x = reversed-Z flag, y = strength, z = march steps, w = unused
};
@group(0) @binding(0) var<uniform> u : SsrU;
@group(0) @binding(1) var sceneTex : texture_2d<f32>;
@group(0) @binding(2) var depthTex : texture_depth_multisampled_2d;
@group(0) @binding(3) var samp : sampler;

struct VOut { @builtin(position) pos : vec4<f32> };

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  return o;
}

fn worldAt(uv : vec2<f32>, z : f32) -> vec3<f32> {
  let ndc = vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, z, 1.0);
  let w = u.invVP * ndc;
  return w.xyz / w.w;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  let dimsI = vec2<i32>(textureDimensions(depthTex));
  let dims = vec2<f32>(dimsI);
  let pix = vec2<i32>(in.pos.xy);
  let c = textureLoad(sceneTex, pix, 0);
  let refl = c.a;
  let rev = u.params.x > 0.5;
  let z0 = textureLoad(depthTex, pix, 0);
  let skyZ = select(1.0, 0.0, rev);
  if (refl < 0.05 || abs(z0 - skyZ) < 1e-6) {
    return vec4<f32>(c.rgb, 1.0);
  }
  let uv0 = in.pos.xy / dims;
  let P = worldAt(uv0, z0);
  // Depth-derived normal, closer-side differencing (kSsaoGtao's edge-safe scheme).
  let pxr = min(pix + vec2<i32>(1, 0), dimsI - vec2<i32>(1, 1));
  let pxl = max(pix - vec2<i32>(1, 0), vec2<i32>(0, 0));
  let pyd = min(pix + vec2<i32>(0, 1), dimsI - vec2<i32>(1, 1));
  let pyu = max(pix - vec2<i32>(0, 1), vec2<i32>(0, 0));
  let zr = textureLoad(depthTex, pxr, 0);
  let zl = textureLoad(depthTex, pxl, 0);
  let zd = textureLoad(depthTex, pyd, 0);
  let zu = textureLoad(depthTex, pyu, 0);
  let px = vec2<f32>(1.0, 0.0) / dims;
  let py = vec2<f32>(0.0, 1.0) / dims;
  var ddxv = worldAt(uv0 + px, zr) - P;
  if (abs(zl - z0) < abs(zr - z0)) { ddxv = P - worldAt(uv0 - px, zl); }
  var ddyv = worldAt(uv0 + py, zd) - P;
  if (abs(zu - z0) < abs(zd - z0)) { ddyv = P - worldAt(uv0 - py, zu); }
  var N = normalize(cross(ddyv, ddxv));
  let Vv = normalize(u.camPos.xyz - P);
  if (dot(N, Vv) < 0.0) { N = -N; }
  let R = reflect(-Vv, N);

  let steps = i32(u.params.z + 0.5);
  let dt = u.camPos.w / f32(steps);
  let jitter = fract(sin(dot(in.pos.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453);
  var t = dt * (0.5 + 0.5 * jitter);
  var tPrev = 0.0;
  var hit = false;
  var hitUv = vec2<f32>(0.0, 0.0);
  for (var i = 0; i < 64; i = i + 1) {
    if (i >= steps) { break; }
    let Q = P + R * t;
    let q4 = u.vp * vec4<f32>(Q, 1.0);
    if (q4.w < 1e-4) { break; }
    let qn = q4.xyz / q4.w;
    let quv = vec2<f32>(qn.x * 0.5 + 0.5, 0.5 - qn.y * 0.5);
    if (quv.x < 0.0 || quv.x > 1.0 || quv.y < 0.0 || quv.y > 1.0) { break; }
    let sz = textureLoad(depthTex, vec2<i32>(quv * dims), 0);
    var behind = qn.z > sz;
    if (rev) { behind = qn.z < sz; }
    if (behind) {
      // Binary refinement between the last-clear and first-behind march points.
      var lo = tPrev;
      var hi = t;
      for (var r = 0; r < 4; r = r + 1) {
        let mid = 0.5 * (lo + hi);
        let Qm = P + R * mid;
        let m4 = u.vp * vec4<f32>(Qm, 1.0);
        let mn = m4.xyz / max(m4.w, 1e-4);
        let muv = vec2<f32>(mn.x * 0.5 + 0.5, 0.5 - mn.y * 0.5);
        let msz = textureLoad(depthTex, vec2<i32>(clamp(muv, vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 1.0)) * dims), 0);
        var mBehind = mn.z > msz;
        if (rev) { mBehind = mn.z < msz; }
        if (mBehind) { hi = mid; } else { lo = mid; }
      }
      let Qh = P + R * hi;
      let h4 = u.vp * vec4<f32>(Qh, 1.0);
      let hn = h4.xyz / max(h4.w, 1e-4);
      let huv = vec2<f32>(hn.x * 0.5 + 0.5, 0.5 - hn.y * 0.5);
      let hsz = textureLoad(depthTex, vec2<i32>(clamp(huv, vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 1.0)) * dims), 0);
      // Thickness: accept only when the marched point is NEAR the surface it crossed — a ray
      // passing far behind a foreground object keeps marching context, not a false hit.
      let S = worldAt(huv, hsz);
      // Refinement converges a GENUINE hit to within centimetres of the crossed surface; a ray
      // that slid BEHIND geometry (the classic SSR leak) refines to a point far from it.
      if (length(Qh - S) < max(dt * 0.5, 0.2)) {
        hit = true;
        hitUv = huv;
      }
      break;
    }
    tPrev = t;
    t = t + dt;
  }
  var outRgb = c.rgb;
  if (hit) {
    let hcol = textureSampleLevel(sceneTex, samp, hitUv, 0.0).rgb;
    let NdotV = max(dot(N, Vv), 0.0);
    let fres = 0.25 + 0.75 * pow(1.0 - NdotV, 2.0);
    let edge = min(min(hitUv.x, 1.0 - hitUv.x), min(hitUv.y, 1.0 - hitUv.y));
    let fade = clamp(edge * 8.0, 0.0, 1.0);
    outRgb = outRgb + hcol * refl * fres * fade * u.params.y;
  }
  return vec4<f32>(outRgb, 1.0);
}
)WGSL";

  // VOLUMETRIC LIGHT SHAFTS (the OmniLight way): sun visibility was BAKED per probe cell
  // (slab 1 alpha of the probe volume); this pass marches 16 steps from the camera to the
  // depth hit, accumulating height-falloff fog density x baked sun visibility x a
  // Henyey-Greenstein phase toward the sun — additive (One/One) onto the linear HDR scene.
  // God rays through doorways and trees at the cost of 16 3D-texture taps on lit pixels.
  const char *kVolScatter = R"WGSL(
struct VolU {
  invVP     : mat4x4<f32>,
  camPos    : vec4<f32>,   // xyz camera, w = max march distance
  sunDir    : vec4<f32>,   // xyz TOWARD sun, w = HG g
  sunCol    : vec4<f32>,   // rgb sun energy, w = density (per metre)
  volOrigin : vec4<f32>,   // xyz probe-volume origin, w = height falloff H
  volParams : vec4<f32>,   // xyz inv extent, w = z dim
};
@group(0) @binding(0) var<uniform> u : VolU;
@group(0) @binding(1) var depthTex : texture_depth_multisampled_2d;
@group(0) @binding(2) var volTex : texture_3d<f32>;
@group(0) @binding(3) var volSamp : sampler;

struct VOut { @builtin(position) pos : vec4<f32> };

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  let dims = vec2<f32>(textureDimensions(depthTex));
  let pix = vec2<i32>(in.pos.xy);
  let z = textureLoad(depthTex, pix, 0);
  let uv = in.pos.xy / dims;
  let ndc = vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, max(z, 1e-6), 1.0);
  let w4 = u.invVP * ndc;
  let world = w4.xyz / w4.w;
  let toP = world - u.camPos.xyz;
  let distP = length(toP);
  let dir = toP / max(distP, 1e-4);
  let dist = min(distP, u.camPos.w);
  let steps = 16.0;
  let dt = dist / steps;
  let cosSV = dot(dir, normalize(u.sunDir.xyz));
  let g = u.sunDir.w;
  let g2 = g * g;
  let phase = (1.0 - g2) / (12.566371 * pow(max(1.0 + g2 - 2.0 * g * cosSV, 1e-4), 1.5));
  // per-pixel jitter breaks step banding; TAA settles the noise
  let jit = fract(sin(dot(in.pos.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453);
  var accum = 0.0;
  for (var i = 0.0; i < steps; i = i + 1.0) {
    let t = (i + jit) * dt;
    let p = u.camPos.xyz + dir * t;
    let dens = u.sunCol.w * exp(-max(p.z, 0.0) / u.volOrigin.w);
    var vis = 1.0;
    let uvw = (p - u.volOrigin.xyz) * u.volParams.xyz;
    if (all(uvw >= vec3<f32>(0.0, 0.0, 0.0)) && all(uvw <= vec3<f32>(1.0, 1.0, 1.0))) {
      let dz = u.volParams.w;
      let zSub = clamp(uvw.z, 0.5 / dz, 1.0 - 0.5 / dz) * 0.25;
      vis = textureSampleLevel(volTex, volSamp, vec3<f32>(uvw.x, uvw.y, 0.25 + zSub), 0.0).a;
    }
    accum = accum + dens * vis * dt;
  }
  let scatter = u.sunCol.rgb * (accum * phase);  // HG phase is sphere-normalised already
  return vec4<f32>(scatter, 0.0);
}
)WGSL";

  // OmniLight loading overlay: a minimal sun-dot + progress bar, alpha-blended over the
  // tonemapped frame while the bake cooks. params = {progress 0..1, aspect W/H, pulse t, alpha}.
  const char *kOmniProgress = R"WGSL(
struct BarU { params : vec4<f32> };
@group(0) @binding(0) var<uniform> u : BarU;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  o.uv = vec2<f32>(p[vi].x * 0.5 + 0.5, 0.5 - p[vi].y * 0.5);
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  let aspect = u.params.y;
  let cy = 0.925;
  let halfW = 0.13;
  let halfH = 0.0045;
  let amber = vec3<f32>(1.0, 0.66, 0.28);
  var col = vec3<f32>(0.0, 0.0, 0.0);
  var a = 0.0;
  // track (rounded by a soft vertical falloff)
  let dx = abs(in.uv.x - 0.5);
  let dy = abs(in.uv.y - cy);
  if (dx < halfW + 0.004 && dy < halfH + 0.003) {
    let edge = smoothstep(halfH + 0.003, halfH, dy) * smoothstep(halfW + 0.004, halfW, dx);
    col = vec3<f32>(0.06, 0.06, 0.07);
    a = 0.55 * edge;
    // fill
    let fillX = 0.5 - halfW + 2.0 * halfW * clamp(u.params.x, 0.0, 1.0);
    if (in.uv.x < fillX && dy < halfH && dx < halfW) {
      col = amber;
      a = 0.92;
    }
  }
  // pulsing sun dot left of the bar (aspect-corrected circle)
  let cdx = (in.uv.x - (0.5 - halfW - 0.028)) * aspect;
  let cdy = in.uv.y - cy;
  let r = sqrt(cdx * cdx * (1.0 / (aspect * aspect)) * aspect * aspect + cdy * cdy);
  let rr = sqrt((in.uv.x - (0.5 - halfW - 0.028)) * (in.uv.x - (0.5 - halfW - 0.028)) * aspect * aspect + cdy * cdy) / 1.0;
  let pulse = 0.75 + 0.25 * sin(u.params.z * 4.0);
  let dot = smoothstep(0.0085, 0.006, rr);
  if (dot > 0.0) {
    col = mix(col, amber, dot);
    a = max(a, dot * 0.95 * pulse);
  }
  if (a <= 0.001) {
    discard;
  }
  return vec4<f32>(col * a, a);  // premultiplied src-over
}
)WGSL";


  // Image-cubemap skybox: the wgpu counterpart of WREN's cubemap path (skybox.vert/.frag). Same
  // SkyU uniform + fullscreen triangle + per-pixel camera-basis ray as kSkyAtmosphere; the ray
  // samples the cube with WREN's (x, y, -z) convention (skybox.frag samples vec3(uv.xy, -uv.z)).
  // Faces are display-encoded images sampled and written straight through — the LDR identity —
  // matching how the world author sees them in WREN.
  const char *kSkyCubemap = R"WGSL(
struct SkyU {
  right    : vec4<f32>,
  up       : vec4<f32>,
  fwd      : vec4<f32>,
  sunDir   : vec4<f32>,
  sunColor : vec4<f32>,
  worldUp  : vec4<f32>,  // .w = sky mode (CPU-side branch selector; unread here)
};
@group(0) @binding(0) var<uniform> u : SkyU;
@group(0) @binding(1) var skyCube : texture_cube<f32>;
@group(0) @binding(2) var skySamp : sampler;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) ndc : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], u.up.w, 1.0);  // far-plane z from up.w (see kSkyAtmosphere)
  o.ndc = p[vi];
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  let ray = normalize(u.fwd.xyz + in.ndc.x * u.right.xyz + in.ndc.y * u.up.xyz);
  let c = textureSample(skyCube, skySamp, vec3<f32>(ray.x, ray.y, -ray.z));
  // sunColor.w > 0.5 = HDR arm: decode the display-encoded face so the AgX round-trip is
  // near-identity for authored LDR cubemaps (instead of double-compressing them).
  if (u.sunColor.w > 0.5) {
    return vec4<f32>(pow(max(c.rgb, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(2.2, 2.2, 2.2)), 1.0);
  }
  return vec4<f32>(c.rgb, 1.0);
}
)WGSL";

  // SSAO estimate (wgpu counterpart of WREN's GTAO / Viewpoint.ambientOcclusionRadius): depth-only
  // screen-space AO. Samples the camera clip-depth prepass (R32Float, ndc z), linearizes to meters
  // (near 0.05 / far 1000 — buildViewProj's constants), and accumulates occlusion over a 12-tap
  // spiral: a tap occludes when it is CLOSER by [bias .. range] meters (the range check rejects
  // unrelated foreground silhouettes). Output is an AO factor (1 = open, <1 = occluded), blurred by
  // the shared bloom blur and MULTIPLIED onto the scene. params: x = radius in UV, y = intensity.
  const char *kSsaoEstimate = R"WGSL(
struct U { params : vec4<f32> };
@group(0) @binding(0) var<uniform> u : U;
@group(0) @binding(1) var depthTex : texture_2d<f32>;
@group(0) @binding(2) var samp : sampler;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  o.uv = vec2<f32>(p[vi].x * 0.5 + 0.5, 0.5 - p[vi].y * 0.5);
  return o;
}

fn linDepth(z : f32) -> f32 {
  // ndc z (0..1) → view-space meters for near=0.05, far=1000 (buildViewProj).
  let nearZ = 0.05;
  let farZ = 1000.0;
  return nearZ * farZ / (farZ - z * (farZ - nearZ));
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  var z0 = textureSampleLevel(depthTex, samp, in.uv, 0.0).r;
  if (u.params.z > 0.5) { z0 = 1.0 - z0; }  // reversed-Z prepass → undo before linearizing
  if (z0 >= 0.9999) {
    return vec4<f32>(1.0, 1.0, 1.0, 1.0);  // sky/background: fully open
  }
  let d0 = linDepth(z0);
  // 6 opposite-pair directions (12 taps): occlusion is the CURVATURE d0 - (ds+ + ds-)/2, not the
  // first difference — on any oblique flat surface the two opposite gradients cancel exactly, so
  // planes produce zero AO (the first-difference kernel painted false dark rings on clean floors,
  // e.g. spot.omniworld top-down). Radius shrinks with distance for a constant world-size kernel.
  let r = u.params.x * clamp(10.0 / d0, 0.25, 2.5);
  var occ = 0.0;
  for (var i = 0; i < 6; i = i + 1) {
    let a = (f32(i) + 0.5) * 1.0472;           // 6 directions over 2π
    let rr = r * (0.4 + 0.6 * f32(i) / 5.0);
    let duv = vec2<f32>(cos(a), sin(a)) * rr;
    var zp = textureSampleLevel(depthTex, samp, in.uv + duv, 0.0).r;
    var zm = textureSampleLevel(depthTex, samp, in.uv - duv, 0.0).r;
    if (u.params.z > 0.5) { zp = 1.0 - zp; zm = 1.0 - zm; }
    let dp = linDepth(zp);
    let dm = linDepth(zm);
    // Curvature: positive when BOTH sides are closer on average (a crease/contact), zero on planes.
    let curv = d0 - 0.5 * (dp + dm);
    // Silhouette reject: ignore pairs where either side jumps by more than the AO range.
    let valid = step(abs(d0 - dp), 6.0) * step(abs(d0 - dm), 6.0);
    occ = occ + clamp(curv / 0.5, 0.0, 1.0) * step(0.05, curv) * valid;
  }
  let ao = clamp(1.0 - u.params.y * (occ / 6.0), 0.0, 1.0);
  return vec4<f32>(ao, ao, ao, 1.0);
}
)WGSL";

  // AgX filmic tonemap post-pass (the HDR finishing step that gives WREN its contrast/colour
  // response): samples the RGBA16F scene target, decodes the scene shader's gamma encode back to
  // linear (pow 2.2 — order-preserving, so >1 HDR values survive the fp16 round trip), applies
  // exposure then the AgX curve (same constants as kSolidLitAgX / the WebGL2 preview), and writes
  // display-referred LDR. params.x = exposure.
  const char *kAgxTonemapPost = R"WGSL(
struct U { params : vec4<f32> };  // x = exposure, y = curve sel, z = time, w = flag bits (1 = vignette+grain, 2 = auto-exposure, 4 = SENSOR: no display dither)
@group(0) @binding(0) var<uniform> u : U;
@group(0) @binding(1) var tex : texture_2d<f32>;
@group(0) @binding(2) var samp : sampler;
@group(0) @binding(3) var adaptTex : texture_2d<f32>;  // 1x1: temporally adapted avg luminance

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  o.uv = vec2<f32>(p[vi].x * 0.5 + 0.5, 0.5 - p[vi].y * 0.5);
  return o;
}

fn agxDefaultContrast(x : vec3<f32>) -> vec3<f32> {
  let x2 = x * x;
  let x4 = x2 * x2;
  return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4
       - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

fn agx(cIn : vec3<f32>) -> vec3<f32> {
  let inset = mat3x3<f32>(
    vec3<f32>(0.842479062253094, 0.0423282422610123, 0.0423756549057051),
    vec3<f32>(0.0784335999999992, 0.878468636469772, 0.0784336),
    vec3<f32>(0.0792237451477643, 0.0791661274605434, 0.879142973793104));
  let outset = mat3x3<f32>(
    vec3<f32>(1.19687900512017, -0.0528968517574562, -0.0529716355144438),
    vec3<f32>(-0.0980208811401368, 1.15190312990417, -0.0980434501171241),
    vec3<f32>(-0.0990297440797205, -0.0989611768448433, 1.15107367264116));
  var c = inset * cIn;
  c = clamp((log2(max(c, vec3<f32>(1e-10))) + 12.47393) / (12.47393 + 4.026069),
            vec3<f32>(0.0), vec3<f32>(1.0));
  c = agxDefaultContrast(c);
  c = outset * c;
  return clamp(c, vec3<f32>(0.0), vec3<f32>(1.0));
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  // The scene shader's hdrMode arm writes LINEAR radiance — no decode here (the old pow(2.2)
  // "undo the encode" belonged to the display-referred era and double-compressed everything).
  let linear = max(textureSampleLevel(tex, samp, in.uv, 0.0).rgb, vec3<f32>(0.0, 0.0, 0.0));
  var exposure = u.params.x;
  if (exposure <= 0.0) { exposure = 1.0; }
  let camBits = u32(u.params.w + 0.5);
  // AUTO-EXPOSURE (camera pass): scale toward a mid-grey target from the temporally adapted
  // scene luminance — walk toward the bright doorway and the view adapts like an eye/camera.
  if ((camBits & 2u) != 0u) {
    let avg = textureLoad(adaptTex, vec2<i32>(0, 0), 0).r;
    exposure = exposure * clamp(0.16 / max(avg, 1e-4), 0.55, 1.7);  // gentle: keep night moody
  }
  var outc : vec3<f32>;
  if (u.params.y > 0.5) {
    // WREN-exact curve arm (hdr_resolve.frag: 1 - exp(-x * exposure), then gamma) — the
    // same-pipeline A/B so "better than WREN" is judged on the curve alone.
    let mapped = vec3<f32>(1.0, 1.0, 1.0) - exp(-linear * exposure);
    outc = pow(max(mapped, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(1.0 / 2.2, 1.0 / 2.2, 1.0 / 2.2));
  } else {
    // 0.65 = AgX exposure calibration (A/B-tuned on the city, 2026-08-19): AgX's toe lifts darks
    // at raw exposure 1.0 (asphalt read pale grey); 0.65 seats mid-grey where the LDR/WREN look
    // has it while keeping the filmic shoulder. Authored Viewpoint.exposure 1.0 = this calibrated
    // default; the WREN-curve arm above stays uncalibrated for exact parity.
    outc = agx(linear * exposure * 0.65);
  }
  // CAMERA FEEL: subtle vignette + animated photographic grain — the finishing pass that
  // separates "render" from "photograph". Deterministic per frame index (time = frame * dt).
  if ((camBits & 1u) != 0u) {
    let d = distance(in.uv, vec2<f32>(0.5, 0.5));
    outc = outc * (1.0 - 0.20 * smoothstep(0.42, 0.85, d));
    let gn = fract(sin(dot(in.pos.xy + vec2<f32>(u.params.z * 61.7, u.params.z * 123.4),
                           vec2<f32>(12.9898, 78.233))) * 43758.5453);
    outc = outc + vec3<f32>((gn - 0.5) * 0.010);
  }
  // W3 SENSOR bit (4): no display dither. A Camera DEVICE's bytes are a measurement, not a
  // picture — WREN's hdr_resolve.frag has no dither, and a fixed +-1-code spatial pattern is
  // noise a controller's thresholding or an ML consumer can see. The bit is set only by
  // OM_WGPU_XFER_WREN_SENSOR, never by the main view, so the display path is unchanged.
  if ((camBits & 4u) != 0u) {
    return vec4<f32>(outc, 1.0);
  }
  // The tonemap owns the display dither now (the scene shader's was pre-curve and therefore
  // non-uniform after it; the HDR path previously had none — banding on AgX sky gradients).
  let dith = fract(sin(dot(in.pos.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453);
  return vec4<f32>(outc + vec3<f32>((dith - 0.5) / 255.0), 1.0);
}
)WGSL";

  // Auto-exposure adaptation: a 1x1 pass sampling a sparse grid of the linear HDR scene,
  // computing geometric-mean luminance and easing toward it from the previous frame's value
  // (ping-pong) — the eye's adaptation, on the GPU, no readback.
  const char *kAdaptLum = R"WGSL(
@group(0) @binding(0) var scene : texture_2d<f32>;
@group(0) @binding(1) var samp : sampler;
@group(0) @binding(2) var prevA : texture_2d<f32>;

struct VOut { @builtin(position) pos : vec4<f32> };

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  var sum = 0.0;
  for (var y = 0; y < 5; y = y + 1) {
    for (var x = 0; x < 8; x = x + 1) {
      let uv = vec2<f32>((f32(x) + 0.5) / 8.0, (f32(y) + 0.5) / 5.0);
      let c = textureSampleLevel(scene, samp, uv, 0.0).rgb;
      let l = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
      sum = sum + log(max(l, 1e-4));
    }
  }
  let avg = exp(sum / 40.0);
  let prev = textureLoad(prevA, vec2<i32>(0, 0), 0).r;
  return vec4<f32>(mix(prev, avg, 0.07), 0.0, 0.0, 1.0);
}
)WGSL";

  // Temporal anti-aliasing resolve (post-tonemap LDR): reproject last frame's resolved output
  // through depth + the previous view-proj, clamp it to the current 3x3 neighbourhood (kills
  // ghosting), and blend. With the sub-pixel projection jitter this integrates shading over
  // time — specular sparkle, leaf-card crawl and thin-feature shimmer settle out.
  const char *kTaaMvResolve = R"WGSL(
struct TaaU {
  invVP  : mat4x4<f32>,
  prevVP : mat4x4<f32>,
  params : vec4<f32>,  // x = rev-Z flag, y = history weight, z = unused, w = history valid
};
@group(0) @binding(0) var<uniform> u : TaaU;
@group(0) @binding(1) var curTex : texture_2d<f32>;
@group(0) @binding(2) var histTex : texture_2d<f32>;
@group(0) @binding(3) var depthTex : texture_depth_multisampled_2d;
@group(0) @binding(4) var samp : sampler;

struct VOut { @builtin(position) pos : vec4<f32> };

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  let dims = vec2<f32>(textureDimensions(curTex));
  let pix = vec2<i32>(in.pos.xy);
  let cur = textureLoad(curTex, pix, 0).rgb;
  if (u.params.w < 0.5) {
    return vec4<f32>(cur, 1.0);
  }
  let z = textureLoad(depthTex, pix, 0);
  let uv = in.pos.xy / dims;
  let ndc = vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, z, 1.0);
  let w4 = u.invVP * ndc;
  let world = w4.xyz / w4.w;
  let pc = u.prevVP * vec4<f32>(world, 1.0);
  if (pc.w < 1e-4) {
    return vec4<f32>(cur, 1.0);
  }
  let pn = pc.xyz / pc.w;
  let puv = vec2<f32>(pn.x * 0.5 + 0.5, 0.5 - pn.y * 0.5);
  if (puv.x < 0.0 || puv.x > 1.0 || puv.y < 0.0 || puv.y > 1.0) {
    return vec4<f32>(cur, 1.0);
  }
  var hist = textureSampleLevel(histTex, samp, puv, 0.0).rgb;
  // neighbourhood clamp (3x3 min/max of the current frame) — the standard anti-ghosting box
  var mn = cur;
  var mx = cur;
  for (var dy = -1; dy <= 1; dy = dy + 1) {
    for (var dx = -1; dx <= 1; dx = dx + 1) {
      let q = clamp(pix + vec2<i32>(dx, dy), vec2<i32>(0, 0), vec2<i32>(dims) - vec2<i32>(1, 1));
      let c = textureLoad(curTex, q, 0).rgb;
      mn = min(mn, c);
      mx = max(mx, c);
    }
  }
  hist = clamp(hist, mn, mx);
  return vec4<f32>(mix(cur, hist, u.params.y), 1.0);
}
)WGSL";


  // Bloom post-process (wgpu counterpart of WREN's Viewpoint.bloomThreshold): three entry points
  // over one fullscreen-triangle vertex shader — bright-pass extract (LDR: keep near-clipped
  // pixels, the clamped emissive/sun whites), separable 9-tap gaussian blur (params.xy = step in
  // UV), and additive composite back onto the scene (pipeline blend = One/One).
  const char *kBloomPost = R"WGSL(
struct U { params : vec4<f32> };  // extract: x=threshold | blur: xy=dir*texel | composite: x=strength
@group(0) @binding(0) var<uniform> u : U;
@group(0) @binding(1) var tex : texture_2d<f32>;
@group(0) @binding(2) var samp : sampler;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  o.uv = vec2<f32>(p[vi].x * 0.5 + 0.5, 0.5 - p[vi].y * 0.5);
  return o;
}

@fragment
fn fs_extract(in : VOut) -> @location(0) vec4<f32> {
  let c = textureSampleLevel(tex, samp, in.uv, 0.0).rgb;
  let lum = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
  if (u.params.y > 0.5) {
    // HDR extract: `tex` is the LINEAR RGBA16F scene and params.x is the Viewpoint's
    // bloomThreshold in real luma units (WREN semantics -- only genuine HDR sources fire:
    // the sun disk, strong emissives, tight speculars). Normalized by 1/(1+lum) so the
    // LDR ping-pong chain can carry the halo; energy still shapes it.
    let k = clamp((lum - u.params.x) / max(u.params.x, 1e-3), 0.0, 1.0);
    let n = c * (k / (1.0 + lum));
    return vec4<f32>(min(n, vec3<f32>(1.0, 1.0, 1.0)), 1.0);
  }
  let k = smoothstep(u.params.x, 1.0, lum);
  return vec4<f32>(c * k, 1.0);
}

@fragment
fn fs_blur(in : VOut) -> @location(0) vec4<f32> {
  let w = array<f32, 5>(0.227027, 0.194594, 0.121622, 0.054054, 0.016216);
  var acc = textureSampleLevel(tex, samp, in.uv, 0.0).rgb * w[0];
  for (var i = 1; i < 5; i = i + 1) {
    let off = u.params.xy * f32(i);
    acc = acc + textureSampleLevel(tex, samp, in.uv + off, 0.0).rgb * w[i];
    acc = acc + textureSampleLevel(tex, samp, in.uv - off, 0.0).rgb * w[i];
  }
  return vec4<f32>(acc, 1.0);
}

@fragment
fn fs_composite(in : VOut) -> @location(0) vec4<f32> {
  let b = textureSampleLevel(tex, samp, in.uv, 0.0).rgb;
  return vec4<f32>(b * u.params.x, 1.0);  // additive blend (One/One) onto the scene
}
)WGSL";

  const char *kFullScreenOverlay = R"WGSL(
struct U { color : vec4<f32> };
@group(0) @binding(0) var<uniform> u : U;

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> @builtin(position) vec4<f32> {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  return vec4<f32>(p[vi], 0.0, 1.0);
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
  return u.color;
}
)WGSL";

  // R4 step-3c-A — screen-space textured-quad shader. Draws a texture into an NDC sub-rect
  // (no vertex buffer; the quad is generated from @builtin(vertex_index) + the rect uniform),
  // no lighting. The compositing primitive for device-output insets (camera/range-finder/
  // display image as a corner HUD) and any 2D overlay. v is flipped so an image that is
  // top-row-first lands right-side-up.
  // P7 (WREN retirement) -- the wgpu main view's HUD quad. See OmWgpuShaders.hpp for why this
  // is a separate pipeline from kTexturedQuad rather than a tweak of it.
  const char *kHudQuad = R"WGSL(
struct HudU {
  rect : vec4<f32>,   // (x0, y0, x1, y1) in NDC, y up
  tint : vec4<f32>,   // straight RGBA multiplier; the pass runs after the tonemap
  ctrl : vec4<f32>,   // x: 1 = sample the texture, 0 = flat tint; y: 1 = flip V
};
@group(0) @binding(0) var<uniform> u : HudU;
@group(0) @binding(1) var tex : texture_2d<f32>;
@group(0) @binding(2) var samp : sampler;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var q = array<vec2<f32>, 6>(
    vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 0.0), vec2<f32>(0.0, 1.0),
    vec2<f32>(1.0, 0.0), vec2<f32>(1.0, 1.0), vec2<f32>(0.0, 1.0));
  let c = q[vi];
  let ndc = vec2<f32>(mix(u.rect.x, u.rect.z, c.x), mix(u.rect.y, u.rect.w, c.y));
  var o : VOut;
  o.pos = vec4<f32>(ndc, 0.0, 1.0);
  // c.y runs 0 at the rect's NDC-y0 (its BOTTOM on screen) to 1 at y1 (its TOP), so the
  // first texel row lands at the top when v = 1 - c.y. ctrl.y flips that for sources whose
  // first row is the bottom one.
  let v = select(1.0 - c.y, c.y, u.ctrl.y > 0.5);
  o.uv = vec2<f32>(c.x, v);
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  var c = u.tint;
  if (u.ctrl.x > 0.5) {
    c = c * textureSample(tex, samp, in.uv);
  }
  // Premultiplied output for a One / OneMinusSrcAlpha blend.
  return vec4<f32>(c.rgb * c.a, c.a);
}
)WGSL";

  const char *kTexturedQuad = R"WGSL(
struct U { rect : vec4<f32> };  // (x0, y0, x1, y1) in NDC
@group(0) @binding(0) var<uniform> u : U;
@group(0) @binding(1) var tex : texture_2d<f32>;
@group(0) @binding(2) var samp : sampler;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var q = array<vec2<f32>, 6>(
    vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 0.0), vec2<f32>(0.0, 1.0),
    vec2<f32>(1.0, 0.0), vec2<f32>(1.0, 1.0), vec2<f32>(0.0, 1.0));
  let c = q[vi];
  let ndc = vec2<f32>(mix(u.rect.x, u.rect.z, c.x), mix(u.rect.y, u.rect.w, c.y));
  var o : VOut;
  o.pos = vec4<f32>(ndc, 0.0, 1.0);
  o.uv = vec2<f32>(c.x, 1.0 - c.y);
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  return textureSample(tex, samp, in.uv);
}
)WGSL";

  // T1.4 TAA — the temporal-resolve pass (WGSL port of taa-preview.html's resolveFS).
  // A fullscreen post pass (no vertex buffer; 6-vert quad from @builtin(vertex_index)) that
  // reprojects the previous resolved frame by a screen-space motion vector, neighborhood-clamps
  // it to the 3x3 AABB of the current frame (Karis 2014 ghost suppressor), rejects history that
  // reprojected off-screen, and blends current⊕history by an exponential feedback factor. Inputs:
  // curTex @1 (this frame, jittered scene) + histTex @2 (previous resolved frame) + a filtering
  // sampler @3. Controls in the @0 uniform: resMotion (xy = target px size, zw = screen-space
  // motion in px), ctrl (x = feedback / history weight, y = taa-enabled, z = clamp-enabled).
  // ctrl.y < 0.5 → passthrough of curTex (byte-identical to no-TAA), so the pass is a safe no-op.
  const char *kTaaResolve = R"WGSL(
struct TaaParams {
  resMotion : vec4<f32>,   // xy = target px size, zw = screen-space motion (px)
  ctrl      : vec4<f32>,   // x = feedback, y = taa-enabled, z = clamp-enabled, w unused
};
@group(0) @binding(0) var<uniform> u : TaaParams;
@group(0) @binding(1) var curTex : texture_2d<f32>;
@group(0) @binding(2) var histTex : texture_2d<f32>;
@group(0) @binding(3) var samp : sampler;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var q = array<vec2<f32>, 6>(
    vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 0.0), vec2<f32>(0.0, 1.0),
    vec2<f32>(1.0, 0.0), vec2<f32>(1.0, 1.0), vec2<f32>(0.0, 1.0));
  let c = q[vi];
  var o : VOut;
  o.pos = vec4<f32>(c * 2.0 - 1.0, 0.0, 1.0);  // fullscreen NDC
  o.uv = vec2<f32>(c.x, 1.0 - c.y);            // flip Y → texture space
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  let res = u.resMotion.xy;
  let uv = in.uv;
  let cur = textureSampleLevel(curTex, samp, uv, 0.0).rgb;
  if (u.ctrl.y < 0.5) {  // TAA disabled → passthrough
    return vec4<f32>(cur, 1.0);
  }
  let motion = u.resMotion.zw;
  let histUv = uv - motion / res;
  var hist = textureSampleLevel(histTex, samp, histUv, 0.0).rgb;
  if (u.ctrl.z > 0.5) {  // 3x3 neighborhood-AABB clamp (suppresses ghosting on motion)
    var mn = cur;
    var mx = cur;
    let texel = vec2<f32>(1.0, 1.0) / res;
    for (var y : i32 = -1; y <= 1; y = y + 1) {
      for (var x : i32 = -1; x <= 1; x = x + 1) {
        let s = textureSampleLevel(curTex, samp, uv + vec2<f32>(f32(x), f32(y)) * texel, 0.0).rgb;
        mn = min(mn, s);
        mx = max(mx, s);
      }
    }
    hist = clamp(hist, mn, mx);
  }
  // Drop history that reprojected off-screen (no valid sample there).
  let onScreen = select(0.0, 1.0,
    histUv.x >= 0.0 && histUv.x <= 1.0 && histUv.y >= 0.0 && histUv.y <= 1.0);
  let fb = u.ctrl.x * onScreen;
  return vec4<f32>(mix(cur, hist, fb), 1.0);
}
)WGSL";

  // T1.3 fog — analytic distance-fog resolve (the foundation toward volumetric; full froxel
  // ray-marched scattering is a later tier). A fullscreen post pass (quad from vertex_index, no
  // vertex buffer) that samples a scene-colour texture + a metric view-distance texture (R32Float,
  // as produced by clearAndDrawSceneRangeF32/DepthF32) and blends the scene toward the fog colour by
  // an exponential transmittance term `1 - exp(-density * distance)`. Bindings: FogParams uniform @0
  // (rgb = fog colour, w = density 1/m; `params` reserved for height fog) + sceneTex @1 + depthTex @2
  // (unfilterable) + a non-filtering sampler @3. params.w < 0.5 → scene passthrough (safe no-op).
  const char *kFogResolve = R"WGSL(
struct FogParams {
  fogColor : vec4<f32>,   // rgb = fog/inscatter colour, w = density (1/m)
  params   : vec4<f32>,   // x = height falloff (reserved), y = base height (reserved), z unused, w = enabled
};
@group(0) @binding(0) var<uniform> u : FogParams;
@group(0) @binding(1) var sceneTex : texture_2d<f32>;
@group(0) @binding(2) var depthTex : texture_2d<f32>;
@group(0) @binding(3) var samp : sampler;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var q = array<vec2<f32>, 6>(
    vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 0.0), vec2<f32>(0.0, 1.0),
    vec2<f32>(1.0, 0.0), vec2<f32>(1.0, 1.0), vec2<f32>(0.0, 1.0));
  let c = q[vi];
  var o : VOut;
  o.pos = vec4<f32>(c * 2.0 - 1.0, 0.0, 1.0);
  o.uv = vec2<f32>(c.x, 1.0 - c.y);
  return o;
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  let scene = textureSampleLevel(sceneTex, samp, in.uv, 0.0).rgb;
  if (u.params.w < 0.5) {  // fog disabled → passthrough
    return vec4<f32>(scene, 1.0);
  }
  let dist = textureSampleLevel(depthTex, samp, in.uv, 0.0).r;  // metric view distance (m)
  let density = max(u.fogColor.w, 0.0);
  let fogFactor = clamp(1.0 - exp(-density * dist), 0.0, 1.0);
  return vec4<f32>(mix(scene, u.fogColor.rgb, fogFactor), 1.0);
}
)WGSL";

  // R4 step-3c-A.1 — picking shader. Identical Scene uniform + vertex stage as
  // kSolidLit, but the fragment emits baseColor FLAT (no lighting), so an encoded
  // per-draw ID round-trips through an RGBA8 target unchanged for readback.
  const char *kSolidPick = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
};

@group(0) @binding(0) var<uniform> u : Scene;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> @builtin(position) vec4<f32> {
  return u.viewProj * (u.model * vec4<f32>(in.position, 1.0));
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
  return u.baseColor;  // flat encoded ID — no shading
}
)WGSL";

  // T1.2 CSM sub-step 3 — shadow-receiving lit shader. kSolidLit + a hardware
  // depth-comparison sample of the light-space shadow map. Uses its OWN uniform
  // (ShadowScene) because a shadow receiver needs the light viewProj (a full
  // mat4) which doesn't fit the lit Scene struct's pad0/pad1 slack — mirrors how
  // kSolidRangeF32 carries its own {viewProj,view,model} layout. Bind group:
  //   @binding(0) ShadowScene uniform (256 B: + lightViewProj + shadowParams)
  //   @binding(1) shadow depth texture (the clip.z light-depth map, R32Float as
  //               a sampled texture — NOT depth_2d, so we compare manually)
  //   @binding(2) sampler (filtering/clamp)
  // The fragment transforms worldPos by lightViewProj → light NDC, looks up the
  // stored light-space depth, and darkens the diffuse term when the fragment is
  // farther than the stored depth + bias (i.e. occluded). shadowParams.x is a
  // master strength in [0,1]; 0 → no shadowing → byte-identical to kSolidLit, so
  // an unbound/zeroed shadow setup is a safe no-op. UNREFERENCED until the
  // sub-step-3 pipeline + 3-entry bind group land; this establishes the WGSL
  // contract (sub-step-2a pattern: shader first, plumbing next).
  const char *kSolidLitShadow = R"WGSL(
struct ShadowScene {
  viewProj      : mat4x4<f32>,
  model         : mat4x4<f32>,
  lightViewProj : mat4x4<f32>,
  baseColor     : vec4<f32>,
  light         : vec4<f32>,
  shadowParams  : vec4<f32>,   // x = strength [0,1], y = depth bias, zw unused
};

@group(0) @binding(0) var<uniform> u : ShadowScene;
@group(0) @binding(1) var shadowTex : texture_2d<f32>;
@group(0) @binding(2) var shadowSamp : sampler;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) worldNormal : vec3<f32>,
  @location(1) worldPos    : vec3<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  let n3 = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.worldNormal = normalize(n3);
  out.worldPos = worldPos.xyz;
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let L = normalize(u.light.xyz);
  let ambient = u.light.w;
  let diff = max(dot(in.worldNormal, -L), 0.0);
  let intensity = clamp(ambient + diff, 0.0, 1.0);

  // Shadow term. strength 0 → fully lit (byte-identical to kSolidLit) so an
  // unbound/zeroed shadow setup is a safe no-op.
  var shadow = 1.0;
  let strength = u.shadowParams.x;
  if (strength > 0.0) {
    let lc = u.lightViewProj * vec4<f32>(in.worldPos, 1.0);
    if (lc.w > 0.0) {
      let ndc = lc.xyz / lc.w;
      // light-NDC xy [-1,1] → uv [0,1] (wgpu: y flips)
      let uv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
      if (uv.x >= 0.0 && uv.x <= 1.0 && uv.y >= 0.0 && uv.y <= 1.0 && ndc.z <= 1.0) {
        // R32Float is non-filterable in core WebGPU, so the bind group uses a
        // NON-filtering sampler + unfilterable-float texture; sample exact texels
        // at LOD 0 (textureSampleLevel, not the filtered textureSample).
        // 3x3 PCF: depth-compare a 1-texel neighbourhood and average occlusion,
        // for soft shadow edges. texel size from textureDimensions, so it is
        // resolution-independent. occluded=0 -> fully lit (byte-identical to the
        // old single tap); occluded=9 -> fully shadowed (also identical); only
        // edge fragments (0<occluded<9) get the soft penumbra.
        let bias = max(u.shadowParams.y, 1e-4);
        let texelUV = 1.0 / vec2<f32>(textureDimensions(shadowTex));
        var occluded = 0.0;
        for (var dy = -1; dy <= 1; dy = dy + 1) {
          for (var dx = -1; dx <= 1; dx = dx + 1) {
            let off = vec2<f32>(f32(dx), f32(dy)) * texelUV;
            let stored = textureSampleLevel(shadowTex, shadowSamp, uv + off, 0.0).r;
            if (ndc.z > stored + bias) {
              occluded = occluded + 1.0;
            }
          }
        }
        shadow = 1.0 - strength * (occluded / 9.0);   // soft occlusion
      }
    }
  }

  let lit = u.baseColor.rgb * (ambient + (intensity - ambient) * shadow);
  return vec4<f32>(lit, u.baseColor.a);
}
)WGSL";

  // T1.2 CSM (multi-cascade) — the N-cascade generalisation of kSolidLitShadow.
  // Pairs with OmWgpuSceneRenderer::buildCascadeLightViewProjs (the CPU fit). Instead
  // of one light frustum + one depth map, it carries:
  //   - lightViewProj : array<mat4x4<f32>, 4>  (one tight ortho VP per cascade)
  //   - cascadeSplits : vec4<f32>  (the FAR linear view-depth boundary of cascades 0..3,
  //                                 in the same units as the camera clip.w)
  //   - a texture_2d_array shadow map (one layer per cascade)
  // The fragment selects its cascade by its LINEAR view depth (the camera clip.w, passed
  // as a varying — a perspective projection's clip.w IS the view-space distance, the same
  // trick kSolidDistance uses), then PCF-samples that array layer through the cascade's own
  // light VP. shadowParams.z carries the live cascade count; shadowParams.x strength 0 →
  // byte-identical to kSolidLit, so an unbound/zeroed CSM setup is a safe no-op. UNREFERENCED
  // until the multi-cascade depth pass + pipeline (texture_2d_array bind) wire it — mirrors
  // how kSolidLitShadow's 3a string landed. naga-validated standalone (wgpu-py create_shader_
  // module on the live device) before commit; pipeline/bind-group validation arrives with the
  // wiring step. R32Float layers are non-filterable, so it uses textureSampleLevel + a
  // non-filtering sampler, exactly like kSolidLitShadow.
  const char *kSolidLitCsm = R"WGSL(
struct CsmScene {
  viewProj      : mat4x4<f32>,
  model         : mat4x4<f32>,
  lightViewProj : array<mat4x4<f32>, 4>,
  baseColor     : vec4<f32>,
  light         : vec4<f32>,             // xyz = directional light dir, w = ambient
  cascadeSplits : vec4<f32>,             // far view-depth of cascades 0,1,2,3
  shadowParams  : vec4<f32>,             // x = strength, y = bias, z = cascade count, w unused
};

@group(0) @binding(0) var<uniform> u : CsmScene;
@group(0) @binding(1) var shadowTexArray : texture_2d_array<f32>;
@group(0) @binding(2) var shadowSamp : sampler;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) worldNormal : vec3<f32>,
  @location(1) worldPos    : vec3<f32>,
  @location(2) viewDepth   : f32,        // camera clip.w = linear view-space distance
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  let n3 = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.worldNormal = normalize(n3);
  out.worldPos = worldPos.xyz;
  out.viewDepth = out.position.w;
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let L = normalize(u.light.xyz);
  let ambient = u.light.w;
  let diff = max(dot(in.worldNormal, -L), 0.0);
  let intensity = clamp(ambient + diff, 0.0, 1.0);

  // Shadow term. strength 0 → fully lit (byte-identical to kSolidLit).
  var shadow = 1.0;
  let strength = u.shadowParams.x;
  if (strength > 0.0) {
    // Pick the cascade whose far boundary first exceeds this fragment's view depth.
    // Unrolled compares (no dynamic vec component indexing); the matrix array IS
    // dynamically indexed, which WGSL permits.
    var ci : i32 = 0;
    if (in.viewDepth > u.cascadeSplits.x) { ci = 1; }
    if (in.viewDepth > u.cascadeSplits.y) { ci = 2; }
    if (in.viewDepth > u.cascadeSplits.z) { ci = 3; }
    let nc = i32(u.shadowParams.z);
    ci = clamp(ci, 0, max(nc - 1, 0));

    let lc = u.lightViewProj[ci] * vec4<f32>(in.worldPos, 1.0);
    if (lc.w > 0.0) {
      let ndc = lc.xyz / lc.w;
      let uv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
      if (uv.x >= 0.0 && uv.x <= 1.0 && uv.y >= 0.0 && uv.y <= 1.0 && ndc.z <= 1.0) {
        // 3x3 PCF over the selected array layer (resolution-independent texel size).
        let bias = max(u.shadowParams.y, 1e-4);
        let texelUV = 1.0 / vec2<f32>(textureDimensions(shadowTexArray));
        var occluded = 0.0;
        for (var dy = -1; dy <= 1; dy = dy + 1) {
          for (var dx = -1; dx <= 1; dx = dx + 1) {
            let off = vec2<f32>(f32(dx), f32(dy)) * texelUV;
            let stored = textureSampleLevel(shadowTexArray, shadowSamp, uv + off, ci, 0.0).r;
            if (ndc.z > stored + bias) {
              occluded = occluded + 1.0;
            }
          }
        }
        shadow = 1.0 - strength * (occluded / 9.0);
      }
    }
  }

  let lit = u.baseColor.rgb * (ambient + (intensity - ambient) * shadow);
  return vec4<f32>(lit, u.baseColor.a);
}
)WGSL";

  // T1.1: kSolidLit + AgX tonemap. Identical vertex stage + Lambertian shade as
  // kSolidLit, but the fragment runs the lit colour through the AgX filmic
  // image-formation transform (Sobotka/Wrensch minimal fit) before output — the
  // first real Tier-1 fidelity shader IN THE ENGINE (vs the engine-agnostic
  // WebGL2 spec preview docs/developer/agx-tonemap-preview.html, ported verbatim:
  // 3x3 inset → per-channel log2 encode over the AgX range → 6th-order contrast
  // curve → 3x3 outset → clamp). AgX desaturates highlights gracefully and avoids
  // the ACES "notorious-six" hue shift on saturated emissives (LEDs/beacons/sky).
  // Wired as a SEPARATE pipeline selected by a default-false flag, so the lit
  // path stays byte-identical (proven by the golden-image gate); opt-in via
  // OmCamera OMNISIM_CAMERA_AGX=1. See engine-migration-plan.md §14.4 T1.1.
  const char *kSolidLitAgX = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
};

@group(0) @binding(0) var<uniform> u : Scene;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) worldNormal : vec3<f32>,
  // T1.1 specular foundation: interpolated world position so the fragment can
  // form the view vector V = normalize(cameraPos - worldPos). Inert until the
  // fragment reads it (a later sub-step).
  @location(1) worldPos : vec3<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  let n3 = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.worldNormal = normalize(n3);
  out.worldPos = worldPos.xyz;
  return out;
}

// AgX 6th-order default-contrast curve (operates in the log-encoded [0,1] range).
fn agxDefaultContrast(x : vec3<f32>) -> vec3<f32> {
  let x2 = x * x;
  let x4 = x2 * x2;
  return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4
       - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

fn agx(cIn : vec3<f32>) -> vec3<f32> {
  // WGSL mat3x3 is column-major (each vec3 arg is a column) — same layout as the
  // GLSL preview's mat3 constructor, so the 9 constants map 1:1.
  let inset = mat3x3<f32>(
    vec3<f32>(0.842479062253094, 0.0423282422610123, 0.0423756549057051),
    vec3<f32>(0.0784335999999992, 0.878468636469772, 0.0784336),
    vec3<f32>(0.0792237451477643, 0.0791661274605434, 0.879142973793104));
  let outset = mat3x3<f32>(
    vec3<f32>(1.19687900512017, -0.0528968517574562, -0.0529716355144438),
    vec3<f32>(-0.0980208811401368, 1.15190312990417, -0.0980434501171241),
    vec3<f32>(-0.0990297440797205, -0.0989611768448433, 1.15107367264116));
  var c = inset * cIn;
  // log2 encode to ~[0,1] over the AgX dynamic range, then the contrast curve.
  c = clamp((log2(max(c, vec3<f32>(1e-10))) + 12.47393) / (12.47393 + 4.026069),
            vec3<f32>(0.0), vec3<f32>(1.0));
  c = agxDefaultContrast(c);
  c = outset * c;
  return clamp(c, vec3<f32>(0.0), vec3<f32>(1.0));
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let L = normalize(u.light.xyz);
  let ambient = u.light.w;
  let diff = max(dot(in.worldNormal, -L), 0.0);
  let intensity = clamp(ambient + diff, 0.0, 1.0);
  // Exposure (linear scale) in pad0.x lifts the lit colour into HDR range BEFORE
  // AgX, so the tonemap actually compresses rather than curving an already-LDR
  // signal (mirrors the preview's `sceneHDR * uExposure`). The host writes 1.0
  // by default (EV 0) → agx(lit*1.0) == agx(lit), byte-identical to the no-
  // exposure path. The <=0 guard keeps an unset/zero pad0 at unity, never black.
  var exposure = u.pad0.x;
  if (exposure <= 0.0) { exposure = 1.0; }
  // T1.1 HDR source: emissive (baseColor's PBRAppearance emissiveColor ×
  // emissiveIntensity, packed by the host into pad1.xyz) is added to the
  // diffuse-lit colour BEFORE exposure + AgX. A bright/over-1 emissive (LED,
  // beacon, sky) pushes `lit` past 1.0 so AgX has genuine HDR to compress
  // (highlight desaturation) instead of curving an already-LDR signal. Default
  // emissive 0 → lit == diffuse, byte-identical to the no-emissive AgX path.
  let emissive = u.pad1.xyz;
  // T1.1 specular HDR source: a Blinn-Phong highlight. smoothness (1-roughness)
  // is in pad1.w, the camera world pos in pad0.yzw, the fragment world pos is
  // interpolated. The half-vector uses -L (the to-light direction, matching the
  // diffuse dot(N,-L) convention). Gated on smoothness>0 so every roughness-1
  // world (and any frame the host leaves pad1.w=0) skips it entirely — no
  // normalize, no NaN risk, exactly zero contribution → byte-identical. A smooth
  // surface gets a bright (>1, ×4 gain) white highlight for AgX to compress +
  // desaturate, which is the point of the increment.
  let smoothness = u.pad1.w;
  var specColor = vec3<f32>(0.0, 0.0, 0.0);
  if (smoothness > 0.0) {
    let camPos = u.pad0.yzw;
    let V = normalize(camPos - in.worldPos);
    let H = normalize(-L + V);
    let nDotH = max(dot(in.worldNormal, H), 0.0);
    let shininess = exp2(1.0 + smoothness * 7.0);   // 2 (rough) .. 256 (mirror)
    let spec = pow(nDotH, shininess) * smoothness * 4.0;
    specColor = vec3<f32>(spec, spec, spec);
  }
  let lit = (u.baseColor.rgb * intensity + emissive + specColor) * exposure;
  return vec4<f32>(agx(lit), u.baseColor.a);
}
)WGSL";

  // R5 depth/RangeFinder shader. Same Scene uniform + vertex transform as
  // kSolidLit; fragment outputs LINEAR view-space distance as grayscale.
  // clip.w == view-space -z == linear distance along the camera axis, so we
  // read it straight from the vertex output (no depth-texture copy). far is in
  // pad0.x. The 0 far guard keeps an unset pad0 from producing NaN/inf.
  const char *kSolidDistance = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
};

@group(0) @binding(0) var<uniform> u : Scene;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) viewDepth : f32,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.viewDepth = out.position.w;   // clip.w = linear view-space distance
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  let far = max(u.pad0.x, 1e-3);
  let d = clamp(in.viewDepth / far, 0.0, 1.0);
  return vec4<f32>(d, d, d, 1.0);
}
)WGSL";

  // R5b RangeFinder shader, F32 variant. Identical vertex transform to
  // kSolidDistance, but the fragment writes the RAW linear view-space distance
  // (meters) into an R32Float color target — no far-plane normalization, no
  // 8-bit quantization. This is what a real RangeFinder device node outputs:
  // metric depth at full float precision. The R32Float readback is copyable
  // (unlike the Depth24Plus depth attachment), so clip.w → color works directly.
  const char *kSolidDistanceF32 = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
};

@group(0) @binding(0) var<uniform> u : Scene;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) viewDepth : f32,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.viewDepth = out.position.w;   // clip.w = linear view-space distance (m)
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  // Raw metric distance in R; G/B unused, A=1. Target is R32Float so only
  // the R channel is stored/read back.
  return vec4<f32>(in.viewDepth, 0.0, 0.0, 1.0);
}
)WGSL";

  // T1.2 CSM shadow-map depth shader. Like kSolidDistanceF32 but writes the
  // post-projection NDC depth `clip.z / clip.w` (== @builtin(position).z, ∈[0,1]
  // in wgpu) instead of `clip.w`. This is the correct depth for a SHADOW MAP and
  // — unlike clip.w — it is well-defined under an ORTHOGRAPHIC light projection
  // (where clip.w ≡ 1, so clip.w-based depth collapses to a constant; that bug
  // was caught by the sub-step-2 light-depth verification). For a perspective
  // projection clip.z/clip.w is the usual non-linear depth; for the ortho light
  // matrix from buildOrthoLightViewProj it is the linear light-space depth the
  // shadow comparison needs. Renders into the same R32Float target as the other
  // F32 depth passes; reads back width*height floats in [0,1].
  const char *kSolidClipDepthF32 = R"WGSL(
struct Scene {
  viewProj  : mat4x4<f32>,
  model     : mat4x4<f32>,
  baseColor : vec4<f32>,
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
};

@group(0) @binding(0) var<uniform> u : Scene;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  // @builtin(position).z in the fragment is already the perspective-divided NDC
  // depth (clip.z/clip.w) in wgpu's [0,1] range — correct for both perspective
  // and orthographic projections. Write it to R; G/B unused, A=1 (R32Float).
  return vec4<f32>(in.position.z, 0.0, 0.0, 1.0);
}
)WGSL";

  // Pass-1 (shadow map) clip-depth with the light VP in a tiny per-cascade uniform (b1, dynamic
  // offset) and the model read from the SAME per-draw Scene slots pass 2 uses (b0, offset +64).
  // The per-frame pass-1 staging (layers x draws x 256 B, ~3.5 MB/frame on a 4.6k-draw scene at
  // 3 cascades) is GONE; the CPU writes 64 B per cascade layer instead.
  const char *kSolidClipDepthSharedVP = R"WGSL(
struct Slot {
  viewProjDead : mat4x4<f32>,   // dead bytes of the shared slot layout (pass 2 reads its VP from LightU)
  model        : mat4x4<f32>,
  padStrideA   : mat4x4<f32>,   // pad to the 256-B slot stride (array element stride == struct size)
  padStrideB   : mat4x4<f32>,
};
struct VpU { vp : mat4x4<f32> };

@group(0) @binding(0) var<storage, read> slots : array<Slot>;
@group(0) @binding(1) var<uniform> vpu : VpU;

struct VertexIn {
  @builtin(instance_index) slot : u32,   // firstInstance = the draw's Scene-slot index
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};
struct VertexOut {
  @builtin(position) position : vec4<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  var out : VertexOut;
  out.position = vpu.vp * (slots[in.slot].model * vec4<f32>(in.position, 1.0));
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  return vec4<f32>(in.position.z, 0.0, 0.0, 1.0);
}
)WGSL";

  // GTAO estimate + depth-aware denoise (port of WREN's gtao.frag / gtao_spatial_denoise.frag —
  // horizon-based, cosine-weighted AO, WREN's quality edge). PERF DESIGN: samples the SCENE's own
  // MSAA depth attachment (sample 0) via textureLoad instead of a dedicated geometry prepass —
  // that prepass cost a third full-scene encode (~4.6k draws on the city). Normals are
  // reconstructed from depth (closer-side differencing), so no normal buffer is needed either.
  // Differences from WREN, by design: interleaved-gradient noise replaces the 24-frame temporal
  // jitter (no reprojection infra), 3 slices x 4 steps x 2 signs run per frame. The estimate
  // outputs DISPLAY-encoded AO (pow 1/2.2) so the existing Dst*Zero multiply onto the tonemapped
  // scene approximates a linear-light multiply. fs_denoise is a 4x4 depth-aware filter replacing
  // the gaussian that bled AO halos across silhouettes.
  const char *kSsaoGtao = R"WGSL(
struct U {
  p0 : vec4<f32>,  // x = radius (world m), y = intensity, z = reversed-Z flag, w = slice count
  p1 : vec4<f32>,  // x = tanHalfFovX, y = tanHalfFovY, z = nearZ, w = farZ
  p2 : vec4<f32>,  // x = full W, y = full H, z = projScale (0.5*H/tanHalfFovY), w = max pixel radius
  r0 : vec4<f32>,  // camera right (world) — kept for layout stability, unused since depth-normals
  u0 : vec4<f32>,
  b0 : vec4<f32>,
};
@group(0) @binding(0) var<uniform> u : U;
@group(0) @binding(1) var depthTex : texture_depth_multisampled_2d;
@group(0) @binding(2) var aoTex : texture_2d<f32>;  // denoise input; unused by fs_main

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) uv : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 0.0, 1.0);
  o.uv = vec2<f32>(p[vi].x * 0.5 + 0.5, 0.5 - p[vi].y * 0.5);
  return o;
}

fn rawDepthAt(px : vec2<i32>) -> f32 {
  let dim = vec2<i32>(textureDimensions(depthTex));
  let c = clamp(px, vec2<i32>(0, 0), dim - vec2<i32>(1, 1));
  var z = textureLoad(depthTex, c, 0);
  if (u.p0.z > 0.5) { z = 1.0 - z; }  // reversed-Z attachment → undo before linearizing
  return z;
}

fn linOf(z : f32) -> f32 {
  let nearZ = u.p1.z;
  let farZ = u.p1.w;
  return nearZ * farZ / (farZ - z * (farZ - nearZ));
}

// view-space position at a FULL-res pixel (v grows DOWN; view y grows UP; z = +depth forward);
// z <= 0 = sky sentinel
fn viewPosAt(px : vec2<i32>) -> vec3<f32> {
  let z = rawDepthAt(px);
  if (z >= 0.9999) { return vec3<f32>(0.0, 0.0, 0.0); }
  let d = linOf(z);
  let uv = (vec2<f32>(px) + vec2<f32>(0.5, 0.5)) / vec2<f32>(u.p2.x, u.p2.y);
  let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
  return vec3<f32>(ndc.x * u.p1.x * d, ndc.y * u.p1.y * d, d);
}

@fragment
fn fs_main(in : VOut) -> @location(0) vec4<f32> {
  // this fragment runs at HALF res; sample the full-res depth at the pixel center it covers
  let px = vec2<i32>(in.pos.xy * 2.0);
  let pos = viewPosAt(px);
  if (pos.z <= 0.0) { return vec4<f32>(1.0, 1.0, 1.0, 1.0); }  // sky
  // normal from depth: difference toward whichever neighbour is closer in depth (avoids
  // silhouette bleed), cross product oriented toward the viewer
  let pr = viewPosAt(px + vec2<i32>(1, 0));
  let pl = viewPosAt(px - vec2<i32>(1, 0));
  let pu = viewPosAt(px + vec2<i32>(0, 1));
  let pd = viewPosAt(px - vec2<i32>(0, 1));
  var ddx = pr - pos;
  if (pl.z > 0.0 && (pr.z <= 0.0 || abs(pl.z - pos.z) < abs(pr.z - pos.z))) { ddx = pos - pl; }
  var ddy = pu - pos;
  if (pd.z > 0.0 && (pu.z <= 0.0 || abs(pd.z - pos.z) < abs(pu.z - pos.z))) { ddy = pos - pd; }
  var n = cross(ddy, ddx);
  if (dot(n, n) < 1e-12) { return vec4<f32>(1.0, 1.0, 1.0, 1.0); }
  n = normalize(n);
  let viewVec = normalize(-pos);
  if (dot(n, viewVec) < 0.0) { n = -n; }

  let projPix = clamp(u.p0.x * u.p2.z / pos.z, 2.0, u.p2.w);
  let stepPix = projPix / 4.0;
  let ign = fract(52.9829189 * fract(0.06711056 * f32(px.x) + 0.00583715 * f32(px.y)));

  // Falloff window follows the radius the disc ACTUALLY searches, not the authored one.
  // WREN's gtao.frag used its projected radius unclamped, so its world-space window
  // (FALLOFF_END2 = radius^2, FALLOFF_START2 = 0.16 -- i.e. full weight inside 0.2*radius,
  // tapering to zero at radius) always matched the disc it sampled. The pixel clamp above is
  // a perf guard this port added, and once it binds -- which it does for anything nearer than
  // radius*projScale/maxPix, i.e. essentially always -- every sample lands far inside a
  // 0.4 m FALLOFF_START2 and is taken at FULL weight. The kernel then degenerates into a flat,
  // unattenuated 64-pixel disc: a fixed-size dark ring glued to every silhouette, independent
  // of world scale. That is the "shadow aura". Back-projecting the clamped radius restores the
  // pairing; where the clamp does not bind, effR == u.p0.x and this is WREN's formula exactly.
  let effR = projPix * pos.z / u.p2.z;
  let falloffEnd2 = max(effR * effR, 1e-6);
  let falloffStart2 = falloffEnd2 * 0.04;
  let sliceCount = i32(u.p0.w + 0.5);
  var aoSum = 0.0;
  for (var s = 0; s < 4; s = s + 1) {
    if (s >= sliceCount) { break; }
    let phi = 3.14159265 * (f32(s) + ign) / u.p0.w;
    let dir2v = vec2<f32>(cos(phi), sin(phi));            // slice direction in VIEW xy
    let dirPx = vec2<f32>(dir2v.x, -dir2v.y);             // pixel steps (v flipped vs view y)
    var h = vec2<f32>(-1.0, -1.0);
    var stepCur = fract(ign * 7.0) * (stepPix - 1.0) + 1.0;
    for (var j = 0; j < 4; j = j + 1) {
      let offs = vec2<i32>(dirPx * stepCur);
      stepCur = stepCur + stepPix;
      let sp = viewPosAt(px + offs);
      if (sp.z > 0.0) {
        let toS = sp - pos;
        let d2 = dot(toS, toS);
        let hc = dot(toS, viewVec) * inverseSqrt(max(d2, 1e-8));
        let fall = 2.0 * clamp((d2 - falloffStart2) / (falloffEnd2 - falloffStart2), 0.0, 1.0);
        h.x = max(h.x, hc - fall);
      }
      let sm = viewPosAt(px - offs);
      if (sm.z > 0.0) {
        let toS = sm - pos;
        let d2 = dot(toS, toS);
        let hc = dot(toS, viewVec) * inverseSqrt(max(d2, 1e-8));
        let fall = 2.0 * clamp((d2 - falloffStart2) / (falloffEnd2 - falloffStart2), 0.0, 1.0);
        h.y = max(h.y, hc - fall);
      }
    }
    let h1 = acos(clamp(h.x, -1.0, 1.0));
    let h2 = acos(clamp(h.y, -1.0, 1.0));
    let searchDir3 = vec3<f32>(dir2v.x, dir2v.y, 0.0);
    let bitangent = normalize(cross(searchDir3, viewVec));
    let planeNormal = cross(viewVec, bitangent);
    let projN = n - bitangent * dot(n, bitangent);
    let projLen = length(projN);
    let invProjLen = 1.0 / (projLen + 1e-6);
    let cosXi = dot(projN, planeNormal) * invProjLen;
    let nAng = acos(clamp(cosXi, -1.0, 1.0)) - 1.5707963;
    let cosN = dot(projN, viewVec) * invProjLen;
    let sinN2 = -2.0 * cosXi;
    let hx = nAng + max(-h1 - nAng, -1.5707963);
    let hy = nAng + min(h2 - nAng, 1.5707963);
    let arc = projLen * 0.25 *
              ((hx * sinN2 + cosN - cos(2.0 * hx - nAng)) + (hy * sinN2 + cosN - cos(2.0 * hy - nAng)));
    aoSum = aoSum + clamp(arc, 0.0, 1.0);
  }
  let distFade = clamp(0.01 * (200.0 - pos.z), 0.0, 1.0);
  let aoLin = 1.0 - ((1.0 - aoSum / u.p0.w) * distFade);
  let ao = pow(clamp(mix(1.0, aoLin, u.p0.y), 0.0, 1.0), 1.0 / 2.2);
  return vec4<f32>(ao, ao, ao, 1.0);
}

// Depth-aware 4x4 spatial denoise (gtao_spatial_denoise.frag): AO from the half-res estimate
// (aoTex), reference depth from the scene MSAA attachment.
@fragment
fn fs_denoise(in : VOut) -> @location(0) vec4<f32> {
  let hpx = vec2<i32>(in.pos.xy);           // half-res pixel
  let px = hpx * 2;                          // full-res pixel it covers
  let zRef = rawDepthAt(px);
  if (zRef >= 0.9999) { return vec4<f32>(1.0, 1.0, 1.0, 1.0); }
  let dRef = linOf(zRef);
  let aoDim = vec2<i32>(textureDimensions(aoTex));
  var sum = 0.0;
  var wsum = 0.0;
  for (var dy = -1; dy <= 2; dy = dy + 1) {
    for (var dx = -1; dx <= 2; dx = dx + 1) {
      let sp = clamp(hpx + vec2<i32>(dx, dy), vec2<i32>(0, 0), aoDim - vec2<i32>(1, 1));
      let ao = textureLoad(aoTex, sp, 0).r;
      let ds = linOf(rawDepthAt(sp * 2));
      let rel = abs(ds - dRef) / (dRef * 0.1);
      let w = max(0.0, 0.1 - rel) * 30.0 + 1e-4;
      sum = sum + ao * w;
      wsum = wsum + w;
    }
  }
  let ao = sum / wsum;
  return vec4<f32>(ao, ao, ao, 1.0);
}
)WGSL";

  // R5d Lidar shader: RADIAL range (euclidean distance from the camera to the
  // surface point), not planar depth. A Lidar ray measures true distance, so
  // off-axis pixels read length(viewPos) > clip.w. Needs the view matrix (to
  // recover view-space position) in addition to viewProj, so this uses its own
  // 192-byte uniform {viewProj, view, model} — distinct from the lit/depth
  // Scene struct but the same 192 B minBindingSize, so it reuses the scene
  // bind-group layout. Output is RAW metres into an R32Float target.
  const char *kSolidRangeF32 = R"WGSL(
struct RangeScene {
  viewProj : mat4x4<f32>,
  view     : mat4x4<f32>,
  model    : mat4x4<f32>,
};

@group(0) @binding(0) var<uniform> u : RangeScene;

struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) viewPos : vec3<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.viewPos = (u.view * worldPos).xyz;   // position relative to the camera
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  // Radial distance from the camera origin to this fragment, in metres.
  return vec4<f32>(length(in.viewPos), 0.0, 0.0, 1.0);
}
)WGSL";

  const char *kTriangleVertexBuffer = R"WGSL(
struct VertexIn {
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
};

struct VertexOut {
  @builtin(position) position : vec4<f32>,
  @location(0) normal : vec3<f32>,
  @location(1) uv     : vec2<f32>,
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  var out : VertexOut;
  out.position = vec4<f32>(in.position, 1.0);
  out.normal = in.normal;
  out.uv = in.uv;
  return out;
}

@fragment
fn fs_main(in : VertexOut) -> @location(0) vec4<f32> {
  // Color = positive half of normal vector. This makes the fragment
  // output a function of the per-vertex normal attribute, which
  // proves the vertex-buffer fetch + interpolation works end-to-end.
  let c = max(in.normal, vec3<f32>(0.0));
  return vec4<f32>(c, 1.0);
}
)WGSL";

}  // namespace OmWgpuShaders
