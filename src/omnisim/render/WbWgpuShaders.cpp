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

#include "WbWgpuShaders.hpp"

namespace WbWgpuShaders {

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
  light     : vec4<f32>,
  pad0      : vec4<f32>,
  pad1      : vec4<f32>,
  // TextureTransform as a 2D affine: uv' = (dot(uvA.xy,uv), dot(uvA.zw,uv)) + uvB.xy.
  // Identity (1,0,0,1)/(0,0) when the appearance has none. Grass{scale 40 40} on a bare
  // Plane was sampled untransformed -> one tile smeared across 200 m (white-streak bug).
  uvA       : vec4<f32>,
  uvB       : vec4<f32>,
};
struct LightU {
  lightViewProj : mat4x4<f32>,
  shadowParams  : vec4<f32>,   // x = strength [0,1], y = depth bias, z = hemisphere-IBL enabled (>0.5), w = day-night direct scale (0 = off)
  skyColor      : vec4<f32>,   // rgb = sky/zenith ambient colour, w = ambient intensity scale
  groundColor   : vec4<f32>,   // rgb = ground ambient colour (xyz), w unused
  upDir         : vec4<f32>,   // xyz = world up direction (for the hemisphere blend), w unused
  fogParams     : vec4<f32>,   // rgb = display-space fog colour, w = exp density (0 = no fog)
};

@group(0) @binding(0) var<uniform> u : Scene;
@group(0) @binding(1) var albedoTex : texture_2d<f32>;
@group(0) @binding(2) var roughTex : texture_2d<f32>;
@group(0) @binding(3) var metalTex : texture_2d<f32>;
@group(0) @binding(4) var normalTex : texture_2d<f32>;
@group(0) @binding(5) var albedoSamp : sampler;
@group(0) @binding(6) var shadowTex : texture_2d<f32>;
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
};

@vertex
fn vs_main(in : VertexIn) -> VertexOut {
  let worldPos = u.model * vec4<f32>(in.position, 1.0);
  let n3 = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
  var out : VertexOut;
  out.position = u.viewProj * worldPos;
  out.worldNormal = normalize(n3);
  out.uv = vec2<f32>(dot(u.uvA.xy, in.uv), dot(u.uvA.zw, in.uv)) + u.uvB.xy;
  out.worldPos = worldPos.xyz;
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
  // WREN-parity: sample albedo RAW (no sRGB→linear decode). WREN lights the raw texel and gamma-
  // encodes the output (the R5 camera-sensor parity proved its texture path is linear), washing
  // textured surfaces bright/pastel — the glTF-correct decode left wgpu's city pavements dark
  // brown + saturated vs WREN's near-white (the dominant failing pixels in the parity gate).
  let albedoTexLin = textureSample(albedoTex, albedoSamp, in.uv).rgb;
  let albedo = albedoTexLin * u.baseColor.rgb;
  let metal = clamp(textureSample(metalTex, albedoSamp, in.uv).r, 0.0, 1.0);

  // Shadow term (PCF 3x3) from the light-space depth map. strength 0 → fully lit.
  var shadow = 1.0;
  let strength = lu.shadowParams.x;
  if (strength > 0.0) {
    // NORMAL-OFFSET shadows: shift the receiver point ~1.5 shadow texels (≈12 cm on the 45 m
    // fitted frustum) along the geometric normal before projecting into light space. The PCF taps
    // compare against texels up to 3 away; on a slanted receiver (flat floor + low sun) those
    // legitimately differ by ~0.5 m of depth, which bias scaling cannot cover without metre-scale
    // peter-panning — the texel-grain false-occlusion BLOCKS seen on spot.wbt. The offset moves
    // the receiver off its own surface so every tap clears it.
    let shadowPos = in.worldPos + Ngeo * 0.12;
    let lc = lu.lightViewProj * vec4<f32>(shadowPos, 1.0);
    if (lc.w > 0.0) {
      let ndc = lc.xyz / lc.w;
      let suv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
      if (suv.x >= 0.0 && suv.x <= 1.0 && suv.y >= 0.0 && suv.y <= 1.0 && ndc.z <= 1.0) {
        // DEBUG (bias passed NEGATIVE): visualize the raw depth-compare error field instead of
        // shading — red = fragment deeper than stored (would occlude), green = shallower, scaled
        // x20. The definitive view of WHY a comparison misbehaves (sign/shape/magnitude).
        if (lu.shadowParams.y < 0.0) {
          let st = textureSampleLevel(shadowTex, shadowSamp, suv, 0.0).r;
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
        let nClip = lu.lightViewProj * vec4<f32>(Ngeo, 0.0);
        let nzSafe = sign(nClip.z) * max(abs(nClip.z), 1e-4);
        let dzduv = clamp(vec2<f32>(-2.0 * nClip.x / nzSafe, 2.0 * nClip.y / nzSafe),
                          vec2<f32>(-10.0, -10.0), vec2<f32>(10.0, 10.0));
        // PCF 5x5 at 1.5-texel spread: a wider, graded penumbra so the city-scale map's blocky
        // shadow edges soften toward WREN's filtered look.
        var occ = 0.0;
        for (var dy = -2; dy <= 2; dy = dy + 1) {
          for (var dx = -2; dx <= 2; dx = dx + 1) {
            let off = vec2<f32>(f32(dx), f32(dy)) * 1.5 * texelUV;
            let zExp = ndc.z + dzduv.x * off.x + dzduv.y * off.y;
            let stored = textureSampleLevel(shadowTex, shadowSamp, suv + off, 0.0).r;
            if (zExp > stored + bias) {
              occ = occ + 1.0;
            }
          }
        }
        shadow = 1.0 - strength * (occ / 25.0);
      }
    }
  }

  // GGX direct specular (Cook-Torrance), same as kSolidLitTextured.
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

  // Unshadowed ambient + shadowed direct (diffuse + specular).
  // R4 lighting convergence: hemisphere-IBL ambient — matches OmniSimSky's directional sky fill
  // (sky colour from above, ground colour from below, blended by the surface normal's up-component),
  // so shadowed regions take a sky tint instead of a flat grey. shadowParams.z <= 0.5 → flat scalar
  // ambient (byte-identical to the pre-hemisphere path; the default when no sky params are supplied).
  let diffuse = albedo * (1.0 - metal);
  var ambTerm = vec3<f32>(ambient, ambient, ambient);
  if (lu.shadowParams.z > 0.5) {
    let upd = normalize(lu.upDir.xyz);
    let t = clamp(dot(N, upd) * 0.5 + 0.5, 0.0, 1.0);
    ambTerm = mix(lu.groundColor.rgb, lu.skyColor.rgb, t) * lu.skyColor.w;
  }
  let ambientPart = diffuse * ambTerm;
  // Day-night: shadowParams.w (0 = legacy off → byte-identical) scales the DIRECT term only —
  // 1.0 in full day (unchanged), → 0 as the sun sets, so geometry darkens with the sky dome.
  var directScale = 1.0;
  if (lu.shadowParams.w > 0.001) {
    directScale = lu.shadowParams.w;
  }
  let directPart = (diffuse * NdotL + specBRDF * NdotL) * shadow * directScale;
  // Self-emission (pad1.xyz = emissiveColor × intensity): independent of sun/shadows/day-night —
  // shop strips, traffic lights and headlights stay lit at night.
  let col = ambientPart + directPart + u.pad1.xyz;
  // WREN-exact exponential fog (fog.frag): factor = exp2(-density·d), blend = pow(1-factor, 2.2).
  // Mixed in LINEAR space BEFORE the display encode — WREN fogs inside its HDR pipeline, so the
  // fade composes dark/moody; mixing the raw pale fog colour in display space made distant
  // streets/grass vanish toward WHITE (the user-visible zoom-out fade).
  var colOut = col;
  if (lu.fogParams.w > 0.0) {
    let dist = length(u.pad0.yzw - in.worldPos);
    let fogF = pow(clamp(1.0 - exp2(-lu.fogParams.w * dist), 0.0, 1.0), 2.2);
    let fogLin = pow(max(lu.fogParams.xyz, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(2.2));
    colOut = mix(colOut, fogLin, fogF);
  }
  var outRgb = pow(max(colOut, vec3<f32>(0.0, 0.0, 0.0)), vec3<f32>(1.0 / 2.2));
  // ±½-LSB hash dither before the 8-bit write: faint wide gradients (a broad rough-surface GGX
  // sheen, the sky dome) otherwise quantize into visible banding contours — WREN avoids this via
  // its HDR pipeline + dithered tonemap; the spot.wbt flat floor showed ours as giant dark rings.
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
  right    : vec4<f32>,  // camera right * tan(fovX/2)
  up       : vec4<f32>,  // camera up * tan(fovY/2)
  fwd      : vec4<f32>,  // camera forward
  sunDir   : vec4<f32>,  // TOWARD the sun, normalized
  sunColor : vec4<f32>,  // DirectionalLight colour
  worldUp  : vec4<f32>,  // world up (opposite gravity)
};
@group(0) @binding(0) var<uniform> u : SkyU;

struct VOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) ndc : vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vi], 1.0, 1.0);
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
  sky = sky + (disk * 2.0 + halo) * u.sunColor.xyz * day;
  let dith = fract(sin(dot(in.pos.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453);
  return vec4<f32>(sky + vec3<f32>((dith - 0.5) / 255.0), 1.0);
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
  // e.g. spot.wbt top-down). Radius shrinks with distance for a constant world-size kernel.
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
struct U { params : vec4<f32> };
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
  let enc = textureSampleLevel(tex, samp, in.uv, 0.0).rgb;
  let linear = pow(max(enc, vec3<f32>(0.0)), vec3<f32>(2.2));  // undo the scene shader's encode
  var exposure = u.params.x;
  if (exposure <= 0.0) { exposure = 1.0; }
  return vec4<f32>(agx(linear * exposure), 1.0);
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
  // Pairs with WbWgpuSceneRenderer::buildCascadeLightViewProjs (the CPU fit). Instead
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
  // WbCamera OMNISIM_CAMERA_AGX=1. See engine-migration-plan.md §14.4 T1.1.
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

}  // namespace WbWgpuShaders
