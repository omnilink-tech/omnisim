# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""T1.2 CSM (cascaded shadow maps) — end-to-end render PROTOTYPE / validator.

Renders the on-GPU multi-cascade-shadow design that the engine wiring
(WbWgpuRenderTarget) will implement, entirely in Python via wgpu-py (bundled
wgpu_native + naga), so it needs NO native engine build:

  * a Python port of the engine's `buildCascadeLightViewProjs`
    (src/omnisim/nodes/WbWgpuSceneRenderer.cpp) — verified bit-for-bit against
    the C++ frustum-corner-containment invariant;
  * the LIVE `kSolidLitCsm` shader, read straight out of
    src/omnisim/render/WbWgpuShaders.cpp (so this doubles as a regression guard:
    break the shader and this fails);
  * render N cascade depth layers into a `texture_2d_array`, then the lit+shadow
    pass, and READ BACK pixels.

It asserts the design is correct: a floor point occluded by a caster is SHADOWED
while a point away from it stays LIT, strength-0 is a no-op, and the shadowed
point's linear view depth routes it through a NON-zero cascade (so multi-cascade
selection + a per-cascade light VP + the right array layer are genuinely
exercised, not a single near cascade covering everything).

Requires: `pip install wgpu numpy` + a GPU. Run: `python docs/developer/csm_render_prototype.py`.
Exit 0 = all checks pass. This is a design/validation harness, not a shipped path.
"""
import math
import os
import re
import sys

import numpy as np
import wgpu

# ---------------------------------------------------------------------------
# Verified math (column-major float[16]) — ports of WbWgpuSceneRenderer.cpp.
# ---------------------------------------------------------------------------
def perspective(fovY, aspect, zN, zF):
    f = 1.0 / math.tan(fovY * 0.5)
    m = [0.0] * 16
    m[0] = f / aspect; m[5] = f; m[10] = zF / (zN - zF); m[11] = -1.0; m[14] = (zF * zN) / (zN - zF)
    return m

def ortho_offcenter(l, r, b, t, n, f):
    m = [0.0] * 16
    m[0] = 2.0 / (r - l); m[5] = 2.0 / (t - b); m[10] = 1.0 / (n - f)
    m[12] = -(r + l) / (r - l); m[13] = -(t + b) / (t - b); m[14] = n / (n - f); m[15] = 1.0
    return m

def mul4(A, B):
    o = [0.0] * 16
    for col in range(4):
        for row in range(4):
            o[col * 4 + row] = sum(A[k * 4 + row] * B[col * 4 + k] for k in range(4))
    return o

def mulvec4(M, v):
    return [sum(M[k * 4 + row] * v[k] for k in range(4)) for row in range(4)]

def invert4(m):
    inv = [0.0] * 16
    inv[0] = m[5]*m[10]*m[15]-m[5]*m[11]*m[14]-m[9]*m[6]*m[15]+m[9]*m[7]*m[14]+m[13]*m[6]*m[11]-m[13]*m[7]*m[10]
    inv[4] = -m[4]*m[10]*m[15]+m[4]*m[11]*m[14]+m[8]*m[6]*m[15]-m[8]*m[7]*m[14]-m[12]*m[6]*m[11]+m[12]*m[7]*m[10]
    inv[8] = m[4]*m[9]*m[15]-m[4]*m[11]*m[13]-m[8]*m[5]*m[15]+m[8]*m[7]*m[13]+m[12]*m[5]*m[11]-m[12]*m[7]*m[9]
    inv[12] = -m[4]*m[9]*m[14]+m[4]*m[10]*m[13]+m[8]*m[5]*m[14]-m[8]*m[6]*m[13]-m[12]*m[5]*m[10]+m[12]*m[6]*m[9]
    inv[1] = -m[1]*m[10]*m[15]+m[1]*m[11]*m[14]+m[9]*m[2]*m[15]-m[9]*m[3]*m[14]-m[13]*m[2]*m[11]+m[13]*m[3]*m[10]
    inv[5] = m[0]*m[10]*m[15]-m[0]*m[11]*m[14]-m[8]*m[2]*m[15]+m[8]*m[3]*m[14]+m[12]*m[2]*m[11]-m[12]*m[3]*m[10]
    inv[9] = -m[0]*m[9]*m[15]+m[0]*m[11]*m[13]+m[8]*m[1]*m[15]-m[8]*m[3]*m[13]-m[12]*m[1]*m[11]+m[12]*m[3]*m[9]
    inv[13] = m[0]*m[9]*m[14]-m[0]*m[10]*m[13]-m[8]*m[1]*m[14]+m[8]*m[2]*m[13]+m[12]*m[1]*m[10]-m[12]*m[2]*m[9]
    inv[2] = m[1]*m[6]*m[15]-m[1]*m[7]*m[14]-m[5]*m[2]*m[15]+m[5]*m[3]*m[14]+m[13]*m[2]*m[7]-m[13]*m[3]*m[6]
    inv[6] = -m[0]*m[6]*m[15]+m[0]*m[7]*m[14]+m[4]*m[2]*m[15]-m[4]*m[3]*m[14]-m[12]*m[2]*m[7]+m[12]*m[3]*m[6]
    inv[10] = m[0]*m[5]*m[15]-m[0]*m[7]*m[13]-m[4]*m[1]*m[15]+m[4]*m[3]*m[13]+m[12]*m[1]*m[7]-m[12]*m[3]*m[5]
    inv[14] = -m[0]*m[5]*m[14]+m[0]*m[6]*m[13]+m[4]*m[1]*m[14]-m[4]*m[2]*m[13]-m[12]*m[1]*m[6]+m[12]*m[2]*m[5]
    inv[3] = -m[1]*m[6]*m[11]+m[1]*m[7]*m[10]+m[5]*m[2]*m[11]-m[5]*m[3]*m[10]-m[9]*m[2]*m[7]+m[9]*m[3]*m[6]
    inv[7] = m[0]*m[6]*m[11]-m[0]*m[7]*m[10]-m[4]*m[2]*m[11]+m[4]*m[3]*m[10]+m[8]*m[2]*m[7]-m[8]*m[3]*m[6]
    inv[11] = -m[0]*m[5]*m[11]+m[0]*m[7]*m[9]+m[4]*m[1]*m[11]-m[4]*m[3]*m[9]-m[8]*m[1]*m[7]+m[8]*m[3]*m[5]
    inv[15] = m[0]*m[5]*m[10]-m[0]*m[6]*m[9]-m[4]*m[1]*m[10]+m[4]*m[2]*m[9]+m[8]*m[1]*m[6]-m[8]*m[2]*m[5]
    det = m[0]*inv[0]+m[1]*inv[4]+m[2]*inv[8]+m[3]*inv[12]
    if det == 0.0:
        return None
    det = 1.0 / det
    return [inv[i] * det for i in range(16)]

KBASIS = [0, 0, -1, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1]

def build_view(world):
    return mul4(KBASIS, invert4(world))

def build_cascade_splits(cn, cf, nc, lam):
    s = [0.0] * (nc + 1)
    s[0] = cn; s[nc] = cf
    ratio = cf / cn
    for i in range(1, nc):
        p = i / nc
        s[i] = lam * (cn * ratio ** p) + (1 - lam) * (cn + (cf - cn) * p)
    return s

def build_cascade_lvps(cam_world, hfov, aspect, cn, cf, light_world, nc, lam):
    splits = build_cascade_splits(cn, cf, nc, lam)
    light_view = build_view(light_world)
    cam_view = build_view(cam_world)
    vfov = 2.0 * math.atan(math.tan(hfov * 0.5) / aspect)
    out = []
    for c in range(nc):
        camvp = mul4(perspective(vfov, aspect, splits[c], splits[c + 1]), cam_view)
        inv = invert4(camvp)
        lo = [1e30, 1e30, 1e30]; hi = [-1e30, -1e30, -1e30]
        for ci in range(8):
            ndc = [1.0 if ci & 1 else -1.0, 1.0 if ci & 2 else -1.0, 1.0 if ci & 4 else 0.0, 1.0]
            w = mulvec4(inv, ndc); iw = 1.0 / w[3] if w[3] else 1.0
            lv = mulvec4(light_view, [w[0] * iw, w[1] * iw, w[2] * iw, 1.0])
            for k in range(3):
                lo[k] = min(lo[k], lv[k]); hi[k] = max(hi[k], lv[k])
        cx = (lo[0] + hi[0]) * 0.5; cy = (lo[1] + hi[1]) * 0.5
        h = max(0.5 * max(hi[0] - lo[0], hi[1] - lo[1]), 1e-4)
        depth = hi[2] - lo[2]; zN = -hi[2] - depth; zF = -lo[2] + 1e-3
        if zF <= zN:
            zF = zN + 1e-3
        out.append(mul4(ortho_offcenter(cx - h, cx + h, cy - h, cy + h, zN, zF), light_view))
    return splits, out

# ---------------------------------------------------------------------------
# Webots-convention look-at world matrix (local +X forward, +Z up, +Y left).
# ---------------------------------------------------------------------------
def _norm(v):
    l = math.sqrt(sum(c * c for c in v)); return [c / l for c in v]

def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def lookat_world(eye, target, world_up=(0, 0, 1)):
    x = _norm([target[i] - eye[i] for i in range(3)])  # +X forward
    y = _norm(_cross(world_up, x))                      # +Y left
    z = _cross(x, y)                                    # +Z up
    return [x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0, eye[0], eye[1], eye[2], 1]

# ---------------------------------------------------------------------------
# Scene geometry: large floor (z=0) + small elevated caster. pos3/norm3/uv2.
# ---------------------------------------------------------------------------
def quad(cx, cy, z, hx, hy):
    n = [0.0, 0.0, 1.0]
    p = [(cx-hx, cy-hy, z), (cx+hx, cy-hy, z), (cx+hx, cy+hy, z),
         (cx-hx, cy-hy, z), (cx+hx, cy+hy, z), (cx-hx, cy+hy, z)]
    uv = [(0, 0), (1, 0), (1, 1), (0, 0), (1, 1), (0, 1)]
    out = []
    for i in range(6):
        out += [p[i][0], p[i][1], p[i][2], n[0], n[1], n[2], uv[i][0], uv[i][1]]
    return out

def load_csm_shader():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(os.path.dirname(here))
    src = os.path.join(repo, "src", "omnisim", "render", "WbWgpuShaders.cpp")
    text = open(src, "r", encoding="utf-8").read()
    m = re.search(r'kSolidLitCsm\s*=\s*R"WGSL\((.*?)\)WGSL"', text, re.DOTALL)
    if not m:
        raise SystemExit(f"could not find kSolidLitCsm in {src}")
    return m.group(1)


def main():
    ZCAST = 3.0
    ALL = quad(0, 0, 0, 30, 30) + quad(0, 0, ZCAST, 2.0, 2.0)
    NC = 3
    SHADOW_RES = 1024
    W = H = 256
    HFOV, ASPECT, CN, CF = 1.0, W / H, 0.5, 60.0
    cam_world = lookat_world((0, -18, 14), (0, 4, 0))
    light_world = lookat_world((6, 6, 20), (0, 0, 0))
    splits, lvps = build_cascade_lvps(cam_world, HFOV, ASPECT, CN, CF, light_world, NC, 0.6)
    cam_view = build_view(cam_world)
    cam_vp = mul4(perspective(2.0 * math.atan(math.tan(HFOV * 0.5) / ASPECT), ASPECT, CN, CF), cam_view)
    light_dir = [light_world[0], light_world[1], light_world[2]]  # local +X = light travel dir

    adapter = wgpu.gpu.request_adapter_sync(power_preference="high-performance")
    device = adapter.request_device_sync()
    print(f"adapter: {device.adapter.info.get('device')} / {device.adapter.info.get('backend_type')}")
    print(f"splits: {[round(s, 2) for s in splits]}")

    vbuf = device.create_buffer_with_data(data=np.array(ALL, dtype=np.float32).tobytes(),
                                          usage=wgpu.BufferUsage.VERTEX)
    VBL = [{"array_stride": 32, "step_mode": wgpu.VertexStepMode.vertex, "attributes": [
        {"format": wgpu.VertexFormat.float32x3, "offset": 0, "shader_location": 0},
        {"format": wgpu.VertexFormat.float32x3, "offset": 12, "shader_location": 1},
        {"format": wgpu.VertexFormat.float32x2, "offset": 24, "shader_location": 2}]}]
    IDENT = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]

    # --- depth pass: clip.z into each R32Float array layer (mirrors kSolidClipDepthF32) ---
    DEPTH_WGSL = """
struct U { lightViewProj : mat4x4<f32>, model : mat4x4<f32> };
@group(0) @binding(0) var<uniform> u : U;
@vertex
fn vs_main(@location(0) p : vec3<f32>, @location(1) n : vec3<f32>, @location(2) uv : vec2<f32>) -> @builtin(position) vec4<f32> {
  return u.lightViewProj * (u.model * vec4<f32>(p, 1.0));
}
@fragment
fn fs_main(@builtin(position) pos : vec4<f32>) -> @location(0) vec4<f32> { return vec4<f32>(pos.z, 0.0, 0.0, 1.0); }
"""
    depth_mod = device.create_shader_module(code=DEPTH_WGSL)
    depth_bgl = device.create_bind_group_layout(entries=[
        {"binding": 0, "visibility": wgpu.ShaderStage.VERTEX,
         "buffer": {"type": wgpu.BufferBindingType.uniform, "min_binding_size": 128}}])
    depth_pipe = device.create_render_pipeline(
        layout=device.create_pipeline_layout(bind_group_layouts=[depth_bgl]),
        vertex={"module": depth_mod, "entry_point": "vs_main", "buffers": VBL},
        primitive={"topology": wgpu.PrimitiveTopology.triangle_list},
        depth_stencil={"format": wgpu.TextureFormat.depth24plus, "depth_write_enabled": True,
                       "depth_compare": wgpu.CompareFunction.less},
        fragment={"module": depth_mod, "entry_point": "fs_main",
                  "targets": [{"format": wgpu.TextureFormat.r32float}]})
    shadow_tex = device.create_texture(
        size=(SHADOW_RES, SHADOW_RES, NC), format=wgpu.TextureFormat.r32float,
        usage=wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.TEXTURE_BINDING)
    shadow_depth = device.create_texture(
        size=(SHADOW_RES, SHADOW_RES, 1), format=wgpu.TextureFormat.depth24plus,
        usage=wgpu.TextureUsage.RENDER_ATTACHMENT)
    for c in range(NC):
        ubuf = device.create_buffer_with_data(data=np.array(lvps[c] + IDENT, dtype=np.float32).tobytes(),
                                              usage=wgpu.BufferUsage.UNIFORM)
        bg = device.create_bind_group(layout=depth_bgl, entries=[
            {"binding": 0, "resource": {"buffer": ubuf, "offset": 0, "size": 128}}])
        view = shadow_tex.create_view(base_array_layer=c, array_layer_count=1,
                                      dimension=wgpu.TextureViewDimension.d2)
        enc = device.create_command_encoder()
        rp = enc.begin_render_pass(
            color_attachments=[{"view": view, "clear_value": (1.0, 0, 0, 1),
                                "load_op": wgpu.LoadOp.clear, "store_op": wgpu.StoreOp.store}],
            depth_stencil_attachment={"view": shadow_depth.create_view(), "depth_clear_value": 1.0,
                                      "depth_load_op": wgpu.LoadOp.clear, "depth_store_op": wgpu.StoreOp.store})
        rp.set_pipeline(depth_pipe); rp.set_bind_group(0, bg); rp.set_vertex_buffer(0, vbuf)
        rp.draw(len(ALL) // 8); rp.end()
        device.queue.submit([enc.finish()])

    # --- lit+shadow pass with the LIVE kSolidLitCsm ---
    csm_mod = device.create_shader_module(code=load_csm_shader())
    csm_bgl = device.create_bind_group_layout(entries=[
        {"binding": 0, "visibility": wgpu.ShaderStage.VERTEX | wgpu.ShaderStage.FRAGMENT,
         "buffer": {"type": wgpu.BufferBindingType.uniform, "min_binding_size": 448}},
        {"binding": 1, "visibility": wgpu.ShaderStage.FRAGMENT,
         "texture": {"sample_type": wgpu.TextureSampleType.unfilterable_float,
                     "view_dimension": wgpu.TextureViewDimension.d2_array}},
        {"binding": 2, "visibility": wgpu.ShaderStage.FRAGMENT,
         "sampler": {"type": wgpu.SamplerBindingType.non_filtering}}])
    color_fmt = wgpu.TextureFormat.rgba8unorm
    csm_pipe = device.create_render_pipeline(
        layout=device.create_pipeline_layout(bind_group_layouts=[csm_bgl]),
        vertex={"module": csm_mod, "entry_point": "vs_main", "buffers": VBL},
        primitive={"topology": wgpu.PrimitiveTopology.triangle_list},
        depth_stencil={"format": wgpu.TextureFormat.depth24plus, "depth_write_enabled": True,
                       "depth_compare": wgpu.CompareFunction.less},
        fragment={"module": csm_mod, "entry_point": "fs_main", "targets": [{"format": color_fmt}]})
    samp = device.create_sampler(mag_filter=wgpu.FilterMode.nearest, min_filter=wgpu.FilterMode.nearest)
    arr_view = shadow_tex.create_view(dimension=wgpu.TextureViewDimension.d2_array)

    def csm_uniform(strength):
        lvp_flat = []
        for c in range(4):
            lvp_flat += (lvps[c] if c < NC else IDENT)
        csplits = [splits[1], splits[2] if NC >= 2 else 1e9, splits[3] if NC >= 3 else 1e9, 1e9]
        return np.array(cam_vp + IDENT + lvp_flat + [0.8, 0.8, 0.8, 1.0]
                        + [light_dir[0], light_dir[1], light_dir[2], 0.25]
                        + csplits + [strength, 2e-3, float(NC), 0.0], dtype=np.float32).tobytes()

    def render(strength):
        ubuf = device.create_buffer_with_data(data=csm_uniform(strength), usage=wgpu.BufferUsage.UNIFORM)
        bg = device.create_bind_group(layout=csm_bgl, entries=[
            {"binding": 0, "resource": {"buffer": ubuf, "offset": 0, "size": 448}},
            {"binding": 1, "resource": arr_view},
            {"binding": 2, "resource": samp}])
        color = device.create_texture(size=(W, H, 1), format=color_fmt,
                                      usage=wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.COPY_SRC)
        cdepth = device.create_texture(size=(W, H, 1), format=wgpu.TextureFormat.depth24plus,
                                       usage=wgpu.TextureUsage.RENDER_ATTACHMENT)
        enc = device.create_command_encoder()
        rp = enc.begin_render_pass(
            color_attachments=[{"view": color.create_view(), "clear_value": (0, 0, 0, 1),
                                "load_op": wgpu.LoadOp.clear, "store_op": wgpu.StoreOp.store}],
            depth_stencil_attachment={"view": cdepth.create_view(), "depth_clear_value": 1.0,
                                      "depth_load_op": wgpu.LoadOp.clear, "depth_store_op": wgpu.StoreOp.store})
        rp.set_pipeline(csm_pipe); rp.set_bind_group(0, bg); rp.set_vertex_buffer(0, vbuf)
        rp.draw(len(ALL) // 8); rp.end()
        device.queue.submit([enc.finish()])
        data = device.queue.read_texture(
            {"texture": color, "mip_level": 0, "origin": (0, 0, 0)},
            {"offset": 0, "bytes_per_row": W * 4, "rows_per_image": H}, (W, H, 1))
        return np.frombuffer(data, dtype=np.uint8).reshape(H, W, 4)

    def project(wp):
        c = mulvec4(cam_vp, [wp[0], wp[1], wp[2], 1.0])
        return (int((c[0] / c[3] * 0.5 + 0.5) * W), int((0.5 - c[1] / c[3] * 0.5) * H)) if c[3] > 0 else None

    def lum(img, pt):
        x = max(0, min(W - 1, pt[0])); y = max(0, min(H - 1, pt[1]))
        px = img[y, x]; return float(px[0]) + float(px[1]) + float(px[2]), tuple(int(c) for c in px[:3])

    img_lit = render(0.0)
    img_shadow = render(0.8)
    shadow_pt = project((0.0, 0.0, 0.0))     # floor directly under the caster -> expect shadow
    lit_pt = project((10.0, 0.0, 0.0))       # floor to the side, same depth -> expect lit
    print(f"shadow_pt={shadow_pt} lit_pt={lit_pt}")

    fails = 0
    def check(ok, msg):
        nonlocal fails
        print(f"  [{'PASS' if ok else 'FAIL'}] {msg}")
        if not ok:
            fails += 1

    sh_off, sh_off_px = lum(img_lit, shadow_pt)
    sh_on, sh_on_px = lum(img_shadow, shadow_pt)
    li_off, _ = lum(img_lit, lit_pt)
    li_on, _ = lum(img_shadow, lit_pt)
    print(f"  shadow_pt: lit={sh_off_px} sum={sh_off:.0f} -> shadowed={sh_on_px} sum={sh_on:.0f}")
    print(f"  lit_pt:    lit sum={li_off:.0f} -> shadowed sum={li_on:.0f}")

    check(np.array_equal(img_lit, render(0.0)), "strength 0 deterministic")
    check(sh_off > 0, "floor-under-caster is lit (geometry visible) when shadows off")
    check(sh_on < sh_off * 0.85, "floor-under-caster DARKENS with shadows on (caster casts)")
    check(0 <= lit_pt[1] < H and abs(li_on - li_off) < max(8.0, li_off * 0.05),
          "floor-away-from-caster stays lit (no false shadow)")

    cw = mulvec4(cam_vp, [0.0, 0.0, 0.0, 1.0]); view_depth = cw[3]
    sel = 0
    if view_depth > splits[1]: sel = 1
    if view_depth > splits[2]: sel = 2
    if view_depth > splits[3]: sel = 3
    sel = min(sel, NC - 1)
    print(f"  shadow_pt view-depth(clip.w)={view_depth:.2f} -> cascade {sel} of {NC}")
    check(sel >= 1, f"shadow routes through a non-zero cascade (cascade {sel}) — multi-cascade selection exercised")

    print(f"\n{'ALL CHECKS PASSED' if not fails else 'SOME CHECKS FAILED'} ({fails} failures)")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
