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
"""Execute the live WGSL resolve on a GPU, with controlled color/depth histories."""
from pathlib import Path
import re
import unittest
import pytest

pytestmark = pytest.mark.engine  # GPU lane, never part of engine-free unit checks.


class TemporalValidationGPU(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import numpy as np
        import wgpu
        cls.np, cls.wgpu = np, wgpu
        cls.device = wgpu.gpu.request_adapter_sync(power_preference='low-power').request_device_sync()
        source = (Path(__file__).resolve().parents[2] / 'src/omnisim/render/OmWgpuShaders.cpp').read_text()
        shader = re.search(r'const char \*kTaaMvResolve = R"WGSL\((.*?)\)WGSL";', source, re.S)[1]
        module = cls.device.create_shader_module(code=shader)
        cls.pipeline = cls.device.create_render_pipeline(
            layout='auto', vertex={'module': module, 'entry_point': 'vs_main'},
            fragment={'module': module, 'entry_point': 'fs_main',
                      'targets': [{'format': 'rgba8unorm'}, {'format': 'r32float'}]})

    def resolve(self, validation=True, old_depth=.5, depth=.5, valid=True, color_delta=.2, shift=0,
                motion_shift=None, motion_depth=.5, motion_valid=1, history_shift=0,
                sample_depths=None, motion_sample=None):
        np, wgpu, device = self.np, self.wgpu, self.device
        size = (16, 16, 1)
        # A feature-rich neighborhood prevents the color clamp masking stale history.
        cur = np.zeros((16, 16, 4), dtype=np.uint8)
        cur[:, :, :3] = (np.indices((16, 16)).sum(axis=0) % 2 * 200)[:, :, None]
        cur[:, :, 3] = 255
        history = cur.copy()
        history[:, :, :3] = np.clip(cur[:, :, :3].astype(float) + color_delta*255, 0, 255).astype('uint8')
        history = np.roll(history, history_shift, axis=1)
        textures = []
        def texture(fmt, samples=1):
            usage = wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.TEXTURE_BINDING
            if samples == 1: usage |= wgpu.TextureUsage.COPY_SRC | wgpu.TextureUsage.COPY_DST
            t = device.create_texture(size=size, format=fmt, usage=usage, sample_count=samples)
            textures.append(t)
            return t
        current, previous = texture('rgba8unorm'), texture('rgba8unorm')
        z, hz = texture('depth32float', 4), texture('r32float')
        motion = texture('rgba16float', 4)
        color, depth_out = texture('rgba8unorm'), texture('r32float')
        for t, data in ((current, cur), (previous, history), (hz, np.full((16,16), old_depth, dtype='float32'))):
            device.queue.write_texture({'texture': t}, data, {'bytes_per_row': 64}, size)
        ident = np.eye(4, dtype='float32').flatten()
        prev = ident.copy()
        prev[12] = shift
        uniform = np.concatenate([ident, prev, np.array([motion_shift is not None, .9, validation, valid], dtype='float32')])
        ub = device.create_buffer_with_data(data=uniform, usage=wgpu.BufferUsage.UNIFORM)
        sampler = device.create_sampler(min_filter='linear', mag_filter='linear')
        bg = device.create_bind_group(layout=self.pipeline.get_bind_group_layout(0), entries=[
            {'binding': 0, 'resource': {'buffer': ub, 'size': 144}},
            {'binding': 1, 'resource': current.create_view()},
            {'binding': 2, 'resource': previous.create_view()},
            {'binding': 3, 'resource': z.create_view()},
            {'binding': 4, 'resource': sampler},
            {'binding': 5, 'resource': hz.create_view()},
            {'binding': 6, 'resource': motion.create_view()}])
        encoder = device.create_command_encoder()
        clear = encoder.begin_render_pass(color_attachments=[], depth_stencil_attachment={
            'view': z.create_view(), 'depth_clear_value': depth, 'depth_load_op': 'clear',
            'depth_store_op': 'store'})
        clear.end()
        if sample_depths is not None:
            module = device.create_shader_module(code='''
@vertex fn vs(@builtin(vertex_index) i:u32) -> @builtin(position) vec4f {
  var p = array<vec2f,3>(vec2f(-1,-1),vec2f(3,-1),vec2f(-1,3)); return vec4f(p[i],0,1);
}
@fragment fn fs(@builtin(sample_index) sample:u32) -> @builtin(frag_depth) f32 {
  let depths = array<f32,4>(%s); return depths[sample];
}''' % ','.join(str(float(d)) for d in sample_depths))
            pipeline = device.create_render_pipeline(layout='auto',
                vertex={'module': module, 'entry_point': 'vs'}, multisample={'count': 4},
                fragment={'module': module, 'entry_point': 'fs', 'targets': []},
                depth_stencil={'format': 'depth32float', 'depth_write_enabled': True, 'depth_compare': 'always'})
            render = encoder.begin_render_pass(color_attachments=[], depth_stencil_attachment={
                'view': z.create_view(), 'depth_load_op': 'load', 'depth_store_op': 'store'})
            render.set_pipeline(pipeline); render.draw(3); render.end()
        if motion_shift is not None:
            validity = str(float(motion_valid)) if motion_sample is None else f'select(-1.0,1.0,sample=={motion_sample}u)'
            module = device.create_shader_module(code='''
@vertex fn vs(@builtin(vertex_index) i:u32) -> @builtin(position) vec4f {
  var p = array<vec2f,3>(vec2f(-1,-1),vec2f(3,-1),vec2f(-1,3));
  return vec4f(p[i],0,1);
}
@fragment fn fs(@builtin(position) p:vec4f, @builtin(sample_index) sample:u32) -> @location(0) vec4f {
  return vec4f((floor(p.xy)+vec2f(0.5))/16.0 + vec2f(%s,0), %s, %s);
}''' % (float(motion_shift), float(motion_depth), validity))
            pipeline = device.create_render_pipeline(layout='auto',
                vertex={'module': module, 'entry_point': 'vs'}, multisample={'count': 4},
                fragment={'module': module, 'entry_point': 'fs', 'targets': [{'format': 'rgba16float'}]})
            render = encoder.begin_render_pass(color_attachments=[{
                'view': motion.create_view(), 'clear_value': (0,0,0,0), 'load_op': 'clear', 'store_op': 'store'}])
            render.set_pipeline(pipeline)
            render.draw(3)
            render.end()
        render = encoder.begin_render_pass(color_attachments=[{
            'view': t.create_view(), 'resolve_target': None, 'clear_value': (0,0,0,0),
            'load_op': 'clear', 'store_op': 'store'} for t in (color, depth_out)])
        render.set_pipeline(self.pipeline)
        render.set_bind_group(0, bg)
        render.draw(3)
        render.end()
        device.queue.submit([encoder.finish()])
        image = np.frombuffer(device.queue.read_texture({'texture': color}, {'bytes_per_row': 64}, size),
                              dtype='uint8').reshape(16,16,4).copy()
        saved_depth = np.frombuffer(device.queue.read_texture({'texture': depth_out}, {'bytes_per_row': 64}, size),
                                    dtype='float32').reshape(16,16).copy()
        for t in textures: t.destroy()
        ub.destroy()
        return image, cur, saved_depth

    def test_disocclusion_rejects_old_surface(self):
        before, cur, _ = self.resolve(validation=False, old_depth=.3)
        after, _, z = self.resolve(old_depth=.3)
        self.assertGreater(int(self.np.abs(before.astype(int)-cur).sum()), 1000)
        self.np.testing.assert_array_equal(after, cur)
        self.np.testing.assert_allclose(z, .5)

    def test_coplanar_color_change_reduces_trails(self):
        before, cur, _ = self.resolve(validation=False)
        after, _, _ = self.resolve()
        error_before = self.np.abs(before.astype(int)-cur).sum()
        error_after = self.np.abs(after.astype(int)-cur).sum()
        self.assertLess(error_after, error_before*.65)

    def test_stable_detail_still_accumulates(self):
        result, cur, _ = self.resolve(color_delta=.02)
        delta = result[:, :, :3].astype(int)-cur[:, :, :3]
        # Bright checks already equal the neighborhood maximum and get clamped;
        # dark checks must retain the small, stable history contribution.
        self.assertGreater(float(delta[cur[:, :, 0] == 0].mean()), 3)
        self.assertLessEqual(int(delta.max()), 5)

    def test_bilinear_taps_do_not_leak_across_depth_edges(self):
        depths = self.np.full((16,16), .5, dtype='float32')
        depths[:, 8:] = .3
        before, _, _ = self.resolve(validation=False, old_depth=depths, color_delta=0, shift=1/16)
        after, _, _ = self.resolve(old_depth=depths, color_delta=0, shift=1/16)
        # Reprojection lands halfway across the silhouette. Only the black tap
        # belongs to this surface; the bright tap is a different depth.
        self.assertGreater(int(before[5,7,0]), 80)
        self.assertEqual(int(after[5,7,0]), 0)

    def test_first_frame_sky_and_camera_cut(self):
        for args in ({'valid': False}, {'depth': 0}, {'shift': 4}):
            result, cur, _ = self.resolve(**args)
            self.np.testing.assert_array_equal(result, cur)

    def test_moving_detail_follows_object_instead_of_camera(self):
        before, cur, _ = self.resolve(history_shift=1, color_delta=.02)
        after, _, _ = self.resolve(history_shift=1, color_delta=.02, motion_shift=1/16)
        roi = self.np.s_[2:14, 2:14, :3]
        self.assertLess(float(self.np.abs(after[roi].astype(int)-cur[roi]).mean()), 3)
        self.assertGreater(float(self.np.abs(before[roi].astype(int)-cur[roi]).mean()), 30)

    def test_object_motion_uses_previous_surface_depth(self):
        # The object moved toward the camera; checking today's depth against
        # yesterday's image would throw away valid detail from the same surface.
        result, cur, _ = self.resolve(old_depth=.25, color_delta=.02, motion_shift=0, motion_depth=.25)
        self.assertGreater(int(self.np.abs(result.astype(int)-cur).sum()), 1000)

    def test_new_geometry_and_invalid_motion_reject_history(self):
        for args in ({'motion_valid': -1}, {'motion_shift': 2}, {'motion_depth': .3}):
            result, cur, _ = self.resolve(**dict({'motion_shift': 0}, **args))
            self.np.testing.assert_array_equal(result, cur)

    def test_motion_comes_from_the_nearest_msaa_sample(self):
        args = dict(sample_depths=(.25,.25,.5,.25), motion_shift=0, color_delta=.02)
        result, cur, z = self.resolve(**args, motion_sample=2)
        self.np.testing.assert_allclose(z, .5)
        self.assertGreater(int(self.np.abs(result.astype(int)-cur).sum()), 1000)
        rejected, cur, _ = self.resolve(**args, motion_sample=0)
        self.np.testing.assert_array_equal(rejected, cur)


if __name__ == '__main__':
    unittest.main()
