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
"""Rasterize the production object-motion shader with controlled rigid/deformed triangles."""
from pathlib import Path
import re
import unittest
import pytest

pytestmark = pytest.mark.engine


class ObjectMotionGPU(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import numpy as np
        import wgpu
        cls.np, cls.wgpu = np, wgpu
        cls.device = wgpu.gpu.request_adapter_sync(power_preference='low-power').request_device_sync()
        source = (Path(__file__).resolve().parents[2] / 'src/omnisim/render/OmWgpuShaders.cpp').read_text()
        shader = re.search(r'const char \*kObjectMotion = R"WGSL\((.*?)\)WGSL";', source, re.S)[1]
        cls.module = cls.device.create_shader_module(code=shader)

    def raster(self, deformation=False, alpha=255, valid=True, scene_depth=.5, transparent=False):
        np, wgpu, device = self.np, self.wgpu, self.device
        resources = []
        def buffer(data, usage):
            b = device.create_buffer_with_data(data=data, usage=usage)
            resources.append(b)
            return b
        def texture(fmt, samples=1):
            usage = wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.TEXTURE_BINDING
            if samples == 1: usage |= wgpu.TextureUsage.COPY_SRC | wgpu.TextureUsage.COPY_DST
            t = device.create_texture(size=(16,16,1), format=fmt, sample_count=samples, usage=usage)
            resources.append(t)
            return t
        identity = np.eye(4, dtype='float32').flatten()
        previous = identity.copy()
        previous[12] = .25
        ub = buffer(np.concatenate([identity, identity]), wgpu.BufferUsage.UNIFORM)
        slot = np.concatenate([identity, previous, np.array([1,0,0,1, 0,0,0,0,
                               1 if valid else -1, deformation, 0, transparent], dtype='float32')])
        slots = buffer(slot, wgpu.BufferUsage.STORAGE)
        # Deliberately non-sequential indices. Previous positions must use the
        # indexed vertex ID, not the order triangles issue their vertices.
        vertices = np.array([[-1,-1,.5, 0,0,1, 0,0], [3,-1,.5, 0,0,1, 1,0],
                             [-1,3,.5, 0,0,1, 0,1]], dtype='float32')
        old = np.ones((3,4), dtype='float32')
        old[:, :3] = vertices[:, :3]
        if deformation:
            old[:,0] += .125
            old[:,2] = .25
        positions = buffer(old, wgpu.BufferUsage.STORAGE)
        vb = buffer(vertices, wgpu.BufferUsage.VERTEX)
        ib = buffer(np.array([2,0,1], dtype='uint32'), wgpu.BufferUsage.INDEX)
        albedo = texture('rgba8unorm')
        texels = np.full((16,16,4), 255, dtype='uint8'); texels[:,:,3] = alpha
        device.queue.write_texture({'texture': albedo}, texels, {'bytes_per_row': 64}, (16,16,1))
        motion, resolved, depth = texture('rgba16float',4), texture('rgba16float'), texture('depth32float',4)
        pipeline = device.create_render_pipeline(layout='auto',
            vertex={'module': self.module, 'entry_point': 'vs_main', 'buffers': [{
                'array_stride': 32, 'attributes': [
                    {'format': 'float32x3', 'offset': 0, 'shader_location': 0},
                    {'format': 'float32x2', 'offset': 24, 'shader_location': 2}]}]},
            fragment={'module': self.module, 'entry_point': 'fs_main', 'targets': [{'format': 'rgba16float'}]},
            depth_stencil={'format': 'depth32float', 'depth_write_enabled': False,
                           'depth_compare': 'greater-equal' if transparent else 'equal'},
            multisample={'count': 4})
        bg = device.create_bind_group(layout=pipeline.get_bind_group_layout(0), entries=[
            {'binding': 0, 'resource': {'buffer': ub}}, {'binding': 1, 'resource': {'buffer': slots}},
            {'binding': 2, 'resource': {'buffer': positions}},
            {'binding': 3, 'resource': albedo.create_view()},
            {'binding': 4, 'resource': device.create_sampler()}])
        encoder = device.create_command_encoder()
        clear = encoder.begin_render_pass(color_attachments=[], depth_stencil_attachment={
            'view': depth.create_view(), 'depth_clear_value': scene_depth,
            'depth_load_op': 'clear', 'depth_store_op': 'store'})
        clear.end()
        render = encoder.begin_render_pass(color_attachments=[{
            'view': motion.create_view(), 'resolve_target': resolved.create_view(),
            'clear_value': (0,0,0,0), 'load_op': 'clear', 'store_op': 'store'}],
            depth_stencil_attachment={'view': depth.create_view(), 'depth_read_only': True})
        render.set_pipeline(pipeline); render.set_bind_group(0, bg)
        render.set_vertex_buffer(0, vb); render.set_index_buffer(ib, 'uint32')
        render.draw_indexed(3); render.end()
        device.queue.submit([encoder.finish()])
        image = np.frombuffer(device.queue.read_texture({'texture': resolved}, {'bytes_per_row': 128}, (16,16,1)),
                              dtype='float16').reshape(16,16,4).astype('float32')
        for resource in resources: resource.destroy()
        return image

    def test_rigid_transform_reprojects_previous_position(self):
        result = self.raster()
        self.np.testing.assert_allclose(result[7,7], [7.5/16+.125, 7.5/16, .5, 1], atol=.001)

    def test_deformation_uses_previous_indexed_vertices_and_depth(self):
        result = self.raster(deformation=True)
        self.np.testing.assert_allclose(result[7,7], [7.5/16+.1875, 7.5/16, .25, 1], atol=.001)

    def test_occluded_and_alpha_cutout_fragments_write_nothing(self):
        for args in ({'scene_depth': .75}, {'alpha': 0}):
            self.np.testing.assert_array_equal(self.raster(**args), 0)

    def test_new_and_transparent_surfaces_reject_history(self):
        for args in ({'valid': False}, {'valid': False, 'transparent': True, 'scene_depth': .25}):
            self.np.testing.assert_allclose(self.raster(**args)[7,7], [0,0,0,-1])


if __name__ == '__main__':
    unittest.main()
