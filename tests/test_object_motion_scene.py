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
"""End-to-end motion collection, MSAA pass, temporal resolve and unchanged sensors."""
from pathlib import Path
import json
import os
import re
import subprocess
import tempfile
import unittest
from omnisim.paths import resolve_omnisim_binary
from scripts.dev.render_ab import render, summarize_gpu_profile

REPO = Path(__file__).resolve().parents[1]


class ObjectMotionScene(unittest.TestCase):
    def test_live_cloth_uses_deformation_motion(self):
        import numpy as np
        from PIL import Image
        binary = os.environ.get('OMNISIM_BINARY') or resolve_omnisim_binary()
        if not binary: self.skipTest('OmniSim binary is not built')
        source = REPO/'projects/samples/demos/worlds/rendering/camera_cloth_wgpu_smoke.omniworld'
        with tempfile.TemporaryDirectory(prefix='omnisim-cloth-motion-') as directory:
            output = Path(directory)
            fd, name = tempfile.mkstemp(prefix='.motion_cloth_', suffix='.omniworld', dir=source.parent)
            os.close(fd)
            world = Path(name)
            try:
                text = source.read_text().replace('WorldInfo {', 'WorldInfo {\n FPS 8', 1)
                # The source is a sensor test. Use that proven sensor pose for
                # the main viewport too; its authored viewport looks away.
                text = re.sub(r'Viewpoint\s*\{[^}]*\}',
                              'Viewpoint { position -1.6 0 .85 orientation 0 0 1 0 fieldOfView .9 }', text, count=1)
                world.write_text(text, encoding='utf-8')
                env = dict(OMNISIM_BINARY=str(binary), OMNILIGHT='0', OMNISIM_WGPU_TAA='1',
                           OMNISIM_WGPU_TAA_VALIDATION='1', OMNISIM_WGPU_MOTION_VECTORS='1',
                           OMNISIM_WGPU_CAMFX='0', OMNISIM_WGPU_AUTOEXP='0',
                           OMNISIM_WGPU_GPU_TIMING=str(output/'render.gpu'),
                           OMNISIM_CAM_SAMPLE_DIR=str(output), OMNISIM_CAM_SAMPLE_STEPS='8,64')
                if os.name == 'nt':
                    runtime = REPO/'msys64/mingw64/bin/newton-runtime'
                    env['PATH'] = str(runtime)+os.pathsep+str(runtime.parent)+os.pathsep+os.environ.get('PATH','')
                row = render(world, output/'cloth.png', 32, env, output/'engine.log', output/'render.cpu', 35)
                self.assertTrue(row['dumped'], row)
                self.assertEqual(row['errors'], 0, row)
                with Image.open(output/'sample_0008.ppm') as im: a = np.asarray(im, dtype='int16')
                with Image.open(output/'sample_0064.ppm') as im: b = np.asarray(im, dtype='int16')
                self.assertGreater(int((np.abs(a-b)>5).sum()), 50)
                profiles = summarize_gpu_profile((output/'render.gpu').read_text(), warmup_frames=0)
                available = [p for p in profiles if p['status']=='available']
                if available:
                    self.assertTrue(any(p.get('motionUs',{}).get('p50',0)>0 for p in available), profiles)
            finally:
                world.unlink(missing_ok=True)

    def test_motion_and_geometry_edits_preserve_sensor_images(self):
        import numpy as np
        from PIL import Image
        binary = os.environ.get('OMNISIM_BINARY') or resolve_omnisim_binary()
        if not binary: self.skipTest('OmniSim binary is not built')
        with tempfile.TemporaryDirectory(prefix='omnisim-motion-') as directory:
            output = Path(directory)
            # Optional evidence destination; normal tests clean up all images.
            if os.environ.get('OMNISIM_TEST_MOTION_OUTPUT'):
                output = Path(os.environ['OMNISIM_TEST_MOTION_OUTPUT']).resolve()
                output.mkdir(parents=True, exist_ok=True)
            checker = (np.indices((128,128)).sum(axis=0)//2 % 2 * 200 + 30).astype('uint8')
            Image.fromarray(checker).convert('RGB').save(output/'detail.png')
            fd, name = tempfile.mkstemp(prefix='.object_motion_', suffix='.omniworld', dir=REPO/'tests/rendering/worlds')
            os.close(fd)
            world = Path(name)
            try:
                for arm in (0,1):
                    dest = output/str(arm); dest.mkdir(exist_ok=True)
                    names = [f'{frame:02}{suffix}.png' for frame in range(12) for suffix in ('','-sensor')]
                    names += ['render.gpu','gpu-errors.log','states.json','error.txt']
                    for filename in names: (dest/filename).unlink(missing_ok=True)
                    world.write_text('''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 32 FPS 15 }
DEF VIEW Viewpoint { position 0 0 2.3 orientation 0 1 0 .35 }
Background { skyColor [ .08 .08 .08 ] }
DirectionalLight { intensity 3 direction .5 .2 -1 ambientIntensity .5 }
Solid { translation 4 0 -.1 children [ Shape {
  appearance PBRAppearance { baseColor .45 .45 .45 roughness 1 metalness 0 }
  geometry Box { size 12 10 .2 }
} ] }
DEF BLOCK Solid { translation 3 -.45 1 children [ Shape {
  appearance PBRAppearance { baseColorMap ImageTexture { url [ %s ] } roughness 1 metalness 0 }
  geometry DEF BOX Box { size .25 .9 1.2 }
} ] }
Robot { supervisor TRUE controller "object_motion_probe" controllerArgs [ %s ] children [
  Camera { name "probe" translation 0 0 2.3 rotation 0 1 0 .35 width 320 height 200 }
] }
''' % (json.dumps((output/'detail.png').as_posix()), json.dumps(dest.as_posix())), encoding='utf-8')
                    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNILIGHT='0', OMNISIM_WGPU_TAA='1',
                               OMNISIM_WGPU_TAA_VALIDATION='1', OMNISIM_WGPU_MOTION_VECTORS=str(arm),
                               OMNISIM_WGPU_CAMFX='0', OMNISIM_WGPU_AUTOEXP='0',
                               OMNISIM_WGPU_GPU_TIMING=str(dest/'render.gpu'),
                               OMNISIM_WGPU_ERRLOG=str(dest/'gpu-errors.log'), OMNISIM_LOG_PATH=str(dest/'engine.log'))
                    if os.name == 'nt':
                        runtime = REPO/'msys64/mingw64/bin/newton-runtime'
                        env['PATH'] = str(runtime)+os.pathsep+str(runtime.parent)+os.pathsep+env.get('PATH','')
                    with (dest/'stdout.log').open('wb') as stdout:
                        process = subprocess.Popen([str(binary), str(world), '--mode=realtime', '--stdout', '--stderr'],
                                                   cwd=REPO, env=env, stdout=stdout, stderr=subprocess.STDOUT)
                        try: code = process.wait(timeout=50)
                        finally:
                            if process.poll() is None:
                                if os.name == 'nt': subprocess.run(['taskkill','/F','/T','/PID',str(process.pid)], capture_output=True)
                                else: process.kill()
                                process.wait(timeout=10)
                    log = (dest/'stdout.log').read_text(errors='replace')
                    self.assertEqual(code, 0, log[-4000:])
                    self.assertNotIn('Validation Error', log)
                    self.assertNotIn('wgpu error', log.lower())
                    self.assertFalse((dest/'error.txt').exists())
                def image(arm, name):
                    with Image.open(output/str(arm)/name) as im: return np.asarray(im.convert('RGB'),dtype='int16')
                for frame in range(12):
                    np.testing.assert_array_equal(image(0,f'{frame:02}-sensor.png'), image(1,f'{frame:02}-sensor.png'))
                self.assertGreater(int((np.abs(image(1,'00.png')-image(1,'11.png')) > 10).sum()), 10000)
                self.assertEqual(json.loads((output/'0/states.json').read_text()),json.loads((output/'1/states.json').read_text()))
                profiles = summarize_gpu_profile((output/'1/render.gpu').read_text(),warmup_frames=0)
                available = [p for p in profiles if p['status']=='available']
                if available:
                    self.assertTrue(any(p.get('motionUs',{}).get('p50',0)>0 for p in available), profiles)
            finally:
                world.unlink(missing_ok=True)


if __name__ == '__main__': unittest.main()
