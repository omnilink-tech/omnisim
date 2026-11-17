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
"""Compare real rendered shadows with culling/reuse independently enabled."""
import json
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest

from omnisim.paths import resolve_omnisim_binary
from scripts.dev.render_ab import summarize_profile, summarize_gpu_profile

REPO = Path(__file__).resolve().parents[1]

class LocalShadowCache(unittest.TestCase):
    def test_edits_match_uncached_unculled_reference(self):
        import numpy as np
        from PIL import Image
        binary = os.environ.get('OMNISIM_BINARY') or resolve_omnisim_binary()
        if not binary:
            self.skipTest('OmniSim binary is not built')
        with tempfile.TemporaryDirectory(prefix='omnisim-shadow-cache-') as directory:
            output = Path(directory)
            fd, name = tempfile.mkstemp(prefix='.shadow_cache_', suffix='.omniworld', dir=REPO / 'tests/rendering/worlds')
            os.close(fd)
            world = Path(name)
            profiles = []
            try:
                for arm, (cull, cache) in enumerate(((0, 0), (1, 0), (1, 1))):
                    dest = output / str(arm)
                    dest.mkdir()
                    world.write_text('''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 32 FPS 30 }
DEF VIEW Viewpoint { position 0 0 3 orientation 0 1 0 0.45 }
Background { skyColor [ .01 .01 .01 ] }
DirectionalLight { intensity 0 direction 0 0 -1 }
DEF LAMP PointLight { location 3 -1.5 3 intensity 15 attenuation 0 0 1 radius 20 castShadows TRUE }
Solid { translation 4 0 -.1 children [ Shape {
  appearance PBRAppearance { baseColor .65 .65 .65 roughness 1 metalness 0 }
  geometry Box { size 12 10 .2 }
} ] }
DEF BLOCK Solid { translation 3 0 .5 children [ DEF BLOCK_SHAPE Shape {
  appearance DEF BLOCK_MAT PBRAppearance { baseColor .4 .4 .4 roughness 1 metalness 0 }
  geometry DEF BLOCK_BOX Box { size 1 1 1 }
} ] }
Robot { supervisor TRUE controller "local_shadow_cache_probe" controllerArgs [ %s ] children [
  Camera { name "probe" translation 0 0 3 rotation 0 1 0 .45 width 320 height 200 }
] }
''' % json.dumps(dest.as_posix()), encoding='utf-8')
                    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNILIGHT='0', OMNISIM_WGPU_TAA='0',
                               OMNISIM_WGPU_CAMFX='0', OMNISIM_WGPU_AUTOEXP='0',
                               OMNISIM_WGPU_LOCAL_SHADOW_CULL=str(cull), OMNISIM_WGPU_LOCAL_SHADOW_CACHE=str(cache),
                               OMNISIM_WGPU_REPORT=str(dest / 'render.report'), OMNISIM_WGPU_REPORT_EVERY='1',
                               OMNISIM_LOG_PATH=str(dest / 'engine.log'))
                    # Also exercise asynchronous GPU timing with both main-view
                    # and synchronous sensor readback; neither may change pixels.
                    env['OMNISIM_WGPU_GPU_TIMING'] = str(dest / 'render.gpu') if arm == 2 else '0'
                    if os.name == 'nt':
                        runtime = REPO / 'msys64/mingw64/bin/newton-runtime'
                        env['PATH'] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get('PATH', '')
                    with (dest / 'stdout.log').open('wb') as stdout:
                        proc = subprocess.Popen([str(binary), str(world), '--mode=realtime', '--stdout', '--stderr'],
                                                cwd=REPO, env=env, stdout=stdout, stderr=subprocess.STDOUT)
                        try:
                            code = proc.wait(timeout=60)
                        finally:
                            if proc.poll() is None:
                                if os.name == 'nt':
                                    subprocess.run(['taskkill', '/F', '/T', '/PID', str(proc.pid)], capture_output=True)
                                else:
                                    proc.kill()
                                proc.wait(timeout=10)
                    self.assertEqual(code, 0, (dest / 'stdout.log').read_text(errors='replace')[-4000:])
                    self.assertFalse((dest / 'error.txt').exists())
                    profile = summarize_profile((dest / 'render.report').read_text())
                    self.assertIsNotNone(profile)
                    profiles.append(profile)
                    if arm == 2:
                        gpu = summarize_gpu_profile((dest / 'render.gpu').read_text(), warmup_frames=0)
                        available = [row for row in gpu if row['status'] == 'available']
                        self.assertGreaterEqual(len(gpu), 2)
                        if available:
                            self.assertGreaterEqual(len(available), 2)
                            self.assertTrue(any([320, 200] in row['dimensions'] for row in available))
                            self.assertTrue(all(row['gpuSpanUs']['p50'] > 0 for row in available))
                        else:
                            self.assertTrue(all(row['status'] == 'unavailable' for row in gpu))
                names = ('initial', 'unchanged', 'camera', 'lamp', 'caster', 'geometry', 'no_cast',
                         'transparent', 'disabled', 'enabled', 'inserted', 'grown1', 'grown2', 'grown3',
                         'removed', 'radius')
                originals = {}
                sensor_names = names
                for name in names + tuple(name + '-sensor' for name in sensor_names):
                    with Image.open(output / '0' / (name + '.png')) as im:
                        reference = np.asarray(im.convert('RGB'), dtype=np.int16)
                    originals[name] = reference
                    for arm in (1, 2):
                        with Image.open(output / str(arm) / (name + '.png')) as im:
                            actual = np.asarray(im.convert('RGB'), dtype=np.int16)
                        self.assertEqual(actual.shape, reference.shape)
                        self.assertLessEqual(int(np.abs(actual-reference).max()), 1, (name, arm))
                for before, after in (('camera', 'lamp'), ('lamp', 'caster'), ('caster', 'geometry'),
                                      ('geometry', 'no_cast'), ('disabled', 'enabled'), ('enabled', 'inserted')):
                    self.assertGreater(int((np.abs(originals[before]-originals[after]) > 10).sum()), 100, (before, after))
                self.assertGreater(int((np.abs(originals['caster-sensor']-originals['geometry-sensor']) > 10).sum()), 100)
                self.assertGreater(int((np.abs(originals['enabled-sensor']-originals['inserted-sensor']) > 10).sum()), 100)
                for before, after in (('inserted', 'grown1'), ('grown1', 'grown2'), ('grown2', 'grown3')):
                    self.assertGreater(int((np.abs(originals[before]-originals[after]) > 10).sum()), 100)
                    self.assertGreater(int((np.abs(originals[before+'-sensor']-originals[after+'-sensor']) > 10).sum()), 100)
                for arm in (0, 1, 2):
                    state = json.loads((output / str(arm) / 'enabled.json').read_text())
                    self.assertEqual(state['transparency'], 0.0)
                self.assertEqual(profiles[0]['local_cache_hit_fraction'], 0)
                self.assertLess(profiles[1]['localDraws']['p50'], profiles[0]['localDraws']['p50'])
                self.assertGreater(profiles[2]['local_cache_hit_fraction'], .6)
                self.assertEqual(profiles[2]['localDraws']['p50'], 0)
                print('16 shadow states matched reference; baseline/culled/cached profiles:', json.dumps(profiles))
            except Exception:
                shutil.copytree(output, REPO / '.local-runs/local-shadow-failure', dirs_exist_ok=True)
                raise
            finally:
                world.unlink(missing_ok=True)
                world.with_name(f'.{world.stem}.omniperspective').unlink(missing_ok=True)

if __name__ == '__main__':
    unittest.main()
