# Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
"""Real main-view shadow response to a moving lamp and an unchanged blocker."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from omnisim.paths import resolve_omnisim_binary

REPO = Path(__file__).resolve().parents[1]


class LocalLightShadows(unittest.TestCase):
    def test_shadow_follows_moving_light(self):
        from PIL import Image
        import numpy as np
        binary = os.environ.get('OMNISIM_BINARY') or resolve_omnisim_binary()
        if not binary:
            self.skipTest('OmniSim binary is not built')
        with tempfile.TemporaryDirectory(prefix='omnisim-local-shadow-') as directory:
            output = Path(directory)
            fd, name = tempfile.mkstemp(prefix='.local_shadow_', suffix='.omniworld', dir=REPO / 'tests/rendering/worlds')
            os.close(fd)
            world = Path(name)
            world.write_text('''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 32 FPS 30 }
Viewpoint { position 0 0 3 orientation 0 1 0 0.45 }
Background { skyColor [ 0.01 0.01 0.01 ] }
DirectionalLight { intensity 0 direction 0 0 -1 }
DEF LAMP PointLight { location 3 -1.5 3 intensity 15 attenuation 0 0 1 castShadows FALSE }
Solid { translation 4 0 -0.1 children [ Shape {
  appearance PBRAppearance { baseColor 0.65 0.65 0.65 roughness 1 metalness 0 }
  geometry Box { size 12 10 0.2 }
} ] }
Solid { translation 3 0 0.5 children [ Shape {
  appearance PBRAppearance { baseColor 0.4 0.4 0.4 roughness 1 metalness 0 }
  geometry Box { size 1 1 1 }
} ] }
Robot { supervisor TRUE controller "local_light_probe" controllerArgs [ %s ] }
''' % json.dumps(output.as_posix()), encoding='utf-8')
            env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNILIGHT='0', OMNISIM_WGPU_TAA='0',
                       OMNISIM_LOG_PATH=str(output / 'engine.log'))
            if os.name == 'nt':
                runtime = REPO / 'msys64/mingw64/bin/newton-runtime'
                env['PATH'] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get('PATH', '')
            try:
                with (output / 'stdout.log').open('wb') as stdout:
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
                log = (output / 'stdout.log').read_text(errors='replace')
                self.assertEqual(code, 0, log[-4000:])
                centres, masks = [], []
                for side in (0, 1):
                    with Image.open(output / f'{side}_0.png') as image:
                        lit = np.asarray(image.convert('RGB'), dtype=float).mean(axis=2)
                    with Image.open(output / f'{side}_1.png') as image:
                        shadowed = np.asarray(image.convert('RGB'), dtype=float).mean(axis=2)
                    loss = lit - shadowed
                    mask = loss > 15
                    self.assertGreater(float(mask.mean()), 0.002, {'side': side, 'max_darkening': float(loss.max())})
                    centres.append(float(np.nonzero(mask)[1].mean()) / mask.shape[1])
                    masks.append(mask)
                self.assertGreater(abs(centres[0] - centres[1]), 0.02, centres)
                self.assertGreater(float(np.logical_xor(*masks).mean()), 0.003)
                print('Moving-light shadow centres:', centres)
            finally:
                world.unlink(missing_ok=True)
                world.with_name(f'.{world.stem}.omniperspective').unlink(missing_ok=True)


if __name__ == '__main__':
    unittest.main()
