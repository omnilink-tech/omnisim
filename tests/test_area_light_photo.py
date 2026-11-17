"""Area-light geometry reaches the photo snapshot and raster proxies are excluded."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from omnisim.paths import resolve_omnisim_binary

REPO = Path(__file__).resolve().parents[1]


class AreaLightPhoto(unittest.TestCase):
    def test_area_emission_and_clear_solid_glass(self):
        from PIL import Image, ImageStat
        binary = os.environ.get('OMNISIM_BINARY') or resolve_omnisim_binary()
        if not binary:
            self.skipTest('OmniSim binary is not built')
        with tempfile.TemporaryDirectory(prefix='omnisim-area-photo-') as directory:
            root = Path(directory)
            levels = []
            for on in ('TRUE', 'FALSE'):
                world, image_path = root / f'{on}.omniworld', root / f'{on}.png'
                world.write_text('''#OMNISIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimAreaLight.proto"
WorldInfo { basicTimeStep 16 }
Viewpoint { position 0 0 1.5 orientation 0 1 0 0.3 }
Background { skyColor [ 0 0 0 ] }
DirectionalLight { intensity 0 direction 0 0 -1 }
OmniSimAreaLight { translation 2 0 2 size 2 2 intensity 8 on %s }
Solid { translation 3 0 -0.1 children [ Shape {
  appearance PBRAppearance { baseColor 0.7 0.7 0.7 roughness 1 metalness 0 }
  geometry Box { size 8 8 0.2 }
} ] }
Solid { translation 3 1 0.5 children [ Shape {
  appearance PBRAppearance { transparency 1 refraction TRUE indexOfRefraction 1.5 metalness 0 }
  geometry Box { size 0.5 0.5 1 }
} ] }
''' % on, encoding='utf-8')
                env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_PHOTO_OUTPUT=str(image_path),
                           OMNISIM_PHOTO_WIDTH='96', OMNISIM_PHOTO_HEIGHT='64', OMNISIM_PHOTO_SAMPLES='16',
                           OMNISIM_PHOTO_SECONDS='10', OMNISIM_PHOTO_DENOISE='0',
                           OMNISIM_LOG_PATH=str(root / f'{on}.log'))
                if os.name == 'nt':
                    runtime = REPO / 'msys64/mingw64/bin/newton-runtime'
                    env['PATH'] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get('PATH', '')
                with (root / f'{on}.stdout').open('wb') as stdout:
                    proc = subprocess.Popen([str(binary), str(world), '--mode=realtime', '--stdout', '--stderr'],
                                            cwd=REPO, env=env, stdout=stdout, stderr=subprocess.STDOUT)
                    try:
                        code = proc.wait(timeout=45)
                    finally:
                        if proc.poll() is None:
                            if os.name == 'nt':
                                subprocess.run(['taskkill', '/F', '/T', '/PID', str(proc.pid)], capture_output=True)
                            else:
                                proc.kill()
                            proc.wait(timeout=10)
                self.assertEqual(code, 0, (root / f'{on}.stdout').read_text(errors='replace')[-4000:])
                report = json.loads(Path(str(image_path) + '.json').read_text())
                self.assertEqual(report['triangles'], 26, report)
                self.assertEqual(report['solid_glass_materials'], 1, report)
                self.assertEqual(report['explicit_lights'], 0, report)
                self.assertEqual(report['emissive_triangles'], 2 if on == 'TRUE' else 0, report)
                with Image.open(image_path) as image:
                    levels.append(sum(ImageStat.Stat(image.convert('RGB')).mean) / 3)
            self.assertGreater(levels[0], 30, levels)
            self.assertLess(levels[1], 2, levels)
            print('Area light on/off image means:', levels)


if __name__ == '__main__':
    unittest.main()
