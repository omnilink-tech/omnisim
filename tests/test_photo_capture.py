# Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
"""Native photo capture: real scene, actual PNG, bounded work, process exits."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from omnisim.paths import resolve_omnisim_binary

REPO = Path(__file__).resolve().parents[1]


class PhotoCapture(unittest.TestCase):
    def test_photo_writes_image_report_and_exits(self):
        binary = os.environ.get("OMNISIM_BINARY") or resolve_omnisim_binary()
        if not binary:
            self.skipTest("OmniSim binary is not built")
        from PIL import Image, ImageStat
        with tempfile.TemporaryDirectory(prefix="omnisim-photo-") as directory:
            root = Path(directory)
            world = root / "photo.omniworld"
            world.write_text('''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 16 }
Viewpoint { position 0 0 1.3 orientation 0 0 1 0 }
Background { skyColor [ 0.06 0.08 0.1 ] }
DirectionalLight { direction 1 -0.2 -0.6 intensity 3 }
Solid { translation 3 0 1.3 children [ Shape {
  appearance PBRAppearance { baseColor 0.8 0.08 0.04 roughness 0.5 metalness 0 }
  geometry Sphere { radius 0.65 subdivision 3 }
} ] }
''', encoding="utf-8")
            output = root / "photo.png"
            env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_PHOTO_OUTPUT=str(output),
                       OMNISIM_PHOTO_WIDTH="96", OMNISIM_PHOTO_HEIGHT="64",
                       OMNISIM_PHOTO_SAMPLES="4", OMNISIM_PHOTO_SECONDS="10",
                       OMNISIM_LOG_PATH=str(root / "engine.log"))
            if os.name == "nt":
                runtime = REPO / "msys64/mingw64/bin/newton-runtime"
                env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
            with (root / "stdout.log").open("wb") as stdout:
                proc = subprocess.Popen([str(binary), str(world), "--mode=realtime", "--stdout", "--stderr"],
                                        cwd=REPO, env=env, stdout=stdout, stderr=subprocess.STDOUT)
                try:
                    code = proc.wait(timeout=50)
                finally:
                    if proc.poll() is None:
                        if os.name == "nt":
                            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                                           capture_output=True, check=False)
                        else:
                            proc.kill()
                        proc.wait(timeout=10)
            log = (root / "stdout.log").read_text(errors="replace")
            self.assertEqual(code, 0, log[-4000:])
            self.assertTrue(output.exists(), log[-4000:])
            report = json.loads(Path(str(output) + ".json").read_text())
            self.assertEqual(report["completed_samples"], 4)
            self.assertFalse(report["time_limited"])
            self.assertGreater(report["triangles"], 0)
            with Image.open(output) as image:
                self.assertEqual(image.size, (96, 64))
                rgb = image.convert("RGB")
                self.assertGreater(ImageStat.Stat(rgb).stddev[0], 8)
                r, g, b = rgb.getpixel((48, 32))
                self.assertGreater(r, g + 15, (r, g, b))
            print("native photo:", report)


if __name__ == "__main__":
    unittest.main()
