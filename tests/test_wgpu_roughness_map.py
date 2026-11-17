# Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
"""Engine regression: a PBR roughness texture overrides the scalar roughness.

Run with python -m unittest discover -s tests -p test_wgpu_roughness_map.py.
OMNISIM_BINARY can select a candidate binary or the pre-fix negative control.
No stored screenshot is the oracle: a white map must match scalar roughness=1,
independently of the scalar authored alongside that map. A smooth reference
must differ, so a missing/flat render cannot satisfy the test.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from omnisim.paths import resolve_omnisim_binary


REPO = Path(__file__).resolve().parents[1]


class RoughnessMapRendering(unittest.TestCase):
    def test_map_overrides_scalar(self):
        binary = os.environ.get("OMNISIM_BINARY") or resolve_omnisim_binary()
        if not binary:
            self.skipTest("OmniSim binary is not built")
        with tempfile.TemporaryDirectory(prefix="omnisim-roughness-") as temporary:
            out = Path(temporary) / "result.json"
            fd, world_name = tempfile.mkstemp(
                prefix=".roughness_map_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds"
            )
            os.close(fd)
            world = Path(world_name)
            world.write_text('''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 16 }
Viewpoint { position 0 0 0 orientation 0 0 1 0 }
Background { skyColor [ 0.06 0.08 0.1 ] }
DirectionalLight { direction 1 -0.2 -0.2 intensity 2 }
Robot {
  supervisor TRUE
  controller "roughness_map_probe"
  controllerArgs [ %s ]
  children [ Camera { name "camera" width 96 height 96 fieldOfView 0.8 } ]
}
Solid {
  translation 2 0 0
  children [ Shape {
    appearance DEF SURFACE PBRAppearance {
      baseColor 0.3 0.3 0.3
      metalness 0
      roughness 0
      roughnessMap ImageTexture { url [ "textures/roughness_white.ppm" ] }
    }
    geometry Sphere { radius 0.55 subdivision 3 }
  } ]
}
''' % json.dumps(out.as_posix()), encoding="utf-8")
            env = dict(os.environ, OMNISIM_HOME=str(REPO),
                       OMNISIM_LOG_PATH=str(Path(temporary) / "engine.log"),
                       OMNISIM_NO_WINDOW="1", OMNISIM_WGPU_TAA="0",
                       # This probe edits materials between samples; force a fresh
                       # collection instead of testing sensor draw-cache invalidation.
                       OMNISIM_WGPU_SENSOR_DRAW_CACHE="0")
            # The bundled interpreter is sufficient for this stdlib-only controller.
            if os.name == "nt":
                runtime = REPO / "msys64/mingw64/bin/newton-runtime"
                env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
            try:
                proc = subprocess.Popen(
                    [str(binary), str(world), "--batch", "--mode=fast", "--stdout", "--stderr"],
                    cwd=REPO, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
                )
                try:
                    stdout, _ = proc.communicate(timeout=60)
                except subprocess.TimeoutExpired:
                    if os.name == "nt":
                        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                                       capture_output=True, check=False)
                    else:
                        proc.kill()
                    stdout, _ = proc.communicate(timeout=10)
                    self.fail("material probe timed out: " + stdout.decode(errors="replace")[-4000:])
                self.assertTrue(out.exists(), stdout.decode(errors="replace")[-4000:])
                result = json.loads(out.read_text(encoding="utf-8"))
                self.assertNotIn("error", result)
                self.assertEqual(proc.returncode, 0, result)
                self.assertGreater(result["image_range"], 10, result)
                self.assertLessEqual(result["map_ignores_scalar"]["max"], 2, result)
                self.assertLessEqual(result["map_matches_scalar_reference"]["max"], 2, result)
                self.assertGreater(result["scalar_still_matters"]["mean"], 0.2, result)
                print("roughness-map render:", binary, json.dumps(result, sort_keys=True))
            finally:
                world.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
