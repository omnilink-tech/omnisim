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
"""Look-check the film set without running the shift: the Husky world plus
scene.vrml(), no robot controller, a few frames from each camera.

    python agents/omnilink_demos/warehouse_shift/film/smoke.py <out_dir>
"""
import os
import shutil
import subprocess
import sys
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import scene  # noqa: E402

from omnisim.paths import linux_runtime_env, resolve_omnisim_binary  # noqa: E402

out = Path(sys.argv[1])
shutil.rmtree(out, ignore_errors=True)
out.mkdir(parents=True)
src = Path("projects/samples/demos/worlds/chat/omnilink_husky.omniworld")
text = src.read_text(encoding="utf-8").replace('controller "omnilink_mobile_bridge"', 'controller "<none>"')
world = src.with_name(f".film_smoke_{uuid.uuid4().hex[:8]}.omniworld")
world.write_text(text + scene.vrml(500), encoding="utf-8")
binary = resolve_omnisim_binary()
env = linux_runtime_env(".", None)
env.update(OMNISIM_FILM_DIR=str(out), OMNISIM_NO_WINDOW="1", OMNISIM_LOG_PATH=str(out / "engine.log"))
env["PATH"] = os.pathsep.join([str(Path(binary).parent / "newton-runtime"), str(Path(binary).parent), env.get("PATH", "")])
try:
    p = subprocess.Popen([binary, "--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout", "--stderr",
                          str(world)], env=env, stdout=open(out / "stdout.log", "w"), stderr=subprocess.STDOUT)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90 and sum(1 for _ in out.rglob("*.jpg")) < 6:
        time.sleep(1)
    p.kill()
    p.wait()
finally:
    world.unlink(missing_ok=True)
print(sorted(str(f) for f in out.rglob("*.jpg"))[-2:])
