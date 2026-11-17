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

"""An 'omnisim://' asset missing from the install falls back to this release on GitHub.

The defect this pins (2026-10-09, found running a ghostloop backend on the
v9.2.1 installer): an installed release ships no PROTO files -- the packager
skips every protos/ folder, as upstream Webots does -- and fetches them from
GitHub for its own tag. The worlds it ships already name them that way. But a
world a user or an agent writes with 'omnisim://' PROTOs, the form every doc
recommends, could not load on an install: OmUrl's fallback pointed at the local
install path (changed 2026-04-11), and the existence check had already rewritten
the URL in place, so the fallback never saw the 'omnisim://' prefix either. The
load logged "changed by fallback mechanism" and then "Skipped PROTO ... not
available at <install>/projects/...".

Probe: a world (outside the checkout) that names a PROTO no checkout has. The
engine must say it is fetching it from this release on GitHub, at the tagged
path. The fetch itself is allowed to fail (the file does not exist anywhere and
the test may run offline); what is pinned is where the engine looks.

    python -m pytest tests/test_omnisim_url_fallback_to_release.py -v

OMNISIM_BINARY selects a scratch build. One engine launch, a few seconds.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
MISSING = "projects/objects/floors/protos/OmniSimUrlFallbackProbe.proto"

WORLD = f"""#VRML_SIM R2025a utf8
EXTERNPROTO "omnisim://{MISSING}"
WorldInfo {{ basicTimeStep 8 }}
Viewpoint {{ orientation 0 0 1 0 position -3 0 1 }}
OmniSimUrlFallbackProbe {{ }}
Solid {{ translation 0 0 1 boundingObject Sphere {{ radius 0.05 }} physics Physics {{ }} }}
"""


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


def _version():
    text = (REPO / "omnisim" / "__init__.py").read_text(encoding="utf-8")
    return re.search(r'__version__\s*=\s*"([^"]+)"', text).group(1)


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def test_missing_omnisim_asset_is_fetched_from_this_release(tmp_path):
    assert not (REPO / MISSING).exists(), "the probe PROTO must not exist in the checkout"
    world = tmp_path / "probe.omniworld"
    world.write_text(WORLD, encoding="utf-8")
    log = tmp_path / "engine.log"
    # The trajectory probe ends the run once the falling Solid passes 300 ms of sim time.
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(tmp_path / "traj.tsv"), OMNISIM_PROBE_TRAJ_MS="300")
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=60, capture_output=True)
    except subprocess.TimeoutExpired:
        pass  # the world has nothing to run; only the load-time log line matters
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    remote = f"https://raw.githubusercontent.com/omnilink-tech/omnisim/v{_version()}/{MISSING}"
    assert remote in text, (
        "a missing omnisim:// PROTO must be fetched from this release's tree on GitHub (%s); log tail:\n%s"
        % (remote, text[-1500:]))
    assert "changed by fallback mechanism" not in text, "the fallback is the normal path on an install, not an error"
