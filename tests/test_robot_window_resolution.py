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

"""Every tracked world's HTML robot window resolves the way the ENGINE looks for it.

Found 2026-10-20 by the XGO Mini2S clip lane on public v9.2.6: a world saved outside
the install could not open the OmniLink chat panel ("The 'omnilink_chat' robot window
library has not been found."). `OmRobot::windowFile()` looked for
`<plugins>/robot_windows/<key>/<key>.html` in the world's OWN project (and PROTO
projects) only, and `omnilink_chat` lived in `projects/samples/demos/plugins/`. The
same gap silently broke two tracked worlds: `omnilink_launcher` (its window was only
in resources/, so the controller was handed a phantom `omnilink_launcher.dll`) and
`omnitug500_courier` (a project with no chat copy).

The engine now falls back to the install's shared plugin folders
(`OmControllerPlugin::defaultHtmlWindowFile`: resources/projects/... and
projects/default/...). This mirrors that search, statically, so a window that only
one project can see fails here instead of in front of a user.
"""

from __future__ import annotations

import pathlib
import re
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
SHARED_ROOTS = (ROOT / "resources" / "projects", ROOT / "projects" / "default")
_WINDOW = re.compile(r'^\s*window\s+"([^"]+)"', re.M)


def _tracked_worlds():
    out = subprocess.run(["git", "ls-files", "*.omniworld", "*.wbt"], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout.split()
    # Frozen evidence copies and the owner's workspaces are not worlds anyone loads.
    return [ROOT / f for f in out if "/evidence/" not in f and not f.startswith("social/")]


def _project_of(world: pathlib.Path) -> pathlib.Path:
    """The engine's project for a world: the folder holding its `worlds/` folder."""
    for parent in world.parents:
        if parent.name == "worlds":
            return parent.parent
    return world.parent


def _shared_window(key: str) -> pathlib.Path | None:
    """`<plugins>/robot_windows/<key>/<key>.html` under any `plugins` folder of the
    shared roots -- the folders OmControllerPlugin's search walks."""
    for root in SHARED_ROOTS:
        if not root.is_dir():
            continue
        for plugins in root.rglob("plugins"):
            f = plugins / "robot_windows" / key / f"{key}.html"
            if f.is_file():
                return f
    return None


def test_every_world_window_resolves_like_the_engine():
    missing = []
    for world in _tracked_worlds():
        text = world.read_text(encoding="utf-8", errors="replace")
        for key in _WINDOW.findall(text):
            if key.startswith("<"):
                continue
            own = _project_of(world) / "plugins" / "robot_windows" / key / f"{key}.html"
            if not own.is_file() and _shared_window(key) is None:
                missing.append(f"{world.relative_to(ROOT)}: window {key!r}")
    assert not missing, "robot windows no world can open:\n" + "\n".join(missing)


def test_the_chat_panel_is_shared_not_per_project():
    """One copy, where every world finds it. A second per-project copy would drift
    from it, and would win over it for that project's worlds."""
    shared = _shared_window("omnilink_chat")
    assert shared is not None and shared.parent.parent.parent.parent == ROOT / "resources" / "projects"
    copies = [p for p in ROOT.glob("projects/**/plugins/robot_windows/omnilink_chat")
              if "/evidence/" not in p.as_posix()]
    assert not copies, copies


def test_the_engine_falls_back_to_the_shared_window():
    """The C++ half is engine code (no unit lane runs it); pin that the fallback
    and the phantom-library guard are both still wired."""
    robot = (ROOT / "src/omnisim/nodes/OmRobot.cpp").read_text(encoding="utf-8")
    plugin = (ROOT / "src/omnisim/core/OmControllerPlugin.cpp").read_text(encoding="utf-8")
    assert "return OmControllerPlugin::defaultHtmlWindowFile(window(), extension);" in robot
    assert "!QFile::exists(absoluteFilePath) && !windowFile().isEmpty()" in robot
    assert "QString OmControllerPlugin::defaultHtmlWindowFile(" in plugin
    assert "isLocatedInInstallationDirectory(pluginPath, true)" in plugin


def test_every_shared_html_window_ships_in_the_package():
    """v9.2.6 shipped resources/.../robot_windows with only `generic` and
    `generic_window`: `omnilink_launcher` was never in the package list, so the
    installed launcher gallery had no page. A shared window must be listed."""
    listed = (ROOT / "scripts/packaging/files_core.txt").read_text(encoding="utf-8").splitlines()
    shared = ROOT / "resources/projects/plugins/robot_windows"
    missing = []
    for d in sorted(p for p in shared.iterdir() if p.is_dir()):
        if not (d / f"{d.name}.html").is_file():
            continue                       # a library window, built and listed by name
        rel = f"resources/projects/plugins/robot_windows/{d.name}/"
        for ext in ("html", "js", "css"):
            if any(d.glob(f"*.{ext}")) and f"{rel}*.{ext}" not in listed:
                missing.append(f"{rel}*.{ext}")
    assert not missing, missing
