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

"""OmniSim Launcher — supervisor controller for the in-sim demo navigator.

Pairs with `resources/projects/plugins/robot_windows/omnilink_launcher/` (the
side-panel UI) and `demos.json` (the curated demo catalogue). When the user
clicks "Launch" on a demo card, the side panel sends `load:<repo-relative
path>` to this controller, which resolves the path against the repo root and
calls `Supervisor.worldLoad()` to switch worlds.

Wire protocol with the side panel:

    panel  -> controller        controller -> panel
    ------------------------    ---------------------------------
    "ready"                     "manifest:<json>"   demo catalogue
    "load:<repo-rel-path>"      "loading:<path>"    optimistic ack
                                "error:<msg>"       resolution failed
                                "status:<text>"     advisory text
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from omnisim import Supervisor  # type: ignore[import-not-found]


CONTROLLER_DIR = Path(__file__).resolve().parent
# controllers/omnilink_launcher  ->  projects/samples/demos/controllers
#                                ->  projects/samples/demos
#                                ->  projects/samples
#                                ->  projects
#                                ->  <repo root>
REPO_ROOT = CONTROLLER_DIR.parents[4]
MANIFEST = CONTROLLER_DIR / "demos.json"


def _load_manifest() -> dict:
    try:
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        sys.stderr.write(f"[launcher] failed to read {MANIFEST}: {e}\n")
        return {"categories": []}
    return _drop_absent_demos(manifest)


def _drop_absent_demos(manifest: dict) -> dict:
    """Hide cards whose world is not present in this checkout.

    The catalogue is authored against the full private tree, but a robot
    package held from public distribution takes its worlds with it (see
    scripts/release/publish_deny.txt). Without this filter those cards still
    render and only fail on click, with "unknown or missing world" -- a
    catalogue advertising demos the checkout cannot run. Filtering at manifest
    load keeps one catalogue honest in every checkout, and a category that
    empties out drops away with its cards.
    """
    kept_categories = []
    dropped = 0
    for category in manifest.get("categories", []):
        demos = [d for d in category.get("demos", []) if _resolve_world(d.get("world", ""))]
        dropped += len(category.get("demos", [])) - len(demos)
        if demos:
            kept_categories.append({**category, "demos": demos})
    if dropped:
        sys.stderr.write(f"[launcher] {dropped} demo card(s) hidden - world not in this checkout\n")
    return {**manifest, "categories": kept_categories}


def _resolve_world(rel_path: str) -> Path | None:
    """Resolve a repo-relative world path. Returns None if it escapes the repo
    or doesn't exist."""
    candidate = (REPO_ROOT / rel_path).resolve()
    try:
        candidate.relative_to(REPO_ROOT)
    except ValueError:
        return None
    if not candidate.is_file() or candidate.suffix != ".wbt":
        return None
    return candidate


def _send(robot: Supervisor, tag: str, payload: str = "") -> None:
    msg = tag if not payload else f"{tag}:{payload}"
    robot.wwiSendText(msg)


def main() -> None:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    manifest = _load_manifest()
    manifest_json = json.dumps(manifest, separators=(",", ":"))

    # Push manifest eagerly — the panel may already be open if the user
    # re-launches without closing the side window.
    _send(robot, "manifest", manifest_json)
    _send(robot, "status", "ready — pick a demo")

    while robot.step(timestep) != -1:
        msg = robot.wwiReceiveText()
        while msg:
            if msg == "ready":
                _send(robot, "manifest", manifest_json)
            elif msg.startswith("load:"):
                rel = msg[len("load:"):]
                world = _resolve_world(rel)
                if world is None:
                    _send(robot, "error", f"unknown or missing world: {rel}")
                else:
                    _send(robot, "loading", str(world))
                    # worldLoad terminates this controller; no code after this
                    # runs in the launcher world.
                    robot.worldLoad(str(world))
                    return
            else:
                _send(robot, "status", f"unknown message: {msg}")
            msg = robot.wwiReceiveText()


if __name__ == "__main__":
    main()
