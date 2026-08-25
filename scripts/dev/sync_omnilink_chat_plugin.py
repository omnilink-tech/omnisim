#!/usr/bin/env python3
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

"""Keep the shipped and demo-local OmniLink chat robot-window assets identical."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CANONICAL = REPO_ROOT / "resources" / "projects" / "plugins" / "robot_windows" / "omnilink_chat"
DEMO_COPY = REPO_ROOT / "projects" / "samples" / "demos" / "plugins" / "robot_windows" / "omnilink_chat"
ASSETS = ("omnilink_chat.html", "omnilink_chat.css", "omnilink_chat.js")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="Copy canonical resources into the demo project.")
    args = parser.parse_args(argv)
    drift = []
    for name in ASSETS:
        source = CANONICAL / name
        target = DEMO_COPY / name
        if source.read_bytes() != target.read_bytes():
            drift.append(name)
            if args.write:
                shutil.copyfile(source, target)
    if drift and not args.write:
        print("OmniLink chat plugin drift: " + ", ".join(drift))
        print("Run: python scripts/dev/sync_omnilink_chat_plugin.py --write")
        return 1
    if drift:
        print("Synchronized: " + ", ".join(drift))
    else:
        print("OmniLink chat plugin copies are synchronized.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
