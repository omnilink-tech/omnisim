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

"""The demo-local robot window must match the canonical shipped plugin."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CANONICAL = REPO_ROOT / "resources" / "projects" / "plugins" / "robot_windows" / "omnilink_chat"
DEMO_COPY = REPO_ROOT / "projects" / "samples" / "demos" / "plugins" / "robot_windows" / "omnilink_chat"


def test_omnilink_chat_plugin_is_synchronized():
    for name in ("omnilink_chat.html", "omnilink_chat.css", "omnilink_chat.js"):
        assert (CANONICAL / name).read_bytes() == (DEMO_COPY / name).read_bytes(), (
            f"{name} drifted; run scripts/dev/sync_omnilink_chat_plugin.py --write"
        )
