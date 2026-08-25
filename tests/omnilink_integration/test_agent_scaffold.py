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

"""Offline tests for ``python -m omnisim agent new``."""

from __future__ import annotations

import ast
import json

from omnisim.agent.cli import main


def test_mobile_scaffold_is_complete_and_parseable(tmp_path):
    assert main([
        "new", "inspection_rover", "--robot-class", "mobile",
        "--output-root", str(tmp_path),
    ]) == 0
    target = tmp_path / "inspection_rover"
    assert {path.name for path in target.iterdir()} == {
        "README.md", "inspection_rover_agent.py", "omnilink.json", "profile.json"
    }
    manifest = json.loads((target / "omnilink.json").read_text(encoding="utf-8"))
    assert manifest["world"].endswith("omnilink_husky.omniworld")
    assert manifest["bridge_url_env"] == "INSPECTION_ROVER_BRIDGE_URL"
    source = (target / "inspection_rover_agent.py").read_text(encoding="utf-8")
    ast.parse(source)
    assert "OmniLinkAgentRunner(" in source
    assert '"drive_forward"' in source


def test_scaffold_refuses_overwrite(tmp_path):
    args = ["new", "safe_bot", "--robot-class", "arm", "--output-root", str(tmp_path)]
    assert main(args) == 0
    try:
        main(args)
    except SystemExit as exc:
        assert exc.code == 2
    else:
        raise AssertionError("scaffold overwrote an existing agent")
