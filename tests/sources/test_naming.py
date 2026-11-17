#!/usr/bin/env python
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

"""Naming guard for the Webots -> OmniSim rebrand.

Asserts that the live counts emitted by [rename_audit.py](../../scripts/dev/rename_audit.py)
do not exceed the baseline snapshot at [tests/baselines/rename_audit_baseline.json](../baselines/rename_audit_baseline.json).

Any deliberate Webots-name reduction (path rename, header forwarder
removal, etc.) re-snapshots the JSON file with the lower count; this
test then makes sure the next change does not regress (re-add
Webots-named symbols).

The test is intentionally one-directional: counts can drop without
failure (no need to re-snapshot every commit), but a single new
WEBOTS_HOME read or webots:// URL in a non-historical file flips it red.

Naming policy lives in [AGENTS.md §0](../../AGENTS.md): the product is
OmniSim; Webots-named symbols are reserved for upstream attribution,
Wb*-prefixed engine internals, and on-disk paths that still literally
contain the string.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
AUDIT_SCRIPT = REPO_ROOT / "scripts" / "dev" / "rename_audit.py"
BASELINE = REPO_ROOT / "tests" / "baselines" / "rename_audit_baseline.json"


class TestNamingDoesNotRegress(unittest.TestCase):
    """Webots-named symbol counts must not grow above the recorded baseline."""

    @classmethod
    def setUpClass(cls):
        if not BASELINE.exists():
            raise unittest.SkipTest(
                f"Baseline not present: {BASELINE.relative_to(REPO_ROOT)}. "
                "Regenerate with: "
                "py -3 scripts/dev/rename_audit.py --json > tests/baselines/rename_audit_baseline.json"
            )
        with BASELINE.open("r", encoding="utf-8") as fh:
            cls.baseline = json.load(fh)

        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        proc = subprocess.run(
            [sys.executable, str(AUDIT_SCRIPT), "--json"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            env=env,
            check=True,
        )
        cls.current = json.loads(proc.stdout)

    def test_no_category_regressed(self):
        regressions = []
        for name, base in self.baseline.items():
            cur = self.current.get(name)
            self.assertIsNotNone(cur, msg=f"category {name!r} disappeared from audit output")
            if cur["count"] > base["count"]:
                regressions.append(
                    f"  {name}: baseline={base['count']}  current={cur['count']}"
                    f"  (+{cur['count'] - base['count']})"
                )
        if regressions:
            self.fail(
                "Rebrand regression -- one or more Webots-named categories grew "
                "above the baseline recorded in tests/baselines/rename_audit_baseline.json:\n"
                + "\n".join(regressions)
                + "\n\nIf this growth is intentional (e.g. you imported a new upstream "
                "subsystem) re-snapshot with:\n"
                "  py -3 scripts/dev/rename_audit.py --json > tests/baselines/rename_audit_baseline.json"
            )


if __name__ == "__main__":
    unittest.main()
