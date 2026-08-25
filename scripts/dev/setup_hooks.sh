#!/usr/bin/env bash
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

# One-time per-clone setup: route git hooks through the versioned .githooks/
# directory so post-merge / post-checkout fire after pulls and branch switches.
set -e
ROOT="$(git rev-parse --show-toplevel)"
git -C "$ROOT" config core.hooksPath .githooks
chmod +x "$ROOT/.githooks/"* 2>/dev/null || true
echo "Hooks enabled (core.hooksPath=.githooks)."
# Teach blame to skip the tree-wide mechanical sweeps (renames, header flips) so
# it lands on whoever actually wrote each line. GitHub reads the file directly.
git -C "$ROOT" config blame.ignoreRevsFile .git-blame-ignore-revs
echo "Blame ignore-list enabled (blame.ignoreRevsFile=.git-blame-ignore-revs)."
echo "Preview cleanups any time with:"
echo "  python scripts/dev/clean_orphans.py --dry-run"
