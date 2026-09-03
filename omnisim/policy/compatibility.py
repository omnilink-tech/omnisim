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

"""Pinned deploy-environment baselines for flagship G1 sequences."""

from __future__ import annotations

import hashlib
import json
import os


# Re-pinned 2026-09-02. The only change since the 2026-07-25 pin is the world
# extension migration (6f495ca01, 2026-08-15): every sequence's WALK_WORLD
# went `g1_box_grasp.wbt` -> `g1_box_grasp.omniworld`. Attributed exactly --
# reversing `.omniworld` -> `.wbt` in the assembled values reproduces all four
# previous digests bit-for-bit, so key sets and every other value are
# unchanged. Counts are unchanged for the same reason.
G1_ENV_BASELINES = {
    "box_delivery": (92, "56e74c7d53d1e0bd64f76fa5263dbbf2f513fd48531b945f3deefc2f6e8dffba"),
    "box_delivery_classic": (30, "aa9dde41592cb7a488b1dc0a3a49df976bd6854b8fe604495daf2835d09c0c57"),
    "turn_solo": (35, "7b053881f1d9670fdf0f1bd323b18208c1389ae49edc2f7043e1588d7c71c73a"),
    "walk_turn_walk": (40, "41ac094466896879834a1d18b3dc672d0dad1e6568e4efc5384fb68f26e7fcfd"),
}


# The assembled env contains absolute paths built from the checkout root --
# including scratch OUTPUT files like BATON_MODE_FILE. Hashing them verbatim
# made the digest depend on where the repo lives, so this pin failed in every
# clone except the one that computed it (a fresh public clone reported two G1
# sequences "changed" while the dev tree passed). The pin exists to catch a
# change to the DEPLOY CONTRACT, so normalize the checkout root away first:
# the key set and every non-path value are still hashed exactly.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))


def _normalize(value: str) -> str:
    """Replace this checkout's root with a stable placeholder, either slash style."""
    for root in (_REPO_ROOT, _REPO_ROOT.replace("\\", "/")):
        if root and root in value:
            value = value.replace(root, "<REPO>")
    return value


def env_digest(env: dict[str, str]) -> str:
    normalized = {k: _normalize(v) for k, v in env.items()}
    payload = json.dumps(normalized, sort_keys=True,
                         separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def verify_g1_envs(assembler) -> list[str]:
    issues: list[str] = []
    for name, (expected_count, expected_hash) in G1_ENV_BASELINES.items():
        env = assembler(name)
        actual = (len(env), env_digest(env))
        if actual != (expected_count, expected_hash):
            issues.append(
                f"{name}: deploy environment changed: expected "
                f"{expected_count}/{expected_hash}, got {actual[0]}/{actual[1]}")
    return issues
