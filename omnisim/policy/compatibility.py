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


G1_ENV_BASELINES = {
    "box_delivery": (92, "40c24c1c5bf4ced0e29a3707ae5ca19280ba5352e533320784553903b0ed5197"),
    "box_delivery_classic": (30, "f85d670540aeac455505c2e41f3c2868689046c21c0276332325aac2399a140d"),
    "turn_solo": (35, "cebfcbd708a4c9b9e8e72a6085a4a87fd7fe97f00c253f7972d2cced614217c5"),
    "walk_turn_walk": (40, "111e19f139a9d86eaa127fe74548f548e4375bb4a00e491d1feefbb1308bc0c3"),
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
