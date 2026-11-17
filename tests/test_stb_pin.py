# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Every place that fetches stb by hand fetches the commit the tree's submodule records.

omichel/stb deleted its ``patch-1`` branch (found 2026-11-05: the public train-image
build failed with "Remote branch patch-1 not found"). The commit ``src/stb`` points at is
still served by hash, so the Dockerfiles and the RunPod bootstrap fetch it by hash. This
keeps them equal to the submodule's gitlink, and stops a branch clone from coming back.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
FETCHERS = ("docker/Dockerfile.train", "docker/Dockerfile.runtime", "cloud/runpod/bootstrap.sh")


def _gitlink() -> str:
    try:
        out = subprocess.run(["git", "-C", str(ROOT), "ls-tree", "HEAD", "src/stb"],
                             capture_output=True, text=True, check=True).stdout.split()
    except (OSError, subprocess.CalledProcessError):
        pytest.skip("not a git checkout")
    if len(out) < 3 or out[1] != "commit":
        pytest.skip("src/stb is not a submodule here")
    return out[2]


@pytest.mark.parametrize("rel", FETCHERS)
def test_stb_is_fetched_by_the_submodule_commit(rel):
    path = ROOT / rel
    if not path.exists():
        pytest.skip(f"{rel} is not in this tree")
    text = path.read_text(encoding="utf-8")
    assert "patch-1 https://github.com/omichel/stb" not in text and "-b patch-1" not in text, \
        f"{rel} clones stb's deleted patch-1 branch"
    pins = set(re.findall(r"\b[0-9a-f]{40}\b", text))
    assert _gitlink() in pins, f"{rel} does not fetch the commit src/stb records ({_gitlink()[:9]})"


def test_gitmodules_names_no_deleted_branch():
    assert "branch = patch-1" not in (ROOT / ".gitmodules").read_text(encoding="utf-8")
