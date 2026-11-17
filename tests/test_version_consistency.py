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

"""One version string, four copies, must agree.

The release label lives in four places that nothing links: the Python package
(`omnisim/__init__.py`), the engine (`OmApplicationInfo::omniSimVersion`), the
training image (`docker/Dockerfile.train` ARG OMNISIM_TAG) and the workflow
that builds it (`.github/workflows/train-image.yml`). A bump that misses one
ships a doctor that reports 8.2.0 against an engine that says 8.1.17, or a
training image cloned from the wrong tag. Pure text reads; no engine.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

PY_INIT = REPO_ROOT / "omnisim" / "__init__.py"
APP_INFO = REPO_ROOT / "src" / "omnisim" / "core" / "OmApplicationInfo.cpp"
DOCKERFILE = REPO_ROOT / "docker" / "Dockerfile.train"
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "train-image.yml"
# The fifth copy: the tag scripts/packaging/generic_distro.py names the installer
# from, and the one publish_snapshot.sh commits as "bump distribution package".
PACKAGE_VERSION = REPO_ROOT / "scripts" / "packaging" / "omnisim_version.txt"
BUMP_SCRIPT = REPO_ROOT / "scripts" / "release" / "bump_version.py"
ALL_SITES = (PY_INIT, APP_INFO, DOCKERFILE, WORKFLOW, PACKAGE_VERSION)

_SEMVER = r"\d+\.\d+\.\d+"


def _one(pattern: str, path: Path) -> str:
    text = path.read_text(encoding="utf-8", errors="replace")
    hits = re.findall(pattern, text)
    assert hits, f"{path.relative_to(REPO_ROOT)}: no match for {pattern!r}"
    assert len(set(hits)) == 1, f"{path.relative_to(REPO_ROOT)}: conflicting values {sorted(set(hits))}"
    return hits[0]


def python_package_version() -> str:
    return _one(rf'^__version__\s*=\s*"({_SEMVER})"', PY_INIT) if False else \
        re.search(rf'^__version__\s*=\s*"({_SEMVER})"', PY_INIT.read_text(encoding="utf-8"),
                  re.M).group(1)


def engine_version() -> str:
    return _one(rf'omniSimVersionString\s*=\s*"({_SEMVER})"', APP_INFO)


def dockerfile_tag() -> str:
    return _one(rf'^ARG OMNISIM_TAG=v({_SEMVER})', DOCKERFILE) if False else \
        re.search(rf'^ARG OMNISIM_TAG=v({_SEMVER})', DOCKERFILE.read_text(encoding="utf-8"),
                  re.M).group(1)


def workflow_tags() -> set[str]:
    """Every default tag the workflow carries: the input default and the env fallback."""
    text = WORKFLOW.read_text(encoding="utf-8")
    defaults = set(re.findall(rf'default:\s*"v({_SEMVER})"', text))
    fallbacks = set(re.findall(rf"\|\|\s*'v({_SEMVER})'", text))
    assert defaults, "train-image.yml: no `default: \"vX.Y.Z\"` input"
    assert fallbacks, "train-image.yml: no `|| 'vX.Y.Z'` env fallback"
    return defaults | fallbacks


def test_all_four_version_sites_agree():
    py = python_package_version()
    versions = {
        "omnisim/__init__.py __version__": py,
        "OmApplicationInfo.cpp omniSimVersion": engine_version(),
        "docker/Dockerfile.train OMNISIM_TAG": dockerfile_tag(),
    }
    for tag in sorted(workflow_tags()):
        versions[f".github/workflows/train-image.yml v{tag}"] = tag
    distinct = set(versions.values())
    assert len(distinct) == 1, "version sites disagree:\n" + "\n".join(
        f"  {k}: {v}" for k, v in versions.items())


def test_version_is_semver():
    assert re.fullmatch(_SEMVER, python_package_version())


def package_version_tag() -> str:
    text = PACKAGE_VERSION.read_text(encoding="utf-8")
    m = re.match(rf"v({_SEMVER})\s*$", text)
    assert m, f"scripts/packaging/omnisim_version.txt: expected exactly `vX.Y.Z`, got {text!r}"
    return m.group(1)


def test_packaging_version_file_agrees():
    """The installer's tag file is a fifth copy the four-site test above never read."""
    assert package_version_tag() == python_package_version()


# ---- scripts/release/bump_version.py ------------------------------------------
#
# The script that rewrites every site in one commit must (a) recognise the
# current state as "nothing to do" and (b) be able to go red -- i.e. actually
# find and rewrite every site -- without writing a byte in --dry-run. Both run
# the script as a subprocess, the way an operator does; no engine.

def _bump_dry_run(version: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(BUMP_SCRIPT), version, "--dry-run"],
                          cwd=REPO_ROOT, capture_output=True, text=True, timeout=120)


def _site_bytes() -> dict[Path, bytes]:
    return {p: p.read_bytes() for p in ALL_SITES}


def test_bump_version_dry_run_on_current_version_reports_no_change():
    before = _site_bytes()
    res = _bump_dry_run(python_package_version())
    assert res.returncode == 0, res.stdout + res.stderr
    assert "no change" in res.stdout, res.stdout + res.stderr
    assert _site_bytes() == before, "--dry-run wrote to a version site"


def test_bump_version_dry_run_to_another_version_diffs_every_site_and_writes_nothing():
    major, minor, patch = map(int, python_package_version().split("."))
    target = f"{major}.{minor}.{patch + 1}"
    before = _site_bytes()
    res = _bump_dry_run(target)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "no change" not in res.stdout
    for site in ALL_SITES:
        rel = site.relative_to(REPO_ROOT).as_posix()
        assert f"+++ b/{rel}" in res.stdout, f"{rel} missing from the dry-run diff:\n{res.stdout}"
    assert f"+__version__ = \"{target}\"" in res.stdout
    assert f"+ARG OMNISIM_TAG=v{target}" in res.stdout
    assert _site_bytes() == before, "--dry-run wrote to a version site"
