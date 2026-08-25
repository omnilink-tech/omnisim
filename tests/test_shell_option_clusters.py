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

"""A bundled short-option cluster containing `f` must not be followed by a flag.

THE BUG THIS EXISTS FOR. `dependencies/Makefile.linux` carried four calls of the
shape::

    tar xfm --no-same-owner $(PACKAGE) -C $(DEST)

In the bundled cluster ``xfm`` the ``f`` takes the NEXT argument as the archive
name, so tar tried to open a file literally called ``--no-same-owner``::

    tar: --no-same-owner: Cannot open: No such file or directory
    tar: Error is not recoverable: exiting now

Every Linux dependency unpack -- Qt, OIS, assimp, openssl -- was broken by it,
for fifteen days, and nothing noticed. `dependencies/Makefile.linux` is gated
behind ``ifeq ($(OSTYPE),linux)``, so every check that ran on Windows was
structurally incapable of seeing it, and the only Linux build in the project
fires *after* a release is already public. It was found by a user's failed image
build (fixed in v5.5.1, commit 5f44cab9).

WHY A TEST AND NOT A COMMENT. The same shape sat unfixed in
``dependencies/Makefile.mac`` at the time this was written -- not yet malformed,
but one inserted long option away from it, on the one platform with no automated
verification of any kind. A rule that is only written down gets re-broken; this
one runs everywhere, needs no engine, no GPU and no network, and takes
milliseconds.

THE RULE. If a token is a bundled short-option cluster CONTAINING ``f``
(``xfm``, ``-xzf``, ``czf`` ...), the very next token must be the operand,
not another option. Write ``-xmf FILE`` (``f`` last, operand binds to it) or
pass the flag before the cluster.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

# Files where a shell command line can appear. Deliberately narrow: this is
# about build/packaging recipes, which are the ones no CI on this project runs.
PATTERNS = ("Makefile*", "*.mk", "*.sh", "Dockerfile*")

SKIP_DIRS = {
    ".git", ".claude", "msys64", "dependencies/webots-qt", "src/glm", "src/stb",
    "node_modules", "_scratch", ".build_tmp", "distribution",
}

# `tar xfm --flag`, `tar -xzf --flag`, `unzip -of --flag` ... The cluster must
# be short options only (no `--`), must CONTAIN `f` anywhere, and must be followed by a
# token starting with `-`. A lone `-f` is fine; the hazard is the bundle.
_HAZARD = re.compile(
    r"""\b(?:tar|unzip|cpio)\s+          # an archiver whose -f takes an operand
        (-?[a-zA-Z]*f[a-zA-Z]*)\s+       # a short cluster CONTAINING f, any position
        (--?[A-Za-z][-\w]*)              # ...followed by ANOTHER option
    """,
    re.VERBOSE,
)


def _candidate_files() -> list[Path]:
    out: list[Path] = []
    for pat in PATTERNS:
        for p in REPO.rglob(pat):
            rel = p.relative_to(REPO).as_posix()
            if any(rel.startswith(d) or f"/{d}/" in f"/{rel}" for d in SKIP_DIRS):
                continue
            if p.is_file():
                out.append(p)
    return out


def test_no_option_after_a_cluster_containing_f() -> None:
    """The v5.5.1 bug shape, anywhere in the tree."""
    offenders: list[str] = []
    for path in _candidate_files():
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for lineno, line in enumerate(text.splitlines(), 1):
            stripped = line.strip()
            if stripped.startswith("#") or stripped.startswith("//"):
                continue
            m = _HAZARD.search(line)
            if m:
                offenders.append(
                    f"{path.relative_to(REPO).as_posix()}:{lineno}: "
                    f"'{m.group(1)}' is a bundled cluster containing 'f', so the next "
                    f"argument is taken as the FILENAME -- but it is '{m.group(2)}'. "
                    f"Write the flag before the cluster, or use '-f' last. Line: {stripped}"
                )
    assert not offenders, (
        "A bundled short-option cluster containing 'f' is followed by another option. "
        "The 'f' will consume that option as the archive name and the command will fail "
        "with 'Cannot open: No such file or directory'. This broke every Linux dependency "
        "unpack for fifteen days (fixed in v5.5.1).\n  " + "\n  ".join(offenders)
    )


def test_the_detector_actually_fires() -> None:
    """A guard that has never gone red should be assumed broken.

    The project learned this twice in one week: an assertion that cannot fail
    reads green forever. So prove the regex catches the exact historical line.
    """
    historical = "\t@tar xfm --no-same-owner $(WEBOTS_DEPENDENCY_PATH)/$(QT_PACKAGE) -C $(WEBOTS_HOME)"
    m = _HAZARD.search(historical)
    assert m is not None, "the detector does not catch the bug it was written for"
    assert m.group(1) == "xfm" and m.group(2) == "--no-same-owner"

    # ...and does not fire on either correct form.
    for ok in (
        "\t@tar --no-same-owner -xmf $(DEST)/$(PKG) -C $(HOME)",
        "\t@tar -xmf $(PKG) -C $(HOME)",
        "\t@tar xfm $(PKG) -C $(HOME)",
        "\ttar -xzf archive.tar.gz",
    ):
        assert _HAZARD.search(ok) is None, f"false positive on: {ok}"
