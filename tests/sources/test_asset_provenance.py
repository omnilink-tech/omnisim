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

"""Every redistributable binary asset must ship with a licence trail.

test_license.py polices SOURCE HEADERS: it reads the top of .c/.cpp/.h/.hpp/.py
files and checks the Apache boilerplate is there. A .png, a .stl, a .wav or a
.wasm has nowhere to put a header, so that test cannot see any of them -- which
is how a tree accumulates thousands of redistributed binaries whose origin and
terms nobody recorded. This test closes that hole from the other side: it walks
the tracked binary assets and asserts each one has a licence or provenance file
somewhere above it.

Run it standalone for the full report, grouped by directory:

    python tests/sources/test_asset_provenance.py --report

Add --all to list every uncovered directory rather than the first 40.
"""

from __future__ import print_function

import collections
import os
import subprocess
import sys
import unittest

OMNISIM_HOME = os.path.normpath(
    os.environ.get('OMNISIM_HOME', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')))

# Redistributable binary payloads: geometry, textures, audio, fonts, compiled
# code and trained weights. Extensions only -- the point is to catch the files
# whose licence cannot travel inside the file itself.
ASSET_EXTENSIONS = frozenset([
    '.stl', '.dae', '.obj', '.glb', '.gltf', '.fbx',      # geometry
    '.png', '.jpg', '.jpeg', '.hdr',                      # images / environment maps
    '.wav', '.mp3', '.ogg',                               # audio
    '.ttf', '.otf', '.woff', '.woff2',                    # fonts
    '.wasm',                                              # compiled code
    '.pt', '.onnx', '.npz',                               # trained weights / arrays
    '.svg', '.gif', '.ico', '.icns', '.bmp', '.tga',      # more images
    '.exr', '.ktx', '.dds',                               # more environment / texture formats
    '.mp4', '.webm',                                      # video
    '.pdf',                                               # documents
])
# NOTE on the four lines above (added 2026-08-24). They were absent, and an
# extension this set does not name is not merely unchecked -- it is INVISIBLE:
# the gate can report "0 uncovered" while an unattributable file sits in the
# tree, because it never looked at it. Re-running the classifier over the wider
# set found 21 such files (20 .npz and one .ico). None turned out to be a real
# licensing problem, but that was luck rather than diligence, and the reason it
# was luck is that nothing would have said otherwise. Widening costs zero new
# uncovered assets now that the five missing PROVENANCE.md records exist.

# A file whose name starts with one of these (case-insensitively) confers
# coverage on its directory and everything beneath it. Covers LICENSE,
# LICENSE.upstream, LICENSE-DejaVu.txt, license.txt, NOTICE, NOTICE.upstream,
# PROVENANCE.md, PROVENANCE.txt, COPYING, COPYING.LESSER.
LICENCE_FILE_PREFIXES = ('LICENSE', 'LICENCE', 'NOTICE', 'PROVENANCE', 'COPYING')


def _git(*args):
    return subprocess.run(['git'] + list(args), cwd=OMNISIM_HOME, check=True,
                          capture_output=True, text=True).stdout.splitlines()


def _tracked_files():
    """Every path git tracks, as repo-relative forward-slash strings.

    Tracked-only, on purpose: an untracked asset is not something this
    repository ships, so policing it would make the verdict depend on local
    workspace state (a colleague's scratch renders, gitignored run output)
    rather than on the repository.
    """
    return [line for line in _git('ls-files') if line]


def _licence_files():
    """Licence/provenance files present in the working tree, tracked or not.

    ASYMMETRIC WITH _tracked_files ON PURPOSE. Assets are counted only when
    tracked, but a licence file counts the moment it exists on disk. Several
    lanes work in this tree at once, and a NOTICE written five minutes ago but
    not yet committed is a licence trail that exists -- failing the build for
    it would punish exactly the work this test is meant to encourage. Untracked
    is still filtered through .gitignore, so scratch output cannot confer
    coverage.
    """
    listed = _git('ls-files', '--cached', '--others', '--exclude-standard')
    return [path for path in listed if path and _is_licence_file(path)]


def _is_licence_file(relative_path):
    return os.path.basename(relative_path).upper().startswith(LICENCE_FILE_PREFIXES)


def _covered_directories():
    """Directories that hold a licence/provenance file.

    THE REPOSITORY ROOT IS DELIBERATELY EXCLUDED. The root carries LICENSE and
    NOTICE, so counting it would mark every asset in the tree as covered and
    this test would assert nothing at all.
    """
    return {os.path.dirname(path) for path in _licence_files() if os.path.dirname(path)}


def _ancestors(relative_path):
    """The asset's own directory and each parent, stopping before the root."""
    directory = os.path.dirname(relative_path)
    while directory:
        yield directory
        directory = os.path.dirname(directory)


# --- own-work categories ------------------------------------------------------
#
# Paths under these prefixes are OmniLink's own work, produced inside this
# repository rather than redistributed from anywhere, so the repository's own
# LICENSE already covers them and no third-party trail exists to record.
#
# KEEP THIS LIST SMALL, AND KEEP IT NARROW. Each entry is a claim that OmniLink
# authored the bytes; the optional extension tuple exists so a claim about
# trained weights cannot silently start covering photographs dropped into the
# same tree. If you cannot say who made a file, it does not belong here -- it
# belongs in KNOWN_UNCOVERED_ASSETS, or it needs a licence file beside it.
OWN_WORK = (
    # Policy checkpoints produced by this repo's own in-engine RL pipeline
    # (projects/policies/training/). Weights only -- any image, mesh or audio
    # that appears under this tree still has to justify itself.
    ('projects/policies/', ('.pt', '.onnx')),
    # NOTE: resources/branding/ USED to be claimed here. It no longer is, and
    # that is the fix rather than a regression -- resources/branding/LICENSE now
    # states the reservation as an actual notice, so the marks are covered by a
    # licence file like everything else and this entry matched nothing.
    #
    # It is worth recording WHY the old entry was wrong, because the shape
    # recurs: its comment asserted that the brand assets are "deliberately NOT
    # covered by the Apache grant ... a trademark reservation, not a missing
    # licence" -- a carve-out that, at the time, no legal file in the tree
    # actually made. TRADEMARKS.md reserved the MARKS; nothing reserved the
    # COPYRIGHT in the image files, and Apache-2.0 Sec 6 does not reach them.
    # A comment in a test is not a notice. If you ever find yourself explaining
    # a licensing position in a code comment, the position belongs in NOTICE or
    # in a LICENSE file, and the comment belongs there too.
    # Trajectory plots rendered by this repo's own omni_quest analysis scripts
    # from its own run logs.
    ('projects/omni_quest/docs/', ('.png', '.gif')),
)

# --- known gaps ---------------------------------------------------------------
#
# Assets that are NOT covered and NOT own work: real, enumerable licensing debt,
# baselined so this test can run as a green CI gate against NEW uncovered assets
# while the existing backlog stays visible and countable.
#
# THESE ARE NOT EXEMPTIONS AND THEY ARE NOT ALL LOW-RISK. Each entry names a
# tree that needs a LICENSE, NOTICE or PROVENANCE file recording where its
# contents came from and under what terms. Fixing one means adding that file --
# at which point test_known_gaps_are_still_gaps tells you to delete the line.
#
# Baselined 2026-08-22 at 2103 uncovered assets, and worked down from there.
# Counts are indicative; the entries are prefixes, so ordinary file churn does
# not invalidate them.
#
# Do not add to this list to make a red build green. A directory of assets you
# are adding gets a provenance file.
#
# CLEARED 2026-08-22 -- eleven entries were deleted from this list because the
# trees they named now carry provenance records and no longer hold a single
# uncovered asset. They are named here so the shrink is auditable rather than
# silent:
#
#   projects/objects/     -> per-tree records added by the object-library lane
#   docs/                 -> docs/PROVENANCE.md; 508 assets separated by their
#                            git add-commit into 480 inherited from the squashed
#                            Webots import and 28 authored here
#   projects/samples/     -> projects/samples/PROVENANCE.md (31 inherited, 37 own)
#   projects/default/     -> projects/default/PROVENANCE.md (37 inherited, 1 own)
#   tests/                -> tests/PROVENANCE.md (50 inherited, 18 own benchmark output)
#   resources/web/        -> resources/web/PROVENANCE.md (10 inherited, 4 own)
#   resources/images/     -> resources/images/PROVENANCE.md (4 inherited, 2 own)
#   scripts/packaging/    -> scripts/packaging/PROVENANCE.md (1 inherited, 2 brand marks)
#   src/wren/             -> src/wren/PROVENANCE.md (the WREN logo, Cyberbotics
#                            Apache-2.0; the terms were never in doubt, only the
#                            location of the licence file that stated them)
#   projects/robot_combat/-> projects/robot_combat/PROVENANCE.md (2 own icons)
#   social/Omnivoice/     -> ⚠ NOT a clean clearance. It fell off this list
#                            because social/Omnivoice/LICENSE (plain Apache-2.0,
#                            for the TOOL'S CODE) sits above the asset and this
#                            test's coverage rule is ancestor-based, so the
#                            ElevenLabs voice sample beneath it reads as covered
#                            when it is not. That FALSE POSITIVE is recorded and
#                            contained in social/Omnivoice/samples/PROVENANCE.md,
#                            which states the vendor's actual terms and what
#                            would settle them. The lesson generalises: a bare
#                            LICENSE file is a licence for the code beside it,
#                            not a provenance record for every binary below it.
KNOWN_UNCOVERED_ASSETS = (
)

KNOWN_UNCOVERED_PREFIXES = tuple(prefix for prefix, _ in KNOWN_UNCOVERED_ASSETS)


def _is_own_work(path):
    for prefix, extensions in OWN_WORK:
        if path.startswith(prefix):
            if extensions is None or os.path.splitext(path)[1].lower() in extensions:
                return True
    return False


def _classify():
    """(covered, own_work, known_gap, uncovered) lists of repo-relative asset paths."""
    covered_directories = _covered_directories()

    covered, own_work, known_gap, uncovered = [], [], [], []
    for path in _tracked_files():
        if os.path.splitext(path)[1].lower() not in ASSET_EXTENSIONS:
            continue
        if any(directory in covered_directories for directory in _ancestors(path)):
            covered.append(path)
        elif _is_own_work(path):
            own_work.append(path)
        elif path.startswith(KNOWN_UNCOVERED_PREFIXES):
            known_gap.append(path)
        else:
            uncovered.append(path)
    return covered, own_work, known_gap, uncovered


def _group_report(paths, limit=40):
    """Uncovered paths grouped by directory, biggest group first."""
    groups = collections.Counter(os.path.dirname(path) or '.' for path in paths)
    lines = ['%d uncovered asset(s) in %d director(ies):' % (len(paths), len(groups))]
    for index, (directory, count) in enumerate(groups.most_common()):
        if limit is not None and index >= limit:
            lines.append('  ... and %d more director(ies)' % (len(groups) - limit))
            break
        lines.append('  %6d  %s/' % (count, directory))
    return '\n'.join(lines)


class TestAssetProvenance(unittest.TestCase):
    """Binary assets must ship with a licence trail."""

    def test_assets_have_a_licence_trail(self):
        """Every tracked binary asset is covered, own work, or a listed known gap."""
        _, _, _, uncovered = _classify()
        self.assertEqual(
            uncovered, [],
            msg='%s\n\nFix by adding a LICENSE / NOTICE / PROVENANCE file to the directory (or to '
                'a parent) recording where the files came from and under what terms. Only if the '
                'bytes are OmniLink\'s own work does an OWN_WORK entry apply, and only if the debt '
                'is somebody else\'s to clear does a KNOWN_UNCOVERED_ASSETS entry apply.'
                % _group_report(uncovered)
        )

    def test_known_gaps_are_still_gaps(self):
        """A baselined gap with nothing left under it must be removed from the baseline.

        This is what stops the baseline rotting into a permanent exemption list.
        """
        _, _, known_gap, _ = _classify()
        live = set()
        for path in known_gap:
            for prefix in KNOWN_UNCOVERED_PREFIXES:
                if path.startswith(prefix):
                    live.add(prefix)
        stale = sorted(set(KNOWN_UNCOVERED_PREFIXES) - live)
        self.assertEqual(
            stale, [],
            msg='%d KNOWN_UNCOVERED_ASSETS entr(ies) no longer cover any uncovered asset -- the '
                'debt is cleared or the path is gone. Delete their lines:\n%s' %
                (len(stale), '\n'.join('  ' + prefix for prefix in stale))
        )

    def test_own_work_entries_match_something(self):
        """An OWN_WORK claim that matches nothing is dead weight -- and a stale claim
        about authorship is worse than no claim, so it must not linger.

        SKIPPED on a release snapshot, and the distinction matters. This test asks
        "is this exemption stale?", and the only tree that can answer is the one an
        exemption would be ADDED to -- the development tree. A published snapshot is
        the development tree minus scripts/release/publish_deny.txt, so an entry can
        match nothing there for a reason that is not staleness at all: every asset it
        covers was deliberately withheld. projects/policies/ is exactly that case --
        its .pt checkpoints are held under the LAFAN1 licence analysis, so on public
        the entry looks dead and is not.

        The deny-list denies itself, so its absence IS the signal that this is a
        filtered tree; the same heuristic the licence-pointer gate uses. The other
        two tests in this file stay live on a snapshot, because "is every asset
        covered?" is a question a snapshot can answer, and must."""
        deny_list = os.path.join(OMNISIM_HOME, 'scripts', 'release', 'publish_deny.txt')
        if not os.path.exists(deny_list):
            raise unittest.SkipTest(
                'publish_deny.txt absent: this is a release snapshot, not the '
                'development tree, so an unmatched OWN_WORK entry means "withheld", '
                'not "stale". Run this test on the development tree.')
        _, own_work, _, _ = _classify()
        matched = {prefix for prefix, _ in OWN_WORK
                   for path in own_work if path.startswith(prefix)}
        unused = sorted({prefix for prefix, _ in OWN_WORK} - matched)
        self.assertEqual(
            unused, [],
            msg='%d OWN_WORK entr(ies) match no asset. Delete their lines:\n%s' %
                (len(unused), '\n'.join('  ' + prefix for prefix in unused))
        )


def _main():
    covered, own_work, known_gap, uncovered = _classify()
    total = len(covered) + len(own_work) + len(known_gap) + len(uncovered)
    print('tracked assets : %d' % total)
    print('  covered      : %d  (licence/provenance file at or above the asset)' % len(covered))
    print('  own work     : %d  (OWN_WORK)' % len(own_work))
    print('  known gap    : %d  (KNOWN_UNCOVERED_ASSETS -- real, enumerated debt)' % len(known_gap))
    print('  UNCOVERED    : %d' % len(uncovered))
    print('')
    limit = None if '--all' in sys.argv else 40
    print('--- KNOWN GAPS (baselined debt) ---')
    print(_group_report(known_gap, limit=limit))
    print('')
    print('--- UNCOVERED (must be empty) ---')
    print(_group_report(uncovered, limit=limit))
    return 1 if uncovered else 0


if __name__ == '__main__':
    if '--report' in sys.argv or '--all' in sys.argv:
        sys.exit(_main())
    unittest.main()
