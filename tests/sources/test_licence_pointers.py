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

"""Every repo path an attribution document points at must resolve.

NOTICE and THIRD_PARTY_NOTICES.md discharge Apache-2.0 section 4 and the
attribution clauses of the licences this tree vendors. They do it by POINTING:
"the licence text is at <robot package>/LICENSE.upstream". A pointer that no
longer resolves is not a typo -- it is an attribution that has silently stopped
being made, and nothing else in the repository can notice.

(This paragraph used to name projects/robots/nasa/valkyrie/LICENSE.upstream as
its example. That package was deleted on 2026-08-22, so the docstring of the
test that catches dead licence pointers had itself acquired one.)

That is not hypothetical here. src/ode/ was deleted wholesale in bdc02139 while
both documents still cited licence files inside it.

Run it standalone to see every pointer and its verdict:

    python tests/sources/test_licence_pointers.py --report
"""

from __future__ import print_function

import glob
import os
import re
import sys
import unittest

OMNISIM_HOME = os.path.normpath(
    os.environ.get('OMNISIM_HOME', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')))

# The attribution documents this test guards.
LICENCE_DOCUMENTS = ('NOTICE', 'THIRD_PARTY_NOTICES.md')

# A markdown link target: [text](target)
MARKDOWN_LINK = re.compile(r'\]\(([^)\s]+)\)')

# A bare path in prose or in a code span. The leading segment is checked
# against the repository's real top-level entries (see _top_level_names), which
# is what keeps this from matching every slash-separated phrase in English.
# Braces are admitted so the shorthand these documents use for a family of
# files -- Code200{0,1,2}.ttf -- is captured whole rather than truncated at the
# brace and then reported as a phantom dangling path.
PATH_LIKE = re.compile(r'[A-Za-z0-9_.+{}-]+(?:/[A-Za-z0-9_.*+,{}-]+)+/?')

# The brace shorthand above, e.g. Code200{0,1,2}.ttf -> three real filenames.
BRACE_GROUP = re.compile(r'\{([^{}]*)\}')

# Trailing sentence punctuation and wrappers to peel off a captured path.
TRAILING_JUNK = '.,;:)"\'`*'

# A GitHub-style line anchor, e.g. OmNewtonBackend.cpp#L2384
LINE_ANCHOR = re.compile(r'#L\d+(?:-L\d+)?$')

# --- historical sections ------------------------------------------------------
#
# Both documents keep a section that describes dependencies which HAVE BEEN
# REMOVED, and names the paths their licence texts used to live at. Those
# mentions are correct precisely BECAUSE the paths are gone -- requiring them to
# resolve would be requiring the document to lie. So a path cited under a
# heading that marks the section as historical is recorded and reported, but
# does not fail the test.
#
# The exemption is scoped to the section, not to the path: cite src/ode/COPYING
# in the live attribution table and this test still fires.
ATX_HEADING = re.compile(r'^#{1,6}\s+(.*?)\s*#*$')
UNDERLINE_RULE = re.compile(r'^[-=]{3,}$')
HISTORICAL_HEADING = re.compile(r'histor|removed|retired|deleted|no longer|superseded', re.I)


# --- held from the public snapshot --------------------------------------------
#
# A few components are DELIBERATELY absent from the public release snapshot: the
# release deny-list (scripts/release/publish_deny.txt) removes them because their
# licence does not permit us to redistribute them, or because they are held for
# another documented reason. Their licence texts go with them -- a licence file
# is worthless beside material that is not there.
#
# Both attribution documents still describe those components, and both say in
# their own prose that the files are excluded. So on a PUBLIC checkout their
# pointers cannot resolve, and that is correct rather than broken: the material
# and its attribution are absent together, which is the opposite of the silent
# failure this test exists to catch.
#
# The exemption is deliberately NOT inferred from the deny-list at runtime,
# because the deny-list is itself deny-listed -- a public checkout has no copy to
# read. It is written out here instead, and test_held_entries_are_really_held
# checks this tuple against the real deny-list whenever one is present, so the
# list cannot rot into a way of hiding a genuine deletion.
#
# An entry ending in '/' covers everything beneath it.
HELD_FROM_PUBLIC_SNAPSHOT = (
    'projects/robots/franka_emika/',
    'resources/fonts/Code2000.ttf',
    'resources/fonts/Code2001.ttf',
    'resources/fonts/Code2002.ttf',
    'resources/web/wwi/fonts/Code2000.woff2',
    'resources/web/wwi/fonts/Code2001.woff2',
    'resources/web/wwi/fonts/Code2002.woff2',
    'scripts/release/publish_deny.txt',
)


# --- submodules ---------------------------------------------------------------
#
# NOTICE attributes glm and stb by pointing INSIDE src/glm and src/stb, which
# are submodules. A default `git clone` -- and a default actions/checkout --
# leaves those directories empty, so those pointers cannot resolve even though
# the documents are right and the licence files are there in any complete
# checkout. That does not excuse them (an attribution nobody receives is not an
# attribution), so the test still fails; this exists so the failure names the
# cause instead of advising the reader to delete something correct.


def _uninitialised_submodules():
    """Submodule paths registered in .gitmodules whose working tree is empty."""
    gitmodules = os.path.join(OMNISIM_HOME, '.gitmodules')
    if not os.path.exists(gitmodules):
        return set()
    registered = []
    with open(gitmodules, 'r', encoding='utf-8') as handle:
        for line in handle:
            key, separator, value = line.partition('=')
            if separator and key.strip() == 'path' and value.strip():
                registered.append(value.strip())
    empty = set()
    for path in registered:
        path = path.replace(os.sep, '/').strip('/')
        absolute = os.path.join(OMNISIM_HOME, os.path.normpath(path))
        if not os.path.isdir(absolute) or not os.listdir(absolute):
            empty.add(path)
    return empty


def _owning_submodule(path, uninitialised):
    """The uninitialised submodule a cited path sits inside, or None."""
    for submodule in uninitialised:
        if path == submodule or path.startswith(submodule + '/'):
            return submodule
    return None


def _top_level_names():
    """Real top-level entries of the repository, used to anchor path detection."""
    return {name for name in os.listdir(OMNISIM_HOME) if not name.startswith('.')}


def _clean(candidate):
    """Normalise one captured token into a repo-relative path, or None."""
    candidate = candidate.strip().strip(TRAILING_JUNK)
    if not candidate:
        return None
    if candidate.startswith(('http://', 'https://', 'mailto:', '#', '//')):
        return None
    candidate = LINE_ANCHOR.sub('', candidate)
    candidate = candidate.rstrip('/')
    # A bare "a/b" with no dot and no known root is more likely prose than a
    # path; the top-level check in _pointers is what actually decides.
    return candidate or None


def _headings(lines):
    """[(0-based line index, heading text)] for both markdown heading styles.

    NOTICE is plain text and underlines its headings with --- or ===;
    THIRD_PARTY_NOTICES.md uses ###. Both are recognised.
    """
    headings = []
    for index, line in enumerate(lines):
        match = ATX_HEADING.match(line)
        if match and match.group(1):
            headings.append((index, match.group(1)))
        elif (line.strip() and index + 1 < len(lines)
                and UNDERLINE_RULE.match(lines[index + 1].strip())
                and not UNDERLINE_RULE.match(line.strip())):
            headings.append((index, line.strip()))
    return headings


def _historical_lines(lines):
    """Set of 0-based line indices that sit under a historical/removed heading."""
    headings = _headings(lines)
    historical = set()
    for position, (index, text) in enumerate(headings):
        if not HISTORICAL_HEADING.search(text):
            continue
        end = headings[position + 1][0] if position + 1 < len(headings) else len(lines)
        historical.update(range(index, end))
    return historical


def _pointers(document_path, top_level):
    """{path: (set of 1-based line numbers, all_historical)} for cited repo paths."""
    with open(document_path, 'r', encoding='utf-8') as handle:
        lines = handle.read().splitlines()
    historical = _historical_lines(lines)

    found = {}
    for index, line in enumerate(lines):
        candidates = list(MARKDOWN_LINK.findall(line)) + list(PATH_LIKE.findall(line))
        for candidate in candidates:
            path = _clean(candidate)
            if path is None:
                continue
            if path.split('/', 1)[0] not in top_level:
                continue
            numbers, live = found.setdefault(path, (set(), set()))
            numbers.add(index + 1)
            if index not in historical:
                live.add(index + 1)
    return found


def _expand_braces(path):
    """Code200{0,1,2}.ttf -> [Code2000.ttf, Code2001.ttf, Code2002.ttf]."""
    match = BRACE_GROUP.search(path)
    if match is None:
        return [path]
    expanded = []
    for alternative in match.group(1).split(','):
        expanded.extend(_expand_braces(path[:match.start()] + alternative.strip() + path[match.end():]))
    return expanded


def _resolves(path):
    """True if every brace alternative exists (globs need one match each)."""
    for variant in _expand_braces(path):
        absolute = os.path.join(OMNISIM_HOME, os.path.normpath(variant))
        if '*' in variant or '?' in variant:
            if not glob.glob(absolute):
                return False
        elif not os.path.exists(absolute):
            return False
    return True


def _is_held(path):
    """True if every brace alternative of `path` is held from the public snapshot."""
    for variant in _expand_braces(path):
        variant = variant.strip('/')
        if not any(variant == entry.rstrip('/') if not entry.endswith('/')
                   else variant.startswith(entry)
                   for entry in HELD_FROM_PUBLIC_SNAPSHOT):
            return False
    return True


def _audit():
    """(dangling, historical, held, total) -- pointers that do not resolve, by cause.

    dangling   [(document, path, lines)] cited OUTSIDE a historical section, and
               not held from the public snapshot. Bugs.
    historical [(document, path, lines)] cited ONLY inside a historical section.
               Reported, not failed -- the component was removed and the document
               is correct to say where it used to live.
    held       [(document, path, lines)] absent because the release deny-list
               removes the component AND its licence text together. Reported, not
               failed -- see HELD_FROM_PUBLIC_SNAPSHOT.
    total      distinct repo paths cited across both documents.
    """
    top_level = _top_level_names()
    dangling, historical, held, total = [], [], [], 0
    for document in LICENCE_DOCUMENTS:
        document_path = os.path.join(OMNISIM_HOME, document)
        if not os.path.exists(document_path):
            dangling.append((document, '<the document itself is missing>', []))
            continue
        pointers = _pointers(document_path, top_level)
        total += len(pointers)
        for path, (lines, live) in sorted(pointers.items()):
            if _resolves(path):
                continue
            if not live:
                historical.append((document, path, sorted(lines)))
            elif _is_held(path):
                held.append((document, path, sorted(live)))
            else:
                dangling.append((document, path, sorted(live)))
    return dangling, historical, held, total


class TestLicencePointers(unittest.TestCase):
    """Attribution documents must not point at paths that no longer exist."""

    def test_licence_document_pointers_resolve(self):
        """Every live repo-relative path cited by NOTICE / THIRD_PARTY_NOTICES.md exists."""
        dangling, _, _, _ = _audit()
        report = '\n'.join(
            '  %-24s %s   (line%s %s)' % (document, path, '' if len(lines) == 1 else 's',
                                          ', '.join(str(line) for line in lines))
            for document, path, lines in dangling)
        uninitialised = _uninitialised_submodules()
        stranded = sorted({owner for _, path, _ in dangling
                           for owner in [_owning_submodule(path, uninitialised)] if owner})
        note = ''
        if stranded:
            note = ('\n\nNOTE: %s registered in .gitmodules but NOT checked out (the directory '
                    'is empty), so pointers into it cannot resolve here. That is a checkout '
                    'problem, not a document problem -- run `git submodule update --init`, or set '
                    '`submodules: true` on actions/checkout. Do not delete the attribution.'
                    % ', '.join(stranded))
        self.assertEqual(
            dangling, [],
            msg='%d attribution pointer(s) do not resolve. Each one is an attribution that has '
                'stopped being made:\n%s\n\nFix by correcting the path, or by moving the entry '
                'into the document\'s historical section if the component was removed.%s'
                % (len(dangling), report, note)
        )

    def test_held_entries_are_really_held(self):
        """Every HELD_FROM_PUBLIC_SNAPSHOT entry is genuinely removed by the deny-list.

        This is what stops the exemption becoming a place to hide a deletion. On a
        public checkout the deny-list is itself denied, so there is nothing to check
        against and the test skips; on the development tree -- which is where an
        entry would be added -- it must be justified by a real deny-list pathspec.
        """
        specs = _deny_list_pathspecs()
        if specs is None:
            self.skipTest('scripts/release/publish_deny.txt absent (public checkout)')
        import subprocess
        denied = set()
        for spec in specs:
            result = subprocess.run(['git', 'ls-files', '--', spec],
                                    cwd=OMNISIM_HOME, capture_output=True, text=True)
            denied.update(line.strip() for line in result.stdout.splitlines() if line.strip())
        unjustified = []
        for entry in HELD_FROM_PUBLIC_SNAPSHOT:
            if entry.endswith('/'):
                covered = any(path.startswith(entry) for path in denied)
            else:
                covered = entry in denied
            if not covered:
                unjustified.append(entry)
        self.assertEqual(
            unjustified, [],
            msg='%d HELD_FROM_PUBLIC_SNAPSHOT entr%s not removed by any deny-list pathspec: %s. '
                'Either the component is no longer held -- in which case delete the entry and '
                'let the pointer be checked again -- or it was deleted outright, in which case '
                'move its attribution into a historical section instead of exempting it.'
                % (len(unjustified), 'y is' if len(unjustified) == 1 else 'ies are',
                   ', '.join(unjustified))
        )

    def test_the_documents_still_point_at_things(self):
        """A document that cites nothing resolvable has stopped attributing.

        Guards the extractor as much as the documents: if a future rewrite of
        either file defeats path detection, this test goes red instead of the
        suite quietly passing on an empty pointer set.
        """
        _, _, _, total = _audit()
        self.assertGreater(
            total, 50,
            msg='Only %d repo paths were detected across %s. Either the documents stopped '
                'citing licence locations, or the path extractor in this test no longer matches '
                'how they are written.' % (total, ', '.join(LICENCE_DOCUMENTS))
        )


def _deny_list_pathspecs():
    """Non-comment pathspecs from the release deny-list, or None if it is absent.

    Absent is the normal case on a public checkout: the deny-list denies itself.
    """
    deny = os.path.join(OMNISIM_HOME, 'scripts', 'release', 'publish_deny.txt')
    if not os.path.exists(deny):
        return None
    specs = []
    with open(deny, 'r', encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if line and not line.startswith('#'):
                specs.append(line)
    return specs


def _main():
    dangling, historical, held, total = _audit()
    uninitialised = sorted(_uninitialised_submodules())
    print('documents  : %s' % ', '.join(LICENCE_DOCUMENTS))
    if uninitialised:
        print('submodules : %s not checked out -- pointers into them cannot resolve here'
              % ', '.join(uninitialised))
    print('pointers   : %d distinct repo paths cited' % total)
    print('dangling   : %d  (cited as live -- these FAIL)' % len(dangling))
    print('historical : %d  (cited only under a "removed"/"historical" heading -- reported only)'
          % len(historical))
    print('held       : %d  (component AND licence text removed by the release deny-list '
          '-- reported only)' % len(held))
    print('')
    for label, rows in (('DANGLING', dangling), ('HISTORICAL', historical), ('HELD', held)):
        print('--- %s ---' % label)
        for document, path, lines in rows:
            print('  %-24s %s   (lines %s)' % (document, path, ', '.join(str(line) for line in lines)))
        print('')
    return 1 if dangling else 0


if __name__ == '__main__':
    if '--report' in sys.argv:
        sys.exit(_main())
    unittest.main()
