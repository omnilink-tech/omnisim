#!/usr/bin/env python

# Copyright 1996-2024 Cyberbotics Ltd.
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
#
# Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

"""Test that checks that all the source files have the Apache 2 license."""

import unittest
import os
import fnmatch
import re
import subprocess
import hashlib

from io import open

OMNISIM_HOME = os.path.normpath(os.environ['OMNISIM_HOME'])

with open(os.path.join(OMNISIM_HOME, 'resources', 'version.txt'), 'r') as file:
    version = file.readlines()[0].strip()

year = int(version[1:5])
if version[-1] == 'a':
    year -= 1

APACHE2_LICENSE_C = """/*
 * Copyright 1996-20XX Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */""".replace('20XX', str(year))

APACHE2_LICENSE_CPP = """// Copyright 1996-20XX Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.""".replace('20XX', str(year))

APACHE2_LICENSE_PYTHON = """# Copyright 1996-20XX Cyberbotics Ltd.
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
# limitations under the License.""".replace('20XX', str(year))

PYTHON_OPTIONAL_HEADERS = [
    '#!/usr/bin/env python2',
    '#!/usr/bin/env python3',
    '#!/usr/bin/env python',
]

# Files that originated in Webots but were changed for OmniSim must carry a
# modification notice (Apache-2.0 section 4(b)). The notice sits inside the
# comment block, directly below the Apache boilerplate.
# See docs/developer/copyright-headers.md.
MODIFICATION_NOTICE = 'Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.'

# For the '//' and '#' styles the boilerplate ends at "limitations under the
# License.", so a trailing notice keeps startswith() satisfied. The C style
# closes with '*/', so it needs an explicit modified variant.
APACHE2_LICENSE_C_MODIFIED = (
    APACHE2_LICENSE_C[:-len('\n */')] + '\n *\n * ' + MODIFICATION_NOTICE + '\n */'
)

# Files authored by OmniLink with no Webots ancestry carry an OmniLink-only header.
OMNILINK_LICENSE_C, OMNILINK_LICENSE_CPP, OMNILINK_LICENSE_PYTHON = (
    re.sub(r'Copyright 1996-\d{4} Cyberbotics Ltd\.', 'Copyright 2026 OmniLink', header)
    for header in (APACHE2_LICENSE_C, APACHE2_LICENSE_CPP, APACHE2_LICENSE_PYTHON)
)

# --- vendored third-party components -----------------------------------------
#
# A vendored Apache-2.0 component carries somebody else's copyright line, so it
# can never match the two headers above. This used to be handled by skipping
# 'src/omnisim/external' wholesale -- which meant the tree's only two vendored
# C/C++ components were checked for nothing at all, and an unlicensed header sat
# there invisibly (see the note on the directory list in setUp).
#
# Instead of skipping, the directories below are held to a WEAKER but still real
# rule: the file must carry the verbatim Apache-2.0 boilerplate underneath SOME
# copyright line. That catches a file with no licence header, which is the
# failure the blanket skip hid.
VENDORED_APACHE2_DIRECTORIES = [
    # google/highwayhash SipHash, Apache-2.0, (c) 2015-2017 Google Inc.
    # Attribution and provenance: src/omnisim/external/siphash/NOTICE
    'src/omnisim/external/siphash',
]

# Official OIDN 2.5.1 headers use SPDX instead of a full boilerplate. Pin the
# unmodified files and accompanying license; do not exempt the directory.
#
# The digest is taken over LF-normalised bytes, and the pin is the digest of what
# git STORES. Pinning the raw working-tree bytes made the check pass on exactly
# one machine: the files were authored with CRLF and the directory's
# .gitattributes (`*.h text eol=lf`) was added afterwards, which normalises the
# blob but does not rewrite a file already on disk. So the author's checkout
# hashed CRLF and matched, while the committed blob -- and therefore every fresh
# clone and every CI runner -- hashed LF and did not. It went red on the public
# licence workflow from 2026-09-11 and stayed red through the v8.5.0 cut.
OIDN_HEADERS = {
    'src/omnisim/render/third_party/oidn/oidn.h':
        'f59758986a79ed60739a9a7a13a39a99cc6c4b35a04d9135769480124fa1f0af',
    'src/omnisim/render/third_party/oidn/config.h':
        '6610766cac81b9cc952b3d6ca049eb889750bc503d814d9c9761e20ea759bed9',
}

# Vendored third-party sources that carry no licence header of their own, sit in
# a directory holding the upstream LICENSE, and must stay byte-for-byte upstream.
#
# For the ALOHA study that last clause is an explicit, written invariant:
# projects/robots/trossen/aloha/PROVENANCE.md says "Upstream source is retained
# under `source/act`; modified files are outside that directory." So pasting a
# header into one of these would BREAK the provenance claim rather than satisfy
# it -- and pasting an OmniLink copyright line onto somebody else's MIT code
# would be wrong regardless of what any test wanted.
#
# Exempting the directory would check nothing, which is the mistake the note on
# VENDORED_APACHE2_DIRECTORIES above describes. Pinning the content excuses the
# missing header and still detects any edit, which is the trade the OIDN entries
# make. Each value is (sha256 of the LF-normalised bytes, the LICENSE that
# covers the file, a marker that must appear in that LICENSE).
#
# As with OIDN the digest is over LF-NORMALISED bytes taken from what git
# STORES, not from the working tree. Pinning raw working-tree bytes is what made
# the OIDN check pass on exactly one machine and go red on every fresh clone.
VENDORED_PINNED_SOURCES = {
    # ACT, MIT, (c) 2023 Tony Z. Zhao -- https://github.com/tonyzhaozh/act at
    # 742c753c0d4a5d87076c8f69e5628c79a8cc5488. The ALOHA arm model is converted
    # FROM these; see projects/robots/trossen/aloha/PROVENANCE.md.
    'projects/robots/trossen/aloha/source/act/constants.py': (
        '2e27d4412e2b7e1f2e4148105fdf514d8c52ec6c8f1c86ef0549c475e8b91d86',
        'projects/robots/trossen/aloha/source/act/LICENSE',
        'MIT License'),
    # ALOHA hardware scripts, MIT, (c) 2023 Tony Z. Zhao.
    'projects/robots/trossen/aloha/source/hardware/aloha_scripts/constants.py': (
        'cef81cfcbb41bb3a2e835d909f7a29e57527563cb4a150580ee1f195b7c4da83',
        'projects/robots/trossen/aloha/source/hardware/LICENSE',
        'MIT License'),
    'projects/robots/trossen/aloha/source/hardware/aloha_scripts/real_env.py': (
        '30482952e73c0385bd8edfb238ceb4201458ce5fa5afae218cf03b75549150f4',
        'projects/robots/trossen/aloha/source/hardware/LICENSE',
        'MIT License'),
}

# The Apache boilerplate with its leading copyright line removed, per comment
# style. '//' and '#' headers open with the copyright line; the C header opens
# with '/*' first, so two lines come off there.
APACHE2_BODY_CPP = APACHE2_LICENSE_CPP.split('\n', 1)[1]
APACHE2_BODY_PYTHON = APACHE2_LICENSE_PYTHON.split('\n', 1)[1]
APACHE2_BODY_C = APACHE2_LICENSE_C.split('\n', 2)[2]

# Any plausible copyright line, in each comment style.
THIRD_PARTY_COPYRIGHT_CPP = re.compile(r'^// Copyright .+\n')
THIRD_PARTY_COPYRIGHT_PYTHON = re.compile(r'^# Copyright .+\n')
THIRD_PARTY_COPYRIGHT_C = re.compile(r'^/\*\n \* Copyright .+\n')

# --- known gaps ---------------------------------------------------------------
#
# Files that carry NO licence header today. Baselined 2026-08-22 so this test can
# serve as a green CI gate against NEW regressions while the existing debt stays
# visible and countable.
#
# THESE ARE NOT EXEMPTIONS. Every one is an OmniLink-authored file that simply
# needs the standard header pasted at the top -- a one-minute fix owned by the
# lane that authored it. The list cannot rot: test_known_gaps_are_still_gaps
# fails the moment a listed file starts passing or stops existing, and the fix
# for that failure is to delete the line.
#
# Do not add to this list to make a red build green. A file you are adding or
# touching gets the header.
KNOWN_MISSING_LICENSE_HEADERS = {
    # Empty as of 2026-08-22: the 12 OmniLink-authored files that were baselined
    # here have been given the standard header. Keep it empty -- an entry added
    # here is licence debt, not an exemption, and test_known_gaps_are_still_gaps
    # below will go red the moment a listed file is fixed or removed.
}


class TestLicense(unittest.TestCase):
    """Unit test for checking that all the source files have the Apache 2 license."""

    @staticmethod
    def _trackedFiles():
        """Paths git tracks, as OMNISIM_HOME-relative forward-slash strings.

        The walk below is a filesystem walk, so without this it also polices
        files git does not track -- gitignored scratch output, a colleague's
        untracked WIP -- and the verdict then depends on local workspace state
        rather than on the repository. scripts/dev/rename_audit.py already
        learned this the hard way and reads the index for the same reason.

        Returns None if git is unavailable, in which case no filtering happens
        and the test behaves exactly as it did before.
        """
        try:
            out = subprocess.run(['git', 'ls-files'], cwd=OMNISIM_HOME, check=True,
                                 capture_output=True, text=True).stdout
        except (OSError, subprocess.CalledProcessError):
            return None
        return set(out.splitlines())

    def setUp(self):
        """Get all the source files which require a license check."""
        tracked = self._trackedFiles()
        directories = [
            'src/controller/c',
            'src/controller/cpp',
            'src/controller/launcher',
            'src/omnisim',
            'projects',
            'include/controller',
            'include/plugins',
            'scripts',
            'packages',
            'omnisim',
            # 'agents' is walked in full (the "not here yet" caveat this comment
            # used to carry was stale by 2026-09-01 -- the entry had already been
            # widened to the whole tree four lines below it).
            'agents'
        ]

        # Deliberately empty. 'src/omnisim/external' used to sit here, which is
        # how an unlicensed header assembled from CC BY-SA Stack Overflow answers
        # shipped in an Apache-2.0 tree without this test ever seeing it. The two
        # components under that directory are now checked: the clean-room
        # compilation_timestamp.h carries the ordinary OmniLink header, and
        # siphash is checked against VENDORED_APACHE2_DIRECTORIES above.
        # Anything added here should be one precise path with a stated reason,
        # never a parent directory that swallows future additions.
        skippedDirectoryPaths = []
        skippedDirectoryPathsFull = [os.path.join(OMNISIM_HOME, os.path.normpath(path))
                                     for path in skippedDirectoryPaths]

        skippedFilePaths = [
            'scripts/packaging/iscc_formatter.c',
            'src/controller/c/sha1.c',
            'src/controller/c/sha1.h'
        ]

        skippedDirectories = [
            'build'
        ]

        extensions = ['*.c', '*.cpp', '*.h', '*.hpp', '*.py', '*.java', 'Makefile']

        self.sources = []
        for directory in directories:
            for rootPath, dirNames, fileNames in os.walk(os.path.join(OMNISIM_HOME, os.path.normpath(directory))):
                shouldContinue = False
                relativeRootPath = rootPath.replace(OMNISIM_HOME + os.sep, '')
                for skippedPath in skippedDirectoryPathsFull:
                    if rootPath.startswith(skippedPath):
                        shouldContinue = True
                        break
                currentDirectories = rootPath.replace(OMNISIM_HOME + os.sep, '').split(os.sep)
                for directory in skippedDirectories:
                    if directory in currentDirectories:
                        shouldContinue = True
                        break
                if fileNames == '__init__.py':
                    shouldContinue = True
                if shouldContinue:
                    continue
                for extension in extensions:
                    for fileName in fnmatch.filter(fileNames, extension):
                        relativePath = os.path.join(relativeRootPath, fileName).replace(os.sep, '/')
                        if relativePath in skippedFilePaths:
                            continue
                        if tracked is not None and relativePath not in tracked:
                            continue  # untracked or gitignored: not part of the repository
                        file = os.path.join(rootPath, fileName)
                        self.sources.append(file)

    @staticmethod
    def _isVendoredApache2(source):
        """True if source lives under one of VENDORED_APACHE2_DIRECTORIES."""
        relativePath = os.path.relpath(source, OMNISIM_HOME).replace(os.sep, '/')
        return any(relativePath.startswith(directory + '/')
                   for directory in VENDORED_APACHE2_DIRECTORIES)

    @staticmethod
    def _hasVendoredApache2(content, style):
        """True if content is the Apache-2.0 boilerplate under some copyright line.

        Used only for vendored components, whose copyright line belongs to the
        upstream author rather than to Cyberbotics or OmniLink.
        """
        pattern, body = {
            'c': (THIRD_PARTY_COPYRIGHT_C, APACHE2_BODY_C),
            'cpp': (THIRD_PARTY_COPYRIGHT_CPP, APACHE2_BODY_CPP),
            'python': (THIRD_PARTY_COPYRIGHT_PYTHON, APACHE2_BODY_PYTHON),
        }[style]
        match = pattern.match(content)
        return match is not None and content[match.end():].startswith(body)

    def _verdicts(self):
        """{repo-relative path: bool} -- does each source carry an accepted header?"""
        verdicts = {}
        for source in self.sources:
            relativePath = os.path.relpath(source, OMNISIM_HOME).replace(os.sep, '/')
            with open(source, 'r', encoding='utf-8') as content_file:
                content = content_file.read()
            if source.endswith('.c') or source.endswith('.h'):
                style = 'c'
                accepted = (APACHE2_LICENSE_C, APACHE2_LICENSE_C_MODIFIED, OMNILINK_LICENSE_C)
            elif source.endswith('.cpp') or source.endswith('.hpp') or source.endswith('.java'):
                style = 'cpp'
                accepted = (APACHE2_LICENSE_CPP, OMNILINK_LICENSE_CPP)
            elif source.endswith('.py') or source.endswith('Makefile'):
                style = 'python'
                for pythonHeader in PYTHON_OPTIONAL_HEADERS:
                    if content.startswith(pythonHeader + '\n'):
                        content = content[len(pythonHeader):].lstrip('\n')
                accepted = (APACHE2_LICENSE_PYTHON, OMNILINK_LICENSE_PYTHON)
            else:
                self.fail('Unsupported file extension "%s".' % source)
            if relativePath in VENDORED_PINNED_SOURCES:
                expected, licensePath, marker = VENDORED_PINNED_SOURCES[relativePath]
                with open(source, 'rb') as pinned:
                    raw = pinned.read().replace(b'\r\n', b'\n')
                digest = hashlib.sha256(raw).hexdigest()
                licenseFile = os.path.join(OMNISIM_HOME, licensePath)
                licenseText = ''
                if os.path.isfile(licenseFile):
                    with open(licenseFile, encoding='utf-8') as handle:
                        licenseText = handle.read()
                verdicts[relativePath] = (digest == expected and marker in licenseText)
            elif relativePath in OIDN_HEADERS:
                with open(source, 'rb') as header:
                    raw = header.read().replace(b'\r\n', b'\n')
                digest = hashlib.sha256(raw).hexdigest()
                with open(os.path.join(os.path.dirname(source), 'LICENSE.txt'), encoding='utf-8') as license_file:
                    license_text = license_file.read()
                verdicts[relativePath] = (digest == OIDN_HEADERS[relativePath]
                    and content.startswith('// Copyright 2018 Intel Corporation\n// SPDX-License-Identifier: Apache-2.0')
                    and 'Apache License' in license_text and 'Version 2.0' in license_text)
            elif self._isVendoredApache2(source):
                verdicts[relativePath] = self._hasVendoredApache2(content, style)
            else:
                verdicts[relativePath] = any(content.startswith(header) for header in accepted)
        return verdicts

    def test_sources_have_license(self):
        """Test that sources have the license."""
        verdicts = self._verdicts()
        missing = sorted(path for path, ok in verdicts.items()
                         if not ok and path not in KNOWN_MISSING_LICENSE_HEADERS)
        self.assertEqual(
            missing, [],
            msg='%d source file(s) do not carry an accepted Apache 2.0 licence header:\n%s\n\n'
                'Paste the standard header at the top of each file. For a file with no Webots '
                'ancestry use the OmniLink form:\n%s' %
                (len(missing), '\n'.join('  ' + path for path in missing), OMNILINK_LICENSE_PYTHON)
        )

    def test_known_gaps_are_still_gaps(self):
        """Every KNOWN_MISSING_LICENSE_HEADERS entry must still be a real, present gap.

        This is what stops the baseline rotting into a permanent exemption list:
        the moment somebody adds the header (or deletes the file), its line here
        becomes wrong and this test says so.
        """
        verdicts = self._verdicts()
        fixed = sorted(path for path in KNOWN_MISSING_LICENSE_HEADERS if verdicts.get(path) is True)
        gone = sorted(path for path in KNOWN_MISSING_LICENSE_HEADERS if path not in verdicts)
        self.assertEqual(
            fixed, [],
            msg='%d file(s) listed in KNOWN_MISSING_LICENSE_HEADERS now carry a licence header. '
                'Delete their lines from that set:\n%s' %
                (len(fixed), '\n'.join('  ' + path for path in fixed))
        )
        self.assertEqual(
            gone, [],
            msg='%d path(s) listed in KNOWN_MISSING_LICENSE_HEADERS are no longer checked (moved, '
                'renamed, deleted or now untracked). Delete their lines from that set:\n%s' %
                (len(gone), '\n'.join('  ' + path for path in gone))
        )

    def test_pinned_vendored_sources_are_still_checked(self):
        """Every VENDORED_PINNED_SOURCES entry must still name a file this suite reads.

        A pin whose file was moved, renamed or deleted is dead weight that reads
        like coverage: the path simply never comes up, the dict entry is never
        consulted, and nothing says so. This is the guard
        test_known_gaps_are_still_gaps gives the exemption baseline, applied to
        the pins.
        """
        verdicts = self._verdicts()
        gone = sorted(path for path in VENDORED_PINNED_SOURCES if path not in verdicts)
        self.assertEqual(
            gone, [],
            msg='%d pinned vendored source(s) are no longer checked (moved, renamed, deleted or '
                'now untracked). Delete their entries from VENDORED_PINNED_SOURCES, or restore '
                'the files:\n%s' % (len(gone), '\n'.join('  ' + path for path in gone))
        )


if __name__ == '__main__':
    unittest.main()
