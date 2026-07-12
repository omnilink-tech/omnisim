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


class TestLicense(unittest.TestCase):
    """Unit test for checking that all the source files have the Apache 2 license."""

    def setUp(self):
        """Get all the source files which require a license check."""
        directories = [
            'src/controller/c',
            'src/controller/cpp',
            'src/controller/launcher',
            'src/omnisim',
            'src/wren',
            'projects',
            'include/controller',
            'include/plugins',
            'scripts'
        ]

        skippedDirectoryPaths = [
            'src/omnisim/external',
        ]
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
                        if os.path.join(relativeRootPath, fileName).replace(os.sep, '/') in skippedFilePaths:
                            continue
                        file = os.path.join(rootPath, fileName)
                        self.sources.append(file)

    def test_sources_have_license(self):
        """Test that sources have the license."""
        for source in self.sources:
            with open(source, 'r', encoding='utf-8') as content_file:
                content = content_file.read()
                if source.endswith('.c') or source.endswith('.h'):
                    accepted = (APACHE2_LICENSE_C, APACHE2_LICENSE_C_MODIFIED, OMNILINK_LICENSE_C)
                elif source.endswith('.cpp') or source.endswith('.hpp') or source.endswith('.java'):
                    accepted = (APACHE2_LICENSE_CPP, OMNILINK_LICENSE_CPP)
                elif source.endswith('.py') or source.endswith('Makefile'):
                    for pythonHeader in PYTHON_OPTIONAL_HEADERS:
                        if content.startswith(pythonHeader + '\n'):
                            content = content[len(pythonHeader):].lstrip('\n')
                    accepted = (APACHE2_LICENSE_PYTHON, OMNILINK_LICENSE_PYTHON)
                else:
                    self.assertTrue(
                        False,
                        msg='Unsupported file extension "%s".' % source
                    )
                    continue
                self.assertTrue(
                    any(content.startswith(header) for header in accepted),
                    msg='Source file "%s" doesn\'t contain the correct Apache 2.0 License:\n%s' %
                        (source, accepted[0])
                )


if __name__ == '__main__':
    unittest.main()
