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

"""A pacman package that owns no files must not become an installer entry.

The v9.1.0 Release job failed in Inno Setup with `Source file "...\\msys64" does
not exist`: MSYS2's GCC 16.2 left mingw-w64-x86_64-cc-libs as a package with
no files, `pacman -Qql` printed nothing, and ''.split('\\n') == [''] put an
empty path into msys64_files. The dev box (GCC 15.2) never showed it.
"""

import os
import sys
import unittest
from unittest import mock

PACKAGING = os.path.join(os.path.dirname(__file__), '..', '..', 'scripts', 'packaging')
sys.path.insert(0, os.path.abspath(PACKAGING))

import windows_distro  # noqa: E402


class EmptyPackageTest(unittest.TestCase):
    def test_a_package_with_no_files_lists_nothing(self):
        with mock.patch.object(windows_distro.subprocess, 'check_output', return_value=b''):
            self.assertEqual(windows_distro.package_files('mingw-w64-x86_64-cc-libs'), [])

    def test_real_file_lists_are_unchanged(self):
        out = b'/mingw64/\n/mingw64/bin/\n/mingw64/bin/libstdc++-6.dll\n'
        with mock.patch.object(windows_distro.subprocess, 'check_output', return_value=out):
            self.assertEqual(windows_distro.package_files('mingw-w64-x86_64-libstdc++'),
                             ['/mingw64/', '/mingw64/bin/', '/mingw64/bin/libstdc++-6.dll'])

    def test_an_empty_dependency_tree_lists_nothing(self):
        with mock.patch.object(windows_distro.subprocess, 'check_output', return_value=b'\n'):
            self.assertEqual(windows_distro.list_dependencies('make'), [])


if __name__ == '__main__':
    unittest.main()
