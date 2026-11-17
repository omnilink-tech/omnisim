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

"""Test that the C, C++ and shader source code is compliant with ClangFormat."""
import unittest

import difflib
import os
import shutil
import subprocess

from io import open


@unittest.skipIf(
    shutil.which(os.environ.get('CLANG_FORMAT', 'clang-format')) is None,
    'clang-format is not installed -- skipping rather than failing, so an '
    'absent TOOL is visibly distinct from non-compliant SOURCES')
class TestClangFormat(unittest.TestCase):
    """Unit test for ClangFormat compliance."""

    def setUp(self):
        """Set up called before each test."""
        self.OMNISIM_HOME = os.path.normpath(os.environ['OMNISIM_HOME'])
        self.CLANG_FORMAT = os.environ.get('CLANG_FORMAT', 'clang-format')

    def _runClangFormat(self, f):
        """Run clang format on 'f' file."""
        return subprocess.check_output([self.CLANG_FORMAT, '-style=file', f])

    def test_clang_format_is_correctly_installed(self):
        """Test ClangFormat is correctly installed."""
        self.assertTrue(
            shutil.which(self.CLANG_FORMAT) is not None,
            msg='ClangFormat is not installed on this computer.'
        )
        clangFormatConfigFile = os.path.join(self.OMNISIM_HOME, '.clang-format')
        self.assertTrue(
            os.path.exists(clangFormatConfigFile),
            msg=clangFormatConfigFile + ' not found.'
        )

    def test_sources_are_clang_format_compliant(self):
        """Test that sources are ClangFormat compliant."""
        directories = [
            'include/controller',
            'projects',
            'resources/projects',
            'tests',
            'src/controller/c',
            'src/controller/cpp',
            'src/controller/launcher',
            'src/license/sign',
            'src/omnisim'
        ]
        skippedPaths = [
            'src/omnisim/external'
        ]
        skippedFiles = [
            'src/controller/c/sha1.c',
            'src/controller/c/sha1.h'
        ]
        skippedDirectories = [
            'build',
            'python',
            'java'
        ]
        skippedPathsFull = [os.path.join(self.OMNISIM_HOME, os.path.normpath(path)) for path in skippedPaths]
        skippedFilesFull = [os.path.join(self.OMNISIM_HOME, os.path.normpath(file)) for file in skippedFiles]

        extensions = ['c', 'h', 'cpp', 'hpp', 'cc', 'hh', 'c++', 'h++', 'vert', 'frag']
        modified_files = os.path.join(self.OMNISIM_HOME, 'tests', 'sources', 'modified_files.txt')
        sources = []
        if os.path.isfile(modified_files):
            with open(modified_files, 'r') as file:
                for line in file:
                    line = line.strip()
                    extension = os.path.splitext(line)[1][1:].lower()
                    if extension not in extensions:
                        continue
                    found = False
                    for directory in directories:
                        if line.startswith(directory):
                            found = True
                            break
                    if not found:
                        continue
                    found = False
                    for directory in skippedPaths + skippedFiles:
                        if line.startswith(directory):
                            found = True
                            break
                    if found:
                        continue
                    for directory in skippedDirectories:
                        currentDirectories = line.split(os.sep)
                        if directory in currentDirectories:
                            found = True
                    if found:
                        continue
                    sources.append(os.path.normpath(line))
        else:
            for directory in directories:
                path = os.path.join(self.OMNISIM_HOME, os.path.normpath(directory))
                for rootPath, dirNames, fileNames in os.walk(path):
                    shouldContinue = False
                    for skippedPath in skippedPathsFull:
                        if rootPath.startswith(skippedPath):
                            shouldContinue = True
                            break
                    for directory in skippedDirectories:
                        currentDirectories = rootPath.replace(self.OMNISIM_HOME + os.sep, '').split(os.sep)
                        if directory in currentDirectories:
                            shouldContinue = True
                            break
                    if shouldContinue:
                        continue
                    for fileName in fileNames:
                        extension = os.path.splitext(fileName)[1][1:].lower()
                        if extension not in extensions:
                            continue
                        path = os.path.normpath(os.path.join(rootPath, fileName))
                        if path not in skippedFilesFull:
                            sources.append(path)
        curdir = os.getcwd()
        os.chdir(self.OMNISIM_HOME)
        for source in sources:
            diff = ''
            with open(source, encoding='utf8') as file:
                try:
                    for line in difflib.context_diff(self._runClangFormat(source).decode('utf-8').splitlines(),
                                                     file.read().splitlines()):
                        diff += line + '\n'
                except UnicodeDecodeError:
                    self.assertTrue(False, msg='utf-8 decode problem in %s' % source)
                self.assertTrue(
                    len(diff) == 0,
                    msg='Source file "%s" is not compliant with ClangFormat:\n\nDIFF:%s' % (source, diff)
                )
        os.chdir(curdir)


if __name__ == '__main__':
    unittest.main()
