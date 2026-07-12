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

"""Test header version."""
import unittest
import os
import fnmatch
import subprocess

IGNORED_PROTOS = [
    'tests/protos/protos/Bc21bCameraProto.proto',
    'resources/templates/protos/template.proto'
]

IGNORED_WORLDS = [
    'tests/protos/worlds/backward_compability_enu_flu.wbt',
    'tests/cache/worlds/backwards_compatibility.wbt'
]

SKIPPED_DIRECTORIES = [
    'dependencies',
    'distribution',
    '.git'
]


class TestHeaderVersion(unittest.TestCase):
    """Unit test of the PROTO and world headers."""

    def setUp(self):
        OMNISIM_HOME = os.path.normpath(os.environ['OMNISIM_HOME'])

        """Get all the PROTO files to be tested."""
        ignoredWorldsFull = [os.path.join(OMNISIM_HOME, os.path.normpath(file)) for file in IGNORED_WORLDS]
        ignoredProtosFull = [os.path.join(OMNISIM_HOME, os.path.normpath(file)) for file in IGNORED_PROTOS]

        # 1. Get Webots version (without revision)
        self.version = None
        with open(os.path.join(OMNISIM_HOME, 'resources', 'version.txt')) as file:
            content = file.read()
            self.version = content.splitlines()[0].strip().split()[0]
        # 2. Get all the PROTO files
        self.files = []
        for rootPath, dirNames, fileNames in os.walk(OMNISIM_HOME):
            dirNames[:] = [d for d in dirNames if d not in SKIPPED_DIRECTORIES]
            for fileName in fnmatch.filter(fileNames, '*.proto'):
                proto = os.path.join(rootPath, fileName)
                if proto not in ignoredProtosFull:
                    self.files.append((proto, '#VRML_SIM %s utf8' % self.version))
        # 3. Get all the world files
        #
        # Only worlds git actually tracks. This walks the filesystem, so without this
        # check it also grades build output and tool scratch files -- e.g. the
        # `.harness__*.wbt` scratch worlds the validation harness writes next to a
        # world it loads. Those are gitignored, never ship, and are not ours to
        # format; grading them turns this guard into noise, and a guard that cries
        # wolf is a guard nobody reads.
        for rootPath, dirNames, fileNames in os.walk(OMNISIM_HOME):
            dirNames[:] = [d for d in dirNames if d not in SKIPPED_DIRECTORIES]
            for fileName in fnmatch.filter(fileNames, '*.wbt'):
                world = os.path.join(rootPath, fileName)
                if world in ignoredWorldsFull:
                    continue
                if subprocess.run(['git', 'check-ignore', '-q', world]).returncode == 0:
                    continue
                self.files.append((world, '#VRML_SIM %s utf8' % self.version))
        # 4. Get all the .wbproj files
        for rootPath, dirNames, fileNames in os.walk(OMNISIM_HOME):
            dirNames[:] = [d for d in dirNames if d not in SKIPPED_DIRECTORIES]
            for fileName in fnmatch.filter(fileNames, '*.wbproj'):
                projFile = os.path.join(rootPath, fileName)
                # Only check files that aren't ignored by git
                if subprocess.run(['git', 'check-ignore', '-q', projFile]).returncode != 0:
                    self.files.append((projFile, 'OmniSim Project File version %s' % self.version))

    def test_header_version(self):
        """Test that the PROTO and world files have the correct header."""
        for currentFile in self.files:
            fileToTest = currentFile[0]
            # encoding is not optional: Python defaults to the locale codec, which on a
            # Windows checkout is cp1252 -- and any non-ASCII byte in any world/PROTO then
            # raises UnicodeDecodeError here, so this guard never runs at all.
            with open(fileToTest, encoding='utf-8') as file:
                content = file.read()
                if content == '':
                    continue
                line = content.splitlines()[0].strip()
                self.assertTrue(
                    line.startswith(currentFile[1]),
                    msg='Wrong header in file: "%s"' % fileToTest
                )


if __name__ == '__main__':
    unittest.main()
