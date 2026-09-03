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

"""Test quality of the source code using Cppcheck."""
import unittest
import os
import multiprocessing
import shutil
import sys


class TestCppCheck(unittest.TestCase):
    """Unit test for CppCheck errors."""

    def setUp(self):
        """Set up called before each test."""
        self.OMNISIM_HOME = os.path.normpath(os.environ['OMNISIM_HOME'])
        self.reportFilename = os.path.join(self.OMNISIM_HOME, 'tests', 'cppcheck_report.txt')
        self.extensions = ['c', 'h', 'cpp', 'hpp', 'cc', 'hh', 'c++', 'h++']
        if (sys.platform.startswith('linux')):
            self.platformOptions = ' -D__linux__'
        elif (sys.platform.startswith('win32')):
            self.platformOptions = ' -D_WIN32'
        else:
            self.platformOptions = ' -D__APPLE__'

        with open(os.path.join(self.OMNISIM_HOME, 'resources', 'version.txt'), 'r') as file:
            version = file.readlines()[0].strip()
            self.platformOptions += ' -DLIBCONTROLLER_VERSION=' + version

    @unittest.skipIf(shutil.which('cppcheck') is None, 'cppcheck is not installed')
    def test_cppcheck_is_correctly_installed(self):
        """Test Cppcheck is correctly installed."""
        self.assertTrue(
            shutil.which('cppcheck') is not None,
            msg='Cppcheck is not installed on this computer.'
        )

    def run_cppcheck(self, command):
        """Run Cppcheck command and check for errors.

        ⚠ THIS USED TO PASS WITHOUT CHECKING ANYTHING. `os.system` returned an
        error code nobody read, and the verdict hung off
        `if os.path.isfile(reportFilename)` -- so with cppcheck absent the
        command failed silently, no report was written, the branch was skipped
        and the test reported success having analysed zero files. A green that
        cannot go red is worse than no test, so the report is now REQUIRED:
        a missing cppcheck skips loudly, and a cppcheck that ran but produced
        no report fails.
        """
        if shutil.which('cppcheck') is None:
            self.skipTest('cppcheck is not installed -- refusing to report a '
                          'pass for an analysis that did not run')
        curdir = os.getcwd()
        os.chdir(self.OMNISIM_HOME)
        try:
            if os.path.isfile(self.reportFilename):
                os.remove(self.reportFilename)
            # warning: on Windows, the length of command is limited to 8192 characters
            os.system(command)
            self.assertTrue(
                os.path.isfile(self.reportFilename),
                msg='Cppcheck produced no report (%s). The analysis did not '
                    'run, so this test cannot vouch for the sources.'
                    % self.reportFilename
            )
            with open(self.reportFilename, 'r') as reportFile:
                reportText = reportFile.read()
            self.assertTrue(
                not reportText,
                msg='Cppcheck detected some errors:\n\n%s' % reportText
            )
            os.remove(self.reportFilename)
        finally:
            os.chdir(curdir)

    def add_source_files(self, sourceDirs, skippedDirs, skippedFiles=[]):
        command = ''
        modified_files = os.path.join(self.OMNISIM_HOME, 'tests', 'sources', 'modified_files.txt')
        if os.path.isfile(modified_files):
            with open(modified_files, 'r') as file:
                for line in file:
                    line = line.strip()
                    extension = os.path.splitext(line)[1][1:].lower()
                    if extension not in self.extensions:
                        continue
                    for sourceDir in sourceDirs:
                        if line.startswith(sourceDir):
                            shouldSkip = False
                            for skipped in skippedDirs + skippedFiles:
                                if line.startswith(skipped):
                                    shouldSkip = True
                                    break
                            if not shouldSkip:
                                command += ' \"' + line + '\"'
                            continue
            for source in skippedFiles:
                command += ' --suppress=\"*:' + source + '\"'
        else:
            for source in skippedFiles:
                command += ' --suppress=\"*:' + source + '\"'
            for source in skippedDirs:
                command += ' -i\"' + source + '\"'
            for source in sourceDirs:
                command += ' \"' + source + '\"'
        return command

    def test_sources_with_cppcheck(self):
        """Test Webots with Cppcheck."""
        sourceDirs = [
            'src/omnisim',
            'src/controller/c',
            'src/controller/cpp',
            'src/controller/launcher',
            'resources/projects'
        ]
        skippedDirs = [
            'src/omnisim/build',
            'src/omnisim/external',
            'resources/projects/libraries/qt_utils/build',
            'include/opencv2',
            'include/qt'
        ]
        includeDirs = [
            'include/controller/c',
            'include/glad',
            'src/omnisim/app',
            'src/omnisim/control',
            'src/omnisim/core',
            'src/omnisim/editor',
            'src/omnisim/engine',
            'src/omnisim/external',
            'src/omnisim/gui',
            'src/omnisim/license',
            'src/omnisim/maths',
            'src/omnisim/nodes',
            'src/omnisim/ode',
            'src/omnisim/plugins',
            'src/omnisim/scene_tree',
            'src/omnisim/sound',
            'src/omnisim/user_commands',
            'src/omnisim/util',
            'src/omnisim/vrml',
            'src/omnisim/widgets',
        ]
        skippedFiles = [
            'src/controller/c/sha1.c',
            'src/controller/c/sha1.h'
        ]
        if not sys.platform.startswith('win32'):
            skippedFiles.append('src/omnisim/core/OmWindowsRegistry.hpp')
        command = 'cppcheck --platform=native --enable=warning,style,performance,portability --inconclusive -q'
        command += self.platformOptions
        command += ' --library=qt -j %s' % str(multiprocessing.cpu_count())
        command += ' --inline-suppr --suppress=invalidPointerCast --suppress=useStlAlgorithm --suppress=uninitMemberVar'
        command += ' --suppress=noCopyConstructor --suppress=noOperatorEq --suppress=strdupCalled --suppress=unknownMacro'
        command += ' --suppress=duplInheritedMember --suppress=constParameterCallback'
        command += ' --check-level=exhaustive' if os.environ.get('CI') else ' --suppress=normalCheckLevelMaxBranches'
        # command += ' --xml '  # Uncomment this line to get more information on the errors
        command += ' --output-file=\"' + self.reportFilename + '\"'
        for include in includeDirs:
            command += ' -I\"' + include + '\"'
        sources = self.add_source_files(sourceDirs, skippedDirs, skippedFiles)
        if not sources:
            return
        command += sources
        self.run_cppcheck(command)

    def test_projects_with_cppcheck(self):
        """Test projects with Cppcheck."""
        sourceDirs = [
            'projects/default',
            'projects/devices',
            'projects/languages',
            'projects/objects',
            'projects/robots',
            'projects/samples',
        ]
        skippedDirs = [
            'projects/robots/nex/plugins/robot_windows/fire_bird_6_window/build',
        ]
        skippedFiles = []
        command = 'cppcheck --platform=native --enable=warning,style,performance,portability --inconclusive -q'
        command += self.platformOptions
        command += ' --library=qt --inline-suppr --suppress=invalidPointerCast --suppress=useStlAlgorithm -UKROS_COMPILATION'
        command += ' --suppress=strdupCalled --suppress=ctuOneDefinitionRuleViolation --suppress=unknownMacro'
        command += ' --suppress=duplInheritedMember --suppress=constParameterCallback'
        command += ' --check-level=exhaustive' if os.environ.get('CI') else ' --suppress=normalCheckLevelMaxBranches'
        # command += ' --xml'  # Uncomment this line to get more information on the errors
        command += ' --std=c++03 --output-file=\"' + self.reportFilename + '\"'
        sources = self.add_source_files(sourceDirs, skippedDirs, skippedFiles)
        if not sources:
            return
        command += sources
        self.run_cppcheck(command)


if __name__ == '__main__':
    unittest.main()
