#!/usr/bin/env python

# Copyright 1996-2025 Cyberbotics Ltd.
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

"""Test and save all the released worlds to make sure they don't generate warnings."""
import unittest
import os
import fnmatch
import sys
import platform
from subprocess import Popen, PIPE, TimeoutExpired

if sys.version_info[0] != 3 or sys.version_info[1] < 7:
    sys.exit('This script requires Python version 3.7')

if platform.system() == 'Darwin':
    CACHE_DIR = os.path.join(os.path.expanduser('~'), 'Library', 'Caches', 'Cyberbotics', 'Webots', 'assets')
elif platform.system() == 'Linux':
    CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'Cyberbotics', 'Webots', 'assets')
elif platform.system() == 'Windows':
    CACHE_DIR = os.path.join(os.path.expanduser('~'), 'AppData', 'Local', 'Cyberbotics', 'Webots', 'cache', 'assets')


class TestWorldsWarnings(unittest.TestCase):
    """Unit test of the worlds."""

    def setUp(self):
        """Get all the worlds."""
        if 'OMNISIM_HOME' in os.environ:
            OMNISIM_HOME = os.path.normpath(os.environ['OMNISIM_HOME'])
        else:
            OMNISIM_HOME = ''
        self.crashError = '(core dumped) "$webotsHome/bin/omnisim-bin" "$@"'
        self.skippedMessages = [
            'AL lib: (WW) alc_initconfig: Failed to initialize backend "pulse"',
            # the ros controller of complete_test.wbt is started when loading the world because the robot-window is open
            'Failed to contact master at',
            'Cannot initialize the sound engine',
            self.crashError,  # To remove once #6125 is fixed
            # fontconfig tries to restore system font directory mtimes after scanning; fails without write permission on CI
            'Unable to revert mtime:'
        ]
        # Set empty.wbt as the first world (to trigger the 'System below the minimal requirements' message)
        _emptyDir = os.path.join(OMNISIM_HOME, 'resources', 'projects', 'worlds')
        _emptyCandidates = [os.path.join(_emptyDir, 'empty' + ext) for ext in ('.omniworld', '.wbt')]
        self.worlds = [next((c for c in _emptyCandidates if os.path.isfile(c)), _emptyCandidates[0])]
        # Get all the worlds from projects
        for directory in ['projects']:
            for rootPath, dirNames, fileNames in os.walk(os.path.join(OMNISIM_HOME, directory)):
                for fileName in [_f for _f in fileNames if _f.endswith(('.wbt', '.omniworld'))]:
                    world = os.path.join(rootPath, fileName)
                    self.worlds.append(world)
        # Prefer the canonical omnisim-bin; fall back to the legacy webots alias.
        self.webotsFullPath = None
        if sys.platform == 'win32':
            binDir = os.path.join(OMNISIM_HOME, 'msys64', 'mingw64', 'bin')
            candidates = [os.path.join(binDir, 'omnisim-bin.exe'), os.path.join(binDir, 'webots.exe')]
            self.webotsFullPath = next((c for c in candidates if os.path.isfile(c)), candidates[0])
        else:
            base = OMNISIM_HOME if 'OMNISIM_HOME' in os.environ else '..'
            candidates = [os.path.join(base, 'bin', 'omnisim-bin'), os.path.join(base, 'omnisim-bin'), os.path.join(base, 'webots')]
            self.webotsFullPath = next((c for c in candidates if os.path.isfile(c)), None)
            if self.webotsFullPath is None:
                print('Error: omnisim-bin binary not found')
                sys.exit(1)
            self.webotsFullPath = os.path.normpath(self.webotsFullPath)

    def test_worlds_warnings_and_cache(self):
        """Test all the 'projects' worlds for loading warnings and if they reference un-cached assets."""
        problematicWorlds = []
        worldWarnings = {}
        crashedWorlds = []
        worldsWithNonCachedAssets = []
        cacheSizeBefore = len(os.listdir(CACHE_DIR))
        for i in range(len(self.worlds)):
            print('Testing: %d/%d: %s' % (i + 1, len(self.worlds), self.worlds[i]))
            self.process = Popen([
                self.webotsFullPath,
                self.worlds[i],
                '--stdout',
                '--stderr',
                '--minimize',
                '--batch',
                '--mode=pause',
                '--update-world'
            ], stdin=PIPE, stdout=PIPE, stderr=PIPE, text=True)
            try:
                output, errors = self.process.communicate()
            except TimeoutExpired:
                self.process.kill()
                output, errors = self.process.communicate()

            # First world is empty.wbt, used to trigger the warning about system requirements not being met
            if i == 0:
                continue
            if errors:
                unwanted = [e for e in errors.splitlines() if not any(message in e for message in self.skippedMessages)]
                if unwanted:
                    problematicWorlds.append(self.worlds[i])
                    worldWarnings[self.worlds[i]] = unwanted
            if errors and self.crashError in str(errors):
                crashedWorlds.append(self.worlds[i])

            # url.wbt and camera.omniworld are exceptional as they contain non-cached assets on purpose, adapt cache size accordingly
            worldName = os.path.basename(self.worlds[i])
            if worldName == 'url.wbt':
                cacheSizeBefore += 22
            if worldName == 'camera.omniworld':
                cacheSizeBefore += 26

            cacheSizeAfter = len(os.listdir(CACHE_DIR))
            if cacheSizeAfter - cacheSizeBefore > 0:
                worldsWithNonCachedAssets.append(self.worlds[i])
                cacheSizeBefore = cacheSizeAfter  # update the counter so it doesn't trigger again for the same reason

        if crashedWorlds:
            print('\n\t'.join(['Impossible to test the following worlds because they crash when loading:'] + crashedWorlds))
        if problematicWorlds:
            details = ['The following worlds have unwanted warnings:']
            for world in problematicWorlds:
                details.append(f'{world} ({len(worldWarnings[world])} warning(s))')
                for warning in worldWarnings[world]:
                    details.append('  ' + warning)
            self.fail('\n\t'.join(details))

        if worldsWithNonCachedAssets:
            print('\nThe following worlds reference non-cached assets:')
            for world in worldsWithNonCachedAssets:
                print('- ' + world)

        if crashedWorlds or problematicWorlds or worldsWithNonCachedAssets:
            self.fail('Problematic worlds have been found.')


if __name__ == '__main__':
    unittest.main()
