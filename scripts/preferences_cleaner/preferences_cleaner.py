#!/usr/bin/env python3

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

'''Cleanup all the OmniSim preferences (and the legacy Webots ones).'''

from __future__ import print_function  # To display a correct error message when running from Python 2.
import platform
import sys

# Due to the use of pathlib and winreg.
assert sys.version_info >= (3, 5), 'Python 3.5 or later is required to run this script.'


def cleanupLinuxPreferences():
    from pathlib import Path

    # (organisation directory, preferences file glob)
    # Current: OmniLink/OmniSim-*.conf. Legacy (pre-rebrand): Cyberbotics/Webots-*.conf.
    locations = [
        (Path.home() / '.config' / 'OmniLink', 'OmniSim-*.conf'),
        (Path.home() / '.config' / 'Cyberbotics', 'Webots-*.conf'),
    ]
    for preferencesDir, pattern in locations:
        for preferencesPath in preferencesDir.glob(pattern):
            print('Clearing the "%s" preferences...' % preferencesPath)
            preferencesPath = Path(preferencesPath)
            preferencesPath.unlink()


def cleanupMacOSPreferences():
    import subprocess
    from pathlib import Path

    preferencesDir = Path.home() / 'Library' / 'Preferences'
    # Current: com.omnilink.OmniSim-*.plist (org "OmniLink" -> lowercased domain).
    # Legacy (pre-rebrand): com.cyberbotics.*.plist.
    patterns = ['com.omnilink.OmniSim-*', 'com.cyberbotics.*']
    for pattern in patterns:
        for preferencesPath in preferencesDir.glob(pattern):
            preferencesPath = Path(preferencesPath)
            preferenceReference = preferencesPath.stem
            print('Clearing the "%s" preferences...' % preferenceReference)
            subprocess.run(['defaults', 'remove', preferenceReference])
            if preferencesPath.exists():
                preferencesPath.unlink()


def cleanupWindowsPreferences():
    import winreg

    def deleteKeyRecursively(key):
        k = winreg.OpenKey(winreg.HKEY_CURRENT_USER, key, 0, winreg.KEY_ALL_ACCESS)
        subKeys = []
        try:
            i = 0
            while True:
                subKeys.append(key + '\\' + winreg.EnumKey(k, i))
                i += 1
        except WindowsError:
            pass  # Reached the end of the key enum.
        winreg.CloseKey(k)
        for subKey in subKeys:
            deleteKeyRecursively(subKey)
        winreg.DeleteKey(winreg.HKEY_CURRENT_USER, key)

    # Current: Software\OmniLink. Legacy (pre-rebrand): Software\Cyberbotics.
    cleanedSomething = False
    for organisationKey in ('Software\\OmniLink', 'Software\\Cyberbotics'):
        try:
            print("Clearing the '%s' registry key..." % organisationKey)
            deleteKeyRecursively(organisationKey)
            cleanedSomething = True
        except FileNotFoundError:
            pass  # This organisation key is absent; try the next one.
        except PermissionError:
            print("You don't have the right to delete the '%s' registry key." % organisationKey, file=sys.stderr)
    if not cleanedSomething:
        print("Nothing to clean.")


if __name__ == "__main__":
    osName = platform.system()
    print('Cleaning-up OmniSim preferences on %s...' % osName)
    if osName == 'Linux':
        cleanupLinuxPreferences()
    elif osName == 'Darwin':
        cleanupMacOSPreferences()
    elif osName == 'Windows':
        cleanupWindowsPreferences()
    else:
        sys.exit('Unsupported OS: ' + osName)
    print('Done.')
