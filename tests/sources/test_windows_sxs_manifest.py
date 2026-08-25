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

"""Guard the Windows side-by-side (SxS) assembly contract.

omnisim-bin.exe embeds [src/omnisim/gui/manifest.xml](../../src/omnisim/gui/manifest.xml),
which declares a <dependency> on a *private assembly*. Windows resolves that
dependency by looking, next to the executable, for a file named exactly
`<assembly-name>.manifest`. That file is shipped by
[dependencies/Makefile.windows](../../dependencies/Makefile.windows).

If the two names ever drift apart, the loader cannot resolve the assembly and
**refuses to start the executable at all** -- `WinError 14001, "the side-by-side
configuration is incorrect"`. There is no partial degradation and no warning at
build time: the link succeeds, the binary looks perfectly valid, and it simply
will not run.

This drift is easy to introduce, because the assembly name carries the vendor's
brand (it was `Cyberbotics.Webots.Mingw64.Libraries`, it is now
`OmniLink.OmniSim.Mingw64.Libraries`), so any rebrand sweep is tempted to touch
one file and not the other. This test pins the three things that must agree.
"""

import os
import re
import unittest
import xml.etree.ElementTree as ET

OMNISIM_HOME = os.environ.get('OMNISIM_HOME',
                              os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..')))

ASM_NS = '{urn:schemas-microsoft-com:asm.v1}'

EXE_MANIFEST = os.path.join(OMNISIM_HOME, 'src', 'omnisim', 'gui', 'manifest.xml')
DEPENDENCIES_DIR = os.path.join(OMNISIM_HOME, 'dependencies')
DEPENDENCIES_MAKEFILE = os.path.join(DEPENDENCIES_DIR, 'Makefile.windows')


def required_assembly_name():
    """The private assembly omnisim-bin.exe declares it depends on."""
    root = ET.parse(EXE_MANIFEST).getroot()
    dependency = root.find(f'{ASM_NS}dependency/{ASM_NS}dependentAssembly/{ASM_NS}assemblyIdentity')
    assert dependency is not None, f'{EXE_MANIFEST} declares no dependent assembly'
    return dependency.get('name')


class TestWindowsSxsManifest(unittest.TestCase):
    def test_the_declared_assembly_is_actually_provided(self):
        """The assembly the .exe asks for must exist, under exactly that filename."""
        name = required_assembly_name()
        provider = os.path.join(DEPENDENCIES_DIR, f'{name}.manifest')
        self.assertTrue(
            os.path.isfile(provider),
            f'omnisim-bin.exe depends on the private assembly "{name}", but no\n'
            f'"{name}.manifest" exists in dependencies/. Windows will refuse to start the\n'
            f'executable (WinError 14001). Rename the assembly manifest to match, or revert\n'
            f'the <dependency> name in {EXE_MANIFEST}.')

    def test_the_assembly_identifies_itself_by_the_same_name(self):
        """A manifest whose filename and internal assemblyIdentity disagree is not found."""
        name = required_assembly_name()
        provider = os.path.join(DEPENDENCIES_DIR, f'{name}.manifest')
        if not os.path.isfile(provider):
            self.skipTest('covered by test_the_declared_assembly_is_actually_provided')
        identity = ET.parse(provider).getroot().find(f'{ASM_NS}assemblyIdentity')
        self.assertIsNotNone(identity, f'{provider} declares no assemblyIdentity')
        self.assertEqual(
            identity.get('name'), name,
            f'{os.path.basename(provider)} is named for the assembly "{name}" but identifies\n'
            f'itself as "{identity.get("name")}". Windows matches on the identity, not the\n'
            f'filename, so the dependency would go unresolved.')

    def test_the_build_installs_it_next_to_the_binary(self):
        """Shipping the manifest in the source tree is useless if it is never installed."""
        name = required_assembly_name()
        with open(DEPENDENCIES_MAKEFILE, encoding='utf-8') as fh:
            makefile = fh.read()
        self.assertIn(
            f'{name}.manifest', makefile,
            f'dependencies/Makefile.windows never installs "{name}.manifest" next to the\n'
            f'binary, so a clean build produces an executable that cannot start.')

    def test_every_dll_the_assembly_declares_is_a_real_dependency(self):
        """The assembly's <file> entries are deliberately withheld from the plain bin copy.

        windows_distro.py skips these DLLs when it sweeps the MSYS2 tree, precisely because
        they are meant to arrive via the private assembly instead. If a DLL is listed here
        but not skipped there, it gets shipped twice and the SxS copy is shadowed.
        """
        name = required_assembly_name()
        provider = os.path.join(DEPENDENCIES_DIR, f'{name}.manifest')
        if not os.path.isfile(provider):
            self.skipTest('covered by test_the_declared_assembly_is_actually_provided')
        declared = {f.get('name') for f in ET.parse(provider).getroot().findall(f'{ASM_NS}file')}

        distro = os.path.join(OMNISIM_HOME, 'scripts', 'packaging', 'windows_distro.py')
        with open(distro, encoding='utf-8') as fh:
            skip_paths = re.search(r'skip_paths\s*=\s*\[([^\]]*)\]', fh.read())
        self.assertIsNotNone(skip_paths, 'windows_distro.py no longer defines skip_paths')
        skipped = skip_paths.group(1)

        for dll in sorted(declared):
            self.assertIn(
                dll, skipped,
                f'"{dll}" is provided by the {name} private assembly, but windows_distro.py\n'
                f'does not skip it when copying the MSYS2 tree -- it would be installed twice.')


if __name__ == '__main__':
    unittest.main()
