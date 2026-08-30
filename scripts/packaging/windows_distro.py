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

"""Generate Windows OmniSim package."""

from generic_distro import OmniSimPackage, print_error_message_and_exit
import ctypes
import datetime
import os
import subprocess


def convert_to_windows_path_separator(path):
    return path.replace('/', '\\')


def list_dependencies(package):
    return subprocess.check_output(['pactree', '-u', package]).decode().strip().split('\n')


class WindowsOmniSimPackage(OmniSimPackage):
    def __init__(self, package_name):
        super().__init__(package_name)
        self.application_file_path = self.application_name_lowercase_and_dashes + '.iss'
        self.msys64_files = []

    def create_omnisim_bundle(self, include_commit_file):
        super().create_omnisim_bundle(include_commit_file)

        self.add_folder_recursively(os.path.join(self.omnisim_home, 'msys64'))
        self.check_newton_runtime_bundle()
        self.check_wgpu_native_bundle()

        print('creating ISS descriptor')

        self.iss_script = open(self.application_file_path, 'w')
        self.iss_script.write(
          "[Setup]\n"
          "SourceDir=..\\..\n"
          # AppId drives the uninstall registry key (<AppId>_is1). It MUST be the
          # OmniSim id: keying it on "Webots" made the installer's pre-install check
          # below match an *upstream Webots* installation and silently uninstall it.
          "AppId=OmniSim\n"
          f"AppName={self.application_name}\n"
          f"AppVersion={self.full_version}\n"
          f"AppVerName={self.application_name} {self.full_version}\n"
          f"AppCopyright=Copyright (c) {datetime.date.today().year} OmniLink. "
          "Built on Webots (c) Cyberbotics Ltd. Apache-2.0.\n"
          "AppPublisher=OmniLink\n"
          "AppPublisherURL=https://www.omnilink-agents.com\n"
          # tells Windows Explorer to reload environment variables (e.g., OMNISIM_HOME)
          "ChangesEnvironment=yes\n"
          "Compression=lzma2/fast\n"
          "DefaultDirName={autopf}\\" + self.application_name + "\n"
          "DefaultGroupName=OmniLink\n"
          "UninstallDisplayIcon={app}\\msys64\\mingw64\\bin\\omnisim-bin.exe\n"
          "PrivilegesRequired=admin\n"
          "UsePreviousPrivileges=no\n"
          "PrivilegesRequiredOverridesAllowed=dialog commandline\n"
          f"OutputBaseFileName={self.application_name_lowercase_and_dashes}-{self.package_version}_setup\n"
          f"OutputDir={convert_to_windows_path_separator(self.distribution_path)}\n"
          "ChangesAssociations=yes\n"
          "DisableStartupPrompt=yes\n"
          "ArchitecturesInstallIn64BitMode=x64\n"
          "ArchitecturesAllowed=x64\n"
          "UsePreviousAppDir=yes\n"
          "\n[Dirs]\n"
        )

        # add directories
        print('  adding folders')
        for dir in self.package_folders:
            self.make_dir(dir)
        self.copy_msys64_dependencies()

        # add files
        print('  adding files')
        self.iss_script.write('\n[Files]\n')
        for file in self.package_files:
            self.copy_file(file)
        self.copy_msys64_files()

        self.iss_script.write(
            "\n[Icons]\n"
            "Name: \"{app}\\" + self.application_name + "\"; Filename: \"{app}\\msys64\\mingw64\\bin\\omnisimw.exe\"; "
            "WorkingDir: \"{app}\"; Comment: \"Robot simulator\"\n"
            "Name: \"{group}\\" + self.application_name + "\"; Filename: \"{app}\\msys64\\mingw64\\bin\\omnisimw.exe\"; "
            "WorkingDir: \"{app}\"; Comment: \"Robot simulator\"\n"
            "Name: \"{userdesktop}\\" + self.application_name + "\"; Filename: \"{app}\\msys64\\mingw64\\bin\\omnisimw.exe\"; "
            "WorkingDir: \"{app}""\"; Comment: \"Robot simulator\"\n"
            "Name: \"{group}\\Uninstall " + self.application_name + "\"; Filename: \"{uninstallexe}\"; WorkingDir: \"{app}\"; "
            "Comment: \"Uninstall " + self.application_name + "\"\n"
            # ProgID is `omnisimfile`: it is what Explorer surfaces in "Open with" and
            # the file-type description, so it must carry the OmniSim brand. BOTH world
            # extensions are registered: `.omniworld` is what OmniSim writes, `.wbt` the
            # legacy one it still reads (dual-read, single-write).
            "\n[Registry]\n"
            "Root: HKA; SubKey: \"Software\\Classes\\.omniworld\"; ValueType: string; ValueData: \"omnisimfile\"; "
            "Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\.omniworld\"; ValueType: string; ValueName: \"Content Type\"; ValueData: "
            "\"application/omnisimfile\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\.wbt\"; ValueType: string; ValueData: \"omnisimfile\"; "
            "Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\.wbt\"; ValueType: string; ValueName: \"Content Type\"; ValueData: "
            "\"application/omnisimfile\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\omnisimfile\"; ValueType: string; ValueData: "
            "\"OmniSim world\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\omnisimfile\\DefaultIcon\"; ValueType: string; ValueData: "
            "\"{app}\\resources\\icons\\core\\omnisim_doc.ico\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\omnisimfile\\shell\\open\"; ValueType: string; ValueName: "
            "\"FriendlyAppName\"; ValueData: \"OmniSim\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\omnisimfile\\shell\\open\\command\"; ValueType: string; ValueData: "
            "\"\"\"{app}\\msys64\\mingw64\\bin\\omnisimw.exe\"\" \"\"%1\"\"\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\Applications\\omnisimw.exe\"; ValueType: string; "
            "ValueName: \"SupportedTypes\"; ValueData: \".omniworld;.wbt\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\Applications\\omnisimw.exe\"; ValueType: string; "
            "ValueName: \"FriendlyAppName\"; ValueData: \"OmniSim\"; Flags: uninsdeletekey\n"
            "Root: HKCU; SubKey: \"Software\\OmniLink\"; Flags: uninsdeletekeyifempty dontcreatekey\n"
            f"Root: HKCU; SubKey: \"Software\\OmniLink\\{self.application_name} {self.full_version}\"; "
            "Flags: uninsdeletekey dontcreatekey\n"
            "Root: HKA; SubKey: \"SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Environment\"; ValueType: string; "
            "ValueName: \"OMNISIM_HOME\"; ValueData: \"{app}\"; Flags: preservestringtype\n"
            # On some systems (as already reported by two Chinese users), some unknown third party software badly installs a
            # zlib1.dll and libeay32.dll in the C:\Windows\System32 folder.
            # Similarly, libjpeg-8.dll may be found there.
            # This is a very bad practise as such DLLs conflicts with the same DLLs provided in the msys64 folder of OmniSim.
            # So, we will delete any of these libraries from the C:\Windows\System32 folder before installing OmniSim.
            "\n[InstallDelete]\n"
            "Type: files; Name: \"{sys}\\zlib1.dll\"\n"
            "Type: files; Name: \"{sys}\\libeay32.dll\"\n"
            "Type: files; Name: \"{sys}\\libjpeg-8.dll\"\n"
            "\n[Code]\n"
            "function InitializeSetup(): Boolean;\n"
            "var\n"
            "  ResultCode: Integer;\n"
            "  Uninstall: String;\n"
            "begin\n"
            "  if isAdmin and RegQueryStringValue(HKLM, 'Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\"
            "OmniSim_is1', 'UninstallString', Uninstall) then begin\n"
            "    if MsgBox('A version of OmniSim is already installed for all users on this computer. "
            "It will be removed and replaced by the version you are installing.', mbInformation, MB_OKCANCEL) = IDOK "
            "then begin\n"
            "      Exec(RemoveQuotes(Uninstall), ' /SILENT', '', SW_SHOWNORMAL, ewWaitUntilTerminated, ResultCode);\n"
            "      Result := TRUE;\n"
            "    end else begin\n"
            "      Result := FALSE;\n"
            "    end;\n"
            "  end else begin\n"
            "    Result := TRUE;\n"
            "  end;\n"
            "  if RegQueryStringValue(HKCU, 'Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\"
            "OmniSim_is1', 'UninstallString', Uninstall) then begin\n"
            "    if MsgBox('A version of OmniSim is already installed for the current user on this computer. It "
            "will be removed and replaced by the version you are installing.', mbInformation, MB_OKCANCEL) = IDOK "
            "then begin\n"
            "      Exec(RemoveQuotes(Uninstall), ' /SILENT', '', SW_SHOWNORMAL, ewWaitUntilTerminated, ResultCode);\n"
            "      Result := TRUE;\n"
            "    end else begin\n"
            "      Result := FALSE;\n"
            "    end;\n"
            "  end else begin\n"
            "    Result := TRUE;\n"
            "  end;\n"
            "  if RegQueryStringValue(HKLM32, 'Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\OmniSim_is1', "
            "'UninstallString', Uninstall) then begin\n"
            "    if MsgBox('A version of OmniSim (32 bit) is already installed on this computer. It will be removed "
            "and replaced by the version (64 bit) you are installing.', mbInformation, MB_OKCANCEL) = IDOK then begin\n"
            "      Exec(RemoveQuotes(Uninstall), ' /SILENT', '', SW_SHOWNORMAL, ewWaitUntilTerminated, ResultCode);\n"
            "      Result := TRUE;\n"
            "    end else begin\n"
            "      Result := FALSE;\n"
            "    end;\n"
            "  end else begin\n"
            "    Result := TRUE;\n"
            "  end;\n"
            "end;\n\n"
            "procedure CurUninstallStepChanged(CurUninstallStep: TUninstallStep);\n"
            "var\n"
            "  ResultCode: Integer;\n"
            "begin\n"
            "  if (CurUninstallStep = usPostUninstall) and DirExists(ExpandConstant('{app}')) then begin\n"
            "    if MsgBox(ExpandConstant('{app}') + ' was modified!'#13#10#13#10 +\n"
            "        'It seems you created or modified some files in this folder.'#13#10#13#10 +\n"
            "        'This is your last chance to do a backup of these files.'#13#10#13#10 +\n"
            "        'Do you want to delete the whole '+ ExpandConstant('{app}') +' folder now?'#13#10, mbConfirmation, "
            "MB_YESNO) = IDYES\n"
            "    then begin  // User clicked YES\n"
            "      // fix read-only status of all files and folders to be able to delete them\n"
            "      Exec('cmd.exe', '/c \"attrib -R ' + ExpandConstant('{app}') + '\\*.* /s /d\"', '', SW_HIDE, "
            "ewWaitUntilTerminated, ResultCode);\n"
            "      DelTree(ExpandConstant('{app}'), True, True, True);\n"
            "    end else begin  // User clicked NO\n"
            "      Abort;\n"
            "    end;\n"
            "  end;\n"
            "end;\n\n"
            "\n[Run]\n"
            "Filename: {app}\\msys64\\mingw64\\bin\\omnisimw.exe; Description: \"Launch OmniSim\"; Flags: nowait postinstall "
            "skipifsilent\n"
            )

        self.iss_script.close()

        if 'INNO_SETUP_HOME' in os.environ:
            INNO_SETUP_HOME = os.getenv('INNO_SETUP_HOME')
        else:
            INNO_SETUP_HOME = "/C/Program Files (x86)/Inno Setup 6"
        # `self.application_file_path` is derived from the application name
        # ("OmniSim" -> "omnisim.iss"); it is the script we just WROTE, so it is also
        # the one to compile. (This used to be a hardcoded "webots.iss", which no
        # longer exists after the rebrand -- the installer build could not succeed.)
        print(f"creating {self.application_name_lowercase_and_dashes}-{self.package_version}_setup.exe (takes long)\n")
        subprocess.run(
            [INNO_SETUP_HOME + '/iscc', '-Q', self.application_file_path]
        ).check_returncode()

        print('Done.')

    def check_wgpu_native_bundle(self):
        """Report whether wgpu_native.dll -- the ONLY renderer since the WREN
        deletion -- is staged next to the binary. Like the Newton bundle it rides
        the recursive msys64/ copy, so its presence at packaging time is the whole
        question. Without it a clean install simulates and shows a white viewport
        with '[render] wgpu-native is UNAVAILABLE' on every world (public issue
        #8: v8.1.5-v8.1.9 all shipped this way, because the release workflow never
        ran scripts/dev/setup_wgpu_native.sh). Set OMNISIM_REQUIRE_RENDERER_BUNDLE=1
        for every public package so that state is fatal rather than a warning."""
        bin_dir = os.path.join(self.omnisim_home, 'msys64', 'mingw64', 'bin')
        dll = os.path.join(bin_dir, 'wgpu_native.dll')
        if os.path.isfile(dll):
            print('  \033[1;32mwgpu-native bundled (wgpu_native.dll next to the binary) -- '
                  'stock install will render\033[0m')
            return
        msg = ('wgpu_native.dll is NOT next to the binary (' + dll + '). This installer has '
               'NO renderer on a clean box: white viewport, no screenshots, no Camera devices. '
               'Run `bash scripts/dev/setup_wgpu_native.sh` and rebuild before packaging.')
        if os.environ.get('OMNISIM_REQUIRE_RENDERER_BUNDLE') == '1':
            print_error_message_and_exit(msg)
        print('  \033[1;33mWARNING: ' + msg + '\033[0m')

    def check_newton_runtime_bundle(self):
        """Report whether the Newton runtime (warp/newton + the _pth redirect) is
        staged next to the binary. The whole msys64/ tree ships recursively, so a
        staged bundle (scripts/packaging/bundle_newton_runtime.py) is captured
        automatically. Newton is the only physics backend: without the bundle a
        clean install can render but cannot simulate. Set
        OMNISIM_REQUIRE_NEWTON_BUNDLE=1 for every public package so that state is
        fatal rather than a shippable warning."""
        bin_dir = os.path.join(self.omnisim_home, 'msys64', 'mingw64', 'bin')
        warp_dir = os.path.join(bin_dir, 'newton-runtime', 'site-packages', 'warp')
        has_warp = os.path.isdir(warp_dir)
        listing = os.listdir(bin_dir) if os.path.isdir(bin_dir) else []
        has_pth = any(f.startswith('python3') and f.endswith('._pth') for f in listing)
        if has_warp and has_pth:
            print('  \033[1;32mNewton runtime bundled (warp/newton + _pth) -- '
                  'stock install will run Newton\033[0m')
            return
        missing = []
        if not has_warp:
            missing.append('newton-runtime/site-packages/warp')
        if not has_pth:
            missing.append('python3XX._pth')
        msg = ('Newton runtime NOT bundled (missing: ' + ', '.join(missing) + '). '
               'This installer has no working physics on a clean box. Run '
               '`make bundle-newton-runtime` before packaging for a Newton-capable '
               'release (default-flip-plan.md L6).')
        if os.environ.get('OMNISIM_REQUIRE_NEWTON_BUNDLE') == '1':
            print_error_message_and_exit(msg)
        print('  \033[1;33mWARNING: ' + msg + '\033[0m')

    def test_file(self, filename):
        if os.path.isabs(filename) or filename.startswith('$'):
            return   # ignore absolute file names
        if filename.find('*') != -1:
            return  # ignore wildcard filenames
        local_file_path = os.path.join('..', '..', filename)
        if not os.access(local_file_path, os.F_OK):
            print_error_message_and_exit(f"Missing file: {filename}")

    def make_dir(self, directory):
        win_directory = convert_to_windows_path_separator(directory)
        win_directory = win_directory.replace('\\\\', '\\')
        self.iss_script.write("Name: \"{app}\\" + win_directory + "\"\n")

    def copy_file(self, path):
        super().copy_file(path)

        dir_path = os.path.dirname(path)
        file_name = os.path.basename(path)
        file_details = os.path.splitext(file_name)
        file_extension = file_details[1]

        self.iss_script.write("Source: \"" + convert_to_windows_path_separator(path) + "\"; "
                              "DestDir: \"{app}\\" + convert_to_windows_path_separator(dir_path) + "\"")
        if file_name.startswith('.'):
            self.iss_script.write('; Attribs: hidden')
        if file_extension in ['.png', '.jpg']:
            self.iss_script.write('; Flags: nocompression')
        self.iss_script.write("\n")

    def compute_name_with_prefix_and_extension(self, basename, options):
        platform_independent = 'linux' not in options and 'windows' not in options and 'mac' not in options
        if platform_independent or 'windows' in options:
            if 'exe' in options:
                return basename + '.exe'
            if 'dll' in options:
                return basename + '.dll'
            return basename
        return ""

    def set_file_attribute(self, file, attribute):
        if attribute.lower() == 'hidden':
            flag = 0x02
        else:
            print_error_message_and_exit(f"Unknown file attribute: {attribute}")

        ret = ctypes.windll.kernel32.SetFileAttributesW(file, flag)
        if not ret:
            raise ctypes.WinError()

    def copy_msys64_dependencies(self):
        # list all the pacman dependencies needed by OmniSim, including sub-dependencies
        dependencies = list(set(  # use a set to make sure to avoid duplication
            list_dependencies('make') +
            list_dependencies('coreutils') +
            list_dependencies('mingw-w64-x86_64-gcc')
        ))

        # add specific folder dependencies needed by OmniSim
        folders = ['/tmp', '/mingw64', '/mingw64/bin', '/mingw64/bin/cpp',
                   '/mingw64/include',
                   '/mingw64/include/libssh',
                   '/mingw64/lib', '/mingw64/share',
                   '/mingw64/share/qt6', '/mingw64/share/qt6/plugins', '/mingw64/share/qt6/translations',
                   '/mingw64/share/qt6/plugins/imageformats', '/mingw64/share/qt6/plugins/platforms',
                   '/mingw64/share/qt6/plugins/tls', '/mingw64/share/qt6/plugins/styles']
        skip_paths = ['/usr/share/', '/mingw64/bin/zlib1.dll', '/mingw64/bin/libjpeg-8.dll']

        # add all the files and folders corresponding to the pacman dependencies
        for dependency in dependencies:
            print("  processing " + dependency, flush=True)
            for file in subprocess.check_output(['pacman', '-Qql', dependency]).decode().strip().split('\n'):
                skip = False
                for skip_path in skip_paths:
                    if file.startswith(skip_path):
                        skip = True
                        break
                if skip:
                    continue
                if not file.endswith('/'):
                    self.msys64_files.append(file)
                else:
                    folder = file.rstrip('/')
                    if folder not in folders:
                        folders.append(folder)

        for folder in folders:
            # remove initial '/' from folder otherwise absolute path cannot be joined
            self.make_dir(os.path.join('msys64', folder.lstrip('/')))

    def copy_msys64_files(self):
        # add the dependencies provided in the files_msys64.txt file
        root = subprocess.check_output(['cygpath', '-w', '/']).decode().strip().rstrip('\\')
        with open('files_msys64.txt', 'r') as file:
            for line in file:
                line = line.strip()
                if not line.startswith('#') and line:
                    if line in self.msys64_files:
                        print('  \033[1;31m' + line + ' is already included\033[0m')
                    else:
                        self.msys64_files.append(line)

        # automatically compute the dependencies of ffmpeg
        print("  processing ffmpeg dependencies (DLLs)", flush=True)
        for ffmpeg_dll in subprocess.check_output(['bash', 'ffmpeg_dependencies.sh'], shell=True).decode('utf-8').split():
            self.msys64_files.append('/mingw64/bin/' + ffmpeg_dll)

        # write every dependency file in the ISS file for files
        for file in self.msys64_files:
            file = file.replace('/', '\\')
            if file in ['\\mingw64\\bin\\libstdc++-6.dll',
                        '\\mingw64\\bin\\libgcc_s_seh-1.dll',
                        '\\mingw64\\bin\\libwinpthread-1.dll']:
                # The MinGW runtime the engine links. Upstream Webots shipped these
                # ONLY under bin\cpp and relied on its exe launcher prepending that
                # directory to PATH (launcher.c). Every other way of starting the
                # engine -- `omnisim.bat demo` (the conformance gate -> headless
                # runner), the HTTP harness, the capture service, the MCP server, a
                # bare omnisim-bin.exe -- spawns the binary directly, and on a
                # stock install Windows then raises three "DLL not found" dialogs
                # and exits 0xC0000135 before a log line is written. Public issue
                # #9 (v8.1.12): `doctor` READY, `demo` blocked. The dev tree never
                # showed it because the toolchain's copies sit in mingw64\bin
                # there. Ship them BESIDE the exe as well: the application
                # directory is first in the Windows DLL search order, so this
                # works whatever PATH the launcher did or did not set. bin\cpp is
                # kept for the launcher and the bundled compiler.
                self.iss_script.write('Source: "' + root + file + '"; '
                                      'DestDir: "{app}\\msys64' + os.path.dirname(file) + '"\n')
                self.iss_script.write('Source: "' + root + file + '"; '
                                      'DestDir: "{app}\\msys64' + os.path.dirname(file) + '\\cpp"\n')
            else:
                self.iss_script.write('Source: "' + root + file + '"; DestDir: "{app}\\msys64' + os.path.dirname(file) + '"\n')
