/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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
 *
 * Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
 */

// Description:  This is the source code of omnisim.exe and omnisimw.exe, the binary launchers of omnisim-bin.exe
//               (Windows only). The legacy names webots.exe and webotsw.exe are shipped as byte-identical copies.
//               The launcher adds paths to the PATH environment variable to ensure that the correct libraries will
//               be found, starts omnisim-bin.exe passing all the command line arguments, waits until the completion
//               of omnisim-bin.exe and returns its exit status code.
//               The main advantage of a binary launcher over a batch launcher is that the binary launcher
//               doesn't open a DOS cmd.exe console in the background, whereas this is unavoidable when running a
//               batch file.
//               omnisimw.exe is a windows application (to be started from the icon or menu).
//               omnisim.exe is a DOS application (to be started from a DOS command prompt or a script).
//               Starting omnisim.exe from the icon is fine, but will open a DOS command prompt in the background.
//               (a similar naming convention is used for python.exe / pythonw.exe, java.exe / javaw.exe, etc.)
//               The launcher is filename-agnostic: it locates its own directory at runtime, so the same binary
//               works under any of the four shipped names.

#include <shlwapi.h>
#include <stdio.h>
#include <string.h>
#include <windows.h>

static int fail(const char *function, const char *info) {
  DWORD e = GetLastError();
  if (e) {
    LPSTR m = NULL;
    FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, e,
                  MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US), (LPSTR)&m, 0, NULL);
    char message[1024];
    const char *lf = info ? "\n" : "";
    const char *i = info ? info : "";
    // cppcheck-suppress nullPointer
    snprintf(message, sizeof(message), "%s failed with error %lu.\n%s%s%s", function, e, m, i, lf);
    LocalFree(m);
#ifdef WEBOTSW
    MessageBox(NULL, message, "OmniSim launcher error", MB_ICONERROR | MB_OK);
#else
    fprintf(stderr, message);
#endif
  } else
    fprintf(stderr, "%s failed with no error.\n", function);
  exit(e);
}

int main(int argc, char *argv[]) {
  // We retrieve the command line in wchar_t from the Windows system.
  const int LENGTH = 4096;
  wchar_t *module_path = malloc(LENGTH * sizeof(wchar_t));
  if (!GetModuleFileNameW(NULL, module_path, LENGTH))
    fail("GetModuleFileNameW", 0);
  const wchar_t *last_slash = wcsrchr(module_path, L'\\');
  if (!last_slash)
    fail("wcsrchr", "The launcher path has no directory component.");
  const int dir_len = (int)(last_slash - module_path);
  wchar_t *dir = malloc(LENGTH * sizeof(wchar_t));
  wcsncpy(dir, module_path, dir_len);
  dir[dir_len] = L'\0';
  free(module_path);

  // Launch the canonical binary omnisim-bin.exe from the launcher's own directory.
  wchar_t *command_line = malloc(LENGTH * sizeof(wchar_t));
  wcscpy(command_line, dir);
  wcscat(command_line, L"\\omnisim-bin.exe");
  const wchar_t *arguments = PathGetArgsW(GetCommandLineW());
  if (arguments && arguments[0] != L'\0') {
    wcscat(command_line, L" ");
    wcscat(command_line, arguments);
  }

  // add "<dir>", "<dir>\cpp" and "<msys64 root>\usr\bin" to the PATH environment variable
  wchar_t *old_path = malloc(LENGTH * sizeof(wchar_t));
  wchar_t *new_path = malloc(LENGTH * sizeof(wchar_t));
  wcscpy(new_path, dir);
  wcscat(new_path, L";");
  wcscat(new_path, dir);
  wcscat(new_path, L"\\cpp;");
  const wchar_t *tail = L"mingw64\\bin";
  const int tail_len = (int)wcslen(tail);
  if (dir_len > tail_len && _wcsnicmp(dir + dir_len - tail_len, tail, tail_len) == 0) {
    // dir is <msys64 root>\mingw64\bin -- also expose <msys64 root>\usr\bin
    wcsncat(new_path, dir, dir_len - tail_len);  // keeps the trailing backslash
    wcscat(new_path, L"usr\\bin;");
  }
  free(dir);
  if (!GetEnvironmentVariableW(L"PATH", old_path, LENGTH))
    fail("GetEnvironmentVariableW", "PATH");
  wcscat(new_path, old_path);
  free(old_path);
  if (!SetEnvironmentVariableW(L"PATH", new_path))
    fail("SetEnvironmentVariableW", "PATH");
  free(new_path);
  if (!SetEnvironmentVariableW(L"QT_ENABLE_HIGHDPI_SCALING", L"1"))
    fail("SetEnvironmentVariableW", "QT_ENABLE_HIGHDPI_SCALING=1");

  // if set, we need to remove this environment variable set by Qt5 which conflicts with Qt6
  SetEnvironmentVariableW(L"QT_QPA_PLATFORM_PLUGIN_PATH", NULL);

  // start the omnisim-bin.exe process, wait for completion and return exit code
  STARTUPINFOW info = {sizeof(info)};
  PROCESS_INFORMATION process_info;

  while (1) {
    if (!CreateProcessW(NULL, command_line, NULL, NULL, TRUE, 0, NULL, NULL, &info, &process_info))
      fail("CreateProcess", "Cannot launch OmniSim binary");

    // omnisim-bin.exe should be killed whenever its parent launcher terminates.
    HANDLE job = CreateJobObject(NULL, NULL);
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION jeli = {0};
    jeli.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    if (!SetInformationJobObject(job, JobObjectExtendedLimitInformation, &jeli, sizeof(jeli)))
      fail("SetInformationJobObject", 0);
    if (!AssignProcessToJobObject(job, process_info.hProcess))
      fail("AssignProcessToJobObject", 0);

    // wait for omnisim-bin.exe to terminate
    WaitForSingleObject(process_info.hProcess, INFINITE);  // return zero in case of success
    DWORD exit_code;
    if (!GetExitCodeProcess(process_info.hProcess, &exit_code))
      fail("GetExitCodeProcess", 0);
    if (!CloseHandle(process_info.hProcess))
      fail("CloseHandle", 0);
    if (!CloseHandle(process_info.hThread))
      fail("CloseHandle", 0);
    if (exit_code != 3030)  // special return code to restart OmniSim, see OmGuiApplication.cpp
      return exit_code;
  }
  free(command_line);
  return 0;
}
