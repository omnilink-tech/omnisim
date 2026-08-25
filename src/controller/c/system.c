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

#include <assert.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>  // strlen
#include <sys/stat.h>

#include <omnisim/utils/system.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

#ifdef _WIN32
static char *buffer = NULL;

static void free_buffer() {
  free(buffer);
}
#endif

const char *wbu_system_getenv(const char *variable) {
#ifdef _WIN32
  wchar_t *value;
  size_t size = strlen(variable) + 1;
  wchar_t *wvariable = (wchar_t *)malloc(size * sizeof(wchar_t));
  MultiByteToWideChar(CP_UTF8, 0, variable, -1, wvariable, size);
  size = GetEnvironmentVariableW(wvariable, NULL, 0);
  if (size == 0)
    return NULL;  // not defined
  value = (wchar_t *)malloc(size * sizeof(wchar_t));
  GetEnvironmentVariableW(wvariable, value, size);
  free(wvariable);
  size = WideCharToMultiByte(CP_UTF8, 0, value, -1, NULL, 0, NULL, NULL);
  if (buffer == NULL)
    atexit(free_buffer);
  else
    free(buffer);
  buffer = (char *)malloc(size);
  WideCharToMultiByte(CP_UTF8, 0, value, -1, buffer, size, NULL, NULL);
  free(value);
  return (const char *)buffer;
#else
  return getenv(variable);
#endif
}

const char *wbu_system_short_path(const char *path) {
#ifdef _WIN32
  int size = MultiByteToWideChar(CP_UTF8, 0, path, -1, NULL, 0);
  wchar_t *w_path = (wchar_t *)malloc(size * sizeof(wchar_t));
  MultiByteToWideChar(CP_UTF8, 0, path, -1, w_path, size);
  size = GetShortPathNameW(w_path, NULL, 0);
  wchar_t *w_short_path = (wchar_t *)malloc(size * sizeof(wchar_t));
  GetShortPathNameW(w_path, w_short_path, size);
  free(w_path);
  size = WideCharToMultiByte(CP_UTF8, 0, w_short_path, -1, NULL, 0, NULL, NULL);
  if (buffer == NULL)
    atexit(free_buffer);
  else
    free(buffer);
  buffer = (char *)malloc(size);
  WideCharToMultiByte(CP_UTF8, 0, w_short_path, -1, buffer, size, NULL, NULL);
  // cppcheck-suppress memleak
  return (const char *)buffer;
#else
  return path;
#endif
}

// Path to the per-instance tmp folder the OmniSim engine created for this run.
// Intern controllers only.
// The function NAME keeps its legacy spelling on purpose: it is exported from libController and
// declared in the public header include/controller/c/omnisim/utils/system.h, so renaming it is an
// ABI break for already-compiled external controllers. Ditto the folder's own on-disk name
// ("webots-<tmpId>"), which the engine writes -- see OmStandardPaths::webotsTmpPath().
const char *wbu_system_webots_instance_path(bool refresh) {
  static const char *cached_instance_path = NULL;
  if (cached_instance_path && !refresh)
    return cached_instance_path;
  // OMNISIM_INSTANCE_PATH is the only name read. The engine writes exactly this name
  // (OmController::setProcessEnvironment) and no longer writes the legacy WEBOTS_INSTANCE_PATH
  // twin, so the two halves of the rendezvous stay in lockstep. If only the legacy name is
  // present the engine binary predates the rename: say so instead of falling through to the
  // extern-controller path and hanging for 50 s with no explanation.
  cached_instance_path = getenv("OMNISIM_INSTANCE_PATH");
  if (cached_instance_path && cached_instance_path[0])
    return cached_instance_path;
  cached_instance_path = NULL;
  // once only: the caller polls this in a 50-iteration retry loop
  static bool warned_legacy_instance_path = false;
  const char *legacy = getenv("WEBOTS_INSTANCE_PATH");
  if (legacy && legacy[0] && !warned_legacy_instance_path) {
    warned_legacy_instance_path = true;
    fprintf(stderr,
            "Warning: WEBOTS_INSTANCE_PATH is set but OMNISIM_INSTANCE_PATH is not. This libController reads only "
            "OMNISIM_INSTANCE_PATH; the simulator binary launching it is older than that rename. Rebuild it, or run "
            "'python -m omnisim doctor'.\n");
  }
  return NULL;
}

// compute the path to the tmp directory
// extern controllers only
const char *wbu_system_tmpdir() {
  static char *tmpdir = NULL;
  if (tmpdir)
    return tmpdir;
#ifdef _WIN32
  const char *LOCALAPPDATA = getenv("LOCALAPPDATA");
  assert(LOCALAPPDATA && LOCALAPPDATA[0]);
  const size_t len = strlen(LOCALAPPDATA) + 6;  // adding "\\Temp"
  tmpdir = malloc(len);
  snprintf(tmpdir, len, "%s\\Temp", LOCALAPPDATA);
#elif defined(__linux__)
  // choose between snap or default tmp path
  // OMNISIM_HOME is the only install-root variable this runtime reads; the legacy WEBOTS_HOME
  // spelling was retired with the rest of the WEBOTS_* runtime contract.
  const char *install_root = getenv("OMNISIM_HOME");
  if (install_root && install_root[0]) {
    // Inherited from upstream Webots: its Ubuntu snap package confines writes, so an install
    // rooted under /snap/webots must use the snap's own tmp folder. OmniSim ships no snap, so
    // this branch only ever fires on an upstream-packaged install -- the literal is that
    // package's real path and stays accurate as written.
    if (strstr(install_root, "/snap/webots") != NULL) {
      const char *HOME = getenv("HOME");
      if (HOME && HOME[0]) {
        const size_t len = strlen(HOME) + strlen("/snap/webots/common/tmp") + 1;
        char *path = malloc(len);
        snprintf(path, len, "%s/snap/webots/common/tmp", HOME);
        tmpdir = path;
      }
    }
  }
  if (tmpdir == NULL)
    tmpdir = "/tmp";
#elif defined(__APPLE__)
  tmpdir = "/tmp";
#endif
  return tmpdir;
}
