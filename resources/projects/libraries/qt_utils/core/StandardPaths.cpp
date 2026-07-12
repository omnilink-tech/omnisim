#include "StandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QProcessEnvironment>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <webots/robot.h>

#include <QtCore/QDir>

#include <cassert>

using namespace webotsQtUtils;

#ifdef _WIN32
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#else
// dummy function used to know the full path
// of the current library
extern "C" {
static void foo() {
}
}
#endif

// Resolve the install root from the environment. OMNISIM_HOME is the canonical
// name (AGENTS.md §1); WEBOTS_HOME is kept as a fallback so a pre-rebrand
// environment still works. Returns "" when neither is set -- same as before,
// the caller appends "/" and downstream code copes with a bare relative path.
static QString resolveHomeFromEnv() {
  const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  const QString omnisimHome = env.value("OMNISIM_HOME");
  if (!omnisimHome.isEmpty())
    return omnisimHome;
  return env.value("WEBOTS_HOME");
}

const QString &StandardPaths::getWebotsHomePath() {
  static QString path(resolveHomeFromEnv() + "/");
  return path;
};

const QString &StandardPaths::getCurrentLibraryPath() {
  static bool defined = false;
  static QString path;

  if (!defined) {
#ifdef _WIN32
    WCHAR buffer[MAX_PATH] = {0};
    GetModuleFileNameW((HINSTANCE)&__ImageBase, buffer, sizeof(buffer));
    path = QString::fromWCharArray(buffer);
    path = path.replace('\\', '/');
#else
    Dl_info dl_info;
    dladdr(reinterpret_cast<void *>(foo), &dl_info);
    path = dl_info.dli_fname;
#endif
    path = path.mid(0, path.lastIndexOf('/') + 1);
    assert(!path.isEmpty());
    defined = true;
  }

  return path;
}

const QString &StandardPaths::getControllerPath() {
  static bool defined = false;
  static QString path;

  if (!defined) {
    path = QCoreApplication::applicationFilePath();
    path = path.mid(0, path.lastIndexOf('/') + 1);
    defined = true;
  }

  return path;
}

const QString &StandardPaths::getProjectPath() {
  static bool defined = false;
  static QString path;

  if (!defined) {
    path = QDir(wb_robot_get_project_path()).path();
    if (!path.endsWith('/'))
      path.append('/');
    defined = true;
  }

  return path;
}
