// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#include "OmLog.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <mutex>

#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QMetaType>
#include <QtCore/QUrl>

// File logger. Default destination: <WEBOTS_HOME>/omnisim_log.txt, but
// see initFileLog below for per-instance fallback behavior.
static std::ofstream gLogFile;
static std::mutex gLogMutex;
static bool gFileLogEnabled = false;
static QString gLogFilePath;

static void fileLogWrite(const char *message) {
  if (!gFileLogEnabled)
    return;
  std::lock_guard<std::mutex> lock(gLogMutex);
  if (gLogFile.is_open()) {
    // Per-call flush so the log file is observable from outside the
    // process between writes (headless_runner tails it; debugging
    // sessions follow it live). The companion P5 fixes elsewhere
    // (silencing the per-joint queued-log + capping repeated warnings)
    // already eliminated the high-rate log sites that made per-call
    // flush a world-load bottleneck.
    gLogFile << message << "\n" << std::flush;
  }
}

void OmLog::initFileLog(const QString &logPath) {
  std::lock_guard<std::mutex> lock(gLogMutex);

  // Env override wins (callers that pin a path for their own bookkeeping).
  QString path = qEnvironmentVariable("OMNISIM_LOG_PATH");
  if (path.isEmpty())
    path = logPath;

  gLogFile.open(path.toUtf8().constData(), std::ios::out | std::ios::trunc);
  if (!gLogFile.is_open()) {
    // Another OmniSim instance is holding the canonical log. Fall back to a
    // per-process file so this instance still produces a log somewhere.
    const QFileInfo fi(path);
    const qint64 pid = QCoreApplication::applicationPid();
    const QString fallback = fi.dir().filePath(QString("omnisim_log.%1.txt").arg(pid));
    gLogFile.open(fallback.toUtf8().constData(), std::ios::out | std::ios::trunc);
    if (gLogFile.is_open()) {
      std::cerr << "[OmniSim] log path '" << path.toUtf8().constData()
                << "' was busy; logging to '" << fallback.toUtf8().constData()
                << "' instead." << std::endl;
      path = fallback;
    }
  }

  if (gLogFile.is_open()) {
    gFileLogEnabled = true;
    gLogFilePath = path;
    gLogFile << "=== OmniSim Log Started (pid="
             << QCoreApplication::applicationPid() << "): "
             << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toUtf8().constData()
             << " ===\n" << std::flush;
    std::cerr << "[OmniSim] log: " << path.toUtf8().constData() << std::endl;
    // Clear any stale Newton backend-verdict sidecar left next to the log by a
    // PREVIOUS run. OmNewtonBackend::finalizeWorld() (re)writes "<log>.newton.json"
    // only when Newton actually finalises the world; removing it here -- the one
    // point a fresh run truncates the log -- guarantees the file's presence means
    // "Newton drove THIS run", with no cross-run staleness even for a forced-ODE
    // run where the Newton backend is never constructed. env_fingerprint reads it
    // in preference to scraping the log so the on-screen physics label can't
    // mislabel a Newton run as ODE.
    QFile::remove(path + QStringLiteral(".newton.json"));
  }
}

QString OmLog::logFilePath() {
  return gLogFilePath;
}

void OmLog::closeFileLog() {
  std::lock_guard<std::mutex> lock(gLogMutex);
  if (gLogFile.is_open()) {
    gLogFile << "=== OmniSim Log Ended ===\n";
    gLogFile.close();
  }
  gFileLogEnabled = false;
}

void OmLog::fileLog(const QString &message) {
  fileLogWrite(message.toUtf8().constData());
}

void OmLog::diagnostic(const char *code, const QString &message, const QString &file, int line,
                       const char *severity) {
  static const bool enabled = qEnvironmentVariableIsSet("OMNISIM_STRUCTURED_LOG");
  if (!enabled)
    return;
  QString tagged = QString("[OMNISIM-DIAG] code=%1 severity=%2").arg(code).arg(severity);
  if (!file.isEmpty()) {
    QString escapedFile = file;
    escapedFile.replace('"', "\\\"");
    tagged += QString(" file=\"%1\"").arg(escapedFile);
  }
  if (line >= 0)
    tagged += QString(" line=%1").arg(line);
  QString escapedMsg = message;
  escapedMsg.replace('"', "\\\"");
  tagged += QString(" message=\"%1\"").arg(escapedMsg);
  fileLogWrite(tagged.toUtf8().constData());
}

static OmLog *gInstance = NULL;

void OmLog::cleanup() {
  gInstance->mPostponedPopUpMessageQueue.clear();
  gInstance->mPendingConsoleMessages.clear();
  delete gInstance;
  gInstance = NULL;
}

OmLog *OmLog::instance() {
  if (!gInstance) {
    gInstance = new OmLog();
    qRegisterMetaType<OmLog::Level>("OmLog::Level");
    qAddPostRoutine(OmLog::cleanup);
  }

  return gInstance;
}

static bool gStdoutRedirect = false;
static bool gStderrRedirect = false;

void OmLog::enableStdOutRedirectToTerminal() {
  gStdoutRedirect = true;
};

void OmLog::enableStdErrRedirectToTerminal() {
  gStderrRedirect = true;
};

void OmLog::debug(const QString &message, bool popup, Filter filter) {
  debug(message, filterName(filter), popup);
}

void OmLog::info(const QString &message, bool popup, Filter filter) {
  info(message, filterName(filter), popup);
}

void OmLog::warning(const QString &message, bool popup, Filter filter) {
  warning(message, filterName(filter), popup);
}

void OmLog::error(const QString &message, bool popup, Filter filter) {
  error(message, filterName(filter), popup);
}

void OmLog::debug(const QString &message, const QString &name, bool popup) {
  const char *header = "DEBUG: ";
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, name, DEBUG);
    return;
  }

  std::cerr << header << message.toUtf8().constData() << "\n" << std::flush;
  fileLogWrite((std::string(header) + message.toUtf8().constData()).c_str());
  if (!instance()->mConsoleLogsPostponed &&
      instance()->receivers(SIGNAL(logEmitted(OmLog::Level, const QString &, bool, const QString &))) > 1)
    instance()->emitLog(DEBUG, header + message, popup, name);
  else
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, header + message, name, DEBUG);
}

void OmLog::info(const QString &message, const QString &name, bool popup) {
  const char *header = "INFO: ";
  fileLogWrite((std::string(header) + message.toUtf8().constData()).c_str());
  if (gStdoutRedirect)
    std::cout << header << message.toUtf8().constData() << "\n" << std::flush;
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, name, INFO);
    return;
  }

  // P5 hang fix 2026-05-28: when console logs are postponed (the common
  // case during world load), short-circuit to enqueue without calling
  // QObject::receivers(). The receivers() lookup walks Qt's internal
  // connection list under a meta-object mutex, and at high call rates
  // (40+ huskies × 4 joints each, or 40+ top_plate_link warnings) the
  // cumulative cost stalls world load. Since the receivers() result is
  // only used by the !postponed branch, we can skip it entirely when
  // postponed is true -- semantics unchanged.
  if (instance()->mConsoleLogsPostponed) {
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, header + message, name, INFO);
    return;
  }

  const int numberOfReceivers = instance()->receivers(SIGNAL(logEmitted(OmLog::Level, const QString &, bool, const QString &)));
  if (numberOfReceivers > 1)
    instance()->emitLog(INFO, header + message, popup, name);
  else
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, header + message, name, INFO);
}

void OmLog::warning(const QString &message, const QString &name, bool popup) {
  const char *header = "WARNING: ";
  fileLogWrite((std::string(header) + message.toUtf8().constData()).c_str());
  if (gStderrRedirect)
    std::cerr << header << message.toUtf8().constData() << "\n" << std::flush;
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, name, WARNING);
    return;
  }

  // P5 hang fix: skip the QObject::receivers() lookup when postponed.
  // See companion comment in info() above.
  if (instance()->mConsoleLogsPostponed) {
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, header + message, name, WARNING);
    return;
  }

  const int numberOfReceivers = instance()->receivers(SIGNAL(logEmitted(OmLog::Level, const QString &, bool, const QString &)));
  if (numberOfReceivers > 1)
    instance()->emitLog(WARNING, header + message, popup, name);
  else
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, header + message, name, WARNING);
}

void OmLog::error(const QString &message, const QString &name, bool popup) {
  const int numberOfReceivers = instance()->receivers(SIGNAL(logEmitted(OmLog::Level, const QString &, bool, const QString &)));
  const char *header = "ERROR: ";
  fileLogWrite((std::string(header) + message.toUtf8().constData()).c_str());
  if (gStderrRedirect || numberOfReceivers == 0)
    std::cerr << header << message.toUtf8().constData() << "\n" << std::flush;
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, name, ERROR);
    return;
  }

  if (!instance()->mConsoleLogsPostponed && numberOfReceivers > 1)
    instance()->emitLog(ERROR, header + message, popup, name);
  else
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, header + message, name, ERROR);
}

void OmLog::fatal(const QString &message) {
  const char *header = "FATAL: ";
  fileLogWrite((std::string(header) + message.toUtf8().constData()).c_str());
  std::cerr << header << message.toUtf8().constData() << "\n" << std::flush;
  instance()->emitLog(FATAL, header + message, true, QString());
  ::exit(EXIT_FAILURE);
}

void OmLog::status(const QString &message) {
  instance()->emitLog(STATUS, message, false, QString());
}

const QString &OmLog::filterName(Filter filter) {
  static const QStringList names = QStringList() << "All"
                                                 << "OmniSim"
                                                 << "All OmniSim"
                                                 << "Controller(s)"
                                                 << "All Controller(s)"
                                                 << "Others"
                                                 << "Physics"
                                                 << "Physics Plugins"
                                                 << "Javascript"
                                                 << "Parsing"
                                                 << "Compilation";
  assert(names.size() == FILTER_SIZE);
  assert(filter < FILTER_SIZE);
  return names.at(filter);
};

const QStringList &OmLog::omniSimFilterNames() {
  static const QStringList names = QStringList() << filterName(OMNISIM_OTHERS) << filterName(ODE) << filterName(PHYSICS_PLUGINS)
                                                 << filterName(JAVASCRIPT) << filterName(PARSING) << filterName(COMPILATION);
  return names;
};

const QString &OmLog::levelName(Level level) {
  static const QStringList names = QStringList() << "Debug"
                                                 << "Info"
                                                 << "Warning"
                                                 << "Error"
                                                 << "Fatal"
                                                 << "Status"
                                                 << "Stdout"
                                                 << "Stderr";
  assert(names.size() == LEVEL_SIZE);
  assert(level < LEVEL_SIZE);
  return names.at(level);
};

void OmLog::appendStdout(const QString &message, Filter filter) {
  appendStdout(message, filterName(filter));
}

void OmLog::appendStderr(const QString &message, Filter filter) {
  appendStderr(message, filterName(filter));
}

void OmLog::appendStdout(const QString &message, const QString &name) {
  if (gStdoutRedirect)
    std::cout << message.toUtf8().constData() << std::flush;
  emit instance()->logEmitted(STDOUT, message, false, name);
}

void OmLog::appendStderr(const QString &message, const QString &name) {
  if (gStderrRedirect)
    std::cerr << message.toUtf8().constData() << std::flush;
  emit instance()->logEmitted(STDERR, message, false, name);
}

void OmLog::javascriptLogToConsole(const QString &message, int lineNumber, const QString &sourceUrl) {
  QString sourceFile = QUrl::fromLocalFile(sourceUrl).fileName();
  QString log = "[javascript] " + message + " (" + sourceFile + ":" + QString::number(lineNumber) + ")";
  OmLog::appendStdout(log, OmLog::JAVASCRIPT);
}

void OmLog::emitLog(Level level, const QString &message, bool popup, const QString &name) {
  if (popup)
    emit popupOpen();
  emit logEmitted(level, message, popup, name);
  if (popup)
    emit popupClosed();
}

void OmLog::enqueueMessage(QList<PostponedMessage> &list, const QString &message, const QString &name, Level level) {
  PostponedMessage msg;
  msg.text = message;
  msg.name = name;
  msg.level = level;
  list.append(msg);
}

void OmLog::showPostponedPopUpMessages() {
  bool tmp = instance()->mPopUpMessagesPostponed;
  setPopUpPostponed(false);
  foreach (const PostponedMessage &msg, instance()->mPostponedPopUpMessageQueue) {
    switch (msg.level) {
      case DEBUG:
        debug(msg.text, msg.name, true);
        break;
      case INFO:
        info(msg.text, msg.name, true);
        break;
      case WARNING:
        warning(msg.text, msg.name, true);
        break;
      case ERROR:
        error(msg.text, msg.name, true);
        break;
      default:
        break;
    }
  }
  setPopUpPostponed(tmp);

  instance()->mPostponedPopUpMessageQueue.clear();
}

void OmLog::showPendingConsoleMessages() {
  foreach (const PostponedMessage &msg, instance()->mPendingConsoleMessages)
    emit instance()->logEmitted(msg.level, msg.text, false, msg.name);
  instance()->mPendingConsoleMessages.clear();
}

void OmLog::toggle(FILE *std_stream) {
#ifndef _WIN32  // this doesn't work on Windows
  static int fd[3] = {0, 0, 0};
  const int no = fileno(std_stream);
  assert(no >= 1);  // it shouldn't be stdin
  fflush(std_stream);
  if (fd[no] == 0) {
    static const FILE *stream;  // to make cppcheck happy about resource leak
    fd[no] = dup(no);
    stream = freopen("/dev/null", "w", std_stream);
    if (!stream)
      fprintf(stderr, "Failed to mute %s.", (no == 1) ? "stdout" : "stderr");
  } else {
    dup2(fd[no], no);
    close(fd[no]);
    fd[no] = 0;
  }
#endif
}
