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

#include "OmQjsFile.hpp"
#include "OmLog.hpp"
#include "OmStandardPaths.hpp"

#include <QtCore/QFile>
#include <QtCore/QTextStream>

bool OmQjsFile::fileExists(const QString &filePath) {
  QFile file(filePath);

  if (!file.open(QIODevice::ReadOnly))
    return false;

  return true;
}

QString OmQjsFile::readTextFile(const QString &filePath) {
  QFile file(filePath);

  if (!file.open(QIODevice::ReadOnly)) {
    OmLog::instance()->error(QString("JavaScript error: could not open file: %1.").arg(filePath), false, OmLog::PARSING);
    return "";
  }

  return file.readAll();
}

bool OmQjsFile::writeTextFile(const QString &fileName, const QString &content) {
  QFile file(OmStandardPaths::webotsTmpPath() + fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
    OmLog::instance()->error(
      QString("JavaScript error: could not write file '%1' to temporary path.").arg(fileName).arg(__FUNCTION__), false,
      OmLog::PARSING);
    return false;
  }

  QTextStream outputStream(&file);
  outputStream << content;
  file.close();

  return true;
}
