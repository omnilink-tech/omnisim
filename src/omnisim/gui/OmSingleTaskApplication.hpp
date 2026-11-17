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

#ifndef OM_SINGLE_TASK_APPLICATION_HPP
#define OM_SINGLE_TASK_APPLICATION_HPP

//
// Description: object executing the task defined within the start options
//              and requesting the OmniSim application to exit immediately
//

#include "OmGuiApplication.hpp"

#include <QtCore/QString>

class OmSingleTaskApplication : public QObject {
  Q_OBJECT

public:
  explicit OmSingleTaskApplication(OmGuiApplication::Task task, const QStringList &taskArgument = QStringList(),
                                   QObject *parent = 0, const QString &startupPath = QString()) :
    QObject(parent),
    mTask(task),
    mTaskArguments(taskArgument),
    mStartupPath(startupPath) {}

public slots:
  void run();

signals:
  void finished(int returnCode);

private:
  OmGuiApplication::Task mTask;
  QStringList mTaskArguments;
  QString mStartupPath;

  void convertProto() const;
  void showHelp() const;
  void showSysInfo() const;
};

#endif
