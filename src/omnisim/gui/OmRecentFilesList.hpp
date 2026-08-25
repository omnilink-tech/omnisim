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

#ifndef OM_RECENT_FILES_LIST_HPP
#define OM_RECENT_FILES_LIST_HPP

//
// Description: manages the list of recently opened files
//

#include <QtCore/QObject>

class QMenu;
class QAction;

class OmRecentFilesList : public QObject {
  Q_OBJECT

public:
  OmRecentFilesList(int size, QMenu *parent);
  virtual ~OmRecentFilesList();

  // move file name to top of list
  void makeRecent(const QString &filename);

signals:
  // a file was chosen in the menu
  void fileChosen(const QString &file);

private:
  int mMax;  // max recent files
  QAction **mActions;
  QStringList *mList;
  QMenu *mMenu;

  void update();

#ifdef __linux__
  static QString escapedText(const QString &text);
  static QString unescapedText(const QString &text);
#endif

private slots:
  void actionTriggered();
};

#endif
