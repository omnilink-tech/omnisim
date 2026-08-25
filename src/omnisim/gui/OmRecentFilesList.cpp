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

#include "OmRecentFilesList.hpp"

#include "OmPreferences.hpp"

#include <QtGui/QAction>
#include <QtWidgets/QMenu>

OmRecentFilesList::OmRecentFilesList(int size, QMenu *parent) : QObject(parent) {
  mMax = size;
  mMenu = parent;
  mActions = new QAction *[mMax];
  mList = new QStringList();

  // create mActions
  for (int i = 0; i < mMax; ++i) {
    mActions[i] = new QAction(this);
    mActions[i]->setVisible(false);
    mMenu->addAction(mActions[i]);
    connect(mActions[i], &QAction::triggered, this, &OmRecentFilesList::actionTriggered);
  }

  OmPreferences *prefs = OmPreferences::instance();

  // update actions according to preferences
  for (int i = 0; i < mMax; i++) {
    const QString file = prefs->value(QString("RecentFiles/file%1").arg(i)).toString();
    if (file.isEmpty())
      break;
    mList->append(file);
#ifdef __linux__
    mActions[i]->setText(escapedText(file));
#else
    mActions[i]->setText(file);
#endif
    mActions[i]->setVisible(true);
  }
}

OmRecentFilesList::~OmRecentFilesList() {
  delete mList;
  delete[] mActions;
}

// update actions and preferences
void OmRecentFilesList::update() {
  OmPreferences *prefs = OmPreferences::instance();

  for (int i = 0; i < mMax; i++) {
    if (i < mList->size()) {
      const QString file = mList->at(i);
#ifdef __linux__
      mActions[i]->setText(escapedText(file));
#else
      mActions[i]->setText(file);
#endif
      mActions[i]->setVisible(true);
      prefs->setValue(QString("RecentFiles/file%1").arg(i), file);
    } else
      mActions[i]->setVisible(false);
  }
}

void OmRecentFilesList::makeRecent(const QString &filename) {
  int index = mList->indexOf(filename);

  // if this filename is already at the top: do nothing
  if (index == 0)
    return;

  // remove from middle of list
  if (index != -1)
    mList->removeAt(index);

  // add at the top
  mList->prepend(filename);

  // if list too large: remove last element
  if (mList->size() > mMax)
    mList->removeLast();

  update();
}

void OmRecentFilesList::actionTriggered() {
  const QAction *action = qobject_cast<QAction *>(sender());
  if (!action)
    return;

#ifdef __linux__
  emit fileChosen(unescapedText(action->text()));
#else
  emit fileChosen(action->text());
#endif
}

#ifdef __linux__
// escape all underscores so that they don't get interpreted as hotkeys by Unity
QString OmRecentFilesList::escapedText(const QString &text) {
  if (qgetenv("XDG_CURRENT_DESKTOP") == "Unity") {
    QString escapedTextString(text);
    return escapedTextString.replace("_", "__");
  }
  return text;
}

QString OmRecentFilesList::unescapedText(const QString &text) {
  if (qgetenv("XDG_CURRENT_DESKTOP") == "Unity") {
    QString escapedTextString(text);
    return escapedTextString.replace("__", "_");
  }
  return text;
}
#endif
