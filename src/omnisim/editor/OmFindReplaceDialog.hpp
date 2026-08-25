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

#ifndef OM_FIND_REPLACE_DIALOG_HPP
#define OM_FIND_REPLACE_DIALOG_HPP

//
// Description: Find dialog for the Console
//

#include "OmTextFind.hpp"

#include <QtGui/QTextCursor>
#include <QtWidgets/QDialog>

class QComboBox;
class QCheckBox;
class QPushButton;

class OmFindReplaceDialog : public QDialog {
  Q_OBJECT

public:
  // if 'replace' is false: create a Find dialog
  // if 'replace' is true: create a Find/Replace dialog
  explicit OmFindReplaceDialog(OmTextFind *textFind, bool replace, const QString &title, QWidget *parent = NULL);

  // find string/replace strings
  void setFindString(const QString &findWhat);

  static void findNext(OmTextFind *textFind, QWidget *parent);
  static void findPrevious(OmTextFind *textFind, QWidget *parent);

public slots:
  void next();
  void previous();

private:
  // find dialog
  OmTextFind *mTextFind;
  QComboBox *mFindCombo;
  QCheckBox *mCaseSensitiveCheckBox, *mWholeWordsCheckBox, *mRegExpCheckBox;
  QPushButton *mNextButton;

  // extra widget for replace dialog
  QComboBox *mReplaceCombo;

  void restoreFocus();
  void updateFindComboItems();
  void updateReplaceComboItems();
  OmTextFind::FindFlags findFlags();

  static bool find(OmTextFind *textFind, const QString &text, bool backwards, OmTextFind::FindFlags flags, QWidget *parent);

private slots:
  void replace();
  void replaceAll();
  void performClose();
};

#endif
