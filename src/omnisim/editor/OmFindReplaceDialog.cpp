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

#include "OmFindReplaceDialog.hpp"

#include "OmLineEdit.hpp"
#include "OmMessageBox.hpp"

#include <QtGui/QTextDocument>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QCompleter>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

OmFindReplaceDialog::OmFindReplaceDialog(OmTextFind *textFind, bool replace, const QString &title, QWidget *parent) :
  QDialog(parent, Qt::Tool),  // Qt::Tool allows to handle well the z-order. This is mainly advantageous on Mac
  mTextFind(textFind) {
  setWindowTitle(replace ? tr("Replace in %1").arg(title) : tr("Find in %1").arg(title));
  setAttribute(Qt::WA_DeleteOnClose);

  // create case sensitive completer
  // default completer for QComboBox is case insensitive
  QCompleter *comboBoxCompleter = new QCompleter(this);
  comboBoxCompleter->setCaseSensitivity(Qt::CaseSensitive);

  mFindCombo = new QComboBox(this);
  mFindCombo->setEditable(true);
  mFindCombo->setCompleter(comboBoxCompleter);
  mFindCombo->addItems(mTextFind->findStringList());
  mFindCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  mFindCombo->setMinimumContentsLength(20);
  mFindCombo->setFocus();

  QFormLayout *formLayout = new QFormLayout();
  formLayout->addRow(tr("&Search for:"), mFindCombo);

  if (replace) {
    mReplaceCombo = new QComboBox(this);
    mReplaceCombo->setEditable(true);
    mReplaceCombo->addItems(mTextFind->replaceStringList());
    mReplaceCombo->setCompleter(comboBoxCompleter);
    mReplaceCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    mReplaceCombo->setMinimumContentsLength(20);
    formLayout->addRow(tr("Replace &with:"), mReplaceCombo);
  }

  mRegExpCheckBox = new QCheckBox(tr("Use regular &expressions"), this);
  mWholeWordsCheckBox = new QCheckBox(tr("W&hole words"), this);
  mCaseSensitiveCheckBox = new QCheckBox(tr("&Match case"), this);
  // reset last used options
  OmTextFind::FindFlags flags = mTextFind->lastFindFlags();
  mRegExpCheckBox->setChecked(flags & OmTextFind::FIND_REGULAR_EXPRESSION);
  mWholeWordsCheckBox->setChecked(flags & OmTextFind::FIND_WHOLE_WORDS);
  mCaseSensitiveCheckBox->setChecked(flags & OmTextFind::FIND_CASE_SENSITIVE);

  mNextButton = new QPushButton(tr("&Next"), this);
  QPushButton *closeButton = new QPushButton(tr("&Close"), this);
  QPushButton *prevButton = new QPushButton(tr("&Previous"), this);

  connect(closeButton, &QPushButton::pressed, this, &OmFindReplaceDialog::performClose);
  connect(mNextButton, &QPushButton::pressed, this, &OmFindReplaceDialog::next);
  connect(prevButton, &QPushButton::pressed, this, &OmFindReplaceDialog::previous);

  QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal, this);
  buttonBox->addButton(closeButton, QDialogButtonBox::ActionRole);
  buttonBox->addButton(prevButton, QDialogButtonBox::ActionRole);
  buttonBox->addButton(mNextButton, QDialogButtonBox::ActionRole);

  if (replace) {
    prevButton->setText("Find &Previous");
    mNextButton->setText("Find &Next");

    QPushButton *replaceButton = new QPushButton(tr("&Replace"), this);
    QPushButton *replaceAllButton = new QPushButton(tr("Replace &All"), this);
    connect(replaceButton, &QPushButton::pressed, this, &OmFindReplaceDialog::replace);
    connect(replaceAllButton, &QPushButton::pressed, this, &OmFindReplaceDialog::replaceAll);
    buttonBox->addButton(replaceAllButton, QDialogButtonBox::ActionRole);
    buttonBox->addButton(replaceButton, QDialogButtonBox::ActionRole);

    replaceButton->setDefault(true);
  } else
    mNextButton->setDefault(true);

  QVBoxLayout *optionsLayout = new QVBoxLayout();
  optionsLayout->addWidget(mRegExpCheckBox);
  optionsLayout->addWidget(mWholeWordsCheckBox);
  optionsLayout->addWidget(mCaseSensitiveCheckBox);

  QGridLayout *mainLayout = new QGridLayout(this);
  mainLayout->addLayout(formLayout, 0, 0, Qt::AlignLeft);
  mainLayout->addLayout(optionsLayout, 1, 0, Qt::AlignLeft);
  mainLayout->addWidget(buttonBox, 2, 0, Qt::AlignRight);
}

void OmFindReplaceDialog::setFindString(const QString &findWhat) {
  mFindCombo->insertItem(0, findWhat);
  mFindCombo->setCurrentIndex(0);
  restoreFocus();
}

OmTextFind::FindFlags OmFindReplaceDialog::findFlags() {
  int flags = OmTextFind::FIND_NONE;
  if (mRegExpCheckBox->isChecked())
    flags |= OmTextFind::FIND_REGULAR_EXPRESSION;
  if (mCaseSensitiveCheckBox->isChecked())
    flags |= OmTextFind::FIND_CASE_SENSITIVE;
  if (mWholeWordsCheckBox->isChecked())
    flags |= OmTextFind::FIND_WHOLE_WORDS;
  return static_cast<OmTextFind::FindFlags>(flags);
}

bool OmFindReplaceDialog::find(OmTextFind *textFind, const QString &text, bool backwards, OmTextFind::FindFlags flags,
                               QWidget *parent) {
  bool success = textFind->find(text, flags, backwards);
  if (!success) {
    QString message(tr("\"%1\" was not found.").arg(text) + "\n" + tr("Wrap search and find again?"));
    if (OmMessageBox::question(message, parent) == QMessageBox::Ok) {
      if (backwards)
        success = textFind->findFromEnd(text, flags, backwards);
      else
        success = textFind->findFromBegin(text, flags, backwards);
    }
  }
  return success;
}

void OmFindReplaceDialog::next() {
  find(mTextFind, mFindCombo->currentText(), false, findFlags(), this);
  if (mFindCombo->currentText().isEmpty())
    return;

  updateFindComboItems();
}

void OmFindReplaceDialog::previous() {
  find(mTextFind, mFindCombo->currentText(), true, findFlags(), this);
  if (mFindCombo->currentText().isEmpty())
    return;

  updateFindComboItems();
}

void OmFindReplaceDialog::replace() {
  if (mFindCombo->currentText().isEmpty())
    return;

  mTextFind->replace(mFindCombo->currentText(), mReplaceCombo->currentText(), findFlags());
  updateFindComboItems();
  updateReplaceComboItems();
  find(mTextFind, mFindCombo->currentText(), false, findFlags(), this);
}

void OmFindReplaceDialog::replaceAll() {
  if (mFindCombo->currentText().isEmpty())
    return;

  mTextFind->replaceAll(mFindCombo->currentText(), mReplaceCombo->currentText(), findFlags());
  updateFindComboItems();
  updateReplaceComboItems();
}

void OmFindReplaceDialog::updateFindComboItems() {
  mFindCombo->clear();
  mFindCombo->addItems(mTextFind->findStringList());
}

void OmFindReplaceDialog::updateReplaceComboItems() {
  mReplaceCombo->clear();
  mReplaceCombo->addItems(mTextFind->replaceStringList());
}

void OmFindReplaceDialog::restoreFocus() {
  mNextButton->setFocus();
  mFindCombo->setFocus();
  mFindCombo->lineEdit()->selectAll();
}

void OmFindReplaceDialog::performClose() {
  restoreFocus();
  hide();
}

void OmFindReplaceDialog::findNext(OmTextFind *textFind, QWidget *parent) {
  if (textFind == NULL)
    return;

  QStringList findStringList = textFind->findStringList();
  if (findStringList.isEmpty())
    return;

  find(textFind, findStringList.first(), false, textFind->lastFindFlags(), parent);
}

void OmFindReplaceDialog::findPrevious(OmTextFind *textFind, QWidget *parent) {
  if (textFind == NULL)
    return;

  QStringList findStringList = textFind->findStringList();
  if (findStringList.isEmpty())
    return;

  find(textFind, findStringList.first(), true, textFind->lastFindFlags(), parent);
}
