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

#include "OmDoubleSpinBox.hpp"

#include "OmClipboard.hpp"

#include <QtGui/QKeyEvent>
#include <QtWidgets/QLineEdit>

OmDoubleSpinBox::OmDoubleSpinBox(QWidget *parent) : QDoubleSpinBox(parent), mClipboard(OmClipboard::instance()) {
  setButtonSymbols(PlusMinus);
  setMinimumHeight(sizeHint().height());
}

OmDoubleSpinBox::~OmDoubleSpinBox() {
}

void OmDoubleSpinBox::keyPressEvent(QKeyEvent *event) {
  if (event->matches(QKeySequence::Cut))
    cut();
  else if (event->matches(QKeySequence::Copy))
    copy();
  else if (event->matches(QKeySequence::Paste))
    paste();
  else
    QDoubleSpinBox::keyPressEvent(event);
};

void OmDoubleSpinBox::cut() {
  if (lineEdit()->hasSelectedText()) {
    copy();
    lineEdit()->backspace();  // remove selected text
  }
}

void OmDoubleSpinBox::copy() const {
  if (lineEdit()->hasSelectedText()) {
    bool ok;
    const double value = lineEdit()->selectedText().toDouble(&ok);
    if (ok)
      mClipboard->setDouble(value);
    else
      mClipboard->setString(lineEdit()->selectedText());
  }
}

void OmDoubleSpinBox::paste() {
  if (mClipboard->isEmpty())
    return;

  QLineEdit *le = lineEdit();
  const QString pastedText = mClipboard->stringValue();
  QString newText = le->text();
  if (le->hasSelectedText())
    // remove selected text
    newText.remove(le->selectionStart(), le->selectedText().length());

  // insert pasted text
  newText.insert(le->cursorPosition(), mClipboard->stringValue());
  // check if valid value
  bool ok;
  // cppcheck-suppress ignoredReturnValue
  newText.toDouble(&ok);

  if (ok)
    le->insert(pastedText);
}
