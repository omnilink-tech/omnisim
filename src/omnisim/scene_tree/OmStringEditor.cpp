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

#include "OmStringEditor.hpp"

#include "OmField.hpp"
#include "OmFieldLineEdit.hpp"
#include "OmMFString.hpp"
#include "OmSFString.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

OmStringEditor::OmStringEditor(QWidget *parent) : OmValueEditor(parent), mLineEdit(new OmFieldLineEdit(this)) {
  connect(mLineEdit, &OmFieldLineEdit::returnPressed, this, &OmStringEditor::apply);
  connect(mLineEdit, &OmFieldLineEdit::focusLeft, this, &OmStringEditor::applyIfNeeded);
  mLayout->addWidget(mLineEdit, 1, 1);
}

OmStringEditor::~OmStringEditor() {
}

void OmStringEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mLineEdit->blockSignals(block);
}

QWidget *OmStringEditor::lastEditorWidget() {
  return mLineEdit;
}

void OmStringEditor::takeKeyboardFocus() {
  mLineEdit->setFocus();
  mLineEdit->selectAll();
}

void OmStringEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mString = static_cast<OmSFString *>(singleValue())->value();
    else if (multipleValue())
      mString = static_cast<OmMFString *>(multipleValue())->item(index());
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  mLineEdit->setVisible(!hasRetrictedValues);

  if (!hasRetrictedValues)
    mLineEdit->setText(mString);
}

void OmStringEditor::resetFocus() {
  mLineEdit->clearFocus();
}

void OmStringEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mString != mComboBox->currentText()) ||
                  (!field()->hasRestrictedValues() && mString != mLineEdit->text())))
    apply();
}

void OmStringEditor::apply() {
  mString = field()->hasRestrictedValues() ? mComboBox->currentText() : mLineEdit->text();

  if (singleValue()) {
    const OmSFString *const sfString = static_cast<OmSFString *>(singleValue());
    if (sfString->value() == mString)
      return;

    mPreviousValue->setString(sfString->value());

  } else if (multipleValue()) {
    const OmMFString *const mfString = static_cast<OmMFString *>(multipleValue());
    if (mfString->item(index()) == mString)
      return;

    mPreviousValue->setString(mfString->item(index()));
  }

  mNewValue->setString(mString);
  OmValueEditor::apply();
}
