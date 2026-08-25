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

#include "OmIntEditor.hpp"

#include "OmField.hpp"
#include "OmFieldIntSpinBox.hpp"
#include "OmMFInt.hpp"
#include "OmSFInt.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

OmIntEditor::OmIntEditor(QWidget *parent) : OmValueEditor(parent), mInt(-1), mSpinBox(new OmFieldIntSpinBox(this)) {
  connect(mSpinBox, &OmFieldIntSpinBox::valueApplied, this, &OmIntEditor::apply);
  connect(mSpinBox, &OmFieldIntSpinBox::focusLeft, this, &OmIntEditor::applyIfNeeded);
  mLayout->addWidget(mSpinBox, 1, 1);
}

OmIntEditor::~OmIntEditor() {
}

void OmIntEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mSpinBox->blockSignals(block);
}

QWidget *OmIntEditor::lastEditorWidget() {
  return mSpinBox;
}

void OmIntEditor::takeKeyboardFocus() {
  mSpinBox->setFocus();
  mSpinBox->selectAll();
}

void OmIntEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mInt = static_cast<OmSFInt *>(singleValue())->value();
    else if (multipleValue())
      mInt = static_cast<OmMFInt *>(multipleValue())->item(index());
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  mSpinBox->setVisible(!hasRetrictedValues);

  if (hasRetrictedValues) {
    mLayout->setColumnStretch(0, 0);
    mLayout->setColumnStretch(2, 0);
  } else {
    mLayout->setColumnStretch(0, 1);
    mLayout->setColumnStretch(2, 1);
  }

  if (!hasRetrictedValues)
    mSpinBox->setValueNoSignals(mInt);
}

void OmIntEditor::resetFocus() {
  mSpinBox->clearFocus();
}

void OmIntEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mInt != mComboBox->currentText().toInt()) ||
                  (!field()->hasRestrictedValues() && mInt != mSpinBox->value())))
    apply();
}

void OmIntEditor::apply() {
  mInt = field()->hasRestrictedValues() ? mComboBox->currentText().toInt() : mSpinBox->value();

  if (singleValue()) {
    const OmSFInt *const sfInt = static_cast<OmSFInt *>(singleValue());
    if (sfInt->value() == mInt)
      return;

    mPreviousValue->setInt(sfInt->value());

  } else if (multipleValue()) {
    const OmMFInt *const mfInt = static_cast<OmMFInt *>(multipleValue());
    if (mfInt->item(index()) == mInt)
      return;

    mPreviousValue->setInt(mfInt->item(index()));
  }

  mNewValue->setInt(mInt);
  OmValueEditor::apply();
}
