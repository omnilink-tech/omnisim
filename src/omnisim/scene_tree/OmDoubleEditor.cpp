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

#include "OmDoubleEditor.hpp"

#include "OmField.hpp"
#include "OmFieldDoubleSpinBox.hpp"
#include "OmMFDouble.hpp"
#include "OmSFDouble.hpp"
#include "OmSimulationState.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

OmDoubleEditor::OmDoubleEditor(QWidget *parent) : OmValueEditor(parent), mSpinBox(new OmFieldDoubleSpinBox(this)) {
  connect(mSpinBox, &OmFieldDoubleSpinBox::valueApplied, this, &OmDoubleEditor::apply);
  connect(mSpinBox, &OmFieldDoubleSpinBox::focusLeft, this, &OmDoubleEditor::applyIfNeeded);
  mLayout->addWidget(mSpinBox, 1, 1);
}

OmDoubleEditor::~OmDoubleEditor() {
}

void OmDoubleEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mSpinBox->blockSignals(block);
}

QWidget *OmDoubleEditor::lastEditorWidget() {
  return mSpinBox;
}

void OmDoubleEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mDouble = static_cast<OmSFDouble *>(singleValue())->value();
    else if (multipleValue())
      mDouble = static_cast<OmMFDouble *>(multipleValue())->item(index());
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
    mSpinBox->setValueNoSignals(mDouble);
}

void OmDoubleEditor::resetFocus() {
  mSpinBox->clearFocus();
}

void OmDoubleEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mDouble != mComboBox->currentText().toDouble()) ||
                  (!field()->hasRestrictedValues() && mDouble != mSpinBox->value())))
    apply();
}

void OmDoubleEditor::takeKeyboardFocus() {
  mSpinBox->setFocus();
  mSpinBox->selectAll();
}

void OmDoubleEditor::apply() {
  mDouble = field()->hasRestrictedValues() ? mComboBox->currentText().toDouble() :
                                             OmPrecision::roundValue(mSpinBox->value(), OmPrecision::GUI_MEDIUM);
  if (singleValue()) {
    const OmSFDouble *const sfDouble = static_cast<OmSFDouble *>(singleValue());
    if (sfDouble->value() == mDouble)
      return;

    mPreviousValue->setDouble(sfDouble->value());

  } else if (multipleValue()) {
    const OmMFDouble *const mfDouble = static_cast<OmMFDouble *>(multipleValue());
    if (mfDouble->item(index()) == mDouble)
      return;

    mPreviousValue->setDouble(mfDouble->item(index()));
  }

  mNewValue->setDouble(mDouble);
  OmValueEditor::apply();
}
