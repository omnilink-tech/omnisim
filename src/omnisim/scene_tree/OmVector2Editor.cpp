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

#include "OmVector2Editor.hpp"

#include "OmField.hpp"
#include "OmFieldDoubleSpinBox.hpp"
#include "OmMFVector2.hpp"
#include "OmSFVector2.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

OmVector2Editor::OmVector2Editor(QWidget *parent) : OmValueEditor(parent) {
  static const QStringList LABELS(QStringList() << "x:"
                                                << "y:");
  for (int i = 0; i < 2; ++i) {
    mLabel[i] = new QLabel(LABELS[i], this);
    mLayout->addWidget(mLabel[i], i + 1, 0, Qt::AlignRight);
    mSpinBoxes[i] = new OmFieldDoubleSpinBox(this);
    connect(mSpinBoxes[i], &OmFieldDoubleSpinBox::valueApplied, this, &OmVector2Editor::apply);
    connect(mSpinBoxes[i], &OmFieldDoubleSpinBox::focusLeft, this, &OmVector2Editor::applyIfNeeded);
    mLayout->addWidget(mSpinBoxes[i], i + 1, 1);
    mUnitLabel[i] = new QLabel("m", this);
    mLayout->addWidget(mUnitLabel[i], i + 1, 2);
  }
}

OmVector2Editor::~OmVector2Editor() {
}

void OmVector2Editor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  for (int i = 0; i < 2; ++i)
    mSpinBoxes[i]->blockSignals(block);
}

QWidget *OmVector2Editor::lastEditorWidget() {
  return mSpinBoxes[1];
}

void OmVector2Editor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mVector2 = static_cast<OmSFVector2 *>(singleValue())->value();
    else if (multipleValue())
      mVector2 = static_cast<OmMFVector2 *>(multipleValue())->item(index());
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  for (int i = 0; i < 2; ++i) {
    mLabel[i]->setVisible(!hasRetrictedValues);
    mSpinBoxes[i]->setVisible(!hasRetrictedValues);
    mUnitLabel[i]->setVisible(!hasRetrictedValues);
  }

  if (!hasRetrictedValues)
    updateSpinBoxes();
}

void OmVector2Editor::updateSpinBoxes() {
  mSpinBoxes[0]->setValueNoSignals(mVector2.x());
  mSpinBoxes[1]->setValueNoSignals(mVector2.y());
}

void OmVector2Editor::resetFocus() {
  mSpinBoxes[0]->clearFocus();
  mSpinBoxes[1]->clearFocus();
}

void OmVector2Editor::takeKeyboardFocus() {
  mSpinBoxes[0]->setFocus();
  mSpinBoxes[0]->selectAll();
}

void OmVector2Editor::applyIfNeeded() {
  if (field() &&
      ((field()->hasRestrictedValues() && mVector2 != OmVector2(mComboBox->currentText())) ||
       (!field()->hasRestrictedValues() && (mVector2.x() != mSpinBoxes[0]->value() || mVector2.y() != mSpinBoxes[1]->value()))))
    apply();
}

void OmVector2Editor::apply() {
  if (field()->hasRestrictedValues())
    mVector2 = OmVector2(mComboBox->currentText());
  else
    mVector2.setXy(OmPrecision::roundValue(mSpinBoxes[0]->value(), OmPrecision::GUI_MEDIUM),
                   OmPrecision::roundValue(mSpinBoxes[1]->value(), OmPrecision::GUI_MEDIUM));
  mVector2.clamp();
  if (singleValue()) {
    const OmSFVector2 *const sfVector2 = static_cast<OmSFVector2 *>(singleValue());
    if (sfVector2->value() == mVector2)
      return;

    mPreviousValue->setVector2(sfVector2->value());

  } else if (multipleValue()) {
    const OmMFVector2 *const mfVector2 = static_cast<OmMFVector2 *>(multipleValue());
    if (mfVector2->item(index()) == mVector2)
      return;

    mPreviousValue->setVector2(mfVector2->item(index()));
  }

  mNewValue->setVector2(mVector2);
  OmValueEditor::apply();
}
