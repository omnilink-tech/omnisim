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

#include "OmVector3Editor.hpp"

#include "OmField.hpp"
#include "OmFieldDoubleSpinBox.hpp"
#include "OmMFVector3.hpp"
#include "OmSFVector3.hpp"
#include "OmSimulationState.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

OmVector3Editor::OmVector3Editor(QWidget *parent) : OmValueEditor(parent), mApplied(false) {
  static const QStringList LABELS(QStringList() << "x:"
                                                << "y:"
                                                << "z:");
  for (int i = 0; i < 3; ++i) {
    mLabel[i] = new QLabel(LABELS[i], this);
    mLayout->addWidget(mLabel[i], i + 1, 0, Qt::AlignRight);
    mSpinBoxes[i] = new OmFieldDoubleSpinBox(this);
    connect(mSpinBoxes[i], &OmFieldDoubleSpinBox::valueApplied, this, &OmVector3Editor::apply);
    connect(mSpinBoxes[i], &OmFieldDoubleSpinBox::focusLeft, this, &OmVector3Editor::applyIfNeeded);
    mLayout->addWidget(mSpinBoxes[i], i + 1, 1);
    mUnitLabel[i] = new QLabel("m", this);
    mLayout->addWidget(mUnitLabel[i], i + 1, 2);
  }
}

OmVector3Editor::~OmVector3Editor() {
}

void OmVector3Editor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  for (int i = 0; i < 3; ++i)
    mSpinBoxes[i]->blockSignals(block);
}

QWidget *OmVector3Editor::lastEditorWidget() {
  return mSpinBoxes[2];
}

void OmVector3Editor::edit(bool copyOriginalValue) {
  // don't show the results if the Vector3 editor
  // is edited manually (applied) in order to prevent
  // automatic normalization or default value restoration
  if (mApplied)
    return;

  if (copyOriginalValue) {
    if (singleValue())
      mVector3 = static_cast<OmSFVector3 *>(singleValue())->value();
    else if (multipleValue())
      mVector3 = static_cast<OmMFVector3 *>(multipleValue())->item(index());
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  for (int i = 0; i < 3; ++i) {
    mLabel[i]->setVisible(!hasRetrictedValues);
    mSpinBoxes[i]->setVisible(!hasRetrictedValues);
    mUnitLabel[i]->setVisible(!hasRetrictedValues);
  }

  if (!hasRetrictedValues)
    updateSpinBoxes();
}

void OmVector3Editor::takeKeyboardFocus() {
  mSpinBoxes[0]->setFocus();
  mSpinBoxes[0]->selectAll();
}

void OmVector3Editor::updateSpinBoxes() {
  for (int i = 0; i < 3; ++i)
    if (OmSimulationState::instance()->isPaused() || !mSpinBoxes[i]->hasFocus())  // in order to prevent updating while editing
      mSpinBoxes[i]->setValueNoSignals(mVector3[i]);
}

void OmVector3Editor::resetFocus() {
  for (int i = 0; i < 3; ++i)
    mSpinBoxes[i]->clearFocus();
}

void OmVector3Editor::applyIfNeeded() {
  if (field() &&
      ((field()->hasRestrictedValues() && mVector3 != OmVector3(mComboBox->currentText())) ||
       (!field()->hasRestrictedValues() && (mVector3.x() != mSpinBoxes[0]->value() || mVector3.y() != mSpinBoxes[1]->value() ||
                                            mVector3.z() != mSpinBoxes[2]->value()))))
    apply();
}

void OmVector3Editor::apply() {
  if (field()->hasRestrictedValues())
    mVector3 = OmVector3(mComboBox->currentText());
  else
    mVector3.setXyz(OmPrecision::roundValue(mSpinBoxes[0]->value(), OmPrecision::GUI_MEDIUM),
                    OmPrecision::roundValue(mSpinBoxes[1]->value(), OmPrecision::GUI_MEDIUM),
                    OmPrecision::roundValue(mSpinBoxes[2]->value(), OmPrecision::GUI_MEDIUM));
  mVector3.clamp();
  if (singleValue()) {
    const OmSFVector3 *const sfVector3 = static_cast<OmSFVector3 *>(singleValue());
    if (sfVector3->value() == mVector3)
      return;

    mPreviousValue->setVector3(sfVector3->value());

  } else if (multipleValue()) {
    const OmMFVector3 *const mfVector3 = static_cast<OmMFVector3 *>(multipleValue());
    if (mfVector3->item(index()) == mVector3)
      return;

    mPreviousValue->setVector3(mfVector3->item(index()));
  }

  mNewValue->setVector3(mVector3);
  mApplied = true;
  OmValueEditor::apply();
  mApplied = false;
}
