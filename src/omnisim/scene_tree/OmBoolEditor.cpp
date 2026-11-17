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

#include "OmBoolEditor.hpp"

#include "OmField.hpp"
#include "OmMFBool.hpp"
#include "OmSFBool.hpp"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

OmBoolEditor::OmBoolEditor(QWidget *parent) : OmValueEditor(parent), mCheckBox(new QCheckBox(this)) {
  mCheckBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
#if QT_VERSION >= QT_VERSION_CHECK(6, 7, 0)
  connect(mCheckBox, &QCheckBox::checkStateChanged, this, &OmBoolEditor::apply);
#else
  connect(mCheckBox, &QCheckBox::stateChanged, this, &OmBoolEditor::apply);
#endif
  mLayout->addWidget(mCheckBox, 1, 1);
}

OmBoolEditor::~OmBoolEditor() {
}

void OmBoolEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mCheckBox->blockSignals(block);
}

QWidget *OmBoolEditor::lastEditorWidget() {
  return mCheckBox;
}

void OmBoolEditor::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    if (singleValue())
      mBool = static_cast<OmSFBool *>(singleValue())->value();
    else if (multipleValue())
      mBool = static_cast<OmMFBool *>(multipleValue())->item(index());
  }

  mCheckBox->setVisible(!field()->hasRestrictedValues());

  if (mBool)
    mCheckBox->setCheckState(Qt::Checked);
  else
    mCheckBox->setCheckState(Qt::Unchecked);

  updateText();
}

void OmBoolEditor::takeKeyboardFocus() {
  mCheckBox->setFocus();
}

void OmBoolEditor::updateText() {
  switch (mCheckBox->checkState()) {
    case Qt::Checked:
      mCheckBox->setText(tr("TRUE"));
      break;

    case Qt::Unchecked:
      mCheckBox->setText(tr("FALSE"));
      break;

    default:
      Q_ASSERT(0);
  }
}

void OmBoolEditor::resetFocus() {
  mCheckBox->clearFocus();
}

void OmBoolEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mBool != (mComboBox->currentText() == "TRUE")) ||
                  (!field()->hasRestrictedValues() && mBool != mCheckBox->checkState())))
    apply();
}

void OmBoolEditor::apply() {
  mBool = field()->hasRestrictedValues() ? mComboBox->currentText() == "TRUE" : mCheckBox->checkState();

  if (singleValue()) {
    const OmSFBool *const sfBool = static_cast<OmSFBool *>(singleValue());
    if (sfBool->value() == mBool)
      return;
    mPreviousValue->setBool(sfBool->value());
  } else if (multipleValue()) {
    const OmMFBool *const mfBool = static_cast<OmMFBool *>(multipleValue());
    if (mfBool->item(index()) == mBool)
      return;
    mPreviousValue->setBool(mfBool->item(index()));
  }

  mNewValue->setBool(mBool);
  OmValueEditor::apply();

  updateText();
}
