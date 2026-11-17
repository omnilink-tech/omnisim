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

#include "OmRotationEditor.hpp"

#include "OmField.hpp"
#include "OmFieldDoubleSpinBox.hpp"
#include "OmMFRotation.hpp"
#include "OmQuaternion.hpp"
#include "OmSFRotation.hpp"
#include "OmSimulationState.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

static const QVector<QStringList> LABELS(QVector<QStringList>() << (QStringList() << "x:"
                                                                                  << "y:"
                                                                                  << "z:" << QObject::tr("angle:"))
                                                                << (QStringList() << "w:"
                                                                                  << "x:"
                                                                                  << "y:"
                                                                                  << "z:"));
static const QVector<QStringList> UNITS(QVector<QStringList>() << (QStringList() << "m"
                                                                                 << "m"
                                                                                 << "m"
                                                                                 << "rad")
                                                               << (QStringList() << " "
                                                                                 << " "
                                                                                 << " "
                                                                                 << " "));

OmRotationEditor::OmRotationEditor(QWidget *parent) : OmValueEditor(parent), mApplied(false) {
  mRotationTypeLabel = new QLabel("Rotation type:", this);
  mLayout->addWidget(mRotationTypeLabel, 1, 0, Qt::AlignRight);
  mRotationTypeComboBox = new QComboBox(this);
  mRotationTypeComboBox->addItem("Axis-Angle");
  mRotationTypeComboBox->addItem("Quaternions");
  connect(mRotationTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateRotationType(int)));
  mLayout->addWidget(mRotationTypeComboBox, 1, 1);
  mCurrentRotationType = 0;
  for (int i = 0; i < 4; ++i) {
    mLabel[i] = new QLabel(LABELS[AXIS_ANGLE][i], this);
    mLayout->addWidget(mLabel[i], i + 2, 0, Qt::AlignRight);
    mSpinBoxes[i] = new OmFieldDoubleSpinBox(this, i == 3 ? OmFieldDoubleSpinBox::RADIANS : OmFieldDoubleSpinBox::AXIS);
    connect(mSpinBoxes[i], &OmFieldDoubleSpinBox::valueApplied, this, &OmRotationEditor::apply);
    connect(mSpinBoxes[i], &OmFieldDoubleSpinBox::focusLeft, this, &OmRotationEditor::applyIfNeeded);
    mLayout->addWidget(mSpinBoxes[i], i + 2, 1);
    mUnitLabel[i] = new QLabel(UNITS[AXIS_ANGLE][i], this);
    mLayout->addWidget(mUnitLabel[i], i + 2, 2);
  }
  mNormalizeButton = new QPushButton(tr("Normalize"), this);
  mLayout->addWidget(mNormalizeButton, 7, 1);
  connect(mNormalizeButton, &QPushButton::pressed, this, &OmRotationEditor::normalize);
}

OmRotationEditor::~OmRotationEditor() {
}

void OmRotationEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  for (int i = 0; i < 4; ++i)
    mSpinBoxes[i]->blockSignals(block);
}

QWidget *OmRotationEditor::lastEditorWidget() {
  return mSpinBoxes[3];
}

void OmRotationEditor::edit(bool copyOriginalValue) {
  // don't show the results if the rotation editor
  // is edited manually (applied) in order to prevent
  // automatic normalization
  if (mApplied)
    return;

  if (copyOriginalValue) {
    if (singleValue())
      mRotation = static_cast<OmSFRotation *>(singleValue())->value();
    else if (multipleValue())
      mRotation = static_cast<OmMFRotation *>(multipleValue())->item(index());
    updateSpinBoxes();
  }

  const bool hasRetrictedValues = field()->hasRestrictedValues();
  for (int i = 0; i < 4; ++i) {
    mLabel[i]->setVisible(!hasRetrictedValues);
    mSpinBoxes[i]->setVisible(!hasRetrictedValues);
    mUnitLabel[i]->setVisible(!hasRetrictedValues);
  }
}

void OmRotationEditor::updateSpinBoxes() {
  for (int i = 0; i < 4; ++i)
    if (OmSimulationState::instance()->isPaused() || !mSpinBoxes[i]->hasFocus()) {
      if (mRotationTypeComboBox->currentIndex() == QUATERNIONS) {
        const OmQuaternion quaternion = mRotation.toQuaternion();
        mSpinBoxes[i]->setValueNoSignals(quaternion.ptr()[i]);
      } else  // AXIS_ANGLE
        mSpinBoxes[i]->setValueNoSignals(mRotation[i]);
    }
}

void OmRotationEditor::resetFocus() {
  for (int i = 0; i < 4; ++i)
    mSpinBoxes[i]->clearFocus();
}

void OmRotationEditor::takeKeyboardFocus() {
  mSpinBoxes[0]->setFocus();
  mSpinBoxes[0]->selectAll();
}

OmRotation OmRotationEditor::computeRotation() {
  if (mCurrentRotationType == QUATERNIONS) {
    OmQuaternion quaternion =
      OmQuaternion(mSpinBoxes[0]->value(), mSpinBoxes[1]->value(), mSpinBoxes[2]->value(), mSpinBoxes[3]->value());
    quaternion.normalize();
    return OmRotation(quaternion);
  }
  assert(mCurrentRotationType == AXIS_ANGLE);
  return OmRotation(mSpinBoxes[0]->value(), mSpinBoxes[1]->value(), mSpinBoxes[2]->value(), mSpinBoxes[3]->value());
}

void OmRotationEditor::applyIfNeeded() {
  if (field() && ((field()->hasRestrictedValues() && mRotation != OmRotation(mComboBox->currentText())) ||
                  (!field()->hasRestrictedValues() && mRotation != computeRotation())))
    apply();
}

void OmRotationEditor::apply() {
  mRotation = computeRotation();
  mRotation.normalize();

  if (field()->hasRestrictedValues())
    mRotation = OmRotation(mComboBox->currentText());

  if (singleValue()) {
    const OmSFRotation *const sfRotation = static_cast<OmSFRotation *>(singleValue());
    if (sfRotation->value() == mRotation)
      return;

    mPreviousValue->setRotation(sfRotation->value());

  } else if (multipleValue()) {
    const OmMFRotation *const mfRotation = static_cast<OmMFRotation *>(multipleValue());
    if (mfRotation->item(index()) == mRotation)
      return;

    mPreviousValue->setRotation(mfRotation->item(index()));
  }

  mNewValue->setRotation(mRotation);
  mApplied = true;
  OmValueEditor::apply();
  mApplied = false;
}

void OmRotationEditor::normalize() {
  applyIfNeeded();
  updateSpinBoxes();
}

void OmRotationEditor::updateRotationType(int index) {
  applyIfNeeded();
  for (int i = 0; i < 4; ++i) {
    mLabel[i]->setText(LABELS[mRotationTypeComboBox->currentIndex()][i]);
    mUnitLabel[i]->setText(UNITS[mRotationTypeComboBox->currentIndex()][i]);
    mSpinBoxes[i]->setMode(i == 3 && index == AXIS_ANGLE ? OmFieldDoubleSpinBox::RADIANS : OmFieldDoubleSpinBox::AXIS);
  }
  mCurrentRotationType = index;
  updateSpinBoxes();
}
