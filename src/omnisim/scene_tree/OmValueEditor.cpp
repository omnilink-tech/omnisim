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

#include "OmValueEditor.hpp"

#include "OmEditCommand.hpp"
#include "OmField.hpp"
#include "OmMultipleValue.hpp"
#include "OmSingleValue.hpp"
#include "OmUndoStack.hpp"
#include "OmValue.hpp"
#include "OmVariant.hpp"
#include "OmWorld.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

OmValueEditor::OmValueEditor(QWidget *parent) :
  QWidget(parent),
  mNewValue(new OmVariant()),
  mPreviousValue(new OmVariant()),
  mComboBox(new QComboBox(this)),
  mLayout(new QGridLayout(this)),
  mNode(NULL),
  mField(NULL),
  mValue(NULL),
  mSingleValue(NULL),
  mMultipleValue(NULL),
  mIndex(-1) {
  mLayout->setColumnStretch(0, 0);
  mLayout->addWidget(mComboBox, 0, 1);
  mLayout->setColumnStretch(2, 0);
  mComboBox->setVisible(false);
}

OmValueEditor::~OmValueEditor() {
  delete mNewValue;
  delete mPreviousValue;
}

void OmValueEditor::edit(OmNode *node, OmField *field, int index) {
  mNode = node;
  mField = field;
  mValue = field->value();
  mSingleValue = dynamic_cast<OmSingleValue *>(mValue);
  mMultipleValue = dynamic_cast<OmMultipleValue *>(mValue);
  mIndex = index;

  disconnect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(apply()));
  mComboBox->clear();
  if (mField->singleType() != WB_SF_NODE && mField->hasRestrictedValues()) {
    if (mField->singleType() != WB_SF_STRING) {
      foreach (const OmFieldValueRestriction acceptedVariant, mField->acceptedValues())
        mComboBox->addItem(acceptedVariant.toSimplifiedStringRepresentation());
    } else {  // In case of MF/SF_STRING we don't want to display the starting and ending '"'
      foreach (const OmFieldValueRestriction acceptedVariant, mField->acceptedValues())
        mComboBox->addItem(acceptedVariant.toSimplifiedStringRepresentation().chopped(1).remove(0, 1));
    }
    connect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(apply()), Qt::UniqueConnection);
    connect(field, &OmField::valueChanged, this, &OmValueEditor::updateComboBoxIndex, Qt::UniqueConnection);

    updateComboBoxIndex();
    mComboBox->setVisible(true);
  } else
    mComboBox->setVisible(false);

  // call subclass
  edit(true);

  // start watching this value
  if (mValue)
    connect(mValue, &OmValue::destroyed, this, &OmValueEditor::cleanValue);
}

// disconnect from previously watched value
void OmValueEditor::stopEditing() {
  if (mValue)
    disconnect(mValue, &OmValue::destroyed, this, &OmValueEditor::valueInvalidated);

  mNode = NULL;
  mField = NULL;
  mValue = NULL;
  mSingleValue = NULL;
  mMultipleValue = NULL;
  mIndex = -1;
}

void OmValueEditor::cleanValue() {
  stopEditing();
  emit valueInvalidated();
}

void OmValueEditor::apply() {
  if (mValue == NULL)
    // no valid field to edit
    return;

  OmWorld::instance()->setModifiedFromSceneTree();
  OmUndoStack *stack = OmUndoStack::instance();
  // avoid complete reset of field editor when setting a field value
  stack->blockSignals(true);
  stack->push(new OmEditCommand(mValue, *mPreviousValue, *mNewValue, mIndex));
  stack->blockSignals(false);
  emit valueChanged();
}

void OmValueEditor::updateComboBoxIndex() {
  if (!mField)
    return;
  disconnect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(apply()));
  if (mField->singleType() != WB_SF_STRING) {
    if (mMultipleValue)
      mComboBox->setCurrentText(mMultipleValue->itemToString(mIndex));
    else
      mComboBox->setCurrentText(mValue->toString());
  } else {  // In case of MF/SF_STRING we don't want to display the starting and ending '"'
    if (mMultipleValue)
      mComboBox->setCurrentText(mMultipleValue->itemToString(mIndex).chopped(1).remove(0, 1));
    else
      mComboBox->setCurrentText(mValue->toString().chopped(1).remove(0, 1));
  }
  connect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(apply()), Qt::UniqueConnection);
}
