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

#include "OmFieldEditor.hpp"

#include "OmAction.hpp"
#include "OmActionManager.hpp"
#include "OmApplication.hpp"
#include "OmBoolEditor.hpp"
#include "OmColorEditor.hpp"
#include "OmDoubleEditor.hpp"
#include "OmExtendedStringEditor.hpp"
#include "OmExternProtoEditor.hpp"
#include "OmField.hpp"
#include "OmIntEditor.hpp"
#include "OmLog.hpp"
#include "OmMFNode.hpp"
#include "OmMultipleValue.hpp"
#include "OmNode.hpp"
#include "OmNodeEditor.hpp"
#include "OmNodePane.hpp"
#include "OmNodeUtilities.hpp"
#include "OmProject.hpp"
#include "OmProtoModel.hpp"
#include "OmRotationEditor.hpp"
#include "OmSFNode.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"
#include "OmSelection.hpp"
#include "OmSolid.hpp"
#include "OmStandardPaths.hpp"
#include "OmUndoStack.hpp"
#include "OmVector2Editor.hpp"
#include "OmVector3Editor.hpp"

#include <QtGui/QAction>
#include <QtWidgets/QLabel>
#include <QtWidgets/QStackedLayout>
#include <QtWidgets/QToolButton>

#include <cassert>

// empty value editor: used when nothing needs editing
class OmEmptyEditor : public OmValueEditor {
public:
  explicit OmEmptyEditor(QWidget *parent = NULL) : OmValueEditor(parent) {}

  void recursiveBlockSignals(bool block) override { blockSignals(block); }

  QWidget *lastEditorWidget() override { return NULL; }

protected:
  void edit(bool copyOriginalValue) override {}

  void resetFocus() override {}

protected slots:
  void apply() override {}

private:
  void takeKeyboardFocus() override {}
};

static QSize gMinimumSizeOffset = QSize(0, 0);

OmFieldEditor::OmFieldEditor(QWidget *parent) :
  QWidget(parent),
  mNode(NULL),
  mField(NULL),
  mItem(-1),
  mNodeItem(NULL),
  mIsValidItemIndex(false) {
  setObjectName("fieldEditorGroupBox");

  OmExtendedStringEditor *const stringEditor = new OmExtendedStringEditor(this);
  connect(stringEditor, &OmExtendedStringEditor::editRequested, this, &OmFieldEditor::editRequested);

  OmNodePane *const nodePane = new OmNodePane(this);
  const OmNodeEditor *nodeEditor = nodePane->nodeEditor();
  connect(nodeEditor, &OmNodeEditor::dictionaryUpdateRequested, this, &OmFieldEditor::dictionaryUpdateRequested);

  // create editors
  mEditors.insert(WB_NO_FIELD, new OmEmptyEditor(this));
  mEditors.insert(WB_SF_BOOL, new OmBoolEditor(this));
  mEditors.insert(WB_SF_STRING, stringEditor);
  mEditors.insert(WB_SF_INT32, new OmIntEditor(this));
  mEditors.insert(WB_SF_FLOAT, new OmDoubleEditor(this));
  mEditors.insert(WB_SF_VEC2F, new OmVector2Editor(this));
  mEditors.insert(WB_SF_VEC3F, new OmVector3Editor(this));
  mEditors.insert(WB_SF_ROTATION, new OmRotationEditor(this));
  mEditors.insert(WB_SF_COLOR, new OmColorEditor(this));
  mEditors.insert(WB_SF_NODE, nodePane);

  mExternProtoEditor = new OmExternProtoEditor(this);

  // place all editors in a stacked layout
  mStackedLayout = new QStackedLayout();
  mStackedLayout->setSpacing(0);
  mStackedLayout->setContentsMargins(0, 0, 0, 0);
  foreach (OmValueEditor *editor, mEditors) {
    mStackedLayout->addWidget(editor);
    // trigger 3D view update after field value change
    connect(editor, &OmValueEditor::valueChanged, this, &OmFieldEditor::valueChanged);
  }
  mStackedLayout->addWidget(mExternProtoEditor);
  connect(nodePane->nodeEditor(), &OmValueEditor::valueChanged, this, &OmFieldEditor::valueChanged);
  connect(OmApplication::instance(), &OmApplication::worldLoadCompleted, this, &OmFieldEditor::refreshExternProtoEditor);

  mTitleLabel = new QLabel(this);
  mTitleLabel->setObjectName("titleLabel");
  mTitleLabel->setAlignment(Qt::AlignCenter);
  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  QWidget *wrapper = new QWidget();
  mainLayout->addWidget(wrapper);
  wrapper->setObjectName("wrapper");
  QVBoxLayout *intermediary = new QVBoxLayout();
  wrapper->setLayout(intermediary);
  intermediary->addWidget(mTitleLabel);
  intermediary->addLayout(mStackedLayout);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  intermediary->setContentsMargins(0, 0, 0, 0);
  gMinimumSizeOffset = sizeHint() - mStackedLayout->sizeHint();

  setCurrentWidget(0);
}

OmFieldEditor::~OmFieldEditor() {
}

OmValueEditor *OmFieldEditor::currentEditor() const {
  return static_cast<OmValueEditor *>(mStackedLayout->currentWidget());
}

void OmFieldEditor::refreshExternProtoEditor() {
  OmExternProtoEditor *editor = dynamic_cast<OmExternProtoEditor *>(mExternProtoEditor);
  if (currentEditor() == editor)
    editor->updateContents();
}

void OmFieldEditor::setTitle(const QString &title) {
  if (title.isEmpty())
    mTitleLabel->setText("");
  else
    mTitleLabel->setText("Selection: " + title);
}

QWidget *OmFieldEditor::lastEditorWidget() {
  return currentEditor()->lastEditorWidget();
}

QString OmFieldEditor::nodeAsTitle(OmNode *node) {
  if (!node)
    return "";
  else if (node->isProtoInstance())
    return node->modelName() + " (" + node->nodeModelName() + ")";
  else
    return node->modelName();
}

void OmFieldEditor::updateTitle() {
  assert(mField);

  QString title;
  OmValue *value = mField->value();

  if (mField->type() == WB_MF_NODE && mItem != -1)
    title = nodeAsTitle(static_cast<OmMFNode *>(value)->item(mItem));
  else if (mField->type() == WB_SF_NODE)
    title = mField->name() + " " + nodeAsTitle(static_cast<OmSFNode *>(value)->value());
  else {
    const OmMultipleValue *multipleValue = dynamic_cast<OmMultipleValue *>(value);
    if (multipleValue) {
      if (mItem == -1) {
        int size = multipleValue->size();
        QString type = OmValue::typeToShortName(value->singleType());
        if (size != 1)
          type += "s";

        title = QString("%1 (%2 %3)").arg(mField->name()).arg(size).arg(type);
      } else
        title = QString("%1 (%3 #%2)").arg(mField->name()).arg(mItem + 1).arg(OmValue::typeToShortName(value->singleType()));
    } else
      title = QString("%1 (%2)").arg(mField->name(), value->shortTypeName());
  }

  setTitle(title);
}

void OmFieldEditor::editExternProto() {
  mTitleLabel->setText("IMPORTABLE EXTERNPROTO");
  // disable current editor widget
  OmValueEditor *current = currentEditor();
  current->applyIfNeeded();
  current->stopEditing();
  disconnect(current, &OmValueEditor::valueInvalidated, this, &OmFieldEditor::invalidateValue);

  // enable extern proto
  OmExternProtoEditor *editor = dynamic_cast<OmExternProtoEditor *>(mExternProtoEditor);
  if (editor) {
    editor->updateContents();
    setCurrentWidget(mExternProtoEditor);
  }
}

void OmFieldEditor::editField(OmNode *node, OmField *field, int item) {
  disconnect(this, &OmFieldEditor::valueChanged, this, &OmFieldEditor::updateResetButton);
  if (node == mNode && field == mField && item == mItem) {
    if (field || node)
      updateValue(false);
    return;
  }

  mNode = node;
  mField = field;
  mItem = item;
  mNodeItem = NULL;
  mIsValidItemIndex = false;

  // disable current editor widget
  OmValueEditor *editor = currentEditor();
  editor->applyIfNeeded();
  editor->stopEditing();
  disconnect(editor, &OmValueEditor::valueInvalidated, this, &OmFieldEditor::invalidateValue);

  if (field == NULL && node == NULL) {
    invalidateValue();
    OmActionManager::instance()->action(OmAction::RESET_VALUE)->setEnabled(false);
    return;
  }

  computeFieldInformation();

  updateTitle();

  assert(field);
  OmActionManager::instance()
    ->action(OmAction::RESET_VALUE)
    ->setEnabled(!((field->isMultiple() && mIsValidItemIndex) || mField->isDefault()));

  if (field->isMultiple() && !mIsValidItemIndex) {
    setCurrentWidget(0);
    return;
  }

  // check if the selected item is a Solid node
  const QList<OmValueEditor *> &v = mEditors.values(field->value()->singleType());
  const bool editingSolid = dynamic_cast<OmSolid *>(mNodeItem) != NULL;
  if (v.size() == 1)
    editor = v.at(0);
  else
    editor = editingSolid ? v.at(0) : v.at(1);

  editor->edit(node, field, item);
  setCurrentWidget(editor);
  connect(editor, &OmValueEditor::valueInvalidated, this, &OmFieldEditor::invalidateValue);
  connect(this, &OmFieldEditor::valueChanged, this, &OmFieldEditor::updateResetButton);
}

void OmFieldEditor::invalidateValue() {
  mNode = NULL;
  mField = NULL;
  mItem = -1;
  setCurrentWidget(0);
}

void OmFieldEditor::resetFocus() {
  OmValueEditor *editor = currentEditor();
  editor->resetFocus();
}

void OmFieldEditor::updateResetButton() {
  const OmMultipleValue *const multipleValue = dynamic_cast<OmMultipleValue *>(mField->value());
  bool enabled = !((multipleValue && (mItem >= 0) && (mItem < multipleValue->size())) || mField->isDefault());
  OmActionManager::instance()->action(OmAction::RESET_VALUE)->setEnabled(enabled);
}

void OmFieldEditor::updateValue(bool copyOriginalValue) {
  OmValueEditor *editor = currentEditor();
  editor->edit(copyOriginalValue);
  updateResetButton();
}

void OmFieldEditor::applyChanges() {
  OmValueEditor *editor = currentEditor();
  editor->applyIfNeeded();
}

void OmFieldEditor::computeFieldInformation() {
  assert(mField);

  mNodeItem = NULL;
  mIsValidItemIndex = false;

  // check and store field type information
  OmValue *const value = mField->value();
  OmMultipleValue *const multipleValue = dynamic_cast<OmMultipleValue *>(value);
  const OmMFNode *mfNode = NULL;

  if (multipleValue) {
    mIsValidItemIndex = (mItem >= 0) && (mItem < multipleValue->size());

    if (mIsValidItemIndex) {
      mfNode = dynamic_cast<OmMFNode *>(multipleValue);
      if (mfNode)
        mNodeItem = mfNode->item(mItem);
    }
  } else {
    const OmSFNode *const sfNode = dynamic_cast<OmSFNode *>(value);
    if (sfNode)
      mNodeItem = sfNode->value();
  }
}

void OmFieldEditor::setCurrentWidget(int index) {
  setCurrentWidget(mEditors.value(WbFieldType(index)));
}

void OmFieldEditor::setCurrentWidget(OmValueEditor *editor) {
  mStackedLayout->setCurrentWidget(editor);
}
