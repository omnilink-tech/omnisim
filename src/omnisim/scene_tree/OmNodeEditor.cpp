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

#include "OmNodeEditor.hpp"

#include "OmBaseNode.hpp"
#include "OmField.hpp"
#include "OmFieldLineEdit.hpp"
#include "OmGeometry.hpp"
#include "OmGroup.hpp"
#include "OmLog.hpp"
#include "OmMFNode.hpp"
#include "OmMessageBox.hpp"
#include "OmNode.hpp"
#include "OmNodeModel.hpp"
#include "OmNodeUtilities.hpp"
#include "OmProtoManager.hpp"
#include "OmSFNode.hpp"
#include "OmSelection.hpp"
#include "OmToken.hpp"
#include "OmTransform.hpp"
#include "OmViewpoint.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorldInfo.hpp"

#include <QtCore/QDir>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStackedWidget>

OmNodeEditor::OmNodeEditor(QWidget *parent) :
  OmValueEditor(parent),
  mNode(NULL),
  mDefEdit(new OmFieldLineEdit(this)),
  mUseCount(new QLabel(this)),
  mPrintUrl(new QPushButton("Print EXTERNPROTO", this)),
  mNbTriangles(new QLabel(this)),
  mStackedWidget(new QStackedWidget(this)),
  mMessageBox(false),
  mShowResizeHandlesLabel(new QLabel(tr("3D tools:"), this)),
  mShowResizeHandlesCheckBox(new QCheckBox(tr("show resize handles"), this)) {
  mShowResizeHandlesCheckBox->setChecked(false);
  QWidget *nodePane = new QWidget(this);
  nodePane->setObjectName("NodeEditorBackground");
  QGridLayout *const layout = new QGridLayout(nodePane);
  layout->addWidget(new QLabel("DEF:", this), 0, 0);
  layout->addWidget(mDefEdit, 0, 1);
  layout->addWidget(mUseCount, 1, 1);
  layout->addWidget(mPrintUrl, 3, 1);
  layout->addWidget(mNbTriangles, 4, 1);

  layout->addWidget(mShowResizeHandlesLabel, 5, 0);
  layout->addWidget(mShowResizeHandlesCheckBox, 5, 1);

  // setup layout size policy in order to put all the widgets top - left
  // vertically
  QWidget *vStretch = new QWidget(this);
  layout->addWidget(vStretch, 5, 0);
  layout->setRowStretch(5, 1);
  // horizontally
  QWidget *hStretch = new QWidget(this);
  layout->addWidget(hStretch, 0, 2);
  layout->setColumnStretch(2, 1);

  // Main layout
  mStackedWidget->addWidget(nodePane);
  mStackedWidget->addWidget(new QWidget(this));  // empty pane
  mLayout->addWidget(mStackedWidget, 1, 1);

  connect(mDefEdit, &OmFieldLineEdit::returnPressed, this, &OmNodeEditor::apply);
  connect(mDefEdit, &OmFieldLineEdit::focusLeft, this, &OmNodeEditor::apply);
  connect(mPrintUrl, &QPushButton::pressed, this, &OmNodeEditor::printUrl);
  connect(mShowResizeHandlesCheckBox, &QAbstractButton::toggled, OmSelection::instance(),
          &OmSelection::showResizeManipulatorFromSceneTree, Qt::UniqueConnection);
}

void OmNodeEditor::printUrl() {
  if (!mNode->isProtoInstance())
    return;

  OmLog::info(tr("EXTERNPROTO \"%1\"").arg(OmProtoManager::instance()->externProtoUrl(mNode, true)));
}

void OmNodeEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mStackedWidget->blockSignals(block);
  mDefEdit->blockSignals(block);
}

void OmNodeEditor::edit(bool copyOriginalValue) {
  bool sfnodeChanged = false;
  if (singleValue()) {
    OmNode *newNode = static_cast<OmSFNode *>(singleValue())->value();
    sfnodeChanged = mNode != newNode;
    mNode = newNode;
  }

  if (copyOriginalValue || sfnodeChanged) {
    if (multipleValue())
      mNode = static_cast<OmMFNode *>(multipleValue())->item(index());

    OmBaseNode *const baseNode = dynamic_cast<OmBaseNode *>(mNode);
    if (baseNode) {
      const bool handlesAvailable = baseNode->hasResizeManipulator();

      mShowResizeHandlesLabel->setVisible(handlesAvailable);
      mShowResizeHandlesCheckBox->setVisible(handlesAvailable);
      mShowResizeHandlesCheckBox->setEnabled(true);

      if (OmNodeUtilities::isNodeOrAncestorLocked(baseNode))
        mShowResizeHandlesCheckBox->setEnabled(false);

      if (handlesAvailable) {
        const OmGeometry *g = dynamic_cast<const OmGeometry *>(baseNode);
        if (g)
          mShowResizeHandlesCheckBox->setChecked(g->isResizeManipulatorAttached());
      }
    }
  }

  update();
}

void OmNodeEditor::cleanValue() {
  mNode = NULL;
  OmValueEditor::cleanValue();
  mShowResizeHandlesCheckBox->blockSignals(true);
  mShowResizeHandlesCheckBox->setChecked(false);
  mShowResizeHandlesCheckBox->blockSignals(false);
}

void OmNodeEditor::stopEditing() {
  if (!mNode)
    // value destroyed
    return;

  mShowResizeHandlesCheckBox->blockSignals(true);
  mShowResizeHandlesCheckBox->setChecked(false);
  mShowResizeHandlesCheckBox->blockSignals(false);
}

void OmNodeEditor::update() {
  if (mNode && !mNode->isUseNode()) {
    mStackedWidget->setCurrentIndex(DEF_PANE);
    mDefEdit->setText(mNode->defName());
    if (mNode->defName().isEmpty())
      mUseCount->clear();
    else
      mUseCount->setText(tr("USE count: %1").arg(mNode->useCount()));  // TODO: is this the final implementation?

    if (mNode->isProtoInstance()) {
      mPrintUrl->setVisible(true);
      mPrintUrl->setToolTip(OmProtoManager::instance()->externProtoUrl(mNode, true));
    } else
      mPrintUrl->setVisible(false);
  } else
    mStackedWidget->setCurrentIndex(EMPTY_PANE);

  const OmGeometry *node = dynamic_cast<OmGeometry *>(mNode);
  if (node && !node->isUseNode()) {
    const int maxTriangleNumberToCastShadows = node->maxIndexNumberToCastShadows() / 3;
    int triangleCount = node->triangleCount();
    if (triangleCount > maxTriangleNumberToCastShadows)
      mNbTriangles->setText(tr("Triangle count: %1 (no shadow)").arg(triangleCount));
    else
      mNbTriangles->setText(tr("Triangle count: %1").arg(triangleCount));
  } else
    mNbTriangles->clear();
}

void OmNodeEditor::resetFocus() {
  mDefEdit->clearFocus();
}

void OmNodeEditor::apply() {
  if (!mNode || mStackedWidget->currentIndex() == EMPTY_PANE)
    return;

  // message box popup makes lineEdit lose its focus
  if (mMessageBox)
    return;

  QString newDef = mDefEdit->text();
  const QString &previousDef = mNode->defName();

  if (newDef == previousDef)
    return;

  // block duplicated OmNodeEditor::apply call triggered by focusOutEvent
  mDefEdit->blockSignals(true);

  if (newDef.isEmpty() && mNode->useCount() > 0) {
    OmMessageBox::warning(tr("This DEF cannot be cleared because some USE nodes depend on it."), this);
    mDefEdit->setText(previousDef);  // restore
    mDefEdit->blockSignals(false);
    return;
  }

  bool dictionaryUpdateRequest = false;
  if (!newDef.isEmpty()) {
    // check if the new DEF name is not already used by subsequent USE nodes
    bool defOverlap = false;
    bool useOverlap = false;
    dictionaryUpdateRequest =
      OmVrmlNodeUtilities::hasASubsequentUseOrDefNode(mNode, newDef, previousDef, useOverlap, defOverlap);
    if (dictionaryUpdateRequest) {
      mMessageBox = true;
      QString message;
      if (defOverlap && useOverlap) {
        message = tr("This DEF string is already used by subsequent USE and DEF nodes. "
                     "Applying this change will modify all the USE nodes referring to previous node with same DEF name "
                     "and USE nodes referring to the selected node. \n"
                     "Do you want to continue?");
      } else if (defOverlap) {
        message = tr("This DEF string is already used by subsequent DEF nodes. "
                     "Applying this change will turn USE nodes of the selected node into copies of subsequent DEF node.\n"
                     "Do you want to continue?");
      } else {
        message = tr("This DEF string is already referred to by subsequent USE nodes. "
                     "Applying this change will turn them into copies of the selected node.\n"
                     "Do you want to continue?");
      }

      mMessageBox = false;

      if (OmMessageBox::question(message, this, tr("DEF name change")) == QMessageBox::Cancel) {
        mDefEdit->setText(previousDef);
        mDefEdit->blockSignals(false);
        return;
      }
    }
  }

  mDefEdit->blockSignals(false);

  // apply
  OmToken::makeValidIdentifier(newDef);
  mPreviousValue->setString(previousDef);
  mNewValue->setString(newDef);
  OmValueEditor::apply();

  update();

  if (dictionaryUpdateRequest)
    emit dictionaryUpdateRequested();
}
