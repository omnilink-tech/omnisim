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

#include "OmNodePane.hpp"

#include "OmField.hpp"
#include "OmNodeEditor.hpp"
#include "OmPhysicsViewer.hpp"
#include "OmPose.hpp"
#include "OmPositionViewer.hpp"
#include "OmSolid.hpp"
#include "OmVelocityViewer.hpp"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QTabWidget>

static const QStringList cTabNames = QStringList() << "Node"
                                                   << "Mass"
                                                   << "Position"
                                                   << "Velocity";

OmNodePane::OmNodePane(QWidget *parent) :
  OmValueEditor(parent),
  mTabs(new QTabWidget(this)),
  mNodeEditor(new OmNodeEditor()),
  mPhysicsViewer(new OmPhysicsViewer()),
  mPositionViewer(new OmPositionViewer()),
  mVelocityViewer(new OmVelocityViewer()),
  mPreviousTabName(cTabNames[NODE_TAB]) {
  // tabs added only when this editor has focus
  // otherwise they affects the minimum size of other editors
  mLayout->addWidget(mTabs, 1, 1);
  mTabs->setObjectName("NodePane");
  connect(mTabs, &QTabWidget::currentChanged, this, &OmNodePane::updateSelectedTab);
  updateSelectedTab();
}

OmNodePane::~OmNodePane() {
  disconnect(mTabs, &QTabWidget::currentChanged, this, &OmNodePane::updateSelectedTab);
  delete mNodeEditor;
  mNodeEditor = NULL;
  delete mPhysicsViewer;
  mPhysicsViewer = NULL;
  delete mPositionViewer;
  mPositionViewer = NULL;
  delete mVelocityViewer;
  mVelocityViewer = NULL;
}

void OmNodePane::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mNodeEditor->recursiveBlockSignals(block);
  mPhysicsViewer->blockSignals(block);
  mPositionViewer->blockSignals(block);
  mVelocityViewer->blockSignals(block);
  mTabs->blockSignals(block);
}

void OmNodePane::edit(OmNode *node, OmField *field, int index) {
  mNodeEditor->OmValueEditor::edit(node, field, index);
  OmValueEditor::edit(node, field, index);
}

void OmNodePane::cleanValue() {
  mPhysicsViewer->clean();
  mPositionViewer->clean();
  mVelocityViewer->clean();
  OmValueEditor::cleanValue();
  mNodeEditor->cleanValue();
}

void OmNodePane::stopEditing() {
  OmValueEditor::stopEditing();
  mNodeEditor->stopEditing();
  mPhysicsViewer->stopUpdating();
  mPositionViewer->stopUpdating();
  mVelocityViewer->stopUpdating();
  mPhysicsViewer->clean();
  mPositionViewer->clean();
  mVelocityViewer->clean();
  mNodeEditor->cleanValue();
  // save last selected tab to restore it when a different node is selected
  mPreviousTabName = mTabs->tabText(mTabs->currentIndex());
  // remove tabs
  disconnect(mTabs, &QTabWidget::currentChanged, this, &OmNodePane::updateSelectedTab);
  mTabs->clear();
  connect(mTabs, &QTabWidget::currentChanged, this, &OmNodePane::updateSelectedTab);
}

void OmNodePane::edit(bool copyOriginalValue) {
  if (copyOriginalValue) {
    const OmField *const f = field();

    OmNode *node = NULL;
    if (singleValue())
      node = static_cast<OmSFNode *>(f->value())->value();
    else if (multipleValue())
      node = static_cast<OmMFNode *>(f->value())->item(index());

    if (node)
      node = static_cast<OmBaseNode *>(node)->getFirstFinalizedProtoInstance();

    // update and add tabs if needed
    mNodeEditor->edit(false);
    enableTab(NODE_TAB, mNodeEditor, true);

    OmPose *t = dynamic_cast<OmPose *>(node);
    OmSolid *s = dynamic_cast<OmSolid *>(node);
    mPhysicsViewer->show(s);
    mPositionViewer->show(t);
    enableTab(POSITION_TAB, mPositionViewer, t != NULL);
    mVelocityViewer->show(s);
    enableTab(VELOCITY_TAB, mVelocityViewer, s != NULL);
  }

  update();
}

void OmNodePane::update() {
  mPositionViewer->update();
  mVelocityViewer->update();

  bool physicsEnabled = mPhysicsViewer->update();
  enableTab(PHYSICS_TAB, mPhysicsViewer, physicsEnabled);
}

void OmNodePane::resetFocus() {
  mNodeEditor->resetFocus();
}

void OmNodePane::apply() {
  if (mTabs->currentWidget() == mNodeEditor)
    mNodeEditor->apply();
}

void OmNodePane::updateSelectedTab() {
  mPhysicsViewer->setSelected(mTabs->currentWidget() == mPhysicsViewer);
  mPositionViewer->setSelected(mTabs->currentWidget() == mPositionViewer);
  mVelocityViewer->setSelected(mTabs->currentWidget() == mVelocityViewer);
}

void OmNodePane::enableTab(int index, QWidget *widget, bool enabled) {
  bool tabExists = false;
  int i = 0;
  for (; i <= index; ++i) {
    if (mTabs->tabText(i) == cTabNames[index]) {
      tabExists = true;
      break;
    }
  }

  if (enabled) {
    if (!tabExists) {
      if (i > mTabs->count())
        i = mTabs->count();
      mTabs->insertTab(i, widget, cTabNames[index]);
    }
    if (cTabNames[index] == mPreviousTabName)
      // restore previously selected tab
      mTabs->setCurrentIndex(i);
  } else if (tabExists)
    mTabs->removeTab(i);
}
