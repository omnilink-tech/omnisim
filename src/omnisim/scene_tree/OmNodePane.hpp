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

#ifndef OM_NODE_PANE_HPP
#define OM_NODE_PANE_HPP

//
// Description: pane containing viewers and editors for a OmSFNode or a OmMFNode item
//

#include "OmValueEditor.hpp"

class OmNodeEditor;
class OmPhysicsViewer;
class OmPositionViewer;
class OmVelocityViewer;

class QHBoxLayout;
class QTabWidget;

class OmNodePane : public OmValueEditor {
  Q_OBJECT

public:
  explicit OmNodePane(QWidget *parent = NULL);
  virtual ~OmNodePane() override;

  void recursiveBlockSignals(bool block) override;

  void edit(OmNode *node, OmField *field, int index) override;
  void edit(bool copyOriginalValue) override;
  void stopEditing() override;

  const OmNodeEditor *nodeEditor() const { return mNodeEditor; }

  QWidget *lastEditorWidget() override { return NULL; }

public slots:
  void cleanValue() override;

protected:
  void resetFocus() override;

protected slots:
  void apply() override;

private:
  enum TabIndex { NODE_TAB = 0, PHYSICS_TAB, POSITION_TAB, VELOCITY_TAB };

  // Tab widgets
  QTabWidget *mTabs;
  OmNodeEditor *mNodeEditor;
  OmPhysicsViewer *mPhysicsViewer;
  OmPositionViewer *mPositionViewer;
  OmVelocityViewer *mVelocityViewer;
  // save the selected tab name to restore it when a different node is selected
  QString mPreviousTabName;

  void update();
  void enableTab(int index, QWidget *widget, bool enabled);
  void takeKeyboardFocus() override {}

private slots:
  void updateSelectedTab();
};

#endif
