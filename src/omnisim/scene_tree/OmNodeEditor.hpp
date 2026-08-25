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

#ifndef OM_NODE_EDITOR_HPP
#define OM_NODE_EDITOR_HPP

//
// Description: editor for editing a OmSFNode or a OmMFNode item
//

#include "OmValueEditor.hpp"

class OmFieldLineEdit;
class OmNode;

class QCheckBox;
class QLabel;
class QStackedWidget;
class QPushButton;

class OmNodeEditor : public OmValueEditor {
  Q_OBJECT

public:
  explicit OmNodeEditor(QWidget *parent = NULL);

  void recursiveBlockSignals(bool block) override;

  void edit(bool copyOriginalValue) override;
  void stopEditing() override;
  void resetFocus() override;

  void update();

  QWidget *lastEditorWidget() override { return NULL; }

signals:
  void dictionaryUpdateRequested();

public slots:
  void apply() override;
  void cleanValue() override;

protected:
  enum PaneType { DEF_PANE, EMPTY_PANE };

private:
  OmNode *mNode;
  OmFieldLineEdit *mDefEdit;
  QLabel *mUseCount;
  QPushButton *mPrintUrl;
  QLabel *mNbTriangles;
  QStackedWidget *mStackedWidget;
  bool mMessageBox;

  // actions buttons
  QLabel *mShowResizeHandlesLabel;
  QCheckBox *mShowResizeHandlesCheckBox;

  void setTransformActionVisibile(bool visible);
  void takeKeyboardFocus() override {}
  void printUrl();
};

#endif
