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

#ifndef OM_VECTOR3_EDITOR_HPP
#define OM_VECTOR3_EDITOR_HPP

//
// Description: editor for editing a OmSFVector3 or a OmMFVector3 item
//

#include "OmValueEditor.hpp"
#include "OmVector3.hpp"

class OmFieldDoubleSpinBox;
class QLabel;

class OmVector3Editor : public OmValueEditor {
  Q_OBJECT

public:
  explicit OmVector3Editor(QWidget *parent = NULL);
  virtual ~OmVector3Editor() override;

  void recursiveBlockSignals(bool block) override;

  QWidget *lastEditorWidget() override;

protected:
  void edit(bool copyOriginalValue) override;
  void resetFocus() override;

public slots:
  void applyIfNeeded() override;

protected slots:
  void apply() override;

private slots:
  void updateSpinBoxes();

private:
  OmVector3 mVector3;
  OmFieldDoubleSpinBox *mSpinBoxes[3];
  QLabel *mLabel[3];
  QLabel *mUnitLabel[3];
  void takeKeyboardFocus() override;

  bool mApplied;
};

#endif
