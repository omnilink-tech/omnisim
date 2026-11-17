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

#ifndef OM_COLOR_EDITOR_HPP
#define OM_COLOR_EDITOR_HPP

//
// Description: editor for editing a OmSFColor or a OmMFColor item
//

#include "OmRgb.hpp"
#include "OmValueEditor.hpp"

class OmFieldDoubleSpinBox;
class QLabel;
class QToolButton;

class OmColorEditor : public OmValueEditor {
  Q_OBJECT

public:
  explicit OmColorEditor(QWidget *parent = NULL);
  virtual ~OmColorEditor() override;

  void recursiveBlockSignals(bool block) override;

  QWidget *lastEditorWidget() override;

public slots:
  void applyIfNeeded() override;

protected:
  void edit(bool copyOriginalValue) override;
  void resetFocus() override;

protected slots:
  // cppcheck-suppress virtualCallInConstructor
  void apply() override;

private:
  OmRgb mRgb;
  OmFieldDoubleSpinBox *mSpinBoxes[3];
  QLabel *mLabel[3];
  QToolButton *mColorButton;

  void updateSpinBoxes();
  void updateButton();
  void takeKeyboardFocus() override;

private slots:
  void openColorChooser();
};

#endif
