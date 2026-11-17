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

#ifndef OM_INT_EDITOR_HPP
#define OM_INT_EDITOR_HPP

//
// Description: editor for editing a OmSFInt or a OmMFInt item
//

#include "OmValueEditor.hpp"

class OmFieldIntSpinBox;

class OmIntEditor : public OmValueEditor {
  Q_OBJECT

public:
  explicit OmIntEditor(QWidget *parent = NULL);
  virtual ~OmIntEditor() override;

  void recursiveBlockSignals(bool block) override;

  QWidget *lastEditorWidget() override;

public slots:
  void applyIfNeeded() override;

protected:
  void edit(bool copyOriginalValue) override;
  void resetFocus() override;

protected slots:
  void apply() override;

private:
  int mInt;
  OmFieldIntSpinBox *mSpinBox;
  void takeKeyboardFocus() override;
};

#endif
