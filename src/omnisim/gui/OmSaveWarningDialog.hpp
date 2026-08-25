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

#ifndef OM_SAVE_WARNING_DIALOG_HPP
#define OM_SAVE_WARNING_DIALOG_HPP

//
// Description: dialog offering the user to save a modified before closing it
//

#include <QtWidgets/QMessageBox>

class OmSaveWarningDialog : public QMessageBox {
  Q_OBJECT

public:
  OmSaveWarningDialog(const QString &world, bool hideCheckBox, bool reloading, QWidget *parent = NULL);
  virtual ~OmSaveWarningDialog() {}

private slots:
  void disableSaveWarning(bool);
};

#endif
