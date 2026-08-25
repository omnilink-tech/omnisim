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

#ifndef OM_UPDATED_DIALOG_HPP
#define OM_UPDATED_DIALOG_HPP

//
// Description: Updated dialog for the first launch of Webots 2022a.
//

#include <QtWidgets/QDialog>

class OmUpdatedDialog : public QDialog {
  Q_OBJECT
public:
  explicit OmUpdatedDialog(QWidget *parent = NULL);
  virtual ~OmUpdatedDialog() {}
};

#endif
