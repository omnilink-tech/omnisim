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

#ifndef OM_LINE_EDIT_HPP
#define OM_LINE_EDIT_HPP

//
// Description: line edit using OmniSim cut/copy/paste and undo/redo handling
//

#include <QtWidgets/QLineEdit>

class OmClipboard;

class OmLineEdit : public QLineEdit {
  Q_OBJECT

public:
  explicit OmLineEdit(QWidget *parent = 0);
  explicit OmLineEdit(const QString &contents, QWidget *parent = 0);
  virtual ~OmLineEdit();

protected:
  void keyPressEvent(QKeyEvent *event) override;

private:
  OmClipboard *mClipboard;

  // cut, copy, paste using OmniSim clipboard
  void cut();
  void copy() const;
  void paste();
};

#endif
