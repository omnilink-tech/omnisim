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

#ifndef OM_UNDO_STACK_HPP
#define OM_UNDO_STACK_HPP

//
// Description: Singleton class representing the undo/redo stack of world changes
//

#include <QtGui/QUndoStack>

class OmUndoStack : public QUndoStack {
  Q_OBJECT

public:
  static OmUndoStack *instance();
  static void cleanup();

  virtual void push(QUndoCommand *cmd);

signals:
  void changed();

public slots:
  virtual void undo();
  virtual void redo();

private slots:

  void clearRequest() { mClearRequest = true; }

private:
  OmUndoStack();
  virtual ~OmUndoStack();

  void updateActions();

  bool mClearRequest;
};

#endif
