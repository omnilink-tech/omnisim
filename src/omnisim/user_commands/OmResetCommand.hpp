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

#ifndef OM_RESET_COMMAND_HPP
#define OM_RESET_COMMAND_HPP

//
// Description: Representation of an 'reset' action on field
//              and definition of respective undo and redo functions
//
//              Doesn't support undo() function on SFNode and MFNode fields
//

#include <QtCore/QVector>
#include <QtGui/QUndoCommand>

class OmField;

class OmResetCommand : public QUndoCommand {
public:
  explicit OmResetCommand(OmField *field, QUndoCommand *parent = 0);
  ~OmResetCommand();

  void undo() override;
  void redo() override;

private:
  OmField *mField;
  const OmField *const mPrevField;
};

#endif
