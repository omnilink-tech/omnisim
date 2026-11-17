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

#ifndef OM_EDIT_COMMAND_HPP
#define OM_EDIT_COMMAND_HPP

//
// Description: Representation of an 'edit' action on field or nodes DEF name
//              and definition of respective undo and redo functions
//

#include <QtGui/QUndoCommand>

#include "OmVariant.hpp"

class OmValue;

class OmEditCommand : public QUndoCommand {
public:
  OmEditCommand(OmValue *fieldValue, const OmVariant &prevValue, const OmVariant &nextValue, int index = -1,
                QUndoCommand *parent = 0);
  ~OmEditCommand() {}

  void undo() override;
  void redo() override;

private:
  OmValue *mFieldValue;
  const OmVariant mPrevValue;
  const OmVariant mNextValue;
  const int mIndex;

  void resetValue(const OmVariant &newValue);
};

#endif
