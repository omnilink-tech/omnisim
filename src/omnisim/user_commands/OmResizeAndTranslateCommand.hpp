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

#ifndef OM_RESIZE_AND_TRANSLATE_COMMAND_HPP
#define OM_RESIZE_AND_TRANSLATE_COMMAND_HPP

//
// Description: Representation of 'resize' and 'rescale' actions for indexed face set geometries
//              and definition of respective undo and redo functions
//

#include "OmResizeCommand.hpp"

class OmGeometry;

class OmResizeAndTranslateCommand : public OmResizeCommand {
public:
  OmResizeAndTranslateCommand(OmGeometry *geometry, const OmVector3 &scale, QUndoCommand *parent = 0);
  OmResizeAndTranslateCommand(OmGeometry *geometry, const OmVector3 &scale, const OmVector3 &translation,
                              QUndoCommand *parent = 0);
  ~OmResizeAndTranslateCommand() {}

  void undo() override;
  void redo() override;

private:
  bool mIsTranslationSet;
  const OmVector3 mTranslation;

  void resetValue(bool invertedAction);
};

#endif
