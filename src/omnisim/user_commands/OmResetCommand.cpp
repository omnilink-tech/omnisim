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

#include "OmResetCommand.hpp"
#include "OmField.hpp"
#include "OmTemplateManager.hpp"

#include <cassert>

OmResetCommand::OmResetCommand(OmField *field, QUndoCommand *parent) :
  QUndoCommand(parent),
  mField(field),
  mPrevField(new OmField(*field, field->parentNode())) {
  assert(mField);
  setText(QObject::tr("reset"));
}

OmResetCommand::~OmResetCommand() {
  delete mPrevField;
}

void OmResetCommand::undo() {
  mField->setValue(mPrevField->value());
}

void OmResetCommand::redo() {
  // temporarily block the regeneration, otherwise the field becomes invalid when resetting MF fields
  OmTemplateManager::instance()->blockRegeneration(true);
  mField->reset();
  OmTemplateManager::instance()->blockRegeneration(false);
}
