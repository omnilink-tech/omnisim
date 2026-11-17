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

#include "OmUndoStack.hpp"

#include "OmActionManager.hpp"
#include "OmNode.hpp"
#include "OmTemplateManager.hpp"

static OmUndoStack *gInstance = NULL;

OmUndoStack *OmUndoStack::instance() {
  if (gInstance == NULL)
    gInstance = new OmUndoStack();

  return gInstance;
}

void OmUndoStack::cleanup() {
  gInstance->clear();
  delete gInstance;
}

OmUndoStack::OmUndoStack() {
  setUndoLimit(50);

  connect(OmTemplateManager::instance(), &OmTemplateManager::postNodeRegeneration, this, &OmUndoStack::clearRequest);
};

OmUndoStack::~OmUndoStack() {
  gInstance = NULL;
}

void OmUndoStack::push(QUndoCommand *cmd) {
  // The clear request mechanism is about avoiding ordering issues.
  // Indeed, the clearRequest slot is fired inside the QUndoStack::push() call,
  // but the undo stack is incremented only at the end of this call.

  mClearRequest = false;

  QUndoStack::push(cmd);  // may change the value of mClearRequest via the clearRequest slot

  // cppcheck-suppress knownConditionTrueFalse
  if (mClearRequest)
    clear();

  updateActions();
  // notify the scene tree that some fields changed in order to update the
  // field editor if needed
  // for example in case of translation using the handles from the 3D window
  emit changed();
}

void OmUndoStack::undo() {
  QUndoStack::undo();
  updateActions();
  emit changed();
}

void OmUndoStack::redo() {
  QUndoStack::redo();
  updateActions();
  emit changed();
}

void OmUndoStack::updateActions() {
  OmActionManager::instance()->setEnabled(OmAction::UNDO, canUndo());
  OmActionManager::instance()->setEnabled(OmAction::REDO, canRedo());
}
