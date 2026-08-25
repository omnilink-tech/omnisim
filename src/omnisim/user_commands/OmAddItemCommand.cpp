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

#include "OmAddItemCommand.hpp"

#include "OmField.hpp"
#include "OmMFBool.hpp"
#include "OmMFColor.hpp"
#include "OmMFDouble.hpp"
#include "OmMFInt.hpp"
#include "OmMFRotation.hpp"
#include "OmMFString.hpp"
#include "OmMFVector2.hpp"
#include "OmMFVector3.hpp"
#include "OmMultipleValue.hpp"
#include "OmSingleValue.hpp"

#include <cassert>

OmAddItemCommand::OmAddItemCommand(OmField *const field, OmMultipleValue *fieldValue, int index, QUndoCommand *parent) :
  QUndoCommand(parent),
  mFieldValue(fieldValue),
  mIndex(index) {
  assert(mIndex >= 0 && mFieldValue);
  setText(QObject::tr("add item"));
  if (field->hasRestrictedValues())
    mItem = field->acceptedValues()[0];
}

OmAddItemCommand::OmAddItemCommand(OmMultipleValue *fieldValue, const OmVariant &item, int index, QUndoCommand *parent) :
  QUndoCommand(parent),
  mFieldValue(fieldValue),
  mItem(item),
  mIndex(index) {
  assert(mIndex >= 0 && mFieldValue);
  assert(mItem.isEmpty() || OmValue::toSingle(mFieldValue->type()) == mItem.type());
  setText(QObject::tr("add item"));
}

void OmAddItemCommand::undo() {
  mFieldValue->removeItem(mIndex);
}

void OmAddItemCommand::redo() {
  if (mItem.isEmpty()) {
    mFieldValue->insertDefaultItem(mIndex);
    return;
  }

  switch (mFieldValue->type()) {
    case WB_MF_VEC2F:
      dynamic_cast<OmMFVector2 *>(mFieldValue)->insertItem(mIndex, mItem.toVector2());
      break;
    case WB_MF_VEC3F:
      static_cast<OmMFVector3 *>(mFieldValue)->insertItem(mIndex, mItem.toVector3());
      break;
    case WB_MF_COLOR:
      static_cast<OmMFColor *>(mFieldValue)->insertItem(mIndex, mItem.toColor());
      break;
    case WB_MF_STRING:
      static_cast<OmMFString *>(mFieldValue)->insertItem(mIndex, mItem.toString());
      break;
    case WB_MF_INT32:
      static_cast<OmMFInt *>(mFieldValue)->insertItem(mIndex, mItem.toInt());
      break;
    case WB_MF_FLOAT:
      static_cast<OmMFDouble *>(mFieldValue)->insertItem(mIndex, mItem.toDouble());
      break;
    case WB_MF_ROTATION:
      static_cast<OmMFRotation *>(mFieldValue)->insertItem(mIndex, mItem.toRotation());
      break;
    case WB_MF_BOOL:
      static_cast<OmMFBool *>(mFieldValue)->insertItem(mIndex, mItem.toBool());
      break;
    default:
      assert(false);
      break;
  }
}
