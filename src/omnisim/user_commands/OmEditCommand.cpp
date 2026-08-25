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

#include "OmEditCommand.hpp"
#include "OmMFBool.hpp"
#include "OmMFColor.hpp"
#include "OmMFDouble.hpp"
#include "OmMFInt.hpp"
#include "OmMFNode.hpp"
#include "OmMFRotation.hpp"
#include "OmMFString.hpp"
#include "OmMFVector2.hpp"
#include "OmMFVector3.hpp"
#include "OmNode.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFNode.hpp"
#include "OmSFRotation.hpp"
#include "OmSFString.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"
#include "OmValue.hpp"
#include "OmVariant.hpp"

#include <cassert>

OmEditCommand::OmEditCommand(OmValue *fieldValue, const OmVariant &prevValue, const OmVariant &nextValue, int index,
                             QUndoCommand *parent) :
  QUndoCommand(parent),
  mFieldValue(fieldValue),
  mPrevValue(prevValue),
  mNextValue(nextValue),
  mIndex(index) {
  assert(mFieldValue && (mFieldValue->isMultiple() ^ (mIndex < 0)));
  assert((OmValue::toSingle(mFieldValue->type()) == WB_SF_NODE) ||
         ((OmValue::toSingle(mFieldValue->type()) == prevValue.type()) && (prevValue.type() == nextValue.type())));
  setText(QObject::tr("edit"));
}

void OmEditCommand::undo() {
  resetValue(mPrevValue);
}

void OmEditCommand::redo() {
  resetValue(mNextValue);
}

void OmEditCommand::resetValue(const OmVariant &newValue) {
  switch (mFieldValue->type()) {
    case WB_MF_VEC2F:
      dynamic_cast<OmMFVector2 *>(mFieldValue)->setItem(mIndex, newValue.toVector2());
      break;
    case WB_SF_VEC2F:
      static_cast<OmSFVector2 *>(mFieldValue)->setValue(newValue.toVector2());
      break;
    case WB_MF_VEC3F:
      static_cast<OmMFVector3 *>(mFieldValue)->setItem(mIndex, newValue.toVector3());
      break;
    case WB_SF_VEC3F:
      static_cast<OmSFVector3 *>(mFieldValue)->setValueByUser(newValue.toVector3(), false);
      break;
    case WB_MF_COLOR:
      static_cast<OmMFColor *>(mFieldValue)->setItem(mIndex, newValue.toColor());
      break;
    case WB_SF_COLOR:
      static_cast<OmSFColor *>(mFieldValue)->setValue(newValue.toColor());
      break;
    case WB_MF_STRING:
      static_cast<OmMFString *>(mFieldValue)->setItem(mIndex, newValue.toString());
      break;
    case WB_SF_STRING:
      static_cast<OmSFString *>(mFieldValue)->setValue(newValue.toString());
      break;
    case WB_MF_INT32:
      static_cast<OmMFInt *>(mFieldValue)->setItem(mIndex, newValue.toInt());
      break;
    case WB_SF_INT32:
      static_cast<OmSFInt *>(mFieldValue)->setValue(newValue.toInt());
      break;
    case WB_MF_FLOAT:
      static_cast<OmMFDouble *>(mFieldValue)->setItem(mIndex, newValue.toDouble());
      break;
    case WB_SF_FLOAT:
      static_cast<OmSFDouble *>(mFieldValue)->setValue(newValue.toDouble());
      break;
    case WB_MF_ROTATION:
      static_cast<OmMFRotation *>(mFieldValue)->setItem(mIndex, newValue.toRotation());
      break;
    case WB_SF_ROTATION:
      static_cast<OmSFRotation *>(mFieldValue)->setValueByUser(newValue.toRotation(), false);
      break;
    case WB_MF_BOOL:
      static_cast<OmMFBool *>(mFieldValue)->setItem(mIndex, newValue.toBool());
      break;
    case WB_SF_BOOL:
      static_cast<OmSFBool *>(mFieldValue)->setValue(newValue.toBool());
      break;
    case WB_MF_NODE:
      assert(newValue.type() == WB_SF_STRING);
      static_cast<OmMFNode *>(mFieldValue)->item(mIndex)->setDefName(newValue.toString(), true);
      break;
    case WB_SF_NODE:
      assert(newValue.type() == WB_SF_STRING);
      static_cast<OmSFNode *>(mFieldValue)->value()->setDefName(newValue.toString(), true);
      break;
    default:
      assert(false);
      break;
  }
}
