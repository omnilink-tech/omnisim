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

#include "OmTreeItem.hpp"

#include "OmBackground.hpp"
#include "OmCloth.hpp"
#include "OmField.hpp"
#include "OmGranularGroup.hpp"
#include "OmGroup.hpp"
#include "OmMFNode.hpp"
#include "OmMultipleValue.hpp"
#include "OmNode.hpp"
#include "OmNodeModel.hpp"
#include "OmNodeUtilities.hpp"
#include "OmSFNode.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector2.hpp"
#include "OmSoftBody.hpp"
#include "OmViewpoint.hpp"
#include "OmWorldInfo.hpp"

#include <QtGui/QPixmap>

#include <cassert>

static const QString EMPTY_STRING;
// MFFields with a fixed number of rows
const QStringList OmTreeItem::FIXED_ROWS_MFFIELD = QStringList() << "inertiaMatrix"
                                                                 << "centerOfMass";

static bool gUpdatesEnabled = true;

void OmTreeItem::enableUpdates(bool enabled) {
  gUpdatesEnabled = enabled;
}

OmTreeItem::OmTreeItem(OmNode *node) {
  mIsDataRefreshNeeded = false;
  mType = NODE;
  mParent = NULL;
  mNode = node;
  connect(mNode, &QObject::destroyed, this, &OmTreeItem::makeInvalid);
}

OmTreeItem::OmTreeItem(OmField *field) {
  mType = FIELD;
  mParent = NULL;
  mField = field;

  connect(mNode, &QObject::destroyed, this, &OmTreeItem::makeInvalid);

  OmValue *const value = mField->value();
  OmSingleValue *const singleValue = dynamic_cast<OmSingleValue *>(value);
  if (singleValue) {
    const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(value);
    if (sfnode) {
      connect(sfnode, &OmSFNode::changed, this, &OmTreeItem::sfnodeChanged);
      if (sfnode->value())
        connect(sfnode->value(), &OmNode::defUseNameChanged, this, &OmTreeItem::propagateDataChange, Qt::UniqueConnection);
    } else {
      // Main signal
      connect(singleValue, &OmSFNode::changed, this, &OmTreeItem::propagateDataChange);
      // Signal used by translation and rotation fields of Solids and position fields of Joints only
      const QString &fieldName = field->name();
      if (fieldName == "translation") {
        const OmSFVector3 *const translation = dynamic_cast<OmSFVector3 *>(singleValue);
        if (translation)
          connect(translation, &OmSFVector3::changedByOde, this, &OmTreeItem::propagateDataChange);
        else {
          const OmSFVector2 *const translation2 = dynamic_cast<OmSFVector2 *>(singleValue);
          if (translation2)
            connect(translation2, &OmSFVector2::changedByOmniSim, this, &OmTreeItem::propagateDataChange);
        }
      } else if (fieldName == "rotation") {
        const OmSFRotation *const rotation = dynamic_cast<OmSFRotation *>(singleValue);
        if (rotation)
          connect(rotation, &OmSFRotation::changedByOde, this, &OmTreeItem::propagateDataChange);
      } else if (fieldName == "position") {
        const OmSFDouble *const position = dynamic_cast<OmSFDouble *>(singleValue);
        if (position)
          connect(position, &OmSFDouble::changedByOde, this, &OmTreeItem::propagateDataChange);
      }
    }
    return;
  }

  const OmMultipleValue *const multipleValue = static_cast<OmMultipleValue *>(value);
  // slots are executed in the order they have been connected
  if (mField->type() == WB_MF_NODE) {
    connect(multipleValue, &OmMultipleValue::itemChanged, this, &OmTreeItem::emitChildNeedsDeletion);
    connect(multipleValue, &OmMultipleValue::itemChanged, this, &OmTreeItem::addChild);
  } else
    // otherwise there is no need to recreate the item when the value changes
    connect(multipleValue, &OmMultipleValue::itemChanged, this, &OmTreeItem::propagateDataChange);
  connect(multipleValue, &OmMultipleValue::itemRemoved, this, &OmTreeItem::emitChildNeedsDeletion);
  connect(multipleValue, &OmMultipleValue::cleared, this, &OmTreeItem::emitDeleteAllChildren);
  connect(multipleValue, &OmMultipleValue::itemInserted, this, &OmTreeItem::addChild);
}

OmTreeItem::OmTreeItem(OmField *field, int index) {
  mType = ITEM;
  mParent = NULL;
  mField = field;

  connect(mNode, &QObject::destroyed, this, &OmTreeItem::makeInvalid);
}

OmTreeItem::~OmTreeItem() {
  qDeleteAll(mChildren);
}

void OmTreeItem::propagateDataChange() {
  if (gUpdatesEnabled)
    emit dataChanged();
}

void OmTreeItem::refreshData() {
  if (gUpdatesEnabled) {
    mIsDataRefreshNeeded = false;
    emit dataChanged();
  }
}

QString OmTreeItem::data() const {
  if (!gUpdatesEnabled)
    return QString();

  switch (mType) {
    case NODE:
      return mNode->usefulName();
    case FIELD: {
      if (mField->isSingle())
        return QString("%1 %2").arg(mField->name(), mField->value()->toString(OmPrecision::GUI_LOW));
      else
        return mField->name();
    }
    case ITEM: {
      const OmMultipleValue *const value = dynamic_cast<OmMultipleValue *>(mField->value());
      int r = row();
      if (r >= 0 && r < value->size())
        return value->itemToString(r, OmPrecision::GUI_LOW);
      return EMPTY_STRING;
    }
    case INVALID:
      return EMPTY_STRING;
    default:
      assert(false);
      return EMPTY_STRING;
  }
}

const QPixmap &OmTreeItem::pixmap() const {
  static const QPixmap nodePixmap("enabledIcons:node.png");
  static const QPixmap fieldPixmap("enabledIcons:field.png");
  static const QPixmap protoPixmap("enabledIcons:proto.png");
  static const QPixmap nullPixmap;

  switch (mType) {
    case NODE: {
      if (mNode->isProtoInstance())
        return protoPixmap;
      else
        return nodePixmap;
    }
    case FIELD:
      if (isSFNode()) {
        if (node() && node()->isProtoInstance())
          return protoPixmap;
        else
          return nodePixmap;
      } else
        return fieldPixmap;
    case ITEM:
    case INVALID:
      return nullPixmap;
    default:
      assert(false);
      return nullPixmap;
  }
}

const QString &OmTreeItem::info() const {
  switch (mType) {
    case NODE:
      return mNode->info();
    case FIELD: {
      const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(mField->value());
      if (sfnode && sfnode->value())
        return sfnode->value()->info();
    }
    default:
      return EMPTY_STRING;
  }
}

bool OmTreeItem::isDefault() const {
  switch (mType) {
    case NODE:
      return mNode->isDefault();
    case FIELD:
    case ITEM:
      return mField->isDefault();
    case INVALID:
      return true;
    default:
      assert(false);
      return false;
  }
}

int OmTreeItem::row() const {
  if (!mParent)
    return 0;

  return mParent->mChildren.indexOf(const_cast<OmTreeItem *>(this));
}

bool OmTreeItem::isFixedRowsMFitem() const {
  assert(isItem());

  const OmTreeItem *const p = parent();
  if (p == NULL || !p->isField())
    return false;

  const OmField *f = p->field();
  if (f == NULL)
    return false;

  foreach (const QString &name, FIXED_ROWS_MFFIELD) {
    if (f->name() == name)
      return true;
  }

  if (f->internalFields().size() > 0) {
    f = f->internalFields().at(0);
    if (f == NULL)
      return false;
    foreach (const QString &name, FIXED_ROWS_MFFIELD) {
      if (f->name() == name)
        return true;
    }
  }

  return false;
}

bool OmTreeItem::isNonEmptyFixedRowsMFfield() const {
  assert(isField());

  const OmField *f = field();
  if (f == NULL || !f->isMultiple())
    return false;

  foreach (const QString &name, FIXED_ROWS_MFFIELD) {
    if (f->name() == name)
      return !dynamic_cast<OmMultipleValue *>(f->value())->isEmpty();
  }

  if (f->internalFields().size() > 0) {
    f = f->internalFields().at(0);
    if (f == NULL)
      return false;

    foreach (const QString &name, FIXED_ROWS_MFFIELD) {
      if (f->name() == name)
        return !dynamic_cast<OmMultipleValue *>(f->value())->isEmpty();
    }
  }

  return false;
}

bool OmTreeItem::canInsert() const {
  switch (mType) {
    case NODE:
      if (dynamic_cast<OmWorldInfo *>(mNode))
        return false;
      if (dynamic_cast<OmGranularGroup *>(mNode))
        return false;
      // A Cloth is a leaf: its surface is generated from its own scalar fields
      // and streamed straight to WREN, so there is no child node -- no Shape,
      // no geometry, no appearance -- that it could do anything with.
      if (dynamic_cast<OmCloth *>(mNode))
        return false;
      // A SoftBody is a leaf for the same reason, and one step more so: its
      // surface is not even derived from its own fields, it is newton's tet
      // mesh (OmSoftBody.hpp).
      if (dynamic_cast<OmSoftBody *>(mNode))
        return false;
      return true;
    case FIELD: {
      const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(mField->value());
      if (sfnode)
        return !sfnode->value();
      else {
        if (isNonEmptyFixedRowsMFfield())
          return false;
        return mField->isMultiple();
      }
    }
    case ITEM:
      if (isFixedRowsMFitem())
        return false;
      return mParent->isField() && mParent->mField->isMultiple();
    case INVALID:
      return false;
    default:
      assert(false);
      return false;
  }

  return false;
}

bool OmTreeItem::canCopy() const {
  switch (mType) {
    case NODE:
      if (dynamic_cast<OmWorldInfo *>(mNode))
        return false;
      if (dynamic_cast<OmViewpoint *>(mNode))
        return false;
      return true;
    case FIELD: {
      const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(mField->value());
      if (sfnode)
        return sfnode->value() && !sfnode->value()->isUseNode();
      else
        return mField->isSingle();
    }
    case ITEM:
      return true;
    case INVALID:
      return false;
    default:
      assert(false);
      return false;
  }

  return false;
}

bool OmTreeItem::canCut() const {
  switch (mType) {
    case NODE:
      if (dynamic_cast<OmWorldInfo *>(mNode))
        return false;
      if (dynamic_cast<OmViewpoint *>(mNode))
        return false;
      return true;
    case FIELD: {
      const OmSFNode *sfnode = dynamic_cast<OmSFNode *>(mField->value());
      return sfnode && sfnode->value() && !sfnode->value()->isUseNode();
    }
    case ITEM:
      return true;
    case INVALID:
      return false;
    default:
      assert(false);
      return false;
  }

  return false;
}

bool OmTreeItem::canDelete() const {
  switch (mType) {
    case NODE: {
      if (dynamic_cast<OmWorldInfo *>(mNode))
        return false;
      if (dynamic_cast<OmViewpoint *>(mNode))
        return false;
      return true;
    }
    case FIELD: {
      const OmSFNode *sfnode = dynamic_cast<OmSFNode *>(mField->value());
      return sfnode && sfnode->value() != NULL;
    }
    case ITEM: {
      if (isFixedRowsMFitem())
        return false;

      return true;
    }
    case INVALID:
      return false;
    default:
      assert(false);
      return false;
  }
}

void OmTreeItem::del() {
  switch (mType) {
    case NODE:
    case ITEM: {
      OmMultipleValue *mvalue = static_cast<OmMultipleValue *>(mParent->mField->value());
      mvalue->removeItem(row());
      break;
    }
    case FIELD: {
      OmSFNode *sfnode = static_cast<OmSFNode *>(mField->value());
      sfnode->setValue(NULL);
      break;
    }
    case INVALID:
    default:
      assert(false);
  }
}

// invalidate item and sub-items and return the total number of lines (item) to be removed in the Scene Tree
int OmTreeItem::makeInvalid() {
  mType = INVALID;
  mNode = NULL;

  int count = 1;
  foreach (OmTreeItem *c, mChildren)
    count += c->makeInvalid();

  return count;
}

void OmTreeItem::emitChildNeedsDeletion(int row) {
  mChildren.at(row)->makeInvalid();
  emit childrenNeedDeletion(row, 1);
}

void OmTreeItem::emitDeleteAllChildren() {
  for (int i = mChildren.size() - 1; i >= 0; --i)
    mChildren.at(i)->makeInvalid();

  deleteAllChildren();
}

void OmTreeItem::addChild(int row) {
  emit rowsInserted(row, 1);
}

void OmTreeItem::deleteChild(int row) {
  delete mChildren.at(row);
  mChildren.remove(row);
}

void OmTreeItem::deleteAllChildren() {
  qDeleteAll(mChildren);
  mChildren.clear();
}

void OmTreeItem::sfnodeChanged() {
  assert(mType == FIELD);
  const OmSFNode *sfnode = static_cast<OmSFNode *>(mField->value());
  const OmNode *nodeObject = sfnode->value();

  // delete previous children items
  int count = 0;
  foreach (OmTreeItem *c, mChildren)
    count += c->makeInvalid();
  if (count)
    emit childrenNeedDeletion(0, count);

  if (nodeObject) {
    emit rowsInserted(0, 1);
    connect(sfnode->value(), &OmNode::defUseNameChanged, this, &OmTreeItem::propagateDataChange, Qt::UniqueConnection);
  }
}

bool OmTreeItem::isSFNode() const {
  return mType == FIELD && (dynamic_cast<OmSFNode *>(mField->value()) != NULL);
}

OmNode *OmTreeItem::node() const {
  if (mType == NODE)
    return mNode;

  const OmSFNode *sfNode = dynamic_cast<OmSFNode *>(mField->value());
  if (!sfNode)
    return NULL;

  return sfNode->value();
}

int OmTreeItem::itemIndex(const OmTreeItem *item) const {
  return mChildren.indexOf(const_cast<OmTreeItem *>(item));
}

OmTreeItem *OmTreeItem::lastChild() const {
  if (mChildren.isEmpty())
    return NULL;
  return mChildren.last();
}
