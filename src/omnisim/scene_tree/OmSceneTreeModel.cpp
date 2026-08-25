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

#include "OmSceneTreeModel.hpp"

#include "OmField.hpp"
#include "OmGuiRefreshOracle.hpp"
#include "OmMFNode.hpp"
#include "OmNodeUtilities.hpp"
#include "OmSFNode.hpp"
#include "OmSolid.hpp"
#include "OmTreeItem.hpp"
#include "OmWorld.hpp"

#include <QtGui/QPixmap>

#include <cassert>

OmSceneTreeModel::OmSceneTreeModel(OmGroup *worldRoot) : mRootItem(createItemForField(worldRoot->findField("children"))) {
}

OmSceneTreeModel::~OmSceneTreeModel() {
  delete mRootItem;
}

OmTreeItem *OmSceneTreeModel::createItemForNode(OmNode *node) {
  OmTreeItem *const item = new OmTreeItem(node);

  if (node)
    connect(node, &OmNode::defUseNameChanged, this, &OmSceneTreeModel::updateItemAndChildren);

  // Solid, Device, Joint and JointParameters USE nodes are made expandable and turned into non-USE nodes during dictionary
  // update
  if (node && (!node->isUseNode() || !OmNodeUtilities::isAValidUseableNode(node))) {
    const int n = node->numFields();
    const QVector<OmField *> &fields = node->fieldsOrParameters();
    for (int i = 0; i < n; ++i) {
      OmTreeItem *const child = createItemForField(fields[i]);
      if (child)
        item->appendChild(child);
    }
  }

  return item;
}

void OmSceneTreeModel::createChildrenItemForNode(OmNode *node) {
  assert(node);
  OmTreeItem *const item = findTreeItemFromNode(node, mRootItem);
  assert(item);
  const int n = node->numFields();
  const QVector<OmField *> &fields = node->fieldsOrParameters();
  for (int i = 0; i < n; ++i) {
    OmTreeItem *const child = createItemForField(fields[i]);
    if (child)
      item->appendChild(child);
  }
}

OmTreeItem *OmSceneTreeModel::createItemForField(OmField *field) {
  if (!field || field->isHidden())
    return NULL;

  OmTreeItem *const item = new OmTreeItem(field);

  const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(field->value());
  if (sfnode) {
    connect(item, &OmTreeItem::rowsInserted, this, &OmSceneTreeModel::insertItems);
    connect(item, &OmTreeItem::childrenNeedDeletion, this, &OmSceneTreeModel::removeItems);
    OmNode *const node = sfnode->value();
    if (node) {
      connect(node, &OmNode::defUseNameChanged, this, &OmSceneTreeModel::updateItemAndChildren);
      if (!node->isUseNode()) {
        const int n = node->numFields();
        const QVector<OmField *> &fields = node->fieldsOrParameters();
        for (int i = 0; i < n; ++i) {
          OmTreeItem *const child = createItemForField(fields[i]);
          if (child)
            item->appendChild(child);
        }
      }
    }
    return item;
  }

  connect(item, &OmTreeItem::rowsInserted, this, &OmSceneTreeModel::insertItems);
  connect(item, &OmTreeItem::childrenNeedDeletion, this, &OmSceneTreeModel::removeItems);

  const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(field->value());
  if (mfnode) {
    const int n = mfnode->size();
    for (int i = 0; i < n; ++i)
      item->appendChild(createItemForNode(mfnode->item(i)));
    return item;
  }

  const OmMultipleValue *const mvalue = dynamic_cast<OmMultipleValue *>(field->value());
  if (mvalue) {
    const int n = mvalue->size();
    for (int i = 0; i < n; ++i)
      item->appendChild(new OmTreeItem(field, 0));
    return item;
  }

  return item;
}

QVariant OmSceneTreeModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid())
    return QVariant();

  const OmTreeItem *const item = indexToItem(index);

  switch (role) {
    case Qt::DisplayRole:
      return QVariant(item->data());
    case Qt::DecorationRole:
      return QVariant(item->pixmap());
    case Qt::ToolTipRole:
      return QVariant(item->info());
    default:
      return QVariant();
  }
}

Qt::ItemFlags OmSceneTreeModel::flags(const QModelIndex &index) const {
  if (!index.isValid())
    return Qt::ItemFlags();

  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

QModelIndex OmSceneTreeModel::index(int row, int column, const QModelIndex &parent) const {
  if (!hasIndex(row, column, parent))
    return QModelIndex();

  const OmTreeItem *const parentItem = parent.isValid() ? indexToItem(parent) : mRootItem;
  OmTreeItem *const childItem = parentItem->child(row);

  return childItem ? createIndex(row, column, childItem) : QModelIndex();
}

QModelIndex OmSceneTreeModel::parent(const QModelIndex &index) const {
  if (!index.isValid())
    return QModelIndex();

  OmTreeItem *const childItem = indexToItem(index);
  OmTreeItem *const parentItem = childItem->parent();
  if (parentItem == mRootItem)
    return QModelIndex();

  return createIndex(parentItem->row(), 0, parentItem);
}

int OmSceneTreeModel::rowCount(const QModelIndex &parent) const {
  if (parent.column() > 0)
    return 0;

  const OmTreeItem *const parentItem = parent.isValid() ? indexToItem(parent) : mRootItem;

  return parentItem ? parentItem->childCount() : 0;
}

void OmSceneTreeModel::startWatching(const QModelIndex &index) {
  const OmTreeItem *const item = indexToItem(index);

  if (!item)
    return;

  const int count = item->childCount();

  for (int i = 0; i < count; ++i) {
    const OmTreeItem *const childItem = item->child(i);
    connect(childItem, &OmTreeItem::dataChanged, this, &OmSceneTreeModel::updateData);
  }
}

void OmSceneTreeModel::stopWatching(const QModelIndex &index) {
  const OmTreeItem *const item = indexToItem(index);

  const int count = item->childCount();
  for (int i = 0; i < count; ++i) {
    const OmTreeItem *const childItem = item->child(i);
    disconnect(childItem, &OmTreeItem::dataChanged, this, &OmSceneTreeModel::updateData);
  }
}

OmTreeItem *OmSceneTreeModel::indexToItem(const QModelIndex &index) const {
  return index.isValid() ? static_cast<OmTreeItem *>(index.internalPointer()) : mRootItem;
}

QModelIndex OmSceneTreeModel::itemToIndex(const OmTreeItem *item) const {
  if (item == mRootItem)
    return QModelIndex();

  // create a path from the root item to the specified item
  QList<const OmTreeItem *> ancestors;
  const OmTreeItem *parentItem = item;
  while (parentItem) {
    ancestors.prepend(parentItem);
    parentItem = parentItem->parent();
  }

  // traverse the model from the root index, using the above path
  QModelIndex parentIndex;
  const int as = ancestors.size();
  for (int i = 0; i < as; ++i) {
    const int rows = rowCount(parentIndex);
    for (int j = 0; j < rows; ++j) {
      QModelIndex childIndex = index(j, 0, parentIndex);
      const OmTreeItem *const childItem = indexToItem(childIndex);

      // see if the child item is in the path
      if (childItem == ancestors[i]) {
        // enter this branch from the model
        parentIndex = childIndex;
        break;
      }
    }
  }

  return parentIndex;
}

void OmSceneTreeModel::updateData() {
  OmTreeItem *const item = static_cast<OmTreeItem *>(sender());

  // don't update scene tree item view at each step
  if (!OmGuiRefreshOracle::instance()->canRefreshNow()) {
    item->setDataRefreshNeeded(true);
    return;
  }

  item->setDataRefreshNeeded(false);
  QModelIndex modelIndex = itemToIndex(item);
  emit dataChanged(modelIndex, modelIndex);

  // Without the following line, some values such as MFVector2
  // were not updated correctly when modified from editor
  emit layoutChanged();
}

void OmSceneTreeModel::updateAllSceneTreeValues() {
  const int nRows = mRootItem->childCount();
  QModelIndex topLeft = index(0, 0);
  QModelIndex bottomRight = index(nRows - 1, 0);
  emit dataChanged(topLeft, bottomRight);
}

void OmSceneTreeModel::updateItem(OmTreeItem *item) {
  QModelIndex itemModelIndex = itemToIndex(item);
  emit dataChanged(itemModelIndex, itemModelIndex);
}

void OmSceneTreeModel::updateItemAndChildren(OmNode *node, bool createChildren) {
  if (createChildren)
    createChildrenItemForNode(node);
  QModelIndex itemModelIndex = findModelIndexFromNode(node);
  emit dataChanged(itemModelIndex, itemModelIndex);
}

void OmSceneTreeModel::removeItems(int row, int count) {
  const OmTreeItem *const item = static_cast<OmTreeItem *>(sender());
  const QModelIndex &modelIndex = itemToIndex(item);
  emit rowsAboutToBeRemovedSoon(modelIndex, row, row + count - 1);
  removeRows(row, count, modelIndex);
}

bool OmSceneTreeModel::removeRows(int row, int count, const QModelIndex &parent) {
  beginRemoveRows(parent, row, row + count - 1);

  OmTreeItem *const item = indexToItem(parent);
  if (item->isSFNode())
    item->deleteAllChildren();
  else
    item->deleteChild(row);

  endRemoveRows();

  return true;
}

void OmSceneTreeModel::insertItems(int position, int count) {
  const OmTreeItem *const item = static_cast<OmTreeItem *>(sender());
  const QModelIndex &modelIndex = itemToIndex(item);
  insertRows(position, count, modelIndex);
}

bool OmSceneTreeModel::insertRows(int row, int count, const QModelIndex &parent) {
  beginInsertRows(parent, row, row + count - 1);

  OmTreeItem *const parentItem = indexToItem(parent);

  const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(parentItem->field()->value());
  if (sfnode) {
    OmNode *const node = sfnode->value();
    if (node) {
      if (!node->isUseNode()) {
        const int n = node->numFields();
        for (int i = 0; i < n; ++i) {
          OmTreeItem *const childItem = createItemForField(node->field(i));
          if (childItem) {
            parentItem->appendChild(childItem);
            connect(childItem, &OmTreeItem::dataChanged, this, &OmSceneTreeModel::updateData);
          }
        }
      }

      connect(node, &OmNode::defUseNameChanged, this, &OmSceneTreeModel::updateItemAndChildren, Qt::UniqueConnection);
    }

    // signal OmTreeView
    emit itemInserted(parent);
  } else {
    const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(parentItem->field()->value());

    for (int pos = row; pos < count + row; ++pos) {
      OmTreeItem *const childItem = mfnode ? createItemForNode(mfnode->item(pos)) : new OmTreeItem(parentItem->field(), 0);

      parentItem->insertChild(pos, childItem);
      connect(childItem, &OmTreeItem::dataChanged, this, &OmSceneTreeModel::updateData);

      // signal OmTreeView
      QModelIndex childIndex = itemToIndex(childItem);
      emit itemInserted(childIndex);
    }
  }

  endInsertRows();

  emit dataChanged(parent, parent);
  return true;
}

QModelIndex OmSceneTreeModel::findModelIndexFromNode(OmNode *node, OmTreeItem *current) const {
  if (current->isNode() && current->node() == node)
    return itemToIndex(current);

  if (current->isField() && current->field()->type() == WB_SF_NODE) {
    const OmSFNode *const n = static_cast<OmSFNode *>(current->field()->value());
    if (n->value() == node)
      return itemToIndex(current);
  }

  const int nChild = current->childCount();
  for (int i = 0; i < nChild; ++i) {
    QModelIndex modelIndex = findModelIndexFromNode(node, current->child(i));
    if (modelIndex.isValid())
      return modelIndex;
  }

  return QModelIndex();
}

OmTreeItem *OmSceneTreeModel::findTreeItemFromNode(OmNode *node, OmTreeItem *current) {
  if (current->isNode() && current->node() == node)
    return current;

  if (current->isSFNode()) {
    const OmNode *const candidate = static_cast<OmSFNode *>(current->field()->value())->value();
    if (node == candidate)
      return current;
  }

  const int n = current->childCount();
  for (int i = 0; i < n; ++i) {
    OmTreeItem *const item = findTreeItemFromNode(node, current->child(i));
    if (item)
      return item;
  }

  return NULL;
}

OmTreeItem *OmSceneTreeModel::findUpperNodeItem(const OmTreeItem *item) const {
  const OmTreeItem *i = item;
  while (!i->isNode() && !i->isSFNode()) {
    i = i->parent();
    if (!i)
      return NULL;
  }
  return const_cast<OmTreeItem *>(i);
}

QModelIndex OmSceneTreeModel::findModelIndexFromField(OmField *field, OmTreeItem *current) const {
  if (current->isField() && current->field() == field)
    return itemToIndex(current);

  const int nChild = current->childCount();
  for (int i = 0; i < nChild; ++i) {
    QModelIndex modelIndex = findModelIndexFromField(field, current->child(i));
    if (modelIndex.isValid())
      return modelIndex;
  }

  return QModelIndex();
}

/////////////////////////////////////////////
// Utility functions related to item index //
/////////////////////////////////////////////

int OmSceneTreeModel::itemToTreeIndex(OmTreeItem *item) const {
  const OmTreeItem *const targetItem = item;
  bool itemFound = false;
  int itemIndex = 0;
  const int n = mRootItem->childCount();

  for (int i = 0; !itemFound && i < n; ++i)
    treeIndex(mRootItem->child(i), targetItem, itemFound, itemIndex);

  return itemIndex;
}

void OmSceneTreeModel::treeIndex(const OmTreeItem *const currentItem, const OmTreeItem *const targetItem, bool &itemFound,
                                 int &index) {
  ++index;

  if (currentItem == targetItem) {
    itemFound = true;
    return;
  }

  const int n = currentItem->childCount();
  for (int i = 0; !itemFound && i < n; ++i)
    treeIndex(currentItem->child(i), targetItem, itemFound, index);
}

OmTreeItem *OmSceneTreeModel::treeIndexToItem(int targetIndex) const {
  int itemIndex = targetIndex;
  const int n = mRootItem->childCount();

  for (int i = 0; (itemIndex > 0) && i < n; ++i) {
    OmTreeItem *currentItem = treeIndexToItem(mRootItem->child(i), itemIndex);
    if (itemIndex == 0)
      return currentItem;
  }

  return NULL;
}

OmTreeItem *OmSceneTreeModel::treeIndexToItem(OmTreeItem *currentItem, int &index) {
  --index;

  if (index == 0)
    return currentItem;

  const int n = currentItem->childCount();
  for (int i = 0; (index > 0) && i < n; ++i) {
    OmTreeItem *item = treeIndexToItem(currentItem->child(i), index);
    if (index == 0)
      return item;
  }

  return NULL;
}

// Update the scene tree layout
void OmSceneTreeModel::emitLayoutChanged() {
  emit layoutChanged();
}
