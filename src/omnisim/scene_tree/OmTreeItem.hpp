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

#ifndef OM_TREE_ITEM_HPP
#define OM_TREE_ITEM_HPP

//
// Description: an object that represent a single line in the Scene Tree
//   A Scene Tree line can be:
//     1. a node, e.g. "WorldInfo"
//     2. a field, e.g. "boundingObject NULL", "color 0 1 0"
//     3. an item, e.g. "Author: first name last name <e-mail>"
//

#include <QtCore/QObject>
#include <QtCore/QVector>

class OmNode;
class OmField;
class QPixmap;

class OmTreeItem : public QObject {
  Q_OBJECT

public:
  explicit OmTreeItem(OmNode *node);
  explicit OmTreeItem(OmField *field);
  OmTreeItem(OmField *field, int index);
  virtual ~OmTreeItem();

  OmTreeItem *parent() const { return mParent; }
  void appendChild(OmTreeItem *const child) {
    mChildren.append(child);
    child->mParent = this;
  }
  void insertChild(int index, OmTreeItem *const child) {
    mChildren.insert(index, child);
    child->mParent = this;
  }
  OmTreeItem *child(int row) const { return mChildren.value(row); }
  int childCount() const { return mChildren.count(); }
  void deleteChild(int row);
  void deleteAllChildren();
  QString data() const;
  const QPixmap &pixmap() const;
  const QString &info() const;
  bool isDefault() const;
  bool canDelete() const;
  void del();
  bool canInsert() const;
  bool canCopy() const;
  bool canCut() const;
  int row() const;
  bool isNode() const { return mType == NODE; }
  bool hasNode() const { return mType == NODE || isSFNode(); }
  bool isField() const { return mType == FIELD; }
  bool isSFNode() const;
  bool isItem() const { return mType == ITEM; }
  bool isInvalid() const { return mType == INVALID; }
  OmField *field() const { return (mType == FIELD || mType == ITEM) ? mField : NULL; }
  OmNode *node() const;
  int itemIndex(const OmTreeItem *item) const;
  OmTreeItem *lastChild() const;

  bool isDataRefreshNeeded() const { return mIsDataRefreshNeeded; }
  void setDataRefreshNeeded(bool b) { mIsDataRefreshNeeded = b; }
  void refreshData();

  static void enableUpdates(bool enabled);

signals:
  void dataChanged();
  void childrenNeedDeletion(int row, int count);
  void rowsInserted(int row, int count);

public slots:
  void propagateDataChange();
  int makeInvalid();

private slots:
  void sfnodeChanged();
  void emitChildNeedsDeletion(int row);
  void emitDeleteAllChildren();
  void addChild(int row);

private:
  enum Type { ROOT, NODE, FIELD, ITEM, INVALID };

  Type mType;
  OmTreeItem *mParent;
  QVector<OmTreeItem *> mChildren;
  union {
    OmNode *mNode;
    OmField *mField;
  };
  bool mIsDataRefreshNeeded;

  static const QStringList FIXED_ROWS_MFFIELD;
  bool isFixedRowsMFitem() const;
  bool isNonEmptyFixedRowsMFfield() const;
};

#endif
