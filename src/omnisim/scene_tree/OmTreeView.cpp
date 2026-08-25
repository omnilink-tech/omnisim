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

#include "OmTreeView.hpp"

#include "OmActionManager.hpp"
#include "OmContextMenuGenerator.hpp"
#include "OmNodeOperations.hpp"
#include "OmTreeItem.hpp"

#include <QtWidgets/QHeaderView>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QStyle>
#include <QtWidgets/QStyledItemDelegate>

class OmTreeItemDelegate : public QStyledItemDelegate {
public:
  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override {
    QStyleOptionViewItem itemOption(option);

    const OmTreeItem *item = static_cast<OmTreeItem *>(index.internalPointer());
    if (item->isDefault())
      // paint unmodified tree items in black
      itemOption.palette.setColor(QPalette::Text, mDefaultColor);
    else
      // paint modified tree items in dark cyan
      itemOption.palette.setColor(QPalette::Text, mModifiedColor);

    // call the base class method for drawing the normal
    // parts of the item of the QTreeView.
    QStyledItemDelegate::paint(painter, itemOption, index);
  }

  void setDefaultColor(const QColor &color) { mDefaultColor = color; }

  void setModifiedColor(const QColor &color) { mModifiedColor = color; }

private:
  QColor mDefaultColor, mModifiedColor;
};

OmTreeView::OmTreeView(QWidget *parent) : QTreeView(parent) {
  setObjectName("TreeView");

  mIsScrollActive = false;
  mTreeItemDelegate = new OmTreeItemDelegate();
  style()->polish(this);
  mTreeItemDelegate->setDefaultColor(defaultColor());
  mTreeItemDelegate->setModifiedColor(modifiedColor());
  setItemDelegate(mTreeItemDelegate);

  // display an horizontal scroll bar rather than
  // cutting strings with '...' characters
  header()->setSectionResizeMode(QHeaderView::ResizeToContents);
  header()->setStretchLastSection(false);
  header()->setDefaultSectionSize(10000);

  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, &QTreeView::customContextMenuRequested, this, &OmTreeView::showMenu);
}

OmTreeView::~OmTreeView() {
  delete mTreeItemDelegate;
}

void OmTreeView::focusInEvent(QFocusEvent *event) {
  QTreeView::focusInEvent(event);
  OmActionManager::instance()->enableTextEditActions(false, true);
  OmActionManager::instance()->setFocusObject(this);

  // when this widget gets keyboard focus the higlighted color of the current
  // item does not change unless we force Qt to redraw the tree view, so do
  // this by refreshing the item model
  emit refreshRequested();

  emit focusIn();
}

void OmTreeView::focusOutEvent(QFocusEvent *event) {
  if (OmActionManager::instance()->focusObject() == this)
    OmActionManager::instance()->setFocusObject(NULL);
  emit refreshRequested();
}

void OmTreeView::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Left) {
    if (currentIndex().parent() != rootIndex() && !isExpanded(currentIndex()))
      setCurrentIndex(currentIndex().parent());
    else if (isExpanded(currentIndex()))
      collapse(currentIndex());
  } else if (event->key() == Qt::Key_Right && isExpanded(currentIndex()))
    setCurrentIndex(currentIndex().model()->index(0, 0, currentIndex()));
  else if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)
    emit doubleClickOrEnterPressed();
  else
    QTreeView::keyPressEvent(event);

  scrollToSelection();
}

void OmTreeView::currentChanged(const QModelIndex &current, const QModelIndex &previous) {
  emit selectionHasChanged();
}

void OmTreeView::itemInserted(const QModelIndex &index) {
  if (!OmNodeOperations::instance()->isFromSupervisor())
    // select new tree item
    setCurrentIndex(index);
}

void OmTreeView::showMenu(const QPoint &position) {
  emit beforeContextMenuShowed();
  const QModelIndexList indexes = selectionModel()->selectedIndexes();
  if (indexes.isEmpty())
    return;
  const OmTreeItem *item = static_cast<OmTreeItem *>(indexes.at(0).internalPointer());
  assert(item);
  OmContextMenuGenerator::generateContextMenu(mapToGlobal(position), item->node(), NULL);
}

void OmTreeView::scrollToModelIndex(const QModelIndex &index) {
  mIsScrollActive = true;
  scrollTo(index);
  mIsScrollActive = false;
}

void OmTreeView::scrollTo(const QModelIndex &index, QTreeView::ScrollHint hint) {
  if (!mIsScrollActive)
    return;

  QTreeView::scrollTo(index, hint);

  if (!index.isValid())
    return;

  // compute improved horizontal scroll position
  int level = -1;
  QModelIndex parentIndex = index.parent();
  while (parentIndex.isValid()) {
    ++level;
    parentIndex = parentIndex.parent();
  }
  horizontalScrollBar()->setValue(indentation() * level);
}

void OmTreeView::mouseDoubleClickEvent(QMouseEvent *event) {
  emit doubleClickOrEnterPressed();
}
