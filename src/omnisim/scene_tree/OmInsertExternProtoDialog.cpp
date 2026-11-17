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

#include "OmInsertExternProtoDialog.hpp"

#include <OmDownloader.hpp>
#include "OmClipboard.hpp"
#include "OmLog.hpp"
#include "OmMessageBox.hpp"
#include "OmNetwork.hpp"
#include "OmPreferences.hpp"
#include "OmProtoManager.hpp"
#include "OmUrl.hpp"

#include <QtCore/QRegularExpression>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QTreeWidgetItem>
#include <QtWidgets/QVBoxLayout>

OmInsertExternProtoDialog::OmInsertExternProtoDialog(QWidget *parent) : QDialog(parent), mRetrievalTriggered(false) {
  QVBoxLayout *const layout = new QVBoxLayout(this);

  mSearchBar = new QLineEdit(this);
  mSearchBar->setClearButtonEnabled(true);

  mTree = new QTreeWidget(this);
  mTree->setHeaderHidden(true);
  connect(mTree, &QTreeWidget::doubleClicked, this, &OmInsertExternProtoDialog::accept);

  // define buttons
  mCancelButton = new QPushButton(tr("Cancel"), this);
  mCancelButton->setFocusPolicy(Qt::ClickFocus);
  mInsertButton = new QPushButton(tr("Insert"), this);
  mInsertButton->setFocusPolicy(Qt::ClickFocus);
  mInsertButton->setEnabled(false);
  connect(mCancelButton, &QPushButton::pressed, this, &OmInsertExternProtoDialog::reject);
  connect(mInsertButton, &QPushButton::pressed, this, &OmInsertExternProtoDialog::accept);

  QDialogButtonBox *const buttonBox = new QDialogButtonBox(this);
  buttonBox->addButton(mCancelButton, QDialogButtonBox::RejectRole);
  buttonBox->addButton(mInsertButton, QDialogButtonBox::AcceptRole);
  // define layout
  layout->addWidget(mSearchBar);
  layout->addWidget(mTree);
  layout->addWidget(buttonBox);

  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &OmInsertExternProtoDialog::updateSelection);

  // retrieve PROTO dependencies of all locally available PROTO prior to generating the dialog
  connect(OmProtoManager::instance(), &OmProtoManager::dependenciesAvailable, this,
          &OmInsertExternProtoDialog::updateProtoTree);
  OmProtoManager::instance()->retrieveLocalProtoDependencies();
}

void OmInsertExternProtoDialog::updateProtoTree() {
  if (qobject_cast<OmProtoManager *>(sender())) {
    disconnect(OmProtoManager::instance(), &OmProtoManager::retrievalCompleted, this,
               &OmInsertExternProtoDialog::updateProtoTree);
    connect(mSearchBar, &QLineEdit::textChanged, this, &OmInsertExternProtoDialog::updateProtoTree);
  }

  mTree->clear();

  QTreeWidgetItem *const worldFileProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Current World File)"), OmProtoManager::PROTO_WORLD);
  QTreeWidgetItem *const projectProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Current Project)"), OmProtoManager::PROTO_PROJECT);
  QTreeWidgetItem *const extraProtosItem =
    new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), OmProtoManager::PROTO_EXTRA);
  QTreeWidgetItem *const omniSimProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (OmniSim Projects)"), OmProtoManager::PROTO_OMNISIM);

  const QRegularExpression regexp(
    QRegularExpression::wildcardToRegularExpression(mSearchBar->text(), QRegularExpression::UnanchoredWildcardConversion),
    QRegularExpression::CaseInsensitiveOption);

  const int categories[4] = {OmProtoManager::PROTO_WORLD, OmProtoManager::PROTO_PROJECT, OmProtoManager::PROTO_EXTRA,
                             OmProtoManager::PROTO_OMNISIM};
  QTreeWidgetItem *const items[4] = {worldFileProtosItem, projectProtosItem, extraProtosItem, omniSimProtosItem};

  QStringList existingImportableExternProto;  // existing importable EXTERNPROTO entries
  QVector<const OmExternProto *> existingInstantiatedExternProto;
  foreach (const OmExternProto *item, OmProtoManager::instance()->externProto()) {
    if (item->isImportable())
      existingImportableExternProto << item->name();
    else
      existingInstantiatedExternProto.append(item);
  }

  for (int i = 0; i < 4; ++i) {
    OmProtoManager::instance()->generateProtoInfoMap(categories[i]);
    QMapIterator<QString, OmProtoInfo *> it(OmProtoManager::instance()->protoInfoMap(categories[i]));
    while (it.hasNext()) {
      const QString protoName = it.next().key();
      const QString &protoUrl = it.value()->url();

      // list only items that aren't in the panel already
      if (existingImportableExternProto.contains(protoName))
        continue;

      // don't display items that have the same name and a different URL as an instantiated PROTO
      bool conflictingInstantiated = false;
      foreach (const OmExternProto *instantiated, existingInstantiatedExternProto) {
        // the URL might differ, but they might point to the same object (ex: one is omnisim://, the other absolute)
        if (instantiated->name() != protoName || OmUrl::resolveUrl(instantiated->url()) == OmUrl::resolveUrl(protoUrl))
          continue;

        conflictingInstantiated = true;
        break;
      }
      if (conflictingInstantiated)
        continue;

      // don't display PROTOs which contain a "hidden" or a "deprecated" tag
      const QStringList tags = it.value()->tags();
      if (tags.contains("deprecated", Qt::CaseInsensitive) || tags.contains("hidden", Qt::CaseInsensitive))
        continue;

      if (protoName.contains(regexp))
        items[i]->addChild(new QTreeWidgetItem(items[i], QStringList(protoName)));
    }
  }

  if (worldFileProtosItem->childCount() > 0)
    mTree->addTopLevelItem(worldFileProtosItem);
  if (projectProtosItem->childCount() > 0)
    mTree->addTopLevelItem(projectProtosItem);
  if (extraProtosItem->childCount() > 0)
    mTree->addTopLevelItem(extraProtosItem);
  if (omniSimProtosItem->childCount() > 0)
    mTree->addTopLevelItem(omniSimProtosItem);

  if (mSearchBar->text().length() > 0)
    mTree->expandAll();
}

void OmInsertExternProtoDialog::accept() {
  if (mTree->selectedItems().size() == 0 || !mInsertButton->isEnabled())
    return;

  const QList<OmExternProto *> &clipboardBuffer = OmProtoManager::instance()->externProtoClipboardBuffer();
  bool conflict = false;
  foreach (const OmExternProto *proto, clipboardBuffer) {
    if (proto && proto->name() == mTree->selectedItems().at(0)->text(0) && !mRetrievalTriggered) {
      conflict = true;
      break;
    }
  }

  if (conflict) {
    const QMessageBox::StandardButton clipboardBufferWarningDialog = OmMessageBox::warning(
      "One or more PROTO nodes with the same name as the one you are about to insert is contained in the clipboard. Do "
      "you want to continue? This operation will clear the clipboard.",
      this, "Warning", QMessageBox::Cancel, QMessageBox::Ok | QMessageBox::Cancel);

    if (clipboardBufferWarningDialog == QMessageBox::Ok) {
      OmProtoManager::instance()->clearExternProtoClipboardBuffer();
      OmClipboard::instance()->clear();
    } else
      return;
  }

  // When declaring an EXTERNPROTO, the associated node and all the sub-proto it depends on are downloaded. Since a-priori is
  // unknown which among them is already available, it must be assumed that none is and therefore this function is called twice,
  // the second time by the retriever, and only then the dialog can be accepted
  if (!mRetrievalTriggered) {
    const QTreeWidgetItem *topLevel = mTree->selectedItems().at(0);
    while (topLevel->parent())
      topLevel = topLevel->parent();

    mProto = mTree->selectedItems().at(0)->text(0);
    mPath = OmProtoManager::instance()->protoUrl(mProto, topLevel->type());
    assert(!mPath.isEmpty());
    if (mPath.isEmpty()) {
      OmLog::error(tr("PROTO '%1' does not belong to category '%2'.").arg(mProto).arg(topLevel->type()));
      return;
    }

    connect(OmProtoManager::instance(), &OmProtoManager::retrievalCompleted, this, &OmInsertExternProtoDialog::accept);
    mRetrievalTriggered = true;  // the second time the accept function is called, no retrieval should occur
    OmProtoManager::instance()->retrieveExternProto(mPath);  // note: already takes care of declaring it
    return;
  }

  // this point should only be reached after the retrieval and therefore from this point the PROTO must be available locally
  if (OmUrl::isWeb(mPath) && !OmNetwork::instance()->isCachedWithMapUpdate(mPath)) {
    OmLog::error(tr("Retrieval of PROTO '%1' was unsuccessful, the asset should be cached but it is not.").arg(mProto));
    QDialog::reject();
  }

  // the addition must be declared as EXTERNPROTO so that it is added to the world file when saving
  OmProtoManager::instance()->declareExternProto(mProto, mPath, true);

  QDialog::accept();
}

void OmInsertExternProtoDialog::updateSelection() {
  if (mTree->selectedItems().size() == 0)
    return;

  const QTreeWidgetItem *const selectedItem = mTree->selectedItems().at(0);
  const QTreeWidgetItem *topLevel = selectedItem;
  while (topLevel->parent())
    topLevel = topLevel->parent();

  if (selectedItem->childCount() > 0 || topLevel == selectedItem) {
    mInsertButton->setEnabled(false);  // selected a category or folder
    return;
  }

  mInsertButton->setEnabled(true);
}
