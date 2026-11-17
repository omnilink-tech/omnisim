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

#include "OmGuidedTour.hpp"

#include "OmApplication.hpp"
#include "OmConsole.hpp"
#include "OmMFString.hpp"
#include "OmSFString.hpp"
#include "OmSimulationState.hpp"
#include "OmStandardPaths.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"

#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtCore/QTimer>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QTreeWidgetItem>

#include <float.h>

static OmGuidedTour *gInstance = NULL;

OmGuidedTour *OmGuidedTour::instance(QWidget *parent) {
  if (!gInstance)
    gInstance = new OmGuidedTour(parent);

  return gInstance;
}

OmGuidedTour::OmGuidedTour(QWidget *parent) :
  QDialog(parent, Qt::Tool) {  // Qt::Tool allows to handle well the z-order. This is mainly advantageous on Mac
  mIndex = -1;
  mDeadline = DBL_MAX;
  mReady = true;

  setAttribute(Qt::WA_DeleteOnClose, true);

  mTimer = new QTimer(this);
  connect(mTimer, &QTimer::timeout, this, &OmGuidedTour::shoot);
  mTimer->start(250);  // trigger every 250 milliseconds

  setWindowTitle(tr("Guided Tour - OmniSim"));
  setWindowOpacity(0.95);
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  QHBoxLayout *mainLayout = new QHBoxLayout(this);
  QVBoxLayout *rightPaneLayout = new QVBoxLayout();
  QHBoxLayout *headerLayout = new QHBoxLayout();
  QHBoxLayout *buttonLayout = new QHBoxLayout();

  mTree = new QTreeWidget(this);
  mTree->setHeaderHidden(true);
  mTree->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &OmGuidedTour::selectWorld);

  QPixmap pixmap("coreIcons:omnisim64x64.png");
  QLabel *pixmapLabel = new QLabel(this);
  pixmapLabel->setPixmap(pixmap);

  mTitleLabel = new QLabel(this);
  setTitleText(tr("OmniSim Guided Tour!"));

  headerLayout->addWidget(pixmapLabel);
  headerLayout->addSpacing(20);
  headerLayout->addWidget(mTitleLabel);
  headerLayout->addStretch();

  mInfoText =
    new QPlainTextEdit(tr("Welcome to the OmniSim Guided Tour.") + "\n" +
                         tr("The tour will take you through many examples and will give you an overview of OmniSim features.") +
                         "\n\n" + tr("Check [Auto] or press [Next] to start...") + "\n",
                       this);
  mInfoText->setReadOnly(true);

  mAutoBox = new QCheckBox(tr("Auto"), this);
  mPrevButton = new QPushButton(tr("Previous"), this);
  mPrevButton->setEnabled(false);
  mNextButton = new QPushButton(tr("Next"), this);
  mNextButton->setDefault(true);
  mNextButton->setAutoDefault(true);
  QPushButton *closeButton = new QPushButton(tr("Close"), this);

  connect(closeButton, &QPushButton::pressed, this, &OmGuidedTour::close);
  connect(mPrevButton, &QPushButton::pressed, this, &OmGuidedTour::prev);
  connect(mNextButton, &QPushButton::pressed, this, &OmGuidedTour::next);
  connect(mAutoBox, &QCheckBox::clicked, this, &OmGuidedTour::setSimulationDeadline);

  buttonLayout->addWidget(mAutoBox);
  buttonLayout->addSpacing(100);
  buttonLayout->addWidget(mPrevButton);
  buttonLayout->addWidget(mNextButton);
  buttonLayout->addWidget(closeButton);

  rightPaneLayout->addLayout(headerLayout);
  rightPaneLayout->addWidget(mInfoText);
  rightPaneLayout->addLayout(buttonLayout);

  mainLayout->addWidget(mTree);
  mainLayout->addLayout(rightPaneLayout);

  loadWorldList();
  updateGUI();
  connect(OmApplication::instance(), &OmApplication::worldLoadCompleted, this, &OmGuidedTour::worldLoaded,
          Qt::UniqueConnection);
}

OmGuidedTour::~OmGuidedTour() {
  delete mTimer;
  gInstance = NULL;
}

void OmGuidedTour::loadWorldList() {
  QFile file(OmStandardPaths::projectsPath() + "guided_tour.txt");
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    return;

  QTreeWidgetItem *treeWidgetItem = NULL;
  QTextStream stream(&file);
  while (!stream.atEnd()) {
    QString line = stream.readLine();
    if (line.startsWith("#") || line.isEmpty())
      continue;
    if (line.startsWith("[")) {
      const int p = line.indexOf("]");
      const QString &title = line.mid(1, p - 1);
      treeWidgetItem = new QTreeWidgetItem(QStringList(title));
      mTree->addTopLevelItem(treeWidgetItem);
      mSections.append(treeWidgetItem);
      continue;
    }
    if (treeWidgetItem == NULL)  // ignore files if a section was not declared
      continue;
    const QStringList list = line.split(" ");
    mFilenames.append(list[0]);
    int duration;
    if (list.size() > 1)
      duration = list[1].toInt();
    else
      duration = 20;  // default to 20 seconds
    mDurations.append(duration);
    const QString &title = list[0].mid(list[0].lastIndexOf("/") + 1);
    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidgetItem, QStringList(title));
    item->setIcon(0, QIcon("coreIcons:omnisim_doc.png"));
    treeWidgetItem->addChild(item);
    mWorlds.append(item);
  }
}

void OmGuidedTour::setTitleText(const QString &title) {
  mTitleLabel->setText("<h2>" + title + "</h2>");
}

static QString formatInfo(const OmMFString &info) {
  QString outputText;
  QString item;
  for (int i = 0; i < info.size(); i++) {
    item = info.item(i);
    const QString lowerCaseItem = item.toLower();
    if (!lowerCaseItem.startsWith("date")) {
      if (item.contains("Author", Qt::CaseInsensitive)) {
        item.replace(QString("Author"), QString("Credits"), Qt::CaseInsensitive);
        outputText += "\n" + item + "\n";
      } else if (item.contains("Authors", Qt::CaseInsensitive)) {
        item.replace(QString("Authors"), QString("Credits"), Qt::CaseInsensitive);
        outputText += "\n" + item + "\n";
      } else
        outputText += item + "\n";
    }
  }
  return outputText;
}

void OmGuidedTour::worldLoaded() {
  mReady = true;
}

void OmGuidedTour::updateGUI() {
  if (mFilenames.isEmpty()) {
    setTitleText(tr("Internal error"));
    mInfoText->setPlainText(tr("The Guided Tour is not available."));
  } else if (mIndex == -1) {
    setTitleText(tr("OmniSim Guided Tour"));
    mInfoText->setPlainText(tr("Welcome to the OmniSim Guided Tour.") + "\n" +
                            tr("The tour will take you through many examples and "
                               "will give you an overview of OmniSim features.") +
                            "\n\n" + tr("Check [Auto] or press [Next] to start..."));
  } else if (mIndex == mFilenames.size()) {
    setTitleText(tr("That's all Folks!"));
    mInfoText->setPlainText(tr("Thanks for viewing the OmniSim Guided Tour.") + "\n\n" + tr("Press [Close] to terminate..."));
  } else {  // Normal case
    // Sets world's title
    if (!OmWorld::instance()->fileName().endsWith(mFilenames[mIndex])) {
      // New world still loading
      // Reset title and info until correct info is available
      const QString &title = mFilenames[mIndex].mid(mFilenames[mIndex].lastIndexOf("/") + 1);
      setTitleText(title + QString(" (%1/%2)").arg(mIndex + 1).arg(mFilenames.size()));
      mInfoText->setPlainText(tr("Loading..."));
      connect(OmApplication::instance(), &OmApplication::worldLoadCompleted, this, &OmGuidedTour::updateGUI,
              Qt::UniqueConnection);
    } else {
      disconnect(OmApplication::instance(), &OmApplication::worldLoadCompleted, this, &OmGuidedTour::updateGUI);
      // Formats and displays all WorldInfo.info items
      setTitleText(OmWorld::instance()->worldInfo()->title() + QString(" (%1/%2)").arg(mIndex + 1).arg(mFilenames.size()));
      const OmMFString &info = OmWorld::instance()->worldInfo()->info();
      mInfoText->setPlainText(formatInfo(info));
    }
  }
  // Updates buttons
  mNextButton->setEnabled(mIndex < (mFilenames.size() - 1));
  mPrevButton->setEnabled(mIndex > 0);
  mAutoBox->setEnabled(!mFilenames.isEmpty());
}

void OmGuidedTour::prev() {
  if (!mReady)
    return;
  mIndex--;
  selectCurrent();
  mAutoBox->setChecked(false);
  mDeadline = DBL_MAX;
  loadWorld();
}

void OmGuidedTour::next() {
  if (!mReady)
    return;
  mIndex++;
  selectCurrent();
  mAutoBox->setChecked(false);
  mDeadline = DBL_MAX;
  loadWorld();
}

void OmGuidedTour::nextWorld() {
  // Called only if mDeadline was reached; most probably mAutoBox->isChecked() = true
  mIndex = (mIndex + 1) % mFilenames.size();  // loop
  selectCurrent();
  setSimulationDeadline(mAutoBox->isChecked());
  loadWorld();
}

void OmGuidedTour::selectCurrent() {
  disconnect(mTree, &QTreeWidget::itemSelectionChanged, this, &OmGuidedTour::selectWorld);
  for (int i = 0; i < mWorlds.size(); i++)
    mWorlds[i]->setSelected(i == mIndex);
  for (int i = 0; i < mSections.size(); i++) {
    mSections[i]->setSelected(false);
    if (mWorlds[mIndex]->parent() == mSections[i])
      mSections[i]->setExpanded(true);
  }
  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &OmGuidedTour::selectWorld);
  mTree->scrollToItem(mWorlds[mIndex]);
}

void OmGuidedTour::shoot() {
  // Called by mTimer every 250 milliseconds
  if (mReady && OmSimulationState::instance()->time() >= mDeadline)
    nextWorld();
}

void OmGuidedTour::setSimulationDeadline(bool autoChecked) {
  // On the first user-click
  if (mIndex < 0 && mReady) {
    nextWorld();
    return;
  }
  if (mIndex >= mFilenames.size())  // last world
    return;
  if (autoChecked)
    mDeadline = 1000 * mDurations[mIndex];
  else
    mDeadline = DBL_MAX;
}

void OmGuidedTour::loadWorld() {
  if (mIndex < 0 || mIndex >= mFilenames.size())
    return;
  const QString &fn = OmStandardPaths::omniSimHomePath()
#ifdef __APPLE__
                      + "Contents/"
#endif
                      + mFilenames[mIndex];
  assert(mReady);
  mReady = false;
  emit loadWorldRequest(fn);  // Load now!
  updateGUI();
}

void OmGuidedTour::selectWorld() {
  // prevent selecting a new world if in the process of loading, canceling the previous one or if invalid
  if (!mReady || mTree->selectedItems().size() < 1 || OmApplication::instance()->wasWorldLoadingCanceled())
    return;
  QTreeWidgetItem *item = mTree->selectedItems().at(0);
  mIndex = mWorlds.indexOf(item);
  if (mIndex < 0) {  // section is selected
    if (item->childCount() > 0) {
      item->setExpanded(true);
      // select first world of the section
      mTree->blockSignals(true);  // emit signal only once when child is selected
      item->setSelected(false);
      mTree->blockSignals(false);
      item->child(0)->setSelected(true);
    }
    return;
  }
  loadWorld();
  setSimulationDeadline(mAutoBox->isChecked());
}
