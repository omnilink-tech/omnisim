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

#include "OmPhysicsViewer.hpp"

#include "OmGuiRefreshOracle.hpp"
#include "OmSolid.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

OmPhysicsViewer::OmPhysicsViewer(QWidget *parent) :
  QWidget(parent),
  mSolid(NULL),
  mIsSelected(false),
  mIncludingExcludingDescendants(new QComboBox(this)),
  mRelativeAbsolute(new QComboBox(this)),
  mMassLabel(new QLabel(this)),
  mDensityLabel(new QLabel(this)),
  mInertiaMatrixMainLabel(new QLabel(tr("inertia matrix:"), this)) {
  void (QComboBox::*indexChangedSignal)(int) = &QComboBox::currentIndexChanged;
  QGridLayout *gridLayout = new QGridLayout(this);

  mIncludingExcludingDescendants->setMinimumHeight(mIncludingExcludingDescendants->sizeHint().height());
  mIncludingExcludingDescendants->insertItem(0, tr("excluding descendants"));
  mIncludingExcludingDescendants->insertItem(1, tr("including descendants"));
  mIncludingExcludingDescendants->setToolTip(tr("Display mass properties of the selected solid only"));
  gridLayout->addWidget(mIncludingExcludingDescendants, 0, 2, 1, 3, Qt::AlignVCenter);
  connect(mIncludingExcludingDescendants, indexChangedSignal, this, &OmPhysicsViewer::updateIncludingExcludingDescendantsData);

  // Mass
  QLabel *label = new QLabel(tr("mass:"), this);
  mMassLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  gridLayout->addWidget(label, 1, 0, 1, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mMassLabel, 1, 2, 1, 3, Qt::AlignVCenter);

  // Density
  label = new QLabel(tr("density:"), this);
  mDensityLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  gridLayout->addWidget(label, 2, 0, 1, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mDensityLabel, 2, 2, 1, 3, Qt::AlignVCenter);

  // Center of mass
  label = new QLabel("CoM:", this);
  label->setToolTip("Solid's center of mass");
  QLabel *valueLabel = NULL;
  for (int i = 0; i < 3; ++i) {
    valueLabel = new QLabel(this);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    mCenterOfMassLabel.append(valueLabel);
  }

  mRelativeAbsolute->setMinimumHeight(mRelativeAbsolute->sizeHint().height());
  mRelativeAbsolute->setToolTip(tr("Coordinates relative to selected's solid frame"));

  connect(mRelativeAbsolute, indexChangedSignal, this, &OmPhysicsViewer::updateCoordinatesSystem);

  gridLayout->addWidget(label, 3, 0, Qt::AlignVCenter);
  gridLayout->addWidget(mRelativeAbsolute, 3, 1, Qt::AlignVCenter);
  gridLayout->addWidget(mCenterOfMassLabel[0], 3, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mCenterOfMassLabel[1], 3, 3, Qt::AlignVCenter);
  gridLayout->addWidget(mCenterOfMassLabel[2], 3, 4, Qt::AlignVCenter);

  // Inertia matrix
  mInertiaMatrixMainLabel->setToolTip("Inertia matrix expressed within the solid frame centered at CoM");
  for (int i = 0; i < 9; ++i) {
    valueLabel = new QLabel(this);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    mInertiaMatrixLabel.append(valueLabel);
  }
  gridLayout->addWidget(mInertiaMatrixLabel[0], 4, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[1], 4, 3, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[2], 4, 4, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixMainLabel, 5, 0, 1, 2, Qt::AlignTop);
  gridLayout->addWidget(mInertiaMatrixLabel[3], 5, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[4], 5, 3, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[5], 5, 4, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[6], 6, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[7], 6, 3, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[8], 6, 4, Qt::AlignVCenter);

  // Set labels to be modified by the main stylesheet
  mInertiaMatrixLabel[0]->setObjectName("inertiaMatrixDiagonalCoefficientLabel");
  mInertiaMatrixLabel[4]->setObjectName("inertiaMatrixDiagonalCoefficientLabel");
  mInertiaMatrixLabel[8]->setObjectName("inertiaMatrixDiagonalCoefficientLabel");
  mInertiaMatrixLabel[1]->setObjectName("inertiaMatrixPrimaryCoefficientLabel");
  mInertiaMatrixLabel[2]->setObjectName("inertiaMatrixPrimaryCoefficientLabel");
  mInertiaMatrixLabel[5]->setObjectName("inertiaMatrixPrimaryCoefficientLabel");
  mInertiaMatrixLabel[3]->setObjectName("inertiaMatrixSecondaryCoefficientLabel");
  mInertiaMatrixLabel[6]->setObjectName("inertiaMatrixSecondaryCoefficientLabel");
  mInertiaMatrixLabel[7]->setObjectName("inertiaMatrixSecondaryCoefficientLabel");

  gridLayout->setColumnStretch(0, 0);
  gridLayout->setColumnStretch(1, 0);
  gridLayout->setColumnStretch(2, 1);
  gridLayout->setColumnStretch(3, 1);
  gridLayout->setColumnStretch(4, 1);

  mRelativeAbsolute->insertItem(0, tr("relative"));
  mRelativeAbsolute->insertItem(1, tr("absolute"));
}

OmPhysicsViewer::~OmPhysicsViewer() {
  mSolid = NULL;
}

void OmPhysicsViewer::clean() {
  mSolid = NULL;
}

void OmPhysicsViewer::stopUpdating() {
  if (mSolid) {
    disconnect(mSolid, &OmSolid::massPropertiesChanged, this, &OmPhysicsViewer::update);
    disconnect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this,
               &OmPhysicsViewer::updateCenterOfMass);
    disconnect(mSolid, &OmSolid::positionChangedArtificially, this, &OmPhysicsViewer::updateCenterOfMass);
  }
}

void OmPhysicsViewer::show(OmSolid *solid) {
  mSolid = solid;

  if (mSolid)
    connect(mSolid, &OmSolid::destroyed, this, &OmPhysicsViewer::clean, Qt::UniqueConnection);

  if (mSolid && mIsSelected) {
    connect(mSolid, &OmSolid::massPropertiesChanged, this, &OmPhysicsViewer::update, Qt::UniqueConnection);
    connect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmPhysicsViewer::updateCenterOfMass,
            Qt::UniqueConnection);
    connect(mSolid, &OmSolid::positionChangedArtificially, this, &OmPhysicsViewer::updateCenterOfMass, Qt::UniqueConnection);
  }
}

bool OmPhysicsViewer::update() {
  bool enabled = mSolid && (mSolid->globalMass() > 0.0);
  if (mIsSelected && enabled && mSolid->areOdeObjectsCreated()) {
    updateMass();
    updateDensity();
    updateCenterOfMass();
    updateInertiaMatrix();
    return enabled;
  }

  mMassLabel->clear();
  mDensityLabel->clear();
  for (int i = 0; i < 9; ++i)
    mInertiaMatrixLabel[i]->clear();
  for (int j = 0; j < 3; ++j)
    mCenterOfMassLabel[j]->clear();
  return enabled;
}

void OmPhysicsViewer::updateMass() {
  const double lm = mSolid->mass();
  const double gm = mSolid->globalMass();
  if (gm > 0.0) {
    const double currentMass = mIncludingExcludingDescendants->currentIndex() == LOCAL ? lm : gm;
    mMassLabel->setText(QString("%1 kg").arg(OmPrecision::doubleToString(currentMass, OmPrecision::GUI_MEDIUM)));
  } else
    mMassLabel->clear();
}

void OmPhysicsViewer::updateDensity() {
  const double d = mSolid->density();
  const double ad = mSolid->averageDensity();
  if (ad >= 0.0) {
    const double currentDensity = mIncludingExcludingDescendants->currentIndex() == LOCAL ? d : ad;
    mDensityLabel->setText(QString("%1 kg/m^3").arg(OmPrecision::doubleToString(currentDensity, OmPrecision::GUI_MEDIUM)));
  } else
    mDensityLabel->clear();
}

void OmPhysicsViewer::updateCenterOfMass() {
  bool skipUpdate = OmGuiRefreshOracle::instance()->canRefreshNow() == false;
  skipUpdate |= !mIsSelected || (!mSolid) || mSolid->areOdeObjectsCreated() == false;
  if (skipUpdate)
    return;

  mSolid->updateGlobalCenterOfMass();
  mCenterOfMass[LOCAL][RELATIVE_POSITION] = mSolid->centerOfMass();
  const OmMatrix4 &m = mSolid->matrix();
  mCenterOfMass[LOCAL][ABSOLUTE_POSITION] = m * mSolid->centerOfMass();
  mCenterOfMass[GLOBAL][ABSOLUTE_POSITION] = mSolid->globalCenterOfMass();
  mCenterOfMass[GLOBAL][RELATIVE_POSITION] = m.pseudoInversed(mCenterOfMass[GLOBAL][ABSOLUTE_POSITION]);
  if (mSolid->globalMass() != 0.0) {
    const OmVector3 &com = mCenterOfMass[mIncludingExcludingDescendants->currentIndex()][mRelativeAbsolute->currentIndex()];
    for (int i = 0; i < 3; ++i)
      mCenterOfMassLabel[i]->setText(OmPrecision::doubleToString(com[i], OmPrecision::GUI_MEDIUM));
  } else {
    for (int i = 0; i < 3; ++i)
      mCenterOfMassLabel[i]->clear();
  }
}

void OmPhysicsViewer::updateInertiaMatrix() {
  if (mSolid->mass() != 0.0 && (mIncludingExcludingDescendants->currentIndex() == LOCAL)) {
    mInertiaMatrixMainLabel->setText(tr("Inertia matrix:"));
    const double *const I = mSolid->inertiaMatrix();
    for (int i = 0; i < 3; ++i) {
      mInertiaMatrixLabel[i]->setText(OmPrecision::doubleToString(I[i], OmPrecision::GUI_MEDIUM));
      mInertiaMatrixLabel[i + 3]->setText(OmPrecision::doubleToString(I[i + 4], OmPrecision::GUI_MEDIUM));
      mInertiaMatrixLabel[i + 6]->setText(OmPrecision::doubleToString(I[i + 8], OmPrecision::GUI_MEDIUM));
    }
  } else {
    for (int i = 0; i < 9; ++i)
      mInertiaMatrixLabel[i]->clear();
    mInertiaMatrixMainLabel->clear();
  }
}

void OmPhysicsViewer::setSelected(bool selected) {
  mIsSelected = selected;
  triggerPhysicsUpdates();
}

void OmPhysicsViewer::triggerPhysicsUpdates() {
  if (mSolid == NULL)
    return;

  if (mIsSelected) {
    connect(mSolid, &OmSolid::massPropertiesChanged, this, &OmPhysicsViewer::update, Qt::UniqueConnection);
    connect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmPhysicsViewer::updateCenterOfMass,
            Qt::UniqueConnection);
    connect(mSolid, &OmSolid::positionChangedArtificially, this, &OmPhysicsViewer::updateCenterOfMass, Qt::UniqueConnection);
    update();
  } else {
    disconnect(mSolid, &OmSolid::massPropertiesChanged, this, &OmPhysicsViewer::update);
    disconnect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this,
               &OmPhysicsViewer::updateCenterOfMass);
    disconnect(mSolid, &OmSolid::positionChangedArtificially, this, &OmPhysicsViewer::updateCenterOfMass);
  }
}

void OmPhysicsViewer::updateCoordinatesSystem() {
  if (mRelativeAbsolute->currentIndex() == RELATIVE_POSITION)
    mRelativeAbsolute->setToolTip(tr("Coordinates with respect to selected's solid frame"));
  else
    mRelativeAbsolute->setToolTip(tr("Coordinates with respect to world's frame"));

  updateCenterOfMass();
}

void OmPhysicsViewer::updateIncludingExcludingDescendantsData() {
  if (mIncludingExcludingDescendants->currentIndex() == LOCAL)
    mIncludingExcludingDescendants->setToolTip(tr("Display mass properties of the selected solid only"));
  else
    mIncludingExcludingDescendants->setToolTip(
      tr("Display averaged mass properties of the selected solid augmented by its descendants"));

  update();
}
