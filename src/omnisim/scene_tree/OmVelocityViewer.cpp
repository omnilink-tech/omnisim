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

#include "OmVelocityViewer.hpp"

#include "OmGuiRefreshOracle.hpp"
#include "OmNodeUtilities.hpp"
#include "OmSolid.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

OmVelocityViewer::OmVelocityViewer(QWidget *parent) :
  QWidget(parent),
  mSolid(NULL),
  mIsSelected(false),
  mRelativeToComboBox(new QComboBox(this)) {
  QVBoxLayout *vBoxLayout = new QVBoxLayout(this);

  // Relative to combo box
  mRelativeToComboBox->setMinimumHeight(mRelativeToComboBox->sizeHint().height());
  mRelativeToComboBox->setToolTip(tr("Select relatively to which solid the velocity should be measured"));
  vBoxLayout->addWidget(mRelativeToComboBox);
  connect(mRelativeToComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this,
          &OmVelocityViewer::updateRelativeTo);

  // Labels
  QGridLayout *labelLayout = new QGridLayout();
  labelLayout->addWidget(new QLabel(tr("Linear velocity:")), 0, 0);
  labelLayout->addWidget(new QLabel(tr("Linear velocity magnitude:")), 1, 0);
  labelLayout->addWidget(new QLabel(tr("Angular velocity:")), 2, 0);
  labelLayout->addWidget(new QLabel(tr("Angular velocity magnitude:")), 3, 0);

  mLinearVelocityLabels.resize(4);
  mAngularVelocityLabels.resize(4);
  for (int i = 0; i < 4; ++i) {
    mLinearVelocityLabels[i] = new QLabel(this);
    mAngularVelocityLabels[i] = new QLabel(this);
    mLinearVelocityLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    mAngularVelocityLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    if (i < 3) {
      labelLayout->addWidget(mLinearVelocityLabels[i], 0, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
      labelLayout->addWidget(mAngularVelocityLabels[i], 2, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
    } else {
      labelLayout->addWidget(mLinearVelocityLabels[i], 1, 1, Qt::AlignVCenter | Qt::AlignLeft);
      labelLayout->addWidget(mAngularVelocityLabels[i], 3, 1, Qt::AlignVCenter | Qt::AlignLeft);
    }
  }
  vBoxLayout->addLayout(labelLayout);
}

OmVelocityViewer::~OmVelocityViewer() {
  mSolid = NULL;
}

void OmVelocityViewer::clean() {
  if (mSolid)
    disconnect(mSolid, &OmSolid::destroyed, this, &OmVelocityViewer::clean);
  mSolid = NULL;
}

void OmVelocityViewer::stopUpdating() {
  if (mSolid)
    disconnect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmVelocityViewer::requestUpdate);
}

void OmVelocityViewer::show(OmSolid *solid) {
  if (mSolid)
    disconnect(mSolid, &OmSolid::destroyed, this, &OmVelocityViewer::clean);

  mSolid = solid;

  updateRelativeToComboBox();

  if (mSolid) {
    connect(mSolid, &OmSolid::destroyed, this, &OmVelocityViewer::clean, Qt::UniqueConnection);

    if (mIsSelected)
      connect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmVelocityViewer::requestUpdate,
              Qt::UniqueConnection);
  }
}

void OmVelocityViewer::requestUpdate() {
  if (OmGuiRefreshOracle::instance()->canRefreshNow())
    update();
}

void OmVelocityViewer::update() {
  if (mIsSelected && mSolid) {
    const OmSolid *solid = mSolid;
    if (mRelativeToComboBox->currentIndex() == 0)
      solid = NULL;
    else {
      for (int i = 0; i < mRelativeToComboBox->currentIndex(); ++i)
        solid = solid->upperSolid();
    }
    OmVector3 linearVelocity = mSolid->relativeLinearVelocity(solid);
    OmVector3 angularVelocity = mSolid->relativeAngularVelocity(solid);
    for (int i = 0; i < 3; ++i) {
      mLinearVelocityLabels[i]->setText(OmPrecision::doubleToString(linearVelocity[i], OmPrecision::GUI_MEDIUM));
      mAngularVelocityLabels[i]->setText(OmPrecision::doubleToString(angularVelocity[i], OmPrecision::GUI_MEDIUM));
    }
    mLinearVelocityLabels[3]->setText(OmPrecision::doubleToString(linearVelocity.length(), OmPrecision::GUI_MEDIUM));
    mAngularVelocityLabels[3]->setText(OmPrecision::doubleToString(angularVelocity.length(), OmPrecision::GUI_MEDIUM));
    return;
  }

  for (int i = 0; i < mLinearVelocityLabels.size(); ++i)
    mLinearVelocityLabels[i]->clear();
  for (int i = 0; i < mAngularVelocityLabels.size(); ++i)
    mAngularVelocityLabels[i]->clear();
}

void OmVelocityViewer::updateRelativeTo(int index) {
  update();
}

void OmVelocityViewer::setSelected(bool selected) {
  mIsSelected = selected;
  triggerPhysicsUpdates();
}

void OmVelocityViewer::triggerPhysicsUpdates() {
  if (mSolid == NULL)
    return;

  if (mIsSelected) {
    connect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmVelocityViewer::requestUpdate,
            Qt::UniqueConnection);
    update();
  } else
    disconnect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmVelocityViewer::requestUpdate);
}

void OmVelocityViewer::updateRelativeToComboBox() {
  mRelativeToComboBox->clear();
  if (mSolid) {
    mRelativeToComboBox->insertItem(0, tr("Absolute"));
    int i = 0;
    OmSolid *solid = OmNodeUtilities::findUpperSolid(mSolid);
    while (solid) {
      ++i;
      if (solid->nodeModelName() == solid->fullName())
        mRelativeToComboBox->insertItem(i, tr("Relative to %1 (depth level %2)").arg(solid->fullName()).arg(i));
      else
        mRelativeToComboBox->insertItem(i, tr("Relative to %1").arg(solid->fullName()));
      solid = OmNodeUtilities::findUpperSolid(solid);
    }
  }
}
