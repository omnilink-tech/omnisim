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

#include "OmPositionViewer.hpp"

#include "OmGuiRefreshOracle.hpp"
#include "OmPose.hpp"
#include "OmSolid.hpp"
#include "OmTransform.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

OmPositionViewer::OmPositionViewer(QWidget *parent) :
  QWidget(parent),
  mPose(NULL),
  mIsSelected(false),
  mRelativeToComboBox(new QComboBox(this)) {
  QVBoxLayout *vBoxLayout = new QVBoxLayout(this);

  // Relative to combo box
  mRelativeToComboBox->setMinimumHeight(mRelativeToComboBox->sizeHint().height());
  mRelativeToComboBox->setToolTip(tr("Select relatively to which solid the position should be measured"));
  vBoxLayout->addWidget(mRelativeToComboBox);
  connect(mRelativeToComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this,
          &OmPositionViewer::updateRelativeTo);

  // Labels
  mScaleTitleLabel = new QLabel(this);
  QGridLayout *labelLayout = new QGridLayout();
  labelLayout->addWidget(new QLabel(tr("Position:")), 0, 0);
  labelLayout->addWidget(new QLabel(tr("Rotation:")), 1, 0);
  labelLayout->addWidget(mScaleTitleLabel, 2, 0);

  mPositionLabels.resize(3);
  for (int i = 0; i < mPositionLabels.size(); ++i) {
    mPositionLabels[i] = new QLabel(this);
    mPositionLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    labelLayout->addWidget(mPositionLabels[i], 0, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
  }
  mRotationLabels.resize(4);
  for (int i = 0; i < mRotationLabels.size(); ++i) {
    mRotationLabels[i] = new QLabel(this);
    mRotationLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    labelLayout->addWidget(mRotationLabels[i], 1, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
  }
  mScaleLabels.resize(3);
  for (int i = 0; i < mScaleLabels.size(); ++i) {
    mScaleLabels[i] = new QLabel(this);
    mScaleLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    labelLayout->addWidget(mScaleLabels[i], 2, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
  }
  vBoxLayout->addLayout(labelLayout);
}

OmPositionViewer::~OmPositionViewer() {
  mPose = NULL;
}

void OmPositionViewer::clean() {
  if (mPose)
    disconnect(mPose, &OmPose::destroyed, this, &OmPositionViewer::clean);
  mPose = NULL;
}

void OmPositionViewer::stopUpdating() {
  if (mPose) {
    disconnect(mPose->translationFieldValue(), &OmSFVector3::changed, this, &OmPositionViewer::update);
    disconnect(mPose->rotationFieldValue(), &OmSFRotation::changed, this, &OmPositionViewer::update);
    disconnect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmPositionViewer::requestUpdate);
  }
}

void OmPositionViewer::show(OmPose *pose) {
  if (mPose)
    disconnect(mPose, &OmPose::destroyed, this, &OmPositionViewer::clean);

  mPose = pose;

  updateRelativeToComboBox();

  if (mPose) {
    connect(mPose, &OmPose::destroyed, this, &OmPositionViewer::clean, Qt::UniqueConnection);

    if (mIsSelected) {
      connect(mPose->translationFieldValue(), &OmSFVector3::changed, this, &OmPositionViewer::update, Qt::UniqueConnection);
      connect(mPose->rotationFieldValue(), &OmSFRotation::changed, this, &OmPositionViewer::update, Qt::UniqueConnection);
      connect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmPositionViewer::requestUpdate,
              Qt::UniqueConnection);
    }
  }
}

void OmPositionViewer::requestUpdate() {
  if (OmGuiRefreshOracle::instance()->canRefreshNow())
    update();
}

void OmPositionViewer::update() {
  if (mIsSelected && mPose) {
    OmVector3 position(mPose->position());
    OmVector3 scale;
    const OmTransform *transform = dynamic_cast<const OmTransform *>(mPose);
    if (transform)
      scale = transform->absoluteScale();
    else {
      transform = mPose->upperTransform();
      scale = transform ? transform->absoluteScale() : OmVector3(1.0, 1.0, 1.0);
    }

    OmRotation rotation;
    if (mRelativeToComboBox->currentIndex() == 0)
      rotation.fromMatrix3(mPose->rotationMatrix());
    else {
      const OmPose *pose = mPose;
      OmQuaternion q;
      for (int i = 0; i < mRelativeToComboBox->currentIndex(); ++i) {
        assert(pose);
        q = pose->relativeQuaternion() * q;
        pose = pose->upperPose();
      }
      // compute relative rotation
      q.normalize();
      rotation.fromQuaternion(q);

      // compute relative scale
      OmVector3 otherAbsoluteScale;
      transform = dynamic_cast<const OmTransform *>(pose);
      if (transform)
        otherAbsoluteScale = transform->absoluteScale();
      else {
        transform = pose->upperTransform();
        otherAbsoluteScale = transform ? transform->absoluteScale() : OmVector3(1.0, 1.0, 1.0);
      }
      scale /= otherAbsoluteScale;

      // compute relative translation
      position = pose->rotationMatrix().transposed() * ((position - pose->position()));
      position /= otherAbsoluteScale;
    }

    rotation.normalize();
    if (rotation.almostEquals(OmRotation(), 0.000001))
      rotation = OmRotation();

    for (int i = 0; i < mPositionLabels.size(); ++i)
      mPositionLabels[i]->setText(OmPrecision::doubleToString(position[i], OmPrecision::GUI_MEDIUM));
    for (int i = 0; i < mRotationLabels.size(); ++i)
      mRotationLabels[i]->setText(OmPrecision::doubleToString(rotation[i], OmPrecision::GUI_MEDIUM));
    if (!scale.almostEquals(OmVector3(1, 1, 1))) {
      mScaleTitleLabel->setText(tr("Scale:"));
      for (int i = 0; i < mScaleLabels.size(); ++i)
        mScaleLabels[i]->setText(OmPrecision::doubleToString(scale[i], OmPrecision::GUI_MEDIUM));
    } else {
      mScaleTitleLabel->clear();
      for (int i = 0; i < mScaleLabels.size(); ++i)
        mScaleLabels[i]->clear();
    }
    return;
  }

  for (int i = 0; i < mPositionLabels.size(); ++i)
    mPositionLabels[i]->clear();
  for (int i = 0; i < mRotationLabels.size(); ++i)
    mRotationLabels[i]->clear();
  for (int i = 0; i < mScaleLabels.size(); ++i)
    mScaleLabels[i]->clear();
}

void OmPositionViewer::updateRelativeTo(int index) {
  update();
}

void OmPositionViewer::setSelected(bool selected) {
  mIsSelected = selected;
  triggerPhysicsUpdates();
}

void OmPositionViewer::triggerPhysicsUpdates() {
  if (mPose == NULL)
    return;

  if (mIsSelected) {
    connect(mPose->translationFieldValue(), &OmSFVector3::changed, this, &OmPositionViewer::update, Qt::UniqueConnection);
    connect(mPose->rotationFieldValue(), &OmSFRotation::changed, this, &OmPositionViewer::update, Qt::UniqueConnection);
    connect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmPositionViewer::requestUpdate,
            Qt::UniqueConnection);
    update();
  } else {
    disconnect(mPose->translationFieldValue(), &OmSFVector3::changed, this, &OmPositionViewer::update);
    disconnect(mPose->rotationFieldValue(), &OmSFRotation::changed, this, &OmPositionViewer::update);
    disconnect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshUpdated, this, &OmPositionViewer::requestUpdate);
  }
}

void OmPositionViewer::updateRelativeToComboBox() {
  mRelativeToComboBox->clear();
  if (mPose) {
    mRelativeToComboBox->insertItem(0, tr("Absolute"));
    int i = 0;
    const OmPose *pose = mPose->upperPose();
    while (pose) {
      ++i;
      if (pose->nodeModelName() == pose->fullName())
        mRelativeToComboBox->insertItem(i, tr("Relative to %1 (depth level %2)").arg(pose->fullName()).arg(i));
      else
        mRelativeToComboBox->insertItem(i, tr("Relative to %1").arg(pose->fullName()));
      pose = pose->upperPose();
    }
  }
}
