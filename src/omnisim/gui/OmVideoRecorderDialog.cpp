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

#include "OmVideoRecorderDialog.hpp"

#include "OmDoubleSpinBox.hpp"
#include "OmIntSpinBox.hpp"
#include "OmPreferences.hpp"

#include <QtGui/QScreen>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QListWidgetItem>
#include <QtWidgets/QSlider>
#include <cassert>

OmVideoRecorderDialog::OmVideoRecorderDialog(QWidget *parent, const QSize &currentResolution, double minAcceleration) :
  QDialog(parent),
  mAvailableResolutions(),
  mResolutionList(this) {
  setWindowTitle(tr("Choose video parameters"));

  const QScreen *screen = QGuiApplication::screenAt(QCursor::pos());
  const QSize fullScreen(screen->geometry().width(), screen->geometry().height());

  tryToAddResolution(OmResolution(352, 288, "CIF"), fullScreen);
  tryToAddResolution(OmResolution(426, 240, "240p"), fullScreen);
  tryToAddResolution(OmResolution(480, 320, "HVGA"), fullScreen);
  tryToAddResolution(OmResolution(640, 360, "360p"), fullScreen);
  tryToAddResolution(OmResolution(640, 480, "VGA"), fullScreen);
  tryToAddResolution(OmResolution(720, 480, "NTSC"), fullScreen);
  tryToAddResolution(OmResolution(800, 600, "SVGA"), fullScreen);
  tryToAddResolution(OmResolution(854, 480, "480p"), fullScreen);
  tryToAddResolution(OmResolution(1024, 768, "XGA"), fullScreen);
  tryToAddResolution(OmResolution(1280, 720, "HD Ready"), fullScreen);
  tryToAddResolution(OmResolution(1920, 1080, "Full HD"), fullScreen);
  tryToAddResolution(OmResolution(2560, 1440, "WQHD"), fullScreen);
  tryToAddResolution(OmResolution(3840, 2160, "4K UHD"), fullScreen);
  tryToAddResolution(OmResolution(fullScreen.width(), fullScreen.height(), ""), fullScreen);

  foreach (const OmResolution &r, mAvailableResolutions)
    mResolutionList.addItem(r.toString());

  // quality
  mQualitySpinBox = new OmIntSpinBox(this);
  mQualitySpinBox->setRange(1, 100);
  mQualitySpinBox->setMaximumWidth(80);

  // acceleration
  mAccelerationSpinBox = new OmDoubleSpinBox(this);
  mAccelerationSpinBox->setRange(minAcceleration, 999);
  mAccelerationSpinBox->setMaximumWidth(80);
  mAccelerationSpinBox->setToolTip(
    "The minimum value depends on the `WorldInfo.basicTimeStep` value.\nReduce it to enable a bigger slow motion effect.");

  // caption
  mCaptionCheckBox = new QCheckBox("Show acceleration value", this);

  // dialog buttons
  QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);

  connect(buttonBox, &QDialogButtonBox::accepted, this, &OmVideoRecorderDialog::accept);
  connect(buttonBox, &QDialogButtonBox::rejected, this, &OmVideoRecorderDialog::reject);

  QGridLayout *mainLayout = new QGridLayout(this);
  mainLayout->setVerticalSpacing(30);

  // row 0 - resolution
  mainLayout->addWidget(new QLabel(tr("Resolution:"), this), 0, 0);
  mainLayout->addWidget(&mResolutionList, 0, 1, 6, 2);
  // row 1 - quality
  mainLayout->addWidget(new QLabel(tr("Quality:"), this), 6, 0);
  mainLayout->addWidget(mQualitySpinBox, 6, 1);
  // row 2 - acceleration
  mainLayout->addWidget(new QLabel(tr("Video acceleration:"), this), 7, 0);
  mainLayout->addWidget(mAccelerationSpinBox, 7, 1);
  // row 3 - caption
  mainLayout->addWidget(new QLabel(tr("Video caption:"), this), 8, 0);
  mainLayout->addWidget(mCaptionCheckBox, 8, 1);
  // row 4 - buttons
  mainLayout->addWidget(buttonBox, 9, 1, 1, 3);

  mainLayout->setSizeConstraint(QLayout::SetFixedSize);

  // set default values
  int qualityValue, resolutionIndex;
  double accelerationValue;
  bool captionValue;
  OmPreferences::instance()->moviePreferences(resolutionIndex, qualityValue, accelerationValue, captionValue);
  mResolutionList.setCurrentRow(resolutionIndex);
  mQualitySpinBox->setValue(qualityValue);
  mAccelerationSpinBox->setValue(accelerationValue);
  mCaptionCheckBox->setChecked(captionValue);
}

OmVideoRecorderDialog::~OmVideoRecorderDialog() {
}

void OmVideoRecorderDialog::accept() {
  OmPreferences::instance()->setMoviePreferences(mResolutionList.currentRow(), quality(), acceleration(), showCaption());
  QDialog::accept();
}

QSize OmVideoRecorderDialog::resolution() const {
  const int index = mResolutionList.currentRow();
  assert(index >= 0 && index < mAvailableResolutions.count());
  const OmResolution &chosenResolution = mAvailableResolutions.at(index);
  return QSize(chosenResolution.width, chosenResolution.height);
}

int OmVideoRecorderDialog::quality() const {
  return mQualitySpinBox->value();
}

double OmVideoRecorderDialog::acceleration() const {
  return mAccelerationSpinBox->value();
}

bool OmVideoRecorderDialog::showCaption() const {
  return mCaptionCheckBox->isChecked();
}

void OmVideoRecorderDialog::tryToAddResolution(OmResolution resolution, const QSize &fullScreen) {
  if (!mAvailableResolutions.contains(resolution) && resolution.width <= fullScreen.width() &&
      resolution.height <= fullScreen.height() &&
      resolution.width % 2 == 0  // ffmpeg requires that width and height be divisible by 2
      && resolution.height % 2 == 0) {
    // Test if resolution is fullscreen
    if (resolution.width == fullScreen.width() && resolution.height == fullScreen.height()) {
      if (!resolution.label.isEmpty())
        resolution.label += " - ";
      resolution.label += "fullscreen";
    }
    mAvailableResolutions.append(resolution);
  }
}
