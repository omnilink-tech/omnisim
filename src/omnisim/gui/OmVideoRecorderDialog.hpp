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

#ifndef OM_VIDEO_RECORDER_DIALOG_HPP
#define OM_VIDEO_RECORDER_DIALOG_HPP

//
// Description: Qt dialog for setting video parameters
//

#include <QtCore/QList>
#include <QtWidgets/QDialog>
#include <QtWidgets/QListWidget>

class OmDoubleSpinBox;
class OmIntSpinBox;

class QCheckBox;
class QListWidget;
class QSize;

class OmVideoRecorderDialog : public QDialog {
  Q_OBJECT

public:
  // creates a dialog to set up video parameters
  OmVideoRecorderDialog(QWidget *parent, const QSize &currentResolution, double minAcceleration);
  virtual ~OmVideoRecorderDialog();

  QSize resolution() const;
  int quality() const;
  double acceleration() const;
  bool showCaption() const;

private:
  struct OmResolution {
    int width;
    int height;
    QString label;

    OmResolution(int width, int height, const QString &label) : width(width), height(height), label(label) {}

    QString toString() const { return QString::number(width) + "x" + QString::number(height) + " (" + label + ")"; }

    inline bool operator==(const OmResolution &other) const { return (width == other.width && height == other.height); }
  };

  // Add the resolution only if it is not already added into mAvailableResolutions.
  void tryToAddResolution(OmResolution resolution, const QSize &fullScreen);

  QList<OmResolution> mAvailableResolutions;
  QListWidget mResolutionList;
  OmIntSpinBox *mQualitySpinBox;
  OmDoubleSpinBox *mAccelerationSpinBox;
  QCheckBox *mCaptionCheckBox;

private slots:
  void accept() override;
};

#endif
