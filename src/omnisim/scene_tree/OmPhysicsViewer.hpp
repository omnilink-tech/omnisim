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

#ifndef OM_PHYSICS_VIEWER_HPP
#define OM_PHYSICS_VIEWER_HPP

//
// Description: viewer showing information about Solid's physics
//

#include <QtWidgets/QWidget>

#include "OmVector3.hpp"

class QComboBox;
class QHBoxLayout;
class QLabel;
class OmSolid;

class OmPhysicsViewer : public QWidget {
  Q_OBJECT

public:
  explicit OmPhysicsViewer(QWidget *parent = NULL);
  virtual ~OmPhysicsViewer();

  void show(OmSolid *solid);

  bool update();
  void stopUpdating();
  void setSelected(bool selected);
  void triggerPhysicsUpdates();

public slots:
  void clean();

private:
  OmSolid *mSolid;
  bool mIsSelected;

  // Layouts
  QHBoxLayout *mTopLayout;

  // Combo boxes
  QComboBox *mIncludingExcludingDescendants;
  QComboBox *mRelativeAbsolute;

  // Labels
  QLabel *mMassLabel;
  QLabel *mDensityLabel;
  QLabel *mInertiaMatrixMainLabel;
  QList<QLabel *> mInertiaMatrixLabel;
  QList<QLabel *> mCenterOfMassLabel;

  // Values
  OmVector3 mCenterOfMass[2][2];

  enum CenterOfMassPosition { RELATIVE_POSITION = 0, ABSOLUTE_POSITION = 1 };
  enum CenterOfMassCoordinateSystem { LOCAL = 0, GLOBAL = 1 };

  // Updates
  void updateMass();
  void updateDensity();
  void updateInertiaMatrix();

private slots:
  void updateCenterOfMass();
  void updateCoordinatesSystem();
  void updateIncludingExcludingDescendantsData();
};

#endif
