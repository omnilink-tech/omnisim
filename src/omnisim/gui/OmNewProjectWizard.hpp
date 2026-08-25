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

#ifndef OM_NEW_PROJECT_WIZARD_HPP
#define OM_NEW_PROJECT_WIZARD_HPP

//
// Description: Wizard for creating a new OmniSim project
//

#include "OmNewWorldWizard.hpp"

class OmLineEdit;
class OmProject;

class OmNewProjectWizard : public OmNewWorldWizard {
  Q_OBJECT

public:
  explicit OmNewProjectWizard(QWidget *parent = NULL);
  virtual ~OmNewProjectWizard() override;

  void accept() override;
  bool validateCurrentPage() override;

protected:
  const int directoryId() const { return 2; }
  virtual const int worldId() const override { return 3; }
  virtual const int conclusionId() const override { return 4; }

  const QString title() const override { return tr("Create an OmniSim project directory"); }
  const QString introTitle() const override { return tr("New project creation"); }
  const QString introText() const override { return tr("This wizard will help you creating a new project folder."); }
  const QString conclusionText() const override { return tr("The following folders and files will be created:"); }
  void updateUI() override;

private slots:
  void chooseDirectory();

private:
  OmProject *mProject;
  OmLineEdit *mDirEdit;

  QString proposeNewProjectPath() const;
  QWizardPage *createDirectoryPage();
};

#endif
