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

#include "WbNewVersionDialog.hpp"

#include "WbApplicationInfo.hpp"
#include "WbDesktopServices.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"
#include "WbVersion.hpp"

#include <QtCore/QDir>

#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStyle>
#include <QtWidgets/QVBoxLayout>

static const QString gThemeNamesAndDescription[NUMBER_OF_THEMES][2] = {
  {"Classic",
   QObject::tr("The Classic theme — the original light look, comfortable on bright screens.")},
  {"Night", QObject::tr("The Night theme features clean lines, flat design, and a subdued color palette with highlights in "
                        "just the right places to make interacting with OmniSim even more intuitive.")},
  {"Dusk", QObject::tr("The Dusk theme is a second modern dark theme, also featuring the same clean flat design, with a light "
                       "green accent color. It's the developers' favorite!")}};

bool WbNewVersionDialog::run() {
  WbNewVersionDialog dialog;
  return dialog.exec();
}

WbNewVersionDialog::WbNewVersionDialog() {
  style()->polish(this);
  QDir::addSearchPath("newVersionIconPath", WbStandardPaths::resourcesPath() + newVersionIconPath());
  style()->polish(this);

  const WbVersion &version = WbApplicationInfo::version();
  const QString &omniSimVersion = WbApplicationInfo::omniSimVersion();
  setWindowTitle(tr("Welcome to OmniSim %1").arg(omniSimVersion));

  QVBoxLayout *vBoxLayout = new QVBoxLayout(this);

  QLabel *label =
    new QLabel(tr("OmniSim %1 is built on Webots %2.%3. See the "
                  "<a style='color: #5DADE2;' href='https://github.com/omnilink-tech/omnisim/releases/tag/v%1'>"
                  "release notes</a> for what's new in this version.")
                 .arg(omniSimVersion)
                 .arg(version.majorNumber())
                 .arg(QChar(version.minorNumber() + 'a')));
  connect(label, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  vBoxLayout->addWidget(label);
  vBoxLayout->addSpacing(10);

  QVBoxLayout *groupBoxLayout = new QVBoxLayout();

  // Pre-select the theme already stored in the preferences; when none matches,
  // fall back to the dark Night theme (index 1) — the OmniSim default face —
  // rather than the light Classic theme.
  static const int defaultThemeIndex = 1;  // Night
  const QString currentTheme = WbPreferences::instance()->value("General/theme").toString();
  int selectedTheme = defaultThemeIndex;
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (currentTheme.contains(gThemeNamesAndDescription[i][0], Qt::CaseInsensitive)) {
      selectedTheme = i;
      break;
    }
  }

  // list of themes
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (i > 0)
      groupBoxLayout->addSpacing(10);
    QHBoxLayout *themeLayout = new QHBoxLayout();
    mRadioButtons[i] = new QRadioButton('&' + gThemeNamesAndDescription[i][0], this);
    mRadioButtons[i]->setMinimumWidth(80);
    mRadioButtons[i]->setObjectName(gThemeNamesAndDescription[i][0].toLower());
    mRadioButtons[i]->setChecked(i == selectedTheme);
    themeLayout->addWidget(mRadioButtons[i]);
    QLabel *themeDescription = new QLabel(gThemeNamesAndDescription[i][1]);
    themeDescription->setWordWrap(true);
    themeLayout->addWidget(themeDescription, 1);
    groupBoxLayout->addLayout(themeLayout);
  }

  for (int i = 0; i < NUMBER_OF_THEMES; ++i)
    connect(mRadioButtons[i], &QRadioButton::toggled, this, &WbNewVersionDialog::updatePreview);

  QGroupBox *groupBox = new QGroupBox(tr("Themes:"));
  groupBox->setLayout(groupBoxLayout);
  vBoxLayout->addWidget(groupBox);

  // preview
  QGroupBox *previewBox = new QGroupBox(tr("Preview:"));
  mPreviewLabel = new QLabel();
  QHBoxLayout *previewLayout = new QHBoxLayout();
  previewLayout->addStretch();
  previewLayout->addWidget(mPreviewLabel);
  previewLayout->addStretch();
  previewBox->setLayout(previewLayout);
  vBoxLayout->addWidget(previewBox);

  // OmniLink follow box — replaces the upstream "Webots newsletter" CTA.
  QGroupBox *newsletterBox = new QGroupBox(tr("Stay in the loop:"));
  QVBoxLayout *newsletterLayout = new QVBoxLayout();
  label = new QLabel(tr("OmniSim is the simulator built by <a style='color: #000; font-weight: bold; text-decoration: none;' "
                        "href='https://www.omnilink-agents.com'>OmniLink</a> — for the OmniLink agentic AI platform. "
                        "Follow the project for new releases, demos, and the latest developments."));
  connect(label, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  label->setWordWrap(true);
  newsletterLayout->addWidget(label);
  newsletterBox->setLayout(newsletterLayout);
  vBoxLayout->addWidget(newsletterBox);

  // main button
  QPushButton *startButton = new QPushButton(tr("Start OmniSim with the selected theme."));
  vBoxLayout->addWidget(startButton);

  setLayout(vBoxLayout);
  connect(startButton, &QPushButton::clicked, this, &WbNewVersionDialog::startButtonPressed);
  updatePreview();
}

void WbNewVersionDialog::updatePreview() {
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (mRadioButtons[i]->isChecked()) {
      mPreviewLabel->setPixmap(
        QPixmap(WbStandardPaths::resourcesPath() + "images/themes/" + mRadioButtons[i]->objectName() + ".png"));
      break;
    }
  }
}

void WbNewVersionDialog::startButtonPressed() {
  QString theme;
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (mRadioButtons[i]->isChecked()) {
      theme = "omnisim_" + mRadioButtons[i]->objectName() + ".qss";
      break;
    }
  }
  WbPreferences::instance()->setValue("General/theme", theme);
  // force the sync in case we restart just after quitting the dialog (see #7662)
  WbPreferences::instance()->sync();
  done(QDialog::Accepted);
}
