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

#include "OmAboutBox.hpp"

#include "OmApplicationInfo.hpp"
#include "OmDesktopServices.hpp"
#include "OmVersion.hpp"

#include <QtCore/QDate>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

OmAboutBox::OmAboutBox(QWidget *parent) : QDialog(parent) {
  setModal(true);
  setWindowTitle(tr("About OmniSim"));
  // Brand mark — OmniLink monogram. Source of truth: resources/branding/omnilink/.
  // The `images:omnisim.png` file is regenerated from orb/orb.png by
  // scripts/branding/build_omnilink_icons.py, so this load picks up the OmniLink
  // identity automatically.
  QPixmap omniLinkMark("images:omnisim.png");
  QVBoxLayout *layout = new QVBoxLayout();
  QGridLayout *firstRowLayout = new QGridLayout();
  firstRowLayout->setColumnMinimumWidth(0, 150);
  firstRowLayout->setColumnMinimumWidth(1, 250);

  QLabel *image = new QLabel();
  image->setPixmap(omniLinkMark.scaled(128, 128, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  firstRowLayout->addWidget(image, 0, 0, 1, 1, Qt::AlignCenter);

  QLabel *versionInfo = new QLabel();
  connect(versionInfo, &QLabel::linkActivated, &OmDesktopServices::openUrl);
  QDateTime releaseTimestamp;
  releaseTimestamp.setSecsSinceEpoch(OmApplicationInfo::releaseDate());
  // Mimosa accent (#F6E905) for OmniLink links — matches the canonical brand
  // palette in resources/branding/omnilink/BRAND.md.
  versionInfo->setText(QString("<p style='font-size: large;'><b>OmniSim %1</b></p>"
                               "<p style='color: #555;'>by <a style='color: #000; font-weight: bold; text-decoration: none;' "
                               "href='https://www.omnilink-agents.com'>OmniLink</a> &middot; %2</p>"
                               "<p>The simulator built by <a style='color: #000; font-weight: bold; text-decoration: none;' "
                               "href='https://www.omnilink-agents.com'>OmniLink</a>, for OmniLink agents.<br>"
                               "Free and open-source, built on top of the <a style='color: #5DADE2;' "
                               "href='https://github.com/cyberbotics/webots'>Webots</a> %3 engine by Cyberbotics Ltd.<br>"
                               "Licensed under the <a style='color: #5DADE2;' "
                               "href='https://www.apache.org/licenses/LICENSE-2.0'>Apache License, Version 2.0</a>.</p>"
                               "<p style='color: #888; font-size: small;'>© OmniLink &middot; "
                               "Upstream Webots © Cyberbotics 1998-%4</p>")
                         .arg(OmApplicationInfo::omniSimVersion())
                         .arg(releaseTimestamp.date().toString("MMMM dd, yyyy"))
                         .arg(OmApplicationInfo::version().toString())
                         .arg(QDate::currentDate().year()));
  firstRowLayout->addWidget(versionInfo, 0, 1, 1, 2, Qt::AlignCenter);

  QGridLayout *secondRowLayout = new QGridLayout();
  secondRowLayout->setRowMinimumHeight(0, 20);
  QLabel *description = new QLabel();
  connect(description, &QLabel::linkActivated, &OmDesktopServices::openUrl);
  description->setWordWrap(true);
  description->setText("<a style='color: #5DADE2;' href='https://www.omnilink-agents.com'>OmniLink</a>");
  secondRowLayout->addWidget(description, 1, 0, 1, 1, Qt::AlignBottom | Qt::AlignCenter);

  description = new QLabel();
  connect(description, &QLabel::linkActivated, &OmDesktopServices::openUrl);
  description->setWordWrap(true);
  description->setText("<a style='color: #5DADE2;' href='https://github.com/omnilink-tech/omnisim'>GitHub</a>");
  secondRowLayout->addWidget(description, 1, 1, 1, 1, Qt::AlignBottom | Qt::AlignCenter);

  description = new QLabel();
  connect(description, &QLabel::linkActivated, &OmDesktopServices::openUrl);
  description->setWordWrap(true);
  description->setText("<a style='color: #5DADE2;' href='https://github.com/cyberbotics/webots'>Upstream Webots</a>");
  secondRowLayout->addWidget(description, 1, 2, 1, 1, Qt::AlignBottom | Qt::AlignCenter);

  layout->addLayout(firstRowLayout);
  layout->addLayout(secondRowLayout);
  setLayout(layout);
}
