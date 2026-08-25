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
//
// OmniSim splash screen — orb-centered, brand-canon dark composition.
// No Webots-derived robot screenshots: the splash is pure OmniLink identity
// (see resources/branding/omnilink/BRAND.md).

#include "OmSplashScreen.hpp"

#include "OmApplicationInfo.hpp"
#include "OmStandardPaths.hpp"

#include <QtCore/QString>
#include <QtGui/QPainter>
#include <QtGui/QPainterPath>
#include <QtGui/QRadialGradient>

namespace {
  // Canvas (1100 × 640 — larger than the legacy Webots splash, intentionally cinematic).
  constexpr int kW = 1100;
  constexpr int kH = 640;

  // Orb composition.
  constexpr int kOrbSize = 320;
  constexpr int kOrbCenterX = kW / 2;
  constexpr int kOrbCenterY = 230;

  // Wordmark baseline.
  constexpr int kWordmarkTop = kOrbCenterY + kOrbSize / 2 + 24;
  constexpr int kWordmarkHeight = 120;

  // OmniLink brand palette — see resources/branding/omnilink/BRAND.md.
  inline QColor omnilinkBlack() { return QColor(0x00, 0x00, 0x00); }
  inline QColor omnilinkCream() { return QColor(0xEE, 0xEE, 0xE0); }
  inline QColor omnilinkMimosa() { return QColor(0xF6, 0xE9, 0x05); }
}  // namespace

OmSplashScreen::OmSplashScreen() {
  QPixmap base(kW, kH);
  base.fill(omnilinkBlack());
  QSplashScreen::setPixmap(base);

  // Load the canonical OmniLink orb directly from the branding source.
  // 1024×1024 RGBA master — Qt scales smoothly down to kOrbSize.
  const QString orbPath = OmStandardPaths::resourcesPath() + "branding/omnilink/orb/orb.png";
  mLogo = QImage(orbPath);

  // Defaults (QSS theme files may override via the Q_PROPERTY hooks).
  mBackgroundColor = omnilinkBlack();
  mCompanyColor = omnilinkCream();
  mTaglineColor = omnilinkMimosa();
  mVersionColor = QColor(0xAA, 0xAA, 0xA0);
  mLoadingColor = QColor(0x88, 0x88, 0x88);
  mAccentColor = omnilinkMimosa();
  mStarColor = omnilinkCream();

#ifdef _WIN32
  setWindowModality(Qt::ApplicationModal);
#endif
}

OmSplashScreen::~OmSplashScreen() {
}

void OmSplashScreen::drawContents(QPainter *painter) {
#ifndef __linux__
  // Manual DPI compensation so the layout doesn't blow up on hi-DPI Windows/macOS.
  const double dpi = 96.0 / logicalDpiX();
#else
  const double dpi = 1.0;
#endif

  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setRenderHint(QPainter::SmoothPixmapTransform, true);

  // 1. Solid black canvas.
  painter->fillRect(QRect(0, 0, kW, kH), mBackgroundColor);

  // 2. Starfield — deterministic LCG so the field is stable across launches.
  //    Three tiers of dot size/opacity keep the field from looking uniform.
  {
    painter->save();
    painter->setPen(Qt::NoPen);
    uint32_t s = 0xC0DE2026u;
    auto next = [&s]() {
      s = s * 1664525u + 1013904223u;
      return s;
    };
    for (int i = 0; i < 160; ++i) {
      const double x = static_cast<double>(next() % (kW * 100)) / 100.0;
      const double y = static_cast<double>(next() % (kH * 100)) / 100.0;
      // Skip stars that would land directly on the orb (keeps the centerpiece clean).
      const double dx = x - kOrbCenterX;
      const double dy = y - kOrbCenterY;
      if (dx * dx + dy * dy < (kOrbSize * 0.45) * (kOrbSize * 0.45))
        continue;
      const int tier = next() % 100;
      double r;
      int a;
      if (tier < 70) {
        r = 0.6;
        a = 55;
      } else if (tier < 92) {
        r = 1.0;
        a = 110;
      } else if (tier < 99) {
        r = 1.5;
        a = 175;
      } else {
        r = 2.1;
        a = 220;
      }
      QColor c = mStarColor;
      c.setAlpha(a);
      painter->setBrush(c);
      painter->drawEllipse(QPointF(x, y), r, r);
    }
    painter->restore();
  }

  // 3. Mimosa radial glow behind the orb — gives the centerpiece weight on black.
  {
    const QPointF center(kOrbCenterX, kOrbCenterY);
    const double glowR = kOrbSize * 1.35;
    QRadialGradient glow(center, glowR);
    QColor inner = mAccentColor;
    inner.setAlpha(70);
    QColor mid = mAccentColor;
    mid.setAlpha(22);
    QColor outer = mAccentColor;
    outer.setAlpha(0);
    glow.setColorAt(0.0, inner);
    glow.setColorAt(0.35, mid);
    glow.setColorAt(1.0, outer);
    painter->save();
    painter->setPen(Qt::NoPen);
    painter->setBrush(glow);
    painter->drawEllipse(center, glowR, glowR);
    painter->restore();
  }

  // 4. Orb mark.
  if (!mLogo.isNull()) {
    const QRect orbRect(kOrbCenterX - kOrbSize / 2, kOrbCenterY - kOrbSize / 2, kOrbSize, kOrbSize);
    painter->drawImage(orbRect, mLogo);
  }

  // 5. Wordmark — "OMNISIM" centered below the orb.
  //    Raleway Light is the only display font the GUI registers (see
  //    OmGuiApplication::addApplicationFont). Qt falls back gracefully if absent.
  {
    QFont font("Raleway", static_cast<int>(82 * dpi), QFont::Light);
    font.setLetterSpacing(QFont::AbsoluteSpacing, 8.0);
    painter->setFont(font);
    painter->setPen(mCompanyColor);
    const QRect wordmarkRect(0, kWordmarkTop, kW, kWordmarkHeight);
    painter->drawText(wordmarkRect, Qt::AlignHCenter | Qt::AlignTop, "OMNISIM");
  }

  // 6. Mimosa accent bar — short horizontal rule under the wordmark.
  {
    const int barW = 140;
    const int barX = (kW - barW) / 2;
    const int barY = kWordmarkTop + 92;
    painter->fillRect(QRect(barX, barY, barW, 2), mAccentColor);
  }

  // 7. Tagline — small, letterspaced, Mimosa.
  {
    QFont font("Raleway", static_cast<int>(12 * dpi), QFont::Light);
    font.setLetterSpacing(QFont::AbsoluteSpacing, 3.5);
    painter->setFont(font);
    painter->setPen(mTaglineColor);
    const QRect taglineRect(0, kWordmarkTop + 104, kW, 26);
    painter->drawText(taglineRect, Qt::AlignHCenter | Qt::AlignTop, "BY OMNILINK — FOR OMNILINK AGENTS");
  }

  // 8. Loading message (live, mutated during boot).
  {
    QFont font("Helvetica", static_cast<int>(10 * dpi));
    painter->setFont(font);
    painter->setPen(mLoadingColor);
    const QRect loadingRect(0, kH - 44, kW, 22);
    painter->drawText(loadingRect, Qt::AlignHCenter | Qt::AlignVCenter, mMessage);
  }

  // 9. Version, bottom-right.
  {
    QFont font("Raleway", static_cast<int>(11 * dpi), QFont::Light);
    painter->setFont(font);
    painter->setPen(mVersionColor);
    const QRect versionRect(kW - 220, kH - 28, 200, 20);
    painter->drawText(versionRect, Qt::AlignRight | Qt::AlignVCenter, OmApplicationInfo::omniSimVersion());
  }

  // 10. Hairline Mimosa border framing the whole composition.
  {
    QPen pen(mAccentColor);
    pen.setWidth(1);
    painter->setPen(pen);
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(0, 0, kW - 1, kH - 1);
  }
}
