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

#ifndef WB_SPLASH_SCREEN_HPP
#define WB_SPLASH_SCREEN_HPP

#include <QtGui/QImage>
#include <QtWidgets/QSplashScreen>

class QPainter;

class WbSplashScreen : public QSplashScreen {
  Q_OBJECT
  Q_PROPERTY(QColor backgroundColor MEMBER mBackgroundColor READ backgroundColor WRITE setBackgroundColor)
  Q_PROPERTY(QColor companyColor MEMBER mCompanyColor READ companyColor WRITE setCompanyColor)
  Q_PROPERTY(QColor taglineColor MEMBER mTaglineColor READ taglineColor WRITE setTaglineColor)
  Q_PROPERTY(QColor versionColor MEMBER mVersionColor READ versionColor WRITE setVersionColor)
  Q_PROPERTY(QColor loadingColor MEMBER mLoadingColor READ loadingColor WRITE setLoadingColor)
  Q_PROPERTY(QColor accentColor MEMBER mAccentColor READ accentColor WRITE setAccentColor)
  Q_PROPERTY(QColor starColor MEMBER mStarColor READ starColor WRITE setStarColor)

public:
  WbSplashScreen();
  ~WbSplashScreen() override;
  void drawContents(QPainter *painter) override;
  void setLiveMessage(const QString &message) { mMessage = message; }

  const QColor &backgroundColor() const { return mBackgroundColor; }
  const QColor &companyColor() const { return mCompanyColor; }
  const QColor &taglineColor() const { return mTaglineColor; }
  const QColor &versionColor() const { return mVersionColor; }
  const QColor &loadingColor() const { return mLoadingColor; }
  const QColor &accentColor() const { return mAccentColor; }
  const QColor &starColor() const { return mStarColor; }

  void setBackgroundColor(const QColor &color) { mBackgroundColor = color; }
  void setCompanyColor(const QColor &color) { mCompanyColor = color; }
  void setTaglineColor(const QColor &color) { mTaglineColor = color; }
  void setVersionColor(const QColor &color) { mVersionColor = color; }
  void setLoadingColor(const QColor &color) { mLoadingColor = color; }
  void setAccentColor(const QColor &color) { mAccentColor = color; }
  void setStarColor(const QColor &color) { mStarColor = color; }

private:
  QString mMessage;
  QImage mLogo;

  QColor mBackgroundColor;
  QColor mCompanyColor;
  QColor mTaglineColor;
  QColor mVersionColor;
  QColor mLoadingColor;
  QColor mAccentColor;
  QColor mStarColor;
};

#endif  // WB_SPLASH_SCREEN_HPP
