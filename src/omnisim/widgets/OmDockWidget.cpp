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

#include "OmDockWidget.hpp"

#include "OmDockTitleBar.hpp"

OmDockWidget::OmDockWidget(QWidget *parent) : QDockWidget(parent) {
  mTitleBar = new OmDockTitleBar(true, this);
  connect(mTitleBar, &OmDockTitleBar::closeClicked, this, &OmDockWidget::close);
  connect(mTitleBar, &OmDockTitleBar::maximizeClicked, this, &OmDockWidget::needsMaximize);
  connect(mTitleBar, &OmDockTitleBar::minimizeClicked, this, &OmDockWidget::needsMinimize);
  connect(mTitleBar, &OmDockTitleBar::floatClicked, this, &OmDockWidget::makeFloat);
  connect(this, &OmDockWidget::topLevelChanged, mTitleBar, &OmDockTitleBar::setFloating);

  setTitleBarWidget(mTitleBar);
}

OmDockWidget::~OmDockWidget() {
}

void OmDockWidget::setWindowTitle(const QString &title) {
  mTitleBar->setTitle(title);
}

void OmDockWidget::setTabbedTitle(const QString &title) {
  QDockWidget::setWindowTitle(title);
}

void OmDockWidget::setMaximized(bool maximized) {
  if (maximized)
    setFeatures(QDockWidget::NoDockWidgetFeatures);
  else
    setFeatures(DockWidgetClosable | DockWidgetMovable | DockWidgetFloatable);

  mTitleBar->setMaximized(maximized);
}

bool OmDockWidget::isMaximized() const {
  return mTitleBar->isMaximized();
}

void OmDockWidget::makeFloat() {
#ifdef __APPLE__
  // otherwise the dockWidget titlebar is sometimes
  // (when clicked on the float button) hidden
  // behind the OS menu bar
  if (pos().y() <= 0)
    move(pos().x(), pos().y() + 50);
#endif
  setFloating(!isFloating());
}
