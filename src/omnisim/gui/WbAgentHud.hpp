// Copyright 2026 OmniLink
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

#ifndef WB_AGENT_HUD_HPP
#define WB_AGENT_HUD_HPP

//
// Description: OmniLink Agent dock — two tabs.
//
//   Status — read-only HTML panel polling a runner's /status endpoint
//            (originally the Warehouse Foreman runner). Survives the
//            "no agent connected" case with a friendly placeholder.
//   Chat   — interactive prompt/transcript that POSTs to a bridge's
//            /prompt endpoint (omnilink_*_bridge). Lets an operator
//            type the same kind of natural-language commands the
//            in-world side menu accepts, without opening a robot
//            window. Falls back gracefully when no bridge is running.
//
// Configuration:
//   OMNI_AGENT_HUD_URL          — status endpoint (default
//                                 http://127.0.0.1:51521/status).
//   OMNI_AGENT_HUD_REFRESH_MS   — status poll interval (default 1500 ms).
//   OMNI_AGENT_HUD_ENABLED      — set to "0" to skip the dock entirely.
//   OMNI_AGENT_HUD_FONT_PX      — body font size at startup (default 16).
//   OMNI_AGENT_HUD_PROMPT_URL   — bridge /prompt endpoint for the Chat
//                                 tab (default http://127.0.0.1:8765/prompt,
//                                 the default port the omnilink_*_bridge
//                                 controllers serve on).

#include "WbDockWidget.hpp"

#include <QtCore/QString>

class QNetworkAccessManager;
class QNetworkReply;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class QTextBrowser;
class QTimer;
class QToolButton;

class WbAgentHud : public WbDockWidget {
  Q_OBJECT

public:
  explicit WbAgentHud(QWidget *parent = NULL);
  ~WbAgentHud() override;

  // True iff the OMNI_AGENT_HUD_ENABLED env var is not set to "0".
  static bool isEnabled();

private slots:
  void poll();
  void onReply(QNetworkReply *reply);
  void increaseFont();
  void decreaseFont();
  void resetFont();
  // Chat tab handlers
  void sendPrompt();
  void onPromptReply(QNetworkReply *reply);

private:
  void renderError(const QString &shortReason, const QString &detail);
  void renderStatus(const QByteArray &json);
  void applyFontSize();
  void persistFontSize() const;
  // Append a single line to the chat transcript with the given CSS class
  // (e.g. "user", "agent", "tool", "system", "error").
  void appendChatLine(const QString &kind, const QString &text);

  QString mUrl;
  QString mPromptUrl;
  int mRefreshMs;
  int mFontPx;
  int mDefaultFontPx;
  QTabWidget *mTabs;
  QTextBrowser *mView;
  // Chat-tab widgets.
  QTextBrowser *mChatLog;
  QPlainTextEdit *mChatInput;
  QPushButton *mChatSend;
  // The chat tab uses its own QNetworkAccessManager so the long /prompt
  // POSTs don't time-slice with the high-frequency /status polls.
  QNetworkAccessManager *mChatNet;
  QNetworkAccessManager *mNet;
  QTimer *mTimer;
  qint64 mStartedAtMs;
  bool mEverGotPayload;
  QByteArray mLastPayload;
};

#endif  // WB_AGENT_HUD_HPP
