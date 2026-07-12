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

#include "WbAgentHud.hpp"

#include <QtCore/QByteArray>
#include <QtCore/QDateTime>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonValue>
#include <QtCore/QSettings>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtGui/QKeySequence>
#include <QtGui/QShortcut>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkRequest>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QtGui/QTextCursor>

namespace {

  QString defaultUrl() {
    const QByteArray fromEnv = qgetenv("OMNI_AGENT_HUD_URL");
    if (!fromEnv.isEmpty())
      return QString::fromUtf8(fromEnv);
    // Default to the omnilink_*_bridge `/get_robot_state` endpoint — the
    // beginner OmniLink chat demos are the most common case and the HUD
    // gets a useful payload (robot id, model, joint angles or pose, mode,
    // sim_time) the moment one of those worlds loads. The renderer also
    // recognises the legacy Warehouse-Foreman /status shape and falls
    // back to its dense usage layout when that's what's on the wire, so
    // both deployment styles "just work" without env-var tweaks. Operators
    // pointing at a non-default runner can still override with
    // OMNI_AGENT_HUD_URL.
    return QStringLiteral("http://127.0.0.1:8765/get_robot_state");
  }

  int defaultRefreshMs() {
    const QByteArray fromEnv = qgetenv("OMNI_AGENT_HUD_REFRESH_MS");
    bool ok = false;
    const int parsed = fromEnv.toInt(&ok);
    if (ok && parsed >= 250 && parsed <= 60000)
      return parsed;
    return 1500;
  }

  QString defaultPromptUrl() {
    const QByteArray fromEnv = qgetenv("OMNI_AGENT_HUD_PROMPT_URL");
    if (!fromEnv.isEmpty())
      return QString::fromUtf8(fromEnv);
    // Default to the omnilink_*_bridge controllers' /prompt port. Every
    // omnilink_<robot>.wbt demo serves on 8765 so the chat tab "just
    // works" against any of them with zero configuration.
    return QStringLiteral("http://127.0.0.1:8765/prompt");
  }

  // Body font size in pixels at startup. The previous default of 11 px
  // was too small for comfortable reading at HD resolution; 16 px puts
  // the headline numbers at glance-distance-readable size with room to
  // grow via Ctrl+= for low-vision operators. The clamp range
  // [10, 36] covers everything from "compact, lots fits" to
  // "across-the-room readable" without breaking layout.
  constexpr int kMinFontPx = 10;
  constexpr int kMaxFontPx = 36;
  constexpr int kBaseDefaultFontPx = 16;
  constexpr int kFontStepPx = 2;
  constexpr const char *kSettingsGroup = "OmniLink/AgentHud";
  constexpr const char *kSettingsFontKey = "fontPx";

  int defaultFontPx() {
    const QByteArray fromEnv = qgetenv("OMNI_AGENT_HUD_FONT_PX");
    bool ok = false;
    const int parsed = fromEnv.toInt(&ok);
    if (ok && parsed >= kMinFontPx && parsed <= kMaxFontPx)
      return parsed;
    return kBaseDefaultFontPx;
  }

  // Read the operator's last-saved font size from QSettings, falling
  // back to the env-var default if no preference has been recorded yet.
  // Persistent preference wins over env so an operator who bumped to
  // 22 px three sessions ago doesn't have to re-bump on every launch.
  int loadSavedFontPx(int fallback) {
    QSettings settings;
    settings.beginGroup(QString::fromLatin1(kSettingsGroup));
    const int stored = settings.value(QString::fromLatin1(kSettingsFontKey), fallback).toInt();
    settings.endGroup();
    if (stored < kMinFontPx || stored > kMaxFontPx)
      return fallback;
    return stored;
  }

  // Format an integer with thousands separators. Used for token counts
  // that can routinely run into millions — readability matters when an
  // operator is glancing at the HUD.
  QString formatBigInt(qlonglong value) {
    QString s = QString::number(value);
    int insertPos = s.length() - 3;
    while (insertPos > 0) {
      s.insert(insertPos, QChar(','));
      insertPos -= 3;
    }
    return s;
  }

  QString fmtDouble(double v, int decimals) {
    return QString::number(v, 'f', decimals);
  }

  QString safeStr(const QJsonValue &v, const QString &fallback = QStringLiteral("--")) {
    if (v.isString())
      return v.toString();
    if (v.isDouble())
      return QString::number(v.toDouble());
    if (v.isBool())
      return v.toBool() ? QStringLiteral("true") : QStringLiteral("false");
    if (v.isNull() || v.isUndefined())
      return fallback;
    return fallback;
  }

}  // namespace

bool WbAgentHud::isEnabled() {
  const QByteArray flag = qgetenv("OMNI_AGENT_HUD_ENABLED");
  if (flag.isEmpty())
    return true;
  const QString s = QString::fromUtf8(flag).trimmed().toLower();
  return s != QStringLiteral("0") && s != QStringLiteral("false") && s != QStringLiteral("no");
}

WbAgentHud::WbAgentHud(QWidget *parent) :
  WbDockWidget(parent),
  mUrl(defaultUrl()),
  mPromptUrl(defaultPromptUrl()),
  mRefreshMs(defaultRefreshMs()),
  mFontPx(loadSavedFontPx(defaultFontPx())),
  mDefaultFontPx(defaultFontPx()),
  mTabs(NULL),
  mView(NULL),
  mChatLog(NULL),
  mChatInput(NULL),
  mChatSend(NULL),
  mChatNet(NULL),
  mNet(NULL),
  mTimer(NULL),
  mStartedAtMs(QDateTime::currentMSecsSinceEpoch()),
  mEverGotPayload(false) {
  setWindowTitle(tr("OmniLink Agent"));
  setObjectName("OmniLinkAgentHud");

  QWidget *container = new QWidget(this);
  QVBoxLayout *layout = new QVBoxLayout(container);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  // Toolbar with font-size controls. Three buttons keeps the surface
  // small and the affordance discoverable — operators with low-vision
  // needs can find them without learning shortcuts, and the buttons
  // mirror exactly what the keyboard shortcuts do (Ctrl+= / Ctrl+− /
  // Ctrl+0). Buttons are large enough to hit-target reliably even when
  // the HUD is at minimum width.
  QToolBar *toolbar = new QToolBar(container);
  toolbar->setIconSize(QSize(20, 20));
  toolbar->setStyleSheet(
    "QToolBar { background: #2a2a2d; border-bottom: 1px solid #3a3a3f; "
    "padding: 4px; spacing: 4px; }"
    "QToolButton { color: #d8d8d8; background: transparent; "
    "padding: 4px 10px; min-width: 28px; min-height: 24px; "
    "font-weight: bold; }"
    "QToolButton:hover { background: #3a3a3f; }");

  QToolButton *btnSmaller = new QToolButton(toolbar);
  btnSmaller->setText(QStringLiteral("A−"));
  btnSmaller->setToolTip(tr("Decrease HUD text size (Ctrl+−)"));
  connect(btnSmaller, &QToolButton::clicked, this, &WbAgentHud::decreaseFont);
  toolbar->addWidget(btnSmaller);

  QToolButton *btnReset = new QToolButton(toolbar);
  btnReset->setText(QStringLiteral("A"));
  btnReset->setToolTip(tr("Reset HUD text size to default (Ctrl+0)"));
  connect(btnReset, &QToolButton::clicked, this, &WbAgentHud::resetFont);
  toolbar->addWidget(btnReset);

  QToolButton *btnBigger = new QToolButton(toolbar);
  btnBigger->setText(QStringLiteral("A+"));
  btnBigger->setToolTip(tr("Increase HUD text size (Ctrl+=)"));
  connect(btnBigger, &QToolButton::clicked, this, &WbAgentHud::increaseFont);
  toolbar->addWidget(btnBigger);

  layout->addWidget(toolbar);

  // The OmniLink Agent dock now hosts two tabs:
  //   1. Status — read-only HTML poll of a runner's /status URL.
  //   2. Chat   — interactive prompt/transcript that talks to a bridge's
  //               /prompt endpoint (default: omnilink_*_bridge on 8765).
  // The toolbar stays above the tabs so the font controls scale both
  // views (the chat transcript inherits the same monospace stylesheet).
  mTabs = new QTabWidget(container);
  mTabs->setDocumentMode(true);

  // ── Status tab (the original HUD) ───────────────────────────────
  mView = new QTextBrowser(mTabs);
  mView->setOpenExternalLinks(true);
  mTabs->addTab(mView, tr("Status"));

  // ── Chat tab ────────────────────────────────────────────────────
  QWidget *chatContainer = new QWidget(mTabs);
  QVBoxLayout *chatLayout = new QVBoxLayout(chatContainer);
  chatLayout->setContentsMargins(8, 8, 8, 8);
  chatLayout->setSpacing(8);

  mChatLog = new QTextBrowser(chatContainer);
  mChatLog->setOpenExternalLinks(true);
  mChatLog->setReadOnly(true);
  chatLayout->addWidget(mChatLog, 1);

  mChatInput = new QPlainTextEdit(chatContainer);
  mChatInput->setPlaceholderText(tr("Tell the robot what to do…"));
  mChatInput->setMaximumHeight(80);
  chatLayout->addWidget(mChatInput);

  QHBoxLayout *btnRow = new QHBoxLayout();
  btnRow->setContentsMargins(0, 0, 0, 0);
  mChatSend = new QPushButton(tr("Send"), chatContainer);
  mChatSend->setDefault(true);
  btnRow->addStretch(1);
  btnRow->addWidget(mChatSend);
  chatLayout->addLayout(btnRow);

  mTabs->addTab(chatContainer, tr("Chat"));

  layout->addWidget(mTabs);
  setWidget(container);

  connect(mChatSend, &QPushButton::clicked, this, &WbAgentHud::sendPrompt);

  appendChatLine("system",
                 tr("OmniLink Chat — type a command for the robot. "
                    "Default endpoint: %1. Override with "
                    "OMNI_AGENT_HUD_PROMPT_URL.").arg(mPromptUrl));

  applyFontSize();

  // Keyboard shortcuts. Ctrl+= and Ctrl+− are the conventional zoom
  // bindings (browsers, IDEs); Ctrl+0 resets — matching the Web Content
  // accessibility convention so the muscle memory transfers. Shortcuts
  // are scoped to this widget so they don't fight with Webots' global
  // shortcuts when the HUD doesn't have focus.
  QShortcut *scIncrease = new QShortcut(QKeySequence(QStringLiteral("Ctrl+=")), this);
  scIncrease->setContext(Qt::WidgetWithChildrenShortcut);
  connect(scIncrease, &QShortcut::activated, this, &WbAgentHud::increaseFont);

  QShortcut *scIncreasePlus = new QShortcut(QKeySequence(QStringLiteral("Ctrl++")), this);
  scIncreasePlus->setContext(Qt::WidgetWithChildrenShortcut);
  connect(scIncreasePlus, &QShortcut::activated, this, &WbAgentHud::increaseFont);

  QShortcut *scDecrease = new QShortcut(QKeySequence(QStringLiteral("Ctrl+-")), this);
  scDecrease->setContext(Qt::WidgetWithChildrenShortcut);
  connect(scDecrease, &QShortcut::activated, this, &WbAgentHud::decreaseFont);

  QShortcut *scReset = new QShortcut(QKeySequence(QStringLiteral("Ctrl+0")), this);
  scReset->setContext(Qt::WidgetWithChildrenShortcut);
  connect(scReset, &QShortcut::activated, this, &WbAgentHud::resetFont);

  mNet = new QNetworkAccessManager(this);
  connect(mNet, &QNetworkAccessManager::finished, this, &WbAgentHud::onReply);

  // Separate network manager for the chat /prompt POSTs. The chat
  // round-trip can take several seconds (the bridge waits for the
  // OmniLink chat-with-tools loop to settle); keeping it off the
  // status manager prevents the high-frequency /status polls from
  // queue-stalling behind a slow chat.
  mChatNet = new QNetworkAccessManager(this);
  connect(mChatNet, &QNetworkAccessManager::finished, this, &WbAgentHud::onPromptReply);

  mTimer = new QTimer(this);
  mTimer->setInterval(mRefreshMs);
  connect(mTimer, &QTimer::timeout, this, &WbAgentHud::poll);
  mTimer->start();

  // Render an initial "polling..." panel before the first reply lands
  // so the dock isn't empty during the first refresh window.
  renderError(tr("polling..."),
              tr("Waiting for the first response from %1.").arg(mUrl));

  // Fire one immediate poll so the first response arrives in <1s for
  // a freshly-started runner instead of waiting the full interval.
  QTimer::singleShot(50, this, &WbAgentHud::poll);
}

void WbAgentHud::increaseFont() {
  const int next = qMin(mFontPx + kFontStepPx, kMaxFontPx);
  if (next == mFontPx)
    return;
  mFontPx = next;
  applyFontSize();
  persistFontSize();
}

void WbAgentHud::decreaseFont() {
  const int next = qMax(mFontPx - kFontStepPx, kMinFontPx);
  if (next == mFontPx)
    return;
  mFontPx = next;
  applyFontSize();
  persistFontSize();
}

void WbAgentHud::resetFont() {
  if (mFontPx == mDefaultFontPx)
    return;
  mFontPx = mDefaultFontPx;
  applyFontSize();
  persistFontSize();
}

void WbAgentHud::applyFontSize() {
  if (!mView)
    return;
  const QString css = QStringLiteral(
    "QTextBrowser { background: #1a1a1d; color: #d8d8d8; "
    "font-family: 'JetBrains Mono', 'Consolas', 'Menlo', monospace; "
    "font-size: %1px; padding: 10px; border: 0; }").arg(mFontPx);
  mView->setStyleSheet(css);
  // Chat surface gets the same monospaced palette so Ctrl+= / Ctrl+−
  // scale both tabs together.
  if (mChatLog) {
    mChatLog->setStyleSheet(css);
  }
  if (mChatInput) {
    const QString inputCss = QStringLiteral(
      "QPlainTextEdit { background: #08080a; color: #f3eddc; "
      "font-family: 'JetBrains Mono', 'Consolas', 'Menlo', monospace; "
      "font-size: %1px; padding: 6px 8px; border: 1px solid #1d1d20; "
      "border-radius: 6px; } "
      "QPlainTextEdit:focus { border-color: #fbe283; }").arg(mFontPx);
    mChatInput->setStyleSheet(inputCss);
  }
  if (mChatSend) {
    const QString btnCss = QStringLiteral(
      "QPushButton { background: #fbe283; color: #0b0b0c; "
      "border: 1px solid #d6a72d; border-radius: 6px; "
      "padding: 6px 16px; font-weight: 600; font-size: %1px; } "
      "QPushButton:hover { background: #ffe890; } "
      "QPushButton:disabled { opacity: 0.5; }").arg(mFontPx);
    mChatSend->setStyleSheet(btnCss);
  }
  // Re-render with the cached payload (or an error placeholder) so the
  // size change shows up instantly without waiting for the next poll.
  if (!mLastPayload.isEmpty())
    renderStatus(mLastPayload);
}

void WbAgentHud::appendChatLine(const QString &kind, const QString &text) {
  if (!mChatLog)
    return;
  // Lightweight HTML rendering. The dark palette mirrors the in-world
  // omnilink_chat plugin so operators get a consistent look whether
  // they're chatting in the dock or in the robot window.
  QString color;
  QString prefix;
  if (kind == "user") {
    color = QStringLiteral("#fbe283");
    prefix = QStringLiteral("you ▸");
  } else if (kind == "agent") {
    color = QStringLiteral("#f3eddc");
    prefix = QStringLiteral("agent ▸");
  } else if (kind == "tool") {
    color = QStringLiteral("#8eb6ff");
    prefix = QStringLiteral("→");
  } else if (kind == "error") {
    color = QStringLiteral("#ff8a82");
    prefix = QStringLiteral("error ▸");
  } else {  // system
    color = QStringLiteral("#8a8472");
    prefix = QStringLiteral("·");
  }
  const QString escaped = text.toHtmlEscaped();
  const QString html = QStringLiteral(
    "<div style='margin:4px 0; color:%1;'><span style='opacity:0.65;'>%2</span> %3</div>")
                         .arg(color, prefix, escaped);
  mChatLog->append(html);
  QTextCursor cursor = mChatLog->textCursor();
  cursor.movePosition(QTextCursor::End);
  mChatLog->setTextCursor(cursor);
}

void WbAgentHud::sendPrompt() {
  if (!mChatInput || !mChatNet)
    return;
  const QString text = mChatInput->toPlainText().trimmed();
  if (text.isEmpty())
    return;
  appendChatLine("user", text);
  mChatInput->clear();
  mChatSend->setEnabled(false);

  QNetworkRequest req((QUrl(mPromptUrl)));
  req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  // The bridge's chat-with-tools loop can take >5s on the first turn
  // (OmniLink cold-start). Give it room.
  req.setTransferTimeout(60000);
  QJsonObject body;
  body.insert("text", text);
  mChatNet->post(req, QJsonDocument(body).toJson(QJsonDocument::Compact));
}

void WbAgentHud::onPromptReply(QNetworkReply *reply) {
  if (!reply)
    return;
  reply->deleteLater();
  if (mChatSend)
    mChatSend->setEnabled(true);

  if (reply->error() != QNetworkReply::NoError) {
    const QString err = reply->errorString();
    if (err.contains("refused", Qt::CaseInsensitive))
      appendChatLine("error",
                     tr("Bridge not reachable at %1. Is an omnilink_<robot>.wbt demo "
                        "running? (Override the URL with OMNI_AGENT_HUD_PROMPT_URL.)")
                       .arg(mPromptUrl));
    else
      appendChatLine("error", tr("HTTP error: %1").arg(err));
    return;
  }

  const QByteArray raw = reply->readAll();
  const QJsonDocument doc = QJsonDocument::fromJson(raw);
  if (!doc.isObject()) {
    appendChatLine("error", tr("Bridge returned non-JSON body: %1").arg(QString::fromUtf8(raw.left(200))));
    return;
  }
  const QJsonObject obj = doc.object();
  // Echo each tool the agent actually fired (so the operator sees the
  // chain that happened, not just the final text).
  const QJsonArray actions = obj.value("actions").toArray();
  for (const QJsonValue &v : actions) {
    const QJsonObject a = v.toObject();
    const QString tool = a.value("tool").toString();
    const QString result = a.value("result").toString();
    const QString summary = a.value("summary").toString();
    appendChatLine("tool",
                   QStringLiteral("%1 · %2 · %3").arg(tool, result, summary));
  }
  const QString response = obj.value("response").toString();
  if (!response.isEmpty())
    appendChatLine("agent", response);
  const QString error = obj.value("error").toString();
  if (!error.isEmpty())
    appendChatLine("error", error);
}

void WbAgentHud::persistFontSize() const {
  QSettings settings;
  settings.beginGroup(QString::fromLatin1(kSettingsGroup));
  settings.setValue(QString::fromLatin1(kSettingsFontKey), mFontPx);
  settings.endGroup();
}

WbAgentHud::~WbAgentHud() {
}

void WbAgentHud::poll() {
  QNetworkRequest req((QUrl(mUrl)));
  // Tight per-request timeout — if the runner is slow, the HUD should
  // surface that as a "stale" indicator rather than queuing requests.
  req.setTransferTimeout(2000);
  req.setHeader(QNetworkRequest::UserAgentHeader,
                QByteArrayLiteral("OmniSim-AgentHud/1.0"));
  mNet->get(req);
}

void WbAgentHud::onReply(QNetworkReply *reply) {
  reply->deleteLater();
  if (reply->error() != QNetworkReply::NoError) {
    const QString reason = (reply->error() == QNetworkReply::ConnectionRefusedError)
      ? tr("no agent reachable")
      : reply->errorString();
    const bool defaultUrl = mUrl == QStringLiteral("http://127.0.0.1:8765/get_robot_state");
    const QString detail = defaultUrl
      ? tr("HUD is polling %1 every %2 ms. Open one of the "
           "<code>omnilink_&lt;robot&gt;.wbt</code> demo worlds and this panel "
           "will populate with that robot's live state (pose, joints, mode). "
           "For a non-default runner, point OMNI_AGENT_HUD_URL at its /status URL.")
          .arg(mUrl).arg(mRefreshMs)
      : tr("HUD is polling %1 every %2 ms. Start the corresponding agent "
           "runner and this panel will populate automatically.")
          .arg(mUrl).arg(mRefreshMs);
    renderError(reason, detail);
    return;
  }
  const QByteArray body = reply->readAll();
  if (body.isEmpty()) {
    renderError(tr("empty response"),
                tr("Runner at %1 returned no body.").arg(mUrl));
    return;
  }
  mEverGotPayload = true;
  mLastPayload = body;
  renderStatus(body);
}

void WbAgentHud::renderError(const QString &shortReason, const QString &detail) {
  const QString uptime = QString::number(
    (QDateTime::currentMSecsSinceEpoch() - mStartedAtMs) / 1000);
  QString html;
  html += QStringLiteral(
    "<table width='100%' cellpadding='6' cellspacing='0'>"
    "<tr><td style='font-size:1.15em;color:#f4d03f;font-weight:bold;'>"
    "OmniLink agent — %1"
    "</td></tr>"
    "<tr><td style='color:#a0a0a0;'>%2</td></tr>"
    "<tr><td style='color:#606060;font-size:0.85em;padding-top:12px;'>"
    "URL: %3<br>HUD up %4s. Refresh every %5 ms."
    "</td></tr>"
    "</table>")
    .arg(shortReason)
    .arg(detail)
    .arg(mUrl)
    .arg(uptime)
    .arg(mRefreshMs);
  mView->setHtml(html);
}

void WbAgentHud::renderStatus(const QByteArray &json) {
  QJsonParseError perr;
  const QJsonDocument doc = QJsonDocument::fromJson(json, &perr);
  if (perr.error != QJsonParseError::NoError || !doc.isObject()) {
    renderError(tr("non-JSON response"),
                tr("Got %1 bytes but couldn't parse as JSON: %2")
                  .arg(json.size()).arg(perr.errorString()));
    return;
  }
  const QJsonObject root = doc.object();

  // Bridge state from an omnilink_*_bridge: id + model + (q | x/y/yaw) +
  // mode + sim_time. The legacy Warehouse-Foreman /status has none of
  // these keys, so this branch is unambiguous.
  if (root.contains("id") && root.contains("model") && root.contains("mode")) {
    const QString id = safeStr(root.value("id"));
    const QString model = safeStr(root.value("model"));
    const QString mode = safeStr(root.value("mode"));
    const QString fault = safeStr(root.value("fault"));
    const double simTime = root.value("sim_time").toDouble();

    QString jointsHtml;
    if (root.value("q").isArray()) {
      QStringList parts;
      const QJsonArray q = root.value("q").toArray();
      for (int i = 0; i < q.size(); ++i)
        parts << QStringLiteral("q%1=%2").arg(i + 1).arg(q.at(i).toDouble(), 0, 'f', 3);
      jointsHtml = QStringLiteral(
        "<tr><td style='color:#888;width:140px;'>joints</td>"
        "<td style='color:#d8d8d8;'>%1</td></tr>")
        .arg(parts.join(QStringLiteral(", ")));
    }

    QString tcpHtml;
    if (root.value("tcp").isArray()) {
      const QJsonArray a = root.value("tcp").toArray();
      if (a.size() == 3) {
        tcpHtml = QStringLiteral(
          "<tr><td style='color:#888;'>TCP (xyz)</td>"
          "<td style='color:#d8d8d8;'>(%1, %2, %3)</td></tr>")
          .arg(a.at(0).toDouble(), 0, 'f', 3)
          .arg(a.at(1).toDouble(), 0, 'f', 3)
          .arg(a.at(2).toDouble(), 0, 'f', 3);
      }
    }

    QString poseHtml;
    if (root.contains("x") && root.contains("y") && root.contains("yaw")) {
      const double x = root.value("x").toDouble();
      const double y = root.value("y").toDouble();
      const double yawDeg = root.value("yaw").toDouble() * 57.29577951;
      const double vLin = root.value("v_linear").toDouble();
      const double vAng = root.value("v_angular").toDouble() * 57.29577951;
      poseHtml = QStringLiteral(
        "<tr><td style='color:#888;width:140px;'>pose</td>"
        "<td style='color:#d8d8d8;'>x=%1, y=%2, yaw=%3°</td></tr>"
        "<tr><td style='color:#888;'>velocity</td>"
        "<td style='color:#d8d8d8;'>v=%4 m/s, ω=%5 °/s</td></tr>")
        .arg(x, 0, 'f', 3)
        .arg(y, 0, 'f', 3)
        .arg(yawDeg, 0, 'f', 1)
        .arg(vLin, 0, 'f', 3)
        .arg(vAng, 0, 'f', 1);
    }

    const QString modeColor = fault.isEmpty()
      ? (mode == QStringLiteral("idle") || mode == QStringLiteral("hold")
           ? QStringLiteral("#52c41a")
           : QStringLiteral("#f4d03f"))
      : QStringLiteral("#e74c3c");

    QString faultHtml;
    if (!fault.isEmpty()) {
      faultHtml = QStringLiteral(
        "<tr><td style='color:#888;'>fault</td>"
        "<td style='color:#e74c3c;'>%1</td></tr>")
        .arg(fault.toHtmlEscaped());
    }

    const QString html = QStringLiteral(
      "<table width='100%' cellpadding='6' cellspacing='0'>"
      "<tr><td colspan='2' style='font-size:1.15em;color:#fbe283;font-weight:bold;'>"
      "%1 — bridge state"
      "</td></tr>"
      "<tr><td style='color:#888;width:140px;'>robot</td>"
      "<td style='color:#d8d8d8;'>%2 <span style='color:#606060;'>(id: %3)</span></td></tr>"
      "<tr><td style='color:#888;'>mode</td>"
      "<td style='color:%4;'>%5</td></tr>"
      "%6%7%8%9"
      "<tr><td style='color:#888;'>sim time</td>"
      "<td style='color:#d8d8d8;'>%10 s</td></tr>"
      "<tr><td colspan='2' style='color:#606060;font-size:0.85em;padding-top:14px;'>"
      "Endpoint: %11<br>Override via OMNI_AGENT_HUD_URL."
      "</td></tr>"
      "</table>")
      .arg(model)
      .arg(model)
      .arg(id)
      .arg(modeColor)
      .arg(mode)
      .arg(poseHtml)
      .arg(jointsHtml)
      .arg(tcpHtml)
      .arg(faultHtml)
      .arg(simTime, 0, 'f', 2)
      .arg(mUrl);
    mView->setHtml(html);
    return;
  }

  const QString agent = safeStr(root.value("agent"), QStringLiteral("Agent"));
  const int toolsRegistered = root.value("tools_registered").toInt();
  const int activitySize = root.value("activity_log_size").toInt();

  QStringList specialists;
  if (root.value("specialists_known").isArray()) {
    const QJsonArray a = root.value("specialists_known").toArray();
    for (const QJsonValue &v : a)
      specialists << safeStr(v);
  }

  // Pull the live mission counters that orchestrators expose. Names
  // differ by agent (foreman / picker / captain); show whichever is
  // present.
  qlonglong completes = 0;
  for (const QString &k : {QStringLiteral("foreman_complete_calls_this_session"),
                           QStringLiteral("complete_calls_this_session"),
                           QStringLiteral("captain_complete_calls_this_session")}) {
    if (root.contains(k)) {
      completes = root.value(k).toVariant().toLongLong();
      break;
    }
  }

  // Last action: short summary of the most recent tool call.
  QString lastTool, lastDetail, lastTs, lastKind;
  if (root.value("last_action").isObject()) {
    const QJsonObject la = root.value("last_action").toObject();
    lastTool = safeStr(la.value("tool"));
    lastDetail = safeStr(la.value("detail"));
    lastTs = safeStr(la.value("timestamp"));
    lastKind = safeStr(la.value("kind"), QStringLiteral("info"));
  }

  // Usage block — the headline numbers.
  qlonglong inputUnits = 0, outputUnits = 0, totalUnits = 0;
  qlonglong cachedInputUnits = 0, freshInputUnits = 0;
  double cacheHitRatio = 0.0;
  qlonglong tokensPerHour = 0;
  double creditsPerHour = 0.0;
  double credits = 0.0;
  double elapsedS = 0.0;
  bool usageAvailable = false;
  if (root.value("usage").isObject()) {
    const QJsonObject u = root.value("usage").toObject();
    usageAvailable = u.value("available").toBool(true);
    inputUnits = u.value("input_units").toVariant().toLongLong();
    outputUnits = u.value("output_units").toVariant().toLongLong();
    totalUnits = u.value("total_units").toVariant().toLongLong();
    cachedInputUnits = u.value("cached_input_units").toVariant().toLongLong();
    freshInputUnits = u.value("fresh_input_units").toVariant().toLongLong();
    cacheHitRatio = u.value("cache_hit_ratio").toDouble();
    tokensPerHour = u.value("tokens_per_hour").toVariant().toLongLong();
    creditsPerHour = u.value("credits_per_hour").toDouble();
    credits = u.value("credits").toDouble();
    elapsedS = u.value("elapsed_s").toDouble();
  }

  // Colour cues for the action kind: success=green, warning=yellow,
  // error/fault=red, anything else neutral grey.
  QString kindColor = QStringLiteral("#a0a0a0");
  if (lastKind == QStringLiteral("success"))
    kindColor = QStringLiteral("#52c41a");
  else if (lastKind == QStringLiteral("warning"))
    kindColor = QStringLiteral("#f4d03f");
  else if (lastKind == QStringLiteral("error") || lastKind == QStringLiteral("fault"))
    kindColor = QStringLiteral("#e74c3c");

  // Cache hit colour: green if we're seeing meaningful savings, grey
  // otherwise. The threshold (40%) is arbitrary but matches the point
  // at which dollars start to actually move.
  const QString cacheColor = (cacheHitRatio >= 0.40)
    ? QStringLiteral("#52c41a")
    : (cacheHitRatio > 0.0 ? QStringLiteral("#d8d8d8") : QStringLiteral("#606060"));

  QString specialistsHtml;
  if (!specialists.isEmpty()) {
    specialistsHtml = QStringLiteral(
      "<tr><td style='color:#888;width:140px;'>specialists</td>"
      "<td style='color:#d8d8d8;'>%1</td></tr>")
      .arg(specialists.join(QStringLiteral(", ")));
  }

  QString cacheRow;
  if (usageAvailable && inputUnits > 0) {
    cacheRow = QStringLiteral(
      "<tr><td style='color:#888;'>cache hit</td>"
      "<td style='color:%1;'>%2%  &nbsp; (%3 cached / %4 input)</td></tr>")
      .arg(cacheColor)
      .arg(fmtDouble(cacheHitRatio * 100.0, 0))
      .arg(formatBigInt(cachedInputUnits))
      .arg(formatBigInt(inputUnits));
  }

  // Dollars per hour at Gemini 3 Flash list price. Effective rate
  // accounts for the 75% cache discount on cached input. We don't try
  // to discount per-engine because the runner doesn't surface engine
  // here — Gemini list is the most common case.
  double grossDollarsPerHour = 0.0;
  double effectiveDollarsPerHour = 0.0;
  if (elapsedS > 0) {
    const double inputPerHour = inputUnits * 3600.0 / elapsedS;
    const double outputPerHour = outputUnits * 3600.0 / elapsedS;
    const double cachedPerHour = cachedInputUnits * 3600.0 / elapsedS;
    grossDollarsPerHour = inputPerHour * 0.50 / 1.0e6 + outputPerHour * 3.00 / 1.0e6;
    // Cached input is billed at ~25% of fresh-input price (75% off).
    const double effectiveInputCostPerHour =
      (inputPerHour - cachedPerHour) * 0.50 / 1.0e6 + cachedPerHour * 0.125 / 1.0e6;
    effectiveDollarsPerHour = effectiveInputCostPerHour + outputPerHour * 3.00 / 1.0e6;
  }

  QString html;
  html += QStringLiteral(
    "<table width='100%' cellpadding='4' cellspacing='0'>"
    "<tr><td colspan='2' style='font-size:1.15em;color:#f4d03f;font-weight:bold;padding-bottom:6px;'>"
    "%1"
    "</td></tr>"
    "<tr><td style='color:#888;width:140px;'>tools registered</td><td style='color:#d8d8d8;'>%2</td></tr>"
    "%3"
    "<tr><td style='color:#888;'>complete_mission</td><td style='color:#d8d8d8;'>%4</td></tr>"
    "<tr><td style='color:#888;'>activity entries</td><td style='color:#d8d8d8;'>%5</td></tr>"
    "<tr><td colspan='2' style='padding-top:10px;color:#888;font-weight:bold;'>last action</td></tr>"
    "<tr><td style='color:#888;'>tool</td><td style='color:%6;'>%7</td></tr>"
    "<tr><td style='color:#888;vertical-align:top;'>detail</td><td style='color:#d8d8d8;'>%8</td></tr>"
    "<tr><td style='color:#888;'>at</td><td style='color:#606060;'>%9</td></tr>")
    .arg(agent)
    .arg(toolsRegistered)
    .arg(specialistsHtml)
    .arg(formatBigInt(completes))
    .arg(activitySize)
    .arg(kindColor)
    .arg(lastTool.isEmpty() ? QStringLiteral("(none)") : lastTool)
    .arg(lastDetail.isEmpty() ? QStringLiteral("(none)") : lastDetail.toHtmlEscaped())
    .arg(lastTs.isEmpty() ? QStringLiteral("--") : lastTs);

  if (usageAvailable) {
    html += QStringLiteral(
      "<tr><td colspan='2' style='padding-top:10px;color:#888;font-weight:bold;'>token usage</td></tr>"
      "<tr><td style='color:#888;'>input</td><td style='color:#d8d8d8;'>%1</td></tr>"
      "<tr><td style='color:#888;'>output</td><td style='color:#d8d8d8;'>%2</td></tr>"
      "<tr><td style='color:#888;'>total</td><td style='color:#d8d8d8;'>%3</td></tr>"
      "%4"
      "<tr><td style='color:#888;'>tokens/hr</td><td style='color:#d8d8d8;'>%5</td></tr>"
      "<tr><td colspan='2' style='padding-top:10px;color:#888;font-weight:bold;'>cost (Gemini 3 Flash list)</td></tr>"
      "<tr><td style='color:#888;'>$/hr gross</td><td style='color:#d8d8d8;'>$%6</td></tr>"
      "<tr><td style='color:#888;'>$/hr w/cache</td><td style='color:%7;'>$%8</td></tr>"
      "<tr><td style='color:#888;'>credits</td><td style='color:#d8d8d8;'>%9</td></tr>"
      "<tr><td style='color:#888;'>window</td><td style='color:#606060;'>%10s</td></tr>")
      .arg(formatBigInt(inputUnits))
      .arg(formatBigInt(outputUnits))
      .arg(formatBigInt(totalUnits))
      .arg(cacheRow)
      .arg(formatBigInt(tokensPerHour))
      .arg(fmtDouble(grossDollarsPerHour, 2))
      .arg(cacheColor)
      .arg(fmtDouble(effectiveDollarsPerHour, 2))
      .arg(fmtDouble(credits, 4))
      .arg(fmtDouble(elapsedS, 0));
  } else {
    html += QStringLiteral(
      "<tr><td colspan='2' style='padding-top:10px;color:#606060;'>"
      "Usage meter not initialised on this runner."
      "</td></tr>");
  }

  html += QStringLiteral(
    "<tr><td colspan='2' style='padding-top:14px;color:#404040;font-size:0.85em;'>"
    "polling %1 every %2 ms"
    "</td></tr></table>")
    .arg(mUrl).arg(mRefreshMs);

  mView->setHtml(html);
}
