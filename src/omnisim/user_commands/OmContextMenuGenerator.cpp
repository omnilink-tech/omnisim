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

#include "OmContextMenuGenerator.hpp"

#include "OmActionManager.hpp"
#include "OmNodeModel.hpp"
#include "OmNodeUtilities.hpp"
#include "OmRobot.hpp"
#include "OmViewpoint.hpp"
#include "OmWorldInfo.hpp"

#include <QtCore/QEvent>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QObject>
#include <QtCore/QSharedPointer>
#include <QtCore/QTimer>
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#include <QtGui/QFont>
#include <QtGui/QKeyEvent>
#include <QtNetwork/QAbstractSocket>
#include <QtNetwork/QNetworkProxy>
#include <QtNetwork/QTcpSocket>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMenu>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include <functional>

namespace {
  // Native in-simulator chat card. A frameless, draggable, on-brand panel
  // that talks to a robot's OmniLink bridge over HTTP (POST /prompt) and
  // shows the conversation inline -- no browser tab. Replies also surface
  // as the in-world speech bubble (the bridge draws that on /prompt). On
  // open it pulls /chat_config for the persona name + greeting. No Q_OBJECT:
  // all wiring uses functor connects + virtual event overrides, so the file
  // needs no moc pass.
  class RobotChatCard : public QWidget {
  public:
    RobotChatCard(const QString &baseUrl, QWidget *parent)
      // A normal top-level window OWNED by the main window: it gets a native
      // title bar (close X in the corner) and is movable, stays grouped with
      // and on top of OmniSim, and is destroyed with it -- NOT a stray
      // frameless orphan that floats off and can't be closed.
      : QWidget(parent, Qt::Window),
        mBase(baseUrl) {
      setAttribute(Qt::WA_DeleteOnClose);
      setWindowTitle(tr("Talk to the Robot"));
      resize(380, 480);

      QVBoxLayout *outer = new QVBoxLayout(this);
      outer->setContentsMargins(0, 0, 0, 0);
      mCard = new QFrame(this);
      mCard->setObjectName("card");
      outer->addWidget(mCard);

      QVBoxLayout *col = new QVBoxLayout(mCard);
      col->setContentsMargins(0, 0, 0, 0);
      col->setSpacing(0);

      QFrame *header = new QFrame(mCard);
      header->setObjectName("header");
      QHBoxLayout *hl = new QHBoxLayout(header);
      hl->setContentsMargins(14, 10, 14, 10);
      mTitle = new QLabel(tr("Robot"), header);
      mTitle->setObjectName("title");
      hl->addWidget(mTitle);
      hl->addStretch();
      col->addWidget(header);

      mScroll = new QScrollArea(mCard);
      mScroll->setObjectName("scroll");
      mScroll->setWidgetResizable(true);
      mScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      mScroll->setFrameShape(QFrame::NoFrame);
      mScroll->setFocusPolicy(Qt::NoFocus);
      QWidget *logHost = new QWidget();
      mLogHost = logHost;
      logHost->setObjectName("loghost");
      mLog = new QVBoxLayout(logHost);
      mLog->setContentsMargins(12, 12, 12, 12);
      mLog->setSpacing(8);
      mLog->addStretch();
      mScroll->setWidget(logHost);
      col->addWidget(mScroll, 1);
      // Clicking the transcript moves focus off the text box, so bare +/-
      // zoom the card (the input keeps +/- for typing, e.g. negatives).
      logHost->installEventFilter(this);
      mScroll->viewport()->installEventFilter(this);

      QFrame *composer = new QFrame(mCard);
      composer->setObjectName("composer");
      QHBoxLayout *cl = new QHBoxLayout(composer);
      cl->setContentsMargins(10, 10, 10, 10);
      cl->setSpacing(8);
      mInput = new QLineEdit(composer);
      mInput->setPlaceholderText(tr("Talk to the robot\xE2\x80\xA6"));  // …
      QPushButton *sendBtn = new QPushButton(tr("Send"), composer);
      mSend = sendBtn;
      sendBtn->setObjectName("send");
      sendBtn->setCursor(Qt::PointingHandCursor);
      cl->addWidget(mInput, 1);
      cl->addWidget(sendBtn);
      col->addWidget(composer);

      mCard->setStyleSheet(cardStyle());

      connect(sendBtn, &QPushButton::clicked, this, [this]() { sendPrompt(); });
      connect(mInput, &QLineEdit::returnPressed, this, [this]() { sendPrompt(); });

      // Font zoom (+/-): scale every bubble + the input by changing the
      // card's font. The stylesheets deliberately set NO font-size, so all
      // text inherits this font and rescales together.
      setFocusPolicy(Qt::StrongFocus);
      mBaseFont = font();
      mBaseFont.setPointSizeF(11.0);
      applyFontScale();

      fetchConfig();
    }

  protected:
    void keyPressEvent(QKeyEvent *e) override {
      const int k = e->key();
      if (k == Qt::Key_Plus || k == Qt::Key_Equal) {       // '+' (and the +/= key)
        bumpFont(1);
        e->accept();
        return;
      }
      if (k == Qt::Key_Minus || k == Qt::Key_Underscore) {  // '-'
        bumpFont(-1);
        e->accept();
        return;
      }
      if (k == Qt::Key_0 && (e->modifiers() & Qt::ControlModifier)) {  // Ctrl+0 resets
        mFontScale = 1.0;
        applyFontScale();
        e->accept();
        return;
      }
      QWidget::keyPressEvent(e);
    }

    bool eventFilter(QObject *obj, QEvent *ev) override {
      if (ev->type() == QEvent::MouseButtonPress)
        setFocus();  // take key focus so bare +/- zoom; don't consume the click
      return QWidget::eventFilter(obj, ev);
    }

  private:
    static QString cardStyle() {
      return
        // One uniform background for card + chat area (no contrasting panel
        // behind the bubbles). No font-size anywhere — text inherits the
        // card's scalable font (see applyFontScale).
        "#card,#scroll,#loghost{background:#15151a;}"
        // Each message row is a plain QWidget; the stylesheet style would
        // otherwise paint it a default grey. Force it transparent so only
        // the bubble shows (targeted #brow selector -> doesn't touch the bubble).
        "#brow{background:transparent;}"
        "#header{background:#1b1b21;border-bottom:1px solid #26262d;}"
        "#title{color:#fbe283;font-weight:600;}"
        "QScrollBar:vertical{background:transparent;width:9px;margin:3px;}"
        "QScrollBar::handle:vertical{background:#33333b;border-radius:4px;min-height:28px;}"
        "QScrollBar::handle:vertical:hover{background:#45454f;}"
        "QScrollBar::add-line,QScrollBar::sub-line{height:0;}"
        "#composer{background:#1b1b21;border-top:1px solid #26262d;}"
        "QLineEdit{background:#0e0e12;color:#f3eddc;border:1px solid #2a2a32;border-radius:10px;"
        "padding:9px 12px;}"
        "QLineEdit:focus{border-color:#d6a72d;}"
        "#send{background:#fbe283;color:#0b0b0c;border:none;border-radius:10px;padding:9px 16px;"
        "font-weight:600;}"
        "#send:hover{background:#ffe890;}";
    }

    void bumpFont(int dir) {
      mFontScale = qBound(0.65, mFontScale * (dir > 0 ? 1.1 : 1.0 / 1.1), 2.4);
      applyFontScale();
    }

    QFont scaledFont() const {
      QFont f = mBaseFont;
      f.setPointSizeF(qMax(6.0, mBaseFont.pointSizeF() * mFontScale));
      return f;
    }

    void applyFontScale() {
      const QFont f = scaledFont();
      // setFont() on a parent does NOT reach stylesheet-styled children in
      // Qt, so set the font on each bubble + the input/send explicitly.
      if (mLogHost) {
        const QList<QLabel *> labels = mLogHost->findChildren<QLabel *>();
        for (QLabel *lb : labels)
          lb->setFont(f);
      }
      if (mInput)
        mInput->setFont(f);
      if (mSend)
        mSend->setFont(f);
      if (mTitle) {
        QFont tf = f;
        tf.setPointSizeF(f.pointSizeF() + 2.0);
        tf.setBold(true);
        mTitle->setFont(tf);
      }
    }

    QWidget *addBubble(const QString &kind, const QString &text) {
      QWidget *rowW = new QWidget();
      rowW->setObjectName("brow");
      QHBoxLayout *row = new QHBoxLayout(rowW);
      row->setContentsMargins(0, 0, 0, 0);
      QLabel *b = new QLabel(text, rowW);
      b->setWordWrap(true);
      b->setTextInteractionFlags(Qt::TextSelectableByMouse);
      b->setMaximumWidth(300);
      // Clean, borderless bubbles on the uniform background — no box behind
      // them, no font-size (inherits the scalable card font).
      QString css;
      if (kind == "user")
        css = "background:#f6d873;color:#1a1206;border-radius:14px;padding:9px 13px;";
      else if (kind == "tool")
        css = "background:rgba(109,209,120,0.12);color:#88de92;border-radius:9px;"
              "padding:6px 11px;font-family:'Consolas','SF Mono',monospace;";
      else if (kind == "error")
        css = "background:#37191b;color:#ff9a93;border-radius:13px;padding:9px 13px;";
      else
        css = "background:#24242b;color:#f3eddc;border-radius:14px;padding:9px 13px;";
      b->setStyleSheet(css);
      b->setFont(scaledFont());  // after the stylesheet so the zoom size sticks
      if (kind == "user") {
        row->addStretch();
        row->addWidget(b);
      } else {
        row->addWidget(b);
        row->addStretch();
      }
      mLog->insertWidget(mLog->count() - 1, rowW);  // before the trailing stretch
      QScrollBar *sb = mScroll->verticalScrollBar();
      QTimer::singleShot(0, this, [sb]() { if (sb) sb->setValue(sb->maximum()); });
      return rowW;
    }

    // Minimal async HTTP/1.1 over a plain QTcpSocket (the bridge is local
    // plain-HTTP). Deliberately NOT QNetworkAccessManager: in this Qt5 +
    // GCC15 build QNAM pulls qsslcertificate.h / qregexp.h, whose
    // `class QStringList;` forward-decls clash with the QStringList alias.
    void httpRequest(const QByteArray &method, const QString &path, const QByteArray &body,
                     std::function<void(bool, const QByteArray &)> cb) {
      const QUrl u(mBase);
      const QString host = u.host().isEmpty() ? QStringLiteral("127.0.0.1") : u.host();
      const int port = u.port(8765);
      QTcpSocket *sock = new QTcpSocket(this);
      QTimer *timer = new QTimer(this);
      timer->setSingleShot(true);
      QSharedPointer<QByteArray> acc(new QByteArray());
      QSharedPointer<bool> done(new bool(false));

      std::function<void(bool)> finish = [this, sock, timer, acc, done, cb](bool ok) {
        if (*done)
          return;
        *done = true;
        timer->stop();
        QByteArray respBody;
        if (ok) {
          const int idx = acc->indexOf("\r\n\r\n");
          respBody = (idx >= 0) ? acc->mid(idx + 4) : QByteArray();
        }
        timer->deleteLater();
        sock->deleteLater();
        cb(ok, respBody);
      };

      connect(sock, &QTcpSocket::connected, sock, [sock, method, path, body, host, port]() {
        QByteArray req = method + " " + path.toUtf8() + " HTTP/1.1\r\n";
        req += "Host: " + host.toUtf8() + ":" + QByteArray::number(port) + "\r\n";
        req += "Content-Type: application/json\r\n";
        req += "Content-Length: " + QByteArray::number(body.size()) + "\r\n";
        req += "Connection: close\r\n\r\n";
        req += body;
        sock->write(req);
      });
      connect(sock, &QTcpSocket::readyRead, sock, [sock, acc]() { acc->append(sock->readAll()); });
      connect(sock, &QTcpSocket::disconnected, this, [finish]() { finish(true); });
      connect(sock, &QAbstractSocket::errorOccurred, this,
              [acc, finish](QAbstractSocket::SocketError err) {
                if (err == QAbstractSocket::RemoteHostClosedError)
                  return;  // expected close after "Connection: close"; disconnected delivers the body
                finish(!acc->isEmpty());
              });
      connect(timer, &QTimer::timeout, this, [finish]() { finish(false); });
      // Bypass OmniSim's app-wide system proxy (OmNetwork::setProxy via
      // QNetworkProxyFactory::systemProxyForQuery): a configured/injected
      // proxy can't route to 127.0.0.1, which made the bridge "unreachable".
      sock->setProxy(QNetworkProxy::NoProxy);
      timer->start(125000);
      sock->connectToHost(host, port);
    }

    void fetchConfig() {
      httpRequest("GET", "/chat_config", QByteArray(), [this](bool ok, const QByteArray &body) {
        if (!ok) {
          mTitle->setText(tr("Robot Chat"));
          addBubble("error", tr("Can't reach this robot's chat bridge \xE2\x80\x94 is its controller running?"));
          return;
        }
        const QJsonObject c = QJsonDocument::fromJson(body).object();
        QString name = c.value("display_name").toString();
        if (name.isEmpty())
          name = c.value("robot").toString();
        const QString shown = name.isEmpty() ? tr("Robot Chat") : name;
        mTitle->setText(shown);
        setWindowTitle(tr("Talk to %1").arg(shown));
        const QString greeting = c.value("greeting").toString();
        if (!greeting.isEmpty())
          addBubble("agent", greeting);
      });
    }

    void sendPrompt() {
      const QString text = mInput->text().trimmed();
      if (text.isEmpty() || mBusy)
        return;
      addBubble("user", text);
      mInput->clear();
      mBusy = true;
      mThinkingRow = addBubble("agent", QString::fromUtf8("\xE2\x80\xA6"));  // …
      QJsonObject bodyObj;
      bodyObj["text"] = text;
      const QByteArray body = QJsonDocument(bodyObj).toJson(QJsonDocument::Compact);
      httpRequest("POST", "/prompt", body, [this](bool ok, const QByteArray &respBody) {
        mBusy = false;
        if (mThinkingRow) {
          mThinkingRow->deleteLater();
          mThinkingRow = NULL;
        }
        if (!ok) {
          addBubble("error", tr("network error \xE2\x80\x94 the robot didn't respond."));
          return;
        }
        const QJsonObject obj = QJsonDocument::fromJson(respBody).object();
        const QJsonArray actions = obj.value("actions").toArray();
        for (int i = 0; i < actions.size(); ++i) {
          const QJsonObject a = actions.at(i).toObject();
          const QString tool = a.value("tool").toString();
          const QString res = a.value("result").toString();
          const QString summary = a.value("summary").toString();
          addBubble(res == "ok" ? "tool" : "error",
                    QString::fromUtf8("\xE2\x86\x92 ") + tool + (summary.isEmpty() ? QString() : (QString::fromUtf8(" \xC2\xB7 ") + summary)));
        }
        const QString err = obj.value("error").toString();
        if (!err.isEmpty())
          addBubble("error", err);
        const QString resp = obj.value("response").toString();
        addBubble("agent", resp.isEmpty() ? tr("(no reply)") : resp);
      });
    }

    // CUT THE PENDING REPLIES LOOSE BEFORE OUR WIDGETS DIE.
    //
    // ~QWidget deletes child widgets BEFORE ~QObject drops incoming
    // connections, and a connected QTcpSocket emits disconnected() /
    // errorOccurred() from its OWN destructor. Those are connected with
    // `this` as context, so with the card half-destroyed the reply lambda
    // still runs -- against an mLog and mScroll that have already been
    // freed. Measured crash: closing the card with a request in flight
    // (replies take 7-20 s, so the window is wide) produced
    //     QBoxLayout::insert: index 1674138815 out of range (max: 0)
    // followed by 0xC0000005. The two numbers are the signature of a dead
    // object: count() is VIRTUAL so it dispatched through a stale vptr and
    // returned garbage, while insertWidget() is non-virtual, entered real
    // QBoxLayout code, and read 0 from the freed d_ptr.
    //
    // A QPointer guard does NOT fix this -- QPointer is cleared in
    // ~QObject, which is also too late. Severing the signals here, before
    // any child is freed, is what makes it safe.
    ~RobotChatCard() override {
      for (QTcpSocket *s : findChildren<QTcpSocket *>())
        s->disconnect();
      for (QTimer *t : findChildren<QTimer *>())
        t->disconnect();
    }

    QString mBase;
    QFrame *mCard;
    QLabel *mTitle;
    QScrollArea *mScroll;
    QWidget *mLogHost = NULL;
    QVBoxLayout *mLog;
    QLineEdit *mInput;
    QPushButton *mSend = NULL;
    QWidget *mThinkingRow = NULL;
    bool mBusy = false;
    QFont mBaseFont;
    double mFontScale = 1.0;
  };
}  // namespace

namespace OmContextMenuGenerator {
  static bool gAreNodeActionsEnabled = false;
  static bool gAreRobotActionsEnabled = false;
  static bool gAreProtoActionsEnabled = false;
  static bool gAreExternProtoActionsEnabled = false;
  static QMenu *gOverlaysMenu = NULL;

  void enableNodeActions(bool enabled) {
    gAreNodeActionsEnabled = enabled;
  }
  void enableRobotActions(bool enabled) {
    gAreRobotActionsEnabled = enabled;
  }
  void enableProtoActions(bool enabled) {
    gAreProtoActionsEnabled = enabled;
  }
  void enableExternProtoActions(bool enabled) {
    gAreExternProtoActionsEnabled = enabled;
  }
  void setOverlaysMenu(QMenu *menu) {
    gOverlaysMenu = menu;
  }

  const QStringList fillTransformToItems(const OmNode *selectedNode) {
    // populate transform combo box
    QStringList suitableModels;

    if (selectedNode && !selectedNode->isUseNode() && (selectedNode->useCount() == 0) && !selectedNode->isProtoInstance() &&
        (dynamic_cast<const OmGroup *>(selectedNode))) {
      // find all basic nodes
      QStringList basicModels = OmNodeModel::baseModelNames();

      // cache intensive searches results
      int hasDeviceChildren = -1;
      // find all nodes suitable for transform
      foreach (const QString &modelName, basicModels) {
        const OmNodeUtilities::Answer answer =
          OmNodeUtilities::isSuitableForTransform(selectedNode, modelName, &hasDeviceChildren);
        if (answer != OmNodeUtilities::UNSUITABLE)
          suitableModels << modelName;
      }
    }
    return suitableModels;
  }

  void generateContextMenu(const QPoint &position, const OmNode *selectedNode, QWidget *parentWidget) {
    QMenu *contextMenu = new QMenu(parentWidget);
    contextMenu->setObjectName("ContextMenu");
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::CUT));
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::COPY));
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::PASTE));
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::RESET_VALUE));
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::EDIT_FIELD));
    contextMenu->addSeparator();
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::ADD_NEW));
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::DEL));
    contextMenu->addSeparator();
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::MOVE_VIEWPOINT_TO_OBJECT));
    QMenu *viewMenu = contextMenu->addMenu(QObject::tr("Ali&gn View to Object"));
    viewMenu->addAction(OmActionManager::instance()->action(OmAction::OBJECT_FRONT_VIEW));
    viewMenu->addAction(OmActionManager::instance()->action(OmAction::OBJECT_BACK_VIEW));
    viewMenu->addAction(OmActionManager::instance()->action(OmAction::OBJECT_LEFT_VIEW));
    viewMenu->addAction(OmActionManager::instance()->action(OmAction::OBJECT_RIGHT_VIEW));
    viewMenu->addAction(OmActionManager::instance()->action(OmAction::OBJECT_TOP_VIEW));
    viewMenu->addAction(OmActionManager::instance()->action(OmAction::OBJECT_BOTTOM_VIEW));
    contextMenu->addSeparator();

    // selection-dependent actions
    if (selectedNode) {
      // actions for robots
      if (gAreRobotActionsEnabled) {
        contextMenu->addAction(OmActionManager::instance()->action(OmAction::SHOW_ROBOT_WINDOW));

        // OmniLink: "Talk to the Robot" -> open this robot's full-page chat
        // in the operator's web browser. Shown only for robots whose
        // controller is an OmniLink chat bridge. The chat is served by the
        // bridge at 127.0.0.1:<port>/chat, where <port> is the controller's
        // --port arg (default 8765) -- so each robot opens its own tab
        // driving its own agent.
        const OmRobot *talkRobot = dynamic_cast<const OmRobot *>(selectedNode);
        if (talkRobot) {
          const QString ctrl = talkRobot->controllerName();
          if (ctrl.contains("bridge") || ctrl.contains("omnilink")) {
            int chatPort = 8765;
            const QStringList cArgs = talkRobot->controllerArgs();
            for (int i = 0; i + 1 < cArgs.size(); ++i) {
              if (cArgs.at(i) == "--port") {
                chatPort = cArgs.at(i + 1).toInt();
                break;
              }
            }
            const QString baseUrl = QString("http://127.0.0.1:%1").arg(chatPort);
            QAction *talkAction = new QAction(QObject::tr("Talk to the Robot"), contextMenu);
            talkAction->setStatusTip(QObject::tr("Open an in-simulator chat with this robot."));
            QObject::connect(talkAction, &QAction::triggered, contextMenu, [baseUrl, parentWidget]() {
              QWidget *mainWin = parentWidget ? parentWidget->window() : NULL;
              RobotChatCard *card = new RobotChatCard(baseUrl, mainWin);
              if (mainWin) {  // place it inside the OmniSim window, upper-right
                const QRect g = mainWin->geometry();
                const int x = g.right() - card->width() - 32;
                const int y = g.top() + 90;
                card->move(qMax(g.left() + 12, x), qMax(g.top() + 12, y));
              }
              card->show();
              card->raise();
              card->activateWindow();
            });
            contextMenu->addAction(talkAction);
          }
        }

        assert(gOverlaysMenu);
        QMenu *subMenu = contextMenu->addMenu(QObject::tr("Overlays"));
        subMenu->setEnabled(false);
        QListIterator<QAction *> actionIt(gOverlaysMenu->actions());
        while (actionIt.hasNext()) {
          const QAction *action = actionIt.next();
          const QMenu *robotMenu = action->menu();
          if (robotMenu && robotMenu->property("robot").value<void *>() == selectedNode) {
            if (!robotMenu->isEnabled())
              break;
            assert(!robotMenu->actions().isEmpty());
            QListIterator<QAction *> menuIt(robotMenu->actions());
            bool enabled = true;
            while (menuIt.hasNext()) {
              QMenu *deviceMenu = menuIt.next()->menu();
              enabled = enabled || deviceMenu->isEnabled();
              subMenu->addMenu(deviceMenu);
            }
            subMenu->setEnabled(enabled);
          }
        }

        contextMenu->addSeparator();
      }

      // actions for nodes in general
      if (gAreNodeActionsEnabled) {
        QMenu *subMenu = contextMenu->addMenu(QObject::tr("Follow Object"));
        subMenu->addAction(OmActionManager::instance()->action(OmAction::FOLLOW_NONE));
        subMenu->addAction(OmActionManager::instance()->action(OmAction::FOLLOW_TRACKING));
        subMenu->addAction(OmActionManager::instance()->action(OmAction::FOLLOW_MOUNTED));
        subMenu->addAction(OmActionManager::instance()->action(OmAction::FOLLOW_PAN_AND_TILT));

        subMenu = contextMenu->addMenu(QObject::tr("Optional Rendering"));
        subMenu->addAction(OmActionManager::instance()->action(OmAction::CENTER_OF_MASS));
        subMenu->addAction(OmActionManager::instance()->action(OmAction::CENTER_OF_BUOYANCY));
        subMenu->addAction(OmActionManager::instance()->action(OmAction::SUPPORT_POLYGON));

        contextMenu->addSeparator();

        const OmBaseNode *selectedBaseNode = static_cast<const OmBaseNode *>(selectedNode);
        if (selectedBaseNode->nodeType() == WB_NODE_ROBOT)
          contextMenu->addAction(OmActionManager::instance()->action(OmAction::EXPORT_URDF));

        if (!gAreProtoActionsEnabled) {
          subMenu = contextMenu->addMenu(QObject::tr("Transform To..."));
          const QStringList suitableTransformToModels = fillTransformToItems(selectedNode);

          if (!suitableTransformToModels.isEmpty()) {
            foreach (const QString &model, suitableTransformToModels) {
              const QAction *action = subMenu->addAction(model);
              QObject::connect(action, &QAction::triggered, OmActionManager::instance(),
                               &OmActionManager::forwardTransformToActionToSceneTree);
            }
          } else
            subMenu->setEnabled(false);
        }
      }

      // actions for PROTO nodes
      if (gAreProtoActionsEnabled) {
        QAction *editProtoAction(OmActionManager::instance()->action(OmAction::EDIT_PROTO_SOURCE));
        contextMenu->addAction(editProtoAction);
        if (gAreExternProtoActionsEnabled) {
          editProtoAction->setStatusTip(QObject::tr("Copy and edit the PROTO file in Text Editor."));
          contextMenu->addAction(OmActionManager::instance()->action(OmAction::SHOW_PROTO_SOURCE));
        } else
          editProtoAction->setStatusTip(QObject::tr("Edit the PROTO file in Text Editor."));
        editProtoAction->setToolTip(editProtoAction->statusTip());

        if (selectedNode->isTemplate())
          contextMenu->addAction(OmActionManager::instance()->action(OmAction::SHOW_PROTO_RESULT));

        contextMenu->addAction(OmActionManager::instance()->action(OmAction::CONVERT_TO_BASE_NODES));
        contextMenu->addAction(OmActionManager::instance()->action(OmAction::CONVERT_ROOT_TO_BASE_NODES));
      }
      contextMenu->addSeparator();
    }
    contextMenu->addAction(OmActionManager::instance()->action(OmAction::OPEN_HELP));

    QObject *focusObject = OmActionManager::instance()->focusObject();
    OmActionManager::instance()->setFocusObject(contextMenu);
    contextMenu->exec(position);
    OmActionManager::instance()->setFocusObject(focusObject);
  }
}  // namespace OmContextMenuGenerator
