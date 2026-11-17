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

#ifndef OM_FIELD_EDITOR_HPP
#define OM_FIELD_EDITOR_HPP

//
// Description: editor for editing a OmField at the bottom of the Scene Tree
//

#include <QtCore/QMultiMap>
#include <QtWidgets/QWidget>

#include "../../../include/controller/c/omnisim/supervisor.h"

class OmExternProtoEditor;
class OmField;
class OmNode;
class OmValueEditor;

class QLabel;
class QStackedLayout;

class OmFieldEditor : public QWidget {
  Q_OBJECT

public:
  explicit OmFieldEditor(QWidget *parent = NULL);
  virtual ~OmFieldEditor();

  // start editing this field
  void editField(OmNode *node, OmField *field, int item = -1);

  // start editing the EXTERNPROTO panel
  void editExternProto();

  // update displayed values
  void updateValue(bool copyOriginalValue = true);

  // remove focus from children widgets
  void resetFocus();

  // apply changes to the field value
  void applyChanges();

  void setTitle(const QString &title);

  QWidget *lastEditorWidget();

  OmValueEditor *currentEditor() const;

signals:
  // emitted when the file has to be opened in text editor
  // title can be used for example for showing human-readable file name in case of cached assets
  void editRequested(const QString &fileName);
  // emitted when the dictionary needs to be updated (e.g., a DEF name was changed)
  void dictionaryUpdateRequested();
  void valueChanged();

protected:
  OmNode *mNode;

private:
  QMultiMap<WbFieldType, OmValueEditor *> mEditors;
  OmExternProtoEditor *mExternProtoEditor;
  QStackedLayout *mStackedLayout;
  QWidget *mEmptyPane;
  OmField *mField;
  int mItem;
  OmNode *mNodeItem;
  bool mIsValidItemIndex;

  QLabel *mTitleLabel;

  QString nodeAsTitle(OmNode *node);
  void updateTitle();
  void setCurrentWidget(int index);
  void setCurrentWidget(OmValueEditor *editor);
  void setTransformActionVisibile(bool visible);
  void computeFieldInformation();

private slots:
  void invalidateValue();
  void updateResetButton();
  void refreshExternProtoEditor();
};

#endif
