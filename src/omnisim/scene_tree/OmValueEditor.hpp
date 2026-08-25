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

#ifndef OM_VALUE_EDITOR_HPP
#define OM_VALUE_EDITOR_HPP

//
// Description: Abstract base class for all value editors of the Scene Tree
//
// Inherited by:
//    OmBoolEditor, OmColorEditor, OmDoubleEditor, OmIntEditor, OmRotationEditor,
//    OmStringEditor, OmVector2Editor, OmVector3Editor
//

#include <QtWidgets/QWidget>

class QComboBox;
class QGridLayout;

class OmNode;
class OmField;
class OmValue;
class OmSingleValue;
class OmMultipleValue;
class OmVariant;

class OmValueEditor : public QWidget {
  Q_OBJECT

public:
  virtual ~OmValueEditor();

  // start editing this value: this editor is shown
  virtual void edit(OmNode *node, OmField *field, int index);

  // copy from OmValue to editor widgets
  virtual void edit(bool copyOriginalValue) = 0;

  // stop editing: this editor is no longer visible
  virtual void stopEditing();

  // remove focus from children widgets
  virtual void resetFocus() = 0;

  // block signals of the widget and children
  virtual void recursiveBlockSignals(bool block) = 0;

  // set keyboard focus to first spinbox or line edit
  virtual void takeKeyboardFocus() = 0;

  virtual QWidget *lastEditorWidget() = 0;

signals:
  // the value was invalidated, e.g. deleted
  void valueInvalidated();
  void valueChanged();

public slots:
  virtual void cleanValue();
  // copy from editor widgets to OmValue if editor value has changed or world is being to be saved
  virtual void applyIfNeeded() { apply(); }

protected:
  explicit OmValueEditor(QWidget *parent = NULL);

  OmNode *node() const { return mNode; }
  OmField *field() const { return mField; }

  // return as OmSingleValue if the value is single
  OmSingleValue *singleValue() const { return mSingleValue; }

  // return as OmMultipleValue if the value is multiple
  OmMultipleValue *multipleValue() const { return mMultipleValue; }

  // return the item index in case the value is multiple
  int index() const { return mIndex; }

  OmVariant *mNewValue;
  OmVariant *mPreviousValue;
  QComboBox *mComboBox;
  QGridLayout *mLayout;

protected slots:
  virtual void apply();

private slots:
  void updateComboBoxIndex();

private:
  OmNode *mNode;
  OmField *mField;
  OmValue *mValue;
  OmSingleValue *mSingleValue;
  OmMultipleValue *mMultipleValue;
  int mIndex;
};

#endif
