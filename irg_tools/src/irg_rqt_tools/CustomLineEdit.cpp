#include "CustomLineEdit.h"

CustomLineEdit::CustomLineEdit(QWidget *parent)
 : QLineEdit(parent)
{
}

void CustomLineEdit::focusInEvent(QFocusEvent *e)
{
  QLineEdit::focusInEvent(e);
  emit(focussed(true));
}

void CustomLineEdit::focusOutEvent(QFocusEvent *e)
{
  QLineEdit::focusOutEvent(e);
  emit(focussed(false));
}
